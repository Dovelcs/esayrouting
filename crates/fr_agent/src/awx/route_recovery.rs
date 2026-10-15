//! Per-net restore and partial-result semantics for AWX route transactions.
//!
//! This module deliberately delegates copper restoration and grading to
//! `native_execution`; it only makes the route_bus recovery decision explicit.
use super::native_execution::{self, NativeGrade};
use crate::board_adapter::BoardAdapter;
use std::{cmp::Ordering, collections::BTreeSet};

/// Why copper was restored while recovering a graded attempt.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum RecoveryRejection {
    /// Candidate copper introduced violations; attributed bus nets were restored
    /// individually, with whole-group rollback when attribution failed.
    NewViolations { count: usize, nets: Vec<String> },
    /// Only these still-open nets were restored; connected bus nets remain.
    OpenNets(Vec<String>),
    /// Geometry is clean and connected but did not meet the requested proof policy.
    ProofRequired,
}

/// Ordered exactly like the current fanout driver: violations, open nets,
/// missing proof, then vias. A lower key is preferred.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RecoveryRank {
    pub new_violations: usize,
    pub open_nets: usize,
    pub unproved: usize,
    pub vias: usize,
}

impl Ord for RecoveryRank {
    fn cmp(&self, other: &Self) -> Ordering {
        (
            self.new_violations,
            self.open_nets,
            self.unproved,
            self.vias,
        )
            .cmp(&(
                other.new_violations,
                other.open_nets,
                other.unproved,
                other.vias,
            ))
    }
}

impl PartialOrd for RecoveryRank {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Candidate after applying route_bus's per-net rollback and grading again.
#[derive(Clone)]
pub struct RecoveredCandidate {
    pub board: BoardAdapter,
    pub grade: NativeGrade,
    pub restored_nets: BTreeSet<String>,
    pub rejection: Option<RecoveryRejection>,
    pub rank: RecoveryRank,
    pub complete: bool,
}

impl RecoveredCandidate {
    /// A rejected partial result is still a usable board result; this reports
    /// whether it satisfies the full completion contract.
    pub fn is_complete(&self) -> bool {
        self.complete
    }
}

/// Restore violating/open nets individually and re-grade, matching route_bus's
/// bounded grade recovery. Unattributable or still-failing attempts restore the
/// whole requested bus to the incoming board.
pub fn recover_candidate(
    before: &BoardAdapter,
    mut candidate: BoardAdapter,
    requested: &BTreeSet<String>,
    proved: bool,
    allow_unproved: bool,
) -> Result<RecoveredCandidate, String> {
    let mut grade = native_execution::grade(before, &candidate, requested)?;
    let mut restored_nets = BTreeSet::new();
    let mut rejection = None;
    // route_bus retries the named-net rollback and grade up to three times.
    // When no failing bus net can be named, or every requested net is bad, its
    // final exit restores the whole bus to the incoming board.
    for pass in 0..3 {
        if grade.new_violations == 0 && grade.open.is_empty() {
            break;
        }
        let violating = if grade.new_violations > 0 {
            new_violation_nets(before, &candidate)?
        } else {
            BTreeSet::new()
        };
        let mut bad: BTreeSet<_> = grade.open.iter().cloned().collect();
        bad.extend(violating.iter().cloned());
        bad.retain(|net| requested.contains(net));
        if rejection.is_none() {
            rejection = if grade.new_violations > 0 {
                Some(RecoveryRejection::NewViolations {
                    count: grade.new_violations,
                    nets: violating
                        .iter()
                        .filter(|net| requested.contains(*net))
                        .cloned()
                        .collect(),
                })
            } else if !grade.open.is_empty() {
                Some(RecoveryRejection::OpenNets(grade.open.clone()))
            } else {
                None
            };
        }

        if bad.is_empty() || bad.len() == requested.len() || pass == 2 {
            native_execution::restore_nets(before, &mut candidate, requested)?;
            restored_nets.extend(requested.iter().cloned());
            grade = native_execution::grade(before, &candidate, requested)?;
            break;
        }
        native_execution::restore_nets(before, &mut candidate, &bad)?;
        restored_nets.extend(bad);
        grade = native_execution::grade(before, &candidate, requested)?;
    }

    // Preserve why the original candidate was rolled back even when restoring
    // it makes the final board clean. Completion is separately derived from the
    // final grade and proof policy by `is_complete`.
    if rejection.is_none() && !proved && !allow_unproved {
        rejection = Some(RecoveryRejection::ProofRequired);
    }
    let complete = grade.new_violations == 0 && grade.open.is_empty() && (proved || allow_unproved);
    let rank = RecoveryRank {
        new_violations: usize::from(grade.new_violations > 0),
        open_nets: grade.open.len(),
        unproved: usize::from(!proved),
        vias: grade.vias,
    };
    Ok(RecoveredCandidate {
        board: candidate,
        grade,
        restored_nets,
        rejection,
        rank,
        complete,
    })
}

/// Attribute newly introduced native clearance violations to their item nets.
/// If an item/net cannot be identified, callers see an empty attribution and
/// take route_bus's conservative whole-group rollback path.
fn new_violation_nets(
    before: &BoardAdapter,
    after: &BoardAdapter,
) -> Result<BTreeSet<String>, String> {
    let current = after.new_violations_since(before);
    let mut item_nets = std::collections::BTreeMap::<i32, BTreeSet<String>>::new();
    for key in after.board.get_items() {
        let item = after.board.item(key);
        let names = item_nets.entry(item.id().0).or_default();
        for &number in item.net_numbers() {
            if let Some(net) = after.board.rules.nets.get(number) {
                names.insert(net.name.clone());
            }
        }
    }
    let mut nets = BTreeSet::new();
    for (first, second, _, _, _) in &current {
        if let Some(names) = item_nets.get(first) {
            nets.extend(names.iter().cloned());
        }
        if let Some(names) = item_nets.get(second) {
            nets.extend(names.iter().cloned());
        }
    }
    Ok(nets)
}

/// Explicitly restore an entire failed group transaction to its incoming
/// board, preserving earlier successful groups, and grade that rollback.
pub fn restore_failed_group(
    original: &BoardAdapter,
    requested: &BTreeSet<String>,
) -> Result<(BoardAdapter, NativeGrade), String> {
    let mut restored = original.clone();
    native_execution::restore_nets(original, &mut restored, requested)?;
    let grade = native_execution::grade(original, &restored, requested)?;
    Ok((restored, grade))
}
