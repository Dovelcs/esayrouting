//! Serialization and transactional native realization for the AWX source stage.
use super::escape_moves::{Move, MoveKind};
use super::{
    destination_realize::{self, AskedBerth},
    joint_escape::JointPlan,
    native_execution::{self, NativeGrade},
    whole_context::RoutingRules,
};
use crate::board_adapter::BoardAdapter;
use serde::Serialize;
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};

/// A physical source-piece decision. `grade` always comes from native board
/// connectivity and clearance evaluation; rejected trials retain their failing grade.
#[derive(Clone, Debug, Serialize)]
pub struct SourceNetDecision {
    pub net: String,
    pub accepted: bool,
    pub reason: String,
    pub grade: NativeGrade,
    pub traces_added: usize,
    pub vias_added: usize,
    pub copper_mm_added: f64,
}

/// Result of applying a selected source plan to an isolated board transaction.
#[derive(Clone)]
pub struct SourceRealization {
    pub board: BoardAdapter,
    pub grade: NativeGrade,
    pub nets: Vec<SourceNetDecision>,
    pub audits: Vec<SourceMoveAudit>,
    pub refusals: Vec<SourceMoveRefusal>,
    pub traces_added: usize,
    pub vias_added: usize,
    pub copper_mm_added: f64,
}

/// Source endpoint readback and move-banning types share the verified native
/// measurement and exact ban class used by destination realization.
pub type SourceMoveAudit = super::destination_realize::DestinationMoveAudit;
pub type SourceMoveRefusal = super::destination_realize::DestinationMoveRefusal;

/// Apply the exact selected `JointPlan.pieces` to a cloned board, one net at a
/// time. A candidate net is committed only when native invariants hold and the
/// real clearance audit finds no new violations. Rejected nets leave accepted
/// earlier nets intact; the caller's input board is never mutated.
pub fn realize_source(
    board: &BoardAdapter,
    plan: &JointPlan,
    rules: &RoutingRules,
    drills: &mut BTreeMap<String, f64>,
    protected: &BTreeSet<String>,
) -> Result<SourceRealization, String> {
    rules.validate()?;

    let aliases: BTreeMap<String, Vec<String>> = (1..=board.board.rules.nets.max_net_number())
        .filter_map(|n| board.board.rules.nets.get(n))
        .fold(BTreeMap::new(), |mut map, net| {
            map.entry(super::escape_moves::short_name(&net.name).to_owned())
                .or_default()
                .push(net.name.clone());
            map
        });
    let resolve = |name: &str| -> Result<String, String> {
        if board.net_number(name).is_ok() {
            return Ok(name.to_owned());
        }
        let hits = aliases
            .get(name)
            .ok_or_else(|| format!("unknown AWX net {name}"))?;
        if hits.len() != 1 {
            return Err(format!("ambiguous AWX basename {name}"));
        }
        Ok(hits[0].clone())
    };

    // Normalize and group the real selected physical pieces. Resolve every
    // name before making a candidate so malformed plans cannot partially apply.
    let mut pieces: BTreeMap<String, Vec<&super::joint_geometry::Piece>> = BTreeMap::new();
    let mut protected_nets = BTreeSet::new();
    for piece in plan.pieces.values() {
        let net = resolve(&piece.net)?;
        if protected.contains(&net) {
            protected_nets.insert(net.clone());
        }
        pieces.entry(net).or_default().push(piece);
    }
    for option in plan.chosen.values() {
        if let super::joint_escape::OptionMove::Escape { movement } = option {
            let net = resolve(&movement.net)?;
            if protected.contains(&net) {
                protected_nets.insert(net.clone());
            }
            // Preserve every asked source tooth in the transaction/audit set,
            // even when the plan carries no physical piece for that choice.
            pieces.entry(net).or_default();
        }
    }

    let mut committed = board.clone();
    let mut committed_drills = drills.clone();
    let mut decisions = Vec::with_capacity(pieces.len());
    let mut audits = Vec::new();
    let mut refusals = Vec::new();
    for (net, net_pieces) in pieces {
        let before_net = committed.clone();
        let net_set = BTreeSet::from([net.clone()]);
        let asks = source_asks(&before_net, plan, &net, &resolve)?;
        if protected_nets.contains(&net) {
            let grade = native_execution::grade(&before_net, &before_net, &net_set)?;
            let reason = "source plan selects a protected net";
            for ask in &asks {
                audits.push(destination_realize::refused_audit(ask, reason));
                refusals.push(destination_realize::refusal_for(ask, reason.into()));
            }
            decisions.push(SourceNetDecision {
                net,
                accepted: false,
                reason: reason.into(),
                grade,
                traces_added: 0,
                vias_added: 0,
                copper_mm_added: 0.0,
            });
            continue;
        }
        let mut trial = before_net.clone();
        let mut trial_drills = committed_drills.clone();
        let before_traces = trial.board.get_traces().len();
        let before_vias = trial.board.get_vias().len();

        let mut materialize_error = None;
        'pieces: for piece in &net_pieces {
            for via in &piece.vias {
                if let Err(error) = native_execution::via(
                    &mut trial,
                    &net,
                    via.point,
                    2.0 * via.radius,
                    2.0 * via.drill_radius,
                ) {
                    materialize_error = Some(error);
                    break 'pieces;
                }
                let radius = (via.radius * trial.units_per_mm).round() as i32;
                let drill = (2.0 * via.drill_radius * trial.units_per_mm).round() as i32;
                trial_drills.insert(
                    format!("awx_via_r{radius}_d{drill}"),
                    2.0 * via.drill_radius,
                );
            }
            for leg in &piece.legs {
                if let Err(error) = native_execution::trace(
                    &mut trial,
                    &net,
                    &leg.layer,
                    leg.a,
                    leg.b,
                    rules.track_width,
                ) {
                    materialize_error = Some(error);
                    break 'pieces;
                }
            }
        }

        let invariant = trial.assert_invariants(&before_net, protected, false);
        let before_grade = native_execution::grade(&before_net, &before_net, &net_set)?;
        let grade = native_execution::grade(&before_net, &trial, &net_set)?;
        let native_accepted =
            materialize_error.is_none() && invariant.is_ok() && grade.new_violations == 0;
        let native_reason = match (materialize_error, invariant, grade.new_violations) {
            (Some(error), _, _) => format!("native source insertion refused: {error}"),
            (None, Err(error), _) => format!("native invariant refusal: {error}"),
            (None, Ok(()), count) if count > 0 => {
                format!("native clearance audit found {count} new violation(s)")
            }
            (None, Ok(()), _) => "native invariant and clearance audit passed".into(),
        };
        let mut trial_audits = Vec::new();
        let mut audit_reason = None;
        if native_accepted {
            trial_audits = asks
                .iter()
                .map(|ask| destination_realize::audit_berth(&trial, ask))
                .collect::<Result<Vec<_>, _>>()?;
            if let Some(failed) = trial_audits.iter().find(|audit| !audit.exact) {
                audit_reason = Some(format!(
                    "source tooth audit refused {}: {}",
                    failed.ball,
                    failed
                        .readback_error
                        .as_deref()
                        .unwrap_or(&failed.lost.join(", "))
                ));
            }
        }
        let accepted = native_accepted && audit_reason.is_none();
        let reason = audit_reason.unwrap_or(native_reason);
        if native_accepted {
            audits.extend(trial_audits.iter().cloned());
        }
        if !accepted {
            if native_accepted {
                for (ask, audit) in asks.iter().zip(&trial_audits) {
                    if !audit.exact {
                        let refusal_reason = audit.readback_error.clone().unwrap_or_else(|| {
                            format!("source tooth audit lost {}", audit.lost.join(", "))
                        });
                        refusals.push(destination_realize::refusal_for(ask, refusal_reason));
                    }
                }
            } else {
                for ask in &asks {
                    audits.push(destination_realize::refused_audit(ask, &reason));
                    refusals.push(destination_realize::refusal_for(ask, reason.clone()));
                }
            }
        }
        let traces_added = trial.board.get_traces().len().saturating_sub(before_traces);
        let vias_added = trial.board.get_vias().len().saturating_sub(before_vias);
        let copper_mm_added = (grade.copper_mm - before_grade.copper_mm).max(0.0);
        decisions.push(SourceNetDecision {
            net: net.clone(),
            accepted,
            reason,
            grade: grade.clone(),
            traces_added: if accepted { traces_added } else { 0 },
            vias_added: if accepted { vias_added } else { 0 },
            copper_mm_added: if accepted { copper_mm_added } else { 0.0 },
        });

        if accepted {
            committed = trial;
            committed_drills = trial_drills;
        }
    }

    let routed: BTreeSet<_> = decisions
        .iter()
        .map(|decision| decision.net.clone())
        .collect();
    let grade = native_execution::grade(board, &committed, &routed)?;
    let traces_added = committed
        .board
        .get_traces()
        .len()
        .saturating_sub(board.board.get_traces().len());
    let vias_added = committed
        .board
        .get_vias()
        .len()
        .saturating_sub(board.board.get_vias().len());
    let copper_mm_added = decisions
        .iter()
        .filter(|decision| decision.accepted)
        .map(|decision| decision.copper_mm_added)
        .sum();
    *drills = committed_drills;

    Ok(SourceRealization {
        board: committed,
        grade,
        nets: decisions,
        audits,
        refusals,
        traces_added,
        vias_added,
        copper_mm_added,
    })
}

fn source_asks<F>(
    board: &BoardAdapter,
    plan: &JointPlan,
    net: &str,
    resolve: &F,
) -> Result<Vec<AskedBerth>, String>
where
    F: Fn(&str) -> Result<String, String>,
{
    let mut asks = Vec::new();
    for (ball, selected) in &plan.chosen {
        let super::joint_escape::OptionMove::Escape { movement } = selected else {
            continue;
        };
        if resolve(&movement.net)? != net {
            continue;
        }
        let pair = plan
            .hints
            .iter()
            .find(|hint| hint.ball == *ball)
            .and_then(|hint| hint.hint.get("pair"))
            .and_then(serde_json::Value::as_str)
            .map(str::to_owned);
        let piece = plan
            .pieces
            .get(ball)
            .ok_or_else(|| format!("source plan lacks physical piece for {ball}"))?;
        asks.push(destination_realize::asked_berth_for_piece(
            board,
            ball.clone(),
            movement.clone(),
            pair,
            piece,
            false,
        )?);
    }
    Ok(asks)
}

/// Convert a planned move to the native router's full hint representation.
///
/// The router consumes the exit exactly, and for pages-first planning it also
/// lays the selected legs verbatim. Dog-bone paths are carried only when the
/// source legs describe a nontrivial walk to the via site.
pub fn full_move(m: &Move, pages: bool) -> Value {
    let face = match m.direction {
        super::escape_moves::Direction::Left => "left",
        super::escape_moves::Direction::Right => "right",
        super::escape_moves::Direction::Up => "up",
        super::escape_moves::Direction::Down => "down",
    };
    let kind = match m.kind {
        MoveKind::Surface => "surface",
        MoveKind::Dogbone => "dogbone",
        MoveKind::ViaInPad => "via_in_pad",
    };
    let mut hint = json!({
        "face": face,
        "exit": m.exit_pt,
        "layer": m.layer,
        "kind": kind,
        "site": m.site,
    });

    if pages && !m.legs.is_empty() {
        hint["legs"] = json!(m
            .legs
            .iter()
            .map(|leg| (leg.a, leg.b, leg.layer.as_str()))
            .collect::<Vec<_>>());
    }

    if m.kind == MoveKind::Dogbone {
        if let Some(site) = m.site {
            let stub: Vec<_> = m.legs.iter().filter(|leg| leg.layer != m.layer).collect();
            if let Some(last) = stub.last() {
                let reaches_site = (last.b[0] - site[0]).hypot(last.b[1] - site[1]) < 1e-6;
                let axis_aligned = stub.first().is_some_and(|first| {
                    (first.b[0] - first.a[0])
                        .abs()
                        .min((first.b[1] - first.a[1]).abs())
                        < 1e-6
                });
                if reaches_site && (stub.len() > 1 || axis_aligned) {
                    let mut path = Vec::with_capacity(stub.len() + 1);
                    path.push(stub[0].a);
                    path.extend(stub.iter().map(|leg| leg.b));
                    hint["path"] = json!(path);
                }
            }
        }
    }
    hint
}
