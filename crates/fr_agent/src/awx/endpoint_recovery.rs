//! ORIGINAL default source rip-up custody/ranking for a failed endpoint route.
//!
//! This is the dependency-free port of the single-ended rip ladder:
//!
//! * `single_ended_loop.py:1073..1600` — the progressive attack ladder, its
//!   re-attribution of the *latest* frontier against the *latest stripped*
//!   board, the exact once-only slot-0 guard (`:1118..1143`, gated on exactly
//!   one ripped victim), the rip-and-retry history gate and the terminal
//!   restore-on-failure contract.
//! * `rip_up_reroute.py:78` `rip_up_net` and `:605` `restore_net` — custody of
//!   the pre-rip copper and the #134 collision-refused restore.
//! * `reroute_loop.py:467` — the same ladder run from the reroute queue.
//! * `blocking_analysis.py` — `analyze_frontier_blocking` / `rank_blockers`
//!   (default `count`) and `filter_rippable_blockers`.
//! * `routing_defaults.py:132` `MAX_RIPUP = 3` — the ladder bound.
//!
//! Geometry and ranking contracts are reused verbatim from the already-ported
//! `awx_grid::blocking_analysis` frontier attribution (the same capsule/via
//! kernels and `Ranking::Count` order consumed by
//! `replan_probes.rs::frontier_blocker_order`). No native surrogate ranker is
//! introduced.
//!
//! ## Source ledger (what may be ripped)
//!
//! Eligibility is ONLY this-run copper: the copper `current` carries that the
//! actual input `baseline` does not (`lane_copper`'s `delta` rule). A victim is
//! torn by cloning `current`, stripping the victim nets, then restoring their
//! `baseline` copper (`native_execution::strip_nets` + `restore_nets`). Original
//! file copper, user-fixed copper and protected nets are therefore preserved by
//! construction — this module never invents rip authority. The `#521`/import
//! "pre-existing rip" branch of `rip_up_net` (`is_existing_route`) is irrelevant
//! here because the current baseline carries zero of those nets' copper: only
//! the transactional delta is ever removed.
//!
//! ## Via-in-pad unblock blame keystone (wired, never fabricated)
//!
//! `single_ended_loop.py:1034..1071` front-loads a **via-in-pad unblock blame**
//! keystone: `_place_shrunk_via_in_pad` / `plane_blocker_detection` records the
//! foreign net whose committed Trace/Via copper boxes the pad
//! (`pcb_data._via_unblock_blame`), and the ladder promotes that named keystone
//! to slot 0 because frontier attribution structurally cannot see the copper
//! under the pad. The native backend emits the same set on the PRIVATE report
//! key `_via_unblock_blame` (a vector of native net numbers, present only when a
//! declined tap's pad is boxed by committed foreign copper -- never an in-flight
//! guess, never a fabricated net). The driver pops that key out of the report
//! before logging and hands it to [`RecoveryState::set_attempt_blame`]; this
//! module promotes only the blame nets that are THIS-RUN eligible, canonically
//! sorted by native net id and deduplicated, so static Pin/imported Cu owners and
//! protected/ancestry nets are never ripped.
//!
//! ## Empty is valid, not failure
//!
//! The classic single-ended blockers are frequently PIN copper (pads, planes,
//! pre-existing static copper) that no amount of ripping can clear. An empty
//! frontier, an empty eligible set or an empty ranked order all return
//! `Ok(None)` — the ladder simply has nothing to rip — UNLESS the via-unblock
//! blame keystone names a THIS-RUN net, which is promoted even when the ranked
//! frontier is empty (the whole point of the keystone is that the blocking
//! copper under the pad is invisible to frontier attribution).

use super::{braid_connect, native_execution};
use crate::board_adapter::BoardAdapter;
use awx_grid::blocking_analysis::{
    self, NetObstacles, Ranking, TrackObstacle, ViaObstacle,
};
use fr_engine::ids::{FixedState, NetNo};
use fr_geom::{Point, Polyline};
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::sync::Arc;

/// Source `routing_defaults.py:132` `MAX_RIPUP`. The ladder tries N = 1, 2, ...,
/// `MAX_RIPUP` blockers. Deliberately not configurable here and deliberately not
/// raised to the reverted `N = 5` experiment.
const MAX_RIPUP: usize = 3;

/// One committed rip-and-retry: the attacker `target` and the victim nets that
/// were torn to let it land. This is the source `rip_and_retry_history` /
/// `record_casualty` pair, kept per-run on the caller's `RecoveryState` — never
/// in a global or task-local.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct RecoveryCommit {
    pub target: NetNo,
    pub victims: Vec<NetNo>,
}

/// Run-scoped rip custody. Holds the victims queued for the driver's post-main
/// reroute pass and the successful-commit history the ladder's gate consults.
/// No globals, no task memory: the driver owns exactly one instance.
#[derive(Default)]
pub(crate) struct RecoveryState {
    queued_victims: VecDeque<NetNo>,
    /// Per-victim restore custody. Victims committed by the same rip share one
    /// immutable pre-rip snapshot; a later rip replaces this victim's snapshot
    /// even if it is already queued, matching the source `_rip_saved` overwrite.
    victim_restore_boards: BTreeMap<NetNo, Arc<BoardAdapter>>,
    /// Source `queued_net_ids`: a net is enqueued at most once while it is still
    /// queued, so a victim ripped by several committed frames appears once.
    queued_net_ids: BTreeSet<NetNo>,
    successful_commits: Vec<RecoveryCommit>,
    /// Source `record_rip_ancestry` (`routing_state.py:311`): victim -> the
    /// attacker chain that ripped it, so a rerouted victim never rips its ripper
    /// (or the ripper's ancestors) back.
    rip_ancestry: BTreeMap<NetNo, BTreeSet<NetNo>>,
    /// Source `restore_protected_net_ids` (`#622` victim-priority restore): nets
    /// whose pre-rip copper was restored with authority and so must not be
    /// re-ripped on reroute. Modelled faithfully, never guessed: the producer is
    /// absent in this port (see the module-level missing-primitive note).
    restore_protected_net_ids: BTreeSet<NetNo>,
    /// Endpoints of the edge currently under recovery, in the source
    /// `get_net_endpoints` `(orig_x, orig_y)` convention. The endpoint owner
    /// records these before each edge; they feed the `count` ranking's
    /// endpoint-proximity tie-break. `None` means the source itself had no such
    /// endpoint — never a fabricated coordinate.
    attempt_source_xy: Option<[f64; 2]>,
    attempt_target_xy: Option<[f64; 2]>,
    /// Source `pcb_data._via_unblock_blame` for the edge currently under
    /// recovery (`single_ended_loop.py:1034..1071`): the foreign net whose
    /// committed copper boxes a shrunk via's pad. Set by the endpoint owner from
    /// the backend's private report key before each edge and consumed once by
    /// the ladder (the pop resets it), so no edge ever inherits a stale keystone.
    /// Empty means the failing attempt named no blame — never a fabricated net.
    attempt_blame: BTreeSet<NetNo>,
    /// Snapshot and existing routed nets at the start of a nested source
    /// reconcile epoch. The ordinary top-level pass continues to use its
    /// original source baseline.
    epoch_start: Option<Arc<BoardAdapter>>,
    epoch_existing_nets: BTreeSet<NetNo>,
    nested_authorized_nets: BTreeSet<NetNo>,
    nested: bool,
    nested_stub_boards: BTreeMap<NetNo, Arc<BoardAdapter>>,
    priority_sweep_taken: bool,
    routed_result_nets: BTreeSet<NetNo>,
    preexisting_hints: BTreeMap<NetNo, Vec<NetNo>>,
}

impl RecoveryState {
    /// Record the endpoints of the edge about to be recovered (source
    /// `single_source_xy` / `single_target_xy`, `single_ended_loop.py:955..960`).
    /// The endpoint owner calls this before each edge; `None` is passed through
    /// rather than invented.
    pub(crate) fn set_attempt_endpoints(
        &mut self,
        source_xy: Option<[f64; 2]>,
        target_xy: Option<[f64; 2]>,
    ) {
        self.attempt_source_xy = source_xy;
        self.attempt_target_xy = target_xy;
    }

    /// Record the ORIGINAL source via-unblock blame keystone for the edge about
    /// to be recovered (`single_ended_loop.py:1034..1071`,
    /// `pcb_data._via_unblock_blame`). Called by the endpoint owner from the
    /// backend's private report key before each edge. REPLACES any standing
    /// blame, so an edge (and an attempt) never inherits a stale keystone; an
    /// empty set is a genuine no-blame failure, never a fabricated net.
    pub(crate) fn set_attempt_blame(&mut self, blame: BTreeSet<NetNo>) {
        self.attempt_blame = blame;
    }

    /// Pop the standing blame for THIS edge (the source ladder consumes the
    /// keystone once). Leaves the state empty so the next edge starts fresh.
    fn take_attempt_blame(&mut self) -> BTreeSet<NetNo> {
        std::mem::take(&mut self.attempt_blame)
    }

    /// Pop the next victim the driver must reroute after the main pass and after
    /// the taps (the original source phase contract). FIFO, so victims queue in
    /// the ladder's rip order. A popped victim leaves the dedup set, so a later
    /// rip of the same net is not silently lost.
    pub(crate) fn pop_queued_victim(&mut self) -> Option<(NetNo, Arc<BoardAdapter>)> {
        while let Some(victim) = self.queued_victims.pop_front() {
            self.queued_net_ids.remove(&victim);
            if let Some(saved) = self.victim_restore_boards.remove(&victim) {
                return Some((victim, saved));
            }
        }
        None
    }

    pub(crate) fn begin_nested(&mut self, start: &BoardAdapter, authorized: &BTreeSet<NetNo>) {
        self.queued_victims.clear();
        self.victim_restore_boards.clear();
        self.queued_net_ids.clear();
        self.successful_commits.clear();
        self.rip_ancestry.clear();
        self.restore_protected_net_ids.clear();
        self.routed_result_nets.clear();
        self.preexisting_hints.clear();
        self.nested_stub_boards.clear();
        self.priority_sweep_taken = false;
        self.attempt_blame.clear();
        self.attempt_source_xy = None;
        self.attempt_target_xy = None;
        self.epoch_start = Some(Arc::new(start.clone()));
        let existing_copper = start
            .board
            .get_traces()
            .into_iter()
            .chain(start.board.get_vias())
            .map(|key| {
                let detachable = !start.is_source_protected_copper(key);
                let nets = start.board.item(key).net_numbers().to_vec();
                (detachable, nets)
            })
            .collect::<Vec<_>>();
        self.epoch_existing_nets = existing_copper
            .iter()
            .flat_map(|(_, nets)| nets.iter().copied())
            .collect();
        self.nested_authorized_nets = authorized.clone();
        let detachable_nets: BTreeSet<NetNo> = existing_copper
            .iter()
            .filter(|(detachable, _)| *detachable)
            .flat_map(|(_, nets)| nets.iter().copied())
            .collect();
        self.routed_result_nets
            .extend(authorized.intersection(&detachable_nets).copied());
        self.nested = true;
    }

    pub(crate) fn take_preexisting_hints(&mut self) -> BTreeMap<NetNo, Vec<NetNo>> {
        std::mem::take(&mut self.preexisting_hints)
    }

    pub(crate) fn note_routed_result(&mut self, net: NetNo) {
        self.routed_result_nets.insert(net);
    }

    pub(crate) fn routed_result_nets(&self) -> &BTreeSet<NetNo> {
        &self.routed_result_nets
    }

    pub(crate) fn record_terminal_restore(
        &mut self,
        net: NetNo,
        saved: Arc<BoardAdapter>,
        status: &str,
    ) {
        if status == "stub" {
            self.nested_stub_boards.insert(net, saved);
        } else if status == "full" || status == "full_open" {
            self.nested_stub_boards.remove(&net);
        }
    }

    pub(crate) fn take_priority_candidates(&mut self) -> Vec<(NetNo, Arc<BoardAdapter>)> {
        if !self.nested || self.priority_sweep_taken {
            return Vec::new();
        }
        self.priority_sweep_taken = true;
        std::mem::take(&mut self.nested_stub_boards).into_iter().collect()
    }

    pub(crate) fn restore_protected_nets(&self) -> &BTreeSet<NetNo> {
        &self.restore_protected_net_ids
    }

    pub(crate) fn commit_priority_restore(
        &mut self,
        target: NetNo,
        owners: Vec<NetNo>,
        before: &BoardAdapter,
    ) {
        self.record_commit(target, owners.clone(), before);
        self.restore_protected_net_ids.insert(target);
        self.routed_result_nets.insert(target);
        for owner in owners {
            self.routed_result_nets.remove(&owner);
        }
    }

    /// Successful-commit history, needed by the driver for casualty custody and
    /// by `already_tried` for the source history/termination gate.
    pub(crate) fn source_history(&self) -> &[RecoveryCommit] {
        &self.successful_commits
    }

    /// Source `rip_combo_already_tried` (`polarity_swap.py:160`): a combo whose
    /// canonical victim set already succeeded for this target is not re-run.
    /// Compared as a set, matching the source frozenset key.
    fn already_tried(&self, target: NetNo, victims: &[NetNo]) -> bool {
        let wanted: BTreeSet<NetNo> = victims.iter().copied().collect();
        self.successful_commits.iter().any(|commit| {
            commit.target == target && commit.victims.iter().copied().collect::<BTreeSet<_>>() == wanted
        })
    }

    /// Source `record_rip_ancestry` (`routing_state.py:311..324`): the ripped
    /// net's ancestry is the ripper's own chain plus the ripper, minus the ripped
    /// net itself. The `- {victim}` term is the mutual-ancestor closure — a net
    /// is never its own ancestor. Replaces any prior value, exactly as source.
    fn record_rip_ancestry(&mut self, attacker: NetNo, victim: NetNo) {
        if attacker == victim {
            return;
        }
        let mut ancestry = self.rip_ancestry.get(&attacker).cloned().unwrap_or_default();
        ancestry.insert(attacker);
        ancestry.remove(&victim);
        self.rip_ancestry.insert(victim, ancestry);
    }

    /// Source `rip_exclude_set` (`routing_state.py:357..362`): nets a (re)routing
    /// `target` must NOT rip — itself, its rip ancestry, and every `#622`
    /// restore-protected net. Consumed by both the frontier attribution
    /// (`exclude_net_ids`) and the rippable filter.
    fn rip_exclude_set(&self, target: NetNo) -> BTreeSet<NetNo> {
        let mut exclude: BTreeSet<NetNo> = BTreeSet::from([target]);
        if let Some(ancestry) = self.rip_ancestry.get(&target) {
            exclude.extend(ancestry.iter().copied());
        }
        exclude.extend(self.restore_protected_net_ids.iter().copied());
        exclude
    }

    /// Record a committed rip: remember the combo (history gate), record the
    /// victim->attacker ancestry, and queue the victims for reroute (custody
    /// hand-off to the driver), deduplicating against the nets still queued.
    /// The landed attacker leaves the queue set (`single_ended_loop.py:1372`
    /// `queued_net_ids.discard(net_id)`), so if it is ripped again later it can
    /// be re-queued.
    fn record_commit(&mut self, target: NetNo, victims: Vec<NetNo>, before_rip: &BoardAdapter) {
        self.queued_net_ids.remove(&target);
        self.routed_result_nets.insert(target);
        for victim in &victims {
            self.routed_result_nets.remove(victim);
        }
        // `before_rip` is the same custody source for every victim in this
        // committed combo. Share one owned snapshot instead of cloning the
        // entire board once per victim. The source keeps a per-victim copper
        // payload; restore_nets selects that victim by name from this snapshot.
        let saved_board = Arc::new(before_rip.clone());
        for &victim in &victims {
            self.record_rip_ancestry(target, victim);
            self.victim_restore_boards
                .insert(victim, Arc::clone(&saved_board));
            if self.queued_net_ids.insert(victim) {
                self.queued_victims.push_back(victim);
            }
        }
        self.successful_commits.push(RecoveryCommit { target, victims });
    }
}

/// The accepted rip result: the landed board, the caller's typed target data
/// (achieved endpoints, grade, route — whatever `T` the retry produced), and the
/// victims that were torn and must be rerouted by the driver.
pub(crate) struct RipAccepted<T> {
    pub board: BoardAdapter,
    pub target_data: T,
    pub victims: Vec<NetNo>,
}

pub(crate) struct FailedVictimRestore {
    pub board: Option<BoardAdapter>,
    pub status: &'static str,
    pub full_restore_error: Option<String>,
    pub full_invariant_error: Option<String>,
    pub stub_error: Option<String>,
    pub restored_segments: usize,
    pub restored_vias: usize,
    pub retained_new_drc: Vec<serde_json::Value>,
}

#[derive(Clone)]
struct EscapeStubSegment {
    start: Point,
    end: Point,
    far_mm: [f64; 2],
    layer: i32,
    half_width: i32,
    clearance_class: i32,
    fixed_state: FixedState,
}

#[derive(Clone)]
struct EscapeStubVia {
    padstack_name: String,
    center: Point,
    center_mm: [f64; 2],
    clearance_class: i32,
    fixed_state: FixedState,
    attach_allowed: bool,
}

/// Resolve a native net number to its board name.
fn net_name(board: &BoardAdapter, net: NetNo) -> Result<String, String> {
    board
        .board
        .rules
        .nets
        .get(net)
        .map(|net| net.name.clone())
        .ok_or_else(|| format!("native board has no net {net}"))
}

fn retained_drc_rows(
    board: &BoardAdapter,
    rows: &[crate::board_adapter::NewClearanceViolation],
    phase: &str,
) -> Vec<serde_json::Value> {
    let items: BTreeMap<_, _> = board
        .board
        .get_items()
        .into_iter()
        .map(|key| (board.board.item(key).id().0, key))
        .collect();
    rows.iter()
        .map(|violation| {
            let record = &violation.record;
            let item_row = |item_id: i32, kind: &str| {
                let nets = items
                    .get(&item_id)
                    .map(|&key| {
                        board
                            .board
                            .item(key)
                            .net_numbers()
                            .iter()
                            .filter_map(|&net| board.board.rules.nets.get(net))
                            .map(|net| net.name.clone())
                            .collect::<Vec<_>>()
                    })
                    .unwrap_or_default();
                serde_json::json!({"item_id":item_id,"type":kind,"nets":nets})
            };
            serde_json::json!({
                "phase": phase,
                "signature": violation.signature,
                "pair": [
                    item_row(record.first_id, &record.first_type),
                    item_row(record.second_id, &record.second_type),
                ],
                "layer": record.layer,
                "native_gap_units": record.actual_clearance_mm * board.units_per_mm,
                "native_gap_mm": record.actual_clearance_mm,
                "exact_copper_clearance_mm": record.exact_copper_clearance_mm,
                "expected_clearance_mm": record.expected_clearance_mm,
                "exact_shortfall_mm": record.exact_copper_clearance_mm
                    .map(|gap| record.expected_clearance_mm - gap),
                "overlap_bounds": record.overlap_bounds,
            })
        })
        .collect()
}

fn victim_copper_added(before: &BoardAdapter, after: &BoardAdapter, victim: NetNo) -> (usize, usize) {
    let before_ids: BTreeSet<_> = before
        .board
        .get_traces()
        .into_iter()
        .chain(before.board.get_vias())
        .filter(|&key| before.board.item(key).contains_net(victim))
        .map(|key| before.board.item(key).id().0)
        .collect();
    let count = |keys: Vec<fr_engine::board::ItemKey>| {
        keys.into_iter()
            .filter(|&key| {
                let item = after.board.item(key);
                item.contains_net(victim) && !before_ids.contains(&item.id().0)
            })
            .count()
    };
    (count(after.board.get_traces()), count(after.board.get_vias()))
}

fn segment_already_present(
    board: &BoardAdapter,
    net: NetNo,
    candidate: &EscapeStubSegment,
) -> bool {
    let a_key = crate::board_adapter::canonical_point_key(&candidate.start);
    let b_key = crate::board_adapter::canonical_point_key(&candidate.end);
    board.board.get_traces().into_iter().any(|key| {
        let item = board.board.item(key);
        if !item.contains_net(net) {
            return false;
        }
        let trace = item.trace();
        if trace.layer() != candidate.layer || trace.half_width() != candidate.half_width {
            return false;
        }
        trace.polyline().corners().windows(2).any(|pair| {
            let start = crate::board_adapter::canonical_point_key(&pair[0]);
            let end = crate::board_adapter::canonical_point_key(&pair[1]);
            (start == a_key && end == b_key) || (start == b_key && end == a_key)
        })
    })
}

fn via_already_present(board: &BoardAdapter, net: NetNo, candidate: &EscapeStubVia) -> bool {
    let center = crate::board_adapter::canonical_point_key(&candidate.center);
    let center_mm_key = rounded_mm_key(candidate.center_mm);
    board.board.get_vias().into_iter().any(|key| {
        let item = board.board.item(key);
        let item_center = item.center(&board.board);
        let point = item_center.to_float();
        let item_mm_key = rounded_mm_key([
            point.x / board.units_per_mm,
            point.y / board.units_per_mm,
        ]);
        item.contains_net(net)
            && item
                .padstack(&board.board)
                .is_some_and(|stack| stack.name.as_str() == candidate.padstack_name)
            && crate::board_adapter::canonical_point_key(&item_center) == center
            && item_mm_key == center_mm_key
    })
}

fn round_three_mm(value: f64) -> i64 {
    (value * 1000.0).round_ties_even() as i64
}

fn rounded_mm_key(point: [f64; 2]) -> Option<(i64, i64)> {
    point
        .iter()
        .all(|value| value.is_finite())
        .then(|| (round_three_mm(point[0]), round_three_mm(point[1])))
}

fn axis_near_mm(a: [f64; 2], b: [f64; 2]) -> bool {
    (a[0] - b[0]).abs() < 1e-3 && (a[1] - b[1]).abs() < 1e-3
}

pub(crate) fn restore_failed_victim(
    saved: &BoardAdapter,
    current: &BoardAdapter,
    baseline: &BoardAdapter,
    protected: &BTreeSet<String>,
    victim: NetNo,
) -> FailedVictimRestore {
    let mut outcome = FailedVictimRestore {
        board: None,
        status: "declined",
        full_restore_error: None,
        full_invariant_error: None,
        stub_error: None,
        restored_segments: 0,
        restored_vias: 0,
        retained_new_drc: Vec::new(),
    };
    let victim_name = match net_name(saved, victim) {
        Ok(name) => name,
        Err(error) => {
            outcome.full_restore_error = Some(error);
            return outcome;
        }
    };
    let victim_names = BTreeSet::from([victim_name]);

    let mut full = current.clone();
    match native_execution::restore_nets(saved, &mut full, &victim_names) {
        Err(error) => outcome.full_restore_error = Some(error),
        Ok(()) => {
            let (check, violations) =
                full.assert_invariants_with_violation_report(baseline, protected, true);
            if check.is_ok() {
                let open = fr_engine::drc::NetIncompletes::new(
                    &full.board,
                    victim,
                    &full.board.get_connectable_items(victim),
                )
                .count();
                let (segments, vias) = victim_copper_added(current, &full, victim);
                outcome.board = Some(full);
                outcome.status = if open == 0 { "full" } else { "full_open" };
                outcome.restored_segments = segments;
                outcome.restored_vias = vias;
                return outcome;
            }
            outcome.full_invariant_error = check.err();
            outcome
                .retained_new_drc
                .extend(retained_drc_rows(&full, &violations, "full_restore"));
        }
    }

    let mut pad_centers = Vec::<[f64; 2]>::new();
    for key in current.board.get_pins() {
        let item = current.board.item(key);
        if !item.contains_net(victim) {
            continue;
        }
        let center = item.center(&current.board).to_float();
        pad_centers.push([
            center.x / current.units_per_mm,
            center.y / current.units_per_mm,
        ]);
    }

    let mut stubs = Vec::<EscapeStubSegment>::new();
    for key in saved.board.get_traces() {
        let item = saved.board.item(key);
        if !item.contains_net(victim) || saved.is_source_protected_copper(key) {
            continue;
        }
        let trace = item.trace();
        for pair in trace.polyline().corners().windows(2) {
            let start = pair[0].to_float();
            let end = pair[1].to_float();
            let start_mm = [start.x / saved.units_per_mm, start.y / saved.units_per_mm];
            let end_mm = [end.x / saved.units_per_mm, end.y / saved.units_per_mm];
            if (end_mm[0] - start_mm[0]).hypot(end_mm[1] - start_mm[1]) > 2.0 {
                continue;
            }
            let far_mm = if pad_centers
                .iter()
                .any(|pad| axis_near_mm(start_mm, *pad))
            {
                Some(end_mm)
            } else if pad_centers.iter().any(|pad| axis_near_mm(end_mm, *pad)) {
                Some(start_mm)
            } else {
                None
            };
            let Some(far_mm) = far_mm else { continue };
            let rounded_far = rounded_mm_key(far_mm)
                .map(|(x, y)| [x as f64 / 1000.0, y as f64 / 1000.0])
                .unwrap_or(far_mm);
            stubs.push(EscapeStubSegment {
                start: pair[0].clone(),
                end: pair[1].clone(),
                far_mm: rounded_far,
                layer: trace.layer(),
                half_width: trace.half_width(),
                clearance_class: item.clearance_class(),
                fixed_state: item.fixed_state(),
            });
        }
    }
    let far_end_keys: BTreeSet<_> = stubs
        .iter()
        .filter_map(|stub| rounded_mm_key(stub.far_mm))
        .collect();

    let mut stub_vias = Vec::<EscapeStubVia>::new();
    for key in saved.board.get_vias() {
        let item = saved.board.item(key);
        if !item.contains_net(victim) || saved.is_source_protected_copper(key) {
            continue;
        }
        let Some(stack) = item.padstack(&saved.board) else {
            outcome
                .stub_error
                .get_or_insert_with(|| "saved via has no padstack".into());
            continue;
        };
        let Some(via) = item.as_via() else {
            outcome
                .stub_error
                .get_or_insert_with(|| "saved via item lost via data".into());
            continue;
        };
        let center = item.center(&saved.board);
        let point = center.to_float();
        let center_mm = [point.x / saved.units_per_mm, point.y / saved.units_per_mm];
        let is_terminal = rounded_mm_key(center_mm)
            .is_some_and(|key| far_end_keys.contains(&key))
            || pad_centers
                .iter()
                .any(|pad| axis_near_mm(center_mm, *pad));
        if !is_terminal {
            continue;
        }
        stub_vias.push(EscapeStubVia {
            padstack_name: stack.name.clone(),
            center,
            center_mm,
            clearance_class: item.clearance_class(),
            fixed_state: item.fixed_state(),
            attach_allowed: via.attach_allowed,
        });
    }
    if stubs.is_empty() && stub_vias.is_empty() {
        outcome
            .stub_error
            .get_or_insert_with(|| "no eligible terminal escape stubs".into());
        return outcome;
    }

    let mut stage = current.clone();
    let mut usable_stubs = 0usize;
    for stub in &stubs {
        if stub.layer < 0 || stub.layer as usize >= stage.board.layer_structure.layers.len() {
            outcome.stub_error.get_or_insert_with(|| {
                format!("stub trace uses unknown saved layer {}", stub.layer)
            });
            continue;
        }
        if stub.clearance_class < 0
            || stub.clearance_class >= stage.board.rules.clearance_matrix.get_class_count()
        {
            outcome
                .stub_error
                .get_or_insert_with(|| "stub trace has an unknown clearance class".into());
            continue;
        }
        if segment_already_present(&stage, victim, stub) {
            usable_stubs += 1;
            continue;
        }
        let mut candidate = stage.clone();
        let polyline = Polyline::from_two_points(&stub.start, &stub.end);
        let inserted = candidate.board.insert_trace_without_cleaning(
            polyline,
            stub.layer,
            stub.half_width,
            &[victim],
            stub.clearance_class,
            stub.fixed_state,
        );
        if inserted.is_none() {
            outcome
                .stub_error
                .get_or_insert_with(|| "stub trace insertion failed".into());
            continue;
        }
        let report = candidate.new_clearance_violations_since(&stage);
        if report.signatures.is_empty() {
            stage = candidate;
            outcome.restored_segments += 1;
            usable_stubs += 1;
        } else {
            outcome.retained_new_drc.extend(retained_drc_rows(
                &candidate,
                &report.records,
                "stub_segment",
            ));
            outcome
                .stub_error
                .get_or_insert_with(|| "stub trace caused retained new DRC".into());
        }
    }
    for stub in &stub_vias {
        if stub.clearance_class < 0
            || stub.clearance_class >= stage.board.rules.clearance_matrix.get_class_count()
        {
            outcome
                .stub_error
                .get_or_insert_with(|| "stub via has an unknown clearance class".into());
            continue;
        }
        if via_already_present(&stage, victim, stub) {
            usable_stubs += 1;
            continue;
        }
        let Some(padstack) = stage
            .board
            .library
            .padstacks
            .get_by_name(&stub.padstack_name)
        else {
            outcome.stub_error.get_or_insert_with(|| {
                format!("stub via padstack {} is unknown", stub.padstack_name)
            });
            continue;
        };
        let mut candidate = stage.clone();
        candidate.board.insert_via(
            padstack.id,
            stub.center.clone(),
            &[victim],
            stub.clearance_class,
            stub.fixed_state,
            stub.attach_allowed,
        );
        let report = candidate.new_clearance_violations_since(&stage);
        if report.signatures.is_empty() {
            stage = candidate;
            outcome.restored_vias += 1;
            usable_stubs += 1;
        } else {
            outcome.retained_new_drc.extend(retained_drc_rows(
                &candidate,
                &report.records,
                "stub_via",
            ));
            outcome
                .stub_error
                .get_or_insert_with(|| "stub via caused retained new DRC".into());
        }
    }
    if usable_stubs == 0 {
        outcome
            .stub_error
            .get_or_insert_with(|| "no stub copper passed insertion and retained DRC".into());
        return outcome;
    }
    let (check, violations) =
        stage.assert_invariants_with_violation_report(baseline, protected, true);
    if let Err(error) = check {
        outcome.stub_error = Some(error);
        outcome
            .retained_new_drc
            .extend(retained_drc_rows(&stage, &violations, "stub_final_invariants"));
        return outcome;
    }
    outcome.status = "stub";
    outcome.board = Some(stage);
    outcome
}

/// Nets the run itself introduced: present in `current`, absent from the actual
/// input `baseline`, and neither the `target`, `protected`, nor any net in the
/// source `rip_exclude_set` (the `target`, its rip ancestry, the `#622`
/// restore-protected nets). Mirrors `lane_copper`'s `delta` membership rule
/// (`replan_probes.rs:1969`).
fn this_run_eligible(
    current: &BoardAdapter,
    baseline: &BoardAdapter,
    target: NetNo,
    protected: &BTreeSet<String>,
    excluded: &BTreeSet<NetNo>,
    nested: bool,
    authorized: &BTreeSet<NetNo>,
) -> Result<BTreeSet<String>, String> {
    let target_name = net_name(current, target)?;
    let baseline_ids: BTreeSet<i32> = baseline
        .board
        .get_traces()
        .into_iter()
        .chain(baseline.board.get_vias())
        .map(|key| baseline.board.item(key).id().0)
        .collect();
    let mut eligible = BTreeSet::new();
    for key in current
        .board
        .get_traces()
        .into_iter()
        .chain(current.board.get_vias())
    {
        let item = current.board.item(key);
        if current.is_source_protected_copper(key) {
            continue;
        }
        let is_new = !baseline_ids.contains(&item.id().0);
        let is_authorized_existing = nested
            && authorized.iter().any(|net| item.contains_net(*net))
            && !current.is_source_protected_copper(key);
        if !is_new && !is_authorized_existing {
            continue;
        }
        for &number in item.net_numbers() {
            let Some(net) = current.board.rules.nets.get(number) else {
                continue;
            };
            if net.name == target_name || protected.contains(&net.name) || excluded.contains(&number)
            {
                continue;
            }
            eligible.insert(net.name.clone());
        }
    }
    Ok(eligible)
}

/// Build the exact frontier keep-out for one eligible net on the report's own
/// route-layer index space: whole-net trace capsules and via copper priced at
/// the native net-class pair clearance (mirrors `lane_copper`'s `all` set and
/// `frontier_blocker_order`'s `TrackObstacle`/`ViaObstacle` margins).
fn build_net_obstacles(
    board: &BoardAdapter,
    net: NetNo,
    net_label: &str,
    route_layers: &[String],
    target: NetNo,
    target_class: i32,
) -> Result<Option<NetObstacles>, String> {
    let upm = board.units_per_mm;
    let other_class = board
        .board
        .rules
        .net_class_of(net)
        .get_trace_clearance_class();
    let mut tracks = Vec::new();
    for key in board.board.get_traces() {
        let item = board.board.item(key);
        if !item.net_numbers().contains(&net) {
            continue;
        }
        let trace = item.trace();
        let layer_no = trace.layer();
        let layer_name = &board.board.layer_structure.layers[layer_no as usize].name;
        let Some(layer) = route_layers.iter().position(|name| name == layer_name) else {
            continue;
        };
        let target_half = board.board.rules.get_trace_half_width(target, layer_no) as f64 / upm;
        let other_half = trace.half_width() as f64 / upm;
        let clearance =
            board.board.clearance_value(target_class, other_class, layer_no) as f64 / upm;
        let margin = target_half + other_half + clearance;
        let points = trace.polyline().corner_approx_arr();
        for pair in points.windows(2) {
            tracks.push(TrackObstacle {
                start: [pair[0].x / upm, pair[0].y / upm],
                end: [pair[1].x / upm, pair[1].y / upm],
                layer,
                margin,
            });
        }
    }
    let mut vias = Vec::new();
    for key in board.board.get_vias() {
        let item = board.board.item(key);
        if !item.net_numbers().contains(&net) {
            continue;
        }
        let stack = item
            .padstack(&board.board)
            .ok_or_else(|| format!("rip-up via for {net_label} has no padstack"))?;
        let diameter = (stack.from_layer()..=stack.to_layer())
            .filter_map(|layer| stack.get_shape(layer))
            .map(|shape| shape.max_width())
            .fold(0.0_f64, f64::max)
            / upm;
        let center = item.center(&board.board).to_float();
        // A via spans every route layer; use the tightest pair clearance over the
        // report's route layers, exactly as the via keep-out applies on each.
        let mut margin = f64::INFINITY;
        for name in route_layers {
            let layer_no = board.board.layer_structure.get_no(name);
            if layer_no < 0 {
                continue;
            }
            let target_half =
                board.board.rules.get_trace_half_width(target, layer_no) as f64 / upm;
            let clearance =
                board.board.clearance_value(target_class, other_class, layer_no) as f64 / upm;
            margin = margin.min(diameter / 2.0 + target_half + clearance);
        }
        if !margin.is_finite() {
            continue;
        }
        vias.push(ViaObstacle {
            point: [center.x / upm, center.y / upm],
            margin,
        });
    }
    if tracks.is_empty() && vias.is_empty() {
        return Ok(None);
    }
    Ok(Some(NetObstacles {
        net_id: net,
        net_name: net_label.to_owned(),
        tracks,
        vias,
    }))
}

/// Rank eligible victims by the ORIGINAL source default (`blocking_analysis.py`
/// `rank_blockers(..., 'count')`), reusing the ported `awx_grid` frontier
/// attribution. `excluded` carries the target so it is never attributed to
/// itself; protected nets are filtered after ranking.
///
/// `board` is the board the frontier was produced on: the untouched `current`
/// for the initial failure, and the *stripped* trial for every N > 1 re-analysis
/// (`single_ended_loop.py:1089` re-attributes against the ripped `pcb_data`), so
/// a victim already torn is invisible to the fresh ranking. `target_xy` /
/// `source_xy` are the source endpoint-proximity tie-break inputs, forwarded
/// verbatim from the recorder.
fn rank_victims(
    board: &BoardAdapter,
    target: NetNo,
    eligible: &BTreeSet<String>,
    report: &braid_connect::RungReport,
    target_xy: Option<[f64; 2]>,
    source_xy: Option<[f64; 2]>,
) -> Result<Vec<String>, String> {
    if report.blocked_cells.is_empty() {
        return Ok(Vec::new());
    }
    let target_class = board
        .board
        .rules
        .net_class_of(target)
        .get_trace_clearance_class();
    let mut candidates = Vec::new();
    for name in eligible {
        let Ok(net) = board.net_number(name) else {
            continue;
        };
        if let Some(obstacles) =
            build_net_obstacles(board, net, name, &report.route_layers, target, target_class)?
        {
            candidates.push(obstacles);
        }
    }
    let excluded: BTreeSet<i32> = BTreeSet::from([target]);
    let mut attributed = blocking_analysis::analyze_frontier_blocking(
        &report.blocked_cells,
        &candidates,
        &excluded,
        report.grid_step_mm,
        report.route_layers.len(),
        // Source `target_xy` / `source_xy`: the `count` ranking's endpoint
        // proximity. Straight from the recorder — `None` only when the source
        // itself had no endpoint (never a fabricated coordinate or name tie).
        target_xy,
        source_xy,
    );
    blocking_analysis::rank_blockers(
        &mut attributed.entries,
        Ranking::Count,
        &BTreeSet::new(),
        0.0,
    );
    let mut order = Vec::new();
    for entry in attributed.entries {
        let Some(net) = board.board.rules.nets.get(entry.net_id) else {
            continue;
        };
        if eligible.contains(&net.name) && !order.contains(&net.name) {
            order.push(net.name.clone());
        }
    }
    Ok(order)
}

/// Collect source-backed blockers that existed at the input/epoch boundary
/// and still obstruct the actual frontier, while excluding nets the current
/// pass can already rip (including nets only hidden by rip ancestry).
fn record_preexisting_hints(
    current: &BoardAdapter,
    source: &BoardAdapter,
    existing_nets: &BTreeSet<NetNo>,
    target: NetNo,
    protected: &BTreeSet<String>,
    nested: bool,
    authorized: &BTreeSet<NetNo>,
    report: &braid_connect::RungReport,
    target_xy: Option<[f64; 2]>,
    source_xy: Option<[f64; 2]>,
    state: &mut RecoveryState,
) -> Result<(), String> {
    if report.blocked_cells.is_empty() {
        return Ok(());
    }
    let permissive_excluded = BTreeSet::from([target]);
    let currently_rippable = this_run_eligible(
        current,
        source,
        target,
        protected,
        &permissive_excluded,
        nested,
        authorized,
    )?
    .into_iter()
    .filter_map(|name| current.net_number(&name).ok())
    .collect::<BTreeSet<_>>();
    let routed_results = state.routed_result_nets.clone();
    let conduction_area_nets: BTreeSet<NetNo> = source
        .board
        .get_conduction_areas()
        .into_iter()
        .flat_map(|key| source.board.item(key).net_numbers().to_vec())
        .collect();
    let candidates: BTreeSet<NetNo> = existing_nets
        .difference(&currently_rippable)
        .copied()
        .filter(|net| {
            *net != target
                && *net != 0
                && !authorized.contains(net)
                && !routed_results.contains(net)
                && !conduction_area_nets.contains(net)
                && current
                    .board
                    .rules
                    .nets
                    .get(*net)
                    .is_some_and(|entry| !entry.contains_plane() && !protected.contains(&entry.name))
        })
        .collect();
    if candidates.is_empty() {
        return Ok(());
    }
    let candidate_names: BTreeSet<String> = candidates
        .iter()
        .filter_map(|&net| current.board.rules.nets.get(net).map(|n| n.name.clone()))
        .collect();
    let mut owners = Vec::<NetNo>::new();
    let mut seen = BTreeSet::<NetNo>::new();
    let anchors: Vec<[f64; 2]> = [source_xy, target_xy].into_iter().flatten().collect();
    let step = report.grid_step_mm;
    if step.is_finite() && step > 0.0 && !anchors.is_empty() {
        let radius_cells = (2.0 / step) as usize;
        let radius_mm = radius_cells.max(4) as f64 * step;
        let near_cells = report
            .blocked_cells
            .iter()
            .copied()
            .filter(|(gx, gy, _)| {
                let center = [*gx as f64 * step, *gy as f64 * step];
                anchors.iter().any(|anchor| {
                    (center[0] - anchor[0]).hypot(center[1] - anchor[1]) <= radius_mm
                })
            })
            .collect();
        append_ranked_hints(
            current,
            target,
            &candidate_names,
            report,
            near_cells,
            target_xy,
            source_xy,
            &mut owners,
            &mut seen,
        );
    }
    if owners.len() < 6 {
        append_ranked_hints(
            current,
            target,
            &candidate_names,
            report,
            report.blocked_cells.clone(),
            target_xy,
            source_xy,
            &mut owners,
            &mut seen,
        );
    }
    // Endpoint-ring fallback follows source board trace and polyline order.
    for anchor in anchors {
        for key in current.board.get_traces() {
            let item = current.board.item(key);
            let trace = item.trace();
            let points = trace.polyline().corners();
            for pair in points.windows(2) {
                let a = pair[0].to_float();
                let b = pair[1].to_float();
                let a_mm = [a.x / current.units_per_mm, a.y / current.units_per_mm];
                let b_mm = [b.x / current.units_per_mm, b.y / current.units_per_mm];
                if (a_mm[0].min(b_mm[0]) - 1.0) > anchor[0]
                    || anchor[0] > (a_mm[0].max(b_mm[0]) + 1.0)
                    || (a_mm[1].min(b_mm[1]) - 1.0) > anchor[1]
                    || anchor[1] > (a_mm[1].max(b_mm[1]) + 1.0)
                    || crate::board_adapter::point_segment_distance(anchor, a_mm, b_mm) > 1.0
                {
                    continue;
                }
                for &net in item.net_numbers() {
                    if candidates.contains(&net) && seen.insert(net) {
                        owners.push(net);
                    }
                }
            }
        }
    }
    if !owners.is_empty() {
        let recorded = state.preexisting_hints.entry(target).or_default();
        for net in owners {
            if !recorded.contains(&net) {
                recorded.push(net);
            }
        }
    }
    Ok(())
}

fn append_ranked_hints(
    current: &BoardAdapter,
    target: NetNo,
    candidate_names: &BTreeSet<String>,
    report: &braid_connect::RungReport,
    cells: Vec<(i32, i32, usize)>,
    target_xy: Option<[f64; 2]>,
    source_xy: Option<[f64; 2]>,
    owners: &mut Vec<NetNo>,
    seen: &mut BTreeSet<NetNo>,
) {
    if cells.is_empty() || owners.len() >= 6 {
        return;
    }
    let mut subset = report.clone();
    subset.blocked_cells = cells;
    if let Ok(ranked) = rank_victims(
        current,
        target,
        candidate_names,
        &subset,
        target_xy,
        source_xy,
    ) {
        for name in ranked {
            let Ok(net) = current.net_number(&name) else { continue };
            if seen.insert(net) {
                owners.push(net);
                if owners.len() >= 6 {
                    break;
                }
            }
        }
    }
}

/// Promote the source `_via_unblock_blame` keystone to the FRONT of the ranked
/// victims (`single_ended_loop.py:1034..1071`). Frontier attribution structurally
/// cannot see the copper beneath a via's pad, so a boxed-pad failure names the
/// blocking foreign net explicitly and the ladder rips that net FIRST.
///
/// A blame net is honoured ONLY when it is THIS-RUN eligible: `eligible` is
/// already the `lane_copper` delta set with the `target`, its rip ancestry, the
/// `#622` restore-protected nets and `protected` nets removed, so a static Pin,
/// imported/original file Cu, a protected net or the target itself can never be
/// promoted or ripped. Names are canonicalised by native net id (ascending) and
/// deduplicated; an empty or fully-filtered blame leaves `order` unchanged.
fn promote_blame(
    board: &BoardAdapter,
    eligible: &BTreeSet<String>,
    blame: &BTreeSet<NetNo>,
    order: &mut Vec<String>,
) {
    let mut promoted: Vec<(NetNo, String)> = blame
        .iter()
        .filter_map(|&net| {
            board
                .board
                .rules
                .nets
                .get(net)
                .map(|entry| (net, entry.name.clone()))
        })
        .filter(|(_, name)| eligible.contains(name))
        .collect();
    promoted.sort_by_key(|(net, _)| *net);
    let mut front: Vec<String> = Vec::new();
    let mut promoted_names: BTreeSet<String> = BTreeSet::new();
    for (_, name) in promoted {
        if promoted_names.insert(name.clone()) {
            front.push(name);
        }
    }
    if front.is_empty() {
        return;
    }
    order.retain(|name| !promoted_names.contains(name));
    for name in front.into_iter().rev() {
        order.insert(0, name);
    }
}

/// Source-directed rip-up and retry for one failed `target` endpoint route.
///
/// `frontier` is the constrained-router report from the failed attempt. The
/// ladder reuses the *latest* report (the initial frontier, then each fresh
/// `retry` report) to attribute blocking nets against the *latest stripped*
/// trial board, then tries N = 1..=3 victims. Each trial **clones** `current`,
/// strips the victim nets and restores their `baseline` copper, then asks
/// `retry` to land the target on that board. A strip/restore *refusal*
/// (multi-net/shared copper) is a failed candidate: the atomic clone leaves
/// `current` untouched and the ladder moves on — never an error that aborts the
/// whole job. A success must first re-prove `baseline.assert_invariants`
/// (strict, protected) before it is accepted, and that error is never masked.
///
/// The once-only slot-0 guard (`single_ended_loop.py:1118..1143`) is preserved:
/// when the re-analysis after N = 1's failure — which must have ripped *exactly
/// one* victim — names a *different* top pick, that pick is promoted and retried
/// alone before the ladder extends to pairs.
///
/// A retry that yields no fresh frontier does NOT terminate the ladder
/// (`single_ended_loop.py:1087`, `:1563..1596`): the last valid report (or the
/// initial frontier / standing order) is kept and N advances.
///
/// The source `_via_unblock_blame` keystone (`single_ended_loop.py:1034..1071`)
/// is consumed once from `state` and promoted to the FRONT of the ranked
/// victims (see [`promote_blame`]) so the named blocking net is ripped first.
/// The `retry` closure's third tuple element is the keystone of a FRESH retry
/// report when the backend emitted one: the newest such blame replaces the
/// standing one for later re-rankings, while an absent key leaves it untouched.
///
/// On success the victims are recorded in `state` (successful-commit history,
/// victim->attacker rip ancestry, and the driver's deduplicated reroute queue)
/// and returned typed in [`RipAccepted`].
pub(crate) fn try_source_ripup<T, F>(
    current: &BoardAdapter,
    baseline: &BoardAdapter,
    target: NetNo,
    frontier: &braid_connect::RungReport,
    protected: &BTreeSet<String>,
    state: &mut RecoveryState,
    mut retry: F,
) -> Result<Option<RipAccepted<T>>, String>
where
    F: FnMut(
        &BoardAdapter,
    ) -> Result<
        (
            Option<(BoardAdapter, T)>,
            Option<braid_connect::RungReport>,
            // Newest retry keystone, `Some` only when the fresh report carried
            // the private `_via_unblock_blame` key.
            Option<BTreeSet<NetNo>>,
        ),
        String,
    >,
{
    // Source `rip_exclude_set(state, target)`: the target, its rip ancestry and
    // the #622 restore-protected nets are never attributed, never ripped.
    let excluded = state.rip_exclude_set(target);
    let nested = state.nested;
    let authorization = if nested {
        state.nested_authorized_nets.clone()
    } else {
        BTreeSet::new()
    };
    let epoch_start = state.epoch_start.clone();
    let effective_baseline = epoch_start.as_deref().unwrap_or(baseline);
    let hint_board = effective_baseline;
    let target_xy = state.attempt_target_xy;
    let source_xy = state.attempt_source_xy;
    let mut standing_blame = state.take_attempt_blame();
    let eligible = this_run_eligible(
        current,
        effective_baseline,
        target,
        protected,
        &excluded,
        nested,
        &authorization,
    )?;
    let hint_net_ids: BTreeSet<NetNo> = if nested {
        state.epoch_existing_nets.clone()
    } else {
        hint_board
            .board
            .get_traces()
            .into_iter()
            .chain(hint_board.board.get_vias())
            .flat_map(|key| {
                hint_board
                    .board
                    .item(key)
                    .net_numbers()
                    .iter()
                    .copied()
                    .collect::<Vec<_>>()
            })
            .collect()
    };
    record_preexisting_hints(
        current,
        hint_board,
        &hint_net_ids,
        target,
        protected,
        nested,
        &authorization,
        frontier,
        target_xy,
        source_xy,
        state,
    )?;
    if eligible.is_empty() {
        return Ok(None);
    }
    // Default order from the ported frontier attribution. An empty frontier is
    // NOT terminal here: the source keystone below is structurally invisible to
    // this ranking, so the ladder may still have one named net to rip.
    let mut order = rank_victims(current, target, &eligible, frontier, target_xy, source_xy)?;
    // ORIGINAL source keystone promotion (`single_ended_loop.py:1034..1071`):
    // consume the edge's via-unblock blame (the pop resets it) and promote the
    // nameable blocking net(s) to the FRONT of the ranked victims.
    promote_blame(current, &eligible, &standing_blame, &mut order);
    if order.is_empty() {
        return Ok(None);
    }

    // Source `last_retry_blocked_cells = blocked_cells`: the standing report
    // starts as the initial frontier and is replaced only by a fresh retry
    // report. It is never `None`, so a missing fresh report keeps the last valid
    // one rather than terminating the ladder.
    let mut last_report = frontier.clone();
    // The board the standing report was produced on: the untouched `current`
    // until the first trial, then that trial's stripped clone. Re-ranking uses
    // this, not `current` (the source re-attributes against the ripped board).
    let mut last_trial: Option<BoardAdapter> = None;
    // Source `len(ripped_items)`: the number of victims actually torn by the most
    // recent attempt, which gates the slot-0 guard on exactly one.
    let mut ripped_count = 0usize;
    let mut slot0_guard_used = false;
    // Newest keystone delivered by a fresh retry report (`Some` only when that
    // report carried the private `_via_unblock_blame` key). It supersedes the
    // standing blame on the next re-ranking; an absent key never clears it.
    let mut pending_blame: Option<BTreeSet<NetNo>> = None;
    let mut n = 0usize;
    while n < MAX_RIPUP {
        n += 1;
        // N > 1: re-attribute from the LAST retry's blocked cells, against the
        // LAST stripped trial board, and promote the freshest top pick
        // (`single_ended_loop.py:1087..1154`).
        if n > 1 {
            let ranked_board = last_trial.as_ref().unwrap_or(current);
            let mut fresh =
                rank_victims(ranked_board, target, &eligible, &last_report, target_xy, source_xy)?;
            // Keep the NEWEST retry blame when the fresh report named one; a
            // report without the key leaves the standing blame intact, so a
            // stale keystone is never resurrected and a fresh one is never lost.
            if let Some(blame) = pending_blame.take() {
                standing_blame = blame;
            }
            promote_blame(ranked_board, &eligible, &standing_blame, &mut fresh);
            let already: BTreeSet<String> = order.iter().take(n - 1).cloned().collect();
            let Some(next) = fresh.iter().find(|name| !already.contains(*name)).cloned() else {
                // "No additional rippable blockers from retry analysis".
                break;
            };
            if n == 2
                && !slot0_guard_used
                && ripped_count == 1
                && order.first().is_some_and(|slot0| *slot0 != next)
            {
                // Slot-0 guard (audit, structural): the ladder only grows a
                // PREFIX around slot 0, so a refuted first pick poisons every
                // combo. Release the singleton and retry the fresh pick ALONE
                // once before extending to pairs.
                slot0_guard_used = true;
                order.retain(|name| name != &next);
                order.insert(0, next);
                ripped_count = 0;
                n = 0;
                continue;
            }
            order.retain(|name| name != &next);
            if n - 1 < order.len() {
                order.insert(n - 1, next);
            } else {
                order.push(next);
            }
        }
        if n > order.len() {
            break; // Not enough blockers to rip.
        }
        let victims: Vec<String> = order[..n].to_vec();
        let victim_nos: Vec<NetNo> = victims
            .iter()
            .map(|name| current.net_number(name))
            .collect::<Result<_, _>>()?;
        if state.already_tried(target, &victim_nos) {
            ripped_count = 0;
            continue; // Rip-history gate (#376): this combo already succeeded.
        }

        // Custody: clone, tear this-run copper, restore the input copper. The
        // original `current` is never mutated, and a strip/restore refusal
        // (multi-net/shared copper) is a failed candidate the ladder skips --
        // not an error that aborts the whole job.
        let mut trial = current.clone();
        let victim_set: BTreeSet<String> = victims.iter().cloned().collect();
        if native_execution::strip_nets(&mut trial, &victim_set).is_err()
            || native_execution::restore_nets(baseline, &mut trial, &victim_set).is_err()
        {
            ripped_count = 0;
            continue;
        }
        ripped_count = victim_nos.len();
        let (success, fresh, fresh_blame) = retry(&trial)?;
        if fresh_blame.is_some() {
            pending_blame = fresh_blame;
        }
        if let Some((board, target_data)) = success {
            // An accepted in-run board must still satisfy the strict, protected
            // baseline invariants; a real violation is surfaced, not masked.
            board.assert_invariants(baseline, protected, true)?;
            state.record_commit(target, victim_nos.clone(), current);
            return Ok(Some(RipAccepted {
                board,
                target_data,
                victims: victim_nos,
            }));
        }
        // No fresh frontier from this retry: keep the last valid report (and the
        // trial it was produced on) and advance to the next N.
        if let Some(report) = fresh {
            record_preexisting_hints(
                current,
                hint_board,
                &hint_net_ids,
                target,
                protected,
                nested,
                &authorization,
                &report,
                target_xy,
                source_xy,
                state,
            )?;
            last_report = report;
        }
        last_trial = Some(trial);
    }
    Ok(None)
}

#[cfg(test)]
#[path = "terminal_restore_tests.rs"]
mod terminal_restore_tests;

#[cfg(test)]
#[path = "nested_recovery_tests.rs"]
mod nested_recovery_tests;
