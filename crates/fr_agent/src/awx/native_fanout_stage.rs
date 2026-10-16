//! Guarded, one-pass adapter around the native single-pin fanout operation.

use crate::board_adapter::{BoardAdapter, NewClearanceViolation};
use fr_engine::{
    autoroute::AutorouteControl,
    board::{AutorouteAttemptState, ItemKey, ItemSet},
    datastructures::{StopToken, TimeLimit},
    drc::NetIncompletes,
    ids::{FixedState, ItemId},
    pipeline::{autorouter::BatchAutorouter, fanout::ordered_fanout_pins, PassCounters},
};
use fr_settings::RouterSettings;
use serde::{Deserialize, Serialize};
use std::{
    collections::{BTreeMap, BTreeSet, HashSet},
    panic::{catch_unwind, AssertUnwindSafe},
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc, Arc,
    },
    thread,
    time::{Duration, Instant},
};

#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct NativeFanoutConfig {
    pub enabled: bool,
    pub budget_ms: u64,
    pub max_pin_ms: u64,
    pub pin_sorting_order: String,
    pub fine_grid_fallback: bool,
}

impl Default for NativeFanoutConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            budget_ms: 30_000,
            max_pin_ms: 1_000,
            pin_sorting_order: "surroundings_density".into(),
            fine_grid_fallback: false,
        }
    }
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct NativeFanoutSummary {
    pub enabled: bool,
    pub attempts: usize,
    pub accepted_direct: usize,
    pub accepted_via: usize,
    pub declined: usize,
    pub guard_rejected: usize,
    pub elapsed_ms: u64,
    pub timed_out: bool,
    pub reasons: Vec<String>,
    pub failed_pins: Vec<NativeFanoutFailure>,
}

#[derive(Clone, Debug, Serialize)]
pub struct NativeFanoutFailure {
    pub pin_id: i32,
    pub net: i32,
    pub net_name: String,
    pub reason: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct NativeRescueConfig {
    pub enabled: bool,
    pub budget_ms: u64,
    pub max_connection_ms: u64,
    pub post_grid_budget_ms: u64,
    pub multi_start: usize,
    pub stage: NativeRescueStage,
    pub post_awx_budget_ms: u64,
}

#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum NativeRescueStage {
    BeforeAwx,
    #[default]
    AfterAwx,
}

impl Default for NativeRescueConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            budget_ms: 30_000,
            max_connection_ms: 1_000,
            post_grid_budget_ms: 30_000,
            multi_start: 1,
            stage: NativeRescueStage::AfterAwx,
            post_awx_budget_ms: 0,
        }
    }
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct NativeRescueNetSummary {
    pub net: i32,
    pub name: String,
    pub before: usize,
    pub after: usize,
    pub attempts: usize,
    pub accepted: usize,
    pub timeouts: usize,
    pub last_from_item_id: Option<i32>,
    pub last_to_item_id: Option<i32>,
    pub last_reject_reason: Option<String>,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct NativeRescueSummary {
    pub enabled: bool,
    pub attempts: usize,
    pub accepted: usize,
    pub guard_rejected: usize,
    pub native_failed: usize,
    pub timed_out: bool,
    pub elapsed_ms: u64,
    pub connection_timeouts: usize,
    pub per_net: Vec<NativeRescueNetSummary>,
    pub attempt_details: Vec<NativeRescueAttemptSummary>,
    pub batch_pass_no: i32,
    pub batch_enhancements: bool,
    pub batch_suppress_net_rip: bool,
    pub batch_remove_unconnected_vias: bool,
    pub batch_regression_rollbacks: i32,
    pub batch_routed: i32,
    pub batch_not_routed: i32,
    pub batch_skipped: i32,
    pub batch_ripped: i32,
    pub batch_before: usize,
    pub batch_after: usize,
    pub batch_accepted: bool,
    pub batch_guard_reason: Option<String>,
    pub batch_elapsed_ms: u64,
    pub batch_timed_out: bool,
    pub batch_per_net: Vec<NativeBatchNetSummary>,
    pub batch_source_per_net: Vec<NativeBatchNetSummary>,
    pub batch_best_per_net: Vec<NativeBatchNetSummary>,
    pub batch_final_trial_per_net: Vec<NativeBatchNetSummary>,
    pub batch_drc_records: Vec<NewClearanceViolation>,
    pub batch_passes: Vec<NativeBatchPassSummary>,
    pub multi_start_requested: usize,
    pub multi_start_actual_workers: usize,
    pub multi_start_winning_variant: Option<usize>,
    pub multi_start_variants: Vec<NativeRescueVariantSummary>,
    pub reasons: Vec<String>,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct NativeRescueVariantSummary {
    pub variant: usize,
    pub seed: Option<i64>,
    pub before: usize,
    pub after: usize,
    pub accepted: bool,
    pub guard_reason: Option<String>,
    pub elapsed_ms: u64,
    pub timed_out: bool,
    pub error: Option<String>,
}

struct NativeBatchWorkerResult {
    board: BoardAdapter,
    summary: NativeRescueSummary,
    reasons: BTreeMap<String, usize>,
    after: usize,
    elapsed_ms: u64,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct NativeBatchPassSummary {
    pub pass_no: i32,
    pub before: usize,
    pub after: usize,
    pub best_before: usize,
    pub best_after: usize,
    pub routed: i32,
    pub not_routed: i32,
    pub skipped: i32,
    pub ripped: i32,
    pub accepted: bool,
    pub guard_reason: Option<String>,
    pub source_net_incompletes: BTreeMap<i32, usize>,
    pub best_net_incompletes: BTreeMap<i32, usize>,
    pub candidate_net_incompletes: BTreeMap<i32, usize>,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct NativeRescueAttemptSummary {
    pub net: i32,
    pub net_name: String,
    pub from_item_id: i32,
    pub to_item_id: i32,
    pub from_connected_len: usize,
    pub to_connected_len: usize,
    pub time_limit_fired: bool,
    pub native_state: String,
    pub reason: Option<String>,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct NativeBatchNetSummary {
    pub net: i32,
    pub name: String,
    pub before: usize,
    pub after: usize,
}

fn sorter(value: &str) -> Option<&'static str> {
    match value {
        "inner_first" => Some("inner_first"),
        "outer_first" => Some("outer_first"),
        "distance_to_closest_on_net" => Some("distanceToClosestOnNet"),
        "surroundings_density" => Some("surroundingsDensity"),
        _ => None,
    }
}

fn incomplete_count(board: &BoardAdapter, net: i32) -> usize {
    NetIncompletes::new(&board.board, net, &board.board.get_connectable_items(net))
        .incompletes
        .len()
}

fn item_ids(board: &BoardAdapter) -> BTreeSet<i32> {
    board
        .board
        .get_items()
        .into_iter()
        .map(|key| board.board.item(key).id().0)
        .collect()
}

fn connected_ids(board: &BoardAdapter, key: ItemKey, net: i32) -> BTreeSet<i32> {
    board
        .board
        .connected_set(key, net, false)
        .iter()
        .map(|k| board.board.item(k).id().0)
        .collect()
}

fn board_nets(board: &BoardAdapter) -> BTreeSet<i32> {
    board
        .board
        .get_items()
        .into_iter()
        .flat_map(|key| board.board.item(key).net_numbers().iter().copied())
        .collect()
}

fn connected_partitions(board: &BoardAdapter) -> BTreeMap<i32, Vec<BTreeSet<i32>>> {
    let nets = board_nets(board);
    nets.into_iter()
        .map(|net| {
            let groups = board
                .board
                .get_connected_sets(net)
                .into_iter()
                .map(|set| {
                    set.iter()
                        .filter_map(|key| {
                            let item = board.board.item(key);
                            item.is_pin().then_some(item.id().0)
                        })
                        .collect::<BTreeSet<_>>()
                })
                .filter(|group| !group.is_empty())
                .collect();
            (net, groups)
        })
        .collect()
}

fn guard_candidate(
    candidate: &BoardAdapter,
    source: &BoardAdapter,
    protected: &BTreeSet<String>,
    pin: ItemKey,
    net: i32,
    original_connected: &BTreeSet<i32>,
    old_ids: &BTreeSet<i32>,
    baseline_partitions: &BTreeMap<i32, Vec<BTreeSet<i32>>>,
    baseline_incompletes: &BTreeMap<i32, usize>,
    min_ring: f64,
    source_ids: &BTreeSet<i32>,
    rescue_target_before: Option<usize>,
) -> Result<bool, String> {
    candidate.assert_invariants(source, protected, true)?;
    let new_keys: Vec<_> = candidate
        .board
        .get_items()
        .into_iter()
        .filter(|&key| !old_ids.contains(&candidate.board.item(key).id().0))
        .collect();
    let after_connected = connected_ids(candidate, pin, net);
    for key in candidate.board.get_items() {
        let item = candidate.board.item(key);
        if source_ids.contains(&item.id().0) || (!item.is_trace() && !item.is_via()) {
            continue;
        }
        if item.net_count() != 1 {
            return Err("generated copper is shared or has no net".into());
        }
        let item_net = item.net_number(0);
        let net_name = source
            .board
            .rules
            .nets
            .get(item_net)
            .map(|n| n.name.as_str())
            .unwrap_or("");
        if protected.contains(net_name) {
            continue;
        }
        let class_id = source
            .board
            .rules
            .nets
            .get(item_net)
            .ok_or("generated copper net is missing from source board")?
            .get_net_class();
        let net_class = &source.board.rules.net_classes[class_id];
        if item.is_trace() {
            let layer = item.trace().layer();
            if item.trace().half_width() < net_class.get_trace_half_width(layer) {
                return Err(format!(
                    "generated trace below source width on layer {layer}"
                ));
            }
        } else {
            let via = item.as_via().ok_or("generated via has no via record")?;
            let rule = net_class
                .get_via_rule()
                .ok_or("source net class has no via rule")?;
            let allowed = &source.board.rules.via_rules[rule];
            let info = source
                .board
                .rules
                .via_infos
                .iter()
                .find(|&id| {
                    allowed.contains(id)
                        && source.board.rules.via_infos[id].get_padstack() == via.padstack_no()
                })
                .ok_or("generated via is outside source net-class via rule")?;
            let source_padstack_no = source.board.rules.via_infos[info].get_padstack();
            let candidate_padstack = candidate
                .board
                .library
                .padstacks
                .get(via.padstack_no())
                .ok_or("generated via padstack is missing")?;
            let source_padstack = source
                .board
                .library
                .padstacks
                .get(source_padstack_no)
                .ok_or("source ViaInfo padstack is missing")?;
            let candidate_drill = candidate_padstack
                .get_explicit_drill_radius()
                .ok_or("generated via has no explicit drill")?;
            let source_drill = source_padstack
                .get_explicit_drill_radius()
                .ok_or("source ViaInfo has no explicit drill")?;
            if (candidate_drill - source_drill).abs() > 1.0 {
                return Err("generated via drill differs from source ViaInfo".into());
            }
            let min_diameter = (item.first_layer(&candidate.board)
                ..=item.last_layer(&candidate.board))
                .filter_map(|layer| candidate_padstack.get_shape(layer))
                .map(|shape| shape.max_width())
                .fold(f64::INFINITY, f64::min);
            if !min_diameter.is_finite()
                || (min_diameter - 2.0 * candidate_drill) / 2.0 + 1e-9
                    < min_ring * candidate.units_per_mm
            {
                return Err("generated via fails minimum annular-ring guard".into());
            }
            if via.attach_allowed != source.board.rules.via_infos[info].attach_smd_allowed() {
                return Err("generated via attachment flag differs from source ViaInfo".into());
            }
        }
    }

    let connected_pin_gained = candidate.board.get_pins().into_iter().any(|key| {
        let item = candidate.board.item(key);
        item.contains_net(net)
            && after_connected.contains(&item.id().0)
            && !original_connected.contains(&item.id().0)
    });
    let connected_new_via = new_keys.iter().any(|&key| {
        let item = candidate.board.item(key);
        item.is_via() && item.contains_net(net) && after_connected.contains(&item.id().0)
    });
    let target_after = incomplete_count(candidate, net);
    let rescue_decreased = rescue_target_before.is_some_and(|before| target_after < before);
    if let Some(before) = rescue_target_before {
        if target_after >= before {
            return Err(format!(
                "target net incomplete count did not decrease ({before} -> {target_after})"
            ));
        }
    }
    if !connected_new_via && !connected_pin_gained && !rescue_decreased {
        return Err("native route added no endpoint-connected evidence".into());
    }

    for key in candidate.board.get_items() {
        let item = candidate.board.item(key);
        if source_ids.contains(&item.id().0) || (!item.is_trace() && !item.is_via()) {
            continue;
        }
        for &item_net in item.net_numbers() {
            let anchored = candidate.board.get_pins().into_iter().any(|pin_key| {
                let pin_item = candidate.board.item(pin_key);
                pin_item.contains_net(item_net)
                    && connected_ids(candidate, pin_key, item_net).contains(&item.id().0)
            });
            if !anchored {
                return Err("generated copper is not connected to a pin on its net".into());
            }
        }
    }
    let after_partitions = connected_partitions(candidate);
    for (other_net, groups) in baseline_partitions {
        let now = after_partitions
            .get(other_net)
            .ok_or("existing net connectivity disappeared")?;
        for original_group in groups {
            if !now.iter().any(|group| original_group.is_subset(group)) {
                return Err(format!(
                    "previously connected pins split on net {other_net}"
                ));
            }
        }
    }
    for (&other_net, &before) in baseline_incompletes {
        let after = incomplete_count(candidate, other_net);
        if after > before {
            return Err(format!("net {other_net} incomplete count increased"));
        }
    }
    Ok(connected_new_via)
}

fn total_incomplete_count(board: &BoardAdapter) -> usize {
    board_nets(board)
        .into_iter()
        .map(|net| incomplete_count(board, net))
        .sum()
}

fn batch_net_summaries(board: &BoardAdapter) -> Vec<NativeBatchNetSummary> {
    board_nets(board)
        .into_iter()
        .map(|net| {
            let count = incomplete_count(board, net);
            NativeBatchNetSummary {
                net,
                name: board
                    .board
                    .rules
                    .nets
                    .get(net)
                    .map(|n| n.name.clone())
                    .unwrap_or_default(),
                before: count,
                after: count,
            }
        })
        .collect()
}

fn run_batch_rip_rescue(
    board: &mut BoardAdapter,
    source: &BoardAdapter,
    protected: &BTreeSet<String>,
    source_ids: &BTreeSet<i32>,
    min_ring: f64,
    stage_deadline: Instant,
    order_seed: Option<i64>,
    summary: &mut NativeRescueSummary,
    reasons: &mut BTreeMap<String, usize>,
) {
    let started = Instant::now();
    summary.batch_before = total_incomplete_count(board);
    summary.batch_after = summary.batch_before;
    summary.batch_source_per_net = batch_net_summaries(source);
    summary.batch_best_per_net = batch_net_summaries(board);
    summary.batch_final_trial_per_net = batch_net_summaries(board);
    if summary.batch_before == 0 || Instant::now() >= stage_deadline {
        summary.batch_timed_out = Instant::now() >= stage_deadline;
        summary.batch_elapsed_ms = started.elapsed().as_millis() as u64;
        return;
    }
    let initial_per_net: BTreeMap<_, _> = board_nets(board)
        .into_iter()
        .map(|net| (net, incomplete_count(board, net)))
        .collect();
    let source_incompletes: BTreeMap<_, _> = board_nets(source)
        .into_iter()
        .map(|net| (net, incomplete_count(source, net)))
        .collect();
    let source_partitions = connected_partitions(source);
    let mut best = board.clone();
    let mut best_count = summary.batch_before;
    let mut trial = board.clone();
    let mut settings = RouterSettings::default();
    settings.set_layer_count(trial.board.layer_count() as usize);
    settings.automatic_neckdown = Some(false);
    settings.neck_width_um = Some(0.0);
    settings.strict_drc = Some(false);

    let eligible_nets: BTreeSet<_> = board_nets(&trial)
        .into_iter()
        .filter(|&net| {
            trial
                .board
                .rules
                .nets
                .get(net)
                .is_some_and(|n| !protected.contains(&n.name))
        })
        .collect();
    if eligible_nets.is_empty() {
        summary.batch_guard_reason =
            Some("no unprotected nets are eligible for native batch routing".into());
        summary.batch_elapsed_ms = started.elapsed().as_millis() as u64;
        return;
    }
    let protected_item_ids: HashSet<_> = trial
        .board
        .get_items()
        .into_iter()
        .filter_map(|key| {
            let item = trial.board.item(key);
            item.net_numbers()
                .iter()
                .any(|net| {
                    trial
                        .board
                        .rules
                        .nets
                        .get(*net)
                        .is_some_and(|n| protected.contains(&n.name))
                })
                .then_some(item.id().0)
        })
        .collect();
    let mut batch = BatchAutorouter::for_job(&trial.board, &settings);
    batch.enhancements = true;
    // Keep native one-time failed-net recovery; source/protected copper is SystemFixed.
    batch.suppress_net_rip = false;
    batch.remove_unconnected_vias = false;
    summary.batch_enhancements = batch.enhancements;
    summary.batch_suppress_net_rip = batch.suppress_net_rip;
    summary.batch_remove_unconnected_vias = batch.remove_unconnected_vias;
    batch.defer_strict_drc_to_caller = true;
    batch.pass_threads = 1;
    batch.order_seed = order_seed;
    batch.net_filter = Some(eligible_nets);
    batch.skip_items = Some(protected_item_ids);
    let ignored_incompletes = fr_engine::pipeline::stats::ignored_incomplete_count(&trial.board);
    let mut regression_rollbacks = 0i32;

    let stop = StopToken::new();
    let timer_stop = stop.clone();
    let timeout = Arc::new(AtomicBool::new(false));
    let timer_timeout = Arc::clone(&timeout);
    let (sender, receiver) = mpsc::channel::<()>();
    let wait = stage_deadline.saturating_duration_since(Instant::now());
    let timer_thread = thread::spawn(move || match receiver.recv_timeout(wait) {
        Err(mpsc::RecvTimeoutError::Timeout) => {
            timer_timeout.store(true, Ordering::Relaxed);
            timer_stop.request_stop();
            true
        }
        Ok(()) | Err(mpsc::RecvTimeoutError::Disconnected) => false,
    });

    let mut pass_no = 0i32;
    loop {
        if best_count == 0 || Instant::now() >= stage_deadline || stop.is_stop_requested() {
            break;
        }
        pass_no += 1;
        summary.batch_pass_no = pass_no;
        let before_count = total_incomplete_count(&trial);
        let before_ids = item_ids(&trial);
        let best_before = best_count;
        let best_incompletes: BTreeMap<_, _> = board_nets(&best)
            .into_iter()
            .map(|net| (net, incomplete_count(&best, net)))
            .collect();
        let best_ids = item_ids(&best);
        let mut pass_summary = NativeBatchPassSummary {
            pass_no,
            before: before_count,
            best_before,
            best_after: best_count,
            source_net_incompletes: source_incompletes.clone(),
            best_net_incompletes: best_incompletes.clone(),
            ..Default::default()
        };
        let mut pass_board = trial.clone();

        let saved_rules: Vec<_> = pass_board
            .board
            .rules
            .net_classes
            .iter()
            .map(|id| (id, pass_board.board.rules.net_classes[id].no_neckdown))
            .collect();
        let mut saved_fixed = BTreeMap::<ItemId, FixedState>::new();
        for key in pass_board.board.get_items() {
            let item = pass_board.board.item(key);
            if !item.is_trace() && !item.is_via() {
                continue;
            }
            let protected_net = item.net_numbers().iter().any(|net| {
                pass_board
                    .board
                    .rules
                    .nets
                    .get(*net)
                    .is_some_and(|n| protected.contains(&n.name))
            });
            let source_copper = source_ids.contains(&item.id().0)
                || pass_board.is_source_protected_copper(key)
                || protected_net;
            let may_unfix = !source_copper
                && item.net_count() == 1
                && item.component_no() == 0
                && item.fixed_state() != FixedState::SystemFixed;
            if source_copper || may_unfix {
                saved_fixed.insert(item.id(), item.fixed_state());
                let next_state = if source_copper {
                    FixedState::SystemFixed
                } else {
                    FixedState::Unfixed
                };
                if item.fixed_state() != next_state {
                    pass_board.board.item_mut(key).set_fixed_state(next_state);
                }
            }
        }
        for (class, _) in &saved_rules {
            pass_board.board.rules_mut().net_classes[*class].no_neckdown = true;
        }

        let (continued, counters): (bool, PassCounters) =
            batch.autoroute_pass(&mut pass_board.board, &settings, pass_no, &stop);

        for (class, previous) in &saved_rules {
            pass_board.board.rules_mut().net_classes[*class].no_neckdown = *previous;
        }
        let new_ids: BTreeSet<_> = item_ids(&pass_board)
            .difference(&before_ids)
            .copied()
            .collect();
        for key in pass_board.board.get_items() {
            let item = pass_board.board.item(key);
            let id = item.id();
            if let Some(previous) = saved_fixed.get(&id) {
                pass_board.board.item_mut(key).set_fixed_state(*previous);
            } else if new_ids.contains(&id.0)
                && (item.is_trace() || item.is_via())
                && item.net_count() == 1
                && item.component_no() == 0
                && !pass_board.is_source_protected_copper(key)
                && !item.net_numbers().iter().any(|net| {
                    pass_board
                        .board
                        .rules
                        .nets
                        .get(*net)
                        .is_some_and(|n| protected.contains(&n.name))
                })
            {
                pass_board
                    .board
                    .item_mut(key)
                    .set_fixed_state(FixedState::Unfixed);
            }
        }

        summary.batch_routed += counters.routed;
        summary.batch_not_routed += counters.not_routed;
        summary.batch_skipped += counters.skipped;
        summary.batch_ripped += counters.ripped;
        let candidate_incompletes: BTreeMap<_, _> = board_nets(&pass_board)
            .into_iter()
            .map(|net| (net, incomplete_count(&pass_board, net)))
            .collect();
        let after_count: usize = candidate_incompletes.values().sum();
        pass_summary.candidate_net_incompletes = candidate_incompletes.clone();
        pass_summary.after = after_count;
        pass_summary.routed = counters.routed;
        pass_summary.not_routed = counters.not_routed;
        pass_summary.skipped = counters.skipped;
        pass_summary.ripped = counters.ripped;
        if let Err(reason) = pass_board.assert_invariants(source, protected, true) {
            if reason == "new full-board clearance violation" {
                summary.batch_drc_records = pass_board
                    .new_clearance_violations_since(source)
                    .records
                    .into_iter()
                    .take(3)
                    .collect();
            }
            summary.batch_guard_reason = Some(reason.clone());
            pass_summary.guard_reason = Some(reason.clone());
            summary.guard_rejected += 1;
            *reasons
                .entry(format!("batch pass {pass_no}: {reason}"))
                .or_default() += 1;
            pass_summary.best_after = best_count;
            summary.batch_passes.push(pass_summary);
            break;
        } else {
            if continued
                && BatchAutorouter::should_rollback_regression(
                    pass_no,
                    after_count as i32,
                    best_count as i32,
                    ignored_incompletes,
                    regression_rollbacks,
                )
            {
                trial = best.clone();
                batch.reset_anti_oscillation_state();
                regression_rollbacks += 1;
                batch.suppress_net_rip = true;
                summary.batch_suppress_net_rip = true;
                summary.batch_regression_rollbacks = regression_rollbacks;
                let reason = format!(
                    "pass {pass_no} regressed to {after_count} incomplete connections; restored best board ({best_count})"
                );
                summary.batch_guard_reason = Some(reason.clone());
                pass_summary.guard_reason = Some(reason.clone());
                pass_summary.best_after = best_count;
                *reasons
                    .entry(format!("batch pass {pass_no}: {reason}"))
                    .or_default() += 1;
                summary.batch_passes.push(pass_summary);
                continue;
            }
            let decreased_net = best_incompletes.iter().find_map(|(&net, &before)| {
                (candidate_incompletes.get(&net).copied().unwrap_or(0) < before).then_some(net)
            });
            let Some(reduced_net) = decreased_net else {
                let reason = format!("pass {pass_no} reduced no best-board net incomplete count");
                summary.batch_guard_reason = Some(reason.clone());
                pass_summary.guard_reason = Some(reason.clone());
                *reasons.entry(reason).or_default() += 1;
                trial = pass_board;
                pass_summary.best_after = best_count;
                summary.batch_passes.push(pass_summary);
                if !continued {
                    break;
                }
                continue;
            };
            let source_pin_id = source.board.get_pins().into_iter().find_map(|key| {
                let item = source.board.item(key);
                if !item.contains_net(reduced_net) {
                    return None;
                }
                Some(item.id())
            });
            let Some(source_pin_id) = source_pin_id else {
                let reason = format!(
                    "pass {pass_no} reduced net {reduced_net} without a surviving source pin"
                );
                summary.batch_guard_reason = Some(reason.clone());
                pass_summary.guard_reason = Some(reason.clone());
                *reasons.entry(reason).or_default() += 1;
                trial = pass_board;
                pass_summary.best_after = best_count;
                summary.batch_passes.push(pass_summary);
                if !continued {
                    break;
                }
                continue;
            };
            let Some(best_pin) = best.board.get_item(source_pin_id) else {
                let reason = format!("best board lost source pin {}", source_pin_id.0);
                summary.batch_guard_reason = Some(reason.clone());
                pass_summary.guard_reason = Some(reason.clone());
                *reasons.entry(reason).or_default() += 1;
                trial = pass_board;
                pass_summary.best_after = best_count;
                summary.batch_passes.push(pass_summary);
                if !continued {
                    break;
                }
                continue;
            };
            let Some(candidate_pin) = pass_board.board.get_item(source_pin_id) else {
                let reason = format!("pass {pass_no} lost source pin {}", source_pin_id.0);
                summary.batch_guard_reason = Some(reason.clone());
                pass_summary.guard_reason = Some(reason.clone());
                *reasons.entry(reason).or_default() += 1;
                trial = pass_board;
                pass_summary.best_after = best_count;
                summary.batch_passes.push(pass_summary);
                if !continued {
                    break;
                }
                continue;
            };
            let guard_original_connected = connected_ids(&best, best_pin, reduced_net);
            let guard = guard_candidate(
                &pass_board,
                source,
                protected,
                candidate_pin,
                reduced_net,
                &guard_original_connected,
                &best_ids,
                &source_partitions,
                &source_incompletes,
                min_ring,
                source_ids,
                best_incompletes.get(&reduced_net).copied(),
            );
            match guard {
                Ok(_) if after_count < best_count => {
                    best_count = after_count;
                    best = pass_board.clone();
                    summary.batch_accepted = true;
                    summary.batch_guard_reason = None;
                    pass_summary.accepted = true;
                }
                Ok(_) => {
                    summary.batch_guard_reason = Some(format!(
                        "pass {pass_no} did not reduce best-board incomplete count ({best_before} -> {after_count})"
                    ));
                    pass_summary.guard_reason = summary.batch_guard_reason.clone();
                }
                Err(reason) => {
                    summary.batch_guard_reason = Some(reason.clone());
                    pass_summary.guard_reason = Some(reason.clone());
                    summary.guard_rejected += 1;
                    *reasons
                        .entry(format!("batch pass {pass_no}: {reason}"))
                        .or_default() += 1;
                }
            }
            trial = pass_board;
        }
        pass_summary.best_after = best_count;
        summary.batch_passes.push(pass_summary);
        if !continued {
            if summary.batch_guard_reason.is_none() {
                let reason =
                    format!("pass {pass_no} native batch reported no further routing work");
                summary.batch_guard_reason = Some(reason.clone());
                *reasons.entry(reason).or_default() += 1;
            }
            break;
        }
    }

    drop(sender);
    summary.batch_timed_out =
        timer_thread.join().unwrap_or(false) || timeout.load(Ordering::Relaxed);
    summary.batch_elapsed_ms = started.elapsed().as_millis() as u64;
    if summary.batch_accepted {
        *board = best;
        summary.accepted += summary.batch_before.saturating_sub(best_count);
    }
    summary.batch_after = total_incomplete_count(board);
    summary.batch_best_per_net = batch_net_summaries(board);
    summary.batch_final_trial_per_net = batch_net_summaries(&trial);
    summary.batch_per_net = initial_per_net
        .into_iter()
        .map(|(net, before)| NativeBatchNetSummary {
            net,
            name: board
                .board
                .rules
                .nets
                .get(net)
                .map(|n| n.name.clone())
                .unwrap_or_default(),
            before,
            after: incomplete_count(board, net),
        })
        .collect();
    if summary.batch_timed_out {
        *reasons
            .entry("native batch rescue deadline fired".into())
            .or_default() += 1;
    }
}

fn run_batch_rip_multi_start(
    board: &mut BoardAdapter,
    source: &BoardAdapter,
    protected: &BTreeSet<String>,
    source_ids: &BTreeSet<i32>,
    min_ring: f64,
    stage_deadline: Instant,
    requested: usize,
    summary: &mut NativeRescueSummary,
    reasons: &mut BTreeMap<String, usize>,
) {
    summary.multi_start_requested = requested;
    let before = total_incomplete_count(board);
    if before == 0 || Instant::now() >= stage_deadline {
        summary.batch_before = before;
        summary.batch_after = before;
        summary.batch_guard_reason = Some(if before == 0 {
            "native batch rescue skipped: board has no open connections".into()
        } else {
            "native batch rescue skipped: stage deadline already expired".into()
        });
        summary.batch_timed_out = Instant::now() >= stage_deadline;
        summary.batch_per_net = batch_net_summaries(board);
        summary.batch_source_per_net = batch_net_summaries(source);
        summary.batch_best_per_net = batch_net_summaries(board);
        summary.batch_final_trial_per_net = batch_net_summaries(board);
        summary.multi_start_actual_workers = 0;
        summary.multi_start_winning_variant = None;
        return;
    }
    if requested == 1 {
        let started = Instant::now();
        run_batch_rip_rescue(
            board,
            source,
            protected,
            source_ids,
            min_ring,
            stage_deadline,
            None,
            summary,
            reasons,
        );
        summary.multi_start_actual_workers = 1;
        summary.multi_start_winning_variant = Some(0);
        summary
            .multi_start_variants
            .push(NativeRescueVariantSummary {
                variant: 0,
                seed: None,
                before,
                after: total_incomplete_count(board),
                accepted: summary.batch_accepted,
                guard_reason: summary.batch_guard_reason.clone(),
                elapsed_ms: started.elapsed().as_millis() as u64,
                timed_out: summary.batch_timed_out,
                error: None,
            });
        return;
    }

    let entry_board = board.clone();
    let entry_summary = summary.clone();
    let entry_reasons = reasons.clone();
    let entry_count = total_incomplete_count(&entry_board);
    let mut best_board = entry_board.clone();
    let mut best_count = entry_count;
    let mut best_summary = entry_summary.clone();
    let mut best_reasons = entry_reasons.clone();
    let mut winning_variant = None;
    let mut handles = Vec::new();
    let mut variant_summaries = Vec::new();
    let mut actual_workers = 0usize;
    let mut launch_skipped = 0usize;

    for variant in 0..requested {
        if Instant::now() >= stage_deadline {
            let skipped = requested - variant;
            launch_skipped = skipped;
            for skipped_variant in variant..requested {
                variant_summaries.push(NativeRescueVariantSummary {
                    variant: skipped_variant,
                    seed: (skipped_variant > 0).then_some(0x5eed_0000_i64 + skipped_variant as i64),
                    before: entry_count,
                    after: entry_count,
                    accepted: false,
                    guard_reason: Some("stage deadline expired before worker launch".into()),
                    elapsed_ms: 0,
                    timed_out: true,
                    error: None,
                });
            }
            break;
        }
        let seed = (variant > 0).then_some(0x5eed_0000_i64 + variant as i64);
        let mut worker_board = entry_board.clone();
        let worker_source = source.clone();
        let worker_protected = protected.clone();
        let worker_source_ids = source_ids.clone();
        let mut worker_summary = entry_summary.clone();
        let mut worker_reasons = entry_reasons.clone();
        let worker_deadline = stage_deadline;
        let spawn_started = Instant::now();
        let handle = thread::Builder::new()
            .name(format!("native-rescue-{variant}"))
            .spawn(move || {
                let worker_started = Instant::now();
                let result = catch_unwind(AssertUnwindSafe(|| {
                    run_batch_rip_rescue(
                        &mut worker_board,
                        &worker_source,
                        &worker_protected,
                        &worker_source_ids,
                        min_ring,
                        worker_deadline,
                        seed,
                        &mut worker_summary,
                        &mut worker_reasons,
                    );
                    let after = total_incomplete_count(&worker_board);
                    (worker_board, worker_summary, worker_reasons, after)
                }));
                let elapsed_ms = worker_started.elapsed().as_millis() as u64;
                match result {
                    Ok((board, summary, reasons, after)) => Ok(NativeBatchWorkerResult {
                        board,
                        summary,
                        reasons,
                        after,
                        elapsed_ms,
                    }),
                    Err(_) => Err(elapsed_ms),
                }
            });
        match handle {
            Ok(handle) => {
                actual_workers += 1;
                handles.push((variant, seed, handle));
            }
            Err(error) => variant_summaries.push(NativeRescueVariantSummary {
                variant,
                seed,
                before: entry_count,
                after: entry_count,
                accepted: false,
                guard_reason: None,
                elapsed_ms: spawn_started.elapsed().as_millis() as u64,
                timed_out: false,
                error: Some(format!("failed to spawn native rescue worker: {error}")),
            }),
        }
    }

    for (variant, seed, handle) in handles {
        match handle.join() {
            Ok(Ok(worker)) => {
                variant_summaries.push(NativeRescueVariantSummary {
                    variant,
                    seed,
                    before: entry_count,
                    after: worker.after,
                    accepted: worker.summary.batch_accepted,
                    guard_reason: worker.summary.batch_guard_reason.clone(),
                    elapsed_ms: worker.elapsed_ms,
                    timed_out: worker.summary.batch_timed_out,
                    error: None,
                });
                if variant == 0 && worker.after >= best_count {
                    best_summary = worker.summary.clone();
                    best_reasons = worker.reasons.clone();
                    winning_variant = Some(0);
                }
                if worker.after < best_count {
                    best_count = worker.after;
                    best_board = worker.board;
                    best_summary = worker.summary;
                    best_reasons = worker.reasons;
                    winning_variant = Some(variant);
                }
            }
            Ok(Err(elapsed_ms)) => variant_summaries.push(NativeRescueVariantSummary {
                variant,
                seed,
                before: entry_count,
                after: entry_count,
                accepted: false,
                guard_reason: None,
                elapsed_ms,
                timed_out: false,
                error: Some("native rescue worker panicked".into()),
            }),
            Err(_) => variant_summaries.push(NativeRescueVariantSummary {
                variant,
                seed,
                before: entry_count,
                after: entry_count,
                accepted: false,
                guard_reason: None,
                elapsed_ms: 0,
                timed_out: false,
                error: Some("native rescue worker panicked before reporting completion".into()),
            }),
        }
    }
    variant_summaries.sort_by_key(|variant| variant.variant);
    if launch_skipped > 0 {
        best_summary.batch_timed_out = true;
        *best_reasons
            .entry("native rescue deadline expired before worker launch".into())
            .or_default() += launch_skipped;
    }
    *board = best_board;
    *summary = best_summary;
    *reasons = best_reasons;
    summary.multi_start_requested = requested;
    summary.multi_start_actual_workers = actual_workers;
    summary.multi_start_winning_variant = winning_variant;
    summary.multi_start_variants = variant_summaries;
}

pub fn run(
    board: &mut BoardAdapter,
    source: &BoardAdapter,
    protected: &BTreeSet<String>,
    cfg: &NativeFanoutConfig,
    deadline: Instant,
    min_ring: f64,
) -> Result<NativeFanoutSummary, String> {
    if !min_ring.is_finite() || min_ring < 0.0 {
        return Err("minimum annular ring must be finite and non-negative".into());
    }
    let Some(native_order) = sorter(&cfg.pin_sorting_order) else {
        return Err(format!(
            "unsupported native fanout pin_sorting_order: {}",
            cfg.pin_sorting_order
        ));
    };
    let mut summary = NativeFanoutSummary {
        enabled: cfg.enabled,
        ..Default::default()
    };
    let start = Instant::now();
    if !cfg.enabled {
        summary.elapsed_ms = start.elapsed().as_millis() as u64;
        return Ok(summary);
    }
    if cfg.budget_ms == 0 || cfg.max_pin_ms == 0 {
        return Err("enabled native fanout requires positive budget_ms and max_pin_ms".into());
    }
    let stage_deadline = deadline.min(start + Duration::from_millis(cfg.budget_ms));
    let mut settings = RouterSettings::default();
    settings.set_layer_count(board.board.layer_count() as usize);
    settings.fanout.pin_sorting_order = Some(native_order.into());
    settings.automatic_neckdown = Some(false);
    settings.vias_allowed = Some(true);
    settings.fanout.fallback_to_board_vias = Some(false);
    settings.fanout.ripup_allowed = Some(false);
    let pins = ordered_fanout_pins(&board.board, &settings, true);
    let source_ids = item_ids(source);
    let mut baseline_partitions = connected_partitions(board);
    let mut baseline_incompletes: BTreeMap<_, _> = board_nets(board)
        .into_iter()
        .map(|net| (net, incomplete_count(board, net)))
        .collect();
    let mut reject_reasons = BTreeMap::<String, usize>::new();
    for pin in pins {
        if Instant::now() >= stage_deadline {
            summary.timed_out = true;
            break;
        }
        let item = board.board.item(pin);
        if !item.is_pin() || item.net_count() != 1 {
            summary.declined += 1;
            continue;
        }
        let net = item.net_number(0);
        let pin_id = item.id().0;
        let net_name = board
            .board
            .rules
            .nets
            .get(net)
            .map(|n| n.name.clone())
            .unwrap_or_default();
        if protected.contains(&net_name)
            || item.first_layer(&board.board) != item.last_layer(&board.board)
            || board.board.layer_count() < 2
            || board.board.rules.net_class_of(net).get_via_rule().is_none()
        {
            summary.declined += 1;
            continue;
        }
        summary.attempts += 1;
        let original_connected = connected_ids(board, pin, net);
        let old_ids = item_ids(board);
        let mut candidate = board.clone();
        let target_class = candidate
            .board
            .rules
            .nets
            .get(net)
            .ok_or("fanout net is missing from candidate board")?
            .get_net_class();
        let saved_no_neckdown = candidate.board.rules.net_classes[target_class].no_neckdown;
        candidate.board.rules_mut().net_classes[target_class].no_neckdown = true;
        let remaining = stage_deadline.saturating_duration_since(Instant::now());
        let per_pin = remaining
            .min(Duration::from_millis(cfg.max_pin_ms))
            .as_millis()
            .clamp(1, i32::MAX as u128) as i32;
        let stop = StopToken::new();
        let outcome = candidate.board.fanout(
            pin,
            &settings,
            -1,
            Some(&stop),
            Some(TimeLimit::wall_clock(per_pin)),
        );
        candidate.board.rules_mut().net_classes[target_class].no_neckdown = saved_no_neckdown;
        if outcome.state != AutorouteAttemptState::Routed {
            summary.declined += 1;
            if !matches!(
                outcome.state,
                AutorouteAttemptState::AlreadyConnected
                    | AutorouteAttemptState::NoUnconnectedNets
                    | AutorouteAttemptState::ConnectedToPlane
                    | AutorouteAttemptState::Skipped
            ) {
                let reason = if outcome.details.trim().is_empty() {
                    format!("native {:?}", outcome.state)
                } else {
                    format!("native {:?}: {}", outcome.state, outcome.details.trim())
                };
                summary.failed_pins.push(NativeFanoutFailure {
                    pin_id,
                    net,
                    net_name: net_name.clone(),
                    reason,
                });
            }
            *reject_reasons
                .entry(format!("native {:?}", outcome.state))
                .or_default() += 1;
            continue;
        }
        match guard_candidate(
            &candidate,
            source,
            protected,
            pin,
            net,
            &original_connected,
            &old_ids,
            &baseline_partitions,
            &baseline_incompletes,
            min_ring,
            &source_ids,
            None,
        ) {
            Ok(connected_via) => {
                *board = candidate;
                summary.accepted_via += usize::from(connected_via);
                summary.accepted_direct += usize::from(!connected_via);
                baseline_partitions = connected_partitions(board);
                baseline_incompletes = baseline_incompletes
                    .keys()
                    .copied()
                    .map(|n| (n, incomplete_count(board, n)))
                    .collect();
            }
            Err(reason) => {
                summary.guard_rejected += 1;
                summary.failed_pins.push(NativeFanoutFailure {
                    pin_id,
                    net,
                    net_name: net_name.clone(),
                    reason: reason.clone(),
                });
                *reject_reasons.entry(reason).or_default() += 1;
            }
        }
    }
    summary.timed_out |= Instant::now() >= stage_deadline;
    summary.reasons = reject_reasons
        .into_iter()
        .map(|(reason, count)| format!("{reason} ({count})"))
        .collect();
    summary.elapsed_ms = start.elapsed().as_millis() as u64;
    Ok(summary)
}

pub fn run_rescue(
    board: &mut BoardAdapter,
    source: &BoardAdapter,
    protected: &BTreeSet<String>,
    cfg: &NativeRescueConfig,
    deadline: Instant,
    min_ring: f64,
) -> Result<NativeRescueSummary, String> {
    if !min_ring.is_finite() || min_ring < 0.0 {
        return Err("minimum annular ring must be finite and non-negative".into());
    }
    if cfg.multi_start == 0 {
        return Err("native rescue multi_start must be at least 1".into());
    }
    let start = Instant::now();
    let mut summary = NativeRescueSummary {
        enabled: cfg.enabled,
        multi_start_requested: cfg.multi_start,
        ..Default::default()
    };
    if !cfg.enabled {
        summary.elapsed_ms = start.elapsed().as_millis() as u64;
        return Ok(summary);
    }
    if cfg.budget_ms == 0 || cfg.max_connection_ms == 0 {
        return Err(
            "enabled native rescue requires positive budget_ms and max_connection_ms".into(),
        );
    }
    let stage_deadline = deadline.min(start + Duration::from_millis(cfg.budget_ms));
    let mut reasons = BTreeMap::<String, usize>::new();
    let source_ids = item_ids(source);
    if cfg.stage == NativeRescueStage::AfterAwx {
        let mut baseline_partitions = connected_partitions(board);
        let mut baseline_incompletes: BTreeMap<_, _> = board_nets(board)
            .into_iter()
            .map(|net| (net, incomplete_count(board, net)))
            .collect();
        let mut settings = RouterSettings::default();
        settings.set_layer_count(board.board.layer_count() as usize);

        let open_nets: Vec<_> = board_nets(board)
            .into_iter()
            .filter_map(|net| {
                let count = incomplete_count(board, net);
                (count > 0).then(|| {
                    let name = board
                        .board
                        .rules
                        .nets
                        .get(net)
                        .map(|n| n.name.clone())
                        .unwrap_or_default();
                    (net, name, count)
                })
            })
            .collect();
        for (net, name, initial_count) in open_nets {
            if protected.contains(&name) {
                summary.per_net.push(NativeRescueNetSummary {
                    net,
                    name,
                    before: initial_count,
                    after: initial_count,
                    last_reject_reason: Some("protected net skipped".into()),
                    ..Default::default()
                });
                continue;
            }
            let mut net_summary = NativeRescueNetSummary {
                net,
                name: name.clone(),
                before: initial_count,
                after: incomplete_count(board, net),
                ..Default::default()
            };
            let attempt_limit = initial_count;
            let mut tried_edges = BTreeSet::<(i32, i32)>::new();
            while net_summary.attempts < attempt_limit && Instant::now() < stage_deadline {
                let live =
                    NetIncompletes::new(&board.board, net, &board.board.get_connectable_items(net));
                if live.incompletes.is_empty() {
                    break;
                }
                let selected = live.incompletes.into_iter().find_map(|airline| {
                    let from_id = board.board.item(airline.from_item).id().0;
                    let to_id = board.board.item(airline.to_item).id().0;
                    let edge = (from_id.min(to_id), from_id.max(to_id));
                    (!tried_edges.contains(&edge)).then_some((airline, edge))
                });
                let Some((airline, edge)) = selected else {
                    break;
                };
                tried_edges.insert(edge);
                let target_before = incomplete_count(board, net);
                net_summary.attempts += 1;
                summary.attempts += 1;
                net_summary.last_from_item_id = Some(board.board.item(airline.from_item).id().0);
                net_summary.last_to_item_id = Some(board.board.item(airline.to_item).id().0);

                let from_connected = board.board.connected_set(airline.from_item, net, false);
                let to_connected = board.board.connected_set(airline.to_item, net, false);
                let original_connected: BTreeSet<_> = from_connected
                    .iter()
                    .map(|key| board.board.item(key).id().0)
                    .collect();
                let old_ids = item_ids(board);
                let mut candidate = board.clone();

                let mut saved_fixed = BTreeMap::<ItemId, FixedState>::new();
                for key in candidate.board.get_items() {
                    let item = candidate.board.item(key);
                    if source_ids.contains(&item.id().0)
                        || candidate.is_source_protected_copper(key)
                        || (!item.is_trace() && !item.is_via())
                        || item.net_count() != 1
                        || item.component_no() != 0
                        || item.fixed_state() == FixedState::SystemFixed
                    {
                        continue;
                    }
                    saved_fixed.insert(item.id(), item.fixed_state());
                }
                for key in candidate.board.get_items() {
                    let id = candidate.board.item(key).id();
                    if saved_fixed.contains_key(&id) {
                        candidate
                            .board
                            .item_mut(key)
                            .set_fixed_state(FixedState::Unfixed);
                    }
                }

                let mut ctrl = AutorouteControl::new_default(&candidate.board, net, &settings);
                ctrl.with_neckdown = false;
                ctrl.no_neckdown = true;
                ctrl.ripup_allowed = false;
                let remaining = stage_deadline.saturating_duration_since(Instant::now());
                let connection_ms = remaining
                    .min(Duration::from_millis(cfg.max_connection_ms))
                    .as_millis()
                    .clamp(1, i32::MAX as u128) as i32;
                let stop = StopToken::new();
                let time_limit_fired = Arc::new(AtomicBool::new(false));
                candidate.board.init_autoroute(
                    net,
                    ctrl.trace_clearance_class,
                    Some(&stop),
                    Some(
                        TimeLimit::wall_clock(connection_ms)
                            .with_fired_flag(Arc::clone(&time_limit_fired)),
                    ),
                    false,
                );
                let mut ripped_items = ItemSet::new();
                let outcome = candidate.board.autoroute_connection(
                    &from_connected,
                    &to_connected,
                    &ctrl,
                    &mut ripped_items,
                    None,
                );
                for key in candidate.board.get_items() {
                    let id = candidate.board.item(key).id();
                    if let Some(previous_state) = saved_fixed.get(&id) {
                        candidate
                            .board
                            .item_mut(key)
                            .set_fixed_state(*previous_state);
                    }
                }
                let connection_timed_out = time_limit_fired.load(Ordering::Relaxed);
                let mut attempt_detail = NativeRescueAttemptSummary {
                    net,
                    net_name: name.clone(),
                    from_item_id: board.board.item(airline.from_item).id().0,
                    to_item_id: board.board.item(airline.to_item).id().0,
                    from_connected_len: from_connected.len(),
                    to_connected_len: to_connected.len(),
                    time_limit_fired: connection_timed_out,
                    native_state: format!("{:?}", outcome.state),
                    reason: None,
                };
                if connection_timed_out {
                    net_summary.timeouts += 1;
                    summary.connection_timeouts += 1;
                }

                if outcome.state != AutorouteAttemptState::Routed || !ripped_items.is_empty() {
                    summary.native_failed += 1;
                    let reason = if outcome.state != AutorouteAttemptState::Routed {
                        if outcome.details.trim().is_empty() {
                            format!("native {:?}", outcome.state)
                        } else {
                            format!("native {:?}: {}", outcome.state, outcome.details.trim())
                        }
                    } else {
                        "native rescue reported ripped items".to_owned()
                    };
                    let reason = if connection_timed_out {
                        format!("{reason} (connection time limit fired)")
                    } else {
                        reason
                    };
                    net_summary.last_reject_reason = Some(reason.clone());
                    attempt_detail.reason = Some(reason.clone());
                    summary.attempt_details.push(attempt_detail);
                    *reasons.entry(reason).or_default() += 1;
                    continue;
                }

                match guard_candidate(
                    &candidate,
                    source,
                    protected,
                    airline.from_item,
                    net,
                    &original_connected,
                    &old_ids,
                    &baseline_partitions,
                    &baseline_incompletes,
                    min_ring,
                    &source_ids,
                    Some(target_before),
                ) {
                    Ok(_) => {
                        *board = candidate;
                        net_summary.accepted += 1;
                        summary.accepted += 1;
                        net_summary.after = incomplete_count(board, net);
                        baseline_partitions = connected_partitions(board);
                        baseline_incompletes = baseline_incompletes
                            .keys()
                            .copied()
                            .map(|n| (n, incomplete_count(board, n)))
                            .collect();
                    }
                    Err(reason) => {
                        summary.guard_rejected += 1;
                        net_summary.last_reject_reason = Some(reason.clone());
                        attempt_detail.reason = Some(reason.clone());
                        *reasons.entry(reason).or_default() += 1;
                    }
                }
                summary.attempt_details.push(attempt_detail);
                net_summary.after = incomplete_count(board, net);
            }
            if Instant::now() >= stage_deadline {
                summary.timed_out = true;
            }
            net_summary.after = incomplete_count(board, net);
            summary.per_net.push(net_summary);
            if summary.timed_out {
                break;
            }
        }
    }
    if cfg.stage == NativeRescueStage::BeforeAwx {
        *reasons
            .entry("before_awx uses native batch scheduling".into())
            .or_default() += 1;
    }
    run_batch_rip_multi_start(
        board,
        source,
        protected,
        &source_ids,
        min_ring,
        stage_deadline,
        cfg.multi_start,
        &mut summary,
        &mut reasons,
    );
    summary.timed_out |= summary.batch_timed_out || Instant::now() >= stage_deadline;
    summary.reasons = reasons
        .into_iter()
        .map(|(reason, count)| format!("{reason} ({count})"))
        .collect();
    summary.elapsed_ms = start.elapsed().as_millis() as u64;
    Ok(summary)
}
