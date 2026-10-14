//! Native incremental, perturbation, and crossover replan coordinator.
//!
//! The coordinator only advances from physical probe results. Requested moves
//! are never treated as landed until the probe reports their achieved ends.

use super::{
    escape_moves::Move,
    replan_probes::{
        probe_many, rank_candidates, replan_cost_model, ProbeCache, ProbeCandidate, ProbeEnd,
        ProbeResult,
    },
    replan_state::ReplanState,
    replan_types::AcceptedMove,
    route_evolution::RouteGrade,
};
use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum ReplanMode {
    #[default]
    Incremental,
    Rebraid,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FinalizationPath {
    Strip,
    Rebraid,
}

#[derive(Clone, Debug)]
pub struct FinalizationRecord {
    pub round: usize,
    pub path: FinalizationPath,
    pub grade: RouteGrade,
    pub whole_rounds: usize,
    pub relayed: Vec<String>,
    pub salvaged: Vec<String>,
}

#[derive(Clone)]
pub struct NativeReplanOptions {
    pub rounds: usize,
    pub worst: usize,
    pub probes: usize,
    pub min_vias: usize,
    pub gate_min: usize,
    pub gates: usize,
    pub census: usize,
    pub joint: bool,
    pub coupled: String,
    pub grade: String,
    pub mode: ReplanMode,
    pub apply_strip: bool,
    pub seed: u64,
    pub jobs: usize,
    pub perturb_nets: usize,
    pub perturb_seed: u64,
    pub perturb_tries: usize,
    pub cross_fraction: f64,
    pub cross_parent: Option<Box<ReplanState>>,
    pub source_climb: u32,
    /// Per-operation environment overrides. They are passed to operators and
    /// never installed process-wide, so parallel routing remains isolated.
    pub descend_env: Vec<(String, String)>,
    pub probe_env: Vec<(String, String)>,
}

impl Default for NativeReplanOptions {
    fn default() -> Self {
        Self {
            rounds: 4,
            worst: 3,
            probes: 1,
            min_vias: 3,
            gate_min: 2,
            gates: 3,
            census: 4,
            joint: true,
            coupled: "census".into(),
            grade: "sub".into(),
            mode: ReplanMode::Incremental,
            apply_strip: true,
            seed: 1,
            jobs: 1,
            perturb_nets: 0,
            perturb_seed: 1,
            perturb_tries: 1,
            cross_fraction: 0.5,
            cross_parent: None,
            source_climb: 14,
            descend_env: Vec::new(),
            probe_env: Vec::new(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct AcceptedRecord {
    pub round: usize,
    pub net: String,
    pub kind: AcceptedKind,
    pub tag: String,
    /// Nets whose copper/end geometry physically landed on the accepted board.
    pub landed_nets: Vec<String>,
    pub grade: RouteGrade,
    pub achieved_src: Option<Move>,
    pub achieved_dst: Option<Move>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AcceptedKind {
    Incremental,
    Jumped,
    Crossed,
}

#[derive(Clone)]
pub struct NativeReplanResult {
    pub state: ReplanState,
    pub accepted: Vec<AcceptedRecord>,
    pub stopped_no_candidate: bool,
    pub finalized_rounds: Vec<FinalizationRecord>,
    /// Effective operation-local overrides, with `probe_env` taking precedence
    /// over duplicate keys in `descend_env`.
    pub environment_overrides: BTreeMap<String, String>,
}

enum FinalizedRound {
    Strip(super::replan_finalize::StripFinalization),
    Rebraid(super::replan_finalize::RebraidFinalization),
}

/// Run native replan rounds against the current routed world. Candidate
/// rankings and physical trials are delegated to the native probe stage.
pub fn run_replan(
    mut state: ReplanState,
    options: NativeReplanOptions,
) -> Result<NativeReplanResult, String> {
    if options.jobs == 0 {
        return Err("native replan jobs must be positive".into());
    }
    if !options.cross_fraction.is_finite() || !(0.0..=1.0).contains(&options.cross_fraction) {
        return Err("native replan cross fraction must be in [0, 1]".into());
    }
    let environment_overrides = raw_environment_overrides(&options);
    state.set_environment_overrides(environment_overrides.clone());
    state.refresh_world()?;
    state.apply_replan_modes(&options.coupled, &options.grade)?;
    apply_probe_env(&mut state, &options)?;
    let mut result = run_rounds(state, options)?;
    result.environment_overrides = environment_overrides;
    Ok(result)
}

/// Merge operation-local environment maps without changing the process
/// environment. Probe-specific values override same-named descent values.
pub fn raw_environment_overrides(options: &NativeReplanOptions) -> BTreeMap<String, String> {
    let mut values = BTreeMap::new();
    for (key, value) in &options.descend_env {
        values.insert(key.clone(), value.clone());
    }
    for (key, value) in &options.probe_env {
        values.insert(key.clone(), value.clone());
    }
    values
}

fn apply_probe_env(state: &mut ReplanState, options: &NativeReplanOptions) -> Result<(), String> {
    let mut values = BTreeMap::new();
    for (key, value) in options.descend_env.iter().chain(&options.probe_env) {
        values.insert(key.as_str(), value.as_str());
    }
    let mut source_climb = Some(options.source_climb);
    let mut destination_climb = None;
    let mut attempts = 1usize;
    let mut budget_x = 4.0f64;
    let mut ladder = "open".to_owned();
    let mut smooth = false;
    let mut exact_pages = false;
    let mut swimmer_page_price = 2.0f64;
    let mut swim_vias = 2.0f64;
    let mut swim_price_fallback = None;
    let mut page_price_override = None;
    let mut swim_vias_override = None;
    let mut plan_judge = String::new();
    for (key, value) in values {
        match key {
            "SRC_CLIMB" => {
                source_climb = Some(
                    value
                        .parse::<u32>()
                        .map_err(|_| "SRC_CLIMB must be a nonnegative integer")?,
                );
            }
            "DST_CLIMB" => {
                destination_climb = Some(
                    value
                        .parse::<u32>()
                        .map_err(|_| "DST_CLIMB must be a nonnegative integer")?,
                );
            }
            "PROBE_ATTEMPTS" => {
                attempts = value
                    .parse::<usize>()
                    .map_err(|_| "PROBE_ATTEMPTS must be a positive integer")?;
                if attempts == 0 {
                    return Err("PROBE_ATTEMPTS must be positive".into());
                }
            }
            "PROBE_BUDGET_X" => {
                budget_x = value
                    .parse::<f64>()
                    .map_err(|_| "PROBE_BUDGET_X must be numeric")?;
            }
            "PROBE_LADDER" => ladder = value.to_owned(),
            "PROBE_SMOOTH" => {
                smooth = match value {
                    "1" | "true" | "yes" => true,
                    "0" | "false" | "no" => false,
                    _ => return Err("PROBE_SMOOTH must be boolean".into()),
                };
            }
            "BRAID_EXACT_PAGES" => {
                exact_pages = match value {
                    "1" | "true" | "yes" => true,
                    "0" | "false" | "no" => false,
                    _ => return Err("BRAID_EXACT_PAGES must be boolean".into()),
                };
            }
            "SWIM_PRICE" => {
                let price = value
                    .parse::<f64>()
                    .map_err(|_| "SWIM_PRICE must be numeric")?;
                swim_price_fallback = Some(price);
            }
            "BRAID_SWIM_W" | "prices.PAGES" => {
                page_price_override = Some(
                    value
                        .parse::<f64>()
                        .map_err(|_| "BRAID_SWIM_W must be numeric")?,
                );
            }
            "SWIM_VIAS" | "prices.JUDGE" => {
                swim_vias_override = Some(
                    value
                        .parse::<f64>()
                        .map_err(|_| "SWIM_VIAS must be numeric")?,
                );
            }
            "PLAN_JUDGE" => plan_judge = value.to_owned(),
            // These exact source schedule/corridor settings are consumed by
            // ReplanState's prediction builder. Preserve raw values on state,
            // including explicit zero, rather than interpreting them here.
            "BRAID_PAIRS"
            | "BRAID_BLOCK_PUSH"
            | "BRAID_PAIR_GAP"
            | "BRAID_PAIR_DIVE_EXTRA"
            | "BRAID_JOG_VIA"
            | "PLAN_PAGES_SIDERS"
            | "BRAID_BRANCH" => {}
            _ => {
                return Err(format!(
                    "unsupported native replan environment override {key}"
                ))
            }
        }
    }
    if let Some(price) = swim_price_fallback {
        swimmer_page_price = price;
        swim_vias = price;
    }
    if let Some(price) = page_price_override {
        swimmer_page_price = price;
    }
    if let Some(price) = swim_vias_override {
        swim_vias = price;
    }
    if let Some(climb) = source_climb {
        state.probe.source_input.climb = Some(climb);
    }
    if let Some(climb) = destination_climb {
        state.probe.destination_input.climb = Some(climb);
    }
    state.apply_probe_config(attempts, budget_x, &ladder, smooth)?;
    state.apply_cost_config(exact_pages, swimmer_page_price, swim_vias)?;
    state.apply_plan_judge(&plan_judge)?;
    if source_climb.is_some() {
        state.probe.source_menus =
            super::joint_escape::build_menus(&state.probe.source_input, &state.snapshot)?;
        state.probe.source_plan = super::joint_escape::solve_menus(
            &state.probe.source_input,
            state.probe.source_menus.clone(),
        )?;
    }
    if destination_climb.is_some() {
        state.probe.destination_menus =
            super::joint_escape::build_menus(&state.probe.destination_input, &state.snapshot)?;
        state.probe.destination_plan = super::joint_escape::solve_menus(
            &state.probe.destination_input,
            state.probe.destination_menus.clone(),
        )?;
    }
    Ok(())
}

fn run_rounds(
    mut state: ReplanState,
    options: NativeReplanOptions,
) -> Result<NativeReplanResult, String> {
    let mut cache = ProbeCache::default();
    let mut accepted = Vec::new();
    let mut finalized_rounds = Vec::new();
    let mut stopped_no_candidate = false;
    let mut rng = SeededRng::new(
        if options.perturb_nets > 0 || options.cross_parent.is_some() {
            options.perturb_seed
        } else {
            options.seed
        },
    );
    let mut bans_src = std::collections::BTreeSet::<(String, String)>::new();
    let mut bans_dst = BTreeMap::<String, std::collections::BTreeSet<String>>::new();
    let mut bans_pair = std::collections::BTreeSet::<(String, String, String)>::new();

    let rounds = if options.perturb_nets > 0 || options.cross_parent.is_some() {
        options.rounds.min(1)
    } else {
        options.rounds
    };
    for round in 1..=rounds {
        let round_start = state.clone();
        state.refresh_verdict(
            options.worst,
            options.min_vias,
            options.gate_min,
            options.gates,
        )?;
        let mut targets = if options.perturb_nets > 0 {
            sample_names(&state.names, options.perturb_nets, &mut rng)
        } else if options.cross_parent.is_some() {
            cross_targets(
                &state,
                options.cross_parent.as_deref().unwrap(),
                options.cross_fraction,
                &mut rng,
            )
        } else {
            state.worst.clone()
        };
        if options.perturb_nets == 0 && options.cross_parent.is_none() {
            targets.truncate(options.worst);
        }
        if targets.is_empty() {
            stopped_no_candidate = true;
            break;
        }

        let bad_at_round_start = targets.clone();
        let blockers_at_round_start: BTreeMap<_, _> = bad_at_round_start
            .iter()
            .map(|name| {
                (
                    name.clone(),
                    state
                        .verdicts
                        .get(name)
                        .map(|verdict| {
                            verdict
                                .census
                                .iter()
                                .take(options.census)
                                .cloned()
                                .collect::<Vec<_>>()
                        })
                        .unwrap_or_default(),
                )
            })
            .collect();
        let gatekeepers_at_round_start =
            if options.perturb_nets == 0 && options.cross_parent.is_none() {
                state.gatekeepers.clone()
            } else {
                Vec::new()
            };
        let mut accepted_this_round = 0usize;
        for net in targets {
            let is_cross = options.cross_parent.is_some();
            let is_perturb = options.perturb_nets > 0;
            let candidates = if is_cross {
                cross_candidates(&state, options.cross_parent.as_deref().unwrap(), &net)
            } else {
                ranked_candidates(
                    &state,
                    &net,
                    options.probes,
                    options.perturb_tries,
                    is_perturb,
                    options.joint,
                    &mut rng,
                    &bans_src,
                    &bans_dst,
                    &bans_pair,
                )?
            };
            if candidates.is_empty() {
                continue;
            }
            let results = probe_many(&state, &candidates, options.jobs, &mut cache)?;
            let mut eligible: Vec<_> = results
                .iter()
                .filter(|probe| accepted_probe(probe, &state.current_grade, is_perturb, is_cross))
                .cloned()
                .collect();
            eligible.sort_by(|a, b| grade_order(&a.grade, &b.grade));
            if !is_perturb && !is_cross {
                for probe in &results {
                    if probe.unjudged || accepted_probe(probe, &state.current_grade, false, false) {
                        continue;
                    }
                    ban_probe(probe, &mut bans_src, &mut bans_dst, &mut bans_pair);
                }
            }
            let Some(probe) = eligible.into_iter().next() else {
                continue;
            };

            let record = AcceptedRecord {
                round,
                net: net.clone(),
                kind: if is_cross {
                    AcceptedKind::Crossed
                } else if is_perturb {
                    AcceptedKind::Jumped
                } else {
                    AcceptedKind::Incremental
                },
                tag: probe.candidate.tag.clone(),
                landed_nets: landed_net_names(&net, &probe),
                grade: probe.grade.clone(),
                achieved_src: probe.achieved_src.clone(),
                achieved_dst: probe.achieved_dst.clone(),
            };
            let accepted_move = AcceptedMove {
                net: net.clone(),
                src_got: probe.achieved_src.clone(),
                dst_got: probe.achieved_dst.clone(),
                comove_got: probe.comove_got.clone(),
            };
            state.advance(
                accepted_move,
                probe.board.clone(),
                probe.grade.clone(),
                Some(probe.fanout_after.clone()),
                &probe.relaid,
            )?;
            accepted.push(record);
            accepted_this_round += 1;
        }
        if options.perturb_nets == 0 && options.cross_parent.is_none() {
            let bad: std::collections::BTreeSet<_> = bad_at_round_start.iter().cloned().collect();
            let mut seen_gatekeepers: std::collections::BTreeSet<String> = accepted
                .iter()
                .filter(|record| record.round == round)
                .map(|record| record.net.clone())
                .collect();
            for gate in gatekeepers_at_round_start.into_iter().take(options.gates) {
                if bad.contains(&gate) || seen_gatekeepers.contains(&gate) {
                    continue;
                }
                if accepted_this_round >= options.worst.saturating_add(2) {
                    break;
                }
                let blocked_by: Vec<_> = bad_at_round_start
                    .iter()
                    .filter(|bad_net| {
                        blockers_at_round_start
                            .get(*bad_net)
                            .is_some_and(|names| names.iter().any(|name| name == &gate))
                    })
                    .cloned()
                    .collect();
                if blocked_by.len() < options.gate_min {
                    continue;
                }
                let mut candidates = ranked_candidates(
                    &state,
                    &gate,
                    options.probes,
                    options.perturb_tries,
                    false,
                    options.joint,
                    &mut rng,
                    &bans_src,
                    &bans_dst,
                    &bans_pair,
                )?;
                for candidate in &mut candidates {
                    candidate.extra_relay = blocked_by.clone();
                }
                if candidates.is_empty() {
                    continue;
                }
                let probes = probe_many(&state, &candidates, options.jobs, &mut cache)?;
                let mut standing: Vec<_> = probes
                    .iter()
                    .filter(|probe| accepted_probe(probe, &state.current_grade, false, false))
                    .cloned()
                    .collect();
                standing.sort_by(|a, b| grade_order(&a.grade, &b.grade));
                let Some(probe) = standing.into_iter().next() else {
                    continue;
                };
                let record = AcceptedRecord {
                    round,
                    net: gate.clone(),
                    kind: AcceptedKind::Incremental,
                    tag: format!("gatekeeper:{}", blocked_by.len()),
                    landed_nets: landed_net_names(&gate, &probe),
                    grade: probe.grade.clone(),
                    achieved_src: probe.achieved_src.clone(),
                    achieved_dst: probe.achieved_dst.clone(),
                };
                state.advance(
                    AcceptedMove {
                        net: gate.clone(),
                        src_got: probe.achieved_src.clone(),
                        dst_got: probe.achieved_dst.clone(),
                        comove_got: probe.comove_got.clone(),
                    },
                    probe.board.clone(),
                    probe.grade.clone(),
                    Some(probe.fanout_after.clone()),
                    &probe.relaid,
                )?;
                accepted.push(record);
                seen_gatekeepers.insert(gate);
                accepted_this_round += 1;
            }
        }
        if accepted_this_round == 0 {
            stopped_no_candidate = true;
            break;
        }
        let changed: std::collections::BTreeSet<_> = accepted
            .iter()
            .filter(|record| record.round == round)
            .map(|record| record.net.clone())
            .collect();
        let use_strip = options.mode == ReplanMode::Incremental && options.apply_strip;
        let finalized = if use_strip {
            match super::replan_finalize::apply_strip(&state, &changed) {
                Ok(strip) if strip_endpoints_agree(&state, &changed, &strip.plan_ends) => {
                    Some(FinalizedRound::Strip(strip))
                }
                // A strip result with ends that do not match the actual probe
                // landing must be judged by the full native rebraid.
                Ok(_) | Err(_) => super::replan_finalize::rebraid(&state)
                    .ok()
                    .map(FinalizedRound::Rebraid),
            }
        } else {
            super::replan_finalize::rebraid(&state)
                .ok()
                .map(FinalizedRound::Rebraid)
        };
        let Some(finalized) = finalized else {
            state = round_start;
            accepted.retain(|record| record.round != round);
            stopped_no_candidate = true;
            break;
        };
        let (grade, path, whole_rounds, relayed, salvaged) = match &finalized {
            FinalizedRound::Strip(value) => (
                value.grade.clone(),
                FinalizationPath::Strip,
                0,
                Vec::new(),
                value.salvaged.clone(),
            ),
            FinalizedRound::Rebraid(value) => (
                value.grade.clone(),
                FinalizationPath::Rebraid,
                value.whole_rounds,
                value.relayed.clone(),
                Vec::new(),
            ),
        };
        let allow_non_improving_strip = path == FinalizationPath::Strip
            && (options.perturb_nets > 0 || options.cross_parent.is_some());
        if !grade.better_than(&round_start.current_grade) && !allow_non_improving_strip {
            state = round_start;
            accepted.retain(|record| record.round != round);
            stopped_no_candidate = true;
            break;
        }
        install_finalized_world(&mut state, finalized)?;
        if state.refresh_world().is_err()
            || state
                .refresh_verdict(
                    options.worst,
                    options.min_vias,
                    options.gate_min,
                    options.gates,
                )
                .is_err()
        {
            state = round_start;
            accepted.retain(|record| record.round != round);
            stopped_no_candidate = true;
            break;
        }
        finalized_rounds.push(FinalizationRecord {
            round,
            path,
            grade,
            whole_rounds,
            relayed,
            salvaged,
        });
    }

    Ok(NativeReplanResult {
        state,
        accepted,
        stopped_no_candidate,
        finalized_rounds,
        environment_overrides: BTreeMap::new(),
    })
}

fn ranked_candidates(
    state: &ReplanState,
    net: &str,
    probes: usize,
    perturb_tries: usize,
    perturb: bool,
    joint: bool,
    rng: &mut SeededRng,
    bans_src: &std::collections::BTreeSet<(String, String)>,
    bans_dst: &BTreeMap<String, std::collections::BTreeSet<String>>,
    bans_pair: &std::collections::BTreeSet<(String, String, String)>,
) -> Result<Vec<ProbeCandidate>, String> {
    let model = replan_cost_model(state)?;
    let verdict = state.verdicts.get(net);
    let ends: Vec<ProbeEnd> = match verdict.and_then(|v| v.walled_at.as_deref()) {
        Some("tooth") => vec![ProbeEnd::Source],
        Some("berth") => vec![ProbeEnd::Destination],
        _ => vec![ProbeEnd::Destination, ProbeEnd::Source],
    };
    let mut by_end = BTreeMap::<u8, Vec<ProbeCandidate>>::new();
    for end in ends {
        let src_bans: std::collections::BTreeSet<String> = bans_src
            .iter()
            .filter(|(banned_net, _)| banned_net == net)
            .map(|(_, key)| key.clone())
            .collect();
        let empty_bans = std::collections::BTreeSet::new();
        let ban_set = if end == ProbeEnd::Source {
            &src_bans
        } else {
            bans_dst.get(net).unwrap_or(&empty_bans)
        };
        let (ranked, _) = rank_candidates(
            state,
            net,
            end,
            ban_set,
            if perturb { usize::MAX } else { probes + 1 },
            &model,
        )?;
        let src = end == ProbeEnd::Source;
        let mut entries: Vec<_> = ranked.into_iter().map(|r| r.candidate).collect();
        if perturb {
            let current = state.ends.get(net).and_then(|e| {
                if src {
                    e.src_move.as_ref()
                } else {
                    e.dst_move.as_ref()
                }
            });
            entries.retain(|candidate| {
                current.is_none_or(|c| {
                    candidate.movement.direction != c.direction
                        || candidate.movement.layer != c.layer
                })
            });
            rng.shuffle(&mut entries);
            entries.truncate(8);
        } else {
            entries.truncate(probes);
        }
        by_end.insert(if src { 0 } else { 1 }, entries);
    }
    let mut candidates = Vec::new();
    for end in [1u8, 0u8] {
        if let Some(entries) = by_end.get(&end) {
            candidates.extend(entries.iter().cloned());
        }
    }
    if !perturb && joint && by_end.contains_key(&0) && by_end.contains_key(&1) {
        for (i, src) in by_end[&0].iter().take(probes).enumerate() {
            for (j, dst) in by_end[&1].iter().take(probes).enumerate() {
                let mut candidate = src.clone();
                candidate.dst = dst.dst.clone();
                candidate.dst_ball = dst.dst_ball.clone();
                candidate.comove.extend(dst.comove.iter().cloned());
                candidate.comove.sort();
                candidate.comove.dedup();
                candidate.tag = format!("both{i}_{j}");
                let pair_key = (
                    net.to_owned(),
                    src_class_key(&src.movement),
                    dst_class_key(&dst.movement),
                );
                if !bans_pair.contains(&pair_key) {
                    candidates.push(candidate);
                }
            }
        }
    }
    if perturb {
        rng.shuffle(&mut candidates);
        candidates.truncate(perturb_tries.max(1));
    }
    Ok(candidates)
}

fn ban_probe(
    probe: &ProbeResult,
    bans_src: &mut std::collections::BTreeSet<(String, String)>,
    bans_dst: &mut BTreeMap<String, std::collections::BTreeSet<String>>,
    bans_pair: &mut std::collections::BTreeSet<(String, String, String)>,
) {
    if let (Some(source), Some(destination)) = (&probe.candidate.src, &probe.candidate.dst) {
        bans_pair.insert((
            probe.candidate.net.clone(),
            src_class_key(source),
            dst_class_key(destination),
        ));
        return;
    }
    if let Some(movement) = &probe.candidate.src {
        bans_src.insert((probe.candidate.net.clone(), src_class_key(movement)));
    }
    if let Some(movement) = &probe.candidate.dst {
        bans_dst
            .entry(probe.candidate.net.clone())
            .or_default()
            .insert(dst_class_key(movement));
    }
}

fn src_class_key(movement: &Move) -> String {
    format!(
        "{:?}|{}|{:?}|{}",
        movement.direction, movement.layer, movement.kind, movement.climb
    )
}

fn dst_class_key(movement: &Move) -> String {
    format!("{:?}|{}", movement.direction, movement.layer)
}

fn pad_key(state: &ReplanState, net: &str, source: bool) -> Option<String> {
    let menus = if source {
        &state.probe.source_menus
    } else {
        &state.probe.destination_menus
    };
    let component = if source {
        &state.probe.source_part
    } else {
        &state.probe.destination_part
    };
    let short = super::escape_moves::short_name(net);
    menus
        .items
        .values()
        .find(|pad| {
            (pad.net == net || super::escape_moves::short_name(&pad.net) == short)
                && pad.component_ref == *component
        })
        .map(|pad| pad.key.clone())
}

fn cross_targets(
    state: &ReplanState,
    parent: &ReplanState,
    fraction: f64,
    rng: &mut SeededRng,
) -> Vec<String> {
    let mut differing: Vec<_> = state
        .names
        .iter()
        .filter(|name| {
            state
                .ends
                .get(*name)
                .zip(parent.ends.get(*name))
                .is_some_and(|(a, b)| {
                    end_differs(a.src_move.as_ref(), b.src_move.as_ref(), a.tooth, b.tooth)
                        || end_differs(a.dst_move.as_ref(), b.dst_move.as_ref(), a.exit, b.exit)
                })
        })
        .cloned()
        .collect();
    differing.sort();
    rng.shuffle(&mut differing);
    let count = if differing.is_empty() {
        0
    } else {
        ((fraction * differing.len() as f64).round() as usize)
            .max(1)
            .min(differing.len())
    };
    differing.truncate(count);
    differing
}

fn cross_candidates(state: &ReplanState, parent: &ReplanState, net: &str) -> Vec<ProbeCandidate> {
    let (Some(a), Some(b)) = (state.ends.get(net), parent.ends.get(net)) else {
        return Vec::new();
    };
    let src = if end_differs(a.src_move.as_ref(), b.src_move.as_ref(), a.tooth, b.tooth) {
        b.src_move.clone()
    } else {
        None
    };
    let dst = if end_differs(a.dst_move.as_ref(), b.dst_move.as_ref(), a.exit, b.exit) {
        b.dst_move.clone()
    } else {
        None
    };
    let src_ball = src.as_ref().and_then(|_| pad_key(state, net, true));
    let dst_ball = dst.as_ref().and_then(|_| pad_key(state, net, false));
    let mut out = Vec::new();
    if src.as_ref().is_some_and(|_| src_ball.is_none())
        || dst.as_ref().is_some_and(|_| dst_ball.is_none())
    {
        return out;
    }
    if let (Some(s), Some(d)) = (&src, &dst) {
        out.push(ProbeCandidate {
            net: net.into(),
            ball: dst_ball.clone().unwrap_or_default(),
            src_ball: src_ball.clone(),
            dst_ball: dst_ball.clone(),
            end: ProbeEnd::Destination,
            movement: d.clone(),
            src: Some(s.clone()),
            dst: Some(d.clone()),
            comove: Vec::new(),
            extra_relay: Vec::new(),
            tag: "cross-both".into(),
        });
    }
    if let Some(d) = dst {
        out.push(ProbeCandidate {
            net: net.into(),
            ball: dst_ball.clone().unwrap_or_default(),
            src_ball: None,
            dst_ball: dst_ball.clone(),
            end: ProbeEnd::Destination,
            movement: d.clone(),
            src: None,
            dst: Some(d),
            comove: Vec::new(),
            extra_relay: Vec::new(),
            tag: "cross-dst".into(),
        });
    }
    if let Some(s) = src {
        out.push(ProbeCandidate {
            net: net.into(),
            ball: src_ball.clone().unwrap_or_default(),
            src_ball: src_ball.clone(),
            dst_ball: None,
            end: ProbeEnd::Source,
            movement: s.clone(),
            src: Some(s),
            dst: None,
            comove: Vec::new(),
            extra_relay: Vec::new(),
            tag: "cross-src".into(),
        });
    }
    out
}

fn end_differs(a: Option<&Move>, b: Option<&Move>, a_point: [f64; 2], b_point: [f64; 2]) -> bool {
    match (a, b) {
        (None, None) => false,
        (Some(_), None) | (None, Some(_)) => true,
        (Some(a), Some(b)) => {
            a.direction != b.direction
                || a.layer != b.layer
                || a.kind != b.kind
                || (a_point[0] - b_point[0]).hypot(a_point[1] - b_point[1]) > 0.3
        }
    }
}

fn accepted_probe(probe: &ProbeResult, reference: &RouteGrade, perturb: bool, cross: bool) -> bool {
    if probe.fail.is_some()
        || probe.refused
        || probe.unjudged
        || probe.refused_nets.contains(&probe.candidate.net)
    {
        return false;
    }
    if !perturb && probe.grade.opens.iter().any(|n| n == &probe.candidate.net) {
        return false;
    }
    let class_matches = probe
        .candidate
        .src
        .as_ref()
        .zip(probe.achieved_src.as_ref())
        .is_none_or(|(a, b)| a.direction == b.direction && a.layer == b.layer)
        && probe
            .candidate
            .dst
            .as_ref()
            .zip(probe.achieved_dst.as_ref())
            .is_none_or(|(a, b)| a.direction == b.direction && a.layer == b.layer)
        && (probe.candidate.src.is_none() || probe.achieved_src.is_some())
        && (probe.candidate.dst.is_none() || probe.achieved_dst.is_some());
    if !cross && !class_matches {
        return false;
    }
    if perturb {
        true
    } else if cross {
        probe.grade.opens.len() <= reference.opens.len()
    } else {
        probe.grade.better_than(reference)
    }
}

fn strip_endpoints_agree(
    state: &ReplanState,
    changed: &std::collections::BTreeSet<String>,
    plan_ends: &serde_json::Map<String, serde_json::Value>,
) -> bool {
    const END_AGREE_MM: f64 = 0.5;
    for net in changed {
        let Some(actual) = state.ends.get(net) else {
            return false;
        };
        let Some(points) = plan_ends.get(net).and_then(serde_json::Value::as_array) else {
            return false;
        };
        for (index, expected) in [actual.tooth, actual.exit].iter().enumerate() {
            let Some(point) = points
                .get(index)
                .and_then(serde_json::Value::as_array)
                .filter(|point| point.len() >= 2)
            else {
                return false;
            };
            let (Some(x), Some(y)) = (
                point.first().and_then(serde_json::Value::as_f64),
                point.get(1).and_then(serde_json::Value::as_f64),
            ) else {
                return false;
            };
            if (expected[0] - x).hypot(expected[1] - y) > END_AGREE_MM {
                return false;
            }
        }
    }
    true
}

fn install_finalized_world(
    state: &mut ReplanState,
    finalized: FinalizedRound,
) -> Result<(), String> {
    let (routed, fanout, plan_sidecar, plan_ends, plan_hash, pack, census, census_hash, grade) =
        match finalized {
            FinalizedRound::Strip(value) => (
                value.routed,
                value.fanout,
                value.plan_sidecar,
                value.plan_ends,
                value.plan_sidecar_hash,
                value.pack_sidecar,
                value.census_sidecar,
                value.census_sidecar_hash,
                value.grade,
            ),
            FinalizedRound::Rebraid(value) => (
                value.routed,
                value.fanout,
                value.plan_sidecar,
                value.plan_ends,
                value.plan_sidecar_hash,
                value.pack_sidecar,
                value.census_sidecar,
                value.census_sidecar_hash,
                value.grade,
            ),
        };
    state.world.routed = routed;
    state.world.fanout = fanout;
    state.world.plan_sidecar = plan_sidecar;
    state.world.plan_ends = plan_ends;
    state.world.plan_sidecar_hash = plan_hash;
    state.world.pack_sidecar = pack;
    state.world.census_sidecar = census;
    state.world.census_sidecar_hash = census_hash;
    state.current_grade = grade;
    state.fanout_seed = state.world.fanout.clone();
    for net in &state.names {
        state
            .fan_src
            .insert(net.clone(), state.world.fanout.clone());
    }
    Ok(())
}

fn landed_net_names(net: &str, probe: &ProbeResult) -> Vec<String> {
    let mut names = vec![net.to_owned()];
    names.extend(probe.relaid.iter().cloned());
    names.extend(probe.comove_got.keys().cloned());
    names.sort();
    names.dedup();
    names
}

fn grade_order(a: &RouteGrade, b: &RouteGrade) -> std::cmp::Ordering {
    (a.opens.len(), a.vias).cmp(&(b.opens.len(), b.vias))
}

fn sample_names(names: &[String], count: usize, rng: &mut SeededRng) -> Vec<String> {
    let mut chosen = names.to_vec();
    rng.shuffle(&mut chosen);
    chosen.truncate(count.min(chosen.len()));
    chosen
}

struct SeededRng(u64);
impl SeededRng {
    fn new(seed: u64) -> Self {
        Self(seed ^ 0x9e3779b97f4a7c15)
    }
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn shuffle<T>(&mut self, values: &mut [T]) {
        for i in (1..values.len()).rev() {
            let j = (self.next() as usize) % (i + 1);
            values.swap(i, j);
        }
    }
}
