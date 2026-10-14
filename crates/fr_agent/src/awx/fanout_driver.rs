//! Native transaction driver for the complete AWX fanout and whole-route path.
use super::{
    board_snapshot::BoardSnapshot,
    integration::{self, AwxBusReport, AwxExecution},
    joint_escape::{self, JointPlan, OptionMove},
    native_execution,
    whole_context::WholeContext,
    whole_ends::{self, EndsFeedback, EndsModel, EndsProblem},
};
use crate::{
    board_adapter::BoardAdapter,
    spec::{RouteGroup, RouteSpec},
};
use std::{
    collections::{BTreeMap, BTreeSet},
    time::Instant,
};

fn repair_passives(
    board: &mut BoardAdapter,
    spec: &mut RouteSpec,
    moved_components: &mut BTreeSet<String>,
    permit_translation: bool,
) -> Result<(super::passive_clearance::PlacementPass, Vec<String>), String> {
    use super::passive_clearance as pc;
    let cfg = spec
        .awx
        .as_ref()
        .ok_or("missing AWX physical configuration")?;
    if cfg.passive_geometry.is_empty() {
        return Ok((
            pc::PlacementPass {
                moved: None,
                held: vec![],
                unresolved: BTreeMap::new(),
            },
            vec![],
        ));
    }
    let snapshot = BoardSnapshot::capture(board, &cfg.components, &cfg.via_drills)?;
    let body_bounds = pc::merge_body_bounds(&cfg.passive_geometry, &cfg.component_body_bounds);
    let captured = pc::capture_obstacles_with_bodies(board, &body_bounds)?;
    let caps = pc::configured_caps(
        board,
        &cfg.passive_geometry,
        &snapshot.pads,
        &captured.pad_classes,
    )?;
    let mut balls: BTreeMap<i32, Vec<[f64; 2]>> = BTreeMap::new();
    for pad in &snapshot.pads {
        balls.entry(pad.net_id).or_default().push(pad.point);
    }
    let options = if permit_translation {
        pc::RepairOptions::default()
    } else {
        pc::RepairOptions {
            displacement: 0.,
            max_displacement: 0.,
            ..pc::RepairOptions::default()
        }
    };
    let mut pass = pc::repair_board_once(board, &caps, &captured.obstacles, &balls, options)?;
    for (reference, geometry) in &cfg.passive_geometry {
        let locked = geometry.locked
            || board
                .board
                .components
                .get_by_name(reference)
                .is_some_and(|component| component.position_fixed);
        if locked && !pass.held.contains(reference) {
            pass.held.push(reference.clone());
        }
    }
    pass.held.sort();
    pass.held.dedup();
    if let Some(change) = &pass.moved {
        moved_components.insert(change.reference.clone());
        if let Some(geometry) = spec
            .awx
            .as_mut()
            .and_then(|a| a.passive_geometry.get_mut(&change.reference))
        {
            geometry.pose.position = None;
        }
    }
    if pass.unresolved.is_empty() {
        return Ok((pass, vec![]));
    }

    let current_cfg = spec.awx.as_ref().ok_or("missing AWX configuration")?;
    let snapshot = BoardSnapshot::capture(board, &current_cfg.components, &current_cfg.via_drills)?;
    let body_bounds = pc::merge_body_bounds(
        &current_cfg.passive_geometry,
        &current_cfg.component_body_bounds,
    );
    let captured = pc::capture_obstacles_with_bodies(board, &body_bounds)?;
    let caps = pc::configured_caps(
        board,
        &current_cfg.passive_geometry,
        &snapshot.pads,
        &captured.pad_classes,
    )?;
    let fab_floor = super::fab_rules::fab_floor_ladder(
        board.board.layer_count() as usize,
        current_cfg.fabrication,
        super::fab_rules::EscalationPolicy::Off,
        &current_cfg.fabrication_overrides,
        &BTreeMap::new(),
    )
    .into_iter()
    .next()
    .ok_or("fabrication rule produced no via-nudge drill floor")?;
    let drill_floors =
        pc::resolve_via_nudge_drill_floors(current_cfg.rules.hole_to_hole_clearance, &fab_floor)?;
    let mut nudged = Vec::new();
    for (reference, shortfall) in &pass.unresolved {
        let Some(cap) = caps.iter().find(|c| &c.reference == reference) else {
            continue;
        };
        if shortfall.vias.is_empty() {
            continue;
        }
        for via_key in pc::offending_vias(board, cap, &cap.pose) {
            let n = board
                .board
                .item(via_key)
                .net_numbers()
                .first()
                .copied()
                .ok_or("offending via lost its net")?;
            if !shortfall.vias.contains_key(&n) {
                continue;
            }
            let fresh_captured = pc::capture_obstacles_with_bodies(board, &body_bounds)?;
            let problem = pc::capture_via_nudge_problem(
                board,
                via_key,
                cap,
                &cap.pose,
                &fresh_captured,
                &drill_floors,
                current_cfg.via_nudge.max_shift_mm,
            )?;
            if let Some(move_) = pc::plan_via_nudge(&problem)? {
                pc::apply_via_nudge(board, via_key, &move_)?;
                nudged.push(reference.clone());
                break;
            }
        }
    }
    nudged.sort();
    nudged.dedup();
    Ok((pass, nudged))
}

fn names(groups: &[&RouteGroup]) -> BTreeSet<String> {
    groups.iter().flat_map(|g| g.nets.iter().cloned()).collect()
}

fn filter_groups_to_nets(groups: &[&RouteGroup], selected: &BTreeSet<String>) -> Vec<RouteGroup> {
    let selected_short: BTreeSet<_> = selected
        .iter()
        .map(|n| super::escape_moves::short_name(n).to_owned())
        .collect();
    let mut out = Vec::new();
    for source in groups {
        let mut group = (*source).clone();
        group
            .nets
            .retain(|n| selected_short.contains(super::escape_moves::short_name(n)));
        if group.nets.is_empty() {
            continue;
        }
        if let Some(awx) = group.awx.as_mut() {
            awx.pairs.retain(|p| {
                selected_short.contains(super::escape_moves::short_name(&p.p))
                    && selected_short.contains(super::escape_moves::short_name(&p.n))
            });
        }
        out.push(group);
    }
    out
}

fn filter_plan_to_nets(plan: &JointPlan, selected: &BTreeSet<String>) -> JointPlan {
    let selected_short: BTreeSet<_> = selected
        .iter()
        .map(|n| super::escape_moves::short_name(n).to_owned())
        .collect();
    let keep: BTreeSet<_> = plan
        .pieces
        .iter()
        .filter_map(|(key, piece)| {
            selected_short
                .contains(super::escape_moves::short_name(&piece.net))
                .then_some(key.clone())
        })
        .collect();
    let mut out = plan.clone();
    out.pieces.retain(|key, _| keep.contains(key));
    out.chosen.retain(|key, _| keep.contains(key));
    out.selected_indices.retain(|key, _| keep.contains(key));
    out.hints.retain(|hint| keep.contains(&hint.ball));
    out
}

fn destination_move_class(movement: &super::escape_moves::Move) -> String {
    format!(
        "{:?}|{:?}|{}|{:.2},{:.2}",
        movement.kind, movement.direction, movement.layer, movement.exit_pt[0], movement.exit_pt[1]
    )
}

fn destination_refusal_class(
    refusal: &super::destination_realize::DestinationMoveRefusal,
) -> String {
    let class = &refusal.ban_class;
    format!(
        "{:?}|{:?}|{}|{:.2},{:.2}",
        class.kind,
        class.direction,
        class.layer,
        class.exit_pt_rounded[0],
        class.exit_pt_rounded[1]
    )
}

fn add_refusal_bans(
    bans: &mut BTreeSet<(String, String)>,
    refusal: &super::destination_realize::DestinationMoveRefusal,
    plan: &JointPlan,
    pairs: &[super::pairs::Pair],
) {
    bans.insert((refusal.ball.clone(), destination_refusal_class(refusal)));
    let Some(pair_name) = refusal.pair.as_deref() else {
        return;
    };
    for pair in pairs.iter().filter(|pair| pair.base == pair_name) {
        for member in [&pair.p, &pair.n] {
            for (ball, option) in &plan.chosen {
                if let OptionMove::Escape { movement } = option {
                    if super::escape_moves::short_name(&movement.net)
                        == super::escape_moves::short_name(member)
                    {
                        bans.insert((ball.clone(), destination_move_class(movement)));
                    }
                }
            }
        }
    }
}

fn filter_move_bans(
    menus: &mut super::joint_escape::JointMenus,
    banned: &BTreeSet<(String, String)>,
) {
    if banned.is_empty() {
        return;
    }
    for (ball, moves) in &mut menus.menus {
        moves
            .retain(|movement| !banned.contains(&(ball.clone(), destination_move_class(movement))));
    }
    for (ball, options) in &mut menus.options {
        options.retain(|option| match &option.option {
            OptionMove::Escape { movement } => {
                !banned.contains(&(ball.clone(), destination_move_class(movement)))
            }
            _ => true,
        });
    }
}

fn price_source_refusals(
    feedback: &mut EndsFeedback,
    decisions: &[super::source_realize::SourceNetDecision],
    plan: &JointPlan,
    pairs: &[super::pairs::Pair],
    flow: super::whole_frame::FlowFrame,
    physical_layers: &[String],
    round: usize,
) -> Result<(), String> {
    feedback.points_in_flow_frame = true;
    for decision in decisions.iter().filter(|decision| !decision.accepted) {
        let movement = plan.chosen.iter().find_map(|(name, choice)| match choice {
            OptionMove::Escape { movement }
                if super::escape_moves::short_name(name)
                    == super::escape_moves::short_name(&decision.net) =>
            {
                Some(movement)
            }
            _ => None,
        });
        let Some(movement) = movement else {
            continue;
        };
        let lane = pairs
            .iter()
            .find(|pair| {
                super::escape_moves::short_name(&pair.p)
                    == super::escape_moves::short_name(&decision.net)
                    || super::escape_moves::short_name(&pair.n)
                        == super::escape_moves::short_name(&decision.net)
            })
            .map(|pair| pair.base.clone())
            .unwrap_or_else(|| decision.net.clone());
        let item = whole_ends::FeedbackEnd {
            lane,
            end: 0,
            points: vec![flow.to_frame(movement.exit_pt)],
            layer: flow.to_frame_layer(&movement.layer, physical_layers)?,
            times: 1,
            round: Some(round),
        };
        if !feedback.avoid.iter().any(|old| {
            old.lane == item.lane
                && old.end == item.end
                && old.layer == item.layer
                && old.points == item.points
        }) {
            feedback.avoid.push(item);
        }
    }
    Ok(())
}

fn native_seed_plan(
    context: &WholeContext,
    destination_plan: &JointPlan,
    names: &[String],
) -> Result<serde_json::Value, String> {
    let mut berth_sig = serde_json::Map::new();
    for name in names {
        let movement = destination_plan
            .chosen
            .iter()
            .find_map(|(ball, option)| match option {
                OptionMove::Escape { movement }
                    if super::escape_moves::short_name(&movement.net)
                        == super::escape_moves::short_name(name) =>
                {
                    Some((ball, movement))
                }
                _ => None,
            })
            .ok_or_else(|| format!("destination seed plan has no selected berth for {name}"))?;
        let signature = whole_ends::move_signature(movement.1)?;
        let value: serde_json::Value =
            serde_json::from_str(&signature).map_err(|e| e.to_string())?;
        berth_sig.insert(name.clone(), value);
    }
    Ok(serde_json::json!({
        "chi": if context.chirality_mirrored {-1} else {1},
        "berth_sig": berth_sig,
    }))
}

fn plan_ends(
    board: &BoardAdapter,
    spec: &RouteSpec,
    groups: &[&RouteGroup],
    source: &str,
    destination: &str,
    nets: &BTreeSet<String>,
    deadline: Instant,
    feedback: &EndsFeedback,
    seed_plans: Option<(&JointPlan, &JointPlan)>,
    banned_source: &BTreeSet<(String, String)>,
    banned_destination: &BTreeSet<(String, String)>,
) -> Result<
    (
        BoardSnapshot,
        JointPlan,
        JointPlan,
        joint_escape::JointMenus,
        joint_escape::JointMenus,
        EndsModel,
    ),
    String,
> {
    let cfg = spec
        .awx
        .as_ref()
        .ok_or("missing AWX physical configuration")?;
    cfg.rules.validate()?;
    let snapshot = BoardSnapshot::capture(board, &cfg.components, &cfg.via_drills)?;
    let si = integration::joint_input(&snapshot, spec, groups, source, false, deadline)?;
    let di = integration::joint_input(&snapshot, spec, groups, destination, true, deadline)?;
    let mut sm = joint_escape::build_menus(&si, &snapshot)?;
    let mut dm = joint_escape::build_menus(&di, &snapshot)?;
    whole_ends::retain_measurable_menu_moves(&si, &mut sm)?;
    whole_ends::retain_measurable_menu_moves(&di, &mut dm)?;
    filter_move_bans(&mut sm, banned_source);
    filter_move_bans(&mut dm, banned_destination);
    let sp = joint_escape::solve_menus(&si, sm.clone())?;
    let dp = joint_escape::solve_menus(&di, dm.clone())?;
    let pair_rows: Vec<_> = groups
        .iter()
        .flat_map(|g| {
            g.awx
                .as_ref()
                .into_iter()
                .flat_map(|a| a.pairs.iter().cloned())
        })
        .collect();
    let mut held = BTreeMap::new();
    let mut seed = BTreeMap::new();
    if let Some((_, previous_destination)) = seed_plans {
        let freed_berths: BTreeSet<_> = feedback
            .avoid
            .iter()
            .filter(|e| e.end == 1)
            .map(|e| e.lane.as_str())
            .chain(
                feedback
                    .pairs
                    .iter()
                    .flat_map(|p| p.iter().filter(|e| e.end == 1).map(|e| e.lane.as_str())),
            )
            .collect();
        for (name, option) in &previous_destination.chosen {
            if let OptionMove::Escape { movement } = option {
                seed.insert(name.clone(), movement.clone());
                if !freed_berths.contains(name.as_str()) {
                    held.insert(name.clone(), whole_ends::move_signature(movement)?);
                }
            }
        }
    }
    let mut free_teeth: BTreeSet<String> = feedback
        .avoid
        .iter()
        .filter(|e| e.end == 0)
        .map(|e| e.lane.clone())
        .collect();
    free_teeth.extend(
        feedback
            .pairs
            .iter()
            .flat_map(|p| p.iter().filter(|e| e.end == 0).map(|e| e.lane.clone())),
    );
    let mut legs_by_base = BTreeMap::new();
    for pair in &pair_rows {
        legs_by_base.insert(pair.base.clone(), vec![pair.p.clone(), pair.n.clone()]);
    }
    for lane in free_teeth.clone() {
        if let Some(legs) = legs_by_base.get(&lane) {
            free_teeth.extend(legs.iter().cloned());
        }
    }
    let has_feedback = !feedback.avoid.is_empty() || !feedback.pairs.is_empty();
    let choice = whole_ends::choose(&EndsProblem {
        snapshot: &snapshot,
        source_input: &si,
        destination_input: &di,
        source_menus: &sm,
        destination_menus: &dm,
        requested: nets,
        pairs: &pair_rows,
        feedback,
        laid_source: Some(&sp),
        laid_destination: Some(&dp),
        source_free: true,
        free_teeth: has_feedback.then_some(&free_teeth),
        fixed: &held,
        seed: &seed,
        learned: &BTreeSet::new(),
        deadline,
        rules: &cfg.rules,
    })?;
    Ok((
        snapshot,
        choice.source,
        choice.destination,
        sm,
        dm,
        choice.model,
    ))
}

/// Execute one connected AWX group transaction at a time. Groups with equal
/// source/destination arrays are solved jointly, including their differential pairs.
pub fn execute(
    board: &BoardAdapter,
    spec: &RouteSpec,
    deadline: Instant,
) -> Result<AwxExecution, String> {
    if Instant::now() >= deadline {
        return Err("AWX shared deadline exhausted before fanout".into());
    }
    let related = integration::related_groups(spec)?;
    let mut current = board.clone();
    let mut reports = Vec::new();
    let mut protected = spec.protected_names();
    let mut all_requested_routed = true;
    let mut deferred_nets = BTreeSet::new();
    let mut runtime_spec = spec.clone();
    let mut moved_components = BTreeSet::new();
    let mut failed_groups = BTreeSet::new();
    let mut runtime_events = super::runtime_stats::AwxRuntimeEvents::default();
    let mut seed_captures = Vec::new();
    let mut drills = runtime_spec
        .awx
        .as_ref()
        .ok_or("missing AWX physical configuration")?
        .via_drills
        .clone();
    let cfg = runtime_spec
        .awx
        .as_ref()
        .ok_or("missing AWX physical configuration")?;
    let discovered_buses = if cfg.bus_discovery.enabled {
        let snapshot = BoardSnapshot::capture(board, &cfg.components, &cfg.via_drills)?;
        Some(super::bus_discovery::discover_buses(
            &snapshot,
            &super::bus_discovery::DiscoveryRequest {
                min_bus_nets: cfg.bus_discovery.min_bus_nets,
            },
            cfg.rules.via_size,
        )?)
    } else {
        None
    };
    for groups in related {
        let cfg = groups
            .first()
            .and_then(|g| g.awx.as_ref())
            .ok_or("missing AWX group")?;
        let requested = names(&groups);
        let original = current.clone();
        if groups
            .iter()
            .any(|g| g.depends_on.iter().any(|d| failed_groups.contains(d)))
        {
            all_requested_routed = false;
            let mut refused = BTreeMap::new();
            for net in &requested {
                refused.insert(
                    net.clone(),
                    "an AWX dependency group did not complete".into(),
                );
            }
            let grade = native_execution::grade(&original, &original, &requested)?;
            reports.push(AwxBusReport {
                groups: groups.iter().map(|g| g.id.clone()).collect(),
                source: cfg.source.clone(),
                destination: cfg.destination.clone(),
                requested: requested.iter().cloned().collect(),
                routed: vec![],
                refused,
                joint_source: None,
                joint_destination: None,
                rounds: 0,
                grade,
                proved: false,
                reason: "skipped because a dependency group failed".into(),
                placement: None,
                via_nudged: vec![],
                source_decisions: vec![],
                source_audits: vec![],
                source_refusals: vec![],
                destination_decisions: vec![],
                destination_audits: vec![],
                destination_refusals: vec![],
                smoothing: None,
                eligible_nets: vec![],
                selected_ladder_nets: vec![],
                omitted_ladder_nets: vec![],
                ladder_order: vec![],
                deferred_ladder_nets: vec![],
            });
            failed_groups.extend(groups.iter().map(|g| g.id.clone()));
            continue;
        }
        let mut transaction_base = original.clone();
        let mut joint_reports: Option<(joint_escape::JointReport, joint_escape::JointReport)> =
            None;
        let mut placement_state: Option<super::passive_clearance::PlacementPass> = None;
        let mut nudged_state = BTreeSet::new();
        let mut source_decision_state = Vec::new();
        let mut source_audit_state = Vec::new();
        let mut source_refusal_state = Vec::new();
        let mut destination_state: Option<super::destination_realize::DestinationRealization> =
            None;
        let mut banned_source = BTreeSet::<(String, String)>::new();
        let mut banned_destination = BTreeSet::<(String, String)>::new();
        let mut ladder_order_state = Vec::new();
        let mut selected_ladder_state = Vec::new();
        let mut deferred_ladder_state = Vec::new();
        let result = (|| -> Result<(BoardAdapter, AwxBusReport), String> {
            let rules = runtime_spec
                .awx
                .as_ref()
                .ok_or("missing AWX configuration")?
                .rules
                .clone();
            let allow_unproved = groups
                .iter()
                .all(|g| g.awx.as_ref().is_some_and(|a| a.allow_unproved));
            let max_rounds = groups
                .iter()
                .filter_map(|g| g.awx.as_ref().map(|a| a.fanout_rounds))
                .max()
                .unwrap_or(1)
                .max(1);
            let mut feedback = EndsFeedback::default();
            let mut previous: Option<(JointPlan, JointPlan)> = None;
            let mut best: Option<(
                BoardAdapter,
                AwxBusReport,
                super::route_recovery::RecoveryRank,
            )> = None;
            for round in 0..max_rounds {
                if Instant::now() >= deadline {
                    break;
                }
                let (
                    _snapshot,
                    source_plan,
                    destination_plan,
                    source_menus,
                    destination_menus,
                    ends_model,
                ) = plan_ends(
                    &transaction_base,
                    &runtime_spec,
                    &groups,
                    &cfg.source,
                    &cfg.destination,
                    &requested,
                    deadline,
                    &feedback,
                    previous.as_ref().map(|(s, d)| (s, d)),
                    &banned_source,
                    &banned_destination,
                )?;
                joint_reports = Some((source_plan.report.clone(), destination_plan.report.clone()));
                let mut round_drills = drills.clone();
                let source_realization = super::source_realize::realize_source(
                    &transaction_base,
                    &source_plan,
                    &rules,
                    &mut round_drills,
                    &protected,
                )?;
                let source_decisions = source_realization.nets;
                source_decision_state = source_decisions.clone();
                source_audit_state = source_realization.audits.clone();
                source_refusal_state = source_realization.refusals.clone();
                let mut fanout = source_realization.board;
                let source_snapshot = BoardSnapshot::capture(
                    &fanout,
                    &runtime_spec.awx.as_ref().unwrap().components,
                    &round_drills,
                )?;
                let candidate_nets: Vec<String> =
                    if runtime_spec.awx.as_ref().unwrap().bus_discovery.enabled {
                        discovered_buses
                            .as_ref()
                            .into_iter()
                            .flatten()
                            .filter(|bus| {
                                bus.taken
                                    && ((bus.src == cfg.source && bus.dest == cfg.destination)
                                        || (bus.src == cfg.destination && bus.dest == cfg.source))
                            })
                            .flat_map(|bus| bus.nets.iter().cloned())
                            .filter(|name| {
                                requested
                                    .iter()
                                    .any(|net| super::escape_moves::short_name(net) == name)
                            })
                            .collect::<BTreeSet<_>>()
                            .into_iter()
                            .collect()
                    } else {
                        requested
                            .iter()
                            .map(|name| super::escape_moves::short_name(name).to_owned())
                            .collect()
                    };
                let ladder_order = super::lane_order::produce_source_order(
                    &source_snapshot,
                    &cfg.source,
                    &cfg.destination,
                    &candidate_nets,
                    &rules,
                )?;
                let ladder = super::route_ladder::RouteLadder::new(
                    ladder_order.clone(),
                    runtime_spec.awx.as_ref().unwrap().route_ladder.requested_k,
                    runtime_spec.awx.as_ref().unwrap().route_ladder.rounds,
                );
                let eligible_nets: Vec<_> = ladder
                    .ordered_nets()
                    .iter()
                    .filter_map(|short| {
                        requested
                            .iter()
                            .find(|net| super::escape_moves::short_name(net) == short)
                            .cloned()
                    })
                    .collect();
                let selected_ladder_nets: Vec<_> = ladder
                    .selected_nets()
                    .iter()
                    .filter_map(|short| {
                        requested
                            .iter()
                            .find(|net| super::escape_moves::short_name(net) == short)
                            .cloned()
                    })
                    .collect();
                let selected_requested: BTreeSet<_> =
                    selected_ladder_nets.iter().cloned().collect();
                let pair_rows: Vec<_> = groups
                    .iter()
                    .flat_map(|g| {
                        g.awx
                            .as_ref()
                            .into_iter()
                            .flat_map(|a| a.pairs.iter().cloned())
                    })
                    .collect();
                for refusal in &source_refusal_state {
                    add_refusal_bans(&mut banned_source, refusal, &source_plan, &pair_rows);
                }
                let omitted_ladder_nets: Vec<_> =
                    requested.difference(&selected_requested).cloned().collect();
                ladder_order_state = ladder_order.clone();
                selected_ladder_state = selected_ladder_nets.clone();
                deferred_ladder_state = omitted_ladder_nets.clone();
                deferred_nets.extend(omitted_ladder_nets.iter().cloned());
                let destination_realization = super::destination_realize::realize_destination(
                    &fanout,
                    &filter_plan_to_nets(&destination_plan, &selected_requested),
                    &rules,
                    &mut round_drills,
                    &protected,
                )?;
                let has_destination_refusal = !destination_realization.refusals.is_empty();
                for refusal in &destination_realization.refusals {
                    add_refusal_bans(
                        &mut banned_destination,
                        refusal,
                        &destination_plan,
                        &pair_rows,
                    );
                }
                let destination_decisions = destination_realization.nets.clone();
                let destination_audits = destination_realization.audits.clone();
                let destination_refusals = destination_realization.refusals.clone();
                fanout = destination_realization.board.clone();
                destination_state = Some(destination_realization);
                let (mut placement, via_nudged) = repair_passives(
                    &mut fanout,
                    &mut runtime_spec,
                    &mut moved_components,
                    round == 0,
                )?;
                if let Some(previous_pass) = &placement_state {
                    if placement.moved.is_none() {
                        placement.moved = previous_pass.moved.clone();
                    }
                    placement.held.extend(previous_pass.held.iter().cloned());
                    placement.held.sort();
                    placement.held.dedup();
                }
                placement_state = Some(placement.clone());
                nudged_state.extend(via_nudged);
                if round == 0 {
                    if let Some(change) = &placement.moved {
                        super::passive_clearance::apply_component_translation(
                            &mut transaction_base,
                            &change.reference,
                            change.after.position,
                        )?;
                    }
                }
                let has_source_refusal = source_refusal_state.iter().any(|r| {
                    selected_ladder_nets.iter().any(|net| {
                        super::escape_moves::short_name(net)
                            == super::escape_moves::short_name(&r.net)
                    })
                });
                if has_destination_refusal || has_source_refusal {
                    if round + 1 < max_rounds {
                        previous = Some((source_plan, destination_plan));
                        continue;
                    }
                    break;
                }
                let routed_groups_owned = filter_groups_to_nets(&groups, &selected_requested);
                let routed_groups: Vec<_> = routed_groups_owned.iter().collect();
                if selected_requested.is_empty() {
                    let grade =
                        native_execution::grade(&transaction_base, &fanout, &selected_requested)?;
                    let refused =
                        omitted_ladder_nets
                            .iter()
                            .map(|n| {
                                (n.clone(),
                        "omitted by the source coherent ladder; left to route_remaining".into())
                            })
                            .collect();
                    let report = AwxBusReport {
                        groups: groups.iter().map(|g| g.id.clone()).collect(),
                        source: cfg.source.clone(),
                        destination: cfg.destination.clone(),
                        requested: requested.iter().cloned().collect(),
                        routed: vec![],
                        refused,
                        joint_source: Some(source_plan.report.clone()),
                        joint_destination: Some(destination_plan.report.clone()),
                        rounds: round + 1,
                        grade,
                        proved: false,
                        reason: "the coherent ladder selected no Whole Route nets".into(),
                        placement: Some(placement),
                        via_nudged: nudged_state.iter().cloned().collect(),
                        source_decisions: source_decisions.clone(),
                        source_audits: source_audit_state.clone(),
                        source_refusals: source_refusal_state.clone(),
                        destination_decisions,
                        destination_audits,
                        destination_refusals,
                        smoothing: None,
                        eligible_nets,
                        selected_ladder_nets,
                        omitted_ladder_nets: omitted_ladder_nets.clone(),
                        ladder_order: ladder_order.clone(),
                        deferred_ladder_nets: omitted_ladder_nets,
                    };
                    let rank = super::route_recovery::RecoveryRank {
                        new_violations: usize::from(report.grade.new_violations > 0),
                        open_nets: report.grade.open.len(),
                        unproved: 1,
                        vias: report.grade.vias,
                    };
                    if best.as_ref().is_none_or(|(_, _, old)| rank < *old) {
                        best = Some((fanout.clone(), report, rank));
                    }
                    break;
                }
                let selected_short: BTreeSet<_> = selected_requested
                    .iter()
                    .map(|net| super::escape_moves::short_name(net).to_owned())
                    .collect();
                let selected_source_decisions: Vec<_> = source_decisions
                    .iter()
                    .filter(|decision| {
                        selected_short.contains(super::escape_moves::short_name(&decision.net))
                    })
                    .cloned()
                    .collect();
                let attempt = integration::route_attempt(
                    &transaction_base,
                    &fanout,
                    &runtime_spec,
                    &routed_groups,
                    &source_plan,
                    &filter_plan_to_nets(&destination_plan, &selected_requested),
                    &source_menus,
                    &destination_menus,
                    &round_drills,
                    ladder.rounds(),
                    deadline,
                )?;
                runtime_events.whole_rounds = Some(
                    runtime_events
                        .whole_rounds
                        .unwrap_or(0)
                        .saturating_add(attempt.whole_rounds),
                );
                runtime_events.relay_count = Some(
                    runtime_events
                        .relay_count
                        .unwrap_or(0)
                        .saturating_add(attempt.relay_count),
                );
                runtime_events.whole_route_elapsed_us = Some(
                    runtime_events
                        .whole_route_elapsed_us
                        .unwrap_or(0)
                        .saturating_add(attempt.whole_route_elapsed_us),
                );
                runtime_events.relayer_elapsed_us = Some(
                    runtime_events
                        .relayer_elapsed_us
                        .unwrap_or(0)
                        .saturating_add(attempt.relayer_elapsed_us),
                );
                let proof = attempt.whole.plan.as_ref().is_some_and(|p| p.proved);
                let mut routed_candidate = attempt.board;
                let smoothing = if runtime_spec
                    .awx
                    .as_ref()
                    .is_some_and(|a| a.board_smoothing.enabled)
                {
                    let base_ids: BTreeSet<_> = fanout
                        .board
                        .get_traces()
                        .into_iter()
                        .map(|key| fanout.board.item(key).id().0)
                        .collect();
                    let requested_ids: BTreeSet<_> = selected_requested
                        .iter()
                        .map(|net| routed_candidate.net_number(net))
                        .collect::<Result<_, _>>()?;
                    let lane_items: Vec<_> = routed_candidate
                        .board
                        .get_traces()
                        .into_iter()
                        .filter(|key| {
                            let item = routed_candidate.board.item(*key);
                            !base_ids.contains(&item.id().0)
                                && item
                                    .net_numbers()
                                    .iter()
                                    .any(|net| requested_ids.contains(net))
                        })
                        .collect();
                    let smooth_cfg = runtime_spec
                        .awx
                        .as_ref()
                        .ok_or("missing AWX smoothing configuration")?;
                    let snapshot = BoardSnapshot::capture(
                        &routed_candidate,
                        &smooth_cfg.components,
                        &round_drills,
                    )?;
                    let dense_fields = smooth_cfg
                        .smoothing_metadata
                        .as_ref()
                        .ok_or("smoothing enabled without explicit dense-pad metadata")?
                        .dense_fields(&snapshot, &smooth_cfg.rules)?;
                    Some(super::board_smoothing::smooth_board(
                        &mut routed_candidate,
                        &lane_items,
                        &dense_fields,
                    )?)
                } else {
                    None
                };
                let recovery = super::route_recovery::recover_candidate(
                    &transaction_base,
                    routed_candidate,
                    &selected_requested,
                    proof,
                    allow_unproved,
                )?;
                let valid = recovery.is_complete();
                let rank = recovery.rank;
                let recovery_rejection = recovery.rejection.clone();
                let candidate = recovery.board;
                let grade = recovery.grade;
                if valid {
                    integration::verify_requested(&candidate, &runtime_spec, &routed_groups)?;
                }
                let mut refused: BTreeMap<String, String> = omitted_ladder_nets
                    .iter()
                    .map(|net| {
                        (
                            net.clone(),
                            "omitted by coherent K prefix; left to route_remaining".into(),
                        )
                    })
                    .collect();
                match &recovery_rejection {
                    Some(super::route_recovery::RecoveryRejection::NewViolations {
                        count,
                        nets,
                    }) => {
                        let affected: BTreeSet<_> = if nets.is_empty() {
                            selected_requested.clone()
                        } else {
                            nets.iter().cloned().collect()
                        };
                        for net in affected {
                            refused.insert(net, format!("native recovery restored copper after {count} new violation(s)"));
                        }
                    }
                    Some(super::route_recovery::RecoveryRejection::OpenNets(nets)) => {
                        for net in nets {
                            refused
                                .insert(net.clone(), "native recovery restored an open net".into());
                        }
                    }
                    Some(super::route_recovery::RecoveryRejection::ProofRequired) => {
                        for net in &selected_requested {
                            refused.insert(
                                net.clone(),
                                "whole-route geometry lacks the required proof".into(),
                            );
                        }
                    }
                    None => {}
                }
                for n in &grade.open {
                    refused.insert(
                        n.clone(),
                        "net remains open after whole-route attempt".into(),
                    );
                }
                for decision in selected_source_decisions
                    .iter()
                    .filter(|d| !d.accepted && grade.open.contains(&d.net))
                {
                    refused.insert(
                        decision.net.clone(),
                        format!("source realization refused: {}", decision.reason),
                    );
                }
                let report = AwxBusReport {
                    groups: groups.iter().map(|g| g.id.clone()).collect(),
                    source: cfg.source.clone(),
                    destination: cfg.destination.clone(),
                    requested: requested.iter().cloned().collect(),
                    routed: grade.connected.clone(),
                    refused,
                    joint_source: Some(source_plan.report.clone()),
                    joint_destination: Some(destination_plan.report.clone()),
                    rounds: round + 1,
                    grade: grade.clone(),
                    proved: proof,
                    reason: attempt.whole.reasons.join("; "),
                    placement: Some(placement),
                    via_nudged: nudged_state.iter().cloned().collect(),
                    source_decisions: source_decisions.clone(),
                    source_audits: source_audit_state.clone(),
                    source_refusals: source_refusal_state.clone(),
                    destination_decisions,
                    destination_audits,
                    destination_refusals: destination_refusals.clone(),
                    smoothing,
                    eligible_nets: eligible_nets.clone(),
                    selected_ladder_nets: selected_ladder_nets.clone(),
                    omitted_ladder_nets: omitted_ladder_nets.clone(),
                    ladder_order: ladder_order.clone(),
                    deferred_ladder_nets: omitted_ladder_nets.clone(),
                };
                if best.as_ref().is_none_or(|(_, _, old)| rank < *old) {
                    best = Some((candidate.clone(), report.clone(), rank));
                }
                if valid {
                    drills = round_drills;
                    if runtime_spec
                        .awx
                        .as_ref()
                        .and_then(|a| a.seed_export.directory.as_ref())
                        .is_some()
                        && attempt.whole.plan.is_some()
                        && attempt.whole.geometry.is_some()
                    {
                        let source_refused = selected_source_decisions.iter().any(|d| !d.accepted);
                        let destination_refused = destination_refusals.iter().any(|r| {
                            selected_requested.iter().any(|name| {
                                super::escape_moves::short_name(name)
                                    == super::escape_moves::short_name(&r.net)
                            })
                        });
                        if !source_refused && !destination_refused {
                            let names: Vec<_> = selected_requested.iter().cloned().collect();
                            let mut seed_spec = runtime_spec.clone();
                            seed_spec
                                .nets
                                .retain(|net| selected_requested.contains(&net.name));
                            seed_spec.groups = routed_groups_owned.clone();
                            let seed_plan =
                                filter_plan_to_nets(&destination_plan, &selected_requested);
                            let plan_seed = native_seed_plan(&attempt.context, &seed_plan, &names)?;
                            seed_captures.push(integration::AwxSeedCapture {
                                board_dsn: runtime_spec.input.clone(),
                                spec: seed_spec,
                                requested_k: i64::try_from(names.len())
                                    .map_err(|_| "seed net count overflow")?,
                                destination: cfg.destination.clone(),
                                names,
                                routed: candidate.clone(),
                                fanout: fanout.clone(),
                                source_plan: filter_plan_to_nets(&source_plan, &selected_requested),
                                destination_plan: filter_plan_to_nets(
                                    &destination_plan,
                                    &selected_requested,
                                ),
                                whole: attempt.whole.clone(),
                                context: attempt.context.clone(),
                                flow: attempt.flow,
                                grade: grade.clone(),
                                plan_seed,
                            });
                        }
                    }
                    return Ok((candidate, report));
                }
                let mut next = super::whole_feedback::next_round(
                    &attempt.context,
                    &attempt.whole,
                    &ends_model,
                    &feedback,
                    &grade.open,
                    round + 1,
                    &source_plan,
                    &destination_plan,
                );
                let pair_rows: Vec<_> = groups
                    .iter()
                    .flat_map(|g| {
                        g.awx
                            .as_ref()
                            .into_iter()
                            .flat_map(|a| a.pairs.iter().cloned())
                    })
                    .collect();
                price_source_refusals(
                    &mut next,
                    &selected_source_decisions,
                    &source_plan,
                    &pair_rows,
                    attempt.flow,
                    &attempt.physical_layers,
                    round + 1,
                )?;
                let same_ends = if let Some((old_source, old_destination)) = &previous {
                    serde_json::to_string(&old_source.chosen).map_err(|e| e.to_string())?
                        == serde_json::to_string(&source_plan.chosen).map_err(|e| e.to_string())?
                        && serde_json::to_string(&old_destination.chosen)
                            .map_err(|e| e.to_string())?
                            == serde_json::to_string(&destination_plan.chosen)
                                .map_err(|e| e.to_string())?
                } else {
                    false
                };
                if same_ends && !feedback.avoid.is_empty() {
                    let trapped_open_lane = !grade.open.is_empty() && feedback.named.iter().any(|lane| {
                        let members: Vec<_> = pair_rows.iter().find(|pair| &pair.base == lane)
                            .map(|pair| vec![pair.p.as_str(),pair.n.as_str()]).unwrap_or_else(||vec![lane.as_str()]);
                        let open_lane = grade.open.iter().any(|net|members.iter().any(|m|
                            super::escape_moves::short_name(m)==super::escape_moves::short_name(net)));
                        let trapped = |plan:&JointPlan| plan.chosen.iter().any(|(name,choice)|
                            members.iter().any(|m|super::escape_moves::short_name(m)==super::escape_moves::short_name(name))
                                && matches!(choice,OptionMove::Escape{movement} if movement.kind==super::escape_moves::MoveKind::ViaInPad));
                        open_lane && (trapped(&source_plan)||trapped(&destination_plan))
                    });
                    next = super::whole_feedback::raise_round(
                        &attempt.context,
                        &next,
                        round + 1,
                        trapped_open_lane,
                    );
                }
                let changed = next.avoid.len() > feedback.avoid.len()
                    || next.pairs.len() > feedback.pairs.len()
                    || next.named.len() > feedback.named.len()
                    || next.over != feedback.over;
                if !changed {
                    break;
                }
                feedback = next;
                previous = Some((source_plan, destination_plan));
            }
            if let Some((candidate, report, _)) = best {
                Ok((candidate, report))
            } else if let Some(destination) = destination_state
                .as_ref()
                .filter(|d| !d.refusals.is_empty())
            {
                Err(format!(
                    "AWX destination berth realization exhausted retries: {}",
                    destination
                        .refusals
                        .iter()
                        .map(|r| format!("{}: {}", r.ball, r.reason))
                        .collect::<Vec<_>>()
                        .join("; ")
                ))
            } else {
                Err(
                    "AWX shared deadline exhausted before a complete fanout/whole-route attempt"
                        .into(),
                )
            }
        })();
        match result {
            Ok((next, report)) => {
                let selected_complete = report.selected_ladder_nets.is_empty()
                    || (report.grade.open.is_empty()
                        && report.grade.new_violations == 0
                        && (report.proved
                            || groups
                                .iter()
                                .all(|g| g.awx.as_ref().is_some_and(|a| a.allow_unproved))));
                if selected_complete {
                    protected.extend(report.routed.iter().cloned());
                } else {
                    all_requested_routed = false;
                }
                if !report.omitted_ladder_nets.is_empty() || !selected_complete {
                    all_requested_routed = false;
                }
                if let Some(change) = report.placement.as_ref().and_then(|p| p.moved.as_ref()) {
                    moved_components.insert(change.reference.clone());
                }
                current = next;
                if !selected_complete || !report.omitted_ladder_nets.is_empty() {
                    failed_groups.extend(groups.iter().map(|g| g.id.clone()));
                }
                reports.push(report);
            }
            Err(e) => {
                all_requested_routed = false;
                let mut refused = BTreeMap::new();
                for n in &requested {
                    refused.insert(n.clone(), e.clone());
                }
                for decision in source_decision_state.iter().filter(|d| !d.accepted) {
                    refused.insert(
                        decision.net.clone(),
                        format!(
                            "source realization refused: {}; transaction: {e}",
                            decision.reason
                        ),
                    );
                }
                // Per-net rollback is explicit, so a failed transaction cannot
                // disturb prior successful groups or the original route copper.
                let (_, grade) =
                    super::route_recovery::restore_failed_group(&original, &requested)?;
                let joint_source = joint_reports.as_ref().map(|r| r.0.clone());
                let joint_destination = joint_reports.as_ref().map(|r| r.1.clone());
                let destination_decisions = destination_state
                    .as_ref()
                    .map(|d| d.nets.clone())
                    .unwrap_or_default();
                let destination_audits = destination_state
                    .as_ref()
                    .map(|d| d.audits.clone())
                    .unwrap_or_default();
                let destination_refusals = destination_state
                    .as_ref()
                    .map(|d| d.refusals.clone())
                    .unwrap_or_default();
                failed_groups.extend(groups.iter().map(|g| g.id.clone()));
                reports.push(AwxBusReport {
                    groups: groups.iter().map(|g| g.id.clone()).collect(),
                    source: cfg.source.clone(),
                    destination: cfg.destination.clone(),
                    requested: requested.iter().cloned().collect(),
                    routed: vec![],
                    refused,
                    joint_source,
                    joint_destination,
                    rounds: 0,
                    grade,
                    proved: false,
                    reason: e,
                    placement: None,
                    via_nudged: vec![],
                    source_decisions: source_decision_state,
                    source_audits: source_audit_state,
                    source_refusals: source_refusal_state,
                    destination_decisions,
                    destination_audits,
                    destination_refusals,
                    smoothing: None,
                    eligible_nets: ladder_order_state.clone(),
                    selected_ladder_nets: selected_ladder_state.clone(),
                    omitted_ladder_nets: deferred_ladder_state.clone(),
                    ladder_order: ladder_order_state,
                    deferred_ladder_nets: deferred_ladder_state,
                });
            }
        }
    }
    Ok(AwxExecution {
        board: current,
        buses: reports,
        protected,
        all_requested_routed,
        moved_components,
        runtime_events,
        discovered_buses,
        deferred_nets,
        seed_captures,
    })
}
