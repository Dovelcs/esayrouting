use crate::{
    board_adapter::{BoardAdapter, Pad},
    planner::*,
    spec::{LoadedSpec, NetRule, RouteSpec},
};
use fr_engine::{datastructures::StopToken, pipeline::autorouter::BatchAutorouter};
use fr_settings::RouterSettings;
use serde::Serialize;
use std::{
    collections::{BTreeMap, BTreeSet},
    sync::mpsc,
    time::{Duration, Instant},
};

#[derive(Debug, Serialize)]
pub struct ExecutionReport {
    pub success: bool,
    pub output_committed: bool,
    pub deadline_exhausted: bool,
    pub rule_scope: String,
    pub length_metric: String,
    pub channel_protection: String,
    pub rules: Vec<crate::spec::RuleReport>,
    pub solver_workers: usize,
    pub conflict_workers: usize,
    pub detail_workers: usize,
    pub rejected_net_attempts: Vec<String>,
    pub stages: Vec<String>,
    pub candidate_count: usize,
    pub conflict_count: usize,
    pub selected_ids: Vec<String>,
    pub selected_candidates: Vec<PlanCandidate>,
    pub routing_changes: Vec<RoutingChange>,
    pub routing_changes_base: String,
    pub routing_counts_before: RoutingCounts,
    pub routing_counts_after: RoutingCounts,
    pub solve_seconds: f64,
    pub validation_seconds: f64,
    pub elapsed_seconds: f64,
    pub feedback_iterations: usize,
    pub solver_scope: String,
    pub output_written: bool,
    pub routed_groups: Vec<String>,
    pub remaining_incomplete: i32,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub awx_buses: Vec<crate::awx::integration::AwxBusReport>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub awx_statistics: Option<crate::awx::runtime_stats::AwxRunStats>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub awx_bus_discovery: Option<Vec<crate::awx::bus_discovery::DiscoveredBus>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub awx_evolution: Option<AwxEvolutionSummary>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub awx_seed_manifests: Option<Vec<std::path::PathBuf>>,
}
#[derive(Debug, Serialize)]
pub struct AwxEvolutionSummary {
    pub tag: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ledger_path: Option<std::path::PathBuf>,
    pub best_world_id: String,
    pub best_world_name: String,
    pub best_world_origin: String,
    pub population: usize,
    pub generations: usize,
    pub open_nets: Vec<String>,
    pub drc_violations: usize,
    pub vias: usize,
}
#[derive(Debug, Serialize)]
pub struct RoutingCounts {
    pub traces: usize,
    pub vias: usize,
}
fn routing_counts(board: &BoardAdapter) -> RoutingCounts {
    RoutingCounts {
        traces: board.board.get_traces().len(),
        vias: board
            .board
            .item_repository()
            .iter()
            .filter(|&k| board.board.item(k).is_via())
            .count(),
    }
}
#[derive(Debug, Serialize)]
pub struct RoutingChange {
    pub change: String,
    pub item_id: i32,
    pub before: Option<serde_json::Value>,
    pub after: Option<serde_json::Value>,
}
fn routing_snapshot(board: &BoardAdapter) -> BTreeMap<i32, serde_json::Value> {
    board.board.item_repository().iter().filter_map(|key|{let item=board.board.item(key);let nets:Vec<_>=item.net_numbers().iter().filter_map(|&n|board.board.rules.nets.get(n).map(|n|n.name.clone())).collect();let id=item.id().0;let geometry=if item.is_trace(){let trace=item.trace();serde_json::json!({"kind":"trace","net_names":nets,"layer":board.board.layer_structure.layers[trace.layer() as usize].name,"points_mm":trace.polyline().corner_approx_arr().into_iter().map(|p|[p.x/board.units_per_mm,p.y/board.units_per_mm]).collect::<Vec<_>>(),"width_mm":2.0*trace.half_width() as f64/board.units_per_mm,"clearance_class":item.clearance_class(),"fixed_state":format!("{:?}",item.fixed_state())})}else if item.is_via(){let p=item.center(&board.board).to_float();let pad=item.padstack(&board.board)?;serde_json::json!({"kind":"via","net_names":nets,"point_mm":[p.x/board.units_per_mm,p.y/board.units_per_mm],"padstack":pad.name,"from_layer":board.board.layer_structure.layers[pad.from_layer() as usize].name,"to_layer":board.board.layer_structure.layers[pad.to_layer() as usize].name,"clearance_class":item.clearance_class(),"fixed_state":format!("{:?}",item.fixed_state())})}else{return None;};Some((id,geometry))}).collect()
}
fn routing_changes(before: &BoardAdapter, after: &BoardAdapter) -> Vec<RoutingChange> {
    let old = routing_snapshot(before);
    let new = routing_snapshot(after);
    let ids: BTreeSet<_> = old.keys().chain(new.keys()).copied().collect();
    ids.into_iter()
        .filter_map(|item_id| {
            let a = old.get(&item_id);
            let b = new.get(&item_id);
            if a == b {
                return None;
            }
            let change = match (a, b) {
                (None, Some(_)) => "added",
                (Some(_), None) => "removed",
                _ => "modified",
            };
            Some(RoutingChange {
                change: change.into(),
                item_id,
                before: a.cloned(),
                after: b.cloned(),
            })
        })
        .collect()
}
fn budget(deadline: Instant) -> Result<(), String> {
    if Instant::now() >= deadline {
        Err("whole-run budget exhausted; no SES written".into())
    } else {
        Ok(())
    }
}
fn leg(pad: &Pad, rule: &NetRule, layer: &str, points: Vec<[f64; 2]>) -> RouteLeg {
    RouteLeg {
        net: pad.net.clone(),
        layer: layer.into(),
        points,
        width_mm: rule.width_mm,
        clearance_mm: rule.clearance_mm,
    }
}
fn candidate(
    id: String,
    set: String,
    group: &str,
    legs: Vec<RouteLeg>,
    requires_any: Vec<Vec<String>>,
) -> PlanCandidate {
    let length: f64 = legs
        .iter()
        .flat_map(|l| l.points.windows(2))
        .map(|s| ((s[0][0] - s[1][0]).powi(2) + (s[0][1] - s[1][1]).powi(2)).sqrt())
        .sum();
    PlanCandidate {
        id,
        selection_set_id: set,
        group_id: group.into(),
        cost: (length * 1000.0).ceil() as i64,
        legs,
        vias: vec![],
        reservations: vec![],
        requires_any,
    }
}

pub fn generate_problem(
    board: &BoardAdapter,
    spec: &RouteSpec,
    deadline: Instant,
) -> Result<PlannerProblem, String> {
    let pads = board.pads();
    let mut problem = PlannerProblem {
        selection_sets: vec![],
        candidates: vec![],
        conflicts: vec![],
        capacities: vec![],
    };
    let mut escapes: BTreeMap<String, Vec<PlanCandidate>> = BTreeMap::new();
    for group in ordered_groups(spec)? {
        budget(deadline)?;
        for name in &group.nets {
            let n = board.net_number(name)?;
            let count = board
                .board
                .get_pins()
                .into_iter()
                .filter(|&k| board.board.item(k).contains_net(n))
                .count();
            if count < 2 {
                return Err(format!("active group {} net {name} requires at least two routable pad endpoints, got {count}",group.id));
            }
        }
        if group.nets.iter().all(|name| {
            board.net_number(name).is_ok_and(|n| {
                board
                    .board
                    .get_connected_sets(n)
                    .iter()
                    .filter(|s| s.iter().any(|k| board.board.item(k).is_pin()))
                    .count()
                    <= 1
            })
        }) {
            continue;
        }
        let mut selected = Vec::new();
        for eg in &group.endpoint_groups {
            let endpoint_group = spec
                .endpoint_groups
                .iter()
                .find(|e| &e.id == eg)
                .ok_or("missing endpoint group")?;
            for e in &endpoint_group.endpoints {
                let pad = pads
                    .iter()
                    .find(|p| p.chip == e.chip && p.pad == e.pad && p.net == e.net)
                    .ok_or_else(|| {
                        format!("endpoint {}.{} net {} missing", e.chip, e.pad, e.net)
                    })?;
                if !selected.iter().any(|p: &&Pad| p.reference == pad.reference) {
                    selected.push(pad);
                }
            }
        }
        if selected.is_empty() {
            selected = pads
                .iter()
                .filter(|p| group.nets.contains(&p.net))
                .collect();
        }
        // Every other signal pad on an affected chip participates, even if it is not a bus member.

        let chips: BTreeSet<_> = selected.iter().map(|p| p.chip.clone()).collect();
        let affected: Vec<_> = pads
            .iter()
            .filter(|p| {
                !p.net.is_empty()
                    && (selected.iter().any(|q| q.reference == p.reference)
                        || (group.joint_escape && chips.contains(&p.chip)))
            })
            .collect();
        for pad in affected {
            budget(deadline)?;
            let key = format!("{}:{}", group.id, pad.reference);
            if escapes.contains_key(&key) {
                continue;
            }
            let derived = board.derived_rule(pad)?;
            let rule = spec
                .nets
                .iter()
                .find(|r| r.name == pad.net)
                .unwrap_or(&derived);
            let is_member = group.nets.contains(&pad.net) && !rule.protected;
            let set = format!("escape:{}:{}", group.id, pad.reference);
            let mut variants = Vec::new();
            for layer in pad
                .layers
                .iter()
                .filter(|l| rule.allowed_layers.contains(l))
            {
                for (i, dir) in [
                    [1.0, 0.0],
                    [-1.0, 0.0],
                    [0.0, 1.0],
                    [0.0, -1.0],
                    [0.7071067811865475, 0.7071067811865475],
                    [-0.7071067811865475, 0.7071067811865475],
                    [0.7071067811865475, -0.7071067811865475],
                    [-0.7071067811865475, -0.7071067811865475],
                ]
                .iter()
                .enumerate()
                {
                    let stub_length = if group.joint_escape {
                        0.6
                    } else {
                        2.0 / board.units_per_mm
                    };
                    let end = board.quantized([
                        pad.point[0] + dir[0] * stub_length,
                        pad.point[1] + dir[1] * stub_length,
                    ])?;
                    let l = leg(pad, rule, layer, vec![pad.point, end]);
                    let id = format!("{set}:{layer}:{i}");
                    let mut c = candidate(id, set.clone(), &group.id, vec![l], vec![]);
                    let mut check_spec = spec.clone();
                    if !is_member {
                        check_spec.protected_nets.retain(|name| name != &pad.net);
                        for r in &mut check_spec.nets {
                            if r.name == pad.net {
                                r.protected = false;
                            }
                        }
                    }
                    if board.check_candidate(&c, &check_spec).is_err() {
                        continue;
                    }
                    if !is_member {
                        let radius = rule.width_mm / 2.0 + rule.clearance_mm;
                        c.reservations.push(ReservedRegion {
                            owner_net: pad.net.clone(),
                            layer: layer.clone(),
                            min: [
                                pad.point[0].min(end[0]) - radius,
                                pad.point[1].min(end[1]) - radius,
                            ],
                            max: [
                                pad.point[0].max(end[0]) + radius,
                                pad.point[1].max(end[1]) + radius,
                            ],
                        });
                        c.legs.clear();
                    }
                    variants.push(c);
                }
            }
            if variants.is_empty() {
                return Err(format!(
                    "blocked pad {} net {}: no geometrically valid cardinal/diagonal escape",
                    pad.reference, pad.net
                ));
            }
            problem.selection_sets.push(SelectionSet {
                id: set,
                group_id: group.id.clone(),
                candidate_ids: variants.iter().map(|c| c.id.clone()).collect(),
            });
            problem.candidates.extend(variants.clone());
            escapes.insert(key, variants);
        }
        for name in &group.nets {
            let endpoints: Vec<_> = selected
                .iter()
                .copied()
                .filter(|p| &p.net == name)
                .collect();
            if endpoints.len() < 2 {
                return Err(format!(
                    "group {} net {name}: fewer than two endpoints",
                    group.id
                ));
            }
            for pair in endpoints.windows(2) {
                let a = pair[0];
                let b = pair[1];
                let rule = spec
                    .nets
                    .iter()
                    .find(|r| &r.name == name)
                    .ok_or("missing net rule")?;
                if rule.protected {
                    return Err(format!("protected net {name} cannot be routed"));
                }
                let set = format!("connect:{}:{}:{}", group.id, a.reference, b.reference);
                let mut routes = Vec::new();
                let aa = &escapes[&format!("{}:{}", group.id, a.reference)];
                let bb = &escapes[&format!("{}:{}", group.id, b.reference)];
                for ca in aa {
                    for cb in bb {
                        budget(deadline)?;
                        let la = &ca.legs[0];
                        let lb = &cb.legs[0];
                        if la.layer != lb.layer {
                            continue;
                        }
                        let x = *la.points.last().unwrap();
                        let y = *lb.points.last().unwrap();
                        let paths = vec![
                            vec![x, y],
                            vec![x, [x[0], y[1]], y],
                            vec![x, [y[0], x[1]], y],
                        ];
                        for (index, path) in paths.into_iter().enumerate() {
                            let mut path = path;
                            path.dedup();
                            if path.len() < 2 {
                                continue;
                            }
                            let c = candidate(
                                format!("{set}:{}:{}:{index}", ca.id, cb.id),
                                set.clone(),
                                &group.id,
                                vec![leg(a, rule, &la.layer, path)],
                                vec![vec![ca.id.clone()], vec![cb.id.clone()]],
                            );
                            if board.check_candidate(&c, spec).is_ok() {
                                routes.push(c);
                            }
                        }
                    }
                }
                // Native maze search also connects opposite-surface pad exits through
                // authorized via types; every resulting route remains an explicit candidate.
                for from_layer in &rule.allowed_layers {
                    for to_layer in &rule.allowed_layers {
                        budget(deadline)?;
                        if let (Some(ca), Some(cb)) = (
                            aa.iter().find(|c| c.legs[0].layer == *from_layer),
                            bb.iter().find(|c| c.legs[0].layer == *to_layer),
                        ) {
                            if let Ok((legs, vias)) =
                                maze_route(board, spec, ca, cb, rule, deadline)
                            {
                                let mut c = candidate(
                                    format!("{set}:maze:{from_layer}:{to_layer}"),
                                    set.clone(),
                                    &group.id,
                                    legs,
                                    vec![vec![ca.id.clone()], vec![cb.id.clone()]],
                                );
                                c.vias = vias;
                                if !c.legs.is_empty() && board.check_candidate(&c, spec).is_ok() {
                                    routes.push(c);
                                }
                            }
                        }
                    }
                }
                if routes.is_empty() {
                    return Err(format!("blocked group {} net {name}: no candidate fits engine obstacles and hard channels",group.id));
                }
                problem.selection_sets.push(SelectionSet {
                    id: set,
                    group_id: group.id.clone(),
                    candidate_ids: routes.iter().map(|c| c.id.clone()).collect(),
                });
                problem.candidates.extend(routes);
            }
        }
    }
    problem.conflicts =
        crate::conflict::build_conflicts(board, spec, &problem.candidates, deadline)?;
    Ok(problem)
}
fn maze_route(
    board: &BoardAdapter,
    spec: &RouteSpec,
    a: &PlanCandidate,
    b: &PlanCandidate,
    rule: &NetRule,
    deadline: Instant,
) -> Result<(Vec<RouteLeg>, Vec<PlanVia>), String> {
    let mut trial = board.clone();
    trial.insert_candidate(a, spec)?;
    trial.insert_candidate(b, spec)?;
    let before = trial.clone();
    let n = trial.net_number(&rule.name)?;
    let mut settings = RouterSettings::default();
    settings.set_layer_count(trial.board.layer_count() as usize);
    settings.vias_allowed = Some(rule.via_budget != Some(0));
    settings.automatic_neckdown = Some(false);
    settings.strict_drc = Some(true);
    for (i, l) in trial.board.layer_structure.layers.iter().enumerate() {
        settings.set_layer_active(i, rule.allowed_layers.contains(&l.name));
    }
    let class = trial.board.rules.nets.get(n).unwrap().get_net_class();
    let h = (rule.width_mm * trial.units_per_mm / 2.0).ceil();
    if !h.is_finite() || h <= 0.0 || h > 10_000_000.0 {
        return Err("invalid maze trace width".into());
    }
    let half = h as i32;
    trial.board.rules_mut().net_classes[class].set_trace_half_width(half);
    trial.board.rules_mut().net_classes[class].no_neckdown = true;
    let stop = StopToken::new();
    let token = stop.clone();
    let (tx, rx) = mpsc::channel();
    let remaining = deadline.saturating_duration_since(Instant::now());
    let timer = std::thread::spawn(move || {
        if rx.recv_timeout(remaining).is_err() {
            token.request_stop();
        }
    });
    let mut router = BatchAutorouter::for_job(&trial.board, &settings);
    router.net_filter = Some([n].into_iter().collect());
    router.pass_threads = 1;
    router.suppress_net_rip = true;
    router.autoroute_pass(&mut trial.board, &settings, 1, &stop);
    let _ = tx.send(());
    let _ = timer.join();
    budget(deadline)?;
    trial.assert_preserved(&before)?;
    let ids: BTreeSet<_> = before
        .board
        .item_repository()
        .iter()
        .map(|k| before.board.item(k).id().0)
        .collect();
    let legs = trial
        .board
        .get_traces()
        .into_iter()
        .filter(|&k| !ids.contains(&trial.board.item(k).id().0))
        .map(|k| {
            let trace = trial.board.item(k).trace();
            let layer = trial.board.layer_structure.layers[trace.layer() as usize]
                .name
                .clone();
            RouteLeg {
                net: rule.name.clone(),
                layer,
                points: trace
                    .polyline()
                    .corners()
                    .into_iter()
                    .map(|p| {
                        let p = p.to_float();
                        [p.x / trial.units_per_mm, p.y / trial.units_per_mm]
                    })
                    .collect(),
                width_mm: 2.0 * trace.half_width() as f64 / trial.units_per_mm,
                clearance_mm: rule.clearance_mm,
            }
        })
        .collect();
    let vias = trial
        .board
        .item_repository()
        .iter()
        .filter(|&k| trial.board.item(k).is_via() && !ids.contains(&trial.board.item(k).id().0))
        .map(|k| {
            let item = trial.board.item(k);
            let pad = item.padstack(&trial.board).unwrap();
            let p = item.center(&trial.board).to_float();
            let bounds = pad.get_shape(pad.from_layer()).unwrap().bounding_box();
            PlanVia {
                net: rule.name.clone(),
                point: [p.x / trial.units_per_mm, p.y / trial.units_per_mm],
                from_layer: trial.board.layer_structure.layers[pad.from_layer() as usize]
                    .name
                    .clone(),
                to_layer: trial.board.layer_structure.layers[pad.to_layer() as usize]
                    .name
                    .clone(),
                diameter_mm: bounds.width().max(bounds.height()) as f64 / trial.units_per_mm,
                padstack: pad.name.clone(),
            }
        })
        .collect();
    Ok((legs, vias))
}
pub(crate) fn reservation_conflict(a: &PlanCandidate, b: &PlanCandidate) -> bool {
    for r in &a.reservations {
        for s in &b.reservations {
            if r.owner_net != s.owner_net
                && r.layer == s.layer
                && r.min[0] <= s.max[0]
                && r.max[0] >= s.min[0]
                && r.min[1] <= s.max[1]
                && r.max[1] >= s.min[1]
            {
                return true;
            }
        }
        for l in &b.legs {
            if r.owner_net == l.net || r.layer != l.layer {
                continue;
            }
            let d = l.width_mm / 2.0 + l.clearance_mm;
            for s in l.points.windows(2) {
                if segment_hits_rect(
                    s[0],
                    s[1],
                    [r.min[0] - d, r.min[1] - d],
                    [r.max[0] + d, r.max[1] + d],
                ) {
                    return true;
                }
            }
        }
    }
    if !b.reservations.is_empty() && !a.legs.is_empty() {
        let mut x = a.clone();
        x.reservations.clear();
        let mut y = b.clone();
        y.legs.clear();
        return reservation_conflict(&y, &x);
    }
    false
}
fn segment_hits_rect(a: [f64; 2], b: [f64; 2], min: [f64; 2], max: [f64; 2]) -> bool {
    let mut lo: f64 = 0.0;
    let mut hi: f64 = 1.0;
    for i in 0..2 {
        let d = b[i] - a[i];
        if d.abs() < 1e-15 {
            if a[i] < min[i] || a[i] > max[i] {
                return false;
            }
        } else {
            let x = (min[i] - a[i]) / d;
            let y = (max[i] - a[i]) / d;
            lo = lo.max(x.min(y));
            hi = hi.min(x.max(y));
            if lo > hi {
                return false;
            }
        }
    }
    true
}

pub fn execute_selected(
    board: &BoardAdapter,
    spec: &RouteSpec,
    problem: &PlannerProblem,
    selected: &[String],
    deadline: Instant,
) -> Result<BoardAdapter, String> {
    problem.validate_selection(selected)?;
    let chosen: Vec<_> = problem
        .candidates
        .iter()
        .filter(|c| selected.contains(&c.id))
        .collect();
    for (i, a) in chosen.iter().enumerate() {
        for b in &chosen[i + 1..] {
            if reservation_conflict(a, b) {
                return Err("selected route violates reserved neighboring-pad exit".into());
            }
        }
    }
    let mut trial = board.clone();
    for group in ordered_groups(spec)? {
        budget(deadline)?;
        for c in chosen.iter().filter(|c| c.group_id == group.id) {
            trial.insert_candidate(c, spec)?;
        }
        trial.assert_preserved(board)?;
        for name in &group.nets {
            let rule = spec
                .nets
                .iter()
                .find(|r| &r.name == name)
                .ok_or("missing rule")?;
            let n = trial.net_number(name)?;
            for key in trial
                .board
                .get_traces()
                .into_iter()
                .filter(|&k| trial.board.item(k).contains_net(n))
            {
                let trace = trial.board.item(key).trace();
                let layer = trial.board.layer_structure.layers[trace.layer() as usize]
                    .name
                    .clone();
                let width = 2.0 * trace.half_width() as f64 / trial.units_per_mm;
                if width + 1e-12 < rule.width_mm || !rule.allowed_layers.contains(&layer) {
                    return Err(format!(
                        "existing/routed net {name} copper fails width/layer rule"
                    ));
                }
                let leg = RouteLeg {
                    net: name.clone(),
                    layer,
                    points: trace
                        .polyline()
                        .corner_approx_arr()
                        .into_iter()
                        .map(|p| [p.x / trial.units_per_mm, p.y / trial.units_per_mm])
                        .collect(),
                    width_mm: width,
                    clearance_mm: rule.clearance_mm,
                };
                let check = candidate(
                    format!("rule_check:{name}:{}", trial.board.item(key).id().0),
                    "check".into(),
                    "",
                    vec![leg],
                    vec![],
                );
                trial
                    .check_candidate(&check, spec)
                    .map_err(|e| format!("existing/routed net {name} clearance rule: {e}"))?;
            }

            let pins = trial
                .board
                .get_pins()
                .into_iter()
                .filter(|&k| trial.board.item(k).contains_net(n))
                .collect::<Vec<_>>();
            if let Some(&first) = pins.first() {
                let connected = trial.board.connected_set(first, n, false);
                if pins
                    .iter()
                    .any(|&k| !connected.contains(trial.board.item(k).id()))
                {
                    return Err(format!(
                        "group {} net {name}: incomplete connectivity",
                        group.id
                    ));
                }
            }
            let length = if rule.max_length_mm.is_some() {
                Some(two_terminal_path_length(&trial, name)?)
            } else {
                None
            };
            if rule
                .max_length_mm
                .is_some_and(|limit| length.is_some_and(|length| length > limit + 1e-9))
            {
                return Err(format!("net {name}: maximum length exceeded"));
            }
            let vias = trial
                .board
                .item_repository()
                .iter()
                .filter(|&k| trial.board.item(k).is_via() && trial.board.item(k).contains_net(n))
                .count();
            if vias > rule.via_budget.unwrap_or(u32::MAX) as usize {
                return Err(format!("net {name}: via budget exceeded"));
            }
        }
    }
    budget(deadline)?;
    trial.assert_preserved(board)?;
    Ok(trial)
}
pub fn ordered_groups(spec: &RouteSpec) -> Result<Vec<&crate::spec::RouteGroup>, String> {
    let mut ordered = Vec::new();
    let mut done = BTreeSet::new();
    while ordered.len() < spec.groups.len() {
        let mut ready: Vec<_> = spec
            .groups
            .iter()
            .filter(|g| !done.contains(&g.id) && g.depends_on.iter().all(|id| done.contains(id)))
            .collect();
        ready.sort_by(|a, b| b.priority.cmp(&a.priority).then(a.id.cmp(&b.id)));
        let next = ready
            .first()
            .ok_or("group dependency cycle or unknown dependency")?;
        done.insert(next.id.clone());
        ordered.push(*next);
    }
    Ok(ordered)
}
fn route_remaining(
    trial: &mut BoardAdapter,
    spec: &RouteSpec,
    chosen: &[&PlanCandidate],
    deadline: Instant,
) -> Result<(Vec<String>, bool), String> {
    let mut rejected = Vec::new();
    let before = trial.clone();
    let protected = spec.protected_names();
    let joint: BTreeSet<_> = spec
        .groups
        .iter()
        .flat_map(|g| g.nets.iter().cloned())
        .collect();
    if spec.groups.is_empty()
        && spec.channels.is_empty()
        && chosen.iter().all(|c| c.reservations.is_empty())
    {
        return route_remaining_parallel(trial, spec, &protected, deadline);
    }
    let nets: Vec<_> = trial
        .board
        .rules
        .nets
        .iter()
        .filter(|n| !joint.contains(&n.name) && !protected.contains(&n.name))
        .map(|n| (n.net_number, n.name.clone()))
        .collect();
    for (n, name) in nets {
        if budget(deadline).is_err() {
            return Ok((rejected, true));
        }
        let mut trial_net = trial.clone();
        let mut obstacles = Vec::new();
        for c in chosen {
            for r in &c.reservations {
                if r.owner_net == name {
                    continue;
                }
                let lo = trial_net.point(r.min)?.to_float();
                let hi = trial_net.point(r.max)?.to_float();
                let area = fr_geom::Area::Shape(fr_geom::Shape::Tile(fr_geom::TileShape::IntBox(
                    fr_geom::IntBox::new(
                        lo.x.floor() as i32,
                        lo.y.floor() as i32,
                        hi.x.ceil() as i32,
                        hi.y.ceil() as i32,
                    ),
                )));
                let layer = trial_net.board.layer_structure.get_no(&r.layer);
                obstacles.push(trial_net.board.insert_obstacle(
                    area,
                    layer,
                    1,
                    fr_engine::ids::FixedState::UserFixed,
                ));
            }
        }
        for ch in &spec.channels {
            let (min, max) = match &ch.region {
                crate::spec::ChannelRegion::Rectangle { min, max } => (*min, *max),
                crate::spec::ChannelRegion::Polyline { points, width_mm } => {
                    let r = width_mm / 2.0;
                    let mut min = [f64::INFINITY; 2];
                    let mut max = [f64::NEG_INFINITY; 2];
                    for p in points {
                        for i in 0..2 {
                            min[i] = min[i].min(p[i] - r);
                            max[i] = max[i].max(p[i] + r);
                        }
                    }
                    (min, max)
                }
            };
            let lo = trial_net.point(min)?.to_float();
            let hi = trial_net.point(max)?.to_float();
            let area = fr_geom::Area::Shape(fr_geom::Shape::Tile(fr_geom::TileShape::IntBox(
                fr_geom::IntBox::new(
                    lo.x.floor() as i32,
                    lo.y.floor() as i32,
                    hi.x.ceil() as i32,
                    hi.y.ceil() as i32,
                ),
            )));
            for layer in 0..trial_net.board.layer_count() {
                obstacles.push(trial_net.board.insert_obstacle(
                    area.clone(),
                    layer,
                    1,
                    fr_engine::ids::FixedState::UserFixed,
                ));
            }
        }
        let mut settings = RouterSettings::default();
        settings.set_layer_count(trial_net.board.layer_count() as usize);
        settings.strict_drc = Some(true);
        let stop = StopToken::new();
        let token = stop.clone();
        let (tx, rx) = mpsc::channel();
        let remaining = deadline.saturating_duration_since(Instant::now());
        let timer = std::thread::spawn(move || {
            if rx.recv_timeout(remaining).is_err() {
                token.request_stop();
            }
        });
        let mut router = BatchAutorouter::for_job(&trial_net.board, &settings);
        router.net_filter = Some([n].into_iter().collect());
        router.pass_threads = 1;
        for pass in 1..=8 {
            if stop.is_stop_requested() {
                break;
            }
            let (attempted, _) =
                router.autoroute_pass(&mut trial_net.board, &settings, pass, &stop);
            if !attempted {
                break;
            }
        }
        let _ = tx.send(());
        let _ = timer.join();
        let exhausted = Instant::now() >= deadline;
        for key in obstacles {
            trial_net.board.item_mut(key).unfix();
            trial_net.board.remove_item(key);
        }
        // Reject the entire per-net transaction when fixed/protected geometry, existing pin
        // connectivity, or the baseline violation set changes adversely.
        match trial_net.assert_invariants(&before, &protected, false) {
            Ok(()) => {
                if routing_snapshot(&trial_net) == routing_snapshot(trial) {
                    rejected.push(format!(
                        "{name}: validated attempt contained no routing increment"
                    ));
                }
                *trial = trial_net;
            }
            Err(error) => rejected.push(format!("{name}: {error}")),
        }
        if exhausted || Instant::now() >= deadline {
            return Ok((rejected, true));
        }
    }
    trial.assert_invariants(&before, &protected, false)?;
    Ok((rejected, false))
}

fn route_remaining_unconstrained_parallel(
    trial: &mut BoardAdapter,
    spec: &RouteSpec,
    protected: &BTreeSet<String>,
    deadline: Instant,
) -> Result<(Vec<String>, bool), String> {
    let before = trial.clone();
    let mut best = trial.clone();
    let mut rejected = Vec::new();
    let stop = StopToken::new();
    let token = stop.clone();
    let (tx, rx) = mpsc::channel();
    let remaining = deadline.saturating_duration_since(Instant::now());
    let timer = std::thread::spawn(move || {
        if rx.recv_timeout(remaining).is_err() {
            token.request_stop();
        }
    });
    for pass in 1..=8 {
        if stop.is_stop_requested() || Instant::now() >= deadline { break; }
        let mut attempt = best.clone();
        let frozen: Vec<_> = attempt.board.item_repository().iter()
            .filter(|&key| {
                let item = attempt.board.item(key);
                (item.is_trace() || item.is_via()) && item.net_numbers().iter().any(|&n| {
                    attempt.board.rules.nets.get(n).is_some_and(|net| protected.contains(&net.name))
                })
            })
            .map(|key| (attempt.board.item(key).id(), attempt.board.item(key).fixed_state()))
            .collect();
        for (id, previous_state) in &frozen {
            let key = { attempt.board.item_repository().iter().find(|&key| attempt.board.item(key).id() == *id) };
            if let Some(key) = key {
                attempt.board.item_mut(key).set_fixed_state((*previous_state).max(fr_engine::ids::FixedState::UserFixed));
            }
        }
        let mut settings = RouterSettings::default();
        settings.set_layer_count(attempt.board.layer_count() as usize);
        settings.strict_drc = Some(false);
        let mut router = BatchAutorouter::for_job(&attempt.board, &settings);
        router.pass_threads = spec.execution.threads;
        router.enhancements = true;
        router.suppress_net_rip = true;
        router.skip_items = Some(
            attempt.board.item_repository().iter()
                .filter(|&key| attempt.board.item(key).net_numbers().iter().any(|&n| {
                    attempt.board.rules.nets.get(n).is_some_and(|net| protected.contains(&net.name))
                }))
                .map(|key| attempt.board.item(key).id().0)
                .collect(),
        );
        let (attempted, _) = router.autoroute_pass(&mut attempt.board, &settings, pass, &stop);
        for (id, state) in &frozen {
            let key = { attempt.board.item_repository().iter().find(|&key| attempt.board.item(key).id() == *id) };
            if let Some(key) = key { attempt.board.item_mut(key).set_fixed_state(*state); }
        }
        match attempt.assert_invariants(&before, protected, false) {
            Ok(()) => {
                if routing_snapshot(&attempt) == routing_snapshot(&best) {
                    rejected.push(format!("parallel_pass_{pass}: validated attempt contained no routing increment"));
                }
                best = attempt;
            }
            Err(error) => rejected.push(format!("parallel_pass_{pass}: {error}")),
        }
        if !attempted || fr_engine::pipeline::stats::incomplete_count(&best.board, None) == 0 { break; }
    }
    let _ = tx.send(());
    let _ = timer.join();
    let exhausted = Instant::now() >= deadline;
    best.assert_invariants(&before, protected, false)?;
    *trial = best;
    Ok((rejected, exhausted))
}

fn remaining_channel_areas(board: &BoardAdapter, region: &crate::spec::ChannelRegion) -> Result<Vec<fr_geom::Area>, String> {
    match region {
        crate::spec::ChannelRegion::Rectangle { min, max } => {
            let lo = board.point(*min)?.to_float();
            let hi = board.point(*max)?.to_float();
            Ok(vec![fr_geom::Area::Shape(fr_geom::Shape::Tile(fr_geom::TileShape::IntBox(
                fr_geom::IntBox::new(lo.x.floor() as i32, lo.y.floor() as i32, hi.x.ceil() as i32, hi.y.ceil() as i32),
            )))])
        }
        crate::spec::ChannelRegion::Polyline { points, width_mm } => {
            let half_width = width_mm / 2.0;
            if points.len() < 2 || !half_width.is_finite() || half_width <= 0.0 {
                return Err("reserved polyline channel has insufficient geometry".into());
            }
            let mut areas = Vec::new();
            let radius = (half_width * board.units_per_mm).ceil() as i32;
            for point in points {
                let center = match board.point(*point)? {
                    fr_geom::Point::Int(point) => point,
                    _ => return Err("reserved polyline channel point is not native integer geometry".into()),
                };
                areas.push(fr_geom::Area::Shape(fr_geom::Shape::Circle(fr_geom::Circle::new(center, radius))));
            }
            for segment in points.windows(2) {
                let mut left = crate::awx::pairs::offset_polyline(segment, half_width);
                let mut right = crate::awx::pairs::offset_polyline(segment, -half_width);
                right.reverse();
                left.extend(right);
                let corners: Vec<_> = left.into_iter().map(|point| board.point(point)).collect::<Result<_, _>>()?;
                let polygon = fr_geom::PolygonShape::from_points(&corners);
                if fr_geom::PolylineShapeImpl::dimension(&polygon) >= 2 {
                    areas.push(fr_geom::Area::Shape(fr_geom::Shape::Polygon(polygon)));
                }
            }
            Ok(areas)
        }
    }
}

fn route_remaining_parallel(
    trial: &mut BoardAdapter,
    spec: &RouteSpec,
    protected: &BTreeSet<String>,
    deadline: Instant,
) -> Result<(Vec<String>, bool), String> {
    let eligible: BTreeSet<_> = trial.board.rules.nets.iter()
        .filter(|net| !protected.contains(&net.name))
        .map(|net| net.name.clone())
        .collect();
    let has_net_rules = spec.nets.iter().any(|rule| eligible.contains(&rule.name));
    let has_foreign_channels = spec.groups.iter().any(|owner| {
        owner.channels.iter().any(|channel| eligible.iter().any(|net| {
            !owner.nets.contains(net)
                && !spec.groups.iter().any(|group| group.nets.contains(net) && group.channels.contains(channel))
        }))
    });
    if !has_net_rules && !has_foreign_channels {
        return route_remaining_unconstrained_parallel(trial, spec, protected, deadline);
    }
    let before = trial.clone();
    let mut best = trial.clone();
    let mut rejected = Vec::new();
    let stop = StopToken::new();
    let token = stop.clone();
    let (tx, rx) = mpsc::channel();
    let remaining = deadline.saturating_duration_since(Instant::now());
    let timer = std::thread::spawn(move || {
        if rx.recv_timeout(remaining).is_err() {
            token.request_stop();
        }
    });
    let nets: Vec<_> = best.board.rules.nets.iter()
        .filter(|net| !protected.contains(&net.name))
        .map(|net| (net.net_number, net.name.clone()))
        .collect();
    for pass in 1..=8 {
      if stop.is_stop_requested() || Instant::now() >= deadline { break; }
      let mut attempted_any = false;
      for (net_no, name) in &nets {
        if stop.is_stop_requested() || Instant::now() >= deadline { break; }
        let connectable = best.board.get_connectable_items(*net_no);
        if fr_engine::drc::NetIncompletes::new(&best.board, *net_no, &connectable).count() == 0 {
            continue;
        }
        let rule = spec.nets.iter().find(|rule| rule.name == *name);
        let mut settings = RouterSettings::default();
        settings.set_layer_count(best.board.layer_count() as usize);
        settings.strict_drc = Some(true);
        if let Some(rule) = rule {
            settings.vias_allowed = Some(rule.via_budget != Some(0));
            for (index, layer) in best.board.layer_structure.layers.iter().enumerate() {
                settings.set_layer_active(index, rule.allowed_layers.contains(&layer.name));
            }
        }
            let mut attempt = best.clone();
            let frozen: Vec<_> = attempt.board.item_repository().iter()
                .filter(|&key| {
                    let item = attempt.board.item(key);
                    (item.is_trace() || item.is_via()) && item.net_numbers().iter().any(|&number| {
                        attempt.board.rules.nets.get(number)
                            .is_some_and(|net| protected.contains(&net.name))
                    })
                })
                .map(|key| (attempt.board.item(key).id(), attempt.board.item(key).fixed_state()))
                .collect();
            for (id, state) in &frozen {
                let key = {
                    attempt.board.item_repository().iter()
                        .find(|&key| attempt.board.item(key).id() == *id)
                };
                if let Some(key) = key {
                    attempt.board.item_mut(key)
                        .set_fixed_state((*state).max(fr_engine::ids::FixedState::UserFixed));
                }
            }
            let class_id = attempt.board.rules.nets.get(*net_no)
                .ok_or_else(|| format!("remaining net {name} disappeared from board rules"))?
                .get_net_class();
            let original_class = attempt.board.rules.net_classes[class_id].clone();
            if let Some(rule) = rule {
                let half_width = (rule.width_mm * attempt.units_per_mm / 2.0).ceil();
                if !half_width.is_finite() || half_width <= 0.0 || half_width > 10_000_000.0 {
                    return Err(format!("invalid deferred trace width for net {name}"));
                }
                let net_class = &mut attempt.board.rules_mut().net_classes[class_id];
                net_class.set_trace_half_width(half_width as i32);
                net_class.no_neckdown = true;
                let desired = (rule.clearance_mm * attempt.units_per_mm).ceil() as i32;
                let base = net_class.get_trace_clearance_class();
                let class_name = format!("fr_agent_remaining_{net_no}_{desired}");
                let layer_count = attempt.board.layer_count();
                let matrix = &mut attempt.board.rules_mut().clearance_matrix;
                let clearance_class = if matrix.get_no(&class_name) >= 0 {
                    matrix.get_no(&class_name)
                } else {
                    matrix.append_class(&class_name);
                    let created = matrix.get_no(&class_name);
                    for layer in 0..layer_count {
                        for other in 0..created {
                            let old = matrix.get_value(base, other, layer, false)
                                .max(matrix.get_value(other, base, layer, false));
                            matrix.set_value(created, other, layer, old.max(desired));
                            matrix.set_value(other, created, layer, old.max(desired));
                        }
                    }
                    created
                };
                attempt.board.rules_mut().net_classes[class_id].set_trace_clearance_class(clearance_class);
            }
            let channel_ids: BTreeSet<_> = spec.groups.iter()
                .filter(|group| group.nets.contains(name))
                .flat_map(|group| group.channels.iter().cloned())
                .collect();
            let mut channel_obstacles = Vec::new();
            for channel in spec.channels.iter().filter(|channel| {
                !channel_ids.contains(&channel.id)
                    && spec.groups.iter().any(|group| {
                        !group.nets.contains(name) && group.channels.contains(&channel.id)
                    })
            }) {
                let areas = remaining_channel_areas(&attempt, &channel.region)?;
                for area in areas {
                    for layer in 0..attempt.board.layer_count() {
                        channel_obstacles.push(attempt.board.insert_obstacle(
                            area.clone(), layer, 1, fr_engine::ids::FixedState::UserFixed,
                        ));
                    }
                }
            }
            let mut router = BatchAutorouter::for_job(&attempt.board, &settings);
            router.net_filter = Some([*net_no].into_iter().collect());
            router.pass_threads = spec.execution.threads;
            router.enhancements = true;
            router.suppress_net_rip = true;
            router.skip_items = Some(
                attempt.board.item_repository().iter()
                    .filter(|&key| attempt.board.item(key).net_numbers().iter().any(|&number| {
                        attempt.board.rules.nets.get(number)
                            .is_some_and(|net| protected.contains(&net.name))
                    }))
                    .map(|key| attempt.board.item(key).id().0)
                    .collect(),
            );
            let (attempted, _) = router.autoroute_pass(&mut attempt.board, &settings, pass, &stop);
            attempted_any |= attempted;
            for key in channel_obstacles {
                attempt.board.item_mut(key).unfix();
                attempt.board.remove_item(key);
            }
            for (id, state) in &frozen {
                let key = {
                    attempt.board.item_repository().iter()
                        .find(|&key| attempt.board.item(key).id() == *id)
                };
                if let Some(key) = key {
                    attempt.board.item_mut(key).set_fixed_state(*state);
                }
            }
            attempt.board.rules_mut().net_classes[class_id] = original_class;
            if let Some(limit) = rule.and_then(|rule| rule.via_budget) {
                let via_count = attempt.board.get_vias().into_iter()
                    .filter(|&key| attempt.board.item(key).contains_net(*net_no)).count();
                if via_count > limit as usize {
                    rejected.push(format!("{name}: constrained attempt exceeded via budget {limit}"));
                    continue;
                }
            }
            match attempt.assert_invariants(&before, protected, false) {
                Ok(()) => {
                    if routing_snapshot(&attempt) == routing_snapshot(&best) {
                        rejected.push(format!("{name}: validated attempt contained no routing increment"));
                    } else {
                        best = attempt;
                    }
                }
                Err(error) => rejected.push(format!("{name}: {error}")),
            }
      }
      if !attempted_any || fr_engine::pipeline::stats::incomplete_count(&best.board, None) == 0 {
          break;
      }
    }
    for (net_no, name) in &nets {
        let connectable = best.board.get_connectable_items(*net_no);
        if fr_engine::drc::NetIncompletes::new(&best.board, *net_no, &connectable).count() > 0 {
            rejected.push(format!("{name}: remains incomplete after eight constrained autoroute passes"));
        }
    }
    let _ = tx.send(());
    let _ = timer.join();
    let exhausted = Instant::now() >= deadline;
    best.assert_invariants(&before, protected, false)?;
    *trial = best;
    Ok((rejected, exhausted))
}

fn merge_requested_scope(seed: &RouteSpec, requested: &RouteSpec) -> Result<RouteSpec, String> {
    let mut merged = seed.clone();
    for requested_rule in &requested.nets {
        if let Some(seed_rule) = merged.nets.iter().find(|rule| rule.name == requested_rule.name) {
            if seed_rule.width_mm.to_bits() != requested_rule.width_mm.to_bits()
                || seed_rule.clearance_mm.to_bits() != requested_rule.clearance_mm.to_bits()
                || seed_rule.allowed_layers.iter().collect::<BTreeSet<_>>()
                    != requested_rule.allowed_layers.iter().collect::<BTreeSet<_>>()
                || seed_rule.via_budget != requested_rule.via_budget
                || seed_rule.max_length_mm.map(f64::to_bits) != requested_rule.max_length_mm.map(f64::to_bits)
                || seed_rule.protected != requested_rule.protected
            {
                return Err(format!("seed and controller hard rules conflict for net {}", requested_rule.name));
            }
        } else {
            merged.nets.push(requested_rule.clone());
        }
    }
    for requested_group in &requested.groups {
        if let Some(seed_group) = merged.groups.iter_mut().find(|group| group.id == requested_group.id) {
            if seed_group.priority != requested_group.priority || seed_group.joint_escape != requested_group.joint_escape {
                return Err(format!("seed and controller group settings conflict for {}", requested_group.id));
            }
            let mut awx_pairs_to_add = Vec::new();
            if let (Some(seed_awx), Some(requested_awx)) = (&seed_group.awx, &requested_group.awx) {
                if seed_awx.source != requested_awx.source
                    || seed_awx.destination != requested_awx.destination
                    || seed_awx.routing_layers != requested_awx.routing_layers
                    || seed_awx.fanout_rounds != requested_awx.fanout_rounds
                    || seed_awx.geometry_rounds != requested_awx.geometry_rounds
                    || seed_awx.allow_unproved != requested_awx.allow_unproved
                {
                    return Err(format!("seed and controller AWX settings conflict for group {}", requested_group.id));
                }
                for requested_pair in &requested_awx.pairs {
                    if let Some(seed_pair) = seed_awx.pairs.iter().find(|pair| pair.base == requested_pair.base) {
                        if seed_pair.p != requested_pair.p || seed_pair.n != requested_pair.n {
                            return Err(format!("seed and controller AWX pair conflicts for {}", requested_pair.base));
                        }
                    } else {
                        awx_pairs_to_add.push(requested_pair.clone());
                    }
                }
            } else if seed_group.awx.is_none() {
                seed_group.awx = requested_group.awx.clone();
            }
            if let Some(seed_awx) = seed_group.awx.as_mut() {
                seed_awx.pairs.extend(awx_pairs_to_add);
            }
            for name in &requested_group.nets {
                if !seed_group.nets.contains(name) { seed_group.nets.push(name.clone()); }
            }
            for id in &requested_group.endpoint_groups {
                if !seed_group.endpoint_groups.contains(id) { seed_group.endpoint_groups.push(id.clone()); }
            }
            for id in &requested_group.channels {
                if !seed_group.channels.contains(id) { seed_group.channels.push(id.clone()); }
            }
            for id in &requested_group.depends_on {
                if !seed_group.depends_on.contains(id) { seed_group.depends_on.push(id.clone()); }
            }
        } else {
            merged.groups.push(requested_group.clone());
        }
    }
    for requested_channel in &requested.channels {
        if let Some(seed_channel) = merged.channels.iter().find(|channel| channel.id == requested_channel.id) {
            if serde_json::to_value(&seed_channel.region).ok() != serde_json::to_value(&requested_channel.region).ok() {
                return Err(format!("seed and controller channel rules conflict for {}", requested_channel.id));
            }
        } else {
            merged.channels.push(requested_channel.clone());
        }
    }
    for requested_endpoints in &requested.endpoint_groups {
        if let Some(seed_endpoints) = merged.endpoint_groups.iter_mut().find(|group| group.id == requested_endpoints.id) {
            for endpoint in &requested_endpoints.endpoints {
                if let Some(existing) = seed_endpoints.endpoints.iter().find(|known| known.chip == endpoint.chip && known.pad == endpoint.pad) {
                    if existing.net != endpoint.net {
                        return Err(format!("seed and controller endpoint rules conflict for {}.{}", endpoint.chip, endpoint.pad));
                    }
                } else {
                    seed_endpoints.endpoints.push(endpoint.clone());
                }
            }
        } else {
            merged.endpoint_groups.push(requested_endpoints.clone());
        }
    }
    for protected in &requested.protected_nets {
        if !merged.protected_nets.contains(protected) { merged.protected_nets.push(protected.clone()); }
    }
    Ok(merged)
}

fn verify_all_requested_rules(board: &BoardAdapter, spec: &RouteSpec) -> Result<(), String> {
    let groups = ordered_groups(spec)?;
    if !groups.is_empty() {
        crate::awx::integration::verify_requested(board, spec, &groups)?;
    }
    let grouped_nets: BTreeSet<_> = spec.groups.iter().flat_map(|group| group.nets.iter().cloned()).collect();
    for rule in spec.nets.iter().filter(|rule| !grouped_nets.contains(&rule.name)) {
        let group = crate::spec::RouteGroup {
            id: format!("__net_rule_verification__{}", rule.name),
            nets: vec![rule.name.clone()],
            endpoint_groups: Vec::new(),
            channels: Vec::new(),
            priority: 0,
            depends_on: Vec::new(),
            joint_escape: false,
            awx: None,
        };
        crate::awx::integration::verify_requested(board, spec, &[&group])?;
    }
    Ok(())
}

pub fn run(loaded: &LoadedSpec, dry_run: bool) -> Result<ExecutionReport, String> {
    loaded.spec.validate().map_err(|e| e.to_string())?;
    let evolve_enabled = loaded
        .spec
        .awx
        .as_ref()
        .is_some_and(|awx| awx.evolve.enabled);
    let bus_discovery_enabled = loaded
        .spec
        .awx
        .as_ref()
        .is_some_and(|awx| awx.bus_discovery.enabled);
    let awx_groups_enabled = !loaded.spec.groups.is_empty() && loaded.spec.awx.is_some();
    if awx_groups_enabled || evolve_enabled || bus_discovery_enabled {
        return run_awx(loaded, dry_run);
    }
    if loaded.spec.nets.iter().any(|r| r.protected) {
        return Err("Unsupported numeric protected NetRule; use protected_nets for freeze-only declarations".into());
    }
    let start = Instant::now();
    let deadline = start + Duration::from_millis(loaded.spec.execution.budget_ms);
    let bytes = std::fs::read(&loaded.spec.input).map_err(|e| e.to_string())?;
    let board = BoardAdapter::load(&bytes)?;
    for name in loaded.spec.protected_names() {
        board
            .net_number(&name)
            .map_err(|e| format!("protected net board validation: {e}"))?;
    }
    let mut problem = generate_problem(&board, &loaded.spec, deadline)?;
    let mut iterations = 0;
    let mut solve_seconds = 0.0;
    let validation = Instant::now();
    let (mut trial, outcome) = loop {
        budget(deadline)?;
        let remaining = deadline
            .saturating_duration_since(Instant::now())
            .as_millis() as u64;
        let outcome = solve_joint(
            &problem,
            loaded.spec.execution.threads,
            remaining.max(1),
            loaded.spec.execution.seed,
        )?;
        solve_seconds += outcome.wall_time_seconds;
        if !matches!(outcome.status, PlanStatus::Optimal | PlanStatus::Feasible) {
            return Err(format!("finite candidate model {:?} after {iterations} geometry feedback cuts; no SES written",outcome.status));
        }
        budget(deadline)?;
        match execute_selected(
            &board,
            &loaded.spec,
            &problem,
            &outcome.selected_ids,
            deadline,
        ) {
            Ok(trial) => break (trial, outcome),
            Err(error) => {
                budget(deadline)?;
                iterations += 1;
                if iterations >= 64 {
                    return Err(format!("geometry feedback limit: {error}; no SES written"));
                }
                problem.capacities.push(CapacityConstraint {
                    candidate_ids: outcome.selected_ids.clone(),
                    capacity: outcome.selected_ids.len().saturating_sub(1),
                });
            }
        }
    };
    let mut rejected_net_attempts = Vec::new();
    let mut deadline_exhausted = false;
    if loaded.spec.route_remaining {
        let chosen: Vec<_> = problem
            .candidates
            .iter()
            .filter(|c| outcome.selected_ids.contains(&c.id))
            .collect();
        let outcome = route_remaining(&mut trial, &loaded.spec, &chosen, deadline)?;
        rejected_net_attempts = outcome.0;
        deadline_exhausted = outcome.1;
    }
    let validation_seconds = validation.elapsed().as_secs_f64() - solve_seconds;
    let remaining_incomplete = fr_engine::pipeline::stats::incomplete_count(&trial.board, None);
    let success =
        !deadline_exhausted && (!loaded.spec.route_remaining || remaining_incomplete == 0);
    let output_committed = !dry_run && (success || loaded.spec.allow_partial_output);
    if output_committed {
        if !loaded.spec.allow_partial_output {
            budget(deadline)?;
        }
        let output = trial.ses();
        atomic_export(&loaded.spec.output, &output)?;
    }
    let mut rules = loaded.spec.structure_reports();
    for r in &mut rules {
        r.status = crate::spec::RuleStatus::Executed;
        r.detail = "executed with full-board geometry/connectivity/DRC regression checks".into();
        if r.rule.starts_with("net.") {
            let target = loaded.spec.nets.iter().find(|n| {
                [
                    "width_mm",
                    "clearance_mm",
                    "allowed_layers",
                    "via_budget",
                    "max_length_mm",
                    "protected",
                ]
                .iter()
                .any(|attribute| r.rule == format!("net.{}.{attribute}", n.name))
            });
            let active =
                target.is_some_and(|n| loaded.spec.groups.iter().any(|g| g.nets.contains(&n.name)));
            if !active && !r.rule.ends_with(".protected") {
                r.status = crate::spec::RuleStatus::Supported;
                r.detail="legacy numeric rule retained; not certified by preserve-only or unselected-net operation".into();
            }
        }

        if r.rule.starts_with("protected_nets.") {
            r.detail="executed: original net ownership, entity geometry and fixed state preserved; no new protected copper".into();
        }
        if r.rule == "route_remaining" && !success {
            r.status = crate::spec::RuleStatus::Unmet;
            r.detail = format!(
                "remaining={remaining_incomplete}, deadline_exhausted={deadline_exhausted}"
            );
        }
    }
    let selected_candidates = problem
        .candidates
        .iter()
        .filter(|c| outcome.selected_ids.contains(&c.id))
        .cloned()
        .collect();
    let routing_changes = routing_changes(&board, &trial);
    Ok(ExecutionReport {
        awx_buses: Vec::new(),
        awx_statistics: None,
        awx_bus_discovery: None,
        awx_evolution: None,
        awx_seed_manifests: None,
        selected_candidates,
        routing_changes,
        routing_changes_base: "normalized_input".into(),
        routing_counts_before: routing_counts(&board),
        routing_counts_after: routing_counts(&trial),
        solver_workers: loaded.spec.execution.threads,
        conflict_workers: 1,
        detail_workers: if loaded.spec.groups.is_empty() && loaded.spec.channels.is_empty() {
            loaded.spec.execution.threads
        } else {
            1
        },
        success,
        output_committed,
        deadline_exhausted,
        rejected_net_attempts,
        rule_scope: "selected_group_entire_net_numeric_rules; protected_legacy_geometry_preserved"
            .into(),
        length_metric: "planar_trace_length".into(),
        channel_protection:
            "conservative_whole_channel_bounding_rectangle_on_all_layers_in_remaining_phase".into(),
        rules,
        stages: vec![
            "stage0_board_rules".into(),
            "stage1_escape_and_channels".into(),
            "stage2_joint_cp_sat".into(),
            "stage3_geometry_transaction".into(),
            "stage4_verification_export".into(),
        ],
        candidate_count: problem.candidates.len(),
        conflict_count: problem.conflicts.len(),
        selected_ids: outcome.selected_ids,
        solve_seconds,
        validation_seconds,
        elapsed_seconds: start.elapsed().as_secs_f64(),
        feedback_iterations: iterations,
        solver_scope: "candidate_model".into(),
        output_written: output_committed,
        routed_groups: ordered_groups(&loaded.spec)?
            .iter()
            .map(|g| g.id.clone())
            .collect(),
        remaining_incomplete,
    })
}

fn export_seed_captures(
    root: &std::path::Path,
    captures: &[crate::awx::integration::AwxSeedCapture],
) -> Result<Vec<std::path::PathBuf>, String> {
    if captures.is_empty() { return Ok(Vec::new()); }
    let parent=root.parent().filter(|path|!path.as_os_str().is_empty()).unwrap_or_else(||std::path::Path::new("."));
    if !parent.is_dir() { return Err(format!("seed export parent does not exist: {}",parent.display())); }
    std::fs::create_dir(root).map_err(|e|format!("create seed export root {}: {e}",root.display()))?;
    let empty_verdicts=std::collections::BTreeMap::<String,crate::awx::replan_types::NetVerdict>::new();
    let result = (|| {
        let mut manifests=Vec::new();
        for (index,capture) in captures.iter().enumerate() {
            let bundle=root.join(format!("bus_{:04}",index+1));
            manifests.push(crate::awx::evolution_export::export_native_seed_bundle(
                crate::awx::evolution_export::NativeSeedExportInput {
                    board_dsn:&capture.board_dsn,
                    output_dir:&bundle,
                    routed:&capture.routed,
                    fanout:&capture.fanout,
                    spec:&capture.spec,
                    requested_k:capture.requested_k,
                    destination:&capture.destination,
                    names:&capture.names,
                    context:&capture.context,
                    flow:capture.flow,
                    plan_seed:&capture.plan_seed,
                    census:None,
                    verdicts:&empty_verdicts,
                },
            )?);
        }
        Ok(manifests)
    })();
    if result.is_err() {
        let _ = std::fs::remove_dir_all(root);
    }
    result
}

fn run_awx(loaded: &LoadedSpec, dry_run: bool) -> Result<ExecutionReport, String> {
    let start = Instant::now();
    if loaded.spec.awx.as_ref().is_some_and(|awx| awx.evolve.enabled) {
        return run_awx_evolution(loaded, dry_run, start);
    }
    let deadline = start + Duration::from_millis(loaded.spec.execution.budget_ms);
    let source_dsn = std::fs::read(&loaded.spec.input).map_err(|e| e.to_string())?;
    let board = BoardAdapter::load(&source_dsn)?;
    let awx = crate::awx::fanout_driver::execute(&board, &loaded.spec, deadline)?;
    let discovery_only = loaded.spec.groups.is_empty()
        && !loaded.spec.route_remaining
        && awx.discovered_buses.is_some();
    let mut trial = awx.board;
    let mut rejected = Vec::new();
    let mut exhausted = Instant::now() >= deadline;
    let run_remaining = !discovery_only
        && loaded.spec.route_remaining
        && (awx.all_requested_routed || !awx.deferred_nets.is_empty())
        && !exhausted;
    if run_remaining {
        let mut remaining_spec = loaded.spec.clone();
        remaining_spec.protected_nets = awx.protected.iter().cloned().collect();
        // The whole transaction has already certified every requested net.
        // Remaining native routing freezes that copper and never substitutes it.
        let result = route_remaining_parallel(&mut trial, &remaining_spec, &awx.protected, deadline)?;
        rejected = result.0;
        exhausted = result.1;
    }
    trial.assert_invariants_with_moved_components(
        &board, &loaded.spec.protected_names(), false, &awx.moved_components,
    )?;
    if !discovery_only {
        verify_all_requested_rules(&trial, &loaded.spec)?;
    }
    if !awx.moved_components.is_empty() {
        verify_placement_session_replay(&source_dsn, &trial, &awx.moved_components)?;
    }
    let remaining_incomplete = fr_engine::pipeline::stats::incomplete_count(&trial.board, None);
    let all_awx_nets_complete = awx.all_requested_routed && awx.deferred_nets.is_empty();
    let remaining_completed = run_remaining && !exhausted && remaining_incomplete == 0;
    let success = discovery_only
        || (!exhausted && (all_awx_nets_complete || remaining_completed)
            && (!loaded.spec.route_remaining || remaining_incomplete == 0));
    let committed = !dry_run
        && !discovery_only
        && (success || loaded.spec.allow_partial_output);
    if committed {
        atomic_export(&loaded.spec.output, &trial.ses())?;
    }
    let seed_manifests = if let Some(directory)=loaded.spec.awx.as_ref().and_then(|a|a.seed_export.directory.as_ref()) {
        if dry_run { None } else { Some(export_seed_captures(directory,&awx.seed_captures)?) }
    } else { None };
    let mut rules = loaded.spec.structure_reports();
    for rule in &mut rules {
        rule.status = if discovery_only {
            crate::spec::RuleStatus::Supported
        } else if success {
            crate::spec::RuleStatus::Executed
        } else {
            crate::spec::RuleStatus::Unmet
        };
        rule.detail = if discovery_only {
            "bus discovery executed against the native board; no routing or session output was requested".into()
        } else {
            "AWX joint/whole transaction validated against native board geometry, connectivity and requested hard rules".into()
        };
        if !discovery_only && rule.rule.starts_with("protected_nets.") {
            rule.status = crate::spec::RuleStatus::Executed;
            rule.detail = "original protected copper and fixed state preserved".into();
        }
        if rule.rule == "route_remaining" && !success {
            rule.status = crate::spec::RuleStatus::Unmet;
            rule.detail = format!("remaining={remaining_incomplete}, deadline_exhausted={exhausted}");
        }
    }
    let rounds = awx.buses.iter().map(|b| b.rounds).sum();
    let awx_statistics = crate::awx::runtime_stats::AwxRunStats::from_buses(
        &awx.buses, Some(&awx.runtime_events),
    );
    let routed_groups = if success {
        ordered_groups(&loaded.spec)?.iter().map(|g|g.id.clone()).collect()
    } else {
        awx.buses.iter().filter(|b| b.deferred_ladder_nets.is_empty()
            && b.grade.open.is_empty() && b.grade.new_violations == 0)
            .flat_map(|b| b.groups.clone()).collect()
    };
    Ok(ExecutionReport {
        success, output_committed: committed, output_written: committed,
        deadline_exhausted: exhausted, rule_scope: "whole_bus_transaction_native_certified".into(),
        length_metric: "endpoint_copper_path_length".into(),
        channel_protection: "requested_native_copper_envelope".into(), rules,
        solver_workers: loaded.spec.execution.threads, conflict_workers: 1, detail_workers: 1,
        rejected_net_attempts: rejected,
        stages: if discovery_only {
            vec!["awx_bus_discovery".into()]
        } else {
            vec!["awx_joint_escape".into(), "awx_passive_clearance".into(), "awx_whole_solve_geometry_snap_polish_audit".into(), "native_transaction_certification".into(), "verification_export".into()]
        },
        candidate_count: 0, conflict_count: 0, selected_ids: Vec::new(), selected_candidates: Vec::new(),
        routing_changes: routing_changes(&board, &trial), routing_changes_base: "normalized_input".into(),
        routing_counts_before: routing_counts(&board), routing_counts_after: routing_counts(&trial),
        solve_seconds: start.elapsed().as_secs_f64(), validation_seconds: 0.,
        elapsed_seconds: start.elapsed().as_secs_f64(), feedback_iterations: rounds,
        solver_scope: "awx_joint_and_whole_cp_sat".into(), routed_groups, remaining_incomplete,
        awx_buses: awx.buses,
        awx_statistics: Some(awx_statistics),
        awx_bus_discovery: awx.discovered_buses,
        awx_evolution: None,
        awx_seed_manifests: seed_manifests,
    })
}

fn run_awx_evolution(loaded: &LoadedSpec, dry_run: bool, start: Instant) -> Result<ExecutionReport, String> {
    use crate::awx::{
        evolution_native::{self, NativeEvolutionOperators},
        route_evolution::{self, DescentOptions, EvolutionConfig},
    };
    use crate::spec::config_options::{CoupledMode, GradeMode};

    let spec = &loaded.spec;
    let awx = spec.awx.as_ref().ok_or("missing AWX evolution configuration")?;
    let options = &awx.evolve;
    let tag = options.tag.as_deref().ok_or("awx.evolve.tag is required")?;
    let requested_k = options.requested_k.ok_or("awx.evolve.requested_k is required")?;
    let board_path = options.board.as_ref().ok_or("awx.evolve.board is required")?;
    let base_bytes = std::fs::read(board_path).map_err(|e| format!("{}: {e}",board_path.display()))?;
    let base_board = BoardAdapter::load(&base_bytes)?;
    let seed_base = loaded.config_path.parent().ok_or("configuration has no parent directory")?.to_path_buf();

    let descent = DescentOptions {
        rounds: options.descent.rounds,
        worst: options.descent.worst,
        probes: options.descent.probes,
        min_vias: options.descent.min_vias,
        coupled: match options.descent.coupled { CoupledMode::Census => "census".into() },
        grade: match options.descent.grade { GradeMode::Sub => "sub".into(), GradeMode::Inproc => "inproc".into() },
    };
    let config = EvolutionConfig {
        population: options.population,
        generations: options.generations,
        jumps: options.jumps,
        crossovers: options.crossovers,
        jobs: options.jobs,
        jump_nets: options.jump_nets,
        seed: options.seed,
        descent,
    };
    let as_env = |values: &std::collections::BTreeMap<String,String>| -> Vec<(String,String)> {
        values.iter().map(|(key,value)|(key.clone(),value.clone())).collect()
    };
    let operators = NativeEvolutionOperators::native(spec.clone(), seed_base);
    let evolution = route_evolution::evolve(
        &operators,
        &config,
        &options.seeds,
        requested_k,
        board_path,
        &options.destination,
        &as_env(&options.descend_env),
        &as_env(&options.jump_env),
    )?;

    // The seed manifest carries the actual route scope. A source-compatible
    // evolve job can therefore leave groups/nets empty in its input TOML.
    let route_spec = &evolution.best.data.spec;
    let requested_spec = merge_requested_scope(route_spec, spec)?;
    let mut trial = evolution.best.data.routed.clone();
    let selected: BTreeSet<_> = route_spec.nets.iter().map(|net|net.name.clone()).collect();
    let mut protected_for_remaining = spec.protected_names();
    protected_for_remaining.extend(route_spec.protected_names());
    let protected: BTreeSet<_> = protected_for_remaining.union(&selected).cloned().collect();
    let mut rejected_net_attempts = Vec::new();
    let mut deadline_exhausted = Instant::now() >= start + Duration::from_millis(spec.execution.budget_ms);
    let mut remaining_ran = false;
    if spec.route_remaining && !deadline_exhausted {
        let mut remaining_spec=requested_spec.clone();
        remaining_spec.route_remaining=spec.route_remaining;
        remaining_spec.allow_partial_output=spec.allow_partial_output;
        remaining_spec.execution=spec.execution.clone();
        remaining_spec.protected_nets=protected.iter().cloned().collect();
        let result=route_remaining_parallel(&mut trial,&remaining_spec,&protected,start+Duration::from_millis(spec.execution.budget_ms))?;
        rejected_net_attempts=result.0;
        deadline_exhausted=result.1;
        remaining_ran=true;
    }
    trial.assert_invariants(&base_board,&protected_for_remaining,false)?;
    verify_all_requested_rules(&trial, &requested_spec)?;
    let grade=evolution_native::grade_board(&trial,&requested_spec)?;
    let remaining_incomplete=fr_engine::pipeline::stats::incomplete_count(&trial.board,None);
    let success=grade.opens.is_empty() && grade.drc==0 && !deadline_exhausted
        && (!remaining_ran || remaining_incomplete==0);
    let committed=!dry_run && (success || spec.allow_partial_output);
    let (seed_manifests, ledger_path) = if dry_run {
        (None, None)
    } else {
        let output_dir = loaded
            .evolve_output_directory
            .as_deref()
            .ok_or("missing resolved AWX evolve output directory")?;
        if let Some(parent) = output_dir.parent() {
            std::fs::create_dir_all(parent).map_err(|e| {
                format!("create AWX evolve output root {}: {e}", parent.display())
            })?;
        }
        std::fs::create_dir(output_dir).map_err(|e| {
            format!("create AWX evolve tag directory {}: {e}", output_dir.display())
        })?;
        let exported = (|| {
            for (generation_index, worlds) in evolution.generations.iter().enumerate() {
                let generation_dir = output_dir.join(format!("g{}", generation_index + 1));
                std::fs::create_dir(&generation_dir).map_err(|e| {
                    format!("create AWX evolve generation directory {}: {e}", generation_dir.display())
                })?;
                for (world_index, world) in worlds.iter().enumerate() {
                    let world_dir = generation_dir.join(format!("w{world_index}"));
                    crate::awx::evolution_export::export_native_world_bundle(
                        &world.data,
                        &world_dir,
                    )?;
                }
            }
            let best_dir = output_dir.join(format!("best_k{requested_k}"));
            let mut best_world = evolution.best.data.clone();
            best_world.routed = trial.clone();
            let manifest = crate::awx::evolution_export::export_native_world_bundle(
                &best_world,
                &best_dir,
            )?;
            let world_summary = |world: &route_evolution::EvolutionWorld<evolution_native::NativeWorld>| {
                serde_json::json!({
                    "id": world.id.as_str(),
                    "name": world.name.as_str(),
                    "origin": world.origin.as_str(),
                    "grade": {
                        "opens": &world.grade.opens,
                        "drc": world.grade.drc,
                        "vias": world.grade.vias,
                    },
                })
            };
            let ledger = serde_json::json!({
                "tag": tag,
                "K": requested_k,
                "pop": options.population,
                "generations": evolution.generations.iter().enumerate().map(|(index, worlds)| {
                    serde_json::json!({
                        "generation": index + 1,
                        "worlds": worlds.iter().map(&world_summary).collect::<Vec<_>>(),
                    })
                }).collect::<Vec<_>>(),
                "population": evolution.population.iter().map(&world_summary).collect::<Vec<_>>(),
                "best": world_summary(&evolution.best),
            });
            let ledger_path = output_dir.join(format!("evolve_k{requested_k}.json"));
            let ledger_bytes = serde_json::to_vec_pretty(&ledger).map_err(|e| e.to_string())?;
            atomic_export(&ledger_path, &ledger_bytes)?;
            Ok::<_, String>((manifest, ledger_path))
        })();
        match exported {
            Ok((manifest, ledger_path)) => (Some(vec![manifest]), Some(ledger_path)),
            Err(error) => {
                let _ = std::fs::remove_dir_all(output_dir);
                return Err(error);
            }
        }
    };
    if committed {
        if let Err(error) = atomic_export(&spec.output, &trial.ses()) {
            if seed_manifests.is_some() {
                if let Some(output_dir) = loaded.evolve_output_directory.as_deref() {
                    let _ = std::fs::remove_dir_all(output_dir);
                }
            }
            return Err(error);
        }
    }

    let mut rules=requested_spec.structure_reports();
    for rule in &mut rules {
        if rule.rule=="route_remaining" && !remaining_ran {
            rule.status=crate::spec::RuleStatus::Supported;
            rule.detail="route_remaining was not run by the native evolution transaction".into();
        } else {
            rule.status=if success {crate::spec::RuleStatus::Executed} else {crate::spec::RuleStatus::Unmet};
            rule.detail="native evolution best world was regraded and checked against configured routing rules".into();
        }
    }
    let summary=AwxEvolutionSummary {
        tag: tag.to_owned(),
        ledger_path,
        best_world_id:evolution.best.id.clone(),
        best_world_name:evolution.best.name.clone(),
        best_world_origin:evolution.best.origin.clone(),
        population:evolution.population.len(),
        generations:evolution.generations.len(),
        open_nets:grade.opens.clone(),
        drc_violations:grade.drc,
        vias:grade.vias,
    };
    let routed_groups=if success {ordered_groups(&requested_spec)?.iter().map(|group|group.id.clone()).collect()} else {Vec::new()};
    Ok(ExecutionReport {
        success,
        output_committed:committed,
        deadline_exhausted,
        rule_scope:"native_evolution_best_world".into(),
        length_metric:"native evolution grade".into(),
        channel_protection:"native seed board and configured rules".into(),
        rules,
        solver_workers:options.jobs,
        conflict_workers:1,
        detail_workers:1,
        rejected_net_attempts,
        stages:vec!["native_evolution_import".into(),"native_evolution_generations".into(),"native_best_world_grade".into(),"native_transaction_certification".into(),"verification_export".into()],
        candidate_count:0,
        conflict_count:0,
        selected_ids:Vec::new(),
        selected_candidates:Vec::new(),
        routing_changes:routing_changes(&base_board,&trial),
        routing_changes_base:"evolution_board".into(),
        routing_counts_before:routing_counts(&base_board),
        routing_counts_after:routing_counts(&trial),
        solve_seconds:start.elapsed().as_secs_f64(),
        validation_seconds:0.0,
        elapsed_seconds:start.elapsed().as_secs_f64(),
        feedback_iterations:evolution.generations.len(),
        solver_scope:"awx_route_evolution_native".into(),
        output_written:committed,
        routed_groups,
        remaining_incomplete,
        awx_buses:Vec::new(),
        awx_statistics:None,
        awx_bus_discovery:None,
        awx_evolution:Some(summary),
        awx_seed_manifests:seed_manifests,
    })
}

/// Prove that the same Specctra session we will export restores each authorized component
/// translation and its pins when replayed over the source DSN.
fn verify_placement_session_replay(
    source_dsn: &[u8],
    expected: &BoardAdapter,
    moved_components: &std::collections::BTreeSet<String>,
) -> Result<(), String> {
    // Recover only true, explicit physical dimensions from the expected native board.
    // These candidates authorize SES padstacks before replay; the serialized SES itself
    // never supplies trusted via dimensions or net authorization.
    let mut via_candidates = Vec::new();
    for key in expected.board.get_vias() {
        let item = expected.board.item(key);
        let net = item.net_numbers().iter().find_map(|&n| expected.board.rules.nets.get(n).map(|rule| rule.name.clone()))
            .ok_or("placement replay via has no net")?;
        let pad = item.padstack(&expected.board).ok_or("placement replay via has no padstack")?;
        // Source-DSN vias are already authorized by their native net-class rule;
        // only explicitly drilled native additions can be registered as candidates.
        let Some(drill) = pad.get_explicit_drill_radius() else { continue };
        let diameter = (pad.from_layer()..=pad.to_layer()).filter_map(|layer| pad.get_shape(layer))
            .map(|shape| shape.max_width()).fold(0.0_f64, f64::max);
        via_candidates.push((net, diameter / expected.units_per_mm, 2.0 * drill / expected.units_per_mm));
    }
    let replay = BoardAdapter::load_session_with_via_candidates(source_dsn, &expected.ses(), &via_candidates)?;
    let scale = expected.units_per_mm;
    for reference in moved_components {
        let a = expected.board.components.get_by_name(reference)
            .ok_or_else(|| format!("placement output lost component {reference}"))?;
        let b = replay.board.components.get_by_name(reference)
            .ok_or_else(|| format!("SES replay lost moved component {reference}"))?;
        let a = a.get_location().ok_or_else(|| format!("component {reference} has no output pose"))?.to_float();
        let b = b.get_location().ok_or_else(|| format!("component {reference} has no replayed pose"))?.to_float();
        if (a.x - b.x).abs() > 1.0 || (a.y - b.y).abs() > 1.0 {
            return Err(format!("SES replay placement differs for {reference}"));
        }
        let pads_a: std::collections::BTreeMap<_, _> = expected.pads().into_iter()
            .filter(|p| p.chip == *reference).map(|p| (p.reference, p.point)).collect();
        let pads_b: std::collections::BTreeMap<_, _> = replay.pads().into_iter()
            .filter(|p| p.chip == *reference).map(|p| (p.reference, p.point)).collect();
        if pads_a.len() != pads_b.len() || pads_a.iter().any(|(key, p)| {
            pads_b.get(key).is_none_or(|q| (p[0]-q[0]).hypot(p[1]-q[1]) > 1.0 / scale)
        }) {
            return Err(format!("SES replay pin placement differs for {reference}"));
        }
    }
    Ok(())
}

fn atomic_export(path: &std::path::Path, bytes: &[u8]) -> Result<(), String> {
    use std::io::Write;
    let parent = path.parent().ok_or("output has no parent directory")?;
    let pending = parent.join(format!(
        ".fr_agent_ses_{}_{}.pending",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_err(|e| e.to_string())?
            .as_nanos()
    ));
    let result = (|| {
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&pending)
            .map_err(|e| e.to_string())?;
        file.write_all(bytes).map_err(|e| e.to_string())?;
        file.sync_all().map_err(|e| e.to_string())?;
        std::fs::hard_link(&pending, path).map_err(|e| e.to_string())
    })();
    let _ = std::fs::remove_file(&pending);
    result
}

pub fn two_terminal_path_length(board: &BoardAdapter, name: &str) -> Result<f64, String> {
    let n = board.net_number(name)?;
    let pins: Vec<_> = board
        .board
        .get_pins()
        .into_iter()
        .filter(|&k| board.board.item(k).contains_net(n))
        .collect();
    if pins.len() != 2 {
        return Err("max_length_mm unsupported for multi-terminal net".into());
    }
    let mut nodes: Vec<(i32, [f64; 2])> = Vec::new();
    let mut segments = Vec::new();
    let mut bridges = Vec::new();
    fn node(nodes: &mut Vec<(i32, [f64; 2])>, layer: i32, p: [f64; 2]) -> usize {
        if let Some(i) = nodes.iter().position(|(l, q)| *l == layer && q == &p) {
            i
        } else {
            nodes.push((layer, p));
            nodes.len() - 1
        }
    }
    for k in board.board.get_connectable_items(n) {
        let item = board.board.item(k);
        if item.is_trace() {
            let trace = item.trace();
            let points = trace.polyline().corner_approx_arr();
            for s in points.windows(2) {
                let a = [s[0].x, s[0].y];
                let b = [s[1].x, s[1].y];
                node(&mut nodes, trace.layer(), a);
                node(&mut nodes, trace.layer(), b);
                segments.push((trace.layer(), a, b));
            }
        } else if item.is_pin() || item.is_via() {
            let p = item.center(&board.board).to_float();
            let p = [p.x, p.y];
            let mut previous = None;
            for layer in item.first_layer(&board.board)..=item.last_layer(&board.board) {
                let i = node(&mut nodes, layer, p);
                if let Some(j) = previous {
                    bridges.push((j, i, 0.0));
                }
                previous = Some(i);
            }
        } else {
            return Err("max_length_mm unsupported for conduction-area path".into());
        }
    }
    // Fail closed for proper same-layer interior/interior crossings. The initial
    // unique-chain model intentionally does not invent an unrecorded junction.
    for i in 0..segments.len() {
        for j in i + 1..segments.len() {
            let (l, a, b) = segments[i];
            let (m, c, d) = segments[j];
            if l != m {
                continue;
            }
            let r = [b[0] - a[0], b[1] - a[1]];
            let t = [d[0] - c[0], d[1] - c[1]];
            let cross = |x: [f64; 2], y: [f64; 2]| x[0] * y[1] - x[1] * y[0];
            let den = cross(r, t);
            if den.abs() < 1e-12 {
                continue;
            }
            let q = [c[0] - a[0], c[1] - a[1]];
            let u = cross(q, t) / den;
            let v = cross(q, r) / den;
            if u > 1e-12 && u < 1.0 - 1e-12 && v > 1e-12 && v < 1.0 - 1e-12 {
                return Err("max_length_mm unsupported for interior trace crossings".into());
            }
        }
    }
    let mut edges: Vec<BTreeMap<usize, f64>> = vec![BTreeMap::new(); nodes.len()];
    for (layer, a, b) in segments {
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let den = dx * dx + dy * dy;
        if den == 0.0 {
            continue;
        }
        let mut splits: Vec<_> = nodes
            .iter()
            .enumerate()
            .filter_map(|(i, (l, p))| {
                if *l != layer {
                    return None;
                }
                let t = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / den;
                let cross = ((p[0] - a[0]) * dy - (p[1] - a[1]) * dx).abs();
                if (0.0..=1.0).contains(&t) && cross <= 1e-6 {
                    Some((t, i))
                } else {
                    None
                }
            })
            .collect();
        splits.sort_by(|a, b| a.0.total_cmp(&b.0));
        for s in splits.windows(2) {
            let i = s[0].1;
            let j = s[1].1;
            let w = (s[1].0 - s[0].0) * den.sqrt() / board.units_per_mm;
            edges[i].insert(j, w);
            edges[j].insert(i, w);
        }
    }
    for (i, j, w) in bridges {
        edges[i].insert(j, w);
        edges[j].insert(i, w);
    }
    let locate = |k| {
        let item = board.board.item(k);
        let p = item.center(&board.board).to_float();
        nodes
            .iter()
            .position(|(l, q)| *l == item.first_layer(&board.board) && *q == [p.x, p.y])
            .ok_or("path endpoint absent")
    };
    let start = locate(pins[0])?;
    let end = locate(pins[1])?;
    let mut distance = vec![f64::INFINITY; nodes.len()];
    let mut visited = BTreeSet::new();
    distance[start] = 0.0;
    while let Some((i, _)) = distance
        .iter()
        .enumerate()
        .filter(|(i, d)| !visited.contains(i) && d.is_finite())
        .min_by(|a, b| a.1.total_cmp(b.1))
    {
        if edges[i].len() > 2 {
            return Err("max_length_mm unsupported for branched/looped net".into());
        }
        visited.insert(i);
        for (&j, &w) in &edges[i] {
            distance[j] = distance[j].min(distance[i] + w);
        }
    }
    let edge_count: usize = visited.iter().map(|&i| edges[i].len()).sum();
    if edge_count / 2 >= visited.len() {
        return Err("max_length_mm unsupported for looped net".into());
    }
    if !distance[end].is_finite() {
        return Err("max_length_mm endpoint path disconnected".into());
    }
    Ok(distance[end])
}

#[cfg(test)]
mod execution_seed_regression_tests {
    use super::*;
    use crate::spec::{Channel, ChannelRegion, Execution, RouteGroup};

    #[test]
    fn swept_l_channel_obstacle_leaves_its_bounding_box_corner_free() {
        let board = BoardAdapter::load(include_bytes!("../tests/fixtures/two_chip.dsn"))
            .expect("fixture loads");
        let areas = remaining_channel_areas(
            &board,
            &ChannelRegion::Polyline {
                points: vec![[2.0, 2.0], [8.0, 2.0], [8.0, 8.0]],
                width_mm: 2.0,
            },
        )
        .expect("native swept polyline area");
        let inside_arm = board.point([5.0, 2.0]).unwrap().to_float();
        let rounded_cap = board.point([8.5, 2.0]).unwrap().to_float();
        let free_corner = board.point([3.0, 7.0]).unwrap().to_float();
        assert!(areas.iter().any(|area| area.contains_float(&inside_arm)));
        assert!(areas.iter().any(|area| area.contains_float(&rounded_cap)));
        assert!(areas.iter().all(|area| !area.contains_float(&free_corner)));
    }

    #[test]
    fn deferred_unrelated_net_avoids_another_groups_reserved_channel() {
        let mut board = BoardAdapter::load(include_bytes!("../tests/fixtures/two_chip.dsn"))
            .expect("fixture loads");
        let rule = |name: &str, width_mm: f64| NetRule {
            name: name.into(),
            width_mm,
            clearance_mm: 0.1,
            allowed_layers: vec!["Top".into()],
            via_budget: Some(0),
            max_length_mm: None,
            protected: false,
        };
        let seed_spec = RouteSpec {
            version: 1,
            route_remaining: true,
            allow_partial_output: false,
            protected_nets: vec![],
            input: "board.dsn".into(),
            output: "routed.ses".into(),
            nets: vec![rule("BUS", 0.15)],
            endpoint_groups: vec![],
            channels: vec![Channel {
                id: "bus_reserved".into(),
                region: ChannelRegion::Rectangle { min: [9.0, 2.0], max: [11.0, 7.0] },
            }],
            groups: vec![
                RouteGroup {
                    id: "bus".into(),
                    nets: vec!["BUS".into()],
                    endpoint_groups: vec![],
                    channels: vec![],
                    priority: 0,
                    depends_on: vec![],
                    joint_escape: false,
                    awx: None,
                },
                RouteGroup {
                    id: "channel_owner".into(),
                    nets: vec!["BUS".into()],
                    endpoint_groups: vec![],
                    channels: vec!["bus_reserved".into()],
                    priority: 0,
                    depends_on: vec![],
                    joint_escape: false,
                    awx: None,
                },
            ],
            execution: Execution::default(),
            advanced: None,
            awx: None,
        };
        let mut controller_spec = seed_spec.clone();
        controller_spec.nets.push(rule("OTHER", 0.3));
        controller_spec.groups[0].nets.push("OTHER".into());
        let spec = merge_requested_scope(&seed_spec, &controller_spec)
            .expect("controller K-prefix extends the captured seed group");
        assert!(spec.groups[0].nets.contains(&"OTHER".to_owned()));
        let (rejected, exhausted) = route_remaining_parallel(
            &mut board,
            &spec,
            &BTreeSet::from(["BUS".to_owned()]),
            Instant::now() + Duration::from_secs(10),
        )
        .expect("deferred autorouter runs");
        assert!(!exhausted, "deferred route exhausted its budget: {rejected:?}");
        let other = board.net_number("OTHER").unwrap();
        let connectable = board.board.get_connectable_items(other);
        assert_eq!(fr_engine::drc::NetIncompletes::new(&board.board, other, &connectable).count(), 0);
        for key in board.board.get_traces().into_iter().filter(|&key| board.board.item(key).contains_net(other)) {
            let trace = board.board.item(key).trace();
            assert!(2.0 * trace.half_width() as f64 / board.units_per_mm >= 0.3 - 1e-9,
                "deferred OTHER trace ignored its configured width");
            let points = trace.polyline().corner_approx_arr();
            for segment in points.windows(2) {
                for step in 0..=20 {
                    let t = step as f64 / 20.0;
                    let x = (segment[0].x + (segment[1].x - segment[0].x) * t) / board.units_per_mm;
                    let y = (segment[0].y + (segment[1].y - segment[0].y) * t) / board.units_per_mm;
                    assert!(!(9.0..=11.0).contains(&x) || !(2.0..=7.0).contains(&y),
                        "unrelated OTHER trace entered BUS reserved channel at ({x}, {y})");
                }
            }
        }
    }
}
