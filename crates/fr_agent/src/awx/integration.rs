//! Actual AWX route_bus driver boundary. Related groups share one joint/whole problem.
use super::{
    board_snapshot::{BoardSnapshot, CanonicalGeometry},
    escape_moves::{self, Pad},
    fab_rules,
    joint_escape::{self, JointInput, JointPair, JointPlan, OptionMove},
    joint_geometry::{JointSizes, NetLeg},
    native_execution, pairs,
    whole_context::RoutingRules,
};
use crate::{
    board_adapter::BoardAdapter,
    spec::{RouteGroup, RouteSpec},
};
use serde::Serialize;
use std::{
    collections::{BTreeMap, BTreeSet},
    time::Instant,
};
#[derive(Clone, Debug, Serialize)]
pub struct AwxBusReport {
    pub groups: Vec<String>,
    pub source: String,
    pub destination: String,
    pub requested: Vec<String>,
    pub routed: Vec<String>,
    pub refused: BTreeMap<String, String>,
    pub joint_source: Option<joint_escape::JointReport>,
    pub joint_destination: Option<joint_escape::JointReport>,
    pub rounds: usize,
    pub grade: native_execution::NativeGrade,
    pub proved: bool,
    pub reason: String,
    pub placement: Option<super::passive_clearance::PlacementPass>,
    pub via_nudged: Vec<String>,
    pub source_decisions: Vec<super::source_realize::SourceNetDecision>,
    pub source_audits: Vec<super::source_realize::SourceMoveAudit>,
    pub source_refusals: Vec<super::source_realize::SourceMoveRefusal>,
    pub destination_decisions: Vec<super::destination_realize::DestinationNetDecision>,
    pub destination_audits: Vec<super::destination_realize::DestinationMoveAudit>,
    pub destination_refusals: Vec<super::destination_realize::DestinationMoveRefusal>,
    pub smoothing: Option<super::board_smoothing::BoardSmoothingResult>,
    pub eligible_nets: Vec<String>,
    pub selected_ladder_nets: Vec<String>,
    pub omitted_ladder_nets: Vec<String>,
    pub ladder_order: Vec<String>,
    pub deferred_ladder_nets: Vec<String>,
}
pub struct AwxExecution {
    pub board: BoardAdapter,
    pub buses: Vec<AwxBusReport>,
    pub protected: BTreeSet<String>,
    pub all_requested_routed: bool,
    pub moved_components: BTreeSet<String>,
    pub runtime_events: super::runtime_stats::AwxRuntimeEvents,
    /// `None` means discovery was disabled; `Some(vec![])` is an observed empty result.
    pub discovered_buses: Option<Vec<super::bus_discovery::DiscoveredBus>>,
    pub deferred_nets: BTreeSet<String>,
    pub seed_captures: Vec<AwxSeedCapture>,
}

/// Actual per-scope native evolution seed state captured at a successful route
/// attempt. The exporter consumes these artifacts directly; it never reconstructs
/// pack/plan metadata from `AwxBusReport` summaries.
#[derive(Clone)]
pub struct AwxSeedCapture {
    pub board_dsn: std::path::PathBuf,
    pub spec: RouteSpec,
    pub requested_k: i64,
    pub destination: String,
    pub names: Vec<String>,
    pub routed: BoardAdapter,
    pub fanout: BoardAdapter,
    pub source_plan: JointPlan,
    pub destination_plan: JointPlan,
    pub whole: super::whole_route::WholeLoopOutcome,
    pub context: super::whole_context::WholeContext,
    pub flow: super::whole_frame::FlowFrame,
    pub grade: native_execution::NativeGrade,
    pub plan_seed: serde_json::Value,
}
pub fn related_groups<'a>(spec: &'a RouteSpec) -> Result<Vec<Vec<&'a RouteGroup>>, String> {
    let mut out: Vec<Vec<&RouteGroup>> = Vec::new();
    let mut order = Vec::<&RouteGroup>::new();
    let mut done = BTreeSet::new();
    while order.len() < spec.groups.len() {
        let mut ready: Vec<_> = spec
            .groups
            .iter()
            .filter(|g| !done.contains(&g.id) && g.depends_on.iter().all(|n| done.contains(n)))
            .collect();
        ready.sort_by(|a, b| b.priority.cmp(&a.priority).then(a.id.cmp(&b.id)));
        let Some(g) = ready.first().copied() else {
            return Err("AWX group dependencies do not converge".into());
        };
        done.insert(g.id.clone());
        order.push(g);
    }
    for g in order {
        let a = g
            .awx
            .as_ref()
            .ok_or("Unsupported: group requires AWX source/destination/layer settings")?;
        if let Some(group) = out.iter_mut().find(|gs| {
            gs[0]
                .awx
                .as_ref()
                .is_some_and(|p| p.source == a.source && p.destination == a.destination)
        }) {
            if group[0].awx.as_ref().unwrap().routing_layers != a.routing_layers {
                return Err("related AWX groups have conflicting routing stacks".into());
            }
            group.push(g);
        } else {
            out.push(vec![g]);
        }
    }
    Ok(out)
}
pub fn joint_input(
    snapshot: &BoardSnapshot,
    spec: &RouteSpec,
    groups: &[&RouteGroup],
    part: &str,
    destination: bool,
    deadline: Instant,
) -> Result<JointInput, String> {
    let awx = spec
        .awx
        .as_ref()
        .ok_or("Unsupported: AWX needs physical logical components and fabrication rules")?;
    let first = groups
        .first()
        .ok_or("empty AWX group problem")?
        .awx
        .as_ref()
        .unwrap();
    let requested: BTreeSet<_> = groups.iter().flat_map(|g| g.nets.iter().cloned()).collect();
    let pad_meta: Vec<_> = snapshot
        .pads
        .iter()
        .filter(|p| p.component_ref == part)
        .collect();
    if pad_meta.is_empty() {
        return Err(format!("AWX logical part {part} is absent"));
    }
    if pad_meta.iter().any(|p| {
        !matches!(
            p.component_kind,
            super::whole_context::ComponentKind::Bga
                | super::whole_context::ComponentKind::Connector
        )
    }) {
        return Err(
            "Unsupported: AWX escape requires explicitly identified BGA/connector array geometry"
                .into(),
        );
    }
    let pads: Vec<_> = pad_meta
        .iter()
        .map(|p| Pad {
            key: p.key.clone(),
            net: p.net.clone(),
            net_id: p.net_id,
            component_ref: p.component_ref.clone(),
            pad_number: p.pad_number.clone(),
            point: p.point,
            layers: p.layers.clone(),
            size: p.size,
        })
        .collect();
    let plane_ids: BTreeSet<_> = snapshot.zones.iter().map(|z| z.net_id).collect();
    let drops: BTreeSet<_> = pad_meta
        .iter()
        .filter(|p| {
            plane_ids.contains(&p.net_id) && !p.net.is_empty() && !requested.contains(&p.net)
        })
        .map(|p| p.net.clone())
        .collect();
    let others: BTreeSet<_> = pad_meta
        .iter()
        .filter(|p| p.net_id > 0 && !requested.contains(&p.net) && !drops.contains(&p.net))
        .map(|p| p.net.clone())
        .collect();
    let partner = if destination {
        &first.source
    } else {
        &first.destination
    };
    let other_pads: Vec<_> = snapshot
        .pads
        .iter()
        .filter(|p| &p.component_ref == partner)
        .map(|p| Pad {
            key: p.key.clone(),
            net: p.net.clone(),
            net_id: p.net_id,
            component_ref: p.component_ref.clone(),
            pad_number: p.pad_number.clone(),
            point: p.point,
            layers: p.layers.clone(),
            size: p.size,
        })
        .collect();
    let board_floors = BTreeMap::from([
        ("track_width".into(), awx.rules.track_width),
        ("clearance".into(), awx.rules.clearance),
        ("via_diameter".into(), awx.rules.via_size),
        ("via_drill".into(), awx.rules.via_drill),
        ("hole_to_hole".into(), awx.rules.hole_to_hole_clearance),
        ("board_edge".into(), awx.rules.board_edge_clearance),
    ]);
    let floors = fab_rules::fab_floor_ladder(
        snapshot.layers.len(),
        awx.fabrication,
        awx.escalation,
        &awx.fabrication_overrides,
        &board_floors,
    );
    let sizes = JointSizes::measure(
        &pads,
        awx.rules.track_width,
        awx.rules.fan_clearance,
        awx.rules.hole_to_hole_clearance,
        awx.rules.via_size,
        awx.rules.via_drill,
        &floors,
        awx.rules.track_width,
        awx.rules.clearance,
        awx.rules.grid_step,
        pairs::pitch(&awx.rules),
    );
    let pairs = groups
        .iter()
        .flat_map(|g| g.awx.as_ref().unwrap().pairs.iter())
        .map(|p| JointPair {
            base: p.base.clone(),
            p: p.p.clone(),
            n: p.n.clone(),
        })
        .collect();
    let segments = snapshot
        .segments
        .iter()
        .map(|s| NetLeg {
            net_id: s.net_id,
            net: s.net.clone(),
            leg: escape_moves::Leg {
                a: s.a,
                b: s.b,
                layer: s.layer.clone(),
            },
        })
        .collect();
    let bus_groups = groups
        .iter()
        .flat_map(|g| g.nets.iter().map(move |n| (n.clone(), g.id.clone())))
        .collect();
    let plane_layers = snapshot.zones.iter().map(|z| z.layer.clone()).collect();
    let time_limit = deadline
        .checked_duration_since(Instant::now())
        .map(|v| v.as_secs_f64())
        .ok_or("AWX shared deadline exhausted")?;
    let far = super::joint_geometry::far_face(&pads, &other_pads);
    Ok(JointInput {
        reference: part.into(),
        pads,
        bus: requested,
        others,
        drops,
        other_layers: first.routing_layers.clone(),
        routing_layers: first.routing_layers.clone(),
        plane_layers,
        far,
        preferred: BTreeMap::new(),
        pairs,
        hands: BTreeMap::new(),
        bus_groups,
        sizes,
        segments,
        zones: snapshot.zones.clone(),
        pockets: Vec::new(),
        only: None,
        vias_only: false,
        climb: None,
        climb_other: 2,
        street: 3,
        solve_batches: 100,
        solve_workers: i32::try_from(spec.execution.threads).map_err(|_| "AWX workers overflow")?,
        phase1_deterministic_time: 30.,
        time_limit: Some(time_limit),
    })
}
pub fn lay_joint(
    board: &BoardAdapter,
    plan: &JointPlan,
    rules: &RoutingRules,
    drills: &mut BTreeMap<String, f64>,
    protected: &BTreeSet<String>,
) -> Result<BoardAdapter, String> {
    let mut trial = board.clone();
    let names: BTreeMap<String, Vec<String>> = (1..=board.board.rules.nets.max_net_number())
        .filter_map(|n| board.board.rules.nets.get(n))
        .fold(BTreeMap::new(), |mut m, n| {
            m.entry(escape_moves::short_name(&n.name).into())
                .or_default()
                .push(n.name.clone());
            m
        });
    let full = |name: &str| -> Result<String, String> {
        if board.net_number(name).is_ok() {
            return Ok(name.into());
        }
        let hits = names
            .get(name)
            .ok_or_else(|| format!("unknown AWX net {name}"))?;
        if hits.len() != 1 {
            return Err(format!("ambiguous AWX basename {name}"));
        }
        Ok(hits[0].clone())
    };
    for piece in plan.pieces.values() {
        let net = full(&piece.net)?;
        if protected.contains(&net) {
            return Err("AWX joint escape requested protected copper".into());
        }
        for v in &piece.vias {
            native_execution::via(
                &mut trial,
                &net,
                v.point,
                2. * v.radius,
                2. * v.drill_radius,
            )?;
            let radius = (v.radius * trial.units_per_mm).round() as i32;
            let drill = (2. * v.drill_radius * trial.units_per_mm).round() as i32;
            drills.insert(format!("awx_via_r{radius}_d{drill}"), 2. * v.drill_radius);
        }
        for leg in &piece.legs {
            native_execution::trace(
                &mut trial,
                &net,
                &leg.layer,
                leg.a,
                leg.b,
                rules.track_width,
            )?;
        }
    }
    trial.assert_invariants(board, protected, false)?;
    Ok(trial)
}

/// A complete whole-route round on the physically realized joint fanout. The outer
/// driver supplies its selected ends and can repeat this with feedback-priced ends.
pub struct BusAttempt {
    pub board: BoardAdapter,
    pub context: super::whole_context::WholeContext,
    pub flow: super::whole_frame::FlowFrame,
    pub whole: super::whole_route::WholeLoopOutcome,
    pub grade: native_execution::NativeGrade,
    pub relayed: Vec<String>,
    pub whole_rounds: usize,
    pub relay_count: usize,
    pub whole_route_elapsed_us: u64,
    pub relayer_elapsed_us: u64,
    pub physical_layers: Vec<String>,
}
pub fn route_attempt(
    original: &BoardAdapter,
    fanout: &BoardAdapter,
    spec: &RouteSpec,
    groups: &[&RouteGroup],
    source_plan: &JointPlan,
    destination_plan: &JointPlan,
    source_menus: &super::joint_escape::JointMenus,
    destination_menus: &super::joint_escape::JointMenus,
    drills: &BTreeMap<String, f64>,
    whole_round_limit: usize,
    deadline: Instant,
) -> Result<BusAttempt, String> {
    let cfg = spec
        .awx
        .as_ref()
        .ok_or("missing AWX physical configuration")?;
    let first = groups
        .first()
        .and_then(|g| g.awx.as_ref())
        .ok_or("missing AWX group")?;
    let nets: BTreeSet<_> = groups.iter().flat_map(|g| g.nets.iter().cloned()).collect();
    let prs: Vec<_> = groups
        .iter()
        .flat_map(|g| g.awx.as_ref().unwrap().pairs.iter().cloned())
        .collect();
    let mut options = super::whole_types::WholeOptions::default();
    options.routing_layers = first.routing_layers.clone();
    options.workers =
        i32::try_from(spec.execution.threads).map_err(|_| "AWX worker count overflow")?;
    let mut board = fanout.clone();
    let mut snapshot = BoardSnapshot::capture(&board, &cfg.components, drills)?;
    let mut source = source_plan.clone();
    let mut destination = destination_plan.clone();
    let mut relayed = BTreeSet::new();
    let mut seen = BTreeSet::new();
    let mut built = super::context_builder::build(
        &snapshot,
        &first.source,
        &first.destination,
        &nets,
        &prs,
        &source,
        &destination,
        source_menus,
        destination_menus,
        &cfg.rules,
        &first.routing_layers,
    )?;
    let max_relays = first.routing_layers.len().saturating_mul(2).max(1);
    let mut whole = None;
    let mut whole_rounds = 0usize;
    let mut relay_count = 0usize;
    let mut whole_route_elapsed_us = 0u64;
    let mut relayer_elapsed_us = 0u64;
    for _ in 0..=max_relays {
        let problem = super::whole_types::WholeProblem {
            rules: super::whole_types::WholeRules::from_context(&built.context)?,
            context: built.context.clone(),
            frame: built.frame,
            cuts: vec![],
            soft_cuts: vec![],
            history: vec![],
            hint: None,
            options: options.clone(),
            blocked_fronts: built.blocked_fronts.clone(),
        };
        let oracle = CanonicalGeometry {
            snapshot: &snapshot,
            frame: built.flow,
        };
        let remaining = deadline
            .checked_duration_since(Instant::now())
            .ok_or("AWX deadline exhausted before whole routing")?
            .as_millis()
            .min(u64::MAX as u128) as u64;
        let outcome = super::whole_route::run(&problem, &oracle, whole_round_limit, remaining)?;
        whole_rounds = whole_rounds.saturating_add(outcome.rounds);
        whole_route_elapsed_us = whole_route_elapsed_us.saturating_add(outcome.elapsed_us);
        let Some(plan) = outcome.plan.as_ref() else {
            whole = Some(outcome);
            break;
        };
        let signature = serde_json::to_string(&plan.layers).map_err(|e| e.to_string())?;
        let repeated = !seen.insert(signature);
        let (fresh, report) = super::relayer::relay_native_board(
            &mut board,
            &built.context,
            plan,
            built.flow,
            &snapshot.layers,
            &cfg.components,
            drills,
        )?;
        relay_count = relay_count.saturating_add(report.relayed_ends);
        relayer_elapsed_us = relayer_elapsed_us.saturating_add(report.elapsed_us.unwrap_or(0));
        whole = Some(outcome);
        if report.moved.is_empty() && report.dropped_vias == 0 {
            break;
        }
        if repeated {
            return Err("AWX relayer/whole-route endpoint layers did not stabilize".into());
        }
        relayed.extend(report.moved);
        snapshot = fresh;
        let whole_plan = whole
            .as_ref()
            .and_then(|w| w.plan.as_ref())
            .ok_or("AWX relayer lost whole plan")?;
        sync_plan_endpoint_layers(&mut source, &whole_plan.layers, &prs, 0);
        sync_plan_endpoint_layers(&mut destination, &whole_plan.layers, &prs, 1);
        built = super::context_builder::build(
            &snapshot,
            &first.source,
            &first.destination,
            &nets,
            &prs,
            &source,
            &destination,
            source_menus,
            destination_menus,
            &cfg.rules,
            &first.routing_layers,
        )?;
    }
    let whole = whole.ok_or("AWX whole-route loop produced no outcome")?;
    if let Some(geometry) = &whole.geometry {
        // The native backend consumes actual physical legs/barrels only; no virtual
        // lane centre can be substituted for a pair's copper.
        native_execution::require_native_geometry(geometry)?;
        let installed = built.context.install(geometry.lanes.clone())?;
        board = native_execution::materialize(
            &board,
            &installed,
            built.flow,
            &cfg.rules,
            &spec.protected_names(),
        )?;
    }
    let grade = native_execution::grade(original, &board, &nets)?;
    Ok(BusAttempt {
        board,
        context: built.context,
        flow: built.flow,
        whole,
        grade,
        relayed: relayed.into_iter().collect(),
        whole_rounds,
        relay_count,
        whole_route_elapsed_us,
        relayer_elapsed_us,
        physical_layers: snapshot.layers.clone(),
    })
}

fn sync_plan_endpoint_layers(
    plan: &mut JointPlan,
    lane_layers: &BTreeMap<String, Vec<String>>,
    pairs: &[super::pairs::Pair],
    end: usize,
) {
    for (lane, layers) in lane_layers {
        if layers.is_empty() {
            continue;
        }
        let layer = if end == 0 {
            layers.first()
        } else {
            layers.last()
        };
        let Some(layer) = layer else {
            continue;
        };
        let members: Vec<&str> = pairs
            .iter()
            .find(|p| &p.base == lane)
            .map(|p| vec![p.p.as_str(), p.n.as_str()])
            .unwrap_or_else(|| vec![lane.as_str()]);
        for (name, option) in &mut plan.chosen {
            if members.iter().any(|member| {
                *member == name
                    || super::escape_moves::short_name(member)
                        == super::escape_moves::short_name(name)
            }) {
                if let OptionMove::Escape { movement } = option {
                    movement.layer = layer.clone();
                }
            }
        }
    }
}
/// Certify the requested numerical and channel rules against the realized board,
/// after routing and before the single transaction is committed.
pub fn verify_requested(
    board: &BoardAdapter,
    spec: &RouteSpec,
    groups: &[&RouteGroup],
) -> Result<(), String> {
    use crate::planner::{PlanCandidate, RouteLeg};
    let pads = board.pads();
    for group in groups {
        for endpoint_group in &group.endpoint_groups {
            let endpoints = spec
                .endpoint_groups
                .iter()
                .find(|e| &e.id == endpoint_group)
                .ok_or("unknown endpoint group")?;
            for endpoint in &endpoints.endpoints {
                let native = pads
                    .iter()
                    .find(|p| p.chip == endpoint.chip && p.pad == endpoint.pad)
                    .ok_or_else(|| {
                        format!("unknown board endpoint {}.{}", endpoint.chip, endpoint.pad)
                    })?;
                if native.net != endpoint.net {
                    return Err(format!(
                        "endpoint {}.{} belongs to {}, requested {}",
                        endpoint.chip, endpoint.pad, native.net, endpoint.net
                    ));
                }
            }
        }
        for name in &group.nets {
            let rule = spec
                .nets
                .iter()
                .find(|r| &r.name == name)
                .ok_or("missing active net rule")?;
            let n = board.net_number(name)?;
            for key in board
                .board
                .get_traces()
                .into_iter()
                .filter(|&k| board.board.item(k).contains_net(n))
            {
                let item = board.board.item(key);
                let trace = item.trace();
                let layer = board.board.layer_structure.layers[trace.layer() as usize]
                    .name
                    .clone();
                let width = 2. * trace.half_width() as f64 / board.units_per_mm;
                let points: Vec<_> = trace
                    .polyline()
                    .corner_approx_arr()
                    .iter()
                    .map(|p| [p.x / board.units_per_mm, p.y / board.units_per_mm])
                    .collect();
                let check = PlanCandidate {
                    id: format!("awx_native_rule:{}", item.id().0),
                    selection_set_id: "certification".into(),
                    group_id: group.id.clone(),
                    cost: 0,
                    legs: vec![RouteLeg {
                        net: name.clone(),
                        layer,
                        points: points.clone(),
                        width_mm: width,
                        clearance_mm: rule.clearance_mm,
                    }],
                    vias: vec![],
                    reservations: vec![],
                    requires_any: vec![],
                };
                // This invokes the native clearance checker on existing physical
                // copper. It does not enumerate or select foundation candidates.
                board.check_candidate(&check, spec)?;
                if !group.channels.is_empty() {
                    for ab in points.windows(2) {
                        if !group
                            .channels
                            .iter()
                            .filter_map(|id| spec.channels.iter().find(|c| &c.id == id))
                            .any(|c| {
                                crate::board_adapter::segment_in_channel(
                                    ab[0],
                                    ab[1],
                                    width / 2. + rule.clearance_mm,
                                    &c.region,
                                )
                            })
                        {
                            return Err(format!(
                                "net {name}: realized trace exceeds hard channel envelope"
                            ));
                        }
                    }
                }
            }
            let vias: Vec<_> = board
                .board
                .get_vias()
                .into_iter()
                .filter(|&k| board.board.item(k).contains_net(n))
                .collect();
            if vias.len() > rule.via_budget.unwrap_or(u32::MAX) as usize {
                return Err(format!("net {name}: via budget exceeded"));
            }
            for key in vias {
                let item = board.board.item(key);
                let p = item.center(&board.board).to_float();
                let point = [p.x / board.units_per_mm, p.y / board.units_per_mm];
                let pad = item
                    .padstack(&board.board)
                    .ok_or("native via without padstack")?;
                let mut radius = 0f64;
                for layer in pad.from_layer()..=pad.to_layer() {
                    if let Some(shape) = item.drill_shape_on_layer(&board.board, layer) {
                        let b = shape.bounding_box();
                        radius = radius.max(
                            (b.ur.x - b.ll.x).max(b.ur.y - b.ll.y) as f64
                                / (2. * board.units_per_mm),
                        );
                    }
                }
                if !group.channels.is_empty()
                    && !group
                        .channels
                        .iter()
                        .filter_map(|id| spec.channels.iter().find(|c| &c.id == id))
                        .any(|c| {
                            crate::board_adapter::segment_in_channel(
                                point,
                                point,
                                radius + rule.clearance_mm,
                                &c.region,
                            )
                        })
                {
                    return Err(format!(
                        "net {name}: realized via exceeds hard channel envelope"
                    ));
                }
            }
            if let Some(limit) = rule.max_length_mm {
                if crate::execution::two_terminal_path_length(board, name)? > limit + 1e-9 {
                    return Err(format!("net {name}: maximum endpoint path length exceeded"));
                }
            }
        }
    }
    Ok(())
}
