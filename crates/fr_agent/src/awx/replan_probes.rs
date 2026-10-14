//! Native replan candidate menus and isolated physical probes.
//!
//! Candidates are made from the exact joint-escape menus. A probe strips the
//! affected copper from the active routed board, restores each affected net
//! from its last fanout lineage, realizes the requested source/destination
//! moves, runs the native scoped AWX route attempt, and grades its landed board.

use super::{
    board_snapshot::BoardSnapshot,
    braid_band::SourceRungBandFactory,
    braid_connect::{
        self, LadderMode, RungBandFactory, RungBaseRequest, RungEndpoint, RungExecutor,
    },
    braid_corridor::{self, CorridorBandContext, CorridorPlan, CorridorState},
    destination_realize,
    escape_moves::{self, Move},
    joint_escape::{JointMenus, JointPlan, OptionMove},
    native_execution,
    native_rung::{NativeRungExecutor, NativeRungRoute},
    replan_cost::{ReplanCandidate, ReplanCostInput, ReplanCostModel},
    replan_state::ReplanState,
    replan_types::EndState,
    route_evolution::RouteGrade,
    source_realize,
};
use crate::{board_adapter::BoardAdapter, spec::RouteGroup};
use serde::Serialize;
use std::{
    collections::{BTreeMap, BTreeSet},
    time::{Duration, Instant},
};

#[derive(Clone)]
struct RungFabricationSettings {
    fabrication: super::fab_rules::FabTier,
    escalation: super::fab_rules::EscalationPolicy,
    overrides: BTreeMap<String, f64>,
    clearance_floor_mm: f64,
    track_floor_mm: f64,
    may_narrow: bool,
    via_rungs: Vec<(f64, f64)>,
}

fn rung_fabrication_settings(
    state: &ReplanState,
    board: &BoardAdapter,
    net_id: fr_engine::ids::NetNo,
    route_layers: &[String],
) -> Result<RungFabricationSettings, String> {
    let awx = state
        .world
        .spec
        .awx
        .as_ref()
        .ok_or("native replan requires AWX fabrication settings")?;
    let rules = &awx.rules;
    let (run_via_size, run_via_drill) = rules
        .net_via_sizes
        .get(&(net_id as i32))
        .map(|sizes| (sizes[0], sizes[1]))
        .unwrap_or((rules.via_size, rules.via_drill));
    let board_floors = BTreeMap::from([
        ("track_width".to_owned(), rules.track_width),
        ("clearance".to_owned(), rules.clearance),
        ("via_diameter".to_owned(), run_via_size),
        ("via_drill".to_owned(), run_via_drill),
        ("hole_to_hole".to_owned(), rules.hole_to_hole_clearance),
        ("board_edge".to_owned(), rules.board_edge_clearance),
    ]);
    let class = board
        .board
        .rules
        .net_class_of(net_id)
        .get_trace_clearance_class();
    let class_name = board
        .board
        .rules
        .clearance_matrix
        .get_name(class)
        .unwrap_or_default();
    let mut rule_floors = BTreeMap::new();
    if !class_name.eq_ignore_ascii_case("default") {
        let mut floor = 0i32;
        for layer in route_layers {
            let layer_no = board.board.layer_structure.get_no(layer);
            if layer_no >= 0 {
                floor = floor.max(board.board.clearance_value(class, class, layer_no));
            }
        }
        if floor > 0 {
            rule_floors.insert("clearance".to_owned(), floor as f64 / board.units_per_mm);
        }
    }
    let stack_layers = board.board.layer_structure.layers.iter().count();
    let copper_layers = if stack_layers == 0 { 2 } else { stack_layers };
    let via_rungs = super::fab_rules::escalation_rungs(
        copper_layers,
        awx.fabrication,
        awx.escalation,
        &awx.fabrication_overrides,
        &board_floors,
        &rule_floors,
    )
    .into_iter()
    .map(|floor| (floor.via_diameter, floor.via_drill))
    .collect();
    let clearance_floor_mm = rule_floors.get("clearance").copied().unwrap_or(0.0).max(
        if awx.escalation == super::fab_rules::EscalationPolicy::Board {
            rules.clearance
        } else {
            0.0
        },
    );
    let track_floor_mm = rule_floors.get("track_width").copied().unwrap_or(0.0).max(
        if awx.escalation == super::fab_rules::EscalationPolicy::Board {
            rules.track_width
        } else {
            0.0
        },
    );
    // The fixed AWX `connect.make_config` call does not expose a neck-down
    // override; its source GridRouteConfig default is true.
    Ok(RungFabricationSettings {
        fabrication: awx.fabrication,
        escalation: awx.escalation,
        overrides: awx.fabrication_overrides.clone(),
        clearance_floor_mm,
        track_floor_mm,
        may_narrow: true,
        via_rungs,
    })
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
pub enum ProbeEnd {
    Source,
    Destination,
}

/// One menu entry, retaining its physical pad identity so the exact option can
/// be selected again during native realization.
#[derive(Clone, Debug, Serialize)]
pub struct ProbeCandidate {
    pub net: String,
    pub ball: String,
    pub src_ball: Option<String>,
    pub dst_ball: Option<String>,
    pub end: ProbeEnd,
    pub movement: Move,
    pub src: Option<Move>,
    pub dst: Option<Move>,
    pub comove: Vec<String>,
    /// Additional lanes explicitly selected by a rescue/gatekeeper phase.
    pub extra_relay: Vec<String>,
    pub tag: String,
}

#[derive(Clone, Debug, Serialize)]
pub struct RankedCandidate {
    pub candidate: ProbeCandidate,
    pub cost: f64,
    pub co_move_conflicts: usize,
}

#[derive(Clone)]
pub struct ProbeResult {
    pub candidate: ProbeCandidate,
    /// Exact fanout copper from which the landed routed board was produced.
    pub fanout_after: BoardAdapter,
    /// Real native board after candidate-only route and materialization.
    pub board: BoardAdapter,
    pub grade: RouteGrade,
    pub achieved_src: Option<Move>,
    pub achieved_dst: Option<Move>,
    pub comove_got: BTreeMap<String, Move>,
    pub relaid: Vec<String>,
    pub refused: bool,
    pub refused_nets: Vec<String>,
    /// Real constrained-router collision frontier from refused last calls.
    pub rung_reports: BTreeMap<String, braid_connect::RungReport>,
    pub fail: Option<String>,
    pub unjudged: bool,
}

#[derive(Default)]
pub struct ProbeCache {
    rows: BTreeMap<String, ProbeResult>,
}

/// Build the exact source cost model from the independent native replan-bus
/// producer and the current source braid-model output. Missing prediction or
/// bus provenance is an error; no estimated bus order or residual is invented.
pub fn replan_cost_model(state: &ReplanState) -> Result<ReplanCostModel, String> {
    let buses = super::replan_buses::produce_replan_bus_inputs(state)?;
    let current_prediction = state.current_model_prediction()?;
    let mut tooth_layer = BTreeMap::new();
    let mut tooth_vias = BTreeMap::new();
    for name in &state.names {
        let movement = state
            .ends
            .get(name)
            .and_then(|ends| ends.src_move.clone())
            .or_else(|| {
                exact_current_move(state, &state.probe.source_menus, name, ProbeEnd::Source)
                    .map(|(_, movement)| movement)
            })
            .ok_or_else(|| {
                format!("current source tooth move unavailable for cost model net {name}")
            })?;
        tooth_layer.insert(name.clone(), movement.layer);
        tooth_vias.insert(name.clone(), movement.vias);
    }
    let residual = state
        .names
        .iter()
        .map(|name| (name.clone(), state.residual.get(name).copied()))
        .collect();
    ReplanCostModel::new(ReplanCostInput {
        names: state.names.clone(),
        destination_choice: buses.destination_choice,
        launch: buses.launch,
        tooth_layer,
        tooth_vias,
        buses: buses.buses,
        source_box: buses.source_box,
        destination_box: buses.destination_box,
        chi: buses.chi,
        exact_pages: state.probe.exact_pages,
        swimmer_page_price: state.probe.swimmer_page_price,
        swim_vias: state.probe.swim_vias,
        residual,
        current_prediction,
    })
}

fn json_key<T: serde::Serialize>(value: &T) -> Result<String, String> {
    serde_json::to_string(value)
        .map_err(|error| format!("serialize replan probe identity: {error}"))
}

fn group_for<'a>(state: &'a ReplanState, net: &str) -> Result<&'a RouteGroup, String> {
    state
        .world
        .spec
        .groups
        .iter()
        .find(|group| group.nets.iter().any(|name| name == net))
        .ok_or_else(|| format!("replan candidate net {net} has no route group"))
}

fn scoped_group(state: &ReplanState, group: &RouteGroup, names: &BTreeSet<String>) -> RouteGroup {
    let mut out = group.clone();
    out.nets = names
        .iter()
        .filter(|net| {
            state
                .world
                .spec
                .nets
                .iter()
                .any(|known| &known.name == *net)
        })
        .cloned()
        .collect();
    if let Some(awx) = out.awx.as_mut() {
        awx.pairs = state
            .world
            .spec
            .groups
            .iter()
            .filter(|candidate| {
                candidate.awx.as_ref().is_some_and(|other| {
                    other.source == awx.source && other.destination == awx.destination
                })
            })
            .flat_map(|candidate| {
                candidate
                    .awx
                    .iter()
                    .flat_map(|other| other.pairs.iter().cloned())
            })
            .filter(|pair| names.contains(&pair.p) && names.contains(&pair.n))
            .collect();
    }
    out
}

fn same_move(a: &Move, b: &Move) -> bool {
    match (serde_json::to_value(a), serde_json::to_value(b)) {
        (Ok(a), Ok(b)) => a == b,
        _ => false,
    }
}

fn movement_from_option(option: &OptionMove) -> Option<&Move> {
    match option {
        OptionMove::Escape { movement } => Some(movement),
        _ => None,
    }
}

fn current_for(end: &EndState, side: ProbeEnd) -> Option<&Move> {
    match side {
        ProbeEnd::Source => end.src_move.as_ref(),
        ProbeEnd::Destination => end.dst_move.as_ref(),
    }
}

fn class_key(movement: &Move, end: ProbeEnd) -> String {
    let direction = format!("{:?}", movement.direction);
    let kind = format!("{:?}", movement.kind);
    match end {
        ProbeEnd::Destination => format!("{direction}|{}", movement.layer),
        ProbeEnd::Source => format!("{direction}|{}|{kind}|{}", movement.layer, movement.climb),
    }
}

fn close_to_current(movement: &Move, current: &EndState, end: ProbeEnd) -> bool {
    let point = match end {
        ProbeEnd::Source => current.tooth,
        ProbeEnd::Destination => current.exit,
    };
    let layer = match end {
        ProbeEnd::Source => &current.tooth_layer,
        ProbeEnd::Destination => &current.dest_layer,
    };
    movement.layer == *layer
        && (movement.exit_pt[0] - point[0]).hypot(movement.exit_pt[1] - point[1]) < 0.35
}

fn proper_cross(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> bool {
    let orient = |p: [f64; 2], q: [f64; 2], r: [f64; 2]| {
        (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    };
    let eps = 1e-9;
    let ab_c = orient(a, b, c);
    let ab_d = orient(a, b, d);
    let cd_a = orient(c, d, a);
    let cd_b = orient(c, d, b);
    ((ab_c > eps && ab_d < -eps) || (ab_c < -eps && ab_d > eps))
        && ((cd_a > eps && cd_b < -eps) || (cd_a < -eps && cd_b > eps))
}

type CopperSegment = ([f64; 2], [f64; 2], String);

fn point_segment_distance(point: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let length2 = dx * dx + dy * dy;
    let t = if length2 <= f64::EPSILON {
        0.0
    } else {
        (((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length2).clamp(0.0, 1.0)
    };
    (point[0] - (a[0] + t * dx)).hypot(point[1] - (a[1] + t * dy))
}

fn segment_distance(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> f64 {
    if proper_cross(a, b, c, d) {
        0.0
    } else {
        point_segment_distance(a, c, d)
            .min(point_segment_distance(b, c, d))
            .min(point_segment_distance(c, a, b))
            .min(point_segment_distance(d, a, b))
    }
}

fn net_segments(board: &BoardAdapter, net: &str) -> Vec<CopperSegment> {
    let Ok(number) = board.net_number(net) else {
        return Vec::new();
    };
    let scale = board.units_per_mm;
    let mut result = Vec::new();
    for key in board.board.get_traces() {
        let item = board.board.item(key);
        if !item.net_numbers().contains(&number) {
            continue;
        }
        let layer = board.board.layer_structure.layers[item.first_layer(&board.board) as usize]
            .name
            .clone();
        let points = item.trace().polyline().corner_approx_arr();
        for pair in points.windows(2) {
            result.push((
                [pair[0].x / scale, pair[0].y / scale],
                [pair[1].x / scale, pair[1].y / scale],
                layer.clone(),
            ));
        }
    }
    result
}

fn net_via_points(board: &BoardAdapter, net: &str) -> Vec<[f64; 2]> {
    let Ok(number) = board.net_number(net) else {
        return Vec::new();
    };
    let scale = board.units_per_mm;
    board
        .board
        .get_vias()
        .into_iter()
        .filter_map(|key| {
            let item = board.board.item(key);
            item.net_numbers().contains(&number).then(|| {
                let center = item.center(&board.board).to_float();
                [center.x / scale, center.y / scale]
            })
        })
        .collect()
}

fn near_point(a: [f64; 2], b: [f64; 2], tolerance: f64) -> bool {
    (a[0] - b[0]).abs() < tolerance && (a[1] - b[1]).abs() < tolerance
}

/// Match replan.py::conflicts against routed lane copper absent from each
/// lane's last fanout provenance board.
fn lane_conflicts(state: &ReplanState, movement: &Move, net: &str) -> Vec<String> {
    let mut out = Vec::new();
    for other in &state.names {
        if other == net {
            continue;
        }
        let fanout = state.fan_src.get(other).unwrap_or(&state.fanout_seed);
        let fanout_segments = net_segments(fanout, other);
        let fanout_vias = net_via_points(fanout, other);
        let segments: Vec<_> = net_segments(&state.world.routed, other)
            .into_iter()
            .filter(|segment| {
                !fanout_segments.iter().any(|old| {
                    segment.2 == old.2
                        && ((near_point(segment.0, old.0, 0.002)
                            && near_point(segment.1, old.1, 0.002))
                            || (near_point(segment.0, old.1, 0.002)
                                && near_point(segment.1, old.0, 0.002)))
                })
            })
            .collect();
        let vias: Vec<_> = net_via_points(&state.world.routed, other)
            .into_iter()
            .filter(|via| !fanout_vias.iter().any(|old| near_point(*via, *old, 0.002)))
            .collect();
        let hit = movement.legs.iter().any(|leg| {
            segments.iter().any(|(a, b, layer)| {
                layer == &leg.layer && segment_distance(leg.a, leg.b, *a, *b) < 0.30
            }) || vias
                .iter()
                .any(|via| point_segment_distance(*via, leg.a, leg.b) < 0.35)
        }) || movement.site.is_some_and(|site| {
            segments
                .iter()
                .any(|(a, b, _)| point_segment_distance(site, *a, *b) < 0.35)
                || vias
                    .iter()
                    .any(|via| (site[0] - via[0]).hypot(site[1] - via[1]) < 0.70)
        });
        if hit {
            out.push(other.clone());
        }
    }
    out
}

fn berth_conflicts(state: &ReplanState, movement: &Move, net: &str) -> Vec<String> {
    let [x0, y0, x1, y1] = state.probe.destination_menus.grid.bbox;
    let inside = |point: [f64; 2]| {
        point[0] >= x0 - 0.3 && point[0] <= x1 + 0.3 && point[1] >= y0 - 0.3 && point[1] <= y1 + 0.3
    };
    let mut out = Vec::new();
    for other in &state.names {
        if other == net {
            continue;
        }
        let segments: Vec<_> = net_segments(&state.fanout_seed, other)
            .into_iter()
            .filter(|(a, b, _)| inside(*a) && inside(*b))
            .collect();
        let vias: Vec<_> = net_via_points(&state.fanout_seed, other)
            .into_iter()
            .filter(|point| inside(*point))
            .collect();
        let hit = movement.legs.iter().any(|leg| {
            segments.iter().any(|(a, b, layer)| {
                layer == &leg.layer && segment_distance(leg.a, leg.b, *a, *b) < 0.30
            }) || vias
                .iter()
                .any(|via| point_segment_distance(*via, leg.a, leg.b) < 0.35)
        }) || movement.site.is_some_and(|site| {
            segments
                .iter()
                .any(|(a, b, _)| point_segment_distance(site, *a, *b) < 0.35)
                || vias
                    .iter()
                    .any(|via| (site[0] - via[0]).hypot(site[1] - via[1]) < 0.70)
        });
        if hit {
            out.push(other.clone());
        }
    }
    out
}

fn crosses_current_berth(state: &ReplanState, movement: &Move, net: &str) -> bool {
    state.ends.iter().any(|(other_net, ends)| {
        if other_net == net {
            return false;
        }
        let other = ends.dst_move.clone().or_else(|| {
            exact_current_move(
                state,
                &state.probe.destination_menus,
                other_net,
                ProbeEnd::Destination,
            )
            .map(|(_, movement)| movement)
        });
        let Some(other) = other.as_ref() else {
            return false;
        };
        movement.legs.iter().any(|leg| {
            other.legs.iter().any(|other_leg| {
                leg.layer == other_leg.layer && proper_cross(leg.a, leg.b, other_leg.a, other_leg.b)
            })
        })
    })
}

/// Return the legal, class-deduplicated menu in model-unsorted order. The
/// caller applies the source B.cost model to this list; this function never
/// substitutes a hand-written cost for that model.
pub fn candidate_menu(
    state: &ReplanState,
    net: &str,
    end: ProbeEnd,
    bans: &BTreeSet<String>,
    limit: usize,
) -> Result<(Vec<ProbeCandidate>, usize), String> {
    let _group = group_for(state, net)?;
    let menus = match end {
        ProbeEnd::Source => &state.probe.source_menus,
        ProbeEnd::Destination => &state.probe.destination_menus,
    };
    let current_class = if end == ProbeEnd::Destination {
        exact_current_move(state, menus, net, end)
            .map(|(_, movement)| (movement.direction, movement.layer))
    } else {
        None
    };
    let current = state.ends.get(net);
    let mut entries = Vec::new();
    for (ball, options) in &menus.options {
        if !ball.starts_with(&format!("{}#", escape_moves::short_name(net))) {
            continue;
        }
        for option in options {
            let Some(movement) = movement_from_option(&option.option) else {
                continue;
            };
            if movement.net != net
                && escape_moves::short_name(&movement.net) != escape_moves::short_name(net)
            {
                continue;
            }
            if current.is_some_and(|ends| {
                current_for(ends, end).is_some_and(|old| same_move(old, movement))
                    || close_to_current(movement, ends, end)
            }) {
                continue;
            }
            if current_class.as_ref().is_some_and(|(direction, layer)| {
                movement.direction == *direction && movement.layer == *layer
            }) {
                continue;
            }
            if end == ProbeEnd::Source
                && state.ends.iter().any(|(other, ends)| {
                    other != net
                        && (movement.exit_pt[0] - ends.tooth[0]).abs() < 0.16
                        && (movement.exit_pt[1] - ends.tooth[1]).abs() < 0.16
                })
            {
                continue;
            }
            if end == ProbeEnd::Destination && crosses_current_berth(state, movement, net) {
                continue;
            }
            if end == ProbeEnd::Destination
                && movement.kind == super::escape_moves::MoveKind::Surface
            {
                if let (Some(pad), Some(grid)) = (menus.items.get(ball), Some(&menus.grid)) {
                    let axis = if matches!(
                        movement.direction,
                        escape_moves::Direction::Up | escape_moves::Direction::Down
                    ) {
                        1
                    } else {
                        0
                    };
                    let pitch = if axis == 1 {
                        grid.pitch_y
                    } else {
                        grid.pitch_x
                    };
                    let depth = if pitch > 0.0 {
                        (movement.exit_pt[axis] - pad.point[axis]).abs() / pitch
                    } else {
                        f64::INFINITY
                    };
                    if depth > 3.6 {
                        continue;
                    }
                }
            }
            let key = class_key(movement, end);
            if bans.contains(&key) {
                continue;
            }
            let comove = if end == ProbeEnd::Destination {
                berth_conflicts(state, movement, net)
            } else {
                vec![]
            };
            if end == ProbeEnd::Destination && comove.len() > 3 {
                continue;
            }
            entries.push(ProbeCandidate {
                net: net.to_owned(),
                ball: ball.clone(),
                src_ball: (end == ProbeEnd::Source).then(|| ball.clone()),
                dst_ball: (end == ProbeEnd::Destination).then(|| ball.clone()),
                end,
                movement: movement.clone(),
                src: (end == ProbeEnd::Source).then(|| movement.clone()),
                dst: (end == ProbeEnd::Destination).then(|| movement.clone()),
                comove,
                extra_relay: vec![],
                tag: format!(
                    "{}:{}:{}",
                    net,
                    if end == ProbeEnd::Source {
                        "src"
                    } else {
                        "dst"
                    },
                    entries.len()
                ),
            });
        }
    }
    let menu_len = entries.len();
    let mut seen = BTreeSet::new();
    entries.retain(|candidate| seen.insert(class_key(&candidate.movement, end)));
    entries.truncate(limit);
    Ok((entries, menu_len))
}

/// Package a cross/jump move with the exact physical pad key needed by the
/// native realization plan. If a move was synthesized outside the menu, the
/// closest same-net pad is retained; `probe_one` will still reject it unless
/// the native menu contains that exact physical option.
pub fn candidate_for_move(
    state: &ReplanState,
    net: &str,
    end: ProbeEnd,
    movement: Move,
    tag: impl Into<String>,
) -> Result<ProbeCandidate, String> {
    if escape_moves::short_name(&movement.net) != escape_moves::short_name(net) {
        return Err(format!(
            "move net {} does not match candidate {net}",
            movement.net
        ));
    }
    let menus = match end {
        ProbeEnd::Source => &state.probe.source_menus,
        ProbeEnd::Destination => &state.probe.destination_menus,
    };
    let exact = menus.options.iter().find_map(|(ball, options)| {
        options
            .iter()
            .any(|option| {
                movement_from_option(&option.option).is_some_and(|old| same_move(old, &movement))
            })
            .then(|| ball.clone())
    });
    let point = movement.legs.first().map(|leg| leg.a).or_else(|| {
        state.ends.get(net).map(|ends| match end {
            ProbeEnd::Source => ends.tooth,
            ProbeEnd::Destination => ends.exit,
        })
    });
    let ball = exact.or_else(|| {
        menus
            .items
            .iter()
            .filter(|(_, pad)| escape_moves::short_name(&pad.net) == escape_moves::short_name(net))
            .min_by(|(_, a), (_, b)| {
                let distance = |pad: &&super::escape_moves::Pad| {
                    point.map_or(f64::INFINITY, |p| {
                        (pad.point[0] - p[0]).hypot(pad.point[1] - p[1])
                    })
                };
                distance(a).total_cmp(&distance(b))
            })
            .map(|(ball, _)| ball.clone())
    });
    let Some(ball) = ball else {
        return Err(format!(
            "no native {} pad found for {net}",
            match end {
                ProbeEnd::Source => "source",
                ProbeEnd::Destination => "destination",
            }
        ));
    };
    Ok(ProbeCandidate {
        net: net.to_owned(),
        ball: ball.clone(),
        src_ball: (end == ProbeEnd::Source).then(|| ball.clone()),
        dst_ball: (end == ProbeEnd::Destination).then(|| ball.clone()),
        end,
        movement: movement.clone(),
        src: (end == ProbeEnd::Source).then_some(movement.clone()),
        dst: (end == ProbeEnd::Destination).then_some(movement.clone()),
        comove: if end == ProbeEnd::Destination {
            berth_conflicts(state, &movement, net)
        } else {
            vec![]
        },
        extra_relay: vec![],
        tag: tag.into(),
    })
}

/// Apply the source `B.cost` model to every legal menu candidate and preserve
/// the reference ordering: model cost, co-move count, vias, then climb. The
/// cost model itself is supplied by `replan_cost`; this module does not invent
/// a surrogate score.
pub fn rank_candidates(
    state: &ReplanState,
    net: &str,
    end: ProbeEnd,
    bans: &BTreeSet<String>,
    limit: usize,
    model: &ReplanCostModel,
) -> Result<(Vec<RankedCandidate>, usize), String> {
    let (candidates, menu_len) = candidate_menu(state, net, end, bans, usize::MAX)?;
    let mut ranked = candidates
        .into_iter()
        .map(|candidate| {
            let request = match end {
                ProbeEnd::Destination => ReplanCandidate::Destination {
                    net: net.to_owned(),
                    movement: candidate.movement.clone(),
                },
                ProbeEnd::Source => ReplanCandidate::Source {
                    net: net.to_owned(),
                    movement: candidate.movement.clone(),
                },
            };
            let cost = model.cost(&request)?;
            if !cost.is_finite() {
                return Err(format!("non-finite replan model cost for {net}"));
            }
            let co_move_conflicts = candidate.comove.len();
            Ok(RankedCandidate {
                candidate,
                cost,
                co_move_conflicts,
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    ranked.sort_by(|a, b| {
        a.cost
            .total_cmp(&b.cost)
            .then(a.co_move_conflicts.cmp(&b.co_move_conflicts))
            .then(a.candidate.movement.vias.cmp(&b.candidate.movement.vias))
            .then(a.candidate.movement.climb.cmp(&b.candidate.movement.climb))
    });
    ranked.truncate(limit);
    Ok((ranked, menu_len))
}

fn end_matches(movement: &Move, end: ProbeEnd, current: &EndState) -> bool {
    close_to_current(movement, current, end)
}

fn exact_current_move(
    state: &ReplanState,
    menus: &JointMenus,
    net: &str,
    end: ProbeEnd,
) -> Option<(String, Move)> {
    let current = state.ends.get(net)?;
    let wanted = current_for(current, end);
    menus.options.iter().find_map(|(ball, options)| {
        if !ball.starts_with(&format!("{}#", escape_moves::short_name(net))) {
            return None;
        }
        options.iter().find_map(|option| {
            let movement = movement_from_option(&option.option)?;
            let found = wanted.is_some_and(|m| same_move(m, movement))
                || end_matches(movement, end, current);
            found.then(|| (ball.clone(), movement.clone()))
        })
    })
}

fn set_plan_move(
    plan: &mut JointPlan,
    menus: &JointMenus,
    ball: &str,
    movement: &Move,
) -> Result<(), String> {
    let options = menus
        .options
        .get(ball)
        .ok_or_else(|| format!("probe menu lost pad {ball}"))?;
    let Some((index, option)) = options.iter().enumerate().find(|(_, option)| {
        movement_from_option(&option.option).is_some_and(|old| same_move(old, movement))
    }) else {
        return Err(format!(
            "candidate {} is absent from its native menu",
            movement.net
        ));
    };
    plan.chosen.insert(ball.to_owned(), option.option.clone());
    plan.pieces.insert(ball.to_owned(), option.piece.clone());
    plan.selected_indices.insert(ball.to_owned(), index);
    Ok(())
}

fn retain_plan_nets(plan: &mut JointPlan, names: &BTreeSet<String>) {
    let keep: BTreeSet<_> = plan
        .pieces
        .iter()
        .filter_map(|(ball, piece)| {
            (names.contains(&piece.net)
                || names.iter().any(|name| {
                    escape_moves::short_name(name) == escape_moves::short_name(&piece.net)
                }))
            .then(|| ball.clone())
        })
        .collect();
    plan.pieces.retain(|ball, _| keep.contains(ball));
    plan.chosen.retain(|ball, _| keep.contains(ball));
    plan.selected_indices.retain(|ball, _| keep.contains(ball));
    plan.hints.retain(|hint| keep.contains(&hint.ball));
}

fn probe_corridor_overrides(
    state: &ReplanState,
) -> Result<super::braid_setup::BraidCorridorOverrides, String> {
    let env = &state.environment_overrides;
    let float = |key: &str, empty: Option<f64>| -> Result<Option<f64>, String> {
        env.get(key)
            .map(|value| {
                if value.is_empty() {
                    return Ok(empty);
                }
                value.parse::<f64>().map(Some).map_err(|_| {
                    format!("invalid {key} value in native replan environment: {value:?}")
                })
            })
            .transpose()
            .map(Option::flatten)
            .and_then(|value| {
                if value.is_some_and(|value| !value.is_finite()) {
                    Err(format!("non-finite {key} in native replan environment"))
                } else {
                    Ok(value)
                }
            })
    };
    let integer = |key: &str, empty: Option<i64>| -> Result<Option<i64>, String> {
        env.get(key)
            .map(|value| {
                if value.is_empty() {
                    return Ok(empty);
                }
                value.parse::<i64>().map(Some).map_err(|_| {
                    format!("invalid {key} value in native replan environment: {value:?}")
                })
            })
            .transpose()
            .map(Option::flatten)
    };
    let pair_gap = float("BRAID_PAIR_GAP", Some(0.0))?.filter(|value| *value != 0.0);
    let block_push = integer("BRAID_BLOCK_PUSH", Some(0))?
        .map(|value| u8::try_from(value).map_err(|_| "BRAID_BLOCK_PUSH must fit 0..255"))
        .transpose()?;
    let pages_siders = integer("PLAN_PAGES_SIDERS", Some(0))?
        .map(|value| u8::try_from(value).map_err(|_| "PLAN_PAGES_SIDERS must fit 0..255"))
        .transpose()?;
    let pairs = integer("BRAID_PAIRS", None)?.map(|value| value != 0);
    Ok(super::braid_setup::BraidCorridorOverrides {
        track_width: None,
        spec_clearance: None,
        via_size: None,
        grid: None,
        lane_pitch: None,
        exit_pitch: None,
        pairs,
        block_push,
        pair_gap,
        pair_dive_extra_pitches: float("BRAID_PAIR_DIVE_EXTRA", Some(0.0))?,
        jog_via: float("BRAID_JOG_VIA", Some(1.0))?,
        approach_reserve: float("BRAID_APPROACH_RESERVE", Some(1.0))?,
        end_keep: None,
        pages_siders,
        slope_pitch: None,
        branch: env.get("BRAID_BRANCH").map(|value| value != "0"),
    })
}

fn probe_corridor_states(
    state: &ReplanState,
    names: &[String],
    snapshot: &BoardSnapshot,
) -> Result<Vec<super::braid_corridor::CorridorState>, String> {
    state
        .world
        .spec
        .awx
        .as_ref()
        .ok_or("native probe corridor requires AWX metadata")?;
    // `replan.py::braid_run` starts the standalone braid CLI from the
    // realized board. It has no plan seed or router Move directions, so infer
    // endpoints and layers from that physical board as the source does.
    let seed = super::braid_plan::BraidPlanSeed {
        ends: BTreeMap::new(),
        tooth_layer: BTreeMap::new(),
        dest_layer: BTreeMap::new(),
        tooth_dir: BTreeMap::new(),
        stub_dir: BTreeMap::new(),
        chi: state.probe.chi,
        berth_sig: BTreeMap::new(),
    };
    let corridor_overrides = probe_corridor_overrides(state)?;
    let pairs = corridor_overrides.pairs.unwrap_or(false);
    let corridor_options =
        super::braid_setup::resolve_corridor_options(&state.probe.rules, &corridor_overrides)?;
    let setup = super::braid_setup::prepare(&super::braid_plan::BraidPlanInput {
        snapshot,
        source: &state.probe.source_part,
        destination: &state.probe.destination_part,
        names,
        seed: &seed,
        options: &super::braid_plan::BraidSetupOptions { pairs },
        rules: &state.probe.rules,
        corridor_options: &corridor_options,
        exact_pages: state.probe.exact_pages,
        swimmer_page_price: state.probe.swimmer_page_price,
    })?;
    super::braid_corridor::build_corridor_states(
        &setup,
        &corridor_options,
        state.probe.exact_pages,
        state.probe.swimmer_page_price,
    )
}

fn corridor_route_order(state: &CorridorState) -> Vec<String> {
    let swimmers: BTreeSet<_> = state
        .sched_cur
        .as_ref()
        .map(|schedule| schedule.swimmers.iter().cloned().collect())
        .unwrap_or_default();
    let mut order: Vec<_> = state
        .target
        .iter()
        .filter(|net| !swimmers.contains(*net))
        .cloned()
        .collect();
    let mut swimmer_order: Vec<_> = state
        .members
        .iter()
        .filter(|net| swimmers.contains(*net))
        .cloned()
        .collect();
    swimmer_order.sort_by(|a, b| {
        let displacement = |net: &str| {
            (state.launch_o.get(net).copied().unwrap_or(0.0)
                - state.target_o.get(net).copied().unwrap_or(0.0))
            .abs()
        };
        displacement(b).total_cmp(&displacement(a))
    });
    order.extend(swimmer_order);
    order
}

fn destination_alternatives(state: &ReplanState, net: &str) -> Vec<RungEndpoint> {
    let mut out = Vec::new();
    for options in state.probe.destination_menus.options.values() {
        for option in options {
            let Some(movement) = movement_from_option(&option.option) else {
                continue;
            };
            if escape_moves::short_name(&movement.net) != escape_moves::short_name(net) {
                continue;
            }
            let endpoint = RungEndpoint {
                point: movement.exit_pt,
                layer: movement.layer.clone(),
            };
            if !out.iter().any(|old: &RungEndpoint| {
                old.layer == endpoint.layer
                    && (old.point[0] - endpoint.point[0]).abs() < 1e-6
                    && (old.point[1] - endpoint.point[1]).abs() < 1e-6
            }) {
                out.push(endpoint);
            }
        }
    }
    out
}

fn pair_connectors(
    band: &CorridorBandContext,
    net: &str,
) -> (
    Option<Vec<braid_connect::PairConnector>>,
    Option<Vec<braid_connect::PairConnector>>,
) {
    let Some(points) = band.lanes.get(net).filter(|points| points.len() >= 3) else {
        return (None, None);
    };
    let lengths: Vec<_> = points
        .windows(2)
        .map(|segment| (segment[1][0] - segment[0][0]).hypot(segment[1][1] - segment[0][1]))
        .collect();
    let mut cumulative = vec![0.0];
    for length in &lengths {
        cumulative.push(cumulative.last().copied().unwrap_or(0.0) + length);
    }
    let total = *cumulative.last().unwrap_or(&0.0);
    if total < 1.5 {
        return (None, None);
    }
    let walk = |target: f64, reverse: bool| -> Option<braid_connect::PairConnector> {
        for (index, length) in lengths.iter().copied().enumerate() {
            if length <= 1e-9 || cumulative[index] + length < target {
                continue;
            }
            let fraction = (target - cumulative[index]) / length;
            let p = points[index];
            let q = points[index + 1];
            let sign = if reverse { -1.0 } else { 1.0 };
            return Some(braid_connect::PairConnector {
                point: [
                    p[0] + (q[0] - p[0]) * fraction,
                    p[1] + (q[1] - p[1]) * fraction,
                ],
                direction: [sign * (q[0] - p[0]) / length, sign * (q[1] - p[1]) / length],
            });
        }
        None
    };
    let on_run = |index: usize| {
        cumulative[index]
            + 0.05_f64.max(1.0_f64.max(0.5 * lengths[index]).min(lengths[index] - 0.1))
    };
    let mut starts = vec![walk(on_run(0), false)]
        .into_iter()
        .flatten()
        .collect::<Vec<_>>();
    let last = lengths.len() - 1;
    let end_target = cumulative[last] + lengths[last] - (on_run(last) - cumulative[last]);
    let mut ends = vec![walk(end_target, true)]
        .into_iter()
        .flatten()
        .collect::<Vec<_>>();
    if lengths.len() >= 2 {
        if let Some(connector) = walk(on_run(1), false) {
            starts.push(connector);
        }
        let prior = lengths.len() - 2;
        let prior_target = cumulative[prior] + lengths[prior] - (on_run(prior) - cumulative[prior]);
        if let Some(connector) = walk(prior_target, true) {
            ends.push(connector);
        }
    }
    if starts
        .first()
        .zip(ends.first())
        .is_some_and(|(start, end)| {
            (start.point[0] - end.point[0]).hypot(start.point[1] - end.point[1]) < 1.5
        })
    {
        return (None, None);
    }
    (
        (!starts.is_empty()).then_some(starts),
        (!ends.is_empty()).then_some(ends),
    )
}

fn pair_rung_request(
    state: &ReplanState,
    band: &CorridorBandContext,
    board: &BoardAdapter,
    net: &str,
) -> Result<Option<braid_connect::PairRungRequest>, String> {
    let Some([positive, negative]) = band.pair_members.get(net) else {
        return Ok(None);
    };
    let [[tooth_p, tooth_n], [berth_p, berth_n]] = band
        .pair_ends
        .get(net)
        .copied()
        .ok_or_else(|| format!("pair endpoint geometry missing for {net}"))?;
    let pair_layers = band
        .pair_layers
        .get(net)
        .ok_or_else(|| format!("pair layer geometry missing for {net}"))?;
    let allow_cross = state
        .environment_overrides
        .get("BRAID_PAIR_CROSS")
        .is_some_and(|value| value != "0" && !value.is_empty());
    let (start_connectors, end_connectors) = pair_connectors(band, net);
    Ok(Some(braid_connect::PairRungRequest {
        positive_net: positive.clone(),
        positive_net_id: board.net_number(positive)?,
        negative_net: negative.clone(),
        negative_net_id: board.net_number(negative)?,
        start_points: [tooth_p, tooth_n],
        end_points: [berth_p, berth_n],
        start_layer: band
            .tooth_layer
            .get(net)
            .cloned()
            .ok_or_else(|| format!("pair positive tooth layer missing for {net}"))?,
        end_layer: band
            .dest_layer
            .get(net)
            .cloned()
            .ok_or_else(|| format!("pair positive destination layer missing for {net}"))?,
        negative_start_layer: pair_layers[0].clone(),
        negative_end_layer: pair_layers[1].clone(),
        gap: band.options.pair_gap,
        lead_mm: 0.15,
        allow_cross,
        start_direction: band.tooth_dir.get(net).copied(),
        end_direction: band.stub_dir.get(net).copied(),
        start_connectors,
        end_connectors,
        a_given: None,
        b_given: None,
        x_given: None,
    }))
}

fn source_paste_apertures(
    snapshot: &BoardSnapshot,
) -> BTreeMap<String, braid_connect::RungPastePadMetadata> {
    snapshot
        .pads
        .iter()
        .map(|pad| {
            (
                pad.key.clone(),
                braid_connect::RungPastePadMetadata {
                    source_center: pad.point,
                    source_rotation_deg: pad.rect_rotation,
                    paste_apertures: snapshot
                        .paste_apertures(&pad.key)
                        .map(|apertures| apertures.to_vec()),
                },
            )
        })
        .collect()
}

struct ProbeBoardMetadata {
    snapshot: BoardSnapshot,
    paste_pads: BTreeMap<String, braid_connect::RungPastePadMetadata>,
    free_via_pads: Vec<super::native_grid_window::FreeViaPad>,
}

fn same_net_pad_metadata(
    state: &ReplanState,
    board: &BoardAdapter,
) -> Result<
    (
        Vec<crate::awx::whole_context::ContextPad>,
        Vec<crate::awx::whole_context::CopperVia>,
    ),
    String,
> {
    let awx = state
        .world
        .spec
        .awx
        .as_ref()
        .ok_or("native rung pad metadata requires AWX config")?;
    let current = BoardSnapshot::capture(board, &awx.components, &awx.via_drills)?;
    Ok((current.pads, current.vias))
}

fn probe_board_metadata(
    state: &ReplanState,
    board: &BoardAdapter,
    drills: &BTreeMap<String, f64>,
) -> Result<ProbeBoardMetadata, String> {
    let awx = state
        .world
        .spec
        .awx
        .as_ref()
        .ok_or("native probe metadata requires AWX configuration")?;
    let snapshot = BoardSnapshot::capture(board, &awx.components, drills)?;
    let mut free_via_pads = Vec::new();
    for pad in &snapshot.pads {
        if pad.non_plated_hole || !pad.drill.is_finite() || pad.drill <= 0.0 {
            continue;
        }
        let point_board = match board.point(pad.point)? {
            fr_geom::Point::Int(point) => point,
            fr_geom::Point::Rational(_) => continue,
        };
        let drill_board = (pad.drill * board.units_per_mm).round();
        if drill_board <= 0.0 || drill_board > i32::MAX as f64 {
            continue;
        }
        let drill_axis_board = pad
            .drill_axis
            .map(|axis| {
                let convert = |point| -> Result<[i32; 2], String> {
                    match board.point(point)? {
                        fr_geom::Point::Int(point) => Ok([point.x, point.y]),
                        fr_geom::Point::Rational(_) => {
                            Err("slot drill axis exceeds board coordinate range".into())
                        }
                    }
                };
                Ok::<[[i32; 2]; 2], String>([convert(axis[0])?, convert(axis[1])?])
            })
            .transpose()?;
        free_via_pads.push(super::native_grid_window::FreeViaPad {
            net_id: pad.net_id,
            point_board: [point_board.x, point_board.y],
            drill_board: drill_board as i32,
            non_plated_hole: pad.non_plated_hole,
            drill_axis_board,
        });
    }
    let paste_pads = source_paste_apertures(&snapshot);
    Ok(ProbeBoardMetadata {
        snapshot,
        paste_pads,
        free_via_pads,
    })
}

fn run_rung(
    state: &ReplanState,
    metadata: &ProbeBoardMetadata,
    group: &RouteGroup,
    corridor: &CorridorPlan,
    all_contexts: &[CorridorBandContext],
    band: &CorridorBandContext,
    board: BoardAdapter,
    net: &str,
    stage: &str,
    unrouted: &[String],
    landed: &BTreeSet<String>,
    corridor_done: &BTreeSet<usize>,
    deadline: Instant,
    bands: &mut SourceRungBandFactory,
    executor: &mut NativeRungExecutor,
) -> Result<braid_connect::RungResult<NativeRungRoute>, String> {
    let start = RungEndpoint {
        point: *band
            .teeth
            .get(net)
            .ok_or_else(|| format!("corridor source tooth missing for {net}"))?,
        layer: band
            .tooth_layer
            .get(net)
            .cloned()
            .ok_or_else(|| format!("corridor source layer missing for {net}"))?,
    };
    let end = RungEndpoint {
        point: *band
            .stubs
            .get(net)
            .ok_or_else(|| format!("corridor destination berth missing for {net}"))?,
        layer: band
            .dest_layer
            .get(net)
            .cloned()
            .ok_or_else(|| format!("corridor destination layer missing for {net}"))?,
    };
    let pair = pair_rung_request(state, band, &board, net)?;
    let net_id = pair
        .as_ref()
        .map(|pair| pair.positive_net_id)
        .map_or_else(|| board.net_number(net), Ok)?;
    let window_points = band
        .lanes
        .get(net)
        .filter(|points| !points.is_empty())
        .cloned()
        .unwrap_or_else(|| vec![start.point, end.point]);
    let mut virtual_tracks = band.virtual_of(unrouted)?;
    if matches!(stage, "last_call" | "rip") {
        virtual_tracks.extend(band.approach_virt(net)?);
    }
    let cross_reservations =
        braid_corridor::cross_reserve(all_contexts, band.index, net, landed, corridor_done)?;
    let virtual_vias = band.virtual_vias_of(unrouted)?;
    let route_layers = group
        .awx
        .as_ref()
        .ok_or_else(|| format!("route group for {net} has no AWX configuration"))?
        .routing_layers
        .clone();
    let mode = match state.probe.probe_ladder.as_str() {
        "open" => LadderMode::Open,
        "full" => LadderMode::Full,
        other => return Err(format!("unsupported PROBE_LADDER mode {other}")),
    };
    let base_iterations = 200_000usize;
    let max_iterations = if matches!(stage, "last_call" | "rip") {
        ((base_iterations.max(50_000) as f64 * state.probe.probe_budget_x.max(1.0))
            .ceil()
            .min(usize::MAX as f64)) as usize
    } else {
        base_iterations
    };
    let (same_net_pads, existing_vias) = same_net_pad_metadata(state, &board)?;
    let tap_fab = rung_fabrication_settings(state, &board, net_id, &route_layers)?;
    let base = RungBaseRequest {
        corridor: corridor.clone(),
        board,
        net: net.to_owned(),
        net_id,
        stage: stage.to_owned(),
        start,
        end,
        b_alternatives: destination_alternatives(state, net),
        pair: pair.clone(),
        window_points,
        virtual_tracks,
        cross_reservations,
        virtual_vias,
        rules: state.probe.rules.clone(),
        fabrication: tap_fab.fabrication,
        escalation: tap_fab.escalation,
        fabrication_overrides: tap_fab.overrides,
        fab_clearance_floor_mm: tap_fab.clearance_floor_mm,
        fab_track_floor_mm: tap_fab.track_floor_mm,
        may_narrow: tap_fab.may_narrow,
        fab_via_rungs: tap_fab.via_rungs,
        max_iterations,
        route_layers,
        paste_pads: metadata.paste_pads.clone(),
        free_via_pads: metadata.free_via_pads.clone(),
        same_net_pads,
        existing_vias,
        band_slack: pair.as_ref().map_or(0.0, |_| {
            (band.options.track_width + band.options.pair_gap) / 2.0
        }),
        deadline: Some(deadline),
    };
    braid_connect::connect_ladder(&base, mode, bands, executor)
}

fn free_rung_request(
    state: &ReplanState,
    metadata: &ProbeBoardMetadata,
    group: &RouteGroup,
    corridor: &CorridorPlan,
    all_contexts: &[CorridorBandContext],
    band: &CorridorBandContext,
    board: BoardAdapter,
    net: &str,
    stage: &str,
    unrouted: &[String],
    landed: &BTreeSet<String>,
    corridor_done: &BTreeSet<usize>,
    deadline: Instant,
    margin: f64,
    use_window: bool,
    use_approach: bool,
    use_cross_reservations: bool,
) -> Result<braid_connect::RungRequest, String> {
    let start = RungEndpoint {
        point: *band
            .teeth
            .get(net)
            .ok_or_else(|| format!("source tooth missing for {net}"))?,
        layer: band
            .tooth_layer
            .get(net)
            .cloned()
            .ok_or_else(|| format!("source layer missing for {net}"))?,
    };
    let end = RungEndpoint {
        point: *band
            .stubs
            .get(net)
            .ok_or_else(|| format!("destination berth missing for {net}"))?,
        layer: band
            .dest_layer
            .get(net)
            .cloned()
            .ok_or_else(|| format!("destination layer missing for {net}"))?,
    };
    let pair_data = pair_rung_request(state, band, &board, net)?;
    let single_route = matches!(stage, "econ" | "rip_free");
    let pair = if single_route {
        None
    } else {
        pair_data.clone()
    };
    let net_id = pair_data
        .as_ref()
        .map(|pair| pair.positive_net_id)
        .map_or_else(|| board.net_number(net), Ok)?;
    let route_net = pair_data
        .as_ref()
        .filter(|_| single_route)
        .map_or_else(|| net.to_owned(), |pair| pair.positive_net.clone());
    let window_points = band
        .lanes
        .get(net)
        .filter(|line| !line.is_empty())
        .cloned()
        .unwrap_or_else(|| vec![start.point, end.point]);
    let mut virtual_tracks = band.virtual_of(unrouted)?;
    if use_approach {
        virtual_tracks.extend(band.approach_virt(net)?);
    }
    if use_cross_reservations {
        virtual_tracks.extend(braid_corridor::cross_reserve(
            all_contexts,
            band.index,
            net,
            landed,
            corridor_done,
        )?);
    }
    let layers = group
        .awx
        .as_ref()
        .ok_or_else(|| format!("route group for {net} has no AWX settings"))?
        .routing_layers
        .clone();
    let spec = braid_connect::RungSpec {
        label: braid_connect::RungLabel::Free6,
        slack: 0.0,
        open_layers: false,
        margin,
        use_window: true,
    };
    let request_window = if !use_window {
        None
    } else if matches!(stage, "econ" | "rip_free") {
        band.lanes.get(net).filter(|line| !line.is_empty()).cloned()
    } else {
        Some(window_points)
    };
    let max_iterations = if matches!(stage, "rip" | "rip_probe" | "rip_free" | "econ") {
        ((200_000usize.max(50_000) as f64 * state.probe.probe_budget_x)
            .ceil()
            .min(usize::MAX as f64)) as usize
    } else {
        200_000
    };
    let (same_net_pads, existing_vias) = same_net_pad_metadata(state, &board)?;
    let tap_fab = rung_fabrication_settings(state, &board, net_id, &layers)?;
    Ok(braid_connect::RungRequest {
        board,
        net: route_net,
        net_id,
        stage: stage.to_owned(),
        start,
        end,
        b_alternatives: destination_alternatives(state, net),
        pair: pair.clone(),
        spec,
        band: None,
        band_slack: pair.as_ref().map_or(0.0, |_| {
            (band.options.track_width + band.options.pair_gap) / 2.0
        }),
        window_points: request_window,
        virtual_tracks,
        virtual_vias: band.virtual_vias_of(unrouted)?,
        rules: state.probe.rules.clone(),
        fabrication: tap_fab.fabrication,
        escalation: tap_fab.escalation,
        fabrication_overrides: tap_fab.overrides,
        fab_clearance_floor_mm: tap_fab.clearance_floor_mm,
        fab_track_floor_mm: tap_fab.track_floor_mm,
        may_narrow: tap_fab.may_narrow,
        fab_via_rungs: tap_fab.via_rungs,
        max_iterations,
        route_layers: layers,
        paste_pads: metadata.paste_pads.clone(),
        free_via_pads: metadata.free_via_pads.clone(),
        same_net_pads,
        existing_vias,
        deadline: Some(deadline),
    })
}

fn clip_virtual_round_ends(
    pieces: Vec<crate::awx::whole_context::ReservationTube>,
    ends: &[[f64; 2]],
    keep_radius: f64,
) -> Vec<crate::awx::whole_context::ReservationTube> {
    let mut clipped = Vec::new();
    for piece in pieces {
        let mut parts = vec![(piece.a, piece.b)];
        for &point in ends {
            let mut next = Vec::new();
            for (a, b) in parts {
                let delta = [b[0] - a[0], b[1] - a[1]];
                let length_squared = delta[0] * delta[0] + delta[1] * delta[1];
                if length_squared < 1e-12 {
                    continue;
                }
                let projection = (((point[0] - a[0]) * delta[0] + (point[1] - a[1]) * delta[1])
                    / length_squared)
                    .clamp(0.0, 1.0);
                let closest = [a[0] + projection * delta[0], a[1] + projection * delta[1]];
                if (closest[0] - point[0]).hypot(closest[1] - point[1]) >= keep_radius {
                    next.push((a, b));
                    continue;
                }
                let length = length_squared.sqrt();
                let t0 = (projection - keep_radius / length).max(0.0);
                let t1 = (projection + keep_radius / length).min(1.0);
                if t0 > 1e-6 {
                    next.push((a, [a[0] + t0 * delta[0], a[1] + t0 * delta[1]]));
                }
                if t1 < 1.0 - 1e-6 {
                    next.push(([a[0] + t1 * delta[0], a[1] + t1 * delta[1]], b));
                }
            }
            parts = next;
        }
        clipped.extend(
            parts
                .into_iter()
                .filter(|(a, b)| (a[0] - b[0]).hypot(a[1] - b[1]) > 1e-9)
                .map(|(a, b)| crate::awx::whole_context::ReservationTube {
                    a,
                    b,
                    ..piece.clone()
                }),
        );
    }
    clipped
}

fn run_main_lane(
    state: &ReplanState,
    metadata: &ProbeBoardMetadata,
    group: &RouteGroup,
    corridor: &CorridorPlan,
    all_contexts: &[CorridorBandContext],
    band: &CorridorBandContext,
    board: BoardAdapter,
    net: &str,
    stage: &str,
    unrouted: &[String],
    landed: &BTreeSet<String>,
    corridor_done: &BTreeSet<usize>,
    deadline: Instant,
    max_iterations: usize,
    boost_swimmer: bool,
    bands: &mut SourceRungBandFactory,
    executor: &mut NativeRungExecutor,
) -> Result<Option<NativeRungRoute>, String> {
    let start = RungEndpoint {
        point: *band
            .teeth
            .get(net)
            .ok_or_else(|| format!("corridor source tooth missing for {net}"))?,
        layer: band
            .tooth_layer
            .get(net)
            .cloned()
            .ok_or_else(|| format!("corridor source layer missing for {net}"))?,
    };
    let end = RungEndpoint {
        point: *band
            .stubs
            .get(net)
            .ok_or_else(|| format!("corridor destination berth missing for {net}"))?,
        layer: band
            .dest_layer
            .get(net)
            .cloned()
            .ok_or_else(|| format!("corridor destination layer missing for {net}"))?,
    };
    let window_points = band
        .lanes
        .get(net)
        .filter(|points| !points.is_empty())
        .cloned()
        .unwrap_or_else(|| vec![start.point, end.point]);
    let schedule = band
        .schedule
        .as_ref()
        .ok_or_else(|| format!("corridor schedule missing for {net}"))?;
    let swimmer = schedule.page_of(net).is_none();
    let spec = braid_connect::RungSpec {
        label: braid_connect::RungLabel::Slack03,
        slack: 0.0,
        open_layers: false,
        margin: if swimmer {
            if boost_swimmer {
                6.0
            } else {
                2.0
            }
        } else {
            0.6
        },
        use_window: true,
    };
    let band_mask = if swimmer {
        None
    } else {
        Some(bands.band_for(&corridor.band, net, spec)?)
    };
    let mut virtual_tracks = band.virtual_of(unrouted)?;
    virtual_tracks.extend(braid_corridor::cross_reserve(
        all_contexts,
        band.index,
        net,
        landed,
        corridor_done,
    )?);
    let keep = band.options.track_width + band.options.clearance + 0.05;
    virtual_tracks = clip_virtual_round_ends(virtual_tracks, &[start.point, end.point], keep);
    let pair = pair_rung_request(state, band, &board, net)?;
    let net_id = pair
        .as_ref()
        .map(|pair| pair.positive_net_id)
        .map_or_else(|| board.net_number(net), Ok)?;
    let (same_net_pads, existing_vias) = same_net_pad_metadata(state, &board)?;
    let route_layers = group
        .awx
        .as_ref()
        .ok_or_else(|| format!("route group for {net} has no AWX configuration"))?
        .routing_layers
        .clone();
    let tap_fab = rung_fabrication_settings(state, &board, net_id, &route_layers)?;
    let request = braid_connect::RungRequest {
        board,
        net: net.to_owned(),
        net_id,
        stage: stage.to_owned(),
        start,
        end,
        b_alternatives: Vec::new(),
        pair: pair.clone(),
        spec,
        band: band_mask,
        band_slack: pair.as_ref().map_or(0.0, |_| {
            (band.options.track_width + band.options.pair_gap) / 2.0
        }),
        window_points: Some(window_points),
        virtual_tracks,
        virtual_vias: band.virtual_vias_of(unrouted)?,
        rules: state.probe.rules.clone(),
        fabrication: tap_fab.fabrication,
        escalation: tap_fab.escalation,
        fabrication_overrides: tap_fab.overrides,
        fab_clearance_floor_mm: tap_fab.clearance_floor_mm,
        fab_track_floor_mm: tap_fab.track_floor_mm,
        may_narrow: tap_fab.may_narrow,
        fab_via_rungs: tap_fab.via_rungs,
        max_iterations,
        route_layers,
        paste_pads: metadata.paste_pads.clone(),
        free_via_pads: metadata.free_via_pads.clone(),
        same_net_pads,
        existing_vias,
        deadline: Some(deadline),
    };
    match executor.route(request)? {
        braid_connect::RungExecution::Found(route) => Ok(Some(route)),
        braid_connect::RungExecution::Refused { .. } => Ok(None),
    }
}

struct ProbeBraidOutcome {
    board: BoardAdapter,
    refused: Vec<String>,
    achieved: BTreeMap<String, RungEndpoint>,
    reports: BTreeMap<String, braid_connect::RungReport>,
}

pub struct LegacyBraidResult {
    pub board: BoardAdapter,
    pub refused: Vec<String>,
}

fn env_usize(state: &ReplanState, key: &str, default: usize) -> Result<usize, String> {
    state
        .environment_overrides
        .get(key)
        .map(|value| {
            value
                .parse::<usize>()
                .map_err(|_| format!("invalid {key} value in native replan environment: {value:?}"))
        })
        .unwrap_or(Ok(default))
}

fn legacy_group(state: &ReplanState) -> Result<RouteGroup, String> {
    let groups: Vec<_> = state
        .probe
        .groups
        .iter()
        .map(|id| {
            state
                .world
                .spec
                .groups
                .iter()
                .find(|group| &group.id == id)
                .ok_or_else(|| format!("native replan group {id} is absent from route spec"))
        })
        .collect::<Result<_, _>>()?;
    let first = groups
        .first()
        .ok_or("native replan has no active route group")?;
    let mut group = (**first).clone();
    group.nets = state.names.clone();
    if let Some(awx) = group.awx.as_mut() {
        awx.pairs = groups
            .iter()
            .flat_map(|group| {
                group
                    .awx
                    .iter()
                    .flat_map(|group| group.pairs.iter().cloned())
            })
            .filter(|pair| state.names.contains(&pair.p) && state.names.contains(&pair.n))
            .collect();
    }
    Ok(group)
}

/// Run the already-migrated legacy braid executor over the native fanout
/// board. Full mode inherits BRAID_* controls; probe mode uses PROBE_* values.
pub fn run_legacy_braid(
    state: &ReplanState,
    base_board: &BoardAdapter,
    full: bool,
) -> Result<LegacyBraidResult, String> {
    let mut configured = state.clone();
    if full {
        configured.probe.probe_attempts = env_usize(state, "BRAID_ATTEMPTS", 6)?;
        configured.probe.probe_budget_x = state
            .environment_overrides
            .get("BRAID_BUDGET_X")
            .map(|value| {
                value.parse::<f64>().map_err(|_| {
                    format!("invalid BRAID_BUDGET_X in native replan environment: {value:?}")
                })
            })
            .transpose()?
            .unwrap_or(4.0);
        configured.probe.probe_ladder = state
            .environment_overrides
            .get("BRAID_LADDER")
            .cloned()
            .unwrap_or_else(|| "full".into());
        configured.probe.probe_smooth = state
            .environment_overrides
            .get("BRAID_SMOOTH")
            .map_or(true, |value| value != "0");
    }
    let group = legacy_group(&configured)?;
    if configured.probe.probe_ladder != "open" && configured.probe.probe_ladder != "full" {
        return Err(format!(
            "unsupported {} ladder mode {}",
            if full { "BRAID" } else { "PROBE" },
            configured.probe.probe_ladder
        ));
    }
    let awx = configured
        .world
        .spec
        .awx
        .as_ref()
        .ok_or("native replan requires AWX configuration")?;
    let drills = awx.via_drills.clone();
    let deadline = configured.probe.deadline;
    let outcome = probe_braid(
        &configured,
        &group,
        &configured.names,
        base_board,
        &drills,
        deadline,
    )?;
    let mut board = outcome.board;
    if configured.probe.probe_smooth {
        smooth_legacy_board(&configured, &group, base_board, &drills, &mut board)?;
    }
    Ok(LegacyBraidResult {
        board,
        refused: outcome.refused,
    })
}

fn smooth_legacy_board(
    state: &ReplanState,
    group: &RouteGroup,
    fanout: &BoardAdapter,
    drills: &BTreeMap<String, f64>,
    board: &mut BoardAdapter,
) -> Result<(), String> {
    let awx = state
        .world
        .spec
        .awx
        .as_ref()
        .ok_or("native braid smoothing requires AWX metadata")?;
    let metadata = awx
        .smoothing_metadata
        .as_ref()
        .ok_or("BRAID_SMOOTH=1 requires explicit dense-pad smoothing metadata")?;
    let snapshot = BoardSnapshot::capture(board, &awx.components, drills)?;
    let dense_fields = metadata.dense_fields(&snapshot, &awx.rules)?;
    let base_ids: BTreeSet<_> = fanout
        .board
        .get_traces()
        .into_iter()
        .map(|key| fanout.board.item(key).id().0)
        .collect();
    let requested_ids: BTreeSet<_> = state
        .names
        .iter()
        .map(|net| board.net_number(net))
        .collect::<Result<_, _>>()?;
    let pair_ids: BTreeSet<_> = group
        .awx
        .as_ref()
        .into_iter()
        .flat_map(|meta| meta.pairs.iter())
        .flat_map(|pair| [pair.p.as_str(), pair.n.as_str()])
        .filter_map(|net| board.net_number(net).ok())
        .collect();
    let lane_items: Vec<_> = board
        .board
        .get_traces()
        .into_iter()
        .filter(|key| {
            let item = board.board.item(*key);
            !base_ids.contains(&item.id().0)
                && item
                    .net_numbers()
                    .iter()
                    .any(|net| requested_ids.contains(net) && !pair_ids.contains(net))
        })
        .collect();
    super::board_smoothing::smooth_board(board, &lane_items, &dense_fields)?;
    Ok(())
}

struct RipSolution {
    board: BoardAdapter,
    routed: Vec<String>,
    achieved: BTreeMap<String, RungEndpoint>,
    reports: BTreeMap<String, braid_connect::RungReport>,
    routes: BTreeMap<String, NativeRungRoute>,
}

#[derive(Default)]
struct LaneCopper {
    tracks: Vec<crate::awx::whole_context::ReservationTube>,
    vias: Vec<crate::awx::whole_context::CopperVia>,
}

fn physical_lane_nets(band: &CorridorBandContext, lane: &str) -> Vec<String> {
    band.pair_members
        .get(lane)
        .map(|members| vec![members[0].clone(), members[1].clone()])
        .unwrap_or_else(|| vec![lane.to_owned()])
}

fn lane_copper(
    state: &ReplanState,
    board: &BoardAdapter,
    baseline: &BoardAdapter,
    band: &CorridorBandContext,
    lane: &str,
) -> Result<(LaneCopper, LaneCopper), String> {
    let awx = state
        .world
        .spec
        .awx
        .as_ref()
        .ok_or("rip_for copper extraction requires AWX metadata")?;
    let nets = physical_lane_nets(band, lane);
    let net_ids: BTreeSet<_> = nets
        .iter()
        .map(|name| board.net_number(name))
        .collect::<Result<_, _>>()?;
    let baseline_ids: BTreeSet<_> = baseline
        .board
        .get_traces()
        .into_iter()
        .chain(baseline.board.get_vias())
        .filter_map(|key| {
            let item = baseline.board.item(key);
            item.net_numbers()
                .iter()
                .any(|net| net_ids.contains(net))
                .then_some(item.id().0)
        })
        .collect();
    let mut all = LaneCopper::default();
    let mut delta = LaneCopper::default();
    for key in board.board.get_traces() {
        let item = board.board.item(key);
        let Some(net_id) = item
            .net_numbers()
            .iter()
            .find(|net| net_ids.contains(net))
            .copied()
        else {
            continue;
        };
        let net_name = board
            .board
            .rules
            .nets
            .get(net_id)
            .map(|net| net.name.clone())
            .unwrap_or_else(|| lane.to_owned());
        let trace = item.trace();
        let layer = board.board.layer_structure.layers[trace.layer() as usize]
            .name
            .clone();
        let points: Vec<_> = trace
            .polyline()
            .corner_approx_arr()
            .into_iter()
            .map(|point| [point.x / board.units_per_mm, point.y / board.units_per_mm])
            .collect();
        let radius = trace.half_width() as f64 / board.units_per_mm;
        let into = |target: &mut LaneCopper| {
            for segment in points.windows(2) {
                target
                    .tracks
                    .push(crate::awx::whole_context::ReservationTube {
                        net: net_name.clone(),
                        a: segment[0],
                        b: segment[1],
                        layer: layer.clone(),
                        radius,
                    });
            }
        };
        into(&mut all);
        if !baseline_ids.contains(&item.id().0) {
            into(&mut delta);
        }
    }
    for key in board.board.get_vias() {
        let item = board.board.item(key);
        let Some(net_id) = item
            .net_numbers()
            .iter()
            .find(|net| net_ids.contains(net))
            .copied()
        else {
            continue;
        };
        let stack = item
            .padstack(&board.board)
            .ok_or_else(|| format!("rip_for via for {lane} has no padstack"))?;
        let diameter = (stack.from_layer()..=stack.to_layer())
            .filter_map(|layer| stack.get_shape(layer))
            .map(|shape| shape.max_width())
            .fold(0.0_f64, f64::max)
            / board.units_per_mm;
        let drill = *awx
            .via_drills
            .get(&stack.name)
            .ok_or_else(|| format!("rip_for via {} has no explicit drill metadata", stack.name))?;
        let point = item.center(&board.board).to_float();
        let point = [point.x / board.units_per_mm, point.y / board.units_per_mm];
        let layers = (stack.from_layer()..=stack.to_layer())
            .map(|layer| {
                board.board.layer_structure.layers[layer as usize]
                    .name
                    .clone()
            })
            .collect();
        let via = crate::awx::whole_context::CopperVia {
            net: board
                .board
                .rules
                .nets
                .get(net_id)
                .map(|net| net.name.clone())
                .unwrap_or_else(|| lane.to_owned()),
            net_id,
            point,
            diameter,
            drill,
            layers,
        };
        all.vias.push(via.clone());
        if !baseline_ids.contains(&item.id().0) {
            delta.vias.push(via);
        }
    }
    Ok((all, delta))
}

fn frontier_blocker_order(
    state: &ReplanState,
    board: &BoardAdapter,
    baseline: &BoardAdapter,
    band: &CorridorBandContext,
    group: &RouteGroup,
    landed: &BTreeSet<String>,
    refused: &BTreeSet<String>,
    current: &str,
    report: &braid_connect::RungReport,
) -> Result<Vec<String>, String> {
    if report.blocked_cells.is_empty() {
        return Ok(Vec::new());
    }
    let configured_layers = &group
        .awx
        .as_ref()
        .ok_or_else(|| "frontier analysis requires AWX route layers".to_owned())?
        .routing_layers;
    if configured_layers != &report.route_layers {
        return Err("rung frontier layer order differs from its route configuration".into());
    }
    let route_layers = &report.route_layers;
    let mut owner_by_id = BTreeMap::<i32, String>::new();
    let mut candidates = Vec::new();
    for lane in &band.members {
        if lane == current || refused.contains(lane) || !landed.contains(lane) {
            continue;
        }
        let (all, delta) = lane_copper(state, board, baseline, band, lane)?;
        if delta.tracks.is_empty() && delta.vias.is_empty() {
            continue;
        }
        let legs = physical_lane_nets(band, lane);
        let primary = legs
            .first()
            .ok_or_else(|| format!("lane {lane} has no conductor"))?;
        let net_id = board.net_number(primary)?;
        for leg in &legs {
            owner_by_id.insert(board.net_number(leg)?, lane.clone());
        }
        let mut tracks = Vec::new();
        for segment in all.tracks {
            let Some(layer) = route_layers
                .iter()
                .position(|layer| layer == &segment.layer)
            else {
                continue;
            };
            let width = segment.radius * 2.0;
            tracks.push(awx_grid::blocking_analysis::TrackObstacle {
                start: segment.a,
                end: segment.b,
                layer,
                margin: state.probe.rules.track_width / 2.0
                    + width / 2.0
                    + state.probe.rules.clearance,
            });
        }
        let vias = all
            .vias
            .into_iter()
            .map(|via| awx_grid::blocking_analysis::ViaObstacle {
                point: via.point,
                margin: via.diameter / 2.0
                    + state.probe.rules.track_width / 2.0
                    + state.probe.rules.clearance,
            })
            .collect();
        candidates.push(awx_grid::blocking_analysis::NetObstacles {
            net_id,
            net_name: lane.clone(),
            tracks,
            vias,
        });
    }
    let excluded: BTreeSet<_> = physical_lane_nets(band, current)
        .iter()
        .filter_map(|net| board.net_number(net).ok())
        .collect();
    let target = band.stubs.get(current).copied();
    let source = band.teeth.get(current).copied();
    let mut attributed = awx_grid::blocking_analysis::analyze_frontier_blocking(
        &report.blocked_cells,
        &candidates,
        &excluded,
        report.grid_step_mm,
        route_layers.len(),
        target,
        source,
    );
    let both_directions = BTreeSet::new();
    let costs = BTreeMap::new();
    let ranking = match state
        .environment_overrides
        .get("KICAD_RIPUP_BLOCKER_SELECT")
        .map(String::as_str)
        .unwrap_or("count")
    {
        "near-target" => awx_grid::blocking_analysis::Ranking::NearTarget,
        "bidir" => awx_grid::blocking_analysis::Ranking::Bidirectional(&both_directions),
        "cost" => awx_grid::blocking_analysis::Ranking::Cost(&costs),
        // `mincut` is a probe/reordering mode, not a blocking-analysis sort.
        "count" | "mincut" => awx_grid::blocking_analysis::Ranking::Count,
        other => return Err(format!("unsupported source rip blocker ranking {other}")),
    };
    awx_grid::blocking_analysis::rank_blockers(
        &mut attributed.entries,
        ranking,
        &BTreeSet::new(),
        0.0,
    );
    Ok(attributed
        .entries
        .into_iter()
        .filter_map(|entry| owner_by_id.get(&entry.net_id).cloned())
        .filter(|lane| lane != current && landed.contains(lane) && !refused.contains(lane))
        .fold(Vec::new(), |mut ordered, lane| {
            if !ordered.contains(&lane) {
                ordered.push(lane);
            }
            ordered
        }))
}

fn source_mincut_order(
    probe: &LaneCopper,
    victims: &[(String, LaneCopper)],
    rules: &super::whole_context::RoutingRules,
) -> Vec<String> {
    let mut cut = Vec::new();
    let reach = rules.clearance + rules.track_width;
    for path in &probe.tracks {
        for (name, copper) in victims {
            if cut.contains(name) {
                continue;
            }
            let hit = copper.tracks.iter().any(|obstacle| {
                obstacle.layer == path.layer
                    && segment_distance(path.a, path.b, obstacle.a, obstacle.b)
                        < reach + obstacle.radius - rules.track_width / 2.0 + 1e-6
            }) || copper.vias.iter().any(|via| {
                point_segment_distance(via.point, path.a, path.b)
                    < via.diameter / 2.0 + rules.clearance + rules.track_width / 2.0 + 1e-6
            });
            if hit && !cut.contains(name) {
                cut.push(name.clone());
            }
        }
    }
    for path_via in &probe.vias {
        for (name, copper) in victims {
            if cut.contains(name) {
                continue;
            }
            let hit = copper.tracks.iter().any(|obstacle| {
                point_segment_distance(path_via.point, obstacle.a, obstacle.b)
                    < path_via.diameter / 2.0 + rules.clearance + obstacle.radius + 1e-6
            }) || copper.vias.iter().any(|via| {
                (via.point[0] - path_via.point[0]).hypot(via.point[1] - path_via.point[1])
                    < (via.diameter + path_via.diameter) / 2.0 + rules.clearance + 1e-6
            });
            if hit && !cut.contains(name) {
                cut.push(name.clone());
            }
        }
    }
    cut
}

fn native_route_copper(route: &NativeRungRoute, lane: &str) -> LaneCopper {
    let mut copper = LaneCopper::default();
    for piece in &route.route_segments {
        copper
            .tracks
            .push(crate::awx::whole_context::ReservationTube {
                net: lane.to_owned(),
                a: piece.a,
                b: piece.b,
                layer: piece.layer.clone(),
                radius: piece.width / 2.0,
            });
    }
    for via in &route.route_vias {
        copper.vias.push(crate::awx::whole_context::CopperVia {
            net: lane.to_owned(),
            net_id: via.net_id,
            point: via.point,
            diameter: via.diameter,
            drill: via.drill,
            layers: via.layers.clone(),
        });
    }
    copper
}

fn rip_limits(state: &ReplanState) -> Result<(usize, usize), String> {
    let integer = |key: &str, default: usize| -> Result<usize, String> {
        state
            .environment_overrides
            .get(key)
            .map(|value| {
                value.parse::<usize>().map_err(|_| {
                    format!("invalid {key} value in native replan environment: {value:?}")
                })
            })
            .unwrap_or(Ok(default))
    };
    Ok((
        integer("BRAID_RIP_VICTIMS", 3)?,
        integer("BRAID_RIP_DEPTH", 1)?,
    ))
}

fn victim_physical_names(band: &CorridorBandContext, lanes: &[String]) -> Vec<String> {
    let mut names = Vec::new();
    for lane in lanes {
        for net in physical_lane_nets(band, lane) {
            if !names.contains(&net) {
                names.push(net);
            }
        }
    }
    names
}

fn run_free_rungs(
    state: &ReplanState,
    metadata: &ProbeBoardMetadata,
    group: &RouteGroup,
    corridor: &CorridorPlan,
    all_contexts: &[CorridorBandContext],
    band: &CorridorBandContext,
    board: BoardAdapter,
    net: &str,
    stage: &str,
    unrouted: &[String],
    landed: &BTreeSet<String>,
    corridor_done: &BTreeSet<usize>,
    deadline: Instant,
    use_approach: bool,
    executor: &mut NativeRungExecutor,
) -> Result<Option<NativeRungRoute>, String> {
    for (margin, use_window) in [(2.5, true), (4.0, true), (6.0, false)] {
        let mut request = free_rung_request(
            state,
            metadata,
            group,
            corridor,
            all_contexts,
            band,
            board.clone(),
            net,
            stage,
            unrouted,
            landed,
            corridor_done,
            deadline,
            margin,
            use_window,
            use_approach,
            false,
        )?;
        match executor.route(request)? {
            braid_connect::RungExecution::Found(route) => return Ok(Some(route)),
            braid_connect::RungExecution::Refused { .. } => {}
        }
    }
    Ok(None)
}

#[allow(clippy::too_many_arguments)]
fn rip_for_lane(
    state: &ReplanState,
    metadata: &ProbeBoardMetadata,
    group: &RouteGroup,
    names: &[String],
    baseline: &BoardAdapter,
    board: &BoardAdapter,
    candidate: &str,
    refused_others: &[String],
    report: &braid_connect::RungReport,
    landed: &BTreeSet<String>,
    corridor_done: &BTreeSet<usize>,
    plans: &[CorridorPlan],
    all_contexts: &[CorridorBandContext],
    deadline: Instant,
    protect: &BTreeSet<String>,
    depth: usize,
    max_victims: usize,
    lanes_from: Option<&[String]>,
    probe_margin: f64,
    soft_cost: Option<f64>,
    bound: Option<(usize, f64)>,
    accept: Option<&dyn Fn(&NativeRungRoute, &BTreeMap<String, NativeRungRoute>) -> bool>,
    bands: &mut SourceRungBandFactory,
    executor: &mut NativeRungExecutor,
) -> Result<Option<RipSolution>, String> {
    let Some(corridor_index) = plans
        .iter()
        .position(|plan| plan.members.iter().any(|member| member == candidate))
    else {
        return Ok(None);
    };
    let corridor = &plans[corridor_index];
    let band = &all_contexts[corridor_index];
    let protected: BTreeSet<_> = protect.iter().cloned().collect();
    let mut named = if let Some(lanes_from) = lanes_from {
        lanes_from
            .iter()
            .filter(|lane| {
                *lane != candidate
                    && !protected.contains(*lane)
                    && landed.contains(*lane)
                    && band.members.contains(*lane)
            })
            .cloned()
            .collect::<Vec<_>>()
    } else {
        frontier_blocker_order(
            state,
            board,
            baseline,
            band,
            group,
            landed,
            &refused_others.iter().cloned().collect(),
            candidate,
            report,
        )?
        .into_iter()
        .filter(|lane| !protected.contains(lane))
        .collect()
    };
    named.dedup();
    if named.is_empty() {
        return Ok(None);
    }

    // Match source rip_for's optimistic probe: remove every attributed lane,
    // keep its real copper as soft cost, and search the refused lane free in
    // the planned window. The returned path determines the cut by geometry.
    let mut probe_board = board.clone();
    let physical_named = victim_physical_names(band, &named);
    let mut soft_tracks = Vec::new();
    let mut soft_vias = Vec::new();
    let mut victim_geometry = Vec::new();
    for lane in &named {
        let (_, delta) = lane_copper(state, board, baseline, band, lane)?;
        soft_tracks.extend(delta.tracks.clone());
        soft_vias.extend(delta.vias.clone());
        victim_geometry.push((lane.clone(), delta));
    }
    native_execution::strip_nets(&mut probe_board, &physical_named.iter().cloned().collect())?;
    native_execution::restore_nets(
        baseline,
        &mut probe_board,
        &physical_named.iter().cloned().collect(),
    )?;
    let mut request = free_rung_request(
        state,
        metadata,
        group,
        corridor,
        all_contexts,
        band,
        probe_board,
        candidate,
        "rip_probe",
        refused_others,
        landed,
        corridor_done,
        deadline,
        probe_margin,
        true,
        lanes_from.is_none(),
        true,
    )?;
    request.spec.margin = probe_margin;
    if let Some(pair) = request.pair.take() {
        // Source rip_for prices one centerline route from the pair alias's P id.
        request.net = pair.positive_net;
        request.net_id = pair.positive_net_id;
    }
    let probe =
        executor.route_min_cut(request, soft_tracks, soft_vias, soft_cost.unwrap_or(5.0))?;
    let Some(probe) = probe else {
        return Ok(None);
    };
    // Keep the router's emitted segment order: source cut selection walks the
    // priced path and takes victims at their first geometric intersection.
    let probe_geometry = native_route_copper(&probe, candidate);
    if let Some((via_bound, length_bound)) = bound {
        let probe_vias = probe_geometry.vias.len();
        let probe_length: f64 = probe_geometry
            .tracks
            .iter()
            .map(|track| (track.a[0] - track.b[0]).hypot(track.a[1] - track.b[1]))
            .sum();
        if !(probe_vias < via_bound
            || (probe_vias == via_bound && probe_length < length_bound - 0.5))
        {
            return Ok(None);
        }
    }
    let cut = source_mincut_order(&probe_geometry, &victim_geometry, &state.probe.rules);
    let mut trials = Vec::<Vec<String>>::new();
    if cut.is_empty() {
        trials.push(Vec::new());
    } else {
        let victims: Vec<_> = cut.iter().take(max_victims).cloned().collect();
        for count in 1..=victims.len() {
            trials.push(victims[..count].to_vec());
        }
    }
    if lanes_from.is_none() {
        for lane in named.iter().take(max_victims) {
            if !trials.iter().any(|trial| trial == &[lane.clone()]) {
                trials.push(vec![lane.clone()]);
            }
        }
    }

    for victims in trials {
        let physical_victims = victim_physical_names(band, &victims);
        let victim_set: BTreeSet<_> = physical_victims.iter().cloned().collect();
        let mut trial_board = board.clone();
        native_execution::strip_nets(&mut trial_board, &victim_set)?;
        native_execution::restore_nets(baseline, &mut trial_board, &victim_set)?;
        let mut trial_landed = landed.clone();
        for victim in &victims {
            trial_landed.remove(victim);
        }
        let mut trial_achieved = BTreeMap::new();
        let mut trial_reports = BTreeMap::new();
        let candidate_rung = if lanes_from.is_some() {
            run_free_rungs(
                state,
                metadata,
                group,
                corridor,
                all_contexts,
                band,
                trial_board.clone(),
                candidate,
                "rip_free",
                refused_others,
                &trial_landed,
                corridor_done,
                deadline,
                false,
                executor,
            )?
        } else {
            run_rung(
                state,
                metadata,
                group,
                corridor,
                all_contexts,
                band,
                trial_board.clone(),
                candidate,
                "rip",
                refused_others,
                &trial_landed,
                corridor_done,
                deadline,
                bands,
                executor,
            )?
            .route
        };
        let Some(candidate_route) = candidate_rung else {
            continue;
        };
        record_route_endpoints(&mut trial_achieved, band, candidate, &candidate_route);
        trial_board = candidate_route.board.clone();
        trial_landed.insert(candidate.to_owned());
        let mut relaid_routes = BTreeMap::new();
        let mut lost = false;
        for victim in &victims {
            let Some(victim_index) = plans.iter().position(|plan| plan.members.contains(victim))
            else {
                lost = true;
                break;
            };
            let victim_band = &all_contexts[victim_index];
            let victim_corridor = &plans[victim_index];
            let (victim_route, victim_report) = if lanes_from.is_some() {
                (
                    run_free_rungs(
                        state,
                        metadata,
                        group,
                        victim_corridor,
                        all_contexts,
                        victim_band,
                        trial_board.clone(),
                        victim,
                        "rip_free",
                        refused_others,
                        &trial_landed,
                        corridor_done,
                        deadline,
                        false,
                        executor,
                    )?,
                    None,
                )
            } else {
                let victim_result = run_rung(
                    state,
                    metadata,
                    group,
                    victim_corridor,
                    all_contexts,
                    victim_band,
                    trial_board.clone(),
                    victim,
                    "rip",
                    refused_others,
                    &trial_landed,
                    corridor_done,
                    deadline,
                    bands,
                    executor,
                )?;
                (victim_result.route, victim_result.report)
            };
            let victim_route = match victim_route {
                Some(route) => route,
                None if depth > 0 => {
                    if lanes_from.is_none() && victim_report.is_none() {
                        lost = true;
                        break;
                    }
                    let mut next_protect = protected.clone();
                    next_protect.insert(candidate.to_owned());
                    let nested = rip_for_lane(
                        state,
                        metadata,
                        group,
                        names,
                        baseline,
                        &trial_board,
                        victim,
                        refused_others,
                        victim_report.as_ref().unwrap_or(report),
                        &trial_landed,
                        corridor_done,
                        plans,
                        all_contexts,
                        deadline,
                        &next_protect,
                        depth - 1,
                        max_victims,
                        lanes_from,
                        probe_margin,
                        soft_cost,
                        bound,
                        accept,
                        bands,
                        executor,
                    )?;
                    let Some(nested) = nested else {
                        lost = true;
                        break;
                    };
                    trial_board = nested.board;
                    trial_landed.extend(nested.routed);
                    trial_achieved.extend(nested.achieved);
                    trial_reports.extend(nested.reports);
                    relaid_routes.extend(nested.routes);
                    continue;
                }
                None => {
                    lost = true;
                    break;
                }
            };
            if lost {
                break;
            }
            if !trial_landed.contains(victim) {
                record_route_endpoints(&mut trial_achieved, victim_band, victim, &victim_route);
                trial_board = victim_route.board.clone();
                trial_landed.insert(victim.clone());
            }
            relaid_routes.insert(victim.clone(), victim_route);
        }
        if lost {
            continue;
        }
        if accept.is_some_and(|accept| !accept(&candidate_route, &relaid_routes)) {
            continue;
        }
        let routed_net_names = victim_physical_names(
            band,
            &std::iter::once(candidate.to_owned())
                .chain(victims.iter().cloned())
                .collect::<Vec<_>>(),
        );
        let routed_set: BTreeSet<_> = routed_net_names.iter().cloned().collect();
        let trial_grade = native_execution::grade(board, &trial_board, &routed_set)?;
        if !trial_grade.open.is_empty() || trial_grade.new_violations != 0 {
            continue;
        }
        let mut routes = BTreeMap::from([(candidate.to_owned(), candidate_route)]);
        routes.extend(relaid_routes);
        let mut routed = vec![candidate.to_owned()];
        routed.extend(victims);
        return Ok(Some(RipSolution {
            board: trial_board,
            routed,
            achieved: trial_achieved,
            reports: trial_reports,
            routes,
        }));
    }
    Ok(None)
}

fn record_route_endpoints(
    achieved: &mut BTreeMap<String, RungEndpoint>,
    band: &CorridorBandContext,
    lane: &str,
    route: &NativeRungRoute,
) {
    if let (Some([positive, negative]), Some([positive_end, negative_end])) =
        (band.pair_members.get(lane), route.achieved_pair.as_ref())
    {
        achieved.insert(positive.clone(), positive_end.clone());
        achieved.insert(negative.clone(), negative_end.clone());
    } else {
        achieved.insert(lane.to_owned(), route.achieved.clone());
    }
}

fn copper_metrics(copper: &LaneCopper) -> (usize, f64) {
    (
        copper.vias.len(),
        copper
            .tracks
            .iter()
            .map(|track| (track.a[0] - track.b[0]).hypot(track.a[1] - track.b[1]))
            .sum(),
    )
}

#[allow(clippy::too_many_arguments)]
fn run_economy_relay(
    state: &ReplanState,
    metadata: &ProbeBoardMetadata,
    group: &RouteGroup,
    names: &[String],
    baseline: &BoardAdapter,
    board: &mut BoardAdapter,
    landed: &mut BTreeSet<String>,
    refused: &[String],
    achieved: &mut BTreeMap<String, RungEndpoint>,
    plans: &[CorridorPlan],
    contexts: &[CorridorBandContext],
    corridor_done: &BTreeSet<usize>,
    deadline: Instant,
    bands: &mut SourceRungBandFactory,
    executor: &mut NativeRungExecutor,
) -> Result<(), String> {
    let float = |key: &str, default: f64| -> Result<f64, String> {
        state
            .environment_overrides
            .get(key)
            .map_or(Ok(default), |value| {
                value.parse::<f64>().map_err(|_| {
                    format!("invalid {key} value in native replan environment: {value:?}")
                })
            })
    };
    let integer = |key: &str, default: i32| -> Result<i32, String> {
        state
            .environment_overrides
            .get(key)
            .map_or(Ok(default), |value| {
                value.parse::<i32>().map_err(|_| {
                    format!("invalid {key} value in native replan environment: {value:?}")
                })
            })
    };
    let long_limit = float("BRAID_ECON_LONG", 3.0)?;
    let mm_per_via = float("BRAID_ECON_MM_PER_VIA", 6.0)?;
    let joint_mode = integer("BRAID_ECON_JOINT", 1)?;
    if !long_limit.is_finite() || !mm_per_via.is_finite() || mm_per_via < 0.0 {
        return Err("invalid non-finite or negative braid economy setting".into());
    }

    let mut heavy = Vec::<(String, usize, f64, bool, usize)>::new();
    for (index, plan) in plans.iter().enumerate() {
        let context = &contexts[index];
        for lane in &plan.members {
            if !landed.contains(lane) {
                continue;
            }
            let (_, copper) = lane_copper(state, board, baseline, context, lane)?;
            let (vias, length) = copper_metrics(&copper);
            let (Some(tooth), Some(berth)) = (context.teeth.get(lane), context.stubs.get(lane))
            else {
                continue;
            };
            let airline = (tooth[0] - berth[0]).hypot(tooth[1] - berth[1]);
            let is_long = length - airline > long_limit;
            if vias >= 3 || (length > 0.0 && is_long) {
                heavy.push((lane.clone(), index, length, is_long, vias));
            }
        }
    }
    heavy.sort_by(|a, b| b.4.cmp(&a.4));

    for (lane, index, old_length, is_long, old_vias) in heavy {
        let context = &contexts[index];
        let corridor = &plans[index];
        let old_physical = victim_physical_names(context, std::slice::from_ref(&lane));
        let old_set: BTreeSet<_> = old_physical.iter().cloned().collect();
        let mut stripped = board.clone();
        native_execution::strip_nets(&mut stripped, &old_set)?;
        native_execution::restore_nets(baseline, &mut stripped, &old_set)?;
        let empty_unrouted = Vec::new();
        let free_route = run_free_rungs(
            state,
            metadata,
            group,
            corridor,
            contexts,
            context,
            stripped,
            &lane,
            "econ",
            &empty_unrouted,
            landed,
            corridor_done,
            deadline,
            false,
            executor,
        )?;

        let mut accepted_single = None;
        let mut too_long = false;
        if let Some(route) = free_route {
            let candidate_copper = native_route_copper(&route, &lane);
            let (new_vias, new_length) = copper_metrics(&candidate_copper);
            if new_vias < old_vias
                && (mm_per_via <= 0.0
                    || new_length <= old_length + mm_per_via * (old_vias - new_vias) as f64)
            {
                accepted_single = Some(route);
            } else if new_vias == old_vias && new_length < old_length - 0.5 {
                accepted_single = Some(route);
            } else if new_vias < old_vias {
                too_long = true;
            }
        }
        if let Some(route) = accepted_single.as_ref() {
            let routed = BTreeSet::from_iter(old_physical.iter().cloned());
            let grade = native_execution::grade(board, &route.board, &routed)?;
            if !grade.open.is_empty() || grade.new_violations != 0 {
                accepted_single = None;
            }
        }

        let single_has_via = accepted_single
            .as_ref()
            .is_some_and(|route| !route.route_vias.is_empty());
        let should_joint = joint_mode != 0
            && (accepted_single.is_none() || joint_mode >= 2 && single_has_via)
            && (joint_mode >= 2 || accepted_single.is_none() && (too_long || is_long));
        if !should_joint {
            if let Some(route) = accepted_single {
                *board = route.board.clone();
                record_route_endpoints(achieved, context, &lane, &route);
            }
            continue;
        }
        let baseline_metrics = accepted_single
            .as_ref()
            .map_or((old_vias, old_length), |route| {
                copper_metrics(&native_route_copper(route, &lane))
            });
        let lanes_here: Vec<_> = corridor
            .members
            .iter()
            .filter(|other| *other != &lane && landed.contains(*other))
            .cloned()
            .collect();
        if lanes_here.is_empty() {
            if let Some(route) = accepted_single {
                *board = route.board.clone();
                record_route_endpoints(achieved, context, &lane, &route);
            }
            continue;
        }
        let old_metrics: BTreeMap<_, _> = lanes_here
            .iter()
            .map(|victim| {
                lane_copper(state, board, baseline, context, victim)
                    .map(|(_, copper)| (victim.clone(), copper_metrics(&copper)))
            })
            .collect::<Result<_, _>>()?;
        let guard = mm_per_via;
        let accept = |candidate: &NativeRungRoute, relaid: &BTreeMap<String, NativeRungRoute>| {
            let mut rows = vec![(
                baseline_metrics.0,
                baseline_metrics.1,
                copper_metrics(&native_route_copper(candidate, &lane)).0,
                copper_metrics(&native_route_copper(candidate, &lane)).1,
            )];
            for (victim, route) in relaid {
                if let Some((old_vias, old_length)) = old_metrics.get(victim) {
                    let (new_vias, new_length) =
                        copper_metrics(&native_route_copper(route, victim));
                    rows.push((*old_vias, *old_length, new_vias, new_length));
                }
            }
            if rows.iter().any(|(old_v, old_l, new_v, new_l)| {
                new_v > old_v
                    || (guard > 0.0
                        && *new_l > *old_l + guard * old_v.saturating_sub(*new_v) as f64 + 1.0)
            }) {
                return false;
            }
            let old_v: usize = rows.iter().map(|row| row.0).sum();
            let new_v: usize = rows.iter().map(|row| row.2).sum();
            let old_l: f64 = rows.iter().map(|row| row.1).sum();
            let new_l: f64 = rows.iter().map(|row| row.3).sum();
            new_v < old_v && (guard <= 0.0 || new_l <= old_l + guard * (old_v - new_v) as f64)
        };
        let report = braid_connect::RungReport {
            blocked_cells: Vec::new(),
            route_layers: group
                .awx
                .as_ref()
                .ok_or("economy probe has no AWX route layers")?
                .routing_layers
                .clone(),
            window: [0.0; 4],
            grid_step_mm: state.probe.rules.grid_step,
            expanded: 0,
            truncated: false,
        };
        let (max_victims, _) = rip_limits(state)?;
        let rip = rip_for_lane(
            state,
            metadata,
            group,
            names,
            baseline,
            board,
            &lane,
            refused,
            &report,
            landed,
            corridor_done,
            plans,
            contexts,
            deadline,
            &BTreeSet::new(),
            0,
            max_victims,
            Some(&lanes_here),
            2.5,
            Some(1.0),
            Some(baseline_metrics),
            Some(&accept),
            bands,
            executor,
        )?;
        if let Some(solution) = rip {
            *board = solution.board;
            landed.extend(solution.routed);
            achieved.extend(solution.achieved);
        } else if let Some(route) = accepted_single {
            *board = route.board.clone();
            record_route_endpoints(achieved, context, &lane, &route);
        }
    }
    Ok(())
}

fn probe_braid(
    state: &ReplanState,
    group: &RouteGroup,
    names: &[String],
    base_board: &BoardAdapter,
    drills: &BTreeMap<String, f64>,
    deadline: Instant,
) -> Result<ProbeBraidOutcome, String> {
    let metadata = probe_board_metadata(state, base_board, drills)?;
    let corridor_states = probe_corridor_states(state, names, &metadata.snapshot)?;
    let plans = corridor_states
        .iter()
        .cloned()
        .map(CorridorState::into_plan)
        .collect::<Result<Vec<_>, _>>()?;
    let all_contexts: Vec<_> = plans.iter().map(|plan| plan.band.clone()).collect();
    let mut bands = SourceRungBandFactory;
    let mut executor = NativeRungExecutor::default();
    let mut best: Option<(
        usize,
        usize,
        usize,
        BoardAdapter,
        Vec<String>,
        BTreeMap<String, RungEndpoint>,
    )> = None;
    let scope: BTreeSet<_> = names.iter().cloned().collect();
    for attempt in 0..state.probe.probe_attempts {
        let mut board = base_board.clone();
        let mut landed = BTreeSet::new();
        let mut refused = Vec::new();
        let mut achieved = BTreeMap::new();
        let mut corridor_done = BTreeSet::new();
        for ((corridor_state, corridor_plan), context) in
            corridor_states.iter().zip(&plans).zip(&all_contexts)
        {
            let mut routed = BTreeSet::new();
            let mut failed_rescues = 0usize;
            for net in corridor_route_order(corridor_state) {
                let unrouted: Vec<_> = corridor_state
                    .members
                    .iter()
                    .filter(|other| **other != net && !routed.contains(*other))
                    .cloned()
                    .collect();
                let first = run_main_lane(
                    state,
                    &metadata,
                    group,
                    corridor_plan,
                    &all_contexts,
                    context,
                    board.clone(),
                    &net,
                    "attempt",
                    &unrouted,
                    &landed,
                    &corridor_done,
                    deadline,
                    200_000,
                    false,
                    &mut bands,
                    &mut executor,
                )?;
                let route = if first.is_none() && failed_rescues < 3 {
                    let rescue_iterations = ((200_000usize.max(50_000) as f64
                        * state.probe.probe_budget_x.max(0.0))
                    .ceil()
                    .min(usize::MAX as f64)) as usize;
                    if rescue_iterations == 0 {
                        return Err("PROBE_BUDGET_X produced a zero rescue budget".into());
                    }
                    let rescue = run_main_lane(
                        state,
                        &metadata,
                        group,
                        corridor_plan,
                        &all_contexts,
                        context,
                        board.clone(),
                        &net,
                        "rescue",
                        &unrouted,
                        &landed,
                        &corridor_done,
                        deadline,
                        rescue_iterations,
                        true,
                        &mut bands,
                        &mut executor,
                    )?;
                    if rescue.is_none() {
                        failed_rescues += 1;
                    }
                    rescue
                } else {
                    first
                };
                match route {
                    Some(route) => {
                        record_route_endpoints(&mut achieved, context, &net, &route);
                        board = route.board;
                        landed.insert(net.clone());
                        routed.insert(net.clone());
                    }
                    None => refused.push(net),
                }
            }
            corridor_done.insert(context.index);
        }
        let grade = native_execution::grade(base_board, &board, &scope)?;
        let base_grade = native_execution::grade(base_board, base_board, &scope)?;
        let added_vias = grade.vias.saturating_sub(base_grade.vias);
        let key = (landed.len(), usize::MAX - added_vias);
        let replace = best
            .as_ref()
            .is_none_or(|(count, via_score, _, _, _, _)| key > (*count, *via_score));
        if replace {
            best = Some((key.0, key.1, attempt, board, refused.clone(), achieved));
        }
        if refused.is_empty() {
            break;
        }
    }
    let Some((_, _, _, mut board, mut refused, mut achieved)) = best else {
        return Err("native braid probe produced no attempt".into());
    };

    let mut landed: BTreeSet<_> = plans
        .iter()
        .flat_map(|plan| plan.members.iter())
        .filter(|net| !refused.contains(*net))
        .cloned()
        .collect();
    let mut corridor_done = BTreeSet::new();
    let mut reports = BTreeMap::new();
    for plan in &plans {
        corridor_done.insert(plan.index);
    }
    for net in refused.clone() {
        let Some(index) = plans.iter().position(|plan| plan.members.contains(&net)) else {
            continue;
        };
        let others: Vec<_> = refused
            .iter()
            .filter(|other| **other != net)
            .cloned()
            .collect();
        let rung = run_rung(
            state,
            &metadata,
            group,
            &plans[index],
            &all_contexts,
            &all_contexts[index],
            board.clone(),
            &net,
            "last_call",
            &others,
            &landed,
            &corridor_done,
            deadline,
            &mut bands,
            &mut executor,
        )?;
        match rung.route {
            Some(route) => {
                record_route_endpoints(&mut achieved, &all_contexts[index], &net, &route);
                board = route.board;
                landed.insert(net.clone());
                refused.retain(|old| old != &net);
            }
            None => {
                if let Some(report) = rung.report {
                    reports.insert(net.clone(), report);
                }
            }
        }
    }
    let (max_victims, rip_depth) = rip_limits(state)?;
    for net in refused.clone() {
        let Some(report) = reports.get(&net).cloned() else {
            continue;
        };
        let others: Vec<_> = refused
            .iter()
            .filter(|other| **other != net)
            .cloned()
            .collect();
        if let Some(solution) = rip_for_lane(
            state,
            &metadata,
            group,
            names,
            base_board,
            &board,
            &net,
            &others,
            &report,
            &landed,
            &corridor_done,
            &plans,
            &all_contexts,
            deadline,
            &BTreeSet::new(),
            rip_depth,
            max_victims,
            None,
            6.0,
            None,
            None,
            None,
            &mut bands,
            &mut executor,
        )? {
            board = solution.board;
            landed.extend(solution.routed);
            achieved.extend(solution.achieved);
            reports.extend(solution.reports);
            refused.retain(|old| old != &net);
            reports.remove(&net);
        }
    }
    run_economy_relay(
        state,
        &metadata,
        group,
        names,
        base_board,
        &mut board,
        &mut landed,
        &refused,
        &mut achieved,
        &plans,
        &all_contexts,
        &corridor_done,
        deadline,
        &mut bands,
        &mut executor,
    )?;
    let refused_physical: Vec<_> = refused
        .iter()
        .flat_map(|net| {
            all_contexts
                .iter()
                .find_map(|context| context.pair_members.get(net).cloned())
                .map(|members| vec![members[0].clone(), members[1].clone()])
                .unwrap_or_else(|| vec![net.clone()])
        })
        .collect();
    Ok(ProbeBraidOutcome {
        board,
        refused: refused_physical,
        achieved,
        reports,
    })
}

fn scope_names(state: &ReplanState, candidate: &ProbeCandidate) -> BTreeSet<String> {
    let mut names = BTreeSet::from([candidate.net.clone()]);
    let mut requested: Vec<String> = candidate
        .comove
        .iter()
        .chain(candidate.extra_relay.iter())
        .cloned()
        .collect();
    for movement in candidate.src.iter().chain(candidate.dst.iter()) {
        requested.extend(lane_conflicts(state, movement, &candidate.net));
    }
    if let Some(destination) = &candidate.dst {
        requested.extend(berth_conflicts(state, destination, &candidate.net));
    }
    if state.probe.coupled == "census" {
        if let Some(verdict) = state.verdicts.get(&candidate.net) {
            requested.extend(verdict.census.iter().cloned());
        }
    }
    for requested_name in requested {
        let exact: Vec<_> = state
            .names
            .iter()
            .filter(|name| **name == requested_name)
            .collect();
        let hits: Vec<_> = if exact.is_empty() {
            state
                .names
                .iter()
                .filter(|name| {
                    escape_moves::short_name(name) == escape_moves::short_name(&requested_name)
                })
                .collect()
        } else {
            exact
        };
        if hits.len() == 1 {
            names.insert(hits[0].clone());
        }
    }
    if let Ok(group) = group_for(state, &candidate.net) {
        for pair in group
            .awx
            .as_ref()
            .into_iter()
            .flat_map(|awx| awx.pairs.iter())
        {
            let partner = if escape_moves::short_name(&pair.p)
                == escape_moves::short_name(&candidate.net)
            {
                Some(&pair.n)
            } else if escape_moves::short_name(&pair.n) == escape_moves::short_name(&candidate.net)
            {
                Some(&pair.p)
            } else {
                None
            };
            if let Some(partner) = partner {
                if let Some(known) = state.names.iter().find(|name| {
                    escape_moves::short_name(name) == escape_moves::short_name(partner)
                }) {
                    names.insert(known.clone());
                }
            }
        }
    }
    names
}

fn fanout_base(state: &ReplanState, names: &BTreeSet<String>) -> Result<BoardAdapter, String> {
    let mut board = state.world.routed.clone();
    native_execution::strip_nets(&mut board, names)?;
    for name in names {
        let source = state.fan_src.get(name).unwrap_or(&state.fanout_seed);
        native_execution::restore_nets(source, &mut board, &BTreeSet::from([name.clone()]))?;
    }
    Ok(board)
}

fn failed_result(
    candidate: ProbeCandidate,
    state: &ReplanState,
    fanout: BoardAdapter,
    error: String,
) -> ProbeResult {
    ProbeResult {
        candidate,
        fanout_after: fanout,
        board: state.world.routed.clone(),
        grade: state.current_grade.clone(),
        achieved_src: None,
        achieved_dst: None,
        comove_got: BTreeMap::new(),
        relaid: vec![],
        refused: false,
        refused_nets: vec![],
        rung_reports: BTreeMap::new(),
        fail: Some(error),
        unjudged: true,
    }
}

fn probe_one(state: &ReplanState, candidate: ProbeCandidate) -> ProbeResult {
    if state.probe.coupled != "census" {
        return failed_result(
            candidate,
            state,
            state.fanout_seed.clone(),
            format!("unsupported native coupled mode {}", state.probe.coupled),
        );
    }
    if state.probe.probe_ladder != "open" && state.probe.probe_ladder != "full" {
        return failed_result(
            candidate,
            state,
            state.fanout_seed.clone(),
            format!("unsupported PROBE_LADDER mode {}", state.probe.probe_ladder),
        );
    }
    let names = scope_names(state, &candidate);
    let group = match group_for(state, &candidate.net) {
        Ok(group) => scoped_group(state, group, &names),
        Err(error) => return failed_result(candidate, state, state.fanout_seed.clone(), error),
    };
    let mut fanout = match fanout_base(state, &names) {
        Ok(board) => board,
        Err(error) => return failed_result(candidate, state, state.fanout_seed.clone(), error),
    };
    let rescue_budget = (state.world.spec.execution.budget_ms as f64
        * state.probe.probe_budget_x.max(0.0))
    .min(u64::MAX as f64) as u64;
    let rescue_deadline = Instant::now()
        .checked_add(Duration::from_millis(rescue_budget))
        .unwrap_or(state.probe.deadline);
    let deadline = rescue_deadline.min(state.probe.deadline);
    let source_menus = &state.probe.source_menus;
    let destination_menus = &state.probe.destination_menus;
    let mut source_plan = state.probe.source_plan.clone();
    let mut destination_plan = state.probe.destination_plan.clone();
    for name in &names {
        if let Some((ball, movement)) =
            exact_current_move(state, source_menus, name, ProbeEnd::Source)
        {
            if let Err(error) = set_plan_move(&mut source_plan, source_menus, &ball, &movement) {
                return failed_result(candidate, state, fanout, error);
            }
        }
        if let Some((ball, movement)) =
            exact_current_move(state, destination_menus, name, ProbeEnd::Destination)
        {
            if let Err(error) =
                set_plan_move(&mut destination_plan, destination_menus, &ball, &movement)
            {
                return failed_result(candidate, state, fanout, error);
            }
        }
    }
    let source_candidate = candidate.src.as_ref().map(|movement| {
        let ball = candidate
            .src_ball
            .as_deref()
            .or((candidate.end == ProbeEnd::Source).then_some(candidate.ball.as_str()))
            .ok_or_else(|| "source candidate has no pad identity".to_owned())?;
        set_plan_move(&mut source_plan, source_menus, ball, movement)
    });
    if let Some(Err(error)) = source_candidate {
        return failed_result(candidate, state, fanout, error);
    }
    let destination_candidate = candidate.dst.as_ref().map(|movement| {
        let ball = candidate
            .dst_ball
            .as_deref()
            .or((candidate.end == ProbeEnd::Destination).then_some(candidate.ball.as_str()))
            .ok_or_else(|| "destination candidate has no pad identity".to_owned())?;
        set_plan_move(&mut destination_plan, destination_menus, ball, movement)
    });
    if let Some(Err(error)) = destination_candidate {
        return failed_result(candidate, state, fanout, error);
    }
    retain_plan_nets(&mut source_plan, &names);
    retain_plan_nets(&mut destination_plan, &names);

    let awx = state
        .world
        .spec
        .awx
        .as_ref()
        .expect("native AWX state validated");
    let mut drills = awx.via_drills.clone();
    let protected = state.world.spec.protected_names();
    let source_result = match source_realize::realize_source(
        &fanout,
        &source_plan,
        &awx.rules,
        &mut drills,
        &protected,
    ) {
        Ok(realized) => realized,
        Err(error) => {
            return failed_result(
                candidate,
                state,
                fanout,
                format!("source realization: {error}"),
            );
        }
    };
    let source_refusals: Vec<_> = source_result
        .nets
        .iter()
        .filter(|decision| names.contains(&decision.net) && !decision.accepted)
        .collect();
    if !source_refusals.is_empty() {
        let failed_nets: Vec<_> = source_refusals
            .iter()
            .map(|decision| decision.net.clone())
            .collect();
        let reasons = source_refusals
            .iter()
            .map(|decision| format!("{}: {}", decision.net, decision.reason))
            .collect::<Vec<_>>()
            .join("; ");
        let mut result = failed_result(
            candidate,
            state,
            source_result.board,
            format!("source: {reasons}"),
        );
        result.refused = true;
        result.refused_nets = failed_nets;
        return result;
    }
    fanout = source_result.board;
    let destination_result = match destination_realize::realize_destination(
        &fanout,
        &destination_plan,
        &awx.rules,
        &mut drills,
        &protected,
    ) {
        Ok(realized) => realized,
        Err(error) => {
            return failed_result(
                candidate,
                state,
                fanout,
                format!("destination realization: {error}"),
            );
        }
    };
    let destination_refusals: Vec<_> = destination_result
        .nets
        .iter()
        .filter(|decision| names.contains(&decision.net) && !decision.accepted)
        .collect();
    if !destination_refusals.is_empty() {
        let failed_nets: Vec<_> = destination_refusals
            .iter()
            .map(|decision| decision.net.clone())
            .collect();
        let reasons = destination_refusals
            .iter()
            .map(|decision| format!("{}: {}", decision.net, decision.reason))
            .collect::<Vec<_>>()
            .join("; ");
        let mut result = failed_result(
            candidate,
            state,
            destination_result.board,
            format!("destination: {reasons}"),
        );
        result.refused = true;
        result.refused_nets = failed_nets;
        return result;
    }
    fanout = destination_result.board;
    let probe_names: Vec<_> = state
        .names
        .iter()
        .filter(|name| names.contains(*name))
        .cloned()
        .collect();
    let braid = match probe_braid(state, &group, &probe_names, &fanout, &drills, deadline) {
        Ok(braid) => braid,
        Err(error) => {
            return failed_result(
                candidate,
                state,
                fanout,
                format!("native braid probe: {error}"),
            );
        }
    };
    let mut landed_board = braid.board;
    if state.probe.probe_smooth {
        let smooth = (|| -> Result<(), String> {
            let awx = state
                .world
                .spec
                .awx
                .as_ref()
                .ok_or("native probe smoothing requires AWX metadata")?;
            let metadata = awx
                .smoothing_metadata
                .as_ref()
                .ok_or("PROBE_SMOOTH=1 requires explicit dense-pad smoothing metadata")?;
            let snapshot = BoardSnapshot::capture(&landed_board, &awx.components, &drills)?;
            let dense_fields = metadata.dense_fields(&snapshot, &awx.rules)?;
            let base_ids: BTreeSet<_> = fanout
                .board
                .get_traces()
                .into_iter()
                .map(|key| fanout.board.item(key).id().0)
                .collect();
            let requested_ids: BTreeSet<_> = names
                .iter()
                .map(|net| landed_board.net_number(net))
                .collect::<Result<_, _>>()?;
            let pair_ids: BTreeSet<_> = group
                .awx
                .as_ref()
                .into_iter()
                .flat_map(|meta| meta.pairs.iter())
                .flat_map(|pair| [pair.p.as_str(), pair.n.as_str()])
                .filter_map(|net| landed_board.net_number(net).ok())
                .collect();
            let lane_items: Vec<_> = landed_board
                .board
                .get_traces()
                .into_iter()
                .filter(|key| {
                    let item = landed_board.board.item(*key);
                    !base_ids.contains(&item.id().0)
                        && item
                            .net_numbers()
                            .iter()
                            .any(|net| requested_ids.contains(net) && !pair_ids.contains(net))
                })
                .collect();
            super::board_smoothing::smooth_board(&mut landed_board, &lane_items, &dense_fields)?;
            Ok(())
        })();
        if let Err(error) = smooth {
            return failed_result(
                candidate,
                state,
                fanout,
                format!("native PROBE_SMOOTH: {error}"),
            );
        }
    }
    let grade = match super::evolution_native::grade_board(&landed_board, &state.world.spec) {
        Ok(grade) => grade,
        Err(error) => {
            return failed_result(
                candidate,
                state,
                fanout,
                format!("whole-board grade: {error}"),
            );
        }
    };
    let refused_nets = braid.refused;
    let mut achieved_dst = candidate.dst.clone();
    let mut comove_got = BTreeMap::new();
    for audit in &destination_result.audits {
        if !audit.exact {
            continue;
        }
        let Some(mut movement) = destination_plan
            .chosen
            .get(&audit.ball)
            .and_then(movement_from_option)
            .cloned()
        else {
            continue;
        };
        movement.direction = audit.achieved_direction.unwrap_or(movement.direction);
        movement.layer = audit.achieved_layer.clone().unwrap_or(movement.layer);
        if let Some(point) = audit.achieved_tooth {
            movement.exit_pt = point;
        }
        movement.vias = audit.vias as u32;
        if escape_moves::short_name(&movement.net) == escape_moves::short_name(&candidate.net) {
            achieved_dst = Some(movement);
        } else {
            comove_got.insert(movement.net.clone(), movement);
        }
    }
    let mut achieved_src = candidate.src.clone();
    if let Some(movement) = achieved_dst.as_mut() {
        if let Some(endpoint) = braid.achieved.get(&movement.net) {
            movement.exit_pt = endpoint.point;
            movement.layer.clone_from(&endpoint.layer);
        }
    }
    for movement in comove_got.values_mut() {
        if let Some(endpoint) = braid.achieved.get(&movement.net) {
            movement.exit_pt = endpoint.point;
            movement.layer.clone_from(&endpoint.layer);
        }
    }
    let candidate_net = candidate.net.clone();
    let unjudged = !refused_nets.is_empty();
    ProbeResult {
        achieved_src,
        achieved_dst,
        comove_got,
        candidate,
        fanout_after: fanout,
        board: landed_board,
        grade,
        relaid: names
            .iter()
            .filter(|name| **name != candidate_net)
            .cloned()
            .collect(),
        refused: !refused_nets.is_empty(),
        refused_nets,
        rung_reports: braid.reports,
        fail: None,
        unjudged,
    }
}

fn cache_key(state: &ReplanState, candidate: &ProbeCandidate) -> Result<String, String> {
    let copper = super::evolution_native::copper_fingerprint(&state.world.routed)?;
    let fanout = super::evolution_native::copper_fingerprint(&state.fanout_seed)?;
    let mut lineage = BTreeMap::new();
    for name in scope_names(state, candidate) {
        if let Some(board) = state.fan_src.get(&name) {
            lineage.insert(name, super::evolution_native::copper_fingerprint(board)?);
        }
    }
    let mut move_identity = candidate.clone();
    move_identity.tag.clear();
    Ok(format!(
        "{}:{}:{}:{}:{}:{}:{}:{}:{}:{}:{}",
        state.world.seed_stem.display(),
        state.world.requested_k,
        copper,
        fanout,
        json_key(&state.world.spec)?,
        json_key(&lineage)?,
        json_key(&state.ends)?,
        json_key(&state.probe.source_plan)?,
        json_key(&state.probe.destination_plan)?,
        json_key(&(
            state.probe.probe_attempts,
            state.probe.probe_budget_x,
            &state.probe.probe_ladder,
            state.probe.probe_smooth,
            &state.probe.coupled,
            &state.probe.grade_mode,
            state.probe.exact_pages,
            state.probe.swimmer_page_price,
            state.probe.swim_vias,
        ))?,
        json_key(&move_identity)?
    ))
}

/// Probe candidates in bounded parallel batches; cached results retain their
/// original seed, routed-copper, fanout-copper, and complete move identity.
/// Results always follow input order. Individual operation failures are kept
/// as refused/unjudged result rows so later candidates still run.
pub fn probe_many(
    state: &ReplanState,
    candidates: &[ProbeCandidate],
    jobs: usize,
    cache: &mut ProbeCache,
) -> Result<Vec<ProbeResult>, String> {
    if jobs == 0 {
        return Err("replan probe jobs must be positive".into());
    }
    let mut out = vec![None; candidates.len()];
    let mut missing = Vec::new();
    for (index, candidate) in candidates.iter().enumerate() {
        let key = cache_key(state, candidate)?;
        if let Some(result) = cache.rows.get(&key) {
            out[index] = Some(result.clone());
        } else {
            missing.push((index, key, candidate.clone()));
        }
    }
    for chunk in missing.chunks(jobs) {
        std::thread::scope(|scope| {
            let handles: Vec<_> = chunk
                .iter()
                .map(|(index, key, candidate)| {
                    scope.spawn(move || (*index, key.clone(), probe_one(state, candidate.clone())))
                })
                .collect();
            for handle in handles {
                if let Ok((index, key, result)) = handle.join() {
                    cache.rows.insert(key, result.clone());
                    out[index] = Some(result);
                }
            }
        });
    }
    Ok(out
        .into_iter()
        .enumerate()
        .map(|(index, result)| {
            result.unwrap_or_else(|| {
                failed_result(
                    candidates[index].clone(),
                    state,
                    state.fanout_seed.clone(),
                    "native probe worker panicked".into(),
                )
            })
        })
        .collect())
}

/// Single-candidate form used by sequential descent and by callers that need
/// to inspect one physical landing before deciding whether to commit it.
pub fn probe_run(
    state: &ReplanState,
    candidate: ProbeCandidate,
    cache: &mut ProbeCache,
) -> Result<ProbeResult, String> {
    probe_many(state, &[candidate], 1, cache).map(|mut results| results.remove(0))
}
