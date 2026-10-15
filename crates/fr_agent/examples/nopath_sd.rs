use fr_agent::{
    awx::{
        board_snapshot::BoardSnapshot,
        braid_connect::{
            rung_specs, LadderMode, RungEndpoint, RungExecution, RungExecutor, RungRequest,
        },
        fab_rules::{EscalationPolicy, FabTier},
        native_obstacles::build_obstacle_map,
        native_rung::{route_multilayer_endpoint_sets, NativeRungExecutor},
        whole_context::RoutingRules,
    },
    board_adapter::BoardAdapter,
};
use fr_engine::{
    board::{ItemKey, ItemKind},
    drc::NetIncompletes,
};
use fr_geom::{FloatPoint, IntBox};
use serde::Serialize;
use std::{collections::BTreeMap, env, fs, path::PathBuf};

const NET: &str = "SD_MISO";
const TARGET_PIN: &str = "u1-6f57a0a0b03da9fe131";
const GRID_STEP_MM: f64 = 0.25;
const VIA_DIAMETER_MM: f64 = 0.6096;
const VIA_DRILL_MM: f64 = 0.3;

#[derive(Serialize)]
struct ProbeOut {
    snapshot: String,
    net: String,
    target_identity: String,
    target_pin: ItemOut,
    source_endpoint_mm: [f64; 2],
    target_endpoint_mm: [f64; 2],
    source_grid: [i32; 2],
    target_grid: [i32; 2],
    route_layers: Vec<String>,
    grid_step_mm: f64,
    track_width_mm: f64,
    clearance_mm: f64,
    via_diameter_mm: f64,
    via_drill_mm: f64,
    endpoint_group_pins: Vec<String>,
    same_entry_endpoint_sets: serde_json::Value,
    direct_endpoint_control: RouteOut,
    target_neighborhood: Vec<CellOut>,
    nearby_foreign_copper: Vec<NearbyOut>,
}

#[derive(Serialize)]
struct ItemOut {
    item_id_run_local: i32,
    component: String,
    pin: String,
    center_mm: [f64; 2],
    copper: Vec<ShapeOut>,
}

#[derive(Serialize)]
struct ShapeOut {
    layer: String,
    kind: String,
    width_mm: f64,
    height_mm: f64,
}

#[derive(Serialize)]
struct RouteOut {
    accepted: bool,
    reason: Option<String>,
    expanded: usize,
    frontier_count: usize,
    frontier_near_target: Vec<[i32; 3]>,
    window_mm: Option<[f64; 4]>,
}

#[derive(Serialize)]
struct CellOut {
    grid: [i32; 2],
    layer: String,
    blocked_by_foreign_base_map: bool,
    native_endpoint_cell: bool,
    blocked_after_exact_endpoint_override: bool,
    isolated_blocking_nets: Vec<String>,
    via_blocked_by_foreign_base_map: bool,
    isolated_via_blocking_nets: Vec<String>,
}

#[derive(Serialize)]
struct NearbyOut {
    grid: [i32; 2],
    layer: String,
    blocker_item_id_run_local: i32,
    kind: String,
    component: Option<String>,
    pin: Option<String>,
    net: String,
    native_shape_distance_mm: f64,
}

fn pin_identity(board: &BoardAdapter, key: ItemKey) -> Option<(String, String)> {
    let item = board.board.item(key);
    let ItemKind::Pin(pin) = &item.kind else {
        return None;
    };
    let component = board.board.components.get(item.component_no());
    let package = board.board.library.packages.get(component.get_package());
    let pad = package.get_pin(pin.pin_index)?.name.clone();
    Some((component.name.clone(), pad))
}

fn identity_text(board: &BoardAdapter, key: ItemKey) -> Option<String> {
    let (component, pin) = pin_identity(board, key)?;
    Some(format!("{component}-{pin}"))
}

fn item_out(board: &BoardAdapter, key: ItemKey) -> ItemOut {
    let item = board.board.item(key);
    let (component, pin) = pin_identity(board, key).unwrap_or_default();
    let center = item.center(&board.board).to_float();
    let copper = (0..board.board.layer_count() as i32)
        .filter_map(|layer| {
            item.drill_shape_on_layer(&board.board, layer).map(|shape| {
                let bounds = shape.bounding_box();
                ShapeOut {
                    layer: board.board.layer_structure.layers[layer as usize]
                        .name
                        .clone(),
                    kind: format!("{shape:?}"),
                    width_mm: (bounds.ur.x - bounds.ll.x) as f64 / board.units_per_mm,
                    height_mm: (bounds.ur.y - bounds.ll.y) as f64 / board.units_per_mm,
                }
            })
        })
        .collect();
    ItemOut {
        item_id_run_local: item.id().0,
        component,
        pin,
        center_mm: [center.x / board.units_per_mm, center.y / board.units_per_mm],
        copper,
    }
}

fn point_mm(board: &BoardAdapter, point: FloatPoint) -> [f64; 2] {
    [point.x / board.units_per_mm, point.y / board.units_per_mm]
}

fn route_layers(board: &BoardAdapter, net_id: fr_engine::ids::NetNo) -> Vec<String> {
    let class = board.board.rules.net_class_of(net_id);
    board
        .board
        .layer_structure
        .layers
        .iter()
        .enumerate()
        .filter(|(i, layer)| layer.is_signal && class.is_active_routing_layer(*i as i32))
        .map(|(_, layer)| layer.name.clone())
        .collect()
}

fn request(
    board: &BoardAdapter,
    net_id: fr_engine::ids::NetNo,
    net: &str,
    start: [f64; 2],
    end: [f64; 2],
    layers: &[String],
) -> Result<RungRequest, String> {
    let first_layer = board.board.layer_structure.get_no(&layers[0]);
    let class = board.board.rules.net_class_of(net_id);
    let track_width_mm = 2.0 * class.get_trace_half_width(first_layer) as f64 / board.units_per_mm;
    let clearance_mm = board.board.clearance_value(
        class.get_trace_clearance_class(),
        class.get_trace_clearance_class(),
        first_layer,
    ) as f64
        / board.units_per_mm;
    let mut rules = RoutingRules {
        grid_step: GRID_STEP_MM,
        track_width: track_width_mm,
        clearance: clearance_mm,
        via_size: VIA_DIAMETER_MM,
        via_drill: VIA_DRILL_MM,
        pair_gap: 0.0,
        lane_pitch: 1.0,
        lane_min: 0.25,
        min_turning_radius: 0.0,
        max_turn_angle: 180.0,
        max_setback_angle: 45.0,
        routing_clearance_margin: 1.0,
        board_edge_clearance: 0.0,
        hole_to_hole_clearance: 0.0,
        fan_clearance: 0.0,
        geometry_pair_room: false,
        geometry_pair_turn_room_steps: None,
        same_net_pad_clearance: 0.0,
        net_via_sizes: BTreeMap::new(),
    };
    rules
        .net_via_sizes
        .insert(net_id, [VIA_DIAMETER_MM, VIA_DRILL_MM]);
    Ok(RungRequest {
        board: board.clone(),
        net: net.to_owned(),
        net_id,
        stage: "sd_miso_singleton_probe".into(),
        start: RungEndpoint {
            point: start,
            layer: layers[0].clone(),
        },
        end: RungEndpoint {
            point: end,
            layer: layers[0].clone(),
        },
        b_alternatives: Vec::new(),
        pair: None,
        spec: rung_specs(LadderMode::Open)[2],
        band: None,
        band_slack: 0.0,
        window_points: None,
        virtual_tracks: Vec::new(),
        virtual_vias: Vec::new(),
        free_via_pads: Vec::new(),
        same_net_pads: Vec::new(),
        existing_vias: BoardSnapshot::capture_vias(board, &BTreeMap::new())?,
        paste_pads: BTreeMap::new(),
        rules,
        fabrication: FabTier::Standard,
        escalation: EscalationPolicy::Off,
        fabrication_overrides: BTreeMap::new(),
        fab_clearance_floor_mm: 0.0,
        fab_track_floor_mm: 0.0,
        may_narrow: false,
        fab_via_rungs: Vec::new(),
        max_iterations: 200_000,
        route_layers: layers.to_vec(),
        deadline: None,
    })
}

fn run_route(
    board: &BoardAdapter,
    net_id: fr_engine::ids::NetNo,
    from: [f64; 2],
    to: [f64; 2],
    layers: &[String],
) -> Result<(RouteOut, Option<[f64; 4]>), String> {
    let req = request(board, net_id, NET, from, to, layers)?;
    req.rules.validate()?;
    match NativeRungExecutor::default().route(req)? {
        RungExecution::Found(route) => Ok((
            RouteOut {
                accepted: true,
                reason: None,
                expanded: route.grid_path.len(),
                frontier_count: 0,
                frontier_near_target: Vec::new(),
                window_mm: None,
            },
            None,
        )),
        RungExecution::Refused { reason, report } => {
            let Some(report) = report else {
                return Ok((
                    RouteOut {
                        accepted: false,
                        reason,
                        expanded: 0,
                        frontier_count: 0,
                        frontier_near_target: Vec::new(),
                        window_mm: None,
                    },
                    None,
                ));
            };
            let target_grid = [
                (to[0] / report.grid_step_mm).round_ties_even() as i32,
                (to[1] / report.grid_step_mm).round_ties_even() as i32,
            ];
            let mut near = report.blocked_cells.iter().copied().collect::<Vec<_>>();
            near.sort_by_key(|&(x, y, layer)| {
                (
                    (x - target_grid[0]).pow(2) + (y - target_grid[1]).pow(2),
                    layer as i32,
                )
            });
            near.truncate(48);
            let win = report.window;
            Ok((
                RouteOut {
                    accepted: false,
                    reason,
                    expanded: report.expanded,
                    frontier_count: report.blocked_cells.len(),
                    frontier_near_target: near
                        .into_iter()
                        .map(|(x, y, l)| [x, y, l as i32])
                        .collect(),
                    window_mm: Some(win),
                },
                Some(win),
            ))
        }
    }
}

fn group_candidates(
    board: &BoardAdapter,
    group: &fr_engine::board::ItemSet,
    layers: &[String],
) -> Vec<RungEndpoint> {
    let mut candidates = Vec::new();
    for key in group.iter() {
        let item = board.board.item(key);
        let points = match &item.kind {
            ItemKind::Pin(_) | ItemKind::Via(_) => vec![item.center(&board.board)],
            ItemKind::Trace(_) => vec![item.first_corner(), item.last_corner()],
            ItemKind::ConductionArea(_) => board.board.ratsnest_corners(key),
            _ => Vec::new(),
        };
        for point in points {
            let mm = point_mm(board, point.to_float());
            for layer in layers {
                let layer_no = board.board.layer_structure.get_no(layer);
                if layer_no >= 0 && item.is_on_layer(&board.board, layer_no) {
                    candidates.push(RungEndpoint {
                        point: mm,
                        layer: layer.clone(),
                    });
                }
            }
        }
    }
    candidates.sort_by(|a, b| {
        a.point[0]
            .total_cmp(&b.point[0])
            .then_with(|| a.point[1].total_cmp(&b.point[1]))
            .then_with(|| a.layer.cmp(&b.layer))
    });
    candidates.dedup_by(|a, b| a.point == b.point && a.layer == b.layer);
    candidates
}

fn main_probe(
    dsn: PathBuf,
    ses: PathBuf,
    output: PathBuf,
) -> Result<(), Box<dyn std::error::Error>> {
    let dsn_bytes = fs::read(&dsn)?;
    let ses_bytes = fs::read(&ses)?;
    let via_candidates = {
        let source = BoardAdapter::load(&dsn_bytes).map_err(std::io::Error::other)?;
        source
            .board
            .rules
            .nets
            .iter()
            .map(|n| (n.name.clone(), VIA_DIAMETER_MM, VIA_DRILL_MM))
            .collect::<Vec<_>>()
    };
    let board =
        BoardAdapter::load_session_with_via_candidates(&dsn_bytes, &ses_bytes, &via_candidates)
            .map_err(std::io::Error::other)?;
    let net_id = board.net_number(NET)?;
    let target_key = board
        .board
        .get_items()
        .into_iter()
        .find(|&key| identity_text(&board, key).as_deref() == Some(TARGET_PIN))
        .ok_or_else(|| format!("target pin identity {TARGET_PIN} not present"))?;
    let target = point_mm(
        &board,
        board.board.item(target_key).center(&board.board).to_float(),
    );
    let groups = board.board.get_connected_sets(net_id);
    let target_group = groups
        .iter()
        .find(|g| g.contains(board.board.item(target_key).id()))
        .ok_or("target pin has no connected group")?;
    let mut group_pins = target_group
        .iter()
        .filter_map(|k| identity_text(&board, k))
        .collect::<Vec<_>>();
    group_pins.sort();
    group_pins.dedup();
    let gaps = NetIncompletes::new(
        &board.board,
        net_id,
        &board.board.get_connectable_items(net_id),
    );
    let edge = gaps
        .incompletes
        .iter()
        .find(|edge| {
            let from_group = groups
                .iter()
                .find(|g| g.contains(board.board.item(edge.from_item).id()));
            let to_group = groups
                .iter()
                .find(|g| g.contains(board.board.item(edge.to_item).id()));
            (from_group.is_some_and(|g| g.contains(board.board.item(target_key).id())))
                ^ (to_group.is_some_and(|g| g.contains(board.board.item(target_key).id())))
        })
        .ok_or("no residual SD_MISO airline connects to target pad identity")?;
    let (from_key, to_key, from_point, to_point) = if groups
        .iter()
        .find(|g| g.contains(board.board.item(edge.to_item).id()))
        .is_some_and(|g| g.contains(board.board.item(target_key).id()))
    {
        (
            edge.from_item,
            edge.to_item,
            edge.from_corner,
            edge.to_corner,
        )
    } else {
        (
            edge.to_item,
            edge.from_item,
            edge.to_corner,
            edge.from_corner,
        )
    };
    if to_key != target_key {
        return Err("selected airline target is not the stable target pin".into());
    }
    let from_mm = point_mm(&board, from_point);
    let to_mm = point_mm(&board, to_point);
    let layers = route_layers(&board, net_id);
    if layers.is_empty() {
        return Err("SD_MISO has no active signal routing layers".into());
    }
    let from_group = groups
        .iter()
        .find(|g| g.contains(board.board.item(from_key).id()))
        .ok_or("source airline item has no connected group")?;
    let starts = group_candidates(&board, from_group, &layers);
    let targets = group_candidates(&board, target_group, &layers);
    if starts.is_empty() || targets.is_empty() {
        return Err("native endpoint group has no physical active-layer candidates".into());
    }
    let endpoint_request = request(
        &board,
        net_id,
        NET,
        starts[0].point,
        targets[0].point,
        &layers,
    )?;
    let (endpoint_board, endpoint_report) =
        route_multilayer_endpoint_sets(endpoint_request, from_key, to_key, &starts, &targets)
            .map_err(std::io::Error::other)?;
    let same_entry_endpoint_sets = serde_json::json!({
        "report": endpoint_report,
        "route_board_returned": endpoint_board.is_some(),
        "source_candidate_count": starts.len(),
        "target_candidate_count": targets.len(),
        "source_candidates": starts.iter().map(|e|serde_json::json!({"point_mm":e.point,"layer":e.layer})).collect::<Vec<_>>(),
        "target_candidates": targets.iter().map(|e|serde_json::json!({"point_mm":e.point,"layer":e.layer})).collect::<Vec<_>>(),
    });
    let (direct_endpoint_control, win) =
        run_route(&board, net_id, from_mm, to_mm, &layers).map_err(std::io::Error::other)?;

    let width_mm = 2.0
        * board
            .board
            .rules
            .net_class_of(net_id)
            .get_trace_half_width(board.board.layer_structure.get_no(&layers[0])) as f64
        / board.units_per_mm;
    let clearance_mm = request(&board, net_id, NET, from_mm, to_mm, &layers)?
        .rules
        .clearance;
    let grid_step_board = (GRID_STEP_MM * board.units_per_mm).round() as i32;
    let window = win.unwrap_or([
        to_mm[0] - 3.0,
        to_mm[1] - 3.0,
        to_mm[0] + 3.0,
        to_mm[1] + 3.0,
    ]);
    let margin = GRID_STEP_MM * 2.0;
    let to_board = |mm: f64| (mm * board.units_per_mm).round() as i32;
    let native_window = IntBox::new(
        to_board(window[0] - margin),
        to_board(window[1] - margin),
        to_board(window[2] + margin),
        to_board(window[3] + margin),
    );
    let probe_request = request(&board, net_id, NET, from_mm, to_mm, &layers)?;
    let class = board.board.rules.net_class_of(net_id);
    let map = build_obstacle_map(
        &board,
        &probe_request,
        native_window,
        grid_step_board,
        &layers,
        &[net_id],
        class.get_trace_clearance_class() as i32,
        class.get_trace_clearance_class() as i32,
    )
    .map_err(std::io::Error::other)?;
    let map_layers = map
        .layer_numbers
        .iter()
        .enumerate()
        .map(|(idx, layer_no)| {
            (
                idx,
                board.board.layer_structure.layers[*layer_no as usize]
                    .name
                    .clone(),
            )
        })
        .collect::<Vec<_>>();
    let step_mm = map.config.grid_step as f64 / board.units_per_mm;
    let grid = |p: [f64; 2]| {
        [
            (p[0] / step_mm).round_ties_even() as i32,
            (p[1] / step_mm).round_ties_even() as i32,
        ]
    };
    let tgrid = grid(target);
    let mut cells = Vec::new();
    let mut nearest = Vec::new();
    let mut isolated_maps =
        BTreeMap::<fr_engine::ids::NetNo, awx_grid::obstacle_map::GridObstacleMap>::new();
    for dy in -1..=1 {
        for dx in -1..=1 {
            let (gx, gy) = (tgrid[0] + dx, tgrid[1] + dy);
            for (layer_idx, layer) in &map_layers {
                let blocked = map.obstacles.is_blocked(gx, gy, *layer_idx);
                let via_blocked = map.obstacles.is_via_blocked(gx, gy);
                let endpoint_cell = dx == 0 && dy == 0 && *layer == layers[0];
                if !blocked && !via_blocked {
                    cells.push(CellOut {
                        grid: [gx, gy],
                        layer: layer.clone(),
                        blocked_by_foreign_base_map: false,
                        native_endpoint_cell: endpoint_cell,
                        blocked_after_exact_endpoint_override: false,
                        isolated_blocking_nets: Vec::new(),
                        via_blocked_by_foreign_base_map: false,
                        isolated_via_blocking_nets: Vec::new(),
                    });
                    continue;
                }
                let point = fr_geom::FloatPoint::new(
                    gx as f64 * map.config.grid_step as f64,
                    gy as f64 * map.config.grid_step as f64,
                );
                let mut dists = Vec::new();
                let mut candidate_nets = std::collections::BTreeSet::new();
                for key in board.board.get_items() {
                    let item = board.board.item(key);
                    if !item.is_on_board() || item.net_numbers().contains(&net_id) {
                        continue;
                    }
                    let board_layer = map.layer_numbers[*layer_idx];
                    let d = item
                        .drill_shape_on_layer(&board.board, board_layer)
                        .map(|s| s.distance(&point) / board.units_per_mm)
                        .or_else(|| {
                            item.as_trace()
                                .filter(|t| t.layer() == board_layer)
                                .map(|t| t.polyline().distance(&point) / board.units_per_mm)
                        });
                    if let Some(distance) = d.filter(|v| v.is_finite()) {
                        let (component, pin) = pin_identity(&board, key)
                            .map(|(c, p)| (Some(c), Some(p)))
                            .unwrap_or((None, None));
                        let net = item
                            .net_numbers()
                            .first()
                            .and_then(|n| board.board.rules.nets.get(*n).map(|r| r.name.clone()))
                            .unwrap_or_default();
                        dists.push(NearbyOut {
                            grid: [gx, gy],
                            layer: layer.clone(),
                            blocker_item_id_run_local: item.id().0,
                            kind: format!("{:?}", item.board_item_type()),
                            component,
                            pin,
                            net,
                            native_shape_distance_mm: distance,
                        });
                        if distance <= VIA_DIAMETER_MM / 2.0 + clearance_mm + 0.25 {
                            if let Some(&foreign_net) = item.net_numbers().first() {
                                candidate_nets.insert(foreign_net);
                            }
                        }
                    }
                }
                let mut isolated = Vec::new();
                for &foreign_net in &candidate_nets {
                    if !isolated_maps.contains_key(&foreign_net) {
                        let excluded = board
                            .board
                            .rules
                            .nets
                            .iter()
                            .map(|n| n.net_number)
                            .filter(|n| *n != net_id && *n != foreign_net)
                            .collect::<Vec<_>>();
                        let single_net = build_obstacle_map(
                            &board,
                            &probe_request,
                            native_window,
                            grid_step_board,
                            &layers,
                            &excluded,
                            class.get_trace_clearance_class() as i32,
                            class.get_trace_clearance_class() as i32,
                        )
                        .map_err(std::io::Error::other)?;
                        isolated_maps.insert(foreign_net, single_net.obstacles);
                    }
                    if isolated_maps
                        .get(&foreign_net)
                        .is_some_and(|m| m.is_blocked(gx, gy, *layer_idx))
                    {
                        if let Some(rule) = board
                            .board
                            .rules
                            .nets
                            .iter()
                            .find(|rule| rule.net_number == foreign_net)
                        {
                            isolated.push(rule.name.clone());
                        }
                    }
                }
                let mut via_isolated = Vec::new();
                for foreign_net in &candidate_nets {
                    if isolated_maps
                        .get(foreign_net)
                        .is_some_and(|m| m.is_via_blocked(gx, gy))
                    {
                        if let Some(rule) = board
                            .board
                            .rules
                            .nets
                            .iter()
                            .find(|rule| rule.net_number == *foreign_net)
                        {
                            via_isolated.push(rule.name.clone());
                        }
                    }
                }
                cells.push(CellOut {
                    grid: [gx, gy],
                    layer: layer.clone(),
                    blocked_by_foreign_base_map: blocked,
                    native_endpoint_cell: endpoint_cell,
                    blocked_after_exact_endpoint_override: blocked && !endpoint_cell,
                    isolated_blocking_nets: isolated,
                    via_blocked_by_foreign_base_map: via_blocked,
                    isolated_via_blocking_nets: via_isolated,
                });
                dists.sort_by(|a, b| {
                    a.native_shape_distance_mm
                        .total_cmp(&b.native_shape_distance_mm)
                });
                nearest.extend(dists.into_iter().take(1));
            }
        }
    }
    nearest.sort_by(|a, b| {
        a.native_shape_distance_mm
            .total_cmp(&b.native_shape_distance_mm)
    });
    nearest.truncate(32);
    let out = ProbeOut {
        snapshot: ses.display().to_string(),
        net: NET.into(),
        target_identity: TARGET_PIN.into(),
        target_pin: item_out(&board, target_key),
        source_endpoint_mm: from_mm,
        target_endpoint_mm: to_mm,
        source_grid: grid(from_mm),
        target_grid: tgrid,
        route_layers: layers,
        grid_step_mm: step_mm,
        track_width_mm: width_mm,
        clearance_mm,
        via_diameter_mm: VIA_DIAMETER_MM,
        via_drill_mm: VIA_DRILL_MM,
        endpoint_group_pins: group_pins,
        same_entry_endpoint_sets,
        direct_endpoint_control,
        target_neighborhood: cells,
        nearby_foreign_copper: nearest,
    };
    if let Some(parent) = output.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::write(&output, serde_json::to_vec_pretty(&out)?)?;
    println!("{}", output.display());
    Ok(())
}

fn main() {
    let mut args = env::args_os().skip(1);
    let dsn = PathBuf::from(args.next().unwrap_or_default());
    let ses = PathBuf::from(args.next().unwrap_or_default());
    let out = PathBuf::from(args.next().unwrap_or_default());
    if dsn.as_os_str().is_empty() || ses.as_os_str().is_empty() || out.as_os_str().is_empty() {
        eprintln!("usage: nopath_sd <board.dsn> <final28.ses> <output.json>");
        std::process::exit(2);
    }
    if let Err(e) = main_probe(dsn, ses, out) {
        eprintln!("nopath_sd: {e}");
        std::process::exit(1);
    }
}
