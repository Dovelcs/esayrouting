use fr_agent::{
    awx::{
        braid_connect::{
            rung_specs, LadderMode, RungEndpoint, RungExecution, RungExecutor, RungRequest,
        },
        fab_rules::{EscalationPolicy, FabTier},
        native_rung::{route_multilayer_endpoint_sets, NativeRungExecutor},
        whole_context::RoutingRules,
    },
    board_adapter::BoardAdapter,
};
use fr_engine::{
    board::{ItemKey, ItemKind},
    drc::NetIncompletes,
};
use fr_geom::{FloatPoint, IntBox, Point as EnginePoint};
use serde::Serialize;
use std::{collections::BTreeMap, env, fs, path::PathBuf};

// Read-only include so this standalone example can call the crate-private exact
// pad raster implementation without widening production API visibility.
#[path = "../src/awx/native_pad_raster.rs"]
mod pad_raster_source_diagnostic;

#[derive(Serialize)]
struct ItemOut {
    id_run_local: i32,
    kind: String,
    reference: Option<String>,
    pad: Option<String>,
    net: String,
    center_mm: [f64; 2],
    copper_shapes: Vec<ShapeOut>,
    clearance_class: i32,
}

#[derive(Serialize)]
struct ShapeOut {
    layer: String,
    shape_kind: String,
    width_mm: f64,
    height_mm: f64,
}

#[derive(Serialize)]
struct ProbeOut {
    gap: usize,
    net: String,
    from: ItemOut,
    to: ItemOut,
    route_layers: Vec<String>,
    grid_step_mm: f64,
    clearance_mm: f64,
    track_width_mm: f64,
    via_diameter_mm: f64,
    via_drill_mm: f64,
    endpoints_grid: [[i32; 2]; 2],
    native_router: serde_json::Value,
    frontier_cells: usize,
    frontier_near_endpoints: Vec<FrontierCellOut>,
    native_map_endpoint_neighborhoods: Vec<MapNeighborhoodOut>,
    native_map_frontier_readback: Vec<MapCellOut>,
    closest_foreign_items: Vec<NearestItemOut>,
}

#[derive(Serialize)]
struct FrontierCellOut {
    grid: [i32; 2],
    layer: String,
    point_mm: [f64; 2],
    endpoint_distance_cells: f64,
}

#[derive(Serialize)]
struct NearestItemOut {
    frontier_grid: [i32; 2],
    frontier_layer: String,
    item_id_run_local: i32,
    kind: String,
    reference: Option<String>,
    pad: Option<String>,
    net: String,
    copper_shape_distance_mm: f64,
}

#[derive(Serialize)]
struct MapNeighborhoodOut {
    endpoint: String,
    center_grid: [i32; 2],
    layer: String,
    open_same_layer_neighbors: usize,
    cells: Vec<MapCellOut>,
}

#[derive(Serialize)]
struct MapCellOut {
    grid: [i32; 2],
    point_mm: [f64; 2],
    blocked_in_native_base_map: bool,
    foreign_pad_contributors: Vec<MapBlockerOut>,
}

#[derive(Serialize)]
struct MapBlockerOut {
    item_id_run_local: i32,
    reference: Option<String>,
    pad: Option<String>,
    net: String,
    copper_margin_mm: f64,
    native_raster_cell_match: bool,
}

fn item_ref(board: &BoardAdapter, key: ItemKey) -> (Option<String>, Option<String>) {
    let item = board.board.item(key);
    if let ItemKind::Pin(pin) = &item.kind {
        let component = board.board.components.get(item.component_no());
        let package = board.board.library.packages.get(component.get_package());
        let pad = package.get_pin(pin.pin_index).map(|p| p.name.clone());
        return (Some(component.name.clone()), pad);
    }
    (None, None)
}

fn item_out(board: &BoardAdapter, key: ItemKey) -> ItemOut {
    let item = board.board.item(key);
    let (reference, pad) = item_ref(board, key);
    let net = item
        .net_numbers()
        .first()
        .and_then(|n| board.board.rules.nets.get(*n).map(|r| r.name.clone()))
        .unwrap_or_default();
    let center = item.center(&board.board).to_float();
    let copper_shapes = if item.is_pin() || item.is_via() {
        item.padstack(&board.board)
            .map(|ps| {
                (0..board.board.layer_count() as i32)
                    .filter_map(|layer| {
                        ps.get_shape(layer).map(|shape| {
                            let bounds = shape.bounding_box();
                            ShapeOut {
                                layer: board.board.layer_structure.layers[layer as usize]
                                    .name
                                    .clone(),
                                shape_kind: format!("{:?}", shape),
                                width_mm: bounds.width() as f64 / board.units_per_mm,
                                height_mm: bounds.height() as f64 / board.units_per_mm,
                            }
                        })
                    })
                    .collect()
            })
            .unwrap_or_default()
    } else {
        Vec::new()
    };
    ItemOut {
        id_run_local: item.id().0,
        kind: format!("{:?}", item.board_item_type()),
        reference,
        pad,
        net,
        center_mm: [center.x / board.units_per_mm, center.y / board.units_per_mm],
        copper_shapes,
        clearance_class: item.clearance_class(),
    }
}

fn key_by_id(board: &BoardAdapter, id: i32) -> Result<ItemKey, String> {
    board
        .board
        .get_items()
        .into_iter()
        .find(|&key| board.board.item(key).id().0 == id)
        .ok_or_else(|| format!("item id {id} absent in loaded baseline"))
}

fn net_route_layers(board: &BoardAdapter, net: fr_engine::ids::NetNo) -> Vec<(i32, String)> {
    board
        .board
        .layer_structure
        .layers
        .iter()
        .enumerate()
        .filter(|(i, layer)| {
            layer.is_signal
                && board
                    .board
                    .rules
                    .net_class_of(net)
                    .is_active_routing_layer(*i as i32)
        })
        .map(|(i, layer)| (i as i32, layer.name.clone()))
        .collect()
}

fn via_size_for_layer(board: &BoardAdapter, layer: i32) -> Result<f64, String> {
    let size = board
        .board
        .rules
        .via_infos
        .iter()
        .filter_map(|id| {
            board
                .board
                .library
                .padstacks
                .get(board.board.rules.via_infos[id].get_padstack())
        })
        .filter_map(|padstack| padstack.get_shape(layer))
        .map(|shape| shape.max_width() / board.units_per_mm)
        .fold(0.0_f64, f64::max);
    if size > 0.0 {
        Ok(size)
    } else {
        Err("DSN has no native via copper envelope on route layer".into())
    }
}

fn rounded_grid(point_mm: [f64; 2], board: &BoardAdapter, step_board: i32) -> [i32; 2] {
    let point = [
        point_mm[0] * board.units_per_mm,
        point_mm[1] * board.units_per_mm,
    ];
    [
        (point[0] / step_board as f64).round_ties_even() as i32,
        (point[1] / step_board as f64).round_ties_even() as i32,
    ]
}

fn request_window(board: &BoardAdapter, request: &RungRequest) -> Result<IntBox, String> {
    let points = [request.start.point, request.end.point];
    let min_x = points.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min);
    let max_x = points
        .iter()
        .map(|p| p[0])
        .fold(f64::NEG_INFINITY, f64::max);
    let min_y = points.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min);
    let max_y = points
        .iter()
        .map(|p| p[1])
        .fold(f64::NEG_INFINITY, f64::max);
    let center = [(min_x + max_x) / 2.0, (min_y + max_y) / 2.0];
    let half = (max_x - min_x).max(max_y - min_y) / 2.0 + request.spec.margin;
    let low = board.point([center[0] - half, center[1] - half])?;
    let high = board.point([center[0] + half, center[1] + half])?;
    let (EnginePoint::Int(low), EnginePoint::Int(high)) = (low, high) else {
        return Err("native window coordinate exceeds integer board range".into());
    };
    let bounds = board.board.bounding_box;
    let window = IntBox::new(
        low.x.max(bounds.ll.x),
        low.y.max(bounds.ll.y),
        high.x.min(bounds.ur.x),
        high.y.min(bounds.ur.y),
    );
    if window.is_empty() {
        return Err("native map window is empty".into());
    }
    Ok(window)
}

fn native_map_neighborhoods(
    request: &RungRequest,
    net_id: i32,
    from: [i32; 2],
    to: [i32; 2],
    frontier: &[(i32, i32, usize)],
) -> Result<(Vec<MapNeighborhoodOut>, Vec<MapCellOut>), String> {
    let board = &request.board;
    let step_board = (request.rules.grid_step * board.units_per_mm)
        .round()
        .max(1.0) as i32;
    let clearance_class = board
        .board
        .rules
        .net_class_of(net_id)
        .get_trace_clearance_class();
    // Endpoint-set routing passes its trace class for both copper and via clearance.
    let map = fr_agent::awx::native_obstacles::build_obstacle_map(
        board,
        request,
        request_window(board, request)?,
        step_board,
        &request.route_layers,
        &[net_id],
        clearance_class,
        clearance_class,
    )?;
    let top_index = request
        .route_layers
        .iter()
        .position(|layer| layer == "TopLayer")
        .ok_or("TopLayer is not in request route layers")?;
    let layer_no = map.layer_numbers[top_index];
    let track_half = map.config.track_width as f64 / 2.0;
    let contributors_for = |grid: [i32; 2], layer_no: i32| {
        let mut contributors = Vec::new();
        for key in board.board.get_items() {
            let item = board.board.item(key);
            if !item.is_pin()
                || !item.is_on_board()
                || item.net_numbers().contains(&net_id)
                || item.first_layer(&board.board) > layer_no
                || item.last_layer(&board.board) < layer_no
            {
                continue;
            }
            let Some(shape) = item.drill_shape_on_layer(&board.board, layer_no) else {
                continue;
            };
            let clearance = board
                .board
                .clearance_value(clearance_class, item.clearance_class(), layer_no)
                .max(map.config.clearance)
                .max(0);
            let margin = clearance as f64 + track_half;
            let raster_cells = pad_raster_source_diagnostic::source_pad_cells(
                &shape,
                step_board,
                board.units_per_mm,
                margin,
                0.5,
            )
            .unwrap_or_else(|| {
                pad_raster_source_diagnostic::source_polygon_cells(
                    &shape.corner_approx_arr(),
                    step_board,
                    margin,
                    board.units_per_mm,
                )
            });
            if raster_cells.contains(&grid) {
                let (reference, pad) = item_ref(board, key);
                let owner_net = item
                    .net_numbers()
                    .first()
                    .and_then(|n| board.board.rules.nets.get(*n).map(|rule| rule.name.clone()))
                    .unwrap_or_default();
                contributors.push(MapBlockerOut {
                    item_id_run_local: item.id().0,
                    reference,
                    pad,
                    net: owner_net,
                    copper_margin_mm: margin / board.units_per_mm,
                    native_raster_cell_match: true,
                });
            }
        }
        contributors
    };
    let mut out = Vec::new();
    for (label, center) in [("source", from), ("target", to)] {
        let mut cells = Vec::new();
        let mut open = 0;
        for dx in -1..=1 {
            for dy in -1..=1 {
                if dx == 0 && dy == 0 {
                    continue;
                }
                let grid = [center[0] + dx, center[1] + dy];
                let blocked = map.obstacles.is_blocked(grid[0], grid[1], top_index);
                if !blocked {
                    open += 1;
                }
                let point = FloatPoint::new(
                    grid[0] as f64 * step_board as f64,
                    grid[1] as f64 * step_board as f64,
                );
                cells.push(MapCellOut {
                    grid,
                    point_mm: [point.x / board.units_per_mm, point.y / board.units_per_mm],
                    blocked_in_native_base_map: blocked,
                    foreign_pad_contributors: contributors_for(grid, layer_no),
                });
            }
        }
        out.push(MapNeighborhoodOut {
            endpoint: label.to_owned(),
            center_grid: center,
            layer: "TopLayer".into(),
            open_same_layer_neighbors: open,
            cells,
        });
    }
    let frontier_out = frontier
        .iter()
        .filter_map(|&(gx, gy, layer_index)| {
            request.route_layers.get(layer_index)?;
            let native_layer = *map.layer_numbers.get(layer_index)?;
            let point =
                FloatPoint::new(gx as f64 * step_board as f64, gy as f64 * step_board as f64);
            Some(MapCellOut {
                grid: [gx, gy],
                point_mm: [point.x / board.units_per_mm, point.y / board.units_per_mm],
                blocked_in_native_base_map: map.obstacles.is_blocked(gx, gy, layer_index),
                foreign_pad_contributors: contributors_for([gx, gy], native_layer),
            })
        })
        .collect::<Vec<_>>();
    Ok((out, frontier_out))
}

fn run_one(
    board: &BoardAdapter,
    gap: usize,
    net: &str,
    from_id: i32,
    to_id: i32,
    grid_step_mm: f64,
) -> Result<ProbeOut, String> {
    let from_key = key_by_id(board, from_id)?;
    let to_key = key_by_id(board, to_id)?;
    let net_id = board.net_number(net)?;
    let route_layers_with_ids = net_route_layers(board, net_id);
    let route_layer = route_layers_with_ids
        .iter()
        .find(|(_, layer)| layer == "TopLayer")
        .map(|(id, _)| *id)
        .or_else(|| route_layers_with_ids.first().map(|(id, _)| *id))
        .ok_or_else(|| format!("net {net} has no active routing layers"))?;
    let class = board.board.rules.net_class_of(net_id);
    let track_width_mm = 2.0 * class.get_trace_half_width(route_layer) as f64 / board.units_per_mm;
    let clearance_mm = board.board.clearance_value(
        class.get_trace_clearance_class(),
        class.get_trace_clearance_class(),
        route_layer,
    ) as f64
        / board.units_per_mm;
    let via_size_mm = via_size_for_layer(board, route_layer)?;
    let mut rules = RoutingRules {
        grid_step: grid_step_mm,
        track_width: track_width_mm,
        clearance: clearance_mm,
        via_size: via_size_mm,
        via_drill: 0.3,
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
    rules.net_via_sizes.insert(net_id, [via_size_mm, 0.3]);
    let from = board.board.item(from_key).center(&board.board).to_float();
    let to = board.board.item(to_key).center(&board.board).to_float();
    let route_layers = route_layers_with_ids
        .iter()
        .map(|(_, name)| name.clone())
        .collect::<Vec<_>>();
    let existing_vias =
        fr_agent::awx::board_snapshot::BoardSnapshot::capture_vias(board, &BTreeMap::new())?;
    let [ax, ay] = [from.x / board.units_per_mm, from.y / board.units_per_mm];
    let [bx, by] = [to.x / board.units_per_mm, to.y / board.units_per_mm];
    let grid = |x: f64, y: f64| {
        [
            (x / grid_step_mm).round_ties_even() as i32,
            (y / grid_step_mm).round_ties_even() as i32,
        ]
    };
    let start = RungEndpoint {
        point: [ax, ay],
        layer: "TopLayer".into(),
    };
    let end = RungEndpoint {
        point: [bx, by],
        layer: "TopLayer".into(),
    };
    let request_for = |start: RungEndpoint, end: RungEndpoint| RungRequest {
        board: board.clone(),
        net: net.to_owned(),
        net_id,
        stage: format!("nopath_usb_gap_{gap}"),
        start,
        end,
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
        existing_vias: existing_vias.clone(),
        paste_pads: BTreeMap::new(),
        rules: rules.clone(),
        fabrication: FabTier::Standard,
        escalation: EscalationPolicy::Off,
        fabrication_overrides: BTreeMap::new(),
        fab_clearance_floor_mm: 0.0,
        fab_track_floor_mm: 0.0,
        may_narrow: false,
        fab_via_rungs: Vec::new(),
        max_iterations: 200_000,
        route_layers: route_layers.clone(),
        deadline: None,
    };
    request_for(start.clone(), end.clone()).rules.validate()?;
    let (endpoint_board, endpoint_report) = route_multilayer_endpoint_sets(
        request_for(start.clone(), end.clone()),
        from_key,
        to_key,
        &[start.clone()],
        &[end.clone()],
    )?;
    let endpoint_report = serde_json::json!({
        "same_endpoint_entry": endpoint_report,
        "route_board_returned": endpoint_board.is_some(),
    });

    // This direct RungExecutor call is a diagnostic control for raw frontier
    // cells. It deliberately does not stand in for the endpoint-set entry above.
    let frontier_request = request_for(start, end);
    let map_request = request_for(
        RungEndpoint {
            point: [ax, ay],
            layer: "TopLayer".into(),
        },
        RungEndpoint {
            point: [bx, by],
            layer: "TopLayer".into(),
        },
    );
    let (frontier_report, frontier) = match NativeRungExecutor::default().route(frontier_request)? {
        RungExecution::Found(route) => (
            serde_json::json!({"accepted":true,"expanded":route.grid_path.len(),"path_cells":route.grid_path.len()}),
            Vec::new(),
        ),
        RungExecution::Refused { reason, report } => {
            let report = report.unwrap_or_else(|| fr_agent::awx::braid_connect::RungReport {
                blocked_cells: Vec::new(),
                route_layers: route_layers.clone(),
                window: [0.0; 4],
                grid_step_mm,
                expanded: 0,
                truncated: true,
            });
            let cells = report.blocked_cells.clone();
            (
                serde_json::json!({"accepted":false,"reason":reason,"expanded":report.expanded,"window_mm":report.window,"truncated":report.truncated}),
                cells,
            )
        }
    };
    let (map_neighborhoods, map_frontier) =
        native_map_neighborhoods(&map_request, net_id, grid(ax, ay), grid(bx, by), &frontier)?;
    let mut near: Vec<_> = frontier
        .iter()
        .map(|&(x, y, layer)| FrontierCellOut {
            grid: [x, y],
            layer: route_layers
                .get(layer)
                .cloned()
                .unwrap_or_else(|| format!("layer_{layer}")),
            point_mm: [x as f64 * grid_step_mm, y as f64 * grid_step_mm],
            endpoint_distance_cells: ((x as f64 - grid(ax, ay)[0] as f64)
                .hypot(y as f64 - grid(ax, ay)[1] as f64))
            .min((x as f64 - grid(bx, by)[0] as f64).hypot(y as f64 - grid(bx, by)[1] as f64)),
        })
        .collect();
    near.sort_by(|a, b| {
        a.endpoint_distance_cells
            .total_cmp(&b.endpoint_distance_cells)
    });
    near.truncate(32);
    let mut closest = Vec::new();
    for cell in near.iter().take(12) {
        let point = FloatPoint::new(
            cell.point_mm[0] * board.units_per_mm,
            cell.point_mm[1] * board.units_per_mm,
        );
        let layer_no = board.board.layer_structure.get_no(&cell.layer);
        let mut distances = Vec::new();
        for key in board.board.get_items() {
            let item = board.board.item(key);
            if item.net_numbers().contains(&net_id) || !item.is_on_board() {
                continue;
            }
            let distance = board
                .board
                .item_tile_shapes(item)
                .iter()
                .enumerate()
                .filter(|(i, _)| board.board.shape_layer(key, *i as i32) == layer_no)
                .filter_map(|(_, shape)| shape.as_ref())
                .map(|shape| shape.distance(&point) / board.units_per_mm)
                .fold(f64::INFINITY, f64::min);
            if distance.is_finite() {
                let (reference, pad) = item_ref(board, key);
                let owner = item
                    .net_numbers()
                    .first()
                    .and_then(|n| board.board.rules.nets.get(*n).map(|r| r.name.clone()))
                    .unwrap_or_default();
                distances.push(NearestItemOut {
                    frontier_grid: cell.grid,
                    frontier_layer: cell.layer.clone(),
                    item_id_run_local: item.id().0,
                    kind: format!("{:?}", item.board_item_type()),
                    reference,
                    pad,
                    net: owner,
                    copper_shape_distance_mm: distance,
                });
            }
        }
        distances.sort_by(|a, b| {
            a.copper_shape_distance_mm
                .total_cmp(&b.copper_shape_distance_mm)
        });
        closest.extend(distances.into_iter().take(3));
    }
    Ok(ProbeOut {
        gap,
        net: net.to_owned(),
        from: item_out(board, from_key),
        to: item_out(board, to_key),
        route_layers,
        grid_step_mm,
        clearance_mm,
        track_width_mm,
        via_diameter_mm: via_size_mm,
        via_drill_mm: 0.3,
        endpoints_grid: [grid(ax, ay), grid(bx, by)],
        native_router: serde_json::json!({"endpoint_sets_entry": endpoint_report, "frontier_control": frontier_report}),
        frontier_cells: frontier.len(),
        frontier_near_endpoints: near,
        native_map_endpoint_neighborhoods: map_neighborhoods,
        native_map_frontier_readback: map_frontier,
        closest_foreign_items: closest,
    })
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = env::args_os().skip(1);
    let dsn = PathBuf::from(
        args.next()
            .ok_or("usage: nopath_usb <board.dsn> <baseline.ses> <out_dir>")?,
    );
    let ses = PathBuf::from(args.next().ok_or("missing baseline.ses")?);
    let out_dir = PathBuf::from(args.next().ok_or("missing out_dir")?);
    let dsn_bytes = fs::read(&dsn)?;
    let ses_bytes = fs::read(&ses)?;
    let source = BoardAdapter::load(&dsn_bytes).map_err(std::io::Error::other)?;
    let via_candidates = source
        .board
        .rules
        .nets
        .iter()
        .map(|net| {
            let route_layer = net_route_layers(&source, net.net_number)
                .first()
                .map(|(id, _)| *id)
                .ok_or_else(|| format!("net {} has no active routing layer", net.name))?;
            Ok((
                net.name.clone(),
                via_size_for_layer(&source, route_layer)?,
                0.3,
            ))
        })
        .collect::<Result<Vec<_>, String>>()
        .map_err(std::io::Error::other)?;
    let board =
        BoardAdapter::load_session_with_via_candidates(&dsn_bytes, &ses_bytes, &via_candidates)
            .map_err(std::io::Error::other)?;
    let gaps = [(15, "USB_FS_DP", 392, 390), (18, "USB_FS_DM", 389, 391)];
    let mut rows = Vec::new();
    for (gap, net, from_id, to_id) in gaps {
        let from = key_by_id(&board, from_id).map_err(std::io::Error::other)?;
        let incompletes = NetIncompletes::new(
            &board.board,
            board.net_number(net)?,
            &board.board.get_connectable_items(board.net_number(net)?),
        );
        let found = incompletes.incompletes.iter().any(|a| {
            board.board.item(a.from_item).id().0 == from_id
                || board.board.item(a.to_item).id().0 == from_id
        });
        let _ = from;
        if !found {
            return Err(
                format!("gap {gap} endpoints do not match this SES connectivity snapshot").into(),
            );
        }
        rows.push(run_one(&board, gap, net, from_id, to_id, 0.25).map_err(std::io::Error::other)?);
    }
    fs::create_dir_all(&out_dir)?;
    let out = out_dir.join("nopath_usb.json");
    fs::write(&out, serde_json::to_vec_pretty(&rows)?)?;
    println!("{}", out.display());
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("nopath_usb: {error}");
        std::process::exit(1);
    }
}
