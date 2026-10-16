use fr_agent::{
    awx::{
        board_snapshot::BoardSnapshot,
        braid_connect::{rung_specs, LadderMode, RungEndpoint, RungRequest},
        fab_rules::{EscalationPolicy, FabTier},
        native_obstacles::build_obstacle_map,
        whole_context::RoutingRules,
    },
    awx_endpoint::{endpoint_pad_metadata, PadAccess},
    board_adapter::BoardAdapter,
};
use fr_engine::{
    board::{ItemKey, ItemKind},
    drc::NetIncompletes,
    ids::ClearanceClassNo,
};
use fr_geom::{ConvexShape, FloatPoint, IntBox, TileShapeImpl};
use serde::{Deserialize, Serialize};
use sha2::Digest;
use std::{collections::BTreeMap, env, fs, path::PathBuf};

#[derive(Deserialize)]
struct Config {
    dsn: PathBuf,
    ses: PathBuf,
    output: PathBuf,
    grid_step_mm: f64,
    via_diameter_mm: f64,
    via_drill_mm: f64,
    nets: Vec<String>,
    #[serde(default)]
    pad_access: PadAccess,
    #[serde(default)]
    cell_probes: Vec<CellProbeRequest>,
}

#[derive(Deserialize)]
struct CellProbeRequest {
    net: String,
    grid: [i32; 2],
    layer: String,
}

#[derive(Serialize)]
struct Report {
    session: PathBuf,
    dsn_sha256: String,
    session_sha256: String,
    units_per_mm: f64,
    board_layers: Vec<String>,
    native_trace_counts_by_net_layer: BTreeMap<String, usize>,
    ses_wire_counts_by_net_layer: BTreeMap<String, usize>,
    grid_step_mm: f64,
    via_diameter_mm: f64,
    via_drill_mm: f64,
    pad_access: &'static str,
    via_candidate_note: &'static str,
    selection: &'static str,
    state_note: &'static str,
    probes: Vec<NetProbe>,
}

#[derive(Serialize)]
struct NetProbe {
    net: String,
    airline: Airline,
    route_layers: Vec<String>,
    actual_grid_step_mm: f64,
    track_width_mm: f64,
    clearance_mm: f64,
    rule_pair: String,
    map_scope_note: &'static str,
    neighborhood_radius_cells: i32,
    endpoints: Vec<EndpointProbe>,
    endpoint_group_sample: Vec<String>,
    endpoint_group_sample_limit: usize,
    specified_cells: Vec<SpecifiedCellProbe>,
}

#[derive(Serialize)]
struct SpecifiedCellProbe {
    grid: [i32; 2],
    layer: String,
    inside_search_window: bool,
    map_track_blocked: bool,
    blockers: Vec<Blocker>,
}

#[derive(Serialize)]
struct Airline {
    from: Endpoint,
    to: Endpoint,
}

#[derive(Serialize)]
struct Endpoint {
    item_id: i32,
    item_type: String,
    net: String,
    pin_identity: Option<String>,
    item_center_mm: [f64; 2],
    airline_endpoint_mm: [f64; 2],
    connected_group_sample: Vec<String>,
}

#[derive(Serialize)]
struct EndpointProbe {
    endpoint_item_id: i32,
    actual_endpoint_mm: [f64; 2],
    grid_center_mm: [f64; 2],
    grid_center_offset_mm: f64,
    cells: Vec<CellProbe>,
}

#[derive(Serialize)]
struct CellProbe {
    grid: [i32; 2],
    layer: String,
    map_track_blocked: bool,
    map_via_blocked: bool,
    exact_endpoint_override_would_clear_track: bool,
    exact_endpoint_override_applies: bool,
    blockers: Vec<Blocker>,
    via_candidates: Vec<Blocker>,
}

#[derive(Serialize)]
struct Blocker {
    item_id: i32,
    item_type: String,
    net: String,
    pin_identity: Option<String>,
    native_item_center_mm: Option<[f64; 2]>,
    native_trace_centerline_mm: Option<Vec<[f64; 2]>>,
    native_trace_half_width_mm: Option<f64>,
    native_shape_debug: Option<String>,
    native_shape_bounds_mm: Option<[f64; 4]>,
    layer: String,
    copper_distance_mm: f64,
    track_radius_mm: f64,
    clearance_mm: f64,
    required_centerline_distance_mm: f64,
    physical_clearance_violation: bool,
    source_cell_hit: Option<bool>,
    rule_pair: String,
    cell_center_to_actual_endpoint_mm: f64,
    evidence: &'static str,
    raster_method: &'static str,
}

fn item_net(board: &BoardAdapter, key: ItemKey) -> String {
    board.board.item(key).net_numbers().first()
        .and_then(|n| board.board.rules.nets.get(*n))
        .map(|n| n.name.clone()).unwrap_or_default()
}

fn pin_identity(board: &BoardAdapter, key: ItemKey) -> Option<String> {
    let item = board.board.item(key);
    let ItemKind::Pin(pin) = &item.kind else { return None };
    let component = board.board.components.get(item.component_no());
    let package = board.board.library.packages.get(component.get_package());
    let pad = package.get_pin(pin.pin_index)?.name.clone();
    Some(format!("{}-{pad}", component.name))
}

fn item_type(board: &BoardAdapter, key: ItemKey) -> String {
    format!("{:?}", board.board.item(key).board_item_type())
}

fn point_mm(board: &BoardAdapter, point: FloatPoint) -> [f64; 2] {
    [point.x / board.units_per_mm, point.y / board.units_per_mm]
}

fn route_layers(board: &BoardAdapter, net_id: fr_engine::ids::NetNo) -> Vec<String> {
    let class = board.board.rules.net_class_of(net_id);
    board.board.layer_structure.layers.iter().enumerate()
        .filter(|(i, layer)| layer.is_signal && class.is_active_routing_layer(*i as i32))
        .map(|(_, layer)| layer.name.clone()).collect()
}

fn request(board: &BoardAdapter, net_id: fr_engine::ids::NetNo, net: &str,
           start: [f64; 2], end: [f64; 2], layers: &[String], step: f64,
           via_diameter: f64, via_drill: f64) -> Result<RungRequest, String> {
    let class = board.board.rules.net_class_of(net_id);
    let first_layer = board.board.layer_structure.get_no(&layers[0]);
    let width = 2.0 * class.get_trace_half_width(first_layer) as f64 / board.units_per_mm;
    let clearance_class = class.get_trace_clearance_class();
    let clearance = board.board.clearance_value(clearance_class, clearance_class, first_layer) as f64 / board.units_per_mm;
    let mut rules = RoutingRules {
        grid_step: step, track_width: width, clearance, via_size: via_diameter, via_drill,
        pair_gap: 0.0, lane_pitch: 1.0, lane_min: 0.25, min_turning_radius: 0.0,
        max_turn_angle: 180.0, max_setback_angle: 45.0, routing_clearance_margin: 1.0,
        board_edge_clearance: 0.0, hole_to_hole_clearance: 0.0, fan_clearance: 0.0,
        geometry_pair_room: false, geometry_pair_turn_room_steps: None,
        same_net_pad_clearance: 0.0, net_via_sizes: BTreeMap::new(),
    };
    rules.net_via_sizes.insert(net_id, [via_diameter, via_drill]);
    let (free_via_pads, same_net_pads) = endpoint_pad_metadata(board, false)?;
    Ok(RungRequest {
        board: board.clone(), net: net.to_owned(), net_id, stage: "frozen_blocker_diagnostic".into(),
        start: RungEndpoint { point: start, layer: layers[0].clone() },
        end: RungEndpoint { point: end, layer: layers[0].clone() },
        b_alternatives: Vec::new(), pair: None, spec: rung_specs(LadderMode::Open)[2],
        band: None, band_slack: 0.0, window_points: None, virtual_tracks: Vec::new(),
        virtual_vias: Vec::new(), free_via_pads, same_net_pads,
        existing_vias: BoardSnapshot::capture_vias(board, &BTreeMap::new())?,
        paste_pads: BTreeMap::new(), rules,
        fabrication: FabTier::Standard, escalation: EscalationPolicy::Off,
        fabrication_overrides: BTreeMap::new(), fab_clearance_floor_mm: 0.0,
        fab_track_floor_mm: 0.0, may_narrow: false, fab_via_rungs: Vec::new(),
        min_annular_ring_mm: 0.1, max_iterations: 0, route_layers: layers.to_vec(), deadline: None,
    })
}

fn group_names(board: &BoardAdapter, group: &fr_engine::board::ItemSet, cap: usize) -> Vec<String> {
    let mut names = group.iter().filter_map(|k| pin_identity(board, k)).collect::<Vec<_>>();
    names.sort(); names.dedup(); names.truncate(cap); names
}

fn native_trace_counts(board: &BoardAdapter) -> BTreeMap<String, usize> {
    let mut counts = BTreeMap::new();
    for key in board.board.get_traces() {
        let item = board.board.item(key);
        let Some(trace) = item.as_trace() else { continue };
        let layer = board.board.layer_structure.layers[trace.layer() as usize].name.clone();
        let net = item.net_numbers().first()
            .and_then(|n| board.board.rules.nets.get(*n))
            .map(|n| n.name.clone()).unwrap_or_default();
        *counts.entry(format!("{net}|{layer}")).or_insert(0) += 1;
    }
    counts
}

fn ses_wire_counts(src: &[u8]) -> Result<BTreeMap<String, usize>, String> {
    let expressions = fr_dsn::sexpr::parse(src).map_err(|e| format!("SES parse: {e}"))?;
    let root = expressions.first().and_then(frd_root).ok_or("SES missing root")?;
    let routes = root.find("routes").ok_or("SES missing routes")?;
    let network = routes.find("network_out").ok_or("SES missing network_out")?;
    let mut counts = BTreeMap::new();
    for net_scope in network.sublists().filter(|list| list.is("net")) {
        let Some(net_name) = net_scope.args().first().and_then(fr_dsn::sexpr::Sexpr::as_atom)
            .map(|atom| atom.text.clone()) else { continue };
        for record in net_scope.sublists().filter(|list| list.is("wire")) {
            let Some(layer_name) = record.find("path")
                .and_then(|path| path.args().first())
                .and_then(fr_dsn::sexpr::Sexpr::as_atom)
                .map(|atom| atom.text.clone()) else { continue };
            *counts.entry(format!("{net_name}|{layer_name}")).or_insert(0) += 1;
        }
    }
    Ok(counts)
}

fn frd_root(expr: &fr_dsn::sexpr::Sexpr) -> Option<&fr_dsn::sexpr::List> {
    expr.as_list().filter(|list| list.is("session"))
}

fn endpoint(board: &BoardAdapter, key: ItemKey, point: FloatPoint,
            group: &fr_engine::board::ItemSet) -> Endpoint {
    Endpoint { item_id: board.board.item(key).id().0, item_type: item_type(board, key),
        net: item_net(board, key), pin_identity: pin_identity(board, key),
        item_center_mm: point_mm(board, board.board.item(key).center(&board.board).to_float()),
        airline_endpoint_mm: point_mm(board, point), connected_group_sample: group_names(board, group, 8) }
}

fn segment_distance(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let dx = b[0]-a[0]; let dy = b[1]-a[1];
    let l2 = dx*dx+dy*dy;
    let t = if l2 == 0.0 { 0.0 } else { (((p[0]-a[0])*dx+(p[1]-a[1])*dy)/l2).clamp(0.0,1.0) };
    ((p[0]-a[0]-t*dx).powi(2)+(p[1]-a[1]-t*dy).powi(2)).sqrt()
}

fn local_items(board: &BoardAdapter, p: [f64; 2], radius_mm: f64, layer: i32) -> Vec<ItemKey> {
    let r = (radius_mm * board.units_per_mm).ceil() as i32;
    let x = (p[0] * board.units_per_mm).round() as i32;
    let y = (p[1] * board.units_per_mm).round() as i32;
    let area = IntBox::new(x-r, y-r, x+r, y+r).to_tile_shape();
    board.board.overlapping_items_of(&ConvexShape::Tile(area), layer).iter().collect()
}

fn track_source_cell_hit(board: &BoardAdapter, key: ItemKey, layer_no: i32,
                         gx: i32, gy: i32, route_class: ClearanceClassNo,
                         clearance_floor_mm: f64, track_width_mm: f64,
                         step_native: i32) -> Option<bool> {
    let item = board.board.item(key);
    let clearance = board.board.clearance_value(route_class, item.clearance_class(), layer_no).max(0);
    let floor_native = (clearance_floor_mm * board.units_per_mm).round() as i32;
    let clearance = clearance.max(floor_native) as f64;
    if let Some(trace) = item.as_trace().filter(|t| t.layer() == layer_no) {
        let corners = trace.polyline().corners();
        let margin = track_width_mm * board.units_per_mm / 2.0 + trace.half_width() as f64 + clearance;
        let hit = corners.windows(2).any(|w| {
            let a = w[0].to_float(); let b = w[1].to_float();
            fr_agent::awx::native_via_obstacles::capsule_cells(
                [a.x, a.y], [b.x, b.y], margin, step_native, board.units_per_mm)
                .contains(&[gx, gy])
        });
        return Some(hit);
    }
    if matches!(&item.kind, ItemKind::Pin(_)) {
        let Some(shape) = item.drill_shape_on_layer(&board.board, layer_no) else { return Some(false); };
        let margin = clearance + track_width_mm * board.units_per_mm / 2.0;
        if let Some(cells) = fr_agent::awx::native_pad_raster::source_pad_cells(
            &shape, step_native, board.units_per_mm, margin, 0.5) {
            return Some(cells.contains(&[gx, gy]));
        }
        let cells = fr_agent::awx::native_pad_raster::source_polygon_cells(
            &shape.corner_approx_arr(), step_native, margin, board.units_per_mm);
        return Some(cells.contains(&[gx, gy]));
    }
    None
}

fn blocker_for(board: &BoardAdapter, key: ItemKey, layer_no: i32, layer: &str,
               p: [f64; 2], endpoint_mm: [f64; 2], track_radius: f64,
               route_clearance_class: ClearanceClassNo,
               clearance_floor: f64, step: f64, is_via: bool,
               source_cell_hit: Option<bool>) -> Option<Blocker> {
    let item = board.board.item(key);
    if !item.is_on_board() { return None; }
    let native_item_center_mm = (item.is_pin() || item.is_via()).then(|| {
        point_mm(board, item.center(&board.board).to_float())
    });
    let native_trace = item.as_trace().filter(|t| t.layer() == layer_no);
    let native_trace_centerline_mm = native_trace.map(|trace| {
        trace.polyline().corners().iter().map(|point| {
            let point = point.to_float();
            point_mm(board, point)
        }).collect::<Vec<_>>()
    });
    let native_trace_half_width_mm = native_trace
        .map(|trace| trace.half_width() as f64 / board.units_per_mm);
    let clearance_native = board.board.clearance_value(
        route_clearance_class, item.clearance_class(), layer_no).max(0) as f64;
    let clearance = (clearance_native / board.units_per_mm).max(clearance_floor);
    let copper = item.drill_shape_on_layer(&board.board, layer_no);
    let native_shape_debug = copper.as_ref().map(|shape| format!("{shape:?}"));
    let native_shape_bounds_mm = copper.as_ref().map(|shape| {
        let bounds = shape.bounding_box();
        [
            bounds.ll.x as f64 / board.units_per_mm,
            bounds.ll.y as f64 / board.units_per_mm,
            bounds.ur.x as f64 / board.units_per_mm,
            bounds.ur.y as f64 / board.units_per_mm,
        ]
    });
    let (distance, method) = if let Some(shape) = copper {
        let point = FloatPoint::new(p[0] * board.units_per_mm, p[1] * board.units_per_mm);
        let d = shape.distance(&point) / board.units_per_mm;
        let method = if is_via {
            "via candidate; actual pad copper queried, source via stamp is candidate-size-dependent"
        } else if fr_agent::awx::native_pad_raster::source_pad_cells(
            &shape, (step * board.units_per_mm).round() as i32, board.units_per_mm,
            (track_radius + clearance) * board.units_per_mm, 0.5).is_some() {
            "source_pad_cells circle/rectangle with source corner buffer"
        } else {
            "source_polygon_cells polygon margin"
        };
        (d, method)
    } else if let Some(trace) = item.as_trace().filter(|t| t.layer() == layer_no) {
        let points = trace.polyline().corners();
        let d = points.windows(2).map(|w| segment_distance(p,
            [w[0].to_float().x/board.units_per_mm, w[0].to_float().y/board.units_per_mm],
            [w[1].to_float().x/board.units_per_mm, w[1].to_float().y/board.units_per_mm])).fold(f64::INFINITY, f64::min);
        let half = trace.half_width() as f64/board.units_per_mm;
        (d - half, if is_via { "via candidate; trace capsule uses candidate via size" } else { "trace capsule; native capsule_cells membership" })
    } else { return None; };
    let required = track_radius + clearance;
    if distance > required + step * 1.1 { return None; }
    Some(Blocker { item_id: item.id().0, item_type: item_type(board,key), net: item_net(board,key),
        pin_identity: pin_identity(board,key), native_item_center_mm, native_trace_centerline_mm,
        native_trace_half_width_mm, native_shape_debug, native_shape_bounds_mm,
        layer: layer.to_owned(), copper_distance_mm: distance,
        track_radius_mm: track_radius, clearance_mm: clearance,
        required_centerline_distance_mm: required,
        physical_clearance_violation: distance < required,
        source_cell_hit,
        rule_pair: format!("route clearance class {:?} / item class {:?} on {layer}; resolved {clearance} mm",
            route_clearance_class, item.clearance_class()),
        cell_center_to_actual_endpoint_mm: ((p[0]-endpoint_mm[0]).powi(2)+(p[1]-endpoint_mm[1]).powi(2)).sqrt(),
        evidence: if is_via {"candidate; via item stamps use candidate-specific geometry"} else if source_cell_hit == Some(true) {"confirmed per-object source-raster cell hit; physical test is reported separately"} else if source_cell_hit == Some(false) {"this object's source raster does not stamp this cell"} else {"nearby geometric candidate; source raster implementation does not expose attribution for this item type"},
        raster_method: method })
}

fn analyze_net(board: &BoardAdapter, net_name: &str, step: f64, via_diameter: f64,
               via_drill: f64, specified_cells: &[CellProbeRequest])
    -> Result<NetProbe, Box<dyn std::error::Error>> {
    let net_id = board.net_number(net_name)?;
    let groups = board.board.get_connected_sets(net_id);
    let gaps = NetIncompletes::new(&board.board, net_id, &board.board.get_connectable_items(net_id));
    let edge = gaps.incompletes.first().ok_or_else(|| format!("{net_name} has no residual airline"))?;
    let from_group = groups.iter().find(|g| g.contains(board.board.item(edge.from_item).id())).ok_or("missing from group")?;
    let to_group = groups.iter().find(|g| g.contains(board.board.item(edge.to_item).id())).ok_or("missing to group")?;
    let from = endpoint(board, edge.from_item, edge.from_corner, from_group);
    let to = endpoint(board, edge.to_item, edge.to_corner, to_group);
    let airline = Airline { from, to };
    let endpoints = [(edge.from_item, edge.from_corner, from_group), (edge.to_item, edge.to_corner, to_group)];
    let layers = route_layers(board, net_id);
    if layers.is_empty() { return Err(format!("{net_name}: no active routing layers").into()); }
    let a = point_mm(board, edge.from_corner); let b = point_mm(board, edge.to_corner);
    let req = request(board, net_id, net_name, a, b, &layers, step, via_diameter, via_drill)
        .map_err(std::io::Error::other)?;
    let class = board.board.rules.net_class_of(net_id);
    let clearance_class = class.get_trace_clearance_class();
    let width = req.rules.track_width;
    let clearance = req.rules.clearance;
    let rule_pair = format!("clearance class {:?}/{:?}; layer {}", clearance_class, clearance_class, layers[0]);
    let grid_native = (step * board.units_per_mm).round() as i32;
    let actual_step = grid_native as f64 / board.units_per_mm;
    let window = IntBox::new(
        ((a[0].min(b[0])-2.0)*board.units_per_mm).floor() as i32,
        ((a[1].min(b[1])-2.0)*board.units_per_mm).floor() as i32,
        ((a[0].max(b[0])+2.0)*board.units_per_mm).ceil() as i32,
        ((a[1].max(b[1])+2.0)*board.units_per_mm).ceil() as i32);
    let map = build_obstacle_map(board, &req, window, grid_native, &layers, &[net_id],
        clearance_class as i32, clearance_class as i32).map_err(std::io::Error::other)?;
    let mut endpoint_probes = Vec::new();
    for (key, corner, _) in endpoints {
        let actual = point_mm(board, corner);
        let grid = [(actual[0]/actual_step).round_ties_even() as i32, (actual[1]/actual_step).round_ties_even() as i32];
        let center = [grid[0] as f64*actual_step, grid[1] as f64*actual_step];
        let mut cells = Vec::new();
        for dy in -2..=2 { for dx in -2..=2 {
            let g = [grid[0]+dx, grid[1]+dy];
            let p = [g[0] as f64*actual_step, g[1] as f64*actual_step];
            for (layer_idx, &layer_no) in map.layer_numbers.iter().enumerate() {
                let layer = &layers[layer_idx];
                let nearby = local_items(board, p, 1.8, layer_no);
                let mut blockers = Vec::new();
                let mut via_candidates = Vec::new();
                for other in nearby {
                    let item = board.board.item(other);
                    if item.net_numbers().contains(&net_id) { continue; }
                    if let Some(b) = blocker_for(board, other, layer_no, layer, p, actual,
                        width/2.0, clearance_class, clearance, actual_step, false,
                        track_source_cell_hit(board, other, layer_no, g[0], g[1], clearance_class,
                            clearance, width, grid_native)) { blockers.push(b); }
                    if let Some(b) = blocker_for(board, other, layer_no, layer, p, actual,
                        via_diameter/2.0, clearance_class, clearance, actual_step, true, None) { via_candidates.push(b); }
                }
                blockers.sort_by(|a,b| a.copper_distance_mm.total_cmp(&b.copper_distance_mm));
                via_candidates.sort_by(|a,b| a.copper_distance_mm.total_cmp(&b.copper_distance_mm));
                let exact = dx == 0 && dy == 0
                    && board.board.item(key).drill_shape_on_layer(&board.board, layer_no).is_some();
                cells.push(CellProbe { grid: g, layer: layer.clone(),
                    map_track_blocked: map.obstacles.is_blocked(g[0],g[1],layer_idx),
                    map_via_blocked: map.obstacles.is_via_blocked(g[0],g[1]),
                    exact_endpoint_override_would_clear_track: exact && map.obstacles.is_blocked(g[0],g[1],layer_idx),
                    exact_endpoint_override_applies: exact, blockers, via_candidates });
            }
        }}
        endpoint_probes.push(EndpointProbe { endpoint_item_id: board.board.item(key).id().0,
            actual_endpoint_mm: actual, grid_center_mm: center,
            grid_center_offset_mm: ((center[0]-actual[0]).powi(2)+(center[1]-actual[1]).powi(2)).sqrt(), cells });
    }
    let mut specified = Vec::new();
    for probe in specified_cells.iter().filter(|probe| probe.net == net_name) {
        let Some(layer_index) = layers.iter().position(|layer| layer == &probe.layer) else {
            return Err(format!("{}: specified probe layer {} is not active", net_name, probe.layer).into());
        };
        let Some(&layer_no) = map.layer_numbers.get(layer_index) else {
            return Err(format!("{}: specified probe layer has no native number", net_name).into());
        };
        let point = [probe.grid[0] as f64 * actual_step, probe.grid[1] as f64 * actual_step];
        let nearby = local_items(board, point, 1.8, layer_no);
        let mut blockers = Vec::new();
        for other in nearby {
            if board.board.item(other).net_numbers().contains(&net_id) { continue; }
            if let Some(blocker) = blocker_for(board, other, layer_no, &probe.layer, point,
                point, width / 2.0, clearance_class, clearance, actual_step, false,
                track_source_cell_hit(board, other, layer_no, probe.grid[0], probe.grid[1],
                    clearance_class, clearance, width, grid_native))
            {
                blockers.push(blocker);
            }
        }
        blockers.sort_by(|a,b| a.copper_distance_mm.total_cmp(&b.copper_distance_mm));
        let point_native = [probe.grid[0].saturating_mul(grid_native), probe.grid[1].saturating_mul(grid_native)];
        let inside_search_window = point_native[0] >= window.ll.x && point_native[0] <= window.ur.x
            && point_native[1] >= window.ll.y && point_native[1] <= window.ur.y;
        specified.push(SpecifiedCellProbe {
            grid: probe.grid,
            layer: probe.layer.clone(),
            inside_search_window,
            map_track_blocked: map.obstacles.is_blocked(probe.grid[0], probe.grid[1], layer_index),
            blockers,
        });
    }
    Ok(NetProbe { net: net_name.into(), airline, route_layers: layers,
        actual_grid_step_mm: map.config.grid_step as f64 / board.units_per_mm,
        track_width_mm: width,
        clearance_mm: clearance, rule_pair,
        map_scope_note: "One frozen per-net obstacle map covers the selected airline bounding box plus 2 mm; all reported 5x5 neighborhoods are inside it. This is local evidence, not a reachability proof.",
        neighborhood_radius_cells: 2,
        endpoints: endpoint_probes,
        endpoint_group_sample: group_names(board, from_group, 8), endpoint_group_sample_limit: 8,
        specified_cells: specified })
}

fn run(config_path: PathBuf) -> Result<(), Box<dyn std::error::Error>> {
    let config_bytes = fs::read(&config_path)?;
    let cfg: Config = if config_path.extension().and_then(|ext| ext.to_str()) == Some("toml") {
        toml::from_str(std::str::from_utf8(&config_bytes)?)?
    } else {
        serde_json::from_slice(&config_bytes)?
    };
    if cfg.nets.is_empty() || cfg.grid_step_mm <= 0.0 { return Err("nets must be nonempty and grid_step_mm positive".into()); }
    let dsn_bytes = fs::read(&cfg.dsn)?; let ses_bytes = fs::read(&cfg.ses)?;
    let via_candidates = {
        let mut source = BoardAdapter::load(&dsn_bytes).map_err(std::io::Error::other)?;
        BoardSnapshot::apply_configured_via_drill(&mut source, cfg.via_drill_mm)?;
        source.board.rules.nets.iter().map(|n| (n.name.clone(),cfg.via_diameter_mm,cfg.via_drill_mm)).collect::<Vec<_>>()
    };
    let mut board = BoardAdapter::load_session_with_via_candidates(&dsn_bytes,&ses_bytes,&via_candidates)
        .map_err(std::io::Error::other)?;
    BoardSnapshot::apply_configured_via_drill(&mut board, cfg.via_drill_mm)?;
    board.configure_awx_hole_rules(0.0);
    board.board.set_pad_area_contacts(cfg.pad_access.uses_copper_area());
    let mut probes = Vec::new();
    for net in &cfg.nets {
        probes.push(analyze_net(&board,net,cfg.grid_step_mm,cfg.via_diameter_mm,cfg.via_drill_mm,
            &cfg.cell_probes)?);
    }
    let report = Report { session: cfg.ses, grid_step_mm: cfg.grid_step_mm,
        dsn_sha256: format!("{:x}", sha2::Sha256::digest(&dsn_bytes)),
        session_sha256: format!("{:x}", sha2::Sha256::digest(&ses_bytes)),
        units_per_mm: board.units_per_mm,
        board_layers: board.board.layer_structure.layers.iter().map(|layer| layer.name.clone()).collect(),
        native_trace_counts_by_net_layer: native_trace_counts(&board),
        ses_wire_counts_by_net_layer: ses_wire_counts(&ses_bytes)?,
        via_diameter_mm: cfg.via_diameter_mm, via_drill_mm: cfg.via_drill_mm,
        pad_access: cfg.pad_access.as_str(),
        via_candidate_note: "Via map provenance is candidate-only: session does not expose all existing hole geometry; H2H0 is configured.",
        selection: "One deterministic NetIncompletes airline per named net; each endpoint shows 5x5 cells per active signal layer. Group samples cap at eight pins.",
        state_note: "This is a read-only diagnosis of the supplied final SES state; it does not reconstruct the state at a failed route attempt.", probes };
    if let Some(parent) = cfg.output.parent() { fs::create_dir_all(parent)?; }
    fs::write(&cfg.output,serde_json::to_vec_pretty(&report)?)?;
    println!("{}",cfg.output.display()); Ok(())
}

fn main() {
    let mut args = env::args_os().skip(1);
    let config = args.next().map(PathBuf::from);
    if config.is_none() || args.next().is_some() { eprintln!("usage: cargo run -p fr_agent --example awx_blockers -- <config.toml|config.json>"); std::process::exit(2); }
    if let Err(e) = run(config.unwrap()) { eprintln!("awx_blockers: {e}"); std::process::exit(1); }
}
