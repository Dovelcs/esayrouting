use fr_agent::{
    awx::{
        braid_connect::{rung_specs, LadderMode, RungEndpoint, RungRequest},
        fab_rules::{EscalationPolicy, FabTier},
        native_obstacles,
        whole_context::RoutingRules,
    },
    board_adapter::BoardAdapter,
};
use fr_engine::{
    board::{ItemKey, ItemKind},
    drc::NetIncompletes,
    ids::NetNo,
};
use fr_geom::{FloatPoint, IntBox, IntPoint, Point};
use serde::Serialize;
use std::{
    collections::{BTreeMap, BTreeSet},
    env, fs,
    path::PathBuf,
};

const PIN_REF: &str = "u1-d52f49d10ebf941e14";
const GRID_STEP_MM: f64 = 0.25;
const VIA_DRILL_MM: f64 = 0.3;

#[derive(Serialize)]
struct Report {
    input_dsn: PathBuf,
    input_ses: PathBuf,
    net: String,
    pin_ref: String,
    pin: ItemOut,
    connected_pin_count: usize,
    connected_group_candidate_count: usize,
    connected_group_candidates_by_layer: BTreeMap<String, usize>,
    relevant_gaps: Vec<GapOut>,
    grid_step_mm: f64,
    track_width_mm: f64,
    clearance_by_layer_mm: BTreeMap<String, f64>,
    via_diameter_mm: f64,
    via_drill_mm: f64,
    local_grid: LocalGridOut,
    via_in_pad_probe: ViaInPadOut,
    obstacle_semantics: &'static str,
}

#[derive(Serialize)]
struct ItemOut {
    item_id_run_local: i32,
    reference: String,
    pad: String,
    xy_mm: [f64; 2],
    copper: Vec<CopperOut>,
}

#[derive(Serialize)]
struct CopperOut {
    layer: String,
    shape_kind: String,
    width_mm: f64,
    height_mm: f64,
}

#[derive(Serialize)]
struct GapOut {
    from_item_id_run_local: i32,
    to_item_id_run_local: i32,
    from_xy_mm: [f64; 2],
    to_xy_mm: [f64; 2],
    from_group_pin_count: usize,
    to_group_pin_count: usize,
    from_group_candidate_count: usize,
    to_group_candidate_count: usize,
    from_group_contains_probe_pin: bool,
    to_group_contains_probe_pin: bool,
}

#[derive(Serialize)]
struct LocalGridOut {
    probe_cell: [i32; 2],
    center_by_layer: Vec<CellOut>,
    neighbors_by_layer: Vec<LayerNeighborsOut>,
}

#[derive(Serialize)]
struct ViaInPadOut {
    exact_copper_grid_cells: usize,
    via_blocked_cells: usize,
    via_open_cells: Vec<[i32; 2]>,
    via_keepout_margin_mm: f64,
}

#[derive(Serialize)]
struct LayerNeighborsOut {
    layer: String,
    blocked_count: usize,
    open_count: usize,
    neighbors: Vec<CellOut>,
}

#[derive(Serialize)]
struct CellOut {
    grid: [i32; 2],
    xy_mm: [f64; 2],
    blocked: bool,
    via_blocked: bool,
    nearest_foreign: Vec<OwnerOut>,
}

#[derive(Serialize)]
struct OwnerOut {
    item_id_run_local: i32,
    kind: String,
    net: String,
    component: Option<String>,
    pad: Option<String>,
    shape_distance_mm: f64,
    required_margin_mm: f64,
    within_track_margin_plus_cell_tolerance: bool,
}

fn pin_parts(board: &BoardAdapter, key: ItemKey) -> Option<(String, String)> {
    let item = board.board.item(key);
    let pin = item.as_pin()?;
    let component = board.board.components.get(item.component_no());
    let package = board.board.library.packages.get(component.get_package());
    let pin_name = package.get_pin(pin.pin_index)?.name.clone();
    Some((component.name.clone(), pin_name))
}

fn pin_ref(board: &BoardAdapter, key: ItemKey) -> Option<String> {
    let (component, pin) = pin_parts(board, key)?;
    Some(format!(
        "{}-{}",
        component.to_ascii_lowercase(),
        pin.to_ascii_lowercase()
    ))
}

fn net_route_layers(board: &BoardAdapter, net: NetNo) -> Vec<(i32, String)> {
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
        Err("DSN has no native via copper on selected layer".into())
    }
}

fn native_terminal_points(board: &BoardAdapter, key: ItemKey) -> Vec<fr_geom::Point> {
    let item = board.board.item(key);
    match &item.kind {
        ItemKind::Pin(_) | ItemKind::Via(_) => vec![item.center(&board.board)],
        ItemKind::Trace(_) => vec![item.first_corner(), item.last_corner()],
        ItemKind::ConductionArea(_) => board
            .board
            .ratsnest_corners(key)
            .into_iter()
            .filter(|point| {
                let first = item.first_layer(&board.board);
                let last = item.last_layer(&board.board);
                (0..board.board.tile_shape_count(key)).any(|i| {
                    let layer = board.board.shape_layer(key, i as i32);
                    layer >= first
                        && layer <= last
                        && board
                            .board
                            .tile_shape(key, i as i32)
                            .is_some_and(|shape| shape.contains(point))
                })
            })
            .collect(),
        _ => Vec::new(),
    }
}

fn candidates(
    board: &BoardAdapter,
    group: &fr_engine::board::ItemSet,
    layers: &[(i32, String)],
) -> (usize, BTreeMap<String, usize>) {
    let mut seen = BTreeSet::new();
    let mut by_layer = BTreeMap::<String, usize>::new();
    for member in group.iter() {
        let item = board.board.item(member);
        for point in native_terminal_points(board, member) {
            let p = point.to_float();
            for (layer_no, name) in layers {
                if item.is_on_layer(&board.board, *layer_no)
                    && seen.insert((p.x.to_bits(), p.y.to_bits(), name.clone()))
                {
                    *by_layer.entry(name.clone()).or_default() += 1;
                }
            }
        }
    }
    (seen.len(), by_layer)
}

fn group_pin_count(board: &BoardAdapter, group: &fr_engine::board::ItemSet) -> usize {
    group
        .iter()
        .filter(|&key| board.board.item(key).is_pin())
        .count()
}

fn item_output(board: &BoardAdapter, key: ItemKey) -> Result<ItemOut, String> {
    let item = board.board.item(key);
    let (component, pad) = pin_parts(board, key).ok_or("selected item is not a pin")?;
    let center = item.center(&board.board).to_float();
    let copper = item
        .padstack(&board.board)
        .ok_or("pin has no native padstack")?;
    let copper = (0..board.board.layer_count() as i32)
        .filter_map(|layer| {
            copper.get_shape(layer).map(|shape| {
                let bounds = shape.bounding_box();
                CopperOut {
                    layer: board.board.layer_structure.layers[layer as usize]
                        .name
                        .clone(),
                    shape_kind: format!("{:?}", shape),
                    width_mm: bounds.width() as f64 / board.units_per_mm,
                    height_mm: bounds.height() as f64 / board.units_per_mm,
                }
            })
        })
        .collect();
    Ok(ItemOut {
        item_id_run_local: item.id().0,
        reference: component,
        pad,
        xy_mm: [center.x / board.units_per_mm, center.y / board.units_per_mm],
        copper,
    })
}

fn owners_at(
    board: &BoardAdapter,
    net: NetNo,
    route_class: fr_engine::ids::ClearanceClassNo,
    layer_no: i32,
    point: FloatPoint,
    track_width_mm: f64,
    clearance_floor_mm: f64,
    grid_step_mm: f64,
) -> Vec<OwnerOut> {
    let mut rows = Vec::new();
    for key in board.board.get_items() {
        let item = board.board.item(key);
        if !item.is_on_board() || item.net_numbers().contains(&net) {
            continue;
        }
        let kind = item.board_item_type();
        let mut best = f64::INFINITY;
        for (i, shape) in board.board.item_tile_shapes(item).iter().enumerate() {
            if board.board.shape_layer(key, i as i32) != layer_no {
                continue;
            }
            if let Some(shape) = shape {
                best = best.min(shape.distance(&point) / board.units_per_mm);
            }
        }
        if !best.is_finite() {
            continue;
        }
        let pair_clearance =
            board
                .board
                .clearance_value(route_class, item.clearance_class(), layer_no) as f64
                / board.units_per_mm;
        let margin = pair_clearance.max(clearance_floor_mm) + track_width_mm / 2.0;
        let within = best <= margin + grid_step_mm * std::f64::consts::FRAC_1_SQRT_2;
        if within {
            let (component, pad) =
                pin_parts(board, key).map_or((None, None), |(a, b)| (Some(a), Some(b)));
            let net_name = item
                .net_numbers()
                .first()
                .and_then(|n| board.board.rules.nets.get(*n))
                .map(|r| r.name.clone())
                .unwrap_or_default();
            rows.push(OwnerOut {
                item_id_run_local: item.id().0,
                kind: format!("{kind:?}"),
                net: net_name,
                component,
                pad,
                shape_distance_mm: best,
                required_margin_mm: margin,
                within_track_margin_plus_cell_tolerance: within,
            });
        }
    }
    rows.sort_by(|a, b| a.shape_distance_mm.total_cmp(&b.shape_distance_mm));
    rows.truncate(4);
    rows
}

fn run() -> Result<PathBuf, Box<dyn std::error::Error>> {
    let mut args = env::args_os().skip(1);
    let dsn = PathBuf::from(
        args.next()
            .ok_or("usage: nopath_gnd <board.dsn> <final28.ses> <out_dir>")?,
    );
    let ses = PathBuf::from(args.next().ok_or("missing final28.ses")?);
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
            let layer = net_route_layers(&source, net.net_number)
                .first()
                .map(|(i, _)| *i)
                .ok_or_else(|| format!("net {} has no active route layer", net.name))?;
            Ok((
                net.name.clone(),
                via_size_for_layer(&source, layer)?,
                VIA_DRILL_MM,
            ))
        })
        .collect::<Result<Vec<_>, String>>()
        .map_err(std::io::Error::other)?;
    let board =
        BoardAdapter::load_session_with_via_candidates(&dsn_bytes, &ses_bytes, &via_candidates)
            .map_err(std::io::Error::other)?;
    let net = board.net_number("GND")?;
    let layers = net_route_layers(&board, net);
    let selected = board
        .pads()
        .into_iter()
        .find(|pad| pad.reference.to_ascii_lowercase().replace('.', "-") == PIN_REF)
        .ok_or_else(|| format!("stable pin ref {PIN_REF} absent from final28 SES board"))?;
    let key = selected.key.ok_or("selected pin has no native key")?;
    let group = board.board.connected_set(key, net, false);
    let pin_id = board.board.item(key).id();
    let (candidate_count, candidate_layers) = candidates(&board, &group, &layers);
    let incompletes =
        NetIncompletes::new(&board.board, net, &board.board.get_connectable_items(net));
    let mut gaps = Vec::new();
    let mut opposite_groups = Vec::new();
    for line in &incompletes.incompletes {
        let from_group = board.board.connected_set(line.from_item, net, false);
        let to_group = board.board.connected_set(line.to_item, net, false);
        let from_contains = from_group.contains(pin_id);
        let to_contains = to_group.contains(pin_id);
        if !from_contains && !to_contains {
            continue;
        }
        let from = board.board.item(line.from_item);
        let to = board.board.item(line.to_item);
        let (from_candidates, _) = candidates(&board, &from_group, &layers);
        let (to_candidates, _) = candidates(&board, &to_group, &layers);
        let from_point = line.from_corner;
        let to_point = line.to_corner;
        gaps.push(GapOut {
            from_item_id_run_local: from.id().0,
            to_item_id_run_local: to.id().0,
            from_xy_mm: [
                from_point.x / board.units_per_mm,
                from_point.y / board.units_per_mm,
            ],
            to_xy_mm: [
                to_point.x / board.units_per_mm,
                to_point.y / board.units_per_mm,
            ],
            from_group_pin_count: group_pin_count(&board, &from_group),
            to_group_pin_count: group_pin_count(&board, &to_group),
            from_group_candidate_count: from_candidates,
            to_group_candidate_count: to_candidates,
            from_group_contains_probe_pin: from_contains,
            to_group_contains_probe_pin: to_contains,
        });
        opposite_groups.push(if from_contains { to_group } else { from_group });
    }
    if gaps.is_empty() {
        return Err("selected stable pin has no current GND incomplete in final28 SES".into());
    }

    let (route_layer_no, route_layer) = layers
        .iter()
        .find(|(_, name)| name == "TopLayer")
        .or_else(|| layers.first())
        .ok_or("GND has no active signal layer")?;
    let net_class = board.board.rules.net_class_of(net);
    let track_width_mm =
        2.0 * net_class.get_trace_half_width(*route_layer_no) as f64 / board.units_per_mm;
    let clearance_mm = board.board.clearance_value(
        net_class.get_trace_clearance_class(),
        net_class.get_trace_clearance_class(),
        *route_layer_no,
    ) as f64
        / board.units_per_mm;
    let via_size_mm = via_size_for_layer(&board, *route_layer_no)?;
    let clearance_by_layer_mm: BTreeMap<String, f64> = layers
        .iter()
        .map(|(i, name)| {
            (
                name.clone(),
                board.board.clearance_value(
                    net_class.get_trace_clearance_class(),
                    net_class.get_trace_clearance_class(),
                    *i,
                ) as f64
                    / board.units_per_mm,
            )
        })
        .collect();
    let rules = RoutingRules {
        grid_step: GRID_STEP_MM,
        track_width: track_width_mm,
        clearance: clearance_mm,
        via_size: via_size_mm,
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
        net_via_sizes: BTreeMap::from([(net, [via_size_mm, VIA_DRILL_MM])]),
    };
    let center = board.board.item(key).center(&board.board).to_float();
    let point_mm = [center.x / board.units_per_mm, center.y / board.units_per_mm];
    let half = 4.0;
    let bb = board.board.bounding_box;
    let window = IntBox::new(
        (((point_mm[0] - half) * board.units_per_mm).floor() as i32).max(bb.ll.x),
        (((point_mm[1] - half) * board.units_per_mm).floor() as i32).max(bb.ll.y),
        (((point_mm[0] + half) * board.units_per_mm).ceil() as i32).min(bb.ur.x),
        (((point_mm[1] + half) * board.units_per_mm).ceil() as i32).min(bb.ur.y),
    );
    let mut request = RungRequest {
        board: board.clone(),
        net: "GND".into(),
        net_id: net,
        stage: "nopath_gnd_local_map".into(),
        start: RungEndpoint {
            point: point_mm,
            layer: route_layer.clone(),
        },
        end: RungEndpoint {
            point: point_mm,
            layer: route_layer.clone(),
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
        existing_vias: Vec::new(),
        paste_pads: BTreeMap::new(),
        rules,
        fabrication: FabTier::Standard,
        escalation: EscalationPolicy::Off,
        fabrication_overrides: BTreeMap::new(),
        fab_clearance_floor_mm: 0.0,
        fab_track_floor_mm: 0.0,
        may_narrow: false,
        fab_via_rungs: vec![(via_size_mm, VIA_DRILL_MM)],
        max_iterations: 200_000,
        route_layers: layers.iter().map(|(_, name)| name.clone()).collect(),
        deadline: None,
    };
    request.rules.validate()?;
    let grid_step_board = (GRID_STEP_MM * board.units_per_mm).round() as i32;
    let class = board
        .board
        .rules
        .net_class_of(net)
        .get_trace_clearance_class();
    let obs = native_obstacles::build_obstacle_map(
        &board,
        &request,
        window,
        grid_step_board,
        &request.route_layers,
        &[net],
        class,
        class,
    )
    .map_err(std::io::Error::other)?;
    let gx = native_obstacles::grid_round(center.x.round_ties_even() as i32, grid_step_board);
    let gy = native_obstacles::grid_round(center.y.round_ties_even() as i32, grid_step_board);
    let mut center_by_layer = Vec::new();
    let mut neighbors_by_layer = Vec::new();
    for (index, layer_name) in request.route_layers.iter().enumerate() {
        let layer_no = obs.layer_numbers[index];
        let point_at = |x: i32, y: i32| {
            FloatPoint::new(
                x as f64 * grid_step_board as f64,
                y as f64 * grid_step_board as f64,
            )
        };
        let cell_out = |x: i32, y: i32| {
            let point = point_at(x, y);
            CellOut {
                grid: [x, y],
                xy_mm: [point.x / board.units_per_mm, point.y / board.units_per_mm],
                blocked: obs.obstacles.is_blocked(x, y, index),
                via_blocked: obs.obstacles.is_via_blocked(x, y),
                nearest_foreign: owners_at(
                    &board,
                    net,
                    class,
                    layer_no,
                    point,
                    track_width_mm,
                    clearance_by_layer_mm[layer_name],
                    GRID_STEP_MM,
                ),
            }
        };
        center_by_layer.push(cell_out(gx, gy));
        let mut neighbors = Vec::new();
        for (dx, dy) in [
            (-1, -1),
            (0, -1),
            (1, -1),
            (-1, 0),
            (1, 0),
            (-1, 1),
            (0, 1),
            (1, 1),
        ] {
            neighbors.push(cell_out(gx + dx, gy + dy));
        }
        let blocked_count = neighbors.iter().filter(|c| c.blocked).count();
        neighbors_by_layer.push(LayerNeighborsOut {
            layer: layer_name.clone(),
            blocked_count,
            open_count: 8 - blocked_count,
            neighbors,
        });
    }
    let pad_shape = board
        .board
        .item(key)
        .drill_shape_on_layer(&board.board, *route_layer_no)
        .ok_or("selected pin has no copper shape on launch layer")?;
    let pad_bounds = pad_shape.bounding_box();
    let x0 = native_obstacles::grid_round(pad_bounds.ll.x, grid_step_board).saturating_sub(1);
    let y0 = native_obstacles::grid_round(pad_bounds.ll.y, grid_step_board).saturating_sub(1);
    let x1 = native_obstacles::grid_round(pad_bounds.ur.x, grid_step_board).saturating_add(1);
    let y1 = native_obstacles::grid_round(pad_bounds.ur.y, grid_step_board).saturating_add(1);
    let mut exact_copper_grid_cells = 0;
    let mut via_blocked_cells = 0;
    let mut via_open_cells = Vec::new();
    for cell_x in x0..=x1 {
        for cell_y in y0..=y1 {
            let point = Point::Int(IntPoint::new(
                cell_x.saturating_mul(grid_step_board),
                cell_y.saturating_mul(grid_step_board),
            ));
            if !pad_shape.contains(&point) {
                continue;
            }
            exact_copper_grid_cells += 1;
            if obs.obstacles.is_via_blocked(cell_x, cell_y) {
                via_blocked_cells += 1;
            } else {
                via_open_cells.push([cell_x, cell_y]);
            }
        }
    }
    let via_in_pad_probe = ViaInPadOut {
        exact_copper_grid_cells,
        via_blocked_cells,
        via_open_cells,
        via_keepout_margin_mm: via_size_mm / 2.0 + clearance_mm,
    };
    let report = Report {
        input_dsn: dsn.clone(), input_ses: ses.clone(), net: "GND".into(), pin_ref: PIN_REF.into(),
        pin: item_output(&board, key)?, connected_pin_count: group_pin_count(&board, &group),
        connected_group_candidate_count: candidate_count, connected_group_candidates_by_layer: candidate_layers,
        relevant_gaps: gaps, grid_step_mm: GRID_STEP_MM, track_width_mm, clearance_by_layer_mm,
        via_diameter_mm: via_size_mm, via_drill_mm: VIA_DRILL_MM,
        local_grid: LocalGridOut { probe_cell: [gx,gy], center_by_layer, neighbors_by_layer },
        via_in_pad_probe,
        obstacle_semantics: "Native map built from final28 SES with source GND net excluded; owner rows are geometric candidates based on exact native tile-shape distance plus route half-width/resolved clearance and cell tolerance, not per-item raster attribution. Exact is_blocked/is_via_blocked values come from the composed native maps. Same-net pad clearance is zero, no extra foreign margin. Endpoint overrides are not applied to this local neighbor map.",
    };
    fs::create_dir_all(&out_dir)?;
    let path = out_dir.join("nopath_gnd.json");
    fs::write(&path, serde_json::to_vec_pretty(&report)?)?;
    Ok(path)
}

fn main() {
    match run() {
        Ok(path) => println!("{}", path.display()),
        Err(error) => {
            eprintln!("nopath_gnd: {error}");
            std::process::exit(1);
        }
    }
}
