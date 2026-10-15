use fr_agent::{
    awx::{
        braid_connect::{rung_specs, LadderMode, RungEndpoint, RungRequest},
        fab_rules::{EscalationPolicy, FabTier},
        native_obstacles,
        whole_context::RoutingRules,
    },
    board_adapter::BoardAdapter,
};
use fr_engine::board::BoardItemType;
use fr_geom::{FloatPoint, IntBox, IntPoint, Point, Shape};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, env, fs, path::PathBuf};

#[path = "../src/awx/native_pad_raster.rs"]
mod native_pad_raster;

const REQUESTED_GRID_MM: f64 = 0.25;
const VIA_DRILL_MM: f64 = 0.3;

#[derive(Serialize)]
struct Scene {
    schema_version: u32,
    source_dsn: SourceFile,
    source_ses: SourceFile,
    units_per_mm: f64,
    nets: Vec<NetOut>,
    layers: Vec<LayerOut>,
    pads: Vec<PadOut>,
    tracks: Vec<TrackOut>,
    vias: Vec<ViaOut>,
    obstacles: Vec<ObstacleAreaOut>,
    outline: OutlineOut,
    imported_rules: ImportedRules,
}

#[derive(Serialize)]
struct SourceFile {
    path: PathBuf,
    sha256: String,
}

#[derive(Serialize)]
struct NetOut {
    id: i32,
    name: String,
    class_name: String,
}

#[derive(Serialize)]
struct LayerOut {
    native_index: i32,
    name: String,
    is_signal: bool,
}

#[derive(Serialize)]
struct PadOut {
    id: i32,
    reference: String,
    pin: String,
    net_ids: Vec<i32>,
    center_mm: [f64; 2],
    clearance_class: i32,
    copper_shapes: Vec<ShapeOut>,
    drill_mm: Option<f64>,
}

#[derive(Serialize)]
struct ShapeOut {
    layer: String,
    kind: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    center_mm: Option<[f64; 2]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    radius_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    points_mm: Option<Vec<[f64; 2]>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    recognized_rect: Option<RecognizedRect>,
    #[serde(skip_serializing_if = "Option::is_none")]
    native_shape_type: Option<&'static str>,
}

#[derive(Serialize)]
struct RecognizedRect {
    center_mm: [f64; 2],
    width_mm: f64,
    height_mm: f64,
    rotation_deg: f64,
    corner_radius_mm: f64,
}

#[derive(Serialize)]
struct TrackOut {
    id: i32,
    net_ids: Vec<i32>,
    layer: String,
    width_mm: f64,
    points_mm: Vec<[f64; 2]>,
}

#[derive(Serialize)]
struct ViaOut {
    id: i32,
    net_ids: Vec<i32>,
    center_mm: [f64; 2],
    diameter_mm: f64,
    drill_mm: Option<f64>,
    layers: Vec<String>,
}

#[derive(Serialize)]
struct ObstacleAreaOut {
    id: i32,
    kind: String,
    net_ids: Vec<i32>,
    clearance_class: i32,
    layers: Vec<String>,
    tile_polygons_mm: Vec<Vec<[f64; 2]>>,
    tracks_allowed: bool,
    vias_allowed: bool,
    copper_pour_allowed: bool,
    in_footprint: bool,
}

#[derive(Serialize)]
struct OutlineOut {
    points_mm: Vec<Vec<[f64; 2]>>,
}

#[derive(Serialize)]
struct ImportedRules {
    source: &'static str,
    units_per_mm: f64,
    classes: Vec<ClearanceClassOut>,
    matrix: Vec<ClearancePairOut>,
    net_classes: Vec<NetClassOut>,
}

#[derive(Serialize)]
struct ClearanceClassOut {
    id: i32,
    name: String,
}

#[derive(Serialize)]
struct ClearancePairOut {
    class_a: i32,
    class_b: i32,
    layer: String,
    native_value: i32,
    imported_value_mm: f64,
}

#[derive(Serialize)]
struct NetClassOut {
    name: String,
    trace_clearance_class: i32,
    active_layers: Vec<String>,
    trace_widths_mm: BTreeMap<String, f64>,
}

#[derive(Serialize)]
struct GridsFile {
    source_dsn_sha256: String,
    source_ses_sha256: String,
    scenes: Vec<GridScene>,
}

#[derive(Serialize)]
struct GridScene {
    name: String,
    net_id: i32,
    grid_step_mm_actual: f64,
    requested_grid_step_mm: f64,
    /// Inclusive cells being compared with original AWX.
    grid_window: [i32; 4],
    /// Larger native build window so `fence_window` cannot create a border inside the query.
    build_window_grid: [i32; 4],
    layers: Vec<String>,
    routing_rules: RoutingRules,
    resolved_class_pairs: Vec<ResolvedPair>,
    track_blocked: Vec<[i32; 3]>,
    via_blocked: Vec<[i32; 2]>,
}

#[derive(Serialize)]
struct ResolvedPair {
    route_class: i32,
    item_class: i32,
    layer: String,
    imported_matrix_mm: f64,
    effective_helper_clearance_mm: f64,
    native_build_clearance_mm: f64,
}

struct SceneSpec {
    name: &'static str,
    net: &'static str,
    grid_window: [i32; 4],
}

fn sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn point_mm(point: fr_geom::FloatPoint, upm: f64) -> [f64; 2] {
    [point.x / upm, point.y / upm]
}

fn shape_out(shape: &Shape, layer: String, upm: f64) -> ShapeOut {
    match shape {
        Shape::Circle(circle) => ShapeOut {
            layer,
            kind: "circle",
            center_mm: Some([circle.center.x as f64 / upm, circle.center.y as f64 / upm]),
            radius_mm: Some(circle.radius as f64 / upm),
            points_mm: None,
            recognized_rect: None,
            native_shape_type: Some("circle"),
        },
        _ => {
            let rect = native_pad_raster::source_rectangle(shape, upm).map(|rect| RecognizedRect {
                center_mm: [rect.center_x_mm, rect.center_y_mm],
                width_mm: rect.half_width_mm * 2.0,
                height_mm: rect.half_height_mm * 2.0,
                rotation_deg: rect.rotation_deg,
                corner_radius_mm: rect.corner_radius_mm,
            });
            ShapeOut {
                layer,
                kind: "polygon",
                center_mm: None,
                radius_mm: None,
                points_mm: Some(
                    shape
                        .corner_approx_arr()
                        .iter()
                        .map(|p| [p.x / upm, p.y / upm])
                        .collect(),
                ),
                native_shape_type: Some(if rect.is_some() { "rect" } else { "custom" }),
                recognized_rect: rect,
            }
        }
    }
}

fn identity(board: &BoardAdapter, key: fr_engine::board::ItemKey) -> (String, String) {
    let item = board.board.item(key);
    if let Some(pin) = item.as_pin() {
        let component = board.board.components.get(item.component_no());
        let package = board.board.library.packages.get(component.get_package());
        let name = package
            .get_pin(pin.pin_index)
            .map(|p| p.name.clone())
            .unwrap_or_default();
        (component.name.clone(), name)
    } else {
        (String::new(), String::new())
    }
}

fn scene_data(
    board: &BoardAdapter,
    dsn: &PathBuf,
    dsn_bytes: &[u8],
    ses: &PathBuf,
    ses_bytes: &[u8],
) -> Result<Scene, String> {
    let b = &board.board;
    let upm = board.units_per_mm;
    let nets = b
        .rules
        .nets
        .iter()
        .map(|net| NetOut {
            id: net.net_number,
            name: net.name.clone(),
            class_name: b.rules.net_class_of(net.net_number).get_name().to_owned(),
        })
        .collect();
    let layers = b
        .layer_structure
        .layers
        .iter()
        .enumerate()
        .map(|(i, layer)| LayerOut {
            native_index: i as i32,
            name: layer.name.clone(),
            is_signal: layer.is_signal,
        })
        .collect();
    let mut pads = Vec::new();
    let mut tracks = Vec::new();
    let mut vias = Vec::new();
    let mut obstacle_areas = Vec::new();
    for key in b.get_items() {
        let item = b.item(key);
        let (reference, pin) = identity(board, key);
        if item.board_item_type() == BoardItemType::Pin {
            let copper_shapes = (item.first_layer(b)..=item.last_layer(b))
                .filter(|&layer| b.layer_structure.layers[layer as usize].is_signal)
                .filter_map(|layer| {
                    let shape = item.drill_shape_on_layer(b, layer)?;
                    Some(shape_out(
                        &shape,
                        b.layer_structure.layers[layer as usize].name.clone(),
                        upm,
                    ))
                })
                .collect();
            let drill_mm = item
                .padstack(b)
                .and_then(|stack| stack.get_explicit_drill_radius())
                .map(|radius| 2.0 * radius / upm);
            pads.push(PadOut {
                id: item.id().0,
                reference,
                pin,
                net_ids: item.net_numbers().to_vec(),
                center_mm: point_mm(item.center(b).to_float(), upm),
                clearance_class: item.clearance_class(),
                copper_shapes,
                drill_mm,
            });
        } else if item.board_item_type() == BoardItemType::Trace {
            let Some(trace) = item.as_trace() else {
                continue;
            };
            let points_mm = trace
                .polyline()
                .corner_approx_arr()
                .iter()
                .map(|p| [p.x / upm, p.y / upm])
                .collect();
            tracks.push(TrackOut {
                id: item.id().0,
                net_ids: item.net_numbers().to_vec(),
                layer: b.layer_structure.layers[trace.layer() as usize]
                    .name
                    .clone(),
                width_mm: 2.0 * trace.half_width() as f64 / upm,
                points_mm,
            });
        } else if item.board_item_type() == BoardItemType::Via {
            let center_mm = point_mm(item.center(b).to_float(), upm);
            let padstack = item.padstack(b);
            let diameter_mm = padstack
                .map(|stack| {
                    (item.first_layer(b)..=item.last_layer(b))
                        .filter_map(|layer| stack.get_shape(layer))
                        .map(|shape| shape.max_width() / upm)
                        .fold(0.0_f64, f64::max)
                })
                .unwrap_or_default();
            let drill_mm = padstack
                .and_then(|stack| stack.get_explicit_drill_radius())
                .map(|radius| 2.0 * radius / upm);
            let layer_names = (item.first_layer(b)..=item.last_layer(b))
                .map(|layer| b.layer_structure.layers[layer as usize].name.clone())
                .collect();
            vias.push(ViaOut {
                id: item.id().0,
                net_ids: item.net_numbers().to_vec(),
                center_mm,
                diameter_mm,
                drill_mm,
                layers: layer_names,
            });
        } else if let Some(area) = item.as_obstacle_area() {
            let pieces = area
                .get_area(b)
                .split_to_convex()
                .ok_or("native obstacle area cannot be decomposed")?;
            let layers = (item.first_layer(b)..=item.last_layer(b))
                .filter(|&layer| b.layer_structure.layers[layer as usize].is_signal)
                .map(|layer| b.layer_structure.layers[layer as usize].name.clone())
                .collect();
            obstacle_areas.push(ObstacleAreaOut {
                id: item.id().0,
                kind: format!("{:?}", item.board_item_type()),
                net_ids: item.net_numbers().to_vec(),
                clearance_class: item.clearance_class(),
                layers,
                tile_polygons_mm: pieces
                    .iter()
                    .map(|shape| {
                        shape
                            .corner_approx_arr()
                            .iter()
                            .map(|p| [p.x / upm, p.y / upm])
                            .collect()
                    })
                    .collect(),
                tracks_allowed: item.is_via_obstacle_area(),
                vias_allowed: item.is_wire_obstacle_area(),
                copper_pour_allowed: false,
                in_footprint: item.is_component_obstacle_area(),
            });
        }
    }
    let outline = OutlineOut {
        points_mm: board
            .board_outline_polygons
            .iter()
            .map(|ring| {
                ring.iter()
                    .map(|p| [p[0] as f64 / upm, p[1] as f64 / upm])
                    .collect()
            })
            .collect(),
    };
    let matrix = &b.rules.clearance_matrix;
    let classes = (0..matrix.get_class_count())
        .filter_map(|id| {
            matrix.get_name(id).map(|name| ClearanceClassOut {
                id,
                name: name.to_owned(),
            })
        })
        .collect::<Vec<_>>();
    let mut pairs = Vec::new();
    for a in 0..matrix.get_class_count() {
        for c in 0..matrix.get_class_count() {
            for (layer_no, layer) in b.layer_structure.layers.iter().enumerate() {
                let native_value = matrix.get_value(a, c, layer_no as i32, false);
                pairs.push(ClearancePairOut {
                    class_a: a,
                    class_b: c,
                    layer: layer.name.clone(),
                    native_value,
                    imported_value_mm: native_value as f64 / upm,
                });
            }
        }
    }
    let net_classes = b
        .rules
        .nets
        .iter()
        .map(|net| {
            let class = b.rules.net_class_of(net.net_number);
            let mut widths = BTreeMap::new();
            let mut active = Vec::new();
            for (layer_no, layer) in b.layer_structure.layers.iter().enumerate() {
                if class.is_active_routing_layer(layer_no as i32) {
                    active.push(layer.name.clone());
                }
                widths.insert(
                    layer.name.clone(),
                    2.0 * class.get_trace_half_width(layer_no as i32) as f64 / upm,
                );
            }
            NetClassOut {
                name: class.get_name().to_owned(),
                trace_clearance_class: class.get_trace_clearance_class(),
                active_layers: active,
                trace_widths_mm: widths,
            }
        })
        .collect();
    Ok(Scene {
        schema_version: 1,
        source_dsn: SourceFile {
            path: dsn.clone(),
            sha256: sha256(dsn_bytes),
        },
        source_ses: SourceFile {
            path: ses.clone(),
            sha256: sha256(ses_bytes),
        },
        units_per_mm: upm,
        nets,
        layers,
        pads,
        tracks,
        vias,
        obstacles: obstacle_areas,
        outline,
        imported_rules: ImportedRules {
            source: "Rust DSN import, not independent truth",
            units_per_mm: upm,
            classes,
            matrix: pairs,
            net_classes,
        },
    })
}

fn route_layers(board: &BoardAdapter, net_id: i32) -> Vec<String> {
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

fn via_size_for_net(board: &BoardAdapter, net_id: i32) -> Result<f64, String> {
    let class = board.board.rules.net_class_of(net_id);
    let via_rule = class
        .get_via_rule()
        .or_else(|| board.board.rules.via_rules.get_first())
        .ok_or_else(|| format!("net {net_id} has no native via rule"))?;
    let mut diameter = 0.0_f64;
    for via_info in board.board.rules.via_rules[via_rule].vias() {
        let padstack_no = board.board.rules.via_infos[*via_info].get_padstack();
        let padstack = board
            .board
            .library
            .padstacks
            .get(padstack_no)
            .ok_or_else(|| format!("net {net_id} via padstack is missing"))?;
        for layer in 0..board.board.layer_structure.layers.len() as i32 {
            if let Some(shape) = padstack.get_shape(layer) {
                diameter = diameter.max(shape.max_width() / board.units_per_mm);
            }
        }
    }
    if diameter > 0.0 {
        Ok(diameter)
    } else {
        Err(format!("net {net_id} via rule has no copper shape"))
    }
}

fn rules_for(board: &BoardAdapter, net_id: i32) -> Result<RoutingRules, String> {
    let layers = route_layers(board, net_id);
    let first_layer_name = layers.first().ok_or("net has no active signal layer")?;
    let first_layer = board.board.layer_structure.get_no(first_layer_name);
    let class = board.board.rules.net_class_of(net_id);
    let trace_clearance_class = class.get_trace_clearance_class();
    let width = 2.0 * class.get_trace_half_width(first_layer) as f64 / board.units_per_mm;
    let clearance =
        board
            .board
            .clearance_value(trace_clearance_class, trace_clearance_class, first_layer)
            as f64
            / board.units_per_mm;
    let via_by_net = board
        .board
        .rules
        .nets
        .iter()
        .map(|n| {
            Ok((
                n.net_number,
                [via_size_for_net(board, n.net_number)?, VIA_DRILL_MM],
            ))
        })
        .collect::<Result<BTreeMap<_, _>, String>>()?;
    let [via_size, via_drill] = *via_by_net.get(&net_id).ok_or("net via size is missing")?;
    Ok(RoutingRules {
        grid_step: REQUESTED_GRID_MM,
        track_width: width,
        clearance,
        via_size,
        via_drill,
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
        net_via_sizes: via_by_net,
    })
}

fn make_request(
    board: &BoardAdapter,
    net_id: i32,
    rules: RoutingRules,
    layers: Vec<String>,
) -> Result<RungRequest, String> {
    let net = board
        .board
        .rules
        .nets
        .get(net_id)
        .map(|n| n.name.clone())
        .unwrap_or_default();
    let center = board
        .pads()
        .into_iter()
        .find(|p| p.net == net)
        .map(|p| p.point)
        .unwrap_or([0.0, 0.0]);
    let endpoint = RungEndpoint {
        point: center,
        layer: layers[0].clone(),
    };
    let existing_vias =
        fr_agent::awx::board_snapshot::BoardSnapshot::capture_vias(board, &BTreeMap::new())?;
    Ok(RungRequest {
        board: board.clone(),
        net: net.clone(),
        net_id,
        stage: "awx_parity_export".into(),
        start: endpoint.clone(),
        end: endpoint,
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
        existing_vias,
        paste_pads: BTreeMap::new(),
        rules,
        fabrication: FabTier::Standard,
        escalation: EscalationPolicy::Off,
        fabrication_overrides: BTreeMap::new(),
        fab_clearance_floor_mm: 0.0,
        fab_track_floor_mm: 0.0,
        may_narrow: false,
        fab_via_rungs: Vec::new(),
        max_iterations: 1,
        route_layers: layers,
        deadline: None,
    })
}

fn grids(
    board: &BoardAdapter,
    specs: &[SceneSpec],
    dsn_hash: &str,
    ses_hash: &str,
) -> Result<GridsFile, String> {
    let upm = board.units_per_mm;
    let mut scenes = Vec::new();
    for spec in specs {
        let net_id = board.net_number(spec.net)?;
        let rules = rules_for(board, net_id)?;
        let layers = route_layers(board, net_id);
        let request = make_request(board, net_id, rules.clone(), layers.clone())?;
        let step = (REQUESTED_GRID_MM * upm).round() as i32;
        let actual_step_mm = step as f64 / upm;
        let [x0, y0, x1, y1] = spec.grid_window;
        // `native_obstacles::build_obstacle_map` fences the edge of the supplied
        // window. Keep the queried region at least twelve cells inside that edge.
        const FENCE_GUARD_CELLS: i32 = 12;
        let build_window_grid = [
            x0.saturating_sub(FENCE_GUARD_CELLS),
            y0.saturating_sub(FENCE_GUARD_CELLS),
            x1.saturating_add(FENCE_GUARD_CELLS),
            y1.saturating_add(FENCE_GUARD_CELLS),
        ];
        let [bx0, by0, bx1, by1] = build_window_grid;
        let window = IntBox::new(
            bx0.saturating_mul(step),
            by0.saturating_mul(step),
            bx1.saturating_mul(step),
            by1.saturating_mul(step),
        );
        let route_class = board
            .board
            .rules
            .net_class_of(net_id)
            .get_trace_clearance_class();
        let map = native_obstacles::build_obstacle_map(
            board,
            &request,
            window,
            step,
            &layers,
            &[net_id],
            route_class,
            route_class,
        )?;
        let mut track_blocked = Vec::new();
        for gx in x0..=x1 {
            for gy in y0..=y1 {
                for layer_idx in 0..layers.len() {
                    if map.obstacles.is_blocked(gx, gy, layer_idx) {
                        track_blocked.push([gx, gy, layer_idx as i32]);
                    }
                }
            }
        }
        let mut via_blocked = Vec::new();
        for gx in x0..=x1 {
            for gy in y0..=y1 {
                if map.obstacles.is_via_blocked(gx, gy) {
                    via_blocked.push([gx, gy]);
                }
            }
        }
        let mut resolved_class_pairs = Vec::new();
        for layer in &layers {
            let layer_no = board.board.layer_structure.get_no(layer);
            for class_id in 0..board.board.rules.clearance_matrix.get_class_count() {
                let imported = board.board.rules.clearance_matrix.get_value(
                    route_class,
                    class_id,
                    layer_no,
                    false,
                );
                let effective = board.board.clearance_value(route_class, class_id, layer_no);
                let requested = (rules.clearance * upm).round() as i32;
                resolved_class_pairs.push(ResolvedPair {
                    route_class,
                    item_class: class_id,
                    layer: layer.clone(),
                    imported_matrix_mm: imported as f64 / upm,
                    effective_helper_clearance_mm: effective as f64 / upm,
                    native_build_clearance_mm: effective.max(requested).max(0) as f64 / upm,
                });
            }
        }
        scenes.push(GridScene {
            name: spec.name.into(),
            net_id,
            grid_step_mm_actual: actual_step_mm,
            requested_grid_step_mm: REQUESTED_GRID_MM,
            grid_window: spec.grid_window,
            build_window_grid,
            layers,
            routing_rules: rules,
            resolved_class_pairs,
            track_blocked,
            via_blocked,
        });
    }
    Ok(GridsFile {
        source_dsn_sha256: dsn_hash.to_owned(),
        source_ses_sha256: ses_hash.to_owned(),
        scenes,
    })
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = env::args_os().skip(1);
    let dsn = PathBuf::from(
        args.next()
            .ok_or("usage: awx_parity_export <board.dsn> <awx4_residual.ses> <out_dir>")?,
    );
    let ses = PathBuf::from(args.next().ok_or("missing frozen SES")?);
    let output_dir = PathBuf::from(args.next().ok_or("missing output directory")?);
    let dsn_bytes = fs::read(&dsn)?;
    let ses_bytes = fs::read(&ses)?;
    let source = BoardAdapter::load(&dsn_bytes).map_err(std::io::Error::other)?;
    let candidates = source
        .board
        .rules
        .nets
        .iter()
        .map(|n| {
            Ok((
                n.name.clone(),
                via_size_for_net(&source, n.net_number)?,
                VIA_DRILL_MM,
            ))
        })
        .collect::<Result<Vec<_>, String>>()?;
    let board = BoardAdapter::load_session_with_via_candidates(&dsn_bytes, &ses_bytes, &candidates)
        .map_err(std::io::Error::other)?;
    let scene = scene_data(&board, &dsn, &dsn_bytes, &ses, &ses_bytes)?;
    let specs = [
        SceneSpec {
            name: "GND",
            net: "GND",
            grid_window: [126, 100, 144, 118],
        },
        SceneSpec {
            name: "SD_MISO",
            net: "SD_MISO",
            grid_window: [180, 116, 196, 130],
        },
        SceneSpec {
            name: "USB_FS_DP",
            net: "USB_FS_DP",
            grid_window: [132, 196, 152, 212],
        },
        SceneSpec {
            name: "USB_FS_DM",
            net: "USB_FS_DM",
            grid_window: [132, 196, 152, 212],
        },
    ];
    let grid_file = grids(
        &board,
        &specs,
        &scene.source_dsn.sha256,
        &scene.source_ses.sha256,
    )?;
    fs::create_dir_all(&output_dir)?;
    fs::write(
        output_dir.join("scene.json"),
        serde_json::to_vec_pretty(&scene)?,
    )?;
    fs::write(
        output_dir.join("rust_grids.json"),
        serde_json::to_vec_pretty(&grid_file)?,
    )?;
    println!("{}", output_dir.display());
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("awx_parity_export: {error}");
        std::process::exit(1);
    }
}
