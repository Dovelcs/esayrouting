//! Independent DRC of the AWX KiCad output against the original four-layer DSN.
//!
//! Uses the repository's existing KiCad parser only to read route geometry. All copper is
//! inserted without autorouter legality checks, then the mature fr-engine DSN DRC evaluates the
//! unchanged source board rules and shapes. No sibling .kicad_pro/.kicad_dru is read.
//!
//! Usage: cargo run --release -p fr_agent --example awx_output_drc -- \
//!   <config.json> (the previous five positional arguments remain available)

use std::{
    collections::{HashMap, HashSet},
    env, fs,
    path::Path,
    process::Command,
    sync::Arc,
};

use fr_agent::board_adapter::BoardAdapter;
use fr_engine::{
    board::{BasicBoard, ItemKey, ItemKind},
    ids::{FixedState, NetNo},
    rules::default_item_clearance_classes::ItemClass,
};
use fr_geom::{Circle, ConvexShape, IntPoint, Point, Polyline};
use serde::Deserialize;
use serde_json::{json, Value};
use sha2::Digest;

const PARSER_EXPORT: &str = r#"
import json, sys
from kicad_parser import parse_kicad_pcb
p = parse_kicad_pcb(sys.argv[1])
print(json.dumps({
  'copper_layers': p.board_info.copper_layers,
  'nets': [{'id': n.net_id, 'name': n.name} for n in p.nets.values()],
  'segments': [{'start':[s.start_x,s.start_y], 'end':[s.end_x,s.end_y], 'width':s.width,
                'layer':s.layer, 'net':p.nets.get(s.net_id).name if p.nets.get(s.net_id) else '',
                'net_id':s.net_id, 'uuid':s.uuid} for s in p.segments],
  'vias': [{'at':[v.x,v.y], 'size':v.size, 'drill':v.drill,
            'layers':v.layers, 'net':p.nets.get(v.net_id).name if p.nets.get(v.net_id) else '',
            'net_id':v.net_id, 'uuid':v.uuid} for v in p.vias]
}, separators=(',', ':')))
"#;

#[derive(Debug, Deserialize)]
struct RouteExport {
    copper_layers: Vec<String>,
    segments: Vec<Segment>,
    vias: Vec<RouteVia>,
    nets: Vec<RouteNet>,
}
#[derive(Debug, Deserialize)]
struct Segment {
    start: [f64; 2],
    end: [f64; 2],
    width: f64,
    layer: String,
    net: String,
    net_id: i32,
    uuid: String,
}
#[derive(Debug, Deserialize)]
struct RouteVia {
    at: [f64; 2],
    size: f64,
    drill: f64,
    layers: Vec<String>,
    net: String,
    net_id: i32,
    uuid: String,
}
#[derive(Debug, Deserialize)]
struct RouteNet {
    id: i32,
    name: String,
}

#[derive(Debug, Deserialize)]
struct DrcConfig {
    dsn_path: std::path::PathBuf,
    pcb_path: std::path::PathBuf,
    scene_path: std::path::PathBuf,
    report_path: std::path::PathBuf,
    py_router_path: std::path::PathBuf,
    expected_output_sha256: String,
    rules_contract_path: Option<std::path::PathBuf>,
}

fn kicad_geometry(pcb: &Path, py_router: &Path) -> Result<RouteExport, Box<dyn std::error::Error>> {
    let workspace = Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(4)
        .ok_or("cannot determine workspace root from Cargo manifest")?;
    let mut paths = vec![
        py_router.to_path_buf(),
        py_router.join("grid"),
        workspace.join("outputs/fix_validation/awx_parity_runtime/pydeps"),
        workspace.join("outputs/fix_validation/awx_python_clean/pydeps"),
    ];
    if let Some(existing) = env::var_os("PYTHONPATH") {
        paths.extend(std::env::split_paths(&existing));
    }
    let pythonpath = std::env::join_paths(paths)?;
    let python = env::var_os("PYTHON").unwrap_or_else(|| "python3".into());
    let output = Command::new(python)
        .arg("-c")
        .arg(PARSER_EXPORT)
        .arg(pcb)
        .env("PYTHONPATH", pythonpath)
        .env("PYTHONDONTWRITEBYTECODE", "1")
        .output()?;
    if !output.status.success() {
        return Err(format!(
            "existing kicad_parser failed: {}",
            String::from_utf8_lossy(&output.stderr)
        )
        .into());
    }
    Ok(serde_json::from_slice(&output.stdout)?)
}

fn mm_to_native(units_per_mm: f64, mm: f64) -> i32 {
    (mm * units_per_mm).round() as i32
}
fn csv_field(value: &str) -> String {
    format!("\"{}\"", value.replace('"', "\"\""))
}
fn json_cell(value: &Value) -> String {
    match value {
        Value::Null => String::new(),
        Value::String(s) => s.clone(),
        _ => value.to_string(),
    }
}
fn item_kind(item: &fr_engine::board::Item) -> &'static str {
    match &item.kind {
        ItemKind::Pin(_) => "pin",
        ItemKind::Via(_) => "via",
        ItemKind::Trace(_) => "trace",
        ItemKind::ObstacleArea(_) => "keepout",
        ItemKind::ConductionArea(_) => "zone",
        ItemKind::ComponentOutline(_) => "component_outline",
        ItemKind::BoardOutline(_) => "board_outline",
    }
}
fn net_names(board: &BasicBoard, key: ItemKey) -> Vec<String> {
    board
        .item(key)
        .net_numbers()
        .iter()
        .filter_map(|n| board.rules.nets.get(*n).map(|x| x.name.clone()))
        .collect()
}
fn item_record(
    board: &BasicBoard,
    key: ItemKey,
    route_meta: &HashMap<i32, Value>,
    source_pin_drills: &HashMap<(String, String), Option<f64>>,
) -> Value {
    let item = board.item(key);
    let id = item.id().0;
    let class = item.clearance_class();
    let class_name = board.rules.clearance_matrix.get_name(class).unwrap_or("?");
    let mut value = json!({"native_item_id":id, "kind":item_kind(item), "nets":net_names(board,key), "clearance_class":{"index":class,"name":class_name}});
    if let Some(pin) = item.as_pin() {
        let component = board.components.get(item.component_no());
        let package = board.library.packages.get(component.get_package());
        let pin_name = package
            .get_pin(pin.pin_index)
            .map(|p| p.name.clone())
            .unwrap_or_default();
        let drill = source_pin_drills
            .get(&(component.name.clone(), pin_name.clone()))
            .copied()
            .flatten();
        let first_layer = item.first_layer(board);
        let last_layer = item.last_layer(board);
        let layer_name = |layer: i32| {
            board
                .layer_structure
                .layers
                .get(layer as usize)
                .map(|l| l.name.clone())
                .unwrap_or_default()
        };
        value["identity"] = json!({"reference":component.name,"pin":pin_name,"layer_span":{"first":first_layer,"first_name":layer_name(first_layer),"last":last_layer,"last_name":layer_name(last_layer)},"drill_known":drill.is_some(),"source_drill_mm":drill});
    }
    if let Some(meta) = route_meta.get(&id) {
        value["source_geometry"] = meta.clone();
    } else {
        let mut boxes = Vec::new();
        for i in 0..board.tile_shape_count(key) {
            if let Some(shape) = board.tile_shape(key, i) {
                let b = shape.bounding_box();
                let res = board
                    .communication
                    .get_resolution(fr_engine::structure::Unit::Mm);
                boxes.push(json!({"layer":board.layer_structure.layers.get(board.shape_layer(key,i) as usize).map(|l|l.name.clone()).unwrap_or_default(),
                    "bbox_mm":[b.ll.x as f64/res,b.ll.y as f64/res,b.ur.x as f64/res,b.ur.y as f64/res],
                    "engine_shape_vertices_mm":shape.corner_approx_arr().iter().map(|p|json!([p.x/res,p.y/res])).collect::<Vec<_>>()}));
            }
        }
        value["native_shape_bboxes"] = json!(boxes);
    }
    value
}
fn named_net(board: &BasicBoard, name: &str) -> Result<NetNo, Box<dyn std::error::Error>> {
    board
        .rules
        .nets
        .iter()
        .find(|n| n.name == name)
        .map(|n| n.net_number)
        .ok_or_else(|| format!("output geometry references unknown source net {name:?}").into())
}
fn native_layer(
    board: &BasicBoard,
    copper_names: &[String],
    name: &str,
) -> Result<i32, Box<dyn std::error::Error>> {
    let ix = copper_names
        .iter()
        .position(|x| x == name)
        .ok_or_else(|| format!("unknown output copper layer {name}"))?;
    let signal: Vec<_> = board
        .layer_structure
        .layers
        .iter()
        .enumerate()
        .filter(|(_, l)| l.is_signal)
        .map(|(i, _)| i as i32)
        .collect();
    signal
        .get(ix)
        .copied()
        .ok_or_else(|| format!("output layer index {ix} absent from original DSN stack").into())
}
fn scene_rectangle(scene: &Value) -> Result<[f64; 4], Box<dyn std::error::Error>> {
    let rings = scene["outline_polygons_mm"]
        .as_array()
        .ok_or("source scene has no outline polygons")?;
    if rings.len() != 1 {
        return Err(format!("expected one source outline ring, found {}", rings.len()).into());
    }
    let ring = rings[0]
        .as_array()
        .ok_or("source outline ring is not an array")?;
    if ring.len() != 4 {
        return Err(format!(
            "expected a four-corner rectangle, found {} corners",
            ring.len()
        )
        .into());
    }
    let points: Vec<[f64; 2]> = ring
        .iter()
        .map(|p| {
            let a = p.as_array().ok_or("outline point is not an array")?;
            if a.len() != 2 {
                return Err("outline point does not have two coordinates".into());
            }
            Ok([
                a[0].as_f64().ok_or("outline x is not numeric")?,
                a[1].as_f64().ok_or("outline y is not numeric")?,
            ])
        })
        .collect::<Result<_, Box<dyn std::error::Error>>>()?;
    let left = points.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min);
    let bottom = points.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min);
    let right = points
        .iter()
        .map(|p| p[0])
        .fold(f64::NEG_INFINITY, f64::max);
    let top = points
        .iter()
        .map(|p| p[1])
        .fold(f64::NEG_INFINITY, f64::max);
    if right <= left
        || top <= bottom
        || points
            .iter()
            .any(|p| !((p[0] == left || p[0] == right) && (p[1] == bottom || p[1] == top)))
    {
        return Err("source outline is not an axis-aligned rectangle".into());
    }
    Ok([left, bottom, right, top])
}

fn source_pin_drills(
    scene: &Value,
) -> Result<HashMap<(String, String), Option<f64>>, Box<dyn std::error::Error>> {
    let pins = scene["pins"]
        .as_array()
        .ok_or("source scene has no pin records")?;
    let mut result = HashMap::new();
    for pin in pins {
        let reference = pin["reference"]
            .as_str()
            .ok_or("source pin reference missing")?
            .to_string();
        let name = pin["pin"]
            .as_str()
            .ok_or("source pin name missing")?
            .to_string();
        result.insert((reference, name), pin["source_drill_mm"].as_f64());
    }
    Ok(result)
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = env::args_os().skip(1).collect();
    let workspace = Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(4)
        .ok_or("cannot determine workspace root from Cargo manifest")?;
    let (
        dsn_path,
        pcb_path,
        scene_path,
        report_path,
        py_router,
        expected_output_sha,
        rules_contract_path,
    ) = if args.len() == 1 {
        let config: DrcConfig = serde_json::from_slice(&fs::read(&args[0])?)?;
        (
            config.dsn_path,
            config.pcb_path,
            config.scene_path,
            config.report_path,
            config.py_router_path,
            config.expected_output_sha256,
            config.rules_contract_path.unwrap_or_else(|| {
                workspace.join("outputs/fix_validation/awx_original_drc/original_rules.json")
            }),
        )
    } else if args.len() == 5 {
        (
            args[0].clone().into(),
            args[1].clone().into(),
            args[2].clone().into(),
            args[3].clone().into(),
            args[4].clone().into(),
            "360077ad32cec696859b02c5ea7e42c869fd453ea7b24411d9e2ea190f374fa7".to_owned(),
            workspace.join("outputs/fix_validation/awx_original_drc/original_rules.json"),
        )
    } else {
        return Err("usage: awx_output_drc <config.json> | <board_4.dsn> <board_routed.kicad_pcb> <input_scene.json> <report.json> <py_router>".into());
    };
    let dsn_path = dsn_path.as_path();
    let pcb_path = pcb_path.as_path();
    let scene_path = scene_path.as_path();
    let report_path = report_path.as_path();
    let py_router = py_router.as_path();
    let dsn = fs::read(dsn_path)?;
    let scene: Value = serde_json::from_slice(&fs::read(scene_path)?)?;
    let source_sha = format!("{:x}", sha2::Sha256::digest(&dsn));
    if scene["source_dsn_sha256"].as_str() != Some(source_sha.as_str()) {
        return Err("input_scene source DSN hash does not match checker DSN".into());
    }
    let scene_counts = scene["native_base_counts"].clone();
    let expected_pins = scene_counts["pins"].as_u64().unwrap_or(0) as usize;
    let pcb_bytes = fs::read(pcb_path)?;
    let output_sha = format!("{:x}", sha2::Sha256::digest(&pcb_bytes));
    if output_sha != expected_output_sha {
        return Err(format!("routed output hash changed: {output_sha}").into());
    }
    let geometry = kicad_geometry(pcb_path, py_router)?;
    let output_readback_path = pcb_path
        .parent()
        .ok_or("output PCB has no parent directory")?
        .join("output_readback.json");
    let output_readback: Value = serde_json::from_slice(&fs::read(output_readback_path)?)?;
    let matched_pads = output_readback["scene_parser_readback"]["pad_identity_match_count"]
        .as_u64()
        .unwrap_or(0) as usize;
    let center_tolerance = output_readback["scene_parser_readback"]["center_tolerance_mm"]
        .as_f64()
        .unwrap_or(f64::INFINITY);
    let error_keys = [
        "missing_scene_pads",
        "extra_parsed_pads",
        "center_mismatches",
        "layer_mismatches",
        "shape_mismatches",
        "net_membership_mismatches",
        "polygon_mismatches",
    ];
    let readback_errors = error_keys.iter().any(|k| {
        output_readback["errors"][k]
            .as_array()
            .map_or(true, |a| !a.is_empty())
    });
    if matched_pads != expected_pins || center_tolerance > 0.000001 || readback_errors {
        return Err(
            "frozen parser readback does not prove every source pad identity/center alignment"
                .into(),
        );
    }
    if geometry.segments.len()
        != output_readback["parser_counts"]["tracks"]
            .as_u64()
            .unwrap_or(u64::MAX) as usize
        || geometry.vias.len()
            != output_readback["parser_counts"]["vias"]
                .as_u64()
                .unwrap_or(u64::MAX) as usize
    {
        return Err("route geometry export counts disagree with frozen parser readback".into());
    }
    let pin_drill_map = source_pin_drills(&scene)?;
    let unknown_pin_drills = pin_drill_map.values().filter(|v| v.is_none()).count();
    let outline_rect = scene_rectangle(&scene)?;
    let mut outside_copper = Vec::new();
    for s in &geometry.segments {
        let half = s.width / 2.0;
        let bounds = [
            s.start[0].min(s.end[0]) - half,
            s.start[1].min(s.end[1]) - half,
            s.start[0].max(s.end[0]) + half,
            s.start[1].max(s.end[1]) + half,
        ];
        if bounds[0] < outline_rect[0]
            || bounds[1] < outline_rect[1]
            || bounds[2] > outline_rect[2]
            || bounds[3] > outline_rect[3]
        {
            outside_copper.push(json!({"kind":"trace_capsule","uuid":s.uuid,"net":s.net,"layer":s.layer,"start_mm":s.start,"end_mm":s.end,"width_mm":s.width,"copper_bbox_mm":bounds}));
        }
    }
    for v in &geometry.vias {
        let radius = v.size / 2.0;
        let bounds = [
            v.at[0] - radius,
            v.at[1] - radius,
            v.at[0] + radius,
            v.at[1] + radius,
        ];
        if bounds[0] < outline_rect[0]
            || bounds[1] < outline_rect[1]
            || bounds[2] > outline_rect[2]
            || bounds[3] > outline_rect[3]
        {
            outside_copper.push(json!({"kind":"via_circle","uuid":v.uuid,"net":v.net,"layers":v.layers,"center_mm":v.at,"diameter_mm":v.size,"copper_bbox_mm":bounds}));
        }
    }
    let containment = json!({"outline_rect_mm":outline_rect,"method":"exact swept-circle/capsule bounding extrema against source axis-aligned rectangle; no edge setback applied","checked_traces":geometry.segments.len(),"checked_vias":geometry.vias.len(),"outside_copper_items":outside_copper});
    let route_geometry_path = report_path
        .parent()
        .ok_or("report has no parent directory")?
        .join("route_geometry.json");
    let route_geometry = json!({"schema":1,"source_output_pcb":pcb_path,"source_output_sha256":output_sha,"units":"mm","coordinate_frame":"KiCad parser readback, direct native x/y (no axis swap)","copper_layers":geometry.copper_layers,"nets":geometry.nets.iter().map(|n|json!({"id":n.id,"name":n.name})).collect::<Vec<_>>(),
        "tracks":geometry.segments.iter().map(|s|json!({"uuid":s.uuid,"net":s.net,"net_id":s.net_id,"layer":s.layer,"start":s.start,"end":s.end,"width_mm":s.width})).collect::<Vec<_>>(),
        "vias":geometry.vias.iter().map(|v|json!({"uuid":v.uuid,"net":v.net,"net_id":v.net_id,"at":v.at,"diameter_mm":v.size,"drill_mm":v.drill,"layers":v.layers})).collect::<Vec<_>>()});
    let mut source = BoardAdapter::load(&dsn).map_err(std::io::Error::other)?;
    let units_per_mm = source.units_per_mm;
    let board = &mut source.board;
    // Grade the source matrix with no tolerance added to its numeric threshold.
    let loaded_default_tolerance_um = board.rules.clearance_tolerance_um;
    Arc::make_mut(&mut board.rules).clearance_tolerance_um = 0.0;
    let signal_count = board
        .layer_structure
        .layers
        .iter()
        .filter(|l| l.is_signal)
        .count();
    if geometry.copper_layers.len() != signal_count {
        return Err(format!(
            "output has {} copper layers; DSN has {signal_count}",
            geometry.copper_layers.len()
        )
        .into());
    }
    let baseline_count = board.get_pins().len();
    if baseline_count != expected_pins {
        return Err(format!(
            "native pin count changed: scene {expected_pins}, live {}",
            baseline_count
        )
        .into());
    }
    let baseline = board.clone();
    let mut route_meta: HashMap<i32, Value> = HashMap::new();

    // Vias first, then traces. insert_via's same-net split therefore sees source copper only;
    // traces themselves are raw-inserted below, without cleaning, snapping or validity filters.
    for via in &geometry.vias {
        if via.layers.len() < 2 {
            return Err(
                format!("via {} has fewer than two declared copper layers", via.uuid).into(),
            );
        }
        let mut span: Vec<_> = via
            .layers
            .iter()
            .map(|n| native_layer(board, &geometry.copper_layers, n))
            .collect::<Result<_, _>>()?;
        span.sort_unstable();
        span.dedup();
        let from = *span.first().ok_or("via has no copper span")?;
        let to = *span.last().ok_or("via has no copper span")?;
        let net = named_net(board, &via.net)?;
        let center = Point::Int(IntPoint::new(
            mm_to_native(units_per_mm, via.at[0]),
            mm_to_native(units_per_mm, via.at[1]),
        ));
        let resolution = board
            .communication
            .get_resolution(fr_engine::structure::Unit::Mm);
        let radius = (via.size * resolution / 2.0).round() as i32;
        let circle = ConvexShape::Circle(Circle::new(IntPoint::new(0, 0), radius));
        let lib = Arc::make_mut(&mut board.library);
        let padstack = lib.padstacks.add_shape_range(&circle, from, to);
        lib.padstacks
            .get_mut(padstack)
            .ok_or("new via padstack missing")?
            .set_explicit_drill_radius(Some(via.drill * resolution / 2.0))?;
        let cc = board
            .rules
            .net_class_of(net)
            .default_item_clearance_classes
            .get(ItemClass::Via);
        let key = board.insert_via(padstack, center, &[net], cc, FixedState::Unfixed, true);
        let id = board.item(key).id().0;
        let layer_names: Vec<_> = span
            .iter()
            .filter_map(|n| {
                board
                    .layer_structure
                    .layers
                    .get(*n as usize)
                    .map(|l| l.name.clone())
            })
            .collect();
        route_meta.insert(id, json!({"source_uuid":via.uuid,"kind":"via","center_mm":via.at,"diameter_mm":via.size,"drill_mm":via.drill,"layers":layer_names,"net":via.net,"attach_allowed":board.item(key).as_via().map(|v|v.attach_allowed).unwrap_or(false)}));
    }
    for segment in &geometry.segments {
        let net = named_net(board, &segment.net)?;
        let layer = native_layer(board, &geometry.copper_layers, &segment.layer)?;
        let p0 = Point::Int(IntPoint::new(
            mm_to_native(units_per_mm, segment.start[0]),
            mm_to_native(units_per_mm, segment.start[1]),
        ));
        let p1 = Point::Int(IntPoint::new(
            mm_to_native(units_per_mm, segment.end[0]),
            mm_to_native(units_per_mm, segment.end[1]),
        ));
        let width_native = segment.width
            * board
                .communication
                .get_resolution(fr_engine::structure::Unit::Mm);
        let half_width = (width_native / 2.0).round() as i32;
        let cc = board.rules.net_class_of(net).get_trace_clearance_class();
        let key = board
            .insert_trace_without_cleaning(
                Polyline::from_points(&[p0, p1]),
                layer,
                half_width,
                &[net],
                cc,
                FixedState::Unfixed,
            )
            .ok_or_else(|| format!("raw insertion skipped output track {}", segment.uuid))?;
        let id = board.item(key).id().0;
        route_meta.insert(id, json!({"source_uuid":segment.uuid,"kind":"trace","start_mm":segment.start,"end_mm":segment.end,"width_mm":segment.width,"layer":segment.layer,"net":segment.net}));
    }

    let base_drc = fr_engine::drc::all_clearance_violations(&baseline);
    let final_drc = fr_engine::drc::all_clearance_violations(board);
    let route_ids: HashSet<i32> = route_meta.keys().copied().collect();
    let mut violation_rows = Vec::new();
    for v in &final_drc {
        let first = board.item(v.first_item);
        let second = board.item(v.second_item);
        let b = v.shape.bounding_box();
        let res = board
            .communication
            .get_resolution(fr_engine::structure::Unit::Mm);
        let involved_route =
            route_ids.contains(&first.id().0) || route_ids.contains(&second.id().0);
        let expected_mm = v.expected_clearance / res;
        let layer_name = board
            .layer_structure
            .layers
            .get(v.layer as usize)
            .map(|l| l.name.clone())
            .unwrap_or_else(|| format!("layer#{}", v.layer));
        let class_a = first.clearance_class();
        let class_b = second.clearance_class();
        let class_name_a = board
            .rules
            .clearance_matrix
            .get_name(class_a)
            .unwrap_or("?");
        let class_name_b = board
            .rules
            .clearance_matrix
            .get_name(class_b)
            .unwrap_or("?");
        let nets_a = net_names(board, v.first_item);
        let nets_b = net_names(board, v.second_item);
        let known_nets_a = !nets_a.is_empty();
        let known_nets_b = !nets_b.is_empty();
        let shares_net = nets_a.iter().any(|n| nets_b.contains(n));
        let via_pin = if first.is_via() && second.is_pin() {
            Some((v.first_item, v.second_item))
        } else if first.is_pin() && second.is_via() {
            Some((v.second_item, v.first_item))
        } else {
            None
        };
        let via_on_multilayer_pin = via_pin.is_some_and(|(via_key, pin_key)| {
            shares_net
                && board
                    .item(pin_key)
                    .as_pin()
                    .is_some_and(|_| !board.item(pin_key).drill_allowed(board))
                && board
                    .item(via_key)
                    .as_via()
                    .is_some_and(|via| via.attach_allowed)
        });
        let touching = v.actual_clearance <= 0.0;
        let on_boundary_or_keepout = first.is_board_outline()
            || second.is_board_outline()
            || first.is_obstacle_area()
            || second.is_obstacle_area();
        let violation_type = if via_on_multilayer_pin {
            "via_on_multilayer_pin"
        } else if on_boundary_or_keepout {
            "boundary_or_keepout_clearance_candidate"
        } else if touching && known_nets_a && known_nets_b && !shares_net {
            "short_contact"
        } else if touching && (!known_nets_a || !known_nets_b) {
            "netless_copper_contact"
        } else {
            "copper_spacing"
        };
        let clearance_shortfall_mm = expected_mm - v.actual_clearance / res;
        let shortfall_um = clearance_shortfall_mm * 1000.0;
        let is_structure_marker = via_on_multilayer_pin;
        let source_rule = if is_structure_marker {
            json!({"kind":"native_structural_obstacle_semantics","symbol":"fr_engine::board::Item::drill_allowed / Item::is_obstacle","semantics":"Pin.drill_allowed is first_layer==last_layer; same-net Via-vs-Pin remains obstacle when the pin spans multiple layers","via_attach_allowed":via_pin.and_then(|(via_key,_)|board.item(via_key).as_via().map(|v|v.attach_allowed)).unwrap_or(false),"source_drill_known":via_pin.map(|(_,pin_key)|{let pin=board.item(pin_key);let c=board.components.get(pin.component_no());let p=board.library.packages.get(c.get_package());let n=pin.as_pin().and_then(|x|p.get_pin(x.pin_index)).map(|x|x.name.clone()).unwrap_or_default();pin_drill_map.get(&(c.name.clone(),n)).copied().flatten().is_some()}).unwrap_or(false),"not_a_numeric_DSN_clearance_rule":true})
        } else {
            json!({"method":"source DSN rules loaded by BoardAdapter; raw clearance matrix get_value(class_a,class_b,layer,false)","class_pair":[{"index":class_a,"name":class_name_a},{"index":class_b,"name":class_name_b}],"effective_raw_matrix_value_mm":expected_mm})
        };
        violation_rows.push(json!({"type":violation_type,"layer":layer_name,
            "coordinates_mm":{"point":[(b.ll.x+b.ur.x) as f64/(2.0*res),(b.ll.y+b.ur.y) as f64/(2.0*res)],"meaning":"clearance violation-region marker center"},
            "net_names":[nets_a,nets_b],
            "items":[item_record(board,v.first_item,&route_meta,&pin_drill_map),item_record(board,v.second_item,&route_meta,&pin_drill_map)],
            "required_clearance_mm":if is_structure_marker {Value::Null} else {json!(expected_mm)},"actual_clearance_mm":if is_structure_marker {Value::Null} else {json!(v.actual_clearance/res)},
            "clearance_shortfall_um":if is_structure_marker {Value::Null} else {json!(shortfall_um)},"near_threshold_2um":if is_structure_marker {Value::Null} else {json!(shortfall_um<=2.0)},
            "sub_native_coordinate_unit_sensitive":!is_structure_marker && shortfall_um<0.0254,
            "native_clearance_probe":{"raw_matrix_value_mm":expected_mm,"measured_geometry_gap_mm":v.actual_clearance/res,"interpreted_as_source_spacing_finding":!is_structure_marker},
            "source_rule":source_rule,
            "scope":if is_structure_marker {"native structure marker: engine drill_allowed requires first_layer==last_layer; a same-net via against a multilayer pin is an obstacle. Source pin drill unknown; this is not a numeric spacing violation or asserted electrical short."} else if on_boundary_or_keepout {"native boundary/keepout interaction is a candidate; no numeric source edge setback was declared and containment is separately checked against the exact source rectangle"} else {"raw clearance matrix threshold, zero tolerance; spacing shortfall retained"},
            "classification":if is_structure_marker {"new_route_structure_interaction"} else if involved_route {"new_route_interaction"} else {"existing_baseline_geometry"},"baseline_clearance_violation":!involved_route}));
    }
    let board_res_mm = board
        .communication
        .get_resolution(fr_engine::structure::Unit::Mm);
    let baseline_at_loaded_default_tolerance = base_drc
        .iter()
        .filter(|v| {
            (v.expected_clearance - v.actual_clearance) / board_res_mm * 1000.0
                > loaded_default_tolerance_um
        })
        .count();
    let baseline_rows: Vec<_> = base_drc.iter().map(|v| {
        let shortfall_um = (v.expected_clearance - v.actual_clearance) / board_res_mm * 1000.0;
        json!({"native_item_ids":[baseline.item(v.first_item).id().0,baseline.item(v.second_item).id().0],
            "layer":baseline.layer_structure.layers.get(v.layer as usize).map(|l|l.name.clone()).unwrap_or_default(),
            "required_clearance_mm":v.expected_clearance/board_res_mm,"actual_clearance_mm":v.actual_clearance/board_res_mm,
            "clearance_shortfall_um":shortfall_um,"near_threshold_2um":shortfall_um<=2.0})
    }).collect();
    let new_raw_spacing_rows: Vec<_> = violation_rows
        .iter()
        .filter(|v| v["classification"] == "new_route_interaction")
        .collect();
    let new_spacing_bands = json!({
        "shortfall_over_2um":new_raw_spacing_rows.iter().filter(|v|v["clearance_shortfall_um"].as_f64().unwrap_or(0.0)>2.0).count(),
        "shortfall_0_1_to_2um":new_raw_spacing_rows.iter().filter(|v|{let x=v["clearance_shortfall_um"].as_f64().unwrap_or(0.0);x>0.1&&x<=2.0}).count(),
        "shortfall_at_most_0_1um":new_raw_spacing_rows.iter().filter(|v|v["clearance_shortfall_um"].as_f64().is_some_and(|x|x<=0.1)).count(),
        "below_one_native_coordinate_unit_0_0254um":new_raw_spacing_rows.iter().filter(|v|v["sub_native_coordinate_unit_sensitive"]==true).count()
    });
    let report = json!({
        "schema":1,"status":"completed","checker":"fr_engine::drc::all_clearance_violations",
        "rule_source":{"dsn_path":dsn_path,"dsn_sha256":source_sha,"rules_contract":rules_contract_path,"loaded_with":"BoardAdapter::load(raw DSN)","project_sidecar_rules_read":false,"clearance_margin_added":false},
        "geometry_source":{"pcb_path":pcb_path,"pcb_sha256":output_sha,"scene_path":scene_path,"input_scene_source_dsn_sha256":scene["source_dsn_sha256"],"coordinate_mapping":"direct native x/y; frozen readback matched all source pad identities and centers within 1e-6 mm with no mismatches","pad_identity_match_count":matched_pads,"center_tolerance_mm":center_tolerance,"input_pins":baseline_count,"output_traces":geometry.segments.len(),"output_vias":geometry.vias.len(),"inserted_route_items":route_meta.len(),"raw_insert_without_cleanup":true,"output_layer_names":geometry.copper_layers},
        "baseline":{"current_source_pin_pin_and_other_violations_at_zero_tolerance":base_drc.len(),"loaded_source_default_clearance_tolerance_um":loaded_default_tolerance_um,"count_if_loaded_default_tolerance_applied":baseline_at_loaded_default_tolerance,"previous_native_pin_pin_reference_count":88,"current_violations":baseline_rows},
        "clearance_violations":violation_rows,
        "copper_containment":containment,
        "unknown_or_ungraded":{"runjob_hole_to_hole_clearance_mm":0.0,"hole_to_hole_scope":"disabled_by_job; no positive threshold synthesized","source_pin_drills_unknown":unknown_pin_drills,"source_pad_hole_clearance":"incomplete where source pad drill metadata is absent","board_edge_clearance":"no numeric source/job edge setback; exact copper containment against source rectangle is separately checked without a positive margin","annular_ring":"no source drill/minimum-annular-ring rule"},
        "counts":{"baseline_clearance_violations_at_zero_tolerance":base_drc.len(),"baseline_count_at_loaded_default_tolerance":baseline_at_loaded_default_tolerance,"output_native_clearance_events":final_drc.len(),"new_route_clearance_events":final_drc.len()-base_drc.len(),"new_raw_spacing_findings":new_raw_spacing_rows.len(),"new_via_on_multilayer_pin_structure_markers":violation_rows.iter().filter(|v|v["type"]=="via_on_multilayer_pin").count(),"baseline_only_events":violation_rows.iter().filter(|v|v["classification"]=="existing_baseline_geometry").count(),"new_spacing_shortfall_bands":new_spacing_bands,"engine_deduplication":"item pair + layer; multiple rows can share the same coordinates"}
    });
    if let Some(parent) = report_path.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::write(
        &route_geometry_path,
        serde_json::to_vec_pretty(&route_geometry)?,
    )?;
    let csv_path = report_path
        .parent()
        .ok_or("report has no parent directory")?
        .join("drc.csv");
    let mut csv = String::from("type,classification,marker_x_mm,marker_y_mm,layer,item_a_id,item_a_kind,item_a_uuid,item_a_pin_ref,item_a_pin_name,item_a_layers,item_a_drill_known,item_a_nets,item_b_id,item_b_kind,item_b_uuid,item_b_pin_ref,item_b_pin_name,item_b_layers,item_b_drill_known,item_b_nets,required_clearance_mm,actual_clearance_mm,clearance_shortfall_um,near_threshold_2um,source_rule\n");
    for row in &violation_rows {
        let items = row["items"].as_array().ok_or("DRC row has no item pair")?;
        let item_cells = |item: &Value| -> Vec<String> {
            let source = &item["source_geometry"];
            let identity = &item["identity"];
            let layers = if source["layers"].is_array() {
                source["layers"].clone()
            } else if source["layer"].is_string() {
                json!([source["layer"]])
            } else if identity["layer_span"].is_object() {
                json!([
                    identity["layer_span"]["first_name"],
                    identity["layer_span"]["last_name"]
                ])
            } else {
                item["native_shape_bboxes"]
                    .as_array()
                    .map(|a| {
                        json!(a
                            .iter()
                            .filter_map(|x| x["layer"].as_str())
                            .collect::<Vec<_>>())
                    })
                    .unwrap_or(Value::Null)
            };
            vec![
                json_cell(&item["native_item_id"]),
                json_cell(&item["kind"]),
                json_cell(&source["source_uuid"]),
                json_cell(&identity["reference"]),
                json_cell(&identity["pin"]),
                json_cell(&layers),
                json_cell(&identity["drill_known"]),
                json_cell(&item["nets"]),
            ]
        };
        let marker = &row["coordinates_mm"]["point"];
        let mut cells = vec![
            json_cell(&row["type"]),
            json_cell(&row["classification"]),
            json_cell(&marker[0]),
            json_cell(&marker[1]),
            json_cell(&row["layer"]),
        ];
        cells.extend(item_cells(&items[0]));
        cells.extend(item_cells(&items[1]));
        cells.extend([
            json_cell(&row["required_clearance_mm"]),
            json_cell(&row["actual_clearance_mm"]),
            json_cell(&row["clearance_shortfall_um"]),
            json_cell(&row["near_threshold_2um"]),
            json_cell(&row["source_rule"]),
        ]);
        csv.push_str(
            &cells
                .iter()
                .map(|c| csv_field(c))
                .collect::<Vec<_>>()
                .join(","),
        );
        csv.push('\n');
    }
    fs::write(csv_path, csv)?;
    fs::write(report_path, serde_json::to_vec_pretty(&report)?)?;
    println!("{}", serde_json::to_string(&report["counts"])?);
    Ok(())
}

fn main() {
    if let Err(e) = run() {
        eprintln!("awx_output_drc: {e}");
        std::process::exit(1);
    }
}
