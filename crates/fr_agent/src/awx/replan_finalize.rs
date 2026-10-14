//! Native `--apply=strip` closeout for incremental replan rounds.
//!
//! A probe's routed board is the authoritative routed result.  The fanout
//! board is rebuilt from each net's last realized fanout provenance, then
//! missing pad-exit copper is salvaged from that routed board.  No filename
//! or external router participates in this transition.
use super::{
    board_snapshot::BoardSnapshot, evolution_native, native_execution, replan_state::ReplanState,
    whole_context::WholeContext,
};
use crate::board_adapter::BoardAdapter;
use fr_geom::Polyline;
use serde_json::Value;
use std::{collections::BTreeSet, time::Instant};

const ARRAY_WINDOW_PAD_MM: f64 = 2.0;

#[derive(Clone)]
pub struct StripFinalization {
    pub routed: BoardAdapter,
    pub fanout: BoardAdapter,
    pub plan_sidecar: Value,
    pub plan_ends: serde_json::Map<String, Value>,
    pub plan_sidecar_hash: String,
    pub pack_sidecar: Value,
    pub census_sidecar: Option<Value>,
    pub census_sidecar_hash: Option<String>,
    pub salvaged: Vec<String>,
    pub grade: super::route_evolution::RouteGrade,
}

/// Result of the non-strip apply path after the legacy native braid executor.
#[derive(Clone)]
pub struct RebraidFinalization {
    pub routed: BoardAdapter,
    pub fanout: BoardAdapter,
    pub plan_sidecar: Value,
    pub plan_ends: serde_json::Map<String, Value>,
    pub plan_sidecar_hash: String,
    pub pack_sidecar: Value,
    pub census_sidecar: Option<Value>,
    pub census_sidecar_hash: Option<String>,
    pub grade: super::route_evolution::RouteGrade,
    pub whole_rounds: usize,
    pub relayed: Vec<String>,
    /// Lanes the legacy braid left open; partial boards remain gradeable.
    pub refused: Vec<String>,
}

fn sidecar_hash(value: &Value) -> Result<String, String> {
    use sha1::{Digest, Sha1};
    let digest = format!(
        "{:x}",
        Sha1::digest(serde_json::to_vec(value).map_err(|e| e.to_string())?)
    );
    Ok(digest.chars().take(16).collect())
}

/// Stable 16-hex content key used by native world fingerprints.
pub fn hash_sidecar(value: &Value) -> Result<String, String> {
    sidecar_hash(value)
}

fn bounds(board: &BoardAdapter, component: &str) -> Result<[f64; 4], String> {
    let pads: Vec<_> = board
        .pads()
        .into_iter()
        .filter(|p| p.chip == component)
        .collect();
    if pads.is_empty() {
        return Err(format!("native replan array {component} has no pads"));
    }
    let mut b = [
        f64::INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NEG_INFINITY,
    ];
    for pad in pads {
        b[0] = b[0].min(pad.point[0]);
        b[1] = b[1].min(pad.point[1]);
        b[2] = b[2].max(pad.point[0]);
        b[3] = b[3].max(pad.point[1]);
    }
    Ok([
        b[0] - ARRAY_WINDOW_PAD_MM,
        b[1] - ARRAY_WINDOW_PAD_MM,
        b[2] + ARRAY_WINDOW_PAD_MM,
        b[3] + ARRAY_WINDOW_PAD_MM,
    ])
}

fn inside(p: [f64; 2], b: [f64; 4]) -> bool {
    b[0] <= p[0] && p[0] <= b[2] && b[1] <= p[1] && p[1] <= b[3]
}

fn segment_near(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> bool {
    let close =
        |p: [f64; 2], q: [f64; 2]| (p[0] - q[0]).abs() < 0.002 && (p[1] - q[1]).abs() < 0.002;
    (close(a, c) && close(b, d)) || (close(a, d) && close(b, c))
}

fn segment_is_fanout(
    fanout: &BoardAdapter,
    net: fr_engine::ids::NetNo,
    layer: i32,
    a: [f64; 2],
    b: [f64; 2],
) -> bool {
    fanout.board.get_traces().iter().any(|&key| {
        let item = fanout.board.item(key);
        if !item.net_numbers().contains(&net) || item.trace().layer() != layer {
            return false;
        }
        item.trace()
            .polyline()
            .corner_approx_arr()
            .windows(2)
            .any(|pair| {
                let c = [
                    pair[0].x / fanout.units_per_mm,
                    pair[0].y / fanout.units_per_mm,
                ];
                let d = [
                    pair[1].x / fanout.units_per_mm,
                    pair[1].y / fanout.units_per_mm,
                ];
                segment_near(a, b, c, d)
            })
    })
}

fn updated_pack(
    old: &Value,
    routed: &BoardAdapter,
    fanout: &BoardAdapter,
    names: &[String],
) -> Result<Value, String> {
    let mut sidecar = old.clone();
    let root = sidecar
        .as_object_mut()
        .ok_or("native braid pack sidecar is not an object")?;
    let lanes = root.entry("lanes").or_insert_with(|| serde_json::json!({}));
    let lanes = lanes
        .as_object_mut()
        .ok_or("native braid pack sidecar lanes is not an object")?;
    for name in names {
        let net = routed.net_number(name)?;
        let mut segments = Vec::new();
        for key in routed.board.get_traces() {
            let item = routed.board.item(key);
            if !item.net_numbers().contains(&net) {
                continue;
            }
            let trace = item.trace();
            let layer = trace.layer();
            let points = trace.polyline().corner_approx_arr();
            for pair in points.windows(2) {
                let a = [
                    pair[0].x / routed.units_per_mm,
                    pair[0].y / routed.units_per_mm,
                ];
                let b = [
                    pair[1].x / routed.units_per_mm,
                    pair[1].y / routed.units_per_mm,
                ];
                if segment_is_fanout(fanout, net, layer, a, b) {
                    continue;
                }
                // Native sidecars encode the layer as its stable stack index;
                // this keeps every field numeric while retaining pack geometry.
                segments.push(serde_json::json!([
                    a[0],
                    a[1],
                    b[0],
                    b[1],
                    layer,
                    2.0 * trace.half_width() as f64 / routed.units_per_mm,
                ]));
            }
        }
        let mut vias = Vec::new();
        for key in routed.board.get_vias() {
            let item = routed.board.item(key);
            if !item.net_numbers().contains(&net) {
                continue;
            }
            let center = item.center(&routed.board).to_float();
            let p = [
                center.x / routed.units_per_mm,
                center.y / routed.units_per_mm,
            ];
            let in_fanout = fanout.board.get_vias().iter().any(|&fkey| {
                let f = fanout.board.item(fkey);
                if !f.net_numbers().contains(&net) {
                    return false;
                }
                let q = f.center(&fanout.board).to_float();
                (p[0] - q.x / fanout.units_per_mm).abs() < 0.002
                    && (p[1] - q.y / fanout.units_per_mm).abs() < 0.002
            });
            if !in_fanout {
                vias.push(serde_json::json!(p));
            }
        }
        let lane = lanes
            .entry(name.clone())
            .or_insert_with(|| serde_json::json!({}));
        let lane = lane
            .as_object_mut()
            .ok_or_else(|| format!("native pack lane {name} is not an object"))?;
        lane.insert("segs".into(), Value::Array(segments));
        lane.insert("vias".into(), Value::Array(vias));
    }
    Ok(sidecar)
}

/// Rebuild pack lane copper from the true routed/fanout native board pair.
pub fn update_pack_sidecar(
    old: &Value,
    routed: &BoardAdapter,
    fanout: &BoardAdapter,
    names: &[String],
) -> Result<Value, String> {
    updated_pack(old, routed, fanout, names)
}

fn sidecar_from_context(
    old: &Value,
    context: &WholeContext,
    flow: super::whole_frame::FlowFrame,
) -> Result<Value, String> {
    let mut sidecar = old.clone();
    let root = sidecar
        .as_object_mut()
        .ok_or("native plan sidecar is not an object")?;
    let physical_layers = &context.copper_layers;
    let to_board_vector = |v: [f64; 2]| {
        let origin = flow.to_board([0.0, 0.0]);
        let point = flow.to_board(v);
        [point[0] - origin[0], point[1] - origin[1]]
    };
    let mut ends = serde_json::Map::new();
    let mut tooth_layer = serde_json::Map::new();
    let mut dest_layer = serde_json::Map::new();
    let mut tooth_dir = serde_json::Map::new();
    let mut stub_dir = serde_json::Map::new();
    for (name, end) in &context.ends {
        ends.insert(
            name.clone(),
            serde_json::json!([flow.to_board(end.tooth), flow.to_board(end.berth)]),
        );
        if let Some(layer) = context.tooth_layer.get(name) {
            tooth_layer.insert(
                name.clone(),
                Value::String(flow.to_board_layer(layer, physical_layers)?),
            );
        }
        if let Some(layer) = context.dest_layer.get(name) {
            dest_layer.insert(
                name.clone(),
                Value::String(flow.to_board_layer(layer, physical_layers)?),
            );
        }
        if let Some(dir) = context.tooth_dir.get(name) {
            tooth_dir.insert(name.clone(), serde_json::json!(to_board_vector(*dir)));
        }
        if let Some(dir) = context.stub_dir.get(name) {
            stub_dir.insert(name.clone(), serde_json::json!(to_board_vector(*dir)));
        }
    }
    root.insert("ends".into(), Value::Object(ends));
    root.insert("tooth_layer".into(), Value::Object(tooth_layer));
    root.insert("dest_layer".into(), Value::Object(dest_layer));
    root.insert("tooth_dir".into(), Value::Object(tooth_dir));
    root.insert("stub_dir".into(), Value::Object(stub_dir));
    Ok(sidecar)
}

/// Write the full native plan sidecar from actual fanout context in board coordinates.
pub fn update_plan_sidecar(
    old: &Value,
    context: &WholeContext,
    flow: super::whole_frame::FlowFrame,
) -> Result<Value, String> {
    sidecar_from_context(old, context, flow)
}

/// Run the native counterpart of replan.py's non-strip `BRAID` branch. The
/// source/destination ends have already been physically realized by the probe
/// transaction (`world.fanout`); braid the complete active set with the
/// migrated legacy corridor executor.
pub fn rebraid(state: &ReplanState) -> Result<RebraidFinalization, String> {
    let probe = &state.probe;
    if Instant::now() >= probe.deadline {
        return Err("native replan deadline exhausted before full rebraid".into());
    }
    let braided = super::replan_probes::run_legacy_braid(state, &state.world.fanout, true)?;
    let routed = braided.board;
    let grade = evolution_native::grade_board(&routed, &state.world.spec)?;
    let pack_sidecar = updated_pack(
        &state.world.pack_sidecar,
        &routed,
        &state.world.fanout,
        &state.names,
    )?;
    let fanout_context = fanout_context(state, &state.world.fanout)?;
    let plan_sidecar = sidecar_from_context(
        &state.world.plan_sidecar,
        &fanout_context.context,
        fanout_context.flow,
    )?;
    let plan_ends = plan_sidecar
        .get("ends")
        .and_then(Value::as_object)
        .cloned()
        .ok_or("native rebraid plan sidecar lost ends object")?;
    let plan_sidecar_hash = sidecar_hash(&plan_sidecar)?;
    Ok(RebraidFinalization {
        routed,
        fanout: state.world.fanout.clone(),
        plan_sidecar,
        plan_ends,
        plan_sidecar_hash,
        pack_sidecar,
        census_sidecar: state.world.census_sidecar.clone(),
        census_sidecar_hash: state.world.census_sidecar_hash.clone(),
        grade,
        // The legacy braid runner does not expose whole-route round or relay
        // counters; zero means those counters were not produced by this path.
        whole_rounds: 0,
        relayed: Vec::new(),
        refused: braided.refused,
    })
}

fn copy_window_copper(
    source: &BoardAdapter,
    target: &mut BoardAdapter,
    net: &str,
    b: [f64; 4],
) -> Result<usize, String> {
    let source_net = source.net_number(net)?;
    let target_net = target.net_number(net)?;
    let mut added = 0;
    for key in source.board.get_traces() {
        let item = source.board.item(key);
        if !item.net_numbers().contains(&source_net) {
            continue;
        }
        let trace = item.trace();
        let corners = trace.polyline().corner_approx_arr();
        let segments: Vec<_> = corners
            .windows(2)
            .filter_map(|pair| {
                let a = [
                    pair[0].x / source.units_per_mm,
                    pair[0].y / source.units_per_mm,
                ];
                let z = [
                    pair[1].x / source.units_per_mm,
                    pair[1].y / source.units_per_mm,
                ];
                (inside(a, b) && inside(z, b)).then_some((a, z))
            })
            .collect();
        if segments.is_empty() {
            continue;
        }
        let layer = source.board.layer_structure.layers[item.first_layer(&source.board) as usize]
            .name
            .clone();
        let target_layer = target.board.layer_structure.get_no(&layer);
        if target_layer < 0 {
            return Err(format!("native salvage layer {layer} is absent"));
        }
        for (a, z) in segments {
            let pa = target.point(a)?;
            let pz = target.point(z)?;
            let polyline = Polyline::from_two_points(&pa, &pz);
            target
                .board
                .insert_trace_without_cleaning(
                    polyline,
                    target_layer,
                    trace.half_width(),
                    &[target_net],
                    item.clearance_class(),
                    item.fixed_state(),
                )
                .ok_or("native replan pad-exit salvage trace insertion failed")?;
            added += 1;
        }
    }
    for key in source.board.get_vias() {
        let item = source.board.item(key);
        if !item.net_numbers().contains(&source_net) {
            continue;
        }
        let center = item.center(&source.board).to_float();
        let p = [
            center.x / source.units_per_mm,
            center.y / source.units_per_mm,
        ];
        if !inside(p, b) {
            continue;
        }
        let stack = item
            .padstack(&source.board)
            .ok_or("salvage source via has no padstack")?;
        let target_pad = target
            .board
            .library
            .padstacks
            .get_by_name(&stack.name)
            .ok_or_else(|| format!("native salvage via padstack {} is absent", stack.name))?
            .id;
        let point = target.point(p)?;
        target.board.insert_via(
            target_pad,
            point,
            &[target_net],
            item.clearance_class(),
            item.fixed_state(),
            stack.attach_allowed,
        );
        added += 1;
    }
    Ok(added)
}

pub(super) fn fanout_context(
    state: &ReplanState,
    fanout: &BoardAdapter,
) -> Result<super::context_builder::ContextBuild, String> {
    let awx = state
        .world
        .spec
        .awx
        .as_ref()
        .ok_or("native replan requires AWX configuration")?;
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
    let cfg = first
        .awx
        .as_ref()
        .ok_or("native replan route group lacks AWX configuration")?;
    let nets: BTreeSet<_> = groups
        .iter()
        .flat_map(|group| group.nets.iter().cloned())
        .collect();
    let pairs: Vec<_> = groups
        .iter()
        .flat_map(|group| {
            group
                .awx
                .as_ref()
                .into_iter()
                .flat_map(|group| group.pairs.iter().cloned())
        })
        .collect();
    let snapshot = BoardSnapshot::capture(fanout, &awx.components, &awx.via_drills)?;
    super::context_builder::build(
        &snapshot,
        &cfg.source,
        &cfg.destination,
        &nets,
        &pairs,
        &state.probe.source_plan,
        &state.probe.destination_plan,
        &state.probe.source_menus,
        &state.probe.destination_menus,
        &state.probe.rules,
        &cfg.routing_layers,
    )
}

/// Materialize strip/apply semantics after incremental probes have advanced.
/// Each run net is restored from its own most recent fanout board; untouched
/// nets use the seed fanout. Missing array-facing endpoints are salvaged from
/// the routed board in the exact source/destination windows.
pub fn apply_strip(
    state: &ReplanState,
    changed: &BTreeSet<String>,
) -> Result<StripFinalization, String> {
    if let Some(net) = changed.iter().find(|net| !state.names.contains(net)) {
        return Err(format!("changed replan net {net} is not active"));
    }
    let awx = state
        .world
        .spec
        .awx
        .as_ref()
        .ok_or("native replan requires AWX configuration")?;
    let group = state
        .world
        .spec
        .groups
        .iter()
        .find_map(|group| {
            let awx_group = group.awx.as_ref()?;
            (awx_group.destination == state.world.destination
                && group.nets.iter().any(|net| state.names.contains(net)))
            .then_some(awx_group)
        })
        .ok_or("native replan cannot resolve its source/destination array pair")?;
    let source_component = awx
        .components
        .iter()
        .find(|c| c.reference == group.source)
        .ok_or_else(|| {
            format!(
                "native replan source array {} is not configured",
                group.source
            )
        })?;
    let destination_component = awx
        .components
        .iter()
        .find(|c| c.reference == group.destination)
        .ok_or_else(|| {
            format!(
                "native replan destination array {} is not configured",
                group.destination
            )
        })?;
    let source_box = bounds(&state.world.routed, &source_component.reference)?;
    let destination_box = bounds(&state.world.routed, &destination_component.reference)?;

    if let Some(net) = changed.iter().find(|net| !state.fan_src.contains_key(*net)) {
        return Err(format!("changed replan net {net} has no fanout provenance"));
    }
    let mut fanout = state.world.routed.clone();
    let names: BTreeSet<_> = state.names.iter().cloned().collect();
    native_execution::strip_nets(&mut fanout, &names)?;
    for net in &state.names {
        let source = state.fan_src.get(net).unwrap_or(&state.fanout_seed);
        native_execution::restore_nets(source, &mut fanout, &BTreeSet::from([net.clone()]))?;
    }

    let mut salvaged = Vec::new();
    for net in &state.names {
        let net_id = fanout.net_number(net)?;
        let missing_source = !fanout.board.get_traces().iter().any(|&key| {
            let item = fanout.board.item(key);
            item.net_numbers().contains(&net_id)
                && item.trace().polyline().corner_approx_arr().iter().any(|p| {
                    inside(
                        [p.x / fanout.units_per_mm, p.y / fanout.units_per_mm],
                        source_box,
                    )
                })
        });
        let missing_destination = !fanout.board.get_traces().iter().any(|&key| {
            let item = fanout.board.item(key);
            item.net_numbers().contains(&net_id)
                && item.trace().polyline().corner_approx_arr().iter().any(|p| {
                    inside(
                        [p.x / fanout.units_per_mm, p.y / fanout.units_per_mm],
                        destination_box,
                    )
                })
        });
        if missing_source {
            let count = copy_window_copper(&state.world.routed, &mut fanout, net, source_box)?;
            if count > 0 {
                salvaged.push(format!("{net}.src"));
            }
        }
        if missing_destination {
            let count = copy_window_copper(&state.world.routed, &mut fanout, net, destination_box)?;
            if count > 0 {
                salvaged.push(format!("{net}.dst"));
            }
        }
    }
    fanout.board.mark_stitching_vias();
    let context = fanout_context(state, &fanout)?;
    let plan_sidecar =
        sidecar_from_context(&state.world.plan_sidecar, &context.context, context.flow)?;
    let plan_ends = plan_sidecar
        .get("ends")
        .and_then(Value::as_object)
        .cloned()
        .ok_or("native strip plan sidecar lost ends object")?;
    let plan_sidecar_hash = sidecar_hash(&plan_sidecar)?;
    let pack_sidecar = updated_pack(
        &state.world.pack_sidecar,
        &state.world.routed,
        &fanout,
        &state.names,
    )?;
    let grade = evolution_native::grade_board(&state.world.routed, &state.world.spec)?;
    Ok(StripFinalization {
        routed: state.world.routed.clone(),
        fanout,
        plan_sidecar,
        plan_ends,
        plan_sidecar_hash,
        pack_sidecar,
        census_sidecar: state.world.census_sidecar.clone(),
        census_sidecar_hash: state.world.census_sidecar_hash.clone(),
        salvaged,
        grade,
    })
}
