//! Ordinary DSN airlines into the ported AWX GridRouter, with native transactions.
//!
//! This is the ORIGINAL phase driver, not a per-net exhaustive airline loop. The
//! source counterparts are:
//! * `single_ended_loop.route_single_ended_nets` (=472) — one pass over the MPS
//!   net order; multipoint nets route ONE main edge here and defer every other
//!   MST edge to phase 3; plain 2-terminal nets route through the ordinary
//!   single-ended path on the net's full-Cu connected groups.
//! * `single_ended_routing.route_multipoint_main` (=3965) — phase 1 main-edge
//!   selection: component MST, edges longest-first, `max_main_attempts =
//!   min(len(mst_edges), 8)`, winning edge moved to the front for phase 3, and
//!   the source `#348` "phase 1 exhausted from the pads" deferral rooted at the
//!   copper-richest existing component.
//! * `single_ended_routing.get_all_segment_tap_points` (=4577) — exact tap
//!   samples (`max(1, int(length / grid_step))` steps, vertices included, vias
//!   on every physical Cu layer), first writer's owner coord per cell.
//! * `phase3_routing.run_phase3_tap_routing` (=563) — tap phase after ALL nets'
//!   mains, XOR-routed edge eligibility in the remaining original MST order,
//!   growing Cu sources and the target's own-island candidates, with the fresh
//!   component labels recomputed after every commit.
//! * `single_ended_loop` rip ladder (=962..1600) via `reroute_loop.py:467` —
//!   failed-route frontiers handed to the recovery module, victims rerouted
//!   from the queue after the main stage and after the tap casualties.
//!
//! The heavy mechanics (native search, island-tap weld/split of a this-run
//! trace, via-pad unblock, landing aliases) live in the native backend; this
//! driver only builds the per-edge endpoint sets and commits board revisions.

use crate::{
    awx::{
        braid_connect::{rung_specs, LadderMode, RungEndpoint, RungReport, RungRequest},
        endpoint_recovery::{try_source_ripup, RecoveryState, RipAccepted},
        fab_rules::{EscalationPolicy, FabTier},
        native_rung::{endpoint_grid_calls, route_multilayer_endpoint_sets_in_run},
        net_multipoint::{plan_native_multipoint, MultipointPlan, MultipointTerminal},
        whole_context::RoutingRules,
    },
    board_adapter::BoardAdapter,
};
use fr_engine::{
    board::{ItemKey, ItemKind, ItemSet},
    drc::NetIncompletes,
    ids::{ItemId, NetNo},
    library::BoardLibrary,
    rules::BoardRules,
    structure::{Components, LayerStructure},
};
use fr_geom::Point;
use serde::Deserialize;
use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    io::{self, Write},
    path::Path,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

/// Source `route_multipoint_main` main-edge attempt bound.
const MAX_MAIN_ATTEMPTS: usize = 8;
const RECORD_BEST_FULLCHECK_EVERY: u64 = 64;

pub fn endpoint_pad_metadata(
    board: &BoardAdapter,
    require_hole_metadata: bool,
) -> Result<
    (
        Vec<crate::awx::native_grid_window::FreeViaPad>,
        Vec<crate::awx::whole_context::ContextPad>,
    ),
    String,
> {
    use crate::awx::whole_context::{ComponentKind, ContextPad, CopperShape, PadCopperShape};
    let mut free_via_pads = Vec::new();
    let mut same_net_pads = Vec::new();
    for pad in board.pads() {
        let Some(key) = pad.key else { continue };
        let item = board.board.item(key);
        let padstack = item
            .padstack(&board.board)
            .ok_or_else(|| format!("pad {} has no padstack metadata", pad.reference))?;
        let explicit_drill = padstack
            .get_explicit_drill_radius()
            .map(|r| 2.0 * r / board.units_per_mm);
        // `BoardAdapter::pads()` reports signal layers only. Inspect the
        // native pin span so intervening power layers cannot hide a PTH.
        let spans_stack = item.first_layer(&board.board) < item.last_layer(&board.board);
        let drill = explicit_drill.unwrap_or(0.0);
        if require_hole_metadata && spans_stack && drill <= 0.0 {
            return Err(format!("Unsupported: requested hole-to-hole clearance needs explicit PTH drill metadata for pad {}", pad.reference));
        }
        let mut size = [0.0f64; 2];
        // Exact native pin copper on the physical signal layers, exported in the
        // same board-millimetre frame as `BoardSnapshot::capture`
        // (board_snapshot.rs). The native shape is authoritative: no bounding
        // box substitutes for the actual copper and no component body
        // rotation/mirror is applied, so the exported geometry is the exact
        // transformed native copper, not a fabricated enclosure.
        let mut copper_shapes = Vec::new();
        for layer in item.first_layer(&board.board)..=item.last_layer(&board.board) {
            if let Some(shape) = item.drill_shape_on_layer(&board.board, layer) {
                let bounds = shape.bounding_box();
                size[0] = size[0].max((bounds.ur.x - bounds.ll.x) as f64 / board.units_per_mm);
                size[1] = size[1].max((bounds.ur.y - bounds.ll.y) as f64 / board.units_per_mm);
                if board.board.layer_structure.layers[layer as usize].is_signal {
                    let layer_name = board.board.layer_structure.layers[layer as usize]
                        .name
                        .clone();
                    let exported = match &shape {
                        fr_geom::Shape::Circle(c) => CopperShape::Circle {
                            center: [
                                c.center.x as f64 / board.units_per_mm,
                                c.center.y as f64 / board.units_per_mm,
                            ],
                            radius: c.radius as f64 / board.units_per_mm,
                        },
                        _ => match crate::awx::native_pad_raster::source_rectangle(
                            &shape,
                            board.units_per_mm,
                        ) {
                            Some(rect) => CopperShape::RoundedRectangle {
                                center: [rect.center_x_mm, rect.center_y_mm],
                                half_size: [rect.half_width_mm, rect.half_height_mm],
                                rotation: rect.rotation_deg,
                                corner_radius: rect.corner_radius_mm,
                            },
                            None => CopperShape::Polygon {
                                outline: shape
                                    .corner_approx_arr()
                                    .iter()
                                    .map(|p| {
                                        [
                                            p.x as f64 / board.units_per_mm,
                                            p.y as f64 / board.units_per_mm,
                                        ]
                                    })
                                    .collect(),
                            },
                        },
                    };
                    copper_shapes.push(PadCopperShape {
                        layer: layer_name,
                        shape: exported,
                    });
                }
            }
        }
        let net_id = item.net_numbers().first().copied().unwrap_or(0);
        let net = board
            .board
            .rules
            .nets
            .get(net_id)
            .map(|n| n.name.clone())
            .unwrap_or_default();
        let point = [pad.point[0], pad.point[1]];
        same_net_pads.push(ContextPad {
            key: pad.reference.clone(),
            component_ref: pad.chip.clone(),
            pad_number: pad.pad.clone(),
            net,
            net_id: net_id as i32,
            point,
            size,
            layers: pad.layers.clone(),
            drill,
            drill_axis: None,
            non_plated_hole: false,
            corner_radius: 0.0,
            pad_type: if drill > 0.0 {
                "thru_hole"
            } else if spans_stack {
                // Known multilayer electrical topology with no explicit drill
                // dimension: an honest native-multilayer classification, not an
                // SMD pad and not a fabricated plated barrel.
                "native_multilayer"
            } else {
                "smd"
            }
            .into(),
            shape: String::new(),
            rect_rotation: 0.0,
            copper_shapes,
            component_kind: ComponentKind::Other,
            movable: false,
        });
        if drill > 0.0 {
            let board_point = board.point(point)?;
            let fr_geom::Point::Int(board_point) = board_point else {
                return Err(format!(
                    "Unsupported: non-integral PTH center for pad {}",
                    pad.reference
                ));
            };
            let drill_board = (drill * board.units_per_mm).round();
            if drill_board <= 0.0 || drill_board > i32::MAX as f64 {
                return Err(format!(
                    "invalid explicit drill metadata for pad {}",
                    pad.reference
                ));
            }
            free_via_pads.push(crate::awx::native_grid_window::FreeViaPad {
                net_id,
                point_board: [board_point.x, board_point.y],
                drill_board: drill_board as i32,
                non_plated_hole: false,
                drill_axis_board: None,
            });
        }
    }
    Ok((free_via_pads, same_net_pads))
}

fn write_atomic(path: &Path, bytes: &[u8]) -> Result<(), String> {
    use std::sync::atomic::{AtomicU64, Ordering};
    static NEXT_TEMP: AtomicU64 = AtomicU64::new(0);
    let parent = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    let name = path
        .file_name()
        .ok_or("destination has no filename")?
        .to_string_lossy();
    let (temp_path, mut file) = (0..100)
        .find_map(|_| {
            let n = NEXT_TEMP.fetch_add(1, Ordering::Relaxed);
            let candidate = parent.join(format!(".{name}.{}.{}.tmp", std::process::id(), n));
            match fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(&candidate)
            {
                Ok(file) => Some((candidate, file)),
                Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => None,
                Err(_) => None,
            }
        })
        .ok_or("could not create temporary output beside destination")?;
    let result = (|| {
        file.write_all(bytes).map_err(|e| e.to_string())?;
        file.sync_all().map_err(|e| e.to_string())?;
        drop(file);
        fs::rename(&temp_path, path).map_err(|e| e.to_string())
    })();
    if result.is_err() {
        let _ = fs::remove_file(&temp_path);
    }
    result
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct EndpointJob {
    input: String,
    output: String,
    log: String,
    budget_ms: u64,
    grid_step_mm: f64,
    max_iterations: usize,
    threads: usize,
    via_drill_mm: f64,
    min_annular_ring_mm: f64,
    hole_to_hole_clearance_mm: f64,
    #[serde(default)]
    protected_nets: BTreeSet<String>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ResidualJob {
    input: String,
    session: String,
    output: String,
    log: String,
    #[serde(default)]
    nets: Vec<String>,
    budget_ms: u64,
    grid_step_mm: f64,
    max_iterations: usize,
    threads: usize,
    via_drill_mm: f64,
    min_annular_ring_mm: f64,
    hole_to_hole_clearance_mm: f64,
    #[serde(default)]
    protected_nets: BTreeSet<String>,
}

fn rules_for_layer(
    board: &BoardAdapter,
    net: NetNo,
    layer: i32,
    grid: f64,
    drill: f64,
    min_annular: f64,
    hole_clearance: f64,
) -> Result<RoutingRules, String> {
    let class = board.board.rules.net_class_of(net);
    let width = 2.0 * class.get_trace_half_width(layer) as f64 / board.units_per_mm;
    let clearance_class = class.get_trace_clearance_class();
    let clearance = board
        .board
        .clearance_value(clearance_class, clearance_class, layer) as f64
        / board.units_per_mm;
    let via_size = board
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
        .filter_map(|p| p.get_shape(layer))
        .map(|s| s.max_width() / board.units_per_mm)
        .fold(0_f64, f64::max);
    if via_size <= 0.0 {
        return Err("DSN has no native via copper envelope".into());
    }
    if !drill.is_finite()
        || drill <= 0.0
        || !min_annular.is_finite()
        || min_annular < 0.0
        || !hole_clearance.is_finite()
        || hole_clearance < 0.0
        || via_size - drill < 2.0 * min_annular
    {
        return Err("explicit test drill violates native via annular ring".into());
    }
    Ok(RoutingRules {
        grid_step: grid,
        track_width: width,
        clearance,
        via_size,
        via_drill: drill,
        pair_gap: clearance,
        lane_pitch: width + clearance,
        lane_min: width,
        min_turning_radius: 0.0,
        max_turn_angle: 180.0,
        max_setback_angle: 45.0,
        routing_clearance_margin: 1.0,
        board_edge_clearance: 0.0,
        hole_to_hole_clearance: hole_clearance,
        fan_clearance: clearance,
        geometry_pair_room: false,
        geometry_pair_turn_room_steps: None,
        same_net_pad_clearance: 0.0,
        net_via_sizes: BTreeMap::new(),
    })
}

const LOG_WRITE_SAMPLE_EVERY: u64 = 1024;

#[derive(Clone, Copy, Default)]
struct LogWriteProfile {
    write_calls: u64,
    bytes: u64,
    flush_calls: u64,
    sampled_write_calls: u64,
    sampled_write_elapsed_ns: u128,
    log_event_calls: u64,
    log_event_elapsed_ns: u128,
    serialize_elapsed_ns: u128,
    write_flush_elapsed_ns: u128,
}

thread_local! {
    static LOG_WRITE_PROFILE: std::cell::RefCell<LogWriteProfile> = std::cell::RefCell::new(LogWriteProfile::default());
    static LOG_EVENT_BUFFER: std::cell::RefCell<Vec<u8>> = const { std::cell::RefCell::new(Vec::new()) };
}

struct ProfiledLogWriter<'a> {
    inner: &'a mut fs::File,
}

impl Write for ProfiledLogWriter<'_> {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        let ordinal = LOG_WRITE_PROFILE.with(|profile| {
            let mut profile = profile.borrow_mut();
            profile.write_calls += 1;
            profile.write_calls
        });
        let sampled = ordinal % LOG_WRITE_SAMPLE_EVERY == 0;
        let started = sampled.then(Instant::now);
        let result = self.inner.write(bytes);
        LOG_WRITE_PROFILE.with(|profile| {
            let mut profile = profile.borrow_mut();
            if let Ok(written) = &result {
                profile.bytes += *written as u64;
            }
            if let Some(started) = started {
                profile.sampled_write_calls += 1;
                profile.sampled_write_elapsed_ns += started.elapsed().as_nanos();
            }
        });
        result
    }

    fn flush(&mut self) -> io::Result<()> {
        LOG_WRITE_PROFILE.with(|profile| profile.borrow_mut().flush_calls += 1);
        self.inner.flush()
    }
}

fn log_event(log: &mut fs::File, event: &serde_json::Value) -> Result<(), String> {
    let started = Instant::now();
    let result = LOG_EVENT_BUFFER.with(|buffer| {
        let mut buffer = buffer.borrow_mut();
        buffer.clear();
        let serialize_started = Instant::now();
        let serialize_result = serde_json::to_writer(&mut *buffer, event)
            .map_err(|e| e.to_string())
            .map(|()| buffer.push(b'\n'));
        let serialize_elapsed_ns = serialize_started.elapsed().as_nanos();
        LOG_WRITE_PROFILE.with(|profile| {
            profile.borrow_mut().serialize_elapsed_ns += serialize_elapsed_ns;
        });
        serialize_result?;

        let write_started = Instant::now();
        let result = (|| {
            let mut writer = ProfiledLogWriter { inner: log };
            writer.write_all(&buffer).map_err(|e| e.to_string())?;
            writer.flush().map_err(|e| e.to_string())
        })();
        LOG_WRITE_PROFILE.with(|profile| {
            profile.borrow_mut().write_flush_elapsed_ns += write_started.elapsed().as_nanos();
        });
        result
    });
    LOG_WRITE_PROFILE.with(|profile| {
        let mut profile = profile.borrow_mut();
        profile.log_event_calls += 1;
        profile.log_event_elapsed_ns += started.elapsed().as_nanos();
    });
    result
}

fn log_write_profile_json() -> serde_json::Value {
    let profile = LOG_WRITE_PROFILE.with(|profile| *profile.borrow());
    serde_json::json!({
        "coverage":"thread-local aggregate for log_event; write_calls count Write::write calls through the forwarding wrapper, not OS syscalls; bytes are successful returned byte counts; flush_calls count forwarded Write::flush calls.",
        "write_calls":profile.write_calls,
        "bytes":profile.bytes,
        "flush_calls":profile.flush_calls,
        "log_event_calls":profile.log_event_calls,
        "log_event_format_write_flush_elapsed_ns":profile.log_event_elapsed_ns,
        "serialize_elapsed_ns":profile.serialize_elapsed_ns,
        "write_flush_elapsed_ns":profile.write_flush_elapsed_ns,
        "sampled_write_every_n_calls":LOG_WRITE_SAMPLE_EVERY,
        "sampled_write_calls":profile.sampled_write_calls,
        "sampled_write_elapsed_ns":profile.sampled_write_elapsed_ns,
        "sampled_write_elapsed_is_estimate":true
    })
}

/// True when a native contact handle (tile shape) of `key` on one of its own
/// layers contains `point`. Gates conduction-area terminals: only corners a
/// native area shape actually contains may attach, and no geometry outside
/// the native shapes is invented.
fn native_contact_handle_contains(board: &BoardAdapter, key: ItemKey, point: &Point) -> bool {
    let item = board.board.item(key);
    let from = item.first_layer(&board.board);
    let to = item.last_layer(&board.board);
    (0..board.board.tile_shape_count(key)).any(|index| {
        let layer = board.board.shape_layer(key, index);
        layer >= from
            && layer <= to
            && board
                .board
                .tile_shape(key, index)
                .is_some_and(|shape| shape.contains(point))
    })
}

/// The exact ORIGINAL native terminal points of one item the grid router may
/// attach to without inventing or interpolating geometry. Pins and vias
/// attach at their drill center; a trace only at its two polyline corners
/// (an interior corner would require splitting the trace). A conduction area
/// contributes the ratsnest corners a native contact handle contains.
fn item_terminal_points(board: &BoardAdapter, key: ItemKey) -> Vec<Point> {
    let item = board.board.item(key);
    match &item.kind {
        ItemKind::Pin(_) | ItemKind::Via(_) => vec![item.center(&board.board)],
        ItemKind::Trace(_) => vec![item.first_corner(), item.last_corner()],
        ItemKind::ConductionArea(_) => board
            .board
            .ratsnest_corners(key)
            .into_iter()
            .filter(|point| native_contact_handle_contains(board, key, point))
            .collect(),
        _ => Vec::new(),
    }
}

/// Deterministic Point+layer candidates from every native terminal in one
/// native connected group, restricted to the supplied active signal layers.
/// Duplicates at the same exact board point and layer collapse.
fn connected_group_candidates(
    board: &BoardAdapter,
    group: &ItemSet,
    active_layers: &[(i32, String)],
) -> Vec<RungEndpoint> {
    let mut seen = BTreeSet::new();
    let mut endpoints = Vec::new();
    for member in group.iter() {
        let item = board.board.item(member);
        for point in item_terminal_points(board, member) {
            let exact = point.to_float();
            let mm = [exact.x / board.units_per_mm, exact.y / board.units_per_mm];
            for (layer_index, layer_name) in active_layers {
                if !item.is_on_layer(&board.board, *layer_index) {
                    continue;
                }
                if seen.insert((mm[0].to_bits(), mm[1].to_bits(), layer_name.clone())) {
                    endpoints.push(RungEndpoint {
                        point: mm,
                        layer: layer_name.clone(),
                    });
                }
            }
        }
    }
    endpoints.sort_by(|a, b| {
        (a.point[0].to_bits(), a.point[1].to_bits(), &a.layer).cmp(&(
            b.point[0].to_bits(),
            b.point[1].to_bits(),
            &b.layer,
        ))
    });
    endpoints
}

fn stable_connected_group_ids(board: &BoardAdapter, net: NetNo) -> Vec<Vec<i32>> {
    let mut groups = board
        .board
        .get_connected_sets(net)
        .into_iter()
        .map(|group| {
            let mut ids: Vec<i32> = group
                .iter()
                .map(|member| board.board.item(member).id().0)
                .collect();
            ids.sort_unstable();
            ids
        })
        .collect::<Vec<_>>();
    groups.sort();
    groups
}

/// Final-board inventory only. NetIncompletes and connected components are each
/// computed once per net; failure history is explicitly net-level evidence.
fn final_unconnected_inventory(
    board: &BoardAdapter,
    nets: &[(NetNo, String)],
    attempts: &[serde_json::Value],
    skips: &[serde_json::Value],
    profile: &DriverProfile,
) -> serde_json::Value {
    let pad_refs: BTreeMap<i32, String> = board
        .pads()
        .into_iter()
        .filter_map(|pad| {
            pad.key
                .map(|key| (board.board.item(key).id().0, pad.reference))
        })
        .collect();
    let mut inventory = Vec::new();
    for (net, name) in nets {
        let items = board.board.get_connectable_items(*net);
        let remaining = NetIncompletes::new(&board.board, *net, &items);
        if remaining.count() == 0 {
            continue;
        }
        let groups = board.board.get_connected_sets(*net);
        let group_members: Vec<Vec<i32>> = groups
            .iter()
            .map(|group| {
                let mut ids: Vec<i32> = group
                    .iter()
                    .map(|key| board.board.item(key).id().0)
                    .collect();
                ids.sort_unstable();
                ids
            })
            .collect();
        let component_for = |key: ItemKey| -> Vec<i32> {
            let id = board.board.item(key).id().0;
            group_members
                .iter()
                .find(|members| members.binary_search(&id).is_ok())
                .cloned()
                .unwrap_or_default()
        };
        let endpoint = |key: ItemKey, corner: fr_geom::FloatPoint| {
            let item = board.board.item(key);
            let id = item.id().0;
            let item_kind = match &item.kind {
                ItemKind::Pin(_) => "pin",
                ItemKind::Via(_) => "via",
                ItemKind::Trace(_) => "trace",
                ItemKind::ObstacleArea(_) => "obstacle_area",
                ItemKind::ConductionArea(_) => "conduction_area",
                ItemKind::ComponentOutline(_) => "component_outline",
                ItemKind::BoardOutline(_) => "board_outline",
            };
            let layers: Vec<String> = board
                .board
                .layer_structure
                .layers
                .iter()
                .enumerate()
                .filter(|(layer, info)| {
                    if !info.is_signal {
                        return false;
                    }
                    let layer = *layer as i32;
                    if item.is_pin() || item.is_via() {
                        item.padstack(&board.board)
                            .and_then(|padstack| padstack.get_shape(layer))
                            .is_some()
                    } else {
                        item.first_layer(&board.board) == layer
                    }
                })
                .map(|(_, layer)| layer.name.clone())
                .collect();
            serde_json::json!({
                "item_id": id,
                "pin_identity": pad_refs.get(&id),
                "item_kind": item_kind,
                "xy_mm": [corner.x / board.units_per_mm, corner.y / board.units_per_mm],
                "layers": layers,
                "component_item_ids": component_for(key),
                "component_pin_members": component_for(key).iter().filter_map(|id| pad_refs.get(id).cloned()).collect::<Vec<_>>(),
            })
        };
        let gaps: Vec<_> = remaining
            .incompletes
            .iter()
            .map(|line| {
                serde_json::json!({
                    "from": endpoint(line.from_item, line.from_corner),
                    "to": endpoint(line.to_item, line.to_corner),
                })
            })
            .collect();
        let net_attempts: Vec<_> = attempts
            .iter()
            .filter(|event| event["net"].as_str() == Some(name.as_str()))
            .collect();
        let retry_count = profile
            .nets
            .get(name)
            .and_then(|stages| stages.get("retry_build_endpoint_request"))
            .map(|counter| counter.calls)
            .unwrap_or(0);
        let last_failure = net_attempts.iter().rev().find_map(|event| {
            let result = &event["result"];
            if result["accepted"].as_bool().unwrap_or(false) {
                return None;
            }
            let diagnostic = &result["_endpoint_diagnostic"];
            let failure_stage = diagnostic["stage"].as_str();
            let raw = result["reason"].as_str();
            let lower = raw.unwrap_or_default().to_ascii_lowercase();
            let category = match failure_stage {
                Some("group_merge" | "net_count") => "connectivity",
                Some("drc") => "drc",
                Some("search_no_path") => "search_no_path",
                Some(stage) if stage.starts_with("deadline") => "deadline",
                _ if lower.contains("budget_exhaustion") => "budget_exhaustion",
                _ => "unknown_decline",
            };
            Some(serde_json::json!({
                "route_stage": event["stage"], "failure_stage": failure_stage,
                "reason": raw, "category": category,
                "expanded": result.get("expanded").or_else(|| result.get("expanded_nodes")),
                "grid_router_calls": result["grid_router_calls"],
                "evidence_scope": "net_history",
            }))
        });
        let skip = skips
            .iter()
            .find(|skip| skip["net"].as_str() == Some(name.as_str()));
        inventory.push(serde_json::json!({
            "net": name, "net_number": net, "net_incompletes": remaining.count(),
            "airlines": gaps, "attempt_count": net_attempts.len() as u64 + retry_count,
            "primary_attempt_count": net_attempts.len(), "rip_retry_count": retry_count,
            "last_net_failure": last_failure,
            "unattempted_or_skipped_reason": skip.map(|v| &v["reason"]),
            "connected_component_count": groups.len(),
        }));
    }
    serde_json::Value::Array(inventory)
}

fn resolved_destination(path: &Path) -> Result<std::path::PathBuf, String> {
    if path.exists() {
        return fs::canonicalize(path).map_err(|e| e.to_string());
    }
    let parent = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    Ok(fs::canonicalize(parent)
        .map_err(|e| e.to_string())?
        .join(path.file_name().ok_or("destination has no filename")?))
}
fn same_file(a: &Path, b: &Path) -> bool {
    if a == b {
        return true;
    }
    #[cfg(unix)]
    {
        use std::os::unix::fs::MetadataExt;
        if let (Ok(a), Ok(b)) = (fs::metadata(a), fs::metadata(b)) {
            return a.dev() == b.dev() && a.ino() == b.ino();
        }
    }
    false
}

// ---------------------------------------------------------------------------------------------
// Phase driver

/// Immutable per-run parameters shared by every route attempt.
#[derive(Clone)]
struct EndpointContext {
    free_via_pads: Vec<crate::awx::native_grid_window::FreeViaPad>,
    same_net_pads: Vec<crate::awx::whole_context::ContextPad>,
    grid_step_mm: f64,
    via_drill_mm: f64,
    min_annular_ring_mm: f64,
    hole_to_hole_clearance_mm: f64,
    max_iterations: usize,
    deadline: Instant,
}

/// One multipoint net whose main edge landed (or was deferred from phase 1) and
/// whose remaining MST edges wait for the phase-3 tap pass after EVERY net's
/// main. The plan and the routed terminal indices are the source
/// `multipoint_pad_info` / `routed_pad_indices` custody.
struct PendingNet {
    net: NetNo,
    name: String,
    plan: MultipointPlan,
    routed: BTreeSet<usize>,
    route_layers: Vec<(i32, String)>,
}

/// Outcome of one net's phase-1 main pass.
enum Phase1Outcome {
    /// Multipoint main landed (or deferred): the remaining edges wait for
    /// phase 3 with this plan and these routed terminal indices.
    Pending {
        plan: MultipointPlan,
        routed: BTreeSet<usize>,
    },
    /// Plain 2-terminal net handled by the ordinary single-ended path.
    Plain,
    /// No main edge landed and no existing-copper deferral applied.
    Failed,
}

struct Driver<'a> {
    ctx: EndpointContext,
    baseline: &'a BoardAdapter,
    protected: &'a BTreeSet<String>,
    log: fs::File,
    attempts: Vec<serde_json::Value>,
    skips: Vec<serde_json::Value>,
    accepted: usize,
    recovery: RecoveryState,
    /// Every multipoint net whose main landed (or was deferred) and whose
    /// remaining MST edges wait for the phase-3 tap pass. This is the source
    /// `state.pending_multipoint_nets` custody: a net whose main copper is torn
    /// as a rip victim must have its entry invalidated here so the tap pass can
    /// never launch from a phantom main.
    pending: Vec<PendingNet>,
    /// Best consistent board seen AFTER EACH accepted revision (main, tap,
    /// rip, reroute), not only at phase boundaries, matching the source
    /// best-board custody. A rip whose victims were not rerouted leaves the run
    /// with casualties, so the deadline export must not ship worse connectivity
    /// than the best earlier consistent state.
    best_board: BoardAdapter,
    /// Actual-copper incomplete count of `best_board` (i32, source count type).
    best_incompletes: i32,
    incomplete_counts: BTreeMap<NetNo, i32>,
    incomplete_total: i32,
    count_witness: IncompleteCountWitness,
    record_best_calls: u64,
    profile: DriverProfile,
}

struct IncompleteCountWitness {
    item_epoch: u64,
    global_epoch: u64,
    rules: Arc<BoardRules>,
    library: Arc<BoardLibrary>,
    components: Arc<Components>,
    layers: Arc<LayerStructure>,
    netset: Vec<(NetNo, String)>,
}

impl IncompleteCountWitness {
    fn new(board: &BoardAdapter) -> Self {
        Self {
            item_epoch: board.board.item_repository().epoch(),
            global_epoch: board.board.item_repository().global_epoch(),
            rules: board.board.rules.clone(),
            library: board.board.library.clone(),
            components: board.board.components.clone(),
            layers: board.board.layer_structure.clone(),
            netset: board
                .board
                .rules
                .nets
                .iter()
                .map(|net| (net.net_number, net.name.clone()))
                .collect(),
        }
    }

    fn permits_affected_refresh(&self, board: &BoardAdapter) -> bool {
        let netset_matches =
            self.netset.len() == board.board.rules.nets.max_net_number() as usize
            && self
                .netset
                .iter()
                .zip(board.board.rules.nets.iter())
                .all(|((number, name), net)| *number == net.net_number && *name == net.name);
        self.item_epoch != board.board.item_repository().epoch()
            && self.global_epoch == board.board.item_repository().global_epoch()
            && Arc::ptr_eq(&self.rules, &board.board.rules)
            && Arc::ptr_eq(&self.library, &board.board.library)
            && Arc::ptr_eq(&self.components, &board.board.components)
            && Arc::ptr_eq(&self.layers, &board.board.layer_structure)
            && netset_matches
    }
}

fn full_incomplete_counts(board: &BoardAdapter) -> (BTreeMap<NetNo, i32>, i32) {
    let counts: BTreeMap<NetNo, i32> = board
        .board
        .rules
        .nets
        .iter()
        .map(|net| {
            (
                net.net_number,
                fr_engine::pipeline::stats::incomplete_count_for_net(&board.board, net.net_number),
            )
        })
        .collect();
    let total = counts
        .values()
        .copied()
        .fold(0i32, i32::wrapping_add);
    (counts, total)
}

#[derive(Default)]
struct DriverProfile {
    nets: BTreeMap<String, BTreeMap<String, ProfileCounter>>,
    record_best: BTreeMap<String, RecordBestMetrics>,
}

#[derive(Default)]
struct RecordBestMetrics {
    incomplete_count_calls: u64,
    incomplete_count_elapsed_ns: u128,
    full_refresh_calls: u64,
    affected_refresh_calls: u64,
    refreshed_net_count: u64,
    cache_fullcheck_calls: u64,
    cache_fullcheck_mismatches: u64,
    cache_fullcheck_last_diff: i64,
    board_clone_calls: u64,
    board_clone_elapsed_ns: u128,
    best_not_improved_calls: u64,
}

#[derive(Default)]
struct ProfileCounter {
    calls: u64,
    elapsed_ns: u128,
}

impl DriverProfile {
    fn add(&mut self, net: &str, stage: &str, elapsed: Duration) {
        let counter = self
            .nets
            .entry(net.to_string())
            .or_default()
            .entry(stage.to_string())
            .or_default();
        counter.calls += 1;
        counter.elapsed_ns += elapsed.as_nanos();
    }

    fn json(&self) -> serde_json::Value {
        let mut nets = serde_json::Map::new();
        for (net, stages) in &self.nets {
            let mut stage_values = serde_json::Map::new();
            for (stage, counter) in stages {
                stage_values.insert(stage.clone(), serde_json::json!({
                    "calls": counter.calls,
                    "elapsed_ns": counter.elapsed_ns,
                    "inclusive": matches!(stage.as_str(), "route_net_fully" | "phase1_route" | "phase3_taps" | "drain_reroute_queue" | "recover_edge" | "route_endpoint_set"),
                }));
            }
            nets.insert(net.clone(), serde_json::Value::Object(stage_values));
        }
        serde_json::Value::Object(nets)
    }

    fn record_best_json(&self) -> serde_json::Value {
        let record_best = self.record_best.iter().map(|(net, metrics)| {
            (net.clone(), serde_json::json!({
                "incomplete_count_calls":metrics.incomplete_count_calls,
                "incomplete_count_elapsed_ns":metrics.incomplete_count_elapsed_ns,
                "full_refresh_calls":metrics.full_refresh_calls,
                "affected_refresh_calls":metrics.affected_refresh_calls,
                "refreshed_net_count":metrics.refreshed_net_count,
                "cache_fullcheck_calls":metrics.cache_fullcheck_calls,
                "cache_fullcheck_mismatches":metrics.cache_fullcheck_mismatches,
                "cache_fullcheck_last_diff":metrics.cache_fullcheck_last_diff,
                "board_clone_calls":metrics.board_clone_calls,
                "board_clone_elapsed_ns":metrics.board_clone_elapsed_ns,
                "best_not_improved_calls":metrics.best_not_improved_calls
            }))
        }).collect::<serde_json::Map<_, _>>();
        serde_json::Value::Object(record_best)
    }
}

/// Pop the native backend's PRIVATE `_via_unblock_blame` key from a route
/// report. The value is the source `pcb_data._via_unblock_blame` set
/// (`single_ended_loop.py:1034..1071`): the native net numbers of the committed
/// foreign Trace/Via copper that boxes a shrunk via's pad on a declined tap.
/// The key is private telemetry, so it is removed here -- before the report is
/// ever logged or stored as an attempt -- and never becomes a report-schema or
/// CLI field. `Some(set)` means the key was present (the backend recorded a real
/// keystone, possibly empty); `None` means it was absent, so an existing
/// standing blame is left untouched rather than masked by an empty absence. A
/// malformed value yields `Some(empty)`, never a fabricated net.
fn take_via_unblock_blame(report: &mut serde_json::Value) -> Option<BTreeSet<NetNo>> {
    let raw = report.as_object_mut()?.remove("_via_unblock_blame")?;
    Some(
        raw.as_array()
            .into_iter()
            .flatten()
            .filter_map(|value| value.as_i64())
            .filter_map(|number| i32::try_from(number).ok())
            .collect(),
    )
}

impl Driver<'_> {
    /// Snapshot `board` as the run's best consistent state when its actual
    /// copper has strictly fewer remaining incompletes. Callers provide the
    /// complete set of nets changed by the accepted transaction.
    fn record_best(
        &mut self,
        board: &BoardAdapter,
        net_name: &str,
        changed_nets: &[NetNo],
    ) -> Result<(), String> {
        let started = Instant::now();
        let count_started = Instant::now();
        let affected: BTreeSet<NetNo> = changed_nets.iter().copied().collect();
        let affected_refresh = !affected.is_empty()
            && affected.iter().all(|net| self.incomplete_counts.contains_key(net))
            && self.count_witness.permits_affected_refresh(board);
        if affected_refresh {
            for net in &affected {
                let previous = self.incomplete_counts[net];
                let count = fr_engine::pipeline::stats::incomplete_count_for_net(&board.board, *net);
                self.incomplete_counts.insert(*net, count);
                self.incomplete_total = self
                    .incomplete_total
                    .wrapping_sub(previous)
                    .wrapping_add(count);
            }
        } else {
            (self.incomplete_counts, self.incomplete_total) = full_incomplete_counts(board);
        }
        self.count_witness = IncompleteCountWitness::new(board);
        let count = self.incomplete_total;
        let count_elapsed = count_started.elapsed();
        let metrics = self.profile.record_best.entry(net_name.to_string()).or_default();
        metrics.incomplete_count_calls += 1;
        metrics.incomplete_count_elapsed_ns += count_elapsed.as_nanos();
        if affected_refresh {
            metrics.affected_refresh_calls += 1;
            metrics.refreshed_net_count += affected.len() as u64;
        } else {
            metrics.full_refresh_calls += 1;
            metrics.refreshed_net_count += self.incomplete_counts.len() as u64;
        }
        self.record_best_calls += 1;
        if self.record_best_calls % RECORD_BEST_FULLCHECK_EVERY == 0 {
            let full_count_started = Instant::now();
            let full_count =
                fr_engine::pipeline::stats::incomplete_count(&board.board, None);
            metrics.cache_fullcheck_calls += 1;
            metrics.cache_fullcheck_last_diff = i64::from(full_count) - i64::from(count);
            if full_count != count {
                metrics.cache_fullcheck_mismatches += 1;
                metrics.incomplete_count_elapsed_ns += full_count_started.elapsed().as_nanos();
                return Err(format!(
                    "record_best net-count cache disagrees with full count: incremental={count}, full={full_count}"
                ));
            }
            metrics.incomplete_count_elapsed_ns += full_count_started.elapsed().as_nanos();
        }
        if count < self.best_incompletes {
            let clone_started = Instant::now();
            self.best_board = board.clone();
            metrics.board_clone_calls += 1;
            metrics.board_clone_elapsed_ns += clone_started.elapsed().as_nanos();
            self.best_incompletes = count;
        } else {
            metrics.best_not_improved_calls += 1;
        }
        self.profile.add(net_name, "record_best", started.elapsed());
        Ok(())
    }

    /// One native endpoint-set route, logging the source attempt_start/end pair
    /// and returning the landed board (if any) plus the failed-route frontier for
    /// recovery.
    #[allow(clippy::too_many_arguments)]
    fn route_endpoint_set(
        &mut self,
        board: &BoardAdapter,
        net: NetNo,
        name: &str,
        stage: &str,
        from_item: ItemKey,
        to_item: ItemKey,
        starts: &[RungEndpoint],
        targets: &[RungEndpoint],
        route_layers: &[(i32, String)],
    ) -> Result<(Option<BoardAdapter>, Option<RungReport>), String> {
        let route_started = Instant::now();
        if starts.is_empty() || targets.is_empty() {
            return Ok((None, None));
        }
        let build_started = Instant::now();
        let request = build_endpoint_request(
            board,
            &self.ctx,
            net,
            name,
            stage,
            starts[0].clone(),
            targets[0].clone(),
            route_layers,
        )?;
        self.profile
            .add(name, "build_endpoint_request", build_started.elapsed());
        let mut event = serde_json::json!({"event":"attempt_start","net":name,"stage":stage,
            "from_item":board.board.item(from_item).id().0,"to_item":board.board.item(to_item).id().0,
            "start":starts[0].point,"end":targets[0].point,
            "start_layers":starts.iter().map(|e| e.layer.clone()).collect::<Vec<_>>(),
            "end_layers":targets.iter().map(|e| e.layer.clone()).collect::<Vec<_>>(),
            "scheduler":"native NetIncompletes Delaunay/Kruskal airlines",
            "width_mm":request.rules.track_width,"clearance_mm":request.rules.clearance,
            "algorithm":"awx_grid::GridRouter","new_vias_allowed":true,
            "new_via_drill_source":"explicit job rule",
            "existing_pad_drill_source":"exact native padstack electrical copper; DSN drill unknown where absent",
            "via_drill_mm":request.rules.via_drill});
        let log_started = Instant::now();
        log_event(&mut self.log, &event)?;
        self.profile
            .add(name, "attempt_log_write", log_started.elapsed());
        let calls_before = endpoint_grid_calls();
        let (candidate, mut report, frontier) = match route_multilayer_endpoint_sets_in_run(
            request,
            from_item,
            to_item,
            starts,
            targets,
            self.baseline,
            self.protected,
        ) {
            Ok(v) => v,
            Err(e) => (None, serde_json::json!({"accepted":false,"reason":e}), None),
        };
        report["grid_router_calls"] = (endpoint_grid_calls() - calls_before).into();
        // ORIGINAL source keystone: take the private `_via_unblock_blame` key OUT
        // of the report BEFORE it is logged or kept as an attempt, then hand this
        // edge's blame to the rip custody. `set_attempt_blame` REPLACES the
        // standing blame, so every edge (and every attempt) starts from its own
        // keystone -- an attempt without the key clears the blame rather than
        // carrying a stale one forward.
        self.recovery
            .set_attempt_blame(take_via_unblock_blame(&mut report).unwrap_or_default());
        event["event"] = "attempt_end".into();
        event["elapsed_ns"] = serde_json::json!(route_started.elapsed().as_nanos());
        event["result"] = report;
        let log_started = Instant::now();
        log_event(&mut self.log, &event)?;
        self.profile
            .add(name, "attempt_log_write", log_started.elapsed());
        self.profile
            .add(name, "route_endpoint_set", route_started.elapsed());
        self.attempts.push(event);
        Ok((candidate, frontier))
    }

    /// Hand a failed-route frontier to the source rip-up custody module. The
    /// `retry` closure re-runs the SAME edge on the trial board; rejected trials
    /// leave `self` (board, attempts, accepted) untouched. On acceptance the
    /// victims are queued for the driver's reroute pass.
    #[allow(clippy::too_many_arguments)]
    fn recover_edge<T: Clone>(
        &mut self,
        board: &BoardAdapter,
        net: NetNo,
        name: &str,
        from_item: ItemKey,
        to_item: ItemKey,
        starts: &[RungEndpoint],
        targets: &[RungEndpoint],
        route_layers: &[(i32, String)],
        frontier: &RungReport,
        target_data: T,
    ) -> Result<Option<RipAccepted<T>>, String> {
        let recover_started = Instant::now();
        let ctx = self.ctx.clone();
        let baseline = self.baseline;
        let protected = self.protected;
        let net_name = name.to_string();
        let retry_profile = Arc::new(Mutex::new(Vec::<u128>::new()));
        let retry_profile_in = retry_profile.clone();
        let starts0 = starts.to_vec();
        let targets0 = targets.to_vec();
        let route_layers0 = route_layers.to_vec();
        let retry = move |trial: &BoardAdapter| {
            let build_started = Instant::now();
            let request = build_endpoint_request(
                trial,
                &ctx,
                net,
                &net_name,
                "source_rip_retry",
                starts0[0].clone(),
                targets0[0].clone(),
                &route_layers0,
            )?;
            retry_profile_in
                .lock()
                .unwrap()
                .push(build_started.elapsed().as_nanos());
            let (candidate, mut report, fresh) = route_multilayer_endpoint_sets_in_run(
                request, from_item, to_item, &starts0, &targets0, baseline, protected,
            )?;
            // A fresh retry report that emitted its own `_via_unblock_blame`
            // keystone is carried back so the ladder can keep the NEWEST blame
            // (an absent key returns `None` and leaves the standing blame
            // untouched, never resurrecting a stale promise).
            let blame = take_via_unblock_blame(&mut report);
            Ok(match candidate {
                Some(candidate) => (Some((candidate, target_data.clone())), None, None),
                None => (None, fresh, blame),
            })
        };
        // Preserve the actual attempt endpoints in millimeters for the source
        // frontier-blocking proximity tiebreak; the analyzer performs the grid
        // conversion once using its report's grid step.
        let source_xy = starts.first().map(|endpoint| endpoint.point);
        let target_xy = targets.first().map(|endpoint| endpoint.point);
        self.recovery.set_attempt_endpoints(source_xy, target_xy);
        let accepted = try_source_ripup(
            board,
            baseline,
            net,
            frontier,
            protected,
            &mut self.recovery,
            retry,
        )?;
        if let Ok(retry_builds) = retry_profile.lock() {
            for elapsed_ns in retry_builds.iter() {
                self.profile.add(
                    name,
                    "retry_build_endpoint_request",
                    Duration::from_nanos((*elapsed_ns).min(u64::MAX as u128) as u64),
                );
            }
        }
        if let Some(accepted) = &accepted {
            let event = serde_json::json!({"event":"rip_accepted","net":name,
                "victims":accepted.victims});
            log_event(&mut self.log, &event)?;
            // Source pending custody: every victim's main is this-run copper the
            // rip just tore, so its pending plan/routed state is now phantom.
            // Invalidate it HERE, at rip time, so the phase-3 tap pass can never
            // consult a stale entry before the queue reroutes the victim.
            for victim in &accepted.victims {
                self.pending.retain(|entry| entry.net != *victim);
            }
        }
        self.profile
            .add(name, "recover_edge", recover_started.elapsed());
        Ok(accepted)
    }

    /// Source `route_multipoint_main` phase 1 plus the ordinary 2-terminal path.
    fn phase1_route(
        &mut self,
        board: &mut BoardAdapter,
        net: NetNo,
        name: &str,
        route_layers: &[(i32, String)],
    ) -> Result<Phase1Outcome, String> {
        let started = Instant::now();
        let result = self.phase1_route_inner(board, net, name, route_layers);
        self.profile.add(name, "phase1_route", started.elapsed());
        result
    }

    fn phase1_route_inner(
        &mut self,
        board: &mut BoardAdapter,
        net: NetNo,
        name: &str,
        route_layers: &[(i32, String)],
    ) -> Result<Phase1Outcome, String> {
        let plan = plan_native_multipoint(board, net, route_layers)?;
        let Some(plan) = plan else {
            let routed = self.plain_route(board, net, name, route_layers)?;
            return Ok(if routed {
                Phase1Outcome::Plain
            } else {
                Phase1Outcome::Failed
            });
        };
        if plan.edges.is_empty() {
            return Ok(Phase1Outcome::Plain);
        }
        let max_attempts = plan.edges.len().min(MAX_MAIN_ATTEMPTS);
        let mut last_frontier: Option<RungReport> = None;
        let mut last_edge: Option<(usize, usize)> = None;
        for attempt in 0..max_attempts {
            if Instant::now() >= self.ctx.deadline {
                break;
            }
            let (a, b, _len) = plan.edges[attempt];
            last_edge = Some((a, b));
            let starts = terminal_island_candidates(board, net, &plan.terminals[a], route_layers);
            let targets = terminal_island_candidates(board, net, &plan.terminals[b], route_layers);
            if starts.is_empty() || targets.is_empty() {
                continue;
            }
            let (candidate, frontier) = self.route_endpoint_set(
                board,
                net,
                name,
                "multipoint_phase1_main",
                plan.terminals[a].item,
                plan.terminals[b].item,
                &starts,
                &targets,
                route_layers,
            )?;
            if let Some(new_board) = candidate {
                *board = new_board;
                self.accepted += 1;
                self.record_best(board, name, &[net])?;
                let mut routed = BTreeSet::new();
                routed.insert(a);
                routed.insert(b);
                return Ok(Phase1Outcome::Pending { plan, routed });
            }
            last_frontier = frontier;
        }
        // Recovery on the last edge's frontier before declaring phase 1 boxed in.
        if let (Some(frontier), Some((a, b))) = (last_frontier, last_edge) {
            let starts = terminal_island_candidates(board, net, &plan.terminals[a], route_layers);
            let targets = terminal_island_candidates(board, net, &plan.terminals[b], route_layers);
            let mut data = BTreeSet::new();
            data.insert(a);
            data.insert(b);
            if let Some(accepted) = self.recover_edge(
                board,
                net,
                name,
                plan.terminals[a].item,
                plan.terminals[b].item,
                &starts,
                &targets,
                route_layers,
                &frontier,
                data,
            )? {
                *board = accepted.board;
                self.accepted += 1;
                let mut changed_nets = accepted.victims.clone();
                changed_nets.push(net);
                self.record_best(board, name, &changed_nets)?;
                return Ok(Phase1Outcome::Pending {
                    plan,
                    routed: accepted.target_data,
                });
            }
        }
        // Source #348: every main-edge attempt launched from PAD cells failed.
        // When existing copper already joins terminals, hand phase 3 a synthetic
        // empty main result rooted at the LARGEST copper-joined component (never
        // a fabricated singleton).
        if net_has_existing_copper(board, net) {
            if let Some(base) = copper_richest_component(board, net, &plan, self.ctx.grid_step_mm)?
            {
                if !base.is_empty() {
                    return Ok(Phase1Outcome::Pending { plan, routed: base });
                }
            }
        }
        Ok(Phase1Outcome::Failed)
    }

    /// Ordinary 2-terminal single-ended route on the net's full-Cu connected
    /// groups. Recomputed after every acceptance so group candidates reflect the
    /// new board.
    fn plain_route(
        &mut self,
        board: &mut BoardAdapter,
        net: NetNo,
        name: &str,
        route_layers: &[(i32, String)],
    ) -> Result<bool, String> {
        loop {
            if Instant::now() >= self.ctx.deadline {
                return Ok(false);
            }
            let current =
                NetIncompletes::new(&board.board, net, &board.board.get_connectable_items(net));
            if current.count() == 0 {
                return Ok(true);
            }
            let mut committed = false;
            let mut tried: BTreeSet<(Vec<i32>, Vec<i32>)> = BTreeSet::new();
            for edge in &current.incompletes {
                let from_group = board.board.connected_set(edge.from_item, net, false);
                let to_group = board.board.connected_set(edge.to_item, net, false);
                let starts = connected_group_candidates(board, &from_group, route_layers);
                let targets = connected_group_candidates(board, &to_group, route_layers);
                if starts.is_empty() || targets.is_empty() {
                    continue;
                }
                let mut from_members: Vec<i32> = from_group
                    .iter()
                    .map(|member| board.board.item(member).id().0)
                    .collect();
                from_members.sort_unstable();
                let mut to_members: Vec<i32> = to_group
                    .iter()
                    .map(|member| board.board.item(member).id().0)
                    .collect();
                to_members.sort_unstable();
                let mut group_key = (from_members, to_members);
                if group_key.1 < group_key.0 {
                    group_key = (group_key.1, group_key.0);
                }
                if !tried.insert(group_key) {
                    continue;
                }
                if Instant::now() >= self.ctx.deadline {
                    return Ok(false);
                }
                let (candidate, frontier) = self.route_endpoint_set(
                    board,
                    net,
                    name,
                    "ordinary_dsn_multilayer",
                    edge.from_item,
                    edge.to_item,
                    &starts,
                    &targets,
                    route_layers,
                )?;
                if let Some(new_board) = candidate {
                    *board = new_board;
                    self.accepted += 1;
                    self.record_best(board, name, &[net])?;
                    committed = true;
                    break;
                }
                if let Some(frontier) = frontier {
                    if let Some(accepted) = self.recover_edge(
                        board,
                        net,
                        name,
                        edge.from_item,
                        edge.to_item,
                        &starts,
                        &targets,
                        route_layers,
                        &frontier,
                        (),
                    )? {
                        *board = accepted.board;
                        self.accepted += 1;
                        let mut changed_nets = accepted.victims.clone();
                        changed_nets.push(net);
                        self.record_best(board, name, &changed_nets)?;
                        committed = true;
                        break;
                    }
                }
            }
            if !committed {
                return Ok(false);
            }
        }
    }

    /// Source phase-3 tap pass for one pending multipoint net (source
    /// `single_ended_routing.route_multipoint_taps` remaining-edge loop
    /// =4902..=4959).
    ///
    /// Edges are taken in the remaining original MST order (longest-first) but
    /// REPEATEDLY: each pass routes the FIRST eligible edge -- one whose two
    /// PHASE-1 components are not both routed and not both unrouted (the XOR
    /// rule) -- until no edge is eligible or every terminal is connected,
    /// bounded by `len(edges) * 2 + len(terminals)` passes. Absorbing a branch
    /// therefore unlocks edges that were ineligible earlier in the same pass
    /// (AB main, then BC, then revisit CD). When no edge is eligible, an
    /// orphaned terminal is attempted directly from its NEAREST routed pad once
    /// (the source fallback that feeds real blockers to the rip ladder).
    ///
    /// The plan labels seed source-compatible eligibility, but the current
    /// board's authoritative connectivity is refreshed each pass. A previous
    /// tap can physically join an intermediate terminal while routing to a
    /// different target; leaving that terminal under its stale phase-1 label
    /// would issue a zero-cell retry which the native transaction correctly
    /// rejects for not reducing incompletes. Treat a terminal as routed when
    /// its CURRENT copper component contains any explicitly routed terminal.
    fn phase3_taps(
        &mut self,
        board: &mut BoardAdapter,
        net: NetNo,
        name: &str,
        plan: &MultipointPlan,
        routed: &mut BTreeSet<usize>,
        route_layers: &[(i32, String)],
    ) -> Result<(), String> {
        let started = Instant::now();
        let result = self.phase3_taps_inner(board, net, name, plan, routed, route_layers);
        self.profile.add(name, "phase3_taps", started.elapsed());
        result
    }

    fn phase3_taps_inner(
        &mut self,
        board: &mut BoardAdapter,
        net: NetNo,
        name: &str,
        plan: &MultipointPlan,
        routed: &mut BTreeSet<usize>,
        route_layers: &[(i32, String)],
    ) -> Result<(), String> {
        let mut remaining: Vec<(usize, usize, f64)> = plan.edges.clone();
        let mut failed_edges: BTreeSet<(usize, usize)> = BTreeSet::new();
        let mut fallback_attempted: BTreeSet<usize> = BTreeSet::new();
        // Source `max_passes = len(remaining_edges) * 2 + len(pad_info)`.
        let max_passes = plan.edges.len() * 2 + plan.terminals.len();

        for _pass in 0..max_passes {
            if Instant::now() >= self.ctx.deadline {
                break;
            }
            let current_components =
                crate::awx::net_multipoint::canonical_components(board, net, &plan.terminals)?;
            let effectively_routed = |index: usize| {
                routed.contains(&index)
                    || routed
                        .iter()
                        .any(|&r| current_components[r] == current_components[index])
            };
            let mut all_routed = true;
            for i in 0..plan.terminals.len() {
                if !effectively_routed(i) {
                    all_routed = false;
                    break;
                }
            }
            if all_routed {
                break; // All pads connected.
            }

            // The Python phase-3 source union is the copper of every currently
            // routed component. Keep that union (rather than just this edge's
            // anchor), while excluding disconnected same-net islands. Retain
            // one terminal representative per component in first-terminal order.
            let source_component_ids = routed
                .iter()
                .map(|&index| current_components[index])
                .collect::<BTreeSet<_>>();
            let mut seen_source_components = BTreeSet::new();
            let source_terminal_indices = (0..plan.terminals.len())
                .filter(|&index| {
                    source_component_ids.contains(&current_components[index])
                        && seen_source_components.insert(current_components[index])
                })
                .collect::<Vec<_>>();

            // First eligible remaining edge (XOR), skipping edges that failed.
            let mut edge: Option<(usize, usize)> = None;
            for &(a, b, _len) in &remaining {
                let key = if a < b { (a, b) } else { (b, a) };
                if failed_edges.contains(&key) {
                    continue;
                }
                let a_routed = effectively_routed(a);
                let b_routed = effectively_routed(b);
                if a_routed && !b_routed {
                    edge = Some((a, b));
                    break;
                }
                if b_routed && !a_routed {
                    edge = Some((b, a));
                    break;
                }
            }

            // Source orphan fallback: attempt each stranded pad directly once,
            // from its nearest routed pad (Manhattan, strict `<`).
            if edge.is_none() {
                let mut best: Option<(usize, usize, f64)> = None;
                for i in 0..plan.terminals.len() {
                    if effectively_routed(i) || fallback_attempted.contains(&i) {
                        continue;
                    }
                    let (xi, yi) = (plan.terminals[i].point_mm[0], plan.terminals[i].point_mm[1]);
                    for &j in routed.iter() {
                        let d = (xi - plan.terminals[j].point_mm[0]).abs()
                            + (yi - plan.terminals[j].point_mm[1]).abs();
                        if best.map_or(true, |(_, _, current)| d < current) {
                            best = Some((j, i, d));
                        }
                    }
                }
                if let Some((src, tgt, _d)) = best {
                    fallback_attempted.insert(tgt);
                    edge = Some((src, tgt));
                }
            }

            let Some((src_idx, tgt_idx)) = edge else {
                break;
            };

            let primary_source_component = current_components[src_idx];
            let mut source_component_item_ids = BTreeSet::new();
            let mut primary_source_candidates = Vec::new();
            let mut additional_source_candidates = Vec::new();
            for &source_index in &source_terminal_indices {
                let group =
                    board
                        .board
                        .connected_set(plan.terminals[source_index].item, net, false);
                for member in group.iter() {
                    source_component_item_ids.insert(board.board.item(member).id().0);
                }
                let component_id = current_components[source_index];
                let component_terminal = if component_id == primary_source_component {
                    &plan.terminals[src_idx]
                } else {
                    &plan.terminals[source_index]
                };
                let component_candidates = terminal_island_candidates_in_group(
                    board,
                    &group,
                    component_terminal,
                    route_layers,
                );
                let destination = if component_id == primary_source_component {
                    &mut primary_source_candidates
                } else {
                    &mut additional_source_candidates
                };
                for endpoint in component_candidates {
                    if !destination.iter().any(|existing: &RungEndpoint| {
                        existing.layer == endpoint.layer && existing.point == endpoint.point
                    }) {
                        destination.push(endpoint);
                    }
                }
            }
            let sources = growing_source_candidates(
                board,
                self.baseline,
                net,
                &primary_source_candidates,
                &additional_source_candidates,
                &source_component_item_ids,
                route_layers,
                self.ctx.grid_step_mm,
            );
            let targets =
                terminal_island_candidates(board, net, &plan.terminals[tgt_idx], route_layers);
            // Source `if not sources: continue` -- NOT marked failed.
            if sources.is_empty() || targets.is_empty() {
                continue;
            }

            let edge_key = if src_idx < tgt_idx {
                (src_idx, tgt_idx)
            } else {
                (tgt_idx, src_idx)
            };
            let (candidate, frontier) = self.route_endpoint_set(
                board,
                net,
                name,
                "multipoint_phase3_tap",
                plan.terminals[src_idx].item,
                plan.terminals[tgt_idx].item,
                &sources,
                &targets,
                route_layers,
            )?;
            if let Some(new_board) = candidate {
                *board = new_board;
                self.accepted += 1;
                self.record_best(board, name, &[net])?;
                routed.insert(src_idx);
                routed.insert(tgt_idx);
                remaining.retain(|&(x, y, _)| {
                    !((x == src_idx && y == tgt_idx) || (x == tgt_idx && y == src_idx))
                });
                continue;
            }
            let mut recovered = false;
            if let Some(frontier) = frontier {
                let mut data = BTreeSet::new();
                data.insert(src_idx);
                data.insert(tgt_idx);
                if let Some(accepted) = self.recover_edge(
                    board,
                    net,
                    name,
                    plan.terminals[src_idx].item,
                    plan.terminals[tgt_idx].item,
                    &sources,
                    &targets,
                    route_layers,
                    &frontier,
                    data,
                )? {
                    *board = accepted.board;
                    self.accepted += 1;
                    let mut changed_nets = accepted.victims.clone();
                    changed_nets.push(net);
                    self.record_best(board, name, &changed_nets)?;
                    routed.insert(src_idx);
                    routed.insert(tgt_idx);
                    remaining.retain(|&(x, y, _)| {
                        !((x == src_idx && y == tgt_idx) || (x == tgt_idx && y == src_idx))
                    });
                    recovered = true;
                }
            }
            if !recovered {
                // Source `failed_edges.add(edge_key)`: this edge is never
                // re-selected; the pass loop moves on to the next eligible edge.
                failed_edges.insert(edge_key);
            }
        }
        Ok(())
    }

    /// Route one net end to end (main + immediate taps). Used for the rip
    /// victims the ladder queued, which the source reroutes after the main stage
    /// and again after the tap casualties.
    fn route_net_fully(
        &mut self,
        board: &mut BoardAdapter,
        net: NetNo,
        name: &str,
        route_layers: &[(i32, String)],
    ) -> Result<bool, String> {
        let started = Instant::now();
        let result = self.route_net_fully_inner(board, net, name, route_layers);
        self.profile.add(name, "route_net_fully", started.elapsed());
        result
    }

    fn route_net_fully_inner(
        &mut self,
        board: &mut BoardAdapter,
        net: NetNo,
        name: &str,
        route_layers: &[(i32, String)],
    ) -> Result<bool, String> {
        match self.phase1_route(board, net, name, route_layers)? {
            Phase1Outcome::Plain => Ok(true),
            Phase1Outcome::Failed => Ok(false),
            Phase1Outcome::Pending { plan, routed } => {
                let mut routed = routed;
                self.phase3_taps(board, net, name, &plan, &mut routed, route_layers)?;
                let items = board.board.get_connectable_items(net);
                Ok(NetIncompletes::new(&board.board, net, &items).count() == 0)
            }
        }
    }

    /// Reroute the victims queued by every accepted rip, in FIFO rip order,
    /// each with its main edge and immediate taps. Bounded by the run deadline.
    fn drain_reroute_queue(&mut self, board: &mut BoardAdapter) -> Result<(), String> {
        let started = Instant::now();
        let result = self.drain_reroute_queue_inner(board);
        self.profile
            .add("<queue>", "drain_reroute_queue", started.elapsed());
        result
    }

    fn drain_reroute_queue_inner(&mut self, board: &mut BoardAdapter) -> Result<(), String> {
        while let Some((victim, saved_board)) = self.recovery.pop_queued_victim() {
            let Some(name) = board
                .board
                .rules
                .nets
                .get(victim)
                .map(|net| net.name.clone())
            else {
                continue;
            };
            if self.protected.contains(&name) {
                continue;
            }
            let route_layers = net_route_layers(board, victim);
            // Source pending custody (`phase3_routing._reroute_phase3_ripped_nets`):
            // a ripped victim's main copper was this-run copper, so its pending
            // entry still references the torn main. Invalidate it BEFORE the
            // reroute -- on a reroute failure no phantom main is left behind for
            // the tap pass to launch from.
            self.pending.retain(|entry| entry.net != victim);
            let rerouted = Instant::now() < self.ctx.deadline
                && !route_layers.is_empty()
                && self.route_net_fully(board, victim, &name, &route_layers)?;
            if !rerouted {
                let mut restored = board.clone();
                let names = BTreeSet::from([name.clone()]);
                if crate::awx::native_execution::restore_nets(&saved_board, &mut restored, &names)
                    .is_ok()
                    && restored
                        .assert_invariants(self.baseline, self.protected, true)
                        .is_ok()
                {
                    *board = restored;
                    self.record_best(board, &name, &[victim])?;
                }
            }
        }
        Ok(())
    }
}

/// Build the native endpoint request for one edge. The rules are taken from the
/// edge's launch layer; the native backend refines the search itself.
#[allow(clippy::too_many_arguments)]
fn build_endpoint_request(
    board: &BoardAdapter,
    ctx: &EndpointContext,
    net: NetNo,
    name: &str,
    stage: &str,
    start: RungEndpoint,
    end: RungEndpoint,
    route_layers: &[(i32, String)],
) -> Result<RungRequest, String> {
    let rule_layer = route_layers
        .iter()
        .find(|(_, layer)| *layer == start.layer)
        .map(|(no, _)| *no)
        .unwrap_or(route_layers[0].0);
    let rules = rules_for_layer(
        board,
        net,
        rule_layer,
        ctx.grid_step_mm,
        ctx.via_drill_mm,
        ctx.min_annular_ring_mm,
        ctx.hole_to_hole_clearance_mm,
    )?;
    Ok(RungRequest {
        board: board.clone(),
        net: name.to_string(),
        net_id: net,
        stage: stage.to_string(),
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
        free_via_pads: ctx.free_via_pads.clone(),
        same_net_pads: ctx.same_net_pads.clone(),
        existing_vias: crate::awx::board_snapshot::BoardSnapshot::capture_vias(
            board,
            &BTreeMap::new(),
        )?,
        paste_pads: BTreeMap::new(),
        rules,
        fabrication: FabTier::Standard,
        escalation: EscalationPolicy::Off,
        fabrication_overrides: BTreeMap::new(),
        fab_clearance_floor_mm: 0.0,
        fab_track_floor_mm: 0.0,
        may_narrow: false,
        fab_via_rungs: Vec::new(),
        max_iterations: ctx.max_iterations,
        route_layers: route_layers.iter().map(|(_, n)| n.clone()).collect(),
        deadline: Some(ctx.deadline),
    })
}

/// The net's native active routing layers, in stack order.
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

/// Terminals of one endpoint's own copper island plus the exact terminal anchor
/// (the source owner-weld contract). The anchor is only offered at the
/// terminal's own launch layer.
fn terminal_island_candidates(
    board: &BoardAdapter,
    net: NetNo,
    terminal: &MultipointTerminal,
    route_layers: &[(i32, String)],
) -> Vec<RungEndpoint> {
    let group = board.board.connected_set(terminal.item, net, false);
    terminal_island_candidates_in_group(board, &group, terminal, route_layers)
}

fn terminal_island_candidates_in_group(
    board: &BoardAdapter,
    group: &ItemSet,
    terminal: &MultipointTerminal,
    route_layers: &[(i32, String)],
) -> Vec<RungEndpoint> {
    let mut candidates = connected_group_candidates(board, group, route_layers);
    let anchor = RungEndpoint {
        point: terminal.point_mm,
        layer: terminal.layer.clone(),
    };
    if !candidates
        .iter()
        .any(|endpoint| endpoint.layer == anchor.layer && endpoint.point == anchor.point)
    {
        candidates.insert(0, anchor);
    }
    candidates
}

/// Growing Cu-source launch set for a phase-3 tap: terminals and every interior
/// tap of a THIS-RUN trace in the currently routed component union. The native
/// backend may split those traces at the projected point. Imported/protected
/// copper contributes no interior tap because it may never be split.
fn growing_source_candidates(
    board: &BoardAdapter,
    baseline: &BoardAdapter,
    net: NetNo,
    primary_source_candidates: &[RungEndpoint],
    additional_source_candidates: &[RungEndpoint],
    source_component_item_ids: &BTreeSet<i32>,
    route_layers: &[(i32, String)],
    grid_step_mm: f64,
) -> Vec<RungEndpoint> {
    let mut candidates = primary_source_candidates.to_vec();
    // Preserve the selected source terminal's and its tap-sample candidates'
    // existing order first.
    for endpoint in this_run_trace_tap_points(
        board,
        baseline,
        net,
        source_component_item_ids,
        route_layers,
        grid_step_mm,
    ) {
        if !candidates
            .iter()
            .any(|existing| existing.layer == endpoint.layer && existing.point == endpoint.point)
        {
            candidates.push(endpoint);
        }
    }
    // Python phase 3 unions island cells from every routed component after
    // the main-result segment samples. These candidates were collected once
    // per component from its representative terminal.
    for endpoint in additional_source_candidates {
        if !candidates
            .iter()
            .any(|existing| existing.layer == endpoint.layer && existing.point == endpoint.point)
        {
            candidates.push(endpoint.clone());
        }
    }
    candidates
}

/// Source `get_all_segment_tap_points` sampling restricted to this run's copper
/// in the routed-component union: exact samples at
/// `max(1, int(length / grid_step))` steps with both vertices included, plus
/// this-run vias repeated on every physical Cu route layer. The first writer's
/// exact owner coord is kept per grid cell.
fn this_run_trace_tap_points(
    board: &BoardAdapter,
    baseline: &BoardAdapter,
    net: NetNo,
    source_component_item_ids: &BTreeSet<i32>,
    route_layers: &[(i32, String)],
    grid_step_mm: f64,
) -> Vec<RungEndpoint> {
    let upm = board.units_per_mm;
    let baseline_ids: BTreeSet<i32> = baseline
        .board
        .get_traces()
        .into_iter()
        .chain(baseline.board.get_vias())
        .map(|key| baseline.board.item(key).id().0)
        .collect();
    let mut cells: BTreeMap<(i64, i64, usize), [f64; 2]> = BTreeMap::new();
    for key in board.board.get_traces() {
        let item = board.board.item(key);
        if !item.contains_net(net)
            || baseline_ids.contains(&item.id().0)
            || !source_component_item_ids.contains(&item.id().0)
        {
            continue;
        }
        let layer_no = item.trace().layer();
        let Some(index) = route_layers.iter().position(|(no, _)| *no == layer_no) else {
            continue;
        };
        let corners = item.trace().polyline().corner_approx_arr();
        for window in corners.windows(2) {
            let x0 = window[0].x as f64 / upm;
            let y0 = window[0].y as f64 / upm;
            let x1 = window[1].x as f64 / upm;
            let y1 = window[1].y as f64 / upm;
            let (dx, dy) = (x1 - x0, y1 - y0);
            let length = (dx * dx + dy * dy).sqrt();
            if length < 0.001 {
                insert_tap_cell(&mut cells, x0, y0, index, grid_step_mm);
            } else {
                let steps = 1.max((length / grid_step_mm) as i64) as usize;
                for i in 0..=steps {
                    let t = i as f64 / steps as f64;
                    insert_tap_cell(&mut cells, x0 + t * dx, y0 + t * dy, index, grid_step_mm);
                }
            }
        }
    }
    for key in board.board.get_vias() {
        let item = board.board.item(key);
        if !item.contains_net(net)
            || baseline_ids.contains(&item.id().0)
            || !source_component_item_ids.contains(&item.id().0)
        {
            continue;
        }
        let center = item.center(&board.board).to_float();
        let (x, y) = (center.x / upm, center.y / upm);
        for index in 0..route_layers.len() {
            insert_tap_cell(&mut cells, x, y, index, grid_step_mm);
        }
    }
    cells
        .into_iter()
        .map(|((_, _, index), point)| RungEndpoint {
            point,
            layer: route_layers[index].1.clone(),
        })
        .collect()
}

fn insert_tap_cell(
    cells: &mut BTreeMap<(i64, i64, usize), [f64; 2]>,
    x: f64,
    y: f64,
    index: usize,
    grid_step_mm: f64,
) {
    let gx = (x / grid_step_mm).round_ties_even() as i64;
    let gy = (y / grid_step_mm).round_ties_even() as i64;
    cells.entry((gx, gy, index)).or_insert([x, y]);
}

/// True when the net has any existing trace or via copper on this board.
fn net_has_existing_copper(board: &BoardAdapter, net: NetNo) -> bool {
    board
        .board
        .get_traces()
        .into_iter()
        .any(|key| board.board.item(key).contains_net(net))
        || board
            .board
            .get_vias()
            .into_iter()
            .any(|key| board.board.item(key).contains_net(net))
}

/// Source `route_multipoint_main` #348 deferral base: the canonical component
/// whose island carries the most existing copper, tie-broken on the source's
/// canonical geometry (the terminal grid cells). The component ids come from the
/// plan (authoritative connectivity), never a fabricated singleton.
///
/// The copper count is the source `get_terminal_component_info.copper_count`:
/// SEGMENTS per component only -- vias are not copper for this count.
fn copper_richest_component(
    board: &BoardAdapter,
    net: NetNo,
    plan: &MultipointPlan,
    grid_step_mm: f64,
) -> Result<Option<BTreeSet<usize>>, String> {
    if plan.terminals.len() < 3 {
        return Ok(None);
    }
    let sets = board.board.get_connected_sets(net);
    let mut set_of_id: BTreeMap<ItemId, usize> = BTreeMap::new();
    let mut copper_per_set: Vec<usize> = Vec::with_capacity(sets.len());
    for (si, set) in sets.iter().enumerate() {
        for id in set.ids() {
            set_of_id.insert(id, si);
        }
        copper_per_set.push(
            set.iter()
                .filter(|&key| board.board.item(key).is_trace())
                .count(),
        );
    }
    let mut best: Option<(usize, i64, i64, usize)> = None;
    for (i, terminal) in plan.terminals.iter().enumerate() {
        let copper = set_of_id
            .get(&board.board.item(terminal.item).id())
            .map(|&si| copper_per_set[si])
            .unwrap_or(0);
        // Source tie-break `(-pad_info[i][0], -pad_info[i][1])` uses the
        // terminal's canonical grid cell (round-ties-even).
        let grid = awx_grid::soft_copper::to_grid(terminal.point_mm, grid_step_mm);
        let key = (copper, -i64::from(grid[0]), -i64::from(grid[1]), i);
        best = Some(match best {
            None => key,
            Some(current) => {
                if key > current {
                    key
                } else {
                    current
                }
            }
        });
    }
    let Some((best_copper, _, _, best_index)) = best else {
        return Ok(None);
    };
    if best_copper == 0 {
        return Ok(None);
    }
    let base_component = plan.terminals[best_index].component;
    let base: BTreeSet<usize> = plan
        .terminals
        .iter()
        .enumerate()
        .filter(|(_, terminal)| terminal.component == base_component)
        .map(|(i, _)| i)
        .collect();
    Ok(Some(base))
}

/// Source `phase3_routing._build_congestion_fn` 1mm bucket radius.
const BOXED_IN_RADIUS_MM: f64 = 1.0;

fn density_push(
    buckets: &mut BTreeMap<(i64, i64), Vec<(f64, f64, String)>>,
    x: f64,
    y: f64,
    net: String,
) {
    buckets
        .entry((
            (x / BOXED_IN_RADIUS_MM).floor() as i64,
            (y / BOXED_IN_RADIUS_MM).floor() as i64,
        ))
        .or_default()
        .push((x, y, net));
}

/// 1mm-bucket spatial hash of every pad and via -> foreign-copper density. NC
/// pads count too -- they physically wall a terminal regardless of net
/// (source `_build_congestion_fn`).
fn build_foreign_density(board: &BoardAdapter) -> BTreeMap<(i64, i64), Vec<(f64, f64, String)>> {
    let mut buckets: BTreeMap<(i64, i64), Vec<(f64, f64, String)>> = BTreeMap::new();
    for pad in board.pads() {
        density_push(&mut buckets, pad.point[0], pad.point[1], pad.net);
    }
    for key in board.board.get_vias() {
        let item = board.board.item(key);
        let center = item.center(&board.board).to_float();
        let net = item
            .net_numbers()
            .first()
            .and_then(|n| board.board.rules.nets.get(*n))
            .map(|n| n.name.clone())
            .unwrap_or_default();
        density_push(
            &mut buckets,
            center.x / board.units_per_mm,
            center.y / board.units_per_mm,
            net,
        );
    }
    buckets
}

/// Count FOREIGN pads/vias within `BOXED_IN_RADIUS_MM` of `(tx, ty)` on the 3x3
/// neighbouring 1mm buckets (source `_congestion`, `nid != net_id`).
fn foreign_density(
    buckets: &BTreeMap<(i64, i64), Vec<(f64, f64, String)>>,
    net: &str,
    tx: f64,
    ty: f64,
) -> i64 {
    let bx = (tx / BOXED_IN_RADIUS_MM).floor() as i64;
    let by = (ty / BOXED_IN_RADIUS_MM).floor() as i64;
    let mut n = 0i64;
    for dx in -1..=1 {
        for dy in -1..=1 {
            let Some(cell) = buckets.get(&(bx + dx, by + dy)) else {
                continue;
            };
            for (x, y, nid) in cell {
                if nid.as_str() != net
                    && (x - tx).abs() <= BOXED_IN_RADIUS_MM
                    && (y - ty).abs() <= BOXED_IN_RADIUS_MM
                {
                    n += 1;
                }
            }
        }
    }
    n
}

/// Source `phase3_routing._order_nets_by_boxed_in_risk` (issue #347): order the
/// pending multipoint nets so the most boxed-in pending terminal routes FIRST.
/// Risk = max over the net's PENDING (still-unrouted) terminals of the local
/// foreign-copper density (pads/vias within 1mm). Deterministic: integer risk,
/// stable sort preserving insertion order on ties.
fn boxed_in_risk_order(board: &BoardAdapter, pending: &[PendingNet]) -> Vec<NetNo> {
    if pending.len() < 2 {
        return pending.iter().map(|entry| entry.net).collect();
    }
    let buckets = build_foreign_density(board);
    let risks: Vec<i64> = pending
        .iter()
        .map(|entry| {
            let mut best = 0i64;
            for (i, terminal) in entry.plan.terminals.iter().enumerate() {
                if entry.routed.contains(&i) {
                    continue;
                }
                let density = foreign_density(
                    &buckets,
                    &entry.name,
                    terminal.point_mm[0],
                    terminal.point_mm[1],
                );
                best = best.max(density);
            }
            best
        })
        .collect();
    let mut order: Vec<usize> = (0..pending.len()).collect();
    order.sort_by(|&a, &b| risks[b].cmp(&risks[a]));
    order.into_iter().map(|i| pending[i].net).collect()
}

pub fn run(path: &Path) -> Result<(), String> {
    crate::awx::native_pad_raster::clear_source_raster_cache();
    crate::board_adapter::clear_validation_baseline_cache();
    let preload_started = Instant::now();
    crate::awx::native_rung::reset_endpoint_runtime_profile();
    crate::board_adapter::reset_validation_metrics();
    crate::awx::native_obstacles::reset_obstacle_detail_profile();
    LOG_WRITE_PROFILE.with(|profile| *profile.borrow_mut() = LogWriteProfile::default());
    let job: EndpointJob = toml::from_str(&fs::read_to_string(path).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())?;
    if job.budget_ms == 0
        || job.max_iterations == 0
        || job.threads == 0
        || !job.min_annular_ring_mm.is_finite()
        || job.min_annular_ring_mm < 0.0
        || !job.hole_to_hole_clearance_mm.is_finite()
        || job.hole_to_hole_clearance_mm < 0.0
        || !job.grid_step_mm.is_finite()
        || job.grid_step_mm <= 0.0
    {
        return Err("invalid AWX endpoint execution parameters".into());
    }
    let input_path = fs::canonicalize(&job.input).map_err(|e| e.to_string())?;
    let output_path = resolved_destination(Path::new(&job.output))?;
    let log_path = resolved_destination(Path::new(&job.log))?;
    if same_file(&input_path, &output_path)
        || same_file(&input_path, &log_path)
        || same_file(&output_path, &log_path)
    {
        return Err("input, output and log must be different files".into());
    }
    let input = fs::read(&input_path).map_err(|e| e.to_string())?;
    let mut board = BoardAdapter::load(&input)?;
    board.configure_awx_hole_rules(job.hole_to_hole_clearance_mm);
    let (free_via_pads, same_net_pads) =
        endpoint_pad_metadata(&board, job.hole_to_hole_clearance_mm > 0.0)?;
    let baseline = board.clone();
    let before = fr_engine::pipeline::stats::incomplete_count(&board.board, None);
    let (incomplete_counts, incomplete_total) = full_incomplete_counts(&board);
    if incomplete_total != before {
        return Err(format!(
            "per-net initial count disagrees with full count: per_net={incomplete_total}, full={before}"
        ));
    }
    let count_witness = IncompleteCountWitness::new(&board);
    let preload_ns = preload_started.elapsed().as_nanos();
    let started = Instant::now();
    let deadline = started + Duration::from_millis(job.budget_ms);
    let log = fs::File::create(&log_path).map_err(|e| e.to_string())?;
    let mut driver = Driver {
        ctx: EndpointContext {
            free_via_pads,
            same_net_pads,
            grid_step_mm: job.grid_step_mm,
            via_drill_mm: job.via_drill_mm,
            min_annular_ring_mm: job.min_annular_ring_mm,
            hole_to_hole_clearance_mm: job.hole_to_hole_clearance_mm,
            max_iterations: job.max_iterations,
            deadline,
        },
        baseline: &baseline,
        protected: &job.protected_nets,
        log,
        attempts: Vec::new(),
        skips: Vec::new(),
        accepted: 0,
        recovery: RecoveryState::default(),
        pending: Vec::new(),
        best_board: board.clone(),
        best_incompletes: before,
        incomplete_counts,
        incomplete_total,
        count_witness,
        record_best_calls: 0,
        profile: DriverProfile::default(),
    };
    let mut phase_ns = BTreeMap::<String, u128>::new();

    // Original native nets in DSN order, dispatched through the default
    // multipoint ordering exactly once before routing. Root source `route.py`
    // (default ordering `mps`) and `routing_defaults.py` select `mps`; the raw
    // DSN net order below is the untouched input to that single call.
    let original_nets: Vec<(NetNo, String)> = board
        .board
        .rules
        .nets
        .iter()
        .map(|n| (n.net_number, n.name.clone()))
        .collect();
    phase_ns.insert(
        "setup_after_budget_start".into(),
        started.elapsed().as_nanos(),
    );
    let phase_started = Instant::now();
    let nets = crate::awx::net_ordering::order_native_nets_mps(&board, &original_nets)?;
    phase_ns.insert("net_ordering".into(), phase_started.elapsed().as_nanos());

    // ---- Phase 1: one main edge per multipoint net; plain nets fully. ----
    // Best-board custody now lives on the driver and is snapshotted after EVERY
    // accepted revision (main, tap, rip, reroute), not only at phase boundaries.
    let phase_started = Instant::now();
    for (net_index, (net, name)) in nets.iter().enumerate() {
        if Instant::now() >= deadline {
            driver.skips.extend(nets[net_index..].iter().map(
                |(_, remaining_name)| serde_json::json!({"net":remaining_name,"reason":"deadline"}),
            ));
            break;
        }
        let initial =
            NetIncompletes::new(&board.board, *net, &board.board.get_connectable_items(*net));
        if initial.count() == 0 {
            continue;
        }
        if job.protected_nets.contains(name) {
            driver
                .skips
                .push(serde_json::json!({"net":name,"reason":"protected"}));
            continue;
        }
        let route_layers = net_route_layers(&board, *net);
        if route_layers.is_empty() {
            driver
                .skips
                .push(serde_json::json!({"net":name,"reason":"no active routing layer"}));
            continue;
        }
        match driver.phase1_route(&mut board, *net, name, &route_layers)? {
            Phase1Outcome::Pending { plan, routed } => driver.pending.push(PendingNet {
                net: *net,
                name: name.clone(),
                plan,
                routed,
                route_layers,
            }),
            Phase1Outcome::Plain | Phase1Outcome::Failed => {}
        }
    }
    phase_ns.insert("phase1_main".into(), phase_started.elapsed().as_nanos());

    // Victims ripped while routing the mains are rerouted before the tap pass;
    // a reroute invalidates the victim's stale pending entry inside the drain.
    let phase_started = Instant::now();
    driver.drain_reroute_queue(&mut board)?;
    phase_ns.insert(
        "reroute_after_main".into(),
        phase_started.elapsed().as_nanos(),
    );

    // ---- Phase 3: remaining MST edges of every pending net, after ALL mains. ----
    // Source `_order_nets_by_boxed_in_risk`: the most boxed-in pending nets route
    // FIRST (measured on the board as it stands after every main), NOT the
    // phase-1 insertion order.
    let phase_started = Instant::now();
    let phase3_order = boxed_in_risk_order(&board, &driver.pending);
    phase_ns.insert(
        "phase3_risk_ordering".into(),
        phase_started.elapsed().as_nanos(),
    );
    let phase_started = Instant::now();
    for net_no in phase3_order {
        if Instant::now() >= deadline {
            break;
        }
        // A net ripped by an earlier tap is retired from `pending` at rip time;
        // one no longer present is skipped exactly like the source
        // `if net_id not in state.pending_multipoint_nets: continue`.
        let Some(position) = driver.pending.iter().position(|entry| entry.net == net_no) else {
            continue;
        };
        // Take only THIS entry out so a rip during its taps can still retire a
        // DIFFERENT pending victim from `driver.pending` at rip time.
        let mut entry = driver.pending.remove(position);
        driver.phase3_taps(
            &mut board,
            entry.net,
            &entry.name,
            &entry.plan,
            &mut entry.routed,
            &entry.route_layers,
        )?;
        driver.pending.push(entry);
    }
    phase_ns.insert("phase3_taps".into(), phase_started.elapsed().as_nanos());

    // Victims ripped by the tap edges are rerouted after the taps.
    let phase_started = Instant::now();
    driver.drain_reroute_queue(&mut board)?;
    phase_ns.insert(
        "reroute_after_taps".into(),
        phase_started.elapsed().as_nanos(),
    );
    let routing_loop_elapsed_ns = started.elapsed().as_nanos();
    // Budget may end with casualties: keep the best consistent board instead of
    // the ripped one, so the run never claims worse connectivity than it made.
    let phase_started = Instant::now();
    let final_live_cached_total = driver.incomplete_total;
    let final_live_incompletes = fr_engine::pipeline::stats::incomplete_count(&board.board, None);
    let final_live_diff = i64::from(final_live_incompletes) - i64::from(final_live_cached_total);
    if final_live_diff != 0 {
        return Err(format!(
            "record_best final count disagrees with full count: cached={}, full={final_live_incompletes}, diff={final_live_diff}",
            final_live_cached_total
        ));
    }
    let best_board_restored = final_live_incompletes > driver.best_incompletes;
    if best_board_restored {
        board = driver.best_board.clone();
        let (counts, total) = full_incomplete_counts(&board);
        if total != driver.best_incompletes {
            return Err(format!(
                "best-board count snapshot disagrees with per-net recount: cached={}, recounted={total}",
                driver.best_incompletes
            ));
        }
        driver.incomplete_counts = counts;
        driver.incomplete_total = total;
        driver.count_witness = IncompleteCountWitness::new(&board);
    }
    let record_best_final_check = serde_json::json!({
        "full_live_count":final_live_incompletes,
        "driver_count_before_restore":final_live_cached_total,
        "final_diff":final_live_diff,
        "best_board_restored":best_board_restored,
        "driver_count_scope":if best_board_restored {"best_board_after_sync"} else {"live_board"},
        "driver_count_after_restore":driver.incomplete_total,
    });
    phase_ns.insert(
        "best_board_restore_check".into(),
        phase_started.elapsed().as_nanos(),
    );
    let routing_budget_elapsed_ns = started.elapsed().as_nanos();

    let phase_started = Instant::now();
    board.assert_invariants(&baseline, &job.protected_nets, true)?;
    phase_ns.insert("final_invariant".into(), phase_started.elapsed().as_nanos());
    let phase_started = Instant::now();
    let session = board.ses();
    phase_ns.insert(
        "ses_serialization".into(),
        phase_started.elapsed().as_nanos(),
    );
    // Restore authorized physical via definitions independently from the source
    // and explicit job rules before materializing the serialized route copper.
    let replay_layer = baseline
        .board
        .layer_structure
        .layers
        .iter()
        .position(|l| l.is_signal)
        .ok_or("no signal stack layer")? as i32;
    let replay_candidates: Vec<_> = baseline
        .board
        .rules
        .nets
        .iter()
        .map(|n| {
            let r = rules_for_layer(
                &baseline,
                n.net_number,
                replay_layer,
                job.grid_step_mm,
                job.via_drill_mm,
                job.min_annular_ring_mm,
                job.hole_to_hole_clearance_mm,
            )?;
            Ok((n.name.clone(), r.via_size, r.via_drill))
        })
        .collect::<Result<_, String>>()?;
    let phase_started = Instant::now();
    let mut replay =
        BoardAdapter::load_session_with_via_candidates(&input, &session, &replay_candidates)?;
    phase_ns.insert("ses_replay_load".into(), phase_started.elapsed().as_nanos());
    replay.configure_awx_hole_rules(job.hole_to_hole_clearance_mm);
    let phase_started = Instant::now();
    replay.assert_session_invariants(&baseline, &job.protected_nets, true)?;
    phase_ns.insert(
        "ses_replay_assert".into(),
        phase_started.elapsed().as_nanos(),
    );
    let phase_started = Instant::now();
    let session_replay_new_violations = replay.new_session_violations_since(&baseline).len();
    let replay_vias =
        crate::awx::board_snapshot::BoardSnapshot::capture_vias(&replay, &BTreeMap::new())?;
    if replay.board.get_vias().len() != board.board.get_vias().len()
        || replay.board.get_traces().len() != board.board.get_traces().len()
        || fr_engine::pipeline::stats::incomplete_count(&replay.board, None)
            != fr_engine::pipeline::stats::incomplete_count(&board.board, None)
        || session_replay_new_violations != 0
    {
        return Err(
            "serialized session replay changed copper counts/connectivity or added violations"
                .into(),
        );
    }
    phase_ns.insert(
        "ses_replay_checks".into(),
        phase_started.elapsed().as_nanos(),
    );
    let attempts = driver.attempts;
    let accepted = driver.accepted;
    let skips = driver.skips;
    let phase_started = Instant::now();
    let new_violations = board
        .violations()
        .difference(&baseline.violations())
        .count();
    phase_ns.insert("final_drc_count".into(), phase_started.elapsed().as_nanos());
    let phase_started = Instant::now();
    let validation_profile = crate::board_adapter::validation_metrics_json();
    let validation_profile_ns = phase_started.elapsed().as_nanos();
    phase_ns.insert("validation_profile_snapshot".into(), validation_profile_ns);
    let obstacle_detail_profile = crate::awx::native_obstacles::obstacle_detail_profile_json();
    let legacy_wall_seconds = started.elapsed().as_secs_f64();
    let final_inventory_started = Instant::now();
    let final_unconnected =
        final_unconnected_inventory(&board, &nets, &attempts, &skips, &driver.profile);
    let final_inventory_ns = final_inventory_started.elapsed().as_nanos();
    phase_ns.insert("final_unconnected_inventory".into(), final_inventory_ns);
    let final_profile_started = Instant::now();
    let backend_runtime_profile = crate::awx::native_rung::endpoint_runtime_profile();
    let backend_snapshot_ns = final_profile_started.elapsed().as_nanos();
    phase_ns.insert("backend_profile_snapshot".into(), backend_snapshot_ns);
    let endpoint_elapsed_ns_before_report = started.elapsed().as_nanos();
    let accounted_ns = phase_ns.values().sum::<u128>();
    let unaccounted_at_report_build_ns =
        endpoint_elapsed_ns_before_report.saturating_sub(accounted_ns);
    let report_build_started = Instant::now();
    let report = serde_json::json!({
        "actual_solver":"awx_grid::GridRouter","scope":"all native incompletes; active signal stack and native via transitions",
        "scheduler":"native NetIncompletes Delaunay/Kruskal airlines; recomputed after each accepted transaction",
        "whole_bga_bus_stages":"not applicable: flattened component identity",
        "budget_ms":job.budget_ms,"requested_threads":job.threads,"effective_threads":1,
        "parallel_policy":"serial board transactions","wall_seconds":legacy_wall_seconds,
        "timing_profile": {
            "coverage":"phase timings are additive top-level intervals; driver function and per-net timings are inclusive and overlap phase totals; backend runtime profile has backend-defined nesting.",
            "budget_wall_seconds_ends_at":"deadline is checked only in routing phases; routing_loop_elapsed_ns marks the second reroute drain completion before best-board restore and final validation.",
            "preload_setup_elapsed_ns":preload_ns,
            "routing_budget_elapsed_ns":routing_budget_elapsed_ns,
            "routing_loop_elapsed_ns":routing_loop_elapsed_ns,
            "endpoint_elapsed_ns_before_report":endpoint_elapsed_ns_before_report,
            "phases_elapsed_ns":phase_ns,
            "validation_profile_snapshot_ns":validation_profile_ns,
            "driver_by_net":driver.profile.json(),
            "record_best_detail_by_net":driver.profile.record_best_json(),
            "record_best_final_check":record_best_final_check,
            "log_write_profile":log_write_profile_json(),
            "unaccounted_ns":unaccounted_at_report_build_ns,
            "unaccounted_scope":"endpoint_elapsed_ns_before_report minus mutually exclusive phases_elapsed_ns; excludes preload_setup_elapsed_ns, report build/serialization, SES output write, and stdout write.",
            "report_build_serialization_and_output":"measured after report creation and recorded in the JSONL runtime_output_profile line; that profile line's own write is excluded.",
        },
        "backend_runtime_profile":backend_runtime_profile,
        "obstacle_detail_profile":obstacle_detail_profile,
        "final_unconnected_nets":final_unconnected,
        "before_incompletes":before,"after_incompletes":fr_engine::pipeline::stats::incomplete_count(&board.board,None),
        "session_replay_incompletes":fr_engine::pipeline::stats::incomplete_count(&replay.board,None),
        "session_replay_vias":replay_vias.len(), "session_replay_via_geometry":replay_vias,
        "session_replay_new_violations":session_replay_new_violations,
        "accepted":accepted,"grid_router_calls":attempts.iter().filter_map(|v|v["result"]["grid_router_calls"].as_u64()).sum::<u64>(),
        "new_vias":board.board.get_vias().len().saturating_sub(baseline.board.get_vias().len()),"new_violations":new_violations,"validation_profile":validation_profile,
        "attempts":attempts,"skipped":skips,"output":job.output
    });
    let report_build_ns = report_build_started.elapsed().as_nanos();
    let serialized_report = serde_json::to_string_pretty(&report).map_err(|e| e.to_string())?;
    let report_serialize_ns = report_build_started
        .elapsed()
        .as_nanos()
        .saturating_sub(report_build_ns);
    let output_started = Instant::now();
    write_atomic(&output_path, &session)?;
    let session_write_ns = output_started.elapsed().as_nanos();
    let stdout_started = Instant::now();
    let mut stdout = std::io::stdout().lock();
    writeln!(stdout, "{}", serialized_report).map_err(|e| e.to_string())?;
    stdout.flush().map_err(|e| e.to_string())?;
    let stdout_write_ns = stdout_started.elapsed().as_nanos();
    drop(stdout);
    log_event(
        &mut driver.log,
        &serde_json::json!({
            "event":"runtime_output_profile",
            "report_serialization_ns":report_serialize_ns,
            "report_build_ns":report_build_ns,
            "endpoint_elapsed_ns_before_report":endpoint_elapsed_ns_before_report,
            "final_inventory_ns":final_inventory_ns,
            "ses_output_write_ns":session_write_ns,
            "stdout_write_ns":stdout_write_ns,
            "profile_line_write_included":false,
        }),
    )?;
    Ok(())
}

/// Route only the airlines present on entry. Endpoints are frozen from the SES
/// snapshot, so copper added during this run cannot create new targets.
pub fn run_residual(path: &Path) -> Result<(), String> {
    crate::awx::native_pad_raster::clear_source_raster_cache();
    crate::board_adapter::clear_validation_baseline_cache();
    crate::board_adapter::reset_validation_metrics();
    let job: ResidualJob = toml::from_str(&fs::read_to_string(path).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())?;
    if job.budget_ms == 0
        || job.max_iterations == 0
        || job.threads == 0
        || !job.grid_step_mm.is_finite()
        || job.grid_step_mm <= 0.0
        || !job.min_annular_ring_mm.is_finite()
        || job.min_annular_ring_mm < 0.0
        || !job.hole_to_hole_clearance_mm.is_finite()
        || job.hole_to_hole_clearance_mm < 0.0
    {
        return Err("invalid residual execution parameters".into());
    }
    let job_dir = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    let resolve_from_job = |value: &str| {
        let p = Path::new(value);
        if p.is_absolute() {
            p.to_path_buf()
        } else {
            job_dir.join(p)
        }
    };
    let input_path = fs::canonicalize(resolve_from_job(&job.input)).map_err(|e| e.to_string())?;
    let session_path =
        fs::canonicalize(resolve_from_job(&job.session)).map_err(|e| e.to_string())?;
    let output_path = resolved_destination(&resolve_from_job(&job.output))?;
    let log_path = resolved_destination(&resolve_from_job(&job.log))?;
    if same_file(&input_path, &output_path)
        || same_file(&input_path, &log_path)
        || same_file(&output_path, &log_path)
    {
        return Err("input, output and log must be different files".into());
    }
    if same_file(&session_path, &output_path) || same_file(&session_path, &log_path) {
        return Err("session input, output and log must be different files".into());
    }
    let input = fs::read(&input_path).map_err(|e| e.to_string())?;
    let session_input = fs::read(&session_path).map_err(|e| e.to_string())?;
    let mut dsn_board = BoardAdapter::load(&input)?;
    dsn_board.configure_awx_hole_rules(job.hole_to_hole_clearance_mm);
    let replay_layer = dsn_board
        .board
        .layer_structure
        .layers
        .iter()
        .position(|l| l.is_signal)
        .ok_or("no signal stack layer")? as i32;
    let via_candidates = dsn_board
        .board
        .rules
        .nets
        .iter()
        .map(|n| {
            let rules = rules_for_layer(
                &dsn_board,
                n.net_number,
                replay_layer,
                job.grid_step_mm,
                job.via_drill_mm,
                job.min_annular_ring_mm,
                job.hole_to_hole_clearance_mm,
            )?;
            Ok((n.name.clone(), rules.via_size, rules.via_drill))
        })
        .collect::<Result<Vec<_>, String>>()?;
    let mut board =
        BoardAdapter::load_session_with_via_candidates(&input, &session_input, &via_candidates)?;
    board.configure_awx_hole_rules(job.hole_to_hole_clearance_mm);
    let (free_via_pads, same_net_pads) =
        endpoint_pad_metadata(&board, job.hole_to_hole_clearance_mm > 0.0)?;
    let baseline = board.clone();
    let selected: BTreeSet<&str> = job.nets.iter().map(String::as_str).collect();
    let unknown: Vec<_> = selected
        .iter()
        .filter(|name| !board.board.rules.nets.iter().any(|n| n.name == **name))
        .copied()
        .collect();
    if !unknown.is_empty() {
        return Err(format!("unknown residual nets: {}", unknown.join(", ")));
    }
    let mut initial_gaps = Vec::new();
    let mut initial_by_net = BTreeMap::<String, i32>::new();
    let mut net_skips = Vec::new();
    for n in board.board.rules.nets.iter() {
        if !selected.is_empty() && !selected.contains(n.name.as_str()) {
            continue;
        }
        let gaps = NetIncompletes::new(
            &board.board,
            n.net_number,
            &board.board.get_connectable_items(n.net_number),
        );
        initial_by_net.insert(n.name.clone(), gaps.count());
        if gaps.count() == 0 {
            continue;
        }
        if job.protected_nets.contains(&n.name) {
            net_skips.push(serde_json::json!({"net":n.name,"reason":"protected","initial_incompletes":gaps.count()}));
            continue;
        }
        let layers = net_route_layers(&board, n.net_number);
        if layers.is_empty() {
            net_skips.push(serde_json::json!({"net":n.name,"reason":"no_active_routing_layer","initial_incompletes":gaps.count()}));
            continue;
        }
        for edge in gaps.incompletes {
            let groups = board.board.get_connected_sets(n.net_number);
            let from_id = board.board.item(edge.from_item).id().0;
            let to_id = board.board.item(edge.to_item).id().0;
            let from_group = groups
                .iter()
                .find(|group| group.contains(board.board.item(edge.from_item).id()));
            let to_group = groups
                .iter()
                .find(|group| group.contains(board.board.item(edge.to_item).id()));
            let group_ids = |group: Option<&ItemSet>| {
                let mut ids: Vec<i32> = group
                    .into_iter()
                    .flat_map(|g| g.iter())
                    .map(|member| board.board.item(member).id().0)
                    .collect();
                ids.sort_unstable();
                ids
            };
            let from_group_ids = group_ids(from_group);
            let to_group_ids = group_ids(to_group);
            initial_gaps.push((
                n.net_number,
                n.name.clone(),
                edge.from_item,
                edge.to_item,
                from_id,
                to_id,
                from_group_ids,
                from_group
                    .map(|group| connected_group_candidates(&board, group, &layers))
                    .unwrap_or_default(),
                to_group_ids,
                to_group
                    .map(|group| connected_group_candidates(&board, group, &layers))
                    .unwrap_or_default(),
                layers.clone(),
            ));
        }
    }
    let started = Instant::now();
    let deadline = started + Duration::from_millis(job.budget_ms);
    let (incomplete_counts, incomplete_total) = full_incomplete_counts(&board);
    let best_incompletes = fr_engine::pipeline::stats::incomplete_count(&board.board, None);
    if incomplete_total != best_incompletes {
        return Err(format!(
            "per-net initial count disagrees with full count: per_net={incomplete_total}, full={best_incompletes}"
        ));
    }
    let count_witness = IncompleteCountWitness::new(&board);
    let mut driver = Driver {
        ctx: EndpointContext {
            free_via_pads,
            same_net_pads,
            grid_step_mm: job.grid_step_mm,
            via_drill_mm: job.via_drill_mm,
            min_annular_ring_mm: job.min_annular_ring_mm,
            hole_to_hole_clearance_mm: job.hole_to_hole_clearance_mm,
            max_iterations: job.max_iterations,
            deadline,
        },
        baseline: &baseline,
        protected: &job.protected_nets,
        log: fs::File::create(&log_path).map_err(|e| e.to_string())?,
        attempts: Vec::new(),
        skips: Vec::new(),
        accepted: 0,
        recovery: RecoveryState::default(),
        pending: Vec::new(),
        best_board: board.clone(),
        best_incompletes,
        incomplete_counts,
        incomplete_total,
        count_witness,
        record_best_calls: 0,
        profile: DriverProfile::default(),
    };
    let mut outcomes = Vec::new();
    for (
        index,
        (
            net,
            name,
            from_item,
            to_item,
            from_id,
            to_id,
            initial_from_group,
            starts,
            initial_to_group,
            targets,
            layers,
        ),
    ) in initial_gaps.iter().enumerate()
    {
        let before_net =
            NetIncompletes::new(&board.board, *net, &board.board.get_connectable_items(*net));
        let groups_before = stable_connected_group_ids(&board, *net);
        let current_from_group = groups_before.iter().find(|ids| ids.contains(from_id));
        let current_to_group = groups_before.iter().find(|ids| ids.contains(to_id));
        let before_groups = groups_before.len();
        if current_from_group.is_some() && current_from_group == current_to_group {
            outcomes.push(serde_json::json!({"initial_gap":index,"net":name,"status":"already_connected","source_initial_group_native_item_ids":initial_from_group,"target_initial_group_native_item_ids":initial_to_group,"source_group_before_native_item_ids":current_from_group,"target_group_before_native_item_ids":current_to_group,"net_groups_before":before_groups,"net_groups_after":before_groups}));
            continue;
        }
        if Instant::now() >= deadline {
            outcomes.push(serde_json::json!({"initial_gap":index,"net":name,"status":"budget_exhausted","net_groups_before":before_groups}));
            continue;
        }
        if starts.is_empty() || targets.is_empty() {
            outcomes.push(serde_json::json!({"initial_gap":index,"net":name,"status":"no_native_endpoint","net_groups_before":before_groups}));
            continue;
        }
        let (candidate, _) = driver.route_endpoint_set(
            &board,
            *net,
            name,
            "residual_only",
            *from_item,
            *to_item,
            starts,
            targets,
            layers,
        )?;
        let native_report = driver.attempts.last().map(|a| a["result"].clone());
        let mut status = "native_declined";
        if let Some(trial) = candidate {
            let after =
                NetIncompletes::new(&trial.board, *net, &trial.board.get_connectable_items(*net));
            if after.count() < before_net.count() {
                trial.assert_invariants(&baseline, &job.protected_nets, true)?;
                board = trial;
                driver.accepted += 1;
                driver.record_best(&board, name, &[*net])?;
                status = "accepted";
            } else {
                status = "rejected_no_net_progress";
            }
        }
        let after =
            NetIncompletes::new(&board.board, *net, &board.board.get_connectable_items(*net));
        let groups_after = stable_connected_group_ids(&board, *net);
        let start_candidates_json: Vec<serde_json::Value> = starts
            .iter()
            .map(|endpoint| serde_json::json!({"point_mm":endpoint.point,"layer":endpoint.layer}))
            .collect();
        let target_candidates_json: Vec<serde_json::Value> = targets
            .iter()
            .map(|endpoint| serde_json::json!({"point_mm":endpoint.point,"layer":endpoint.layer}))
            .collect();
        outcomes.push(serde_json::json!({"initial_gap":index,"net":name,"status":status,
            "source_initial_group_native_item_ids":initial_from_group,"target_initial_group_native_item_ids":initial_to_group,
            "source_group_before_native_item_ids":current_from_group,"target_group_before_native_item_ids":current_to_group,
            "source_group_after_native_item_ids":groups_after.iter().find(|ids| ids.contains(from_id)),
            "target_group_after_native_item_ids":groups_after.iter().find(|ids| ids.contains(to_id)),
            "net_groups_before":before_groups,"net_groups_after":after.connected_group_count(),
            "net_incompletes_before":before_net.count(),"net_incompletes_after":after.count(),
            "start_candidates":start_candidates_json,"target_candidates":target_candidates_json,"native_result":native_report}));
    }
    board.assert_invariants(&baseline, &job.protected_nets, true)?;
    let session = board.ses();
    let mut replay =
        BoardAdapter::load_session_with_via_candidates(&input, &session, &via_candidates)?;
    replay.configure_awx_hole_rules(job.hole_to_hole_clearance_mm);
    replay.assert_session_invariants(&baseline, &job.protected_nets, true)?;
    replay.assert_session_invariants(&board, &job.protected_nets, true)?;
    let replay_new_violations = replay.new_session_violations_since(&baseline).len();
    let final_board_incompletes =
        fr_engine::pipeline::stats::incomplete_count(&board.board, None);
    let final_replay_incompletes =
        fr_engine::pipeline::stats::incomplete_count(&replay.board, None);
    let final_diff = i64::from(final_board_incompletes) - i64::from(driver.incomplete_total);
    if final_diff != 0 {
        return Err(format!(
            "record_best residual count disagrees with full count: cached={}, full={final_board_incompletes}, diff={final_diff}",
            driver.incomplete_total
        ));
    }
    if replay_new_violations != 0
        || replay.board.get_traces().len() != board.board.get_traces().len()
        || replay.board.get_vias().len() != board.board.get_vias().len()
        || final_replay_incompletes != final_board_incompletes
    {
        return Err(
            "serialized SES replay changed copper/connectivity or added DRC violations".into(),
        );
    }
    let after_by_net: BTreeMap<String, i32> = board
        .board
        .rules
        .nets
        .iter()
        .filter(|n| initial_by_net.contains_key(&n.name))
        .map(|n| {
            (
                n.name.clone(),
                NetIncompletes::new(
                    &board.board,
                    n.net_number,
                    &board.board.get_connectable_items(n.net_number),
                )
                .count(),
            )
        })
        .collect();
    let replay_by_net: BTreeMap<String, i32> = replay
        .board
        .rules
        .nets
        .iter()
        .filter(|n| initial_by_net.contains_key(&n.name))
        .map(|n| {
            (
                n.name.clone(),
                NetIncompletes::new(
                    &replay.board,
                    n.net_number,
                    &replay.board.get_connectable_items(n.net_number),
                )
                .count(),
            )
        })
        .collect();
    let complete = after_by_net.values().all(|count| *count == 0);
    let validation_profile = crate::board_adapter::validation_metrics_json();
    let output = serde_json::json!({"scope":"frozen initial residual airlines only","nets":if selected.is_empty(){"all".to_string()}else{job.nets.join(",")},
        "initial_net_incompletes":initial_by_net,"after_net_incompletes":after_by_net,
        "routing_complete":complete,"success":complete,
        "record_best_final_check":{"full_count":final_board_incompletes,"driver_count":driver.incomplete_total,"final_diff":final_diff},
        "record_best_detail_by_net":driver.profile.record_best_json(),
        "ses_replay_net_incompletes":replay_by_net,
        "ses_replay_new_violations":replay_new_violations,
        "accepted":driver.accepted,"budget_ms":job.budget_ms,"requested_threads":job.threads,"effective_threads":1,
        "elapsed_seconds":started.elapsed().as_secs_f64(),"gap_outcomes":outcomes,
        "attempts":driver.attempts,"frozen_initial_airline_count":initial_gaps.len(),"validation_profile":validation_profile,
        "net_skips":net_skips,
        "endpoint_policy":"frozen native terminal endpoints in entry-time connected groups; no baseline trace split/interior endpoints; no targets from newly committed Cu",
        "output":output_path.display().to_string()});
    write_atomic(&output_path, &session)?;
    println!(
        "{}",
        serde_json::to_string_pretty(&output).map_err(|e| e.to_string())?
    );
    Ok(())
}
