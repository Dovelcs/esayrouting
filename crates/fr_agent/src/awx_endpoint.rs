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
        native_fanout_stage::{
            self, NativeFanoutConfig, NativeRescueConfig, NativeRescueStage, NativeRescueSummary,
        },
        native_rung::route_multilayer_endpoint_sets_in_run,
        net_rescue::{plan_rescue, RescuePlan},
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
    io::Write,
    path::Path,
    sync::Arc,
    time::{Duration, Instant},
};

/// Source `route_multipoint_main` main-edge attempt bound.
const MAX_MAIN_ATTEMPTS: usize = 8;
const RECORD_BEST_FULLCHECK_EVERY: u64 = 64;

#[derive(Clone)]
struct RescueComponent {
    group: ItemSet,
    pin_count: usize,
    tie_id: i32,
    points: Vec<[f64; 2]>,
    candidates: Vec<RungEndpoint>,
}

fn rescue_components(
    board: &BoardAdapter,
    net: NetNo,
    route_layers: &[(i32, String)],
) -> Vec<RescueComponent> {
    let mut components = board
        .board
        .get_connected_sets(net)
        .into_iter()
        .map(|group| {
            let mut ids: Vec<i32> = group
                .iter()
                .map(|key| board.board.item(key).id().0)
                .collect();
            ids.sort_unstable();
            let pin_count = group
                .iter()
                .filter(|key| board.board.item(*key).is_pin())
                .count();
            RescueComponent {
                group: group.clone(),
                tie_id: ids.first().copied().unwrap_or(i32::MAX),
                pin_count,
                points: rescue_component_points(board, &group),
                candidates: connected_group_candidates(board, &group, route_layers),
            }
        })
        .collect::<Vec<_>>();
    components.retain(|component| {
        component.pin_count > 0 && !component.points.is_empty() && !component.candidates.is_empty()
    });
    components.sort_by_key(|component| component.tie_id);
    components
}

fn rescue_component_points(board: &BoardAdapter, group: &ItemSet) -> Vec<[f64; 2]> {
    let mm = |point: Point| {
        let exact = point.to_float();
        [exact.x / board.units_per_mm, exact.y / board.units_per_mm]
    };
    let mut points = Vec::new();
    // Source ordering: pad centers, member segment endpoints, then via centers.
    for member in group.iter().filter(|key| board.board.item(*key).is_pin()) {
        points.push(mm(board.board.item(member).center(&board.board)));
    }
    for member in group.iter().filter(|key| board.board.item(*key).is_trace()) {
        let item = board.board.item(member);
        points.push(mm(item.first_corner()));
        points.push(mm(item.last_corner()));
    }
    for member in group.iter().filter(|key| board.board.item(*key).is_via()) {
        points.push(mm(board.board.item(member).center(&board.board)));
    }
    points
}

fn rescue_pair(a: &[[f64; 2]], b: &[[f64; 2]]) -> Option<(f64, [f64; 2], [f64; 2])> {
    let sample = |points: &[[f64; 2]]| {
        if points.len() > 300 {
            let step = points.len() / 300 + 1;
            points.iter().step_by(step).copied().collect()
        } else {
            points.to_vec()
        }
    };
    let a = sample(a);
    let b = sample(b);
    let mut best: Option<(f64, [f64; 2], [f64; 2])> = None;
    for from in &a {
        for to in &b {
            let distance = (to[0] - from[0]).hypot(to[1] - from[1]);
            // Preserve the first equal-distance pair, matching the source's
            // strict-minimum traversal over its ordered point lists.
            let replace = best
                .as_ref()
                .map_or(true, |(best_distance, _, _)| distance < *best_distance);
            if replace {
                best = Some((distance, *from, *to));
            }
        }
    }
    best
}

fn rescue_midpoint_key(a: [f64; 2], b: [f64; 2]) -> (i64, i64) {
    (
        (((a[0] + b[0]) * 50.0).round_ties_even()) as i64,
        (((a[1] + b[1]) * 50.0).round_ties_even()) as i64,
    )
}

fn point_in_rescue_window(point: [f64; 2], window: [f64; 4]) -> bool {
    point[0] >= window[0]
        && point[0] <= window[2]
        && point[1] >= window[1]
        && point[1] <= window[3]
}

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

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PadAccess {
    Legacy,
    CopperArea,
}

impl Default for PadAccess {
    fn default() -> Self {
        Self::CopperArea
    }
}

impl PadAccess {
    pub fn uses_copper_area(self) -> bool {
        matches!(self, Self::CopperArea)
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Legacy => "legacy",
            Self::CopperArea => "copper_area",
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct EndpointJob {
    input: String,
    output: String,
    log: String,
    budget_ms: u64,
    #[serde(default = "default_grid_step_mm")]
    grid_step_mm: f64,
    max_iterations: usize,
    threads: usize,
    via_drill_mm: f64,
    min_annular_ring_mm: f64,
    hole_to_hole_clearance_mm: f64,
    #[serde(default)]
    pad_access: PadAccess,
    #[serde(default)]
    pre_fanout: NativeFanoutConfig,
    #[serde(default)]
    native_rescue: NativeRescueConfig,
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
    #[serde(default = "default_grid_step_mm")]
    grid_step_mm: f64,
    max_iterations: usize,
    threads: usize,
    via_drill_mm: f64,
    min_annular_ring_mm: f64,
    hole_to_hole_clearance_mm: f64,
    #[serde(default)]
    pad_access: PadAccess,
    #[serde(default)]
    protected_nets: BTreeSet<String>,
}

fn default_grid_step_mm() -> f64 {
    crate::awx::routing_defaults::CONNECTION_GRID_STEP_MM
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
    let legal_through = crate::awx::native_rung::allowed_through_via_sizes(
        board,
        net,
        min_annular,
    )?;
    let class_via_size = class.get_via_rule().and_then(|rule_id| {
        board.board.rules.via_rules[rule_id]
            .vias()
            .iter()
            .find_map(|via_id| {
                let info = &board.board.rules.via_infos[*via_id];
                board
                    .board
                    .library
                    .padstacks
                    .get(info.get_padstack())?
                    .get_shape(layer)
                    .map(|shape| shape.max_width() / board.units_per_mm)
            })
    });
    let via_size = legal_through
        .first()
        .map(|sizes| sizes[0])
        .or(class_via_size)
        .unwrap_or(drill + 2.0 * min_annular);
    let via_drill = legal_through
        .first()
        .map(|sizes| sizes[1])
        .unwrap_or(drill);
    if via_size <= 0.0 {
        return Err("source net class has no via copper envelope".into());
    }
    if !drill.is_finite()
        || drill <= 0.0
        || !min_annular.is_finite()
        || min_annular < 0.0
        || !hole_clearance.is_finite()
        || hole_clearance < 0.0
    {
        return Err("invalid native via drill or clearance input".into());
    }
    Ok(RoutingRules {
        grid_step: grid,
        track_width: width,
        clearance,
        via_size,
        via_drill,
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
        net_via_sizes: legal_through
            .first()
            .map(|sizes| BTreeMap::from([(net as i32, *sizes)]))
            .unwrap_or_default(),
    })
}

thread_local! {
    static LOG_EVENT_BUFFER: std::cell::RefCell<Vec<u8>> = const { std::cell::RefCell::new(Vec::new()) };
}

fn log_event(log: &mut fs::File, event: &serde_json::Value) -> Result<(), String> {
    LOG_EVENT_BUFFER.with(|buffer| {
        let mut buffer = buffer.borrow_mut();
        buffer.clear();
        serde_json::to_writer(&mut *buffer, event)
            .map_err(|e| e.to_string())
            .map(|()| buffer.push(b'\n'))?;
        log.write_all(&buffer).map_err(|e| e.to_string())?;
        log.flush().map_err(|e| e.to_string())
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
    retry_counts: &BTreeMap<String, u64>,
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
        let retry_count = retry_counts.get(name).copied().unwrap_or(0);
        let last_failure = net_attempts.iter().rev().find_map(|event| {
            let result = &event["result"];
            if result["accepted"].as_bool().unwrap_or(false) {
                return None;
            }
            let diagnostic = &result["_endpoint_diagnostic"];
            let search = &diagnostic["search_evidence"];
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
            let search_summary = search.as_object().map(|_| {
                serde_json::json!({
                    "search_graph_scope": search["search_graph_scope"],
                    "search_termination": search["search_termination"],
                    "forward_probe_termination": search["forward_probe_termination"],
                    "backward_probe_termination": search["backward_probe_termination"],
                    "forward_full_termination": search["forward_full_termination"],
                    "backward_full_termination": search["backward_full_termination"],
                    "probe_policy_early_stop": search["probe_policy_early_stop"],
                    "small_via_rung_retry_ran": search["small_via_rung_retry_ran"],
                    "small_via_rung_forward_termination": search["small_via_rung_forward_termination"],
                    "small_via_rung_backward_termination": search["small_via_rung_backward_termination"],
                    "small_via_rung_forward_full_termination": search["small_via_rung_forward_full_termination"],
                    "small_via_rung_backward_full_termination": search["small_via_rung_backward_full_termination"],
                    "small_via_rung_probe_policy_early_stop": search["small_via_rung_probe_policy_early_stop"],
                    "via_pad_unblock": search["via_pad_unblock"],
                })
            });
            Some(serde_json::json!({
                "route_stage": event["stage"], "failure_stage": failure_stage,
                "reason": raw, "category": category,
                "expanded": result.get("expanded").or_else(|| result.get("expanded_nodes")),
                "evidence_scope": "net_history",
                "search_graph_scope": search["search_graph_scope"],
                "search_termination": search["search_termination"],
                "search_evidence": search_summary,
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

/// Distinguishes a complete reroute from an accepted main route whose taps
/// remain open, and from a reroute that landed no main copper.
enum RecoveryRouteOutcome {
    Complete,
    PartialMain,
    NoMain,
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
    retry_counts: BTreeMap<String, u64>,
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
        changed_nets: &[NetNo],
    ) -> Result<(), String> {
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
        self.record_best_calls += 1;
        if self.record_best_calls % RECORD_BEST_FULLCHECK_EVERY == 0 {
            let full_count =
                fr_engine::pipeline::stats::incomplete_count(&board.board, None);
            if full_count != count {
                return Err(format!(
                    "record_best net-count cache disagrees with full count: incremental={count}, full={full_count}"
                ));
            }
        }
        if count < self.best_incompletes {
            self.best_board = board.clone();
            self.best_incompletes = count;
        }
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
        self.route_endpoint_set_with_plan(
            board, net, name, stage, from_item, to_item, starts, targets, route_layers, None,
        )
    }

    /// Same native endpoint route with an optional exact scoped rescue plan.
    #[allow(clippy::too_many_arguments)]
    fn route_endpoint_set_with_plan(
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
        rescue: Option<&RescuePlan>,
    ) -> Result<(Option<BoardAdapter>, Option<RungReport>), String> {
        let route_started = Instant::now();
        if starts.is_empty() || targets.is_empty() {
            return Ok((None, None));
        }
        let mut request = build_endpoint_request(
            board,
            &self.ctx,
            net,
            name,
            stage,
            starts[0].clone(),
            targets[0].clone(),
            route_layers,
        )?;
        if let Some(plan) = rescue {
            request.rules.grid_step = plan.grid_step_mm;
            request.max_iterations = plan.max_iterations;
            request.spec.margin = 0.0;
            request.window_points = Some(vec![
                [plan.window_mm[0], plan.window_mm[1]],
                [plan.window_mm[2], plan.window_mm[1]],
                [plan.window_mm[2], plan.window_mm[3]],
                [plan.window_mm[0], plan.window_mm[3]],
            ]);
        }
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
        log_event(&mut self.log, &event)?;
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
        log_event(&mut self.log, &event)?;
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
        let ctx = self.ctx.clone();
        let baseline = self.baseline;
        let protected = self.protected;
        let net_name = name.to_string();
        let mut retry_count = 0u64;
        let starts0 = starts.to_vec();
        let targets0 = targets.to_vec();
        let route_layers0 = route_layers.to_vec();
        let mut retry = |trial: &BoardAdapter| {
            retry_count += 1;
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
            &mut retry,
        )?;
        *self.retry_counts.entry(name.to_string()).or_default() += retry_count;
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
        let outcome = self.phase1_route_inner(board, net, name, route_layers)?;
        if matches!(&outcome, Phase1Outcome::Plain | Phase1Outcome::Pending { .. }) {
            self.recovery.note_routed_result(net);
        }
        Ok(outcome)
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
                self.record_best(board, &[net])?;
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
                self.record_best(board, &changed_nets)?;
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
                    self.record_best(board, &[net])?;
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
                        self.record_best(board, &changed_nets)?;
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

    /// Last-chance exact-nominal-geometry rescue. This pass has no rip-up
    /// authority and only commits a route that joins its selected native
    /// components while reducing the authoritative incomplete count.
    fn fine_grid_rescue(
        &mut self,
        board: &mut BoardAdapter,
        nets: &[(NetNo, String)],
        pin_scope: Option<&BTreeSet<i32>>,
    ) -> Result<serde_json::Value, String> {
        let mut ordered = nets.to_vec();
        ordered.sort_by(|a, b| a.1.cmp(&b.1).then_with(|| a.0.cmp(&b.0)));
        let mut summaries = serde_json::Map::new();
        for (net, name) in ordered {
            if self.protected.contains(&name) {
                continue;
            }
            let route_layers = net_route_layers(board, net);
            if route_layers.is_empty() {
                let remaining = NetIncompletes::new(
                    &board.board,
                    net,
                    &board.board.get_connectable_items(net),
                )
                .count();
                if remaining > 0 {
                    summaries.insert(
                        name,
                        serde_json::json!({"attempted":0,"accepted":0,"remaining":remaining,"grids_mm":[]}),
                    );
                }
                continue;
            }
            let mut attempts = 0usize;
            let mut accepted = 0usize;
            let mut failed_gaps = BTreeSet::new();
            let mut grids = Vec::new();
            while attempts < crate::awx::routing_defaults::RESCUE_MAX_EDGES_PER_NET
                && Instant::now() < self.ctx.deadline
            {
                let live = NetIncompletes::new(
                    &board.board,
                    net,
                    &board.board.get_connectable_items(net),
                );
                if live.count() == 0 {
                    break;
                }
                let components = rescue_components(board, net, &route_layers);
                if components.len() < 2 {
                    break;
                }
                let main_index = components
                    .iter()
                    .enumerate()
                    .min_by(|(_, a), (_, b)| {
                        b.pin_count
                            .cmp(&a.pin_count)
                            .then_with(|| a.tie_id.cmp(&b.tie_id))
                    })
                    .map(|(index, _)| index)
                    .ok_or("rescue has no eligible component")?;
                let main = &components[main_index];
                let main_has_scope_pin = pin_scope.is_some_and(|scope| {
                    main.group.iter().any(|key| {
                        let item = board.board.item(key);
                        item.is_pin() && scope.contains(&item.id().0)
                    })
                });
                let mut gaps = Vec::new();
                for (island_index, island) in components.iter().enumerate() {
                    if island_index == main_index {
                        continue;
                    }
                    if let Some(scope) = pin_scope {
                        let island_has_scope_pin = island.group.iter().any(|key| {
                            let item = board.board.item(key);
                            item.is_pin() && scope.contains(&item.id().0)
                        });
                        if !main_has_scope_pin && !island_has_scope_pin {
                            continue;
                        }
                    }
                    let Some((distance, from, to)) = rescue_pair(&main.points, &island.points)
                    else {
                        continue;
                    };
                    let key = rescue_midpoint_key(from, to);
                    if !failed_gaps.contains(&key) {
                        gaps.push((distance, island.tie_id, key, from, to, island.clone()));
                    }
                }
                // Python's stable distance-only sort preserves component order
                // for equal gaps; our component order is deterministic by id.
                gaps.sort_by(|a, b| a.0.total_cmp(&b.0));
                let Some((_distance, _tie_id, key, from, to, island)) = gaps.into_iter().next()
                else {
                    break;
                };
                let plan = plan_rescue(
                    self.ctx.grid_step_mm,
                    self.ctx.max_iterations,
                    from,
                    to,
                )?;
                attempts += 1;
                if !grids.contains(&plan.grid_step_mm) {
                    grids.push(plan.grid_step_mm);
                }
                let starts: Vec<_> = main
                    .candidates
                    .iter()
                    .filter(|endpoint| point_in_rescue_window(endpoint.point, plan.window_mm))
                    .cloned()
                    .collect();
                let targets: Vec<_> = island
                    .candidates
                    .iter()
                    .filter(|endpoint| point_in_rescue_window(endpoint.point, plan.window_mm))
                    .cloned()
                    .collect();
                let intended_endpoints_in_window = point_in_rescue_window(from, plan.window_mm)
                    && point_in_rescue_window(to, plan.window_mm)
                    && starts.iter().any(|point| point.point == from)
                    && targets.iter().any(|point| point.point == to);
                if !intended_endpoints_in_window || starts.is_empty() || targets.is_empty() {
                    failed_gaps.insert(key);
                    continue;
                }
                let from_item = main
                    .group
                    .iter()
                    .find(|key| board.board.item(*key).is_pin())
                    .ok_or("rescue source group has no pin")?;
                let to_item = island
                    .group
                    .iter()
                    .find(|key| board.board.item(*key).is_pin())
                    .ok_or("rescue target group has no pin")?;
                let before_count = live.count();
                let (candidate, _frontier) = self.route_endpoint_set_with_plan(
                    board,
                    net,
                    &name,
                    "fine_grid_rescue",
                    from_item,
                    to_item,
                    &starts,
                    &targets,
                    &route_layers,
                    Some(&plan),
                )?;
                let Some(candidate) = candidate else {
                    failed_gaps.insert(key);
                    continue;
                };
                let intended_groups_merged = candidate
                    .board
                    .connected_set(from_item, net, false)
                    .contains(candidate.board.item(to_item).id());
                let after_count = NetIncompletes::new(
                    &candidate.board,
                    net,
                    &candidate.board.get_connectable_items(net),
                )
                .count();
                if !intended_groups_merged || after_count >= before_count {
                    failed_gaps.insert(key);
                    continue;
                }
                if candidate
                    .assert_invariants(self.baseline, self.protected, true)
                    .is_err()
                {
                    failed_gaps.insert(key);
                    continue;
                }
                *board = candidate;
                self.accepted += 1;
                accepted += 1;
                self.record_best(board, &[net])?;
                // Accepted merges invalidate the prior failed-pair set only
                // through new component geometry; historical failed keys stay
                // fenced exactly as in the source loop.
            }
            let remaining = NetIncompletes::new(
                &board.board,
                net,
                &board.board.get_connectable_items(net),
            )
            .count();
            if attempts > 0 || remaining > 0 {
                grids.sort_by(f64::total_cmp);
                summaries.insert(
                    name,
                    serde_json::json!({
                        "attempted":attempts,
                        "accepted":accepted,
                        "remaining":remaining,
                        "grids_mm":grids,
                    }),
                );
            }
        }
        Ok(serde_json::Value::Object(summaries))
    }

    /// Final single pass using the current native incomplete edges and both
    /// complete connected groups, matching the residual-only endpoint policy.
    /// The initial edge list is frozen so each airline is attempted at most once;
    /// group membership is refreshed after every accepted connection.
    fn connected_group_residual_rescue(
        &mut self,
        board: &mut BoardAdapter,
        nets: &[(NetNo, String)],
    ) -> Result<serde_json::Value, String> {
        let mut edges = Vec::new();
        let mut seen = BTreeSet::new();
        for (net, name) in nets {
            if self.protected.contains(name) {
                continue;
            }
            let layers = net_route_layers(board, *net);
            if layers.is_empty() {
                continue;
            }
            let incomplete = NetIncompletes::new(
                &board.board,
                *net,
                &board.board.get_connectable_items(*net),
            );
            for edge in incomplete.incompletes {
                let from_id = board.board.item(edge.from_item).id().0;
                let to_id = board.board.item(edge.to_item).id().0;
                if seen.insert((*net, from_id.min(to_id), from_id.max(to_id))) {
                    let from_group = board.board.connected_set(edge.from_item, *net, false);
                    let to_group = board.board.connected_set(edge.to_item, *net, false);
                    let witness_id = |anchor: ItemKey, group: &ItemSet| {
                        let item = board.board.item(anchor);
                        if item.is_pin() || item.is_via() {
                            item.id().0
                        } else {
                            group
                                .iter()
                                .filter_map(|key| {
                                    let member = board.board.item(key);
                                    (member.is_pin() || member.is_via())
                                        .then_some(member.id().0)
                                })
                                .min()
                                .unwrap_or(item.id().0)
                        }
                    };
                    edges.push((
                        *net,
                        name.clone(),
                        from_id,
                        to_id,
                        witness_id(edge.from_item, &from_group),
                        witness_id(edge.to_item, &to_group),
                        layers.clone(),
                    ));
                }
            }
        }

        let mut attempted = 0usize;
        let mut accepted = 0usize;
        let mut outcomes = Vec::new();
        for (net, name, initial_from_id, initial_to_id, from_witness, to_witness, layers) in edges {
            if Instant::now() >= self.ctx.deadline {
                break;
            }
            let Some(from_item) = board
                .board
                .item_repository()
                .iter()
                .find(|&key| board.board.item(key).id().0 == from_witness)
            else {
                outcomes.push(serde_json::json!({"net":name,"initial_from_item":initial_from_id,
                    "initial_to_item":initial_to_id,"from_witness":from_witness,
                    "to_witness":to_witness,"status":"from_witness_missing"}));
                continue;
            };
            let Some(to_item) = board
                .board
                .item_repository()
                .iter()
                .find(|&key| board.board.item(key).id().0 == to_witness)
            else {
                outcomes.push(serde_json::json!({"net":name,"initial_from_item":initial_from_id,
                    "initial_to_item":initial_to_id,"from_witness":from_witness,
                    "to_witness":to_witness,"status":"to_witness_missing"}));
                continue;
            };
            let from_id = board.board.item(from_item).id();
            let to_id = board.board.item(to_item).id();
            if !board.board.item(from_item).contains_net(net)
                || !board.board.item(to_item).contains_net(net)
            {
                outcomes.push(serde_json::json!({"net":name,"initial_from_item":initial_from_id,
                    "initial_to_item":initial_to_id,"from_witness":from_witness,
                    "to_witness":to_witness,"status":"witness_net_mismatch"}));
                continue;
            }
            let groups = board.board.get_connected_sets(net);
            let from_group = groups.iter().find(|group| group.contains(from_id));
            let to_group = groups.iter().find(|group| group.contains(to_id));
            if from_group.is_none() || to_group.is_none() {
                outcomes.push(serde_json::json!({"net":name,"initial_from_item":initial_from_id,
                    "initial_to_item":initial_to_id,"from_witness":from_witness,
                    "to_witness":to_witness,"status":"missing_connected_group"}));
                continue;
            }
            if groups
                .iter()
                .any(|group| group.contains(from_id) && group.contains(to_id))
            {
                outcomes.push(serde_json::json!({"net":name,"initial_from_item":initial_from_id,
                    "initial_to_item":initial_to_id,"from_witness":from_witness,
                    "to_witness":to_witness,"status":"already_connected"}));
                continue;
            }
            let starts = connected_group_candidates(board, from_group.unwrap(), &layers);
            let targets = connected_group_candidates(board, to_group.unwrap(), &layers);
            if starts.is_empty() || targets.is_empty() {
                outcomes.push(serde_json::json!({"net":name,"initial_from_item":initial_from_id,
                    "initial_to_item":initial_to_id,"from_witness":from_witness,
                    "to_witness":to_witness,"status":"no_native_endpoint"}));
                continue;
            }

            let before_net = NetIncompletes::new(
                &board.board,
                net,
                &board.board.get_connectable_items(net),
            )
            .count();
            let before_global =
                fr_engine::pipeline::stats::incomplete_count(&board.board, None);
            attempted += 1;
            let (candidate, _) = self.route_endpoint_set(
                board,
                net,
                &name,
                "residual_only",
                from_item,
                to_item,
                &starts,
                &targets,
                &layers,
            )?;
            let Some(candidate) = candidate else {
                outcomes.push(serde_json::json!({"net":name,"initial_from_item":initial_from_id,
                    "initial_to_item":initial_to_id,"from_witness":from_witness,
                    "to_witness":to_witness,"status":"native_declined",
                    "start_candidates":starts.len(),"target_candidates":targets.len()}));
                continue;
            };
            let after_net = NetIncompletes::new(
                &candidate.board,
                net,
                &candidate.board.get_connectable_items(net),
            )
            .count();
            let after_global =
                fr_engine::pipeline::stats::incomplete_count(&candidate.board, None);
            if after_net >= before_net
                || after_global >= before_global
                || candidate
                    .assert_invariants(self.baseline, self.protected, true)
                    .is_err()
            {
                outcomes.push(serde_json::json!({"net":name,"initial_from_item":initial_from_id,
                    "initial_to_item":initial_to_id,"from_witness":from_witness,
                    "to_witness":to_witness,"status":"rejected_validation",
                    "net_incompletes_before":before_net,"net_incompletes_after":after_net,
                    "board_incompletes_before":before_global,"board_incompletes_after":after_global}));
                continue;
            }
            *board = candidate;
            accepted += 1;
            self.accepted += 1;
            self.record_best(board, &[net])?;
            outcomes.push(serde_json::json!({"net":name,"initial_from_item":initial_from_id,
                "initial_to_item":initial_to_id,"from_witness":from_witness,
                "to_witness":to_witness,"status":"accepted",
                "start_candidates":starts.len(),"target_candidates":targets.len(),
                "net_incompletes_before":before_net,"net_incompletes_after":after_net,
                "board_incompletes_before":before_global,"board_incompletes_after":after_global}));
        }
        Ok(serde_json::json!({"attempted":attempted,"accepted":accepted,"outcomes":outcomes}))
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
        self.phase3_taps_inner(board, net, name, plan, routed, route_layers)
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
                self.record_best(board, &[net])?;
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
                    self.record_best(board, &changed_nets)?;
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
    ) -> Result<RecoveryRouteOutcome, String> {
        self.route_net_fully_inner(board, net, name, route_layers)
    }

    fn route_net_fully_inner(
        &mut self,
        board: &mut BoardAdapter,
        net: NetNo,
        name: &str,
        route_layers: &[(i32, String)],
    ) -> Result<RecoveryRouteOutcome, String> {
        match self.phase1_route(board, net, name, route_layers)? {
            Phase1Outcome::Plain => Ok(RecoveryRouteOutcome::Complete),
            Phase1Outcome::Failed => Ok(RecoveryRouteOutcome::NoMain),
            Phase1Outcome::Pending { plan, routed } => {
                let mut routed = routed;
                self.phase3_taps(board, net, name, &plan, &mut routed, route_layers)?;
                let items = board.board.get_connectable_items(net);
                if NetIncompletes::new(&board.board, net, &items).count() == 0 {
                    Ok(RecoveryRouteOutcome::Complete)
                } else {
                    Ok(RecoveryRouteOutcome::PartialMain)
                }
            }
        }
    }

    /// Reroute the victims queued by every accepted rip, in FIFO rip order,
    /// each with its main edge and immediate taps. Bounded by the run deadline.
    fn drain_reroute_queue(&mut self, board: &mut BoardAdapter) -> Result<(), String> {
        self.drain_reroute_queue_inner(board)
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
            let outcome = if Instant::now() >= self.ctx.deadline || route_layers.is_empty() {
                RecoveryRouteOutcome::NoMain
            } else {
                self.route_net_fully(board, victim, &name, &route_layers)?
            };
            if matches!(outcome, RecoveryRouteOutcome::NoMain) {
                let restore = crate::awx::endpoint_recovery::restore_failed_victim(
                    &saved_board,
                    board,
                    self.baseline,
                    self.protected,
                    victim,
                );
                let event = serde_json::json!({
                    "event": "terminal_restore",
                    "net": name,
                    "status": restore.status,
                    "full_restore_error": restore.full_restore_error,
                    "full_invariant_error": restore.full_invariant_error,
                    "stub_error": restore.stub_error,
                    "restored_segments": restore.restored_segments,
                    "restored_vias": restore.restored_vias,
                    "drc_scope": "rejected_restore_candidates",
                    "retained_new_drc": restore.retained_new_drc,
                });
                log_event(&mut self.log, &event)?;
                if let Some(restored) = restore.board {
                    *board = restored;
                    self.record_best(board, &[victim])?;
                }
                self.recovery.record_terminal_restore(
                    victim,
                    Arc::clone(&saved_board),
                    &restore.status,
                );
            }
        }
        Ok(())
    }

    /// Run one bounded source-style nested pass on a frozen subset of nets.
    fn scoped_lap(
        &mut self,
        board: &mut BoardAdapter,
        scope: &[NetNo],
        authorized: &BTreeSet<NetNo>,
        source: &BoardAdapter,
    ) -> Result<BTreeMap<NetNo, Vec<NetNo>>, String> {
        self.recovery.begin_nested(board, authorized);
        self.retry_counts.clear();
        self.pending.clear();

        for &net in scope {
            if Instant::now() >= self.ctx.deadline { break; }
            let Some(name) = board.board.rules.nets.get(net).map(|n| n.name.clone()) else { continue; };
            if self.protected.contains(&name) { continue; }
            let items = board.board.get_connectable_items(net);
            if NetIncompletes::new(&board.board, net, &items).count() == 0 { continue; }
            let route_layers = net_route_layers(board, net);
            if route_layers.is_empty() { continue; }
            if let Phase1Outcome::Pending { plan, routed } = self.phase1_route(board, net, &name, &route_layers)? {
                self.pending.push(PendingNet { net, name, plan, routed, route_layers });
            }
        }
        self.drain_reroute_queue(board)?;

        let tap_order = boxed_in_risk_order(board, &self.pending);
        for net in tap_order {
            if Instant::now() >= self.ctx.deadline { break; }
            let Some(position) = self.pending.iter().position(|entry| entry.net == net) else { continue; };
            let mut entry = self.pending.remove(position);
            let items = board.board.get_connectable_items(net);
            if NetIncompletes::new(&board.board, net, &items).count() > 0 {
                self.phase3_taps(board, entry.net, &entry.name, &entry.plan, &mut entry.routed, &entry.route_layers)?;
            }
            self.pending.push(entry);
        }
        self.drain_reroute_queue(board)?;

        for (victim, saved) in self.recovery.take_priority_candidates() {
            if Instant::now() >= self.ctx.deadline { break; }
            let Some(candidate) = crate::awx::final_reconcile::prepare_priority_restore(
                board, source, self.protected, victim, &saved,
                self.recovery.routed_result_nets(), self.recovery.restore_protected_nets(),
            )? else { continue; };
            let before = board.clone();
            let changed = candidate.owners.iter().copied().chain([candidate.victim]).collect::<Vec<_>>();
            *board = candidate.board;
            self.recovery.commit_priority_restore(candidate.victim, candidate.owners, &before);
            self.accepted += 1;
            self.record_best(board, &changed)?;
            log_event(&mut self.log, &serde_json::json!({
                "event":"nested_priority_restore", "victim":candidate.victim,
                "owners":changed.iter().copied().filter(|n| *n != candidate.victim).collect::<Vec<_>>(),
                "verdict":candidate.verdict
            }))?;
        }
        self.drain_reroute_queue(board)?;

        let live_count = fr_engine::pipeline::stats::incomplete_count(&board.board, None);
        if live_count > self.best_incompletes {
            *board = self.best_board.clone();
            let (counts, total) = full_incomplete_counts(board);
            self.incomplete_counts = counts;
            self.incomplete_total = total;
            self.count_witness = IncompleteCountWitness::new(board);
        }

        let scoped = scope.iter().filter_map(|&net| {
            board.board.rules.nets.get(net).map(|info| (net, info.name.clone()))
        }).collect::<Vec<_>>();
        self.fine_grid_rescue(board, &scoped, None)?;
        self.connected_group_residual_rescue(board, &scoped)?;
        Ok(self.recovery.take_preexisting_hints())
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
        min_annular_ring_mm: ctx.min_annular_ring_mm,
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
    let job: EndpointJob = toml::from_str(&fs::read_to_string(path).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())?;
    if job.native_rescue.post_awx_budget_ms > 0
        && job.native_rescue.stage != NativeRescueStage::BeforeAwx
    {
        return Err("post_awx_budget_ms requires native_rescue.stage = before_awx".into());
    }
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
    crate::awx::board_snapshot::BoardSnapshot::apply_configured_via_drill(
        &mut board,
        job.via_drill_mm,
    )?;
    board.configure_awx_hole_rules(job.hole_to_hole_clearance_mm);
    board
        .board
        .set_pad_area_contacts(job.pad_access.uses_copper_area());

    // Preserve the configured-rule source as the invariant baseline. The
    // optional guarded fanout stage may prepare Cu for the endpoint driver,
    // but it does not redefine the source used for attribution and guards.
    let baseline = board.clone();
    let before = fr_engine::pipeline::stats::incomplete_count(&baseline.board, None);
    let started = Instant::now();
    let overall_deadline = started + Duration::from_millis(job.budget_ms);
    let late_native_budget_ms = if job.native_rescue.stage == NativeRescueStage::BeforeAwx {
        job.native_rescue.post_awx_budget_ms
    } else {
        job.native_rescue.budget_ms
    };
    let rescue_reserved_ms = if job.native_rescue.enabled {
        late_native_budget_ms
            .saturating_add(job.native_rescue.post_grid_budget_ms)
            .min(job.budget_ms / 2)
    } else {
        0
    };
    let native_rescue_reserved_ms = if job.native_rescue.enabled {
        late_native_budget_ms.min(rescue_reserved_ms)
    } else {
        0
    };
    let post_grid_rescue_reserved_ms = rescue_reserved_ms - native_rescue_reserved_ms;
    let native_rescue_before_awx = job.native_rescue.enabled
        && job.native_rescue.stage == NativeRescueStage::BeforeAwx;
    let deadline = overall_deadline - Duration::from_millis(rescue_reserved_ms);
    let native_rescue_deadline =
        overall_deadline - Duration::from_millis(post_grid_rescue_reserved_ms);
    let pre_fanout = native_fanout_stage::run(
        &mut board,
        &baseline,
        &job.protected_nets,
        &job.pre_fanout,
        deadline,
        job.min_annular_ring_mm,
    )?;

    let (free_via_pads, same_net_pads) =
        endpoint_pad_metadata(&board, job.hole_to_hole_clearance_mm > 0.0)?;
    let (incomplete_counts, incomplete_total) = full_incomplete_counts(&board);
    let prepared_before = fr_engine::pipeline::stats::incomplete_count(&board.board, None);
    if incomplete_total != prepared_before {
        return Err(format!(
            "per-net prepared count disagrees with full count: per_net={incomplete_total}, full={prepared_before}"
        ));
    }
    let count_witness = IncompleteCountWitness::new(&board);
    let mut log = fs::File::create(&log_path).map_err(|e| e.to_string())?;
    log_event(
        &mut log,
        &serde_json::json!({"stage":"pre_fanout","summary":pre_fanout}),
    )?;
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
        best_incompletes: prepared_before,
        incomplete_counts,
        incomplete_total,
        count_witness,
        record_best_calls: 0,
        retry_counts: BTreeMap::new(),
    };

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

    let pre_fine_started = Instant::now();
    let pre_fine_before = fr_engine::pipeline::stats::incomplete_count(&board.board, None);
    let original_failed_pin_ids: BTreeSet<i32> = pre_fanout
        .failed_pins
        .iter()
        .map(|failure| failure.pin_id)
        .collect();
    let failed_pin_net_pairs: BTreeSet<(i32, NetNo)> = pre_fanout
        .failed_pins
        .iter()
        .map(|failure| (failure.pin_id, failure.net))
        .collect();
    let pre_fine_deadline = (started + Duration::from_millis(job.pre_fanout.budget_ms)).min(deadline);
    let eligibility_scan_requested = pre_fanout.enabled
        && job.pre_fanout.fine_grid_fallback
        && !failed_pin_net_pairs.is_empty();
    let eligibility_scan_enabled =
        eligibility_scan_requested && Instant::now() < pre_fine_deadline;
    let mut failed_pin_scope = BTreeSet::new();
    let mut failed_pin_exclusions = Vec::new();
    let mut eligibility_scan_truncated = eligibility_scan_requested && !eligibility_scan_enabled;
    if eligibility_scan_enabled {
        for (pin_index, &(pin_id, net)) in failed_pin_net_pairs.iter().enumerate() {
            if Instant::now() >= pre_fine_deadline {
                eligibility_scan_truncated = pin_index < failed_pin_net_pairs.len();
                break;
            }
            let Some(source_key) = baseline.board.get_item(ItemId(pin_id)) else {
                failed_pin_exclusions.push(serde_json::json!({"pin_id":pin_id,"net":net,"reason":"missing_from_source"}));
                continue;
            };
            let source_pin = baseline.board.item(source_key);
            if !source_pin.is_pin()
                || !source_pin.contains_net(net)
                || source_pin.first_layer(&baseline.board) != source_pin.last_layer(&baseline.board)
            {
                failed_pin_exclusions.push(serde_json::json!({"pin_id":pin_id,"net":net,"reason":"not_original_single_layer_smd"}));
                continue;
            }
            let Some(current_key) = board.board.get_item(ItemId(pin_id)) else {
                failed_pin_exclusions.push(serde_json::json!({"pin_id":pin_id,"net":net,"reason":"missing_from_current_board"}));
                continue;
            };
            let current_pin = board.board.item(current_key);
            if !current_pin.is_pin()
                || !current_pin.contains_net(net)
                || current_pin.first_layer(&board.board) != current_pin.last_layer(&board.board)
            {
                failed_pin_exclusions.push(serde_json::json!({"pin_id":pin_id,"net":net,"reason":"no_longer_single_layer_pin_on_original_net"}));
                continue;
            }
            let live = NetIncompletes::new(
                &board.board,
                net,
                &board.board.get_connectable_items(net),
            );
            if live.count() == 0 {
                failed_pin_exclusions.push(serde_json::json!({"pin_id":pin_id,"net":net,"reason":"net_already_connected"}));
                continue;
            }
            let route_layers = net_route_layers(&board, net);
            if route_layers.is_empty() {
                failed_pin_exclusions.push(serde_json::json!({"pin_id":pin_id,"net":net,"reason":"no_active_route_layers"}));
                continue;
            }
            let pin_layer = current_pin.first_layer(&board.board);
            if !route_layers.iter().any(|(layer, _)| *layer == pin_layer) {
                failed_pin_exclusions.push(serde_json::json!({"pin_id":pin_id,"net":net,"reason":"pin_layer_not_active"}));
                continue;
            }
            let group = board.board.connected_set(current_key, net, false);
            let candidates = connected_group_candidates(&board, &group, &route_layers);
            if candidates.is_empty() {
                failed_pin_exclusions.push(serde_json::json!({"pin_id":pin_id,"net":net,"reason":"no_active_group_candidate"}));
                continue;
            }
            let has_other_active_layer = route_layers.iter().any(|(layer, name)| {
                *layer != pin_layer
                    && candidates
                        .iter()
                        .any(|candidate| candidate.layer.as_str() == name.as_str())
            });
            if has_other_active_layer {
                failed_pin_exclusions.push(serde_json::json!({"pin_id":pin_id,"net":net,"reason":"connected_group_has_other_active_layer_candidate"}));
                continue;
            }
            failed_pin_scope.insert(pin_id);
        }
    }
    let failed_net_ids: BTreeSet<NetNo> = failed_pin_net_pairs
        .iter()
        .filter_map(|(pin_id, net)| failed_pin_scope.contains(pin_id).then_some(*net))
        .collect();
    let pre_fine_nets: Vec<(NetNo, String)> = original_nets
        .iter()
        .filter(|(net, _)| failed_net_ids.contains(net))
        .cloned()
        .collect();
    let pre_fine_skip_reason = if !pre_fanout.enabled {
        Some("pre_fanout_disabled")
    } else if !job.pre_fanout.fine_grid_fallback {
        Some("fine_grid_fallback_disabled")
    } else if pre_fanout.failed_pins.is_empty() {
        Some("no_native_failed_pins")
    } else if eligibility_scan_truncated || Instant::now() >= pre_fine_deadline {
        Some("pre_fanout_budget_exhausted")
    } else if failed_pin_scope.is_empty() || pre_fine_nets.is_empty() {
        Some("no_eligible_failed_pin")
    } else {
        None
    };
    let (pre_fine_summary, pre_fine_ran) = if pre_fine_skip_reason.is_none() {
        let previous_deadline = driver.ctx.deadline;
        driver.ctx.deadline = pre_fine_deadline;
        let result = driver.fine_grid_rescue(&mut board, &pre_fine_nets, Some(&failed_pin_scope));
        driver.ctx.deadline = previous_deadline;
        (result?, true)
    } else {
        (serde_json::Value::Null, false)
    };
    let after_pre_fine_grid =
        fr_engine::pipeline::stats::incomplete_count(&board.board, None);
    let (pre_fine_counts, pre_fine_total) = full_incomplete_counts(&board);
    if pre_fine_total != after_pre_fine_grid || driver.incomplete_total != after_pre_fine_grid {
        return Err(format!(
            "pre-native-failure fine-grid count disagrees with driver/full board: driver={}, per_net={pre_fine_total}, full={after_pre_fine_grid}",
            driver.incomplete_total
        ));
    }
    driver.incomplete_counts = pre_fine_counts;
    driver.incomplete_total = pre_fine_total;
    driver.count_witness = IncompleteCountWitness::new(&board);
    let no_scope_gap_candidate = pre_fine_ran
        && pre_fine_summary.as_object().is_some_and(|summaries| {
            summaries
                .values()
                .all(|summary| summary["attempted"].as_u64().unwrap_or(0) == 0)
        });
    let pre_fine_reported_skip_reason = pre_fine_skip_reason
        .or_else(|| no_scope_gap_candidate.then_some("no_scope_connected_group_gap"));
    let pre_fine_grid_rescue = serde_json::json!({
        "ran":pre_fine_ran,
        "skip_reason":pre_fine_reported_skip_reason,
        "original_failed_pin_ids":original_failed_pin_ids,
        "eligible_pin_ids":failed_pin_scope,
        "excluded_pins":failed_pin_exclusions,
        "before_incompletes":pre_fine_before,
        "after_incompletes":after_pre_fine_grid,
        "elapsed_ms":pre_fine_started.elapsed().as_millis() as u64,
        "summary":pre_fine_summary
    });

    let mut early_native_rescue: Option<(i32, i32, NativeRescueSummary)> = None;
    if native_rescue_before_awx {
        let early_before = fr_engine::pipeline::stats::incomplete_count(&board.board, None);
        let early_summary = native_fanout_stage::run_rescue(
            &mut board,
            &baseline,
            &job.protected_nets,
            &job.native_rescue,
            deadline,
            job.min_annular_ring_mm,
        )?;
        let (early_counts, early_total) = full_incomplete_counts(&board);
        let early_after = fr_engine::pipeline::stats::incomplete_count(&board.board, None);
        if early_total != early_after {
            return Err(format!(
                "per-net early native rescue count disagrees with full count: per_net={early_total}, full={early_after}"
            ));
        }
        if early_after > early_before {
            return Err(format!(
                "early native rescue increased incompletes: before={early_before}, after={early_after}"
            ));
        }
        driver.incomplete_counts = early_counts;
        driver.incomplete_total = early_total;
        driver.count_witness = IncompleteCountWitness::new(&board);
        driver.best_board = board.clone();
        driver.best_incompletes = early_total;
        early_native_rescue = Some((early_before, early_after, early_summary));
    }

    let nets = crate::awx::net_ordering::order_native_nets_mps(&board, &original_nets)?;

    // ---- Phase 1: one main edge per multipoint net; plain nets fully. ----
    // Best-board custody now lives on the driver and is snapshotted after EVERY
    // accepted revision (main, tap, rip, reroute), not only at phase boundaries.
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

    // Victims ripped while routing the mains are rerouted before the tap pass;
    // a reroute invalidates the victim's stale pending entry inside the drain.
    driver.drain_reroute_queue(&mut board)?;

    // ---- Phase 3: remaining MST edges of every pending net, after ALL mains. ----
    // Source `_order_nets_by_boxed_in_risk`: the most boxed-in pending nets route
    // FIRST (measured on the board as it stands after every main), NOT the
    // phase-1 insertion order.
    let phase3_order = boxed_in_risk_order(&board, &driver.pending);

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

    // Victims ripped by the tap edges are rerouted after the taps.
    driver.drain_reroute_queue(&mut board)?;

    // Budget may end with casualties: keep the best consistent board instead of
    // the ripped one, so the run never claims worse connectivity than it made.
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
    let full_live_count_before_rescue =
        fr_engine::pipeline::stats::incomplete_count(&board.board, None);
    if full_live_count_before_rescue != driver.incomplete_total {
        return Err(format!(
            "record_best count disagrees after best-board restore: cached={}, full={full_live_count_before_rescue}",
            driver.incomplete_total
        ));
    }
    let fine_grid_rescue = driver.fine_grid_rescue(&mut board, &nets, None)?;
    let connected_group_residual_rescue =
        driver.connected_group_residual_rescue(&mut board, &nets)?;
    let mut final_reconcile_laps = Vec::new();
    let mut final_scope = crate::awx::final_reconcile::open_nets(&board, &job.protected_nets);
    final_scope.sort_by_key(|net| {
        nets.iter().position(|(candidate, _)| candidate == net).unwrap_or(usize::MAX)
    });
    let net_names = |board: &BoardAdapter, ids: &BTreeSet<NetNo>| -> Vec<String> {
        ids.iter().filter_map(|&net| board.board.rules.nets.get(net).map(|info| info.name.clone())).collect()
    };
    let final_scope_names: Vec<String> = final_scope.iter()
        .filter_map(|&net| board.board.rules.nets.get(net).map(|info| info.name.clone()))
        .collect();
    let scope_set: BTreeSet<NetNo> = final_scope.iter().copied().collect();
    let source_net_set: BTreeSet<NetNo> = original_nets.iter().map(|(net, _)| *net).collect();
    let mut authorized = BTreeSet::new();
    let initial_hints = driver.recovery.take_preexisting_hints();
    let initial_grants = crate::awx::final_reconcile::grant_authority(
        &board, &initial_hints, &scope_set, &job.protected_nets, &mut authorized,
    );
    let initial_grant_set: BTreeSet<NetNo> = initial_grants.iter().copied().collect();
    let initial_grant_names = net_names(&board, &initial_grant_set);
    let mut pending_hints = initial_hints;
    let mut stop_reason = if final_scope.is_empty() { "no_open_nets" } else { "lap_limit" };
    for lap in 0..3 {
        if Instant::now() >= deadline {
            stop_reason = "deadline";
            break;
        }
        let open_now: BTreeSet<NetNo> = crate::awx::final_reconcile::open_nets(
            &board,
            &job.protected_nets,
        )
        .into_iter()
        .collect();
        let mut lap_scope_set: BTreeSet<NetNo> = scope_set
            .iter()
            .copied()
            .filter(|net| {
                source_net_set.contains(net)
                    && board
                        .board
                        .rules
                        .nets
                        .get(*net)
                        .is_some_and(|info| !job.protected_nets.contains(&info.name))
            })
            .collect();
        lap_scope_set.extend(authorized.iter().copied().filter(|net| {
            open_now.contains(net)
                && source_net_set.contains(net)
                && board
                    .board
                    .rules
                    .nets
                    .get(*net)
                    .is_some_and(|info| !job.protected_nets.contains(&info.name))
        }));
        let lap_scope: Vec<NetNo> = nets
            .iter()
            .filter_map(|(net, _)| lap_scope_set.contains(net).then_some(*net))
            .collect();
        let lap_scope_names: Vec<String> = lap_scope
            .iter()
            .filter_map(|&net| board.board.rules.nets.get(net).map(|info| info.name.clone()))
            .collect();
        if lap_scope.is_empty() {
            stop_reason = "no_open_nets";
            break;
        }
        let before_board = board.clone();
        let before_best = driver.best_board.clone();
        let before_best_count = driver.best_incompletes;
        let before_counts = driver.incomplete_counts.clone();
        let before_total = driver.incomplete_total;
        let before_witness = IncompleteCountWitness::new(&before_board);
        let before_quality = crate::awx::final_reconcile::quality_score(&before_board);
        let mut mutable = scope_set.clone();
        mutable.extend(authorized.iter().copied());
        let lap_result = driver.scoped_lap(&mut board, &lap_scope, &authorized, &baseline);
        let mut hints = match lap_result {
            Ok(hints) => hints,
            Err(error) => {
                board = before_board.clone();
                driver.best_board = before_best;
                driver.best_incompletes = before_best_count;
                driver.incomplete_counts = before_counts;
                driver.incomplete_total = before_total;
                driver.count_witness = before_witness;
                final_reconcile_laps.push(serde_json::json!({"lap":lap+1,"scope":lap_scope,"authority":authorized,
                    "scope_names":lap_scope_names,"authority_names":net_names(&board, &authorized),
                    "before":before_quality,"after":before_quality,"accepted":false,"new_grants":[],
                    "new_grant_names":[],"error":error}));
                stop_reason = "lap_error";
                break;
            }
        };
        let live_quality = crate::awx::final_reconcile::quality_score(&board);
        let best_quality = crate::awx::final_reconcile::quality_score(&driver.best_board);
        let live_guard = crate::awx::final_reconcile::assert_lap_candidate(
            &before_board, &board, &baseline, &job.protected_nets, &mutable,
        );
        let best_guard = crate::awx::final_reconcile::assert_lap_candidate(
            &before_board, &driver.best_board, &baseline, &job.protected_nets, &mutable,
        );
        if live_guard.is_err() && best_guard.is_err() {
            let error = format!("live candidate rejected: {}; best candidate rejected: {}",
                live_guard.unwrap_err(), best_guard.unwrap_err());
            board = before_board.clone();
            driver.best_board = before_best;
            driver.best_incompletes = before_best_count;
            driver.incomplete_counts = before_counts;
            driver.incomplete_total = before_total;
            driver.count_witness = before_witness;
            final_reconcile_laps.push(serde_json::json!({"lap":lap+1,"scope":lap_scope,"authority":authorized,
                "scope_names":lap_scope_names,"authority_names":net_names(&board, &authorized),
                "before":before_quality,"live_candidate":live_quality,"best_candidate":best_quality,
                "accepted":false,"new_grants":[],"new_grant_names":[],"error":error}));
            stop_reason = "candidate_rejected";
            break;
        }
        let choose_best = best_guard.is_ok()
            && (live_guard.is_err()
                || best_quality.incomplete_connections < live_quality.incomplete_connections);
        if choose_best {
            board = driver.best_board.clone();
        } else {
            driver.best_board = board.clone();
        }
        let selected_quality = if choose_best { best_quality.clone() } else { live_quality.clone() };
        let selected_name = if choose_best { "best" } else { "live" };
        let new_grants = crate::awx::final_reconcile::grant_authority(
            &board, &hints, &scope_set, &job.protected_nets, &mut authorized,
        );
        let new_grant_set: BTreeSet<NetNo> = new_grants.iter().copied().collect();
        let new_grant_names = net_names(&board, &new_grant_set);
        let (counts, total) = full_incomplete_counts(&board);
        driver.incomplete_counts = counts;
        driver.incomplete_total = total;
        driver.count_witness = IncompleteCountWitness::new(&board);
        driver.best_incompletes = total;
        let improved = selected_quality.broken_nets < before_quality.broken_nets
            || selected_quality.disconnected_pins < before_quality.disconnected_pins
            || selected_quality.incomplete_connections < before_quality.incomplete_connections;
        final_reconcile_laps.push(serde_json::json!({"lap":lap+1,"scope":lap_scope,"authority":authorized,
            "scope_names":lap_scope_names,"authority_names":net_names(&board, &authorized),
            "new_grant_names":new_grant_names,
            "before":before_quality,"after":selected_quality,"live_candidate":live_quality,
            "best_candidate":best_quality,"selected":selected_name,
            "accepted":true,"new_grants":new_grants,"improved":improved}));
        let open_custody_remains = crate::awx::final_reconcile::open_nets(
            &board,
            &job.protected_nets,
        )
        .iter()
        .any(|net| {
            (scope_set.contains(net) || authorized.contains(net))
                && source_net_set.contains(net)
                && board
                    .board
                    .rules
                    .nets
                    .get(*net)
                    .is_some_and(|info| !job.protected_nets.contains(&info.name))
        });
        if !open_custody_remains {
            stop_reason = "scope_completed";
            break;
        }
        if !improved && new_grants.is_empty() {
            stop_reason = "no_improvement_or_new_authority";
            break;
        }
        pending_hints = std::mem::take(&mut hints);
        if lap == 2 { stop_reason = "lap_limit"; }
    }
    let final_reconcile = serde_json::json!({"scope":final_scope,"scope_names":final_scope_names,
        "initial_authority":initial_grants,"initial_authority_names":initial_grant_names,
        "laps":final_reconcile_laps,"stop_reason":stop_reason,"remaining_authority":authorized,
        "remaining_authority_names":net_names(&board, &authorized),
        "pending_hints":pending_hints});
    let (before_native_rescue, after_native_rescue, native_rescue) =
        if let Some(early_result) = early_native_rescue.take() {
            early_result
        } else {
            let before_native_rescue =
                fr_engine::pipeline::stats::incomplete_count(&board.board, None);
            let original_best_incompletes = driver.best_incompletes;
            let native_rescue = native_fanout_stage::run_rescue(
                &mut board,
                &baseline,
                &job.protected_nets,
                &job.native_rescue,
                native_rescue_deadline,
                job.min_annular_ring_mm,
            )?;
            let (incomplete_counts, incomplete_total) = full_incomplete_counts(&board);
            let after_native_rescue =
                fr_engine::pipeline::stats::incomplete_count(&board.board, None);
            if incomplete_total != after_native_rescue {
                return Err(format!(
                    "per-net native rescue count disagrees with full count: per_net={incomplete_total}, full={after_native_rescue}"
                ));
            }
            driver.incomplete_counts = incomplete_counts;
            driver.incomplete_total = incomplete_total;
            driver.count_witness = IncompleteCountWitness::new(&board);
            if incomplete_total > original_best_incompletes {
                return Err(format!(
                    "native rescue increased incompletes beyond the prior best: before_best={original_best_incompletes}, after={incomplete_total}"
                ));
            }
            driver.best_board = board.clone();
            driver.best_incompletes = incomplete_total;
            (before_native_rescue, after_native_rescue, native_rescue)
        };
    let mut post_awx_native_rescue = None;
    let mut post_awx_native_accepted = false;
    let mut post_awx_native_workers = 0usize;
    let mut post_awx_native_improved = false;
    if native_rescue_before_awx {
        let post_awx_before = fr_engine::pipeline::stats::incomplete_count(&board.board, None);
        let (post_awx_entry_counts, post_awx_entry_total) = full_incomplete_counts(&board);
        if post_awx_entry_total != post_awx_before {
            return Err(format!(
                "per-net post-AWX native rescue entry count disagrees with full count: per_net={post_awx_entry_total}, full={post_awx_before}"
            ));
        }
        let post_awx_best_before = driver.best_incompletes;
        let open_before_post_awx =
            crate::awx::final_reconcile::open_nets(&board, &job.protected_nets);
        let post_awx_skip_reason = if native_rescue_reserved_ms == 0 {
            Some("no_post_awx_budget_reserved")
        } else if open_before_post_awx.is_empty() {
            Some("no_open_nets")
        } else {
            None
        };
        if let Some(skip_reason) = post_awx_skip_reason {
            post_awx_native_rescue = Some(serde_json::json!({
                "before_incompletes":post_awx_before,
                "after_incompletes":post_awx_before,
                "summary":serde_json::Value::Null,
                "budget_ms":native_rescue_reserved_ms,
                "skip_reason":skip_reason
            }));
        } else {
            let mut post_awx_config = job.native_rescue.clone();
            post_awx_config.stage = NativeRescueStage::AfterAwx;
            post_awx_config.budget_ms = native_rescue_reserved_ms;
            post_awx_config.post_awx_budget_ms = 0;
            let post_awx_summary = native_fanout_stage::run_rescue(
                &mut board,
                &baseline,
                &job.protected_nets,
                &post_awx_config,
                native_rescue_deadline,
                job.min_annular_ring_mm,
            )?;
            let (post_awx_counts, post_awx_total) = full_incomplete_counts(&board);
            let post_awx_after = fr_engine::pipeline::stats::incomplete_count(&board.board, None);
            if post_awx_total != post_awx_after {
                return Err(format!(
                    "per-net post-AWX native rescue count disagrees with full count: per_net={post_awx_total}, full={post_awx_after}"
                ));
            }
            if post_awx_after > post_awx_before || post_awx_after > post_awx_best_before {
                return Err(format!(
                    "post-AWX native rescue worsened its entry/best count: before={post_awx_before}, best={post_awx_best_before}, after={post_awx_after}"
                ));
            }
            driver.incomplete_counts = post_awx_counts;
            driver.incomplete_total = post_awx_total;
            driver.count_witness = IncompleteCountWitness::new(&board);
            driver.best_board = board.clone();
            driver.best_incompletes = post_awx_total;
            post_awx_native_accepted = post_awx_summary.accepted > 0;
            post_awx_native_workers = post_awx_summary.multi_start_actual_workers;
            post_awx_native_improved = post_awx_after < post_awx_before;
            post_awx_native_rescue = Some(serde_json::json!({
                "before_incompletes":post_awx_before,
                "after_incompletes":post_awx_after,
                "summary":post_awx_summary,
                "budget_ms":native_rescue_reserved_ms,
                "skip_reason":serde_json::Value::Null
            }));
        }
    }
    let native_improved = after_native_rescue < before_native_rescue || post_awx_native_improved;
    let post_grid_before =
        fr_engine::pipeline::stats::incomplete_count(&board.board, None);
    let (post_entry_counts, post_entry_total) = full_incomplete_counts(&board);
    if post_entry_total != post_grid_before {
        return Err(format!(
            "per-net post-grid entry count disagrees with full count: per_net={post_entry_total}, full={post_grid_before}"
        ));
    }
    driver.incomplete_counts = post_entry_counts;
    driver.incomplete_total = post_entry_total;
    driver.count_witness = IncompleteCountWitness::new(&board);
    if post_entry_total <= driver.best_incompletes {
        driver.best_board = board.clone();
        driver.best_incompletes = post_entry_total;
    }
    let post_grid_started = Instant::now();
    let open_after_native = crate::awx::final_reconcile::open_nets(&board, &job.protected_nets);
    let post_grid_skip_reason = if !native_improved {
        Some("native_no_global_improvement")
    } else if post_grid_rescue_reserved_ms == 0 {
        Some("no_post_grid_budget_reserved")
    } else if open_after_native.is_empty() {
        Some("no_open_nets")
    } else if Instant::now() >= overall_deadline {
        Some("overall_deadline_reached")
    } else {
        None
    };
    let (post_fine_grid_rescue, post_connected_group_residual_rescue, post_grid_ran) =
        if post_grid_skip_reason.is_none() {
            driver.ctx.deadline = overall_deadline.min(
                Instant::now() + Duration::from_millis(post_grid_rescue_reserved_ms),
            );
            let fine = driver.fine_grid_rescue(&mut board, &nets, None)?;
            let residual = driver.connected_group_residual_rescue(&mut board, &nets)?;
            (fine, residual, true)
        } else {
            (serde_json::Value::Null, serde_json::Value::Null, false)
        };
    board = driver.best_board.clone();
    let (post_counts, post_total) = full_incomplete_counts(&board);
    let after_post_grid_rescue =
        fr_engine::pipeline::stats::incomplete_count(&board.board, None);
    if post_total != after_post_grid_rescue {
        return Err(format!(
            "per-net post-native grid rescue count disagrees with full count: per_net={post_total}, full={after_post_grid_rescue}"
        ));
    }
    if post_total > post_grid_before {
        return Err(format!(
            "post-native grid rescue worsened its entry board: before_post={post_grid_before}, after_post={post_total}"
        ));
    }
    driver.incomplete_counts = post_counts;
    driver.incomplete_total = post_total;
    driver.count_witness = IncompleteCountWitness::new(&board);
    driver.best_board = board.clone();
    driver.best_incompletes = post_total;
    let post_native_grid_rescue = serde_json::json!({
        "ran":post_grid_ran,
        "skip_reason":post_grid_skip_reason,
        "before_incompletes":post_grid_before,
        "after_incompletes":after_post_grid_rescue,
        "elapsed_ms":post_grid_started.elapsed().as_millis() as u64,
        "fine_grid_rescue":post_fine_grid_rescue,
        "connected_group_residual_rescue":post_connected_group_residual_rescue
    });
    let final_board_incompletes =
        fr_engine::pipeline::stats::incomplete_count(&board.board, None);
    if final_board_incompletes != driver.incomplete_total {
        return Err(format!(
            "record_best final count disagrees after terminal rescues: cached={}, full={final_board_incompletes}",
            driver.incomplete_total
        ));
    }
    let record_best_final_check = serde_json::json!({
        "full_live_count":final_board_incompletes,
        "driver_count_after_restore":driver.incomplete_total,
        "final_diff":i64::from(final_board_incompletes) - i64::from(driver.incomplete_total),
        "driver_count_scope":"final_board_after_rescue",
        "full_live_count_before_rescue":full_live_count_before_rescue,
        "driver_count_before_restore":final_live_cached_total,
        "live_final_diff_before_rescue":final_live_diff,
        "best_board_restored":best_board_restored,
        "driver_count_scope_before_rescue":if best_board_restored {"best_board_after_sync"} else {"live_board"},
        "driver_count_after_restore_and_rescue":driver.incomplete_total,
        "full_final_count_after_rescue":final_board_incompletes,
        "final_diff_after_rescue":i64::from(final_board_incompletes) - i64::from(driver.incomplete_total),
    });

    board.assert_invariants(&baseline, &job.protected_nets, true)?;

    let session = board.ses();
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
    let mut replay =
        BoardAdapter::load_session_with_via_candidates(&input, &session, &replay_candidates)?;
    crate::awx::board_snapshot::BoardSnapshot::apply_configured_via_drill(
        &mut replay,
        job.via_drill_mm,
    )?;

    replay.configure_awx_hole_rules(job.hole_to_hole_clearance_mm);
    replay
        .board
        .set_pad_area_contacts(job.pad_access.uses_copper_area());
    replay.assert_session_invariants(&baseline, &job.protected_nets, true)?;

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

    let attempts = driver.attempts;
    let accepted = driver.accepted;
    let skips = driver.skips;
    let new_violations = board.new_violations_since(&baseline).len();
    let fanout_accepted = pre_fanout.accepted_direct + pre_fanout.accepted_via > 0;
    let native_accepted = native_rescue.accepted > 0 || post_awx_native_accepted;
    let actual_solver = match (native_accepted, fanout_accepted) {
        (true, true) => "native fanout + native maze + awx_grid::GridRouter",
        (true, false) => "native maze + awx_grid::GridRouter",
        (false, true) => "native fanout + awx_grid::GridRouter",
        (false, false) => "awx_grid::GridRouter",
    };
    let native_effective_workers = native_rescue
        .multi_start_actual_workers
        .max(post_awx_native_workers);
    let effective_threads = 1usize.max(native_effective_workers);

    let legacy_wall_seconds = started.elapsed().as_secs_f64();
    let final_unconnected =
        final_unconnected_inventory(&board, &nets, &attempts, &skips, &driver.retry_counts);
    let mut report = serde_json::json!({
        "actual_solver":actual_solver,"scope":"all native incompletes; active signal stack and native via transitions",
        "pad_access":job.pad_access.as_str(),
        "pre_fanout":pre_fanout,
        "pre_fine_grid_rescue":pre_fine_grid_rescue,
        "grid_step_mm":job.grid_step_mm,
        "scheduler":"native NetIncompletes Delaunay/Kruskal airlines; recomputed after each accepted transaction",
        "whole_bga_bus_stages":"not applicable: flattened component identity",
        "budget_ms":job.budget_ms,"requested_threads":job.threads,
        "effective_threads":effective_threads,
        "native_rescue_reserved_ms":native_rescue_reserved_ms,
        "post_grid_rescue_reserved_ms":post_grid_rescue_reserved_ms,
        "parallel_policy":"serial AWX board transactions; native multi-start on independent board clones","wall_seconds":legacy_wall_seconds,
        "record_best_final_check":record_best_final_check,
        "fine_grid_rescue":fine_grid_rescue,
        "connected_group_residual_rescue":connected_group_residual_rescue,
        "final_reconcile":final_reconcile,
        "native_rescue":native_rescue,
        "post_native_grid_rescue":post_native_grid_rescue,
        "final_unconnected_nets":final_unconnected,
        "before_incompletes":before,"after_incompletes":fr_engine::pipeline::stats::incomplete_count(&board.board,None),
        "after_final_incompletes":final_board_incompletes,
        "after_pre_fanout_incompletes":prepared_before,
        "after_pre_fine_grid_incompletes":after_pre_fine_grid,
        "before_native_rescue_incompletes":before_native_rescue,
        "after_native_rescue_incompletes":after_native_rescue,
        "after_post_grid_rescue_incompletes":after_post_grid_rescue,
        "session_replay_incompletes":fr_engine::pipeline::stats::incomplete_count(&replay.board,None),
        "session_replay_vias":replay_vias.len(), "session_replay_via_geometry":replay_vias,
        "session_replay_new_violations":session_replay_new_violations,
        "accepted":accepted,
        "new_vias":board.board.get_vias().len().saturating_sub(baseline.board.get_vias().len()),"new_violations":new_violations,
        "attempts":attempts,"skipped":skips,"output":job.output
    });
    let report_object = report
        .as_object_mut()
        .ok_or("final endpoint report is not a JSON object")?;
    report_object.insert(
        "awx_effective_threads".into(),
        serde_json::Value::from(1usize),
    );
    report_object.insert(
        "native_effective_workers".into(),
        serde_json::Value::from(native_effective_workers),
    );
    report_object.insert(
        "native_requested_starts".into(),
        serde_json::Value::from(job.native_rescue.multi_start),
    );
    let native_rescue_stage = match job.native_rescue.stage {
        NativeRescueStage::BeforeAwx => "before_awx",
        NativeRescueStage::AfterAwx => "after_awx",
    };
    report_object.insert(
        "native_rescue_stage".into(),
        serde_json::Value::from(native_rescue_stage),
    );
    if native_rescue_before_awx {
        report_object.insert(
            "post_awx_native_rescue_reserved_ms".into(),
            serde_json::Value::from(native_rescue_reserved_ms),
        );
        if let Some(post_awx_native_rescue) = post_awx_native_rescue {
            report_object.insert("post_awx_native_rescue".into(), post_awx_native_rescue);
        }
    }
    let serialized_report = serde_json::to_string_pretty(&report).map_err(|e| e.to_string())?;
    write_atomic(&output_path, &session)?;
    let mut stdout = std::io::stdout().lock();
    writeln!(stdout, "{}", serialized_report).map_err(|e| e.to_string())?;
    stdout.flush().map_err(|e| e.to_string())?;
    drop(stdout);
    Ok(())
}

/// Route only the airlines present on entry. Endpoints are frozen from the SES
/// snapshot, so copper added during this run cannot create new targets.
pub fn run_residual(path: &Path) -> Result<(), String> {
    crate::awx::native_pad_raster::clear_source_raster_cache();
    crate::board_adapter::clear_validation_baseline_cache();
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
    crate::awx::board_snapshot::BoardSnapshot::apply_configured_via_drill(
        &mut dsn_board,
        job.via_drill_mm,
    )?;
    dsn_board.configure_awx_hole_rules(job.hole_to_hole_clearance_mm);
    dsn_board
        .board
        .set_pad_area_contacts(job.pad_access.uses_copper_area());
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
    crate::awx::board_snapshot::BoardSnapshot::apply_configured_via_drill(
        &mut board,
        job.via_drill_mm,
    )?;
    board.configure_awx_hole_rules(job.hole_to_hole_clearance_mm);
    board
        .board
        .set_pad_area_contacts(job.pad_access.uses_copper_area());
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
        retry_counts: BTreeMap::new(),
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
                driver.record_best(&board, &[*net])?;
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
    crate::awx::board_snapshot::BoardSnapshot::apply_configured_via_drill(
        &mut replay,
        job.via_drill_mm,
    )?;
    replay.configure_awx_hole_rules(job.hole_to_hole_clearance_mm);
    replay
        .board
        .set_pad_area_contacts(job.pad_access.uses_copper_area());
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
    let output = serde_json::json!({"scope":"frozen initial residual airlines only","nets":if selected.is_empty(){"all".to_string()}else{job.nets.join(",")},
        "pad_access":job.pad_access.as_str(),
        "initial_net_incompletes":initial_by_net,"after_net_incompletes":after_by_net,
        "routing_complete":complete,"success":complete,
        "record_best_final_check":{"full_count":final_board_incompletes,"driver_count":driver.incomplete_total,"final_diff":final_diff},
        "ses_replay_net_incompletes":replay_by_net,
        "ses_replay_new_violations":replay_new_violations,
        "accepted":driver.accepted,"grid_step_mm":job.grid_step_mm,"budget_ms":job.budget_ms,"requested_threads":job.threads,"effective_threads":1,
        "elapsed_seconds":started.elapsed().as_secs_f64(),"gap_outcomes":outcomes,
        "attempts":driver.attempts,"frozen_initial_airline_count":initial_gaps.len(),
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

#[cfg(test)]
mod pad_access_tests {
    use super::{
        build_endpoint_request, rules_for_layer, EndpointContext, EndpointJob, PadAccess,
        ResidualJob,
    };
    use crate::{
        awx::{board_snapshot::BoardSnapshot, native_rung},
        board_adapter::BoardAdapter,
    };
    use std::time::{Duration, Instant};

    const CLASS_VIA_DSN: &str = r#"(pcb test (resolution mil 1000) (unit mil)
      (structure (layer Top (type signal)) (layer Bottom (type signal))
        (boundary (rect pcb 0 0 1000 1000)) (via VIA_SMALL VIA_LARGE)
        (rule (width 10) (clearance 4)))
      (library
        (padstack VIA_SMALL (shape (circle Top 20)) (shape (circle Bottom 20)))
        (padstack VIA_LARGE (shape (circle Top 24)) (shape (circle Bottom 24))))
      (network (net SMALL) (net LARGE)
        (class small SMALL (circuit (use_via VIA_SMALL)) (rule (width 10) (clearance 4)))
        (class large LARGE (circuit (use_via VIA_LARGE)) (rule (width 10) (clearance 4))))
      (wiring))"#;

    fn board_with_test_drills() -> BoardAdapter {
        let mut board = BoardAdapter::load(CLASS_VIA_DSN.as_bytes()).unwrap();
        BoardSnapshot::apply_configured_via_drill(&mut board, 0.3).unwrap();
        board
    }

    fn endpoint_context() -> EndpointContext {
        EndpointContext {
            free_via_pads: Vec::new(),
            same_net_pads: Vec::new(),
            grid_step_mm: 0.05,
            via_drill_mm: 0.3,
            min_annular_ring_mm: 0.1,
            hole_to_hole_clearance_mm: 0.0,
            max_iterations: 10_000,
            deadline: Instant::now() + Duration::from_secs(5),
        }
    }

    fn endpoint_job_toml(pad_access: Option<&str>) -> String {
        let pad_access = pad_access
            .map(|mode| format!("pad_access = \"{mode}\"\n"))
            .unwrap_or_default();
        format!(
            "input = \"board.dsn\"\noutput = \"routed.ses\"\nlog = \"routing.jsonl\"\nbudget_ms = 300000\ngrid_step_mm = 0.05\nmax_iterations = 200000\nthreads = 1\nvia_drill_mm = 0.3\nmin_annular_ring_mm = 0.1\nhole_to_hole_clearance_mm = 0\n{pad_access}protected_nets = []\n"
        )
    }

    fn residual_job_toml(pad_access: Option<&str>) -> String {
        let pad_access = pad_access
            .map(|mode| format!("pad_access = \"{mode}\"\n"))
            .unwrap_or_default();
        format!(
            "input = \"board.dsn\"\nsession = \"routed.ses\"\noutput = \"residual.ses\"\nlog = \"residual.json\"\nnets = []\nbudget_ms = 300000\ngrid_step_mm = 0.05\nmax_iterations = 200000\nthreads = 1\nvia_drill_mm = 0.3\nmin_annular_ring_mm = 0.1\nhole_to_hole_clearance_mm = 0\n{pad_access}protected_nets = []\n"
        )
    }

    #[test]
    fn endpoint_and_residual_jobs_default_to_copper_area() {
        let endpoint: EndpointJob = toml::from_str(&endpoint_job_toml(None)).unwrap();
        let residual: ResidualJob = toml::from_str(&residual_job_toml(None)).unwrap();

        assert!(matches!(endpoint.pad_access, PadAccess::CopperArea));
        assert!(matches!(residual.pad_access, PadAccess::CopperArea));
    }

    #[test]
    fn endpoint_job_accepts_explicit_legacy_mode() {
        let endpoint: EndpointJob =
            toml::from_str(&endpoint_job_toml(Some("legacy"))).unwrap();

        assert!(matches!(endpoint.pad_access, PadAccess::Legacy));
    }

    #[test]
    fn residual_job_accepts_explicit_copper_area_mode() {
        let residual: ResidualJob =
            toml::from_str(&residual_job_toml(Some("copper_area"))).unwrap();

        assert!(matches!(residual.pad_access, PadAccess::CopperArea));
    }

    #[test]
    fn endpoint_rules_use_each_nets_source_class_via() {
        let board = board_with_test_drills();
        let top = board.board.layer_structure.get_no("Top");
        for (name, expected) in [("SMALL", 0.508), ("LARGE", 0.6096)] {
            let net = board.net_number(name).unwrap();
            let rules = rules_for_layer(&board, net, top, 0.05, 0.3, 0.1, 0.0).unwrap();
            assert!((rules.via_size - expected).abs() < 1e-6, "{name}: {rules:?}");
            assert!((rules.via_drill - 0.3).abs() < 1e-6, "{name}: {rules:?}");
            assert_eq!(rules.net_via_sizes.len(), 1);
            let selected = rules.net_via_sizes.get(&(net as i32)).unwrap();
            assert!((selected[0] - expected).abs() <= 1e-9, "{name}: {selected:?}");
            assert!((selected[1] - 0.3).abs() <= 1e-9, "{name}: {selected:?}");
        }
    }

    #[test]
    fn endpoint_request_drops_unlisted_refit_and_unblock_vias() {
        let board = board_with_test_drills();
        let net = board.net_number("SMALL").unwrap();
        let layers = vec![(board.board.layer_structure.get_no("Top"), "Top".to_owned())];
        let request = build_endpoint_request(
            &board,
            &endpoint_context(),
            net,
            "SMALL",
            "test",
            crate::awx::braid_connect::RungEndpoint {
                point: [2.0, 2.0],
                layer: "Top".to_owned(),
            },
            crate::awx::braid_connect::RungEndpoint {
                point: [4.0, 2.0],
                layer: "Top".to_owned(),
            },
            &layers,
        )
        .unwrap();
        let mut request = request;
        request.rules.via_size = 1.5;
        request.rules.via_drill = 0.4;
        request.fab_via_rungs = vec![(1.5, 0.4), (0.6096, 0.3), (0.508, 0.3)];
        let mut starts = vec![request.start.clone()];
        let mut targets = vec![request.end.clone()];
        assert!(native_rung::normalize_request_vias(&mut request, &mut starts, &mut targets)
            .unwrap()
            .is_none());
        assert!((request.rules.via_size - 0.508).abs() < 1e-6);
        assert!((request.rules.via_drill - 0.3).abs() < 1e-6);
        assert_eq!(request.fab_via_rungs.len(), 1);
        assert!((request.fab_via_rungs[0].0 - 0.508).abs() <= 1e-9);
        assert!((request.fab_via_rungs[0].1 - 0.3).abs() <= 1e-9);
        assert_eq!(request.rules.net_via_sizes.len(), 1);
        let selected = request.rules.net_via_sizes.get(&(net as i32)).unwrap();
        assert!((selected[0] - 0.508).abs() <= 1e-9);
        assert!((selected[1] - 0.3).abs() <= 1e-9);
    }


}
