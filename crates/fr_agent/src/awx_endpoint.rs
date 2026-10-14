//! Ordinary DSN airlines into the ported AWX GridRouter, with native transactions.
use crate::{
    awx::{
        braid_connect::{rung_specs, LadderMode, RungEndpoint, RungRequest},
        fab_rules::{EscalationPolicy, FabTier},
        native_rung::{endpoint_grid_calls, route_multilayer_endpoint_sets},
        whole_context::RoutingRules,
    },
    board_adapter::BoardAdapter,
};
use fr_engine::{
    board::{ItemKey, ItemKind, ItemSet},
    drc::NetIncompletes,
    ids::NetNo,
};
use fr_geom::Point;
use serde::Deserialize;
use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    io::Write,
    path::Path,
    time::{Duration, Instant},
};

fn endpoint_pad_metadata(
    board: &BoardAdapter,
    require_hole_metadata: bool,
) -> Result<
    (
        Vec<crate::awx::native_grid_window::FreeViaPad>,
        Vec<crate::awx::whole_context::ContextPad>,
    ),
    String,
> {
    use crate::awx::whole_context::{ComponentKind, ContextPad};
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
        for layer in item.first_layer(&board.board)..=item.last_layer(&board.board) {
            if let Some(shape) = item.drill_shape_on_layer(&board.board, layer) {
                let bounds = shape.bounding_box();
                size[0] = size[0].max((bounds.ur.x - bounds.ll.x) as f64 / board.units_per_mm);
                size[1] = size[1].max((bounds.ur.y - bounds.ll.y) as f64 / board.units_per_mm);
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
            pad_type: if drill > 0.0 { "thru_hole" } else { "smd" }.into(),
            shape: String::new(),
            rect_rotation: 0.0,
            copper_shapes: Vec::new(),
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

fn log_event(log: &mut fs::File, event: &serde_json::Value) -> Result<(), String> {
    writeln!(log, "{event}").map_err(|e| e.to_string())?;
    log.flush().map_err(|e| e.to_string())
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

pub fn run(path: &Path) -> Result<(), String> {
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
    let input = fs::read(&job.input).map_err(|e| e.to_string())?;
    let mut board = BoardAdapter::load(&input)?;
    let (free_via_pads, same_net_pads) =
        endpoint_pad_metadata(&board, job.hole_to_hole_clearance_mm > 0.0)?;
    let baseline = board.clone();
    let before = fr_engine::pipeline::stats::incomplete_count(&board.board, None);
    let started = Instant::now();
    let deadline = started + Duration::from_millis(job.budget_ms);
    let mut log = fs::File::create(&job.log).map_err(|e| e.to_string())?;
    let mut attempts = Vec::new();
    let mut skips = Vec::new();
    let mut accepted = 0;
    let nets: Vec<_> = board
        .board
        .rules
        .nets
        .iter()
        .map(|n| (n.net_number, n.name.clone()))
        .collect();
    for (net, name) in nets {
        let initial =
            NetIncompletes::new(&board.board, net, &board.board.get_connectable_items(net));
        if initial.count() == 0 {
            continue;
        }
        if job.protected_nets.contains(&name) {
            skips.push(serde_json::json!({"net":name,"reason":"protected"}));
            continue;
        }
        // Keep the mature native Delaunay/Kruskal airline order. Refresh after
        // every accepted transaction so endpoint groups reflect the new board.
        loop {
            if Instant::now() >= deadline {
                skips.push(serde_json::json!({"net":name,"reason":"deadline"}));
                break;
            }
            let current =
                NetIncompletes::new(&board.board, net, &board.board.get_connectable_items(net));
            if current.count() == 0 {
                break;
            }
            let mut committed = false;
            // Suppress duplicate airline edges during this board revision.
            // A successful commit starts a fresh pass with a fresh set.
            let mut tried = BTreeSet::new();
            for edge in current.incompletes {
                let from = board.board.item(edge.from_item);
                let to = board.board.item(edge.to_item);
                let from_id = from.id().0;
                let to_id = to.id().0;
                let route_layers: Vec<_> = board
                    .board
                    .layer_structure
                    .layers
                    .iter()
                    .enumerate()
                    .filter(|(i, l)| {
                        l.is_signal
                            && board
                                .board
                                .rules
                                .net_class_of(net)
                                .is_active_routing_layer(*i as i32)
                    })
                    .map(|(i, l)| (i as i32, l.name.clone()))
                    .collect();
                if route_layers.is_empty() {
                    continue;
                }
                // The airline's two items stand for two native connected
                // groups. Build the candidate sets from every eligible
                // ORIGINAL terminal in those groups, not from the single
                // airline corner replicated on each layer.
                let from_group = board.board.connected_set(edge.from_item, net, false);
                let to_group = board.board.connected_set(edge.to_item, net, false);
                let start_candidates =
                    connected_group_candidates(&board, &from_group, &route_layers);
                let target_candidates =
                    connected_group_candidates(&board, &to_group, &route_layers);
                if start_candidates.is_empty() || target_candidates.is_empty() {
                    continue;
                }
                let start_point = start_candidates[0].point;
                let end_point = target_candidates[0].point;
                // Suppress repeat work on the same ordered group pair during
                // this board revision using stable native group identities
                // (their member item ids), never the per-airline corner.
                // The set is rebuilt after every acceptance below.
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
                if Instant::now() >= deadline {
                    break;
                }
                // Rule selection keeps its old contract: the first route
                // layer the original from-item sits on. Candidates from
                // other members of the connected group can also start the
                // route, so fall back to the first selected candidate's layer
                // when the representative item is off the active stack
                // (uniform benchmark rules make these equivalent).
                let rule_layer = route_layers
                    .iter()
                    .find(|(i, _)| from.is_on_layer(&board.board, *i))
                    .map(|(i, _)| *i)
                    .or_else(|| {
                        route_layers
                            .iter()
                            .find(|(_, name)| *name == start_candidates[0].layer)
                            .map(|(i, _)| *i)
                    })
                    .expect("start candidate set is non-empty");
                let rules = rules_for_layer(
                    &board,
                    net,
                    rule_layer,
                    job.grid_step_mm,
                    job.via_drill_mm,
                    job.min_annular_ring_mm,
                    job.hole_to_hole_clearance_mm,
                )?;
                let mut event = serde_json::json!({"event":"attempt_start","net":name,
                    "from_item":from_id,"to_item":to_id,"start":start_point,"end":end_point,
                    "start_layers":start_candidates.iter().map(|e| e.layer.clone()).collect::<Vec<_>>(),
                    "end_layers":target_candidates.iter().map(|e| e.layer.clone()).collect::<Vec<_>>(),
                    "scheduler":"native NetIncompletes Delaunay/Kruskal airlines",
                    "width_mm":rules.track_width,"clearance_mm":rules.clearance,
                    "algorithm":"awx_grid::GridRouter","new_vias_allowed":true,
                    "new_via_drill_source":"explicit job rule",
                    "existing_pad_drill_source":"native explicit padstack metadata",
                    "via_drill_mm":rules.via_drill});
                log_event(&mut log, &event)?;
                let request = RungRequest {
                    board: board.clone(),
                    net: name.clone(),
                    net_id: net,
                    stage: "ordinary_dsn_multilayer".into(),
                    start: start_candidates[0].clone(),
                    end: target_candidates[0].clone(),
                    b_alternatives: Vec::new(),
                    pair: None,
                    spec: rung_specs(LadderMode::Open)[2],
                    band: None,
                    band_slack: 0.0,
                    window_points: None,
                    virtual_tracks: Vec::new(),
                    virtual_vias: Vec::new(),
                    free_via_pads: free_via_pads.clone(),
                    same_net_pads: same_net_pads.clone(),
                    existing_vias: crate::awx::board_snapshot::BoardSnapshot::capture_vias(
                        &board,
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
                    max_iterations: job.max_iterations,
                    route_layers: route_layers.iter().map(|(_, n)| n.clone()).collect(),
                    deadline: Some(deadline),
                };
                let calls_before = endpoint_grid_calls();
                let (candidate, mut report) = match route_multilayer_endpoint_sets(
                    request,
                    edge.from_item,
                    edge.to_item,
                    &start_candidates,
                    &target_candidates,
                ) {
                    Ok(v) => v,
                    Err(e) => (None, serde_json::json!({"accepted":false,"reason":e})),
                };
                report["grid_router_calls"] = (endpoint_grid_calls() - calls_before).into();
                event["event"] = "attempt_end".into();
                event["result"] = report;
                log_event(&mut log, &event)?;
                attempts.push(event);
                if let Some(candidate) = candidate {
                    board = candidate;
                    accepted += 1;
                    committed = true;
                    break;
                }
            }
            if !committed {
                break;
            }
        }
    }
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
    let replay =
        BoardAdapter::load_session_with_via_candidates(&input, &session, &replay_candidates)?;
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

    let report = serde_json::json!({
        "actual_solver":"awx_grid::GridRouter","scope":"all native incompletes; active signal stack and native via transitions",
        "scheduler":"native NetIncompletes Delaunay/Kruskal airlines; recomputed after each accepted transaction",
        "whole_bga_bus_stages":"not applicable: flattened component identity",
        "budget_ms":job.budget_ms,"requested_threads":job.threads,"effective_threads":1,
        "parallel_policy":"serial board transactions","wall_seconds":started.elapsed().as_secs_f64(),
        "before_incompletes":before,"after_incompletes":fr_engine::pipeline::stats::incomplete_count(&board.board,None),
        "session_replay_incompletes":fr_engine::pipeline::stats::incomplete_count(&replay.board,None),
        "session_replay_vias":replay_vias.len(), "session_replay_via_geometry":replay_vias,
        "session_replay_new_violations":session_replay_new_violations,
        "accepted":accepted,"grid_router_calls":attempts.iter().filter_map(|v|v["result"]["grid_router_calls"].as_u64()).sum::<u64>(),
        "new_vias":board.board.get_vias().len().saturating_sub(baseline.board.get_vias().len()),"new_violations":board.violations().difference(&baseline.violations()).count(),
        "attempts":attempts,"skipped":skips,"output":job.output
    });
    let serialized_report = serde_json::to_string_pretty(&report).map_err(|e| e.to_string())?;
    write_atomic(&output_path, &session)?;
    println!("{}", serialized_report);
    Ok(())
}
