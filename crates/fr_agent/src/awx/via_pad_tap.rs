//! Source adapter for `single_ended_routing._place_shrunk_via_in_pad_impl`
//! (#189) and `plane_pad_tap.tap_pad_with_escalation`.
//!
//! Candidate/radius ordering, in-pad and escape-cell enumeration, source A*,
//! refit gate, and the uncommitted output are implemented here. [`TapBackend`]
//! supplies only native current-board pairwise distance and grid-coordinate
//! primitives which are not exposed by the public BoardAdapter surface.

use super::{
    braid_connect::RungRequest,
    corridor::Point,
    whole_context::{ContextPad, CopperSegment, CopperVia},
};
use crate::board_adapter::BoardAdapter;
use awx_grid::{GridObstacleMap, TrackMarginArg};
use fr_engine::autoroute::point_router::SourceGridConfig;
use fr_engine::{board::BoardItemType, ids::NetNo};
use fr_geom::{IntBox, IntPoint, Point as EnginePoint, Shape};
use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TapViaCandidate {
    pub diameter_mm: f64,
    pub drill_mm: f64,
    /// Source `escalation_rungs` index > 0; used only for caller diagnostics.
    pub escalated: bool,
}

/// A source via-map cell that the caller must release for the retry, matching
/// the source unblocker's `add_free_via` effect.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TapMapRelease {
    pub cell: (i32, i32),
}

pub struct TapAttemptRequest<'a> {
    pub board: &'a BoardAdapter,
    pub rung: &'a RungRequest,
    pub pad: &'a ContextPad,
    pub pad_layer: &'a str,
    pub candidate: TapViaCandidate,
    pub search_radius_mm: f64,
    pub track_width_mm: f64,
    pub effective_board_edge_clearance_mm: f64,
    pub inflight_vias: &'a [CopperVia],
    pub inflight_segments: &'a [CopperSegment],
    pub via_map: &'a GridObstacleMap,
    pub route_map: &'a GridObstacleMap,
    pub source_config: &'a SourceGridConfig,
    pub native_config: &'a super::native_obstacles::NativeGridConfig,
}

/// Only native pairwise geometry evidence is delegated. Candidate ordering,
/// search, A* routing, refit, and output are owned here.
pub trait TapBackend {
    /// Pairwise exact signed-distance primitive against one current foreign
    /// native copper item, with source-selected layer/class clearance.
    fn foreign_item_distance_mm(
        &mut self,
        board: &BoardAdapter,
        rung: &RungRequest,
        point: Point,
        layer: &str,
        item_index: usize,
    ) -> Result<Option<f64>, String>;
    fn foreign_item_kind(
        &self,
        board: &BoardAdapter,
        rung: &RungRequest,
        item_index: usize,
    ) -> TapForeignKind;
    fn foreign_via_diameter_mm(
        &mut self,
        board: &BoardAdapter,
        rung: &RungRequest,
        item_index: usize,
    ) -> Result<f64, String>;
    /// Number and source kind ordering of current foreign tracks/pads/vias;
    /// distances are returned in that exact order by `foreign_item_distance_mm`.
    fn foreign_item_count(&self, board: &BoardAdapter, rung: &RungRequest, layer: &str) -> usize;
    /// Source per-item route clearance and stack clearance (`_pair_floor` /
    /// `stack_clearance`) in mm, selected for this net and layer.
    fn pair_clearance_mm(&self, rung: &RungRequest, item_index: usize, layer: Option<&str>) -> f64;
    /// Native cell mapping for the request's global `GridCoord`.
    fn grid_cell(
        &self,
        board: &BoardAdapter,
        point: Point,
        grid_step_mm: f64,
    ) -> Result<(i32, i32), String>;
    /// Source signed distance to the current Edge.Cuts outline/cutouts.
    fn board_edge_signed_distance_mm(
        &self,
        board: &BoardAdapter,
        point: Point,
    ) -> Result<f64, String>;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TapForeignKind {
    SegmentOrPad,
    Via,
}

#[derive(Clone, Copy)]
struct FineConfig {
    grid_step_mm: f64,
    clearance_mm: f64,
    track_width_mm: f64,
}

fn fine_tap_configs(request: &TapRequest<'_>, pad: &ContextPad) -> Vec<FineConfig> {
    let grid = request.rung.rules.grid_step.min(0.05);
    if !request.may_narrow {
        return vec![FineConfig {
            grid_step_mm: grid,
            clearance_mm: request.rung.rules.clearance,
            track_width_mm: request.rung.rules.track_width,
        }];
    }
    let fab_track = request.fab_track_floor_mm;
    let track = fab_track.max(
        pad.size[0]
            .min(pad.size[1])
            .min(request.rung.rules.track_width),
    );
    let nominal = request.rung.rules.clearance;
    let clearance_floor = nominal.min(request.fab_clearance_floor_mm);
    let clearances = if nominal <= clearance_floor {
        vec![round_mm(clearance_floor, 4)]
    } else {
        let step = (nominal - clearance_floor) / 4.0;
        (1..=4)
            .map(|i| round_mm(nominal - step * i as f64, 4))
            .collect()
    };
    clearances
        .into_iter()
        .map(|clearance_mm| FineConfig {
            grid_step_mm: grid,
            clearance_mm,
            track_width_mm: track,
        })
        .collect()
}

fn native_dimension(mm: f64, upm: f64) -> Result<i32, String> {
    let value = mm * upm;
    if !value.is_finite() || value < 0.0 || value > i32::MAX as f64 {
        return Err("via-pad fine map dimension is outside native board range".into());
    }
    Ok(value.round() as i32)
}

fn tap_native_config(
    request: &TapRequest<'_>,
    candidate: TapViaCandidate,
    fine: FineConfig,
) -> Result<super::native_obstacles::NativeGridConfig, String> {
    let upm = request.board.units_per_mm;
    let mut config = request.native_config.clone();
    config.grid_step = native_dimension(fine.grid_step_mm, upm)?.max(1);
    config.clearance = native_dimension(fine.clearance_mm, upm)?;
    config.track_width = native_dimension(fine.track_width_mm, upm)?;
    config.via_size = native_dimension(candidate.diameter_mm, upm)?;
    config.via_drill = native_dimension(candidate.drill_mm, upm)?;
    config.board_edge_clearance = native_dimension(request.effective_board_edge_clearance_mm, upm)?;
    config.extra_clearance = 0.0;
    let run = (config.via_size, config.via_drill);
    let small = config
        .fab_via_rungs
        .iter()
        .copied()
        .find(|(diameter, _)| *diameter < run.0);
    let mut ordered = Vec::new();
    if let Some(size) = small.filter(|size| *size != run) {
        ordered.push(size);
    }
    let mut per_net = config
        .net_via_sizes
        .values()
        .copied()
        .filter(|size| *size != run && !ordered.contains(size))
        .collect::<Vec<_>>();
    per_net.sort_by(|a, b| b.0.cmp(&a.0).then_with(|| b.1.cmp(&a.1)));
    per_net.dedup();
    ordered.extend(per_net);
    config.ordered_via_rungs = ordered;
    Ok(config)
}

fn tap_window(
    board: &BoardAdapter,
    pad: &ContextPad,
    radius_mm: f64,
    fine: FineConfig,
    candidate: TapViaCandidate,
) -> Result<IntBox, String> {
    let upm = board.units_per_mm;
    let half_mm = if radius_mm > 0.0 {
        radius_mm + 3.0
    } else {
        pad.size[0].max(pad.size[1]) / 2.0
            + candidate.diameter_mm
            + fine.clearance_mm
            + 2.0 * fine.grid_step_mm
    };
    let center = [pad.point[0] * upm, pad.point[1] * upm];
    let half = half_mm * upm;
    if !half.is_finite() || half < 0.0 || !center[0].is_finite() || !center[1].is_finite() {
        return Err("invalid via-pad fine map window".into());
    }
    let bounds = board.board.bounding_box();
    let x0 = (center[0] - half).floor().max(bounds.ll.x as f64) as i32;
    let y0 = (center[1] - half).floor().max(bounds.ll.y as f64) as i32;
    let x1 = (center[0] + half).ceil().min(bounds.ur.x as f64) as i32;
    let y1 = (center[1] + half).ceil().min(bounds.ur.y as f64) as i32;
    if x0 > x1 || y0 > y1 {
        return Err("via-pad fine map window is outside board bounds".into());
    }
    Ok(IntBox::new(x0, y0, x1, y1))
}

fn clamp_candidate_to_pad(
    request: &TapRequest<'_>,
    pad: &ContextPad,
    candidate: TapViaCandidate,
) -> TapViaCandidate {
    let floors = request
        .fab_candidates
        .iter()
        .map(|floor| super::fab_rules::FabFloor {
            clearance: 0.0,
            track_width: 0.0,
            via_diameter: floor.diameter_mm,
            via_drill: floor.drill_mm,
            hole_to_hole: 0.0,
            pad_hole_to_hole: 0.0,
            annular: (floor.diameter_mm - floor.drill_mm) / 2.0,
            board_edge: 0.0,
        })
        .collect::<Vec<_>>();
    let source_pad = super::escape_moves::Pad {
        key: pad.key.clone(),
        net: pad.net.clone(),
        net_id: pad.net_id,
        component_ref: pad.component_ref.clone(),
        pad_number: pad.pad_number.clone(),
        point: pad.point,
        layers: pad.layers.clone(),
        size: pad.size,
    };
    let clamped = super::fab_rules::clamp_via_to_pad(
        candidate.diameter_mm,
        candidate.drill_mm,
        &source_pad,
        &floors,
        None,
    );
    TapViaCandidate {
        diameter_mm: round_mm(clamped.diameter, 3),
        drill_mm: round_mm(clamped.drill, 3),
        escalated: candidate.escalated || clamped.rung > 0,
    }
}

fn build_tap_maps(
    request: &TapRequest<'_>,
    pad_layer: &str,
    candidate: TapViaCandidate,
    radius_mm: f64,
    fine: FineConfig,
) -> Result<
    (
        GridObstacleMap,
        GridObstacleMap,
        super::native_obstacles::NativeGridConfig,
    ),
    String,
> {
    let config = tap_native_config(request, candidate, fine)?;
    let window = tap_window(request.board, request.pad, radius_mm, fine, candidate)?;
    let via_built = super::native_obstacles::build_obstacle_map_with_config(
        request.board,
        request.rung,
        window,
        &[request.rung.net_id],
        config.clone(),
    )?;
    let mut route_config = config;
    // Source try_tap_pad(routing_clearance_cushion=True): routing maps add a
    // half-cell reserve; the separate via-placement map remains exact.
    route_config.extra_clearance += route_config.grid_step as f64 / 2.0;
    let route_built = super::native_obstacles::build_obstacle_map_on_layer(
        request.board,
        request.rung,
        window,
        &[request.rung.net_id],
        route_config,
        pad_layer,
    )?;
    Ok((
        via_built.obstacles,
        route_built.obstacles,
        route_built.config,
    ))
}

pub struct TapRequest<'a> {
    pub board: &'a BoardAdapter,
    pub rung: &'a RungRequest,
    pub pad: &'a ContextPad,
    /// Maps from the failed outer attempt, retained by the caller for context.
    /// This tap builder never reuses them: every candidate/fine pass builds a
    /// fresh local map using its own via size, grid, clearance and track width.
    pub via_map: &'a GridObstacleMap,
    pub route_map: &'a GridObstacleMap,
    pub source_config: &'a SourceGridConfig,
    pub native_config: &'a super::native_obstacles::NativeGridConfig,
    pub layer_index: usize,
    /// Source-resolved fab floors and escalation gate, passed by the native caller.
    pub fab_clearance_floor_mm: f64,
    pub fab_track_floor_mm: f64,
    pub may_narrow: bool,
    /// Source-resolved `config.rule_floors` + fab-tier escalation candidates,
    /// in the original order (configured via is inserted by this module).
    pub fab_candidates: &'a [TapViaCandidate],
    /// Resolved `KICAD_ESCAPE_STUB_RADIUS`; zero disables the off-pad arm.
    pub escape_radius_mm: f64,
    pub effective_board_edge_clearance_mm: f64,
    /// Current in-flight copper not yet committed to `board`; this is not
    /// reconstructed from a stale snapshot.
    pub inflight_vias: &'a [CopperVia],
    pub inflight_segments: &'a [CopperSegment],
    pub backend: &'a mut dyn TapBackend,
}

#[derive(Clone, Debug)]
pub struct TapResult {
    pub via: CopperVia,
    pub grid_cell: (i32, i32),
    pub pad_layer_index: usize,
    pub stub_segments: Vec<CopperSegment>,
    /// Equivalent of `pcb_data._unblock_via_sizes` for the later path emitter.
    pub registered_via_sizes: BTreeMap<(i32, i32), (f64, f64)>,
    /// Via-map cells to free for the source retry; each is the exact cell whose
    /// physical via and size registration are returned above.
    pub map_releases: Vec<TapMapRelease>,
    pub selected_candidate: TapViaCandidate,
    pub used_search_radius_mm: f64,
}

/// Try the configured via and source fab escalation candidates in source order.
/// The returned via and stubs are never committed to `board`.
pub fn place_shrunk_via_in_pad(request: TapRequest<'_>) -> Result<Option<TapResult>, String> {
    let pad = request.pad;
    if pad.drill > 0.0 || !pad.drill.is_finite() {
        return Ok(None);
    }
    if pad.layers.iter().any(|layer| layer == "*.Cu") {
        return Ok(None);
    }
    let Some((_, pad_layer)) = pad.layers.iter().enumerate().find(|(_, layer)| {
        !layer.starts_with('*') && request.rung.board.board.layer_structure.get_no(layer) >= 0
    }) else {
        return Ok(None);
    };
    let Some(pad_layer_index) = request
        .rung
        .route_layers
        .iter()
        .position(|layer| layer == pad_layer)
    else {
        return Ok(None);
    };
    if request.layer_index != pad_layer_index {
        return Err("selected via-pad layer index disagrees with the rung layer order".into());
    }
    if !request.escape_radius_mm.is_finite()
        || request.escape_radius_mm < 0.0
        || !request.effective_board_edge_clearance_mm.is_finite()
        || request.effective_board_edge_clearance_mm < 0.0
    {
        return Err("invalid via-pad tap radius or board-edge clearance".into());
    }

    // #581 source order: in-pad first unless same-net pad-via clearance is
    // active, followed by the configured off-pad escape radius.
    let allow_in_pad = request.rung.rules.same_net_pad_clearance <= 0.0;
    let mut radii = Vec::with_capacity(2);
    if allow_in_pad {
        radii.push(0.0);
    }
    if request.escape_radius_mm > 0.0 {
        radii.push(request.escape_radius_mm);
    }
    if radii.is_empty() {
        return Ok(None);
    }

    let configured = TapViaCandidate {
        diameter_mm: request.rung.rules.via_size,
        drill_mm: request.rung.rules.via_drill,
        escalated: false,
    };
    let mut candidates = Vec::with_capacity(request.fab_candidates.len() + 1);
    candidates.push(configured);
    candidates.extend_from_slice(request.fab_candidates);
    let mut seen_diameter = Vec::new();
    candidates.retain_mut(|candidate| {
        candidate.diameter_mm = round_mm(candidate.diameter_mm, 3);
        candidate.drill_mm = round_mm(candidate.drill_mm, 3);
        if !candidate.diameter_mm.is_finite()
            || !candidate.drill_mm.is_finite()
            || candidate.drill_mm <= 0.0
            || candidate.drill_mm >= candidate.diameter_mm
            || candidate.diameter_mm > request.rung.rules.via_size + 1e-9
            || seen_diameter
                .iter()
                .any(|diameter: &f64| (*diameter - candidate.diameter_mm).abs() <= 1e-12)
        {
            return false;
        }
        seen_diameter.push(candidate.diameter_mm);
        true
    });

    let mut map_releases = Vec::new();
    let copper_layers = request
        .board
        .board
        .layer_structure
        .layers
        .iter()
        .map(|layer| layer.name.clone())
        .collect::<Vec<_>>();
    if copper_layers.is_empty() {
        return Err("via-pad tap requires explicit board copper layers".into());
    }
    for radius in radii {
        for source_candidate in candidates.iter().copied() {
            let candidate = clamp_candidate_to_pad(&request, pad, source_candidate);
            if candidate.drill_mm <= 0.0
                || candidate.drill_mm >= candidate.diameter_mm
                || candidate.diameter_mm > request.rung.rules.via_size + 1e-9
            {
                continue;
            }
            for fine in fine_tap_configs(&request, pad) {
                let (via_map, route_map, native_config) =
                    build_tap_maps(&request, pad_layer, candidate, radius.min(3.0), fine)?;
                let attempt = TapAttemptRequest {
                    board: request.board,
                    rung: request.rung,
                    pad,
                    pad_layer,
                    candidate,
                    search_radius_mm: radius.min(3.0),
                    track_width_mm: fine.track_width_mm,
                    effective_board_edge_clearance_mm: request.effective_board_edge_clearance_mm,
                    inflight_vias: request.inflight_vias,
                    inflight_segments: request.inflight_segments,
                    via_map: &via_map,
                    route_map: &route_map,
                    source_config: request.source_config,
                    native_config: &native_config,
                };
                let rung = 0; // this candidate is the freshly built map's run via geometry
                let pad_cell =
                    request
                        .backend
                        .grid_cell(request.board, pad.point, fine.grid_step_mm)?;
                let mut positions = Vec::<(f64, Point, (i32, i32))>::new();
                let mut escape_positions = Vec::<(f64, Point, (i32, i32))>::new();
                let center_blocked = via_map.is_via_blocked_rung(pad_cell.0, pad_cell.1, rung);
                if !center_blocked {
                    positions.push((0.0, pad.point, pad_cell));
                }
                // Source `find_via_position`: inspect grid-cell ring sites within
                // the actual pad copper before considering the escape radius.
                let pad_half = [pad.size[0] / 2.0, pad.size[1] / 2.0];
                let cell_step = fine.grid_step_mm;
                let rx = (pad_half[0] / cell_step).ceil().max(1.0) as i32;
                let ry = (pad_half[1] / cell_step).ceil().max(1.0) as i32;
                for ring in 1..=rx.max(ry) {
                    for dx in -ring..=ring {
                        for dy in -ring..=ring {
                            if dx.abs() != ring && dy.abs() != ring
                                || dx.abs() > rx
                                || dy.abs() > ry
                            {
                                continue;
                            }
                            let cell =
                                (pad_cell.0.saturating_add(dx), pad_cell.1.saturating_add(dy));
                            let point = [cell.0 as f64 * cell_step, cell.1 as f64 * cell_step];
                            if !pad_contains(pad, pad_layer, point) {
                                continue;
                            }
                            if !via_map.is_via_blocked_rung(cell.0, cell.1, rung) {
                                let distance2 = squared_distance(pad.point, point);
                                positions.push((distance2, point, cell));
                            }
                        }
                    }
                }
                if attempt.search_radius_mm > 0.0 {
                    let radius_cells = (attempt.search_radius_mm / cell_step).ceil() as i32;
                    for ring in 1..=radius_cells {
                        for dx in -ring..=ring {
                            for dy in -ring..=ring {
                                if dx.abs() != ring && dy.abs() != ring {
                                    continue;
                                }
                                let cell =
                                    (pad_cell.0.saturating_add(dx), pad_cell.1.saturating_add(dy));
                                let point = [cell.0 as f64 * cell_step, cell.1 as f64 * cell_step];
                                if squared_distance(pad.point, point)
                                    > attempt.search_radius_mm * attempt.search_radius_mm + 1e-12
                                    || via_map.is_via_blocked_rung(cell.0, cell.1, rung)
                                {
                                    continue;
                                }
                                escape_positions.push((
                                    squared_distance(pad.point, point),
                                    point,
                                    cell,
                                ));
                            }
                        }
                    }
                }
                positions.sort_by(|a, b| a.0.total_cmp(&b.0));
                positions.dedup_by_key(|entry| entry.2);
                escape_positions.sort_by(|a, b| a.0.total_cmp(&b.0));
                escape_positions.dedup_by_key(|entry| entry.2);

                let mut selected = None;
                for (_, point, cell) in positions {
                    if inflight_blocks(
                        point,
                        candidate,
                        request.rung.net_id as i32,
                        fine.clearance_mm,
                        request.inflight_vias,
                        request.inflight_segments,
                    ) {
                        continue;
                    }
                    if !refit_all_layers(
                        request.backend,
                        request.board,
                        request.rung,
                        point,
                        candidate,
                        &copper_layers,
                    )? {
                        continue;
                    }
                    selected = Some((point, cell, Vec::new()));
                    break;
                }
                if selected.is_none() && !escape_positions.is_empty() {
                    let mut escape = Vec::new();
                    for (_, point, cell) in escape_positions {
                        if inflight_blocks(
                            point,
                            candidate,
                            request.rung.net_id as i32,
                            fine.clearance_mm,
                            request.inflight_vias,
                            request.inflight_segments,
                        ) || !refit_all_layers(
                            request.backend,
                            request.board,
                            request.rung,
                            point,
                            candidate,
                            &copper_layers,
                        )? {
                            continue;
                        }
                        escape.push(cell);
                    }
                    if let Some((cell, segments)) =
                        route_escape_candidates(&attempt, pad_cell, &escape)?
                    {
                        let point = [
                            cell.0 as f64 * fine.grid_step_mm,
                            cell.1 as f64 * fine.grid_step_mm,
                        ];
                        selected = Some((point, cell, segments));
                    }
                }
                let Some((point, _cell, segments)) = selected else {
                    continue;
                };
                let (Some(first), Some(last)) = (copper_layers.first(), copper_layers.last())
                else {
                    return Err("via-pad tap requires explicit board copper layers".into());
                };
                let via = CopperVia {
                    net: request.rung.net.clone(),
                    net_id: request.rung.net_id as i32,
                    point,
                    diameter: candidate.diameter_mm,
                    drill: candidate.drill_mm,
                    layers: vec![first.clone(), last.clone()],
                };
                // Return/registration cells are consumed by the outer rung path,
                // whose coordinate transform is the original (coarser) grid.
                let output_cell = request.backend.grid_cell(
                    request.board,
                    point,
                    request.rung.rules.grid_step,
                )?;
                let mut registered_via_sizes = BTreeMap::new();
                registered_via_sizes
                    .insert(output_cell, (candidate.diameter_mm, candidate.drill_mm));
                map_releases.push(TapMapRelease { cell: output_cell });
                return Ok(Some(TapResult {
                    via,
                    grid_cell: output_cell,
                    pad_layer_index,
                    stub_segments: segments,
                    registered_via_sizes,
                    map_releases: std::mem::take(&mut map_releases),
                    selected_candidate: candidate,
                    used_search_radius_mm: attempt.search_radius_mm,
                }));
            }
        }
    }
    Ok(None)
}

fn round_mm(value: f64, digits: i32) -> f64 {
    let scale = 10_f64.powi(digits);
    (value * scale).round_ties_even() / scale
}

fn squared_distance(a: Point, b: Point) -> f64 {
    (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)
}

fn point_segment_distance(point: Point, a: Point, b: Point) -> f64 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let length2 = dx * dx + dy * dy;
    if length2 <= 1e-24 {
        return squared_distance(point, a).sqrt();
    }
    let t = (((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length2).clamp(0.0, 1.0);
    squared_distance(point, [a[0] + t * dx, a[1] + t * dy]).sqrt()
}

fn inflight_blocks(
    point: Point,
    candidate: TapViaCandidate,
    own_net: i32,
    clearance: f64,
    vias: &[CopperVia],
    segments: &[CopperSegment],
) -> bool {
    vias.iter().filter(|via| via.net_id != own_net).any(|via| {
        squared_distance(point, via.point).sqrt()
            < candidate.diameter_mm / 2.0 + via.diameter / 2.0 + clearance
    }) || segments
        .iter()
        .filter(|segment| segment.net_id != own_net)
        .any(|segment| {
            point_segment_distance(point, segment.a, segment.b)
                < candidate.diameter_mm / 2.0 + segment.width / 2.0 + clearance
        })
}

fn pad_contains(pad: &ContextPad, layer: &str, point: Point) -> bool {
    let shapes: Vec<_> = pad
        .copper_shapes
        .iter()
        .filter(|shape| shape.layer == layer)
        .collect();
    !shapes.is_empty()
        && shapes
            .iter()
            .any(|shape| shape.shape.signed_distance(point) <= 1e-9)
}

fn route_escape_candidates(
    attempt: &TapAttemptRequest<'_>,
    pad_cell: (i32, i32),
    candidates: &[(i32, i32)],
) -> Result<Option<((i32, i32), Vec<CopperSegment>)>, String> {
    if candidates.is_empty() {
        return Ok(None);
    }
    let Some(full_layer_index) = attempt
        .rung
        .route_layers
        .iter()
        .position(|layer| layer == attempt.pad_layer)
    else {
        return Ok(None);
    };
    if attempt.native_config.route_layers.len() != 1
        || attempt
            .native_config
            .route_layers
            .first()
            .map(String::as_str)
            != Some(attempt.pad_layer)
    {
        return Err("tap routing map is not aligned to the selected pad layer".into());
    }
    // Source build_routing_obstacle_map is a one-layer map; its selected copper
    // layer is therefore always native search layer 0.
    let layer = 0u8;
    let mut local_config = attempt.source_config.clone();
    let layer_cost = local_config
        .layer_costs
        .get(full_layer_index)
        .copied()
        .unwrap_or(1000);
    let direction = local_config
        .layer_direction_preferences
        .get(full_layer_index)
        .copied()
        .unwrap_or(255);
    local_config.layer_costs = vec![layer_cost];
    local_config.layer_direction_preferences = vec![direction];
    let router = local_config.router();
    let sources = candidates.iter().map(|&(gx, gy)| (gx, gy, layer)).collect();
    let (path, _, _) = fr_engine::autoroute::point_router::route_with_frontier(
        &router,
        attempt.route_map,
        sources,
        vec![(pad_cell.0, pad_cell.1, layer)],
        attempt.rung.max_iterations.min(u32::MAX as usize) as u32,
        TrackMarginArg::Scalar(0.0),
        None,
        None,
    );
    let Some(path) = path else { return Ok(None) };
    if path.is_empty() {
        return Ok(None);
    }
    let step_mm = attempt.native_config.grid_step as f64 / attempt.board.units_per_mm;
    if !step_mm.is_finite() || step_mm <= 0.0 {
        return Err("via-pad tap route grid scale is invalid".into());
    }
    let mut points = Vec::with_capacity(path.len() + 2);
    let Some(&(via_x, via_y, _)) = path.first() else {
        return Ok(None);
    };
    let via_cell = (via_x, via_y);
    points.push([via_x as f64 * step_mm, via_y as f64 * step_mm]);
    points.extend(
        path.iter()
            .skip(1)
            .map(|&(gx, gy, _)| [gx as f64 * step_mm, gy as f64 * step_mm]),
    );
    points.push(attempt.pad.point);
    let mut segments = Vec::new();
    for pair in points.windows(2) {
        if squared_distance(pair[0], pair[1]) <= 1e-16 {
            continue;
        }
        segments.push(CopperSegment {
            net: attempt.rung.net.clone(),
            net_id: attempt.rung.net_id as i32,
            a: pair[0],
            b: pair[1],
            width: attempt.track_width_mm,
            layer: attempt.pad_layer.to_owned(),
        });
    }
    Ok((!segments.is_empty()).then_some((via_cell, segments)))
}

fn refit_all_layers(
    backend: &mut dyn TapBackend,
    board: &BoardAdapter,
    rung: &RungRequest,
    point: Point,
    candidate: TapViaCandidate,
    copper_layers: &[String],
) -> Result<bool, String> {
    const EPS_MM: f64 = 0.05;
    for layer in copper_layers {
        let count = backend.foreign_item_count(board, rung, layer);
        for index in 0..count {
            let Some(distance) =
                backend.foreign_item_distance_mm(board, rung, point, layer, index)?
            else {
                continue;
            };
            let clearance = backend.pair_clearance_mm(rung, index, Some(layer));
            let required = match backend.foreign_item_kind(board, rung, index) {
                TapForeignKind::SegmentOrPad => candidate.diameter_mm / 2.0 + clearance - EPS_MM,
                TapForeignKind::Via => {
                    // For via items the primitive distance is center-to-center;
                    // native geometry supplies the foreign barrel diameter.
                    let foreign_diameter = backend.foreign_via_diameter_mm(board, rung, index)?;
                    candidate.diameter_mm / 2.0 + foreign_diameter / 2.0 + clearance - EPS_MM
                }
            };
            if distance < required {
                return Ok(false);
            }
        }
    }
    let stack_clearance = backend.pair_clearance_mm(rung, usize::MAX, None);
    for via in rung
        .existing_vias
        .iter()
        .filter(|via| via.net_id != rung.net_id as i32)
    {
        let separation = (squared_distance(point, via.point)).sqrt();
        if separation < candidate.diameter_mm / 2.0 + via.diameter / 2.0 + stack_clearance - EPS_MM
        {
            return Ok(false);
        }
    }
    if rung.rules.hole_to_hole_clearance > 0.0 {
        for via in rung
            .existing_vias
            .iter()
            .filter(|via| via.net_id == rung.net_id as i32)
        {
            if squared_distance(point, via.point).sqrt()
                < candidate.drill_mm / 2.0 + via.drill / 2.0 + rung.rules.hole_to_hole_clearance
            {
                return Ok(false);
            }
        }
    }
    Ok(true)
}
