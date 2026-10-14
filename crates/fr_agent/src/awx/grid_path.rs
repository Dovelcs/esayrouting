//! Convert source GridRouter grid paths into physical copper geometry.
//!
//! This module does not search for routes. It mirrors the geometric contract
//! of `single_ended_routing._path_to_segments_vias` for an already selected
//! path and returns board-mm primitives for the native rung materializer.

use crate::board_adapter::BoardAdapter;
use fr_engine::{ids::NetNo, rules::via::ViaInfoId};
use fr_geom::{FloatPoint, Shape};
use std::collections::{BTreeMap, BTreeSet};

use super::{braid_connect::RungEndpoint, corridor::Point, whole_context::CopperVia};

const TERMINAL_EPS_MM: f64 = 0.001;
const TERMINAL_MERGE_EPS_MM: f64 = 1e-6;
const PAD_SAMPLE_STEP_MM: f64 = 0.02;
const VIA_MATCH_TOL_MM: f64 = 0.001;
const UNBLOCK_REFIT_MARGIN_MM: f64 = 0.05;

pub struct GridPathRequest<'a> {
    /// Global GridRouter cells and indices into `route_layers`.
    pub path: &'a [(i32, i32, u8)],
    /// Integer board units per grid cell; the source grid origin is `(0,0)`.
    pub grid_step_board: i32,
    /// Exact GridRouteConfig layer order, not the board's full layer list.
    pub route_layers: &'a [String],
    /// Exact source and selected-target terminals in board millimetres.
    pub start: &'a RungEndpoint,
    pub target: &'a RungEndpoint,
    pub board: &'a BoardAdapter,
    pub net: NetNo,
    /// Ordered via-info candidates available to this net's routing rule.
    pub via_candidates: &'a [ViaInfoId],
    pub via_size_mm: f64,
    pub via_drill_mm: f64,
    pub hole_to_hole_clearance_mm: f64,
    /// Ordered source `escalation_rungs` candidates, excluding the configured via.
    pub via_refit_rungs: &'a [(f64, f64)],
    /// Source `pcb_data._unblock_via_sizes`: grid cell -> registered (diameter, drill) mm.
    pub registered_via_sizes: &'a BTreeMap<(i32, i32), (f64, f64)>,
    /// Existing vias with explicit physical dimensions and identity from the board snapshot.
    pub existing_vias: &'a [CopperVia],
    /// Same-net through-hole pad centers and via centers projected to global grid cells.
    pub through_hole_positions: &'a BTreeSet<(i32, i32)>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct GridPathSegment {
    pub start_mm: Point,
    pub end_mm: Point,
    pub layer: String,
    pub width_mm: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct GridPathVia {
    pub point_mm: Point,
    pub diameter_mm: f64,
    pub drill_mm: f64,
    /// The real native through-via definition selected from the board's net rule.
    pub via_info: ViaInfoId,
    /// Adjacent route layers at this path transition. `via_info` gives the physical span.
    pub from_layer: String,
    pub to_layer: String,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct GridPathGeometry {
    pub segments: Vec<GridPathSegment>,
    pub vias: Vec<GridPathVia>,
}

/// Convert the already-routed grid cells into traces/vias and exact terminal
/// connectors. A layer transition at a reusable hole emits no duplicate via.
pub fn path_to_geometry(request: GridPathRequest<'_>) -> Result<GridPathGeometry, String> {
    if request.path.is_empty() {
        return Ok(GridPathGeometry::default());
    }
    if request.grid_step_board <= 0 {
        return Err("grid path step must be positive".into());
    }
    if !request.board.units_per_mm.is_finite() || request.board.units_per_mm <= 0.0 {
        return Err("grid path board scale is invalid".into());
    }
    if !request.via_size_mm.is_finite()
        || !request.via_drill_mm.is_finite()
        || !request.hole_to_hole_clearance_mm.is_finite()
        || request.via_size_mm <= 0.0
        || request.via_drill_mm <= 0.0
        || request.via_drill_mm >= request.via_size_mm
        || request.hole_to_hole_clearance_mm < 0.0
    {
        return Err("grid path via dimensions or hole clearance are invalid".into());
    }

    let path = simplify_path(request.path);
    let mut points = path
        .iter()
        .map(|&(gx, gy, _)| grid_to_mm(gx, gy, request.grid_step_board, request.board.units_per_mm))
        .collect::<Result<Vec<_>, _>>()?;
    let first_layer = route_layer(&request, path[0].2)?;
    let last_layer = route_layer(&request, path[path.len() - 1].2)?;
    let merge_start = path.len() >= 2
        && merge_terminal_to_exact(&request, &path, &mut points, 0, 1, request.start)?;
    let merge_end = path.len() >= 2
        && merge_terminal_to_exact(
            &request,
            &path,
            &mut points,
            path.len() - 1,
            path.len() - 2,
            request.target,
        )?;
    let mut geometry = GridPathGeometry::default();
    if !merge_start
        && differs(request.start.point, points[0])
        && terminal_copper_on_layer(
            &request,
            request.start.point,
            &request.start.layer,
            &first_layer,
        )?
    {
        geometry.segments.push(GridPathSegment {
            start_mm: request.start.point,
            end_mm: points[0],
            layer: first_layer.to_owned(),
            width_mm: trace_width_mm(&request, &first_layer)?,
        });
    }

    for index in 0..path.len().saturating_sub(1) {
        let (gx, gy, layer_a) = path[index];
        let (_, _, layer_b) = path[index + 1];
        let a = points[index];
        let b = points[index + 1];
        if layer_a != layer_b {
            if request.through_hole_positions.contains(&(gx, gy)) {
                continue;
            }
            let (requested_diameter, requested_drill) = request
                .registered_via_sizes
                .get(&(gx, gy))
                .copied()
                .unwrap_or((request.via_size_mm, request.via_drill_mm));
            if !requested_diameter.is_finite()
                || !requested_drill.is_finite()
                || requested_diameter <= 0.0
                || requested_drill <= 0.0
                || requested_drill >= requested_diameter
            {
                return Err(format!("invalid registered via size at ({gx},{gy})"));
            }
            let (via_info, diameter_mm, drill_mm, from_board, to_board) =
                select_refit_via(&request, a, requested_diameter, requested_drill)?;
            if reusable_same_net_via(&request, a, drill_mm)? {
                continue;
            }
            geometry.vias.push(GridPathVia {
                point_mm: a,
                diameter_mm,
                drill_mm,
                via_info,
                from_layer: route_layer(&request, layer_a)?.to_owned(),
                to_layer: route_layer(&request, layer_b)?.to_owned(),
            });
            // Access the physical span here as well: candidate selection only
            // accepts full board through padstacks, so no adjacent-layer via is
            // silently substituted for the source's through via.
            let _ = (from_board, to_board);
        } else if a != b {
            let layer = route_layer(&request, layer_a)?;
            geometry.segments.push(GridPathSegment {
                start_mm: a,
                end_mm: b,
                layer: layer.to_owned(),
                width_mm: trace_width_mm(&request, layer)?,
            });
        }
    }

    if !merge_end
        && differs(request.target.point, *points.last().unwrap())
        && terminal_copper_on_layer(
            &request,
            request.target.point,
            &request.target.layer,
            &last_layer,
        )?
    {
        geometry.segments.push(GridPathSegment {
            start_mm: *points.last().unwrap(),
            end_mm: request.target.point,
            layer: last_layer.to_owned(),
            width_mm: trace_width_mm(&request, route_layer(&request, path.last().unwrap().2)?)?,
        });
    }
    Ok(geometry)
}

fn simplify_path(path: &[(i32, i32, u8)]) -> Vec<(i32, i32, u8)> {
    if path.len() <= 2 {
        return path.to_vec();
    }
    let mut out = Vec::with_capacity(path.len());
    out.push(path[0]);
    for index in 1..path.len() - 1 {
        let previous = path[index - 1];
        let current = path[index];
        let next = path[index + 1];
        if previous.2 != current.2 || current.2 != next.2 {
            out.push(current);
            continue;
        }
        let dx1 = (current.0 - previous.0) as f64;
        let dy1 = (current.1 - previous.1) as f64;
        let dx2 = (next.0 - current.0) as f64;
        let dy2 = (next.1 - current.1) as f64;
        let len1 = (dx1 * dx1 + dy1 * dy1).sqrt().max(1.0);
        let len2 = (dx2 * dx2 + dy2 * dy2).sqrt().max(1.0);
        let differs =
            (dx1 / len1 - dx2 / len2).abs() > 0.01 || (dy1 / len1 - dy2 / len2).abs() > 0.01;
        if differs {
            out.push(current);
        }
    }
    out.push(*path.last().unwrap());
    out
}

fn grid_to_mm(gx: i32, gy: i32, step: i32, units_per_mm: f64) -> Result<Point, String> {
    let x = (gx as i64)
        .checked_mul(step as i64)
        .ok_or("grid path x coordinate overflow")?;
    let y = (gy as i64)
        .checked_mul(step as i64)
        .ok_or("grid path y coordinate overflow")?;
    Ok([x as f64 / units_per_mm, y as f64 / units_per_mm])
}

fn route_layer<'a>(request: &'a GridPathRequest<'_>, index: u8) -> Result<&'a str, String> {
    let name = request
        .route_layers
        .get(index as usize)
        .ok_or_else(|| format!("grid path layer index {index} is outside route_layers"))?;
    let layer_no = request.board.board.layer_structure.get_no(name);
    if layer_no < 0 || !request.board.board.layer_structure.layers[layer_no as usize].is_signal {
        return Err(format!("grid path references non-copper layer {name}"));
    }
    Ok(name)
}

fn trace_width_mm(request: &GridPathRequest<'_>, layer: &str) -> Result<f64, String> {
    let layer_no = request.board.board.layer_structure.get_no(layer);
    if layer_no < 0 {
        return Err(format!("grid path layer is absent from board: {layer}"));
    }
    let width = 2.0
        * request
            .board
            .board
            .rules
            .get_trace_half_width(request.net, layer_no) as f64
        / request.board.units_per_mm;
    if !width.is_finite() || width <= 0.0 {
        return Err(format!(
            "grid path has invalid native trace width on {layer}"
        ));
    }
    Ok(width)
}

fn differs(a: Point, b: Point) -> bool {
    (a[0] - b[0]).abs() > TERMINAL_EPS_MM || (a[1] - b[1]).abs() > TERMINAL_EPS_MM
}

fn distance(a: Point, b: Point) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

fn merge_terminal_to_exact(
    request: &GridPathRequest<'_>,
    path: &[(i32, i32, u8)],
    points: &mut [Point],
    terminal_index: usize,
    neighbor_index: usize,
    terminal: &RungEndpoint,
) -> Result<bool, String> {
    let terminal_cell = path[terminal_index];
    let neighbor_cell = path[neighbor_index];
    if terminal_cell.2 != neighbor_cell.2 {
        return Ok(false);
    }
    let layer = route_layer(request, terminal_cell.2)?;
    if terminal.layer != layer {
        return Ok(false);
    }
    let grid_point = points[terminal_index];
    if (terminal.point[0] - grid_point[0]).abs() < 1e-9
        && (terminal.point[1] - grid_point[1]).abs() < 1e-9
    {
        return Ok(false);
    }
    let layer_no = request.board.board.layer_structure.get_no(layer);
    let own_class = request
        .board
        .board
        .rules
        .net_class_of(request.net)
        .get_trace_clearance_class();
    let base_clearance = request
        .board
        .board
        .rules
        .clearance_matrix
        .get_value(own_class, own_class, layer_no, false) as f64;
    let half_width = (trace_width_mm(request, layer)? * request.board.units_per_mm) / 2.0;
    let margin = base_clearance + half_width;
    if foreign_pad_distance(request, grid_point, layer_no, base_clearance)? >= margin {
        return Ok(false);
    }
    if foreign_pad_distance(request, terminal.point, layer_no, base_clearance)? < margin {
        return Ok(false);
    }
    let neighbor = points[neighbor_index];
    let cell_limit = 1.5 * request.grid_step_board as f64 / request.board.units_per_mm;
    if distance(grid_point, neighbor) > cell_limit {
        return Ok(false);
    }
    if segment_foreign_pad_distance(request, terminal.point, neighbor, layer_no, base_clearance)?
        < margin - TERMINAL_MERGE_EPS_MM * request.board.units_per_mm
    {
        return Ok(false);
    }
    points[terminal_index] = terminal.point;
    Ok(true)
}

/// Minimum edge distance to foreign copper pads, adjusted by pairwise class
/// excess over the moving net's self-clearance, matching source `_pt_foreign_pad_dist`.
fn foreign_pad_distance(
    request: &GridPathRequest<'_>,
    point_mm: Point,
    layer: i32,
    base_clearance_board: f64,
) -> Result<f64, String> {
    let rules = &request.board.board.rules;
    let own_class = rules.net_class_of(request.net).get_trace_clearance_class();
    let point = FloatPoint::new(
        point_mm[0] * request.board.units_per_mm,
        point_mm[1] * request.board.units_per_mm,
    );
    let mut best = f64::INFINITY;
    for key in request.board.board.get_pins() {
        let item = request.board.board.item(key);
        if item.net_numbers().is_empty()
            || item.contains_net(request.net)
            || !item.is_on_layer(&request.board.board, layer)
        {
            continue;
        }
        let Some(shape) = item.drill_shape_on_layer(&request.board.board, layer) else {
            continue;
        };
        let pair_clearance =
            rules
                .clearance_matrix
                .get_value(own_class, item.clearance_class(), layer, false) as f64;
        let excess = (pair_clearance - base_clearance_board).max(0.0);
        best = best.min(shape_edge_distance(&shape, point) - excess);
    }
    Ok(best)
}

/// Source samples a short terminal segment every 20 um (including both ends).
fn segment_foreign_pad_distance(
    request: &GridPathRequest<'_>,
    start_mm: Point,
    end_mm: Point,
    layer: i32,
    base_clearance_board: f64,
) -> Result<f64, String> {
    let length_mm = distance(start_mm, end_mm);
    let intervals = ((length_mm / PAD_SAMPLE_STEP_MM).trunc() as usize + 1).max(2);
    let mut best = f64::INFINITY;
    for index in 0..=intervals {
        let t = index as f64 / intervals as f64;
        let point = [
            start_mm[0] + (end_mm[0] - start_mm[0]) * t,
            start_mm[1] + (end_mm[1] - start_mm[1]) * t,
        ];
        best = best.min(foreign_pad_distance(
            request,
            point,
            layer,
            base_clearance_board,
        )?);
    }
    Ok(best)
}

fn shape_edge_distance(shape: &Shape, point: FloatPoint) -> f64 {
    if shape.contains_float(&point) {
        return 0.0;
    }
    match shape {
        Shape::Polygon(_) => {
            let corners = shape.corner_approx_arr();
            if corners.len() < 2 {
                return f64::INFINITY;
            }
            let mut best = f64::INFINITY;
            for index in 0..corners.len() {
                let a = corners[index];
                let b = corners[(index + 1) % corners.len()];
                best = best.min(point_segment_distance(point, a, b));
            }
            best
        }
        _ => shape.distance(&point),
    }
}

fn point_segment_distance(point: FloatPoint, a: FloatPoint, b: FloatPoint) -> f64 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let length_sq = dx * dx + dy * dy;
    if length_sq <= 1e-12 {
        return point.distance(&a);
    }
    let t = (((point.x - a.x) * dx + (point.y - a.y) * dy) / length_sq).clamp(0.0, 1.0);
    let closest = FloatPoint::new(a.x + t * dx, a.y + t * dy);
    point.distance(&closest)
}

fn terminal_copper_on_layer(
    request: &GridPathRequest<'_>,
    point: Point,
    terminal_layer: &str,
    path_layer: &str,
) -> Result<bool, String> {
    if terminal_layer == path_layer {
        return Ok(true);
    }
    let gx = round_to_cell(
        point[0] * request.board.units_per_mm,
        request.grid_step_board,
    )?;
    let gy = round_to_cell(
        point[1] * request.board.units_per_mm,
        request.grid_step_board,
    )?;
    Ok(request.through_hole_positions.contains(&(gx, gy)))
}

fn round_to_cell(coord_board: f64, step_board: i32) -> Result<i32, String> {
    let value = (coord_board / step_board as f64).round();
    if !value.is_finite() || value < i32::MIN as f64 || value > i32::MAX as f64 {
        return Err("terminal grid cell exceeds supported range".into());
    }
    Ok(value as i32)
}

type ViaGeometry = (ViaInfoId, f64, f64, i32, i32);

fn select_refit_via(
    request: &GridPathRequest<'_>,
    point_mm: Point,
    requested_diameter_mm: f64,
    requested_drill_mm: f64,
) -> Result<ViaGeometry, String> {
    let mut candidates = vec![(requested_diameter_mm, requested_drill_mm)];
    for &(diameter, drill) in request.via_refit_rungs {
        if diameter < requested_diameter_mm - 1e-9
            && !candidates.iter().any(|(old_d, old_h)| {
                (old_d - diameter).abs() < 1e-9 && (old_h - drill).abs() < 1e-9
            })
        {
            candidates.push((diameter, drill));
        }
    }
    let copper: Vec<i32> = request
        .board
        .board
        .layer_structure
        .layers
        .iter()
        .enumerate()
        .filter_map(|(index, layer)| layer.is_signal.then_some(index as i32))
        .collect();
    let (Some(&first), Some(&last)) = (copper.first(), copper.last()) else {
        return Err("grid path board has no copper layers".into());
    };
    let mut exact_requested = None;
    for &(diameter, drill) in &candidates {
        let Some(via) = matching_via_info(request, first, last, diameter, drill)? else {
            continue;
        };
        if exact_requested.is_none()
            && diameter == requested_diameter_mm
            && drill == requested_drill_mm
        {
            exact_requested = Some(via);
        }
        if via_fits(request, point_mm, first, last, diameter, drill)? {
            return Ok(via);
        }
    }
    // Source `_unblock_via_refit` returns the original record if no smaller
    // rung clears; `_emit_via_size` then still writes that honest size.
    exact_requested.ok_or_else(|| {
        format!(
            "no allowed full-board via matches registered {:.3}/{:.3} mm",
            requested_diameter_mm, requested_drill_mm
        )
    })
}

fn matching_via_info(
    request: &GridPathRequest<'_>,
    first: i32,
    last: i32,
    requested_diameter_mm: f64,
    requested_drill_mm: f64,
) -> Result<Option<ViaGeometry>, String> {
    let copper: Vec<i32> = (first..=last)
        .filter(|&layer| request.board.board.layer_structure.layers[layer as usize].is_signal)
        .collect();
    for &via_info_id in request.via_candidates {
        let info = &request.board.board.rules.via_infos[via_info_id];
        let padstack = request
            .board
            .board
            .library
            .padstacks
            .get(info.get_padstack())
            .ok_or("grid path via candidate has no native padstack")?;
        if padstack.from_layer() != first || padstack.to_layer() != last {
            continue;
        }
        let diameter_board = copper
            .iter()
            .filter_map(|&layer| padstack.get_shape(layer))
            .map(|shape| shape.bounding_box().max_width() as f64)
            .fold(0.0, f64::max);
        let drill_board = 2.0 * padstack.get_drill_radius();
        let diameter_mm = diameter_board / request.board.units_per_mm;
        let drill_mm = drill_board / request.board.units_per_mm;
        if (diameter_mm - requested_diameter_mm).abs() <= VIA_MATCH_TOL_MM
            && (drill_mm - requested_drill_mm).abs() <= VIA_MATCH_TOL_MM
        {
            return Ok(Some((via_info_id, diameter_mm, drill_mm, first, last)));
        }
    }
    Ok(None)
}

fn via_fits(
    request: &GridPathRequest<'_>,
    point_mm: Point,
    first_copper: i32,
    last_copper: i32,
    diameter_mm: f64,
    requested_drill_mm: f64,
) -> Result<bool, String> {
    let board = &request.board.board;
    let upm = request.board.units_per_mm;
    let point_board = [point_mm[0] * upm, point_mm[1] * upm];
    let own_class = board
        .rules
        .net_class_of(request.net)
        .get_trace_clearance_class();
    let mut own_stack_clearance = 0.0f64;
    for layer in first_copper..=last_copper {
        if !board.layer_structure.layers[layer as usize].is_signal {
            continue;
        }
        let own = board
            .rules
            .clearance_matrix
            .get_value(own_class, own_class, layer, false) as f64;
        own_stack_clearance = own_stack_clearance.max(own);
        let need = diameter_mm * upm / 2.0 + own - UNBLOCK_REFIT_MARGIN_MM * upm;

        let pad_distance = foreign_pad_distance(request, point_mm, layer, own)?;
        if pad_distance < need {
            return Ok(false);
        }

        for key in board.get_traces() {
            let trace = board.item(key);
            if trace.contains_net(request.net)
                || trace.net_numbers().is_empty()
                || trace.first_layer(board) != layer
            {
                continue;
            }
            let trace_class = trace.clearance_class();
            let pair_clearance =
                board
                    .rules
                    .clearance_matrix
                    .get_value(own_class, trace_class, layer, false) as f64;
            let corners = trace.trace().polyline().corner_approx_arr();
            let radius = trace.trace().half_width() as f64;
            for segment in corners.windows(2) {
                let distance = point_segment_distance(
                    FloatPoint::new(point_board[0], point_board[1]),
                    segment[0],
                    segment[1],
                ) - radius;
                let required =
                    diameter_mm * upm / 2.0 + pair_clearance - UNBLOCK_REFIT_MARGIN_MM * upm;
                if distance < required {
                    return Ok(false);
                }
            }
        }

        // `_seg_foreign_seg_dist` also treats vias as degenerate segments on
        // their copper layers, using the same per-layer clearance class.
        let layer_name = &board.layer_structure.layers[layer as usize].name;
        for via in request.existing_vias {
            if via.net_id == request.net as i32 || !via.layers.iter().any(|name| name == layer_name)
            {
                continue;
            }
            let d = distance(via.point, point_mm) * upm - via.diameter * upm / 2.0;
            let other_net_class = board
                .rules
                .net_class_of(via.net_id as NetNo)
                .get_trace_clearance_class();
            let pair_clearance =
                board
                    .rules
                    .clearance_matrix
                    .get_value(own_class, other_net_class, layer, false) as f64;
            let required = diameter_mm * upm / 2.0 + pair_clearance - UNBLOCK_REFIT_MARGIN_MM * upm;
            if d < required {
                return Ok(false);
            }
        }
    }

    // `_seg_foreign_via_dist` evaluates via bodies at stack clearance. This is
    // intentionally a separate check from each layer's route-map clearance.
    for via in request.existing_vias {
        if via.net_id == request.net as i32 {
            continue;
        }
        let d = distance(via.point, point_mm) - via.diameter / 2.0;
        let other_class = board
            .rules
            .net_class_of(via.net_id as NetNo)
            .get_trace_clearance_class();
        let pair_stack = (first_copper..=last_copper)
            .filter(|&layer| board.layer_structure.layers[layer as usize].is_signal)
            .filter(|&layer| {
                via.layers
                    .contains(&board.layer_structure.layers[layer as usize].name)
            })
            .map(|layer| {
                board
                    .rules
                    .clearance_matrix
                    .get_value(own_class, other_class, layer, false) as f64
            })
            .fold(own_stack_clearance, f64::max)
            / upm;
        let required = diameter_mm / 2.0 + pair_stack - UNBLOCK_REFIT_MARGIN_MM;
        if d < required {
            return Ok(false);
        }
    }

    // Mechanical drill spacing is enforced even for same-net vias.
    for via in request.existing_vias {
        if via.net_id != request.net as i32 {
            continue;
        }
        let d = distance(via.point, point_mm);
        if d < via.drill / 2.0 + requested_drill_mm / 2.0 + request.hole_to_hole_clearance_mm {
            return Ok(false);
        }
    }
    Ok(true)
}

fn reusable_same_net_via(
    request: &GridPathRequest<'_>,
    point: Point,
    new_drill_mm: f64,
) -> Result<bool, String> {
    for item in request.existing_vias {
        if item.net_id != request.net as i32 {
            continue;
        }
        let via_distance = distance(item.point, point);
        if via_distance < (new_drill_mm + item.drill) / 2.0 + request.hole_to_hole_clearance_mm
            && via_distance <= item.diameter / 2.0
        {
            return Ok(true);
        }
    }
    Ok(false)
}
