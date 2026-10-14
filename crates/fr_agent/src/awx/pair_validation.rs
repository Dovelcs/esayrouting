//! Source-equivalent validation for uncommitted coupled-pair pieces.
//!
//! This ports `awx/connect.py::_legs_clear` and `awx/pairs.py::intra_ok`.
//! It deliberately examines only candidate copper against the source local
//! window; historical board DRCs outside the candidate are not failures here.

use super::{
    braid_connect::RungRequest,
    corridor::Point,
    pair_connection::{PairBandEnd, PairPiece, PairVia},
};
use fr_engine::ids::NetNo;
use std::collections::BTreeSet;

const WINDOW_ITEM_MARGIN_MM: f64 = 2.0;
const CHECK_EPS_MM: f64 = 1e-6;
const PAIR_EPS_MM: f64 = 0.002;
const SOURCE_HUG_OVER_MM: f64 = 0.005;

#[derive(Clone, Copy)]
struct Segment {
    a: Point,
    b: Point,
    width: f64,
    layer: usize,
    net: NetNo,
}

#[derive(Clone, Copy)]
struct ViaDisc {
    point: Point,
    diameter: f64,
    net: NetNo,
}

#[derive(Clone)]
struct PadDisc {
    center: Point,
    size: Point,
    layers: BTreeSet<String>,
    drilled: bool,
}

/// Validate proposed pair copper using the source local-window clearance and
/// pair-short rules. `window_points` affects only the local board crop;
/// `band_ends` affects only source band membership checks.
pub fn validate_pieces(
    request: &RungRequest,
    pieces: &[PairPiece],
    vias: &[PairVia],
    virtual_tolerance: f64,
    check_band: bool,
    band_ends: &[PairBandEnd],
    window_points: &[Point],
    margin_extra: f64,
    other_copper: &[PairPiece],
    other_vias: &[PairVia],
) -> Result<Option<String>, String> {
    let pair = request
        .pair
        .as_ref()
        .ok_or("pair validation requires a differential-pair request")?;
    let (p_id, n_id) = (pair.positive_net_id, pair.negative_net_id);
    let own = BTreeSet::from([p_id, n_id]);
    let rules = &request.rules;
    let clearance = ((rules.clearance + SOURCE_HUG_OVER_MM) * 1_000_000.0).round() / 1_000_000.0;
    if !clearance.is_finite()
        || clearance < 0.0
        || !rules.clearance.is_finite()
        || rules.clearance < 0.0
        || !rules.track_width.is_finite()
        || rules.track_width <= 0.0
        || !rules.via_size.is_finite()
        || rules.via_size <= 0.0
        || !virtual_tolerance.is_finite()
        || virtual_tolerance < 0.0
        || !margin_extra.is_finite()
        || margin_extra < 0.0
    {
        return Err("invalid native pair validation rules or tolerance".into());
    }
    if pieces.iter().any(|piece| {
        !own.contains(&piece.net_id)
            || !piece.width.is_finite()
            || piece.width <= 0.0
            || !piece
                .a
                .iter()
                .chain(&piece.b)
                .all(|coord| coord.is_finite())
            || request.board.board.layer_structure.get_no(&piece.layer) < 0
    }) || vias.iter().any(|via| {
        !own.contains(&via.net_id)
            || !via.diameter.is_finite()
            || via.diameter <= 0.0
            || !via.drill.is_finite()
            || via.drill < 0.0
            || via.drill >= via.diameter
            || !via.point.iter().all(|coord| coord.is_finite())
    }) || other_copper.iter().any(|piece| {
        !piece.width.is_finite()
            || piece.width <= 0.0
            || !piece
                .a
                .iter()
                .chain(&piece.b)
                .all(|coord| coord.is_finite())
    }) || other_vias.iter().any(|via| {
        !via.diameter.is_finite()
            || via.diameter <= 0.0
            || !via.drill.is_finite()
            || via.drill < 0.0
            || via.drill >= via.diameter
            || !via.point.iter().all(|coord| coord.is_finite())
    }) {
        return Err("invalid native pair piece or via".into());
    }

    let Some((window, window_vias, pads)) =
        local_window_obstacles(request, window_points, margin_extra)?
    else {
        return Ok(Some("pair validation local window is empty".into()));
    };
    let board = &request.board.board;
    let scale = request.board.units_per_mm;
    let mut candidate = Vec::new();
    for piece in pieces {
        candidate.push(Segment {
            a: piece.a,
            b: piece.b,
            width: piece.width,
            layer: board.layer_structure.get_no(&piece.layer) as usize,
            net: piece.net_id,
        });
    }
    // `_legs_clear` checks each candidate barrel on every configured pair
    // routing layer, regardless of the via's actual layer span.
    for via in vias {
        for layer in &request.route_layers {
            let layer_no = board.layer_structure.get_no(layer);
            if layer_no >= 0 {
                candidate.push(Segment {
                    a: via.point,
                    b: via.point,
                    width: via.diameter,
                    layer: layer_no as usize,
                    net: via.net_id,
                });
            }
        }
    }

    let mut foreign_segments = Vec::new();
    let mut foreign_vias = Vec::new();
    for key in board.get_traces() {
        let item = board.item(key);
        let net_ids = item.net_numbers();
        if net_ids.is_empty() {
            continue;
        }
        let layer = item.first_layer(board);
        if layer < 0 {
            continue;
        }
        let layer_name = &board.layer_structure.layers[layer as usize].name;
        let Some(layer_index) = board
            .layer_structure
            .layers
            .iter()
            .position(|l| &l.name == layer_name)
        else {
            continue;
        };
        let trace = item.trace();
        let points = trace.polyline().corner_approx_arr();
        let width = 2.0 * trace.half_width() as f64 / scale;
        for pair_points in points.windows(2) {
            let a = [pair_points[0].x / scale, pair_points[0].y / scale];
            let b = [pair_points[1].x / scale, pair_points[1].y / scale];
            if !segment_overlaps_window(a, b, width, window) {
                continue;
            }
            for &net in net_ids.iter().filter(|net| !own.contains(*net)) {
                foreign_segments.push(Segment {
                    a,
                    b,
                    width,
                    layer: layer_index,
                    net,
                });
            }
        }
    }
    for key in board.get_vias() {
        let item = board.item(key);
        let foreign_nets: Vec<_> = item
            .net_numbers()
            .iter()
            .copied()
            .filter(|net| !own.contains(net))
            .collect();
        if foreign_nets.is_empty() {
            continue;
        }
        let center = item.center(board).to_float();
        let point = [center.x / scale, center.y / scale];
        let diameter = item
            .padstack(board)
            .and_then(|padstack| {
                (0..board.layer_count()).find_map(|layer| {
                    padstack.get_shape(layer).map(|shape| {
                        let bbox = shape.bounding_box();
                        bbox.width().max(bbox.height()) as f64 / scale
                    })
                })
            })
            .ok_or("foreign pair-window via has no copper shape")?;
        if !point_near_window(point, diameter, window) {
            continue;
        }
        for net in foreign_nets {
            foreign_vias.push(ViaDisc {
                point,
                diameter,
                net,
            });
        }
    }
    let mut foreign_pieces = Vec::new();
    for piece in other_copper {
        let layer = board.layer_structure.get_no(&piece.layer);
        if layer < 0 {
            return Err(format!(
                "unknown layer in pair crossover copper: {}",
                piece.layer
            ));
        }
        foreign_pieces.push(Segment {
            a: piece.a,
            b: piece.b,
            width: piece.width,
            layer: layer as usize,
            net: piece.net_id,
        });
    }
    for via in other_vias {
        foreign_vias.push(ViaDisc {
            point: via.point,
            diameter: via.diameter,
            net: via.net_id,
        });
    }

    for segment in &candidate {
        let layer_name = &board.layer_structure.layers[segment.layer].name;
        for foreign in foreign_segments.iter().chain(&foreign_pieces) {
            if foreign.layer != segment.layer {
                continue;
            }
            let needed = clearance + (segment.width + foreign.width) / 2.0;
            if segment_distance(segment.a, segment.b, foreign.a, foreign.b) < needed - CHECK_EPS_MM
            {
                return Ok(Some(format!(
                    "pair piece on {layer_name} grazes net {} copper near ({:.2},{:.2})",
                    foreign.net, segment.a[0], segment.a[1]
                )));
            }
        }
        for tube in &request.virtual_tracks {
            if tube.layer == *layer_name {
                let needed =
                    clearance + (segment.width + rules.track_width) / 2.0 - virtual_tolerance;
                if segment_distance(segment.a, segment.b, tube.a, tube.b) < needed - CHECK_EPS_MM {
                    return Ok(Some(format!(
                        "pair piece on {layer_name} grazes virtual copper near ({:.2},{:.2})",
                        segment.a[0], segment.a[1]
                    )));
                }
            }
        }
        for via in &window_vias {
            let tolerance = if via.net == VIRTUAL_NET {
                virtual_tolerance
            } else {
                0.0
            };
            let needed = clearance + (segment.width + via.diameter) / 2.0 - tolerance;
            if point_segment_distance(via.point, segment.a, segment.b) < needed - CHECK_EPS_MM {
                return Ok(Some(format!(
                    "pair piece on {layer_name} grazes net {} via at ({:.2},{:.2})",
                    via.net, via.point[0], via.point[1]
                )));
            }
        }
        for via in &foreign_vias {
            let needed = clearance + (segment.width + via.diameter) / 2.0;
            if point_segment_distance(via.point, segment.a, segment.b) < needed - CHECK_EPS_MM {
                return Ok(Some(format!(
                    "pair piece on {layer_name} grazes net {} via at ({:.2},{:.2})",
                    via.net, via.point[0], via.point[1]
                )));
            }
        }
        for pad in &pads {
            if !pad.drilled && !pad.layers.contains(layer_name) {
                continue;
            }
            let radius = pad.size[0].hypot(pad.size[1]) / 2.0;
            if point_segment_distance(pad.center, segment.a, segment.b)
                < clearance + segment.width / 2.0 + radius - CHECK_EPS_MM
            {
                let midpoint = [
                    (segment.a[0] + segment.b[0]) / 2.0,
                    (segment.a[1] + segment.b[1]) / 2.0,
                ];
                let ex = ((pad.center[0] - midpoint[0]).abs() - pad.size[0] / 2.0).max(0.0);
                let ey = ((pad.center[1] - midpoint[1]).abs() - pad.size[1] / 2.0).max(0.0);
                if ex.hypot(ey) < clearance + segment.width / 2.0 - CHECK_EPS_MM {
                    return Ok(Some(format!(
                        "pair piece on {layer_name} grazes pad near ({:.2},{:.2})",
                        pad.center[0], pad.center[1]
                    )));
                }
            }
        }
    }

    if check_band {
        if let Some(band) = &request.band {
            for end in band_ends {
                if !band.contains(end.point, &end.layer) {
                    return Ok(Some(format!(
                        "pair piece end ({:.2},{:.2}) on {} is outside the band",
                        end.point[0], end.point[1], end.layer
                    )));
                }
            }
        }
    }
    if let Some(reason) = pair_short_reason(
        pieces,
        vias,
        p_id,
        n_id,
        rules.track_width,
        rules.via_size,
        clearance,
    ) {
        return Ok(Some(reason));
    }
    Ok(None)
}

const VIRTUAL_NET: NetNo = 10_000_000;

fn local_window_obstacles(
    request: &RungRequest,
    window_points: &[Point],
    margin_extra: f64,
) -> Result<Option<([f64; 4], Vec<ViaDisc>, Vec<PadDisc>)>, String> {
    let pair = request
        .pair
        .as_ref()
        .ok_or("pair validation requires pair configuration")?;
    let mut points = vec![
        pair.start_points[0],
        pair.start_points[1],
        pair.end_points[0],
        pair.end_points[1],
    ];
    points.extend_from_slice(window_points);
    if points.is_empty() || points.iter().flatten().any(|v| !v.is_finite()) {
        return Err("pair validation window has invalid coordinates".into());
    }
    let margin = request.spec.margin + margin_extra;
    if !margin.is_finite() || margin < 0.0 {
        return Err("pair validation window margin is invalid".into());
    }
    let mut bounds = [
        points.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min),
        points.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min),
        points
            .iter()
            .map(|p| p[0])
            .fold(f64::NEG_INFINITY, f64::max),
        points
            .iter()
            .map(|p| p[1])
            .fold(f64::NEG_INFINITY, f64::max),
    ];
    let half = ((bounds[2] - bounds[0]).max(bounds[3] - bounds[1])) / 2.0 + margin;
    let cx = (bounds[0] + bounds[2]) / 2.0;
    let cy = (bounds[1] + bounds[3]) / 2.0;
    bounds = [cx - half, cy - half, cx + half, cy + half];
    let bbox = request.board.board.bounding_box();
    let scale = request.board.units_per_mm;
    bounds[0] = bounds[0].max(bbox.ll.x as f64 / scale);
    bounds[1] = bounds[1].max(bbox.ll.y as f64 / scale);
    bounds[2] = bounds[2].min(bbox.ur.x as f64 / scale);
    bounds[3] = bounds[3].min(bbox.ur.y as f64 / scale);
    if bounds[0] > bounds[2] || bounds[1] > bounds[3] {
        return Ok(None);
    }
    bounds = [
        bounds[0] - WINDOW_ITEM_MARGIN_MM,
        bounds[1] - WINDOW_ITEM_MARGIN_MM,
        bounds[2] + WINDOW_ITEM_MARGIN_MM,
        bounds[3] + WINDOW_ITEM_MARGIN_MM,
    ];

    let board = &request.board.board;
    let own = [pair.positive_net_id, pair.negative_net_id];
    let mut vias = request
        .virtual_vias
        .iter()
        .map(|via| ViaDisc {
            point: via.point,
            diameter: request.rules.via_size,
            net: VIRTUAL_NET,
        })
        .collect::<Vec<_>>();
    for key in board.get_vias() {
        let item = board.item(key);
        let foreign_nets: Vec<_> = item
            .net_numbers()
            .iter()
            .copied()
            .filter(|net| !own.contains(net))
            .collect();
        if foreign_nets.is_empty() {
            continue;
        }
        let center = item.center(board).to_float();
        let point = [center.x / scale, center.y / scale];
        let diameter = item
            .padstack(board)
            .and_then(|padstack| {
                (0..board.layer_count()).find_map(|layer| {
                    padstack.get_shape(layer).map(|shape| {
                        let bbox = shape.bounding_box();
                        bbox.width().max(bbox.height()) as f64 / scale
                    })
                })
            })
            .ok_or("local-window via has no native copper shape")?;
        if point_near_window(point, diameter, bounds) {
            vias.extend(foreign_nets.iter().map(|&net| ViaDisc {
                point,
                diameter,
                net,
            }));
        }
    }

    let mut pads = Vec::new();
    for key in board.get_pins() {
        let item = board.item(key);
        if item.net_numbers().iter().any(|net| own.contains(net)) {
            continue;
        }
        let Some(padstack) = item.padstack(board) else {
            continue;
        };
        if padstack.hole_only {
            continue;
        }
        let drill = padstack.get_drill_radius() > 0.0;
        let center = item.center(board).to_float();
        let center = [center.x / scale, center.y / scale];
        let mut pad_layers = BTreeSet::new();
        let mut max_size = [0.0f64; 2];
        for layer in item.first_layer(board)..=item.last_layer(board) {
            if layer < 0 || !board.layer_structure.layers[layer as usize].is_signal {
                continue;
            }
            let layer_name = board.layer_structure.layers[layer as usize].name.clone();
            pad_layers.insert(layer_name);
            if let Some(shape) = item.drill_shape_on_layer(board, layer) {
                let box_ = shape.bounding_box();
                max_size[0] = max_size[0].max(box_.width() as f64 / scale);
                max_size[1] = max_size[1].max(box_.height() as f64 / scale);
            }
        }
        if !point_near_window(center, max_size[0].max(max_size[1]), bounds) {
            continue;
        }
        pads.push(PadDisc {
            center,
            size: max_size,
            layers: pad_layers,
            drilled: drill,
        });
    }
    Ok(Some((bounds, vias, pads)))
}

fn segment_overlaps_window(a: Point, b: Point, width: f64, window: [f64; 4]) -> bool {
    let radius = width / 2.0;
    !(a[0].max(b[0]) + radius < window[0]
        || a[0].min(b[0]) - radius > window[2]
        || a[1].max(b[1]) + radius < window[1]
        || a[1].min(b[1]) - radius > window[3])
}

fn point_near_window(point: Point, reach: f64, window: [f64; 4]) -> bool {
    window[0] - reach <= point[0]
        && point[0] <= window[2] + reach
        && window[1] - reach <= point[1]
        && point[1] <= window[3] + reach
}

fn point_segment_distance(point: Point, a: Point, b: Point) -> f64 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let length_squared = dx * dx + dy * dy;
    if length_squared <= 1e-12 {
        return (point[0] - a[0]).hypot(point[1] - a[1]);
    }
    let t = (((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length_squared).clamp(0.0, 1.0);
    (point[0] - (a[0] + t * dx)).hypot(point[1] - (a[1] + t * dy))
}

fn segment_distance(a: Point, b: Point, c: Point, d: Point) -> f64 {
    let orient = |p: Point, q: Point, r: Point| {
        (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    };
    let (o1, o2, o3, o4) = (
        orient(a, b, c),
        orient(a, b, d),
        orient(c, d, a),
        orient(c, d, b),
    );
    if o1 * o2 < 0.0 && o3 * o4 < 0.0 {
        return 0.0;
    }
    point_segment_distance(a, c, d)
        .min(point_segment_distance(b, c, d))
        .min(point_segment_distance(c, a, b))
        .min(point_segment_distance(d, a, b))
}

fn pair_short_reason(
    pieces: &[PairPiece],
    vias: &[PairVia],
    p_id: NetNo,
    n_id: NetNo,
    track: f64,
    via_size: f64,
    clearance: f64,
) -> Option<String> {
    let need_segments = track + clearance - PAIR_EPS_MM;
    for p in pieces.iter().filter(|piece| piece.net_id == p_id) {
        for n in pieces
            .iter()
            .filter(|piece| piece.net_id == n_id && piece.layer == p.layer)
        {
            let distance = segment_distance(p.a, p.b, n.a, n.b);
            if distance < need_segments {
                return Some(format!(
                    "P/N pair short: legs are {distance:.3} mm apart on {}",
                    p.layer
                ));
            }
        }
    }
    let need_via_segment = via_size / 2.0 + clearance + track / 2.0 - PAIR_EPS_MM;
    for via in vias {
        let other_net = if via.net_id == p_id { n_id } else { p_id };
        for piece in pieces.iter().filter(|piece| piece.net_id == other_net) {
            if point_segment_distance(via.point, piece.a, piece.b) < need_via_segment {
                return Some("P/N pair short: via barrel approaches the opposite leg".into());
            }
        }
    }
    let need_via_via = via_size + clearance - PAIR_EPS_MM;
    for (index, a) in vias.iter().enumerate() {
        for b in vias.iter().skip(index + 1).filter(|b| b.net_id != a.net_id) {
            if (a.point[0] - b.point[0]).hypot(a.point[1] - b.point[1]) < need_via_via {
                return Some("P/N pair short: opposing via barrels are too close".into());
            }
        }
    }
    None
}
