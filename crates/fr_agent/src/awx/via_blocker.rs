//! Port of `plane_blocker_detection.py` `find_via_position_blocker` (`20..90`).
//!
//! Pure READ-only board geometry. Given a candidate via position, return the
//! EXACT foreign net whose mature Trace/Via copper boxes it (the source
//! declined-tap blame query consumed by `native_rung::record_via_unblock_blame`).
//!
//! Source contract, ported verbatim:
//! * copper is scanned trace segments FIRST, then through-vias;
//! * a candidate only qualifies on the STRICT test
//!   `distance2 < (new_via_radius + foreign_half_width + rules.clearance)^2`;
//! * among qualifying copper the NEAREST (strict `<`) wins, so the first item
//!   in scan order keeps a tie;
//! * the routing target (`exclude`) and every `protected` net are skipped.
//!
//! No Pads and no net-named guesses are ever blockers: only native Trace/Via
//! copper on the current committed board is read. There is deliberately no
//! via-rung selection, cost tuning, or layer fabrication here — traces are
//! read on every layer because a through-via spans the full copper stack, and a
//! via's copper diameter is taken from its public padstack geometry over its
//! own full layer span.

use super::whole_context::RoutingRules;
use crate::board_adapter::BoardAdapter;
use fr_engine::ids::NetNo;
use std::collections::BTreeSet;

/// Return the nearest mature foreign Trace/Via net whose copper blocks a via
/// placed at `point` (board millimetres), or `None` when nothing qualifies.
///
/// `exclude` is the routing target; `protected` is supplied by the caller (the
/// native net-0 default set is the caller's responsibility, never hard-coded
/// here).
pub(crate) fn find_via_position_blocker(
    board: &BoardAdapter,
    point: [f64; 2],
    rules: &RoutingRules,
    exclude: NetNo,
    protected: &BTreeSet<NetNo>,
) -> Option<NetNo> {
    let upm = board.units_per_mm;
    if !upm.is_finite() || upm <= 0.0 {
        return None;
    }
    let new_radius = rules.via_size / 2.0;
    let clearance = rules.clearance;

    let mut best_net: Option<NetNo> = None;
    let mut best_dist2 = f64::INFINITY;

    // Source order: adjacent-corner trace segments first.
    for key in board.board.get_traces() {
        let item = board.board.item(key);
        let Some(net) = single_net(item.net_numbers()) else {
            continue;
        };
        if net == exclude || protected.contains(&net) {
            continue;
        }
        let Some(trace) = item.as_trace() else {
            continue;
        };
        let polyline = trace.polyline();
        let half = trace.half_width() as f64 / upm;
        let threshold = new_radius + half + clearance;
        let threshold2 = threshold * threshold;
        for index in 0..(polyline.corner_count() - 1).max(0) {
            let a = polyline.corner(index).to_float();
            let b = polyline.corner(index + 1).to_float();
            let distance2 = point_segment_distance_sq(
                point,
                [a.x / upm, a.y / upm],
                [b.x / upm, b.y / upm],
            );
            if distance2 < threshold2 && distance2 < best_dist2 {
                best_dist2 = distance2;
                best_net = Some(net);
            }
        }
    }

    // Then through-vias, priced from their raw public padstack copper geometry.
    for key in board.board.get_vias() {
        let item = board.board.item(key);
        let Some(net) = single_net(item.net_numbers()) else {
            continue;
        };
        if net == exclude || protected.contains(&net) {
            continue;
        }
        let Some(stack) = item.padstack(&board.board) else {
            continue;
        };
        let diameter = (stack.from_layer()..=stack.to_layer())
            .filter_map(|layer| stack.get_shape(layer))
            .map(|shape| shape.max_width())
            .fold(0.0_f64, f64::max)
            / upm;
        let center = item.center(&board.board).to_float();
        let threshold = new_radius + diameter / 2.0 + clearance;
        let dx = center.x / upm - point[0];
        let dy = center.y / upm - point[1];
        let distance2 = dx * dx + dy * dy;
        if distance2 < threshold * threshold && distance2 < best_dist2 {
            best_dist2 = distance2;
            best_net = Some(net);
        }
    }

    best_net
}

/// The source copper item carries exactly one net; anything else is not guessed.
fn single_net(nets: &[NetNo]) -> Option<NetNo> {
    match nets {
        [net] => Some(*net),
        _ => None,
    }
}

/// Squared point-to-segment distance in board millimetres, with the source's
/// clamped projection (`t` clamped to `[0,1]`) and degenerate-segment fallback.
fn point_segment_distance_sq(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let length2 = dx * dx + dy * dy;
    if length2 <= 0.0 {
        let ex = p[0] - a[0];
        let ey = p[1] - a[1];
        return ex * ex + ey * ey;
    }
    let t = (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2).clamp(0.0, 1.0);
    let ex = p[0] - (a[0] + t * dx);
    let ey = p[1] - (a[1] + t * dy);
    ex * ex + ey * ey
}
