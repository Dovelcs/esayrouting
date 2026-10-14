//! Source-compatible pair connection orchestration from `awx/connect.py`.
//!
//! This module owns the source pair approach, routed/geometric connector,
//! envelope split, and retry decisions. The backend owns live-board checks and
//! the native GridRouter/PoseRouter bridge; it must execute one coupled pose
//! route for the pair and may execute the source's single envelope connector.

use super::{
    braid_connect::{PairConnector, PairRungRequest, RungBandConstraint, RungReport, RungRequest},
    corridor::{self, Point},
    pairs,
};
use fr_engine::ids::NetNo;

const HUG_OVER_MM: f64 = 0.005;
const STRAIGHTEN_STEPS: f64 = 1.2;
const VIRT_TOL_FRAC: f64 = 0.1;
const GEO_CONNECTOR_LEAD_MM: f64 = 0.3;

#[derive(Clone, Debug, PartialEq)]
pub struct PairPiece {
    pub a: Point,
    pub b: Point,
    pub width: f64,
    pub layer: String,
    pub net_id: NetNo,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PairVia {
    pub point: Point,
    pub diameter: f64,
    pub drill: f64,
    pub layers: Vec<String>,
    pub net_id: NetNo,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PairBandEnd {
    pub point: Point,
    pub layer: String,
}

/// Source `a_given`/`b_given`: a connector already drawn by a whole-route plan.
#[derive(Clone, Debug, PartialEq)]
pub struct PairGivenEnd {
    pub pieces: Vec<PairPiece>,
    pub vias: Vec<PairVia>,
    pub end_points: [Point; 2],
    pub direction: Point,
    pub layer: String,
}

/// Source `x_given`: the already-drawn crossover joining two coupled spans.
#[derive(Clone, Debug, PartialEq)]
pub struct PairGivenCrossover {
    pub heading: Point,
    pub layers: [String; 2],
    pub entry_points: [Point; 2],
    pub exit_points: [Point; 2],
    /// Positive then negative, preserving each leg's ordered source runs.
    pub legs: [Vec<PairPiece>; 2],
    pub vias: Vec<PairVia>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct EnvelopeRun {
    pub layer: String,
    pub points: Vec<Point>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct EnvelopeRoute {
    /// Ordered centreline runs; a layer transition between runs is a via.
    pub runs: Vec<EnvelopeRun>,
    pub vias: Vec<PairVia>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum BandRestriction {
    /// The band is closed inside this disc on the named layer.
    OutsideDisc {
        center: Point,
        radius: f64,
        layer: String,
    },
    /// Close the rear half-disc behind a connector approach on every layer.
    ForwardHalfDisc {
        center: Point,
        direction: Point,
        radius: f64,
        grid: f64,
    },
}

#[derive(Clone, Debug)]
pub struct EnvelopeRequest {
    pub start: Point,
    pub end: Point,
    pub start_layer: String,
    pub end_layer: String,
    pub positive_net_id: NetNo,
    pub negative_net_id: NetNo,
    pub envelope_track_width: f64,
    pub envelope_via_size: f64,
    pub envelope_via_drill: f64,
    pub virtual_size: (f64, f64),
    pub margin: f64,
    pub band_slack: f64,
    pub window_points: Vec<Point>,
    pub band_restrictions: Vec<BandRestriction>,
}

#[derive(Clone, Debug)]
pub struct PairPoseRequest {
    pub start_points: [Point; 2],
    pub end_points: [Point; 2],
    pub start_layer: String,
    pub end_layer: String,
    pub start_direction: Option<Point>,
    pub end_direction: Option<Point>,
    pub gap: f64,
    pub margin: f64,
    pub band_slack: f64,
    pub window_points: Vec<Point>,
    /// Source connectors and approaches already present in the pose router's
    /// local window, matching `_connect_pair_prod`'s window augmentation.
    pub preexisting_pieces: Vec<PairPiece>,
    pub preexisting_vias: Vec<PairVia>,
    /// Earlier span copper present in `_connect_pair_crossover`'s cloned PCB.
    pub other_copper: Vec<PairPiece>,
    pub other_vias: Vec<PairVia>,
    pub band_restrictions: Vec<BandRestriction>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PairPoseRoute {
    pub pieces: Vec<PairPiece>,
    pub vias: Vec<PairVia>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum PairPoseOutcome {
    Found(PairPoseRoute),
    Refused {
        reason: Option<String>,
        report: Option<RungReport>,
        /// True when the source router returned neither new segments nor vias.
        empty: bool,
        /// True when the source router refused without an expanded blocked frontier.
        frontier_empty: bool,
        /// True when the native path escaped its fenced local window.
        escaped_window: bool,
    },
}

/// Runtime bridge implemented by `native_rung`: it sees the live board and
/// performs the only conversion between board millimetres and native grid cells.
pub trait PairConnectionBackend {
    /// Source `_legs_clear`: exact foreign copper, pad, via, band and virtual
    /// reservation check for a not-yet-committed pair connector.
    fn validate_pieces(
        &mut self,
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
    ) -> Result<Option<String>, String>;

    /// Source `connect()` for a single envelope centreline, with both pair
    /// net IDs exempt as own copper and the source virtual-size semantics.
    fn route_envelope(
        &mut self,
        request: &RungRequest,
        envelope: &EnvelopeRequest,
    ) -> Result<Option<EnvelopeRoute>, String>;

    /// One coupled source pose route. Never implement this as two single-net
    /// searches; `native_rung` delegates it to the imported production router.
    fn route_pair_pose(
        &mut self,
        request: &RungRequest,
        pose: &PairPoseRequest,
    ) -> Result<PairPoseOutcome, String>;
}

#[derive(Clone, Debug, PartialEq)]
pub struct PairConnectionResult {
    pub pieces: Vec<PairPiece>,
    pub vias: Vec<PairVia>,
    pub start_points: [Point; 2],
    pub end_points: [Point; 2],
    pub start_layer: String,
    pub end_layer: String,
    pub start_direction: Option<Point>,
    pub end_direction: Option<Point>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum PairConnectionOutcome {
    Found(PairConnectionResult),
    Refused {
        reason: Option<String>,
        report: Option<RungReport>,
    },
}

/// Port of `connect_pair`: make any source-directed approach/connector, then
/// hand its final pair poses to one production coupled router call.
pub fn connect_pair(
    request: &RungRequest,
    pair: &PairRungRequest,
    backend: &mut impl PairConnectionBackend,
) -> Result<PairConnectionOutcome, String> {
    validate_input(request, pair)?;
    let clearance = source_clearance(request);
    let gap = pair.gap.max(clearance);
    let half = (request.rules.track_width + gap) / 2.0;
    let a_p = pair.start_points[0];
    let a_n = pair.start_points[1];
    let b_p = pair.end_points[0];
    let b_n = pair.end_points[1];

    // The source plan's pre-drawn crossover takes precedence over every
    // approach/crossover search and is routed as two separately given spans.
    if let Some(crossover) = &pair.x_given {
        return connect_given_crossover(request, pair, crossover, backend, gap, half);
    }

    // Source `connect_pair` tries its envelope crossover at each production
    // approach scale before the ordinary `_connect_pair_prod` path.
    if let (Some(a_dir), Some(b_dir)) = (pair.start_direction, pair.end_direction) {
        let h0 = pairs::hand_vector(Some(a_dir), a_p, a_n, false);
        let h1 = pairs::hand_vector(Some(b_dir), b_p, b_n, true);
        if h0 != 0 && h1 != 0 && h0 != h1 && pair.allow_cross {
            for scale in [1.0, 2.0, 3.0] {
                if let Some(found) =
                    connect_opposite_hands(request, pair, backend, gap, half, scale)?
                {
                    return Ok(PairConnectionOutcome::Found(found));
                }
            }
        }
    }

    let mut report = None;
    let mut attempt = 0usize;
    let mut approach_scale = 1.0;
    let mut use_connectors = true;
    loop {
        match connect_pair_prod(
            request,
            pair,
            backend,
            gap,
            half,
            approach_scale,
            attempt,
            if use_connectors {
                pair.start_connectors.as_deref()
            } else {
                None
            },
            if use_connectors {
                pair.end_connectors.as_deref()
            } else {
                None
            },
            pair.a_given.as_ref(),
            pair.b_given.as_ref(),
            &[],
            &[],
        )? {
            Attempt::Found(found) => return Ok(PairConnectionOutcome::Found(found)),
            Attempt::Refused {
                reason,
                report: got,
                retry_scale,
            } => {
                report = got.or(report);
                let Some((next_scale, drop_connectors)) = retry_scale else {
                    return Ok(PairConnectionOutcome::Refused { reason, report });
                };
                approach_scale = next_scale;
                use_connectors &= !drop_connectors;
                attempt += 1;
            }
        }
    }
}

enum Attempt {
    Found(PairConnectionResult),
    Refused {
        reason: Option<String>,
        report: Option<RungReport>,
        retry_scale: Option<(f64, bool)>,
    },
}

fn connect_given_crossover(
    request: &RungRequest,
    pair: &PairRungRequest,
    crossover: &PairGivenCrossover,
    backend: &mut impl PairConnectionBackend,
    gap: f64,
    half: f64,
) -> Result<PairConnectionOutcome, String> {
    let crossover_pieces = crossover.legs.iter().flatten().cloned().collect::<Vec<_>>();
    let first_end = PairGivenEnd {
        pieces: crossover_pieces,
        vias: crossover.vias.clone(),
        end_points: crossover.entry_points,
        direction: [-crossover.heading[0], -crossover.heading[1]],
        layer: crossover.layers[0].clone(),
    };
    let mut first_pair = pair.clone();
    first_pair.end_points = crossover.entry_points;
    first_pair.end_layer = crossover.layers[0].clone();
    first_pair.negative_end_layer = crossover.layers[0].clone();
    first_pair.end_direction = Some([-crossover.heading[0], -crossover.heading[1]]);
    first_pair.start_connectors = None;
    first_pair.end_connectors = None;
    first_pair.a_given = pair.a_given.clone();
    first_pair.b_given = Some(first_end);
    first_pair.x_given = None;
    let first = match connect_pair_prod(
        request,
        &first_pair,
        backend,
        gap,
        half,
        1.0,
        0,
        None,
        None,
        first_pair.a_given.as_ref(),
        first_pair.b_given.as_ref(),
        &[],
        &[],
    )? {
        Attempt::Found(route) => route,
        Attempt::Refused { reason, report, .. } => {
            return Ok(PairConnectionOutcome::Refused {
                reason: Some(format!(
                    "source given crossover span 1 refused: {}",
                    reason.unwrap_or_default()
                )),
                report,
            });
        }
    };

    let second_start = PairGivenEnd {
        pieces: Vec::new(),
        vias: Vec::new(),
        end_points: crossover.exit_points,
        direction: crossover.heading,
        layer: crossover.layers[1].clone(),
    };
    let mut second_pair = pair.clone();
    second_pair.start_points = crossover.exit_points;
    second_pair.start_layer = crossover.layers[1].clone();
    second_pair.negative_start_layer = crossover.layers[1].clone();
    second_pair.start_direction = Some(crossover.heading);
    second_pair.start_connectors = None;
    second_pair.end_connectors = None;
    second_pair.a_given = Some(second_start);
    second_pair.b_given = pair.b_given.clone();
    second_pair.x_given = None;
    let second = match connect_pair_prod(
        request,
        &second_pair,
        backend,
        gap,
        half,
        1.0,
        0,
        None,
        None,
        second_pair.a_given.as_ref(),
        second_pair.b_given.as_ref(),
        &first.pieces,
        &first.vias,
    )? {
        Attempt::Found(route) => route,
        Attempt::Refused { reason, report, .. } => {
            return Ok(PairConnectionOutcome::Refused {
                reason: Some(format!(
                    "source given crossover span 2 refused: {}",
                    reason.unwrap_or_default()
                )),
                report,
            });
        }
    };
    let mut pieces = first.pieces;
    pieces.extend(second.pieces);
    let mut vias = first.vias;
    vias.extend(second.vias);
    Ok(PairConnectionOutcome::Found(PairConnectionResult {
        pieces,
        vias,
        start_points: pair.start_points,
        end_points: pair.end_points,
        start_layer: pair.start_layer.clone(),
        end_layer: pair.end_layer.clone(),
        start_direction: pair.start_direction,
        end_direction: pair.end_direction,
    }))
}

/// Source `_connect_pair_cross`: route an envelope, then put one physical pair
/// crossover at a natural or slid dive. The pair legs remain coupled throughout.
fn connect_opposite_hands(
    request: &RungRequest,
    pair: &PairRungRequest,
    backend: &mut impl PairConnectionBackend,
    gap: f64,
    half: f64,
    approach_scale: f64,
) -> Result<Option<PairConnectionResult>, String> {
    let (Some(a_dir), Some(b_dir)) = (pair.start_direction, pair.end_direction) else {
        return Ok(None);
    };
    let p_id = pair.positive_net_id;
    let n_id = pair.negative_net_id;
    let track = request.rules.track_width;
    let clearance = source_clearance(request);
    let via_size = request.rules.via_size;
    let via_drill = request.rules.via_drill;
    let pitch = via_size + clearance;
    let appr = round6(pitch * approach_scale);
    let reach = 3.0 * pitch;
    let via_half = envelope_via_half(request, half);
    let (a_m, a_e, a_ep, a_en, segs_a) = cross_approach(
        pair.start_points,
        a_dir,
        &pair.start_layer,
        appr,
        half,
        track,
        p_id,
        n_id,
    );
    let (b_m, b_e, b_ep, b_en, segs_b) = cross_approach(
        pair.end_points,
        b_dir,
        &pair.end_layer,
        appr,
        half,
        track,
        p_id,
        n_id,
    );
    let envelope_track = round6(2.0 * half / std::f64::consts::FRAC_PI_8.cos() + track);
    let envelope_via = round6(2.0 * via_half + via_size);
    let envelope_drill = round6(envelope_via - (via_size - via_drill));
    let radius = appr + reach + clearance;
    let radius_back = 2.0 * envelope_track + clearance;
    let mut restrictions = Vec::new();
    if pair.start_layer != pair.end_layer {
        restrictions.push(BandRestriction::OutsideDisc {
            center: a_e,
            radius,
            layer: pair.end_layer.clone(),
        });
        restrictions.push(BandRestriction::OutsideDisc {
            center: b_e,
            radius,
            layer: pair.start_layer.clone(),
        });
    }
    restrictions.push(BandRestriction::ForwardHalfDisc {
        center: a_e,
        direction: a_dir,
        radius: radius_back,
        grid: request.rules.grid_step,
    });
    restrictions.push(BandRestriction::ForwardHalfDisc {
        center: b_e,
        direction: b_dir,
        radius: radius_back,
        grid: request.rules.grid_step,
    });
    let mut window_points = request.window_points.clone().unwrap_or_default();
    window_points.extend([
        pair.start_points[0],
        pair.start_points[1],
        pair.end_points[0],
        pair.end_points[1],
    ]);
    let envelope = EnvelopeRequest {
        start: a_e,
        end: b_e,
        start_layer: pair.start_layer.clone(),
        end_layer: pair.end_layer.clone(),
        positive_net_id: p_id,
        negative_net_id: n_id,
        envelope_track_width: envelope_track,
        envelope_via_size: envelope_via,
        envelope_via_drill: envelope_drill,
        virtual_size: (track, via_size),
        margin: request.spec.margin,
        band_slack: request.band_slack,
        window_points,
        band_restrictions: restrictions,
    };
    let Some(route) = backend.route_envelope(request, &envelope)? else {
        return Ok(None);
    };
    let mut runs: Vec<_> = route
        .runs
        .into_iter()
        .map(|run| EnvelopeRun {
            layer: run.layer,
            points: simplify(&run.points, 0.02),
        })
        .filter(|run| run.points.len() >= 2)
        .collect();
    if runs.is_empty() {
        return Ok(None);
    }
    runs[0].points.insert(0, a_m);
    runs.last_mut().unwrap().points.push(b_m);
    runs = runs
        .into_iter()
        .map(|r| EnvelopeRun {
            layer: r.layer,
            points: simplify(&r.points, 0.02),
        })
        .filter(|r| r.points.len() >= 2)
        .collect();
    let mut poly = Vec::new();
    for run in &runs {
        for &p in &run.points {
            if poly.last().is_none_or(|q| corridor::distance(*q, p) > 1e-6) {
                poly.push(p);
            }
        }
    }
    let mut checked_points = poly.clone();
    checked_points.extend(pair.start_points);
    checked_points.extend(pair.end_points);
    let approach_legs = [segs_a, segs_b].concat();
    let virtual_tolerance = VIRT_TOL_FRAC * clearance;
    let mut candidates = vec![runs.clone()];
    if pair.start_layer != pair.end_layer {
        candidates.extend(cross_slides(
            &poly,
            &pair.start_layer,
            &pair.end_layer,
            reach + pitch,
        ));
    }
    for candidate_runs in candidates {
        for k in (0..candidate_runs.len().saturating_sub(1)).rev() {
            if let Some((legs, vias)) = cross_join_at(
                request,
                pair,
                backend,
                &candidate_runs,
                k,
                (a_m, b_m, a_ep, a_en, b_ep, b_en),
                &approach_legs,
                half,
                via_half,
                pitch,
                virtual_tolerance,
                &checked_points,
            )? {
                return Ok(Some(PairConnectionResult {
                    pieces: legs,
                    vias,
                    start_points: pair.start_points,
                    end_points: pair.end_points,
                    start_layer: pair.start_layer.clone(),
                    end_layer: pair.end_layer.clone(),
                    start_direction: Some(a_dir),
                    end_direction: Some(b_dir),
                }));
            }
        }
    }
    Ok(None)
}

fn cross_approach(
    tips: [Point; 2],
    direction: Point,
    layer: &str,
    distance: f64,
    half: f64,
    track: f64,
    positive: NetNo,
    negative: NetNo,
) -> (Point, Point, Point, Point, Vec<PairPiece>) {
    let midpoint = pairs::mid(tips[0], tips[1]);
    let normal = [-direction[1], direction[0]];
    let side = if corridor::cross(direction, corridor::sub(tips[0], midpoint)) >= 0.0 {
        1.0
    } else {
        -1.0
    };
    let end_mid = corridor::add(midpoint, corridor::mul(direction, distance));
    let p_end = corridor::add(end_mid, corridor::mul(normal, side * half));
    let n_end = corridor::sub(end_mid, corridor::mul(normal, side * half));
    let pieces = [
        PairPiece {
            a: tips[0],
            b: p_end,
            width: track,
            layer: layer.into(),
            net_id: positive,
        },
        PairPiece {
            a: tips[1],
            b: n_end,
            width: track,
            layer: layer.into(),
            net_id: negative,
        },
    ]
    .into_iter()
    .filter(|piece| corridor::distance(piece.a, piece.b) > 1e-4)
    .collect();
    (
        midpoint,
        corridor::add(midpoint, corridor::mul(direction, distance)),
        p_end,
        n_end,
        pieces,
    )
}

fn cross_slides(poly: &[Point], a_layer: &str, b_layer: &str, keep: f64) -> Vec<Vec<EnvelopeRun>> {
    let total: f64 = poly
        .windows(2)
        .map(|w| corridor::distance(w[0], w[1]))
        .sum();
    let mut candidates = Vec::new();
    let mut s = 0.0;
    for (i, edge) in poly.windows(2).enumerate() {
        let length = corridor::distance(edge[0], edge[1]);
        for (offset, point, is_vertex) in [
            (
                length / 2.0,
                corridor::mul(corridor::add(edge[0], edge[1]), 0.5),
                false,
            ),
            (length, edge[1], true),
        ] {
            let at = s + offset;
            if keep <= at && at <= total - keep {
                candidates.push((total - at, i, is_vertex, point));
            }
        }
        s += length;
    }
    candidates.sort_by(|a, b| {
        a.0.total_cmp(&b.0)
            .then(a.1.cmp(&b.1))
            .then(a.2.cmp(&b.2))
            .then(a.3[0].total_cmp(&b.3[0]))
            .then(a.3[1].total_cmp(&b.3[1]))
    });
    candidates
        .into_iter()
        .take(12)
        .filter_map(|(_, i, is_vertex, point)| {
            let before = if is_vertex {
                poly[..=i + 1].to_vec()
            } else {
                [poly[..=i].to_vec(), vec![point]].concat()
            };
            let after = if is_vertex {
                poly[i + 1..].to_vec()
            } else {
                [vec![point], poly[i + 1..].to_vec()].concat()
            };
            if before.len() < 2 || after.len() < 2 {
                return None;
            }
            Some(vec![
                EnvelopeRun {
                    layer: a_layer.into(),
                    points: before,
                },
                EnvelopeRun {
                    layer: b_layer.into(),
                    points: after,
                },
            ])
        })
        .collect()
}

#[allow(clippy::too_many_arguments)]
fn cross_join_at(
    request: &RungRequest,
    pair: &PairRungRequest,
    backend: &mut impl PairConnectionBackend,
    runs: &[EnvelopeRun],
    index: usize,
    endpoints: (Point, Point, Point, Point, Point, Point),
    approach_legs: &[PairPiece],
    half: f64,
    via_half: f64,
    pitch: f64,
    virtual_tolerance: f64,
    checked_points: &[Point],
) -> Result<Option<(Vec<PairPiece>, Vec<PairVia>)>, String> {
    let (a_mid, b_mid, a_ep, a_en, b_ep, b_en) = endpoints;
    let incoming = &runs[index];
    let outgoing = &runs[index + 1];
    let vertex = *incoming.points.last().unwrap();
    let d_in = corridor::unit(corridor::sub(
        vertex,
        incoming.points[incoming.points.len() - 2],
    ))?;
    let d_out = corridor::unit(corridor::sub(outgoing.points[1], vertex))?;
    let first_dir = corridor::unit(corridor::sub(runs[0].points[1], runs[0].points[0]))?;
    let last = runs.last().unwrap();
    let last_dir = corridor::unit(corridor::sub(
        *last.points.last().unwrap(),
        last.points[last.points.len() - 2],
    ))?;
    let side = |d: Point, tip: Point, mid: Point| {
        if corridor::cross(d, corridor::sub(tip, mid)) >= 0.0 {
            1.0
        } else {
            -1.0
        }
    };
    let side_in = side(first_dir, a_ep, a_mid);
    let side_out = side(last_dir, b_ep, b_mid);
    if side_in == side_out {
        return Ok(None);
    }
    let n_in = [-d_in[1], d_in[0]];
    let n_out = [-d_out[1], d_out[0]];
    let e_in = [
        corridor::add(vertex, corridor::mul(n_in, side_in * half)),
        corridor::sub(vertex, corridor::mul(n_in, side_in * half)),
    ];
    let e_out = [
        corridor::add(vertex, corridor::mul(n_out, side_out * half)),
        corridor::sub(vertex, corridor::mul(n_out, side_out * half)),
    ];
    let Some((mut in_pieces, mut in_vias)) = super::pair_geometry::split_envelope(
        &runs[..=index],
        &[],
        a_ep,
        a_en,
        e_in[0],
        e_in[1],
        a_mid,
        vertex,
        half,
        via_half,
        request.rules.track_width,
        request.rules.via_size,
        request.rules.via_drill,
        pair.positive_net_id,
        pair.negative_net_id,
        &request.route_layers,
        (
            &pair.start_layer,
            &pair.start_layer,
            &incoming.layer,
            &incoming.layer,
        ),
    )?
    else {
        return Ok(None);
    };
    let Some((out_pieces, out_vias)) = super::pair_geometry::split_envelope(
        &runs[index + 1..],
        &[],
        e_out[0],
        e_out[1],
        b_ep,
        b_en,
        vertex,
        b_mid,
        half,
        via_half,
        request.rules.track_width,
        request.rules.via_size,
        request.rules.via_drill,
        pair.positive_net_id,
        pair.negative_net_id,
        &request.route_layers,
        (
            &outgoing.layer,
            &outgoing.layer,
            &pair.end_layer,
            &pair.end_layer,
        ),
    )?
    else {
        return Ok(None);
    };
    let mut base_pieces = approach_legs.to_vec();
    base_pieces.append(&mut in_pieces);
    base_pieces.extend(out_pieces);
    let mut base_vias = in_vias;
    base_vias.extend(out_vias);
    if backend
        .validate_pieces(
            request,
            &base_pieces,
            &base_vias,
            virtual_tolerance,
            false,
            &[],
            checked_points,
            2.0,
            &[],
            &[],
        )?
        .is_some()
    {
        return Ok(None);
    }
    let push = (via_half - half).max(0.0);
    let side_in = [
        (pair.positive_net_id, side_in),
        (pair.negative_net_id, -side_in),
    ];
    let side_out = [
        (pair.positive_net_id, side_out),
        (pair.negative_net_id, -side_out),
    ];
    let offset_barrel = |point: Point, direction: Point, side: f64| {
        corridor::add(
            point,
            corridor::mul([-direction[1], direction[0]], side * push),
        )
    };
    let first_offsets = |backend: &mut dyn PairConnectionBackend,
                         net: NetNo|
     -> Result<Vec<(f64, Vec<PairPiece>, PairVia)>, String> {
        let mut options = Vec::new();
        for u in [0.0, 0.5 * pitch, pitch] {
            let Some((trimmed, tip)) = trim_leg(
                &base_pieces,
                net,
                &incoming.layer,
                e_in[index_of(net, pair)],
                u,
            ) else {
                continue;
            };
            let side = side_in.iter().find(|x| x.0 == net).unwrap().1;
            let via_point = offset_barrel(tip, d_in, side);
            for e in [pitch, 1.5 * pitch, 2.0 * pitch, 3.0 * pitch] {
                let Some((mut pieces, tail)) = trim_leg(
                    &trimmed,
                    net,
                    &outgoing.layer,
                    e_out[index_of(net, pair)],
                    e,
                ) else {
                    continue;
                };
                pieces.push(PairPiece {
                    a: tip,
                    b: via_point,
                    width: request.rules.track_width,
                    layer: incoming.layer.clone(),
                    net_id: net,
                });
                pieces.push(PairPiece {
                    a: via_point,
                    b: tail,
                    width: request.rules.track_width,
                    layer: outgoing.layer.clone(),
                    net_id: net,
                });
                let via = PairVia {
                    point: via_point,
                    diameter: request.rules.via_size,
                    drill: request.rules.via_drill,
                    layers: thru_layers(&request.route_layers),
                    net_id: net,
                };
                if backend
                    .validate_pieces(
                        request,
                        &pieces[pieces.len() - 2..],
                        std::slice::from_ref(&via),
                        virtual_tolerance,
                        false,
                        &[],
                        checked_points,
                        2.0,
                        &[],
                        &[],
                    )?
                    .is_none()
                {
                    options.push((u + e, pieces, via));
                }
            }
        }
        options.sort_by(|a, b| a.0.total_cmp(&b.0));
        Ok(options)
    };
    let second_offsets = |backend: &mut dyn PairConnectionBackend,
                          net: NetNo|
     -> Result<Vec<(f64, Vec<PairPiece>, PairVia)>, String> {
        let mut options = Vec::new();
        for u in [0.0, 0.5 * pitch] {
            let Some((trimmed, near)) = trim_leg(
                &base_pieces,
                net,
                &incoming.layer,
                e_in[index_of(net, pair)],
                u,
            ) else {
                continue;
            };
            let side = side_out.iter().find(|x| x.0 == net).unwrap().1;
            for w in [pitch, 1.5 * pitch, 2.0 * pitch, 3.0 * pitch] {
                let Some((mut pieces, tip)) = trim_leg(
                    &trimmed,
                    net,
                    &outgoing.layer,
                    e_out[index_of(net, pair)],
                    w,
                ) else {
                    continue;
                };
                let via_point = offset_barrel(tip, d_out, side);
                pieces.push(PairPiece {
                    a: near,
                    b: via_point,
                    width: request.rules.track_width,
                    layer: incoming.layer.clone(),
                    net_id: net,
                });
                pieces.push(PairPiece {
                    a: via_point,
                    b: tip,
                    width: request.rules.track_width,
                    layer: outgoing.layer.clone(),
                    net_id: net,
                });
                let via = PairVia {
                    point: via_point,
                    diameter: request.rules.via_size,
                    drill: request.rules.via_drill,
                    layers: thru_layers(&request.route_layers),
                    net_id: net,
                };
                if backend
                    .validate_pieces(
                        request,
                        &pieces[pieces.len() - 2..],
                        std::slice::from_ref(&via),
                        virtual_tolerance,
                        false,
                        &[],
                        checked_points,
                        2.0,
                        &[],
                        &[],
                    )?
                    .is_none()
                {
                    options.push((u + w, pieces, via));
                }
            }
        }
        options.sort_by(|a, b| a.0.total_cmp(&b.0));
        Ok(options)
    };
    let mut last_reason: Option<String> = None;
    for (first, second) in [
        (pair.positive_net_id, pair.negative_net_id),
        (pair.negative_net_id, pair.positive_net_id),
    ] {
        let f = first_offsets(backend, first)?;
        let s = second_offsets(backend, second)?;
        let mut combos = Vec::new();
        for x in &f {
            for y in &s {
                combos.push((x.0 + y.0, x, y));
            }
        }
        combos.sort_by(|a, b| a.0.total_cmp(&b.0));
        for (_, x, y) in combos {
            if corridor::distance(x.2.point, y.2.point) < pitch - 1e-6 {
                continue;
            }
            let mut pieces = x.1.clone();
            pieces.extend(y.1.clone());
            let mut vias = base_vias.clone();
            vias.push(x.2.clone());
            vias.push(y.2.clone());
            if pair_intra_ok(
                &pieces,
                &vias,
                pair,
                request.rules.track_width,
                request.rules.via_size,
                source_clearance(request),
            ) {
                return Ok(Some((pieces, vias)));
            }
            last_reason = Some("crossover barrels or legs violate pair spacing".into());
        }
    }
    let _ = last_reason;
    Ok(None)
}

fn index_of(net: NetNo, pair: &PairRungRequest) -> usize {
    usize::from(net == pair.negative_net_id)
}

fn thru_layers(layers: &[String]) -> Vec<String> {
    match (layers.first(), layers.last()) {
        (Some(a), Some(b)) => vec![a.clone(), b.clone()],
        _ => Vec::new(),
    }
}

fn trim_leg(
    pieces: &[PairPiece],
    net: NetNo,
    layer: &str,
    endpoint: Point,
    mut amount: f64,
) -> Option<(Vec<PairPiece>, Point)> {
    let mut output = pieces.to_vec();
    let mut at = endpoint;
    while amount > 1e-9 {
        let mut found = None;
        for (i, piece) in output.iter().enumerate() {
            if piece.net_id != net || piece.layer != layer {
                continue;
            }
            if corridor::distance(piece.b, at) <= 1e-3 {
                found = Some((i, piece.a, true));
                break;
            }
            if corridor::distance(piece.a, at) <= 1e-3 {
                found = Some((i, piece.b, false));
                break;
            }
        }
        let (i, far, at_end) = found?;
        let length = corridor::distance(far, at);
        if length <= amount + 1e-9 {
            output.remove(i);
            amount -= length;
            at = far;
        } else {
            let t = amount / length;
            let next = corridor::add(at, corridor::mul(corridor::sub(far, at), t));
            if at_end {
                output[i].b = next;
            } else {
                output[i].a = next;
            }
            at = next;
            amount = 0.0;
        }
    }
    Some((output, at))
}

fn pair_intra_ok(
    pieces: &[PairPiece],
    vias: &[PairVia],
    pair: &PairRungRequest,
    track: f64,
    via: f64,
    clearance: f64,
) -> bool {
    let mut legs = std::collections::BTreeMap::new();
    for piece in pieces {
        let key = if piece.net_id == pair.positive_net_id {
            "P"
        } else {
            "N"
        };
        legs.entry(key.to_owned())
            .or_insert_with(Vec::new)
            .push(pairs::PairPiece {
                points: vec![piece.a, piece.b],
                layer: piece.layer.clone(),
            });
    }
    let barrels: Vec<_> = vias
        .iter()
        .map(|v| pairs::PairBarrel {
            point: v.point,
            polarity: if v.net_id == pair.positive_net_id {
                "P".into()
            } else {
                "N".into()
            },
        })
        .collect();
    pairs::intra_ok(&legs, &barrels, track, via, clearance, 0.002)
}

fn validate_input(request: &RungRequest, pair: &PairRungRequest) -> Result<(), String> {
    if pair.positive_net_id == pair.negative_net_id {
        return Err("pair connection needs distinct positive and negative net IDs".into());
    }
    if pair
        .start_points
        .iter()
        .chain(pair.end_points.iter())
        .flatten()
        .any(|v| !v.is_finite())
    {
        return Err("pair connection endpoint is nonfinite".into());
    }
    if pair.gap < 0.0 || !pair.gap.is_finite() || !pair.lead_mm.is_finite() || pair.lead_mm < 0.0 {
        return Err("pair connection gap/lead is invalid".into());
    }
    if request.rules.track_width <= 0.0
        || request.rules.via_size <= 0.0
        || request.rules.grid_step <= 0.0
    {
        return Err("pair connection routing dimensions must be positive".into());
    }
    Ok(())
}

fn source_clearance(request: &RungRequest) -> f64 {
    ((request.rules.clearance + HUG_OVER_MM) * 1_000_000.0).round() / 1_000_000.0
}

fn validation_points(request: &RungRequest, pair: &PairRungRequest) -> Vec<Point> {
    let mut points = request.window_points.clone().unwrap_or_default();
    points.extend(pair.start_points);
    points.extend(pair.end_points);
    points
}

#[allow(clippy::too_many_arguments)]
fn connect_pair_prod(
    request: &RungRequest,
    pair: &PairRungRequest,
    backend: &mut impl PairConnectionBackend,
    gap: f64,
    half: f64,
    approach_scale: f64,
    attempt: usize,
    start_connectors: Option<&[PairConnector]>,
    end_connectors: Option<&[PairConnector]>,
    a_given: Option<&PairGivenEnd>,
    b_given: Option<&PairGivenEnd>,
    other_copper: &[PairPiece],
    other_vias: &[PairVia],
) -> Result<Attempt, String> {
    let track = request.rules.track_width;
    let clearance = source_clearance(request);
    let via_size = request.rules.via_size;
    let via_drill = request.rules.via_drill;
    let pitch = via_size + clearance;
    let approach_len = round6(pitch * approach_scale);
    let mut a_points = pair.start_points;
    let mut b_points = pair.end_points;
    let mut a_layer = pair.start_layer.clone();
    let mut b_layer = pair.end_layer.clone();
    let mut a_dir = pair.start_direction;
    let mut b_dir = pair.end_direction;
    let mut connector_pieces = Vec::new();
    let mut connector_vias = Vec::new();
    let mut approach_segments = Vec::new();
    let mut connected_a = false;
    let mut connected_b = false;

    for (is_start, given) in [(true, a_given), (false, b_given)] {
        let Some(given) = given else { continue };
        let points = validation_points(request, pair);
        let band_ends = [
            PairBandEnd {
                point: given.end_points[0],
                layer: given.layer.clone(),
            },
            PairBandEnd {
                point: given.end_points[1],
                layer: given.layer.clone(),
            },
        ];
        if let Some(reason) = backend.validate_pieces(
            request,
            &given.pieces,
            &given.vias,
            0.0,
            true,
            &band_ends,
            &points,
            0.0,
            other_copper,
            other_vias,
        )? {
            return Ok(Attempt::Refused {
                reason: Some(format!("source given connector refused: {reason}")),
                report: None,
                retry_scale: None,
            });
        }
        connector_pieces.extend(given.pieces.clone());
        connector_vias.extend(given.vias.clone());
        if is_start {
            a_points = given.end_points;
            a_layer = given.layer.clone();
            a_dir = Some(given.direction);
            connected_a = true;
        } else {
            b_points = given.end_points;
            b_layer = given.layer.clone();
            b_dir = Some(given.direction);
            connected_b = true;
        }
    }

    if a_dir.is_some()
        && b_dir.is_some()
        && (start_connectors.is_some() || end_connectors.is_some())
    {
        let via_half = envelope_via_half(request, half);
        for (is_start, candidates, tips, direction, layer) in [
            (
                true,
                start_connectors,
                a_points,
                a_dir.unwrap(),
                a_layer.clone(),
            ),
            (
                false,
                end_connectors,
                b_points,
                b_dir.unwrap(),
                b_layer.clone(),
            ),
        ] {
            let Some(candidates) = candidates.filter(|x| !x.is_empty()) else {
                continue;
            };
            let far = candidates[0].point;
            let mut connector: Option<(Vec<PairPiece>, Vec<PairVia>, [Point; 2], Point, String)> =
                None;
            for candidate in candidates {
                let mut geometric = None;
                for forward in [clearance + track, 0.0] {
                    geometric = geo_connector(
                        tips,
                        direction,
                        candidate.direction,
                        &layer,
                        half,
                        track,
                        clearance,
                        pair.positive_net_id,
                        pair.negative_net_id,
                        GEO_CONNECTOR_LEAD_MM,
                        forward,
                    );
                    if geometric.is_none() {
                        break;
                    }
                    let (pieces, ends) = geometric.as_ref().unwrap();
                    let validation_points = validation_points(request, pair);
                    let band_ends = [
                        PairBandEnd {
                            point: ends[0],
                            layer: layer.clone(),
                        },
                        PairBandEnd {
                            point: ends[1],
                            layer: layer.clone(),
                        },
                    ];
                    if backend
                        .validate_pieces(
                            request,
                            pieces,
                            &[],
                            0.0,
                            true,
                            &band_ends,
                            &validation_points,
                            0.0,
                            other_copper,
                            other_vias,
                        )?
                        .is_none()
                    {
                        connector = Some((
                            pieces.clone(),
                            Vec::new(),
                            *ends,
                            candidate.direction,
                            layer.clone(),
                        ));
                        break;
                    }
                }
                if connector.is_some() {
                    break;
                }
            }
            if connector.is_none() {
                if let Some(routed) = routed_connector(
                    request, pair, backend, tips, direction, &layer, far, half, via_half,
                )? {
                    connector = Some(routed);
                }
            }
            if let Some((pieces, vias, ends, end_dir, end_layer)) = connector {
                connector_pieces.extend(pieces);
                connector_vias.extend(vias);
                if is_start {
                    a_points = ends;
                    a_dir = Some(end_dir);
                    a_layer = end_layer;
                    connected_a = true;
                } else {
                    b_points = ends;
                    b_dir = Some(end_dir);
                    b_layer = end_layer;
                    connected_b = true;
                }
            }
        }
    }

    if let (Some(a_direction), Some(b_direction)) = (a_dir, b_dir) {
        if !connected_a {
            let (pieces, ends, direction) = make_approach(
                request,
                pair,
                backend,
                pair.start_points,
                a_direction,
                &pair.start_layer,
                approach_len,
                half,
                track,
                pair.positive_net_id,
                pair.negative_net_id,
                start_connectors,
            )?;
            approach_segments.extend(pieces);
            a_points = ends;
            a_dir = Some(direction);
        }
        if !connected_b {
            let (pieces, ends, direction) = make_approach(
                request,
                pair,
                backend,
                pair.end_points,
                b_direction,
                &pair.end_layer,
                approach_len,
                half,
                track,
                pair.positive_net_id,
                pair.negative_net_id,
                end_connectors,
            )?;
            approach_segments.extend(pieces);
            b_points = ends;
            b_dir = Some(direction);
        }
    }

    let mut window_points = request.window_points.clone().unwrap_or_default();
    window_points.extend([a_points[0], a_points[1], b_points[0], b_points[1]]);
    for (end, direction) in [(a_points, a_dir), (b_points, b_dir)] {
        if let Some(direction) = direction {
            window_points.push(corridor::add(end[0], corridor::mul(direction, 1.5)));
            window_points.push(corridor::add(end[1], corridor::mul(direction, 1.5)));
        }
    }
    let pose = PairPoseRequest {
        start_points: a_points,
        end_points: b_points,
        start_layer: a_layer.clone(),
        end_layer: b_layer.clone(),
        start_direction: a_dir,
        end_direction: b_dir,
        gap,
        margin: request.spec.margin,
        band_slack: request.band_slack,
        window_points,
        preexisting_pieces: connector_pieces
            .clone()
            .into_iter()
            .chain(approach_segments.clone())
            .collect(),
        preexisting_vias: connector_vias.clone(),
        other_copper: other_copper.to_vec(),
        other_vias: other_vias.to_vec(),
        band_restrictions: Vec::new(),
    };
    match backend.route_pair_pose(request, &pose)? {
        PairPoseOutcome::Found(route) => {
            if route.pieces.is_empty() {
                if attempt < 2 && (start_connectors.is_some() || end_connectors.is_some()) {
                    return Ok(Attempt::Refused {
                        reason: Some("coupled pair pose returned no copper".into()),
                        report: None,
                        retry_scale: Some((approach_scale, true)),
                    });
                }
                return Ok(Attempt::Refused {
                    reason: Some("coupled pair pose returned no copper".into()),
                    report: None,
                    retry_scale: None,
                });
            }
            let mut pieces = connector_pieces;
            pieces.extend(approach_segments);
            pieces.extend(route.pieces);
            let mut vias = connector_vias;
            vias.extend(route.vias);
            Ok(Attempt::Found(PairConnectionResult {
                pieces,
                vias,
                start_points: a_points,
                end_points: b_points,
                start_layer: a_layer,
                end_layer: b_layer,
                start_direction: a_dir,
                end_direction: b_dir,
            }))
        }
        PairPoseOutcome::Refused {
            reason,
            report,
            empty,
            frontier_empty,
            escaped_window,
        } => {
            if escaped_window {
                return Ok(Attempt::Refused {
                    reason: Some("coupled pair route escaped its local window".into()),
                    report,
                    retry_scale: None,
                });
            }
            let retry_scale =
                if frontier_empty && attempt < 2 && a_given.is_none() && b_given.is_none() {
                    Some(((attempt + 2) as f64, true))
                } else if empty
                    && attempt < 2
                    && a_given.is_none()
                    && b_given.is_none()
                    && (start_connectors.is_some() || end_connectors.is_some())
                {
                    Some((approach_scale, true))
                } else {
                    None
                };
            let reason = if empty && (start_connectors.is_some() || end_connectors.is_some()) {
                Some(
                    "coupled pair pose returned no copper; source retries without lane connectors"
                        .into(),
                )
            } else {
                reason
            };
            Ok(Attempt::Refused {
                reason,
                report,
                retry_scale,
            })
        }
    }
}

fn round6(value: f64) -> f64 {
    (value * 1_000_000.0).round() / 1_000_000.0
}

fn envelope_via_half(request: &RungRequest, half: f64) -> f64 {
    let rules = &request.rules;
    let clearance = source_clearance(request);
    ((rules.via_size + clearance) / 2.0)
        .max((rules.via_size / 2.0 + clearance + rules.track_width / 2.0 - half) / 0.7071 + 0.005)
}

fn make_approach(
    request: &RungRequest,
    pair: &PairRungRequest,
    backend: &mut impl PairConnectionBackend,
    tips: [Point; 2],
    direction: Point,
    layer: &str,
    length: f64,
    half: f64,
    track: f64,
    positive: NetNo,
    negative: NetNo,
    lane_candidates: Option<&[PairConnector]>,
) -> Result<(Vec<PairPiece>, [Point; 2], Point), String> {
    let midpoint = pairs::mid(tips[0], tips[1]);
    let normal = [-direction[1], direction[0]];
    let side = if corridor::cross(direction, corridor::sub(tips[0], midpoint)) >= 0.0 {
        1.0
    } else {
        -1.0
    };
    let clearance = source_clearance(request);
    let separation = corridor::dot(corridor::sub(tips[0], tips[1]), normal).abs();
    if separation > 2.0 * half + 2.0 * track {
        if let Some(lane) = lane_candidates.and_then(|xs| xs.first()) {
            for forward in [clearance + track, 0.0] {
                if let Some((pieces, ends)) = geo_connector(
                    tips,
                    direction,
                    lane.direction,
                    layer,
                    half,
                    track,
                    clearance,
                    positive,
                    negative,
                    0.0,
                    forward,
                ) {
                    let points = validation_points(request, pair);
                    let band_ends = [
                        PairBandEnd {
                            point: ends[0],
                            layer: layer.into(),
                        },
                        PairBandEnd {
                            point: ends[1],
                            layer: layer.into(),
                        },
                    ];
                    if backend
                        .validate_pieces(
                            request,
                            &pieces,
                            &[],
                            0.0,
                            true,
                            &band_ends,
                            &points,
                            0.0,
                            &[],
                            &[],
                        )?
                        .is_none()
                    {
                        return Ok((pieces, ends, lane.direction));
                    }
                }
            }
        }
        let convergence = (separation - 2.0 * half) / 2.0 / 30_f64.to_radians().tan();
        let mut run = 0.0;
        while run <= 3.0 + 1e-9 {
            let kp = corridor::add(tips[0], corridor::mul(direction, run));
            let kn = corridor::add(tips[1], corridor::mul(direction, run));
            let end_mid = corridor::add(midpoint, corridor::mul(direction, run + convergence));
            let ends = [
                corridor::add(end_mid, corridor::mul(normal, side * half)),
                corridor::sub(end_mid, corridor::mul(normal, side * half)),
            ];
            let mut pieces = Vec::new();
            if run > 1e-6 {
                pieces.push(PairPiece {
                    a: tips[0],
                    b: kp,
                    width: track,
                    layer: layer.into(),
                    net_id: positive,
                });
                pieces.push(PairPiece {
                    a: tips[1],
                    b: kn,
                    width: track,
                    layer: layer.into(),
                    net_id: negative,
                });
            }
            pieces.push(PairPiece {
                a: kp,
                b: ends[0],
                width: track,
                layer: layer.into(),
                net_id: positive,
            });
            pieces.push(PairPiece {
                a: kn,
                b: ends[1],
                width: track,
                layer: layer.into(),
                net_id: negative,
            });
            let mut points = validation_points(request, pair);
            points.extend(tips);
            if backend
                .validate_pieces(
                    request,
                    &pieces,
                    &[],
                    0.0,
                    false,
                    &[],
                    &points,
                    0.0,
                    &[],
                    &[],
                )?
                .is_none()
            {
                return Ok((pieces, ends, direction));
            }
            run += 0.1;
        }
    }
    let far_mid = corridor::add(midpoint, corridor::mul(direction, length));
    let ends = [
        corridor::add(far_mid, corridor::mul(normal, side * half)),
        corridor::sub(far_mid, corridor::mul(normal, side * half)),
    ];
    let mut pieces = Vec::new();
    for (i, net) in [positive, negative].into_iter().enumerate() {
        if corridor::distance(tips[i], ends[i]) > 1e-4 {
            pieces.push(PairPiece {
                a: tips[i],
                b: ends[i],
                width: track,
                layer: layer.into(),
                net_id: net,
            });
        }
    }
    Ok((pieces, ends, direction))
}

fn geo_connector(
    tips: [Point; 2],
    escape: Point,
    heading: Point,
    layer: &str,
    half: f64,
    track: f64,
    clearance: f64,
    positive: NetNo,
    negative: NetNo,
    lead: f64,
    forward: f64,
) -> Option<(Vec<PairPiece>, [Point; 2])> {
    let normal = [-heading[1], heading[0]];
    let dn = corridor::dot(escape, normal);
    if dn.abs() < 0.5 {
        return None;
    }
    let midpoint = pairs::mid(tips[0], tips[1]);
    let along = |p: Point| corridor::dot(corridor::sub(p, midpoint), heading);
    let across = |p: Point| corridor::dot(corridor::sub(p, midpoint), escape);
    if (along(tips[0]) - along(tips[1])).abs() < track + clearance + 0.02 {
        return None;
    }
    let (ahead, behind) = if along(tips[0]) > along(tips[1]) {
        (0, 1)
    } else {
        (1, 0)
    };
    let pitch = 2.0 * half;
    let u_a = across(tips[ahead]);
    let u_b = across(tips[behind]);
    let u_a_turn = u_a.max(u_b - pitch / dn.abs()) + forward;
    let u_b_turn = u_a_turn + pitch / dn.abs();
    let mut turns = [tips[0], tips[1]];
    for (leg, target_u) in [(ahead, u_a_turn), (behind, u_b_turn)] {
        let delta = target_u - across(tips[leg]);
        if delta < -1e-6 {
            return None;
        }
        if delta > 1e-6 {
            turns[leg] = corridor::add(tips[leg], corridor::mul(escape, delta));
        }
    }
    let end_along = along(turns[ahead]).max(along(turns[behind])) + lead;
    let mut ends = [tips[0], tips[1]];
    let mut pieces = Vec::new();
    for leg in 0..2 {
        let run = end_along - along(turns[leg]);
        if run < track {
            return None;
        }
        let end = corridor::add(turns[leg], corridor::mul(heading, run));
        ends[leg] = end;
        let net_id = if leg == 0 { positive } else { negative };
        if corridor::distance(tips[leg], turns[leg]) > 1e-6 {
            pieces.push(PairPiece {
                a: tips[leg],
                b: turns[leg],
                width: track,
                layer: layer.into(),
                net_id,
            });
        }
        pieces.push(PairPiece {
            a: turns[leg],
            b: end,
            width: track,
            layer: layer.into(),
            net_id,
        });
    }
    Some((pieces, ends))
}

fn routed_connector(
    request: &RungRequest,
    pair: &PairRungRequest,
    backend: &mut impl PairConnectionBackend,
    tips: [Point; 2],
    direction: Point,
    layer: &str,
    far: Point,
    half: f64,
    via_half: f64,
) -> Result<Option<(Vec<PairPiece>, Vec<PairVia>, [Point; 2], Point, String)>, String> {
    let track = request.rules.track_width;
    let via_size = request.rules.via_size;
    let via_drill = request.rules.via_drill;
    let clearance = source_clearance(request);
    let midpoint = pairs::mid(tips[0], tips[1]);
    let start = corridor::add(midpoint, corridor::mul(direction, pair.lead_mm));
    let envelope_track = round6(2.0 * half + track);
    let envelope_via = round6(2.0 * via_half + via_size);
    let envelope_drill = round6(envelope_via - (via_size - via_drill));
    let mut layer_order = vec![layer.to_owned()];
    layer_order.extend(
        request
            .route_layers
            .iter()
            .filter(|candidate| candidate.as_str() != layer)
            .cloned(),
    );
    if let Some(band) = request.band.as_deref() {
        layer_order.sort_by_key(|candidate| !band.contains(far, candidate));
    }
    for far_layer in layer_order {
        let envelope = EnvelopeRequest {
            start,
            end: far,
            start_layer: layer.to_owned(),
            end_layer: far_layer.clone(),
            positive_net_id: pair.positive_net_id,
            negative_net_id: pair.negative_net_id,
            envelope_track_width: envelope_track,
            envelope_via_size: envelope_via,
            envelope_via_drill: envelope_drill,
            virtual_size: (track, via_size),
            margin: request.spec.margin,
            band_slack: request.band_slack,
            window_points: vec![tips[0], tips[1], far],
            band_restrictions: Vec::new(),
        };
        let Some(route) = backend.route_envelope(request, &envelope)? else {
            continue;
        };
        let runs: Vec<_> = route
            .runs
            .iter()
            .map(|run| EnvelopeRun {
                layer: run.layer.clone(),
                points: simplify(&run.points, 0.02),
            })
            .filter(|run| run.points.len() >= 2)
            .collect();
        if runs.is_empty() {
            continue;
        }
        let first = &runs[0];
        let last = runs.last().unwrap();
        let start_dir = corridor::unit(corridor::sub(first.points[1], first.points[0]))?;
        let end_dir = corridor::unit(corridor::sub(
            *last.points.last().unwrap(),
            last.points[last.points.len() - 2],
        ))?;
        let normal = [-end_dir[1], end_dir[0]];
        let side = if corridor::cross(start_dir, corridor::sub(tips[0], start)) >= 0.0 {
            1.0
        } else {
            -1.0
        };
        let terminal = *last.points.last().unwrap();
        let ends = [
            corridor::add(terminal, corridor::mul(normal, side * half)),
            corridor::sub(terminal, corridor::mul(normal, side * half)),
        ];
        let Some((pieces, vias)) = super::pair_geometry::split_envelope(
            &runs,
            &route.vias,
            tips[0],
            tips[1],
            ends[0],
            ends[1],
            start,
            terminal,
            half,
            via_half,
            track,
            via_size,
            via_drill,
            pair.positive_net_id,
            pair.negative_net_id,
            &request.route_layers,
            (
                pair.start_layer.as_str(),
                pair.start_layer.as_str(),
                first.layer.as_str(),
                last.layer.as_str(),
            ),
        )?
        else {
            return Ok(None);
        };
        let mut validation_window = validation_points(request, pair);
        validation_window.extend([tips[0], tips[1], far]);
        if backend
            .validate_pieces(
                request,
                &pieces,
                &vias,
                0.0,
                false,
                &[],
                &validation_window,
                0.0,
                &[],
                &[],
            )?
            .is_some()
        {
            return Ok(None);
        }
        let _ = (clearance, route.vias);
        return Ok(Some((pieces, vias, ends, end_dir, last.layer.clone())));
    }
    Ok(None)
}

fn simplify(points: &[Point], tiny: f64) -> Vec<Point> {
    if points.len() < 3 {
        return points.to_vec();
    }
    let mut out = vec![points[0]];
    for (i, &p) in points.iter().enumerate().skip(1) {
        if corridor::distance(*out.last().unwrap(), p) < tiny {
            if i + 1 == points.len() && out.len() == 1 {
                *out.last_mut().unwrap() = p;
            }
            continue;
        }
        out.push(p);
    }
    if corridor::distance(*out.last().unwrap(), *points.last().unwrap()) > 1e-9 {
        *out.last_mut().unwrap() = *points.last().unwrap();
    }
    let mut reduced = vec![out[0]];
    for i in 1..out.len().saturating_sub(1) {
        let d1 =
            corridor::unit(corridor::sub(out[i], *reduced.last().unwrap())).unwrap_or([0., 0.]);
        let d2 = corridor::unit(corridor::sub(out[i + 1], out[i])).unwrap_or([0., 0.]);
        if corridor::cross(d1, d2).abs() < 1e-6 && corridor::dot(d1, d2) > 0.0 {
            continue;
        }
        reduced.push(out[i]);
    }
    reduced.push(*out.last().unwrap());
    reduced
}
