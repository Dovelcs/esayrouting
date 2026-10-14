//! Geometry helpers ported from `awx/pairs.py` at the pinned source revision.
//! Coordinates and dimensions are in millimetres.

use super::{
    corridor::Point,
    pair_connection::{EnvelopeRun, PairPiece, PairVia},
};
use fr_engine::ids::NetNo;

fn sub(a: Point, b: Point) -> Point {
    [b[0] - a[0], b[1] - a[1]]
}
fn len(v: Point) -> f64 {
    v[0].hypot(v[1])
}
fn unit(a: Point, b: Point) -> Point {
    let v = sub(a, b);
    let n = len(v);
    if n < 1e-12 {
        [0.0, 0.0]
    } else {
        [v[0] / n, v[1] / n]
    }
}
fn left(d: Point) -> Point {
    [-d[1], d[0]]
}
fn cross(a: Point, b: Point) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

/// Port of `pairs.offset_polyline`: left offset, mitered at vertices.
pub fn offset_polyline(points: &[Point], h: f64) -> Vec<Point> {
    if points.len() < 2 {
        return points.to_vec();
    }
    let mut out = Vec::with_capacity(points.len());
    for i in 0..points.len() {
        let p = points[i];
        let n = if i == 0 {
            left(unit(points[0], points[1]))
        } else if i + 1 == points.len() {
            left(unit(points[i - 1], p))
        } else {
            let n1 = left(unit(points[i - 1], p));
            let n2 = left(unit(p, points[i + 1]));
            let b = [n1[0] + n2[0], n1[1] + n2[1]];
            let bl = len(b);
            let ch = bl / 2.0;
            if bl < 1e-6 || ch < 0.3 {
                out.push([p[0] + n1[0] * h, p[1] + n1[1] * h]);
                continue;
            }
            let m = h / ch;
            let miter = [b[0] / bl * m, b[1] / bl * m];
            out.push([p[0] + miter[0], p[1] + miter[1]]);
            continue;
        };
        out.push([p[0] + n[0] * h, p[1] + n[1] * h]);
    }
    out
}

/// Port of `pairs._chain`, ordering directed or reversed pieces from `start`.
pub fn chain_pieces(
    pieces: &[PairPiece],
    start: Point,
    tol: f64,
) -> (Vec<EnvelopeRun>, Vec<usize>) {
    let mut left_indices: Vec<usize> = (0..pieces.len()).collect();
    let mut runs: Vec<EnvelopeRun> = Vec::new();
    let mut cur = start;
    while !left_indices.is_empty() {
        let mut found = None;
        'search: for (slot, &idx) in left_indices.iter().enumerate() {
            let p = &pieces[idx];
            for (a, b) in [(p.a, p.b), (p.b, p.a)] {
                if len(sub(cur, a)) <= tol {
                    found = Some((slot, idx, a, b, p.layer.clone()));
                    break 'search;
                }
            }
        }
        let Some((slot, _idx, a, b, layer)) = found else {
            break;
        };
        left_indices.remove(slot);
        if runs.last().is_none_or(|r| r.layer != layer) {
            runs.push(EnvelopeRun {
                layer,
                points: vec![a],
            });
        }
        runs.last_mut().unwrap().points.push(b);
        cur = b;
    }
    (runs, left_indices)
}

fn simplify(points: &[Point], tiny: f64) -> Vec<Point> {
    let pts = points;
    if pts.len() < 3 {
        return pts.to_vec();
    }
    let mut out = vec![pts[0]];
    for (i, &p) in pts.iter().enumerate().skip(1) {
        if len(sub(*out.last().unwrap(), p)) < tiny {
            if i + 1 == pts.len() && out.len() == 1 {
                *out.last_mut().unwrap() = p;
            }
            continue;
        }
        out.push(p);
    }
    if len(sub(*out.last().unwrap(), *pts.last().unwrap())) > 1e-9 {
        *out.last_mut().unwrap() = *pts.last().unwrap();
    }
    let mut res = vec![out[0]];
    for i in 1..out.len() - 1 {
        let d1 = unit(*res.last().unwrap(), out[i]);
        let d2 = unit(out[i], out[i + 1]);
        if cross(d1, d2).abs() < 1e-6 && d1[0] * d2[0] + d1[1] * d2[1] > 0.0 {
            continue;
        }
        res.push(out[i]);
    }
    res.push(*out.last().unwrap());
    res
}

fn cut_front(poly: &[Point], mut distance: f64) -> Vec<Point> {
    let mut pts = poly.to_vec();
    while pts.len() >= 2 && distance > 0.0 {
        let d = len(sub(pts[0], pts[1]));
        if d <= distance {
            distance -= d;
            pts.remove(0);
        } else {
            let u = unit(pts[0], pts[1]);
            pts[0] = [pts[0][0] + u[0] * distance, pts[0][1] + u[1] * distance];
            distance = 0.0;
        }
    }
    pts
}

fn cut_back(poly: &[Point], distance: f64) -> Vec<Point> {
    let mut rev = poly.to_vec();
    rev.reverse();
    let mut out = cut_front(&rev, distance);
    out.reverse();
    out
}

fn thru(layers: &[String]) -> Vec<String> {
    match (layers.first(), layers.last()) {
        (Some(a), Some(b)) => vec![a.clone(), b.clone()],
        _ => Vec::new(),
    }
}

fn source_via_key(via: &PairVia) -> Result<(u64, u64, NetNo), String> {
    let x = super::numeric::round_digits(via.point[0], 4)?;
    let y = super::numeric::round_digits(via.point[1], 4)?;
    // Python float equality treats -0.0 and +0.0 as equal.
    Ok((
        if x == 0.0 {
            0.0_f64.to_bits()
        } else {
            x.to_bits()
        },
        if y == 0.0 {
            0.0_f64.to_bits()
        } else {
            y.to_bits()
        },
        via.net_id,
    ))
}

/// Port of `pairs.split_envelope` for the source's ordinary (`cross=false`) split.
#[allow(clippy::too_many_arguments)]
pub fn split_envelope(
    runs: &[EnvelopeRun],
    _vias: &[PairVia],
    p_tip: Point,
    n_tip: Point,
    p_end: Point,
    n_end: Point,
    a_pt: Point,
    b_pt: Point,
    half: f64,
    via_half: f64,
    track: f64,
    via_size: f64,
    via_drill: f64,
    positive: NetNo,
    negative: NetNo,
    layers: &[String],
    tip_layers: (&str, &str, &str, &str),
) -> Result<Option<(Vec<PairPiece>, Vec<PairVia>)>, String> {
    if runs.is_empty() {
        return Ok(None);
    }
    let runs: Vec<EnvelopeRun> = runs
        .iter()
        .map(|r| EnvelopeRun {
            layer: r.layer.clone(),
            points: simplify(&r.points, 0.02),
        })
        .filter(|r| r.points.len() >= 2)
        .collect();
    if runs.is_empty() {
        return Ok(None);
    }
    let d0 = unit(runs[0].points[0], runs[0].points[1]);
    let sp = if cross(d0, sub(a_pt, p_tip)) >= 0.0 {
        1.0
    } else {
        -1.0
    };
    let last = runs.last().unwrap();
    let dl = unit(
        last.points[last.points.len() - 2],
        *last.points.last().unwrap(),
    );
    let sp_end = if cross(dl, sub(b_pt, p_end)) >= 0.0 {
        1.0
    } else {
        -1.0
    };
    if sp_end != sp {
        return Ok(None);
    }

    let mut normals = Vec::new();
    let mut dirs = Vec::new();
    for k in 0..runs.len().saturating_sub(1) {
        let pts = &runs[k].points;
        let d = unit(pts[pts.len() - 2], *pts.last().unwrap());
        dirs.push(d);
        normals.push(left(d));
    }
    let tl = [tip_layers.0, tip_layers.1, tip_layers.2, tip_layers.3];
    let p_tip_layer = tl[0];
    let n_tip_layer = tl[1];
    let p_end_layer = tl[2];
    let n_end_layer = tl[3];
    let mut out_pieces = Vec::new();
    let mut out_vias = Vec::new();
    for (sign, nid, tip, end, tip_layer, end_layer) in [
        (sp, positive, p_tip, p_end, p_tip_layer, p_end_layer),
        (-sp, negative, n_tip, n_end, n_tip_layer, n_end_layer),
    ] {
        if tip_layer != runs[0].layer {
            out_vias.push(PairVia {
                point: tip,
                diameter: via_size,
                drill: via_drill,
                layers: thru(layers),
                net_id: nid,
            });
        }
        if end_layer != last.layer {
            out_vias.push(PairVia {
                point: end,
                diameter: via_size,
                drill: via_drill,
                layers: thru(layers),
                net_id: nid,
            });
        }
        for (k, run) in runs.iter().enumerate() {
            let run_len: f64 = run.points.windows(2).map(|w| len(sub(w[0], w[1]))).sum();
            let jog = 0.15_f64.min(run_len / 4.0);
            let mut core = if run_len <= 3.0 * jog + 1e-9 || run_len < 0.06 {
                Vec::new()
            } else {
                offset_polyline(&run.points, sign * half)
            };
            if !core.is_empty() {
                if k > 0 {
                    core = cut_front(&core, 2.0 * jog);
                }
                if k + 1 < runs.len() {
                    core = cut_back(&core, jog);
                }
            }
            let mut poly = Vec::new();
            if k > 0 {
                let v = run.points[0];
                let n = normals[k - 1];
                let d_in = dirs[k - 1];
                let vp = [v[0] + sign * n[0] * via_half, v[1] + sign * n[1] * via_half];
                poly.push(vp);
                if !core.is_empty() {
                    let d_out = unit(run.points[0], run.points[1]);
                    let inner = sign * cross(d_in, d_out) > 1e-9;
                    if !inner {
                        poly.push([vp[0] + d_in[0] * jog, vp[1] + d_in[1] * jog]);
                    }
                }
            }
            poly.extend(core);
            if k + 1 < runs.len() {
                let v = *run.points.last().unwrap();
                let n = normals[k];
                let vp = [v[0] + sign * n[0] * via_half, v[1] + sign * n[1] * via_half];
                poly.push(vp);
                out_vias.push(PairVia {
                    point: vp,
                    diameter: via_size,
                    drill: via_drill,
                    layers: thru(layers),
                    net_id: nid,
                });
            }
            if k == 0 {
                poly.insert(0, tip);
            }
            if k + 1 == runs.len() {
                poly.push(end);
            }
            for w in poly.windows(2) {
                if len(sub(w[0], w[1])) >= 1e-6 {
                    out_pieces.push(PairPiece {
                        a: w[0],
                        b: w[1],
                        width: track,
                        layer: run.layer.clone(),
                        net_id: nid,
                    });
                }
            }
        }
    }
    let mut seen = std::collections::HashSet::new();
    let mut unique_vias = Vec::with_capacity(out_vias.len());
    for via in out_vias {
        if seen.insert(source_via_key(&via)?) {
            unique_vias.push(via);
        }
    }
    Ok(Some((out_pieces, unique_vias)))
}
