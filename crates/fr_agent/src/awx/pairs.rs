//! AWX caad45cd pairs.py: handedness, fabrication envelope and exact pair connector/crossover geometry.
use super::corridor::{
    add, cross, distance, dot, length, mul, point_segment_distance, sub, unit, Point,
};
use super::whole_context::{ContextPad, RoutingRules};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Pair {
    pub base: String,
    pub p: String,
    pub n: String,
}
pub fn pair_names(names: &[String], _admit_all: bool) -> Vec<Pair> {
    pair_names_filtered(names, &BTreeSet::new(), true)
}
pub fn pair_names_filtered(
    names: &[String],
    only: &BTreeSet<String>,
    admit_all: bool,
) -> Vec<Pair> {
    let mut by: BTreeMap<String, [Option<String>; 2]> = BTreeMap::new();
    for name in names {
        let found = [
            ("_P", 0),
            ("_N", 1),
            ("P", 0),
            ("N", 1),
            ("+", 0),
            ("-", 1),
            ("_p", 0),
            ("_n", 1),
        ]
        .iter()
        .find_map(|&(suffix, polarity)| {
            name.strip_suffix(suffix)
                .filter(|b| !b.is_empty())
                .map(|b| (b, polarity))
        });
        if let Some((base, polarity)) = found {
            by.entry(base.into()).or_insert([None, None])[polarity] = Some(name.clone());
        }
    }
    by.into_iter()
        .filter_map(|(base, [p, n])| {
            if !admit_all && !only.is_empty() && !only.contains(&base) {
                return None;
            }
            Some(Pair { base, p: p?, n: n? })
        })
        .collect()
}
pub fn members(names: &[String], pairs: &[Pair]) -> (Vec<String>, BTreeMap<String, [String; 2]>) {
    let prs: BTreeMap<_, _> = pairs
        .iter()
        .filter(|p| !names.contains(&p.base))
        .map(|p| (p.base.clone(), [p.p.clone(), p.n.clone()]))
        .collect();
    let legs: BTreeMap<_, _> = prs
        .iter()
        .flat_map(|(b, ps)| ps.iter().map(move |n| (n.clone(), b.clone())))
        .collect();
    let mut seen = BTreeSet::new();
    let mut out = Vec::new();
    for n in names {
        if let Some(b) = legs.get(n) {
            if seen.insert(b.clone()) {
                out.push(b.clone());
            }
        } else {
            out.push(n.clone());
        }
    }
    (out, prs)
}
pub fn mid(a: Point, b: Point) -> Point {
    mul(add(a, b), 0.5)
}
pub fn hand_vector(direction: Option<Point>, p: Point, n: Point, arriving: bool) -> i32 {
    let Some(mut d) = direction else { return 0 };
    if arriving {
        d = mul(d, -1.);
    }
    let c = cross(d, sub(p, n));
    if c.abs() < 1e-6 {
        0
    } else if c > 0. {
        1
    } else {
        -1
    }
}
pub fn hand(direction: super::escape_moves::Direction, p: Point, n: Point, arriving: bool) -> i32 {
    let d = match direction {
        super::escape_moves::Direction::Left => [-1., 0.],
        super::escape_moves::Direction::Right => [1., 0.],
        super::escape_moves::Direction::Up => [0., -1.],
        super::escape_moves::Direction::Down => [0., 1.],
    };
    hand_vector(Some(d), p, n, arriving)
}
pub fn pitch(cfg: &RoutingRules) -> f64 {
    cfg.track_width + cfg.pair_gap
}
pub fn corner_buffer(grid: f64) -> f64 {
    grid / 2.
}
pub fn pad_distance(dx: f64, dy: f64, hx: f64, hy: f64, cr: f64) -> f64 {
    let qx = dx.abs() - (hx - cr);
    let qy = dy.abs() - (hy - cr);
    qx.max(0.).hypot(qy.max(0.)) + qx.max(qy).min(0.) - cr
}
pub fn pad_corner_buffer(dx: f64, dy: f64, hx: f64, hy: f64, cr: f64, grid: f64) -> f64 {
    let threshold = if cr <= 0. { grid / 2. } else { 0. };
    if dx.abs() > hx - cr - threshold && dy.abs() > hy - cr - threshold {
        corner_buffer(grid)
    } else {
        0.
    }
}
pub fn turn_straight_steps(cfg: &RoutingRules) -> i64 {
    (cfg.min_turning_radius / cfg.grid_step).ceil() as i64
}
pub fn via_straight_steps(cfg: &RoutingRules) -> i64 {
    (turn_straight_steps(cfg) + 1).max(3)
}
pub fn via_straight(cfg: &RoutingRules, u: Point) -> f64 {
    via_straight_steps(cfg) as f64 * cfg.grid_step / u[0].abs().max(u[1].abs())
}
pub fn via_ring(cfg: &RoutingRules, extra: f64, size: Option<f64>) -> f64 {
    (((size.unwrap_or(cfg.via_size) / 2. + cfg.track_width / 2. + cfg.clearance + extra)
        / cfg.grid_step)
        .ceil()
        + 0.25)
        * cfg.grid_step
}
pub fn probe_len(cfg: &RoutingRules, u: Option<Point>) -> f64 {
    let step = if let Some(u) = u {
        cfg.grid_step / u[0].abs().max(u[1].abs())
    } else {
        cfg.grid_step * 2_f64.sqrt()
    };
    3. * step
}
pub fn handover_setback(_cfg: &RoutingRules) -> f64 {
    0.
}
pub fn end_connector(cfg: &RoutingRules, tips: [Point; 2]) -> f64 {
    cfg.grid_step
        .max((distance(tips[0], tips[1]) / 2. - pitch(cfg) / 2.).abs())
        + cfg.grid_step
}
pub fn end_run(cfg: &RoutingRules, tips: [Point; 2], u: Option<Point>) -> f64 {
    end_connector(cfg, tips) + handover_setback(cfg) + probe_len(cfg, u)
}
pub fn dive_room(cfg: &RoutingRules, tips: [Point; 2], u: Option<Point>) -> f64 {
    end_connector(cfg, tips)
        + handover_setback(cfg)
        + probe_len(cfg, u).max(via_straight(cfg, u.unwrap_or([0.5_f64.sqrt(); 2])))
}
pub fn stair_spread(cfg: &RoutingRules, d: Point) -> f64 {
    let a = d[1].abs().atan2(d[0].abs());
    let a = a.min(std::f64::consts::FRAC_PI_2 - a);
    if a < 1e-9 || std::f64::consts::FRAC_PI_4 - a < 1e-9 {
        return 0.;
    }
    let t = a.tan();
    let r = (turn_straight_steps(cfg) + 1) as f64 * cfg.grid_step;
    let (h, d) = if t <= 0.5 {
        (r * (1. - t) / t, r)
    } else {
        (r, r * t / (1. - t))
    };
    h * d / (h + d) * a.cos()
}
pub fn envelope_via_half(cfg: &RoutingRules, half: f64) -> f64 {
    ((cfg.via_size + cfg.clearance) / 2.)
        .max((cfg.via_size / 2. + cfg.clearance + cfg.track_width / 2. - half) / 0.7071 + 0.005)
}
pub fn dive_offset(cfg: &RoutingRules, half: f64) -> f64 {
    // Mirror diff_pair_routing._pair_via_offset() with the run-level rules
    // available in WholeContext (no per-net impedance overrides are carried).
    let minimum_via_centres =
        (cfg.via_size + cfg.clearance).max(cfg.via_drill + cfg.hole_to_hole_clearance);
    let track_via_clearance =
        (cfg.clearance + cfg.track_width / 2. + cfg.via_size / 2.) * cfg.routing_clearance_margin;
    let production_offset = half
        .max(minimum_via_centres / 2.)
        .max(track_via_clearance - half);
    production_offset.max(envelope_via_half(cfg, half))
}
pub fn under_pad(ball: &ContextPad, pad: &ContextPad, via_size: f64) -> bool {
    let ball_copper: BTreeSet<_> = ball.layers.iter().filter(|l| l.ends_with(".Cu")).collect();
    let pad_copper: BTreeSet<_> = pad.layers.iter().filter(|l| l.ends_with(".Cu")).collect();
    ball.key != pad.key
        && pad.size[0] * pad.size[1] > ball.size[0] * ball.size[1]
        && !ball_copper.is_empty()
        && !pad_copper.is_empty()
        && ball_copper.is_disjoint(&pad_copper)
        && (pad.point[0] - ball.point[0]).abs() <= pad.size[0] / 2. + via_size / 2. - 0.05
        && (pad.point[1] - ball.point[1]).abs() <= pad.size[1] / 2. + via_size / 2. - 0.05
}
pub fn segment_distance(a: Point, b: Point, c: Point, d: Point) -> f64 {
    let den = cross(sub(b, a), sub(d, c));
    if den.abs() > 1e-15 {
        let t = cross(sub(c, a), sub(d, c)) / den;
        let u = cross(sub(c, a), sub(b, a)) / den;
        if (0. ..=1.).contains(&t) && (0. ..=1.).contains(&u) {
            return 0.;
        }
    }
    point_segment_distance(a, c, d)
        .min(point_segment_distance(b, c, d))
        .min(point_segment_distance(c, a, b))
        .min(point_segment_distance(d, a, b))
}
pub fn poly_dist(a: &[Point], b: &[Point]) -> f64 {
    a.windows(2)
        .flat_map(|x| {
            b.windows(2)
                .map(move |y| segment_distance(x[0], x[1], y[0], y[1]))
        })
        .fold(f64::INFINITY, f64::min)
}
pub fn end_legs(
    tips: [Point; 2],
    esc: Point,
    q: Point,
    heading: Point,
    half: f64,
    apart: f64,
    grid: f64,
    reach: f64,
) -> Option<[Vec<Point>; 2]> {
    let e = unit(esc).ok()?;
    let u = unit(heading).ok()?;
    let side = if cross(e, sub(tips[0], mid(tips[0], tips[1]))) >= 0. {
        1.
    } else {
        -1.
    };
    let normal = [-u[1], u[0]];
    let ends = [
        add(q, mul(normal, side * half)),
        sub(q, mul(normal, side * half)),
    ];
    let c45 = std::f64::consts::FRAC_PI_4.cos() - 1e-9;
    let options = |t: Point, end: Point| {
        let mut out = Vec::new();
        for k in 0..=(reach / grid) as usize {
            let knee = add(t, mul(e, k as f64 * grid));
            let Ok(v) = unit(sub(end, knee)) else {
                continue;
            };
            let first = if k > 0 { e } else { v };
            if dot(first, e) <= 1e-9 || (k > 0 && dot(v, e) < c45) || dot(v, u) < c45 {
                continue;
            }
            let pts = if k > 0 {
                vec![t, knee, end]
            } else {
                vec![t, end]
            };
            out.push((
                pts.windows(2).map(|w| distance(w[0], w[1])).sum::<f64>(),
                pts,
            ));
        }
        out.sort_by(|a, b| a.0.total_cmp(&b.0));
        out
    };
    let op = options(tips[0], ends[0]);
    let on = options(tips[1], ends[1]);
    let mut best: Option<(f64, [Vec<Point>; 2])> = None;
    for (lp, p) in op {
        for (ln, n) in &on {
            if best.as_ref().is_some_and(|x| lp + ln >= x.0) {
                break;
            }
            if poly_dist(&p, n) >= apart - 1e-9 {
                best = Some((lp + ln, [p.clone(), n.clone()]));
            }
        }
    }
    best.map(|b| b.1)
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PairPiece {
    pub points: Vec<Point>,
    pub layer: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PairBarrel {
    pub point: Point,
    pub polarity: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Crossover {
    pub at: Point,
    pub heading: Point,
    pub layers: [String; 2],
    pub entry: BTreeMap<String, Point>,
    pub exit: BTreeMap<String, Point>,
    pub poses: [Point; 2],
    pub legs: BTreeMap<String, Vec<PairPiece>>,
    pub vias: Vec<PairBarrel>,
    pub span: [f64; 2],
}
pub fn crossover(
    v: Point,
    u: Point,
    s_in: i32,
    half: f64,
    via_size: f64,
    via_half: f64,
    track: f64,
    clearance: f64,
    grid: f64,
    l1: &str,
    l2: &str,
    first: &str,
    floor: f64,
) -> Option<Crossover> {
    let u = unit(u).ok()?;
    let left = [-u[1], u[0]];
    let to_xy = |x, y| add(v, add(mul(u, x), mul(left, y)));
    let yp = s_in as f64 * half;
    let yf = if first == "P" { yp } else { -yp };
    let sg = if yf > 0. { 1. } else { -1. };
    let vy = sg * via_half;
    let jog = half + via_half;
    let need = (via_size + clearance)
        .max((via_size / 2. + track / 2. + clearance) / std::f64::consts::FRAC_PI_4.sin());
    let dx = (need / grid - 1e-9).ceil() * grid;
    let xa = -(dx / 2. / grid).floor() * grid;
    let xb = xa + dx;
    let st = (vy - yf).abs();
    let x0 = (xa - st).min(xb - jog);
    let x1 = (xa + jog).max(xb + st);
    let step = grid / u[0].abs().max(u[1].abs());
    let pi = ((x0 - floor) / step + 1e-9).floor() * step;
    let po = ((x1 + floor) / step - 1e-9).ceil() * step;
    let xi = pi + floor;
    let xo = po - floor;
    let make = |a: Vec<Point>, b: Vec<Point>| {
        vec![
            PairPiece {
                points: a.iter().map(|p| to_xy(p[0], p[1])).collect(),
                layer: l1.into(),
            },
            PairPiece {
                points: b.iter().map(|p| to_xy(p[0], p[1])).collect(),
                layer: l2.into(),
            },
        ]
    };
    let f = make(
        vec![[xi, yf], [xa - st, yf], [xa, vy]],
        vec![[xa, vy], [xa + jog, -yf], [xo, -yf]],
    );
    let s = make(
        vec![[xi, -yf], [xb - jog, -yf], [xb, vy]],
        vec![[xb, vy], [xb + st, yf], [xo, yf]],
    );
    let (p, n) = if first == "P" { (f, s) } else { (s, f) };
    let legs = BTreeMap::from([("P".into(), p), ("N".into(), n)]);
    let vias = vec![
        PairBarrel {
            point: to_xy(xa, vy),
            polarity: first.into(),
        },
        PairBarrel {
            point: to_xy(xb, vy),
            polarity: if first == "P" { "N" } else { "P" }.into(),
        },
    ];
    if !intra_ok(&legs, &vias, track, via_size, clearance, 0.002) {
        return None;
    }
    Some(Crossover {
        at: v,
        heading: u,
        layers: [l1.into(), l2.into()],
        entry: BTreeMap::from([("P".into(), to_xy(xi, yp)), ("N".into(), to_xy(xi, -yp))]),
        exit: BTreeMap::from([("P".into(), to_xy(xo, -yp)), ("N".into(), to_xy(xo, yp))]),
        poses: [to_xy(pi, 0.), to_xy(po, 0.)],
        legs,
        vias,
        span: [pi, po],
    })
}
pub fn intra_ok(
    legs: &BTreeMap<String, Vec<PairPiece>>,
    vias: &[PairBarrel],
    track: f64,
    via_size: f64,
    clearance: f64,
    tol: f64,
) -> bool {
    let segs = |pol: &str| {
        legs.get(pol)
            .into_iter()
            .flatten()
            .flat_map(|p| {
                p.points
                    .windows(2)
                    .filter(|w| distance(w[0], w[1]) > 1e-9)
                    .map(move |w| (w[0], w[1], p.layer.clone()))
            })
            .collect::<Vec<_>>()
    };
    let p = segs("P");
    let n = segs("N");
    if p.iter().any(|a| {
        n.iter()
            .any(|b| a.2 == b.2 && segment_distance(a.0, a.1, b.0, b.1) < track + clearance - tol)
    }) {
        return false;
    }
    for v in vias {
        let other = if v.polarity == "P" { &n } else { &p };
        if other.iter().any(|s| {
            point_segment_distance(v.point, s.0, s.1) < via_size / 2. + clearance + track / 2. - tol
        }) {
            return false;
        }
    }
    !vias.iter().any(|a| {
        vias.iter().any(|b| {
            a.polarity != b.polarity && distance(a.point, b.point) < via_size + clearance - tol
        })
    })
}
pub fn crossover_shape(cfg: &RoutingRules, u: Point, side: i32) -> Option<(Vec<Point>, [f64; 2])> {
    let u = unit(u).ok()?;
    let c = crossover(
        [0., 0.],
        u,
        side,
        pitch(cfg) / 2.,
        cfg.via_size,
        dive_offset(cfg, pitch(cfg) / 2.),
        cfg.track_width,
        cfg.clearance,
        cfg.grid_step,
        "F.Cu",
        "B.Cu",
        "P",
        handover_setback(cfg),
    )?;
    let step = cfg.grid_step / u[0].abs().max(u[1].abs());
    let barrels = c
        .vias
        .iter()
        .map(|b| [dot(b.point, u), cross(u, b.point)])
        .collect();
    Some((barrels, [-c.span[0] + 3. * step, c.span[1] + 3. * step]))
}
pub fn crossover_room(cfg: &RoutingRules, tips: [Point; 2], u: Option<Point>, end: usize) -> f64 {
    let uu = u.unwrap_or([0.5_f64.sqrt(); 2]);
    let run = crossover_shape(cfg, uu, 1)
        .map(|s| s.1[end])
        .unwrap_or(via_straight(cfg, uu));
    end_connector(cfg, tips) + handover_setback(cfg) + probe_len(cfg, u).max(run)
}
pub fn pose_via_cells(cfg: &RoutingRules, half: f64) -> i64 {
    let tvc =
        (cfg.clearance + cfg.track_width / 2. + cfg.via_size / 2.) * cfg.routing_clearance_margin;
    let spacing = half
        .max((cfg.via_size + cfg.clearance) / 2.)
        .max(tvc - half);
    (spacing / cfg.grid_step + 0.5).floor().max(1.) as i64
}
pub fn dive_barrels(
    site: Point,
    centerline: &[super::whole_context::LayerPiece],
    offset: f64,
) -> Vec<Point> {
    let mut best: Option<((i32, f64), Point, Point)> = None;
    for p in centerline {
        if distance(p.a, p.b) < 1e-6 {
            continue;
        }
        let rank = if distance(p.b, site) < 1e-3 {
            0
        } else if distance(p.a, site) < 1e-3 {
            1
        } else {
            2
        };
        let d = point_segment_distance(site, p.a, p.b);
        if best
            .as_ref()
            .is_none_or(|b| rank < b.0 .0 || (rank == b.0 .0 && d < b.0 .1))
        {
            best = Some(((rank, d), p.a, p.b));
        }
    }
    if let Some((_, a, b)) = best {
        let u = unit(sub(b, a)).unwrap();
        let n = [-u[1], u[0]];
        vec![add(site, mul(n, offset)), sub(site, mul(n, offset))]
    } else {
        vec![site]
    }
}
pub fn offset_polyline(points: &[Point], offset: f64) -> Vec<Point> {
    if points.len() < 2 {
        return points.to_vec();
    }
    let mut out = Vec::new();
    for (i, &p) in points.iter().enumerate() {
        let normal = |d: Point| [-d[1], d[0]];
        if i == 0 {
            let d = unit(sub(points[1], points[0])).unwrap_or([0., 0.]);
            out.push(add(p, mul(normal(d), offset)));
        } else if i + 1 == points.len() {
            let d = unit(sub(points[i], points[i - 1])).unwrap_or([0., 0.]);
            out.push(add(p, mul(normal(d), offset)));
        } else {
            let d1 = unit(sub(p, points[i - 1])).unwrap_or([0., 0.]);
            let d2 = unit(sub(points[i + 1], p)).unwrap_or([0., 0.]);
            let n1 = normal(d1);
            let n2 = normal(d2);
            let sum = add(n1, n2);
            let l = length(sum);
            let cosine = l / 2.;
            if l < 1e-6 || cosine < 0.3 {
                out.push(add(p, mul(n1, offset)));
            } else {
                out.push(add(p, mul(sum, offset / (cosine * l))));
            }
        }
    }
    out
}
pub fn pose_probe_steps(_cfg: &RoutingRules) -> i64 {
    3
}
pub fn pose_turn_units(cfg: &RoutingRules) -> i64 {
    ((cfg.max_turn_angle / 45. + 0.5) as i64).max(8)
}
pub fn pose_turn_over(moves: &[Option<i32>], units: i64) -> Option<usize> {
    let (mut steps, mut t1, mut t2) = (0usize, 0i64, 0i64);
    for (i, m) in moves.iter().enumerate() {
        let Some(m) = m else {
            t1 = 0;
            t2 = 0;
            continue;
        };
        steps += 1;
        let m = *m as i64;
        if m != 0 {
            t1 = if steps % 100 == 0 { m } else { t1 + m };
            t2 = if steps % 100 == 50 { m } else { t2 + m };
            if t1.abs() > units || t2.abs() > units {
                return Some(i);
            }
        }
    }
    None
}
pub fn jog_room(cfg: &RoutingRules) -> f64 {
    via_straight(cfg, [0.5_f64.sqrt(); 2]) + 2. * turn_straight_steps(cfg) as f64 * cfg.grid_step
}
