//! AWX caad45cd: corridor.Spine, octo_hull, build_wrap_spine and braid.ring_spine.
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
pub type Point = [f64; 2];
pub fn add(a: Point, b: Point) -> Point {
    [a[0] + b[0], a[1] + b[1]]
}
pub fn sub(a: Point, b: Point) -> Point {
    [a[0] - b[0], a[1] - b[1]]
}
pub fn mul(a: Point, t: f64) -> Point {
    [a[0] * t, a[1] * t]
}
pub fn dot(a: Point, b: Point) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
pub fn cross(a: Point, b: Point) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
pub fn length(a: Point) -> f64 {
    a[0].hypot(a[1])
}
pub fn distance(a: Point, b: Point) -> f64 {
    length(sub(a, b))
}
pub fn unit(a: Point) -> Result<Point, String> {
    let l = length(a);
    if !l.is_finite() || l < 1e-12 {
        Err("zero/nonfinite direction".into())
    } else {
        Ok(mul(a, 1. / l))
    }
}
pub fn point_segment_distance(p: Point, a: Point, b: Point) -> f64 {
    let d = sub(b, a);
    let l = dot(d, d);
    let t = if l < 1e-18 {
        0.
    } else {
        (dot(sub(p, a), d) / l).clamp(0., 1.)
    };
    distance(p, add(a, mul(d, t)))
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Spine {
    pub pts: Vec<Point>,
    pub lengths: Vec<f64>,
    pub d: Vec<Point>,
    pub nrm: Vec<Point>,
    pub s: Vec<f64>,
    pub length: f64,
    pub turn: Vec<f64>,
}
impl Spine {
    pub fn new(points: &[Point]) -> Result<Self, String> {
        let mut pts = Vec::new();
        for &p in points {
            if p.iter().any(|v| !v.is_finite()) {
                return Err("nonfinite spine point".into());
            }
            if pts.last().is_none_or(|&q| distance(p, q) > 1e-6) {
                pts.push(p);
            }
        }
        if pts.len() < 2 {
            return Err("a spine needs two distinct points".into());
        }
        let lengths: Vec<_> = pts.windows(2).map(|w| distance(w[0], w[1])).collect();
        let d: Vec<_> = pts
            .windows(2)
            .zip(&lengths)
            .map(|(w, &l)| mul(sub(w[1], w[0]), 1. / l))
            .collect();
        let nrm = d.iter().map(|v| [-v[1], v[0]]).collect();
        let mut s = vec![0.];
        for &l in &lengths {
            s.push(s.last().unwrap() + l);
        }
        let length = *s.last().unwrap();
        let turn = d
            .windows(2)
            .map(|w| cross(w[0], w[1]).atan2(dot(w[0], w[1])).to_degrees())
            .collect();
        Ok(Self {
            pts,
            lengths,
            d,
            nrm,
            s,
            length,
            turn,
        })
    }
    pub fn seg_of(&self, s: f64) -> usize {
        self.s
            .partition_point(|v| *v <= s)
            .saturating_sub(1)
            .min(self.lengths.len() - 1)
    }
    pub fn xy(&self, s: f64, o: f64) -> Point {
        let k = self.seg_of(s);
        add(
            add(self.pts[k], mul(self.d[k], s - self.s[k])),
            mul(self.nrm[k], o),
        )
    }
    pub fn corners(&self, min_deg: f64) -> Vec<(usize, f64, f64)> {
        self.turn
            .iter()
            .enumerate()
            .filter(|(_, t)| t.abs() >= min_deg)
            .map(|(j, &t)| (j + 1, self.s[j + 1], t))
            .collect()
    }
    pub fn project_pt(&self, p: Point) -> Point {
        let (mut best, mut bs, mut bo, mut bk, mut bt) = (f64::INFINITY, 0., 0., 0, 0.);
        for k in 0..self.lengths.len() {
            let rel = sub(p, self.pts[k]);
            let t = dot(rel, self.d[k]).clamp(0., self.lengths[k]);
            let foot = add(self.pts[k], mul(self.d[k], t));
            let d2 = dot(sub(p, foot), sub(p, foot));
            if d2 < best - 1e-12 {
                best = d2;
                bs = self.s[k] + t;
                bo = dot(rel, self.nrm[k]);
                bk = k;
                bt = t;
            }
        }
        let end = bt >= self.lengths[bk] - 1e-9 && bk + 1 < self.lengths.len();
        let start = bt <= 1e-9 && bk > 0;
        if end || start {
            let v = if end { bk + 1 } else { bk };
            let turn = self.turn[v - 1];
            let outer = if turn == 0. {
                1.
            } else if turn > 0. {
                -1.
            } else {
                1.
            };
            let o1 = dot(sub(p, self.pts[v - 1]), self.nrm[v - 1]);
            let o2 = dot(sub(p, self.pts[v]), self.nrm[v]);
            bo = outer * o1.abs().max(o2.abs());
            bs = self.s[v];
        }
        [bs, bo]
    }
    pub fn project(&self, points: &[Point]) -> Vec<Point> {
        points.iter().map(|&p| self.project_pt(p)).collect()
    }
    pub fn lane_line(&self, so: &[Point], fixed: &BTreeSet<usize>) -> Vec<[f64; 3]> {
        let mut pts: Vec<(f64, f64, usize)> = so
            .iter()
            .enumerate()
            .map(|(i, p)| (p[0], p[1], i))
            .collect();
        let mut at: BTreeMap<usize, [f64; 3]> = BTreeMap::new();
        for j in 1..self.lengths.len() {
            if self.turn[j - 1].abs() < 1e-6 {
                continue;
            }
            let sj = self.s[j];
            let Some(mut b) = pts.iter().position(|p| p.0 >= sj - 1e-9) else {
                continue;
            };
            if b == 0 {
                continue;
            }
            let mut a = b - 1;
            let mut x = None;
            let oc = pts[a].1
                + (pts[b].1 - pts[a].1) * (sj - pts[a].0) / (pts[b].0 - pts[a].0).max(1e-12);
            if oc * self.turn[j - 1] < 0. {
                let m = mul(
                    add(self.nrm[j - 1], self.nrm[j]),
                    1. / (1. + dot(self.nrm[j - 1], self.nrm[j])).max(1e-6),
                );
                let meet = add(self.pts[j], mul(m, oc));
                let p = self.xy(pts[a].0, pts[a].1);
                let q = self.xy(pts[b].0, pts[b].1);
                if distance(p, meet) + distance(meet, q) <= 2. * distance(p, q) + 1e-9 {
                    at.insert(pts[a].2, [meet[0], meet[1], sj]);
                }
                continue;
            }
            while a >= 1
                && b + 1 < pts.len()
                && self.seg_of(pts[a - 1].0) == j - 1
                && self.seg_of(pts[b + 1].0) == j
            {
                let p0 = self.xy(pts[a - 1].0, pts[a - 1].1);
                let p1 = self.xy(pts[a].0, pts[a].1);
                let q0 = self.xy(pts[b].0, pts[b].1);
                let q1 = self.xy(pts[b + 1].0, pts[b + 1].1);
                let r = sub(p1, p0);
                let q = sub(q1, q0);
                let den = cross(r, q);
                if den.abs() < 1e-12 {
                    break;
                }
                let w = sub(q0, p0);
                let t = cross(w, q) / den;
                let u = cross(w, r) / den;
                if t < 1. - 1e-9 && !fixed.contains(&pts[a].2) {
                    pts.remove(a);
                    a -= 1;
                    b -= 1;
                    continue;
                }
                if u > 1e-9 && !fixed.contains(&pts[b].2) {
                    pts.remove(b);
                    continue;
                }
                if t >= 1. - 1e-9 && u <= 1e-9 {
                    let meet = add(p0, mul(r, t));
                    if distance(p1, meet) + distance(meet, q0) <= 2. * distance(p1, q0) + 1e-9 {
                        x = Some(meet);
                    }
                }
                break;
            }
            if let Some(x) = x {
                at.insert(pts[a].2, [x[0], x[1], sj]);
            }
        }
        let mut out = Vec::new();
        for (s, o, i) in pts {
            let p = self.xy(s, o);
            out.push([p[0], p[1], s]);
            if let Some(x) = at.get(&i) {
                out.push(*x);
            }
        }
        out
    }
    pub fn lane_xy(&self, so: &[Point]) -> Vec<Point> {
        let mut out = Vec::new();
        let push = |out: &mut Vec<Point>, p| {
            if out.last().is_none_or(|&q| distance(p, q) > 1e-6) {
                out.push(p);
            }
        };
        for w in so.windows(2) {
            let (sa, oa, sb, ob) = (w[0][0], w[0][1], w[1][0], w[1][1]);
            push(&mut out, self.xy(sa, oa));
            for j in 1..self.lengths.len() {
                let sj = self.s[j];
                if !(sa + 1e-9 < sj && sj < sb - 1e-9) || self.turn[j - 1].abs() < 1e-6 {
                    continue;
                }
                let o = oa + (ob - oa) * (sj - sa) / (sb - sa).max(1e-12);
                if o.abs() > 1e-9 {
                    let m = mul(
                        add(self.nrm[j - 1], self.nrm[j]),
                        1. / (1. + dot(self.nrm[j - 1], self.nrm[j])).max(1e-6),
                    );
                    let mp = add(self.pts[j], mul(m, o));
                    let a = out.last().copied().unwrap_or(self.xy(sa, oa));
                    let b = self.xy(sb, ob);
                    if dot(sub(mp, a), self.d[j - 1]) > 0. && dot(sub(b, mp), self.d[j]) > 0. {
                        push(&mut out, mp);
                    }
                } else {
                    push(&mut out, self.pts[j]);
                }
            }
            push(&mut out, self.xy(sb, ob));
        }
        if out.len() == 1 {
            out.push(out[0]);
        }
        out
    }
    pub fn extend(&self, back: f64, fwd: f64) -> Result<Self, String> {
        let mut p = self.pts.clone();
        if back > 0. {
            p[0] = sub(p[0], mul(self.d[0], back));
        }
        if fwd > 0. {
            let i = p.len() - 1;
            p[i] = add(p[i], mul(*self.d.last().unwrap(), fwd));
        }
        Self::new(&p)
    }
}
pub fn simplify(points: &[Point], epsilon: f64) -> Vec<Point> {
    if points.len() < 3 {
        return points.to_vec();
    }
    let mut keep = BTreeSet::from([0, points.len() - 1]);
    let mut stack = vec![(0, points.len() - 1)];
    while let Some((a, b)) = stack.pop() {
        let mut best = 0.;
        let mut bi = None;
        for i in a + 1..b {
            let d = point_segment_distance(points[i], points[a], points[b]);
            if d > best {
                best = d;
                bi = Some(i);
            }
        }
        if best > epsilon {
            let i = bi.unwrap();
            keep.insert(i);
            stack.push((a, i));
            stack.push((i, b));
        }
    }
    let mut out = Vec::new();
    for i in keep {
        let p = points[i];
        if out.last().is_none_or(|&q| distance(p, q) > 1e-6) {
            out.push(p);
        }
    }
    out
}
pub fn octo_hull(points: &[Point], margin: f64) -> Result<Vec<Point>, String> {
    if points.is_empty() {
        return Err("empty octilinear hull".into());
    }
    let dirs: Vec<Point> = (0..8)
        .map(|k| {
            let a = k as f64 * std::f64::consts::FRAC_PI_4;
            [a.cos(), a.sin()]
        })
        .collect();
    let h: Vec<_> = dirs
        .iter()
        .map(|&d| {
            points
                .iter()
                .map(|&p| dot(p, d))
                .fold(f64::NEG_INFINITY, f64::max)
                + margin
        })
        .collect();
    let mut out = Vec::new();
    for k in 0..8 {
        let j = (k + 1) % 8;
        let den = cross(dirs[k], dirs[j]);
        out.push([
            (h[k] * dirs[j][1] - h[j] * dirs[k][1]) / den,
            (dirs[k][0] * h[j] - dirs[j][0] * h[k]) / den,
        ]);
    }
    Ok(out)
}
pub fn sweep_round(path: &[Point], center: Point) -> f64 {
    path.windows(2)
        .map(|w| {
            let a = sub(w[0], center);
            let b = sub(w[1], center);
            cross(a, b).atan2(dot(a, b))
        })
        .sum()
}
pub fn build_wrap_spine(
    dest: &[Point],
    stubs: &[Point],
    teeth: &[Point],
    ccw: bool,
    arrive: Point,
    margin: f64,
    reach_deg: f64,
    per_edge: usize,
    extra: &[Point],
    wrap_side: bool,
) -> Result<Spine, String> {
    if dest.is_empty() || stubs.is_empty() || teeth.is_empty() || per_edge == 0 {
        return Err("wrap spine needs destination, stub and tooth points".into());
    }
    let points: Vec<_> = dest.iter().chain(stubs).chain(extra).copied().collect();
    let v = octo_hull(&points, margin)?;
    let mean = |p: &[Point]| mul(p.iter().copied().fold([0., 0.], add), 1. / p.len() as f64);
    let center = mean(dest);
    let ct = mean(teeth);
    let mut ring = Vec::new();
    for k in 0..8 {
        for i in 0..per_edge {
            ring.push(add(
                v[k],
                mul(sub(v[(k + 1) % 8], v[k]), i as f64 / per_edge as f64),
            ));
        }
    }
    let ang = |p: Point| (p[1] - center[1]).atan2(p[0] - center[0]);
    let front = (0..8)
        .min_by(|&a, &b| {
            let f = |k| {
                let edge = sub(v[(k + 1) % 8], v[k]);
                let normal = [edge[1], -edge[0]];
                dot(normal, arrive) / length(normal).max(1e-12)
            };
            f(a).total_cmp(&f(b))
        })
        .unwrap();
    let step = if ccw { 1isize } else { -1 };
    let mut start = if ccw {
        ((front + 1) % 8) * per_edge
    } else {
        front * per_edge
    };
    let swept = |p: Point, a0: f64| {
        let d = ang(p) - a0;
        (if ccw { d } else { -d }).rem_euclid(std::f64::consts::TAU)
    };
    for _ in 0..ring.len() {
        let a0 = ang(ring[start]);
        if stubs
            .iter()
            .all(|&p| swept(p, a0) < 1.5 * std::f64::consts::PI)
        {
            break;
        }
        start = (start as isize - step).rem_euclid(ring.len() as isize) as usize;
    }
    let a0 = ang(ring[start]);
    let end = stubs.iter().map(|&p| swept(p, a0)).fold(0., f64::max) + reach_deg.to_radians();
    let ahead: Vec<_> = (0..ring.len())
        .map(|i| (start as isize + step * i as isize).rem_euclid(ring.len() as isize) as usize)
        .filter(|&i| swept(ring[i], a0) <= end)
        .collect();
    if ahead.is_empty() {
        return Err("no forward wrap hull samples".into());
    }
    let cv = sub(center, ct);
    let bearing = |q: Point| {
        let q = sub(q, ct);
        cross(cv, q).atan2(dot(cv, q))
    };
    let side = if wrap_side {
        if ccw {
            -1.
        } else {
            1.
        }
    } else {
        stubs.iter().map(|&p| bearing(p)).sum()
    };
    let tangent = if side > 0. {
        *ahead
            .iter()
            .max_by(|&&a, &&b| bearing(ring[a]).total_cmp(&bearing(ring[b])))
            .unwrap()
    } else {
        *ahead
            .iter()
            .min_by(|&&a, &&b| bearing(ring[a]).total_cmp(&bearing(ring[b])))
            .unwrap()
    };
    let at = ahead.iter().position(|&i| i == tangent).unwrap();
    let mut path = vec![ct];
    path.extend(ahead[at..].iter().map(|&i| ring[i]));
    Spine::new(&simplify(&path, 0.02))
}
pub fn ring_spine(
    dest: &[Point],
    handoffs: &[Point],
    stubs: &[Point],
    start: Point,
    ccw: bool,
    arrive: Point,
    tails: &[Point],
    pitch: f64,
    wrap_side: bool,
) -> Result<(Spine, Spine), String> {
    let core = build_wrap_spine(
        dest,
        stubs,
        &[start],
        ccw,
        arrive,
        pitch,
        8.,
        40,
        tails,
        wrap_side,
    )?;
    let first = core.pts[0];
    let last = *core.pts.last().unwrap();
    let back = handoffs
        .iter()
        .map(|&t| -dot(sub(t, first), core.d[0]) + 0.3)
        .fold(0.3, f64::max);
    let fwd = stubs
        .iter()
        .map(|&t| dot(sub(t, last), *core.d.last().unwrap()) + 0.3)
        .fold(0.3, f64::max);
    let extended = core.extend(back, fwd)?;
    Ok((core, extended))
}
pub fn line_extent(sp: &Spine, s: f64, rects: &[[f64; 4]], grow: f64) -> Option<Point> {
    let p = sp.xy(s, 0.);
    let n = sp.nrm[sp.seg_of(s)];
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for r in rects {
        let (mut a, mut b) = (f64::NEG_INFINITY, f64::INFINITY);
        for (axis, low, high) in [(0, r[0] - grow, r[2] + grow), (1, r[1] - grow, r[3] + grow)] {
            if n[axis].abs() < 1e-12 {
                if !(low <= p[axis] && p[axis] <= high) {
                    a = 1.;
                    b = -1.;
                }
                continue;
            }
            let t0 = (low - p[axis]) / n[axis];
            let t1 = (high - p[axis]) / n[axis];
            a = a.max(t0.min(t1));
            b = b.min(t0.max(t1));
        }
        if a <= b {
            lo = lo.min(a);
            hi = hi.max(b);
        }
    }
    if lo <= hi {
        Some([lo, hi])
    } else {
        None
    }
}
pub fn round_cover(p: Point, r: Point) -> Vec<[f64; 4]> {
    let c = 0.5_f64.sqrt();
    vec![
        [p[0] - r[0], p[1] - c * r[1], p[0] + r[0], p[1] + c * r[1]],
        [p[0] - c * r[0], p[1] - r[1], p[0] + c * r[0], p[1] + r[1]],
    ]
}
pub fn piece_u(u: f64, trunk: bool, handoff: f64) -> f64 {
    if trunk {
        u.min(handoff + 1e-9)
    } else {
        u.max(handoff + 1e-9)
    }
}
pub fn held_interval(
    intervals: &[Point],
    reference: f64,
    previous: Option<Point>,
    terminal: Option<f64>,
) -> Option<Point> {
    if let Some(&q) = intervals
        .iter()
        .find(|q| q[0] <= reference && reference <= q[1])
    {
        return Some(q);
    }
    let overlap: Vec<_> = intervals
        .iter()
        .copied()
        .filter(|q| previous.is_some_and(|p| q[1].min(p[1]) > q[0].max(p[0])))
        .collect();
    if overlap.len() > 1 {
        if let Some(t) = terminal {
            return overlap.into_iter().min_by(|a, b| {
                (a[0] - t)
                    .max(t - a[1])
                    .max(0.)
                    .total_cmp(&(b[0] - t).max(t - b[1]).max(0.))
            });
        }
    }
    if !overlap.is_empty() {
        let p = previous.unwrap();
        let mut best = overlap[0];
        for q in overlap.into_iter().skip(1) {
            if q[1].min(p[1]) - q[0].max(p[0]) > best[1].min(p[1]) - best[0].max(p[0]) {
                best = q;
            }
        }
        return Some(best);
    }
    intervals.iter().copied().min_by(|a, b| {
        (reference - a[0])
            .abs()
            .min((reference - a[1]).abs())
            .total_cmp(&(reference - b[0]).abs().min((reference - b[1]).abs()))
    })
}
pub fn part_home(
    spines: &BTreeMap<String, Spine>,
    spans: &BTreeMap<String, Point>,
    center: Point,
) -> Option<String> {
    spines
        .keys()
        .min_by(|a, b| {
            let score = |k: &String| {
                let s = spans[k];
                let sp = &spines[k];
                (
                    !(s[0] > 1e-6 && s[1] < sp.length - 1e-6),
                    sp.project_pt(center)[1].abs(),
                )
            };
            let sa = score(a);
            let sb = score(b);
            sa.0.cmp(&sb.0).then(sa.1.total_cmp(&sb.1)).then(a.cmp(b))
        })
        .cloned()
}
pub fn pads_across(spines: &BTreeMap<String, Spine>, pads: [Point; 2]) -> bool {
    let projected: BTreeMap<_, _> = spines
        .iter()
        .map(|(k, s)| (k.clone(), [s.project_pt(pads[0]), s.project_pt(pads[1])]))
        .collect();
    let spans = projected
        .iter()
        .map(|(k, p)| (k.clone(), [p[0][0].min(p[1][0]), p[0][0].max(p[1][0])]))
        .collect();
    let center = mul(add(pads[0], pads[1]), 0.5);
    part_home(spines, &spans, center).is_some_and(|k| {
        let p = projected[&k];
        (p[0][1] - p[1][1]).abs() > (p[0][0] - p[1][0]).abs()
    })
}
pub fn side_carry(from: &Spine, to: &Spine, center: Point, offset_hi: f64, room: f64) -> i32 {
    let s = from.project_pt(center)[0];
    if to.project_pt(from.xy(s, offset_hi + room))[1] >= to.project_pt(center)[1] {
        1
    } else {
        -1
    }
}
