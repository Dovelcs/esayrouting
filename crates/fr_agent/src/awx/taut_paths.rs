//! AWX caad45cd: braid static copper -> taut_fast/topo_strings -> taut_clean.
//! The reference cleanliness policy is assert-only; it does not invent a new homotopy/reseed search.
use super::{
    corridor::Point,
    whole_context::{ContextPad, WholeContext},
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;
const FREEZE: f64 = 0.35;
const STEP: f64 = 0.12;
const COARSE: f64 = 0.48;
const OUT: f64 = 1e-3;
const CROSS_SIN: f64 = 0.5;
const CAP: f64 = 0.05;
const REACH: f64 = 0.9;
const KMAX: usize = 16;
const PUSHES: usize = 6;
const BLOCK: usize = 25;
const MAX_STRING_PTS: usize = 5000;
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Disc {
    pub point: Point,
    pub radius: f64,
    pub name: String,
    pub net_id: i32,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Capsule {
    pub a: Point,
    pub b: Point,
    pub radius: f64,
    pub name: String,
    pub net_id: i32,
}
#[derive(Clone, Debug)]
pub struct Obstacles {
    pub discs: Arc<Vec<Disc>>,
    pub caps: Arc<Vec<Capsule>>,
    near_d: Arc<BTreeMap<(i64, i64), Vec<usize>>>,
    near_c: Arc<BTreeMap<(i64, i64), Vec<usize>>>,
    pub excluded_discs: BTreeSet<usize>,
    pub excluded_caps: BTreeSet<usize>,
    pub own_net: i32,
}
pub type ObstacleSet = Obstacles;
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Violation {
    pub depth: f64,
    pub direction: Point,
}
fn d2(a: Point, b: Point) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    dx * dx + dy * dy
}
fn distance(a: Point, b: Point) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
fn seg_point(a: Point, b: Point, p: Point) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let length = dx * dx + dy * dy;
    let t = if length < 1e-12 {
        0.0
    } else {
        ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length
    }
    .clamp(0.0, 1.0);
    (p[0] - (a[0] + t * dx)).hypot(p[1] - (a[1] + t * dy))
}
fn proper_cross(a: Point, b: Point, c: Point, e: Point) -> bool {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let (ex, ey) = (e[0] - c[0], e[1] - c[1]);
    let den = dx * ey - dy * ex;
    if den.abs() < 1e-12 {
        return false;
    }
    let t = ((c[0] - a[0]) * ey - (c[1] - a[1]) * ex) / den;
    let u = ((c[0] - a[0]) * dy - (c[1] - a[1]) * dx) / den;
    t > 0.0 && t < 1.0 && u > 0.0 && u < 1.0
}
fn seg_seg(a: Point, b: Point, c: Point, e: Point) -> f64 {
    if proper_cross(a, b, c, e) {
        0.0
    } else {
        seg_point(a, b, c)
            .min(seg_point(a, b, e))
            .min(seg_point(c, e, a))
            .min(seg_point(c, e, b))
    }
}
fn cell(p: Point) -> (i64, i64) {
    (p[0].trunc() as i64, p[1].trunc() as i64)
}
impl Obstacles {
    pub fn new(discs: Vec<Disc>, caps: Vec<Capsule>) -> Self {
        let mut dg: BTreeMap<(i64, i64), Vec<usize>> = BTreeMap::new();
        let mut cg: BTreeMap<(i64, i64), Vec<usize>> = BTreeMap::new();
        for (i, d) in discs.iter().enumerate() {
            for x in (d.point[0] - d.radius - 0.3).trunc() as i64
                ..=(d.point[0] + d.radius + 0.3).trunc() as i64
            {
                for y in (d.point[1] - d.radius - 0.3).trunc() as i64
                    ..=(d.point[1] + d.radius + 0.3).trunc() as i64
                {
                    dg.entry((x, y)).or_default().push(i);
                }
            }
        }
        for (i, c) in caps.iter().enumerate() {
            for x in (c.a[0].min(c.b[0]) - c.radius - 0.3).trunc() as i64
                ..=(c.a[0].max(c.b[0]) + c.radius + 0.3).trunc() as i64
            {
                for y in (c.a[1].min(c.b[1]) - c.radius - 0.3).trunc() as i64
                    ..=(c.a[1].max(c.b[1]) + c.radius + 0.3).trunc() as i64
                {
                    cg.entry((x, y)).or_default().push(i);
                }
            }
        }
        let mut centres = BTreeSet::new();
        for &(x, y) in dg.keys().chain(cg.keys()) {
            for dx in [-1, 0, 1] {
                for dy in [-1, 0, 1] {
                    centres.insert((x + dx, y + dy));
                }
            }
        }
        let mut nd = BTreeMap::new();
        let mut nc = BTreeMap::new();
        for (x, y) in centres {
            let mut dd = Vec::new();
            let mut cc = Vec::new();
            let mut seen = BTreeSet::new();
            for dx in [-1, 0, 1] {
                for dy in [-1, 0, 1] {
                    dd.extend(dg.get(&(x + dx, y + dy)).into_iter().flatten().copied());
                    for &i in cg.get(&(x + dx, y + dy)).into_iter().flatten() {
                        if seen.insert(i) {
                            cc.push(i);
                        }
                    }
                }
            }
            if !dd.is_empty() {
                nd.insert((x, y), dd);
            }
            if !cc.is_empty() {
                nc.insert((x, y), cc);
            }
        }
        Self {
            discs: Arc::new(discs),
            caps: Arc::new(caps),
            near_d: Arc::new(nd),
            near_c: Arc::new(nc),
            excluded_discs: BTreeSet::new(),
            excluded_caps: BTreeSet::new(),
            own_net: -2,
        }
    }
    pub fn exclude(&self, net: i32) -> Self {
        let mut out = self.clone();
        out.excluded_discs.extend(
            self.discs
                .iter()
                .enumerate()
                .filter(|(_, d)| d.net_id == net)
                .map(|(i, _)| i),
        );
        out.excluded_caps.extend(
            self.caps
                .iter()
                .enumerate()
                .filter(|(_, c)| c.net_id == net)
                .map(|(i, _)| i),
        );
        out.own_net = net;
        out
    }
    fn near_discs(&self, p: Point) -> impl Iterator<Item = usize> + '_ {
        self.near_d
            .get(&cell(p))
            .into_iter()
            .flatten()
            .copied()
            .filter(|i| !self.excluded_discs.contains(i))
    }
    fn near_caps(&self, p: Point) -> impl Iterator<Item = usize> + '_ {
        self.near_c
            .get(&cell(p))
            .into_iter()
            .flatten()
            .copied()
            .filter(|i| !self.excluded_caps.contains(i))
    }
    pub fn violation(&self, p: Point) -> Option<Violation> {
        self.violation_with_pad(p, 0.0)
    }
    pub fn point_violation(&self, p: Point) -> bool {
        self.violation(p).is_some()
    }
    pub fn point_violation_with_pad(&self, p: Point, pad: f64) -> bool {
        self.violation_with_pad(p, pad).is_some()
    }
    pub fn violation_with_pad(&self, p: Point, pad: f64) -> Option<Violation> {
        let mut best = 0.0;
        let mut winner = None;
        for i in self.near_discs(p) {
            let d = &self.discs[i];
            let length = distance(p, d.point);
            let depth = d.radius + pad - length;
            if depth > best {
                best = depth;
                winner = Some(Violation {
                    depth,
                    direction: if length < 1e-9 {
                        [1.0, 0.0]
                    } else {
                        [(p[0] - d.point[0]) / length, (p[1] - d.point[1]) / length]
                    },
                });
            }
        }
        for i in self.near_caps(p) {
            let c = &self.caps[i];
            let (dx, dy) = (c.b[0] - c.a[0], c.b[1] - c.a[1]);
            let length2 = dx * dx + dy * dy;
            let t = if length2 < 1e-12 {
                0.0
            } else {
                ((p[0] - c.a[0]) * dx + (p[1] - c.a[1]) * dy) / length2
            }
            .clamp(0.0, 1.0);
            let q = [c.a[0] + t * dx, c.a[1] + t * dy];
            let length = distance(p, q);
            let depth = c.radius + pad - length;
            if depth > best {
                best = depth;
                winner = Some(Violation {
                    depth,
                    direction: if length < 1e-9 {
                        if length2 > 1e-12 {
                            [-dy / length2.sqrt(), dx / length2.sqrt()]
                        } else {
                            [1.0, 0.0]
                        }
                    } else {
                        [(p[0] - q[0]) / length, (p[1] - q[1]) / length]
                    },
                });
            }
        }
        winner
    }
    pub fn seg_clear(&self, a: Point, b: Point) -> bool {
        let steps = ((distance(a, b) / 0.9).trunc() as usize).max(1);
        let mut dd = BTreeSet::new();
        let mut cc = BTreeSet::new();
        for k in 0..=steps {
            let t = k as f64 / steps as f64;
            let p = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
            dd.extend(self.near_discs(p));
            cc.extend(self.near_caps(p));
        }
        for i in dd {
            let d = &self.discs[i];
            if seg_point(a, b, d.point) < d.radius {
                return false;
            }
        }
        let (x0, x1, y0, y1) = (
            a[0].min(b[0]),
            a[0].max(b[0]),
            a[1].min(b[1]),
            a[1].max(b[1]),
        );
        for i in cc {
            let c = &self.caps[i];
            if c.a[0].min(c.b[0]) - c.radius > x1
                || c.a[0].max(c.b[0]) + c.radius < x0
                || c.a[1].min(c.b[1]) - c.radius > y1
                || c.a[1].max(c.b[1]) + c.radius < y0
            {
                continue;
            }
            if seg_seg(a, b, c.a, c.b) < c.radius {
                return false;
            }
        }
        true
    }
}
fn pad_obstacles(
    p: &ContextPad,
    layer: &str,
    m: f64,
    discs: &mut Vec<Disc>,
    caps: &mut Vec<Capsule>,
) {
    if !(p.drill > 0.0 || p.layers.iter().any(|l| l == layer || l == "*.Cu")) {
        return;
    }
    let name = format!("{}.{}", p.component_ref, p.pad_number);
    if p.non_plated_hole || p.pad_type == "np_thru_hole" {
        if p.drill > 0.0 {
            discs.push(Disc {
                point: p.point,
                radius: p.drill / 2.0 + m,
                name,
                net_id: p.net_id,
            });
        }
        return;
    }
    let (hx, hy) = (p.size[0] / 2.0, p.size[1] / 2.0);
    let (lo, hi) = (hx.min(hy), hx.max(hy));
    let axis = if hx >= hy {
        [hi - lo, 0.0]
    } else {
        [0.0, hi - lo]
    };
    let a = [p.point[0] - axis[0], p.point[1] - axis[1]];
    let b = [p.point[0] + axis[0], p.point[1] + axis[1]];
    let tilted = p.rect_rotation.abs() > 1e-6;
    if p.shape == "circle" {
        discs.push(Disc {
            point: p.point,
            radius: hi + m,
            name,
            net_id: p.net_id,
        });
    } else if p.shape == "oval" && !tilted {
        caps.push(Capsule {
            a,
            b,
            radius: lo + m,
            name,
            net_id: p.net_id,
        });
    } else if matches!(p.shape.as_str(), "rect" | "roundrect") && !tilted && lo <= 5.5 * m {
        caps.push(Capsule {
            a,
            b,
            radius: lo + m,
            name: name.clone(),
            net_id: p.net_id,
        });
        let (x0, y0, x1, y1) = (
            p.point[0] - hx,
            p.point[1] - hy,
            p.point[0] + hx,
            p.point[1] + hy,
        );
        for (a, b) in [
            ([x0, y0], [x1, y0]),
            ([x1, y0], [x1, y1]),
            ([x1, y1], [x0, y1]),
            ([x0, y1], [x0, y0]),
        ] {
            caps.push(Capsule {
                a,
                b,
                radius: m,
                name: name.clone(),
                net_id: p.net_id,
            });
        }
    } else {
        discs.push(Disc {
            point: p.point,
            radius: hx.hypot(hy) + m,
            name,
            net_id: p.net_id,
        });
    }
}
fn build_base(
    ctx: &WholeContext,
    kids: &BTreeSet<i32>,
    layer: &str,
    m: f64,
    skip_refs: &BTreeSet<String>,
) -> Result<Obstacles, String> {
    if !m.is_finite() || m < 0.0 {
        return Err("invalid AWX obstacle margin".into());
    }
    let mut discs = Vec::new();
    let mut caps = Vec::new();
    for p in &ctx.pads {
        if !skip_refs.contains(&p.component_ref) {
            pad_obstacles(p, layer, m, &mut discs, &mut caps);
        }
    }
    for s in &ctx.base_segments {
        if !kids.contains(&s.net_id) && s.layer == layer {
            caps.push(Capsule {
                a: s.a,
                b: s.b,
                radius: s.width / 2.0 + m,
                name: format!("seg:{}", s.net_id),
                net_id: s.net_id,
            });
        }
    }
    for v in &ctx.base_vias {
        discs.push(Disc {
            point: v.point,
            radius: v.diameter / 2.0 + m,
            name: format!("via:{}", v.net_id),
            net_id: v.net_id,
        });
    }
    Ok(Obstacles::new(discs, caps))
}
pub fn build_obstacles(
    ctx: &WholeContext,
    nid: i32,
    kids: &BTreeSet<i32>,
    layer: &str,
    margin: Option<f64>,
) -> Result<Obstacles, String> {
    build_obstacles_with_skip(ctx, nid, kids, layer, margin, &BTreeSet::new())
}
pub fn build_obstacles_with_skip(
    ctx: &WholeContext,
    nid: i32,
    kids: &BTreeSet<i32>,
    layer: &str,
    margin: Option<f64>,
    skip_refs: &BTreeSet<String>,
) -> Result<Obstacles, String> {
    let mut base_kids = kids.clone();
    if !(kids.contains(&nid) && kids.len() > 1) {
        base_kids.remove(&nid);
    }
    Ok(build_base(
        ctx,
        &base_kids,
        layer,
        margin.unwrap_or(ctx.cfg.clearance + 0.005 + ctx.cfg.track_width / 2.0),
        skip_refs,
    )?
    .exclude(nid))
}

pub fn densify(points: &[Point], step: f64) -> Vec<Point> {
    if points.is_empty() {
        return vec![];
    }
    let mut out = vec![points[0]];
    for &q in &points[1..] {
        let p = *out.last().unwrap();
        let n = ((distance(p, q) / step).trunc() as usize).max(1);
        for k in 1..=n {
            out.push([
                p[0] + (q[0] - p[0]) * k as f64 / n as f64,
                p[1] + (q[1] - p[1]) * k as f64 / n as f64,
            ]);
        }
    }
    out
}
pub fn shortcut(points: &[Point], obs: &Obstacles) -> Vec<Point> {
    shortcut_with(points, |a, b| obs.seg_clear(a, b))
}
fn shortcut_with<F: Fn(Point, Point) -> bool>(points: &[Point], clear: F) -> Vec<Point> {
    if points.is_empty() {
        return vec![];
    }
    let mut out = vec![points[0]];
    let mut i = 0;
    while i + 1 < points.len() {
        let mut j = points.len() - 1;
        while j > i + 1 && !clear(points[i], points[j]) {
            j = if j - i > 8 { (i + j) / 2 } else { j - 1 };
        }
        while j > i + 1 && !clear(points[i], points[j]) {
            j -= 1;
        }
        out.push(points[j]);
        i = j;
    }
    out
}
pub fn relax_slow(src: Point, dst: Point, obs: &Obstacles, rounds: usize) -> (Vec<Point>, usize) {
    let mut points = densify(&[src, dst], STEP);
    let mut used = 0;
    for iteration in 0..rounds {
        used = iteration + 1;
        let mut moved = 0.0;
        for i in 1..points.len().saturating_sub(1) {
            let p = points[i];
            if d2(p, src) < FREEZE * FREEZE || d2(p, dst) < FREEZE * FREEZE {
                continue;
            }
            let mut q = [
                0.5 * p[0] + 0.25 * points[i - 1][0] + 0.25 * points[i + 1][0],
                0.5 * p[1] + 0.25 * points[i - 1][1] + 0.25 * points[i + 1][1],
            ];
            for _ in 0..6 {
                let Some(v) = obs.violation(q) else {
                    break;
                };
                q = [
                    q[0] + v.direction[0] * (v.depth + 0.01),
                    q[1] + v.direction[1] * (v.depth + 0.01),
                ];
            }
            moved += distance(p, q);
            points[i] = q;
        }
        if iteration % 25 == 24 {
            points = densify(&shortcut(&points, obs), STEP);
        }
        if moved < 1e-4 * points.len() as f64 {
            break;
        }
    }
    (densify(&shortcut(&points, obs), STEP), used)
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct CleanDiagnostic {
    pub midfield_cap: usize,
    pub midfield_disc: usize,
    pub end_zone: usize,
    pub shallow: usize,
    pub violations: usize,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RelaxedPath {
    pub points: Vec<Point>,
    pub iterations: usize,
    pub status: String,
    pub reseeds: usize,
    pub diagnostic: CleanDiagnostic,
    pub diverged: bool,
}
pub fn assess(points: Vec<Point>, iterations: usize, obs: &Obstacles) -> RelaxedPath {
    let mut diagnostic = CleanDiagnostic::default();
    if points.len() < 2 {
        return RelaxedPath {
            points,
            iterations,
            status: "clean".into(),
            reseeds: 0,
            diagnostic,
            diverged: false,
        };
    }
    let (src, dst) = (points[0], *points.last().unwrap());
    for pair in points.windows(2) {
        let n = ((distance(pair[0], pair[1]) / 0.25).trunc() as usize).max(1);
        for k in 0..n {
            let t = k as f64 / n as f64;
            let p = [
                pair[0][0] + t * (pair[1][0] - pair[0][0]),
                pair[0][1] + t * (pair[1][1] - pair[0][1]),
            ];
            if d2(p, src) < FREEZE * FREEZE || d2(p, dst) < FREEZE * FREEZE {
                continue;
            }
            if let Some(v) = obs.violation(p) {
                diagnostic.violations += 1;
                if v.depth <= 0.05 {
                    diagnostic.shallow += 1;
                } else if d2(p, src) < 1.5 * 1.5 || d2(p, dst) < 1.5 * 1.5 {
                    diagnostic.end_zone += 1;
                } else {
                    let mut best = 0.0;
                    let mut capsule = false;
                    for i in obs.near_discs(p) {
                        let d = &obs.discs[i];
                        let depth = d.radius - distance(p, d.point);
                        if depth > best {
                            best = depth;
                            capsule = false;
                        }
                    }
                    for i in obs.near_caps(p) {
                        let c = &obs.caps[i];
                        let depth = c.radius - seg_point(c.a, c.b, p);
                        if depth > best {
                            best = depth;
                            capsule = true;
                        }
                    }
                    if capsule {
                        diagnostic.midfield_cap += 1;
                    } else {
                        diagnostic.midfield_disc += 1;
                    }
                }
            }
        }
    }
    let status = if diagnostic.violations == 0 {
        "clean"
    } else if diagnostic.midfield_cap + diagnostic.midfield_disc == 0 {
        "tolerated"
    } else {
        "violating"
    };
    RelaxedPath {
        points,
        iterations,
        status: status.into(),
        reseeds: 0,
        diagnostic,
        diverged: false,
    }
}
pub fn relax_clean(src: Point, dst: Point, obs: &Obstacles) -> RelaxedPath {
    let (points, iterations) = relax_slow(src, dst, obs, 400);
    assess(points, iterations, obs)
}

#[derive(Clone)]
struct StringModel {
    discs: Vec<Disc>,
    caps: Vec<Capsule>,
}
fn string_model(
    base: &Obstacles,
    skip_d: &BTreeSet<usize>,
    skip_c: &BTreeSet<usize>,
) -> StringModel {
    let mut discs: Vec<_> = base
        .discs
        .iter()
        .enumerate()
        .filter(|(i, _)| !skip_d.contains(i))
        .map(|(_, d)| d.clone())
        .collect();
    let mut caps = Vec::new();
    let mut pad_index: BTreeMap<(String, i32), usize> = BTreeMap::new();
    let mut pad_caps: Vec<Vec<&Capsule>> = Vec::new();
    for (i, c) in base.caps.iter().enumerate() {
        if skip_c.contains(&i) {
            continue;
        }
        if c.name.starts_with("seg:") {
            caps.push(c.clone());
        } else {
            let key = (c.name.clone(), c.net_id);
            let index = if let Some(&index) = pad_index.get(&key) {
                index
            } else {
                let index = pad_caps.len();
                pad_index.insert(key, index);
                pad_caps.push(Vec::new());
                index
            };
            pad_caps[index].push(c);
        }
    }
    for cs in pad_caps {
        let x0 = cs
            .iter()
            .flat_map(|c| [c.a[0], c.b[0]])
            .fold(f64::INFINITY, f64::min);
        let x1 = cs
            .iter()
            .flat_map(|c| [c.a[0], c.b[0]])
            .fold(f64::NEG_INFINITY, f64::max);
        let y0 = cs
            .iter()
            .flat_map(|c| [c.a[1], c.b[1]])
            .fold(f64::INFINITY, f64::min);
        let y1 = cs
            .iter()
            .flat_map(|c| [c.a[1], c.b[1]])
            .fold(f64::NEG_INFINITY, f64::max);
        let p = [(x0 + x1) / 2.0, (y0 + y1) / 2.0];
        let radius = cs
            .iter()
            .map(|c| distance(c.a, p).max(distance(c.b, p)) + c.radius)
            .fold(0.0, f64::max);
        discs.push(Disc {
            point: p,
            radius,
            name: cs[0].name.clone(),
            net_id: cs[0].net_id,
        });
    }
    StringModel { discs, caps }
}
#[derive(Clone, Default)]
struct Candidates {
    discs: Vec<usize>,
    caps: Vec<usize>,
}
fn nearest(mut distances: Vec<(f64, usize)>, total: usize) -> Vec<usize> {
    let k = KMAX.min(total);
    if k == 0 {
        return vec![];
    }
    distances.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    distances.into_iter().take(k).map(|(_, i)| i).collect()
}
fn candidates(points: &[Point], own: &[i32], model: &StringModel, extra: f64) -> Vec<Candidates> {
    points
        .iter()
        .enumerate()
        .map(|(i, &p)| {
            let reach = REACH + extra;
            let discs = model
                .discs
                .iter()
                .enumerate()
                .filter(|(_, d)| d.net_id != own[i])
                .filter_map(|(index, d)| {
                    let depth = distance(p, d.point) - d.radius;
                    (depth <= reach).then_some((depth, index))
                })
                .collect();
            let caps = model
                .caps
                .iter()
                .enumerate()
                .filter(|(_, c)| c.net_id != own[i])
                .filter_map(|(index, c)| {
                    let dx = c.b[0] - c.a[0];
                    let dy = c.b[1] - c.a[1];
                    let l2 = if dx * dx + dy * dy > 1e-12 {
                        dx * dx + dy * dy
                    } else {
                        1.0
                    };
                    let t = (((p[0] - c.a[0]) * dx + (p[1] - c.a[1]) * dy) / l2).clamp(0.0, 1.0);
                    let depth = (p[0] - c.a[0] - t * dx).hypot(p[1] - c.a[1] - t * dy) - c.radius;
                    (depth <= reach).then_some((depth, index))
                })
                .collect();
            Candidates {
                discs: nearest(discs, model.discs.len()),
                caps: nearest(caps, model.caps.len()),
            }
        })
        .collect()
}
fn deepest_k(
    p: Point,
    tangent: Point,
    model: &StringModel,
    candidates: &Candidates,
    disc_inflation: f64,
    cap_inflations: Option<&[f64]>,
    dend: f64,
) -> Violation {
    let mut best = Violation {
        depth: f64::NEG_INFINITY,
        direction: [1.0, 0.0],
    };
    for &i in &candidates.discs {
        let d = &model.discs[i];
        let length = distance(p, d.point);
        let depth = d.radius - length + disc_inflation;
        if depth > best.depth {
            best = Violation {
                depth,
                direction: if length < 1e-9 {
                    [1.0, 0.0]
                } else {
                    [(p[0] - d.point[0]) / length, (p[1] - d.point[1]) / length]
                },
            };
        }
    }
    for &i in &candidates.caps {
        let c = &model.caps[i];
        let (dx, dy) = (c.b[0] - c.a[0], c.b[1] - c.a[1]);
        let l2 = dx * dx + dy * dy;
        let l = if l2 > 0.0 { l2 } else { 1.0 }.sqrt();
        let cross = (tangent[0] * dy / l - tangent[1] * dx / l).abs();
        if cross > CROSS_SIN {
            continue;
        }
        let t = if l2 > 1e-12 {
            ((p[0] - c.a[0]) * dx + (p[1] - c.a[1]) * dy) / l2
        } else {
            0.0
        }
        .clamp(0.0, 1.0);
        let q = [c.a[0] + t * dx, c.a[1] + t * dy];
        let length = distance(p, q);
        let inflation = cap_inflations
            .map(|all| {
                let h = all[i];
                (dend - h).clamp(0.0, h)
            })
            .unwrap_or(0.0);
        let depth = c.radius - length + inflation;
        if depth > best.depth {
            best = Violation {
                depth,
                direction: if length < 1e-9 {
                    [-dy / l, dx / l]
                } else {
                    [(p[0] - q[0]) / length, (p[1] - q[1]) / length]
                },
            };
        }
    }
    best
}
fn simplify(points: &[Point], tol: f64) -> Vec<Point> {
    if points.len() < 3 {
        return points.to_vec();
    }
    let mut keep = vec![false; points.len()];
    keep[0] = true;
    keep[points.len() - 1] = true;
    let mut stack = vec![(0, points.len() - 1)];
    while let Some((a, b)) = stack.pop() {
        if b - a < 2 {
            continue;
        }
        let (dx, dy) = (points[b][0] - points[a][0], points[b][1] - points[a][1]);
        let length = dx.hypot(dy);
        let mut best = -1.0;
        let mut index = a + 1;
        for i in a + 1..b {
            let d = if length < 1e-12 {
                distance(points[i], points[a])
            } else {
                ((points[i][0] - points[a][0]) * dy - (points[i][1] - points[a][1]) * dx).abs()
                    / length
            };
            if d > best {
                best = d;
                index = i;
            }
        }
        if best > tol {
            keep[index] = true;
            stack.push((a, index));
            stack.push((index, b));
        }
    }
    points
        .iter()
        .zip(keep)
        .filter_map(|(&p, keep)| keep.then_some(p))
        .collect()
}
fn resample(points: &[Point], obs: &Obstacles, step: f64) -> Vec<Point> {
    densify(&shortcut(&simplify(points, 2e-4), obs), step)
}
fn hausdorff(a: &[Point], b: &[Point]) -> f64 {
    fn directed(a: &[Point], b: &[Point]) -> f64 {
        a.iter()
            .map(|&p| {
                b.iter()
                    .map(|&q| distance(p, q))
                    .fold(f64::INFINITY, f64::min)
            })
            .fold(0.0, f64::max)
    }
    directed(a, b).max(directed(b, a))
}
struct Batch {
    points: Vec<Point>,
    offsets: Vec<usize>,
    prev: Vec<usize>,
    next: Vec<usize>,
    own: Vec<i32>,
    free: Vec<bool>,
}
impl Batch {
    fn pack(polys: &[Vec<Point>], ends: &[[Point; 2]], nids: &[i32]) -> Self {
        let mut out = Self {
            points: Vec::new(),
            offsets: vec![0],
            prev: Vec::new(),
            next: Vec::new(),
            own: Vec::new(),
            free: Vec::new(),
        };
        for (s, poly) in polys.iter().enumerate() {
            let first = out.points.len();
            let last = first + poly.len() - 1;
            for (i, &p) in poly.iter().enumerate() {
                let index = first + i;
                out.points.push(p);
                out.prev.push(index.saturating_sub(1).max(first));
                out.next.push((index + 1).min(last));
                out.own.push(nids[s]);
                out.free.push(
                    index != first
                        && index != last
                        && distance(p, ends[s][0]) >= FREEZE
                        && distance(p, ends[s][1]) >= FREEZE,
                );
            }
            out.offsets.push(out.points.len());
        }
        out
    }
    fn split(&self) -> Vec<Vec<Point>> {
        self.offsets
            .windows(2)
            .map(|w| self.points[w[0]..w[1]].to_vec())
            .collect()
    }
}
fn tangents(points: &[Point], prev: &[usize], next: &[usize]) -> Vec<Point> {
    points
        .iter()
        .enumerate()
        .map(|(i, _)| {
            let d = [
                points[next[i]][0] - points[prev[i]][0],
                points[next[i]][1] - points[prev[i]][1],
            ];
            let length = d[0].hypot(d[1]);
            let safe = if length < 1e-12 { 1.0 } else { length };
            [d[0] / safe, d[1] / safe]
        })
        .collect()
}
fn project(
    points: &mut [Point],
    free: &[bool],
    model: &StringModel,
    candidates: &[Candidates],
    tangents: &[Point],
) {
    let mut active: Vec<_> = free
        .iter()
        .enumerate()
        .filter_map(|(i, &free)| free.then_some(i))
        .collect();
    for _ in 0..PUSHES {
        if active.is_empty() {
            break;
        }
        let mut next = Vec::new();
        for i in active {
            let v = deepest_k(
                points[i],
                tangents[i],
                model,
                &candidates[i],
                0.0,
                None,
                0.0,
            );
            if v.depth > 0.0 {
                points[i][0] += v.direction[0] * (v.depth + OUT);
                points[i][1] += v.direction[1] * (v.depth + OUT);
                next.push(i);
            }
        }
        active = next;
    }
}
fn round_batch(batch: &mut Batch, model: &StringModel, candidates: &[Candidates]) -> Vec<f64> {
    let original = &batch.points;
    let mut next = Vec::with_capacity(original.len());
    for (i, &p) in original.iter().enumerate() {
        let q = [
            0.5 * p[0] + 0.25 * (original[batch.prev[i]][0] + original[batch.next[i]][0]),
            0.5 * p[1] + 0.25 * (original[batch.prev[i]][1] + original[batch.next[i]][1]),
        ];
        let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
        let length = dx.hypot(dy);
        let scale = if length > CAP { CAP / length } else { 1.0 };
        next.push(if batch.free[i] {
            [p[0] + dx * scale, p[1] + dy * scale]
        } else {
            p
        });
    }
    let ts = tangents(&next, &batch.prev, &batch.next);
    project(&mut next, &batch.free, model, candidates, &ts);
    let moved = next
        .iter()
        .zip(original)
        .map(|(&a, &b)| distance(a, b))
        .collect();
    batch.points = next;
    moved
}
fn level_many(
    mut polys: Vec<Vec<Point>>,
    ends: &[[Point; 2]],
    nids: &[i32],
    obstacles: &[Obstacles],
    model: &StringModel,
    rounds: usize,
    tol: f64,
    step: f64,
) -> (Vec<Vec<Point>>, Vec<usize>, Vec<bool>) {
    let n = polys.len();
    let mut used = vec![0; n];
    let mut previous: Vec<Option<Vec<Point>>> = vec![None; n];
    let mut active: Vec<_> = (0..n).collect();
    let mut diverged = vec![false; n];
    while !active.is_empty() {
        let cur: Vec<_> = active.iter().map(|&s| polys[s].clone()).collect();
        let end: Vec<_> = active.iter().map(|&s| ends[s]).collect();
        let ids: Vec<_> = active.iter().map(|&s| nids[s]).collect();
        let mut batch = Batch::pack(&cur, &end, &ids);
        let mut nearby = candidates(&batch.points, &batch.own, model, 0.0);
        let mut anchor = batch.points.clone();
        let mut moved = vec![0.0; batch.points.len()];
        for _ in 0..BLOCK {
            moved = round_batch(&mut batch, model, &nearby);
            for &s in &active {
                used[s] += 1;
            }
            if batch
                .points
                .iter()
                .zip(&anchor)
                .map(|(&a, &b)| distance(a, b))
                .fold(0.0, f64::max)
                > 0.5
            {
                nearby = candidates(&batch.points, &batch.own, model, 0.0);
                anchor = batch.points.clone();
            }
            if moved.iter().copied().fold(0.0, f64::max) < 1e-6 {
                break;
            }
        }
        let parts = batch.split();
        let mut next = Vec::new();
        for (k, &s) in active.iter().enumerate() {
            let resampled = resample(&parts[k], &obstacles[s], step);
            let runaway = resampled.len() > MAX_STRING_PTS;
            if runaway {
                diverged[s] = true;
                eprintln!("taut: string {s} DIVERGED ({} points at step {step}) -- frozen where it stands",resampled.len());
            }
            let movement = moved[batch.offsets[k]..batch.offsets[k + 1]]
                .iter()
                .copied()
                .fold(0.0, f64::max);
            let settled = runaway
                || previous[s]
                    .as_ref()
                    .is_some_and(|p| !p.is_empty() && hausdorff(&resampled, p) < tol)
                || used[s] >= rounds
                || movement < 1e-6;
            previous[s] = Some(resampled.clone());
            polys[s] = resampled;
            if !settled {
                next.push(s);
            }
        }
        active = next;
    }
    (polys, used, diverged)
}
/// Same-base, per-own-net masked coarse/fine Jacobi relaxation from taut_fast.relax_many.
pub fn relax_many(
    items: &[(Point, Point, Obstacles)],
    rounds: usize,
    start: Option<&[Option<Vec<Point>>]>,
) -> Result<Vec<RelaxedPath>, String> {
    if items.is_empty() {
        return Ok(vec![]);
    }
    let base = &items[0].2;
    if items.iter().any(|item| {
        !Arc::ptr_eq(&base.discs, &item.2.discs) || !Arc::ptr_eq(&base.caps, &item.2.caps)
    }) {
        return Err("taut batch models must share their actual static obstacle base".into());
    }
    let own: BTreeSet<_> = items.iter().map(|item| item.2.own_net).collect();
    let shared_own = own.len() == 1;
    let mut skip_d = base.excluded_discs.clone();
    let mut skip_c = base.excluded_caps.clone();
    for item in &items[1..] {
        skip_d = skip_d
            .intersection(&item.2.excluded_discs)
            .copied()
            .collect();
        skip_c = skip_c
            .intersection(&item.2.excluded_caps)
            .copied()
            .collect();
    }
    if shared_own {
        skip_d.retain(|&i| !own.contains(&base.discs[i].net_id));
        skip_c.retain(|&i| !own.contains(&base.caps[i].net_id));
    }
    let model = string_model(base, &skip_d, &skip_c);
    let ends: Vec<_> = items.iter().map(|item| [item.0, item.1]).collect();
    let nids: Vec<_> = items.iter().map(|item| item.2.own_net).collect();
    let obss: Vec<_> = items.iter().map(|item| item.2.clone()).collect();
    let polys: Vec<_> = items
        .iter()
        .enumerate()
        .map(|(i, item)| {
            densify(
                start
                    .and_then(|s| s.get(i))
                    .and_then(|s| s.as_deref())
                    .unwrap_or(&[item.0, item.1]),
                COARSE,
            )
        })
        .collect();
    let (polys, u1, d1) = level_many(polys, &ends, &nids, &obss, &model, rounds, 0.02, COARSE);
    let polys = polys.iter().map(|p| densify(p, STEP)).collect();
    let (polys, u2, d2) = level_many(polys, &ends, &nids, &obss, &model, rounds, 0.02, STEP);
    let mut result = Vec::new();
    for (s, poly) in polys.into_iter().enumerate() {
        let resampled = resample(&poly, &obss[s], STEP);
        let mut batch = Batch::pack(&[resampled], &[ends[s]], &[nids[s]]);
        let candidates = candidates(&batch.points, &batch.own, &model, 0.0);
        let ts = tangents(&batch.points, &batch.prev, &batch.next);
        project(&mut batch.points, &batch.free, &model, &candidates, &ts);
        let mut assessed = assess(batch.points, u1[s] + u2[s], &obss[s]);
        assessed.diverged = d1[s] || d2[s];
        result.push(assessed);
    }
    Ok(result)
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TautOptions {
    pub fast: bool,
    pub rounds: usize,
    pub margin: Option<f64>,
}
impl Default for TautOptions {
    fn default() -> Self {
        Self {
            fast: true,
            rounds: 400,
            margin: None,
        }
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PathsResult {
    pub paths: BTreeMap<String, Vec<Point>>,
    pub diagnostics: BTreeMap<String, RelaxedPath>,
}
pub fn paths(
    ctx: &WholeContext,
    bundle_layer: &str,
) -> Result<BTreeMap<String, Vec<Point>>, String> {
    Ok(paths_with_options(ctx, bundle_layer, &TautOptions::default())?.paths)
}
pub fn paths_with_options(
    ctx: &WholeContext,
    bundle_layer: &str,
    options: &TautOptions,
) -> Result<PathsResult, String> {
    ctx.cfg.validate()?;
    let mut kids = BTreeSet::new();
    for name in &ctx.lanes {
        kids.insert(
            *ctx.byname
                .get(name)
                .ok_or_else(|| format!("taut lane {name} missing native identity"))?,
        );
    }
    for legs in ctx.pairs.values() {
        for leg in legs {
            kids.insert(
                *ctx.byname
                    .get(leg)
                    .ok_or_else(|| format!("taut pair leg {leg} missing native identity"))?,
            );
        }
    }
    let base = build_base(
        ctx,
        &kids,
        bundle_layer,
        options
            .margin
            .unwrap_or(ctx.cfg.clearance + 0.005 + ctx.cfg.track_width / 2.0),
        &BTreeSet::new(),
    )?;
    let mut items = Vec::new();
    for name in &ctx.lanes {
        let ends = ctx
            .ends
            .get(name)
            .ok_or_else(|| format!("taut lane {name} has no measured/planned ends"))?;
        let own = ctx.byname[name];
        items.push((ends.tooth, ends.berth, base.exclude(own)));
    }
    let outputs = if options.fast {
        relax_many(&items, options.rounds, None)?
    } else {
        items
            .iter()
            .map(|(src, dst, obs)| {
                let (p, it) = relax_slow(*src, *dst, obs, options.rounds);
                assess(p, it, obs)
            })
            .collect()
    };
    let mut paths = BTreeMap::new();
    let mut diagnostics = BTreeMap::new();
    for (name, path) in ctx.lanes.iter().zip(outputs) {
        if path.status == "violating" {
            eprintln!(
                "TAUT VIOLATING: {name} -- {:?} (assert-only)",
                path.diagnostic
            );
        } else if path.status == "tolerated" {
            eprintln!("TAUT tolerated: {name} -- {:?}", path.diagnostic);
        }
        paths.insert(name.clone(), path.points.clone());
        diagnostics.insert(name.clone(), path);
    }
    Ok(PathsResult { paths, diagnostics })
}
