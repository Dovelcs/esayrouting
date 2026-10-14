//! AWX `taut_fast.relax_spine` and `corridor.RampedObstacles` port.
//!
//! This is the corridor-axis relaxation, with per-obstacle ramped inflation.
//! It is separate from lane-string relaxation because tube crossings are
//! transparent to the axis and the parts' inflation follows end-zone flow.

use super::{corridor::{self, Point}, taut_paths::{self, Obstacles, Violation}};

const FREEZE: f64 = 0.35;
const STEP: f64 = 0.12;
const COARSE: f64 = 4.0 * STEP;
const OUT: f64 = 1e-3;
const CROSS_SIN: f64 = 0.5;
const CAP: f64 = 0.05;
const REACH: f64 = 0.9;
const KMAX: usize = 16;
const PUSHES: usize = 6;
const BLOCK: usize = 25;

#[derive(Debug, Clone, Copy)]
pub struct ExtraTube {
    pub a: Point,
    pub b: Point,
    pub radius: f64,
    /// Each tube ramps by its own inflation, defaulting to bundle half-width.
    pub inflation: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct RampZone {
    pub center: Point,
    /// Unit flow into the corridor from this end.
    pub flow: Point,
    pub radius: f64,
}

pub struct RampedObstacles<'a> {
    pub base: &'a Obstacles,
    pub ends: [Point; 2],
    pub h: f64,
    pub extra: Vec<ExtraTube>,
    pub zones: Option<[RampZone; 2]>,
}

impl<'a> RampedObstacles<'a> {
    pub fn new(
        base: &'a Obstacles,
        ends: [Point; 2],
        h: f64,
        extra: &[ExtraTube],
        zones: Option<[RampZone; 2]>,
    ) -> Self {
        Self { base, ends, h, extra: extra.to_vec(), zones }
    }

    fn d_end(&self, p: Point) -> f64 {
        distance(p, self.ends[0]).min(distance(p, self.ends[1]))
    }

    fn inflation(d: f64, amount: f64) -> f64 {
        (d - amount).clamp(0.0, amount)
    }

    fn part_inflation(&self, p: Point) -> f64 {
        if let Some(zones) = self.zones {
            zones.iter().map(|z| dot(sub(p, z.center), z.flow) - z.radius)
                .fold(f64::INFINITY, f64::min).clamp(0.0, self.h)
        } else {
            Self::inflation(self.d_end(p), self.h)
        }
    }

    /// Return the deepest part or tube violation, preserving base-obstacle
    /// exclusions and its spatial index through `Obstacles`.
    pub fn point_violation(&self, p: Point) -> Option<Violation> {
        let mut best = self.base.violation_with_pad(p, self.part_inflation(p));
        let d = self.d_end(p);
        for tube in &self.extra {
            let axis = sub(tube.b, tube.a);
            let l2 = dot(axis, axis);
            let t = if l2 < 1e-12 { 0.0 } else { (dot(sub(p, tube.a), axis) / l2).clamp(0.0, 1.0) };
            let closest = add(tube.a, mul(axis, t));
            let delta = sub(p, closest);
            let dist = length(delta);
            let depth = tube.radius + Self::inflation(d, tube.inflation) - dist;
            if depth > 0.0 && best.as_ref().is_none_or(|old| depth > old.depth) {
                let direction = if dist > 1e-9 {
                    mul(delta, 1.0 / dist)
                } else {
                    let axis_len = if l2 > 1e-12 { l2.sqrt() } else { 1.0 };
                    [-axis[1] / axis_len, axis[0] / axis_len]
                };
                best = Some(Violation { depth, direction });
            }
        }
        best
    }

    /// The reference samples each segment at 0.25 mm and includes both ends.
    pub fn seg_clear(&self, a: Point, b: Point) -> bool {
        let l = distance(a, b);
        let n = (l / 0.25) as usize;
        let n = n.max(1);
        (0..=n).all(|k| {
            let t = k as f64 / n as f64;
            self.point_violation(add(a, mul(sub(b, a), t))).is_none()
        })
    }
}

#[derive(Clone, Copy)]
struct CandidateSet {
    discs: [usize; KMAX],
    disc_len: usize,
    tubes: [usize; KMAX],
    tube_len: usize,
}

impl CandidateSet {
    fn new() -> Self { Self { discs: [0; KMAX], disc_len: 0, tubes: [0; KMAX], tube_len: 0 } }
}

/// Relax a corridor spine using the reference's two spacing levels, Jacobi
/// smoothing, bounded motion, repeated ramped projection, block resampling,
/// convergence test, and overlong-chord fallback.
pub fn relax_spine(init: &[Point], obstacles: &RampedObstacles<'_>, rounds: usize) -> (Vec<Point>, usize) {
    if init.len() < 2 { return (init.to_vec(), 0); }
    let ends = [init[0], *init.last().unwrap()];
    let chord = distance(ends[0], ends[1]);
    let max_length = 3.0 * chord + 10.0;
    let mut points = taut_paths::densify(init, COARSE);
    let mut used = 0usize;

    for step in [COARSE, STEP] {
        points = taut_paths::densify(&points, step);
        let mut previous: Option<Vec<Point>> = None;
        let mut quiet = false;
        while used < rounds && !quiet {
            let reach_extra = obstacles.h.max(obstacles.extra.iter().map(|t| t.inflation).fold(0.0, f64::max)) + 0.6;
            let mut candidates = build_candidates(&points, obstacles, REACH + reach_extra);
            let mut anchor = points.clone();
            for _ in 0..BLOCK {
                used += 1;
                if points.len() < 3 { quiet = true; break; }
                let free = free_mask(&points, ends);
                let mut next = points.clone();
                for i in 1..points.len() - 1 {
                    next[i] = [
                        0.5 * points[i][0] + 0.25 * (points[i - 1][0] + points[i + 1][0]),
                        0.5 * points[i][1] + 0.25 * (points[i - 1][1] + points[i + 1][1]),
                    ];
                }
                for i in 0..next.len() {
                    if !free[i] { next[i] = points[i]; continue; }
                    let delta = sub(next[i], points[i]);
                    let magnitude = length(delta);
                    if magnitude > CAP { next[i] = add(points[i], mul(delta, CAP / magnitude)); }
                }
                project(&mut next, &free, &candidates, obstacles);
                let moved = points.iter().zip(&next).map(|(&a, &b)| distance(a, b)).fold(0.0, f64::max);
                points = next;
                let drift = points.iter().zip(&anchor).map(|(&a, &b)| distance(a, b)).fold(0.0, f64::max);
                if drift > 0.5 {
                    candidates = build_candidates(&points, obstacles, REACH + reach_extra);
                    anchor.clone_from(&points);
                }
                if moved < 1e-6 { quiet = true; break; }
            }
            if polyline_length(&points) > max_length {
                return (taut_paths::densify(&ends, STEP), used);
            }
            let resampled = resample(&points, obstacles, step);
            if let Some(prev) = &previous {
                if !prev.is_empty() && hausdorff(&resampled, prev) < 0.02 {
                    points = resampled;
                    break;
                }
            }
            previous = Some(resampled.clone());
            points = resampled;
        }
    }

    let reach_extra = obstacles.h.max(obstacles.extra.iter().map(|t| t.inflation).fold(0.0, f64::max)) + 0.6;
    let candidates = build_candidates(&points, obstacles, REACH + reach_extra);
    let free = free_mask(&points, ends);
    project(&mut points, &free, &candidates, obstacles);
    (points, used)
}

fn build_candidates(points: &[Point], obs: &RampedObstacles<'_>, reach: f64) -> Vec<CandidateSet> {
    points.iter().map(|&p| {
        let mut result = CandidateSet::new();
        let mut discs: Vec<_> = obs.base.discs.iter().enumerate()
            .filter(|(i, _)| !obs.base.excluded_discs.contains(i))
            .map(|(i, d)| (distance(p, d.point) - d.radius, i))
            .filter(|(gap, _)| *gap <= reach).collect();
        discs.sort_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.cmp(&b.1)));
        result.disc_len = discs.len().min(KMAX);
        for (slot, (_, idx)) in result.discs.iter_mut().zip(discs).take(result.disc_len) { *slot = idx; }

        let mut tubes: Vec<_> = obs.extra.iter().enumerate()
            .map(|(i, t)| (corridor::point_segment_distance(p, t.a, t.b) - t.radius, i))
            .filter(|(gap, _)| *gap <= reach).collect();
        tubes.sort_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.cmp(&b.1)));
        result.tube_len = tubes.len().min(KMAX);
        for (slot, (_, idx)) in result.tubes.iter_mut().zip(tubes).take(result.tube_len) { *slot = idx; }
        result
    }).collect()
}

fn project(points: &mut [Point], free: &[bool], candidates: &[CandidateSet], obs: &RampedObstacles<'_>) {
    let tangents = tangents(points);
    for i in 0..points.len() {
        if !free[i] { continue; }
        for _ in 0..PUSHES {
            let Some(v) = deepest(points[i], tangents[i], &candidates[i], obs) else { break; };
            points[i] = add(points[i], mul(v.direction, v.depth + OUT));
        }
    }
}

fn deepest(p: Point, tangent: Point, candidates: &CandidateSet, obs: &RampedObstacles<'_>) -> Option<Violation> {
    let d_end = obs.d_end(p);
    let pad = obs.part_inflation(p);
    let mut best: Option<Violation> = None;
    for &idx in &candidates.discs[..candidates.disc_len] {
        let disc = &obs.base.discs[idx];
        let delta = sub(p, disc.point);
        let dist = length(delta);
        let depth = disc.radius + pad - dist;
        if depth > 0.0 && best.as_ref().is_none_or(|v| depth > v.depth) {
            best = Some(Violation { depth, direction: if dist < 1e-9 { [1.0, 0.0] } else { mul(delta, 1.0 / dist) } });
        }
    }
    for &idx in &candidates.tubes[..candidates.tube_len] {
        let tube = &obs.extra[idx];
        let axis = sub(tube.b, tube.a);
        let l2 = dot(axis, axis);
        let l = if l2 > 0.0 { l2.sqrt() } else { 1.0 };
        let t = if l2 > 1e-12 { (dot(sub(p, tube.a), axis) / l2).clamp(0.0, 1.0) } else { 0.0 };
        let delta = sub(p, add(tube.a, mul(axis, t)));
        let dist = length(delta);
        let cross = if l2 > 0.0 { (tangent[0] * axis[1] / l - tangent[1] * axis[0] / l).abs() } else { 0.0 };
        if cross > CROSS_SIN { continue; }
        let depth = tube.radius + RampedObstacles::inflation(d_end, tube.inflation) - dist;
        if depth > 0.0 && best.as_ref().is_none_or(|v| depth > v.depth) {
            best = Some(Violation { depth, direction: if dist < 1e-9 { [-axis[1] / l, axis[0] / l] } else { mul(delta, 1.0 / dist) } });
        }
    }
    best
}

fn free_mask(points: &[Point], ends: [Point; 2]) -> Vec<bool> {
    points.iter().enumerate().map(|(i, &p)| {
        i > 0 && i + 1 < points.len() && distance(p, ends[0]) >= FREEZE && distance(p, ends[1]) >= FREEZE
    }).collect()
}

fn tangents(points: &[Point]) -> Vec<Point> {
    (0..points.len()).map(|i| {
        let delta = if i == 0 { sub(points[1], points[0]) }
            else if i + 1 == points.len() { sub(points[i], points[i - 1]) }
            else { sub(points[i + 1], points[i - 1]) };
        let l = length(delta);
        if l < 1e-12 { [1.0, 0.0] } else { mul(delta, 1.0 / l) }
    }).collect()
}

fn simplify(points: &[Point], tolerance: f64) -> Vec<Point> {
    if points.len() < 3 { return points.to_vec(); }
    let mut keep = vec![false; points.len()];
    keep[0] = true;
    keep[points.len() - 1] = true;
    let mut stack = vec![(0usize, points.len() - 1)];
    while let Some((a, b)) = stack.pop() {
        if b - a < 2 { continue; }
        let start = points[a];
        let end = points[b];
        let delta = sub(end, start);
        let l = length(delta);
        let mut maximum = 0.0;
        let mut index = None;
        for i in a + 1..b {
            let d = if l < 1e-12 { distance(points[i], start) }
                else { (cross(sub(points[i], start), delta)).abs() / l };
            if d > maximum { maximum = d; index = Some(i); }
        }
        if maximum > tolerance {
            if let Some(m) = index { keep[m] = true; stack.push((a, m)); stack.push((m, b)); }
        }
    }
    points.iter().zip(keep).filter_map(|(&p, keep)| keep.then_some(p)).collect()
}

fn shortcut(points: &[Point], obs: &RampedObstacles<'_>) -> Vec<Point> {
    if points.is_empty() { return Vec::new(); }
    let mut out = vec![points[0]];
    let mut i = 0;
    while i + 1 < points.len() {
        let mut j = points.len() - 1;
        while j > i + 1 && !obs.seg_clear(points[i], points[j]) {
            j = if j - i > 8 { (i + j) / 2 } else { j - 1 };
        }
        while j > i + 1 && !obs.seg_clear(points[i], points[j]) { j -= 1; }
        out.push(points[j]);
        i = j;
    }
    out
}

fn resample(points: &[Point], obs: &RampedObstacles<'_>, step: f64) -> Vec<Point> {
    let simple = simplify(points, 2e-4);
    taut_paths::densify(&shortcut(&simple, obs), step)
}

fn hausdorff(a: &[Point], b: &[Point]) -> f64 {
    fn directed(x: &[Point], y: &[Point]) -> f64 {
        x.iter().map(|&p| y.iter().map(|&q| distance(p, q)).fold(f64::INFINITY, f64::min))
            .fold(0.0, f64::max)
    }
    directed(a, b).max(directed(b, a))
}

fn polyline_length(points: &[Point]) -> f64 {
    points.windows(2).map(|w| distance(w[0], w[1])).sum()
}

fn add(a: Point, b: Point) -> Point { [a[0] + b[0], a[1] + b[1]] }
fn sub(a: Point, b: Point) -> Point { [a[0] - b[0], a[1] - b[1]] }
fn mul(a: Point, scale: f64) -> Point { [a[0] * scale, a[1] * scale] }
fn dot(a: Point, b: Point) -> f64 { a[0] * b[0] + a[1] * b[1] }
fn cross(a: Point, b: Point) -> f64 { a[0] * b[1] - a[1] * b[0] }
fn length(a: Point) -> f64 { a[0].hypot(a[1]) }
fn distance(a: Point, b: Point) -> f64 { length(sub(a, b)) }
