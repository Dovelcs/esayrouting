//! Source-compatible braid obstacle queries and corridor spine geometry.
pub use super::taut_spine::ExtraTube;
use super::{
    corridor::{self, Point, Spine},
    taut_paths::{self, Capsule, Disc, Obstacles},
    taut_spine::{self, RampZone, RampedObstacles},
    whole_context::WholeContext,
};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Debug)]
pub struct BraidObstacles {
    /// Inflated, layer-specific static obstacles used to derive the spine base.
    pub spine_obs: Obstacles,
    /// Uninflated end-array and array-scale pad discs used for corridor grouping.
    pub pad_obs: Obstacles,
}

/// One previously placed corridor spine. `half_width` is the placed corridor's
/// H; each segment is represented as an LPITCH-radius tube, inflated by the sum
/// of the old and new corridor half-widths when a new spine is built.
pub type LaidSpine = (Vec<Point>, f64);

pub type SpineExtra = ExtraTube;

fn component_counts(ctx: &WholeContext) -> BTreeMap<&str, usize> {
    let mut counts = BTreeMap::new();
    for pad in &ctx.pads {
        *counts.entry(pad.component_ref.as_str()).or_default() += 1;
    }
    counts
}

/// Source `unthreadable(fp)`: a part is a spine barrier only when the
/// smallest edge gap between any two pad bounding discs is below a track plus
/// two hug clearances.
fn component_unthreadable(ctx: &WholeContext, pads: &[&super::whole_context::ContextPad]) -> bool {
    let need = ctx.cfg.track_width + 2.0 * (ctx.cfg.clearance + 0.005);
    for (i, a) in pads.iter().enumerate() {
        let ra = a.size[0].max(a.size[1]) / 2.0;
        for b in pads.iter().skip(i + 1) {
            let rb = b.size[0].max(b.size[1]) / 2.0;
            let d = (a.point[0] - b.point[0]).hypot(a.point[1] - b.point[1]) - ra - rb;
            if d < need {
                return true;
            }
        }
    }
    false
}

/// Build the real `braid.py` board obstacle models. `kids` are all selected
/// route net IDs, and `end_refs` are the source-array footprint refs plus the
/// destination refs of those lanes.
pub fn board_obstacles(
    ctx: &WholeContext,
    kids: &BTreeSet<i32>,
    bundle_layer: &str,
    end_refs: &BTreeSet<String>,
) -> Result<BraidObstacles, String> {
    let spine_obs = taut_paths::build_obstacles(ctx, -1, kids, bundle_layer, None)?;
    let counts = component_counts(ctx);
    let mut groups: BTreeMap<&str, Vec<&super::whole_context::ContextPad>> = BTreeMap::new();
    for pad in &ctx.pads {
        groups
            .entry(pad.component_ref.as_str())
            .or_default()
            .push(pad);
    }
    let mut discs = Vec::new();
    for (component, pads) in groups {
        if !end_refs.contains(component) && counts.get(component).copied().unwrap_or(0) < 10 {
            continue;
        }
        for pad in pads {
            if pad.non_plated_hole || pad.pad_type == "np_thru_hole" {
                continue;
            }
            let hx = pad.size[0] / 2.0;
            let hy = pad.size[1] / 2.0;
            let radius = if pad.shape == "circle" || pad.shape == "oval" {
                hx.max(hy)
            } else {
                hx.hypot(hy)
            };
            discs.push(Disc {
                point: pad.point,
                radius,
                name: format!("{}.{}", component, pad.pad_number),
                net_id: pad.net_id,
            });
        }
    }
    Ok(BraidObstacles {
        spine_obs,
        pad_obs: Obstacles::new(discs, Vec::new()),
    })
}

fn pair_net_names(ctx: &WholeContext, members: &[String]) -> BTreeSet<String> {
    let mut names: BTreeSet<String> = members.iter().cloned().collect();
    for (base, pair) in &ctx.pairs {
        if names.contains(base) {
            names.insert(pair[0].clone());
            names.insert(pair[1].clone());
        }
    }
    names
}

/// Source `ctx.obs_but(nm, members, layer)`: exclude all member copper and
/// both physical legs of any pair member, while retaining the target's own
/// terminal shapes only as removed by the base obstacle derivation.
pub fn obs_but(
    ctx: &WholeContext,
    net: &str,
    members: &[String],
    layer: &str,
) -> Result<Obstacles, String> {
    let target = *ctx
        .byname
        .get(net)
        .ok_or_else(|| format!("braid obstacle query has unknown net {net}"))?;
    let names = pair_net_names(ctx, members);
    if names.iter().any(|name| !ctx.byname.contains_key(name)) {
        return Err("braid obstacle query has unresolved pair-member net IDs".into());
    }
    let kids: BTreeSet<i32> = names
        .iter()
        .filter_map(|name| ctx.byname.get(name).copied())
        .collect();
    taut_paths::build_obstacles(ctx, target, &kids, layer, None)
}

fn point_segment_distance(p: Point, a: Point, b: Point) -> f64 {
    corridor::point_segment_distance(p, a, b)
}

fn point_in_ring(p: Point, ring: &[Point]) -> bool {
    if ring.len() < 3 {
        return false;
    }
    let mut inside = false;
    for i in 0..ring.len() {
        let a = ring[i];
        let b = ring[(i + 1) % ring.len()];
        if (a[1] > p[1]) != (b[1] > p[1])
            && p[0] < a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
        {
            inside = !inside;
        }
    }
    inside
}

/// Source braid `_on_board`: enforce both the native board bounds and the
/// actual outline inset. Multiple outline rings use even-odd containment.
pub fn on_board(ctx: &WholeContext, p: Point, inset: f64) -> bool {
    if !inset.is_finite() || inset < 0.0 || p.iter().any(|v| !v.is_finite()) {
        return false;
    }
    let [x0, y0, x1, y1] = ctx.board_bounds;
    if x0 < x1
        && y0 < y1
        && !(x0 + inset <= p[0] && p[0] <= x1 - inset && y0 + inset <= p[1] && p[1] <= y1 - inset)
    {
        return false;
    }
    let rings: Vec<_> = ctx
        .board_outline
        .iter()
        .filter(|ring| ring.len() >= 3)
        .collect();
    if rings.is_empty() {
        return true;
    }
    let mut inside = false;
    for ring in rings {
        if point_in_ring(p, ring) {
            inside = !inside;
        }
        for i in 0..ring.len() {
            if point_segment_distance(p, ring[i], ring[(i + 1) % ring.len()]) < inset {
                return false;
            }
        }
    }
    inside
}

/// Source `Corridor._edge_inset`.
pub fn edge_inset(ctx: &WholeContext) -> f64 {
    let edge = if ctx.cfg.board_edge_clearance > 0.0 {
        ctx.cfg.board_edge_clearance
    } else {
        ctx.cfg.clearance
    };
    edge + ctx.cfg.track_width / 2.0 + 0.05
}

fn big_part_spine_obstacles(
    ctx: &WholeContext,
    members: &[String],
    spine_obs: &Obstacles,
) -> Result<Obstacles, String> {
    let mut own = BTreeSet::new();
    for name in members {
        own.insert(
            ctx.src_ref
                .get(name)
                .ok_or_else(|| format!("braid spine missing source array ref for {name}"))?
                .clone(),
        );
        own.insert(
            ctx.ends
                .get(name)
                .ok_or_else(|| format!("braid spine missing ends for {name}"))?
                .destination
                .clone(),
        );
    }
    let mut groups: BTreeMap<&str, Vec<&super::whole_context::ContextPad>> = BTreeMap::new();
    for pad in &ctx.pads {
        groups
            .entry(pad.component_ref.as_str())
            .or_default()
            .push(pad);
    }
    let big: BTreeSet<String> = groups
        .iter()
        .filter(|(component, pads)| {
            pads.len() >= 10 && !own.contains(**component) && component_unthreadable(ctx, pads)
        })
        .map(|(component, _)| (*component).to_owned())
        .collect();
    if big.is_empty() {
        return Ok(Obstacles::new(Vec::new(), Vec::new()));
    }
    let pad_component = |name: &str| -> Option<&str> {
        big.iter()
            .find(|component| {
                name.strip_prefix(component.as_str())
                    .is_some_and(|rest| rest.starts_with('.'))
            })
            .map(String::as_str)
    };
    let mut discs: Vec<Disc> = spine_obs
        .discs
        .iter()
        .filter(|d| pad_component(&d.name).is_some())
        .cloned()
        .collect();
    let mut grouped_caps: BTreeMap<(String, i32), Vec<&Capsule>> = BTreeMap::new();
    for cap in spine_obs.caps.iter() {
        if cap.name.starts_with("seg:") {
            continue;
        }
        if pad_component(&cap.name).is_some() {
            grouped_caps
                .entry((cap.name.clone(), cap.net_id))
                .or_default()
                .push(cap);
        }
    }
    for ((name, net_id), caps) in grouped_caps {
        let (min_x, max_x) = caps
            .iter()
            .flat_map(|c| [c.a[0], c.b[0]])
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), x| {
                (lo.min(x), hi.max(x))
            });
        let (min_y, max_y) = caps
            .iter()
            .flat_map(|c| [c.a[1], c.b[1]])
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), y| {
                (lo.min(y), hi.max(y))
            });
        let centre = [(min_x + max_x) / 2.0, (min_y + max_y) / 2.0];
        let radius = caps
            .iter()
            .flat_map(|c| [c.a, c.b].into_iter().map(move |p| (p, c.radius)))
            .map(|(p, r)| (p[0] - centre[0]).hypot(p[1] - centre[1]) + r)
            .fold(0.0, f64::max);
        discs.push(Disc {
            point: centre,
            radius,
            name,
            net_id,
        });
    }
    Ok(Obstacles::new(discs, Vec::new()))
}

fn distance(a: Point, b: Point) -> f64 {
    corridor::distance(a, b)
}

fn unit(v: Point) -> Point {
    let n = corridor::length(v);
    if n > 1e-12 {
        [v[0] / n, v[1] / n]
    } else {
        [1.0, 0.0]
    }
}

fn angle_deg(a: Point, b: Point) -> f64 {
    corridor::dot(unit(a), unit(b))
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees()
}

fn polyline_len(points: &[Point]) -> f64 {
    points.windows(2).map(|w| distance(w[0], w[1])).sum()
}

fn point_before_end(points: &[Point], back: f64) -> Point {
    let mut acc = 0.0;
    for i in (0..points.len().saturating_sub(1)).rev() {
        let (a, b) = (points[i], points[i + 1]);
        let len = distance(a, b);
        if acc + len >= back {
            let t = (back - acc) / len.max(1e-12);
            return corridor::add(b, corridor::mul(corridor::sub(a, b), t));
        }
        acc += len;
    }
    points[0]
}

fn point_after_start(points: &[Point], forward: f64) -> Point {
    let rev: Vec<_> = points.iter().rev().copied().collect();
    point_before_end(&rev, forward)
}

fn flow_dir(path: &[Point], end_dir: Point, at_start: bool) -> Point {
    const PROBE: f64 = 1.5;
    if path.len() < 2 {
        return unit(end_dir);
    }
    let taut = if polyline_len(path) < 2.0 * PROBE {
        corridor::sub(*path.last().unwrap(), path[0])
    } else if at_start {
        corridor::sub(point_after_start(path, PROBE), path[0])
    } else {
        corridor::sub(*path.last().unwrap(), point_before_end(path, PROBE))
    };
    let taut = unit(taut);
    if angle_deg(taut, end_dir) <= 45.0 {
        unit(end_dir)
    } else {
        taut
    }
}

fn simplify(points: &[Point], tol: f64) -> Vec<Point> {
    if points.len() < 3 {
        return points.to_vec();
    }
    let mut keep = vec![false; points.len()];
    keep[0] = true;
    keep[points.len() - 1] = true;
    let mut stack = vec![(0usize, points.len() - 1)];
    while let Some((a, b)) = stack.pop() {
        let mut best = tol;
        let mut index = None;
        for i in a + 1..b {
            let d = point_segment_distance(points[i], points[a], points[b]);
            if d > best {
                best = d;
                index = Some(i);
            }
        }
        if let Some(i) = index {
            keep[i] = true;
            stack.push((a, i));
            stack.push((i, b));
        }
    }
    let mut out = Vec::new();
    for (p, keep) in points.iter().copied().zip(keep) {
        if keep && out.last().is_none_or(|q| distance(p, *q) > 1e-6) {
            out.push(p);
        }
    }
    out
}

fn resample(points: &[Point], n: usize) -> Vec<Point> {
    if points.len() == 1 {
        return vec![points[0]; n];
    }
    let segments: Vec<_> = points.windows(2).map(|w| distance(w[0], w[1])).collect();
    let total: f64 = segments.iter().sum();
    if total < 1e-9 {
        return vec![points[0]; n];
    }
    (0..n)
        .map(|k| {
            let target = total * k as f64 / (n - 1) as f64;
            let mut acc = 0.0;
            for (i, len) in segments.iter().copied().enumerate() {
                if acc + len >= target - 1e-12 && len > 0.0 {
                    let t = (target - acc) / len;
                    return corridor::add(
                        points[i],
                        corridor::mul(corridor::sub(points[i + 1], points[i]), t),
                    );
                }
                acc += len;
            }
            *points.last().unwrap()
        })
        .collect()
}

fn mean_path(paths: &[Vec<Point>], n: usize) -> Vec<Point> {
    let sampled: Vec<_> = paths.iter().map(|p| resample(p, n)).collect();
    (0..n)
        .map(|i| {
            let sum = sampled
                .iter()
                .fold([0.0, 0.0], |a, p| [a[0] + p[i][0], a[1] + p[i][1]]);
            [sum[0] / sampled.len() as f64, sum[1] / sampled.len() as f64]
        })
        .collect()
}

fn segment_bad_count(obs: &RampedObstacles<'_>, a: Point, b: Point) -> usize {
    let len = distance(a, b);
    let n = (len / 0.25).floor().max(1.0) as usize;
    (0..=n)
        .filter(|&k| {
            let t = k as f64 / n as f64;
            obs.point_violation(corridor::add(a, corridor::mul(corridor::sub(b, a), t)))
                .is_some()
        })
        .count()
}

fn octilinearise(points: &[Point], obs: &RampedObstacles<'_>, flows: [Point; 2]) -> Vec<Point> {
    let points = simplify(points, 1.0);
    if points.len() < 3 {
        return points;
    }
    let mut out = vec![points[0]];
    for i in 0..points.len() - 1 {
        let (p, q) = (points[i], points[i + 1]);
        let v = corridor::sub(q, p);
        let angle = v[1].atan2(v[0]).to_degrees();
        let nearest = (angle / 45.0).round();
        let on_flow = if i == 0 {
            angle_deg(v, flows[0]) <= 3.0
        } else if i == points.len() - 2 {
            angle_deg(v, flows[1]) <= 3.0
        } else {
            false
        };
        if on_flow || (angle - 45.0 * nearest).abs() < 1.0 {
            out.push(q);
            continue;
        }
        let low = 45.0 * (angle / 45.0).floor();
        let d1 = [low.to_radians().cos(), low.to_radians().sin()];
        let d2 = [
            (low + 45.0).to_radians().cos(),
            (low + 45.0).to_radians().sin(),
        ];
        let det = corridor::cross(d1, d2);
        let alpha = corridor::cross(v, d2) / det;
        let beta = corridor::cross(d1, v) / det;
        let candidates = [
            corridor::add(p, corridor::mul(d1, alpha)),
            corridor::add(p, corridor::mul(d2, beta)),
        ];
        let own = segment_bad_count(obs, p, q);
        let scores =
            candidates.map(|m| segment_bad_count(obs, p, m) + segment_bad_count(obs, m, q));
        let pick = if scores[0] <= scores[1] { 0 } else { 1 };
        if scores[pick] <= own {
            out.push(candidates[pick]);
        }
        out.push(q);
    }
    simplify(&out, 0.08)
}

fn line_intersection(p: Point, d: Point, q: Point, e: Point) -> Option<Point> {
    let det = corridor::cross(d, e);
    if det.abs() < 1e-9 {
        return None;
    }
    let s = corridor::cross(corridor::sub(q, p), e) / det;
    Some(corridor::add(p, corridor::mul(d, s)))
}

fn merge_short_legs(points: &[Point], min_len: f64) -> Vec<Point> {
    let mut p = points.to_vec();
    loop {
        let mut changed = false;
        if p.len() <= 3 {
            break;
        }
        for i in 1..p.len() - 2 {
            if distance(p[i], p[i + 1]) >= min_len {
                continue;
            }
            let d1 = unit(corridor::sub(p[i], p[i - 1]));
            let d2 = unit(corridor::sub(p[i + 2], p[i + 1]));
            let det = corridor::cross(d1, d2);
            if det.abs() < 1e-9 {
                continue;
            }
            let s = corridor::cross(corridor::sub(p[i + 1], p[i - 1]), d2) / det;
            let m = corridor::add(p[i - 1], corridor::mul(d1, s));
            if s <= 0.0 || corridor::dot(corridor::sub(p[i + 2], m), d2) <= 0.0 {
                continue;
            }
            p.splice(i..=i + 1, [m]);
            changed = true;
            break;
        }
        if !changed {
            break;
        }
    }
    p
}

fn clear_legs(points: &[Point], obs: &RampedObstacles<'_>) -> Vec<Point> {
    let mut p = points.to_vec();
    for _ in 0..8 {
        let mut moved = false;
        for i in 1..p.len().saturating_sub(2) {
            let (a, b) = (p[i], p[i + 1]);
            let n = (distance(a, b) / 0.1).floor().max(1.0) as usize;
            let mut worst: Option<taut_paths::Violation> = None;
            for k in 0..=n {
                let t = k as f64 / n as f64;
                let q = corridor::add(a, corridor::mul(corridor::sub(b, a), t));
                if let Some(v) = obs.point_violation(q) {
                    if worst.is_none_or(|w| v.depth > w.depth) {
                        worst = Some(v);
                    }
                }
            }
            let Some(v) = worst else { continue };
            let d = unit(corridor::sub(b, a));
            let normal = [-d[1], d[0]];
            let sign = if corridor::dot(normal, v.direction) >= 0.0 {
                1.0
            } else {
                -1.0
            };
            let shifted = corridor::add(a, corridor::mul(normal, (v.depth + 0.02) * sign));
            let prev = unit(corridor::sub(a, p[i - 1]));
            let next = unit(corridor::sub(p[i + 2], b));
            let (Some(na), Some(nb)) = (
                line_intersection(shifted, d, p[i - 1], prev),
                line_intersection(shifted, d, p[i + 2], next),
            ) else {
                continue;
            };
            p[i] = na;
            p[i + 1] = nb;
            moved = true;
        }
        if !moved {
            break;
        }
    }
    let mut out = Vec::new();
    for q in p {
        if out.last().is_none_or(|p| distance(*p, q) > 1e-6) {
            out.push(q);
        }
    }
    out
}

fn spread(points: &[Point], center: Point, direction: Point) -> f64 {
    points
        .iter()
        .map(|p| corridor::dot(corridor::sub(*p, center), direction))
        .fold(0.0, f64::max)
}

fn spread_across(points: &[Point], center: Point, direction: Point) -> f64 {
    points
        .iter()
        .map(|p| corridor::cross(corridor::sub(*p, center), direction).abs())
        .fold(0.0, f64::max)
}

/// Faithful pure-geometry port of `corridor.build_spine`. `base_obs` must be
/// the big, unthreadable, non-end-array pad obstacles produced by
/// [`build_corridor_spine`]; `extra` are already laid spine segments as their
/// source tube radius and inflation.
#[allow(clippy::too_many_arguments)]
pub fn build_spine(
    paths: &[Vec<Point>],
    base_obs: &Obstacles,
    h: f64,
    extra: &[ExtraTube],
    teeth: &[Point],
    stubs: &[Point],
    tooth_dirs: &[Point],
    stub_dirs: &[Point],
    relax: bool,
) -> Result<Spine, String> {
    let n = paths.len();
    if n == 0
        || teeth.len() != n
        || stubs.len() != n
        || tooth_dirs.len() != n
        || stub_dirs.len() != n
        || paths.iter().any(|path| path.len() < 2)
        || !h.is_finite()
        || h < 0.0
    {
        return Err("invalid braid spine input dimensions or half-width".into());
    }
    let center_t = teeth
        .iter()
        .fold([0.0, 0.0], |a, p| [a[0] + p[0], a[1] + p[1]]);
    let center_s = stubs
        .iter()
        .fold([0.0, 0.0], |a, p| [a[0] + p[0], a[1] + p[1]]);
    let ct = corridor::mul(center_t, 1.0 / n as f64);
    let cs = corridor::mul(center_s, 1.0 / n as f64);
    let u_t = unit(
        tooth_dirs
            .iter()
            .zip(paths)
            .fold([0.0, 0.0], |sum, (d, path)| {
                corridor::add(sum, flow_dir(path, *d, true))
            }),
    );
    let u_s = unit(
        stub_dirs
            .iter()
            .zip(paths)
            .fold([0.0, 0.0], |sum, (d, path)| {
                corridor::add(sum, flow_dir(path, corridor::mul(*d, -1.0), false))
            }),
    );
    let straight = angle_deg(u_t, u_s) <= 30.0;
    if straight {
        let u = unit(corridor::add(u_t, u_s));
        let mid = corridor::mul(corridor::add(ct, cs), 0.5);
        let ta = corridor::dot(corridor::sub(ct, mid), u);
        let tb = corridor::dot(corridor::sub(cs, mid), u);
        let a = corridor::add(mid, corridor::mul(u, ta));
        let b = corridor::add(mid, corridor::mul(u, tb));
        let rt = spread(teeth, a, u) + 0.5;
        let rs = spread(stubs, b, corridor::mul(u, -1.0)) + 0.5;
        let p1 = corridor::add(a, corridor::mul(u, rt));
        let p2 = corridor::sub(b, corridor::mul(u, rs));
        let gap = corridor::dot(corridor::sub(p2, p1), u);
        if gap <= 0.5 {
            return Spine::new(&simplify(&[a, b], 0.08));
        }
        let ramp = RampedObstacles::new(base_obs, [ct, cs], h, extra, None);
        if ramp.seg_clear(p1, p2) {
            return Spine::new(&simplify(&[a, p1, p2, b], 0.08));
        }
    }

    let a = ct;
    let b = cs;
    let rt = spread(teeth, ct, u_t) + 0.5;
    let rs = spread(stubs, cs, corridor::mul(u_s, -1.0)) + 0.5;
    let p1 = corridor::add(ct, corridor::mul(u_t, rt));
    let p2 = corridor::sub(cs, corridor::mul(u_s, rs));
    if distance(p2, p1) <= 0.5 {
        return Spine::new(&simplify(&[a, b], 0.08));
    }
    let h_eff = h
        .max(spread_across(teeth, ct, u_t) + 0.1)
        .max(spread_across(stubs, cs, u_s) + 0.1);
    let zones = [
        RampZone {
            center: ct,
            flow: u_t,
            radius: rt,
        },
        RampZone {
            center: cs,
            flow: corridor::mul(u_s, -1.0),
            radius: rs,
        },
    ];
    let ramp = RampedObstacles::new(base_obs, [ct, cs], h_eff, extra, Some(zones));
    let mean = mean_path(paths, 60);
    let mut init = vec![p1];
    init.extend(
        mean.into_iter()
            .filter(|p| distance(*p, ct) > rt && distance(*p, cs) > rs),
    );
    init.push(p2);

    let (_n_arc, mid) = if relax {
        let (relaxed_points, _used) = taut_spine::relax_spine(&init, &ramp, 400);
        let n_arc = simplify(&relaxed_points, 0.08).len();
        let mut mid = octilinearise(&relaxed_points, &ramp, [u_t, u_s]);
        if mid.len() >= 2 {
            let mut joined = vec![p1];
            joined.extend(
                mid.iter()
                    .skip(1)
                    .take(mid.len().saturating_sub(2))
                    .copied(),
            );
            joined.push(p2);
            mid = clear_legs(&merge_short_legs(&joined, 0.7), &ramp);
        }
        (n_arc, mid)
    } else {
        (init.len(), init)
    };
    let mut spine_points = Vec::with_capacity(mid.len() + 2);
    spine_points.push(a);
    spine_points.extend(mid);
    spine_points.push(b);
    let mut result = simplify(&spine_points, 0.08);
    while result.len() > 2 {
        let sp = Spine::new(&result)?;
        let bad: BTreeSet<_> = sp
            .turn
            .iter()
            .enumerate()
            .filter(|(_, turn)| turn.abs() > 120.0)
            .map(|(j, _)| j + 1)
            .collect();
        if bad.is_empty() {
            break;
        }
        result = simplify(
            &result
                .iter()
                .enumerate()
                .filter_map(|(i, p)| (!bad.contains(&i)).then_some(*p))
                .collect::<Vec<_>>(),
            0.08,
        );
    }
    if Spine::new(&result)?
        .corners(80.0)
        .iter()
        .filter(|(_, _, turn)| turn.abs() > 80.0)
        .count()
        > 4
    {
        result = simplify(&[a, b], 0.08);
    }
    Spine::new(&result)
}

/// Source `ctx.spine_of` wrapper, including its big-part-only obstacle model
/// and already-laid corridor tubes.
pub fn build_corridor_spine(
    ctx: &WholeContext,
    members: &[String],
    spine_obs: &Obstacles,
    laid: &[LaidSpine],
    half_width: Option<f64>,
    relax: bool,
) -> Result<Spine, String> {
    if members.is_empty() {
        return Err("cannot build an empty braid corridor spine".into());
    }
    let h = half_width
        .unwrap_or(ctx.cfg.lane_pitch * (members.len() as f64 - 1.0) / 2.0 + ctx.cfg.lane_pitch);
    let mut extra = Vec::new();
    for (points, old_h) in laid {
        for w in points.windows(2) {
            extra.push(ExtraTube {
                a: w[0],
                b: w[1],
                radius: ctx.cfg.lane_pitch,
                inflation: old_h + h,
            });
        }
    }
    let paths: Vec<_> = members
        .iter()
        .map(|name| {
            ctx.paths
                .get(name)
                .cloned()
                .ok_or_else(|| format!("braid spine has no taut path for {name}"))
        })
        .collect::<Result<_, _>>()?;
    let teeth: Vec<_> = members
        .iter()
        .map(|name| {
            ctx.ends
                .get(name)
                .map(|e| e.tooth)
                .ok_or_else(|| format!("braid spine has no tooth for {name}"))
        })
        .collect::<Result<_, _>>()?;
    let stubs: Vec<_> = members
        .iter()
        .map(|name| {
            ctx.ends
                .get(name)
                .map(|e| e.berth)
                .ok_or_else(|| format!("braid spine has no berth for {name}"))
        })
        .collect::<Result<_, _>>()?;
    let tooth_dirs: Vec<_> = members
        .iter()
        .map(|name| {
            ctx.tooth_dir
                .get(name)
                .copied()
                .ok_or_else(|| format!("braid spine has no tooth direction for {name}"))
        })
        .collect::<Result<_, _>>()?;
    let stub_dirs: Vec<_> = members
        .iter()
        .map(|name| {
            ctx.stub_dir
                .get(name)
                .copied()
                .ok_or_else(|| format!("braid spine has no stub direction for {name}"))
        })
        .collect::<Result<_, _>>()?;
    let base = big_part_spine_obstacles(ctx, members, spine_obs)?;
    build_spine(
        &paths,
        &base,
        h,
        &extra,
        &teeth,
        &stubs,
        &tooth_dirs,
        &stub_dirs,
        relax,
    )
}
