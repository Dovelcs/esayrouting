//! Euclidean whole-route polish, ported from AWX whole_polish.py.
use super::{
    corridor::{distance, point_segment_distance, Point},
    linear_program::LinearProgram,
    pairs,
    whole_context::{LaneGeometry, LayerPiece, WholeContext},
    whole_frame::WholeFrame,
    whole_geometry::{GeometryFinding, GeometryResult},
};
use std::collections::{BTreeMap, BTreeSet};

const ROUNDS: usize = 12;
const CYCLES: usize = 3;
const STILL: f64 = 1e-6;
const EPS: f64 = 1e-4;

#[derive(Clone)]
struct Lane {
    x: Vec<Point>,
    layers: Vec<String>,
    fixed: Vec<bool>,
    no_copper: BTreeSet<usize>,
}

#[derive(Clone)]
struct Constraint {
    terms: Vec<(String, usize, Point)>,
    rhs: f64,
    kind: String,
    label: String,
    current: f64,
}

fn densify(
    ctx: &WholeContext,
    lane: &str,
    route: &LaneGeometry,
    max_step: f64,
) -> Result<Lane, String> {
    if route.xy.len() < 2 || route.layers.len() + 1 != route.xy.len() {
        return Err("AWX polish received malformed lane".into());
    }
    let mut x = vec![route.xy[0]];
    let mut layers = Vec::new();
    for (w, layer) in route.xy.windows(2).zip(&route.layers) {
        let n = (distance(w[0], w[1]) / max_step).ceil().max(1.0) as usize;
        for k in 1..=n {
            let t = k as f64 / n as f64;
            x.push([
                w[0][0] + (w[1][0] - w[0][0]) * t,
                w[0][1] + (w[1][1] - w[0][1]) * t,
            ]);
            layers.push(layer.clone());
        }
    }
    let mut fixed: Vec<bool> = (0..x.len())
        .map(|i| i == 0 || i + 1 == x.len() || route.snapped)
        .collect();
    let mut no_copper = BTreeSet::new();
    let pair_tips = ctx.pair_ends.get(lane);
    if !route.ends.is_empty() {
        for end in &route.ends {
            let pose_index = x
                .iter()
                .enumerate()
                .min_by(|a, b| distance(*a.1, end.pose).total_cmp(&distance(*b.1, end.pose)))
                .map(|v| v.0)
                .unwrap_or(0);
            let Some(tips) = pair_tips else {
                return Err(format!(
                    "AWX pair {lane} end poses lack physical tooth/berth tips"
                ));
            };
            let same0 = distance(end.tips[0], tips[0][0]) + distance(end.tips[1], tips[0][1]);
            let swap0 = distance(end.tips[0], tips[0][1]) + distance(end.tips[1], tips[0][0]);
            let same1 = distance(end.tips[0], tips[1][0]) + distance(end.tips[1], tips[1][1]);
            let swap1 = distance(end.tips[0], tips[1][1]) + distance(end.tips[1], tips[1][0]);
            let end_index = if same0.min(swap0) <= same1.min(swap1) {
                0
            } else {
                1
            };
            if end_index == 0 {
                for i in 0..=pose_index {
                    fixed[i] = true;
                }
            } else {
                for i in pose_index..x.len() {
                    fixed[i] = true;
                }
            }
            if end_index == 0 {
                no_copper.extend(0..pose_index);
            } else {
                no_copper.extend(pose_index..x.len() - 1);
            }
        }
    }
    for end in &route.ends {
        let pose_i = x
            .iter()
            .enumerate()
            .min_by(|a, b| distance(*a.1, end.pose).total_cmp(&distance(*b.1, end.pose)))
            .map(|v| v.0)
            .unwrap_or(0);
        let Some(tips) = pair_tips else {
            return Err(format!(
                "AWX pair {lane} end poses lack physical tooth/berth tips"
            ));
        };
        let same0 = distance(end.tips[0], tips[0][0]) + distance(end.tips[1], tips[0][1]);
        let swap0 = distance(end.tips[0], tips[0][1]) + distance(end.tips[1], tips[0][0]);
        let same1 = distance(end.tips[0], tips[1][0]) + distance(end.tips[1], tips[1][1]);
        let swap1 = distance(end.tips[0], tips[1][1]) + distance(end.tips[1], tips[1][0]);
        let startward = same0.min(swap0) <= same1.min(swap1);
        let direction = if startward { -1isize } else { 1isize };
        let mut u = end.heading;
        let mag = u[0].hypot(u[1]);
        if mag < 1e-9 {
            return Err("AWX pair pose has zero heading".into());
        }
        u = [u[0] / mag, u[1] / mag];
        let run = pairs::pose_probe_steps(&ctx.cfg) as f64 * ctx.cfg.grid_step
            / u[0].abs().max(u[1].abs());
        let mut s = 0.;
        let mut j = pose_i as isize;
        while j >= 0 && (j as usize) < x.len() {
            if j as usize != pose_i {
                s += distance(x[(j - direction) as usize], x[j as usize]);
                if s > run + 1e-9 {
                    break;
                }
            }
            x[j as usize] = [end.pose[0] + u[0] * s, end.pose[1] + u[1] * s];
            fixed[j as usize] = true;
            if (direction > 0
                && j + 1 < x.len() as isize
                && route.layers[j as usize] != route.layers[(j + 1) as usize])
                || (direction < 0
                    && j > 0
                    && route.layers[(j - 1) as usize] != route.layers[j as usize])
            {
                break;
            }
            j += direction;
        }
    }
    if let Some(c) = &route.cross {
        for (i, w) in x.windows(2).enumerate() {
            let mid = [(w[0][0] + w[1][0]) / 2., (w[0][1] + w[1][1]) / 2.];
            let along =
                (mid[0] - c.poses[0][0]) * c.heading[0] + (mid[1] - c.poses[0][1]) * c.heading[1];
            let span = (c.poses[1][0] - c.poses[0][0]) * c.heading[0]
                + (c.poses[1][1] - c.poses[0][1]) * c.heading[1];
            if along >= -1e-6 && along <= span + 1e-6 {
                no_copper.insert(i);
            }
        }
    }
    Ok(Lane {
        x,
        layers,
        fixed,
        no_copper,
    })
}

fn pair_approaches(ctx: &WholeContext, n: &str, l: &mut Lane) -> Result<(), String> {
    if !ctx.pairs.contains_key(n) {
        return Ok(());
    }
    let tips = ctx
        .pair_ends
        .get(n)
        .ok_or_else(|| format!("AWX pair {n} lacks source/destination terminal tips"))?;
    let cone = ctx.cfg.max_setback_angle.to_radians();
    for end in 0..2 {
        let reverse = end == 1;
        let mut x = l.x.clone();
        let mut lays = l.layers.clone();
        let mut fixed = l.fixed.clone();
        let seg_count = x.len() - 1;
        let mut no_copper = if reverse {
            l.no_copper.iter().map(|i| seg_count - 1 - *i).collect()
        } else {
            l.no_copper.clone()
        };
        if reverse {
            x.reverse();
            lays.reverse();
            fixed.reverse();
        }
        let s = arc_lengths(&x);
        let setback = pairs::end_run(&ctx.cfg, tips[end], None);
        if *s.last().unwrap_or(&0.) < 3. * setback {
            continue;
        }
        let mut k = s.partition_point(|v| *v < setback).min(x.len() - 1);
        if k == 0 {
            continue;
        }
        let way = if end == 0 {
            ctx.tooth_dir.get(n).copied()
        } else {
            ctx.stub_dir.get(n).map(|u| [-u[0], -u[1]])
        };
        let vias: Vec<_> = (1..lays.len())
            .filter(|&i| lays[i] != lays[i - 1])
            .collect();
        let mut joined = false;
        if way.is_none() && vias.iter().any(|v| *v <= k) {
            continue;
        }
        let mut a0 = x[0];
        let mut target = x[k];
        if let Some(w) = way {
            let wn = w[0].hypot(w[1]);
            if wn < 1e-12 {
                return Err(format!("AWX pair {n} has zero end-stub direction"));
            }
            let u = [w[0] / wn, w[1] / wn];
            let connector = pairs::end_connector(&ctx.cfg, tips[end]);
            a0 = [x[0][0] + u[0] * connector, x[0][1] + u[1] * connector];
            let v = [target[0] - a0[0], target[1] - a0[1]];
            let vl = v[0].hypot(v[1]);
            let dir = if vl > 1e-12 {
                [v[0] / vl, v[1] / vl]
            } else {
                u
            };
            let angle = (u[0] * dir[1] - u[1] * dir[0])
                .atan2(u[0] * dir[0] + u[1] * dir[1])
                .clamp(-cone, cone);
            let mut d = [
                u[0] * angle.cos() - u[1] * angle.sin(),
                u[0] * angle.sin() + u[1] * angle.cos(),
            ];
            target = [a0[0] + d[0] * vl, a0[1] + d[1] * vl];
            // When the first dive lies within this setback, join it to the same straight run.
            if let Some(via) = vias.iter().copied().filter(|v| *v > 1 && *v <= k).min() {
                let w = [x[via][0] - a0[0], x[via][1] - a0[1]];
                let mut oct: Vec<_> = (0..8)
                    .map(|j| {
                        let a = j as f64 * std::f64::consts::PI / 4.;
                        [a.cos(), a.sin()]
                    })
                    .filter(|v| v[0] * u[0] + v[1] * u[1] >= cone.cos() - 1e-9)
                    .collect();
                if let Some(best) = oct.drain(..).max_by(|a, b| {
                    (a[0] * w[0] + a[1] * w[1]).total_cmp(&(b[0] * w[0] + b[1] * w[1]))
                }) {
                    d = best;
                }
                let projected = (w[0] * d[0] + w[1] * d[1]).max(0.);
                let mut run = pairs::via_straight(&ctx.cfg, d) + ctx.cfg.grid_step;
                if pair_opposite_hands(ctx, n) {
                    for side in [-1, 1] {
                        if let Some(shape) = pairs::crossover_shape(&ctx.cfg, d, side) {
                            run = run.max(shape.1[0].max(shape.1[1]) + ctx.cfg.grid_step);
                        }
                    }
                }
                let point = [
                    a0[0] + d[0] * projected.max(run),
                    a0[1] + d[1] * projected.max(run),
                ];
                x[via] = point;
                k = via;
                target = point;
                joined = true;
            }
        }
        let l0 = distance(a0, x[0]);
        let l1 = distance(target, a0);
        let corner = if l0 > 1e-12 {
            ((k as f64 * l0 / (l0 + l1).max(1e-12)).round() as usize)
                .clamp(1, k.saturating_sub(1).max(1))
        } else {
            0
        };
        for i in 1..=k {
            if corner > 0 && i <= corner {
                x[i] = [
                    x[0][0] + (a0[0] - x[0][0]) * i as f64 / corner as f64,
                    x[0][1] + (a0[1] - x[0][1]) * i as f64 / corner as f64,
                ];
            } else {
                let t =
                    (i.saturating_sub(corner)) as f64 / (k.saturating_sub(corner).max(1)) as f64;
                x[i] = [
                    a0[0] + (target[0] - a0[0]) * t,
                    a0[1] + (target[1] - a0[1]) * t,
                ];
            }
            fixed[i] = true;
        }
        if !joined {
            let k2 = s.partition_point(|v| *v < s[k] + setback);
            if k2 < x.len() - 1 && !vias.iter().any(|v| k < *v && *v <= k2) {
                let endp = x[k2];
                for i in k + 1..k2 {
                    let t = (i - k) as f64 / (k2 - k) as f64;
                    x[i] = [
                        target[0] + (endp[0] - target[0]) * t,
                        target[1] + (endp[1] - target[1]) * t,
                    ];
                }
            }
        }
        let connector = pairs::end_connector(&ctx.cfg, tips[end]);
        let conn_i = s.partition_point(|v| *v < connector).min(seg_count);
        for i in 0..conn_i {
            no_copper.insert(i);
        }
        if reverse {
            x.reverse();
            lays.reverse();
            fixed.reverse();
            no_copper = no_copper.iter().map(|i| seg_count - 1 - *i).collect();
        }
        l.x = x;
        l.layers = lays;
        l.fixed = fixed;
        l.no_copper = no_copper;
    }
    Ok(())
}

fn pair_opposite_hands(ctx: &WholeContext, n: &str) -> bool {
    let Some(t) = ctx.pair_ends.get(n) else {
        return false;
    };
    let a = pairs::hand_vector(ctx.tooth_dir.get(n).copied(), t[0][0], t[0][1], false);
    let b = pairs::hand_vector(ctx.stub_dir.get(n).copied(), t[1][0], t[1][1], true);
    a != 0 && b != 0 && a != b
}

fn segment_distance(a: Point, b: Point, c: Point, d: Point) -> (f64, f64, f64, Point, Point) {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let cd = [d[0] - c[0], d[1] - c[1]];
    let cross = |u: Point, v: Point| u[0] * v[1] - u[1] * v[0];
    let den = cross(ab, cd);
    if den.abs() > 1e-15 {
        let ac = [c[0] - a[0], c[1] - a[1]];
        let t = cross(ac, cd) / den;
        let u = cross(ac, ab) / den;
        if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
            let p = [a[0] + ab[0] * t, a[1] + ab[1] * t];
            return (0.0, t, u, p, p);
        }
    }
    let proj = |p: Point, x: Point, y: Point| {
        let v = [y[0] - x[0], y[1] - x[1]];
        let n = v[0] * v[0] + v[1] * v[1];
        let t = if n == 0.0 {
            0.0
        } else {
            (((p[0] - x[0]) * v[0] + (p[1] - x[1]) * v[1]) / n).clamp(0.0, 1.0)
        };
        (t, [x[0] + v[0] * t, x[1] + v[1] * t])
    };
    let (u, q) = proj(a, c, d);
    let mut best = (distance(a, q), 0.0, u, a, q);
    let (t, p) = proj(b, c, d);
    if distance(b, p) < best.0 {
        best = (distance(b, p), 1.0, t, b, p);
    }
    let (t, p) = proj(c, a, b);
    if distance(c, p) < best.0 {
        best = (distance(c, p), t, 0.0, p, c);
    }
    let (t, p) = proj(d, a, b);
    if distance(d, p) < best.0 {
        best = (distance(d, p), t, 1.0, p, d);
    }
    best
}

fn closest(p: Point, a: Point, b: Point) -> Point {
    let d = [b[0] - a[0], b[1] - a[1]];
    let n = d[0] * d[0] + d[1] * d[1];
    let t = if n < 1e-24 {
        0.0
    } else {
        (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / n).clamp(0.0, 1.0)
    };
    [a[0] + d[0] * t, a[1] + d[1] * t]
}
fn deviates_from(route: &LaneGeometry, x: &[Point]) -> bool {
    x.iter().any(|p| {
        route
            .xy
            .windows(2)
            .map(|w| distance(*p, closest(*p, w[0], w[1])))
            .fold(f64::INFINITY, f64::min)
            > 1e-6
    })
}

fn rotate_point(p: Point, old: Point, ou: Point, new: Point, nu: Point) -> Point {
    let on = [-ou[1], ou[0]];
    let nn = [-nu[1], nu[0]];
    let d = [p[0] - old[0], p[1] - old[1]];
    let a = d[0] * ou[0] + d[1] * ou[1];
    let b = d[0] * on[0] + d[1] * on[1];
    [
        new[0] + a * nu[0] + b * nn[0],
        new[1] + a * nu[1] + b * nn[1],
    ]
}
fn update_crossover(
    c: &mut pairs::Crossover,
    old_route: &LaneGeometry,
    new_route: &LaneGeometry,
) -> Result<(), String> {
    let old = c.clone();
    let i = old_route
        .xy
        .iter()
        .enumerate()
        .min_by(|a, b| distance(*a.1, old.at).total_cmp(&distance(*b.1, old.at)))
        .map(|v| v.0)
        .ok_or("crossover has no route vertices")?;
    let j = new_route
        .xy
        .iter()
        .enumerate()
        .min_by(|a, b| distance(*a.1, old_route.xy[i]).total_cmp(&distance(*b.1, old_route.xy[i])))
        .map(|v| v.0)
        .unwrap_or(i);
    let dir = |x: &[Point], i: usize| {
        let a = if i > 0 { x[i - 1] } else { x[i] };
        let b = if i + 1 < x.len() { x[i + 1] } else { x[i] };
        let d = [b[0] - a[0], b[1] - a[1]];
        let l = d[0].hypot(d[1]).max(1e-12);
        [d[0] / l, d[1] / l]
    };
    let ou = {
        let d = old.heading;
        let l = d[0].hypot(d[1]).max(1e-12);
        [d[0] / l, d[1] / l]
    };
    let nu = dir(&new_route.xy, j);
    let new_at = new_route.xy[j];
    let t = |p: Point| rotate_point(p, old.at, ou, new_at, nu);
    c.at = new_at;
    c.heading = nu;
    for p in c.entry.values_mut() {
        *p = t(*p);
    }
    for p in c.exit.values_mut() {
        *p = t(*p);
    }
    for p in &mut c.poses {
        *p = t(*p);
    }
    for legs in c.legs.values_mut() {
        for leg in legs {
            for p in &mut leg.points {
                *p = t(*p);
            }
        }
    }
    for via in &mut c.vias {
        via.point = t(via.point);
    }
    Ok(())
}

fn offset_vertices(points: &[Point], offset: f64) -> Vec<Point> {
    if points.len() < 2 {
        return points.to_vec();
    }
    let unit = |a: Point, b: Point| {
        let d = [b[0] - a[0], b[1] - a[1]];
        let n = d[0].hypot(d[1]).max(1e-12);
        [d[0] / n, d[1] / n]
    };
    let normals: Vec<_> = points
        .windows(2)
        .map(|w| {
            let d = unit(w[0], w[1]);
            [-d[1], d[0]]
        })
        .collect();
    let mut out = Vec::with_capacity(points.len());
    out.push([
        points[0][0] + normals[0][0] * offset,
        points[0][1] + normals[0][1] * offset,
    ]);
    for i in 1..points.len() - 1 {
        let a = normals[i - 1];
        let b = normals[i];
        let q = [a[0] + b[0], a[1] + b[1]];
        let qn = q[0].hypot(q[1]);
        let cos_half = qn / 2.0;
        if qn < 1e-6 || cos_half < 0.3 {
            out.push([points[i][0] + a[0] * offset, points[i][1] + a[1] * offset]);
            continue;
        }
        let m = [q[0] / qn, q[1] / qn];
        let scale = offset / cos_half;
        out.push([points[i][0] + m[0] * scale, points[i][1] + m[1] * scale]);
    }
    let n = *normals.last().unwrap();
    out.push([
        points.last().unwrap()[0] + n[0] * offset,
        points.last().unwrap()[1] + n[1] * offset,
    ]);
    out
}

// Keep the braid.py `_pairs._simplify` and `_offset_side` behavior together:
// pair legs are offsets of each complete same-layer run, not independent
// segment offsets. The latter leaves gaps at outside corners and can make an
// inner leg run backwards through a short bend.
fn simplify_pair_run(points: &[Point]) -> Vec<Point> {
    if points.len() < 3 {
        return points.to_vec();
    }
    let mut out = vec![points[0]];
    for (i, &p) in points.iter().enumerate().skip(1) {
        if distance(*out.last().unwrap(), p) < 0.02 {
            if i + 1 == points.len() {
                // Source keeps the original run end unless this is its only
                // retained point.
                if out.len() == 1 { out[0] = p; }
            }
            continue;
        }
        out.push(p);
    }
    if distance(*out.last().unwrap(), *points.last().unwrap()) > 1e-9 {
        *out.last_mut().unwrap() = *points.last().unwrap();
    }
    let mut res = vec![out[0]];
    for i in 1..out.len().saturating_sub(1) {
        let a = *res.last().unwrap();
        let b = out[i];
        let c = out[i + 1];
        let u = [b[0] - a[0], b[1] - a[1]];
        let v = [c[0] - b[0], c[1] - b[1]];
        let ul = u[0].hypot(u[1]).max(1e-12);
        let vl = v[0].hypot(v[1]).max(1e-12);
        let cross = (u[0] * v[1] - u[1] * v[0]) / (ul * vl);
        let dot = (u[0] * v[0] + u[1] * v[1]) / (ul * vl);
        if cross.abs() >= 1e-6 || dot <= 0.0 {
            res.push(b);
        }
    }
    res.push(*out.last().unwrap());
    res
}

fn offset_pair_side(run: &[Point], offset: f64) -> Vec<Point> {
    let mut pts = run.to_vec();
    loop {
        let off = offset_vertices(&pts, offset);
        if pts.len() <= 2 { return off; }
        let backwards = (0..pts.len() - 1).find(|&i| {
            let a = [pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]];
            let b = [off[i + 1][0] - off[i][0], off[i + 1][1] - off[i][1]];
            a[0] * b[0] + a[1] * b[1] <= 0.0
        });
        let Some(i) = backwards else { return off; };
        let drop = if i + 1 < pts.len() - 1 { i + 1 } else { i };
        pts.remove(drop);
    }
}

fn pair_leg_runs(work: &Lane) -> Vec<(String, Vec<Point>)> {
    let mut layer_order = vec!["F.Cu".to_string(), "B.Cu".to_string()];
    let extras: BTreeSet<_> = work.layers.iter().cloned().collect();
    layer_order.extend(extras.into_iter().filter(|l| l != "F.Cu" && l != "B.Cu"));
    let mut runs = Vec::new();
    for layer in layer_order {
        let mut layer_runs: Vec<Vec<Point>> = Vec::new();
        for i in 0..work.layers.len() {
            if work.layers[i] != layer || work.no_copper.contains(&i) { continue; }
            let (p, q) = (work.x[i], work.x[i + 1]);
            if let Some(r) = layer_runs.iter_mut().find(|r| distance(p, *r.last().unwrap()) < 1e-4) {
                r.push(q);
            } else if let Some(r) = layer_runs.iter_mut().find(|r| distance(q, r[0]) < 1e-4) {
                r.insert(0, p);
            } else {
                layer_runs.push(vec![p, q]);
            }
            // Match braid._chain_runs' repeated end-to-start merge.
            loop {
                let merge = (0..layer_runs.len()).find_map(|i| {
                    (0..layer_runs.len()).find_map(|j| {
                        (i != j && distance(*layer_runs[i].last().unwrap(), layer_runs[j][0]) < 1e-4)
                            .then_some((i, j))
                    })
                });
                let Some((i, j)) = merge else { break; };
                let tail = layer_runs[j][1..].to_vec();
                layer_runs[i].extend(tail);
                layer_runs.remove(j);
            }
        }
        runs.extend(layer_runs.into_iter().map(|r| (layer.clone(), simplify_pair_run(&r))));
    }
    runs
}

fn rebuild_pair_legs(
    ctx: &WholeContext,
    n: &str,
    work: &Lane,
    route: &LaneGeometry,
) -> Result<BTreeMap<String, Vec<LayerPiece>>, String> {
    let pair = ctx.pairs.get(n).ok_or("pair lane has no P/N mapping")?;
    let half = pairs::pitch(&ctx.cfg) / 2.0;
    let d = [work.x[1][0] - work.x[0][0], work.x[1][1] - work.x[0][1]];
    let center = super::pairs::mid(work.x[0], work.x[1]);
    let tips = ctx
        .pair_ends
        .get(n)
        .map(|x| x[0])
        .unwrap_or([center, center]);
    let pdelta = [tips[0][0] - center[0], tips[0][1] - center[1]];
    let pside = if d[0] * pdelta[1] - d[1] * pdelta[0] >= 0.0 {
        1.0
    } else {
        -1.0
    };
    let mut out = BTreeMap::<String, Vec<LayerPiece>>::from([
        (pair[0].clone(), Vec::new()),
        (pair[1].clone(), Vec::new()),
    ]);
    for (layer, run) in pair_leg_runs(work) {
        if run.len() < 2 { continue; }
        for (leg, offset) in [(&pair[0], half * pside), (&pair[1], -half * pside)] {
            let path = offset_pair_side(&run, offset);
            for w in path.windows(2) {
                out.get_mut(leg).unwrap().push(LayerPiece { a: w[0], b: w[1], layer: layer.clone() });
            }
        }
    }
    for (end_index, e) in route.ends.iter().enumerate() {
        for (polarity, points) in e.legs.iter().enumerate() {
            let leg = &pair[polarity.min(1)];
            let _ = end_index;
            for w in points.windows(2) {
                out.get_mut(leg).unwrap().push(LayerPiece {
                    a: w[0],
                    b: w[1],
                    layer: e.layer.clone(),
                });
            }
        }
    }
    if let Some(c) = &route.cross {
        for (polarity, label) in [(0, "P"), (1, "N")] {
            if let Some(pieces) = c.legs.get(label) {
                let leg = &pair[polarity];
                for piece in pieces {
                    for w in piece.points.windows(2) {
                        out.get_mut(leg).unwrap().push(LayerPiece {
                            a: w[0],
                            b: w[1],
                            layer: piece.layer.clone(),
                        });
                    }
                }
            }
        }
    }
    Ok(out)
}

pub(crate) fn pair_legs_for_audit(
    ctx: &WholeContext,
    n: &str,
    route: &LaneGeometry,
) -> Result<BTreeMap<String, Vec<LayerPiece>>, String> {
    if !ctx.pairs.contains_key(n) {
        return Err(format!("{n} is not a differential pair lane"));
    }
    if !route.paired_legs.is_empty() {
        return Ok(route.paired_legs.clone());
    }
    let work = densify(ctx, n, route, 4. * ctx.cfg.grid_step)?;
    rebuild_pair_legs(ctx, n, &work, route)
}

fn via_indices(l: &Lane) -> Vec<usize> {
    (1..l.layers.len())
        .filter(|&i| l.layers[i] != l.layers[i - 1])
        .collect()
}

fn physical_segments(work: &Lane) -> Vec<LayerPiece> {
    work.x
        .windows(2)
        .zip(&work.layers)
        .enumerate()
        .filter(|(i, _)| !work.no_copper.contains(i))
        .map(|(_, (w, l))| LayerPiece {
            a: w[0],
            b: w[1],
            layer: l.clone(),
        })
        .collect()
}

fn planned_crossover(
    ctx: &WholeContext,
    n: &str,
    work: &Lane,
    routes: &BTreeMap<String, LaneGeometry>,
    lanes: &BTreeMap<String, Lane>,
    held: &BTreeSet<String>,
) -> Option<pairs::Crossover> {
    if held.contains(n)
        || routes[n].snapped
        || routes[n].cross.is_some()
        || !pair_opposite_hands(ctx, n)
    {
        return routes[n].cross.clone();
    }
    let i = *via_indices(work).first()?;
    if i == 0 || i + 1 >= work.x.len() {
        return None;
    }
    let raw = [
        work.x[i][0] - work.x[i - 1][0],
        work.x[i][1] - work.x[i - 1][1],
    ];
    let len = raw[0].hypot(raw[1]);
    if len < 1e-12 {
        return None;
    }
    let u = [raw[0] / len, raw[1] / len];
    let h = ctx.pair_ends.get(n)?;
    let s_in = pairs::hand_vector(ctx.tooth_dir.get(n).copied(), h[0][0], h[0][1], false);
    if s_in == 0 {
        return None;
    }
    let pair = ctx.pairs.get(n)?;
    let layer0 = work.layers[i - 1].as_str();
    let layer1 = work.layers[i].as_str();
    let half = pairs::pitch(&ctx.cfg) / 2.;
    let off = pairs::dive_offset(&ctx.cfg, half);
    let mut best: Option<(f64, pairs::Crossover)> = None;
    for first in [&pair[0], &pair[1]] {
        let polarity = if first == &pair[0] { "P" } else { "N" };
        let Some(c) = pairs::crossover(
            work.x[i],
            u,
            s_in,
            half,
            ctx.cfg.via_size,
            off,
            ctx.cfg.track_width,
            ctx.cfg.clearance,
            ctx.cfg.grid_step,
            layer0,
            layer1,
            polarity,
            pairs::handover_setback(&ctx.cfg),
        ) else {
            continue;
        };
        let mut room = f64::INFINITY;
        let own = own_ids(ctx, n);
        for v in &c.vias {
            for m in lanes.keys().filter(|m| *m != n) {
                for seg in physical_segments(&lanes[m]) {
                    room = room.min(
                        point_segment_distance(v.point, seg.a, seg.b)
                            - ctx.cfg.via_size / 2.
                            - ctx.cfg.clearance
                            - ctx.cfg.track_width / 2.,
                    );
                }
            }
            for s in &ctx.base_segments {
                if !own.contains(&s.net_id) {
                    room = room.min(
                        point_segment_distance(v.point, s.a, s.b)
                            - ctx.cfg.via_size / 2.
                            - ctx.cfg.clearance
                            - s.width / 2.,
                    );
                }
            }
            for p in &ctx.pads {
                if !own.contains(&p.net_id) {
                    let d = p
                        .copper_shapes
                        .iter()
                        .map(|x| x.shape.signed_distance(v.point))
                        .fold(f64::INFINITY, f64::min);
                    room = room.min(d - ctx.cfg.via_size / 2. - ctx.cfg.clearance);
                }
            }
            for v0 in &ctx.base_vias {
                if !own.contains(&v0.net_id) {
                    room = room.min(
                        distance(v.point, v0.point)
                            - (ctx.cfg.via_size / 2. + ctx.cfg.clearance + v0.diameter / 2.).max(
                                ctx.cfg.via_drill / 2.
                                    + v0.drill / 2.
                                    + ctx.cfg.hole_to_hole_clearance,
                            ),
                    );
                }
            }
        }
        for (label, legs) in &c.legs {
            let _ = label;
            for piece in legs {
                for w in piece.points.windows(2) {
                    for s in &ctx.base_segments {
                        if !own.contains(&s.net_id) && s.layer == piece.layer {
                            let (d, _, _, _, _) = segment_distance(w[0], w[1], s.a, s.b);
                            room = room.min(
                                d - ctx.cfg.track_width / 2. - ctx.cfg.clearance - s.width / 2.,
                            );
                        }
                    }
                    for pad in &ctx.pads {
                        if !own.contains(&pad.net_id) && pad.layers.contains(&piece.layer) {
                            for p in [
                                w[0],
                                w[1],
                                [(w[0][0] + w[1][0]) / 2., (w[0][1] + w[1][1]) / 2.],
                            ] {
                                room = room.min(
                                    pad_edge(ctx, p, pad, &piece.layer, true)
                                        - ctx.cfg.track_width / 2.
                                        - ctx.cfg.clearance,
                                );
                            }
                        }
                    }
                    for m in lanes.keys().filter(|m| *m != n) {
                        for seg in physical_segments(&lanes[m]) {
                            if seg.layer == piece.layer {
                                let (d, _, _, _, _) = segment_distance(w[0], w[1], seg.a, seg.b);
                                room = room.min(d - ctx.cfg.track_width - ctx.cfg.clearance);
                            }
                        }
                    }
                }
            }
        }
        if best.as_ref().is_none_or(|(score, _)| room > *score) {
            best = Some((room, c));
        }
    }
    best.map(|x| x.1)
}

pub(crate) fn planned_crossover_for_audit(
    ctx: &WholeContext,
    n: &str,
    geometry: &GeometryResult,
) -> Result<Option<pairs::Crossover>, String> {
    let Some(route) = geometry.lanes.get(n) else {
        return Ok(None);
    };
    if geometry.held.contains(n) || route.snapped {
        return Ok(route.cross.clone());
    }
    let mut lanes = BTreeMap::new();
    for (k, r) in &geometry.lanes {
        lanes.insert(k.clone(), densify(ctx, k, r, 4. * ctx.cfg.grid_step)?);
    }
    Ok(planned_crossover(
        ctx,
        n,
        &lanes[n],
        &geometry.lanes,
        &lanes,
        &geometry.held,
    ))
}

fn lane_barrels(
    ctx: &WholeContext,
    lane: &str,
    work: &Lane,
    route: &LaneGeometry,
    crossover: Option<&pairs::Crossover>,
) -> Vec<(usize, Point, String)> {
    let vias = via_indices(work);
    let mut out = Vec::new();
    let Some(legs) = ctx.pairs.get(lane) else {
        return vias
            .into_iter()
            .map(|i| (i, work.x[i], lane.to_owned()))
            .collect();
    };
    let off = pairs::dive_offset(&ctx.cfg, pairs::pitch(&ctx.cfg) / 2.0);
    let cross_at = route
        .cross
        .as_ref()
        .map(|c| c.at)
        .or_else(|| crossover.map(|c| c.at));
    let mut cross = crossover.cloned();
    let mut current = route.clone();
    current.xy = work.x.clone();
    if let Some(c) = cross.as_mut() {
        let _ = update_crossover(c, route, &current);
    }
    for (order, &i) in vias.iter().enumerate() {
        if let Some(c) = cross.as_ref() {
            if let Some(cross_order) = route
                .vias
                .iter()
                .enumerate()
                .min_by(|a, b| {
                    distance(*a.1, cross_at.unwrap()).total_cmp(&distance(*b.1, cross_at.unwrap()))
                })
                .map(|x| x.0)
            {
                if order == cross_order {
                    for barrel in &c.vias {
                        let ix = if barrel.polarity == "P" { 0 } else { 1 };
                        out.push((i, barrel.point, legs[ix].clone()));
                    }
                    continue;
                }
            }
        }
        let d = if i > 0 {
            [
                work.x[i][0] - work.x[i - 1][0],
                work.x[i][1] - work.x[i - 1][1],
            ]
        } else {
            [1., 0.]
        };
        let len = d[0].hypot(d[1]).max(1e-12);
        let normal = [-d[1] / len, d[0] / len];
        let center = work.x[i];
        let old = route.vias.get(order).copied().unwrap_or(center);
        for net in legs {
            let side = route
                .paired_vias
                .get(net)
                .and_then(|v| v.get(order))
                .map(|p| {
                    let q = [p[0] - old[0], p[1] - old[1]];
                    if q[0] * normal[0] + q[1] * normal[1] < 0.0 {
                        -1.0
                    } else {
                        1.0
                    }
                })
                .unwrap_or(1.0);
            out.push((
                i,
                [
                    center[0] + normal[0] * side * off,
                    center[1] + normal[1] * side * off,
                ],
                net.clone(),
            ));
        }
    }
    out
}

fn pad_edge(
    ctx: &WholeContext,
    p: Point,
    pad: &super::whole_context::ContextPad,
    layer: &str,
    pair: bool,
) -> f64 {
    if pad.non_plated_hole {
        return distance(p, pad.point) - pad.drill / 2.;
    }
    let d = pad
        .copper_shapes
        .iter()
        .filter(|s| s.layer == layer)
        .map(|s| s.shape.signed_distance(p))
        .fold(f64::INFINITY, f64::min);
    if pair {
        d
    } else {
        pairs::pad_corner_buffer(
            p[0] - pad.point[0],
            p[1] - pad.point[1],
            pad.size[0] / 2.,
            pad.size[1] / 2.,
            pad.corner_radius,
            ctx.cfg.grid_step,
        )
        .mul_add(-1.0, d)
    }
}

fn pad_edge_normal(
    ctx: &WholeContext,
    p: Point,
    pad: &super::whole_context::ContextPad,
    layer: &str,
    pair: bool,
) -> (f64, Option<Point>) {
    if pad.non_plated_hole {
        let d = [p[0] - pad.point[0], p[1] - pad.point[1]];
        let l = d[0].hypot(d[1]);
        return (
            distance(p, pad.point) - pad.drill / 2.,
            (l > 1e-9).then_some([d[0] / l, d[1] / l]),
        );
    }
    let mut best = (f64::INFINITY, None);
    for entry in pad.copper_shapes.iter().filter(|s| s.layer == layer) {
        let (d, q) = match &entry.shape {
            super::whole_context::CopperShape::Circle { center, radius } => {
                let v = [p[0] - center[0], p[1] - center[1]];
                let l = v[0].hypot(v[1]);
                if l < 1e-12 {
                    (-*radius, None)
                } else {
                    (l - *radius, Some([v[0] / l, v[1] / l]))
                }
            }
            super::whole_context::CopperShape::RoundedRectangle {
                center,
                half_size,
                rotation,
                corner_radius,
            } => {
                let a = rotation.to_radians();
                let dx = p[0] - center[0];
                let dy = p[1] - center[1];
                let local = [a.cos() * dx + a.sin() * dy, -a.sin() * dx + a.cos() * dy];
                let hx = half_size[0] - corner_radius;
                let hy = half_size[1] - corner_radius;
                let nearest = [local[0].clamp(-hx, hx), local[1].clamp(-hy, hy)];
                let delta = [local[0] - nearest[0], local[1] - nearest[1]];
                let l = delta[0].hypot(delta[1]);
                if l < 1e-12 {
                    (-(hx - local[0].abs()).min(hy - local[1].abs()), None)
                } else {
                    let ql = [
                        nearest[0] + delta[0] / l * corner_radius,
                        nearest[1] + delta[1] / l * corner_radius,
                    ];
                    let q = [
                        center[0] + a.cos() * ql[0] - a.sin() * ql[1],
                        center[1] + a.sin() * ql[0] + a.cos() * ql[1],
                    ];
                    let normal = [
                        a.cos() * delta[0] / l - a.sin() * delta[1] / l,
                        a.sin() * delta[0] / l + a.cos() * delta[1] / l,
                    ];
                    (l - *corner_radius, Some(normal))
                }
            }
            super::whole_context::CopperShape::Polygon { outline } => {
                let mut qbest = Point::default();
                let mut d = f64::INFINITY;
                for i in 0..outline.len() {
                    let q = closest(p, outline[i], outline[(i + 1) % outline.len()]);
                    let x = distance(p, q);
                    if x < d {
                        d = x;
                        qbest = q;
                    }
                }
                let inside = super::joint_geometry::point_in_polygon(p, outline);
                let v = if d < 1e-12 {
                    None
                } else if inside {
                    Some([(qbest[0] - p[0]) / d, (qbest[1] - p[1]) / d])
                } else {
                    Some([(p[0] - qbest[0]) / d, (p[1] - qbest[1]) / d])
                };
                (if inside { -d } else { d }, v)
            }
        };
        let cb = if pair {
            0.
        } else {
            pairs::pad_corner_buffer(
                p[0] - pad.point[0],
                p[1] - pad.point[1],
                pad.size[0] / 2.,
                pad.size[1] / 2.,
                pad.corner_radius,
                ctx.cfg.grid_step,
            )
        };
        if d - cb < best.0 {
            best = (d - cb, q);
        }
    }
    best
}

fn own_ids(ctx: &WholeContext, n: &str) -> BTreeSet<i32> {
    let mut out = BTreeSet::new();
    if let Some(i) = ctx.byname.get(n) {
        out.insert(*i);
    }
    if let Some(p) = ctx.pairs.get(n) {
        out.extend(p.iter().filter_map(|x| ctx.byname.get(x).copied()));
    }
    out
}

fn line_constraints(
    ctx: &WholeContext,
    lanes: &BTreeMap<String, Lane>,
    routes: &BTreeMap<String, LaneGeometry>,
    held: &BTreeSet<String>,
) -> Vec<Constraint> {
    let cfg = &ctx.cfg;
    let tw = cfg.track_width;
    let cl = cfg.clearance;
    let g = cfg.grid_step;
    let half = g / 2.;
    let margin = g;
    let snap_half = pairs::pitch(cfg) / 2.0 / (std::f64::consts::PI / 8.0).cos() + half;
    let ring = pairs::via_ring(cfg, 0., None);
    let ring_pair = pairs::via_ring(cfg, pairs::pitch(cfg) / 2., None);
    let need_vv = (cfg.via_size + cl).max(cfg.via_drill + cfg.hole_to_hole_clearance) + g;
    let need_static = tw / 2. + cl + half;
    let names: Vec<_> = lanes.keys().cloned().collect();
    let mut rows = Vec::new();
    let crossover_shapes: BTreeMap<_, _> = names
        .iter()
        .filter_map(|n| {
            planned_crossover(ctx, n, &lanes[n], routes, lanes, held).map(|c| (n.clone(), c))
        })
        .collect();
    // Same-layer lane pitch. The allowance includes the snap-grid landing of both lanes and the pair's leg reach.
    for (ai, a) in names.iter().enumerate() {
        let la = &lanes[a];
        let pa = physical_segments(la);
        for b in &names[ai + 1..] {
            let lb = &lanes[b];
            let pb = physical_segments(lb);
            for sa in &pa {
                for sb in &pb {
                    if sa.layer != sb.layer {
                        continue;
                    }
                    let (d, t, u, p, q) = segment_distance(sa.a, sa.b, sb.a, sb.b);
                    let reach_a = if ctx.pairs.contains_key(a) && routes[a].paired_legs.is_empty() {
                        snap_half + half
                    } else {
                        0.
                    };
                    let reach_b = if ctx.pairs.contains_key(b) && routes[b].paired_legs.is_empty() {
                        snap_half + half
                    } else {
                        0.
                    };
                    let need = tw + cl + g + reach_a + reach_b;
                    if d < 1e-9 || d >= need + margin {
                        continue;
                    }
                    let n = [(p[0] - q[0]) / d, (p[1] - q[1]) / d];
                    let ia = la.x.windows(2).position(|w| w[0] == sa.a && w[1] == sa.b);
                    let ib = lb.x.windows(2).position(|w| w[0] == sb.a && w[1] == sb.b);
                    if let (Some(i), Some(j)) = (ia, ib) {
                        rows.push(Constraint {
                            terms: vec![
                                (a.clone(), i, [n[0] * (1. - t), n[1] * (1. - t)]),
                                (a.clone(), i + 1, [n[0] * t, n[1] * t]),
                                (b.clone(), j, [-n[0] * (1. - u), -n[1] * (1. - u)]),
                                (b.clone(), j + 1, [-n[0] * u, -n[1] * u]),
                            ],
                            rhs: need + EPS - d,
                            kind: "pitch".into(),
                            label: format!("{a}/{b} {}", sa.layer),
                            current: d - need,
                        });
                    }
                }
            }
        }
    }
    // Static copper applies to the lane's actual pieces. Sampling its nearest point keeps pad outlines exact.
    for n in &names {
        let l = &lanes[n];
        let own = own_ids(ctx, n);
        let pair = ctx.pairs.contains_key(n);
        let segs = physical_segments(l);
        for s in segs {
            for obs in &ctx.base_segments {
                if obs.layer != s.layer || own.contains(&obs.net_id) {
                    continue;
                }
                let (d, t, _, p, _) = segment_distance(s.a, s.b, obs.a, obs.b);
                let need = need_static + obs.width / 2.;
                if d < need + margin {
                    if let Some(i) = l.x.windows(2).position(|w| w[0] == s.a && w[1] == s.b) {
                        if d > 1e-9 {
                            let v = [(p[0] - obs.a[0]) / d, (p[1] - obs.a[1]) / d];
                            rows.push(Constraint {
                                terms: vec![
                                    (n.clone(), i, [v[0] * (1. - t), v[1] * (1. - t)]),
                                    (n.clone(), i + 1, [v[0] * t, v[1] * t]),
                                ],
                                rhs: need + EPS - d,
                                kind: "static".into(),
                                label: format!("{} copper {}", s.layer, obs.net),
                                current: d - need,
                            });
                        }
                    }
                }
            }
            for pad in &ctx.pads {
                if own.contains(&pad.net_id)
                    || (!pad.layers.is_empty() && !pad.layers.contains(&s.layer))
                {
                    continue;
                }
                for k in 0..=4 {
                    let t = k as f64 / 4.;
                    let p = [
                        s.a[0] + (s.b[0] - s.a[0]) * t,
                        s.a[1] + (s.b[1] - s.a[1]) * t,
                    ];
                    let (d, normal) = pad_edge_normal(ctx, p, pad, &s.layer, pair);
                    let need = need_static + if pair { snap_half } else { 0.0 };
                    if d < need + margin {
                        if let Some(i) = l.x.windows(2).position(|w| w[0] == s.a && w[1] == s.b) {
                            if let Some(v) = normal {
                                rows.push(Constraint {
                                    terms: vec![
                                        (n.clone(), i, [v[0] * (1. - t), v[1] * (1. - t)]),
                                        (n.clone(), i + 1, [v[0] * t, v[1] * t]),
                                    ],
                                    rhs: need + EPS - d,
                                    kind: "static".into(),
                                    label: format!("pad {}.{}", pad.component_ref, pad.pad_number),
                                    current: d - need,
                                });
                            } else {
                                rows.push(Constraint {
                                    terms: Vec::new(),
                                    rhs: 1.0,
                                    kind: "static".into(),
                                    label: format!(
                                        "{n} inside pad {}.{}",
                                        pad.component_ref, pad.pad_number
                                    ),
                                    current: d - need,
                                });
                            }
                        }
                    }
                }
            }
            for via in &ctx.base_vias {
                if own.contains(&via.net_id) || !via.layers.contains(&s.layer) {
                    continue;
                }
                let p = closest(via.point, s.a, s.b);
                let d = distance(via.point, p) - via.diameter / 2.;
                let need = need_static;
                if d < need + margin {
                    if let Some(i) = l.x.windows(2).position(|w| w[0] == s.a && w[1] == s.b) {
                        let v = [
                            (p[0] - via.point[0]) / (distance(via.point, p)).max(1e-9),
                            (p[1] - via.point[1]) / (distance(via.point, p)).max(1e-9),
                        ];
                        rows.push(Constraint {
                            terms: vec![
                                (n.clone(), i, [v[0] * 0.5, v[1] * 0.5]),
                                (n.clone(), i + 1, [v[0] * 0.5, v[1] * 0.5]),
                            ],
                            rhs: need - d,
                            kind: "static".into(),
                            label: format!("via {}", via.net),
                            current: d - need,
                        });
                    }
                }
            }
        }
    }
    // A crossover's P/N legs are copper held in the frame, not a copper-free centerline.
    for owner in &names {
        let Some(c) = crossover_shapes.get(owner) else {
            continue;
        };
        let Some(pair) = ctx.pairs.get(owner) else {
            continue;
        };
        let own = own_ids(ctx, owner);
        for (polarity, label) in [(0, "P"), (1, "N")] {
            let net = &pair[polarity];
            for piece in c.legs.get(label).into_iter().flatten() {
                for w in piece.points.windows(2) {
                    for m in &names {
                        if m == owner {
                            continue;
                        }
                        let other = &lanes[m];
                        for j in 0..other.x.len() - 1 {
                            if other.no_copper.contains(&j) || other.layers[j] != piece.layer {
                                continue;
                            }
                            let (d, _t, u, p, q) =
                                segment_distance(w[0], w[1], other.x[j], other.x[j + 1]);
                            let need = tw
                                + cl
                                + g
                                + if ctx.pairs.contains_key(m) {
                                    snap_half + half
                                } else {
                                    0.
                                };
                            if d < 1e-9 || d >= need + margin {
                                continue;
                            }
                            let v = [(q[0] - p[0]) / d, (q[1] - p[1]) / d];
                            rows.push(Constraint {
                                terms: vec![
                                    (m.clone(), j, [-v[0] * (1. - u), -v[1] * (1. - u)]),
                                    (m.clone(), j + 1, [-v[0] * u, -v[1] * u]),
                                ],
                                rhs: need + EPS - d,
                                kind: "pitch".into(),
                                label: format!("{owner}/{m} crossover"),
                                current: d - need,
                            });
                        }
                    }
                    for s in &ctx.base_segments {
                        if own.contains(&s.net_id) || s.layer != piece.layer {
                            continue;
                        }
                        let (d, _, _, _, _) = segment_distance(w[0], w[1], s.a, s.b);
                        let need = need_static + s.width / 2.;
                        if d < need + margin {
                            rows.push(Constraint {
                                terms: Vec::new(),
                                rhs: 1.,
                                kind: "static".into(),
                                label: format!("{owner} crossover leg {net} copper {}", s.net),
                                current: d - need,
                            });
                        }
                    }
                    for pad in &ctx.pads {
                        if own.contains(&pad.net_id)
                            || (!pad.layers.is_empty() && !pad.layers.contains(&piece.layer))
                        {
                            continue;
                        }
                        let p = [(w[0][0] + w[1][0]) / 2., (w[0][1] + w[1][1]) / 2.];
                        let d = pad_edge(ctx, p, pad, &piece.layer, true);
                        let need = need_static;
                        if d < need + margin {
                            rows.push(Constraint {
                                terms: Vec::new(),
                                rhs: 1.,
                                kind: "static".into(),
                                label: format!(
                                    "{owner} crossover leg pad {}.{}",
                                    pad.component_ref, pad.pad_number
                                ),
                                current: d - need,
                            });
                        }
                    }
                }
            }
        }
    }
    // Planned dives are checked against other lanes, each other, and static board copper.
    let barrels: Vec<_> = names
        .iter()
        .flat_map(|n| {
            lane_barrels(ctx, n, &lanes[n], &routes[n], crossover_shapes.get(n))
                .into_iter()
                .map(move |(i, p, net)| (n.clone(), i, p, net))
        })
        .collect();
    for (n, i, b, net) in &barrels {
        let own = own_ids(ctx, n);
        let vxoff = if geometry_held(routes, n, held) {
            0.0
        } else {
            half
        };
        for m in &names {
            if m == n {
                continue;
            }
            let other = &lanes[m];
            let pair_other = ctx.pairs.contains_key(m);
            let need = if pair_other { ring_pair } else { ring } + vxoff + g;
            for j in 0..other.x.len() - 1 {
                if other.no_copper.contains(&j) {
                    continue;
                }
                let q = closest(*b, other.x[j], other.x[j + 1]);
                let d = distance(*b, q);
                if d < 1e-9 || d >= need + margin {
                    continue;
                }
                let v = [(b[0] - q[0]) / d, (b[1] - q[1]) / d];
                rows.push(Constraint {
                    terms: vec![
                        (n.clone(), *i, v),
                        (m.clone(), j, [-v[0] * (1.0 - 0.5), -v[1] * (1.0 - 0.5)]),
                        (m.clone(), j + 1, [-v[0] * 0.5, -v[1] * 0.5]),
                    ],
                    rhs: need + EPS - d,
                    kind: "via-lane".into(),
                    label: format!("{n}~{m}"),
                    current: d - need,
                });
            }
        }
        for (other, k, c, _net2) in &barrels {
            if other == n || ((other, k) < (n, i)) {
                continue;
            }
            let d = distance(*b, *c);
            let need = need_vv + vxoff;
            if d < 1e-9 || d >= need + margin {
                continue;
            }
            let v = [(b[0] - c[0]) / d, (b[1] - c[1]) / d];
            rows.push(Constraint {
                terms: vec![(n.clone(), *i, v), (other.clone(), *k, [-v[0], -v[1]])],
                rhs: need + EPS - d,
                kind: "via-via".into(),
                label: format!("{n}~{other}"),
                current: d - need,
            });
        }
        for layer in &ctx.layers {
            for other in &names {
                if other == n {
                    continue;
                }
                let Some(cross) = routes[other].cross.as_ref() else {
                    continue;
                };
                if !ctx.pairs.contains_key(other) {
                    continue;
                }
                for label in ["P", "N"] {
                    for piece in cross.legs.get(label).into_iter().flatten() {
                        if piece.layer != *layer {
                            continue;
                        }
                        for w in piece.points.windows(2) {
                            let q = closest(*b, w[0], w[1]);
                            let d = distance(*b, q) - tw / 2.;
                            let need = ring - tw / 2. + half + vxoff;
                            if d < need + margin {
                                let delta = [b[0] - q[0], b[1] - q[1]];
                                let len = delta[0].hypot(delta[1]);
                                if len > 1e-9 {
                                    rows.push(Constraint {
                                        terms: vec![(
                                            n.clone(),
                                            *i,
                                            [delta[0] / len, delta[1] / len],
                                        )],
                                        rhs: need + EPS - d,
                                        kind: "via-static".into(),
                                        label: format!("{n}~{other} crossover leg"),
                                        current: d - need,
                                    });
                                } else {
                                    rows.push(Constraint {
                                        terms: Vec::new(),
                                        rhs: 1.,
                                        kind: "via-static".into(),
                                        label: format!("{n} inside {other} crossover leg"),
                                        current: d - need,
                                    });
                                }
                            }
                        }
                    }
                }
            }
            for s in &ctx.base_segments {
                if own.contains(&s.net_id) || s.layer != *layer {
                    continue;
                }
                let q = closest(*b, s.a, s.b);
                let d = distance(*b, q) - s.width / 2.;
                let need = cfg.via_size / 2. + cfg.clearance + half + vxoff;
                if d < need + margin {
                    let len = distance(*b, q) + s.width / 2.;
                    if len > 1e-9 {
                        let v = [(b[0] - q[0]) / len, (b[1] - q[1]) / len];
                        rows.push(Constraint {
                            terms: vec![(n.clone(), *i, v)],
                            rhs: need + EPS - d,
                            kind: "via-static".into(),
                            label: format!("{net}~copper {}", s.net),
                            current: d - need,
                        });
                    }
                }
            }
            for pad in &ctx.pads {
                if own.contains(&pad.net_id)
                    || (!pad.layers.is_empty() && !pad.layers.contains(layer))
                {
                    continue;
                }
                let (d, normal) = pad_edge_normal(ctx, *b, pad, layer, ctx.pairs.contains_key(n));
                let need = cfg.via_size / 2. + cfg.clearance + half + vxoff;
                if d < need + margin {
                    if let Some(v) = normal {
                        rows.push(Constraint {
                            terms: vec![(n.clone(), *i, v)],
                            rhs: need + EPS - d,
                            kind: "via-static".into(),
                            label: format!("{net}~pad {}.{}", pad.component_ref, pad.pad_number),
                            current: d - need,
                        });
                    } else {
                        rows.push(Constraint {
                            terms: Vec::new(),
                            rhs: 1.,
                            kind: "via-static".into(),
                            label: format!(
                                "{net} inside pad {}.{}",
                                pad.component_ref, pad.pad_number
                            ),
                            current: d - need,
                        });
                    }
                }
            }
        }
    }
    let tvc = (cl + tw / 2. + cfg.via_size / 2.) * cfg.routing_clearance_margin;
    let spc = ((pairs::pitch(cfg) / 2.)
        .max((cfg.via_size + cl) / 2.)
        .max(tvc - pairs::pitch(cfg) / 2.)
        / g
        + 0.5)
        .floor()
        .max(1.);
    let r_line = cfg.via_size / 2. + tw / 2. + cl + pairs::pitch(cfg) / 2.;
    let r_via = cfg.via_size + cl;
    for n in &names {
        if !ctx.pairs.contains_key(n) {
            continue;
        }
        let l = &lanes[n];
        for i in via_indices(l) {
            let mut d = [l.x[i][0] - l.x[i - 1][0], l.x[i][1] - l.x[i - 1][1]];
            if d[0].hypot(d[1]) < 1e-9 && i + 1 < l.x.len() {
                d = [l.x[i + 1][0] - l.x[i][0], l.x[i + 1][1] - l.x[i][1]];
            }
            let k = (d[1].atan2(d[0]) / (std::f64::consts::PI / 4.)).round() as i32;
            let (ux, uy) = match k.rem_euclid(8) {
                0 => (1., 0.),
                1 => (1., 1.),
                2 => (0., 1.),
                3 => (-1., 1.),
                4 => (-1., 0.),
                5 => (-1., -1.),
                6 => (0., -1.),
                _ => (1., -1.),
            };
            let o = [-uy * spc * g, ux * spc * g];
            let mut cells = vec![
                l.x[i],
                [l.x[i][0] + o[0], l.x[i][1] + o[1]],
                [l.x[i][0] - o[0], l.x[i][1] - o[1]],
            ];
            for site in cells.drain(..) {
                for m in &names {
                    if m == n {
                        continue;
                    }
                    let other = &lanes[m];
                    let hw = if ctx.pairs.contains_key(m) {
                        snap_half + half
                    } else {
                        0.
                    };
                    for j in 0..other.x.len() - 1 {
                        if other.no_copper.contains(&j) {
                            continue;
                        }
                        let q = closest(site, other.x[j], other.x[j + 1]);
                        let d = distance(site, q);
                        let need = r_line
                            + hw
                            + g
                            + if geometry_held(routes, n, held) {
                                0.
                            } else {
                                half
                            };
                        if d < 1e-9 || d >= need + margin {
                            continue;
                        }
                        let v = [(site[0] - q[0]) / d, (site[1] - q[1]) / d];
                        rows.push(Constraint {
                            terms: vec![
                                (n.clone(), i, v),
                                (m.clone(), j, [-v[0] * 0.5, -v[1] * 0.5]),
                                (m.clone(), j + 1, [-v[0] * 0.5, -v[1] * 0.5]),
                            ],
                            rhs: need + EPS - d,
                            kind: "dive-lane".into(),
                            label: format!("{n}~{m}"),
                            current: d - need,
                        });
                    }
                    for (other, k, barrel, _net) in &barrels {
                        if other == n {
                            continue;
                        }
                        let d = distance(site, *barrel);
                        let need = r_via + g;
                        if d < 1e-9 || d >= need + margin {
                            continue;
                        }
                        let v = [(site[0] - barrel[0]) / d, (site[1] - barrel[1]) / d];
                        rows.push(Constraint {
                            terms: vec![(n.clone(), i, v), (other.clone(), *k, [-v[0], -v[1]])],
                            rhs: need + EPS - d,
                            kind: "dive-via".into(),
                            label: format!("{n}~{other}"),
                            current: d - need,
                        });
                    }
                }
            }
        }
    }
    // Pair-router dives retain the source straight run on both sides of every layer transition.
    for n in &names {
        if !ctx.pairs.contains_key(n) {
            continue;
        }
        let l = &lanes[n];
        for i in via_indices(l) {
            let unit = |v: Point| {
                let d = v[0].hypot(v[1]).max(1e-12);
                [v[0] / d, v[1] / d]
            };
            let before_dir = unit([l.x[i][0] - l.x[i - 1][0], l.x[i][1] - l.x[i - 1][1]]);
            let after_dir = unit([l.x[i + 1][0] - l.x[i][0], l.x[i + 1][1] - l.x[i][1]]);
            for (before, u) in [(true, before_dir), (false, after_dir)] {
                let mut j = i;
                if before {
                    while j > 0 {
                        let v = unit([l.x[j][0] - l.x[j - 1][0], l.x[j][1] - l.x[j - 1][1]]);
                        if v[0] * u[0] + v[1] * u[1] < 1. - 1e-6 {
                            break;
                        }
                        j -= 1;
                    }
                } else {
                    while j + 1 < l.x.len() - 1 {
                        let v =
                            unit([l.x[j + 2][0] - l.x[j + 1][0], l.x[j + 2][1] - l.x[j + 1][1]]);
                        if v[0] * u[0] + v[1] * u[1] < 1. - 1e-6 {
                            break;
                        }
                        j += 1;
                    }
                }
                let from = if before { l.x[j] } else { l.x[i] };
                let to = if before { l.x[i] } else { l.x[j + 1] };
                let length = (to[0] - from[0]) * u[0] + (to[1] - from[1]) * u[1];
                let mut required = pairs::via_straight(cfg, u) + g;
                if let Some(c) = crossover_shapes.get(n) {
                    let old = routes[n]
                        .xy
                        .iter()
                        .enumerate()
                        .min_by(|a, b| distance(*a.1, c.at).total_cmp(&distance(*b.1, c.at)))
                        .map(|v| v.0)
                        .unwrap_or(0);
                    let new = l
                        .x
                        .iter()
                        .enumerate()
                        .min_by(|a, b| distance(*a.1, l.x[i]).total_cmp(&distance(*b.1, l.x[i])))
                        .map(|v| v.0)
                        .unwrap_or(i);
                    if old == new {
                        required = if before { -c.span[0] } else { c.span[1] } + g;
                    }
                }
                if length < required - 1e-6 {
                    let terms = if before {
                        vec![(n.clone(), i, u), (n.clone(), j, [-u[0], -u[1]])]
                    } else {
                        vec![(n.clone(), j + 1, u), (n.clone(), i, [-u[0], -u[1]])]
                    };
                    rows.push(Constraint {
                        terms,
                        rhs: required + EPS - length,
                        kind: "pair-straight".into(),
                        label: n.clone(),
                        current: length - required,
                    });
                }
            }
            let dot = before_dir[0] * after_dir[0] + before_dir[1] * after_dir[1];
            if dot < 1. - 1e-6 {
                rows.push(Constraint {
                    terms: Vec::new(),
                    rhs: 1.0,
                    kind: "pair-straight".into(),
                    label: n.clone(),
                    current: dot - 1.,
                });
            }
            let all = via_indices(l);
            for end in 0..2 {
                let vopt = if end == 0 { all.first() } else { all.last() };
                let Some(&v) = vopt else {
                    continue;
                };
                let tip = if end == 0 {
                    ctx.pair_ends
                        .get(n)
                        .map(|e| e[0])
                        .unwrap_or([l.x[0], l.x[0]])
                } else {
                    ctx.pair_ends
                        .get(n)
                        .map(|e| e[1])
                        .unwrap_or([*l.x.last().unwrap(), *l.x.last().unwrap()])
                };
                let u = if end == 0 {
                    ctx.tooth_dir.get(n).copied()
                } else {
                    ctx.stub_dir.get(n).map(|v| [-v[0], -v[1]])
                };
                let need = pairs::dive_room(cfg, tip, u);
                let p = if end == 0 {
                    l.x[0]
                } else {
                    *l.x.last().unwrap()
                };
                let delta = if end == 0 {
                    [l.x[v][0] - p[0], l.x[v][1] - p[1]]
                } else {
                    [p[0] - l.x[v][0], p[1] - l.x[v][1]]
                };
                let len = delta[0].hypot(delta[1]);
                if len < need - 1e-6 && len > 1e-9 {
                    let u = [delta[0] / len, delta[1] / len];
                    rows.push(Constraint {
                        terms: if end == 0 {
                            vec![(n.clone(), v, u)]
                        } else {
                            vec![(n.clone(), v, [-u[0], -u[1]])]
                        },
                        rhs: need + EPS - len,
                        kind: "pair-end".into(),
                        label: n.clone(),
                        current: len - need,
                    });
                }
            }
        }
    }
    // Fixed-end rows preserve the source's single-ended router arc near both stubs.
    for (n, l) in lanes {
        if ctx.pairs.contains_key(n) {
            continue;
        }
        let ways = [
            ctx.tooth_dir.get(n).copied(),
            ctx.stub_dir.get(n).map(|u| [-u[0], -u[1]]),
        ];
        for (end, way) in ways.into_iter().enumerate() {
            let Some(w) = way else {
                continue;
            };
            let dirs = join_arc(w);
            for i in 0..l.x.len() - 1 {
                let arc = if end == 0 {
                    (0..=i).map(|j| distance(l.x[j], l.x[j + 1])).sum::<f64>()
                } else {
                    (i + 1..l.x.len())
                        .map(|j| distance(l.x[j - 1], l.x[j]))
                        .sum::<f64>()
                };
                if arc > tw + cl {
                    continue;
                }
                for u in &dirs {
                    let v = [l.x[i + 1][0] - l.x[i][0], l.x[i + 1][1] - l.x[i][1]];
                    let val = v[0] * u[0] + v[1] * u[1];
                    if val < margin {
                        rows.push(Constraint {
                            terms: vec![(n.clone(), i, [-u[0], -u[1]]), (n.clone(), i + 1, *u)],
                            rhs: EPS - val,
                            kind: "stub join".into(),
                            label: format!("{n}{}", if end == 0 { '<' } else { '>' }),
                            current: val,
                        });
                    }
                }
            }
        }
    }
    // Shape LP rows keep the track-scale turn at or below a fold while the clearance rows move it.
    for (n, l) in lanes {
        let s = arc_lengths(&l.x);
        let w = tw + cl;
        for i in 1..l.x.len() - 1 {
            if l.fixed[i] || l.layers[i - 1] != l.layers[i] {
                continue;
            }
            let b = (0..i).rev().find(|&j| s[i] - s[j] >= w).unwrap_or(0);
            let f = (i + 1..l.x.len())
                .find(|&j| s[j] - s[i] >= w)
                .unwrap_or(l.x.len() - 1);
            let d1 = [l.x[i][0] - l.x[b][0], l.x[i][1] - l.x[b][1]];
            let d2 = [l.x[f][0] - l.x[i][0], l.x[f][1] - l.x[i][1]];
            if d1[0] * d2[0] + d1[1] * d2[1] < 0.0 {
                rows.push(Constraint {
                    terms: vec![
                        (n.clone(), i, [d2[0] - d1[0], d2[1] - d1[1]]),
                        (n.clone(), b, [-d2[0], -d2[1]]),
                        (n.clone(), f, d1),
                    ],
                    rhs: -(d1[0] * d2[0] + d1[1] * d2[1]),
                    kind: "shape".into(),
                    label: n.clone(),
                    current: d1[0] * d2[0] + d1[1] * d2[1],
                });
            }
        }
    }
    let _ = (ring, ring_pair, need_vv, snap_half);
    rows
}

fn arc_lengths(x: &[Point]) -> Vec<f64> {
    let mut s = vec![0.0; x.len()];
    for i in 1..x.len() {
        s[i] = s[i - 1] + distance(x[i - 1], x[i]);
    }
    s
}

fn turn(a: Point, b: Point, c: Point) -> f64 {
    let u = [b[0] - a[0], b[1] - a[1]];
    let v = [c[0] - b[0], c[1] - b[1]];
    (u[0] * v[1] - u[1] * v[0])
        .atan2(u[0] * v[0] + u[1] * v[1])
        .to_degrees()
}
fn scale_point(x: &[Point], s: &[f64], at: f64) -> Point {
    let u = at.clamp(0., *s.last().unwrap_or(&0.));
    let k = s
        .partition_point(|v| *v < u)
        .clamp(1, s.len().saturating_sub(1));
    let den = s[k] - s[k - 1];
    let t = if den.abs() < 1e-12 {
        0.
    } else {
        (u - s[k - 1]) / den
    };
    [
        x[k - 1][0] + (x[k][0] - x[k - 1][0]) * t,
        x[k - 1][1] + (x[k][1] - x[k - 1][1]) * t,
    ]
}
fn join_arc(u: Point) -> Vec<Point> {
    let oct: Vec<Point> = (0..8)
        .map(|k| {
            let a = k as f64 * std::f64::consts::PI / 4.;
            [a.cos(), a.sin()]
        })
        .collect();
    let ok: Vec<_> = oct
        .iter()
        .enumerate()
        .filter(|(_, d)| d[0] * u[0] + d[1] * u[1] >= -1e-9)
        .map(|(k, _)| k)
        .collect();
    let lo = ok
        .iter()
        .copied()
        .find(|k| !ok.contains(&((k + 7) % 8)))
        .unwrap_or(0);
    let hi = ok
        .iter()
        .copied()
        .find(|k| !ok.contains(&((k + 1) % 8)))
        .unwrap_or(7);
    let a = [-oct[lo][1], oct[lo][0]];
    let b = [oct[hi][1], -oct[hi][0]];
    if a[0] * b[0] + a[1] * b[1] > 1. - 1e-9 {
        vec![a]
    } else {
        vec![a, b]
    }
}
fn shape_faults(l: &Lane, scale: f64, notch: f64, tiny: f64) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    let mut start = 0;
    while start < l.layers.len() {
        let mut end = start;
        while end + 1 < l.layers.len() && l.layers[end + 1] == l.layers[start] {
            end += 1;
        }
        let mut points = vec![(l.x[start], start)];
        for i in start + 1..=end + 1 {
            let p = l.x[i];
            if distance(points.last().unwrap().0, p) < tiny {
                continue;
            }
            if points.len() > 1 {
                let a = points[points.len() - 2].0;
                let b = points.last().unwrap().0;
                let cr = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
                if cr.abs() < 1e-7 {
                    points.pop();
                }
            }
            points.push((p, i));
        }
        if points.len() >= 3 {
            let mut local = Vec::new();
            for w in points.windows(3) {
                let t = turn(w[0].0, w[1].0, w[2].0);
                let outlen = distance(w[1].0, w[2].0);
                if t.abs() > 100. {
                    out.push((w[1].1, w[1].1));
                }
                local.push((w[1].1, t, outlen));
            }
            let x: Vec<_> = points.iter().map(|v| v.0).collect();
            let s = arc_lengths(&x);
            for i in 1..x.len() - 1 {
                let p = scale_point(&x, &s, s[i] - scale);
                let q = scale_point(&x, &s, s[i] + scale);
                let a = turn(p, x[i], q);
                if a.abs() > 100. {
                    out.push((points[i].1, points[i].1));
                }
            }
            for w in local.windows(2) {
                if w[0].1.abs() > 60.
                    && w[1].1.abs() > 60.
                    && w[0].1 * w[1].1 < 0.
                    && w[0].2 < notch
                {
                    out.push((w[0].0, w[0].0));
                }
            }
        }
        start = end + 1;
    }
    out.sort();
    out.dedup();
    out
}
fn drop_faults(l: &mut Lane, scale: f64, notch: f64, tiny: f64) -> usize {
    let mut dropped = 0;
    for _ in 0..50 {
        let s = arc_lengths(&l.x);
        let vias = via_indices(l);
        let mut hit = None;
        for (i0, i1) in shape_faults(l, scale, notch, tiny) {
            let mut j0 = (0..i0).rev().find(|&j| s[i0] - s[j] >= scale).unwrap_or(0);
            let mut j1 = (i1 + 1..l.x.len())
                .find(|&j| s[j] - s[i1] >= scale)
                .unwrap_or(l.x.len() - 1);
            for (i, f) in l.fixed.iter().enumerate() {
                if *f && i <= i0 {
                    j0 = j0.max(i);
                }
                if *f && i >= i1 {
                    j1 = j1.min(i);
                }
            }
            for &v in &vias {
                if v <= i0 {
                    j0 = j0.max(v);
                }
                if v >= i1 {
                    j1 = j1.min(v);
                }
            }
            if j1 >= j0 + 2 {
                hit = Some((j0, j1));
                break;
            }
        }
        let Some((a, b)) = hit else { break };
        let x = l.x.clone();
        let layers = l.layers.clone();
        let fixed = l.fixed.clone();
        l.x = x[..=a].iter().chain(x[b..].iter()).copied().collect();
        l.fixed = fixed[..=a]
            .iter()
            .chain(fixed[b..].iter())
            .copied()
            .collect();
        l.layers = layers[..a]
            .iter()
            .chain(layers[b - 1..].iter())
            .cloned()
            .collect();
        l.no_copper = l
            .no_copper
            .iter()
            .filter_map(|&i| {
                if i < a {
                    Some(i)
                } else if i >= b {
                    Some(i - (b - a))
                } else {
                    None
                }
            })
            .collect();
        dropped += 1;
    }
    dropped
}

fn geometry_held(
    routes: &BTreeMap<String, LaneGeometry>,
    n: &str,
    held: &BTreeSet<String>,
) -> bool {
    held.contains(n) || routes.get(n).is_some_and(|r| r.snapped)
}

fn fixed(lanes: &BTreeMap<String, Lane>, key: &(String, usize)) -> bool {
    lanes.get(&key.0).is_none_or(|l| l.fixed[key.1])
}

fn solve_round(
    lanes: &mut BTreeMap<String, Lane>,
    rows: &[Constraint],
    trust: f64,
    shape_scale: f64,
    budget_ms: u64,
) -> Result<Option<(f64, Vec<GeometryFinding>)>, String> {
    let active: Vec<_> = rows.iter().filter(|r| !r.terms.is_empty()).collect();
    let mut movable = BTreeSet::<(String, usize)>::new();
    let mut slack_of = Vec::new();
    for r in &active {
        for (n, i, _) in &r.terms {
            let k = (n.clone(), *i);
            if !fixed(lanes, &k) {
                movable.insert(k);
            }
        }
    }
    let seeds: Vec<_> = movable.iter().cloned().collect();
    for (n, i) in seeds {
        for d in [-2isize, -1, 1, 2] {
            let j = i as isize + d;
            if j > 0 && (j as usize) + 1 < lanes[&n].x.len() {
                let key = (n.clone(), j as usize);
                if !fixed(lanes, &key) {
                    movable.insert(key);
                }
            }
        }
    }
    if movable.is_empty() {
        return Ok(None);
    }
    let index: BTreeMap<_, _> = movable
        .iter()
        .enumerate()
        .map(|(i, k)| (k.clone(), i))
        .collect();
    let nv = index.len();
    let mut lp = LinearProgram::default();
    let mut dx = Vec::with_capacity(nv);
    let mut dy = Vec::with_capacity(nv);
    for _ in 0..nv {
        dx.push(lp.variable(-trust, trust, 0.));
        dy.push(lp.variable(-trust, trust, 0.));
    }
    // L1 displacement keeps the smallest movement; second differences keep a stretch smooth.
    for k in 0..nv {
        for v in [dx[k], dy[k]] {
            let a = lp.variable(0., f64::INFINITY, 1.);
            lp.le(vec![(v, 1.), (a, -1.)], 0.);
            lp.le(vec![(v, -1.), (a, -1.)], 0.);
        }
    }
    for ((n, i), k) in &index {
        for axis in 0..2 {
            let mut terms = vec![(if axis == 0 { dx[*k] } else { dy[*k] }, 1.)];
            for j in [
                i.checked_sub(1),
                i.checked_add(1).filter(|j| *j < lanes[n].x.len()),
            ]
            .into_iter()
            .flatten()
            {
                if let Some(q) = index.get(&(n.clone(), j)) {
                    terms.push((if axis == 0 { dx[*q] } else { dy[*q] }, -0.5));
                }
            }
            let a = lp.variable(0., f64::INFINITY, 2.);
            let mut r1 = terms.clone();
            r1.push((a, -1.));
            lp.le(r1, 0.);
            let mut r2: Vec<_> = terms.iter().map(|(v, c)| (*v, -*c)).collect();
            r2.push((a, -1.));
            lp.le(r2, 0.);
        }
    }
    for r in &active {
        let slack = lp.variable(0., f64::INFINITY, 1e4);
        let mut terms = Vec::new();
        for (n, i, c) in &r.terms {
            if let Some(k) = index.get(&(n.clone(), *i)) {
                terms.push((dx[*k], -c[0]));
                terms.push((dy[*k], -c[1]));
            }
        }
        terms.push((slack, -1.));
        lp.le(terms, -r.rhs);
        slack_of.push((slack, r));
    }
    // Source shape safeguard: linearized track-scale turn must remain below a fold.
    for ((n, i), k) in &index {
        if *i == 0 || i + 1 >= lanes[n].x.len() || fixed(lanes, &(n.clone(), *i)) {
            continue;
        }
        let s = arc_lengths(&lanes[n].x);
        let w = shape_scale;
        let before = (0..*i).rev().find(|&j| s[*i] - s[j] >= w).unwrap_or(i - 1);
        let after = (*i + 1..s.len())
            .find(|&j| s[j] - s[*i] >= w)
            .unwrap_or(i + 1);
        let a = [
            lanes[n].x[*i][0] - lanes[n].x[before][0],
            lanes[n].x[*i][1] - lanes[n].x[before][1],
        ];
        let b = [
            lanes[n].x[after][0] - lanes[n].x[*i][0],
            lanes[n].x[after][1] - lanes[n].x[*i][1],
        ];
        let mut terms = Vec::new();
        terms.push((dx[*k], b[0] - a[0]));
        terms.push((dy[*k], b[1] - a[1]));
        if let Some(q) = index.get(&(n.clone(), before)) {
            terms.push((dx[*q], -b[0]));
            terms.push((dy[*q], -b[1]));
        }
        if let Some(q) = index.get(&(n.clone(), after)) {
            terms.push((dx[*q], a[0]));
            terms.push((dy[*q], a[1]));
        }
        lp.le(terms, a[0] * b[0] + a[1] * b[1]);
    }
    for (i, v) in lp.variables.iter_mut().enumerate() {
        let frac = ((i + 1) as f64 * 0.6180339887498949).fract();
        v.cost += 1e-4 * (0.5 + frac);
    }
    let solution = lp.solve(budget_ms)?;
    if !solution.proved {
        return Ok(None);
    }
    let mut max_move = 0.0_f64;
    for ((n, i), k) in &index {
        let p = &mut lanes.get_mut(n).unwrap().x[*i];
        let quant = 2.0_f64.powi(-24);
        let mx = (solution.values[dx[*k]] / quant).round() * quant;
        let my = (solution.values[dy[*k]] / quant).round() * quant;
        p[0] += mx;
        p[1] += my;
        max_move = max_move.max(mx.abs()).max(my.abs());
    }
    let mut paid = Vec::new();
    for (v, r) in slack_of {
        let amount = solution.values[v];
        if amount > 1e-5 {
            let lane = r.terms.first().map(|x| x.0.clone()).unwrap_or_default();
            let other = r.label.split('~').nth(1).map(str::to_owned);
            paid.push(GeometryFinding {
                kind: r.kind.clone(),
                frame: "T".into(),
                lane,
                other,
                column: 0,
                amount,
            });
        }
    }
    Ok(Some((max_move, paid)))
}

/// Polish source geometry in board coordinates by repeatedly linearizing its audit bars.
pub fn polish(
    ctx: &WholeContext,
    frame: &WholeFrame,
    geometry: &GeometryResult,
    budget_ms: u64,
) -> Result<GeometryResult, String> {
    ctx.validate()?;
    if !geometry.metadata_complete {
        return Err("AWX polish requires complete source geometry metadata".into());
    }
    if geometry.failed {
        return Err("AWX polish refuses a geometry marked failed".into());
    }
    if budget_ms == 0 {
        return Err("AWX polish budget exhausted".into());
    }
    let max_step = 4. * ctx.cfg.grid_step;
    let mut lanes = BTreeMap::new();
    for (n, r) in &geometry.lanes {
        if let Some(pair) = ctx.pairs.get(n) {
            if !r.vias.is_empty()
                && pair.iter().any(|leg| {
                    r.paired_vias
                        .get(leg)
                        .is_none_or(|v| v.len() != r.vias.len())
                })
            {
                return Err(format!(
                    "AWX polish needs exact physical P/N barrel sites for every dive on {n}"
                ));
            }
        }
        let mut l = densify(ctx, n, r, max_step)?;
        if geometry.held.contains(n) || r.snapped {
            l.fixed.fill(true);
        }
        if ctx.pairs.contains_key(n)
            && r.ends.is_empty()
            && !geometry.held.contains(n)
            && !r.snapped
        {
            pair_approaches(ctx, n, &mut l)?;
        }
        if !r.ends.is_empty() {
            for i in 0..l.x.len() {
                if i < 2 || i + 2 >= l.x.len() {
                    l.fixed[i] = true;
                }
            }
        }
        lanes.insert(n.clone(), l);
    }
    let mut out = geometry.clone();
    let mut left = budget_ms;
    for _cycle in 0..CYCLES {
        for lane in lanes.values_mut() {
            drop_faults(
                lane,
                ctx.cfg.track_width + ctx.cfg.clearance,
                ctx.cfg.track_width + ctx.cfg.grid_step,
                ctx.cfg.track_width / 25.,
            );
        }
        for _round in 0..ROUNDS {
            if left == 0 {
                break;
            }
            let rows = line_constraints(ctx, &lanes, &geometry.lanes, &geometry.held);
            let before = rows.iter().filter(|r| r.current < -1e-6).count();
            if before == 0 {
                break;
            }
            let started = std::time::Instant::now();
            let Some((movement, _paid)) = solve_round(
                &mut lanes,
                &rows,
                2. * ctx.cfg.grid_step,
                ctx.cfg.track_width + ctx.cfg.clearance,
                left,
            )?
            else {
                break;
            };
            left = left.saturating_sub(started.elapsed().as_millis() as u64);
            if movement <= STILL {
                break;
            }
        }
        let mut flattened = 0;
        for lane in lanes.values_mut() {
            flattened += drop_faults(
                lane,
                ctx.cfg.track_width + ctx.cfg.clearance,
                ctx.cfg.track_width + ctx.cfg.grid_step,
                ctx.cfg.track_width / 25.,
            );
        }
        if flattened == 0 {
            break;
        }
    }
    for (n, l) in &lanes {
        let mut r = out
            .lanes
            .get(n)
            .cloned()
            .ok_or_else(|| format!("polish lost lane {n}"))?;
        r.xy = l.x.clone();
        r.layers = l.layers.clone();
        r.uxy =
            r.xy.windows(2)
                .map(|w| {
                    [
                        w[1][0] - w[0][0],
                        w[1][1] - w[0][1],
                        (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]),
                    ]
                })
                .collect();
        r.vias = (1..r.layers.len())
            .filter(|&i| r.layers[i] != r.layers[i - 1])
            .map(|i| r.xy[i])
            .collect();
        let original = &geometry.lanes[n];
        let old_centres = &original.vias;
        let new_indices = via_indices(l);
        if ctx.pairs.contains_key(n) {
            for (leg, points) in &original.paired_vias {
                let mut moved = Vec::new();
                for (order, p) in points.iter().enumerate() {
                    let old = old_centres.get(order).copied().unwrap_or(*p);
                    let old_i = original
                        .xy
                        .iter()
                        .enumerate()
                        .min_by(|a, b| distance(*a.1, old).total_cmp(&distance(*b.1, old)))
                        .map(|x| x.0)
                        .unwrap_or(0);
                    let new_i = new_indices.get(order).copied().unwrap_or(old_i);
                    let oc = old_centres.get(order).copied().unwrap_or(old);
                    let nc = r.vias.get(order).copied().unwrap_or(oc);
                    let od = if old_i > 0 {
                        [
                            original.xy[old_i][0] - original.xy[old_i - 1][0],
                            original.xy[old_i][1] - original.xy[old_i - 1][1],
                        ]
                    } else {
                        [1., 0.]
                    };
                    let nd = if new_i > 0 {
                        [
                            r.xy[new_i][0] - r.xy[new_i - 1][0],
                            r.xy[new_i][1] - r.xy[new_i - 1][1],
                        ]
                    } else {
                        od
                    };
                    let ol = od[0].hypot(od[1]).max(1e-12);
                    let nl = nd[0].hypot(nd[1]).max(1e-12);
                    let ou = [od[0] / ol, od[1] / ol];
                    let nu = [nd[0] / nl, nd[1] / nl];
                    let on = [-ou[1], ou[0]];
                    let nn = [-nu[1], nu[0]];
                    let delta = [p[0] - oc[0], p[1] - oc[1]];
                    let along = delta[0] * ou[0] + delta[1] * ou[1];
                    let across = delta[0] * on[0] + delta[1] * on[1];
                    moved.push([
                        nc[0] + nu[0] * along + nn[0] * across,
                        nc[1] + nu[1] * along + nn[1] * across,
                    ]);
                }
                r.paired_vias.insert(leg.clone(), moved);
            }
        }
        let current = r.clone();
        if let Some(cross) = r.cross.as_mut() {
            if deviates_from(original, &lanes[n].x) {
                update_crossover(cross, original, &current)?;
                if let Some(pair) = ctx.pairs.get(n) {
                    if let Some(order) = original
                        .vias
                        .iter()
                        .enumerate()
                        .min_by(|a, b| {
                            distance(*a.1, original.cross.as_ref().unwrap().at)
                                .total_cmp(&distance(*b.1, original.cross.as_ref().unwrap().at))
                        })
                        .map(|x| x.0)
                    {
                        for barrel in &cross.vias {
                            let idx = if barrel.polarity == "P" { 0 } else { 1 };
                            if let Some(points) = r.paired_vias.get_mut(&pair[idx]) {
                                if order < points.len() {
                                    points[order] = barrel.point;
                                }
                            }
                        }
                    }
                }
            }
        }
        if r.cross.is_none()
            && ctx.pairs.contains_key(n)
            && !geometry.held.contains(n)
            && !r.snapped
        {
            if let Some(cross) =
                planned_crossover(ctx, n, l, &geometry.lanes, &lanes, &geometry.held)
            {
                let pair = &ctx.pairs[n];
                if let Some(order) = original
                    .vias
                    .iter()
                    .enumerate()
                    .min_by(|a, b| distance(*a.1, cross.at).total_cmp(&distance(*b.1, cross.at)))
                    .map(|x| x.0)
                {
                    for barrel in &cross.vias {
                        let leg = if barrel.polarity == "P" {
                            &pair[0]
                        } else {
                            &pair[1]
                        };
                        if let Some(v) = r.paired_vias.get_mut(leg) {
                            if order < v.len() {
                                v[order] = barrel.point;
                            }
                        }
                    }
                }
            }
        }
        if ctx.pairs.contains_key(n) && !geometry.held.contains(n) && !r.snapped {
            r.paired_legs = rebuild_pair_legs(ctx, n, l, &r)?;
        }
        let band = ctx
            .band_of(n, &r, 0.0, false)
            .map_err(|e| format!("AWX polish cannot rebuild band for {n}: {e}"))?;
        out.bands.insert(n.clone(), band);
        out.lanes.insert(n.clone(), r);
    }
    let remaining = line_constraints(ctx, &lanes, &geometry.lanes, &geometry.held);
    out.paid = remaining
        .iter()
        .filter(|r| r.current < -1e-6)
        .map(|r| GeometryFinding {
            kind: r.kind.clone(),
            frame: "T".into(),
            lane: r.terms.first().map(|x| x.0.clone()).unwrap_or_default(),
            other: r.label.split('~').nth(1).map(str::to_owned),
            column: r.terms.first().map(|x| x.1 as i64).unwrap_or(0),
            amount: -r.current,
        })
        .collect();
    if left == 0 {
        return Err("AWX polish exhausted its time budget before completing its rounds".into());
    }
    // Feedback the room the continuous solve could not meet: via constraints become cuts,
    // static pad findings become island flips or layer cuts.
    for r in &remaining {
        if r.current >= -1e-6 {
            continue;
        }
        if matches!(
            r.kind.as_str(),
            "via-lane"
                | "via-via"
                | "via-static"
                | "dive-lane"
                | "dive-via"
                | "pair-straight"
                | "pair-end"
        ) {
            if let Some((n, i, _)) = r.terms.first() {
                let point = lanes[n].x[*i];
                let spine = frame
                    .cls
                    .get(n)
                    .and_then(|k| frame.rings.get(k))
                    .unwrap_or(&frame.spine);
                let u = spine.project_pt(point)[0];
                let w = ctx.cfg.via_size / 2.
                    + if ctx.pairs.contains_key(n) {
                        pairs::pitch(&ctx.cfg) / 2.
                    } else {
                        0.
                    };
                let cut = super::whole_types::ViaCut {
                    lane: n.clone(),
                    u,
                    w,
                    built: false,
                };
                if !out
                    .cuts
                    .vcuts
                    .iter()
                    .any(|v| v.lane == cut.lane && (v.u - cut.u).abs() < 1e-6)
                {
                    out.cuts.vcuts.push(cut);
                }
            }
        }
        if r.kind == "static" {
            if let Some(raw) = r.label.split("pad ").nth(1) {
                let key = raw.replace(' ', "");
                if let Some(island) = geometry.islands.get(&key) {
                    if let Some((n, _, _)) = r.terms.first() {
                        if let Some(box_) = geometry.island_boxes.get(island) {
                            if box_.layers.len() < ctx.layers.len() && ctx.layers.len() > 2 {
                                let first = *box_.layers.iter().next().unwrap();
                                let layer = if first == 0 { 1 } else { 0 };
                                let c = super::whole_types::LayerCut {
                                    lane: n.clone(),
                                    island: island.clone(),
                                    layer,
                                    blocked: Some(box_.layers.iter().map(|&x| x as i64).collect()),
                                    bounds: box_.bounds,
                                    span: None,
                                };
                                if !out
                                    .cuts
                                    .lcuts
                                    .iter()
                                    .any(|x| x.lane == c.lane && x.island == c.island)
                                {
                                    out.cuts.lcuts.push(c);
                                }
                            } else {
                                out.flips.insert((n.clone(), island.clone()));
                            }
                        }
                    }
                }
            }
        }
        if r.kind == "stub join" {
            if let Some((n, _, _)) = r.terms.first() {
                let end = r.label.ends_with('>');
                let s = arc_lengths(&lanes[n].x);
                let own = own_ids(ctx, n);
                let mut nearest: Option<(f64, String)> = None;
                for (i, p) in lanes[n].x.iter().enumerate() {
                    if if end {
                        s.last().copied().unwrap_or(0.) - s[i]
                            >= 2. * (ctx.cfg.track_width + ctx.cfg.clearance)
                    } else {
                        s[i] >= 2. * (ctx.cfg.track_width + ctx.cfg.clearance)
                    } {
                        continue;
                    }
                    let layer = &lanes[n].layers[i.min(lanes[n].layers.len() - 1)];
                    for pad in &ctx.pads {
                        if own.contains(&pad.net_id) {
                            continue;
                        }
                        let d = pad_edge(ctx, *p, pad, layer, ctx.pairs.contains_key(n));
                        if let Some(island) = geometry
                            .islands
                            .get(&format!("{}.{}", pad.component_ref, pad.pad_number))
                        {
                            if nearest.as_ref().is_none_or(|old| d < old.0) {
                                nearest = Some((d, island.clone()));
                            }
                        }
                    }
                }
                if let Some((_, island)) = nearest {
                    out.flips.insert((n.clone(), island));
                }
            }
        }
    }
    Ok(out)
}
