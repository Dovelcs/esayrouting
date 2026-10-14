//! AWX whole_ends.py geometric front and plane-ball escape capacity producers.
use super::{
    board_snapshot::CanonicalGeometry,
    corridor::{self, Point},
    escape_moves::{self, Move, Pad},
    joint_geometry::{self, JointGeometry},
    numeric,
    whole_context::WholeContext,
    whole_ends::{EndOption, EndsProblem},
    whole_frame::FlowFrame,
};
use serde::{Deserialize, Serialize};
use std::{
    collections::{BTreeMap, BTreeSet},
    time::Instant,
};
pub type Ball = (usize, String);
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PlaneWay {
    pub end: usize,
    pub site: Option<Point>,
    pub radius: f64,
    pub drill_radius: f64,
    pub stub: Option<[Point; 2]>,
    pub layer: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PlaneWays {
    pub balls: Vec<(Ball, Vec<usize>, Vec<(usize, Ball)>)>,
    pub kill: BTreeMap<(String, usize, usize), BTreeSet<usize>>,
    pub ways: Vec<PlaneWay>,
    pub n_ways: usize,
    pub seconds: f64,
}
fn transformed_pad(p: &Pad, flow: FlowFrame, physical: &[String]) -> Result<Pad, String> {
    let mut p = p.clone();
    p.point = flow.to_frame(p.point);
    p.layers = p
        .layers
        .iter()
        .map(|l| flow.to_frame_layer(l, physical))
        .collect::<Result<_, _>>()?;
    if flow.quarter_turns.rem_euclid(2) == 1 {
        p.size.swap(0, 1);
    }
    Ok(p)
}
fn kill_way(
    w: &PlaneWay,
    segs: &[(Point, Point, String, f64)],
    vias: &[(Point, f64, f64)],
    ray: Option<[Point; 2]>,
    hw_fan: f64,
    clear: f64,
    bar: f64,
    h2h: f64,
) -> bool {
    if let Some(site) = w.site {
        if segs.iter().any(|(a, b, _, hw)| {
            corridor::point_segment_distance(site, *a, *b) < w.radius + hw + clear - 1e-9
        }) {
            return true;
        }
        if vias.iter().any(|(p, r, d)| {
            corridor::distance(*p, site) < w.radius + r + clear - 1e-9
                || corridor::distance(*p, site) < w.drill_radius + d + h2h - 1e-9
        }) {
            return true;
        }
        if ray.is_some_and(|[a, b]| {
            corridor::point_segment_distance(site, a, b) - w.radius < bar - 1e-9
        }) {
            return true;
        }
    }
    if let Some([a, b]) = w.stub {
        if segs.iter().any(|(p, q, l, hw)| {
            l == &w.layer
                && super::pairs::segment_distance(a, b, *p, *q) < hw_fan + hw + clear - 1e-9
        }) {
            return true;
        }
        if vias
            .iter()
            .any(|(p, r, _)| corridor::point_segment_distance(*p, a, b) < r + hw_fan + clear - 1e-9)
        {
            return true;
        }
    }
    false
}
pub fn plane_ways(
    problem: &EndsProblem<'_>,
    ctx: &WholeContext,
    flow: FlowFrame,
    lanes: &[(String, Vec<String>)],
    t: &BTreeMap<String, Vec<EndOption>>,
    b: &BTreeMap<String, Vec<EndOption>>,
    cur: &BTreeMap<String, Move>,
) -> Result<Option<PlaneWays>, String> {
    let started = Instant::now();
    let run: BTreeSet<_> = lanes
        .iter()
        .flat_map(|(_, lg)| lg.iter().cloned())
        .collect();
    let run_ids: BTreeSet<_> = run
        .iter()
        .map(|n| {
            ctx.byname
                .get(n)
                .copied()
                .ok_or_else(|| format!("unknown plane-way lane {n}"))
        })
        .collect::<Result<_, _>>()?;
    // This is the source's shallow PCB copy with only the run's copper removed.
    let mut stripped = problem.snapshot.clone();
    stripped.segments.retain(|s| !run_ids.contains(&s.net_id));
    stripped.vias.retain(|v| !run_ids.contains(&v.net_id));
    let oracle = CanonicalGeometry {
        snapshot: &stripped,
        frame: flow,
    };
    let mut ways = Vec::new();
    let mut balls = Vec::new();
    let mut array = BTreeMap::new();
    for (end, input) in [(0, problem.source_input), (1, problem.destination_input)] {
        let drop_names: BTreeSet<_> = input
            .drops
            .iter()
            .map(|n| escape_moves::short_name(n).to_string())
            .filter(|n| {
                !input
                    .others
                    .iter()
                    .any(|q| escape_moves::short_name(q) == n)
                    && !run.contains(n)
            })
            .collect();
        let pads: Vec<_> = input
            .pads
            .iter()
            .map(|p| transformed_pad(p, flow, &problem.snapshot.layers))
            .collect::<Result<_, _>>()?;
        let items: BTreeMap<_, _> = pads
            .iter()
            .filter(|p| p.net_id > 0 && drop_names.contains(escape_moves::short_name(&p.net)))
            .map(|p| {
                (
                    format!("{}#{}", escape_moves::short_name(&p.net), p.pad_number),
                    p,
                )
            })
            .collect();
        if items.is_empty() {
            continue;
        }
        let grid = escape_moves::grid_of(&pads)?;
        let sz = &input.sizes;
        let mut zones = input.zones.clone();
        for z in &mut zones {
            z.layer = flow.to_frame_layer(&z.layer, &problem.snapshot.layers)?;
            for p in &mut z.outline {
                *p = flow.to_frame(*p);
            }
        }
        let mut dw: BTreeMap<String, Vec<usize>> = BTreeMap::new();
        let mut sw: BTreeMap<String, Vec<(usize, Ball)>> = BTreeMap::new();
        for (key, p) in &items {
            for d in joint_geometry::drops(&oracle, &grid, p, &pads, sz, &[], Some(&zones)) {
                dw.entry(key.clone()).or_default().push(ways.len());
                ways.push(PlaneWay {
                    end,
                    site: Some(d.site),
                    radius: d.radius,
                    drill_radius: d.drill_radius,
                    stub: d.stub,
                    layer: d.layer,
                });
            }
        }
        let near = |d: f64, pitch: f64| d < 0.01 || (d - pitch).abs() < 0.01;
        for (key, p) in &items {
            let home = ctx
                .layers
                .iter()
                .find(|l| p.layers.contains(l))
                .ok_or("plane ball has no copper layer")?;
            for (other, q) in &items {
                if key == other
                    || p.net_id != q.net_id
                    || !q.layers.contains(home)
                    || !near((p.point[0] - q.point[0]).abs(), grid.pitch_x)
                    || !near((p.point[1] - q.point[1]).abs(), grid.pitch_y)
                {
                    continue;
                }
                if oracle.segment_clear(
                    p.net_id,
                    home,
                    p.point,
                    q.point,
                    sz.clearance + sz.track / 2.,
                ) {
                    sw.entry(key.clone())
                        .or_default()
                        .push((ways.len(), (end, other.clone())));
                    ways.push(PlaneWay {
                        end,
                        site: None,
                        radius: 0.,
                        drill_radius: 0.,
                        stub: Some([p.point, q.point]),
                        layer: home.clone(),
                    });
                }
            }
        }
        for key in items.keys() {
            let ds = dw.remove(key).unwrap_or_default();
            let ss = sw.remove(key).unwrap_or_default();
            if !ds.is_empty() || !ss.is_empty() {
                balls.push(((end, key.clone()), ds, ss));
            }
        }
        array.insert(
            end,
            (
                sz.clone(),
                grid.bbox,
                grid.pitch_x.max(grid.pitch_y) / 2.,
                pads,
            ),
        );
    }
    if balls.is_empty() {
        let n = ways.len();
        return Ok(Some(PlaneWays {
            balls,
            kill: BTreeMap::new(),
            ways,
            n_ways: n,
            seconds: started.elapsed().as_secs_f64(),
        }));
    }
    let mut cells: BTreeMap<(usize, i64, i64), BTreeSet<usize>> = BTreeMap::new();
    for (i, w) in ways.iter().enumerate() {
        let mut pts = Vec::new();
        if let Some(p) = w.site {
            pts.push(p);
        }
        if let Some(s) = w.stub {
            pts.extend(s);
        }
        for q in pts {
            cells
                .entry((w.end, q[0].floor() as i64, q[1].floor() as i64))
                .or_default()
                .insert(i);
        }
    }
    let hw_fan = problem.rules.track_width / 2.;
    let clear = problem.rules.fan_clearance;
    let bar =
        problem.rules.track_width / 2. + problem.rules.clearance + problem.rules.grid_step / 2.;
    let mut killed = BTreeMap::new();
    for (lane, lg) in lanes {
        for (end, opts) in [(0, t.get(lane)), (1, b.get(lane))] {
            let Some((sz, bbox, half, pads)) = array.get(&end) else {
                continue;
            };
            let opts = opts.ok_or("missing plane-way end options")?;
            for (i, o) in opts.iter().enumerate() {
                if o.moves.len() != lg.len() {
                    return Err("plane-way pair option missing physical leg".into());
                }
                let mut segs = Vec::new();
                let mut vias = Vec::new();
                let mut rays = Vec::new();
                for (leg, m) in lg.iter().zip(&o.moves) {
                    let pad = pads
                        .iter()
                        .find(|p| escape_moves::short_name(&p.net) == leg);
                    let is_current = cur.get(leg).is_some_and(|c| {
                        serde_json::to_value(c).ok() == serde_json::to_value(m).ok()
                    });
                    if !m.legs.is_empty() || !is_current {
                        for l in &m.legs {
                            segs.push((l.a, l.b, l.layer.clone(), hw_fan));
                        }
                        if let Some(site) = m.site {
                            let own = pad.is_some_and(|p| corridor::distance(site, p.point) < 1e-6);
                            let size = if own {
                                pad.and_then(|p| sz.inpad.get(&p.key))
                                    .copied()
                                    .unwrap_or(sz.via)
                            } else {
                                sz.via
                            };
                            vias.push((site, size.radius, size.drill_radius));
                        }
                    } else {
                        let nid = *ctx
                            .byname
                            .get(leg)
                            .ok_or("plane-way missing physical net")?;
                        let near = |p: Point| {
                            bbox[0] - 2. <= p[0]
                                && p[0] <= bbox[2] + 2.
                                && bbox[1] - 2. <= p[1]
                                && p[1] <= bbox[3] + 2.
                        };
                        for s in ctx
                            .base_segments
                            .iter()
                            .filter(|s| s.net_id == nid && (near(s.a) || near(s.b)))
                        {
                            segs.push((s.a, s.b, s.layer.clone(), s.width / 2.));
                        }
                        for v in ctx
                            .base_vias
                            .iter()
                            .filter(|v| v.net_id == nid && near(v.point))
                        {
                            vias.push((v.point, v.diameter / 2., v.drill / 2.));
                        }
                    }
                    let (dx, dy) = m.direction.delta();
                    let d = [dx as f64, dy as f64];
                    rays.push([m.exit_pt, corridor::add(m.exit_pt, corridor::mul(d, 1.6))]);
                }
                let pts: Vec<_> = segs
                    .iter()
                    .flat_map(|(a, b, _, _)| [*a, *b])
                    .chain(vias.iter().map(|(p, _, _)| *p))
                    .chain(rays.iter().flat_map(|p| *p))
                    .collect();
                if pts.is_empty() {
                    continue;
                }
                let min = |k| {
                    pts.iter()
                        .map(|p| p[k])
                        .fold(f64::INFINITY, f64::min)
                        .floor() as i64
                };
                let max = |k| {
                    pts.iter()
                        .map(|p| p[k])
                        .fold(f64::NEG_INFINITY, f64::max)
                        .floor() as i64
                };
                let mut candidates = BTreeSet::new();
                for x in min(0) - 1..max(0) + 2 {
                    for y in min(1) - 1..max(1) + 2 {
                        if let Some(ids) = cells.get(&(end, x, y)) {
                            candidates.extend(ids);
                        }
                    }
                }
                let pocket = if o.moves.len() == 2
                    && o.moves[0].direction == o.moves[1].direction
                    && o.moves[0].layer == o.moves[1].layer
                {
                    let p = &o.moves[0];
                    let n = &o.moves[1];
                    let (dx, dy) = p.direction.delta();
                    let d = [dx as f64, dy as f64];
                    Some((
                        p.layer.clone(),
                        vec![
                            corridor::sub(p.exit_pt, corridor::mul(d, *half)),
                            corridor::sub(n.exit_pt, corridor::mul(d, *half)),
                            corridor::add(n.exit_pt, corridor::mul(d, 1.6)),
                            corridor::add(p.exit_pt, corridor::mul(d, 1.6)),
                        ],
                    ))
                } else {
                    None
                };
                let hits: BTreeSet<_> = candidates
                    .into_iter()
                    .filter(|&j| {
                        let w = &ways[j];
                        kill_way(w, &segs, &vias, None, hw_fan, clear, bar, sz.hole_to_hole)
                            || rays.iter().any(|r| {
                                kill_way(w, &[], &[], Some(*r), hw_fan, clear, bar, sz.hole_to_hole)
                            })
                            || pocket.as_ref().is_some_and(|(layer, quad)| {
                                w.site
                                    .is_some_and(|p| joint_geometry::in_pocket(quad, p, None))
                                    || w.stub.is_some_and(|[a, b]| {
                                        w.layer == *layer
                                            && joint_geometry::in_pocket(quad, a, Some(b))
                                    })
                            })
                    })
                    .collect();
                if !hits.is_empty() {
                    killed.insert((lane.clone(), end, i), hits);
                }
            }
        }
    }
    let n = ways.len();
    Ok(Some(PlaneWays {
        balls,
        kill: killed,
        ways,
        n_ways: n,
        seconds: started.elapsed().as_secs_f64(),
    }))
}
pub fn walled_balls(
    pw: &PlaneWays,
    lanes: &[(String, Vec<String>)],
    state: &BTreeMap<String, [usize; 2]>,
) -> Vec<String> {
    let mut killed = BTreeSet::<usize>::new();
    for (lane, _) in lanes {
        if let Some(indices) = state.get(lane) {
            for end in 0..2 {
                if let Some(ids) = pw.kill.get(&(lane.clone(), end, indices[end])) {
                    killed.extend(ids.iter().copied());
                }
            }
        }
    }
    let alive: BTreeMap<_, _> = pw
        .balls
        .iter()
        .map(|(ball, dw, _)| (ball.clone(), dw.iter().any(|i| !killed.contains(i))))
        .collect();
    pw.balls
        .iter()
        .filter(|(ball, _, sw)| {
            !alive[ball]
                && !sw
                    .iter()
                    .any(|(i, to)| !killed.contains(i) && alive.get(to).copied().unwrap_or(false))
        })
        .map(|((end, key), _, _)| format!("{end}:{key}"))
        .collect()
}
fn corner_blocked(
    ctx: &WholeContext,
    nid: i32,
    layer: &str,
    points: &[Point],
    bar: f64,
) -> Vec<bool> {
    let mut out = vec![false; points.len()];
    if points.is_empty() {
        return out;
    }
    let x0 = points.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min) - bar - 3.;
    let x1 = points
        .iter()
        .map(|p| p[0])
        .fold(f64::NEG_INFINITY, f64::max)
        + bar
        + 3.;
    let y0 = points.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min) - bar - 3.;
    let y1 = points
        .iter()
        .map(|p| p[1])
        .fold(f64::NEG_INFINITY, f64::max)
        + bar
        + 3.;
    for pad in &ctx.pads {
        if (nid != 0 && pad.net_id == nid)
            || pad.point[0] < x0
            || pad.point[0] > x1
            || pad.point[1] < y0
            || pad.point[1] > y1
        {
            continue;
        }
        for (i, p) in points.iter().enumerate() {
            let delta = corridor::sub(*p, pad.point);
            let d = if pad.non_plated_hole {
                corridor::length(delta) - pad.drill / 2.
            } else if pad.drill > 0. || pad.layers.iter().any(|l| l == "*.Cu" || l == layer) {
                super::pairs::pad_distance(
                    delta[0],
                    delta[1],
                    pad.size[0] / 2.,
                    pad.size[1] / 2.,
                    pad.corner_radius,
                ) - super::pairs::pad_corner_buffer(
                    delta[0],
                    delta[1],
                    pad.size[0] / 2.,
                    pad.size[1] / 2.,
                    pad.corner_radius,
                    ctx.cfg.grid_step,
                )
            } else {
                continue;
            };
            out[i] |= d < bar - 1e-9;
        }
    }
    out
}
pub fn front(
    ctx: &WholeContext,
    nid: i32,
    m: &Move,
    kids: &BTreeSet<i32>,
) -> Result<(u8, Option<[f64; 4]>), String> {
    let (dx, dy) = m.direction.delta();
    let d = [dx as f64, dy as f64];
    let step = 0.05;
    let n = (1.2f64 / step + 1e-9).floor() as usize;
    let pts: Vec<_> = (0..=n)
        .map(|k| corridor::add(m.exit_pt, corridor::mul(d, step * k as f64)))
        .collect();
    let mut children = kids.clone();
    children.insert(nid);
    let layers = &ctx.copper_layers;
    if layers.is_empty() {
        return Err("front requires physical copper stack".into());
    }
    let tbar = ctx.cfg.clearance + ctx.cfg.track_width / 2.;
    let vbar = ctx.cfg.via_size / 2. + ctx.cfg.clearance;
    let corners: BTreeMap<_, _> = layers
        .iter()
        .map(|l| (l.clone(), corner_blocked(ctx, nid, l, &pts, tbar)))
        .collect();
    let mut obstacles = BTreeMap::new();
    for layer in layers {
        obstacles.insert(
            layer.clone(),
            super::taut_paths::build_obstacles(ctx, nid, &children, layer, Some(tbar))?,
        );
    }
    let blocked =
        |layer: &String, k: usize| obstacles[layer].point_violation(pts[k]) || corners[layer][k];
    if !obstacles.contains_key(&m.layer) {
        return Err(format!("front move uses unknown layer {}", m.layer));
    }
    let mut hits: Vec<_> = (1..=n).filter(|&k| blocked(&m.layer, k)).collect();
    if !hits.is_empty()
        && ctx.layers.len() > 2
        && m.kind != escape_moves::MoveKind::Surface
        && ctx
            .layers
            .iter()
            .any(|l| l != &m.layer && (1..=n).all(|k| !blocked(l, k)))
    {
        hits.clear();
    }
    let Some(&k0) = hits.first() else {
        return Ok((0, None));
    };
    let k1 = (hits.last().unwrap() + 1).min(n);
    let mut via_obstacles = Vec::new();
    for layer in layers {
        via_obstacles.push((
            super::taut_paths::build_obstacles(ctx, nid, &children, layer, Some(vbar))?,
            corner_blocked(ctx, nid, layer, &pts[..k0], vbar),
        ));
    }
    let fit: Vec<_> = (0..k0)
        .filter(|&k| {
            via_obstacles
                .iter()
                .all(|(o, c)| !o.point_violation(pts[k]) && !c[k])
        })
        .collect();
    let route_layers = if ctx.layers.len() > 2 {
        &ctx.layers
    } else {
        layers
    };
    let on: Vec<_> = fit
        .iter()
        .copied()
        .filter(|&k| {
            route_layers
                .iter()
                .any(|l| l != &m.layer && (k..=k1).all(|j| !blocked(l, j)))
        })
        .collect();
    let span = if on.is_empty() {
        None
    } else {
        Some([
            numeric::round_digits(on[0] as f64 * step, 3)?,
            numeric::round_digits(*on.last().unwrap() as f64 * step, 3)?,
            numeric::round_digits(k0 as f64 * step, 3)?,
            numeric::round_digits(k1 as f64 * step, 3)?,
        ])
    };
    Ok((if fit.is_empty() { 2 } else { 1 }, span))
}
pub fn exit_front(
    ctx: &WholeContext,
    nid: i32,
    m: &Move,
    kids: &BTreeSet<i32>,
) -> Result<u8, String> {
    Ok(front(ctx, nid, m, kids)?.0)
}
pub fn front_span(
    ctx: &WholeContext,
    nid: i32,
    m: &Move,
    kids: &BTreeSet<i32>,
) -> Result<Option<[f64; 4]>, String> {
    Ok(front(ctx, nid, m, kids)?.1)
}
