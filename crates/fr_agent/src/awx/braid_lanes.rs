//! Fixed-source `Corridor.lay_lanes` and the geometry helpers it owns.
//!
//! This stage retains the source ordering of its decisions: exit legs and
//! crossings first, early dives, leg island avoidance and corner room, page
//! intervals/diver windows, lane centrelines, island deflection/ring ordering,
//! then swim-change diamonds and physical lane polylines.

use super::{
    braid_corridor::{CorridorState, LayerInterval, LayerRun, StaticIsland},
    braid_geometry, braid_offsets,
    braid_schedule::BraidSchedule,
    corridor::{self, Point},
};
use std::collections::{BTreeMap, BTreeSet};

const LEG_REQ: f64 = 0.35;
const HW_COL: f64 = 0.15;
const RING_DIP: f64 = 0.5;
const ISLAND_VETO: f64 = 100.0;
const ISLAND_GAP: f64 = 0.10;
const TOOTH_CLEAR_EXTRA: f64 = 0.04;

fn other_layer(layer: &str) -> &'static str {
    if layer == "F.Cu" {
        "B.Cu"
    } else {
        "F.Cu"
    }
}

fn interpolate(points: &[(f64, f64)], s: f64) -> Option<f64> {
    let (first, last) = (points.first()?, points.last()?);
    if s < first.0 - 1e-9 || s > last.0 + 1e-9 {
        return None;
    }
    if points.len() == 1 {
        return Some(first.1);
    }
    let idx = points
        .partition_point(|point| point.0 <= s)
        .saturating_sub(1)
        .min(points.len() - 2);
    let (a, b) = (points[idx], points[idx + 1]);
    let t = if (b.0 - a.0).abs() < 1e-12 {
        0.0
    } else {
        (s - a.0) / (b.0 - a.0)
    };
    Some(a.1 + (b.1 - a.1) * t)
}

fn interpolate_line(points: &[[f64; 2]], s: f64) -> Option<f64> {
    if points.is_empty() || s < points[0][0] - 1e-9 || s > points[points.len() - 1][0] + 1e-9 {
        return None;
    }
    if points.len() == 1 {
        return Some(points[0][1]);
    }
    let i = points
        .partition_point(|p| p[0] <= s)
        .saturating_sub(1)
        .min(points.len() - 2);
    let a = points[i];
    let b = points[i + 1];
    let t = if (b[0] - a[0]).abs() < 1e-12 {
        0.0
    } else {
        (s - a[0]) / (b[0] - a[0])
    };
    Some(a[1] + (b[1] - a[1]) * t)
}

fn source_s_end(state: &CorridorState) -> Result<f64, String> {
    let max = state
        .se
        .values()
        .map(|point| point[0])
        .reduce(f64::max)
        .ok_or("lay_lanes requires at least one member")?
        + 0.5;
    if !state.far_exit.is_empty() {
        let min = state
            .s_leg_min
            .ok_or("far-exit island scan missing minimum leg s")?;
        Ok(max.max(min + 10.0 * state.lane_min()))
    } else {
        Ok(max)
    }
}

/// Static non-array copper as source-compatible inflated `(s,o)` islands.
pub fn static_islands(
    state: &CorridorState,
) -> Result<BTreeMap<String, Vec<StaticIsland>>, String> {
    let ctx = &state.context;
    let mut arrays: BTreeSet<String> = state
        .members
        .iter()
        .filter_map(|net| ctx.src_ref.get(net).cloned())
        .collect();
    for net in &state.members {
        arrays.insert(
            ctx.ends
                .get(net)
                .ok_or_else(|| format!("static island geometry missing ends for {net}"))?
                .destination
                .clone(),
        );
    }
    let grow = state.options.clearance + state.options.track_width / 2.0;
    let mut offsets: Vec<f64> = state
        .st
        .values()
        .chain(state.se.values())
        .map(|point| point[1])
        .collect();
    offsets.extend(state.launch_o.values().copied());
    offsets.extend(state.target_o.values().copied());
    if offsets.is_empty() {
        return Err("static island geometry has no lane offsets".into());
    }
    let o_lo = offsets.iter().copied().fold(f64::INFINITY, f64::min) - 3.0;
    let o_hi = offsets.iter().copied().fold(f64::NEG_INFINITY, f64::max) + 3.0;
    let mut raw: BTreeMap<String, Vec<StaticIsland>> =
        BTreeMap::from([("F.Cu".into(), vec![]), ("B.Cu".into(), vec![])]);
    let s_end = source_s_end(state)?;
    for pad in &ctx.pads {
        if arrays.contains(&pad.component_ref) {
            continue;
        }
        let (hx, hy, layers) = if pad.pad_type == "np_thru_hole" {
            let radius = pad.drill / 2.0;
            (radius, radius, vec!["F.Cu".to_owned(), "B.Cu".to_owned()])
        } else {
            let layers = if pad.drill > 0.0 {
                vec!["F.Cu".to_owned(), "B.Cu".to_owned()]
            } else {
                pad.layers
                    .iter()
                    .filter(|layer| *layer == "F.Cu" || *layer == "B.Cu")
                    .cloned()
                    .collect()
            };
            (pad.size[0] / 2.0, pad.size[1] / 2.0, layers)
        };
        if layers.is_empty() || hx <= 0.0 || hy <= 0.0 {
            continue;
        }
        let projected: Vec<_> = [-hx, hx]
            .into_iter()
            .flat_map(|dx| {
                [-hy, hy].into_iter().map(move |dy| {
                    state
                        .spine
                        .project_pt([pad.point[0] + dx, pad.point[1] + dy])
                })
            })
            .collect();
        let s_lo = projected.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min) - grow;
        let s_hi = projected
            .iter()
            .map(|p| p[0])
            .fold(f64::NEG_INFINITY, f64::max)
            + grow;
        let o_lo_pad = projected.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min) - grow;
        let o_hi_pad = projected
            .iter()
            .map(|p| p[1])
            .fold(f64::NEG_INFINITY, f64::max)
            + grow;
        if s_hi < state.s0 || s_lo > s_end || o_hi_pad < o_lo || o_lo_pad > o_hi {
            continue;
        }
        for layer in layers {
            raw.entry(layer).or_default().push(StaticIsland {
                bounds: [s_lo, s_hi, o_lo_pad, o_hi_pad],
                what: format!("{}.{}", pad.component_ref, pad.pad_number),
            });
        }
    }
    for boxes in raw.values_mut() {
        boxes.sort_by(|a, b| {
            a.bounds
                .iter()
                .zip(b.bounds.iter())
                .find_map(|(x, y)| {
                    let c = x.total_cmp(y);
                    (c != std::cmp::Ordering::Equal).then_some(c)
                })
                .unwrap_or_else(|| a.what.cmp(&b.what))
        });
        let mut merged: Vec<StaticIsland> = Vec::new();
        for island in boxes.drain(..) {
            if let Some(existing) = merged.iter_mut().find(|old| {
                island.bounds[0] <= old.bounds[1] + ISLAND_GAP
                    && old.bounds[0] <= island.bounds[1] + ISLAND_GAP
                    && island.bounds[2] <= old.bounds[3] + ISLAND_GAP
                    && old.bounds[2] <= island.bounds[3] + ISLAND_GAP
            }) {
                existing.bounds[0] = existing.bounds[0].min(island.bounds[0]);
                existing.bounds[1] = existing.bounds[1].max(island.bounds[1]);
                existing.bounds[2] = existing.bounds[2].min(island.bounds[2]);
                existing.bounds[3] = existing.bounds[3].max(island.bounds[3]);
                existing.what.push('+');
                existing.what.push_str(&island.what);
            } else {
                merged.push(island);
            }
        }
        *boxes = merged;
    }
    Ok(raw)
}

fn leg_on_island(
    state: &CorridorState,
    net: &str,
    s: f64,
    layer: &str,
    islands: &BTreeMap<String, Vec<StaticIsland>>,
) -> bool {
    let Some(rank) = state.target.iter().position(|n| n == net) else {
        return false;
    };
    let Some(o_launch) = state.py.get(rank).copied() else {
        return false;
    };
    let Some(o_exit) = state.se.get(net).map(|point| point[1]) else {
        return false;
    };
    let (lo, hi) = (o_launch.min(o_exit), o_launch.max(o_exit));
    islands.get(layer).into_iter().flatten().any(|island| {
        let b = island.bounds;
        b[0] <= s && s <= b[1] && b[2] < hi && lo < b[3]
    })
}

fn island_edges(
    state: &CorridorState,
    net: &str,
    s: f64,
    layer: &str,
    islands: &BTreeMap<String, Vec<StaticIsland>>,
) -> Vec<f64> {
    let Some(rank) = state.target.iter().position(|n| n == net) else {
        return vec![];
    };
    let Some(o_exit) = state.se.get(net).map(|point| point[1]) else {
        return vec![];
    };
    let o_launch = state.py[rank];
    let (lo, hi) = (o_launch.min(o_exit), o_launch.max(o_exit));
    islands
        .get(layer)
        .into_iter()
        .flatten()
        .filter(|island| {
            let b = island.bounds;
            b[0] <= s && s <= b[1] && b[2] < hi && lo < b[3]
        })
        .flat_map(|island| [island.bounds[0] - 0.05, island.bounds[1] + 0.05])
        .collect()
}

fn leg_split_at(
    state: &CorridorState,
    net: &str,
    s: f64,
    layer: &str,
    islands: &BTreeMap<String, Vec<StaticIsland>>,
) -> Option<f64> {
    let target_rank = state.target.iter().position(|name| name == net)?;
    let destination_layer = state.context.dest_layer.get(net)?;
    if destination_layer == layer {
        return None;
    }
    let o_launch = *state.py.get(target_rank)?;
    let o_exit = state.se.get(net)?[1];
    let lo = o_launch.min(o_exit);
    let hi = o_launch.max(o_exit);
    let n = (hi - lo).abs();
    let sign = if o_launch > o_exit { 1.0 } else { -1.0 };
    let t_span = |bounds: [f64; 4]| {
        let a = bounds[2].max(lo);
        let b = bounds[3].min(hi);
        if sign > 0.0 {
            (a - o_exit, b - o_exit)
        } else {
            (o_exit - b, o_exit - a)
        }
    };
    let mut t_split: Option<f64> = None;
    for island in islands.get(layer).into_iter().flatten() {
        let b = island.bounds;
        if b[0] <= s && s <= b[1] && b[2] < hi && lo < b[3] {
            let (_, tb) = t_span(b);
            t_split = Some(t_split.unwrap_or(0.0).max(tb + state.via_need()));
        }
    }
    let t = t_split?;
    if t > n - state.via_need() {
        return None;
    }
    for island in islands.get(destination_layer).into_iter().flatten() {
        let b = island.bounds;
        if b[0] <= s && s <= b[1] && b[2] < hi && lo < b[3] {
            let (ta, _) = t_span(b);
            if ta < t + state.via_need() {
                return None;
            }
        }
    }
    let o_split = o_exit + sign * t;
    for other in &state.members {
        if other == net {
            continue;
        }
        let (o_m, s_end) = if let Some(o) = state.exit_block.get(other) {
            (*o, *state.exit_leg_s.get(other)?)
        } else {
            (*state.target_o.get(other)?, state.se.get(other)?[0])
        };
        if s_end > s + 0.05 && o_exit.min(o_split) < o_m && o_m < o_exit.max(o_split) {
            return None;
        }
    }
    Some(o_split)
}

fn need_at(schedule: &BraidSchedule, net: &str, _s: f64) -> Option<&'static str> {
    let page = schedule.page.get(net)?.as_deref()?;
    Some(other_layer(page))
}

/// Source inverse of `u_of_s`; flat reserved segments resolve to their last s.
pub fn s_of_u(state: &CorridorState, u: f64) -> f64 {
    let (ss, us) = (&state.s_pts, &state.u_pts);
    if ss.len() < 2 || us.len() < 2 {
        return state.s0;
    }
    let mut k = us
        .partition_point(|v| *v <= u)
        .saturating_sub(1)
        .min(us.len() - 2);
    if k >= ss.len() - 1 {
        k = ss.len() - 2;
    }
    if us[k + 1] > us[k] {
        ss[k] + (u - us[k]) * (ss[k + 1] - ss[k]) / (us[k + 1] - us[k])
    } else {
        ss[k + 1]
    }
}

fn round_teeth_ahead(
    state: &CorridorState,
    net: &str,
    s_a: f64,
    o_a: f64,
    s_b: f64,
    o_b: f64,
) -> Vec<Point> {
    let layer = state.context.tooth_layer.get(net);
    let mut ahead: Vec<_> = state
        .members
        .iter()
        .filter(|other| other.as_str() != net)
        .filter_map(|other| {
            state
                .st
                .get(other)
                .filter(|p| {
                    s_a + 1e-6 < p[0]
                        && p[0] < s_b - 1e-6
                        && state.context.tooth_layer.get(other) == layer
                })
                .map(|p| (p[0], p[1], other.clone()))
        })
        .collect();
    ahead.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut out = Vec::new();
    let (mut sa, mut oa) = (s_a, o_a);
    for (sk, ok, other) in ahead {
        let pitch = state.options.track_width
            + state.options.clearance
            + TOOTH_CLEAR_EXTRA
            + braid_offsets::lane_w(state, net)
            + braid_offsets::lane_w(state, &other);
        if sk - sa < pitch {
            continue;
        }
        let line = oa + (o_b - oa) * (sk - sa) / (s_b - sa).max(1e-9);
        if (line - ok).abs() >= pitch {
            continue;
        }
        let mut side = if o_b >= ok { 1.0 } else { -1.0 };
        if let (Some(direction), Some(end)) = (
            state.context.tooth_dir.get(&other),
            state.context.ends.get(&other),
        ) {
            let p = end.tooth;
            let after = state
                .spine
                .project_pt([p[0] + 0.1 * direction[0], p[1] + 0.1 * direction[1]]);
            let before = state.spine.project_pt(p);
            if (after[1] - before[1]).abs() > (after[0] - before[0]).abs() {
                side = if after[1] > before[1] { 1.0 } else { -1.0 };
            }
        }
        let ds = (sk - sa).max(1e-6);
        let mut h = pitch;
        for _ in 0..6 {
            h = h.min(3.0 * pitch.max(pitch * (ds.hypot(ok + side * h - oa) / ds)));
        }
        let ow = ok + side * h;
        out.push([sk, ow]);
        (sa, oa) = (sk, ow);
    }
    out
}

fn branch_inner(state: &CorridorState, net: &str) -> (Vec<(String, f64)>, i32) {
    let side = *state.exit_side.get(net).unwrap_or(&0);
    let mut block: Vec<_> = state
        .exit_block
        .keys()
        .filter(|n| state.exit_side.get(*n) == Some(&side))
        .cloned()
        .collect();
    block.sort_by(|a, b| {
        (side as f64 * state.launch_o[a]).total_cmp(&(side as f64 * state.launch_o[b]))
    });
    let index = block.iter().position(|n| n == net).unwrap_or(block.len());
    let inner = (0..index)
        .map(|k| {
            (
                block[k].clone(),
                (state.launch_o[&block[k + 1]] - state.launch_o[&block[k]]).abs(),
            )
        })
        .collect();
    (inner, side)
}

fn branch_ramps(state: &CorridorState, net: &str) -> Vec<(f64, f64)> {
    let (inner, _) = branch_inner(state, net);
    let tangent = 22.5_f64.to_radians().tan();
    let mut out = Vec::new();
    for (i, (_, gap)) in inner.iter().enumerate() {
        let Some(end) = state.exit_leg_s.get(&inner[i].0).copied() else {
            continue;
        };
        let mut distance = inner[i..].iter().map(|(_, g)| *g).sum::<f64>();
        distance -= inner[i + 1..]
            .iter()
            .filter(|(m, _)| state.exit_leg_s.get(m).is_some_and(|other| *other < end))
            .map(|(_, g)| *g)
            .sum::<f64>();
        out.push((end + tangent * distance.max(0.0), *gap));
    }
    out
}

fn branch_breaks(state: &CorridorState, net: &str) -> Vec<f64> {
    branch_ramps(state, net)
        .into_iter()
        .flat_map(|(s, g)| [s, s + g])
        .collect()
}

fn branch_o(state: &CorridorState, net: &str, s: f64) -> f64 {
    let (_, sign) = branch_inner(state, net);
    let ramps = branch_ramps(state, net);
    let origin = state.launch_o[net];
    let mut distance = 0.0;
    for (start, gap) in ramps {
        distance += gap * ((s - start) / gap.max(1e-6)).clamp(0.0, 1.0);
    }
    origin - sign as f64 * distance
}

fn turn_in(state: &CorridorState, net: &str, slot: f64) -> Option<f64> {
    let p = state.context.ends.get(net)?.berth;
    let d = *state.context.stub_dir.get(net)?;
    let end = state.spine.project_pt(p);
    let next = state
        .spine
        .project_pt([p[0] + d[0] * 0.1, p[1] + d[1] * 0.1]);
    let ds = (next[0] - end[0]) / 0.1;
    let do_ = (next[1] - end[1]) / 0.1;
    if do_.abs() < 0.2 || ds > -0.1 {
        return None;
    }
    let t = (slot - end[1]) / do_;
    let turn_s = end[0] + t * ds;
    let s_e = state.se.get(net)?[0];
    if t <= 0.0 || !(state.s1 + 0.05 < turn_s && turn_s < s_e - 0.02) {
        None
    } else {
        Some(turn_s)
    }
}

pub fn layer_profile(state: &CorridorState, net: &str) -> Result<Vec<LayerRun>, String> {
    let tooth = *state
        .st
        .get(net)
        .ok_or_else(|| format!("lane profile missing tooth {net}"))?;
    let berth = *state
        .se
        .get(net)
        .ok_or_else(|| format!("lane profile missing berth {net}"))?;
    let mut seq = vec![(
        tooth[0],
        state
            .context
            .tooth_layer
            .get(net)
            .cloned()
            .ok_or_else(|| format!("lane profile missing tooth layer {net}"))?,
    )];
    let mut req: Vec<_> = state.req.get(net).into_iter().flatten().cloned().collect();
    req.sort_by(|a, b| a.start.total_cmp(&b.start));
    seq.extend(req.into_iter().map(|i| (i.start, i.layer)));
    if let (Some(s), Some(layer)) = (state.exit_leg_s.get(net), state.leg_layer.get(net)) {
        seq.push((*s, layer.clone()));
    }
    seq.push((
        berth[0],
        state
            .context
            .dest_layer
            .get(net)
            .cloned()
            .ok_or_else(|| format!("lane profile missing destination layer {net}"))?,
    ));
    let mut out = Vec::new();
    for (s, layer) in seq {
        if out.last().is_none_or(|last: &LayerRun| last.layer != layer) {
            out.push(LayerRun { s, layer });
        }
    }
    Ok(out)
}

fn corner_dive(state: &CorridorState, net: &str) -> Result<bool, String> {
    let Some(&s) = state.exit_leg_s.get(net) else {
        return Ok(false);
    };
    let Some(layer) = state.leg_layer.get(net) else {
        return Ok(false);
    };
    if !state.is_branch {
        return Ok(true);
    }
    let profile = layer_profile(state, net)?;
    let before: Vec<_> = profile
        .iter()
        .filter(|run| run.s < s - 1e-6)
        .map(|run| run.layer.as_str())
        .collect();
    Ok(before
        .last()
        .copied()
        .unwrap_or(profile.first().ok_or("empty lane profile")?.layer.as_str())
        != layer)
}

fn allowed_vec(
    state: &CorridorState,
    net: &str,
    ss: &[f64],
    layer: &str,
) -> Result<Vec<bool>, String> {
    ss.iter()
        .map(|s| Ok(state.allowed(net, *s, layer)))
        .collect()
}

fn planned_vec(
    state: &CorridorState,
    net: &str,
    ss: &[f64],
    layer: &str,
    schedule: &BraidSchedule,
) -> Result<Vec<bool>, String> {
    let mut allowed = allowed_vec(state, net, ss, layer)?;
    if schedule.page_of(net).is_none() {
        return Ok(allowed);
    }
    let profile = layer_profile(state, net)?;
    let mut on = vec![false; ss.len()];
    for (index, run) in profile.iter().enumerate() {
        if run.layer != layer {
            continue;
        }
        let lo = if index == 0 { f64::NEG_INFINITY } else { run.s };
        let hi = profile.get(index + 1).map_or(f64::INFINITY, |next| next.s);
        for (i, s) in ss.iter().enumerate() {
            if *s >= lo && *s < hi {
                on[i] = true;
            }
        }
    }
    let both_f = allowed_vec(state, net, ss, "F.Cu")?;
    let both_b = allowed_vec(state, net, ss, "B.Cu")?;
    if profile.len() > 1 {
        let mut i = 0;
        while i < ss.len() {
            if !(both_f[i] && both_b[i]) {
                i += 1;
                continue;
            }
            let start = i;
            while i + 1 < ss.len() && both_f[i + 1] && both_b[i + 1] {
                i += 1;
            }
            let lo = ss[start];
            let hi = ss[i];
            if profile
                .iter()
                .skip(1)
                .any(|run| lo - 0.03 <= run.s && run.s <= hi + 0.03)
            {
                for value in &mut on[start..=i] {
                    *value = true;
                }
            }
            i += 1;
        }
    }
    for (target, planned) in allowed.iter_mut().zip(on) {
        *target &= planned;
    }
    Ok(allowed)
}

fn via_static_ok(
    state: &CorridorState,
    net: &str,
    point: Point,
    extra: f64,
) -> Result<bool, String> {
    let ctx = &state.context;
    let mut own = BTreeSet::new();
    own.insert(
        *ctx.byname
            .get(net)
            .ok_or_else(|| format!("via static check unknown net {net}"))?,
    );
    if let Some(pair) = ctx.pairs.get(net) {
        for member in pair {
            if let Some(id) = ctx.byname.get(member) {
                own.insert(*id);
            }
        }
    }
    let need = state.options.via_size / 2.0 + state.options.clearance + extra;
    for pad in &ctx.pads {
        if own.contains(&pad.net_id) || pad.pad_type == "np_thru_hole" {
            continue;
        }
        let dx = (point[0] - pad.point[0]).abs();
        let dy = (point[1] - pad.point[1]).abs();
        if dx > 2.0 || dy > 2.0 {
            continue;
        }
        let d = if pad.shape == "circle" {
            (point[0] - pad.point[0]).hypot(point[1] - pad.point[1]) - pad.size[0] / 2.0
        } else {
            let x = dx - pad.size[0] / 2.0;
            let y = dy - pad.size[1] / 2.0;
            x.max(0.0).hypot(y.max(0.0)) + x.max(y).min(0.0)
        };
        if d < need {
            return Ok(false);
        }
    }
    for segment in &ctx.base_segments {
        if own.contains(&segment.net_id)
            || (segment.a[0] - point[0]).abs() > 3.0
            || (segment.a[1] - point[1]).abs() > 3.0
        {
            continue;
        }
        if corridor::point_segment_distance(point, segment.a, segment.b) - segment.width / 2.0
            < need
        {
            return Ok(false);
        }
    }
    for via in &ctx.base_vias {
        if !own.contains(&via.net_id)
            && (corridor::distance(point, via.point) - via.diameter / 2.0 < need)
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn simplify_so(points: &[(f64, f64)], tolerance: f64) -> Vec<(f64, f64)> {
    if points.len() < 3 {
        return points.to_vec();
    }
    fn recurse(points: &[(f64, f64)], lo: usize, hi: usize, tol: f64, keep: &mut BTreeSet<usize>) {
        if hi <= lo + 1 {
            return;
        }
        let a = points[lo];
        let b = points[hi];
        let dx = b.0 - a.0;
        let dy = b.1 - a.1;
        let mut best = (0.0, lo);
        for i in lo + 1..hi {
            let d = if dx * dx + dy * dy < 1e-18 {
                (points[i].0 - a.0).hypot(points[i].1 - a.1)
            } else {
                ((dy * points[i].0 - dx * points[i].1 + b.0 * a.1 - b.1 * a.0).abs())
                    / (dx * dx + dy * dy).sqrt()
            };
            if d > best.0 {
                best = (d, i);
            }
        }
        if best.0 > tol {
            keep.insert(best.1);
            recurse(points, lo, best.1, tol, keep);
            recurse(points, best.1, hi, tol, keep);
        }
    }
    let mut keep = BTreeSet::from([0, points.len() - 1]);
    recurse(points, 0, points.len() - 1, tolerance, &mut keep);
    keep.into_iter().map(|i| points[i]).collect()
}

/// Deflect scheduled page lanes around static copper islands, retaining the
/// source's near-corner choice, outward ordering, and tail/leg endpoint rules.
pub fn deflect_islands(state: &mut CorridorState, schedule: &BraidSchedule) -> Result<(), String> {
    let islands = static_islands(state)?;
    state.islands_by_layer = islands.clone();
    state.deflect_islands.clear();
    let o_at = |state: &CorridorState, net: &str, s: f64| {
        state
            .mid
            .get(net)
            .and_then(|line| interpolate_line(line, s))
    };
    for layer in ["F.Cu", "B.Cu"] {
        for island in islands.get(layer).into_iter().flatten() {
            let [mut s_lo, mut s_hi, o_lo, o_hi] = island.bounds;
            let in_tail = s_lo >= state.s1 - 0.1;
            if in_tail {
                s_lo = s_lo.max(state.s1 + 0.1);
            } else {
                s_lo = s_lo.max(state.s0 + 0.1);
                s_hi = s_hi.min(state.s1 - 0.1);
            }
            if s_hi <= s_lo {
                continue;
            }
            let center = (s_lo + s_hi) / 2.0;
            let mut on_layer = Vec::<(f64, String)>::new();
            for net in &state.members {
                if schedule.page_of(net).is_none() {
                    continue;
                }
                if !state.allowed(net, center, layer) {
                    continue;
                }
                if !in_tail
                    && state.allowed(net, center, other_layer(layer))
                    && schedule.page_of(net) != Some(layer)
                {
                    continue;
                }
                let Some(o) = o_at(state, net, center) else {
                    continue;
                };
                on_layer.push((o, net.clone()));
            }
            let inside: Vec<_> = on_layer
                .iter()
                .filter(|(o, net)| {
                    let a = o_at(state, net, s_lo).unwrap_or(*o);
                    let b = o_at(state, net, s_hi).unwrap_or(*o);
                    (*o > o_lo && *o < o_hi) || (a > o_lo && a < o_hi) || (b > o_lo && b < o_hi)
                })
                .cloned()
                .collect();
            if inside.is_empty() {
                continue;
            }
            let mut side_of = BTreeMap::<String, i32>::new();
            for (o, net) in &inside {
                if in_tail && state.exit_block.contains_key(net) {
                    side_of.insert(net.clone(), *state.exit_side.get(net).unwrap_or(&1));
                    continue;
                }
                let a = o_at(state, net, s_lo).unwrap_or(*o);
                let b = o_at(state, net, s_hi).unwrap_or(*o);
                let north = a.max(b) - o_lo;
                let south = o_hi - a.min(b);
                side_of.insert(net.clone(), if north <= south { -1 } else { 1 });
            }
            let others: Vec<_> = islands
                .get(layer)
                .into_iter()
                .flatten()
                .filter(|box_| {
                    box_.what != island.what && box_.bounds[0] <= s_hi && s_lo <= box_.bounds[1]
                })
                .collect();
            let off_islands = |mut offset: f64, sign: i32| {
                for _ in 0..8 {
                    let hit: Vec<_> = others
                        .iter()
                        .filter(|box_| box_.bounds[2] < offset && offset < box_.bounds[3])
                        .collect();
                    if hit.is_empty() {
                        return offset;
                    }
                    offset = if sign < 0 {
                        hit.iter()
                            .map(|b| b.bounds[2])
                            .fold(f64::INFINITY, f64::min)
                    } else {
                        hit.iter()
                            .map(|b| b.bounds[3])
                            .fold(f64::NEG_INFINITY, f64::max)
                    };
                }
                offset
            };
            let mut wants = BTreeMap::<String, f64>::new();
            for sign in [-1, 1] {
                let edge = if sign < 0 { o_lo } else { o_hi };
                let mut group: Vec<_> = on_layer
                    .iter()
                    .filter(|(o, net)| {
                        if let Some(&sg) = side_of.get(net) {
                            sg == sign
                        } else if sign < 0 {
                            *o <= edge
                        } else {
                            *o >= edge
                        }
                    })
                    .cloned()
                    .collect();
                group.sort_by(|a, b| (sign as f64 * a.0).total_cmp(&(sign as f64 * b.0)));
                let mut previous: Option<(f64, f64)> = None;
                for (offset, net) in group {
                    let limit = previous.map_or(edge, |(old, new)| {
                        new + sign as f64 * (state.options.exit_pitch.min((offset - old).abs()))
                    });
                    let candidate = if sign < 0 {
                        offset.min(limit)
                    } else {
                        offset.max(limit)
                    };
                    let candidate = off_islands(candidate, sign);
                    if (candidate - offset).abs() > 1e-9 {
                        wants.insert(net.clone(), candidate);
                    }
                    previous = Some((offset, candidate));
                }
            }
            if wants.is_empty() {
                continue;
            }
            for (net, new_offset) in &wants {
                let old = state.mid.get(net).cloned().unwrap_or_default();
                if old.len() < 2 {
                    continue;
                }
                let Some(o_in) = o_at(state, net, s_lo) else {
                    continue;
                };
                let Some(o_out) = o_at(state, net, s_hi) else {
                    continue;
                };
                let d_in = 0.3_f64.max((new_offset - o_in).abs());
                let d_out = 0.3_f64.max((new_offset - o_out).abs());
                let (s_a, s_b) = if in_tail {
                    let end = if state.exit_block.contains_key(net) {
                        state.exit_leg_s[net].min(state.se[net][0])
                    } else {
                        state.se[net][0]
                    };
                    (
                        (s_lo - d_in).max(state.s1 + 0.05),
                        (s_hi + d_out).min(end - 0.05),
                    )
                } else {
                    (
                        (s_lo - d_in).max(state.s0 + 0.05),
                        (s_hi + d_out).min(state.s1 - 0.05),
                    )
                };
                if s_a >= s_lo {
                    continue;
                }
                if s_b <= s_hi {
                    if in_tail
                        && state.exit_block.contains_key(net)
                        && state.exit_leg_s[net] > s_hi
                        && s_hi + 0.05 >= s_lo
                    {
                        let leg_s = state.exit_leg_s[net];
                        let oa = o_at(state, net, s_a).unwrap_or(o_in);
                        let mut kept: Vec<_> = old
                            .iter()
                            .copied()
                            .filter(|p| p[0] < s_a - 1e-9 || p[0] > leg_s + 1e-9)
                            .collect();
                        kept.extend([[s_a, oa], [s_lo, *new_offset], [leg_s, *new_offset]]);
                        kept.sort_by(|a, b| a[0].total_cmp(&b[0]));
                        state.mid.insert(net.clone(), kept);
                        if let Some(last) = state.legs.get_mut(net).and_then(|legs| legs.last_mut())
                        {
                            last.1 = *new_offset;
                        }
                    }
                    continue;
                }
                let oa = o_at(state, net, s_a).unwrap_or(o_in);
                let ob = o_at(state, net, s_b).unwrap_or(o_out);
                let mut kept: Vec<_> = old
                    .into_iter()
                    .filter(|p| p[0] < s_a - 1e-9 || p[0] > s_b + 1e-9)
                    .collect();
                kept.extend([
                    [s_a, oa],
                    [s_lo, *new_offset],
                    [s_hi, *new_offset],
                    [s_b, ob],
                ]);
                kept.sort_by(|a, b| a[0].total_cmp(&b[0]));
                state.mid.insert(net.clone(), kept);
            }
            for (net, offset) in wants {
                state
                    .deflect_islands
                    .entry(net)
                    .or_default()
                    .push(state.spine.xy(center, offset));
            }
        }
    }
    Ok(())
}

fn order_ring(state: &mut CorridorState, schedule: &BraidSchedule) -> Result<(), String> {
    let mut order: Vec<_> = state
        .members
        .iter()
        .filter(|net| {
            state.exit_block.contains_key(*net)
                && state.mid.get(*net).is_some_and(|line| !line.is_empty())
        })
        .cloned()
        .collect();
    if order.len() < 2 {
        return Ok(());
    }
    let side = *state.exit_side.get(&order[0]).unwrap_or(&1);
    order.sort_by(|a, b| {
        (side as f64 * state.launch_o[a]).total_cmp(&(side as f64 * state.launch_o[b]))
    });
    let ds = 0.025;
    let s_lo = order
        .iter()
        .map(|net| state.mid[net][0][0])
        .fold(f64::INFINITY, f64::min);
    let s_hi = order
        .iter()
        .map(|net| state.exit_leg_s[net])
        .fold(f64::NEG_INFINITY, f64::max);
    let n = ((s_hi - s_lo) / ds).ceil().max(0.0) as usize + 1;
    let sample: Vec<_> = (0..n).map(|i| s_lo + i as f64 * ds).collect();
    let mut u = BTreeMap::<String, Vec<Option<f64>>>::new();
    let mut profiles = BTreeMap::new();
    for net in &order {
        let line = &state.mid[net];
        let end = state.exit_leg_s[net];
        let profile = layer_profile(state, net)?;
        profiles.insert(net.clone(), profile);
        let values = sample
            .iter()
            .map(|s| {
                if *s < line[0][0] - 1e-9 || *s > end + 1e-9 {
                    None
                } else {
                    interpolate_line(line, *s).map(|o| side as f64 * o)
                }
            })
            .collect();
        u.insert(net.clone(), values);
    }
    let mut corner_via = BTreeMap::new();
    for net in &order {
        let e = state.exit_leg_s[net];
        let before: Vec<_> = profiles[net]
            .iter()
            .filter(|run| run.s < e - 1e-6)
            .map(|run| run.layer.as_str())
            .collect();
        let leg = state
            .leg_layer
            .get(net)
            .map(String::as_str)
            .unwrap_or(&state.context.dest_layer[net]);
        corner_via.insert(
            net.clone(),
            before
                .last()
                .copied()
                .unwrap_or(profiles[net][0].layer.as_str())
                != leg,
        );
    }
    let mut dive_s = BTreeMap::new();
    for net in &order {
        if !corner_via[net] {
            continue;
        }
        let e = state.exit_leg_s[net];
        let pins: Vec<_> = state
            .req
            .get(net)
            .into_iter()
            .flatten()
            .filter(|run| run.start < e)
            .map(|run| run.end)
            .collect();
        let lo = pins
            .into_iter()
            .reduce(f64::max)
            .map_or(sample[0], |s| s + 0.2);
        let legs: Vec<_> = order
            .iter()
            .filter(|other| *other != net && state.legs.get(*other).is_some_and(|v| !v.is_empty()))
            .map(|other| {
                (
                    state.legs[other][state.legs[other].len() - 1],
                    state.via_need()
                        + braid_offsets::lane_w(state, net)
                        + braid_offsets::lane_w(state, other),
                )
            })
            .collect();
        let mut chosen = e;
        for i in (0..sample.partition_point(|s| *s <= e + 1e-9)).rev() {
            if sample[i] < lo {
                break;
            }
            let Some(offset) = u[net][i] else {
                continue;
            };
            let o = side as f64 * offset;
            let blocked = legs.iter().any(|((s, oa, ob), r)| {
                (s - sample[i]).abs() < *r && oa.min(*ob) - r < o && o < oa.max(*ob) + r
            });
            if !blocked {
                chosen = sample[i];
                break;
            }
        }
        dive_s.insert(net.clone(), chosen);
    }
    state.dive_s_plan = dive_s.clone();
    let islands = static_islands(state)?;
    state.islands_by_layer = islands.clone();
    let mut planned = BTreeMap::<String, BTreeMap<String, Vec<bool>>>::new();
    for net in &order {
        for layer in ["F.Cu", "B.Cu"] {
            planned.entry(net.clone()).or_default().insert(
                layer.to_owned(),
                planned_vec(state, net, &sample, layer, schedule)?,
            );
        }
    }
    for (index, net) in order.iter().enumerate() {
        let e = state.exit_leg_s[net];
        let u0 = side as f64 * state.launch_o[net];
        let mut floor = vec![f64::NEG_INFINITY; sample.len()];
        for (layer, boxes) in &islands {
            let opposite = other_layer(layer);
            for island in boxes {
                let b = island.bounds;
                let (u_lo, u_hi) = (
                    (side as f64 * b[2]).min(side as f64 * b[3]),
                    (side as f64 * b[2]).max(side as f64 * b[3]),
                );
                if u0 <= u_hi || e < b[0] {
                    continue;
                }
                for (i, s) in sample.iter().enumerate() {
                    if state.allowed(net, *s, layer) && !state.allowed(net, *s, opposite) {
                        let d = (b[0] - s).max(s - b[1]).max(0.0);
                        let candidate = u_hi + 0.02 - d;
                        if candidate > u_lo {
                            floor[i] = floor[i].max(candidate);
                        }
                    }
                }
            }
        }
        if corner_via[net] {
            let dive = dive_s[net];
            let i = sample
                .partition_point(|s| *s <= dive + 1e-9)
                .saturating_sub(1);
            let extra = (state.options.via_size - state.options.track_width) / 2.0 + 0.02;
            for boxes in islands.values() {
                for island in boxes {
                    let b = island.bounds;
                    let (_u_lo, u_hi) = (
                        (side as f64 * b[2]).min(side as f64 * b[3]),
                        (side as f64 * b[2]).max(side as f64 * b[3]),
                    );
                    if u0 > u_hi || dive < b[0] - extra || dive > b[1] + extra {
                        continue;
                    }
                    if u[net][i].is_some_and(|v| v < u_hi + extra) {
                        floor[i] = floor[i].max(u_hi + extra);
                    }
                }
            }
        }
        for other in &order[..index] {
            let width = braid_offsets::lane_w(state, net) + braid_offsets::lane_w(state, other);
            let spacing = state.lane_min() + width;
            let radius = state.via_need() + width;
            let Some(values) = u.get(other) else {
                continue;
            };
            let shared: Vec<_> = sample
                .iter()
                .enumerate()
                .filter_map(|(i, s)| {
                    let same = planned[net]["F.Cu"][i] && planned[other]["F.Cu"][i]
                        || planned[net]["B.Cu"][i] && planned[other]["B.Cu"][i];
                    if same {
                        values[i].map(|value| (i, *s, value))
                    } else {
                        None
                    }
                })
                .collect();
            for (j, s) in sample.iter().enumerate() {
                for (_i, so, value) in &shared {
                    let d = (*s - *so).abs();
                    if d <= spacing {
                        floor[j] =
                            floor[j].max(*value + (spacing * spacing - d * d).max(0.0).sqrt());
                    }
                }
                if corner_via[net]
                    && *s >= dive_s[net] - radius
                    && *s <= e.min(dive_s[net] + radius)
                {
                    if let Some(value) = values[j] {
                        let d = (*s - dive_s[net]).abs();
                        floor[j] = floor[j].max(value + (radius * radius - d * d).max(0.0).sqrt());
                    }
                }
                if corner_via[other] {
                    let dive = dive_s[other];
                    let ix = sample
                        .partition_point(|v| *v <= dive + 1e-9)
                        .saturating_sub(1);
                    if let Some(value) = values[ix] {
                        let d = (*s - dive).abs();
                        if d <= radius {
                            floor[j] =
                                floor[j].max(value + (radius * radius - d * d).max(0.0).sqrt());
                        }
                    }
                }
            }
        }
        let mut values: Vec<Option<f64>> = u[net]
            .iter()
            .enumerate()
            .map(|(i, value)| value.map(|old| if floor[i] > old + 1e-6 { floor[i] } else { old }))
            .collect();
        let live: Vec<_> = values
            .iter()
            .enumerate()
            .filter_map(|(i, v)| v.map(|_| i))
            .collect();
        if live.is_empty() {
            continue;
        }
        let mut compact: Vec<_> = live.iter().map(|i| values[*i].unwrap()).collect();
        for i in 1..compact.len() {
            compact[i] = compact[i].max(compact[i - 1] - ds);
        }
        for i in (1..compact.len().saturating_sub(1)).rev() {
            compact[i] = compact[i].max(compact[i + 1] - ds);
        }
        let max_dip = (RING_DIP / ds).round().max(1.0) as usize;
        for _ in 0..compact.len() {
            let mut changed = false;
            let mut i = 1;
            while i + 1 < compact.len() {
                if !(compact[i] < compact[i - 1] - 1e-9 && compact[i] <= compact[i + 1] + 1e-9) {
                    i += 1;
                    continue;
                }
                let (mut left, mut right) = (i, i);
                while left > 0 && compact[left - 1] >= compact[i] - 1e-9 {
                    left -= 1;
                }
                while right + 1 < compact.len() && compact[right + 1] >= compact[i] - 1e-9 {
                    right += 1;
                }
                if left == i || right == i {
                    i += 1;
                    continue;
                }
                let level = compact[left].min(compact[right]);
                let (mut a, mut b) = (i, i);
                while a > left && compact[a - 1] < level - 1e-9 {
                    a -= 1;
                }
                while b < right && compact[b + 1] < level - 1e-9 {
                    b += 1;
                }
                if b - a + 1 <= max_dip {
                    for v in &mut compact[a..=b] {
                        *v = level;
                    }
                    changed = true;
                }
                i = b + 1;
            }
            if !changed {
                break;
            }
        }
        for (i, v) in live.iter().zip(compact) {
            values[*i] = Some(v);
        }
        let mut points: Vec<_> = values
            .iter()
            .enumerate()
            .filter_map(|(i, v)| {
                if sample[i] < e - 1e-9 {
                    v.map(|value| (sample[i], side as f64 * value))
                } else {
                    None
                }
            })
            .collect();
        let o_end = interpolate(&points, e).unwrap_or_else(|| {
            side as f64 * state.py[state.target.iter().position(|n| n == net).unwrap()]
        });
        points.push((e, o_end));
        if points.is_empty() {
            continue;
        }
        let first = points[0].0;
        let old = state.mid[net].clone();
        let mut keep: Vec<_> = old
            .into_iter()
            .filter(|p| p[0] < first - 1e-9 || p[0] > e + 1e-9)
            .collect();
        keep.extend(simplify_so(&points, 0.004).into_iter().map(|(s, o)| [s, o]));
        keep.sort_by(|a, b| a[0].total_cmp(&b[0]));
        state.mid.insert(net.clone(), keep);
        if let Some(last) = state.legs.get_mut(net).and_then(|legs| legs.last_mut()) {
            last.1 = o_end;
        }
    }
    Ok(())
}

fn place_and_decide(
    state: &mut CorridorState,
    schedule: &BraidSchedule,
    islands: &BTreeMap<String, Vec<StaticIsland>>,
    pre: &BTreeMap<String, Vec<(f64, f64, String)>>,
    avoid: Option<&braid_offsets::LegAvoid<'_>>,
    extra_cands: &BTreeMap<String, Vec<f64>>,
) -> Result<
    (
        BTreeMap<String, Vec<(f64, f64, String)>>,
        BTreeMap<String, f64>,
        BTreeMap<String, Vec<String>>,
    ),
    String,
> {
    let members = state.members.clone();
    let target_rank: BTreeMap<_, _> = state
        .target
        .iter()
        .enumerate()
        .map(|(i, n)| (n.clone(), i))
        .collect();
    let mut placed = Vec::<(f64, f64, f64)>::new();
    let mut placed_by = BTreeMap::<String, (f64, f64, f64)>::new();
    state.exit_leg_s.clear();
    let mut exit_order: Vec<_> = state
        .members
        .iter()
        .filter(|n| state.exit_block.contains_key(*n))
        .cloned()
        .collect();
    exit_order.sort_by(|a, b| {
        let ka = if state.far_exit.contains(a) {
            (1, state.exit_block[a].abs(), state.se[a][0])
        } else {
            (0, state.se[a][0], state.exit_block[a].abs())
        };
        let kb = if state.far_exit.contains(b) {
            (1, state.exit_block[b].abs(), state.se[b][0])
        } else {
            (0, state.se[b][0], state.exit_block[b].abs())
        };
        ka.0.cmp(&kb.0)
            .then(ka.1.total_cmp(&kb.1))
            .then(ka.2.total_cmp(&kb.2))
    });
    for net in &exit_order {
        let end = state.se[net];
        let offset = state.py[target_rank[net]];
        let mut base = end[0];
        let mut avoid_fn: Option<Box<dyn Fn(&str, f64) -> bool>> = None;
        if state.far_exit.contains(net) {
            let floor = state
                .s_leg_min
                .ok_or_else(|| format!("far exit {net} has no minimum leg s"))?;
            base = base.max(floor);
            let existing = avoid;
            avoid_fn = Some(Box::new(move |name, s| {
                s < floor - 1e-9 || existing.is_some_and(|f| f(name, s))
            }));
        }
        let avoid_ref = avoid_fn
            .as_ref()
            .map(|f| &**f as &braid_offsets::LegAvoid<'_>)
            .or(avoid);
        let extra = if avoid.is_some() {
            extra_cands.get(net).map(Vec::as_slice).unwrap_or(&[])
        } else {
            &[]
        };
        let leg_s = braid_offsets::leg_s(
            state, net, base, offset, end[1], false, &placed, avoid_ref, extra,
        )?;
        state.exit_leg_s.insert(net.clone(), leg_s);
        if state.options.pairs && state.is_branch && state.context.pairs.contains_key(net) {
            let half = super::pairs::pitch(&state.context.cfg) / 2.0;
            placed.push((leg_s - half, offset.min(end[1]), offset.max(end[1])));
            placed.push((leg_s + half, offset.min(end[1]), offset.max(end[1])));
        } else {
            placed.push((leg_s, offset.min(end[1]), offset.max(end[1])));
        }
        placed_by.insert(net.clone(), *placed.last().unwrap());
    }
    state.leg_layer.clear();
    let mut crossings = BTreeMap::<String, Vec<f64>>::new();
    let mut leg_cross = BTreeMap::<String, Vec<String>>::new();
    for side in [-1, 1] {
        let group: Vec<_> = exit_order
            .iter()
            .filter(|net| state.exit_side.get(*net).copied() == Some(side))
            .cloned()
            .collect();
        if group.is_empty() {
            continue;
        }
        let b_count = group
            .iter()
            .filter(|net| {
                state
                    .context
                    .dest_layer
                    .get(*net)
                    .is_some_and(|l| l == "B.Cu")
            })
            .count();
        let layer = if 2 * b_count > group.len() {
            "B.Cu"
        } else {
            "F.Cu"
        };
        for net in &group {
            state.leg_layer.insert(net.clone(), layer.to_owned());
            let s_leg = state.exit_leg_s[net];
            let exit = state.exit_block[net];
            let offset = state.py[target_rank[net]];
            let lo = offset.min(exit);
            let hi = offset.max(exit);
            for other in &members {
                if other == net {
                    continue;
                }
                let (other_o, end_s) = if let Some(o) = state.exit_block.get(other) {
                    (
                        *o,
                        state
                            .exit_leg_s
                            .get(other)
                            .copied()
                            .unwrap_or(state.se[other][0]),
                    )
                } else {
                    (state.target_o[other], state.se[other][0])
                };
                if end_s > s_leg + 0.05
                    && (lo < other_o && other_o < hi || (other_o - exit).abs() < state.leg_offset())
                {
                    crossings.entry(other.clone()).or_default().push(s_leg);
                    leg_cross
                        .entry(net.clone())
                        .or_default()
                        .push(other.clone());
                }
            }
        }
    }
    state.crossings.clear();
    for (crossed, ss) in &crossings {
        let mut owners = Vec::new();
        for (owner, list) in &leg_cross {
            if list.iter().any(|name| name == crossed) {
                if let Some(s) = state.exit_leg_s.get(owner) {
                    owners.push((owner.clone(), *s));
                }
            }
        }
        let mut entries: Vec<_> = owners.into_iter().map(|(owner, s)| (s, owner)).collect();
        entries.sort_by(|a, b| a.0.total_cmp(&b.0));
        state.crossings.insert(crossed.clone(), entries);
        let _ = ss;
    }

    let mut ivs = pre.clone();
    for net in &members {
        ivs.entry(net.clone()).or_default();
    }
    let mut leg_req_min = BTreeMap::<String, f64>::new();
    let mut decided = BTreeSet::new();
    let current_layer =
        |ivs: &BTreeMap<String, Vec<(f64, f64, String)>>, net: &str, s: f64| -> Option<String> {
            ivs.get(net)
                .and_then(|items| {
                    items
                        .iter()
                        .filter(|item| item.0 < s)
                        .max_by(|a, b| a.0.total_cmp(&b.0))
                        .map(|item| item.2.clone())
                })
                .or_else(|| schedule.page.get(net).cloned().flatten())
        };
    let move_cost = |state: &CorridorState,
                     net: &str,
                     s_leg: f64,
                     layer: &str,
                     placed: &[(f64, f64, f64)],
                     islands: &BTreeMap<String, Vec<StaticIsland>>,
                     extra: &[f64]|
     -> Result<f64, String> {
        let offset = state.py[target_rank[net]];
        let end = state.se[net][1];
        let floor = if state.far_exit.contains(net) {
            state.s_leg_min
        } else {
            None
        };
        let avoid = |name: &str, s: f64| {
            leg_on_island(state, name, s, layer, islands) || floor.is_some_and(|min| s < min - 1e-9)
        };
        let moved = braid_offsets::leg_s(
            state,
            net,
            s_leg,
            offset,
            end,
            false,
            placed,
            Some(&avoid),
            extra,
        )?;
        if (moved - s_leg).abs() < 1e-9 || leg_on_island(state, net, moved, layer, islands) {
            Ok(ISLAND_VETO)
        } else {
            Ok(
                (state.options.jog_via * (moved - s_leg).abs() / state.options.lane_pitch)
                    .min(ISLAND_VETO),
            )
        }
    };
    let mut layer_order = exit_order.clone();
    layer_order.sort_by(|a, b| state.exit_leg_s[a].total_cmp(&state.exit_leg_s[b]));
    for net in &layer_order {
        let s_leg = state.exit_leg_s[net];
        let own = current_layer(&ivs, net, s_leg);
        let crossed = leg_cross.get(net).cloned().unwrap_or_default();
        let mut costs = BTreeMap::new();
        for layer in ["F.Cu", "B.Cu"] {
            let mut cost = 0.0;
            let offset = state.py[target_rank[net]];
            let exit = state.se[net][1];
            let lo = offset.min(exit) - state.leg_offset();
            let hi = offset.max(exit) + state.leg_offset();
            let mut clash = false;
            for other in &members {
                if other == net {
                    continue;
                }
                let sep = state.options.lane_pitch
                    + braid_offsets::lane_w(state, net)
                    + braid_offsets::lane_w(state, other);
                if decided.contains(other) && state.leg_layer.get(other).is_some_and(|l| l == layer)
                {
                    let other_s = state
                        .exit_leg_s
                        .get(other)
                        .copied()
                        .unwrap_or(f64::INFINITY);
                    let a = state.py[target_rank[other]].min(state.se[other][1]);
                    let b = state.py[target_rank[other]].max(state.se[other][1]);
                    if (other_s - s_leg).abs() < sep && a < hi && b > lo {
                        clash = true;
                    }
                }
                if state
                    .context
                    .dest_layer
                    .get(other)
                    .is_some_and(|l| l == layer)
                {
                    let other_point = state.se[other];
                    let (other_s, other_o) = (other_point[0], other_point[1]);
                    if (other_s - s_leg).abs() < sep && lo < other_o && other_o < hi {
                        clash = true;
                    }
                }
            }
            if clash {
                cost += ISLAND_VETO;
            }
            for other in &crossed {
                if current_layer(&ivs, other, s_leg).as_deref() == Some(layer) {
                    cost += 1.0
                        + if state
                            .context
                            .dest_layer
                            .get(other)
                            .is_some_and(|l| l == layer)
                        {
                            1.0
                        } else {
                            0.0
                        };
                }
            }
            if own.as_deref().is_some_and(|l| l != layer) {
                cost += 1.0;
            }
            if state
                .context
                .dest_layer
                .get(net)
                .is_some_and(|l| l != layer)
            {
                cost += 1.0;
            }
            if leg_on_island(state, net, s_leg, layer, islands)
                && leg_split_at(state, net, s_leg, layer, islands).is_none()
            {
                let others: Vec<_> = placed_by
                    .iter()
                    .filter(|(name, _)| name.as_str() != net)
                    .map(|(_, placed)| *placed)
                    .collect();
                cost += move_cost(
                    state,
                    net,
                    s_leg,
                    layer,
                    &others,
                    islands,
                    extra_cands.get(net).map(Vec::as_slice).unwrap_or(&[]),
                )?;
            }
            costs.insert(layer, cost);
        }
        let chosen = if costs["F.Cu"] <= costs["B.Cu"] {
            "F.Cu"
        } else {
            "B.Cu"
        };
        state.leg_layer.insert(net.clone(), chosen.to_owned());
        decided.insert(net.clone());
        let other = other_layer(chosen);
        let a = s_leg - LEG_REQ;
        for crossed_net in crossed {
            let mut b = s_leg + LEG_REQ;
            if state.exit_block.contains_key(&crossed_net) {
                b = b.min(state.exit_leg_s[&crossed_net] - 0.03);
            } else {
                b = b.min(state.se[&crossed_net][0] - 0.03);
            }
            if b > a {
                if schedule.page_of(&crossed_net) != Some(other) {
                    leg_req_min
                        .entry(crossed_net.clone())
                        .and_modify(|v| *v = v.min(a))
                        .or_insert(a);
                }
                ivs.entry(crossed_net)
                    .or_default()
                    .push((a, b, other.to_owned()));
            }
        }
    }
    Ok((ivs, leg_req_min, leg_cross))
}

fn build_midlines(
    state: &mut CorridorState,
    schedule: &BraidSchedule,
    members: &[String],
    target_rank: &BTreeMap<String, usize>,
) -> Result<(), String> {
    state.mid.clear();
    state.legs.clear();
    state.jogs.clear();
    for net in members {
        let [s_t, o_t] = state.st[net];
        let [s_e, o_e] = state.se[net];
        let mut legs = Vec::new();
        let mut jogs = Vec::new();
        let mut points = Vec::<Point>::new();
        if let Some(offset) = state.join_block.get(net) {
            let s = state.join_leg_s[net];
            if (s - s_t).abs() > 1e-9 {
                jogs.push(([s_t, o_t], [s, o_t]));
            }
            legs.push((s, o_t, *offset));
            points.push([s, *offset]);
        } else {
            points.push([s_t, o_t]);
            points.extend(
                round_teeth_ahead(
                    state,
                    net,
                    s_t,
                    o_t,
                    s_of_u(state, 0.0),
                    state.launch_o[net],
                )
                .into_iter()
                .map(|p| [p[0], p[1]]),
            );
        }
        points.push([s_of_u(state, 0.0), state.launch_o[net]]);
        points.push([state.s1, state.py[target_rank[net]]]);
        if state.exit_block.contains_key(net) && state.is_branch {
            let s = state.exit_leg_s[net];
            for start in branch_breaks(state, net) {
                if state.s1 < start && start < s {
                    points.push([start, branch_o(state, net, start)]);
                }
            }
            points.push([s, branch_o(state, net, s)]);
            legs.push((s, branch_o(state, net, s), o_e));
            if (s - s_e).abs() > 1e-9 {
                jogs.push(([s, o_e], [s_e, o_e]));
            }
        } else if state.exit_block.contains_key(net) {
            let s = state.exit_leg_s[net];
            points.push([if s < s_e { s } else { s_e }, state.py[target_rank[net]]]);
            if s > s_e + 1e-9 {
                points.push([s, state.py[target_rank[net]]]);
            }
            legs.push((s, state.py[target_rank[net]], o_e));
            if (s - s_e).abs() > 1e-9 {
                jogs.push(([s, o_e], [s_e, o_e]));
            }
        } else {
            if state.row_slot.contains_key(net) {
                let turn = turn_in(state, net, state.py[target_rank[net]]).or_else(|| {
                    state
                        .row_up
                        .get(net)
                        .filter(|v| !v.is_empty())
                        .map(|near| {
                            near.iter()
                                .map(|n| state.se[n][0])
                                .fold(f64::NEG_INFINITY, f64::max)
                                + state.options.lane_pitch
                        })
                        .filter(|s| state.s1 + 0.05 < *s && *s < s_e - 0.02)
                });
                if let Some(turn) = turn {
                    points.push([turn, state.py[target_rank[net]]]);
                }
            }
            points.push([s_e, o_e]);
        }
        let mut clean = Vec::<Point>::new();
        for p in points {
            if clean.last().is_none_or(|old| p[0] >= old[0] - 1e-9) {
                clean.push([p[0], clean.last().map_or(p[0], |old| p[0].max(old[0]))]);
            }
        }
        if clean.is_empty() {
            return Err(format!("lay_lanes emitted empty centerline for {net}"));
        }
        state.mid.insert(net.clone(), clean);
        state.legs.insert(net.clone(), legs);
        state.jogs.insert(net.clone(), jogs);
    }
    let _ = schedule;
    Ok(())
}

fn add_corner_points(state: &mut CorridorState, members: &[String]) -> Result<(), String> {
    let corners: Vec<_> = (1..state.spine.pts.len().saturating_sub(1))
        .filter(|j| state.spine.turn[*j - 1].abs() >= 1.0)
        .map(|j| state.spine.s[j])
        .collect();
    for net in members {
        let old = state.mid[net].clone();
        if old.len() < 2 {
            continue;
        }
        let mut added = Vec::new();
        for s in &corners {
            if old[0][0] + 0.01 < *s && *s < old[old.len() - 1][0] - 0.01 {
                if let Some(o) = interpolate_line(&old, *s) {
                    added.push([s - 0.002, o]);
                    added.push([s + 0.002, o]);
                }
            }
        }
        if !added.is_empty() {
            let mut next: Vec<_> = old
                .into_iter()
                .filter(|p| !added.iter().any(|a| (p[0] - a[0]).abs() < 0.002))
                .collect();
            next.extend(added);
            next.sort_by(|a, b| a[0].total_cmp(&b[0]));
            state.mid.insert(net.clone(), next);
        }
    }
    Ok(())
}

fn build_xy(state: &mut CorridorState, members: &[String]) -> Result<(), String> {
    state.lanes.clear();
    state.mid_xy.clear();
    for net in members {
        let mut pieces = Vec::new();
        for points in state.mid[net].windows(2) {
            pieces.push((
                (points[0][0] + points[1][0]) / 2.0,
                state.spine.lane_xy(points),
            ));
        }
        state.mid_xy.insert(net.clone(), pieces.clone());
        let mut poly = vec![*state
            .teeth
            .get(net)
            .ok_or_else(|| format!("lane xy missing tooth {net}"))?];
        if state.join_block.contains_key(net) {
            let (s, a, b) = state.legs[net][0];
            poly.push(state.spine.xy(s, a));
            poly.push(state.spine.xy(s, b));
        }
        for (_, xy) in pieces {
            for p in xy {
                if corridor::distance(*poly.last().unwrap(), p) > 1e-6 {
                    poly.push(p);
                }
            }
        }
        if state.exit_block.contains_key(net) {
            let (s, a, b) = *state
                .legs
                .get(net)
                .and_then(|legs| legs.last())
                .ok_or_else(|| format!("lane xy missing exit leg {net}"))?;
            poly.push(state.spine.xy(s, a));
            poly.push(state.spine.xy(s, b));
        }
        poly.push(
            *state
                .stubs
                .get(net)
                .ok_or_else(|| format!("lane xy missing stub {net}"))?,
        );
        let mut compact = Vec::new();
        for p in poly {
            if compact
                .last()
                .is_none_or(|old| corridor::distance(*old, p) > 1e-6)
            {
                compact.push(p);
            }
        }
        state.lanes.insert(net.clone(), compact);
    }
    Ok(())
}

fn chord_islands(
    state: &CorridorState,
    schedule: &BraidSchedule,
    members: &[String],
) -> BTreeMap<String, Vec<String>> {
    let mut result = BTreeMap::new();
    for net in members {
        let Some(page) = schedule.page_of(net) else {
            result.insert(net.clone(), vec![]);
            continue;
        };
        let a = (state.s0, state.launch_o[net]);
        let b = (state.s1, state.target_o[net]);
        let own: BTreeSet<_> = [
            state.context.src_ref.get(net).cloned(),
            state.context.ends.get(net).map(|e| e.destination.clone()),
        ]
        .into_iter()
        .flatten()
        .collect();
        let mut hits = Vec::new();
        for island in state.islands_by_layer.get(page).into_iter().flatten() {
            let box_ = island.bounds;
            let mut t0: f64 = 0.0;
            let mut t1: f64 = 1.0;
            let dx = b.0 - a.0;
            let dy = b.1 - a.1;
            let mut ok = true;
            for (p, q) in [
                (-dx, a.0 - box_[0]),
                (dx, box_[1] - a.0),
                (-dy, a.1 - box_[2]),
                (dy, box_[3] - a.1),
            ] {
                if p.abs() < 1e-12 {
                    if q < 0.0 {
                        ok = false;
                        break;
                    }
                    continue;
                }
                let t = q / p;
                if p < 0.0 {
                    t0 = t0.max(t);
                } else {
                    t1 = t1.min(t);
                }
                if t0 > t1 {
                    ok = false;
                    break;
                }
            }
            let component = island
                .what
                .split('.')
                .next()
                .unwrap_or(&island.what)
                .to_owned();
            if ok && !own.contains(&component) {
                hits.push(island.what.clone());
            }
        }
        result.insert(net.clone(), hits);
    }
    result
}

fn reserve_swimmer_hops(
    state: &mut CorridorState,
    schedule: &BraidSchedule,
    members: &[String],
) -> Result<(), String> {
    state.hops.clear();
    state.swim_changes.clear();
    if members.is_empty() {
        return Ok(());
    }
    let observations = BTreeMap::from([
        (
            "F.Cu".to_owned(),
            braid_geometry::obs_but(&state.context, &members[0], members, "F.Cu")?,
        ),
        (
            "B.Cu".to_owned(),
            braid_geometry::obs_but(&state.context, &members[0], members, "B.Cu")?,
        ),
    ]);
    let pad_radius = (state.options.via_size - state.options.track_width) / 2.0;
    let mut placed = Vec::<(Point, String)>::new();
    for net in members {
        if state.exit_leg_s.contains_key(net)
            && state.legs.get(net).is_some_and(|legs| !legs.is_empty())
            && corner_dive(state, net)?
        {
            let leg = state.legs[net][state.legs[net].len() - 1];
            placed.push((state.spine.xy(state.exit_leg_s[net], leg.1), net.clone()));
        }
        if let Some(offset) = state.leg_split.get(net) {
            if let Some(s) = state.exit_leg_s.get(net) {
                placed.push((state.spine.xy(*s, *offset), net.clone()));
            }
        }
    }
    let pair_half = super::pairs::pitch(&state.context.cfg) / 2.0;
    for net in members {
        if schedule.page_of(net).is_some() {
            continue;
        }
        let pair_half_self = if state.context.pairs.contains_key(net) {
            pair_half
        } else {
            0.0
        };
        let mut want = Vec::<(f64, String)>::new();
        for other in members {
            if schedule.page_of(other).is_none() {
                continue;
            }
            if other == net || !schedule.inverted(net, other)? {
                continue;
            }
            let lo = state.mid[net][0][0].max(state.mid[other][0][0]);
            let hi = state.mid[net][state.mid[net].len() - 1][0]
                .min(state.mid[other][state.mid[other].len() - 1][0]);
            if hi - lo < 0.1 {
                continue;
            }
            let count = ((hi - lo) / 0.05).ceil() as usize;
            let samples: Vec<_> = (0..count).map(|i| lo + i as f64 * 0.05).collect();
            let diffs: Vec<_> = samples
                .iter()
                .map(|s| {
                    interpolate_line(&state.mid[net], *s)
                        .zip(interpolate_line(&state.mid[other], *s))
                        .map(|(a, b)| a - b)
                })
                .collect();
            for i in 0..diffs.len().saturating_sub(1) {
                if let (Some(a), Some(b)) = (diffs[i], diffs[i + 1]) {
                    if a.signum() != b.signum() {
                        if let Some(layer) = need_at(schedule, other, samples[i]) {
                            want.push((samples[i], layer.to_owned()));
                        }
                    }
                }
            }
        }
        if want.is_empty() {
            continue;
        }
        want.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut sequence = vec![(state.mid[net][0][0], state.context.tooth_layer[net].clone())];
        sequence.extend(want.iter().cloned());
        sequence.push((
            state.mid[net].last().unwrap()[0],
            state.context.dest_layer[net].clone(),
        ));
        let mut spots = Vec::new();
        for pair in sequence.windows(2) {
            let (s_a, l_a) = (&pair[0].0, &pair[0].1);
            let (s_b, l_b) = (&pair[1].0, &pair[1].1);
            if l_a == l_b || s_b - s_a < 0.1 {
                continue;
            }
            let mid = (*s_a + *s_b) / 2.0;
            let mut candidates = Vec::new();
            let mut s = *s_a + 0.05;
            while s < *s_b - 0.049 {
                candidates.push(s);
                s += 0.05;
            }
            candidates.sort_by(|a, b| (a - mid).abs().total_cmp(&(b - mid).abs()));
            let offsets = [
                0.0, 0.15, -0.15, 0.3, -0.3, 0.45, -0.45, 0.6, -0.6, 0.9, -0.9, 1.2, -1.2,
            ];
            let mut found = None;
            'site: for s in candidates {
                let Some(o0) = interpolate_line(&state.mid[net], s) else {
                    continue;
                };
                for do_ in offsets {
                    let point = state.spine.xy(s, o0 + do_);
                    if observations
                        .values()
                        .any(|ob| ob.point_violation_with_pad(point, pad_radius))
                    {
                        continue;
                    }
                    let line_clash = members.iter().filter(|other| *other != net).any(|other| {
                        line_distance(state, other, s, o0 + do_)
                            < 0.30
                                + braid_offsets::lane_w(state, other)
                                + braid_offsets::lane_w(state, net)
                    });
                    if line_clash {
                        continue;
                    }
                    let too_close = placed.iter().any(|(old, owner)| {
                        owner != net
                            && corridor::distance(point, *old)
                                < state.options.via_size
                                    + state.options.clearance
                                    + 2.0 * pair_half_self
                                    + if state.context.pairs.contains_key(owner) {
                                        pair_half
                                    } else {
                                        0.0
                                    }
                    });
                    if too_close {
                        continue;
                    }
                    if !via_static_ok(state, net, point, pair_half_self)? {
                        continue;
                    }
                    found = Some(point);
                    break 'site;
                }
            }
            if let Some(point) = found {
                spots.push(point);
                placed.push((point, net.clone()));
            }
        }
        if !spots.is_empty() {
            state.hops.insert(net.clone(), spots);
        }
        let changes = sequence.windows(2).filter(|p| p[0].1 != p[1].1).count();
        state.swim_changes.insert(net.clone(), changes);
    }
    Ok(())
}

fn line_distance(state: &CorridorState, net: &str, s: f64, o: f64) -> f64 {
    let Some(line) = state.mid.get(net) else {
        return f64::INFINITY;
    };
    let mut best = f64::INFINITY;
    for pair in line.windows(2) {
        if pair[1][0] < s - 0.6 || pair[0][0] > s + 0.6 {
            continue;
        }
        let dx = pair[1][0] - pair[0][0];
        let dy = pair[1][1] - pair[0][1];
        let length = dx * dx + dy * dy;
        let t = if length < 1e-12 {
            0.0
        } else {
            (((s - pair[0][0]) * dx + (o - pair[0][1]) * dy) / length).clamp(0.0, 1.0)
        };
        best = best.min((s - pair[0][0] - t * dx).hypot(o - pair[0][1] - t * dy));
    }
    best
}

pub fn lay_lanes(state: &mut CorridorState, schedule: &BraidSchedule) -> Result<(), String> {
    state.options.validate()?;
    if state.members.is_empty() {
        return Ok(());
    }
    state.sched_cur = Some(schedule.clone());
    state.width = state.l_free - state.reserve();
    if !state.width.is_finite() {
        return Err("braid layout width is nonfinite".into());
    }
    state.layout_need = 0.0;
    let members = state.members.clone();
    let target_rank: BTreeMap<_, _> = state
        .target
        .iter()
        .enumerate()
        .map(|(i, net)| (net.clone(), i))
        .collect();
    for net in &members {
        if !target_rank.contains_key(net) {
            return Err(format!("braid target order omits {net}"));
        }
    }
    let islands = static_islands(state)?;
    state.islands_by_layer = islands.clone();

    // A tail lane crossing a static island on a layer it owes to reach its
    // page/berth takes that via before the island; no extra via is introduced.
    let mut pre = BTreeMap::<String, Vec<(f64, f64, String)>>::new();
    for layer in ["F.Cu", "B.Cu"] {
        let other = other_layer(layer);
        for island in islands.get(layer).into_iter().flatten() {
            let [s_lo, s_hi, o_lo, o_hi] = island.bounds;
            if s_lo < state.s1 - 0.1 {
                continue;
            }
            for net in &members {
                let Some(page) = schedule.page_of(net) else {
                    continue;
                };
                let berth = state.se[net];
                if berth[0] <= s_lo + 0.05 {
                    continue;
                }
                let o_t = state.py[target_rank[net]];
                let o_here = if state.exit_block.contains_key(net) {
                    o_t
                } else {
                    let t0 = ((s_lo - state.s1) / (berth[0] - state.s1).max(1e-9)).clamp(0.0, 1.0);
                    let t1 = ((s_hi - state.s1) / (berth[0] - state.s1).max(1e-9)).clamp(0.0, 1.0);
                    let a = o_t + t0 * (berth[1] - o_t);
                    let b = o_t + t1 * (berth[1] - o_t);
                    if !(a.min(b) < o_hi && a.max(b) > o_lo) {
                        continue;
                    }
                    (a + b) / 2.0
                };
                if !(o_lo < o_here && o_here < o_hi) {
                    continue;
                }
                let (start, end) = if page == other {
                    (state.s1 + 0.05, (berth[0] - 0.05).min(s_hi + 0.3))
                } else if state.context.dest_layer[net] == other {
                    ((state.s1 + 0.05).max(s_lo - 0.3), berth[0] - 0.05)
                } else {
                    continue;
                };
                if end > start
                    && !pre
                        .get(net)
                        .is_some_and(|ivs| ivs.iter().any(|iv| (iv.0 - start).abs() < 1e-6))
                {
                    pre.entry(net.clone())
                        .or_default()
                        .push((start, end, other.to_owned()));
                }
            }
        }
    }
    let no_extras = BTreeMap::new();
    let (mut intervals, mut leg_req_min, _) =
        place_and_decide(state, schedule, &islands, &pre, None, &no_extras)?;
    let original_layers = state.leg_layer.clone();
    let bad_legs: Vec<_> = members
        .iter()
        .filter(|net| state.exit_block.contains_key(*net))
        .filter(|net| {
            original_layers.get(*net).is_some_and(|layer| {
                leg_on_island(state, net, state.exit_leg_s[*net], layer, &islands)
                    && leg_split_at(state, net, state.exit_leg_s[*net], layer, &islands).is_none()
            })
        })
        .cloned()
        .collect();
    let mut extra_cands = BTreeMap::new();
    for net in &bad_legs {
        extra_cands.insert(
            net.clone(),
            island_edges(
                state,
                net,
                state.exit_leg_s[net],
                &original_layers[net],
                &islands,
            ),
        );
    }
    if !bad_legs.is_empty() {
        let avoid_state = state.clone();
        let avoid = |net: &str, s: f64| {
            bad_legs.iter().any(|candidate| {
                candidate == net
                    && original_layers
                        .get(candidate)
                        .is_some_and(|layer| leg_on_island(&avoid_state, net, s, layer, &islands))
            })
        };
        (intervals, leg_req_min, _) =
            place_and_decide(state, schedule, &islands, &pre, Some(&avoid), &extra_cands)?;
    }

    let arriving = |state: &CorridorState,
                    ivs: &BTreeMap<String, Vec<(f64, f64, String)>>,
                    net: &str|
     -> Option<String> {
        let s = *state.exit_leg_s.get(net)?;
        ivs.get(net)
            .and_then(|list| {
                list.iter()
                    .filter(|iv| iv.0 < s)
                    .max_by(|a, b| a.0.total_cmp(&b.0))
                    .map(|iv| iv.2.clone())
            })
            .or_else(|| schedule.page.get(net).cloned().flatten())
    };
    let leg_order = |state: &CorridorState, net: &str| {
        if state.far_exit.contains(net) {
            (1_u8, state.exit_block[net].abs(), state.se[net][0])
        } else {
            (0_u8, state.se[net][0], state.exit_block[net].abs())
        }
    };
    let mut stuck = BTreeSet::<(String, String)>::new();
    for _ in 0..3 {
        let mut found = Vec::<(String, String, f64)>::new();
        for net in &members {
            if !state.exit_block.contains_key(net) {
                continue;
            }
            let Some(layer) = arriving(state, &intervals, net) else {
                continue;
            };
            if state.leg_layer.get(net) == Some(&layer) {
                continue;
            }
            let s = state.exit_leg_s[net];
            let o = state.py[target_rank[net]];
            for other in &members {
                if other == net || !state.exit_block.contains_key(other) {
                    continue;
                }
                let r = state.via_need()
                    + braid_offsets::lane_w(state, net)
                    + braid_offsets::lane_w(state, other);
                let a = state.py[target_rank[other]].min(state.se[other][1]);
                let b = state.py[target_rank[other]].max(state.se[other][1]);
                if (state.exit_leg_s[other] - s).abs() < r - 1e-6 && a - r < o && o < b + r {
                    found.push((net.clone(), other.clone(), r));
                }
            }
        }
        if found.is_empty() {
            break;
        }
        let mut blocks = BTreeMap::<String, Vec<(f64, f64, String)>>::new();
        for (a, b, r) in found {
            let (mut mover, mut keep) = if leg_order(state, &b) > leg_order(state, &a) {
                (b.clone(), a.clone())
            } else {
                (a.clone(), b.clone())
            };
            if stuck.contains(&(mover.clone(), keep.clone())) {
                std::mem::swap(&mut mover, &mut keep);
            }
            blocks
                .entry(mover.clone())
                .or_default()
                .push((state.exit_leg_s[&keep], r, keep));
        }
        let before: BTreeMap<_, _> = blocks
            .keys()
            .map(|net| (net.clone(), state.exit_leg_s[net]))
            .collect();
        let avoid_state = state.clone();
        let avoid = |net: &str, s: f64| {
            bad_legs.iter().any(|candidate| {
                candidate == net
                    && original_layers
                        .get(candidate)
                        .is_some_and(|layer| leg_on_island(&avoid_state, net, s, layer, &islands))
            }) || blocks.get(net).is_some_and(|items| {
                items
                    .iter()
                    .any(|(blocked, r, _)| (s - blocked).abs() < r - 1e-6)
            })
        };
        (intervals, leg_req_min, _) =
            place_and_decide(state, schedule, &islands, &pre, Some(&avoid), &extra_cands)?;
        for (mover, items) in blocks {
            if (state.exit_leg_s[&mover] - before[&mover]).abs() < 1e-9 {
                for (_, _, keep) in items {
                    stuck.insert((mover.clone(), keep));
                }
            }
        }
    }
    state.leg_split.clear();
    for net in &members {
        if !state.exit_block.contains_key(net) {
            continue;
        }
        let layer = state
            .leg_layer
            .get(net)
            .ok_or_else(|| format!("exit leg layer missing for {net}"))?;
        if leg_on_island(state, net, state.exit_leg_s[net], layer, &islands) {
            if let Some(o) = leg_split_at(state, net, state.exit_leg_s[net], layer, &islands) {
                state.leg_split.insert(net.clone(), o);
            }
        }
    }

    leg_req_min.clear();
    let mut req: BTreeMap<String, Vec<LayerInterval>> =
        members.iter().map(|net| (net.clone(), vec![])).collect();
    for net in &members {
        let list = intervals.get(net).cloned().unwrap_or_default();
        let kept: Vec<_> = list
            .iter()
            .filter(|iv| {
                !list
                    .iter()
                    .any(|other| other.2 != iv.2 && iv.0 < other.1 && other.0 < iv.1)
            })
            .cloned()
            .collect();
        if kept.is_empty() {
            continue;
        }
        let page = schedule.page_of(net);
        let dives: Vec<_> = kept
            .iter()
            .filter(|iv| Some(iv.2.as_str()) != page)
            .map(|iv| iv.0)
            .collect();
        if let Some(first) = dives.into_iter().reduce(f64::min) {
            leg_req_min
                .entry(net.clone())
                .and_modify(|x| *x = x.min(first))
                .or_insert(first);
        }
        req.get_mut(net).unwrap().extend(
            kept.into_iter()
                .map(|(start, end, layer)| LayerInterval { start, end, layer }),
        );
    }
    let mut lines = BTreeMap::<String, (f64, f64, f64, f64)>::new();
    for net in &members {
        let start = state
            .join_leg_s
            .get(net)
            .copied()
            .unwrap_or(state.s0.max(state.st[net][0]));
        lines.insert(
            net.clone(),
            (
                start,
                state.launch_o[net],
                state.s1,
                state.py[target_rank[net]],
            ),
        );
    }
    let cross_s = |a: &str, b: &str| -> Option<f64> {
        let x = lines[a];
        let y = lines[b];
        let lo = x.0.max(y.0);
        let hi = x.2.min(y.2);
        if hi <= lo {
            return None;
        }
        let at =
            |l: (f64, f64, f64, f64), s: f64| l.1 + (l.3 - l.1) * (s - l.0) / (l.2 - l.0).max(1e-9);
        let d0 = at(x, lo) - at(y, lo);
        let d1 = at(x, hi) - at(y, hi);
        if (d0 - d1).abs() < 1e-12 {
            return None;
        }
        let s = lo + (hi - lo) * d0 / (d0 - d1);
        (lo <= s && s <= hi).then_some(s)
    };
    for net in &members {
        let Some(page) = schedule.page_of(net) else {
            continue;
        };
        let mut crossings: Vec<_> = members
            .iter()
            .filter(|other| *other != net && schedule.inverted(net, other).unwrap_or(false))
            .filter_map(|other| cross_s(net, other))
            .collect();
        if crossings.is_empty() {
            continue;
        }
        crossings.sort_by(f64::total_cmp);
        let mut start = state.s0 + 0.05;
        if state.context.tooth_layer[net] != page {
            start = start.min((crossings[0] - HW_COL).max(state.s0 + 0.02));
        }
        let mut end = state.s1 - 0.05;
        if state.context.dest_layer[net] != page {
            end = end.max((crossings[crossings.len() - 1] + HW_COL).min(state.s1 - 0.02));
        }
        if state.exit_block.contains_key(net) {
            end = end.min(state.exit_leg_s[net] - 0.35);
        }
        if let Some(v) = leg_req_min.get(net) {
            end = end.min(v - 0.03);
        }
        if end > start {
            req.get_mut(net).unwrap().push(LayerInterval {
                start,
                end,
                layer: page.to_owned(),
            });
        }
    }
    state.req = req;
    let mut back = BTreeMap::<String, Vec<(f64, f64)>>::new();
    for net in &members {
        let mut windows = vec![(state.s1 - 0.1, 1e9)];
        if schedule.page_of(net).is_none() {
            windows.push((-1e9, 1e9));
        }
        let mut bs: Vec<_> = state.req[net]
            .iter()
            .filter(|iv| iv.layer == "B.Cu" && iv.start < state.s1 - 0.1)
            .map(|iv| (iv.start, iv.end))
            .collect();
        let mut fs: Vec<_> = state.req[net]
            .iter()
            .filter(|iv| iv.layer == "F.Cu")
            .map(|iv| (iv.start, iv.end))
            .collect();
        bs.sort_by(|a, b| a.0.total_cmp(&b.0));
        fs.sort_by(|a, b| a.0.total_cmp(&b.0));
        if let (Some(first), Some(last)) = (bs.first(), bs.last()) {
            windows.push((
                fs.iter()
                    .filter(|iv| iv.1 <= first.0)
                    .map(|iv| iv.1)
                    .reduce(f64::max)
                    .unwrap_or(-1e9),
                fs.iter()
                    .filter(|iv| iv.0 >= last.1)
                    .map(|iv| iv.0)
                    .reduce(f64::min)
                    .unwrap_or(1e9),
            ));
        }
        if state.context.tooth_layer[net] == "B.Cu" {
            windows.push((
                -1e9,
                fs.iter().map(|iv| iv.0).reduce(f64::min).unwrap_or(1e9),
            ));
        }
        back.insert(net.clone(), windows);
    }
    state.back_windows = back;

    build_midlines(state, schedule, &members, &target_rank)?;
    deflect_islands(state, schedule)?;
    if state.is_branch {
        order_ring(state, schedule)?;
    }
    add_corner_points(state, &members)?;
    build_xy(state, &members)?;
    reserve_swimmer_hops(state, schedule, &members)?;
    state.layer_profiles.clear();
    for net in &members {
        state
            .layer_profiles
            .insert(net.clone(), layer_profile(state, net)?);
    }
    state.islands = chord_islands(state, schedule, &members);
    Ok(())
}
