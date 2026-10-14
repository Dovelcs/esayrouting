//! Exact mutable offset planning port of the pinned AWX `Corridor.offsets`.
use super::{braid_corridor::CorridorState, braid_schedule::BraidSchedule, corridor::Point};
use std::collections::{BTreeMap, BTreeSet};

// Fixed source braid.py:345, the BRANCH trunk handoff offset.
const HAND_DS: f64 = 0.6;

fn relax_pitch(values: &[f64], floors: &[f64]) -> Vec<f64> {
    let mut result = values.to_vec();
    if result.len() < 2 {
        return result;
    }
    for _ in 0..60 {
        let mut moved = false;
        for i in 0..result.len() - 1 {
            let floor = floors.get(i).copied().unwrap_or(0.0);
            let gap = result[i + 1] - result[i];
            if gap < floor - 1e-9 {
                let push = (floor - gap) / 2.0;
                result[i] -= push;
                result[i + 1] += push;
                moved = true;
            }
        }
        if !moved {
            break;
        }
    }
    result
}

/// Copper envelope contributed by one member of a differential pair.
pub fn lane_w(state: &CorridorState, net: &str) -> f64 {
    if !state.options.pairs || !state.context.pairs.contains_key(net) {
        return 0.0;
    }
    let pair_half = (state.options.track_width + state.context.cfg.pair_gap) / 2.0;
    let via_half = ((state.options.via_size + state.options.clearance) / 2.0).max(
        (state.options.via_size / 2.0 + state.options.clearance + state.options.track_width / 2.0
            - pair_half)
            / 0.7071
            + 0.005,
    );
    let envelope_radius = via_half + state.options.via_size / 2.0;
    pair_half.max(
        envelope_radius + state.options.clearance + state.options.track_width / 2.0
            - state.options.lane_pitch,
    )
}

/// Minimum offset between two adjacent slots, including their pair widths,
/// schedule pages, endpoint layers, and the current run slopes.
pub fn pair_floor(
    state: &CorridorState,
    a: &str,
    b: &str,
    base: f64,
    schedule: Option<&BraidSchedule>,
    at_launch: bool,
    side: f64,
) -> Result<f64, String> {
    let base = base + lane_w(state, a) + lane_w(state, b);
    let Some(schedule) = schedule.filter(|_| state.options.slope_pitch) else {
        return Ok(base);
    };
    let page_a = schedule
        .page
        .get(a)
        .ok_or_else(|| format!("schedule page missing for {a}"))?;
    let page_b = schedule
        .page
        .get(b)
        .ok_or_else(|| format!("schedule page missing for {b}"))?;
    let end_layers = if at_launch {
        &state.context.tooth_layer
    } else {
        &state.context.dest_layer
    };
    let da = if at_launch { 1.0 } else { -1.0 };
    let toward = |net: &str, direction_side: f64| {
        let slope = state.slope.get(net).copied().unwrap_or(0.0) * da;
        if slope * direction_side > 0.0 {
            (1.0 + slope * slope).sqrt()
        } else {
            1.0
        }
    };
    let sa = toward(a, side);
    let sb = toward(b, -side);
    match (page_a.as_deref(), page_b.as_deref()) {
        (None, None) => Ok(base),
        (None, Some(page)) => {
            if end_layers.get(a).is_some_and(|layer| layer == page) {
                Ok(base * sb)
            } else {
                Ok(base)
            }
        }
        (Some(page), None) => {
            if end_layers.get(b).is_some_and(|layer| layer == page) {
                Ok(base * sa)
            } else {
                Ok(base)
            }
        }
        (Some(pa), Some(pb)) if pa == pb => Ok(base * sa.max(sb)),
        (Some(pa), Some(pb)) => {
            let mut need = base;
            if end_layers.get(a).is_some_and(|layer| layer != pa) {
                need = need.max(state.via_need() * sb);
            }
            if end_layers.get(b).is_some_and(|layer| layer != pb) {
                need = need.max(state.via_need() * sa);
            }
            Ok(need)
        }
    }
}

/// Order head-on exits by the source's nested berth-row relation.
pub fn head_order(
    state: &mut CorridorState,
    names: &[String],
    ends: &BTreeMap<String, Point>,
) -> Result<Vec<String>, String> {
    let mut side = BTreeMap::new();
    for net in names {
        let berth = *ends
            .get(net)
            .ok_or_else(|| format!("braid offsets: exit endpoint missing for {net}"))?;
        let destination = state
            .context
            .ends
            .get(net)
            .ok_or_else(|| format!("braid offsets: destination reference missing for {net}"))?
            .destination
            .as_str();
        let destination_pads: Vec<_> = state
            .context
            .pads
            .iter()
            .filter(|pad| pad.component_ref == destination)
            .collect();
        if destination_pads.is_empty() {
            return Err(format!(
                "braid offsets: destination array {destination} has no native pads"
            ));
        }
        let center = [
            destination_pads.iter().map(|pad| pad.point[0]).sum::<f64>()
                / destination_pads.len() as f64,
            destination_pads.iter().map(|pad| pad.point[1]).sum::<f64>()
                / destination_pads.len() as f64,
        ];
        let d = state.spine.project_pt(center)[1] - berth[1];
        side.insert(net.clone(), if d.abs() < 1e-6 { 0.0 } else { d.signum() });
    }
    let mut before: BTreeMap<String, BTreeSet<String>> = names
        .iter()
        .map(|name| (name.clone(), BTreeSet::new()))
        .collect();
    for a in names {
        for b in names {
            if a == b
                || state.context.dest_layer.get(a) != state.context.dest_layer.get(b)
                || side[a] == 0.0
            {
                continue;
            }
            if ends[b][0] > ends[a][0] + 0.05
                && (ends[b][1] - ends[a][1]).abs() < state.row_offset()
            {
                if side[a] > 0.0 {
                    before.get_mut(a).unwrap().insert(b.clone());
                } else {
                    before.get_mut(b).unwrap().insert(a.clone());
                }
            }
        }
    }
    let mut up: BTreeMap<String, BTreeSet<String>> = names
        .iter()
        .map(|name| (name.clone(), BTreeSet::new()))
        .collect();
    for a in names {
        for b in names {
            if a != b && (before[a].contains(b) || before[b].contains(a)) && ends[a][0] < ends[b][0]
            {
                up.get_mut(b).unwrap().insert(a.clone());
            }
        }
    }
    state.row_slot.clear();
    state.row_side.clear();
    state.row_up = up
        .iter()
        .map(|(name, nearer)| (name.clone(), nearer.iter().cloned().collect()))
        .collect();
    let mut row_names: Vec<_> = names
        .iter()
        .filter(|name| !up[*name].is_empty() || up.values().any(|members| members.contains(*name)))
        .cloned()
        .collect();
    row_names.sort_by(|a, b| ends[a][0].total_cmp(&ends[b][0]));
    for name in row_names {
        let sd = if side[&name] != 0.0 {
            side[&name]
        } else {
            up[&name]
                .iter()
                .find_map(|near| (side[near] != 0.0).then_some(side[near]))
                .unwrap_or(1.0)
        };
        state.row_side.insert(name.clone(), sd as i32);
        let mut offset = -sd * ends[&name][1];
        for nearer in &up[&name] {
            offset = offset.max(-sd * ends[nearer][1] + state.options.lane_pitch);
            if let Some(slot) = state.row_slot.get(nearer) {
                offset = offset.max(-sd * slot + state.options.lane_pitch);
            }
        }
        state.row_slot.insert(name, -sd * offset);
    }
    let mut want = BTreeMap::new();
    for name in names {
        want.insert(
            name.clone(),
            state.row_slot.get(name).copied().unwrap_or(ends[name][1]),
        );
    }
    let mut left = names.to_vec();
    left.sort_by(|a, b| {
        want[a]
            .total_cmp(&want[b])
            .then(ends[a][1].total_cmp(&ends[b][1]))
    });
    let mut order = Vec::with_capacity(names.len());
    while !left.is_empty() {
        let remaining: BTreeSet<_> = left.iter().cloned().collect();
        let next = left
            .iter()
            .find(|name| !before[*name].intersection(&remaining).next().is_some())
            .cloned();
        let Some(next) = next else {
            order.extend(left);
            break;
        };
        order.push(next.clone());
        left.retain(|name| name != &next);
    }
    Ok(order)
}

/// Slope-aware floor for two reserved lanes adjacent to a slot.
pub fn secant_floor(
    state: &CorridorState,
    a: &str,
    b: &str,
    base: f64,
    schedule: Option<&BraidSchedule>,
) -> f64 {
    let slope = schedule
        .into_iter()
        .flat_map(|schedule| {
            [a, b].into_iter().filter_map(|net| {
                schedule.page.get(net).and_then(Option::as_ref)?;
                Some(state.slope.get(net).copied().unwrap_or(0.0).abs())
            })
        })
        .fold(0.0_f64, f64::max);
    (base + lane_w(state, a) + lane_w(state, b)) * (1.0 + slope * slope).sqrt()
}

/// `Corridor._leg_s` source placement and clash tie-break for a joining/exit leg.
pub type LegAvoid<'a> = dyn Fn(&str, f64) -> bool + 'a;

pub fn leg_s(
    state: &CorridorState,
    net: &str,
    slot_s: f64,
    offset_a: f64,
    offset_b: f64,
    at_tooth: bool,
    placed: &[(f64, f64, f64)],
    avoid: Option<&LegAvoid<'_>>,
    extra: &[f64],
) -> Result<f64, String> {
    leg_s_with_options(
        state, net, slot_s, offset_a, offset_b, at_tooth, placed, avoid, extra, false, None, None,
    )
}

fn leg_s_with_options(
    state: &CorridorState,
    net: &str,
    slot_s: f64,
    offset_a: f64,
    offset_b: f64,
    at_tooth: bool,
    placed: &[(f64, f64, f64)],
    avoid: Option<&LegAvoid<'_>>,
    extra: &[f64],
    layer_only: bool,
    own: Option<Point>,
    own_layer: Option<&str>,
) -> Result<f64, String> {
    let own = own.unwrap_or(if at_tooth {
        *state
            .st
            .get(net)
            .ok_or_else(|| format!("missing tooth for {net}"))?
    } else {
        *state
            .se
            .get(net)
            .ok_or_else(|| format!("missing berth for {net}"))?
    });
    let own_layer = own_layer.unwrap_or(
        if at_tooth {
            state.context.tooth_layer.get(net)
        } else {
            state.context.dest_layer.get(net)
        }
        .ok_or_else(|| format!("missing free-end layer for {net}"))?,
    );
    let mut ends = Vec::<(f64, f64, f64, f64)>::new();
    let mut ends_layer = Vec::new();
    for other in &state.members {
        if other == net {
            continue;
        }
        for (index, (point, layer)) in [
            (state.st.get(other), state.context.tooth_layer.get(other)),
            (state.se.get(other), state.context.dest_layer.get(other)),
        ]
        .into_iter()
        .enumerate()
        {
            let (Some(point), Some(layer)) = (point, layer) else {
                return Err(format!("free-end state missing for {other}"));
            };
            if layer != own_layer
                && (point[0] - own[0]).abs() < 0.05
                && (point[1] - own[1]).abs() < 0.05
            {
                continue;
            }
            let (mut lo, mut hi) = (point[0], point[0]);
            if state.context.pairs.contains_key(other) {
                if let Some(pair_ends) = state.context.pair_ends.get(other) {
                    let span = &pair_ends[index];
                    let s: Vec<_> = span.iter().map(|p| state.spine.project_pt(*p)[0]).collect();
                    lo = s[0].min(s[1]);
                    hi = s[0].max(s[1]);
                }
            }
            let end = (point[0], point[1], lo, hi);
            ends.push(end);
            if layer == own_layer {
                ends_layer.push(end);
            }
        }
    }
    let (lo_o, hi_o) = (offset_a.min(offset_b), offset_a.max(offset_b));
    let end_clash = state.options.lane_pitch - 1e-6;
    let half =
        if state.options.pairs && state.options.branch && state.context.pairs.contains_key(net) {
            (state.options.track_width + state.context.cfg.pair_gap) / 2.0
        } else {
            0.0
        };
    let own_hw = if half > 0.0 {
        vec![-half, half]
    } else {
        vec![0.0]
    };
    let own_ext = own_hw.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
    let clash = |candidate: f64| {
        let foreign = if layer_only { &ends_layer } else { &ends };
        let mut count = foreign
            .iter()
            .filter(|p| {
                p.2 - end_clash - own_ext < candidate
                    && candidate < p.3 + end_clash + own_ext
                    && lo_o - 0.05 < p.1
                    && p.1 < hi_o + 0.05
            })
            .count();
        count += placed
            .iter()
            .filter(|(s, lo, hi)| {
                own_hw
                    .iter()
                    .map(|h| (s - candidate - h).abs())
                    .fold(f64::INFINITY, f64::min)
                    < state.options.track_width + 0.1 + 0.02
                    && *lo < hi_o
                    && *hi > lo_o
            })
            .count();
        count
    };
    let jogged = |candidate: f64| {
        let lo_s = candidate.min(own[0]) - 0.1;
        let hi_s = candidate.max(own[0]) + 0.1;
        ends_layer
            .iter()
            .any(|p| p.2 < hi_s && p.3 > lo_s && (p.1 - own[1]).abs() < state.leg_offset())
    };
    let room = |candidate: f64| {
        ends.iter()
            .filter(|p| lo_o - 0.05 < p.1 && p.1 < hi_o + 0.05)
            .map(|p| ((p.2 - candidate).max(candidate - p.3).max(0.0) - own_ext).max(0.0))
            .chain(
                placed
                    .iter()
                    .filter(|(_, lo, hi)| *lo < hi_o && *hi > lo_o)
                    .map(|(s, _, _)| {
                        own_hw
                            .iter()
                            .map(|h| (s - candidate - h).abs())
                            .fold(f64::INFINITY, f64::min)
                    }),
            )
            .fold(f64::INFINITY, f64::min)
    };
    let bad = |candidate: f64| avoid.is_some_and(|predicate| predicate(net, candidate));
    if clash(slot_s) == 0 && !bad(slot_s) {
        return Ok(slot_s);
    }
    let mut candidates = vec![
        slot_s + state.options.lane_pitch,
        slot_s - state.options.lane_pitch,
        slot_s + 2.0 * state.options.lane_pitch,
        slot_s - 2.0 * state.options.lane_pitch,
    ];
    let mut extras = extra.to_vec();
    extras.sort_by(|a, b| (a - slot_s).abs().total_cmp(&(b - slot_s).abs()));
    candidates.extend(extras);
    candidates.retain(|candidate| !jogged(*candidate) && room(*candidate) >= state.lane_min());
    let mut best: Option<(usize, f64)> = None;
    for &candidate in &candidates {
        if bad(candidate) {
            continue;
        }
        let n = clash(candidate);
        if n == 0 {
            return Ok(candidate);
        }
        if best.map_or(true, |(count, old)| {
            (n, -room(candidate)) < (count, -room(old))
        }) {
            best = Some((n, candidate));
        }
    }
    if best.is_none() {
        for &candidate in &candidates {
            let n = clash(candidate);
            if best.map_or(true, |(count, old)| {
                (n, -room(candidate)) < (count, -room(old))
            }) {
                best = Some((n, candidate));
            }
        }
    }
    Ok(best.map(|(_, candidate)| candidate).unwrap_or(slot_s))
}

pub fn clear_block(
    state: &CorridorState,
    mut base: f64,
    side: f64,
    s_from: f64,
    s_to: f64,
    net: &str,
) -> Result<f64, String> {
    let base0 = base;
    if state.options.block_push == 0 {
        return Ok(base);
    }
    let mut cleared = false;
    for _ in 0..15 {
        let a = state.spine.xy(s_from, base);
        let b = state.spine.xy(s_to, base);
        let front = super::braid_geometry::obs_but(&state.context, net, &state.members, "F.Cu")?;
        let back = super::braid_geometry::obs_but(&state.context, net, &state.members, "B.Cu")?;
        if front.seg_clear(a, b) && back.seg_clear(a, b) {
            cleared = true;
            break;
        }
        base += side * 0.1;
    }
    if state.options.block_push >= 2 && !cleared {
        return Ok(base0);
    }
    Ok(base)
}

pub fn fit_block(
    state: &CorridorState,
    base: f64,
    side: f64,
    order: &[String],
    gaps: &[f64],
    run_end: f64,
) -> Result<Vec<f64>, String> {
    if gaps.is_empty() {
        return Ok(gaps.to_vec());
    }
    let edge = super::braid_geometry::edge_inset(&state.context);
    let samples = [state.s1, state.s1 + (run_end - state.s1) * 0.5, run_end];
    let fits = |offset: f64| {
        samples.iter().all(|s| {
            super::braid_geometry::on_board(&state.context, state.spine.xy(*s, offset), edge)
        })
    };
    let far = base + side * gaps.iter().sum::<f64>();
    if fits(far) {
        return Ok(gaps.to_vec());
    }
    let (mut limit, mut offset) = (None, base);
    while side * (offset - far) <= 1e-9 && fits(offset) {
        limit = Some(offset);
        offset += side * 0.02;
    }
    let Some(limit) = limit else {
        return Ok(gaps.to_vec());
    };
    let needed = side * (far - limit);
    let mut minimums = Vec::with_capacity(gaps.len());
    for index in 1..order.len() {
        minimums.push(pair_floor(
            state,
            &order[index - 1],
            &order[index],
            state.lane_min(),
            None,
            false,
            1.0,
        )?);
    }
    let room: f64 = gaps
        .iter()
        .zip(&minimums)
        .map(|(gap, min)| (gap - min).max(0.0))
        .sum();
    let lambda = if room > 1e-9 {
        (1.0 - needed / room).max(0.0)
    } else {
        0.0
    };
    Ok(gaps
        .iter()
        .zip(minimums)
        .map(|(gap, min)| min + (gap - min).max(0.0) * lambda)
        .collect())
}

/// Compute launch/target offsets, join/exit blocks, and the exact source orders.
pub fn offsets(
    state: &mut CorridorState,
    launch_pitch: f64,
    schedule: Option<&BraidSchedule>,
) -> Result<(), String> {
    let st = state.st.clone();
    let se = state.se.clone();
    if st.len() != state.members.len() || se.len() != state.members.len() {
        return Err(
            "braid offsets require source and destination spine coordinates for every member"
                .into(),
        );
    }
    if !launch_pitch.is_finite() || launch_pitch <= 0.0 {
        return Err("braid offsets launch pitch must be finite and positive".into());
    }
    if let Some(schedule) = schedule.filter(|_| !state.launch_o.is_empty()) {
        let length = (state.s1 - state.s0).max(1e-6);
        state.slope = state
            .members
            .iter()
            .map(|net| {
                let launch = state.launch_o.get(net).copied().unwrap_or(0.0);
                let target = state.target_o.get(net).copied().unwrap_or(launch);
                (net.clone(), (target - launch) / length)
            })
            .collect();
        let _ = schedule;
    } else {
        state.slope.clear();
    }

    let mut heads_l = state.heads_l.clone();
    heads_l.sort_by(|a, b| st[a][1].total_cmp(&st[b][1]));
    let mut launch_offsets;
    if schedule.is_some() && state.options.slope_pitch {
        let floors = heads_l
            .windows(2)
            .map(|pair| pair_floor(state, &pair[0], &pair[1], launch_pitch, schedule, true, 1.0))
            .collect::<Result<Vec<_>, _>>()?;
        let values: Vec<_> = heads_l.iter().map(|net| st[net][1]).collect();
        let relaxed = relax_pitch(&values, &floors);
        let shift = relaxed
            .iter()
            .zip(&heads_l)
            .map(|(offset, net)| (offset - st[net][1]).abs())
            .fold(0.0_f64, f64::max);
        state.s0 = if heads_l.is_empty() {
            state.s0_base
        } else {
            state.s0_base.max(
                heads_l
                    .iter()
                    .map(|net| st[net][0])
                    .fold(f64::NEG_INFINITY, f64::max)
                    + shift,
            )
        };
        launch_offsets = relaxed;
    } else {
        state.s0 = state.s0_base;
        launch_offsets = Vec::with_capacity(heads_l.len());
        for (i, net) in heads_l.iter().enumerate() {
            let current = st[net][1];
            if let Some(previous) = i.checked_sub(1).and_then(|j| heads_l.get(j)) {
                let floor = pair_floor(state, previous, net, launch_pitch, None, true, 1.0)?;
                launch_offsets.push(current.max(launch_offsets[i - 1] + floor));
            } else {
                launch_offsets.push(current);
            }
        }
    }
    let mut launch_o: BTreeMap<String, f64> = heads_l
        .iter()
        .cloned()
        .zip(launch_offsets.iter().copied())
        .collect();

    state.join_block.clear();
    state.join_leg_s.clear();
    for side in [-1.0, 1.0] {
        let mut joiners: Vec<_> = state
            .joiners
            .iter()
            .filter(|net| state.join_side.get(*net).copied().unwrap_or(0) as f64 == side)
            .cloned()
            .collect();
        if joiners.is_empty() {
            continue;
        }
        let extent = launch_offsets
            .iter()
            .map(|o| side * o)
            .chain(joiners.iter().map(|net| side * st[net][1]))
            .fold(f64::NEG_INFINITY, f64::max);
        let mut base = side
            * (extent + state.block_gap())
                .max(-state.options.lane_pitch * (joiners.len() as f64 - 1.0) / 2.0);
        let far = base + side * state.options.lane_pitch * (joiners.len() as f64 - 1.0);
        let mut placed = Vec::<(f64, f64, f64)>::new();
        let mut by_tooth = joiners.clone();
        by_tooth.sort_by(|a, b| st[a][0].total_cmp(&st[b][0]));
        for net in &by_tooth {
            let s = leg_s(
                state,
                net,
                st[net][0],
                st[net][1],
                far,
                true,
                &placed,
                None,
                &[],
            )?;
            state.join_leg_s.insert(net.clone(), s);
            if state.options.pairs && state.context.pairs.contains_key(net) && state.options.branch
            {
                let half = (state.options.track_width + state.context.cfg.pair_gap) / 2.0;
                placed.push((s - half, st[net][1].min(far), st[net][1].max(far)));
                placed.push((s + half, st[net][1].min(far), st[net][1].max(far)));
            } else {
                placed.push((s, st[net][1].min(far), st[net][1].max(far)));
            }
        }
        joiners.sort_by(|a, b| {
            state.join_leg_s[a]
                .total_cmp(&state.join_leg_s[b])
                .then(st[a][0].total_cmp(&st[b][0]))
        });
        let min_leg = joiners
            .iter()
            .map(|net| state.join_leg_s[net])
            .fold(f64::INFINITY, f64::min);
        base = clear_block(state, base, side, min_leg, state.s0, &joiners[0])?;
        let mut gaps = vec![0.0; joiners.len().saturating_sub(1)];
        for i in 1..joiners.len() {
            gaps[i - 1] = pair_floor(
                state,
                &joiners[i - 1],
                &joiners[i],
                state.options.lane_pitch,
                None,
                true,
                1.0,
            )?;
        }
        let sum: f64 = gaps.iter().sum();
        for (i, net) in joiners.iter().enumerate() {
            launch_o.insert(
                net.clone(),
                base + side * (sum - gaps.iter().take(i).sum::<f64>()),
            );
            state.join_block.insert(net.clone(), launch_o[net]);
        }
    }

    let pin: BTreeSet<_> = state.pin_target.clone();
    let unpinned: Vec<_> = state
        .heads_e
        .iter()
        .filter(|net| !pin.contains(*net))
        .cloned()
        .collect();
    let head_exit = head_order(state, &unpinned, &se)?;
    let mut floors = Vec::with_capacity(head_exit.len().saturating_sub(1));
    for i in 1..head_exit.len() {
        floors.push(pair_floor(
            state,
            &head_exit[i - 1],
            &head_exit[i],
            state.options.exit_pitch,
            schedule,
            false,
            1.0,
        )?);
    }
    for i in 0..head_exit.len().saturating_sub(1) {
        let a = &head_exit[i];
        let b = &head_exit[i + 1];
        if state.row_slot.contains_key(a)
            && state.row_slot.contains_key(b)
            && state.context.dest_layer.get(a) != state.context.dest_layer.get(b)
        {
            let via = schedule.is_some_and(|sc| {
                [a, b].iter().any(|n| {
                    sc.page
                        .get(*n)
                        .and_then(Option::as_deref)
                        .is_some_and(|page| {
                            state
                                .context
                                .dest_layer
                                .get(*n)
                                .is_some_and(|layer| page != layer)
                        })
                })
            });
            floors[i] = if via { state.via_need() } else { 0.05 };
        } else if schedule.is_some()
            && (state.row_slot.contains_key(a) || state.row_slot.contains_key(b))
        {
            let swim_a = schedule.unwrap().page.get(a).is_some_and(Option::is_none);
            let swim_b = schedule.unwrap().page.get(b).is_some_and(Option::is_none);
            if swim_a != swim_b {
                let page_net = if swim_a { b } else { a };
                let slope = state.slope.get(page_net).copied().unwrap_or(0.0).abs();
                floors[i] = floors[i].max(
                    (state.options.exit_pitch + lane_w(state, a) + lane_w(state, b))
                        * (1.0 + slope * slope).sqrt(),
                );
            }
        }
    }
    let desired: Vec<_> = head_exit
        .iter()
        .map(|net| state.row_slot.get(net).copied().unwrap_or(se[net][1]))
        .collect();
    let mut py = if head_exit.is_empty() {
        Vec::new()
    } else {
        relax_pitch(&desired, &floors)
    };
    if !state.row_slot.is_empty() {
        for (i, net) in head_exit.iter().enumerate() {
            if let Some(bound) = state.row_slot.get(net) {
                if state.row_side.get(net).copied().unwrap_or(0) > 0 {
                    py[i] = py[i].min(*bound);
                } else {
                    py[i] = py[i].max(*bound);
                }
            }
        }
        let low: Vec<_> = head_exit
            .iter()
            .enumerate()
            .filter_map(|(i, net)| (state.row_side.get(net) == Some(&1)).then_some(i))
            .collect();
        let high: Vec<_> = head_exit
            .iter()
            .enumerate()
            .filter_map(|(i, net)| (state.row_side.get(net) == Some(&-1)).then_some(i))
            .collect();
        if let Some(last_low) = low.iter().max().copied() {
            for i in (0..last_low).rev() {
                let mut bound =
                    py[i + 1] - floors.get(i).copied().unwrap_or(state.options.exit_pitch);
                if state.row_slot.contains_key(&head_exit[i]) {
                    if let Some(j) = (i + 1..head_exit.len()).find(|j| {
                        state.row_slot.contains_key(&head_exit[*j])
                            && state.context.dest_layer.get(&head_exit[*j])
                                == state.context.dest_layer.get(&head_exit[i])
                    }) {
                        bound = bound.min(
                            py[j]
                                - secant_floor(
                                    state,
                                    &head_exit[i],
                                    &head_exit[j],
                                    state.options.exit_pitch,
                                    schedule,
                                ),
                        );
                    }
                }
                py[i] = py[i].min(bound);
            }
        }
        if let Some(first) = high.iter().min().copied() {
            for i in first + 1..head_exit.len() {
                let mut bound = py[i - 1]
                    + floors
                        .get(i - 1)
                        .copied()
                        .unwrap_or(state.options.exit_pitch);
                if state.row_slot.contains_key(&head_exit[i]) {
                    if let Some(j) = (0..i).rev().find(|j| {
                        state.row_slot.contains_key(&head_exit[*j])
                            && state.context.dest_layer.get(&head_exit[*j])
                                == state.context.dest_layer.get(&head_exit[i])
                    }) {
                        bound = bound.max(
                            py[j]
                                + secant_floor(
                                    state,
                                    &head_exit[i],
                                    &head_exit[j],
                                    state.options.exit_pitch,
                                    schedule,
                                ),
                        );
                    }
                }
                py[i] = py[i].max(bound);
            }
        }
    }
    let mut target_o: BTreeMap<String, f64> =
        head_exit.iter().cloned().zip(py.iter().copied()).collect();
    for net in &pin {
        if state.heads_e.contains(net) {
            target_o.insert(net.clone(), se[net][1]);
        }
    }

    state.exit_block.clear();
    for side in [-1.0, 1.0] {
        let xs: Vec<_> = state
            .siders
            .iter()
            .filter(|net| state.exit_side.get(*net).copied().unwrap_or(0) as f64 == side)
            .cloned()
            .collect();
        if xs.is_empty() {
            continue;
        }
        let mut ports: Vec<_> = xs
            .iter()
            .filter(|net| !state.join_block.contains_key(*net))
            .cloned()
            .collect();
        ports.sort_by(|a, b| se[a][0].total_cmp(&se[b][0]));
        let mut joined: Vec<_> = xs
            .iter()
            .filter(|net| state.join_block.contains_key(*net))
            .cloned()
            .collect();
        joined.sort_by(|a, b| st[b][0].total_cmp(&st[a][0]));
        let mut order = ports;
        order.extend(joined);
        if state.keep_order {
            order = xs.clone();
            order.sort_by(|a, b| (side * launch_o[a]).total_cmp(&(side * launch_o[b])));
        }
        let extent = py
            .iter()
            .map(|o| side * o)
            .chain(xs.iter().map(|net| side * se[net][1]))
            .fold(f64::NEG_INFINITY, f64::max);
        let mut gap = state.block_gap();
        let edge = head_exit
            .iter()
            .max_by(|a, b| {
                (side * target_o.get(*a).copied().unwrap_or(se[*a][1]))
                    .total_cmp(&(side * target_o.get(*b).copied().unwrap_or(se[*b][1])))
            })
            .cloned();
        if let Some(edge) = edge.filter(|name| state.row_slot.contains_key(name)) {
            gap = gap.max(secant_floor(
                state,
                &edge,
                &order[0],
                state.options.lane_pitch,
                schedule,
            ));
        }
        let mut base =
            side * (extent + gap).max(-state.options.lane_pitch * (xs.len() as f64 - 1.0) / 2.0);
        let in_branch = state.is_branch;
        let to_branch = state.options.branch && xs.len() >= 2 && !in_branch;
        let s_run = if to_branch {
            state.s1 + HAND_DS
        } else {
            xs.iter()
                .map(|net| se[net][0])
                .fold(f64::NEG_INFINITY, f64::max)
        };
        if !to_branch && !in_branch {
            base = clear_block(state, base, side, state.s1, s_run, &order[0])?;
        }
        let mut gaps = Vec::with_capacity(order.len().saturating_sub(1));
        for i in 1..order.len() {
            gaps.push(pair_floor(
                state,
                &order[i - 1],
                &order[i],
                state.options.lane_pitch,
                schedule,
                false,
                side,
            )?);
        }
        if !in_branch {
            gaps = fit_block(state, base, side, &order, &gaps, s_run)?;
        }
        let mut accumulated = 0.0;
        for (i, net) in order.iter().enumerate() {
            if i > 0 {
                accumulated += gaps[i - 1];
            }
            let offset = base + side * accumulated;
            target_o.insert(net.clone(), offset);
            state.exit_block.insert(net.clone(), offset);
        }
    }
    if state.is_branch {
        state.s0 = state.s0_base;
        launch_o = state
            .members
            .iter()
            .map(|net| (net.clone(), st[net][1]))
            .collect();
        target_o = launch_o.clone();
        state.exit_block = state
            .siders
            .iter()
            .filter_map(|net| launch_o.get(net).map(|offset| (net.clone(), *offset)))
            .collect();
    }
    state.launch_o = launch_o;
    state.target_o = target_o;
    state.target = state.members.clone();
    state
        .target
        .sort_by(|a, b| state.target_o[a].total_cmp(&state.target_o[b]));
    state.launch = state.members.clone();
    state
        .launch
        .sort_by(|a, b| state.launch_o[a].total_cmp(&state.launch_o[b]));
    state.ly = state.launch.iter().map(|net| state.launch_o[net]).collect();
    state.py = state.target.iter().map(|net| state.target_o[net]).collect();
    Ok(())
}
