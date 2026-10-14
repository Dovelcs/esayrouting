//! Source-equivalent inputs for the plan-only `braid.setup` path.
use super::{
    board_snapshot::BoardSnapshot,
    braid_corridor::BraidCorridorOptions,
    braid_plan::BraidPlanInput,
    corridor::{self, Point},
    escape_moves::{self, Grid},
    pairs,
    taut_paths::Obstacles,
    whole_context::{Ends, WholeContext},
    whole_frame::{FlowFrame, PerimeterCut},
};
use std::collections::{BTreeMap, BTreeSet};

const SNAP_TOL: f64 = 0.006;
const HUG_OVER: f64 = 0.005;
const BASE_LANE_PITCH: f64 = 0.35;
const BASE_EXIT_PITCH: f64 = 0.38;
const DEFAULT_PAGES_SIDERS: u8 = 1;
const DEFAULT_BRANCH: bool = true;
const DEFAULT_SLOPE_PITCH: bool = true;

#[derive(Clone, Debug, Default)]
pub struct BraidCorridorOverrides {
    pub track_width: Option<f64>,
    pub spec_clearance: Option<f64>,
    pub via_size: Option<f64>,
    pub grid: Option<f64>,
    pub lane_pitch: Option<f64>,
    pub exit_pitch: Option<f64>,
    pub pairs: Option<bool>,
    pub block_push: Option<u8>,
    pub pair_gap: Option<f64>,
    pub pair_dive_extra_pitches: Option<f64>,
    pub jog_via: Option<f64>,
    pub approach_reserve: Option<f64>,
    pub end_keep: Option<f64>,
    pub pages_siders: Option<u8>,
    pub slope_pitch: Option<bool>,
    pub branch: Option<bool>,
}

/// Resolve the fixed source's braid settings from the actual route rules.
/// `clearance` in BraidCorridorOptions is braid.CLEAR: the spec clearance plus
/// the fixed 5 um hug margin.
pub fn resolve_corridor_options(
    rules: &super::whole_context::RoutingRules,
    overrides: &BraidCorridorOverrides,
) -> Result<BraidCorridorOptions, String> {
    rules.validate()?;
    let round6 = |v: f64| round_even(v * 1_000_000.0) / 1_000_000.0;
    let track_width = round6(overrides.track_width.unwrap_or(rules.track_width));
    let spec_clearance = round6(overrides.spec_clearance.unwrap_or(rules.clearance));
    let clearance = round6(spec_clearance + HUG_OVER);
    let lane_slice = round6(track_width + clearance);
    let lane_pitch = overrides
        .lane_pitch
        .unwrap_or_else(|| BASE_LANE_PITCH.max(lane_slice));
    let exit_pitch = overrides
        .exit_pitch
        .unwrap_or_else(|| BASE_EXIT_PITCH.max(lane_slice));
    let pair_gap = overrides
        .pair_gap
        .filter(|value| *value != 0.0)
        .unwrap_or(rules.pair_gap);
    let options = BraidCorridorOptions {
        track_width,
        clearance,
        half_sep: (track_width + spec_clearance) / 2.0,
        via_size: round6(overrides.via_size.unwrap_or(rules.via_size)),
        pair_gap,
        lane_pitch,
        exit_pitch,
        grid: round6(overrides.grid.unwrap_or(rules.grid_step)),
        pairs: overrides.pairs.unwrap_or(false),
        block_push: overrides.block_push.unwrap_or(1),
        // braid.plan_braid calls setup without the optional plan-marker
        // plumbing used by the production writer; this route is not pages-first.
        pages_first: false,
        pages_siders: overrides.pages_siders.unwrap_or(DEFAULT_PAGES_SIDERS),
        slope_pitch: overrides.slope_pitch.unwrap_or(DEFAULT_SLOPE_PITCH),
        branch: overrides.branch.unwrap_or(DEFAULT_BRANCH),
        pair_dive_extra: overrides.pair_dive_extra_pitches.unwrap_or(2.0)
            * (track_width + pair_gap),
        jog_via: overrides.jog_via.unwrap_or(1.0),
        approach_reserve: overrides.approach_reserve.unwrap_or(1.0),
        end_keep: overrides.end_keep.unwrap_or(track_width + clearance + 0.05),
    };
    options.validate()?;
    Ok(options)
}

#[derive(Clone)]
pub struct BraidSetupResult {
    pub snapshot: BoardSnapshot,
    pub context: WholeContext,
    pub source_grid: Grid,
    pub destination_grid: Grid,
    pub source_centres: BTreeMap<String, Point>,
    pub destination_centres: BTreeMap<String, Point>,
    pub source_refs: BTreeMap<String, String>,
    pub kids: BTreeSet<i32>,
    pub pair_members: BTreeMap<String, [String; 2]>,
    pub berth_sig: BTreeMap<String, String>,
    pub chi: i32,
    pub mirror_axis_y: Option<f64>,
    pub pages_first: bool,
    pub dest_cut: Option<PerimeterCut>,
    pub spine_obstacles: Obstacles,
    pub pad_obstacles: Obstacles,
    pub groups: Vec<Vec<String>>,
}

fn point_before_end(path: &[Point], distance: f64) -> Point {
    if path.is_empty() {
        return [0.0, 0.0];
    }
    let mut left = distance;
    for segment in path.windows(2).rev() {
        let length = corridor::distance(segment[0], segment[1]);
        if length >= left {
            let fraction = left / length;
            return [
                segment[1][0] + (segment[0][0] - segment[1][0]) * fraction,
                segment[1][1] + (segment[0][1] - segment[1][1]) * fraction,
            ];
        }
        left -= length;
    }
    path[0]
}

fn unit_or_x(v: Point) -> Point {
    let length = corridor::length(v);
    if length <= 1e-12 {
        [1.0, 0.0]
    } else {
        [v[0] / length, v[1] / length]
    }
}

fn angle_degrees(a: Point, b: Point) -> f64 {
    corridor::dot(a, b).clamp(-1.0, 1.0).acos().to_degrees()
}

fn find_root(parent: &mut [usize], mut i: usize) -> usize {
    while parent[i] != i {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    i
}

fn first_max_by_score<'a, T>(values: &'a [T], mut score: impl FnMut(&T) -> f64) -> Option<&'a T> {
    let mut best = values.first()?;
    let mut best_score = score(best);
    for value in &values[1..] {
        let next = score(value);
        if next > best_score {
            best = value;
            best_score = next;
        }
    }
    Some(best)
}

fn first_min_by_score<'a, T>(values: &'a [T], mut score: impl FnMut(&T) -> f64) -> Option<&'a T> {
    let mut best = values.first()?;
    let mut best_score = score(best);
    for value in &values[1..] {
        let next = score(value);
        if next < best_score {
            best = value;
            best_score = next;
        }
    }
    Some(best)
}

fn centre_of_context(ctx: &WholeContext, reference: &str) -> Result<Point, String> {
    let pads: Vec<_> = ctx
        .pads
        .iter()
        .filter(|p| p.component_ref == reference)
        .collect();
    if pads.is_empty() {
        return Err(format!("braid setup component {reference} has no pads"));
    }
    let (x, y) = pads
        .iter()
        .fold((0.0, 0.0), |(x, y), p| (x + p.point[0], y + p.point[1]));
    Ok([x / pads.len() as f64, y / pads.len() as f64])
}

fn grid_of_context(ctx: &WholeContext, reference: &str) -> Result<Grid, String> {
    let pads: Vec<_> = ctx
        .pads
        .iter()
        .filter(|p| p.component_ref == reference)
        .map(|p| escape_moves::Pad {
            key: p.key.clone(),
            net: p.net.clone(),
            net_id: p.net_id,
            component_ref: p.component_ref.clone(),
            pad_number: p.pad_number.clone(),
            point: p.point,
            layers: p.layers.clone(),
            size: p.size,
        })
        .collect();
    escape_moves::grid_of(&pads)
}

fn mirror_layer(name: &str) -> String {
    if let Some(rest) = name.strip_prefix("F.") {
        format!("B.{rest}")
    } else if let Some(rest) = name.strip_prefix("B.") {
        format!("F.{rest}")
    } else {
        name.to_owned()
    }
}

fn round_even(value: f64) -> f64 {
    let floor = value.floor();
    let frac = value - floor;
    if (frac - 0.5).abs() < 1e-12 {
        if (floor as i64).rem_euclid(2) == 0 {
            floor
        } else {
            floor + 1.0
        }
    } else {
        value.round()
    }
}

fn mirror_layers(ctx: &mut WholeContext) {
    for layer in &mut ctx.layers {
        *layer = mirror_layer(layer);
    }
    for layer in &mut ctx.copper_layers {
        *layer = mirror_layer(layer);
    }
    for pad in &mut ctx.pads {
        for layer in &mut pad.layers {
            *layer = mirror_layer(layer);
        }
        for shape in &mut pad.copper_shapes {
            shape.layer = mirror_layer(&shape.layer);
        }
    }
    for segment in &mut ctx.base_segments {
        segment.layer = mirror_layer(&segment.layer);
    }
    for via in &mut ctx.base_vias {
        for layer in &mut via.layers {
            *layer = mirror_layer(layer);
        }
    }
    for layer in ctx
        .tooth_layer
        .values_mut()
        .chain(ctx.dest_layer.values_mut())
    {
        *layer = mirror_layer(layer);
    }
}

fn build_corridor_groups(
    ctx: &WholeContext,
    names: &[String],
    spine_obs: &Obstacles,
    pad_obs: &Obstacles,
    options: &BraidCorridorOptions,
    destination_centres: &BTreeMap<String, Point>,
    source_centres: &BTreeMap<String, Point>,
) -> Result<Vec<Vec<String>>, String> {
    if names.is_empty() {
        return Ok(Vec::new());
    }
    let mut parent: Vec<usize> = (0..names.len()).collect();
    let indices: BTreeMap<_, _> = names
        .iter()
        .enumerate()
        .map(|(i, n)| (n.as_str(), i))
        .collect();
    let arrivals: BTreeMap<_, _> = names
        .iter()
        .map(|n| {
            let path = ctx
                .paths
                .get(n)
                .ok_or_else(|| format!("corridor clustering missing taut path {n}"))?;
            let centre = *destination_centres
                .get(n)
                .ok_or_else(|| format!("corridor clustering missing destination centre for {n}"))?;
            Ok((
                n.clone(),
                unit_or_x(corridor::sub(point_before_end(path, 2.0), centre)),
            ))
        })
        .collect::<Result<_, String>>()?;
    for (i, a) in names.iter().enumerate() {
        for b in &names[i + 1..] {
            if ctx.ends[a].destination != ctx.ends[b].destination {
                continue;
            }
            if corridor::distance(ctx.ends[a].berth, ctx.ends[b].berth) > 6.0 {
                continue;
            }
            if angle_degrees(arrivals[a], arrivals[b]) <= 60.0 {
                let ra = find_root(&mut parent, indices[a.as_str()]);
                let rb = find_root(&mut parent, indices[b.as_str()]);
                if ra != rb {
                    parent[ra] = rb;
                }
            }
        }
    }
    let mut by_root = BTreeMap::<usize, usize>::new();
    let mut groups = Vec::<Vec<String>>::new();
    for (i, name) in names.iter().enumerate() {
        let root = find_root(&mut parent, i);
        let group = *by_root.entry(root).or_insert_with(|| {
            groups.push(Vec::new());
            groups.len() - 1
        });
        groups[group].push(name.clone());
    }
    let mut todo = groups;
    todo.sort_by_key(|g| std::cmp::Reverse(g.len()));
    let mut output = Vec::new();
    while !todo.is_empty() {
        let group = todo.remove(0);
        if group.len() == 1 {
            output.push(group);
            continue;
        }
        let h =
            options.lane_pitch * (group.len().saturating_sub(1) as f64) / 2.0 + options.lane_pitch;
        let spine = super::braid_geometry::build_corridor_spine(
            ctx,
            &group,
            spine_obs,
            &[],
            Some(h),
            false,
        )?;
        let p0 = *spine
            .pts
            .first()
            .ok_or("corridor clustering got an empty spine")?;
        let p1 = *spine
            .pts
            .last()
            .ok_or("corridor clustering got an empty spine")?;
        let d0 = *spine
            .d
            .first()
            .ok_or("corridor clustering got a spine without a start direction")?;
        let dn = *spine
            .d
            .last()
            .ok_or("corridor clustering got a spine without an end direction")?;
        let mut keep = Vec::new();
        let mut split = Vec::new();
        for name in &group {
            let tooth = ctx.ends[name].tooth;
            let stub = ctx.ends[name].berth;
            let t_e = corridor::dot(corridor::sub(stub, p1), dn);
            let t_0 = corridor::dot(corridor::sub(tooth, p0), d0);
            let mut okay = true;
            if t_e > 1.0 {
                let back = corridor::sub(stub, corridor::mul(dn, t_e));
                let centre = destination_centres.get(name).copied();
                okay = run_clear(stub, back, dn, centre, pad_obs)
                    || wrap_clear(stub, dn, centre, pad_obs, 6.0);
            }
            if okay && t_0 < -1.0 {
                let forward = corridor::sub(tooth, corridor::mul(d0, t_0));
                okay = run_clear(
                    tooth,
                    forward,
                    d0,
                    source_centres.get(name).copied(),
                    pad_obs,
                );
            }
            if okay {
                keep.push(name.clone());
            } else {
                split.push(name.clone());
            }
        }
        if !split.is_empty() && !keep.is_empty() {
            output.push(keep);
            todo.extend(initial_groups(ctx, &split, destination_centres)?);
            todo.sort_by_key(|g| std::cmp::Reverse(g.len()));
        } else {
            output.push(group);
        }
    }
    output.sort_by_key(|g| std::cmp::Reverse(g.len()));
    Ok(output)
}

fn initial_groups(
    ctx: &WholeContext,
    names: &[String],
    destination_centres: &BTreeMap<String, Point>,
) -> Result<Vec<Vec<String>>, String> {
    // Re-enter the same single-linkage arrival grouping for a split set.
    if names.len() <= 1 {
        return Ok(names.iter().cloned().map(|n| vec![n]).collect());
    }
    let mut parent: Vec<usize> = (0..names.len()).collect();
    let arrivals: Vec<_> = names
        .iter()
        .map(|n| {
            let path = ctx
                .paths
                .get(n)
                .ok_or_else(|| format!("corridor clustering missing taut path {n}"))?;
            let centre = *destination_centres
                .get(n)
                .ok_or_else(|| format!("corridor clustering missing destination centre for {n}"))?;
            Ok(unit_or_x(corridor::sub(
                point_before_end(path, 2.0),
                centre,
            )))
        })
        .collect::<Result<_, String>>()?;
    for i in 0..names.len() {
        for j in i + 1..names.len() {
            if ctx.ends[&names[i]].destination != ctx.ends[&names[j]].destination
                || corridor::distance(ctx.ends[&names[i]].berth, ctx.ends[&names[j]].berth) > 6.0
            {
                continue;
            }
            if angle_degrees(arrivals[i], arrivals[j]) <= 60.0 {
                let a = find_root(&mut parent, i);
                let b = find_root(&mut parent, j);
                if a != b {
                    parent[a] = b;
                }
            }
        }
    }
    let mut groups = Vec::<Vec<String>>::new();
    let mut roots = BTreeMap::new();
    for (i, name) in names.iter().enumerate() {
        let root = find_root(&mut parent, i);
        let idx = *roots.entry(root).or_insert_with(|| {
            groups.push(Vec::new());
            groups.len() - 1
        });
        groups[idx].push(name.clone());
    }
    groups.sort_by_key(|g| std::cmp::Reverse(g.len()));
    Ok(groups)
}

fn run_clear(
    end: Point,
    far: Point,
    direction: Point,
    centre: Option<Point>,
    obstacles: &Obstacles,
) -> bool {
    if obstacles.seg_clear(end, far) {
        return true;
    }
    let Some(centre) = centre else {
        return false;
    };
    let normal = corridor::sub(end, centre);
    let normal = corridor::sub(
        normal,
        corridor::mul(direction, corridor::dot(normal, direction)),
    );
    let length = corridor::length(normal);
    if length < 1e-9 {
        return false;
    }
    let normal = corridor::mul(normal, 1.0 / length);
    for k in 1..=3 {
        let offset = 0.35 * k as f64;
        let e2 = corridor::add(end, corridor::mul(normal, offset));
        let f2 = corridor::add(far, corridor::mul(normal, offset));
        if obstacles.seg_clear(end, e2) && obstacles.seg_clear(e2, f2) {
            return true;
        }
    }
    false
}

fn wrap_clear(
    end: Point,
    direction: Point,
    centre: Option<Point>,
    obstacles: &Obstacles,
    width: f64,
) -> bool {
    let Some(centre) = centre else {
        return false;
    };
    let normal = corridor::sub(end, centre);
    let normal = corridor::sub(
        normal,
        corridor::mul(direction, corridor::dot(normal, direction)),
    );
    let length = corridor::length(normal);
    let sides = if length > 1e-9 {
        let n = corridor::mul(normal, 1.0 / length);
        vec![n, corridor::mul(n, -1.0)]
    } else {
        vec![[-direction[1], direction[0]], [direction[1], -direction[0]]]
    };
    for k in 1..=3 {
        let e2 = corridor::add(end, corridor::mul(direction, 0.35 * k as f64));
        if !obstacles.seg_clear(end, e2) {
            continue;
        }
        for side in &sides {
            let f2 = corridor::add(e2, corridor::mul(*side, 2.0 * width));
            if obstacles.seg_clear(e2, f2) {
                return true;
            }
        }
    }
    false
}

/// Rebuild the board, endpoints, layers, escape directions and obstacle inputs
/// consumed by the fixed-source `braid.setup` before Corridor construction.
fn configured_end(
    seed: &super::braid_plan::BraidPlanSeed,
    name: &str,
    destination: &str,
) -> Result<Ends, String> {
    let end = seed
        .ends
        .get(name)
        .ok_or_else(|| format!("braid plan has no endpoints for {name}"))?;
    if end.tooth.iter().chain(&end.berth).any(|v| !v.is_finite()) {
        return Err(format!("braid plan has nonfinite endpoint for {name}"));
    }
    Ok(Ends {
        tooth: end.tooth,
        berth: end.berth,
        destination: destination.into(),
    })
}

pub fn prepare(input: &BraidPlanInput<'_>) -> Result<BraidSetupResult, String> {
    if input.source == input.destination {
        return Err("braid setup source and destination must differ".into());
    }
    if input.names.is_empty() {
        return Err("braid setup needs at least one named lane".into());
    }
    input.rules.validate()?;
    input.corridor_options.validate()?;
    let snapshot = input.snapshot;
    if !snapshot
        .pads
        .iter()
        .any(|p| p.component_ref == input.source)
        || !snapshot
            .pads
            .iter()
            .any(|p| p.component_ref == input.destination)
    {
        return Err("braid setup source/destination component is absent from snapshot".into());
    }
    let original_ids = resolve_names(snapshot, input.names)?;
    let mut seen_names = BTreeSet::new();
    if input.names.iter().any(|n| !seen_names.insert(n.as_str())) {
        return Err("braid setup names contain a duplicate lane".into());
    }

    // `braid.setup` only folds a pair into one lane when both its explicit
    // `pairs` argument and the BRAID_PAIRS source setting are enabled.
    let (lane_names, pair_members) = if input.options.pairs && input.corridor_options.pairs {
        pairs::members(input.names, &pairs::pair_names(input.names, true))
    } else {
        (input.names.to_vec(), BTreeMap::new())
    };
    let mut ids = original_ids.clone();
    for (base, legs) in &pair_members {
        let p_id = *original_ids
            .get(&legs[0])
            .ok_or_else(|| format!("braid pair {base} is missing its P leg {}", legs[0]))?;
        if !original_ids.contains_key(&legs[1]) {
            return Err(format!(
                "braid pair {base} is missing its N leg {}",
                legs[1]
            ));
        }
        ids.insert(base.clone(), p_id);
    }
    // Chi is derived after pair aggregation in the source, with each pair's P
    // leg standing in for its member; a plan-side chi then takes precedence.
    let _board_chi = chirality(snapshot, &lane_names, &ids, input.destination)?;
    let chi = input.seed.chi;
    let mirror_axis_y = if chi < 0 {
        let middle = (snapshot.board_bounds[1] + snapshot.board_bounds[3]) / 2.0;
        Some(round_even(middle / 0.05) * 0.05)
    } else {
        None
    };

    let mut physical_ends = BTreeMap::<String, Ends>::new();
    let mut wanted = Vec::new();
    for name in &lane_names {
        if !pair_members.contains_key(name) && !input.seed.ends.contains_key(name) {
            wanted.push(name.clone());
        }
    }
    for legs in pair_members.values() {
        for leg in legs {
            if !input.seed.ends.contains_key(leg) {
                wanted.push(leg.clone());
            }
        }
    }
    wanted.sort();
    wanted.dedup();
    for name in wanted {
        let id = original_ids[&name];
        physical_ends.insert(
            name.clone(),
            endpoints(
                snapshot,
                id,
                input.destination,
                input.corridor_options.via_size,
            )?,
        );
    }
    for name in &lane_names {
        if input.seed.ends.contains_key(name) {
            physical_ends.insert(
                name.clone(),
                configured_end(input.seed, name, input.destination)?,
            );
        }
    }
    for legs in pair_members.values() {
        for leg in legs {
            if input.seed.ends.contains_key(leg) {
                physical_ends.insert(
                    leg.clone(),
                    configured_end(input.seed, leg, input.destination)?,
                );
            }
        }
    }

    let mut ends = BTreeMap::<String, Ends>::new();
    let mut tooth_layer = BTreeMap::new();
    let mut dest_layer = BTreeMap::new();
    let mut tooth_dir = BTreeMap::new();
    let mut stub_dir = BTreeMap::new();
    let mut source_refs = BTreeMap::new();
    let mut kids = BTreeSet::new();
    let mut pair_ends = BTreeMap::<String, [[Point; 2]; 2]>::new();
    let mut pair_layers = BTreeMap::<String, [String; 2]>::new();
    for name in &lane_names {
        let id = ids[name];
        kids.insert(id);
        if let Some(legs) = pair_members.get(name) {
            kids.insert(original_ids[&legs[0]]);
            kids.insert(original_ids[&legs[1]]);
            // The aggregate lane's physical ends are always the P/N midpoints,
            // even if the plan also carried a pair-base end entry.
            if input.seed.ends.contains_key(name) {
                let _ = configured_end(input.seed, name, input.destination)?;
                if input.seed.tooth_layer.get(name).is_none()
                    || input.seed.dest_layer.get(name).is_none()
                    || input.seed.tooth_dir.get(name).is_none()
                    || input.seed.stub_dir.get(name).is_none()
                {
                    return Err(format!(
                        "braid plan has incomplete pair-base metadata for {name}"
                    ));
                }
            }
            let p = physical_ends
                .get(&legs[0])
                .ok_or_else(|| format!("braid setup has no physical P ends for {name}"))?;
            let n = physical_ends
                .get(&legs[1])
                .ok_or_else(|| format!("braid setup has no physical N ends for {name}"))?;
            pair_ends.insert(name.clone(), [[p.tooth, n.tooth], [p.berth, n.berth]]);
            ends.insert(
                name.clone(),
                Ends {
                    tooth: pairs::mid(p.tooth, n.tooth),
                    berth: pairs::mid(p.berth, n.berth),
                    destination: input.destination.into(),
                },
            );
        } else {
            ends.insert(
                name.clone(),
                physical_ends
                    .get(name)
                    .cloned()
                    .ok_or_else(|| format!("braid setup has no physical ends for {name}"))?,
            );
        }
        let end = &ends[name];
        let pads: Vec<_> = snapshot.pads.iter().filter(|p| p.net_id == id).collect();
        if pads.is_empty() {
            return Err(format!("braid setup {name} has no physical pads"));
        }
        let planned = input.seed.ends.contains_key(name);
        let src_ref = first_min_by_score(&pads, |p| corridor::distance(p.point, end.tooth))
            .map(|p| p.component_ref.clone())
            .ok_or_else(|| format!("braid setup cannot resolve source array for {name}"))?;
        let tl = if planned {
            input
                .seed
                .tooth_layer
                .get(name)
                .cloned()
                .ok_or_else(|| format!("braid plan is missing tooth_layer for {name}"))?
        } else {
            layer_at(snapshot, id, end.tooth, &pads)
        };
        let dl = if planned {
            input
                .seed
                .dest_layer
                .get(name)
                .cloned()
                .ok_or_else(|| format!("braid plan is missing dest_layer for {name}"))?
        } else {
            layer_at(snapshot, id, end.berth, &pads)
        };
        let td = if planned {
            *input
                .seed
                .tooth_dir
                .get(name)
                .ok_or_else(|| format!("braid plan is missing tooth_dir for {name}"))?
        } else {
            end_direction(snapshot, id, end.tooth, &pads)
        };
        let sd = if planned {
            *input
                .seed
                .stub_dir
                .get(name)
                .ok_or_else(|| format!("braid plan is missing stub_dir for {name}"))?
        } else {
            end_direction(snapshot, id, end.berth, &pads)
        };
        if td.iter().chain(&sd).any(|v| !v.is_finite()) {
            return Err(format!(
                "braid setup has nonfinite end direction for {name}"
            ));
        }
        ends.insert(name.clone(), end.clone());
        tooth_layer.insert(name.clone(), tl);
        dest_layer.insert(name.clone(), dl);
        tooth_dir.insert(name.clone(), td);
        stub_dir.insert(name.clone(), sd);
        source_refs.insert(name.clone(), src_ref);
    }

    // Source pair metadata comes from each physical P/N leg, not from the
    // midpoint's inferred layer or direction. Preserve the original P/N order.
    for (base, legs) in &pair_members {
        let p_id = original_ids[&legs[0]];
        let n_id = original_ids[&legs[1]];
        let p_pads: Vec<_> = snapshot.pads.iter().filter(|p| p.net_id == p_id).collect();
        let n_pads: Vec<_> = snapshot.pads.iter().filter(|p| p.net_id == n_id).collect();
        let p = &physical_ends[&legs[0]];
        let n = &physical_ends[&legs[1]];
        let p_tl = input
            .seed
            .tooth_layer
            .get(&legs[0])
            .cloned()
            .unwrap_or_else(|| layer_at(snapshot, p_id, p.tooth, &p_pads));
        let p_dl = input
            .seed
            .dest_layer
            .get(&legs[0])
            .cloned()
            .unwrap_or_else(|| layer_at(snapshot, p_id, p.berth, &p_pads));
        let n_tl = input
            .seed
            .tooth_layer
            .get(&legs[1])
            .cloned()
            .unwrap_or_else(|| layer_at(snapshot, n_id, n.tooth, &n_pads));
        let n_dl = input
            .seed
            .dest_layer
            .get(&legs[1])
            .cloned()
            .unwrap_or_else(|| layer_at(snapshot, n_id, n.berth, &n_pads));
        let p_td = input
            .seed
            .tooth_dir
            .get(&legs[0])
            .copied()
            .unwrap_or_else(|| end_direction(snapshot, p_id, p.tooth, &p_pads));
        let p_sd = input
            .seed
            .stub_dir
            .get(&legs[0])
            .copied()
            .unwrap_or_else(|| end_direction(snapshot, p_id, p.berth, &p_pads));
        if p_td.iter().chain(&p_sd).any(|v| !v.is_finite()) {
            return Err(format!(
                "braid setup has nonfinite P-leg direction for {base}"
            ));
        }
        tooth_layer.insert(base.clone(), p_tl);
        dest_layer.insert(base.clone(), p_dl);
        tooth_dir.insert(base.clone(), p_td);
        stub_dir.insert(base.clone(), p_sd);
        pair_layers.insert(base.clone(), [n_tl, n_dl]);
    }

    let mut byname = BTreeMap::<String, i32>::new();
    for (full, &id) in &snapshot.byname {
        let short = short_name(full).to_owned();
        if byname
            .insert(short.clone(), id)
            .is_some_and(|old| old != id)
        {
            return Err(format!("braid setup has ambiguous native basename {short}"));
        }
    }
    for (base, legs) in &pair_members {
        byname.insert(base.clone(), original_ids[&legs[0]]);
    }
    let plane_nets = snapshot
        .zones
        .iter()
        .filter_map(|z| {
            snapshot
                .byname
                .iter()
                .find(|(_, id)| **id == z.net_id)
                .map(|(name, _)| name.clone())
        })
        .collect();
    let native_to_logical_components = snapshot
        .pads
        .iter()
        .map(|p| (p.key.clone(), p.component_ref.clone()))
        .collect();
    let copper_layers: Vec<_> = snapshot
        .layers
        .iter()
        .filter(|l| is_copper_layer(l))
        .cloned()
        .collect();
    let mut rules = input.rules.clone();
    rules.track_width = input.corridor_options.track_width;
    rules.clearance = (input.corridor_options.clearance - HUG_OVER).max(0.0);
    rules.via_size = input.corridor_options.via_size;
    rules.grid_step = input.corridor_options.grid;
    rules.lane_pitch = input.corridor_options.lane_pitch;
    rules.lane_min = rules.track_width + input.corridor_options.clearance + 0.02;
    rules.pair_gap = input.corridor_options.pair_gap;
    let mut context = WholeContext {
        cfg: rules,
        layers: copper_layers.clone(),
        copper_layers,
        board_bounds: snapshot.board_bounds,
        board_outline: snapshot.board_outline.clone(),
        pads: snapshot.pads.clone(),
        source: input.source.into(),
        destination: input.destination.into(),
        lanes: lane_names.clone(),
        byname,
        pairs: pair_members.clone(),
        ends,
        src_ref: source_refs.clone(),
        tooth_layer,
        dest_layer,
        tooth_dir,
        stub_dir,
        pair_ends,
        pair_layers,
        paths: BTreeMap::new(),
        base_segments: snapshot.segments.clone(),
        base_vias: snapshot.vias.clone(),
        dest_cut: None,
        plane_nets,
        native_to_logical_components,
        chirality_mirrored: mirror_axis_y.is_some(),
    };
    if let Some(axis) = mirror_axis_y {
        context = context.transformed(FlowFrame {
            mirror_axis_y: Some(axis),
            quarter_turns: 0,
            center: [0.0, 0.0],
        });
        mirror_layers(&mut context);
    }
    let back = context
        .tooth_layer
        .values()
        .filter(|l| *l == "B.Cu")
        .count();
    let bundle_layer = if 2 * back > context.tooth_layer.len() {
        "B.Cu"
    } else {
        "F.Cu"
    };
    context.paths = super::taut_paths::paths(&context, bundle_layer)?;

    let mut end_refs: BTreeSet<String> = context.src_ref.values().cloned().collect();
    end_refs.extend(context.ends.values().map(|e| e.destination.clone()));
    let obstacles =
        super::braid_geometry::board_obstacles(&context, &kids, bundle_layer, &end_refs)?;
    let source_grid = grid_of_context(&context, input.source)?;
    let destination_grid = grid_of_context(&context, input.destination)?;
    let source_centres: BTreeMap<_, _> = lane_names
        .iter()
        .map(|n| {
            let reference = context
                .src_ref
                .get(n)
                .ok_or_else(|| format!("braid setup missing source ref for {n}"))?;
            Ok((n.clone(), centre_of_context(&context, reference)?))
        })
        .collect::<Result<_, String>>()?;
    let destination_centres: BTreeMap<_, _> = lane_names
        .iter()
        .map(|n| {
            let reference = &context.ends[n].destination;
            Ok((n.clone(), centre_of_context(&context, reference)?))
        })
        .collect::<Result<_, String>>()?;
    let groups = build_corridor_groups(
        &context,
        &lane_names,
        &obstacles.spine_obs,
        &obstacles.pad_obs,
        input.corridor_options,
        &destination_centres,
        &source_centres,
    )?;
    Ok(BraidSetupResult {
        snapshot: snapshot.clone(),
        context,
        source_grid,
        destination_grid,
        source_centres,
        destination_centres,
        source_refs,
        kids,
        pair_members,
        berth_sig: input.seed.berth_sig.clone(),
        chi,
        mirror_axis_y,
        pages_first: false,
        dest_cut: None,
        spine_obstacles: obstacles.spine_obs,
        pad_obstacles: obstacles.pad_obs,
        groups,
    })
}

#[derive(Default)]
struct Snapper {
    points: Vec<Point>,
    buckets: BTreeMap<(i64, i64), Vec<usize>>,
}
impl Snapper {
    fn key(&mut self, p: Point) -> usize {
        let cell = 2.0 * SNAP_TOL;
        let bucket = ((p[0] / cell).floor() as i64, (p[1] / cell).floor() as i64);
        for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(candidates) = self.buckets.get(&(bucket.0 + dx, bucket.1 + dy)) {
                    for &i in candidates {
                        let old = self.points[i];
                        if (old[0] - p[0]).abs() <= SNAP_TOL && (old[1] - p[1]).abs() <= SNAP_TOL {
                            return i;
                        }
                    }
                }
            }
        }
        let point = [
            (p[0] * 1000.0).round() / 1000.0,
            (p[1] * 1000.0).round() / 1000.0,
        ];
        let i = self.points.len();
        self.points.push(point);
        self.buckets.entry(bucket).or_default().push(i);
        i
    }
}

fn short_name(name: &str) -> &str {
    name.rsplit('/').next().unwrap_or(name)
}

fn resolve_names(
    snapshot: &BoardSnapshot,
    names: &[String],
) -> Result<BTreeMap<String, i32>, String> {
    let mut aliases = BTreeMap::<String, Vec<(String, i32)>>::new();
    for (name, &id) in &snapshot.byname {
        aliases
            .entry(short_name(name).to_owned())
            .or_default()
            .push((name.clone(), id));
    }
    let mut out = BTreeMap::new();
    for name in names {
        let candidates = aliases
            .get(name)
            .ok_or_else(|| format!("braid setup unknown net {name}"))?;
        if candidates.len() != 1 {
            return Err(format!("braid setup ambiguous net basename {name}"));
        }
        out.insert(name.clone(), candidates[0].1);
    }
    Ok(out)
}

fn point_box(p: Point, pad: &super::whole_context::ContextPad, margin: f64) -> bool {
    (p[0] - pad.point[0]).abs() <= pad.size[0] / 2.0 + margin
        && (p[1] - pad.point[1]).abs() <= pad.size[1] / 2.0 + margin
}

fn grid_for(snapshot: &BoardSnapshot, reference: &str) -> Result<Grid, String> {
    let pads: Vec<_> = snapshot
        .pads
        .iter()
        .filter(|p| p.component_ref == reference)
        .map(|p| escape_moves::Pad {
            key: p.key.clone(),
            net: p.net.clone(),
            net_id: p.net_id,
            component_ref: p.component_ref.clone(),
            pad_number: p.pad_number.clone(),
            point: p.point,
            layers: p.layers.clone(),
            size: p.size,
        })
        .collect();
    escape_moves::grid_of(&pads)
}

fn pad_box(snapshot: &BoardSnapshot, reference: &str) -> Result<[f64; 4], String> {
    let pads: Vec<_> = snapshot
        .pads
        .iter()
        .filter(|p| p.component_ref == reference)
        .collect();
    if pads.is_empty() {
        return Err(format!(
            "braid setup component {reference} has no mapped pads"
        ));
    }
    Ok([
        pads.iter()
            .map(|p| p.point[0])
            .fold(f64::INFINITY, f64::min),
        pads.iter()
            .map(|p| p.point[1])
            .fold(f64::INFINITY, f64::min),
        pads.iter()
            .map(|p| p.point[0])
            .fold(f64::NEG_INFINITY, f64::max),
        pads.iter()
            .map(|p| p.point[1])
            .fold(f64::NEG_INFINITY, f64::max),
    ])
}

fn centre_of(snapshot: &BoardSnapshot, reference: &str) -> Result<Point, String> {
    let pads: Vec<_> = snapshot
        .pads
        .iter()
        .filter(|p| p.component_ref == reference)
        .collect();
    if pads.is_empty() {
        return Err(format!(
            "braid setup component {reference} has no mapped pads"
        ));
    }
    let (x, y) = pads
        .iter()
        .fold((0.0, 0.0), |(x, y), p| (x + p.point[0], y + p.point[1]));
    Ok([x / pads.len() as f64, y / pads.len() as f64])
}

fn chirality(
    snapshot: &BoardSnapshot,
    names: &[String],
    ids: &BTreeMap<String, i32>,
    destination: &str,
) -> Result<i32, String> {
    let mut source_points = BTreeMap::<String, Point>::new();
    let mut destination_points = BTreeMap::<String, Point>::new();
    let mut source_refs = Vec::<String>::new();
    let mut counts = BTreeMap::<String, usize>::new();
    for name in names {
        let id = ids[name];
        if let Some(p) = snapshot
            .pads
            .iter()
            .find(|p| p.net_id == id && p.component_ref == destination)
        {
            destination_points.insert(name.clone(), p.point);
        }
        if let Some(p) = snapshot
            .pads
            .iter()
            .find(|p| p.net_id == id && p.component_ref != destination)
        {
            source_points.insert(name.clone(), p.point);
            if !counts.contains_key(&p.component_ref) {
                source_refs.push(p.component_ref.clone());
            }
            *counts.entry(p.component_ref.clone()).or_default() += 1;
        }
    }
    if source_points.is_empty() || destination_points.is_empty() {
        return Ok(1);
    }
    let mut source_ref = source_refs
        .first()
        .cloned()
        .ok_or("braid setup cannot identify source chirality array")?;
    for reference in source_refs.iter().skip(1) {
        if counts[reference] > counts[&source_ref] {
            source_ref = reference.clone();
        }
    }
    let source_box = pad_box(snapshot, &source_ref)?;
    let destination_box = pad_box(snapshot, destination)?;
    let source_center = [
        (source_box[0] + source_box[2]) / 2.0,
        (source_box[1] + source_box[3]) / 2.0,
    ];
    let destination_center = [
        (destination_box[0] + destination_box[2]) / 2.0,
        (destination_box[1] + destination_box[3]) / 2.0,
    ];
    let axis = corridor::sub(destination_center, source_center);
    let mut moment = 0.0;
    for name in names {
        if let Some(p) = source_points.get(name) {
            moment += corridor::cross(axis, corridor::sub(*p, source_center));
        }
    }
    for name in names {
        if let Some(p) = destination_points.get(name) {
            moment += corridor::cross(axis, corridor::sub(*p, source_center));
        }
    }
    Ok(if moment < 0.0 { -1 } else { 1 })
}

fn is_copper_layer(layer: &str) -> bool {
    layer.ends_with(".Cu") && layer != "*.Cu"
}

fn layer_at(
    snapshot: &BoardSnapshot,
    net_id: i32,
    point: Point,
    pads: &[&super::whole_context::ContextPad],
) -> String {
    let mut best: Option<(f64, String)> = None;
    for segment in snapshot.segments.iter().filter(|s| s.net_id == net_id) {
        let touches = (segment.a[0] - point[0]).abs() + (segment.a[1] - point[1]).abs() < 0.005
            || (segment.b[0] - point[0]).abs() + (segment.b[1] - point[1]).abs() < 0.005;
        if !touches {
            continue;
        }
        let length = corridor::distance(segment.a, segment.b);
        if best.as_ref().is_none_or(|(old, _)| length > *old) {
            best = Some((length, segment.layer.clone()));
        }
    }
    if let Some((_, layer)) = best {
        return layer;
    }
    let mut near = pads.to_vec();
    near.sort_by(|a, b| {
        corridor::distance(a.point, point).total_cmp(&corridor::distance(b.point, point))
    });
    for pad in near {
        if (pad.point[0] - point[0]).abs() <= pad.size[0] / 2.0 + 0.02
            && (pad.point[1] - point[1]).abs() <= pad.size[1] / 2.0 + 0.02
        {
            let layers: Vec<_> = pad.layers.iter().filter(|l| is_copper_layer(l)).collect();
            if layers.len() == 1 {
                return layers[0].clone();
            }
        }
    }
    "F.Cu".into()
}

fn owner(
    snapshot: &BoardSnapshot,
    net_id: i32,
    point: Point,
    pads: &[&super::whole_context::ContextPad],
) -> String {
    let mut snap = Snapper::default();
    let mut adjacency = BTreeMap::<usize, BTreeSet<usize>>::new();
    for segment in snapshot.segments.iter().filter(|s| s.net_id == net_id) {
        let a = snap.key(segment.a);
        let b = snap.key(segment.b);
        if a != b {
            adjacency.entry(a).or_default().insert(b);
            adjacency.entry(b).or_default().insert(a);
        }
    }
    let start = snap.key(point);
    let mut seen = BTreeSet::from([start]);
    let mut stack = vec![start];
    while let Some(u) = stack.pop() {
        for &v in adjacency.get(&u).into_iter().flatten() {
            if seen.insert(v) {
                stack.push(v);
            }
        }
    }
    for pad in pads {
        if seen.iter().any(|&i| point_box(snap.points[i], pad, 0.02)) {
            return pad.component_ref.clone();
        }
    }
    first_min_by_score(pads, |p| corridor::distance(p.point, point))
        .map(|p| p.component_ref.clone())
        .unwrap_or_default()
}

fn endpoints(
    snapshot: &BoardSnapshot,
    net_id: i32,
    destination: &str,
    via_size: f64,
) -> Result<Ends, String> {
    let net_pads: Vec<_> = snapshot
        .pads
        .iter()
        .filter(|p| p.net_id == net_id)
        .collect();
    if net_pads.is_empty() {
        return Err(format!("braid setup net {net_id} has no pads"));
    }
    let net_vias: Vec<_> = snapshot
        .vias
        .iter()
        .filter(|v| v.net_id == net_id)
        .collect();
    let segments: Vec<_> = snapshot
        .segments
        .iter()
        .filter(|s| s.net_id == net_id)
        .collect();
    let mut snap = Snapper::default();
    let mut endpoints_by_segment = Vec::new();
    for s in &segments {
        endpoints_by_segment.push((snap.key(s.a), snap.key(s.b)));
    }
    let node_count = snap.points.len();
    let mut adjacency = vec![BTreeSet::<usize>::new(); node_count + net_vias.len()];
    let via_node = |p: Point| -> Option<usize> {
        net_vias.iter().enumerate().find_map(|(i, v)| {
            (corridor::distance(p, v.point) <= v.diameter / 2.0).then_some(node_count + i)
        })
    };
    for &(a, b) in &endpoints_by_segment {
        if a == b {
            continue;
        }
        let na = via_node(snap.points[a]).unwrap_or(a);
        let nb = via_node(snap.points[b]).unwrap_or(b);
        adjacency[a].insert(nb);
        adjacency[b].insert(na);
    }
    let anchors: Vec<_> = net_pads
        .iter()
        .map(|p| (p.point, p.size[0].max(p.size[1]) / 2.0))
        .chain(net_vias.iter().map(|v| (v.point, v.diameter / 2.0)))
        .collect();
    let clear_of = |point: Point, anchors: &[(Point, f64)]| {
        anchors
            .iter()
            .all(|(p, r)| corridor::distance(point, *p) > 0.02_f64.max(*r))
    };
    let free_from = |include_vias: bool| -> Vec<Point> {
        (0..node_count)
            .filter(|&i| adjacency[i].len() == 1)
            .map(|i| snap.points[i])
            .filter(|p| {
                if include_vias {
                    clear_of(*p, &anchors)
                } else {
                    clear_of(
                        *p,
                        &net_pads
                            .iter()
                            .map(|p| (p.point, p.size[0].max(p.size[1]) / 2.0))
                            .collect::<Vec<_>>(),
                    )
                }
            })
            .collect()
    };
    let mut free = free_from(true);
    if free.is_empty() {
        free = free_from(false);
    }
    if free.is_empty() {
        return Err(format!("{net_id}: no free stub end"));
    }
    let net_refs = net_pads.as_slice();
    let mut at_dest: Vec<_> = free
        .iter()
        .copied()
        .filter(|p| owner(snapshot, net_id, *p, net_refs) == destination)
        .collect();
    let mut at_src: Vec<_> = free
        .iter()
        .copied()
        .filter(|p| !at_dest.contains(p))
        .collect();
    if at_src.is_empty() || at_dest.is_empty() {
        let pads_only_free = free_from(false);
        if at_dest.is_empty() {
            at_dest = pads_only_free
                .iter()
                .copied()
                .filter(|p| owner(snapshot, net_id, *p, net_refs) == destination)
                .collect();
        }
        if at_src.is_empty() {
            at_src = pads_only_free
                .iter()
                .copied()
                .filter(|p| {
                    !at_dest.contains(p) && owner(snapshot, net_id, *p, net_refs) != destination
                })
                .collect();
        }
    }
    if !at_dest.is_empty() && !at_src.is_empty() {
        let src = *first_max_by_score(&at_src, |p| {
            at_dest
                .iter()
                .map(|q| corridor::distance(*p, *q))
                .fold(0.0, f64::max)
        })
        .unwrap();
        let berth = *first_max_by_score(&at_dest, |p| corridor::distance(src, *p)).unwrap();
        return Ok(Ends {
            tooth: src,
            berth,
            destination: destination.into(),
        });
    }
    if !at_src.is_empty() {
        free = at_src;
    }
    if free.len() > 1 {
        free.sort_by(|a, b| {
            let nearest = |p: &Point| {
                anchors
                    .iter()
                    .map(|(q, _)| corridor::distance(*p, *q))
                    .fold(f64::INFINITY, f64::min)
            };
            nearest(b).total_cmp(&nearest(a))
        });
    }
    let tooth = free[0];
    let own = owner(snapshot, net_id, tooth, net_refs);
    let far: Vec<_> = net_pads
        .iter()
        .copied()
        .filter(|p| p.component_ref != own)
        .collect();
    let far = if far.is_empty() {
        net_refs.to_vec()
    } else {
        far
    };
    let mut target = *first_max_by_score(&far, |p| corridor::distance(tooth, p.point)).unwrap();
    for p in &far {
        if p.key != target.key && under_pad(target, p, via_size) {
            target = p;
            break;
        }
    }
    Ok(Ends {
        tooth,
        berth: target.point,
        destination: target.component_ref.clone(),
    })
}

fn under_pad(
    ball: &super::whole_context::ContextPad,
    pad: &super::whole_context::ContextPad,
    via_size: f64,
) -> bool {
    if pad.size[0] * pad.size[1] <= ball.size[0] * ball.size[1] {
        return false;
    }
    let a: BTreeSet<_> = ball.layers.iter().filter(|l| is_copper_layer(l)).collect();
    let b: BTreeSet<_> = pad.layers.iter().filter(|l| is_copper_layer(l)).collect();
    if a.is_empty() || b.is_empty() || !a.is_disjoint(&b) {
        return false;
    }
    let radius = via_size / 2.0;
    (pad.point[0] - ball.point[0]).abs() <= pad.size[0] / 2.0 + radius - 0.05
        && (pad.point[1] - ball.point[1]).abs() <= pad.size[1] / 2.0 + radius - 0.05
}

fn end_direction(
    snapshot: &BoardSnapshot,
    net_id: i32,
    point: Point,
    pads: &[&super::whole_context::ContextPad],
) -> Point {
    let mut snap = Snapper::default();
    let mut adjacency = BTreeMap::<usize, Vec<(usize, usize)>>::new();
    for (i, s) in snapshot
        .segments
        .iter()
        .enumerate()
        .filter(|(_, s)| s.net_id == net_id)
    {
        let a = snap.key(s.a);
        let b = snap.key(s.b);
        if a == b {
            continue;
        }
        adjacency.entry(a).or_default().push((b, i));
        adjacency.entry(b).or_default().push((a, i));
    }
    let start = snap.key(point);
    let mut seen = BTreeSet::from([start]);
    let mut frontier = vec![start];
    let mut order = Vec::<(f64, Point, Point)>::new();
    while !frontier.is_empty() {
        let mut next = Vec::new();
        for u in frontier {
            let inside = pads.iter().any(|p| point_box(snap.points[u], p, 0.02));
            if inside && u != start {
                continue;
            }
            for &(v, si) in adjacency.get(&u).into_iter().flatten() {
                if !seen.insert(v) {
                    continue;
                }
                let s = &snapshot.segments[si];
                order.push((corridor::distance(s.a, s.b), snap.points[u], snap.points[v]));
                next.push(v);
            }
        }
        frontier = next;
    }
    let Some((_, near, far)) = order
        .into_iter()
        .reduce(|best, item| if item.0 > best.0 { item } else { best })
    else {
        return [1.0, 0.0];
    };
    let v = corridor::sub(near, far);
    let length = corridor::length(v);
    if length > 1e-6 {
        [v[0] / length, v[1] / length]
    } else {
        [1.0, 0.0]
    }
}

fn layer_at_for_end(
    snapshot: &BoardSnapshot,
    net_id: i32,
    point: Point,
    pads: &[&super::whole_context::ContextPad],
) -> String {
    layer_at(snapshot, net_id, point, pads)
}
