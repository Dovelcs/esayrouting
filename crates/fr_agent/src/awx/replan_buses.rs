//! Native producer for the `replan.Board.buses()` / `planned_buses()` inputs.
//!
//! This is the B.buses route-specific corridor clustering. It is separate
//! from route-ladder grouping and preserves the braid's destination-arrival,
//! single-link, and straight-spine reach rules.

use super::{
    context_builder,
    corridor::{self, Point},
    escape_moves::{self, Move},
    joint_escape::JointPair,
    pairs,
    replan_state::ReplanState,
    taut_paths,
    whole_context::{Ends, WholeContext},
};
use std::collections::{BTreeMap, BTreeSet};

const CLUSTER_DISTANCE_MM: f64 = 6.0;
const ARRIVAL_DISTANCE_MM: f64 = 2.0;
const REACH_TOLERANCE_MM: f64 = 1.0;
const REACH_JOG_MM: f64 = 0.35;

/// Exact board-derived inputs consumed by `ReplanCostInput`.
#[derive(Clone, Debug)]
pub struct ReplanBusInputs {
    pub destination_choice: BTreeMap<String, Move>,
    pub launch: BTreeMap<String, Point>,
    pub buses: Vec<Vec<String>>,
    pub source_box: [f64; 4],
    pub destination_box: [f64; 4],
    pub chi: i32,
}

/// Reproduce `B.buses()` -> `fanout_from_plan.planned_buses(st, choice)`.
/// All geometry and chirality come from the current native probe state and
/// its validated full plan sidecar; unavailable state is an error or an
/// omitted choice exactly as in the source, never replaced with a default.
pub fn produce_replan_bus_inputs(state: &ReplanState) -> Result<ReplanBusInputs, String> {
    let names = state.names.clone();
    if names.is_empty() {
        return Err("replan planned_buses has no current named ends".into());
    }
    if let Some(name) = names.iter().find(|name| !state.ends.contains_key(*name)) {
        return Err(format!(
            "replan planned_buses missing current ends for {name}"
        ));
    }
    let mut choice = BTreeMap::new();
    for name in &names {
        if let Some(movement) = state.ends.get(name).and_then(|end| end.dst_move.clone()) {
            if movement.net != escape_moves::short_name(name) {
                return Err(format!(
                    "replan planned_buses current destination move net mismatch for {name}"
                ));
            }
            choice.insert(name.clone(), movement);
        }
    }
    let launch: BTreeMap<String, Point> = choice
        .keys()
        .map(|name| {
            state
                .ends
                .get(name)
                .map(|end| (name.clone(), end.tooth))
                .ok_or_else(|| format!("replan planned_buses missing source tooth for {name}"))
        })
        .collect::<Result<_, _>>()?;

    let chi = state.probe.chi;
    let sidecar_chi = state
        .world
        .plan_sidecar
        .get("chi")
        .and_then(serde_json::Value::as_i64)
        .map(|value| value as i32)
        .filter(|value| *value == 1 || *value == -1)
        .ok_or("replan planned_buses requires recorded source chirality")?;
    if chi != sidecar_chi || (chi != 1 && chi != -1) {
        return Err("replan planned_buses chirality disagrees with the full plan sidecar".into());
    }

    let source_box = state.probe.source_menus.grid.bbox;
    let destination_box = state.probe.destination_menus.grid.bbox;
    validate_box("source", source_box)?;
    validate_box("destination", destination_box)?;

    let buses = if choice.is_empty() {
        Vec::new()
    } else {
        produce_corridors(state, &names, &choice, &launch)?
    };
    Ok(ReplanBusInputs {
        destination_choice: choice,
        launch,
        buses,
        source_box,
        destination_box,
        chi,
    })
}

fn validate_box(name: &str, bounds: [f64; 4]) -> Result<(), String> {
    if bounds.iter().any(|v| !v.is_finite()) || bounds[0] > bounds[2] || bounds[1] > bounds[3] {
        return Err(format!(
            "replan planned_buses has invalid {name} array bounds"
        ));
    }
    Ok(())
}

fn produce_corridors(
    state: &ReplanState,
    all_names: &[String],
    choice: &BTreeMap<String, Move>,
    launch: &BTreeMap<String, Point>,
) -> Result<Vec<Vec<String>>, String> {
    let probe = &state.probe;
    let pairs: Vec<pairs::Pair> = probe
        .destination_input
        .pairs
        .iter()
        .map(|pair: &JointPair| pairs::Pair {
            base: pair.base.clone(),
            p: pair.p.clone(),
            n: pair.n.clone(),
        })
        .collect();
    let all_net_set: BTreeSet<String> = all_names.iter().cloned().collect();
    let mut built = context_builder::build(
        &state.snapshot,
        &probe.source_part,
        &probe.destination_part,
        &all_net_set,
        &pairs,
        &probe.source_plan,
        &probe.destination_plan,
        &probe.source_menus,
        &probe.destination_menus,
        &probe.rules,
        &probe.source_input.routing_layers,
    )?;
    let ctx = &mut built.context;
    ctx.lanes = all_names.to_vec();
    for name in all_names {
        let end = state
            .ends
            .get(name)
            .ok_or_else(|| format!("replan planned_buses missing ends for {name}"))?;
        let berth = choice
            .get(name)
            .map(|movement| movement.exit_pt)
            .unwrap_or(end.exit);
        ctx.ends.insert(
            name.clone(),
            Ends {
                tooth: end.tooth,
                berth,
                destination: probe.destination_part.clone(),
            },
        );
    }

    // braid.bundle_layer_of(st['tooth0']): choose B only when strictly more
    // than half of all named source teeth are physically on B.Cu.
    let mut b_teeth = 0usize;
    for name in all_names {
        let id = *state
            .snapshot
            .byname
            .get(name)
            .ok_or_else(|| format!("replan planned_buses missing native net {name}"))?;
        let tooth = state
            .ends
            .get(name)
            .ok_or_else(|| format!("replan planned_buses missing tooth {name}"))?
            .tooth;
        let layer = state
            .snapshot
            .segments
            .iter()
            .find(|segment| {
                segment.net_id == id
                    && ((segment.a[0] - tooth[0]).abs() + (segment.a[1] - tooth[1]).abs() < 0.005
                        || (segment.b[0] - tooth[0]).abs() + (segment.b[1] - tooth[1]).abs()
                            < 0.005)
            })
            .map(|segment| segment.layer.as_str())
            .unwrap_or("F.Cu");
        b_teeth += usize::from(layer == "B.Cu");
    }
    let bundle_layer = if 2 * b_teeth > all_names.len() {
        "B.Cu"
    } else {
        "F.Cu"
    };

    // planned_buses asks taut_paths for every current destination choice,
    // while plan_state's obstacle callback excludes all named run copper.
    let all_paths = taut_paths::paths(ctx, bundle_layer)?;
    let names: Vec<String> = all_names
        .iter()
        .filter(|name| choice.contains_key(*name))
        .cloned()
        .collect();
    let paths: BTreeMap<String, Vec<Point>> = names
        .iter()
        .map(|name| {
            all_paths
                .get(name)
                .cloned()
                .map(|path| (name.clone(), path))
                .ok_or_else(|| format!("replan planned_buses has no taut path for {name}"))
        })
        .collect::<Result<_, _>>()?;
    let teeth: BTreeMap<String, Point> = names
        .iter()
        .map(|name| {
            launch
                .get(name)
                .copied()
                .map(|point| (name.clone(), point))
                .ok_or_else(|| format!("replan planned_buses has no launch point for {name}"))
        })
        .collect::<Result<_, _>>()?;
    let stubs: BTreeMap<String, Point> = names
        .iter()
        .map(|name| (name.clone(), choice[name].exit_pt))
        .collect();

    let id0 = *ctx
        .byname
        .get(&names[0])
        .ok_or_else(|| format!("replan planned_buses missing native net {}", names[0]))?;
    let kids: BTreeSet<i32> = all_names
        .iter()
        .filter_map(|name| ctx.byname.get(name).copied())
        .collect();
    let obstacles = taut_paths::build_obstacles(ctx, id0, &kids, bundle_layer, None)?;
    let centres = component_centres(ctx)?;
    let source_center = *centres
        .get(&probe.source_part)
        .ok_or("replan planned_buses missing source array centre")?;
    let destination_center = *centres
        .get(&probe.destination_part)
        .ok_or("replan planned_buses missing destination array centre")?;
    let centre_by_net: BTreeMap<String, Point> = names
        .iter()
        .map(|name| (name.clone(), destination_center))
        .collect();
    let source_centre_by_net: BTreeMap<String, Point> = names
        .iter()
        .map(|name| (name.clone(), source_center))
        .collect();
    cluster_corridors(
        &names,
        &paths,
        &teeth,
        &stubs,
        &|a, b| obstacles.seg_clear(a, b),
        &centre_by_net,
        &source_centre_by_net,
    )
}

fn component_centres(ctx: &WholeContext) -> Result<BTreeMap<String, Point>, String> {
    let mut sums = BTreeMap::<String, (f64, f64, usize)>::new();
    for pad in &ctx.pads {
        let entry = sums.entry(pad.component_ref.clone()).or_insert((0., 0., 0));
        entry.0 += pad.point[0];
        entry.1 += pad.point[1];
        entry.2 += 1;
    }
    Ok(sums
        .into_iter()
        .filter_map(|(name, (x, y, n))| (n > 0).then_some((name, [x / n as f64, y / n as f64])))
        .collect())
}

fn cluster_corridors<F>(
    names: &[String],
    paths: &BTreeMap<String, Vec<Point>>,
    teeth: &BTreeMap<String, Point>,
    stubs: &BTreeMap<String, Point>,
    pad_clear: &F,
    centres: &BTreeMap<String, Point>,
    src_centres: &BTreeMap<String, Point>,
) -> Result<Vec<Vec<String>>, String>
where
    F: Fn(Point, Point) -> bool,
{
    if names.is_empty() {
        return Ok(Vec::new());
    }
    let mut parent: Vec<usize> = (0..names.len()).collect();
    let mut index = BTreeMap::new();
    for (i, name) in names.iter().enumerate() {
        index.insert(name.as_str(), i);
    }
    let mut arrivals = BTreeMap::new();
    for name in names {
        let path = paths
            .get(name)
            .ok_or_else(|| format!("corridor clustering missing taut path {name}"))?;
        let before = point_before_end(path, ARRIVAL_DISTANCE_MM);
        let centre = *centres
            .get(name)
            .ok_or_else(|| format!("corridor clustering missing destination centre for {name}"))?;
        arrivals.insert(name.clone(), unit_or_zero(sub(before, centre)));
    }
    for (i, a) in names.iter().enumerate() {
        for b in &names[i + 1..] {
            let dist = corridor::distance(
                *stubs
                    .get(a)
                    .ok_or_else(|| format!("corridor clustering missing stub {a}"))?,
                *stubs
                    .get(b)
                    .ok_or_else(|| format!("corridor clustering missing stub {b}"))?,
            );
            if dist > CLUSTER_DISTANCE_MM {
                continue;
            }
            let angle = angle_degrees(arrivals[a], arrivals[b]);
            if angle <= 60.0 {
                union(&mut parent, index[a.as_str()], index[b.as_str()]);
            }
        }
    }
    let mut groups = Vec::<Vec<String>>::new();
    let mut group_index = BTreeMap::<usize, usize>::new();
    for (i, name) in names.iter().enumerate() {
        let root = find(&mut parent, i);
        let at = *group_index.entry(root).or_insert_with(|| {
            groups.push(Vec::new());
            groups.len() - 1
        });
        groups[at].push(name.clone());
    }
    sort_groups(&mut groups);

    let mut todo = groups;
    let mut out = Vec::new();
    while let Some(group) = take_first(&mut todo) {
        if group.len() == 1 {
            out.push(group);
            continue;
        }
        let (p0, p1, d) = spine(&group, teeth, stubs)?;
        let mut keep = Vec::new();
        let mut split = Vec::new();
        for name in &group {
            let tooth = teeth[name];
            let stub = stubs[name];
            let t_e = dot(sub(stub, p1), d);
            let t_0 = dot(sub(tooth, p0), d);
            let mut okay = true;
            if t_e > REACH_TOLERANCE_MM {
                let back_pt = sub(stub, mul(d, t_e));
                let centre = centres.get(name).copied();
                okay = run_clear(stub, back_pt, d, centre, pad_clear)
                    || wrap_clear(stub, d, centre, pad_clear);
            }
            if okay && t_0 < -REACH_TOLERANCE_MM {
                let fwd_pt = sub(tooth, mul(d, t_0));
                okay = run_clear(tooth, fwd_pt, d, src_centres.get(name).copied(), pad_clear);
            }
            if okay {
                keep.push(name.clone());
            } else {
                split.push(name.clone());
            }
        }
        if !split.is_empty() && !keep.is_empty() {
            out.push(keep);
            let subgroups =
                cluster_corridors(&split, paths, teeth, stubs, pad_clear, centres, src_centres)?;
            todo.extend(subgroups);
            sort_groups(&mut todo);
        } else {
            out.push(group);
        }
    }
    sort_groups(&mut out);
    Ok(out)
}

fn sort_groups(groups: &mut [Vec<String>]) {
    groups.sort_by(|a, b| b.len().cmp(&a.len()));
}

fn take_first<T>(items: &mut Vec<T>) -> Option<T> {
    if items.is_empty() {
        None
    } else {
        Some(items.remove(0))
    }
}

fn point_before_end(path: &[Point], distance: f64) -> Point {
    let Some(_) = path.last() else {
        return [0., 0.];
    };
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

fn unit_or_zero(v: Point) -> Point {
    let length = corridor::length(v);
    if length <= 1e-12 {
        [1., 0.]
    } else {
        [v[0] / length, v[1] / length]
    }
}

fn angle_degrees(a: Point, b: Point) -> f64 {
    let dot = (a[0] * b[0] + a[1] * b[1]).clamp(-1., 1.);
    dot.acos().to_degrees()
}

fn find(parent: &mut [usize], mut at: usize) -> usize {
    while parent[at] != at {
        parent[at] = parent[parent[at]];
        at = parent[at];
    }
    at
}

fn union(parent: &mut [usize], a: usize, b: usize) {
    let ra = find(parent, a);
    let rb = find(parent, b);
    if ra != rb {
        parent[ra] = rb;
    }
}

fn spine(
    group: &[String],
    teeth: &BTreeMap<String, Point>,
    stubs: &BTreeMap<String, Point>,
) -> Result<(Point, Point, Point), String> {
    let mut t = [0., 0.];
    let mut s = [0., 0.];
    for name in group {
        let tooth = teeth
            .get(name)
            .ok_or_else(|| format!("corridor spine missing tooth {name}"))?;
        let stub = stubs
            .get(name)
            .ok_or_else(|| format!("corridor spine missing stub {name}"))?;
        t = [t[0] + tooth[0], t[1] + tooth[1]];
        s = [s[0] + stub[0], s[1] + stub[1]];
    }
    let count = group.len() as f64;
    let p0 = [t[0] / count, t[1] / count];
    let p1 = [s[0] / count, s[1] / count];
    let delta = sub(p1, p0);
    let length = corridor::length(delta);
    let d = if length <= 1e-12 {
        [1., 0.]
    } else {
        [delta[0] / length, delta[1] / length]
    };
    Ok((p0, p1, d))
}

fn run_clear<F: Fn(Point, Point) -> bool>(
    end: Point,
    far: Point,
    direction: Point,
    centre: Option<Point>,
    pad_clear: &F,
) -> bool {
    if pad_clear(end, far) {
        return true;
    }
    let Some(centre) = centre else {
        return false;
    };
    let mut normal = sub(end, centre);
    let along = dot(normal, direction);
    normal = sub(normal, mul(direction, along));
    let length = corridor::length(normal);
    if length < 1e-9 {
        return false;
    }
    normal = mul(normal, 1. / length);
    for k in 1..=3 {
        let offset = REACH_JOG_MM * k as f64;
        let shifted_end = [end[0] + offset * normal[0], end[1] + offset * normal[1]];
        let shifted_far = [far[0] + offset * normal[0], far[1] + offset * normal[1]];
        if pad_clear(end, shifted_end) && pad_clear(shifted_end, shifted_far) {
            return true;
        }
    }
    false
}

fn wrap_clear<F: Fn(Point, Point) -> bool>(
    end: Point,
    direction: Point,
    centre: Option<Point>,
    pad_clear: &F,
) -> bool {
    let Some(centre) = centre else {
        return false;
    };
    let radial = sub(end, centre);
    let along = dot(radial, direction);
    let perpendicular = sub(radial, mul(direction, along));
    let length = corridor::length(perpendicular);
    let sides = if length > 1e-9 {
        [
            mul(perpendicular, 1. / length),
            mul(perpendicular, -1. / length),
        ]
    } else {
        [[-direction[1], direction[0]], [direction[1], -direction[0]]]
    };
    for k in 1..=3 {
        let offset = REACH_JOG_MM * k as f64;
        let forward = [
            end[0] + offset * direction[0],
            end[1] + offset * direction[1],
        ];
        if !pad_clear(end, forward) {
            continue;
        }
        for side in sides {
            let far = [
                forward[0] + 2. * CLUSTER_DISTANCE_MM * side[0],
                forward[1] + 2. * CLUSTER_DISTANCE_MM * side[1],
            ];
            if pad_clear(forward, far) {
                return true;
            }
        }
    }
    false
}

fn dot(a: Point, b: Point) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn sub(a: Point, b: Point) -> Point {
    [a[0] - b[0], a[1] - b[1]]
}

fn mul(a: Point, k: f64) -> Point {
    [a[0] * k, a[1] * k]
}
