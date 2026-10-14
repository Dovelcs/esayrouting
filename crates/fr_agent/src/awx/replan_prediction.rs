//! Recompute `replan.Board.model` from the current fanout board and its braid plan.
//!
//! This is intentionally separate from candidate cost and from the actual
//! whole-route result: it asks the plan-only braid producer for the current
//! schedule, then applies `plan_ends.vias_from_pages` per net.
use super::{
    braid_plan::{self, BraidPlanInput, BraidPlanSeed, BraidSetupOptions},
    braid_setup::{resolve_corridor_options, BraidCorridorOverrides},
    corridor::Point,
    escape_moves::{self, Move},
    replan_cost::{ReplanNetPrediction, ReplanPrediction},
    replan_state::ReplanState,
    whole_context::Ends,
};
use std::collections::{BTreeMap, BTreeSet, HashMap};

/// Reproduce `Board.model(named)`: filter the current board choices to the
/// requested names, run the real plan-only braid schedule on those ends, and
/// return the per-net via prediction with residuals disabled.
pub fn evaluate_model(
    state: &ReplanState,
    named: &BTreeSet<String>,
) -> Result<ReplanPrediction, String> {
    if named.is_empty() {
        return Err("B.model requires at least one named net".into());
    }
    if state.probe.plan_judge == "ends" {
        return Err(
            "PLAN_JUDGE=ends uses whole_ends.judge and has no per-net braid prediction".into(),
        );
    }
    if !matches!(state.probe.plan_judge.as_str(), "" | "count" | "flat") {
        return Err(format!(
            "unsupported source PLAN_JUDGE mode {:?}",
            state.probe.plan_judge
        ));
    }

    let mut choices = BTreeMap::<String, Move>::new();
    let names: Vec<_> = state
        .names
        .iter()
        .filter(|name| named.contains(*name))
        .filter_map(|name| {
            state.ends.get(name).and_then(|end| {
                end.dst_move
                    .clone()
                    .map(|movement| (name.clone(), movement))
            })
        })
        .map(|(name, movement)| {
            choices.insert(name.clone(), movement);
            name
        })
        .collect();
    if names.is_empty() {
        return Err(format!(
            "B.model has no destination choices among requested nets: {}",
            named.iter().cloned().collect::<Vec<_>>().join(", ")
        ));
    }
    let missing: Vec<_> = named
        .iter()
        .filter(|name| !choices.contains_key(*name))
        .cloned()
        .collect();
    if !missing.is_empty() {
        return Err(format!(
            "B.model cannot price requested nets without current destination choices: {}",
            missing.join(", ")
        ));
    }

    let mut ends = BTreeMap::new();
    let mut tooth_layer = BTreeMap::new();
    let mut dest_layer = BTreeMap::new();
    let mut tooth_dir = BTreeMap::new();
    let mut stub_dir = BTreeMap::new();
    let mut berth_sig = BTreeMap::new();
    for name in &names {
        let end = state
            .ends
            .get(name)
            .ok_or_else(|| format!("B.model current ends missing for {name}"))?;
        let movement = choices
            .get(name)
            .ok_or_else(|| format!("B.model destination choice missing for {name}"))?;
        ends.insert(
            name.clone(),
            Ends {
                tooth: end.tooth,
                berth: end.exit,
                destination: state.probe.destination_part.clone(),
            },
        );
        if end.tooth_layer.is_empty() || end.dest_layer.is_empty() {
            return Err(format!("B.model layer data missing for {name}"));
        }
        tooth_layer.insert(name.clone(), end.tooth_layer.clone());
        dest_layer.insert(name.clone(), end.dest_layer.clone());
        tooth_dir.insert(
            name.clone(),
            end_direction(&state.snapshot, name, end.tooth)?,
        );
        stub_dir.insert(name.clone(), direction_vector(movement)?);
        berth_sig.insert(name.clone(), super::whole_ends::move_signature(movement)?);
    }

    // `braid_plan_of` supplies only ends/layers/directions/signatures; it does
    // not carry the fanout-loop pages-first marker or ends-model cut.
    let seed = BraidPlanSeed {
        ends,
        tooth_layer,
        dest_layer,
        tooth_dir,
        stub_dir,
        chi: state.probe.chi,
        berth_sig,
    };
    let options = BraidSetupOptions { pairs: false };
    let corridor_overrides = corridor_overrides_from_state(state)?;
    let corridor_options = resolve_corridor_options(&state.probe.rules, &corridor_overrides)?;
    let schedule = braid_plan::plan_braid(&BraidPlanInput {
        snapshot: &state.snapshot,
        source: &state.probe.source_part,
        destination: &state.probe.destination_part,
        names: &names,
        seed: &seed,
        options: &options,
        rules: &state.probe.rules,
        corridor_options: &corridor_options,
        // Board.model -> judge_by_braid rebuilds braid_plan_of, whose seed
        // does not carry pages_first. braid.setup therefore resets exact pages
        // to false even when the separate B.cost model enables it.
        exact_pages: false,
        swimmer_page_price: state.probe.swimmer_page_price,
    })?;

    let mut per_net = BTreeMap::new();
    for name in &names {
        let movement = choices
            .get(name)
            .ok_or_else(|| format!("B.model destination choice missing for {name}"))?;
        let end = state
            .ends
            .get(name)
            .ok_or_else(|| format!("B.model current ends missing for {name}"))?;
        let source_move = end
            .src_move
            .as_ref()
            .ok_or_else(|| format!("B.model source tooth via count missing for {name}"))?;
        let lane = schedule
            .get(name)
            .ok_or_else(|| format!("plan_braid schedule omitted requested net {name}"))?;
        let mut vias = source_move.vias as f64 + lane.cross_vias as f64;
        match &lane.page {
            None => {
                let swimmer = if state.probe.plan_judge == "count" {
                    lane.swim_changes
                        .map(|changes| changes as f64)
                        .unwrap_or(state.probe.swim_vias)
                } else {
                    state.probe.swim_vias
                };
                vias += swimmer + movement.vias as f64;
            }
            Some(page) => {
                if let Some(changes) = lane.changes {
                    vias += changes as f64 + movement.vias as f64;
                } else {
                    let arrival = if let Some(leg) = &lane.exit_leg_layer {
                        (leg != page) as u8 as f64 + (&movement.layer != leg) as u8 as f64
                    } else {
                        (&movement.layer != page) as u8 as f64
                    };
                    vias +=
                        (&end.tooth_layer != page) as u8 as f64 + arrival + movement.vias as f64;
                }
            }
        }
        if !vias.is_finite() {
            return Err(format!(
                "B.model produced a non-finite prediction for {name}"
            ));
        }
        per_net.insert(
            name.clone(),
            ReplanNetPrediction {
                page: lane.page.clone(),
                predicted_vias: vias,
            },
        );
    }
    let total_vias: f64 = per_net.values().map(|value| value.predicted_vias).sum();
    if !total_vias.is_finite() {
        return Err("B.model produced a non-finite total prediction".into());
    }
    Ok(ReplanPrediction {
        per_net,
        total_vias,
    })
}

fn direction_vector(movement: &Move) -> Result<Point, String> {
    let (x, y) = movement.direction.delta();
    let vector = [x as f64, y as f64];
    if vector == [0.0, 0.0] {
        return Err(format!(
            "B.model destination direction is invalid for {}",
            movement.net
        ));
    }
    Ok(vector)
}

fn corridor_overrides_from_state(state: &ReplanState) -> Result<BraidCorridorOverrides, String> {
    let env = &state.environment_overrides;
    let float = |key: &str, empty: Option<f64>| -> Result<Option<f64>, String> {
        env.get(key)
            .map(|value| {
                if value.is_empty() {
                    return Ok(empty);
                }
                value
                    .parse::<f64>()
                    .map_err(|_| {
                        format!("invalid {key} value in native replan environment: {value:?}")
                    })
                    .map(Some)
            })
            .transpose()
            .map(Option::flatten)
            .and_then(|value| {
                if value.is_some_and(|value| !value.is_finite()) {
                    Err(format!("non-finite {key} in native replan environment"))
                } else {
                    Ok(value)
                }
            })
    };
    let integer = |key: &str, empty: Option<i64>| -> Result<Option<i64>, String> {
        env.get(key)
            .map(|value| {
                if value.is_empty() {
                    return Ok(empty);
                }
                value
                    .parse::<i64>()
                    .map_err(|_| {
                        format!("invalid {key} value in native replan environment: {value:?}")
                    })
                    .map(Some)
            })
            .transpose()
            .map(Option::flatten)
    };
    let pair_gap = float("BRAID_PAIR_GAP", Some(0.0))?.filter(|value| *value != 0.0);
    let jog_via = float("BRAID_JOG_VIA", Some(1.0))?;
    let block_push = integer("BRAID_BLOCK_PUSH", Some(0))?
        .map(|value| u8::try_from(value).map_err(|_| "BRAID_BLOCK_PUSH must fit 0..255"))
        .transpose()?;
    let pages_siders = integer("PLAN_PAGES_SIDERS", Some(0))?
        .map(|value| u8::try_from(value).map_err(|_| "PLAN_PAGES_SIDERS must fit 0..255"))
        .transpose()?;
    let pairs = integer("BRAID_PAIRS", Some(0))?.map(|value| value != 0);
    let branch = env.get("BRAID_BRANCH").map(|value| value != "0");
    Ok(BraidCorridorOverrides {
        track_width: None,
        spec_clearance: None,
        via_size: None,
        grid: None,
        lane_pitch: None,
        exit_pitch: None,
        pairs,
        block_push,
        pair_gap,
        pair_dive_extra_pitches: float("BRAID_PAIR_DIVE_EXTRA", Some(0.0))?,
        jog_via,
        // Source: float(awx_settings.get('BRAID_APPROACH_RESERVE', '1.0') or 0).
        approach_reserve: float("BRAID_APPROACH_RESERVE", Some(0.0))?,
        // No BRAID_* override exists for this field; resolver applies the
        // source default track_width + braid.CLEAR + 0.05.
        end_keep: None,
        pages_siders,
        slope_pitch: None,
        branch,
    })
}

/// Port of `braid._end_dir`: walk same-net copper out from the free tooth,
/// stop at pad copper, then use the longest stub segment pointed toward the
/// free end. Nodes within 6 microns coalesce as in the source snapper.
fn end_direction(
    snapshot: &super::board_snapshot::BoardSnapshot,
    net: &str,
    start: Point,
) -> Result<Point, String> {
    let net_id = snapshot
        .byname
        .get(net)
        .copied()
        .ok_or_else(|| format!("B.model native net missing for {net}"))?;
    let mut points = Vec::<Point>::new();
    let mut buckets = HashMap::<(i64, i64), Vec<usize>>::new();
    let mut adjacency = Vec::<Vec<(usize, usize)>>::new();
    let intern = |point: Point,
                  points: &mut Vec<Point>,
                  buckets: &mut HashMap<(i64, i64), Vec<usize>>,
                  adjacency: &mut Vec<Vec<(usize, usize)>>| {
        let q = 0.012;
        let bucket = ((point[0] / q).floor() as i64, (point[1] / q).floor() as i64);
        for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(found) = buckets.get(&(bucket.0 + dx, bucket.1 + dy)) {
                    for &index in found {
                        let existing = points[index];
                        if (existing[0] - point[0]).abs() <= 0.006
                            && (existing[1] - point[1]).abs() <= 0.006
                        {
                            return index;
                        }
                    }
                }
            }
        }
        let index = points.len();
        points.push([
            escape_moves::rounded(point[0], 3),
            escape_moves::rounded(point[1], 3),
        ]);
        adjacency.push(Vec::new());
        buckets.entry(bucket).or_default().push(index);
        index
    };
    let mut edge_lengths = Vec::new();
    for segment in snapshot
        .segments
        .iter()
        .filter(|segment| segment.net_id == net_id)
    {
        let a = intern(segment.a, &mut points, &mut buckets, &mut adjacency);
        let b = intern(segment.b, &mut points, &mut buckets, &mut adjacency);
        let edge = edge_lengths.len();
        edge_lengths.push(escape_moves::distance(segment.a, segment.b));
        adjacency[a].push((b, edge));
        adjacency[b].push((a, edge));
    }
    let start = intern(start, &mut points, &mut buckets, &mut adjacency);
    let pads: Vec<_> = snapshot
        .pads
        .iter()
        .filter(|pad| pad.net_id == net_id)
        .collect();
    let inside_pad = |point: Point| {
        pads.iter().any(|pad| {
            (point[0] - pad.point[0]).abs() <= pad.size[0] / 2.0 + 0.02
                && (point[1] - pad.point[1]).abs() <= pad.size[1] / 2.0 + 0.02
        })
    };
    let mut seen = BTreeSet::from([start]);
    let mut frontier = vec![start];
    let mut walked = Vec::<(usize, usize, usize)>::new();
    while !frontier.is_empty() {
        let mut next = Vec::new();
        for node in frontier {
            if node != start && inside_pad(points[node]) {
                continue;
            }
            for &(other, edge) in &adjacency[node] {
                if seen.insert(other) {
                    walked.push((edge, node, other));
                    next.push(other);
                }
            }
        }
        frontier = next;
    }
    let Some(mut longest) = walked.first() else {
        return Ok([1.0, 0.0]);
    };
    for edge in walked.iter().skip(1) {
        if edge_lengths[edge.0] > edge_lengths[longest.0] {
            longest = edge;
        }
    }
    let (_, near, far) = longest;
    let vector = [
        points[*near][0] - points[*far][0],
        points[*near][1] - points[*far][1],
    ];
    let length = vector[0].hypot(vector[1]);
    if length <= 1e-6 {
        Ok([1.0, 0.0])
    } else {
        Ok([vector[0] / length, vector[1] / length])
    }
}
