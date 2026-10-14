//! Source-produced coherent river order for the route_bus K ladder.
//!
//! This is the native counterpart of `coherent_nets` + `make_bench.write_ladder`:
//! candidates without a free copper stub are discarded, remaining lanes are
//! clustered by their static-copper taut paths, and whole rivers are ordered
//! largest first with unclusterable lanes at the end.

use super::{
    board_snapshot::BoardSnapshot,
    corridor::{distance, Point},
    pairs, taut_paths,
    whole_context::{Ends, RoutingRules, WholeContext},
};
use std::collections::{BTreeMap, BTreeSet};

const SNAP_MM: f64 = 0.005;
const CLUSTER_WIDTH_MM: f64 = 1.5;
const CLUSTER_THRESHOLD: f64 = 0.55;
const RESAMPLE_STEP_MM: f64 = 0.25;

/// Generate the source's coherent K ladder from a post-source-fanout snapshot.
/// `candidate_nets` is the bus's source-ordered net set; names are the short
/// KiCad net names used by `BoardSnapshot::byname`.
pub fn produce_source_order(
    snapshot: &BoardSnapshot,
    source: &str,
    destination: &str,
    candidate_nets: &[String],
    rules: &RoutingRules,
) -> Result<Vec<String>, String> {
    rules.validate()?;
    if source == destination {
        return Err("lane order requires distinct source and destination".into());
    }

    let mut ids_by_short = BTreeMap::new();
    for (native_name, &id) in &snapshot.byname {
        let short = native_name.rsplit('/').next().unwrap_or(native_name);
        if ids_by_short
            .insert(short.to_owned(), id)
            .is_some_and(|old| old != id)
        {
            return Err(format!(
                "duplicate net basename {short} in lane-order snapshot"
            ));
        }
    }
    let mut eligible = Vec::new();
    let mut ends = BTreeMap::new();
    let mut byname = snapshot.byname.clone();
    for name in candidate_nets {
        let Some(&net_id) = snapshot.byname.get(name).or_else(|| ids_by_short.get(name)) else {
            continue;
        };
        let Some((tooth, _tooth_layer)) = source_stub(snapshot, net_id)? else {
            continue;
        };
        let source_pads: Vec<_> = snapshot
            .pads
            .iter()
            .filter(|p| p.net_id == net_id && p.component_ref == source)
            .collect();
        let destination_pads: Vec<_> = snapshot
            .pads
            .iter()
            .filter(|p| p.net_id == net_id && p.component_ref == destination)
            .collect();
        if source_pads.len() != 1 || destination_pads.len() != 1 {
            continue;
        }
        eligible.push(name.clone());
        ends.insert(
            name.clone(),
            Ends {
                tooth,
                berth: destination_pads[0].point,
                destination: destination.to_owned(),
            },
        );
        byname.insert(name.clone(), net_id);
    }
    if eligible.is_empty() {
        return Ok(Vec::new());
    }

    let pair_list = pairs::pair_names(&eligible, true);
    let (_, pair_map) = pairs::members(&eligible, &pair_list);
    let ctx = WholeContext {
        cfg: rules.clone(),
        layers: snapshot.layers.clone(),
        copper_layers: snapshot.layers.clone(),
        board_bounds: snapshot.board_bounds,
        board_outline: snapshot.board_outline.clone(),
        pads: snapshot.pads.clone(),
        source: source.to_owned(),
        destination: destination.to_owned(),
        lanes: eligible.clone(),
        byname,
        pairs: pair_map,
        ends,
        src_ref: BTreeMap::new(),
        tooth_layer: BTreeMap::new(),
        dest_layer: BTreeMap::new(),
        tooth_dir: BTreeMap::new(),
        stub_dir: BTreeMap::new(),
        pair_ends: BTreeMap::new(),
        pair_layers: BTreeMap::new(),
        paths: BTreeMap::new(),
        base_segments: snapshot.segments.clone(),
        base_vias: snapshot.vias.clone(),
        dest_cut: None,
        plane_nets: snapshot
            .zones
            .iter()
            .filter_map(|z| {
                snapshot
                    .byname
                    .iter()
                    .find(|(_, id)| **id == z.net_id)
                    .map(|(n, _)| n.clone())
            })
            .collect(),
        native_to_logical_components: snapshot
            .pads
            .iter()
            .map(|p| (p.key.clone(), p.component_ref.clone()))
            .collect(),
        chirality_mirrored: false,
    };

    let bundle_layer = eligible
        .iter()
        .filter_map(|name| {
            let id = *ctx.byname.get(name)?;
            source_stub(snapshot, id)
                .ok()
                .flatten()
                .map(|(_, layer)| layer)
        })
        .fold(BTreeMap::<String, usize>::new(), |mut counts, layer| {
            *counts.entry(layer).or_default() += 1;
            counts
        })
        .into_iter()
        .max_by(|a, b| a.1.cmp(&b.1).then_with(|| b.0.cmp(&a.0)))
        .map(|x| x.0)
        .unwrap_or_else(|| "F.Cu".into());
    let paths = taut_paths::paths(&ctx, &bundle_layer)?;
    let clustered = cluster(&eligible, &paths);
    let mut rivers: Vec<Vec<String>> = clustered.into_iter().filter(|c| c.len() > 1).collect();
    rivers.sort_by(|a, b| b.len().cmp(&a.len()));
    let placed: BTreeSet<String> = rivers.iter().flatten().cloned().collect();
    let mut order: Vec<String> = rivers.into_iter().flatten().collect();
    order.extend(eligible.into_iter().filter(|n| !placed.contains(n)));
    Ok(order)
}

/// Return a free stub endpoint and its copper layer, or `None` when the net
/// has no source stub. Endpoints are distinct graph neighbors after snapping;
/// via barrels count as one node, matching `braid.endpoints`.
fn source_stub(snapshot: &BoardSnapshot, net_id: i32) -> Result<Option<(Point, String)>, String> {
    let segs: Vec<_> = snapshot
        .segments
        .iter()
        .filter(|s| s.net_id == net_id)
        .collect();
    if segs.is_empty() {
        return Ok(None);
    }
    let mut adj = BTreeMap::<(i64, i64), BTreeSet<(i64, i64)>>::new();
    let mut coords = BTreeMap::<(i64, i64), Point>::new();
    let snap = |p: Point| {
        (
            (p[0] / SNAP_MM).round() as i64,
            (p[1] / SNAP_MM).round() as i64,
        )
    };
    for s in &segs {
        let (a, b) = (snap(s.a), snap(s.b));
        if a == b {
            continue;
        }
        coords.insert(a, s.a);
        coords.insert(b, s.b);
        adj.entry(a).or_default().insert(b);
        adj.entry(b).or_default().insert(a);
    }
    let vias: Vec<_> = snapshot
        .vias
        .iter()
        .filter(|v| v.net_id == net_id)
        .collect();
    let pads: Vec<_> = snapshot
        .pads
        .iter()
        .filter(|p| p.net_id == net_id)
        .collect();
    let mut free = Vec::new();
    for (node, neighbors) in &adj {
        if neighbors.len() != 1 {
            continue;
        }
        let p = coords[node];
        let in_via = vias.iter().any(|v| distance(p, v.point) <= v.diameter / 2.);
        if in_via {
            continue;
        }
        let in_pad = pads
            .iter()
            .any(|pad| distance(p, pad.point) <= (pad.size[0].max(pad.size[1]) / 2.).max(0.02));
        if !in_pad {
            free.push(p);
        }
    }
    if free.is_empty() {
        free = adj
            .iter()
            .filter(|(_, n)| n.len() == 1)
            .filter_map(|(k, _)| coords.get(k).copied())
            .filter(|p| {
                !pads.iter().any(|pad| {
                    distance(*p, pad.point) <= (pad.size[0].max(pad.size[1]) / 2.).max(0.02)
                })
            })
            .collect();
    }
    if free.is_empty() {
        return Ok(None);
    }
    free.sort_by(|a, b| {
        let nearest = |p: Point| {
            pads.iter()
                .map(|pad| distance(p, pad.point))
                .fold(f64::INFINITY, f64::min)
        };
        nearest(*b).total_cmp(&nearest(*a))
    });
    let tooth = free[0];
    let layer = segs
        .iter()
        .filter_map(|s| {
            let d = distance(s.a, tooth).min(distance(s.b, tooth));
            (d <= SNAP_MM * 2.).then_some((distance(s.a, s.b), s.layer.clone()))
        })
        .max_by(|a, b| a.0.total_cmp(&b.0))
        .map(|x| x.1)
        .unwrap_or_else(|| "F.Cu".into());
    Ok(Some((tooth, layer)))
}

fn resample(path: &[Point]) -> Vec<Point> {
    if path.is_empty() {
        return Vec::new();
    }
    let mut out = vec![path[0]];
    let mut carry = 0.;
    for pair in path.windows(2) {
        let len = distance(pair[0], pair[1]);
        if len < 1e-9 {
            continue;
        }
        let mut t = RESAMPLE_STEP_MM - carry;
        while t <= len {
            let f = t / len;
            out.push([
                pair[0][0] + (pair[1][0] - pair[0][0]) * f,
                pair[0][1] + (pair[1][1] - pair[0][1]) * f,
            ]);
            t += RESAMPLE_STEP_MM;
        }
        carry = (carry + len) % RESAMPLE_STEP_MM;
    }
    if let Some(last) = path.last() {
        out.push(*last);
    }
    out
}
fn togetherness(a: &[Point], b: &[Point]) -> f64 {
    let (short, other) = if a.len() <= b.len() { (a, b) } else { (b, a) };
    if short.is_empty() || other.is_empty() {
        return 0.;
    }
    let near = short
        .iter()
        .filter(|p| other.iter().any(|q| distance(**p, *q) <= CLUSTER_WIDTH_MM))
        .count();
    near as f64 / short.len() as f64
}
fn cluster(names: &[String], paths: &BTreeMap<String, Vec<Point>>) -> Vec<Vec<String>> {
    let mut sim = BTreeMap::<(String, String), f64>::new();
    for (i, a) in names.iter().enumerate() {
        for b in &names[i + 1..] {
            let value = match (paths.get(a), paths.get(b)) {
                (Some(a), Some(b)) => togetherness(&resample(a), &resample(b)),
                _ => 0.,
            };
            sim.insert((a.clone(), b.clone()), value);
            sim.insert((b.clone(), a.clone()), value);
        }
    }
    let mut groups: Vec<Vec<String>> = names.iter().cloned().map(|n| vec![n]).collect();
    while groups.len() > 1 {
        let (mut best, mut bi, mut bj) = (-1., 0, 1);
        for i in 0..groups.len() {
            for j in i + 1..groups.len() {
                let mut total = 0.0;
                for a in &groups[i] {
                    for b in &groups[j] {
                        total += sim.get(&(a.clone(), b.clone())).copied().unwrap_or(0.);
                    }
                }
                let score = total / (groups[i].len() * groups[j].len()) as f64;
                if score > best {
                    best = score;
                    bi = i;
                    bj = j;
                }
            }
        }
        if best < CLUSTER_THRESHOLD {
            break;
        }
        let right = groups.remove(bj);
        groups[bi].extend(right);
    }
    groups
}
