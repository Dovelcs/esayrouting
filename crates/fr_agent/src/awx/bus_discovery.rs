//! Port of `awx/route_bus.py::find_buses` from AWX commit
//! caad45cd252f9a50f1fb4b3b15461bdf1b9457a1.
//!
//! Discovery consumes a `BoardSnapshot`, so callers must provide the declared
//! logical package mapping used to capture it. It deliberately does not infer
//! package membership from flattened board entities.

use super::{
    board_snapshot::BoardSnapshot,
    escape_moves::short_name,
    pairs::{self, Pair},
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const DEFAULT_MIN_BUS_NETS: usize = 8;
const DEFAULT_LEFT_PARTS: usize = 6;
const GRID_COORD_TOLERANCE_MM: f64 = 1e-3;

/// Inputs that affect `route_bus.find_buses` semantics.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct DiscoveryRequest {
    /// Minimum admitted signal nets shared by a component pair.
    pub min_bus_nets: usize,
}

impl Default for DiscoveryRequest {
    fn default() -> Self {
        Self {
            min_bus_nets: DEFAULT_MIN_BUS_NETS,
        }
    }
}

/// One candidate emitted by the original `find_buses`, including candidates
/// that the whole-route runner would leave to ordinary routing.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct DiscoveredBus {
    pub src: String,
    pub dest: String,
    pub nets: Vec<String>,
    pub taken: bool,
    pub why: Option<String>,
    /// Other small signal nets shared by the endpoints but excluded from this bus.
    pub left: BTreeMap<String, String>,
}

struct ArrayGrid;

/// Discover bus candidates with the same admission, array, ordering, and
/// explanatory fields as `route_bus.py --find`.
pub fn discover_buses(
    snapshot: &BoardSnapshot,
    request: &DiscoveryRequest,
    via_size_mm: f64,
) -> Result<Vec<DiscoveredBus>, String> {
    if request.min_bus_nets == 0 {
        return Err("bus discovery min_bus_nets must be positive".into());
    }
    if !via_size_mm.is_finite() || via_size_mm <= 0. {
        return Err("bus discovery via_size_mm must be finite and positive".into());
    }

    let mut pads_by_component: BTreeMap<&str, Vec<&super::whole_context::ContextPad>> =
        BTreeMap::new();
    let mut pads_by_net: BTreeMap<&str, Vec<&super::whole_context::ContextPad>> = BTreeMap::new();
    for pad in &snapshot.pads {
        pads_by_component
            .entry(&pad.component_ref)
            .or_default()
            .push(pad);
        if !pad.net.is_empty() && pad.net_id > 0 {
            pads_by_net.entry(&pad.net).or_default().push(pad);
        }
    }

    // The Python producer first counts all nets that join exactly two
    // >2-pad components. Admission is a separate, stricter step below.
    let many: BTreeSet<&str> = pads_by_component
        .iter()
        .filter_map(|(reference, pads)| (pads.len() > 2).then_some(*reference))
        .collect();
    let mut pair_counts: BTreeMap<(String, String), usize> = BTreeMap::new();
    for pads in pads_by_net.values() {
        let ends: BTreeSet<&str> = pads
            .iter()
            .map(|p| p.component_ref.as_str())
            .filter(|reference| many.contains(reference))
            .collect();
        if ends.len() == 2 {
            let mut ends = ends.into_iter();
            let a = ends.next().expect("two ends").to_owned();
            let b = ends.next().expect("two ends").to_owned();
            *pair_counts.entry((a, b)).or_default() += 1;
        }
    }

    let mut arrays = BTreeMap::<String, Option<ArrayGrid>>::new();
    let mut out = Vec::new();
    let zone_ids: BTreeSet<i32> = snapshot.zones.iter().map(|z| z.net_id).collect();

    for ((a, b), raw_count) in pair_counts {
        if raw_count < request.min_bus_nets {
            continue;
        }
        for reference in [&a, &b] {
            arrays.entry(reference.clone()).or_insert_with(|| {
                pads_by_component
                    .get(reference.as_str())
                    .and_then(|pads| analyze_array_grid(pads))
            });
        }
        let array_a = arrays.get(&a).is_some_and(Option::is_some);
        let array_b = arrays.get(&b).is_some_and(Option::is_some);
        if !array_a && !array_b {
            continue;
        }

        let admitted = admitted_nets(snapshot, &a, &b, via_size_mm);
        if admitted.len() < request.min_bus_nets {
            continue;
        }

        // route_bus sorts arrays first, then by physical pad count, then ref.
        let mut endpoints = [a.clone(), b.clone()];
        endpoints.sort_by(|left, right| {
            let left_array = arrays.get(left).is_some_and(Option::is_some);
            let right_array = arrays.get(right).is_some_and(Option::is_some);
            right_array
                .cmp(&left_array)
                .then_with(|| {
                    pads_by_component
                        .get(right.as_str())
                        .map_or(0, Vec::len)
                        .cmp(&pads_by_component.get(left.as_str()).map_or(0, Vec::len))
                })
                .then_with(|| left.cmp(right))
        });
        let src = endpoints[0].clone();
        let dest = endpoints[1].clone();
        let taken = array_a && array_b;
        let why = (!taken).then(|| {
            format!(
                "{dest} is no ball array: the whole route takes an array at both ends, a row part is left to the router"
            )
        });
        let mut nets: Vec<String> = admitted
            .iter()
            .map(|name| short_name(name).to_owned())
            .collect();
        nets.sort();
        let left = left_out(snapshot, &a, &b, &admitted, &zone_ids);
        out.push(DiscoveredBus {
            src,
            dest,
            nets,
            taken,
            why,
            left,
        });
    }

    out.sort_by(|a, b| {
        b.taken
            .cmp(&a.taken)
            .then_with(|| b.nets.len().cmp(&a.nets.len()))
            .then_with(|| a.src.cmp(&b.src))
            .then_with(|| a.dest.cmp(&b.dest))
    });
    Ok(out)
}

fn admitted_nets(snapshot: &BoardSnapshot, src: &str, dest: &str, via_size_mm: f64) -> Vec<String> {
    let all_short: BTreeSet<String> = snapshot
        .pads
        .iter()
        .filter(|p| !p.net.is_empty() && p.net_id > 0)
        .map(|p| short_name(&p.net).to_owned())
        .collect();
    let mut partner = BTreeMap::<String, String>::new();
    for Pair { p, n, .. } in pairs::pair_names(&all_short.iter().cloned().collect::<Vec<_>>(), true)
    {
        partner.insert(p.clone(), n.clone());
        partner.insert(n, p);
    }

    let mut admitted = Vec::new();
    for (net_name, pads) in &snapshot_nets(snapshot) {
        let references: BTreeSet<&str> = pads.iter().map(|p| p.component_ref.as_str()).collect();
        if references.len() < 2 || !references.contains(src) || !references.contains(dest) {
            continue;
        }
        if references.iter().any(|reference| {
            *reference != src
                && *reference != dest
                && snapshot
                    .pads
                    .iter()
                    .filter(|p| p.component_ref == *reference)
                    .count()
                    > 2
        }) {
            continue;
        }
        if pads.len() == 2 {
            admitted.push(net_name.clone());
            continue;
        }
        let mate = partner.get(short_name(net_name));
        let waypoint = |pad: &&super::whole_context::ContextPad| {
            if mate.is_none() {
                return false;
            }
            let component_pads: Vec<_> = snapshot
                .pads
                .iter()
                .filter(|other| other.component_ref == pad.component_ref)
                .collect();
            component_pads.len() == 2
                && component_pads
                    .iter()
                    .any(|other| other.key != pad.key && short_name(&other.net) == mate.unwrap())
        };
        let ends: Vec<_> = pads
            .iter()
            .filter(|pad| {
                !waypoint(pad)
                    && !pads.iter().any(|other| {
                        other.key != pad.key && pairs::under_pad(other, pad, via_size_mm)
                    })
            })
            .collect();
        if ends.len() == 2
            && ends
                .iter()
                .map(|p| p.component_ref.as_str())
                .collect::<BTreeSet<_>>()
                .len()
                == 2
        {
            admitted.push(net_name.clone());
        }
    }
    admitted.sort();
    admitted
}

fn snapshot_nets<'a>(
    snapshot: &'a BoardSnapshot,
) -> BTreeMap<String, Vec<&'a super::whole_context::ContextPad>> {
    let mut nets = BTreeMap::new();
    for pad in &snapshot.pads {
        if !pad.net.is_empty() && pad.net_id > 0 {
            nets.entry(pad.net.clone())
                .or_insert_with(Vec::new)
                .push(pad);
        }
    }
    nets
}

fn left_out(
    snapshot: &BoardSnapshot,
    a: &str,
    b: &str,
    admitted: &[String],
    zone_ids: &BTreeSet<i32>,
) -> BTreeMap<String, String> {
    let admitted: BTreeSet<&str> = admitted.iter().map(String::as_str).collect();
    let mut out = BTreeMap::new();
    for (net_name, pads) in snapshot_nets(snapshot) {
        let refs: BTreeSet<&str> = pads.iter().map(|p| p.component_ref.as_str()).collect();
        if !refs.contains(a)
            || !refs.contains(b)
            || admitted.contains(net_name.as_str())
            || zone_ids.contains(&pads[0].net_id)
            || refs.len() > DEFAULT_LEFT_PARTS
            || pads.len() > 2 * DEFAULT_LEFT_PARTS
        {
            continue;
        }
        let mut reasons = Vec::new();
        let extra: Vec<_> = refs
            .iter()
            .copied()
            .filter(|r| *r != a && *r != b)
            .collect();
        if !extra.is_empty() {
            reasons.push(format!("also on {}", extra.join(", ")));
        }
        for end in [a, b] {
            let count = pads.iter().filter(|p| p.component_ref == end).count();
            if count > 1 {
                reasons.push(format!("{count} pads on {end}"));
            }
        }
        if !reasons.is_empty() {
            out.insert(short_name(&net_name).to_owned(), reasons.join("; "));
        }
    }
    out
}

fn analyze_array_grid(pads: &[&super::whole_context::ContextPad]) -> Option<ArrayGrid> {
    let copper: Vec<_> = pads
        .iter()
        .copied()
        .filter(|p| !p.non_plated_hole)
        .collect();
    if copper.len() < 4 {
        return None;
    }
    let rows = cluster_positions(copper.iter().map(|p| p.point[1]).collect());
    let cols = cluster_positions(copper.iter().map(|p| p.point[0]).collect());
    if rows.len() < 2 || cols.len() < 2 {
        return None;
    }
    let pitch_x = dominant_pitch(&cols)?;
    let pitch_y = dominant_pitch(&rows)?;
    let min_x = cols[0] - pitch_x / 2.;
    let max_x = cols[cols.len() - 1] + pitch_x / 2.;
    let min_y = rows[0] - pitch_y / 2.;
    let max_y = rows[rows.len() - 1] + pitch_y / 2.;
    if rows.len() < 3 || cols.len() < 3 {
        return None;
    }
    let deep: BTreeSet<i32> = copper
        .iter()
        .filter(|p| {
            (p.point[0] - min_x).min(max_x - p.point[0]) > 2. * pitch_x
                && (p.point[1] - min_y).min(max_y - p.point[1]) > 2. * pitch_y
        })
        .map(|p| p.net_id)
        .collect();
    (deep.len() >= 2).then_some(ArrayGrid)
}

fn cluster_positions(mut values: Vec<f64>) -> Vec<f64> {
    values.sort_by(f64::total_cmp);
    let mut groups: Vec<Vec<f64>> = Vec::new();
    for value in values {
        if let Some(group) = groups.last_mut() {
            if value - group[group.len() - 1] <= GRID_COORD_TOLERANCE_MM {
                group.push(value);
                continue;
            }
        }
        groups.push(vec![value]);
    }
    groups
        .into_iter()
        .map(|group| group.iter().sum::<f64>() / group.len() as f64)
        .collect()
}

fn dominant_pitch(positions: &[f64]) -> Option<f64> {
    if positions.len() < 2 {
        return None;
    }
    // The Python implementation rounds to 0.01 mm and chooses the first
    // encountered pitch on ties.
    let mut counts = Vec::<(i64, usize)>::new();
    for diff in positions
        .windows(2)
        .map(|w| ((w[1] - w[0]) * 100.).round_ties_even() as i64)
    {
        if let Some((_, n)) = counts.iter_mut().find(|(pitch, _)| *pitch == diff) {
            *n += 1;
        } else {
            counts.push((diff, 1));
        }
    }
    let (pitch, count) = counts.into_iter().reduce(|best, candidate| {
        if candidate.1 > best.1 {
            candidate
        } else {
            best
        }
    })?;
    if (count as f64) < positions.len().saturating_sub(1) as f64 * 0.5 || pitch <= 0 {
        return None;
    }
    Some(pitch as f64 / 100.)
}
