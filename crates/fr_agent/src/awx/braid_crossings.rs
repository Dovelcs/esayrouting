//! Plan-only layer profiles and inter-corridor crossing via prices.

use super::braid_corridor::{CorridorPlan, LayerRun};
use std::collections::BTreeMap;

/// Return the actual tooth-to-berth layer runs recorded by a corridor plan.
pub fn layer_profile<'a>(plan: &'a CorridorPlan, net: &str) -> Option<&'a [LayerRun]> {
    plan.layer_profiles.get(net).map(Vec::as_slice)
}

/// Return the planned layer-change count for a swimmer. When no page crossing
/// required a stored swim count, the source charges only a tooth/berth layer
/// mismatch.
pub fn swim_changes(plan: &CorridorPlan, net: &str) -> Option<usize> {
    if plan.schedule.page_of(net).is_some() {
        return None;
    }
    if let Some(changes) = plan.swim_changes.get(net) {
        return Some(*changes);
    }
    let profile = layer_profile(plan, net)?;
    let (first, last) = (profile.first()?, profile.last()?);
    Some(usize::from(first.layer != last.layer))
}

/// Count a two-via dive for each later corridor lane that crosses any earlier
/// corridor lane on a layer occupied by that earlier lane. Multiple crossings
/// still cost one dive per net, as in braid.py::cross_corridor_vias.
pub fn cross_corridor_vias(plans: &[CorridorPlan]) -> Result<BTreeMap<String, usize>, String> {
    let mut out = BTreeMap::new();
    for (later_index, later) in plans.iter().enumerate() {
        for net in &later.members {
            if later.schedule.page_of(net).is_none() {
                continue;
            }
            let Some(polyline) = later.lanes.get(net) else {
                continue;
            };
            if polyline.len() < 2 {
                continue;
            }
            let profile = layer_profile(later, net)
                .filter(|runs| !runs.is_empty())
                .ok_or_else(|| format!("page lane {net} has no layer profile"))?;
            let mut needs_dive = false;
            'earlier: for earlier in &plans[..later_index] {
                for other in &earlier.members {
                    let Some(other_polyline) = earlier.lanes.get(other) else {
                        continue;
                    };
                    if other_polyline.len() < 2 {
                        continue;
                    }
                    for point in polyline_crossings(polyline, other_polyline)? {
                        let current_s = later.spine.project_pt(point)[0];
                        let earlier_s = earlier.spine.project_pt(point)[0];
                        let layer = layer_at(profile, current_s);
                        if earlier.schedule.page_of(other).is_none()
                            || earlier.allowed(other, earlier_s, layer)
                        {
                            needs_dive = true;
                            break 'earlier;
                        }
                    }
                }
            }
            if needs_dive {
                out.insert(net.clone(), 2);
            }
        }
    }
    Ok(out)
}

fn layer_at(profile: &[LayerRun], s: f64) -> &str {
    let mut layer = profile[0].layer.as_str();
    for run in profile {
        if run.s <= s + 1e-9 {
            layer = &run.layer;
        }
    }
    layer
}

/// Enumerate proper segment intersections only; endpoint touches and collinear
/// overlaps are not crossings in the source geometry.
fn polyline_crossings(a: &[[f64; 2]], b: &[[f64; 2]]) -> Result<Vec<[f64; 2]>, String> {
    if a.iter().chain(b).flatten().any(|v| !v.is_finite()) {
        return Err("nonfinite planned corridor lane point".into());
    }
    let mut out = Vec::new();
    for a_segment in a.windows(2) {
        for b_segment in b.windows(2) {
            let (p1, p2) = (a_segment[0], a_segment[1]);
            let (p3, p4) = (b_segment[0], b_segment[1]);
            let d1 = orient(p3, p4, p1);
            let d2 = orient(p3, p4, p2);
            let d3 = orient(p1, p2, p3);
            let d4 = orient(p1, p2, p4);
            if d1 * d2 < 0.0 && d3 * d4 < 0.0 {
                let t = d1 / (d1 - d2);
                out.push([p1[0] + t * (p2[0] - p1[0]), p1[1] + t * (p2[1] - p1[1])]);
            }
        }
    }
    Ok(out)
}

fn orient(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}
