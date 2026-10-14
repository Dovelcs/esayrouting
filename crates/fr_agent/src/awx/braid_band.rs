//! Source-compatible hard per-layer corridor band from `Corridor.band_of`.
use super::{
    braid_connect::{RungBandConstraint, RungBandFactory, RungSpec},
    braid_corridor::{CorridorBandContext, LayerRun},
    corridor::Point,
};
use std::sync::Mutex;

const BIG: f64 = 1.0e6;
const BAND_STEP: f64 = 0.002;
const BREAK_EPS: f64 = 1.0e-7;
const PAGE_TUBE: f64 = 0.35;
const SWIM_TUBE: f64 = 1.2;
const SLOPE_W: f64 = 0.02;
const LEG_W: f64 = 0.5;
const LEG_O: f64 = 0.2;

#[derive(Clone, Debug)]
struct BandEdges {
    low: Vec<f64>,
    high: Vec<f64>,
}

/// Immutable hard point/layer mask produced by the source per-layer band
/// equations. Search adapters should query `contains` while expanding cells.
#[derive(Debug)]
pub struct CorridorBand {
    context: CorridorBandContext,
    net: String,
    slack: f64,
    open_layers: bool,
    samples: Vec<f64>,
    edges: Mutex<std::collections::BTreeMap<String, BandEdges>>,
    swimmer: bool,
    scheduled: bool,
    page_lane: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SourceRungBandFactory;

impl RungBandFactory for SourceRungBandFactory {
    fn band_for(
        &mut self,
        context: &CorridorBandContext,
        net: &str,
        spec: RungSpec,
    ) -> Result<Box<dyn RungBandConstraint>, String> {
        Ok(Box::new(band(context, net, spec.slack, spec.open_layers)?))
    }
}

/// Build the lane's exact source band. The sample interval covers the full
/// projected spine range; `Spine.project_pt` clamps all board points to this
/// interval. It includes the source's 2 um grid and all vertex/step/crossing
/// breakpoints with either-side epsilon samples.
pub fn band(
    context: &CorridorBandContext,
    net: &str,
    slack: f64,
    open_layers: bool,
) -> Result<CorridorBand, String> {
    if !slack.is_finite() || slack < 0.0 {
        return Err("braid band slack must be finite and nonnegative".into());
    }
    if !context.members.iter().any(|member| member == net) {
        return Err(format!("braid band net {net} is not a corridor member"));
    }
    let own = context
        .mid
        .get(net)
        .ok_or_else(|| format!("braid band has no midline for {net}"))?;
    if own.len() < 2 || own.iter().any(|p| p.iter().any(|v| !v.is_finite())) {
        return Err(format!("braid band has malformed midline for {net}"));
    }
    for member in &context.members {
        if context.mid.get(member).is_none_or(|p| p.len() < 2) {
            return Err(format!(
                "braid band has no valid dynamic neighbor line for {member}"
            ));
        }
    }
    let samples = band_samples(context, net)?;
    let scheduled = context.schedule.is_some();
    let swimmer = context
        .schedule
        .as_ref()
        .is_some_and(|s| s.page.get(net).and_then(Option::as_ref).is_none());
    let page_lane = scheduled && !swimmer;
    let mut edges = std::collections::BTreeMap::new();
    if !swimmer {
        edges.insert(
            "F.Cu".to_owned(),
            neighbor_edges(context, net, "F.Cu", &samples)?,
        );
        edges.insert(
            "B.Cu".to_owned(),
            neighbor_edges(context, net, "B.Cu", &samples)?,
        );
    }
    Ok(CorridorBand {
        context: context.clone(),
        net: net.to_owned(),
        slack,
        open_layers,
        samples,
        edges: Mutex::new(edges),
        swimmer,
        scheduled,
        page_lane,
    })
}

fn lane_point(line: &[Point], s: f64) -> f64 {
    if s <= line[0][0] {
        return line[0][1];
    }
    if s >= line[line.len() - 1][0] {
        return line[line.len() - 1][1];
    }
    let upper = line.partition_point(|p| p[0] <= s).min(line.len() - 1);
    let a = line[upper - 1];
    let b = line[upper];
    let t = (s - a[0]) / (b[0] - a[0]).max(1e-12);
    a[1] + t * (b[1] - a[1])
}

fn interval_layer_allowed(context: &CorridorBandContext, net: &str, s: f64, layer: &str) -> bool {
    let dive_extra = if context.pair_names.contains(net) {
        context.options.pair_dive_extra
    } else {
        0.0
    };
    if context.req.get(net).is_some_and(|req| {
        req.iter().any(|span| {
            span.layer != layer && s >= span.start + dive_extra && s <= span.end - dive_extra
        })
    }) {
        return false;
    }
    if layer == "B.Cu" {
        return context.back_windows.get(net).is_some_and(|windows| {
            windows
                .iter()
                .any(|(lo, hi)| s >= lo - dive_extra && s <= hi + dive_extra)
        });
    }
    true
}

fn profile_has_layer(profile: &[LayerRun], s: f64, layer: &str) -> bool {
    profile.iter().enumerate().any(|(i, run)| {
        run.layer == layer
            && s >= if i == 0 { f64::NEG_INFINITY } else { run.s }
            && s < profile.get(i + 1).map_or(f64::INFINITY, |next| next.s)
    })
}

fn planned_neighbor_allowed(
    context: &CorridorBandContext,
    net: &str,
    layer: &str,
    samples: &[f64],
) -> Vec<bool> {
    let plain: Vec<_> = samples
        .iter()
        .map(|&s| interval_layer_allowed(context, net, s, layer))
        .collect();
    let Some(schedule) = &context.schedule else {
        return plain;
    };
    if schedule.page.get(net).and_then(Option::as_ref).is_none() {
        return plain;
    }
    let Some(profile) = context.layer_profiles.get(net) else {
        return vec![false; samples.len()];
    };
    let mut on: Vec<_> = samples
        .iter()
        .map(|&s| profile_has_layer(profile, s, layer))
        .collect();
    let both: Vec<_> = samples
        .iter()
        .map(|&s| {
            interval_layer_allowed(context, net, s, "F.Cu")
                && interval_layer_allowed(context, net, s, "B.Cu")
        })
        .collect();
    if profile.len() > 1 && both.iter().any(|x| *x) {
        let changes: Vec<_> = profile.iter().skip(1).map(|run| run.s).collect();
        let mut i = 0;
        while i < both.len() {
            if !both[i] {
                i += 1;
                continue;
            }
            let start = i;
            while i + 1 < both.len() && both[i + 1] {
                i += 1;
            }
            let end = i;
            let lo = samples[start];
            let hi = samples[end];
            if changes.iter().any(|c| *c >= lo - 0.03 && *c <= hi + 0.03) {
                for slot in &mut on[start..=end] {
                    *slot = true;
                }
            }
            i += 1;
        }
    }
    plain
        .iter()
        .zip(on)
        .map(|(allowed, planned)| *allowed && planned)
        .collect()
}

fn band_samples(context: &CorridorBandContext, net: &str) -> Result<Vec<f64>, String> {
    let spine = &context.spine;
    let lo = 0.0;
    let hi = spine.length;
    let count = ((hi - lo) / BAND_STEP).ceil() as usize;
    let mut samples: Vec<f64> = (0..=count)
        .map(|i| lo + i as f64 * BAND_STEP)
        .filter(|s| *s <= hi + BAND_STEP)
        .collect();
    let own = context
        .mid
        .get(net)
        .ok_or_else(|| format!("missing band midline for {net}"))?;
    for member in &context.members {
        let line = context
            .mid
            .get(member)
            .ok_or_else(|| format!("missing band midline for {member}"))?;
        let mut breaks: Vec<f64> = line.iter().map(|p| p[0]).collect();
        if let Some(req) = context.req.get(member) {
            for interval in req {
                breaks.extend([interval.start, interval.end]);
            }
        }
        if let Some(windows) = context.back_windows.get(member) {
            for &(a, b) in windows {
                if a.abs() < 1e8 {
                    breaks.push(a);
                }
                if b.abs() < 1e8 {
                    breaks.push(b);
                }
            }
        }
        if member != net {
            let a = own[0][0].max(line[0][0]);
            let b = own[own.len() - 1][0].min(line[line.len() - 1][0]);
            if b > a {
                let mut union = vec![a, b];
                union.extend(own.iter().map(|p| p[0]).filter(|s| *s > a && *s < b));
                union.extend(line.iter().map(|p| p[0]).filter(|s| *s > a && *s < b));
                union.sort_by(f64::total_cmp);
                union.dedup_by(|x, y| *x == *y);
                let diffs: Vec<_> = union
                    .iter()
                    .map(|&s| lane_point(line, s) - lane_point(own, s))
                    .collect();
                for i in 0..union.len().saturating_sub(1) {
                    if diffs[i] == 0.0 {
                        breaks.push(union[i]);
                    } else if diffs[i] * diffs[i + 1] < 0.0 {
                        let t = diffs[i] / (diffs[i] - diffs[i + 1]);
                        breaks.push(union[i] + t * (union[i + 1] - union[i]));
                    }
                }
                if diffs.last().is_some_and(|d| *d == 0.0) {
                    breaks.push(*union.last().unwrap());
                }
            }
        }
        for s in breaks {
            if s >= lo - 1.0 && s <= hi + 1.0 {
                samples.extend([s - BREAK_EPS, s, s + BREAK_EPS]);
            }
        }
    }
    samples.retain(|s| s.is_finite());
    samples.sort_by(f64::total_cmp);
    samples.dedup_by(|a, b| *a == *b);
    if samples.len() < 2 {
        return Err("braid band has an empty sample interval".into());
    }
    Ok(samples)
}

fn neighbor_edges(
    context: &CorridorBandContext,
    net: &str,
    layer: &str,
    samples: &[f64],
) -> Result<BandEdges, String> {
    let own = context
        .mid
        .get(net)
        .ok_or_else(|| format!("missing band midline for {net}"))?;
    let own_offsets: Vec<_> = samples.iter().map(|&s| lane_point(own, s)).collect();
    let planned: std::collections::BTreeMap<String, Vec<bool>> = if context.options.branch {
        context
            .members
            .iter()
            .filter(|member| *member != net)
            .map(|member| {
                (
                    member.clone(),
                    planned_neighbor_allowed(context, member, layer, samples),
                )
            })
            .collect()
    } else {
        std::collections::BTreeMap::new()
    };
    let mut low = Vec::with_capacity(samples.len());
    let mut high = Vec::with_capacity(samples.len());
    for (i, &s) in samples.iter().enumerate() {
        let o = own_offsets[i];
        let mut prev = -BIG;
        let mut next = BIG;
        for other in &context.members {
            if other == net {
                continue;
            }
            let line = &context.mid[other];
            if s < line[0][0] - 1e-9 || s > line[line.len() - 1][0] + 1e-9 {
                continue;
            }
            let allowed = if context.options.branch {
                planned.get(other).is_some_and(|mask| mask[i])
            } else {
                interval_layer_allowed(context, other, s, layer)
            };
            if !allowed {
                continue;
            }
            let o_other = lane_point(line, s);
            if o_other < o {
                prev = prev.max(o_other);
            } else if o_other > o {
                next = next.min(o_other);
            }
        }
        low.push(if prev > -BIG / 2.0 {
            (prev + o) / 2.0 + context.options.half_sep
        } else {
            -BIG
        });
        high.push(if next < BIG / 2.0 {
            (next + o) / 2.0 - context.options.half_sep
        } else {
            BIG
        });
    }
    Ok(BandEdges { low, high })
}

fn interp(xs: &[f64], ys: &[f64], x: f64) -> f64 {
    if x <= xs[0] {
        return ys[0];
    }
    if x >= xs[xs.len() - 1] {
        return ys[ys.len() - 1];
    }
    let upper = xs.partition_point(|v| *v <= x).min(xs.len() - 1);
    let t = (x - xs[upper - 1]) / (xs[upper] - xs[upper - 1]).max(1e-12);
    ys[upper - 1] + t * (ys[upper] - ys[upper - 1])
}

impl RungBandConstraint for CorridorBand {
    fn contains(&self, point_mm: Point, layer: &str) -> bool {
        let Some(mid) = self.context.mid.get(&self.net) else {
            return false;
        };
        let [s, o] = self.context.spine.project_pt(point_mm);
        let present = s >= mid[0][0] - 1e-9 && s <= mid[mid.len() - 1][0] + 1e-9;
        let o_mid = lane_point(mid, s);
        let ok_layer =
            self.open_layers || interval_layer_allowed(&self.context, &self.net, s, layer);
        let mut allowed = present && ok_layer;
        if allowed {
            let (mut lo, mut hi) = if self.swimmer {
                (o_mid - SWIM_TUBE, o_mid + SWIM_TUBE)
            } else {
                let edge = {
                    let mut cached = match self.edges.lock() {
                        Ok(cache) => cache,
                        Err(_) => return false,
                    };
                    if !cached.contains_key(layer) {
                        let Ok(edge) =
                            neighbor_edges(&self.context, &self.net, layer, &self.samples)
                        else {
                            return false;
                        };
                        cached.insert(layer.to_owned(), edge);
                    }
                    let edge = cached.get(layer).unwrap().clone();
                    drop(cached);
                    edge
                };
                (
                    interp(&self.samples, &edge.low, s),
                    interp(&self.samples, &edge.high, s),
                )
            };
            if self.page_lane {
                lo = lo.min(o_mid - PAGE_TUBE);
                hi = hi.max(o_mid + PAGE_TUBE);
            }
            let slope_floor = if self.scheduled {
                let d_s: Vec<_> = mid
                    .windows(2)
                    .map(|w| (w[1][0] - w[0][0]).max(1e-9))
                    .collect();
                let segment = mid
                    .partition_point(|p| p[0] <= s)
                    .saturating_sub(1)
                    .min(d_s.len().saturating_sub(1));
                let slope = (mid[segment + 1][1] - mid[segment][1]).abs() / d_s[segment];
                0.03 + SLOPE_W * slope.min(30.0)
            } else {
                0.03
            };
            lo = lo.min(o_mid - slope_floor) - self.slack;
            hi = hi.max(o_mid + slope_floor) + self.slack;
            allowed = o >= lo && o <= hi;
        }
        if ok_layer {
            if let Some(legs) = self.context.legs.get(&self.net) {
                for &(s_leg, oa, ob) in legs {
                    if (s - s_leg).abs() <= LEG_W + self.slack
                        && o >= oa.min(ob) - LEG_O - self.slack
                        && o <= oa.max(ob) + LEG_O + self.slack
                    {
                        return true;
                    }
                }
            }
            if let Some(jogs) = self.context.jogs.get(&self.net) {
                for &(start, end) in jogs {
                    let (sa, oa) = (start[0], start[1]);
                    let (sb, _ob) = (end[0], end[1]);
                    if s >= sa.min(sb) - LEG_O && s <= sa.max(sb) + LEG_O && (o - oa).abs() <= LEG_O
                    {
                        return true;
                    }
                }
            }
        }
        allowed
    }
}
