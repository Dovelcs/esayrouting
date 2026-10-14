//! AWX whole_route.py layer_cut, own_spans and typed history/cut accumulation.
use super::{
    corridor::{distance, Point},
    numeric,
    whole_context::LaneGeometry,
    whole_types::{GeometryCuts, HistoryAudit, HotFinding, LayerCut, ViaCut},
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct IslandBox {
    pub bounds: [f64; 4],
    pub layers: BTreeSet<usize>,
}
pub fn layer_cut(
    lane: &str,
    island: &str,
    b: &IslandBox,
    layer: usize,
    nlayers: usize,
) -> Option<LayerCut> {
    if !b.layers.contains(&layer) || b.layers.len() >= nlayers {
        return None;
    }
    Some(LayerCut {
        lane: lane.into(),
        island: island.into(),
        layer: if nlayers == 2 { 1 - layer } else { 0 },
        blocked: if nlayers > 2 {
            Some(b.layers.iter().map(|&v| v as i64).collect())
        } else {
            None
        },
        bounds: b.bounds,
        span: None,
    })
}
pub fn dense_uxy(points: &[[f64; 3]], step: f64) -> Vec<[f64; 3]> {
    if points.is_empty() {
        return vec![];
    }
    let mut out = Vec::new();
    for w in points.windows(2) {
        let m = (distance([w[0][1], w[0][2]], [w[1][1], w[1][2]]) / step)
            .ceil()
            .max(1.) as usize;
        for i in 0..m {
            let t = i as f64 / m as f64;
            out.push([
                w[0][0] + (w[1][0] - w[0][0]) * t,
                w[0][1] + (w[1][1] - w[0][1]) * t,
                w[0][2] + (w[1][2] - w[0][2]) * t,
            ]);
        }
    }
    out.push(*points.last().unwrap());
    out
}
fn at_u(rows: &[[f64; 3]], u: f64) -> Option<Point> {
    for w in rows.windows(2) {
        if w[0][0] <= u && u <= w[1][0] {
            let t = if w[1][0] > w[0][0] {
                (u - w[0][0]) / (w[1][0] - w[0][0])
            } else {
                0.
            };
            return Some([
                w[0][1] + (w[1][1] - w[0][1]) * t,
                w[0][2] + (w[1][2] - w[0][2]) * t,
            ]);
        }
    }
    None
}
pub fn own_spans(
    cuts: &GeometryCuts,
    lanes: &BTreeMap<String, LaneGeometry>,
    nlayers: usize,
    track: f64,
    clearance: f64,
    grid: f64,
) -> Result<GeometryCuts, String> {
    if nlayers <= 2 {
        return Ok(cuts.clone());
    }
    let reach = track / 2. + clearance + grid;
    let mut out = cuts.clone();
    let rows: BTreeMap<_, _> = lanes
        .iter()
        .map(|(n, g)| (n.clone(), dense_uxy(&g.uxy, grid / 2.)))
        .collect();
    for c in &mut out.vcuts {
        let Some(r) = rows.get(&c.lane).filter(|r| !r.is_empty()) else {
            continue;
        };
        let Some(site) = at_u(r, c.u) else { continue };
        let i = r
            .iter()
            .enumerate()
            .min_by(|a, b| (a.1[0] - c.u).abs().total_cmp(&(b.1[0] - c.u).abs()))
            .unwrap()
            .0;
        let mut lo = i;
        let mut hi = i;
        let near = |j: usize| distance([r[j][1], r[j][2]], site) <= c.w;
        while lo > 0 && near(lo - 1) {
            lo -= 1;
        }
        while hi + 1 < r.len() && near(hi + 1) {
            hi += 1;
        }
        let a = r[lo][0].max(c.u - c.w);
        let b = r[hi][0].min(c.u + c.w);
        c.u = numeric::round_digits((a + b) / 2., 4)?;
        c.w = numeric::round_digits((b - a) / 2., 4)?;
    }
    for c in &mut out.lcuts {
        let Some(r) = rows.get(&c.lane).filter(|r| !r.is_empty()) else {
            continue;
        };
        let Some(g) = lanes.get(&c.lane) else {
            continue;
        };
        let mut us = Vec::new();
        for p in super::whole_frame::dense(&g.xy, grid / 2.) {
            let b = c.bounds;
            if b[0] - reach <= p[0]
                && p[0] <= b[2] + reach
                && b[1] - reach <= p[1]
                && p[1] <= b[3] + reach
            {
                let near = r
                    .iter()
                    .min_by(|a, b| distance([a[1], a[2]], p).total_cmp(&distance([b[1], b[2]], p)))
                    .unwrap();
                if distance([near[1], near[2]], p) <= 2. * reach {
                    us.push(near[0]);
                }
            }
        }
        if !us.is_empty() {
            c.span = Some([
                numeric::round_digits(us.iter().copied().fold(f64::INFINITY, f64::min), 4)?,
                numeric::round_digits(us.iter().copied().fold(f64::NEG_INFINITY, f64::max), 4)?,
            ]);
        }
    }
    Ok(out)
}
pub fn snap_layer_cuts(
    lane: &str,
    point: Point,
    layer: usize,
    islands: &BTreeMap<String, IslandBox>,
    nlayers: usize,
    reach: f64,
) -> Vec<LayerCut> {
    let distance_box = |b: &IslandBox| {
        let r = b.bounds;
        (r[0] - point[0])
            .max(0.)
            .max(point[0] - r[2])
            .hypot((r[1] - point[1]).max(0.).max(point[1] - r[3]))
    };
    islands
        .iter()
        .filter(|(_, b)| {
            b.layers.len() == 1 && b.layers.contains(&layer) && distance_box(b) <= reach
        })
        .min_by(|a, b| {
            distance_box(a.1)
                .total_cmp(&distance_box(b.1))
                .then(a.0.cmp(b.0))
        })
        .and_then(|(id, b)| layer_cut(lane, id, b, layer, nlayers))
        .into_iter()
        .collect()
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct FeedbackHistory {
    pub cuts: Vec<GeometryCuts>,
    pub history: Vec<HistoryAudit>,
    pub flips: BTreeSet<(String, String)>,
    pub rounds: usize,
    pub best_score: Option<usize>,
    pub stalled: usize,
    pub endpoint_prices: BTreeMap<(String, usize), i64>,
}
impl FeedbackHistory {
    pub fn add_round(
        &mut self,
        cuts: GeometryCuts,
        hot: Vec<HotFinding>,
        new_flips: BTreeSet<(String, String)>,
        score: usize,
        patience: usize,
    ) -> Result<bool, String> {
        let fresh = new_flips.iter().any(|x| !self.flips.contains(x));
        self.flips.extend(new_flips);
        self.cuts.push(cuts);
        if !hot.is_empty() {
            self.history.push(HistoryAudit { hot });
        }
        self.rounds += 1;
        if self.best_score.is_none_or(|best| score < best) {
            self.best_score = Some(score);
            self.stalled = 0;
        } else if !fresh {
            self.stalled += 1;
        }
        Ok(self.stalled < patience)
    }
    pub fn via_cut(lane: &str, u: f64, w: f64, built: bool) -> ViaCut {
        ViaCut {
            lane: lane.into(),
            u,
            w,
            built,
        }
    }
}
