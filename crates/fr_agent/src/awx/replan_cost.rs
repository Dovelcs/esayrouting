//! Exact `replan.py` candidate-cost adapter for the fixed AWX source.
//!
//! The source's B.cost delegates to plan_ends.judged_cost. Its braid schedule
//! groups must be supplied in the order produced by B.buses()/planned_buses;
//! this module never substitutes fanout groups or a whole-route objective.
use super::{
    braid_schedule::{build_schedule, ScheduleInput},
    corridor::Point,
    escape_moves::Move,
};
use std::collections::{BTreeMap, BTreeSet};

const VIA_MM: f64 = 7.5;
const BOX_PAD: f64 = 0.3;
const BOX_EPS: f64 = 0.05;

#[derive(Clone, Debug)]
pub struct ReplanCostInput {
    /// Full net names in the route's stable order.
    pub names: Vec<String>,
    /// B.choice: current destination choices measured on the fanout board.
    pub destination_choice: BTreeMap<String, Move>,
    /// Current source tooth / launch coordinates.
    pub launch: BTreeMap<String, Point>,
    /// Current source escape layer and via count, from plan_state.
    pub tooth_layer: BTreeMap<String, String>,
    pub tooth_vias: BTreeMap<String, u32>,
    /// Exact B.buses() output: planned_buses groups and their member order.
    pub buses: Vec<Vec<String>>,
    pub source_box: [f64; 4],
    pub destination_box: [f64; 4],
    /// Pair chirality from plan_state, exactly +1 or -1.
    pub chi: i32,
    /// Residual evidence from real vias - B.model prediction. `None` must
    /// explicitly mean the original source leaves this net unpriced (for
    /// example, refused); the model never silently fills missing data.
    pub residual: BTreeMap<String, Option<f64>>,
    /// Source schedule.py's BRAID_EXACT_PAGES setting.
    pub exact_pages: bool,
    /// Source prices.PAGES (schedule exact-page objective).
    pub swimmer_page_price: f64,
    /// Source prices.JUDGE/SWIM_VIAS (cost assigned to a schedule swimmer).
    pub swim_vias: f64,
    /// B.model output from the current board's real braid verdict.
    pub current_prediction: BTreeMap<String, ReplanNetPrediction>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ReplanNetPrediction {
    pub page: Option<String>,
    pub predicted_vias: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ReplanPrediction {
    pub per_net: BTreeMap<String, ReplanNetPrediction>,
    pub total_vias: f64,
}

#[derive(Clone, Debug)]
pub enum ReplanCandidate {
    Destination { net: String, movement: Move },
    Source { net: String, movement: Move },
}

impl ReplanCandidate {
    pub fn net(&self) -> &str {
        match self {
            Self::Destination { net, .. } | Self::Source { net, .. } => net,
        }
    }
}

#[derive(Clone, Debug)]
pub struct ReplanCostModel {
    input: ReplanCostInput,
}

impl ReplanCostModel {
    pub fn new(input: ReplanCostInput) -> Result<Self, String> {
        if input.chi != 1 && input.chi != -1 {
            return Err("replan cost requires source chirality +1 or -1".into());
        }
        for (label, b) in [
            ("source", input.source_box),
            ("destination", input.destination_box),
        ] {
            if b.iter().any(|v| !v.is_finite()) || b[0] > b[2] || b[1] > b[3] {
                return Err(format!("replan cost has invalid {label} array bounds"));
            }
        }
        if input.buses.is_empty() && !input.destination_choice.is_empty() {
            return Err("replan cost requires exact planned_buses groups".into());
        }
        if !input.swimmer_page_price.is_finite()
            || input.swimmer_page_price < 0.0
            || !input.swim_vias.is_finite()
            || input.swim_vias < 0.0
        {
            return Err("replan schedule prices must be finite and nonnegative".into());
        }
        let names: BTreeSet<_> = input.names.iter().collect();
        if names.len() != input.names.len() {
            return Err("replan cost names contain duplicates".into());
        }
        for (net, value) in &input.residual {
            if !names.contains(net) || value.is_some_and(|v| !v.is_finite()) {
                return Err(format!("invalid replan residual evidence for {net}"));
            }
        }
        for (net, pred) in &input.current_prediction {
            if !names.contains(net) || !pred.predicted_vias.is_finite() {
                return Err(format!("invalid current braid prediction for {net}"));
            }
        }
        Ok(Self { input })
    }

    /// Cost equivalent to `Board.cost` / `plan_ends.judged_cost` for one
    /// proposed destination or source move, holding every other end fixed.
    pub fn cost(&self, candidate: &ReplanCandidate) -> Result<f64, String> {
        let net = candidate.net();
        if !self.input.names.iter().any(|n| n == net) {
            return Err(format!(
                "replan candidate net {net} is outside the named board"
            ));
        }
        let mut choice = self.input.destination_choice.clone();
        let mut launch = self.input.launch.clone();
        let mut tooth_layer = self.input.tooth_layer.clone();
        let mut tooth_vias = self.input.tooth_vias.clone();
        match candidate {
            ReplanCandidate::Destination { net, movement } => {
                if super::escape_moves::short_name(&movement.net)
                    != super::escape_moves::short_name(net)
                {
                    return Err("destination candidate net does not match its move".into());
                }
                choice.insert(net.clone(), movement.clone());
            }
            ReplanCandidate::Source { net, movement } => {
                if super::escape_moves::short_name(&movement.net)
                    != super::escape_moves::short_name(net)
                {
                    return Err("source candidate net does not match its move".into());
                }
                if !choice.contains_key(net) {
                    return Err(format!("source candidate {net} has no destination choice"));
                }
                launch.insert(net.clone(), movement.exit_pt);
                tooth_layer.insert(net.clone(), movement.layer.clone());
                tooth_vias.insert(net.clone(), movement.vias);
            }
        }
        for n in choice.keys() {
            if !launch.contains_key(n)
                || !tooth_layer.contains_key(n)
                || !tooth_vias.contains_key(n)
            {
                return Err(format!(
                    "replan cost state is incomplete for selected net {n}"
                ));
            }
        }
        let (choice, launch, tooth_layer, source_box, destination_box) =
            self.canonical_frame(choice, launch, tooth_layer);
        let per_net = self.schedule_costs(&choice, &launch, &tooth_layer)?;
        let mut total: f64 = per_net.values().sum();
        for (n, value) in &per_net {
            match self.input.residual.get(n) {
                Some(Some(residual)) => total += *residual,
                Some(None) => {}
                None => return Err(format!("replan residual status missing for {n}")),
            }
            if !value.is_finite() {
                return Err(format!("replan schedule price is non-finite for {n}"));
            }
        }
        for n in choice.keys() {
            total += *tooth_vias
                .get(n)
                .ok_or_else(|| format!("source via count missing for {n}"))?
                as f64;
        }
        total += ride_mm(&choice, &launch, destination_box, source_box)? / VIA_MM;
        if !total.is_finite() {
            return Err("replan candidate cost is non-finite".into());
        }
        Ok(total)
    }

    /// Return the authoritative per-net prediction and page supplied by the
    /// current board's B.model/`judge_by_braid` evaluation (residual disabled).
    pub fn predict_current(&self) -> Result<ReplanPrediction, String> {
        let mut per_net = BTreeMap::new();
        for net in self.input.destination_choice.keys() {
            let p = self.input.current_prediction.get(net).ok_or_else(|| {
                format!("current braid model prediction missing for selected net {net}")
            })?;
            per_net.insert(net.clone(), p.clone());
        }
        let total_vias = per_net.values().map(|p| p.predicted_vias).sum();
        Ok(ReplanPrediction {
            per_net,
            total_vias,
        })
    }

    fn canonical_frame(
        &self,
        mut choice: BTreeMap<String, Move>,
        mut launch: BTreeMap<String, Point>,
        mut tooth_layer: BTreeMap<String, String>,
    ) -> (
        BTreeMap<String, Move>,
        BTreeMap<String, Point>,
        BTreeMap<String, String>,
        [f64; 4],
        [f64; 4],
    ) {
        let (mut src_box, mut dst_box) = (self.input.source_box, self.input.destination_box);
        if self.input.chi > 0 {
            return (choice, launch, tooth_layer, src_box, dst_box);
        }
        // plan_ends.PairFrame mirrors about the rounded midpoint of current
        // launches and destination-array bounds; faces and F/B layers swap.
        let mut ys: Vec<f64> = launch.values().map(|p| p[1]).collect();
        ys.extend([dst_box[1], dst_box[3]]);
        let cy = (((ys.iter().copied().fold(f64::INFINITY, f64::min)
            + ys.iter().copied().fold(f64::NEG_INFINITY, f64::max))
            / 2.0)
            * 2000.0)
            .round()
            / 2000.0;
        for point in launch.values_mut() {
            point[1] = 2.0 * cy - point[1];
        }
        for movement in choice.values_mut() {
            movement.exit_pt[1] = 2.0 * cy - movement.exit_pt[1];
            movement.direction = match movement.direction {
                super::escape_moves::Direction::Up => super::escape_moves::Direction::Down,
                super::escape_moves::Direction::Down => super::escape_moves::Direction::Up,
                direction => direction,
            };
            movement.layer = flip_layer(&movement.layer);
        }
        for layer in tooth_layer.values_mut() {
            *layer = flip_layer(layer);
        }
        src_box = mirror_box(src_box, cy);
        dst_box = mirror_box(dst_box, cy);
        (choice, launch, tooth_layer, src_box, dst_box)
    }

    fn schedule_costs(
        &self,
        choice: &BTreeMap<String, Move>,
        launch: &BTreeMap<String, Point>,
        tooth_layer: &BTreeMap<String, String>,
    ) -> Result<BTreeMap<String, f64>, String> {
        let mut out = BTreeMap::new();
        for group in &self.input.buses {
            let grp: Vec<_> = group
                .iter()
                .filter(|n| choice.contains_key(*n))
                .cloned()
                .collect();
            if grp.is_empty() {
                continue;
            }
            if grp.len() == 1 {
                let n = &grp[0];
                let layer = tooth_layer
                    .get(n)
                    .ok_or_else(|| format!("tooth layer missing for {n}"))?;
                let berth = choice
                    .get(n)
                    .ok_or_else(|| format!("berth move missing for {n}"))?;
                out.insert(
                    n.clone(),
                    bool_cost(berth.layer != *layer) + berth.vias as f64,
                );
                continue;
            }
            let target_order = self.target_order(&grp, choice, launch)?;
            let axis = self.axis(&grp, choice, launch)?;
            let mut lo = grp.clone();
            lo.sort_by(|a, b| {
                dot(*launch.get(a).unwrap(), axis).total_cmp(&dot(*launch.get(b).unwrap(), axis))
            });
            let group_destination_layers: BTreeMap<_, _> = grp
                .iter()
                .filter_map(|n| choice.get(n).map(|m| (n.clone(), m.layer.clone())))
                .collect();
            let schedule = build_schedule(ScheduleInput {
                launch: lo,
                target: target_order,
                tooth_layer: tooth_layer.clone(),
                dest_layer: group_destination_layers,
                exact_pages: self.input.exact_pages,
                swimmer_page_price: self.input.swimmer_page_price,
            })?;
            for n in &grp {
                let layer = tooth_layer
                    .get(n)
                    .ok_or_else(|| format!("tooth layer missing for {n}"))?;
                let berth = choice
                    .get(n)
                    .ok_or_else(|| format!("berth move missing for {n}"))?;
                let p = schedule
                    .page
                    .get(n)
                    .ok_or_else(|| format!("schedule omitted page result for {n}"))?;
                let vias = if let Some(page) = p {
                    bool_cost(*layer != *page) + bool_cost(berth.layer != *page) + berth.vias as f64
                } else {
                    self.input.swim_vias + berth.vias as f64
                };
                out.insert(n.clone(), vias);
            }
        }
        Ok(out)
    }

    fn axis(
        &self,
        group: &[String],
        choice: &BTreeMap<String, Move>,
        launch: &BTreeMap<String, Point>,
    ) -> Result<Point, String> {
        let mut dx = 0.;
        let mut dy = 0.;
        for n in group {
            let a = launch
                .get(n)
                .ok_or_else(|| format!("launch point missing for {n}"))?;
            let b = choice
                .get(n)
                .ok_or_else(|| format!("destination move missing for {n}"))?
                .exit_pt;
            dx += b[0] - a[0];
            dy += b[1] - a[1];
        }
        let h = dx.hypot(dy).max(1.0);
        Ok([-dy / h, dx / h])
    }

    fn target_order(
        &self,
        group: &[String],
        choice: &BTreeMap<String, Move>,
        launch: &BTreeMap<String, Point>,
    ) -> Result<Vec<String>, String> {
        let axis = self.axis(group, choice, launch)?;
        let mut lo = group.to_vec();
        lo.sort_by(|a, b| {
            dot(*launch.get(a).unwrap(), axis).total_cmp(&dot(*launch.get(b).unwrap(), axis))
        });
        let lidx: BTreeMap<_, _> = lo
            .iter()
            .enumerate()
            .map(|(i, n)| (n.as_str(), i))
            .collect();
        let mut tgt = group.to_vec();
        tgt.sort_by(|a, b| {
            let ka = round_six(dot(choice[a].exit_pt, axis));
            let kb = round_six(dot(choice[b].exit_pt, axis));
            ka.total_cmp(&kb)
                .then_with(|| lidx[a.as_str()].cmp(&lidx[b.as_str()]))
        });
        Ok(tgt)
    }
}

fn round_six(v: f64) -> f64 {
    (v * 1_000_000.0).round() / 1_000_000.0
}
fn dot(a: Point, b: Point) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn bool_cost(value: bool) -> f64 {
    if value {
        1.0
    } else {
        0.0
    }
}
fn flip_layer(layer: &str) -> String {
    if let Some(rest) = layer.strip_prefix("F.") {
        format!("B.{rest}")
    } else if let Some(rest) = layer.strip_prefix("B.") {
        format!("F.{rest}")
    } else {
        layer.to_string()
    }
}
fn mirror_box(bounds: [f64; 4], cy: f64) -> [f64; 4] {
    [
        bounds[0],
        2.0 * cy - bounds[3],
        bounds[2],
        2.0 * cy - bounds[1],
    ]
}

fn ride_mm(
    choice: &BTreeMap<String, Move>,
    launch: &BTreeMap<String, Point>,
    destination_box: [f64; 4],
    source_box: [f64; 4],
) -> Result<f64, String> {
    let mut total = 0.;
    for (n, m) in choice {
        let a = *launch
            .get(n)
            .ok_or_else(|| format!("launch point missing for {n}"))?;
        let b = m.exit_pt;
        total += around_box(a, b, destination_box);
        total += around_box(a, b, source_box) - distance(a, b);
    }
    Ok(total)
}

fn around_box(a: Point, b: Point, bounds: [f64; 4]) -> f64 {
    let padded = [
        bounds[0] - BOX_PAD,
        bounds[1] - BOX_PAD,
        bounds[2] + BOX_PAD,
        bounds[3] + BOX_PAD,
    ];
    let inner = [
        padded[0] + BOX_EPS,
        padded[1] + BOX_EPS,
        padded[2] - BOX_EPS,
        padded[3] - BOX_EPS,
    ];
    if !segment_hits_box(a, b, inner) {
        return distance(a, b);
    }
    let corners = [
        [padded[0], padded[1]],
        [padded[2], padded[1]],
        [padded[0], padded[3]],
        [padded[2], padded[3]],
    ];
    let mut best = f64::INFINITY;
    for c1 in corners {
        for c2 in corners {
            if segment_hits_box(a, c1, inner)
                || segment_hits_box(c2, b, inner)
                || (c1 != c2 && segment_hits_box(c1, c2, inner))
            {
                continue;
            }
            best = best.min(distance(a, c1) + distance(c1, c2) + distance(c2, b));
        }
    }
    if best.is_finite() {
        best
    } else {
        distance(a, b)
    }
}

fn segment_hits_box(a: Point, b: Point, r: [f64; 4]) -> bool {
    let (mut lo, mut hi) = (0.0_f64, 1.0_f64);
    for (p, q) in [
        (-(b[0] - a[0]), a[0] - r[0]),
        (b[0] - a[0], r[2] - a[0]),
        (-(b[1] - a[1]), a[1] - r[1]),
        (b[1] - a[1], r[3] - a[1]),
    ] {
        if p.abs() < 1e-15 {
            if q < 0. {
                return false;
            }
        } else {
            let t = q / p;
            if p < 0. {
                lo = lo.max(t);
            } else {
                hi = hi.min(t);
            }
            if lo > hi {
                return false;
            }
        }
    }
    true
}

fn distance(a: Point, b: Point) -> f64 {
    (b[0] - a[0]).hypot(b[1] - a[1])
}
