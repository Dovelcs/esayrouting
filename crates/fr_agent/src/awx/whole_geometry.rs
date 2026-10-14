//! Direct producer port of AWX whole_geo.py. Joint continuous geometry, not finite route candidates.
use super::{
    corridor::{self, Point, Spine},
    linear_program::{LinearProgram, LpSolution},
    numeric, pairs,
    whole_context::{LaneGeometry, WholeContext},
    whole_frame::{face, Face, WholeFrame},
    whole_types::WholePlan,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
const W_HARD: f64 = 1e4;
const K_MAX: f64 = 8.;
const TANG: [f64; 7] = [0., 1., -1., 2.5, -2.5, 5., -5.];
const LEN_T: [f64; 8] = [0.5, -0.5, 1., -1., 2., -2., 4., -4.];
type Key = (String, String, i64);
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Piece {
    pub frame: String,
    pub lane: String,
    pub k0: i64,
    pub k1: i64,
    pub o0: Option<f64>,
    pub o1: Option<f64>,
    pub hold0: i64,
    pub hold1: i64,
    pub ex0: i64,
    pub ex1: i64,
    pub reference: Vec<Point>,
    pub handoff_offset: Option<f64>,
}
impl Piece {
    pub fn reference_at(&self, s: f64) -> f64 {
        interp(&self.reference, s)
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GeometryFinding {
    pub kind: String,
    pub frame: String,
    pub lane: String,
    pub other: Option<String>,
    pub column: i64,
    pub amount: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GeometryResult {
    pub lanes: BTreeMap<String, LaneGeometry>,
    pub paid: Vec<GeometryFinding>,
    pub flips: BTreeSet<(String, String)>,
    pub objective: f64,
    pub held: BTreeSet<String>,
    pub folded: Vec<String>,
    pub failed: bool,
    pub conflicts: Vec<GeometryConflict>,
    pub changes: BTreeMap<String, Vec<f64>>,
    pub bands: BTreeMap<String, super::whole_context::RouterBand>,
    pub cuts: super::whole_types::GeometryCuts,
    pub islands: BTreeMap<String, String>,
    pub island_boxes: BTreeMap<String, super::feedback::IslandBox>,
    pub metadata_complete: bool,
}
#[derive(Clone, Debug)]
pub struct GeometryDomain {
    pub grid: f64,
    pub pieces: Vec<Piece>,
    pub spines: BTreeMap<String, Spine>,
    pub changes: BTreeMap<String, Vec<f64>>,
    pub pair_halfwidth: BTreeMap<String, f64>,
    pub via_span: BTreeMap<String, f64>,
    pub via_offset: BTreeMap<String, f64>,
    pub ring_out: BTreeMap<String, Point>,
    pub ring_standoff: f64,
}
fn interp(rows: &[Point], s: f64) -> f64 {
    if rows.is_empty() {
        return 0.;
    }
    if s < rows[0][0] {
        return rows[0][1];
    }
    for w in rows.windows(2) {
        if s < w[1][0] {
            let t = if w[1][0] > w[0][0] {
                (s - w[0][0]) / (w[1][0] - w[0][0])
            } else {
                0.
            };
            return w[0][1] + t * (w[1][1] - w[0][1]);
        }
    }
    rows.last().unwrap()[1]
}
impl GeometryDomain {
    pub fn new(ctx: &WholeContext, frame: &WholeFrame, plan: &WholePlan) -> Result<Self, String> {
        let g = 4. * ctx.cfg.grid_step;
        let hold = numeric::checked_round_i64(ctx.cfg.track_width / g)?.max(1);
        let exc = (ctx.cfg.lane_min / g).ceil().max(2.) as i64;
        let turn = pairs::turn_straight_steps(&ctx.cfg) as f64 * ctx.cfg.grid_step;
        let pair_hold =
            |tips: [Point; 2]| hold.max((pairs::end_run(&ctx.cfg, tips, None) / g).ceil() as i64);
        let mut pieces = Vec::new();
        let mut spines = BTreeMap::from([("T".into(), frame.spine.clone())]);
        spines.extend(frame.rings.clone());
        let mut changes = BTreeMap::new();
        let mut pair_halfwidth = BTreeMap::new();
        let mut via_span = BTreeMap::new();
        let mut via_offset = BTreeMap::new();
        let grid2 = ctx.cfg.grid_step / 2.;
        let via_r = (ctx.cfg.via_size / 2. + ctx.cfg.clearance + ctx.cfg.track_width / 2.)
            .max(pairs::via_ring(&ctx.cfg, 0., None) + 2. * grid2);
        let mut depths = Vec::new();
        for n in &frame.m {
            if let Some(k) = plan.branch.get(n) {
                if let Some(ends) = ctx.pair_ends.get(n) {
                    let ring = &frame.rings[k];
                    depths.push(
                        (ring.project_pt(frame.land[n])[1]
                            - ring.project_pt(pairs::mid(ends[1][0], ends[1][1]))[1])
                            .abs(),
                    );
                }
            }
        }
        let standoff = depths
            .into_iter()
            .max_by(f64::total_cmp)
            .unwrap_or(turn + ctx.cfg.track_width + ctx.cfg.clearance);
        for n in &frame.m {
            let is_pair = ctx.pairs.contains_key(n);
            let half = if is_pair {
                pairs::pitch(&ctx.cfg) / 2. / std::f64::consts::FRAC_PI_8.cos() + grid2
            } else {
                0.
            };
            pair_halfwidth.insert(n.clone(), half);
            via_span.insert(
                n.clone(),
                via_r
                    + if is_pair {
                        pairs::pitch(&ctx.cfg) / 2.
                    } else {
                        0.
                    },
            );
            via_offset.insert(
                n.clone(),
                if is_pair {
                    pairs::dive_offset(&ctx.cfg, pairs::pitch(&ctx.cfg) / 2.)
                } else {
                    0.
                },
            );
            let ch: Vec<_> = plan
                .changes
                .get(n)
                .into_iter()
                .flatten()
                .map(|u| u.max(plan.entry[n]).min(plan.end[n]))
                .collect();
            changes.insert(n.clone(), ch);
            let [s0, o0] = frame.st[n];
            let f0 = face(frame.tooth[n], frame.sb);
            let mut h0 = if matches!(f0, Face::E | Face::W) {
                if is_pair {
                    pair_hold(ctx.pair_ends[n][0])
                } else {
                    hold
                }
            } else {
                0
            };
            if frame.start[n] != frame.tooth[n] {
                h0 = hold
                    .max(((turn + ctx.cfg.track_width + ctx.cfg.clearance) / g).ceil() as i64 + 1);
            }
            let mut reference = frame.mid[n].clone();
            if let Some(k) = plan.branch.get(n) {
                let hk = plan.handoff[k];
                let oh = frame.o_h[n];
                let dh = (oh - interp(&reference, hk)).abs();
                let a = (hk - dh).max(s0);
                let ar = interp(&reference, a);
                reference.retain(|p| p[0] <= a);
                reference.push([a, ar]);
                reference.push([hk, oh]);
                pieces.push(Piece {
                    frame: "T".into(),
                    lane: n.clone(),
                    k0: numeric::checked_round_i64(s0 / g)?,
                    k1: numeric::checked_round_i64(hk / g)?,
                    o0: Some(o0),
                    o1: None,
                    hold0: h0,
                    hold1: 0,
                    ex0: (h0 + 1).max(exc),
                    ex1: 0,
                    reference,
                    handoff_offset: None,
                });
                let ring = &frame.rings[k];
                let [sb0, ob0] = ring.project_pt(frame.spine.xy(hk, oh));
                let [sb1, ob1] = ring.project_pt(frame.land[n]);
                let h1 = if is_pair {
                    hold.max(
                        ((turn + ctx.cfg.track_width + ctx.cfg.clearance) / g).ceil() as i64 + 1,
                    )
                } else {
                    0
                };
                let dpads = ctx.pads_of(&ctx.destination);
                let center = pairs::mid(
                    [
                        dpads.iter().map(|p| p.point[0]).sum::<f64>() / dpads.len() as f64,
                        dpads.iter().map(|p| p.point[1]).sum::<f64>() / dpads.len() as f64,
                    ],
                    [
                        dpads.iter().map(|p| p.point[0]).sum::<f64>() / dpads.len() as f64,
                        dpads.iter().map(|p| p.point[1]).sum::<f64>() / dpads.len() as f64,
                    ],
                );
                let side = if ob1 >= ring.project_pt(center)[1] {
                    1.
                } else {
                    -1.
                };
                let (o_run, d_in) = if is_pair {
                    (ob1, h1 as f64 * g + ctx.cfg.track_width + ctx.cfg.clearance)
                } else {
                    (
                        ob1 + side * standoff,
                        standoff + ctx.cfg.track_width + ctx.cfg.clearance,
                    )
                };
                let s_run = sb1 - d_in;
                let s_out = sb0 + (o_run - ob0).abs();
                let rpts = if s_out < s_run {
                    vec![[sb0, ob0], [s_out, o_run], [s_run, o_run], [sb1, ob1]]
                } else if s_run > sb0 + 1e-9 {
                    vec![[sb0, ob0], [s_run, o_run], [sb1, ob1]]
                } else {
                    vec![[sb0, ob0], [sb1, ob1]]
                };
                pieces.push(Piece {
                    frame: k.clone(),
                    lane: n.clone(),
                    k0: numeric::checked_round_i64(sb0 / g)?,
                    k1: numeric::checked_round_i64(sb1 / g)?,
                    o0: None,
                    o1: Some(ob1),
                    hold0: 0,
                    hold1: h1,
                    ex0: 0,
                    ex1: (h1 + 1).max(exc),
                    reference: rpts,
                    handoff_offset: Some(oh),
                });
            } else {
                let [s1, o1] = if frame.land[n] != frame.bend[n] {
                    frame.spine.project_pt(frame.land[n])
                } else {
                    frame.se[n]
                };
                let f1 = face(frame.bend[n], frame.db);
                let h1 = if frame.land[n] != frame.bend[n] {
                    hold.max(
                        ((turn + ctx.cfg.track_width + ctx.cfg.clearance) / g).ceil() as i64 + 1,
                    )
                } else if matches!(f1, Face::E | Face::W) {
                    if is_pair {
                        pair_hold(ctx.pair_ends[n][1])
                    } else {
                        hold
                    }
                } else {
                    0
                };
                pieces.push(Piece {
                    frame: "T".into(),
                    lane: n.clone(),
                    k0: numeric::checked_round_i64(s0 / g)?,
                    k1: numeric::checked_round_i64(s1 / g)?,
                    o0: Some(o0),
                    o1: Some(o1),
                    hold0: h0,
                    hold1: h1,
                    ex0: (h0 + 1).max(exc),
                    ex1: (h1 + 1).max(exc),
                    reference,
                    handoff_offset: None,
                });
            }
        }
        if pieces.iter().any(|p| p.k1 < p.k0) {
            return Err("AWX geometry piece reverses along its frame".into());
        }
        let mut ring_out = BTreeMap::new();
        for (k, sp) in &frame.rings {
            let o = frame.spine.project_pt(sp.pts[0])[1];
            let sum = frame
                .m
                .iter()
                .filter(|n| plan.branch.get(*n) == Some(k))
                .map(|n| frame.o_h[n] - o)
                .sum::<f64>();
            ring_out.insert(k.clone(), [if sum > 0. { 1. } else { -1. }, o]);
        }
        Ok(Self {
            grid: g,
            pieces,
            spines,
            changes,
            pair_halfwidth,
            via_span,
            via_offset,
            ring_out,
            ring_standoff: standoff,
        })
    }
    pub fn route_u(&self, tag: &str, s: f64, plan: &WholePlan) -> f64 {
        if tag == "T" {
            s
        } else {
            plan.handoff[tag] + s - plan.rs[tag]
        }
    }
    pub fn frame_s(&self, tag: &str, u: f64, plan: &WholePlan) -> f64 {
        if tag == "T" {
            u
        } else {
            plan.rs[tag] + u - plan.handoff[tag]
        }
    }
    pub fn layer_at(
        &self,
        ctx: &WholeContext,
        plan: &WholePlan,
        lane: &str,
        u: f64,
    ) -> Result<usize, String> {
        let count = self.changes[lane].iter().filter(|&&v| v < u).count();
        if let Some(runs) = plan.layers.get(lane) {
            let layer = runs
                .get(count)
                .ok_or("AWX solved layer profile incomplete")?;
            ctx.layers
                .iter()
                .position(|l| l == layer)
                .ok_or("AWX solved layer is outside stack".into())
        } else if ctx.layers.len() == 2 {
            Ok(ctx
                .layers
                .iter()
                .position(|l| l == &ctx.tooth_layer[lane])
                .ok_or("AWX tooth layer unknown")?
                ^ (count & 1))
        } else {
            Err("AWX multilayer solve omitted layer profile".into())
        }
    }
    pub fn layers_at(
        &self,
        ctx: &WholeContext,
        plan: &WholePlan,
        lane: &str,
        u: f64,
    ) -> Result<BTreeSet<usize>, String> {
        let near: Vec<_> = self.changes[lane]
            .iter()
            .filter(|&&v| (u - v).abs() <= self.via_span[lane])
            .collect();
        if near.is_empty() {
            Ok(BTreeSet::from([self.layer_at(ctx, plan, lane, u)?]))
        } else if ctx.layers.len() == 2 {
            Ok((0..2).collect())
        } else {
            let mut out = BTreeSet::new();
            for &v in near {
                out.insert(self.layer_at(ctx, plan, lane, v - 1e-6)?);
                out.insert(self.layer_at(ctx, plan, lane, v + 1e-6)?);
            }
            Ok(out)
        }
    }
    pub fn below(plan: &WholePlan, a: &str, b: &str, u: f64) -> Result<bool, String> {
        let ia = plan
            .launch
            .iter()
            .position(|n| n == a)
            .ok_or("AWX launch lane absent")?;
        let ib = plan
            .launch
            .iter()
            .position(|n| n == b)
            .ok_or("AWX launch lane absent")?;
        let key = if a < b {
            format!("{a}|{b}")
        } else {
            format!("{b}|{a}")
        };
        let crossed = plan.cross.get(&key).is_some_and(|c| c.u <= u + 1e-9);
        Ok((ia < ib) ^ crossed)
    }
}
#[derive(Clone, Debug)]
pub struct StaticSide {
    pub frame: String,
    pub lane: String,
    pub column: i64,
    pub lo: f64,
    pub hi: f64,
    pub side: i32,
    pub island: String,
}
#[derive(Clone, Debug)]
pub struct JointLp {
    pub program: LinearProgram,
    pub variables: BTreeMap<Key, usize>,
    pub tags: Vec<(usize, GeometryFinding)>,
    pub orders: BTreeMap<(String, i64), Vec<String>>,
    pub vias: Vec<(String, String, f64, i64)>,
}
#[derive(Clone, Debug)]
pub struct OffsetSolution {
    pub offsets: BTreeMap<Key, f64>,
    pub paid: Vec<GeometryFinding>,
    pub orders: BTreeMap<(String, i64), Vec<String>>,
    pub vias: Vec<(String, String, f64, i64)>,
    pub objective: f64,
}
impl JointLp {
    fn elastic(&mut self, terms: Vec<(usize, f64)>, rhs: f64, tag: GeometryFinding) {
        let (_, slack) = self.program.elastic_le(terms, rhs, W_HARD);
        self.tags.push((slack, tag));
    }
}
fn tag(kind: &str, p: &Piece, k: i64, other: Option<String>) -> GeometryFinding {
    GeometryFinding {
        kind: kind.into(),
        frame: p.frame.clone(),
        lane: p.lane.clone(),
        other,
        column: k,
        amount: 0.,
    }
}
fn slope_segments(
    vars: &BTreeMap<Key, usize>,
    frame: &str,
    lane: &str,
    k: i64,
) -> Vec<(usize, usize)> {
    let key = |k| (frame.into(), lane.into(), k);
    let Some(&j) = vars.get(&key(k)) else {
        return vec![];
    };
    let mut out = Vec::new();
    if let Some(&p) = vars.get(&key(k + 1)) {
        out.push((p, j));
    }
    if let Some(&p) = vars.get(&key(k - 1)) {
        out.push((j, p));
    }
    out
}
impl GeometryDomain {
    pub fn lane_mm(
        &self,
        p: &Piece,
        ka: i64,
        kb: i64,
        previous: Option<&BTreeMap<Key, f64>>,
    ) -> f64 {
        let a = ka.min(kb).max(p.k0);
        let b = ka.max(kb).min(p.k1);
        let value = |k| {
            previous
                .and_then(|o| o.get(&(p.frame.clone(), p.lane.clone(), k)))
                .copied()
                .unwrap_or_else(|| p.reference_at(k as f64 * self.grid))
        };
        (a..b)
            .map(|k| self.grid.hypot(value(k + 1) - value(k)))
            .sum()
    }
    pub fn lane_lam(&self, p: &Piece, k: i64, previous: Option<&BTreeMap<Key, f64>>) -> f64 {
        let a = (k - 3).max(p.k0);
        let b = (k + 3).min(p.k1);
        if b > a {
            (self.lane_mm(p, a, b, previous) / ((b - a) as f64 * self.grid)).max(1.)
        } else {
            1.
        }
    }
    pub fn build_and_solve(
        &self,
        ctx: &WholeContext,
        frame: &WholeFrame,
        plan: &WholePlan,
        sides: &[StaticSide],
        previous: Option<&BTreeMap<Key, f64>>,
        budget_ms: u64,
    ) -> Result<OffsetSolution, String> {
        let grid2 = ctx.cfg.grid_step / 2.;
        let p_min = ctx
            .cfg
            .lane_min
            .max(ctx.cfg.track_width + ctx.cfg.clearance + 2. * grid2);
        let p_comf = 2. * ctx.cfg.lane_min;
        let p_mid = (p_min + p_comf) / 2.;
        let span = (ctx.board_bounds[2] - ctx.board_bounds[0])
            .max(ctx.board_bounds[3] - ctx.board_bounds[1]);
        let mut lp = JointLp {
            program: LinearProgram::default(),
            variables: BTreeMap::new(),
            tags: Vec::new(),
            orders: BTreeMap::new(),
            vias: Vec::new(),
        };
        for p in &self.pieces {
            for k in p.k0..=p.k1 {
                let id = lp.program.variable(-2. * span, 2. * span, 0.);
                lp.variables
                    .insert((p.frame.clone(), p.lane.clone(), k), id);
            }
        }
        let tangent_variables = lp.variables.clone();
        let tangents = |sl: &[(usize, usize)]| -> Vec<(f64, f64)> {
            if let Some(prev) = previous {
                if !sl.is_empty() {
                    let reverse: BTreeMap<_, _> =
                        tangent_variables.iter().map(|(key, &j)| (j, key)).collect();
                    let mean = sl
                        .iter()
                        .map(|&(a, b)| {
                            (prev.get(reverse[&a]).copied().unwrap_or(0.)
                                - prev.get(reverse[&b]).copied().unwrap_or(0.))
                                / self.grid
                        })
                        .sum::<f64>()
                        / sl.len() as f64;
                    let mut ts = vec![
                        numeric::round_digits(mean - 0.35, 4).unwrap(),
                        numeric::round_digits(mean, 4).unwrap(),
                        numeric::round_digits(mean + 0.35, 4).unwrap(),
                        1.5,
                        -1.5,
                        4.,
                        -4.,
                    ];
                    ts.sort_by(f64::total_cmp);
                    ts.dedup();
                    return ts
                        .into_iter()
                        .filter(|x| x.abs() > 1e-3)
                        .map(|t| (t, 1.))
                        .collect();
                }
            }
            let mut ratio = f64::INFINITY;
            for i in 0..=4000 {
                let k = i as f64 * K_MAX / 4000.;
                let max = TANG
                    .iter()
                    .map(|t| (1. + t * k) / (1. + t * t).sqrt())
                    .fold(f64::NEG_INFINITY, f64::max);
                ratio = ratio.min(max / (1. + k * k).sqrt());
            }
            TANG.iter()
                .copied()
                .filter(|t| *t != 0.)
                .map(|t| (t, 1. / ratio))
                .collect()
        };
        for f in self.spines.keys() {
            let ps: Vec<_> = self.pieces.iter().filter(|p| &p.frame == f).collect();
            if ps.is_empty() {
                continue;
            }
            let kmin = ps.iter().map(|p| p.k0).min().unwrap();
            let kmax = ps.iter().map(|p| p.k1).max().unwrap();
            for k in kmin..=kmax {
                let u = self.route_u(f, k as f64 * self.grid, plan);
                let mut present: Vec<_> = ps
                    .iter()
                    .filter(|p| p.k0 <= k && k <= p.k1)
                    .map(|p| p.lane.clone())
                    .collect();
                present.sort_by(|a, b| {
                    if Self::below(plan, a, b, u).unwrap_or(false) {
                        std::cmp::Ordering::Less
                    } else {
                        std::cmp::Ordering::Greater
                    }
                });
                if present.is_empty() {
                    continue;
                }
                lp.orders.insert((f.clone(), k), present.clone());
                let mut layer_sets = BTreeMap::new();
                for n in &present {
                    let lu = plan
                        .branch
                        .get(n)
                        .map(|b| corridor::piece_u(u, f == "T", plan.handoff[b]))
                        .unwrap_or(u);
                    layer_sets.insert(n.clone(), self.layers_at(ctx, plan, n, lu)?);
                }
                for layer in 0..ctx.layers.len() {
                    let same: Vec<_> = present
                        .iter()
                        .filter(|n| layer_sets[*n].contains(&layer))
                        .collect();
                    for pair in same.windows(2) {
                        let (a, b) = (pair[0], pair[1]);
                        let pa = *ps.iter().find(|p| &p.lane == a).unwrap();
                        let pb = *ps.iter().find(|p| &p.lane == b).unwrap();
                        let ja = lp.variables[&(f.clone(), a.clone(), k)];
                        let jb = lp.variables[&(f.clone(), b.clone(), k)];
                        let sa = slope_segments(&lp.variables, f, a, k);
                        let sb = slope_segments(&lp.variables, f, b, k);
                        let turn_room = ctx
                            .cfg
                            .geometry_pair_turn_room_steps
                            .unwrap_or(if ctx.layers.len() > 2 { 1. } else { 0. })
                            * ctx.cfg.grid_step;
                        let nturn = (pairs::turn_straight_steps(&ctx.cfg) as f64
                            * ctx.cfg.grid_step
                            / self.grid
                            - 1e-9)
                            .ceil() as i64;
                        let turning = |n: &String, p: &Piece| {
                            ctx.pairs.contains_key(n)
                                && ((p.o0.is_some() && k - p.k0 <= p.hold0 + nturn)
                                    || (p.o1.is_some() && p.k1 - k <= p.hold1 + nturn))
                        };
                        let mut sep = p_min
                            + self.pair_halfwidth[a]
                            + self.pair_halfwidth[b]
                            + if turning(a, pa) || turning(b, pb) {
                                turn_room
                            } else {
                                0.
                            };
                        let terminal = |p: &Piece| {
                            if k - p.k0 < p.ex0 {
                                p.o0.map(|v| (0, v))
                            } else if p.k1 - k < p.ex1 {
                                p.o1.map(|v| (1, v))
                            } else {
                                None
                            }
                        };
                        if let (Some(x), Some(y)) = (terminal(pa), terminal(pb)) {
                            if x.0 == y.0 {
                                sep = sep.min(((x.1 - y.1).abs() - self.grid / 20.).max(0.));
                            }
                        }
                        lp.elastic(
                            vec![(ja, 1.), (jb, -1.)],
                            -sep,
                            tag("pitch", pa, k, Some(b.clone())),
                        );
                        let mut flat_only = None;
                        if let Some(prev) = previous {
                            let slope = |n: &String| {
                                let o0 = prev.get(&(f.clone(), n.clone(), k));
                                let o1 = prev
                                    .get(&(f.clone(), n.clone(), k + 1))
                                    .or_else(|| prev.get(&(f.clone(), n.clone(), k - 1)));
                                match (o0, o1) {
                                    (Some(a), Some(b)) => (b - a).abs() / self.grid,
                                    _ => 0.,
                                }
                            };
                            let ka = slope(a);
                            let kb = slope(b);
                            if (ka - kb).abs() > 1. && ka.min(kb) < 0.3 {
                                flat_only = Some(if ka < kb { sa.clone() } else { sb.clone() });
                            }
                        }
                        let combos: Vec<Vec<(usize, usize)>> = if let Some(flat) = flat_only {
                            flat.into_iter().map(|x| vec![x]).collect()
                        } else {
                            (0..sa.len().max(sb.len()))
                                .map(|i| {
                                    let mut sl = Vec::new();
                                    if !sa.is_empty() {
                                        sl.push(sa[i.min(sa.len() - 1)]);
                                    }
                                    if !sb.is_empty() {
                                        sl.push(sb[i.min(sb.len() - 1)]);
                                    }
                                    sl
                                })
                                .collect()
                        };
                        for ta in if previous.is_some() {
                            TANG.to_vec()
                        } else {
                            vec![0.]
                        } {
                            let scale = if ta == 0. { 1. } else { tangents(&[])[0].1 };
                            let al = scale / (1. + ta * ta).sqrt();
                            let be = scale * ta / (1. + ta * ta).sqrt();
                            let rows = if be == 0. {
                                vec![vec![]]
                            } else {
                                combos.clone()
                            };
                            for segments in rows {
                                let mut terms = vec![(ja, 1.), (jb, -1.)];
                                for (j1, j0) in &segments {
                                    let coef = sep * be / (segments.len() as f64 * self.grid);
                                    terms.push((*j1, coef));
                                    terms.push((*j0, -coef));
                                }
                                lp.elastic(terms, -sep * al, tag("pitch", pa, k, Some(b.clone())));
                            }
                        }
                        let e = lp.program.variable(0., f64::INFINITY, self.grid);
                        lp.program.le(
                            vec![(ja, 1.), (jb, -1.), (e, -1.)],
                            -(p_comf + self.pair_halfwidth[a] + self.pair_halfwidth[b]),
                        );
                        let e = lp.program.variable(0., f64::INFINITY, 3. * self.grid);
                        lp.program.le(
                            vec![(ja, 1.), (jb, -1.), (e, -1.)],
                            -(p_mid + self.pair_halfwidth[a] + self.pair_halfwidth[b]),
                        );
                    }
                }
            }
        }
        // Via rows, including exact crossover barrel stagger and per-layer nearest neighbours.
        let via_ring = (ctx.cfg.via_size / 2. + ctx.cfg.clearance + ctx.cfg.track_width / 2.)
            .max(pairs::via_ring(&ctx.cfg, 0., None) + ctx.cfg.grid_step);
        let crossover_axis = pairs::crossover_shape(&ctx.cfg, [1., 0.], 1)
            .map(|c| c.0)
            .unwrap_or_default();
        for p in &self.pieces {
            for &cu in &self.changes[&p.lane] {
                let branch = plan.branch.get(&p.lane);
                let trunk = branch.is_none_or(|b| cu <= plan.handoff[b]);
                if (p.frame == "T") != trunk {
                    continue;
                }
                let site_s = self.frame_s(&p.frame, cu, plan);
                let kc = numeric::checked_round_i64(site_s / self.grid)?;
                if !(p.k0 <= kc && kc <= p.k1) {
                    continue;
                }
                lp.vias.push((p.frame.clone(), p.lane.clone(), cu, kc));
                let jc = lp.variables[&(p.frame.clone(), p.lane.clone(), kc)];
                let crossover = plan.crossover_lanes.contains(&p.lane)
                    && self.changes[&p.lane]
                        .first()
                        .is_some_and(|u| (*u - cu).abs() < 1e-9);
                let mut side = 0;
                if crossover {
                    if let (Some(prev), Some(order)) =
                        (previous, lp.orders.get(&(p.frame.clone(), kc)))
                    {
                        if let Some(i) = order.iter().position(|n| n == &p.lane) {
                            let value = prev
                                .get(&(p.frame.clone(), p.lane.clone(), kc))
                                .copied()
                                .unwrap_or(0.);
                            let up = order[i + 1..]
                                .iter()
                                .map(|n| {
                                    prev.get(&(p.frame.clone(), n.clone(), kc))
                                        .copied()
                                        .unwrap_or(value)
                                        - value
                                })
                                .fold(9., f64::min);
                            let down = order[..i]
                                .iter()
                                .map(|n| {
                                    value
                                        - prev
                                            .get(&(p.frame.clone(), n.clone(), kc))
                                            .copied()
                                            .unwrap_or(value)
                                })
                                .fold(9., f64::min);
                            side = if up >= down { 1 } else { -1 };
                        }
                    }
                }
                let barrels: Vec<(f64, f64, i32)> = if crossover {
                    crossover_axis
                        .iter()
                        .map(|b| (b[0], b[1].abs(), side))
                        .collect()
                } else {
                    vec![(0., self.via_offset[&p.lane], 0)]
                };
                for (along, across, barrel_side) in barrels {
                    let s = site_s + along;
                    let column = numeric::checked_round_i64(s / self.grid)?;
                    let jv = lp
                        .variables
                        .get(&(p.frame.clone(), p.lane.clone(), column))
                        .copied()
                        .unwrap_or(jc);
                    for k in ((s - via_ring) / self.grid).floor() as i64
                        ..=((s + via_ring) / self.grid).ceil() as i64
                    {
                        let ds = k as f64 * self.grid - s;
                        if ds.abs() >= via_ring {
                            continue;
                        }
                        let Some(order) = lp.orders.get(&(p.frame.clone(), k)).cloned() else {
                            continue;
                        };
                        let Some(i) = order.iter().position(|n| n == &p.lane) else {
                            continue;
                        };
                        if !lp
                            .variables
                            .contains_key(&(p.frame.clone(), p.lane.clone(), k))
                        {
                            continue;
                        }
                        let mut neighbours: Vec<usize> =
                            [i.checked_sub(1), i.checked_sub(2), Some(i + 1), Some(i + 2)]
                                .into_iter()
                                .flatten()
                                .filter(|&j| j < order.len())
                                .collect();
                        for candidates in [
                            ((i + 1)..order.len()).collect::<Vec<_>>(),
                            (0..i).rev().collect(),
                        ] {
                            for layer in 0..ctx.layers.len() {
                                let mut chosen = None;
                                for &j in &candidates {
                                    let u = self.route_u(&p.frame, k as f64 * self.grid, plan);
                                    let u = plan
                                        .branch
                                        .get(&order[j])
                                        .map(|b| {
                                            corridor::piece_u(u, p.frame == "T", plan.handoff[b])
                                        })
                                        .unwrap_or(u);
                                    if self.layers_at(ctx, plan, &order[j], u)?.contains(&layer) {
                                        chosen = Some(j);
                                        break;
                                    }
                                }
                                if let Some(j) = chosen {
                                    if !neighbours.contains(&j) {
                                        neighbours.push(j);
                                    }
                                }
                            }
                        }
                        for j in neighbours {
                            let n = &order[j];
                            let up = j > i;
                            if barrel_side != 0 && (barrel_side > 0) != up {
                                continue;
                            }
                            let Some(&jm) = lp.variables.get(&(p.frame.clone(), n.clone(), k))
                            else {
                                continue;
                            };
                            let pn = self
                                .pieces
                                .iter()
                                .find(|q| q.frame == p.frame && &q.lane == n)
                                .unwrap();
                            let mut need = (via_ring * via_ring - ds * ds).sqrt()
                                + across
                                + self.pair_halfwidth[n];
                            if k - pn.k0 < pn.ex0 || pn.k1 - k < pn.ex1 {
                                need += self.grid / 4.;
                            }
                            let sign = if up { 1. } else { -1. };
                            lp.elastic(
                                vec![(jv, sign), (jm, -sign)],
                                -need,
                                tag("via", p, k, Some(n.clone())),
                            );
                            for (j1, j0) in slope_segments(&lp.variables, &p.frame, n, k) {
                                for (t, scale) in tangents(&[(j1, j0)]) {
                                    let al = scale / (1. + t * t).sqrt();
                                    let be = scale * t / (1. + t * t).sqrt();
                                    lp.elastic(
                                        vec![
                                            (jv, sign),
                                            (jm, -sign),
                                            (j1, sign * need * be / self.grid),
                                            (j0, -sign * need * be / self.grid),
                                        ],
                                        -need * al,
                                        tag("via", p, k, Some(n.clone())),
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }
        let via_via = (ctx.cfg.via_size + ctx.cfg.clearance)
            .max(ctx.cfg.via_drill + ctx.cfg.hole_to_hole_clearance)
            + ctx.cfg.grid_step;
        for i in 0..lp.vias.len() {
            for j in i + 1..lp.vias.len() {
                let (f, n, u, k) = lp.vias[i].clone();
                let (f2, m, v, k2) = lp.vias[j].clone();
                if f != f2 || n == m {
                    continue;
                }
                let ds = (self.frame_s(&f, u, plan) - self.frame_s(&f2, v, plan)).abs();
                if ds >= via_via {
                    continue;
                }
                let middle = numeric::floor_div(k + k2, 2)?;
                let Some(order) = lp.orders.get(&(f.clone(), middle)) else {
                    continue;
                };
                let (Some(a), Some(b)) = (
                    order.iter().position(|x| x == &n),
                    order.iter().position(|x| x == &m),
                ) else {
                    continue;
                };
                let (na, ka, nb, kb) = if a < b {
                    (n.clone(), k, m.clone(), k2)
                } else {
                    (m.clone(), k2, n.clone(), k)
                };
                let ja = lp.variables[&(f.clone(), na.clone(), ka)];
                let jb = lp.variables[&(f.clone(), nb.clone(), kb)];
                let width = (via_via * via_via - ds * ds).sqrt()
                    + self.via_offset[&n]
                    + self.via_offset[&m];
                let p = self
                    .pieces
                    .iter()
                    .find(|p| p.frame == f && p.lane == na)
                    .unwrap();
                lp.elastic(
                    vec![(ja, 1.), (jb, -1.)],
                    -width,
                    tag("viavia", p, middle, Some(nb)),
                );
            }
        }
        // Source free offset intervals: true board bounds and the arrays' grown boxes.
        let mut previous_interval = BTreeMap::<(String, String), Point>::new();
        for p in &self.pieces {
            let inside = |q: Point, b: [f64; 4]| {
                b[0] + 1e-6 < q[0] && q[0] < b[2] - 1e-6 && b[1] + 1e-6 < q[1] && q[1] < b[3] - 1e-6
            };
            let own = [
                inside(frame.tooth[&p.lane], frame.sb),
                !plan.branch.contains_key(&p.lane) && inside(frame.bend[&p.lane], frame.db),
            ];
            for k in p.k0..=p.k1 {
                if (p.o0.is_some() && k - p.k0 <= p.hold0)
                    || (p.o1.is_some() && p.k1 - k <= p.hold1)
                {
                    continue;
                }
                let near = k - p.k0 < p.ex0 || p.k1 - k < p.ex1;
                let is_via = lp
                    .vias
                    .iter()
                    .any(|(f, n, _, c)| f == &p.frame && n == &p.lane && *c == k);
                let margin = if is_via {
                    ctx.cfg.via_size / 2.
                        + ctx.cfg.clearance
                        + ctx.cfg.grid_step / 2.
                        + self.via_offset[&p.lane]
                } else if near {
                    0.
                } else {
                    ctx.cfg.track_width / 2.
                };
                let iv = self.intervals(
                    ctx,
                    frame,
                    &p.frame,
                    k as f64 * self.grid,
                    numeric::round_digits(margin, 4)?,
                    if p.frame == "T" { own } else { [false, false] },
                );
                if iv.is_empty() {
                    continue;
                }
                let key = (p.frame.clone(), p.lane.clone());
                let terminal = if p.k1 - k <= k - p.k0 { p.o1 } else { p.o0 };
                if let Some(bounds) = corridor::held_interval(
                    &iv,
                    p.reference_at(k as f64 * self.grid),
                    previous_interval.get(&key).copied(),
                    terminal,
                ) {
                    previous_interval.insert(key, bounds);
                    let j = lp.variables[&(p.frame.clone(), p.lane.clone(), k)];
                    lp.elastic(
                        vec![(j, -1.)],
                        -bounds[0],
                        tag("bound", p, k, Some("lo".into())),
                    );
                    lp.elastic(
                        vec![(j, 1.)],
                        bounds[1],
                        tag("bound", p, k, Some("hi".into())),
                    );
                }
            }
        }
        // Terminal holds, slope, convex length tangents, bending and per-column turn cap.
        for p in &self.pieces {
            for (k, o, h, sign) in [(p.k0, p.o0, p.hold0, 1i64), (p.k1, p.o1, p.hold1, -1)] {
                if let Some(o) = o {
                    for i in 0..=h {
                        if let Some(&j) =
                            lp.variables
                                .get(&(p.frame.clone(), p.lane.clone(), k + i * sign))
                        {
                            lp.program.variables[j].lower = o;
                            lp.program.variables[j].upper = o;
                        }
                    }
                }
            }
            for k in p.k0..p.k1 {
                let a = lp.variables[&(p.frame.clone(), p.lane.clone(), k)];
                let b = lp.variables[&(p.frame.clone(), p.lane.clone(), k + 1)];
                lp.elastic(
                    vec![(b, 1.), (a, -1.)],
                    K_MAX * self.grid,
                    tag("slope", p, k, None),
                );
                lp.elastic(
                    vec![(a, 1.), (b, -1.)],
                    K_MAX * self.grid,
                    tag("slope", p, k, None),
                );
                let d = lp.program.variable(0., f64::INFINITY, 1.);
                for t in LEN_T {
                    let r = (1. + t * t).sqrt();
                    lp.program.le(
                        vec![(b, t / r), (a, -t / r), (d, -1.)],
                        -self.grid * (1. / r - 1.),
                    );
                }
                if k > p.k0 {
                    let prev = lp.variables[&(p.frame.clone(), p.lane.clone(), k - 1)];
                    let d = lp.program.variable(0., f64::INFINITY, 3.);
                    lp.program
                        .le(vec![(b, 1.), (a, -2.), (prev, 1.), (d, -1.)], 0.);
                    lp.program
                        .le(vec![(b, -1.), (a, 2.), (prev, -1.), (d, -1.)], 0.);
                    lp.elastic(
                        vec![(b, 1.), (a, -2.), (prev, 1.)],
                        self.grid,
                        tag("turn", p, k, None),
                    );
                    lp.elastic(
                        vec![(b, -1.), (a, 2.), (prev, -1.)],
                        self.grid,
                        tag("turn", p, k, None),
                    );
                }
            }
        }
        // Pair turning runs, straight-through dives and monotone terminal approaches (source 775–866).
        let turn_columns =
            (pairs::turn_straight_steps(&ctx.cfg) as f64 * ctx.cfg.grid_step / self.grid - 1e-9)
                .ceil() as i64;
        if let Some(prev) = previous {
            for p in &self.pieces {
                if ctx.pairs.contains_key(&p.lane) {
                    for k in p.k0..p.k1 {
                        if k - turn_columns < p.k0 {
                            continue;
                        }
                        let keys = [
                            (p.frame.clone(), p.lane.clone(), k + 1),
                            (p.frame.clone(), p.lane.clone(), k),
                            (p.frame.clone(), p.lane.clone(), k - turn_columns + 1),
                            (p.frame.clone(), p.lane.clone(), k - turn_columns),
                        ];
                        let ids = keys.clone().map(|q| lp.variables.get(&q).copied());
                        if let [Some(b), Some(a), Some(q1), Some(q0)] = ids {
                            let slope = |i, j| {
                                (prev.get(&keys[i]).copied().unwrap_or(0.)
                                    - prev.get(&keys[j]).copied().unwrap_or(0.))
                                    / self.grid
                            };
                            let angle = (slope(0, 1).atan() + slope(2, 3).atan()) / 2.;
                            let lo = angle - std::f64::consts::FRAC_PI_8;
                            let hi = angle + std::f64::consts::FRAC_PI_8;
                            let allow = if lo.abs().max(hi.abs()) >= (2. * K_MAX).atan() {
                                2. * K_MAX
                            } else {
                                (hi.tan() - lo.tan()).min(2. * K_MAX)
                            };
                            lp.elastic(
                                vec![(b, 1.), (a, -1.), (q1, -1.), (q0, 1.)],
                                allow * self.grid,
                                tag("pturn", p, k, None),
                            );
                            lp.elastic(
                                vec![(b, -1.), (a, 1.), (q1, 1.), (q0, -1.)],
                                allow * self.grid,
                                tag("pturn", p, k, None),
                            );
                        }
                    }
                }
                for (terminal, zone, forward, offset) in [
                    (p.k1, (p.k1 - p.ex1)..p.k1, true, p.o1),
                    (p.k0, p.k0..p.k0 + p.ex0, false, p.o0),
                ] {
                    let Some(offset) = offset else { continue };
                    let columns: Vec<_> = zone
                        .filter(|k| {
                            lp.variables
                                .contains_key(&(p.frame.clone(), p.lane.clone(), *k))
                                && lp.variables.contains_key(&(
                                    p.frame.clone(),
                                    p.lane.clone(),
                                    *k + 1,
                                ))
                        })
                        .collect();
                    if columns.is_empty() {
                        continue;
                    }
                    let far = if forward {
                        columns[0]
                    } else {
                        columns.last().unwrap() + 1
                    };
                    let delta = offset
                        - prev
                            .get(&(p.frame.clone(), p.lane.clone(), far))
                            .copied()
                            .unwrap_or(offset);
                    if delta.abs() < 1e-6 {
                        continue;
                    }
                    let sign = if (delta > 0.) == forward { 1. } else { -1. };
                    for k in columns {
                        let a = lp.variables[&(p.frame.clone(), p.lane.clone(), k)];
                        let b = lp.variables[&(p.frame.clone(), p.lane.clone(), k + 1)];
                        lp.elastic(
                            vec![(a, sign), (b, -sign)],
                            0.,
                            tag("approach", p, terminal, None),
                        );
                    }
                }
            }
        }
        for (f, n, u, kc) in lp.vias.clone() {
            if !ctx.pairs.contains_key(&n) {
                continue;
            }
            let p = self
                .pieces
                .iter()
                .find(|p| p.frame == f && p.lane == n)
                .unwrap();
            let crossed = plan.crossover_lanes.contains(&n)
                && self.changes[&n]
                    .first()
                    .is_some_and(|c| (*c - u).abs() < 1e-9);
            let run = pairs::crossover_shape(&ctx.cfg, [0.5_f64.sqrt(); 2], 1)
                .map(|s| s.1)
                .unwrap_or([pairs::via_straight(&ctx.cfg, [0.5_f64.sqrt(); 2]); 2]);
            let normal = (pairs::via_straight(&ctx.cfg, [0.5_f64.sqrt(); 2]) + ctx.cfg.grid_step)
                / self.grid;
            let mut before = if crossed {
                ((run[0] + ctx.cfg.grid_step) / self.grid - 1e-9).ceil() as i64
            } else {
                (normal - 1e-9).ceil() as i64
            };
            let mut after = if crossed {
                ((run[1] + ctx.cfg.grid_step) / self.grid - 1e-9).ceil() as i64
            } else {
                before
            };
            let near1 = if ctx.layers.len() > 2 {
                p.o1.is_some()
                    && self.lane_mm(p, kc, p.k1, previous)
                        <= (p.hold1 + after + turn_columns) as f64 * self.grid
            } else {
                p.o1.is_some() && p.k1 - kc <= p.hold1 + after + turn_columns
            };
            let near0 = if ctx.layers.len() > 2 {
                p.o0.is_some()
                    && self.lane_mm(p, p.k0, kc, previous)
                        <= (p.hold0 + before + turn_columns) as f64 * self.grid
            } else {
                p.o0.is_some() && kc - p.k0 <= p.hold0 + before + turn_columns
            };
            if ctx.layers.len() > 2 {
                let lam = self.lane_lam(p, kc, previous);
                before = (before as f64 / lam - 1e-9).ceil().max(1.) as i64;
                after = (after as f64 / lam - 1e-9).ceil().max(1.) as i64;
            }
            if near1 {
                let o = p.o1.unwrap();
                for k in (p.k0.max(kc - before))..p.k1 - p.hold1 {
                    if let Some(&j) = lp.variables.get(&(f.clone(), n.clone(), k)) {
                        lp.elastic(vec![(j, 1.)], o, tag("pdive", p, kc, None));
                        lp.elastic(vec![(j, -1.)], -o, tag("pdive", p, kc, None));
                    }
                }
            } else if near0 {
                let o = p.o0.unwrap();
                for k in p.k0 + p.hold0 + 1..=p.k1.min(kc + after) {
                    if let Some(&j) = lp.variables.get(&(f.clone(), n.clone(), k)) {
                        lp.elastic(vec![(j, 1.)], o, tag("pdive", p, kc, None));
                        lp.elastic(vec![(j, -1.)], -o, tag("pdive", p, kc, None));
                    }
                }
            } else {
                for k in (p.k0 + 1).max(kc - before + 1)..=(p.k1 - 1).min(kc + after - 1) {
                    let a = lp.variables[&(f.clone(), n.clone(), k - 1)];
                    let b = lp.variables[&(f.clone(), n.clone(), k)];
                    let c = lp.variables[&(f.clone(), n.clone(), k + 1)];
                    lp.elastic(
                        vec![(c, 1.), (b, -2.), (a, 1.)],
                        0.,
                        tag("pdive", p, kc, None),
                    );
                    lp.elastic(
                        vec![(c, -1.), (b, 2.), (a, -1.)],
                        0.,
                        tag("pdive", p, kc, None),
                    );
                }
            }
        }
        for n in &frame.m {
            if let Some(k) = plan.branch.get(n) {
                let t = self
                    .pieces
                    .iter()
                    .find(|p| p.frame == "T" && &p.lane == n)
                    .ok_or("AWX missing trunk piece")?;
                let r = self
                    .pieces
                    .iter()
                    .find(|p| &p.frame == k && &p.lane == n)
                    .ok_or("AWX missing ring piece")?;
                let sp = &self.spines[k];
                let oh = r.handoff_offset.ok_or("ring handoff offset missing")?;
                let s = t.k1 as f64 * self.grid;
                let o1 = sp.project_pt(frame.spine.xy(s, oh))[1];
                let o2 = sp.project_pt(frame.spine.xy(s, oh + 1.))[1];
                let beta = o2 - o1;
                let alpha = o1 - beta * oh;
                let jt = lp.variables[&("T".into(), n.clone(), t.k1)];
                let jr = lp.variables[&(k.clone(), n.clone(), r.k0)];
                lp.program.equal(vec![(jr, 1.), (jt, -beta)], alpha);
                let [sg, edge] = self.ring_out[k];
                lp.elastic(vec![(jt, -sg)], -sg * edge, tag("ringside", t, t.k1, None));
            }
        }
        for side in sides {
            if let Some(&j) =
                lp.variables
                    .get(&(side.frame.clone(), side.lane.clone(), side.column))
            {
                let p = self
                    .pieces
                    .iter()
                    .find(|p| p.frame == side.frame && p.lane == side.lane)
                    .unwrap();
                let (sign, rhs) = if side.side < 0 {
                    (1., side.lo)
                } else {
                    (-1., -side.hi)
                };
                lp.elastic(
                    vec![(j, sign)],
                    rhs,
                    tag("static", p, side.column, Some(side.island.clone())),
                );
                let e = lp.program.variable(0., f64::INFINITY, self.grid);
                lp.program
                    .le(vec![(j, sign), (e, -1.)], rhs - ctx.cfg.clearance);
            }
        }
        for (j, v) in lp.program.variables.iter_mut().enumerate() {
            v.cost += 1e-4 * (0.5 + ((j + 1) as f64 * 0.6180339887498949).rem_euclid(1.));
        }
        let solution = lp.program.solve_by_dual(budget_ms)?;
        let offsets = lp
            .variables
            .iter()
            .map(|(k, &j)| (k.clone(), solution.values[j]))
            .collect();
        let mut paid = Vec::new();
        for (slack, mut finding) in lp.tags {
            if solution.values[slack] > 1e-4 {
                finding.amount = solution.values[slack];
                paid.push(finding);
            }
        }
        Ok(OffsetSolution {
            offsets,
            paid,
            orders: lp.orders,
            vias: lp.vias,
            objective: solution.objective,
        })
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GeometryConflict {
    pub frame: String,
    pub xy: Point,
    pub lanes: Vec<String>,
    pub kind: String,
}
#[derive(Clone, Debug)]
pub struct StaticObject {
    pub label: String,
    pub rectangles: Vec<[f64; 4]>,
    pub layers: BTreeSet<usize>,
    pub owner: Option<String>,
}
impl GeometryDomain {
    pub fn intervals(
        &self,
        ctx: &WholeContext,
        frame: &WholeFrame,
        tag: &str,
        s: f64,
        margin: f64,
        own: [bool; 2],
    ) -> Vec<Point> {
        let bounds = ctx.board_bounds;
        let span = (bounds[2] - bounds[0]).max(bounds[3] - bounds[1]);
        let step = ctx.cfg.track_width / 6.;
        let edge = if ctx.cfg.board_edge_clearance > 0. {
            ctx.cfg.board_edge_clearance
        } else {
            ctx.cfg.clearance
        } + ctx.cfg.track_width / 2.;
        let mut boxes = [frame.sb, frame.db];
        for (i, part) in [&frame.src, &frame.dst].iter().enumerate() {
            if own[i] {
                let ps = ctx.pads_of(part);
                let radius = ps
                    .iter()
                    .map(|p| p.size[0].max(p.size[1]) / 2.)
                    .fold(0., f64::max)
                    + ctx.cfg.clearance;
                let raw = ctx.box_of(part).unwrap();
                boxes[i] = [
                    boxes[i][0].max(raw[0] - radius),
                    boxes[i][1].max(raw[1] - radius),
                    boxes[i][2].min(raw[2] + radius),
                    boxes[i][3].min(raw[3] + radius),
                ];
            }
        }
        let sp = &self.spines[tag];
        let mut out = Vec::new();
        let mut begin = None;
        let mut last = 0.;
        let count = ((2. * span + 1e-9) / step).floor().max(0.) as usize + 1;
        for i in 0..count {
            let offset = -span + i as f64 * step;
            let p = sp.xy(s, offset);
            let okay = bounds[0] + edge <= p[0]
                && p[0] <= bounds[2] - edge
                && bounds[1] + edge <= p[1]
                && p[1] <= bounds[3] - edge
                && !boxes.iter().any(|b| {
                    b[0] - margin < p[0]
                        && p[0] < b[2] + margin
                        && b[1] - margin < p[1]
                        && p[1] < b[3] + margin
                });
            if okay {
                begin.get_or_insert(offset);
                last = offset;
            } else if let Some(lo) = begin.take() {
                out.push([lo, last]);
            }
        }
        if let Some(lo) = begin {
            out.push([lo, last]);
        }
        out
    }
    pub fn static_objects(
        &self,
        ctx: &WholeContext,
        frame: &WholeFrame,
    ) -> (Vec<StaticObject>, BTreeMap<String, String>) {
        let skip = BTreeSet::from([frame.src.clone(), frame.dst.clone()]);
        let mut split = BTreeSet::new();
        let parts: BTreeSet<_> = ctx.pads.iter().map(|p| p.component_ref.clone()).collect();
        for part in &parts {
            let ps: Vec<_> = ctx
                .pads_of(part)
                .into_iter()
                .filter(|p| p.drill > 0. || p.layers.iter().any(|l| ctx.layers.contains(l)))
                .collect();
            if ps.len() == 2 && corridor::pads_across(&self.spines, [ps[0].point, ps[1].point]) {
                split.insert(part.clone());
            }
        }
        let islands = ctx.part_islands(&skip, &split);
        let mut grouped: BTreeMap<(String, BTreeSet<usize>), Vec<[f64; 4]>> = BTreeMap::new();
        for p in &ctx.pads {
            if skip.contains(&p.component_ref) {
                continue;
            }
            let layers: BTreeSet<_> = if p.drill > 0. {
                (0..ctx.layers.len()).collect()
            } else {
                ctx.layers
                    .iter()
                    .enumerate()
                    .filter(|(_, l)| p.layers.contains(l))
                    .map(|(i, _)| i)
                    .collect()
            };
            if layers.is_empty() {
                continue;
            }
            let half = if p.non_plated_hole {
                [p.drill / 2.; 2]
            } else {
                [p.size[0] / 2., p.size[1] / 2.]
            };
            let bounds = ctx.board_bounds;
            if !(bounds[0] < p.point[0]
                && p.point[0] < bounds[2]
                && bounds[1] < p.point[1]
                && p.point[1] < bounds[3])
            {
                continue;
            }
            let label = islands
                .get(&p.key)
                .cloned()
                .unwrap_or_else(|| p.component_ref.clone());
            let rects = if (p.non_plated_hole
                || p.copper_shapes
                    .iter()
                    .any(|s| matches!(s.shape, super::whole_context::CopperShape::Circle { .. })))
                && (half[0] - half[1]).abs() < 1e-9
            {
                corridor::round_cover(p.point, half)
            } else {
                vec![[
                    p.point[0] - half[0],
                    p.point[1] - half[1],
                    p.point[0] + half[0],
                    p.point[1] + half[1],
                ]]
            };
            grouped.entry((label, layers)).or_default().extend(rects);
        }
        let mut objects: Vec<_> = grouped
            .into_iter()
            .map(|((label, layers), rectangles)| StaticObject {
                label,
                layers,
                rectangles,
                owner: None,
            })
            .collect();
        let mut net_lane = BTreeMap::new();
        for n in &frame.m {
            if let Some(&id) = ctx.byname.get(n) {
                net_lane.insert(id, n.clone());
            }
            if let Some(legs) = ctx.pairs.get(n) {
                for leg in legs {
                    if let Some(&id) = ctx.byname.get(leg) {
                        net_lane.insert(id, n.clone());
                    }
                }
            }
        }
        let reach = ctx.cfg.track_width + ctx.cfg.clearance;
        for seg in &ctx.base_segments {
            let Some(layer) = ctx.layers.iter().position(|l| l == &seg.layer) else {
                continue;
            };
            for (a, b) in [(seg.a, seg.b), (seg.b, seg.a)] {
                let near = [frame.sb, frame.db].iter().any(|r| {
                    ((a[0] - r[0]).abs().min((a[0] - r[2]).abs()) < reach
                        && r[1] - reach <= a[1]
                        && a[1] <= r[3] + reach)
                        || ((a[1] - r[1]).abs().min((a[1] - r[3]).abs()) < reach
                            && r[0] - reach <= a[0]
                            && a[0] <= r[2] + reach)
                });
                if !near {
                    continue;
                }
                let l = super::corridor::distance(a, b);
                let t = if l > 1e-9 { (reach / l).min(1.) } else { 0. };
                let end =
                    super::corridor::add(a, super::corridor::mul(super::corridor::sub(b, a), t));
                let r = seg.width / 2.;
                objects.push(StaticObject {
                    label: format!("stub {}", net_lane.get(&seg.net_id).unwrap_or(&seg.net)),
                    rectangles: vec![[
                        a[0].min(end[0]) - r,
                        a[1].min(end[1]) - r,
                        a[0].max(end[0]) + r,
                        a[1].max(end[1]) + r,
                    ]],
                    layers: BTreeSet::from([layer]),
                    owner: net_lane.get(&seg.net_id).cloned(),
                });
            }
        }
        for via in &ctx.base_vias {
            let reach = ctx.cfg.via_size / 2. + ctx.cfg.clearance + ctx.cfg.track_width / 2.;
            if [frame.sb, frame.db].iter().any(|b| {
                b[0] - reach <= via.point[0]
                    && via.point[0] <= b[2] + reach
                    && b[1] - reach <= via.point[1]
                    && via.point[1] <= b[3] + reach
            }) {
                let r = via.diameter / 2.;
                objects.push(StaticObject {
                    label: format!("svia {}", net_lane.get(&via.net_id).unwrap_or(&via.net)),
                    rectangles: vec![[
                        via.point[0] - r,
                        via.point[1] - r,
                        via.point[0] + r,
                        via.point[1] + r,
                    ]],
                    layers: (0..ctx.layers.len()).collect(),
                    owner: net_lane.get(&via.net_id).cloned(),
                });
            }
        }
        (objects, islands)
    }
    pub fn reanchor(
        &mut self,
        frame: &WholeFrame,
        plan: &WholePlan,
        solution: &OffsetSolution,
    ) -> Result<(), String> {
        for n in &frame.m {
            if let Some(k) = plan.branch.get(n) {
                let t = self
                    .pieces
                    .iter()
                    .find(|p| p.frame == "T" && &p.lane == n)
                    .ok_or("missing trunk anchor")?;
                let Some(&offset) = solution.offsets.get(&("T".into(), n.clone(), t.k1)) else {
                    continue;
                };
                let s = t.k1 as f64 * self.grid;
                let projected = self.spines[k].project_pt(frame.spine.xy(s, offset));
                let r = self
                    .pieces
                    .iter_mut()
                    .find(|p| &p.frame == k && &p.lane == n)
                    .ok_or("missing ring anchor")?;
                r.k0 = numeric::checked_round_i64(projected[0] / self.grid)?
                    .max(0)
                    .min(r.k1 - 2);
                r.handoff_offset = Some(offset);
                if !r.reference.is_empty() {
                    r.reference[0] = projected;
                }
            }
        }
        Ok(())
    }
}
#[derive(Clone, Debug)]
struct Projection {
    span: Point,
    offset: Point,
}
#[derive(Clone, Debug)]
struct IslandSplit {
    lanes: Vec<String>,
    below: Vec<f64>,
    above: Vec<f64>,
    pins: Vec<Option<i32>>,
}
impl GeometryDomain {
    fn island_reach(
        &self,
        ctx: &WholeContext,
        frame: &WholeFrame,
        tag: &str,
        p: &Projection,
    ) -> Option<Vec<Point>> {
        let grow = ctx.cfg.track_width / 2.
            + ctx.cfg.clearance
            + ctx.cfg.grid_step / 2.
            + self.pair_halfwidth.values().copied().fold(0., f64::max);
        for (lo, hi) in [(p.span[0], p.span[1]), (p.span[0] - grow, p.span[1] + grow)] {
            let mut out = Vec::new();
            let mut s = lo;
            while s <= hi + 1e-9 {
                let iv = self.intervals(
                    ctx,
                    frame,
                    tag,
                    s,
                    numeric::round_digits(ctx.cfg.track_width / 2., 4).ok()?,
                    [false, false],
                );
                let around: Vec<_> = iv
                    .iter()
                    .filter(|q| q[0] <= p.offset[0] + 1e-6 && q[1] >= p.offset[1] - 1e-6)
                    .collect();
                if !around.is_empty() {
                    out.push([
                        around.iter().map(|q| q[0]).fold(f64::INFINITY, f64::min),
                        around
                            .iter()
                            .map(|q| q[1])
                            .fold(f64::NEG_INFINITY, f64::max),
                    ]);
                }
                s += self.grid;
            }
            if !out.is_empty() {
                return Some(out);
            }
        }
        None
    }
    pub fn static_sides(
        &self,
        ctx: &WholeContext,
        frame: &WholeFrame,
        plan: &WholePlan,
        sol: &OffsetSolution,
        objects: &[StaticObject],
        flips: &BTreeSet<(String, String)>,
    ) -> Result<Vec<StaticSide>, String> {
        let lane_st = ctx.cfg.track_width / 2. + ctx.cfg.clearance + ctx.cfg.grid_step / 2.;
        let win = 12. * ctx.cfg.lane_min;
        let p_min = ctx
            .cfg
            .lane_min
            .max(ctx.cfg.track_width + ctx.cfg.clearance + ctx.cfg.grid_step);
        let mut projected = BTreeMap::<(String, usize), Projection>::new();
        let mut home = BTreeMap::new();
        let mut carry = BTreeMap::new();
        for (i, obj) in objects.iter().enumerate() {
            let bounds = [
                obj.rectangles
                    .iter()
                    .map(|r| r[0])
                    .fold(f64::INFINITY, f64::min),
                obj.rectangles
                    .iter()
                    .map(|r| r[1])
                    .fold(f64::INFINITY, f64::min),
                obj.rectangles
                    .iter()
                    .map(|r| r[2])
                    .fold(f64::NEG_INFINITY, f64::max),
                obj.rectangles
                    .iter()
                    .map(|r| r[3])
                    .fold(f64::NEG_INFINITY, f64::max),
            ];
            let center = [(bounds[0] + bounds[2]) / 2., (bounds[1] + bounds[3]) / 2.];
            let mut spans = BTreeMap::new();
            for (f, sp) in &self.spines {
                let qs = [
                    [bounds[0], bounds[1]],
                    [bounds[0], bounds[3]],
                    [bounds[2], bounds[1]],
                    [bounds[2], bounds[3]],
                ]
                .map(|q| sp.project_pt(q));
                let span = [
                    qs.iter().map(|q| q[0]).fold(f64::INFINITY, f64::min),
                    qs.iter().map(|q| q[0]).fold(f64::NEG_INFINITY, f64::max),
                ];
                let offset = [
                    qs.iter().map(|q| q[1]).fold(f64::INFINITY, f64::min),
                    qs.iter().map(|q| q[1]).fold(f64::NEG_INFINITY, f64::max),
                ];
                spans.insert(f.clone(), span);
                projected.insert((f.clone(), i), Projection { span, offset });
            }
            let h = corridor::part_home(&self.spines, &spans, center)
                .ok_or("AWX island has no home frame")?;
            for (f, sp) in &self.spines {
                carry.insert(
                    (i, f.clone()),
                    if f == &h {
                        1
                    } else {
                        corridor::side_carry(
                            &self.spines[&h],
                            sp,
                            center,
                            projected[&(h.clone(), i)].offset[1],
                            lane_st,
                        )
                    },
                );
            }
            home.insert(i, h);
        }
        let mut roomless = BTreeMap::new();
        for ((f, i), pr) in &projected {
            if objects[*i].owner.is_some() || pr.span[1] < 0. {
                continue;
            }
            let mut reach = self.island_reach(ctx, frame, f, pr).unwrap_or_default();
            if reach.is_empty() {
                let mut s = pr.span[0];
                while s <= pr.span[1] + 1e-9 {
                    let iv =
                        self.intervals(ctx, frame, f, s, ctx.cfg.track_width / 2., [false, false]);
                    let near: Vec<_> = iv
                        .iter()
                        .filter(|q| q[1] >= pr.offset[0] - win && q[0] <= pr.offset[1] + win)
                        .collect();
                    if !near.is_empty() {
                        reach.push([
                            near.iter().map(|q| q[0]).fold(f64::INFINITY, f64::min),
                            near.iter().map(|q| q[1]).fold(f64::NEG_INFINITY, f64::max),
                        ]);
                    }
                    s += self.grid;
                }
            }
            let need = 2. * lane_st + ctx.cfg.track_width;
            let below = reach.iter().all(|q| pr.offset[0] - q[0] >= need);
            let above = reach.iter().all(|q| q[1] - pr.offset[1] >= need);
            if below != above {
                roomless.insert((f.clone(), *i), if !below { 1 } else { -1 });
            }
        }
        let mut means = BTreeMap::<(String, String, usize), Vec<f64>>::new();
        for ((f, n, k), &o) in &sol.offsets {
            let s = *k as f64 * self.grid;
            for (i, obj) in objects.iter().enumerate() {
                let pr = &projected[&(f.clone(), i)];
                let g = lane_st + self.pair_halfwidth[n];
                if obj.owner.as_ref() != Some(n)
                    && pr.span[0] - g <= s
                    && s <= pr.span[1] + g
                    && pr.offset[0] - win <= o
                    && o <= pr.offset[1] + win
                {
                    means.entry((f.clone(), n.clone(), i)).or_default().push(o);
                }
            }
        }
        let mean = |v: &[f64]| v.iter().sum::<f64>() / v.len() as f64;
        let mut sides = BTreeMap::<(String, String, usize), i32>::new();
        let mut splits = BTreeMap::<(String, usize, usize), IslandSplit>::new();
        let mut split_objects = BTreeSet::new();
        for ((f, i), pr) in &projected {
            if pr.span[1] < 0. {
                continue;
            }
            let mid = numeric::checked_round_i64((pr.span[0] + pr.span[1]) / 2. / self.grid)?;
            let lo = (pr.span[0] / self.grid).floor() as i64 - 1;
            let hi = (pr.span[1] / self.grid).ceil() as i64 + 1;
            let mut columns: Vec<_> = (lo..=hi)
                .filter(|&k| {
                    sol.orders
                        .get(&(f.clone(), k))
                        .is_some_and(|o| !o.is_empty())
                })
                .collect();
            columns.sort_by_key(|k| (*k - mid).abs());
            let Some(&km) = columns.first() else { continue };
            let order = &sol.orders[&(f.clone(), km)];
            let um = self.route_u(f, km as f64 * self.grid, plan);
            let reach = self.island_reach(ctx, frame, f, pr).unwrap_or_default();
            let mut room = [f64::INFINITY; 2];
            let mut interval = [f64::NEG_INFINITY, f64::INFINITY];
            for q in reach {
                room[0] = room[0].min(pr.offset[0] - q[0]);
                room[1] = room[1].min(q[1] - pr.offset[1]);
                interval[0] = interval[0].max(q[0]);
                interval[1] = interval[1].min(q[1]);
            }
            if f != "T" {
                let sp = &self.spines[f];
                let mid = (pr.offset[0] + pr.offset[1]) / 2.;
                let pads = ctx.pads_of(&ctx.destination);
                if pads.is_empty() {
                    return Err("ring destination has no pads".into());
                }
                let centre = [
                    pads.iter().map(|p| p.point[0]).sum::<f64>() / pads.len() as f64,
                    pads.iter().map(|p| p.point[1]).sum::<f64>() / pads.len() as f64,
                ];
                let [_, own] = sp.project_pt(centre);
                let outward = if mid >= own { 1. } else { -1. };
                room[if outward > 0. { 0 } else { 1 }] -= self.ring_standoff;
            }
            split_objects.insert((f.clone(), *i));
            for layer in 0..ctx.layers.len() {
                if !objects[*i].layers.contains(&layer) {
                    continue;
                }
                let mut lanes = Vec::new();
                for n in order {
                    let u = plan
                        .branch
                        .get(n)
                        .map(|b| corridor::piece_u(um, f == "T", plan.handoff[b]))
                        .unwrap_or(um);
                    if objects[*i].owner.as_ref() == Some(n)
                        || !self.layers_at(ctx, plan, n, u)?.contains(&layer)
                    {
                        continue;
                    }
                    if let Some(values) = means.get(&(f.clone(), n.clone(), *i)) {
                        let m = mean(values);
                        if interval[0] - 1e-6 <= m && m <= interval[1] + 1e-6 {
                            lanes.push(n.clone());
                        }
                    }
                }
                if lanes.is_empty() {
                    continue;
                }
                let mut below = Vec::new();
                let mut above = Vec::new();
                let mut pins = Vec::new();
                for n in &lanes {
                    let bar = lane_st + self.pair_halfwidth[n];
                    let m = mean(&means[&(f.clone(), n.clone(), *i)]);
                    below.push((m - (pr.offset[0] - bar)).max(0.));
                    above.push((pr.offset[1] + bar - m).max(0.));
                    let p = self
                        .pieces
                        .iter()
                        .find(|p| &p.frame == f && &p.lane == n)
                        .unwrap();
                    let mut pin = None;
                    for (k, o) in [(p.k0, p.o0), (p.k1, p.o1)] {
                        if let Some(o) = o {
                            if pr.span[0] - bar <= k as f64 * self.grid
                                && k as f64 * self.grid <= pr.span[1] + bar
                            {
                                pin = Some(if o < (pr.offset[0] + pr.offset[1]) / 2. {
                                    -1
                                } else {
                                    1
                                });
                            }
                        }
                    }
                    if ctx.layers.len() > 2 {
                        if let Some(&sd) = sides.get(&(f.clone(), n.clone(), *i)) {
                            pin = Some(sd);
                        }
                    }
                    pins.push(pin);
                }
                let spare = |side: usize, ls: &[String]| {
                    let mut used = 0.;
                    let mut previous = None::<&String>;
                    for n in ls {
                        used += if let Some(p) = previous {
                            p_min + self.pair_halfwidth[p] + self.pair_halfwidth[n]
                        } else {
                            lane_st + self.pair_halfwidth[n]
                        };
                        previous = Some(n);
                    }
                    room[side] - used
                };
                let forced = roomless.get(&(f.clone(), *i)).copied();
                let mut best = None;
                for j in 0..=lanes.len() {
                    if forced == Some(1) && j != 0 || forced == Some(-1) && j != lanes.len() {
                        continue;
                    }
                    if pins[..j].contains(&Some(1)) || pins[j..].contains(&Some(-1)) {
                        continue;
                    }
                    let mut cost = below[..j].iter().sum::<f64>() + above[j..].iter().sum::<f64>();
                    let mut low = lanes[..j].to_vec();
                    low.reverse();
                    let sb = spare(0, &low);
                    let sa = spare(1, &lanes[j..]);
                    if sb < -1e-9 || sa < -1e-9 {
                        cost += 1e6;
                    } else if ctx.cfg.geometry_pair_room
                        && ((sb < p_min && low.iter().any(|n| ctx.pairs.contains_key(n)))
                            || (sa < p_min && lanes[j..].iter().any(|n| ctx.pairs.contains_key(n))))
                    {
                        cost += 1e3;
                    }
                    if best.is_none_or(|(old, _)| cost < old) {
                        best = Some((cost, j));
                    }
                }
                if let Some((_, j)) = best {
                    for (k, n) in lanes.iter().enumerate() {
                        sides.insert((f.clone(), n.clone(), *i), if k < j { -1 } else { 1 });
                    }
                    splits.insert(
                        (f.clone(), *i, layer),
                        IslandSplit {
                            lanes,
                            below,
                            above,
                            pins,
                        },
                    );
                }
            }
        }
        // Tight gaps between adjacent islands: a lane crosses each island at most once, outward only.
        let gap_used = |lanes: &[String]| {
            let mut used = 0.;
            let mut previous = None::<&String>;
            for n in lanes {
                used += if let Some(p) = previous {
                    p_min + self.pair_halfwidth[p] + self.pair_halfwidth[n]
                } else {
                    lane_st + self.pair_halfwidth[n]
                };
                previous = Some(n);
            }
            used + lanes
                .last()
                .map(|n| lane_st + self.pair_halfwidth[n])
                .unwrap_or(0.)
        };
        for f in self.spines.keys() {
            for layer in 0..ctx.layers.len() {
                let indices: Vec<_> = objects
                    .iter()
                    .enumerate()
                    .filter(|(i, _)| splits.contains_key(&(f.clone(), *i, layer)))
                    .map(|(i, _)| i)
                    .collect();
                let mut gaps = Vec::new();
                for &a in &indices {
                    for &b in &indices {
                        let pa = &projected[&(f.clone(), a)];
                        let pb = &projected[&(f.clone(), b)];
                        let lo = pa.span[0].max(pb.span[0]);
                        let hi = pa.span[1].min(pb.span[1]);
                        let gap = pb.offset[0] - pa.offset[1];
                        if a == b || hi <= lo || gap < 0. || gap >= win {
                            continue;
                        }
                        if indices.iter().any(|&c| {
                            c != a && c != b && {
                                let p = &projected[&(f.clone(), c)];
                                p.span[0] < hi
                                    && p.span[1] > lo
                                    && p.offset[0] >= pa.offset[1] - 1e-9
                                    && p.offset[1] <= pb.offset[0] + 1e-9
                            }
                        }) {
                            continue;
                        }
                        let gap = objects[a]
                            .rectangles
                            .iter()
                            .flat_map(|p| {
                                objects[b].rectangles.iter().map(move |q| {
                                    (q[0] - p[2])
                                        .max(p[0] - q[2])
                                        .max(0.)
                                        .hypot((q[1] - p[3]).max(p[1] - q[3]).max(0.))
                                })
                            })
                            .fold(f64::INFINITY, f64::min);
                        gaps.push((a, b, gap));
                    }
                }
                let mut pushed = BTreeSet::new();
                for _ in 0..50 {
                    let mut moved = false;
                    for &(a, b, gap) in &gaps {
                        let sa = &splits[&(f.clone(), a, layer)];
                        let sb = &splits[&(f.clone(), b, layer)];
                        let middle: Vec<_> = sa
                            .lanes
                            .iter()
                            .filter(|n| {
                                sb.lanes.contains(*n)
                                    && sides.get(&(f.clone(), (*n).clone(), a)) == Some(&1)
                                    && sides.get(&(f.clone(), (*n).clone(), b)) == Some(&-1)
                            })
                            .cloned()
                            .collect();
                        if middle.is_empty() || gap_used(&middle) <= gap + 1e-9 {
                            continue;
                        }
                        let low = &middle[0];
                        let high = middle.last().unwrap();
                        let ia = sa.lanes.iter().position(|n| n == low).unwrap();
                        let ib = sb.lanes.iter().position(|n| n == high).unwrap();
                        let mut options = Vec::new();
                        if sa.pins[ia] != Some(1)
                            && roomless.get(&(f.clone(), a)) != Some(&1)
                            && !pushed.contains(&(low.clone(), a))
                        {
                            options.push((sa.below[ia] - sa.above[ia], a, low.clone(), -1));
                        }
                        if sb.pins[ib] != Some(-1)
                            && roomless.get(&(f.clone(), b)) != Some(&-1)
                            && !pushed.contains(&(high.clone(), b))
                        {
                            options.push((sb.above[ib] - sb.below[ib], b, high.clone(), 1));
                        }
                        options.sort_by(|a, b| {
                            a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2))
                        });
                        if let Some((_, island, n, side)) = options.into_iter().next() {
                            sides.insert((f.clone(), n.clone(), island), side);
                            pushed.insert((n, island));
                            moved = true;
                        }
                    }
                    if !moved {
                        break;
                    }
                }
            }
        }
        let mut columns = Vec::new();
        for p in &self.pieces {
            let laid: Vec<_> = (p.k0..=p.k1)
                .filter(|k| {
                    sol.offsets
                        .contains_key(&(p.frame.clone(), p.lane.clone(), *k))
                })
                .collect();
            if laid.is_empty() {
                continue;
            }
            let first = laid[0];
            for k in p.k0..=p.k1 {
                let offset = sol
                    .offsets
                    .get(&(p.frame.clone(), p.lane.clone(), k))
                    .copied()
                    .unwrap_or_else(|| {
                        if k > first {
                            let previous = *laid.iter().filter(|&&old| old < k).max().unwrap();
                            sol.offsets[&(p.frame.clone(), p.lane.clone(), previous)]
                        } else {
                            let begin = p.reference_at(p.k0 as f64 * self.grid);
                            let end = sol.offsets[&(p.frame.clone(), p.lane.clone(), first)];
                            begin + (end - begin) * (k - p.k0) as f64 / (first - p.k0).max(1) as f64
                        }
                    });
                columns.push((p, k, offset));
            }
        }
        let mut meets = BTreeMap::<(String, usize), BTreeMap<String, Vec<f64>>>::new();
        for (p, k, offset) in &columns {
            let s = *k as f64 * self.grid;
            let u = self.route_u(&p.frame, s, plan);
            let u = plan
                .branch
                .get(&p.lane)
                .map(|b| corridor::piece_u(u, p.frame == "T", plan.handoff[b]))
                .unwrap_or(u);
            let layers = self.layers_at(ctx, plan, &p.lane, u)?;
            let at_end = (*k - p.k0 < 2 && p.o0.is_some()) || (p.k1 - *k < 2 && p.o1.is_some());
            for (i, obj) in objects.iter().enumerate() {
                let pr = &projected[&(p.frame.clone(), i)];
                let bar = lane_st + self.pair_halfwidth[&p.lane];
                if obj.owner.as_ref() == Some(&p.lane)
                    || layers.is_disjoint(&obj.layers)
                    || s < pr.span[0] - bar
                    || s > pr.span[1] + bar
                    || *offset < pr.offset[0] - win
                    || *offset > pr.offset[1] + win
                    || (at_end
                        && !obj.label.starts_with("stub ")
                        && !obj.label.starts_with("svia "))
                {
                    continue;
                }
                if let Some(ext) =
                    corridor::line_extent(&self.spines[&p.frame], s, &obj.rectangles, bar)
                {
                    if ext[0].max(pr.offset[0] - bar) <= ext[1].min(pr.offset[1] + bar)
                        && (ext[0] - *offset).max(*offset - ext[1]).max(0.) <= win
                    {
                        meets
                            .entry((p.lane.clone(), i))
                            .or_default()
                            .entry(p.frame.clone())
                            .or_default()
                            .push(*offset);
                    }
                }
            }
        }
        let mut decided = BTreeMap::new();
        for ((n, i), by_frame) in &meets {
            if by_frame.len() < 2 {
                continue;
            }
            let h = &home[i];
            let (source, side) = if let Some(&side) = sides.get(&(h.clone(), n.clone(), *i)) {
                (h.clone(), side)
            } else {
                let mut chosen = None;
                for (f, values) in by_frame {
                    let score = (values.len(), f == h);
                    if chosen.as_ref().is_none_or(|(_, old)| score > *old) {
                        chosen = Some((f.clone(), score));
                    }
                }
                let f = chosen.unwrap().0;
                let pr = &projected[&(f.clone(), *i)];
                let side = sides.get(&(f.clone(), n.clone(), *i)).copied().unwrap_or(
                    if mean(&by_frame[&f]) < (pr.offset[0] + pr.offset[1]) / 2. {
                        -1
                    } else {
                        1
                    },
                );
                (f, side)
            };
            decided.insert((n.clone(), *i), side * carry[&(*i, source)]);
        }
        let mut out = Vec::new();
        for (p, k, offset) in columns {
            let s = k as f64 * self.grid;
            let u = self.route_u(&p.frame, s, plan);
            let u = plan
                .branch
                .get(&p.lane)
                .map(|b| corridor::piece_u(u, p.frame == "T", plan.handoff[b]))
                .unwrap_or(u);
            let layers = self.layers_at(ctx, plan, &p.lane, u)?;
            let at_end = (k - p.k0 < 2 && p.o0.is_some()) || (p.k1 - k < 2 && p.o1.is_some());
            for (i, obj) in objects.iter().enumerate() {
                let pr = &projected[&(p.frame.clone(), i)];
                if obj.owner.as_ref() == Some(&p.lane)
                    || offset < pr.offset[0] - win
                    || offset > pr.offset[1] + win
                    || (at_end
                        && !obj.label.starts_with("stub ")
                        && !obj.label.starts_with("svia "))
                {
                    continue;
                }
                let bar = lane_st + self.pair_halfwidth[&p.lane];
                if !layers.is_disjoint(&obj.layers)
                    && pr.span[0] - bar <= s
                    && s <= pr.span[1] + bar
                {
                    if let Some(mut ext) =
                        corridor::line_extent(&self.spines[&p.frame], s, &obj.rectangles, bar)
                    {
                        ext[0] = ext[0].max(pr.offset[0] - bar);
                        ext[1] = ext[1].min(pr.offset[1] + bar);
                        if ext[0] <= ext[1] && (ext[0] - offset).max(offset - ext[1]).max(0.) <= win
                        {
                            let mut side =
                                sides.get(&(p.frame.clone(), p.lane.clone(), i)).copied();
                            if let Some(&carried) = decided.get(&(p.lane.clone(), i)) {
                                side = Some(carried * carry[&(i, p.frame.clone())]);
                            }
                            if side.is_none() {
                                for (kt, ot) in [(p.k0, p.o0), (p.k1, p.o1)] {
                                    if let Some(o) = ot {
                                        if pr.span[0] - bar <= kt as f64 * self.grid
                                            && kt as f64 * self.grid <= pr.span[1] + bar
                                        {
                                            side =
                                                Some(if o < (pr.offset[0] + pr.offset[1]) / 2. {
                                                    -1
                                                } else {
                                                    1
                                                });
                                        }
                                    }
                                }
                            }
                            let mut side = side.unwrap_or_else(|| {
                                if means
                                    .get(&(p.frame.clone(), p.lane.clone(), i))
                                    .map(|m| mean(m))
                                    .unwrap_or(offset)
                                    < (pr.offset[0] + pr.offset[1]) / 2.
                                {
                                    -1
                                } else {
                                    1
                                }
                            });
                            if flips.contains(&(p.lane.clone(), obj.label.clone())) {
                                side = -side;
                            }
                            if let Some(&forced) = roomless.get(&(p.frame.clone(), i)) {
                                let pinned = [(p.k0, p.o0), (p.k1, p.o1)].iter().any(|(kt, ot)| {
                                    ot.is_some()
                                        && pr.span[0] - bar <= *kt as f64 * self.grid
                                        && *kt as f64 * self.grid <= pr.span[1] + bar
                                });
                                if !split_objects.contains(&(p.frame.clone(), i))
                                    && !pinned
                                    && !decided.contains_key(&(p.lane.clone(), i))
                                {
                                    side = forced;
                                }
                            }
                            out.push(StaticSide {
                                frame: p.frame.clone(),
                                lane: p.lane.clone(),
                                column: k,
                                lo: ext[0],
                                hi: ext[1],
                                side,
                                island: obj.label.clone(),
                            });
                        }
                    }
                }
                if sol
                    .vias
                    .iter()
                    .any(|(f, n, _, kc)| f == &p.frame && n == &p.lane && *kc == k)
                {
                    let bar = ctx.cfg.via_size / 2.
                        + ctx.cfg.clearance
                        + ctx.cfg.grid_step / 2.
                        + self.pair_halfwidth[&p.lane].max(self.via_offset[&p.lane]);
                    if pr.span[0] - bar <= s && s <= pr.span[1] + bar {
                        out.push(StaticSide {
                            frame: p.frame.clone(),
                            lane: p.lane.clone(),
                            column: k,
                            lo: pr.offset[0] - bar,
                            hi: pr.offset[1] + bar,
                            side: if offset < (pr.offset[0] + pr.offset[1]) / 2. {
                                -1
                            } else {
                                1
                            },
                            island: format!("via {}", obj.label),
                        });
                    }
                }
            }
        }
        Ok(out)
    }
}
fn chamfer(pieces: &mut Vec<super::whole_context::LayerPiece>, cut: f64) {
    if pieces.len() < 2 {
        return;
    }
    let last = pieces.last().unwrap().clone();
    let out = super::corridor::distance(last.a, last.b);
    if out <= cut {
        return;
    }
    let c2 = super::corridor::add(
        last.a,
        super::corridor::mul(super::corridor::sub(last.b, last.a), cut / out),
    );
    let mut back = cut;
    for i in (0..pieces.len() - 1).rev() {
        if pieces[i].layer != last.layer {
            break;
        }
        let len = super::corridor::distance(pieces[i].a, pieces[i].b);
        if len > back {
            let c1 = super::corridor::add(
                pieces[i].b,
                super::corridor::mul(super::corridor::sub(pieces[i].a, pieces[i].b), back / len),
            );
            let layer = last.layer.clone();
            pieces.truncate(i + 1);
            pieces[i].b = c1;
            pieces.push(super::whole_context::LayerPiece {
                a: c1,
                b: c2,
                layer: layer.clone(),
            });
            pieces.push(super::whole_context::LayerPiece {
                a: c2,
                b: last.b,
                layer,
            });
            return;
        }
        back -= len;
    }
}
impl GeometryDomain {
    pub fn emit(
        &self,
        ctx: &WholeContext,
        frame: &WholeFrame,
        plan: &WholePlan,
        solution: &OffsetSolution,
        objects: &[StaticObject],
        islands: BTreeMap<String, String>,
        flips: BTreeSet<(String, String)>,
    ) -> Result<GeometryResult, String> {
        use super::whole_context::LayerPiece;
        let mut lanes = BTreeMap::new();
        let mut island_boxes = BTreeMap::<String, super::feedback::IslandBox>::new();
        for object in objects {
            let b = object.rectangles.iter().fold(
                [
                    f64::INFINITY,
                    f64::INFINITY,
                    f64::NEG_INFINITY,
                    f64::NEG_INFINITY,
                ],
                |mut b, r| {
                    b[0] = b[0].min(r[0]);
                    b[1] = b[1].min(r[1]);
                    b[2] = b[2].max(r[2]);
                    b[3] = b[3].max(r[3]);
                    b
                },
            );
            let item =
                island_boxes
                    .entry(object.label.clone())
                    .or_insert(super::feedback::IslandBox {
                        bounds: b,
                        layers: BTreeSet::new(),
                    });
            item.bounds = [
                item.bounds[0].min(b[0]),
                item.bounds[1].min(b[1]),
                item.bounds[2].max(b[2]),
                item.bounds[3].max(b[3]),
            ];
            item.layers.extend(&object.layers);
        }
        for n in &frame.m {
            let mut pieces = Vec::<LayerPiece>::new();
            let mut uxy = Vec::new();
            for p in self.pieces.iter().filter(|p| &p.lane == n) {
                let sp = &self.spines[&p.frame];
                let mut so = Vec::new();
                let mut fixed = BTreeSet::new();
                let change_s: Vec<_> = self.changes[n]
                    .iter()
                    .filter(|&&u| {
                        (p.frame == "T")
                            == (!plan.branch.contains_key(n) || u <= plan.handoff[&plan.branch[n]])
                    })
                    .map(|&u| self.frame_s(&p.frame, u, plan))
                    .collect();
                for k in p.k0..p.k1 {
                    let a = k as f64 * self.grid;
                    let b = (k + 1) as f64 * self.grid;
                    let oa = solution.offsets[&(p.frame.clone(), n.clone(), k)];
                    let ob = solution.offsets[&(p.frame.clone(), n.clone(), k + 1)];
                    so.push([a, oa]);
                    for &s in &change_s {
                        if a < s && s < b {
                            fixed.insert(so.len());
                            so.push([s, oa + (ob - oa) * (s - a) / (b - a)]);
                        }
                    }
                }
                so.push([
                    p.k1 as f64 * self.grid,
                    solution.offsets[&(p.frame.clone(), n.clone(), p.k1)],
                ]);
                fixed.insert(0);
                fixed.insert(so.len() - 1);
                let line = sp.lane_line(&so, &fixed);
                for w in line.windows(2) {
                    let a = [w[0][0], w[0][1]];
                    let b = [w[1][0], w[1][1]];
                    if super::corridor::distance(a, b) < 1e-9 {
                        continue;
                    }
                    let u = self.route_u(&p.frame, (w[0][2] + w[1][2]) / 2., plan);
                    let u = plan
                        .branch
                        .get(n)
                        .map(|tag| corridor::piece_u(u, p.frame == "T", plan.handoff[tag]))
                        .unwrap_or(u);
                    let layer = ctx.layers[self.layer_at(ctx, plan, n, u)?].clone();
                    if let Some(last) = pieces.last().cloned() {
                        let d1 = super::corridor::sub(last.b, last.a);
                        let d2 = super::corridor::sub(b, a);
                        let l1 = super::corridor::length(d1);
                        let l2 = super::corridor::length(d2);
                        if last.layer == layer
                            && l1 > 1e-9
                            && l2 > 1e-9
                            && super::corridor::dot(d1, d2) / (l1 * l2) < -0.5
                            && l1.min(l2) < ctx.cfg.lane_min
                        {
                            pieces.last_mut().unwrap().b = b;
                            continue;
                        }
                        if super::corridor::distance(last.b, a) > 1e-9 {
                            pieces.push(LayerPiece {
                                a: last.b,
                                b: a,
                                layer: layer.clone(),
                            });
                        }
                    }
                    pieces.push(LayerPiece { a, b, layer });
                }
                for k in p.k0..=p.k1 {
                    let s = k as f64 * self.grid;
                    let point = sp.xy(s, solution.offsets[&(p.frame.clone(), n.clone(), k)]);
                    uxy.push([
                        numeric::round_digits(self.route_u(&p.frame, s, plan), 4)?,
                        numeric::round_digits(point[0], 4)?,
                        numeric::round_digits(point[1], 4)?,
                    ]);
                }
            }
            if pieces.is_empty() {
                return Err(format!("AWX geometry produced no lane {n}"));
            }
            pieces[0].a = frame.tooth[n];
            pieces.last_mut().unwrap().b = frame.land[n];
            let last_layer = pieces.last().unwrap().layer.clone();
            if super::corridor::distance(frame.land[n], frame.bend[n]) > 1e-9 {
                pieces.push(LayerPiece {
                    a: frame.land[n],
                    b: frame.bend[n],
                    layer: last_layer.clone(),
                });
                if ctx.pairs.contains_key(n) && frame.land[n] != frame.bend[n] {
                    chamfer(
                        &mut pieces,
                        pairs::turn_straight_steps(&ctx.cfg) as f64 * ctx.cfg.grid_step,
                    );
                }
            }
            if frame.start[n] != frame.tooth[n] {
                pieces[0].a = frame.start[n];
                pieces.insert(
                    0,
                    LayerPiece {
                        a: frame.tooth[n],
                        b: frame.start[n],
                        layer: pieces[0].layer.clone(),
                    },
                );
                let mut reversed: Vec<_> = pieces
                    .iter()
                    .rev()
                    .map(|p| LayerPiece {
                        a: p.b,
                        b: p.a,
                        layer: p.layer.clone(),
                    })
                    .collect();
                chamfer(
                    &mut reversed,
                    pairs::turn_straight_steps(&ctx.cfg) as f64 * ctx.cfg.grid_step,
                );
                pieces = reversed
                    .into_iter()
                    .rev()
                    .map(|p| LayerPiece {
                        a: p.b,
                        b: p.a,
                        layer: p.layer,
                    })
                    .collect();
            }
            let mut changed = true;
            while changed && pieces.len() >= 2 {
                changed = false;
                for i in 0..pieces.len() - 1 {
                    let a = &pieces[i];
                    let b = &pieces[i + 1];
                    if a.layer != b.layer {
                        continue;
                    }
                    let d1 = super::corridor::sub(a.b, a.a);
                    let d2 = super::corridor::sub(b.b, b.a);
                    let l1 = super::corridor::length(d1);
                    let l2 = super::corridor::length(d2);
                    if l1 < 1e-9
                        || l2 < 1e-9
                        || (super::corridor::dot(d1, d2) / (l1 * l2) < -0.5
                            && l1.min(l2) < ctx.cfg.lane_min)
                    {
                        let merged = LayerPiece {
                            a: a.a,
                            b: b.b,
                            layer: a.layer.clone(),
                        };
                        pieces.splice(i..=i + 1, [merged]);
                        changed = true;
                        break;
                    }
                }
            }
            let mut merged = Vec::<LayerPiece>::new();
            for p in pieces {
                if let Some(last) = merged.last_mut() {
                    if last.layer == p.layer
                        && super::corridor::cross(
                            super::corridor::sub(last.b, last.a),
                            super::corridor::sub(p.b, last.a),
                        )
                        .abs()
                            < 1e-9
                        && super::corridor::distance(last.b, p.a) < 1e-9
                    {
                        last.b = p.b;
                        continue;
                    }
                }
                merged.push(p);
            }
            let mut xy = vec![merged[0].a];
            xy.extend(merged.iter().map(|p| p.b));
            let layers = merged.iter().map(|p| p.layer.clone()).collect();
            let mut vias = Vec::new();
            for (f, lane, u, k) in &solution.vias {
                if lane != n {
                    continue;
                }
                let s = self.frame_s(f, *u, plan);
                let lo = (s / self.grid).floor() as i64;
                let o = match (
                    solution.offsets.get(&(f.clone(), n.clone(), lo)),
                    solution.offsets.get(&(f.clone(), n.clone(), lo + 1)),
                ) {
                    (Some(&a), Some(&b)) => a + (b - a) * (s / self.grid - lo as f64),
                    _ => solution.offsets[&(f.clone(), n.clone(), *k)],
                };
                vias.push(self.spines[f].xy(s, o));
            }
            let mut paired_legs = BTreeMap::new();
            let mut paired_vias = BTreeMap::new();
            if let Some(legs) = ctx.pairs.get(n) {
                let half = pairs::pitch(&ctx.cfg) / 2.;
                let mut sign = pairs::hand_vector(
                    ctx.tooth_dir.get(n).copied(),
                    ctx.pair_ends[n][0][0],
                    ctx.pair_ends[n][0][1],
                    false,
                );
                if sign == 0 {
                    sign = 1;
                }
                for (leg, side) in [
                    (legs[0].clone(), sign as f64),
                    (legs[1].clone(), -sign as f64),
                ] {
                    let points = pairs::offset_polyline(&xy, side * half);
                    let ps = points
                        .windows(2)
                        .zip(merged.iter())
                        .map(|(w, p)| LayerPiece {
                            a: w[0],
                            b: w[1],
                            layer: p.layer.clone(),
                        })
                        .collect();
                    paired_legs.insert(leg.clone(), ps);
                    let mut barrels = Vec::new();
                    for &site in &vias {
                        let points =
                            pairs::dive_barrels(site, &merged, pairs::dive_offset(&ctx.cfg, half));
                        if points.len() != 2 {
                            return Err("AWX pair dive lacks physical incoming direction".into());
                        }
                        barrels.push(points[if side > 0. { 0 } else { 1 }]);
                    }
                    paired_vias.insert(leg, barrels);
                }
            }
            uxy.sort_by(|a, b| {
                a[0].total_cmp(&b[0])
                    .then(a[1].total_cmp(&b[1]))
                    .then(a[2].total_cmp(&b[2]))
            });
            lanes.insert(
                n.clone(),
                LaneGeometry {
                    xy,
                    layers,
                    vias,
                    uxy,
                    paired_legs,
                    paired_vias,
                    snapped: false,
                    ends: Vec::new(),
                    cross: None,
                },
            );
        }
        let mut cuts = super::whole_types::GeometryCuts::default();
        for finding in &solution.paid {
            let Some(p) = self
                .pieces
                .iter()
                .find(|p| p.frame == finding.frame && p.lane == finding.lane)
            else {
                continue;
            };
            if finding.kind == "static" {
                if let Some(label) = &finding.other {
                    if let Some(b) = island_boxes.get(label.trim_start_matches("via ")) {
                        if label.starts_with("via ") {
                            for (f, n, u, k) in &solution.vias {
                                if f == &finding.frame
                                    && n == &finding.lane
                                    && (*k - finding.column).abs() as f64 * self.grid
                                        <= self.via_span[n] + self.grid
                                {
                                    cuts.vcuts.push(super::whole_types::ViaCut {
                                        lane: n.clone(),
                                        u: *u,
                                        w: self.via_span[n],
                                        built: false,
                                    });
                                }
                            }
                        } else {
                            let qs = [
                                [b.bounds[0], b.bounds[1]],
                                [b.bounds[0], b.bounds[3]],
                                [b.bounds[2], b.bounds[1]],
                                [b.bounds[2], b.bounds[3]],
                            ]
                            .map(|q| self.spines[&p.frame].project_pt(q)[0]);
                            let a = qs.iter().copied().fold(f64::INFINITY, f64::min);
                            let z = qs.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                            cuts.cuts.push(super::whole_types::IslandCut {
                                lane: p.lane.clone(),
                                island: label.clone(),
                                u_lo: self.route_u(&p.frame, a, plan),
                                u_hi: self.route_u(&p.frame, z, plan),
                            });
                            let layer = self.layer_at(
                                ctx,
                                plan,
                                &p.lane,
                                self.route_u(&p.frame, finding.column as f64 * self.grid, plan),
                            )?;
                            if let Some(c) = super::feedback::layer_cut(
                                &p.lane,
                                label,
                                b,
                                layer,
                                ctx.layers.len(),
                            ) {
                                cuts.lcuts.push(c);
                            }
                        }
                    }
                }
            } else if matches!(finding.kind.as_str(), "via" | "viavia" | "pdive") {
                for (f, n, u, k) in &solution.vias {
                    if f == &finding.frame && n == &finding.lane && (*k - finding.column).abs() <= 1
                    {
                        cuts.vcuts.push(super::whole_types::ViaCut {
                            lane: n.clone(),
                            u: *u,
                            w: 2. * ctx.cfg.lane_pitch,
                            built: false,
                        });
                    }
                }
            }
        }
        let mut bands = BTreeMap::new();
        for (n, g) in &lanes {
            bands.insert(n.clone(), ctx.band_of(n, g, 0., false)?);
        }
        Ok(GeometryResult {
            lanes,
            paid: solution.paid.clone(),
            flips,
            objective: solution.objective,
            held: BTreeSet::new(),
            folded: Vec::new(),
            failed: false,
            conflicts: Vec::new(),
            changes: self.changes.clone(),
            bands,
            cuts,
            islands,
            island_boxes,
            metadata_complete: true,
        })
    }
}
pub fn generate(
    ctx: &WholeContext,
    frame: &WholeFrame,
    plan: &WholePlan,
    flips: &BTreeSet<(String, String)>,
    budget_ms: u64,
) -> Result<GeometryResult, String> {
    let start = std::time::Instant::now();
    let mut domain = GeometryDomain::new(ctx, frame, plan)?;
    let (objects, islands) = domain.static_objects(ctx, frame);
    let first = domain.build_and_solve(ctx, frame, plan, &[], None, budget_ms)?;
    domain.reanchor(frame, plan, &first)?;
    let sides = domain.static_sides(ctx, frame, plan, &first, &objects, flips)?;
    let left = budget_ms.saturating_sub(start.elapsed().as_millis() as u64);
    let second = domain.build_and_solve(ctx, frame, plan, &sides, Some(&first.offsets), left)?;
    domain.emit(ctx, frame, plan, &second, &objects, islands, flips.clone())
}
