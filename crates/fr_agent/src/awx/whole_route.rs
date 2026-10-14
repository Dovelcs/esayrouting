//! In-process Rust port of AWX whole_route.py loop (caad45cd252f9a50f1fb4b3b15461bdf1b9457a1).
use super::{
    corridor::Point,
    feedback,
    joint_geometry::JointGeometry,
    whole_audit::{self, AuditReport},
    whole_geometry::{self, GeometryResult},
    whole_polish, whole_snap, whole_solve,
    whole_types::{GeometryCuts, HistoryAudit, HotFinding, WholeOutcome, WholePlan, WholeProblem},
};
use serde::{Deserialize, Serialize};
use std::{
    collections::{BTreeMap, BTreeSet},
    time::{Duration, Instant},
};
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WholeLoopOutcome {
    pub plan: Option<WholePlan>,
    pub geometry: Option<GeometryResult>,
    pub rounds: usize,
    pub reasons: Vec<String>,
    pub history: Vec<HistoryAudit>,
    pub cuts: Vec<GeometryCuts>,
    pub soft_cuts: Vec<GeometryCuts>,
    pub flips: BTreeSet<(String, String)>,
    pub converged: bool,
    pub ends_crowded: bool,
    pub stopped_not_converging: bool,
    pub best_round: usize,
    pub best_score: Option<usize>,
    pub budget_exhausted: bool,
    /// Wall-clock duration of this whole-route invocation, measured through outcome assembly.
    #[serde(default)]
    pub elapsed_us: u64,
}
#[derive(Clone, Debug)]
pub struct LoopOptions {
    pub rounds: usize,
    pub budget_ms: u64,
    pub patience: usize,
    pub seed_flips: BTreeSet<(String, String)>,
}
#[derive(Clone)]
struct CutBatch {
    cuts: GeometryCuts,
    given: BTreeSet<(String, String)>,
}
struct LoopState<'a, G: JointGeometry + ?Sized> {
    problem: WholeProblem,
    oracle: &'a G,
    deadline: Instant,
    options: LoopOptions,
    plan: Option<WholePlan>,
    last: Option<GeometryResult>,
    best: Option<(WholePlan, GeometryResult)>,
    best_score: Option<usize>,
    best_round: usize,
    stall: usize,
    round: usize,
    flips: BTreeSet<(String, String)>,
    cuts: Vec<CutBatch>,
    history: Vec<HistoryAudit>,
    soft: Vec<GeometryCuts>,
    reasons: Vec<String>,
    previous_end_hot: BTreeSet<(String, Vec<String>, usize)>,
    ends_crowded: bool,
    not_converging: bool,
    converged: bool,
    geometry_cache: BTreeMap<String, GeometryResult>,
    polish_cache: BTreeMap<String, GeometryResult>,
    snap_cache: BTreeMap<String, GeometryResult>,
}
fn stable<T: Serialize>(value: &T) -> Result<String, String> {
    serde_json::to_string(value).map_err(|e| e.to_string())
}
fn dedup<T: Serialize + Clone>(rows: impl IntoIterator<Item = T>) -> Result<Vec<T>, String> {
    let mut seen = BTreeSet::new();
    let mut out = vec![];
    for row in rows {
        if seen.insert(stable(&row)?) {
            out.push(row)
        }
    }
    Ok(out)
}
fn hot(report: &AuditReport) -> HistoryAudit {
    HistoryAudit {
        hot: report
            .findings
            .iter()
            .map(|f| HotFinding {
                point: f.point,
                kind: f.kind.to_uppercase(),
                lanes: f.lanes.clone(),
            })
            .collect(),
    }
}
fn finding_score(report: &AuditReport) -> usize {
    if !report.complete {
        return 999;
    }
    let mut score = 0;
    for kind in [
        "dive", "static", "shape", "swim", "pitch", "bands", "missing",
    ] {
        score += report.counts.get(kind).copied().unwrap_or(0)
    }
    score + usize::from(report.band_outside_mm > 0.)
}
impl<'a, G: JointGeometry + ?Sized> LoopState<'a, G> {
    fn remaining(&self) -> Result<u64, String> {
        let d = self.deadline.saturating_duration_since(Instant::now());
        if d.is_zero() {
            return Err("AWX loop budget exhausted".into());
        }
        Ok(d.as_millis().max(1).min(u64::MAX as u128) as u64)
    }
    fn geometry(&mut self) -> Result<GeometryResult, String> {
        let plan = self.plan.as_ref().ok_or("AWX loop without a solved plan")?;
        let key = stable(&(plan, &self.flips))?;
        if let Some(g) = self.geometry_cache.get(&key) {
            return Ok(g.clone());
        }
        let g = whole_geometry::generate(
            &self.problem.context,
            &self.problem.frame,
            plan,
            &self.flips,
            self.remaining()?,
        )?;
        self.geometry_cache.insert(key, g.clone());
        Ok(g)
    }
    fn polish(&mut self, g: &GeometryResult) -> Result<GeometryResult, String> {
        let key = stable(g)?;
        if let Some(p) = self.polish_cache.get(&key) {
            return Ok(p.clone());
        }
        let p = whole_polish::polish(
            &self.problem.context,
            &self.problem.frame,
            g,
            self.remaining()?,
        )?;
        self.polish_cache.insert(key, p.clone());
        Ok(p)
    }
    fn snap(
        &mut self,
        g: &GeometryResult,
        pairs_only: bool,
        keep: Option<&HistoryAudit>,
    ) -> Result<GeometryResult, String> {
        let key = stable(&(g, pairs_only, keep))?;
        if let Some(s) = self.snap_cache.get(&key) {
            return Ok(s.clone());
        }
        let s = whole_snap::snap(
            &self.problem.context,
            &self.problem.frame,
            g,
            self.oracle,
            pairs_only,
            keep,
            self.remaining()?,
        )?;
        self.snap_cache.insert(key, s.clone());
        Ok(s)
    }
    fn audit(&self, g: &GeometryResult) -> Result<AuditReport, String> {
        whole_audit::audit(&self.problem.context, g)
    }
    fn addhot(&mut self, report: &AuditReport) -> bool {
        let hot = hot(report);
        if hot.hot.is_empty() {
            return false;
        }
        self.history.push(hot);
        true
    }
    fn progress(&mut self, score: usize, g: &GeometryResult, fresh: bool) -> bool {
        if self.best_score.is_none_or(|best| score < best) {
            self.best_score = Some(score);
            self.best_round = self.round;
            self.stall = 0;
            self.best = self.plan.clone().map(|p| (p, g.clone()));
        } else if !fresh {
            self.stall += 1
        }
        if self.stall >= self.options.patience {
            self.not_converging = true;
            self.reasons.push(format!(
                "round {}: not converging; score {score}, best {:?} at round {}",
                self.round, self.best_score, self.best_round
            ));
            return false;
        }
        true
    }
    fn own_spans(
        &self,
        cuts: &GeometryCuts,
        geo: &GeometryResult,
        laid: Option<&GeometryResult>,
    ) -> Result<GeometryCuts, String> {
        let mut lanes = geo.lanes.clone();
        if let Some(laid) = laid {
            for (n, l) in &laid.lanes {
                let mut l = l.clone();
                l.uxy = geo.lanes.get(n).map(|g| g.uxy.clone()).unwrap_or_default();
                lanes.insert(n.clone(), l);
            }
        }
        let cfg = &self.problem.context.cfg;
        feedback::own_spans(
            cuts,
            &lanes,
            self.problem.options.routing_layers.len(),
            cfg.track_width,
            cfg.clearance,
            cfg.grid_step,
        )
    }
    fn audit_layer_cuts(
        &self,
        report: &AuditReport,
        geo: &GeometryResult,
    ) -> Result<Vec<super::whole_types::LayerCut>, String> {
        let nl = self.problem.options.routing_layers.len();
        let mut out = vec![];
        for f in &report.findings {
            if f.kind.to_uppercase() != "STATIC" {
                continue;
            }
            let (Some(layer), Some(pad)) = (&f.layer, &f.obstacle) else {
                continue;
            };
            let Some(li) = self
                .problem
                .options
                .routing_layers
                .iter()
                .position(|l| l == layer)
            else {
                continue;
            };
            for lane in &f.lanes {
                if let Some(id) = geo.islands.get(pad) {
                    if let Some(b) = geo.island_boxes.get(id) {
                        if let Some(c) = feedback::layer_cut(lane, id, b, li, nl) {
                            out.push(c)
                        }
                    }
                } else if nl > 2 {
                    let r = 0.5_f64.max(3. * f.required_clearance);
                    out.push(super::whole_types::LayerCut {
                        lane: lane.clone(),
                        island: pad.clone(),
                        layer: 0,
                        blocked: Some(vec![li as i64]),
                        bounds: [
                            f.point[0] - r,
                            f.point[1] - r,
                            f.point[0] + r,
                            f.point[1] + r,
                        ],
                        span: None,
                    });
                }
            }
        }
        dedup(out)
    }
    fn smooth_cuts(
        &self,
        geo: &GeometryResult,
        polished: &GeometryResult,
        report: &AuditReport,
        new: &BTreeSet<(String, String)>,
    ) -> Result<GeometryCuts, String> {
        let mut cuts = GeometryCuts {
            cuts: geo
                .cuts
                .cuts
                .iter()
                .filter(|c| !new.contains(&(c.lane.clone(), c.island.clone())))
                .cloned()
                .collect(),
            vcuts: geo
                .cuts
                .vcuts
                .iter()
                .chain(&polished.cuts.vcuts)
                .cloned()
                .collect(),
            lcuts: self
                .audit_layer_cuts(report, geo)?
                .into_iter()
                .filter(|c| {
                    self.problem.options.routing_layers.len() > 2
                        || !new.contains(&(c.lane.clone(), c.island.clone()))
                })
                .collect(),
        };
        cuts.lcuts
            .extend(geo.cuts.lcuts.iter().chain(&polished.cuts.lcuts).cloned());
        cuts.lcuts = dedup(cuts.lcuts)?;
        self.own_spans(&cuts, geo, Some(polished))
    }
    fn compiled_cuts(&self) -> Result<GeometryCuts, String> {
        let mut all = GeometryCuts::default();
        for batch in &self.cuts {
            all.cuts.extend(
                batch
                    .cuts
                    .cuts
                    .iter()
                    .filter(|c| {
                        let key = (c.lane.clone(), c.island.clone());
                        !self.flips.contains(&key) || batch.given.contains(&key)
                    })
                    .cloned(),
            );
            all.vcuts.extend(batch.cuts.vcuts.clone());
            all.lcuts.extend(batch.cuts.lcuts.clone());
        }
        all.lcuts = dedup(all.lcuts)?;
        Ok(all)
    }
    fn resolve(&mut self) -> Result<bool, String> {
        self.remaining()?;
        let cuts = self.compiled_cuts()?;
        let mut p = self.problem.clone();
        p.hint = self.plan.clone();
        p.history = self.history.clone();
        p.soft_cuts = self.soft.clone();
        p.cuts = vec![cuts.clone()];
        if p.options.routing_layers.len() > 2 {
            p.options.firm_cuts = true;
            p.options.allow_unproved = true;
            return self.accept(whole_solve::solve_until(&p, Some(self.deadline))?);
        }
        match whole_solve::solve_until(&p, Some(self.deadline))? {
            WholeOutcome::Plan(plan) => {
                self.plan = Some(plan);
                Ok(true)
            }
            WholeOutcome::NoProvedPlan { status, .. } => {
                self.reasons
                    .push(format!("resolve without proved plan: {status}"));
                self.remaining()?;
                p.options.allow_unproved = true;
                if self.cuts.is_empty() {
                    return self.accept(whole_solve::solve_until(&p, Some(self.deadline))?);
                }
                self.soft.push(cuts);
                p.soft_cuts = self.soft.clone();
                p.cuts.clear();
                let out = whole_solve::solve_until(&p, Some(self.deadline))?;
                let accepted = self.accept(out)?;
                if accepted {
                    self.cuts.clear()
                }
                Ok(accepted)
            }
        }
    }
    fn accept(&mut self, out: WholeOutcome) -> Result<bool, String> {
        match out {
            WholeOutcome::Plan(p) => {
                self.plan = Some(p);
                Ok(true)
            }
            WholeOutcome::NoProvedPlan {
                status,
                best,
                bound,
            } => {
                self.reasons.push(format!(
                    "solver ended {status}; best {best:?}, bound {bound:?}"
                ));
                Ok(false)
            }
        }
    }
    fn at_ends(&self, h: &HistoryAudit) -> BTreeSet<(String, Vec<String>, usize)> {
        let ctx = &self.problem.context;
        let mut result = BTreeSet::new();
        for end in 0..2 {
            let pts: Vec<_> = ctx
                .ends
                .values()
                .map(|e| if end == 0 { e.tooth } else { e.berth })
                .collect();
            if pts.is_empty() {
                continue;
            }
            let b = [
                pts.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min),
                pts.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min),
                pts.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max),
                pts.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max),
            ];
            for f in &h.hot {
                let mut lanes: Vec<_> = f
                    .lanes
                    .iter()
                    .map(|n| {
                        ctx.pairs
                            .iter()
                            .find(|(_, p)| p.contains(n))
                            .map(|(alias, _)| alias.clone())
                            .unwrap_or_else(|| n.clone())
                    })
                    .filter(|n| ctx.ends.contains_key(n) || ctx.pairs.contains_key(n))
                    .collect();
                lanes.sort();
                lanes.dedup();
                if lanes.is_empty() {
                    continue;
                }
                let dx = (b[0] - f.point[0]).max(0.).max(f.point[0] - b[2]);
                let dy = (b[1] - f.point[1]).max(0.).max(f.point[1] - b[3]);
                let near = if dx == 0. && dy > 0. {
                    let y = if f.point[1] < b[1] { b[1] } else { b[3] };
                    let count = pts.iter().filter(|p| (p[1] - y).abs() < 1e-3).count();
                    dy <= (count + 2) as f64 * ctx.cfg.lane_min
                } else {
                    dx.hypot(dy) <= 2. * self.problem.rules.via_need + ctx.cfg.lane_min
                };
                if near {
                    result.insert((f.kind.clone(), lanes, end));
                }
            }
        }
        result
    }
    fn crowded(&mut self, report: &AuditReport, answered: bool) -> bool {
        let now = self.at_ends(&hot(report));
        if !answered
            && (now.iter().any(|f| f.0 == "PITCH" || f.0 == "STATIC")
                || self.round > 1 && now.iter().any(|f| self.previous_end_hot.contains(f)))
        {
            self.ends_crowded = true;
            self.reasons
                .push("findings at the ends require the fanout to change".into());
            return true;
        }
        self.previous_end_hot = now;
        false
    }
    fn add_cut_batch(&mut self, cuts: GeometryCuts, given: &BTreeSet<(String, String)>) {
        self.cuts.push(CutBatch {
            cuts,
            given: given.clone(),
        })
    }
    fn smooth_passes(&mut self, geo: &GeometryResult, p: &GeometryResult) -> Result<bool, String> {
        let pairs = self.snap(p, true, None)?;
        self.last = Some(pairs.clone());
        if pairs.failed {
            let cuts = GeometryCuts {
                cuts: vec![],
                lcuts: vec![],
                vcuts: pairs
                    .cuts
                    .vcuts
                    .iter()
                    .filter(|c| {
                        !p.cuts
                            .vcuts
                            .iter()
                            .any(|old| stable(old).ok() == stable(c).ok())
                    })
                    .cloned()
                    .collect(),
            };
            if cuts.vcuts.is_empty() {
                self.reasons
                    .push("a pair cannot be laid and has no dive to move".into());
                return Ok(false);
            }
            if !self.progress(1500, &pairs, false) {
                return Ok(false);
            }
            let cuts = self.own_spans(&cuts, geo, None)?;
            self.add_cut_batch(cuts, &self.flips.clone());
            return self.resolve();
        }
        let q = self.polish(&pairs)?;
        let qr = self.audit(&q)?;
        let mut chosen = q.clone();
        let mut qf = q.clone();
        let mut fit = whole_audit::gate(&q, &qr).passed;
        if !fit {
            let keep = hot(&qr);
            let pk = self.snap(p, true, Some(&keep))?;
            if !pk.failed {
                let qk = self.polish(&pk)?;
                let kr = self.audit(&qk)?;
                qf = qk.clone();
                if whole_audit::gate(&qk, &kr).passed {
                    chosen = qk;
                    fit = true
                }
            }
        }
        if !fit {
            let new: BTreeSet<_> = qf.flips.difference(&self.flips).cloned().collect();
            self.flips.extend(qf.flips);
            if !self.progress(1000 + finding_score(&qr), &q, !new.is_empty()) {
                return Ok(false);
            }
            if !self.addhot(&qr) {
                self.reasons
                    .push("the singles do not fit round the pairs, without priced places".into());
                return Ok(false);
            }
            return self.resolve();
        }
        let snapped = self.snap(&chosen, false, None)?;
        let report = self.audit(&snapped)?;
        self.last = Some(snapped.clone());
        // whole_lint is part of the geometric audit interface; its separate result is merged below.
        let lint = whole_audit::lint(&self.problem.context, &snapped)?;
        if whole_audit::gate(&snapped, &report).passed && whole_audit::gate(&snapped, &lint).passed
        {
            self.converged = true;
            self.best = self.plan.clone().map(|plan| (plan, snapped));
            self.best_round = self.round;
            self.best_score = Some(0);
            return Ok(false);
        }
        let folded = lint.findings.len();
        if !self.progress(finding_score(&report) + folded, &snapped, false) {
            return Ok(false);
        }
        let mut found = report;
        found.findings.extend(lint.findings);
        if !self.addhot(&found) {
            self.reasons
                .push("the snapped plan does not pass, without priced places".into());
            return Ok(false);
        }
        let lc: Vec<_> = snapped
            .cuts
            .lcuts
            .iter()
            .filter(|c| {
                !chosen
                    .cuts
                    .lcuts
                    .iter()
                    .any(|old| stable(old).ok() == stable(c).ok())
            })
            .cloned()
            .collect();
        if !lc.is_empty() {
            let c = self.own_spans(
                &GeometryCuts {
                    cuts: vec![],
                    vcuts: vec![],
                    lcuts: dedup(lc)?,
                },
                geo,
                Some(&chosen),
            )?;
            self.add_cut_batch(c, &self.flips.clone())
        }
        self.resolve()
    }
    fn body(&mut self) -> Result<(), String> {
        for round in 1..=self.options.rounds {
            self.round = round;
            self.remaining()?;
            let given = self.flips.clone();
            let geo = self.geometry()?;
            let p = self.polish(&geo)?;
            let report = self.audit(&p)?;
            let passes = whole_audit::gate(&p, &report).passed;
            self.last = Some(p.clone());
            let new: BTreeSet<_> = p.flips.difference(&given).cloned().collect();
            let cuts = self.smooth_cuts(&geo, &p, &report, &new)?;
            let mut n = cuts.cuts.len() + cuts.vcuts.len() + cuts.lcuts.len();
            if !passes && self.addhot(&report) {
                n += 1
            }
            let answered = !new.is_empty()
                || self.problem.options.routing_layers.len() > 2 && !cuts.lcuts.is_empty();
            if !passes && self.crowded(&report, answered) {
                break;
            }
            if !new.is_empty() {
                if !self.progress(2000 + finding_score(&report), &p, true) {
                    break;
                }
                self.flips.extend(p.flips);
                if n == 0 {
                    continue;
                }
                self.add_cut_batch(cuts, &given);
                if !self.resolve()? {
                    break;
                }
                continue;
            }
            if passes {
                if self.smooth_passes(&geo, &p)? {
                    continue;
                }
                break;
            }
            if !self.progress(2000 + finding_score(&report), &p, false) {
                break;
            }
            if n == 0 {
                self.reasons
                    .push("no flips, cuts or findings with a place left".into());
                break;
            }
            self.add_cut_batch(cuts, &given);
            if !self.resolve()? {
                break;
            }
        }
        Ok(())
    }
}
pub fn run<G: JointGeometry + ?Sized>(
    problem: &WholeProblem,
    oracle: &G,
    rounds: usize,
    budget_ms: u64,
) -> Result<WholeLoopOutcome, String> {
    run_with_options(
        problem,
        oracle,
        LoopOptions {
            rounds,
            budget_ms,
            patience: 2,
            seed_flips: BTreeSet::new(),
        },
    )
}
pub fn run_with_options<G: JointGeometry + ?Sized>(
    problem: &WholeProblem,
    oracle: &G,
    options: LoopOptions,
) -> Result<WholeLoopOutcome, String> {
    let started = Instant::now();
    if options.rounds == 0 || options.patience == 0 {
        return Err("AWX loop requires positive rounds and patience".into());
    }
    let deadline = Instant::now()
        .checked_add(Duration::from_millis(options.budget_ms))
        .ok_or("AWX loop deadline overflow")?;
    let flips = options.seed_flips.clone();
    let cuts = problem
        .cuts
        .iter()
        .cloned()
        .map(|cuts| CutBatch {
            cuts,
            given: flips.clone(),
        })
        .collect();
    let mut state = LoopState {
        problem: problem.clone(),
        oracle,
        deadline,
        options,
        plan: None,
        last: None,
        best: None,
        best_score: None,
        best_round: 0,
        stall: 0,
        round: 0,
        flips,
        cuts,
        history: problem.history.clone(),
        soft: problem.soft_cuts.clone(),
        reasons: vec![],
        previous_end_hot: BTreeSet::new(),
        ends_crowded: false,
        not_converging: false,
        converged: false,
        geometry_cache: BTreeMap::new(),
        polish_cache: BTreeMap::new(),
        snap_cache: BTreeMap::new(),
    };
    let first = whole_solve::solve_until(problem, Some(deadline))?;
    if state.accept(first)? {
        if let Err(e) = state.body() {
            if Instant::now() >= deadline {
                state.reasons.push(e)
            } else {
                return Err(e);
            }
        }
    }
    let (plan, geometry) = if let Some((p, g)) = state.best {
        (Some(p), Some(g))
    } else {
        (state.plan, state.last)
    };
    Ok(WholeLoopOutcome {
        plan,
        geometry,
        rounds: state.round,
        reasons: state.reasons,
        history: state.history,
        cuts: state.cuts.into_iter().map(|b| b.cuts).collect(),
        soft_cuts: state.soft,
        flips: state.flips,
        converged: state.converged,
        ends_crowded: state.ends_crowded,
        stopped_not_converging: state.not_converging,
        best_round: state.best_round,
        best_score: state.best_score,
        budget_exhausted: Instant::now() >= deadline,
        elapsed_us: started.elapsed().as_micros().min(u64::MAX as u128) as u64,
    })
}
