//! Typed boundary for the complete AWX whole_solve model at caad45cd252f9a50f1fb4b3b15461bdf1b9457a1.
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
pub type Point = [f64; 2];
pub type Bounds = [f64; 4];
pub use crate::awx::{whole_context::WholeContext, whole_frame::WholeFrame};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WholeRules {
    pub track: f64,
    pub clearance: f64,
    pub via_size: f64,
    pub via_need: f64,
    pub lane_pitch: f64,
    pub grid_step: f64,
    pub via_drill: f64,
    pub hole_to_hole_clearance: f64,
    pub board_edge_clearance: f64,
    pub fan_clearance: f64,
    pub pair_pitch: f64,
    pub pair_turn_straight_steps: i64,
    pub pair_diagonal_via_straight: f64,
    pub pair_diagonal_crossover_run: f64,
    pub pair_end_connector: BTreeMap<String, [f64; 2]>,
    pub pair_dive_room: BTreeMap<String, [f64; 2]>,
    pub pair_crossover_room: BTreeMap<String, [f64; 2]>,
    pub pair_dive_offset: BTreeMap<String, f64>,
}
impl WholeRules {
    pub fn from_context(ctx: &WholeContext) -> Result<Self, String> {
        ctx.cfg.validate()?;
        let cfg = &ctx.cfg;
        let diagonal = [0.5_f64.sqrt(); 2];
        let dive = super::pairs::via_straight(cfg, diagonal) + cfg.grid_step;
        let xo = super::pairs::crossover_shape(cfg, diagonal, 1)
            .map(|shape| shape.1[0].max(shape.1[1]) + cfg.grid_step)
            .unwrap_or(dive);
        let mut rules = Self {
            track: cfg.track_width,
            clearance: cfg.clearance,
            via_size: cfg.via_size,
            via_need: cfg.via_size / 2.0 + cfg.clearance + cfg.track_width / 2.0 + 0.03,
            lane_pitch: cfg.lane_min,
            grid_step: cfg.grid_step,
            via_drill: cfg.via_drill,
            hole_to_hole_clearance: cfg.hole_to_hole_clearance,
            board_edge_clearance: cfg.board_edge_clearance,
            fan_clearance: cfg.fan_clearance,
            pair_pitch: super::pairs::pitch(cfg),
            pair_turn_straight_steps: super::pairs::turn_straight_steps(cfg),
            pair_diagonal_via_straight: dive,
            pair_diagonal_crossover_run: xo,
            pair_end_connector: BTreeMap::new(),
            pair_dive_room: BTreeMap::new(),
            pair_crossover_room: BTreeMap::new(),
            pair_dive_offset: BTreeMap::new(),
        };
        for n in &ctx.lanes {
            if !ctx.pairs.contains_key(n) {
                continue;
            }
            let tips = ctx
                .pair_ends
                .get(n)
                .ok_or("AWX pair terminal tips missing")?;
            let dirs = [
                ctx.tooth_dir.get(n).copied().unwrap_or([1.0, 0.0]),
                ctx.stub_dir.get(n).copied().unwrap_or([1.0, 0.0]),
            ];
            rules.pair_end_connector.insert(
                n.clone(),
                [
                    super::pairs::end_connector(cfg, tips[0]),
                    super::pairs::end_connector(cfg, tips[1]),
                ],
            );
            rules.pair_dive_room.insert(
                n.clone(),
                [
                    super::pairs::dive_room(cfg, tips[0], Some(dirs[0])),
                    super::pairs::dive_room(cfg, tips[1], Some(dirs[1])),
                ],
            );
            rules.pair_crossover_room.insert(
                n.clone(),
                [
                    super::pairs::crossover_room(cfg, tips[0], Some(dirs[0]), 0),
                    super::pairs::crossover_room(cfg, tips[1], Some(dirs[1]), 1),
                ],
            );
            rules.pair_dive_offset.insert(
                n.clone(),
                super::pairs::dive_offset(cfg, rules.pair_pitch / 2.0),
            );
        }
        Ok(rules)
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct IslandCut {
    pub island: String,
    pub lane: String,
    pub u_lo: f64,
    pub u_hi: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ViaCut {
    pub lane: String,
    pub u: f64,
    pub w: f64,
    pub built: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LayerCut {
    pub lane: String,
    pub island: String,
    pub layer: usize,
    pub blocked: Option<Vec<i64>>,
    #[serde(rename = "box")]
    pub bounds: Bounds,
    pub span: Option<[f64; 2]>,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct GeometryCuts {
    pub cuts: Vec<IslandCut>,
    pub vcuts: Vec<ViaCut>,
    pub lcuts: Vec<LayerCut>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HotFinding {
    pub point: Point,
    pub kind: String,
    pub lanes: Vec<String>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HistoryAudit {
    pub hot: Vec<HotFinding>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WholeOptions {
    pub routing_layers: Vec<String>,
    pub via_ends: bool,
    pub solve_gates: bool,
    pub firm_cuts: bool,
    pub allow_unproved: bool,
    pub deterministic_batches: i32,
    pub workers: i32,
    pub stall_model_reductions: usize,
    pub warm_deterministic_time: f64,
    pub primary_subsolvers: Vec<String>,
    pub fallback_subsolvers: Vec<String>,
}
impl Default for WholeOptions {
    fn default() -> Self {
        Self {
            routing_layers: vec!["F.Cu".into(), "B.Cu".into()],
            via_ends: true,
            solve_gates: true,
            firm_cuts: false,
            allow_unproved: false,
            deterministic_batches: 100,
            workers: 4,
            stall_model_reductions: 3,
            warm_deterministic_time: 30.0,
            primary_subsolvers: vec![
                "default_lp".into(),
                "max_lp".into(),
                "core".into(),
                "objective_lb_search".into(),
            ],
            fallback_subsolvers: vec!["quick_restart".into(), "no_lp".into(), "core".into()],
        }
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WholeProblem {
    pub context: WholeContext,
    pub frame: WholeFrame,
    pub rules: WholeRules,
    pub cuts: Vec<GeometryCuts>,
    pub soft_cuts: Vec<GeometryCuts>,
    pub history: Vec<HistoryAudit>,
    pub hint: Option<WholePlan>,
    pub options: WholeOptions,
    #[serde(default)]
    pub blocked_fronts: BTreeMap<String, BTreeMap<usize, [f64; 4]>>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Crossing {
    pub u: f64,
    pub ring: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RootProof {
    pub over: i64,
    pub vias: i64,
    pub sig: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct UnprovedPlan {
    pub best: f64,
    pub bound: f64,
    pub over: i64,
    pub bound_over: i64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WholePlan {
    #[serde(rename = "H0")]
    pub arrival: f64,
    #[serde(rename = "Hk")]
    pub handoff: BTreeMap<String, f64>,
    pub rs: BTreeMap<String, f64>,
    pub cross: BTreeMap<String, Crossing>,
    pub changes: BTreeMap<String, Vec<f64>>,
    pub entry: BTreeMap<String, f64>,
    pub end: BTreeMap<String, f64>,
    pub tend: BTreeMap<String, f64>,
    pub branch: BTreeMap<String, String>,
    #[serde(rename = "final")]
    pub final_order: Vec<String>,
    pub launch: Vec<String>,
    pub layers: BTreeMap<String, Vec<String>>,
    pub root: Option<RootProof>,
    pub proved: bool,
    pub gate_over: Vec<serde_json::Value>,
    pub firm_broken: Vec<serde_json::Value>,
    pub soft_broken: Vec<serde_json::Value>,
    pub over_nets: BTreeMap<String, i64>,
    pub unproved: Option<UnprovedPlan>,
    #[serde(default)]
    pub crossover_lanes: BTreeSet<String>,
    #[serde(default)]
    pub movers: BTreeMap<String, String>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum WholeOutcome {
    Plan(WholePlan),
    NoProvedPlan {
        status: String,
        best: Option<f64>,
        bound: Option<f64>,
    },
}
