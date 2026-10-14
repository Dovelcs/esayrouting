//! Typed state exchanged by the native replan stages.
use super::{
    escape_moves::Move,
    joint_escape::{JointInput, JointMenus, JointPlan},
    route_evolution::RouteGrade,
    whole_context::RoutingRules,
};
use serde::{Deserialize, Serialize};
use std::{collections::BTreeMap, time::Instant};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct EndState {
    /// Endpoint of the source tooth in board millimetres.
    pub tooth: [f64; 2],
    /// Endpoint of the destination tooth/berth in board millimetres.
    pub exit: [f64; 2],
    pub tooth_layer: String,
    pub dest_layer: String,
    /// Exact current menu choice when the seed sidecar carries one.
    pub src_move: Option<Move>,
    pub dst_move: Option<Move>,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct NetVerdict {
    /// `F`, `B`, or `swim` when the braid evidence provides a page class.
    pub class: Option<String>,
    /// B.model per-net prediction from an authoritative source, when retained.
    pub predicted_vias: Option<f64>,
    /// B.model scheduled page; `None` with `predicted_vias` present means swimmer.
    pub model_page: Option<String>,
    /// Actual lane vias, excluding the fanout/tooth vias already present on F.
    pub lane_vias: usize,
    pub actual_vias: usize,
    pub refused: bool,
    pub walled_at: Option<String>,
    pub launch_rank: Option<usize>,
    pub target_rank: Option<usize>,
    pub last_call: Option<usize>,
    pub rip: Vec<String>,
    pub frontier: Vec<(String, usize)>,
    pub mincut: Vec<String>,
    /// Ordered braid blocker census, including any carried prior census.
    pub census: Vec<String>,
}

#[derive(Clone, Debug)]
pub struct AcceptedMove {
    pub net: String,
    pub src_got: Option<Move>,
    pub dst_got: Option<Move>,
    pub comove_got: BTreeMap<String, Move>,
}

#[derive(Clone, Debug)]
pub struct ReplanVerdict {
    pub grade: RouteGrade,
    pub nets: BTreeMap<String, NetVerdict>,
    pub predicted: BTreeMap<String, f64>,
    pub residual: BTreeMap<String, f64>,
    pub worst: Vec<String>,
    pub gatekeepers: Vec<String>,
}

/// Exact inputs and solved end plans for one physical fanout pair. Probe stages
/// rebuild this after an accepted move so menus and plans always match the board.
#[derive(Clone)]
pub struct ProbeContext {
    pub source_part: String,
    pub destination_part: String,
    pub groups: Vec<String>,
    pub chi: i32,
    pub source_input: JointInput,
    pub destination_input: JointInput,
    pub source_menus: JointMenus,
    pub destination_menus: JointMenus,
    pub source_plan: JointPlan,
    pub destination_plan: JointPlan,
    pub rules: RoutingRules,
    pub deadline: Instant,
    /// Number of physically routed candidates to retain per end.
    pub probe_attempts: usize,
    /// Source replan's rescue/last-call budget multiplier.
    pub probe_budget_x: f64,
    /// Source BRAID_LADDER mode (`open` by default).
    pub probe_ladder: String,
    /// Source PROBE_SMOOTH setting (`0` by default).
    pub probe_smooth: bool,
    /// Coupled re-lay selection from the source CLI (currently `census`).
    pub coupled: String,
    /// Source grade implementation selector (`sub` by default).
    pub grade_mode: String,
    /// Source BRAID_EXACT_PAGES switch (off by default).
    pub exact_pages: bool,
    /// Source BRAID_SWIM_W / prices.SWIM_PRICE (2 vias by default).
    pub swimmer_page_price: f64,
    /// `prices.JUDGE` / SWIM_VIAS, defaulting to SWIM_PRICE (2).
    pub swim_vias: f64,
    /// Source PLAN_JUDGE selector: unset, count, flat, or ends.
    pub plan_judge: String,
}
