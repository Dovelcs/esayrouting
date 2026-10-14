//! Native orchestration for the source `braid.plan_braid` plan-only path.
//!
//! The stage composes the native setup, corridor plan-only geometry, page
//! schedule, and cross-corridor crossing price without writing board copper.

use super::{
    board_snapshot::BoardSnapshot,
    corridor::Point,
    whole_context::{Ends, RoutingRules},
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Explicit plan-side inputs consumed by `braid.setup(plan=...)`.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BraidPlanSeed {
    pub ends: BTreeMap<String, Ends>,
    pub tooth_layer: BTreeMap<String, String>,
    pub dest_layer: BTreeMap<String, String>,
    pub tooth_dir: BTreeMap<String, Point>,
    pub stub_dir: BTreeMap<String, Point>,
    pub chi: i32,
    pub berth_sig: BTreeMap<String, String>,
}

/// Source `braid.setup` switch. Other planner settings are separate because
/// they affect the corridor or schedule stage rather than setup.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct BraidSetupOptions {
    pub pairs: bool,
}

/// Exact inputs for one plan-only braid judgment. Geometry and frame metadata
/// are explicit so a routed-board approximation cannot silently replace them.
pub struct BraidPlanInput<'a> {
    pub snapshot: &'a BoardSnapshot,
    pub source: &'a str,
    pub destination: &'a str,
    pub names: &'a [String],
    pub seed: &'a BraidPlanSeed,
    pub options: &'a BraidSetupOptions,
    pub rules: &'a RoutingRules,
    pub corridor_options: &'a super::braid_corridor::BraidCorridorOptions,
    /// Fixed-source `BRAID_EXACT_PAGES` value.
    pub exact_pages: bool,
    /// Fixed-source `prices.PAGES` value.
    pub swimmer_page_price: f64,
}

/// Per-net projection of `braid.plan_braid`'s plan-only result.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BraidPlanNet {
    pub corridor: usize,
    pub islands: Vec<String>,
    pub launch_idx: Option<usize>,
    pub target_idx: Option<usize>,
    pub page: Option<String>,
    pub birth_b: bool,
    pub joiner: bool,
    pub side_exit: bool,
    pub exit_leg_layer: Option<String>,
    pub lane: Vec<Point>,
    pub changes: Option<usize>,
    pub swim_changes: Option<usize>,
    pub cross_vias: usize,
    pub mid: Vec<Point>,
    pub req: Vec<(f64, f64, String)>,
    pub bwin: Vec<(f64, f64)>,
    pub s0: Option<f64>,
    pub s1: Option<f64>,
}

/// Native plan-only equivalent of `braid.plan_braid`.
pub fn plan_braid(input: &BraidPlanInput<'_>) -> Result<BTreeMap<String, BraidPlanNet>, String> {
    if input.names.is_empty() {
        return Ok(BTreeMap::new());
    }
    if !input.swimmer_page_price.is_finite() || input.swimmer_page_price < 0. {
        return Err("braid plan swimmer page price must be finite and nonnegative".into());
    }
    if input.options.pairs {
        return Err("source plan_braid calls setup with pairs=false".into());
    }
    if input.corridor_options.pages_first {
        return Err("source braid_plan_of omits pages_first; setup therefore uses false".into());
    }
    if input.exact_pages {
        return Err("source setup resets exact page scheduling when pages_first is false".into());
    }
    let setup = super::braid_setup::prepare(input)?;
    let corridors = super::braid_corridor::build_corridors(
        &setup,
        input.corridor_options,
        false,
        input.swimmer_page_price,
    )?;
    let crossing = super::braid_crossings::cross_corridor_vias(&corridors)?;
    let mut out = BTreeMap::new();
    for corridor in &corridors {
        let schedule = &corridor.schedule;
        for name in &corridor.members {
            let page = schedule.page_of(name).map(str::to_owned);
            let exit_leg_layer = corridor.exit_leg_layer.get(name).cloned();
            let changes = if page.is_some() {
                super::braid_crossings::layer_profile(corridor, name)
                    .map(|profile| profile.len().saturating_sub(1))
            } else {
                None
            };
            let swim_changes = if page.is_none() {
                super::braid_crossings::swim_changes(corridor, name)
            } else {
                None
            };
            let required = corridor.required.get(name).cloned().unwrap_or_default();
            let bwin = corridor.back_windows.get(name).cloned().unwrap_or_default();
            let intervals = required
                .into_iter()
                .map(|i| (i.start, i.end, i.layer))
                .collect();
            let mut output_page = page;
            let mut output_leg = exit_leg_layer;
            if setup.mirror_axis_y.is_some() {
                output_page = output_page.map(mirror_layer);
                output_leg = output_leg.map(mirror_layer);
            }
            let plan = BraidPlanNet {
                corridor: corridor.index,
                islands: corridor.islands.get(name).cloned().unwrap_or_default(),
                launch_idx: corridor.launch.iter().position(|n| n == name),
                target_idx: corridor.target.iter().position(|n| n == name),
                page: output_page,
                birth_b: schedule.birth_b.contains(name),
                joiner: corridor.joiners.contains(name),
                side_exit: corridor.siders.contains(name),
                exit_leg_layer: output_leg,
                lane: corridor.lanes.get(name).cloned().unwrap_or_default(),
                changes,
                swim_changes,
                cross_vias: crossing.get(name).copied().unwrap_or(0),
                mid: corridor.mid.get(name).cloned().unwrap_or_default(),
                req: intervals,
                bwin,
                s0: Some(corridor.s0),
                s1: Some(corridor.s1),
            };
            if out.insert(name.clone(), plan).is_some() {
                return Err(format!("braid plan emitted lane {name} more than once"));
            }
        }
    }
    for name in input.names {
        if !out.contains_key(name) {
            return Err(format!("braid plan omitted requested lane {name}"));
        }
    }
    Ok(out)
}

fn mirror_layer(layer: String) -> String {
    if let Some(rest) = layer.strip_prefix("F.") {
        format!("B.{rest}")
    } else if let Some(rest) = layer.strip_prefix("B.") {
        format!("F.{rest}")
    } else {
        layer
    }
}
