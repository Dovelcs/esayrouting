//! Source-compatible rung policy for `braid.Corridor.connect_ladder`.
//!
//! This module owns the rung order and the request passed to the native point
//! router. Corridor-specific band construction is supplied by a source-state
//! adapter; a generic tube around the lane is not equivalent to `band_of`.

use super::{
    braid_corridor::{CorridorBandContext, CorridorPlan},
    corridor::Point,
    native_grid_window::FreeViaPad,
    pair_connection::{PairGivenCrossover, PairGivenEnd},
    whole_context::{ContextPad, CopperShape, CopperVia, ReservationTube, RoutingRules},
};
use crate::board_adapter::BoardAdapter;
use fr_engine::ids::NetNo;
use std::collections::BTreeMap;
use std::time::Instant;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LadderMode {
    /// Source `BRAID_LADDER=full`: all five rungs in order.
    Full,
    /// Source `BRAID_LADDER=open`: exact suffix starting at rung three.
    Open,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RungLabel {
    Slack03,
    Slack08,
    Slack16Open,
    FreeWindow4,
    Free6,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RungSpec {
    pub label: RungLabel,
    /// Slack passed to the source per-net `band_of` function.
    pub slack: f64,
    /// The third rung is the first to permit both routing layers.
    pub open_layers: bool,
    pub margin: f64,
    /// The fifth rung deliberately drops `window_pts`; the fourth keeps it.
    pub use_window: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RungEndpoint {
    pub point: Point,
    pub layer: String,
}

/// One planned pair connector candidate `(point, direction)` from the source
/// pair-end geometry.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PairConnector {
    pub point: Point,
    pub direction: Point,
}

/// Additional endpoints and constraints used by the source `connect_pair`
/// branch. The two point arrays are ordered P then N.
#[derive(Clone, Debug, PartialEq)]
pub struct PairRungRequest {
    pub positive_net: String,
    pub positive_net_id: NetNo,
    pub negative_net: String,
    pub negative_net_id: NetNo,
    pub start_points: [Point; 2],
    pub end_points: [Point; 2],
    pub start_layer: String,
    pub end_layer: String,
    pub negative_start_layer: String,
    pub negative_end_layer: String,
    pub gap: f64,
    /// Source `connect_pair` default `lead=0.15` (mm).
    pub lead_mm: f64,
    /// Source `BRAID_PAIR_CROSS` switch.
    pub allow_cross: bool,
    pub start_direction: Option<Point>,
    pub end_direction: Option<Point>,
    pub start_connectors: Option<Vec<PairConnector>>,
    pub end_connectors: Option<Vec<PairConnector>>,
    /// Whole-route geometry inputs from source `_geo`; remain `None` when the
    /// caller only has the plan lane, as in the probe CLI.
    pub a_given: Option<PairGivenEnd>,
    pub b_given: Option<PairGivenEnd>,
    pub x_given: Option<PairGivenCrossover>,
}

/// Source pad aperture metadata tied to its baseline board pose. `None` means
/// the source did not establish aperture geometry; `Some([])` is verified
/// absence. The consumer uses the current `ContextPad` pose to rebase shapes.
#[derive(Clone, Debug)]
pub struct RungPastePadMetadata {
    pub source_center: Point,
    pub source_rotation_deg: f64,
    pub paste_apertures: Option<Vec<CopperShape>>,
}

/// Hard point-and-layer predicate for the source `band_of` mask. The native
/// router must query this while expanding search states, not just grade the
/// completed path against it.
pub trait RungBandConstraint {
    fn contains(&self, point_mm: Point, layer: &str) -> bool;
}

/// Source plan and live route state shared by every rung for one lane.
#[derive(Clone)]
pub struct RungBaseRequest {
    /// The attempt's current board, including copper routed before this lane.
    /// Each rung receives an independent clone and must leave this baseline
    /// unmodified when it refuses.
    pub board: BoardAdapter,
    pub corridor: CorridorPlan,
    pub net: String,
    pub net_id: NetNo,
    /// Source `stage` key used for per-rung statistics and diagnostics.
    pub stage: String,
    pub start: RungEndpoint,
    pub end: RungEndpoint,
    pub b_alternatives: Vec<RungEndpoint>,
    /// Present for a pair member; the native executor then performs the
    /// coupled pair route using these two conductor endpoints.
    pub pair: Option<PairRungRequest>,
    /// Source `lane_xy[nm]`, or the explicit source fallback `[a, b]`.
    pub window_points: Vec<Point>,
    /// Within-corridor virtual copper passed as `virt` to connect_ladder.
    pub virtual_tracks: Vec<ReservationTube>,
    /// `reserve(ctx, net)` cross-corridor stamps appended by connect_ladder.
    pub cross_reservations: Vec<ReservationTube>,
    pub virtual_vias: Vec<CopperVia>,
    /// Explicit DSN-backed pad drilling evidence for the native obstacle and
    /// via-free-position paths; never estimated from a native padstack.
    pub free_via_pads: Vec<FreeViaPad>,
    /// Current snapshot pad geometry/drilling for same-net keep-outs.
    pub same_net_pads: Vec<ContextPad>,
    /// Current snapshot vias with their explicit physical drills.
    pub existing_vias: Vec<CopperVia>,
    /// Per-pad source paste geometry keyed by native pad key.
    pub paste_pads: BTreeMap<String, RungPastePadMetadata>,
    /// Minimum annular ring required by this run's explicit fabrication policy.
    pub min_annular_ring_mm: f64,
    pub rules: RoutingRules,
    /// Source AWX fabrication resolver, retained through every rung.
    pub fabrication: crate::awx::fab_rules::FabTier,
    pub escalation: crate::awx::fab_rules::EscalationPolicy,
    pub fabrication_overrides: BTreeMap<String, f64>,
    /// Source-resolved per-net rule floors used by via-pad fine placement.
    pub fab_clearance_floor_mm: f64,
    pub fab_track_floor_mm: f64,
    pub may_narrow: bool,
    /// Ordered output of the exact source fab resolver, in millimetres.
    pub fab_via_rungs: Vec<(f64, f64)>,
    /// Source GridRouteConfig.max_iterations, defaulting to 200000 at its
    /// configuration boundary.
    pub max_iterations: usize,
    /// Exact source GridRouteConfig.layers order, not all copper board layers.
    pub route_layers: Vec<String>,
    /// A pair member calls `connect_pair` with half the pair pitch here.
    pub band_slack: f64,
    pub deadline: Option<Instant>,
}

/// Complete low-level request for one exact rung. The first three rungs carry
/// a per-net band; the last two are band-free, with the fifth also dropping
/// the planned search window.
pub struct RungRequest {
    pub board: BoardAdapter,
    pub net: String,
    pub net_id: NetNo,
    pub stage: String,
    pub start: RungEndpoint,
    pub end: RungEndpoint,
    pub b_alternatives: Vec<RungEndpoint>,
    pub pair: Option<PairRungRequest>,
    pub spec: RungSpec,
    pub band: Option<Box<dyn RungBandConstraint>>,
    pub band_slack: f64,
    pub window_points: Option<Vec<Point>>,
    pub virtual_tracks: Vec<ReservationTube>,
    pub virtual_vias: Vec<CopperVia>,
    pub free_via_pads: Vec<FreeViaPad>,
    pub same_net_pads: Vec<ContextPad>,
    pub existing_vias: Vec<CopperVia>,
    pub paste_pads: BTreeMap<String, RungPastePadMetadata>,
    pub min_annular_ring_mm: f64,
    pub rules: RoutingRules,
    pub fabrication: crate::awx::fab_rules::FabTier,
    pub escalation: crate::awx::fab_rules::EscalationPolicy,
    pub fabrication_overrides: BTreeMap<String, f64>,
    pub fab_clearance_floor_mm: f64,
    pub fab_track_floor_mm: f64,
    pub may_narrow: bool,
    pub fab_via_rungs: Vec<(f64, f64)>,
    pub max_iterations: usize,
    pub route_layers: Vec<String>,
    pub deadline: Option<Instant>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RungAttempt {
    pub label: RungLabel,
    pub refused: bool,
    pub reason: Option<String>,
    /// Real blocked-copper frontier from the constrained point router.
    pub report: Option<RungReport>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RungReport {
    /// Raw awx_grid frontier cells in global grid coordinates and configured
    /// route-layer indices, matching the source `report["blocked"]` contract.
    pub blocked_cells: Vec<(i32, i32, usize)>,
    /// Exact GridRouteConfig layer order used to interpret each frontier index.
    pub route_layers: Vec<String>,
    /// Search bounds in mm: [min_x, min_y, max_x, max_y].
    pub window: [f64; 4],
    pub grid_step_mm: f64,
    pub expanded: usize,
    pub truncated: bool,
}

pub enum RungExecution<Route> {
    Found(Route),
    Refused {
        reason: Option<String>,
        report: Option<RungReport>,
    },
}

pub struct RungResult<Route> {
    pub net: String,
    pub stage: String,
    pub selected: Option<RungSpec>,
    /// Uncommitted route from the first successful rung, if any.
    pub route: Option<Route>,
    pub attempts: Vec<RungAttempt>,
    /// Final refused rung's report, matching source `connect_ladder`'s
    /// report dictionary consumed by blocker-directed `rip_for`.
    pub report: Option<RungReport>,
}

/// Builder backed by the full source Corridor state. It computes the
/// `band_of(net, slack, open_layers)` mask for the exact per-net plan.
pub trait RungBandFactory {
    fn band_for(
        &mut self,
        corridor: &CorridorBandContext,
        net: &str,
        spec: RungSpec,
    ) -> Result<Box<dyn RungBandConstraint>, String>;
}

/// Implemented by the constrained native point-to-point router.
pub trait RungExecutor {
    type Route;

    /// Returns an uncommitted candidate. `Refused` must not mutate the board
    /// carried by the request; the next rung is run from the same baseline.
    fn route(&mut self, request: RungRequest) -> Result<RungExecution<Self::Route>, String>;
}

const FULL_RUNGS: [RungSpec; 5] = [
    RungSpec {
        label: RungLabel::Slack03,
        slack: 0.3,
        open_layers: false,
        margin: 2.0,
        use_window: true,
    },
    RungSpec {
        label: RungLabel::Slack08,
        slack: 0.8,
        open_layers: false,
        margin: 2.0,
        use_window: true,
    },
    RungSpec {
        label: RungLabel::Slack16Open,
        slack: 1.6,
        open_layers: true,
        margin: 3.0,
        use_window: true,
    },
    RungSpec {
        label: RungLabel::FreeWindow4,
        slack: 0.0,
        open_layers: false,
        margin: 4.0,
        use_window: true,
    },
    RungSpec {
        label: RungLabel::Free6,
        slack: 0.0,
        open_layers: false,
        margin: 6.0,
        use_window: false,
    },
];

pub fn rung_specs(mode: LadderMode) -> &'static [RungSpec] {
    match mode {
        LadderMode::Full => &FULL_RUNGS,
        LadderMode::Open => &FULL_RUNGS[2..],
    }
}

/// Applies source `connect_ladder`: try each rung in order and stop at the
/// first route. The outer Corridor attempt loop owns board grading and best
/// attempt restoration; this function never commits a returned route.
pub fn connect_ladder<E: RungExecutor, B: RungBandFactory>(
    base: &RungBaseRequest,
    mode: LadderMode,
    bands: &mut B,
    executor: &mut E,
) -> Result<RungResult<E::Route>, String> {
    if base.window_points.is_empty() {
        return Err("braid connect ladder requires source lane window points".into());
    }
    if !base.band_slack.is_finite() || base.band_slack < 0.0 {
        return Err("braid connect ladder band slack must be finite and nonnegative".into());
    }
    if base.max_iterations == 0 {
        return Err("braid connect ladder max_iterations must be positive".into());
    }
    if base.route_layers.is_empty() {
        return Err("braid connect ladder requires source route layers".into());
    }
    let mut attempts = Vec::new();
    let mut final_report = None;
    for spec in rung_specs(mode) {
        let band = if matches!(
            spec.label,
            RungLabel::Slack03 | RungLabel::Slack08 | RungLabel::Slack16Open
        ) {
            Some(bands.band_for(&base.corridor.band, &base.net, *spec)?)
        } else {
            None
        };
        let request = RungRequest {
            board: base.board.clone(),
            net: base.net.clone(),
            net_id: base.net_id,
            stage: base.stage.clone(),
            start: base.start.clone(),
            end: base.end.clone(),
            b_alternatives: base.b_alternatives.clone(),
            pair: base.pair.clone(),
            spec: *spec,
            band,
            band_slack: base.band_slack,
            window_points: spec.use_window.then(|| base.window_points.clone()),
            virtual_tracks: base
                .virtual_tracks
                .iter()
                .chain(&base.cross_reservations)
                .cloned()
                .collect(),
            virtual_vias: base.virtual_vias.clone(),
            free_via_pads: base.free_via_pads.clone(),
            same_net_pads: base.same_net_pads.clone(),
            existing_vias: base.existing_vias.clone(),
            paste_pads: base.paste_pads.clone(),
            min_annular_ring_mm: base.min_annular_ring_mm,
            rules: base.rules.clone(),
            fabrication: base.fabrication,
            escalation: base.escalation,
            fabrication_overrides: base.fabrication_overrides.clone(),
            fab_clearance_floor_mm: base.fab_clearance_floor_mm,
            fab_track_floor_mm: base.fab_track_floor_mm,
            may_narrow: base.may_narrow,
            fab_via_rungs: base.fab_via_rungs.clone(),
            max_iterations: base.max_iterations,
            route_layers: base.route_layers.clone(),
            deadline: base.deadline,
        };
        match executor.route(request)? {
            RungExecution::Found(route) => {
                attempts.push(RungAttempt {
                    label: spec.label,
                    refused: false,
                    reason: None,
                    report: None,
                });
                return Ok(RungResult {
                    net: base.net.clone(),
                    stage: base.stage.clone(),
                    selected: Some(*spec),
                    route: Some(route),
                    attempts,
                    report: None,
                });
            }
            RungExecution::Refused { reason, report } => {
                final_report = report.clone();
                attempts.push(RungAttempt {
                    label: spec.label,
                    refused: true,
                    reason,
                    report,
                });
            }
        }
    }
    Ok(RungResult {
        net: base.net.clone(),
        stage: base.stage.clone(),
        selected: None,
        route: None,
        attempts,
        report: final_report,
    })
}
