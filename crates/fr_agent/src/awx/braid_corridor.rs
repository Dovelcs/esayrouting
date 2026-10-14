//! Plan-only corridor geometry shared by braid planning and crossing analysis.
//!
//! `CorridorPlan` contains geometry produced from one source corridor. The
//! schedule and crossing modules consume these exact lanes and layer intervals;
//! they do not reconstruct or approximate corridor geometry.

use super::{
    braid_schedule::BraidSchedule,
    corridor::{Point, Spine},
    taut_paths::Obstacles,
    whole_context::{CopperVia, ReservationTube, WholeContext},
};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Debug, PartialEq)]
pub struct LayerInterval {
    pub start: f64,
    pub end: f64,
    pub layer: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct LayerRun {
    pub s: f64,
    pub layer: String,
}

/// All board/rule inputs that the source `Corridor` geometry reads.
///
/// These values must be resolved from the route specification and explicit
/// braid settings at the caller boundary. This type intentionally has no
/// `Default` implementation so callers cannot silently substitute guessed
/// geometry parameters.
#[derive(Clone, Debug)]
pub struct BraidCorridorOptions {
    pub track_width: f64,
    pub clearance: f64,
    pub half_sep: f64,
    pub via_size: f64,
    pub pair_gap: f64,
    pub lane_pitch: f64,
    pub exit_pitch: f64,
    pub grid: f64,
    pub pairs: bool,
    pub block_push: u8,
    pub pages_first: bool,
    pub pages_siders: u8,
    pub slope_pitch: bool,
    pub branch: bool,
    /// Pair dive expansion in board millimetres, already scaled by pair pitch.
    pub pair_dive_extra: f64,
    pub jog_via: f64,
    pub approach_reserve: f64,
    pub end_keep: f64,
}

impl BraidCorridorOptions {
    pub fn validate(&self) -> Result<(), String> {
        for (name, value) in [
            ("track width", self.track_width),
            ("clearance", self.clearance),
            ("half separation", self.half_sep),
            ("via size", self.via_size),
            ("lane pitch", self.lane_pitch),
            ("exit pitch", self.exit_pitch),
            ("grid", self.grid),
        ] {
            if !value.is_finite() || value <= 0.0 {
                return Err(format!("braid corridor {name} must be finite and positive"));
            }
        }
        if !self.pair_dive_extra.is_finite() || self.pair_dive_extra < 0.0 {
            return Err("braid corridor pair dive extra must be finite and nonnegative".into());
        }
        if !self.jog_via.is_finite() || self.jog_via < 0.0 {
            return Err("braid corridor jog via must be finite and nonnegative".into());
        }
        if !self.pair_gap.is_finite() || self.pair_gap < 0.0 {
            return Err("braid corridor pair gap must be finite and nonnegative".into());
        }
        for (name, value) in [
            ("approach reserve", self.approach_reserve),
            ("end keep", self.end_keep),
        ] {
            if !value.is_finite() || value < 0.0 {
                return Err(format!(
                    "braid corridor {name} must be finite and nonnegative"
                ));
            }
        }
        if self.pages_siders > 2 {
            return Err("braid corridor pages_siders must be 0, 1, or 2".into());
        }
        if self.block_push > 2 {
            return Err("braid corridor block_push must be 0, 1, or 2".into());
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct StaticIsland {
    /// Spine-coordinate box: [s_lo, s_hi, o_lo, o_hi].
    pub bounds: [f64; 4],
    pub what: String,
}

/// Shared mutable state for the source Corridor methods. The split offset and
/// lane modules mutate this state directly so all stages use one geometry.
#[derive(Clone)]
pub struct CorridorState {
    pub index: usize,
    pub members: Vec<String>,
    pub context: WholeContext,
    pub options: BraidCorridorOptions,
    pub spine_obstacles: Obstacles,
    pub pad_obstacles: Obstacles,
    pub extra_tubes: Vec<(Vec<Point>, f64)>,
    pub half_width: f64,
    pub spine_core: Spine,
    pub spine: Spine,
    pub teeth: BTreeMap<String, Point>,
    pub stubs: BTreeMap<String, Point>,
    pub st: BTreeMap<String, Point>,
    pub se: BTreeMap<String, Point>,
    pub s0: f64,
    pub s0_base: f64,
    pub s1: f64,
    pub heads_l: Vec<String>,
    pub joiners: Vec<String>,
    pub heads_e: Vec<String>,
    pub siders: Vec<String>,
    pub join_side: BTreeMap<String, i32>,
    pub exit_side: BTreeMap<String, i32>,
    pub far_exit: BTreeSet<String>,
    pub pin_target: BTreeSet<String>,
    pub keep_order: bool,
    pub wrap: bool,
    pub is_branch: bool,
    pub s_leg_min: Option<f64>,
    pub s_ball: Option<f64>,
    pub r_max: Option<f64>,
    pub join_block: BTreeMap<String, f64>,
    pub join_leg_s: BTreeMap<String, f64>,
    pub exit_block: BTreeMap<String, f64>,
    pub exit_leg_s: BTreeMap<String, f64>,
    pub launch_o: BTreeMap<String, f64>,
    pub target_o: BTreeMap<String, f64>,
    pub launch: Vec<String>,
    pub target: Vec<String>,
    pub ly: Vec<f64>,
    pub py: Vec<f64>,
    pub slope: BTreeMap<String, f64>,
    pub l_free: f64,
    pub s_pts: Vec<f64>,
    pub u_pts: Vec<f64>,
    pub reserved: Vec<(f64, f64)>,
    pub row_slot: BTreeMap<String, f64>,
    pub row_side: BTreeMap<String, i32>,
    pub row_up: BTreeMap<String, Vec<String>>,
    pub req: BTreeMap<String, Vec<LayerInterval>>,
    pub back_windows: BTreeMap<String, Vec<(f64, f64)>>,
    pub mid: BTreeMap<String, Vec<Point>>,
    pub legs: BTreeMap<String, Vec<(f64, f64, f64)>>,
    pub jogs: BTreeMap<String, Vec<(Point, Point)>>,
    pub mid_xy: BTreeMap<String, Vec<(f64, Vec<Point>)>>,
    pub lanes: BTreeMap<String, Vec<Point>>,
    pub req_xy: BTreeMap<String, Vec<Vec<Point>>>,
    pub leg_layer: BTreeMap<String, String>,
    pub leg_split: BTreeMap<String, f64>,
    pub layer_profiles: BTreeMap<String, Vec<LayerRun>>,
    pub sched_cur: Option<BraidSchedule>,
    pub swim_changes: BTreeMap<String, usize>,
    pub islands_by_layer: BTreeMap<String, Vec<StaticIsland>>,
    pub islands: BTreeMap<String, Vec<String>>,
    pub dive_s_plan: BTreeMap<String, f64>,
    pub crossings: BTreeMap<String, Vec<(f64, String)>>,
    pub hops: BTreeMap<String, Vec<Point>>,
    pub deflect_islands: BTreeMap<String, Vec<Point>>,
    pub width: f64,
    pub layout_need: f64,
}

impl CorridorState {
    pub fn block_gap(&self) -> f64 {
        0.45
    }
    pub fn via_need(&self) -> f64 {
        self.options.via_size / 2.0 + self.options.clearance + self.options.track_width / 2.0 + 0.03
    }
    pub fn lane_min(&self) -> f64 {
        self.options.track_width + self.options.clearance + 0.02
    }
    pub fn row_offset(&self) -> f64 {
        0.7
    }
    pub fn leg_offset(&self) -> f64 {
        0.2
    }
    pub fn leg_half_width(&self) -> f64 {
        0.5
    }
    pub fn classification_s_tolerance(&self) -> f64 {
        0.5
    }
    pub fn classification_offset_tolerance(&self) -> f64 {
        0.2
    }
    pub fn head_run(&self) -> f64 {
        3.0
    }
    pub fn reserve(&self) -> f64 {
        0.3
    }

    /// Source `Corridor.allowed`: required spans pin a lane to their layer;
    /// B.Cu is otherwise available only in the derived back-layer windows.
    pub fn allowed(&self, net: &str, s: f64, layer: &str) -> bool {
        if self.req.get(net).is_some_and(|intervals| {
            intervals
                .iter()
                .any(|span| span.start <= s && s <= span.end && span.layer != layer)
        }) {
            return false;
        }
        if layer == "B.Cu" {
            return self
                .back_windows
                .get(net)
                .is_some_and(|windows| windows.iter().any(|(lo, hi)| *lo <= s && s <= *hi));
        }
        true
    }
}

/// Source `Corridor.lane_w`, shared by spine width, offset, and lane stages.
pub fn lane_width(context: &WholeContext, options: &BraidCorridorOptions, net: &str) -> f64 {
    if !options.pairs || !context.pairs.contains_key(net) {
        return 0.0;
    }
    let pitch = super::pairs::pitch(&context.cfg);
    let half = pitch / 2.0;
    let via_half = ((options.via_size + options.clearance) / 2.0).max(
        (options.via_size / 2.0 + options.clearance + options.track_width / 2.0 - half) / 0.7071
            + 0.005,
    );
    let envelope_radius = via_half + options.via_size / 2.0;
    half.max(envelope_radius + options.clearance + options.track_width / 2.0 - options.lane_pitch)
}

/// Source-equivalent plan-only output for a single corridor.
#[derive(Clone, Debug)]
pub struct CorridorPlan {
    pub index: usize,
    pub members: Vec<String>,
    pub spine: Spine,
    pub launch: Vec<String>,
    pub target: Vec<String>,
    pub schedule: BraidSchedule,
    /// Required-layer polyline in physical board coordinates.
    pub lanes: BTreeMap<String, Vec<Point>>,
    /// Lane points used by the source swimmer layer-change analysis.
    pub mid: BTreeMap<String, Vec<Point>>,
    /// Layer-constrained lane stretches, in corridor spine coordinates.
    pub required: BTreeMap<String, Vec<LayerInterval>>,
    /// Along-spine windows where a B.Cu lane is permitted.
    pub back_windows: BTreeMap<String, Vec<(f64, f64)>>,
    pub exit_leg_layer: BTreeMap<String, String>,
    /// Tooth, required-layer, exit-leg, and berth runs after equal neighbors merge.
    pub layer_profiles: BTreeMap<String, Vec<LayerRun>>,
    /// Number of required changes for swimmers, including the source fallback.
    pub swim_changes: BTreeMap<String, usize>,
    /// Source plan-side island labels intersected by each scheduled page chord.
    pub islands: BTreeMap<String, Vec<String>>,
    pub joiners: Vec<String>,
    pub siders: Vec<String>,
    pub s0: f64,
    pub s1: f64,
    /// Complete immutable source Corridor geometry for rung band construction.
    pub band: CorridorBandContext,
}

/// Non-recursive immutable view of a completed source Corridor used to build
/// the real `Corridor.band_of` mask for plan/rung trials.
#[derive(Clone, Debug)]
pub struct CorridorBandContext {
    pub index: usize,
    pub members: Vec<String>,
    pub spine: Spine,
    pub options: BraidCorridorOptions,
    pub schedule: Option<BraidSchedule>,
    pub teeth: BTreeMap<String, Point>,
    pub stubs: BTreeMap<String, Point>,
    pub st: BTreeMap<String, Point>,
    pub se: BTreeMap<String, Point>,
    pub launch: Vec<String>,
    pub target: Vec<String>,
    pub launch_o: BTreeMap<String, f64>,
    pub target_o: BTreeMap<String, f64>,
    pub s0: f64,
    pub s1: f64,
    pub l_free: f64,
    pub heads_l: Vec<String>,
    pub joiners: Vec<String>,
    pub heads_e: Vec<String>,
    pub siders: Vec<String>,
    pub far_exit: BTreeSet<String>,
    pub is_branch: bool,
    pub wrap: bool,
    pub keep_order: bool,
    pub pin_target: BTreeSet<String>,
    pub join_side: BTreeMap<String, i32>,
    pub exit_side: BTreeMap<String, i32>,
    pub join_block: BTreeMap<String, f64>,
    pub join_leg_s: BTreeMap<String, f64>,
    pub exit_block: BTreeMap<String, f64>,
    pub exit_leg_s: BTreeMap<String, f64>,
    pub req: BTreeMap<String, Vec<LayerInterval>>,
    pub back_windows: BTreeMap<String, Vec<(f64, f64)>>,
    pub mid: BTreeMap<String, Vec<Point>>,
    pub legs: BTreeMap<String, Vec<(f64, f64, f64)>>,
    pub jogs: BTreeMap<String, Vec<(Point, Point)>>,
    pub mid_xy: BTreeMap<String, Vec<(f64, Vec<Point>)>>,
    pub lanes: BTreeMap<String, Vec<Point>>,
    pub req_xy: BTreeMap<String, Vec<Vec<Point>>>,
    pub leg_layer: BTreeMap<String, String>,
    pub leg_split: BTreeMap<String, f64>,
    pub layer_profiles: BTreeMap<String, Vec<LayerRun>>,
    pub swim_changes: BTreeMap<String, usize>,
    pub islands_by_layer: BTreeMap<String, Vec<StaticIsland>>,
    pub islands: BTreeMap<String, Vec<String>>,
    pub hops: BTreeMap<String, Vec<Point>>,
    pub dive_s_plan: BTreeMap<String, f64>,
    pub pair_names: BTreeSet<String>,
    pub pair_members: BTreeMap<String, [String; 2]>,
    pub pair_ends: BTreeMap<String, [[Point; 2]; 2]>,
    pub pair_layers: BTreeMap<String, [String; 2]>,
    pub source_paths: BTreeMap<String, Vec<Point>>,
    pub all_ends: BTreeMap<String, super::whole_context::Ends>,
    pub tooth_dir: BTreeMap<String, Point>,
    pub stub_dir: BTreeMap<String, Point>,
    pub slope: BTreeMap<String, f64>,
    pub tooth_layer: BTreeMap<String, String>,
    pub dest_layer: BTreeMap<String, String>,
    pub byname: BTreeMap<String, i32>,
    pub copper_layers: Vec<String>,
    pub via_drill: f64,
}

impl CorridorBandContext {
    /// The plan-only source `Corridor.allowed` rule for one band sample.
    pub fn allowed(&self, net: &str, s: f64, layer: &str) -> bool {
        if self.req.get(net).is_some_and(|intervals| {
            intervals
                .iter()
                .any(|span| span.start <= s && s <= span.end && span.layer != layer)
        }) {
            return false;
        }
        if layer == "B.Cu" {
            return self
                .back_windows
                .get(net)
                .is_some_and(|windows| windows.iter().any(|(lo, hi)| *lo <= s && s <= *hi));
        }
        true
    }

    fn allowed_with_pair_dive(&self, net: &str, s: f64, layer: &str) -> bool {
        let extra = if self.pair_names.contains(net) {
            self.options.pair_dive_extra
        } else {
            0.0
        };
        if self.req.get(net).is_some_and(|intervals| {
            intervals.iter().any(|span| {
                span.layer != layer
                    && s >= span.start + extra - 1e-9
                    && s <= span.end - extra + 1e-9
            })
        }) {
            return false;
        }
        if layer == "B.Cu" {
            return self.back_windows.get(net).is_some_and(|windows| {
                windows
                    .iter()
                    .any(|(lo, hi)| s >= lo - extra - 1e-9 && s <= hi + extra + 1e-9)
            });
        }
        true
    }

    fn planned_layers(
        &self,
        net: &str,
        s: f64,
        reach: f64,
        step: f64,
    ) -> Result<Vec<String>, String> {
        let profile = self
            .layer_profiles
            .get(net)
            .filter(|profile| !profile.is_empty())
            .ok_or_else(|| format!("virtual corridor lane {net} has no layer profile"))?;
        let mut layer = profile[0].layer.clone();
        for run in profile {
            if run.s <= s + 1e-9 {
                layer.clone_from(&run.layer);
            }
        }
        let other = if layer == "F.Cu" { "B.Cu" } else { "F.Cu" };
        if !self.allowed(net, s, other) {
            return Ok(vec![layer]);
        }
        for run in profile.iter().skip(1) {
            let distance = (run.s - s).abs();
            if distance > reach + 1e-9 {
                continue;
            }
            let samples = ((distance / step).floor() as usize).max(1);
            let both_allowed = (0..=samples).all(|index| {
                let at = s + (run.s - s) * index as f64 / samples as f64;
                self.allowed(net, at, "F.Cu") && self.allowed(net, at, "B.Cu")
            });
            if both_allowed {
                return Ok(vec![layer, other.to_owned()]);
            }
        }
        Ok(vec![layer])
    }

    fn push_virtual_segment(
        &self,
        out: &mut Vec<ReservationTube>,
        net: &str,
        a: Point,
        b: Point,
        layer: &str,
    ) {
        if super::corridor::distance(a, b) > 1e-9 {
            out.push(ReservationTube {
                net: net.to_owned(),
                a,
                b,
                layer: layer.to_owned(),
                radius: self.options.track_width / 2.0,
            });
        }
    }

    /// Source `Corridor.virtual_of`: planned lane centre-lines as virtual
    /// copper, pair lanes expanded into their two physical conductors.
    pub fn virtual_of(&self, unrouted: &[String]) -> Result<Vec<ReservationTube>, String> {
        if self.pair_names.is_empty() {
            return self.virtual_of_plain(unrouted);
        }
        let singles: Vec<_> = unrouted
            .iter()
            .filter(|net| !self.pair_names.contains(*net))
            .cloned()
            .collect();
        let mut output = self.virtual_of_plain(&singles)?;
        let half = (self.options.track_width + self.options.pair_gap) / 2.0;
        for net in unrouted.iter().filter(|net| self.pair_names.contains(*net)) {
            for centerline in self.virtual_of_plain(std::slice::from_ref(net))? {
                let direction =
                    super::corridor::unit(super::corridor::sub(centerline.b, centerline.a))?;
                let normal = [-direction[1], direction[0]];
                for side in [-1.0, 1.0] {
                    let offset = super::corridor::mul(normal, side * half);
                    self.push_virtual_segment(
                        &mut output,
                        net,
                        super::corridor::add(centerline.a, offset),
                        super::corridor::add(centerline.b, offset),
                        &centerline.layer,
                    );
                }
            }
        }
        Ok(output)
    }

    fn virtual_of_plain(&self, unrouted: &[String]) -> Result<Vec<ReservationTube>, String> {
        let mut output = Vec::new();
        for net in unrouted {
            let mid = self
                .mid
                .get(net)
                .filter(|points| !points.is_empty())
                .ok_or_else(|| format!("virtual corridor lane {net} has no midline"))?;
            let swim = self
                .schedule
                .as_ref()
                .is_some_and(|schedule| schedule.page_of(net).is_none());
            let tooth_layer = self
                .tooth_layer
                .get(net)
                .ok_or_else(|| format!("virtual corridor lane {net} has no tooth layer"))?;
            let dest_layer = self
                .dest_layer
                .get(net)
                .ok_or_else(|| format!("virtual corridor lane {net} has no destination layer"))?;
            let mut cuts = Vec::new();
            for interval in self.req.get(net).into_iter().flatten() {
                cuts.extend([interval.start, interval.end]);
            }
            for &(lo, hi) in self.back_windows.get(net).into_iter().flatten() {
                if lo.abs() < 1e8 {
                    cuts.push(lo);
                }
                if hi.abs() < 1e8 {
                    cuts.push(hi);
                }
            }
            cuts.sort_by(f64::total_cmp);
            cuts.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
            let s_start = mid[0][0];
            let s_end = mid[mid.len() - 1][0];
            for pair in mid.windows(2) {
                let (sa, oa, sb, ob) = (pair[0][0], pair[0][1], pair[1][0], pair[1][1]);
                let mut bounds = vec![sa];
                bounds.extend(
                    cuts.iter()
                        .copied()
                        .filter(|s| *s > sa + 1e-6 && *s < sb - 1e-6),
                );
                bounds.push(sb);
                for span in bounds.windows(2) {
                    let (s_a, s_b) = (span[0], span[1]);
                    let s_mid = (s_a + s_b) / 2.0;
                    let t_a = (s_a - sa) / (sb - sa).max(1e-9);
                    let t_b = (s_b - sa) / (sb - sa).max(1e-9);
                    let o_a = oa + t_a * (ob - oa);
                    let o_b = oa + t_b * (ob - oa);
                    let points = self.spine.lane_xy(&[[s_a, o_a], [s_b, o_b]]);
                    let tail = (s_b - s_end).abs() < 1e-6 && !self.exit_block.contains_key(net);
                    let head = (s_a - s_start).abs() < 1e-6 && !self.join_block.contains_key(net);
                    if swim && !tail && !head {
                        continue;
                    }
                    for segment in points.windows(2) {
                        for layer in ["F.Cu", "B.Cu"] {
                            if tail && layer != dest_layer || head && layer != tooth_layer {
                                continue;
                            }
                            let planned = swim
                                || self
                                    .planned_layers(net, s_mid, 3.0, 0.05)?
                                    .iter()
                                    .any(|l| l == layer);
                            if self.allowed(net, s_mid, layer) && planned {
                                self.push_virtual_segment(
                                    &mut output,
                                    net,
                                    segment[0],
                                    segment[1],
                                    layer,
                                );
                            }
                        }
                    }
                }
            }

            if let Some(legs) = self.legs.get(net) {
                for (index, &(s_leg, o_a, o_b)) in legs.iter().enumerate() {
                    let a = self.spine.xy(s_leg, o_a);
                    let b = self.spine.xy(s_leg, o_b);
                    let is_exit = self.exit_block.contains_key(net) && index + 1 == legs.len();
                    if is_exit && self.leg_layer.contains_key(net) {
                        let layer = &self.leg_layer[net];
                        if let Some(&split) = self
                            .leg_split
                            .get(net)
                            .filter(|&&split| split > o_a.min(o_b) && split < o_a.max(o_b))
                        {
                            let mid = self.spine.xy(s_leg, split);
                            self.push_virtual_segment(&mut output, net, a, mid, layer);
                            self.push_virtual_segment(&mut output, net, mid, b, dest_layer);
                        } else {
                            self.push_virtual_segment(&mut output, net, a, b, layer);
                        }
                        continue;
                    }
                    if self.join_block.contains_key(net) && index == 0 {
                        self.push_virtual_segment(&mut output, net, a, b, tooth_layer);
                        continue;
                    }
                    for layer in ["F.Cu", "B.Cu"] {
                        if self.allowed(net, s_leg, layer) {
                            self.push_virtual_segment(&mut output, net, a, b, layer);
                        }
                    }
                }
            }
            if let Some(jogs) = self.jogs.get(net) {
                for &(start, end) in jogs {
                    let is_exit = self.exit_block.contains_key(net)
                        && (end[0] - self.se[net][0]).abs() < 1e-6
                        && (end[1] - self.se[net][1]).abs() < 1e-6;
                    let layer = if is_exit { dest_layer } else { tooth_layer };
                    self.push_virtual_segment(
                        &mut output,
                        net,
                        self.spine.xy(start[0], start[1]),
                        self.spine.xy(end[0], end[1]),
                        layer,
                    );
                }
            }
        }
        Ok(output)
    }

    /// Source `Corridor.approach_virt`: reserve each other member's berth
    /// approach on the exact arrival layer.
    pub fn approach_virt(&self, net: &str) -> Result<Vec<ReservationTube>, String> {
        let reserve = self.options.approach_reserve;
        if reserve <= 0.0 {
            return Ok(Vec::new());
        }
        let mut output = Vec::new();
        for other in &self.members {
            if other == net {
                continue;
            }
            let Some(&stub) = self.stubs.get(other) else {
                continue;
            };
            let Some(&direction) = self.stub_dir.get(other) else {
                continue;
            };
            let Some(layer) = self.dest_layer.get(other) else {
                continue;
            };
            self.push_virtual_segment(
                &mut output,
                other,
                stub,
                super::corridor::add(stub, super::corridor::mul(direction, reserve)),
                layer,
            );
        }
        Ok(output)
    }

    /// Source `Corridor.virtual_vias_of`: reserve each planned tail dive,
    /// leg split, and swimmer hop as a physical virtual via.
    pub fn virtual_vias_of(&self, unrouted: &[String]) -> Result<Vec<CopperVia>, String> {
        let mut sites: Vec<(String, Point)> = Vec::new();
        for net in unrouted {
            let Some(&leg_s) = self.exit_leg_s.get(net) else {
                continue;
            };
            let Some(legs) = self.legs.get(net).filter(|legs| !legs.is_empty()) else {
                continue;
            };
            let profile = self
                .layer_profiles
                .get(net)
                .filter(|profile| !profile.is_empty())
                .ok_or_else(|| format!("virtual via lane {net} has no layer profile"))?;
            let corner_dive = if !self.options.branch || !self.leg_layer.contains_key(net) {
                true
            } else {
                let leg_layer = &self.leg_layer[net];
                let before = profile
                    .iter()
                    .filter(|run| run.s < leg_s - 1e-6)
                    .last()
                    .unwrap_or(&profile[0]);
                before.layer != *leg_layer
            };
            if corner_dive {
                sites.push((net.clone(), self.spine.xy(leg_s, legs[legs.len() - 1].1)));
            }
            if let Some(&split) = self.leg_split.get(net) {
                sites.push((net.clone(), self.spine.xy(leg_s, split)));
            }
            sites.extend(
                self.hops
                    .get(net)
                    .into_iter()
                    .flatten()
                    .copied()
                    .map(|p| (net.clone(), p)),
            );
        }
        sites
            .into_iter()
            .map(|(net, point)| {
                let net_id = self
                    .byname
                    .get(&net)
                    .copied()
                    .or_else(|| {
                        self.pair_members
                            .get(&net)
                            .and_then(|members| self.byname.get(&members[0]).copied())
                    })
                    .ok_or_else(|| format!("virtual via lane {net} has no board net id"))?;
                Ok(CopperVia {
                    net,
                    net_id,
                    point,
                    diameter: self.options.via_size,
                    drill: self.via_drill,
                    layers: self.copper_layers.clone(),
                })
            })
            .collect()
    }
}

/// Source top-level `cross_reserve`: stamp only the first/last 1.5 mm of
/// every not-yet-routed lane from corridors still pending, then clip those
/// reservations around every free tooth/berth so they cannot seal an end.
pub fn cross_reserve(
    corridors: &[CorridorBandContext],
    active_index: usize,
    net: &str,
    landed: &BTreeSet<String>,
    corridor_done: &BTreeSet<usize>,
) -> Result<Vec<ReservationTube>, String> {
    let mut out = Vec::new();
    for corridor in corridors {
        if corridor_done.contains(&corridor.index) || corridor.index == active_index {
            continue;
        }
        if corridor.members.iter().any(|member| member == net) {
            continue;
        }
        for other in &corridor.members {
            if landed.contains(other) {
                continue;
            }
            let polyline = corridor
                .lanes
                .get(other)
                .filter(|line| line.len() >= 2)
                .or_else(|| {
                    corridor
                        .source_paths
                        .get(other)
                        .filter(|line| line.len() >= 2)
                })
                .cloned()
                .or_else(|| {
                    corridor
                        .all_ends
                        .get(other)
                        .map(|ends| vec![ends.tooth, ends.berth])
                })
                .ok_or_else(|| format!("cross-corridor reservation has no path for {other}"))?;
            for (points, layer) in [
                (&polyline, corridor.tooth_layer.get(other)),
                (
                    &polyline.iter().rev().copied().collect::<Vec<_>>(),
                    corridor.dest_layer.get(other),
                ),
            ] {
                let layer =
                    layer.ok_or_else(|| format!("cross-corridor lane {other} has no end layer"))?;
                let mut travelled = 0.0;
                for pair in points.windows(2) {
                    if travelled >= 1.5 {
                        break;
                    }
                    let length = super::corridor::distance(pair[0], pair[1]);
                    if length <= 1e-9 {
                        continue;
                    }
                    let mut end = pair[1];
                    if travelled + length > 1.5 {
                        let ratio = (1.5 - travelled) / length;
                        end = [
                            pair[0][0] + (pair[1][0] - pair[0][0]) * ratio,
                            pair[0][1] + (pair[1][1] - pair[0][1]) * ratio,
                        ];
                    }
                    if super::corridor::distance(pair[0], end) > 1e-9 {
                        out.push(ReservationTube {
                            net: other.clone(),
                            a: pair[0],
                            b: end,
                            layer: layer.clone(),
                            radius: corridor.options.track_width / 2.0,
                        });
                    }
                    travelled += length;
                }
            }
        }
    }
    let ends: Vec<_> = corridors
        .first()
        .map(|corridor| {
            corridor
                .all_ends
                .values()
                .flat_map(|ends| [ends.tooth, ends.berth])
                .collect()
        })
        .unwrap_or_default();
    clip_reservations_round_ends(
        out,
        &ends,
        corridors.first().map_or(0.0, |c| c.options.end_keep),
    )
}

fn clip_reservations_round_ends(
    pieces: Vec<ReservationTube>,
    ends: &[Point],
    keep: f64,
) -> Result<Vec<ReservationTube>, String> {
    let mut clipped = Vec::new();
    for piece in pieces {
        let mut parts = vec![(piece.a, piece.b)];
        for &end in ends {
            let mut next = Vec::new();
            for (a, b) in parts {
                let delta = super::corridor::sub(b, a);
                let len2 = super::corridor::dot(delta, delta);
                if len2 < 1e-12 {
                    continue;
                }
                let t = (super::corridor::dot(super::corridor::sub(end, a), delta) / len2)
                    .clamp(0.0, 1.0);
                let closest = super::corridor::add(a, super::corridor::mul(delta, t));
                if super::corridor::distance(closest, end) >= keep {
                    next.push((a, b));
                    continue;
                }
                let length = len2.sqrt();
                let t0 = (t - keep / length).max(0.0);
                let t1 = (t + keep / length).min(1.0);
                if t0 > 1e-6 {
                    next.push((a, super::corridor::add(a, super::corridor::mul(delta, t0))));
                }
                if t1 < 1.0 - 1e-6 {
                    next.push((super::corridor::add(a, super::corridor::mul(delta, t1)), b));
                }
            }
            parts = next;
        }
        clipped.extend(
            parts
                .into_iter()
                .filter(|(a, b)| super::corridor::distance(*a, *b) > 1e-9)
                .map(|(a, b)| ReservationTube {
                    a,
                    b,
                    ..piece.clone()
                }),
        );
    }
    Ok(clipped)
}

impl CorridorPlan {
    /// The source `Corridor.allowed` predicate for a point on a lane.
    pub fn allowed(&self, net: &str, s: f64, layer: &str) -> bool {
        if self.required.get(net).is_some_and(|intervals| {
            intervals
                .iter()
                .any(|interval| interval.start <= s && s <= interval.end && interval.layer != layer)
        }) {
            return false;
        }
        if layer == "B.Cu" {
            return self
                .back_windows
                .get(net)
                .is_some_and(|windows| windows.iter().any(|(lo, hi)| *lo <= s && s <= *hi));
        }
        true
    }
}

fn component_centre(context: &WholeContext, component: &str) -> Result<Point, String> {
    let pads: Vec<_> = context
        .pads
        .iter()
        .filter(|pad| pad.component_ref == component)
        .collect();
    if pads.is_empty() {
        return Err(format!("braid corridor component {component} has no pads"));
    }
    let (x, y) = pads.iter().fold((0.0, 0.0), |(x, y), pad| {
        (x + pad.point[0], y + pad.point[1])
    });
    Ok([x / pads.len() as f64, y / pads.len() as f64])
}

fn extend_corridor_spine(
    core: &Spine,
    teeth: &BTreeMap<String, Point>,
    stubs: &BTreeMap<String, Point>,
    context: &WholeContext,
    options: &BraidCorridorOptions,
    members: &[String],
) -> Result<Spine, String> {
    let p0 = *core
        .pts
        .first()
        .ok_or("braid corridor spine has no start")?;
    let pn = *core.pts.last().ok_or("braid corridor spine has no end")?;
    let d0 = *core
        .d
        .first()
        .ok_or("braid corridor spine has no start direction")?;
    let dn = *core
        .d
        .last()
        .ok_or("braid corridor spine has no end direction")?;
    let back = teeth
        .values()
        .map(|p| -super::corridor::dot(super::corridor::sub(*p, p0), d0) + 0.3)
        .fold(0.3, f64::max);
    let mut forward = stubs
        .values()
        .map(|p| super::corridor::dot(super::corridor::sub(*p, pn), dn) + 0.3)
        .fold(0.3, f64::max);
    let mut ball_s = Vec::new();
    let mut far = Vec::new();
    let mut along_spine = false;
    for net in members {
        let destination = &context
            .ends
            .get(net)
            .ok_or_else(|| format!("braid corridor net {net} has no ends"))?
            .destination;
        for pad in context
            .pads
            .iter()
            .filter(|pad| &pad.component_ref == destination)
        {
            ball_s.push(super::corridor::dot(
                super::corridor::sub(pad.point, pn),
                dn,
            ));
        }
        let stub = *stubs
            .get(net)
            .ok_or_else(|| format!("braid corridor net {net} has no berth"))?;
        far.push(super::corridor::dot(super::corridor::sub(stub, pn), dn));
        let direction = context
            .stub_dir
            .get(net)
            .ok_or_else(|| format!("braid corridor net {net} has no stub direction"))?;
        along_spine |= super::corridor::dot(*direction, dn) > 0.7;
    }
    if !ball_s.is_empty()
        && !far.is_empty()
        && (far.iter().copied().fold(f64::NEG_INFINITY, f64::max)
            > ball_s.iter().copied().fold(f64::NEG_INFINITY, f64::max) - 1e-6
            || along_spine)
    {
        forward = forward.max(
            ball_s
                .iter()
                .copied()
                .fold(f64::NEG_INFINITY, f64::max)
                .max(far.iter().copied().fold(f64::NEG_INFINITY, f64::max))
                + 10.0 * (options.track_width + options.clearance + 0.02),
        );
    }
    core.extend(back, forward)
}

fn classify(state: &mut CorridorState) -> Result<(), String> {
    let s_tolerance = state.classification_s_tolerance();
    let o_tolerance = state.classification_offset_tolerance();
    state.s0 = state
        .st
        .values()
        .map(|p| p[0])
        .fold(f64::NEG_INFINITY, f64::max)
        + 0.3;
    state.s0_base = state.s0;
    let mut heads_l = Vec::new();
    let mut joiners = Vec::new();
    let mut heads_e = Vec::new();
    for net in &state.members {
        let st = state.st[net];
        if state.is_branch {
            heads_l.push(net.clone());
        } else {
            let head_launch = !state.members.iter().any(|other| {
                other != net
                    && state.st[other][0] > st[0] + s_tolerance
                    && (state.st[other][1] - st[1]).abs() < o_tolerance
            }) && super::braid_geometry::obs_but(
                &state.context,
                net,
                &state.members,
                &state.context.tooth_layer[net],
            )?
            .seg_clear(state.teeth[net], state.spine.xy(state.s0, st[1]));
            if head_launch {
                heads_l.push(net.clone());
            } else {
                joiners.push(net.clone());
            }
        }

        if state.is_branch {
            continue;
        }

        let exit = state.se[net];
        let mut head_exit = !state.members.iter().any(|other| {
            other != net
                && state.se[other][0] < exit[0] - s_tolerance
                && (state.se[other][1] - exit[1]).abs() < o_tolerance
        });
        if head_exit {
            let start = state.spine.xy(exit[0] - state.head_run(), exit[1]);
            head_exit = state.pad_obstacles.seg_clear(start, state.stubs[net]);
        }
        if head_exit && state.options.pages_siders > 0 && state.options.pages_first {
            let dn = *state
                .spine
                .d
                .last()
                .ok_or("braid corridor spine has no arrival direction")?;
            let sd = state.context.stub_dir[net];
            let along = super::corridor::dot(sd, dn);
            if along > 0.7 || (state.options.pages_siders == 1 && along > -0.5) {
                head_exit = false;
            }
        }
        if head_exit {
            heads_e.push(net.clone());
        }
    }
    if state.is_branch {
        heads_e.clear();
        joiners.clear();
    }
    let siders: Vec<_> = state
        .members
        .iter()
        .filter(|net| !heads_e.contains(net))
        .cloned()
        .collect();
    let mut s1_candidates = Vec::new();
    if !heads_e.is_empty() {
        s1_candidates.push(
            heads_e
                .iter()
                .map(|net| state.se[net][0])
                .fold(f64::INFINITY, f64::min)
                - 0.6,
        );
    }
    if !siders.is_empty() {
        s1_candidates.push(
            siders
                .iter()
                .map(|net| state.se[net][0])
                .fold(f64::INFINITY, f64::min)
                - 0.3,
        );
    }
    let s1 = s1_candidates
        .into_iter()
        .reduce(f64::min)
        .ok_or("braid corridor has no arrival slot")?;
    state.s1 = s1;
    state.heads_l = heads_l;
    state.joiners = joiners;
    state.heads_e = heads_e;
    state.siders = siders;
    state.join_side.clear();
    state.exit_side.clear();
    for net in &state.joiners {
        let component = &state.context.src_ref[net];
        let centre = component_centre(&state.context, component)?;
        let offset = state.spine.project_pt(centre)[1];
        state
            .join_side
            .insert(net.clone(), if state.st[net][1] >= offset { 1 } else { -1 });
    }
    for net in &state.siders {
        let component = &state.context.ends[net].destination;
        let centre = component_centre(&state.context, component)?;
        let offset = state.spine.project_pt(centre)[1];
        state
            .exit_side
            .insert(net.clone(), if state.se[net][1] >= offset { 1 } else { -1 });
    }
    state.far_exit.clear();
    state.s_leg_min = None;
    state.s_ball = None;
    state.r_max = None;
    if !state.siders.is_empty() && !state.wrap {
        let refs: BTreeSet<_> = state
            .siders
            .iter()
            .map(|net| state.context.ends[net].destination.clone())
            .collect();
        let pads: Vec<_> = state
            .context
            .pads
            .iter()
            .filter(|pad| refs.contains(&pad.component_ref))
            .collect();
        if !pads.is_empty() {
            let s_ball = pads
                .iter()
                .map(|pad| state.spine.project_pt(pad.point)[0])
                .fold(f64::NEG_INFINITY, f64::max);
            let r_max = pads
                .iter()
                .map(|pad| pad.size[0].max(pad.size[1]) / 2.0)
                .fold(0.0, f64::max);
            state.s_ball = Some(s_ball);
            state.r_max = Some(r_max);
            for net in &state.siders {
                if state.se[net][0] > s_ball - 1e-6 {
                    state.far_exit.insert(net.clone());
                }
            }
            let dn = *state
                .spine
                .d
                .last()
                .ok_or("braid corridor spine has no arrival direction")?;
            for net in &state.siders {
                if super::corridor::dot(state.context.stub_dir[net], dn) > 0.7 {
                    state.far_exit.insert(net.clone());
                }
            }
            if !state.far_exit.is_empty() {
                state.s_leg_min = Some(
                    s_ball
                        + r_max
                        + state.options.clearance
                        + state.options.track_width / 2.0
                        + 0.05,
                );
            }
        }
    }
    Ok(())
}

fn reserve_intervals(state: &mut CorridorState) {
    state.reserved.clear();
    state.s_pts = vec![state.s0, state.s1];
    state.u_pts = vec![0.0, state.s1 - state.s0];
    state.l_free = state.s1 - state.s0;
}

/// Construct full mutable source Corridor states, retaining every geometry
/// field needed by later rung probes.
pub fn build_corridor_states(
    setup: &super::braid_setup::BraidSetupResult,
    options: &BraidCorridorOptions,
    exact_pages: bool,
    swimmer_page_price: f64,
) -> Result<Vec<CorridorState>, String> {
    options.validate()?;
    if !swimmer_page_price.is_finite() || swimmer_page_price < 0.0 {
        return Err("braid corridor swimmer page price must be finite and nonnegative".into());
    }
    let mut results = Vec::new();
    let mut laid = Vec::<(Vec<Point>, f64)>::new();
    for (index, members) in setup.groups.iter().enumerate() {
        let mut corridor_options = options.clone();
        corridor_options.pages_first = setup.pages_first;
        let half_width = corridor_options.lane_pitch * members.len().saturating_sub(1) as f64 / 2.0
            + corridor_options.lane_pitch
            + members
                .iter()
                .map(|net| lane_width(&setup.context, &corridor_options, net))
                .sum::<f64>();
        let core = super::braid_geometry::build_corridor_spine(
            &setup.context,
            members,
            &setup.spine_obstacles,
            &laid,
            Some(half_width),
            true,
        )?;
        let teeth: BTreeMap<_, _> = members
            .iter()
            .map(|net| {
                setup
                    .context
                    .ends
                    .get(net)
                    .map(|ends| (net.clone(), ends.tooth))
                    .ok_or_else(|| format!("braid corridor {index} member {net} has no tooth"))
            })
            .collect::<Result<_, _>>()?;
        let stubs: BTreeMap<_, _> = members
            .iter()
            .map(|net| {
                setup
                    .context
                    .ends
                    .get(net)
                    .map(|ends| (net.clone(), ends.berth))
                    .ok_or_else(|| format!("braid corridor {index} member {net} has no berth"))
            })
            .collect::<Result<_, _>>()?;
        let spine = extend_corridor_spine(
            &core,
            &teeth,
            &stubs,
            &setup.context,
            &corridor_options,
            members,
        )?;
        let st: BTreeMap<_, _> = members
            .iter()
            .map(|net| (net.clone(), spine.project_pt(teeth[net])))
            .collect();
        let se: BTreeMap<_, _> = members
            .iter()
            .map(|net| (net.clone(), spine.project_pt(stubs[net])))
            .collect();
        let mut state = CorridorState {
            index,
            members: members.clone(),
            context: setup.context.clone(),
            options: corridor_options,
            spine_obstacles: setup.spine_obstacles.clone(),
            pad_obstacles: setup.pad_obstacles.clone(),
            extra_tubes: laid.clone(),
            half_width,
            spine_core: core.clone(),
            spine,
            teeth,
            stubs,
            st,
            se,
            s0: 0.0,
            s0_base: 0.0,
            s1: 0.0,
            heads_l: Vec::new(),
            joiners: Vec::new(),
            heads_e: Vec::new(),
            siders: Vec::new(),
            join_side: BTreeMap::new(),
            exit_side: BTreeMap::new(),
            far_exit: BTreeSet::new(),
            pin_target: BTreeSet::new(),
            keep_order: false,
            wrap: false,
            is_branch: false,
            s_leg_min: None,
            s_ball: None,
            r_max: None,
            join_block: BTreeMap::new(),
            join_leg_s: BTreeMap::new(),
            exit_block: BTreeMap::new(),
            exit_leg_s: BTreeMap::new(),
            launch_o: BTreeMap::new(),
            target_o: BTreeMap::new(),
            launch: Vec::new(),
            target: Vec::new(),
            ly: Vec::new(),
            py: Vec::new(),
            slope: BTreeMap::new(),
            l_free: 0.0,
            s_pts: Vec::new(),
            u_pts: Vec::new(),
            reserved: Vec::new(),
            row_slot: BTreeMap::new(),
            row_side: BTreeMap::new(),
            row_up: BTreeMap::new(),
            req: BTreeMap::new(),
            back_windows: BTreeMap::new(),
            mid: BTreeMap::new(),
            legs: BTreeMap::new(),
            jogs: BTreeMap::new(),
            mid_xy: BTreeMap::new(),
            lanes: BTreeMap::new(),
            req_xy: BTreeMap::new(),
            leg_layer: BTreeMap::new(),
            leg_split: BTreeMap::new(),
            layer_profiles: BTreeMap::new(),
            sched_cur: None,
            swim_changes: BTreeMap::new(),
            islands_by_layer: BTreeMap::new(),
            islands: BTreeMap::new(),
            dive_s_plan: BTreeMap::new(),
            crossings: BTreeMap::new(),
            hops: BTreeMap::new(),
            deflect_islands: BTreeMap::new(),
            width: 0.0,
            layout_need: 0.0,
        };
        classify(&mut state)?;
        super::braid_offsets::offsets(&mut state, 0.35, None)?;
        reserve_intervals(&mut state);
        let mut schedule =
            super::braid_schedule::build_schedule(super::braid_schedule::ScheduleInput {
                launch: state.launch.clone(),
                target: state.target.clone(),
                tooth_layer: state.context.tooth_layer.clone(),
                dest_layer: state.context.dest_layer.clone(),
                exact_pages,
                swimmer_page_price,
            })?;
        if state.options.slope_pitch {
            super::braid_offsets::offsets(&mut state, 0.35, Some(&schedule))?;
            reserve_intervals(&mut state);
            schedule =
                super::braid_schedule::build_schedule(super::braid_schedule::ScheduleInput {
                    launch: state.launch.clone(),
                    target: state.target.clone(),
                    tooth_layer: state.context.tooth_layer.clone(),
                    dest_layer: state.context.dest_layer.clone(),
                    exact_pages,
                    swimmer_page_price,
                })?;
        }
        state.sched_cur = Some(schedule.clone());
        super::braid_lanes::lay_lanes(&mut state, &schedule)?;
        let mut profiles = BTreeMap::new();
        for net in &state.members {
            let profile = super::braid_lanes::layer_profile(&state, net)?;
            profiles.insert(net.clone(), profile);
        }
        state.layer_profiles = profiles;
        laid.push((state.spine_core.pts.clone(), state.half_width));
        results.push(state);
    }
    Ok(results)
}

impl CorridorState {
    pub fn band_context(&self) -> CorridorBandContext {
        CorridorBandContext {
            index: self.index,
            members: self.members.clone(),
            spine: self.spine.clone(),
            options: self.options.clone(),
            schedule: self.sched_cur.clone(),
            teeth: self.teeth.clone(),
            stubs: self.stubs.clone(),
            st: self.st.clone(),
            se: self.se.clone(),
            launch: self.launch.clone(),
            target: self.target.clone(),
            launch_o: self.launch_o.clone(),
            target_o: self.target_o.clone(),
            s0: self.s0,
            s1: self.s1,
            l_free: self.l_free,
            heads_l: self.heads_l.clone(),
            joiners: self.joiners.clone(),
            heads_e: self.heads_e.clone(),
            siders: self.siders.clone(),
            far_exit: self.far_exit.clone(),
            is_branch: self.is_branch,
            wrap: self.wrap,
            keep_order: self.keep_order,
            pin_target: self.pin_target.clone(),
            join_side: self.join_side.clone(),
            exit_side: self.exit_side.clone(),
            join_block: self.join_block.clone(),
            join_leg_s: self.join_leg_s.clone(),
            exit_block: self.exit_block.clone(),
            exit_leg_s: self.exit_leg_s.clone(),
            req: self.req.clone(),
            back_windows: self.back_windows.clone(),
            mid: self.mid.clone(),
            legs: self.legs.clone(),
            jogs: self.jogs.clone(),
            mid_xy: self.mid_xy.clone(),
            lanes: self.lanes.clone(),
            req_xy: self.req_xy.clone(),
            leg_layer: self.leg_layer.clone(),
            leg_split: self.leg_split.clone(),
            layer_profiles: self.layer_profiles.clone(),
            swim_changes: self.swim_changes.clone(),
            islands_by_layer: self.islands_by_layer.clone(),
            islands: self.islands.clone(),
            hops: self.hops.clone(),
            dive_s_plan: self.dive_s_plan.clone(),
            pair_names: if self.options.pairs {
                self.context.pairs.keys().cloned().collect()
            } else {
                BTreeSet::new()
            },
            pair_members: self.context.pairs.clone(),
            pair_ends: self.context.pair_ends.clone(),
            pair_layers: self.context.pair_layers.clone(),
            source_paths: self.context.paths.clone(),
            all_ends: self.context.ends.clone(),
            tooth_dir: self.context.tooth_dir.clone(),
            stub_dir: self.context.stub_dir.clone(),
            slope: self.slope.clone(),
            tooth_layer: self.context.tooth_layer.clone(),
            dest_layer: self.context.dest_layer.clone(),
            byname: self.context.byname.clone(),
            copper_layers: self.context.copper_layers.clone(),
            via_drill: self.context.cfg.via_drill,
        }
    }

    pub fn into_plan(self) -> Result<CorridorPlan, String> {
        let schedule = self
            .sched_cur
            .clone()
            .ok_or("completed corridor state has no schedule")?;
        Ok(CorridorPlan {
            index: self.index,
            members: self.members.clone(),
            spine: self.spine.clone(),
            launch: self.launch.clone(),
            target: self.target.clone(),
            schedule,
            lanes: self.lanes.clone(),
            mid: self.mid.clone(),
            required: self.req.clone(),
            back_windows: self.back_windows.clone(),
            exit_leg_layer: self.leg_layer.clone(),
            layer_profiles: self.layer_profiles.clone(),
            swim_changes: self.swim_changes.clone(),
            islands: self.islands.clone(),
            joiners: self.joiners.clone(),
            siders: self.siders.clone(),
            s0: self.s0,
            s1: self.s1,
            band: self.band_context(),
        })
    }
}

/// Preserve the original plan-only `CorridorPlan` API for D22/D19 while
/// allowing rung operators to retain the full source state through the sibling
/// [`build_corridor_states`] entry point.
pub fn build_corridors(
    setup: &super::braid_setup::BraidSetupResult,
    options: &BraidCorridorOptions,
    exact_pages: bool,
    swimmer_page_price: f64,
) -> Result<Vec<CorridorPlan>, String> {
    build_corridor_states(setup, options, exact_pages, swimmer_page_price)?
        .into_iter()
        .map(CorridorState::into_plan)
        .collect()
}
