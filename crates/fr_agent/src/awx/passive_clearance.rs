//! AWX beneath-only post-fanout placement engine from placement/fanout_clearance.py.
use super::{
    corridor::{self, Point},
    numeric,
    whole_context::ContextPad,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use crate::board_adapter::BoardAdapter;
use fr_geom::{Polyline, Vector};
const EPS: f64 = 1e-6;
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlacementPose {
    pub position: Point,
    pub rotation: f64,
    pub front: bool,
}
/// Configured pose assertion. Position is optional so an exported board can reuse the same
/// physical metadata while taking its seed from the current native placement.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ConfiguredPose {
    #[serde(default)]
    pub position: Option<Point>,
    pub rotation: f64,
    pub front: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ComponentGeometry {
    pub pose: ConfiguredPose,
    /// Footprint-local courtyard; when absent, the source footprint-local own box.
    pub courtyard: Option<[f64; 4]>,
    pub footprint_bounds: [f64; 4],
    /// Allowed cap-pad region beneath the target package for the single beneath-only pass.
    #[serde(default)]
    pub beneath: Option<[f64; 4]>,
    #[serde(default)]
    pub locked: bool,
    #[serde(default)]
    pub owns_board_outline: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PlacementChange {
    pub reference: String,
    pub before: PlacementPose,
    pub after: PlacementPose,
    pub pad_refs: Vec<String>,
}
#[derive(Clone, Debug)]
pub struct CapPad {
    pub key: String,
    pub offset: Point,
    pub half_size: Point,
    pub net_id: i32,
    pub layers: BTreeSet<String>,
    pub clearance_class: i32,
}
#[derive(Clone, Debug)]
pub struct Cap {
    pub reference: String,
    pub seed: PlacementPose,
    pub pose: PlacementPose,
    pub local_bounds: [f64; 4],
    pub pads: Vec<CapPad>,
    pub beneath: Option<[f64; 4]>,
}
fn rotate(p: Point, degrees: f64) -> Point {
    let a = degrees.to_radians();
    [
        a.cos() * p[0] - a.sin() * p[1],
        a.sin() * p[0] + a.cos() * p[1],
    ]
}
pub fn rotate_bounds(bounds: [f64; 4], rotation: f64) -> [f64; 4] {
    let q = [
        [bounds[0], bounds[1]],
        [bounds[0], bounds[3]],
        [bounds[2], bounds[1]],
        [bounds[2], bounds[3]],
    ]
    .map(|p| rotate(p, -rotation));
    [
        q.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min),
        q.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min),
        q.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max),
        q.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max),
    ]
}
fn translated(b: [f64; 4], p: Point) -> [f64; 4] {
    [b[0] + p[0], b[1] + p[1], b[2] + p[0], b[3] + p[1]]
}
pub fn inside(rect: [f64; 4], bounds: [f64; 4]) -> bool {
    rect[0] >= bounds[0] - 1e-3
        && rect[1] >= bounds[1] - 1e-3
        && rect[2] <= bounds[2] + 1e-3
        && rect[3] <= bounds[3] + 1e-3
}
pub fn rect_gap(a: [f64; 4], b: [f64; 4]) -> f64 {
    (b[0] - a[2])
        .max(a[0] - b[2])
        .max(0.)
        .hypot((b[1] - a[3]).max(a[1] - b[3]).max(0.))
}
pub fn point_rect(p: Point, b: [f64; 4]) -> f64 {
    (b[0] - p[0])
        .max(p[0] - b[2])
        .max(0.)
        .hypot((b[1] - p[1]).max(p[1] - b[3]).max(0.))
}
pub fn segment_rect(a: Point, b: Point, r: [f64; 4]) -> f64 {
    if inside([a[0], a[1], a[0], a[1]], r) || inside([b[0], b[1], b[0], b[1]], r) {
        return 0.;
    }
    let corners = [[r[0], r[1]], [r[2], r[1]], [r[2], r[3]], [r[0], r[3]]];
    (0..4)
        .map(|i| super::pairs::segment_distance(a, b, corners[i], corners[(i + 1) % 4]))
        .fold(f64::INFINITY, f64::min)
}
fn inside_outline(point: Point, polygons: &[Vec<Point>]) -> bool {
    polygons.iter().filter(|p| super::joint_geometry::point_in_polygon(point, p)).count() % 2 == 1
}
fn rect_outline_violation(rect: [f64; 4], polygons: &[Vec<Point>]) -> f64 {
    if polygons.is_empty() { return 0.; }
    let corners = [[rect[0],rect[1]],[rect[2],rect[1]],[rect[2],rect[3]],[rect[0],rect[3]]];
    let rect_edges: Vec<_> = (0..4).map(|i| (corners[i], corners[(i+1)%4])).collect();
    let mut violation = 0.;
    if corners.iter().any(|p| !inside_outline(*p, polygons)) { violation += 0.001; }
    for polygon in polygons {
        if polygon.iter().any(|p| p[0] >= rect[0] && p[0] <= rect[2] && p[1] >= rect[1] && p[1] <= rect[3]) {
            violation += 0.001;
        }
        for i in 0..polygon.len() {
            let a = polygon[i]; let b = polygon[(i+1)%polygon.len()];
            if rect_edges.iter().any(|(c,d)| super::pairs::segment_distance(a,b,*c,*d) <= EPS) {
                violation += 0.001;
                break;
            }
        }
    }
    violation
}
fn point_outline_clear(point: Point, radius: f64, polygons: &[Vec<Point>]) -> bool {
    polygons.is_empty() || (inside_outline(point, polygons) && polygons.iter().all(|p|
        (0..p.len()).all(|i| corridor::point_segment_distance(point, p[i], p[(i+1)%p.len()]) >= radius - EPS)))
}
fn segment_outline_clear(a: Point, b: Point, half_width: f64, polygons: &[Vec<Point>]) -> bool {
    polygons.is_empty() || (inside_outline(a, polygons) && inside_outline(b, polygons)
        && polygons.iter().all(|p| (0..p.len()).all(|i|
            super::pairs::segment_distance(a,b,p[i],p[(i+1)%p.len()]) >= half_width - EPS)))
}
impl Cap {
    pub fn from_pads(
        reference: &str,
        geometry: &ComponentGeometry,
        pads: &[ContextPad],
        classes: &BTreeMap<String, i32>,
    ) -> Result<Self, String> {
        let initial_position = geometry.pose.position.ok_or("passive seed position was not resolved from native board")?;
        if initial_position.iter().any(|x| !x.is_finite()) || !geometry.pose.rotation.is_finite() {
            return Err("invalid passive seed pose".into());
        }
        let local_bounds = geometry.courtyard.unwrap_or(geometry.footprint_bounds);
        if local_bounds.iter().any(|v| !v.is_finite())
            || local_bounds[0] > local_bounds[2]
            || local_bounds[1] > local_bounds[3]
        {
            return Err("invalid passive courtyard/footprint box".into());
        }
        let mut pp = Vec::new();
        for p in pads.iter().filter(|p| p.component_ref == reference && !p.layers.is_empty()) {
            let a = p.rect_rotation.to_radians();
            let hx = p.size[0] / 2.;
            let hy = p.size[1] / 2.;
            let half_size = [
                a.cos().abs() * hx + a.sin().abs() * hy,
                a.sin().abs() * hx + a.cos().abs() * hy,
            ];
            pp.push(CapPad {
                key: p.key.clone(),
                offset: corridor::sub(p.point, initial_position),
                half_size,
                net_id: p.net_id,
                layers: if p.non_plated_hole {
                    BTreeSet::new()
                } else {
                    p.layers.iter().cloned().collect()
                },
                clearance_class: *classes
                    .get(&p.key)
                    .ok_or("missing native pad clearance class")?,
            });
        }
        if pp.is_empty() { return Err(format!("passive component {reference} has no captured copper pads")); }
        Ok(Self {
            reference: reference.into(),
            seed: PlacementPose { position: initial_position, rotation: geometry.pose.rotation, front: geometry.pose.front },
            pose: PlacementPose { position: initial_position, rotation: geometry.pose.rotation, front: geometry.pose.front },
            local_bounds,
            pads: pp,
            beneath: geometry.beneath,
        })
    }
    pub fn rect(&self, pose: &PlacementPose) -> [f64; 4] {
        translated(
            rotate_bounds(self.local_bounds, pose.rotation),
            pose.position,
        )
    }
    pub fn pad_rects(&self, pose: &PlacementPose) -> Vec<([f64; 4], &CapPad)> {
        let delta = (pose.rotation - self.seed.rotation).rem_euclid(360.);
        let swap = numeric::checked_round_i64(delta)
            .expect("validated rotation")
            .rem_euclid(180)
            == 90;
        self.pads
            .iter()
            .map(|p| {
                let q = corridor::add(pose.position, rotate(p.offset, -delta));
                let h = if swap {
                    [p.half_size[1], p.half_size[0]]
                } else {
                    p.half_size
                };
                ([q[0] - h[0], q[1] - h[1], q[0] + h[0], q[1] + h[1]], p)
            })
            .collect()
    }
    pub fn pad_bbox(&self, pose: &PlacementPose) -> [f64; 4] {
        let pads = self.pad_rects(pose);
        if pads.is_empty() {
            return [
                pose.position[0],
                pose.position[1],
                pose.position[0],
                pose.position[1],
            ];
        }
        [
            pads.iter().map(|(b, _)| b[0]).fold(f64::INFINITY, f64::min),
            pads.iter().map(|(b, _)| b[1]).fold(f64::INFINITY, f64::min),
            pads.iter()
                .map(|(b, _)| b[2])
                .fold(f64::NEG_INFINITY, f64::max),
            pads.iter()
                .map(|(b, _)| b[3])
                .fold(f64::NEG_INFINITY, f64::max),
        ]
    }
    pub fn stays_beneath(&self, pose: &PlacementPose) -> bool {
        self.beneath.is_none_or(|b| inside(self.pad_bbox(pose), b))
    }
    pub fn candidate_positions(
        &self,
        max_displacement: f64,
        step: f64,
        grid: f64,
    ) -> Result<Vec<Point>, String> {
        if [max_displacement, step, grid]
            .iter()
            .any(|v| !v.is_finite())
            || max_displacement < 0.
            || step <= 0.
            || grid <= 0.
        {
            return Err("invalid passive displacement lattice".into());
        }
        let n = (max_displacement / step).floor() as i64;
        let mut seen = BTreeSet::new();
        let mut out = Vec::new();
        for ix in -n..=n {
            for iy in -n..=n {
                let offset = [
                    numeric::checked_round_i64(ix as f64 * step / grid)? as f64 * grid,
                    numeric::checked_round_i64(iy as f64 * step / grid)? as f64 * grid,
                ];
                if corridor::length(offset) > max_displacement + 1e-9 {
                    continue;
                }
                let p = corridor::add(self.seed.position, offset);
                let key = [
                    numeric::checked_round_i64(numeric::round_digits(p[0], 4)? * 1e4)?,
                    numeric::checked_round_i64(numeric::round_digits(p[1], 4)? * 1e4)?,
                ];
                if seen.insert(key) {
                    out.push(p);
                }
            }
        }
        Ok(out)
    }
    pub fn attraction(
        &self,
        pose: &PlacementPose,
        balls: &BTreeMap<i32, Vec<Point>>,
        capture: f64,
    ) -> f64 {
        self.pad_rects(pose)
            .iter()
            .filter_map(|(r, p)| {
                balls.get(&p.net_id).map(|balls| {
                    balls
                        .iter()
                        .map(|b| corridor::distance(*b, [(r[0] + r[2]) / 2., (r[1] + r[3]) / 2.]))
                        .fold(f64::INFINITY, f64::min)
                        .min(capture)
                })
            })
            .sum()
    }
}

/// Copper obstacles captured from the actual board. All dimensions are millimetres and layers
/// are native signal-layer names; callers must not synthesize component dimensions from a DSN
/// logical component label.
#[derive(Clone, Debug)]
pub enum CopperObstacle {
    Track { net_id: i32, layer: String, a: Point, b: Point, half_width: f64, clearance: f64, target_class: i32 },
    Pad { net_id: i32, layers: BTreeSet<String>, rect: [f64; 4], clearance: f64, target_class: i32 },
    Via { net_id: i32, layers: BTreeSet<String>, center: Point, radius: f64, clearance: f64, target_class: i32 },
    ComponentBody { reference: String, rect: [f64; 4] },
    BoardBounds { rect: [f64; 4] },
    BoardOutline { polygons: Vec<Vec<Point>> },
}

#[derive(Clone, Debug)]
pub struct RepairOptions {
    pub step: f64,
    pub grid: f64,
    pub displacement: f64,
    pub max_displacement: f64,
    pub growth: f64,
    pub max_passes: usize,
    pub capture: f64,
    pub beneath_only: bool,
}

impl Default for RepairOptions {
    fn default() -> Self {
        Self { step: 0.2, grid: 0.1, displacement: 2.0, max_displacement: 3.0,
            growth: 1.5, max_passes: 30, capture: 2.0, beneath_only: true }
    }
}

fn rect_circle_gap(rect: [f64; 4], center: Point, radius: f64) -> f64 {
    point_rect(center, rect) - radius
}

/// Per-net clearance deficit against foreign copper. Same-net copper is intentionally ignored.
pub fn shortfalls(cap: &Cap, pose: &PlacementPose, obstacles: &[CopperObstacle]) -> Shortfalls {
    let mut out = Shortfalls::default();
    for (pad_rect, pad) in cap.pad_rects(pose) {
        for obstacle in obstacles {
            let target_class = match obstacle {
                CopperObstacle::Track { target_class, .. }
                | CopperObstacle::Pad { target_class, .. }
                | CopperObstacle::Via { target_class, .. } => *target_class,
                CopperObstacle::ComponentBody { .. } | CopperObstacle::BoardBounds { .. } => continue,
                CopperObstacle::BoardOutline { .. } => continue,
            };
            if target_class != pad.clearance_class { continue; }
            let (net, layers, required) = match obstacle {
                CopperObstacle::Track { net_id, layer, half_width, clearance, .. } =>
                    (*net_id, BTreeSet::from([layer.clone()]), half_width + clearance),
                CopperObstacle::Pad { net_id, layers, clearance, .. } => (*net_id, layers.clone(), *clearance),
                CopperObstacle::Via { net_id, layers, radius, clearance, .. } => (*net_id, layers.clone(), radius + clearance),
                CopperObstacle::ComponentBody { .. } | CopperObstacle::BoardBounds { .. } => continue,
                CopperObstacle::BoardOutline { .. } => continue,
            };
            if pad.net_id == net || pad.layers.is_disjoint(&layers) { continue; }
            let gap = match obstacle {
                CopperObstacle::Track { a, b, .. } => segment_rect(*a, *b, pad_rect) - required,
                CopperObstacle::Pad { rect, .. } => rect_gap(pad_rect, *rect) - required,
                CopperObstacle::Via { center, .. } => rect_circle_gap(pad_rect, *center, required),
                CopperObstacle::ComponentBody { .. } | CopperObstacle::BoardBounds { .. } => continue,
                CopperObstacle::BoardOutline { .. } => continue,
            };
            if gap < 0. {
                let deficit = -gap;
                let by_net = match obstacle {
                    CopperObstacle::Track { .. } => &mut out.tracks,
                    CopperObstacle::Pad { .. } => &mut out.pads,
                    CopperObstacle::Via { .. } => &mut out.vias,
                    CopperObstacle::ComponentBody { .. } | CopperObstacle::BoardBounds { .. } => continue,
                    CopperObstacle::BoardOutline { .. } => continue,
                };
                *by_net.entry(net).or_default() += deficit;
            }
        }
    }
    // Physical courtyard/package exclusion is independent of copper nets and layers.
    let body = cap.rect(pose);
    for obstacle in obstacles {
        match obstacle {
            CopperObstacle::ComponentBody { reference, rect } if reference != &cap.reference => {
                let overlap_x = (body[2].min(rect[2]) - body[0].max(rect[0])).max(0.);
                let overlap_y = (body[3].min(rect[3]) - body[1].max(rect[1])).max(0.);
                if overlap_x > 0. && overlap_y > 0. {
                    *out.pads.entry(-1).or_default() += overlap_x.min(overlap_y);
                }
            }
            CopperObstacle::BoardBounds { rect } if !inside(body, *rect) => {
                let outside = (rect[0] - body[0]).max(0.) + (rect[1] - body[1]).max(0.)
                    + (body[2] - rect[2]).max(0.) + (body[3] - rect[3]).max(0.);
                *out.pads.entry(-2).or_default() += outside.max(EPS);
            }
            CopperObstacle::BoardOutline { polygons } => {
                let violation = rect_outline_violation(body, polygons);
                if violation > 0. { *out.pads.entry(-3).or_default() += violation; }
            }
            _ => {}
        }
    }
    out
}

/// AWX cost function from fanout_clearance.py: clearance deficits dominate movement cost,
/// which dominates loss of same-net attraction (50 / 1 / 0.3).
pub fn repair_cost(cap: &Cap, pose: &PlacementPose, seed: &PlacementPose,
    obstacles: &[CopperObstacle], balls: &BTreeMap<i32, Vec<Point>>, capture: f64) -> f64 {
    let sf = shortfalls(cap, pose, obstacles);
    50. * sf.graze()
        + corridor::distance(pose.position, seed.position)
        + 0.3 * (cap.attraction(seed, balls, capture) - cap.attraction(pose, balls, capture)).max(0.)
}

/// Deterministic one-arm repair with a displacement ladder, clear-position fallback, and the
/// source no-worse-per-net gate. The returned move must still be committed to the native board
/// through `apply_component_translation` so component geometry, pins, and the search tree agree.
pub fn repair_one_arm(cap: &Cap, obstacles: &[CopperObstacle], balls: &BTreeMap<i32, Vec<Point>>,
    options: &RepairOptions) -> Result<(PlacementPose, Shortfalls), String> {
    if options.growth <= 1. || options.max_displacement < 0. || options.displacement < 0.
        || options.max_passes == 0 { return Err("invalid passive repair options".into()); }
    let seed_score = shortfalls(cap, &cap.seed, obstacles);
    let mut pose = cap.seed.clone();
    let mut budget = options.displacement.min(options.max_displacement);
    for _ in 0..options.max_passes {
        let current = repair_cost(cap, &pose, &cap.seed, obstacles, balls, options.capture);
        let mut best = (current, pose.clone());
        for pos in cap.candidate_positions(budget, options.step, options.grid)? {
            let candidate = PlacementPose { position: pos, ..pose.clone() };
            if (options.beneath_only && !cap.stays_beneath(&candidate))
                || shortfalls(cap, &candidate, obstacles).worsens(&seed_score) { continue; }
            let score = repair_cost(cap, &candidate, &cap.seed, obstacles, balls, options.capture);
            if score < best.0 - EPS { best = (score, candidate); }
        }
        if best.0 < current - EPS { pose = best.1; continue; }
        if shortfalls(cap, &pose, obstacles).graze() <= EPS || budget >= options.max_displacement - EPS { break; }
        budget = (budget * options.growth).min(options.max_displacement);
    }
    // Source fallback: if weighted descent leaves foreign-copper contact, choose the cheapest
    // fully clear pose within the full displacement ladder while preserving the no-worse gate.
    if shortfalls(cap, &pose, obstacles).graze() > EPS {
        let mut best: Option<(f64, PlacementPose)> = None;
        for pos in cap.candidate_positions(options.max_displacement, options.step, options.grid)? {
            let candidate = PlacementPose { position: pos, ..pose.clone() };
            if (options.beneath_only && !cap.stays_beneath(&candidate))
                || shortfalls(cap, &candidate, obstacles).worsens(&seed_score)
                || shortfalls(cap, &candidate, obstacles).graze() > EPS { continue; }
            let score = repair_cost(cap, &candidate, &cap.seed, obstacles, balls, options.capture);
            if best.as_ref().is_none_or(|(v, _)| score < *v - EPS) { best = Some((score, candidate)); }
        }
        if let Some((_, clear)) = best { pose = clear; }
    }
    Ok((pose.clone(), shortfalls(cap, &pose, obstacles)))
}

#[derive(Clone, Debug, Serialize)]
pub struct PlacementPass {
    /// At most one reference is changed in this pass. The caller should keep this plan held
    /// while the fanout driver grades/rebuilds its route context.
    pub moved: Option<PlacementChange>,
    pub held: Vec<String>,
    pub unresolved: BTreeMap<String, ShortfallReport>,
}

#[derive(Clone, Debug, Serialize)]
pub struct ShortfallReport {
    pub tracks: BTreeMap<i32, f64>,
    pub pads: BTreeMap<i32, f64>,
    pub vias: BTreeMap<i32, f64>,
}
impl From<Shortfalls> for ShortfallReport {
    fn from(s: Shortfalls) -> Self { Self { tracks: s.tracks, pads: s.pads, vias: s.vias } }
}

/// Apply at most one beneath-only passive move to the actual native board, then return a held
/// decision. `caps` must be built from explicit package geometry and native pads; the function
/// intentionally has no fallback for missing physical metadata.
pub fn repair_board_once(board: &mut BoardAdapter, caps: &[Cap], obstacles: &[CopperObstacle],
    balls: &BTreeMap<i32, Vec<Point>>, mut options: RepairOptions) -> Result<PlacementPass, String> {
    options.beneath_only = true;
    let mut held = Vec::new();
    let mut unresolved = BTreeMap::new();
    let mut selected: Option<(&Cap, PlacementPose)> = None;
    for cap in caps {
        held.push(cap.reference.clone());
        let (after, remaining) = repair_one_arm(cap, obstacles, balls, &options)?;
        if remaining.graze() > EPS { unresolved.insert(cap.reference.clone(), remaining.clone().into()); }
        let delta = corridor::sub(after.position, cap.pose.position);
        if selected.is_none() && corridor::length(delta) > EPS { selected = Some((cap, after)); }
    }
    let moved = if let Some((cap, after)) = selected {
        apply_component_translation(board, &cap.reference, after.position)?;
        Some(PlacementChange { reference: cap.reference.clone(), before: cap.pose.clone(), after,
            pad_refs: cap.pads.iter().map(|p| p.key.clone()).collect() })
    } else { None };
    Ok(PlacementPass { moved, held, unresolved })
}

/// Build passive candidates only from explicitly configured physical bounds and the captured
/// native pad geometry. The board remains authoritative for pose/lock state; configuration must
/// agree with the loaded native placement within one native unit.
pub fn configured_caps(board: &BoardAdapter,
    geometry: &BTreeMap<String, ComponentGeometry>, pads: &[ContextPad],
    classes: &BTreeMap<String, i32>) -> Result<Vec<Cap>, String> {
    let mut out = Vec::new();
    for (reference, configured) in geometry {
        let native = board.board.components.get_by_name(reference)
            .ok_or_else(|| format!("passive geometry references missing component {reference}"))?;
        let loc = native.get_location().ok_or_else(|| format!("passive component {reference} is unplaced"))?.to_float();
        let pose = PlacementPose {
            position: [loc.x / board.units_per_mm, loc.y / board.units_per_mm],
            rotation: native.get_rotation_in_degree(),
            front: native.placed_on_front(),
        };
        let tolerance = 1.01 / board.units_per_mm;
        if configured.pose.position.is_some_and(|p| corridor::distance(pose.position, p) > tolerance)
            || (pose.rotation - configured.pose.rotation).abs() > 1e-6
            || pose.front != configured.pose.front {
            return Err(format!("configured passive pose for {reference} differs from native board"));
        }
        let mut physical = configured.clone();
        physical.pose.position = Some(pose.position);
        physical.locked |= native.position_fixed;
        if physical.locked { continue; }
        if physical.beneath.is_none() {
            return Err(format!("passive geometry for {reference} lacks an explicit beneath region"));
        }
        out.push(Cap::from_pads(reference, &physical, pads, classes)?);
    }
    Ok(out)
}

#[derive(Clone, Debug)]
pub struct CapturedObstacles {
    pub bounds: [f64; 4],
    /// Exact native board outline rings; even/odd nesting preserves cutouts.
    pub board_outline: Vec<Vec<Point>>,
    pub obstacles: Vec<CopperObstacle>,
    /// Native per-pad clearance classes keyed by the same full reference stored in ContextPad.
    pub pad_classes: BTreeMap<String, i32>,
}

/// Merge the movable-passive geometry table with the optional all-component body-bound table;
/// explicit `component_body_bounds` entries take precedence for a reference.
pub fn merge_body_bounds(passive_geometry: &BTreeMap<String, ComponentGeometry>,
    component_body_bounds: &BTreeMap<String, [f64; 4]>) -> BTreeMap<String, [f64; 4]> {
    let mut out: BTreeMap<_, _> = passive_geometry.iter().map(|(reference, g)|
        (reference.clone(), g.courtyard.unwrap_or(g.footprint_bounds))).collect();
    out.extend(component_body_bounds.iter().map(|(k, v)| (k.clone(), *v)));
    out
}

/// Capture actual board copper and per-pair native clearances. Pad/via shapes use their native
/// layer geometry bounding boxes, so rotated/custom shapes are conservatively enclosed. The board
/// outline/cutout polygons remain available from BoardSnapshot and must also gate final candidates.
pub fn capture_obstacles(board: &BoardAdapter) -> Result<CapturedObstacles, String> {
    capture_obstacles_with_bodies(board, &BTreeMap::new())
}

/// Capture obstacle copper plus each placed component's world-space body box. Native component
/// outlines are used when present; otherwise an explicit local body box is required by reference.
pub fn capture_obstacles_with_bodies(board: &BoardAdapter,
    explicit_bodies: &BTreeMap<String, [f64; 4]>) -> Result<CapturedObstacles, String> {
    let b = &board.board;
    let scale = board.units_per_mm;
    let bb = b.bounding_box();
    let bounds = [bb.ll.x as f64 / scale, bb.ll.y as f64 / scale,
        bb.ur.x as f64 / scale, bb.ur.y as f64 / scale];
    let board_outline: Vec<Vec<Point>> = b.get_outline().and_then(|k| b.item(k).as_board_outline())
        .map(|outline| outline.shapes().iter().map(|shape| shape.corner_approx_arr().iter()
            .map(|p| [p.x / scale, p.y / scale]).collect()).collect())
        .unwrap_or_default();
    let matrix = &b.rules.clearance_matrix;
    let mut target_classes = Vec::new();
    for cl in 0..1024 {
        if matrix.get_name(cl).is_none() { break; }
        target_classes.push(cl);
    }
    if target_classes.is_empty() { return Err("native board has no clearance classes".into()); }
    let layer_name = |idx: i32| -> Result<String, String> {
        b.layer_structure.layers.get(idx as usize).map(|l| l.name.clone())
            .ok_or_else(|| format!("native item uses invalid signal layer {idx}"))
    };
    let mm_point = |p: fr_geom::Point| { let f = p.to_float(); [f.x / scale, f.y / scale] };
    let mut obstacles = Vec::new();
    let mut pad_classes = BTreeMap::new();
    for pad in board.pads() {
        if let Some(key) = pad.key { pad_classes.insert(pad.reference.clone(), b.item(key).clearance_class()); }
    }
    for key in b.get_items() {
        let item = b.item(key);
        if item.net_numbers().is_empty() { continue; }
        let net = item.net_numbers()[0];
        if item.is_trace() {
            let trace = item.trace();
            let layer = trace.layer();
            let layer_string = layer_name(layer)?;
            let half_width = trace.half_width() as f64 / scale;
            let corners = trace.polyline().corners();
            for pair in corners.windows(2) {
                let a = mm_point(pair[0].clone()); let c = mm_point(pair[1].clone());
                for &target in &target_classes {
                    let clearance = matrix.get_value(target, item.clearance_class(), layer, false) as f64 / scale;
                    obstacles.push(CopperObstacle::Track { net_id: net, layer: layer_string.clone(), a, b: c,
                        half_width, clearance, target_class: target });
                }
            }
        } else if item.is_via() {
            let center = mm_point(item.center(b));
            let first = item.first_layer(b); let last = item.last_layer(b);
            let mut layers = BTreeSet::new(); let mut radius: f64 = 0.;
            for layer in first..=last {
                layers.insert(layer_name(layer)?);
                if let Some(shape) = item.drill_shape_on_layer(b, layer) {
                    radius = radius.max(shape.bounding_box().max_width() as f64 / (2. * scale));
                }
            }
            for &target in &target_classes {
                let clearance = (first..=last).map(|layer|
                    matrix.get_value(target, item.clearance_class(), layer, false) as f64 / scale)
                    .fold(0., f64::max);
                obstacles.push(CopperObstacle::Via { net_id: net, layers: layers.clone(), center, radius,
                    clearance, target_class: target });
            }
        } else if item.is_pin() {
            let first = item.first_layer(b); let last = item.last_layer(b);
            for layer in first..=last {
                let Some(shape) = item.drill_shape_on_layer(b, layer) else { continue; };
                let rect = shape.bounding_box();
                let rect = [rect.ll.x as f64 / scale, rect.ll.y as f64 / scale,
                    rect.ur.x as f64 / scale, rect.ur.y as f64 / scale];
                let layers = BTreeSet::from([layer_name(layer)?]);
                for &target in &target_classes {
                    let clearance = matrix.get_value(target, item.clearance_class(), layer, false) as f64 / scale;
                    obstacles.push(CopperObstacle::Pad { net_id: net, layers: layers.clone(), rect,
                        clearance, target_class: target });
                }
            }
        }
    }
    let mut native_bodies = BTreeMap::<String, [f64; 4]>::new();
    for key in b.get_items() {
        let item = b.item(key);
        if !item.is_component_outline() || item.component_no() <= 0 { continue; }
        let reference = b.components.get(item.component_no()).name.clone();
        let q = item.as_component_outline().expect("component-outline kind").get_area(b).bounding_box();
        let r = [q.ll.x as f64 / scale, q.ll.y as f64 / scale, q.ur.x as f64 / scale, q.ur.y as f64 / scale];
        native_bodies.entry(reference).and_modify(|old| {
            old[0] = old[0].min(r[0]); old[1] = old[1].min(r[1]); old[2] = old[2].max(r[2]); old[3] = old[3].max(r[3]);
        }).or_insert(r);
    }
    let component_ids: BTreeSet<_> = b.get_items().iter().map(|k| b.item(*k).component_no())
        .filter(|id| *id > 0).collect();
    for id in component_ids {
        let component = b.components.get(id);
        let reference = &component.name;
        let rect = if let Some(local) = explicit_bodies.get(reference) {
            if local.iter().any(|v| !v.is_finite()) || local[0] > local[2] || local[1] > local[3] {
                return Err(format!("invalid explicit body bounds for {reference}"));
            }
            let loc = component.get_location().ok_or_else(|| format!("component {reference} is unplaced"))?.to_float();
            translated(rotate_bounds(*local, component.get_rotation_in_degree()),
                [loc.x / scale, loc.y / scale])
        } else if let Some(native) = native_bodies.get(reference) {
            *native
        } else {
            return Err(format!("component {reference} has no native outline; explicit component_body_bounds metadata is required"));
        };
        obstacles.push(CopperObstacle::ComponentBody { reference: reference.clone(), rect });
    }
    obstacles.push(CopperObstacle::BoardBounds { rect: bounds });
    if !board_outline.is_empty() { obstacles.push(CopperObstacle::BoardOutline { polygons: board_outline.clone() }); }
    Ok(CapturedObstacles { bounds, board_outline, obstacles, pad_classes })
}

#[derive(Clone, Debug, PartialEq)]
pub struct DrillKeepout { pub center: Point, pub radius: f64, pub required: f64 }
#[derive(Clone, Debug, PartialEq)]
pub struct ViaNudgeDrillFloors {
    pub board_via: f64,
    pub board_pad: f64,
    pub fab_via: f64,
    pub fab_pad: f64,
    /// Explicitly identifies the configured board-rule input used for both board floors.
    pub board_source: &'static str,
}
/// Resolve the #756 two-rung rule: the configured board hole-to-hole rule is
/// raised to the selected fabrication floor, then that fabrication floor remains
/// available as the fallback rung. The board has one hole-to-hole rule for both
/// via/via and via/pad pairs; pad_hole_to_hole is fabrication-only.
pub fn resolve_via_nudge_drill_floors(
    board_hole_floor: f64,
    fab: &super::fab_rules::FabFloor,
) -> Result<ViaNudgeDrillFloors, String> {
    let values = [board_hole_floor, fab.hole_to_hole, fab.pad_hole_to_hole];
    if values.iter().any(|v| !v.is_finite() || *v < 0.0) {
        return Err("via-nudge drill floors must be finite and non-negative".into());
    }
    Ok(ViaNudgeDrillFloors {
        board_via: board_hole_floor.max(fab.hole_to_hole),
        board_pad: board_hole_floor.max(fab.pad_hole_to_hole),
        fab_via: fab.hole_to_hole,
        fab_pad: fab.pad_hole_to_hole,
        board_source: "awx.rules.hole_to_hole_clearance",
    })
}
#[derive(Clone, Debug)]
pub struct ViaNudgeProblem {
    pub center: Point,
    pub net_id: i32,
    pub barrel_radius: f64,
    pub drill_radius: f64,
    pub layers: BTreeSet<String>,
    /// Layer name to actual attached fanout-stub width. Each returned connector restores one
    /// of these same-net layer connections from the old center to the new via center.
    pub connector_widths: BTreeMap<String, f64>,
    pub board_bounds: [f64; 4],
    pub board_outline: Vec<Vec<Point>>,
    pub max_shift: f64,
    pub obstacles: Vec<CopperObstacle>,
    /// Optional .kicad_dru track-rule obstacles; tried first and relaxed only if no move exists.
    pub track_rule_obstacles: Vec<CopperObstacle>,
    pub cap_keepouts: Vec<(i32, BTreeSet<String>, [f64; 4], f64)>,
    /// Board-declared drill keepouts, preferred before fab-floor fallback keepouts.
    pub drill_keepouts: Vec<DrillKeepout>,
    pub fab_drill_keepouts: Vec<DrillKeepout>,
}
#[derive(Clone, Debug, Serialize)]
pub struct ViaConnector { pub a: Point, pub b: Point, pub layer: String, pub net_id: i32, pub width: f64 }
#[derive(Clone, Debug, Serialize)]
pub struct ViaNudge { pub from: Point, pub to: Point, pub net_id: i32, pub connectors: Vec<ViaConnector> }

/// Source-shaped #313 via last resort: radial rings at 0.05 mm and 16 directions, innermost
/// legal landing first. It validates the relocated barrel, drill-to-drill spacing, cap copper,
/// and every back-connector against layer-matched foreign copper before returning a move.
pub fn plan_via_nudge(problem: &ViaNudgeProblem) -> Result<Option<ViaNudge>, String> {
    if !problem.max_shift.is_finite() || problem.max_shift < 0.
        || !problem.barrel_radius.is_finite() || problem.barrel_radius <= 0.
        || !problem.drill_radius.is_finite() || problem.drill_radius < 0.
        || problem.center.iter().chain(problem.board_bounds.iter()).any(|v| !v.is_finite())
        || problem.drill_keepouts.iter().chain(problem.fab_drill_keepouts.iter()).any(|d|
            d.center.iter().any(|v| !v.is_finite()) || !d.radius.is_finite() || d.radius < 0.
                || !d.required.is_finite() || d.required < 0.) {
        return Err("invalid via-nudge geometry".into());
    }
    let old = problem.center;
    let drill_rungs: Vec<_> = if problem.drill_keepouts == problem.fab_drill_keepouts {
        vec![&problem.drill_keepouts]
    } else { vec![&problem.drill_keepouts, &problem.fab_drill_keepouts] };
    let strict_tracks = !problem.track_rule_obstacles.is_empty();
    for strict in if strict_tracks { vec![true, false] } else { vec![true] } {
      for drills in &drill_rungs {
       let mut radius = 0.05;
       while radius <= problem.max_shift + 1e-9 {
        for k in 0..16 {
            let angle = k as f64 * std::f64::consts::PI / 8.;
            let new = [numeric::round_digits(old[0] + radius * angle.cos(), 4)?,
                       numeric::round_digits(old[1] + radius * angle.sin(), 4)?];
            if new == old || new[0] - problem.barrel_radius < problem.board_bounds[0]
                || new[1] - problem.barrel_radius < problem.board_bounds[1]
                || new[0] + problem.barrel_radius > problem.board_bounds[2]
                || new[1] + problem.barrel_radius > problem.board_bounds[3] { continue; }
            if !point_outline_clear(new, problem.barrel_radius, &problem.board_outline) { continue; }
            if drills.iter().any(|o| corridor::distance(new, o.center)
                < problem.drill_radius + o.radius + o.required - EPS) { continue; }
            let mut obstacles = problem.obstacles.clone();
            if strict { obstacles.extend(problem.track_rule_obstacles.iter().cloned()); }
            if obstacles.iter().any(|o| match o {
                CopperObstacle::Track { net_id, layer, a, b, half_width, clearance, .. } =>
                    *net_id != problem.net_id && problem.layers.contains(layer)
                        && corridor::point_segment_distance(new, *a, *b)
                            < problem.barrel_radius + half_width + clearance - EPS,
                CopperObstacle::Pad { net_id, layers, rect, clearance, .. } =>
                    *net_id != problem.net_id && !problem.layers.is_disjoint(layers)
                        && point_rect(new, *rect) < problem.barrel_radius + clearance - EPS,
                CopperObstacle::Via { net_id, layers, center, radius, clearance, .. } =>
                    *net_id != problem.net_id && !problem.layers.is_disjoint(layers)
                        && corridor::distance(new, *center)
                            < problem.barrel_radius + radius + clearance - EPS,
                CopperObstacle::ComponentBody { .. } | CopperObstacle::BoardBounds { .. }
                | CopperObstacle::BoardOutline { .. } => false,
            }) { continue; }
            if problem.cap_keepouts.iter().any(|(net, layers, rect, required)| {
                *net != problem.net_id && !problem.layers.is_disjoint(layers)
                    && point_rect(new, *rect) < problem.barrel_radius + required - EPS
            }) { continue; }
            let connectors: Vec<_> = problem.connector_widths.iter().map(|(layer, width)| ViaConnector {
                a: old, b: new, layer: layer.clone(), net_id: problem.net_id, width: *width,
            }).collect();
            if connectors.iter().any(|c| obstacles.iter().any(|o| match o {
                CopperObstacle::Track { net_id, layer, a, b, half_width, clearance, .. } =>
                    *net_id != c.net_id && layer == &c.layer
                        && segment_distance(c.a, c.b, *a, *b) < c.width/2. + half_width + clearance - EPS,
                CopperObstacle::Pad { net_id, layers, rect, clearance, .. } =>
                    *net_id != c.net_id && layers.contains(&c.layer)
                        && segment_rect(c.a, c.b, *rect) < c.width/2. + clearance - EPS,
                CopperObstacle::Via { net_id, layers, center, radius, clearance, .. } =>
                    *net_id != c.net_id && layers.contains(&c.layer)
                        && corridor::point_segment_distance(*center, c.a, c.b) < c.width/2. + radius + clearance - EPS,
                CopperObstacle::ComponentBody { .. } | CopperObstacle::BoardBounds { .. }
                | CopperObstacle::BoardOutline { .. } => false,
            })) { continue; }
            if connectors.iter().any(|c| !segment_outline_clear(c.a, c.b, c.width / 2., &problem.board_outline)) { continue; }
            return Ok(Some(ViaNudge { from: old, to: new, net_id: problem.net_id, connectors }));
        }
        radius += 0.05;
       }
      }
    }
    Ok(None)
}

/// Populate a via-nudge request from an actual native via and the currently moved cap. The
/// caller supplies resolved board/fab drill-to-drill floors because this engine does not model
/// KiCad's min_hole_to_hole rule in RoutingRules.
pub fn capture_via_nudge_problem(board: &BoardAdapter, via_key: fr_engine::board::ItemKey,
    cap: &Cap, cap_pose: &PlacementPose, captured: &CapturedObstacles,
    floors: &ViaNudgeDrillFloors, max_shift: f64) -> Result<ViaNudgeProblem, String> {
    if !max_shift.is_finite() || max_shift < 0. {
        return Err("via-nudge max_shift must be finite and non-negative".into());
    }
    if [floors.board_via, floors.board_pad, floors.fab_via, floors.fab_pad]
        .iter().any(|v| !v.is_finite() || *v < 0.) {
        return Err("via-nudge drill floors must be finite and non-negative".into());
    }
    let b = &board.board;
    let via = b.item(via_key);
    if !via.is_via() || via.net_numbers().len() != 1 { return Err("via-nudge requires one native-net via".into()); }
    let net_id = via.net_numbers()[0];
    let center = {
        let p = via.center(b).to_float();
        [p.x / board.units_per_mm, p.y / board.units_per_mm]
    };
    let mut layers = BTreeSet::new();
    let first = via.first_layer(b); let last = via.last_layer(b);
    let mut barrel_radius: f64 = 0.;
    for layer in first..=last {
        layers.insert(b.layer_structure.layers[layer as usize].name.clone());
        if let Some(shape) = via.drill_shape_on_layer(b, layer) {
            barrel_radius = barrel_radius.max(shape.bounding_box().max_width() as f64 / (2. * board.units_per_mm));
        }
    }
    let drill_radius = via.padstack(b).ok_or("via padstack unavailable")?.get_drill_radius() / board.units_per_mm;
    if barrel_radius <= 0. { return Err("native via has no measurable copper barrel".into()); }
    let mut connector_widths = BTreeMap::<String, f64>::new();
    let connect_tol = barrel_radius.max(0.001);
    for key in b.get_items() {
        let item = b.item(key);
        if item.is_trace() && item.net_numbers() == [net_id] {
            let t = item.trace();
            let corners = t.polyline().corners();
            if corners.len() < 2 { continue; }
            let to_mm = |p: fr_geom::Point| { let q = p.to_float(); [q.x / board.units_per_mm, q.y / board.units_per_mm] };
            let a = to_mm(corners[0].clone()); let z = to_mm(corners[corners.len()-1].clone());
            if corridor::distance(a, center) <= connect_tol || corridor::distance(z, center) <= connect_tol {
                let layer = b.layer_structure.layers[t.layer() as usize].name.clone();
                let width = 2. * t.half_width() as f64 / board.units_per_mm;
                connector_widths.entry(layer).and_modify(|w| *w = w.min(width)).or_insert(width);
            }
        }
    }
    // A via inside a same-net pad must reconnect the pad's copper layers as well. Do not add
    // connectors on every via-span layer: only actual attached stubs and via-in-pad layers count.
    let fallback_width = b.rules.net_class_of(net_id).get_trace_half_width(first) as f64 * 2. / board.units_per_mm;
    let native_center = board.point(center)?;
    for key in b.get_items() {
        let item = b.item(key);
        if item.is_conduction_area() && item.net_numbers() == [net_id] {
            let area = item.as_conduction_area().expect("conduction-area kind").area();
            if area.get_area(b).contains(&native_center) {
                let layer = b.layer_structure.layers[area.layer() as usize].name.clone();
                let width = connector_widths.values().copied().fold(fallback_width, f64::min);
                connector_widths.entry(layer).or_insert(width);
            }
        }
        if !item.is_pin() || item.net_numbers() != [net_id] { continue; }
        let mut contained_layers = Vec::new();
        for layer in item.first_layer(b)..=item.last_layer(b) {
            if item.drill_shape_on_layer(b, layer).is_some_and(|s| s.contains(&native_center)) {
                contained_layers.push(b.layer_structure.layers[layer as usize].name.clone());
            }
        }
        if !contained_layers.is_empty() {
            let width = connector_widths.values().copied().fold(fallback_width, f64::min);
            for layer in contained_layers { connector_widths.entry(layer).or_insert(width); }
        }
    }
    let target_class = via.clearance_class();
    let obstacles = captured.obstacles.iter().filter(|o| match o {
        CopperObstacle::Track { target_class: c, .. }
        | CopperObstacle::Pad { target_class: c, .. }
        | CopperObstacle::Via { target_class: c, .. } => *c == target_class,
        CopperObstacle::ComponentBody { .. } | CopperObstacle::BoardBounds { .. }
        | CopperObstacle::BoardOutline { .. } => false,
    }).cloned().collect();
    let mut cap_keepouts = Vec::new();
    for (rect, pad) in cap.pad_rects(cap_pose) {
        let pad_layers: BTreeSet<_> = pad.layers.intersection(&layers).cloned().collect();
        if pad_layers.is_empty() { continue; }
        let required = pad_layers.iter().filter_map(|name| {
            let index = b.layer_structure.get_no(name);
            (index >= 0).then(|| b.rules.clearance_matrix.get_value(pad.clearance_class,
                target_class, index, false) as f64 / board.units_per_mm)
        }).fold(0., f64::max);
        cap_keepouts.push((pad.net_id, pad_layers, rect, required));
    }
    let mut board_holes = Vec::new(); let mut fab_holes = Vec::new();
    for key in b.get_items() {
        let item = b.item(key);
        if key == via_key || (!item.is_via() && !item.is_pin()) { continue; }
        let Some(stack) = item.padstack(b) else { continue; };
        let radius = stack.get_drill_radius() / board.units_per_mm;
        if radius <= 0. { continue; }
        let p = item.center(b).to_float();
        let center = [p.x / board.units_per_mm, p.y / board.units_per_mm];
        let (board_required, fab_required) = if item.is_via() {
            (floors.board_via, floors.fab_via)
        } else {
            (floors.board_pad, floors.fab_pad)
        };
        board_holes.push(DrillKeepout { center, radius, required: board_required });
        fab_holes.push(DrillKeepout { center, radius, required: fab_required });
    }
    Ok(ViaNudgeProblem { center, net_id, barrel_radius, drill_radius, layers, connector_widths,
        board_bounds: captured.bounds, board_outline: captured.board_outline.clone(), max_shift, obstacles, track_rule_obstacles: Vec::new(),
        cap_keepouts, drill_keepouts: board_holes, fab_drill_keepouts: fab_holes })
}

/// Return native via keys currently shorting a cap pad at the supplied pose. This is the exact
/// offender selection used before the via-nudge last resort; unrelated vias are never considered.
pub fn offending_vias(board: &BoardAdapter, cap: &Cap, pose: &PlacementPose)
    -> Vec<fr_engine::board::ItemKey> {
    let b = &board.board;
    let mut result = BTreeSet::new();
    for key in b.get_items() {
        let via = b.item(key);
        if !via.is_via() || via.net_numbers().len() != 1 { continue; }
        let via_net = via.net_numbers()[0];
        let center_native = via.center(b);
        let center_f = center_native.to_float();
        let center = [center_f.x / board.units_per_mm, center_f.y / board.units_per_mm];
        let mut radius: f64 = 0.;
        let mut layers = BTreeSet::new();
        for layer in via.first_layer(b)..=via.last_layer(b) {
            layers.insert(b.layer_structure.layers[layer as usize].name.clone());
            if let Some(shape) = via.drill_shape_on_layer(b, layer) {
                radius = radius.max(shape.bounding_box().max_width() as f64 / (2. * board.units_per_mm));
            }
        }
        if radius <= 0. { continue; }
        for (rect, pad) in cap.pad_rects(pose) {
            if pad.net_id == via_net || pad.layers.is_disjoint(&layers) { continue; }
            let required = pad.layers.intersection(&layers).filter_map(|name| {
                let layer = b.layer_structure.get_no(name);
                (layer >= 0).then(|| b.rules.clearance_matrix.get_value(pad.clearance_class,
                    via.clearance_class(), layer, false) as f64 / board.units_per_mm)
            }).fold(0., f64::max);
            if point_rect(center, rect) < radius + required - EPS {
                result.insert(key);
                break;
            }
        }
    }
    result.into_iter().collect()
}

fn segment_distance(a: Point, b: Point, c: Point, d: Point) -> f64 {
    super::pairs::segment_distance(a, b, c, d)
}

/// Commit a planned via nudge on the native board. The existing stub stays anchored at its old
/// point; new same-net connectors are inserted on every listed layer, matching the source pass.
pub fn apply_via_nudge(board: &mut BoardAdapter, via_key: fr_engine::board::ItemKey,
    move_: &ViaNudge) -> Result<(), String> {
    let mut staged = board.clone();
    let native = staged.board.item(via_key);
    if !native.is_via() || native.net_numbers() != [move_.net_id] {
        return Err("via-nudge item identity/net mismatch".into());
    }
    let from = staged.point(move_.from)?;
    let to = staged.point(move_.to)?;
    let delta = Vector::get_instance(to.to_float().x as i32 - from.to_float().x as i32,
        to.to_float().y as i32 - from.to_float().y as i32);
    staged.board.translate_item(via_key, &delta);
    let net = move_.net_id;
    let cl = staged.board.rules.net_class_of(net).get_trace_clearance_class();
    for connector in &move_.connectors {
        let layer = staged.board.layer_structure.get_no(&connector.layer);
        if layer < 0 { return Err(format!("unknown via connector layer {}", connector.layer)); }
        let half = (connector.width * 0.5 * staged.units_per_mm).round() as i32;
        if half <= 0 { return Err("via connector width below native resolution".into()); }
        let polyline = Polyline::from_two_points(&from, &to);
        staged.board.insert_trace_without_cleaning(polyline, layer, half, &[net], cl,
            fr_engine::ids::FixedState::UserFixed)
            .ok_or("native via-nudge connector insertion failed")?;
    }
    *board = staged;
    Ok(())
}

/// Commit a translation against real native component members. `translate_item` removes and
/// re-adds every moved item's spatial index entry, and includes pins and component outlines.
pub fn apply_component_translation(board: &mut BoardAdapter, reference: &str, target: Point) -> Result<(), String> {
    let component_no = board.board.components.get_by_name(reference)
        .ok_or_else(|| format!("unknown native component {reference}"))?.id;
    let component = board.board.components.get(component_no);
    if component.position_fixed { return Err(format!("component {reference} is locked")); }
    let location = component.get_location().ok_or_else(|| format!("component {reference} is unplaced"))?;
    let from_native = location.to_float();
    let from = [from_native.x / board.units_per_mm, from_native.y / board.units_per_mm];
    let dx = target[0] - from[0]; let dy = target[1] - from[1];
    if !dx.is_finite() || !dy.is_finite() { return Err("non-finite component translation".into()); }
    let x = (dx * board.units_per_mm).round(); let y = (dy * board.units_per_mm).round();
    if x.abs() > i32::MAX as f64 || y.abs() > i32::MAX as f64 { return Err("component translation exceeds native range".into()); }
    let vector = Vector::get_instance(x as i32, y as i32);
    let members: Vec<_> = board.board.get_items().into_iter()
        .filter(|key| board.board.item(*key).component_no() == component_no).collect();
    for key in members { board.board.translate_item(key, &vector); }
    board.board.components_mut().move_component(component_no, &vector);
    Ok(())
}
#[derive(Clone, Debug, Default)]
pub struct Shortfalls {
    pub tracks: BTreeMap<i32, f64>,
    pub pads: BTreeMap<i32, f64>,
    pub vias: BTreeMap<i32, f64>,
}
impl Shortfalls {
    pub fn graze(&self) -> f64 {
        self.tracks
            .values()
            .chain(self.pads.values())
            .chain(self.vias.values())
            .sum()
    }
    pub fn worsens(&self, seed: &Self) -> bool {
        [
            (&self.tracks, &seed.tracks),
            (&self.pads, &seed.pads),
            (&self.vias, &seed.vias),
        ]
        .into_iter()
        .any(|(a, b)| {
            a.iter()
                .any(|(n, v)| *v > b.get(n).copied().unwrap_or(0.) + EPS)
        })
    }
}
