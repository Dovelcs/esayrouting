//! AWX whole_ctx.py caad45cd: board context, islands and installed per-layer reservations.
use super::corridor::{distance, mul, sub, Point};
use super::whole_frame::PerimeterCut;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RoutingRules {
    pub grid_step: f64,
    pub track_width: f64,
    pub clearance: f64,
    pub via_size: f64,
    pub via_drill: f64,
    pub pair_gap: f64,
    pub lane_pitch: f64,
    pub lane_min: f64,
    pub min_turning_radius: f64,
    pub max_turn_angle: f64,
    #[serde(default = "default_setback_angle")]
    pub max_setback_angle: f64,
    pub routing_clearance_margin: f64,
    pub board_edge_clearance: f64,
    pub hole_to_hole_clearance: f64,
    pub fan_clearance: f64,
    #[serde(default)]
    pub geometry_pair_room: bool,
    #[serde(default)]
    pub geometry_pair_turn_room_steps: Option<f64>,
    #[serde(default = "default_same_net_pad_clearance")]
    pub same_net_pad_clearance: f64,
    #[serde(default)]
    pub net_via_sizes: BTreeMap<i32, [f64; 2]>,
}
impl RoutingRules {
    /// The nominal via envelope used by the whole-route lane congestion model.
    pub fn via_need(&self) -> f64 {
        self.via_size / 2.0 + self.clearance + self.track_width / 2.0 + 0.03
    }
}
impl RoutingRules {
    pub fn validate(&self) -> Result<(), String> {
        for v in [
            self.grid_step,
            self.track_width,
            self.clearance,
            self.via_size,
            self.via_drill,
            self.lane_pitch,
            self.lane_min,
            self.routing_clearance_margin,
        ] {
            if !v.is_finite() || v <= 0. {
                return Err("AWX routing rules must be finite and positive".into());
            }
        }
        for (name, value) in [
            ("pair_gap", self.pair_gap),
            ("min_turning_radius", self.min_turning_radius),
            ("board_edge_clearance", self.board_edge_clearance),
            ("hole_to_hole_clearance", self.hole_to_hole_clearance),
            ("fan_clearance", self.fan_clearance),
        ] {
            if !value.is_finite() || value < 0. {
                return Err(format!("AWX {name} must be finite and nonnegative"));
            }
        }
        if !self.same_net_pad_clearance.is_finite() {
            return Err("invalid same-net pad clearance".into());
        }
        for value in [self.max_turn_angle, self.max_setback_angle] {
            if !value.is_finite() || value <= 0. || value > 180. {
                return Err("AWX angles must be finite in (0,180]".into());
            }
        }
        for (&net, &[diameter, drill]) in &self.net_via_sizes {
            if net <= 0
                || !diameter.is_finite()
                || !drill.is_finite()
                || drill <= 0.
                || diameter <= drill
            {
                return Err("invalid authorized net via dimensions".into());
            }
        }
        if self
            .geometry_pair_turn_room_steps
            .is_some_and(|v| !v.is_finite() || v < 0.)
        {
            return Err("invalid pair turn-room steps".into());
        }
        if self.via_drill >= self.via_size {
            return Err("AWX via drill must be below copper diameter".into());
        }
        Ok(())
    }
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ComponentKind {
    Bga,
    Connector,
    Passive,
    Other,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ContextPad {
    pub key: String,
    pub component_ref: String,
    pub pad_number: String,
    pub net: String,
    pub net_id: i32,
    pub point: Point,
    pub size: Point,
    pub layers: Vec<String>,
    pub drill: f64,
    pub drill_axis: Option<[Point; 2]>,
    #[serde(default)]
    pub non_plated_hole: bool,
    pub corner_radius: f64,
    pub pad_type: String,
    #[serde(default)]
    pub shape: String,
    pub rect_rotation: f64,
    pub copper_shapes: Vec<PadCopperShape>,
    pub component_kind: ComponentKind,
    pub movable: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CopperSegment {
    pub net: String,
    pub net_id: i32,
    pub a: Point,
    pub b: Point,
    pub width: f64,
    pub layer: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CopperVia {
    pub net: String,
    pub net_id: i32,
    pub point: Point,
    pub diameter: f64,
    pub drill: f64,
    pub layers: Vec<String>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Ends {
    pub tooth: Point,
    pub berth: Point,
    pub destination: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WholeContext {
    pub cfg: RoutingRules,
    pub layers: Vec<String>,
    pub copper_layers: Vec<String>,
    pub board_bounds: [f64; 4],
    pub board_outline: Vec<Vec<Point>>,
    pub pads: Vec<ContextPad>,
    pub source: String,
    pub destination: String,
    pub lanes: Vec<String>,
    pub byname: BTreeMap<String, i32>,
    pub pairs: BTreeMap<String, [String; 2]>,
    pub ends: BTreeMap<String, Ends>,
    pub src_ref: BTreeMap<String, String>,
    pub tooth_layer: BTreeMap<String, String>,
    pub dest_layer: BTreeMap<String, String>,
    pub tooth_dir: BTreeMap<String, Point>,
    pub stub_dir: BTreeMap<String, Point>,
    pub pair_ends: BTreeMap<String, [[Point; 2]; 2]>,
    pub pair_layers: BTreeMap<String, [String; 2]>,
    pub paths: BTreeMap<String, Vec<Point>>,
    pub base_segments: Vec<CopperSegment>,
    pub base_vias: Vec<CopperVia>,
    pub dest_cut: Option<PerimeterCut>,
    pub plane_nets: BTreeSet<String>,
    pub native_to_logical_components: BTreeMap<String, String>,
    pub chirality_mirrored: bool,
}
impl WholeContext {
    pub fn validate(&self) -> Result<(), String> {
        self.cfg.validate()?;
        if self.layers.len() < 2 {
            return Err("AWX whole-route requires at least two routing layers".into());
        }
        if self.source == self.destination {
            return Err("AWX source and destination must be distinct logical components".into());
        }
        if self.lanes.is_empty() {
            return Err("AWX context has no admitted lanes".into());
        }
        for lane in &self.lanes {
            if !self.ends.contains_key(lane)
                || !self.tooth_layer.contains_key(lane)
                || !self.dest_layer.contains_key(lane)
                || !self.paths.contains_key(lane)
            {
                return Err(format!("AWX missing terminal/path/layer for {lane}"));
            }
        }
        for ref_ in [&self.source, &self.destination] {
            if !self.pads.iter().any(|p| &p.component_ref == ref_) {
                return Err(format!("AWX logical component {ref_} has no mapped pads"));
            }
        }
        Ok(())
    }
    pub fn pads_of(&self, part: &str) -> Vec<&ContextPad> {
        self.pads
            .iter()
            .filter(|p| p.component_ref == part)
            .collect()
    }
    pub fn box_of(&self, part: &str) -> Result<[f64; 4], String> {
        let ps = self.pads_of(part);
        if ps.is_empty() {
            return Err(format!("no pad box for {part}"));
        }
        Ok([
            ps.iter().map(|p| p.point[0]).fold(f64::INFINITY, f64::min),
            ps.iter().map(|p| p.point[1]).fold(f64::INFINITY, f64::min),
            ps.iter()
                .map(|p| p.point[0])
                .fold(f64::NEG_INFINITY, f64::max),
            ps.iter()
                .map(|p| p.point[1])
                .fold(f64::NEG_INFINITY, f64::max),
        ])
    }
    pub fn stub_direction(&self, net: &str, pt: Point, layer: &str) -> Option<Point> {
        let nid = *self.byname.get(net)?;
        let tw = self.cfg.track_width;
        if self
            .base_vias
            .iter()
            .any(|v| v.net_id == nid && distance(v.point, pt) < tw / 2.)
        {
            return None;
        }
        let mut best: Option<(f64, Point)> = None;
        for s in &self.base_segments {
            if s.net_id != nid || s.layer != layer {
                continue;
            }
            for (a, b) in [(s.a, s.b), (s.b, s.a)] {
                let d = distance(b, pt);
                let l = distance(a, b);
                if l > 1e-6 && best.is_none_or(|x| d < x.0) {
                    best = Some((d, mul(sub(b, a), 1. / l)));
                }
            }
        }
        best.filter(|x| x.0 < tw / 2.).map(|x| x.1)
    }
    pub fn foreign_teeth(
        &self,
        members: &BTreeSet<String>,
        boxes: &[[f64; 4]],
    ) -> Vec<ForeignTooth> {
        let mut ids = BTreeSet::new();
        for n in members {
            if let Some(&id) = self.byname.get(n) {
                ids.insert(id);
            }
            if let Some(legs) = self.pairs.get(n) {
                for leg in legs {
                    if let Some(&id) = self.byname.get(leg) {
                        ids.insert(id);
                    }
                }
            }
        }
        let tol = self.cfg.track_width / 6.;
        let mut out = Vec::new();
        for seg in &self.base_segments {
            if ids.contains(&seg.net_id) {
                continue;
            }
            for p in [seg.a, seg.b] {
                if boxes.iter().any(|b| {
                    (((p[0] - b[0]).abs() < tol || (p[0] - b[2]).abs() < tol)
                        && b[1] - tol <= p[1]
                        && p[1] <= b[3] + tol)
                        || (((p[1] - b[1]).abs() < tol || (p[1] - b[3]).abs() < tol)
                            && b[0] - tol <= p[0]
                            && p[0] <= b[2] + tol)
                }) {
                    out.push(ForeignTooth {
                        point: p,
                        half_width: seg.width / 2.,
                        layer: seg.layer.clone(),
                        net: seg.net.clone(),
                    });
                }
            }
        }
        out
    }
    pub fn part_islands(
        &self,
        skip: &BTreeSet<String>,
        split: &BTreeSet<String>,
    ) -> BTreeMap<String, String> {
        let need = self.cfg.track_width
            + 2. * self.cfg.clearance
            + 2. * super::pairs::corner_buffer(self.cfg.grid_step)
            + self.cfg.grid_step;
        let mut boxes = Vec::new();
        for p in &self.pads {
            if skip.contains(&p.component_ref) {
                continue;
            }
            let layers: BTreeSet<String> = if p.drill > 0. {
                self.layers.iter().cloned().collect()
            } else {
                p.layers
                    .iter()
                    .filter(|l| self.layers.contains(l))
                    .cloned()
                    .collect()
            };
            if !layers.is_empty() {
                boxes.push((
                    p.key.clone(),
                    p.component_ref.clone(),
                    p.pad_number.clone(),
                    if p.non_plated_hole {
                        let radius = p.drill / 2.;
                        [
                            p.point[0] - radius,
                            p.point[1] - radius,
                            p.point[0] + radius,
                            p.point[1] + radius,
                        ]
                    } else {
                        [
                            p.point[0] - p.size[0] / 2.,
                            p.point[1] - p.size[1] / 2.,
                            p.point[0] + p.size[0] / 2.,
                            p.point[1] + p.size[1] / 2.,
                        ]
                    },
                    layers,
                ));
            }
        }
        let mut parent: Vec<usize> = (0..boxes.len()).collect();
        fn root(p: &mut [usize], mut i: usize) -> usize {
            while p[i] != i {
                p[i] = p[p[i]];
                i = p[i];
            }
            i
        }
        let mut bypart: BTreeMap<String, Vec<usize>> = BTreeMap::new();
        for (i, b) in boxes.iter().enumerate() {
            bypart.entry(b.1.clone()).or_default().push(i);
        }
        for (part, is) in &bypart {
            if is.len() != 2 || !split.contains(part) {
                for &i in is {
                    let a = root(&mut parent, i);
                    let b = root(&mut parent, is[0]);
                    parent[a] = b;
                }
            }
        }
        let mut sorted: Vec<usize> = (0..boxes.len()).collect();
        sorted.sort_by(|&i, &j| boxes[i].3[0].total_cmp(&boxes[j].3[0]));
        for (ai, &i) in sorted.iter().enumerate() {
            for &j in &sorted[ai + 1..] {
                let a = &boxes[i];
                let b = &boxes[j];
                if b.3[0] - a.3[2] >= need {
                    break;
                }
                if a.4.is_disjoint(&b.4) {
                    continue;
                }
                let dx = 0_f64.max(b.3[0] - a.3[2]).max(a.3[0] - b.3[2]);
                let dy = 0_f64.max(b.3[1] - a.3[3]).max(a.3[1] - b.3[3]);
                if dx.hypot(dy) < need {
                    let a = root(&mut parent, i);
                    let b = root(&mut parent, j);
                    parent[a] = b;
                }
            }
        }
        let mut members: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for i in 0..boxes.len() {
            members.entry(root(&mut parent, i)).or_default().push(i);
        }
        let mut result = BTreeMap::new();
        for group in members.values() {
            let mut labels = Vec::new();
            for (part, is) in &bypart {
                let included: Vec<_> = group
                    .iter()
                    .filter(|i| boxes[**i].1 == *part)
                    .copied()
                    .collect();
                if included.is_empty() {
                    continue;
                }
                if included.len() == is.len() {
                    labels.push(part.clone());
                } else {
                    let mut nums: Vec<_> = included.iter().map(|&i| boxes[i].2.clone()).collect();
                    nums.sort();
                    labels.push(format!("{part}:{}", nums.join("/")));
                }
            }
            let label = labels.join("+");
            for &i in group {
                result.insert(boxes[i].0.clone(), label.clone());
            }
        }
        result
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ForeignTooth {
    pub point: Point,
    pub half_width: f64,
    pub layer: String,
    pub net: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LaneGeometry {
    pub xy: Vec<Point>,
    pub layers: Vec<String>,
    pub vias: Vec<Point>,
    pub uxy: Vec<[f64; 3]>,
    pub paired_legs: BTreeMap<String, Vec<LayerPiece>>,
    /// Physical barrel centers keyed by their copper net. Pair barrel
    /// positions are supplied by geometry generation because crossovers and
    /// ordinary dives have different placement rules.
    #[serde(default)]
    pub paired_vias: BTreeMap<String, Vec<Point>>,
    pub snapped: bool,
    pub ends: Vec<PairEnd>,
    pub cross: Option<super::pairs::Crossover>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LayerPiece {
    pub a: Point,
    pub b: Point,
    pub layer: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ReservationTube {
    pub net: String,
    pub a: Point,
    pub b: Point,
    pub layer: String,
    pub radius: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct InstalledPlan {
    pub lanes: BTreeMap<String, LaneGeometry>,
    pub virtual_tracks: Vec<ReservationTube>,
    pub virtual_vias: Vec<CopperVia>,
}
impl WholeContext {
    pub fn install(&self, lanes: BTreeMap<String, LaneGeometry>) -> Result<InstalledPlan, String> {
        let mut tracks = Vec::new();
        let mut vias = Vec::new();
        for (n, g) in &lanes {
            if g.layers.len() != g.xy.len().saturating_sub(1) {
                return Err(format!("AWX piece layer cardinality mismatch for {n}"));
            }
            let half = if self.pairs.contains_key(n) {
                (self.cfg.track_width + super::pairs::pitch(&self.cfg)) / 2.
            } else {
                self.cfg.track_width / 2.
            };
            if !g.paired_legs.is_empty() {
                for (net, pieces) in &g.paired_legs {
                    for p in pieces {
                        tracks.push(ReservationTube {
                            net: net.clone(),
                            a: p.a,
                            b: p.b,
                            layer: p.layer.clone(),
                            radius: self.cfg.track_width / 2. + self.cfg.clearance,
                        });
                    }
                }
            } else {
                for (w, layer) in g.xy.windows(2).zip(&g.layers) {
                    tracks.push(ReservationTube {
                        net: n.clone(),
                        a: w[0],
                        b: w[1],
                        layer: layer.clone(),
                        radius: half + self.cfg.clearance,
                    });
                }
            }
            if let Some(legs) = self.pairs.get(n) {
                if !g.vias.is_empty() && g.paired_vias.is_empty() {
                    return Err(format!(
                        "AWX paired lane {n} has via sites without physical P/N barrel geometry"
                    ));
                }
                for leg in legs {
                    let Some(points) = g.paired_vias.get(leg) else {
                        if g.vias.is_empty() {
                            continue;
                        }
                        return Err(format!(
                            "AWX paired lane {n} is missing barrel geometry for {leg}"
                        ));
                    };
                    let Some(&net_id) = self.byname.get(leg) else {
                        return Err(format!("AWX paired lane {n} has no net id for {leg}"));
                    };
                    for &p in points {
                        vias.push(CopperVia {
                            net: leg.clone(),
                            net_id,
                            point: p,
                            diameter: self.cfg.via_size,
                            drill: self.cfg.via_drill,
                            layers: self.layers.clone(),
                        });
                    }
                }
            } else {
                let Some(&net_id) = self.byname.get(n) else {
                    return Err(format!("AWX lane {n} has no net id"));
                };
                for &p in &g.vias {
                    vias.push(CopperVia {
                        net: n.clone(),
                        net_id,
                        point: p,
                        diameter: self.cfg.via_size,
                        drill: self.cfg.via_drill,
                        layers: self.layers.clone(),
                    });
                }
            }
        }
        Ok(InstalledPlan {
            lanes,
            virtual_tracks: tracks,
            virtual_vias: vias,
        })
    }
}
impl WholeContext {
    /// Exact lattice quarter-turn, including native board bounds and every physical pair endpoint.
    pub fn transformed(&self, frame: super::whole_frame::FlowFrame) -> Self {
        let mut out = self.clone();
        let vector = |p: Point| {
            let mut p = p;
            if frame.mirror_axis_y.is_some() {
                p[1] = -p[1];
            }
            for _ in 0..frame.quarter_turns.rem_euclid(4) {
                p = [-p[1], p[0]];
            }
            p
        };
        let b = self.board_bounds;
        let corners =
            [[b[0], b[1]], [b[0], b[3]], [b[2], b[1]], [b[2], b[3]]].map(|p| frame.to_frame(p));
        out.board_bounds = [
            corners.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min),
            corners.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min),
            corners
                .iter()
                .map(|p| p[0])
                .fold(f64::NEG_INFINITY, f64::max),
            corners
                .iter()
                .map(|p| p[1])
                .fold(f64::NEG_INFINITY, f64::max),
        ];
        for p in &mut out.pads {
            p.point = frame.to_frame(p.point);
            if frame.mirror_axis_y.is_some() {
                p.rect_rotation = -p.rect_rotation;
            }
            if frame.quarter_turns.rem_euclid(2) != 0 {
                p.size.swap(0, 1);
            }
            // Size is the physical axis-aligned envelope; only the diagonal
            // residual belongs in the rectangle rotation metadata.
            p.rect_rotation = p.rect_rotation.rem_euclid(90.);
            if let Some(axis) = &mut p.drill_axis {
                for q in axis {
                    *q = frame.to_frame(*q);
                }
            }
            for copper in &mut p.copper_shapes {
                match &mut copper.shape {
                    CopperShape::Circle { center, .. } => *center = frame.to_frame(*center),
                    CopperShape::Polygon { outline } => {
                        for p in outline {
                            *p = frame.to_frame(*p);
                        }
                    }
                    CopperShape::RoundedRectangle {
                        center, rotation, ..
                    } => {
                        *center = frame.to_frame(*center);
                        if frame.mirror_axis_y.is_some() {
                            *rotation = -*rotation;
                        }
                        *rotation += 90. * frame.quarter_turns as f64;
                    }
                }
            }
        }
        for polygon in &mut out.board_outline {
            for p in polygon {
                *p = frame.to_frame(*p);
            }
        }
        for e in out.ends.values_mut() {
            e.tooth = frame.to_frame(e.tooth);
            e.berth = frame.to_frame(e.berth);
        }
        for d in out.tooth_dir.values_mut().chain(out.stub_dir.values_mut()) {
            *d = vector(*d);
        }
        for ends in out.pair_ends.values_mut() {
            for tips in ends {
                for p in tips {
                    *p = frame.to_frame(*p);
                }
            }
        }
        for path in out.paths.values_mut() {
            for p in path {
                *p = frame.to_frame(*p);
            }
        }
        for s in &mut out.base_segments {
            s.a = frame.to_frame(s.a);
            s.b = frame.to_frame(s.b);
        }
        for v in &mut out.base_vias {
            v.point = frame.to_frame(v.point);
        }
        out
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PadCopperShape {
    pub layer: String,
    pub shape: CopperShape,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum CopperShape {
    Circle {
        center: Point,
        radius: f64,
    },
    Polygon {
        outline: Vec<Point>,
    },
    RoundedRectangle {
        center: Point,
        half_size: Point,
        rotation: f64,
        corner_radius: f64,
    },
}
impl CopperShape {
    pub fn signed_distance(&self, p: Point) -> f64 {
        match self {
            Self::Circle { center, radius } => super::corridor::distance(p, *center) - radius,
            Self::RoundedRectangle {
                center,
                half_size,
                rotation,
                corner_radius,
            } => {
                let angle = rotation.to_radians();
                let q = super::corridor::sub(p, *center);
                let d = [
                    angle.cos() * q[0] + angle.sin() * q[1],
                    -angle.sin() * q[0] + angle.cos() * q[1],
                ];
                super::pairs::pad_distance(d[0], d[1], half_size[0], half_size[1], *corner_radius)
            }
            Self::Polygon { outline } => {
                let distance = (0..outline.len())
                    .map(|i| {
                        super::corridor::point_segment_distance(
                            p,
                            outline[i],
                            outline[(i + 1) % outline.len()],
                        )
                    })
                    .fold(f64::INFINITY, f64::min);
                if super::joint_geometry::point_in_polygon(p, outline) {
                    -distance
                } else {
                    distance
                }
            }
        }
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PairEnd {
    pub layer: String,
    pub tips: [Point; 2],
    pub pose: Point,
    pub handover: [Point; 2],
    pub heading: Point,
    pub legs: [Vec<Point>; 2],
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RouterBand {
    pub pieces: Vec<LayerPiece>,
    pub via_sites: Vec<Point>,
    pub end_sites: Vec<Point>,
    pub handover_sites: Vec<Point>,
    pub width: f64,
    pub via_radius: f64,
    pub end_radius: f64,
    pub handover_radius: f64,
    pub open_layers: bool,
}
impl RouterBand {
    pub fn contains(&self, p: Point, layer: &str) -> bool {
        self.pieces.iter().any(|s| {
            (self.open_layers || s.layer == layer)
                && super::corridor::point_segment_distance(p, s.a, s.b) <= self.width + 1e-9
        }) || self
            .via_sites
            .iter()
            .any(|&v| super::corridor::distance(p, v) <= self.via_radius + 1e-9)
            || self
                .end_sites
                .iter()
                .any(|&v| super::corridor::distance(p, v) <= self.end_radius + 1e-9)
            || self
                .handover_sites
                .iter()
                .any(|&v| super::corridor::distance(p, v) <= self.handover_radius + 1e-9)
    }
}
impl WholeContext {
    pub fn band_of(
        &self,
        lane: &str,
        g: &LaneGeometry,
        slack: f64,
        open_layers: bool,
    ) -> Result<RouterBand, String> {
        if g.xy.len() < 2 || g.layers.len() != g.xy.len() - 1 {
            return Err("AWX router band missing planned line/layers".into());
        }
        let mut pieces: Vec<LayerPiece> =
            g.xy.windows(2)
                .zip(&g.layers)
                .map(|(w, l)| LayerPiece {
                    a: w[0],
                    b: w[1],
                    layer: l.clone(),
                })
                .collect();
        let mut ends = vec![g.xy[0], *g.xy.last().unwrap()];
        let mut handovers = Vec::new();
        if !g.ends.is_empty() {
            if pieces.len() < 2 {
                return Err("AWX pair band has end connectors but no body".into());
            }
            pieces = pieces[1..pieces.len() - 1].to_vec();
            ends.clear();
            for e in &g.ends {
                pieces.push(LayerPiece {
                    a: super::pairs::mid(e.handover[0], e.handover[1]),
                    b: e.pose,
                    layer: e.layer.clone(),
                });
                ends.push(e.pose);
                handovers.extend(e.handover);
            }
        }
        if let Some(cross) = &g.cross {
            ends.extend(cross.poses);
            for p in cross.entry.values().chain(cross.exit.values()) {
                handovers.push(*p);
            }
        }
        let grid = self.cfg.grid_step;
        let (width, vr, er, hr) = if g.snapped {
            (
                grid / 2. + slack,
                grid / 2. + slack,
                1.5 * grid + slack,
                grid + slack,
            )
        } else {
            (
                self.cfg.track_width
                    + self.cfg.clearance
                    + slack
                    + if self.pairs.contains_key(lane) {
                        super::pairs::pitch(&self.cfg) / 2.
                    } else {
                        0.
                    },
                self.cfg.via_size / 2. + self.cfg.clearance + self.cfg.track_width / 2. + slack,
                0.,
                0.,
            )
        };
        Ok(RouterBand {
            pieces,
            via_sites: g.vias.clone(),
            end_sites: if g.snapped { ends } else { Vec::new() },
            handover_sites: if g.snapped { handovers } else { Vec::new() },
            width,
            via_radius: vr,
            end_radius: er,
            handover_radius: hr,
            open_layers,
        })
    }
}

fn default_setback_angle() -> f64 {
    45.
}

fn default_same_net_pad_clearance() -> f64 {
    -1.
}
