//! Native DSN snapshot for AWX; logical package identity and missing drill metadata are explicit inputs.
use super::{
    corridor::{point_segment_distance, Point},
    joint_geometry::{JointGeometry, ZoneRegion},
    whole_context::{ComponentKind, ContextPad, CopperSegment, CopperVia},
};
use crate::board_adapter::BoardAdapter;
use fr_geom::{FloatPoint, Shape};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LogicalPad {
    pub native_ref: String,
    pub number: Option<String>,
    pub drill_mm: Option<f64>,
    pub drill_axis: Option<[Point; 2]>,
    #[serde(default)]
    pub non_plated_hole: bool,
    pub shape: Option<String>,
    pub rect_rotation: Option<f64>,
    /// Complete paste opening geometry for this pad; Some([]) means verified absent.
    pub paste_apertures: Option<Vec<super::whole_context::CopperShape>>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LogicalComponent {
    pub reference: String,
    pub kind: ComponentKind,
    pub pads: Vec<LogicalPad>,
    #[serde(default)]
    pub movable: bool,
}
/// Explicit drill/plating evidence from the logical-package mapping. A native
/// pad without a mapped LogicalPad remains Unknown; its layer span is not used
/// to infer whether it is plated.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PadDrillMetadata {
    Unknown,
    Surface,
    Hole { plated: bool, drill_mm: Option<f64> },
}
#[derive(Clone)]
struct PhysicalShape {
    pub key: String,
    pub component: String,
    pub nets: Vec<i32>,
    pub layer: String,
    pub shape: Shape,
    pub movable: bool,
    pub hole: bool,
    pub via_only: bool,
}
#[derive(Clone)]
pub struct BoardSnapshot {
    pub native: BoardAdapter,
    pub pads: Vec<ContextPad>,
    pub segments: Vec<CopperSegment>,
    pub vias: Vec<CopperVia>,
    pub layers: Vec<String>,
    pub board_bounds: [f64; 4],
    pub board_outline: Vec<Vec<Point>>,
    pub zones: Vec<ZoneRegion>,
    pub byname: BTreeMap<String, i32>,
    physical: Vec<PhysicalShape>,
    paste: BTreeMap<String, Option<Vec<super::whole_context::CopperShape>>>,
    pad_drills: BTreeMap<String, PadDrillMetadata>,
}

/// Paste metadata for one undrilled-in-snapshot native pad. `pad` is the
/// actual native pad pose captured from the current board; aperture shapes are
/// returned verbatim from the logical-pad metadata and are not rebased here.
#[derive(Clone, Copy, Debug)]
pub struct SameNetPastePad<'a> {
    pub pad: &'a ContextPad,
    /// `None` means the opening metadata is unknown; `Some(&[])` is a verified
    /// absence; a nonempty slice contains the captured all-layer XY openings.
    pub apertures: Option<&'a [super::whole_context::CopperShape]>,
}

impl BoardSnapshot {
    /// Apply explicit drill diameters to named DSN padstacks before routing or
    /// snapshot capture. Values are diameters in millimetres; unknown names and
    /// invalid dimensions are errors rather than inferred legacy drills.
    pub fn apply_via_drills(
        board: &mut BoardAdapter,
        via_drills: &BTreeMap<String, f64>,
    ) -> Result<(), String> {
        for (name, &drill_mm) in via_drills {
            if !drill_mm.is_finite() || drill_mm <= 0.0 {
                return Err(format!("invalid explicit via drill for {name}"));
            }
            let padstack_id = board
                .board
                .library
                .padstacks
                .get_by_name(name)
                .map(|padstack| padstack.id)
                .ok_or_else(|| format!("explicit via drill names unknown padstack {name}"))?;
            let explicit_radius = board
                .board
                .library
                .padstacks
                .get(padstack_id)
                .and_then(|padstack| padstack.get_explicit_drill_radius());
            if explicit_radius.is_some() {
                continue;
            }
            let radius = drill_mm * board.units_per_mm / 2.0;
            board
                .board
                .library_mut()
                .padstacks
                .get_mut(padstack_id)
                .ok_or_else(|| format!("explicit via drill padstack {name} disappeared"))?
                .set_explicit_drill_radius(Some(radius))?;
        }
        Ok(())
    }

    /// Fill missing drill metadata for registered via infos using the
    /// explicitly configured endpoint drill. Native explicit dimensions win.
    pub fn apply_configured_via_drill(
        board: &mut BoardAdapter,
        drill_mm: f64,
    ) -> Result<(), String> {
        if !drill_mm.is_finite() || drill_mm <= 0.0 {
            return Err("invalid configured via drill".into());
        }
        let names: BTreeSet<_> = board
            .board
            .rules
            .via_infos
            .iter()
            .filter_map(|id| {
                board
                    .board
                    .library
                    .padstacks
                    .get(board.board.rules.via_infos[id].get_padstack())
                    .map(|padstack| padstack.name.clone())
            })
            .collect();
        let drills = names
            .into_iter()
            .map(|name| (name, drill_mm))
            .collect();
        Self::apply_via_drills(board, &drills)
    }

    /// Read actual copper vias without inferring logical package membership.
    pub fn capture_vias(board: &BoardAdapter, via_drills: &BTreeMap<String,f64>) -> Result<Vec<CopperVia>,String> {
        let b=&board.board;
        let scale=board.units_per_mm;
        let mut vias = Vec::new();
        for key in b.get_vias() {
            let item = b.item(key);
            let padstack = item.padstack(b).ok_or("native via padstack missing")?;
            let drill=padstack.get_explicit_drill_radius().map(|r|2.0*r/scale).or_else(||via_drills.get(&padstack.name).copied()).ok_or_else(||format!("Unsupported: native DSN via {} has no physical drill metadata; declare via_drills",padstack.name))?;
            let p = item.center(b).to_float();
            let diameter = (item.first_layer(b)..=item.last_layer(b)).filter_map(|l|padstack.get_shape(l)).map(|s|s.max_width()/scale).fold(0.0_f64,f64::max);
            if !drill.is_finite() || drill<=0.0 || !diameter.is_finite() || diameter<=drill { return Err("invalid physical via metadata".into()); }
            let ls: Vec<_> = (item.first_layer(b)..=item.last_layer(b))
                .map(|l| b.layer_structure.layers[l as usize].name.clone())
                .collect();
            for &nid in item.net_numbers() {
                vias.push(CopperVia {
                    net: b
                        .rules
                        .nets
                        .get(nid)
                        .ok_or("native via net missing")?
                        .name
                        .clone(),
                    net_id: nid,
                    point: [p.x / scale, p.y / scale],
                    diameter,
                    drill,
                    layers: ls.clone(),
                });
            }
        }
        Ok(vias)
    }

    pub fn capture(
        board: &BoardAdapter,
        components: &[LogicalComponent],
        via_drills: &BTreeMap<String, f64>,
    ) -> Result<Self, String> {
        if components.is_empty() {
            return Err("Unsupported: AWX needs declared logical BGA/connector package pad membership; a flattened DSN component is not a physical package".into());
        }
        let native_pads = board.pads();
        let mut assigned = BTreeMap::<String, (&LogicalComponent, &LogicalPad)>::new();
        let mut refs = BTreeSet::new();
        for c in components {
            if c.reference.is_empty() || !refs.insert(&c.reference) || c.pads.is_empty() {
                return Err("invalid/duplicate AWX logical component".into());
            }
            for p in &c.pads {
                if assigned.insert(p.native_ref.clone(), (c, p)).is_some() {
                    return Err(format!("native pad {} mapped twice", p.native_ref));
                }
            }
        }
        let b = &board.board;
        let scale = board.units_per_mm;
        let layers: Vec<_> = b
            .layer_structure
            .layers
            .iter()
            .map(|l| l.name.clone())
            .collect();
        let bb = b.bounding_box();
        let board_bounds = [
            bb.ll.x as f64 / scale,
            bb.ll.y as f64 / scale,
            bb.ur.x as f64 / scale,
            bb.ur.y as f64 / scale,
        ];
        let board_outline = b
            .get_outline()
            .and_then(|key| b.item(key).as_board_outline())
            .map(|outline| {
                outline
                    .shapes()
                    .iter()
                    .map(|s| {
                        s.corner_approx_arr()
                            .iter()
                            .map(|p| [p.x / scale, p.y / scale])
                            .collect()
                    })
                    .collect()
            })
            .unwrap_or_else(Vec::new);
        let byname = (1..=b.rules.nets.max_net_number())
            .filter_map(|n| b.rules.nets.get(n).map(|v| (v.name.clone(), n)))
            .collect();
        let mut pads = Vec::new();
        let mut physical = Vec::new();
        let mut paste = BTreeMap::new();
        let mut pad_drills = BTreeMap::new();
        let mut seen = BTreeSet::new();
        for native in native_pads {
            let Some(key) = native.key else {
                return Err("native pad has no item key".into());
            };
            let item = b.item(key);
            let mapping = assigned.get(&native.reference).copied();
            if mapping.is_some() {
                seen.insert(native.reference.clone());
            }
            let (component, kind, movable, meta) = if let Some((c, p)) = mapping {
                (c.reference.clone(), c.kind.clone(), c.movable, Some(p))
            } else {
                (native.chip.clone(), ComponentKind::Other, false, None)
            };
            paste.insert(
                native.reference.clone(),
                meta.and_then(|p| p.paste_apertures.clone()),
            );
            let drill_metadata = match meta {
                None => PadDrillMetadata::Unknown,
                Some(p) if p.non_plated_hole => PadDrillMetadata::Hole {
                    plated: false,
                    drill_mm: p.drill_mm,
                },
                Some(p) => match p.drill_mm {
                    None => PadDrillMetadata::Unknown,
                    Some(drill) if drill > 0.0 => PadDrillMetadata::Hole {
                        plated: true,
                        drill_mm: Some(drill),
                    },
                    Some(_) => PadDrillMetadata::Surface,
                },
            };
            pad_drills.insert(native.reference.clone(), drill_metadata);
            let drill = if let Some(p) = meta {
                p.drill_mm.unwrap_or(0.)
            } else {
                0.
            };
            if !drill.is_finite() || drill < 0. {
                return Err("invalid explicit pad drill".into());
            }
            if native.layers.len() > 1 && meta.and_then(|p| p.drill_mm).is_none() {
                return Err(format!("Unsupported: multilayer native pad {} has no DSN drill diameter; declare exact drill_mm",native.reference));
            }
            let mut copper_shapes = Vec::new();
            let mut size = [0f64; 2];
            let mut circle = false;
            let mut axis_aligned_rect = true;
            let mut radius = 0.;
            for layer in item.first_layer(b)..=item.last_layer(b) {
                if let Some(shape) = item.drill_shape_on_layer(b, layer) {
                    let bbox = shape.bounding_box();
                    size[0] = size[0].max((bbox.ur.x - bbox.ll.x) as f64 / scale);
                    size[1] = size[1].max((bbox.ur.y - bbox.ll.y) as f64 / scale);
                    if let Shape::Circle(c) = &shape {
                        circle = true;
                        radius = c.radius as f64 / scale;
                    }
                    let corners = shape.corner_approx_arr();
                    axis_aligned_rect &= !matches!(&shape, Shape::Circle(_))
                        && corners.len() == 4
                        && corners.iter().all(|p| {
                            (p.x == bbox.ll.x as f64 || p.x == bbox.ur.x as f64)
                                && (p.y == bbox.ll.y as f64 || p.y == bbox.ur.y as f64)
                        });
                    let layer_name = b.layer_structure.layers[layer as usize].name.clone();
                    let exported = match &shape {
                        Shape::Circle(c) => super::whole_context::CopperShape::Circle {
                            center: [c.center.x as f64 / scale, c.center.y as f64 / scale],
                            radius: c.radius as f64 / scale,
                        },
                        _ => super::whole_context::CopperShape::Polygon {
                            outline: shape
                                .corner_approx_arr()
                                .iter()
                                .map(|p| [p.x / scale, p.y / scale])
                                .collect(),
                        },
                    };
                    copper_shapes.push(super::whole_context::PadCopperShape {
                        layer: layer_name.clone(),
                        shape: exported,
                    });
                    physical.push(PhysicalShape {
                        key: native.reference.clone(),
                        component: component.clone(),
                        nets: item.net_numbers().to_vec(),
                        layer: layer_name,
                        shape,
                        movable,
                        hole: false,
                        via_only: false,
                    });
                }
            }
            let rotation = if axis_aligned_rect {
                0.
            } else { meta.and_then(|p| p.rect_rotation).unwrap_or_else(|| {
                b.components
                    .get(item.component_no())
                    .get_rotation_in_degree()
            }).rem_euclid(90.) };
            let non_plated = meta.is_some_and(|p| p.non_plated_hole);
            if non_plated {
                physical.retain(|s| s.key != native.reference);
                copper_shapes.clear();
                if drill <= 0. {
                    return Err("non-plated hole needs positive explicit drill".into());
                }
                for layer in &layers {
                    let center = board
                        .point(native.point)?
                        .to_int_point()
                        .ok_or("non-integral native hole center")?;
                    physical.push(PhysicalShape {
                        key: native.reference.clone(),
                        component: component.clone(),
                        nets: vec![],
                        layer: layer.clone(),
                        shape: Shape::Circle(fr_geom::Circle::new(
                            center,
                            (drill / 2. * scale).ceil() as i32,
                        )),
                        movable: false,
                        hole: true,
                        via_only: false,
                    });
                }
            }
            pads.push(ContextPad {
                key: native.reference.clone(),
                component_ref: component,
                pad_number: meta.and_then(|p| p.number.clone()).unwrap_or(native.pad),
                net: native.net.clone(),
                net_id: item.net_numbers().first().copied().unwrap_or(0),
                point: native.point,
                size,
                layers: native.layers,
                drill,
                drill_axis: meta.and_then(|p| p.drill_axis),
                non_plated_hole: non_plated,
                corner_radius: radius,
                component_kind: kind,
                movable,
                pad_type: if non_plated {
                    "np_thru_hole"
                } else if drill > 0. {
                    "thru_hole"
                } else {
                    "smd"
                }
                .into(),
                shape: meta.and_then(|p| p.shape.clone()).unwrap_or_else(|| {
                    if circle {
                        "circle".into()
                    } else if axis_aligned_rect {
                        "rect".into()
                    } else {
                        "custom".into()
                    }
                }),
                rect_rotation: rotation,
                copper_shapes,
            });
            let _ = circle;
        }
        if seen.len() != assigned.len() {
            return Err("AWX mapping names a native pad that does not exist".into());
        }
        let mut segments = Vec::new();
        for key in b.get_traces() {
            let item = b.item(key);
            let corners = item.trace().polyline().corner_approx_arr();
            let layer = b.layer_structure.layers[item.first_layer(b) as usize]
                .name
                .clone();
            for &nid in item.net_numbers() {
                let name = b
                    .rules
                    .nets
                    .get(nid)
                    .ok_or("native trace net missing")?
                    .name
                    .clone();
                for w in corners.windows(2) {
                    segments.push(CopperSegment {
                        net: name.clone(),
                        net_id: nid,
                        a: [w[0].x / scale, w[0].y / scale],
                        b: [w[1].x / scale, w[1].y / scale],
                        width: 2. * item.trace().half_width() as f64 / scale,
                        layer: layer.clone(),
                    });
                }
            }
        }
        let vias = Self::capture_vias(board, via_drills)?;
        let mut zones = Vec::new();
        for key in b.get_conduction_areas() {
            let item = b.item(key);
            for i in 0..b.tree_shape_count(fr_engine::board::DEFAULT_TREE, key) {
                if let Some(shape) = b.tree_shape(fr_engine::board::DEFAULT_TREE, key, i) {
                    let layer = b.shape_layer(key, i);
                    let outline: Vec<Point> = shape
                        .corner_approx_arr()
                        .iter()
                        .map(|p| [p.x / scale, p.y / scale])
                        .collect();
                    for &nid in item.net_numbers() {
                        zones.push(ZoneRegion {
                            layer: b.layer_structure.layers[layer as usize].name.clone(),
                            net_id: nid,
                            priority: 0,
                            outline: outline.clone(),
                        });
                    }
                }
            }
        }
        for key in b.get_items() {
            let item = b.item(key);
            let area = if let Some(a) = item.as_obstacle_area() {
                Some(a.get_area(b))
            } else {
                item.as_conduction_area().map(|c| c.area().get_area(b))
            };
            if let Some(area) = area {
                let shapes = area
                    .split_to_convex()
                    .ok_or("Unsupported: native keepout/plane polygon cannot be decomposed")?;
                let layer = b.layer_structure.layers[item.first_layer(b) as usize]
                    .name
                    .clone();
                for shape in shapes {
                    physical.push(PhysicalShape {
                        key: format!("area:{:?}", item.id()),
                        component: String::new(),
                        nets: item.net_numbers().to_vec(),
                        layer: layer.clone(),
                        shape: Shape::Tile(shape),
                        movable: false,
                        hole: false,
                        via_only: item.is_via_obstacle_area(),
                    });
                }
            }
        }
        Ok(Self {
            native: board.clone(),
            pads,
            segments,
            vias,
            layers,
            board_bounds,
            board_outline,
            zones,
            byname,
            physical,
            paste,
            pad_drills,
        })
    }
    pub fn pad(&self, key: &str) -> Option<&ContextPad> {
        self.pads.iter().find(|p| p.key == key)
    }

    /// Return only explicit plating/drill evidence. Unknown package mappings
    /// remain Unknown; native layer count is not a plating signal.
    pub fn pad_drill_metadata(&self, pad_key: &str) -> PadDrillMetadata {
        self.pad_drills
            .get(pad_key)
            .copied()
            .unwrap_or(PadDrillMetadata::Unknown)
    }

    /// Read the paste openings for a native pad key without collapsing unknown
    /// metadata into a verified empty list. Shapes are preserved exactly as
    /// supplied to `capture`; callers with before/after snapshots can use each
    /// snapshot's pad pose when accounting for board movement.
    pub fn paste_apertures(
        &self,
        pad_key: &str,
    ) -> Option<&[super::whole_context::CopperShape]> {
        self.paste.get(pad_key).and_then(Option::as_deref)
    }

    /// Read every same-net pad with no positive drill in this snapshot, matching
    /// the existing same-net pad-via keepout traversal. Use
    /// `pad_drill_metadata` to distinguish a confirmed surface pad from an
    /// unmapped/unknown pad. The aperture is an all-layer XY keepout as in
    /// source #962; `pad.layers` retains the actual native copper-layer list.
    /// Current DSN snapshots do not carry separate graphic/windowpane metadata,
    /// so this accessor does not synthesize it.
    pub fn same_net_paste_pads(
        &self,
        net_id: i32,
    ) -> impl Iterator<Item = SameNetPastePad<'_>> {
        self.pads
            .iter()
            .filter(move |pad| pad.net_id == net_id && pad.drill <= 0.0)
            .map(|pad| SameNetPastePad {
                pad,
                apertures: self.paste_apertures(&pad.key),
            })
    }

    /// Read explicitly declared plated through-hole pads for one native net.
    /// `drill_mm`, the NPTH flag, and `pad_type` come from the logical package
    /// mapping; this does not infer plating from a multilayer span.
    pub fn plated_through_hole_pads_for_net(
        &self,
        net_id: i32,
    ) -> impl Iterator<Item = &ContextPad> {
        self.pads.iter().filter(move |pad| {
            pad.net_id == net_id
                && matches!(
                    self.pad_drill_metadata(&pad.key),
                    PadDrillMetadata::Hole {
                        plated: true,
                        drill_mm: Some(drill)
                    } if drill > 0.0
                )
        })
    }
}
impl JointGeometry for BoardSnapshot {
    fn segment_clear(&self, net_id: i32, layer: &str, a: Point, b: Point, margin: f64) -> bool {
        if !self.layers.iter().any(|l| l == layer) {
            return false;
        }
        if self.point_violation(net_id, layer, a, margin, false, None)
            || self.point_violation(net_id, layer, b, margin, false, None)
        {
            return false;
        }
        let scale = self.native.units_per_mm;
        let aa = [a[0] * scale, a[1] * scale];
        let bb = [b[0] * scale, b[1] * scale];
        for object in &self.physical {
            if object.hole
                || object.via_only
                || object.layer != layer
                || object.nets.contains(&net_id)
            {
                continue;
            }
            let distance = match &object.shape {
                Shape::Circle(c) => {
                    point_segment_distance([c.center.x as f64, c.center.y as f64], aa, bb)
                        - c.radius as f64
                }
                _ => {
                    let points: Vec<Point> = object
                        .shape
                        .corner_approx_arr()
                        .iter()
                        .map(|p| [p.x, p.y])
                        .collect();
                    if object.shape.contains_float(&FloatPoint::new(aa[0], aa[1]))
                        || object.shape.contains_float(&FloatPoint::new(bb[0], bb[1]))
                    {
                        0.
                    } else {
                        (0..points.len())
                            .map(|i| {
                                super::pairs::segment_distance(
                                    aa,
                                    bb,
                                    points[i],
                                    points[(i + 1) % points.len()],
                                )
                            })
                            .fold(f64::INFINITY, f64::min)
                    }
                }
            };
            if distance < margin * scale {
                return false;
            }
        }
        for pad in self.pads.iter().filter(|p| p.non_plated_hole) {
            let axis = pad.drill_axis.unwrap_or([pad.point, pad.point]);
            if super::pairs::segment_distance(a, b, axis[0], axis[1]) < pad.drill / 2. + margin {
                return false;
            }
        }
        if self.segments.iter().any(|s| {
            s.net_id != net_id
                && s.layer == layer
                && super::pairs::segment_distance(a, b, s.a, s.b) < margin + s.width / 2.
        }) {
            return false;
        }
        if self.vias.iter().any(|v| {
            v.net_id != net_id
                && v.layers.iter().any(|l| l == layer)
                && point_segment_distance(v.point, a, b) < margin + v.diameter / 2.
        }) {
            return false;
        }
        true
    }
    fn point_violation(
        &self,
        net_id: i32,
        layer: &str,
        p: Point,
        margin: f64,
        via_mode: bool,
        ignore_partner: Option<i32>,
    ) -> bool {
        let b = self.board_bounds;
        if p[0] - margin < b[0]
            || p[0] + margin > b[2]
            || p[1] - margin < b[1]
            || p[1] + margin > b[3]
        {
            return true;
        }
        let scale = self.native.units_per_mm;
        let q = FloatPoint::new(p[0] * scale, p[1] * scale);
        for s in &self.physical {
            if s.hole
                || s.layer != layer
                || (s.via_only && !via_mode)
                || s.nets.contains(&net_id)
                || ignore_partner.is_some_and(|n| s.nets.contains(&n))
                || (via_mode && s.movable && !s.hole)
            {
                continue;
            }
            if s.shape.contains_float(&q) || s.shape.border_distance(&q) < margin * scale {
                return true;
            }
        }
        for pad in self.pads.iter().filter(|pad| pad.non_plated_hole) {
            let axis = pad.drill_axis.unwrap_or([pad.point, pad.point]);
            if point_segment_distance(p, axis[0], axis[1]) < pad.drill / 2. + margin {
                return true;
            }
        }
        for s in &self.segments {
            if s.layer == layer
                && s.net_id != net_id
                && Some(s.net_id) != ignore_partner
                && point_segment_distance(p, s.a, s.b) < margin + s.width / 2.
            {
                return true;
            }
        }
        for v in &self.vias {
            if v.net_id != net_id
                && Some(v.net_id) != ignore_partner
                && v.layers.iter().any(|l| l == layer)
                && super::corridor::distance(p, v.point) < margin + v.diameter / 2.
            {
                return true;
            }
        }
        false
    }
    fn snap_cell_blocked(
        &self,
        net: i32,
        partner: Option<i32>,
        layer: Option<&str>,
        p: Point,
        extra: f64,
        rules: &super::whole_context::RoutingRules,
    ) -> Result<bool, String> {
        let own = |ids: &[i32]| ids.contains(&net) || partner.is_some_and(|n| ids.contains(&n));
        let copper_margin = if layer.is_some() {
            rules.track_width / 2. + rules.clearance + extra
        } else {
            rules.via_size / 2. + rules.clearance + extra
        };
        let edge = rules.board_edge_clearance
            + if layer.is_some() {
                rules.track_width / 2.
            } else {
                rules.via_size / 2.
            };
        let b = self.board_bounds;
        if p[0] - edge < b[0] || p[0] + edge > b[2] || p[1] - edge < b[1] || p[1] + edge > b[3] {
            return Ok(true);
        }
        if !self.board_outline.is_empty() {
            let inside = self
                .board_outline
                .iter()
                .filter(|poly| super::joint_geometry::point_in_polygon(p, poly))
                .count()
                % 2
                == 1;
            if !inside {
                return Ok(true);
            }
            if self.board_outline.iter().any(|poly| {
                (0..poly.len())
                    .any(|i| point_segment_distance(p, poly[i], poly[(i + 1) % poly.len()]) < edge)
            }) {
                return Ok(true);
            }
        }
        let q = FloatPoint::new(
            p[0] * self.native.units_per_mm,
            p[1] * self.native.units_per_mm,
        );
        for s in &self.physical {
            if s.hole
                || (s.via_only && layer.is_some())
                || self
                    .pads
                    .iter()
                    .any(|p| p.key == s.key && p.non_plated_hole)
                || own(&s.nets)
                || layer.is_some_and(|l| s.layer != l)
            {
                continue;
            }
            let corner = self
                .pads
                .iter()
                .find(|pad| pad.key == s.key)
                .map(|pad| {
                    let delta = super::corridor::sub(p, pad.point);
                    let a = pad.rect_rotation.to_radians();
                    let dx = a.cos() * delta[0] + a.sin() * delta[1];
                    let dy = -a.sin() * delta[0] + a.cos() * delta[1];
                    super::pairs::pad_corner_buffer(
                        dx,
                        dy,
                        pad.size[0] / 2.,
                        pad.size[1] / 2.,
                        pad.corner_radius,
                        rules.grid_step,
                    )
                })
                .unwrap_or(0.);
            if s.shape.contains_float(&q)
                || s.shape.border_distance(&q) < (copper_margin + corner) * self.native.units_per_mm
            {
                return Ok(true);
            }
        }
        for pad in &self.pads {
            if pad.drill <= 0. || pad.net_id == net || Some(pad.net_id) == partner {
                continue;
            }
            let bar = pad.drill / 2.
                + if layer.is_some() {
                    rules.track_width / 2. + rules.clearance + extra
                } else {
                    rules.via_drill / 2. + rules.hole_to_hole_clearance + extra
                };
            let distance = pad
                .drill_axis
                .map(|axis| point_segment_distance(p, axis[0], axis[1]))
                .unwrap_or_else(|| super::corridor::distance(p, pad.point));
            if distance < bar {
                return Ok(true);
            }
        }
        for s in &self.segments {
            if s.net_id == net || Some(s.net_id) == partner || layer.is_some_and(|l| s.layer != l) {
                continue;
            }
            if point_segment_distance(p, s.a, s.b) < s.width / 2. + copper_margin {
                return Ok(true);
            }
        }
        for v in &self.vias {
            if v.net_id == net
                || Some(v.net_id) == partner
                || layer.is_some_and(|l| !v.layers.iter().any(|vl| vl == l))
            {
                continue;
            }
            let distance = super::corridor::distance(p, v.point);
            if distance < v.diameter / 2. + copper_margin {
                return Ok(true);
            }
            if layer.is_none()
                && distance
                    < v.drill / 2. + rules.via_drill / 2. + rules.hole_to_hole_clearance + extra
            {
                return Ok(true);
            }
        }
        Ok(false)
    }
    fn same_net_pad_via_blocked(
        &self,
        net: i32,
        p: Point,
        rules: &super::whole_context::RoutingRules,
    ) -> Result<bool, String> {
        if rules.same_net_pad_clearance <= 0. {
            return Ok(false);
        }
        let margin = rules.via_size / 2. + rules.same_net_pad_clearance + rules.grid_step / 2.;
        for pad in self
            .pads
            .iter()
            .filter(|pad| pad.net_id == net && pad.drill <= 0.)
        {
            let apertures=self.paste.get(&pad.key).and_then(Option::as_ref).ok_or_else(||format!("Unsupported: active same_net_pad_clearance requires complete paste_apertures metadata for {}",pad.key))?;
            if pad
                .copper_shapes
                .iter()
                .any(|c| c.shape.signed_distance(p) <= margin)
                || apertures
                    .iter()
                    .any(|shape| shape.signed_distance(p) <= margin)
            {
                return Ok(true);
            }
        }
        Ok(false)
    }
    fn copper_layers(&self) -> &[String] {
        &self.layers
    }
}
/// The model is in AWX's canonical frame; native geometry stays on the original board.
pub struct CanonicalGeometry<'a> {
    pub snapshot: &'a BoardSnapshot,
    pub frame: super::whole_frame::FlowFrame,
}
impl JointGeometry for CanonicalGeometry<'_> {
    fn segment_clear(&self, net: i32, layer: &str, a: Point, b: Point, margin: f64) -> bool {
        let Ok(mapped) = self.frame.to_board_layer(layer, &self.snapshot.layers) else {
            return false;
        };
        self.snapshot.segment_clear(
            net,
            &mapped,
            self.frame.to_board(a),
            self.frame.to_board(b),
            margin,
        )
    }
    fn point_violation(
        &self,
        net: i32,
        layer: &str,
        p: Point,
        margin: f64,
        via_mode: bool,
        partner: Option<i32>,
    ) -> bool {
        let Ok(mapped) = self.frame.to_board_layer(layer, &self.snapshot.layers) else {
            return true;
        };
        self.snapshot.point_violation(
            net,
            &mapped,
            self.frame.to_board(p),
            margin,
            via_mode,
            partner,
        )
    }
    fn snap_cell_blocked(
        &self,
        net: i32,
        partner: Option<i32>,
        layer: Option<&str>,
        p: Point,
        extra: f64,
        rules: &super::whole_context::RoutingRules,
    ) -> Result<bool, String> {
        let mapped = layer
            .map(|l| self.frame.to_board_layer(l, &self.snapshot.layers))
            .transpose()?;
        self.snapshot.snap_cell_blocked(
            net,
            partner,
            mapped.as_deref(),
            self.frame.to_board(p),
            extra,
            rules,
        )
    }
    fn same_net_pad_via_blocked(
        &self,
        net: i32,
        p: Point,
        rules: &super::whole_context::RoutingRules,
    ) -> Result<bool, String> {
        self.snapshot
            .same_net_pad_via_blocked(net, self.frame.to_board(p), rules)
    }
    fn copper_layers(&self) -> &[String] {
        &self.snapshot.layers
    }
}
