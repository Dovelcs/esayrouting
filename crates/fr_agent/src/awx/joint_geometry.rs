//! Actual-sized joint-escape pieces, manufacturing clamps and static geometry (AWX caad45cd, MIT).
use super::escape_moves::{distance, Direction, Grid, Leg, Move, Pad, Point};
use super::fab_rules::{clamp_via_to_pad, FabFloor, ViaClamp};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
pub const SHARE_TOL: f64 = 1e-3;
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct ViaSize {
    pub radius: f64,
    pub drill_radius: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointSizes {
    pub track: f64,
    pub clearance: f64,
    pub hole_to_hole: f64,
    pub segment_spacing: f64,
    pub via: ViaSize,
    pub inpad: BTreeMap<String, ViaSize>,
    pub clamps: BTreeMap<String, ViaClamp>,
    pub drop_vias: Vec<ViaSize>,
    pub grow: f64,
    pub stack: f64,
    pub home_radius: f64,
    pub lane_bar: f64,
    pub pair_pitch: f64,
    pub pair_exit_reach: f64,
}
impl JointSizes {
    pub fn measure(
        pads: &[Pad],
        track: f64,
        clearance: f64,
        hole_to_hole: f64,
        via_diameter: f64,
        via_drill: f64,
        floors: &[FabFloor],
        route_track: f64,
        route_clearance: f64,
        grid: f64,
        pair_pitch: f64,
    ) -> Self {
        let mut inpad = BTreeMap::new();
        let mut clamps = BTreeMap::new();
        for pad in pads {
            let clamp = clamp_via_to_pad(via_diameter, via_drill, pad, floors, None);
            inpad.insert(
                pad.key.clone(),
                ViaSize {
                    radius: clamp.diameter / 2.0,
                    drill_radius: clamp.drill / 2.0,
                },
            );
            clamps.insert(pad.key.clone(), clamp);
        }
        let pad_radius = pads
            .iter()
            .map(|p| p.size[0].max(p.size[1]))
            .reduce(f64::max)
            .unwrap_or(0.4)
            / 2.0;
        let mut drop_vias: Vec<_> = floors
            .iter()
            .filter(|f| f.via_diameter < via_diameter - 1e-9)
            .map(|f| ViaSize {
                radius: f.via_diameter / 2.0,
                drill_radius: f.via_drill / 2.0,
            })
            .collect();
        drop_vias.sort_by(|a, b| {
            b.radius
                .total_cmp(&a.radius)
                .then(b.drill_radius.total_cmp(&a.drill_radius))
        });
        drop_vias.dedup_by(|a, b| a.radius == b.radius && a.drill_radius == b.drill_radius);
        Self {
            track,
            clearance,
            hole_to_hole: hole_to_hole.max(0.20),
            segment_spacing: track + clearance,
            via: ViaSize {
                radius: via_diameter / 2.0,
                drill_radius: via_drill / 2.0,
            },
            inpad,
            clamps,
            drop_vias,
            grow: via_diameter + clearance,
            stack: track + clearance + 0.005,
            home_radius: (pad_radius + track / 2.0 + clearance)
                .max(via_diameter / 2.0 + track / 2.0 + clearance),
            lane_bar: route_track / 2.0 + route_clearance + grid / 2.0,
            pair_pitch,
            pair_exit_reach: 1.2,
        }
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PieceVia {
    pub point: Point,
    pub radius: f64,
    pub drill_radius: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Piece {
    pub net: String,
    pub legs: Vec<Leg>,
    pub vias: Vec<PieceVia>,
    pub balls: Vec<Point>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Strap {
    pub to: String,
    pub a: Point,
    pub b: Point,
    pub layer: String,
    pub length: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Drop {
    pub site: Point,
    pub stub: Option<[Point; 2]>,
    pub layer: String,
    pub inpad: bool,
    pub radius: f64,
    pub drill_radius: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ZoneRegion {
    pub layer: String,
    pub net_id: i32,
    pub priority: i32,
    pub outline: Vec<Point>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Pocket {
    pub layer: String,
    pub polygon: Vec<Point>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NetLeg {
    pub net_id: i32,
    pub net: String,
    pub leg: Leg,
}
/// Snapshot implementations preserve AWX's two static obstacle modes: tracks clear
/// every passive, while via-mode can exclude the explicitly movable passives.
pub trait JointGeometry {
    fn segment_clear(&self, net_id: i32, layer: &str, a: Point, b: Point, margin: f64) -> bool;
    fn point_violation(
        &self,
        net_id: i32,
        layer: &str,
        p: Point,
        margin: f64,
        via_mode: bool,
        ignore_partner: Option<i32>,
    ) -> bool;
    fn snap_cell_blocked(
        &self,
        _net: i32,
        _partner: Option<i32>,
        _layer: Option<&str>,
        _point: Point,
        _extra: f64,
        _rules: &super::whole_context::RoutingRules,
    ) -> Result<bool, String> {
        Err("Unsupported: snapshot does not provide source-accurate snap copper/drill masks".into())
    }
    fn same_net_pad_via_blocked(
        &self,
        _net: i32,
        _point: Point,
        rules: &super::whole_context::RoutingRules,
    ) -> Result<bool, String> {
        if rules.same_net_pad_clearance > 0. {
            Err("Unsupported: active same-net pad-via rule requires physical paste aperture data absent from this native snapshot".into())
        } else {
            Ok(false)
        }
    }
    fn copper_layers(&self) -> &[String];
}
pub fn point_segment(p: Point, a: Point, b: Point) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let length = d[0] * d[0] + d[1] * d[1];
    let t = if length == 0.0 {
        0.0
    } else {
        ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / length
    }
    .clamp(0.0, 1.0);
    distance(p, [a[0] + t * d[0], a[1] + t * d[1]])
}
pub fn segment_segment(p: Point, q: Point, a: Point, b: Point) -> f64 {
    let cross = |o: Point, a: Point, b: Point| {
        (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    };
    let (d1, d2, d3, d4) = (
        cross(a, b, p),
        cross(a, b, q),
        cross(p, q, a),
        cross(p, q, b),
    );
    if (d1 > 0.0) != (d2 > 0.0)
        && (d3 > 0.0) != (d4 > 0.0)
        && d1 != 0.0
        && d2 != 0.0
        && d3 != 0.0
        && d4 != 0.0
    {
        return 0.0;
    }
    point_segment(p, a, b)
        .min(point_segment(q, a, b))
        .min(point_segment(a, p, q))
        .min(point_segment(b, p, q))
}
pub fn piece_hit(p: &Piece, q: &Piece, same: bool, sz: &JointSizes) -> bool {
    let half = sz.track / 2.0;
    let mut shared = Vec::new();
    for v in &p.vias {
        for w in &q.vias {
            let d = distance(v.point, w.point);
            if same
                && d < SHARE_TOL
                && (v.radius - w.radius).abs() < 1e-9
                && (v.drill_radius - w.drill_radius).abs() < 1e-9
            {
                shared.push((v.point, v.radius + half + sz.clearance));
                continue;
            }
            if d < v.drill_radius + w.drill_radius + sz.hole_to_hole - 1e-6
                || (!same && d < v.radius + w.radius + sz.clearance - 1e-6)
            {
                return true;
            }
        }
    }
    if !same {
        for a in &p.legs {
            for b in &q.legs {
                if a.layer == b.layer
                    && segment_segment(a.a, a.b, b.a, b.b) < sz.segment_spacing - 1e-6
                {
                    return true;
                }
            }
        }
        return p.vias.iter().any(|v| {
            q.legs
                .iter()
                .any(|l| point_segment(v.point, l.a, l.b) < v.radius + half + sz.clearance - 1e-6)
        }) || q.vias.iter().any(|v| {
            p.legs
                .iter()
                .any(|l| point_segment(v.point, l.a, l.b) < v.radius + half + sz.clearance - 1e-6)
        });
    }
    for (x, y) in [(p, q), (q, p)] {
        for l in &x.legs {
            let n = ((l.length() / 0.02).trunc() as usize).max(2);
            for i in 0..=n {
                let t = [
                    l.a[0] + (l.b[0] - l.a[0]) * i as f64 / n as f64,
                    l.a[1] + (l.b[1] - l.a[1]) * i as f64 / n as f64,
                ];
                if x.balls.iter().any(|&b| distance(t, b) < sz.home_radius)
                    || shared.iter().any(|&(b, r)| distance(t, b) < r)
                {
                    continue;
                }
                if y.legs.iter().any(|o| {
                    l.layer == o.layer && point_segment(t, o.a, o.b) < sz.segment_spacing - 1e-6
                }) || y
                    .vias
                    .iter()
                    .any(|v| distance(t, v.point) < v.radius + half + sz.clearance - 1e-6)
                {
                    return true;
                }
            }
        }
    }
    false
}
pub fn piece_box(p: &Piece, grow: f64) -> [f64; 4] {
    let points: Vec<_> = p
        .legs
        .iter()
        .flat_map(|l| [l.a, l.b])
        .chain(p.vias.iter().map(|v| v.point))
        .chain(p.balls.iter().copied())
        .collect();
    [
        points.iter().map(|p| p[0]).reduce(f64::min).unwrap() - grow,
        points.iter().map(|p| p[1]).reduce(f64::min).unwrap() - grow,
        points.iter().map(|p| p[0]).reduce(f64::max).unwrap() + grow,
        points.iter().map(|p| p[1]).reduce(f64::max).unwrap() + grow,
    ]
}
pub fn via_clear<G: JointGeometry + ?Sized>(
    geometry: &G,
    net: i32,
    p: Point,
    radius: f64,
    sz: &JointSizes,
) -> bool {
    geometry
        .copper_layers()
        .iter()
        .all(|layer| !geometry.point_violation(net, layer, p, sz.clearance + radius, true, None))
}
pub fn point_in_polygon(p: Point, polygon: &[Point]) -> bool {
    if polygon.len() < 3 {
        return false;
    }
    let mut inside = false;
    let mut previous = polygon.len() - 1;
    for current in 0..polygon.len() {
        let a = polygon[current];
        let b = polygon[previous];
        if (a[1] > p[1]) != (b[1] > p[1])
            && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]
        {
            inside = !inside;
        }
        previous = current;
    }
    inside
}
pub fn on_own_plane(zones: &[ZoneRegion], net: i32, q: Point) -> bool {
    zones.iter().any(|z| {
        z.net_id == net
            && point_in_polygon(q, &z.outline)
            && !zones.iter().any(|other| {
                other.layer == z.layer
                    && other.net_id != net
                    && other.priority > z.priority
                    && point_in_polygon(q, &other.outline)
            })
    })
}
pub fn signal_layers(
    layers: &[String],
    zones: &[ZoneRegion],
    outer: &BTreeSet<String>,
) -> Vec<String> {
    let planes: BTreeSet<_> = zones
        .iter()
        .filter(|z| z.net_id != 0 && !outer.contains(&z.layer))
        .map(|z| &z.layer)
        .collect();
    layers
        .iter()
        .filter(|layer| !planes.contains(layer))
        .cloned()
        .collect()
}
pub fn plane_layers(
    zones: &[ZoneRegion],
    outer: &BTreeSet<String>,
    routing_layers: &[String],
) -> BTreeSet<String> {
    zones
        .iter()
        .filter(|z| {
            z.net_id != 0 && !outer.contains(&z.layer) && !routing_layers.contains(&z.layer)
        })
        .map(|z| z.layer.clone())
        .collect()
}
pub fn far_face(pads: &[Pad], other: &[Pad]) -> Option<Direction> {
    if pads.is_empty() || other.is_empty() {
        return None;
    }
    let centre = |pads: &[Pad]| {
        [
            pads.iter().map(|p| p.point[0]).sum::<f64>() / pads.len() as f64,
            pads.iter().map(|p| p.point[1]).sum::<f64>() / pads.len() as f64,
        ]
    };
    let a = centre(pads);
    let b = centre(other);
    Direction::ALL.into_iter().min_by(|x, y| {
        let (dx, dy) = x.delta();
        let (ex, ey) = y.delta();
        (dx as f64 * (b[0] - a[0]) + dy as f64 * (b[1] - a[1]))
            .total_cmp(&(ex as f64 * (b[0] - a[0]) + ey as f64 * (b[1] - a[1])))
    })
}
pub fn lane_rays(
    grid: &Grid,
    pads: &[Pad],
    segments: &[NetLeg],
    nets: &BTreeSet<String>,
    reach: f64,
) -> Vec<[Point; 2]> {
    let ids: BTreeSet<_> = pads
        .iter()
        .filter(|p| p.net_id != 0 && nets.contains(super::escape_moves::short_name(&p.net)))
        .map(|p| p.net_id)
        .collect();
    let [x0, y0, x1, y1] = grid.bbox;
    let mut ends: BTreeMap<i32, (f64, Point, (i32, i32))> = BTreeMap::new();
    for s in segments {
        if !ids.contains(&s.net_id) {
            continue;
        }
        for q in [s.leg.a, s.leg.b] {
            if !(x0 - 1.5 <= q[0] && q[0] <= x1 + 1.5 && y0 - 1.5 <= q[1] && q[1] <= y1 + 1.5) {
                continue;
            }
            let (d, u) = [
                (q[0] - x1, (1, 0)),
                (x0 - q[0], (-1, 0)),
                (q[1] - y1, (0, 1)),
                (y0 - q[1], (0, -1)),
            ]
            .into_iter()
            .max_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)))
            .unwrap();
            if d > 0.0 && ends.get(&s.net_id).is_none_or(|old| d > old.0) {
                ends.insert(s.net_id, (d, q, u));
            }
        }
    }
    ends.values()
        .map(|&(_, q, (dx, dy))| [q, [q[0] + dx as f64 * reach, q[1] + dy as f64 * reach]])
        .collect()
}
pub fn lane_faces(rays: &[[Point; 2]]) -> BTreeSet<(i32, i32)> {
    rays.iter()
        .map(|r| {
            let sign = |x: f64| {
                if x > 1e-9 {
                    1
                } else if x < -1e-9 {
                    -1
                } else {
                    0
                }
            };
            (sign(r[1][0] - r[0][0]), sign(r[1][1] - r[0][1]))
        })
        .collect()
}
pub fn beyond_faces(grid: &Grid, q: Point) -> BTreeSet<(i32, i32)> {
    let [x0, y0, x1, y1] = grid.bbox;
    let (gx, gy) = (grid.pitch_x / 4.0, grid.pitch_y / 4.0);
    [
        ((1, 0), q[0] > x1 + gx),
        ((-1, 0), q[0] < x0 - gx),
        ((0, 1), q[1] > y1 + gy),
        ((0, -1), q[1] < y0 - gy),
    ]
    .into_iter()
    .filter_map(|(u, ok)| ok.then_some(u))
    .collect()
}
pub fn lanes_clear(site: Point, radius: f64, rays: &[[Point; 2]], bar: f64) -> bool {
    rays.iter()
        .all(|r| point_segment(site, r[0], r[1]) - radius >= bar - 1e-9)
}
pub fn in_pocket(quad: &[Point], a: Point, b: Option<Point>) -> bool {
    if quad.len() != 4 {
        return false;
    }
    let inside = |p: Point| {
        let signs: Vec<_> = (0..4)
            .map(|i| {
                let q = quad[i];
                let r = quad[(i + 1) % 4];
                (r[0] - q[0]) * (p[1] - q[1]) - (r[1] - q[1]) * (p[0] - q[0])
            })
            .collect();
        signs.iter().all(|&s| s >= 0.0) || signs.iter().all(|&s| s <= 0.0)
    };
    if inside(a) {
        return true;
    }
    let Some(b) = b else {
        return false;
    };
    if inside(b) {
        return true;
    }
    let cross = |p: Point, q: Point, r: Point| {
        (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    };
    (0..4).any(|i| {
        let c = quad[i];
        let d = quad[(i + 1) % 4];
        cross(a, b, c) * cross(a, b, d) < 0.0 && cross(c, d, a) * cross(c, d, b) < 0.0
    })
}
pub fn in_pockets(pockets: &[Pocket], legs: &[Leg], vias: &[Point]) -> bool {
    pockets.iter().any(|p| {
        legs.iter()
            .any(|l| l.layer == p.layer && in_pocket(&p.polygon, l.a, Some(l.b)))
            || vias.iter().any(|&v| in_pocket(&p.polygon, v, None))
    })
}
pub fn pair_exit_clear<G: JointGeometry + ?Sized>(
    geometry: &G,
    net: i32,
    partner: i32,
    m: &Move,
    layer: &str,
    sz: &JointSizes,
) -> bool {
    if sz.pair_exit_reach <= 0.0 {
        return true;
    }
    let (dx, dy) = m.direction.delta();
    let normal = [-dy, dx];
    let mut sides = [true, true];
    let mut k = 1;
    while k as f64 * 0.1 <= sz.pair_exit_reach + 1e-9 {
        let q = [
            m.exit_pt[0] + dx as f64 * 0.1 * k as f64,
            m.exit_pt[1] + dy as f64 * 0.1 * k as f64,
        ];
        let margin = sz.clearance + sz.track / 2.0;
        if geometry.point_violation(net, layer, q, margin, false, Some(partner)) {
            return false;
        }
        for (i, sign) in [1.0, -1.0].into_iter().enumerate() {
            if sides[i]
                && geometry.point_violation(
                    net,
                    layer,
                    [
                        q[0] + normal[0] as f64 * sz.pair_pitch * sign,
                        q[1] + normal[1] as f64 * sz.pair_pitch * sign,
                    ],
                    margin,
                    false,
                    Some(partner),
                )
            {
                sides[i] = false;
            }
        }
        if !sides[0] && !sides[1] {
            return false;
        }
        k += 1;
    }
    true
}
pub fn drops<G: JointGeometry + ?Sized>(
    geometry: &G,
    grid: &Grid,
    p: &Pad,
    pads: &[Pad],
    sizes: &JointSizes,
    rays: &[[Point; 2]],
    zones: Option<&[ZoneRegion]>,
) -> Vec<Drop> {
    let home = geometry
        .copper_layers()
        .iter()
        .find(|layer| p.layers.contains(layer))
        .map(String::as_str)
        .unwrap_or("F.Cu");
    let (hx, hy) = (grid.pitch_x / 2.0, grid.pitch_y / 2.0);
    let pad = p.point;
    let mut sites = Vec::new();
    for sx in [-1.0, 1.0] {
        for sy in [-1.0, 1.0] {
            sites.push([pad[0] + sx * hx, pad[1] + sy * hy]);
        }
    }
    if hx != 0.0 && hy != 0.0 {
        let reach = hx.hypot(hy);
        for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            let neighbour = [pad[0] + 2.0 * hx * dx as f64, pad[1] + 2.0 * hy * dy as f64];
            if !pads.iter().any(|q| {
                (q.point[0] - neighbour[0]).abs() < 0.3 * hx
                    && (q.point[1] - neighbour[1]).abs() < 0.3 * hy
            }) {
                sites.push([pad[0] + reach * dx as f64, pad[1] + reach * dy as f64]);
            }
        }
    }
    let mut out = Vec::new();
    for site in sites {
        if !geometry.segment_clear(
            p.net_id,
            home,
            pad,
            site,
            sizes.clearance + sizes.track / 2.0,
        ) || zones.is_some_and(|zones| !on_own_plane(zones, p.net_id, site))
        {
            continue;
        }
        if let Some(size) = std::iter::once(sizes.via)
            .chain(sizes.drop_vias.iter().copied())
            .find(|s| {
                via_clear(geometry, p.net_id, site, s.radius, sizes)
                    && lanes_clear(site, s.radius, rays, sizes.lane_bar)
            })
        {
            out.push(Drop {
                site,
                stub: Some([pad, site]),
                layer: home.into(),
                inpad: false,
                radius: size.radius,
                drill_radius: size.drill_radius,
            });
        }
    }
    let size = sizes.inpad[&p.key];
    if via_clear(geometry, p.net_id, pad, size.radius, sizes)
        && zones.is_none_or(|zones| on_own_plane(zones, p.net_id, pad))
    {
        out.push(Drop {
            site: pad,
            stub: None,
            layer: home.into(),
            inpad: true,
            radius: size.radius,
            drill_radius: size.drill_radius,
        });
    }
    out
}
