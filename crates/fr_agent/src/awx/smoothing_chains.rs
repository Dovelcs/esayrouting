//! Extraction of source-style smoothing chains across native track items.
//!
//! A polyline item's corners are only implementation details: smoothing chains
//! continue across item boundaries when the same-net, same-layer, same-width
//! copper has exactly two incident legs and no via, pad, or branch anchors it.

use fr_engine::{
    board::{ItemKey, RoutingBoard},
    ids::{ClearanceClassNo, NetNo},
};
use fr_geom::{FloatPoint, Point};
use std::collections::{BTreeMap, BTreeSet};

const COINCIDENCE_TOL_MM: f64 = 0.02;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SmoothingTouchKind {
    Via,
    Pad,
    SiblingTrack,
}

#[derive(Clone, Debug)]
pub struct SmoothingTouch {
    pub point: FloatPoint,
    /// Index in `SmoothingChain::legs`; this is a chain-global leg index.
    pub leg_index: usize,
    /// Copper-to-copper reach in native board units.
    pub reach: f64,
    pub kind: SmoothingTouchKind,
    /// Present for pads and vias, allowing callers to apply exact pad endpoint rules.
    pub item: Option<ItemKey>,
}

/// A source-compatible dense corridor produced from original KiCad pad
/// metadata. Coordinates and measured values use millimetres.
#[derive(Clone, Debug)]
pub struct DensePadField {
    pub component_ref: String,
    pub bounds_mm: [f64; 4],
    pub pitch_mm: f64,
    pub median_pad_size_mm: f64,
}

#[derive(Clone, Debug)]
pub struct SmoothingLeg {
    pub owner: ItemKey,
    /// Leg index inside the owning native polyline item.
    pub leg_index: usize,
    pub a: Point,
    pub b: Point,
    /// Chain extraction only walks selected legs; retained explicitly for callers.
    pub selected: bool,
}

#[derive(Clone, Debug)]
pub struct SmoothingChain {
    pub net: NetNo,
    pub layer: i32,
    pub half_width: i32,
    pub clearance_class: ClearanceClassNo,
    pub legs: Vec<SmoothingLeg>,
    pub touches: Vec<SmoothingTouch>,
    /// Dense-field bounds in native coordinates, limited to fields intersecting
    /// the original chain bounds. The caller checks each candidate segment.
    pub dense_bounds: Vec<[f64; 4]>,
}

#[derive(Clone)]
struct Edge {
    owner: ItemKey,
    leg: usize,
    a: Point,
    b: Point,
    selected: bool,
}

type Vertex = (i64, i64);
type Group = (NetNo, i32, i32, ClearanceClassNo);

fn vertex_key(p: &Point, units_per_mm: f64) -> Vertex {
    let p = p.to_float();
    // The source graph uses 0.001 mm endpoint keys. Keep that tolerance while
    // retaining each edge's original native endpoint for emitted geometry.
    (
        (p.x / units_per_mm * 1000.0).round() as i64,
        (p.y / units_per_mm * 1000.0).round() as i64,
    )
}

fn d(a: FloatPoint, b: FloatPoint) -> f64 {
    (a.x - b.x).hypot(a.y - b.y)
}

fn point_segment(p: FloatPoint, a: FloatPoint, b: FloatPoint) -> (f64, FloatPoint) {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let den = dx * dx + dy * dy;
    let t = if den <= 1e-18 {
        0.0
    } else {
        (((p.x - a.x) * dx + (p.y - a.y) * dy) / den).clamp(0.0, 1.0)
    };
    let q = FloatPoint::new(a.x + t * dx, a.y + t * dy);
    (d(p, q), q)
}

fn cross(a: FloatPoint, b: FloatPoint, c: FloatPoint) -> f64 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

fn segment_closest(
    a: FloatPoint,
    b: FloatPoint,
    c: FloatPoint,
    e: FloatPoint,
) -> (f64, FloatPoint) {
    let (ab_c, ab_e, ce_a, ce_b) = (
        cross(a, b, c),
        cross(a, b, e),
        cross(c, e, a),
        cross(c, e, b),
    );
    if ((ab_c <= 0.0 && ab_e >= 0.0) || (ab_c >= 0.0 && ab_e <= 0.0))
        && ((ce_a <= 0.0 && ce_b >= 0.0) || (ce_a >= 0.0 && ce_b <= 0.0))
    {
        let den = (b.x - a.x) * (e.y - c.y) - (b.y - a.y) * (e.x - c.x);
        if den.abs() > 1e-18 {
            let t = ((c.x - a.x) * (e.y - c.y) - (c.y - a.y) * (e.x - c.x)) / den;
            return (
                0.0,
                FloatPoint::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)),
            );
        }
    }
    let candidates = [
        (point_segment(a, c, e), a),
        (point_segment(b, c, e), b),
        (point_segment(c, a, b), c),
        (point_segment(e, a, b), e),
    ];
    let mut best = (f64::INFINITY, FloatPoint::new(0.0, 0.0));
    for ((distance, q), p) in candidates {
        if distance < best.0 {
            best = (
                distance,
                FloatPoint::new((p.x + q.x) / 2.0, (p.y + q.y) / 2.0),
            );
        }
    }
    best
}

fn point_near_item(board: &RoutingBoard, key: ItemKey, point: FloatPoint, reach: f64) -> bool {
    let item = board.item(key);
    let center = item.center(&board.basic).to_float();
    let bb = item.bounding_box(&board.basic);
    // The bounding-circle test conservatively anchors the vertex for rotated
    // and non-rectangular pad shapes.
    let radius = 0.5 * (bb.width() as f64).hypot(bb.height() as f64) + reach;
    d(center, point) <= radius
}

fn edge_points(edge: &Edge) -> (FloatPoint, FloatPoint) {
    (edge.a.to_float(), edge.b.to_float())
}

/// Extracts source-style chains from selected native tracks. Chain leg indices
/// and touch leg indices are global within each returned chain.
pub fn extract_smoothing_chains(
    board: &RoutingBoard,
    lane_items: &[ItemKey],
    units_per_mm: f64,
    dense_fields: &[DensePadField],
) -> Vec<SmoothingChain> {
    if !units_per_mm.is_finite() || units_per_mm <= 0.0 {
        return Vec::new();
    }
    let tol = COINCIDENCE_TOL_MM * units_per_mm;
    let selected: BTreeSet<_> = lane_items.iter().copied().collect();
    let mut all_by_net: BTreeMap<NetNo, Vec<(Group, Edge)>> = BTreeMap::new();
    let mut group_edges: BTreeMap<Group, Vec<Edge>> = BTreeMap::new();
    let mut endpoint_incidence: BTreeMap<(NetNo, Vertex), usize> = BTreeMap::new();

    for owner in board.get_traces() {
        let item = board.item(owner);
        if !item.is_on_board() || !item.nets_normal() {
            continue;
        }
        let Some(&net) = item.net_numbers().first() else {
            continue;
        };
        let trace = item.trace();
        let layer = trace.layer();
        let half_width = trace.half_width();
        let class = item.clearance_class();
        let group = (net, layer, half_width, class);
        let vertices = trace.polyline().corners();
        for (leg, pair) in vertices.windows(2).enumerate() {
            let edge = Edge {
                owner,
                leg,
                a: pair[0].clone(),
                b: pair[1].clone(),
                selected: selected.contains(&owner),
            };
            let ka = vertex_key(&edge.a, units_per_mm);
            let kb = vertex_key(&edge.b, units_per_mm);
            *endpoint_incidence.entry((net, ka)).or_default() += 1;
            *endpoint_incidence.entry((net, kb)).or_default() += 1;
            all_by_net
                .entry(net)
                .or_default()
                .push((group, edge.clone()));
            if edge.selected {
                group_edges.entry(group).or_default().push(edge);
            }
        }
    }

    let mut chains = Vec::new();
    for ((net, layer, half_width, clearance_class), edges) in group_edges {
        let mut adjacency: BTreeMap<Vertex, Vec<usize>> = BTreeMap::new();
        for (i, edge) in edges.iter().enumerate() {
            adjacency
                .entry(vertex_key(&edge.a, units_per_mm))
                .or_default()
                .push(i);
            adjacency
                .entry(vertex_key(&edge.b, units_per_mm))
                .or_default()
                .push(i);
        }

        let mut anchored = BTreeSet::new();
        for keys in board.get_connectable_items(net) {
            let item = board.item(keys);
            if !item.is_on_board() || item.is_trace() {
                continue;
            }
            let is_pad = item.is_pin();
            let is_via = item.is_via();
            if !is_pad && !is_via {
                continue;
            }
            if layer < item.first_layer(&board.basic) || layer > item.last_layer(&board.basic) {
                continue;
            }
            for vertex in adjacency.keys() {
                let p = edges
                    .iter()
                    .find_map(|e| {
                        (vertex_key(&e.a, units_per_mm) == *vertex)
                            .then(|| e.a.to_float())
                            .or_else(|| {
                                (vertex_key(&e.b, units_per_mm) == *vertex).then(|| e.b.to_float())
                            })
                    })
                    .unwrap_or_else(|| FloatPoint::new(vertex.0 as f64, vertex.1 as f64));
                let reach = half_width as f64 + tol;
                if point_near_item(board, keys, p, reach) {
                    anchored.insert(*vertex);
                }
            }
        }

        let is_interior = |v: &Vertex| {
            adjacency.get(v).is_some_and(|incident| incident.len() == 2)
                && endpoint_incidence.get(&(net, *v)).copied().unwrap_or(0) == 2
                && !anchored.contains(v)
        };
        let mut used = BTreeSet::new();
        let anchors: Vec<_> = adjacency
            .keys()
            .filter(|v| !is_interior(v))
            .copied()
            .collect();
        for start in anchors {
            let incident = adjacency.get(&start).cloned().unwrap_or_default();
            for first in incident {
                if used.contains(&first) {
                    continue;
                }
                let mut current = start;
                let mut chain = Vec::new();
                let mut closed = false;
                let mut edge_index = first;
                loop {
                    if !used.insert(edge_index) {
                        break;
                    }
                    let edge = &edges[edge_index];
                    let next_key = if vertex_key(&edge.a, units_per_mm) == current {
                        vertex_key(&edge.b, units_per_mm)
                    } else {
                        vertex_key(&edge.a, units_per_mm)
                    };
                    let mut oriented = edge.clone();
                    if vertex_key(&oriented.a, units_per_mm) != current {
                        std::mem::swap(&mut oriented.a, &mut oriented.b);
                    }
                    chain.push(oriented);
                    current = next_key;
                    if current == start {
                        closed = true;
                        break;
                    }
                    if !is_interior(&current) || chain.len() >= 100 {
                        break;
                    }
                    let Some(next) = adjacency
                        .get(&current)
                        .and_then(|indices| indices.iter().copied().find(|i| !used.contains(i)))
                    else {
                        break;
                    };
                    edge_index = next;
                }
                if closed || chain.len() < 2 {
                    continue;
                }
                let mut result = SmoothingChain {
                    net,
                    layer,
                    half_width,
                    clearance_class,
                    legs: chain
                        .iter()
                        .map(|e| SmoothingLeg {
                            owner: e.owner,
                            leg_index: e.leg,
                            a: e.a.clone(),
                            b: e.b.clone(),
                            selected: e.selected,
                        })
                        .collect(),
                    touches: Vec::new(),
                    dense_bounds: chain_dense_bounds(&chain, dense_fields, units_per_mm),
                };
                result.touches = collect_touches(board, &result, &chain, &all_by_net, tol);
                chains.push(result);
            }
        }
    }
    chains
}

fn collect_touches(
    board: &RoutingBoard,
    result: &SmoothingChain,
    chain_edges: &[Edge],
    all_by_net: &BTreeMap<NetNo, Vec<(Group, Edge)>>,
    tol: f64,
) -> Vec<SmoothingTouch> {
    let chain_ids: BTreeSet<_> = chain_edges.iter().map(|e| (e.owner, e.leg)).collect();
    let mut touches = Vec::new();
    for (leg_index, edge) in chain_edges.iter().enumerate() {
        let (a, b) = edge_points(edge);
        for (group, sibling) in all_by_net.get(&result.net).into_iter().flatten() {
            if chain_ids.contains(&(sibling.owner, sibling.leg)) || group.1 != result.layer {
                continue;
            }
            // Group widths are stored as half-widths, and source touch reach is
            // the sum of both copper radii plus coincidence tolerance.
            let reach = (result.half_width + sibling_group_width(group)) as f64 + tol;
            let (c, e) = edge_points(sibling);
            let (distance, point) = segment_closest(a, b, c, e);
            if distance < reach {
                touches.push(SmoothingTouch {
                    point,
                    leg_index,
                    reach,
                    kind: SmoothingTouchKind::SiblingTrack,
                    item: Some(sibling.owner),
                });
            }
        }
        for key in board.get_connectable_items(result.net) {
            let item = board.item(key);
            if !item.is_on_board() || item.is_trace() {
                continue;
            }
            let kind = if item.is_pin() {
                SmoothingTouchKind::Pad
            } else if item.is_via() {
                SmoothingTouchKind::Via
            } else {
                continue;
            };
            if result.layer < item.first_layer(&board.basic)
                || result.layer > item.last_layer(&board.basic)
            {
                continue;
            }
            let center = item.center(&board.basic).to_float();
            let bb = item.bounding_box(&board.basic);
            let reach = 0.5 * (bb.width() as f64).hypot(bb.height() as f64)
                + result.half_width as f64
                + tol;
            let (distance, point) = point_segment(center, a, b);
            if distance < reach {
                touches.push(SmoothingTouch {
                    point,
                    leg_index,
                    reach,
                    kind,
                    item: Some(key),
                });
            }
        }
    }
    touches
}

fn sibling_group_width(group: &Group) -> i32 {
    group.2
}

fn chain_dense_bounds(
    chain: &[Edge],
    fields: &[DensePadField],
    units_per_mm: f64,
) -> Vec<[f64; 4]> {
    if chain.is_empty() {
        return Vec::new();
    }
    let bounds = chain.iter().fold(
        [
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ],
        |mut acc, edge| {
            let a = edge.a.to_float();
            let b = edge.b.to_float();
            acc[0] = acc[0].min(a.x).min(b.x);
            acc[1] = acc[1].min(a.y).min(b.y);
            acc[2] = acc[2].max(a.x).max(b.x);
            acc[3] = acc[3].max(a.y).max(b.y);
            acc
        },
    );
    fields
        .iter()
        .filter_map(|field| {
            let [x0, y0, x1, y1] = field.bounds_mm;
            if ![x0, y0, x1, y1].iter().all(|v| v.is_finite()) || x0 > x1 || y0 > y1 {
                return None;
            }
            let native = [
                x0 * units_per_mm,
                y0 * units_per_mm,
                x1 * units_per_mm,
                y1 * units_per_mm,
            ];
            (bounds[0] <= native[2]
                && bounds[2] >= native[0]
                && bounds[1] <= native[3]
                && bounds[3] >= native[1])
                .then_some(native)
        })
        .collect()
}
