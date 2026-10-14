//! Destination realization and read-back audit for the AWX joint escape plan.
use super::{
    escape_moves::{self, Direction, Move, MoveKind},
    joint_escape::{JointPlan, OptionMove},
    native_execution::{self, NativeGrade},
    source_realize,
    whole_context::RoutingRules,
};
use crate::board_adapter::BoardAdapter;
use serde::Serialize;
use std::collections::{BTreeMap, BTreeSet};

const GAP_TOL_MM: f64 = 0.2;

#[derive(Clone, Debug, Serialize)]
pub struct DestinationNetDecision {
    pub net: String,
    pub accepted: bool,
    pub reason: String,
    pub grade: NativeGrade,
    pub traces_added: usize,
    pub vias_added: usize,
    pub copper_mm_added: f64,
}

#[derive(Clone, Debug, Serialize)]
pub struct DestinationMoveAudit {
    pub ball: String,
    pub net: String,
    pub exact: bool,
    pub asked_direction: Direction,
    pub achieved_direction: Option<Direction>,
    pub asked_layer: String,
    pub achieved_layer: Option<String>,
    pub asked_kind: MoveKind,
    pub achieved_kind: Option<MoveKind>,
    pub asked_exit: [f64; 2],
    pub achieved_tooth: Option<[f64; 2]>,
    pub gap_mm: Option<f64>,
    pub outward_mm: Option<f64>,
    pub vias: usize,
    pub lost: Vec<String>,
    pub readback_error: Option<String>,
}

#[derive(Clone, Debug, Serialize)]
pub struct DestinationBanClass {
    pub kind: MoveKind,
    pub direction: Direction,
    pub layer: String,
    pub exit_pt_rounded: [f64; 2],
}

/// Complete refusal feedback for the destination planner. The full move and
/// pair name let the caller ban a single move or the pair as one choice.
#[derive(Clone, Debug, Serialize)]
pub struct DestinationMoveRefusal {
    pub ball: String,
    pub net: String,
    pub movement: Move,
    pub ban_class: DestinationBanClass,
    pub pair: Option<String>,
    pub reason: String,
}

#[derive(Clone)]
pub struct DestinationRealization {
    pub board: BoardAdapter,
    pub grade: NativeGrade,
    pub nets: Vec<DestinationNetDecision>,
    pub audits: Vec<DestinationMoveAudit>,
    pub refusals: Vec<DestinationMoveRefusal>,
    /// (same-rank positions, audited count, inversion count), by asked face.
    pub order: BTreeMap<Direction, (usize, usize, usize)>,
    pub traces_added: usize,
    pub vias_added: usize,
    pub copper_mm_added: f64,
}

#[derive(Clone)]
pub struct AskedBerth {
    pub ball: String,
    pub movement: Move,
    pub pair: Option<String>,
    pub pad: AskedPad,
    pub destination_ref: Option<String>,
}

#[derive(Clone, Debug)]
pub struct AskedPad {
    pub net: String,
    pub component_ref: String,
    pub pad_number: String,
    pub point: [f64; 2],
    pub size: [f64; 2],
}

/// Resolve the original physical pad identity from the plan piece's board
/// point. A net/pad-number pair alone is not unique across components.
pub fn asked_berth_for_piece(
    board: &BoardAdapter,
    ball: String,
    movement: Move,
    pair: Option<String>,
    piece: &super::joint_geometry::Piece,
    destination_end: bool,
) -> Result<AskedBerth, String> {
    let (_, pad_number) = ball
        .rsplit_once('#')
        .ok_or_else(|| format!("invalid AWX physical pad key {ball}"))?;
    let net = full_net(board, &movement.net)?;
    let anchor = *piece
        .balls
        .first()
        .ok_or_else(|| format!("AWX physical piece for {ball} has no pad anchor"))?;
    let matches: Vec<_> = board
        .pads()
        .into_iter()
        .filter(|pad| {
            pad.net == net && pad.pad == pad_number && distance(pad.point, anchor) <= 0.005
        })
        .collect();
    if matches.len() != 1 {
        return Err(format!(
            "AWX physical pad identity for {ball} at {anchor:?} resolved to {} native pads",
            matches.len()
        ));
    }
    let pad = &matches[0];
    let size = native_pad_copper_size(board, pad)?;
    Ok(AskedBerth {
        ball,
        movement,
        pair,
        pad: AskedPad {
            net: pad.net.clone(),
            component_ref: pad.chip.clone(),
            pad_number: pad.pad.clone(),
            point: pad.point,
            size,
        },
        destination_ref: destination_end.then(|| pad.chip.clone()),
    })
}

fn native_pad_copper_size(
    board: &BoardAdapter,
    pad: &crate::board_adapter::Pad,
) -> Result<[f64; 2], String> {
    let key = pad
        .key
        .ok_or_else(|| format!("native pad {}.{} lacks item identity", pad.chip, pad.pad))?;
    let item = board.board.item(key);
    let padstack = item
        .padstack(&board.board)
        .ok_or_else(|| format!("native pad {}.{} lacks copper padstack", pad.chip, pad.pad))?;
    let mut size = [0.0f64; 2];
    for layer in 0..padstack.board_layer_count() {
        let Some(shape) = padstack.get_shape(layer) else {
            continue;
        };
        let bounds = shape.bounding_box();
        size[0] = size[0].max(bounds.width() as f64 / board.units_per_mm);
        size[1] = size[1].max(bounds.height() as f64 / board.units_per_mm);
    }
    if !size.iter().all(|value| value.is_finite()) || size == [0.0, 0.0] {
        return Err(format!(
            "native pad {}.{} has no measurable copper size",
            pad.chip, pad.pad
        ));
    }
    Ok(size)
}

#[derive(Clone, Debug)]
pub struct AchievedBerth {
    pub tooth: [f64; 2],
    pub direction: Direction,
    pub layer: String,
    pub kind: MoveKind,
    pub vias: usize,
    pub site: Option<[f64; 2]>,
}

fn distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    (b[0] - a[0]).hypot(b[1] - a[1])
}

fn snap_direction(dx: f64, dy: f64) -> Direction {
    let length = dx.hypot(dy).max(1e-12);
    Direction::ALL
        .into_iter()
        .min_by(|a, b| {
            let score = |d: Direction| {
                let (x, y) = d.delta();
                (x as f64 - dx / length).powi(2) + (y as f64 - dy / length).powi(2)
            };
            score(*a).total_cmp(&score(*b))
        })
        .unwrap_or(Direction::Up)
}

fn full_net(board: &BoardAdapter, name: &str) -> Result<String, String> {
    if board.net_number(name).is_ok() {
        return Ok(name.to_owned());
    }
    let aliases: BTreeMap<String, Vec<String>> = (1..=board.board.rules.nets.max_net_number())
        .filter_map(|n| board.board.rules.nets.get(n))
        .fold(BTreeMap::new(), |mut map, net| {
            map.entry(escape_moves::short_name(&net.name).to_owned())
                .or_default()
                .push(net.name.clone());
            map
        });
    let hits = aliases
        .get(name)
        .ok_or_else(|| format!("unknown AWX net {name}"))?;
    if hits.len() != 1 {
        return Err(format!("ambiguous AWX basename {name}"));
    }
    Ok(hits[0].clone())
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
enum EndpointNode {
    Point((i64, i64)),
    Via(usize),
}

#[derive(Default)]
struct EndpointSnapper {
    seen: BTreeMap<(i64, i64), Vec<(i64, i64)>>,
}

impl EndpointSnapper {
    fn key(&mut self, point: [f64; 2]) -> (i64, i64) {
        const TOL: f64 = 0.006;
        let bucket = (
            (point[0] / (2.0 * TOL)).floor() as i64,
            (point[1] / (2.0 * TOL)).floor() as i64,
        );
        for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(keys) = self.seen.get(&(bucket.0 + dx, bucket.1 + dy)) {
                    for &key in keys {
                        let existing = [key.0 as f64 / 1000.0, key.1 as f64 / 1000.0];
                        if (existing[0] - point[0]).abs() <= TOL
                            && (existing[1] - point[1]).abs() <= TOL
                        {
                            return key;
                        }
                    }
                }
            }
        }
        let key = (round_mill(point[0]), round_mill(point[1]));
        self.seen.entry(bucket).or_default().push(key);
        key
    }
}

fn round_mill(value: f64) -> i64 {
    (value * 1000.0).round_ties_even() as i64
}

fn key_point(key: (i64, i64)) -> [f64; 2] {
    [key.0 as f64 / 1000.0, key.1 as f64 / 1000.0]
}

fn native_via_size(board: &BoardAdapter, key: fr_engine::board::ItemKey) -> Result<f64, String> {
    let item = board.board.item(key);
    let padstack = item
        .padstack(&board.board)
        .ok_or("native via lacks copper padstack")?;
    let mut diameter = 0.0f64;
    for layer in 0..padstack.board_layer_count() {
        if let Some(shape) = padstack.get_shape(layer) {
            let bounds = shape.bounding_box();
            diameter =
                diameter.max(bounds.width().max(bounds.height()) as f64 / board.units_per_mm);
        }
    }
    if !diameter.is_finite() || diameter <= 0.0 {
        return Err("native via has no measurable copper diameter".into());
    }
    Ok(diameter)
}

/// Native twin of `source_realize.measure_tooth` / `braid.endpoints`.
/// `destination_ref=None` uses the source/default terminal; Some(component)
/// attributes the destination terminal to that component. A bare net or a
/// board with no free endpoint returns None.
pub fn measure_tooth(
    board: &BoardAdapter,
    asked_pad: &AskedPad,
    destination_ref: Option<&str>,
) -> Result<Option<AchievedBerth>, String> {
    let net = full_net(board, &asked_pad.net)?;
    let _target = board
        .pads()
        .into_iter()
        .find(|pad| {
            pad.net == net
                && pad.chip == asked_pad.component_ref
                && pad.pad == asked_pad.pad_number
                && distance(pad.point, asked_pad.point) <= 0.005
        })
        .ok_or_else(|| {
            format!(
                "AWX physical pad {}#{} on {} is absent from native board",
                asked_pad.component_ref,
                asked_pad.pad_number,
                asked_pad
                    .point
                    .iter()
                    .map(|v| format!("{v:.3}"))
                    .collect::<Vec<_>>()
                    .join(",")
            )
        })?;
    let net_id = board.net_number(&net)?;
    let all_pads = board.pads();
    let pads: Vec<_> = all_pads
        .iter()
        .filter(|pad| pad.net == net)
        .cloned()
        .collect();
    let array_pads: Vec<_> = all_pads
        .iter()
        .filter(|pad| pad.chip == asked_pad.component_ref)
        .map(|pad| escape_moves::Pad {
            key: format!("{}#{}", pad.net, pad.pad),
            net: pad.net.clone(),
            net_id: 0,
            component_ref: pad.chip.clone(),
            pad_number: pad.pad.clone(),
            point: pad.point,
            layers: pad.layers.clone(),
            size: [0.0, 0.0],
        })
        .collect();
    let grid = escape_moves::grid_of(&array_pads)?;
    let mut pad_geometries = Vec::with_capacity(pads.len());
    for pad in &pads {
        pad_geometries.push((
            pad.point,
            native_pad_copper_size(board, pad)?,
            pad.chip.clone(),
        ));
    }
    let mut vias = Vec::<([f64; 2], f64)>::new();
    for key in board.board.get_vias() {
        let item = board.board.item(key);
        if !item.contains_net(net_id) {
            continue;
        }
        let center = item.center(&board.board).to_float();
        vias.push((
            [center.x / board.units_per_mm, center.y / board.units_per_mm],
            native_via_size(board, key)?,
        ));
    }
    let mut edges = Vec::<([f64; 2], [f64; 2], String)>::new();
    for key in board.board.get_traces() {
        let item = board.board.item(key);
        if !item.contains_net(net_id) {
            continue;
        }
        let trace = item.trace();
        let layer = board.board.layer_structure.layers[trace.layer() as usize]
            .name
            .clone();
        let corners = trace.polyline().corner_approx_arr();
        for pair in corners.windows(2) {
            edges.push((
                [
                    pair[0].x / board.units_per_mm,
                    pair[0].y / board.units_per_mm,
                ],
                [
                    pair[1].x / board.units_per_mm,
                    pair[1].y / board.units_per_mm,
                ],
                layer.clone(),
            ));
        }
    }
    let mut snapper = EndpointSnapper::default();
    let mut neighbors = BTreeMap::<(i64, i64), BTreeSet<EndpointNode>>::new();
    let mut point_order = Vec::new();
    for (a, b, _) in &edges {
        let ka = snapper.key(*a);
        let kb = snapper.key(*b);
        if ka == kb {
            continue;
        }
        if !neighbors.contains_key(&ka) {
            point_order.push(ka);
        }
        neighbors.entry(ka).or_default();
        if !neighbors.contains_key(&kb) {
            point_order.push(kb);
        }
        neighbors.entry(kb).or_default();
        let place = |key: (i64, i64)| -> EndpointNode {
            let point = key_point(key);
            vias.iter()
                .position(|(center, size)| distance(point, *center) <= size / 2.0)
                .map(EndpointNode::Via)
                .unwrap_or(EndpointNode::Point(key))
        };
        neighbors.get_mut(&ka).unwrap().insert(place(kb));
        neighbors.get_mut(&kb).unwrap().insert(place(ka));
    }
    let anchors: Vec<_> = pad_geometries
        .iter()
        .map(|(point, size, _)| (*point, size[0].max(size[1]) / 2.0))
        .chain(vias.iter().map(|(point, size)| (*point, size / 2.0)))
        .collect();
    let is_free = |key: (i64, i64), anchors: &[([f64; 2], f64)]| {
        let point = key_point(key);
        anchors
            .iter()
            .all(|(anchor, radius)| distance(point, *anchor) > 0.02f64.max(*radius))
    };
    let mut free: Vec<_> = point_order
        .iter()
        .copied()
        .filter(|key| {
            neighbors.get(key).is_some_and(|nodes| nodes.len() == 1) && is_free(*key, &anchors)
        })
        .map(key_point)
        .collect();
    if free.is_empty() {
        let pads_only: Vec<_> = pad_geometries
            .iter()
            .map(|(point, size, _)| (*point, size[0].max(size[1]) / 2.0))
            .collect();
        free = point_order
            .iter()
            .copied()
            .filter(|key| {
                neighbors.get(key).is_some_and(|nodes| nodes.len() == 1)
                    && is_free(*key, &pads_only)
            })
            .map(key_point)
            .collect();
    }
    if free.is_empty() {
        return Ok(None);
    }
    if let Some(destination_ref) = destination_ref {
        let mut at_dest = Vec::new();
        for point in &free {
            if native_endpoint_owner(*point, &edges, &pad_geometries) == destination_ref {
                at_dest.push(*point);
            }
        }
        let mut at_src: Vec<_> = free
            .iter()
            .copied()
            .filter(|point| !at_dest.contains(point))
            .collect();
        if at_dest.is_empty() || at_src.is_empty() {
            let pads_only: Vec<_> = pad_geometries
                .iter()
                .map(|(point, size, _)| (*point, size[0].max(size[1]) / 2.0))
                .collect();
            let free_v: Vec<_> = point_order
                .iter()
                .copied()
                .filter(|key| {
                    neighbors.get(key).is_some_and(|nodes| nodes.len() == 1)
                        && is_free(*key, &pads_only)
                })
                .map(key_point)
                .collect();
            if at_dest.is_empty() {
                at_dest = free_v
                    .iter()
                    .copied()
                    .filter(|point| {
                        native_endpoint_owner(*point, &edges, &pad_geometries) == destination_ref
                    })
                    .collect();
            }
            if at_src.is_empty() {
                at_src = free_v
                    .iter()
                    .copied()
                    .filter(|point| {
                        !at_dest.contains(point)
                            && native_endpoint_owner(*point, &edges, &pad_geometries)
                                != destination_ref
                    })
                    .collect();
            }
        }
        if !at_dest.is_empty() && !at_src.is_empty() {
            let mut farthest_pair = None;
            let mut farthest_distance = f64::NEG_INFINITY;
            for src in &at_src {
                for dest in &at_dest {
                    let d = distance(*src, *dest);
                    if d > farthest_distance {
                        farthest_distance = d;
                        farthest_pair = Some((*src, *dest));
                    }
                }
            }
            let Some((src, dest)) = farthest_pair else {
                return Ok(None);
            };
            let tooth = if asked_pad.component_ref == destination_ref {
                dest
            } else {
                src
            };
            return measure_selected_tooth(asked_pad, tooth, &edges, &vias, &grid);
        }
        if !at_src.is_empty() {
            free = at_src;
        }
    }
    if free.len() > 1 {
        free.sort_by(|a, b| {
            let nearest = |point: [f64; 2]| {
                anchors
                    .iter()
                    .map(|(anchor, _)| distance(point, *anchor))
                    .fold(f64::INFINITY, f64::min)
            };
            nearest(*b).total_cmp(&nearest(*a))
        });
    }
    measure_selected_tooth(asked_pad, free[0], &edges, &vias, &grid)
}

fn native_endpoint_owner(
    point: [f64; 2],
    edges: &[([f64; 2], [f64; 2], String)],
    pads: &[([f64; 2], [f64; 2], String)],
) -> String {
    let mut snapper = EndpointSnapper::default();
    let mut graph = BTreeMap::<(i64, i64), BTreeSet<(i64, i64)>>::new();
    for (a, b, _) in edges {
        let (ka, kb) = (snapper.key(*a), snapper.key(*b));
        graph.entry(ka).or_default().insert(kb);
        graph.entry(kb).or_default().insert(ka);
    }
    let start = snapper.key(point);
    let mut seen = BTreeSet::from([start]);
    let mut pending = vec![start];
    while let Some(node) = pending.pop() {
        if let Some(next) = graph.get(&node) {
            for other in next {
                if seen.insert(*other) {
                    pending.push(*other);
                }
            }
        }
    }
    for (center, size, component) in pads {
        let rx = size[0] / 2.0 + 0.02;
        let ry = size[1] / 2.0 + 0.02;
        if seen.iter().any(|key| {
            let p = key_point(*key);
            (p[0] - center[0]).abs() <= rx && (p[1] - center[1]).abs() <= ry
        }) {
            return component.clone();
        }
    }
    pads.iter()
        .min_by(|a, b| distance(a.0, point).total_cmp(&distance(b.0, point)))
        .map(|pad| pad.2.clone())
        .unwrap_or_default()
}

fn measure_selected_tooth(
    asked_pad: &AskedPad,
    tooth: [f64; 2],
    edges: &[([f64; 2], [f64; 2], String)],
    vias: &[([f64; 2], f64)],
    grid: &escape_moves::Grid,
) -> Result<Option<AchievedBerth>, String> {
    let last = edges.iter().find(|(a, b, _)| {
        (a[0] - tooth[0]).abs() + (a[1] - tooth[1]).abs() < 0.005
            || (b[0] - tooth[0]).abs() + (b[1] - tooth[1]).abs() < 0.005
    });
    let (other, layer) = if let Some((a, b, layer)) = last {
        if (b[0] - tooth[0]).abs() + (b[1] - tooth[1]).abs() < 0.005 {
            (*a, layer.clone())
        } else {
            (*b, layer.clone())
        }
    } else {
        (asked_pad.point, "?".to_owned())
    };
    let segment_direction = snap_direction(tooth[0] - other[0], tooth[1] - other[1]);
    let hx = grid.pitch_x / 2.0;
    let hy = grid.pitch_y / 2.0;
    let beyond = [
        (Direction::Right, tooth[0] > grid.bbox[2] + hx * 0.5),
        (Direction::Left, tooth[0] < grid.bbox[0] - hx * 0.5),
        (Direction::Down, tooth[1] > grid.bbox[3] + hy * 0.5),
        (Direction::Up, tooth[1] < grid.bbox[1] - hy * 0.5),
    ];
    let outside: Vec<_> = beyond
        .iter()
        .filter(|(_, yes)| *yes)
        .map(|(direction, _)| *direction)
        .collect();
    let direction = if outside.len() == 1 {
        outside[0]
    } else if outside.contains(&segment_direction) || outside.is_empty() {
        segment_direction
    } else {
        outside[0]
    };
    let near_vias: Vec<_> = vias
        .iter()
        .filter(|(point, _)| distance(*point, asked_pad.point) < 6.0)
        .collect();
    let kind = if near_vias.iter().any(|(point, size)| {
        distance(*point, asked_pad.point) <= asked_pad.size[0].max(asked_pad.size[1]) / 2.0 + 0.01
    }) {
        MoveKind::ViaInPad
    } else if !near_vias.is_empty() {
        MoveKind::Dogbone
    } else {
        MoveKind::Surface
    };
    Ok(Some(AchievedBerth {
        tooth,
        direction,
        layer,
        kind,
        vias: near_vias.len(),
        site: near_vias.first().map(|(point, _)| *point),
    }))
}

pub fn audit_berth(board: &BoardAdapter, ask: &AskedBerth) -> Result<DestinationMoveAudit, String> {
    let (got, readback_error) = match measure_tooth(board, &ask.pad, ask.destination_ref.as_deref())
    {
        Ok(got) => (got, None),
        Err(error) => (None, Some(error)),
    };
    let axis = ask.movement.direction.along_axis();
    let outward_axis = 1 - axis;
    let (tooth, direction, layer, kind, gap, outward, vias) = match &got {
        Some(g) => (
            Some(g.tooth),
            Some(g.direction),
            Some(g.layer.clone()),
            Some(g.kind),
            Some((g.tooth[axis] - ask.movement.exit_pt[axis]).abs()),
            Some((g.tooth[outward_axis] - ask.movement.exit_pt[outward_axis]).abs()),
            g.vias,
        ),
        None => (None, None, None, None, None, None, 0),
    };
    let exact = direction == Some(ask.movement.direction)
        && layer.as_deref() == Some(ask.movement.layer.as_str())
        && kind == Some(ask.movement.kind)
        && gap.is_some_and(|v| v <= GAP_TOL_MM);
    let mut lost = Vec::new();
    if direction != Some(ask.movement.direction) {
        lost.push("face".into());
    }
    if layer.as_deref() != Some(ask.movement.layer.as_str()) {
        lost.push("layer".into());
    }
    if kind != Some(ask.movement.kind) {
        lost.push("kind".into());
    }
    if !gap.is_some_and(|v| v <= GAP_TOL_MM) {
        lost.push("gap".into());
    }
    Ok(DestinationMoveAudit {
        ball: ask.ball.clone(),
        net: ask.movement.net.clone(),
        exact,
        asked_direction: ask.movement.direction,
        achieved_direction: direction,
        asked_layer: ask.movement.layer.clone(),
        achieved_layer: layer,
        asked_kind: ask.movement.kind,
        achieved_kind: kind,
        asked_exit: ask.movement.exit_pt,
        achieved_tooth: tooth,
        gap_mm: gap,
        outward_mm: outward,
        vias,
        lost,
        readback_error,
    })
}

fn ban_class(movement: &Move) -> DestinationBanClass {
    DestinationBanClass {
        kind: movement.kind,
        direction: movement.direction,
        layer: movement.layer.clone(),
        exit_pt_rounded: movement.exit_pt.map(|v| (v * 100.0).round() / 100.0),
    }
}

pub fn refused_audit(ask: &AskedBerth, reason: &str) -> DestinationMoveAudit {
    DestinationMoveAudit {
        ball: ask.ball.clone(),
        net: ask.movement.net.clone(),
        exact: false,
        asked_direction: ask.movement.direction,
        achieved_direction: None,
        asked_layer: ask.movement.layer.clone(),
        achieved_layer: None,
        asked_kind: ask.movement.kind,
        achieved_kind: None,
        asked_exit: ask.movement.exit_pt,
        achieved_tooth: None,
        gap_mm: None,
        outward_mm: None,
        vias: 0,
        lost: Vec::new(),
        readback_error: Some(reason.into()),
    }
}

pub fn refusal_for(ask: &AskedBerth, reason: String) -> DestinationMoveRefusal {
    DestinationMoveRefusal {
        ball: ask.ball.clone(),
        net: ask.movement.net.clone(),
        movement: ask.movement.clone(),
        ban_class: ban_class(&ask.movement),
        pair: ask.pair.clone(),
        reason,
    }
}

fn order_agreement(audits: &[DestinationMoveAudit]) -> BTreeMap<Direction, (usize, usize, usize)> {
    let mut result = BTreeMap::new();
    for face in Direction::ALL {
        let mut asks: Vec<_> = audits
            .iter()
            .filter(|a| a.asked_direction == face && a.achieved_direction == Some(face))
            .collect();
        if asks.len() < 2 {
            continue;
        }
        let axis = face.along_axis();
        let mut asked = asks.clone();
        let mut got = asks.clone();
        asked.sort_by(|a, b| a.asked_exit[axis].total_cmp(&b.asked_exit[axis]));
        got.sort_by(|a, b| {
            a.achieved_tooth.unwrap()[axis].total_cmp(&b.achieved_tooth.unwrap()[axis])
        });
        let ranks: BTreeMap<_, _> = asked
            .iter()
            .enumerate()
            .map(|(i, a)| (a.ball.as_str(), i))
            .collect();
        let seq: Vec<_> = got.iter().map(|a| ranks[a.ball.as_str()]).collect();
        let kept = seq
            .iter()
            .enumerate()
            .filter(|(i, rank)| *i == **rank)
            .count();
        let inversions = (0..seq.len())
            .flat_map(|i| (i + 1..seq.len()).map(move |j| (i, j)))
            .filter(|(i, j)| seq[*i] > seq[*j])
            .count();
        result.insert(face, (kept, seq.len(), inversions));
        asks.clear();
    }
    result
}

/// Realize each destination net transactionally, then read back the native
/// endpoint, face, layer, kind, and gap for every selected escape. Any exact-berth
/// refusal rolls that net back and is returned for the caller's move/pair ban.
pub fn realize_destination(
    board: &BoardAdapter,
    plan: &JointPlan,
    rules: &RoutingRules,
    drills: &mut BTreeMap<String, f64>,
    protected: &BTreeSet<String>,
) -> Result<DestinationRealization, String> {
    let mut by_net: BTreeMap<String, JointPlan> = BTreeMap::new();
    for (key, piece) in &plan.pieces {
        let net = full_net(board, &piece.net)?;
        let subplan = by_net.entry(net).or_insert_with(|| {
            let mut p = plan.clone();
            p.pieces.clear();
            p.chosen.clear();
            p.selected_indices.clear();
            p.hints.clear();
            p
        });
        subplan.pieces.insert(key.clone(), piece.clone());
        if let Some(choice) = plan.chosen.get(key) {
            subplan.chosen.insert(key.clone(), choice.clone());
        }
        if let Some(index) = plan.selected_indices.get(key) {
            subplan.selected_indices.insert(key.clone(), *index);
        }
        if let Some(hint) = plan.hints.iter().find(|h| &h.ball == key) {
            subplan.hints.push(hint.clone());
        }
    }
    // Preserve the source function's empty-plan behavior: no copper is added,
    // the unchanged board is graded, and the decision/refusal lists are empty.
    let mut committed = board.clone();
    let mut committed_drills = drills.clone();
    let mut decisions = Vec::new();
    let mut audits = Vec::new();
    let mut refusals = Vec::new();
    for (net, subplan) in by_net {
        let mut asks = Vec::new();
        for (ball, option) in &subplan.chosen {
            if let OptionMove::Escape { movement } = option {
                if full_net(board, &movement.net)? != net {
                    continue;
                }
                let pair = plan
                    .hints
                    .iter()
                    .find(|h| &h.ball == ball)
                    .and_then(|h| h.hint.get("pair"))
                    .and_then(|v| v.as_str())
                    .map(str::to_owned);
                let piece = subplan
                    .pieces
                    .get(ball)
                    .ok_or_else(|| format!("destination plan lacks physical piece for {ball}"))?;
                asks.push(asked_berth_for_piece(
                    board,
                    ball.clone(),
                    movement.clone(),
                    pair,
                    piece,
                    true,
                )?);
            }
        }
        let mut trial_drills = committed_drills.clone();
        let realized = source_realize::realize_source(
            &committed,
            &subplan,
            rules,
            &mut trial_drills,
            protected,
        )?;
        let mut net_decision = realized
            .nets
            .into_iter()
            .next()
            .ok_or("destination net produced no realization decision")?;
        let mut trial_audits = Vec::new();
        if net_decision.accepted {
            match asks
                .iter()
                .map(|ask| audit_berth(&realized.board, ask))
                .collect::<Result<Vec<_>, _>>()
            {
                Ok(audits) => trial_audits = audits,
                Err(error) => {
                    net_decision.accepted = false;
                    net_decision.reason = format!("destination berth read-back failed: {error}");
                }
            }
            if let Some(failed) = trial_audits.iter().find(|a| !a.exact) {
                net_decision.accepted = false;
                net_decision.reason = format!(
                    "destination berth audit refused {}: {}",
                    failed.ball,
                    failed
                        .readback_error
                        .as_deref()
                        .unwrap_or(&failed.lost.join(", "))
                );
            }
        }
        audits.extend(trial_audits.clone());
        if net_decision.accepted {
            committed = realized.board;
            committed_drills = trial_drills;
            decisions.push(DestinationNetDecision {
                net: net_decision.net,
                accepted: true,
                reason: net_decision.reason,
                grade: net_decision.grade,
                traces_added: net_decision.traces_added,
                vias_added: net_decision.vias_added,
                copper_mm_added: net_decision.copper_mm_added,
            });
        } else {
            let reason = net_decision.reason.clone();
            let grade =
                native_execution::grade(&committed, &committed, &BTreeSet::from([net.clone()]))?;
            decisions.push(DestinationNetDecision {
                net: net_decision.net,
                accepted: false,
                reason: reason.clone(),
                grade,
                traces_added: 0,
                vias_added: 0,
                copper_mm_added: 0.0,
            });
            if trial_audits.is_empty() {
                for ask in &asks {
                    audits.push(refused_audit(ask, &reason));
                    refusals.push(DestinationMoveRefusal {
                        ball: ask.ball.clone(),
                        net: ask.movement.net.clone(),
                        movement: ask.movement.clone(),
                        ban_class: ban_class(&ask.movement),
                        pair: ask.pair.clone(),
                        reason: reason.clone(),
                    });
                }
            } else {
                for (ask, audit) in asks.iter().zip(&trial_audits) {
                    if !audit.exact {
                        let refusal_reason = audit.readback_error.clone().unwrap_or_else(|| {
                            format!("destination berth audit lost {}", audit.lost.join(", "))
                        });
                        refusals.push(DestinationMoveRefusal {
                            ball: ask.ball.clone(),
                            net: ask.movement.net.clone(),
                            movement: ask.movement.clone(),
                            ban_class: ban_class(&ask.movement),
                            pair: ask.pair.clone(),
                            reason: refusal_reason,
                        });
                    }
                }
            }
        }
    }
    audits.sort_by(|a, b| a.ball.cmp(&b.ball));
    refusals.sort_by(|a, b| a.ball.cmp(&b.ball));
    let order = order_agreement(&audits);
    let routed: BTreeSet<_> = decisions.iter().map(|d| d.net.clone()).collect();
    let grade = native_execution::grade(board, &committed, &routed)?;
    let traces_added = committed
        .board
        .get_traces()
        .len()
        .saturating_sub(board.board.get_traces().len());
    let vias_added = committed
        .board
        .get_vias()
        .len()
        .saturating_sub(board.board.get_vias().len());
    let copper_mm_added = decisions
        .iter()
        .filter(|d| d.accepted)
        .map(|d| d.copper_mm_added)
        .sum();
    *drills = committed_drills;
    Ok(DestinationRealization {
        board: committed,
        grade,
        nets: decisions,
        audits,
        refusals,
        order,
        traces_added,
        vias_added,
        copper_mm_added,
    })
}
