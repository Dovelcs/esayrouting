//! Native implementation of the public `smooth_board.py` flow.
//!
//! The caller supplies the native trace items corresponding to the routed
//! board's lane copper after comparing it with the fanout board. This preserves
//! the source's `keep_input_copper=true` boundary. Candidate chains, same-net
//! touches, and fine-pitch fields are extracted by `smoothing_chains`; this
//! module applies the source's two greedy octolinear shortcut phases, native
//! copper checks, and per-net no-worse connectivity guard.

use super::smoothing_chains::{
    extract_smoothing_chains, DensePadField, SmoothingChain, SmoothingTouch, SmoothingTouchKind,
};
use crate::board_adapter::BoardAdapter;
use fr_engine::{
    board::{ItemKey, RoutingBoard},
    drc::NetIncompletes,
    ids::{FixedState, NetNo},
};
use fr_geom::{FloatPoint, IntPoint, Point, Polyline};
use serde::Serialize;
use std::collections::{BTreeMap, BTreeSet};

const MIN_GAIN_MM: f64 = 0.01;
const MAX_NET_SEGS: usize = 400;
const MAX_CHAIN_SEGS: usize = 100;
const TIE_TOL_MM: f64 = 1e-9;
const DEGENERATE_LEG_MM: f64 = 1e-5;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum BoardSmoothingStatus {
    NoLaneCopper,
    NoEligibleChains,
    Unchanged,
    Changed,
}

#[derive(Clone, Debug, Serialize)]
pub struct BoardSmoothingResult {
    pub status: BoardSmoothingStatus,
    pub nets_seen: usize,
    pub nets_changed: usize,
    pub chains_seen: usize,
    pub chains_rejected: usize,
    pub spans: usize,
    pub segments_removed: usize,
    pub segments_added: usize,
    pub saved_mm: f64,
}

impl Default for BoardSmoothingResult {
    fn default() -> Self {
        Self {
            status: BoardSmoothingStatus::NoLaneCopper,
            nets_seen: 0,
            nets_changed: 0,
            chains_seen: 0,
            chains_rejected: 0,
            spans: 0,
            segments_removed: 0,
            segments_added: 0,
            saved_mm: 0.0,
        }
    }
}

#[derive(Clone, Debug)]
struct TaggedPoint {
    p: FloatPoint,
    original: Option<Point>,
    original_index: Option<usize>,
}

#[derive(Clone, Debug)]
struct Candidate {
    end: usize,
    points: Vec<FloatPoint>,
}

fn dist(a: FloatPoint, b: FloatPoint) -> f64 {
    (a.x - b.x).hypot(a.y - b.y)
}

fn segment_hits_dense_field(a: FloatPoint, b: FloatPoint, bounds: &[[f64; 4]]) -> bool {
    bounds.iter().any(|field| {
        a.x.min(b.x) <= field[2]
            && a.x.max(b.x) >= field[0]
            && a.y.min(b.y) <= field[3]
            && a.y.max(b.y) >= field[1]
    })
}

fn length(points: &[FloatPoint]) -> f64 {
    points.windows(2).map(|w| dist(w[0], w[1])).sum()
}

fn to_native_point(p: FloatPoint) -> Point {
    Point::Int(IntPoint::new(p.x.round() as i32, p.y.round() as i32))
}

fn native_polyline(points: &[FloatPoint], source: &[Point]) -> Option<Polyline> {
    if points.len() < 2 || source.len() != points.len() {
        return None;
    }
    let native: Vec<Point> = points
        .iter()
        .zip(source)
        .map(|(p, original)| {
            let old = original.to_float();
            if p.x == old.x && p.y == old.y {
                original.clone()
            } else {
                to_native_point(*p)
            }
        })
        .collect();
    let polyline = Polyline::from_points(&native);
    (!polyline.is_empty()).then_some(polyline)
}

/// Equivalent candidate order and elbow geometry to `_octolinear_bends`.
fn octolinear_candidates(a: FloatPoint, b: FloatPoint, units_per_mm: f64) -> Vec<Vec<FloatPoint>> {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let (adx, ady) = (dx.abs(), dy.abs());
    let (sx, sy) = (
        if dx >= 0.0 { 1.0 } else { -1.0 },
        if dy >= 0.0 { 1.0 } else { -1.0 },
    );
    let mut out = Vec::with_capacity(3);
    if adx < 1e-9 * units_per_mm
        || ady < 1e-9 * units_per_mm
        || (adx - ady).abs() < 1e-6 * units_per_mm
    {
        out.push(vec![a, b]);
    }
    if adx >= ady {
        out.push(vec![a, FloatPoint::new(a.x + sx * ady, b.y), b]);
        out.push(vec![a, FloatPoint::new(b.x - sx * ady, a.y), b]);
    } else {
        out.push(vec![a, FloatPoint::new(b.x, a.y + sy * adx), b]);
        out.push(vec![a, FloatPoint::new(a.x, b.y - sy * adx), b]);
    }
    out
}

fn candidate_clear(
    board: &mut RoutingBoard,
    points: &[FloatPoint],
    source_start: &Point,
    source_end: &Point,
    chain: &SmoothingChain,
) -> bool {
    if points.len() < 2 {
        return false;
    }
    let mut source = Vec::with_capacity(points.len());
    source.push(source_start.clone());
    source.extend(
        points
            .iter()
            .skip(1)
            .take(points.len() - 2)
            .map(|p| to_native_point(*p)),
    );
    source.push(source_end.clone());
    let Some(polyline) = native_polyline(points, &source) else {
        return false;
    };
    board.check_polyline_trace(
        &polyline,
        chain.layer,
        chain.half_width,
        &[chain.net],
        chain.clearance_class,
    )
}

fn span_free(tagged: &[TaggedPoint], i: usize, j: usize, touches: &[SmoothingTouch]) -> bool {
    let left = tagged[..=i].iter().rev().find_map(|p| p.original_index);
    let right = tagged[j..].iter().find_map(|p| p.original_index);
    let (Some(a), Some(b)) = (left, right) else {
        return false;
    };
    let (lo, hi) = (a.min(b), a.max(b));
    touches
        .iter()
        .filter(|touch| touch.leg_index >= lo && touch.leg_index < hi)
        .all(|touch| {
            let near_start = dist(touch.point, tagged[i].p);
            let near_end = dist(touch.point, tagged[j].p);
            match touch.kind {
                // The native model does not expose the source's exact point-to-pad
                // distance helper here. Only retain a pad contact at the same
                // native endpoint; otherwise reject rather than guess that a
                // bounding-circle contact preserves the terminal.
                SmoothingTouchKind::Pad => near_start < 1e-6 || near_end < 1e-6,
                _ => near_start < touch.reach || near_end < touch.reach,
            }
        })
}

fn collapse(
    board: &mut RoutingBoard,
    tagged: &[TaggedPoint],
    chain: &SmoothingChain,
    min_gain: f64,
    allow_ties: bool,
    units_per_mm: f64,
) -> BTreeMap<usize, Candidate> {
    let m = tagged.len().saturating_sub(1);
    let mut cumulative = vec![0.0; tagged.len()];
    for i in 0..m {
        cumulative[i + 1] = cumulative[i] + dist(tagged[i].p, tagged[i + 1].p);
    }
    let mut result = BTreeMap::new();
    let mut i = 0;
    while i < m {
        let mut accepted = None;
        for j in ((i + 1)..=m).rev() {
            let old_len = cumulative[j] - cumulative[i];
            if !allow_ties && old_len <= min_gain {
                break;
            }
            if !span_free(tagged, i, j, &chain.touches) {
                continue;
            }
            let a = tagged[i].p;
            let b = tagged[j].p;
            for points in octolinear_candidates(a, b, units_per_mm) {
                let new_len = length(&points);
                let emitted_legs = points
                    .windows(2)
                    .filter(|w| dist(w[0], w[1]) > DEGENERATE_LEG_MM * units_per_mm)
                    .count();
                let is_tie = allow_ties
                    && (new_len - old_len).abs() <= TIE_TOL_MM * units_per_mm
                    && emitted_legs < j - i;
                if new_len > old_len - min_gain && !is_tie {
                    continue;
                }
                if is_tie
                    && points
                        .windows(2)
                        .any(|w| segment_hits_dense_field(w[0], w[1], &chain.dense_bounds))
                {
                    continue;
                }
                let mut emitted = vec![a];
                emitted.extend(points[1..points.len() - 1].iter().copied().filter(|bend| {
                    dist(a, *bend) > DEGENERATE_LEG_MM * units_per_mm
                        && dist(*bend, b) > DEGENERATE_LEG_MM * units_per_mm
                }));
                emitted.push(b);
                let source_start = tagged[i]
                    .original
                    .clone()
                    .unwrap_or_else(|| to_native_point(a));
                let source_end = tagged[j]
                    .original
                    .clone()
                    .unwrap_or_else(|| to_native_point(b));
                if !candidate_clear(board, &emitted, &source_start, &source_end, chain) {
                    continue;
                }
                accepted = Some(Candidate {
                    end: j,
                    points: emitted,
                });
                break;
            }
            if accepted.is_some() {
                break;
            }
        }
        if let Some(candidate) = accepted {
            let end = candidate.end;
            result.insert(i, candidate);
            i = end;
        } else {
            i += 1;
        }
    }
    result
}

fn apply(tagged: &[TaggedPoint], spans: &BTreeMap<usize, Candidate>) -> Vec<TaggedPoint> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < tagged.len() {
        if let Some(span) = spans.get(&i) {
            out.push(tagged[i].clone());
            for p in span
                .points
                .iter()
                .skip(1)
                .take(span.points.len().saturating_sub(2))
            {
                out.push(TaggedPoint {
                    p: *p,
                    original: None,
                    original_index: None,
                });
            }
            i = span.end;
        } else {
            out.push(tagged[i].clone());
            i += 1;
        }
    }
    out
}

fn collapsed_span_count(tagged: &[TaggedPoint]) -> usize {
    let mut spans = 0;
    let mut i = 0;
    while i + 1 < tagged.len() {
        let Some(a) = tagged[i].original_index else {
            i += 1;
            continue;
        };
        let mut j = i + 1;
        while j < tagged.len() && tagged[j].original_index.is_none() {
            j += 1;
        }
        if j == tagged.len() {
            break;
        }
        let b = tagged[j].original_index.unwrap();
        if j != i + 1 || b != a + 1 {
            spans += 1;
        }
        i = j;
    }
    spans
}

fn connectivity_score(board: &RoutingBoard, net: NetNo) -> (i32, i32) {
    let items = board.get_connectable_items(net);
    let grade = NetIncompletes::new(&board.basic, net, &items);
    (grade.count(), grade.connected_group_count())
}

fn net_segment_count(board: &RoutingBoard, net: NetNo) -> usize {
    board
        .get_traces()
        .into_iter()
        .filter_map(|key| {
            let item = board.item(key);
            item.net_numbers()
                .contains(&net)
                .then_some(item.trace().polyline().corner_count().max(0) as usize)
        })
        .sum()
}

fn complete_owner_set(
    board: &RoutingBoard,
    chain: &SmoothingChain,
) -> Option<(Vec<ItemKey>, FixedState)> {
    if chain.legs.is_empty()
        || chain.legs.len() > MAX_CHAIN_SEGS
        || chain.legs.iter().any(|leg| !leg.selected)
    {
        return None;
    }
    let mut expected = BTreeMap::<ItemKey, (usize, FixedState)>::new();
    let mut observed = BTreeMap::<ItemKey, usize>::new();
    for leg in &chain.legs {
        let item = board.item(leg.owner);
        if !item.is_on_board() || !item.is_trace() || !item.nets_normal() {
            return None;
        }
        let entry = expected.entry(leg.owner).or_insert((
            item.trace().polyline().corner_count().max(0) as usize,
            item.fixed_state(),
        ));
        if item.fixed_state() != entry.1 {
            return None;
        }
        *observed.entry(leg.owner).or_default() += 1;
    }
    if expected
        .iter()
        .any(|(key, (count, _))| observed.get(key) != Some(count))
    {
        return None;
    }
    let fixed = expected.values().next()?.1;
    if expected.values().any(|(_, state)| *state != fixed) || fixed == FixedState::SystemFixed {
        return None;
    }
    Some((expected.keys().copied().collect(), fixed))
}

/// Applies the public AWX board smoother to caller-selected lane trace items.
/// The controls remain the source defaults; the script exposes only its input
/// and output PCB paths, fanout PCB and net names.
pub fn smooth_board(
    board: &mut BoardAdapter,
    lane_items: &[ItemKey],
    dense_fields: &[DensePadField],
) -> Result<BoardSmoothingResult, String> {
    let mut result = BoardSmoothingResult::default();
    let units_per_mm = board.units_per_mm;
    if !units_per_mm.is_finite() || units_per_mm <= 0.0 {
        return Err("invalid board coordinate scale for AWX smoothing".into());
    }
    if lane_items.is_empty() {
        return Ok(result);
    }
    let chains = extract_smoothing_chains(&board.board, lane_items, units_per_mm, dense_fields);
    if chains.is_empty() {
        result.status = BoardSmoothingStatus::NoEligibleChains;
        return Ok(result);
    }
    result.chains_seen = chains.len();
    let seen_nets: BTreeSet<_> = chains.iter().map(|chain| chain.net).collect();
    result.nets_seen = seen_nets.len();
    let min_gain = MIN_GAIN_MM * units_per_mm;
    let mut changed_nets = BTreeSet::new();
    let mut replaced_owners = BTreeSet::new();

    for chain in chains {
        if net_segment_count(&board.board, chain.net) > MAX_NET_SEGS {
            result.chains_rejected += 1;
            continue;
        }
        let Some((owners, fixed_state)) = complete_owner_set(&board.board, &chain) else {
            result.chains_rejected += 1;
            continue;
        };
        if owners.iter().any(|owner| replaced_owners.contains(owner)) {
            result.chains_rejected += 1;
            continue;
        }
        let source: Vec<Point> = std::iter::once(chain.legs[0].a.clone())
            .chain(chain.legs.iter().map(|leg| leg.b.clone()))
            .collect();
        if source.iter().any(|point| !point.is_int_point()) {
            result.chains_rejected += 1;
            continue;
        }
        let points: Vec<FloatPoint> = source.iter().map(Point::to_float).collect();
        let mut tagged: Vec<TaggedPoint> = points
            .iter()
            .zip(&source)
            .enumerate()
            .map(|(index, (p, original))| TaggedPoint {
                p: *p,
                original: Some(original.clone()),
                original_index: Some(index),
            })
            .collect();
        let first = collapse(
            &mut board.board,
            &tagged,
            &chain,
            min_gain,
            false,
            units_per_mm,
        );
        tagged = apply(&tagged, &first);
        let second = collapse(
            &mut board.board,
            &tagged,
            &chain,
            min_gain,
            true,
            units_per_mm,
        );
        tagged = apply(&tagged, &second);
        let points_after: Vec<FloatPoint> = tagged.iter().map(|point| point.p).collect();
        if points_after == points {
            continue;
        }
        let source_after: Vec<Point> = tagged
            .iter()
            .map(|point| {
                point
                    .original
                    .clone()
                    .unwrap_or_else(|| to_native_point(point.p))
            })
            .collect();
        let Some(polyline) = native_polyline(&points_after, &source_after) else {
            result.chains_rejected += 1;
            continue;
        };
        if polyline.corner_count() as usize >= points.len()
            || !board.board.check_polyline_trace(
                &polyline,
                chain.layer,
                chain.half_width,
                &[chain.net],
                chain.clearance_class,
            )
        {
            result.chains_rejected += 1;
            continue;
        }

        let before = connectivity_score(&board.board, chain.net);
        let mut trial = board.board.deep_copy();
        for owner in &owners {
            trial.remove_item(*owner);
        }
        if !trial.check_polyline_trace(
            &polyline,
            chain.layer,
            chain.half_width,
            &[chain.net],
            chain.clearance_class,
        ) {
            result.chains_rejected += 1;
            continue;
        }
        if trial
            .insert_trace_without_cleaning(
                polyline.clone(),
                chain.layer,
                chain.half_width,
                &[chain.net],
                chain.clearance_class,
                fixed_state,
            )
            .is_none()
        {
            result.chains_rejected += 1;
            continue;
        }
        let after = connectivity_score(&trial, chain.net);
        if after.0 > before.0 || after.1 > before.1 {
            result.chains_rejected += 1;
            continue;
        }

        let old_len: f64 = points.windows(2).map(|w| dist(w[0], w[1])).sum();
        let new_len = polyline.length_approx();
        for owner in &owners {
            board.board.remove_item(*owner);
            replaced_owners.insert(*owner);
        }
        if board
            .board
            .insert_trace_without_cleaning(
                polyline.clone(),
                chain.layer,
                chain.half_width,
                &[chain.net],
                chain.clearance_class,
                fixed_state,
            )
            .is_none()
        {
            return Err("AWX smoother native insertion failed after validated trial".into());
        }
        changed_nets.insert(chain.net);
        result.spans += collapsed_span_count(&tagged);
        result.segments_removed += chain.legs.len();
        result.segments_added += polyline.corner_count().max(0) as usize;
        result.saved_mm += ((old_len - new_len) / units_per_mm).max(0.0);
    }
    result.nets_changed = changed_nets.len();
    result.saved_mm = (result.saved_mm * 10000.0).round() / 10000.0;
    result.status = if result.nets_changed > 0 {
        BoardSmoothingStatus::Changed
    } else {
        BoardSmoothingStatus::Unchanged
    };
    Ok(result)
}
