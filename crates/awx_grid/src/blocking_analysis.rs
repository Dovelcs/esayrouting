//! Frontier attribution from `py_router/blocking_analysis.py` (caad45cd).
use crate::soft_copper::to_grid;
use std::collections::{BTreeMap, BTreeSet};

pub type Cell = (i32, i32, usize);

/// The caller supplies the original per-net/per-layer resolved keep-out.
pub struct TrackObstacle {
    pub start: [f64; 2],
    pub end: [f64; 2],
    pub layer: usize,
    pub margin: f64,
}

pub struct ViaObstacle {
    pub point: [f64; 2],
    pub margin: f64,
}

pub struct NetObstacles {
    pub net_id: i32,
    pub net_name: String,
    pub tracks: Vec<TrackObstacle>,
    pub vias: Vec<ViaObstacle>,
}

#[derive(Clone, Debug)]
pub struct BlockingInfo {
    pub net_id: i32,
    pub net_name: String,
    pub blocked_count: usize,
    pub track_cells: usize,
    pub via_cells: usize,
    pub unique_cells: usize,
    pub near_target_cells: usize,
    pub near_source_cells: usize,
    pub validator_named: bool,
}

pub struct BlockingReport {
    pub entries: Vec<BlockingInfo>,
    pub frontier_cells: usize,
    pub attributed_cells: usize,
    /// Source history-congestion hooks consume these exact deduplicated cells.
    pub frontier: BTreeSet<Cell>,
    pub contested: BTreeSet<Cell>,
}

fn in_capsule(p: [f64; 2], a: [f64; 2], b: [f64; 2], margin: f64) -> bool {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let len_sq = dx * dx + dy * dy;
    let t = if len_sq == 0.0 {
        0.0
    } else {
        (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len_sq).clamp(0.0, 1.0)
    };
    let rx = p[0] - (a[0] + t * dx);
    let ry = p[1] - (a[1] + t * dy);
    let effective_margin = margin - 1e-6;
    rx * rx + ry * ry < effective_margin * effective_margin
}

/// Candidate order is preserved for ties, as with Python's stable sort.
pub fn analyze_frontier_blocking(
    blocked: &[Cell],
    candidates: &[NetObstacles],
    excluded: &BTreeSet<i32>,
    grid_step: f64,
    layer_count: usize,
    target: Option<[f64; 2]>,
    source: Option<[f64; 2]>,
) -> BlockingReport {
    let frontier: BTreeSet<_> = blocked.iter().copied().collect();
    let mut ownership = BTreeMap::<Cell, usize>::new();
    let mut rows = Vec::new();
    for net in candidates {
        if excluded.contains(&net.net_id) {
            continue;
        }
        let mut track = BTreeSet::new();
        let mut via = BTreeSet::new();
        for &(x, y, layer) in &frontier {
            let p = [x as f64 * grid_step, y as f64 * grid_step];
            if net
                .tracks
                .iter()
                .any(|s| s.layer == layer && in_capsule(p, s.start, s.end, s.margin))
            {
                track.insert((x, y, layer));
            }
            if layer < layer_count
                && net
                    .vias
                    .iter()
                    .any(|v| in_capsule(p, v.point, v.point, v.margin))
            {
                via.insert((x, y, layer));
            }
        }
        let total: BTreeSet<_> = track.union(&via).copied().collect();
        if total.is_empty() {
            continue;
        }
        for cell in &total {
            *ownership.entry(*cell).or_default() += 1;
        }
        rows.push((net, track, via, total));
    }
    let radius = (3.0 / grid_step) as i64;
    let near = |cells: &BTreeSet<Cell>, point: Option<[f64; 2]>| -> usize {
        point.map_or(0, |point| {
            let [px, py] = to_grid(point, grid_step);
            cells
                .iter()
                .filter(|&&(x, y, _)| {
                    let dx = i64::from(x) - i64::from(px);
                    let dy = i64::from(y) - i64::from(py);
                    dx * dx + dy * dy <= radius * radius
                })
                .count()
        })
    };
    let entries = rows
        .into_iter()
        .map(|(net, track, via, total)| BlockingInfo {
            net_id: net.net_id,
            net_name: net.net_name.clone(),
            blocked_count: total.len(),
            track_cells: track.len(),
            via_cells: via.len(),
            unique_cells: total
                .iter()
                .filter(|cell| ownership.get(cell) == Some(&1))
                .count(),
            near_target_cells: near(&total, target),
            near_source_cells: near(&total, source),
            validator_named: false,
        })
        .collect();
    BlockingReport {
        entries,
        frontier_cells: frontier.len(),
        attributed_cells: ownership.len(),
        frontier,
        contested: ownership.into_keys().collect(),
    }
}

pub fn apply_known_blockers(
    results: &mut Vec<BlockingInfo>,
    known: &[(i32, String, usize)],
    excluded: &BTreeSet<i32>,
) {
    let by_id: BTreeMap<_, _> = results
        .iter()
        .enumerate()
        .map(|(index, row)| (row.net_id, index))
        .collect();
    for (id, name, count) in known {
        if excluded.contains(id) {
            continue;
        }
        if let Some(&index) = by_id.get(id) {
            results[index].validator_named = true;
        } else {
            results.push(BlockingInfo {
                net_id: *id,
                net_name: name.clone(),
                blocked_count: *count,
                track_cells: *count,
                via_cells: 0,
                unique_cells: *count,
                near_target_cells: *count,
                near_source_cells: 0,
                validator_named: true,
            });
        }
    }
}

pub enum Ranking<'a> {
    Count,
    NearTarget,
    Bidirectional(&'a BTreeSet<i32>),
    Cost(&'a BTreeMap<i32, f64>),
}

pub fn rank_blockers(
    rows: &mut [BlockingInfo],
    ranking: Ranking<'_>,
    bus_members: &BTreeSet<i32>,
    bus_resistance: f64,
) {
    let key = |row: &BlockingInfo| {
        let named = if row.validator_named { 1.0 } else { 0.0 };
        let near = (row.near_target_cells + row.near_source_cells) as f64;
        let unique = row.unique_cells as f64;
        let r = if bus_resistance > 1.0 && bus_members.contains(&row.net_id) && !row.validator_named
        {
            bus_resistance
        } else {
            1.0
        };
        if matches!(&ranking, Ranking::NearTarget) {
            return [
                named,
                near.min(unique) / r,
                near / r,
                unique / r,
                row.blocked_count as f64 / r,
            ];
        }
        let (tier, score) = if row.blocked_count > 0 && row.unique_cells == row.blocked_count {
            (2.0, unique)
        } else {
            (
                1.0,
                unique + near.min(unique) + 0.5 * (row.blocked_count - row.unique_cells) as f64,
            )
        };
        match &ranking {
            Ranking::Bidirectional(both) => [
                named,
                tier,
                score * if both.contains(&row.net_id) { 2.0 } else { 1.0 } / r,
                near / r,
                0.0,
            ],
            Ranking::Cost(costs) => {
                let c = costs.get(&row.net_id).copied().unwrap_or(1.0).max(1.0);
                [named, tier, score / (r * c), near / (r * c), 0.0]
            }
            _ => [named, tier, score / r, near / r, 0.0],
        }
    };
    rows.sort_by(|a, b| {
        let a = key(a);
        let b = key(b);
        a.iter()
            .zip(&b)
            .map(|(a, b)| b.total_cmp(a))
            .find(|order| !order.is_eq())
            .unwrap_or(std::cmp::Ordering::Equal)
    });
}
