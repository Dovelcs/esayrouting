//! `awx/connect.py::_stamp_soft` at caad45cd, with Rust array interfaces.
use crate::GridObstacleMap;
use std::collections::BTreeMap;

#[derive(Clone, Debug)]
pub struct SoftTrack {
    pub start: [f64; 2],
    pub end: [f64; 2],
    pub layer: usize,
    pub width: f64,
}

#[derive(Clone, Debug)]
pub struct SoftVia {
    pub point: [f64; 2],
    pub size: f64,
}

pub fn to_grid(point: [f64; 2], grid_step: f64) -> [i32; 2] {
    let inv_step = 1.0 / grid_step;
    [
        (point[0] * inv_step).round_ties_even() as i32,
        (point[1] * inv_step).round_ties_even() as i32,
    ]
}

/// The source Bresenham walk includes both ends, with strict comparisons.
pub fn walk_line(a: [i32; 2], b: [i32; 2]) -> Vec<[i32; 2]> {
    let dx = (i64::from(b[0]) - i64::from(a[0])).abs();
    let dy = (i64::from(b[1]) - i64::from(a[1])).abs();
    let sx = if a[0] < b[0] { 1 } else { -1 };
    let sy = if a[1] < b[1] { 1 } else { -1 };
    let mut err = dx - dy;
    let mut p = a;
    let mut points = Vec::new();
    loop {
        points.push(p);
        if p == b {
            break;
        }
        let e2 = 2 * err;
        if e2 > -dy {
            err -= dy;
            p[0] += sx;
        }
        if e2 < dx {
            err += dx;
            p[1] += sy;
        }
    }
    points
}

/// The same per-column integer-disc union as `_walk_capsule_cells`.
pub fn walk_capsule_spans(points: &[[i32; 2]], radius: i32) -> Vec<(i32, i32, i32)> {
    let mut columns = BTreeMap::<i32, (i32, i32)>::new();
    for &[x, y] in points {
        let span = columns.entry(x).or_insert((y, y));
        span.0 = span.0.min(y);
        span.1 = span.1.max(y);
    }
    let mut spans = BTreeMap::<i32, (i32, i32)>::new();
    for (x, (lo, hi)) in columns {
        for ex in -radius..=radius {
            let h = ((i64::from(radius) * i64::from(radius)
                - i64::from(ex) * i64::from(ex)) as u64)
                .isqrt() as i32;
            let span = spans.entry(x + ex).or_insert((lo - h, hi + h));
            span.0 = span.0.min(lo - h);
            span.1 = span.1.max(hi + h);
        }
    }
    spans.into_iter().map(|(x, (lo, hi))| (x, lo, hi)).collect()
}

pub fn stamp_soft(
    obstacles: &mut GridObstacleMap,
    grid_step: f64,
    layer_count: usize,
    clearance: f64,
    track_width: f64,
    tracks: &[SoftTrack],
    vias: &[SoftVia],
    soft_cost: f64,
) {
    // GridRouteConfig.cell_cost: REFERENCE_GRID_STEP = 0.1 mm.
    let cost = (soft_cost * 1000.0 / 0.1) as i32;
    let inv_step = 1.0 / grid_step;
    for track in tracks {
        if track.layer >= layer_count {
            continue;
        }
        let radius = ((track.width / 2.0 + clearance + track_width / 2.0) * inv_step)
            as i32;
        let points = walk_line(to_grid(track.start, grid_step), to_grid(track.end, grid_step));
        for (x, lo, hi) in walk_capsule_spans(&points, radius) {
            for y in lo..=hi {
                obstacles.set_layer_proximity(x, y, track.layer, cost);
            }
        }
    }
    for via in vias {
        let radius = ((via.size / 2.0 + clearance + track_width / 2.0) * inv_step)
            as i32;
        let [x, y] = to_grid(via.point, grid_step);
        for layer in 0..layer_count {
            for ex in -radius..=radius {
                for ey in -radius..=radius {
                    if i64::from(ex) * i64::from(ex) + i64::from(ey) * i64::from(ey)
                        <= i64::from(radius) * i64::from(radius)
                    {
                        obstacles.set_layer_proximity(x + ex, y + ey, layer, cost);
                    }
                }
            }
        }
    }
}
