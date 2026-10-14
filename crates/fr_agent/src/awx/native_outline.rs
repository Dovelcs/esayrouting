//! Source polygon-board outline, cutout, and milled-contour grid obstacles.
//!
//! Board rings are retained by `BoardAdapter` in native board units. This
//! module stamps their real polygon geometry; the board bounding box is used
//! only to bound the scan, never as a replacement for a polygon outline.
use crate::{
    awx::native_obstacles::{grid_round, NativeGridConfig},
    board_adapter::BoardAdapter,
};
use awx_grid::obstacle_map::GridObstacleMap;
use fr_engine::ids::LayerNo;
use fr_geom::{IntBox, IntPoint, Point};
use std::collections::{BTreeMap, BTreeSet};

const GRID_TIE_EPS_MM: f64 = 1.0e-6;

#[derive(Clone, Copy)]
struct EdgeClearance {
    track: f64,
    via: f64,
    tie_eps: f64,
}

fn edge_clearance(board: &BoardAdapter, config: &NativeGridConfig) -> EdgeClearance {
    let base = (if config.board_edge_clearance > 0 {
        config.board_edge_clearance
    } else {
        config.clearance
    }) as f64;
    EdgeClearance {
        track: base + config.track_width as f64 / 2.0,
        via: base + config.via_size as f64 / 2.0,
        tie_eps: GRID_TIE_EPS_MM * board.units_per_mm,
    }
}

fn grid_coord(value: f64, step: i32) -> i32 {
    (value / step as f64).round_ties_even() as i32
}

fn grid_dist_floor(distance: f64, step: i32) -> i32 {
    (distance / step as f64).trunc() as i32
}

fn grid_dist_ceil(distance: f64, step: i32) -> i32 {
    (distance / step as f64).ceil() as i32
}

fn map_cell(window: &IntBox, gx: i32, gy: i32, step: i32) -> bool {
    let x = gx.saturating_mul(step);
    let y = gy.saturating_mul(step);
    window.ll.x <= x && x <= window.ur.x && window.ll.y <= y && y <= window.ur.y
}

fn point_inside(point: [f64; 2], polygon: &[[i32; 2]]) -> bool {
    if polygon.len() < 3 {
        return false;
    }
    let mut inside = false;
    for i in 0..polygon.len() {
        let a = polygon[i];
        let b = polygon[(i + 1) % polygon.len()];
        let (x1, y1, x2, y2) = (a[0] as f64, a[1] as f64, b[0] as f64, b[1] as f64);
        if (y1 > point[1]) != (y2 > point[1]) {
            let safe_dy = if y2 == y1 { 1.0 } else { y2 - y1 };
            let x_intercept = (x2 - x1) * (point[1] - y1) / safe_dy + x1;
            if point[0] < x_intercept {
                inside = !inside;
            }
        }
    }
    inside
}

fn point_segment_distance(point: [f64; 2], a: [i32; 2], b: [i32; 2]) -> f64 {
    let (x1, y1) = (a[0] as f64, a[1] as f64);
    let (dx, dy) = ((b[0] - a[0]) as f64, (b[1] - a[1]) as f64);
    let len_sq = dx * dx + dy * dy;
    if len_sq == 0.0 {
        return (point[0] - x1).hypot(point[1] - y1);
    }
    let t = (((point[0] - x1) * dx + (point[1] - y1) * dy) / len_sq).clamp(0.0, 1.0);
    (point[0] - (x1 + t * dx)).hypot(point[1] - (y1 + t * dy))
}

type PadLayerCells = BTreeMap<usize, BTreeSet<(i32, i32)>>;

#[derive(Clone, Copy)]
struct PolygonEdge {
    a: [i32; 2],
    b: [i32; 2],
    min_x: f64,
    max_x: f64,
    min_y: f64,
    max_y: f64,
}

fn ring_edges(ring: &[[i32; 2]]) -> Vec<PolygonEdge> {
    (0..ring.len())
        .map(|i| {
            let a = ring[i];
            let b = ring[(i + 1) % ring.len()];
            PolygonEdge {
                a,
                b,
                min_x: a[0].min(b[0]) as f64,
                max_x: a[0].max(b[0]) as f64,
                min_y: a[1].min(b[1]) as f64,
                max_y: a[1].max(b[1]) as f64,
            }
        })
        .collect()
}

fn inside_on_scanline(edges: &[PolygonEdge], x_cells: &[i32], y: f64, step: i32) -> Vec<bool> {
    let mut crossings: Vec<_> = edges
        .iter()
        .filter_map(|edge| {
            let y1 = edge.a[1] as f64;
            let y2 = edge.b[1] as f64;
            ((y1 > y) != (y2 > y)).then(|| {
                let safe_dy = if y2 == y1 { 1.0 } else { y2 - y1 };
                (edge.b[0] as f64 - edge.a[0] as f64) * (y - y1) / safe_dy + edge.a[0] as f64
            })
        })
        .collect();
    crossings.sort_by(f64::total_cmp);
    x_cells
        .iter()
        .map(|gx| {
            let x = gx.saturating_mul(step) as f64;
            let passed = crossings.partition_point(|crossing| *crossing <= x);
            (crossings.len() - passed) % 2 == 1
        })
        .collect()
}

fn edge_distance_nearby(point: [f64; 2], y_candidates: &[PolygonEdge], threshold: f64) -> f64 {
    y_candidates
        .iter()
        .filter(|edge| point[0] >= edge.min_x - threshold && point[0] <= edge.max_x + threshold)
        .map(|edge| point_segment_distance(point, edge.a, edge.b))
        .fold(f64::INFINITY, f64::min)
}

/// Per-layer on-board pad-copper cells exempted from the outer-edge track band
/// as in obstacle_map.py's self-net-scoped edge-pad rule (#441).
fn edge_pad_exemptions(
    board: &BoardAdapter,
    config: &NativeGridConfig,
    bounds: &IntBox,
    rings: &[Vec<[i32; 2]>],
    window: &IntBox,
    edge_clearance: f64,
) -> Result<PadLayerCells, String> {
    let step = config.grid_step;
    let route_index: BTreeMap<i32, usize> = config
        .layer_numbers
        .iter()
        .enumerate()
        .map(|(index, &layer)| (layer as LayerNo, index))
        .collect();
    let mut exempt = PadLayerCells::new();
    for key in board.board.get_pins() {
        let item = board.board.item(key);
        let pin = item
            .as_pin()
            .ok_or("board pin list contains a non-pin outline item")?;
        let component = board.board.components.get(item.component_no());
        let package = board.board.library.packages.get(component.get_package());
        let package_pin = package
            .get_pin(pin.pin_index)
            .ok_or("edge-pad exemption cannot resolve a native package pin")?;
        let padstack = board
            .board
            .library
            .padstacks
            .get(package_pin.padstack_id)
            .ok_or("edge-pad exemption cannot resolve a native padstack")?;
        if padstack.hole_only {
            continue;
        }
        let shapes = board.board.item_tile_shapes(item);
        for (shape_index, maybe_shape) in shapes.iter().enumerate() {
            let Some(shape) = maybe_shape else { continue };
            let layer = board.board.shape_layer(key, shape_index as i32);
            let Some(&route_layer) = route_index.get(&layer) else {
                continue;
            };
            let shape_bounds = shape.bounding_box();
            let reaches_edge = (shape_bounds.ll.x as f64) < bounds.ll.x as f64 + edge_clearance
                || (shape_bounds.ur.x as f64) > bounds.ur.x as f64 - edge_clearance
                || (shape_bounds.ll.y as f64) < bounds.ll.y as f64 + edge_clearance
                || (shape_bounds.ur.y as f64) > bounds.ur.y as f64 - edge_clearance;
            if !reaches_edge {
                continue;
            }
            let gx0 = grid_round(shape_bounds.ll.x, step).saturating_sub(1);
            let gy0 = grid_round(shape_bounds.ll.y, step).saturating_sub(1);
            let gx1 = grid_round(shape_bounds.ur.x, step).saturating_add(1);
            let gy1 = grid_round(shape_bounds.ur.y, step).saturating_add(1);
            for gx in gx0..=gx1 {
                for gy in gy0..=gy1 {
                    if !map_cell(window, gx, gy, step) {
                        continue;
                    }
                    let x = gx.saturating_mul(step);
                    let y = gy.saturating_mul(step);
                    let center = Point::Int(IntPoint::new(x, y));
                    if !shape.contains(&center) {
                        continue;
                    }
                    let on_board = if rings.is_empty() {
                        bounds.ll.x <= x && x <= bounds.ur.x && bounds.ll.y <= y && y <= bounds.ur.y
                    } else {
                        rings
                            .iter()
                            .any(|ring| point_inside([x as f64, y as f64], ring))
                    };
                    if on_board {
                        exempt.entry(route_layer).or_default().insert((gx, gy));
                    }
                }
            }
        }
    }
    Ok(exempt)
}

fn all_outer_rings(board: &BoardAdapter) -> Vec<Vec<[i32; 2]>> {
    board
        .board_outline_polygons
        .iter()
        .filter(|ring| ring.len() >= 3)
        .cloned()
        .collect()
}

fn stamp_outside_and_edge(
    obstacles: &mut GridObstacleMap,
    window: &IntBox,
    config: &NativeGridConfig,
    bounds: &IntBox,
    rings: &[Vec<[i32; 2]>],
    clear: EdgeClearance,
    exemptions: &PadLayerCells,
) {
    let step = config.grid_step;
    let gmin_x = grid_round(bounds.ll.x, step);
    let gmin_y = grid_round(bounds.ll.y, step);
    let gmax_x = grid_round(bounds.ur.x, step);
    let gmax_y = grid_round(bounds.ur.y, step);
    let track_expand = grid_dist_floor(clear.track, step);
    let via_expand = grid_dist_ceil(clear.via, step);
    let margin = track_expand.max(via_expand).saturating_add(5);
    let layer_count = config.route_layers.len();
    let gx_cells: Vec<_> =
        (gmin_x.saturating_sub(margin)..=gmax_x.saturating_add(margin)).collect();
    let edge_sets: Vec<_> = rings.iter().map(|ring| ring_edges(ring)).collect();
    let all_edges: Vec<_> = edge_sets.iter().flatten().copied().collect();
    let distance_threshold = clear.track.max(clear.via) + step as f64;

    for gy in gmin_y.saturating_sub(margin)..=gmax_y.saturating_add(margin) {
        let y = gy.saturating_mul(step) as f64;
        let mut inside = vec![false; gx_cells.len()];
        for edges in &edge_sets {
            let ring_inside = inside_on_scanline(edges, &gx_cells, y, step);
            for (dst, src) in inside.iter_mut().zip(ring_inside) {
                *dst |= src;
            }
        }
        let y_candidates: Vec<_> = all_edges
            .iter()
            .filter(|edge| {
                y >= edge.min_y - distance_threshold && y <= edge.max_y + distance_threshold
            })
            .copied()
            .collect();
        for (ix, &gx) in gx_cells.iter().enumerate() {
            if !map_cell(window, gx, gy, step) {
                continue;
            }
            let center = [gx.saturating_mul(step) as f64, y];
            if !inside[ix] {
                for layer in 0..layer_count {
                    obstacles.add_static_blocked_cell(gx, gy, layer);
                }
                obstacles.add_static_blocked_via(gx, gy);
                continue;
            }
            let edge_distance = edge_distance_nearby(center, &y_candidates, distance_threshold);
            if edge_distance < clear.track {
                for layer in 0..layer_count {
                    if !exemptions
                        .get(&layer)
                        .is_some_and(|cells| cells.contains(&(gx, gy)))
                    {
                        obstacles.add_static_blocked_cell(gx, gy, layer);
                    }
                }
            }
            if edge_distance < clear.via {
                obstacles.add_static_blocked_via(gx, gy);
            }
        }
    }
}

fn stamp_rectangular_edges(
    obstacles: &mut GridObstacleMap,
    window: &IntBox,
    config: &NativeGridConfig,
    bounds: &IntBox,
    clear: EdgeClearance,
    exemptions: &PadLayerCells,
) {
    let step = config.grid_step;
    let gmin_x = grid_round(bounds.ll.x, step);
    let gmin_y = grid_round(bounds.ll.y, step);
    let gmax_x = grid_round(bounds.ur.x, step);
    let gmax_y = grid_round(bounds.ur.y, step);
    let track = grid_dist_floor(clear.track, step);
    let via = grid_dist_ceil(clear.via, step);
    let margin = track.max(via).saturating_add(5);
    let layer_count = config.route_layers.len();
    for gx in gmin_x.saturating_sub(margin)..=gmax_x.saturating_add(margin) {
        for gy in gmin_y.saturating_sub(margin)..=gmax_y.saturating_add(margin) {
            if !map_cell(window, gx, gy, step) {
                continue;
            }
            if gx <= gmin_x + track
                || gx >= gmax_x - track
                || gy <= gmin_y + track
                || gy >= gmax_y - track
            {
                for layer in 0..layer_count {
                    if !exemptions
                        .get(&layer)
                        .is_some_and(|cells| cells.contains(&(gx, gy)))
                    {
                        obstacles.add_static_blocked_cell(gx, gy, layer);
                    }
                }
            }
            if gx < gmin_x + via || gx > gmax_x - via || gy < gmin_y + via || gy > gmax_y - via {
                obstacles.add_static_blocked_via(gx, gy);
            }
        }
    }
}

fn stamp_cutout(
    obstacles: &mut GridObstacleMap,
    window: &IntBox,
    config: &NativeGridConfig,
    polygon: &[[i32; 2]],
    clear: EdgeClearance,
) {
    if polygon.len() < 3 {
        return;
    }
    let step = config.grid_step;
    let margin = clear.track.max(clear.via) + step as f64;
    let min_x = polygon.iter().map(|p| p[0]).min().unwrap() as f64 - margin;
    let max_x = polygon.iter().map(|p| p[0]).max().unwrap() as f64 + margin;
    let min_y = polygon.iter().map(|p| p[1]).min().unwrap() as f64 - margin;
    let max_y = polygon.iter().map(|p| p[1]).max().unwrap() as f64 + margin;
    let gx0 = grid_coord(min_x, step);
    let gx1 = grid_coord(max_x, step);
    let gy0 = grid_coord(min_y, step);
    let gy1 = grid_coord(max_y, step);
    let gx_cells: Vec<_> = (gx0..=gx1).collect();
    let edges = ring_edges(polygon);
    let distance_threshold = clear.track.max(clear.via) + step as f64;
    for gy in gy0..=gy1 {
        let y = gy.saturating_mul(step) as f64;
        let inside = inside_on_scanline(&edges, &gx_cells, y, step);
        let y_candidates: Vec<_> = edges
            .iter()
            .filter(|edge| {
                y >= edge.min_y - distance_threshold && y <= edge.max_y + distance_threshold
            })
            .copied()
            .collect();
        for (ix, &gx) in gx_cells.iter().enumerate() {
            if !map_cell(window, gx, gy, step) {
                continue;
            }
            let center = [gx.saturating_mul(step) as f64, y];
            let edge = edge_distance_nearby(center, &y_candidates, distance_threshold);
            if inside[ix] || edge < clear.track - clear.tie_eps {
                for layer in 0..config.route_layers.len() {
                    obstacles.add_static_blocked_cell(gx, gy, layer);
                }
            }
            if inside[ix] || edge < clear.via - clear.tie_eps {
                obstacles.add_static_blocked_via(gx, gy);
            }
        }
    }
}

fn stamp_edge_contour(
    obstacles: &mut GridObstacleMap,
    window: &IntBox,
    config: &NativeGridConfig,
    contour: &[[i32; 2]],
    clear: EdgeClearance,
) {
    if contour.len() < 3 {
        return;
    }
    let step = config.grid_step;
    let margin = clear.track.max(clear.via) + step as f64;
    let min_x = contour.iter().map(|p| p[0]).min().unwrap() as f64 - margin;
    let max_x = contour.iter().map(|p| p[0]).max().unwrap() as f64 + margin;
    let min_y = contour.iter().map(|p| p[1]).min().unwrap() as f64 - margin;
    let max_y = contour.iter().map(|p| p[1]).max().unwrap() as f64 + margin;
    let gx0 = grid_coord(min_x, step);
    let gx1 = grid_coord(max_x, step);
    let gy0 = grid_coord(min_y, step);
    let gy1 = grid_coord(max_y, step);
    let gx_cells: Vec<_> = (gx0..=gx1).collect();
    let edges = ring_edges(contour);
    let distance_threshold = clear.track.max(clear.via) + step as f64;
    for gy in gy0..=gy1 {
        let y = gy.saturating_mul(step) as f64;
        let y_candidates: Vec<_> = edges
            .iter()
            .filter(|edge| {
                y >= edge.min_y - distance_threshold && y <= edge.max_y + distance_threshold
            })
            .copied()
            .collect();
        for &gx in &gx_cells {
            if !map_cell(window, gx, gy, step) {
                continue;
            }
            let center = [gx.saturating_mul(step) as f64, y];
            let edge = edge_distance_nearby(center, &y_candidates, distance_threshold);
            if edge < clear.track - clear.tie_eps {
                for layer in 0..config.route_layers.len() {
                    obstacles.add_static_blocked_cell(gx, gy, layer);
                }
            }
            if edge < clear.via - clear.tie_eps {
                obstacles.add_static_blocked_via(gx, gy);
            }
        }
    }
}

/// Stamp actual board outline, cutout, and milled-contour geometry in global
/// grid coordinates. `window` clips materialization for a scoped native search.
pub fn stamp_outline_and_cutouts(
    obstacles: &mut GridObstacleMap,
    board: &BoardAdapter,
    window: IntBox,
    config: &NativeGridConfig,
) -> Result<(), String> {
    if config.grid_step <= 0 {
        return Err("native outline grid step must be positive".into());
    }
    if config.route_layers.is_empty() {
        return Err("native outline requires at least one route layer".into());
    }
    let bounds = board.board.bounding_box();
    if bounds.ll.x > bounds.ur.x || bounds.ll.y > bounds.ur.y {
        return Err("native outline board bounds are invalid".into());
    }
    let clear = edge_clearance(board, config);
    let outlines = all_outer_rings(board);
    let edge = if config.board_edge_clearance > 0 {
        config.board_edge_clearance as f64
    } else {
        config.clearance as f64
    };
    let exemptions = edge_pad_exemptions(board, config, &bounds, &outlines, &window, edge)?;
    if outlines.is_empty() {
        stamp_rectangular_edges(obstacles, &window, config, &bounds, clear, &exemptions);
    } else {
        stamp_outside_and_edge(
            obstacles,
            &window,
            config,
            &bounds,
            &outlines,
            clear,
            &exemptions,
        );
    }
    for cutout in &board.board_cutouts {
        stamp_cutout(obstacles, &window, config, cutout, clear);
    }
    for contour in &board.board_edge_contours {
        stamp_edge_contour(obstacles, &window, config, contour, clear);
    }
    Ok(())
}
