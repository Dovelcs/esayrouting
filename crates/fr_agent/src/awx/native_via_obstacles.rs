//! Source via-placement obstacle stamps for the native base-map builder.
//!
//! Foreign barrels block track cells on each route layer and via cells at the
//! copper-spacing and drill-to-drill thresholds. Foreign pad shapes also block
//! via placement using their actual native tile geometry. Per-net via sizes are
//! stamped into their own via-legality rungs.

use super::native_obstacles::{grid_round, NativeGridConfig};
use super::whole_context::{ContextPad, CopperVia};
use crate::board_adapter::BoardAdapter;
use awx_grid::obstacle_map::GridObstacleMap;
use fr_engine::{
    board::{ItemKey, BoardItemType},
    ids::{ClearanceClassNo, NetNo},
};
use fr_geom::{IntPoint, Point, TileShape};
use std::collections::BTreeSet;

const VIA_DIAGONAL_MARGIN_CELLS: f64 = 0.25;
const GRID_TIE_EPS_MM: f64 = 1e-6;

#[derive(Clone, Copy)]
struct ViaGeometry {
    center: [f64; 2],
    diameter: i32,
    drill: i32,
    class: ClearanceClassNo,
}

fn is_copper_layer(board: &BoardAdapter, name: &str) -> bool {
    // LayerStructure is the concrete DSN routing stack, including signal and
    // power layers. DSN names commonly use Top/Bottom; wildcards are not stack entries.
    !name.starts_with('*') && board.board.layer_structure.get_no(name) >= 0
}

fn resolved_clearance(
    board: &BoardAdapter,
    config: &NativeGridConfig,
    route_class: ClearanceClassNo,
    obstacle_class: ClearanceClassNo,
    layer: i32,
) -> i32 {
    board
        .board
        .clearance_value(route_class, obstacle_class, layer)
        .max(config.clearance)
        .max(0)
}

fn stack_clearance(
    board: &BoardAdapter,
    config: &NativeGridConfig,
    route_class: ClearanceClassNo,
    obstacle_class: ClearanceClassNo,
) -> i32 {
    board
        .board
        .layer_structure
        .layers
        .iter()
        .enumerate()
        .filter(|(_, layer)| is_copper_layer(board, &layer.name))
        .map(|(index, _)| {
            resolved_clearance(board, config, route_class, obstacle_class, index as i32)
        })
        .max()
        .unwrap_or(config.clearance)
}

fn via_class(board: &BoardAdapter, via: &CopperVia) -> Option<ClearanceClassNo> {
    let target = [
        via.point[0] * board.units_per_mm,
        via.point[1] * board.units_per_mm,
    ];
    board.board.get_vias().into_iter().find_map(|key| {
        let item = board.board.item(key);
        let center = item.center(&board.board).to_float();
        let same_net = item.net_numbers().iter().any(|&n| n as i32 == via.net_id);
        (same_net && (center.x - target[0]).abs() <= 1.0 && (center.y - target[1]).abs() <= 1.0)
            .then(|| item.clearance_class())
    })
}

fn via_geometry(board: &BoardAdapter, via: &CopperVia) -> Result<ViaGeometry, String> {
    Ok(ViaGeometry {
        center: [
            via.point[0] * board.units_per_mm,
            via.point[1] * board.units_per_mm,
        ],
        diameter: (via.diameter * board.units_per_mm).round().max(1.0) as i32,
        drill: (via.drill * board.units_per_mm).round().max(0.0) as i32,
        class: via_class(board, via).ok_or_else(|| {
            format!(
                "foreign via metadata has no native item at ({:.4},{:.4})",
                via.point[0], via.point[1]
            )
        })?,
    })
}

fn via_rung_geometries(config: &NativeGridConfig) -> Vec<(i32, i32)> {
    config.ordered_via_rungs.clone()
}

fn circle_cells(center: [i32; 2], radius_cells: f64) -> Vec<[i32; 2]> {
    if !radius_cells.is_finite() || radius_cells < 0.0 {
        return Vec::new();
    }
    let extent = radius_cells.ceil() as i32;
    let radius2 = radius_cells * radius_cells;
    let mut cells = Vec::new();
    for dx in -extent..=extent {
        for dy in -extent..=extent {
            let distance2 = (dx as f64) * (dx as f64) + (dy as f64) * (dy as f64);
            if distance2 <= radius2 {
                cells.push([center[0] + dx, center[1] + dy]);
            }
        }
    }
    cells
}

fn stamp_via_cells(map: &mut GridObstacleMap, cells: &[[i32; 2]], rung: usize, static_base: bool) {
    for &[gx, gy] in cells {
        if rung == 0 && static_base {
            map.add_static_blocked_via(gx, gy);
        } else if rung == 0 {
            map.add_blocked_via(gx, gy);
        } else {
            map.add_blocked_via_rung(rung, gx, gy);
        }
    }
}

fn stamp_track_cells(
    map: &mut GridObstacleMap,
    cells: &[[i32; 2]],
    layer: usize,
    static_base: bool,
) {
    for &[gx, gy] in cells {
        if static_base {
            map.add_static_blocked_cell(gx, gy, layer);
        } else {
            map.add_blocked_cell(gx, gy, layer);
        }
    }
}

fn via_placement_radius(
    foreign_diameter: i32,
    new_diameter: i32,
    stack_clearance: i32,
    step: i32,
) -> f64 {
    let required =
        foreign_diameter as f64 / 2.0 + new_diameter as f64 / 2.0 + stack_clearance as f64;
    (required / step as f64).max(1.0)
}

fn h2h_cells(
    center: [f64; 2],
    foreign_drill: i32,
    new_drill: i32,
    h2h: i32,
    step: i32,
    upm: f64,
) -> Vec<[i32; 2]> {
    if h2h <= 0 || foreign_drill <= 0 {
        return Vec::new();
    }
    let required = foreign_drill as f64 / 2.0 + new_drill as f64 / 2.0 + h2h as f64;
    let epsilon = GRID_TIE_EPS_MM * upm;
    let radius = (required - epsilon).max(0.0);
    let gx = grid_round(center[0].round() as i32, step);
    let gy = grid_round(center[1].round() as i32, step);
    let extent = (required / step as f64).ceil() as i32 + 1;
    let radius2 = radius * radius;
    let mut cells = Vec::new();
    for dx in -extent..=extent {
        for dy in -extent..=extent {
            let gx = gx + dx;
            let gy = gy + dy;
            let cx = gx as f64 * step as f64;
            let cy = gy as f64 * step as f64;
            let ddx = cx - center[0];
            let ddy = cy - center[1];
            if ddx * ddx + ddy * ddy < radius2 {
                cells.push([gx, gy]);
            }
        }
    }
    cells
}

fn drill_axis_cells(
    axis: [[f64; 2]; 2],
    drill: f64,
    new_drill: i32,
    h2h: i32,
    step: i32,
    upm: f64,
) -> Vec<[i32; 2]> {
    if drill <= 0.0 || h2h <= 0 || step <= 0 {
        return Vec::new();
    }
    let margin = drill / 2.0 + new_drill as f64 / 2.0 + h2h as f64;
    capsule_cells(axis[0], axis[1], margin, step, upm)
}

/// Grid centers strictly inside the continuous segment capsule, matching the
/// source `segment_blocked_spans` predicate and its open boundary tie.
pub(crate) fn capsule_cells(
    a: [f64; 2],
    b: [f64; 2],
    margin: f64,
    step: i32,
    upm: f64,
) -> Vec<[i32; 2]> {
    if !margin.is_finite() || margin <= 0.0 || step <= 0 {
        return Vec::new();
    }
    let stepf = step as f64;
    let eps = GRID_TIE_EPS_MM * upm;
    let m_eff = margin - eps;
    if m_eff <= 0.0 {
        return Vec::new();
    }
    let gx0 = ((a[0].min(b[0]) - margin) / stepf).floor() as i32;
    let gy0 = ((a[1].min(b[1]) - margin) / stepf).floor() as i32;
    let gx1 = ((a[0].max(b[0]) + margin) / stepf).ceil() as i32;
    let gy1 = ((a[1].max(b[1]) + margin) / stepf).ceil() as i32;
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let l2 = dx * dx + dy * dy;
    let m2 = m_eff * m_eff;
    let mut cells = Vec::new();
    for gx in gx0..=gx1 {
        for gy in gy0..=gy1 {
            let x = gx as f64 * stepf;
            let y = gy as f64 * stepf;
            let t = if l2 <= 0.0 {
                0.0
            } else {
                (((x - a[0]) * dx + (y - a[1]) * dy) / l2).clamp(0.0, 1.0)
            };
            let ex = x - (a[0] + t * dx);
            let ey = y - (a[1] + t * dy);
            if ex * ex + ey * ey < m2 {
                cells.push([gx, gy]);
            }
        }
    }
    cells
}

fn stamp_foreign_trace_vias(
    obstacles: &mut GridObstacleMap,
    board: &BoardAdapter,
    config: &NativeGridConfig,
    own: &BTreeSet<NetNo>,
    static_base: bool,
) {
    let step = config.grid_step;
    let mut geometries = vec![(0usize, config.via_size, config.via_drill)];
    geometries.extend(
        via_rung_geometries(config)
            .into_iter()
            .enumerate()
            .map(|(i, g)| (i + 1, g.0, g.1)),
    );
    for key in board.board.get_traces() {
        let item = board.board.item(key);
        if item.net_numbers().iter().any(|net| own.contains(net)) {
            continue;
        }
        let Some(trace) = item.as_trace() else {
            continue;
        };
        let polyline = trace.polyline();
        let layer_no = trace.layer();
        for segment_index in 0..(polyline.corner_count() - 1).max(0) as usize {
            let p0 = polyline.corner(segment_index as i32).to_float();
            let p1 = polyline.corner(segment_index as i32 + 1).to_float();
            for (rung, diameter, _) in &geometries {
                let clearance = resolved_clearance(
                    board,
                    config,
                    config.via_clearance_class,
                    item.clearance_class(),
                    layer_no,
                );
                let margin = *diameter as f64 / 2.0
                    + trace.half_width() as f64
                    + clearance as f64
                    + config.extra_clearance;
                let cells =
                    capsule_cells([p0.x, p0.y], [p1.x, p1.y], margin, step, board.units_per_mm);
                stamp_via_cells(obstacles, &cells, *rung, static_base);
            }
        }
    }
}

fn item_copper_shapes(board: &BoardAdapter, key: ItemKey) -> Vec<(i32, TileShape)> {
    let item = board.board.item(key);
    if item
        .padstack(&board.board)
        .is_some_and(|stack| stack.hole_only)
    {
        return Vec::new();
    }
    board
        .board
        .item_tile_shapes(board.board.item(key))
        .into_iter()
        .enumerate()
        .filter_map(|(index, shape)| {
            let shape = shape.as_ref()?.clone();
            let layer = board.board.shape_layer(key, index as i32);
            let name = board
                .board
                .layer_structure
                .layers
                .get(layer as usize)?
                .name
                .as_str();
            is_copper_layer(board, name).then_some((layer, shape))
        })
        .collect()
}

fn stamp_item_via_cells(
    map: &mut GridObstacleMap,
    board: &BoardAdapter,
    config: &NativeGridConfig,
    key: ItemKey,
    new_diameter: i32,
    rung: usize,
    static_base: bool,
) -> Result<(), String> {
    let item = board.board.item(key);
    let shapes = item_copper_shapes(board, key);
    if shapes.is_empty() {
        return Ok(());
    }
    let clearance = shapes
        .iter()
        .map(|(layer, _)| {
            resolved_clearance(
                board,
                config,
                config.via_clearance_class,
                item.clearance_class(),
                *layer,
            )
        })
        .max()
        .unwrap_or(config.clearance);
    let margin = new_diameter as f64 / 2.0 + clearance as f64 + config.extra_clearance as f64;
    let step = config.grid_step;
    let mut cells = BTreeSet::<(i32, i32)>::new();
    for (_, shape) in shapes {
        // Offset the native convex tile shape itself; this preserves its exact
        // pad outline and clearance distance instead of substituting a bbox.
        let expanded = shape.offset(margin);
        let bounds = expanded.bounding_box();
        let gx0 = (bounds.ll.x as f64 / step as f64).floor() as i32 - 1;
        let gy0 = (bounds.ll.y as f64 / step as f64).floor() as i32 - 1;
        let gx1 = (bounds.ur.x as f64 / step as f64).ceil() as i32 + 1;
        let gy1 = (bounds.ur.y as f64 / step as f64).ceil() as i32 + 1;
        for gx in gx0..=gx1 {
            for gy in gy0..=gy1 {
                let p = Point::Int(IntPoint::new(
                    gx.saturating_mul(step),
                    gy.saturating_mul(step),
                ));
                if expanded.contains(&p) {
                    cells.insert((gx, gy));
                }
            }
        }
    }
    let cells: Vec<_> = cells.into_iter().map(|(x, y)| [x, y]).collect();
    stamp_via_cells(map, &cells, rung, static_base);
    Ok(())
}

/// Add foreign via and foreign-pad obstacles to the via-placement map and
/// stamp foreign-via copper into the track maps. Same-net via and drill rings
/// are owned by `native_same_net`.
pub fn add_foreign_via_obstacles(
    obstacles: &mut GridObstacleMap,
    board: &BoardAdapter,
    config: &NativeGridConfig,
    own_nets: &[NetNo],
    pads: &[ContextPad],
    existing_vias: &[CopperVia],
    static_base: bool,
) -> Result<(), String> {
    if config.grid_step <= 0 || config.route_layers.len() != config.layer_numbers.len() {
        return Err("invalid native foreign-via grid configuration".into());
    }
    let own: BTreeSet<_> = own_nets.iter().copied().collect();
    let upm = board.units_per_mm;
    if !upm.is_finite() || upm <= 0.0 {
        return Err("invalid BoardAdapter unit scale for via obstacles".into());
    }
    let step = config.grid_step;
    let mut foreign_vias = Vec::new();

    for via in existing_vias {
        if own.contains(&(via.net_id as NetNo)) {
            continue;
        }
        foreign_vias.push(via_geometry(board, via)?);
    }
    // This map is independent of which copper layers are routable: source
    // stamps trace-to-via footprints on route and non-route copper layers.
    stamp_foreign_trace_vias(obstacles, board, config, &own, static_base);
    let native_pads = board.pads();
    let mut foreign_pad_meta = Vec::new();
    for pad in pads {
        if own.contains(&(pad.net_id as NetNo)) {
            continue;
        }
        let Some(native) = native_pads.iter().find(|p| p.reference == pad.key) else {
            continue;
        };
        let Some(key) = native.key else {
            continue;
        };
        foreign_pad_meta.push((pad, key));
    }

    // Via-to-track keep-out is fixed by the obstacle via and route track, so
    // it is stamped once into the base track maps. The via-placement map is
    // rebuilt below for rung 0 and every distinct configured route-via size.
    for via in &foreign_vias {
        let gx = grid_round(via.center[0].round() as i32, step);
        let gy = grid_round(via.center[1].round() as i32, step);
        let off = ((via.center[0] - gx as f64 * step as f64).powi(2)
            + (via.center[1] - gy as f64 * step as f64).powi(2))
        .sqrt()
            / step as f64;
        for (layer_index, &layer_no) in config.layer_numbers.iter().enumerate() {
            let clearance = resolved_clearance(
                board,
                config,
                config.trace_clearance_class,
                via.class,
                layer_no,
            );
            let track_radius = ((via.diameter as f64 / 2.0
                + config.track_width as f64 / 2.0
                + clearance as f64
                + config.extra_clearance as f64)
                / step as f64)
                .ceil()
                .max(1.0)
                + VIA_DIAGONAL_MARGIN_CELLS
                + off;
            for cell in circle_cells([gx, gy], track_radius) {
                stamp_track_cells(obstacles, &[cell], layer_index, static_base);
            }
        }
    }

    let mut rung_sizes = via_rung_geometries(config);
    // Rung 0 is the configured run geometry; following indexes use the exact
    // `obstacle_cache.via_rungs` order exposed by NativeGridConfig.
    let mut geometries = vec![(0usize, config.via_size, config.via_drill)];
    geometries.extend(
        rung_sizes
            .drain(..)
            .enumerate()
            .map(|(index, (size, drill))| (index + 1, size, drill)),
    );

    for (rung, new_diameter, new_drill) in geometries {
        // Through-vias occupy every physical copper layer. Explicit keepouts
        // foreign planes and native pads need their own via-radius footprint,
        // independently of the thinner track footprint in the layer map.
        for key in board.board.get_items() {
            let item=board.board.item(key);
            if item.is_on_board()
                && !item.net_numbers().iter().any(|n|own.contains(n))
                && matches!(item.board_item_type(),BoardItemType::ObstacleArea|BoardItemType::ViaObstacleArea|BoardItemType::ConductionArea|BoardItemType::Pin) {
                stamp_item_via_cells(obstacles,board,config,key,new_diameter,rung,static_base)?;
            }
        }

        for via in &foreign_vias {
            let gx = grid_round(via.center[0].round() as i32, step);
            let gy = grid_round(via.center[1].round() as i32, step);
            let off = ((via.center[0] - gx as f64 * step as f64).powi(2)
                + (via.center[1] - gy as f64 * step as f64).powi(2))
            .sqrt()
                / step as f64;
            let stack_clearance =
                stack_clearance(board, config, config.via_clearance_class, via.class);
            let radius =
                via_placement_radius(via.diameter, new_diameter, stack_clearance, step) + off;
            let cells = circle_cells([gx, gy], radius);
            stamp_via_cells(obstacles, &cells, rung, static_base);
            let holes = h2h_cells(
                via.center,
                via.drill,
                new_drill,
                config.hole_to_hole_clearance,
                step,
                upm,
            );
            stamp_via_cells(obstacles, &holes, rung, static_base);
        }
        for &(pad, key) in &foreign_pad_meta {
            stamp_item_via_cells(
                obstacles,
                board,
                config,
                key,
                new_diameter,
                rung,
                static_base,
            )?;
            if pad.drill <= 0.0 || config.hole_to_hole_clearance <= 0 {
                continue;
            }
            let axis = pad
                .drill_axis
                .unwrap_or([pad.point, pad.point])
                .map(|p| [p[0] * upm, p[1] * upm]);
            let cells = drill_axis_cells(
                axis,
                pad.drill * upm,
                new_drill,
                config.hole_to_hole_clearance,
                step,
                upm,
            );
            stamp_via_cells(obstacles, &cells, rung, static_base);
        }
    }
    Ok(())
}

#[cfg(test)]
mod layer_name_tests {
    use super::is_copper_layer;
    use crate::board_adapter::BoardAdapter;

    #[test]
    fn copper_layer_detection_uses_the_native_stack_names() {
        let dsn = include_bytes!("../../tests/fixtures/two_chip.dsn");
        let board = BoardAdapter::load(dsn).unwrap();
        assert!(is_copper_layer(&board, "Top"));
        assert!(is_copper_layer(&board, "Bottom"));
        assert!(!is_copper_layer(&board, "*.Cu"));

        let cu_named = String::from_utf8(dsn.to_vec())
            .unwrap()
            .replace("Top", "F.Cu")
            .replace("Bottom", "B.Cu");
        let board = BoardAdapter::load(cu_named.as_bytes()).unwrap();
        assert!(is_copper_layer(&board, "F.Cu"));
        assert!(is_copper_layer(&board, "B.Cu"));
        assert!(!is_copper_layer(&board, "*.Cu"));
    }
}
