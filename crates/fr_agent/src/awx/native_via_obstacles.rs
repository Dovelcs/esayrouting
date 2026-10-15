//! Source via-placement obstacle stamps for the native base-map builder.
//!
//! Foreign barrels block track cells on each route layer and via cells at the
//! copper-spacing and drill-to-drill thresholds. Foreign pad shapes also block
//! via placement using their actual native tile geometry. Per-net via sizes are
//! stamped into their own via-legality rungs.

use super::native_obstacles::{
    grid_round, obstacle_detail_add, obstacle_elapsed_ns, NativeGridConfig,
};
use super::whole_context::{ContextPad, CopperVia};
use crate::board_adapter::BoardAdapter;
use awx_grid::obstacle_map::GridObstacleMap;
use fr_engine::{
    board::{ItemKey, BoardItemType},
    ids::{ClearanceClassNo, NetNo},
};
use fr_geom::{Shape, TileShape};
use std::collections::BTreeSet;

const VIA_DIAGONAL_MARGIN_CELLS: f64 = 0.25;
const GRID_TIE_EPS_MM: f64 = 1e-6;

/// The regular source pad corner buffer is half a grid step; an active extra
/// clearance uses three quarters of a grid step.
const REGULAR_CORNER_BUFFER_FRACTION: f64 = 0.5;
const EXTRA_CORNER_BUFFER_FRACTION: f64 = 0.75;

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
pub fn capsule_cells(
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
        obstacle_detail_add("via_keepout_trace_items_scanned", 1);
        let item = board.board.item(key);
        if item.net_numbers().iter().any(|net| own.contains(net)) {
            obstacle_detail_add("via_keepout_trace_items_own_skipped", 1);
            continue;
        }
        let Some(trace) = item.as_trace() else {
            continue;
        };
        let polyline = trace.polyline();
        let layer_no = trace.layer();
        for segment_index in 0..(polyline.corner_count() - 1).max(0) as usize {
            obstacle_detail_add("via_keepout_trace_segments", 1);
            let p0 = polyline.corner(segment_index as i32).to_float();
            let p1 = polyline.corner(segment_index as i32 + 1).to_float();
            for (rung, diameter, _) in &geometries {
                obstacle_detail_add("via_keepout_trace_segment_rung_pairs", 1);
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
                obstacle_detail_add("via_keepout_trace_capsule_cells_generated", cells.len() as u64);
                obstacle_detail_add("via_keepout_trace_capsule_stamp_cells", cells.len() as u64);
                stamp_via_cells(obstacles, &cells, *rung, static_base);
            }
        }
    }
}

/// Copper (layer, tile raster, real PadCu primitive) tuples of an item.
///
/// The tile raster is the clearance-compensated default-tree shape used for the
/// exact polygon fallback; the primitive is the source `drill_shape` in board
/// coordinates (real circle/rect for pads, `None` for non-drill items).
fn item_copper_shapes(board: &BoardAdapter, key: ItemKey) -> Vec<(i32, TileShape, Option<Shape>)> {
    let item = board.board.item(key);
    if item
        .padstack(&board.board)
        .is_some_and(|stack| stack.hole_only)
    {
        return Vec::new();
    }
    board
        .board
        .item_tile_shapes(item)
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
            let primitive = item.drill_shape(&board.board, index as i32);
            is_copper_layer(board, name).then_some((layer, shape, primitive))
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
) -> Result<bool, String> {
    obstacle_detail_add("stamp_item_via_cells_calls", 1);
    let item = board.board.item(key);
    let shapes = item_copper_shapes(board, key);
    if shapes.is_empty() {
        return Ok(false);
    }
    let clearance = shapes
        .iter()
        .map(|(layer, _, _)| {
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
    // Source `_add_pad_obstacle`/`_pad_via_keepout_cells` buffers the pad corners
    // by half a grid step (three quarters once an extra clearance is in force).
    let corner_buffer_fraction = if config.extra_clearance > 0.0 {
        EXTRA_CORNER_BUFFER_FRACTION
    } else {
        REGULAR_CORNER_BUFFER_FRACTION
    };
    let mut cells = BTreeSet::<(i32, i32)>::new();
    for (_, shape, primitive) in shapes {
        obstacle_detail_add("via_keepout_pad_layer_shapes", 1);
        // Real PadCu circle/rect primitive: use the source-exact raster, which
        // includes the corner buffer the offset-tile raster omits (the
        // TracePin8 corner grazes).
        if let Some(primitive) = primitive.as_ref() {
            if let Some(source_cells) = super::native_pad_raster::source_pad_cells(
                primitive,
                step,
                board.units_per_mm,
                margin,
                corner_buffer_fraction,
            ) {
                obstacle_detail_add("via_keepout_pad_source_shape_count", 1);
                obstacle_detail_add("via_keepout_pad_source_cells_generated", source_cells.len() as u64);
                cells.extend(source_cells.into_iter().map(|[x, y]| (x, y)));
                continue;
            }
        }
        // Custom polygon fallback shares the source's point-in-polygon and
        // Euclidean nearest-edge predicate with the track raster.
        let corners = primitive
            .as_ref()
            .map(Shape::corner_approx_arr)
            .unwrap_or_else(|| shape.corner_approx_arr());
        obstacle_detail_add("via_keepout_pad_polygon_shape_count", 1);
        let polygon_cells = super::native_pad_raster::source_polygon_cells(
                &corners,
                step,
                margin,
                board.units_per_mm,
            );
        obstacle_detail_add("via_keepout_pad_polygon_cells_generated", polygon_cells.len() as u64);
        cells.extend(polygon_cells.into_iter().map(|[x, y]| (x, y)));
    }
    let cells: Vec<_> = cells.into_iter().map(|(x, y)| [x, y]).collect();
    stamp_via_cells(map, &cells, rung, static_base);
    obstacle_detail_add("via_keepout_pad_unique_cells_stamped", cells.len() as u64);
    Ok(!cells.is_empty())
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

    let via_geometry_started = std::time::Instant::now();
    for via in existing_vias {
        obstacle_detail_add("existing_vias_seen", 1);
        if own.contains(&(via.net_id as NetNo)) {
            obstacle_detail_add("existing_vias_own_skipped", 1);
            continue;
        }
        foreign_vias.push(via_geometry(board, via)?);
    }
    obstacle_detail_add(
        "existing_via_geometry_loop_ns",
        obstacle_elapsed_ns(via_geometry_started),
    );
    obstacle_detail_add("existing_via_geometry_objects", foreign_vias.len() as u64);
    // This map is independent of which copper layers are routable: source
    // stamps trace-to-via footprints on route and non-route copper layers.
    let trace_via_started = std::time::Instant::now();
    stamp_foreign_trace_vias(obstacles, board, config, &own, static_base);
    obstacle_detail_add(
        "stamp_foreign_trace_vias_total_ns",
        obstacle_elapsed_ns(trace_via_started),
    );
    let native_pads_started = std::time::Instant::now();
    let native_pads = board.pads();
    obstacle_detail_add(
        "board_pads_construction_ns",
        obstacle_elapsed_ns(native_pads_started),
    );
    let mut foreign_pad_meta = Vec::new();
    let mut native_pad_comparisons = 0u64;
    let context_mapping_started = std::time::Instant::now();
    for pad in pads {
        obstacle_detail_add("context_pads_seen", 1);
        if own.contains(&(pad.net_id as NetNo)) {
            obstacle_detail_add("context_pads_own_skipped", 1);
            continue;
        }
        let mut native_match = None;
        for native in &native_pads {
            native_pad_comparisons += 1;
            if native.reference == pad.key {
                native_match = Some(native);
                break;
            }
        }
        let Some(native) = native_match else {
            obstacle_detail_add("context_pads_native_reference_missing", 1);
            continue;
        };
        let Some(key) = native.key else {
            obstacle_detail_add("context_pads_native_key_missing", 1);
            continue;
        };
        foreign_pad_meta.push((pad, key));
    }
    obstacle_detail_add(
        "context_pad_native_mapping_ns",
        obstacle_elapsed_ns(context_mapping_started),
    );
    obstacle_detail_add("context_pad_native_reference_comparisons", native_pad_comparisons);
    obstacle_detail_add("context_pad_native_mapping_candidates", pads.len() as u64);
    obstacle_detail_add("context_pad_native_mapping_matches", foreign_pad_meta.len() as u64);
    obstacle_detail_add("native_pad_objects_for_mapping", native_pads.len() as u64);

    // Via-to-track keep-out is fixed by the obstacle via and route track, so
    // it is stamped once into the base track maps. The via-placement map is
    // rebuilt below for rung 0 and every distinct configured route-via size.
    let foreign_via_track_started = std::time::Instant::now();
    for via in &foreign_vias {
        let gx = grid_round(via.center[0].round() as i32, step);
        let gy = grid_round(via.center[1].round() as i32, step);
        let off = ((via.center[0] - gx as f64 * step as f64).powi(2)
            + (via.center[1] - gy as f64 * step as f64).powi(2))
        .sqrt()
            / step as f64;
        for (layer_index, &layer_no) in config.layer_numbers.iter().enumerate() {
            obstacle_detail_add("foreign_via_track_layers_stamped", 1);
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
            let cells = circle_cells([gx, gy], track_radius);
            obstacle_detail_add("foreign_via_track_circle_cells_generated", cells.len() as u64);
            for cell in cells {
                stamp_track_cells(obstacles, &[cell], layer_index, static_base);
            }
        }
    }
    obstacle_detail_add(
        "foreign_via_track_keepout_loop_ns",
        obstacle_elapsed_ns(foreign_via_track_started),
    );

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
        obstacle_detail_add("foreign_via_rungs_processed", 1);
        let rung_item_started = std::time::Instant::now();
        let mut rung_item_accepted = 0u64;
        let mut generic_pin_stamps = BTreeSet::new();
        let can_skip_duplicate_static_stamp = rung == 0 && static_base;
        // Through-vias occupy every physical copper layer. Explicit keepouts
        // foreign planes and native pads need their own via-radius footprint,
        // independently of the thinner track footprint in the layer map.
        for key in board.board.get_items() {
            obstacle_detail_add("via_rung_item_candidates", 1);
            let item=board.board.item(key);
            if item.is_on_board()
                && !item.net_numbers().iter().any(|n|own.contains(n))
                && matches!(item.board_item_type(),BoardItemType::ObstacleArea|BoardItemType::ViaObstacleArea|BoardItemType::ConductionArea|BoardItemType::Pin) {
                obstacle_detail_add(&format!("via_rung_item_type_{:?}_accepted", item.board_item_type()), 1);
                rung_item_accepted += 1;
                let stamped = stamp_item_via_cells(
                    obstacles,
                    board,
                    config,
                    key,
                    new_diameter,
                    rung,
                    static_base,
                )?;
                if can_skip_duplicate_static_stamp
                    && item.board_item_type() == BoardItemType::Pin
                    && stamped
                {
                    generic_pin_stamps.insert(key);
                }
            } else {
                obstacle_detail_add("via_rung_items_filtered", 1);
            }
        }
        obstacle_detail_add(
            "via_rung_item_keepout_loop_ns",
            obstacle_elapsed_ns(rung_item_started),
        );
        obstacle_detail_add("via_rung_item_keepouts_accepted", rung_item_accepted);

        let foreign_via_rung_started = std::time::Instant::now();
        for via in &foreign_vias {
            obstacle_detail_add("foreign_via_rung_candidates", 1);
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
            obstacle_detail_add("foreign_via_rung_circle_cells_generated", cells.len() as u64);
            stamp_via_cells(obstacles, &cells, rung, static_base);
            let holes = h2h_cells(
                via.center,
                via.drill,
                new_drill,
                config.hole_to_hole_clearance,
                step,
                upm,
            );
            obstacle_detail_add("foreign_via_h2h_cells_generated", holes.len() as u64);
            stamp_via_cells(obstacles, &holes, rung, static_base);
        }
        obstacle_detail_add(
            "foreign_via_rung_geometry_and_stamp_ns",
            obstacle_elapsed_ns(foreign_via_rung_started),
        );
        let foreign_pad_rung_started = std::time::Instant::now();
        let mut foreign_pad_rung_count = 0u64;
        let mut foreign_pad_drill_axis_ns = 0u64;
        for &(pad, key) in &foreign_pad_meta {
            foreign_pad_rung_count += 1;
            obstacle_detail_add("foreign_pad_rung_candidates", 1);
            if can_skip_duplicate_static_stamp && generic_pin_stamps.contains(&key) {
                obstacle_detail_add("duplicate_stamps_avoided", 1);
            } else {
                stamp_item_via_cells(
                    obstacles,
                    board,
                    config,
                    key,
                    new_diameter,
                    rung,
                    static_base,
                )?;
            }
            if pad.drill <= 0.0 || config.hole_to_hole_clearance <= 0 {
                continue;
            }
            let axis = pad
                .drill_axis
                .unwrap_or([pad.point, pad.point])
                .map(|p| [p[0] * upm, p[1] * upm]);
            let drill_axis_started = std::time::Instant::now();
            let cells = drill_axis_cells(
                axis,
                pad.drill * upm,
                new_drill,
                config.hole_to_hole_clearance,
                step,
                upm,
            );
            foreign_pad_drill_axis_ns = foreign_pad_drill_axis_ns
                .saturating_add(obstacle_elapsed_ns(drill_axis_started));
            obstacle_detail_add("foreign_pad_drill_axis_cells_generated", cells.len() as u64);
            stamp_via_cells(obstacles, &cells, rung, static_base);
        }
        obstacle_detail_add(
            "foreign_pad_raster_and_write_per_rung_ns",
            obstacle_elapsed_ns(foreign_pad_rung_started),
        );
        obstacle_detail_add("foreign_pad_rung_objects", foreign_pad_rung_count);
        obstacle_detail_add("foreign_pad_drill_axis_geometry_ns", foreign_pad_drill_axis_ns);
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
