//! Native BoardAdapter to source grid-obstacle-map bridge.
//! Cell coordinates are global GridCoord coordinates (origin at board (0,0)); all dimensions
//! in `NativeGridConfig` are board units.

use crate::{awx::braid_connect::RungRequest, board_adapter::BoardAdapter};
use awx_grid::obstacle_map::GridObstacleMap;
use fr_engine::{
    board::BoardItemType,
    ids::{ClearanceClassNo, NetNo},
};
use fr_geom::{IntBox, IntPoint, Point};
use std::collections::BTreeMap;

#[derive(Clone, Debug)]
pub struct NativeGridConfig {
    pub grid_step: i32,
    pub clearance: i32,
    pub track_width: i32,
    pub via_size: i32,
    pub via_drill: i32,
    pub hole_to_hole_clearance: i32,
    pub same_net_pad_clearance: i32,
    pub board_edge_clearance: i32,
    /// AWX `connect.make_config` leaves exclusion zones at GridRouteConfig's empty default.
    pub bga_exclusion_zones: Vec<[i32; 4]>,
    /// Source GridRouteConfig defaults (radius 7 mm, cost 0.2) in board units / raw cost.
    pub bga_proximity_radius: i32,
    pub bga_proximity_cost: f64,
    /// Source `build_base_obstacle_map(extra_clearance=...)`, applied only to foreign copper.
    pub extra_clearance: f64,
    pub route_layers: Vec<String>,
    pub layer_numbers: Vec<i32>,
    pub net_via_sizes: BTreeMap<NetNo, (i32, i32)>,
    /// Source net ViaRule metadata keyed by exact native `(diameter, drill)`.
    /// The bool controls whether a same-net SMD pin may accept a new via.
    pub via_rule_geometry: BTreeMap<(i32, i32), (ClearanceClassNo, bool)>,
    /// Source #568 fab ladder, in resolver order, expressed in board units.
    pub fab_via_rungs: Vec<(i32, i32)>,
    /// Exact source map rung order after run rung 0: small-fab rung first when
    /// present, followed by distinct per-net geometries in descending size order.
    pub ordered_via_rungs: Vec<(i32, i32)>,
    pub trace_clearance_class: ClearanceClassNo,
    pub via_clearance_class: ClearanceClassNo,
}

impl NativeGridConfig {
    pub fn from_request(
        board: &BoardAdapter,
        request: &RungRequest,
        trace_clearance_class: ClearanceClassNo,
        via_clearance_class: ClearanceClassNo,
    ) -> Result<Self, String> {
        let upm = board.units_per_mm;
        let to_native = |mm: f64| -> Result<i32, String> {
            let v = (mm * upm).round();
            if !v.is_finite() || v < 0.0 || v > i32::MAX as f64 {
                return Err("native obstacle dimension is outside board-unit range".into());
            }
            Ok(v as i32)
        };
        if !upm.is_finite() || upm <= 0.0 {
            return Err("invalid BoardAdapter unit scale".into());
        }
        let mut layer_numbers = Vec::with_capacity(request.route_layers.len());
        for name in &request.route_layers {
            let no = board.board.layer_structure.get_no(name);
            if no < 0 {
                return Err(format!("route layer is absent from board: {name}"));
            }
            layer_numbers.push(no);
        }
        let net_via_sizes: BTreeMap<NetNo, (i32, i32)> = request
            .rules
            .net_via_sizes
            .iter()
            .map(|(&n, v)| Ok((n as NetNo, (to_native(v[0])?, to_native(v[1])?))))
            .collect::<Result<BTreeMap<_, _>, String>>()?;
        let mut via_rule_geometry = BTreeMap::new();
        if request.pair.is_none() {
            let class = board.board.rules.net_class_of(request.net_id);
            if let Some(rule_id) = class.get_via_rule() {
                let copper_layers: Vec<_> = board
                    .board
                    .layer_structure
                    .layers
                    .iter()
                    .enumerate()
                    .filter_map(|(index, layer)| layer.is_signal.then_some(index as i32))
                    .collect();
                if let (Some(&first), Some(&last)) =
                    (copper_layers.first(), copper_layers.last())
                {
                    for &via_info_id in board.board.rules.via_rules[rule_id].vias() {
                        let info = &board.board.rules.via_infos[via_info_id];
                        let padstack = board
                            .board
                            .library
                            .padstacks
                            .get(info.get_padstack())
                            .ok_or("source ViaRule references a missing padstack")?;
                        if padstack.from_layer() != first || padstack.to_layer() != last {
                            continue;
                        }
                        let Some(drill_radius) = padstack.get_explicit_drill_radius() else {
                            continue;
                        };
                        let diameter = (first..=last)
                            .filter(|&layer| {
                                board.board.layer_structure.layers[layer as usize].is_signal
                            })
                            .filter_map(|layer| padstack.get_shape(layer))
                            .map(|shape| shape.max_width())
                            .fold(0.0_f64, f64::max);
                        let native_diameter = to_native(diameter / upm)?;
                        let native_drill = to_native(2.0 * drill_radius / upm)?;
                        if native_diameter > native_drill && native_drill > 0 {
                            via_rule_geometry.entry((native_diameter, native_drill)).or_insert((
                                info.get_clearance_class_index(),
                                info.attach_smd_allowed(),
                            ));
                        }
                    }
                }
            }
        }
        let round_mm = |value: f64, digits: i32| {
            let scale = 10_f64.powi(digits);
            (value * scale).round_ties_even() / scale
        };
        // Replan resolved the actual AwxSpec fabrication/escalation policy,
        // overrides, board floors, and net-specific rule floors before making
        // RungRequest. Preserve that resolver order exactly; rebuilding with
        // hard-coded Auto/Fab here would silently ignore those source inputs.
        let fab_via_rungs = request
            .fab_via_rungs
            .iter()
            .map(|&(diameter, drill)| {
                Ok((
                    to_native(round_mm(diameter, 3))?,
                    to_native(round_mm(drill, 3))?,
                ))
            })
            .collect::<Result<Vec<_>, String>>()?;
        let run = (
            to_native(round_mm(request.rules.via_size, 4))?,
            to_native(round_mm(request.rules.via_drill, 4))?,
        );
        let small_fab: Option<(i32, i32)> = request
            .fab_via_rungs
            .iter()
            .map(|&(diameter, drill)| (round_mm(diameter, 3), round_mm(drill, 3)))
            .find(|(diameter, _)| *diameter < request.rules.via_size - 1e-9)
            .map(|(diameter, drill)| {
                Ok::<(i32, i32), String>((to_native(diameter)?, to_native(drill)?))
            })
            .transpose()?;
        let mut ordered_via_rungs = Vec::new();
        if let Some(small) = small_fab.filter(|geometry| *geometry != run) {
            ordered_via_rungs.push(small);
        }
        let mut per_net_geometries: Vec<_> = request
            .rules
            .net_via_sizes
            .values()
            .map(|size| {
                Ok((
                    to_native(round_mm(size[0], 4))?,
                    to_native(round_mm(size[1], 4))?,
                ))
            })
            .collect::<Result<Vec<_>, String>>()?;
        per_net_geometries
            .retain(|geometry| *geometry != run && !ordered_via_rungs.contains(geometry));
        per_net_geometries.sort_by(|a, b| b.0.cmp(&a.0).then_with(|| b.1.cmp(&a.1)));
        per_net_geometries.dedup();
        ordered_via_rungs.extend(per_net_geometries);
        Ok(Self {
            grid_step: to_native(request.rules.grid_step)?.max(1),
            clearance: to_native(request.rules.clearance)?,
            track_width: to_native(request.rules.track_width)?,
            via_size: to_native(request.rules.via_size)?,
            via_drill: to_native(request.rules.via_drill)?,
            hole_to_hole_clearance: to_native(request.rules.hole_to_hole_clearance)?,
            same_net_pad_clearance: to_native(request.rules.same_net_pad_clearance.max(0.0))?,
            board_edge_clearance: to_native(request.rules.board_edge_clearance)?,
            bga_exclusion_zones: Vec::new(),
            bga_proximity_radius: to_native(7.0)?,
            bga_proximity_cost: 0.2,
            extra_clearance: 0.0,
            route_layers: request.route_layers.clone(),
            layer_numbers,
            net_via_sizes,
            via_rule_geometry,
            fab_via_rungs,
            ordered_via_rungs,
            trace_clearance_class,
            via_clearance_class,
        })
    }
}

pub struct ObstacleMapResult {
    pub obstacles: GridObstacleMap,
    /// Native board layer number for each configured route layer index.
    pub layer_numbers: Vec<i32>,
    pub config: NativeGridConfig,
}

/// Internal projection for consumers that read only one obstacle channel.
/// Ordinary map entry points always use `Full`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum MapProjection {
    Full,
    TrackOnly,
    ViaOnly,
}

impl MapProjection {
    pub(crate) fn track(self) -> bool {
        matches!(self, Self::Full | Self::TrackOnly)
    }

    pub(crate) fn via(self) -> bool {
        matches!(self, Self::Full | Self::ViaOnly)
    }
}

pub fn grid_round(x: i32, step: i32) -> i32 {
    // Source GridCoord.to_grid uses Java/Python round-to-nearest semantics.
    ((x as f64 / step as f64).round_ties_even()) as i32
}

pub fn cell_center(gx: i32, gy: i32, step: i32) -> (i32, i32) {
    (gx.saturating_mul(step), gy.saturating_mul(step))
}

fn in_box(box_: &IntBox, x: i32, y: i32) -> bool {
    x >= box_.ll.x && x <= box_.ur.x && y >= box_.ll.y && y <= box_.ur.y
}

fn blocks_track_search(kind: BoardItemType) -> bool {
    kind != BoardItemType::ViaObstacleArea
}

/// Source `build_base_obstacle_map(..., static_base=True)` for the immutable copper body.
/// Window fencing, reusable same-net vias and source-owned via keep-outs are composed by the
/// sibling native_grid_window/native_same_net modules immediately after this base pass.
pub fn build_obstacle_map_with_pair_extra(
    board: &BoardAdapter,
    request: &RungRequest,
    window: IntBox,
    grid_step_board: i32,
    route_layers: &[String],
    own_nets: &[NetNo],
    clearance_class: i32,
    via_clearance_class: i32,
    extra_foreign_mm: f64,
) -> Result<ObstacleMapResult, String> {
    if grid_step_board <= 0 {
        return Err("native obstacle grid step must be positive".into());
    }
    if route_layers != request.route_layers.as_slice() {
        return Err("native obstacle layer order differs from RungRequest".into());
    }
    let mut config = NativeGridConfig::from_request(
        board,
        request,
        clearance_class as ClearanceClassNo,
        via_clearance_class as ClearanceClassNo,
    )?;
    let extra = extra_foreign_mm * board.units_per_mm;
    if !extra.is_finite() || extra < 0.0 || extra > i32::MAX as f64 {
        return Err("native extra foreign clearance is outside board-unit range".into());
    }
    config.extra_clearance = extra;
    if config.grid_step != grid_step_board {
        return Err("native obstacle grid step differs from source route configuration".into());
    }
    build_obstacle_map_from_config(board, request, window, own_nets, config, MapProjection::Full)
}

/// Build both native maps from an already-resolved source config. Fine tap
/// passes use this entry with their per-candidate grid, clearance and widths.
pub fn build_obstacle_map_with_config(
    board: &BoardAdapter,
    request: &RungRequest,
    window: IntBox,
    own_nets: &[NetNo],
    config: NativeGridConfig,
) -> Result<ObstacleMapResult, String> {
    build_obstacle_map_from_config(board, request, window, own_nets, config, MapProjection::Full)
}

/// Tap-only map builder. The public ordinary map API remains complete.
pub(crate) fn build_obstacle_map_with_config_projected(
    board: &BoardAdapter,
    request: &RungRequest,
    window: IntBox,
    own_nets: &[NetNo],
    config: NativeGridConfig,
    projection: MapProjection,
) -> Result<ObstacleMapResult, String> {
    build_obstacle_map_from_config(board, request, window, own_nets, config, projection)
}

/// Build the source single-layer `build_routing_obstacle_map` footprint for a tap.
pub fn build_obstacle_map_on_layer(
    board: &BoardAdapter,
    request: &RungRequest,
    window: IntBox,
    own_nets: &[NetNo],
    mut config: NativeGridConfig,
    layer: &str,
) -> Result<ObstacleMapResult, String> {
    let Some(index) = config.route_layers.iter().position(|name| name == layer) else {
        return Err(format!(
            "tap routing layer is outside the configured route stack: {layer}"
        ));
    };
    let layer_no = *config
        .layer_numbers
        .get(index)
        .ok_or("tap routing layer has no native board-layer mapping")?;
    config.route_layers = vec![layer.to_owned()];
    config.layer_numbers = vec![layer_no];
    build_obstacle_map_from_config(board, request, window, own_nets, config, MapProjection::Full)
}

/// Tap-only single-layer map builder. Ordinary callers retain both channels.
pub(crate) fn build_obstacle_map_on_layer_projected(
    board: &BoardAdapter,
    request: &RungRequest,
    window: IntBox,
    own_nets: &[NetNo],
    mut config: NativeGridConfig,
    layer: &str,
    projection: MapProjection,
) -> Result<ObstacleMapResult, String> {
    let Some(index) = config.route_layers.iter().position(|name| name == layer) else {
        return Err(format!(
            "tap routing layer is outside the configured route stack: {layer}"
        ));
    };
    let layer_no = *config
        .layer_numbers
        .get(index)
        .ok_or("tap routing layer has no native board-layer mapping")?;
    config.route_layers = vec![layer.to_owned()];
    config.layer_numbers = vec![layer_no];
    build_obstacle_map_from_config(board, request, window, own_nets, config, projection)
}

fn build_obstacle_map_from_config(
    board: &BoardAdapter,
    request: &RungRequest,
    window: IntBox,
    own_nets: &[NetNo],
    config: NativeGridConfig,
    projection: MapProjection,
) -> Result<ObstacleMapResult, String> {
    let grid_step_board = config.grid_step;
    if grid_step_board <= 0 {
        return Err("native obstacle grid step must be positive".into());
    }
    if config.route_layers.is_empty()
        || config
            .route_layers
            .iter()
            .any(|layer| !request.route_layers.contains(layer))
    {
        return Err("native obstacle layer order differs from RungRequest".into());
    }
    let map_layers = config.layer_numbers.len();
    if map_layers == 0 {
        return Err("native obstacle map requires route layers".into());
    }
    if projection == MapProjection::TrackOnly && map_layers != 1 {
        return Err("track-only obstacle projection requires one route layer".into());
    }
    let mut obstacles = GridObstacleMap::new(map_layers);
    if config.bga_proximity_cost > 0.0 {
        obstacles.set_bga_proximity_radius(
            (config.bga_proximity_radius as f64 / grid_step_board as f64).floor() as i32,
        );
    }
    crate::awx::native_bga_zones::add_bga_zones(&mut obstacles, board, &config)?;
    let own: std::collections::BTreeSet<NetNo> = own_nets.iter().copied().collect();
    let route_layer_to_index: BTreeMap<i32, usize> = config
        .layer_numbers
        .iter()
        .enumerate()
        .map(|(i, &l)| (l, i))
        .collect();
    let route_clearance = config.trace_clearance_class;
    let step = grid_step_board;

    for key in board.board.get_items() {
        let item = board.board.item(key);
        if !item.is_on_board() {
            continue;
        }
        if item.net_numbers().iter().any(|n| own.contains(n)) {
            continue;
        }
        let kind = item.board_item_type();
        // Only physical copper and explicit keep-outs participate in the source base map.
        if !matches!(
            kind,
            BoardItemType::Trace
                | BoardItemType::Pin
                | BoardItemType::ObstacleArea
                | BoardItemType::ViaObstacleArea
                | BoardItemType::ConductionArea
        ) {
            continue;
        }
        if kind == BoardItemType::Trace {
            let Some(trace) = item.as_trace() else {
                continue;
            };
            let layer_no = trace.layer();
            let Some(&layer_index) = route_layer_to_index.get(&layer_no) else {
                // Foreign tracks outside the searched copper stack only need
                // via keep-outs; native_via_obstacles stamps those from the
                // exact continuous centerline capsules.
                continue;
            };
            if !projection.track() {
                continue;
            }
            let clearance = board
                .board
                .clearance_value(route_clearance, item.clearance_class(), layer_no)
                .max(config.clearance)
                .max(0);
            let margin = config.track_width as f64 / 2.0
                + trace.half_width() as f64
                + clearance as f64
                + config.extra_clearance;
            let corners = trace.polyline().corners();
            for points in corners.windows(2) {
                let cells = crate::awx::native_via_obstacles::capsule_cells(
                    [points[0].to_float().x, points[0].to_float().y],
                    [points[1].to_float().x, points[1].to_float().y],
                    margin,
                    step,
                    board.units_per_mm,
                );
                for [gx, gy] in cells {
                    let (x, y) = cell_center(gx, gy, step);
                    if in_box(&window, x, y) {
                        obstacles.add_static_blocked_cell(gx, gy, layer_index);
                    }
                }
            }
            continue;
        }
        // Via-only keep-outs are stamped with each candidate's physical via
        // span by native_via_obstacles. They must not enter the copper-cell
        // map used by same-layer track searches.
        if !blocks_track_search(kind) {
            continue;
        }
        // Source `obstacle_map._add_pad_obstacle` (lines 4131..4270): a foreign
        // pin blocks the candidate track on every physical copper layer it
        // occupies, using the transformed *pad copper* shape (Java
        // `DrillItem.getShapeOnLayer`, not the drill hole) and the source
        // `routing_utils.iter_pad_blocked_cells` corner buffer. The current
        // `item_tile_shapes` offset sampler dropped that buffer, so Circle and
        // rectangle pads are rasterized by `native_pad_raster::source_pad_cells`
        // with the .5/.75 grid fraction; unsupported custom polygons fall back
        // to the exact native polygon offset raster below.
        if kind == BoardItemType::Pin {
            if !projection.track() {
                continue;
            }
            let clearance_class = item.clearance_class();
            let corner_buffer_fraction = if config.extra_clearance > 0.0 {
                0.75
            } else {
                0.5
            };
            let from = item.first_layer(&board.board);
            let to = item.last_layer(&board.board);
            for layer_no in from..=to {
                let Some(&layer_index) = route_layer_to_index.get(&layer_no) else {
                    // Copper outside the searched stack only needs via keep-outs,
                    // which native_via_obstacles stamps from the pad copper.
                    continue;
                };
                let Some(shape) = item.drill_shape_on_layer(&board.board, layer_no) else {
                    continue;
                };
                let layer_clearance = board
                    .board
                    .clearance_value(route_clearance, clearance_class, layer_no)
                    .max(config.clearance)
                    .max(0);
                // Source pair clearance + candidate track half-width + run extra.
                let margin = layer_clearance as f64
                    + config.track_width as f64 / 2.0
                    + config.extra_clearance;
                let source_cells = crate::awx::native_pad_raster::source_pad_cells_shared(
                    &shape,
                    grid_step_board,
                    board.units_per_mm,
                    margin,
                    corner_buffer_fraction,
                );
                match source_cells {
                    Some(cells) => {
                        for &[gx, gy] in cells.iter() {
                            let (x, y) = cell_center(gx, gy, step);
                            if in_box(&window, x, y) {
                                obstacles.add_static_blocked_cell(gx, gy, layer_index);
                            }
                        }
                    }
                    None => {
                        // Source custom polygons use point-in-polygon plus exact
                        // Euclidean distance to the nearest original edge.
                        let cells = crate::awx::native_pad_raster::source_polygon_cells_shared(
                            &shape.corner_approx_arr(),
                            grid_step_board,
                            margin,
                            board.units_per_mm,
                        );
                        for &[gx, gy] in cells.iter() {
                            let (x, y) = cell_center(gx, gy, step);
                            if in_box(&window, x, y) {
                                obstacles.add_static_blocked_cell(gx, gy, layer_index);
                            }
                        }
                    }
                }
            }
            continue;
        }
        let shapes = board.board.item_tile_shapes(item);
        for (shape_index, shape) in shapes.iter().enumerate() {
            let Some(shape) = shape else { continue };
            let layer_no = board.board.shape_layer(key, shape_index as i32);
            let layer_clearance = board
                .board
                .clearance_value(route_clearance, item.clearance_class(), layer_no)
                .max(config.clearance)
                .max(0);
            let Some(&layer_index) = route_layer_to_index.get(&layer_no) else {
                // Source obstacle_map.py: a foreign track on a copper layer outside cfg.layers
                // cannot block a track search, but its barrel must still block a new via.
                // The native tile shape already contains the foreign copper width/shape.
                if projection.via()
                    && matches!(kind, BoardItemType::Trace | BoardItemType::Pin)
                    && board
                        .board
                        .layer_structure
                        .layers
                        .get(layer_no as usize)
                        .is_some_and(|layer| board.board.layer_structure.get_no(&layer.name) >= 0)
                {
                    let via_clearance = board
                        .board
                        .clearance_value(
                            config.via_clearance_class,
                            item.clearance_class(),
                            layer_no,
                        )
                        .max(config.clearance)
                        .max(0);
                    let expanded = shape.offset(
                        config.via_size as f64 / 2.0
                            + via_clearance as f64
                            + config.extra_clearance,
                    );
                    let bounds = expanded.bounding_box();
                    let gx0 = grid_round(bounds.ll.x, step).saturating_sub(1);
                    let gy0 = grid_round(bounds.ll.y, step).saturating_sub(1);
                    let gx1 = grid_round(bounds.ur.x, step).saturating_add(1);
                    let gy1 = grid_round(bounds.ur.y, step).saturating_add(1);
                    for gx in gx0..=gx1 {
                        for gy in gy0..=gy1 {
                            let x = gx.saturating_mul(step);
                            let y = gy.saturating_mul(step);
                            if in_box(&window, x, y)
                                && expanded.contains(&Point::Int(IntPoint::new(x, y)))
                            {
                                obstacles.add_static_blocked_via(gx, gy);
                            }
                        }
                    }
                }
                continue;
            };
            if !projection.track() {
                continue;
            }
            // The native TileShape is the foreign item's copper body; the candidate track's
            // own half-width and the source pairwise class clearance expand that exact geometry.
            let expand =
                layer_clearance as f64 + config.track_width as f64 / 2.0 + config.extra_clearance;
            let expanded = shape.offset(expand);
            let bounds = expanded.bounding_box();
            let gx0 = grid_round(bounds.ll.x, step).saturating_sub(1);
            let gy0 = grid_round(bounds.ll.y, step).saturating_sub(1);
            let gx1 = grid_round(bounds.ur.x, step).saturating_add(1);
            let gy1 = grid_round(bounds.ur.y, step).saturating_add(1);
            for gx in gx0..=gx1 {
                for gy in gy0..=gy1 {
                    let x = gx.saturating_mul(step);
                    let y = gy.saturating_mul(step);
                    if !in_box(&window, x, y) {
                        continue;
                    }
                    if expanded.contains(&Point::Int(IntPoint::new(x, y))) {
                        obstacles.add_static_blocked_cell(gx, gy, layer_index);
                    }
                }
            }
        }
    }
    crate::awx::native_outline::stamp_outline_and_cutouts(&mut obstacles, board, window, &config)?;
    crate::awx::native_via_obstacles::add_foreign_via_obstacles_projected(
        &mut obstacles,
        board,
        &config,
        own_nets,
        &request.same_net_pads,
        &request.existing_vias,
        true,
        projection,
    )?;
    if projection.via() {
        for &net in own_nets {
            crate::awx::native_same_net::add_same_net_via_clearance(
                &mut obstacles,
                board,
                net,
                &config,
                &request.same_net_pads,
                &request.existing_vias,
                &request.paste_pads,
            )?;
            crate::awx::native_same_net::add_same_net_pad_drill_via_clearance(
                &mut obstacles,
                board,
                net,
                &config,
                &request.same_net_pads,
            )?;
        }
    }
    crate::awx::native_grid_window::fence_window(&mut obstacles, board, window, &config)?;
    crate::awx::native_grid_window::add_free_via_positions(
        &mut obstacles,
        board,
        own_nets,
        &request.free_via_pads,
        grid_step_board,
    )?;
    Ok(ObstacleMapResult {
        obstacles,
        layer_numbers: config.layer_numbers.clone(),
        config,
    })
}

#[cfg(test)]
mod tests {
    use super::{
        blocks_track_search, build_obstacle_map_on_layer,
        build_obstacle_map_on_layer_projected, build_obstacle_map_with_config,
        build_obstacle_map_with_config_projected, MapProjection, NativeGridConfig,
    };
    use crate::{
        awx::{
            braid_connect::{RungEndpoint, RungLabel, RungRequest, RungSpec},
            fab_rules::{EscalationPolicy, FabTier},
            whole_context::RoutingRules,
        },
        board_adapter::BoardAdapter,
    };
    use awx_grid::obstacle_map::GridObstacleMap;
    use fr_engine::board::BoardItemType;
    use std::collections::BTreeMap;

    #[test]
    fn via_obstacle_area_allows_tracks_but_keeps_via_cell_blocked() {
        let kind = BoardItemType::ViaObstacleArea;
        let mut map = GridObstacleMap::new(1);
        // Model the two independent maps composed by build_obstacle_map_from_config:
        // a via-only area contributes to the via map, never the track map.
        if blocks_track_search(kind) {
            map.add_static_blocked_cell(0, 0, 0);
        }
        map.add_static_blocked_via(0, 0);

        assert!(!map.is_blocked(0, 0, 0), "same-layer trace must pass");
        assert!(map.is_via_blocked(0, 0), "via placement must remain forbidden");
    }

    #[test]
    fn tap_map_projections_match_their_consumer_channels() {
        let board = BoardAdapter::load(include_bytes!("../../tests/fixtures/two_chip.dsn"))
            .expect("load map projection fixture");
        let net_id = board.net_number("BUS").expect("fixture BUS net");
        let request = RungRequest {
            board: board.clone(),
            net: "BUS".into(),
            net_id,
            stage: "projection_test".into(),
            start: RungEndpoint {
                point: [5.0, 5.0],
                layer: "Top".into(),
            },
            end: RungEndpoint {
                point: [15.0, 5.0],
                layer: "Top".into(),
            },
            b_alternatives: Vec::new(),
            pair: None,
            spec: RungSpec {
                label: RungLabel::Free6,
                slack: 0.0,
                open_layers: false,
                margin: 0.0,
                use_window: false,
            },
            band: None,
            band_slack: 0.0,
            window_points: None,
            virtual_tracks: Vec::new(),
            virtual_vias: Vec::new(),
            free_via_pads: Vec::new(),
            same_net_pads: Vec::new(),
            existing_vias: Vec::new(),
            paste_pads: BTreeMap::new(),
            min_annular_ring_mm: 0.0,
            rules: RoutingRules {
                grid_step: 0.1,
                track_width: 0.15,
                clearance: 0.1,
                via_size: 0.5,
                via_drill: 0.3,
                pair_gap: 0.0,
                lane_pitch: 0.5,
                lane_min: 0.15,
                min_turning_radius: 0.0,
                max_turn_angle: 90.0,
                max_setback_angle: 90.0,
                routing_clearance_margin: 0.0,
                board_edge_clearance: 0.0,
                hole_to_hole_clearance: 0.0,
                fan_clearance: 0.0,
                geometry_pair_room: false,
                geometry_pair_turn_room_steps: None,
                same_net_pad_clearance: 0.0,
                net_via_sizes: BTreeMap::new(),
            },
            fabrication: FabTier::Standard,
            escalation: EscalationPolicy::Off,
            fabrication_overrides: BTreeMap::new(),
            fab_clearance_floor_mm: 0.1,
            fab_track_floor_mm: 0.15,
            may_narrow: false,
            fab_via_rungs: Vec::new(),
            max_iterations: 1_000,
            route_layers: vec!["Top".into(), "Bottom".into()],
            deadline: None,
        };
        let config = NativeGridConfig::from_request(&board, &request, 0, 0)
            .expect("fixture obstacle config");
        let window = board.board.bounding_box();
        let own_nets = [net_id];

        let via_full = build_obstacle_map_with_config(
            &board,
            &request,
            window,
            &own_nets,
            config.clone(),
        )
        .expect("full via map");
        let via_projected = build_obstacle_map_with_config_projected(
            &board,
            &request,
            window,
            &own_nets,
            config.clone(),
            MapProjection::ViaOnly,
        )
        .expect("projected via map");
        for gx in -2..=202 {
            for gy in -2..=102 {
                assert_eq!(
                    via_full.obstacles.is_via_blocked_rung(gx, gy, 0),
                    via_projected.obstacles.is_via_blocked_rung(gx, gy, 0),
                    "via channel differs at ({gx}, {gy})",
                );
            }
        }

        let route_full = build_obstacle_map_on_layer(
            &board,
            &request,
            window,
            &own_nets,
            config.clone(),
            "Top",
        )
        .expect("full route map");
        let route_projected = build_obstacle_map_on_layer_projected(
            &board,
            &request,
            window,
            &own_nets,
            config,
            "Top",
            MapProjection::TrackOnly,
        )
        .expect("projected route map");
        for gx in -2..=202 {
            for gy in -2..=102 {
                assert_eq!(
                    route_full.obstacles.is_blocked(gx, gy, 0),
                    route_projected.obstacles.is_blocked(gx, gy, 0),
                    "track channel differs at ({gx}, {gy})",
                );
            }
        }

        let router = fr_engine::autoroute::point_router::SourceGridConfig::from_source(
            0.1,
            75,
            1,
            &[0],
            Some(vec![1_000]),
            Some(vec![255]),
        )
        .expect("single-layer router config")
        .router();
        let route = |obstacles: &GridObstacleMap| {
            fr_engine::autoroute::point_router::route_with_frontier(
                &router,
                obstacles,
                vec![(50, 50, 0)],
                vec![(150, 50, 0)],
                1_000,
                awx_grid::TrackMarginArg::Scalar(0.0),
                None,
                None,
            )
        };
        let full_path = route(&route_full.obstacles).0;
        let projected_path = route(&route_projected.obstacles).0;
        assert_eq!(full_path, projected_path);
        assert!(full_path.is_some(), "fixture route should be open");
    }
}

/// The ordinary source `connect()` path has no extra pair-centreline expansion.
#[allow(clippy::too_many_arguments)]
pub fn build_obstacle_map(
    board: &BoardAdapter,
    request: &RungRequest,
    window: IntBox,
    grid_step_board: i32,
    route_layers: &[String],
    own_nets: &[NetNo],
    clearance_class: i32,
    via_clearance_class: i32,
) -> Result<ObstacleMapResult, String> {
    build_obstacle_map_with_pair_extra(
        board,
        request,
        window,
        grid_step_board,
        route_layers,
        own_nets,
        clearance_class,
        via_clearance_class,
        0.0,
    )
}

/// Convenience form for the frozen native adapter call contract.
pub fn build_obstacle_map_parts(
    board: &BoardAdapter,
    request: &RungRequest,
    window: IntBox,
    grid_step_board: i32,
    route_layers: &[String],
    own_nets: &[NetNo],
    clearance_class: i32,
    via_clearance_class: i32,
) -> Result<(GridObstacleMap, Vec<i32>), String> {
    let result = build_obstacle_map(
        board,
        request,
        window,
        grid_step_board,
        route_layers,
        own_nets,
        clearance_class,
        via_clearance_class,
    )?;
    Ok((result.obstacles, result.layer_numbers))
}

#[cfg(test)]
mod capsule_regression {
    #[test]
    fn off_grid_diagonal_uses_strict_capsule_boundary() {
        let a = [0.25, 0.1875];
        let b = [4.25, 3.1875];
        // Grid cell center (1, 2) is exactly 1 native unit from this
        // off-grid 3:4 diagonal. The source predicate is d² < (margin-eps)².
        let boundary = crate::awx::native_via_obstacles::capsule_cells(a, b, 1.0 + 1e-6, 1, 1.0);
        assert!(!boundary.contains(&[1, 2]));
        let strictly_inside =
            crate::awx::native_via_obstacles::capsule_cells(a, b, 1.0 + 3e-6, 1, 1.0);
        assert!(strictly_inside.contains(&[1, 2]));
    }
}
