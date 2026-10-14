//! Window fencing and reusable same-net via sites from the source grid router.

use super::native_obstacles::NativeGridConfig;
use crate::board_adapter::BoardAdapter;
use awx_grid::obstacle_map::GridObstacleMap;
use fr_engine::ids::NetNo;
use fr_geom::IntBox;
use std::collections::BTreeSet;

/// Explicit pad drill evidence carried from LogicalPad/ContextPad through the
/// native routing request. Coordinates are global native board units.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FreeViaPad {
    pub net_id: NetNo,
    pub point_board: [i32; 2],
    pub drill_board: i32,
    pub non_plated_hole: bool,
    pub drill_axis_board: Option<[[i32; 2]; 2]>,
}

fn has_outline(board: &BoardAdapter) -> bool {
    board
        .board
        .get_outline()
        .and_then(|key| board.board.item(key).as_board_outline())
        .is_some_and(|outline| outline.shape_count() > 0)
}

fn distance_floor(distance: f64, step: i32) -> Result<i32, String> {
    let cells = (distance / step as f64).trunc();
    if !cells.is_finite() || cells < 0.0 || cells > i32::MAX as f64 {
        return Err("native window track edge expansion is outside grid range".into());
    }
    Ok(cells as i32)
}

fn distance_ceil(distance: f64, step: i32) -> Result<i32, String> {
    let cells = (distance / step as f64).ceil();
    if !cells.is_finite() || cells < 0.0 || cells > i32::MAX as f64 {
        return Err("native window via edge expansion is outside grid range".into());
    }
    Ok(cells as i32)
}

fn checked_add(a: i32, b: i32) -> Result<i32, String> {
    a.checked_add(b)
        .ok_or_else(|| "native window edge grid coordinate overflow".into())
}

fn checked_sub(a: i32, b: i32) -> Result<i32, String> {
    a.checked_sub(b)
        .ok_or_else(|| "native window edge grid coordinate overflow".into())
}

fn round_ties_even_grid(coordinate: f64, step: i32) -> Result<i32, String> {
    if step <= 0 || !coordinate.is_finite() {
        return Err("invalid native free-via coordinate/grid step".into());
    }
    let cell = (coordinate / step as f64).round_ties_even();
    if cell < i32::MIN as f64 || cell > i32::MAX as f64 {
        return Err("native free-via grid coordinate is outside range".into());
    }
    Ok(cell as i32)
}

fn block_edge_band(
    map: &mut GridObstacleMap,
    layers: usize,
    x_range: (i32, i32),
    y_range: (i32, i32),
    track_predicate: impl Fn(i32, i32) -> bool,
    via_predicate: impl Fn(i32, i32) -> bool,
) {
    if x_range.0 > x_range.1 || y_range.0 > y_range.1 {
        return;
    }
    for gx in x_range.0..=x_range.1 {
        for gy in y_range.0..=y_range.1 {
            if track_predicate(gx, gy) {
                for layer in 0..layers {
                    map.add_static_blocked_cell(gx, gy, layer);
                }
            }
            if via_predicate(gx, gy) {
                map.add_static_blocked_via(gx, gy);
            }
        }
    }
}

/// Match net_rescue._fence_window for a polygonal board's stamped local window.
/// Rectangular boards are already fenced by the base map and are left untouched.
pub fn fence_window(
    map: &mut GridObstacleMap,
    board: &BoardAdapter,
    window: IntBox,
    config: &NativeGridConfig,
) -> Result<(), String> {
    if !has_outline(board) {
        return Ok(());
    }
    let step = config.grid_step;
    if step <= 0 {
        return Err("native window grid step must be positive".into());
    }
    if map.num_layers != config.route_layers.len()
        || config.route_layers.len() != config.layer_numbers.len()
    {
        return Err("native window layer order disagrees with obstacle map".into());
    }

    let edge_clearance = if config.board_edge_clearance > 0 {
        config.board_edge_clearance
    } else {
        config.clearance
    };
    let track_expand = distance_floor(
        edge_clearance as f64 + config.track_width as f64 / 2.0,
        step,
    )?;
    let via_expand = distance_ceil(edge_clearance as f64 + config.via_size as f64 / 2.0, step)?;
    let edge_expand = track_expand.max(via_expand);
    let margin = checked_add(edge_expand, 5)?;
    let gmin_x = super::native_obstacles::grid_round(window.ll.x, step);
    let gmin_y = super::native_obstacles::grid_round(window.ll.y, step);
    let gmax_x = super::native_obstacles::grid_round(window.ur.x, step);
    let gmax_y = super::native_obstacles::grid_round(window.ur.y, step);
    let min_track_x = checked_add(gmin_x, track_expand)?;
    let min_track_y = checked_add(gmin_y, track_expand)?;
    let max_track_x = checked_sub(gmax_x, track_expand)?;
    let max_track_y = checked_sub(gmax_y, track_expand)?;
    let min_via_x = checked_add(gmin_x, via_expand)?;
    let min_via_y = checked_add(gmin_y, via_expand)?;
    let max_via_x = checked_sub(gmax_x, via_expand)?;
    let max_via_y = checked_sub(gmax_y, via_expand)?;

    // Left and right sweeps cover the full height, including both corners.
    block_edge_band(
        map,
        config.route_layers.len(),
        (
            checked_sub(gmin_x, margin)?,
            checked_add(gmin_x, edge_expand)?,
        ),
        (checked_sub(gmin_y, margin)?, checked_add(gmax_y, margin)?),
        |gx, _| gx <= min_track_x,
        |gx, _| gx < min_via_x,
    );
    block_edge_band(
        map,
        config.route_layers.len(),
        (
            checked_sub(gmax_x, edge_expand)?,
            checked_add(gmax_x, margin)?,
        ),
        (checked_sub(gmin_y, margin)?, checked_add(gmax_y, margin)?),
        |gx, _| gx >= max_track_x,
        |gx, _| gx > max_via_x,
    );
    // Top and bottom sweeps omit the left/right track bands; those were
    // already covered by the full-height sweeps above.
    let middle_x = (
        checked_add(checked_add(gmin_x, track_expand)?, 1)?,
        checked_sub(checked_sub(gmax_x, track_expand)?, 1)?,
    );
    block_edge_band(
        map,
        config.route_layers.len(),
        middle_x,
        (
            checked_sub(gmin_y, margin)?,
            checked_add(gmin_y, edge_expand)?,
        ),
        |_, gy| gy <= min_track_y,
        |_, gy| gy < min_via_y,
    );
    block_edge_band(
        map,
        config.route_layers.len(),
        middle_x,
        (
            checked_sub(gmax_y, edge_expand)?,
            checked_add(gmax_y, margin)?,
        ),
        |_, gy| gy >= max_track_y,
        |_, gy| gy > max_via_y,
    );
    Ok(())
}

fn owned_via_positions(
    board: &BoardAdapter,
    own_nets: &[NetNo],
    grid_step_board: i32,
) -> Result<BTreeSet<(i32, i32)>, String> {
    if grid_step_board <= 0 {
        return Err("native free-via grid step must be positive".into());
    }
    let own: BTreeSet<_> = own_nets.iter().copied().collect();
    let mut positions = BTreeSet::new();
    for key in board.board.get_vias() {
        let item = board.board.item(key);
        if !item.net_numbers().iter().any(|number| own.contains(number)) {
            continue;
        }
        let center = item.center(&board.board).to_float();
        positions.insert((
            round_ties_even_grid(center.x, grid_step_board)?,
            round_ties_even_grid(center.y, grid_step_board)?,
        ));
    }
    Ok(positions)
}

/// Source routing_context._add_free_via_positions: only explicit plated pads
/// with a real drill are free pad sites, plus existing same-net via centers.
pub fn free_via_positions(
    board: &BoardAdapter,
    own_nets: &[NetNo],
    explicit_pads: &[FreeViaPad],
    grid_step_board: i32,
) -> Result<BTreeSet<(i32, i32)>, String> {
    if grid_step_board <= 0 {
        return Err("native free-via grid step must be positive".into());
    }
    let own: BTreeSet<_> = own_nets.iter().copied().collect();
    if own.is_empty() {
        return Ok(BTreeSet::new());
    }
    let mut positions = owned_via_positions(board, own_nets, grid_step_board)?;
    for pad in explicit_pads {
        if !own.contains(&pad.net_id) || pad.drill_board <= 0 || pad.non_plated_hole {
            continue;
        }
        positions.insert((
            super::native_obstacles::grid_round(pad.point_board[0], grid_step_board),
            super::native_obstacles::grid_round(pad.point_board[1], grid_step_board),
        ));
    }
    Ok(positions)
}

/// Source get_same_net_through_hole_positions, used when route conversion
/// suppresses an emitted via at copper centers or offset drill-barrel centers.
pub fn same_net_through_hole_positions(
    board: &BoardAdapter,
    own_nets: &[NetNo],
    explicit_pads: &[FreeViaPad],
    grid_step_board: i32,
) -> Result<BTreeSet<(i32, i32)>, String> {
    if grid_step_board <= 0 {
        return Err("native through-hole grid step must be positive".into());
    }
    let own: BTreeSet<_> = own_nets.iter().copied().collect();
    let mut positions = owned_via_positions(board, own_nets, grid_step_board)?;
    for pad in explicit_pads {
        if !own.contains(&pad.net_id) || pad.drill_board <= 0 || pad.non_plated_hole {
            continue;
        }
        positions.insert((
            super::native_obstacles::grid_round(pad.point_board[0], grid_step_board),
            super::native_obstacles::grid_round(pad.point_board[1], grid_step_board),
        ));
        if let Some(axis) = pad.drill_axis_board {
            let hole = [
                (axis[0][0] as f64 + axis[1][0] as f64) / 2.0,
                (axis[0][1] as f64 + axis[1][1] as f64) / 2.0,
            ];
            positions.insert((
                round_ties_even_grid(hole[0], grid_step_board)?,
                round_ties_even_grid(hole[1], grid_step_board)?,
            ));
        }
    }
    Ok(positions)
}

/// Add the source routing-context free sites to the obstacle map.
pub fn add_free_via_positions(
    map: &mut GridObstacleMap,
    board: &BoardAdapter,
    own_nets: &[NetNo],
    explicit_pads: &[FreeViaPad],
    grid_step_board: i32,
) -> Result<(), String> {
    let positions = free_via_positions(board, own_nets, explicit_pads, grid_step_board)?;
    if !positions.is_empty() {
        map.add_free_vias_batch(positions.into_iter().collect());
    }
    Ok(())
}
