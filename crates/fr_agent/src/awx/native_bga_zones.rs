//! Stamp only explicitly configured AWX BGA exclusion rectangles.
//!
//! The fixed AWX path calls `connect.make_config` with the native route
//! dimensions and no `bga_exclusion_zones`; its `GridRouteConfig` therefore
//! keeps the empty default. This helper deliberately does not infer packages
//! from BoardAdapter pins or native DSN items.
use crate::{
    awx::native_obstacles::{grid_round, NativeGridConfig},
    board_adapter::BoardAdapter,
};
use awx_grid::obstacle_map::GridObstacleMap;

/// Apply configured BGA zones using the source `GridCoord.to_grid` rounding
/// and `GridObstacleMap.set_bga_zone` semantics. The board parameter stays in
/// the adapter contract; zones are already expressed in its native units.
pub fn add_bga_zones(
    map: &mut GridObstacleMap,
    _board: &BoardAdapter,
    config: &NativeGridConfig,
) -> Result<(), String> {
    if config.grid_step <= 0 {
        return Err("BGA zone stamping requires a positive native grid step".into());
    }
    for (index, zone) in config.bga_exclusion_zones.iter().enumerate() {
        let [min_x, min_y, max_x, max_y] = *zone;
        if min_x > max_x || min_y > max_y {
            return Err(format!("invalid configured BGA exclusion zone {index}"));
        }
        map.set_bga_zone(
            grid_round(min_x, config.grid_step),
            grid_round(min_y, config.grid_step),
            grid_round(max_x, config.grid_step),
            grid_round(max_y, config.grid_step),
        );
    }
    Ok(())
}
