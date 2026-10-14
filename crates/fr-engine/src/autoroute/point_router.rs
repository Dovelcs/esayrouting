//! Thin Rust bridge to the source AWX grid and pose routers.
//!
//! Board geometry is imported by `fr_agent::awx::native_obstacles`; this
//! module deliberately contains no alternate search implementation.

pub use awx_grid::{GridObstacleMap, GridRouter, PoseRouter, TrackMarginArg};

/// The source `GridRouteConfig` values used by `connect.py::make_config`.
#[derive(Clone, Debug)]
pub struct SourceGridConfig {
    pub via_cost: i32,
    pub heuristic_weight: f32,
    pub turn_cost: i32,
    pub via_proximity_cost: i32,
    pub vertical_attraction_radius: i32,
    pub vertical_attraction_bonus: i32,
    pub layer_costs: Vec<i32>,
    pub proximity_heuristic_cost: i32,
    pub layer_direction_preferences: Vec<u8>,
    pub direction_preference_cost: i32,
}

impl SourceGridConfig {
    /// Convert source millimetre cost units to the grid's 1000-cost unit.
    pub fn from_source(
        grid_step_mm: f64,
        via_cost: i32,
        layer_count: usize,
        route_layer_indices: &[usize],
        layer_costs: Option<Vec<i32>>,
        direction_preferences: Option<Vec<u8>>,
    ) -> Result<Self, String> {
        if !grid_step_mm.is_finite() || grid_step_mm <= 0.0 || via_cost < 0 {
            return Err("invalid source grid-router scale or via cost".into());
        }
        let mut costs = layer_costs.unwrap_or_else(|| vec![1000; layer_count]);
        if costs.len() < layer_count {
            costs.resize(layer_count, 1000);
        }
        for layer in 0..layer_count {
            if !route_layer_indices.contains(&layer) {
                costs[layer] = -1;
            }
        }
        let mut preferences = direction_preferences.unwrap_or_else(|| {
            (0..layer_count)
                .map(|i| if i % 2 == 0 { 0 } else { 1 })
                .collect()
        });
        preferences.resize(layer_count, 255);
        Ok(Self {
            via_cost: source_via_cost_units(via_cost, grid_step_mm),
            heuristic_weight: 2.3,
            turn_cost: 1000,
            via_proximity_cost: 10,
            vertical_attraction_radius: 0,
            vertical_attraction_bonus: 0,
            layer_costs: costs,
            proximity_heuristic_cost: 0,
            layer_direction_preferences: preferences,
            direction_preference_cost: 250,
        })
    }

    pub fn router(&self) -> GridRouter {
        GridRouter::new(
            self.via_cost,
            self.heuristic_weight,
            Some(self.turn_cost),
            Some(self.via_proximity_cost),
            self.vertical_attraction_radius,
            self.vertical_attraction_bonus,
            Some(self.layer_costs.clone()),
            Some(self.proximity_heuristic_cost),
            Some(self.layer_direction_preferences.clone()),
            self.direction_preference_cost,
            0,
            0,
            0,
            0,
        )
    }
}

/// `routing_config.py::via_cost_units`: costs are normalized from the source
/// 0.1 mm grid to the active grid, then expressed in the kernel's 1000 units.
pub fn source_via_cost_units(via_cost_mm: i32, grid_step_mm: f64) -> i32 {
    ((via_cost_mm as f64 * 1000.0 * (0.1 / grid_step_mm)).trunc()) as i32
}

/// Run the original GridRouter search and retain its exact blocked frontier.
#[allow(clippy::too_many_arguments)]
pub fn route_with_frontier(
    router: &GridRouter,
    obstacles: &GridObstacleMap,
    sources: Vec<(i32, i32, u8)>,
    targets: Vec<(i32, i32, u8)>,
    max_iterations: u32,
    track_margin: TrackMarginArg,
    start_direction: Option<(i32, i32)>,
    end_direction: Option<(f64, f64)>,
) -> (Option<Vec<(i32, i32, u8)>>, u32, Vec<(i32, i32, u8)>) {
    router.route_with_frontier(
        obstacles,
        sources,
        targets,
        max_iterations,
        false,
        0,
        start_direction,
        end_direction,
        8,
        track_margin,
        10_000_000,
        2.0,
        2.0,
        0,
        0,
    )
}

#[cfg(test)]
mod tests {
    use super::source_via_cost_units;

    #[test]
    fn source_via_cost_tracks_grid_scale_and_pair_cost() {
        assert_eq!(source_via_cost_units(75, 0.1), 75_000);
        assert_eq!(source_via_cost_units(75, 0.05), 150_000);
        assert_eq!(source_via_cost_units(150, 0.1), 150_000);
    }
}
