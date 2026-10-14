//! Native adapter for source `Corridor.connect_ladder` point routes.

use super::{
    braid_connect::{RungBandConstraint, RungEndpoint, RungExecution, RungExecutor, RungRequest},
    native_execution::{self, NativeGrade},
};
use crate::board_adapter::BoardAdapter;
use awx_grid::{GridObstacleMap, GridRouter, TrackMarginArg};
use fr_engine::{
    autoroute::{point_router, AutorouteControl},
    ids::{FixedState, NetNo},
    rules::{via::ViaInfoId, ViaRule},
};
use fr_geom::{FloatPoint, IntBox, IntPoint, Point as EnginePoint, Polyline};
use fr_settings::RouterSettings;
use std::{
    collections::{BTreeMap, BTreeSet},
    time::Instant,
};

#[derive(Clone)]
pub struct NativeRungRoute {
    /// The request board plus the landed copper, still isolated from the caller.
    pub board: BoardAdapter,
    pub achieved: RungEndpoint,
    /// P/N target readback for a coupled lane.
    pub achieved_pair: Option<[RungEndpoint; 2]>,
    pub grade: NativeGrade,
    /// Exact global AWX cells returned by the source grid search. The path is
    /// retained for rip_for cut ordering and audit alongside the board delta.
    pub grid_path: Vec<(i32, i32, usize)>,
    pub grid_step_mm: f64,
    pub route_segments: Vec<super::pair_connection::PairPiece>,
    pub route_vias: Vec<super::pair_connection::PairVia>,
}

#[derive(Default)]
pub struct NativeRungExecutor;

struct NativeTapBackend<'a> {
    board: &'a BoardAdapter,
}

impl NativeTapBackend<'_> {
    fn foreign_items(&self, rung: &RungRequest) -> Vec<fr_engine::board::ItemKey> {
        self.board
            .board
            .get_items()
            .into_iter()
            .filter(|&key| {
                let item = self.board.board.item(key);
                item.is_on_board()
                    && !item.contains_net(rung.net_id)
                    && matches!(
                        item.board_item_type(),
                        fr_engine::board::BoardItemType::Trace
                            | fr_engine::board::BoardItemType::Pin
                            | fr_engine::board::BoardItemType::Via
                    )
            })
            .collect()
    }
}

impl super::via_pad_tap::TapBackend for NativeTapBackend<'_> {
    fn foreign_item_distance_mm(
        &mut self,
        board: &BoardAdapter,
        rung: &RungRequest,
        point: [f64; 2],
        layer: &str,
        item_index: usize,
    ) -> Result<Option<f64>, String> {
        let keys = self.foreign_items(rung);
        let Some(&key) = keys.get(item_index) else {
            return Ok(None);
        };
        let item = board.board.item(key);
        if item.board_item_type() == fr_engine::board::BoardItemType::Via {
            let center = item.center(&board.board).to_float();
            return Ok(Some(
                (center.x / board.units_per_mm - point[0])
                    .hypot(center.y / board.units_per_mm - point[1]),
            ));
        }
        let layer_no = board.board.layer_structure.get_no(layer);
        if layer_no < 0 {
            return Err(format!("tap refit uses unknown layer {layer}"));
        }
        let p = FloatPoint::new(point[0] * board.units_per_mm, point[1] * board.units_per_mm);
        let mut best = f64::INFINITY;
        for (index, shape) in board.board.item_tile_shapes(item).iter().enumerate() {
            if board.board.shape_layer(key, index as i32) != layer_no {
                continue;
            }
            if let Some(shape) = shape {
                best = best.min(shape.distance(&p) / board.units_per_mm);
            }
        }
        Ok(best.is_finite().then_some(best))
    }

    fn foreign_item_kind(
        &self,
        board: &BoardAdapter,
        rung: &RungRequest,
        item_index: usize,
    ) -> super::via_pad_tap::TapForeignKind {
        let keys = self.foreign_items(rung);
        if keys.get(item_index).is_some_and(|&key| {
            board.board.item(key).board_item_type() == fr_engine::board::BoardItemType::Via
        }) {
            super::via_pad_tap::TapForeignKind::Via
        } else {
            super::via_pad_tap::TapForeignKind::SegmentOrPad
        }
    }

    fn foreign_via_diameter_mm(
        &mut self,
        board: &BoardAdapter,
        rung: &RungRequest,
        item_index: usize,
    ) -> Result<f64, String> {
        let keys = self.foreign_items(rung);
        let Some(&key) = keys.get(item_index) else {
            return Err("tap via item index is out of range".into());
        };
        let item = board.board.item(key);
        let pad = item
            .padstack(&board.board)
            .ok_or("foreign tap via has no padstack")?;
        let diameter = (0..board.board.layer_count() as i32)
            .filter_map(|layer| pad.get_shape(layer))
            .map(|shape| shape.max_width())
            .fold(0.0_f64, f64::max);
        Ok(diameter / board.units_per_mm)
    }

    fn foreign_item_count(&self, board: &BoardAdapter, rung: &RungRequest, _layer: &str) -> usize {
        let _ = board;
        self.foreign_items(rung).len()
    }

    fn pair_clearance_mm(&self, rung: &RungRequest, item_index: usize, layer: Option<&str>) -> f64 {
        let own = self
            .board
            .board
            .rules
            .net_class_of(rung.net_id)
            .get_trace_clearance_class();
        let keys = self.foreign_items(rung);
        let mut max = 0i32;
        let layers: Vec<i32> = if let Some(name) = layer {
            let no = self.board.board.layer_structure.get_no(name);
            if no < 0 {
                return rung.rules.clearance;
            };
            vec![no]
        } else {
            // The native LayerStructure is the concrete DSN copper stack,
            // including power planes and Top/Bottom-named layers.
            (0..self.board.board.layer_count()).collect()
        };
        let classes: Vec<i32> = if item_index == usize::MAX {
            keys.iter()
                .map(|&k| self.board.board.item(k).clearance_class())
                .collect()
        } else {
            keys.get(item_index)
                .map(|&k| vec![self.board.board.item(k).clearance_class()])
                .unwrap_or_default()
        };
        for class in classes {
            for &lay in &layers {
                max = max
                    .max(self.board.board.clearance_value(own, class, lay))
                    .max(self.board.board.clearance_value(class, own, lay));
            }
        }
        (max as f64 / self.board.units_per_mm).max(rung.rules.clearance)
    }

    fn grid_cell(
        &self,
        board: &BoardAdapter,
        point: [f64; 2],
        grid_step_mm: f64,
    ) -> Result<(i32, i32), String> {
        let step = grid_step_mm * board.units_per_mm;
        let x = (point[0] * board.units_per_mm / step).round_ties_even();
        let y = (point[1] * board.units_per_mm / step).round_ties_even();
        if !x.is_finite()
            || !y.is_finite()
            || x < i32::MIN as f64
            || x > i32::MAX as f64
            || y < i32::MIN as f64
            || y > i32::MAX as f64
        {
            return Err("tap pad cell is outside native grid range".into());
        }
        Ok((x as i32, y as i32))
    }

    fn board_edge_signed_distance_mm(
        &self,
        board: &BoardAdapter,
        point: [f64; 2],
    ) -> Result<f64, String> {
        if point.iter().any(|value| !value.is_finite()) {
            return Err("nonfinite Edge.Cuts query point".into());
        }
        let rings: Vec<&Vec<[i32; 2]>> = board
            .board_outline_polygons
            .iter()
            .chain(board.board_cutouts.iter())
            .chain(board.board_edge_contours.iter())
            .filter(|ring| ring.len() >= 2)
            .collect();
        if rings.is_empty() {
            return Ok(f64::INFINITY);
        }
        let distance = rings
            .iter()
            .flat_map(|ring| {
                (0..ring.len()).map(move |index| {
                    point_segment_distance_mm(
                        point,
                        [
                            ring[index][0] as f64 / board.units_per_mm,
                            ring[index][1] as f64 / board.units_per_mm,
                        ],
                        [
                            ring[(index + 1) % ring.len()][0] as f64 / board.units_per_mm,
                            ring[(index + 1) % ring.len()][1] as f64 / board.units_per_mm,
                        ],
                    )
                })
            })
            .fold(f64::INFINITY, f64::min);
        let in_outline = board
            .board_outline_polygons
            .iter()
            .any(|ring| point_in_ring_mm(point, ring, board.units_per_mm));
        let in_cutout = board
            .board_cutouts
            .iter()
            .any(|ring| point_in_ring_mm(point, ring, board.units_per_mm));
        Ok(if in_outline && !in_cutout {
            distance
        } else {
            -distance
        })
    }
}

fn point_in_ring_mm(point: [f64; 2], ring: &[[i32; 2]], units_per_mm: f64) -> bool {
    let mut inside = false;
    let mut previous = ring.len().saturating_sub(1);
    for index in 0..ring.len() {
        let a = [
            ring[index][0] as f64 / units_per_mm,
            ring[index][1] as f64 / units_per_mm,
        ];
        let b = [
            ring[previous][0] as f64 / units_per_mm,
            ring[previous][1] as f64 / units_per_mm,
        ];
        if (a[1] > point[1]) != (b[1] > point[1])
            && point[0] < (b[0] - a[0]) * (point[1] - a[1]) / (b[1] - a[1]) + a[0]
        {
            inside = !inside;
        }
        previous = index;
    }
    inside
}

fn point_segment_distance_mm(point: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let delta = [b[0] - a[0], b[1] - a[1]];
    let length_sq = delta[0] * delta[0] + delta[1] * delta[1];
    let t = if length_sq <= 0.0 {
        0.0
    } else {
        (((point[0] - a[0]) * delta[0] + (point[1] - a[1]) * delta[1]) / length_sq).clamp(0.0, 1.0)
    };
    (point[0] - a[0] - t * delta[0]).hypot(point[1] - a[1] - t * delta[1])
}

struct GridSearchResult {
    path: Option<Vec<(i32, i32, u8)>>,
    iterations: usize,
    frontier: Vec<(i32, i32, u8)>,
    window: IntBox,
    grid_step_board: i32,
    layer_numbers: Vec<i32>,
    registered_via_sizes: BTreeMap<(i32, i32), (f64, f64)>,
    forward_probe_iterations: usize,
    backward_probe_iterations: usize,
    via_refit_rungs: Vec<(f64, f64)>,
    /// Source `_route_with_via_unblock` retains its exact working map for the
    /// optional in-pad rescue attempt after both ordinary and #568 searches.
    obstacles: GridObstacleMap,
    router_config: fr_engine::autoroute::point_router::SourceGridConfig,
    native_config: super::native_obstacles::NativeGridConfig,
    source_cells: Vec<(i32, i32, u8)>,
    target_cells: Vec<(i32, i32, u8)>,
}

struct SourceRouteAttempt {
    path: Option<Vec<(i32, i32, u8)>>,
    iterations: usize,
    frontier: Vec<(i32, i32, u8)>,
    forward_probe_iterations: usize,
    backward_probe_iterations: usize,
}

static ENDPOINT_GRID_CALLS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

fn route_core(
    router: &GridRouter,
    obstacles: &GridObstacleMap,
    sources: Vec<(i32, i32, u8)>,
    targets: Vec<(i32, i32, u8)>,
    budget: usize,
    via_exclusion_radius: i32,
    via_rung: usize,
    dynamic: bool,
) -> (Option<Vec<(i32, i32, u8)>>, usize, Vec<(i32, i32, u8)>) {
    ENDPOINT_GRID_CALLS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let budget = budget.min(u32::MAX as usize) as u32;
    let ceiling = if dynamic { 10_000_000 } else { budget };
    let (path, iterations, frontier) = router.route_with_frontier(
        obstacles,
        sources,
        targets,
        budget,
        false,
        via_exclusion_radius,
        None,
        None,
        8,
        TrackMarginArg::Scalar(0.0),
        ceiling,
        2.0,
        2.0,
        0,
        via_rung,
    );
    (path, iterations as usize, frontier)
}

/// Source `_probe_route_with_frontier` policy: forward/backward 5k probes;
/// only run dynamic full searches if both probes exhaust their budgets. It
/// also retries a connected path once with the source via-hole spacing guard.
fn source_route_attempt(
    router: &GridRouter,
    obstacles: &GridObstacleMap,
    sources: &[(i32, i32, u8)],
    targets: &[(i32, i32, u8)],
    max_iterations: usize,
    via_rung: usize,
    via_drill_mm: f64,
    hole_to_hole_mm: f64,
    grid_step_mm: f64,
) -> SourceRouteAttempt {
    fn once(
        router: &GridRouter,
        obstacles: &GridObstacleMap,
        sources: &[(i32, i32, u8)],
        targets: &[(i32, i32, u8)],
        max_iterations: usize,
        via_rung: usize,
        via_exclusion_radius: i32,
    ) -> SourceRouteAttempt {
        let probe_budget = 5_000usize;
        // Source `_dynamic_iterations: config.max_iterations <= 10_000` keeps
        // the static cap; only larger budgets earn the 1e7 dynamic ceiling.
        let dynamic = max_iterations > 10_000;
        let (forward, fi, mut frontier) = route_core(
            router,
            obstacles,
            sources.to_vec(),
            targets.to_vec(),
            probe_budget,
            via_exclusion_radius,
            via_rung,
            false,
        );
        if forward.is_some() {
            return SourceRouteAttempt {
                path: forward,
                iterations: fi,
                frontier: Vec::new(),
                forward_probe_iterations: fi,
                backward_probe_iterations: 0,
            };
        }
        let (backward, bi, back_frontier) = route_core(
            router,
            obstacles,
            targets.to_vec(),
            sources.to_vec(),
            probe_budget,
            via_exclusion_radius,
            via_rung,
            false,
        );
        frontier.extend(back_frontier);
        if let Some(mut path) = backward {
            path.reverse();
            return SourceRouteAttempt {
                path: Some(path),
                iterations: fi + bi,
                frontier: Vec::new(),
                forward_probe_iterations: fi,
                backward_probe_iterations: bi,
            };
        }
        if fi < probe_budget || bi < probe_budget {
            return SourceRouteAttempt {
                path: None,
                iterations: fi + bi,
                frontier,
                forward_probe_iterations: fi,
                backward_probe_iterations: bi,
            };
        }
        let (full_forward, ff, mut full_frontier) = route_core(
            router,
            obstacles,
            sources.to_vec(),
            targets.to_vec(),
            max_iterations,
            via_exclusion_radius,
            via_rung,
            dynamic,
        );
        if full_forward.is_some() {
            return SourceRouteAttempt {
                path: full_forward,
                iterations: fi + bi + ff,
                frontier: Vec::new(),
                forward_probe_iterations: fi,
                backward_probe_iterations: bi,
            };
        }
        let (full_backward, fb, back_frontier) = route_core(
            router,
            obstacles,
            targets.to_vec(),
            sources.to_vec(),
            max_iterations,
            via_exclusion_radius,
            via_rung,
            dynamic,
        );
        full_frontier.extend(back_frontier);
        if let Some(mut path) = full_backward {
            path.reverse();
            SourceRouteAttempt {
                path: Some(path),
                iterations: fi + bi + ff + fb,
                frontier: Vec::new(),
                forward_probe_iterations: fi,
                backward_probe_iterations: bi,
            }
        } else {
            SourceRouteAttempt {
                path: None,
                iterations: fi + bi + ff + fb,
                frontier: full_frontier,
                forward_probe_iterations: fi,
                backward_probe_iterations: bi,
            }
        }
    }

    let first = once(
        router,
        obstacles,
        sources,
        targets,
        max_iterations,
        via_rung,
        0,
    );
    let Some(path) = first.path.as_ref() else {
        return first;
    };
    if hole_to_hole_mm <= 0.0
        || grid_step_mm <= 0.0
        || !path_has_close_vias(path, via_drill_mm, hole_to_hole_mm, grid_step_mm)
    {
        return first;
    }
    let exclusion = (((via_drill_mm + hole_to_hole_mm) / grid_step_mm / 2.0).ceil() as i32).max(1);
    let retry = once(
        router,
        obstacles,
        sources,
        targets,
        max_iterations,
        via_rung,
        exclusion,
    );
    if retry.path.is_some() {
        retry
    } else {
        first
    }
}

fn path_has_close_vias(path: &[(i32, i32, u8)], drill: f64, clearance: f64, step: f64) -> bool {
    if clearance <= 0.0 || step <= 0.0 {
        return false;
    }
    let mut cells = Vec::new();
    for pair in path.windows(2) {
        if pair[0].0 == pair[1].0 && pair[0].1 == pair[1].1 && pair[0].2 != pair[1].2 {
            let cell = (pair[0].0, pair[0].1);
            if !cells.contains(&cell) {
                cells.push(cell);
            }
        }
    }
    let minimum = ((drill + clearance) / step).powi(2);
    for i in 0..cells.len() {
        for j in 0..i {
            let dx = (cells[i].0 - cells[j].0) as f64;
            let dy = (cells[i].1 - cells[j].1) as f64;
            if dx * dx + dy * dy < minimum {
                return true;
            }
        }
    }
    false
}

/// Shared thin call into the fixed-source A* implementation for single-net
/// pair envelope connectors. Obstacle maps and all source policy remain with
/// the caller; this function adds no search behavior of its own.
pub(crate) fn run_source_grid(
    obstacles: &GridObstacleMap,
    grid_step_mm: f64,
    sources: Vec<(i32, i32, u8)>,
    targets: Vec<(i32, i32, u8)>,
    max_iterations: usize,
    track_margin: TrackMarginArg,
    start_direction: Option<(i32, i32)>,
    end_direction: Option<(f64, f64)>,
) -> (Option<Vec<(i32, i32, u8)>>, usize, Vec<(i32, i32, u8)>) {
    let layers = obstacles.blocked_cells.len();
    let route_layers = (0..layers).collect::<Vec<_>>();
    let config = fr_engine::autoroute::point_router::SourceGridConfig::from_source(
        grid_step_mm,
        75,
        layers,
        &route_layers,
        None,
        None,
    )
    .expect("native source grid config uses a positive route grid");
    let router = config.router();
    let (path, iters, frontier) = fr_engine::autoroute::point_router::route_with_frontier(
        &router,
        obstacles,
        sources,
        targets,
        max_iterations.min(u32::MAX as usize) as u32,
        track_margin,
        start_direction,
        end_direction,
    );
    (path, iters as usize, frontier)
}

fn source_grid_search(
    request: &RungRequest,
    board: &BoardAdapter,
    own_nets: &[NetNo],
    soft_tracks: &[super::whole_context::ReservationTube],
    soft_vias: &[super::whole_context::CopperVia],
    soft_cost: Option<f64>,
    clearance_class: i32,
    via_clearance_class: i32,
) -> Result<GridSearchResult, String> {
    let sources = vec![request.start.clone()];
    let mut targets = vec![request.end.clone()];
    targets.extend(request.b_alternatives.iter().cloned());
    source_grid_search_with_endpoints(
        request,
        board,
        own_nets,
        soft_tracks,
        soft_vias,
        soft_cost,
        clearance_class,
        via_clearance_class,
        &sources,
        &targets,
    )
}

#[allow(clippy::too_many_arguments)]
fn source_grid_search_with_endpoints(
    request: &RungRequest,
    board: &BoardAdapter,
    own_nets: &[NetNo],
    soft_tracks: &[super::whole_context::ReservationTube],
    soft_vias: &[super::whole_context::CopperVia],
    soft_cost: Option<f64>,
    clearance_class: i32,
    via_clearance_class: i32,
    source_endpoints: &[RungEndpoint],
    target_endpoints: &[RungEndpoint],
) -> Result<GridSearchResult, String> {
    if request.route_layers.is_empty() || request.max_iterations == 0 {
        return Err("native AWX grid route needs layers and a positive budget".into());
    }
    if source_endpoints.is_empty() || target_endpoints.is_empty() {
        return Err("native AWX grid route needs source and target endpoints".into());
    }
    let upm = board.units_per_mm;
    let grid_step_board = (request.rules.grid_step * upm).round().max(1.0) as i32;
    let source_points = source_endpoints
        .iter()
        .map(|endpoint| match board.point(endpoint.point)? {
            EnginePoint::Int(point) => Ok(point),
            _ => Err("AWX grid source endpoint exceeds native coordinate range".into()),
        })
        .collect::<Result<Vec<_>, String>>()?;
    let target_points = target_endpoints
        .iter()
        .map(|endpoint| match board.point(endpoint.point)? {
            EnginePoint::Int(point) => Ok(point),
            _ => Err("AWX grid target endpoint exceeds native coordinate range".into()),
        })
        .collect::<Result<Vec<_>, String>>()?;
    let window_endpoints = source_endpoints
        .iter()
        .chain(target_endpoints)
        .map(|endpoint| endpoint.point)
        .collect::<Vec<_>>();
    let window = search_window_for_points(
        request,
        &window_endpoints,
        source_points[0],
        target_points[0],
    )?;
    let mut obstacle_result = super::native_obstacles::build_obstacle_map(
        board,
        request,
        window,
        grid_step_board,
        &request.route_layers,
        own_nets,
        clearance_class,
        via_clearance_class,
    )?;
    let mut obstacles = obstacle_result.obstacles;
    let layer_numbers = obstacle_result.layer_numbers;
    let gx0 = (window.ll.x as f64 / grid_step_board as f64).ceil() as i32;
    let gy0 = (window.ll.y as f64 / grid_step_board as f64).ceil() as i32;
    let gx1 = (window.ur.x as f64 / grid_step_board as f64).floor() as i32;
    let gy1 = (window.ur.y as f64 / grid_step_board as f64).floor() as i32;
    fence_grid_window(
        &mut obstacles,
        gx0,
        gy0,
        gx1,
        gy1,
        request.route_layers.len(),
    );
    let source_cells = source_endpoints
        .iter()
        .zip(source_points)
        .filter_map(|(endpoint, point)| {
            request
                .route_layers
                .iter()
                .position(|name| name == &endpoint.layer)
                .map(|layer| {
                    (
                        super::native_obstacles::grid_round(point.x, grid_step_board),
                        super::native_obstacles::grid_round(point.y, grid_step_board),
                        layer as u8,
                    )
                })
        })
        .collect::<Vec<_>>();
    let targets = target_endpoints
        .iter()
        .zip(target_points)
        .filter_map(|(endpoint, point)| {
            request
                .route_layers
                .iter()
                .position(|name| name == &endpoint.layer)
                .map(|layer| {
                    (
                        super::native_obstacles::grid_round(point.x, grid_step_board),
                        super::native_obstacles::grid_round(point.y, grid_step_board),
                        layer as u8,
                    )
                })
        })
        .collect::<Vec<_>>();
    if source_cells.is_empty() || targets.is_empty() {
        return Err("native AWX endpoint candidates have no active route layer".into());
    }
    for &(x, y, l) in source_cells.iter().chain(&targets) {
        obstacles.add_source_target_cell(x, y, l as usize);
    }
    if let Some(band) = request.band.as_deref() {
        for (layer_index, layer_name) in request.route_layers.iter().enumerate() {
            let mut gy = gy0;
            while gy <= gy1 {
                let mut gx = gx0;
                while gx <= gx1 {
                    let p = [
                        gx as f64 * request.rules.grid_step,
                        gy as f64 * request.rules.grid_step,
                    ];
                    if !band.contains(p, layer_name) {
                        obstacles.add_blocked_cell(gx, gy, layer_index);
                    }
                    gx = gx.saturating_add(1);
                }
                gy = gy.saturating_add(1);
                if request.deadline.is_some_and(|d| Instant::now() >= d) {
                    return Err("rung deadline elapsed while rasterizing band".into());
                }
            }
        }
    }
    // Planned corridor reservations are hard virtual foreign copper on normal
    // ladder rungs. Rip probes pass their candidate copper separately below,
    // where source `stamp_soft` adds cost without making it an obstacle.
    for tube in &request.virtual_tracks {
        let Some(layer) = request
            .route_layers
            .iter()
            .position(|name| name == &tube.layer)
        else {
            continue;
        };
        let step_mm = grid_step_board as f64 / upm;
        let cells = awx_grid::soft_copper::walk_line(
            awx_grid::soft_copper::to_grid(tube.a, step_mm),
            awx_grid::soft_copper::to_grid(tube.b, step_mm),
        );
        let radius = ((tube.radius + request.rules.clearance + request.rules.track_width / 2.0)
            / step_mm)
            .ceil() as i32;
        for (x, low, high) in awx_grid::soft_copper::walk_capsule_spans(&cells, radius) {
            for y in low..=high {
                obstacles.add_static_blocked_cell(x, y, layer);
            }
        }
    }
    for via in &request.virtual_vias {
        let step_mm = grid_step_board as f64 / upm;
        let [x, y] = awx_grid::soft_copper::to_grid(via.point, step_mm);
        obstacles.add_static_blocked_via(x, y);
    }
    if let Some(cost) = soft_cost {
        let grid_step_mm = grid_step_board as f64 / upm;
        let tracks = soft_tracks
            .iter()
            .map(|t| {
                let layer = request
                    .route_layers
                    .iter()
                    .position(|n| n == &t.layer)
                    .unwrap_or(usize::MAX);
                awx_grid::soft_copper::SoftTrack {
                    start: t.a,
                    end: t.b,
                    layer,
                    width: t.radius * 2.0,
                }
            })
            .collect::<Vec<_>>();
        let vias = soft_vias
            .iter()
            .map(|v| awx_grid::soft_copper::SoftVia {
                point: v.point,
                size: v.diameter,
            })
            .collect::<Vec<_>>();
        awx_grid::soft_copper::stamp_soft(
            &mut obstacles,
            grid_step_mm,
            request.route_layers.len(),
            request.rules.clearance,
            request.rules.track_width,
            &tracks,
            &vias,
            cost,
        );
    }
    let route_indices = (0..request.route_layers.len()).collect::<Vec<_>>();
    let config = fr_engine::autoroute::point_router::SourceGridConfig::from_source(
        grid_step_board as f64 / upm,
        75,
        request.route_layers.len(),
        &route_indices,
        None,
        None,
    )?;
    let router = config.router();
    let native_config = obstacle_result.config.clone();
    let grid_step_mm = grid_step_board as f64 / upm;
    let mut attempt = source_route_attempt(
        &router,
        &obstacles,
        &source_cells,
        &targets,
        request.max_iterations,
        0,
        request.rules.via_drill,
        request.rules.hole_to_hole_clearance,
        grid_step_mm,
    );
    let mut iterations = attempt.iterations;
    let forward_probe_iterations = attempt.forward_probe_iterations;
    let backward_probe_iterations = attempt.backward_probe_iterations;
    let mut registered_via_sizes = BTreeMap::new();
    if attempt.path.is_none()
        && ((attempt.forward_probe_iterations > 0 && attempt.forward_probe_iterations < 5_000)
            || (attempt.backward_probe_iterations > 0 && attempt.backward_probe_iterations < 5_000))
    {
        if let Some((via_rung, diameter_mm, drill_mm)) =
            small_via_rung(&native_config, request.net_id, upm)
        {
            if obstacles.rung_len(via_rung) > 0 {
                let small = source_route_attempt(
                    &router,
                    &obstacles,
                    &source_cells,
                    &targets,
                    5_000,
                    via_rung,
                    drill_mm,
                    request.rules.hole_to_hole_clearance,
                    grid_step_mm,
                );
                if let Some(path) = small.path.as_ref() {
                    iterations += small.iterations;
                    for pair in path.windows(2) {
                        let (x, y, layer) = pair[0];
                        if layer != pair[1].2 && obstacles.is_via_blocked_rung(x, y, 0) {
                            registered_via_sizes
                                .entry((x, y))
                                .or_insert((diameter_mm, drill_mm));
                        }
                    }
                    attempt = small;
                }
            }
        }
    }
    let via_refit_rungs = native_config
        .fab_via_rungs
        .iter()
        .map(|&(diameter, drill)| (diameter as f64 / upm, drill as f64 / upm))
        .collect();
    Ok(GridSearchResult {
        path: attempt.path,
        iterations,
        frontier: attempt.frontier,
        window,
        grid_step_board,
        layer_numbers,
        registered_via_sizes,
        forward_probe_iterations,
        backward_probe_iterations,
        via_refit_rungs,
        obstacles,
        router_config: config,
        native_config,
        source_cells,
        target_cells: targets,
    })
}

fn small_via_rung(
    config: &super::native_obstacles::NativeGridConfig,
    net: NetNo,
    upm: f64,
) -> Option<(usize, f64, f64)> {
    let (run_diameter, _) = config
        .net_via_sizes
        .get(&net)
        .copied()
        .unwrap_or((config.via_size, config.via_drill));
    let candidate = config
        .fab_via_rungs
        .iter()
        .copied()
        .find(|(diameter, _)| *diameter < run_diameter)?;
    let index = config
        .ordered_via_rungs
        .iter()
        .position(|pair| *pair == candidate)?
        + 1;
    Some((index, candidate.0 as f64 / upm, candidate.1 as f64 / upm))
}

/// Place one source `_place_shrunk_via_in_pad` tap against a specific pad and,
/// on success, apply its `_register_unblock_via` effect to the retained working
/// map: free-via, source/target override on every route layer, and the 11x11
/// allowed block. The via and stub copper stay uncommitted in the returned tap.
#[allow(clippy::too_many_arguments)]
fn place_unblock_tap(
    request: &RungRequest,
    outcome: &mut GridSearchResult,
    pad: &super::whole_context::ContextPad,
    fab_candidates: &[super::via_pad_tap::TapViaCandidate],
    escape_radius_mm: f64,
    edge_clearance: f64,
    inflight_vias: &mut Vec<super::whole_context::CopperVia>,
    inflight_segments: &mut Vec<super::whole_context::CopperSegment>,
) -> Result<Option<super::via_pad_tap::TapResult>, String> {
    let Some(pad_layer) = pad.layers.iter().find(|layer| {
        !layer.starts_with('*')
            && request.board.board.layer_structure.get_no(layer) >= 0
            && request.route_layers.contains(layer)
    }) else {
        return Ok(None);
    };
    let Some(layer_index) = request
        .route_layers
        .iter()
        .position(|layer| layer == pad_layer)
    else {
        return Ok(None);
    };
    let mut backend = NativeTapBackend {
        board: &request.board,
    };
    let Some(tap) = super::via_pad_tap::place_shrunk_via_in_pad(super::via_pad_tap::TapRequest {
        board: &request.board,
        rung: request,
        pad,
        via_map: &outcome.obstacles,
        route_map: &outcome.obstacles,
        source_config: &outcome.router_config,
        native_config: &outcome.native_config,
        layer_index,
        fab_clearance_floor_mm: request.fab_clearance_floor_mm,
        fab_track_floor_mm: request.fab_track_floor_mm,
        may_narrow: request.may_narrow,
        fab_candidates,
        escape_radius_mm,
        effective_board_edge_clearance_mm: effective_pad_edge_clearance(
            &request.board,
            pad,
            edge_clearance,
        ),
        inflight_vias: inflight_vias.as_slice(),
        inflight_segments: inflight_segments.as_slice(),
        backend: &mut backend,
    })?
    else {
        return Ok(None);
    };
    for release in &tap.map_releases {
        outcome
            .obstacles
            .add_free_via(release.cell.0, release.cell.1);
        for layer in 0..request.route_layers.len() {
            outcome
                .obstacles
                .add_source_target_cell(release.cell.0, release.cell.1, layer);
        }
        outcome.obstacles.add_allowed_rect(
            release.cell.0 - 5,
            release.cell.1 - 5,
            release.cell.0 + 5,
            release.cell.1 + 5,
        );
    }
    inflight_vias.push(tap.via.clone());
    inflight_segments.extend(tap.stub_segments.clone());
    Ok(Some(tap))
}

/// Seed a placed tap's escape-via cell as a source/target on EVERY route layer,
/// matching source `new_sources = list(sources) + [(vgx, vgy, li) for li in
/// range(len(layer_names))]`. Without this the retried A* can never terminate
/// on the via and the whole unblock is inert.
fn extend_with_tap_cells(cells: &mut Vec<(i32, i32, u8)>, tap: &super::via_pad_tap::TapResult, layers: usize) {
    for layer in 0..layers {
        cells.push((tap.grid_cell.0, tap.grid_cell.1, layer as u8));
    }
}

/// Source `_route_with_via_unblock` (#189): a boxed ENDPOINT pad leaves its
/// probe frontier exhausted BELOW the probe limit. Drop a DRC-legal fab-floor
/// via inside that pad, seed the via cell on the correct side, and retry the
/// SAME A* -- first at the probe budget, then (for the other, still-untapped
/// side) at a doubled budget. Vias the retry did not reach are discarded.
fn try_via_pad_unblock(
    request: &RungRequest,
    outcome: &mut GridSearchResult,
) -> Result<Option<(Vec<(i32, i32, u8)>, Vec<super::via_pad_tap::TapResult>)>, String> {
    let lim = 5_000usize;
    let target_stuck =
        outcome.backward_probe_iterations > 0 && outcome.backward_probe_iterations < lim;
    let source_stuck =
        outcome.forward_probe_iterations > 0 && outcome.forward_probe_iterations < lim;
    if (!target_stuck && !source_stuck) || request.route_layers.len() < 2 {
        return Ok(None);
    }
    let source_cells = outcome.source_cells.clone();
    let target_cells = outcome.target_cells.clone();
    let layer_count = request.route_layers.len();

    let mut fab_candidates = request
        .fab_via_rungs
        .iter()
        .enumerate()
        .map(|(i, &(d, h))| super::via_pad_tap::TapViaCandidate {
            diameter_mm: d,
            drill_mm: h,
            escalated: i > 0,
        })
        .collect::<Vec<_>>();
    fab_candidates.dedup_by(|a, b| {
        (a.diameter_mm - b.diameter_mm).abs() < 1e-9 && (a.drill_mm - b.drill_mm).abs() < 1e-9
    });
    let escape_radius_mm = std::env::var("KICAD_ESCAPE_STUB_RADIUS")
        .ok()
        .and_then(|v| v.parse::<f64>().ok())
        .filter(|v| v.is_finite() && *v >= 0.0)
        .unwrap_or(1.0);
    let edge_clearance = if request.rules.board_edge_clearance > 0.0 {
        request.rules.board_edge_clearance
    } else {
        request.rules.clearance
    };
    let mut inflight_vias = request.virtual_vias.clone();
    let mut inflight_segments = request
        .virtual_tracks
        .iter()
        .map(|t| super::whole_context::CopperSegment {
            net: t.net.clone(),
            net_id: request.board.net_number(&t.net).unwrap_or(0) as i32,
            a: t.a,
            b: t.b,
            width: t.radius * 2.0,
            layer: t.layer.clone(),
        })
        .collect::<Vec<_>>();
    let mut router = outcome.router_config.router();
    let mut new_sources = source_cells.clone();
    let mut new_targets = target_cells.clone();
    let mut placed = Vec::<super::via_pad_tap::TapResult>::new();
    let mut source_tapped = false;
    let mut target_tapped = false;

    // backward probe (from targets) exhausted -> the TARGET pad is boxed.
    if target_stuck {
        if let Some(pad) =
            pads_near_grid_cells(request, &target_cells, outcome.grid_step_board)
                .into_iter()
                .next()
        {
            if let Some(tap) = place_unblock_tap(
                request,
                outcome,
                pad,
                &fab_candidates,
                escape_radius_mm,
                edge_clearance,
                &mut inflight_vias,
                &mut inflight_segments,
            )? {
                extend_with_tap_cells(&mut new_targets, &tap, layer_count);
                target_tapped = true;
                placed.push(tap);
            }
        }
    }
    // forward probe (from sources) exhausted -> the SOURCE pad is boxed.
    if source_stuck {
        if let Some(pad) =
            pads_near_grid_cells(request, &source_cells, outcome.grid_step_board)
                .into_iter()
                .next()
        {
            if let Some(tap) = place_unblock_tap(
                request,
                outcome,
                pad,
                &fab_candidates,
                escape_radius_mm,
                edge_clearance,
                &mut inflight_vias,
                &mut inflight_segments,
            )? {
                extend_with_tap_cells(&mut new_sources, &tap, layer_count);
                source_tapped = true;
                placed.push(tap);
            }
        }
    }
    if placed.is_empty() {
        return Ok(None);
    }

    // First retry: cap the full A* at the same probe budget the stuck signal
    // fired on -- if the dropped via opened the pad the route is found within
    // it; grinding past it just wastes time to fail again.
    let mut retry = source_route_attempt(
        &router,
        &outcome.obstacles,
        &new_sources,
        &new_targets,
        5_000,
        0,
        request.rules.via_drill,
        request.rules.hole_to_hole_clearance,
        request.rules.grid_step,
    );
    outcome.iterations += retry.iterations;
    if retry.path.is_none() {
        // Second rung: the stuck side got its via but the retry still failed --
        // the OTHER side often needs layer access too. Tap it (source side
        // first, matching source order) and retry once at a doubled budget.
        let (side_cells, is_source) = if !source_tapped {
            (source_cells.clone(), true)
        } else if !target_tapped {
            (target_cells.clone(), false)
        } else {
            (Vec::new(), false)
        };
        let mut second = false;
        if !side_cells.is_empty() {
            for pad in pads_near_grid_cells(request, &side_cells, outcome.grid_step_board) {
                let Some(tap) = place_unblock_tap(
                    request,
                    outcome,
                    pad,
                    &fab_candidates,
                    escape_radius_mm,
                    edge_clearance,
                    &mut inflight_vias,
                    &mut inflight_segments,
                )?
                else {
                    continue;
                };
                if is_source {
                    extend_with_tap_cells(&mut new_sources, &tap, layer_count);
                } else {
                    extend_with_tap_cells(&mut new_targets, &tap, layer_count);
                }
                placed.push(tap);
                second = true;
                break;
            }
        }
        if second {
            retry = source_route_attempt(
                &router,
                &outcome.obstacles,
                &new_sources,
                &new_targets,
                10_000,
                0,
                request.rules.via_drill,
                request.rules.hole_to_hole_clearance,
                request.rules.grid_step,
            );
            outcome.iterations += retry.iterations;
        }
    }
    let Some(path) = retry.path else {
        return Ok(None);
    };
    // Keep only the vias the retry actually reached: the path must terminate on
    // the via cell at a NON-pad layer (or via an off-pad stub), else it is
    // floating copper.
    let used = placed
        .into_iter()
        .filter(|tap| path_uses_tap(&path, tap))
        .collect::<Vec<_>>();
    Ok(Some((path, used)))
}

fn pads_near_grid_cells<'a>(
    request: &'a RungRequest,
    cells: &[(i32, i32, u8)],
    step_board: i32,
) -> Vec<&'a super::whole_context::ContextPad> {
    let step_mm = step_board as f64 / request.board.units_per_mm;
    let mut found = BTreeMap::<String, (f64, &'a super::whole_context::ContextPad)>::new();
    for &(gx, gy, _) in cells {
        let p = [gx as f64 * step_mm, gy as f64 * step_mm];
        for pad in &request.same_net_pads {
            if pad.net_id != request.net_id as i32
                || pad.drill > 0.0
                || pad.non_plated_hole
                || pad.layers.iter().any(|l| l == "*.Cu")
            {
                continue;
            }
            // SMD means a single native physical copper layer. Counting only
            // route-active layers would let a physically 2-layer pin with one
            // active side (contrary to NativeItem.drill_allowed first==last)
            // masquerade as SMD. Require exactly one native copper layer, THEN
            // that sole layer be an active route layer.
            let native_layers = pad
                .layers
                .iter()
                .filter(|layer| {
                    !layer.starts_with('*')
                        && request.board.board.layer_structure.get_no(layer) >= 0
                })
                .collect::<Vec<_>>();
            if native_layers.len() != 1 || !request.route_layers.contains(native_layers[0]) {
                continue;
            }
            if (pad.point[0] - p[0]).abs() > pad.size[0] / 2.0 + 0.05
                || (pad.point[1] - p[1]).abs() > pad.size[1] / 2.0 + 0.05
            {
                continue;
            }
            let distance = (pad.point[0] - p[0]).abs() + (pad.point[1] - p[1]).abs();
            match found.get(&pad.key) {
                Some((best, _)) if *best <= distance => {}
                _ => {
                    found.insert(pad.key.clone(), (distance, pad));
                }
            }
        }
    }
    let mut ordered = found.into_values().collect::<Vec<_>>();
    ordered.sort_by(|a, b| {
        a.0.total_cmp(&b.0)
            .then_with(|| a.1.point[0].total_cmp(&b.1.point[0]))
            .then_with(|| a.1.point[1].total_cmp(&b.1.point[1]))
            .then_with(|| a.1.pad_number.cmp(&b.1.pad_number))
    });
    ordered.into_iter().map(|(_, pad)| pad).collect()
}

fn path_uses_tap(path: &[(i32, i32, u8)], tap: &super::via_pad_tap::TapResult) -> bool {
    // Source `ends = (p2[0], p2[-1])`: the retry reached the via iff the path
    // terminates on the via cell at a NON-pad layer (it arrived through the
    // barrel) or through an off-pad escape stub.
    path.first()
        .into_iter()
        .chain(path.last())
        .any(|&(x, y, l)| {
            (x, y) == tap.grid_cell
                && (l as usize != tap.pad_layer_index || !tap.stub_segments.is_empty())
        })
}

fn materialize_tap(
    board: &mut BoardAdapter,
    tap: &super::via_pad_tap::TapResult,
    net: NetNo,
) -> Result<(), String> {
    let info_id = select_pair_via_info(board, net, tap.via.diameter, tap.via.drill)?;
    let info = board.board.rules.via_infos[info_id].clone();
    let point = board.point(tap.via.point)?;
    board.board.insert_via(
        info.get_padstack(),
        point,
        &[net],
        info.get_clearance_class_index(),
        FixedState::UserFixed,
        info.attach_smd_allowed(),
    );
    for segment in &tap.stub_segments {
        let layer = board.board.layer_structure.get_no(&segment.layer);
        if layer < 0 {
            return Err("tap escape stub uses unknown layer".into());
        }
        let a = board.point(segment.a)?;
        let b = board.point(segment.b)?;
        let width = size_in_units(segment.width / 2.0, board.units_per_mm)?;
        let class = board
            .board
            .rules
            .net_class_of(net)
            .get_trace_clearance_class();
        board
            .board
            .insert_trace_without_cleaning(
                Polyline::from_two_points(&a, &b),
                layer,
                width,
                &[net],
                class,
                FixedState::UserFixed,
            )
            .ok_or("tap escape stub insertion failed")?;
    }
    Ok(())
}

fn effective_pad_edge_clearance(
    board: &BoardAdapter,
    pad: &super::whole_context::ContextPad,
    base: f64,
) -> f64 {
    let mut rings = Vec::<Vec<[f64; 2]>>::new();
    for ring in board
        .board_outline_polygons
        .iter()
        .chain(board.board_cutouts.iter())
        .chain(board.board_edge_contours.iter())
    {
        if ring.len() < 2 {
            continue;
        }
        rings.push(
            ring.iter()
                .map(|p| {
                    [
                        p[0] as f64 / board.units_per_mm,
                        p[1] as f64 / board.units_per_mm,
                    ]
                })
                .collect(),
        );
    }
    if rings.is_empty() {
        return base;
    }
    let mut perimeter = Vec::<[f64; 2]>::new();
    for copper in &pad.copper_shapes {
        match &copper.shape {
            super::whole_context::CopperShape::Circle { center, radius } => {
                for index in 0..64 {
                    let angle = std::f64::consts::TAU * index as f64 / 64.0;
                    perimeter.push([
                        center[0] + radius * angle.cos(),
                        center[1] + radius * angle.sin(),
                    ]);
                }
            }
            super::whole_context::CopperShape::Polygon { outline } => {
                for index in 0..outline.len() {
                    let a = outline[index];
                    let b = outline[(index + 1) % outline.len()];
                    for step in 0..8 {
                        let t = step as f64 / 8.0;
                        perimeter.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]);
                    }
                }
            }
            super::whole_context::CopperShape::RoundedRectangle {
                center,
                half_size,
                rotation,
                corner_radius,
            } => {
                let (hx, hy) = (half_size[0], half_size[1]);
                let radius = corner_radius.min(hx).min(hy).max(0.0);
                let ix = hx - radius;
                let iy = hy - radius;
                let mut local = Vec::new();
                for (a, b) in [
                    ([-ix, -hy], [ix, -hy]),
                    ([hx, -iy], [hx, iy]),
                    ([ix, hy], [-ix, hy]),
                    ([-hx, iy], [-hx, -iy]),
                ] {
                    for step in 0..8 {
                        let t = step as f64 / 8.0;
                        local.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]);
                    }
                }
                if radius > 0.0 {
                    for (cx, cy, start) in [
                        (ix, -iy, -90.0),
                        (ix, iy, 0.0),
                        (-ix, iy, 90.0),
                        (-ix, -iy, 180.0),
                    ] {
                        for step in 0..8 {
                            let theta = (start + 90.0 * step as f64 / 8.0).to_radians();
                            local.push([cx + radius * theta.cos(), cy + radius * theta.sin()]);
                        }
                    }
                }
                let angle = rotation.to_radians();
                for [x, y] in local {
                    perimeter.push([
                        center[0] + x * angle.cos() - y * angle.sin(),
                        center[1] + x * angle.sin() + y * angle.cos(),
                    ]);
                }
            }
        }
    }
    if perimeter.is_empty() {
        return base;
    }
    let mut minimum = f64::INFINITY;
    for p in perimeter {
        for ring in &rings {
            for index in 0..ring.len() {
                minimum = minimum.min(point_segment_distance(
                    p,
                    ring[index],
                    ring[(index + 1) % ring.len()],
                ));
            }
        }
    }
    if minimum.is_finite() {
        base.min(minimum.max(0.0))
    } else {
        base
    }
}

fn point_segment_distance(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let d = dx * dx + dy * dy;
    let t = if d <= 0.0 {
        0.0
    } else {
        (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / d).clamp(0.0, 1.0)
    };
    (p[0] - a[0] - t * dx).hypot(p[1] - a[1] - t * dy)
}

pub(crate) fn fence_grid_window(
    map: &mut GridObstacleMap,
    gx0: i32,
    gy0: i32,
    gx1: i32,
    gy1: i32,
    layers: usize,
) {
    // The AWX source clips its PCBData to this local window before building
    // the map. A blocked perimeter is the equivalent on an unbounded grid;
    // allowed_cells must remain reserved for the source BGA-zone override.
    let left = gx0.saturating_sub(1);
    let right = gx1.saturating_add(1);
    let bottom = gy0.saturating_sub(1);
    let top = gy1.saturating_add(1);
    for layer in 0..layers {
        let mut y = bottom;
        while y <= top {
            map.add_static_blocked_cell(left, y, layer);
            map.add_static_blocked_cell(right, y, layer);
            y = y.saturating_add(1);
        }
        let mut x = gx0;
        while x <= gx1 {
            map.add_static_blocked_cell(x, bottom, layer);
            map.add_static_blocked_cell(x, top, layer);
            x = x.saturating_add(1);
        }
    }
}

impl RungExecutor for NativeRungExecutor {
    type Route = NativeRungRoute;

    fn route(&mut self, request: RungRequest) -> Result<RungExecution<Self::Route>, String> {
        if let Some(pair) = request.pair.clone() {
            return route_pair(request, pair);
        }
        execute_single(request, &[], &[], None)
    }
}

impl NativeRungExecutor {
    /// Source `rip_for` probe: soft copper is priced by awx_grid but never
    /// becomes a blocking obstacle. The returned board and path are uncommitted.
    pub fn route_min_cut(
        &mut self,
        request: RungRequest,
        soft_tracks: Vec<super::whole_context::ReservationTube>,
        soft_vias: Vec<super::whole_context::CopperVia>,
        soft_cost: f64,
    ) -> Result<Option<NativeRungRoute>, String> {
        match execute_single(request, &soft_tracks, &soft_vias, Some(soft_cost))? {
            RungExecution::Found(route) => Ok(Some(route)),
            RungExecution::Refused { reason, .. } => {
                let _ = reason;
                Ok(None)
            }
        }
    }
}

fn execute_single(
    mut request: RungRequest,
    soft_tracks: &[super::whole_context::ReservationTube],
    soft_vias: &[super::whole_context::CopperVia],
    soft_cost: Option<f64>,
) -> Result<RungExecution<NativeRungRoute>, String> {
    // `single_ended_routing._per_net_via_config` installs the current net's
    // actual via before connect builds its obstacle map.
    if let Some(sizes) = request
        .rules
        .net_via_sizes
        .get(&(request.net_id as i32))
        .copied()
    {
        request.rules.via_size = sizes[0];
        request.rules.via_drill = sizes[1];
    }
    if request.net_id != request.board.net_number(&request.net)? {
        return Err(format!("rung net id does not match {}", request.net));
    }
    if request.rules.validate().is_err() {
        return Err("invalid native rung routing rules".into());
    }
    if request.deadline.is_some_and(|d| Instant::now() >= d) {
        return Ok(refused("rung deadline elapsed before search"));
    }
    let mut trial = request.board.clone();
    let (control, settings) = prepare_control_for_net_with_vias(
        &mut trial,
        request.net_id,
        &request.rules,
        &request.route_layers,
        &request.fab_via_rungs,
    )?;
    let mut outcome = source_grid_search(
        &request,
        &trial,
        &[request.net_id],
        soft_tracks,
        soft_vias,
        soft_cost,
        control.trace_clearance_class,
        control.via_clearance_class,
    )?;
    if request.deadline.is_some_and(|d| Instant::now() >= d) {
        return Ok(refused("rung deadline elapsed during source grid search"));
    }
    let grid_step_mm = outcome.grid_step_board as f64 / trial.units_per_mm;
    let mut tap_results = Vec::new();
    let path = if let Some(path) = outcome.path.clone() {
        path
    } else {
        let report = super::braid_connect::RungReport {
            blocked_cells: outcome
                .frontier
                .iter()
                .map(|&(x, y, l)| (x, y, l as usize))
                .collect(),
            route_layers: request.route_layers.clone(),
            window: [
                outcome.window.ll.x as f64 / trial.units_per_mm,
                outcome.window.ll.y as f64 / trial.units_per_mm,
                outcome.window.ur.x as f64 / trial.units_per_mm,
                outcome.window.ur.y as f64 / trial.units_per_mm,
            ],
            grid_step_mm,
            expanded: outcome.iterations,
            truncated: false,
        };
        match try_via_pad_unblock(&request, &mut outcome)? {
            Some((path, taps)) => {
                tap_results = taps;
                path
            }
            None => {
                return Ok(refused_with_report(
                    "source awx_grid and via-pad unblocking returned no path",
                    Some(report),
                ))
            }
        }
    };
    let path_start = *path.first().ok_or("awx_grid returned an empty path")?;
    let path_end = *path.last().ok_or("awx_grid returned an empty path")?;
    let start_candidates = vec![request.start.clone()];
    let mut target_candidates = vec![request.end.clone()];
    target_candidates.extend(
        request
            .b_alternatives
            .iter()
            .filter(|e| request.route_layers.contains(&e.layer))
            .cloned(),
    );
    // Both real path terminals resolve against the ORIGINAL source/target
    // candidate sets, then against any kept unblock tap. A retry that escaped
    // through a newly seeded target tap (or a tapped source pad) ends on the
    // tap's via cell, which is not an original endpoint; resolving it here keeps
    // the exact physical tap point and its actual arrival layer attached.
    let start = endpoint_for_path_terminal(
        &start_candidates,
        path_start,
        &request.route_layers,
        &trial,
        outcome.grid_step_board,
        &tap_results,
    )?
    .ok_or("awx_grid started at a cell without a source endpoint")?;
    let target = endpoint_for_path_terminal(
        &target_candidates,
        path_end,
        &request.route_layers,
        &trial,
        outcome.grid_step_board,
        &tap_results,
    )?
    .ok_or("awx_grid ended at a cell without a source endpoint")?;
    let mut through_hole_positions = super::native_grid_window::same_net_through_hole_positions(
        &trial,
        &[request.net_id],
        &request.free_via_pads,
        outcome.grid_step_board,
    )?;
    let mut registered_via_sizes = outcome.registered_via_sizes.clone();
    for tap in &tap_results {
        through_hole_positions.insert(tap.grid_cell);
        registered_via_sizes.extend(
            tap.registered_via_sizes
                .iter()
                .map(|(&cell, &size)| (cell, size)),
        );
    }
    let via_refit_rungs = outcome.via_refit_rungs.clone();
    let [via_size_mm, via_drill_mm] = request
        .rules
        .net_via_sizes
        .get(&(request.net_id as i32))
        .copied()
        .unwrap_or([request.rules.via_size, request.rules.via_drill]);
    let geometry = super::grid_path::path_to_geometry(super::grid_path::GridPathRequest {
        path: &path,
        grid_step_board: outcome.grid_step_board,
        route_layers: &request.route_layers,
        start: &start,
        target: &target,
        board: &trial,
        net: request.net_id,
        via_candidates: control.via_rule.vias(),
        existing_vias: &request.existing_vias,
        via_size_mm,
        via_drill_mm,
        hole_to_hole_clearance_mm: request.rules.hole_to_hole_clearance,
        registered_via_sizes: &registered_via_sizes,
        via_refit_rungs: &via_refit_rungs,
        through_hole_positions: &through_hole_positions,
    })?;
    for tap in &tap_results {
        materialize_tap(&mut trial, tap, request.net_id)?;
    }
    materialize_grid_geometry(&mut trial, &geometry, request.net_id)?;
    if let Err(e) = trial.assert_invariants(&request.board, &BTreeSet::new(), true) {
        return Ok(refused(&format!("native rung board delta invalid: {e}")));
    }
    let grade = native_execution::grade(
        &request.board,
        &trial,
        &BTreeSet::from([request.net.clone()]),
    )?;
    if !grade.open.is_empty() || grade.new_violations != 0 {
        return Ok(refused("native AWX route failed landed-board grade"));
    }
    let mut segments: Vec<_> = geometry
        .segments
        .iter()
        .map(|s| super::pair_connection::PairPiece {
            a: s.start_mm,
            b: s.end_mm,
            width: s.width_mm,
            layer: s.layer.clone(),
            net_id: request.net_id,
        })
        .collect();
    let mut vias: Vec<_> = geometry
        .vias
        .iter()
        .map(|v| {
            let info = &trial.board.rules.via_infos[v.via_info];
            let padstack = trial.board.library.padstacks.get(info.get_padstack());
            let names = padstack
                .map(|p| {
                    (p.from_layer()..=p.to_layer())
                        .filter_map(|l| {
                            trial
                                .board
                                .layer_structure
                                .layers
                                .get(l as usize)
                                .map(|x| x.name.clone())
                        })
                        .collect()
                })
                .unwrap_or_else(|| vec![v.from_layer.clone(), v.to_layer.clone()]);
            super::pair_connection::PairVia {
                point: v.point_mm,
                diameter: v.diameter_mm,
                drill: v.drill_mm,
                layers: names,
                net_id: request.net_id,
            }
        })
        .collect();
    for tap in &tap_results {
        segments.extend(
            tap.stub_segments
                .iter()
                .map(|s| super::pair_connection::PairPiece {
                    a: s.a,
                    b: s.b,
                    width: s.width,
                    layer: s.layer.clone(),
                    net_id: request.net_id,
                }),
        );
        vias.push(super::pair_connection::PairVia {
            point: tap.via.point,
            diameter: tap.via.diameter,
            drill: tap.via.drill,
            layers: tap.via.layers.clone(),
            net_id: request.net_id,
        });
    }
    let grid_path = path.iter().map(|&(x, y, l)| (x, y, l as usize)).collect();
    drop(settings);
    Ok(RungExecution::Found(NativeRungRoute {
        board: trial,
        achieved: target,
        achieved_pair: None,
        grade,
        grid_path,
        grid_step_mm,
        route_segments: segments,
        route_vias: vias,
    }))
}

fn endpoint_for_grid_cell(
    candidates: &[RungEndpoint],
    cell: (i32, i32, u8),
    route_layers: &[String],
    board: &BoardAdapter,
    step: i32,
) -> Result<Option<RungEndpoint>, String> {
    for endpoint in candidates {
        let point = match board.point(endpoint.point)? {
            EnginePoint::Int(p) => p,
            _ => return Err("grid endpoint is outside native coordinate range".into()),
        };
        let layer = route_layers
            .iter()
            .position(|name| name == &endpoint.layer);
        if layer == Some(cell.2 as usize)
            && super::native_obstacles::grid_round(point.x, step) == cell.0
            && super::native_obstacles::grid_round(point.y, step) == cell.1
        {
            return Ok(Some(endpoint.clone()));
        }
    }
    Ok(None)
}

/// Resolve a path terminal first against the ordinary candidate set, then
/// against any materialized unblock tap whose escape-via cell the retry
/// terminated on. A tap is a same-net pad via, so the honest endpoint is the
/// via's physical point on the path's actual arrival layer; the existing tap
/// materialization and stub copper complete the pad connection.
fn endpoint_for_path_terminal(
    candidates: &[RungEndpoint],
    cell: (i32, i32, u8),
    route_layers: &[String],
    board: &BoardAdapter,
    step: i32,
    taps: &[super::via_pad_tap::TapResult],
) -> Result<Option<RungEndpoint>, String> {
    if let Some(endpoint) = endpoint_for_grid_cell(candidates, cell, route_layers, board, step)? {
        return Ok(Some(endpoint));
    }
    for tap in taps {
        if tap.grid_cell == (cell.0, cell.1) {
            // Return the tap's exact physical via point on the path's ACTUAL
            // arrival layer (`cell.2`), not the pad's own layer. When the retry
            // arrives on an inner/source-escape layer the pad layer differs, and
            // `terminal_copper_on_layer`'s half-cell `round()` ties-even can miss
            // the through-hole cell, silently dropping the connector. Naming the
            // arrival layer makes the connector's layer equal the path layer, so
            // it is emitted without touching grid round policy.
            let layer = route_layers
                .get(cell.2 as usize)
                .cloned()
                .ok_or("via-pad tap arrival layer is outside the route layers")?;
            return Ok(Some(RungEndpoint {
                point: tap.via.point,
                layer,
            }));
        }
    }
    Ok(None)
}

fn materialize_grid_geometry(
    board: &mut BoardAdapter,
    geometry: &super::grid_path::GridPathGeometry,
    net: NetNo,
) -> Result<(), String> {
    for via in &geometry.vias {
        let info = board.board.rules.via_infos[via.via_info].clone();
        let padstack = info.get_padstack();
        let point = board.point(via.point_mm)?;
        board.board.insert_via(
            padstack,
            point,
            &[net],
            info.get_clearance_class_index(),
            FixedState::UserFixed,
            info.attach_smd_allowed(),
        );
    }
    for seg in &geometry.segments {
        let layer = board.board.layer_structure.get_no(&seg.layer);
        if layer < 0 {
            return Err(format!("grid path uses invalid stack layer {}", seg.layer));
        }
        let start = board.point(seg.start_mm)?;
        let end = board.point(seg.end_mm)?;
        let width = size_in_units(seg.width_mm / 2.0, board.units_per_mm)?;
        let class = board
            .board
            .rules
            .net_class_of(net)
            .get_trace_clearance_class();
        board
            .board
            .insert_trace_without_cleaning(
                Polyline::from_two_points(&start, &end),
                layer,
                width,
                &[net],
                class,
                FixedState::UserFixed,
            )
            .ok_or("grid path trace insertion failed")?;
    }
    Ok(())
}

fn refused(reason: &str) -> RungExecution<NativeRungRoute> {
    refused_with_report(reason, None)
}

fn refused_with_report(
    reason: &str,
    report: Option<super::braid_connect::RungReport>,
) -> RungExecution<NativeRungRoute> {
    RungExecution::Refused {
        reason: Some(reason.to_owned()),
        report,
    }
}

fn route_pair(
    request: RungRequest,
    pair: super::braid_connect::PairRungRequest,
) -> Result<RungExecution<NativeRungRoute>, String> {
    let mut backend = super::native_pair_router::NativePairRouter::new(&request)?;
    let outcome = super::pair_connection::connect_pair(&request, &pair, &mut backend)?;
    let result = match outcome {
        super::pair_connection::PairConnectionOutcome::Found(result) => result,
        super::pair_connection::PairConnectionOutcome::Refused { reason, report } => {
            return Ok(RungExecution::Refused { reason, report })
        }
    };
    let mut trial = request.board.clone();
    materialize_pair_geometry(&mut trial, &result.pieces, &result.vias, &pair, &request)?;
    if let Err(error) = trial.assert_invariants(&request.board, &BTreeSet::new(), true) {
        return Ok(refused(&format!(
            "native pair board delta invalid: {error}"
        )));
    }
    let scope = BTreeSet::from([pair.positive_net.clone(), pair.negative_net.clone()]);
    let grade = native_execution::grade(&request.board, &trial, &scope)?;
    if !grade.open.is_empty() || grade.new_violations != 0 {
        return Ok(refused("source pair route failed landed-board grade"));
    }
    let p = RungEndpoint {
        point: result.end_points[0],
        layer: result.end_layer.clone(),
    };
    let n = RungEndpoint {
        point: result.end_points[1],
        layer: pair.negative_end_layer.clone(),
    };
    let grid_step_mm = request.rules.grid_step;
    Ok(RungExecution::Found(NativeRungRoute {
        board: trial,
        achieved: p.clone(),
        achieved_pair: Some([p, n]),
        grade,
        grid_path: Vec::new(),
        grid_step_mm,
        route_segments: result.pieces,
        route_vias: result.vias,
    }))
}

fn materialize_pair_geometry(
    board: &mut BoardAdapter,
    pieces: &[super::pair_connection::PairPiece],
    vias: &[super::pair_connection::PairVia],
    pair: &super::braid_connect::PairRungRequest,
    request: &RungRequest,
) -> Result<(), String> {
    for via in vias {
        let net = if via.net_id == pair.positive_net_id {
            pair.positive_net_id
        } else if via.net_id == pair.negative_net_id {
            pair.negative_net_id
        } else {
            return Err("pair route returned a foreign via net".into());
        };
        let info_id = select_pair_via_info(board, net, via.diameter, via.drill)?;
        let info = board.board.rules.via_infos[info_id].clone();
        let padstack = info.get_padstack();
        let point = board.point(via.point)?;
        board.board.insert_via(
            padstack,
            point,
            &[net],
            info.get_clearance_class_index(),
            FixedState::UserFixed,
            info.attach_smd_allowed(),
        );
    }
    for piece in pieces {
        let layer = board.board.layer_structure.get_no(&piece.layer);
        if layer < 0 {
            return Err(format!(
                "pair route uses invalid stack layer {}",
                piece.layer
            ));
        }
        let net = if piece.net_id == pair.positive_net_id {
            pair.positive_net_id
        } else if piece.net_id == pair.negative_net_id {
            pair.negative_net_id
        } else {
            return Err("pair route returned a foreign track net".into());
        };
        let a = board.point(piece.a)?;
        let b = board.point(piece.b)?;
        let width = size_in_units(piece.width / 2.0, board.units_per_mm)?;
        let class = board
            .board
            .rules
            .net_class_of(net)
            .get_trace_clearance_class();
        board
            .board
            .insert_trace_without_cleaning(
                Polyline::from_two_points(&a, &b),
                layer,
                width,
                &[net],
                class,
                FixedState::UserFixed,
            )
            .ok_or("pair route trace insertion failed")?;
    }
    let _ = request;
    Ok(())
}

fn select_pair_via_info(
    board: &mut BoardAdapter,
    net: NetNo,
    diameter: f64,
    drill: f64,
) -> Result<ViaInfoId, String> {
    let name = board
        .board
        .rules
        .nets
        .get(net)
        .ok_or("pair route via net missing")?
        .name
        .clone();
    super::native_execution::ensure_via_candidate(board, &name, diameter, drill)
}

fn search_window(request: &RungRequest, start: IntPoint, end: IntPoint) -> Result<IntBox, String> {
    search_window_for_points(
        request,
        &[request.start.point, request.end.point],
        start,
        end,
    )
}

pub(crate) fn search_window_for_points(
    request: &RungRequest,
    endpoints: &[[f64; 2]],
    include_a: IntPoint,
    include_b: IntPoint,
) -> Result<IntBox, String> {
    let mut points = endpoints.to_vec();
    if let Some(window_points) = &request.window_points {
        points.extend(window_points.iter().copied());
    }
    if points.iter().flatten().any(|v| !v.is_finite()) {
        return Err("nonfinite native rung window point".into());
    }
    let min_x = points.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min);
    let max_x = points
        .iter()
        .map(|p| p[0])
        .fold(f64::NEG_INFINITY, f64::max);
    let min_y = points.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min);
    let max_y = points
        .iter()
        .map(|p| p[1])
        .fold(f64::NEG_INFINITY, f64::max);
    let center = [(min_x + max_x) / 2.0, (min_y + max_y) / 2.0];
    let half = (max_x - min_x).max(max_y - min_y) / 2.0 + request.spec.margin;
    let low = request.board.point([center[0] - half, center[1] - half])?;
    let high = request.board.point([center[0] + half, center[1] + half])?;
    let (low, high) = match (low, high) {
        (EnginePoint::Int(low), EnginePoint::Int(high)) => (low, high),
        _ => return Err("native rung window exceeds integer engine range".into()),
    };
    let bounds = request.board.board.bounding_box;
    let window = IntBox::new(
        low.x.max(bounds.ll.x),
        low.y.max(bounds.ll.y),
        high.x.min(bounds.ur.x),
        high.y.min(bounds.ur.y),
    );
    if window.is_empty() || !contains(window, include_a) || !contains(window, include_b) {
        return Err("native rung search window excludes an endpoint".into());
    }
    Ok(window)
}

fn contains(bounds: IntBox, point: IntPoint) -> bool {
    bounds.ll.x <= point.x
        && point.x <= bounds.ur.x
        && bounds.ll.y <= point.y
        && point.y <= bounds.ur.y
}

fn prepare_control(
    request: &RungRequest,
) -> Result<(BoardAdapter, AutorouteControl, RouterSettings), String> {
    let mut board = request.board.clone();
    let (control, settings) = prepare_control_for_net_with_vias(
        &mut board,
        request.net_id,
        &request.rules,
        &request.route_layers,
        &request.fab_via_rungs,
    )?;
    Ok((board, control, settings))
}

pub(crate) fn prepare_control_for_net(
    board: &mut BoardAdapter,
    net: NetNo,
    rules: &super::whole_context::RoutingRules,
    route_layers: &[String],
) -> Result<(AutorouteControl, RouterSettings), String> {
    prepare_control_for_net_with_vias(board, net, rules, route_layers, &[])
}

pub(crate) fn prepare_control_for_net_with_vias(
    board: &mut BoardAdapter,
    net: NetNo,
    rules: &super::whole_context::RoutingRules,
    route_layers: &[String],
    fab_via_rungs: &[(f64, f64)],
) -> Result<(AutorouteControl, RouterSettings), String> {
    let layer_count = board.board.layer_count().max(0) as usize;
    let half_width = size_in_units(rules.track_width / 2.0, board.units_per_mm)?;
    let clearance = size_in_units(rules.clearance, board.units_per_mm)?;

    let base_class = board
        .board
        .rules
        .net_class_of(net)
        .get_trace_clearance_class();
    let count_before = board.board.rules.clearance_matrix.get_class_count();
    let matrix = &mut board.board.rules_mut().clearance_matrix;
    let class_name = format!("native_rung_{}_{}", net, count_before);
    if !matrix.append_class(&class_name) {
        return Err("failed to allocate native rung clearance class".into());
    }
    let routed_class = matrix.get_no(&class_name);
    let class_count = matrix.get_class_count();
    for other in 0..class_count {
        for layer in 0..layer_count as i32 {
            let old = matrix
                .get_value(base_class, other, layer, false)
                .max(matrix.get_value(other, base_class, layer, false));
            let value = old.max(clearance);
            matrix.set_value(routed_class, other, layer, value);
            matrix.set_value(other, routed_class, layer, value);
        }
    }

    let mut settings = RouterSettings::default();
    settings.set_layer_count(layer_count);
    settings.vias_allowed = Some(true);
    for (index, layer) in board.board.layer_structure.layers.iter().enumerate() {
        // LayerStructure entries are concrete copper stack layers in DSN,
        // including power planes; only the request's explicit route subset
        // decides which ones are active for this search.
        settings.set_layer_active(index, route_layers.contains(&layer.name));
    }
    let via_costs = settings.get_via_costs();
    let trace_costs = settings.get_trace_costs();
    let mut control = AutorouteControl::new(&board.board, net, &settings, via_costs, trace_costs);
    control.trace_clearance_class = routed_class;
    control.trace_half_width.fill(half_width);
    control.compensated_trace_half_width.fill(half_width);

    let via_rule = matching_via_rule(board, net, rules, fab_via_rungs)?;
    control.via_rule = via_rule;
    control.rebuild_via_info(&board.board, via_costs, net);
    control.vias_allowed = !control.via_infos.is_empty() && settings.get_vias_allowed();
    if !control.vias_allowed {
        control.via_infos.clear();
    }
    Ok((control, settings))
}

pub(crate) fn pair_clearance_mm(
    board: &BoardAdapter,
    positive: NetNo,
    negative: NetNo,
    route_layers: &[String],
) -> Result<f64, String> {
    let p = board
        .board
        .rules
        .net_class_of(positive)
        .get_trace_clearance_class();
    let n = board
        .board
        .rules
        .net_class_of(negative)
        .get_trace_clearance_class();
    let mut max_clearance = 0i32;
    for name in route_layers {
        let layer = board.board.layer_structure.get_no(name);
        if layer < 0 {
            return Err(format!("pair clearance uses unknown layer {name}"));
        }
        max_clearance = max_clearance
            .max(board.board.clearance_value(p, n, layer))
            .max(board.board.clearance_value(n, p, layer));
    }
    Ok(max_clearance.max(0) as f64 / board.units_per_mm)
}

fn matching_via_rule(
    board: &mut BoardAdapter,
    net: NetNo,
    rules: &super::whole_context::RoutingRules,
    fab_via_rungs: &[(f64, f64)],
) -> Result<ViaRule, String> {
    let expected = rules
        .net_via_sizes
        .get(&net)
        .copied()
        .unwrap_or([rules.via_size, rules.via_drill]);
    let mut matched = ViaRule::new(format!("native_rung_{net}"));
    let mut candidates = vec![(expected[0], expected[1])];
    candidates.extend(fab_via_rungs.iter().copied());
    candidates.dedup_by(|a, b| (a.0 - b.0).abs() <= 1e-9 && (a.1 - b.1).abs() <= 1e-9);
    let net_name = board
        .board
        .rules
        .nets
        .get(net)
        .ok_or("native via net missing")?
        .name
        .clone();
    for (diameter, drill) in candidates {
        let info_id =
            super::native_execution::ensure_via_candidate(board, &net_name, diameter, drill)?;
        if !matched.vias().contains(&info_id) {
            matched.append_via(info_id);
        }
    }
    Ok(matched)
}

fn size_in_units(size_mm: f64, units_per_mm: f64) -> Result<i32, String> {
    let size = (size_mm * units_per_mm).round();
    if !size.is_finite() || size < 0.0 || size > i32::MAX as f64 {
        return Err("native rung size outside engine range".into());
    }
    Ok(size as i32)
}

/// Ordinary DSN endpoint adapter: source GridRouter search, exact-terminal
/// geometry, native stack/via materialization, and native transaction grading.
/// No BGA inference, corridor planning, taps or alternate router.
pub fn route_multilayer_endpoints(
    request: RungRequest,
    from_item: fr_engine::board::ItemKey,
    to_item: fr_engine::board::ItemKey,
) -> Result<(Option<BoardAdapter>, serde_json::Value), String> {
    let starts = vec![request.start.clone()];
    let mut targets = vec![request.end.clone()];
    targets.extend(request.b_alternatives.iter().cloned());
    route_multilayer_endpoint_sets(request, from_item, to_item, &starts, &targets)
}

pub fn route_multilayer_endpoint_sets(
    mut request: RungRequest,
    from_item: fr_engine::board::ItemKey,
    to_item: fr_engine::board::ItemKey,
    start_candidates: &[RungEndpoint],
    target_candidates: &[RungEndpoint],
) -> Result<(Option<BoardAdapter>, serde_json::Value), String> {
    if request.route_layers.is_empty() || request.pair.is_some() || request.band.is_some() {
        return Err("endpoint adapter requires active layers and a single-ended request".into());
    }
    let starts = start_candidates
        .iter()
        .filter(|endpoint| request.route_layers.contains(&endpoint.layer))
        .cloned()
        .collect::<Vec<_>>();
    let targets = target_candidates
        .iter()
        .filter(|endpoint| request.route_layers.contains(&endpoint.layer))
        .cloned()
        .collect::<Vec<_>>();
    if starts.is_empty() || targets.is_empty() {
        return Ok((None, serde_json::json!({
            "accepted": false,
            "reason": "endpoint candidate set has no active route layer"
        })));
    }
    request.start = starts[0].clone();
    request.end = targets[0].clone();
    request.b_alternatives = targets.iter().skip(1).cloned().collect();
    request.rules.validate()?;
    let net = request.board.net_number(&request.net)?;
    let class = request.board.board.rules.net_class_of(net).get_trace_clearance_class();
    let calls_before = ENDPOINT_GRID_CALLS.load(std::sync::atomic::Ordering::Relaxed);
    let (mut trial, control, _) = prepare_control(&request)?;
    let mut outcome = source_grid_search_with_endpoints(
        &request,
        &trial,
        &[net],
        &[],
        &[],
        None,
        class,
        class,
        &starts,
        &targets,
    )?;
    let calls = ENDPOINT_GRID_CALLS.load(std::sync::atomic::Ordering::Relaxed) - calls_before;
    let mut report = serde_json::json!({"grid_router_calls":calls,"expanded":outcome.iterations,"path_cells":outcome.path.as_ref().map_or(0,Vec::len),"accepted":false});
    let mut tap_results = Vec::new();
    let path = if let Some(path) = outcome.path.clone() {
        path
    } else {
        // Source `_route_with_via_unblock` (#189): an endpoint whose probe
        // frontier exhausted BELOW the probe limit is boxed in. Drop a
        // DRC-legal fab-floor via inside that pad and retry the SAME search
        // before reporting no-path; only then is the failure honest.
        if request.deadline.is_some_and(|d| Instant::now() >= d) {
            report["reason"] = "deadline elapsed before via-pad unblock".into();
            return Ok((None, report));
        }
        match try_via_pad_unblock(&request, &mut outcome)? {
            Some((path, taps)) => {
                tap_results = taps;
                path
            }
            None => {
                report["grid_router_calls"] =
                    (ENDPOINT_GRID_CALLS.load(std::sync::atomic::Ordering::Relaxed) - calls_before).into();
                report["expanded"] = outcome.iterations.into();
                report["path_cells"] = outcome.path.as_ref().map_or(0, Vec::len).into();
                report["reason"] =
                    "source GridRouter and via-pad unblocking returned no path".into();
                return Ok((None, report));
            }
        }
    };
    report["grid_router_calls"] =
        (ENDPOINT_GRID_CALLS.load(std::sync::atomic::Ordering::Relaxed) - calls_before).into();
    report["expanded"] = outcome.iterations.into();
    report["path_cells"] = path.len().into();
    if request.deadline.is_some_and(|d|Instant::now()>=d) {
        report["reason"] = "deadline elapsed during source GridRouter search".into();
        return Ok((None,report));
    }
    let (Some(&path_start), Some(&path_end)) = (path.first(), path.last()) else {
        report["reason"] = "source GridRouter returned an empty path".into();
        return Ok((None, report));
    };
    let Some(start) = endpoint_for_path_terminal(
        &starts,
        path_start,
        &request.route_layers,
        &trial,
        outcome.grid_step_board,
        &tap_results,
    )? else {
        report["reason"] = "GridRouter path starts outside the source endpoint set".into();
        return Ok((None, report));
    };
    let Some(target) = endpoint_for_path_terminal(
        &targets,
        path_end,
        &request.route_layers,
        &trial,
        outcome.grid_step_board,
        &tap_results,
    )? else {
        report["reason"] = "GridRouter path ends outside the target endpoint set".into();
        return Ok((None, report));
    };
    report["start_endpoint"] = serde_json::json!({
        "point": start.point,
        "layer": start.layer,
    });
    report["end_endpoint"] = serde_json::json!({
        "point": target.point,
        "layer": target.layer,
    });
    let mut through_holes = super::native_grid_window::same_net_through_hole_positions(&trial, &[net], &request.free_via_pads, outcome.grid_step_board)?;
    // A tapped via is a same-net through-hole barrel: skip it in the emitted
    // path's via-through-hole checks and carry its registered physical size.
    let mut registered_via_sizes = outcome.registered_via_sizes.clone();
    for tap in &tap_results {
        through_holes.insert(tap.grid_cell);
        registered_via_sizes.extend(tap.registered_via_sizes.iter().map(|(&cell, &size)| (cell, size)));
    }
    let geometry = super::grid_path::path_to_geometry(super::grid_path::GridPathRequest {
        path:&path,grid_step_board:outcome.grid_step_board,route_layers:&request.route_layers,
        start:&start,target:&target,board:&trial,net,
        via_candidates:control.via_rule.vias(),via_size_mm:request.rules.via_size,via_drill_mm:request.rules.via_drill,
        hole_to_hole_clearance_mm:request.rules.hole_to_hole_clearance,via_refit_rungs:&outcome.via_refit_rungs,
        registered_via_sizes:&registered_via_sizes,existing_vias:&request.existing_vias,through_hole_positions:&through_holes,
    })?;
    // Commit the kept unblock copper, then the grid geometry, so the landed
    // board carries the tapped vias the retry actually reached (#803).
    for tap in &tap_results {
        materialize_tap(&mut trial, tap, net)?;
    }
    materialize_grid_geometry(&mut trial,&geometry,net)?;
    if let Err(error)=trial.assert_preserved(&request.board) {
        let details: Vec<_>=trial.board.get_items().into_iter().flat_map(|key|fr_engine::drc::clearance_violation::clearance_violations(&trial.board,key)).map(|v| {
            let a=trial.board.item(v.first_item);let b=trial.board.item(v.second_item);
            serde_json::json!({"first_id":a.id().0,"first_type":format!("{:?}",a.board_item_type()),"second_id":b.id().0,"second_type":format!("{:?}",b.board_item_type()),"layer":v.layer,"expected_clearance_mm":v.expected_clearance/trial.units_per_mm,"actual_clearance_mm":v.actual_clearance/trial.units_per_mm,"overlap_bounds":format!("{:?}",v.shape.bounding_box())})
        }).collect();
        report["reason"]=error.into();
        report["candidate_violations"]=serde_json::json!(details);
        report["candidate_vias"]=serde_json::json!(geometry.vias.iter().map(|v|serde_json::json!({"point_mm":v.point_mm,"diameter_mm":v.diameter_mm,"drill_mm":v.drill_mm,"from_layer":v.from_layer,"to_layer":v.to_layer})).collect::<Vec<_>>());
        report["candidate_segments"]=serde_json::json!(geometry.segments.iter().map(|s|serde_json::json!({"start_mm":s.start_mm,"end_mm":s.end_mm,"layer":s.layer,"width_mm":s.width_mm})).collect::<Vec<_>>());
        return Ok((None,report));
    }

    let before = fr_engine::drc::NetIncompletes::new(&request.board.board,net,&request.board.board.get_connectable_items(net)).count();
    let after = fr_engine::drc::NetIncompletes::new(&trial.board,net,&trial.board.get_connectable_items(net)).count();
    // `assert_preserved` above compares the trial's full-board clearance
    // violations with the baseline and rejects any new violation. The trial
    // is unchanged since that proof, so the accepted route's new-violation
    // count is exactly zero without running the same full-board scan again.
    let new_violations = 0usize;
    report["incompletes_before"] = before.into();
    report["incompletes_after"] = after.into();
    report["new_violations"] = new_violations.into();
    let tap_stub_segments: usize = tap_results.iter().map(|tap| tap.stub_segments.len()).sum();
    report["segments"] = (geometry.segments.len() + tap_stub_segments).into();
    report["vias"] = (geometry.vias.len() + tap_results.len()).into();
    report["layer_transitions"] = path.windows(2).filter(|w|w[0].2!=w[1].2).count().into();
    let mut via_geometry = geometry.vias.iter().map(|v| {
        let info=&trial.board.rules.via_infos[v.via_info];
        let pad=trial.board.library.padstacks.get(info.get_padstack()).unwrap();
        serde_json::json!({"point_mm":v.point_mm,"diameter_mm":v.diameter_mm,"drill_mm":v.drill_mm,"from_layer":v.from_layer,"to_layer":v.to_layer,"physical_first_layer":pad.from_layer(),"physical_last_layer":pad.to_layer()})
    }).collect::<Vec<_>>();
    for tap in &tap_results {
        via_geometry.push(serde_json::json!({"point_mm":tap.via.point,"diameter_mm":tap.via.diameter,"drill_mm":tap.via.drill,"from_layer":tap.via.layers.first(),"to_layer":tap.via.layers.last()}));
    }
    report["via_geometry"] = serde_json::json!(via_geometry);

    let merged = trial.board.connected_set(from_item,net,false).contains(trial.board.item(to_item).id());
    report["target_groups_merged"] = merged.into();
    if !merged || after>=before || new_violations!=0 {
        report["reason"] = "native transaction did not reduce incompletes without new violations".into();
        return Ok((None,report));
    }
    report["accepted"] = true.into();
    Ok((Some(trial),report))
}

/// Number of actual source GridRouter calls; endpoint CLI is serial.
pub fn endpoint_grid_calls() -> usize { ENDPOINT_GRID_CALLS.load(std::sync::atomic::Ordering::Relaxed) }

/// Compatibility entry for ordinary single-layer endpoints.
pub fn route_single_layer_endpoints(
    request: RungRequest,
    from_item: fr_engine::board::ItemKey,
    to_item: fr_engine::board::ItemKey,
) -> Result<(Option<BoardAdapter>, serde_json::Value), String> {
    if request.route_layers.len() != 1 {
        return Err("single-layer endpoint adapter requires exactly one route layer".into());
    }
    route_multilayer_endpoints(request, from_item, to_item)
}
