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

/// An exact native endpoint that must survive geometry materialization verbatim.
type ExactEndpointOverride = ([f64; 2], String, EnginePoint);

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
    forward_frontier: Vec<(i32, i32, u8)>,
    backward_frontier: Vec<(i32, i32, u8)>,
    forward_iterations: usize,
    backward_iterations: usize,
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
    /// #479 blocked-terminal seeding: each seeded free on-pad grid cell mapped
    /// to the pad's exact native Pin-centre endpoint. The route lands on the
    /// cell (source semantics, geometry unchanged); the centre is the ONLY
    /// extra same-net connector materialized so native trace contact holds.
    landing_aliases: BTreeMap<(i32, i32, u8), RungEndpoint>,
    /// Source `pcb_data._via_unblock_blame` keystone (single_ended_routing
    /// `2850..2859`): the exact foreign net whose mature Trace/Via copper boxes a
    /// pad when the in-pad tap DECLINED and nothing was in flight yet. Populated
    /// only from the `awx::via_blocker` helper (a mature nearest foreign
    /// Trace/Via net only -- never a Pin, never invented). Private working state:
    /// surfaced on the no-path report under the private `_via_unblock_blame` key
    /// so the endpoint owner can hand it to `RecoveryState` and strip it before
    /// public logging; the user-facing report schema is unchanged.
    via_unblock_blame: BTreeSet<NetNo>,
}

struct SourceRouteAttempt {
    path: Option<Vec<(i32, i32, u8)>>,
    iterations: usize,
    frontier: Vec<(i32, i32, u8)>,
    forward_frontier: Vec<(i32, i32, u8)>,
    backward_frontier: Vec<(i32, i32, u8)>,
    forward_iterations: usize,
    backward_iterations: usize,
    forward_probe_iterations: usize,
    backward_probe_iterations: usize,
}

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
                forward_frontier: Vec::new(),
                backward_frontier: Vec::new(),
                forward_iterations: fi,
                backward_iterations: 0,
                forward_probe_iterations: fi,
                backward_probe_iterations: 0,
            };
        }
        let forward_frontier = frontier.clone();
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
        frontier.extend(back_frontier.iter().copied());
        if let Some(mut path) = backward {
            path.reverse();
            return SourceRouteAttempt {
                path: Some(path),
                iterations: fi + bi,
                frontier: Vec::new(),
                forward_frontier,
                backward_frontier: Vec::new(),
                forward_iterations: fi,
                backward_iterations: bi,
                forward_probe_iterations: fi,
                backward_probe_iterations: bi,
            };
        }
        if fi < probe_budget || bi < probe_budget {
            return SourceRouteAttempt {
                path: None,
                iterations: fi + bi,
                frontier,
                forward_frontier,
                backward_frontier: back_frontier,
                forward_iterations: fi,
                backward_iterations: bi,
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
                forward_frontier: Vec::new(),
                backward_frontier: Vec::new(),
                forward_iterations: fi + ff,
                backward_iterations: bi,
                forward_probe_iterations: fi,
                backward_probe_iterations: bi,
            };
        }
        let forward_frontier = full_frontier.clone();
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
        full_frontier.extend(back_frontier.iter().copied());
        if let Some(mut path) = full_backward {
            path.reverse();
            SourceRouteAttempt {
                path: Some(path),
                iterations: fi + bi + ff + fb,
                frontier: Vec::new(),
                forward_frontier,
                backward_frontier: Vec::new(),
                forward_iterations: fi + ff,
                backward_iterations: bi + fb,
                forward_probe_iterations: fi,
                backward_probe_iterations: bi,
            }
        } else {
            SourceRouteAttempt {
                path: None,
                iterations: fi + bi + ff + fb,
                frontier: full_frontier,
                forward_frontier,
                backward_frontier: back_frontier,
                forward_iterations: fi + ff,
                backward_iterations: bi + fb,
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

/// The pad's RAW copper cross-section on one native layer, straight from the
/// endpoint metadata (`copper_shapes`), never a bounding box or a shape-name
/// guess. Endpoint metadata frequently carries an empty `shape` with
/// `rect_rotation == 0`, which must NOT make a circle or custom pad masquerade
/// as a rectangle.
fn pad_layer_copper<'a>(
    pad: &'a super::whole_context::ContextPad,
    layer: &str,
) -> Vec<&'a super::whole_context::CopperShape> {
    pad.copper_shapes
        .iter()
        .filter(|copper| copper.layer == layer)
        .map(|copper| &copper.shape)
        .collect()
}

/// Source `net_queries.pad_landing_extent` over the ACTUAL layer copper: the
/// copper inset by `track_width/2` so a whole track cross-section stays inside,
/// expressed as the region the track centre may occupy. A circle becomes an
/// inset disc and a non-rotated rounded rectangle an inset rectangle. Rotated
/// rectangles and custom polygons keep the source EXCLUDE semantics (`None`) and
/// are never reinterpreted from a bounding box.
enum PadLanding {
    Disc { center: [f64; 2], radius: f64 },
    Rect { center: [f64; 2], half: [f64; 2] },
}

fn pad_landing(
    pad: &super::whole_context::ContextPad,
    layer: &str,
    track_width_mm: f64,
) -> Option<PadLanding> {
    let inset = track_width_mm / 2.0;
    match pad_layer_copper(pad, layer).as_slice() {
        [super::whole_context::CopperShape::Circle { center, radius }] => {
            let radius = radius - inset;
            (radius > 0.0).then_some(PadLanding::Disc {
                center: *center,
                radius,
            })
        }
        [super::whole_context::CopperShape::RoundedRectangle {
            center,
            half_size,
            rotation,
            ..
        }] => {
            if *rotation != 0.0 {
                return None;
            }
            let half = [half_size[0] - inset, half_size[1] - inset];
            (half[0] > 0.0 && half[1] > 0.0).then_some(PadLanding::Rect {
                center: *center,
                half,
            })
        }
        // Custom polygons and ambiguous/multi-shape metadata are excluded
        // rather than guessed from a bounding box.
        _ => None,
    }
}

/// A same-net bridge from a seeded landing cell to the pad's exact Pin centre is
/// only emitted when the whole segment stays inside the pad's own layer copper
/// (`copper_shapes`). This is the source bridge qualification and avoids a fake
/// contact riding outside the real copper.
fn bridge_inside_copper(
    copper: &[&super::whole_context::CopperShape],
    from_mm: [f64; 2],
    to_mm: [f64; 2],
) -> bool {
    if copper.is_empty() {
        return false;
    }
    for step in 0..=8 {
        let t = step as f64 / 8.0;
        let p = [
            from_mm[0] + (to_mm[0] - from_mm[0]) * t,
            from_mm[1] + (to_mm[1] - from_mm[1]) * t,
        ];
        if !copper.iter().any(|shape| shape.signed_distance(p) <= 0.0) {
            return false;
        }
    }
    true
}

/// Source `single_ended_routing._free_on_pad_cells` (#479): grid cells on a
/// pad's own LAYER copper that are FREE for a track on `layer`, keeping the
/// whole track cross-section inside the real copper. Cell indices are global
/// board grid cells so the skip cell and obstacle indices match the search map.
/// `bounds` is the source `bounds` window: seeded cells outside it are dropped.
#[allow(clippy::too_many_arguments)]
fn free_on_pad_cells(
    pad: &super::whole_context::ContextPad,
    layer: &str,
    layer_index: usize,
    obstacles: &GridObstacleMap,
    grid_step_board: i32,
    units_per_mm: f64,
    skip_cell: Option<(i32, i32)>,
    track_width_mm: f64,
    bounds: (i32, i32, i32, i32),
) -> Vec<(i32, i32)> {
    let Some(landing) = pad_landing(pad, layer, track_width_mm) else {
        return Vec::new();
    };
    let (center, extent) = match &landing {
        PadLanding::Disc { center, radius } => (*center, [*radius, *radius]),
        PadLanding::Rect { center, half } => (*center, *half),
    };
    let step_mm = grid_step_board as f64 / units_per_mm;
    let to_grid = |value_mm: f64| -> i32 { (value_mm / step_mm).round_ties_even() as i32 };
    let gx0 = to_grid(center[0] - extent[0]);
    let gy0 = to_grid(center[1] - extent[1]);
    let gx1 = to_grid(center[0] + extent[0]);
    let gy1 = to_grid(center[1] + extent[1]);
    let (bx0, by0, bx1, by1) = bounds;
    let mut cells = Vec::new();
    for cgx in gx0..=gx1 {
        let cx = cgx as f64 * step_mm;
        if (cx - center[0]).abs() > extent[0] {
            continue;
        }
        for cgy in gy0..=gy1 {
            if skip_cell == Some((cgx, cgy)) {
                continue;
            }
            let cy = cgy as f64 * step_mm;
            if (cy - center[1]).abs() > extent[1] {
                continue;
            }
            if let PadLanding::Disc { center, radius } = &landing {
                if (cx - center[0]).powi(2) + (cy - center[1]).powi(2) > radius * radius {
                    continue;
                }
            }
            if obstacles.is_blocked(cgx, cgy, layer_index) {
                continue;
            }
            if cgx < bx0 || cgx > bx1 || cgy < by0 || cgy > by1 {
                continue;
            }
            cells.push((cgx, cgy));
        }
    }
    cells
}

/// Exact native Pin centre for `net` nearest `near_mm`, in board millimetres.
/// The native trace-contact rule is `*point == other.center(self)`, so a seeded
/// landing must bridge to the pin's real centre, not a rounded pad snapshot.
fn native_pin_center_mm(board: &BoardAdapter, net: NetNo, near_mm: [f64; 2]) -> Option<[f64; 2]> {
    let upm = board.units_per_mm;
    let mut best: Option<(f64, [f64; 2])> = None;
    for key in board.board.get_pins() {
        let item = board.board.item(key);
        if !item.contains_net(net) {
            continue;
        }
        let center = item.center(&board.board).to_float();
        let center_mm = [center.x / upm, center.y / upm];
        let d = (center_mm[0] - near_mm[0]).hypot(center_mm[1] - near_mm[1]);
        if d <= 1e-3 && best.as_ref().map_or(true, |(bd, _)| d < *bd) {
            best = Some((d, center_mm));
        }
    }
    best.map(|(_, point)| point)
}

/// Source `single_ended_routing._augment_blocked_pad_terminals` (#479): when a
/// PAD endpoint's grid cell is hard-blocked -- a foreign keep-out reaching over
/// the pad centre -- seed the pad's FREE on-copper cells on that layer as extra
/// endpoint cells, so the route terminates on pad copper instead of failing at
/// an unsteppable start. No-op for the overwhelmingly common free-cell case.
/// Returns the extra `(gx, gy, layer)` cells and the landing alias mapping each
/// seeded cell to the pad's exact native Pin-centre endpoint.
#[allow(clippy::too_many_arguments)]
fn augment_blocked_pad_terminals(
    endpoints: &[RungEndpoint],
    pads: &[super::whole_context::ContextPad],
    net: NetNo,
    obstacles: &GridObstacleMap,
    board: &BoardAdapter,
    route_layers: &[String],
    track_width_mm: f64,
    grid_step_board: i32,
    bounds: (i32, i32, i32, i32),
) -> Result<(Vec<(i32, i32, u8)>, BTreeMap<(i32, i32, u8), RungEndpoint>), String> {
    let upm = board.units_per_mm;
    let mut extra = Vec::new();
    let mut aliases = BTreeMap::new();
    let mut seen = BTreeSet::<(i64, i64, u8)>::new();
    for endpoint in endpoints {
        let Some(layer_index) = route_layers.iter().position(|name| name == &endpoint.layer) else {
            continue;
        };
        let point = match board.point(endpoint.point)? {
            EnginePoint::Int(point) => point,
            _ => continue,
        };
        let gx = super::native_obstacles::grid_round(point.x, grid_step_board);
        let gy = super::native_obstacles::grid_round(point.y, grid_step_board);
        if !obstacles.is_blocked(gx, gy, layer_index) {
            continue;
        }
        for pad in pads {
            if pad.net_id != net
                || (pad.point[0] - endpoint.point[0]).abs() > 1e-3
                || (pad.point[1] - endpoint.point[1]).abs() > 1e-3
            {
                continue;
            }
            let key = (
                (pad.point[0] * 10_000.0).round_ties_even() as i64,
                (pad.point[1] * 10_000.0).round_ties_even() as i64,
                layer_index as u8,
            );
            if seen.insert(key) {
                let free = free_on_pad_cells(
                    pad,
                    &endpoint.layer,
                    layer_index,
                    obstacles,
                    grid_step_board,
                    upm,
                    Some((gx, gy)),
                    track_width_mm,
                    bounds,
                );
                if !free.is_empty() {
                    let center = RungEndpoint {
                        point: native_pin_center_mm(board, net, pad.point).unwrap_or(pad.point),
                        layer: endpoint.layer.clone(),
                    };
                    // #479 bridge qualification: a seeded landing only counts
                    // when the same-net connector to the pad's exact Pin centre
                    // stays inside the pad's own LAYER copper.
                    let copper = pad_layer_copper(pad, &endpoint.layer);
                    let step_mm = grid_step_board as f64 / upm;
                    for (cgx, cgy) in free {
                        let cell_mm = [cgx as f64 * step_mm, cgy as f64 * step_mm];
                        if !bridge_inside_copper(&copper, cell_mm, center.point) {
                            continue;
                        }
                        let cell = (cgx, cgy, layer_index as u8);
                        extra.push(cell);
                        aliases.entry(cell).or_insert_with(|| center.clone());
                    }
                }
            }
            break;
        }
    }
    Ok((extra, aliases))
}

/// Materialize the ONLY extra copper a #479 seeded landing needs: a same-net
/// full-width connector, strictly inside the pad copper, from the landed grid
/// cell to the pad's exact native Pin centre (the native trace-contact point).
/// The grid path geometry itself stays untouched at the landing.
fn push_landing_connector(
    geometry: &mut super::grid_path::GridPathGeometry,
    landing: &RungEndpoint,
    center: &RungEndpoint,
    board: &BoardAdapter,
    net: NetNo,
) -> Result<(), String> {
    if (landing.point[0] - center.point[0]).abs() <= 1e-9
        && (landing.point[1] - center.point[1]).abs() <= 1e-9
    {
        return Ok(());
    }
    if landing.layer != center.layer {
        return Err("blocked-pad landing connector layer differs from the pad layer".into());
    }
    let layer_no = board.board.layer_structure.get_no(&landing.layer);
    if layer_no < 0 {
        return Err("blocked-pad landing connector uses an unknown layer".into());
    }
    let width_mm =
        2.0 * board.board.rules.get_trace_half_width(net, layer_no) as f64 / board.units_per_mm;
    if !width_mm.is_finite() || width_mm <= 0.0 {
        return Err("blocked-pad landing connector has an invalid trace width".into());
    }
    geometry.segments.push(super::grid_path::GridPathSegment {
        start_mm: landing.point,
        end_mm: center.point,
        layer: landing.layer.clone(),
        width_mm,
    });
    Ok(())
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
    let mut obstacle_result = {
        super::native_obstacles::build_obstacle_map(
            board,
            request,
            window,
            grid_step_board,
            &request.route_layers,
            own_nets,
            clearance_class,
            via_clearance_class,
        )?
    };

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
    let mut source_cells = source_endpoints
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
    let mut targets = target_endpoints
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
    // #479 blocked-terminal seeding followed by the source C5 endpoint
    // exemptions and the ±10 allowed block -- all BEFORE the search, over the
    // fully stamped map, in source `route_net_with_obstacles` order: augment
    // endpoints -> endpoint_exempt -> allowed rects -> source/target override.
    let bounds = (gx0, gy0, gx1, gy1);
    let (extra_sources, mut landing_aliases) = augment_blocked_pad_terminals(
        source_endpoints,
        &request.same_net_pads,
        request.net_id,
        &obstacles,
        board,
        &request.route_layers,
        request.rules.track_width,
        grid_step_board,
        bounds,
    )?;
    source_cells.extend(extra_sources);
    let (extra_targets, target_aliases) = augment_blocked_pad_terminals(
        target_endpoints,
        &request.same_net_pads,
        request.net_id,
        &obstacles,
        board,
        &request.route_layers,
        request.rules.track_width,
        grid_step_board,
        bounds,
    )?;
    targets.extend(extra_targets);
    landing_aliases.extend(target_aliases);
    let exempt_radius = (((request.rules.track_width + request.rules.clearance) * upm)
        / grid_step_board as f64) as i32;
    let exempt_positions = source_cells
        .iter()
        .chain(targets.iter())
        .map(|&(x, y, _)| (x, y))
        .collect::<Vec<_>>();
    obstacles.set_endpoint_exempt(exempt_positions, exempt_radius);
    let allow_radius = 10;
    for &(x, y, _) in source_cells.iter().chain(targets.iter()) {
        obstacles.add_allowed_rect(
            x.saturating_sub(allow_radius).max(gx0),
            y.saturating_sub(allow_radius).max(gy0),
            x.saturating_add(allow_radius).min(gx1),
            y.saturating_add(allow_radius).min(gy1),
        );
    }
    for &(x, y, l) in source_cells.iter().chain(targets.iter()) {
        obstacles.add_source_target_cell(x, y, l as usize);
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
        forward_frontier: attempt.forward_frontier,
        backward_frontier: attempt.backward_frontier,
        forward_iterations: attempt.forward_iterations,
        backward_iterations: attempt.backward_iterations,
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
        landing_aliases,
        via_unblock_blame: BTreeSet::new(),
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

/// Source `single_ended_routing:2850..2859` declined-tap blame keystone: when the
/// in-pad via tap DECLINED and NOTHING was in flight yet, ask the `awx::via_blocker`
/// helper for the EXACT foreign net whose mature Trace/Via copper boxes the pad
/// (queried at the pad's copper centre). The helper returns a mature nearest
/// foreign Trace/Via net only, so no Pin blocker is ever claimed from code and no
/// blame is fabricated when it finds nothing.
fn record_via_unblock_blame(
    request: &RungRequest,
    outcome: &mut GridSearchResult,
    pad: &super::whole_context::ContextPad,
    inflight_vias: &[super::whole_context::CopperVia],
    inflight_segments: &[super::whole_context::CopperSegment],
) {
    // Source guard: blame is only attributable before any in-flight via/segment
    // existed, since a later decline may be caused by the in-flight copper itself.
    if !inflight_vias.is_empty() || !inflight_segments.is_empty() {
        return;
    }
    if let Some(blocker) = super::via_blocker::find_via_position_blocker(
        &request.board,
        pad.point,
        &request.rules,
        request.net_id,
        &BTreeSet::from([0]),
    ) {
        outcome.via_unblock_blame.insert(blocker);
    }
}

/// The via-pad tap backend returns placement geometry, while the native source
/// must retain which exact ContextPad caused that placement. `ContextPad.key`
/// is the native pad reference used by `BoardSnapshot`; `BoardAdapter::pads`
/// exposes that same reference with its actual Pin ItemKey.
#[derive(Clone)]
struct OwnedTapResult {
    tap: super::via_pad_tap::TapResult,
    pad_owner: fr_engine::board::ItemKey,
}

fn native_context_pad_owner(
    board: &BoardAdapter,
    pad: &super::whole_context::ContextPad,
) -> Result<fr_engine::board::ItemKey, String> {
    let mut owners = board
        .pads()
        .into_iter()
        .filter(|native| {
            native.reference == pad.key && native.net == pad.net && native.point == pad.point
        })
        .filter_map(|native| native.key)
        .collect::<Vec<_>>();
    owners.sort_unstable();
    owners.dedup();
    match owners.as_slice() {
        [owner] => Ok(*owner),
        _ => Err("via-pad tap ContextPad has no unique native Pin owner".into()),
    }
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
) -> Result<Option<OwnedTapResult>, String> {
    let Some(pad_layer) = pad.layers.iter().find(|layer| {
        !layer.starts_with('*')
            && request.board.board.layer_structure.get_no(layer) >= 0
            && request.route_layers.contains(layer)
    }) else {
        record_via_unblock_blame(
            request,
            outcome,
            pad,
            inflight_vias.as_slice(),
            inflight_segments.as_slice(),
        );
        return Ok(None);
    };
    let Some(layer_index) = request
        .route_layers
        .iter()
        .position(|layer| layer == pad_layer)
    else {
        record_via_unblock_blame(
            request,
            outcome,
            pad,
            inflight_vias.as_slice(),
            inflight_segments.as_slice(),
        );
        return Ok(None);
    };
    let mut backend = NativeTapBackend {
        board: &request.board,
    };
    // Evaluate the tap into an owned result first so no immutable borrow of
    // `outcome` survives into the decline branch, where blame needs `&mut outcome`.
    let tap_result = super::via_pad_tap::place_shrunk_via_in_pad(super::via_pad_tap::TapRequest {
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
    })?;
    let Some(tap) = tap_result else {
        record_via_unblock_blame(
            request,
            outcome,
            pad,
            inflight_vias.as_slice(),
            inflight_segments.as_slice(),
        );
        return Ok(None);
    };
    let pad_owner = native_context_pad_owner(&request.board, pad)?;
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
    Ok(Some(OwnedTapResult { tap, pad_owner }))
}

/// Seed a placed tap's escape-via cell as a source/target on EVERY route layer,
/// matching source `new_sources = list(sources) + [(vgx, vgy, li) for li in
/// range(len(layer_names))]`. Without this the retried A* can never terminate
/// on the via and the whole unblock is inert.
fn extend_with_tap_cells(cells: &mut Vec<(i32, i32, u8)>, tap: &OwnedTapResult, layers: usize) {
    for layer in 0..layers {
        cells.push((tap.tap.grid_cell.0, tap.tap.grid_cell.1, layer as u8));
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
) -> Result<Option<(Vec<(i32, i32, u8)>, Vec<OwnedTapResult>)>, String> {
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
    let mut placed = Vec::<OwnedTapResult>::new();
    let mut source_tapped = false;
    let mut target_tapped = false;

    // backward probe (from targets) exhausted -> the TARGET pad is boxed.
    if target_stuck {
        if let Some(pad) = pads_near_grid_cells(request, &target_cells, outcome.grid_step_board)
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
        if let Some(pad) = pads_near_grid_cells(request, &source_cells, outcome.grid_step_board)
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
        .filter(|tap| path_uses_tap(&path, tap, &request.route_layers, &request.board))
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

fn tap_reaches_terminal_cell(
    tap: &OwnedTapResult,
    cell: (i32, i32, u8),
    route_layers: &[String],
    board: &BoardAdapter,
) -> bool {
    if (cell.0, cell.1) != tap.tap.grid_cell {
        return false;
    }
    let Some(layer_name) = route_layers.get(cell.2 as usize) else {
        return false;
    };
    let layer_no = board.board.layer_structure.get_no(layer_name);
    let first = tap
        .tap
        .via
        .layers
        .first()
        .map(|layer| board.board.layer_structure.get_no(layer));
    let last = tap
        .tap
        .via
        .layers
        .last()
        .map(|layer| board.board.layer_structure.get_no(layer));
    let through_barrel = matches!((first, last), (Some(a), Some(b)) if a >= 0 && b >= 0 && layer_no >= a.min(b) && layer_no <= a.max(b));
    let stub_access = tap
        .tap
        .stub_segments
        .iter()
        .any(|segment| segment.layer == *layer_name);
    let on_pad_layer = cell.2 as usize == tap.tap.pad_layer_index;
    (through_barrel || stub_access) && (!on_pad_layer || stub_access)
}

fn tap_for_path_terminal<'a>(
    taps: &'a [OwnedTapResult],
    cell: (i32, i32, u8),
    route_layers: &[String],
    board: &BoardAdapter,
    net: NetNo,
) -> Result<Option<&'a OwnedTapResult>, String> {
    let mut matching = taps
        .iter()
        .filter(|tap| tap_reaches_terminal_cell(tap, cell, route_layers, board));
    let Some(first) = matching.next() else {
        return Ok(None);
    };
    let group = board.board.connected_set(first.pad_owner, net, false);
    if matching.any(|tap| !group.contains(board.board.item(tap.pad_owner).id())) {
        return Err("path terminal cell aliases taps from disconnected native pads".into());
    }
    Ok(Some(first))
}

fn path_uses_tap(
    path: &[(i32, i32, u8)],
    tap: &OwnedTapResult,
    route_layers: &[String],
    board: &BoardAdapter,
) -> bool {
    // Source `ends = (p2[0], p2[-1])`: retain a via iff an actual terminal
    // lands on its exact cell and route layer, through the physical barrel or
    // the pad-layer escape stub. Pads sharing the XY grid cell are not aliases.
    path.first()
        .into_iter()
        .chain(path.last())
        .any(|&cell| tap_reaches_terminal_cell(tap, cell, route_layers, board))
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
        request.net_id,
        &start_candidates,
        path_start,
        &request.route_layers,
        &trial,
        outcome.grid_step_board,
        &tap_results,
        &outcome.landing_aliases,
    )?
    .ok_or("awx_grid started at a cell without a source endpoint")?;
    let target = endpoint_for_path_terminal(
        request.net_id,
        &target_candidates,
        path_end,
        &request.route_layers,
        &trial,
        outcome.grid_step_board,
        &tap_results,
        &outcome.landing_aliases,
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
        through_hole_positions.insert(tap.tap.grid_cell);
        registered_via_sizes.extend(
            tap.tap
                .registered_via_sizes
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
    let mut geometry = super::grid_path::path_to_geometry(super::grid_path::GridPathRequest {
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
    // #479: for a seeded blocked-pad landing, bridge the landed cell to the
    // pad's exact native Pin centre with a same-net full-width connector that
    // stays inside the pad copper. The grid geometry itself stays land-exact.
    if let Some(center) = outcome.landing_aliases.get(&path_start) {
        push_landing_connector(&mut geometry, &start, center, &trial, request.net_id)?;
    }
    if let Some(center) = outcome.landing_aliases.get(&path_end) {
        push_landing_connector(&mut geometry, &target, center, &trial, request.net_id)?;
    }
    let source_trace_ids = request
        .board
        .board
        .get_traces()
        .into_iter()
        .collect::<BTreeSet<_>>();
    for tap in &tap_results {
        materialize_tap(&mut trial, &tap.tap, request.net_id)?;
    }
    materialize_grid_geometry(&mut trial, &geometry, request.net_id, &[])?;
    // Native trace contact is `*point == other.center(self)`: a newly materialized
    // trace end that sits in the same-net pin/via copper but not exactly at the
    // centre is physically connected yet not contact-connected. Bridge only the
    // traces this transaction just added (difference against the source board),
    // to the exact drill centre, width clamped to the room inside the pad's
    // copper. Previously existing source/runtime traces are never rescanned or
    // modified.
    let new_trace_ids = trial
        .board
        .get_traces()
        .into_iter()
        .filter(|key| !source_trace_ids.contains(key))
        .collect::<Vec<_>>();
    let _bridges = trial
        .board
        .bridge_trace_ends_to_drill_centers_for(&new_trace_ids);
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
            tap.tap
                .stub_segments
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
            point: tap.tap.via.point,
            diameter: tap.tap.via.diameter,
            drill: tap.tap.via.drill,
            layers: tap.tap.via.layers.clone(),
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
        let layer = route_layers.iter().position(|name| name == &endpoint.layer);
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
/// terminated on, then against a #479 seeded blocked-pad landing. A tap is a
/// same-net pad via, so the honest endpoint is the via's physical point on the
/// path's actual arrival layer; the existing tap materialization and stub
/// copper complete the pad connection. A seeded landing resolves to the ACTUAL
/// landing endpoint at the selected grid cell (NOT the pad centre -- that would
/// drag source geometry); its native Pin-centre bridge is materialized later.
fn endpoint_for_path_terminal(
    net: NetNo,
    candidates: &[RungEndpoint],
    cell: (i32, i32, u8),
    route_layers: &[String],
    board: &BoardAdapter,
    step: i32,
    taps: &[OwnedTapResult],
    landing_aliases: &BTreeMap<(i32, i32, u8), RungEndpoint>,
) -> Result<Option<RungEndpoint>, String> {
    if let Some(endpoint) = endpoint_for_grid_cell(candidates, cell, route_layers, board, step)? {
        return Ok(Some(endpoint));
    }
    if let Some(tap) = tap_for_path_terminal(taps, cell, route_layers, board, net)? {
        // Return the chosen tap's exact physical via point on the path's ACTUAL
        // arrival layer, retaining that same tap's native Pad owner.
        let layer = route_layers
            .get(cell.2 as usize)
            .cloned()
            .ok_or("via-pad tap arrival layer is outside the route layers")?;
        return Ok(Some(RungEndpoint {
            point: tap.tap.via.point,
            layer,
        }));
    }
    if landing_aliases.contains_key(&cell) {
        let layer = route_layers
            .get(cell.2 as usize)
            .cloned()
            .ok_or("blocked-pad landing layer is outside the route layers")?;
        let step_mm = step as f64 / board.units_per_mm;
        return Ok(Some(RungEndpoint {
            point: [cell.0 as f64 * step_mm, cell.1 as f64 * step_mm],
            layer,
        }));
    }
    Ok(None)
}

fn materialize_grid_geometry(
    board: &mut BoardAdapter,
    geometry: &super::grid_path::GridPathGeometry,
    net: NetNo,
    exact_endpoints: &[ExactEndpointOverride],
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
        let start = geometry_endpoint(board, seg.start_mm, &seg.layer, exact_endpoints)?;
        let end = geometry_endpoint(board, seg.end_mm, &seg.layer, exact_endpoints)?;
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

/// Bridge endpoints of the provided transaction-created traces to same-net
/// pin or via centres only when the clamped-width bridge fits inside the
/// native drill item's actual copper shape on that layer.
fn bridge_welded_trace_ends_to_drill_centers(
    board: &mut BoardAdapter,
    split_pieces: &[fr_engine::board::ItemKey],
    net: NetNo,
) -> usize {
    let mut bridges = Vec::<(
        EnginePoint,
        EnginePoint,
        i32,
        i32,
        Vec<NetNo>,
        i32,
        FixedState,
    )>::new();
    let pads = board
        .board
        .get_pins()
        .into_iter()
        .chain(board.board.get_vias())
        .collect::<Vec<_>>();
    for &key in split_pieces {
        let Some(trace) = board.board.try_item(key) else {
            continue;
        };
        if !trace.is_trace() || !trace.is_on_board() || !trace.contains_net(net) {
            continue;
        }
        let layer = trace.trace().layer();
        let half_width = trace.trace().half_width();
        let nets = trace.net_numbers().to_vec();
        let clearance_class = trace.clearance_class();
        let fixed_state = trace.fixed_state();
        let ends = [trace.first_corner(), trace.last_corner()];
        for end in ends {
            let contacts = board.board.trace_normal_contacts_at(key, &end, false);
            if contacts.iter().any(|contact| {
                let item = board.board.item(contact);
                item.is_pin() || item.is_via()
            }) {
                continue;
            }
            let candidates = board.board.pick_items(&end, layer, None);
            let candidate_keys = candidates.iter().collect::<BTreeSet<_>>();
            for &pad_key in &pads {
                if !candidate_keys.contains(&pad_key) {
                    continue;
                }
                let pad = board.board.item(pad_key);
                if !pad.contains_net(net) || !pad.is_on_layer(&board.board, layer) {
                    continue;
                }
                let center = pad.center(&board.board);
                if center == end {
                    continue;
                }
                // This trace itself may already be the Q-to-centre seed.
                // Contacts at its endpoint do not include the trace itself.
                if (trace.first_corner() == end && trace.last_corner() == center)
                    || (trace.last_corner() == end && trace.first_corner() == center)
                {
                    break;
                }
                let Some(shape) = pad.drill_shape_on_layer(&board.board, layer) else {
                    continue;
                };
                if shape
                    .as_polygon_shape()
                    .is_some_and(|polygon| !polygon.is_convex())
                {
                    continue;
                }
                if !shape.contains(&center) || !shape.contains(&end) {
                    continue;
                }
                // The island-tap weld may already have seeded this exact
                // Q-to-centre stub. Reuse that local contact instead of
                // inserting a duplicate bridge for the split piece.
                if contacts.iter().any(|contact| {
                    let item = board.board.item(contact);
                    if !item.is_trace()
                        || !item.is_on_board()
                        || !item.contains_net(net)
                        || item.trace().layer() != layer
                    {
                        return false;
                    }
                    let (first, last) = (item.trace().first_corner(), item.trace().last_corner());
                    (first == end && last == center) || (first == center && last == end)
                }) {
                    break;
                }
                let center_float = center.to_float();
                let end_float = end.to_float();
                let room = shape
                    .border_distance(&center_float)
                    .min(shape.border_distance(&end_float));
                let bridge_half_width = half_width.min(room.floor() as i32);
                if bridge_half_width <= 0 {
                    continue;
                }
                if !bridges.iter().any(|(a, b, bridge_layer, ..)| {
                    *bridge_layer == layer
                        && ((*a == end && *b == center) || (*a == center && *b == end))
                }) {
                    bridges.push((
                        end.clone(),
                        center,
                        layer,
                        bridge_half_width,
                        nets.clone(),
                        clearance_class,
                        fixed_state,
                    ));
                }
                break;
            }
        }
    }

    let mut inserted = 0;
    for (start, end, layer, half_width, nets, clearance_class, fixed_state) in bridges {
        let polyline = Polyline::from_points(&[start, end]);
        if board
            .board
            .insert_trace_without_cleaning(
                polyline,
                layer,
                half_width,
                &nets,
                clearance_class,
                fixed_state,
            )
            .is_some()
        {
            inserted += 1;
        }
    }
    inserted
}

/// Resolve a geometry endpoint to its native `Point`. An island-tap weld hands
/// over the engine's EXACT projected point; using it verbatim keeps the new
/// trace in exact contact with the split copper instead of quantizing the
/// millimetres back through `BoardAdapter::point` to an `Int`.
fn geometry_endpoint(
    board: &BoardAdapter,
    point_mm: [f64; 2],
    layer: &str,
    exact_endpoints: &[ExactEndpointOverride],
) -> Result<EnginePoint, String> {
    for (mm, exact_layer, exact) in exact_endpoints {
        if exact_layer == layer
            && (mm[0] - point_mm[0]).abs() <= 1e-9
            && (mm[1] - point_mm[1]).abs() <= 1e-9
        {
            return Ok(exact.clone());
        }
    }
    board.point(point_mm)
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
    let routed_class = {
        let matrix = &board.board.rules.clearance_matrix;
        let class_count = matrix.get_class_count();
        let prefix = format!("native_rung_{net}_");
        let normalize = |value: i32| {
            let value = value.max(0);
            if value % 2 != 0 {
                if value == i32::MAX {
                    value - 1
                } else {
                    value + 1
                }
            } else {
                value
            }
        };
        let mut reusable = None;
        for candidate in 0..class_count {
            if !matrix
                .get_name(candidate)
                .is_some_and(|name| name.starts_with(&prefix))
            {
                continue;
            }
            let equivalent = (0..layer_count as i32).all(|layer| {
                (0..class_count).all(|other| {
                    if other == candidate {
                        let expected_self = normalize(
                            matrix
                                .get_value(base_class, base_class, layer, false)
                                .max(clearance),
                        );
                        let expected_old_candidate = normalize(
                            matrix
                                .get_value(base_class, other, layer, false)
                                .max(matrix.get_value(other, base_class, layer, false))
                                .max(clearance),
                        );
                        expected_self == expected_old_candidate
                            && matrix.get_value(candidate, candidate, layer, false) == expected_self
                    } else {
                        let expected = normalize(
                            matrix
                                .get_value(base_class, other, layer, false)
                                .max(matrix.get_value(other, base_class, layer, false))
                                .max(clearance),
                        );
                        matrix.get_value(candidate, other, layer, false) == expected
                            && matrix.get_value(other, candidate, layer, false) == expected
                    }
                })
            });
            if equivalent {
                reusable = Some(candidate);
                break;
            }
        }
        if let Some(candidate) = reusable {
            candidate
        } else {
            let count_before = class_count;
            let class_name = format!("native_rung_{net}_{count_before}");
            let matrix = &mut board.board.rules_mut().clearance_matrix;
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
            routed_class
        }
    };

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

/// Minimal policy binding the old public wrapper to the in-run entry of the
/// shared endpoint-set routing core. The core itself is identical for both; the
/// only differences are which board is the frozen baseline, which nets are
/// protected, and whether the SOURCE-required split of a same-net THIS-RUN
/// trace at the selected island-tap endpoint is authorised.
struct EndpointSetPolicy<'a> {
    /// Actual captured input board. Imported copper/pins/rules/boundary must be
    /// preserved against it and it is never mutated.
    baseline: &'a BoardAdapter,
    protected: &'a BTreeSet<String>,
    /// In-run entry only: allow the authorised split of a same-net trace that
    /// belongs to this run (id absent from `baseline`) and is not protected.
    split_this_run_trace: bool,
}

fn route_multilayer_endpoint_sets_core(
    request: RungRequest,
    from_item: fr_engine::board::ItemKey,
    to_item: fr_engine::board::ItemKey,
    start_candidates: &[RungEndpoint],
    target_candidates: &[RungEndpoint],
    policy: EndpointSetPolicy<'_>,
) -> Result<
    (
        Option<BoardAdapter>,
        serde_json::Value,
        Option<super::braid_connect::RungReport>,
    ),
    String,
> {
    route_multilayer_endpoint_sets_inner(
        request,
        from_item,
        to_item,
        start_candidates,
        target_candidates,
        policy,
    )
}

fn route_multilayer_endpoint_sets_inner(
    mut request: RungRequest,
    from_item: fr_engine::board::ItemKey,
    to_item: fr_engine::board::ItemKey,
    start_candidates: &[RungEndpoint],
    target_candidates: &[RungEndpoint],
    policy: EndpointSetPolicy<'_>,
) -> Result<
    (
        Option<BoardAdapter>,
        serde_json::Value,
        Option<super::braid_connect::RungReport>,
    ),
    String,
> {
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
        return Ok((
            None,
            serde_json::json!({
                "accepted": false,
                "reason": "endpoint candidate set has no active route layer"
            }),
            None,
        ));
    }
    request.start = starts[0].clone();
    request.end = targets[0].clone();
    request.b_alternatives = targets.iter().skip(1).cloned().collect();
    request.rules.validate()?;
    let net = request.board.net_number(&request.net)?;
    let class = request
        .board
        .board
        .rules
        .net_class_of(net)
        .get_trace_clearance_class();
    let (mut trial, control, _) = {
        prepare_control(&request)?
    };
    let mut outcome = {
        source_grid_search_with_endpoints(
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
        )?
    };
    let mut report = serde_json::json!({"expanded":outcome.iterations,"path_cells":outcome.path.as_ref().map_or(0,Vec::len),"accepted":false});
    // Private endpoint-set evidence for failed/no-reduction runs. Keep this
    // attached to the existing private JSON report; public route types stay
    // unchanged.
    report["_endpoint_diagnostic"] = serde_json::json!({
        "stage": "grid_search",
        "planned_from_item_id": trial.board.item(from_item).id().0,
        "planned_to_item_id": trial.board.item(to_item).id().0,
        "planned_from_group": stable_group_members(&trial, from_item, net).iter().map(|id| id.0).collect::<Vec<_>>(),
        "planned_to_group": stable_group_members(&trial, to_item, net).iter().map(|id| id.0).collect::<Vec<_>>(),
        "actual_taps": [],
        "search_evidence": {
            "forward_probe_iterations": outcome.forward_probe_iterations,
            "backward_probe_iterations": outcome.backward_probe_iterations,
            "forward_iterations": outcome.forward_iterations,
            "backward_iterations": outcome.backward_iterations,
            "sources_cell_count": outcome.source_cells.len(),
            "targets_cell_count": outcome.target_cells.len(),
            "window_board": [outcome.window.ll.x, outcome.window.ll.y, outcome.window.ur.x, outcome.window.ur.y],
            "grid_step_board": outcome.grid_step_board,
            "early_frontier_exhausted_below_probe_budget": outcome.path.is_none() && (outcome.forward_probe_iterations < 5_000 || outcome.backward_probe_iterations < 5_000),
            "probe_budget": 5_000,
            "interpretation": "probe exhaustion classification is policy evidence, not proof of physical unreachability"
        }
    });
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
            report["_endpoint_diagnostic"]["stage"] = "deadline_before_via_pad_unblock".into();
            report["_endpoint_diagnostic"]["search_evidence"]["failfast"] = true.into();
            report["_endpoint_diagnostic"]["search_evidence"]["failfast_reason"] =
                "deadline elapsed".into();
            return Ok((None, report, None));
        }
        let unblock_result = {
            try_via_pad_unblock(&request, &mut outcome)?
        };
        match unblock_result {
            Some((path, taps)) => {
                tap_results = taps;
                path
            }
            None => {
                report["expanded"] = outcome.iterations.into();
                report["path_cells"] = outcome.path.as_ref().map_or(0, Vec::len).into();
                report["reason"] =
                    "source GridRouter and via-pad unblocking returned no path".into();
                let early_frontier = outcome.path.is_none()
                    && (outcome.forward_probe_iterations < 5_000
                        || outcome.backward_probe_iterations < 5_000);
                report["_endpoint_diagnostic"]["search_evidence"] = serde_json::json!({
                    "forward_probe_iterations": outcome.forward_probe_iterations,
                    "backward_probe_iterations": outcome.backward_probe_iterations,
                    "forward_iterations": outcome.forward_iterations,
                    "backward_iterations": outcome.backward_iterations,
                    "sources_cell_count": outcome.source_cells.len(),
                    "targets_cell_count": outcome.target_cells.len(),
                    "window_board": [outcome.window.ll.x, outcome.window.ll.y, outcome.window.ur.x, outcome.window.ur.y],
                    "grid_step_board": outcome.grid_step_board,
                    "early_frontier_exhausted_below_probe_budget": early_frontier,
                    "forward_probe_budget_exhausted": outcome.forward_probe_iterations >= 5_000,
                    "backward_probe_budget_exhausted": outcome.backward_probe_iterations >= 5_000,
                    "probe_budget": 5000,
                    "interpretation": if early_frontier { "one or both probe searches exhausted before budget; this triggers via-pad-unblock policy and does not prove physical unreachability" } else { "probe budget exhaustion or unavailable probe evidence; does not prove physical unreachability" }
                });
                report["_endpoint_diagnostic"]["stage"] = "search_no_path".into();
                // PRIVATE keystone, not user schema: the exact foreign net(s)
                // whose mature Trace/Via copper boxed a pad while the in-pad tap
                // declined with nothing in flight (source `_via_unblock_blame`).
                // The endpoint owner strips this before public logging and hands
                // it to `RecoveryState`; existing report keys are unchanged.
                report["_via_unblock_blame"] = serde_json::json!(outcome
                    .via_unblock_blame
                    .iter()
                    .copied()
                    .collect::<Vec<NetNo>>());
                let frontier = frontier_report(&request, &outcome, trial.units_per_mm);
                return Ok((None, report, Some(frontier)));
            }
        }
    };
    report["expanded"] = outcome.iterations.into();
    report["path_cells"] = path.len().into();
    report["_endpoint_diagnostic"]["actual_taps"] = serde_json::json!(tap_results.iter().map(|tap| serde_json::json!({
        "point_mm": tap.tap.via.point,
        "diameter_mm": tap.tap.via.diameter,
        "drill_mm": tap.tap.via.drill,
        "layers": tap.tap.via.layers,
        "pad_owner_id": trial.board.item(tap.pad_owner).id().0,
        "grid_cell": tap.tap.grid_cell,
        "stub_segments": tap.tap.stub_segments.iter().map(|s| serde_json::json!({"a":s.a,"b":s.b,"layer":s.layer,"width":s.width})).collect::<Vec<_>>(),
    })).collect::<Vec<_>>());
    if request.deadline.is_some_and(|d| Instant::now() >= d) {
        report["reason"] = "deadline elapsed during source GridRouter search".into();
        report["_endpoint_diagnostic"]["stage"] = "deadline_search".into();
        return Ok((None, report, None));
    }
    let (Some(&path_start), Some(&path_end)) = (path.first(), path.last()) else {
        report["reason"] = "source GridRouter returned an empty path".into();
        let frontier = frontier_report(&request, &outcome, trial.units_per_mm);
        return Ok((None, report, Some(frontier)));
    };
    let Some(mut start) = endpoint_for_path_terminal(
        net,
        &starts,
        path_start,
        &request.route_layers,
        &trial,
        outcome.grid_step_board,
        &tap_results,
        &outcome.landing_aliases,
    )?
    else {
        report["reason"] = "GridRouter path starts outside the source endpoint set".into();
        return Ok((None, report, None));
    };
    let Some(mut target) = endpoint_for_path_terminal(
        net,
        &targets,
        path_end,
        &request.route_layers,
        &trial,
        outcome.grid_step_board,
        &tap_results,
        &outcome.landing_aliases,
    )?
    else {
        report["reason"] = "GridRouter path ends outside the target endpoint set".into();
        return Ok((None, report, None));
    };
    report["start_endpoint"] = serde_json::json!({
        "point": start.point,
        "layer": start.layer,
    });
    report["end_endpoint"] = serde_json::json!({
        "point": target.point,
        "layer": target.layer,
    });
    // In-run only: the selected start/target endpoint may land on the interior
    // centreline of a same-net THIS-RUN trace (the island tap). The SOURCE-
    // required weld splits that trace at the EXACTLY projected native point and
    // relocates the route endpoint onto the real split contact so the new trace
    // physically connects. Baseline/protected copper is never split; the stable
    // before-split Pin/Via group identities are snapshotted here so the merge
    // proof does not depend on the possibly-removed representative key.
    let mut stable_from = None;
    let mut stable_to = None;
    let mut selected_owner_keys = None;
    let mut welded_pieces = Vec::new();
    let mut weld_seed_bridges = 0usize;
    let mut exact_endpoints: Vec<ExactEndpointOverride> = Vec::new();
    if policy.split_this_run_trace {
        // A via-pad retry may start or land on a newly created, not-yet-
        // materialized via. Its pad representative remains the only native
        // owner in the working board; ordinary routed terminals resolve to
        // the exact Pin/Via/Trace candidate selected by the search.
        let source_is_candidate = endpoint_for_grid_cell(
            &starts,
            path_start,
            &request.route_layers,
            &trial,
            outcome.grid_step_board,
        )?
        .is_some();
        let target_is_candidate = endpoint_for_grid_cell(
            &targets,
            path_end,
            &request.route_layers,
            &trial,
            outcome.grid_step_board,
        )?
        .is_some();
        // Match the same first tap that endpoint_for_path_terminal resolves
        // after exhausting ordinary candidates. Keep its real Pad ItemKey;
        // source/target side alone does not identify the SMD pad selected from
        // the nearby-cell rescue search.
        let source_tap = if source_is_candidate {
            None
        } else {
            tap_for_path_terminal(&tap_results, path_start, &request.route_layers, &trial, net)?
        };
        let target_tap = if target_is_candidate {
            None
        } else {
            tap_for_path_terminal(&tap_results, path_end, &request.route_layers, &trial, net)?
        };
        let source_is_tap = source_tap.is_some();
        let target_is_tap = target_tap.is_some();
        let source_landing = (!source_is_candidate && !source_is_tap)
            .then(|| outcome.landing_aliases.get(&path_start))
            .flatten();
        let target_landing = (!target_is_candidate && !target_is_tap)
            .then(|| outcome.landing_aliases.get(&path_end))
            .flatten();
        let source_owner_endpoint = source_landing.unwrap_or(&start);
        let target_owner_endpoint = target_landing.unwrap_or(&target);
        report["_endpoint_diagnostic"]["selected_source"] = serde_json::json!({
            "point_mm": start.point, "layer": start.layer,
            "ordinary_candidate": source_is_candidate, "tap": source_is_tap,
            "landing_alias": source_landing.map(|e| serde_json::json!({"point_mm":e.point,"layer":e.layer})),
        });
        report["_endpoint_diagnostic"]["selected_target"] = serde_json::json!({
            "point_mm": target.point, "layer": target.layer,
            "ordinary_candidate": target_is_candidate, "tap": target_is_tap,
            "landing_alias": target_landing.map(|e| serde_json::json!({"point_mm":e.point,"layer":e.layer})),
        });
        let source_owner = if let Some(tap) = source_tap {
            SelectedEndpointOwner {
                item: tap.pad_owner,
                exact_point: None,
            }
        } else {
            match selected_endpoint_owner(
                &trial,
                policy.baseline,
                net,
                source_owner_endpoint,
                request.rules.grid_step,
            ) {
                Ok(Some(owner)) => owner,
                Ok(None) => {
                    report["_endpoint_diagnostic"]["stage"] = "owner_resolution".into();
                    report["reason"] = "selected source endpoint has no exact native owner".into();
                    return Ok((None, report, None));
                }
                Err(reason) => {
                    report["_endpoint_diagnostic"]["stage"] = "owner_resolution".into();
                    report["reason"] = reason.into();
                    return Ok((None, report, None));
                }
            }
        };
        let target_owner = if let Some(tap) = target_tap {
            SelectedEndpointOwner {
                item: tap.pad_owner,
                exact_point: None,
            }
        } else {
            match selected_endpoint_owner(
                &trial,
                policy.baseline,
                net,
                target_owner_endpoint,
                request.rules.grid_step,
            ) {
                Ok(Some(owner)) => owner,
                Ok(None) => {
                    report["_endpoint_diagnostic"]["stage"] = "owner_resolution".into();
                    report["reason"] = "selected target endpoint has no exact native owner".into();
                    return Ok((None, report, None));
                }
                Err(reason) => {
                    report["_endpoint_diagnostic"]["stage"] = "owner_resolution".into();
                    report["reason"] = reason.into();
                    return Ok((None, report, None));
                }
            }
        };
        stable_from = Some(stable_group_members(&trial, source_owner.item, net));
        stable_to = Some(stable_group_members(&trial, target_owner.item, net));
        selected_owner_keys = Some((source_owner.item, target_owner.item));
        let owner_record = |key: fr_engine::board::ItemKey, exact: &Option<EnginePoint>| {
            let item = trial.board.item(key);
            serde_json::json!({
                "item_id": item.id().0,
                "item_type": format!("{:?}", item.board_item_type()),
                "layer": item.first_layer(&trial.board),
                "exact_native_point": exact.as_ref().map(|p| format!("{:?}", p)),
            })
        };
        report["_endpoint_diagnostic"]["selected_source"]["owner"] =
            owner_record(source_owner.item, &source_owner.exact_point);
        report["_endpoint_diagnostic"]["selected_target"]["owner"] =
            owner_record(target_owner.item, &target_owner.exact_point);
        report["_endpoint_diagnostic"]["selected_source"]["before_group"] =
            serde_json::json!(stable_from
                .as_ref()
                .map(|ids| ids.iter().map(|id| id.0).collect::<Vec<_>>()));
        report["_endpoint_diagnostic"]["selected_target"]["before_group"] =
            serde_json::json!(stable_to
                .as_ref()
                .map(|ids| ids.iter().map(|id| id.0).collect::<Vec<_>>()));
        report["_endpoint_diagnostic"]["stage"] = "owner_resolved".into();
        let source_weld_attempted =
            !source_is_tap && source_landing.is_none() && source_owner.exact_point.is_none();
        let source_weld = if source_weld_attempted {
            weld_island_tap_endpoint(
                &mut trial,
                policy.baseline,
                policy.protected,
                net,
                source_owner.item,
                &start,
                &mut welded_pieces,
            )
        } else {
            Ok(None)
        };
        match source_weld {
            Ok(Some((welded, exact, seed_bridges))) => {
                weld_seed_bridges += seed_bridges;
                report["_endpoint_diagnostic"]["source_weld"] = serde_json::json!({
                    "attempted": true,
                    "result": "some",
                    "point_mm": welded.point,
                    "exact_native_point": exact.as_ref().map(|point| format!("{:?}", point)),
                });
                report["start_endpoint"] = serde_json::json!({
                    "point": welded.point,
                    "layer": welded.layer,
                });
                if let Some(exact) = exact {
                    exact_endpoints.push((welded.point, welded.layer.clone(), exact));
                }
                start = welded;
            }
            Ok(None) => {
                report["_endpoint_diagnostic"]["source_weld"] = serde_json::json!({
                    "attempted": source_weld_attempted,
                    "result": if source_weld_attempted { "none" } else { "not_needed" },
                });
                if let Some(exact) = source_owner.exact_point {
                    let endpoint = source_owner_endpoint;
                    exact_endpoints.push((endpoint.point, endpoint.layer.clone(), exact));
                }
            }
            Err(reason) => {
                report["_endpoint_diagnostic"]["stage"] = "weld".into();
                report["reason"] = reason.into();
                return Ok((None, report, None));
            }
        }
        let target_weld_attempted =
            !target_is_tap && target_landing.is_none() && target_owner.exact_point.is_none();
        let target_weld = if target_weld_attempted {
            weld_island_tap_endpoint(
                &mut trial,
                policy.baseline,
                policy.protected,
                net,
                target_owner.item,
                &target,
                &mut welded_pieces,
            )
        } else {
            Ok(None)
        };
        match target_weld {
            Ok(Some((welded, exact, seed_bridges))) => {
                weld_seed_bridges += seed_bridges;
                report["_endpoint_diagnostic"]["target_weld"] = serde_json::json!({
                    "attempted": true,
                    "result": "some",
                    "point_mm": welded.point,
                    "exact_native_point": exact.as_ref().map(|point| format!("{:?}", point)),
                });
                report["end_endpoint"] = serde_json::json!({
                    "point": welded.point,
                    "layer": welded.layer,
                });
                if let Some(exact) = exact {
                    exact_endpoints.push((welded.point, welded.layer.clone(), exact));
                }
                target = welded;
            }
            Ok(None) => {
                report["_endpoint_diagnostic"]["target_weld"] = serde_json::json!({
                    "attempted": target_weld_attempted,
                    "result": if target_weld_attempted { "none" } else { "not_needed" },
                });
                if let Some(exact) = target_owner.exact_point {
                    let endpoint = target_owner_endpoint;
                    exact_endpoints.push((endpoint.point, endpoint.layer.clone(), exact));
                }
            }
            Err(reason) => {
                report["_endpoint_diagnostic"]["stage"] = "weld".into();
                report["reason"] = reason.into();
                return Ok((None, report, None));
            }
        }
    }
    let mut through_holes = super::native_grid_window::same_net_through_hole_positions(
        &trial,
        &[net],
        &request.free_via_pads,
        outcome.grid_step_board,
    )?;
    // A tapped via is a same-net through-hole barrel: skip it in the emitted
    // path's via-through-hole checks and carry its registered physical size.
    let mut registered_via_sizes = outcome.registered_via_sizes.clone();
    for tap in &tap_results {
        through_holes.insert(tap.tap.grid_cell);
        registered_via_sizes.extend(
            tap.tap
                .registered_via_sizes
                .iter()
                .map(|(&cell, &size)| (cell, size)),
        );
    }
    let mut geometry = super::grid_path::path_to_geometry(super::grid_path::GridPathRequest {
        path: &path,
        grid_step_board: outcome.grid_step_board,
        route_layers: &request.route_layers,
        start: &start,
        target: &target,
        board: &trial,
        net,
        via_candidates: control.via_rule.vias(),
        via_size_mm: request.rules.via_size,
        via_drill_mm: request.rules.via_drill,
        hole_to_hole_clearance_mm: request.rules.hole_to_hole_clearance,
        via_refit_rungs: &outcome.via_refit_rungs,
        registered_via_sizes: &registered_via_sizes,
        existing_vias: &request.existing_vias,
        through_hole_positions: &through_holes,
    })?;
    report["_endpoint_diagnostic"]["stage"] = "geometry_planned".into();
    report["_endpoint_diagnostic"]["planned_geometry"] = serde_json::json!({
        "segments": geometry.segments.iter().map(|s| serde_json::json!({"start_mm":s.start_mm,"end_mm":s.end_mm,"layer":s.layer,"width_mm":s.width_mm})).collect::<Vec<_>>(),
        "vias": geometry.vias.iter().map(|v| serde_json::json!({"point_mm":v.point_mm,"from_layer":v.from_layer,"to_layer":v.to_layer})).collect::<Vec<_>>(),
    });
    // #479: bridge a seeded blocked-pad landing to the pad's exact native Pin
    // centre (same-net, inside the pad copper); grid geometry stays land-exact.
    if let Some(center) = outcome.landing_aliases.get(&path_start) {
        push_landing_connector(&mut geometry, &start, center, &trial, net)?;
    }
    if let Some(center) = outcome.landing_aliases.get(&path_end) {
        push_landing_connector(&mut geometry, &target, center, &trial, net)?;
    }
    // Commit the kept unblock copper, then the grid geometry, so the landed
    // board carries the tapped vias the retry actually reached (#803).
    let source_trace_ids = request
        .board
        .board
        .get_traces()
        .into_iter()
        .collect::<BTreeSet<_>>();
    for tap in &tap_results {
        materialize_tap(&mut trial, &tap.tap, net)?;
    }
    materialize_grid_geometry(&mut trial, &geometry, net, &exact_endpoints)?;
    report["_endpoint_diagnostic"]["stage"] = "materialized".into();
    // Native trace contact is `*point == other.center(self)`: a newly materialized
    // trace end that sits in the same-net pin/via copper but not exactly at the
    // centre is physically connected yet not contact-connected. Bridge only the
    // traces this transaction just added (difference against the source board),
    // to the exact drill centre, width clamped to the room inside the pad's
    // copper. Previously existing source/runtime traces are never rescanned or
    // modified.
    let new_trace_ids = trial
        .board
        .get_traces()
        .into_iter()
        .filter(|key| !source_trace_ids.contains(key) && !welded_pieces.contains(key))
        .collect::<Vec<_>>();
    let bridges = bridge_welded_trace_ends_to_drill_centers(&mut trial, &new_trace_ids, net);
    let split_bridges = bridge_welded_trace_ends_to_drill_centers(&mut trial, &welded_pieces, net);
    let bridges = bridges + split_bridges;
    report["_endpoint_diagnostic"]["welded_split_piece_count"] = welded_pieces.len().into();
    report["_endpoint_diagnostic"]["split_contact_bridges"] = split_bridges.into();
    report["_endpoint_diagnostic"]["weld_seed_bridges"] = weld_seed_bridges.into();
    // In-run only: the operation must not ADD copper on a foreign working net,
    // NOR delete or mutate an existing FOREIGN working item (its geometry, net,
    // clearance class or fixed state). `assert_invariants` only guards the
    // frozen baseline/protected copper, so the working-board delta is checked
    // explicitly here.
    if policy.split_this_run_trace {
        let working_ids = request
            .board
            .board
            .get_items()
            .into_iter()
            .map(|key| request.board.board.item(key).id().0)
            .collect::<BTreeSet<_>>();
        if trial.board.get_items().into_iter().any(|key| {
            let item = trial.board.item(key);
            !working_ids.contains(&item.id().0)
                && (item.is_trace() || item.is_via())
                && !item.contains_net(net)
        }) {
            report["reason"] = "in-run route added copper on a foreign working net".into();
            return Ok((None, report, None));
        }
        // Existing foreign WORKING items (added this run by earlier steps) must
        // survive unchanged.
        let baseline_ids = policy
            .baseline
            .board
            .get_items()
            .into_iter()
            .map(|key| policy.baseline.board.item(key).id().0)
            .collect::<BTreeSet<_>>();
        let mut foreign_working = BTreeMap::<i32, String>::new();
        for key in request.board.board.get_items() {
            let item = request.board.board.item(key);
            if !(item.is_trace() || item.is_via()) || item.contains_net(net) {
                continue;
            }
            let id = item.id().0;
            if baseline_ids.contains(&id) {
                continue;
            }
            foreign_working.insert(id, working_item_signature(&request.board, key));
        }
        let mut trial_keys_by_id = BTreeMap::<i32, fr_engine::board::ItemKey>::new();
        for key in trial.board.get_items() {
            let id = trial.board.item(key).id().0;
            trial_keys_by_id.entry(id).or_insert(key);
        }
        for (id, signature) in &foreign_working {
            let Some(&key) = trial_keys_by_id.get(id) else {
                report["reason"] = "in-run route removed a foreign working item".into();
                return Ok((None, report, None));
            };
            if &working_item_signature(&trial, key) != signature {
                report["reason"] =
                    "in-run route changed a foreign working item's geometry, net, class or fixed state"
                        .into();
                return Ok((None, report, None));
            }
        }
        // The same-net island-tap split must not disconnect an earlier stable
        // Pin/Via group: each group snapshotted before the split must still sit
        // in one connected set.
        for group in [stable_from.as_ref(), stable_to.as_ref()]
            .into_iter()
            .flatten()
        {
            if group.is_empty() {
                continue;
            }
            if !trial
                .board
                .get_connected_sets(net)
                .iter()
                .any(|set| group.iter().all(|id| set.contains(*id)))
            {
                report["reason"] =
                    "in-run same-net split disconnected a stable pin/via group".into();
                return Ok((None, report, None));
            }
        }
    }
    if let Err(error) = trial.assert_invariants(policy.baseline, policy.protected, true) {
        let details: Vec<_>=trial.board.get_items().into_iter().flat_map(|key|fr_engine::drc::clearance_violation::clearance_violations(&trial.board,key)).map(|v| {
            let a=trial.board.item(v.first_item);let b=trial.board.item(v.second_item);
            serde_json::json!({"first_id":a.id().0,"first_type":format!("{:?}",a.board_item_type()),"second_id":b.id().0,"second_type":format!("{:?}",b.board_item_type()),"layer":v.layer,"expected_clearance_mm":v.expected_clearance/trial.units_per_mm,"actual_clearance_mm":v.actual_clearance/trial.units_per_mm,"overlap_bounds":format!("{:?}",v.shape.bounding_box())})
        }).collect();
        report["_endpoint_diagnostic"]["stage"] = "drc".into();
        report["reason"] = error.into();
        report["candidate_violations"] = serde_json::json!(details);
        report["candidate_vias"]=serde_json::json!(geometry.vias.iter().map(|v|serde_json::json!({"point_mm":v.point_mm,"diameter_mm":v.diameter_mm,"drill_mm":v.drill_mm,"from_layer":v.from_layer,"to_layer":v.to_layer})).collect::<Vec<_>>());
        report["candidate_segments"]=serde_json::json!(geometry.segments.iter().map(|s|serde_json::json!({"start_mm":s.start_mm,"end_mm":s.end_mm,"layer":s.layer,"width_mm":s.width_mm})).collect::<Vec<_>>());
        return Ok((None, report, None));
    }
    let before = fr_engine::drc::NetIncompletes::new(
        &request.board.board,
        net,
        &request.board.board.get_connectable_items(net),
    )
    .count();
    let after = fr_engine::drc::NetIncompletes::new(
        &trial.board,
        net,
        &trial.board.get_connectable_items(net),
    )
    .count();
    report["_endpoint_diagnostic"]["stage"] = "groups_checked".into();
    if let Some((source_key, target_key)) = selected_owner_keys {
        report["_endpoint_diagnostic"]["actual_source_group_after"] =
            serde_json::json!(stable_group_members(&trial, source_key, net)
                .iter()
                .map(|id| id.0)
                .collect::<Vec<_>>());
        report["_endpoint_diagnostic"]["actual_target_group_after"] =
            serde_json::json!(stable_group_members(&trial, target_key, net)
                .iter()
                .map(|id| id.0)
                .collect::<Vec<_>>());
    }
    // `assert_invariants` above compares the trial's full-board clearance
    // violations with the baseline and rejects any new violation. The trial
    // is unchanged since that proof, so the accepted route's new-violation
    // count is exactly zero without running the same full-board scan again.
    let new_violations = 0usize;
    report["incompletes_before"] = before.into();
    report["incompletes_after"] = after.into();
    report["new_violations"] = new_violations.into();
    let tap_stub_segments: usize = tap_results
        .iter()
        .map(|tap| tap.tap.stub_segments.len())
        .sum();
    report["segments"] = (geometry.segments.len() + tap_stub_segments + bridges).into();
    report["vias"] = (geometry.vias.len() + tap_results.len()).into();
    report["layer_transitions"] = path.windows(2).filter(|w| w[0].2 != w[1].2).count().into();
    let mut via_geometry = geometry.vias.iter().map(|v| {
        let info=&trial.board.rules.via_infos[v.via_info];
        let pad=trial.board.library.padstacks.get(info.get_padstack()).unwrap();
        serde_json::json!({"point_mm":v.point_mm,"diameter_mm":v.diameter_mm,"drill_mm":v.drill_mm,"from_layer":v.from_layer,"to_layer":v.to_layer,"physical_first_layer":pad.from_layer(),"physical_last_layer":pad.to_layer()})
    }).collect::<Vec<_>>();
    for tap in &tap_results {
        via_geometry.push(serde_json::json!({"point_mm":tap.tap.via.point,"diameter_mm":tap.tap.via.diameter,"drill_mm":tap.tap.via.drill,"from_layer":tap.tap.via.layers.first(),"to_layer":tap.tap.via.layers.last()}));
    }
    report["via_geometry"] = serde_json::json!(via_geometry);

    // Merge proof. The in-run entry can split a representative (a this-run
    // trace) out of the board, so it uses the stable Pin/Via group identities
    // snapshotted before the split instead of the possibly-removed key. The old
    // wrapper keeps its exact original key-based check.
    let merged = match (&stable_from, &stable_to) {
        (Some(from_ids), Some(to_ids)) => trial.board.get_connected_sets(net).iter().any(|set| {
            from_ids.iter().any(|id| set.contains(*id)) && to_ids.iter().any(|id| set.contains(*id))
        }),
        _ => trial
            .board
            .connected_set(from_item, net, false)
            .contains(trial.board.item(to_item).id()),
    };
    report["target_groups_merged"] = merged.into();
    report["_endpoint_diagnostic"]["target_groups_merged"] = merged.into();
    report["_endpoint_diagnostic"]["incompletes_before"] = before.into();
    report["_endpoint_diagnostic"]["incompletes_after"] = after.into();
    let groups_after = trial.board.get_connected_sets(net);
    let group_after = |before: &Option<BTreeSet<fr_engine::ids::ItemId>>| {
        before
            .as_ref()
            .and_then(|before| {
                groups_after
                    .iter()
                    .find(|group| before.iter().any(|id| group.contains(*id)))
            })
            .map(|group| {
                group
                    .iter()
                    .map(|key| trial.board.item(key).id().0)
                    .collect::<Vec<_>>()
            })
    };
    report["_endpoint_diagnostic"]["actual_source_group_after"] =
        serde_json::json!(group_after(&stable_from));
    report["_endpoint_diagnostic"]["actual_target_group_after"] =
        serde_json::json!(group_after(&stable_to));
    if !merged || after >= before || new_violations != 0 {
        report["_endpoint_diagnostic"]["stage"] =
            if !merged { "group_merge" } else { "net_count" }.into();
        report["reason"] =
            "native transaction did not reduce incompletes without new violations".into();
        return Ok((None, report, None));
    }
    report["accepted"] = true.into();
    Ok((Some(trial), report, None))
}

/// Old public entry: strict CURRENT-BOARD immutability with an empty protected
/// set and no this-run trace splitting. Behaviour, report keys and control
/// classes are unchanged; only the third tuple element added for the in-run
/// entry is dropped here.
pub fn route_multilayer_endpoint_sets(
    request: RungRequest,
    from_item: fr_engine::board::ItemKey,
    to_item: fr_engine::board::ItemKey,
    start_candidates: &[RungEndpoint],
    target_candidates: &[RungEndpoint],
) -> Result<(Option<BoardAdapter>, serde_json::Value), String> {
    let baseline = request.board.clone();
    let (board, report, _frontier) = route_multilayer_endpoint_sets_core(
        request,
        from_item,
        to_item,
        start_candidates,
        target_candidates,
        EndpointSetPolicy {
            baseline: &baseline,
            protected: &BTreeSet::new(),
            split_this_run_trace: false,
        },
    )?;
    Ok((board, report))
}

/// In-run entry. The frozen baseline is the caller's ACTUAL captured input
/// board (never an empty fabricated board) and `protected` is the authoritative
/// protected-net set. The shared core proves preservation with
/// `trial.assert_invariants(baseline, protected, true)`, forbids copper on any
/// other working net, and -- only for a same-net THIS-RUN trace that is neither
/// in the baseline nor protected -- performs the SOURCE-required split at the
/// selected island-tap endpoint, welding the route geometry endpoint to the
/// real native split/contact point. On a genuine SourceGridSearch no-path
/// failure the third tuple carries the existing frontier report; non-search
/// errors return `None` there.
pub(crate) fn route_multilayer_endpoint_sets_in_run(
    request: RungRequest,
    from_item: fr_engine::board::ItemKey,
    to_item: fr_engine::board::ItemKey,
    start_candidates: &[RungEndpoint],
    target_candidates: &[RungEndpoint],
    baseline: &BoardAdapter,
    protected: &BTreeSet<String>,
) -> Result<
    (
        Option<BoardAdapter>,
        serde_json::Value,
        Option<super::braid_connect::RungReport>,
    ),
    String,
> {
    route_multilayer_endpoint_sets_core(
        request,
        from_item,
        to_item,
        start_candidates,
        target_candidates,
        EndpointSetPolicy {
            baseline,
            protected,
            split_this_run_trace: true,
        },
    )
}

/// Existing frontier report shape surfaced on a SourceGridSearch failure, built
/// only from the search outcome's own window/step/layers/iterations/frontier.
/// No new public telemetry keys are introduced.
fn frontier_report(
    request: &RungRequest,
    outcome: &GridSearchResult,
    units_per_mm: f64,
) -> super::braid_connect::RungReport {
    let (frontier, expanded) = if request.stage == "source_rip_retry" {
        (&outcome.frontier, outcome.iterations)
    } else if outcome.forward_iterations > 0
        && (outcome.backward_iterations == 0
            || outcome.forward_iterations <= outcome.backward_iterations)
    {
        (&outcome.forward_frontier, outcome.forward_iterations)
    } else if outcome.backward_iterations > 0 {
        (&outcome.backward_frontier, outcome.backward_iterations)
    } else {
        (&outcome.frontier, outcome.iterations)
    };
    super::braid_connect::RungReport {
        blocked_cells: frontier
            .iter()
            .map(|&(x, y, layer)| (x, y, layer as usize))
            .collect(),
        route_layers: request.route_layers.clone(),
        window: [
            outcome.window.ll.x as f64 / units_per_mm,
            outcome.window.ll.y as f64 / units_per_mm,
            outcome.window.ur.x as f64 / units_per_mm,
            outcome.window.ur.y as f64 / units_per_mm,
        ],
        grid_step_mm: outcome.grid_step_board as f64 / units_per_mm,
        expanded,
        truncated: false,
    }
}

#[derive(Clone)]
struct SelectedEndpointOwner {
    item: fr_engine::board::ItemKey,
    exact_point: Option<EnginePoint>,
}

/// Exact source tap samples used by `this_run_trace_tap_points`: sample each
/// approximated polyline edge at `max(1, int(length / grid_step))` intervals,
/// including both ends. This proves that a selected interior coordinate is a
/// real candidate sample; it does not assign ownership by nearest geometry.
fn trace_has_source_tap_sample(
    polyline: &Polyline,
    selected_mm: [f64; 2],
    grid_step_mm: f64,
    units_per_mm: f64,
) -> bool {
    if !grid_step_mm.is_finite() || grid_step_mm <= 0.0 || !units_per_mm.is_finite() {
        return false;
    }
    for window in polyline.corner_approx_arr().windows(2) {
        let x0 = window[0].x as f64 / units_per_mm;
        let y0 = window[0].y as f64 / units_per_mm;
        let x1 = window[1].x as f64 / units_per_mm;
        let y1 = window[1].y as f64 / units_per_mm;
        let dx = x1 - x0;
        let dy = y1 - y0;
        let length = (dx * dx + dy * dy).sqrt();
        if length < 0.001 {
            if [x0, y0] == selected_mm {
                return true;
            }
            continue;
        }
        let steps = 1.max((length / grid_step_mm) as i64) as usize;
        for index in 0..=steps {
            let t = index as f64 / steps as f64;
            if [x0 + t * dx, y0 + t * dy] == selected_mm {
                return true;
            }
        }
    }
    false
}

/// Resolve a selected candidate back to the native item that actually owns its
/// terminal coordinate. Candidate lists in the in-run tap path may contain
/// copper from more than the planner's nominal `from_item` component, so the
/// selected coordinate, rather than that nominal representative, owns the
/// source-side split and merge proof.
///
/// Pin/Via centres and Trace endpoints retain their original native Point,
/// including Rational points. Interior Trace owners are accepted only when the
/// selected coordinate lies on that real same-net centreline; the caller then
/// uses the existing exact split/weld path. Coincident items are usable only if
/// they already belong to one native connected set.
fn selected_endpoint_owner(
    board: &BoardAdapter,
    baseline: &BoardAdapter,
    net: NetNo,
    endpoint: &RungEndpoint,
    grid_step_mm: f64,
) -> Result<Option<SelectedEndpointOwner>, String> {
    let layer_no = board.board.layer_structure.get_no(&endpoint.layer);
    if layer_no < 0 {
        return Ok(None);
    }
    if endpoint.point.iter().any(|value| !value.is_finite()) {
        return Err("selected endpoint owner has a nonfinite coordinate".into());
    }

    // Candidate-only spatial query: an endpoint may be rounded onto the
    // neighboring native unit, so cover one unit on each side of its rounded
    // native center. Keep the original board-order traversal below because
    // coincident-owner tie behavior depends on it; the spatial result only
    // skips items that cannot own this endpoint. Fall back to the full scan if
    // the native box cannot be represented safely.
    let sample_native = [
        endpoint.point[0] * board.units_per_mm,
        endpoint.point[1] * board.units_per_mm,
    ];
    let candidate_keys = if board.units_per_mm.is_finite()
        && board.units_per_mm > 0.0
        && sample_native.iter().all(|coordinate| {
            coordinate.is_finite()
                && *coordinate >= i32::MIN as f64 + 1.0
                && *coordinate <= i32::MAX as f64 - 1.0
        }) {
        let center = FloatPoint::new(sample_native[0], sample_native[1]).round();
        center
            .x
            .checked_sub(1)
            .zip(center.y.checked_sub(1))
            .zip(center.x.checked_add(1))
            .zip(center.y.checked_add(1))
            .map(|(((min_x, min_y), max_x), max_y)| {
                let area = fr_geom::Area::Shape(fr_geom::Shape::from(fr_geom::IntBox::new(
                    min_x, min_y, max_x, max_y,
                )));
                board
                    .board
                    .overlapping_items(&area, layer_no)
                    .iter()
                    .collect::<BTreeSet<_>>()
            })
    } else {
        None
    };

    let mut owners = Vec::<(fr_engine::board::ItemKey, Option<EnginePoint>)>::new();
    for key in board.board.get_items() {
        if candidate_keys
            .as_ref()
            .is_some_and(|candidates| !candidates.contains(&key))
        {
            continue;
        }
        let item = board.board.item(key);
        if !item.is_on_board()
            || !item.contains_net(net)
            || !item.is_on_layer(&board.board, layer_no)
        {
            continue;
        }
        let mut exact = None;
        let matches = match item.board_item_type() {
            fr_engine::board::BoardItemType::Pin | fr_engine::board::BoardItemType::Via => {
                let center = item.center(&board.board);
                let p = center.to_float();
                if [p.x / board.units_per_mm, p.y / board.units_per_mm] == endpoint.point {
                    exact = Some(center);
                    true
                } else {
                    false
                }
            }
            fr_engine::board::BoardItemType::Trace => {
                let trace = item.trace();
                let polyline = trace.polyline();
                let first = polyline.corner(0);
                let last = polyline.corner(polyline.lines.len() as i32 - 2);
                let endpoint_point = [first, last].into_iter().find(|point| {
                    let p = point.to_float();
                    [p.x / board.units_per_mm, p.y / board.units_per_mm] == endpoint.point
                });
                if let Some(point) = endpoint_point {
                    exact = Some(point.clone());
                    true
                } else {
                    let baseline_item =
                        baseline.board.item_repository().iter().any(|baseline_key| {
                            baseline.board.item(baseline_key).id() == item.id()
                        });
                    !baseline_item
                        && trace_has_source_tap_sample(
                            polyline,
                            endpoint.point,
                            grid_step_mm,
                            board.units_per_mm,
                        )
                }
            }
            fr_engine::board::BoardItemType::ConductionArea => board
                .board
                .ratsnest_corners(key)
                .into_iter()
                .find(|point| {
                    let p = point.to_float();
                    [p.x / board.units_per_mm, p.y / board.units_per_mm] == endpoint.point
                })
                .filter(|point| {
                    let first = item.first_layer(&board.board);
                    let last = item.last_layer(&board.board);
                    (0..board.board.tile_shape_count(key)).any(|index| {
                        let layer = board.board.shape_layer(key, index);
                        layer >= first
                            && layer <= last
                            && board
                                .board
                                .tile_shape(key, index)
                                .is_some_and(|shape| shape.contains(point))
                    })
                })
                .map(|point| {
                    exact = Some(point.clone());
                })
                .is_some(),
            _ => false,
        };
        if matches {
            owners.push((key, exact));
        }
    }

    let Some((first_key, _)) = owners.first() else {
        return Ok(None);
    };
    let group = board.board.connected_set(*first_key, net, false);
    if owners
        .iter()
        .skip(1)
        .any(|(key, _)| !group.contains(board.board.item(*key).id()))
    {
        return Err("selected endpoint has multiple disconnected native owners".into());
    }
    let exact_points = owners
        .iter()
        .filter_map(|(_, point)| point.as_ref())
        .collect::<Vec<_>>();
    if exact_points
        .iter()
        .skip(1)
        .any(|point| *point != exact_points[0])
    {
        return Err("selected endpoint aliases distinct native Points".into());
    }
    Ok(Some(SelectedEndpointOwner {
        item: *first_key,
        exact_point: exact_points.first().map(|point| (*point).clone()),
    }))
}

/// Stable Pin/Via identities of the connected group at `key`, captured BEFORE
/// any in-run split removes the representative. Only real board items are used;
/// no component info is fabricated. Falls back to the representative's own id
/// when the group carries no pin/via copper.
fn stable_group_members(
    board: &BoardAdapter,
    key: fr_engine::board::ItemKey,
    net: NetNo,
) -> BTreeSet<fr_engine::ids::ItemId> {
    let mut ids = BTreeSet::new();
    if board.board.try_item(key).is_none() {
        return ids;
    }
    let group = board.board.connected_set(key, net, false);
    for member in group.iter() {
        let item = board.board.item(member);
        if item.is_pin() || item.is_via() {
            ids.insert(item.id());
        }
    }
    if ids.is_empty() {
        if let Some(item) = board.board.try_item(key) {
            ids.insert(item.id());
        }
    }
    ids
}

/// Deterministic identity of a working item's type, nets, clearance class,
/// fixed state and geometry, for the in-run foreign-working-item guard. A
/// deletion or any change to these fields surfaces as a different signature.
fn working_item_signature(board: &BoardAdapter, key: fr_engine::board::ItemKey) -> String {
    let item = board.board.item(key);
    let mut signature = format!(
        "{:?}|{:?}|{}|{:?}",
        item.board_item_type(),
        item.net_numbers(),
        item.clearance_class(),
        item.fixed_state()
    );
    if item.is_trace() {
        let trace = item.trace();
        signature.push_str(&format!("|L{}W{}", trace.layer(), trace.half_width()));
        let polyline = trace.polyline();
        for index in 0..polyline.lines.len() as i32 - 1 {
            signature.push_str(&format!("|{:?}", polyline.corner(index)));
        }
    } else if item.is_via() {
        signature.push_str(&format!(
            "|{:?}|{:?}",
            item.center(&board.board),
            item.as_via().map(|via| via.padstack_no())
        ));
    }
    signature
}

/// The chosen route point lies inside this polyline's centreline, away from
/// both end corners. `Int` corners compare exactly; a `Rational` perpendicular
/// foot from `projection_line` is interior by construction.
fn point_is_trace_interior(polyline: &Polyline, point: &EnginePoint) -> bool {
    let Some(last_index) = polyline.lines.len().checked_sub(2) else {
        return false;
    };
    let first = polyline.corner(0);
    let last = polyline.corner(last_index as i32);
    if points_coincide(point, &first) || points_coincide(point, &last) {
        return false;
    }
    if point.is_int_point() {
        polyline.contains(point)
    } else {
        true
    }
}

fn points_coincide(a: &EnginePoint, b: &EnginePoint) -> bool {
    let (fa, fb) = (a.to_float(), b.to_float());
    (fa.x - fb.x).abs() <= 1e-6 && (fa.y - fb.y).abs() <= 1e-6
}

/// Resolve the ACTUAL same-net THIS-RUN trace that owns the source-chosen tap
/// point. `representative` (the caller's from_item/to_item) is usually a Pin or
/// Via, so the owner is searched inside that representative's OWN connected
/// group -- never an arbitrary nearest unrelated trace or a virtual pin.
fn island_tap_owner(
    board: &BoardAdapter,
    baseline: &BoardAdapter,
    protected: &BTreeSet<String>,
    net: NetNo,
    representative: fr_engine::board::ItemKey,
    layer_no: i32,
    point: &EnginePoint,
    sample_mm: [f64; 2],
    units_per_mm: f64,
) -> Option<fr_engine::board::ItemKey> {
    let selected = board.board.try_item(representative)?;
    if board
        .board
        .rules
        .nets
        .get(net)
        .is_some_and(|name| protected.contains(&name.name))
    {
        return None;
    }
    let sample_native = [sample_mm[0] * units_per_mm, sample_mm[1] * units_per_mm];
    if sample_native
        .iter()
        .any(|coordinate| !coordinate.is_finite())
    {
        return None;
    }
    // `selected_endpoint_owner` has already resolved this exact Trace and
    // rejected aliases from another connected component. Revalidate that
    // owner directly; scanning its whole connected group again can encounter
    // a second same-group trace on coincident copper and incorrectly decline
    // an otherwise unambiguous selected owner.
    if selected.is_trace() {
        return island_trace_matches_tap(
            board,
            baseline,
            net,
            representative,
            layer_no,
            point,
            sample_native,
        )
        .then_some(representative);
    }
    let group = board.board.connected_set(representative, net, false);
    let mut owner = None;
    for key in group.iter() {
        if !island_trace_matches_tap(board, baseline, net, key, layer_no, point, sample_native) {
            continue;
        }
        if owner.is_some() {
            // Legacy Pin/Via representatives do not have a selected Trace
            // owner. Preserve the conservative ambiguity rule for that path.
            return None;
        }
        owner = Some(key);
    }
    owner
}

fn island_trace_matches_tap(
    board: &BoardAdapter,
    baseline: &BoardAdapter,
    net: NetNo,
    key: fr_engine::board::ItemKey,
    layer_no: i32,
    point: &EnginePoint,
    sample_native: [f64; 2],
) -> bool {
    let Some(item) = board.board.try_item(key) else {
        return false;
    };
    if !item.is_trace()
        || !item.is_on_board()
        || !item.contains_net(net)
        || item.trace().layer() != layer_no
        || baseline
            .board
            .item_repository()
            .iter()
            .any(|baseline_key| baseline.board.item(baseline_key).id() == item.id())
    {
        return false;
    }
    let polyline = item.trace().polyline();
    // Select only a trace owning the original sampled coordinate, before Int
    // quantization can make a nearby parallel trace appear equally plausible.
    if polyline_sample_distance_squared(polyline, sample_native) > 1e-8 {
        return false;
    }
    if polyline.contains(point) {
        return point_is_trace_interior(polyline, point);
    }
    let Some(projection) = polyline.projection_line(point) else {
        return false;
    };
    let foot = projection.end_point();
    let delta = foot.difference_by(point).to_float();
    let distance_squared = delta.x * delta.x + delta.y * delta.y;
    // `BoardAdapter::point` rounds each native coordinate to the nearest Int,
    // so the proven maximum squared displacement is 0.5 native-unit squared.
    distance_squared <= 0.5000001
        && projection_owner_segment_index(polyline, &projection).is_some()
        && point_is_trace_interior(polyline, &foot)
}

/// Squared distance from a retained source sample to the actual centreline.
/// This is ownership validation only; routing and endpoint selection remain in
/// the source GridRouter path.
fn polyline_sample_distance_squared(polyline: &Polyline, sample: [f64; 2]) -> f64 {
    let mut nearest = f64::INFINITY;
    for index in 0..polyline.lines.len().saturating_sub(2) {
        let a = polyline.corner(index as i32).to_float();
        let b = polyline.corner(index as i32 + 1).to_float();
        let dx = b.x - a.x;
        let dy = b.y - a.y;
        let length_squared = dx * dx + dy * dy;
        if length_squared == 0.0 {
            continue;
        }
        let t =
            (((sample[0] - a.x) * dx + (sample[1] - a.y) * dy) / length_squared).clamp(0.0, 1.0);
        let ex = sample[0] - (a.x + t * dx);
        let ey = sample[1] - (a.y + t * dy);
        nearest = nearest.min(ex * ex + ey * ey);
    }
    nearest
}

/// The ACTUAL centreline segment index (`1..polyline.lines.len()-1`) that the
/// engine's `Polyline::projection_line` selected. The returned `projection`
/// keeps the selected owner line as its END closing line and the
/// integer-anchored PERPENDICULAR through the query Int point as its MIDDLE
/// (`get_line()`) line, so the owner index is recovered by EXACT `Line`
/// identity -- never by an unrelated nearest line or a floating distance
/// tolerance. `Polyline::projection_line`'s own exact corner-side guard is
/// re-applied, so a non-adjacent but geometrically equal line cannot be
/// mistaken for the owner segment.
fn projection_owner_segment_index(
    polyline: &Polyline,
    projection: &fr_geom::LineSegment,
) -> Option<i32> {
    let nearest = projection.get_end_closing_line();
    let last = polyline.lines.len() as i32 - 1;
    let perpendicular = projection.get_line();
    for index in 1..last {
        if &polyline.lines[index as usize] != nearest {
            continue;
        }
        let prev_side = perpendicular.side_of(&polyline.corner(index - 1));
        let next_side = perpendicular.side_of(&polyline.corner(index));
        if prev_side == next_side && prev_side != fr_geom::Side::Collinear {
            continue;
        }
        return Some(index);
    }
    None
}

/// SOURCE-required island-tap weld for the in-run entry. The owner trace is the
/// same-net THIS-RUN member of the representative's connected group that owns
/// the source-chosen point. When the point lies on the trace interior the split
/// is DIRECT (`projection_line` returns `None` for a collinear point); otherwise
/// the engine's exact perpendicular foot is used. The returned endpoint is
/// relocated onto the real split contact and the engine's EXACT native `Point`
/// (if not an `Int`) is returned for the materializer so no millimetre rounding
/// can fabricate a fake contact. Baseline/protected copper is never touched;
/// when no authorised this-run trace interior is found, `Ok(None)` leaves the
/// endpoint unchanged. A UserFixed this-run trace is temporarily unfixed so the
/// mature split may replace it, then both pieces are restored -- no baseline
/// fixed flag is ever modified.
fn weld_island_tap_endpoint(
    board: &mut BoardAdapter,
    baseline: &BoardAdapter,
    protected: &BTreeSet<String>,
    net: NetNo,
    representative: fr_engine::board::ItemKey,
    endpoint: &RungEndpoint,
    pieces: &mut Vec<fr_engine::board::ItemKey>,
) -> Result<Option<(RungEndpoint, Option<EnginePoint>, usize)>, String> {
    let layer_no = board.board.layer_structure.get_no(&endpoint.layer);
    if layer_no < 0 {
        return Ok(None);
    }
    let point = match board.point(endpoint.point)? {
        EnginePoint::Int(point) => point,
        _ => return Ok(None),
    };
    let Some(owner) = island_tap_owner(
        board,
        baseline,
        protected,
        net,
        representative,
        layer_no,
        &EnginePoint::Int(point),
        endpoint.point,
        board.units_per_mm,
    ) else {
        return Ok(None);
    };
    let fixed = board.board.item(owner).fixed_state();
    let polyline = board.board.item(owner).trace().polyline().clone();
    let projection = polyline.projection_line(&EnginePoint::Int(point));
    let exact = match &projection {
        Some(projection) => projection.end_point(),
        // Collinear interior point (a single-segment trace has no perpendicular
        // foot): split DIRECT at the source-chosen point.
        None => EnginePoint::Int(point),
    };
    if !point_is_trace_interior(&polyline, &exact) {
        return Ok(None);
    }
    let restore = fixed != FixedState::Unfixed;
    if restore {
        board
            .board
            .item_mut(owner)
            .set_fixed_state(FixedState::Unfixed);
    }
    // Native copper connectivity requires an explicit shared endpoint. A
    // source-chosen point can lie within a pin's copper while remaining away
    // from its centre; the engine's split guard rejects that case unless an
    // existing trace endpoint already touches the pin. Seed that legal
    // same-net connection before splitting, using only the pad's real copper
    // shape and the trace's available width.
    let seed = match bridge_welded_point_to_drill_center(board, owner, &exact, net, fixed) {
        Some(seed) => Some(seed),
        None => None,
    };
    // Non-collinear tap: reuse the engine's projection line through the query
    // Int and exact foot, and split on the selected owner's actual centreline
    // segment with the mature public `split_trace_at_line` API.
    let mut exact_endpoint = None;
    let split = match &projection {
        Some(projection) => match projection_owner_segment_index(&polyline, projection) {
            Some(line_index) => {
                let split_line = fr_engine::board::optimize::tracked::TLine::fresh(
                    projection.get_line().clone(),
                );
                let split = board
                    .board
                    .split_trace_at_line(owner, line_index, &split_line);
                exact_endpoint = Some(exact.clone());
                split
            }
            None => None,
        },
        // Already-collinear Int interior source point: the source's own
        // `split_trace_at_point` path is correct and unchanged.
        None => board.board.split_trace_at_point(owner, &exact),
    };
    let Some([a, b]) = split else {
        if let Some(seed) = seed {
            board.board.remove_item(seed);
        }
        if restore {
            board.board.item_mut(owner).set_fixed_state(fixed);
        }
        return Ok(None);
    };
    if restore {
        for key in [a, b].into_iter().flatten() {
            board.board.item_mut(key).set_fixed_state(fixed);
        }
    }
    pieces.extend(a);
    pieces.extend(b);
    let point_mm = exact.to_float();
    let upm = board.units_per_mm;
    Ok(Some((
        RungEndpoint {
            point: [point_mm.x / upm, point_mm.y / upm],
            layer: endpoint.layer.clone(),
        },
        exact_endpoint,
        usize::from(seed.is_some()),
    )))
}

/// Insert a same-net copper bridge from a newly selected split point to a
/// real pin/via centre when the point lies inside that item's true drill-layer
/// copper shape. The bridge stays wholly inside the pad and is no wider than
/// either the owner trace or the room available at both ends.
fn bridge_welded_point_to_drill_center(
    board: &mut BoardAdapter,
    owner: fr_engine::board::ItemKey,
    point: &EnginePoint,
    net: NetNo,
    fixed_state: FixedState,
) -> Option<fr_engine::board::ItemKey> {
    let trace = board.board.item(owner);
    let layer = trace.trace().layer();
    let half_width = trace.trace().half_width();
    let nets = trace.net_numbers().to_vec();
    let clearance_class = trace.clearance_class();
    let contacts = board.board.trace_normal_contacts_at(owner, point, false);
    let pads = board
        .board
        .get_pins()
        .into_iter()
        .chain(board.board.get_vias());
    for pad_key in pads {
        let pad = board.board.item(pad_key);
        if !pad.contains_net(net) || !pad.is_on_layer(&board.board, layer) {
            continue;
        }
        let center = pad.center(&board.board);
        if center == *point {
            continue;
        }
        let Some(shape) = pad.drill_shape_on_layer(&board.board, layer) else {
            continue;
        };
        if shape
            .as_polygon_shape()
            .is_some_and(|polygon| !polygon.is_convex())
            || !shape.contains(&center)
            || !shape.contains(point)
        {
            continue;
        }
        let center_float = center.to_float();
        let point_float = point.to_float();
        let room = shape
            .border_distance(&center_float)
            .min(shape.border_distance(&point_float));
        let bridge_half_width = half_width.min(room.floor() as i32);
        if bridge_half_width <= 0 {
            continue;
        }
        if contacts.iter().any(|contact| {
            let item = board.board.item(contact);
            if !item.is_trace()
                || !item.is_on_board()
                || !item.contains_net(net)
                || item.trace().layer() != layer
            {
                return false;
            }
            let (first, last) = (item.trace().first_corner(), item.trace().last_corner());
            (first == *point && last == center) || (first == center && last == *point)
        }) {
            return None;
        }
        return board.board.insert_trace_without_cleaning(
            Polyline::from_two_points(point, &center),
            layer,
            bridge_half_width,
            &nets,
            clearance_class,
            fixed_state,
        );
    }
    None
}

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

#[cfg(test)]
mod weld_drill_center_seed_fixture {
    use super::{
        bridge_welded_point_to_drill_center, bridge_welded_trace_ends_to_drill_centers,
        weld_island_tap_endpoint, BoardAdapter, FixedState, Polyline,
    };
    use crate::awx::braid_connect::RungEndpoint;
    use std::collections::BTreeSet;

    const THREE_ITEM_DSN: &str = r#"(pcb (resolution mil 1000) (unit mil)
      (structure
        (layer Top (type signal)) (layer Inner1 (type signal))
        (layer Inner2 (type signal)) (layer Bottom (type signal))
        (via VIA) (boundary (rect pcb 2000 0 3500 1500))
        (rule (width 10) (clearance 6.03)))
      (library
        (image CHIP (pin PAD 498 0 0))
        (padstack PAD (shape (circle Top 24.02))
          (shape (circle Inner1 24.02)) (shape (circle Inner2 24.02))
          (shape (circle Bottom 24.02)))
        (padstack VIA (shape (circle Top 0.5)) (shape (circle Bottom 0.5)))
      )
      (placement (component CHIP
        (place U1 2789.58 483.74 front 0)
        (place U2 3000 700 front 0)))
      (network (net BUS (pins U1-498 U2-498)))
    )"#;

    #[test]
    fn seeds_pad_interior_before_native_trace_split() {
        let baseline = BoardAdapter::load(THREE_ITEM_DSN.as_bytes()).unwrap();
        let mut board = baseline.clone();
        let net = board.net_number("BUS").unwrap();
        let source = board
            .board
            .get_pins()
            .into_iter()
            .find(|&key| {
                let point = board.board.item(key).center(&board.board).to_float();
                (point.x / board.units_per_mm - 70.855332).abs() < 1e-8
                    && (point.y / board.units_per_mm - 12.286996).abs() < 1e-8
            })
            .unwrap_or_else(|| {
                panic!(
                    "source Pin 498; centers: {:?}",
                    board
                        .board
                        .get_pins()
                        .iter()
                        .map(|&key| board.board.item(key).center(&board.board).to_float())
                        .collect::<Vec<_>>()
                )
            });
        assert_eq!(board.board.get_pins().len(), 2);
        let start = board.point([71.0034648, 12.0005856]).unwrap();
        let end = board.point([70.5034404, 12.50061]).unwrap();
        let query = board.point([70.7534526, 12.2505978]).unwrap();
        let layer = board.board.layer_structure.get_no("Top");
        let width = (0.254 * board.units_per_mm / 2.0).round() as i32;
        let class = board
            .board
            .rules
            .net_class_of(net)
            .get_trace_clearance_class();
        let trace = board
            .board
            .insert_trace_without_cleaning(
                Polyline::from_two_points(&start, &end),
                layer,
                width,
                &[net],
                class,
                FixedState::UserFixed,
            )
            .expect("this-run source trace");
        assert_eq!(
            board.board.get_pins().len() + board.board.get_traces().len(),
            3
        );

        board
            .board
            .item_mut(trace)
            .set_fixed_state(FixedState::Unfixed);
        assert!(board.board.split_trace_at_point(trace, &query).is_none());
        board
            .board
            .item_mut(trace)
            .set_fixed_state(FixedState::UserFixed);
        let mut seeded_board = board.clone();
        let seeded_trace = seeded_board
            .board
            .get_traces()
            .into_iter()
            .find(|&key| seeded_board.board.item(key).first_corner() == start)
            .unwrap();
        let seed = bridge_welded_point_to_drill_center(
            &mut seeded_board,
            seeded_trace,
            &query,
            net,
            FixedState::UserFixed,
        )
        .expect("real pad copper accepts seed bridge");
        seeded_board
            .board
            .item_mut(seeded_trace)
            .set_fixed_state(FixedState::Unfixed);
        assert!(seeded_board
            .board
            .split_trace_at_point(seeded_trace, &query)
            .is_some());
        assert!(seeded_board.board.item(seed).is_on_board());

        let query_float = query.to_float();
        let query_mm = [
            query_float.x / board.units_per_mm,
            query_float.y / board.units_per_mm,
        ];
        assert!((query_mm[0] - 70.7534526).abs() < 1e-9);
        assert!((query_mm[1] - 12.2505978).abs() < 1e-9);
        let endpoint = RungEndpoint {
            point: query_mm,
            layer: "Top".into(),
        };
        let selected = super::selected_endpoint_owner(&board, &baseline, net, &endpoint, 0.25)
            .unwrap()
            .expect("spatial owner lookup resolves the source trace");
        assert_eq!(selected.item, trace);
        assert!(selected.exact_point.is_none());
        let mut split_pieces = Vec::new();
        let welded = weld_island_tap_endpoint(
            &mut board,
            &baseline,
            &BTreeSet::new(),
            net,
            trace,
            &endpoint,
            &mut split_pieces,
        )
        .unwrap()
        .expect("interior pad contact is made splittable by seed bridge");
        assert_eq!(welded.2, 1);
        assert_eq!(split_pieces.len(), 2);
        let connected = board.board.connected_set(source, net, false);
        assert!(split_pieces
            .iter()
            .all(|piece| connected.iter().any(|key| key == *piece)));
        let target = board
            .board
            .get_pins()
            .into_iter()
            .find(|&key| key != source)
            .expect("separate target Pin");
        assert!(!connected.iter().any(|key| key == target));
        assert!(connected.iter().any(|key| {
            board.board.item(key).is_trace()
                && board.board.item(key).trace().first_corner() == query
        }));
        assert!(board.board.get_traces().iter().any(|&key| {
            board.board.item(key).is_on_board()
                && board.board.item(key).first_corner() == query
                && board.board.item(key).last_corner()
                    == board.board.item(source).center(&board.board)
        }));
        let all_added_traces = board.board.get_traces();
        assert_eq!(
            bridge_welded_trace_ends_to_drill_centers(&mut board, &all_added_traces, net),
            0
        );
        assert_eq!(
            board
                .board
                .get_traces()
                .into_iter()
                .filter(|&key| {
                    let trace = board.board.item(key).trace();
                    let first = trace.first_corner();
                    let last = trace.last_corner();
                    (first == query && last == board.board.item(source).center(&board.board))
                        || (last == query && first == board.board.item(source).center(&board.board))
                })
                .count(),
            1
        );
    }
}
