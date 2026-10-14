//! Native bridge for the source coupled-pair connector and PoseRouter.
use super::{
    braid_connect::RungRequest,
    corridor::Point,
    native_obstacles::{self, NativeGridConfig},
    pair_connection::{
        BandRestriction, EnvelopeRequest, EnvelopeRoute, EnvelopeRun, PairBandEnd,
        PairConnectionBackend, PairPiece, PairPoseOutcome, PairPoseRequest, PairVia,
    },
    pair_path::{self, PairPathConfig},
};
use crate::board_adapter::BoardAdapter;
use awx_grid::{obstacle_map::GridObstacleMap, PoseRouter, TrackMarginArg};
use fr_engine::ids::{ClearanceClassNo, NetNo};
use fr_geom::{IntBox, Point as EnginePoint};
use std::time::Instant;

pub struct NativePairRouter<'a> {
    request: &'a RungRequest,
    board: BoardAdapter,
    p: NetNo,
    n: NetNo,
    p_trace_clearance: ClearanceClassNo,
    p_via_clearance: ClearanceClassNo,
}

impl<'a> NativePairRouter<'a> {
    pub fn new(request: &'a RungRequest) -> Result<Self, String> {
        let pair = request
            .pair
            .as_ref()
            .ok_or("native pair router needs a pair request")?;
        if pair.positive_net_id == pair.negative_net_id {
            return Err("pair members resolve to the same native net".into());
        }
        let mut board = request.board.clone();
        let (p_control, _) = super::native_rung::prepare_control_for_net_with_vias(
            &mut board,
            pair.positive_net_id,
            &request.rules,
            &request.route_layers,
            &request.fab_via_rungs,
        )?;
        let (n_control, _) = super::native_rung::prepare_control_for_net_with_vias(
            &mut board,
            pair.negative_net_id,
            &request.rules,
            &request.route_layers,
            &request.fab_via_rungs,
        )?;
        let class_count = board.board.rules.clearance_matrix.get_class_count();
        for class in [
            p_control.trace_clearance_class,
            p_control.via_clearance_class,
            n_control.trace_clearance_class,
            n_control.via_clearance_class,
        ] {
            if class < 0 || class >= class_count {
                return Err("native pair clearance class was not registered on route board".into());
            }
        }
        Ok(Self {
            request,
            board,
            p: pair.positive_net_id,
            n: pair.negative_net_id,
            p_trace_clearance: p_control.trace_clearance_class,
            p_via_clearance: p_control.via_clearance_class,
        })
    }

    fn make_map(
        &self,
        points: &[Point],
        margin: f64,
        own: &[NetNo],
        extra: f64,
        restrictions: &[BandRestriction],
        obstacle_track_width: f64,
        obstacle_via_size: f64,
        obstacle_via_drill: f64,
        center_extra: f64,
        virtual_track_width: f64,
        virtual_via_size: f64,
    ) -> Result<(GridObstacleMap, IntBox, i32), String> {
        let step = (self.request.rules.grid_step * self.board.units_per_mm)
            .round()
            .max(1.0) as i32;
        let mut pts = points.to_vec();
        if let Some(w) = &self.request.window_points {
            pts.extend(w.iter().copied());
        }
        if pts.is_empty() {
            return Err("pair search has no window endpoints".into());
        }
        let (lo_x, hi_x) = pts
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                (lo.min(p[0]), hi.max(p[0]))
            });
        let (lo_y, hi_y) = pts
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                (lo.min(p[1]), hi.max(p[1]))
            });
        let half = ((hi_x - lo_x).max(hi_y - lo_y)) / 2.0 + margin;
        let c = [(lo_x + hi_x) / 2.0, (lo_y + hi_y) / 2.0];
        let a = match self.board.point([c[0] - half, c[1] - half])? {
            EnginePoint::Int(p) => p,
            _ => return Err("pair window outside board coordinate range".into()),
        };
        let b = match self.board.point([c[0] + half, c[1] + half])? {
            EnginePoint::Int(p) => p,
            _ => return Err("pair window outside board coordinate range".into()),
        };
        let bb = self.board.board.bounding_box;
        let window = IntBox::new(
            a.x.max(bb.ll.x),
            a.y.max(bb.ll.y),
            b.x.min(bb.ur.x),
            b.y.min(bb.ur.y),
        );
        if window.is_empty() {
            return Err("empty pair search window".into());
        }
        let mut config = NativeGridConfig::from_request(
            &self.board,
            self.request,
            self.p_trace_clearance,
            self.p_via_clearance,
        )?;
        let native_size = |mm: f64| -> Result<i32, String> {
            let v = (mm * self.board.units_per_mm).round();
            if !v.is_finite() || v < 0.0 || v > i32::MAX as f64 {
                return Err("pair obstacle dimension outside native board range".into());
            }
            Ok(v as i32)
        };
        config.track_width = native_size(obstacle_track_width)?;
        config.via_size = native_size(obstacle_via_size)?;
        config.via_drill = native_size(obstacle_via_drill)?;
        config.extra_clearance = extra * self.board.units_per_mm;
        if center_extra == 0.0 {
            config.net_via_sizes.clear();
            config.fab_via_rungs.clear();
            config.ordered_via_rungs.clear();
        }
        let mut result = native_obstacles::build_obstacle_map_with_config(
            &self.board,
            self.request,
            window,
            own,
            config,
        )?;
        let gx0 = (window.ll.x as f64 / step as f64).ceil() as i32;
        let gy0 = (window.ll.y as f64 / step as f64).ceil() as i32;
        let gx1 = (window.ur.x as f64 / step as f64).floor() as i32;
        let gy1 = (window.ur.y as f64 / step as f64).floor() as i32;
        super::native_rung::fence_grid_window(
            &mut result.obstacles,
            gx0,
            gy0,
            gx1,
            gy1,
            self.request.route_layers.len(),
        );
        self.stamp_constraints(
            &mut result.obstacles,
            step,
            gx0,
            gy0,
            gx1,
            gy1,
            restrictions,
            obstacle_track_width,
            obstacle_via_size,
            obstacle_via_drill,
            center_extra,
            virtual_track_width,
            virtual_via_size,
        )?;
        Ok((result.obstacles, window, step))
    }

    fn stamp_constraints(
        &self,
        map: &mut GridObstacleMap,
        step: i32,
        gx0: i32,
        gy0: i32,
        gx1: i32,
        gy1: i32,
        restrictions: &[BandRestriction],
        track_width: f64,
        via_size: f64,
        via_drill: f64,
        center_extra: f64,
        virtual_track_width: f64,
        virtual_via_size: f64,
    ) -> Result<(), String> {
        for (li, layer) in self.request.route_layers.iter().enumerate() {
            let mut y = gy0;
            while y <= gy1 {
                let mut x = gx0;
                while x <= gx1 {
                    let p = [
                        x as f64 * self.request.rules.grid_step,
                        y as f64 * self.request.rules.grid_step,
                    ];
                    if self
                        .request
                        .band
                        .as_deref()
                        .is_some_and(|b| !b.contains(p, layer))
                        || !restrictions_allow(restrictions, p, layer)
                    {
                        map.add_blocked_cell(x, y, li);
                    }
                    x = x.saturating_add(1);
                }
                y = y.saturating_add(1);
                if self.request.deadline.is_some_and(|d| Instant::now() >= d) {
                    return Err("pair deadline while applying band".into());
                }
            }
        }
        let step_mm = step as f64 / self.board.units_per_mm;
        for tube in &self.request.virtual_tracks {
            let Some(layer) = self
                .request
                .route_layers
                .iter()
                .position(|l| l == &tube.layer)
            else {
                continue;
            };
            let track_margin = virtual_track_width / 2.0
                + self.request.rules.clearance
                + track_width / 2.0
                + center_extra;
            for [x, y] in
                float_capsule_cells(tube.a, tube.b, track_margin, step_mm, gx0, gy0, gx1, gy1)
            {
                map.add_static_blocked_cell(x, y, layer)
            }
            let via_margin = virtual_track_width / 2.0
                + self.request.rules.clearance
                + via_size / 2.0
                + center_extra;
            for [x, y] in
                float_capsule_cells(tube.a, tube.b, via_margin, step_mm, gx0, gy0, gx1, gy1)
            {
                map.add_static_blocked_via(x, y)
            }
        }
        for via in &self.request.virtual_vias {
            let track_margin = virtual_via_size / 2.0
                + self.request.rules.clearance
                + track_width / 2.0
                + center_extra;
            for [x, y] in float_capsule_cells(
                via.point,
                via.point,
                track_margin,
                step_mm,
                gx0,
                gy0,
                gx1,
                gy1,
            ) {
                for layer in 0..self.request.route_layers.len() {
                    map.add_static_blocked_cell(x, y, layer)
                }
            }
            let outer_margin =
                virtual_via_size / 2.0 + via_size / 2.0 + self.request.rules.clearance;
            for [x, y] in float_capsule_cells(
                via.point,
                via.point,
                outer_margin,
                step_mm,
                gx0,
                gy0,
                gx1,
                gy1,
            ) {
                map.add_static_blocked_via(x, y)
            }
            let hole_margin = via_drill + self.request.rules.hole_to_hole_clearance;
            for [x, y] in float_capsule_cells(
                via.point,
                via.point,
                hole_margin,
                step_mm,
                gx0,
                gy0,
                gx1,
                gy1,
            ) {
                map.add_static_blocked_via(x, y)
            }
        }
        Ok(())
    }

    fn stamp_other_pair(
        &self,
        map: &mut GridObstacleMap,
        step: i32,
        window: IntBox,
        pieces: &[PairPiece],
        vias: &[PairVia],
    ) -> Result<(), String> {
        let step_mm = step as f64 / self.board.units_per_mm;
        let gap = self
            .request
            .pair
            .as_ref()
            .map(|p| p.gap)
            .unwrap_or(self.request.rules.pair_gap);
        let clearance = self.request.rules.clearance + (self.request.rules.track_width + gap) / 2.0;
        let gx0 = (window.ll.x as f64 / step as f64).ceil() as i32;
        let gy0 = (window.ll.y as f64 / step as f64).ceil() as i32;
        let gx1 = (window.ur.x as f64 / step as f64).floor() as i32;
        let gy1 = (window.ur.y as f64 / step as f64).floor() as i32;
        for piece in pieces {
            let Some(layer) = self
                .request
                .route_layers
                .iter()
                .position(|l| l == &piece.layer)
            else {
                continue;
            };
            for [x, y] in float_capsule_cells(
                piece.a,
                piece.b,
                piece.width / 2.0 + clearance,
                step_mm,
                gx0,
                gy0,
                gx1,
                gy1,
            ) {
                map.add_static_blocked_cell(x, y, layer)
            }
        }
        for via in vias {
            let track_margin = via.diameter / 2.0 + clearance;
            for [x, y] in float_capsule_cells(
                via.point,
                via.point,
                track_margin,
                step_mm,
                gx0,
                gy0,
                gx1,
                gy1,
            ) {
                for layer in 0..self.request.route_layers.len() {
                    map.add_static_blocked_cell(x, y, layer)
                }
            }
            let via_margin = via.diameter / 2.0 + self.request.rules.via_size / 2.0 + clearance;
            for [x, y] in float_capsule_cells(
                via.point, via.point, via_margin, step_mm, gx0, gy0, gx1, gy1,
            ) {
                map.add_static_blocked_via(x, y)
            }
            let hole_margin = via.drill / 2.0
                + self.request.rules.via_drill / 2.0
                + self.request.rules.hole_to_hole_clearance;
            for [x, y] in float_capsule_cells(
                via.point,
                via.point,
                hole_margin,
                step_mm,
                gx0,
                gy0,
                gx1,
                gy1,
            ) {
                map.add_static_blocked_via(x, y)
            }
        }
        Ok(())
    }
}

impl PairConnectionBackend for NativePairRouter<'_> {
    fn validate_pieces(
        &mut self,
        request: &RungRequest,
        pieces: &[PairPiece],
        vias: &[PairVia],
        virtual_tolerance: f64,
        check_band: bool,
        band_ends: &[PairBandEnd],
        window_points: &[Point],
        margin_extra: f64,
        other_copper: &[PairPiece],
        other_vias: &[PairVia],
    ) -> Result<Option<String>, String> {
        super::pair_validation::validate_pieces(
            request,
            pieces,
            vias,
            virtual_tolerance,
            check_band,
            band_ends,
            window_points,
            margin_extra,
            other_copper,
            other_vias,
        )
    }

    fn route_envelope(
        &mut self,
        request: &RungRequest,
        e: &EnvelopeRequest,
    ) -> Result<Option<EnvelopeRoute>, String> {
        let mut pts = vec![e.start, e.end];
        pts.extend(e.window_points.iter().copied());
        let (mut map, _, step) = self.make_map(
            &pts,
            e.margin,
            &[e.positive_net_id, e.negative_net_id],
            0.0,
            &e.band_restrictions,
            e.envelope_track_width,
            e.envelope_via_size,
            e.envelope_via_drill,
            0.0,
            e.virtual_size.0,
            e.virtual_size.1,
        )?;
        let sl = request
            .route_layers
            .iter()
            .position(|x| x == &e.start_layer)
            .ok_or("envelope start layer missing")?;
        let tl = request
            .route_layers
            .iter()
            .position(|x| x == &e.end_layer)
            .ok_or("envelope end layer missing")?;
        let cell = |p: Point| -> (i32, i32) {
            (
                (p[0] * self.board.units_per_mm / step as f64).round_ties_even() as i32,
                (p[1] * self.board.units_per_mm / step as f64).round_ties_even() as i32,
            )
        };
        let (sx, sy) = cell(e.start);
        let (tx, ty) = cell(e.end);
        map.add_source_target_cell(sx, sy, sl);
        map.add_source_target_cell(tx, ty, tl);
        let (path, _, _) = super::native_rung::run_source_grid(
            &map,
            request.rules.grid_step,
            vec![(sx, sy, sl as u8)],
            vec![(tx, ty, tl as u8)],
            request.max_iterations,
            TrackMarginArg::Scalar(0.0),
            None,
            None,
        );
        let Some(path) = path else { return Ok(None) };
        let mut runs = Vec::new();
        let mut vias = Vec::new();
        let mut current_layer = None;
        for &(x, y, l) in &path {
            let layer = request
                .route_layers
                .get(l as usize)
                .ok_or("envelope path layer out of range")?
                .clone();
            let p = [
                x as f64 * step as f64 / self.board.units_per_mm,
                y as f64 * step as f64 / self.board.units_per_mm,
            ];
            if current_layer.as_deref() != Some(layer.as_str()) {
                if let Some(prev) = current_layer.take() {
                    if let Some(last) = runs
                        .last()
                        .and_then(|r: &EnvelopeRun| r.points.last())
                        .copied()
                    {
                        vias.push(PairVia {
                            point: last,
                            diameter: e.envelope_via_size,
                            drill: e.envelope_via_drill,
                            layers: vec![prev, layer.clone()],
                            net_id: e.positive_net_id,
                        });
                    }
                }
                runs.push(EnvelopeRun {
                    layer: layer.clone(),
                    points: vec![p],
                });
                current_layer = Some(layer);
            } else if let Some(r) = runs.last_mut() {
                if r.points.last() != Some(&p) {
                    r.points.push(p)
                }
            }
        }
        if let Some(r) = runs.first_mut() {
            if let Some(p) = r.points.first_mut() {
                *p = e.start
            }
        }
        if let Some(r) = runs.last_mut() {
            if let Some(p) = r.points.last_mut() {
                *p = e.end
            }
        }
        Ok(Some(EnvelopeRoute { runs, vias }))
    }

    fn route_pair_pose(
        &mut self,
        request: &RungRequest,
        p: &PairPoseRequest,
    ) -> Result<PairPoseOutcome, String> {
        let mut pts = vec![
            p.start_points[0],
            p.start_points[1],
            p.end_points[0],
            p.end_points[1],
        ];
        pts.extend(p.window_points.iter().copied());
        let (mut map, window, step) = self.make_map(
            &pts,
            p.margin,
            &[self.p, self.n],
            (self.request.rules.track_width + p.gap) / 2.0,
            &p.band_restrictions,
            self.request.rules.track_width,
            self.request.rules.via_size,
            self.request.rules.via_drill,
            (self.request.rules.track_width + p.gap) / 2.0,
            self.request.rules.track_width,
            self.request.rules.via_size,
        )?;
        // `_connect_pair_crossover` routes its second span against the first
        // span's copper. Same-call approach connectors are own pair copper in
        // the source map and remain validated as candidate copper afterwards.
        self.stamp_other_pair(&mut map, step, window, &p.other_copper, &p.other_vias)?;
        let li = |name: &str| {
            request
                .route_layers
                .iter()
                .position(|x| x == name)
                .ok_or_else(|| format!("pair pose layer absent: {name}"))
        };
        let sl = li(&p.start_layer)?;
        let tl = li(&p.end_layer)?;
        let mid = |q: [Point; 2]| [(q[0][0] + q[1][0]) / 2.0, (q[0][1] + q[1][1]) / 2.0];
        let (s_x, s_y) = grid(mid(p.start_points), step, self.board.units_per_mm);
        let (t_x, t_y) = grid(mid(p.end_points), step, self.board.units_per_mm);
        map.add_source_target_cell(s_x, s_y, sl);
        map.add_source_target_cell(t_x, t_y, tl);
        let dir_idx = |d: Option<Point>| -> u8 {
            let Some(d) = d else { return 0 };
            let angle = d[1].atan2(d[0]);
            (((angle / (std::f64::consts::PI / 4.0)).round() as i32).rem_euclid(8)) as u8
        };
        let min_radius = (request.rules.min_turning_radius / request.rules.grid_step).max(0.0);
        let turn_cost = (min_radius * std::f64::consts::PI / 4.0 * 1000.0) as i32;
        let max_turn_units = (request.rules.max_turn_angle / 45.0 + 0.5) as i32;
        // `spacing_mm` is the center offset of each leg; PoseRouter takes the
        // full P-to-N separation in cells.
        let spacing = ((self.request.rules.track_width + p.gap) / self.request.rules.grid_step
            + 0.5)
            .floor()
            .max(1.0) as i32;
        let pair_clearance = super::native_rung::pair_clearance_mm(
            &self.board,
            self.p,
            self.n,
            &request.route_layers,
        )?;
        let track_via_clearance = (pair_clearance
            + self.request.rules.track_width / 2.0
            + self.request.rules.via_size / 2.0)
            * self.request.rules.routing_clearance_margin;
        let via_spacing_mm = ((self.request.rules.track_width + p.gap) / 2.0)
            .max((self.request.rules.via_size + pair_clearance) / 2.0)
            .max(track_via_clearance - (self.request.rules.track_width + p.gap) / 2.0);
        let via_spacing = ((via_spacing_mm / self.request.rules.grid_step) + 0.5)
            .floor()
            .max(1.0) as i32;
        // GridRouteConfig scales its 75-step default from the 0.1 mm
        // reference grid before the differential router doubles it.
        let via_cost_units = (75.0 * 1000.0 * (0.1 / request.rules.grid_step)) as i32;
        let via_cost = via_cost_units.saturating_mul(2);
        let router = PoseRouter::new(
            via_cost,
            2.3,
            turn_cost,
            min_radius,
            10,
            spacing,
            max_turn_units,
            0,
            0,
            0,
            0,
            0,
            Some(vec![1000; request.route_layers.len()]),
            None,
            250,
        );
        let (path, iters, frontier, _) = router.route_pose_with_frontier(
            &map,
            s_x,
            s_y,
            sl as u8,
            dir_idx(p.start_direction),
            t_x,
            t_y,
            tl as u8,
            dir_idx(p.end_direction),
            request
                .max_iterations
                .saturating_mul(8)
                .min(u32::MAX as usize) as u32,
            Some(via_spacing),
        );
        let Some(path) = path else {
            let frontier_empty = frontier.is_empty();
            return Ok(PairPoseOutcome::Refused {
                reason: Some("source PoseRouter returned no path".into()),
                report: Some(super::braid_connect::RungReport {
                    blocked_cells: frontier
                        .into_iter()
                        .map(|(x, y, l)| (x, y, l as usize))
                        .collect(),
                    route_layers: request.route_layers.clone(),
                    window: [
                        window.ll.x as f64 / self.board.units_per_mm,
                        window.ll.y as f64 / self.board.units_per_mm,
                        window.ur.x as f64 / self.board.units_per_mm,
                        window.ur.y as f64 / self.board.units_per_mm,
                    ],
                    grid_step_mm: request.rules.grid_step,
                    expanded: iters as usize,
                    truncated: false,
                }),
                empty: true,
                frontier_empty,
                escaped_window: false,
            });
        };
        let pair_clearance = super::native_rung::pair_clearance_mm(
            &self.board,
            self.p,
            self.n,
            &request.route_layers,
        )?;
        let cfg = PairPathConfig {
            grid_step_mm: request.rules.grid_step,
            route_layers: request.route_layers.clone(),
            track_width: request.rules.track_width,
            max_track_width: request.rules.track_width,
            via_size: request.rules.via_size,
            via_drill: request.rules.via_drill,
            pair_clearance,
            hole_to_hole_clearance: request.rules.hole_to_hole_clearance,
            routing_clearance_margin: request.rules.routing_clearance_margin,
            centerline_setback_mm: None,
        };
        let route = pair_path::pose_path_to_geometry(&path, p, &cfg, self.p, self.n)?;
        Ok(PairPoseOutcome::Found(route))
    }
}

fn grid(p: Point, step: i32, upm: f64) -> (i32, i32) {
    (
        (p[0] * upm / step as f64).round_ties_even() as i32,
        (p[1] * upm / step as f64).round_ties_even() as i32,
    )
}

/// Source routing_utils::_capsule_mask: test global grid points against the
/// original floating segment, with the source strict boundary epsilon.
fn float_capsule_cells(
    a: Point,
    b: Point,
    margin: f64,
    step: f64,
    gx0: i32,
    gy0: i32,
    gx1: i32,
    gy1: i32,
) -> Vec<[i32; 2]> {
    let effective = margin - 1e-6;
    if !effective.is_finite() || effective <= 0.0 || !step.is_finite() || step <= 0.0 {
        return Vec::new();
    }
    let lo_x = (((a[0].min(b[0]) - margin) / step).floor() as i32).max(gx0);
    let hi_x = (((a[0].max(b[0]) + margin) / step).ceil() as i32).min(gx1);
    let lo_y = (((a[1].min(b[1]) - margin) / step).floor() as i32).max(gy0);
    let hi_y = (((a[1].max(b[1]) + margin) / step).ceil() as i32).min(gy1);
    if lo_x > hi_x || lo_y > hi_y {
        return Vec::new();
    }
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let length_sq = dx * dx + dy * dy;
    let threshold = effective * effective;
    let mut cells = Vec::new();
    for gx in lo_x..=hi_x {
        let x = gx as f64 * step;
        for gy in lo_y..=hi_y {
            let y = gy as f64 * step;
            let t = if length_sq <= 0.0 {
                0.0
            } else {
                (((x - a[0]) * dx + (y - a[1]) * dy) / length_sq).clamp(0.0, 1.0)
            };
            let ex = x - (a[0] + t * dx);
            let ey = y - (a[1] + t * dy);
            if ex * ex + ey * ey < threshold {
                cells.push([gx, gy]);
            }
        }
    }
    cells
}

fn restrictions_allow(rs: &[BandRestriction], p: Point, layer: &str) -> bool {
    rs.iter().all(|r| match r {
        BandRestriction::OutsideDisc {
            center,
            radius,
            layer: l,
        } if l == layer => dist(*center, p) > *radius,
        BandRestriction::ForwardHalfDisc {
            center,
            direction,
            radius,
            grid,
        } => {
            let dx = p[0] - center[0];
            let dy = p[1] - center[1];
            let norm = (direction[0] * direction[0] + direction[1] * direction[1]).sqrt();
            let dot = if norm > 0.0 {
                (dx * direction[0] + dy * direction[1]) / norm
            } else {
                0.0
            };
            !(dot < 0.0 && dx * dx + dy * dy <= (radius + grid).powi(2))
        }
        _ => true,
    })
}
fn dist(a: Point, b: Point) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
}
