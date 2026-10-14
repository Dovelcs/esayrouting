//! Source `diff_pair_routing.py` PoseRouter path materialization.
//!
//! The input is one coupled PoseRouter result. This module converts its pose
//! cells to a centerline, applies the source bisector offset to both polarities,
//! lays out paired vias at layer transitions, then emits P/N pieces and vias.
use super::pair_connection::{PairPiece, PairPoseRequest, PairPoseRoute, PairVia};
use fr_engine::ids::NetNo;

pub type Point = [f64; 2];
type PathPoint = (f64, f64, usize);

#[derive(Clone, Debug)]
pub struct PairPathConfig {
    /// The pose grid cell size in millimetres.
    pub grid_step_mm: f64,
    /// Layer names indexed by the PoseRouter layer index.
    pub route_layers: Vec<String>,
    /// Source `GridRouteConfig.track_width` and `get_max_track_width()`.
    pub track_width: f64,
    pub max_track_width: f64,
    pub via_size: f64,
    pub via_drill: f64,
    /// Source `pair_clearance(p_id, n_id, kind="stack")` value.
    pub pair_clearance: f64,
    pub hole_to_hole_clearance: f64,
    pub routing_clearance_margin: f64,
    /// `None` is the source AWX `connect.make_config` default: four centerline
    /// half-spacings from each pair endpoint.
    pub centerline_setback_mm: Option<f64>,
}

/// Convert one original PoseRouter path to the coupled route copper.
/// Pose entries use the router's `(gx, gy, theta_idx, layer_idx)` format. The
/// pair request's endpoint points are ordered positive then negative.
pub fn pose_path_to_geometry(
    pose_path: &[(i32, i32, u8, u8)],
    pose: &PairPoseRequest,
    config: &PairPathConfig,
    positive_net_id: NetNo,
    negative_net_id: NetNo,
) -> Result<PairPoseRoute, String> {
    validate(config, pose)?;
    if pose_path.is_empty() {
        return Ok(PairPoseRoute {
            pieces: Vec::new(),
            vias: Vec::new(),
        });
    }

    // diff_pair_routing.py removes in-place turns before geometry conversion:
    // a pose's theta matters to the search, but not to the emitted centerline.
    let mut grid_path: Vec<(i32, i32, usize)> = Vec::with_capacity(pose_path.len());
    for &(gx, gy, _theta, layer) in pose_path {
        let layer = layer as usize;
        if layer >= config.route_layers.len() {
            return Err(format!(
                "PoseRouter layer index {layer} is outside the routing stack"
            ));
        }
        if grid_path
            .last()
            .is_some_and(|p| p.0 == gx && p.1 == gy && p.2 == layer)
        {
            continue;
        }
        grid_path.push((gx, gy, layer));
    }
    let simplified = simplify_path(&grid_path);
    let centerline: Vec<PathPoint> = simplified
        .iter()
        .map(|&(gx, gy, layer)| {
            (
                gx as f64 * config.grid_step_mm,
                gy as f64 * config.grid_step_mm,
                layer,
            )
        })
        .collect();

    let p_start = pose.start_points[0];
    let n_start = pose.start_points[1];
    let p_end = pose.end_points[0];
    let n_end = pose.end_points[1];
    let p_sign = source_p_sign(&centerline, p_start, n_start);
    let n_sign = -p_sign;
    let spacing = (config.track_width + pose.gap) / 2.0;
    let setback = config.centerline_setback_mm.unwrap_or(spacing * 4.0);
    let center_start = midpoint(p_start, n_start);
    let center_end = midpoint(pose.end_points[0], pose.end_points[1]);

    let start_actual_dir = pose.start_direction;
    let end_actual_dir = pose.end_direction;
    let mut centerline = centerline;
    // The source restores a snapped centerline terminal only when it is already
    // within 0.75 grid cells of the exact setback point.
    for (idx, center, direction) in [
        (0usize, center_start, start_actual_dir),
        (usize::MAX, center_end, end_actual_dir),
    ] {
        let Some(direction) = direction else { continue };
        let exact = [
            center[0] + direction[0] * setback,
            center[1] + direction[1] * setback,
        ];
        let actual_idx = if idx == usize::MAX {
            centerline.len().saturating_sub(1)
        } else {
            idx
        };
        if let Some(point) = centerline.get_mut(actual_idx) {
            if distance([point.0, point.1], exact) <= config.grid_step_mm * 0.75 {
                point.0 = exact[0];
                point.1 = exact[1];
            }
        }
    }

    let end_offset_dir = end_actual_dir.map(|d| [-d[0], -d[1]]);
    let mut p_path = offset_path(
        &centerline,
        p_sign,
        spacing,
        start_actual_dir,
        end_offset_dir,
    );
    let mut n_path = offset_path(
        &centerline,
        n_sign,
        spacing,
        start_actual_dir,
        end_offset_dir,
    );
    let via_params = via_parameters(config, spacing)?;
    process_via_positions(
        &simplified,
        &mut p_path,
        &mut n_path,
        config,
        p_sign,
        n_sign,
        spacing * 2.0,
        via_params,
    )?;

    let start_connector_dir = pose.start_direction.unwrap_or([0.0, 0.0]);
    // Source `_float_path_to_geometry` uses the target escape heading as-is
    // for its optional connector extension.
    let end_connector_dir = pose.end_direction.unwrap_or([0.0, 0.0]);
    let p_src_route = p_path.first().map(|p| [p.0, p.1]).unwrap_or(p_start);
    let n_src_route = n_path.first().map(|p| [p.0, p.1]).unwrap_or(n_start);
    let p_tgt_route = p_path.last().map(|p| [p.0, p.1]).unwrap_or(p_end);
    let n_tgt_route = n_path.last().map(|p| [p.0, p.1]).unwrap_or(n_end);
    let (src_p_ext, src_n_ext) = parallel_extensions(
        p_start,
        n_start,
        p_src_route,
        n_src_route,
        start_connector_dir,
        p_sign,
    );
    let (tgt_p_ext, tgt_n_ext) = parallel_extensions(
        p_end,
        n_end,
        p_tgt_route,
        n_tgt_route,
        end_connector_dir,
        p_sign,
    );
    let (p_pieces, p_vias) = float_path_to_geometry(
        &p_path,
        p_start,
        p_end,
        start_connector_dir,
        end_connector_dir,
        src_p_ext,
        tgt_p_ext,
        config,
        positive_net_id,
    )?;
    let (n_pieces, n_vias) = float_path_to_geometry(
        &n_path,
        n_start,
        n_end,
        start_connector_dir,
        end_connector_dir,
        src_n_ext,
        tgt_n_ext,
        config,
        negative_net_id,
    )?;
    let mut pieces = p_pieces;
    pieces.extend(n_pieces);
    let mut vias = p_vias;
    vias.extend(n_vias);
    Ok(PairPoseRoute { pieces, vias })
}

fn validate(config: &PairPathConfig, pose: &PairPoseRequest) -> Result<(), String> {
    if !config.grid_step_mm.is_finite() || config.grid_step_mm <= 0.0 {
        return Err("pair path grid step must be finite and positive".into());
    }
    if config.route_layers.is_empty() {
        return Err("pair path requires a routing layer stack".into());
    }
    for (name, value) in [
        ("track width", config.track_width),
        ("maximum track width", config.max_track_width),
        ("via size", config.via_size),
        ("via drill", config.via_drill),
        ("pair clearance", config.pair_clearance),
        ("hole-to-hole clearance", config.hole_to_hole_clearance),
        ("clearance margin", config.routing_clearance_margin),
        ("pair gap", pose.gap),
    ] {
        if !value.is_finite() || value < 0.0 {
            return Err(format!("pair path {name} must be finite and nonnegative"));
        }
    }
    if config.track_width <= 0.0
        || config.max_track_width <= 0.0
        || config.via_size <= config.via_drill
    {
        return Err("pair path track/via dimensions are invalid".into());
    }
    if config
        .centerline_setback_mm
        .is_some_and(|d| !d.is_finite() || d < 0.0)
    {
        return Err("pair path centerline setback must be finite and nonnegative".into());
    }
    if pose.start_layer.is_empty() || pose.end_layer.is_empty() {
        return Err("pair path endpoint layers are required".into());
    }
    Ok(())
}

fn simplify_path(path: &[(i32, i32, usize)]) -> Vec<(i32, i32, usize)> {
    if path.len() <= 2 {
        return path.to_vec();
    }
    let mut out = vec![path[0]];
    for i in 1..path.len() - 1 {
        let (prev, curr, next) = (path[i - 1], path[i], path[i + 1]);
        if prev.2 != curr.2 || curr.2 != next.2 {
            out.push(curr);
            continue;
        }
        let (dx1, dy1) = ((curr.0 - prev.0) as f64, (curr.1 - prev.1) as f64);
        let (dx2, dy2) = ((next.0 - curr.0) as f64, (next.1 - curr.1) as f64);
        let l1 = (dx1 * dx1 + dy1 * dy1).sqrt().max(1.0);
        let l2 = (dx2 * dx2 + dy2 * dy2).sqrt().max(1.0);
        if (dx1 / l1 - dx2 / l2).abs() > 0.01 || (dy1 / l1 - dy2 / l2).abs() > 0.01 {
            out.push(curr);
        }
    }
    out.push(*path.last().unwrap());
    out
}

fn source_p_sign(path: &[PathPoint], p_start: Point, n_start: Point) -> f64 {
    let (dx, dy) = if path.len() >= 2 {
        (path[1].0 - path[0].0, path[1].1 - path[0].1)
    } else {
        (1.0, 0.0)
    };
    let mid = midpoint(p_start, n_start);
    let cross = dx * (p_start[1] - mid[1]) - dy * (p_start[0] - mid[0]);
    if cross >= 0.0 {
        1.0
    } else {
        -1.0
    }
}

fn offset_path(
    path: &[PathPoint],
    sign: f64,
    spacing: f64,
    start_dir: Option<Point>,
    end_dir: Option<Point>,
) -> Vec<PathPoint> {
    if path.len() < 2 {
        return path.to_vec();
    }
    let mut out = Vec::with_capacity(path.len());
    for i in 0..path.len() {
        let (x, y, layer) = path[i];
        let mut use_corner_scale = true;
        let [dx, dy] = if i == 0 {
            let seg = source_unit([path[1].0 - x, path[1].1 - y]);
            if let Some(start) = start_dir {
                let d = source_unit(start);
                [d[0] + seg[0], d[1] + seg[1]]
            } else {
                use_corner_scale = false;
                seg
            }
        } else if i + 1 == path.len() {
            let seg = source_unit([x - path[i - 1].0, y - path[i - 1].1]);
            if let Some(end) = end_dir {
                let d = source_unit(end);
                [seg[0] + d[0], seg[1] + d[1]]
            } else {
                use_corner_scale = false;
                seg
            }
        } else {
            let prev = path[i - 1];
            let next = path[i + 1];
            if prev.2 != layer || next.2 != layer {
                [x - prev.0, y - prev.1]
            } else {
                let incoming = source_unit([x - prev.0, y - prev.1]);
                let outgoing = source_unit([next.0 - x, next.1 - y]);
                let sum = [incoming[0] + outgoing[0], incoming[1] + outgoing[1]];
                if sum[0].abs() < 0.01 && sum[1].abs() < 0.01 {
                    [x - prev.0, y - prev.1]
                } else {
                    sum
                }
            }
        };
        let raw_length: f64 = (dx * dx + dy * dy).sqrt();
        let length = if raw_length == 0.0 { 1.0 } else { raw_length };
        let (ndx, ndy) = (dx / length, dy / length);
        let corner_scale = if use_corner_scale {
            if length > 0.1 {
                (2.0 / length).min(3.0)
            } else {
                1.0
            }
        } else {
            1.0
        };
        out.push((
            x - ndy * sign * spacing * corner_scale,
            y + ndx * sign * spacing * corner_scale,
            layer,
        ));
    }
    out
}

#[derive(Clone, Copy)]
struct ViaParams {
    via_spacing: f64,
    track_via_clearance: f64,
}

fn via_parameters(config: &PairPathConfig, spacing: f64) -> Result<ViaParams, String> {
    let min_center = (config.via_size + config.pair_clearance)
        .max(config.via_drill + config.hole_to_hole_clearance);
    let track_via_clearance =
        (config.pair_clearance + config.max_track_width / 2.0 + config.via_size / 2.0)
            * config.routing_clearance_margin;
    let via_spacing = spacing
        .max(min_center / 2.0)
        .max(track_via_clearance - spacing);
    if !via_spacing.is_finite() || via_spacing < 0.0 || !track_via_clearance.is_finite() {
        return Err("pair path via geometry is non-finite".into());
    }
    Ok(ViaParams {
        via_spacing,
        track_via_clearance,
    })
}

fn process_via_positions(
    centerline_grid: &[(i32, i32, usize)],
    p_path: &mut Vec<PathPoint>,
    n_path: &mut Vec<PathPoint>,
    config: &PairPathConfig,
    p_sign: f64,
    n_sign: f64,
    pair_spacing: f64,
    params: ViaParams,
) -> Result<(), String> {
    if p_path.is_empty() || n_path.is_empty() || centerline_grid.len() < 2 {
        return Ok(());
    }
    let transitions: Vec<_> = (0..centerline_grid.len() - 1)
        .filter(|&i| centerline_grid[i].2 != centerline_grid[i + 1].2)
        .collect();
    for i in transitions.into_iter().rev() {
        if i + 1 >= p_path.len() || i + 1 >= n_path.len() {
            return Err("pair path layer transition index is outside its offset path".into());
        }
        let (gx, gy, layer1) = centerline_grid[i];
        let layer2 = centerline_grid[i + 1].2;
        let center = [
            gx as f64 * config.grid_step_mm,
            gy as f64 * config.grid_step_mm,
        ];
        let incoming = if i > 0 {
            normalize(
                [
                    center[0] - centerline_grid[i - 1].0 as f64 * config.grid_step_mm,
                    center[1] - centerline_grid[i - 1].1 as f64 * config.grid_step_mm,
                ],
                [1.0, 0.0],
            )
        } else {
            [1.0, 0.0]
        };
        let outgoing = if i + 2 < centerline_grid.len() {
            normalize(
                [
                    centerline_grid[i + 2].0 as f64 * config.grid_step_mm - center[0],
                    centerline_grid[i + 2].1 as f64 * config.grid_step_mm - center[1],
                ],
                incoming,
            )
        } else {
            incoming
        };
        let pvec = [
            (-incoming[1] - outgoing[1]) / 2.0,
            (incoming[0] + outgoing[0]) / 2.0,
        ];
        let perp = if (pvec[0] * pvec[0] + pvec[1] * pvec[1]).sqrt() > 0.001 {
            normalize(pvec, [-incoming[1], incoming[0]])
        } else {
            [-incoming[1], incoming[0]]
        };
        let p_via = [
            center[0] + perp[0] * p_sign * params.via_spacing,
            center[1] + perp[1] * p_sign * params.via_spacing,
        ];
        let n_via = [
            center[0] + perp[0] * n_sign * params.via_spacing,
            center[1] + perp[1] * n_sign * params.via_spacing,
        ];
        let in_perp = [-incoming[1], incoming[0]];
        let out_perp = [-outgoing[1], outgoing[0]];
        let cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0];
        let inner_is_p = if cross < 0.0 {
            p_sign < 0.0
        } else {
            p_sign > 0.0
        };
        let (inner_via, outer_via) = if inner_is_p {
            (p_via, n_via)
        } else {
            (n_via, p_via)
        };
        let inner_to_outer = [outer_via[0] - inner_via[0], outer_via[1] - inner_via[1]];
        let sign_in = if dot(inner_to_outer, in_perp) >= 0.0 {
            1.0
        } else {
            -1.0
        };
        let sign_out = if dot(inner_to_outer, out_perp) >= 0.0 {
            1.0
        } else {
            -1.0
        };
        let d = params.track_via_clearance;
        let (p_approach, n_approach) = if inner_is_p {
            let n = [
                inner_via[0] + in_perp[0] * sign_in * d,
                inner_via[1] + in_perp[1] * sign_in * d,
            ];
            (
                [
                    n[0] - in_perp[0] * sign_in * pair_spacing,
                    n[1] - in_perp[1] * sign_in * pair_spacing,
                ],
                n,
            )
        } else {
            let p = [
                inner_via[0] + in_perp[0] * sign_in * d,
                inner_via[1] + in_perp[1] * sign_in * d,
            ];
            (
                p,
                [
                    p[0] - in_perp[0] * sign_in * pair_spacing,
                    p[1] - in_perp[1] * sign_in * pair_spacing,
                ],
            )
        };
        let (p_exit, n_exit) = if inner_is_p {
            let n = [
                inner_via[0] + out_perp[0] * sign_out * d,
                inner_via[1] + out_perp[1] * sign_out * d,
            ];
            (
                [
                    n[0] - out_perp[0] * sign_out * pair_spacing,
                    n[1] - out_perp[1] * sign_out * pair_spacing,
                ],
                n,
            )
        } else {
            let p = [
                inner_via[0] + out_perp[0] * sign_out * d,
                inner_via[1] + out_perp[1] * sign_out * d,
            ];
            (
                p,
                [
                    p[0] - out_perp[0] * sign_out * pair_spacing,
                    p[1] - out_perp[1] * sign_out * pair_spacing,
                ],
            )
        };
        p_path[i] = (p_approach[0], p_approach[1], layer1);
        n_path[i] = (n_approach[0], n_approach[1], layer1);
        p_path.insert(i + 1, (p_via[0], p_via[1], layer1));
        n_path.insert(i + 1, (n_via[0], n_via[1], layer1));
        p_path[i + 2] = (p_via[0], p_via[1], layer2);
        n_path[i + 2] = (n_via[0], n_via[1], layer2);
        p_path.insert(i + 3, (p_exit[0], p_exit[1], layer2));
        n_path.insert(i + 3, (n_exit[0], n_exit[1], layer2));
    }
    Ok(())
}

fn float_path_to_geometry(
    path: &[PathPoint],
    original_start: Point,
    original_end: Point,
    start_dir: Point,
    end_dir: Point,
    src_extension: f64,
    tgt_extension: f64,
    config: &PairPathConfig,
    net_id: NetNo,
) -> Result<(Vec<PairPiece>, Vec<PairVia>), String> {
    let mut pieces = Vec::new();
    let mut vias = Vec::new();
    if let Some(&(x, y, layer)) = path.first() {
        let name = config
            .route_layers
            .get(layer)
            .ok_or("pair path has invalid source layer")?;
        append_start_connector(
            &mut pieces,
            original_start,
            [x, y],
            start_dir,
            src_extension,
            config.track_width,
            name,
            net_id,
        );
    }
    for pair in path.windows(2) {
        let (x1, y1, l1) = pair[0];
        let (x2, y2, l2) = pair[1];
        if l1 != l2 {
            vias.push(PairVia {
                point: [x1, y1],
                diameter: config.via_size,
                drill: config.via_drill,
                layers: vec!["F.Cu".into(), "B.Cu".into()],
                net_id,
            });
        } else if (x1 - x2).abs() > 0.001 || (y1 - y2).abs() > 0.001 {
            let layer = config
                .route_layers
                .get(l1)
                .ok_or("pair path has invalid segment layer")?
                .clone();
            pieces.push(PairPiece {
                a: [x1, y1],
                b: [x2, y2],
                width: config.track_width,
                layer,
                net_id,
            });
        }
    }
    if let Some(&(x, y, layer)) = path.last() {
        let name = config
            .route_layers
            .get(layer)
            .ok_or("pair path has invalid target layer")?;
        append_end_connector(
            &mut pieces,
            [x, y],
            original_end,
            end_dir,
            tgt_extension,
            config.track_width,
            name,
            net_id,
        );
    }
    Ok((pieces, vias))
}

fn append_start_connector(
    pieces: &mut Vec<PairPiece>,
    endpoint: Point,
    route: Point,
    dir: Point,
    extension: f64,
    width: f64,
    layer: &str,
    net_id: NetNo,
) {
    if distance(endpoint, route) > 0.001 {
        let projection = dot([route[0] - endpoint[0], route[1] - endpoint[1]], dir);
        let use_extension =
            extension > 0.001 && (projection <= 0.001 || projection >= extension - 0.001);
        if use_extension {
            let ext = [
                endpoint[0] + dir[0] * extension,
                endpoint[1] + dir[1] * extension,
            ];
            push_piece(pieces, endpoint, ext, width, layer, net_id);
            push_piece(pieces, ext, route, width, layer, net_id);
        } else {
            push_piece(pieces, endpoint, route, width, layer, net_id);
        }
    }
}

fn append_end_connector(
    pieces: &mut Vec<PairPiece>,
    route: Point,
    endpoint: Point,
    dir: Point,
    extension: f64,
    width: f64,
    layer: &str,
    net_id: NetNo,
) {
    if distance(endpoint, route) > 0.001 {
        let projection = dot([route[0] - endpoint[0], route[1] - endpoint[1]], dir);
        let use_extension =
            extension > 0.001 && (projection <= 0.001 || projection >= extension - 0.001);
        if use_extension {
            let ext = [
                endpoint[0] + dir[0] * extension,
                endpoint[1] + dir[1] * extension,
            ];
            push_piece(pieces, route, ext, width, layer, net_id);
            push_piece(pieces, ext, endpoint, width, layer, net_id);
        } else {
            push_piece(pieces, route, endpoint, width, layer, net_id);
        }
    }
}

fn push_piece(
    pieces: &mut Vec<PairPiece>,
    a: Point,
    b: Point,
    width: f64,
    layer: &str,
    net_id: NetNo,
) {
    pieces.push(PairPiece {
        a,
        b,
        width,
        layer: layer.into(),
        net_id,
    });
}

/// Source `_calculate_parallel_extension`: extend only the polarity whose
/// endpoint connector can be made parallel by an outward stub-direction run.
fn parallel_extensions(
    p_stub: Point,
    n_stub: Point,
    p_route: Point,
    n_route: Point,
    stub_dir: Point,
    p_sign: f64,
) -> (f64, f64) {
    let v_p = [p_route[0] - p_stub[0], p_route[1] - p_stub[1]];
    let v_n = [n_route[0] - n_stub[0], n_route[1] - n_stub[1]];
    let cross = v_p[0] * v_n[1] - v_p[1] * v_n[0];
    if cross.abs() < 0.0001 {
        return (0.0, 0.0);
    }
    let d_cross_vn = stub_dir[0] * v_n[1] - stub_dir[1] * v_n[0];
    let d_cross_vp = stub_dir[0] * v_p[1] - stub_dir[1] * v_p[0];
    let p_ext = if d_cross_vn.abs() > 0.0001 {
        cross / d_cross_vn
    } else {
        0.0
    };
    let n_ext = if d_cross_vp.abs() > 0.0001 {
        -cross / d_cross_vp
    } else {
        0.0
    };
    if p_ext > 0.001 && n_ext > 0.001 {
        if (cross > 0.0 && p_sign < 0.0) || (cross < 0.0 && p_sign > 0.0) {
            (p_ext, 0.0)
        } else {
            (0.0, n_ext)
        }
    } else if p_ext > 0.001 {
        (p_ext, 0.0)
    } else if n_ext > 0.001 {
        (0.0, n_ext)
    } else {
        (0.0, 0.0)
    }
}

fn normalize(v: Point, fallback: Point) -> Point {
    let len = (v[0] * v[0] + v[1] * v[1]).sqrt();
    if len > 1e-12 {
        [v[0] / len, v[1] / len]
    } else {
        fallback
    }
}
fn source_unit(v: Point) -> Point {
    let length = (v[0] * v[0] + v[1] * v[1]).sqrt();
    let length = if length == 0.0 { 1.0 } else { length };
    [v[0] / length, v[1] / length]
}
fn midpoint(a: Point, b: Point) -> Point {
    [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]
}
fn distance(a: Point, b: Point) -> f64 {
    (b[0] - a[0]).hypot(b[1] - a[1])
}
fn dot(a: Point, b: Point) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
