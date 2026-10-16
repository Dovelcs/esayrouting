//! Source `obstacle_map` same-net via-only keep-outs for native rung maps.
//!
//! Copper tracks may still touch their own pads and vias. These helpers only
//! populate the via-placement maps, using current board-unit geometry and the
//! explicit pad/via evidence carried by `RungRequest`.
use super::{
    braid_connect::RungPastePadMetadata,
    native_obstacles::NativeGridConfig,
    whole_context::{ContextPad, CopperShape, CopperVia},
};
use crate::board_adapter::BoardAdapter;
use awx_grid::obstacle_map::GridObstacleMap;
use fr_engine::ids::NetNo;
use std::collections::{BTreeMap, BTreeSet};

type Cell = (i32, i32);
type ViaGeometry = (i32, i32);

/// Return the global cells where a new via must avoid this net's SMD pads and
/// any paste opening metadata that was recorded for those pads.
pub fn same_net_pad_via_keepout_cells(
    board: &BoardAdapter,
    net_id: NetNo,
    config: &NativeGridConfig,
    pads: &[ContextPad],
    paste_pads: &BTreeMap<String, RungPastePadMetadata>,
) -> Result<Vec<Cell>, String> {
    validate_grid(board, config)?;
    if config.same_net_pad_clearance <= 0 {
        return Ok(Vec::new());
    }
    let margin = (config.via_size as f64 / 2.0
        + config.same_net_pad_clearance as f64
        + config.grid_step as f64 / 2.0)
        / board.units_per_mm;
    let step_mm = config.grid_step as f64 / board.units_per_mm;
    let mut result = Vec::new();
    for pad in pads
        .iter()
        .filter(|pad| pad.net_id == net_id as i32 && pad.drill <= 0.0)
    {
        // The source predicate is based on the logical pad's size, shape,
        // rotation, and sub-cell center offset, not on a per-layer copper
        // outline. This also handles pads whose copper outline is unavailable.
        let mut pad_cells = BTreeSet::new();
        pad_cells.extend(source_pad_cells(
            pad,
            margin,
            config.grid_step,
            board.units_per_mm,
        )?);
        if let Some(meta) = paste_pads.get(&pad.key) {
            // A missing source property is represented as no aperture by the
            // DSN import path. Explicit unknown metadata is likewise not
            // turned into an invented shape; explicit shapes are transformed
            // from their recorded source pose to this pad's current pose.
            if let Some(apertures) = &meta.paste_apertures {
                for aperture in apertures {
                    let current = rebase_shape(
                        aperture,
                        meta.source_center,
                        meta.source_rotation_deg,
                        pad.point,
                        pad.rect_rotation,
                    )?;
                    pad_cells.extend(shape_cells(&current, margin, step_mm)?);
                }
            }
        }
        result.extend(pad_cells);
    }
    Ok(result)
}

/// Stamp same-net SMD/paste keep-outs and same-net via-to-via rings. Rings are
/// mirrored into populated via rungs, but this function never allocates a new
/// rung map. A rung search therefore sees the same-net ring beside the base
/// map geometry it already uses.
pub fn add_same_net_via_clearance(
    obstacles: &mut GridObstacleMap,
    board: &BoardAdapter,
    net_id: NetNo,
    config: &NativeGridConfig,
    pads: &[ContextPad],
    existing_vias: &[CopperVia],
    paste_pads: &BTreeMap<String, RungPastePadMetadata>,
) -> Result<(), String> {
    validate_grid(board, config)?;
    let pad_cells = same_net_pad_via_keepout_cells(board, net_id, config, pads, paste_pads)?;
    stamp_cells(obstacles, 0, &pad_cells);

    // #1226 source behavior: re-rasterize the pad keep-out at every populated
    // per-net via rung's own geometry; rung 1 is the armed small-via slot when
    // the map has one extra slot ahead of the per-net sizes.
    for (rung, (diameter, drill)) in populated_pad_rung_geometries(obstacles, config) {
        let mut rung_config = config.clone();
        rung_config.via_size = diameter;
        rung_config.via_drill = drill;
        let cells = same_net_pad_via_keepout_cells(board, net_id, &rung_config, pads, paste_pads)?;
        stamp_cells(obstacles, rung, &cells);
    }

    let ring = same_net_via_ring_cells(board, net_id, config, existing_vias)?;
    stamp_cells(obstacles, 0, &ring);
    for rung in populated_via_rungs(obstacles) {
        stamp_cells(obstacles, rung, &ring);
    }
    Ok(())
}

/// Stamp same-net through-hole pad drill clearance. The centre cell remains
/// landable so the router can reuse the existing through-hole barrel.
pub fn add_same_net_pad_drill_via_clearance(
    obstacles: &mut GridObstacleMap,
    board: &BoardAdapter,
    net_id: NetNo,
    config: &NativeGridConfig,
    pads: &[ContextPad],
) -> Result<(), String> {
    validate_grid(board, config)?;
    if config.hole_to_hole_clearance <= 0 {
        return Ok(());
    }
    let base_drill = same_net_new_via_drill(config, net_id);
    let cells = same_net_pad_drill_keepout_cells(board, net_id, config, pads, base_drill)?;
    stamp_cells(obstacles, 0, &cells);
    for (rung, (_, new_drill)) in populated_pad_rung_geometries(obstacles, config) {
        let rung_cells = same_net_pad_drill_keepout_cells(board, net_id, config, pads, new_drill)?;
        stamp_cells(obstacles, rung, &rung_cells);
    }
    Ok(())
}

fn same_net_pad_drill_keepout_cells(
    board: &BoardAdapter,
    net_id: NetNo,
    config: &NativeGridConfig,
    pads: &[ContextPad],
    new_drill: i32,
) -> Result<Vec<Cell>, String> {
    let mut cells = Vec::new();
    for pad in pads
        .iter()
        .filter(|pad| pad.net_id == net_id as i32 && pad.drill > 0.0)
    {
        let center = to_board_point(pad.point, board.units_per_mm)?;
        let axis_mm = pad.drill_axis.unwrap_or([pad.point, pad.point]);
        let a = to_board_point(axis_mm[0], board.units_per_mm)?;
        let b = to_board_point(axis_mm[1], board.units_per_mm)?;
        let pad_radius = pad.drill * board.units_per_mm / 2.0;
        let required = pad_radius + new_drill as f64 / 2.0 + config.hole_to_hole_clearance as f64;
        if !required.is_finite() || required <= 0.0 {
            return Err(format!(
                "invalid same-net drill clearance for pad {}",
                pad.key
            ));
        }
        let grid = grid_round(center, config.grid_step)?;
        let half_len = distance(a, b) / 2.0;
        let expand = ((required + half_len) / config.grid_step as f64).ceil() as i64 + 1;
        for dx in -expand..=expand {
            for dy in -expand..=expand {
                // Keep the existing plated-barrel center free for layer exchange.
                if dx == 0 && dy == 0 {
                    continue;
                }
                let gx = checked_index(grid.0 as i64 + dx)?;
                let gy = checked_index(grid.1 as i64 + dy)?;
                let cell = cell_center(gx, gy, config.grid_step);
                if point_segment_distance(cell, a, b) < required {
                    cells.push((gx, gy));
                }
            }
        }
    }
    Ok(cells)
}

fn same_net_via_ring_cells(
    board: &BoardAdapter,
    net_id: NetNo,
    config: &NativeGridConfig,
    vias: &[CopperVia],
) -> Result<Vec<Cell>, String> {
    let new_drill = same_net_new_via_drill(config, net_id);
    let mut cells = Vec::new();
    for via in vias.iter().filter(|via| via.net_id == net_id as i32) {
        let center = to_board_point(via.point, board.units_per_mm)?;
        let existing_drill = via.drill * board.units_per_mm;
        let mut ring = (config.via_size + config.clearance) as f64;
        if config.hole_to_hole_clearance > 0 && existing_drill > 0.0 {
            ring = ring.max(
                (existing_drill + new_drill as f64) / 2.0 + config.hole_to_hole_clearance as f64,
            );
        }
        let grid = grid_round(center, config.grid_step)?;
        let off_cells = distance(center, cell_center(grid.0, grid.1, config.grid_step))
            / config.grid_step as f64;
        let radius = (ring / config.grid_step as f64).max(1.0) + off_cells;
        let expansion = radius.ceil() as i64;
        let radius_sq = radius * radius;
        for dx in -expansion..=expansion {
            for dy in -expansion..=expansion {
                if (dx * dx + dy * dy) as f64 <= radius_sq {
                    cells.push((
                        checked_index(grid.0 as i64 + dx)?,
                        checked_index(grid.1 as i64 + dy)?,
                    ));
                }
            }
        }
    }
    Ok(cells)
}

fn same_net_new_via_drill(config: &NativeGridConfig, net_id: NetNo) -> i32 {
    config
        .net_via_sizes
        .get(&net_id)
        .map(|(_, drill)| config.via_drill.max(*drill))
        .unwrap_or(config.via_drill)
}

/// Per-net rung via sizes in source order, plus a populated small-via rung
/// represented conservatively at the configured run via size.
fn populated_pad_rung_geometries(
    obstacles: &GridObstacleMap,
    config: &NativeGridConfig,
) -> Vec<(usize, ViaGeometry)> {
    if !config.ordered_via_rungs.is_empty() {
        return config
            .ordered_via_rungs
            .iter()
            .copied()
            .enumerate()
            .map(|(index, geometry)| (index + 1, geometry))
            .filter(|(rung, _)| obstacles.rung_len(*rung) > 0)
            .collect();
    }

    let run = (config.via_size, config.via_drill);
    let mut per: Vec<_> = config
        .net_via_sizes
        .values()
        .copied()
        .filter(|via| *via != run)
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    per.sort_by(|a, b| b.cmp(a));
    let n = obstacles.rung_count().saturating_sub(1);
    let mut geometries = Vec::new();
    if n == per.len() + 1 {
        for (index, geometry) in per.iter().copied().enumerate() {
            geometries.push((index + 2, geometry));
        }
        if obstacles.rung_len(1) > 0 {
            geometries.push((1, run));
        }
    } else if n == per.len() {
        let mut candidates: Vec<Vec<ViaGeometry>> = per.iter().copied().map(|v| vec![v]).collect();
        for (small_index, small) in per.iter().copied().enumerate() {
            if small.0 >= run.0 {
                continue;
            }
            candidates[0].push(small);
            let rest: Vec<_> = per
                .iter()
                .copied()
                .enumerate()
                .filter(|(index, _)| *index != small_index)
                .map(|(_, geometry)| geometry)
                .collect();
            for rung_index in 1..per.len() {
                candidates[rung_index].push(rest[rung_index - 1]);
            }
        }
        for (index, choices) in candidates.iter().enumerate() {
            let geometry = (
                choices.iter().map(|v| v.0).max().unwrap_or(run.0),
                choices.iter().map(|v| v.1).max().unwrap_or(run.1),
            );
            geometries.push((index + 1, geometry));
        }
    } else if n > 0 {
        let geometry = (
            per.iter()
                .map(|v| v.0)
                .chain([run.0])
                .max()
                .unwrap_or(run.0),
            per.iter()
                .map(|v| v.1)
                .chain([run.1])
                .max()
                .unwrap_or(run.1),
        );
        for rung in 1..=n {
            geometries.push((rung, geometry));
        }
    }
    geometries
        .into_iter()
        .filter(|(rung, _)| obstacles.rung_len(*rung) > 0)
        .collect()
}

fn populated_via_rungs(obstacles: &GridObstacleMap) -> Vec<usize> {
    (1..obstacles.rung_count())
        .filter(|&rung| obstacles.rung_len(rung) > 0)
        .collect()
}

fn stamp_cells(obstacles: &mut GridObstacleMap, rung: usize, cells: &[Cell]) {
    for &(gx, gy) in cells {
        obstacles.add_blocked_via_rung(rung, gx, gy);
    }
}

fn shape_cells(shape: &CopperShape, margin: f64, step: f64) -> Result<Vec<Cell>, String> {
    if !margin.is_finite() || margin < 0.0 || !step.is_finite() || step <= 0.0 {
        return Err("invalid same-net pad keep-out geometry".into());
    }
    let (bounds, geometry_valid) = match shape {
        CopperShape::Circle { center, radius } => (
            [
                center[0] - radius,
                center[1] - radius,
                center[0] + radius,
                center[1] + radius,
            ],
            radius.is_finite() && *radius >= 0.0,
        ),
        CopperShape::Polygon { outline } => {
            if outline.len() < 3 {
                return Err("same-net paste/pad polygon has fewer than three points".into());
            }
            let min_x = outline.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min);
            let min_y = outline.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min);
            let max_x = outline
                .iter()
                .map(|p| p[0])
                .fold(f64::NEG_INFINITY, f64::max);
            let max_y = outline
                .iter()
                .map(|p| p[1])
                .fold(f64::NEG_INFINITY, f64::max);
            ([min_x, min_y, max_x, max_y], true)
        }
        CopperShape::RoundedRectangle {
            center,
            half_size,
            rotation,
            corner_radius,
        } => {
            let angle = rotation.to_radians();
            let ex = half_size[0] * angle.cos().abs() + half_size[1] * angle.sin().abs();
            let ey = half_size[0] * angle.sin().abs() + half_size[1] * angle.cos().abs();
            (
                [
                    center[0] - ex,
                    center[1] - ey,
                    center[0] + ex,
                    center[1] + ey,
                ],
                half_size[0].is_finite()
                    && half_size[1].is_finite()
                    && *corner_radius >= 0.0
                    && corner_radius.is_finite(),
            )
        }
    };
    if !geometry_valid || bounds.iter().any(|v| !v.is_finite()) {
        return Err("non-finite/invalid same-net pad geometry".into());
    }
    let gx0 = floor_index((bounds[0] - margin) / step)?;
    let gy0 = floor_index((bounds[1] - margin) / step)?;
    let gx1 = ceil_index((bounds[2] + margin) / step)?;
    let gy1 = ceil_index((bounds[3] + margin) / step)?;
    let mut cells = Vec::new();
    for gx in gx0..=gx1 {
        for gy in gy0..=gy1 {
            let point = [gx as f64 * step, gy as f64 * step];
            let distance = shape.signed_distance(point);
            if !distance.is_finite() {
                return Err("same-net pad geometry has non-finite signed distance".into());
            }
            if distance < margin {
                cells.push((gx, gy));
            }
        }
    }
    Ok(cells)
}

/// Port of `routing_utils.pad_blocked_cells_array`, including the source
/// strict-boundary test, pad sub-cell offset, rotation, and corner buffer.
fn source_pad_cells(
    pad: &ContextPad,
    margin_mm: f64,
    grid_step: i32,
    units_per_mm: f64,
) -> Result<Vec<Cell>, String> {
    let step = grid_step as f64;
    let margin = margin_mm * units_per_mm;
    let half_w = pad.size[0] * units_per_mm / 2.0;
    let half_h = pad.size[1] * units_per_mm / 2.0;
    let mut corner = match pad.shape.as_str() {
        "circle" | "oval" => half_w.min(half_h),
        "roundrect" => pad.corner_radius * units_per_mm,
        _ => 0.0,
    };
    if !corner.is_finite() {
        corner = 0.0;
    }
    let center = to_board_point(pad.point, units_per_mm)?;
    if !step.is_finite()
        || step <= 0.0
        || !margin.is_finite()
        || margin < 0.0
        || !half_w.is_finite()
        || !half_h.is_finite()
        || half_w < 0.0
        || half_h < 0.0
        || !pad.rect_rotation.is_finite()
    {
        return Err(format!("invalid same-net pad geometry for {}", pad.key));
    }
    let grid = grid_round(center, grid_step)?;
    let off_x = center.0 - grid.0 as f64 * step;
    let off_y = center.1 - grid.1 as f64 * step;
    let angle = pad.rect_rotation.to_radians();
    let (cos, sin) = (angle.cos(), angle.sin());
    let rotated = pad.rect_rotation.abs() > 1e-9 && (pad.rect_rotation.abs() - 180.0).abs() > 1e-9;
    let extent_x = if rotated {
        half_w * cos.abs() + half_h * sin.abs()
    } else {
        half_w
    };
    let extent_y = if rotated {
        half_w * sin.abs() + half_h * cos.abs()
    } else {
        half_h
    };
    let buffer = step / 2.0;
    let x_expand = ((extent_x + margin + buffer) / step).ceil() as i64 + 1;
    let y_expand = ((extent_y + margin + buffer) / step).ceil() as i64 + 1;
    let inner_w = half_w - corner - if corner == 0.0 { buffer } else { 0.0 };
    let inner_h = half_h - corner - if corner == 0.0 { buffer } else { 0.0 };
    let margin_sq = margin * margin;
    let buffered_sq = (margin + buffer) * (margin + buffer);
    let mut cells = Vec::new();
    for ex in -x_expand..=x_expand {
        for ey in -y_expand..=y_expand {
            let gx = checked_index(grid.0 as i64 + ex)?;
            let gy = checked_index(grid.1 as i64 + ey)?;
            let gx_off = ex as f64 * step - off_x;
            let gy_off = ey as f64 * step - off_y;
            let (x, y) = if rotated {
                (gx_off * cos + gy_off * sin, -gx_off * sin + gy_off * cos)
            } else {
                (gx_off, gy_off)
            };
            let ax = x.abs();
            let ay = y.abs();
            let in_corner = ax > inner_w && ay > inner_h;
            let effective_sq = if in_corner { buffered_sq } else { margin_sq };
            let corner_distance_sq = if corner > 0.0 {
                let crw = half_w - corner;
                let crh = half_h - corner;
                if ax > crw && ay > crh {
                    let dist = ((ax - crw).hypot(ay - crh) - corner).max(0.0);
                    Some(dist * dist)
                } else {
                    None
                }
            } else {
                None
            };
            let dist_sq = if let Some(dist_sq) = corner_distance_sq {
                dist_sq
            } else {
                let closest_x = x.clamp(-half_w, half_w);
                let closest_y = y.clamp(-half_h, half_h);
                (x - closest_x).powi(2) + (y - closest_y).powi(2)
            };
            if dist_sq < effective_sq {
                cells.push((gx, gy));
            }
        }
    }
    Ok(cells)
}

fn rebase_shape(
    shape: &CopperShape,
    source_center: [f64; 2],
    source_rotation: f64,
    current_center: [f64; 2],
    current_rotation: f64,
) -> Result<CopperShape, String> {
    if source_center
        .iter()
        .chain(current_center.iter())
        .any(|v| !v.is_finite())
        || !source_rotation.is_finite()
        || !current_rotation.is_finite()
    {
        return Err("paste pad pose is non-finite".into());
    }
    let angle = (current_rotation - source_rotation).to_radians();
    let rotate = |point: [f64; 2]| {
        let x = point[0] - source_center[0];
        let y = point[1] - source_center[1];
        [
            current_center[0] + angle.cos() * x - angle.sin() * y,
            current_center[1] + angle.sin() * x + angle.cos() * y,
        ]
    };
    Ok(match shape {
        CopperShape::Circle { center, radius } => CopperShape::Circle {
            center: rotate(*center),
            radius: *radius,
        },
        CopperShape::Polygon { outline } => CopperShape::Polygon {
            outline: outline.iter().map(|&point| rotate(point)).collect(),
        },
        CopperShape::RoundedRectangle {
            center,
            half_size,
            rotation,
            corner_radius,
        } => CopperShape::RoundedRectangle {
            center: rotate(*center),
            half_size: *half_size,
            rotation: *rotation + current_rotation - source_rotation,
            corner_radius: *corner_radius,
        },
    })
}

fn point_segment_distance(point: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let length_sq = dx * dx + dy * dy;
    if length_sq <= 0.0 {
        return (point.0 - a.0).hypot(point.1 - a.1);
    }
    let t = (((point.0 - a.0) * dx + (point.1 - a.1) * dy) / length_sq).clamp(0.0, 1.0);
    (point.0 - (a.0 + t * dx)).hypot(point.1 - (a.1 + t * dy))
}

fn to_board_point(point_mm: [f64; 2], units_per_mm: f64) -> Result<(f64, f64), String> {
    if !units_per_mm.is_finite() || units_per_mm <= 0.0 || point_mm.iter().any(|v| !v.is_finite()) {
        return Err("invalid same-net obstacle board coordinate scale/point".into());
    }
    Ok((point_mm[0] * units_per_mm, point_mm[1] * units_per_mm))
}

fn grid_round(point: (f64, f64), step: i32) -> Result<Cell, String> {
    if step <= 0 {
        return Err("same-net obstacle grid step must be positive".into());
    }
    Ok((
        checked_index((point.0 / step as f64).round_ties_even() as i64)?,
        checked_index((point.1 / step as f64).round_ties_even() as i64)?,
    ))
}

fn cell_center(gx: i32, gy: i32, step: i32) -> (f64, f64) {
    ((gx as f64) * step as f64, (gy as f64) * step as f64)
}

fn floor_index(value: f64) -> Result<i32, String> {
    if !value.is_finite() {
        return Err("non-finite same-net grid bound".into());
    }
    checked_index(value.floor() as i64)
}

fn ceil_index(value: f64) -> Result<i32, String> {
    if !value.is_finite() {
        return Err("non-finite same-net grid bound".into());
    }
    checked_index(value.ceil() as i64)
}

fn checked_index(value: i64) -> Result<i32, String> {
    i32::try_from(value).map_err(|_| "same-net obstacle cell is out of range".into())
}

fn distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

fn validate_grid(board: &BoardAdapter, config: &NativeGridConfig) -> Result<(), String> {
    if !board.units_per_mm.is_finite() || board.units_per_mm <= 0.0 {
        return Err("invalid BoardAdapter unit scale for same-net obstacles".into());
    }
    if config.grid_step <= 0 || config.via_size < 0 || config.via_drill < 0 || config.clearance < 0
    {
        return Err("invalid NativeGridConfig for same-net obstacles".into());
    }
    Ok(())
}
