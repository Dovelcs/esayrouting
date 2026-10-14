//! Source-faithful net-tie corridor calculation for the native obstacle map.
//!
//! The DSN-backed `BoardAdapter` does not declare KiCad footprint
//! `net_tie_pad_groups`; callers must pass those source groups explicitly.
//! An empty slice means the source board has no available net-tie groups and
//! returns no corridors. Ordinary pads are never inferred to form a net tie.

use crate::{awx::native_obstacles::NativeGridConfig, board_adapter::BoardAdapter};
use fr_engine::{board::ItemKey, ids::NetNo};
use fr_geom::FloatPoint;
use std::collections::{BTreeMap, BTreeSet};

const SAMPLE_MM: f64 = 0.01;

/// One explicitly declared member pad from a KiCad `net_tie_pad_groups` group.
/// `key` must map this source pad to the real board pin; sizes are in native
/// board units and come from the source pad's original size_x/size_y.
#[derive(Debug, Clone)]
pub struct NetTiePadMember {
    pub key: ItemKey,
    pub pad_number: String,
    pub net_id: NetNo,
    pub size_x: f64,
    pub size_y: f64,
}

/// The pad members of one explicitly declared footprint tie group.
#[derive(Debug, Clone, Default)]
pub struct NetTieGroup {
    /// Pad numbers as declared by the source footprint group, before mapping
    /// them to board pins. This lets an explicit but incomplete mapping fail
    /// clearly instead of silently narrowing the tie group.
    pub declared_pad_numbers: Vec<String>,
    pub members: Vec<NetTiePadMember>,
}

/// Per-net safe cells and the partner identities whose obstacle stamps may be
/// lifted only where they intersect these cells.
#[derive(Debug, Clone, Default)]
pub struct NetTieCorridor {
    pub cells: BTreeSet<(i32, i32)>,
    pub safe_cells: BTreeSet<(i32, i32)>,
    pub partner_pad_ids: BTreeSet<ItemKey>,
    pub partner_net_ids: BTreeSet<NetNo>,
}

#[derive(Clone)]
struct PadGeometry {
    key: ItemKey,
    pad_number: String,
    net_id: NetNo,
    center: [f64; 2],
    half_extent: f64,
    shapes: Vec<fr_geom::TileShape>,
}

impl PadGeometry {
    fn contains(&self, x: f64, y: f64) -> bool {
        let point = FloatPoint::new(x, y);
        self.shapes.iter().any(|shape| shape.contains_float(&point))
    }
}

/// Port of `py_router.obstacle_map._compute_net_tie_corridors`.
///
/// Only declared group membership creates ties. A declared member with no
/// matching on-board pin/shape is an error, since guessing a replacement pad
/// could exempt an unrelated route. Geometry tests use the board's actual
/// pad shapes; the source `size_x/size_y` metadata supplies its conservative
/// sampling extent.
pub fn compute_net_tie_corridors(
    board: &BoardAdapter,
    config: &NativeGridConfig,
    groups: &[NetTieGroup],
) -> Result<BTreeMap<NetNo, NetTieCorridor>, String> {
    let mut corridors = BTreeMap::<NetNo, NetTieCorridor>::new();
    if groups.is_empty() {
        return Ok(corridors);
    }

    let present: BTreeSet<_> = board.board.get_items().into_iter().collect();
    let mut geometry = BTreeMap::<ItemKey, PadGeometry>::new();
    for group in groups {
        let declared: BTreeSet<_> = group.declared_pad_numbers.iter().collect();
        let mapped: BTreeSet<_> = group
            .members
            .iter()
            .map(|member| &member.pad_number)
            .collect();
        if declared != mapped {
            return Err(
                "declared net-tie group has pad numbers without a complete ItemKey mapping".into(),
            );
        }
        for member in &group.members {
            if member.net_id <= 0 {
                continue;
            }
            if !present.contains(&member.key) {
                return Err(format!(
                    "declared net-tie pad {} has no matching on-board ItemKey",
                    member.pad_number
                ));
            }
            if !member.size_x.is_finite()
                || !member.size_y.is_finite()
                || member.size_x <= 0.0
                || member.size_y <= 0.0
            {
                return Err(format!(
                    "declared net-tie pad {} has invalid source dimensions",
                    member.pad_number
                ));
            }
            if let Some(existing) = geometry.get(&member.key) {
                if existing.net_id != member.net_id || existing.pad_number != member.pad_number {
                    return Err(format!(
                        "net-tie pad {} has conflicting source mappings",
                        member.pad_number
                    ));
                }
                continue;
            }
            let item = board.board.item(member.key);
            if !item.is_on_board() || item.as_pin().is_none() {
                return Err(format!(
                    "declared net-tie pad {} does not map to an on-board pin",
                    member.pad_number
                ));
            }
            if !item.net_numbers().contains(&member.net_id) {
                return Err(format!(
                    "declared net-tie pad {} net metadata disagrees with its board pin",
                    member.pad_number
                ));
            }
            let center = item.center(&board.board).to_float();
            let shapes: Vec<_> = board
                .board
                .item_tile_shapes(item)
                .into_iter()
                .flatten()
                .cloned()
                .collect();
            if shapes.is_empty() {
                return Err(format!(
                    "declared net-tie pad {} has no copper shape",
                    member.pad_number
                ));
            }
            geometry.insert(
                member.key,
                PadGeometry {
                    key: member.key,
                    pad_number: member.pad_number.clone(),
                    net_id: member.net_id,
                    center: [center.x, center.y],
                    half_extent: member.size_x / 2.0 + member.size_y / 2.0,
                    shapes,
                },
            );
        }
    }

    let fine = SAMPLE_MM * board.units_per_mm;
    if !fine.is_finite() || fine <= 0.0 {
        return Err("invalid board scale for net-tie sampling".into());
    }
    let grid_step = config.grid_step as f64;
    let clearance = config.clearance as f64;
    let half_width = config.track_width as f64 / 2.0;
    let reach = half_width + clearance + grid_step;

    for group in groups {
        let members: Vec<_> = group
            .members
            .iter()
            .filter_map(|m| geometry.get(&m.key))
            .collect();
        for own in &members {
            let partners: Vec<_> = members
                .iter()
                .copied()
                .filter(|partner| partner.net_id > 0 && partner.net_id != own.net_id)
                .collect();
            if partners.is_empty() {
                continue;
            }
            let entry = corridors.entry(own.net_id).or_default();
            for partner in partners {
                let extent = partner.half_extent;
                let (x0, x1) = (partner.center[0] - extent, partner.center[0] + extent);
                let (y0, y1) = (partner.center[1] - extent, partner.center[1] + extent);
                let bad = sample_partner_minus_own(partner, own, [x0, x1, y0, y1], fine);
                // The source skips a partner with no sampled interior. If the
                // partner is wholly inside the own pad, its minus-own sample
                // is empty but the corridor is still fully open.
                if !bad.has_partner_samples {
                    continue;
                }

                let (gx0, gy0) = (
                    grid_round(x0 - reach, config.grid_step),
                    grid_round(y0 - reach, config.grid_step),
                );
                let (gx1, gy1) = (
                    grid_round(x1 + reach, config.grid_step),
                    grid_round(y1 + reach, config.grid_step),
                );
                let threshold = (half_width + fine) * (half_width + fine);
                let mut pair_cells = Vec::new();
                for gx in gx0..=gx1 {
                    for gy in gy0..=gy1 {
                        let x = gx as f64 * grid_step;
                        let y = gy as f64 * grid_step;
                        if distance_squared_to_points([x, y], &bad.points) >= threshold
                            && !(bad.packed_region
                                && bad.region_keys.contains(&lattice_key(x, y, fine)))
                        {
                            pair_cells.push((gx, gy, x, y));
                        }
                    }
                }
                if pair_cells.is_empty() {
                    continue;
                }
                for (gx, gy, x, y) in pair_cells {
                    entry.cells.insert((gx, gy));
                    if own.contains(x, y) {
                        entry.safe_cells.insert((gx, gy));
                    }
                }
                entry.partner_pad_ids.insert(partner.key);
                entry.partner_net_ids.insert(partner.net_id);
            }
        }
    }
    corridors.retain(|_, corridor| !corridor.cells.is_empty());
    Ok(corridors)
}

struct BadRegion {
    has_partner_samples: bool,
    points: Vec<[f64; 2]>,
    packed_region: bool,
    region_keys: BTreeSet<(i64, i64)>,
}

fn sample_partner_minus_own(
    partner: &PadGeometry,
    own: &PadGeometry,
    bounds: [f64; 4],
    fine: f64,
) -> BadRegion {
    let xs = sample_axis(bounds[0], bounds[1], fine);
    let ys = sample_axis(bounds[2], bounds[3], fine);
    let mut points = Vec::new();
    let mut all_keys = BTreeSet::new();
    let mut has_partner_samples = false;
    for &x in &xs {
        for &y in &ys {
            if partner.contains(x, y) {
                has_partner_samples = true;
                if !own.contains(x, y) {
                    all_keys.insert(lattice_key(x, y, fine));
                    points.push([x, y]);
                }
            }
        }
    }
    if points.len() <= 256 {
        return BadRegion {
            has_partner_samples,
            points,
            packed_region: false,
            region_keys: BTreeSet::new(),
        };
    }
    // Source optimization: after identifying the interior on the 0.01 mm
    // lattice, distances need only be checked against its boundary; candidate
    // cell centers quantized into the interior are rejected separately.
    let boundary: Vec<_> = points
        .iter()
        .copied()
        .filter(|p| {
            let (x, y) = lattice_key(p[0], p[1], fine);
            !(all_keys.contains(&(x + 1, y))
                && all_keys.contains(&(x - 1, y))
                && all_keys.contains(&(x, y + 1))
                && all_keys.contains(&(x, y - 1)))
        })
        .collect();
    BadRegion {
        has_partner_samples,
        points: boundary,
        packed_region: true,
        region_keys: all_keys,
    }
}

fn sample_axis(start: f64, end: f64, step: f64) -> Vec<f64> {
    // numpy.arange(start, end + step, step): retain the terminal overshoot
    // rule instead of clipping to the pad bounds.
    let count = (((end + step) - start) / step).ceil().max(1.0) as usize;
    (0..count).map(|i| start + i as f64 * step).collect()
}

fn grid_round(value: f64, step: i32) -> i32 {
    (value / step as f64).round() as i32
}

fn lattice_key(x: f64, y: f64, fine: f64) -> (i64, i64) {
    (
        (x / fine).round_ties_even() as i64,
        (y / fine).round_ties_even() as i64,
    )
}

fn distance_squared_to_points(p: [f64; 2], points: &[[f64; 2]]) -> f64 {
    points
        .iter()
        .map(|q| {
            let dx = p[0] - q[0];
            let dy = p[1] - q[1];
            dx * dx + dy * dy
        })
        .fold(f64::INFINITY, f64::min)
}
