//! Exact scalar pad keep-out rasterizer.
//!
//! Faithful port of the source scalar reference implementation
//! (`routing_utils.dist_sq_to_rounded_rect` and `routing_utils.iter_pad_blocked_cells`),
//! driven from the *actual* copper `fr_geom::Shape` of a pad instead of the pad's
//! declared `size_x`/`size_y`/`shape`/`rect_rotation` metadata.
//!
//! Repeated footprints are memoized by their exact raster inputs. The cached value is
//! still produced by the source scalar algorithm below, whose pad sub-cell offset and
//! corner buffer must reproduce its cell set bit-for-bit.
//!
//! Recognized primitives:
//!   * a genuine `Shape::Circle` (the native circle primitive), and
//!   * an axis-aligned or oriented rectangle primitive, proven from the actual shape
//!     vertices (a quadrilateral whose opposite sides are equal/parallel and whose
//!     adjacent sides are perpendicular). This covers `IntBox`, a degenerate
//!     `IntOctagon` that is really a box, and a four-corner rectangular `PolygonShape`.
//!
//! Any other exact shape (a genuine IntOctagon, an arbitrary Simplex, a source custom
//! polygon, ...) is NOT a rectangle and returns `None`, so the original caller keeps
//! using its actual polygon raster (the source polygon branch, which applies no corner
//! buffer). No bounding-box substitute is produced here.
//!
//! Source geometry limitations (unchanged from the source):
//!   * the shape centre is taken from the actual primitive geometry, never assumed to
//!     equal the native pin/hole anchor (an offset copper shape is handled correctly);
//!   * a rectangle's half-dimensions and rotation are derived from its real vertices,
//!     so a rotated rectangle uses its true edge direction rather than a classification
//!     guess;
//!   * only shapes that are exactly a circle or a rectangle can be represented by the
//!     source rounded-rect model; everything else must fall back to the polygon raster.

use fr_geom::{FloatPoint, Shape};
use super::native_obstacles::{obstacle_detail_add, obstacle_detail_set};
use std::{
    cell::RefCell,
    collections::{HashMap, VecDeque},
    sync::Arc,
};

const CACHE_MAX_ENTRIES: usize = 8192;
const CACHE_MAX_BYTES: usize = 16 * 1024 * 1024;

#[derive(Clone, Hash, PartialEq, Eq)]
enum RasterKey {
    Pad {
        primitive: [u64; 6],
        grid_step: i32,
        units_per_mm: u64,
        margin: u64,
        corner_buffer: u64,
    },
    Polygon {
        corners: Vec<(u64, u64)>,
        grid_step: i32,
        margin: u64,
        units_per_mm: u64,
    },
}

impl RasterKey {
    fn estimated_bytes(&self) -> usize {
        std::mem::size_of::<Self>()
            + match self {
                Self::Pad { .. } => 0,
                Self::Polygon { corners, .. } => corners.len() * std::mem::size_of::<(u64, u64)>(),
            }
    }
}

#[derive(Default)]
struct RasterCache {
    cells: HashMap<RasterKey, Arc<[[i32; 2]]>>,
    insertion_order: VecDeque<(RasterKey, usize)>,
    bytes: usize,
}

impl RasterCache {
    fn get(&mut self, key: &RasterKey) -> Option<Arc<[[i32; 2]]>> {
        self.cells.get(key).cloned()
    }

    fn insert(&mut self, key: RasterKey, cells: Vec<[i32; 2]>) -> Arc<[[i32; 2]]> {
        let output_bytes = cells.len() * std::mem::size_of::<[i32; 2]>();
        let key_bytes = key.estimated_bytes();
        let size = output_bytes
            .saturating_add(key_bytes.saturating_mul(2))
            .saturating_add(128);
        let shared: Arc<[[i32; 2]]> = Arc::from(cells);
        if size > CACHE_MAX_BYTES {
            return shared;
        }
        while self.cells.len() >= CACHE_MAX_ENTRIES || self.bytes.saturating_add(size) > CACHE_MAX_BYTES {
            let Some((old_key, old_size)) = self.insertion_order.pop_front() else { break };
            if self.cells.remove(&old_key).is_some() {
                self.bytes = self.bytes.saturating_sub(old_size);
                obstacle_detail_add("source_raster_cache_evictions", 1);
            }
        }
        self.bytes = self.bytes.saturating_add(size);
        self.insertion_order.push_back((key.clone(), size));
        self.cells.insert(key, Arc::clone(&shared));
        shared
    }
}

thread_local! {
    static SOURCE_RASTER_CACHE: RefCell<RasterCache> = RefCell::new(RasterCache::default());
}

/// Clear source pad/polygon footprints at the start or end of one routing run.
/// The cache is bounded even when callers omit this lifecycle hook.
pub fn clear_source_raster_cache() {
    SOURCE_RASTER_CACHE.with(|cache| *cache.borrow_mut() = RasterCache::default());
}

fn cache_footprint(
    key: RasterKey,
    raster: impl FnOnce() -> Option<Vec<[i32; 2]>>,
) -> Option<Arc<[[i32; 2]]>> {
    SOURCE_RASTER_CACHE.with(|cache| {
        let mut cache = cache.borrow_mut();
        if let Some(cells) = cache.get(&key) {
            obstacle_detail_add("source_raster_cache_hits", 1);
            obstacle_detail_set("source_raster_cache_estimated_bytes", cache.bytes as u64);
            return Some(cells);
        }
        obstacle_detail_add("source_raster_cache_misses", 1);
        let Some(cells) = raster() else {
            obstacle_detail_set("source_raster_cache_estimated_bytes", cache.bytes as u64);
            return None;
        };
        let cells = cache.insert(key, cells);
        obstacle_detail_set("source_raster_cache_estimated_bytes", cache.bytes as u64);
        Some(cells)
    })
}

/// The one absolute-coordinate input is rounded to a nanometre scale in mm, matching
/// `iter_pad_blocked_cells`' `off_x = round(off_x, 9)`.
const OFFSET_DECIMALS: i32 = 9;

/// Round a value to `decimals` decimal places with ties-to-even, matching Python's
/// built-in `round` (which is what the source uses for the sub-cell offset).
fn round_ties_even_decimals(value: f64, decimals: i32) -> f64 {
    if !value.is_finite() {
        return value;
    }
    let scale = 10_f64.powi(decimals);
    (value * scale).round_ties_even() / scale
}

/// Port of `routing_utils.dist_sq_to_rounded_rect`.
///
/// Squared distance from a point (relative to the rectangle centre) to the rounded
/// rectangle with the given half-dimensions and corner radius (`0` for sharp corners).
fn dist_sq_to_rounded_rect(
    point_x: f64,
    point_y: f64,
    half_width: f64,
    half_height: f64,
    corner_radius: f64,
) -> f64 {
    let abs_x = point_x.abs();
    let abs_y = point_y.abs();

    if corner_radius > 0.0 {
        // Inner rectangle bounds (where the corner arcs begin).
        let inner_half_w = half_width - corner_radius;
        let inner_half_h = half_height - corner_radius;

        // Point in a corner region -> distance to the corner arc.
        if abs_x > inner_half_w && abs_y > inner_half_h {
            let dx = abs_x - inner_half_w;
            let dy = abs_y - inner_half_h;
            let dist_to_corner_center = (dx * dx + dy * dy).sqrt();
            let dist = dist_to_corner_center - corner_radius;
            return if dist > 0.0 { dist * dist } else { 0.0 };
        }
    }

    // Point along a flat edge -> rectangular distance.
    let closest_x = (-half_width).max(point_x.min(half_width));
    let closest_y = (-half_height).max(point_y.min(half_height));
    let dx = point_x - closest_x;
    let dy = point_y - closest_y;
    dx * dx + dy * dy
}

/// A recognized pad primitive, in mm, in the source rounded-rect parametrization.
pub(crate) struct PadRect {
    /// Real geometry centre in mm (NOT the pin/hole anchor).
    pub(crate) center_x_mm: f64,
    pub(crate) center_y_mm: f64,
    /// Half dimensions in mm along the local (rotated) axes.
    pub(crate) half_width_mm: f64,
    pub(crate) half_height_mm: f64,
    /// Corner radius in mm (`0` for sharp-cornered rectangles, `min(half)` for a circle).
    pub(crate) corner_radius_mm: f64,
    /// Residual tilt of the rectangle in the global frame (degrees), as the source
    /// `rect_rotation` convention: query points are mapped by `R(-rotation)`.
    pub(crate) rotation_deg: f64,
}

/// Recognizes a genuine circle or an exactly-quadrilateral rectangle from the actual
/// copper shape. Returns `None` for any other shape.
fn recognize_pad_primitive(shape: &Shape, units_per_mm: f64) -> Option<PadRect> {
    // Genuine native circle primitive.
    if let Some(circle) = shape.as_circle() {
        let center_x_mm = circle.center.x as f64 / units_per_mm;
        let center_y_mm = circle.center.y as f64 / units_per_mm;
        let radius_mm = circle.radius as f64 / units_per_mm;
        return Some(PadRect {
            center_x_mm,
            center_y_mm,
            half_width_mm: radius_mm,
            half_height_mm: radius_mm,
            // "Circle use cornerRadius = min half dims" -> a full circle.
            corner_radius_mm: radius_mm,
            rotation_deg: 0.0,
        });
    }

    // Otherwise prove a rectangle from the real vertices. A degenerate IntOctagon that
    // is really a box repeats corners; deduplicate cyclic collisions first.
    let corners = shape.corner_approx_arr();
    let pts = dedup_cyclic_corners(&corners);
    if pts.len() != 4 {
        return None;
    }
    if !is_rectangle(&pts) {
        return None;
    }

    // Centre from the actual geometry (parallelogram centroid).
    let center_x = (pts[0].x + pts[1].x + pts[2].x + pts[3].x) / 4.0;
    let center_y = (pts[0].y + pts[1].y + pts[2].y + pts[3].y) / 4.0;

    // Local frame: first edge direction and length, second edge orthogonal to it.
    let ex = pts[1].x - pts[0].x;
    let ey = pts[1].y - pts[0].y;
    let fx = pts[2].x - pts[1].x;
    let fy = pts[2].y - pts[1].y;
    let width = (ex * ex + ey * ey).sqrt();
    let height = (fx * fx + fy * fy).sqrt();

    let rotation_deg = ey.atan2(ex).to_degrees();

    Some(PadRect {
        center_x_mm: center_x / units_per_mm,
        center_y_mm: center_y / units_per_mm,
        half_width_mm: (width / 2.0) / units_per_mm,
        half_height_mm: (height / 2.0) / units_per_mm,
        corner_radius_mm: 0.0,
        rotation_deg,
    })
}

/// Return the measured rectangle geometry only when the actual shape vertices
/// prove it is a rectangle. Circles remain on their dedicated representation.
pub(crate) fn source_rectangle(shape: &Shape, units_per_mm: f64) -> Option<PadRect> {
    if shape.as_circle().is_some() {
        return None;
    }
    let rect = recognize_pad_primitive(shape, units_per_mm)?;
    (rect.corner_radius_mm == 0.0).then_some(rect)
}

/// Removes consecutive duplicate corners, including the wrap-around duplicate.
fn dedup_cyclic_corners(corners: &[FloatPoint]) -> Vec<FloatPoint> {
    let mut out: Vec<FloatPoint> = Vec::with_capacity(corners.len());
    for &c in corners {
        let duplicate = match out.last() {
            Some(last) => last.x == c.x && last.y == c.y,
            None => false,
        };
        if !duplicate {
            out.push(c);
        }
    }
    if out.len() > 1 {
        let first = out[0];
        let last = out[out.len() - 1];
        if first.x == last.x && first.y == last.y {
            out.pop();
        }
    }
    out
}

/// True if the four cyclic points form a genuine rectangle (opposite edges equal and
/// antiparallel, adjacent edges perpendicular). Uses a tight relative tolerance so only
/// exact rectangle primitives are accepted and arbitrary polygons are rejected.
fn is_rectangle(p: &[FloatPoint]) -> bool {
    let tol = 1e-6;

    let e0 = (p[1].x - p[0].x, p[1].y - p[0].y);
    let e1 = (p[2].x - p[1].x, p[2].y - p[1].y);
    let e2 = (p[3].x - p[2].x, p[3].y - p[2].y);
    let e3 = (p[0].x - p[3].x, p[0].y - p[3].y);

    let len0 = (e0.0 * e0.0 + e0.1 * e0.1).sqrt();
    let len1 = (e1.0 * e1.0 + e1.1 * e1.1).sqrt();
    if len0 <= 0.0 || len1 <= 0.0 {
        return false;
    }

    // Opposite edges must be antiparallel and equal (a parallelogram).
    if (e2.0 + e0.0).abs() > tol * len0 || (e2.1 + e0.1).abs() > tol * len0 {
        return false;
    }
    if (e3.0 + e1.0).abs() > tol * len1 || (e3.1 + e1.1).abs() > tol * len1 {
        return false;
    }

    // Adjacent edges must be perpendicular (a right angle).
    let dot = e0.0 * e1.0 + e0.1 * e1.1;
    dot.abs() <= tol * len0 * len1
}

/// Generate the grid cells blocked by a pad whose real copper geometry is `shape`.
///
/// Exact port of `routing_utils.iter_pad_blocked_cells`, in f64 mm:
///   * `margin_board` is the blocking margin (track_width/2 + clearance + extra) in
///     board units, exactly as the caller already resolves it;
///   * `corner_buffer_fraction` is the caller's corner-buffer multiplier
///     (`0.5` regular, `0.75` for a diff-pair caller's extra clearance > 0); the actual
///     buffer is `grid_step_mm * fraction`;
///   * the grid pitch comes from the existing `grid_step_board / units_per_mm` scale
///     (this function does not alter the grid policy);
///   * the pad sub-cell offset is rounded to 9 decimals (ties-to-even) and the strict
///     `dist_sq < effective_margin_sq` test is applied.
///
/// Returns `None` when the exact copper shape is neither a genuine circle nor an exact
/// rectangle primitive, so the caller keeps its actual polygon raster (which applies no
/// corner buffer). No bounding-box substitute is produced.
pub fn source_pad_cells(
    shape: &Shape,
    grid_step_board: i32,
    units_per_mm: f64,
    margin_board: f64,
    corner_buffer_fraction: f64,
) -> Option<Vec<[i32; 2]>> {
    source_pad_cells_shared(
        shape,
        grid_step_board,
        units_per_mm,
        margin_board,
        corner_buffer_fraction,
    )
    .map(|cells| cells.to_vec())
}

/// Cached form used by map builders that can stamp an immutable shared footprint.
pub(crate) fn source_pad_cells_shared(
    shape: &Shape,
    grid_step_board: i32,
    units_per_mm: f64,
    margin_board: f64,
    corner_buffer_fraction: f64,
) -> Option<Arc<[[i32; 2]]>> {
    if grid_step_board <= 0
        || !units_per_mm.is_finite()
        || units_per_mm <= 0.0
        || !margin_board.is_finite()
        || margin_board < 0.0
        || !corner_buffer_fraction.is_finite()
        || corner_buffer_fraction < 0.0
    {
        return None;
    }
    let pad = recognize_pad_primitive(shape, units_per_mm)?;
    obstacle_detail_add(
        if shape.as_circle().is_some() {
            "source_pad_raster_circle_shapes"
        } else {
            "source_pad_raster_rectangle_shapes"
        },
        1,
    );
    let key = RasterKey::Pad {
        primitive: [
            pad.center_x_mm.to_bits(),
            pad.center_y_mm.to_bits(),
            pad.half_width_mm.to_bits(),
            pad.half_height_mm.to_bits(),
            pad.corner_radius_mm.to_bits(),
            pad.rotation_deg.to_bits(),
        ],
        grid_step: grid_step_board,
        units_per_mm: units_per_mm.to_bits(),
        margin: margin_board.to_bits(),
        corner_buffer: corner_buffer_fraction.to_bits(),
    };
    cache_footprint(key, || {
        source_pad_cells_uncached(
            shape,
            grid_step_board,
            units_per_mm,
            margin_board,
            corner_buffer_fraction,
        )
    })
}

fn source_pad_cells_uncached(
    shape: &Shape,
    grid_step_board: i32,
    units_per_mm: f64,
    margin_board: f64,
    corner_buffer_fraction: f64,
) -> Option<Vec<[i32; 2]>> {
    if grid_step_board <= 0
        || !units_per_mm.is_finite()
        || units_per_mm <= 0.0
        || !margin_board.is_finite()
        || margin_board < 0.0
        || !corner_buffer_fraction.is_finite()
        || corner_buffer_fraction < 0.0
    {
        return None;
    }

    let pad = recognize_pad_primitive(shape, units_per_mm)?;
    let grid_step = grid_step_board as f64 / units_per_mm;
    if !grid_step.is_finite() || grid_step <= 0.0 {
        return None;
    }
    let margin = margin_board / units_per_mm;
    let corner_buffer = grid_step * corner_buffer_fraction;

    let half_width = pad.half_width_mm;
    let half_height = pad.half_height_mm;
    let corner_radius = pad.corner_radius_mm;
    if !half_width.is_finite()
        || !half_height.is_finite()
        || half_width < 0.0
        || half_height < 0.0
        || !corner_radius.is_finite()
        || corner_radius < 0.0
        || !pad.rotation_deg.is_finite()
        || !pad.center_x_mm.is_finite()
        || !pad.center_y_mm.is_finite()
    {
        return None;
    }

    // Quantize the real centre to the grid (source GridCoord.to_grid, ties-to-even) and
    // measure the sub-cell offset from the REAL centre (issue #70).
    let gx = (pad.center_x_mm / grid_step).round_ties_even();
    let gy = (pad.center_y_mm / grid_step).round_ties_even();
    if !gx.is_finite() || !gy.is_finite() {
        return None;
    }
    let pad_gx = gx as i64;
    let pad_gy = gy as i64;

    let off_x = round_ties_even_decimals(
        pad.center_x_mm - pad_gx as f64 * grid_step,
        OFFSET_DECIMALS,
    );
    let off_y = round_ties_even_decimals(
        pad.center_y_mm - pad_gy as f64 * grid_step,
        OFFSET_DECIMALS,
    );

    let rotation_deg = pad.rotation_deg;
    let rotated = rotation_deg.abs() > 1e-9 && (rotation_deg.abs() - 180.0).abs() > 1e-9;
    let (gext_x, gext_y, rc, rs) = if rotated {
        let rrad = rotation_deg.to_radians();
        let rc = rrad.cos();
        let rs = rrad.sin();
        (
            (half_width * rc).abs() + (half_height * rs).abs(),
            (half_width * rs).abs() + (half_height * rc).abs(),
            rc,
            rs,
        )
    } else {
        (half_width, half_height, 0.0, 0.0)
    };

    // +1 cell so the bbox still covers the region once shifted by the sub-cell offset.
    let expand_x_f = ((gext_x + margin + corner_buffer) / grid_step).ceil() + 1.0;
    let expand_y_f = ((gext_y + margin + corner_buffer) / grid_step).ceil() + 1.0;
    if !expand_x_f.is_finite() || !expand_y_f.is_finite() {
        return None;
    }
    let expand_x = expand_x_f as i64;
    let expand_y = expand_y_f as i64;

    let margin_sq = margin * margin;
    let buffered_margin_sq = (margin + corner_buffer) * (margin + corner_buffer);

    // Inner rectangle bounds (where corners start). Rectangular pads get a threshold
    // region; rounded pads do not.
    let corner_threshold = if corner_radius == 0.0 {
        grid_step / 2.0
    } else {
        0.0
    };
    let inner_half_w = half_width - corner_radius - corner_threshold;
    let inner_half_h = half_height - corner_radius - corner_threshold;

    let mut cells: Vec<[i32; 2]> = Vec::new();
    let mut candidates = 0u64;
    for ex in -expand_x..=expand_x {
        for ey in -expand_y..=expand_y {
            candidates += 1;
            // Cell centre relative to the REAL pad centre (issue #70).
            let mut cell_x = ex as f64 * grid_step - off_x;
            let mut cell_y = ey as f64 * grid_step - off_y;
            if rotated {
                // Rotate the global offset into the pad's local frame: R(-rotation).
                let (nx, ny) = (
                    cell_x * rc + cell_y * rs,
                    -cell_x * rs + cell_y * rc,
                );
                cell_x = nx;
                cell_y = ny;
            }
            let abs_x = cell_x.abs();
            let abs_y = cell_y.abs();

            // Buffered margin in corner/diagonal regions where grid discretization can
            // bring a track closer than expected (applies to all pad shapes).
            let in_corner = abs_x > inner_half_w && abs_y > inner_half_h;
            let effective_margin_sq = if in_corner {
                buffered_margin_sq
            } else {
                margin_sq
            };

            let dist_sq = dist_sq_to_rounded_rect(cell_x, cell_y, half_width, half_height, corner_radius);
            if dist_sq < effective_margin_sq {
                let gx_cell = pad_gx + ex;
                let gy_cell = pad_gy + ey;
                if let (Ok(gx_cell), Ok(gy_cell)) = (i32::try_from(gx_cell), i32::try_from(gy_cell)) {
                    cells.push([gx_cell, gy_cell]);
                }
            }
        }
    }
    obstacle_detail_add("source_pad_raster_loop_candidates", candidates);
    obstacle_detail_add("source_pad_raster_cells_generated", cells.len() as u64);

    Some(cells)
}

/// Port of the source custom-polygon raster predicate used by
/// `obstacle_map._emit`: cell centres inside the polygon, or at most
/// `margin - 1e-6 mm` from its nearest edge. Polygon vertices and `margin_board`
/// use native board units; `units_per_mm` converts the source tie epsilon.
pub fn source_polygon_cells(
    corners: &[FloatPoint],
    grid_step_board: i32,
    margin_board: f64,
    units_per_mm: f64,
) -> Vec<[i32; 2]> {
    source_polygon_cells_shared(corners, grid_step_board, margin_board, units_per_mm).to_vec()
}

/// Cached form used by map builders that can stamp an immutable shared footprint.
pub(crate) fn source_polygon_cells_shared(
    corners: &[FloatPoint],
    grid_step_board: i32,
    margin_board: f64,
    units_per_mm: f64,
) -> Arc<[[i32; 2]]> {
    if corners.len() < 3
        || grid_step_board <= 0
        || !margin_board.is_finite()
        || margin_board < 0.0
        || !units_per_mm.is_finite()
        || units_per_mm <= 0.0
        || corners.iter().any(|p| !p.x.is_finite() || !p.y.is_finite())
    {
        return Arc::<[[i32; 2]]>::from(Vec::new());
    }
    let key = RasterKey::Polygon {
        corners: corners.iter().map(|p| (p.x.to_bits(), p.y.to_bits())).collect(),
        grid_step: grid_step_board,
        margin: margin_board.to_bits(),
        units_per_mm: units_per_mm.to_bits(),
    };
    cache_footprint(key, || {
        Some(source_polygon_cells_uncached(corners, grid_step_board, margin_board, units_per_mm))
    })
    .unwrap_or_else(|| Arc::<[[i32; 2]]>::from(Vec::new()))
}

fn source_polygon_cells_uncached(
    corners: &[FloatPoint],
    grid_step_board: i32,
    margin_board: f64,
    units_per_mm: f64,
) -> Vec<[i32; 2]> {
    if corners.len() < 3
        || grid_step_board <= 0
        || !margin_board.is_finite()
        || margin_board < 0.0
        || !units_per_mm.is_finite()
        || units_per_mm <= 0.0
        || corners.iter().any(|p| !p.x.is_finite() || !p.y.is_finite())
    {
        return Vec::new();
    }
    let step = grid_step_board as f64;
    let min_x = corners.iter().map(|p| p.x).fold(f64::INFINITY, f64::min) - margin_board;
    let max_x = corners.iter().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max) + margin_board;
    let min_y = corners.iter().map(|p| p.y).fold(f64::INFINITY, f64::min) - margin_board;
    let max_y = corners.iter().map(|p| p.y).fold(f64::NEG_INFINITY, f64::max) + margin_board;
    let grid = |v: f64| (v / step).round_ties_even() as i32;
    let gx0 = grid(min_x);
    let gx1 = grid(max_x);
    let gy0 = grid(min_y);
    let gy1 = grid(max_y);
    let threshold = margin_board - 1e-6 * units_per_mm;
    let threshold_sq = threshold * threshold;
    let mut cells = Vec::new();
    let mut candidates = 0u64;
    let mut edge_tests = 0u64;
    for gx in gx0.min(gx1)..=gx0.max(gx1) {
        for gy in gy0.min(gy1)..=gy0.max(gy1) {
            candidates += 1;
            let x = gx as f64 * step;
            let y = gy as f64 * step;
            let mut inside = false;
            let mut edge_distance_sq = f64::INFINITY;
            for i in 0..corners.len() {
                edge_tests += 1;
                let a = corners[i];
                let b = corners[(i + 1) % corners.len()];
                if (a.y > y) != (b.y > y)
                    && x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x
                {
                    inside = !inside;
                }
                let dx = b.x - a.x;
                let dy = b.y - a.y;
                let length_sq = dx * dx + dy * dy;
                let t = if length_sq > 0.0 {
                    (((x - a.x) * dx + (y - a.y) * dy) / length_sq).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let ex = x - (a.x + t * dx);
                let ey = y - (a.y + t * dy);
                edge_distance_sq = edge_distance_sq.min(ex * ex + ey * ey);
            }
            if inside || (threshold >= 0.0 && edge_distance_sq <= threshold_sq) {
                cells.push([gx, gy]);
            }
        }
    }
    obstacle_detail_add("source_polygon_raster_loop_candidates", candidates);
    obstacle_detail_add("source_polygon_raster_edge_tests", edge_tests);
    obstacle_detail_add("source_polygon_raster_cells_generated", cells.len() as u64);
    cells
}
