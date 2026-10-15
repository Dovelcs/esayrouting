//! Exact-shape copper gap checks for candidates returned by the board search tree.
//!
//! Search-tree tiles are broad-phase candidates. This module measures the actual pad/via shape
//! and the actual trace centerline with circular end caps, so an octagonal search approximation
//! of a circle is never treated as the copper itself.

use fr_engine::board::{BasicBoard, ItemKey};
use fr_geom::{FloatPoint, Shape};

const GEOM_EPSILON: f64 = 1.0e-9;

#[derive(Clone, Copy, Debug)]
struct Segment {
    a: FloatPoint,
    b: FloatPoint,
}

#[derive(Clone, Debug)]
struct Polygon {
    /// Original bounded outline vertices; a repeated closing vertex is omitted.
    vertices: Vec<FloatPoint>,
}

#[derive(Clone, Debug)]
enum Copper {
    Circle {
        center: FloatPoint,
        radius: f64,
    },
    Polygon(Polygon),
    /// A trace is the union of capsules around its consecutive centerline segments.
    Trace {
        segments: Vec<Segment>,
        radius: f64,
    },
}

/// Return the Euclidean copper-edge gap in native board units.
///
/// `None` means that either item has no copper on `layer`, or its geometry cannot be represented
/// by the supported exact circle, polygon, or trace-capsule forms. Rule lookup and same-net
/// exceptions are deliberately outside this helper.
pub(crate) fn actual_copper_gap(
    board: &BasicBoard,
    a: ItemKey,
    b: ItemKey,
    layer: i32,
) -> Option<f64> {
    let a = copper_for_item(board, a, layer)?;
    let b = copper_for_item(board, b, layer)?;
    let gap = copper_gap(&a, &b)?;
    gap.is_finite().then_some(gap.max(0.0))
}

fn copper_for_item(board: &BasicBoard, key: ItemKey, layer: i32) -> Option<Copper> {
    let item = board.item(key);
    if !item.is_on_board() {
        return None;
    }
    if let Some(trace) = item.as_trace() {
        if trace.layer() != layer || trace.half_width() < 0 {
            return None;
        }
        let corners = trace.polyline().corners();
        if corners.is_empty() {
            return None;
        }
        let points: Vec<_> = corners.iter().map(|point| point.to_float()).collect();
        if points.iter().any(|point| !finite_point(*point)) {
            return None;
        }
        let segments: Vec<_> = points
            .windows(2)
            .map(|pair| Segment {
                a: pair[0],
                b: pair[1],
            })
            .collect();
        if segments.is_empty() {
            // A degenerate one-point trace still has the circular cap implied by its width.
            return Some(Copper::Circle {
                center: points[0],
                radius: trace.half_width() as f64,
            });
        }
        return Some(Copper::Trace {
            segments,
            radius: trace.half_width() as f64,
        });
    }

    if !item.is_drill_item() {
        // Keepouts, outlines, and other non-pad item kinds do not have a copper outline contract
        // exposed by the item API. Do not reinterpret their search-tree tiles as exact copper.
        return None;
    }
    let shape = item.drill_shape_on_layer(board, layer)?;
    copper_for_shape(&shape)
}

fn copper_for_shape(shape: &Shape) -> Option<Copper> {
    if shape.is_empty() || !shape.is_bounded() || shape.dimension() != 2 {
        return None;
    }
    if let Some(circle) = shape.as_circle() {
        let center = FloatPoint::new(circle.center.x as f64, circle.center.y as f64);
        let radius = circle.radius as f64;
        if !finite_point(center) || !radius.is_finite() || radius < 0.0 {
            return None;
        }
        return Some(Copper::Circle { center, radius });
    }

    // TileShape and PolygonShape corners are their filled shape's actual vertices. In particular,
    // this does not call `tile_shape` or use a circle's bounding octagon.
    let mut vertices = shape.corner_approx_arr();
    if vertices.iter().any(|point| !finite_point(*point)) {
        return None;
    }
    if vertices.len() > 1 && points_equal(*vertices.first()?, *vertices.last()?) {
        vertices.pop();
    }
    if vertices.len() < 3 || polygon_area_twice(&vertices).abs() <= GEOM_EPSILON {
        return None;
    }
    Some(Copper::Polygon(Polygon { vertices }))
}

fn copper_gap(a: &Copper, b: &Copper) -> Option<f64> {
    match (a, b) {
        (
            Copper::Circle {
                center: a_center,
                radius: a_radius,
            },
            Copper::Circle {
                center: b_center,
                radius: b_radius,
            },
        ) => {
            // Exact circle-to-circle gap: center distance minus the two true radii.
            Some((distance(*a_center, *b_center) - a_radius - b_radius).max(0.0))
        }
        (Copper::Circle { center, radius }, Copper::Polygon(poly))
        | (Copper::Polygon(poly), Copper::Circle { center, radius }) => {
            Some(circle_polygon_gap(*center, *radius, poly))
        }
        (Copper::Polygon(a), Copper::Polygon(b)) => Some(polygon_polygon_gap(a, b)),
        (
            Copper::Trace { segments, radius },
            Copper::Circle {
                center,
                radius: other_radius,
            },
        )
        | (
            Copper::Circle {
                center,
                radius: other_radius,
            },
            Copper::Trace { segments, radius },
        ) => Some(trace_circle_gap(segments, *radius, *center, *other_radius)),
        (
            Copper::Trace {
                segments: a_segments,
                radius: a_radius,
            },
            Copper::Trace {
                segments: b_segments,
                radius: b_radius,
            },
        ) => Some(trace_trace_gap(
            a_segments, *a_radius, b_segments, *b_radius,
        )),
        (Copper::Trace { segments, radius }, Copper::Polygon(poly))
        | (Copper::Polygon(poly), Copper::Trace { segments, radius }) => {
            Some(trace_polygon_gap(segments, *radius, poly))
        }
    }
}

fn circle_polygon_gap(center: FloatPoint, radius: f64, polygon: &Polygon) -> f64 {
    if point_in_polygon(center, polygon) {
        return 0.0;
    }
    // The circle-to-filled-polygon gap is distance(center, polygon boundary) minus its radius;
    // containment is checked first because a center inside the filled region means overlap.
    let boundary_gap = polygon_edges(polygon)
        .map(|edge| point_segment_distance(center, edge))
        .fold(f64::INFINITY, f64::min);
    (boundary_gap - radius).max(0.0)
}

fn trace_circle_gap(
    segments: &[Segment],
    trace_radius: f64,
    center: FloatPoint,
    circle_radius: f64,
) -> f64 {
    // A trace is the Minkowski sum of each centerline segment and a disk of `trace_radius`.
    // Subtract both radii from centerline-to-center distance to get capsule-to-circle gap.
    let centerline_gap = segments
        .iter()
        .map(|segment| point_segment_distance(center, *segment))
        .fold(f64::INFINITY, f64::min);
    (centerline_gap - trace_radius - circle_radius).max(0.0)
}

fn trace_trace_gap(
    a_segments: &[Segment],
    a_radius: f64,
    b_segments: &[Segment],
    b_radius: f64,
) -> f64 {
    // Two trace capsules are disjoint exactly when their centerline segment distance exceeds the
    // sum of their radii; subtract that sum and clamp overlap to zero.
    let centerline_gap = a_segments
        .iter()
        .flat_map(|a| b_segments.iter().map(move |b| segment_distance(*a, *b)))
        .fold(f64::INFINITY, f64::min);
    (centerline_gap - a_radius - b_radius).max(0.0)
}

fn trace_polygon_gap(segments: &[Segment], radius: f64, polygon: &Polygon) -> f64 {
    let edges: Vec<_> = polygon_edges(polygon).collect();
    for segment in segments {
        if point_in_polygon(segment.a, polygon)
            || point_in_polygon(segment.b, polygon)
            || edges.iter().any(|edge| segments_intersect(*segment, *edge))
        {
            return 0.0;
        }
    }
    // After testing centerline containment/crossing, the nearest capsule-to-region gap is the
    // minimum centerline-to-boundary distance minus the trace radius.
    let centerline_gap = segments
        .iter()
        .flat_map(|trace| {
            edges
                .iter()
                .map(move |edge| segment_distance(*trace, *edge))
        })
        .fold(f64::INFINITY, f64::min);
    (centerline_gap - radius).max(0.0)
}

fn polygon_polygon_gap(a: &Polygon, b: &Polygon) -> f64 {
    if a.vertices
        .iter()
        .copied()
        .any(|point| point_in_polygon(point, b))
        || b.vertices
            .iter()
            .copied()
            .any(|point| point_in_polygon(point, a))
    {
        return 0.0;
    }
    // With filled-region overlap excluded above, minimum edge-to-edge segment distance is the
    // Euclidean distance between the two polygon interiors.
    polygon_edges(a)
        .flat_map(|a_edge| polygon_edges(b).map(move |b_edge| segment_distance(a_edge, b_edge)))
        .fold(f64::INFINITY, f64::min)
}

fn polygon_edges(polygon: &Polygon) -> impl Iterator<Item = Segment> + '_ {
    polygon
        .vertices
        .iter()
        .enumerate()
        .map(|(index, &a)| Segment {
            a,
            b: polygon.vertices[(index + 1) % polygon.vertices.len()],
        })
}

fn point_in_polygon(point: FloatPoint, polygon: &Polygon) -> bool {
    let mut inside = false;
    for edge in polygon_edges(polygon) {
        if point_segment_distance(point, edge) <= GEOM_EPSILON {
            return true;
        }
        let crosses_ray = (edge.a.y > point.y) != (edge.b.y > point.y);
        if crosses_ray {
            let crossing_x =
                edge.a.x + (point.y - edge.a.y) * (edge.b.x - edge.a.x) / (edge.b.y - edge.a.y);
            if crossing_x > point.x {
                inside = !inside;
            }
        }
    }
    inside
}

fn point_segment_distance(point: FloatPoint, segment: Segment) -> f64 {
    let dx = segment.b.x - segment.a.x;
    let dy = segment.b.y - segment.a.y;
    let length_squared = dx * dx + dy * dy;
    if !length_squared.is_finite() {
        return f64::INFINITY;
    }
    if length_squared <= GEOM_EPSILON * GEOM_EPSILON {
        return distance(point, segment.a);
    }
    // Orthogonal projection onto AB, clamped to [0, 1], gives the nearest point on a segment.
    let projection = (((point.x - segment.a.x) * dx + (point.y - segment.a.y) * dy)
        / length_squared)
        .clamp(0.0, 1.0);
    let nearest = FloatPoint::new(segment.a.x + projection * dx, segment.a.y + projection * dy);
    distance(point, nearest)
}

fn segment_distance(a: Segment, b: Segment) -> f64 {
    if segments_intersect(a, b) {
        return 0.0;
    }
    point_segment_distance(a.a, b)
        .min(point_segment_distance(a.b, b))
        .min(point_segment_distance(b.a, a))
        .min(point_segment_distance(b.b, a))
}

fn segments_intersect(a: Segment, b: Segment) -> bool {
    let o1 = orient(a.a, a.b, b.a);
    let o2 = orient(a.a, a.b, b.b);
    let o3 = orient(b.a, b.b, a.a);
    let o4 = orient(b.a, b.b, a.b);
    if ((o1 > 0.0 && o2 < 0.0) || (o1 < 0.0 && o2 > 0.0))
        && ((o3 > 0.0 && o4 < 0.0) || (o3 < 0.0 && o4 > 0.0))
    {
        return true;
    }
    // Endpoint-to-segment projection also covers collinear overlap and tolerant endpoint contact.
    point_segment_distance(b.a, a) <= GEOM_EPSILON
        || point_segment_distance(b.b, a) <= GEOM_EPSILON
        || point_segment_distance(a.a, b) <= GEOM_EPSILON
        || point_segment_distance(a.b, b) <= GEOM_EPSILON
}

fn orient(a: FloatPoint, b: FloatPoint, c: FloatPoint) -> f64 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

fn polygon_area_twice(vertices: &[FloatPoint]) -> f64 {
    (0..vertices.len())
        .map(|index| {
            let a = vertices[index];
            let b = vertices[(index + 1) % vertices.len()];
            a.x * b.y - b.x * a.y
        })
        .sum()
}

fn distance(a: FloatPoint, b: FloatPoint) -> f64 {
    (a.x - b.x).hypot(a.y - b.y)
}

fn points_equal(a: FloatPoint, b: FloatPoint) -> bool {
    (a.x - b.x).abs() <= GEOM_EPSILON && (a.y - b.y).abs() <= GEOM_EPSILON
}

fn finite_point(point: FloatPoint) -> bool {
    point.x.is_finite() && point.y.is_finite()
}

#[cfg(test)]
mod tests {
    use super::{copper_gap, Copper, FloatPoint, Polygon, Segment};

    fn point(x: f64, y: f64) -> FloatPoint {
        FloatPoint::new(x, y)
    }

    fn close(actual: f64, expected: f64) {
        assert!((actual - expected).abs() < 1.0e-9, "{actual} != {expected}");
    }

    #[test]
    fn circle_circle_separated_touching_and_overlapping() {
        let a = Copper::Circle {
            center: point(0.0, 0.0),
            radius: 2.0,
        };
        let separated = Copper::Circle {
            center: point(10.0, 0.0),
            radius: 3.0,
        };
        let touching = Copper::Circle {
            center: point(5.0, 0.0),
            radius: 3.0,
        };
        let overlapping = Copper::Circle {
            center: point(4.0, 0.0),
            radius: 3.0,
        };
        close(copper_gap(&a, &separated).unwrap(), 5.0);
        close(copper_gap(&a, &touching).unwrap(), 0.0);
        close(copper_gap(&a, &overlapping).unwrap(), 0.0);
    }

    #[test]
    fn trace_capsule_uses_round_end_cap_against_circle_pad() {
        let trace = Copper::Trace {
            segments: vec![Segment {
                a: point(0.0, 0.0),
                b: point(10.0, 0.0),
            }],
            radius: 1.0,
        };
        let touching_pad = Copper::Circle {
            center: point(12.0, 0.0),
            radius: 1.0,
        };
        let separated_pad = Copper::Circle {
            center: point(13.0, 0.0),
            radius: 1.0,
        };
        close(copper_gap(&trace, &touching_pad).unwrap(), 0.0);
        close(copper_gap(&trace, &separated_pad).unwrap(), 1.0);
    }

    #[test]
    fn polygon_uses_filled_interior_and_original_edges() {
        let square = Copper::Polygon(Polygon {
            vertices: vec![
                point(0.0, 0.0),
                point(10.0, 0.0),
                point(10.0, 10.0),
                point(0.0, 10.0),
            ],
        });
        let inner = Copper::Circle {
            center: point(5.0, 5.0),
            radius: 1.0,
        };
        let outside = Copper::Circle {
            center: point(13.0, 5.0),
            radius: 1.0,
        };
        close(copper_gap(&square, &inner).unwrap(), 0.0);
        close(copper_gap(&square, &outside).unwrap(), 2.0);
    }
}
