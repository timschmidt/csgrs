//! Geometry distance helpers shared by board-level checks.
//!
//! `csgrs` handles boolean geometry. These helpers fill the gap for clearance
//! fallbacks where two shapes are close but do not intersect.

use geo::{Coord, LineString, MultiPolygon, Polygon};

pub(super) fn polygon_boundary_distance(
    left: &MultiPolygon<f64>,
    right: &MultiPolygon<f64>,
) -> f64 {
    let mut minimum = f64::INFINITY;
    for left_polygon in &left.0 {
        for right_polygon in &right.0 {
            minimum = minimum.min(single_polygon_boundary_distance(
                left_polygon,
                right_polygon,
            ));
        }
    }
    minimum
}

fn single_polygon_boundary_distance(left: &Polygon<f64>, right: &Polygon<f64>) -> f64 {
    let mut minimum = ring_boundary_distance(left.exterior(), right.exterior());

    for left_hole in left.interiors() {
        minimum = minimum.min(ring_boundary_distance(left_hole, right.exterior()));
        for right_hole in right.interiors() {
            minimum = minimum.min(ring_boundary_distance(left_hole, right_hole));
        }
    }

    for right_hole in right.interiors() {
        minimum = minimum.min(ring_boundary_distance(left.exterior(), right_hole));
    }

    minimum
}

fn ring_boundary_distance(left: &LineString<f64>, right: &LineString<f64>) -> f64 {
    let mut minimum = f64::INFINITY;
    for left_segment in left.0.windows(2) {
        for right_segment in right.0.windows(2) {
            minimum = minimum.min(segment_distance(
                left_segment[0],
                left_segment[1],
                right_segment[0],
                right_segment[1],
            ));
        }
    }
    minimum
}

fn segment_distance(
    a_start: Coord<f64>,
    a_end: Coord<f64>,
    b_start: Coord<f64>,
    b_end: Coord<f64>,
) -> f64 {
    if segments_intersect(a_start, a_end, b_start, b_end) {
        return 0.0;
    }

    point_segment_distance(a_start, b_start, b_end)
        .min(point_segment_distance(a_end, b_start, b_end))
        .min(point_segment_distance(b_start, a_start, a_end))
        .min(point_segment_distance(b_end, a_start, a_end))
}

fn point_segment_distance(point: Coord<f64>, start: Coord<f64>, end: Coord<f64>) -> f64 {
    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let length_squared = dx * dx + dy * dy;
    if length_squared == 0.0 {
        return distance([point.x, point.y], [start.x, start.y]);
    }

    let t =
        (((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared).clamp(0.0, 1.0);
    distance([point.x, point.y], [start.x + t * dx, start.y + t * dy])
}

fn segments_intersect(
    a_start: Coord<f64>,
    a_end: Coord<f64>,
    b_start: Coord<f64>,
    b_end: Coord<f64>,
) -> bool {
    let d1 = orientation(a_start, a_end, b_start);
    let d2 = orientation(a_start, a_end, b_end);
    let d3 = orientation(b_start, b_end, a_start);
    let d4 = orientation(b_start, b_end, a_end);

    if d1 == 0.0 && point_on_segment(b_start, a_start, a_end) {
        return true;
    }
    if d2 == 0.0 && point_on_segment(b_end, a_start, a_end) {
        return true;
    }
    if d3 == 0.0 && point_on_segment(a_start, b_start, b_end) {
        return true;
    }
    if d4 == 0.0 && point_on_segment(a_end, b_start, b_end) {
        return true;
    }

    (d1 > 0.0) != (d2 > 0.0) && (d3 > 0.0) != (d4 > 0.0)
}

fn orientation(a: Coord<f64>, b: Coord<f64>, c: Coord<f64>) -> f64 {
    let cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if cross.abs() < 1.0e-12 { 0.0 } else { cross }
}

fn point_on_segment(point: Coord<f64>, start: Coord<f64>, end: Coord<f64>) -> bool {
    point.x >= start.x.min(end.x) - 1.0e-12
        && point.x <= start.x.max(end.x) + 1.0e-12
        && point.y >= start.y.min(end.y) - 1.0e-12
        && point.y <= start.y.max(end.y) + 1.0e-12
}

fn distance(left: [f64; 2], right: [f64; 2]) -> f64 {
    let dx = left[0] - right[0];
    let dy = left[1] - right[1];
    (dx * dx + dy * dy).sqrt()
}
