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
    if !coords_are_finite_4(a_start, a_end, b_start, b_end) {
        return f64::INFINITY;
    }

    if segments_intersect(a_start, a_end, b_start, b_end) {
        return 0.0;
    }

    point_segment_distance(a_start, b_start, b_end)
        .min(point_segment_distance(a_end, b_start, b_end))
        .min(point_segment_distance(b_start, a_start, a_end))
        .min(point_segment_distance(b_end, a_start, a_end))
}

fn point_segment_distance(point: Coord<f64>, start: Coord<f64>, end: Coord<f64>) -> f64 {
    if !coords_are_finite_3(point, start, end) {
        return f64::INFINITY;
    }

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
    if !coords_are_finite_4(a_start, a_end, b_start, b_end) {
        return false;
    }

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
    if !coords_are_finite_3(a, b, c) {
        return 0.0;
    }

    let cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if cross.abs() < 1.0e-12 { 0.0 } else { cross }
}

fn point_on_segment(point: Coord<f64>, start: Coord<f64>, end: Coord<f64>) -> bool {
    if !coords_are_finite_3(point, start, end) {
        return false;
    }

    point.x >= start.x.min(end.x) - 1.0e-12
        && point.x <= start.x.max(end.x) + 1.0e-12
        && point.y >= start.y.min(end.y) - 1.0e-12
        && point.y <= start.y.max(end.y) + 1.0e-12
}

fn distance(left: [f64; 2], right: [f64; 2]) -> f64 {
    if !left[0].is_finite()
        || !left[1].is_finite()
        || !right[0].is_finite()
        || !right[1].is_finite()
    {
        return f64::INFINITY;
    }

    let dx = left[0] - right[0];
    let dy = left[1] - right[1];
    (dx * dx + dy * dy).sqrt()
}

fn coords_are_finite_4(
    first: Coord<f64>,
    second: Coord<f64>,
    third: Coord<f64>,
    fourth: Coord<f64>,
) -> bool {
    first.x.is_finite()
        && first.y.is_finite()
        && second.x.is_finite()
        && second.y.is_finite()
        && third.x.is_finite()
        && third.y.is_finite()
        && fourth.x.is_finite()
        && fourth.y.is_finite()
}

fn coords_are_finite_3(first: Coord<f64>, second: Coord<f64>, third: Coord<f64>) -> bool {
    first.x.is_finite()
        && first.y.is_finite()
        && second.x.is_finite()
        && second.y.is_finite()
        && third.x.is_finite()
        && third.y.is_finite()
}

#[cfg(test)]
mod tests {
    use geo::{Coord, LineString, MultiPolygon, Polygon};

    use super::{
        orientation, point_on_segment, point_segment_distance, polygon_boundary_distance,
        segment_distance, segments_intersect,
    };

    fn square(x: f64, y: f64, size: f64) -> Polygon<f64> {
        Polygon::new(
            LineString(vec![
                Coord { x, y },
                Coord { x: x + size, y },
                Coord {
                    x: x + size,
                    y: y + size,
                },
                Coord { x, y: y + size },
                Coord { x, y },
            ]),
            Vec::new(),
        )
    }

    #[test]
    fn segment_distance_reports_zero_for_endpoint_touch() {
        let left = [Coord { x: 0.0, y: 0.0 }, Coord { x: 2.0, y: 0.0 }];
        let right = [Coord { x: 2.0, y: 0.0 }, Coord { x: 2.0, y: 2.0 }];

        assert_eq!(segment_distance(left[0], left[1], right[0], right[1]), 0.0);
    }

    #[test]
    fn segment_distance_is_expected_for_parallel_lines() {
        let left = [Coord { x: 0.0, y: 0.0 }, Coord { x: 2.0, y: 0.0 }];
        let right = [Coord { x: 0.0, y: 1.0 }, Coord { x: 2.0, y: 1.0 }];

        assert!((segment_distance(left[0], left[1], right[0], right[1]) - 1.0).abs() < 1.0e-9);
    }

    #[test]
    fn point_segment_distance_uses_projection_for_internal_foot() {
        let point = Coord { x: 1.0, y: 1.0 };
        let start = Coord { x: 0.0, y: 0.0 };
        let end = Coord { x: 2.0, y: 0.0 };

        assert!((point_segment_distance(point, start, end) - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn point_segment_distance_falls_back_to_endpoint_for_degenerate_segment() {
        let point = Coord { x: 1.0, y: 2.0 };
        let endpoint = Coord { x: 3.0, y: 4.0 };

        assert_eq!(
            point_segment_distance(point, endpoint, endpoint),
            ((-2.0f64).powi(2) + (-2.0f64).powi(2)).sqrt()
        );
    }

    #[test]
    fn orientation_classifies_collinear_segments_as_zero() {
        assert_eq!(
            orientation(
                Coord { x: 0.0, y: 0.0 },
                Coord { x: 1.0, y: 1.0 },
                Coord { x: 2.0, y: 2.0 }
            ),
            0.0
        );
        assert_eq!(
            orientation(
                Coord { x: 0.0, y: 0.0 },
                Coord { x: 1.0, y: 0.0 },
                Coord { x: 2.0, y: 1.0e-13 },
            ),
            0.0
        );
    }

    #[test]
    fn point_on_segment_is_tolerant_of_closed_bounds() {
        assert!(point_on_segment(
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 2.0, y: 0.0 }
        ));
        assert!(point_on_segment(
            Coord { x: 1.0, y: 0.0 },
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 2.0, y: 0.0 }
        ));
        assert!(point_on_segment(
            Coord { x: 2.0, y: 0.0 },
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 2.0, y: 0.0 }
        ));
        assert!(!point_on_segment(
            Coord { x: 3.0, y: 0.0 },
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 2.0, y: 0.0 }
        ));
    }

    #[test]
    fn polygon_boundary_distance_reports_touching_polygons_as_zero() {
        let left = MultiPolygon(vec![square(0.0, 0.0, 1.0)]);
        let right = MultiPolygon(vec![square(1.0, 0.0, 1.0)]);

        assert_eq!(polygon_boundary_distance(&left, &right), 0.0);
    }

    #[test]
    fn polygon_boundary_distance_reports_separated_polygons_by_expected_gap() {
        let left = MultiPolygon(vec![square(0.0, 0.0, 1.0)]);
        let right = MultiPolygon(vec![square(0.0, 3.0, 1.0)]);

        assert_eq!(polygon_boundary_distance(&left, &right), 2.0);
    }

    #[test]
    fn polygon_boundary_distance_considers_hole_boundaries() {
        let outer = LineString(vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 4.0, y: 0.0 },
            Coord { x: 4.0, y: 4.0 },
            Coord { x: 0.0, y: 4.0 },
            Coord { x: 0.0, y: 0.0 },
        ]);
        let hole = LineString(vec![
            Coord { x: 1.5, y: 1.5 },
            Coord { x: 2.5, y: 1.5 },
            Coord { x: 2.5, y: 2.5 },
            Coord { x: 1.5, y: 2.5 },
            Coord { x: 1.5, y: 1.5 },
        ]);
        let with_hole = Polygon::new(outer, vec![hole]);
        let point_polygon = MultiPolygon(vec![with_hole]);

        let touch_hole = MultiPolygon(vec![Polygon::new(
            LineString(vec![
                Coord { x: 1.5, y: 1.5 },
                Coord { x: 2.5, y: 1.5 },
                Coord { x: 2.5, y: 2.5 },
                Coord { x: 1.5, y: 2.5 },
                Coord { x: 1.5, y: 1.5 },
            ]),
            Vec::new(),
        )]);

        assert_eq!(polygon_boundary_distance(&point_polygon, &touch_hole), 0.0);
    }

    #[test]
    fn polygon_boundary_distance_of_empty_geometry_is_infinite() {
        let left = MultiPolygon(vec![]);
        let right = MultiPolygon(vec![square(0.0, 0.0, 1.0)]);

        assert!(polygon_boundary_distance(&left, &right).is_infinite());
    }

    #[test]
    fn segment_distance_reports_zero_for_overlapping_collinear_segments() {
        let left = [Coord { x: 0.0, y: 0.0 }, Coord { x: 3.0, y: 0.0 }];
        let right = [Coord { x: 1.0, y: 0.0 }, Coord { x: 5.0, y: 0.0 }];

        assert_eq!(segment_distance(left[0], left[1], right[0], right[1]), 0.0);
    }

    #[test]
    fn segments_intersect_treats_touching_endpoints_as_intersection() {
        let left = [Coord { x: 0.0, y: 0.0 }, Coord { x: 2.0, y: 2.0 }];
        let right = [Coord { x: 2.0, y: 2.0 }, Coord { x: 4.0, y: 0.0 }];

        assert!(segments_intersect(left[0], left[1], right[0], right[1]));
    }

    #[test]
    fn polygon_boundary_distance_is_symmetric_for_hole_and_outer_inputs() {
        let outer = square(0.0, 0.0, 2.0);
        let inner = square(0.25, 0.25, 0.5);
        let with_hole = MultiPolygon(vec![Polygon::new(
            outer.exterior().clone(),
            vec![inner.exterior().clone()],
        )]);

        let outer_multi = MultiPolygon(vec![outer.clone()]);
        assert_eq!(
            polygon_boundary_distance(&outer_multi, &with_hole),
            polygon_boundary_distance(&with_hole, &MultiPolygon(vec![outer]))
        );
    }

    #[test]
    fn segment_distance_with_non_finite_endpoints_is_infinite() {
        let left = [
            Coord {
                x: f64::NAN,
                y: 0.0,
            },
            Coord { x: 1.0, y: 0.0 },
        ];
        let right = [Coord { x: 0.0, y: 0.0 }, Coord { x: 1.0, y: 1.0 }];

        assert!(segment_distance(left[0], left[1], right[0], right[1]).is_infinite());
    }

    #[test]
    fn point_segment_distance_with_non_finite_endpoint_is_infinite() {
        assert!(
            point_segment_distance(
                Coord { x: 0.0, y: 0.0 },
                Coord {
                    x: f64::INFINITY,
                    y: 1.0
                },
                Coord { x: 1.0, y: 1.0 }
            )
            .is_infinite()
        );
    }

    #[test]
    fn orientation_with_non_finite_inputs_returns_zero() {
        assert_eq!(
            orientation(
                Coord {
                    x: f64::NAN,
                    y: 0.0
                },
                Coord { x: 1.0, y: 1.0 },
                Coord { x: 2.0, y: 2.0 }
            ),
            0.0
        );
    }

    #[test]
    fn polygon_boundary_distance_skips_invalid_coordinates_as_no_geometry_overlap() {
        let invalid = Polygon::new(
            LineString(vec![
                Coord {
                    x: f64::NAN,
                    y: 0.0,
                },
                Coord {
                    x: f64::NAN,
                    y: 1.0,
                },
                Coord {
                    x: f64::NAN,
                    y: 1.0,
                },
                Coord {
                    x: f64::NAN,
                    y: 0.0,
                },
            ]),
            Vec::new(),
        );

        assert!(
            polygon_boundary_distance(
                &MultiPolygon(vec![invalid]),
                &MultiPolygon(vec![square(0.0, 0.0, 1.0)])
            )
            .is_infinite()
        );
    }
}
