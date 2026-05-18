//! Geometry distance helpers shared by board-level checks.
//!
//! `csgrs` handles boolean geometry. These helpers fill the gap for clearance
//! fallbacks where two shapes are close but do not intersect.

use geo::{Coord, LineString, MultiPolygon, Polygon};
use hyperlimit::{Point2, PredicatePolicy, SegmentIntersection};

use crate::geometry::{RuleGeometryProvenance, SourceGridFacts};

pub(super) fn polygon_boundary_distance(
    left: &MultiPolygon<f64>,
    right: &MultiPolygon<f64>,
) -> f64 {
    polygon_boundary_distance_with_grid(left, right, SourceGridFacts::PRIMITIVE_FLOAT_EDGE)
}

pub(super) fn polygon_boundary_distance_with_grid(
    left: &MultiPolygon<f64>,
    right: &MultiPolygon<f64>,
    grid: SourceGridFacts,
) -> f64 {
    // Boundary-distance fallbacks still return an approximate metric, but
    // topology gates inside the segment walk should see the parser's retained
    // source grid whenever the caller has one. This is the object-layer
    // scheduling discipline from Yap, "Towards Exact Geometric Computation,"
    // *Computational Geometry* 7.1-2 (1997): preserve source structure at the
    // geometric boundary before expanding to scalar arithmetic.
    let mut minimum = f64::INFINITY;
    for left_polygon in &left.0 {
        for right_polygon in &right.0 {
            minimum = minimum.min(single_polygon_boundary_distance(
                left_polygon,
                right_polygon,
                grid,
            ));
        }
    }
    minimum
}

fn single_polygon_boundary_distance(
    left: &Polygon<f64>,
    right: &Polygon<f64>,
    grid: SourceGridFacts,
) -> f64 {
    let mut minimum = ring_boundary_distance(left.exterior(), right.exterior(), grid);

    for left_hole in left.interiors() {
        minimum = minimum.min(ring_boundary_distance(left_hole, right.exterior(), grid));
        for right_hole in right.interiors() {
            minimum = minimum.min(ring_boundary_distance(left_hole, right_hole, grid));
        }
    }

    for right_hole in right.interiors() {
        minimum = minimum.min(ring_boundary_distance(left.exterior(), right_hole, grid));
    }

    minimum
}

fn ring_boundary_distance(
    left: &LineString<f64>,
    right: &LineString<f64>,
    grid: SourceGridFacts,
) -> f64 {
    let mut minimum = f64::INFINITY;
    for left_segment in left.0.windows(2) {
        for right_segment in right.0.windows(2) {
            minimum = minimum.min(segment_distance_with_grid(
                left_segment[0],
                left_segment[1],
                right_segment[0],
                right_segment[1],
                grid,
            ));
        }
    }
    minimum
}

#[cfg(test)]
fn segment_distance(
    a_start: Coord<f64>,
    a_end: Coord<f64>,
    b_start: Coord<f64>,
    b_end: Coord<f64>,
) -> f64 {
    segment_distance_with_grid(
        a_start,
        a_end,
        b_start,
        b_end,
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    )
}

fn segment_distance_with_grid(
    a_start: Coord<f64>,
    a_end: Coord<f64>,
    b_start: Coord<f64>,
    b_end: Coord<f64>,
    grid: SourceGridFacts,
) -> f64 {
    if !coords_are_finite_4(a_start, a_end, b_start, b_end) {
        return f64::INFINITY;
    }

    if segments_intersect_with_grid(a_start, a_end, b_start, b_end, grid) {
        return 0.0;
    }

    point_segment_distance_with_grid(a_start, b_start, b_end, grid)
        .min(point_segment_distance_with_grid(
            a_end, b_start, b_end, grid,
        ))
        .min(point_segment_distance_with_grid(
            b_start, a_start, a_end, grid,
        ))
        .min(point_segment_distance_with_grid(
            b_end, a_start, a_end, grid,
        ))
}

#[cfg(test)]
fn point_segment_distance(point: Coord<f64>, start: Coord<f64>, end: Coord<f64>) -> f64 {
    point_segment_distance_with_grid(point, start, end, SourceGridFacts::PRIMITIVE_FLOAT_EDGE)
}

fn point_segment_distance_with_grid(
    point: Coord<f64>,
    start: Coord<f64>,
    end: Coord<f64>,
    grid: SourceGridFacts,
) -> f64 {
    if !coords_are_finite_3(point, start, end) {
        return f64::INFINITY;
    }

    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let length_squared = dx * dx + dy * dy;
    if exact_coords_equal_with_grid(start, end, grid) {
        return distance([point.x, point.y], [start.x, start.y]);
    }
    if length_squared == 0.0 {
        // Metric-edge underflow: exact equality above proved this is not a
        // point segment, but f64 projection cannot represent the squared
        // length. Fall back to endpoint distance for a finite conservative
        // report magnitude; topology has already been handled by exact
        // predicates before this projection path.
        return distance([point.x, point.y], [start.x, start.y])
            .min(distance([point.x, point.y], [end.x, end.y]));
    }

    let t =
        (((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared).clamp(0.0, 1.0);
    distance([point.x, point.y], [start.x + t * dx, start.y + t * dy])
}

#[cfg(test)]
fn segments_intersect(
    a_start: Coord<f64>,
    a_end: Coord<f64>,
    b_start: Coord<f64>,
    b_end: Coord<f64>,
) -> bool {
    segments_intersect_with_grid(
        a_start,
        a_end,
        b_start,
        b_end,
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    )
}

fn segments_intersect_with_grid(
    a_start: Coord<f64>,
    a_end: Coord<f64>,
    b_start: Coord<f64>,
    b_end: Coord<f64>,
    grid: SourceGridFacts,
) -> bool {
    if !coords_are_finite_4(a_start, a_end, b_start, b_end) {
        return false;
    }

    let Some((a, b, c, d)) = lift_segment_points(a_start, a_end, b_start, b_end, grid) else {
        return false;
    };

    // Clearance geometry still arrives from `geo`/`csgrs` as finite f64 edge
    // coordinates, but f64 must remain an I/O compatibility boundary rather
    // than a topology kernel. IEEE-754 coordinates are lifted to exact dyadic
    // `Real`s, then the closed-segment classifier routes orientation and
    // interval tests through `hyperlimit`. This follows Yap's exact geometric
    // computation boundary: combinatorial decisions are made by exact
    // predicates, while approximate coordinates may be used only to describe
    // inputs or report metric magnitudes. See Yap, "Towards Exact Geometric
    // Computation," Computational Geometry 7.1-2 (1997), and Shewchuk,
    // "Adaptive Precision Floating-Point Arithmetic and Fast Robust Geometric
    // Predicates," Discrete & Computational Geometry 18.3 (1997).
    match hyperlimit::classify_segment_intersection_with_policy(
        &a,
        &b,
        &c,
        &d,
        PredicatePolicy::STRICT,
    )
    .value()
    {
        Some(SegmentIntersection::Disjoint) => false,
        Some(_) => true,
        // A strict predicate over lifted finite dyadics should decide. If a
        // future symbolic source reaches this path undecided, report contact
        // conservatively so a clearance check does not silently miss a
        // violation.
        None => true,
    }
}

fn lift_segment_points(
    a_start: Coord<f64>,
    a_end: Coord<f64>,
    b_start: Coord<f64>,
    b_end: Coord<f64>,
    grid: SourceGridFacts,
) -> Option<(Point2, Point2, Point2, Point2)> {
    let provenance = RuleGeometryProvenance::new("clearance-segment-topology", grid);
    Some((
        lift_coord(a_start, provenance)?,
        lift_coord(a_end, provenance)?,
        lift_coord(b_start, provenance)?,
        lift_coord(b_end, provenance)?,
    ))
}

fn lift_coord(coord: Coord<f64>, provenance: RuleGeometryProvenance) -> Option<Point2> {
    Some(Point2::new(
        provenance.lift_f64(coord.x)?,
        provenance.lift_f64(coord.y)?,
    ))
}

#[cfg(test)]
fn exact_coords_equal(left: Coord<f64>, right: Coord<f64>) -> bool {
    exact_coords_equal_with_grid(left, right, SourceGridFacts::PRIMITIVE_FLOAT_EDGE)
}

fn exact_coords_equal_with_grid(
    left: Coord<f64>,
    right: Coord<f64>,
    grid: SourceGridFacts,
) -> bool {
    // Degenerate segment classification is a topology decision even when the
    // resulting distance magnitude is reported as f64. Lift finite coordinates
    // and ask `hyperlimit` for exact point equality instead of using
    // `length_squared == 0.0`, which can conflate a very small nonzero segment
    // with a point after primitive-float underflow. This keeps the clearance
    // fallback aligned with Yap's exact-geometric-computation boundary; see
    // Yap, "Towards Exact Geometric Computation," Computational Geometry 7.1-2
    // (1997).
    //
    let provenance = RuleGeometryProvenance::new("clearance-degenerate-segment", grid);
    let Some(left) = lift_coord(left, provenance) else {
        return false;
    };
    let Some(right) = lift_coord(right, provenance) else {
        return false;
    };
    hyperlimit::point2_equal_with_policy(&left, &right, PredicatePolicy::STRICT)
        .value()
        .unwrap_or(false)
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

    use crate::geometry::{SourceGridFacts, SourceUnit};

    use super::{
        point_segment_distance, polygon_boundary_distance, polygon_boundary_distance_with_grid,
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
    fn point_segment_distance_keeps_tiny_nonzero_segment_distinct_from_point() {
        let point = Coord {
            x: 1.0e-200,
            y: 0.0,
        };
        let start = Coord { x: 0.0, y: 0.0 };
        let end = Coord {
            x: 1.0e-200,
            y: 0.0,
        };

        assert!(super::exact_coords_equal(start, start));
        assert!(!super::exact_coords_equal(start, end));
        assert_eq!(point_segment_distance(point, start, end), 0.0);
    }

    #[test]
    fn segments_intersect_uses_exact_closed_segment_topology() {
        assert!(segments_intersect(
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 2.0, y: 0.0 },
            Coord { x: 1.0, y: 0.0 },
            Coord { x: 3.0, y: 0.0 }
        ));
        assert!(!segments_intersect(
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 2.0, y: 0.0 },
            Coord { x: 3.0, y: 0.0 },
            Coord { x: 4.0, y: 0.0 }
        ));
    }

    #[test]
    fn polygon_boundary_distance_reports_touching_polygons_as_zero() {
        let left = MultiPolygon(vec![square(0.0, 0.0, 1.0)]);
        let right = MultiPolygon(vec![square(1.0, 0.0, 1.0)]);

        assert_eq!(polygon_boundary_distance(&left, &right), 0.0);
    }

    #[test]
    fn polygon_boundary_distance_accepts_retained_source_grid_provenance() {
        let left = MultiPolygon(vec![square(0.0, 0.0, 1.0)]);
        let right = MultiPolygon(vec![square(1.0, 0.0, 1.0)]);
        let grid = SourceGridFacts::source_grid(SourceUnit::Gerber, 1_000_000);

        assert_eq!(
            polygon_boundary_distance_with_grid(&left, &right, grid),
            0.0
        );
        assert!(!super::exact_coords_equal_with_grid(
            Coord { x: 0.0, y: 0.0 },
            Coord {
                x: 1.0e-200,
                y: 0.0
            },
            grid,
        ));
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
    fn segments_intersect_with_non_finite_inputs_returns_false() {
        assert!(!segments_intersect(
            Coord {
                x: f64::NAN,
                y: 0.0
            },
            Coord { x: 1.0, y: 1.0 },
            Coord { x: 0.0, y: 1.0 },
            Coord { x: 1.0, y: 0.0 }
        ));
    }

    #[test]
    fn segment_distance_does_not_zero_parallel_traces_inside_old_epsilon() {
        let left = [Coord { x: 0.0, y: 0.0 }, Coord { x: 1.0, y: 0.0 }];
        let right = [Coord { x: 0.0, y: 5.0e-13 }, Coord { x: 1.0, y: 5.0e-13 }];

        let measured = segment_distance(left[0], left[1], right[0], right[1]);
        assert!(measured > 0.0);
        assert!((measured - 5.0e-13).abs() <= f64::EPSILON);
    }

    #[test]
    fn segment_distance_still_zeroes_tiny_exact_crossing() {
        let left = [Coord { x: 0.0, y: 0.0 }, Coord { x: 1.0, y: 1.0e-13 }];
        let right = [Coord { x: 0.0, y: 1.0e-13 }, Coord { x: 1.0, y: 0.0 }];

        assert_eq!(segment_distance(left[0], left[1], right[0], right[1]), 0.0);
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
