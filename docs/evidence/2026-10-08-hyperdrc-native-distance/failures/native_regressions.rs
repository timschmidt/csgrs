use super::*;
use crate::geometry::{Polygon, circle_polygon, rect_polygon};
use crate::scalar::scalar;
use crate::{PcbRegion, PcbRegionExt};

#[test]
fn native_circle_distance_is_independent_of_report_resolution() {
    let expected = scalar("4.015625").sqrt().unwrap() - Scalar::from(2);
    for samples in [16, 17, 48, 128] {
        let left = MultiPolygon(vec![circle_polygon([0.0, 0.0], 1.0, samples)]);
        let right = MultiPolygon(vec![circle_polygon([2.0, 0.125], 1.0, samples)]);
        let actual = polygon_boundary_distance_scalar(&left, &right).unwrap();
        assert_eq!(
            crate::scalar::compare(&actual, &expected),
            Some(std::cmp::Ordering::Equal),
            "native circle gap with {samples} report samples"
        );
    }
}

#[test]
fn point_clearance_uses_the_authored_circle() {
    let polygons = MultiPolygon(vec![circle_polygon([0.0, 0.0], 1.0, 48)]);
    let point = [scalar("0.999"), scalar("0.03")];
    let threshold = scalar("0.00075");
    let norm_squared = &point[0] * &point[0] + &point[1] * &point[1];
    let inner_radius = Scalar::one() - &threshold;
    assert!(norm_squared < Scalar::one());
    assert!(norm_squared > &inner_radius * &inner_radius);
    assert!(exact_point_polygon_boundary_within_scalar(
        &point,
        [0.999, 0.03],
        &polygons,
        &threshold
    ));
}

#[test]
fn overlapping_profiles_do_not_publish_internal_seams_as_boundaries() {
    let polygons = MultiPolygon(vec![
        rect_polygon([1.0, 1.0], [2.0, 2.0], 0.0),
        rect_polygon([2.0, 1.0], [2.0, 2.0], 0.0),
    ]);
    // The union is [0,3] x [0,2]. Its closest boundary is one unit away,
    // although the second authored rectangle has an edge through this point.
    assert!(!exact_point_polygon_boundary_within_scalar(
        &[Scalar::one(), Scalar::one()],
        [1.0, 1.0],
        &polygons,
        &scalar("0.1")
    ));
}

#[test]
fn native_hole_boundary_retains_its_radius() {
    let origin = [Scalar::zero(), Scalar::zero()];
    let outer = crate::geometry::circle_region(&origin, &Scalar::from(3)).unwrap();
    let inner = crate::geometry::circle_region(&origin, &Scalar::one()).unwrap();
    let native_outer = outer.native_contours_fast_path().unwrap().unwrap();
    let native_inner = inner.native_contours_fast_path().unwrap().unwrap();
    let region = hypercurve::CurveRegion2::try_from_native_contours(
        native_outer.material_contours().to_vec(),
        native_inner.material_contours().to_vec(),
    )
    .unwrap();
    let polygons = PcbRegion::new(region, None).to_multipolygon();
    assert!(!exact_point_polygon_boundary_within_scalar(
        &origin,
        [0.0, 0.0],
        &polygons,
        &scalar("0.999999")
    ));
}

#[test]
fn unsupported_native_boundary_cannot_borrow_report_segments() {
    use hypercurve::{
        Curve2, CurvePath2, CurveRegionLoopRole, FillRule, LineSeg2, RationalBezier2,
    };
    let curve = RationalBezier2::try_new(
        [(0, 0), (1, 1), (2, 2), (3, 2), (4, 1), (5, 0)]
            .into_iter()
            .map(|(x, y)| hypercurve::Point2::from_values(x, y))
            .collect(),
        [1, 2, 3, 4, 5, 7].into_iter().map(Scalar::from).collect(),
    )
    .unwrap();
    let path = CurvePath2::try_new(vec![
        Curve2::from(curve),
        Curve2::from(
            LineSeg2::try_new(
                hypercurve::Point2::from_values(5, 0),
                hypercurve::Point2::from_values(0, 0),
            )
            .unwrap(),
        ),
    ])
    .unwrap();
    let region = hypercurve::CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
    )
    .unwrap();
    let polygon = Polygon::from_exact_region(
        region,
        LineString(vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 5.0, y: 0.0 },
            Coord {
                x: 325.0 / 113.0,
                y: 175.0 / 113.0,
            },
            Coord { x: 0.0, y: 0.0 },
        ]),
        vec![],
    );
    let query = MultiPolygon(vec![rect_polygon([8.0, 0.0], [1.0, 1.0], 0.0)]);
    assert!(polygon_boundary_distance_scalar(&MultiPolygon(vec![polygon]), &query).is_none());
}

#[test]
fn exhausted_prepared_metric_does_not_publish_a_cached_distance() {
    let left = MultiPolygon(vec![rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0)]);
    let right = MultiPolygon(vec![rect_polygon([2.0, 0.0], [1.0, 1.0], 0.0)]);
    let left = PreparedBoundaryDistance::new(&left).unwrap();
    let right = PreparedBoundaryDistance::new(&right).unwrap();
    assert_eq!(left.distance_to(&right), Some(Scalar::one()));
    let mut observation = None;
    let denied =
        hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
            let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
                [-36, 1368, -11034, 31728, -38280, 16620]
                    .into_iter()
                    .map(Scalar::from)
                    .collect(),
            )
            .unwrap();
            assert!(polynomial.isolate_unit_interval_roots().is_err());
            observation = Some(left.distance_to(&right));
        });
    assert!(denied.result.is_err());
    assert_eq!(observation, Some(None));
}
