//! Retained algebraic endpoints must not use an arbitrary isolator midpoint.
use hypercurve::{
    BooleanOp, Curve2, CurveCertainty, CurvePath2, CurveRegion2, FillRule, FiniteProjectionOptions,
    LineSeg2, Point2, PredicatePolicy, QuadraticBezier2, Real, evaluate_under,
    finite_ring_signed_area,
};

fn point(x: i64, y: i64) -> Point2 {
    Point2::from_values(x, y)
}

fn clipped_parabola(reversed: bool) -> CurveRegion2 {
    let path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(
            point(-2, 4),
            point(0, -4),
            point(2, 4),
        )),
        LineSeg2::try_new(point(2, 4), point(-2, 4)).unwrap().into(),
    ])
    .unwrap();
    let path = if reversed {
        path.reversed().unwrap()
    } else {
        path
    };
    let source = CurveRegion2::try_from_boundary_paths(&[path], FillRule::NonZero).unwrap();
    let points = [point(-3, 2), point(3, 2), point(3, 5), point(-3, 5)];
    let cutter = CurvePath2::try_new(
        (0..4)
            .map(|i| {
                LineSeg2::try_new(points[i].clone(), points[(i + 1) % 4].clone())
                    .unwrap()
                    .into()
            })
            .collect(),
    )
    .unwrap();
    let cutter = CurveRegion2::try_from_boundary_paths(&[cutter], FillRule::NonZero).unwrap();
    source
        .boolean_region(&cutter, BooleanOp::Difference)
        .unwrap()
        .transform_affine(
            &Real::from(2),
            &Real::zero(),
            &Real::zero(),
            &Real::from(3),
            &Real::zero(),
            &Real::zero(),
        )
        .unwrap()
}

fn check_samples(points: &[[f64; 2]]) {
    // Independently, the scaled region is {3*x*x/4 <= y <= 6}.
    let area = finite_ring_signed_area(points).abs();
    let deficit = 16.0 * std::f64::consts::SQRT_2 - area;
    assert!((-0.001..0.014).contains(&deficit), "area deficit {deficit}");
    for sign in [-1.0, 1.0] {
        assert!(
            points
                .iter()
                .any(
                    |[x, y]| (*x - sign * 2.0 * std::f64::consts::SQRT_2).abs() < 0.000125
                        && (*y - 6.0).abs() < 0.000125
                )
        );
    }
    for [x, y] in points {
        assert!((*y - 0.75 * x * x).abs() < 0.00025 || (*y - 6.0).abs() < 0.000125);
    }
}

#[test]
fn cold_and_replayed_algebraic_samples_retain_both_cut_endpoints() {
    let options = FiniteProjectionOptions::try_new(1e-3).unwrap();
    for reversed in [false, true] {
        for finite_first in [false, true] {
            let region = clipped_parabola(reversed);
            assert!(region.has_algebraic_fragments());
            for policy in [
                PredicatePolicy::STRICT,
                PredicatePolicy::APPROXIMATE_512,
                PredicatePolicy::STRICT,
            ] {
                for finite in [finite_first, !finite_first] {
                    let result = evaluate_under(policy, || {
                        if finite {
                            let profiles = region.project_to_finite_profiles_exact(&options)?;
                            assert_eq!(profiles.len(), 1);
                            assert!(profiles[0].holes().is_empty());
                            check_samples(profiles[0].material().points());
                        } else {
                            let profiles = region.boundary_profiles()?;
                            assert_eq!(profiles.len(), 1);
                            let (material, holes) =
                                profiles[0].sample_to_linear_contours(&options)?;
                            assert!(holes.is_empty());
                            let points = material
                                .segments()
                                .iter()
                                .map(|segment| {
                                    [
                                        segment.start().x().to_f64_lossy().unwrap(),
                                        segment.start().y().to_f64_lossy().unwrap(),
                                    ]
                                })
                                .collect::<Vec<_>>();
                            check_samples(&points);
                        }
                        Ok::<_, hypercurve::ExactCurveError>(())
                    });
                    assert_eq!(result.certainty(), CurveCertainty::Certified);
                    result.into_unverified().unwrap();
                }
            }
        }
    }
}
