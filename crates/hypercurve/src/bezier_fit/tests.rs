use super::*;
use crate::{CircularArc2, CurveCertainty, HomogeneousControl2, Similarity2};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn unit_circle() -> (
    std::sync::Arc<[Real; 6]>,
    std::sync::Arc<crate::rational_bezier::RationalQuadraticCircle2>,
) {
    let arc = CircularArc2::try_from_center(
        Point2::from_values(1, 0),
        Point2::from_values(0, 1),
        Point2::from_values(0, 0),
        false,
    )
    .unwrap();
    crate::arc_bezier::circular_conic_provenance(&arc)
}

fn native_circle(gauge: i32) -> RationalQuadraticBezier2 {
    let (implicit, circle) = unit_circle();
    // C(t)=((1-t²)/(1+t²),2t/(1+t²)), independently a unit circle.
    RationalQuadraticBezier2::try_new(
        Point2::from_values(1, 0),
        Point2::from_values(1, 1),
        Point2::from_values(0, 1),
        Real::from(gauge),
        Real::from(gauge),
        Real::from(2 * gauge),
    )
    .unwrap()
    .with_retained_conic_provenance(Some(implicit), Some(circle))
}

fn oracle(t: &Real) -> Point2 {
    let square = t * t;
    Point2::new(
        ((Real::one() - &square) / (Real::one() + &square)).unwrap(),
        ((Real::from(2) * t) / (Real::one() + square)).unwrap(),
    )
}

fn fit(
    curve: &RationalBezier2,
    policy: &CurveContext,
) -> Classification<BezierLineImageFitRelation> {
    let outcome = crate::evaluate_under(policy.predicate_policy(), || {
        curve.fit_exact_line_image_with_policy(policy)
    });
    assert_eq!(outcome.certainty(), CurveCertainty::Certified);
    outcome.certified().unwrap().unwrap()
}

fn assert_same_point(actual: &Point2, expected: &Point2, policy: &CurveContext) {
    assert_eq!(
        is_zero(&actual.distance_squared(expected), policy),
        Some(true)
    );
}

#[test]
fn circles_preserve_exact_line_fit_across_active_images() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for gauge in [1, -7] {
            let native = native_circle(gauge);
            let outcome = crate::evaluate_under(policy.predicate_policy(), || {
                native.fit_exact_line_image_with_policy(&policy)
            });
            assert_eq!(outcome.certainty(), CurveCertainty::Certified);
            assert!(matches!(
                outcome.certified().unwrap().unwrap(),
                Classification::Decided(BezierLineImageFitRelation::NotLine)
            ));
            for degree in [2, 3, 5] {
                let source = RationalBezier2::from(native.clone())
                    .elevated_to_degree(degree)
                    .unwrap();
                for (a, b) in [
                    (Real::zero(), Real::one()),
                    (q(1, 4), q(3, 4)),
                    (q(1, 2).sqrt().unwrap(), Real::one()),
                ] {
                    let Classification::Decided(curve) = source
                        .subcurve_between_exact_with_policy(&a, &b, &policy)
                        .unwrap()
                    else {
                        panic!("finite circular restriction")
                    };
                    assert!(curve.retained_circular_conic().is_some());
                    for reversed in [false, true] {
                        let active = if reversed {
                            curve.reversed()
                        } else {
                            curve.clone()
                        };
                        assert!(matches!(
                            fit(&active, &policy),
                            Classification::Decided(BezierLineImageFitRelation::NotLine)
                        ));
                        for t in [Real::zero(), q(1, 2), Real::one()] {
                            let local = if reversed {
                                Real::one() - &t
                            } else {
                                t.clone()
                            };
                            let source_t = &a + (&b - &a) * local;
                            let Classification::Decided(point) =
                                active.point_at_classified(&t, &policy)
                            else {
                                panic!("finite point")
                            };
                            assert_same_point(&point, &oracle(&source_t), &policy);
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn collapsed_circle_provenance_keeps_point_fitting() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = RationalBezier2::from(native_circle(-7));
        let Classification::Decided(curve) = source
            .subcurve_between_exact_with_policy(&q(1, 2), &q(1, 2), &policy)
            .unwrap()
        else {
            panic!("collapsed finite image")
        };
        assert!(curve.retained_circular_conic().is_some());
        drop(source);
        assert!(matches!(
            fit(&curve, &policy),
            Classification::Decided(BezierLineImageFitRelation::NotLine)
        ));
        let points = curve
            .affine_control_points()
            .unwrap()
            .iter()
            .collect::<Vec<_>>();
        let Classification::Decided(BezierPointImageFitRelation::Fit(point)) =
            fit_control_polygon_point_image(&points, &policy).unwrap()
        else {
            panic!("a collapsed circle remains a point")
        };
        assert_same_point(point.point(), &oracle(&q(1, 2)), &policy);
        let native = RationalQuadraticBezier2::try_new(
            point.point().clone(),
            point.point().clone(),
            point.point().clone(),
            Real::one(),
            Real::one(),
            Real::one(),
        )
        .unwrap();
        assert!(matches!(
            native.fit_exact_point_image_with_policy(&policy).unwrap(),
            Classification::Decided(BezierPointImageFitRelation::Fit(_))
        ));
    }
}

#[test]
fn retraced_images_do_not_acquire_a_segment_fit() {
    let (implicit, circle) = unit_circle();
    // s=4t(1-t) traverses the same circular arc twice.
    let retraced = RationalBezier2::try_new(
        vec![
            Point2::from_values(1, 0),
            Point2::from_values(1, 2),
            Point2::new(q(-5, 11), q(8, 11)),
            Point2::from_values(1, 2),
            Point2::from_values(1, 0),
        ],
        vec![Real::one(), Real::one(), q(11, 3), Real::one(), Real::one()],
    )
    .unwrap()
    .with_implicit_quadratic_conic(implicit, Some(circle));
    let line = RationalBezier2::try_new(
        vec![
            Point2::from_values(0, 0),
            Point2::from_values(1, 0),
            Point2::from_values(0, 0),
        ],
        vec![Real::one(); 3],
    )
    .unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for curve in [&retraced, &line] {
            assert!(matches!(
                fit(curve, &policy),
                Classification::Decided(BezierLineImageFitRelation::NotLine)
            ));
            let points = curve
                .affine_control_points()
                .unwrap()
                .iter()
                .collect::<Vec<_>>();
            assert!(matches!(
                fit_control_polygon_point_image(&points, &policy).unwrap(),
                Classification::Decided(BezierPointImageFitRelation::NotPoint)
            ));
        }
        for t in [q(1, 4), q(1, 2), q(3, 4)] {
            let Classification::Decided(actual) = retraced.point_at_classified(&t, &policy) else {
                panic!("finite retraced circle")
            };
            assert_same_point(
                &actual,
                &oracle(&(Real::from(4) * &t * (Real::one() - &t))),
                &policy,
            );
        }
    }
}

#[test]
fn mixed_weights_and_singular_circle_charts_keep_typed_declines() {
    let (implicit, circle) = unit_circle();
    // s=3t/(1-2t): W=1-4t+13t²>0, yet Bernstein weights have mixed signs.
    let native = RationalQuadraticBezier2::try_new(
        Point2::from_values(1, 0),
        Point2::from_values(1, -3),
        Point2::new(q(-4, 5), q(-3, 5)),
        Real::one(),
        -Real::one(),
        Real::from(10),
    )
    .unwrap()
    .with_retained_conic_provenance(Some(implicit.clone()), Some(circle.clone()));
    let finite = RationalBezier2::from(native.clone());
    // Multiply (1-t²,2t,1+t²) by (1-2t), leaving a common zero at t=1/2.
    let pole = RationalBezier2::from_homogeneous_controls(vec![
        HomogeneousControl2::new(Real::one(), Real::zero(), Real::one()),
        HomogeneousControl2::new(q(1, 3), q(2, 3), q(1, 3)),
        HomogeneousControl2::new(q(-2, 3), Real::zero(), Real::zero()),
        HomogeneousControl2::new(Real::zero(), Real::from(-2), Real::from(-2)),
    ])
    .unwrap()
    .with_implicit_quadratic_conic(implicit, Some(circle));
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for curve in [&finite, &pole] {
            assert_eq!(
                fit(curve, &policy),
                Classification::Uncertain(UncertaintyReason::Unsupported)
            );
            assert_eq!(
                fit(&curve.reversed(), &policy),
                Classification::Uncertain(UncertaintyReason::Unsupported)
            );
        }
        assert_eq!(
            native.fit_exact_line_image_with_policy(&policy).unwrap(),
            Classification::Uncertain(UncertaintyReason::Unsupported)
        );
    }
}

#[test]
fn similarity_and_ordinary_line_controls_preserve_continuation() {
    let transform = Similarity2::try_from_real_affine(
        Real::from(-2),
        Real::zero(),
        Real::zero(),
        Real::from(2),
        Real::from(3),
        Real::from(-1),
    )
    .unwrap();
    let source = RationalBezier2::from(native_circle(1));
    let transformed = source.transform_similarity(&transform);
    drop(source);
    let line = RationalBezier2::try_new(
        vec![
            Point2::from_values(0, 0),
            Point2::from_values(1, 0),
            Point2::from_values(2, 0),
        ],
        vec![Real::one(), Real::from(2), Real::one()],
    )
    .unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert!(matches!(
            fit(&transformed, &policy),
            Classification::Decided(BezierLineImageFitRelation::NotLine)
        ));
        for t in [Real::zero(), q(1, 2), Real::one()] {
            let p = oracle(&t);
            let expected = Point2::new(
                Real::from(3) - Real::from(2) * p.x(),
                Real::from(-1) + Real::from(2) * p.y(),
            );
            let Classification::Decided(actual) = transformed.point_at_classified(&t, &policy)
            else {
                panic!("finite transformed point")
            };
            assert_same_point(&actual, &expected, &policy);
        }
        let Classification::Decided(BezierLineImageFitRelation::Fit(fitted)) = fit(&line, &policy)
        else {
            panic!("ordinary exact line image")
        };
        let offset = fitted.offset_left_exact(Real::one()).unwrap();
        assert_eq!(offset.line().start(), &Point2::from_values(0, 1));
        assert_eq!(offset.line().end(), &Point2::from_values(2, 1));
    }
}

#[test]
fn native_arc_producers_preserve_nonconstant_tiny_circle_images() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for radius in [Real::one(), Real::from(2).powi_i64(-100).unwrap()] {
            let arc = CircularArc2::try_from_center(
                Point2::new(radius.clone(), Real::zero()),
                Point2::new(Real::zero(), radius),
                Point2::from_values(0, 0),
                false,
            )
            .unwrap();
            let decomposition = arc.rational_bezier_decomposition().unwrap();
            for span in decomposition.spans() {
                let native = span.curve();
                assert!(native.retained_circular_conic().is_some());
                assert!(matches!(
                    native.fit_exact_line_image_with_policy(&policy).unwrap(),
                    Classification::Decided(BezierLineImageFitRelation::NotLine)
                ));
                for degree in [2, 3, 5] {
                    let curve = RationalBezier2::from(native.clone())
                        .elevated_to_degree(degree)
                        .unwrap();
                    assert!(matches!(
                        fit(&curve, &policy),
                        Classification::Decided(BezierLineImageFitRelation::NotLine)
                    ));
                    assert!(matches!(
                        fit(&curve.reversed(), &policy),
                        Classification::Decided(BezierLineImageFitRelation::NotLine)
                    ));
                }
            }
        }
    }
}

#[test]
fn unresolved_collapsed_circle_keeps_policy_qualification() {
    let zero = crate::test_support::terminally_unresolved_zero();
    let (implicit, circle) = unit_circle();
    // The angle-addition identity makes this exactly the point (1,0), but
    // its endpoint equality is deliberately beyond bounded normalization.
    let native = RationalQuadraticBezier2::try_new(
        Point2::from_values(1, 0),
        Point2::from_values(1, 0),
        Point2::new(Real::one() + zero, Real::zero()),
        Real::one(),
        Real::one(),
        Real::one(),
    )
    .unwrap()
    .with_retained_conic_provenance(Some(implicit), Some(circle));
    let general = RationalBezier2::from(native.clone());
    for policy in [
        CurveContext::STRICT,
        CurveContext::APPROXIMATE_512,
        CurveContext::STRICT,
    ] {
        for general_form in [false, true] {
            let outcome = crate::evaluate_under(policy.predicate_policy(), || {
                if general_form {
                    general.fit_exact_line_image_with_policy(&policy)
                } else {
                    native.fit_exact_line_image_with_policy(&policy)
                }
            });
            if policy == CurveContext::STRICT {
                assert_eq!(outcome.certainty(), CurveCertainty::Certified);
                assert_eq!(
                    outcome.certified().unwrap().unwrap(),
                    Classification::Uncertain(UncertaintyReason::RealSign)
                );
            } else {
                assert_eq!(outcome.certainty(), CurveCertainty::Approximate512Consumed);
                assert!(matches!(
                    outcome.into_unverified().unwrap(),
                    Classification::Decided(BezierLineImageFitRelation::NotLine)
                ));
            }
        }
    }
}
