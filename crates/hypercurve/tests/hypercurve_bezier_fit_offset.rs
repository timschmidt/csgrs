use hypercurve::{
    BezierAreaMomentPrefixSums2, BezierAreaPrefixSums2, BezierFlatteningOptions,
    BezierLineImageFitRelation, BezierOffsetCandidate2, BezierParallelApproximationCurve2,
    BezierParallelIncidence2, BezierParallelIntersectionCandidates2,
    BezierParallelIntersectionContacts2, BezierParallelVerificationOptions, BezierParameter2,
    Classification, CubicBezier2, Curve2, CurveContext, CurveError, CurvePath2, CurveRegion2,
    CurveRegionLoopRole, FillRule, LineSeg2, Point2, QuadraticBezier2, Rational, RationalBezier2,
    RationalBezierIntersectionPointEvidence2, RationalBezierOverlapOrientation2,
    RationalQuadraticBezier2, Real,
};
use num::bigint::{BigInt, BigUint};
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn policy() -> CurveContext {
    CurveContext::STRICT
}

fn q(numerator: i32, denominator: i32) -> Real {
    (r(numerator) / r(denominator)).unwrap()
}

fn rootless_homogeneous_factor_parabola() -> RationalBezier2 {
    // Homogeneous power basis
    //   (X, Y, W) = (t(t + 2), t^2(t + 2), t + 2)
    // represents the regular non-PH parabola (t, t^2). The common factor has
    // its only root at t=-2, outside the authored parameter interval.
    RationalBezier2::try_new(
        vec![
            p(0, 0),
            Point2::new(q(2, 7), r(0)),
            Point2::new(q(5, 8), q(1, 4)),
            p(1, 1),
        ],
        vec![r(2), q(7, 3), q(8, 3), r(3)],
    )
    .unwrap()
}

fn rootless_homogeneous_factor_vertical() -> RationalBezier2 {
    // (0, 2u(u + 2), u + 2) represents the same finite segment as (0, 2u).
    RationalBezier2::try_new(
        vec![p(0, 0), Point2::new(r(0), q(4, 5)), p(0, 2)],
        vec![r(2), q(5, 2), r(3)],
    )
    .unwrap()
}

fn rootful_homogeneous_factor_vertical() -> RationalBezier2 {
    // (0, 2u(u - 1/3), u - 1/3) has a removable projective base point at
    // u=1/3. Hypercurve deliberately retains that authored domain boundary.
    RationalBezier2::try_new(
        vec![p(0, 0), p(0, -2), p(0, 2)],
        vec![q(-1, 3), q(1, 6), q(2, 3)],
    )
    .unwrap()
}

fn real_representation_samples() -> Vec<Real> {
    let rational = |numerator: i64, denominator: u64| {
        Real::new(Rational::fraction(numerator, denominator).unwrap())
    };
    let pi = Real::pi();
    let exp_two = r(2).exp().unwrap();
    let sqrt_two = r(2).sqrt().unwrap();
    let ln_two = r(2).ln().unwrap();
    let ln_three = r(3).ln().unwrap();
    vec![
        r(7),
        rational(17, 31),
        Real::new(Rational::from_bigint(BigInt::from(1_u8) << 256)),
        Real::new(
            Rational::from_bigint_fraction(
                (BigInt::from(1_u8) << 257) + BigInt::from(19_u8),
                (BigUint::from(1_u8) << 193) + BigUint::from(7_u8),
            )
            .unwrap(),
        ),
        Real::try_from(0.1_f32).unwrap(),
        Real::try_from(0.1_f64).unwrap(),
        pi.clone(),
        &pi * &pi,
        pi.clone().inverse().unwrap(),
        exp_two.clone(),
        &pi * &exp_two,
        (&exp_two / &pi).unwrap(),
        &(&pi * &pi) * &exp_two,
        &pi - r(3),
        sqrt_two.clone(),
        &pi * &sqrt_two,
        &(&(&pi * &pi) * &exp_two) * &sqrt_two,
        ln_two.clone(),
        Real::one() + &ln_two,
        &ln_two * &ln_three,
        r(2).log10().unwrap(),
        r(3).log2().unwrap(),
        rational(1, 5).sin_pi(),
        rational(1, 5).tan_pi().unwrap(),
        r(1).sin(),
    ]
}

fn exact_parameter(parameter: &BezierParameter2) -> &Real {
    match parameter {
        BezierParameter2::Exact(parameter) => parameter,
        BezierParameter2::Algebraic(_) => panic!("expected represented exact parameter"),
    }
}

fn assert_real_eq(left: &Real, right: &Real) {
    assert_eq!(left.partial_cmp(right), Some(std::cmp::Ordering::Equal));
}

#[test]
fn quadratic_line_image_fit_offsets_as_exact_line() {
    let bezier = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0));

    let fit = bezier.fit_exact_line_image(&policy()).unwrap();
    let Classification::Decided(BezierLineImageFitRelation::Fit(fit)) = fit else {
        panic!("collinear quadratic should be a certified line image");
    };
    assert_eq!(fit.line().start(), &p(0, 0));
    assert_eq!(fit.line().end(), &p(2, 0));

    let offset = bezier.offset_left_staged(r(1), &policy()).unwrap();
    let Classification::Decided(BezierOffsetCandidate2::ExactLineImage { offset, preflight }) =
        offset
    else {
        panic!("line-image quadratic should offset as an exact primitive");
    };
    assert!(preflight.is_clear());
    assert_eq!(offset.line().start(), &p(0, 1));
    assert_eq!(offset.line().end(), &p(2, 1));
}

#[test]
fn rational_quadratic_conic_line_image_fit_offsets_as_exact_line() {
    let conic =
        RationalQuadraticBezier2::try_new(p(0, 0), p(1, 0), p(2, 0), r(1), r(2), r(1)).unwrap();

    let fit = conic.fit_exact_line_image(&policy()).unwrap();
    let Classification::Decided(BezierLineImageFitRelation::Fit(fit)) = fit else {
        panic!("same-sign collinear rational quadratic should be a certified line image");
    };
    assert_eq!(fit.control_point_count(), 3);
    assert_eq!(fit.line().start(), &p(0, 0));
    assert_eq!(fit.line().end(), &p(2, 0));

    let offset = conic.offset_left_staged(r(1), &policy()).unwrap();
    let Classification::Decided(BezierOffsetCandidate2::ExactLineImage { offset, preflight }) =
        offset
    else {
        panic!("line-image rational quadratic should offset as an exact primitive");
    };
    assert!(preflight.is_clear());
    assert_eq!(offset.line().start(), &p(0, 1));
    assert_eq!(offset.line().end(), &p(2, 1));
}

#[test]
fn bezier_area_prefix_sums_answer_exact_ranges() {
    let first = QuadraticBezier2::new(p(0, 0), p(1, 1), p(2, 0));
    let second = QuadraticBezier2::new(p(2, 0), p(3, -1), p(4, 0));
    let curves = [first, second];

    let area_prefixes = BezierAreaPrefixSums2::from_quadratics(curves.iter()).unwrap();
    assert_eq!(area_prefixes.segment_count(), 2);
    assert_eq!(
        area_prefixes.range_contribution(0..1).unwrap(),
        curves[0].signed_area_contribution().unwrap()
    );
    assert_eq!(
        area_prefixes.range_contribution(1..2).unwrap(),
        curves[1].signed_area_contribution().unwrap()
    );
    assert_eq!(
        area_prefixes.range_contribution(0..2).unwrap(),
        area_prefixes.total().clone()
    );
    let reversed_start = 2;
    let reversed_end = 1;
    assert_eq!(
        area_prefixes.range_contribution(reversed_start..reversed_end),
        Err(CurveError::InvalidBezierRange)
    );

    let moment_prefixes = BezierAreaMomentPrefixSums2::from_quadratics(curves.iter()).unwrap();
    assert_eq!(moment_prefixes.segment_count(), 2);
    assert_eq!(
        moment_prefixes.range_contribution(0..2).unwrap(),
        moment_prefixes.total().clone()
    );
    assert_eq!(
        moment_prefixes.range_contribution(0..1).unwrap(),
        curves[0].area_moments_contribution().unwrap()
    );
}

#[test]
fn retained_quadratic_parallel_evaluates_exact_point_and_derivative() {
    let source = QuadraticBezier2::new(p(0, 0), p(1, 1), p(2, 0));
    let parallel = source.parallel_left(r(2)).unwrap();

    let point = match parallel.point_at(&q(1, 2), &policy()).unwrap() {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => panic!("midpoint was uncertain: {reason:?}"),
    };
    assert_eq!(point, Point2::new(r(1), q(5, 2)));

    let derivative = match parallel.derivative_at(&q(1, 2), &policy()).unwrap() {
        Classification::Decided(derivative) => derivative,
        Classification::Uncertain(reason) => {
            panic!("parallel derivative was uncertain: {reason:?}")
        }
    };
    assert_eq!(derivative.dx(), &r(6));
    assert_eq!(derivative.dy(), &r(0));
}

#[test]
fn zero_distance_parallel_is_exact_source_even_at_source_cusp() {
    let source = QuadraticBezier2::new(p(0, 0), p(1, 0), p(0, 0));
    let parallel = source.parallel_left(r(0)).unwrap();
    let midpoint = q(1, 2);
    let point = match parallel.point_at(&midpoint, &policy()).unwrap() {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => panic!("identity parallel was uncertain: {reason:?}"),
    };
    assert_eq!(point, source.point_at(midpoint));
    let analysis = match parallel.singularity_analysis(&policy()).unwrap() {
        Classification::Decided(analysis) => analysis,
        Classification::Uncertain(reason) => panic!("identity analysis was uncertain: {reason:?}"),
    };
    assert_eq!(analysis.source_singularities().len(), 1);
    assert_eq!(
        exact_parameter(&analysis.source_singularities()[0]),
        &q(1, 2)
    );
    assert!(analysis.parallel_cusps().is_empty());
}

#[test]
fn quadratic_parallel_isolates_distance_dependent_interior_cusp() {
    // P(t) = (t, t^2). At t=1/2, |P'|^3 = 2*sqrt(2) and
    // P'' x P' = -2, so a left distance sqrt(2) creates a parallel cusp.
    let source = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(1, 1));
    let parallel = source.parallel_left(r(2).sqrt().unwrap()).unwrap();
    let analysis = match parallel.singularity_analysis(&policy()).unwrap() {
        Classification::Decided(analysis) => analysis,
        Classification::Uncertain(reason) => panic!("cusp isolation was uncertain: {reason:?}"),
    };
    assert!(analysis.source_is_regular());
    assert_eq!(analysis.parallel_cusps().len(), 1);
    assert_eq!(exact_parameter(&analysis.parallel_cusps()[0]), &q(1, 2));

    let derivative = match parallel.derivative_at(&q(1, 2), &policy()).unwrap() {
        Classification::Decided(derivative) => derivative,
        Classification::Uncertain(reason) => panic!("cusp derivative was uncertain: {reason:?}"),
    };
    assert_eq!(derivative.zero_status(), hyperreal::ZeroKnowledge::Zero);
}

#[test]
fn quadratic_parallel_retains_nonrepresented_algebraic_cusp() {
    let source = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(1, 1));
    let parallel = source.parallel_left(r(1)).unwrap();
    let analysis = match parallel.singularity_analysis(&policy()).unwrap() {
        Classification::Decided(analysis) => analysis,
        Classification::Uncertain(reason) => {
            panic!("algebraic cusp isolation was uncertain: {reason:?}")
        }
    };
    assert!(analysis.source_is_regular());
    assert_eq!(analysis.parallel_cusps().len(), 1);
    let BezierParameter2::Algebraic(cusp) = &analysis.parallel_cusps()[0] else {
        panic!("expected a retained nonrational algebraic cusp");
    };
    assert_eq!(cusp.root_count(), 1);
    assert!(cusp.polynomial().degree() >= 4);
}

#[test]
fn cubic_parallel_isolates_a_symmetric_pair_of_offset_cusps() {
    let source = CubicBezier2::new(p(0, 0), p(1, -4), p(2, -4), p(3, 0));
    let analysis = match source
        .parallel_left(q(1, 2))
        .unwrap()
        .singularity_analysis(&policy())
        .unwrap()
    {
        Classification::Decided(analysis) => analysis,
        Classification::Uncertain(reason) => panic!("cusp-pair isolation failed: {reason:?}"),
    };
    assert!(analysis.source_is_regular());
    assert_eq!(analysis.parallel_cusps().len(), 2);
    assert!(
        analysis.parallel_cusps()[0]
            .cmp_by_interval(&analysis.parallel_cusps()[1], &policy())
            .unwrap()
            .is_decided()
    );
}

#[test]
fn retained_parallel_accepts_every_hyperreal_representation_as_exact_translation() {
    let parameter = q(1, 3);
    for translation in real_representation_samples() {
        let source = QuadraticBezier2::new(
            Point2::new(translation.clone(), r(0)),
            Point2::new(&translation + r(1), r(1)),
            Point2::new(&translation + r(2), r(0)),
        );
        let parallel = source.parallel_left(q(1, 10)).unwrap();
        let point = match parallel.point_at(&parameter, &policy()).unwrap() {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                panic!("representation-specific parallel evaluation failed: {reason:?}")
            }
        };
        assert!(point.x().to_f64_lossy().is_some());
        assert!(point.y().to_f64_lossy().is_some());
    }
}

#[test]
fn exact_parallel_commutes_with_orientation_preserving_rigid_transform() {
    let source = QuadraticBezier2::new(p(0, 0), p(2, 3), p(5, 1));
    let transformed = QuadraticBezier2::new(
        Point2::new(r(5) - source.start().y(), source.start().x() - r(3)),
        Point2::new(r(5) - source.control().y(), source.control().x() - r(3)),
        Point2::new(r(5) - source.end().y(), source.end().x() - r(3)),
    );
    let parallel = source.parallel_left(q(2, 5)).unwrap();
    let transformed_parallel = transformed.parallel_left(q(2, 5)).unwrap();
    for parameter in [r(0), q(1, 3), q(2, 3), r(1)] {
        let point = match parallel.point_at(&parameter, &policy()).unwrap() {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => panic!("source parallel failed: {reason:?}"),
        };
        let expected = Point2::new(r(5) - point.y(), point.x() - r(3));
        let actual = match transformed_parallel
            .point_at(&parameter, &policy())
            .unwrap()
        {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                panic!("transformed parallel failed: {reason:?}")
            }
        };
        let error = actual.distance_squared(&expected).to_f64_lossy().unwrap();
        assert!(
            error.abs() <= 1.0e-20,
            "rigid-transform replay error was {error}"
        );
    }
}

#[test]
fn cubic_parallel_analysis_keeps_regular_inflection_cusp_free() {
    let source = CubicBezier2::new(p(0, 0), p(1, 2), p(2, -2), p(3, 0));
    let parallel = source.parallel_left(q(1, 100)).unwrap();
    let analysis = match parallel.singularity_analysis(&policy()).unwrap() {
        Classification::Decided(analysis) => analysis,
        Classification::Uncertain(reason) => {
            panic!("regular inflected cubic analysis was uncertain: {reason:?}")
        }
    };
    assert!(analysis.source_is_regular());
    assert!(analysis.parallel_is_cusp_free());
    assert!(analysis.parallel_cusp_polynomial_degree().unwrap() <= 12);
}

#[test]
fn cubic_pythagorean_hodograph_parallel_materializes_exact_rational_bezier() {
    // P(t) = (t - t^3/3, t^2), with
    // P'(t) = (1 - t^2, 2t) and |P'(t)|^2 = (1 + t^2)^2.
    let source = CubicBezier2::new(
        p(0, 0),
        Point2::new(q(1, 3), r(0)),
        Point2::new(q(2, 3), q(1, 3)),
        Point2::new(q(2, 3), r(1)),
    );
    let parallel = source.parallel_left(r(1)).unwrap();
    let exact = match parallel
        .exact_pythagorean_hodograph_offset(&policy())
        .unwrap()
    {
        Classification::Decided(Some(exact)) => exact,
        Classification::Decided(None) => panic!("PH cubic was not recognized"),
        Classification::Uncertain(reason) => panic!("PH recognition was uncertain: {reason:?}"),
    };
    assert_eq!(exact.source_degree(), 3);
    assert_eq!(exact.rational_degree(), 5);
    assert_eq!(exact.speed_polynomial(), &[r(1), r(0), r(1)]);
    assert!(
        exact
            .curve()
            .weights()
            .iter()
            .all(|weight| weight.partial_cmp(&r(0)).is_some_and(|order| order.is_gt()))
    );

    for parameter in [r(0), q(1, 2), r(1)] {
        let analytic = match parallel.point_at(&parameter, &policy()).unwrap() {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                panic!("analytic PH evaluation was uncertain: {reason:?}")
            }
        };
        let rational = exact.curve().point_at(&parameter, &policy()).unwrap();
        assert_eq!(rational, analytic);
    }
}

#[cfg(feature = "predicates")]
#[test]
fn approximate_ph_materialization_never_populates_the_certified_cache() {
    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    let source =
        QuadraticBezier2::new(p(0, 0), Point2::new(Real::one(), undecidable_zero), p(2, 0));
    let parallel = source.parallel_left(Real::one()).unwrap();

    assert!(matches!(
        parallel.exact_pythagorean_hodograph_offset(&CurveContext::STRICT),
        Ok(Classification::Uncertain(_))
    ));
    assert!(matches!(
        parallel.exact_pythagorean_hodograph_offset(&CurveContext::APPROXIMATE_512),
        Ok(Classification::Decided(Some(_)))
    ));
    assert!(matches!(
        parallel.exact_pythagorean_hodograph_offset(&CurveContext::STRICT),
        Ok(Classification::Uncertain(_))
    ));
}

#[test]
fn nonuniform_rational_line_parallel_materializes_exactly() {
    let source =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 0), p(2, 0)], vec![r(1), r(2), r(3)]).unwrap();
    let parallel = source.parallel_left(r(2)).unwrap();
    let exact = match parallel
        .exact_pythagorean_hodograph_offset(&policy())
        .unwrap()
    {
        Classification::Decided(Some(exact)) => exact,
        Classification::Decided(None) => panic!("rational line was not recognized as PH"),
        Classification::Uncertain(reason) => {
            panic!("rational line PH recognition was uncertain: {reason:?}")
        }
    };

    for parameter in [r(0), q(1, 2), r(1)] {
        let analytic = match parallel.point_at(&parameter, &policy()).unwrap() {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                panic!("rational analytic parallel was uncertain: {reason:?}")
            }
        };
        let materialized = exact.curve().point_at(&parameter, &policy()).unwrap();
        assert_eq!(analytic, materialized);
        assert_eq!(analytic.y(), &r(2));
    }
    assert_eq!(
        parallel.point_at(&q(1, 2), &policy()).unwrap(),
        Classification::Decided(Point2::new(q(5, 4), r(2)))
    );
}

#[test]
fn rational_quarter_circle_parallel_materializes_concentric_exact_curve() {
    // Homogeneous power form `(1-t^2, 2t, 1+t^2)` traces the unit-circle
    // quarter. Its Bernstein weights `[1, 1, 2]` deliberately exercise a
    // nonuniform rational parameterization without an approximate scalar.
    let source =
        RationalQuadraticBezier2::try_new(p(1, 0), p(1, 1), p(0, 1), r(1), r(1), r(2)).unwrap();
    let parallel = source.parallel_left(q(1, 2)).unwrap();
    let exact = match parallel
        .exact_pythagorean_hodograph_offset(&policy())
        .unwrap()
    {
        Classification::Decided(Some(exact)) => exact,
        Classification::Decided(None) => panic!("rational circle was not recognized as PH"),
        Classification::Uncertain(reason) => {
            panic!("rational circle PH recognition was uncertain: {reason:?}")
        }
    };

    for parameter in [r(0), q(1, 2), r(1)] {
        let analytic = match parallel.point_at(&parameter, &policy()).unwrap() {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                panic!("rational circle parallel was uncertain: {reason:?}")
            }
        };
        assert_eq!(
            exact.curve().point_at(&parameter, &policy()).unwrap(),
            analytic
        );
        assert_eq!(
            analytic.x() * analytic.x() + analytic.y() * analytic.y(),
            q(1, 4)
        );
    }
}

#[test]
fn noncircular_rational_ph_parallel_preserves_parameter_and_derivative_exactly() {
    // This is the noncircular polynomial PH curve
    // `P(t) = (t - t^3/3, t^2)` represented with the nonconstant projective
    // factor `W(t) = 1 + t`. Its homogeneous hodograph has speed
    // `(1 + t)^2 (1 + t^2)`.
    let source = RationalBezier2::try_new(
        vec![
            p(0, 0),
            Point2::new(q(1, 5), r(0)),
            Point2::new(q(4, 9), q(1, 9)),
            Point2::new(q(2, 3), q(3, 7)),
            Point2::new(q(2, 3), r(1)),
        ],
        vec![r(1), q(5, 4), q(3, 2), q(7, 4), r(2)],
    )
    .unwrap();
    let parallel = source.parallel_left(q(1, 10)).unwrap();
    let analysis = match parallel.singularity_analysis(&policy()).unwrap() {
        Classification::Decided(analysis) => analysis,
        Classification::Uncertain(reason) => {
            panic!("rational PH singularity analysis was uncertain: {reason:?}")
        }
    };
    assert!(analysis.source_is_regular());
    assert!(analysis.parallel_is_cusp_free());

    let exact = match parallel
        .exact_pythagorean_hodograph_offset(&policy())
        .unwrap()
    {
        Classification::Decided(Some(exact)) => exact,
        Classification::Decided(None) => panic!("noncircular rational PH curve was rejected"),
        Classification::Uncertain(reason) => {
            panic!("noncircular rational PH proof was uncertain: {reason:?}")
        }
    };
    let midpoint = q(1, 2);
    let analytic_point = match parallel.point_at(&midpoint, &policy()).unwrap() {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            panic!("rational PH point was uncertain: {reason:?}")
        }
    };
    assert_eq!(analytic_point, Point2::new(q(227, 600), q(31, 100)));
    assert_eq!(
        exact.curve().point_at(&midpoint, &policy()).unwrap(),
        analytic_point
    );

    let analytic_derivative = match parallel.derivative_at(&midpoint, &policy()).unwrap() {
        Classification::Decided(derivative) => derivative,
        Classification::Uncertain(reason) => {
            panic!("rational PH derivative was uncertain: {reason:?}")
        }
    };
    assert_eq!(analytic_derivative.dx(), &q(327, 500));
    assert_eq!(analytic_derivative.dy(), &q(109, 125));
    assert_eq!(
        exact.curve().derivative_at(&midpoint, &policy()).unwrap(),
        analytic_derivative
    );
}

#[test]
fn exact_ph_materialization_has_no_fixed_bernstein_elevation_limit() {
    // Let `a=t-1/2` and `b=1/16`. The cubic below has hodograph
    // `(a^2-b^2, 2ab)` and speed `(t-1/2)^2 + 1/256`. The speed is strictly
    // positive, but its Bernstein coefficients do not all become positive
    // until degree 65, well beyond the former fixed +32 search window.
    let source = CubicBezier2::new(
        p(0, 0),
        Point2::new(q(21, 256), q(-1, 48)),
        Point2::new(q(-1, 384), q(-1, 48)),
        Point2::new(q(61, 768), r(0)),
    );
    let parallel = source.parallel_left(q(1, 10)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let exact = match parallel
            .exact_pythagorean_hodograph_offset(&policy)
            .unwrap()
        {
            Classification::Decided(Some(exact)) => exact,
            Classification::Decided(None) => panic!("strictly regular PH curve was rejected"),
            Classification::Uncertain(reason) => {
                panic!("PH degree elevation was uncertain: {reason:?}")
            }
        };
        assert_eq!(exact.rational_degree(), 65);
        for parameter in [r(0), q(1, 2), r(1)] {
            let analytic = match parallel.point_at(&parameter, &policy).unwrap() {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    panic!("analytic PH parallel was uncertain: {reason:?}")
                }
            };
            assert_eq!(
                exact.curve().point_at(&parameter, &policy).unwrap(),
                analytic
            );
        }
    }
}

#[test]
fn symmetric_algebraic_quarter_circle_parallel_is_exact_under_both_policies() {
    let half_sqrt_two = (r(2).sqrt().unwrap() / r(2)).unwrap();
    let source =
        RationalQuadraticBezier2::try_unit_end_weights(p(1, 0), p(1, 1), p(0, 1), half_sqrt_two)
            .unwrap();
    let parallel = source.parallel_left(q(1, 2)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let exact = match parallel
            .exact_pythagorean_hodograph_offset(&policy)
            .unwrap()
        {
            Classification::Decided(Some(exact)) => exact,
            Classification::Decided(None) => panic!("algebraic rational circle was not PH"),
            Classification::Uncertain(reason) => {
                panic!("algebraic rational circle PH proof was uncertain: {reason:?}")
            }
        };

        assert_eq!(exact.rational_degree(), 2);
        assert_eq!(
            exact.curve().control_points(),
            &[
                Point2::new(q(1, 2), r(0)),
                Point2::new(q(1, 2), q(1, 2)),
                Point2::new(r(0), q(1, 2))
            ]
        );
        assert_eq!(
            exact.curve().weights(),
            &[r(1), (r(2).sqrt().unwrap() / r(2)).unwrap(), r(1)]
        );
        assert_eq!(
            exact.curve().point_at(&r(0), &policy).unwrap(),
            Point2::new(q(1, 2), r(0))
        );
        assert_eq!(
            exact.curve().point_at(&r(1), &policy).unwrap(),
            Point2::new(r(0), q(1, 2))
        );
    }
}

#[test]
fn circular_parallel_materializes_radius_collapse_and_reversal_exactly() {
    let source =
        RationalQuadraticBezier2::try_new(p(1, 0), p(1, 1), p(0, 1), r(1), r(1), r(2)).unwrap();
    for (distance, expected) in [
        (r(1), vec![p(0, 0), p(0, 0), p(0, 0)]),
        (r(2), vec![p(-1, 0), p(-1, -1), p(0, -1)]),
    ] {
        let parallel = source.parallel_left(distance).unwrap();
        let exact = match parallel
            .exact_pythagorean_hodograph_offset(&CurveContext::STRICT)
            .unwrap()
        {
            Classification::Decided(Some(exact)) => exact,
            Classification::Decided(None) => panic!("circular parallel was not exact"),
            Classification::Uncertain(reason) => {
                panic!("circular parallel materialization was uncertain: {reason:?}")
            }
        };
        assert_eq!(exact.rational_degree(), 2);
        assert_eq!(exact.curve().control_points(), expected);
        assert_eq!(exact.curve().weights(), &[r(1), r(1), r(2)]);
    }
}

#[test]
fn rational_parallel_rejects_projective_denominator_boundary() {
    let source =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 1), p(2, 0)], vec![r(1), r(-1), r(1)]).unwrap();
    let analysis = source
        .parallel_left(r(1))
        .unwrap()
        .singularity_analysis(&policy())
        .unwrap();
    assert_eq!(
        analysis,
        Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)
    );
}

#[test]
fn exact_parallel_reversal_preserves_image_and_reverses_parameter_derivative() {
    let source = CubicBezier2::new(p(0, 0), p(1, 2), p(3, 2), p(4, 0));
    let parallel = source.parallel_left(q(1, 3)).unwrap();
    let reversed = parallel.reversed();

    assert_eq!(reversed.distance(), &q(-1, 3));
    assert_eq!(reversed.reversed(), parallel);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for parameter in [r(0), q(1, 4), r(1)] {
            let complement = r(1) - &parameter;
            let expected_point = parallel.point_at(&complement, &policy).unwrap();
            assert_eq!(
                reversed.point_at(&parameter, &policy).unwrap(),
                expected_point
            );

            let Classification::Decided(expected_derivative) =
                parallel.derivative_at(&complement, &policy).unwrap()
            else {
                panic!("source parallel derivative was uncertain");
            };
            let Classification::Decided(actual_derivative) =
                reversed.derivative_at(&parameter, &policy).unwrap()
            else {
                panic!("reversed parallel derivative was uncertain");
            };
            assert_real_eq(actual_derivative.dx(), &(-expected_derivative.dx().clone()));
            assert_real_eq(actual_derivative.dy(), &(-expected_derivative.dy().clone()));
        }
    }
}

#[test]
fn exact_parallel_split_preserves_parameter_map_and_chain_derivative() {
    let source = CubicBezier2::new(p(0, 0), p(1, 2), p(3, 2), p(4, 0));
    let parallel = source.parallel_left(q(1, 3)).unwrap();
    let split_parameter = q(1, 3);

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let Classification::Decided((left, right)) =
            parallel.split_at_exact(&split_parameter, &policy).unwrap()
        else {
            panic!("interior parallel split was uncertain");
        };
        let local = q(1, 2);
        let left_global = q(1, 6);
        let right_global = q(2, 3);
        assert_eq!(
            left.point_at(&local, &policy).unwrap(),
            parallel.point_at(&left_global, &policy).unwrap()
        );
        assert_eq!(
            right.point_at(&local, &policy).unwrap(),
            parallel.point_at(&right_global, &policy).unwrap()
        );

        let Classification::Decided(left_derivative) = left.derivative_at(&local, &policy).unwrap()
        else {
            panic!("left split derivative was uncertain");
        };
        let Classification::Decided(source_left_derivative) =
            parallel.derivative_at(&left_global, &policy).unwrap()
        else {
            panic!("source left derivative was uncertain");
        };
        let expected_left_derivative = source_left_derivative.scaled(&q(1, 3));
        assert_real_eq(left_derivative.dx(), expected_left_derivative.dx());
        assert_real_eq(left_derivative.dy(), expected_left_derivative.dy());

        let Classification::Decided(right_derivative) =
            right.derivative_at(&local, &policy).unwrap()
        else {
            panic!("right split derivative was uncertain");
        };
        let Classification::Decided(source_right_derivative) =
            parallel.derivative_at(&right_global, &policy).unwrap()
        else {
            panic!("source right derivative was uncertain");
        };
        let expected_right_derivative = source_right_derivative.scaled(&q(2, 3));
        assert_real_eq(right_derivative.dx(), expected_right_derivative.dx());
        assert_real_eq(right_derivative.dy(), expected_right_derivative.dy());

        assert_eq!(
            parallel.split_at_exact(&r(0), &policy).unwrap(),
            Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)
        );
        assert_eq!(
            parallel.split_at_exact(&r(1), &policy).unwrap(),
            Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)
        );
    }
}

#[test]
fn rational_parallel_subcurve_preserves_parameter_map_and_chain_derivative() {
    let source =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 0), p(2, 0)], vec![r(1), r(2), r(3)]).unwrap();
    let parallel = source.parallel_left(r(2)).unwrap();
    let start = q(1, 4);
    let end = q(3, 4);

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let Classification::Decided(subcurve) = parallel
            .subcurve_between_exact(&start, &end, &policy)
            .unwrap()
        else {
            panic!("rational parallel subcurve was uncertain");
        };
        for (local, global) in [
            (r(0), start.clone()),
            (q(1, 2), q(1, 2)),
            (r(1), end.clone()),
        ] {
            assert_eq!(
                subcurve.point_at(&local, &policy).unwrap(),
                parallel.point_at(&global, &policy).unwrap()
            );
        }

        let Classification::Decided(subcurve_derivative) =
            subcurve.derivative_at(&q(1, 2), &policy).unwrap()
        else {
            panic!("rational parallel subcurve derivative was uncertain");
        };
        let Classification::Decided(source_derivative) =
            parallel.derivative_at(&q(1, 2), &policy).unwrap()
        else {
            panic!("rational source parallel derivative was uncertain");
        };
        let expected_derivative = source_derivative.scaled(&q(1, 2));
        assert_real_eq(subcurve_derivative.dx(), expected_derivative.dx());
        assert_real_eq(subcurve_derivative.dy(), expected_derivative.dy());

        assert_eq!(
            parallel
                .subcurve_between_exact(&start, &start, &policy)
                .unwrap(),
            Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)
        );
    }
}

#[test]
fn exact_parallel_conservative_bounds_cover_both_offset_sides() {
    let source =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 0), p(2, 0)], vec![r(1), r(2), r(3)]).unwrap();
    for distance in [r(-2), r(2)] {
        let parallel = source.parallel_left(distance).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(bounds) = parallel.conservative_bounds(&policy).unwrap()
            else {
                panic!("parallel bounds were uncertain");
            };
            assert_eq!(bounds.min(), &p(-2, -2));
            assert_eq!(bounds.max(), &p(4, 2));
            for parameter in [r(0), q(1, 4), q(1, 2), q(3, 4), r(1)] {
                let Classification::Decided(point) =
                    parallel.point_at(&parameter, &policy).unwrap()
                else {
                    panic!("parallel point was uncertain");
                };
                assert_eq!(
                    bounds.contains_point(&point, &policy),
                    Classification::Decided(true)
                );
            }
        }
    }
}

#[test]
fn exact_parallel_point_incidence_rejects_the_opposite_normal_branch() {
    let source = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0));
    let parallel = source.parallel_left(r(1)).unwrap();
    let right_parallel = source.parallel_left(r(-1)).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            parallel.point_incidence(&p(1, 1), &policy).unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(vec![
                BezierParameter2::Exact(q(1, 2))
            ]))
        );
        assert_eq!(
            parallel.point_incidence(&p(1, -1), &policy).unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(Vec::new()))
        );
        assert_eq!(
            parallel.contains_point(&p(1, 1), &policy).unwrap(),
            Classification::Decided(true)
        );
        assert_eq!(
            parallel.contains_point(&p(1, -1), &policy).unwrap(),
            Classification::Decided(false)
        );
        assert_eq!(
            right_parallel.point_incidence(&p(1, -1), &policy).unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(vec![
                BezierParameter2::Exact(q(1, 2))
            ]))
        );
        assert_eq!(
            right_parallel.point_incidence(&p(1, 1), &policy).unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(Vec::new()))
        );
    }
}

#[cfg(feature = "predicates")]
#[test]
fn parallel_point_incidence_uses_approximate_512_only_as_a_terminal_decision() {
    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    let source = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0));
    let parallel = source.parallel_left(undecidable_zero).unwrap();

    assert_eq!(
        parallel.point_incidence(&p(1, 0), &CurveContext::STRICT),
        Ok(Classification::Uncertain(
            hypercurve::UncertaintyReason::RealSign
        ))
    );
    assert_eq!(
        parallel.point_incidence(&p(1, 0), &CurveContext::APPROXIMATE_512),
        Ok(Classification::Decided(
            BezierParallelIncidence2::Parameters(vec![BezierParameter2::Exact(q(1, 2))])
        ))
    );
}

#[test]
fn exact_parallel_point_incidence_retains_algebraic_parameters() {
    // `x(t)=t+t^2` reaches x=1 at the nonrepresented root
    // `(-1+sqrt(5))/2`; its tangent is regular over the complete domain.
    let source = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(2, 0));
    let parallel = source.parallel_left(r(1)).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let Classification::Decided(BezierParallelIncidence2::Parameters(parameters)) =
            parallel.point_incidence(&p(1, 1), &policy).unwrap()
        else {
            panic!("algebraic parallel incidence was not decided");
        };
        let [BezierParameter2::Algebraic(parameter)] = parameters.as_slice() else {
            panic!("parallel incidence did not retain its algebraic parameter");
        };
        assert_eq!(parameter.polynomial().degree(), 2);

        assert_eq!(
            parallel.point_incidence(&p(1, -1), &policy).unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(Vec::new()))
        );
    }
}

#[test]
fn rational_parallel_point_incidence_preserves_projective_parameterization() {
    let source =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 0), p(2, 0)], vec![r(1), r(2), r(3)]).unwrap();
    let parallel = source.parallel_left(r(2)).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            parallel
                .point_incidence(&Point2::new(q(5, 4), r(2)), &policy)
                .unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(vec![
                BezierParameter2::Exact(q(1, 2))
            ]))
        );
        assert_eq!(
            parallel
                .point_incidence(&Point2::new(q(5, 4), r(-2)), &policy)
                .unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(Vec::new()))
        );
    }
}

#[test]
fn collapsed_circular_parallel_reports_entire_curve_point_incidence() {
    let source =
        RationalQuadraticBezier2::try_new(p(1, 0), p(1, 1), p(0, 1), r(1), r(1), r(2)).unwrap();
    let parallel = source.parallel_left(r(1)).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            parallel.point_incidence(&p(0, 0), &policy).unwrap(),
            Classification::Decided(BezierParallelIncidence2::EntireCurve)
        );
        assert_eq!(
            parallel.point_incidence(&p(1, 0), &policy).unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(Vec::new()))
        );
    }
}

#[test]
fn parallel_point_incidence_rejects_projective_poles_and_source_singularities() {
    let projective =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 1), p(2, 0)], vec![r(1), r(-1), r(1)])
            .unwrap()
            .parallel_left(r(1))
            .unwrap();
    let singular = QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0))
        .parallel_left(r(1))
        .unwrap();
    let zero_distance_singular = QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0))
        .parallel_left(r(0))
        .unwrap();
    let zero_distance_constant = QuadraticBezier2::new(p(3, 4), p(3, 4), p(3, 4))
        .parallel_left(r(0))
        .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            projective.point_incidence(&p(0, 0), &policy).unwrap(),
            Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)
        );
        assert_eq!(
            singular.point_incidence(&p(0, 1), &policy).unwrap(),
            Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)
        );
        assert_eq!(
            zero_distance_singular
                .point_incidence(&p(0, 0), &policy)
                .unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(vec![
                BezierParameter2::Exact(r(0))
            ]))
        );
        assert_eq!(
            zero_distance_constant
                .point_incidence(&p(3, 4), &policy)
                .unwrap(),
            Classification::Decided(BezierParallelIncidence2::EntireCurve)
        );
    }
}

#[test]
fn supporting_line_incidence_distinguishes_selected_and_opposite_parallels() {
    let source = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0));
    let left = source.parallel_left(r(1)).unwrap();
    let right = source.parallel_left(r(-1)).unwrap();
    let upper = LineSeg2::try_new(p(0, 1), p(2, 1)).unwrap();
    let lower = LineSeg2::try_new(p(0, -1), p(2, -1)).unwrap();
    let vertical = LineSeg2::try_new(p(1, -2), p(1, 2)).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            left.supporting_line_incidence(&upper, &policy).unwrap(),
            Classification::Decided(BezierParallelIncidence2::EntireCurve)
        );
        assert_eq!(
            left.supporting_line_incidence(&lower, &policy).unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(Vec::new()))
        );
        assert_eq!(
            right.supporting_line_incidence(&lower, &policy).unwrap(),
            Classification::Decided(BezierParallelIncidence2::EntireCurve)
        );
        assert_eq!(
            right.supporting_line_incidence(&upper, &policy).unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(Vec::new()))
        );
        assert_eq!(
            left.supporting_line_incidence(&vertical, &policy).unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(vec![
                BezierParameter2::Exact(q(1, 2))
            ]))
        );
    }
}

#[test]
fn finite_supporting_line_incidence_filters_the_squared_opposite_branch() {
    let source = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0));
    let left = source.parallel_left(r(1)).unwrap();
    let right = source.parallel_left(r(-1)).unwrap();
    let diagonal = LineSeg2::try_new(p(0, 0), p(2, 2)).unwrap();
    let reversed = LineSeg2::try_new(p(2, 2), p(0, 0)).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let selected = Classification::Decided(BezierParallelIncidence2::Parameters(vec![
            BezierParameter2::Exact(q(1, 2)),
        ]));
        assert_eq!(
            left.supporting_line_incidence(&diagonal, &policy).unwrap(),
            selected
        );
        assert_eq!(
            left.supporting_line_incidence(&reversed, &policy).unwrap(),
            selected
        );
        assert_eq!(
            right.supporting_line_incidence(&diagonal, &policy).unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(Vec::new()))
        );
    }
}

#[test]
fn supporting_line_incidence_retains_polynomial_and_rational_algebraic_parameters() {
    // `x(t)=t+t^2` reaches x=1 at `(-1+sqrt(5))/2`.
    let polynomial = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let rational = RationalBezier2::try_new(
        vec![p(0, 0), Point2::new(q(1, 2), r(0)), p(2, 0)],
        vec![r(1), r(1), r(1)],
    )
    .unwrap()
    .parallel_left(r(1))
    .unwrap();
    let vertical = LineSeg2::try_new(p(1, -2), p(1, 2)).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for parallel in [&polynomial, &rational] {
            let Classification::Decided(BezierParallelIncidence2::Parameters(parameters)) =
                parallel
                    .supporting_line_incidence(&vertical, &policy)
                    .unwrap()
            else {
                panic!("algebraic supporting-line incidence was not decided");
            };
            let [BezierParameter2::Algebraic(parameter)] = parameters.as_slice() else {
                panic!("supporting-line incidence did not retain its algebraic parameter");
            };
            assert_eq!(parameter.polynomial().degree(), 2);
        }
    }
}

#[test]
fn zero_distance_supporting_line_incidence_keeps_stationary_source_contact() {
    let parallel = QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0))
        .parallel_left(r(0))
        .unwrap();
    let vertical = LineSeg2::try_new(p(0, -1), p(0, 1)).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            parallel
                .supporting_line_incidence(&vertical, &policy)
                .unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(vec![
                BezierParameter2::Exact(r(0))
            ]))
        );
    }
}

#[cfg(feature = "predicates")]
#[test]
fn supporting_line_incidence_uses_approximate_512_only_as_a_terminal_decision() {
    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    let parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
        .parallel_left(undecidable_zero)
        .unwrap();
    let vertical = LineSeg2::try_new(p(1, -1), p(1, 1)).unwrap();

    assert_eq!(
        parallel.supporting_line_incidence(&vertical, &CurveContext::STRICT),
        Ok(Classification::Uncertain(
            hypercurve::UncertaintyReason::RealSign
        ))
    );
    assert_eq!(
        parallel.supporting_line_incidence(&vertical, &CurveContext::APPROXIMATE_512),
        Ok(Classification::Decided(
            BezierParallelIncidence2::Parameters(vec![BezierParameter2::Exact(q(1, 2))])
        ))
    );
}

#[test]
fn supporting_line_incidence_rejects_projective_poles_and_source_singularities() {
    let projective =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 1), p(2, 0)], vec![r(1), r(-1), r(1)])
            .unwrap()
            .parallel_left(r(1))
            .unwrap();
    let singular = QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0))
        .parallel_left(r(1))
        .unwrap();
    let line = LineSeg2::try_new(p(-1, 1), p(2, 1)).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            projective
                .supporting_line_incidence(&line, &policy)
                .unwrap(),
            Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)
        );
        assert_eq!(
            singular.supporting_line_incidence(&line, &policy).unwrap(),
            Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)
        );
    }
}

#[test]
fn parallel_rational_intersection_candidates_retain_both_finite_parameters() {
    let parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let vertical = RationalBezier2::try_new(vec![p(1, 0), p(1, 2)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            parallel
                .intersection_candidates(&vertical, &policy)
                .unwrap(),
            Classification::Decided(BezierParallelIntersectionCandidates2::Candidates {
                parallel_parameters: vec![BezierParameter2::Exact(q(1, 2))],
                other_parameters: vec![BezierParameter2::Exact(q(1, 2))],
            })
        );
    }
}

#[test]
fn parallel_rational_intersection_candidates_retain_algebraic_projection() {
    let parallel = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let vertical = RationalBezier2::try_new(vec![p(1, 0), p(1, 2)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let Classification::Decided(BezierParallelIntersectionCandidates2::Candidates {
            parallel_parameters,
            other_parameters,
        }) = parallel
            .intersection_candidates(&vertical, &policy)
            .unwrap()
        else {
            panic!("parallel/rational algebraic projections were not decided");
        };
        let [BezierParameter2::Algebraic(parameter)] = parallel_parameters.as_slice() else {
            panic!("parallel projection did not retain its algebraic parameter");
        };
        assert!(parameter.polynomial().degree() >= 2);
        assert_eq!(other_parameters, vec![BezierParameter2::Exact(q(1, 2))]);
    }
}

#[test]
fn parallel_rational_intersection_candidates_report_disjoint_and_shared_components() {
    let parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let disjoint = RationalBezier2::try_new(vec![p(10, 0), p(10, 2)], vec![r(1), r(1)]).unwrap();
    let coincident = RationalBezier2::try_new(vec![p(0, 1), p(2, 1)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            parallel
                .intersection_candidates(&disjoint, &policy)
                .unwrap(),
            Classification::Decided(BezierParallelIntersectionCandidates2::NoIntersection)
        );
        assert_eq!(
            parallel
                .intersection_candidates(&coincident, &policy)
                .unwrap(),
            Classification::Decided(BezierParallelIntersectionCandidates2::DegenerateResultant)
        );
    }
}

#[test]
fn parallel_rational_intersections_saturate_rootless_homogeneous_axis_content() {
    let factored_parallel = rootless_homogeneous_factor_parabola()
        .parallel_left(r(1))
        .unwrap();
    let ordinary_parallel = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(1, 1))
        .parallel_left(r(1))
        .unwrap();
    let ordinary_vertical =
        RationalBezier2::try_new(vec![p(0, 0), p(0, 2)], vec![r(1), r(1)]).unwrap();
    let factored_vertical = rootless_homogeneous_factor_vertical();

    for (parallel, vertical) in [
        (&factored_parallel, &ordinary_vertical),
        (&ordinary_parallel, &factored_vertical),
    ] {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(BezierParallelIntersectionCandidates2::Candidates {
                parallel_parameters,
                other_parameters,
            }) = parallel.intersection_candidates(vertical, &policy).unwrap()
            else {
                panic!("rootless homogeneous axis content was not saturated");
            };
            assert_eq!(parallel_parameters.len(), 2);
            assert_eq!(other_parameters.len(), 2);
            assert!(parallel_parameters.contains(&BezierParameter2::Exact(r(0))));
            assert!(other_parameters.contains(&BezierParameter2::Exact(q(1, 2))));
            assert!(other_parameters.contains(&BezierParameter2::Exact(q(5, 8))));

            let Classification::Decided(BezierParallelIntersectionContacts2::Contacts(contacts)) =
                parallel.intersection_contacts(vertical, &policy).unwrap()
            else {
                panic!("rootless homogeneous axis content did not replay completely");
            };
            assert_eq!(contacts.len(), 2);
            assert!(contacts.iter().any(|contact| {
                contact.point() == &RationalBezierIntersectionPointEvidence2::Exact(p(0, 1))
            }));
            assert!(contacts.iter().any(|contact| {
                contact.point()
                    == &RationalBezierIntersectionPointEvidence2::Exact(Point2::new(r(0), q(5, 4)))
            }));
        }
    }
}

#[test]
fn parallel_rational_axis_saturation_retains_in_domain_projective_base_points() {
    let parallel = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(1, 1))
        .parallel_left(r(1))
        .unwrap();
    let rootful = rootful_homogeneous_factor_vertical();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            parallel.intersection_candidates(&rootful, &policy).unwrap(),
            Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)
        );
    }
}

#[cfg(feature = "predicates")]
#[test]
fn parallel_rational_candidates_use_approximate_512_only_as_a_terminal_decision() {
    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    let parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
        .parallel_left(undecidable_zero)
        .unwrap();
    let vertical = RationalBezier2::try_new(vec![p(1, -1), p(1, 1)], vec![r(1), r(1)]).unwrap();

    assert_eq!(
        parallel.intersection_candidates(&vertical, &CurveContext::STRICT),
        Ok(Classification::Uncertain(
            hypercurve::UncertaintyReason::RealSign
        ))
    );
    assert_eq!(
        parallel.intersection_candidates(&vertical, &CurveContext::APPROXIMATE_512),
        Ok(Classification::Decided(
            BezierParallelIntersectionCandidates2::Candidates {
                parallel_parameters: vec![BezierParameter2::Exact(q(1, 2))],
                other_parameters: vec![BezierParameter2::Exact(q(1, 2))],
            }
        ))
    );
}

#[test]
fn parallel_rational_candidates_reject_projective_poles_and_source_singularities() {
    let projective_source =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 1), p(2, 0)], vec![r(1), r(-1), r(1)])
            .unwrap()
            .parallel_left(r(1))
            .unwrap();
    let singular_source = QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0))
        .parallel_left(r(1))
        .unwrap();
    let projective_other =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 1), p(2, 0)], vec![r(1), r(-1), r(1)]).unwrap();
    let finite_other = RationalBezier2::try_new(vec![p(0, 1), p(2, 1)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            projective_source
                .intersection_candidates(&finite_other, &policy)
                .unwrap(),
            Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)
        );
        assert_eq!(
            singular_source
                .intersection_candidates(&finite_other, &policy)
                .unwrap(),
            Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)
        );
        assert_eq!(
            QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
                .parallel_left(r(1))
                .unwrap()
                .intersection_candidates(&projective_other, &policy)
                .unwrap(),
            Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)
        );
    }
}

#[test]
fn zero_distance_parallel_candidates_keep_stationary_source_intersection() {
    let parallel = QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0))
        .parallel_left(r(0))
        .unwrap();
    let vertical = RationalBezier2::try_new(vec![p(0, -1), p(0, 1)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            parallel
                .intersection_candidates(&vertical, &policy)
                .unwrap(),
            Classification::Decided(BezierParallelIntersectionCandidates2::Candidates {
                parallel_parameters: vec![BezierParameter2::Exact(r(0))],
                other_parameters: vec![BezierParameter2::Exact(q(1, 2))],
            })
        );
    }
}

#[test]
fn parallel_rational_contacts_replay_exact_pair_and_transversality() {
    let parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let vertical = RationalBezier2::try_new(vec![p(1, 0), p(1, 2)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let Classification::Decided(BezierParallelIntersectionContacts2::Contacts(contacts)) =
            parallel.intersection_contacts(&vertical, &policy).unwrap()
        else {
            panic!("exact parallel/rational contact was not decided");
        };
        assert_eq!(contacts.len(), 1);
        assert_eq!(
            contacts[0].parallel_parameter(),
            &BezierParameter2::Exact(q(1, 2))
        );
        assert_eq!(
            contacts[0].other_parameter(),
            &BezierParameter2::Exact(q(1, 2))
        );
        assert_eq!(
            contacts[0].point(),
            &RationalBezierIntersectionPointEvidence2::Exact(p(1, 1))
        );
        assert!(contacts[0].is_certified_transverse());
    }
}

#[test]
fn parallel_rational_contacts_reject_the_squared_opposite_branch() {
    let parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let vertical = RationalBezier2::try_new(vec![p(1, -2), p(1, 2)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let Classification::Decided(BezierParallelIntersectionContacts2::Contacts(contacts)) =
            parallel.intersection_contacts(&vertical, &policy).unwrap()
        else {
            panic!("selected parallel branch was not decided");
        };
        assert_eq!(contacts.len(), 1);
        assert_eq!(
            contacts[0].parallel_parameter(),
            &BezierParameter2::Exact(q(1, 2))
        );
        assert_eq!(
            contacts[0].other_parameter(),
            &BezierParameter2::Exact(q(3, 4))
        );
        assert_eq!(
            contacts[0].point(),
            &RationalBezierIntersectionPointEvidence2::Exact(p(1, 1))
        );
    }
}

#[test]
fn parallel_rational_contacts_preserve_negative_distance_and_weight_orientation() {
    let right_parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
        .parallel_left(r(-1))
        .unwrap();
    let vertical = RationalBezier2::try_new(vec![p(1, -2), p(1, 2)], vec![r(-1), r(-1)]).unwrap();
    let rational_source = RationalBezier2::try_new(vec![p(0, 0), p(2, 0)], vec![r(-1), r(-1)])
        .unwrap()
        .parallel_left(r(1))
        .unwrap();
    let positive_vertical =
        RationalBezier2::try_new(vec![p(1, -2), p(1, 2)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let Classification::Decided(BezierParallelIntersectionContacts2::Contacts(contacts)) =
            right_parallel
                .intersection_contacts(&vertical, &policy)
                .unwrap()
        else {
            panic!("negative-distance/weight contact was not decided");
        };
        assert_eq!(contacts.len(), 1);
        assert_eq!(
            contacts[0].other_parameter(),
            &BezierParameter2::Exact(q(1, 4))
        );
        assert_eq!(
            contacts[0].point(),
            &RationalBezierIntersectionPointEvidence2::Exact(p(1, -1))
        );

        let Classification::Decided(BezierParallelIntersectionContacts2::Contacts(contacts)) =
            rational_source
                .intersection_contacts(&positive_vertical, &policy)
                .unwrap()
        else {
            panic!("negative source-weight contact was not decided");
        };
        assert_eq!(contacts.len(), 1);
        assert_eq!(
            contacts[0].other_parameter(),
            &BezierParameter2::Exact(q(3, 4))
        );
        assert_eq!(
            contacts[0].point(),
            &RationalBezierIntersectionPointEvidence2::Exact(p(1, 1))
        );
    }
}

#[test]
fn parallel_rational_contacts_replay_one_algebraic_parameter_exactly() {
    let parallel = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let vertical = RationalBezier2::try_new(vec![p(1, 0), p(1, 2)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let Classification::Decided(BezierParallelIntersectionContacts2::Contacts(contacts)) =
            parallel.intersection_contacts(&vertical, &policy).unwrap()
        else {
            panic!("algebraic/exact parallel contact was not decided");
        };
        assert_eq!(contacts.len(), 1);
        assert!(matches!(
            contacts[0].parallel_parameter(),
            BezierParameter2::Algebraic(_)
        ));
        assert_eq!(
            contacts[0].other_parameter(),
            &BezierParameter2::Exact(q(1, 2))
        );
        assert_eq!(
            contacts[0].point(),
            &RationalBezierIntersectionPointEvidence2::Exact(p(1, 1))
        );
    }
}

#[cfg(feature = "predicates")]
#[test]
fn parallel_rational_contacts_replay_identical_algebraic_parameters() {
    let parallel = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let target = RationalBezier2::try_new(
        vec![p(1, 0), Point2::new(r(1), q(1, 2)), p(1, 2)],
        vec![r(1), r(1), r(1)],
    )
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let Classification::Decided(BezierParallelIntersectionContacts2::Contacts(contacts)) =
            parallel.intersection_contacts(&target, &policy).unwrap()
        else {
            panic!("coupled equal-algebraic parallel contact was not decided");
        };
        assert_eq!(contacts.len(), 1);
        assert!(matches!(
            (
                contacts[0].parallel_parameter(),
                contacts[0].other_parameter()
            ),
            (
                BezierParameter2::Algebraic(_),
                BezierParameter2::Algebraic(_)
            )
        ));
        assert!(matches!(
            contacts[0].point(),
            RationalBezierIntersectionPointEvidence2::Algebraic(_)
        ));
    }
}

#[cfg(feature = "predicates")]
#[test]
fn parallel_rational_contacts_lift_coupled_distinct_algebraic_parameters() {
    let source = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(1, 1));
    let parallel = source.parallel_left(r(0)).unwrap();
    let target = RationalBezier2::try_new(vec![p(0, 1), p(2, 0)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let replay = parallel.intersection_contacts(&target, &policy).unwrap();
        let Classification::Decided(BezierParallelIntersectionContacts2::Contacts(contacts)) =
            replay.clone()
        else {
            panic!("nullity-one coupled algebraic contact was not lifted: {replay:?}");
        };
        assert_eq!(contacts.len(), 1);
        assert!(matches!(
            contacts[0].parallel_parameter(),
            BezierParameter2::Algebraic(_)
        ));
        assert!(matches!(
            contacts[0].other_parameter(),
            BezierParameter2::Algebraic(_)
        ));
        assert_ne!(
            contacts[0].parallel_parameter(),
            contacts[0].other_parameter()
        );
        assert!(matches!(
            contacts[0].point(),
            RationalBezierIntersectionPointEvidence2::Algebraic(_)
        ));
    }
}

#[cfg(feature = "predicates")]
#[test]
fn parallel_rational_lift_pairs_multiple_algebraic_projections_without_cross_product() {
    let source = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(1, 1));
    let parallel = source.parallel_left(r(0)).unwrap();
    let target = RationalBezier2::try_new(
        vec![Point2::new(r(0), q(-1, 5)), Point2::new(r(2), q(9, 5))],
        vec![r(1), r(1)],
    )
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let replay = parallel.intersection_contacts(&target, &policy).unwrap();
        let Classification::Decided(BezierParallelIntersectionContacts2::Contacts(contacts)) =
            replay.clone()
        else {
            panic!("multiple coupled algebraic contacts were not paired: {replay:?}");
        };
        assert_eq!(contacts.len(), 2);
        assert!(contacts.iter().all(|contact| {
            matches!(contact.parallel_parameter(), BezierParameter2::Algebraic(_))
                && matches!(contact.other_parameter(), BezierParameter2::Algebraic(_))
        }));
        assert_ne!(contacts[0].point(), contacts[1].point());
    }
}

#[cfg(feature = "predicates")]
#[test]
fn parallel_rational_contacts_replay_selected_branch_through_algebraic_lift() {
    let source = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(1, 1));
    let parallel = source.parallel_left(r(1)).unwrap();
    let target = RationalBezier2::try_new(vec![p(-1, 1), p(1, 1)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let replay = parallel.intersection_contacts(&target, &policy).unwrap();
        let Classification::Decided(BezierParallelIntersectionContacts2::Contacts(contacts)) =
            replay.clone()
        else {
            panic!("coupled algebraic branch replay was not complete: {replay:?}");
        };
        assert_eq!(contacts.len(), 2);
        assert!(contacts.iter().any(|contact| {
            contact.parallel_parameter() == &BezierParameter2::Exact(r(0))
                && contact.other_parameter() == &BezierParameter2::Exact(q(1, 2))
        }));
        assert!(contacts.iter().any(|contact| {
            matches!(contact.parallel_parameter(), BezierParameter2::Algebraic(_))
                && matches!(contact.other_parameter(), BezierParameter2::Algebraic(_))
        }));
    }
}

#[test]
fn parallel_rational_contacts_handle_higher_nullity_algebraic_fibers() {
    let parallel = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let target =
        RationalBezier2::try_new(vec![p(1, 0), p(1, 0), p(1, 2)], vec![r(1), r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let replay = parallel.intersection_contacts(&target, &policy).unwrap();
        #[cfg(feature = "predicates")]
        {
            let Classification::Decided(BezierParallelIntersectionContacts2::Contacts(contacts)) =
                replay.clone()
            else {
                panic!("higher-nullity algebraic fiber was not replayed: {replay:?}");
            };
            assert_eq!(contacts.len(), 1);
            assert!(matches!(
                contacts[0].parallel_parameter(),
                BezierParameter2::Algebraic(_)
            ));
            assert!(matches!(
                contacts[0].other_parameter(),
                BezierParameter2::Algebraic(_)
            ));
            assert!(matches!(
                contacts[0].point(),
                RationalBezierIntersectionPointEvidence2::Algebraic(_)
            ));
        }
        #[cfg(not(feature = "predicates"))]
        {
            assert!(matches!(
                replay,
                Classification::Decided(BezierParallelIntersectionContacts2::Incomplete {
                    contacts,
                    ..
                }) if contacts.is_empty()
            ));
        }
    }
}

#[test]
fn zero_distance_parallel_contacts_keep_stationary_source_contact() {
    let parallel = QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0))
        .parallel_left(r(0))
        .unwrap();
    let vertical = RationalBezier2::try_new(vec![p(0, -1), p(0, 1)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let Classification::Decided(BezierParallelIntersectionContacts2::Contacts(contacts)) =
            parallel.intersection_contacts(&vertical, &policy).unwrap()
        else {
            panic!("zero-distance stationary source contact was not decided");
        };
        assert_eq!(contacts.len(), 1);
        assert_eq!(
            contacts[0].parallel_parameter(),
            &BezierParameter2::Exact(r(0))
        );
        assert_eq!(
            contacts[0].point(),
            &RationalBezierIntersectionPointEvidence2::Exact(p(0, 0))
        );
        assert!(!contacts[0].is_certified_transverse());
    }
}

#[test]
fn parallel_rational_contacts_resolve_selected_and_opposite_shared_components() {
    let parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let disjoint = RationalBezier2::try_new(vec![p(10, 0), p(10, 2)], vec![r(1), r(1)]).unwrap();
    let coincident = RationalBezier2::try_new(vec![p(0, 1), p(2, 1)], vec![r(1), r(1)]).unwrap();
    let opposite = RationalBezier2::try_new(vec![p(0, -1), p(2, -1)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            parallel.intersection_contacts(&disjoint, &policy).unwrap(),
            Classification::Decided(BezierParallelIntersectionContacts2::NoIntersection)
        );
        let Classification::Decided(BezierParallelIntersectionContacts2::Overlap(overlap)) =
            parallel
                .intersection_contacts(&coincident, &policy)
                .unwrap()
        else {
            panic!("selected line parallel did not retain its shared component");
        };
        assert_eq!(
            overlap.first_range().exact_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );
        assert_eq!(
            overlap.second_range().exact_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );
        assert_eq!(
            overlap.orientation(),
            RationalBezierOverlapOrientation2::Same
        );
        assert_eq!(
            parallel.intersection_contacts(&opposite, &policy).unwrap(),
            Classification::Decided(BezierParallelIntersectionContacts2::NoIntersection)
        );
    }
}

#[test]
fn parallel_rational_contacts_retain_partial_and_reversed_overlap_ranges() {
    let parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let partial = RationalBezier2::try_new(vec![p(1, 1), p(3, 1)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let Classification::Decided(BezierParallelIntersectionContacts2::Overlap(overlap)) =
            parallel.intersection_contacts(&partial, &policy).unwrap()
        else {
            panic!("partial line parallel overlap was not retained");
        };
        assert_eq!(
            overlap.first_range().exact_endpoints(),
            Some((&q(1, 2), &Real::one()))
        );
        assert_eq!(
            overlap.second_range().exact_endpoints(),
            Some((&Real::zero(), &q(1, 2)))
        );
        assert_eq!(
            overlap.orientation(),
            RationalBezierOverlapOrientation2::Same
        );

        let Classification::Decided(BezierParallelIntersectionContacts2::Overlap(reversed)) =
            parallel
                .intersection_contacts(&partial.reversed(), &policy)
                .unwrap()
        else {
            panic!("reversed line parallel overlap was not retained");
        };
        assert_eq!(
            reversed.first_range().exact_endpoints(),
            Some((&q(1, 2), &Real::one()))
        );
        assert_eq!(
            reversed.orientation(),
            RationalBezierOverlapOrientation2::Reversed
        );
    }
}

#[test]
fn zero_distance_non_ph_parallel_reuses_the_exact_source_overlap() {
    let source = QuadraticBezier2::new(p(0, 0), p(1, 1), p(2, 0));
    let parallel = source.parallel_left(Real::zero()).unwrap();
    let same_source = RationalBezier2::try_new(
        source.control_points().into_iter().cloned().collect(),
        vec![Real::one(); 3],
    )
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let Classification::Decided(BezierParallelIntersectionContacts2::Overlap(overlap)) =
            parallel
                .intersection_contacts(&same_source, &policy)
                .unwrap()
        else {
            panic!("zero-distance non-PH source overlap was not retained");
        };
        assert_eq!(
            overlap.first_range().exact_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );
        assert_eq!(
            overlap.second_range().exact_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );
    }
}

#[test]
fn independently_constructed_ph_parallel_reuses_rational_overlap_authority() {
    let source = CubicBezier2::new(
        p(0, 0),
        Point2::new(q(1, 3), Real::zero()),
        Point2::new(q(2, 3), q(1, 3)),
        Point2::new(q(2, 3), Real::one()),
    );
    let parallel = source.parallel_left(Real::one()).unwrap();
    let Classification::Decided(Some(materialized)) = parallel
        .exact_pythagorean_hodograph_offset(&CurveContext::STRICT)
        .unwrap()
    else {
        panic!("canonical PH cubic did not materialize exactly");
    };
    let independently_constructed = RationalBezier2::try_new(
        materialized.curve().control_points().to_vec(),
        materialized.curve().weights().to_vec(),
    )
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let Classification::Decided(BezierParallelIntersectionContacts2::Overlap(overlap)) =
            parallel
                .intersection_contacts(&independently_constructed, &policy)
                .unwrap()
        else {
            panic!("independent PH offset overlap was not retained");
        };
        assert_eq!(
            overlap.first_range().exact_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );
        assert_eq!(
            overlap.second_range().exact_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );
    }
}

#[cfg(feature = "predicates")]
#[test]
fn parallel_rational_contacts_inherit_the_approximate_512_terminal() {
    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    let parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
        .parallel_left(undecidable_zero)
        .unwrap();
    let vertical = RationalBezier2::try_new(vec![p(1, -1), p(1, 1)], vec![r(1), r(1)]).unwrap();

    assert_eq!(
        parallel.intersection_contacts(&vertical, &CurveContext::STRICT),
        Ok(Classification::Uncertain(
            hypercurve::UncertaintyReason::RealSign
        ))
    );
    let Classification::Decided(BezierParallelIntersectionContacts2::Contacts(contacts)) = parallel
        .intersection_contacts(&vertical, &CurveContext::APPROXIMATE_512)
        .unwrap()
    else {
        panic!("APPROXIMATE_512 did not resolve the inherited terminal");
    };
    assert_eq!(contacts.len(), 1);
    assert_eq!(
        contacts[0].point(),
        &RationalBezierIntersectionPointEvidence2::Exact(p(1, 0))
    );
}

#[test]
fn generic_cubic_does_not_claim_exact_ph_parallel() {
    let source = CubicBezier2::new(p(0, 0), p(1, 2), p(2, -2), p(3, 0));
    let parallel = source.parallel_left(r(1)).unwrap();
    assert!(matches!(
        parallel
            .exact_pythagorean_hodograph_offset(&policy())
            .unwrap(),
        Classification::Decided(None)
    ));
}

#[test]
fn staged_offset_promotes_exact_ph_cubic_before_fitting() {
    let source = CubicBezier2::new(
        p(0, 0),
        Point2::new(q(1, 3), r(0)),
        Point2::new(q(2, 3), q(1, 3)),
        Point2::new(q(2, 3), r(1)),
    );
    let result = source.offset_left_staged(q(1, 5), &policy()).unwrap();
    let Classification::Decided(BezierOffsetCandidate2::ExactPythagoreanHodograph {
        offset, ..
    }) = result
    else {
        panic!("staged offset did not select the exact PH lane");
    };
    assert_eq!(offset.distance(), &q(1, 5));
}

#[test]
fn blend2d_quadratic_candidate_matches_exact_parallel_endpoints() {
    let source = QuadraticBezier2::new(p(0, 0), p(1, 2), p(3, 1));
    let candidate = match source
        .blend2d_offset_left_candidate(r(2), &policy())
        .unwrap()
    {
        Classification::Decided(candidate) => candidate,
        Classification::Uncertain(reason) => panic!("candidate was uncertain: {reason:?}"),
    };
    let parallel = source.parallel_left(r(2)).unwrap();
    for (parameter, candidate_point) in [
        (r(0), candidate.curve().start()),
        (r(1), candidate.curve().end()),
    ] {
        let exact = match parallel.point_at(&parameter, &policy()).unwrap() {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                panic!("endpoint parallel evaluation was uncertain: {reason:?}")
            }
        };
        assert_eq!(candidate_point, &exact);
    }
    assert_ne!(candidate.radial_error_bound(), &r(0));
}

#[test]
fn blend2d_straight_quadratic_candidate_has_zero_radial_error() {
    let source = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0));
    let candidate = match source
        .blend2d_offset_left_candidate(r(3), &policy())
        .unwrap()
    {
        Classification::Decided(candidate) => candidate,
        Classification::Uncertain(reason) => panic!("line candidate was uncertain: {reason:?}"),
    };
    assert_eq!(candidate.radial_error_bound(), &r(0));
    assert_eq!(candidate.curve().start(), &p(0, 3));
    assert_eq!(candidate.curve().control(), &p(1, 3));
    assert_eq!(candidate.curve().end(), &p(2, 3));
}

#[test]
fn blend2d_candidate_rejects_opposed_endpoint_tangents() {
    let source = QuadraticBezier2::new(p(-1, 0), p(0, 0), p(-1, 0));
    assert!(matches!(
        source
            .blend2d_offset_left_candidate(r(1), &policy())
            .unwrap(),
        Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)
    ));
}

#[test]
fn blend2d_cubic_reduction_has_exact_join_and_bound() {
    let source = CubicBezier2::new(
        p(0, 0),
        Point2::new(q(1, 3), r(0)),
        Point2::new(q(2, 3), r(0)),
        p(1, 1),
    );
    let reduction = source.blend2d_two_quadratic_reduction().unwrap();
    assert_eq!(reduction.first().end(), reduction.second().start());
    assert_eq!(reduction.first().end(), &source.point_at(q(1, 2)));
    assert_eq!(reduction.same_parameter_error_bound(), &q(1, 54));
    assert_eq!(
        reduction
            .first()
            .endpoint_tangent(hypercurve::BezierEndpoint::End),
        reduction
            .second()
            .endpoint_tangent(hypercurve::BezierEndpoint::Start)
    );
}

#[test]
fn verifier_certifies_exact_straight_parallel_without_subdivision() {
    let source = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0));
    let parallel = source.parallel_left(r(3)).unwrap();
    let candidate = match source
        .blend2d_offset_left_candidate(r(3), &policy())
        .unwrap()
    {
        Classification::Decided(candidate) => candidate,
        Classification::Uncertain(reason) => panic!("line candidate was uncertain: {reason:?}"),
    };
    let options =
        BezierParallelVerificationOptions::try_new(q(1, 1_000_000), 4, &policy()).unwrap();
    let certified = match parallel
        .verify_polynomial_candidate(candidate.curve().clone().into(), &options, &policy())
        .unwrap()
    {
        Classification::Decided(certified) => certified,
        Classification::Uncertain(reason) => panic!("line verification failed: {reason:?}"),
    };
    assert_eq!(certified.maximum_depth(), 0);
    assert_eq!(certified.leaf_count(), 1);
    assert_eq!(certified.error_bound(), options.max_error());
}

#[test]
fn verifier_certifies_curved_blend2d_candidate_by_exact_subdivision() {
    let source = QuadraticBezier2::new(p(0, 0), p(1, 1), p(2, 0));
    let parallel = source.parallel_left(q(1, 4)).unwrap();
    let candidate = match source
        .blend2d_offset_left_candidate(q(1, 4), &policy())
        .unwrap()
    {
        Classification::Decided(candidate) => candidate,
        Classification::Uncertain(reason) => panic!("curved candidate was uncertain: {reason:?}"),
    };
    let options = BezierParallelVerificationOptions::try_new(q(1, 20), 16, &policy()).unwrap();
    let certified = match parallel
        .verify_polynomial_candidate(candidate.curve().clone().into(), &options, &policy())
        .unwrap()
    {
        Classification::Decided(certified) => certified,
        Classification::Uncertain(reason) => panic!("curved verification failed: {reason:?}"),
    };
    assert!(certified.maximum_depth() > 0);
    assert!(certified.leaf_count() > 1);
    assert!(matches!(
        certified.curve(),
        BezierParallelApproximationCurve2::Quadratic(_)
    ));
}

#[test]
fn verifier_rejects_candidate_outside_requested_parallel_tube() {
    let source = QuadraticBezier2::new(p(0, 0), p(1, 1), p(2, 0));
    let parallel = source.parallel_left(r(1)).unwrap();
    let options = BezierParallelVerificationOptions::try_new(q(1, 10), 8, &policy()).unwrap();
    assert!(matches!(
        parallel
            .verify_polynomial_candidate(source.into(), &options, &policy())
            .unwrap(),
        Classification::Uncertain(hypercurve::UncertaintyReason::Unsupported)
    ));
}

#[test]
fn verifier_refuses_source_with_undefined_normal() {
    let source = QuadraticBezier2::new(p(0, 0), p(1, 0), p(0, 0));
    let parallel = source.parallel_left(r(1)).unwrap();
    let options = BezierParallelVerificationOptions::try_new(q(1, 10), 8, &policy()).unwrap();
    assert!(matches!(
        parallel
            .verify_polynomial_candidate(
                BezierParallelApproximationCurve2::Quadratic(QuadraticBezier2::new(
                    p(0, 1),
                    p(1, 1),
                    p(0, 1),
                )),
                &options,
                &policy(),
            )
            .unwrap(),
        Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)
    ));
}

#[test]
fn adaptive_quadratic_construction_subdivides_until_certified() {
    let source = QuadraticBezier2::new(p(0, 0), p(1, 1), p(2, 0));
    let options = BezierParallelVerificationOptions::try_new(q(1, 100), 16, &policy()).unwrap();
    let path = match source
        .approximate_parallel_blend2d_certified(q(1, 4), &options, &policy())
        .unwrap()
    {
        Classification::Decided(path) => path,
        Classification::Uncertain(reason) => {
            panic!("adaptive quadratic construction failed: {reason:?}")
        }
    };
    assert!(path.spans().len() >= 2);
    assert_eq!(path.spans().first().unwrap().source_start(), &r(0));
    assert_eq!(path.spans().last().unwrap().source_end(), &r(1));
    for pair in path.spans().windows(2) {
        assert_eq!(pair[0].source_end(), pair[1].source_start());
        let BezierParallelApproximationCurve2::Quadratic(first) = pair[0].approximation().curve()
        else {
            panic!("quadratic construction emitted non-quadratic candidate");
        };
        let BezierParallelApproximationCurve2::Quadratic(second) = pair[1].approximation().curve()
        else {
            panic!("quadratic construction emitted non-quadratic candidate");
        };
        assert_eq!(first.end(), second.start());
    }
}

#[test]
fn adaptive_cubic_construction_emits_connected_certified_curves() {
    let source = CubicBezier2::new(p(0, 0), p(1, 2), p(2, -1), p(4, 0));
    let options = BezierParallelVerificationOptions::try_new(q(1, 20), 14, &policy()).unwrap();
    let path = match source
        .approximate_parallel_blend2d_certified(q(1, 10), &options, &policy())
        .unwrap()
    {
        Classification::Decided(path) => path,
        Classification::Uncertain(reason) => {
            panic!("adaptive cubic construction failed: {reason:?}")
        }
    };
    assert!(path.spans().len() >= 2);
    assert!(path.construction_maximum_depth() >= 1);
    assert!(path.verification_leaf_count() >= path.spans().len());
    for pair in path.spans().windows(2) {
        assert_eq!(pair[0].source_end(), pair[1].source_start());
        let first_end = match pair[0].approximation().curve() {
            BezierParallelApproximationCurve2::Quadratic(curve) => curve.end(),
            BezierParallelApproximationCurve2::Cubic(curve) => curve.end(),
        };
        let second_start = match pair[1].approximation().curve() {
            BezierParallelApproximationCurve2::Quadratic(curve) => curve.start(),
            BezierParallelApproximationCurve2::Cubic(curve) => curve.start(),
        };
        assert_eq!(first_end, second_start);
    }
}

#[test]
fn levien_candidate_matches_parallel_endpoints_tangents_and_midpoint() {
    let source = CubicBezier2::new(p(0, 0), p(1, 2), p(3, 2), p(4, 0));
    let parallel = source.parallel_left(q(1, 10)).unwrap();
    let candidate = match parallel.levien_cubic_candidate(&policy()).unwrap() {
        Classification::Decided(candidate) => candidate,
        Classification::Uncertain(reason) => panic!("Levien candidate failed: {reason:?}"),
    };
    assert!(candidate.matched_midpoint());
    for parameter in [r(0), q(1, 2), r(1)] {
        let exact = match parallel.point_at(&parameter, &policy()).unwrap() {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => panic!("parallel evaluation failed: {reason:?}"),
        };
        let fitted = candidate.curve().point_at(parameter);
        let error = fitted
            .distance_squared(&exact)
            .to_f64_lossy()
            .expect("Levien replay distance is approximable");
        assert!(error.abs() <= 1.0e-20, "midpoint replay error was {error}");
    }
    for (parameter, endpoint) in [
        (r(0), hypercurve::BezierEndpoint::Start),
        (r(1), hypercurve::BezierEndpoint::End),
    ] {
        let exact = match parallel.derivative_at(&parameter, &policy()).unwrap() {
            Classification::Decided(derivative) => derivative,
            Classification::Uncertain(reason) => panic!("parallel derivative failed: {reason:?}"),
        };
        let fitted = candidate.curve().endpoint_tangent(endpoint);
        let cross = (exact.dx() * fitted.dy() - exact.dy() * fitted.dx())
            .to_f64_lossy()
            .expect("endpoint tangent cross product is approximable");
        assert!(cross.abs() <= 1.0e-20, "endpoint tangent cross was {cross}");
    }
    let options = BezierParallelVerificationOptions::try_new(q(1, 20), 14, &policy()).unwrap();
    assert!(matches!(
        parallel
            .verify_polynomial_candidate(candidate.curve().clone().into(), &options, &policy())
            .unwrap(),
        Classification::Decided(_)
    ));
    let fitted = match source
        .approximate_parallel_blend2d_certified(q(1, 10), &options, &policy())
        .unwrap()
    {
        Classification::Decided(fitted) => fitted,
        Classification::Uncertain(reason) => panic!("adaptive Levien fit failed: {reason:?}"),
    };
    assert_eq!(fitted.spans().len(), 1);
    assert!(matches!(
        fitted.spans()[0].approximation().curve(),
        BezierParallelApproximationCurve2::Cubic(_)
    ));
    // Blend2D's deterministic cubic reduction starts with two quadratic spans;
    // the accepted Levien cubic therefore halves the construction span count.
    let reduction = source.blend2d_two_quadratic_reduction().unwrap();
    assert_eq!(reduction.first().end(), reduction.second().start());
}

#[test]
fn certified_curve_path_parallel_preserves_smooth_exact_connections() {
    let path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(p(1, 0), p(1, 1), p(0, 1))),
        Curve2::from(QuadraticBezier2::new(p(0, 1), p(-1, 1), p(-1, 0))),
    ])
    .unwrap();
    let options = BezierParallelVerificationOptions::try_new(q(1, 20), 16, &policy()).unwrap();
    let parallel = match path
        .approximate_parallel_blend2d_certified(q(1, 10), &options, &policy())
        .unwrap()
    {
        Classification::Decided(parallel) => parallel,
        Classification::Uncertain(reason) => panic!("smooth path offset failed: {reason:?}"),
    };
    assert_eq!(parallel.source_curve_count(), 2);
    assert_eq!(parallel.approximated_source_curve_count(), 2);
    assert!(parallel.output_curve_count() >= 2);
    for pair in parallel.path().curves().windows(2) {
        assert_eq!(pair[0].end(), pair[1].start());
    }
}

#[test]
fn certified_curve_path_promotes_rational_ph_span_without_chords() {
    let source =
        RationalQuadraticBezier2::try_new(p(1, 0), p(1, 1), p(0, 1), r(1), r(1), r(2)).unwrap();
    let path = CurvePath2::try_new(vec![Curve2::from(source)]).unwrap();
    let options = BezierParallelVerificationOptions::try_new(q(1, 20), 12, &policy()).unwrap();
    let parallel = match path
        .approximate_parallel_blend2d_certified(q(1, 2), &options, &policy())
        .unwrap()
    {
        Classification::Decided(parallel) => parallel,
        Classification::Uncertain(reason) => panic!("rational PH path offset failed: {reason:?}"),
    };
    assert_eq!(parallel.source_curve_count(), 1);
    assert_eq!(parallel.exact_source_curve_count(), 1);
    assert_eq!(parallel.approximated_source_curve_count(), 0);
    assert_eq!(parallel.output_curve_count(), 1);
    assert!(matches!(
        parallel.path().curves()[0].geometry(),
        hypercurve::CurveGeometry2::RationalBezier(_)
    ));
}

#[test]
fn certified_curve_path_parallel_leaves_corner_join_to_higher_layer() {
    let path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))),
        Curve2::from(QuadraticBezier2::new(p(2, 0), p(2, 1), p(2, 2))),
    ])
    .unwrap();
    let options = BezierParallelVerificationOptions::try_new(q(1, 20), 12, &policy()).unwrap();
    assert!(matches!(
        path.approximate_parallel_blend2d_certified(q(1, 10), &options, &policy())
            .unwrap(),
        Classification::Uncertain(hypercurve::UncertaintyReason::Unsupported)
    ));
}

#[test]
fn curve_region_uses_certified_parallel_then_regularizes_output_chords() {
    let path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(p(1, 0), p(1, 1), p(0, 1))),
        Curve2::from(QuadraticBezier2::new(p(0, 1), p(-1, 1), p(-1, 0))),
        Curve2::from(QuadraticBezier2::new(p(-1, 0), p(-1, -1), p(0, -1))),
        Curve2::from(QuadraticBezier2::new(p(0, -1), p(1, -1), p(1, 0))),
    ])
    .unwrap();
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::EvenOdd],
        &policy(),
    )
    .unwrap()
    .into_value();
    let parallel_options =
        BezierParallelVerificationOptions::try_new(q(1, 20), 16, &policy()).unwrap();
    let flattening = BezierFlatteningOptions::try_new(q(1, 20), 16, &policy()).unwrap();
    let result = match region
        .offset_with_certified_bezier_parallel(
            q(1, 10),
            &parallel_options,
            &flattening,
            &flattening,
            &policy(),
        )
        .unwrap()
        .into_value()
    {
        Classification::Decided(result) => result,
        Classification::Uncertain(reason) => panic!("certified region offset failed: {reason:?}"),
    };
    assert!(result.evidence().used_certified_parallel_path());
    assert!(!result.evidence().used_segmented_source_fallback());
    assert_eq!(result.evidence().loop_evidence().len(), 1);
    assert_eq!(
        result
            .evidence()
            .certified_pre_regularization_boundary_error(),
        Some(&q(1, 10))
    );
    assert!(!result.evidence().final_boundary_hausdorff_certified());
    assert!(!result.region().boundary_loops().is_empty());
}

#[test]
fn curve_region_evidence_weaker_source_chord_fallback_for_authored_corner() {
    let path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(p(0, 0), p(1, -1), p(2, 0))),
        Curve2::from(QuadraticBezier2::new(p(2, 0), p(2, 1), p(2, 2))),
        Curve2::from(QuadraticBezier2::new(p(2, 2), p(1, 3), p(0, 2))),
        Curve2::from(QuadraticBezier2::new(p(0, 2), p(-1, 1), p(0, 0))),
    ])
    .unwrap();
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::EvenOdd],
        &policy(),
    )
    .unwrap()
    .into_value();
    let parallel_options =
        BezierParallelVerificationOptions::try_new(q(1, 10), 14, &policy()).unwrap();
    let flattening = BezierFlatteningOptions::try_new(q(1, 10), 14, &policy()).unwrap();
    let result = match region
        .offset_with_certified_bezier_parallel(
            q(1, 10),
            &parallel_options,
            &flattening,
            &flattening,
            &policy(),
        )
        .unwrap()
        .into_value()
    {
        Classification::Decided(result) => result,
        Classification::Uncertain(reason) => panic!("corner fallback failed: {reason:?}"),
    };
    assert!(!result.evidence().used_certified_parallel_path());
    assert!(result.evidence().used_segmented_source_fallback());
    assert!(
        result
            .evidence()
            .certified_pre_regularization_boundary_error()
            .is_none()
    );
    assert!(!result.evidence().final_boundary_hausdorff_certified());
    assert!(result.evidence().fallback_evidence().is_some());
}

proptest! {
    #[test]
    fn generated_blend2d_candidates_replay_exact_parallel_endpoints(
        translation_x in -100_i16..100,
        translation_y in -100_i16..100,
        run in 1_i16..20,
        rise in 1_i16..20,
        distance in -10_i16..11,
    ) {
        let tx = Real::from(translation_x);
        let ty = Real::from(translation_y);
        let run = Real::from(run);
        let rise = Real::from(rise);
        let distance = Real::from(distance);
        let source = QuadraticBezier2::new(
            Point2::new(tx.clone(), ty.clone()),
            Point2::new(&tx + &run, &ty + rise),
            Point2::new(&tx + &run * r(2), ty),
        );
        let candidate = match source
            .blend2d_offset_left_candidate(distance.clone(), &policy())
            .unwrap()
        {
            Classification::Decided(candidate) => candidate,
            Classification::Uncertain(reason) => {
                prop_assert!(false, "generated regular candidate was uncertain: {reason:?}");
                unreachable!()
            }
        };
        let parallel = source.parallel_left(distance).unwrap();
        for (parameter, endpoint) in [
            (r(0), candidate.curve().start()),
            (r(1), candidate.curve().end()),
        ] {
            let exact = match parallel.point_at(&parameter, &policy()).unwrap() {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    prop_assert!(false, "generated endpoint evaluation was uncertain: {reason:?}");
                    unreachable!()
                }
            };
            let replay_error = endpoint.distance_squared(&exact).to_f64_lossy().unwrap();
            prop_assert!(replay_error.abs() <= 1.0e-20);
        }
    }
}
