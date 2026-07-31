use hypercurve::{
    BezierAreaMomentPrefixSums2, BezierAreaPrefixSums2, BezierFlatteningOptions,
    BezierLineImageFitRelation, BezierOffsetCandidate2, BezierParallelApproximationCurve2,
    BezierParallelVerificationOptions, BezierParameter2, Classification, CubicBezier2, Curve2,
    CurveContext, CurveError, CurvePath2, CurveRegion2, CurveRegionLoopRole, FillRule, Point2,
    QuadraticBezier2, Rational, RationalQuadraticBezier2, Real,
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
    assert!(exact.rational_degree() >= 5);
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
