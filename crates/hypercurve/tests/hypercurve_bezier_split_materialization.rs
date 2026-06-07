use hypercurve::{
    BezierAlgebraicEndpointImage2, BezierAlgebraicImageStatus, BezierAlgebraicParameter2,
    BezierEndpointPointImage2, BezierEndpointTangentImage2, BezierParameter2,
    BezierParameterInterval, BezierParameterPolynomial, BezierSplitFragment2,
    BezierSplitMaterialization2, BezierSubcurve2, Classification, CubicBezier2, CurveError,
    CurvePolicy, Point2, QuadraticBezier2, RationalQuadraticBezier2, Real, UncertaintyReason,
};
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn assert_topology_error<T>(result: hypercurve::CurveResult<T>) {
    match result {
        Err(CurveError::Topology(_)) => {}
        Ok(_) => panic!("expected topology error"),
        Err(error) => panic!("expected topology error, got {error:?}"),
    }
}

fn exact(value: Real) -> BezierParameter2 {
    match BezierParameter2::exact(value, &policy()).unwrap() {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => {
            panic!("exact parameter unexpectedly uncertain: {reason:?}")
        }
    }
}

fn algebraic_midpoint_interval(start: Real, end: Real) -> BezierParameter2 {
    let polynomial =
        match BezierParameterPolynomial::try_new_power_basis(vec![r(-1), r(2)], &policy()).unwrap()
        {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                panic!("polynomial unexpectedly uncertain: {reason:?}")
            }
        };
    let interval = match BezierParameterInterval::try_new(start, end, &policy()).unwrap() {
        Classification::Decided(interval) => interval,
        Classification::Uncertain(reason) => panic!("interval unexpectedly uncertain: {reason:?}"),
    };
    match BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy()).unwrap() {
        Classification::Decided(parameter) => BezierParameter2::algebraic(parameter),
        Classification::Uncertain(reason) => {
            panic!("algebraic parameter unexpectedly uncertain: {reason:?}")
        }
    }
}

fn algebraic_sqrt_half_interval() -> BezierParameter2 {
    algebraic_sqrt_half_interval_between(q(2, 3), q(3, 4))
}

fn algebraic_sqrt_half_interval_between(start: Real, end: Real) -> BezierParameter2 {
    let polynomial =
        match BezierParameterPolynomial::try_new_power_basis(vec![r(-1), r(0), r(2)], &policy())
            .unwrap()
        {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                panic!("polynomial unexpectedly uncertain: {reason:?}")
            }
        };
    let interval = match BezierParameterInterval::try_new(start, end, &policy()).unwrap() {
        Classification::Decided(interval) => interval,
        Classification::Uncertain(reason) => panic!("interval unexpectedly uncertain: {reason:?}"),
    };
    match BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy()).unwrap() {
        Classification::Decided(parameter) => BezierParameter2::algebraic(parameter),
        Classification::Uncertain(reason) => {
            panic!("algebraic parameter unexpectedly uncertain: {reason:?}")
        }
    }
}

fn algebraic_cubic_midpoint_interval() -> BezierParameter2 {
    let polynomial = match BezierParameterPolynomial::try_new_power_basis(
        vec![r(-1), r(2), r(-1), r(2)],
        &policy(),
    )
    .unwrap()
    {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => {
            panic!("polynomial unexpectedly uncertain: {reason:?}")
        }
    };
    let interval = match BezierParameterInterval::try_new(q(2, 5), q(3, 5), &policy()).unwrap() {
        Classification::Decided(interval) => interval,
        Classification::Uncertain(reason) => panic!("interval unexpectedly uncertain: {reason:?}"),
    };
    match BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy()).unwrap() {
        Classification::Decided(parameter) => BezierParameter2::algebraic(parameter),
        Classification::Uncertain(reason) => {
            panic!("algebraic parameter unexpectedly uncertain: {reason:?}")
        }
    }
}

fn assert_polynomial_endpoint_image(image: &Option<BezierAlgebraicEndpointImage2>) {
    let image = image
        .as_ref()
        .expect("algebraic boundary should retain an endpoint image");
    assert!(image.is_transformed());
    match image.point() {
        BezierEndpointPointImage2::Polynomial(point) => {
            assert_eq!(point.status(), BezierAlgebraicImageStatus::Transformed);
            assert!(point.x().and_then(|x| x.representation()).is_some());
            assert!(point.y().and_then(|y| y.representation()).is_some());
        }
        BezierEndpointPointImage2::RationalQuadratic(_) => {
            panic!("expected polynomial point image")
        }
    }
    match image.tangent() {
        BezierEndpointTangentImage2::Polynomial(tangent) => {
            assert_eq!(tangent.status(), BezierAlgebraicImageStatus::Transformed);
            assert!(tangent.dx().and_then(|dx| dx.representation()).is_some());
            assert!(tangent.dy().and_then(|dy| dy.representation()).is_some());
        }
        BezierEndpointTangentImage2::RationalQuadratic(_) => {
            panic!("expected polynomial tangent image")
        }
    }
}

fn assert_rational_endpoint_image(image: &Option<BezierAlgebraicEndpointImage2>) {
    let image = image
        .as_ref()
        .expect("algebraic boundary should retain a rational endpoint image");
    assert!(image.is_transformed());
    match image.point() {
        BezierEndpointPointImage2::RationalQuadratic(point) => {
            assert_eq!(point.status(), BezierAlgebraicImageStatus::Transformed);
            assert!(point.x().and_then(|x| x.representation()).is_some());
            assert!(point.y().and_then(|y| y.representation()).is_some());
        }
        BezierEndpointPointImage2::Polynomial(_) => panic!("expected rational point image"),
    }
    match image.tangent() {
        BezierEndpointTangentImage2::RationalQuadratic(tangent) => {
            assert_eq!(tangent.status(), BezierAlgebraicImageStatus::Transformed);
            assert!(tangent.dx().and_then(|dx| dx.representation()).is_some());
            assert!(tangent.dy().and_then(|dy| dy.representation()).is_some());
        }
        BezierEndpointTangentImage2::Polynomial(_) => panic!("expected rational tangent image"),
    }
    if let Some(second_derivative) = image.second_derivative() {
        match second_derivative {
            BezierEndpointTangentImage2::RationalQuadratic(second_derivative) => {
                assert_eq!(
                    second_derivative.status(),
                    BezierAlgebraicImageStatus::Transformed
                );
                assert!(
                    second_derivative
                        .dx()
                        .and_then(|dx| dx.representation())
                        .is_some()
                );
                assert!(
                    second_derivative
                        .dy()
                        .and_then(|dy| dy.representation())
                        .is_some()
                );
            }
            BezierEndpointTangentImage2::Polynomial(_) => {
                panic!("expected rational second derivative image")
            }
        }
    }
}

fn assert_rational_second_derivative_endpoint_image(image: &BezierAlgebraicEndpointImage2) {
    match image
        .second_derivative()
        .expect("expected rational second derivative image")
    {
        BezierEndpointTangentImage2::RationalQuadratic(second_derivative) => {
            assert_eq!(
                second_derivative.status(),
                BezierAlgebraicImageStatus::Transformed
            );
            assert!(
                second_derivative
                    .dx()
                    .and_then(|dx| dx.representation())
                    .is_some()
            );
            assert!(
                second_derivative
                    .dy()
                    .and_then(|dy| dy.representation())
                    .is_some()
            );
        }
        BezierEndpointTangentImage2::Polynomial(_) => {
            panic!("expected rational second derivative image")
        }
    }
}

#[test]
fn exact_quadratic_split_materializes_native_subcurves() {
    let curve = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let materialization = match curve
        .split_at_parameters(&[exact(q(1, 2))], &policy())
        .unwrap()
    {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("split unexpectedly uncertain: {reason:?}"),
    };

    assert!(materialization.is_fully_materialized());
    assert_eq!(materialization.fragments().len(), 2);
    let BezierSplitFragment2::Materialized {
        curve: BezierSubcurve2::Quadratic(left),
        ..
    } = &materialization.fragments()[0]
    else {
        panic!("first fragment should be a quadratic");
    };
    let BezierSplitFragment2::Materialized {
        curve: BezierSubcurve2::Quadratic(right),
        ..
    } = &materialization.fragments()[1]
    else {
        panic!("second fragment should be a quadratic");
    };

    let midpoint = curve.point_at(q(1, 2));
    assert_eq!(left.end(), &midpoint);
    assert_eq!(right.start(), &midpoint);
    assert_eq!(left.start(), curve.start());
    assert_eq!(right.end(), curve.end());
}

#[test]
fn split_materialization_constructor_rejects_duplicate_fragments() {
    assert_topology_error(BezierSplitMaterialization2::new(Vec::new()));

    let curve = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let materialization = match curve
        .split_at_parameters(&[exact(q(1, 2))], &policy())
        .unwrap()
    {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("split unexpectedly uncertain: {reason:?}"),
    };

    let first = materialization.fragments()[0].clone();
    let second = materialization.fragments()[1].clone();
    BezierSplitMaterialization2::new(vec![first.clone(), second]).unwrap();
    assert_topology_error(BezierSplitMaterialization2::new(vec![first.clone(), first]));
}

#[test]
fn split_materialization_constructor_rejects_incomplete_source_coverage() {
    let curve = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let materialization = match curve
        .split_at_parameters(&[exact(q(1, 3)), exact(q(2, 3))], &policy())
        .unwrap()
    {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("split unexpectedly uncertain: {reason:?}"),
    };

    let middle = materialization.fragments()[1].clone();
    assert_topology_error(BezierSplitMaterialization2::new(vec![middle]));
}

#[test]
fn split_materialization_constructor_rejects_noncontiguous_fragments() {
    let curve = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let materialization = match curve
        .split_at_parameters(&[exact(q(1, 3)), exact(q(2, 3))], &policy())
        .unwrap()
    {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("split unexpectedly uncertain: {reason:?}"),
    };

    let first = materialization.fragments()[0].clone();
    let third = materialization.fragments()[2].clone();
    assert_topology_error(BezierSplitMaterialization2::new(vec![first, third]));
}

#[test]
fn split_materialization_constructor_rejects_materialized_algebraic_range() {
    let curve = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let materialized = BezierSubcurve2::Quadratic(
        curve
            .subcurve_between_exact(&r(0), &q(1, 2), &policy())
            .unwrap(),
    );
    let fragment = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: algebraic_sqrt_half_interval(),
        curve: materialized,
    };

    assert_topology_error(BezierSplitMaterialization2::new(vec![fragment]));
}

#[test]
fn split_materialization_constructor_rejects_forged_algebraic_endpoint_evidence() {
    let curve = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let materialization = match curve
        .split_at_parameters(
            &[
                exact(q(1, 4)),
                algebraic_sqrt_half_interval(),
                exact(q(4, 5)),
            ],
            &policy(),
        )
        .unwrap()
    {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("split unexpectedly uncertain: {reason:?}"),
    };
    BezierSplitMaterialization2::new(materialization.fragments().to_vec()).unwrap();

    let BezierSplitFragment2::AlgebraicEndpointImages {
        start,
        end,
        source_curve,
        start_image,
        end_image,
    } = materialization.fragments()[1].clone()
    else {
        panic!("expected algebraic endpoint-image fragment");
    };

    assert_topology_error(BezierSplitMaterialization2::new(vec![
        BezierSplitFragment2::AlgebraicEndpointImages {
            start: start.clone(),
            end: end.clone(),
            source_curve: source_curve.clone(),
            start_image: start_image.clone(),
            end_image: None,
        },
    ]));

    let wrong_parameter = match algebraic_cubic_midpoint_interval() {
        BezierParameter2::Algebraic(parameter) => parameter,
        BezierParameter2::Exact(_) => panic!("expected algebraic parameter"),
    };
    let wrong_parameter_image =
        BezierAlgebraicEndpointImage2::quadratic(&curve, &wrong_parameter, &policy()).unwrap();
    assert_topology_error(BezierSplitMaterialization2::new(vec![
        BezierSplitFragment2::AlgebraicEndpointImages {
            start: start.clone(),
            end: end.clone(),
            source_curve: source_curve.clone(),
            start_image: start_image.clone(),
            end_image: Some(wrong_parameter_image),
        },
    ]));

    let wrong_source = BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(0, 4), p(4, 0)));
    assert_topology_error(BezierSplitMaterialization2::new(vec![
        BezierSplitFragment2::AlgebraicEndpointImages {
            start,
            end,
            source_curve: Some(wrong_source),
            start_image,
            end_image,
        },
    ]));
}

#[test]
fn exact_cubic_subcurve_matches_original_endpoints_at_range_bounds() {
    let curve = CubicBezier2::new(p(0, 0), p(2, 6), p(6, -2), p(8, 0));
    let subcurve = curve
        .subcurve_between_exact(&q(1, 4), &q(3, 4), &policy())
        .unwrap();

    assert_eq!(subcurve.start(), &curve.point_at(q(1, 4)));
    assert_eq!(subcurve.end(), &curve.point_at(q(3, 4)));
}

#[test]
fn exact_rational_quadratic_split_preserves_conic_endpoint_evaluation() {
    let curve =
        RationalQuadraticBezier2::try_unit_end_weights(p(1, 0), p(1, 1), p(0, 1), q(1, 2)).unwrap();
    let subcurve = curve
        .subcurve_between_exact(&r(0), &q(1, 2), &policy())
        .unwrap();
    let expected_midpoint = match curve.point_at(q(1, 2), &policy()) {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            panic!("conic midpoint unexpectedly uncertain: {reason:?}")
        }
    };

    assert_eq!(subcurve.start(), curve.start());
    assert_eq!(subcurve.end(), &expected_midpoint);
}

#[test]
fn linear_algebraic_boundary_materializes_native_subcurves() {
    let curve = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let materialization = match curve
        .split_at_parameters(&[algebraic_midpoint_interval(q(2, 5), q(3, 5))], &policy())
        .unwrap()
    {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("split unexpectedly uncertain: {reason:?}"),
    };

    assert!(materialization.is_fully_materialized());
    assert!(!materialization.has_algebraic_endpoint_images());
    assert_eq!(materialization.fragments().len(), 2);
    let BezierSplitFragment2::Materialized {
        start,
        end,
        curve: BezierSubcurve2::Quadratic(left),
    } = &materialization.fragments()[0]
    else {
        panic!("first fragment should be native after linear-root promotion");
    };
    assert_eq!(start.as_exact(), Some(&r(0)));
    assert_eq!(end.as_exact(), Some(&q(1, 2)));
    assert_eq!(left.end(), &curve.point_at(q(1, 2)));
}

#[test]
fn algebraic_boundary_carries_endpoint_images_without_approximate_materialization() {
    let curve = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let materialization = match curve
        .split_at_parameters(
            &[
                exact(q(1, 4)),
                algebraic_sqrt_half_interval(),
                exact(q(4, 5)),
            ],
            &policy(),
        )
        .unwrap()
    {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("split unexpectedly uncertain: {reason:?}"),
    };

    assert!(!materialization.has_unresolved_fragments());
    assert!(materialization.has_algebraic_endpoint_images());
    assert_eq!(materialization.fragments().len(), 4);
    assert!(matches!(
        materialization.fragments()[0],
        BezierSplitFragment2::Materialized { .. }
    ));
    let BezierSplitFragment2::AlgebraicEndpointImages {
        source_curve,
        start_image,
        end_image,
        ..
    } = &materialization.fragments()[1]
    else {
        panic!("left algebraic fragment should carry endpoint images");
    };
    assert!(source_curve.is_some());
    assert!(start_image.is_none());
    assert_polynomial_endpoint_image(end_image);

    let BezierSplitFragment2::AlgebraicEndpointImages {
        source_curve,
        start_image,
        end_image,
        ..
    } = &materialization.fragments()[2]
    else {
        panic!("right algebraic fragment should carry endpoint images");
    };
    assert!(source_curve.is_some());
    assert_polynomial_endpoint_image(start_image);
    assert!(end_image.is_none());
}

#[test]
fn rational_algebraic_boundary_carries_conic_endpoint_images() {
    let curve =
        RationalQuadraticBezier2::try_unit_end_weights(p(1, 0), p(1, 1), p(0, 1), q(1, 2)).unwrap();
    let materialization = match curve
        .split_at_parameters(
            &[
                exact(q(1, 4)),
                algebraic_sqrt_half_interval(),
                exact(q(4, 5)),
            ],
            &policy(),
        )
        .unwrap()
    {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("split unexpectedly uncertain: {reason:?}"),
    };

    assert!(!materialization.has_unresolved_fragments());
    assert!(materialization.has_algebraic_endpoint_images());
    let BezierSplitFragment2::AlgebraicEndpointImages {
        source_curve,
        start_image,
        end_image,
        ..
    } = &materialization.fragments()[1]
    else {
        panic!("rational fragment should carry endpoint images");
    };
    assert!(source_curve.is_some());
    assert!(start_image.is_none());
    assert_rational_endpoint_image(end_image);
}

#[test]
fn rational_algebraic_endpoint_retains_second_derivative_when_constructed() {
    let curve =
        RationalQuadraticBezier2::try_new(p(-1, 1), p(0, -1), p(1, 1), r(1), r(1), r(1)).unwrap();
    let parameter = match algebraic_midpoint_interval(q(2, 5), q(3, 5)) {
        BezierParameter2::Algebraic(parameter) => parameter,
        BezierParameter2::Exact(_) => panic!("expected algebraic parameter"),
    };
    let image =
        BezierAlgebraicEndpointImage2::rational_quadratic(&curve, &parameter, &policy()).unwrap();

    assert_rational_endpoint_image(&Some(image.clone()));
    assert_rational_second_derivative_endpoint_image(&image);
}

#[test]
fn rational_algebraic_boundary_with_zero_denominator_stays_unresolved() {
    let curve =
        RationalQuadraticBezier2::try_unit_end_weights(p(0, 0), p(1, 1), p(2, 0), r(-1)).unwrap();
    let materialization = match curve
        .split_at_parameters(&[algebraic_cubic_midpoint_interval()], &policy())
        .unwrap()
    {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("split unexpectedly uncertain: {reason:?}"),
    };

    assert!(materialization.has_unresolved_fragments());
    assert!(!materialization.has_algebraic_endpoint_images());
    assert!(matches!(
        materialization.fragments()[0],
        BezierSplitFragment2::Unresolved { .. }
    ));
}

#[test]
fn broad_algebraic_interval_refuses_to_order_against_endpoints() {
    let curve = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let split = curve
        .split_at_parameters(
            &[algebraic_sqrt_half_interval_between(r(0), r(1))],
            &policy(),
        )
        .unwrap();

    assert_eq!(
        split,
        Classification::Uncertain(UncertaintyReason::Ordering)
    );
}

proptest! {
    #[test]
    fn exact_quadratic_split_endpoints_match_original(
        start_n in 0_i32..=15,
        width_n in 1_i32..=16,
    ) {
        let end_n = (start_n + width_n).min(16);
        prop_assume!(start_n < end_n);
        let start = q(start_n, 16);
        let end = q(end_n, 16);
        let curve = QuadraticBezier2::new(p(-3, 1), p(5, 9), p(11, -7));
        let subcurve = curve
            .subcurve_between_exact(&start, &end, &policy())
            .map_err(|error| TestCaseError::fail(format!("split failed: {error:?}")))?;

        prop_assert_eq!(subcurve.start(), &curve.point_at(start));
        prop_assert_eq!(subcurve.end(), &curve.point_at(end));
    }
}
