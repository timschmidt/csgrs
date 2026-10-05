use crate::bezier_split::BezierSplitMaterialization2;
use crate::{
    BezierAlgebraicEndpointImage2, BezierAlgebraicImageStatus, RationalBezierAlgebraicTangentImage2,
};
use crate::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, BezierSplitFragment2, BezierSubcurve2, Classification, CubicBezier2,
    CurveContext, Point2, QuadraticBezier2, RationalQuadraticBezier2, Real, UncertaintyReason,
};
use proptest::prelude::*;

fn is_fully_materialized(split: &BezierSplitMaterialization2) -> bool {
    split
        .fragments()
        .iter()
        .all(|fragment| matches!(fragment, BezierSplitFragment2::Materialized { .. }))
}

fn has_retained_beziers(split: &BezierSplitMaterialization2) -> bool {
    split
        .fragments()
        .iter()
        .any(|fragment| matches!(fragment, BezierSplitFragment2::RetainedBezier { .. }))
}

fn decided<T>(value: Classification<T>) -> T {
    match value {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("expected exact value: {reason:?}"),
    }
}

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn policy() -> CurveContext {
    CurveContext::STRICT
}

fn algebraic_midpoint_interval(start: Real, end: Real) -> BezierParameter2 {
    let polynomial = match BezierParameterPolynomial::try_new_power_basis_with_policy(
        vec![r(-1), r(2)],
        &policy(),
    )
    .unwrap()
    {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => {
            panic!("polynomial unexpectedly uncertain: {reason:?}")
        }
    };
    let interval = match BezierParameterInterval::try_new_with_policy(start, end, &policy())
        .unwrap()
    {
        Classification::Decided(interval) => interval,
        Classification::Uncertain(reason) => panic!("interval unexpectedly uncertain: {reason:?}"),
    };
    match BezierAlgebraicParameter2::try_isolate_with_policy(polynomial, interval, &policy())
        .unwrap()
    {
        Classification::Decided(parameter) => BezierParameter2::Algebraic(parameter),
        Classification::Uncertain(reason) => {
            panic!("algebraic parameter unexpectedly uncertain: {reason:?}")
        }
    }
}

fn algebraic_sqrt_half_interval() -> BezierParameter2 {
    algebraic_sqrt_half_interval_between(q(2, 3), q(3, 4))
}

fn algebraic_sqrt_half_interval_between(start: Real, end: Real) -> BezierParameter2 {
    let polynomial = match BezierParameterPolynomial::try_new_power_basis_with_policy(
        vec![r(-1), r(0), r(2)],
        &policy(),
    )
    .unwrap()
    {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => {
            panic!("polynomial unexpectedly uncertain: {reason:?}")
        }
    };
    let interval = match BezierParameterInterval::try_new_with_policy(start, end, &policy())
        .unwrap()
    {
        Classification::Decided(interval) => interval,
        Classification::Uncertain(reason) => panic!("interval unexpectedly uncertain: {reason:?}"),
    };
    match BezierAlgebraicParameter2::try_isolate_with_policy(polynomial, interval, &policy())
        .unwrap()
    {
        Classification::Decided(parameter) => BezierParameter2::Algebraic(parameter),
        Classification::Uncertain(reason) => {
            panic!("algebraic parameter unexpectedly uncertain: {reason:?}")
        }
    }
}

fn algebraic_cubic_midpoint_interval() -> BezierParameter2 {
    let polynomial = match BezierParameterPolynomial::try_new_power_basis_with_policy(
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
    let interval = match BezierParameterInterval::try_new_with_policy(q(2, 5), q(3, 5), &policy())
        .unwrap()
    {
        Classification::Decided(interval) => interval,
        Classification::Uncertain(reason) => panic!("interval unexpectedly uncertain: {reason:?}"),
    };
    match BezierAlgebraicParameter2::try_isolate_with_policy(polynomial, interval, &policy())
        .unwrap()
    {
        Classification::Decided(parameter) => BezierParameter2::Algebraic(parameter),
        Classification::Uncertain(reason) => {
            panic!("algebraic parameter unexpectedly uncertain: {reason:?}")
        }
    }
}

fn assert_transformed_tangent(tangent: &RationalBezierAlgebraicTangentImage2) {
    assert_eq!(tangent.status(), BezierAlgebraicImageStatus::Transformed);
    assert!(tangent.dx().and_then(|dx| dx.representation()).is_some());
    assert!(tangent.dy().and_then(|dy| dy.representation()).is_some());
}

fn assert_endpoint_image(image: &Option<BezierAlgebraicEndpointImage2>) {
    let image = image
        .as_ref()
        .expect("algebraic boundary retains endpoint evidence");
    assert!(image.is_exact());
    let point = decided(image.point().unwrap());
    assert_eq!(point.status(), BezierAlgebraicImageStatus::Transformed);
    assert!(point.x().and_then(|x| x.representation()).is_some());
    assert!(point.y().and_then(|y| y.representation()).is_some());
    assert_transformed_tangent(decided(image.tangent().unwrap()));
    if let Some(second) = image.second_derivative() {
        assert_transformed_tangent(second);
    }
}

fn assert_rational_second_derivative_endpoint_image(image: &BezierAlgebraicEndpointImage2) {
    assert_transformed_tangent(image.second_derivative().expect("second derivative image"));
    assert_transformed_tangent(image.third_derivative().expect("third derivative image"));
}

#[test]
fn exact_quadratic_split_materializes_native_subcurves() {
    let curve = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let materialization = match curve
        .split_at_parameters(&[BezierParameter2::Exact(q(1, 2))], &policy())
        .unwrap()
    {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("split unexpectedly uncertain: {reason:?}"),
    };

    assert!(is_fully_materialized(&materialization));
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
fn exact_cubic_subcurve_matches_original_endpoints_at_range_bounds() {
    let curve = CubicBezier2::new(p(0, 0), p(2, 6), p(6, -2), p(8, 0));
    let subcurve = curve
        .subcurve_between_exact_with_policy(&q(1, 4), &q(3, 4), &policy())
        .unwrap();

    assert_eq!(subcurve.start(), &curve.point_at(q(1, 4)));
    assert_eq!(subcurve.end(), &curve.point_at(q(3, 4)));
}

#[test]
fn exact_rational_quadratic_split_preserves_conic_endpoint_evaluation() {
    let curve =
        RationalQuadraticBezier2::try_unit_end_weights(p(1, 0), p(1, 1), p(0, 1), q(1, 2)).unwrap();
    let Classification::Decided(subcurve) = curve
        .subcurve_between_exact_with_policy(&r(0), &q(1, 2), &policy())
        .unwrap()
    else {
        panic!("finite conic cut must be decided");
    };
    let expected_midpoint = match curve.point_at_with_policy(q(1, 2), &policy()) {
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

    assert!(is_fully_materialized(&materialization));
    assert!(!has_retained_beziers(&materialization));
    assert_eq!(materialization.fragments().len(), 2);
    let BezierSplitFragment2::Materialized {
        start,
        end,
        curve: BezierSubcurve2::Quadratic(left),
    } = &materialization.fragments()[0]
    else {
        panic!("first fragment should be native after linear-root promotion");
    };
    assert_eq!(start.scalar(), Some(&r(0)));
    assert_eq!(end.scalar(), Some(&q(1, 2)));
    assert_eq!(left.end(), &curve.point_at(q(1, 2)));
}

#[test]
fn algebraic_boundary_carries_endpoint_images_without_approximate_materialization() {
    let curve = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let materialization = match curve
        .split_at_parameters(
            &[
                BezierParameter2::Exact(q(1, 4)),
                algebraic_sqrt_half_interval(),
                BezierParameter2::Exact(q(4, 5)),
            ],
            &policy(),
        )
        .unwrap()
    {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("split unexpectedly uncertain: {reason:?}"),
    };

    assert!(has_retained_beziers(&materialization));
    assert_eq!(materialization.fragments().len(), 4);
    assert!(matches!(
        materialization.fragments()[0],
        BezierSplitFragment2::Materialized { .. }
    ));
    let BezierSplitFragment2::RetainedBezier {
        source_curve,
        start_image,
        end_image,
        ..
    } = &materialization.fragments()[1]
    else {
        panic!("left algebraic fragment should carry endpoint images");
    };
    assert!(matches!(source_curve, BezierSubcurve2::Quadratic(_)));
    assert!(start_image.is_none());
    assert_endpoint_image(end_image);

    let BezierSplitFragment2::RetainedBezier {
        source_curve,
        start_image,
        end_image,
        ..
    } = &materialization.fragments()[2]
    else {
        panic!("right algebraic fragment should carry endpoint images");
    };
    assert!(matches!(source_curve, BezierSubcurve2::Quadratic(_)));
    assert_endpoint_image(start_image);
    assert!(end_image.is_none());
}

#[test]
fn algebraic_fragment_reversal_retains_source_evidence_and_toggles_traversal() {
    let curve = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let materialization = match curve
        .split_at_parameters(&[algebraic_sqrt_half_interval()], &policy())
        .unwrap()
    {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("split unexpectedly uncertain: {reason:?}"),
    };
    let forward = materialization.fragments()[0].clone();
    let reversed = forward.reversed().unwrap();

    let BezierSplitFragment2::RetainedBezier {
        reversed: forward_orientation,
        start: forward_start,
        end: forward_end,
        source_curve: forward_source,
        start_image: forward_start_image,
        end_image: forward_end_image,
    } = &forward
    else {
        panic!("expected algebraic endpoint-image fragment");
    };
    let BezierSplitFragment2::RetainedBezier {
        reversed: reverse_orientation,
        start: reverse_start,
        end: reverse_end,
        source_curve: reverse_source,
        start_image: reverse_start_image,
        end_image: reverse_end_image,
    } = &reversed
    else {
        panic!("reversal must retain the algebraic carrier");
    };

    assert!(!forward_orientation);
    assert!(*reverse_orientation);
    assert_eq!(reverse_start, forward_start);
    assert_eq!(reverse_end, forward_end);
    assert_eq!(reverse_source, forward_source);
    assert_eq!(reverse_start_image, forward_start_image);
    assert_eq!(reverse_end_image, forward_end_image);
    assert_eq!(reversed.reversed().unwrap(), forward);
}

#[test]
fn algebraic_endpoint_image_is_a_clone_shared_handle() {
    assert_eq!(
        std::mem::size_of::<BezierAlgebraicEndpointImage2>(),
        std::mem::size_of::<usize>()
    );
}

#[test]
fn rational_algebraic_boundary_carries_conic_endpoint_images() {
    let curve =
        RationalQuadraticBezier2::try_unit_end_weights(p(1, 0), p(1, 1), p(0, 1), q(1, 2)).unwrap();
    let materialization = match curve
        .split_at_parameters(
            &[
                BezierParameter2::Exact(q(1, 4)),
                algebraic_sqrt_half_interval(),
                BezierParameter2::Exact(q(4, 5)),
            ],
            &policy(),
        )
        .unwrap()
    {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("split unexpectedly uncertain: {reason:?}"),
    };

    assert!(has_retained_beziers(&materialization));
    let BezierSplitFragment2::RetainedBezier {
        source_curve,
        start_image,
        end_image,
        ..
    } = &materialization.fragments()[1]
    else {
        panic!("rational fragment should carry endpoint images");
    };
    assert!(matches!(
        source_curve,
        BezierSubcurve2::RationalQuadratic(_)
    ));
    assert!(start_image.is_none());
    assert_endpoint_image(end_image);
}

#[test]
fn rational_algebraic_endpoint_retains_second_derivative_when_constructed() {
    let curve =
        RationalQuadraticBezier2::try_new(p(-1, 1), p(0, -1), p(1, 1), r(1), r(1), r(1)).unwrap();
    let parameter = match algebraic_midpoint_interval(q(2, 5), q(3, 5)) {
        BezierParameter2::Algebraic(parameter) => parameter,
        BezierParameter2::Exact(_) => panic!("expected algebraic parameter"),
    };
    let image = decided(
        BezierAlgebraicEndpointImage2::rational_quadratic(&curve, &parameter, &policy()).unwrap(),
    );

    assert_endpoint_image(&Some(image.clone()));
    assert_rational_second_derivative_endpoint_image(&image);
}

#[test]
fn rational_algebraic_boundary_with_zero_denominator_returns_explicit_uncertainty() {
    let curve =
        RationalQuadraticBezier2::try_unit_end_weights(p(0, 0), p(1, 1), p(2, 0), r(-1)).unwrap();
    let general_curve = crate::RationalBezier2::from(curve.clone());
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for general in [false, true] {
            let retained = algebraic_cubic_midpoint_interval();
            let Classification::Decided(promoted) = retained
                .clone()
                .promote_represented_exact_point_with_policy(&policy)
                .unwrap()
            else {
                panic!("the rational midpoint must admit an exact scalar view");
            };
            assert!(matches!(retained, BezierParameter2::Algebraic(_)));
            assert!(retained.scalar().is_some());
            assert!(matches!(promoted, BezierParameter2::Exact(_)));
            for (view, parameter) in [
                ("exact", BezierParameter2::Exact(q(1, 2))),
                ("cold algebraic", algebraic_cubic_midpoint_interval()),
                ("retained scalar", retained),
                ("promoted", promoted),
            ] {
                let result = if general {
                    general_curve.split_at_parameters(&[parameter], &policy)
                } else {
                    curve.split_at_parameters(&[parameter], &policy)
                };
                assert!(
                    matches!(
                        result,
                        Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
                    ),
                    "pole boundary must remain unsupported: {view}, general={general}"
                );
            }
        }
    }
}

#[test]
fn broad_singleton_isolator_materializes_exact_endpoint_images() {
    let curve = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let split = curve
            .split_at_parameters(&[algebraic_sqrt_half_interval_between(r(0), r(1))], &policy)
            .unwrap();

        let Classification::Decided(split) = split else {
            panic!("validated nonroot domain endpoints must order the singleton isolator");
        };
        assert_eq!(split.fragments().len(), 2);
        assert!(has_retained_beziers(&split));

        let BezierSplitFragment2::RetainedBezier {
            start,
            end,
            source_curve,
            start_image,
            end_image,
            ..
        } = &split.fragments()[0]
        else {
            panic!("left fragment must retain exact endpoint images");
        };
        assert_eq!(start.scalar(), Some(&Real::zero()));
        assert!(matches!(end, BezierParameter2::Algebraic(_)));
        assert!(matches!(source_curve, BezierSubcurve2::Quadratic(_)));
        assert!(start_image.is_none());
        assert_endpoint_image(end_image);

        let BezierSplitFragment2::RetainedBezier {
            start,
            end,
            source_curve,
            start_image,
            end_image,
            ..
        } = &split.fragments()[1]
        else {
            panic!("right fragment must retain exact endpoint images");
        };
        assert!(matches!(start, BezierParameter2::Algebraic(_)));
        assert_eq!(end.scalar(), Some(&Real::one()));
        assert!(matches!(source_curve, BezierSubcurve2::Quadratic(_)));
        assert_endpoint_image(start_image);
        assert!(end_image.is_none());
    }
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
            .subcurve_between_exact_with_policy(&start, &end, &policy())
            .map_err(|error| TestCaseError::fail(format!("split failed: {error:?}")))?;

        prop_assert_eq!(subcurve.start(), &curve.point_at(start));
        prop_assert_eq!(subcurve.end(), &curve.point_at(end));
    }
}

#[test]
fn represented_multi_split_materializes_connected_rational_fragments() {
    let curve = crate::RationalBezier2::try_new(
        vec![p(0, 0), p(1, 3), p(3, 3), p(4, 0)],
        vec![Real::from(1), Real::from(2), Real::from(3), Real::from(4)],
    )
    .unwrap();
    let policy = CurveContext::STRICT;
    let split = decided(
        curve
            .split_at_parameters(
                &[
                    BezierParameter2::Exact(q(3, 4)),
                    BezierParameter2::Exact(q(1, 4)),
                    BezierParameter2::Exact(q(1, 4)),
                ],
                &policy,
            )
            .unwrap(),
    );

    assert!(is_fully_materialized(&split));
    assert_eq!(split.fragments().len(), 3);
    let curves = split
        .fragments()
        .iter()
        .map(|fragment| match fragment {
            BezierSplitFragment2::Materialized {
                curve: BezierSubcurve2::Rational(curve),
                ..
            } => curve,
            _ => panic!("represented rational split did not materialize natively"),
        })
        .collect::<Vec<_>>();
    assert_eq!(curves[0].start(), curve.start());
    assert_eq!(curves[0].end(), curves[1].start());
    assert_eq!(curves[1].end(), curves[2].start());
    assert_eq!(curves[2].end(), curve.end());
}

#[test]
fn rational_algebraic_contact_split_retains_exact_derivative_images() {
    let policy = CurveContext::STRICT;
    let half = || (Real::from(1) / Real::from(2)).unwrap();
    let parabola = crate::RationalBezier2::try_new(
        vec![
            Point2::new(Real::zero(), Real::zero()),
            Point2::new(half(), Real::zero()),
            p(1, 1),
        ],
        vec![Real::one(); 3],
    )
    .unwrap();
    let horizontal = crate::RationalBezier2::try_new(
        vec![
            Point2::new(Real::zero(), half()),
            Point2::new(Real::one(), half()),
        ],
        vec![Real::one(); 2],
    )
    .unwrap();
    let crate::CurveIntersectionCandidates2::Candidates {
        first_parameters, ..
    } = parabola
        .intersection_candidates(&horizontal, &policy)
        .unwrap()
    else {
        panic!("parabola crossing did not retain resultant candidates");
    };
    let split = decided(
        parabola
            .split_at_parameters(&first_parameters, &policy)
            .unwrap(),
    );
    assert_eq!(split.fragments().len(), 2);
    assert!(split.fragments().iter().all(|fragment| matches!(
        fragment,
        BezierSplitFragment2::RetainedBezier {
            start_image,
            end_image,
            ..
        } if start_image.as_ref().is_none_or(|image| image.is_exact())
            && end_image.as_ref().is_none_or(|image| image.is_exact())
    )));
    for image in split
        .fragments()
        .iter()
        .flat_map(|fragment| match fragment {
            BezierSplitFragment2::RetainedBezier {
                start_image,
                end_image,
                ..
            } => [start_image.as_ref(), end_image.as_ref()],
            _ => [None, None],
        })
    {
        let Some(image) = image else { continue };
        assert!(image.second_derivative().is_some());
        assert!(image.third_derivative().is_some());
    }
}
