use hypercurve::{
    BezierAlgebraicImageStatus, BezierAlgebraicParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Classification, CurveContext, Point2, QuadraticBezier2,
    RationalQuadraticBezier2, Real,
};
use hypercurve::{CubicBezier2, RationalBezier2};
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn policy() -> CurveContext {
    CurveContext::STRICT
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::from_values(x, y)
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
    }
}

fn polynomial(coefficients: Vec<Real>) -> BezierParameterPolynomial {
    decided(BezierParameterPolynomial::try_new_power_basis(coefficients, &policy()).unwrap())
}

fn interval(start: Real, end: Real) -> BezierParameterInterval {
    decided(BezierParameterInterval::try_new(start, end, &policy()).unwrap())
}

fn isolate(
    polynomial: BezierParameterPolynomial,
    interval: BezierParameterInterval,
) -> BezierAlgebraicParameter2 {
    decided(BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy()).unwrap())
}

fn sqrt_half_parameter() -> BezierAlgebraicParameter2 {
    isolate(polynomial(vec![r(-1), r(0), r(2)]), interval(q(1, 2), r(1)))
}

#[test]
fn quadratic_point_and_tangent_images_retain_algebraic_coordinate_evidence() {
    let curve = QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2));
    let parameter = sqrt_half_parameter();

    let point = decided(
        curve
            .point_at_algebraic_parameter(&parameter, &policy())
            .unwrap(),
    );
    let tangent = decided(
        curve
            .tangent_at_algebraic_parameter(&parameter, &policy())
            .unwrap(),
    );

    assert_eq!(point.status(), BezierAlgebraicImageStatus::Transformed);
    assert_eq!(
        point.x().unwrap().numerator_coefficients(),
        &[r(0), r(0), r(1)]
    );
    assert!(point.x().unwrap().representation().unwrap().is_valid());
    assert_eq!(
        point.y().unwrap().numerator_coefficients(),
        &[r(0), r(2), r(0)]
    );
    assert!(
        point
            .y()
            .unwrap()
            .representation()
            .unwrap()
            .exact_point_witness()
            .is_none()
    );

    assert_eq!(tangent.status(), BezierAlgebraicImageStatus::Transformed);
    assert_eq!(
        tangent.dx().unwrap().numerator_coefficients(),
        &[r(0), r(2)]
    );
    assert_eq!(tangent.dy().unwrap().numerator_coefficients(), &[r(2)]);
    assert_eq!(
        tangent
            .dy()
            .unwrap()
            .representation()
            .unwrap()
            .exact_point_witness(),
        Some(&r(2))
    );
}

#[test]
fn cubic_point_and_tangent_images_use_power_basis_resultants() {
    let curve = CubicBezier2::new(p(0, 0), p(0, 1), p(0, 2), p(1, 3));
    let parameter = sqrt_half_parameter();

    let point = decided(
        curve
            .point_at_algebraic_parameter(&parameter, &policy())
            .unwrap(),
    );
    let tangent = decided(
        curve
            .tangent_at_algebraic_parameter(&parameter, &policy())
            .unwrap(),
    );

    assert_eq!(point.status(), BezierAlgebraicImageStatus::Transformed);
    assert_eq!(
        point.x().unwrap().numerator_coefficients(),
        &[r(0), r(0), r(0), r(1)]
    );
    assert_eq!(
        point.y().unwrap().numerator_coefficients(),
        &[r(0), r(3), r(0), r(0)]
    );
    assert_eq!(tangent.status(), BezierAlgebraicImageStatus::Transformed);
    assert_eq!(tangent.dx().unwrap().numerator_coefficients(), &[q(3, 2)]);
    assert!(tangent.dx().unwrap().representation().unwrap().is_valid());
    assert_eq!(tangent.dy().unwrap().numerator_coefficients(), &[r(3)]);
}

#[test]
fn nonmonotone_coordinate_image_is_certified_without_sampling() {
    let curve = QuadraticBezier2::new(
        Point2::new(q(9, 16), r(0)),
        Point2::new(q(-3, 16), r(1)),
        Point2::new(q(1, 16), r(2)),
    );
    let parameter = sqrt_half_parameter();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let point = decided(
            curve
                .point_at_algebraic_parameter(&parameter, &policy)
                .unwrap(),
        );

        assert_eq!(point.status(), BezierAlgebraicImageStatus::Transformed);
        let x = point.x().unwrap();
        assert_eq!(x.numerator_coefficients(), &[q(9, 16), q(-3, 2), r(1)]);
        assert!(x.representation().unwrap().is_valid());
        assert_eq!(
            x.compare_to_real(&Real::zero(), &policy),
            Classification::Decided(std::cmp::Ordering::Greater)
        );
        assert_eq!(
            x.compare_to_real(&q(1, 16), &policy),
            Classification::Decided(std::cmp::Ordering::Less)
        );
        assert_eq!(
            point.y().unwrap().numerator_coefficients(),
            &[r(0), r(2), r(0)]
        );
        assert!(point.message().is_none());
    }
}

#[test]
fn rational_quadratic_point_and_tangent_images_retain_quotient_evidence() {
    let conic =
        RationalQuadraticBezier2::try_new(p(0, 0), p(2, 4), p(6, 0), r(1), r(2), r(3)).unwrap();
    let parameter = sqrt_half_parameter();

    let point = decided(
        conic
            .point_at_algebraic_parameter(&parameter, &policy())
            .unwrap(),
    );
    let tangent = decided(
        conic
            .tangent_at_algebraic_parameter(&parameter, &policy())
            .unwrap(),
    );
    let second_derivative = decided(
        conic
            .second_derivative_at_algebraic_parameter(&parameter, &policy())
            .unwrap(),
    );

    assert_eq!(std::mem::size_of_val(&point), std::mem::size_of::<usize>());
    assert_eq!(
        std::mem::size_of_val(&tangent),
        std::mem::size_of::<usize>()
    );
    assert_eq!(point.clone(), point);
    assert_eq!(tangent.clone(), tangent);
    assert_eq!(point.status(), BezierAlgebraicImageStatus::Transformed);
    assert_eq!(
        point.x().unwrap().numerator_coefficients(),
        &[r(0), r(8), r(10)]
    );
    assert_eq!(
        point.y().unwrap().numerator_coefficients(),
        &[r(0), r(16), r(-16)]
    );
    assert_eq!(
        point.x().unwrap().denominator_coefficients(),
        &[r(1), r(2), r(0)]
    );
    assert!(point.x().unwrap().representation().unwrap().is_valid());
    assert!(point.y().unwrap().representation().unwrap().is_valid());

    assert_eq!(tangent.status(), BezierAlgebraicImageStatus::Transformed);
    assert_eq!(
        tangent.dx().unwrap().denominator_coefficients(),
        &[r(1), r(4), r(4), r(0), r(0)]
    );
    assert!(tangent.dx().unwrap().representation().unwrap().is_valid());
    assert!(tangent.dy().unwrap().representation().unwrap().is_valid());
    assert_eq!(
        second_derivative.status(),
        BezierAlgebraicImageStatus::Transformed
    );
    assert_eq!(
        second_derivative.dx().unwrap().denominator_coefficients(),
        &[r(1), r(6), r(12), r(8), r(0), r(0), r(0)]
    );
    assert!(
        second_derivative
            .dx()
            .unwrap()
            .representation()
            .unwrap()
            .is_valid()
    );
    assert!(
        second_derivative
            .dy()
            .unwrap()
            .representation()
            .unwrap()
            .is_valid()
    );
}

#[test]
fn rational_point_image_transforms_exact_real_linear_root() {
    let conic =
        RationalQuadraticBezier2::try_new(p(0, 0), p(2, 4), p(6, 0), r(1), r(2), r(3)).unwrap();
    let parameter = isolate(
        polynomial(vec![r(-1), Real::pi()]),
        interval(q(1, 4), q(1, 2)),
    );

    let point = decided(
        conic
            .point_at_algebraic_parameter(&parameter, &policy())
            .unwrap(),
    );

    assert_eq!(point.status(), BezierAlgebraicImageStatus::Transformed);
    assert!(point.parameter().is_valid());
    let exact_parameter = point
        .parameter()
        .exact_point_witness()
        .expect("a linear exact-Real polynomial has an exact point witness");
    assert_eq!(exact_parameter, &(Real::one() / Real::pi()).unwrap());
    assert!(exact_parameter.exact_rational_ref().is_none());
    let Classification::Decided(exact_point) = conic.point_at(exact_parameter.clone(), &policy())
    else {
        panic!("the exact parameter must evaluate to an affine conic point");
    };
    let x = point.x().unwrap().representation().unwrap();
    let y = point.y().unwrap().representation().unwrap();
    assert!(x.exact_point_witness().is_some());
    assert!(y.exact_point_witness().is_some());
    assert_eq!(x.interval.lower, x.interval.upper);
    assert_eq!(y.interval.lower, y.interval.upper);
    for coordinate in [point.x().unwrap(), point.y().unwrap()] {
        let representation = coordinate.representation().unwrap();
        assert_eq!(
            coordinate.compare_to_real(
                representation.exact_point_witness().unwrap(),
                &CurveContext::STRICT,
            ),
            Classification::Decided(std::cmp::Ordering::Equal),
        );
    }
    assert_eq!(
        point
            .x()
            .unwrap()
            .compare_to_real(exact_point.x(), &CurveContext::STRICT),
        Classification::Decided(std::cmp::Ordering::Equal),
    );
    assert_eq!(
        point
            .y()
            .unwrap()
            .compare_to_real(exact_point.y(), &CurveContext::STRICT),
        Classification::Decided(std::cmp::Ordering::Equal),
    );
    assert!(point.retained_parameter().is_none());
    assert!(point.message().is_none());
}

#[test]
fn conic_point_images_reuse_exact_evaluation_across_weight_charts() {
    for translation in [Real::zero(), Real::pi()] {
        for weights in [
            [r(1), r(2), r(3)],
            [r(2), r(3), r(5)],
            [r(1), q(-1, 2), r(1)],
            [r(2), Real::pi(), r(3)],
            [r(1), Real::pi(), r(1)],
            [r(1), q(1, 2).sqrt().unwrap(), r(1)],
        ] {
            for reverse in [false, true] {
                for swap_axes in [false, true] {
                    let mut controls = [
                        Point2::new(translation.clone(), r(0)),
                        Point2::new(r(2) + &translation, r(4)),
                        Point2::new(r(6) + &translation, r(0)),
                    ];
                    if swap_axes {
                        controls =
                            controls.map(|point| Point2::new(point.y().clone(), point.x().clone()));
                    }
                    let mut weights = weights.clone();
                    if reverse {
                        controls.reverse();
                        weights.reverse();
                    }
                    let conic = RationalQuadraticBezier2::try_new(
                        controls[0].clone(),
                        controls[1].clone(),
                        controls[2].clone(),
                        weights[0].clone(),
                        weights[1].clone(),
                        weights[2].clone(),
                    )
                    .unwrap();
                    let promoted = RationalBezier2::from(conic.clone());
                    for t in [
                        q(1, 2),
                        (r(1) / Real::pi()).unwrap(),
                        (r(2).sqrt().unwrap() / r(2)).unwrap(),
                    ] {
                        let parameter =
                            isolate(polynomial(vec![-t.clone(), r(1)]), interval(r(0), r(1)));
                        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
                            let image = decided(
                                conic
                                    .point_at_algebraic_parameter(&parameter, &policy)
                                    .unwrap(),
                            );
                            assert_eq!(image.status(), BezierAlgebraicImageStatus::Transformed);
                            for point in [
                                decided(conic.point_at(t.clone(), &policy)),
                                promoted.point_at(&t, &policy).unwrap(),
                            ] {
                                for (coordinate, value) in [
                                    (image.x().unwrap(), point.x()),
                                    (image.y().unwrap(), point.y()),
                                ] {
                                    // Equality must be certified across independently evaluated
                                    // public representations, including a transcendental weight
                                    // and the pole-free mixed-sign chart W = 1 - 3t + 3t^2.
                                    assert_eq!(
                                        coordinate.compare_to_real(value, &CurveContext::STRICT),
                                        Classification::Decided(std::cmp::Ordering::Equal),
                                    );
                                    for (other, expected) in [
                                        (value - r(1), std::cmp::Ordering::Greater),
                                        (value + r(1), std::cmp::Ordering::Less),
                                    ] {
                                        assert_eq!(
                                            coordinate
                                                .compare_to_real(&other, &CurveContext::STRICT),
                                            Classification::Decided(expected),
                                        );
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn rational_image_cache_keeps_curve_family_certificate_shapes_distinct() {
    let controls = vec![p(0, 0), p(2, 4), p(6, 0)];
    let weights = vec![r(1), r(2), r(3)];
    let general = RationalBezier2::try_new(controls.clone(), weights.clone()).unwrap();
    let conic = RationalQuadraticBezier2::try_new(
        controls[0].clone(),
        controls[1].clone(),
        controls[2].clone(),
        weights[0].clone(),
        weights[1].clone(),
        weights[2].clone(),
    )
    .unwrap();
    let parameter = sqrt_half_parameter();

    let general_tangent = decided(
        general
            .tangent_at_algebraic_parameter(&parameter, &policy())
            .unwrap(),
    );
    let conic_tangent = decided(
        conic
            .tangent_at_algebraic_parameter(&parameter, &policy())
            .unwrap(),
    );

    assert_eq!(
        general_tangent.dx().unwrap().denominator_coefficients(),
        &[r(3), r(4)]
    );
    assert_eq!(
        conic_tangent.dx().unwrap().denominator_coefficients(),
        &[r(1), r(4), r(4), r(0), r(0)]
    );
}

#[test]
fn rational_point_images_require_finite_affine_coordinates() {
    use hypercurve::{BezierAlgebraicEndpointImage2, CurvePoint2, UncertaintyReason};

    // D(t) = (1-2t)^2, with y numerator -1/2 at the pole.
    // There is no affine point to admit, even though the parameter is exact.
    let conic =
        RationalQuadraticBezier2::try_new(p(0, 0), p(1, 1), p(2, 0), r(1), r(-1), r(1)).unwrap();
    let general = RationalBezier2::from(conic.clone());
    let pole = isolate(polynomial(vec![r(-1), r(2)]), interval(q(2, 5), q(3, 5)));
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for _ in 0..2 {
            for result in [
                conic.point_at_algebraic_parameter(&pole, &policy),
                general.point_at_algebraic_parameter(&pole, &policy),
            ] {
                assert!(matches!(
                    result,
                    Ok(Classification::Uncertain(UncertaintyReason::Boundary))
                ));
            }
            for result in [
                BezierAlgebraicEndpointImage2::rational_quadratic(&conic, &pole, &policy),
                BezierAlgebraicEndpointImage2::rational(&general, &pole, &policy),
            ] {
                assert!(matches!(
                    result,
                    Ok(Classification::Uncertain(UncertaintyReason::Boundary))
                ));
            }
        }
        for (parameter_value, expected) in [
            (q(1, 4), Point2::new(r(-1), q(-3, 2))),
            (q(3, 4), Point2::new(r(3), q(-3, 2))),
        ] {
            let parameter = isolate(
                polynomial(vec![-parameter_value, r(1)]),
                interval(r(0), r(1)),
            );
            for image in [
                decided(
                    conic
                        .point_at_algebraic_parameter(&parameter, &policy)
                        .unwrap(),
                ),
                decided(
                    general
                        .point_at_algebraic_parameter(&parameter, &policy)
                        .unwrap(),
                ),
            ] {
                let point = CurvePoint2::from(image);
                assert!(matches!(
                    point
                        .coincides_with(&CurvePoint2::from(expected.clone()), &policy)
                        .value,
                    Classification::Decided(true)
                ));
            }
        }
    }
}

proptest! {
    #[test]
    fn linear_coordinate_images_match_exact_midpoint_values(
        x0 in -8_i32..=8,
        x1 in -8_i32..=8,
        x2 in -8_i32..=8,
        y0 in -8_i32..=8,
        y1 in -8_i32..=8,
        y2 in -8_i32..=8,
    ) {
        let curve = QuadraticBezier2::new(
            Point2::from_values(x0, y0),
            Point2::from_values(x1, y1),
            Point2::from_values(x2, y2),
        );
        let parameter = isolate(
            polynomial(vec![r(-1), r(2)]),
            interval(q(2, 5), q(3, 5)),
        );

        let point = decided(curve.point_at_algebraic_parameter(&parameter, &policy()).unwrap());
        let tangent = decided(curve.tangent_at_algebraic_parameter(&parameter, &policy()).unwrap());
        let exact_point = curve.point_at(q(1, 2));

        prop_assert_eq!(point.status(), BezierAlgebraicImageStatus::Transformed);
        prop_assert_eq!(
            point.x().unwrap().representation().unwrap().exact_point_witness(),
            Some(exact_point.x())
        );
        prop_assert_eq!(
            point.y().unwrap().representation().unwrap().exact_point_witness(),
            Some(exact_point.y())
        );
        prop_assert_eq!(tangent.status(), BezierAlgebraicImageStatus::Transformed);
    }

    #[test]
    fn rational_line_image_matches_exact_midpoint_values(
        x0 in -8_i32..=8,
        x1 in -8_i32..=8,
        x2 in -8_i32..=8,
        y0 in -8_i32..=8,
        y1 in -8_i32..=8,
        y2 in -8_i32..=8,
        w1 in 1_i32..=8,
    ) {
        let conic = RationalQuadraticBezier2::try_new(
            Point2::from_values(x0, y0),
            Point2::from_values(x1, y1),
            Point2::from_values(x2, y2),
            r(1),
            r(w1),
            r(1),
        ).unwrap();
        let parameter = isolate(
            polynomial(vec![r(-1), r(2)]),
            interval(q(2, 5), q(3, 5)),
        );

        let point = decided(conic.point_at_algebraic_parameter(&parameter, &policy()).unwrap());
        let tangent = decided(conic.tangent_at_algebraic_parameter(&parameter, &policy()).unwrap());
        let exact_point = match conic.point_at(q(1, 2), &policy()) {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => panic!("midpoint unexpectedly uncertain: {reason:?}"),
        };

        prop_assert_eq!(point.status(), BezierAlgebraicImageStatus::Transformed);
        prop_assert_eq!(
            point.x().unwrap().representation().unwrap().exact_point_witness(),
            Some(exact_point.x())
        );
        prop_assert_eq!(
            point.y().unwrap().representation().unwrap().exact_point_witness(),
            Some(exact_point.y())
        );
        prop_assert_eq!(tangent.status(), BezierAlgebraicImageStatus::Transformed);
    }
}

#[test]
fn polynomial_point_images_share_exact_replay_across_coefficient_domains() {
    use hypercurve::{BezierAlgebraicEndpointImage2, CurveCertainty, CurvePoint2};

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let root = sqrt_half_parameter();
        let sqrt_two = Real::from(2).sqrt().unwrap();
        for coefficient in [sqrt_two.clone(), Real::pi()] {
            let quadratic =
                QuadraticBezier2::new(p(0, 0), p(0, 1), Point2::new(coefficient.clone(), r(2)));
            let cubic = CubicBezier2::new(
                p(0, 0),
                p(0, 1),
                p(0, 2),
                Point2::new(coefficient.clone(), r(3)),
            );
            let quadratic_endpoint = decided(
                BezierAlgebraicEndpointImage2::quadratic(&quadratic, &root, &policy).unwrap(),
            );
            let cubic_endpoint =
                decided(BezierAlgebraicEndpointImage2::cubic(&cubic, &root, &policy).unwrap());
            for (image, endpoint, expected) in [
                (
                    decided(
                        quadratic
                            .point_at_algebraic_parameter(&root, &policy)
                            .unwrap(),
                    ),
                    &quadratic_endpoint,
                    Point2::new(&coefficient * q(1, 2), sqrt_two.clone()),
                ),
                (
                    decided(cubic.point_at_algebraic_parameter(&root, &policy).unwrap()),
                    &cubic_endpoint,
                    Point2::new(&coefficient * &sqrt_two * q(1, 4), &sqrt_two * q(3, 2)),
                ),
            ] {
                for point in [image, decided(endpoint.point().unwrap()).clone()] {
                    let outcome = CurvePoint2::from(point)
                        .coincides_with(&CurvePoint2::from(expected.clone()), &policy);
                    assert_eq!(outcome.certainty, CurveCertainty::Certified);
                    assert_eq!(outcome.value, Classification::Decided(true));
                }
            }
        }
    }
}

#[test]
fn polynomial_point_images_retain_nonrational_source_roots() {
    use hypercurve::{Axis2, CurveCertainty, CurvePoint2};
    use std::cmp::Ordering;

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let root = isolate(
            polynomial(vec![-Real::pi(), r(0), r(4)]),
            interval(q(3, 4), r(1)),
        );
        let curve = QuadraticBezier2::new(p(0, 0), p(0, 1), Point2::new(Real::pi(), r(2)));
        let image = decided(curve.point_at_algebraic_parameter(&root, &policy).unwrap());
        let point = CurvePoint2::from(image);
        for (axis, coordinate, expected) in [
            (Axis2::X, Real::pi() * Real::pi() * q(1, 4), Ordering::Equal),
            (Axis2::Y, r(1), Ordering::Greater),
            (Axis2::Y, r(2), Ordering::Less),
        ] {
            let reference = CurvePoint2::from(Point2::new(coordinate.clone(), coordinate));
            let outcome = point.compare_coordinate(&reference, axis, &policy).unwrap();
            assert_eq!(outcome.certainty, CurveCertainty::Certified);
            assert_eq!(outcome.value, Classification::Decided(expected));
        }
    }
}

#[test]
fn polynomial_endpoint_derivatives_replay_exact_values_in_the_shared_carrier() {
    use hypercurve::BezierAlgebraicEndpointImage2;
    use std::cmp::Ordering;

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let root = sqrt_half_parameter();
        let sqrt_two = r(2).sqrt().unwrap();
        for coefficient in [sqrt_two.clone(), Real::pi()] {
            let quadratic =
                QuadraticBezier2::new(p(0, 0), p(0, 1), Point2::new(coefficient.clone(), r(2)));
            let cubic = CubicBezier2::new(
                p(0, 0),
                p(0, 1),
                p(0, 2),
                Point2::new(coefficient.clone(), r(3)),
            );
            let quadratic_endpoint = decided(
                BezierAlgebraicEndpointImage2::quadratic(&quadratic, &root, &policy).unwrap(),
            );
            let cubic_endpoint =
                decided(BezierAlgebraicEndpointImage2::cubic(&cubic, &root, &policy).unwrap());
            assert!(quadratic_endpoint.is_exact());
            assert!(cubic_endpoint.is_exact());
            assert!(quadratic_endpoint.third_derivative().is_none());
            for (direct, retained, dx, dy) in [
                (
                    decided(
                        quadratic
                            .tangent_at_algebraic_parameter(&root, &policy)
                            .unwrap(),
                    ),
                    decided(quadratic_endpoint.tangent().unwrap()),
                    &coefficient * &sqrt_two,
                    r(2),
                ),
                (
                    decided(
                        quadratic
                            .second_derivative_at_algebraic_parameter(&root, &policy)
                            .unwrap(),
                    ),
                    quadratic_endpoint.second_derivative().unwrap(),
                    &coefficient * r(2),
                    r(0),
                ),
                (
                    decided(
                        cubic
                            .tangent_at_algebraic_parameter(&root, &policy)
                            .unwrap(),
                    ),
                    decided(cubic_endpoint.tangent().unwrap()),
                    &coefficient * q(3, 2),
                    r(3),
                ),
                (
                    decided(
                        cubic
                            .second_derivative_at_algebraic_parameter(&root, &policy)
                            .unwrap(),
                    ),
                    cubic_endpoint.second_derivative().unwrap(),
                    &coefficient * &sqrt_two * r(3),
                    r(0),
                ),
                (
                    decided(
                        cubic
                            .third_derivative_at_algebraic_parameter(&root, &policy)
                            .unwrap(),
                    ),
                    cubic_endpoint.third_derivative().unwrap(),
                    &coefficient * r(6),
                    r(0),
                ),
            ] {
                for image in [&direct, retained] {
                    assert_eq!(image.status(), BezierAlgebraicImageStatus::Transformed);
                    for (actual, expected) in
                        [(image.dx().unwrap(), &dx), (image.dy().unwrap(), &dy)]
                    {
                        assert_eq!(
                            actual.compare_to_real(expected, &policy),
                            Classification::Decided(Ordering::Equal)
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn rational_derivative_images_require_finite_affine_domain() {
    use hypercurve::{BezierAlgebraicEndpointImage2, UncertaintyReason};
    use std::cmp::Ordering;

    // With d = 2t - 1, this curve is (2t/d, 1/2 - 1/(2d^2)).
    // Its first three derivatives are (-2/d^2, 2/d^3),
    // (8/d^3, -12/d^4), and (-48/d^4, 96/d^5).
    let conic =
        RationalQuadraticBezier2::try_new(p(0, 0), p(1, 1), p(2, 0), r(1), r(-1), r(1)).unwrap();
    let general = RationalBezier2::from(conic.clone());
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let pole = isolate(polynomial(vec![r(-1), r(2)]), interval(r(0), r(1)));
        for _ in 0..2 {
            for image in [
                conic.tangent_at_algebraic_parameter(&pole, &policy),
                general.tangent_at_algebraic_parameter(&pole, &policy),
                conic.second_derivative_at_algebraic_parameter(&pole, &policy),
            ] {
                assert!(matches!(
                    image,
                    Ok(Classification::Uncertain(UncertaintyReason::Boundary))
                ));
            }
            for order in [1, 2, 3] {
                for images in [
                    conic.derivatives_at_algebraic_parameter(&pole, order, &policy),
                    general.derivatives_at_algebraic_parameter(&pole, order, &policy),
                ] {
                    assert!(matches!(
                        images,
                        Ok(Classification::Uncertain(UncertaintyReason::Boundary))
                    ));
                }
            }
        }
        for images in [
            conic.derivatives_at_algebraic_parameter(&pole, 0, &policy),
            general.derivatives_at_algebraic_parameter(&pole, 0, &policy),
        ] {
            assert!(decided(images.unwrap()).is_empty());
        }
        for (value, expected) in [
            (q(1, 4), [(-8, -16), (-64, -192), (-768, -3072)]),
            (q(3, 4), [(-8, 16), (64, -192), (-768, 3072)]),
        ] {
            let parameter = isolate(polynomial(vec![-value, r(1)]), interval(r(0), r(1)));
            let check = |image: &hypercurve::RationalBezierAlgebraicTangentImage2, order: usize| {
                let (dx, dy) = expected[order - 1];
                for (coordinate, value) in [(image.dx().unwrap(), dx), (image.dy().unwrap(), dy)] {
                    assert_eq!(
                        coordinate.compare_to_real(&r(value), &policy),
                        Classification::Decided(Ordering::Equal)
                    );
                }
            };
            for _ in 0..2 {
                for images in [
                    conic.derivatives_at_algebraic_parameter(&parameter, 3, &policy),
                    general.derivatives_at_algebraic_parameter(&parameter, 3, &policy),
                ] {
                    let images = decided(images.unwrap());
                    assert_eq!(images.len(), 3);
                    for (order, image) in images.iter().enumerate() {
                        check(image, order + 1);
                    }
                }
                for image in [
                    conic.tangent_at_algebraic_parameter(&parameter, &policy),
                    general.tangent_at_algebraic_parameter(&parameter, &policy),
                ] {
                    check(&decided(image.unwrap()), 1);
                }
                check(
                    &decided(
                        conic
                            .second_derivative_at_algebraic_parameter(&parameter, &policy)
                            .unwrap(),
                    ),
                    2,
                );
                for endpoint in [
                    BezierAlgebraicEndpointImage2::rational_quadratic(&conic, &parameter, &policy),
                    BezierAlgebraicEndpointImage2::rational(&general, &parameter, &policy),
                ] {
                    let endpoint = decided(endpoint.unwrap());
                    check(decided(endpoint.tangent().unwrap()), 1);
                    check(endpoint.second_derivative().unwrap(), 2);
                    check(endpoint.third_derivative().unwrap(), 3);
                    assert!(endpoint.is_exact());
                }
            }
        }
    }
}
