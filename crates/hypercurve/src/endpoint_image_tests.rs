use crate::{
    BezierAlgebraicImageStatus, BezierAlgebraicParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Classification, CurveContext, Point2, QuadraticBezier2,
    RationalQuadraticBezier2, Real,
};
use crate::{CubicBezier2, RationalBezier2};

fn policy() -> CurveContext {
    CurveContext::STRICT
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
    }
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

fn p(x: i32, y: i32) -> Point2 {
    Point2::from_values(x, y)
}

fn polynomial(coefficients: Vec<Real>) -> BezierParameterPolynomial {
    decided(BezierParameterPolynomial::try_new_power_basis(coefficients, &policy()).unwrap())
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn r(value: i32) -> Real {
    value.into()
}

fn sqrt_half_parameter() -> BezierAlgebraicParameter2 {
    isolate(polynomial(vec![r(-1), r(0), r(2)]), interval(q(1, 2), r(1)))
}

#[test]
fn rational_point_images_require_finite_affine_coordinates() {
    use crate::{BezierAlgebraicEndpointImage2, CurvePoint2, UncertaintyReason};

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

#[test]
fn polynomial_point_images_share_exact_replay_across_coefficient_domains() {
    use crate::{BezierAlgebraicEndpointImage2, CurveCertainty, CurvePoint2};

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
fn polynomial_endpoint_derivatives_replay_exact_values_in_the_shared_carrier() {
    use crate::BezierAlgebraicEndpointImage2;
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
    use crate::{BezierAlgebraicEndpointImage2, UncertaintyReason};
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
            let check = |image: &crate::RationalBezierAlgebraicTangentImage2, order: usize| {
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
