use crate::{
    BezierAlgebraicImageStatus, BezierAlgebraicParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Classification, CurveContext, Point2, QuadraticBezier2,
    RationalQuadraticBezier2, Real,
};
use crate::{CubicBezier2, RationalBezier2};
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
            .derivatives_at_algebraic_parameter(&parameter, 1, &policy())
            .unwrap()
            .map(|mut images| images.remove(0)),
    );
    let second_derivative = decided(
        conic
            .derivatives_at_algebraic_parameter(&parameter, 2, &policy())
            .unwrap()
            .map(|mut images| images.remove(1)),
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
    assert!(tangent.dx().unwrap().representation().unwrap().is_valid());
    assert!(tangent.dy().unwrap().representation().unwrap().is_valid());
    assert_eq!(
        second_derivative.status(),
        BezierAlgebraicImageStatus::Transformed
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
        let tangent = decided(conic.derivatives_at_algebraic_parameter(&parameter, 1, &policy()).unwrap().map(|mut images| images.remove(0)));
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
fn polynomial_point_images_retain_nonrational_source_roots() {
    use crate::{Axis2, CurveCertainty, CurvePoint2};
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
fn high_order_derivative_images_preserve_selected_source_domains() {
    use crate::{HomogeneousControl2, UncertaintyReason};
    use std::cmp::Ordering;

    // C(t)=(t/(1+t),1/(1+t)), authored with a common homogeneous factor F.
    // Keep the original domain: roots of F are still excluded even though
    // the affine quotient would have a removable singularity there.
    fn choose(n: usize, k: usize) -> u64 {
        (0..k.min(n - k)).fold(1, |value, j| value * (n - j) as u64 / (j + 1) as u64)
    }
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for (factor_power, has_pole) in [(0, false), (3, false), (7, false), (3, true)] {
            let degree = 2 * factor_power + if has_pole { 3 } else { 1 };
            let mut factor = vec![Real::zero(); degree];
            for j in 0..=factor_power {
                let coefficient = Real::from(choose(factor_power, j));
                if has_pole {
                    factor[2 * j] = &factor[2 * j] - &coefficient;
                    factor[2 * j + 2] = &factor[2 * j + 2] + r(3) * coefficient;
                } else {
                    factor[2 * j] = coefficient;
                }
            }
            let controls = (0..=degree)
                .map(|i| {
                    let mut x = Real::zero();
                    let mut y = Real::zero();
                    for (k, coefficient) in factor.iter().enumerate() {
                        if k <= i {
                            y += (coefficient * Real::from(choose(i, k))
                                / Real::from(choose(degree, k)))
                            .unwrap();
                        }
                        if k < i {
                            x += (coefficient * Real::from(choose(i, k + 1))
                                / Real::from(choose(degree, k + 1)))
                            .unwrap();
                        }
                    }
                    let weight = &x + &y;
                    HomogeneousControl2::new(x, y, weight)
                })
                .collect();
            let curve =
                decided(RationalBezier2::from_homogeneous_controls(controls, &policy).unwrap());
            // For the pole case, P=(2t^2-1)(3t^2-1). The denominator is not
            // invertible modulo all of P, but it is nonzero at the selected root.
            let source = if has_pole {
                vec![r(1), r(0), r(-5), r(0), r(6)]
            } else {
                vec![r(-1), r(0), r(2)]
            };
            let parameter = isolate(polynomial(source.clone()), interval(q(2, 3), q(3, 4)));
            let images = decided(
                curve
                    .derivatives_at_algebraic_parameter(&parameter, 8, &policy)
                    .unwrap(),
            );
            assert_eq!(images.len(), 8);
            let t = (r(2).sqrt().unwrap() / r(2)).unwrap();
            let denominator = Real::one() + t;
            let mut power = denominator.clone();
            let mut factorial = 1_u64;
            for (index, image) in images.iter().enumerate() {
                let order = index + 1;
                factorial *= order as u64;
                power *= &denominator;
                // d^k(1/(1+t)) = (-1)^k k!/(1+t)^(k+1), and x=1-y.
                let dy = (r(if order % 2 == 0 { 1 } else { -1 }) * Real::from(factorial) / &power)
                    .unwrap();
                for (coordinate, expected) in [
                    (image.dx().unwrap(), -dy.clone()),
                    (image.dy().unwrap(), dy),
                ] {
                    assert_eq!(
                        coordinate.compare_to_real(&expected, &policy),
                        Classification::Decided(Ordering::Equal)
                    );
                }
            }
            if has_pole {
                let pole = isolate(polynomial(source), interval(q(1, 2), q(3, 5)));
                assert!(matches!(
                    curve.derivatives_at_algebraic_parameter(&pole, 8, &policy),
                    Ok(Classification::Uncertain(UncertaintyReason::Boundary))
                ));
            }
        }
    }
}

#[test]
fn selected_derivative_jets_preserve_high_order_rational_tails() {
    use crate::HomogeneousControl2;
    use std::cmp::Ordering;

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for degree in [8, 40] {
            // C(t)=(t/(1+t^degree),1/(1+t^degree)). Its Taylor series
            // at zero has only powers m*degree and m*degree+1. This oracle
            // does not use the quotient-derivative recurrence. At order 80,
            // the recurrence also needs binomials larger than u64::MAX.
            let controls = (0..=degree)
                .map(|i| {
                    HomogeneousControl2::new(
                        q(i, degree),
                        Real::one(),
                        r(if i == degree { 2 } else { 1 }),
                    )
                })
                .collect();
            let curve =
                decided(RationalBezier2::from_homogeneous_controls(controls, &policy).unwrap());
            let parameter = isolate(polynomial(vec![r(0), r(1)]), interval(r(-1), r(1)));
            let images = decided(
                curve
                    .derivatives_at_algebraic_parameter(&parameter, 2 * degree as usize, &policy)
                    .unwrap(),
            );
            assert_eq!(images.len(), 2 * degree as usize);
            let mut factorial = Real::one();
            for (index, image) in images.iter().enumerate() {
                let order = index as i32 + 1;
                factorial *= r(order);
                for (coordinate, exponent) in [
                    (image.dx().unwrap(), order - 1),
                    (image.dy().unwrap(), order),
                ] {
                    let expected = if exponent % degree == 0 {
                        r(if (exponent / degree) % 2 == 0 { 1 } else { -1 }) * &factorial
                    } else {
                        Real::zero()
                    };
                    assert_eq!(
                        coordinate.compare_to_real(&expected, &policy),
                        Classification::Decided(Ordering::Equal),
                        "degree={degree} derivative_order={order}",
                    );
                }
            }
        }
    }
}

#[cfg(test)]
mod finite_parameter_interval_contract {
    use crate::{
        BezierAlgebraicImageStatus, BezierAlgebraicParameter2, BezierParameter2,
        BezierParameterInterval, BezierParameterPolynomial, Classification, Curve2, CurveContext,
        CurveError, CurveParameter2, ExactCurveError, Point2, QuadraticBezier2, RationalBezier2,
        Real,
    };
    use std::cmp::Ordering;

    fn decided<T>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("exact fixture: {reason:?}"),
        }
    }

    fn square_root(sign: i32, policy: &CurveContext) -> BezierAlgebraicParameter2 {
        let (lower, upper) = if sign < 0 { (-2, -1) } else { (1, 2) };
        let interval = decided(
            BezierParameterInterval::try_new(Real::from(lower), Real::from(upper), policy).unwrap(),
        );
        let polynomial = decided(
            BezierParameterPolynomial::try_new_power_basis(
                vec![Real::from(-2), Real::zero(), Real::one()],
                policy,
            )
            .unwrap(),
        );
        decided(BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy).unwrap())
    }

    #[test]
    fn exterior_root_images_preserve_equations_and_native_curve_domains() {
        let source = QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::new((Real::one() / Real::from(2)).unwrap(), Real::zero()),
            Point2::from_values(1, 1),
        );
        let native = Curve2::from(source.clone());
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for sign in [-1, 1] {
                let root = square_root(sign, &policy);
                let parameter = BezierParameter2::Algebraic(root.clone());
                let value = Real::from(sign) * Real::from(2).sqrt().unwrap();
                assert!(matches!(
                    parameter.cmp_by_refinement(&BezierParameter2::Exact(value.clone()), &policy),
                    Ok(Classification::Decided(Ordering::Equal))
                ));
                // The unrestricted polynomial image of P(t)=(t,t²) is
                // (±sqrt(2),2); the authored segment still owns [0,1].
                let image = decided(source.point_at_algebraic_parameter(&root, &policy).unwrap());
                assert_eq!(image.status(), BezierAlgebraicImageStatus::Transformed);
                assert_eq!(
                    image.x().unwrap().compare_to_real(&value, &policy),
                    Classification::Decided(Ordering::Equal)
                );
                assert_eq!(
                    image.y().unwrap().compare_to_real(&Real::from(2), &policy),
                    Classification::Decided(Ordering::Equal)
                );
                assert!(matches!(
                    native.point_at_with_policy(&CurveParameter2::from(parameter), &policy),
                    Err(ExactCurveError::Invalid {
                        cause: CurveError::InvalidCurveParameter,
                        ..
                    })
                ));
            }
        }
    }

    #[test]
    fn positive_unit_weights_do_not_admit_exterior_algebraic_poles() {
        // W(t)=2-t² has Bernstein weights (2,2,1), all positive on
        // the unit span. At either exterior root, the y numerator is 2,
        // so these are actual affine poles rather than removable factors.
        let source = RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(1, 0),
                Point2::from_values(1, 1),
            ],
            vec![Real::from(2), Real::from(2), Real::one()],
        )
        .unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for sign in [-1, 1] {
                let root = square_root(sign, &policy);
                let image = source.point_at_algebraic_parameter(&root, &policy).unwrap();
                assert!(matches!(
                    image,
                    Classification::Uncertain(crate::UncertaintyReason::Boundary)
                ));
            }
        }
    }
}

mod rational_resultant_projection {
    use super::*;
    use crate::{
        BezierParameter2, Curve2, CurveIntersectionCandidates2, RationalBezierIntersectionContacts2,
    };

    #[test]
    fn rational_resultant_retains_algebraic_parameter_projections() {
        let policy = CurveContext::STRICT;
        let parabola = RationalBezier2::try_new(
            vec![Point2::new(r(0), r(0)), Point2::new(q(1, 2), r(0)), p(1, 1)],
            vec![r(1), r(1), r(1)],
        )
        .unwrap();
        let horizontal = RationalBezier2::try_new(
            vec![Point2::new(r(0), q(1, 2)), Point2::new(r(1), q(1, 2))],
            vec![r(1), r(1)],
        )
        .unwrap();
        let candidates = parabola
            .intersection_candidates(&horizontal, &policy)
            .unwrap();
        let CurveIntersectionCandidates2::Candidates {
            first_parameters,
            second_parameters,
        } = candidates
        else {
            panic!("parabola crossing did not retain resultant candidates");
        };
        assert!(matches!(
            first_parameters.as_slice(),
            [BezierParameter2::Algebraic(_)]
        ));
        assert!(matches!(
            second_parameters.as_slice(),
            [BezierParameter2::Algebraic(_)]
        ));
        let BezierParameter2::Algebraic(first_parameter) = &first_parameters[0] else {
            unreachable!("asserted algebraic parameter")
        };
        let image = decided(
            parabola
                .point_at_algebraic_parameter(first_parameter, &policy)
                .unwrap(),
        );
        assert_eq!(
            image.status(),
            BezierAlgebraicImageStatus::Transformed,
            "{image:?}"
        );
        assert!(
            image
                .x()
                .and_then(|coordinate| coordinate.representation())
                .is_some()
        );
        assert!(
            image
                .y()
                .and_then(|coordinate| coordinate.representation())
                .is_some()
        );
        let derivatives = decided(
            parabola
                .derivatives_at_algebraic_parameter(first_parameter, 3, &policy)
                .unwrap(),
        );
        assert_eq!(derivatives.len(), 3);
        assert!(
            derivatives
                .iter()
                .all(|derivative| derivative.status() == BezierAlgebraicImageStatus::Transformed)
        );
        let represented_coordinate = |order: usize, x_axis: bool| {
            let coordinate = if x_axis {
                derivatives[order - 1].dx()
            } else {
                derivatives[order - 1].dy()
            };
            coordinate
                .and_then(|coordinate| coordinate.representation())
                .and_then(|coordinate| coordinate.exact_point_witness())
                .cloned()
        };
        assert_eq!(represented_coordinate(2, true), Some(r(0)));
        assert_eq!(represented_coordinate(2, false), Some(r(2)));
        assert_eq!(represented_coordinate(3, true), Some(r(0)));
        assert_eq!(represented_coordinate(3, false), Some(r(0)));
        let contacts = parabola
            .intersection_contacts(&horizontal, &policy)
            .unwrap();
        let RationalBezierIntersectionContacts2::Contacts(contacts) = contacts else {
            panic!("algebraic resultant candidates did not replay completely");
        };
        assert_eq!(contacts.len(), 1);
        assert!(contacts[0].first_parameter().scalar().is_none());
        assert!(contacts[0].second_parameter().scalar().is_none());
        assert!((contacts[0].point()).coordinates().is_none());

        let topology = Curve2::from(parabola)
            .intersection_topology_with_policy(&Curve2::from(horizontal), &policy)
            .unwrap()
            .into_value();
        assert_eq!(topology.result().contacts().len(), 1);
        assert_eq!(topology.first().len(), 2);
        assert_eq!(topology.second().len(), 2);
    }
}
