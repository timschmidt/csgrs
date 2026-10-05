mod support;
use hypercurve::{
    Curve2, CurveContext, ExactCurveError, Point2, RationalBezier2, Real, UncertaintyReason,
};

fn rational(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

#[test]
fn rational_line_derivatives_exceed_machine_binomial_orders() {
    let curve = RationalBezier2::try_new(
        vec![
            Point2::new(Real::zero(), Real::zero()),
            Point2::new(Real::one(), Real::from(2)),
        ],
        vec![Real::from(2), Real::from(3)],
    )
    .unwrap();

    for parameter in [Real::zero(), rational(1, 2), Real::one()] {
        let derivatives = curve
            .derivatives_at(&parameter, 128)
            .expect("a degree-one denominator has derivatives of every order");
        assert_eq!(derivatives.len(), 128);
        let unified = Curve2::from(curve.clone());
        assert_eq!(
            crate::support::under(&CurveContext::STRICT, || unified
                .derivatives_at(&parameter.clone().into(), 128))
            .expect("the top-level curve must preserve high derivative completeness")
            .into_value(),
            derivatives
                .iter()
                .cloned()
                .map(hypercurve::CurveVector2::from)
                .collect::<Vec<_>>()
        );
        // x(t)=3t/(2+t), so x^(k)=6*(-1)^(k-1)*k!/(2+t)^(k+1).
        let denominator = Real::from(2) + &parameter;
        let mut expected = (Real::from(6) / (&denominator * &denominator)).unwrap();
        for (index, derivative) in derivatives.iter().enumerate() {
            if index > 0 {
                expected = (-expected * Real::from((index + 1) as u64) / &denominator).unwrap();
            }
            assert_eq!(derivative.dx(), &expected, "order {}", index + 1);
            assert_eq!(derivative.dy(), &(Real::from(2) * &expected));
        }
    }
}

#[test]
fn dense_denominator_derivatives_use_exact_large_binomials() {
    const DEGREE: usize = 68;
    // Bernstein weights 2^i give W(t)=(1+t)^DEGREE; weighted x
    // controls are all 1, so x(t)=1/(1+t)^DEGREE. This exercises
    // genuinely nonzero high denominator derivatives, not only zero tails.
    let mut weight = Real::one();
    let mut points = Vec::new();
    let mut weights = Vec::new();
    for _ in 0..=DEGREE {
        points.push(Point2::new((Real::one() / &weight).unwrap(), Real::zero()));
        weights.push(weight.clone());
        weight *= Real::from(2);
    }
    let curve = RationalBezier2::try_new(points, weights).unwrap();
    for parameter in [Real::zero(), rational(1, 2), Real::one()] {
        let derivatives = curve
            .derivatives_at(&parameter, 80)
            .expect("dense exact coefficients must not be limited by u64 binomials");
        let denominator = Real::one() + &parameter;
        let mut power = Real::one();
        for _ in 0..DEGREE {
            power *= &denominator;
        }
        let mut expected = (Real::one() / power).unwrap();
        for (index, derivative) in derivatives.iter().enumerate() {
            // The independent closed form uses the rising factorial
            // DEGREE*(DEGREE+1)*...*(DEGREE+k-1).
            expected = (-expected * Real::from((DEGREE + index) as u64) / &denominator).unwrap();
            assert_eq!(derivative.dx(), &expected, "order {}", index + 1);
            assert_eq!(derivative.dy(), &Real::zero());
        }
    }
}

#[test]
fn common_transcendental_weight_scale_preserves_high_derivatives() {
    let curve = RationalBezier2::try_new(
        vec![
            Point2::new(Real::zero(), Real::zero()),
            Point2::new(Real::one(), Real::from(2)),
        ],
        vec![Real::from(2) * Real::pi(), Real::from(3) * Real::pi()],
    )
    .unwrap();
    let parameter = rational(1, 3);
    let derivatives = curve
        .derivatives_at(&parameter, 80)
        .expect("a common exact symbolic weight must cancel without approximation");
    let denominator = Real::from(2) + &parameter;
    let mut expected = (Real::from(6) / (&denominator * &denominator)).unwrap();
    for (index, derivative) in derivatives.iter().enumerate() {
        if index > 0 {
            expected = (-expected * Real::from((index + 1) as u64) / &denominator).unwrap();
        }
        assert_eq!(derivative.dx(), &expected);
        assert_eq!(derivative.dy(), &(Real::from(2) * &expected));
    }
}

#[test]
fn high_derivative_requests_preserve_domain_and_size_guards() {
    let curve = RationalBezier2::try_new(
        vec![
            Point2::new(Real::zero(), Real::zero()),
            Point2::new(Real::one(), Real::one()),
        ],
        vec![Real::one(), Real::one()],
    )
    .unwrap();
    for parameter in [Real::from(-1), Real::from(2)] {
        assert!(matches!(
            crate::support::under_value(&CurveContext::STRICT, || curve.derivatives_at(&parameter, 80)),
            Err(ExactCurveError::Blocked(blocker)) if blocker.reason() == UncertaintyReason::Ordering
        ));
    }
    assert!(
        crate::support::under_value(&CurveContext::STRICT, || curve
            .derivatives_at(&Real::zero(), usize::MAX))
        .is_err()
    );
    assert!(
        crate::support::under_value(&CurveContext::STRICT, || curve
            .derivatives_at(&Real::zero(), 0))
        .unwrap()
        .is_empty()
    );
    let derivatives = curve.derivatives_at(&rational(1, 3), 128).unwrap();
    assert_eq!(derivatives[0].dx(), &Real::one());
    assert!(
        derivatives[1..]
            .iter()
            .all(|d| d.dx() == &Real::zero() && d.dy() == &Real::zero())
    );
    let pole = RationalBezier2::try_new(
        vec![
            Point2::new(Real::zero(), Real::zero()),
            Point2::new(Real::one(), Real::one()),
        ],
        vec![Real::one(), Real::from(-1)],
    )
    .unwrap();
    assert!(matches!(
        crate::support::under_value(&CurveContext::STRICT, || pole.derivatives_at(&rational(1, 2), 80)),
        Err(ExactCurveError::Blocked(blocker)) if blocker.reason() == UncertaintyReason::Boundary
    ));
}
