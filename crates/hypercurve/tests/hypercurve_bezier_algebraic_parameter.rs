use std::cmp::Ordering;

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Classification, CurveError, CurvePolicy, Real, UncertaintyReason,
};
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn polynomial(coefficients: Vec<Real>) -> BezierParameterPolynomial {
    match BezierParameterPolynomial::try_new_power_basis(coefficients, &policy()).unwrap() {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => {
            panic!("polynomial unexpectedly uncertain: {reason:?}")
        }
    }
}

fn interval(start: Real, end: Real) -> BezierParameterInterval {
    match BezierParameterInterval::try_new(start, end, &policy()).unwrap() {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("interval unexpectedly uncertain: {reason:?}"),
    }
}

fn isolate(
    polynomial: BezierParameterPolynomial,
    interval: BezierParameterInterval,
) -> BezierAlgebraicParameter2 {
    match BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy()).unwrap() {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("isolator unexpectedly uncertain: {reason:?}"),
    }
}

#[test]
fn linear_midpoint_root_is_a_valid_algebraic_parameter() {
    let polynomial = polynomial(vec![r(-1), r(2)]);
    let interval = interval(r(0), r(1));
    let parameter = isolate(polynomial, interval);

    assert_eq!(parameter.root_count(), 1);
    assert_eq!(parameter.polynomial().degree(), 1);
    assert_eq!(parameter.interval().start(), &r(0));
    assert_eq!(parameter.interval().end(), &r(1));
}

#[test]
fn linear_algebraic_parameter_recovers_represented_root() {
    let parameter = isolate(polynomial(vec![r(-1), r(2)]), interval(q(2, 5), q(3, 5)));

    assert_eq!(
        parameter.represented_linear_root(&policy()).unwrap(),
        Classification::Decided(Some(q(1, 2)))
    );
    assert_eq!(
        BezierParameter2::algebraic(parameter)
            .promote_represented_linear_root(&policy())
            .unwrap(),
        Classification::Decided(BezierParameter2::Exact(q(1, 2)))
    );
}

#[test]
fn nonlinear_algebraic_parameter_does_not_claim_represented_root() {
    let parameter = isolate(
        polynomial(vec![r(-1), r(0), r(2)]),
        interval(q(2, 3), q(3, 4)),
    );

    assert_eq!(
        parameter.represented_linear_root(&policy()).unwrap(),
        Classification::Decided(None)
    );
    assert!(matches!(
        BezierParameter2::algebraic(parameter)
            .promote_represented_linear_root(&policy())
            .unwrap(),
        Classification::Decided(BezierParameter2::Algebraic(_))
    ));
}

#[test]
fn quadratic_singleton_isolator_is_validated_by_sturm_count() {
    // p(t) = t^2 - 2t + 1/2 has one root in [0, 1/2] and the other outside
    // that interval. The exact value is intentionally not materialized.
    let polynomial = polynomial(vec![q(1, 2), r(-2), r(1)]);
    let interval = interval(r(0), q(1, 2));
    let parameter = isolate(polynomial, interval);

    assert_eq!(parameter.root_count(), 1);
}

#[test]
fn multi_root_bracket_is_rejected_as_not_an_isolator() {
    // p(t) = t^2 - t + 1/16 has two distinct roots inside [0, 1].
    let polynomial = polynomial(vec![q(1, 16), r(-1), r(1)]);
    let interval = interval(r(0), r(1));
    let error = BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy())
        .expect_err("two roots in one bracket must not certify a parameter");

    assert_eq!(error, CurveError::InvalidBezierAlgebraicParameter);
}

#[test]
fn endpoint_root_is_rejected_for_algebraic_isolators() {
    let polynomial = polynomial(vec![r(0), r(1)]);
    let interval = interval(r(0), r(1));
    let error = BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy())
        .expect_err("endpoint roots need exact endpoint representation or narrower brackets");

    assert_eq!(error, CurveError::InvalidBezierAlgebraicParameter);
}

#[test]
fn invalid_parameter_intervals_are_rejected() {
    let reversed = BezierParameterInterval::try_new(q(3, 4), q(1, 4), &policy())
        .expect_err("reversed intervals are invalid");
    assert_eq!(reversed, CurveError::InvalidBezierRange);

    let outside = BezierParameterInterval::try_new(r(-1), q(1, 4), &policy())
        .expect_err("out-of-domain intervals are invalid");
    assert_eq!(outside, CurveError::InvalidBezierParameter);
}

#[test]
fn exact_and_algebraic_parameters_compare_only_when_certified() {
    let left = BezierParameter2::exact(q(1, 4), &policy())
        .unwrap()
        .map(|value| value);
    let left = match left {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => {
            panic!("exact parameter unexpectedly uncertain: {reason:?}")
        }
    };

    let polynomial = polynomial(vec![r(-1), r(2)]);
    let algebraic = BezierParameter2::algebraic(isolate(polynomial, interval(q(2, 5), q(3, 5))));

    assert_eq!(
        left.cmp_by_interval(&algebraic, &policy()).unwrap(),
        Classification::Decided(Ordering::Less)
    );

    let overlapping = BezierParameter2::exact(q(1, 2), &policy())
        .unwrap()
        .map(|value| value);
    let overlapping = match overlapping {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => {
            panic!("exact parameter unexpectedly uncertain: {reason:?}")
        }
    };
    assert_eq!(
        overlapping.cmp_by_interval(&algebraic, &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Ordering)
    );
}

proptest! {
    #[test]
    fn linear_integer_polynomials_count_at_most_one_root(
        constant in -32_i32..=32,
        slope in -32_i32..=32,
        start_n in 0_i32..=15,
        width_n in 1_i32..=16,
    ) {
        prop_assume!(slope != 0);
        let end_n = (start_n + width_n).min(16);
        prop_assume!(start_n < end_n);

        let policy = policy();
        let polynomial = match BezierParameterPolynomial::try_new_power_basis(
            vec![r(constant), r(slope)],
            &policy,
        ).unwrap() {
            Classification::Decided(value) => value,
            Classification::Uncertain(_) => return Ok(()),
        };
        let interval = match BezierParameterInterval::try_new(q(start_n, 16), q(end_n, 16), &policy).unwrap() {
            Classification::Decided(value) => value,
            Classification::Uncertain(_) => return Ok(()),
        };

        match polynomial.root_count_in_interval(&interval, &policy) {
            Ok(Classification::Decided(count)) => prop_assert!(count <= 1),
            Ok(Classification::Uncertain(_)) => {}
            Err(CurveError::InvalidBezierAlgebraicParameter) => {}
            Err(error) => return Err(TestCaseError::fail(format!("unexpected error: {error:?}"))),
        }
    }
}
