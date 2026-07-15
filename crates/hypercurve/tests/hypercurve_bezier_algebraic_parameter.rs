use std::cmp::Ordering;

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, BezierParameterRange2, Classification, CurveError, CurvePolicy,
    Real, UncertaintyReason,
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

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
    }
}

fn polynomial(coefficients: Vec<Real>) -> BezierParameterPolynomial {
    match BezierParameterPolynomial::try_new_power_basis(coefficients, &policy()).unwrap() {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => {
            panic!("polynomial unexpectedly uncertain: {reason:?}")
        }
    }
}

#[test]
fn bernstein_conversion_preserves_high_degree_constant_identity() {
    let cubic = decided(
        BezierParameterPolynomial::try_new_bernstein_basis(
            vec![r(1), r(2), r(4), r(8)],
            &policy(),
        )
        .unwrap(),
    );
    let polynomial = decided(
        BezierParameterPolynomial::try_new_bernstein_basis(vec![r(1); 65], &policy()).unwrap(),
    );

    assert_eq!(cubic.coefficients(), &[r(1), r(3), r(3), r(1)]);
    assert_eq!(polynomial.degree(), 0);
    assert_eq!(polynomial.coefficients(), &[r(1)]);
}

#[test]
fn unit_root_isolation_orders_represented_and_algebraic_roots() {
    let polynomial = polynomial(vec![q(1, 4), q(-1, 2), q(-1, 2), r(1)]);
    let roots = match polynomial.isolate_unit_interval_roots(&policy()).unwrap() {
        Classification::Decided(roots) => roots,
        Classification::Uncertain(reason) => panic!("root isolation was uncertain: {reason:?}"),
    };

    assert_eq!(roots.len(), 2);
    assert_eq!(roots[0], BezierParameter2::Exact(q(1, 2)));
    assert!(matches!(roots[1], BezierParameter2::Algebraic(_)));
    assert_eq!(
        roots[0].cmp_by_interval(&roots[1], &policy()).unwrap(),
        Classification::Decided(Ordering::Less)
    );
    let zero = BezierParameter2::Exact(r(0));
    let one = BezierParameter2::Exact(r(1));
    assert_eq!(
        zero.cmp_by_interval(&roots[1], &policy()).unwrap(),
        Classification::Decided(Ordering::Less)
    );
    assert_eq!(
        roots[1].cmp_by_interval(&one, &policy()).unwrap(),
        Classification::Decided(Ordering::Less)
    );
}

#[test]
fn quintic_root_isolation_trace_reuses_sturm_certificates() {
    // (7t-1)(7t-2)(7t-3)(7t-5)(7t-6)
    let polynomial = polynomial(vec![
        r(-180),
        r(2772),
        r(-15043),
        r(36701),
        r(-40817),
        r(16807),
    ]);
    let result = decided(
        polynomial
            .isolate_unit_interval_roots_with_trace(&policy())
            .unwrap(),
    );

    assert_eq!(
        result.roots(),
        &[
            BezierParameter2::Exact(q(1, 7)),
            BezierParameter2::Exact(q(2, 7)),
            BezierParameter2::Exact(q(3, 7)),
            BezierParameter2::Exact(q(5, 7)),
            BezierParameter2::Exact(q(6, 7)),
        ]
    );
    assert!(result.trace().interval_root_counts() > 5);
    assert!(
        result.trace().sturm_sequence_builds() < result.trace().interval_root_counts(),
        "one Sturm certificate should serve multiple interval counts: {:?}",
        result.trace()
    );
    assert!(result.trace().rational_reconstruction_refinements() > 0);
    assert!(result.trace().bisections() > 0);
    assert!(result.trace().maximum_depth() > 0);
}

#[test]
fn cubic_distance_stationary_quintic_isolates_all_five_candidates() {
    // (B(t)-P) dot B'(t) for controls (0,0), (6,10), (-8,-8),
    // (-4,10) and query point (-3,3). All five roots lie in (0,1).
    let polynomial = polynomial(vec![
        r(-36),
        r(1368),
        r(-11034),
        r(31728),
        r(-38280),
        r(16620),
    ]);
    let result = decided(
        polynomial
            .isolate_unit_interval_roots_with_trace(&policy())
            .unwrap(),
    );

    assert_eq!(result.roots().len(), 5);
    assert!(
        result
            .roots()
            .iter()
            .all(|root| matches!(root, BezierParameter2::Algebraic(_)))
    );
    assert_eq!(result.trace().sturm_sequence_builds(), 1);
    assert!(result.trace().interval_root_counts() > result.trace().sturm_sequence_builds());
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
fn algebraic_parameter_recovers_represented_linear_root() {
    let parameter = isolate(polynomial(vec![r(-1), r(2)]), interval(q(2, 5), q(3, 5)));

    assert_eq!(
        parameter.represented_rational_root(&policy()).unwrap(),
        Classification::Decided(Some(q(1, 2)))
    );
    assert_eq!(
        BezierParameter2::algebraic(parameter)
            .promote_represented_rational_root(&policy())
            .unwrap(),
        Classification::Decided(BezierParameter2::Exact(q(1, 2)))
    );
}

#[test]
fn irrational_nonlinear_parameter_remains_algebraic() {
    let parameter = isolate(
        polynomial(vec![r(-1), r(0), r(2)]),
        interval(q(2, 3), q(3, 4)),
    );

    assert_eq!(
        parameter.represented_rational_root(&policy()).unwrap(),
        Classification::Decided(None)
    );
    assert!(matches!(
        BezierParameter2::algebraic(parameter)
            .promote_represented_rational_root(&policy())
            .unwrap(),
        Classification::Decided(BezierParameter2::Algebraic(_))
    ));
}

#[test]
fn oriented_parameter_range_retains_irrational_boundary() {
    let start = BezierParameter2::algebraic(isolate(
        polynomial(vec![r(-1), r(0), r(2)]),
        interval(q(2, 3), q(3, 4)),
    ));
    let range = match BezierParameterRange2::try_new(
        start.clone(),
        BezierParameter2::Exact(Real::one()),
        &policy(),
    )
    .unwrap()
    {
        Classification::Decided(range) => range,
        Classification::Uncertain(reason) => panic!("range unexpectedly uncertain: {reason:?}"),
    };

    assert_eq!(range.start(), &start);
    assert_eq!(range.end(), &Real::one());
    assert_eq!(range.reversed().start(), &Real::one());
    assert!(range.exact_endpoints().is_none());
    let promoted = decided(
        range
            .promote_represented_rational_endpoints(&policy())
            .unwrap(),
    );
    assert!(promoted.exact_endpoints().is_none());
    assert_eq!(promoted.start(), &start);
}

#[test]
fn parameter_range_promotes_represented_rational_boundary() {
    let start = BezierParameter2::algebraic(isolate(
        polynomial(vec![r(-1), r(2)]),
        interval(q(2, 5), q(3, 5)),
    ));
    let range = decided(
        BezierParameterRange2::try_new(start, BezierParameter2::Exact(Real::one()), &policy())
            .unwrap(),
    );

    let promoted = decided(
        range
            .promote_represented_rational_endpoints(&policy())
            .unwrap(),
    );

    assert_eq!(promoted.exact_endpoints(), Some((&q(1, 2), &r(1))));
}

#[test]
fn parameter_range_rejects_equal_or_out_of_domain_boundaries() {
    let midpoint = BezierParameter2::Exact(q(1, 2));
    assert_eq!(
        BezierParameterRange2::try_new(midpoint.clone(), midpoint, &policy()).unwrap_err(),
        CurveError::InvalidBezierRange
    );
    assert_eq!(
        BezierParameterRange2::try_new(
            BezierParameter2::Exact(r(-1)),
            BezierParameter2::Exact(r(1)),
            &policy(),
        )
        .unwrap_err(),
        CurveError::InvalidBezierParameter
    );
}

#[test]
fn nonlinear_algebraic_parameter_reconstructs_exact_rational_root() {
    // (3t - 1)(t^2 + 1) has the sole real root t = 1/3. The cubic defining
    // polynomial ensures promotion does not rely on a linear representation.
    let parameter = isolate(
        polynomial(vec![r(-1), r(3), r(-1), r(3)]),
        interval(q(1, 4), q(1, 2)),
    );

    assert_eq!(
        parameter.represented_rational_root(&policy()).unwrap(),
        Classification::Decided(Some(q(1, 3)))
    );
    assert_eq!(
        BezierParameter2::algebraic(parameter)
            .promote_represented_rational_root(&policy())
            .unwrap(),
        Classification::Decided(BezierParameter2::Exact(q(1, 3)))
    );
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
        Classification::Decided(Ordering::Equal)
    );
}

#[test]
fn endpoint_touching_singleton_isolators_have_strict_order() {
    let defining = polynomial(vec![r(-112), r(576), r(-576)]);
    let left = BezierParameter2::algebraic(isolate(defining.clone(), interval(q(1, 4), q(1, 2))));
    let right = BezierParameter2::algebraic(isolate(defining, interval(q(1, 2), q(3, 4))));

    assert_eq!(
        left.cmp_by_interval(&right, &policy()).unwrap(),
        Classification::Decided(Ordering::Less)
    );
    assert_eq!(
        right.cmp_by_interval(&left, &policy()).unwrap(),
        Classification::Decided(Ordering::Greater)
    );
}

#[test]
fn equivalent_irrational_roots_compare_equal_across_polynomials_and_isolators() {
    let quadratic = BezierParameter2::algebraic(isolate(
        polynomial(vec![r(-1), r(0), r(2)]),
        interval(q(2, 3), q(3, 4)),
    ));
    let cubic = BezierParameter2::algebraic(isolate(
        polynomial(vec![r(-1), r(-1), r(2), r(2)]),
        interval(q(7, 10), q(4, 5)),
    ));

    assert_eq!(
        quadratic.cmp_by_interval(&cubic, &policy()).unwrap(),
        Classification::Decided(Ordering::Equal)
    );
    assert_eq!(
        cubic.cmp_by_interval(&quadratic, &policy()).unwrap(),
        Classification::Decided(Ordering::Equal)
    );
}

#[test]
fn overlapping_distinct_parameters_compare_by_certified_refinement() {
    let irrational = BezierParameter2::algebraic(isolate(
        polynomial(vec![r(-1), r(0), r(2)]),
        interval(q(2, 3), q(3, 4)),
    ));
    let close_rational = BezierParameter2::exact(q(353_553, 500_000), &policy())
        .unwrap()
        .map(|parameter| parameter);
    let close_rational = match close_rational {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => {
            panic!("exact parameter unexpectedly uncertain: {reason:?}")
        }
    };

    assert_eq!(
        close_rational
            .cmp_by_interval(&irrational, &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Ordering)
    );
    assert_eq!(
        close_rational
            .cmp_by_refinement(&irrational, &policy())
            .unwrap(),
        Classification::Decided(Ordering::Less)
    );
}

#[test]
fn algebraic_root_sign_change_tracks_multiplicity_parity() {
    let simple = polynomial(vec![r(-1), r(0), r(2)]);
    let simple_root =
        BezierParameter2::algebraic(isolate(simple.clone(), interval(q(2, 3), q(3, 4))));
    assert_eq!(
        simple
            .changes_sign_at_root(&simple_root, &policy())
            .unwrap(),
        Classification::Decided(true)
    );

    let double = polynomial(vec![r(1), r(0), r(-4), r(0), r(4)]);
    let double_root =
        BezierParameter2::algebraic(isolate(double.clone(), interval(q(2, 3), q(3, 4))));
    assert_eq!(
        double
            .changes_sign_at_root(&double_root, &policy())
            .unwrap(),
        Classification::Decided(false)
    );
}

#[test]
fn represented_root_sign_change_tracks_high_multiplicity_parity() {
    let double = polynomial(vec![q(1, 4), r(-1), r(1)]);
    let triple = polynomial(vec![q(-1, 8), q(3, 4), q(-3, 2), r(1)]);
    let root = BezierParameter2::Exact(q(1, 2));

    assert_eq!(
        double.changes_sign_at_root(&root, &policy()).unwrap(),
        Classification::Decided(false)
    );
    assert_eq!(
        triple.changes_sign_at_root(&root, &policy()).unwrap(),
        Classification::Decided(true)
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
