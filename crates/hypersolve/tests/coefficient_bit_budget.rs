//! Public coefficient limits and nested failure ownership across the full u64 input range.

use hypersolve::work_budget::{
    WorkBudget, WorkKind, consume, exhaustion, observe_coefficient_bits, with_budget,
    with_coefficient_bit_limit,
};

#[test]
fn outermost_exceeded_limit_owns_the_failure() {
    let word_boundary = 1_u64 << 32;
    let sizes = [
        0,
        1,
        word_boundary - 1,
        word_boundary,
        word_boundary + 1,
        1 << 63,
        u64::MAX - 1,
        u64::MAX,
    ];
    for outer in sizes {
        for inner in sizes {
            for bits in sizes {
                let outer_scope = with_coefficient_bit_limit(outer, || {
                    let inner_scope = with_coefficient_bit_limit(inner, || {
                        consume(WorkKind::RootRefinement, 1).unwrap();
                        let observation = observe_coefficient_bits(bits);
                        if bits <= outer && bits <= inner {
                            observation.unwrap();
                            assert_eq!(exhaustion(), None);
                        } else {
                            let denied = observation.unwrap_err();
                            assert_eq!(denied.kind, WorkKind::CoefficientGrowth);
                            assert_eq!(denied.used, 0);
                            assert_eq!(exhaustion(), Some(denied));
                            let child =
                                with_budget(WorkBudget::UNLIMITED, || observe_coefficient_bits(0));
                            assert_eq!(child.result, Err(denied));
                            assert_eq!(observe_coefficient_bits(0), Err(denied));
                        }
                    });
                    assert_eq!(inner_scope.usage.refinements, 1);
                    assert_eq!(inner_scope.result.is_err(), bits > inner || bits > outer);
                    if bits > outer {
                        assert_eq!(exhaustion(), inner_scope.result.err());
                    } else {
                        assert_eq!(exhaustion(), None);
                        observe_coefficient_bits(outer).unwrap();
                    }
                });
                assert_eq!(outer_scope.result.is_err(), bits > outer);
                assert_eq!(outer_scope.usage.refinements, 1);
                assert_eq!(exhaustion(), None);
                let retry = with_coefficient_bit_limit(bits, || observe_coefficient_bits(bits));
                assert_eq!(retry.result, Ok(Ok(())));
            }
        }
    }
}

#[test]
fn first_receipt_names_the_outermost_limit_that_denied_work() {
    for request in [23, usize::MAX as u64] {
        for (outer, inner, limit) in [
            (request - 6, request - 18, request - 6),
            (request - 18, request - 6, request - 18),
            (request, request - 18, request - 18),
        ] {
            let enclosing = with_coefficient_bit_limit(outer, || {
                let nested = with_coefficient_bit_limit(inner, || {
                    let denied = observe_coefficient_bits(request).unwrap_err();
                    assert_eq!(denied.kind, WorkKind::CoefficientGrowth);
                    assert_eq!(denied.limit as u64, limit);
                    assert_eq!(denied.requested as u64, request);
                    assert_eq!(denied.used, 0);
                    assert_eq!(
                        denied.to_string(),
                        format!(
                            "coefficient growth limit exceeded: {request} bits against a limit of {limit}"
                        )
                    );
                    assert_eq!(observe_coefficient_bits(u64::MAX), Err(denied));
                });
                let denied = nested.result.unwrap_err();
                assert_eq!(denied.limit as u64, limit);
                assert_eq!(denied.requested as u64, request);
                if outer < request {
                    assert_eq!(exhaustion(), Some(denied));
                } else {
                    assert_eq!(exhaustion(), None);
                }
            });
            assert_eq!(enclosing.result.is_err(), outer < request);
        }
    }
}

#[test]
fn materialized_product_counts_denominator_bits_and_recovers_after_denial() {
    use hyperreal::Real;
    use hypersolve::DenseTensorPolynomial;

    let tensor = |coefficients: Vec<Real>| {
        DenseTensorPolynomial::try_new(vec![coefficients.len()], coefficients).unwrap()
    };
    let ratio = |n, d| (Real::from(n) / Real::from(d)).unwrap();
    let a = tensor(vec![ratio(1_i64, 16_i64), Real::one()]);
    let b = tensor(vec![ratio(-1, 16), Real::one()]);
    let expected = tensor(vec![ratio(-1, 256), Real::zero(), Real::one()]);
    // (x + 1/16)(x - 1/16) has constant coefficient -1/256. Its
    // numerator needs one bit and its denominator needs nine.
    let completed = with_coefficient_bit_limit(10, || {
        let inner = with_coefficient_bit_limit(9, || {
            assert!(a.multiply(&b).is_none());
        });
        let denied = inner.result.unwrap_err();
        assert_eq!(denied.kind, WorkKind::CoefficientGrowth);
        assert_eq!((denied.limit, denied.used, denied.requested), (9, 0, 10));
        assert_eq!(exhaustion(), None);
        let product = a.multiply(&b).unwrap();
        assert_eq!(product, expected);
        assert_eq!(product.multiply(&tensor(vec![Real::one()])), Some(expected));
    });
    assert_eq!(completed.result, Ok(()));
}
