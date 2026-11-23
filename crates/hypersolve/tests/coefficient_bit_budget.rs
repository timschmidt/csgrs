//! Public coefficient limits and nested failure ownership across the full u64 input range.

use hypersolve::work_budget::{
    BudgetExceeded, WorkBudget, WorkKind, consume, exhaustion, observe_coefficient_bits,
    with_budget, with_coefficient_bit_limit,
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
                            assert_eq!(denied.kind(), WorkKind::CoefficientGrowth);
                            let limit = if bits > outer { outer } else { inner };
                            assert_eq!(denied.limit(), limit);
                            assert_eq!(denied.requested(), bits);
                            assert_eq!(denied.used(), 0);
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
                    assert_eq!(denied.kind(), WorkKind::CoefficientGrowth);
                    assert_eq!(denied.limit(), limit);
                    assert_eq!(denied.requested(), request);
                    assert_eq!(denied.used(), 0);
                    assert_eq!(
                        denied.to_string(),
                        format!(
                            "coefficient growth limit exceeded: {request} bits against a limit of {limit}"
                        )
                    );
                    assert_eq!(observe_coefficient_bits(u64::MAX), Err(denied));
                });
                let denied = nested.result.unwrap_err();
                assert_eq!(denied.limit(), limit);
                assert_eq!(denied.requested(), request);
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
fn denial_above_the_word_ceiling_reports_exact_counts() {
    let ceiling = u64::from(u32::MAX);
    let fold = 1_u64 << 63;
    for (limit, request) in [
        (0, 1),
        (ceiling, ceiling + 1),
        (ceiling + 1, ceiling + 2),
        (ceiling, u64::MAX),
        (fold - 1, fold),
        (fold - 1, u64::MAX),
        (fold, fold + 1),
        (fold + ceiling, u64::MAX),
        (u64::MAX - 1, u64::MAX),
    ] {
        let denied = with_coefficient_bit_limit(limit, || observe_coefficient_bits(request))
            .result
            .unwrap_err();
        assert_eq!(denied.kind(), WorkKind::CoefficientGrowth);
        assert_eq!(
            (denied.limit(), denied.used(), denied.requested()),
            (limit, 0, request)
        );
        assert_eq!(
            denied.to_string(),
            format!("coefficient growth limit exceeded: {request} bits against a limit of {limit}")
        );
        assert_eq!(
            BudgetExceeded::new(WorkKind::CoefficientGrowth, limit, 0, request),
            Some(denied)
        );
        assert_eq!(
            format!("{denied:?}"),
            format!(
                "BudgetExceeded {{ kind: CoefficientGrowth, limit: {limit}, used: 0, requested: {request} }}"
            )
        );
    }
    assert_eq!(
        size_of::<BudgetExceeded>(),
        size_of::<u32>().max(align_of::<usize>()) + 3 * size_of::<usize>()
    );
    assert_eq!(
        size_of::<Option<BudgetExceeded>>(),
        size_of::<BudgetExceeded>()
    );
}

#[test]
fn receipts_are_built_only_for_reportable_denials() {
    let word = u64::try_from(usize::MAX).unwrap();
    for kind in [WorkKind::RootRefinement, WorkKind::AlgebraicPromotion] {
        for (limit, used, requested) in [(0, 0, 1), (5, 3, 3), (word, word, 1), (word, 1, word)] {
            let receipt = BudgetExceeded::new(kind, limit, used, requested).unwrap();
            assert_eq!(
                (
                    receipt.kind(),
                    receipt.limit(),
                    receipt.used(),
                    receipt.requested()
                ),
                (kind, limit, used, requested)
            );
        }
        for (limit, used, requested) in [(0, 0, 0), (5, 3, 2), (3, 5, 1)] {
            assert_eq!(BudgetExceeded::new(kind, limit, used, requested), None);
        }
        if usize::BITS < 64 {
            assert_eq!(BudgetExceeded::new(kind, word + 1, 0, word + 2), None);
        }
    }
    let cancelled = BudgetExceeded::new(WorkKind::Cancellation, 0, 0, 7).unwrap();
    assert_eq!(
        (cancelled.kind(), cancelled.requested()),
        (WorkKind::Cancellation, 7)
    );
    assert_eq!(BudgetExceeded::new(WorkKind::Cancellation, 1, 0, 7), None);
    assert_eq!(BudgetExceeded::new(WorkKind::Cancellation, 0, 1, 7), None);
    for (limit, used, requested) in [(3, 0, 3), (4, 0, 3), (0, 1, 2), (u64::MAX, 0, u64::MAX)] {
        assert_eq!(
            BudgetExceeded::new(WorkKind::CoefficientGrowth, limit, used, requested),
            None
        );
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
        assert_eq!(denied.kind(), WorkKind::CoefficientGrowth);
        assert_eq!(
            (denied.limit(), denied.used(), denied.requested()),
            (9, 0, 10)
        );
        assert_eq!(exhaustion(), None);
        let product = a.multiply(&b).unwrap();
        assert_eq!(product, expected);
        assert_eq!(product.multiply(&tensor(vec![Real::one()])), Some(expected));
    });
    assert_eq!(completed.result, Ok(()));
}

#[test]
fn fused_tensor_products_check_the_materialized_coefficients() {
    use hyperreal::Real;
    use hypersolve::DenseTensorPolynomial;

    // (16 + x)(16 - x) = 256 - x². The largest coefficient needs nine
    // numerator bits and one denominator bit.
    let tensor = |values: &[i64]| {
        DenseTensorPolynomial::try_new(
            vec![values.len()],
            values.iter().copied().map(Real::from).collect(),
        )
        .unwrap()
    };
    let left = tensor(&[16, 1]);
    let right = tensor(&[16, -1]);
    let expected = tensor(&[256, 0, -1]);
    let expected_negative = tensor(&[-256, 0, 1]);
    for (subtract, expected) in [(false, &expected), (true, &expected_negative)] {
        let terms = [(&left, &right, subtract)];
        let retry = with_coefficient_bit_limit(10, || {
            let denied = with_coefficient_bit_limit(9, || {
                let product = DenseTensorPolynomial::sum_products(&terms);
                assert!(product.is_none());
            });
            let receipt = denied.result.unwrap_err();
            assert_eq!(receipt.kind(), WorkKind::CoefficientGrowth);
            assert_eq!(
                (receipt.limit(), receipt.used(), receipt.requested()),
                (9, 0, 10)
            );
            assert_eq!(exhaustion(), None);
            let product = DenseTensorPolynomial::sum_products(&terms).unwrap();
            assert_eq!(&product, expected);
            assert_eq!(product.multiply(&tensor(&[1])).as_ref(), Some(expected));
        });
        assert_eq!(retry.result, Ok(()));
    }
    // Cancellation in the sum leaves three exact zero coefficients. Each
    // is represented by numerator zero and denominator one.
    let cancel = [(&left, &right, false), (&left, &right, true)];
    let zero = with_coefficient_bit_limit(1, || DenseTensorPolynomial::sum_products(&cancel));
    assert_eq!(zero.result.unwrap(), Some(tensor(&[0, 0, 0])));
}
