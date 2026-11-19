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
