use super::{
    BudgetExceeded, CancellationToken, WorkBudget, WorkKind, WorkUsage, consume, exhaustion,
    is_active, observe_coefficient_bits, with_budget, with_cancellable_budget,
    with_coefficient_bit_limit,
};

#[test]
fn work_is_shared_across_kinds_and_nested_scopes() {
    let outcome = with_budget(
        WorkBudget {
            refinements: 3,
            promotions: 1,
        },
        || {
            consume(WorkKind::RootRefinement, 1).unwrap();
            let inner = with_budget(
                WorkBudget {
                    refinements: 9,
                    promotions: 9,
                },
                || {
                    consume(WorkKind::RootRefinement, 2).unwrap();
                    consume(WorkKind::AlgebraicPromotion, 1).unwrap();
                    42
                },
            );
            assert_eq!(inner.result, Ok(42));
            assert_eq!(
                inner.usage,
                WorkUsage {
                    refinements: 2,
                    promotions: 1
                }
            );
            7
        },
    );
    assert_eq!(outcome.result, Ok(7));
    assert_eq!(
        outcome.usage,
        WorkUsage {
            refinements: 3,
            promotions: 1
        }
    );
    assert!(!is_active());
}

#[test]
fn ignored_failure_cannot_publish_success_or_replenish_a_parent() {
    let expected = BudgetExceeded::new(WorkKind::RootRefinement, 1, 1, 1).unwrap();
    let outcome = with_budget(
        WorkBudget {
            refinements: 1,
            promotions: 0,
        },
        || {
            consume(WorkKind::RootRefinement, 1).unwrap();
            let inner = with_budget(
                WorkBudget {
                    refinements: 100,
                    promotions: 100,
                },
                || {
                    assert_eq!(consume(WorkKind::RootRefinement, 1), Err(expected));
                    assert_eq!(consume(WorkKind::RootRefinement, 0), Err(expected));
                    true
                },
            );
            assert_eq!(inner.result, Err(expected));
            assert_eq!(inner.usage, WorkUsage::default());
            assert_eq!(exhaustion(), Some(expected));
            true
        },
    );
    assert_eq!(outcome.result, Err(expected));
    assert_eq!(outcome.usage.refinements, 1);
    assert_eq!(exhaustion(), None);
}

#[test]
fn smaller_inner_budget_can_fail_without_poisoning_outer_work() {
    let outcome = with_budget(
        WorkBudget {
            refinements: 2,
            promotions: 0,
        },
        || {
            let inner = with_budget(WorkBudget::ZERO, || consume(WorkKind::RootRefinement, 1));
            assert!(inner.result.is_err());
            assert_eq!(exhaustion(), None);
            consume(WorkKind::RootRefinement, 2).unwrap();
        },
    );
    assert!(outcome.result.is_ok());
    assert_eq!(outcome.usage.refinements, 2);
}

#[test]
fn scopes_restore_after_unwinding_without_refunding_admitted_work() {
    let outer = with_budget(
        WorkBudget {
            refinements: 3,
            promotions: 0,
        },
        || {
            let result = std::panic::catch_unwind(|| {
                with_budget(
                    WorkBudget {
                        refinements: 2,
                        promotions: 0,
                    },
                    || {
                        consume(WorkKind::RootRefinement, 2).unwrap();
                        panic!("exercise budget restoration");
                    },
                )
            });
            assert!(result.is_err());
            assert_eq!(exhaustion(), None);
            consume(WorkKind::RootRefinement, 1).unwrap();
        },
    );
    assert!(outer.result.is_ok());
    assert_eq!(outer.usage.refinements, 3);
    assert!(!is_active());
}

#[test]
fn threads_start_without_a_budget() {
    let outcome = with_budget(WorkBudget::ZERO, || {
        std::thread::spawn(|| {
            assert!(!is_active());
            consume(WorkKind::RootRefinement, usize::MAX).unwrap();
        })
        .join()
        .unwrap();
    });
    assert!(outcome.result.is_ok());
}

#[test]
fn work_counter_overflow_is_a_typed_failure() {
    let maximal = with_budget(
        WorkBudget {
            refinements: usize::MAX,
            promotions: 0,
        },
        || {
            consume(WorkKind::RootRefinement, usize::MAX).unwrap();
            consume(WorkKind::RootRefinement, 1)
        },
    );
    assert!(maximal.result.is_err());
    assert_eq!(maximal.usage.refinements, usize::MAX);
}

#[test]
fn native_root_refinements_share_the_outer_operation_budget() {
    use crate::{
        IsolatedRootInterval, IsolatedRootRefinementStatus, PredicatePolicy, RootIsolationConfig,
        refine_isolated_univariate_polynomial_interval,
    };
    use hyperreal::Real;

    let polynomial = [Real::from(-2), Real::zero(), Real::one()];
    let interval = IsolatedRootInterval {
        lower: Real::one(),
        upper: Real::from(2),
        exact_root: None,
        distinct_root_count: 1,
    };
    let refine = || {
        refine_isolated_univariate_polynomial_interval(
            &polynomial,
            &interval,
            RootIsolationConfig {
                policy: PredicatePolicy::STRICT,
                max_interval_width: None,
                max_refinement_steps: 2,
            },
        )
    };
    let short = with_budget(
        WorkBudget {
            refinements: 3,
            promotions: 0,
        },
        || {
            let first = refine();
            assert_eq!(first.status, IsolatedRootRefinementStatus::Refined);
            assert_eq!(first.refinement_steps, 2);
            let second = with_budget(
                WorkBudget {
                    refinements: 100,
                    promotions: 0,
                },
                || {
                    let report = refine();
                    assert_eq!(report.status, IsolatedRootRefinementStatus::Undecided);
                    assert_eq!(report.refinement_steps, 1);
                    report
                },
            );
            assert_eq!(second.result.unwrap_err().used(), 3);
            assert_eq!(second.usage.refinements, 1);
        },
    );
    assert_eq!(short.result.unwrap_err().kind(), WorkKind::RootRefinement);
    assert_eq!(short.usage.refinements, 3);
    let retry = with_budget(
        WorkBudget {
            refinements: 4,
            promotions: 0,
        },
        || [refine(), refine()],
    );
    assert_eq!(retry.usage.refinements, 4);
    for report in retry.result.unwrap() {
        assert_eq!(report.status, IsolatedRootRefinementStatus::Refined);
        let refined = report.refined_interval.unwrap();
        assert_eq!(
            refined.upper - refined.lower,
            (Real::one() / Real::from(4)).unwrap()
        );
    }
}

#[test]
fn cancelled_token_denies_work_and_publishes_no_value() {
    let token = CancellationToken::new();
    token.cancel();
    let outcome = with_cancellable_budget(WorkBudget::UNLIMITED, &token, || {
        assert_eq!(
            consume(WorkKind::RootRefinement, 3),
            Err(BudgetExceeded::new(WorkKind::Cancellation, 0, 0, 3).unwrap())
        );
        // Every later request, of either kind, observes the same failure.
        assert_eq!(
            consume(WorkKind::AlgebraicPromotion, 1).unwrap_err().kind(),
            WorkKind::Cancellation
        );
        assert_eq!(exhaustion().unwrap().kind(), WorkKind::Cancellation);
        true
    });
    assert_eq!(outcome.result.unwrap_err().kind(), WorkKind::Cancellation);
    assert_eq!(outcome.usage, WorkUsage::default());
    assert!(!is_active());
    assert_eq!(
        outcome.result.unwrap_err().to_string(),
        "work cancelled; 3 more units requested"
    );
}

#[test]
fn cancellation_during_an_operation_stops_its_next_check() {
    let token = CancellationToken::new();
    let outcome = with_cancellable_budget(
        WorkBudget {
            refinements: 10,
            promotions: 10,
        },
        &token,
        || {
            consume(WorkKind::RootRefinement, 2).unwrap();
            assert_eq!(exhaustion(), None);
            token.cancel();
            // A caller that only polls exhaustion still stops.
            assert_eq!(exhaustion().unwrap().kind(), WorkKind::Cancellation);
            assert_eq!(
                consume(WorkKind::AlgebraicPromotion, 1).unwrap_err().kind(),
                WorkKind::Cancellation
            );
        },
    );
    assert_eq!(outcome.result.unwrap_err().requested(), 0);
    assert_eq!(
        outcome.usage,
        WorkUsage {
            refinements: 2,
            promotions: 0
        }
    );
}

#[test]
fn outer_cancellation_stops_nested_scopes_but_inner_cancellation_does_not_poison_the_parent() {
    let outer = CancellationToken::new();
    let outcome = with_cancellable_budget(WorkBudget::UNLIMITED, &outer, || {
        let inner_token = CancellationToken::new();
        let inner = with_cancellable_budget(WorkBudget::UNLIMITED, &inner_token, || {
            consume(WorkKind::RootRefinement, 1).unwrap();
            inner_token.cancel();
            consume(WorkKind::RootRefinement, 1)
        });
        assert_eq!(inner.result.unwrap_err().kind(), WorkKind::Cancellation);
        assert_eq!(inner.usage.refinements, 1);
        // The parent keeps working and its usage includes the nested unit.
        assert_eq!(exhaustion(), None);
        consume(WorkKind::RootRefinement, 1).unwrap();
        // An ordinary nested budget still sees the enclosing token.
        let nested = with_budget(WorkBudget::UNLIMITED, || {
            consume(WorkKind::RootRefinement, 1).unwrap();
            outer.cancel();
            consume(WorkKind::RootRefinement, 1)
        });
        assert_eq!(nested.result.unwrap_err().kind(), WorkKind::Cancellation);
        assert_eq!(exhaustion().unwrap().kind(), WorkKind::Cancellation);
    });
    assert_eq!(outcome.result.unwrap_err().kind(), WorkKind::Cancellation);
    assert_eq!(outcome.usage.refinements, 3);
    assert!(!is_active());
}

#[test]
fn cancellation_from_another_thread_reaches_a_running_operation() {
    use std::sync::{Arc, Barrier};

    let token = CancellationToken::new();
    let started = Arc::new(Barrier::new(2));
    let cancelled = Arc::new(Barrier::new(2));
    let worker = {
        let token = token.clone();
        let (started, cancelled) = (Arc::clone(&started), Arc::clone(&cancelled));
        std::thread::spawn(move || {
            with_cancellable_budget(WorkBudget::UNLIMITED, &token, || {
                consume(WorkKind::RootRefinement, 1).unwrap();
                started.wait();
                cancelled.wait();
                let mut admitted = 1;
                while consume(WorkKind::RootRefinement, 1).is_ok() {
                    admitted += 1;
                }
                admitted
            })
        })
    };
    started.wait();
    token.cancel();
    cancelled.wait();
    let outcome = worker.join().unwrap();
    assert_eq!(outcome.result.unwrap_err().kind(), WorkKind::Cancellation);
    assert_eq!(outcome.usage.refinements, 1);
    // The caller's thread never entered a scope.
    assert!(!is_active());
}

#[test]
fn cancelled_root_refinement_retries_with_a_new_token() {
    use crate::{
        IsolatedRootInterval, IsolatedRootRefinementStatus, PredicatePolicy, RootIsolationConfig,
        refine_isolated_univariate_polynomial_interval,
    };
    use hyperreal::Real;

    let polynomial = [Real::from(-2), Real::zero(), Real::one()];
    let interval = IsolatedRootInterval {
        lower: Real::one(),
        upper: Real::from(2),
        exact_root: None,
        distinct_root_count: 1,
    };
    let refine = || {
        refine_isolated_univariate_polynomial_interval(
            &polynomial,
            &interval,
            RootIsolationConfig {
                policy: PredicatePolicy::STRICT,
                max_interval_width: None,
                max_refinement_steps: 3,
            },
        )
    };
    let baseline = refine();
    assert_eq!(baseline.status, IsolatedRootRefinementStatus::Refined);

    let cancelled = CancellationToken::new();
    cancelled.cancel();
    let denied = with_cancellable_budget(WorkBudget::UNLIMITED, &cancelled, refine);
    assert_eq!(denied.result.unwrap_err().kind(), WorkKind::Cancellation);
    assert_eq!(denied.usage, WorkUsage::default());

    let fresh = CancellationToken::new();
    let retry = with_cancellable_budget(WorkBudget::UNLIMITED, &fresh, refine);
    assert_eq!(retry.usage.refinements, 3);
    let report = retry.result.unwrap();
    assert_eq!(report.status, IsolatedRootRefinementStatus::Refined);
    assert_eq!(report.refined_interval, baseline.refined_interval);
}

#[test]
fn coefficient_bit_limit_denies_larger_coefficients_only() {
    let outcome = with_coefficient_bit_limit(64, || {
        observe_coefficient_bits(64).unwrap();
        consume(WorkKind::RootRefinement, 5).unwrap();
        assert_eq!(
            observe_coefficient_bits(65),
            Err(BudgetExceeded::new(WorkKind::CoefficientGrowth, 64, 0, 65).unwrap())
        );
        // Later work in the exhausted scope also fails.
        assert_eq!(
            consume(WorkKind::RootRefinement, 1).unwrap_err().kind(),
            WorkKind::CoefficientGrowth
        );
    });
    assert_eq!(
        outcome.result.unwrap_err().kind(),
        WorkKind::CoefficientGrowth
    );
    assert_eq!(outcome.usage.refinements, 5);
    assert_eq!(
        outcome.result.unwrap_err().to_string(),
        "coefficient growth limit exceeded: 65 bits against a limit of 64"
    );
    // Without a limit, observations are free.
    assert!(observe_coefficient_bits(u64::MAX).is_ok());
    assert!(
        with_budget(WorkBudget::ZERO, || observe_coefficient_bits(u64::MAX))
            .result
            .unwrap()
            .is_ok()
    );
}

#[test]
fn coefficient_bit_limits_nest_like_budgets() {
    let outcome = with_coefficient_bit_limit(100, || {
        // A tighter nested limit fails alone.
        let inner = with_coefficient_bit_limit(10, || observe_coefficient_bits(50));
        assert_eq!(
            inner.result.unwrap_err().kind(),
            WorkKind::CoefficientGrowth
        );
        assert_eq!(exhaustion(), None);
        observe_coefficient_bits(50).unwrap();
        // The enclosing limit applies inside an ordinary nested budget.
        let nested = with_budget(WorkBudget::UNLIMITED, || observe_coefficient_bits(101));
        assert_eq!(nested.result.unwrap_err().limit(), 100);
        assert_eq!(exhaustion().unwrap().kind(), WorkKind::CoefficientGrowth);
    });
    assert_eq!(outcome.result.unwrap_err().limit(), 100);
    assert!(!is_active());
}

#[test]
fn cancellation_publication_drops_unpolled_result_and_allows_fresh_retry() {
    let token = CancellationToken::new();
    let retained = std::sync::Arc::new(17_u32);
    let outcome = with_cancellable_budget(WorkBudget::UNLIMITED, &token, || {
        token.cancel();
        std::sync::Arc::clone(&retained)
    });
    assert_eq!(
        outcome.result.unwrap_err(),
        BudgetExceeded::new(WorkKind::Cancellation, 0, 0, 0).unwrap()
    );
    assert_eq!(outcome.usage, WorkUsage::default());
    assert_eq!(std::sync::Arc::strong_count(&retained), 1);
    assert!(!is_active());
    assert_eq!(exhaustion(), None);

    let fresh = CancellationToken::new();
    let retry = with_cancellable_budget(WorkBudget::ZERO, &fresh, || {
        std::sync::Arc::clone(&retained)
    });
    assert_eq!(**retry.result.as_ref().unwrap(), 17);
    assert_eq!(retry.usage, WorkUsage::default());
    assert_eq!(std::sync::Arc::strong_count(&retained), 2);
    drop(retry);
    assert_eq!(std::sync::Arc::strong_count(&retained), 1);
    assert!(!is_active());
}

#[test]
fn cancellation_publication_observes_precancelled_token_after_callback() {
    let token = CancellationToken::new();
    token.cancel();
    let invoked = std::cell::Cell::new(false);
    let outcome = with_cancellable_budget(WorkBudget::UNLIMITED, &token, || {
        invoked.set(true);
        23
    });
    assert!(
        invoked.get(),
        "the boundary check does not preempt the callback"
    );
    assert_eq!(outcome.result.unwrap_err().kind(), WorkKind::Cancellation);
    assert_eq!(outcome.usage, WorkUsage::default());
    assert!(!is_active());
}

#[test]
fn cancellation_publication_observes_cross_thread_request() {
    use std::sync::{Arc, Barrier};
    let token = CancellationToken::new();
    let ready = Barrier::new(2);
    let finish = Barrier::new(2);
    let retained = Arc::new(31_u32);
    let outcome = std::thread::scope(|scope| {
        let worker = scope.spawn(|| {
            with_cancellable_budget(WorkBudget::UNLIMITED, &token, || {
                let value = Arc::clone(&retained);
                ready.wait();
                finish.wait();
                value
            })
        });
        ready.wait();
        token.cancel();
        finish.wait();
        worker.join().unwrap()
    });
    assert_eq!(outcome.result.unwrap_err().kind(), WorkKind::Cancellation);
    assert_eq!(outcome.usage, WorkUsage::default());
    assert_eq!(Arc::strong_count(&retained), 1);
    assert!(!is_active());
}

#[test]
fn cancellation_publication_propagates_unpolled_parent_request() {
    let parent = CancellationToken::new();
    let child = CancellationToken::new();
    let outcome = with_cancellable_budget(WorkBudget::UNLIMITED, &parent, || {
        let inner = with_cancellable_budget(WorkBudget::UNLIMITED, &child, || {
            parent.cancel();
            13
        });
        assert_eq!(inner.result.unwrap_err().kind(), WorkKind::Cancellation);
        assert_eq!(inner.usage, WorkUsage::default());
        37
    });
    assert_eq!(outcome.result.unwrap_err().kind(), WorkKind::Cancellation);
    assert_eq!(outcome.usage, WorkUsage::default());
    assert!(!is_active());
}

#[test]
fn cancellation_publication_keeps_inner_request_local() {
    let child = CancellationToken::new();
    let outcome = with_budget(
        WorkBudget {
            refinements: 2,
            promotions: 0,
        },
        || {
            consume(WorkKind::RootRefinement, 1).unwrap();
            let inner = with_cancellable_budget(WorkBudget::UNLIMITED, &child, || {
                child.cancel();
                13
            });
            assert_eq!(inner.result.unwrap_err().kind(), WorkKind::Cancellation);
            assert_eq!(exhaustion(), None);
            consume(WorkKind::RootRefinement, 1).unwrap();
            37
        },
    );
    assert_eq!(outcome.result, Ok(37));
    assert_eq!(outcome.usage.refinements, 2);
    assert_eq!(outcome.usage.promotions, 0);
    assert!(!is_active());
}

#[test]
fn cancellation_publication_preserves_first_failure() {
    let token = CancellationToken::new();
    let expected = BudgetExceeded::new(WorkKind::RootRefinement, 0, 0, 1).unwrap();
    let outcome = with_cancellable_budget(WorkBudget::ZERO, &token, || {
        assert_eq!(consume(WorkKind::RootRefinement, 1), Err(expected));
        token.cancel();
        41
    });
    assert_eq!(outcome.result, Err(expected));
    assert_eq!(outcome.usage, WorkUsage::default());
    assert!(!is_active());
}

#[test]
fn cancellation_publication_unwinding_restores_parent() {
    let child = CancellationToken::new();
    let outcome = with_budget(
        WorkBudget {
            refinements: 2,
            promotions: 0,
        },
        || {
            let unwound = std::panic::catch_unwind(|| {
                with_cancellable_budget(WorkBudget::UNLIMITED, &child, || {
                    consume(WorkKind::RootRefinement, 1).unwrap();
                    child.cancel();
                    panic!("exercise cancellation-boundary unwinding");
                })
            });
            assert!(unwound.is_err());
            assert_eq!(exhaustion(), None);
            consume(WorkKind::RootRefinement, 1).unwrap();
            43
        },
    );
    assert_eq!(outcome.result, Ok(43));
    assert_eq!(outcome.usage.refinements, 2);
    assert_eq!(outcome.usage.promotions, 0);
    assert!(!is_active());
}

#[test]
fn cancellation_publication_ordinary_budget_and_uncancelled_scope_keep_values() {
    let plain = with_budget(WorkBudget::ZERO, || 47);
    assert_eq!(plain.result, Ok(47));
    assert_eq!(plain.usage, WorkUsage::default());
    let token = CancellationToken::new();
    let cancellable = with_cancellable_budget(WorkBudget::ZERO, &token, || 53);
    assert_eq!(cancellable.result, Ok(53));
    assert_eq!(cancellable.usage, WorkUsage::default());
    assert!(!token.is_cancelled());
    assert!(!is_active());
}

#[test]
fn cancellation_publication_drops_result_after_restoring_parent() {
    struct Returned<'a>(&'a std::cell::Cell<bool>);
    impl Drop for Returned<'_> {
        fn drop(&mut self) {
            assert!(is_active(), "the enclosing budget must remain active");
            assert_eq!(exhaustion(), None, "the cancelled child must be removed");
            consume(WorkKind::RootRefinement, 1).unwrap();
            self.0.set(true);
        }
    }

    let dropped = std::cell::Cell::new(false);
    let outer = with_budget(
        WorkBudget {
            refinements: 1,
            promotions: 0,
        },
        || {
            let token = CancellationToken::new();
            let inner = with_cancellable_budget(WorkBudget::UNLIMITED, &token, || {
                token.cancel();
                Returned(&dropped)
            });
            assert!(
                dropped.get(),
                "the unpublished result must already be dropped"
            );
            assert_eq!(
                inner.result.err(),
                Some(BudgetExceeded::new(WorkKind::Cancellation, 0, 0, 0).unwrap())
            );
            assert_eq!(inner.usage, WorkUsage::default());
            let fresh = CancellationToken::new();
            let retry = with_cancellable_budget(WorkBudget::ZERO, &fresh, || 61);
            assert_eq!(retry.result, Ok(61));
            assert_eq!(retry.usage, WorkUsage::default());
            assert_eq!(exhaustion(), None);
            67
        },
    );
    assert_eq!(outer.result, Ok(67));
    assert_eq!(outer.usage.refinements, 1);
    assert_eq!(outer.usage.promotions, 0);
    assert!(!is_active());
    assert_eq!(exhaustion(), None);
}
