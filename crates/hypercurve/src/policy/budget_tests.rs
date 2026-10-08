use std::cell::Cell;

use hypersolve::work_budget::{WorkBudget, WorkKind, consume, with_budget};

use super::*;

#[test]
fn budget_blockers_remain_transient_after_an_inner_scope_returns() {
    let classification = PolicyClassificationCache::<u8>::new();
    let evaluation = PolicyEvaluationCache::<u8>::new();
    let operation = BoundedPolicyResultCache::<u8, u8>::new();
    let classify = |policy: &CurveContext| -> Result<Classification<u8>, ()> {
        assert_eq!(policy, &CurveContext::STRICT);
        let inner = with_budget(WorkBudget::ZERO, || consume(WorkKind::RootRefinement, 1));
        assert!(inner.result.is_err());
        assert!(hypersolve::work_budget::exhaustion().is_none());
        Ok(Classification::Uncertain(
            UncertaintyReason::BudgetExhausted,
        ))
    };
    let expected = Classification::Uncertain(UncertaintyReason::BudgetExhausted);
    assert_eq!(
        resolve_cached_classification(&classification, &CurveContext::APPROXIMATE_512, classify)
            .unwrap(),
        expected
    );
    assert_eq!(
        resolve_cached_evaluation(&evaluation, &CurveContext::APPROXIMATE_512, classify).unwrap(),
        expected
    );
    assert!(classification.is_empty());
    assert!(evaluation.is_empty());
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let result = resolve_bounded_cached_result(&operation, 0, 1, &policy, |_, context| {
            ExactCurveError::decided(
                crate::CurveOperation2::Classification,
                classify(context).unwrap(),
            )
        });
        assert!(
            matches!(result, Err(ExactCurveError::Blocked(blocker)) if blocker.reason() == UncertaintyReason::BudgetExhausted)
        );
        assert!(operation.get().unwrap().lock().unwrap().is_empty());
    }
}

#[test]
fn exhausted_classification_cannot_seed_a_cache_or_start_an_approximate_retry() {
    let cache = PolicyClassificationCache::new();
    let calls = Cell::new(0);
    let exhausted = with_budget(WorkBudget::ZERO, || {
        let result = resolve_cached_classification(&cache, &CurveContext::APPROXIMATE_512, |_| {
            calls.set(calls.get() + 1);
            let _ = consume(WorkKind::RootRefinement, 1);
            Ok::<_, ()>(Classification::Decided(7))
        })
        .unwrap();
        assert_eq!(
            result,
            Classification::Uncertain(UncertaintyReason::BudgetExhausted)
        );
    });
    assert!(exhausted.result.is_err());
    assert_eq!(calls.get(), 1);
    assert!(cache.is_empty());
    let retry = with_budget(
        WorkBudget {
            refinements: 1,
            promotions: 0,
        },
        || {
            resolve_cached_classification(&cache, &CurveContext::STRICT, |_| {
                consume(WorkKind::RootRefinement, 1).unwrap();
                Ok::<_, ()>(Classification::Decided(8))
            })
            .unwrap()
        },
    );
    assert_eq!(retry.result.unwrap(), Classification::Decided(&8));
    let cached = with_budget(WorkBudget::ZERO, || {
        resolve_cached_classification(
            &cache,
            &CurveContext::STRICT,
            |_| -> Result<Classification<i32>, ()> {
                panic!("the certified value must be reusable without new work")
            },
        )
        .unwrap()
    });
    assert_eq!(cached.result.unwrap(), Classification::Decided(&8));
}

#[test]
fn exhausted_evaluation_cannot_seed_an_immutable_cache() {
    let cache = PolicyEvaluationCache::new();
    let calls = Cell::new(0);
    let exhausted = with_budget(WorkBudget::ZERO, || {
        let result = resolve_cached_evaluation(&cache, &CurveContext::APPROXIMATE_512, |_| {
            calls.set(calls.get() + 1);
            let _ = consume(WorkKind::AlgebraicPromotion, 1);
            Ok::<_, ()>(Classification::Decided(7))
        })
        .unwrap();
        assert_eq!(
            result,
            Classification::Uncertain(UncertaintyReason::BudgetExhausted)
        );
    });
    assert!(exhausted.result.is_err());
    assert_eq!(calls.get(), 1);
    assert!(cache.is_empty());
    assert_eq!(
        resolve_cached_evaluation(&cache, &CurveContext::STRICT, |_| {
            Ok::<_, ()>(Classification::Decided(8))
        })
        .unwrap(),
        Classification::Decided(&8)
    );
}

#[test]
fn work_limited_blockers_do_not_poison_the_operation_cache() {
    let cache = BoundedPolicyResultCache::new();
    let policy = CurveContext::STRICT;
    let exhausted = with_budget(WorkBudget::ZERO, || {
        resolve_bounded_cached_result(&cache, 0, 1, &policy, |_, _| {
            assert!(consume(WorkKind::RootRefinement, 1).is_err());
            Err(ExactCurveError::blocked_unattributed(
                crate::CurveOperation2::Classification,
                UncertaintyReason::BudgetExhausted,
            ))
        })
    });
    assert!(exhausted.result.is_err());
    assert!(cache.get().unwrap().lock().unwrap().is_empty());
    let bounded = policy.bounded_exact_predicate_pass(|| {
        resolve_bounded_cached_result(&cache, 0, 1, &policy, |_, _| {
            Err(ExactCurveError::blocked_unattributed(
                crate::CurveOperation2::Classification,
                UncertaintyReason::Unsupported,
            ))
        })
    });
    assert!(bounded.is_err());
    assert!(cache.get().unwrap().lock().unwrap().is_empty());
    assert_eq!(
        resolve_bounded_cached_result(&cache, 0, 1, &policy, |_, _| Ok(9)),
        Ok(9)
    );
    let cached = with_budget(WorkBudget::ZERO, || {
        resolve_bounded_cached_result(&cache, 0, 1, &policy, |_, _| {
            panic!("the completed operation must be reusable without new work")
        })
    });
    assert_eq!(cached.result.unwrap(), Ok(9));
}
