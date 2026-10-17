use super::*;
use hyperlimit::PredicatePolicy;

fn circle(x: Real, radius: Real) -> Curve2 {
    let center = Point2::new(x.clone(), Real::zero());
    let start = Point2::new(x + radius, Real::zero());
    CircularArc2::try_from_center(start.clone(), start, center, false)
        .unwrap()
        .into()
}

fn query(cache: &mut CurveIntersectionBatchCache, a: &Curve2, b: &Curve2) -> CircleCircleRelation {
    cache
        .circular_support_relation(a, b, &CurveContext::STRICT)
        .unwrap()
        .unwrap()
}

fn same(actual: &Real, expected: &Real) {
    assert_eq!(
        hyperlimit::compare_reals(actual, expected, PredicatePolicy::STRICT).value(),
        Some(std::cmp::Ordering::Equal)
    );
}

fn secant(relation: CircleCircleRelation, x: Real, y: Real) {
    let CircleCircleRelation::Secant {
        first_point,
        second_point,
    } = relation
    else {
        panic!("expected two exact circle intersections: {relation:?}");
    };
    same(first_point.x(), &x);
    same(second_point.x(), &x);
    same(&(first_point.y() * first_point.y()), &(&y * &y));
    same(&(second_point.y() * second_point.y()), &(&y * &y));
    same(&(first_point.y() + second_point.y()), &Real::zero());
    assert_ne!(first_point.y(), second_point.y());
}

#[test]
fn distinct_supports_and_warm_hits_keep_independent_circle_oracles() {
    let mut cache = CurveIntersectionBatchCache::default();
    let pairs = (0..96)
        .map(|n| {
            let x = Real::from(n * 32);
            (
                circle(x.clone(), Real::from(5)),
                circle(&x + Real::from(6), Real::from(5)),
                x,
            )
        })
        .collect::<Vec<_>>();
    for _ in 0..2 {
        for (a, b, x) in &pairs {
            secant(query(&mut cache, a, b), x + Real::from(3), Real::from(4));
        }
    }
    assert_eq!(cache.circular_support_relations.len(), pairs.len());
}

#[test]
fn value_equal_rebuilt_supports_and_changed_geometry_do_not_alias() {
    let mut cache = CurveIntersectionBatchCache::default();
    {
        let x: Real = "0.1".parse().unwrap();
        secant(
            query(
                &mut cache,
                &circle(x.clone(), Real::from(5)),
                &circle(&x + Real::from(6), Real::from(5)),
            ),
            &x + Real::from(3),
            Real::from(4),
        );
    }
    let x = (Real::one() / Real::from(10)).unwrap();
    secant(
        query(
            &mut cache,
            &circle(x.clone(), Real::from(5)),
            &circle(&x + Real::from(6), Real::from(5)),
        ),
        &x + Real::from(3),
        Real::from(4),
    );
    assert_eq!(cache.circular_support_relations.len(), 1);
    assert!(matches!(
        query(
            &mut cache,
            &circle(x.clone(), Real::from(5)),
            &circle(&x + Real::from(11), Real::from(5))
        ),
        CircleCircleRelation::Disjoint
    ));
    assert_eq!(cache.circular_support_relations.len(), 2);
}

#[test]
fn ordered_pairs_tangency_coincidence_and_containment_keep_their_meaning() {
    let mut cache = CurveIntersectionBatchCache::default();
    let a = circle(Real::zero(), Real::from(5));
    let b = circle(Real::from(6), Real::from(5));
    secant(query(&mut cache, &a, &b), Real::from(3), Real::from(4));
    secant(query(&mut cache, &b, &a), Real::from(3), Real::from(4));
    assert_eq!(cache.circular_support_relations.len(), 2);
    let CircleCircleRelation::Tangent { point } =
        query(&mut cache, &a, &circle(Real::from(10), Real::from(5)))
    else {
        panic!("externally tangent circles must retain their contact");
    };
    same(point.x(), &Real::from(5));
    same(point.y(), &Real::zero());
    assert!(matches!(
        query(&mut cache, &a, &circle(Real::zero(), Real::from(5))),
        CircleCircleRelation::Coincident
    ));
    assert!(matches!(
        query(&mut cache, &a, &circle(Real::zero(), Real::one())),
        CircleCircleRelation::Disjoint
    ));
}

#[test]
fn symbolic_and_rational_supports_share_one_correct_batch() {
    let mut cache = CurveIntersectionBatchCache::default();
    let symbolic = Real::pi();
    assert!(symbolic.exact_rational_ref().is_none());
    for x in [symbolic.clone(), Real::zero(), symbolic, Real::from(-10)] {
        secant(
            query(
                &mut cache,
                &circle(x.clone(), Real::from(5)),
                &circle(&x + Real::from(6), Real::from(5)),
            ),
            &x + Real::from(3),
            Real::from(4),
        );
    }
    assert_eq!(cache.circular_support_relations.len(), 3);
}

#[test]
fn subrange_and_superrange_supports_preserve_exact_witnesses() {
    for (offset, scale) in [("1e400", "1"), ("0", "1e-400"), ("1e400", "1e-400")] {
        let x: Real = offset.parse().unwrap();
        let scale: Real = scale.parse().unwrap();
        let mut cache = CurveIntersectionBatchCache::default();
        let a = circle(x.clone(), Real::from(5) * &scale);
        let b = circle(&x + Real::from(6) * &scale, Real::from(5) * &scale);
        for _ in 0..2 {
            secant(
                query(&mut cache, &a, &b),
                &x + Real::from(3) * &scale,
                Real::from(4) * &scale,
            );
        }
    }
}

#[test]
fn exhausted_attempt_does_not_poison_later_support_proof() {
    use hypersolve::work_budget::{WorkBudget, WorkKind, consume, with_budget};
    let a = circle(Real::zero(), Real::from(5));
    let b = circle(Real::from(6), Real::from(5));
    let mut cache = CurveIntersectionBatchCache::default();
    let denied = with_budget(WorkBudget::ZERO, || {
        assert!(consume(WorkKind::RootRefinement, 1).is_err());
        query(&mut cache, &a, &b)
    });
    assert!(denied.result.is_err());
    secant(query(&mut cache, &a, &b), Real::from(3), Real::from(4));
}

#[test]
fn independent_batches_keep_policy_receipts_and_concurrent_results() {
    for (policy, context) in [
        (PredicatePolicy::STRICT, CurveContext::STRICT),
        (
            PredicatePolicy::APPROXIMATE_512,
            CurveContext::APPROXIMATE_512,
        ),
        (PredicatePolicy::STRICT, CurveContext::STRICT),
    ] {
        let result = crate::evaluate_under(policy, || {
            let a = circle(Real::zero(), Real::from(5));
            let b = circle(Real::from(6), Real::from(5));
            let mut cache = CurveIntersectionBatchCache::default();
            for _ in 0..2 {
                secant(
                    cache
                        .circular_support_relation(&a, &b, &context)
                        .unwrap()
                        .unwrap(),
                    Real::from(3),
                    Real::from(4),
                );
            }
        });
        assert!(result.certified().is_some());
    }
    std::thread::scope(|scope| {
        for _ in 0..4 {
            scope.spawn(|| {
                let mut cache = CurveIntersectionBatchCache::default();
                let a = circle(Real::zero(), Real::from(5));
                let b = circle(Real::from(6), Real::from(5));
                secant(query(&mut cache, &a, &b), Real::from(3), Real::from(4));
            });
        }
    });
}

#[test]
fn large_supports_keep_exact_witnesses_across_small_batch_transition() {
    use num::BigInt;

    let result = crate::evaluate_under(PredicatePolicy::STRICT, || {
        let large = Real::from(hyperreal::Rational::from_bigint(
            (BigInt::from(1_u8) << 1_048_576) + BigInt::from(3_u8),
        ));
        let pairs = (0..20)
            .map(|n| {
                let offset = Real::from(n * 32);
                let x = if n % 3 == 0 { &large + offset } else { offset };
                let a = circle(x.clone(), Real::from(5));
                let b = circle(&x + Real::from(6), Real::from(5));
                (a, b, x)
            })
            .collect::<Vec<_>>();
        let mut cache = CurveIntersectionBatchCache::default();
        for _ in 0..2 {
            for (a, b, x) in &pairs {
                secant(query(&mut cache, a, b), x + Real::from(3), Real::from(4));
            }
        }
        // Rebuilt Point2 owners still denote the original exact supports.
        for (_, _, x) in pairs.iter().rev() {
            let a = circle(x.clone(), Real::from(5));
            let b = circle(x + Real::from(6), Real::from(5));
            secant(query(&mut cache, &a, &b), x + Real::from(3), Real::from(4));
        }
        assert_eq!(cache.circular_support_relations.len(), pairs.len());
        let (a, _, x) = &pairs[0];
        let changed = circle(x + Real::from(11), Real::from(5));
        assert!(matches!(
            query(&mut cache, a, &changed),
            CircleCircleRelation::Disjoint
        ));
        secant(
            query(&mut cache, a, &pairs[0].1),
            x + Real::from(3),
            Real::from(4),
        );
        assert_eq!(cache.circular_support_relations.len(), pairs.len() + 1);
    });
    assert!(result.certified().is_some());
}
