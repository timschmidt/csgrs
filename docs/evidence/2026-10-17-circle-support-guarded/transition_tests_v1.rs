
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
        assert!(matches!(query(&mut cache, a, &changed), CircleCircleRelation::Disjoint));
        secant(query(&mut cache, a, &pairs[0].1), x + Real::from(3), Real::from(4));
        assert_eq!(cache.circular_support_relations.len(), pairs.len() + 1);
    });
    assert!(result.certified().is_some());
}
