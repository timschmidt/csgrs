#[cfg(test)]
mod linear_demand_tests {
    use super::*;
    use num::Signed;
    use num::bigint::BigUint;

    #[test]
    fn linear_demand_hint_survives_independent_fact_updates() {
        assert_eq!(AtomicFacts::default().linear_demand(), 0);
        for demand in [i16::MIN, -512, -1, 0, 1, 512, i16::MAX] {
            let facts =
                AtomicFacts::new(BoundCache::Invalid, ExactSignCache::Invalid, false, demand);
            let bound = BoundCache::Valid(BoundInfo::NonZero {
                sign: Some(Sign::Minus),
                msd: Some(-1234567),
                exact_msd: true,
            });
            facts.set_bound_if_invalid(BoundCache::Valid(BoundInfo::Unknown));
            assert_eq!(facts.linear_demand(), demand);
            facts.set_bound_if_unresolved(bound);
            assert_eq!(facts.bound(), bound);
            facts.replace_exact_sign(ExactSignCache::Valid(Sign::Minus));
            facts.store_contains_inverse_trig_or_pi(true);
            assert_eq!(
                facts.snapshot(),
                (bound, ExactSignCache::Valid(Sign::Minus))
            );
            assert_eq!(facts.linear_demand(), demand);
            facts.set_bound(BoundCache::Valid(BoundInfo::Zero));
            assert_eq!(facts.linear_demand(), demand);
            assert_eq!(facts.contains_inverse_trig_or_pi(), Some(true));
        }
    }

    #[test]
    fn linear_demand_hint_saturates_and_accounts_for_binary_scaling() {
        fn with_hint(demand: i16) -> Computable {
            let value = Computable::one();
            value.internal.facts.0.fetch_or(
                (demand as u16 as u64) << AtomicFacts::LINEAR_DEMAND_SHIFT,
                std::sync::atomic::Ordering::Relaxed,
            );
            value
        }
        for demand in [i16::MIN, -512, -1, 0, 1, 512, i16::MAX] {
            let value = with_hint(demand);
            let negated = Node::new(
                Approximation::Negate(value.clone()),
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            );
            assert_eq!(negated.facts.linear_demand(), demand);
            for shift in [i32::MIN, -32768, -17, -1, 0, 1, 17, 32767, i32::MAX] {
                let shifted = Node::new(
                    Approximation::Offset(value.clone(), shift),
                    BoundCache::Invalid,
                    ExactSignCache::Invalid,
                );
                let expected = (i64::from(demand) + i64::from(shift))
                    .clamp(i64::from(i16::MIN), i64::from(i16::MAX))
                    as i16;
                assert_eq!(shifted.facts.linear_demand(), expected);
            }
            for other in [i16::MIN, -123, 0, 123, i16::MAX] {
                let sum = Node::new(
                    Approximation::Add(value.clone(), with_hint(other)),
                    BoundCache::Invalid,
                    ExactSignCache::Invalid,
                );
                let expected = (i32::from(demand.max(other)) + 2).min(i32::from(i16::MAX)) as i16;
                assert_eq!(sum.facts.linear_demand(), expected);
            }
        }
    }

    #[test]
    fn linear_demand_schedule_matches_independent_signed_root_enclosures() {
        use crate::Real;
        const N: usize = 257;
        const REFERENCE_BITS: usize = 512;
        fn expression() -> Real {
            let mut prefix = Real::zero();
            let values: Vec<_> = (1..=N)
                .map(|k| {
                    let root = Real::from((k + 1) as u64).sqrt().unwrap();
                    prefix += if k % 2 == 0 { -root } else { root };
                    if k % 3 == 0 {
                        &prefix * Real::from(2)
                    } else {
                        -&prefix
                    }
                })
                .collect();
            values.into_iter().sum()
        }
        // Sum_i c_i * prefix_i = Sum_k a_k * (Sum_{i>=k} c_i) * sqrt(k+1).
        // Enclose roots using integer square roots, independently of every
        // computable constructor, cache and scheduling decision.
        let mut lower = BigInt::zero();
        let mut upper = BigInt::zero();
        let mut suffix_weight = 0_i64;
        for k in (1..=N).rev() {
            suffix_weight += if k % 3 == 0 { 2 } else { -1 };
            let weight = if k % 2 == 0 {
                -suffix_weight
            } else {
                suffix_weight
            };
            let radicand = BigUint::from(k + 1) << (2 * REFERENCE_BITS);
            let floor = radicand.sqrt();
            let ceil = if &floor * &floor == radicand {
                floor.clone()
            } else {
                &floor + BigUint::one()
            };
            let (lo, hi) = if weight < 0 {
                (ceil, floor)
            } else {
                (floor, ceil)
            };
            lower += BigInt::from(lo) * weight;
            upper += BigInt::from(hi) * weight;
        }
        let denominator = BigUint::one() << REFERENCE_BITS;
        let lower = Rational::from_bigint_fraction(lower, denominator.clone()).unwrap();
        let upper = Rational::from_bigint_fraction(upper, denominator).unwrap();
        let check = |value: &Real, precision| {
            let [actual_lo, actual_hi] = value.certified_dyadic_interval(precision).unwrap();
            assert!(
                actual_lo <= lower && actual_hi >= upper,
                "precision {precision}"
            );
        };
        for precision in [-32, -128, -256] {
            check(&expression(), precision);
        }
        let ascending = expression();
        for precision in [-32, -128, -256] {
            check(&ascending, precision);
        }
        let descending = expression();
        for precision in [-256, -32, -128] {
            check(&descending, precision);
        }
        let concurrent = expression();
        std::thread::scope(|scope| {
            for precision in [-32, -128, -256] {
                let value = &concurrent;
                let check = &check;
                scope.spawn(move || check(value, precision));
            }
        });
    }

    #[test]
    fn linear_demand_schedule_does_not_publish_aborted_results() {
        use std::sync::atomic::{AtomicBool, Ordering};

        fn raw(approximation: Approximation) -> Computable {
            Computable {
                internal: Arc::new(Node::new(
                    approximation,
                    BoundCache::Invalid,
                    ExactSignCache::Invalid,
                )),
                signal: None,
            }
        }
        for shift in [-32, 0, 32] {
            for reverse in [false, true] {
                let leaf = Computable::prescaled_atan(BigInt::from(5));
                let mut nodes = vec![leaf.clone()];
                let mut prefix = leaf.clone();
                for _ in 0..8 {
                    prefix = raw(Approximation::Add(prefix, leaf.clone()));
                    nodes.push(prefix.clone());
                }
                let scaled = raw(Approximation::Offset(prefix, shift));
                nodes.push(scaled.clone());
                let value = if reverse {
                    raw(Approximation::Add(scaled, leaf))
                } else {
                    raw(Approximation::Add(leaf, scaled))
                };
                nodes.push(value.clone());
                let demands: Vec<_> = nodes
                    .iter()
                    .map(|node| node.internal.facts.linear_demand())
                    .collect();
                let stopped = Arc::new(AtomicBool::new(true));
                let _ = value.approx_signal(&Some(stopped.clone()), -128);
                for (node, demand) in nodes.iter().zip(demands) {
                    assert!(node.internal.cache_snapshot().is_none());
                    assert_eq!(node.internal.facts.linear_demand(), demand);
                }
                stopped.store(false, Ordering::Relaxed);
                let resumed = value.approx_signal(&Some(stopped), -128);
                let reference = Computable::prescaled_atan(BigInt::from(5))
                    .multiply(Computable::rational(Rational::from(9)))
                    .shift_left(shift)
                    .add(Computable::prescaled_atan(BigInt::from(5)))
                    .approx(-128);
                assert!((resumed - reference).abs() <= BigInt::from(2));
                assert!(value.internal.cache_snapshot().is_some());
            }
        }
    }

    #[cfg(feature = "serde")]
    #[test]
    fn linear_demand_hint_is_not_serialized_numerical_state() {
        let value = Computable::rational(Rational::from(2))
            .sqrt()
            .add(Computable::rational(Rational::from(3)).sqrt());
        assert!(value.internal.facts.linear_demand() > 0);
        let encoded = serde_json::to_string(&value).unwrap();
        assert!(!encoded.contains("demand") && !encoded.contains("facts"));
        let restored: Computable = serde_json::from_str(&encoded).unwrap();
        assert_eq!(restored.internal.facts.linear_demand(), 0);
        for precision in [-32, -128, -256] {
            let a = value.approx(precision);
            let b = restored.approx(precision);
            assert!((a - b).abs() <= BigInt::from(2));
        }
    }
}
