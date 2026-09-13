#[cfg(test)]
mod cache_rescale_tests {
    use super::*;
    use num::Integer;

    fn rounded(n: &BigInt, gap: u32) -> BigInt {
        if gap > 1_000_000 {
            assert!(n.bits() < u64::from(gap));
            return BigInt::zero();
        }
        let divisor = BigInt::one() << gap;
        let (q, r) = n.div_mod_floor(&divisor);
        if r * 2 >= divisor { q + 1 } else { q }
    }

    #[test]
    fn cache_rescale_rounding_and_misses() {
        for n in -4096..=4096 {
            let value = BigInt::from(n);
            let cached = CachedApproximation {
                precision: -16,
                value: value.clone(),
            };
            assert_eq!(ApproximationCache::value_at_precision(&cached, -17), None);
            for gap in 0..=16 {
                assert_eq!(
                    ApproximationCache::value_at_precision(&cached, -16 + gap as i32),
                    Some(rounded(&value, gap))
                );
            }
            assert_eq!(cached.value, value);
        }
    }

    #[test]
    fn cache_rescale_limb_and_halfway_boundaries() {
        for bits in [
            1_u32, 2, 31, 32, 33, 63, 64, 65, 127, 128, 129, 1024, 4096, 65536,
        ] {
            let unit = BigInt::one() << bits;
            for n in [&unit - 1, unit.clone(), &unit + 1, &unit >> 1] {
                for value in [n.clone(), -n] {
                    let cache = ApproximationCache::new();
                    cache.store(-100_000, value.clone());
                    for gap in [
                        0,
                        1,
                        2,
                        31,
                        32,
                        33,
                        63,
                        64,
                        65,
                        bits - 1,
                        bits,
                        bits + 1,
                        bits + 2,
                    ] {
                        assert_eq!(
                            cache.at_precision(-100_000 + gap as i32),
                            Some(rounded(&value, gap))
                        );
                    }
                    assert_eq!(cache.get(), Some((-100_000, value)));
                }
            }
        }
    }

    #[test]
    fn cache_rescale_extreme_gaps() {
        for n in [-65537, -3, -2, -1, 0, 1, 2, 3, 65537] {
            for (q, p) in [
                (i32::MIN, i32::MAX),
                (-128, i32::MAX),
                (-1, i32::MAX),
                (i32::MIN, 0),
                (i32::MIN, i32::MIN + 1),
            ] {
                let cache = ApproximationCache::new();
                let value = BigInt::from(n);
                cache.store(q, value.clone());
                assert_eq!(cache.at_precision(p), Some(rounded(&value, p.abs_diff(q))));
                assert_eq!(cache.get(), Some((q, value)));
            }
        }
    }

    #[test]
    fn cache_rescale_public_warmed_extreme() {
        for input in [
            Computable::rational(17.into()).sqrt(),
            Computable::rational(17.into()).sqrt().negate(),
            Computable::rational(Rational::fraction(1, 3).unwrap()),
            Computable::pi(),
            Computable::tau(),
        ] {
            assert!(!input.approx(-128).is_zero());
            assert!(input.approx(i32::MAX).is_zero());
            assert!(!input.approx(-256).is_zero());
        }
    }

    #[test]
    fn cache_rescale_concurrent_monotone_publication() {
        let cache = ApproximationCache::new();
        let barrier = std::sync::Barrier::new(8);
        std::thread::scope(|scope| {
            for worker in 0..8 {
                let cache = &cache;
                let barrier = &barrier;
                scope.spawn(move || {
                    barrier.wait();
                    for step in 0..32 {
                        let q = -((step * 8 + worker) + 1);
                        // Every writer publishes a different precision of the
                        // same exact dyadic 3/2; out-of-order writes must not
                        // replace a finer cache with a coarser one.
                        cache.store(q, BigInt::from(3) << (-q - 1));
                        for p in [-1, 0, 1, 64, i32::MAX] {
                            let result = cache.at_precision(p).unwrap();
                            assert_eq!(
                                result,
                                if p == -1 {
                                    3.into()
                                } else if p == 0 {
                                    2.into()
                                } else if p == 1 {
                                    1.into()
                                } else {
                                    BigInt::zero()
                                }
                            );
                        }
                    }
                });
            }
        });
        assert_eq!(cache.get(), Some((-256, BigInt::from(3) << 255)));
    }
}
