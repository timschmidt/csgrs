#[cfg(test)]
mod eager_seed_tests {
    use super::*;
    use num::BigRational;

    fn power_two(p: i32) -> BigRational {
        if p >= 0 {
            BigRational::from_integer(BigInt::one() << p)
        } else {
            BigRational::new(BigInt::one(), BigInt::one() << -p)
        }
    }

    #[test]
    fn transplanted_unary_caches_enclose_independent_exact_endpoint_values() {
        for p in [-24, -1, 0, 1, 24] {
            for n in [-257, -16, -3, -1, 0, 1, 3, 16, 257] {
                for delta_n in -2..=2 {
                    let n = BigInt::from(n);
                    let q = (BigRational::from_integer(n.clone())
                        + BigRational::new(delta_n.into(), 2.into())) * power_two(p);
                    let child = Computable::rational(Rational::from_bigint_fraction(
                        q.numer().clone(), q.denom().magnitude().clone()).unwrap());
                    child.internal.store_cache_value(p, n);
                    for (approximation, exact) in [
                        (Approximation::Square(child.clone()), &q * &q),
                        (Approximation::Negate(child.clone()), -&q),
                        (Approximation::Offset(child.clone(), 17), &q * power_two(17)),
                        (Approximation::Offset(child.clone(), -17), &q * power_two(-17)),
                    ] {
                        let node = Node::new(approximation, BoundCache::Invalid, ExactSignCache::Invalid);
                        let (seed_p, seed) = node.cached_value().expect("available child cache should seed");
                        let scaled = exact / power_two(seed_p);
                        assert!((BigRational::from_integer(seed) - scaled).abs() <= BigRational::one());
                    }
                }
            }
        }
    }

    #[test]
    fn unrepresentable_seed_precision_is_skipped() {
        for p in [i32::MIN, i32::MAX] {
            let child = Computable::one();
            // A synthetic cache tests exponent arithmetic only; do not evaluate
            // this deliberately artificial node as the real number one.
            child.internal.store_cache_value(p, BigInt::one());
            let shift = if p < 0 { -1 } else { 1 };
            let node = Node::new(Approximation::Offset(child.clone(), shift),
                BoundCache::Invalid, ExactSignCache::Invalid);
            assert!(node.cached_value().is_none());
            let node = Node::new(Approximation::Square(child),
                BoundCache::Invalid, ExactSignCache::Invalid);
            assert!(node.cached_value().is_none());
        }
    }
}
