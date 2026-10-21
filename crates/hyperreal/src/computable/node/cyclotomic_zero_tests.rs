#[cfg(test)]
mod cyclotomic_zero_tests {
    use super::*;
    use std::sync::atomic::AtomicBool;

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

    fn ratio(numerator: i64, denominator: u64) -> Computable {
        raw(Approximation::Ratio(
            Rational::fraction(numerator, denominator).unwrap(),
        ))
    }

    fn cos_pi(numerator: i64, denominator: u64) -> Computable {
        raw(Approximation::PrescaledCos(raw(Approximation::Multiply(
            Computable::pi(),
            ratio(numerator, denominator),
        ))))
    }

    fn add(left: Computable, right: Computable) -> Computable {
        raw(Approximation::Add(left, right))
    }

    fn negate(value: Computable) -> Computable {
        raw(Approximation::Negate(value))
    }

    // cos(pi/7) - cos(2 pi/7) + cos(3 pi/7) - 1/2 = 0.
    fn heptagon() -> Computable {
        add(
            add(add(cos_pi(1, 7), negate(cos_pi(2, 7))), cos_pi(3, 7)),
            ratio(-1, 2),
        )
    }

    #[test]
    fn miller_rabin_is_exact_on_known_primes_and_strong_pseudoprimes() {
        for prime in [2, 3, 37, 41, (1 << 61) - 1, (1 << 62) - 57, u64::MAX - 58] {
            assert!(cyclotomic_is_prime(prime), "{prime} is prime");
        }
        // 3215031751 fools bases 2, 3, 5 and 7; 3825123056546413051 fools
        // every prime base through 23. 561 is a Carmichael number.
        for composite in [
            0,
            1,
            561,
            3_215_031_751,
            3_825_123_056_546_413_051,
            (1 << 62) - 1,
            ((1 << 31) - 1) * ((1 << 31) - 1),
        ] {
            assert!(!cyclotomic_is_prime(composite), "{composite} is composite");
        }
    }

    #[test]
    fn selected_moduli_contain_primitive_roots_of_the_field_order() {
        assert_eq!(cyclotomic_totient(1), 1);
        assert_eq!(cyclotomic_totient(32), 16);
        assert_eq!(cyclotomic_totient(288), 96);
        assert_eq!(cyclotomic_totient(612), 192);
        let signal = None;
        let mut proof = CyclotomicZeroProof {
            signal: &signal,
            work_left: Computable::CYCLOTOMIC_MAX_WORK,
        };
        let mut below = Computable::CYCLOTOMIC_PRIME_CEILING;
        for order in [1_u64, 32, 288, 612] {
            let factors = cyclotomic_prime_factors(order);
            let modulus = proof.modulus(order, &factors, below).unwrap();
            assert!(cyclotomic_is_prime(modulus.prime));
            assert!(modulus.prime >= Computable::CYCLOTOMIC_PRIME_FLOOR);
            assert!(modulus.prime < below);
            assert_eq!(modulus.prime % order, 1 % order);
            let root = modulus.powers.get(1).copied().unwrap_or(1);
            assert_eq!(cyclotomic_pow(root, order, modulus.prime), 1);
            for factor in factors {
                assert_ne!(cyclotomic_pow(root, order / factor, modulus.prime), 1);
            }
            below = modulus.prime;
        }
    }

    #[test]
    fn completed_zero_proof_publishes_only_an_exact_sign() {
        let value = heptagon();
        assert_eq!(value.bounded_cyclotomic_zero(), Some(Rational::zero()));
        assert!(value.cached().is_none(), "the proof does not approximate");
        assert!(matches!(
            value.internal.facts.exact_sign(),
            ExactSignCache::Valid(Sign::NoSign)
        ));
    }

    #[test]
    fn nonzero_and_unsupported_graphs_decline_without_publishing() {
        let nonzero = add(heptagon(), ratio(1, 1 << 40));
        assert_eq!(nonzero.bounded_cyclotomic_zero(), None);
        // sin(1/3) is not at a rational multiple of pi.
        let opaque = raw(Approximation::PrescaledSin(ratio(1, 3)));
        let unsupported = add(opaque.clone(), negate(opaque));
        assert_eq!(unsupported.bounded_cyclotomic_zero(), None);
        for value in [nonzero, unsupported] {
            assert!(matches!(
                value.internal.facts.exact_sign(),
                ExactSignCache::Invalid
            ));
        }
    }

    #[test]
    fn reciprocal_of_a_zero_is_never_admitted() {
        // 1/(heptagon) is undefined: no residue proves its operand nonzero.
        let value = add(
            raw(Approximation::Multiply(
                heptagon(),
                raw(Approximation::Inverse(heptagon())),
            )),
            ratio(-1, 1),
        );
        assert_eq!(value.bounded_cyclotomic_zero(), None);
    }

    #[test]
    fn aborted_cyclotomic_proof_does_not_publish_and_can_recover() {
        let mut value = heptagon();
        let clone = value.clone();
        let signal = Arc::new(AtomicBool::new(true));
        value.abort(signal.clone());
        assert_eq!(value.bounded_cyclotomic_zero(), None);
        assert!(matches!(
            clone.internal.facts.exact_sign(),
            ExactSignCache::Invalid
        ));
        signal.store(false, std::sync::atomic::Ordering::Relaxed);
        assert_eq!(value.bounded_cyclotomic_zero(), Some(Rational::zero()));
        assert!(matches!(
            clone.internal.facts.exact_sign(),
            ExactSignCache::Valid(Sign::NoSign)
        ));
    }

    #[test]
    fn shared_subgraphs_are_visited_once() {
        // A doubling chain has 2^depth paths but depth + 1 distinct nodes.
        let mut value = heptagon();
        for _ in 0..200 {
            value = add(value.clone(), value);
        }
        let graph = value.cyclotomic_graph().unwrap();
        assert!(graph.nodes.len() < 260, "{} nodes", graph.nodes.len());
        assert_eq!(value.bounded_cyclotomic_zero(), Some(Rational::zero()));
    }

    #[test]
    fn reciprocal_powers_prove_and_exponent_growth_declines() {
        let base = add(cos_pi(1, 5), ratio(1, 1));
        let reciprocal = raw(Approximation::Inverse(base.clone()));
        let (mut power, mut inverse_power) = (base, reciprocal);
        for _ in 0..3 {
            power = raw(Approximation::Square(power));
            inverse_power = raw(Approximation::Square(inverse_power));
        }
        // (c + 1)^8 / (c + 1)^8 - 1 = 0.
        let identity = add(
            raw(Approximation::Multiply(power, inverse_power.clone())),
            ratio(-1, 1),
        );
        assert_eq!(identity.bounded_cyclotomic_zero(), Some(Rational::zero()));
        // 64 more squarings would need exponent 2^67 and a bound of 2^67
        // bits; both limits decline before any arithmetic on them.
        let mut deep = inverse_power;
        for _ in 0..64 {
            deep = raw(Approximation::Square(deep));
        }
        assert!(deep.cyclotomic_graph().is_none());
        let mut grown = add(cos_pi(1, 5), ratio(1, 1));
        for _ in 0..64 {
            grown = raw(Approximation::Square(grown));
        }
        assert!(grown.cyclotomic_graph().is_none());
        assert_eq!(
            add(grown.clone(), negate(grown)).bounded_cyclotomic_zero(),
            None
        );
    }

    #[test]
    fn node_limit_declines() {
        let mut value = ratio(0, 1);
        for index in 0..Computable::CYCLOTOMIC_MAX_NODES as i64 {
            value = add(value, add(cos_pi(1, 5), ratio(index, 1)));
        }
        assert!(value.cyclotomic_graph().is_none());
        assert_eq!(value.bounded_cyclotomic_zero(), None);
    }
}
