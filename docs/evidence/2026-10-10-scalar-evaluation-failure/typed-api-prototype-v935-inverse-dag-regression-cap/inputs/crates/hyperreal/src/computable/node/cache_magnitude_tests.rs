#[cfg(test)]
mod cache_magnitude_tests {
    use super::*;

    fn fresh(kind: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(
                kind,
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    fn exact_magnitude_fixture(case: usize) -> Computable {
        match case {
            0 => Computable::rational(Rational::fraction(7, 8).unwrap()),
            1 => Computable::rational(Rational::fraction(-7, 8).unwrap()),
            2 => Computable::sqrt_squarefree_rational(Rational::new(2)),
            3 => fresh(Approximation::Sqrt(Computable::rational(
                Rational::fraction(1, 2).unwrap(),
            ))),
            4 => fresh(Approximation::Sqrt(Computable::rational(Rational::new(5)))),
            5 => Computable::rational(Rational::new(2)).shift_left(-100),
            6 => Computable::rational(Rational::new(-2)).shift_left(-100),
            7 => Computable::sqrt_squarefree_rational(Rational::new(2)).shift_left(100),
            8 => Computable::pi(),
            _ => unreachable!(),
        }
    }

    #[test]
    fn approximation_first_preserves_exact_magnitude_certificates() {
        let mut failures = Vec::new();
        for case in 0..9 {
            let expected = exact_magnitude_fixture(case).structural_facts();
            assert!(
                expected
                    .magnitude
                    .is_some_and(|magnitude| magnitude.exact_msd)
            );
            for precision in [0, -16, -128] {
                for inspect_first in [false, true] {
                    let value = exact_magnitude_fixture(case);
                    if inspect_first {
                        assert_eq!(value.structural_facts(), expected);
                    }
                    value.approx(precision).unwrap();
                    let actual = value.structural_facts();
                    if actual != expected {
                        failures.push(format!("case={case}, p={precision}, inspect_first={inspect_first}: {expected:?} -> {actual:?}"));
                    }
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[test]
    fn unknown_magnitude_exp_construction_does_not_evaluate_descendants() {
        // Preserve the original 48-level constructor workload that exhausted
        // the capped v166 process before reaching its fact-query assertions.
        let mut retained = vec![Computable::one()];
        for _ in 0..48 {
            retained.push(fresh(Approximation::PrescaledExp(
                retained.last().unwrap().clone(),
            )));
        }
        let value = retained.last().unwrap().clone().exp();
        assert!(matches!(
            value.internal.approximation,
            Approximation::PrescaledExp(_)
        ));
        assert!(value.internal.cache_snapshot().is_none());
        assert!(
            retained
                .iter()
                .all(|child| child.internal.cache_snapshot().is_none())
        );
        drop(value);
        while retained.pop().is_some() {}

        // A deferred unknown-magnitude input must retain ordinary evaluation.
        // exp(ln(2)) = 2 gives an exact, independent enclosure oracle.
        let logarithm = fresh(Approximation::LnDeferred(Computable::rational(
            Rational::new(2),
        )));
        let value = logarithm.clone().exp();
        assert!(logarithm.internal.cache_snapshot().is_none());
        for precision in [0_i32, -64] {
            let expected = BigInt::from(2) << precision.unsigned_abs();
            assert!((value.approx(precision).unwrap() - expected).abs() <= BigInt::one());
        }
    }
}
