#[cfg(test)]
mod integer_scale_tests {
    use super::algebraic_leaf_tests::measure;
    use super::*;
    use crate::EvaluationBudget;
    use crate::computable::budget::BudgetScope;
    use num::Integer;

    fn oracle(n: &BigInt, p: Precision) -> BigInt {
        if p <= 0 {
            n * BigInt::from(2).pow(p.unsigned_abs())
        } else if u64::from(p as u32) > n.bits() {
            BigInt::zero()
        } else {
            let divisor = BigInt::from(2).pow(p as u32);
            let (q, r) = n.div_mod_floor(&divisor);
            if r * 2 >= divisor { q + 1 } else { q }
        }
    }

    fn integer(n: &BigInt) -> Computable {
        Computable::rational(Rational::from_bigint(n.clone()))
    }

    fn coarse_allocations(raw: bool) {
        let initialization = EvaluationBudget::new(1);
        drop(BudgetScope::enter(&initialization));
        assert_eq!(Computable::one().approx(1), Ok(BigInt::one()));
        let mut observations = Vec::new();
        for bits in [65_i32, 4097, 65_537, 1_000_001] {
            for negative in [false, true] {
                let n: BigInt = (BigInt::one() << (bits - 1)) + 1;
                let n = if negative { -n } else { n };
                let value = integer(&n);
                assert!(matches!(
                    value.internal.approximation,
                    Approximation::Int(_)
                ));
                for p in [bits - 1, bits, bits + 1, i32::MAX] {
                    let expected = oracle(&n, p);
                    let budget = EvaluationBudget::new(1);
                    let (answer, allocations) = measure(|| {
                        if raw {
                            value.internal.approximation.approximate(&None, p)
                        } else {
                            value.approx_with_budget(p, &budget)
                        }
                    });
                    assert_eq!(answer, Ok(expected));
                    assert_eq!(budget.consumed(), u64::from(!raw));
                    assert_eq!(budget.remaining(), u64::from(raw));
                    assert!(value.internal.cache_snapshot().is_none());
                    eprintln!(
                        "integer scale allocation raw={raw}, bits={bits}, negative={negative}, precision={p}, allocations={allocations:?}"
                    );
                    observations.push((bits, negative, p, allocations));
                }
                assert_eq!(value.approx(0), Ok(n));
            }
        }
        // These queries retain at most two output bits. This is an allocation
        // regression guard for the scaling path, not a byte-budget contract.
        // Fixture creation and the exact oracle are outside the measured region.
        assert!(
            observations.iter().all(|row| row.3.bytes <= 128),
            "coarse scaling copied discarded input limbs: {observations:?}"
        );
    }

    #[test]
    fn integer_scale_public_coarse_allocations() {
        coarse_allocations(false);
    }

    #[test]
    fn integer_scale_dispatch_coarse_allocations() {
        coarse_allocations(true);
    }

    #[test]
    fn integer_scale_signed_limb_boundaries_and_halfway_rounding() {
        for bits in [
            1_i32, 2, 31, 32, 33, 63, 64, 65, 127, 128, 129, 1024, 4096, 65_536,
        ] {
            let power = BigInt::one() << bits;
            for magnitude in [&power - 1, power.clone(), &power + 1, &power * 3] {
                for negative in [false, true] {
                    let n = if negative {
                        -&magnitude
                    } else {
                        magnitude.clone()
                    };
                    let value = integer(&n);
                    for p in [
                        -65,
                        -1,
                        0,
                        1,
                        2,
                        31,
                        32,
                        63,
                        64,
                        bits - 1,
                        bits,
                        bits + 1,
                        bits + 2,
                        i32::MAX,
                    ] {
                        let expected = oracle(&n, p);
                        assert_eq!(
                            value.approx(p),
                            Ok(expected.clone()),
                            "bits={bits}, negative={negative}, p={p}"
                        );
                        assert_eq!(
                            Approximation::Int(n.clone()).approximate(&None, p),
                            Ok(expected),
                            "raw bits={bits}, negative={negative}, p={p}"
                        );
                    }
                    assert!(value.internal.cache_snapshot().is_none());
                }
            }
        }
    }

    #[test]
    fn integer_scale_allowances_retries_and_retained_signs() {
        for bits in [8_i32, 65, 4097, 65_537, 1_000_001] {
            for negative in [false, true] {
                let n: BigInt = (BigInt::one() << (bits - 1)) + 1;
                let n = if negative { -n } else { n };
                for p in [-65, 0, bits - 1, bits, bits + 1, i32::MAX] {
                    let value = integer(&n);
                    let expected = oracle(&n, p);
                    for units in [0, 1, 3] {
                        let budget = EvaluationBudget::new(units);
                        let answer = value.approx_with_budget(p, &budget);
                        assert_eq!(
                            answer,
                            if units == 0 {
                                Err(Problem::Exhausted)
                            } else {
                                Ok(expected.clone())
                            }
                        );
                        assert_eq!(budget.consumed(), units.min(1));
                        assert_eq!(budget.consumed() + budget.remaining(), units);
                        assert_eq!(value.approx(p), Ok(expected.clone()));
                        assert!(value.internal.cache_snapshot().is_none());
                        let zero = EvaluationBudget::new(0);
                        let sign = if negative {
                            RealSign::Negative
                        } else {
                            RealSign::Positive
                        };
                        assert!(
                            value
                                .sign_until_with_budget(0, &zero)
                                .is_none_or(|s| s == sign)
                        );
                        assert_eq!(value.sign_until(0), Some(sign));
                        assert_eq!(value.sign_until_with_budget(0, &zero), Some(sign));
                        assert_eq!(zero.consumed(), 0);
                        assert_eq!(value.approx_with_budget(p, &zero), Err(Problem::Exhausted));
                    }
                }
            }
        }
    }

    #[test]
    fn integer_scale_precision_and_cancellation_precedence() {
        let stopped = Some(Arc::new(std::sync::atomic::AtomicBool::new(true)));
        for n in [
            BigInt::zero(),
            BigInt::one(),
            -BigInt::one(),
            (BigInt::one() << 1_000_000) + 1,
        ] {
            let value = integer(&n);
            for p in [i32::MIN, -Computable::MAX_APPROXIMATION_BITS - 1, i32::MAX] {
                for units in [0, 1] {
                    let budget = EvaluationBudget::new(units);
                    let expected = if n.is_zero() {
                        Ok(BigInt::zero())
                    } else if p == i32::MIN {
                        Err(Problem::OutOfRange)
                    } else if p < 0 || units == 0 {
                        Err(Problem::Exhausted)
                    } else {
                        Ok(BigInt::zero())
                    };
                    assert_eq!(value.approx_with_budget(p, &budget), expected);
                    assert_eq!(
                        budget.consumed(),
                        u64::from(!n.is_zero() && p > 0 && units > 0)
                    );
                }
                assert_eq!(value.approx_signal(&stopped, p), Err(Problem::Exhausted));
                assert_eq!(
                    Approximation::Int(n.clone()).approximate(&stopped, p),
                    Err(Problem::Exhausted)
                );
            }
        }
        // The largest supported fractional request remains exact for an integer.
        for n in [BigInt::one(), -BigInt::one()] {
            let p = -Computable::MAX_APPROXIMATION_BITS;
            assert_eq!(integer(&n).approx(p), Ok(oracle(&n, p)));
            assert_eq!(
                Approximation::Int(n.clone()).approximate(&None, p),
                Ok(oracle(&n, p))
            );
        }
    }
}
