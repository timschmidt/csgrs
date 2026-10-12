#[cfg(test)]
mod acos_warm_probe_tests {
    use super::algebraic_leaf_tests::measure;
    use super::*;
    use crate::EvaluationBudget;
    use crate::computable::budget::BudgetScope;

    fn magnitude(bits: usize) -> Rational {
        Rational::from_bigint_fraction(
            (BigInt::one() << (bits - 1)) - 1,
            BigUint::one() << (bits - 1),
        )
        .unwrap()
    }

    fn root(r: &Rational, negative: bool) -> Computable {
        let value = Computable::rational(if negative { -r.clone() } else { r.clone() }).acos();
        let stored = match &value.internal.approximation {
            Approximation::AcosPositiveRational(stored) if !negative => stored,
            Approximation::AcosNegativeRational(stored) if negative => stored,
            _ => panic!("public rational acos did not retain its deferred rational node"),
        };
        assert_eq!(stored, r);
        assert_eq!(stored.stored_component_bits(), r.stored_component_bits());
        assert!(!value.internal.facts.finite_real());
        assert!(value.internal.cache_snapshot().is_none());
        value
    }

    fn survey(bits: usize, negative: bool) {
        // Fixture construction and constant initialization are outside the
        // query allocation window. The negative branch reuses a completed pi
        // approximation; it does not receive a fresh numerical-series budget.
        let initialization = EvaluationBudget::new(0);
        drop(BudgetScope::enter(&initialization));
        Computable::pi().approx(-16).unwrap();
        let r = magnitude(bits);
        assert_eq!(r.stored_component_bits(), ((bits - 1) as u64, bits as u64));
        let reference = root(&r, negative).approx(0).unwrap();
        // For these endpoints, acos(r) is between 0 and 2^-30, while
        // acos(-r) is between 3 and 4. Check the public one-unit error contract
        // independently of the reference used to compare repeated queries.
        assert!(if negative {
            reference == BigInt::from(3) || reference == BigInt::from(4)
        } else {
            reference == BigInt::zero() || reference == BigInt::one()
        });

        for state in ["cold", "domain", "coarse", "complete"] {
            for units in [0, 1, 2, 4, 8] {
                let value = root(&r, negative);
                match state {
                    "cold" => {}
                    // This is a private diagnostic state. Both coarse and
                    // complete warming below use only public numeric APIs.
                    "domain" => value.ensure_finite_real(&None).unwrap(),
                    "coarse" => assert_eq!(value.approx(1), Ok(BigInt::one())),
                    "complete" => assert_eq!(value.approx(0), Ok(reference.clone())),
                    _ => unreachable!(),
                }
                let before = value.internal.cache_snapshot();
                assert_eq!(value.internal.facts.finite_real(), state != "cold");
                assert_eq!(
                    before.as_ref().map(|entry| entry.0),
                    match state {
                        "cold" | "domain" => None,
                        "coarse" => Some(1),
                        "complete" => Some(0),
                        _ => unreachable!(),
                    }
                );
                let sign_before = value.sign_until_with_budget(0, &initialization);
                assert!(sign_before.is_none_or(|sign| sign == RealSign::Positive));
                let budget = EvaluationBudget::new(units);
                let (answer, allocations) = measure(|| value.approx_with_budget(0, &budget));
                let after = value.internal.cache_snapshot();
                eprintln!(
                    "acos warm survey bits={bits}, negative={negative}, state={state}, units={units}, answer={answer:?}, consumed={}, remaining={}, finite={}, before={before:?}, after={after:?}, allocations={allocations:?}",
                    budget.consumed(),
                    budget.remaining(),
                    value.internal.facts.finite_real()
                );
                assert_eq!(budget.consumed() + budget.remaining(), units);
                match &answer {
                    Ok(answer) => assert_eq!(answer, &reference),
                    Err(Problem::Exhausted) => assert_eq!(after, before),
                    Err(error) => panic!("valid acos endpoint returned {error:?}"),
                }
                if state == "complete" {
                    assert_eq!(answer, Ok(reference.clone()));
                    assert_eq!(budget.consumed(), 0);
                }
                if state == "coarse" {
                    let coarse = value.approx_with_budget(1, &initialization).unwrap();
                    if answer.is_err() {
                        assert_eq!(coarse, BigInt::one());
                    } else {
                        // A finer completed enclosure replaces the coarse one.
                        // Its rescaling may round differently while retaining
                        // the same value and one-unit error contract (P4).
                        assert!(if negative {
                            coarse == BigInt::one() || coarse == BigInt::from(2)
                        } else {
                            coarse == BigInt::zero() || coarse == BigInt::one()
                        });
                    }
                }
                assert_eq!(value.approx(0), Ok(reference.clone()));
                assert!(value.internal.facts.finite_real());
                assert_eq!(
                    value.approx_with_budget(0, &initialization),
                    Ok(reference.clone())
                );
                let sign_after = value.sign_until_with_budget(0, &initialization);
                assert!(sign_after.is_none_or(|sign| sign == RealSign::Positive));
                if sign_before.is_some() {
                    assert_eq!(sign_after, sign_before);
                }
                let stored = match &value.internal.approximation {
                    Approximation::AcosPositiveRational(stored)
                    | Approximation::AcosNegativeRational(stored) => stored,
                    _ => unreachable!(),
                };
                assert_eq!(stored, &r);
            }
        }
    }

    #[test]
    fn acos_warm_positive_65() {
        survey(65, false);
    }
    #[test]
    fn acos_warm_negative_65() {
        survey(65, true);
    }
    #[test]
    fn acos_warm_positive_4097() {
        survey(4097, false);
    }
    #[test]
    fn acos_warm_negative_4097() {
        survey(4097, true);
    }
    #[test]
    fn acos_warm_positive_65537() {
        survey(65_537, false);
    }
    #[test]
    fn acos_warm_negative_65537() {
        survey(65_537, true);
    }
    #[test]
    fn acos_warm_positive_1000001() {
        survey(1_000_001, false);
    }
    #[test]
    fn acos_warm_negative_1000001() {
        survey(1_000_001, true);
    }

    fn allocation_regression(negative: bool) {
        let initialization = EvaluationBudget::new(0);
        drop(BudgetScope::enter(&initialization));
        Computable::pi().approx(-16).unwrap();
        let mut observations = Vec::new();
        for bits in [65, 4097, 65_537, 1_000_001] {
            let r = magnitude(bits);
            let value = root(&r, negative);
            assert_eq!(value.approx(1), Ok(BigInt::one()));
            let before = value.internal.cache_snapshot();
            let budget = EvaluationBudget::new(1);
            let (answer, allocations) = measure(|| value.approx_with_budget(0, &budget));
            // Freeze the measured success/exhaustion and work receipts. This
            // optimization must preserve them as well as exact coefficients.
            assert_eq!(
                answer,
                if negative {
                    Err(Problem::Exhausted)
                } else {
                    Ok(BigInt::zero())
                }
            );
            assert_eq!(budget.consumed(), 1);
            assert_eq!(budget.remaining(), 0);
            if negative {
                assert_eq!(value.internal.cache_snapshot(), before);
                assert_eq!(
                    value.approx_with_budget(1, &initialization),
                    Ok(BigInt::one())
                );
            }
            assert!(value.internal.facts.finite_real());
            eprintln!(
                "acos warm allocation bits={bits}, negative={negative}, answer={answer:?}, allocations={allocations:?}"
            );
            observations.push((bits, allocations));
        }
        // At most eight coefficient-sized buffers plus 8 KiB of fixed work.
        // This is a regression ceiling for four measured endpoints, not a
        // promise that cooperative EvaluationBudget units bound bytes or time.
        assert!(
            observations
                .iter()
                .all(|(bits, a)| a.bytes <= 8 * bits.div_ceil(8) + 8192),
            "coarse-warmed acos copied avoidable rational intermediates: {observations:?}"
        );
    }

    #[test]
    fn acos_warm_positive_allocation_regression() {
        allocation_regression(false);
    }

    #[test]
    fn acos_warm_negative_allocation_regression() {
        allocation_regression(true);
    }
}
