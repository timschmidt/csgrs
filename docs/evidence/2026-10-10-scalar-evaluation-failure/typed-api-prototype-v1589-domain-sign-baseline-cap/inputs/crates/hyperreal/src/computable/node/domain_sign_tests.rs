#[cfg(all(test, feature = "serde"))]
mod domain_sign_tests {
    use super::algebraic_leaf_tests::measure;
    use super::*;
    use crate::EvaluationBudget;
    use crate::computable::budget::BudgetScope;

    fn stored(kind: &str, child: Computable) -> Computable {
        serde_json::from_value(serde_json::json!({"internal": {kind: child}})).unwrap()
    }

    fn positive(bits: u32) -> Computable {
        let high = BigUint::one() << bits;
        Computable::rational(
            Rational::from_bigint_fraction(BigInt::from(&high + 1_u8), &high + 3_u8).unwrap(),
        )
    }

    #[test]
    fn domain_sign_canonical_leaves_need_no_coefficient_copy() {
        let mut failures = Vec::new();
        for bits in [1, 65, 516, 2050, 4096, 4097, 1_000_001] {
            for kind in ["Inverse", "Sqrt", "LnDeferred", "PrescaledLn"] {
                let value = stored(kind, positive(bits));
                let budget = EvaluationBudget::new(2);
                let scope = BudgetScope::enter(&budget);
                let (answer, allocations) = measure(|| value.has_finite_real_certificate());
                let exhausted = evaluation_exhausted();
                drop(scope);
                eprintln!(
                    "domain sign leaf bits={bits}, kind={kind}, answer={answer}, exhausted={exhausted}, consumed={}, allocations={allocations:?}",
                    budget.consumed()
                );
                if !answer || exhausted || allocations.bytes != 0 {
                    failures.push((bits, kind));
                }
                assert_eq!(budget.consumed() + budget.remaining(), 2);
                assert!(value.internal.cache_snapshot().is_none());
            }
        }
        assert!(failures.is_empty(), "{failures:?}");
    }

    #[test]
    fn domain_sign_retained_child_proof_precedes_recognition() {
        let mut failures = Vec::new();
        for kind in ["Inverse", "Sqrt", "LnDeferred", "PrescaledLn"] {
            // 3 < pi < 22/7 proves this child positive and finite. Public
            // evaluation obtains the reusable proof; the outer stored node
            // has never been queried. Serializing only the outer constructor
            // would discard the child's caches, so reconnect the warmed child
            // privately for this cross-root certificate diagnostic.
            let child = Computable::pi().add(Computable::integer((-3).into()));
            child.approx(-64).unwrap();
            assert_eq!(child.sign_until(-64), Some(RealSign::Positive));
            assert!(child.internal.facts.finite_real());
            let value = Computable::raw_periodic_operand(match kind {
                "Inverse" => Approximation::Inverse(child),
                "Sqrt" => Approximation::Sqrt(child),
                "LnDeferred" => Approximation::LnDeferred(child),
                "PrescaledLn" => Approximation::PrescaledLn(child),
                _ => unreachable!(),
            });
            let budget = EvaluationBudget::new(1);
            let scope = BudgetScope::enter(&budget);
            let (answer, allocations) = measure(|| value.has_finite_real_certificate());
            let exhausted = evaluation_exhausted();
            drop(scope);
            eprintln!(
                "domain sign retained kind={kind}, answer={answer}, exhausted={exhausted}, consumed={}, allocations={allocations:?}",
                budget.consumed()
            );
            if !answer || exhausted || allocations.bytes != 0 {
                failures.push(kind);
            }
        }
        assert!(failures.is_empty(), "{failures:?}");
    }

    #[test]
    fn domain_sign_invalid_descendants_remain_undefined() {
        for bad_kind in ["Inverse", "Sqrt", "LnDeferred"] {
            let child = if bad_kind == "Inverse" {
                Computable::zero()
            } else {
                Computable::integer((-1).into())
            };
            for outer in ["Inverse", "Sqrt", "LnDeferred", "PrescaledLn"] {
                let bad = stored(bad_kind, child.clone());
                let square = stored("Square", bad);
                let value = stored(outer, square);
                assert!(!value.has_finite_real_certificate());
                let answer = value.approx_with_budget(8, &EvaluationBudget::new(10_000));
                assert!(matches!(
                    answer,
                    Err(Problem::DivideByZero | Problem::SqrtNegative | Problem::NotANumber)
                ));
                assert!(value.internal.cache_snapshot().is_none());
                assert!(!value.internal.facts.finite_real());
                eprintln!("domain sign invalid child={bad_kind}, outer={outer}, answer={answer:?}");
            }
        }
    }

    #[test]
    fn domain_sign_zero_and_negative_boundaries() {
        for (numerator, denominator) in [(0, 1), (-1, 2), (-1, 1), (-3, 2), (1, 2)] {
            for kind in ["Inverse", "Sqrt", "LnDeferred", "PrescaledLn"] {
                let argument = Rational::fraction(numerator, denominator).unwrap();
                let value = stored(kind, Computable::rational(argument));
                let valid = match kind {
                    "Inverse" => numerator != 0,
                    "Sqrt" => numerator >= 0,
                    "LnDeferred" => numerator > 0,
                    "PrescaledLn" => numerator > -(denominator as i64),
                    _ => unreachable!(),
                };
                let answer = value.ensure_finite_real(&None);
                assert_eq!(answer.is_ok(), valid);
                assert_eq!(value.internal.facts.finite_real(), valid);
                let sample = value.approx_with_budget(0, &EvaluationBudget::new(100_000));
                assert_eq!(sample.is_ok(), valid);
                if !valid {
                    assert!(value.internal.cache_snapshot().is_none());
                }
                eprintln!(
                    "domain sign boundary n={numerator}, d={denominator}, kind={kind}, valid={valid}, domain={answer:?}, sample={sample:?}"
                );
            }
        }
    }
}
