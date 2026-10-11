#[cfg(test)]
mod cache_fact_invariance_tests {
    use super::*;

    fn fresh(kind: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(kind, BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        }
    }

    fn numeric_fixture(case: usize) -> Computable {
        match case {
            0 => fresh(Approximation::PrescaledSin(Computable::rational(Rational::fraction(1, 2).unwrap()))),
            1 => fresh(Approximation::LnDeferred(Computable::rational(Rational::new(2)))),
            2 => fresh(Approximation::Add(Computable::pi(), Computable::rational(Rational::fraction(-355, 113).unwrap()))),
            3 => Computable::erfc(Computable::pi()),
            4 => fresh(Approximation::PrescaledExp(fresh(Approximation::Add(Computable::pi(), Computable::rational(Rational::new(-3)))))),
            5 => fresh(Approximation::LnDeferred(fresh(Approximation::Add(Computable::pi(), Computable::rational(Rational::new(-3)))))),
            6 => Computable::rational(Rational::fraction(7, 8).unwrap()),
            7 => Computable::pi(),
            8 => Computable::sqrt_squarefree_rational(Rational::new(2)),
            _ => unreachable!(),
        }
    }

    fn snapshot(value: &Computable) -> (RealStructuralFacts, ZeroKnowledge, Option<Sign>, BoundInfo) {
        (value.structural_facts(), value.zero_status(), value.exact_sign(), value.cheap_bound())
    }

    #[test]
    fn numeric_refinement_does_not_change_structural_facts() {
        let mut failures = Vec::new();
        for case in 0..9 {
            let expected = snapshot(&numeric_fixture(case));
            for warm in 0..3 {
                for query_first in [false, true] {
                    let value = numeric_fixture(case);
                    if query_first {
                        assert_eq!(snapshot(&value), expected);
                    }
                    match warm {
                        0 => { value.approx(-128).unwrap(); }
                        1 => { assert!(value.sign_until(-128).is_some()); }
                        2 => { value.ensure_finite_real(&None).unwrap(); }
                        _ => unreachable!(),
                    }
                    let actual = snapshot(&value);
                    if actual != expected {
                        failures.push(format!("case={case}, warm={warm}, query_first={query_first}: {expected:?} -> {actual:?}"));
                    }
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[test]
    fn cached_signs_do_not_bypass_a_later_precision_floor() {
        let mut failures = Vec::new();
        for case in 0..3 {
            let expected = numeric_fixture(case).sign_until(0);
            assert_eq!(expected, None, "fixture={case}");
            for warm in 0..2 {
                let value = numeric_fixture(case);
                if warm == 0 {
                    value.approx(-128).unwrap();
                } else {
                    assert!(value.sign_until(-128).is_some());
                }
                let actual = value.sign_until(0);
                if actual != expected {
                    failures.push(format!("case={case}, warm={warm}: {expected:?} -> {actual:?}"));
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    fn deep_child() -> Computable {
        let mut value = Computable::one();
        for _ in 0..48 {
            value = fresh(Approximation::PrescaledExp(value));
        }
        value
    }

    fn outer(case: usize, child: Computable) -> Computable {
        match case {
            0 => child.exp(),
            1 => Computable::erfc(child),
            2 => Computable::zero().multiply(child),
            3 => child.clone().add(child.negate()),
            4 => Computable::normal_interval(child.clone(), child),
            5 => Computable::log_dnorm(child),
            _ => unreachable!(),
        }
    }

    #[test]
    fn domain_proof_reuse_does_not_change_structural_facts_or_construction() {
        let mut failures = Vec::new();
        for case in 0..6 {
            let expected = snapshot(&outer(case, deep_child()));
            for warm_before_construction in [false, true] {
                let child = deep_child();
                let value = if warm_before_construction {
                    child.ensure_finite_real(&None).unwrap();
                    outer(case, child)
                } else {
                    let value = outer(case, child.clone());
                    child.ensure_finite_real(&None).unwrap();
                    value
                };
                let actual = snapshot(&value);
                if actual != expected {
                    failures.push(format!("case={case}, warm_before_construction={warm_before_construction}: {expected:?} -> {actual:?}"));
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serialization_preserves_facts_after_numeric_refinement() {
        let mut failures = Vec::new();
        for case in 0..9 {
            let value = numeric_fixture(case);
            value.approx(-128).unwrap();
            let before = snapshot(&value);
            let decoded: Computable = serde_json::from_str(&serde_json::to_string(&value).unwrap()).unwrap();
            let after = snapshot(&decoded);
            if before != after {
                failures.push(format!("case={case}: {before:?} -> {after:?}"));
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }
}
