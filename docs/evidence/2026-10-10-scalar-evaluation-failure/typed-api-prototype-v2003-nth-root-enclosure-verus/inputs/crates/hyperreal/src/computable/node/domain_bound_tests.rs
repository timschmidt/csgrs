#[cfg(all(test, feature = "serde"))]
mod domain_bound_tests {
    use super::algebraic_leaf_tests::measure;
    use super::*;
    use crate::EvaluationBudget;
    use crate::computable::budget::BudgetScope;
    use rug::{Float, Integer, Rational as Q, float::Constant, float::Round};

    fn scaled_pi(shift: u32, squared: bool, negative: bool) -> Computable {
        let value = Computable::pi().multiply(Computable::rational(
            Rational::fraction(1, 1_u64 << shift).unwrap(),
        ));
        let value = if squared { value.square() } else { value };
        if negative { value.negate() } else { value }
    }

    fn public_outer(kind: &str, child: Computable) -> Computable {
        match kind {
            "asin" => child.asin(),
            "acos" => child.acos(),
            "atanh" => child.atanh(),
            _ => unreachable!(),
        }
    }

    fn diagnostic_outer(kind: &str, child: Computable) -> Computable {
        // The diagnostic reconnects the same publicly warmed child Arc. A
        // serialization round-trip would intentionally discard those caches.
        Computable::raw_periodic_operand(match kind {
            "asin" => Approximation::AsinDeferred(child),
            "asin-series" => Approximation::PrescaledAsin(child),
            "acos" => Approximation::AcosPositive(child),
            "atanh" => Approximation::AtanhDirect(child),
            "atanh-series" => Approximation::PrescaledAtanh(child),
            "ln1p" => Approximation::PrescaledLn(child),
            "tan" => Approximation::TanDeferred(child),
            "tan-series" => Approximation::PrescaledTan(child),
            "cot-series" => Approximation::PrescaledCot(child),
            "acosh" => Approximation::AcoshDirect(child),
            "acosh-near" => Approximation::AcoshNearOne(child),
            _ => unreachable!(),
        })
    }

    fn prepare_coarse(child: &Computable) -> RealStructuralFacts {
        child.structural_facts();
        child.approx(8).unwrap();
        assert!(child.internal.facts.finite_real());
        child.structural_facts()
    }

    #[test]
    fn domain_bound_retained_unit_proofs_precede_recognition() {
        let mut failures = Vec::new();
        for squared in [false, true] {
            for negative in [false, true] {
                for kind in [
                    "asin",
                    "asin-series",
                    "acos",
                    "atanh",
                    "atanh-series",
                    "ln1p",
                    "tan",
                    "tan-series",
                    "cot-series",
                ] {
                    let child = scaled_pi(4, squared, negative);
                    let before = prepare_coarse(&child);
                    assert!(before.magnitude.is_some_and(|bound| bound.msd < 0));
                    let value = diagnostic_outer(kind, child.clone());
                    let budget = EvaluationBudget::new(1);
                    let scope = BudgetScope::enter(&budget);
                    let (accepted, allocations) = measure(|| value.has_finite_real_certificate());
                    let exhausted = evaluation_exhausted();
                    drop(scope);
                    let after = child.structural_facts();
                    assert_eq!(before.sign, after.sign);
                    assert_eq!(before.magnitude, after.magnitude);
                    eprintln!(
                        "domain bound unit squared={squared}, negative={negative}, kind={kind}, before={before:?}, accepted={accepted}, exhausted={exhausted}, consumed={}, allocations={allocations:?}",
                        budget.consumed()
                    );
                    if !accepted || exhausted || allocations.bytes != 0 {
                        failures.push((squared, negative, kind));
                    }
                    assert_eq!(budget.consumed() + budget.remaining(), 1);
                }
            }
        }
        assert!(failures.is_empty(), "{failures:?}");
    }

    #[test]
    fn domain_bound_retained_lower_proofs_precede_recognition() {
        let mut failures = Vec::new();
        for input in ["one", "sqrt2", "pi"] {
            for kind in ["acosh", "acosh-near"] {
                let child = match input {
                    "one" => Computable::rational(Rational::one()),
                    "sqrt2" => Computable::rational(Rational::new(2)).sqrt(),
                    "pi" => Computable::pi(),
                    _ => unreachable!(),
                };
                // Integer/constant public approximation bypasses the node's
                // finite flag. Independently establish the diagnostic's domain
                // prerequisite without manufacturing a sign or magnitude fact.
                child.structural_facts();
                child.approx(8).unwrap();
                child.ensure_finite_real(&None).unwrap();
                let before = child.structural_facts();
                assert!(
                    before
                        .magnitude
                        .is_some_and(|bound| bound.exact_msd && bound.msd >= 0)
                );
                let value = diagnostic_outer(kind, child.clone());
                let budget = EvaluationBudget::new(1);
                let scope = BudgetScope::enter(&budget);
                let (accepted, allocations) = measure(|| value.has_finite_real_certificate());
                let exhausted = evaluation_exhausted();
                drop(scope);
                assert_eq!(child.structural_facts(), before);
                eprintln!(
                    "domain bound lower input={input}, kind={kind}, before={before:?}, accepted={accepted}, exhausted={exhausted}, consumed={}, allocations={allocations:?}",
                    budget.consumed()
                );
                if !accepted || exhausted || allocations.bytes != 0 {
                    failures.push((input, kind));
                }
            }
        }
        assert!(failures.is_empty(), "{failures:?}");
    }

    #[test]
    fn domain_bound_required_domain_reuses_available_proofs() {
        let mut failures = Vec::new();
        for squared in [false, true] {
            for negative in [false, true] {
                for kind in ["closed", "open", "above-minus-one", "probability"] {
                    if negative && kind == "probability" {
                        continue;
                    }
                    let child = scaled_pi(4, squared, negative);
                    let before = prepare_coarse(&child);
                    let domain = match kind {
                        "closed" => RequiredRealDomain::ClosedUnit,
                        "open" => RequiredRealDomain::OpenUnit,
                        "above-minus-one" => RequiredRealDomain::AboveMinusOne,
                        "probability" => RequiredRealDomain::Probability,
                        _ => unreachable!(),
                    };
                    let budget = EvaluationBudget::new(1);
                    let scope = BudgetScope::enter(&budget);
                    let (answer, allocations) = measure(|| {
                        child.certify_required_domain(&None, domain, Problem::NotANumber)
                    });
                    let exhausted = evaluation_exhausted();
                    drop(scope);
                    assert_eq!(child.structural_facts(), before);
                    eprintln!(
                        "domain bound required squared={squared}, negative={negative}, kind={kind}, before={before:?}, answer={answer:?}, exhausted={exhausted}, consumed={}, allocations={allocations:?}",
                        budget.consumed()
                    );
                    if answer != Ok(()) || exhausted || allocations.bytes != 0 {
                        failures.push((squared, negative, kind));
                    }
                }
            }
        }
        assert!(failures.is_empty(), "{failures:?}");
    }

    #[test]
    fn domain_bound_upper_estimates_do_not_prove_lower_bounds() {
        // 3 < pi < 22/7 proves 27/32 < x=9*pi/32 < 99/112 < 1.
        // Multiplication's cached upper exponent is zero, which does not
        // provide the lower bound needed by acosh.
        for kind in ["acosh", "acosh-near"] {
            let child =
                Computable::pi().multiply(Computable::rational(Rational::fraction(9, 32).unwrap()));
            let facts = prepare_coarse(&child);
            assert!(
                facts
                    .magnitude
                    .is_some_and(|bound| !bound.exact_msd && bound.msd == 0)
            );
            let value = diagnostic_outer(kind, child);
            assert!(!value.has_finite_real_certificate());
            let answer = value.approx_with_budget(8, &EvaluationBudget::new(100_000));
            eprintln!("domain bound upper-only kind={kind}, facts={facts:?}, answer={answer:?}");
            assert_eq!(answer, Err(Problem::NotANumber));
            assert!(!value.internal.facts.finite_real());
            assert!(value.internal.cache_snapshot().is_none());
        }
    }

    #[test]
    fn domain_bound_endpoints_and_invalid_descendants_remain_typed() {
        for numerator in [-3, -2, -1, 0, 1, 2, 3] {
            for kind in ["asin", "acos", "atanh", "acosh"] {
                let child = Computable::rational(Rational::fraction(numerator, 2).unwrap());
                child.structural_facts();
                let value = diagnostic_outer(kind, child);
                let valid = match kind {
                    "asin" | "acos" => (-2..=2).contains(&numerator),
                    "atanh" => (-1..=1).contains(&numerator),
                    "acosh" => numerator >= 2,
                    _ => unreachable!(),
                };
                let domain = value.ensure_finite_real(&None);
                assert_eq!(domain.is_ok(), valid);
                let answer = value.approx_with_budget(0, &EvaluationBudget::new(100_000));
                eprintln!(
                    "domain bound endpoint n={numerator}, kind={kind}, valid={valid}, domain={domain:?}, answer={answer:?}"
                );
                assert_eq!(answer.is_ok(), valid);
                if !valid {
                    assert_eq!(answer, Err(Problem::NotANumber));
                    assert!(value.internal.cache_snapshot().is_none());
                }
            }
        }
        for bad_kind in ["Inverse", "Sqrt", "LnDeferred"] {
            for kind in ["asin", "acos", "atanh", "acosh", "tan", "cot-series"] {
                let argument =
                    Computable::rational(Rational::new(if bad_kind == "Inverse" { 0 } else { -1 }));
                let bad: Computable =
                    serde_json::from_value(serde_json::json!({"internal": {bad_kind: argument}}))
                        .unwrap();
                let square: Computable =
                    serde_json::from_value(serde_json::json!({"internal": {"Square": bad}}))
                        .unwrap();
                let child: Computable = serde_json::from_value(
                    serde_json::json!({"internal": {"Offset": [square, -8]}}),
                )
                .unwrap();
                child.structural_facts();
                let value = diagnostic_outer(kind, child);
                assert!(!value.has_finite_real_certificate());
                let answer = value.approx_with_budget(8, &EvaluationBudget::new(100_000));
                eprintln!(
                    "domain bound invalid descendant={bad_kind}, kind={kind}, answer={answer:?}"
                );
                assert!(matches!(
                    answer,
                    Err(Problem::DivideByZero | Problem::SqrtNegative | Problem::NotANumber)
                ));
                assert!(value.internal.cache_snapshot().is_none());
                assert!(!value.internal.facts.finite_real());
            }
        }
    }

    fn oracle(kind: &str, shift: u32, squared: bool, negative: bool) -> (Q, Q) {
        let mut lo = Float::with_val_round(256, Constant::Pi, Round::Down)
            .0
            .to_rational()
            .unwrap();
        let mut hi = Float::with_val_round(256, Constant::Pi, Round::Up)
            .0
            .to_rational()
            .unwrap();
        let divisor = Integer::from(1) << shift;
        lo /= &divisor;
        hi /= &divisor;
        if squared {
            lo *= lo.clone();
            hi *= hi.clone();
        }
        if negative {
            (lo, hi) = (-hi, -lo);
        }
        if kind == "acos" {
            (lo, hi) = (hi, lo);
        }
        let mut lo = Float::with_val_round(
            256,
            &lo,
            if kind == "acos" {
                Round::Up
            } else {
                Round::Down
            },
        )
        .0;
        let mut hi = Float::with_val_round(
            256,
            &hi,
            if kind == "acos" {
                Round::Down
            } else {
                Round::Up
            },
        )
        .0;
        match kind {
            "asin" => {
                lo.asin_round(Round::Down);
                hi.asin_round(Round::Up);
            }
            "acos" => {
                lo.acos_round(Round::Down);
                hi.acos_round(Round::Up);
            }
            "atanh" => {
                lo.atanh_round(Round::Down);
                hi.atanh_round(Round::Up);
            }
            _ => unreachable!(),
        }
        (lo.to_rational().unwrap(), hi.to_rational().unwrap())
    }

    fn check_sample(sample: &BigInt, p: Precision, bounds: &(Q, Q)) {
        let unit = if p >= 0 {
            Q::from(Integer::from(1) << p)
        } else {
            Q::from((1, Integer::from(1) << -p))
        };
        let center = Q::from(Integer::from_str_radix(&sample.to_string(), 10).unwrap()) * &unit;
        assert!(Q::from(&center - &unit) <= bounds.0);
        assert!(Q::from(&center + &unit) >= bounds.1);
    }

    fn public_cutpoints(kind: &str) {
        // Explicitly warm pi's shared numerical cache through its public API.
        // Other shared constants may evolve in this exact process order.
        Computable::pi().approx(-512).unwrap();
        for shift in [2, 4, 8] {
            for squared in [false, true] {
                for negative in [false, true] {
                    let bounds = oracle(kind, shift, squared, negative);
                    for state in ["constructed", "child-coarse", "child-fine", "roundtrip"] {
                        for units in [0, 1, 2, 4, 8, 16, 32, 64, 128, 512, 4096] {
                            let child = scaled_pi(shift, squared, negative);
                            if state == "child-coarse" {
                                child.structural_facts();
                                child.approx(8).unwrap();
                            } else if state == "child-fine" {
                                child.approx(-64).unwrap();
                            }
                            let value = public_outer(kind, child);
                            let value = if state == "roundtrip" {
                                serde_json::from_value(serde_json::to_value(value).unwrap())
                                    .unwrap()
                            } else {
                                value
                            };
                            let before = value.internal.cache_snapshot();
                            let before_sign = value.immediate_sign();
                            let budget = EvaluationBudget::new(units);
                            let (answer, allocations) =
                                measure(|| value.approx_with_budget(0, &budget));
                            assert_eq!(budget.consumed() + budget.remaining(), units);
                            let after = value.internal.cache_snapshot();
                            let zero = EvaluationBudget::new(0);
                            let sign = value.sign_until_with_budget(0, &zero);
                            if let Some(known) = before_sign {
                                assert_eq!(sign, Some(known));
                            }
                            if let Some(known) = sign {
                                assert_eq!(
                                    known,
                                    if negative && kind != "acos" {
                                        RealSign::Negative
                                    } else {
                                        RealSign::Positive
                                    }
                                );
                            }
                            eprintln!(
                                "domain bound public kind={kind}, shift={shift}, squared={squared}, negative={negative}, state={state}, units={units}, answer={answer:?}, consumed={}, before_sign={before_sign:?}, sign={sign:?}, before={before:?}, after={after:?}, allocations={allocations:?}",
                                budget.consumed()
                            );
                            match &answer {
                                Ok(sample) => {
                                    check_sample(sample, 0, &bounds);
                                    assert_eq!(value.approx_with_budget(0, &zero), answer);
                                }
                                Err(problem) => {
                                    assert_eq!(*problem, Problem::Exhausted);
                                    assert_eq!(after, before);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn domain_bound_public_asin_cutpoints() {
        public_cutpoints("asin");
    }

    #[test]
    fn domain_bound_public_acos_cutpoints() {
        public_cutpoints("acos");
    }

    #[test]
    fn domain_bound_public_atanh_cutpoints() {
        public_cutpoints("atanh");
    }

    #[test]
    fn domain_bound_public_fine_retries_preserve_cached_facts() {
        for kind in ["asin", "acos", "atanh"] {
            for shift in [2, 4, 8] {
                for squared in [false, true] {
                    for negative in [false, true] {
                        let bounds = oracle(kind, shift, squared, negative);
                        let value = public_outer(kind, scaled_pi(shift, squared, negative));
                        let budget = EvaluationBudget::new(8);
                        let first = value.approx_with_budget(-64, &budget);
                        if let Ok(ref sample) = first {
                            check_sample(sample, -64, &bounds);
                        } else {
                            assert_eq!(first, Err(Problem::Exhausted));
                        }
                        let retry = value.approx(-64).unwrap();
                        check_sample(&retry, -64, &bounds);
                        let sign = value.sign_until_with_budget(0, &EvaluationBudget::new(0));
                        let expected = if negative && kind != "acos" {
                            RealSign::Negative
                        } else {
                            RealSign::Positive
                        };
                        assert_eq!(sign, Some(expected));
                        for p in [-64, -32, 0, 8] {
                            let empty = EvaluationBudget::new(0);
                            check_sample(&value.approx_with_budget(p, &empty).unwrap(), p, &bounds);
                            assert_eq!(value.sign_until_with_budget(0, &empty), Some(expected));
                            assert_eq!(empty.consumed(), 0);
                        }
                        eprintln!(
                            "domain bound retry kind={kind}, shift={shift}, squared={squared}, negative={negative}, first={first:?}, consumed={}, retry={retry}, sign={sign:?}",
                            budget.consumed()
                        );
                    }
                }
            }
        }
    }
}
