#[cfg(test)]
mod algebra_domain_tests {
    use super::*;

    fn fresh(kind: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(kind, BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        }
    }

    fn undefined(case: usize) -> Computable {
        match case {
            0 => fresh(Approximation::Sqrt(Computable::rational(Rational::new(-1)))),
            1 => fresh(Approximation::Inverse(Computable::zero())),
            2 => fresh(Approximation::LnDeferred(Computable::rational(Rational::new(-1)))),
            3 => fresh(Approximation::AsinDeferred(Computable::rational(Rational::new(2)))),
            4 => fresh(Approximation::AtanhDirect(Computable::one())),
            5 => Computable::normal_quantile(Computable::zero(), BigInt::zero(), 0),
            _ => unreachable!(),
        }
    }

    const FORM_COUNT: usize = 17;

    #[test]
    fn coarse_requests_cannot_hide_undefined_descendants() {
        let mut failures = Vec::new();
        let ambiguous_negative = || fresh(Approximation::Add(
            Computable::one(),
            fresh(Approximation::Negate(fresh(Approximation::Add(
                Computable::one(), Computable::one().shift_left(-300),
            )))),
        ));
        for precision in [0, 8, Precision::MAX] {
            for (name, value) in [
                ("cos", fresh(Approximation::PrescaledCos(undefined(0)))),
                ("sin", fresh(Approximation::PrescaledSin(undefined(0)))),
                ("deferred_cos", fresh(Approximation::CosDeferred(undefined(0)))),
                ("deferred_sin", fresh(Approximation::SinDeferred(undefined(0)))),
                ("sinc", fresh(Approximation::SincSmall(undefined(0)))),
                ("cosc", fresh(Approximation::CoscSmall(undefined(0)))),
                ("tiny_sqrt", fresh(Approximation::Sqrt(fresh(Approximation::Negate(fresh(Approximation::Offset(Computable::one(), -300))))))),
                ("tiny_cube_root", fresh(Approximation::NthRoot(fresh(Approximation::Negate(fresh(Approximation::Offset(Computable::one(), -300)))), 3))),
                ("ambiguous_sqrt", fresh(Approximation::Sqrt(ambiguous_negative()))),
                ("ambiguous_cube_root", fresh(Approximation::NthRoot(ambiguous_negative(), 3))),
            ] {
                let result = value.approx(precision);
                if result.is_ok() || value.internal.cache_snapshot().is_some() {
                    failures.push(format!("{name}, p={precision}: {result:?}"));
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[test]
    fn raw_sinc_coarse_results_allow_negative_values() {
        let mut failures = Vec::new();
        for precision in [0, 8, Precision::MAX] {
            let angle = Computable::pi().multiply_rational(Rational::fraction(3, 2).unwrap());
            let value = fresh(Approximation::SincSmall(angle));
            // sin(3*pi/2)/(3*pi/2) = -2/(3*pi), strictly between -1 and 0.
            // At these precisions only -1 or 0 can enclose that negative value.
            let result = value.approx(precision);
            if !matches!(&result, Ok(sample) if *sample == BigInt::from(-1) || sample.is_zero()) {
                failures.push(format!("p={precision}: {result:?}"));
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[test]
    fn real_domain_validation_handles_deep_graphs_without_evaluating_them() {
        let mut retained = vec![Computable::one()];
        for _ in 0..2048 {
            retained.push(fresh(Approximation::PrescaledExp(retained.last().unwrap().clone())));
        }
        let root = retained.last().unwrap();
        assert!(!root.has_finite_real_certificate());
        let zero = Computable::zero().multiply(root.clone());
        assert_eq!(zero.approx(-64), Ok(BigInt::zero()));
        assert!(root.internal.facts.finite_real());
        assert!(retained.iter().all(|value| value.internal.cache_snapshot().is_none()));
        assert!(Computable::zero().multiply(root.clone()).exact_rational().unwrap().is_zero());
        drop(zero);
        // Explicit reverse release keeps destruction of the fixture iterative.
        while retained.pop().is_some() {}
    }

    #[test]
    fn finite_real_cache_survives_fact_updates_and_abort_overrides() {
        use std::sync::atomic::AtomicBool;
        let value = fresh(Approximation::Erfc(Computable::pi()));
        value.ensure_finite_real(&None).unwrap();
        assert!(value.internal.facts.finite_real());
        value.internal.facts.set_bound(BoundCache::Invalid);
        value.internal.facts.set_bound_if_invalid(BoundCache::Valid(BoundInfo::Unknown));
        value.internal.facts.set_bound_if_unresolved(BoundCache::Valid(BoundInfo::with_sign(Sign::Plus, None)));
        value.internal.facts.replace_exact_sign(ExactSignCache::Unknown);
        assert!(value.internal.facts.finite_real());
        assert_eq!(value.ensure_finite_real(&Some(Arc::new(AtomicBool::new(true)))), Err(Problem::Exhausted));
        assert!(value.ensure_finite_real(&None).is_ok());
        assert!(value.internal.cache_snapshot().is_none());
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serialized_domain_cache_is_reproved_and_invalid_facts_stay_unknown() {
        let valid = fresh(Approximation::Erfc(Computable::pi()));
        valid.ensure_finite_real(&None).unwrap();
        let decoded: Computable = serde_json::from_str(&serde_json::to_string(&valid).unwrap()).unwrap();
        assert!(!decoded.internal.facts.finite_real());
        decoded.ensure_finite_real(&None).unwrap();
        assert!(decoded.internal.facts.finite_real());
        for operand in 0..6 {
            for form in 0..FORM_COUNT {
                let value = expression(form, undefined(operand));
                let decoded: Computable = serde_json::from_str(&serde_json::to_string(&value).unwrap()).unwrap();
                assert_eq!(decoded.exact_sign(), None, "operand={operand}, form={form}");
                assert_eq!(decoded.zero_status(), ZeroKnowledge::Unknown, "operand={operand}, form={form}");
                for precision in [0, -64, Precision::MAX] {
                    assert!(decoded.approx(precision).is_err(), "operand={operand}, form={form}, p={precision}");
                    assert!(decoded.internal.cache_snapshot().is_none());
                    assert!(!decoded.internal.facts.finite_real());
                }
            }
        }
    }

    #[test]
    fn every_observed_domain_validation_stop_reaches_the_root() {
        use std::sync::atomic::{AtomicBool, Ordering};
        let mut completed = false;
        for stop_after in 0..2048 {
            let mut child = Computable::one();
            for _ in 0..48 {
                child = fresh(Approximation::PrescaledExp(child));
            }
            let value = Computable::zero().multiply(child);
            let signal = Arc::new(AtomicBool::new(false));
            CLEAR_STOP_AFTER_POLLS.with(|polls| polls.set(Some(stop_after)));
            let result = value.approx_signal(&Some(signal.clone()), -64);
            let observed = CLEAR_STOP_AFTER_POLLS.with(|polls| polls.take().is_none());
            assert!(!signal.load(Ordering::Relaxed));
            if !observed {
                assert_eq!(result, Ok(BigInt::zero()));
                completed = true;
                break;
            }
            assert_eq!(result, Err(Problem::Exhausted), "poll={stop_after}");
            assert!(value.internal.cache_snapshot().is_none());
            assert_eq!(value.approx(-64), Ok(BigInt::zero()));
        }
        assert!(completed);
    }

    fn expression(form: usize, x: Computable) -> Computable {
        let sum = |a, b| fresh(Approximation::Add(a, b));
        let affine = |reverse| {
            let two = Computable::rational(Rational::new(2));
            let product = if reverse {
                fresh(Approximation::Multiply(two, x.clone()))
            } else {
                fresh(Approximation::Multiply(x.clone(), two))
            };
            sum(product, Computable::one())
        };
        match form {
            0 => Computable::zero().multiply(x),
            1 => x.multiply(Computable::zero()),
            2 => x.multiply_rational(Rational::zero()),
            3 => x.clone().add(x.negate()),
            4 => x.clone().negate().add(x),
            5 => sum(x.clone(), Computable::one()).add(x.negate()),
            6 => sum(Computable::one(), x.clone()).add(x.negate()),
            7 => x.clone().negate().add(sum(x, Computable::one())),
            8 => x.clone().negate().add(sum(Computable::one(), x)),
            9 => sum(x.clone(), Computable::one()).add(sum(Computable::one(), x).negate()),
            10 => Computable::normal_interval(x.clone(), x),
            11 => affine(false).add(affine(true).negate()),
            12 => affine(false).negate().add(affine(true)),
            13 => Computable::linear_combination3(
                [x, Computable::one(), Computable::one()],
                [Rational::zero(), Rational::one(), Rational::new(-1)],
            ),
            14 => fresh(Approximation::Multiply(x, Computable::zero())),
            15 => fresh(Approximation::Add(x.clone(), x.negate())),
            16 => fresh(Approximation::NormalInterval { lo: x.clone(), hi: x }),
            _ => unreachable!(),
        }
    }

    #[test]
    fn undefined_operands_cannot_be_cancelled_into_structural_facts() {
        let mut failures = Vec::new();
        for operand in 0..6 {
            for form in 0..FORM_COUNT {
                for api in 0..3 {
                    let value = expression(form, undefined(operand));
                    let definite = match api {
                        0 => value.cheap_bound() != BoundInfo::Unknown,
                        1 => value.exact_sign().is_some(),
                        2 => {
                            let facts = value.structural_facts();
                            facts.sign.is_some() || facts.zero != ZeroKnowledge::Unknown
                        }
                        _ => unreachable!(),
                    };
                    if definite {
                        failures.push(format!("undefined={operand}, form={form}, fact={api}"));
                    }
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[test]
    fn undefined_operands_cannot_be_cancelled_into_numeric_values() {
        let mut failures = Vec::new();
        for operand in 0..6 {
            for form in 0..FORM_COUNT {
                for precision in [-64, 0, Precision::MAX] {
                    let value = expression(form, undefined(operand));
                    let result = value.approx(precision);
                    if result.is_ok() {
                        failures.push(format!("undefined={operand}, form={form}, p={precision}: {result:?}"));
                    }
                    if value.internal.cache_snapshot().is_some() {
                        failures.push(format!("undefined={operand}, form={form}, p={precision}: cached invalid result"));
                    }
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[test]
    fn valid_cancellation_and_zero_scaling_keep_exact_identities() {
        for form in 0..FORM_COUNT {
            for operand in 0..5 {
                let x = match operand {
                    0 => fresh(Approximation::PrescaledSin(Computable::pi().shift_left(-4))),
                    1 => fresh(Approximation::Add(Computable::pi(), Computable::rational(Rational::new(-3)))),
                    2 => fresh(Approximation::Sqrt(Computable::rational(Rational::new(2)))),
                    3 => Computable::ln2(),
                    4 => fresh(Approximation::Erfc(Computable::pi())),
                    _ => unreachable!(),
                };
                let value = expression(form, x);
                let expected = if (5..=8).contains(&form) { Sign::Plus } else { Sign::NoSign };
                if form <= 12 {
                    assert_eq!(value.exact_sign(), Some(expected), "form={form}, operand={operand}");
                }
                for precision in [-64_i32, 0] {
                    let sample = value.approx(precision).unwrap();
                    let expected = if expected == Sign::Plus { BigInt::one() << precision.unsigned_abs() } else { BigInt::zero() };
                    assert!((sample - expected).abs() <= BigInt::one(), "form={form}, operand={operand}, p={precision}");
                }
            }
        }
    }
}
