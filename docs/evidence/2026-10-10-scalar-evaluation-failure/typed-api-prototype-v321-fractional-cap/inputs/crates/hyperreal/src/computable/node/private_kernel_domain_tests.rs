#[cfg(test)]
mod private_kernel_domain_tests {
    use super::*;
    use rug::{Float, Integer, Rational as Q, float::Round};

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

    fn inverse_series(kind: usize, rational: Rational) -> Computable {
        match kind {
            0 => fresh(Approximation::PrescaledAtan(Computable::rational(rational))),
            1 => fresh(Approximation::PrescaledAsinh(Computable::rational(
                rational,
            ))),
            2 => fresh(Approximation::AsinhRational(rational)),
            _ => unreachable!(),
        }
    }

    fn inverse_enclosure(kind: usize, numerator: i64, denominator: u64) -> (Q, Q) {
        let exact = Q::from((numerator, denominator));
        let mut lower = Float::with_val(1024, &exact);
        assert_eq!(lower.to_rational().unwrap(), exact);
        let mut upper = lower.clone();
        if kind == 0 {
            lower.atan_round(Round::Down);
            upper.atan_round(Round::Up);
        } else {
            lower.asinh_round(Round::Down);
            upper.asinh_round(Round::Up);
        }
        (lower.to_rational().unwrap(), upper.to_rational().unwrap())
    }

    fn enclosed(actual: &BigInt, precision: i32, lower: &Q, upper: &Q) -> bool {
        let unit = if precision > 0 {
            Q::from(Integer::from(1) << precision)
        } else {
            Q::from((1, Integer::from(1) << -precision))
        };
        let center = Q::from(Integer::from_str_radix(&actual.to_string(), 10).unwrap()) * &unit;
        Q::from(&center - &unit) <= *lower && Q::from(&center + &unit) >= *upper
    }

    #[test]
    fn raw_asinh_coarse_samples_match_directed_enclosures() {
        let mut failures = Vec::new();
        for kind in [1, 2] {
            for numerator in [-32, -8, -2, -1, 0, 1, 2, 8, 32] {
                let (lower, upper) = inverse_enclosure(kind, numerator, 1);
                for precision in [1, 2, 4] {
                    let value = inverse_series(kind, Rational::new(numerator));
                    let actual = value.approx(precision).unwrap();
                    if !enclosed(&actual, precision, &lower, &upper) {
                        failures.push(format!(
                            "kind={kind}, input={numerator}, p={precision}, sample={actual}"
                        ));
                    }
                }
                let value = inverse_series(kind, Rational::new(numerator));
                assert_eq!(value.approx(i32::MAX), Ok(BigInt::zero()));
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[test]
    fn raw_inverse_series_match_directed_enclosures() {
        for kind in 0..3 {
            for (numerator, denominator) in [
                (0, 1),
                (-1, 2),
                (1, 2),
                (-1, 1),
                (1, 1),
                (-2, 1),
                (2, 1),
                (-8, 1),
                (8, 1),
                (-127, 256),
                (127, 256),
                (-129, 256),
                (129, 256),
            ] {
                let rational = Rational::fraction(numerator, denominator).unwrap();
                let (lower, upper) = inverse_enclosure(kind, numerator, denominator);
                let value = inverse_series(kind, rational.clone());
                let control = if kind == 0 {
                    Computable::rational(rational).atan()
                } else {
                    Computable::rational(rational).asinh()
                };
                let mut values = vec![value.clone(), value, control];
                #[cfg(feature = "serde")]
                {
                    let mut bytes = Vec::new();
                    ciborium::ser::into_writer(&values[0], &mut bytes).unwrap();
                    values.push(ciborium::de::from_reader(bytes.as_slice()).unwrap());
                }
                for (view, value) in values.drain(..).enumerate() {
                    for precision in [1, 0, -32, -128, -256, -8, 1] {
                        let actual = value.approx(precision).unwrap();
                        assert!(
                            enclosed(&actual, precision, &lower, &upper),
                            "kind={kind}, input={numerator}/{denominator}, view={view}, p={precision}: {actual}"
                        );
                        if let Some(sign) = value.structural_facts().sign {
                            assert_eq!(sign, public_sign(Rational::new(numerator).sign()));
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn inverse_series_range_fallback_preserves_observed_stops() {
        use std::sync::atomic::{AtomicBool, Ordering};
        for kind in 0..3 {
            for numerator in [-2, 2] {
                let (lower, upper) = inverse_enclosure(kind, numerator, 1);
                let mut completed = false;
                for stop_after in 0..2048 {
                    let value = inverse_series(kind, Rational::new(numerator));
                    let signal = Arc::new(AtomicBool::new(false));
                    CLEAR_STOP_AFTER_POLLS.with(|polls| polls.set(Some(stop_after)));
                    let result = value.approx_signal(&Some(signal.clone()), -64);
                    let observed = CLEAR_STOP_AFTER_POLLS.with(|polls| polls.take().is_none());
                    assert!(!signal.load(Ordering::Relaxed));
                    if !observed {
                        assert!(enclosed(&result.unwrap(), -64, &lower, &upper));
                        completed = true;
                        break;
                    }
                    assert_eq!(
                        result,
                        Err(Problem::Exhausted),
                        "kind={kind}, input={numerator}, poll={stop_after}"
                    );
                    assert!(value.internal.cache_snapshot().is_none());
                    assert!(enclosed(&value.approx(-64).unwrap(), -64, &lower, &upper));
                }
                assert!(completed, "kind={kind}, input={numerator}");
            }
        }
    }

    fn angle(numerator: i64, denominator: u64, perturbation: i64) -> Computable {
        let value = Computable::pi().multiply(Computable::rational(
            Rational::fraction(numerator, denominator).unwrap(),
        ));
        if perturbation == 0 {
            value
        } else {
            value.add(Computable::rational(
                Rational::fraction(perturbation, 1024).unwrap(),
            ))
        }
    }

    fn tangent(kind: usize, child: Computable) -> Computable {
        match kind {
            0 => fresh(Approximation::PrescaledTan(child)),
            1 => fresh(Approximation::TanDeferred(child)),
            2 => fresh(Approximation::PrescaledCot(child)),
            _ => unreachable!(),
        }
    }

    fn public_tangent(kind: usize, child: Computable) -> Computable {
        if kind == 2 {
            child.clone().cos().multiply(child.sin().inverse())
        } else {
            child.tan()
        }
    }

    #[test]
    fn exact_trig_poles_return_domain_errors_at_every_precision() {
        for kind in 0..3 {
            for numerator in [-5, -3, -1, 0, 1, 3, 5] {
                if numerator == 0 && kind != 2 {
                    continue;
                }
                let child = angle(numerator, if kind == 2 { 1 } else { 2 }, 0);
                let value = tangent(kind, child.clone());
                let mut values = vec![value.clone(), value, public_tangent(kind, child)];
                #[cfg(feature = "serde")]
                {
                    let mut bytes = Vec::new();
                    ciborium::ser::into_writer(&values[0], &mut bytes).unwrap();
                    values.push(ciborium::de::from_reader(bytes.as_slice()).unwrap());
                }
                for (view, value) in values.drain(..).enumerate() {
                    for precision in [i32::MAX, 4, 0, -32, -256, 0] {
                        assert_eq!(
                            value.approx(precision),
                            Err(Problem::DivideByZero),
                            "kind={kind}, numerator={numerator}, view={view}, p={precision}"
                        );
                        assert!(value.internal.cache_snapshot().is_none());
                    }
                }
            }
        }
    }

    fn tangent_enclosure(kind: usize, numerator: i64, delta: i64) -> (Q, Q) {
        use rug::{float::Constant, ops::AddAssignRound};
        assert!(matches!(numerator, -1..=1) && matches!(delta, -1 | 1));
        let (mut lower, _) = Float::with_val_round(1024, Constant::Pi, Round::Down);
        let (mut upper, _) = Float::with_val_round(1024, Constant::Pi, Round::Up);
        if numerator < 0 {
            (lower, upper) = (-upper, -lower);
        } else if numerator == 0 {
            lower = Float::with_val(1024, 0);
            upper = lower.clone();
        }
        if kind != 2 {
            lower /= 2;
            upper /= 2;
        }
        let shift = Float::with_val(1024, Q::from((delta, 1024)));
        lower.add_assign_round(&shift, Round::Down);
        upper.add_assign_round(&shift, Round::Up);
        if kind == 2 {
            // cot decreases on each interval between its poles.
            lower.cot_round(Round::Up);
            upper.cot_round(Round::Down);
            (lower, upper) = (upper, lower);
        } else {
            lower.tan_round(Round::Down);
            upper.tan_round(Round::Up);
        }
        (lower.to_rational().unwrap(), upper.to_rational().unwrap())
    }

    #[test]
    fn nearby_trig_poles_match_directed_enclosures() {
        for kind in 0..3 {
            for numerator in [-1, 0, 1] {
                if numerator == 0 && kind != 2 {
                    continue;
                }
                for delta in [-1, 1] {
                    let child = angle(numerator, if kind == 2 { 1 } else { 2 }, delta);
                    let (lower, upper) = tangent_enclosure(kind, numerator, delta);
                    let value = tangent(kind, child.clone());
                    let mut values = vec![value.clone(), value, public_tangent(kind, child)];
                    #[cfg(feature = "serde")]
                    {
                        let mut bytes = Vec::new();
                        ciborium::ser::into_writer(&values[0], &mut bytes).unwrap();
                        values.push(ciborium::de::from_reader(bytes.as_slice()).unwrap());
                    }
                    for (view, value) in values.drain(..).enumerate() {
                        for precision in [4, 0, -32, -128, -256, 4] {
                            let actual = value.approx(precision).unwrap();
                            assert!(
                                enclosed(&actual, precision, &lower, &upper),
                                "kind={kind}, numerator={numerator}, delta={delta}, view={view}, p={precision}: {actual}"
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn trig_pole_domain_checks_preserve_observed_stops() {
        use std::sync::atomic::{AtomicBool, Ordering};
        for kind in 0..3 {
            for numerator in [-1, 1] {
                for delta in [0, 1] {
                    let mut completed = false;
                    for stop_after in 0..2048 {
                        let value =
                            tangent(kind, angle(numerator, if kind == 2 { 1 } else { 2 }, delta));
                        let signal = Arc::new(AtomicBool::new(false));
                        CLEAR_STOP_AFTER_POLLS.with(|polls| polls.set(Some(stop_after)));
                        let result = value.ensure_finite_real(&Some(signal.clone()));
                        let observed = CLEAR_STOP_AFTER_POLLS.with(|polls| polls.take().is_none());
                        assert!(!signal.load(Ordering::Relaxed));
                        assert!(value.internal.cache_snapshot().is_none());
                        let expected = if delta == 0 {
                            Err(Problem::DivideByZero)
                        } else {
                            Ok(())
                        };
                        if !observed {
                            assert_eq!(result, expected);
                            completed = true;
                            break;
                        }
                        assert_eq!(
                            result,
                            Err(Problem::Exhausted),
                            "kind={kind}, numerator={numerator}, delta={delta}, poll={stop_after}"
                        );
                        assert_eq!(value.ensure_finite_real(&None), expected);
                    }
                    assert!(
                        completed,
                        "kind={kind}, numerator={numerator}, delta={delta}"
                    );
                }
            }
        }
    }

    #[test]
    fn trig_zero_certificates_retain_facts_and_bound_coefficient_work() {
        for numerator in -16..=16 {
            for denominator in 1..=9 {
                for cosine in [false, true] {
                    let child = angle(numerator, denominator, 0);
                    let value = fresh(if cosine {
                        Approximation::CosDeferred(child)
                    } else {
                        Approximation::SinDeferred(child)
                    });
                    let denominator = denominator as i64;
                    let expected = if cosine {
                        2 * numerator % denominator == 0 && (2 * numerator / denominator) % 2 != 0
                    } else {
                        numerator % denominator == 0
                    };
                    assert_eq!(
                        value.has_trig_zero_certificate_when_real(),
                        expected,
                        "cosine={cosine}, numerator={numerator}, denominator={denominator}"
                    );
                }
            }
        }
        for kind in 0..4 {
            let child = angle(1, if kind < 2 { 1 } else { 2 }, 0);
            let zero = fresh(match kind {
                0 => Approximation::PrescaledSin(child),
                1 => Approximation::SinDeferred(child),
                2 => Approximation::PrescaledCos(child),
                _ => Approximation::CosDeferred(child),
            });
            zero.ensure_finite_real(&None).unwrap();
            let _ = zero.structural_facts();
            assert_eq!(
                zero.certify_required_domain(
                    &None,
                    RequiredRealDomain::Nonzero,
                    Problem::DivideByZero
                ),
                Err(Problem::DivideByZero)
            );
            assert_eq!(zero.clone().structural_facts().sign, Some(RealSign::Zero));
            assert!(zero.internal.cache_snapshot().is_none());
        }
        // The recognizer must not allocate a two-billion-bit coefficient.
        for shift in [i32::MIN, i32::MAX] {
            let child = fresh(Approximation::Offset(Computable::pi(), shift));
            let value = fresh(Approximation::SinDeferred(child));
            assert!(!value.has_trig_zero_certificate_when_real());
        }
        let invalid = fresh(Approximation::Inverse(Computable::zero()));
        let annihilated = fresh(Approximation::Multiply(Computable::zero(), invalid));
        let sine = fresh(Approximation::SinDeferred(annihilated));
        let inverse = fresh(Approximation::Inverse(sine.clone()));
        assert_eq!(inverse.approx(-64), Err(Problem::DivideByZero));
        assert_eq!(sine.structural_facts().sign, None);
        assert!(sine.internal.cache_snapshot().is_none());
    }
}
