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
            1 => fresh(Approximation::PrescaledAsinh(Computable::rational(rational))),
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
                        failures.push(format!("kind={kind}, input={numerator}, p={precision}, sample={actual}"));
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
                (0, 1), (-1, 2), (1, 2), (-1, 1), (1, 1), (-2, 1), (2, 1),
                (-8, 1), (8, 1), (-127, 256), (127, 256), (-129, 256), (129, 256),
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
                    assert_eq!(result, Err(Problem::Exhausted), "kind={kind}, input={numerator}, poll={stop_after}");
                    assert!(value.internal.cache_snapshot().is_none());
                    assert!(enclosed(&value.approx(-64).unwrap(), -64, &lower, &upper));
                }
                assert!(completed, "kind={kind}, input={numerator}");
            }
        }
    }
}
