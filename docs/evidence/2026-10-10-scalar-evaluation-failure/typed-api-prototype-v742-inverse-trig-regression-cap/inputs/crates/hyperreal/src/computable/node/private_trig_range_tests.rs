#[cfg(test)]
mod private_trig_range_tests {
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

    fn raw(kind: usize, argument: Computable, rational: Rational) -> Computable {
        fresh(match kind {
            0 => Approximation::SincSmall(argument),
            1 => Approximation::CoscSmall(argument),
            2 => Approximation::PrescaledSin(argument),
            3 => Approximation::PrescaledCos(argument),
            4 => Approximation::PrescaledSinRational(rational),
            5 => Approximation::PrescaledCosRational(rational),
            _ => unreachable!(),
        })
    }

    fn public(kind: usize, rational: Rational) -> Computable {
        let child = Computable::rational(rational.clone());
        match kind {
            0 if rational.sign() == Sign::NoSign => Computable::one(),
            1 if rational.sign() == Sign::NoSign => Computable::half(),
            0 => child.clone().sin().multiply(child.inverse()),
            1 => Computable::one()
                .add(child.clone().cos().negate())
                .multiply(child.square().inverse()),
            2 | 4 => child.sin(),
            3 | 5 => child.cos(),
            _ => unreachable!(),
        }
    }

    fn enclosure(kind: usize, numerator: i64, denominator: u64) -> (Q, Q) {
        if numerator == 0 && kind < 2 {
            let value = Q::from((1, kind as i32 + 1));
            return (value.clone(), value);
        }
        let exact = Q::from((numerator, denominator));
        let mut lower = Float::with_val(1024, &exact);
        assert_eq!(lower.to_rational().unwrap(), exact);
        let mut upper = lower.clone();
        if matches!(kind, 1 | 3 | 5) {
            lower.cos_round(Round::Down);
            upper.cos_round(Round::Up);
        } else {
            lower.sin_round(Round::Down);
            upper.sin_round(Round::Up);
        }
        let (lower, upper) = (lower.to_rational().unwrap(), upper.to_rational().unwrap());
        match kind {
            0 => {
                let (lower, upper) = (lower / &exact, upper / exact);
                if numerator < 0 {
                    (upper, lower)
                } else {
                    (lower, upper)
                }
            }
            1 => {
                let square = exact.clone() * exact;
                (
                    (Q::from(1) - upper) / &square,
                    (Q::from(1) - lower) / square,
                )
            }
            _ => (lower, upper),
        }
    }

    fn enclosed(sample: &BigInt, precision: i32, lower: &Q, upper: &Q) -> bool {
        let unit = if precision > 0 {
            Q::from(Integer::from(1) << precision)
        } else {
            Q::from((1, Integer::from(1) << -precision))
        };
        let center = Q::from(Integer::from_str_radix(&sample.to_string(), 10).unwrap()) * &unit;
        Q::from(&center - &unit) <= *lower && Q::from(&center + &unit) >= *upper
    }

    #[test]
    fn raw_trig_ranges_match_directed_enclosures() {
        let mut inputs = vec![(0, 1)];
        for (n, d) in [
            (1, 16),
            (1, 2),
            (191, 256),
            (3, 4),
            (193, 256),
            (511, 256),
            (2, 1),
            (513, 256),
            (63, 2),
            (65, 2),
            (64, 1),
            (257, 2),
            (512, 1),
            (1024, 1),
            (65536, 1),
            (1048576, 1),
        ] {
            inputs.extend([(-n, d), (n, d)]);
        }
        for kind in 0..6 {
            for &(numerator, denominator) in &inputs {
                let rational = Rational::fraction(numerator, denominator).unwrap();
                let (lower, upper) = enclosure(kind, numerator, denominator);
                let value = raw(
                    kind,
                    Computable::rational(rational.clone()),
                    rational.clone(),
                );
                let mut views = vec![value.clone(), value, public(kind, rational)];
                #[cfg(feature = "serde")]
                {
                    let mut bytes = Vec::new();
                    ciborium::ser::into_writer(&views[0], &mut bytes).unwrap();
                    views.push(ciborium::de::from_reader(bytes.as_slice()).unwrap());
                }
                for (view, value) in views.drain(..).enumerate() {
                    for precision in [1, 0, -4, -64, -256, -4] {
                        let sample = value.approx(precision).unwrap();
                        assert!(
                            enclosed(&sample, precision, &lower, &upper),
                            "kind={kind}, input={numerator}/{denominator}, view={view}, p={precision}: {sample}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn trig_range_reduction_preserves_observed_stops() {
        use std::sync::atomic::{AtomicBool, Ordering};
        for kind in 0..6 {
            for numerator in [-1048576, 1048576] {
                for shape in 0..if kind < 4 { 3 } else { 1 } {
                    let (lower, upper) = enclosure(kind, numerator, 1);
                    let mut completed = false;
                    for stop_after in 0..8192 {
                        let rational = Rational::new(numerator);
                        let signal = Arc::new(AtomicBool::new(false));
                        let mut argument = if shape == 2 {
                            fresh(Approximation::Add(
                                Computable::rational(-rational.clone()),
                                Computable::zero(),
                            ))
                        } else if shape == 1 {
                            fresh(Approximation::Add(
                                Computable::rational(rational.clone()),
                                Computable::zero(),
                            ))
                        } else {
                            Computable::rational(rational.clone())
                        };
                        argument.signal = Some(signal.clone());
                        if shape == 2 {
                            argument = fresh(Approximation::Negate(argument));
                            argument.signal = Some(signal.clone());
                        }
                        let value = raw(kind, argument, rational);
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
                            "kind={kind}, input={numerator}, shape={shape}, poll={stop_after}"
                        );
                        assert!(value.internal.cache_snapshot().is_none());
                        assert!(enclosed(&value.approx(-64).unwrap(), -64, &lower, &upper));
                    }
                    assert!(completed, "kind={kind}, input={numerator}, shape={shape}");
                }
            }
        }
    }

    #[test]
    fn trig_range_boundaries_accept_every_valid_coarse_cache_rounding() {
        for kind in 0..4 {
            let coarse_precision = if kind < 2 { -2 } else { -4 };
            let divisor = if kind < 2 { 64 } else { 16 };
            for numerator in [
                -513_i64, -512, -511, -193, -192, -191, 191, 192, 193, 511, 512, 513,
            ] {
                let rational = Rational::fraction(numerator, 256).unwrap();
                let exact = Q::from((numerator, 256));
                let (lower, upper) = enclosure(kind, numerator, 256);
                for rounding in -1..=1 {
                    let sample = numerator.div_euclid(divisor) + rounding;
                    if (Q::from(sample) - exact.clone() * (256 / divisor)).abs() > 1 {
                        continue;
                    }
                    for shape in 0..4 {
                        let argument = match shape {
                            0 => Approximation::Ratio(rational.clone()),
                            1 => Approximation::Offset(Computable::rational(rational.clone()), 0),
                            2 => Approximation::Negate(Computable::rational(-rational.clone())),
                            _ => Approximation::Add(
                                Computable::rational(rational.clone()),
                                Computable::zero(),
                            ),
                        };
                        let child = Computable {
                            internal: Arc::new(Node::new(
                                argument,
                                BoundCache::Valid(BoundInfo::Unknown),
                                ExactSignCache::Unknown,
                            )),
                            signal: None,
                        };
                        child
                            .internal
                            .store_cache_value(coarse_precision, BigInt::from(sample));
                        let value = raw(kind, child, rational.clone());
                        for precision in [0, -32, -128, -8] {
                            let result = value.approx(precision).unwrap();
                            assert!(
                                enclosed(&result, precision, &lower, &upper),
                                "kind={kind}, numerator={numerator}, sample={sample}, shape={shape}, p={precision}"
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn oversized_trig_scales_retain_facts_and_global_coarse_values() {
        for base in [Computable::pi(), Computable::half()] {
            for shift in [
                i32::MIN,
                i32::MAX,
                -Computable::MAX_APPROXIMATION_BITS - 1,
                Computable::MAX_APPROXIMATION_BITS + 1,
            ] {
                let input = fresh(Approximation::Offset(base.clone(), shift));
                let before = input.structural_facts();
                assert_eq!(before.sign, Some(RealSign::Positive));
                assert!(input.pi_acos_linear_form(32).is_none());
                assert_eq!(input.structural_facts(), before);
                assert!(input.internal.cache_snapshot().is_none());
                for (kind, value) in [input.clone().sin(), input.clone().cos()]
                    .into_iter()
                    .enumerate()
                {
                    for precision in [1, i32::MAX] {
                        let sample = value.approx(precision).unwrap();
                        if kind == 1
                            && shift > 0
                            && base.shared_constant_kind() == Some(SharedConstant::Pi)
                        {
                            // cos(2^n*pi) = 1 for n > 0. An exact constructor
                            // may choose either valid coarse center, 0 or 1.
                            assert!(sample == BigInt::zero() || sample == BigInt::one());
                        } else {
                            assert_eq!(sample, BigInt::zero());
                        }
                    }
                }
                assert_eq!(input.structural_facts().sign, Some(RealSign::Positive));
            }
        }
        let invalid = fresh(Approximation::Offset(
            fresh(Approximation::Inverse(Computable::zero())),
            i32::MIN,
        ));
        for value in [invalid.clone().sin(), invalid.cos()] {
            assert_eq!(value.approx(1), Err(Problem::DivideByZero));
        }
    }

    #[test]
    fn trig_extreme_offset_reduction_stays_typed() {
        for (base, shift) in [(Computable::pi(), i32::MAX), (Computable::half(), i32::MIN)] {
            for cosine in [false, true] {
                eprintln!("trig extreme offset: shift={shift}, cosine={cosine}");
                let value = fresh(Approximation::Offset(base.clone(), shift));
                let result = if cosine {
                    value.try_reduced_cos()
                } else {
                    value.try_reduced_sin()
                };
                assert!(
                    result.is_ok()
                        || matches!(result, Err(Problem::OutOfRange | Problem::Exhausted))
                );
            }
        }
    }
}
