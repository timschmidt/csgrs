#[cfg(test)]
mod trig_lattice_tests {
    use super::*;
    use rug::{Float, Integer, Rational as Q, float::Round};

    fn raw(kind: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(
                kind,
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    fn offset(child: Computable, shift: i32) -> Computable {
        raw(Approximation::Offset(child, shift))
    }

    fn add(a: Computable, b: Computable) -> Computable {
        raw(Approximation::Add(a, b))
    }

    fn multiply(a: Computable, b: Computable) -> Computable {
        raw(Approximation::Multiply(a, b))
    }

    fn fixtures() -> Vec<(Computable, u8)> {
        let mut result = Vec::new();
        for shift in [0, 1, 4096, 1048576, 1048577, i32::MAX] {
            result.push((
                offset(Computable::pi(), shift),
                if shift == 0 { 2 } else { 0 },
            ));
        }
        for shift in [4096, 1048577, i32::MAX] {
            result.push((offset(offset(Computable::pi(), shift), -shift), 2));
            result.push((
                add(
                    offset(Computable::pi(), shift),
                    offset(Computable::pi(), -1),
                ),
                1,
            ));
            result.push((
                add(
                    multiply(Computable::pi(), offset(Computable::one(), shift)),
                    offset(Computable::pi(), -1),
                ),
                1,
            ));
        }
        for shift in [4096, 1048577, i32::MAX - 1] {
            let third = multiply(
                Computable::pi(),
                Computable::rational(Rational::fraction(1, 3).unwrap()),
            );
            result.push((
                add(offset(third.clone(), shift), offset(third, shift + 1)),
                0,
            ));
        }
        result.push((offset(offset(Computable::pi(), i32::MAX), i32::MIN), 1));
        for shift in [4096, 1048577] {
            let mut value = Computable::pi();
            for _ in 0..32 {
                value = offset(offset(value, shift), -shift);
            }
            result.push((value, 2));
        }
        assert_eq!(result.len(), 21);
        result
    }

    fn trig(argument: Computable, kind: usize, public: bool) -> Computable {
        if public {
            match kind {
                0 => argument.sin(),
                1 => argument.cos(),
                2 => argument.tan(),
                _ => argument.clone().cos().multiply(argument.sin().inverse()),
            }
        } else {
            raw(match kind {
                0 => Approximation::PrescaledSin(argument),
                1 => Approximation::PrescaledCos(argument),
                2 => Approximation::PrescaledTan(argument),
                _ => Approximation::PrescaledCot(argument),
            })
        }
    }

    fn expected(kind: usize, index: u8) -> Result<i8, Problem> {
        let sine = [0, 1, 0, -1][usize::from(index)];
        let cosine = [1, 0, -1, 0][usize::from(index)];
        match kind {
            0 => Ok(sine),
            1 => Ok(cosine),
            2 if cosine == 0 => Err(Problem::DivideByZero),
            3 if sine == 0 => Err(Problem::DivideByZero),
            _ => Ok(0),
        }
    }

    fn check(value: &Computable, wanted: Result<i8, Problem>, precision: i32) {
        match wanted {
            Err(error) => assert_eq!(value.approx(precision), Err(error)),
            Ok(integer) => {
                let actual = value.approx(precision).unwrap();
                if precision >= 1 {
                    // Multiple coarse samples satisfy the same one-unit
                    // contract; exact constructors need not choose zero.
                    let (lower, upper) = match integer.cmp(&0) {
                        Ordering::Less => (-1, 0),
                        Ordering::Equal => (-1, 1),
                        Ordering::Greater => (0, 1),
                    };
                    assert!(actual >= BigInt::from(lower) && actual <= BigInt::from(upper));
                } else {
                    let exact = BigInt::from(integer) << precision.unsigned_abs();
                    assert!((actual - exact).magnitude() <= &BigUint::one());
                }
                if let Some(sign) = value.structural_facts().sign {
                    assert_eq!(sign, public_sign(BigInt::from(integer).sign()));
                }
            }
        }
    }

    #[test]
    fn compressed_lattice_values_poles_and_shared_proofs() {
        for (input, index) in fixtures() {
            for negative in [false, true] {
                let (input, index) = if negative {
                    (raw(Approximation::Negate(input.clone())), (4 - index) % 4)
                } else {
                    (input.clone(), index)
                };
                assert_eq!(input.exact_half_pi_index(), Some(index));
                assert!(input.internal.cache_snapshot().is_none());
                for kind in 0..4 {
                    let value = trig(input.clone(), kind, false);
                    let mut views = vec![value.clone(), value, trig(input.clone(), kind, true)];
                    #[cfg(feature = "serde")]
                    {
                        let mut bytes = Vec::new();
                        ciborium::ser::into_writer(&views[0], &mut bytes).unwrap();
                        views.push(ciborium::de::from_reader(bytes.as_slice()).unwrap());
                    }
                    for value in views.drain(..) {
                        for precision in [1, 0, -8, -128, -8, i32::MAX] {
                            check(&value, expected(kind, index), precision);
                        }
                        if let Ok(integer) = expected(kind, index) {
                            assert_eq!(
                                value.structural_facts().sign,
                                Some(public_sign(BigInt::from(integer).sign()))
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn compressed_lattice_matches_independent_small_integer_modular_oracle() {
        for numerator in -12_i64..=12 {
            for denominator in 1..=13_u64 {
                for shift in -6_i32..=6 {
                    let mut n = i128::from(numerator) * 2;
                    let mut d = i128::from(denominator);
                    if shift >= 0 {
                        n <<= shift;
                    } else {
                        d <<= -shift;
                    }
                    let expected = (n % d == 0).then(|| (n / d).rem_euclid(4) as u8);
                    let q =
                        Computable::rational(Rational::fraction(numerator, denominator).unwrap());
                    for argument in [
                        offset(multiply(Computable::pi(), q.clone()), shift),
                        multiply(Computable::pi(), offset(q.clone(), shift)),
                        raw(Approximation::Negate(offset(
                            multiply(Computable::pi(), raw(Approximation::Negate(q))),
                            shift,
                        ))),
                    ] {
                        assert_eq!(
                            argument.exact_half_pi_index(),
                            expected,
                            "n={numerator}, d={denominator}, shift={shift}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn compressed_lattice_carries_cancel_and_invalid_operands_stay_unknown() {
        for shift in [i32::MIN, -1048577, -8, 0, 8, 1048577, i32::MAX] {
            let third = multiply(
                Computable::pi(),
                Computable::rational(Rational::fraction(1, 3).unwrap()),
            );
            let sum = add(
                add(offset(third.clone(), shift), offset(third.clone(), shift)),
                offset(third, shift),
            );
            let difference = add(
                sum,
                raw(Approximation::Negate(offset(Computable::pi(), shift))),
            );
            assert_eq!(difference.exact_half_pi_index(), Some(0));
            for kind in 0..4 {
                check(
                    &trig(difference.clone(), kind, false),
                    expected(kind, 0),
                    -64,
                );
            }
        }
        let invalid = raw(Approximation::Inverse(Computable::zero()));
        for argument in [
            multiply(Computable::zero(), invalid.clone()),
            add(invalid.clone(), raw(Approximation::Negate(invalid.clone()))),
            offset(multiply(Computable::zero(), invalid), i32::MAX),
        ] {
            assert_eq!(argument.exact_half_pi_index(), None);
            for kind in 0..4 {
                for public in [false, true] {
                    assert_eq!(
                        trig(argument.clone(), kind, public).approx(-8),
                        Err(Problem::DivideByZero)
                    );
                }
            }
        }
        assert_eq!(
            offset(Computable::pi(), i32::MIN).exact_half_pi_index(),
            None
        );
        assert_eq!(
            multiply(Computable::pi(), Computable::pi()).exact_half_pi_index(),
            None
        );
        let mut deep = Computable::pi();
        for _ in 0..129 {
            deep = offset(deep, 0);
        }
        assert_eq!(deep.exact_half_pi_index(), None);
    }

    #[test]
    fn compressed_lattice_keeps_observed_stops_and_numeric_publication_separate() {
        for kind in 0..4 {
            for index in [0, 1, 2, 3] {
                let mut completed = false;
                for stop_after in 0..512 {
                    let input = add(
                        offset(Computable::pi(), i32::MAX),
                        multiply(
                            Computable::pi(),
                            Computable::rational(Rational::fraction(index, 2).unwrap()),
                        ),
                    );
                    let value = trig(input, kind, false);
                    let signal = Arc::new(std::sync::atomic::AtomicBool::new(false));
                    CLEAR_STOP_AFTER_POLLS.with(|polls| polls.set(Some(stop_after)));
                    let result = value.approx_signal(&Some(signal), -64);
                    let observed = CLEAR_STOP_AFTER_POLLS.with(|polls| polls.take().is_none());
                    if !observed {
                        assert_eq!(
                            result.as_ref().err(),
                            expected(kind, index as u8).as_ref().err()
                        );
                        check(&value, expected(kind, index as u8), -64);
                        completed = true;
                        break;
                    }
                    assert_eq!(
                        result,
                        Err(Problem::Exhausted),
                        "kind={kind}, index={index}, poll={stop_after}"
                    );
                    assert!(value.internal.cache_snapshot().is_none());
                    check(&value, expected(kind, index as u8), -64);
                }
                assert!(completed);
            }
        }
    }

    #[test]
    fn nearby_finite_trig_values_are_not_mistaken_for_lattice_points() {
        for shift in [1, 8, 16] {
            for numerator in [-1, 1] {
                let q = Q::from((numerator, 1024));
                let mut sin_lo = Float::with_val(1024, &q);
                let mut sin_hi = sin_lo.clone();
                let mut cos_lo = sin_lo.clone();
                let mut cos_hi = sin_lo.clone();
                sin_lo.sin_round(Round::Down);
                sin_hi.sin_round(Round::Up);
                cos_lo.cos_round(Round::Down);
                cos_hi.cos_round(Round::Up);
                let sin = (sin_lo.to_rational().unwrap(), sin_hi.to_rational().unwrap());
                let cos = (cos_lo.to_rational().unwrap(), cos_hi.to_rational().unwrap());
                let input = add(
                    offset(Computable::pi(), shift),
                    Computable::rational(Rational::fraction(numerator, 1024).unwrap()),
                );
                assert_eq!(input.exact_half_pi_index(), None);
                for kind in 0..4 {
                    let (lo, hi) = match kind {
                        0 => sin.clone(),
                        1 => cos.clone(),
                        _ => {
                            let (a, b) = if kind == 2 {
                                (&sin, &cos)
                            } else {
                                (&cos, &sin)
                            };
                            let endpoints =
                                [&a.0 / &b.0, &a.0 / &b.1, &a.1 / &b.0, &a.1 / &b.1].map(Q::from);
                            (
                                endpoints.iter().min().unwrap().clone(),
                                endpoints.iter().max().unwrap().clone(),
                            )
                        }
                    };
                    for public in [false, true] {
                        let value = trig(input.clone(), kind, public);
                        for precision in [-8_i32, -64, -128, -8] {
                            let sample = value.approx(precision).unwrap();
                            let unit = Q::from((1, Integer::from(1) << -precision));
                            let center =
                                Q::from(Integer::from_str_radix(&sample.to_string(), 10).unwrap())
                                    * &unit;
                            assert!(
                                Q::from(&center - &unit) <= lo && Q::from(&center + &unit) >= hi
                            );
                        }
                    }
                }
            }
        }
    }
}
