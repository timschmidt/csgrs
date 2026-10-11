#[cfg(test)]
mod trig_periodic_tests {
    use super::*;
    use rug::{Float, Integer, Rational as Q, float::Round};

    fn raw(kind: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(kind, BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        }
    }

    fn offset(value: Computable, shift: i32) -> Computable {
        raw(Approximation::Offset(value, shift))
    }

    fn add(left: Computable, right: Computable) -> Computable {
        raw(Approximation::Add(left, right))
    }

    fn multiply(left: Computable, right: Computable) -> Computable {
        raw(Approximation::Multiply(left, right))
    }

    fn rational(n: i64, d: u64) -> Computable {
        Computable::rational(Rational::fraction(n, d).unwrap())
    }

    fn fixture(shape: &str, shift: i32, n: i64, d: u64) -> (Computable, Q, Q) {
        let pi = Computable::pi();
        let third = rational(1, 3);
        let pi_third = multiply(pi.clone(), third.clone());
        // For n>=1, 2^n modulo six alternates 2,4. This finite integer
        // identity supplies the oracle independently of the form extractor.
        let reduced_third = Q::from((if shift % 2 == 0 { 4 } else { 2 }, 3));
        let (periodic, q) = match shape {
            "offset_pi" => (offset(pi, shift), Q::from(0)),
            "offset_third" => (offset(pi_third, shift), reduced_third),
            "negative_third" => (raw(Approximation::Negate(offset(pi_third, shift))), -reduced_third),
            "factored_third" => (multiply(pi, offset(third, shift)), reduced_third),
            "split_third" => (multiply(pi, add(offset(third.clone(), shift), offset(third, shift + 1))), Q::from(0)),
            "shifted_plus_half" => (add(offset(pi.clone(), shift), offset(pi, -1)), Q::from((1, 2))),
            "sixth_via_extremes" => (offset(offset(pi_third, i32::MAX), i32::MIN), Q::from((1, 6))),
            "deep_inverse_offsets" => {
                let mut value = pi_third;
                for _ in 0..32 {
                    value = offset(offset(value, shift), -shift);
                }
                (value, Q::from((1, 3)))
            },
            "ordinary_third" => (pi_third, Q::from((1, 3))),
            _ => panic!("unknown fixture"),
        };
        (add(periodic, rational(n, d)), q, Q::from((n, d)))
    }

    fn fixtures() -> Vec<(Computable, Q, Q)> {
        let mut result = Vec::new();
        for shift in [4096, 1048577, i32::MAX] {
            for shape in ["offset_pi", "offset_third", "shifted_plus_half"] {
                for n in [1, -1] { result.push(fixture(shape, shift, n, 1024)); }
            }
            result.push(fixture("factored_third", shift, 1, 1024));
            result.push(fixture("split_third", shift.min(i32::MAX - 1), -1, 1024));
            result.push(fixture("negative_third", shift, 1, 1024));
            for shape in ["offset_pi", "offset_third"] {
                for n in [1000, -1000] { result.push(fixture(shape, shift, n, 1)); }
            }
            result.push(fixture("offset_third", shift, 0, 1));
        }
        for shape in ["sixth_via_extremes", "deep_inverse_offsets", "ordinary_third"] {
            for (n, d) in [(1, 1024), (-1, 1024), (1000, 1), (-1000, 1), (0, 1)] {
                result.push(fixture(shape, if shape == "deep_inverse_offsets" { 1048577 } else { 0 }, n, d));
            }
        }
        assert_eq!(result.len(), 57);
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

    fn enclosure(kind: usize, q: &Q, residual: &Q) -> (Q, Q) {
        let (pi_lower, _) = Float::with_val_round(1024, rug::float::Constant::Pi, Round::Down);
        let (pi_upper, _) = Float::with_val_round(1024, rug::float::Constant::Pi, Round::Up);
        let mut lower = pi_lower.to_rational().unwrap() * q + residual;
        let mut upper = pi_upper.to_rational().unwrap() * q + residual;
        if q < &0 { std::mem::swap(&mut lower, &mut upper); }
        let (base, _) = Float::with_val_round(1024, &lower, Round::Down);
        let radius = upper - base.to_rational().unwrap();
        assert!(radius >= 0);
        // Both derivatives have magnitude <=1 everywhere. Widen the directed
        // point evaluations by the entire angle interval; monotonicity at an
        // arbitrary endpoint (including near extrema) is not assumed.
        let bounds = |cosine: bool| {
            let mut lo = base.clone();
            let mut hi = base.clone();
            if cosine { lo.cos_round(Round::Down); hi.cos_round(Round::Up); }
            else { lo.sin_round(Round::Down); hi.sin_round(Round::Up); }
            (lo.to_rational().unwrap() - &radius, hi.to_rational().unwrap() + &radius)
        };
        let sine = bounds(false);
        let cosine = bounds(true);
        match kind {
            0 => sine,
            1 => cosine,
            _ => {
                let (a, b) = if kind == 2 { (sine, cosine) } else { (cosine, sine) };
                assert!(b.1 < 0 || b.0 > 0);
                let endpoints = [&a.0 / &b.0, &a.0 / &b.1, &a.1 / &b.0, &a.1 / &b.1].map(Q::from);
                (endpoints.iter().min().unwrap().clone(), endpoints.iter().max().unwrap().clone())
            }
        }
    }

    fn check(value: &Computable, oracle: &(Q, Q), precision: i32) {
        let previous = value.structural_facts().sign;
        let sample = value.approx(precision).unwrap();
        let unit = Q::from((1, Integer::from(1) << -precision));
        let center = Q::from(Integer::from_str_radix(&sample.to_string(), 10).unwrap()) * &unit;
        assert!(Q::from(&center - &unit) <= oracle.0 && Q::from(&center + &unit) >= oracle.1,
            "p={precision}, sample={sample}, center={}, oracle=[{}, {}]", center.to_f64(), oracle.0.to_f64(), oracle.1.to_f64());
        let current = value.structural_facts().sign;
        if let Some(previous) = previous { assert_eq!(current, Some(previous)); }
        if let Some(current) = current {
            assert!(oracle.0 > 0 || oracle.1 < 0);
            assert_eq!(current, if oracle.0 > 0 { RealSign::Positive } else { RealSign::Negative });
        }
    }

    fn exact_rug(value: &Rational) -> Q {
        // Rational's human display may use mixed fractions. Import its exact
        // integer parts instead of treating that display as a machine encoding.
        let mut numerator = Integer::from_str_radix(&value.numerator().to_string(), 10).unwrap();
        if value.sign() == Sign::Minus { numerator = -numerator; }
        Q::from((numerator, Integer::from_str_radix(&value.denominator().to_string(), 10).unwrap()))
    }

    #[test]
    fn symbolic_periodic_residuals_match_independent_mpfr_enclosures() {
        for (input, q, residual) in fixtures() {
            assert_eq!(input.exact_half_pi_index(), None);
            for kind in 0..4 {
                let oracle = enclosure(kind, &q, &residual);
                let value = trig(input.clone(), kind, false);
                let mut views = vec![value.clone(), value, trig(input.clone(), kind, true)];
                #[cfg(feature = "serde")]
                {
                    let mut bytes = Vec::new();
                    ciborium::ser::into_writer(&views[0], &mut bytes).unwrap();
                    views.push(ciborium::de::from_reader(bytes.as_slice()).unwrap());
                }
                for value in views.drain(..) {
                    for precision in [0, -8, -256, -8] { check(&value, &oracle, precision); }
                }
            }
        }
    }

    #[test]
    fn symbolic_periodic_reduction_preserves_exact_small_forms() {
        for n in [-9_i64, -4, -1, 0, 1, 4, 9] {
            for d in 1..=13_u64 {
                for shift in -6_i32..=6 {
                    let input = add(
                        add(offset(multiply(Computable::pi(), rational(n, d)), shift), multiply(Computable::pi(), rational(1, 7))),
                        add(rational(n, 13), rational(-i64::from(shift), 17)),
                    );
                    let mut exact = Q::from((n, d));
                    if shift >= 0 { exact *= Integer::from(1) << shift; }
                    else { exact /= Integer::from(1) << -shift; }
                    exact += Q::from((1, 7));
                    let (quadrant, coefficient, residual) = trig_linear_form(&input, 128, &mut 256).unwrap().reduced_angle().unwrap();
                    let coefficient = exact_rug(&coefficient);
                    let residual = exact_rug(&residual);
                    assert!((0..4).contains(&quadrant));
                    assert!(coefficient >= Q::from((-1, 4)) && coefficient < Q::from((1, 4)));
                    let difference = exact - Q::from((quadrant, 2)) - coefficient;
                    assert_eq!(difference.denom(), &Integer::from(1));
                    assert_eq!(Integer::from(difference.numer() % 2), 0);
                    assert_eq!(residual, Q::from((n, 13)) - Q::from((shift, 17)), "n={n}, d={d}, shift={shift}");
                }
            }
        }
    }

    #[test]
    fn symbolic_periodic_rewrites_terminate_under_wrappers_and_cancellation() {
        for (n, d, r) in [(-953, 3, 1000), (953, 3, -1000), (1, 16, 0), (-1, 4, 0)] {
            let angle = add(offset(offset(multiply(Computable::pi(), rational(n, d)), i32::MAX), -i32::MAX), rational(r, 1));
            for kind in 0..4 {
                let oracle = enclosure(kind, &Q::from((n, d)), &Q::from(r));
                let value = trig(angle.clone(), kind, false);
                let views = [value.clone(), raw(Approximation::Negate(value.clone())), add(value.clone(), rational(1, 3)), offset(value, 8)];
                let oracles = [oracle.clone(), (-oracle.1.clone(), -oracle.0.clone()), (oracle.0.clone()+Q::from((1,3)),oracle.1.clone()+Q::from((1,3))), (oracle.0.clone()*256,oracle.1.clone()*256)];
                for (view, oracle) in views.iter().zip(oracles) {
                    for precision in [-8, -128, -8] { check(view, &oracle, precision); }
                }
            }
        }
        for (n, d) in [(-1, 4), (1, 8), (-1, 4096)] {
            let angle = multiply(Computable::pi(), rational(n, d));
            assert!(angle.try_symbolic_sin_cos(&None).unwrap().is_none());
        }
    }

    #[test]
    fn symbolic_periodic_limits_and_invalid_operands_remain_typed() {
        let mut deep = Computable::pi();
        for _ in 0..129 { deep = offset(deep, 0); }
        for input in [offset(Computable::pi(), i32::MIN), add(Computable::pi(), offset(Computable::one(), i32::MAX)), multiply(Computable::pi(), Computable::pi()), multiply(Computable::pi(), Computable::integer(BigInt::one() << 4096)), deep] {
            assert!(input.try_symbolic_sin_cos(&None).unwrap().is_none());
        }
        for (invalid, error) in [(raw(Approximation::Inverse(Computable::zero())), Problem::DivideByZero), (raw(Approximation::Sqrt(rational(-1,1))), Problem::SqrtNegative), (raw(Approximation::LnDeferred(rational(-1,1))), Problem::NotANumber)] {
            for hidden in [invalid.clone(), multiply(Computable::zero(), invalid)] {
                let input = add(offset(Computable::pi(), i32::MAX), hidden);
                assert!(input.try_symbolic_sin_cos(&None).unwrap().is_none());
                for kind in 0..4 {
                    for public in [false, true] {
                        let value = trig(input.clone(), kind, public);
                        assert_eq!(value.approx(-64), Err(error));
                        assert_eq!(value.approx(1), Err(error));
                        assert!(value.internal.cache_snapshot().is_none());
                    }
                }
            }
        }
    }

    #[test]
    fn symbolic_periodic_observed_stops_do_not_publish_numeric_values() {
        for (shape, n, d) in [("shifted_plus_half", 1, 1024), ("offset_third", 1000, 1)] {
            for kind in 0..4 {
                let (_, q, residual) = fixture(shape, i32::MAX, n, d);
                let oracle = enclosure(kind, &q, &residual);
                let mut completed = false;
                for stop_after in 0..4096 {
                    let (input, _, _) = fixture(shape, i32::MAX, n, d);
                    let value = trig(input, kind, false);
                    let signal = Arc::new(std::sync::atomic::AtomicBool::new(false));
                    CLEAR_STOP_AFTER_POLLS.with(|polls| polls.set(Some(stop_after)));
                    let result = value.approx_signal(&Some(signal), -64);
                    let observed = CLEAR_STOP_AFTER_POLLS.with(|polls| polls.take().is_none());
                    if !observed {
                        assert!(result.is_ok());
                        check(&value, &oracle, -64);
                        completed = true;
                        break;
                    }
                    assert_eq!(result, Err(Problem::Exhausted), "kind={kind}, shape={shape}, poll={stop_after}");
                    assert!(value.internal.cache_snapshot().is_none());
                    check(&value, &oracle, -64);
                }
                assert!(completed);
            }
        }
    }
}
