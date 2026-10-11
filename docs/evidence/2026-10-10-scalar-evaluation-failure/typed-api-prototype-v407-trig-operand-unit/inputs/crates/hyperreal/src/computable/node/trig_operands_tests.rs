#[cfg(test)]
mod trig_operands_tests {
    use super::*;
    use rug::{Float, Integer, Rational as Q, float::Round};

    fn raw(kind: Approximation) -> Computable {
        Computable::raw_periodic_operand(kind)
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

    fn negative(value: Computable) -> Computable {
        raw(Approximation::Negate(value))
    }

    fn rational(n: i64, d: u64) -> Computable {
        Computable::rational(Rational::fraction(n, d).unwrap())
    }

    fn residual(name: &str) -> Computable {
        let two = rational(2, 1);
        let sqrt2 = raw(Approximation::Sqrt(two.clone()));
        match name {
            "sqrt2" => sqrt2,
            "ln2" => raw(Approximation::LnDeferred(two)),
            "exp1" => raw(Approximation::PrescaledExp(Computable::one())),
            "exp100" => raw(Approximation::PrescaledExp(rational(100, 1))),
            "atan_half" => raw(Approximation::PrescaledAtan(rational(1, 2))),
            "sqrt2_minus_one" => add(sqrt2, rational(-1, 1)),
            "large_sqrt2" => offset(sqrt2, 10),
            "tiny_sqrt2" => offset(sqrt2, -10),
            "negative_sqrt2" => negative(sqrt2),
            _ => panic!("unknown residual"),
        }
    }

    fn residual_enclosure(name: &str) -> (Q, Q) {
        let input = match name {
            "exp1" => Q::from(1),
            "exp100" => Q::from(100),
            "atan_half" => Q::from((1, 2)),
            _ => Q::from(2),
        };
        let mut lo = Float::with_val(1024, &input);
        assert_eq!(lo.to_rational().unwrap(), input);
        let mut hi = lo.clone();
        match name {
            "ln2" => {
                lo.ln_round(Round::Down);
                hi.ln_round(Round::Up);
            }
            "exp1" | "exp100" => {
                lo.exp_round(Round::Down);
                hi.exp_round(Round::Up);
            }
            "atan_half" => {
                lo.atan_round(Round::Down);
                hi.atan_round(Round::Up);
            }
            _ => {
                lo.sqrt_round(Round::Down);
                hi.sqrt_round(Round::Up);
            }
        }
        let (lo, hi) = (lo.to_rational().unwrap(), hi.to_rational().unwrap());
        match name {
            "sqrt2_minus_one" => (lo - 1, hi - 1),
            "large_sqrt2" => (lo * 1024, hi * 1024),
            "tiny_sqrt2" => (lo / 1024, hi / 1024),
            "negative_sqrt2" => (-hi, -lo),
            _ => (lo, hi),
        }
    }

    fn fixture(shape: &str, shift: i32, name: &str) -> (Computable, Q, (Q, Q)) {
        let pi = Computable::pi();
        let third = rational(1, 3);
        let pi_third = multiply(pi.clone(), third.clone());
        // Independently, 2^n modulo six is four for positive even n, two
        // for odd n. No candidate coefficient extractor supplies the oracle.
        let q = Q::from((if shift % 2 == 0 { 4 } else { 2 }, 3));
        let (periodic, q) = match shape {
            "offset_pi" => (offset(pi, shift), Q::from(0)),
            "offset_third" => (offset(pi_third, shift), q),
            "factored_third" => (multiply(pi, offset(third, shift)), q),
            "split_third" => (
                multiply(pi, add(offset(third.clone(), shift), offset(third, shift + 1))),
                Q::from(0),
            ),
            "shifted_plus_half" => (
                add(offset(pi.clone(), shift), offset(pi, -1)), Q::from((1, 2)),
            ),
            "sixth_via_extremes" => (
                offset(offset(pi_third, i32::MAX), i32::MIN), Q::from((1, 6)),
            ),
            "deep_inverse_offsets" => {
                let mut value = pi_third;
                for _ in 0..32 { value = offset(offset(value, shift), -shift); }
                (value, Q::from((1, 3)))
            }
            "ordinary_third" => (pi_third, Q::from((1, 3))),
            _ => panic!("unknown shape"),
        };
        (add(periodic, residual(name)), q, residual_enclosure(name))
    }

    fn fixtures() -> Vec<(Computable, Q, (Q, Q))> {
        let mut values = Vec::new();
        for shift in [4096, 1048577, i32::MAX] {
            for name in ["sqrt2", "ln2", "tiny_sqrt2", "large_sqrt2"] {
                for shape in ["offset_pi", "offset_third", "shifted_plus_half"] {
                    values.push(fixture(shape, shift, name));
                }
            }
        }
        for name in ["exp1", "atan_half", "sqrt2_minus_one", "negative_sqrt2", "exp100"] {
            values.push(fixture("offset_third", i32::MAX, name));
        }
        for name in ["sqrt2", "large_sqrt2"] {
            values.push(fixture("factored_third", i32::MAX, name));
            values.push(fixture("split_third", i32::MAX - 1, name));
        }
        for shape in ["sixth_via_extremes", "deep_inverse_offsets", "ordinary_third"] {
            values.push(fixture(shape, if shape == "deep_inverse_offsets" { 1048577 } else { 0 }, "sqrt2"));
        }
        assert_eq!(values.len(), 48);
        values
    }

    fn trig(value: Computable, kind: usize, public: bool) -> Computable {
        if public {
            match kind {
                0 => value.sin(),
                1 => value.cos(),
                2 => value.tan(),
                _ => value.clone().cos().multiply(value.sin().inverse()),
            }
        } else {
            raw(match kind {
                0 => Approximation::PrescaledSin(value),
                1 => Approximation::PrescaledCos(value),
                2 => Approximation::PrescaledTan(value),
                _ => Approximation::PrescaledCot(value),
            })
        }
    }

    fn enclosure(kind: usize, q: &Q, residual: &(Q, Q)) -> (Q, Q) {
        let (pi_lower, _) = Float::with_val_round(1024, rug::float::Constant::Pi, Round::Down);
        let (pi_upper, _) = Float::with_val_round(1024, rug::float::Constant::Pi, Round::Up);
        let mut lower = pi_lower.to_rational().unwrap() * q;
        let mut upper = pi_upper.to_rational().unwrap() * q;
        if q < &0 { std::mem::swap(&mut lower, &mut upper); }
        lower += &residual.0;
        upper += &residual.1;
        let (base, _) = Float::with_val_round(1024, &lower, Round::Down);
        let radius = upper - base.to_rational().unwrap();
        assert!(radius >= 0);
        // Derivative magnitude <=1 encloses both functions across the entire
        // argument interval, including intervals that cross an extremum.
        let bounds = |cosine: bool| {
            let mut lo = base.clone();
            let mut hi = base.clone();
            if cosine {
                lo.cos_round(Round::Down); hi.cos_round(Round::Up);
            } else {
                lo.sin_round(Round::Down); hi.sin_round(Round::Up);
            }
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

    fn check(value: &Computable, oracle: &(Q, Q), p: i32) {
        let before = value.structural_facts().sign;
        let sample = value.approx(p).unwrap();
        let unit = Q::from((1, Integer::from(1) << -p));
        let center = Q::from(Integer::from_str_radix(&sample.to_string(), 10).unwrap()) * &unit;
        assert!(Q::from(&center - &unit) <= oracle.0 && Q::from(&center + &unit) >= oracle.1,
            "p={p}, sample={sample}, oracle=[{}, {}]", oracle.0.to_f64(), oracle.1.to_f64());
        let after = value.structural_facts().sign;
        if let Some(before) = before { assert_eq!(after, Some(before)); }
        if let Some(after) = after {
            assert!(oracle.0 > 0 || oracle.1 < 0);
            assert_eq!(after, if oracle.0 > 0 { RealSign::Positive } else { RealSign::Negative });
        }
    }

    #[test]
    fn symbolic_pi_operands_match_independent_enclosures() {
        for (input, q, residual) in fixtures() {
            assert!(input.pi_operand_reduction().is_some());
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
                    for p in [0, -8, -256, -8] { check(&value, &oracle, p); }
                }
            }
        }
    }

    #[test]
    fn symbolic_pi_operands_preserve_nonlinear_atoms_and_wrappers() {
        let (sqrt_lo, sqrt_hi) = residual_enclosure("sqrt2");
        let (pi_lo, _) = Float::with_val_round(1024, rug::float::Constant::Pi, Round::Down);
        let (pi_hi, _) = Float::with_val_round(1024, rug::float::Constant::Pi, Round::Up);
        for (atom, bounds) in [
            (multiply(Computable::pi(), residual("sqrt2")),
                (sqrt_lo.clone() * pi_lo.to_rational().unwrap(), sqrt_hi.clone() * pi_hi.to_rational().unwrap())),
            (multiply(residual("sqrt2"), residual("sqrt2")), (Q::from(2), Q::from(2))),
            (multiply(add(Computable::pi(), residual("sqrt2")), rational(3, 1)),
                ((sqrt_lo.clone() + pi_lo.to_rational().unwrap()) * 3, (sqrt_hi.clone() + pi_hi.to_rational().unwrap()) * 3)),
        ] {
            let periodic = offset(multiply(Computable::pi(), rational(1, 3)), i32::MAX);
            let angle = add(periodic, atom);
            for kind in 0..4 {
                let oracle = enclosure(kind, &Q::from((2, 3)), &bounds);
                let value = trig(angle.clone(), kind, false);
                for (view, oracle) in [
                    (value.clone(), oracle.clone()),
                    (negative(value.clone()), (-oracle.1.clone(), -oracle.0.clone())),
                    (offset(value.clone(), 8), (oracle.0.clone() * 256, oracle.1.clone() * 256)),
                    (add(value, rational(1, 3)), (oracle.0 + Q::from((1, 3)), oracle.1 + Q::from((1, 3)))),
                ] {
                    for p in [-8, -128, -8] { check(&view, &oracle, p); }
                }
            }
        }
    }

    #[test]
    fn symbolic_pi_operands_cancellation_and_poles_keep_domains() {
        for name in ["sqrt2", "ln2", "exp1"] {
            let atom = residual(name);
            for cancellation in [add(atom.clone(), negative(atom.clone())), multiply(Computable::zero(), atom)] {
                for index in 0..4_u8 {
                    let periodic = add(offset(Computable::pi(), i32::MAX), multiply(Computable::pi(), rational(i64::from(index), 2)));
                    let angle = add(periodic, cancellation.clone());
                    assert_eq!(angle.half_pi_index_when_real(), Some(index));
                    for kind in 0..4 {
                        let sine = [0_i64, 1, 0, -1][usize::from(index)];
                        let cosine = [1_i64, 0, -1, 0][usize::from(index)];
                        let expected = match kind {
                            0 => Ok(sine), 1 => Ok(cosine),
                            2 if cosine == 0 => Err(Problem::DivideByZero),
                            3 if sine == 0 => Err(Problem::DivideByZero),
                            _ => Ok(0),
                        };
                        for public in [false, true] {
                            let value = trig(angle.clone(), kind, public);
                            for p in [0, -64, -8] {
                                assert_eq!(value.approx(p), expected.map(|n| BigInt::from(n) << -p));
                            }
                        }
                    }
                }
            }
        }
        for (invalid, error) in [
            (raw(Approximation::Inverse(Computable::zero())), Problem::DivideByZero),
            (raw(Approximation::Sqrt(rational(-1, 1))), Problem::SqrtNegative),
            (raw(Approximation::LnDeferred(rational(-1, 1))), Problem::NotANumber),
        ] {
            for hidden in [invalid.clone(), add(invalid.clone(), negative(invalid.clone())), multiply(Computable::zero(), invalid)] {
                let input = add(offset(Computable::pi(), i32::MAX), hidden);
                for kind in 0..4 {
                    for public in [false, true] {
                        let value = trig(input.clone(), kind, public);
                        for p in [1, -64, 0] { assert_eq!(value.approx(p), Err(error)); }
                        assert!(value.internal.cache_snapshot().is_none());
                    }
                }
            }
        }
    }

    #[test]
    fn symbolic_pi_operands_keep_existing_algebraic_certificates() {
        let atan = raw(Approximation::PrescaledAtan(rational(3, 4)));
        let angle = add(Computable::pi(), atan);
        assert_eq!(angle.clone().sin().exact_rational(), Some(Rational::fraction(-3, 5).unwrap()));
        assert_eq!(angle.cos().exact_rational(), Some(Rational::fraction(-4, 5).unwrap()));
        let angle = add(rational(1, 3).acos(), negative(offset(Computable::pi(), -1)));
        assert_eq!(angle.sin().exact_rational(), Some(Rational::fraction(-1, 3).unwrap()));

        for (factor, q) in [
            (raw(Approximation::Inverse(rational(3, 1))), Q::from((2, 3))),
            (raw(Approximation::Sqrt(rational(4, 1))), Q::from(0)),
        ] {
            let angle = add(offset(multiply(Computable::pi(), factor), i32::MAX), residual("sqrt2"));
            assert!(angle.pi_operand_reduction().is_some());
            for kind in 0..4 {
                let oracle = enclosure(kind, &q, &residual_enclosure("sqrt2"));
                for public in [false, true] {
                    let value = trig(angle.clone(), kind, public);
                    for p in [0, -128, -8] { check(&value, &oracle, p); }
                }
            }
        }
    }

    #[test]
    fn symbolic_pi_operands_limits_decline_without_false_periods() {
        let mut deep = add(Computable::pi(), residual("sqrt2"));
        for _ in 0..129 { deep = offset(deep, 0); }
        let mut wide = Computable::pi();
        for n in 2..20 { wide = add(wide, raw(Approximation::LnDeferred(rational(n, 1)))); }
        for input in [
            residual("sqrt2"),
            multiply(Computable::pi(), residual("sqrt2")),
            add(offset(Computable::pi(), i32::MIN), residual("sqrt2")),
            add(multiply(Computable::pi(), Computable::integer(BigInt::one() << 4096)), residual("sqrt2")),
            deep, wide,
        ] { assert!(input.pi_operand_reduction().is_none()); }
    }

    #[test]
    fn symbolic_pi_operands_observed_stops_preserve_retry_and_facts() {
        for (shape, name) in [
            ("shifted_plus_half", "tiny_sqrt2"),
            ("offset_third", "ln2"),
            ("offset_third", "large_sqrt2"),
        ] {
            for kind in 0..4 {
                let (_, q, residual) = fixture(shape, i32::MAX, name);
                let oracle = enclosure(kind, &q, &residual);
                let mut completed = false;
                for stop_after in 0..4096 {
                    let (input, _, _) = fixture(shape, i32::MAX, name);
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
                    check(&value, &oracle, -8);
                }
                assert!(completed);
            }
        }
    }
}
