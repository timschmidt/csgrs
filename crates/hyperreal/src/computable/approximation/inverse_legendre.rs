// See docs/admission-inverse-legendre.md for the period, clipping and interval
// proofs. Rational endpoints enclose the root; they never replace its node.
fn legendre_derivative(at: &Rational, log_complement: &Computable, second: bool) -> Computable {
    let angle = Computable::rational(at.clone());
    let y = angle.clone().cos().square().add(
        log_complement.clone().exp().multiply(angle.sin().square()),
    );
    let derivative = y.sqrt();
    if second { derivative } else { derivative.inverse() }
}

fn inverse_legendre(
    signal: &Option<Signal>,
    arguments: &[Computable; 2],
    second: bool,
    p: Precision,
) -> BigInt {
    if should_stop(signal) {
        return BigInt::zero();
    }
    let [value, log_complement] = arguments;
    let complete = Computable::elliptic_legendre_complete(log_complement.clone(), second);
    let phase = value.clone().multiply(complete.clone().inverse()).approx_signal(signal, 0);
    if should_stop(signal) {
        return BigInt::zero();
    }
    let odd = !(&phase & BigInt::one()).is_zero();
    let periods = Computable::rational(Rational::from_bigint(phase));
    let half_pi = Computable::pi().shift_left(-1);
    let phase_angle = periods.clone().multiply(half_pi.clone());
    if p >= 4 {
        return scale(phase_angle.approx_signal(signal, p - 2), -2);
    }
    let residual_value = value.clone().add(periods.multiply(complete).negate());
    let (target, local_log) = if odd {
        let exponent = log_complement.clone().shift_left(-1);
        (residual_value.multiply(if second { exponent.negate() } else { exponent }.exp()),
         log_complement.clone().negate())
    } else {
        (residual_value, log_complement.clone())
    };
    let root_precision = p.checked_sub(4).expect("root precision fits i32");
    let grid = root_precision.checked_sub(4).expect("grid precision fits i32");
    let width = interval_dyadic(BigInt::one(), root_precision);
    let k = interval_of(&half_pi, signal, grid)[0].clone();
    if should_stop(signal) {
        return BigInt::zero();
    }
    let mut lower = -k.clone();
    let mut upper = k.clone();
    let mut precision: Precision = -16;
    while &upper - &lower > width {
        if should_stop(signal) {
            return BigInt::zero();
        }
        let midpoint = (&lower + &upper) * HALF_RATIONAL.deref();
        let lower_abs = if lower.sign() == Sign::Minus { -lower.clone() } else { lower.clone() };
        let upper_abs = if upper.sign() == Sign::Minus { -upper.clone() } else { upper.clone() };
        let min_abs = if lower.sign() != Sign::Plus && upper.sign() != Sign::Minus {
            Rational::zero()
        } else {
            interval_min(lower_abs.clone(), upper_abs.clone())
        };
        let max_abs = interval_max(lower_abs, upper_abs);
        let near = legendre_derivative(&min_abs, &local_log, second);
        let far = legendre_derivative(&max_abs, &local_log, second);
        let residual = elliptic_legendre_principal(
            Computable::rational(midpoint.clone()), local_log.clone(), second,
        ).add(target.clone().negate());
        let (residual, minimum, maximum) = loop {
            if should_stop(signal) {
                return BigInt::zero();
            }
            let [alo, ahi] = interval_of(&near, signal, precision);
            let [blo, bhi] = interval_of(&far, signal, precision);
            let minimum = interval_min(alo, blo);
            let maximum = interval_max(ahi, bhi);
            if minimum.sign() == Sign::Plus {
                let residual = interval_of(&residual, signal, precision);
                if &residual[1] - &residual[0]
                    <= &minimum * (&upper - &lower) * Rational::fraction(1, 16).unwrap()
                {
                    break (residual, minimum, maximum);
                }
            }
            precision = precision.checked_sub(32).expect("refinement precision fits i32");
        };
        if should_stop(signal) {
            return BigInt::zero();
        }
        let quotient_lo = residual[0].clone()
            / if residual[0].sign() == Sign::Minus { &minimum } else { &maximum };
        let quotient_hi = residual[1].clone()
            / if residual[1].sign() == Sign::Minus { &maximum } else { &minimum };
        let next_lower = interval_dyadic(ratio(&(&midpoint - quotient_hi), grid) - 1, grid);
        let next_upper = interval_dyadic(ratio(&(&midpoint - quotient_lo), grid) + 1, grid);
        lower = interval_max(lower, interval_clamp(next_lower, &k));
        upper = interval_min(upper, interval_clamp(next_upper, &k));
        if residual[0].sign() == Sign::Plus {
            upper = interval_min(upper, midpoint.clone());
        } else if residual[1].sign() == Sign::Minus {
            lower = interval_max(lower, midpoint);
        }
        debug_assert!(lower <= upper, "certified root intervals intersect");
    }
    let midpoint = (lower + upper) * HALF_RATIONAL.deref();
    crate::trace_dispatch!("computable", "inverse_legendre", "certified-root-interval");
    scale(phase_angle.add(Computable::rational(midpoint)).approx_signal(signal, p - 3), -3)
}
