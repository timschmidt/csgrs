// See docs/admission-inverse-legendre.md for the period, clipping and interval
// proofs. Rational endpoints enclose the root; they never replace its node.
/// Construct the positive forward derivative at an exact rational angle and finite valid log complement.
/// With c=exp(log_complement) and Y=cos(at)^2+c*sin(at)^2, return sqrt(Y) for
/// E or its reciprocal for F. The formula is defined at every real angle;
/// using endpoint values to bound a whole bracket additionally requires the
/// principal chart where it is monotone in absolute angle. This helper does not
/// validate that chart, enclose the derivative or add a local resource budget.
fn legendre_derivative(at: &Rational, log_complement: &Computable, second: bool) -> Computable {
    let angle = Computable::rational(at.clone());
    let y = angle
        .clone()
        .cos()
        .square()
        .add(log_complement.clone().exp().multiply(angle.sin().square()));
    let derivative = y.sqrt();
    if second {
        derivative
    } else {
        derivative.inverse()
    }
}

/// Approximate the retained unwrapped inverse of F/E in `2^p` units using rational root enclosures.
/// The payload is `[value, log(1-m)]`; select E when second is true. For its
/// positive complete integral Q, sample value/Q at precision zero to choose n.
/// A completed one-unit error bound puts the local root in `[-pi/2,pi/2]`,
/// without requiring nearest rounding or strict interior selection. Reject an
/// asserted stop before using the phase. Odd n reciprocates the local complement
/// and rescales the target by sqrt(c) for F or 1/sqrt(c) for E; even n uses c.
/// The final amplitude is n*pi/2 plus the selected local root.
///
/// For p>=4, omit only the bounded local root in this approximation and round
/// a phase sample with two guard bits. Otherwise take k from the lower rational
/// pi/2 enclosure at grid p-8, and isolate the root clipped to `[-k,k]` to width
/// `2^(p-4)`. Bound positive derivatives at the bracket's nearest/farthest absolute
/// angles; refine residuals in 32-bit steps until their width is at most the
/// minimum derivative times the current bracket width divided by sixteen.
/// Signed interval division, outward dyadic rounding, clamping and certified
/// residual signs narrow the bracket. For roots beyond k, derivative bounds apply
/// between the midpoint and clipped endpoint, not outside the current bracket.
///
/// The clipping gap is at most `2^(p-7)`. Add the bracket midpoint to the exact
/// phase and sample at p-3 before rounding; these error allowances stay below
/// one output unit when child enclosures complete correctly. No root estimate
/// replaces the original node. Stop paths return zero for the caller to reject;
/// checked precision subtraction may panic, and refinement, rational arithmetic
/// and child evaluation have no cumulative work or heap-byte budget.
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
    let phase = value
        .clone()
        .multiply(complete.clone().inverse())
        .approx_signal(signal, 0);
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
        (
            residual_value.multiply(if second { exponent.negate() } else { exponent }.exp()),
            log_complement.clone().negate(),
        )
    } else {
        (residual_value, log_complement.clone())
    };
    let root_precision = p.checked_sub(4).expect("root precision fits i32");
    let grid = root_precision
        .checked_sub(4)
        .expect("grid precision fits i32");
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
        let lower_abs = if lower.sign() == Sign::Minus {
            -lower.clone()
        } else {
            lower.clone()
        };
        let upper_abs = if upper.sign() == Sign::Minus {
            -upper.clone()
        } else {
            upper.clone()
        };
        let min_abs = if lower.sign() != Sign::Plus && upper.sign() != Sign::Minus {
            Rational::zero()
        } else {
            interval_min(lower_abs.clone(), upper_abs.clone())
        };
        let max_abs = interval_max(lower_abs, upper_abs);
        let near = legendre_derivative(&min_abs, &local_log, second);
        let far = legendre_derivative(&max_abs, &local_log, second);
        let residual = elliptic_legendre_principal(
            Computable::rational(midpoint.clone()),
            local_log.clone(),
            second,
        )
        .add(target.clone().negate());
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
            precision = precision
                .checked_sub(32)
                .expect("refinement precision fits i32");
        };
        if should_stop(signal) {
            return BigInt::zero();
        }
        let quotient_lo = residual[0].clone()
            / if residual[0].sign() == Sign::Minus {
                &minimum
            } else {
                &maximum
            };
        let quotient_hi = residual[1].clone()
            / if residual[1].sign() == Sign::Minus {
                &maximum
            } else {
                &minimum
            };
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
    scale(
        phase_angle
            .add(Computable::rational(midpoint))
            .approx_signal(signal, p - 3),
        -3,
    )
}
