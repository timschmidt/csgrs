fn inverse(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Plan reciprocal precision from planning facts when available, otherwise fall
    // back to iterative probing. This keeps exact zero short-circuited and avoids
    // a full iterative MSD pass for structural operands that already expose a
    // useful magnitude envelope.
    let (sign, planned_msd) = c.planning_sign_and_msd();
    if sign == Some(Sign::NoSign) {
        return Zero::zero();
    }
    let msd = planned_msd.flatten().unwrap_or_else(|| c.iter_msd());
    let inv_msd = 1 - msd;
    let digits_needed = inv_msd - p + 3;
    let mut prec_needed = msd - digits_needed;
    let mut log_scale_factor = -p - prec_needed;

    let scaled_divisor = loop {
        if log_scale_factor < 0 {
            return Zero::zero();
        }

        let scaled = c.approx_signal(signal, prec_needed);
        if !scaled.is_zero() {
            break scaled;
        }

        if should_stop(signal) {
            return Zero::zero();
        }

        // `iter_msd` is deliberately cheap and may overestimate a value whose
        // leading bits come from cancellation, such as a nested sqrt/log
        // reduction. Refine instead of dividing by a rounded-zero denominator.
        prec_needed -= 8;
        log_scale_factor += 8;
        if log_scale_factor > 16_384 {
            panic!("ArithmeticException");
        }
    };

    let dividend = signed::ONE.deref() << log_scale_factor;
    let abs_scaled_divisor = scaled_divisor.abs();
    let adj_dividend = dividend + (&abs_scaled_divisor >> 1);
    let result: BigInt = adj_dividend / abs_scaled_divisor;

    if scaled_divisor.sign() == Sign::Minus {
        -result
    } else {
        result
    }
}

fn add(signal: &Option<Signal>, c1: &Computable, c2: &Computable, p: Precision) -> BigInt {
    // Addition first tries to prove one operand too small to affect the result
    // at precision p. That dominates deep structural sums and avoids touching
    // tiny terms when signs/MSDs are already known.
    let extra = 4;
    let cutoff = p - extra;
    let (sign1, planning_msd1) = c1.planning_sign_and_msd();
    let (sign2, planning_msd2) = c2.planning_sign_and_msd();
    if sign1 == Some(Sign::NoSign) {
        return c2.approx_signal(signal, p);
    }
    if sign2 == Some(Sign::NoSign) {
        return c1.approx_signal(signal, p);
    }
    let msd1 = planning_msd1.unwrap_or_else(|| c1.msd(cutoff));
    let msd2 = planning_msd2.unwrap_or_else(|| c2.msd(cutoff));

    match (msd1, msd2) {
        (None, None) => return Zero::zero(),
        (None, Some(_)) if sign2.is_some_and(|sign| sign != Sign::NoSign) => {
            return scale(c2.approx_signal(signal, p - extra), -extra);
        }
        (Some(_), None) if sign1.is_some_and(|sign| sign != Sign::NoSign) => {
            return scale(c1.approx_signal(signal, p - extra), -extra);
        }
        (Some(left), Some(_right))
            if sign1 == sign2
                && sign2.is_some_and(|sign| sign != Sign::NoSign)
                && left < cutoff =>
        {
            return scale(c2.approx_signal(signal, p - extra), -extra);
        }
        (Some(_left), Some(right))
            if sign1 == sign2
                && sign1.is_some_and(|sign| sign != Sign::NoSign)
                && right < cutoff =>
        {
            return scale(c1.approx_signal(signal, p - extra), -extra);
        }
        _ => (),
    }

    scale(
        c1.approx_signal(signal, p - 2) + c2.approx_signal(signal, p - 2),
        -2,
    )
}

fn msd_from_appr(prec: Precision, appr: &BigInt) -> Precision {
    prec + appr.magnitude().bits() as Precision - 1
}

fn multiply_with_known_msd(
    signal: &Option<Signal>,
    known: &Computable,
    known_msd: Precision,
    other: &Computable,
    p: Precision,
) -> BigInt {
    // Evaluate the unknown-size operand first, then request only the precision
    // actually needed from the known-size operand. This asymmetric planning is
    // cheaper for products of exact scales and expensive transcendental nodes.
    let prec_other = p - known_msd - 3;
    let appr_other = other.approx_signal(signal, prec_other);

    if appr_other.sign() == Sign::NoSign {
        return Zero::zero();
    }

    let msd_other = msd_from_appr(prec_other, &appr_other);
    let prec_known = p - msd_other - 3;
    let appr_known = known.approx_signal(signal, prec_known);

    let scale_digits = prec_known + prec_other - p;
    scale(appr_known * appr_other, scale_digits)
}

fn multiply(signal: &Option<Signal>, c1: &Computable, c2: &Computable, p: Precision) -> BigInt {
    // Prefer the operand with known larger magnitude as the precision anchor.
    // If one side is effectively zero at the planning cutoff, the product is
    // zero at the requested precision without evaluating both sides deeply.
    let half_prec = (p >> 1) - 1;
    let (sign1, msd1) = c1.planning_sign_and_msd();
    let (sign2, msd2) = c2.planning_sign_and_msd();
    if sign1 == Some(Sign::NoSign) || sign2 == Some(Sign::NoSign) {
        return Zero::zero();
    }
    let msd1 = msd1.unwrap_or_else(|| c1.msd(half_prec));
    let msd2 = msd2.unwrap_or_else(|| c2.msd(half_prec));

    match (msd1, msd2) {
        (None, None) => Zero::zero(),
        (Some(msd_op1), None) => multiply_with_known_msd(signal, c1, msd_op1, c2, p),
        (None, Some(msd_op2)) => multiply_with_known_msd(signal, c2, msd_op2, c1, p),
        (Some(msd_op1), Some(msd_op2)) if msd_op2 > msd_op1 => {
            multiply_with_known_msd(signal, c2, msd_op2, c1, p)
        }
        (Some(msd_op1), Some(_msd_op2)) => multiply_with_known_msd(signal, c1, msd_op1, c2, p),
    }
}

fn square(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Square can reuse one approximation of the child. Constructors create this
    // node for repeated powers so multiplication does not duplicate child work.
    let half_prec = (p >> 1) - 1;
    let (sign, msd) = c.planning_sign_and_msd();
    if sign == Some(Sign::NoSign) {
        return Zero::zero();
    }
    let msd = match msd.unwrap_or_else(|| c.msd(half_prec)) {
        None => {
            return Zero::zero();
        }
        Some(msd) => msd,
    };
    let prec = p - msd - 3;

    let appr = c.approx_signal(signal, prec);

    if appr.sign() == Sign::NoSign {
        return Zero::zero();
    }

    let scale_digits = prec + prec - p;
    scale(&appr * &appr, scale_digits)
}

fn ratio(r: &Rational, p: Precision) -> BigInt {
    // Exact rationals approximate by shifting the numerator/denominator ratio
    // directly; dyadic rationals make this path especially cheap.
    if p >= 0 {
        scale(r.shifted_big_integer(0), -p)
    } else {
        r.shifted_big_integer(-p)
    }
}

fn offset(signal: &Option<Signal>, c: &Computable, n: i32, p: Precision) -> BigInt {
    // x * 2^n at precision p is just x at precision p-n. This is why dyadic
    // scales are represented as Offset nodes instead of generic multiplication.
    c.approx_signal(signal, p - n)
}

fn bound_log2(n: i32) -> i32 {
    let abs_n = n.abs();
    let ln2 = 2.0_f64.ln();
    let n_plus_1: f64 = (abs_n + 1).into();
    let ans: f64 = (n_plus_1.ln() / ln2).ceil();
    ans as i32
}

/* Only intended for Computable values < 0.5, others will be pre-scaled
 * in Computable::exp */
