fn inverse(signal: &Option<Signal>, c: &Computable, p: Precision) -> Result<BigInt, Problem> {
let p = crate::computable::checked_approximation_precision(p)?;

Ok({
    // A raw planning hint may overestimate a near-cancelled denominator by an
    // arbitrary amount. Use an exact magnitude or refine before sizing the
    // reciprocal's numerator and denominator approximations.
    let (sign, planned_msd) = c.certified_sign_and_msd();
    if sign == Some(Sign::NoSign) {
        return Err(Problem::DivideByZero);
    }
    let msd = match planned_msd.flatten() { Some(msd) => msd, None => c.iter_msd_stop_signal(signal, -Computable::MAX_APPROXIMATION_BITS)?.ok_or(Problem::UnknownZero)? };
    let inv_msd = 1_i64 - i64::from(msd);
    let digits_needed = inv_msd - i64::from(p) + 3;
    let prec_needed = i64::from(msd) - digits_needed;
    let log_scale_factor = -i64::from(p) - prec_needed;
    if log_scale_factor < 0 { return Ok(Zero::zero()); }
    let mut prec_needed = precision_from_i64(prec_needed)?;
    let mut log_scale_factor = precision_from_i64(log_scale_factor)?;

    let scaled_divisor = loop {
        if log_scale_factor < 0 {
            return Ok(Zero::zero());
        }

        let scaled = c.approx_signal(signal, prec_needed)?;
        if !scaled.is_zero() {
            break scaled;
        }

        if should_stop(signal) { return Err(Problem::Exhausted); }

        // `iter_msd` is deliberately cheap and may overestimate a value whose
        // leading bits come from cancellation, such as a nested sqrt/log
        // reduction. Refine instead of dividing by a rounded-zero denominator.
        prec_needed = prec_needed.checked_sub(8).ok_or(Problem::OutOfRange)?;
        log_scale_factor = log_scale_factor.checked_add(8).ok_or(Problem::OutOfRange)?;
        if log_scale_factor > 16_384 {
            return Err(Problem::UnknownZero);
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
})
}

fn add(signal: &Option<Signal>, c1: &Computable, c2: &Computable, p: Precision) -> Result<BigInt, Problem> {
let p = crate::computable::checked_approximation_precision(p)?;

Ok({
    // Addition first tries to prove one operand too small to affect the result
    // at precision p. That dominates deep structural sums and avoids touching
    // tiny terms when signs/MSDs are already known.
    let extra = 4;
    let cutoff = p.checked_sub(extra).ok_or(Problem::OutOfRange)?;
    let (sign1, planning_msd1) = c1.certified_sign_and_msd();
    let (sign2, planning_msd2) = c2.certified_sign_and_msd();
    if sign1 == Some(Sign::NoSign) {
        return Ok(c2.approx_signal(signal, p)?);
    }
    if sign2 == Some(Sign::NoSign) {
        return Ok(c1.approx_signal(signal, p)?);
    }
    let msd1 = match planning_msd1.flatten() { Some(msd) => Some(msd), None => c1.msd_signal(signal, cutoff)? };
    let msd2 = match planning_msd2.flatten() { Some(msd) => Some(msd), None => c2.msd_signal(signal, cutoff)? };

    match (msd1, msd2) {
        (None, None) => return Ok(Zero::zero()),
        (None, Some(_)) if sign2.is_some_and(|sign| sign != Sign::NoSign) => {
            return Ok(scale(c2.approx_signal(signal, p - extra)?, -extra));
        }
        (Some(_), None) if sign1.is_some_and(|sign| sign != Sign::NoSign) => {
            return Ok(scale(c1.approx_signal(signal, p - extra)?, -extra));
        }
        (Some(left), Some(_right))
            if sign1 == sign2
                && sign2.is_some_and(|sign| sign != Sign::NoSign)
                && left < cutoff =>
        {
            return Ok(scale(c2.approx_signal(signal, p - extra)?, -extra));
        }
        (Some(_left), Some(right))
            if sign1 == sign2
                && sign1.is_some_and(|sign| sign != Sign::NoSign)
                && right < cutoff =>
        {
            return Ok(scale(c1.approx_signal(signal, p - extra)?, -extra));
        }
        _ => (),
    }

    scale(
        c1.approx_signal(signal, p - 2)? + c2.approx_signal(signal, p - 2)?,
        -2,
    )
})
}

fn msd_from_appr(prec: Precision, appr: &BigInt) -> Result<Precision, Problem> {
    crate::verified::word::checked_bit_length_msd(appr.magnitude().bits(), 1, false, prec)
        .ok_or(Problem::OutOfRange)
}

fn multiply_with_known_msd(
    signal: &Option<Signal>,
    known: &Computable,
    known_msd: Precision,
    other: &Computable,
    p: Precision,
) -> Result<BigInt, Problem> {
let p = crate::computable::checked_approximation_precision(p)?;

Ok({
    // Evaluate the unknown-size operand first, then request only the precision
    // actually needed from the known-size operand. This asymmetric planning is
    // cheaper for products of exact scales and expensive transcendental nodes.
    let prec_other = precision_from_i64(i64::from(p) - i64::from(known_msd) - 3)?;
    let appr_other = other.approx_signal(signal, prec_other)?;

    if appr_other.sign() == Sign::NoSign {
        return Ok(Zero::zero());
    }

    let msd_other = msd_from_appr(prec_other, &appr_other)?;
    let prec_known = precision_from_i64(i64::from(p) - i64::from(msd_other) - 3)?;
    let appr_known = known.approx_signal(signal, prec_known)?;

    let scale_digits = precision_from_i64(i64::from(prec_known) + i64::from(prec_other) - i64::from(p))?;
    scale(appr_known * appr_other, scale_digits)
})
}

fn multiply(signal: &Option<Signal>, c1: &Computable, c2: &Computable, p: Precision) -> Result<BigInt, Problem> {
let p = crate::computable::checked_approximation_precision(p)?;

Ok({
    // Prefer the operand with known larger magnitude as the precision anchor.
    // If one side is effectively zero at the planning cutoff, the product is
    // zero at the requested precision without evaluating both sides deeply.
    let half_prec = (p >> 1) - 1;
    let (sign1, msd1) = c1.certified_sign_and_msd();
    let (sign2, msd2) = c2.certified_sign_and_msd();
    if sign1 == Some(Sign::NoSign) || sign2 == Some(Sign::NoSign) {
        return Ok(Zero::zero());
    }
    let msd1 = match msd1.flatten() { Some(msd) => Some(msd), None => c1.msd_signal(signal, half_prec)? };
    let msd2 = match msd2.flatten() { Some(msd) => Some(msd), None => c2.msd_signal(signal, half_prec)? };

    match (msd1, msd2) {
        (None, None) => Zero::zero(),
        (Some(msd_op1), None) => multiply_with_known_msd(signal, c1, msd_op1, c2, p)?,
        (None, Some(msd_op2)) => multiply_with_known_msd(signal, c2, msd_op2, c1, p)?,
        (Some(msd_op1), Some(msd_op2)) if msd_op2 > msd_op1 => {
            multiply_with_known_msd(signal, c2, msd_op2, c1, p)?
        }
        (Some(msd_op1), Some(_msd_op2)) => multiply_with_known_msd(signal, c1, msd_op1, c2, p)?,
    }
})
}

fn linear_combination3(
    signal: &Option<Signal>,
    form: &LinearCombination3,
    p: Precision,
) -> Result<BigInt, Problem> {
let p = crate::computable::checked_approximation_precision(p)?;

Ok({
    fn weighted(
        signal: &Option<Signal>,
        coefficient: &Computable,
        value: &Rational,
        p: Precision,
    ) -> Result<BigInt, Problem> {
let p = crate::computable::checked_approximation_precision(p)?;

Ok({
        let Some(msd) = value.msd_exact() else {
            return Ok(Zero::zero());
        };
        // Eight coefficient guard bits leave ample room for the rational
        // scale and the truncating integer division below.
        let coefficient_precision = precision_from_i64(i64::from(p) - i64::from(msd) - 8)?;
        let approximation =
            coefficient.approx_signal(signal, coefficient_precision)?;
        let signed_numerator =
            BigInt::from_biguint(value.sign(), value.numerator().clone());
        let mut numerator = approximation * signed_numerator;
        let mut denominator = value.denominator().clone();
        let shift = precision_from_i64(i64::from(coefficient_precision) - i64::from(p))?;
        if shift >= 0 {
            numerator <<= shift as usize;
        } else {
            denominator <<= shift.unsigned_abs() as usize;
        }
        numerator / BigInt::from(denominator)
    })
}

    // Compute each term four bits below the target. Even after the three
    // truncating rational scales, the final division by sixteen keeps the
    // aggregate absolute error below one target unit.
    let guard = 4;
    let precision = p.checked_sub(guard).ok_or(Problem::OutOfRange)?;
    let sum = weighted(
        signal,
        &form.coefficients[0],
        &form.values[0],
        precision,
    )? + weighted(
        signal,
        &form.coefficients[1],
        &form.values[1],
        precision,
    )? + weighted(
        signal,
        &form.coefficients[2],
        &form.values[2],
        precision,
    )?;
    scale(sum, -guard)
})
}

fn square(signal: &Option<Signal>, c: &Computable, p: Precision) -> Result<BigInt, Problem> {
let p = crate::computable::checked_approximation_precision(p)?;

Ok({
    // Square can reuse one approximation of the child. Constructors create this
    // node for repeated powers so multiplication does not duplicate child work.
    let half_prec = (p >> 1) - 1;
    let (sign, _) = c.planning_sign_and_msd();
    if sign == Some(Sign::NoSign) {
        return Ok(Zero::zero());
    }
    // A square's structural MSD estimate can be one binade low, and repeatedly
    // doubling that estimate compounds the error in power chains. Ask for the
    // child's certified/cached MSD here; exact bounds remain allocation-free,
    // while inexact square chains refine before choosing a working precision.
    let msd = match c.msd_signal(signal, half_prec)? {
        None => {
            return Ok(Zero::zero());
        }
        Some(msd) => msd,
    };
    let prec = precision_from_i64(i64::from(p) - i64::from(msd) - 3)?;

    let appr = c.approx_signal(signal, prec)?;

    if appr.sign() == Sign::NoSign {
        return Ok(Zero::zero());
    }

    let scale_digits = precision_from_i64(2 * i64::from(prec) - i64::from(p))?;
    scale(&appr * &appr, scale_digits)
})
}

fn ratio(r: &Rational, p: Precision) -> Result<BigInt, Problem> {
let p = crate::computable::checked_approximation_precision(p)?;

Ok({
    // Exact rationals approximate by shifting the numerator/denominator ratio
    // directly; dyadic rationals make this path especially cheap.
    if p >= 0 {
        scale(r.shifted_big_integer(0), -p)
    } else {
        r.shifted_big_integer(p.checked_neg().ok_or(Problem::OutOfRange)?)
    }
})
}

fn offset(signal: &Option<Signal>, c: &Computable, n: i32, p: Precision) -> Result<BigInt, Problem> {
let p = crate::computable::checked_approximation_precision(p)?;

Ok({
    // x * 2^n at precision p is just x at precision p-n. This is why dyadic
    // scales are represented as Offset nodes instead of generic multiplication.
    c.approx_signal(signal, p.checked_sub(n).ok_or(Problem::OutOfRange)?)?
})
}

fn bound_log2(n: i32) -> i32 {
    // ceil(log2(|n|+1)) equals the bit length of |n|, including zero.
    (u32::BITS - n.unsigned_abs().leading_zeros()) as i32
}

/* Only intended for Computable values < 0.5, others will be pre-scaled
 * in Computable::exp */
