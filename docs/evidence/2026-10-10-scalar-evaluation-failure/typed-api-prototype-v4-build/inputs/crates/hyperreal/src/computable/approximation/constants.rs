// Fractional requests use Chudnovsky; coarse requests use exact known bounds.
// A failed evaluation must never enter a numeric fallback or a shared cache.
const CHUDNOVSKY_C3_OVER_24: u64 = 10_939_058_860_032_000;
const CHUDNOVSKY_GUARD_BITS: u64 = 64;

fn chudnovsky_binary_split(
    signal: &Option<Signal>,
    a: u32,
    b: u32,
) -> Result<(BigInt, BigInt, BigInt), Problem> {
    if should_stop(signal) { return Err(Problem::Exhausted); }
    if b - a == 1 {
        if a == 0 {
            return Ok((BigInt::one(), BigInt::one(), BigInt::from(13_591_409_u32)));
        }
        let k = u64::from(a);
        let p = BigInt::from(6 * k - 5) * BigInt::from(2 * k - 1) * BigInt::from(6 * k - 1);
        let q = BigInt::from(k).pow(3) * BigInt::from(CHUDNOVSKY_C3_OVER_24);
        let mut t = &p * BigInt::from(13_591_409_u64 + 545_140_134_u64 * k);
        if a & 1 == 1 { t = -t; }
        return Ok((p, q, t));
    }
    let mid = a + (b - a) / 2;
    let (left_p, left_q, left_t) = chudnovsky_binary_split(signal, a, mid)?;
    let (right_p, right_q, right_t) = chudnovsky_binary_split(signal, mid, b)?;
    let t = left_t * &right_q + &left_p * right_t;
    Ok((left_p * right_p, left_q * right_q, t))
}

fn pi_chudnovsky(signal: &Option<Signal>, p: Precision) -> Result<BigInt, Problem> {
    if should_stop(signal) { return Err(Problem::Exhausted); }
    let target_bits = u64::try_from(p.checked_neg().ok_or(Problem::OutOfRange)?)
        .map_err(|_| Problem::OutOfRange)?;
    let work_bits = target_bits.checked_add(CHUDNOVSKY_GUARD_BITS).ok_or(Problem::OutOfRange)?;

    // Consecutive absolute Chudnovsky terms have ratio below 2^-43. For k >= 1,
    // bounding the six numerator factors by 6(k+1), the three (3k+j) factors
    // below by 3k, and the linear-factor ratio by 2 proves the bound; the k=0
    // ratio is smaller by direct integer comparison. The alternating tail is
    // therefore below the first omitted term. Two extra terms put it well
    // beneath one work-scale ulp; 64 guard bits also dominate fixed-point sqrt
    // and final-division rounding before one final scale rounding.
    let terms = u32::try_from(work_bits.div_ceil(43).checked_add(2).ok_or(Problem::OutOfRange)?)
        .map_err(|_| Problem::OutOfRange)?;
    let (_, q, t) = chudnovsky_binary_split(signal, 0, terms)?;
    if should_stop(signal) { return Err(Problem::Exhausted); }
    let sqrt_shift = usize::try_from(work_bits.checked_mul(2).ok_or(Problem::OutOfRange)?)
        .map_err(|_| Problem::OutOfRange)?;
    let sqrt_10005 = (BigUint::from(10_005_u32) << sqrt_shift).sqrt();
    let q = q.to_biguint().ok_or(Problem::OutOfRange)?;
    let denominator = t.to_biguint().ok_or(Problem::OutOfRange)?;
    let numerator = q * BigUint::from(426_880_u32) * sqrt_10005;
    let work_scaled = (numerator + (&denominator >> 1_usize)) / denominator;
    let guard = Precision::try_from(CHUDNOVSKY_GUARD_BITS).map_err(|_| Problem::OutOfRange)?;
    Ok(scale(BigInt::from(work_scaled), -guard))
}

fn pi(signal: &Option<Signal>, p: Precision) -> Result<BigInt, Problem> {
    if should_stop(signal) { return Err(Problem::Exhausted); }
    match p {
        0 => Ok(BigInt::from(3)),
        1 => Ok(BigInt::from(2)),
        2.. => Ok(BigInt::zero()), // 0 < pi / 2^p < 1
        _ => pi_chudnovsky(signal, p),
    }
}

fn ln2(signal: &Option<Signal>, p: Precision) -> Result<BigInt, Problem> {
Ok({
    // A fixed atanh/log1p decomposition for ln(2). Keeping ln2 as its own
    // shared constant matters because exp/ln range reduction adds multiples of
    // ln2 frequently. The identity routes each piece through the reduced
    // ln(1+x) kernel using argument reduction followed by a series.
    let prescaled_9 = raw(Approximation::PrescaledLn(Computable::rational(
        Rational::fraction(1, 9).unwrap(),
    )));
    let prescaled_24 = raw(Approximation::PrescaledLn(Computable::rational(
        Rational::fraction(1, 24).unwrap(),
    )));
    let prescaled_80 = raw(Approximation::PrescaledLn(Computable::rational(
        Rational::fraction(1, 80).unwrap(),
    )));

    let ln2_1 = Computable::integer(BigInt::from(7_u8)).multiply(prescaled_9);
    let ln2_2 = Computable::integer(BigInt::from(2_u8)).multiply(prescaled_24);
    let ln2_3 = Computable::integer(BigInt::from(3_u8)).multiply(prescaled_80);

    ln2_1
        .add(ln2_2.negate())
        .add(ln2_3)
        .approx_signal(signal, p)?
})
}

fn ln_constant(signal: &Option<Signal>, n: Rational, p: Precision) -> Result<BigInt, Problem> {
Ok({
    // Non-ln2 logarithm constants reuse the normal logarithm reduction, then
    // benefit from the shared-constant cache on future calls.
    Computable::rational(n).ln().approx_signal(signal, p)?
})
}

fn sqrt_constant(signal: &Option<Signal>, n: Rational, p: Precision) -> Result<BigInt, Problem> {
Ok({
    // sqrt(2) and sqrt(3) are common exact trig results; they share caches even
    // though the approximation kernel is the generic sqrt.
    raw(Approximation::Sqrt(Computable::rational(n))).approx_signal(signal, p)?
})
}

fn acosh2_constant(signal: &Option<Signal>, p: Precision) -> Result<BigInt, Problem> {
Ok({
    // acosh(2) = ln(2 + sqrt(3)). This exact value is common enough in
    // inverse-hyperbolic tests to benefit from the shared-constant cache.
    Computable::rational(Rational::new(2))
        .add(Computable::sqrt_constant(3).unwrap())
        .ln()
        .approx_signal(signal, p)?
})
}

fn asinh1_constant(signal: &Option<Signal>, p: Precision) -> Result<BigInt, Problem> {
Ok({
    // asinh(1) = ln(1 + sqrt(2)), also equal to atanh(sqrt(2)/2).
    Computable::one()
        .add(Computable::sqrt_constant(2).unwrap())
        .ln()
        .approx_signal(signal, p)?
})
}

fn atan2_constant(signal: &Option<Signal>, p: Precision) -> Result<BigInt, Problem> {
Ok({
    // atan(2) = pi/2 - atan(1/2). The exact-rational atan reduction uses this
    // anchor for values near 2; a shared node lets promoted adversarial cases
    // reuse the combined approximation instead of assembling pi and atan(1/2)
    // independently at every precision.
    Computable::pi()
        .multiply(Computable::rational(Rational::fraction(1, 2).unwrap()))
        .add(Computable::atan_inv2_constant().negate())
        .approx_signal(signal, p)?
})
}

fn atan_three_halves_constant(signal: &Option<Signal>, p: Precision) -> Result<BigInt, Problem> {
Ok({
    // atan(3/2) = pi/4 + atan(1/5). This is the second midpoint anchor for
    // exact-rational atan; caching the assembled constant keeps the residual
    // series as the only per-call work after warm-up.
    Computable::pi()
        .multiply(Computable::rational(Rational::fraction(1, 4).unwrap()))
        .add(Computable::atan_inv5_constant())
        .approx_signal(signal, p)?
})
}

fn e_terms_for_precision(signal: &Option<Signal>, p: Precision) -> Result<u32, Problem> {
    // Choose enough 1/k! terms so the binary-split tail is below the requested
    // bit precision. Positive precisions need only a tiny constant amount.
    let needed_bits = if p < 0 { u64::from(p.unsigned_abs()) + 4 } else { 4 };
    // Keep mantissa * 2^shift <= n!, with at most 64 mantissa bits.
    // Exact word multiplication followed by downward truncation preserves
    // this lower bound. Crossing the threshold therefore cannot omit a term.
    let mut mantissa = 1_u64;
    let mut shift = 0_u64;
    let mut n = 0_u32;
    loop {
        if should_stop(signal) { return Err(Problem::Exhausted); }
        // A 64-bit mantissa times a 32-bit factor fits in 128 bits.
        let next = u128::from(mantissa) * u128::from(n + 1);
        let bits = u64::from(u128::BITS - next.leading_zeros());
        if bits + shift > needed_bits {
            return Ok(n);
        }
        let discarded = bits.saturating_sub(u64::BITS.into());
        mantissa = (next >> discarded) as u64;
        shift += discarded;
        n += 1;
    }
}

// Returns (P, Q) for sum_{k=a}^{b-1} 1 / prod_{j=a}^k j == P / Q
fn e_binary_split(signal: &Option<Signal>, a: u32, b: u32) -> Result<(BigUint, BigUint), Problem> {
    if should_stop(signal) { return Err(Problem::Exhausted); }
    // Binary splitting keeps numerator/denominator growth balanced. A linear
    // summation of rationals is noticeably more allocation-heavy for cold e.
    // This is the standard binary-splitting technique for series evaluation.
    if b - a == 1 {
        return Ok((BigUint::one(), BigUint::from(a)));
    }

    let mid = a + (b - a) / 2;
    let (left_p, left_q) = e_binary_split(signal, a, mid)?;
    let (right_p, right_q) = e_binary_split(signal, mid, b)?;
    Ok((left_p * &right_q + right_p, left_q * right_q))
}

fn rounded_ratio(numerator: BigUint, denominator: BigUint, p: Precision) -> Result<BigInt, Problem> {
Ok({
    // All kernels return an integer scaled by 2^-p and accurate within one unit
    // at that scale. Centralizing rounding avoids subtly different half-up
    // behavior between constants.
    if p <= 0 {
        let shift = usize::try_from(p.checked_neg().ok_or(Problem::OutOfRange)?).map_err(|_| Problem::OutOfRange)?;
        let dividend = numerator << shift;
        BigInt::from((dividend + (&denominator >> 1)) / denominator)
    } else {
        let shift = usize::try_from(p).expect("precision shift should fit in usize");
        let scaled_denominator = denominator << shift;
        BigInt::from((numerator + (&scaled_denominator >> 1)) / scaled_denominator)
    }
})
}

fn e(signal: &Option<Signal>, p: Precision) -> Result<BigInt, Problem> {
Ok({
    if should_stop(signal) { return Err(Problem::Exhausted); }
    // 2 < e < 3: coarse results require neither a large power of two nor a series.
    if p >= 2 { return Ok(BigInt::zero()); }
    // e = 1 + sum_{k>=1} 1/k!. The tail is evaluated as one rational by binary
    // splitting and rounded once at the target scale. Rounding only once keeps
    // the exact-real cache stable and avoids accumulating per-term rational
    // normalization costs.
    let terms = e_terms_for_precision(signal, p)?;
    if terms == 0 {
        return Ok(rounded_ratio(BigUint::one(), BigUint::one(), p)?);
    }

    let (tail_p, tail_q) = e_binary_split(signal, 1, terms + 1)?;
    rounded_ratio(tail_q.clone() + tail_p, tail_q, p)?
})
}
