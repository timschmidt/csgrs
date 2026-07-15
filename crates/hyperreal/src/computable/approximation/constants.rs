fn pi(signal: &Option<Signal>, p: Precision) -> BigInt {
    // Machin formula: pi = 4 * (4 atan(1/5) - atan(1/239)).
    // It converges much faster than a generic trig/log identity and is stable
    // enough to serve as the shared pi cache source. This is the same
    // arctangent/Machin-style family used in multiple-precision evaluation.
    let atan5 = Computable::prescaled_atan(BigInt::from(5_u8));
    let atan_239 = Computable::prescaled_atan(BigInt::from(239_u16));
    let four = Computable::integer(BigInt::from(4_u8));
    let four_atan5 = four.clone().multiply(atan5);
    let sum = four_atan5.add(atan_239.negate());
    four.multiply(sum).approx_signal(signal, p)
}

fn ln2(signal: &Option<Signal>, p: Precision) -> BigInt {
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
        .approx_signal(signal, p)
}

fn ln_constant(signal: &Option<Signal>, n: Rational, p: Precision) -> BigInt {
    // Non-ln2 logarithm constants reuse the normal logarithm reduction, then
    // benefit from the shared-constant cache on future calls.
    Computable::rational(n).ln().approx_signal(signal, p)
}

fn sqrt_constant(signal: &Option<Signal>, n: Rational, p: Precision) -> BigInt {
    // sqrt(2) and sqrt(3) are common exact trig results; they share caches even
    // though the approximation kernel is the generic sqrt.
    raw(Approximation::Sqrt(Computable::rational(n))).approx_signal(signal, p)
}

fn acosh2_constant(signal: &Option<Signal>, p: Precision) -> BigInt {
    // acosh(2) = ln(2 + sqrt(3)). This exact value is common enough in
    // inverse-hyperbolic tests to benefit from the shared-constant cache.
    Computable::rational(Rational::new(2))
        .add(Computable::sqrt_constant(3).unwrap())
        .ln()
        .approx_signal(signal, p)
}

fn asinh1_constant(signal: &Option<Signal>, p: Precision) -> BigInt {
    // asinh(1) = ln(1 + sqrt(2)), also equal to atanh(sqrt(2)/2).
    Computable::one()
        .add(Computable::sqrt_constant(2).unwrap())
        .ln()
        .approx_signal(signal, p)
}

fn atan2_constant(signal: &Option<Signal>, p: Precision) -> BigInt {
    // atan(2) = pi/2 - atan(1/2). The exact-rational atan reduction uses this
    // anchor for values near 2; a shared node lets promoted adversarial cases
    // reuse the combined approximation instead of assembling pi and atan(1/2)
    // independently at every precision.
    Computable::pi()
        .multiply(Computable::rational(Rational::fraction(1, 2).unwrap()))
        .add(Computable::atan_inv2_constant().negate())
        .approx_signal(signal, p)
}

fn atan_three_halves_constant(signal: &Option<Signal>, p: Precision) -> BigInt {
    // atan(3/2) = pi/4 + atan(1/5). This is the second midpoint anchor for
    // exact-rational atan; caching the assembled constant keeps the residual
    // series as the only per-call work after warm-up.
    Computable::pi()
        .multiply(Computable::rational(Rational::fraction(1, 4).unwrap()))
        .add(Computable::atan_inv5_constant())
        .approx_signal(signal, p)
}

fn e_terms_for_precision(p: Precision) -> u32 {
    // Choose enough 1/k! terms so the binary-split tail is below the requested
    // bit precision. Positive precisions need only a tiny constant amount.
    let needed_bits = if p < 0 { (-p) as u64 + 4 } else { 4 };
    let mut factorial = BigUint::one();
    let mut n = 0_u32;
    loop {
        let next = &factorial * BigUint::from(n + 1);
        if next.bits() > needed_bits {
            return n;
        }
        factorial = next;
        n += 1;
    }
}

// Returns (P, Q) for sum_{k=a}^{b-1} 1 / prod_{j=a}^k j == P / Q
fn e_binary_split(a: u32, b: u32) -> (BigUint, BigUint) {
    // Binary splitting keeps numerator/denominator growth balanced. A linear
    // summation of rationals is noticeably more allocation-heavy for cold e.
    // This is the standard binary-splitting technique for series evaluation.
    if b - a == 1 {
        return (BigUint::one(), BigUint::from(a));
    }

    let mid = a + (b - a) / 2;
    let (left_p, left_q) = e_binary_split(a, mid);
    let (right_p, right_q) = e_binary_split(mid, b);
    (left_p * &right_q + right_p, left_q * right_q)
}

fn rounded_ratio(numerator: BigUint, denominator: BigUint, p: Precision) -> BigInt {
    // All kernels return an integer scaled by 2^-p and accurate within one unit
    // at that scale. Centralizing rounding avoids subtly different half-up
    // behavior between constants.
    if p <= 0 {
        let shift = usize::try_from(-p).expect("precision shift should fit in usize");
        let dividend = numerator << shift;
        BigInt::from((dividend + (&denominator >> 1)) / denominator)
    } else {
        let shift = usize::try_from(p).expect("precision shift should fit in usize");
        let scaled_denominator = denominator << shift;
        BigInt::from((numerator + (&scaled_denominator >> 1)) / scaled_denominator)
    }
}

fn e(p: Precision) -> BigInt {
    // e = 1 + sum_{k>=1} 1/k!. The tail is evaluated as one rational by binary
    // splitting and rounded once at the target scale. Rounding only once keeps
    // the exact-real cache stable and avoids accumulating per-term rational
    // normalization costs.
    let terms = e_terms_for_precision(p);
    if terms == 0 {
        return rounded_ratio(BigUint::one(), BigUint::one(), p);
    }

    let (tail_p, tail_q) = e_binary_split(1, terms + 1);
    rounded_ratio(tail_q.clone() + tail_p, tail_q, p)
}
