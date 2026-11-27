/// Largest precision request routed first to Chudnovsky: every fractional request `p <= -1`.
/// Coarser requests use Machin. This dispatch choice is not a universal timing
/// claim; a declined Chudnovsky attempt may also fall back to Machin.
const CHUDNOVSKY_PI_THRESHOLD: Precision = -1;
/// Exact integer `640320^3 / 24` in each nonzero Chudnovsky denominator factor.
const CHUDNOVSKY_C3_OVER_24: u64 = 10_939_058_860_032_000;
/// Extra binary digits retained before the final pi rounding.
/// These cover fixed-point square-root/division errors in the selected schedule;
/// they are not a bound on requested precision, coefficient size or total work.
const CHUDNOVSKY_GUARD_BITS: u64 = 64;

/// Build the exact Chudnovsky product/sum triple over the nonempty index interval `[a, b)`.
///
/// Return `(P,Q,T)` with products P and Q and the weighted sum T. Combining left
/// and right intervals uses `T = T_left*Q_right + P_left*T_right`, preserving
/// the rational series sum without term-by-term normalization. Index zero has
/// unit products and term 13,591,409; later terms alternate by index parity.
/// Require `a < b`; an empty interval is not validated or given a neutral result.
///
/// Poll `signal` at each recursive entry and propagate None on a stop. Integer
/// products between polls are not interruptible here. The balanced split bounds
/// recursion depth logarithmically in the interval length, not total work or
/// retained integer bytes.
fn chudnovsky_binary_split(
    signal: &Option<Signal>,
    a: u32,
    b: u32,
) -> Option<(BigInt, BigInt, BigInt)> {
    if should_stop(signal) {
        return None;
    }
    if b - a == 1 {
        if a == 0 {
            return Some((BigInt::one(), BigInt::one(), BigInt::from(13_591_409_u32)));
        }

        let k = u64::from(a);
        let p = BigInt::from(6 * k - 5) * BigInt::from(2 * k - 1) * BigInt::from(6 * k - 1);
        let q = BigInt::from(k).pow(3) * BigInt::from(CHUDNOVSKY_C3_OVER_24);
        let mut t = &p * BigInt::from(13_591_409_u64 + 545_140_134_u64 * k);
        if a & 1 == 1 {
            t = -t;
        }
        return Some((p, q, t));
    }

    let mid = a + (b - a) / 2;
    let (left_p, left_q, left_t) = chudnovsky_binary_split(signal, a, mid)?;
    let (right_p, right_q, right_t) = chudnovsky_binary_split(signal, mid, b)?;
    let t = left_t * &right_q + &left_p * right_t;
    Some((left_p * right_p, left_q * right_q, t))
}

/// Approximate pi in `2^p` units by exact binary splitting and a fixed-point square root.
///
/// Accept nonpositive precisions whose negation fits i32. Add 64 work bits and
/// use `ceil(work_bits/43) + 2` terms: the geometric term-ratio bound and
/// alternation control the omitted tail. Form `426880*sqrt(10005)*Q/T`, rounding
/// the positive work-scale quotient to nearest with upward ties, then discard
/// the guard bits with `scale`. This returns an approximation, not a guarantee
/// of correctly rounded pi at every request.
///
/// None covers checked conversion/arithmetic declines and observed cancellation;
/// it is not a typed exhausted-resource result. Positive p and i32::MIN decline
/// before splitting. Poll once more after splitting, but the subsequent integer
/// square root, products and division have no local cancellation poll or work
/// budget. The caller must reject an interrupted result before publication.
fn pi_chudnovsky(signal: &Option<Signal>, p: Precision) -> Option<BigInt> {
    let target_bits = u64::try_from(p.checked_neg()?).ok()?;
    let work_bits = target_bits.checked_add(CHUDNOVSKY_GUARD_BITS)?;

    // Consecutive absolute Chudnovsky terms have ratio below 2^-43. For k >= 1,
    // bounding the six numerator factors by 6(k+1), the three (3k+j) factors
    // below by 3k, and the linear-factor ratio by 2 proves the bound; the k=0
    // ratio is smaller by direct integer comparison. The alternating tail is
    // therefore below the first omitted term. Two extra terms put it well
    // beneath one work-scale ulp; 64 guard bits also dominate fixed-point sqrt
    // and final-division rounding before one final scale rounding.
    let terms = u32::try_from(work_bits.div_ceil(43).checked_add(2)?).ok()?;
    let (_, q, t) = chudnovsky_binary_split(signal, 0, terms)?;
    if should_stop(signal) {
        return None;
    }

    let sqrt_shift = usize::try_from(work_bits.checked_mul(2)?).ok()?;
    let sqrt_10005 = (BigUint::from(10_005_u32) << sqrt_shift).sqrt();
    let q = q.to_biguint()?;
    let denominator = t.to_biguint()?;
    let numerator = q * BigUint::from(426_880_u32) * sqrt_10005;
    let work_scaled = (numerator + (&denominator >> 1_usize)) / denominator;
    let guard = Precision::try_from(CHUDNOVSKY_GUARD_BITS).ok()?;
    Some(scale(BigInt::from(work_scaled), -guard))
}

/// Select a pi approximation kernel and return an integer in `2^p` units.
///
/// Fractional requests first try Chudnovsky. If that attempt declines without
/// a stop, or the request is coarser, evaluate Machin's exact identity
/// `pi = 4*(4*atan(1/5) - atan(1/239))`. A stop after a declined Chudnovsky
/// attempt returns a zero abort placeholder. The outer evaluator owns cache
/// publication and must not certify that placeholder.
///
/// This routine neither owns the shared pi cache nor makes every precision safe:
/// the fallback still has its own unchecked derived precisions and allocation
/// requirements. Kernel selection is not a total resource contract.
fn pi(signal: &Option<Signal>, p: Precision) -> BigInt {
    if p <= CHUDNOVSKY_PI_THRESHOLD {
        if let Some(result) = pi_chudnovsky(signal, p) {
            return result;
        }
        if should_stop(signal) {
            return BigInt::zero();
        }
    }

    // Machin formula: pi = 4 * (4 atan(1/5) - atan(1/239)).
    // Both reduced arctangents use exact integer denominators. This graph
    // supplies the coarse or declined-Chudnovsky fallback for the pi cache.
    let atan5 = Computable::prescaled_atan(BigInt::from(5_u8));
    let atan_239 = Computable::prescaled_atan(BigInt::from(239_u16));
    let four = Computable::integer(BigInt::from(4_u8));
    let four_atan5 = four.clone().multiply(atan5);
    let sum = four_atan5.add(atan_239.negate());
    four.multiply(sum).approx_signal(signal, p)
}

/// Approximate ln(2) through three small exact log1p arguments.
///
/// Use `7*ln(10/9) - 2*ln(25/24) + 3*ln(81/80)`; multiplying those rational
/// powers gives exactly two. Raw PrescaledLn nodes hold residuals 1/9, 1/24
/// and 1/80 so construction does not re-enter the shared ln2 identity. The
/// expression stays exact until evaluation at `p` with `signal`; the outer
/// constant evaluator owns cache reuse and abort publication.
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

/// Evaluate a positive rational logarithm for the non-ln2 shared constant recipes.
///
/// The dispatcher supplies 3, 5, 6, 7 or 10. Ordinary logarithm reduction retains
/// smooth composite structure, while its single-prime exclusion prevents a
/// shared prime-log cache fill from recursively requesting itself. This helper
/// assumes a positive input, passes `signal` and `p` through, and adds no local
/// domain check, cancellation poll or resource bound.
fn ln_constant(signal: &Option<Signal>, n: Rational, p: Precision) -> BigInt {
    // Non-ln2 logarithm constants reuse the normal logarithm reduction, then
    // benefit from the shared-constant cache on future calls.
    Computable::rational(n).ln().approx_signal(signal, p)
}

/// Approximate a nonnegative rational square root without re-entering constructor recognition.
///
/// The shared table supplies two or three. A raw Sqrt node preserves the generic
/// root kernel instead of simplifying back to the shared constant being filled.
/// The caller owns domain admission and cache publication; this wrapper forwards
/// `signal` and `p` without a separate work limit.
fn sqrt_constant(signal: &Option<Signal>, n: Rational, p: Precision) -> BigInt {
    // sqrt(2) and sqrt(3) are common exact trig results; they share caches even
    // though the approximation kernel is the generic sqrt.
    raw(Approximation::Sqrt(Computable::rational(n))).approx_signal(signal, p)
}

/// Evaluate the exact shared identity `acosh(2) = ln(2 + sqrt(3))` at precision `p`.
/// Reuse the admitted shared sqrt(3) entry; its known table membership justifies
/// unwrap. The outer constant cache handles reuse, while the expression receives
/// `signal` and owns the underlying approximation work.
fn acosh2_constant(signal: &Option<Signal>, p: Precision) -> BigInt {
    // acosh(2) = ln(2 + sqrt(3)). This exact value is common enough in
    // inverse-hyperbolic tests to benefit from the shared-constant cache.
    Computable::rational(Rational::new(2))
        .add(Computable::sqrt_constant(3).unwrap())
        .ln()
        .approx_signal(signal, p)
}

/// Evaluate the shared identity `asinh(1) = ln(1 + sqrt(2))` at precision `p`.
/// This also equals `atanh(sqrt(2)/2)`. The known shared sqrt(2) entry justifies
/// unwrap; the exact expression is evaluated with `signal` and cached by the
/// outer constant machinery, not by this recipe.
fn asinh1_constant(signal: &Option<Signal>, p: Precision) -> BigInt {
    // asinh(1) = ln(1 + sqrt(2)), also equal to atanh(sqrt(2)/2).
    Computable::one()
        .add(Computable::sqrt_constant(2).unwrap())
        .ln()
        .approx_signal(signal, p)
}

/// Evaluate the one-argument constant `atan(2) = pi/2 - atan(1/2)`.
/// This is an anchor for rational arctangent reduction, not the two-argument
/// atan2 operation. Shared child constants retain their exact meanings; the
/// outer evaluator caches the combined result at precision `p`. Forward `signal`
/// without adding a local work or cancellation bound.
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

/// Evaluate the rational-reduction anchor `atan(3/2) = pi/4 + atan(1/5)`.
/// The addition stays on the principal branch. Shared child samples and the
/// outer cache permit reuse at compatible precisions; finer requests can still
/// require work. Evaluate the exact graph with `signal` at precision `p`.
fn atan_three_halves_constant(signal: &Option<Signal>, p: Precision) -> BigInt {
    // atan(3/2) = pi/4 + atan(1/5). This is the second midpoint anchor for
    // exact-rational atan; the shared table can reuse the assembled anchor
    // sample when its retained precision satisfies the next request.
    Computable::pi()
        .multiply(Computable::rational(Rational::fraction(1, 4).unwrap()))
        .add(Computable::atan_inv5_constant())
        .approx_signal(signal, p)
}

/// Plan the factorial-series truncation index for e using a downward-rounded word bound.
///
/// Maintain `mantissa * 2^shift <= n!`, multiplying in u128 before truncating
/// back to at most 64 mantissa bits. Stop when the next factorial's lower bound
/// crosses the requested bit threshold and return n: the evaluator sums through
/// `1/n!`, excluding the crossing term. Four extra requested bits leave room
/// for the factorial tail before rounding. The planner can overestimate the
/// minimal index; it does not promise an exact optimum.
///
/// Nonnegative precisions use a four-bit threshold and currently return three.
/// Negating i32::MIN overflows in checked builds. There is no stop signal or
/// enforced work budget; bounded word storage does not bound the loop's work.
fn e_terms_for_precision(p: Precision) -> u32 {
    // Choose enough 1/k! terms so the binary-split tail is below the requested
    // bit precision. Positive precisions need only a tiny constant amount.
    let needed_bits = if p < 0 { (-p) as u64 + 4 } else { 4 };
    // Keep mantissa * 2^shift <= n!, with at most 64 mantissa bits.
    // Exact word multiplication followed by downward truncation preserves
    // this lower bound. Crossing the threshold therefore cannot omit a term.
    let mut mantissa = 1_u64;
    let mut shift = 0_u64;
    let mut n = 0_u32;
    loop {
        // A 64-bit mantissa times a 32-bit factor fits in 128 bits.
        let next = u128::from(mantissa) * u128::from(n + 1);
        let bits = u64::from(u128::BITS - next.leading_zeros());
        if bits + shift > needed_bits {
            return n;
        }
        let discarded = bits.saturating_sub(u64::BITS.into());
        mantissa = (next >> discarded) as u64;
        shift += discarded;
        n += 1;
    }
}

/// Return `(P,Q)` for the exact sum of reciprocal successive products over `1 <= a < b`.
///
/// The value is `sum(k=a..b-1, 1/product(j=a..k, j))`. Combining subintervals
/// multiplies their denominators and uses `P_left*Q_right + P_right` for the
/// numerator. For a=1 this is the truncated factorial tail of e. Require a
/// positive start and nonempty interval; no invalid-range validation is done.
/// Balanced recursion limits depth, while integer growth and total work remain
/// unbounded by this helper. It has no cancellation signal.
fn e_binary_split(a: u32, b: u32) -> (BigUint, BigUint) {
    // Combine exact integer products over balanced subintervals, leaving
    // quotient rounding until the complete truncated series has been formed.
    if b - a == 1 {
        return (BigUint::one(), BigUint::from(a));
    }

    let mid = a + (b - a) / 2;
    let (left_p, left_q) = e_binary_split(a, mid);
    let (right_p, right_q) = e_binary_split(mid, b);
    (left_p * &right_q + right_p, left_q * right_q)
}

/// Round a nonnegative exact quotient to the grid `2^p`, returning the grid integer.
///
/// Require a positive denominator. Shift the numerator for nonpositive p, or
/// the denominator for positive p, and divide with half the divisor added.
/// This is nearest-integer rounding with ties upward for the exact quotient
/// supplied here; it does not certify an omitted series tail.
/// Zero denominators divide by zero, and negating i32::MIN is unsupported by
/// this arithmetic. Shifts/division have no signal or local allocation budget.
fn rounded_ratio(numerator: BigUint, denominator: BigUint, p: Precision) -> BigInt {
    // Round the supplied nonnegative rational with the same half-up rule
    // for either sign of precision.
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

/// Approximate e by one exact truncated factorial sum followed by one quotient rounding.
///
/// The planner chooses n; binary splitting forms `sum(k=1..n, 1/k!)` and the
/// initial one is added before rounding to `2^p` units. The zero-term branch
/// is defensive; the current nonnegative-precision plan already selects three
/// terms. The omitted positive tail belongs to the approximation error budget,
/// so correctly rounding this rational sum is not a correctly-rounded-e promise.
///
/// This kernel has no signal parameter or inner cancellation poll. Its caller
/// must handle any stop before publishing a result. Extreme precision arithmetic
/// and potentially large integers remain subject to the planner/rounder limits;
/// there is no typed total-work or memory budget here.
fn e(p: Precision) -> BigInt {
    // e = 1 + sum_{k>=1} 1/k!. Evaluate the truncated series as one rational
    // by binary splitting, then round once at the target scale rather than
    // rounding each factorial term separately.
    let terms = e_terms_for_precision(p);
    if terms == 0 {
        return rounded_ratio(BigUint::one(), BigUint::one(), p);
    }

    let (tail_p, tail_q) = e_binary_split(1, terms + 1);
    rounded_ratio(tail_q.clone() + tail_p, tail_q, p)
}
