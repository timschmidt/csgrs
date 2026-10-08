use num::{BigInt, BigUint, One};
type Precision = i32;
pub fn e_terms_for_precision(p: Precision) -> u32 {
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

pub fn e(p: Precision) -> BigInt {
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
