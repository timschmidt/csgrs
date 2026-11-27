/// Multiply an integer by `2^p`, flooring the result when p is negative.
/// Arithmetic right shift rounds negative fractions toward negative infinity,
/// unlike the nearest-integer helpers below. Zero shift returns the owned input.
/// The unsigned shift magnitude handles i32::MIN without signed negation; large
/// left shifts can still allocate correspondingly large integers. No work,
/// allocation or cancellation budget is enforced by this helper.
pub(crate) fn shift(n: BigInt, p: Precision) -> BigInt {
    match 0.cmp(&p) {
        Ordering::Greater => n >> p.unsigned_abs(),
        Ordering::Equal => n,
        Ordering::Less => n << p,
    }
}

/// Round `n * 2^-p` to the nearest integer, with exact halfway cases toward positive infinity.
/// Nonpositive p scales exactly; positive p uses one guard-bit right shift,
/// adds one and shifts once more to implement the same tie rule for both signs.
/// The shared word planner represents the i32::MIN left shift as 2^31 positions
/// without signed overflow. A nonzero result can still grow by that many bits:
/// representable shift arithmetic is not an allocation or total-work bound.
/// Zero remains zero even at that endpoint. This is integer-leaf rounding,
/// not a correctly rounded conversion of an arbitrary computable expression.
pub(crate) fn scale_at_precision(n: BigInt, p: Precision) -> BigInt {
    let (round, bits) = crate::verified::word::approximation_scale_plan(p);
    if round {
        ((n >> bits) + signed::ONE.deref()) >> 1
    } else {
        n << bits
    }
}

/// Round `n * 2^p` to the nearest integer, choosing positive infinity at a tie.
/// Nonnegative p is an exact left shift; negative p uses a guard bit and the
/// floor-shift helper. Thus scale(10, 2) is 40, scale(10, -2) is 3, and
/// scale(-10, -2) is -2. The exponent sign is opposite to scale_at_precision.
/// Computing p+1 only on the negative branch handles i32::MIN safely, but
/// large left shifts retain the caller's allocation/work obligations.
pub(crate) fn scale(n: BigInt, p: Precision) -> BigInt {
    if p >= 0 {
        n << p
    } else {
        let adj = shift(n, p + 1) + signed::ONE.deref();
        adj >> 1
    }
}
