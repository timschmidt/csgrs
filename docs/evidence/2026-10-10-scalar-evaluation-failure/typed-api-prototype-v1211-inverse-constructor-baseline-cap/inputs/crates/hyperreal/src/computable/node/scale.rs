/// Reject impractical fractional requests before shifting or allocating.
/// This guards requested/intermediate precision, not total evaluation work.
pub(crate) fn checked_approximation_precision(p: Precision) -> Result<Precision, Problem> {
    if p == Precision::MIN {
        return Err(Problem::OutOfRange);
    }
    if p < -Computable::MAX_APPROXIMATION_BITS {
        return Err(Problem::Exhausted);
    }
    Ok(p)
}

/// Precision/scale arithmetic uses a wider accumulator before narrowing.
pub(crate) fn precision_from_i64(value: i64) -> Result<Precision, Problem> {
    Precision::try_from(value).map_err(|_| Problem::OutOfRange)
}

pub(crate) fn shift(n: BigInt, p: Precision) -> BigInt {
    match 0.cmp(&p) {
        Ordering::Greater => n >> p.unsigned_abs(),
        Ordering::Equal => n,
        Ordering::Less => n << p,
    }
}

/// Approximate an exact integer at precision p: round n * 2^(-p).
/// The left shift at i32::MIN needs 2^31 bits, which fits u32 but not i32.
pub(crate) fn scale_at_precision(n: BigInt, p: Precision) -> BigInt {
    let (round, bits) = crate::verified::word::approximation_scale_plan(p);
    if round {
        ((n >> bits) + signed::ONE.deref()) >> 1
    } else {
        n << bits
    }
}

/// Scale n by p bits, rounding if this makes n smaller.
/// e.g. scale(10, 2) == 40
///      scale(10, -2) == 3
pub(crate) fn scale(n: BigInt, p: Precision) -> BigInt {
    if p >= 0 {
        n << p
    } else {
        let adj = shift(n, p + 1) + signed::ONE.deref();
        adj >> 1
    }
}
