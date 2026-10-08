pub(crate) fn shift(n: BigInt, p: Precision) -> BigInt {
    match 0.cmp(&p) {
        Ordering::Greater => n >> -p,
        Ordering::Equal => n,
        Ordering::Less => n << p,
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

