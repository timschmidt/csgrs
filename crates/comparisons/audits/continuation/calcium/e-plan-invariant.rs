type Precision = i32;
pub fn check_lower_invariant(p: Precision) -> u32 {
    // Choose enough 1/k! terms so the binary-split tail is below the requested
    // bit precision. Positive precisions need only a tiny constant amount.
    let needed_bits = if p < 0 { (-p) as u64 + 4 } else { 4 };
    // Keep mantissa * 2^shift <= n!, with at most 64 mantissa bits.
    // Exact word multiplication followed by downward truncation preserves
    // this lower bound. Crossing the threshold therefore cannot omit a term.
    let mut mantissa = 1_u64;
    let mut shift = 0_u64;
    let mut n = 0_u32;
    let mut witness = rug::Integer::from(1);
    loop {
        // A 64-bit mantissa times a 32-bit factor fits in 128 bits.
        let next = u128::from(mantissa) * u128::from(n + 1);
        witness *= n + 1;
        let mut lower = rug::Integer::from(next);
        lower <<= u32::try_from(shift).unwrap();
        assert!(lower <= witness);
        assert!(mantissa > 0);
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
