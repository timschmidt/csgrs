/// Convert the exact bit-length difference and comparison correction plus an offset to an MSD.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures ({
        let exact = bit_length_msd(numerator_bits, denominator_bits, below, offset);
        &&& (result.is_none() <==> exact < i32::MIN || exact > i32::MAX)
        &&& match result { Some(msd) => msd == exact, None => true }
    }),
))]
#[inline]
pub(crate) fn checked_bit_length_msd(
    numerator_bits: u64,
    denominator_bits: u64,
    below: bool,
    offset: i32,
) -> Option<i32> {
    let difference =
        numerator_bits as i128 - denominator_bits as i128 - (if below { 1_i128 } else { 0_i128 })
            + offset as i128;
    if difference < i32::MIN as i128 || difference > i32::MAX as i128 {
        None
    } else {
        Some(difference as i32)
    }
}
