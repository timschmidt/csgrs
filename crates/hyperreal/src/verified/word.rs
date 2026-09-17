#[cfg(verus_keep_ghost)]
use vstd::arithmetic::power2::{lemma_pow2_unfold, pow2};
#[cfg(verus_keep_ghost)]
use vstd::prelude::*;

use core::cmp::Ordering;

/// Compare cross products without accepting a wrapped machine product.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures
        result.is_none() <==> (
            left_numerator as int * right_denominator as int > u128::MAX
            || right_numerator as int * left_denominator as int > u128::MAX
        ),
        match result {
            Some(Ordering::Less) => (left_numerator as int * right_denominator as int)
                < (right_numerator as int * left_denominator as int),
            Some(Ordering::Equal) => left_numerator as int * right_denominator as int
                == right_numerator as int * left_denominator as int,
            Some(Ordering::Greater) => (left_numerator as int * right_denominator as int)
                > (right_numerator as int * left_denominator as int),
            None => true,
        },
))]
#[inline]
pub(crate) fn compare_products(
    left_numerator: u128,
    left_denominator: u128,
    right_numerator: u128,
    right_denominator: u128,
) -> Option<Ordering> {
    let left = left_numerator.checked_mul(right_denominator)?;
    let right = right_numerator.checked_mul(left_denominator)?;
    Some(left.cmp(&right))
}

#[cfg(verus_keep_ghost)]
verus! {
pub(crate) open spec fn signed(negative: bool, magnitude: u128) -> int {
    if negative { -(magnitude as int) } else { magnitude as int }
}

pub(crate) proof fn one_shift_is_power_of_two(shift: u32)
    requires shift < 128,
    ensures (1u128 << shift) == pow2(shift as nat),
    decreases shift,
{
    if shift == 0 {
        vstd::arithmetic::power::lemma_pow0(2);
        assert(1u128 << 0u32 == 1u128) by (bit_vector);
    } else {
        one_shift_is_power_of_two((shift - 1) as u32);
        lemma_pow2_unfold(shift as nat);
        assert((1u128 << shift) == 2 * (1u128 << ((shift - 1) as u32))) by (bit_vector)
            requires 0 < shift < 128;
    }
}

/// Widen vstd's bounded-shift multiplication lemma to a two-limb word.
pub(crate) proof fn shift_left_u128_is_mul(value: u128, shift: u32)
    requires shift < 128, value * pow2(shift as nat) <= u128::MAX,
    ensures (value << shift) == value * pow2(shift as nat),
    decreases shift,
{
    if shift == 0 {
        vstd::arithmetic::power::lemma_pow0(2);
        assert(value << 0u32 == value) by (bit_vector);
        assert(value * pow2(0) == value) by (nonlinear_arith) requires pow2(0) == 1;
    } else {
        let previous = pow2((shift - 1) as nat);
        lemma_pow2_unfold(shift as nat);
        assert(value * previous <= u128::MAX && 2 * (value * previous) <= u128::MAX)
            by (nonlinear_arith)
            requires value * pow2(shift as nat) <= u128::MAX,
                pow2(shift as nat) == 2 * previous, previous >= 0, value >= 0;
        shift_left_u128_is_mul(value, (shift - 1) as u32);
        assert((value << ((shift - 1) as u32)) <= u128::MAX / 2);
        assert((value << shift) == 2 * (value << ((shift - 1) as u32))) by (bit_vector)
            requires 0 < shift < 128,
                (value << ((shift - 1) as u32)) <= u128::MAX / 2;
        assert(value * pow2(shift as nat) == 2 * (value * previous))
            by (nonlinear_arith) requires pow2(shift as nat) == 2 * previous;
    }
}
}

/// Add signed magnitudes, reporting overflow rather than discarding high bits.
/// A zero result always has a nonnegative sign.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures
        result.is_none() <==> (
            signed(left_negative, left) + signed(right_negative, right) > u128::MAX
            || signed(left_negative, left) + signed(right_negative, right) < -(u128::MAX as int)
        ),
        match result {
            Some((negative, magnitude)) =>
                signed(negative, magnitude)
                    == signed(left_negative, left) + signed(right_negative, right)
                && (magnitude == 0 ==> !negative),
            None => true,
        },
))]
#[inline]
pub(crate) fn signed_add(
    left_negative: bool,
    left: u128,
    right_negative: bool,
    right: u128,
) -> Option<(bool, u128)> {
    if left_negative == right_negative {
        let magnitude = left.checked_add(right)?;
        Some((left_negative && magnitude != 0, magnitude))
    } else if left > right {
        Some((left_negative, left - right))
    } else if right > left {
        Some((right_negative, right - left))
    } else {
        Some((false, 0))
    }
}

/// Multiplication by a representable power of two, with overflow detection.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures
        result.is_none() <==> (shift >= 128
            || value as int * pow2(shift as nat) as int > u128::MAX),
        match result {
            Some(scaled) => shift < 128
                && scaled == value as int * pow2(shift as nat) as int,
            None => true,
        },
))]
#[inline]
pub(crate) fn checked_shift_left(value: u128, shift: u32) -> Option<u128> {
    if shift >= 128 {
        None
    } else {
        proof! { one_shift_is_power_of_two(shift); }
        value.checked_mul(1_u128 << shift)
    }
}

/// Accept exponent metadata without narrowing it before checking the word bound.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures result.is_none() <==> (shift >= 128 || value as nat * pow2(shift as nat) > u128::MAX),
        match result {
            Some(scaled) => scaled == value as nat * pow2(shift as nat) && shift < 128,
            None => true,
        },
))]
#[inline]
pub(crate) fn checked_shift_left_u64(value: u128, shift: u64) -> Option<u128> {
    if shift >= 128 {
        return None;
    }
    checked_shift_left(value, shift as u32)
}
