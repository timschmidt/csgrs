//! Fixed-width arithmetic gates and exponent metadata shared by scalar consumers.
//!
//! These kernels plan integer rounding, compare checked cross products, add signed
//! magnitudes, align words and combine binary-exponent metadata. They operate on
//! primitive integers without heap allocation or shared caches. An Option decline
//! means the stated native width/range gate failed, not resource exhaustion or a
//! claim that an arbitrary-width computation is impossible.
//!
//! Callers retain authority over fraction validity, the meaning of bit lengths and
//! comparison corrections, and any large integer allocation after a scale plan.
//! Verification-only relations and lemmas describe this same executable arithmetic;
//! ordinary builds erase them and the proof blocks. Their presence is not evidence
//! of a fresh verifier run or a proof of every caller's geometric/numeric meaning.

#[cfg(verus_keep_ghost)]
use vstd::arithmetic::power2::{lemma_pow2_unfold, pow2};
#[cfg(verus_keep_ghost)]
use vstd::prelude::*;

use core::cmp::Ordering;

/// Plan exact scaling or nearest-integer rounding of an integer times 2^-precision.
///
/// For precision <= 0, returns (false, -precision): the consumer left-shifts by
/// that many bits. For positive precision, returns (true, precision-1): the
/// consumer computes ((n >> bits) + 1) >> 1, using arithmetic right shifts to
/// round nearest with ties toward positive infinity for either sign.
///
/// Accepts every i32, including MIN, whose exact shift count is 2^31 after i64
/// negation. This function only returns metadata; it neither rounds a value nor
/// bounds the consumer's potentially enormous allocation or total work.
#[cfg_attr(verus_keep_ghost, verus_spec(plan =>
    ensures
        plan.0 == (precision > 0),
        if precision <= 0 { plan.1 as int == -(precision as int) }
        else { plan.1 as int == precision as int - 1 },
))]
#[inline]
pub(crate) fn approximation_scale_plan(precision: i32) -> (bool, u32) {
    if precision <= 0 {
        (false, (-(precision as i64)) as u32)
    } else {
        (true, (precision - 1) as u32)
    }
}

/// Compare `left_numerator * right_denominator` with the reverse cross product.
///
/// Both it and `right_numerator * left_denominator` must fit u128. Either overflow
/// returns None even when another
/// argument or comparison strategy could decide the order. The left product is
/// checked first. Successful Less/Equal/Greater is the exact product ordering.
///
/// All four raw words, including zero denominators, are accepted arithmetically.
/// Interpreting the answer as an unsigned rational order requires positive
/// denominators supplied by the caller. No sign handling, fraction validation,
/// reduction, arbitrary-width fallback or resource-exhaustion result occurs here.
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
/// Specify the unbounded integer arithmetic behind checked magnitude metadata.
///
/// Subtract denominator bits and a Boolean one-bit correction from numerator bits,
/// then add the signed offset without an i32 range restriction. For positive
/// rationals the magnitude model supplies bit-length and aligned-comparison proofs
/// before using this as a scaled binade exponent. This relation alone does not
/// validate those inputs, exclude zero or prove any operand's numerical magnitude.
pub(crate) open spec fn bit_length_msd(
    numerator_bits: u64, denominator_bits: u64, below: bool, offset: i32,
) -> int {
    numerator_bits as int - denominator_bits as int
        - (if below { 1int } else { 0int }) + offset as int
}

/// Interpret a Boolean sign and u128 magnitude as an unbounded mathematical integer.
///
/// True negates the magnitude; either sign maps zero to zero. This shared ghost
/// meaning connects signed_add and aggregate/dyadic identities without restricting
/// magnitudes to i128 or requiring canonical input-zero signs.
pub(crate) open spec fn signed(negative: bool, magnitude: u128) -> int {
    if negative { -(magnitude as int) } else { magnitude as int }
}

/// Connect the representable factor 1u128 << shift to the mathematical power 2^shift.
///
/// Requires shift < 128. Recursion decreases shift, using the zero-power base and
/// one-bit doubling plus the power recurrence. Word alignment and shifted-limb
/// proofs consume the identity. The lemma is erased from ordinary execution;
/// its source contract is not by itself evidence that verification succeeded.
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

/// Connect a bounded u128 machine left shift to exact multiplication by 2^shift.
///
/// Requires shift < 128 and value*2^shift <= u128::MAX in mathematical arithmetic.
/// These hypotheses exclude discarded high bits, including in each recursive
/// predecessor step. Induction on shift combines the previous identity with a
/// one-bit doubling identity and the power recurrence; zero shift is the base.
/// GCD proofs use this when restoring a factored common power of two. It is a
/// ghost proof obligation, not an executable overflow test or a fresh proof result.
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

/// Add two Boolean-sign/u128-magnitude integers exactly when the magnitude fits.
///
/// True denotes negation. The admitted signed range is -u128::MAX..=u128::MAX,
/// not the narrower i128 range; either sign is accepted on an input zero.
/// Equal signs use checked magnitude addition, returning None on overflow.
/// Opposite signs subtract the smaller magnitude from the larger and always fit.
/// Every zero result is canonicalized to (false, 0), including two negative zeros.
///
/// Used after native product alignment and by rational sign adapters. The function
/// performs no scaling or normalization beyond the zero sign; a decline concerns
/// native magnitude capacity, not cancellation in a larger multi-term sum.
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

/// Multiply a u128 word by 2^shift only when both the shift and result fit this gate.
///
/// Returns None for every shift >= 128, including a zero value. For smaller
/// shifts, checked multiplication by the representable power of two rejects lost
/// high bits, unlike a plain machine left shift. Zero succeeds for shifts below
/// 128; shift zero preserves every input. This is native alignment eligibility,
/// not a saturating operation, arbitrary-width fallback or resource limit.
/// The conditional proof connects the constructed factor to the exact power.
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

/// Apply the checked native left-shift gate to a full u64 exponent difference.
///
/// Rejects shift >= 128 before narrowing to u32, including all such shifts of
/// zero. Smaller shifts delegate to checked_shift_left and decline if the exact
/// scaled magnitude exceeds u128::MAX. Aggregate/dyadic callers can therefore
/// pass wide metadata without truncation; no huge power is allocated or evaluated.
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

/// Return floor(value/2) for every i32 using an arithmetic right shift.
///
/// Negative odd exponents round downward rather than toward zero: -3 becomes -2.
/// Both signed endpoints are safe. The square-root bound consumer uses this to
/// transport a binary magnitude exponent; this kernel alone does not validate
/// that the input is a bound or establish the source value's square-root domain.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures 2 * (result as int) <= value as int,
        (value as int) < 2 * (result as int) + 2,
))]
#[inline]
pub(crate) fn floor_half(value: i32) -> i32 {
    let half = value >> 1;
    proof! {
        assert(2 * (half as i64) <= value as i64 && (value as i64) < 2 * (half as i64) + 2)
            by (bit_vector) requires half == value >> 1;
    }
    half
}

/// Compute numerator_bits - denominator_bits - below + offset, then check i32 range.
///
/// Below contributes one when true. All intermediate arithmetic uses i128, so
/// any u64 bit-length pair and i32 offset fit without wraparound. The correction
/// and offset apply before narrowing: a suitable offset can rescue a bit-length
/// difference that alone would be outside i32. Returns None exactly when the
/// final integer cannot be represented, without allocating or inspecting operands.
///
/// For a positive rational, actual magnitude bit lengths and a certified aligned
/// comparison make this the floor-log2 magnitude plus offset. Those semantic
/// preconditions belong to the caller; zero or inconsistent metadata is not
/// rejected as invalid here. Computable sample bounds also use the arithmetic
/// with their own provenance. None is an exponent-range decline, not proof that
/// the underlying number or a larger-width exponent is unavailable.
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
