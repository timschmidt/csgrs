//! Exact signed differences and reduction of power-of-two denominators.
//!
//! The native Word carrier retains sign/activity, an unsigned numerator and a
//! binary denominator exponent. Limb arrays hold unsigned base-2^64 magnitudes,
//! least significant digit first; this numerical ordering is not byte serialization.
//! The rational adapters translate sign flags, and aggregate consumers use these
//! kernels after their unsigned subtotals have fit.
//!
//! Option has operation-specific meaning here. difference and finish return None
//! exactly for zero, which consumers turn into their canonical zero carrier.
//! difference_word instead returns None for a native alignment or magnitude gate
//! and returns Some for zero. These meanings must remain distinct in callers.
//!
//! Reduction removes only powers of two shared with the denominator. Exponents,
//! including u64::MAX, remain metadata without allocating a giant denominator.
//! Runtime bodies use native arithmetic and owned or borrowed fixed arrays without
//! heap allocation. Limb scans, subtraction and shifting depend on N; no local
//! total-work, stack or cancellation budget is enforced. Ghost relations and proofs
//! state the arithmetic and disappear in ordinary builds.

#[cfg(verus_keep_ghost)]
use super::division::gcd;
#[cfg(verus_keep_ghost)]
use super::limbs::value;
#[cfg(verus_keep_ghost)]
use vstd::arithmetic::{div_mod::*, power2::*};
#[cfg(verus_keep_ghost)]
use vstd::{math::abs, prelude::*};

use super::limbs;
use core::cmp::Ordering;

/// Raw signed dyadic numerator with a binary denominator exponent.
///
/// An active carrier denotes its signed magnitude divided by `2^shift`; an
/// inactive carrier denotes zero regardless of the stored sign or magnitude.
/// Construction and derived Clone/Copy preserve all fields without validation or
/// normalization. Raw inputs can therefore contain inactive nonzero payloads,
/// negative zero or unreduced fractions.
///
/// normalize_word and successful difference_word establish canonical outputs:
/// activity agrees with nonzero magnitude, zero has negative=false and shift=0,
/// and a noninteger nonzero result has an odd numerator. This representation uses
/// an unsigned u128 magnitude with an independent sign, not an i128 range.
#[cfg_attr(verus_keep_ghost, verus_verify)]
#[derive(Clone, Copy)]
pub(crate) struct Word {
    /// Whether the stored numerator contributes value; canonical outputs set this exactly for nonzero magnitude.
    pub active: bool,
    /// Sign of an active numerator; canonical zero clears this flag, while raw inactive inputs may retain it.
    pub negative: bool,
    /// Unsigned native numerator; raw inactive payloads are still checked for alignment by difference_word.
    pub magnitude: u128,
    /// Binary denominator exponent, so the denominator is `2^shift`; canonical zero uses zero.
    pub shift: u64,
}

#[cfg(verus_keep_ghost)]
verus! {
/// Apply a Boolean sign to a mathematical natural-number magnitude.
///
/// Returns an integer with the same absolute value; either sign of zero denotes
/// zero. This relation applies no activity convention or denominator and has no
/// native signed-width restriction.
pub(crate) open spec fn signed(negative: bool, magnitude: nat) -> int {
    if negative { -(magnitude as int) } else { magnitude as int }
}

/// Denote the signed numerator of a raw Word, applying its activity flag.
///
/// Inactive means the integer zero independently of stored fields. For active
/// inputs this applies negative to magnitude. The denominator is deliberately
/// absent: the represented rational is this integer divided by `2^word.shift`.
pub(crate) open spec fn word_value(word: Word) -> int {
    if word.active { signed(word.negative, word.magnitude as nat) } else { 0 }
}

/// Specify canonical activity, zero metadata and binary fraction reduction.
///
/// Activity must agree with positive magnitude. Inactive therefore means zero
/// magnitude, nonnegative metadata and shift zero. A nonzero result either has
/// shift zero or an odd numerator, and its magnitude is coprime to `2^shift`.
/// The predicate describes raw fields; it does not normalize or validate a runtime
/// instance. Active nonzero carriers may have either sign.
pub(crate) open spec fn canonical_word(word: Word) -> bool {
    &&& word.active == (word.magnitude > 0)
    &&& (!word.active ==> !word.negative && word.shift == 0)
    &&& (word.shift == 0 || word.magnitude % 2 == 1)
    &&& gcd(word.magnitude as nat, pow2(word.shift as nat)) == 1
}

/// Select the larger stored denominator exponent, including inactive carriers.
///
/// This common scale bounds both exponents without adding them. It is used for
/// subtraction alignment even when one or both represented values are zero.
pub(crate) open spec fn word_scale(left: Word, right: Word) -> nat {
    if left.shift > right.shift { left.shift as nat } else { right.shift as nat }
}

/// Denote an activity-aware signed numerator at a chosen common binary scale.
///
/// Multiplies word_value by `2^(scale - word.shift)`. Fraction preservation requires
/// `word.shift <= scale`; the definition itself does not impose this precondition.
/// The mathematical integer has no native-width gate and does not describe the
/// raw-magnitude alignment checks performed before inactivity in difference_word.
pub(crate) open spec fn aligned_word(word: Word, scale: nat) -> int {
    word_value(word) * pow2((scale - word.shift) as nat)
}

/// Denote left minus right as a signed numerator at their common scale.
///
/// Subtracts their activity-aware aligned numerators at word_scale. Dividing by
/// `2^word_scale(left, right)` yields the rational difference. The integer itself
/// is neither reduced nor constrained to fit u128, so its existence does not imply
/// that the native difference schedule admits these raw carriers.
pub(crate) open spec fn word_difference(left: Word, right: Word) -> int {
    aligned_word(left, word_scale(left, right)) - aligned_word(right, word_scale(left, right))
}

/// Prove that aligning an activity-aware numerator preserves its fraction.
///
/// Requires the new scale to be no smaller than the stored denominator exponent.
/// Power-of-two factorization establishes the cross-multiplied equality, including
/// inactive and zero carriers. It asserts neither canonical fields nor native
/// alignment eligibility or runtime materialization of either denominator.
pub(crate) proof fn word_alignment_preserves_fraction(word: Word, scale: nat)
    requires word.shift <= scale,
    ensures aligned_word(word, scale) * pow2(word.shift as nat) == word_value(word) * pow2(scale),
{
    lemma_pow2_adds(word.shift as nat, (scale - word.shift) as nat);
    assert(aligned_word(word, scale) * pow2(word.shift as nat) == word_value(word) * pow2(scale))
        by (nonlinear_arith)
        requires pow2(scale) == pow2(word.shift as nat) * pow2((scale - word.shift) as nat);
}

/// Remove part or all of a known exact binary factor from a positive magnitude.
///
/// Requires a positive odd quotient at trailing and exact reconstruction by
/// `2^trailing`, with `removed <= trailing`. The quotient after removing the
/// smaller factor stays positive and reconstructs the original exactly. It is
/// proved odd when the whole trailing factor is removed; removing fewer factors
/// does not assert oddness. The caller must establish the original factorization.
pub(crate) proof fn cancel_power_two(magnitude: nat, trailing: nat, removed: nat)
    requires removed <= trailing,
        magnitude / pow2(trailing) > 0,
        (magnitude / pow2(trailing)) % 2 == 1,
        magnitude == (magnitude / pow2(trailing)) * pow2(trailing),
    ensures magnitude / pow2(removed) > 0,
        magnitude == (magnitude / pow2(removed)) * pow2(removed),
        removed == trailing ==> (magnitude / pow2(removed)) % 2 == 1,
{
    let odd = magnitude / pow2(trailing);
    let remaining = pow2((trailing - removed) as nat);
    let factor = pow2(removed);
    lemma_pow2_pos(removed);
    lemma_pow2_pos((trailing - removed) as nat);
    lemma_pow2_adds(removed, (trailing - removed) as nat);
    assert(magnitude == factor * (odd * remaining)) by (nonlinear_arith)
        requires magnitude == odd * pow2(trailing), pow2(trailing) == factor * remaining;
    assert(magnitude == (odd * remaining) * factor) by (nonlinear_arith)
        requires magnitude == factor * (odd * remaining);
    lemma_fundamental_div_mod_converse_div(magnitude as int, factor as int,
        (odd * remaining) as int, 0);
    assert(odd * remaining > 0) by (nonlinear_arith) requires odd > 0, remaining > 0;
    assert(magnitude == (magnitude / factor) * factor) by (nonlinear_arith)
        requires magnitude == factor * (odd * remaining), magnitude / factor == odd * remaining;
}

/// Transport a signed dyadic fraction through an exact numerator/denominator reduction.
///
/// Requires `after_shift <= before_shift` and reconstruction of before from after
/// times the removed power of two. Cross multiplication proves equality of signed
/// fractions with the same negative flag. Zero is allowed, and no positivity,
/// canonical metadata or coprimality is concluded from this identity alone.
pub(crate) proof fn reduction_preserves_fraction(
    negative: bool, before: nat, after: nat, before_shift: nat, after_shift: nat,
)
    requires after_shift <= before_shift,
        before == after * pow2((before_shift - after_shift) as nat),
    ensures signed(negative, after) * pow2(before_shift)
        == signed(negative, before) * pow2(after_shift),
{
    lemma_pow2_adds(after_shift, (before_shift - after_shift) as nat);
    assert(signed(negative, after) * pow2(before_shift)
        == signed(negative, before) * pow2(after_shift)) by (nonlinear_arith)
        requires before == after * pow2((before_shift - after_shift) as nat),
            pow2(before_shift) == pow2(after_shift) * pow2((before_shift - after_shift) as nat);
}

/// Prove coprimality with a power-of-two denominator from an integer or odd numerator.
///
/// Requires either shift zero or an odd magnitude. At shift zero the denominator
/// is one, so every magnitude, including zero, is reduced; at positive shift the
/// odd-power-of-two GCD lemma applies. This concerns unsigned fraction reduction,
/// not the carrier's activity or sign conventions.
pub(crate) proof fn canonical_is_reduced(magnitude: nat, shift: nat)
    requires shift == 0 || magnitude % 2 == 1,
    ensures gcd(magnitude, pow2(shift)) == 1,
{
    assert(magnitude % 1 == 0);
    assert(gcd(1, 0) == 1);
    assert(gcd(magnitude, 1) == 1);
    if shift == 0 {
        lemma2_to64();
    } else {
        super::gcd::gcd_odd_power_two(magnitude, 1, shift);
        assert(1nat * pow2(shift) == pow2(shift)) by (nonlinear_arith);
    }
}
}

/// Subtract two same-width unsigned magnitudes, returning sign and nonzero absolute value.
///
/// Both arrays use numerical little-endian base-2^64 digits and may contain high
/// zero padding. Compares from the high end, then subtracts the smaller from the
/// larger in an owned buffer; the absolute difference always fits the same N limbs.
/// The returned Boolean is true exactly when positive is smaller than negative.
///
/// None means exact equality, including two empty arrays, and has no overflow or
/// resource-exhaustion meaning. Some always has nonzero magnitude. No denominator
/// reduction occurs here. The input arrays are passed by value, with at most N
/// comparison steps and, for unequal values, N limb subtraction steps.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures result.is_none() <==> value(positive@) == value(negative@),
        match result {
            Some((minus, magnitude)) => value(magnitude@) > 0
                && minus == (value(positive@) < value(negative@))
                && signed(minus, value(magnitude@)) == value(positive@) as int - value(negative@) as int,
            None => true,
        },
))]
#[inline]
pub(crate) fn difference<const N: usize>(
    positive: [u64; N],
    negative: [u64; N],
) -> Option<(bool, [u64; N])> {
    match limbs::compare(&positive, &negative) {
        Ordering::Equal => None,
        Ordering::Greater => {
            let mut magnitude = positive;
            limbs::subtract(&mut magnitude, &negative);
            Some((false, magnitude))
        }
        Ordering::Less => {
            let mut magnitude = negative;
            limbs::subtract(&mut magnitude, &positive);
            Some((true, magnitude))
        }
    }
}

/// Reduce a nonzero limb magnitude and its power-of-two denominator in place.
///
/// Requires `N <= u32::MAX / 64` and a positive represented magnitude. Counts the
/// numerator's trailing binary zeros, removes the lesser of that count and
/// denominator_shift, and returns the remaining denominator exponent. The buffer
/// retains the exact positive quotient with high zero padding as needed. The
/// fraction is unchanged and reduced: either the returned exponent is zero or
/// the numerator is odd. With denominator_shift zero the magnitude is unchanged.
///
/// These preconditions are not runtime validation. Zero, including an empty
/// array, is outside this function's contract and is not canonicalized to a zero
/// carrier; finish filters equality before calling it. The dimension bound keeps
/// bit counts and shift conversion representable rather than limiting total work.
/// The scan and optional in-place shift need no heap allocation or materialized
/// power of two, even when the supplied exponent is u64::MAX.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires N <= u32::MAX / 64, value(magnitude@) > 0,
    ensures result <= denominator_shift,
        value(final(magnitude)@) > 0,
        value(old(magnitude)@)
            == value(final(magnitude)@) * pow2((denominator_shift - result) as nat),
        result == 0 || value(final(magnitude)@) % 2 == 1,
        gcd(value(final(magnitude)@), pow2(result as nat)) == 1,
))]
#[inline]
pub(crate) fn normalize<const N: usize>(magnitude: &mut [u64; N], denominator_shift: u64) -> u64 {
    proof_decl! { let ghost original = magnitude@; }
    let trailing = limbs::trailing_zeros(magnitude);
    let common = (trailing as u64).min(denominator_shift);
    proof! {
        cancel_power_two(value(original), trailing as nat, common as nat);
        if common == 0 {
            lemma2_to64();
            lemma_div_basics(value(original) as int);
        }
    }
    if common != 0 {
        limbs::shift_right(magnitude, common as u32);
    }
    let reduced_shift = denominator_shift - common;
    proof! { canonical_is_reduced(value(magnitude@), reduced_shift as nat); }
    reduced_shift
}

/// Subtract unsigned subtotals at a common binary scale and reduce a nonzero result.
///
/// Requires `N <= u32::MAX / 64`, without runtime dimension validation. The inputs
/// are numerical little-endian magnitudes over the same `2^denominator_shift`.
/// First computes their signed difference; None means exactly equal subtotals,
/// including empty arrays. It does not report capacity decline or exhaustion.
///
/// For unequal subtotals, returns `(negative, magnitude, reduced_shift)`, with a
/// positive absolute numerator, exact sign and no larger exponent. Removes all
/// powers of two shared by that numerator and denominator, leaving an integer or
/// an odd numerator. Consumers map the equality sentinel to their canonical zero;
/// this function itself returns no zero tuple. Owned array arithmetic and exponent
/// metadata avoid allocating either a large integer denominator or a fraction.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires N <= u32::MAX / 64,
    ensures result.is_none() <==> value(positive@) == value(negative@),
        match result {
            Some((minus, magnitude, reduced_shift)) => reduced_shift <= denominator_shift
                && minus == (value(positive@) < value(negative@))
                && value(magnitude@) > 0
                && (reduced_shift == 0 || value(magnitude@) % 2 == 1)
                && gcd(value(magnitude@), pow2(reduced_shift as nat)) == 1
                && value(magnitude@) * pow2((denominator_shift - reduced_shift) as nat)
                    == abs(value(positive@) as int - value(negative@) as int)
                && signed(minus, value(magnitude@)) * pow2(denominator_shift as nat)
                    == (value(positive@) as int - value(negative@) as int) * pow2(reduced_shift as nat),
            None => true,
        },
))]
#[inline]
pub(crate) fn finish<const N: usize>(
    positive: [u64; N],
    negative: [u64; N],
    denominator_shift: u64,
) -> Option<(bool, [u64; N], u64)> {
    let (minus, mut magnitude) = difference(positive, negative)?;
    proof_decl! { let ghost original = magnitude@; }
    let reduced_shift = normalize(&mut magnitude, denominator_shift);
    proof! {
        reduction_preserves_fraction(minus, value(original), value(magnitude@),
            denominator_shift as nat, reduced_shift as nat);
    }
    Some((minus, magnitude, reduced_shift))
}

/// Produce a canonical native dyadic carrier from sign, magnitude and denominator exponent.
///
/// Accepts every Boolean sign, u128 magnitude and u64 exponent. Zero returns an
/// inactive, nonnegative carrier with zero magnitude and shift. Otherwise removes
/// `min(trailing_zeros(magnitude), denominator_shift)` factors of two from both
/// numerator and denominator, preserving the sign and setting active=true.
///
/// The nonzero removed count is at most 127, so the native right shift retains
/// the exact positive quotient. The result is reduced and denotes the original
/// fraction; applying normalization again preserves all fields. No errors, heap
/// allocation or giant power-of-two construction are needed, even at u64::MAX.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures canonical_word(result), result.shift <= denominator_shift,
        result.negative == (negative && magnitude != 0),
        magnitude == result.magnitude as nat * pow2((denominator_shift - result.shift) as nat),
        word_value(result) * pow2(denominator_shift as nat)
            == signed(negative, magnitude as nat) * pow2(result.shift as nat),
))]
#[inline]
pub(crate) fn normalize_word(negative: bool, magnitude: u128, denominator_shift: u64) -> Word {
    if magnitude == 0 {
        proof! {
            canonical_is_reduced(0, 0);
            lemma2_to64();
            assert(0nat * pow2(denominator_shift as nat) == 0nat) by (nonlinear_arith);
            assert(0int * pow2(denominator_shift as nat) == 0int) by (nonlinear_arith);
        }
        return Word {
            active: false,
            negative: false,
            magnitude: 0,
            shift: 0,
        };
    }
    let trailing = super::gcd::trailing_zeros_u128(magnitude);
    let common = (trailing as u64).min(denominator_shift) as u32;
    proof! {
        super::gcd::divide_trailing_factor(magnitude, common);
        super::gcd::trailing_zeros_factor_u128(magnitude);
    }
    let reduced = magnitude >> common;
    let shift = denominator_shift - common as u64;
    proof! {
        canonical_is_reduced(reduced as nat, shift as nat);
        reduction_preserves_fraction(negative, magnitude as nat, reduced as nat,
            denominator_shift as nat, shift as nat);
    }
    Word {
        active: true,
        negative,
        magnitude: reduced,
        shift,
    }
}

/// Attempt the exact difference of two raw dyadic carriers within native magnitude gates.
///
/// Accepts all raw Word fields without requiring canonical input. Uses the maximum
/// stored exponent, checks the left raw alignment, then the right raw alignment,
/// and only then applies activity. Either displacement at least 128 declines even
/// for zero or inactive inputs; either shifted raw magnitude exceeding u128::MAX
/// also declines. Later zeroing or cancellation cannot rescue a failed alignment.
///
/// After those gates, subtracts the activity-aware signed values using a symmetric
/// magnitude range from negative u128::MAX to positive u128::MAX, not i128.
/// Returns None when that signed result exceeds the range. Success is a canonical
/// Word after binary reduction; exact zero is Some with inactive, nonnegative,
/// zero-magnitude, zero-shift fields. Equal represented values can still decline
/// at an earlier raw gate.
///
/// Inputs are copied and unchanged. None is native-schedule ineligibility, unlike
/// the exact-zero sentinel returned by difference and finish. No wider fallback,
/// policy decision or resource-exhaustion report is produced by this helper.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures ({
        let scale = word_scale(left, right);
        let total = word_difference(left, right);
        &&& (result.is_none() <==> scale - left.shift >= 128 || scale - right.shift >= 128
            || left.magnitude as nat * pow2((scale - left.shift) as nat) > u128::MAX
            || right.magnitude as nat * pow2((scale - right.shift) as nat) > u128::MAX
            || total > u128::MAX || total < -(u128::MAX as int))
        &&& match result {
            Some(word) => canonical_word(word) && word.shift <= scale
                && word.negative == (total < 0)
                && word.magnitude as nat * pow2((scale - word.shift) as nat) == abs(total)
                && word_value(word) * pow2(scale) == total * pow2(word.shift as nat),
            None => true,
        }
    }),
))]
#[inline]
pub(crate) fn difference_word(left: Word, right: Word) -> Option<Word> {
    let scale = left.shift.max(right.shift);
    let left_scaled = super::word::checked_shift_left_u64(left.magnitude, scale - left.shift)?;
    let right_scaled = super::word::checked_shift_left_u64(right.magnitude, scale - right.shift)?;
    let left_magnitude = if left.active { left_scaled } else { 0 };
    let right_magnitude = if right.active { right_scaled } else { 0 };
    proof! {
        assert(super::word::signed(left.negative, left_magnitude) == aligned_word(left, scale as nat))
            by (nonlinear_arith)
            requires left_scaled == left.magnitude as nat * pow2((scale - left.shift) as nat),
                left_magnitude == (if left.active { left_scaled } else { 0u128 });
        assert(super::word::signed(!right.negative, right_magnitude) == -aligned_word(right, scale as nat))
            by (nonlinear_arith)
            requires right_scaled == right.magnitude as nat * pow2((scale - right.shift) as nat),
                right_magnitude == (if right.active { right_scaled } else { 0u128 });
    }
    let (negative, magnitude) = super::word::signed_add(
        left.negative,
        left_magnitude,
        !right.negative,
        right_magnitude,
    )?;
    Some(normalize_word(negative, magnitude, scale))
}
