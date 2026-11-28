//! Unsigned fixed-limb arithmetic and its positional-value proof vocabulary.
//!
//! Arrays denote base-2^64 integers with the least significant digit first. This
//! is numerical limb order, independent of host byte order; signs, denominators
//! and canonical rational carriers belong to consumers. High zero padding is
//! permitted and empty arrays denote zero where the individual operation admits
//! them. Dyadic reduction, fixed-buffer GCD, products and accumulation share these
//! definitions instead of maintaining separate integer interpretations.
//!
//! Several runtime operations rely on caller-established preconditions. Subtraction
//! requires an ordered pair; exact left shift requires a fitting result; narrowing
//! to u128 requires at least two source limbs. These are not checked error APIs.
//! Right shift computes floor division and can discard low bits; exact factor
//! cancellation requires the additional divisibility supplied by its caller.
//!
//! Runtime bodies allocate no heap storage. Scans and in-place arithmetic scale
//! with array dimensions; resize returns a fixed output array. There is no local
//! total-work, stack or cancellation budget. Option declines in conversion mean
//! insufficient output capacity. Ghost sequences, powers and induction describe
//! the same arithmetic and allocate no runtime vectors or huge integers.

#[cfg(verus_keep_ghost)]
use vstd::arithmetic::{div_mod::*, power2::*};
#[cfg(verus_keep_ghost)]
use vstd::prelude::*;
#[cfg(verus_keep_ghost)]
use vstd::std_specs::bits::u64_trailing_zeros;

use core::cmp::Ordering;

#[cfg(verus_keep_ghost)]
verus! {
/// The mathematical digit base `2^64`, one larger than the greatest u64 digit.
///
/// Carries, borrows and positional-value lemmas use this natural-number constant;
/// it is not a representable u64 runtime value.
pub(crate) open spec fn radix() -> nat { 0x1_0000_0000_0000_0000 }

/// The positional multiplier `2^(64*index)` for a digit counted from the low end.
///
/// Index is an arbitrary mathematical natural number, with weight(0)=1. The
/// specification does not allocate this power or impose a runtime exponent bound.
pub(crate) open spec fn weight(index: nat) -> nat {
    pow2(64 * index)
}

/// Denote the integer stored in the first length least-significant digits.
///
/// The recommended domain is `0 <= length <= words.len()`. Recursively adds each
/// digit times its positional weight; the empty prefix is zero. Digits beyond
/// the prefix are irrelevant. Recommended bounds are specification assumptions,
/// not runtime input validation.
pub(crate) open spec fn prefix(words: Seq<u64>, length: int) -> nat
    recommends 0 <= length <= words.len(),
    decreases length,
{
    if length <= 0 { 0 } else {
        prefix(words, length - 1) + (words[length - 1] as nat) * weight((length - 1) as nat)
    }
}

/// Denote the whole unsigned integer represented by a finite limb sequence.
///
/// Uses prefix over the complete length, including high zero padding. Empty
/// sequences denote zero. This mathematical natural carries no sign or denominator.
pub(crate) open spec fn value(words: Seq<u64>) -> nat {
    prefix(words, words.len() as int)
}

/// Prove positivity of a positional weight and multiplication by the radix at the next index.
///
/// Combines power-of-two positivity and exponent addition. These facts justify
/// carry/borrow weights and strict bounds for every natural index.
pub(crate) proof fn weight_step(index: nat)
    ensures weight(index) > 0, weight(index + 1) == weight(index) * radix(),
{
    lemma_pow2_pos(64 * index);
    lemma_pow2_adds(64 * index, 64);
    lemma2_to64();
}

/// Factor a positional weight at a sum of indices into the two individual weights.
///
/// For any natural indices, exponent addition gives the identity used when a
/// subsequence is rebased or displaced by whole words.
pub(crate) proof fn weight_add(left: nat, right: nat)
    ensures weight(left + right) == weight(left) * weight(right),
{
    lemma_pow2_adds(64 * left, 64 * right);
}

/// Bound a valid length-digit unsigned prefix strictly below its next positional weight.
///
/// Requires `0 <= length <= words.len()`. Induction uses the u64 digit bound to
/// prove that the prefix fits in exactly that many available digits, including
/// the empty-prefix bound `0 < weight(0)`. This is not a minimal-width assertion.
pub(crate) proof fn prefix_bound(words: Seq<u64>, length: int)
    requires 0 <= length <= words.len(),
    ensures prefix(words, length) < weight(length as nat),
    decreases length,
{
    if length == 0 {
        lemma2_to64();
    } else {
        prefix_bound(words, length - 1);
        weight_step((length - 1) as nat);
        assert(prefix(words, length) < weight(length as nat)) by (nonlinear_arith)
            requires prefix(words, length)
                == prefix(words, length - 1) + words[length - 1] * weight((length - 1) as nat),
                prefix(words, length - 1) < weight((length - 1) as nat),
                0 <= words[length - 1] < radix(),
                weight((length - 1) as nat) > 0,
                weight(length as nat) == weight((length - 1) as nat) * radix();
    }
}

/// Transport digitwise equality of a shared prefix to equality of its integer values.
///
/// Requires nonnegative length within both sequences and equal digits below it.
/// The total sequence lengths and higher digits may differ. The lemma does not
/// assert equality of the complete integers unless those prefixes cover them.
pub(crate) proof fn prefix_equal(left: Seq<u64>, right: Seq<u64>, length: int)
    requires 0 <= length <= left.len(), length <= right.len(),
        forall|index: int| 0 <= index < length ==> #[trigger] left[index] == right[index],
    ensures prefix(left, length) == prefix(right, length),
    decreases length,
{
    if length > 0 {
        prefix_equal(left, right, length - 1);
    }
}

/// Derive whole-integer order from the highest differing digit of equal-length sequences.
///
/// Requires a valid index, a smaller left digit there and equality at every higher
/// index. Bounds on both lower prefixes show they cannot overturn that comparison;
/// then equal high digits extend it to the full values. Lower digits need not match.
proof fn prefix_less(left: Seq<u64>, right: Seq<u64>, index: int)
    requires 0 <= index < left.len(), left.len() == right.len(),
        left[index] < right[index],
        forall|higher: int| index < higher < left.len()
            ==> #[trigger] left[higher] == right[higher],
    ensures value(left) < value(right),
{
    prefix_bound(left, index);
    prefix_bound(right, index);
    weight_step(index as nat);
    assert(prefix(left, index + 1) < prefix(right, index + 1)) by (nonlinear_arith)
        requires 0 <= prefix(left, index) < weight(index as nat),
            0 <= prefix(right, index) < weight(index as nat),
            left[index] < right[index], weight(index as nat) > 0,
            prefix(left, index + 1) == prefix(left, index) + left[index] * weight(index as nat),
            prefix(right, index + 1) == prefix(right, index) + right[index] * weight(index as nat);
    extend_prefix_less(left, right, index + 1, left.len() as int);
}

/// Preserve an established strict prefix order while appending identical higher digits.
///
/// Requires equal sequence lengths, `0 <= start <= end <= left.len()`, strict
/// order at start and equal digits from start through end-1. Induction yields
/// strict order at end; digits beyond that endpoint need not agree.
proof fn extend_prefix_less(left: Seq<u64>, right: Seq<u64>, start: int, end: int)
    requires 0 <= start <= end <= left.len(), left.len() == right.len(),
        prefix(left, start) < prefix(right, start),
        forall|index: int| start <= index < end ==> #[trigger] left[index] == right[index],
    ensures prefix(left, end) < prefix(right, end),
    decreases end - start,
{
    if start < end {
        extend_prefix_less(left, right, start, end - 1);
    }
}

/// Split a prefix into its low part and a rebased higher subrange.
///
/// Requires `0 <= start <= end <= words.len()`. The length-end prefix equals the
/// length-start prefix plus `weight(start)` times the value of the subrange from
/// start to end. This ghost identity provides the place-value factor for shifts
/// and narrowing; it makes no runtime copy or claim of zero in either part.
pub(crate) proof fn prefix_split(words: Seq<u64>, start: int, end: int)
    requires 0 <= start <= end <= words.len(),
    ensures prefix(words, end) == prefix(words, start)
        + weight(start as nat) * value(words.subrange(start, end)),
    decreases end - start,
{
    if start == end {
        assert(value(words.subrange(start, end)) == 0);
        assert(weight(start as nat) * 0nat == 0) by (nonlinear_arith);
    } else {
        prefix_split(words, start, end - 1);
        let tail = words.subrange(start, end);
        let previous = words.subrange(start, end - 1);
        prefix_equal(tail, previous, end - start - 1);
        weight_add(start as nat, (end - start - 1) as nat);
        assert(prefix(words, end) == prefix(words, start)
            + weight(start as nat) * value(tail)) by (nonlinear_arith)
            requires
                prefix(words, end) == prefix(words, end - 1)
                    + words[end - 1] * weight((end - 1) as nat),
                prefix(words, end - 1) == prefix(words, start)
                    + weight(start as nat) * value(previous),
                value(tail) == value(previous)
                    + words[end - 1] * weight((end - start - 1) as nat),
                weight((end - 1) as nat)
                    == weight(start as nat) * weight((end - start - 1) as nat);
    }
}

/// Characterize a zero unsigned prefix by every included digit being zero.
///
/// Requires a valid nonnegative prefix length. Positive positional weights and
/// nonnegative digits rule out cancellation, including the vacuous empty case.
/// Digits outside the prefix are not constrained.
pub(crate) proof fn prefix_zero(words: Seq<u64>, length: int)
    requires 0 <= length <= words.len(),
    ensures prefix(words, length) == 0 <==>
        forall|index: int| 0 <= index < length ==> #[trigger] words[index] == 0,
    decreases length,
{
    if length > 0 {
        prefix_zero(words, length - 1);
        weight_step((length - 1) as nat);
        assert(prefix(words, length) == 0 <==>
            prefix(words, length - 1) == 0 && words[length - 1] == 0) by (nonlinear_arith)
            requires prefix(words, length) == prefix(words, length - 1)
                + words[length - 1] * weight((length - 1) as nat),
                prefix(words, length - 1) >= 0, words[length - 1] >= 0,
                weight((length - 1) as nat) > 0;
    }
}

/// Read a mathematical sequence digit with zero extension on both ends.
///
/// Every integer index is admitted: negative indices and indices at least the
/// sequence length yield zero. Shift formulas use this ghost accessor to express
/// boundary digits without runtime out-of-bounds access.
pub(crate) open spec fn limb(words: Seq<u64>, index: int) -> u64 {
    if 0 <= index < words.len() { words[index] } else { 0 }
}

/// Relate a right-shifted two-digit window to its input digits and discarded remainder.
///
/// Requires `0 < shift < 64`. The low digit shifted right and the high digit
/// shifted left occupy disjoint bit ranges. Multiplying their combined digit by
/// `2^shift` and restoring the low remainder equals the low input plus the low
/// bits of the high input at radix weight. This retains the information needed
/// for floor division; it does not assert that discarded bits were zero.
proof fn shifted_digit_correct(low: u64, high: u64, shift: u32)
    requires 0 < shift < 64,
    ensures
        (((low >> shift) | (high << (64 - shift))) as nat) * pow2(shift as nat)
            + (low as nat) % pow2(shift as nat)
            == low as nat + ((high as nat) % pow2(shift as nat)) * radix(),
{
    let factor = pow2(shift as nat);
    let other = pow2((64 - shift) as nat);
    lemma2_to64();
    lemma_pow2_pos(shift as nat);
    lemma_pow2_pos((64 - shift) as nat);
    lemma_pow2_adds(shift as nat, (64 - shift) as nat);
    vstd::bits::lemma_u64_pow2_no_overflow(shift as nat);
    super::word::one_shift_is_power_of_two(shift);
    assert((1u64 << shift) as u128 == (1u128 << shift)) by (bit_vector)
        requires shift < 64;
    vstd::bits::lemma_u64_low_bits_mask_is_mod(high, shift as nat);
    let bottom = high & ((factor - 1) as u64);
    assert(bottom == (high as nat) % factor);
    assert(bottom * other <= u64::MAX) by (nonlinear_arith)
        requires 0 <= bottom < factor, other > 0, factor * other == radix();
    vstd::bits::lemma_u64_shl_is_mul(bottom, (64 - shift) as u64);
    assert(high << ((64 - shift) as u32) == bottom << ((64 - shift) as u32)) by (bit_vector)
        requires 0 < shift < 64, bottom == high & (((1u64 << shift) - 1) as u64);
    vstd::bits::lemma_u64_shr_is_div(low, shift as u64);
    lemma_fundamental_div_mod(low as int, factor as int);
    let result = (low >> shift) | (high << ((64 - shift) as u32));
    assert(result as u128 == (low >> shift) as u128 + (high << ((64 - shift) as u32)) as u128)
        by (bit_vector)
        requires 0 < shift < 64,
            result == (low >> shift) | (high << ((64 - shift) as u32));
    assert(result * factor + (low as nat) % factor
        == low + ((high as nat) % factor) * radix()) by (nonlinear_arith)
        requires result == (low as nat) / factor + bottom * other,
            low == factor * ((low as nat) / factor) + (low as nat) % factor,
            bottom == (high as nat) % factor, factor * other == radix();
}

/// Extend a valid prefix through zero digits without changing its value.
///
/// Requires `0 <= start <= end <= words.len()` and zeros only in the appended
/// range. The prefix below start may be nonzero. This establishes equality of
/// the two prefix values, not that either prefix itself is zero.
pub(crate) proof fn extend_prefix_zero(words: Seq<u64>, start: int, end: int)
    requires 0 <= start <= end <= words.len(),
        forall|index: int| start <= index < end ==> #[trigger] words[index] == 0,
    ensures prefix(words, start) == prefix(words, end),
    decreases end - start,
{
    if start < end {
        extend_prefix_zero(words, start, end - 1);
        assert(words[end - 1] * weight((end - 1) as nat) == 0)
            by (nonlinear_arith) requires words[end - 1] == 0;
    }
}

/// Describe whole-word right displacement in a sequence of unchanged length.
///
/// Each output digit reads the source at index plus shift through zero extension.
/// Low source words are discarded and vacated high positions are zero; a shift
/// at least the length yields all zeros. This is a ghost construction, while
/// word_shift_value proves the numeric quotient under its narrower hypotheses.
spec fn word_shifted(words: Seq<u64>, shift: nat) -> Seq<u64> {
    Seq::new(words.len(), |index: int| limb(words, index + shift))
}

/// Describe one digit of an intra-word right shift using adjacent source digits.
///
/// The intended bit domain is `shift < 64`: zero returns limb at index, and a
/// positive shift combines that digit with low bits from index+1. Each source
/// lookup is independently zero-extended, so a negative window index can still
/// include an in-range neighbor. The definition has no explicit shift precondition;
/// callers establish the intended range before using its arithmetic meaning.
spec fn shifted_limb(words: Seq<u64>, index: int, shift: u32) -> u64 {
    if shift == 0 { limb(words, index) }
    else { (limb(words, index) >> shift) | (limb(words, index + 1) << (64 - shift)) }
}

/// Prove that whole-word right displacement gives division by the corresponding weight.
///
/// Requires `shift < words.len()`, so the source is nonempty. Splitting at shift
/// isolates the discarded low prefix, whose bound makes it the division remainder;
/// the retained tail and zero padding form the quotient. The sequence definition
/// also exists beyond this domain, but this lemma does not cover those cases.
proof fn word_shift_value(words: Seq<u64>, shift: nat)
    requires shift < words.len(),
    ensures value(word_shifted(words, shift)) == value(words) / weight(shift),
{
    let shifted = word_shifted(words, shift);
    let tail = words.subrange(shift as int, words.len() as int);
    prefix_equal(shifted, tail, tail.len() as int);
    extend_prefix_zero(shifted, tail.len() as int, shifted.len() as int);
    prefix_split(words, shift as int, words.len() as int);
    prefix_bound(words, shift as int);
    weight_step(shift);
    assert(value(words) == value(tail) * weight(shift) + prefix(words, shift as int))
        by (nonlinear_arith)
        requires value(words) == prefix(words, shift as int) + weight(shift) * value(tail);
    lemma_fundamental_div_mod_converse_div(value(words) as int, weight(shift) as int,
        value(tail) as int, prefix(words, shift as int) as int);
}

/// Combine local right-shift digit equations into a weighted prefix equation.
///
/// Requires a positive factor, equal source/output lengths, a valid prefix and
/// the stated digit equation throughout that prefix. Induction telescopes the
/// interior remainders, leaving the lowest source remainder and the next source
/// digit's remainder at the prefix boundary. No power-of-two assumption is added
/// beyond the local equations; a prefix boundary carry need not be zero.
proof fn shift_prefix_carry(source: Seq<u64>, output: Seq<u64>, factor: nat, length: int)
    requires factor > 0, source.len() == output.len(), 0 <= length <= source.len(),
        forall|index: int| 0 <= index < length ==>
            #[trigger] output[index] * factor + (source[index] as nat) % factor
                == source[index] + ((limb(source, index + 1) as nat) % factor) * radix(),
    ensures prefix(output, length) * factor + (limb(source, 0) as nat) % factor
        == prefix(source, length) + ((limb(source, length) as nat) % factor) * weight(length as nat),
    decreases length,
{
    if length == 0 {
        lemma2_to64();
        let remainder = (limb(source, 0) as nat) % factor;
        assert(prefix(output, 0) * factor == 0) by (nonlinear_arith)
            requires prefix(output, 0) == 0;
        assert(remainder * weight(0) == remainder) by (nonlinear_arith)
            requires weight(0) == 1;
    } else {
        shift_prefix_carry(source, output, factor, length - 1);
        weight_step((length - 1) as nat);
        let w = weight((length - 1) as nat);
        let next_w = weight(length as nat);
        let carry = (source[length - 1] as nat) % factor;
        let next_carry = (limb(source, length) as nat) % factor;
        let remainder = (limb(source, 0) as nat) % factor;
        let old_output = prefix(output, length - 1);
        let old_source = prefix(source, length - 1);
        assert(prefix(output, length) * factor + remainder
            == prefix(source, length) + next_carry * next_w) by (nonlinear_arith)
            requires old_output * factor + remainder == old_source + carry * w,
                output[length - 1] * factor + carry == source[length - 1] + next_carry * radix(),
                next_w == w * radix(),
                prefix(output, length) == old_output + output[length - 1] * w,
                prefix(source, length) == old_source + source[length - 1] * w;
    }
}

/// Turn full-length right-shift digit equations into an exact floor quotient.
///
/// Requires equal lengths, a positive factor and the local equation for every
/// digit. The carry at the high boundary is zero by extension, and the low
/// remainder is below factor, so value(output) equals value(source)/factor.
/// This is integer division; exact divisibility is not required or concluded.
proof fn shift_value(source: Seq<u64>, output: Seq<u64>, factor: nat)
    requires factor > 0, source.len() == output.len(),
        forall|index: int| 0 <= index < source.len() ==>
            #[trigger] output[index] * factor + (source[index] as nat) % factor
                == source[index] + ((limb(source, index + 1) as nat) % factor) * radix(),
    ensures value(output) == value(source) / factor,
{
    shift_prefix_carry(source, output, factor, source.len() as int);
    lemma_small_mod(0, factor);
    let carry = (limb(source, source.len() as int) as nat) % factor;
    assert(carry * weight(source.len()) == 0) by (nonlinear_arith) requires carry == 0;
    lemma_fundamental_div_mod_converse_div(value(source) as int, factor as int,
        value(output) as int, ((limb(source, 0) as nat) % factor) as int);
}

/// Extract the exact binary factor determined by the lowest nonzero digit.
///
/// Requires a valid positive digit at index and zero digits below it. The complete
/// value has `64*index + trailing_zeros(words[index])` trailing bits, fewer than
/// the sequence width, and dividing by that power of two gives a positive odd
/// integer. Higher digits are unrestricted; the proof shows they add an even
/// contribution to the quotient.
proof fn first_nonzero_factor(words: Seq<u64>, index: int)
    requires 0 <= index < words.len(), words[index] > 0,
        forall|lower: int| 0 <= lower < index ==> #[trigger] words[lower] == 0,
    ensures ({
        let shift = 64 * index as nat + u64_trailing_zeros(words[index]) as nat;
        shift < 64 * words.len()
            && value(words) / pow2(shift) > 0
            && (value(words) / pow2(shift)) % 2 == 1
            && value(words) == (value(words) / pow2(shift)) * pow2(shift)
    }),
{
    prefix_zero(words, index);
    prefix_split(words, index + 1, words.len() as int);
    weight_step(index as nat);
    let word = words[index];
    super::gcd::trailing_zeros_factor_u64(word);
    let shift = u64_trailing_zeros(word) as nat;
    let odd = (word >> u64_trailing_zeros(word)) as nat;
    let upper = value(words.subrange(index + 1, words.len() as int));
    let other = pow2((64 - shift) as nat);
    let quotient = odd + other * upper;
    let full_shift = 64 * index as nat + shift;
    lemma2_to64();
    lemma_pow2_pos(shift);
    lemma_pow2_pos(full_shift);
    lemma_pow2_pos((64 - shift) as nat);
    lemma_pow2_adds(shift, (64 - shift) as nat);
    lemma_pow2_adds(64 * index as nat, shift);
    assert(value(words) == quotient * pow2(full_shift)) by (nonlinear_arith)
        requires
            value(words) == prefix(words, index + 1) + weight(index as nat + 1) * upper,
            prefix(words, index + 1) == word * weight(index as nat),
            weight(index as nat + 1) == weight(index as nat) * radix(),
            word == odd * pow2(shift), radix() == pow2(shift) * other,
            quotient == odd + other * upper,
            pow2(full_shift) == weight(index as nat) * pow2(shift);
    assert(quotient > 0) by (nonlinear_arith)
        requires quotient == odd + other * upper, odd > 0, other > 0, upper >= 0;
    lemma_pow2_unfold((64 - shift) as nat);
    let half = pow2((63 - shift) as nat);
    assert(quotient == 2 * (half * upper) + odd) by (nonlinear_arith)
        requires quotient == odd + other * upper, other == 2 * half;
    lemma_mod_multiples_vanish((half * upper) as int, odd as int, 2);
    lemma_fundamental_div_mod_converse_div(value(words) as int, pow2(full_shift) as int,
        quotient as int, 0);
}

/// Describe whole-word left displacement within an unchanged sequence length.
///
/// Each output reads index minus shift through zero extension. Vacated low words
/// are zero and high source words may be discarded; a shift at least the length
/// yields zeros. This ghost sequence alone does not promise a lossless product.
pub(crate) open spec fn word_left_shifted(words: Seq<u64>, shift: nat) -> Seq<u64> {
    Seq::new(words.len(), |index: int| limb(words, index - shift))
}

/// Describe one digit of an intra-word left shift with a preceding source digit.
///
/// For the intended range `shift < 64`, zero returns limb at index; otherwise
/// the current low part combines with high bits from index-1. Both lookups are
/// zero-extended independently, so index equal to the source length can expose
/// a high spill digit. There is no explicit shift precondition in this definition;
/// callers establish the intended bit range and any required capacity bound.
pub(crate) open spec fn left_shifted_limb(words: Seq<u64>, index: int, shift: u32) -> u64 {
    if shift == 0 { limb(words, index) }
    else { (limb(words, index) << shift) | (limb(words, index - 1) >> (64 - shift)) }
}

/// Relate a whole-word left displacement to the retained low source prefix.
///
/// Requires `shift < words.len()`. The output value is the prefix of length
/// `words.len()-shift` times weight(shift), since the finite output drops higher
/// source words. Equality to the complete shifted source requires an additional
/// proof that those dropped digits are zero.
pub(crate) proof fn word_left_shift_value(words: Seq<u64>, shift: nat)
    requires shift < words.len(),
    ensures value(word_left_shifted(words, shift))
        == prefix(words, words.len() as int - shift) * weight(shift),
{
    let shifted = word_left_shifted(words, shift);
    let tail = shifted.subrange(shift as int, shifted.len() as int);
    prefix_zero(shifted, shift as int);
    prefix_split(shifted, shift as int, shifted.len() as int);
    prefix_equal(tail, words, tail.len() as int);
    assert(value(shifted) == prefix(words, words.len() as int - shift) * weight(shift))
        by (nonlinear_arith)
        requires value(shifted) == weight(shift) * value(tail),
            value(tail) == prefix(words, words.len() as int - shift);
}

/// Identify a short prefix with the full value when the full value fits its width.
///
/// Requires a valid prefix length and `value(words) < weight(length)`. Splitting
/// the sequence forces the higher subrange's nonnegative value to zero. Used to
/// justify that a whole-word left shift discards only zero padding.
proof fn prefix_is_value_below_bound(words: Seq<u64>, length: int)
    requires 0 <= length <= words.len(), value(words) < weight(length as nat),
    ensures prefix(words, length) == value(words),
{
    prefix_split(words, length, words.len() as int);
    weight_step(length as nat);
    let tail = value(words.subrange(length, words.len() as int));
    assert(tail == 0) by (nonlinear_arith)
        requires value(words) == prefix(words, length) + weight(length as nat) * tail,
            prefix(words, length) >= 0, tail >= 0, weight(length as nat) > 0,
            value(words) < weight(length as nat);
    assert(weight(length as nat) * tail == 0) by (nonlinear_arith) requires tail == 0;
}

/// Account for both the stored digit and outgoing carry of an intra-word left shift.
///
/// Requires `0 < shift < 64`. The combined current/previous digit plus the high
/// current carry at radix weight equals the scaled current digit plus the high
/// part entering from the previous digit. The proof reuses the complementary
/// right-shift relation; it does not assume the outgoing carry is zero.
pub(crate) proof fn left_shifted_digit_correct(current: u64, previous: u64, shift: u32)
    requires 0 < shift < 64,
    ensures
        ((current << shift) | (previous >> (64 - shift))) as nat
            + ((current as nat) / pow2((64 - shift) as nat)) * radix()
            == current * pow2(shift as nat) + (previous as nat) / pow2((64 - shift) as nat),
{
    shifted_digit_correct(previous, current, (64 - shift) as u32);
    let factor = pow2(shift as nat);
    let other = pow2((64 - shift) as nat);
    lemma2_to64();
    lemma_pow2_pos((64 - shift) as nat);
    lemma_pow2_adds(shift as nat, (64 - shift) as nat);
    lemma_fundamental_div_mod(current as int, other as int);
    lemma_fundamental_div_mod(previous as int, other as int);
    let result = (current << shift) | (previous >> ((64 - shift) as u32));
    assert(result == (previous >> ((64 - shift) as u32)) | (current << shift)) by (bit_vector)
        requires result == (current << shift) | (previous >> ((64 - shift) as u32));
    let high = (current as nat) / other;
    let low = (previous as nat) / other;
    assert(result + high * radix() == current * factor + low) by (nonlinear_arith)
        requires
            result * other + (previous as nat) % other
                == previous + ((current as nat) % other) * radix(),
            current == other * high + (current as nat) % other,
            previous == other * low + (previous as nat) % other,
            radix() == factor * other, other > 0;
}

/// Combine local left-shift equations while retaining the high prefix carry.
///
/// Requires positive factor and other, equal sequence lengths, a valid prefix
/// and the stated per-digit equations. Induction proves that the output prefix
/// plus its high carry at weight(length) equals the scaled input prefix. The low
/// incoming carry vanishes by zero extension. Neither a capacity bound nor the
/// relation `factor*other == radix()` is independently required by this lemma.
proof fn left_shift_prefix_carry(source: Seq<u64>, output: Seq<u64>, factor: nat, other: nat, length: int)
    requires factor > 0, other > 0, source.len() == output.len(), 0 <= length <= source.len(),
        forall|index: int| 0 <= index < length ==>
            #[trigger] output[index] + ((source[index] as nat) / other) * radix()
                == source[index] * factor + (limb(source, index - 1) as nat) / other,
    ensures prefix(output, length) + ((limb(source, length - 1) as nat) / other) * weight(length as nat)
        == prefix(source, length) * factor,
    decreases length,
{
    if length == 0 {
        lemma_div_of0(other as int);
        let carry = (limb(source, -1) as nat) / other;
        assert(carry * weight(0) == 0) by (nonlinear_arith) requires carry == 0;
        assert(prefix(source, 0) * factor == 0) by (nonlinear_arith) requires prefix(source, 0) == 0;
    } else {
        left_shift_prefix_carry(source, output, factor, other, length - 1);
        weight_step((length - 1) as nat);
        let w = weight((length - 1) as nat);
        let next_w = weight(length as nat);
        let carry = (limb(source, length - 2) as nat) / other;
        let next_carry = (source[length - 1] as nat) / other;
        let old_output = prefix(output, length - 1);
        let old_source = prefix(source, length - 1);
        assert(prefix(output, length) + next_carry * next_w
            == prefix(source, length) * factor) by (nonlinear_arith)
            requires old_output + carry * w == old_source * factor,
                output[length - 1] + next_carry * radix() == source[length - 1] * factor + carry,
                next_w == w * radix(),
                prefix(output, length) == old_output + output[length - 1] * w,
                prefix(source, length) == old_source + source[length - 1] * w;
    }
}

/// Prove a complete scaled value from left-shift digit equations and a capacity bound.
///
/// Requires positive factor and other, equal lengths, every local digit equation,
/// and `value(source)*factor < weight(source.len())`. The prefix identity then
/// forces the outgoing high carry to zero, leaving exact multiplication in the
/// output. Without the supplied capacity hypothesis, the prefix lemma retains
/// a carry and cannot establish lossless fixed-width scaling.
pub(crate) proof fn left_shift_value(source: Seq<u64>, output: Seq<u64>, factor: nat, other: nat)
    requires factor > 0, other > 0, source.len() == output.len(),
        value(source) * factor < weight(source.len()),
        forall|index: int| 0 <= index < source.len() ==>
            #[trigger] output[index] + ((source[index] as nat) / other) * radix()
                == source[index] * factor + (limb(source, index - 1) as nat) / other,
    ensures value(output) == value(source) * factor,
{
    left_shift_prefix_carry(source, output, factor, other, source.len() as int);
    let carry = (limb(source, source.len() as int - 1) as nat) / other;
    weight_step(source.len());
    assert(carry == 0) by (nonlinear_arith)
        requires value(output) + carry * weight(source.len()) == value(source) * factor,
            value(source) * factor < weight(source.len()), value(output) >= 0,
            carry >= 0, weight(source.len()) > 0;
    assert(carry * weight(source.len()) == 0) by (nonlinear_arith) requires carry == 0;
}

/// Identify the two low digits with their packed u128 value and its exclusive capacity bound.
///
/// Requires at least two source digits. Their shifted bit ranges are disjoint,
/// so bitwise OR equals the low digit plus the high digit times the radix.
/// Also establishes `weight(2) == u128::MAX + 1` in mathematical naturals.
/// Higher digits are not constrained; consumers must check them before narrowing.
pub(crate) proof fn two_limbs_value(words: Seq<u64>)
    requires words.len() >= 2,
    ensures prefix(words, 2) == ((words[0] as u128) | ((words[1] as u128) << 64)),
        weight(2) == u128::MAX as nat + 1,
{
    lemma2_to64();
    weight_step(1);
    let low = words[0];
    let high = words[1];
    assert(prefix(words, 1) == prefix(words, 0) + low * weight(0));
    assert(prefix(words, 0) == 0);
    assert((low as u128 | ((high as u128) << 64))
        == low as u128 + high as u128 * 0x1_0000_0000_0000_0000u128) by (bit_vector);
    assert(prefix(words, 2) == low + high * radix()) by (nonlinear_arith)
        requires prefix(words, 2) == prefix(words, 1) + high * weight(1),
            prefix(words, 1) == low * weight(0), weight(0) == 1, weight(1) == radix();
}
}

/// Compare same-width unsigned limb values exactly, reading from the high end.
///
/// All arrays of the chosen N are admitted, including high zero padding and N=0.
/// Returns at the first unequal digit; two empty or digitwise-equal arrays compare
/// Equal. Reads at most N digit pairs without mutation, sign handling, allocation
/// or an undecided outcome.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures match result {
        Ordering::Less => value(left@) < value(right@),
        Ordering::Equal => value(left@) == value(right@),
        Ordering::Greater => value(left@) > value(right@),
    },
))]
#[inline]
pub(crate) fn compare<const N: usize>(left: &[u64; N], right: &[u64; N]) -> Ordering {
    let mut index = N;
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant index <= N,
            forall|higher: int| index <= higher < N
                ==> #[trigger] left@[higher] == right@[higher],
        decreases index,
    ))]
    while index != 0 {
        index -= 1;
        let ordering = left[index].cmp(&right[index]);
        if !matches!(ordering, Ordering::Equal) {
            proof! {
                if left[index as int] < right[index as int] {
                    prefix_less(left@, right@, index as int);
                } else {
                    prefix_less(right@, left@, index as int);
                }
            }
            return ordering;
        }
    }
    proof! { prefix_equal(left@, right@, N as int); }
    Ordering::Equal
}

/// Subtract a digit and one incoming borrow while retaining the outgoing borrow.
///
/// Accepts every u64 pair and Boolean borrow. For returned (digit, next),
/// `digit + right + borrow = left + next*2^64`, interpreting Booleans as zero/one.
/// Two wrapping subtractions detect the outgoing bit; their individual borrows
/// cannot both be set. The digit is the difference modulo the radix. This private
/// step retains underflow information for the multi-digit subtraction invariant.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures result.0 as int + right as int + (if borrow { 1int } else { 0int })
        == left as int + (if result.1 { radix() as int } else { 0int }),
))]
#[inline]
fn subtract_word(left: u64, right: u64, borrow: bool) -> (u64, bool) {
    let first_borrow = left < right;
    let first_difference = left.wrapping_sub(right);
    proof! {
        assert(first_difference as int + right as int
            == left as int + if first_borrow { radix() as int } else { 0int });
        assert(first_borrow ==> first_difference > 0);
    }
    let borrow_word = if borrow { 1 } else { 0 };
    let second_borrow = first_difference < borrow_word;
    let difference = first_difference.wrapping_sub(borrow_word);
    proof! { assert(!(first_borrow && second_borrow)); }
    (difference, first_borrow || second_borrow)
}

/// Subtract a no-larger unsigned magnitude in place, preserving the exact difference.
///
/// Requires the original left value to be at least right. Both arrays have N
/// numerical little-endian digits; empty arrays satisfy the condition and are a
/// no-op. Visits all N digits from low to high, propagating borrow, and leaves
/// right unchanged. The comparison in dyadic/GCD callers establishes the ordering.
///
/// There is no runtime ordering check or returned borrow. Violating the precondition
/// can silently wrap the buffer, so this is not a checked subtraction interface.
/// No sign, normalization, allocation or resource-budget outcome is supplied.
#[cfg_attr(verus_keep_ghost, verus_spec(
    requires value(left@) >= value(right@),
    ensures value(final(left)@) + value(right@) == value(old(left)@),
))]
#[inline]
pub(crate) fn subtract<const N: usize>(left: &mut [u64; N], right: &[u64; N]) {
    proof_decl! { let ghost original = left@; }
    let mut index = 0;
    let mut borrow = false;
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant
            index <= N, original.len() == N,
            value(original) >= value(right@),
            forall|higher: int| index <= higher < N
                ==> #[trigger] left@[higher] == original[higher],
            prefix(left@, index as int) + prefix(right@, index as int)
                == prefix(original, index as int)
                    + if borrow { weight(index as nat) } else { 0nat },
        decreases N - index,
    ))]
    while index < N {
        proof_decl! {
            let ghost before = left@;
            let ghost incoming = borrow;
        }
        let (difference, next_borrow) = subtract_word(left[index], right[index], borrow);
        left[index] = difference;
        proof! {
            prefix_equal(before, left@, index as int);
            weight_step(index as nat);
            let w = weight(index as nat);
            let next_w = weight(index as nat + 1);
            let out = prefix(left@, index as int);
            let sub = prefix(right@, index as int);
            let source = prefix(original, index as int);
            assert(prefix(left@, index as int + 1) + prefix(right@, index as int + 1)
                == prefix(original, index as int + 1)
                    + if next_borrow { next_w } else { 0nat }) by (nonlinear_arith)
                requires
                    out + sub == source + if incoming { w } else { 0nat },
                    difference as int + right[index as int] + (if incoming { 1int } else { 0int })
                        == original[index as int] + if next_borrow { radix() as int } else { 0int },
                    next_w == w * radix(),
                    prefix(left@, index as int + 1) == out + difference * w,
                    prefix(right@, index as int + 1) == sub + right[index as int] * w,
                    prefix(original, index as int + 1) == source + original[index as int] * w;
        }
        borrow = next_borrow;
        index += 1;
    }
    proof! {
        prefix_bound(left@, N as int);
        assert(!borrow);
    }
}

/// Replace a limb value by floor division by a power of two, in place.
///
/// Requires `N <= u32::MAX / 64` and `shift < 64*N`, excluding empty arrays.
/// Splits the shift into whole-word and intra-word parts, visits all N output
/// positions from low to high and zero-fills vacated high positions. This order
/// preserves unread source digits; the zero bit-shift branch avoids shifting a
/// u64 by 64. Shift zero retains the input.
///
/// Low discarded bits need not be zero: callers using this for exact factor
/// cancellation must establish divisibility separately. Preconditions are not
/// validated as errors, and behavior beyond them is not an extension contract.
/// There is no allocation, returned remainder or overflow/underflow Option.
#[cfg_attr(verus_keep_ghost, verus_spec(
    requires N <= u32::MAX / 64, shift < 64 * N,
    ensures value(final(words)@) == value(old(words)@) / pow2(shift as nat),
))]
#[inline]
pub(crate) fn shift_right<const N: usize>(words: &mut [u64; N], shift: u32) {
    proof_decl! { let ghost original = words@; }
    let word_shift = (shift / 64) as usize;
    let bit_shift = shift % 64;
    let mut index = 0;
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant
            index <= N, N <= u32::MAX / 64,
            word_shift < N, bit_shift < 64,
            original.len() == N,
            forall|done: int| 0 <= done < index ==> #[trigger] words@[done]
                == shifted_limb(original, done + word_shift, bit_shift),
            forall|pending: int| index <= pending < N
                ==> #[trigger] words@[pending] == original[pending],
        decreases N - index,
    ))]
    while index < N {
        let source = index + word_shift;
        let shifted = if source >= N {
            0
        } else if bit_shift == 0 {
            words[source]
        } else {
            let high = if source + 1 < N { words[source + 1] } else { 0 };
            (words[source] >> bit_shift) | (high << (64 - bit_shift))
        };
        proof! {
            if source >= N && bit_shift > 0 {
                assert((0u64 >> bit_shift) | (0u64 << ((64 - bit_shift) as u32)) == 0)
                    by (bit_vector);
            }
            assert(shifted == shifted_limb(original, index as int + word_shift, bit_shift));
        }
        words[index] = shifted;
        index += 1;
    }
    proof! {
        let source = word_shifted(original, word_shift as nat);
        let factor = pow2(bit_shift as nat);
        lemma_pow2_pos(bit_shift as nat);
        if bit_shift == 0 {
            prefix_equal(words@, source, N as int);
            lemma2_to64();
            lemma_div_basics(value(source) as int);
        } else {
            assert forall|i: int| 0 <= i < source.len() implies
                #[trigger] words@[i] * factor + (source[i] as nat) % factor
                    == source[i] + ((limb(source, i + 1) as nat) % factor) * radix() by {
                shifted_digit_correct(limb(original, i + word_shift),
                    limb(original, i + word_shift + 1), bit_shift);
            }
            shift_value(source, words@, factor);
        }
        word_shift_value(original, word_shift as nat);
        weight_step(word_shift as nat);
        lemma_div_denominator(value(original) as int, weight(word_shift as nat) as int, factor as int);
        lemma_fundamental_div_mod(shift as int, 64);
        lemma_pow2_adds(64 * word_shift as nat, bit_shift as nat);
    }
}

/// Count low zero bits, using the full buffer width as the zero-value sentinel.
///
/// Requires `N <= u32::MAX / 64` so the bit count fits u32; no runtime dimension
/// validation occurs. Scans from the low end and combines the first nonzero digit's
/// bit count with its word displacement. For nonzero input the count is below
/// `64*N`, and removing that factor yields a positive odd integer. Zero returns
/// `64*N`, including zero for an empty array. No digits are changed or allocated.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires N <= u32::MAX / 64,
    ensures
        result <= 64 * N,
        (result == 64 * N <==> value(words@) == 0),
        value(words@) > 0 ==> (
            value(words@) / pow2(result as nat) > 0
            && (value(words@) / pow2(result as nat)) % 2 == 1
            && value(words@) == (value(words@) / pow2(result as nat)) * pow2(result as nat)),
))]
#[inline]
pub(crate) fn trailing_zeros<const N: usize>(words: &[u64; N]) -> u32 {
    let mut index = 0;
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant index <= N, N <= u32::MAX / 64,
            forall|lower: int| 0 <= lower < index ==> #[trigger] words@[lower] == 0,
        decreases N - index,
    ))]
    while index < N {
        let word = words[index];
        if word != 0 {
            proof! {
                prefix_zero(words@, N as int);
                first_nonzero_factor(words@, index as int);
            }
            return index as u32 * 64 + word.trailing_zeros();
        }
        index += 1;
    }
    proof! { prefix_zero(words@, N as int); }
    N as u32 * 64
}

/// Restore a binary factor in place when the complete scaled value fits the buffer.
///
/// Requires `N <= u32::MAX / 64`, `shift < 64*N` and
/// `value(words)*2^shift < 2^(64*N)`. Thus empty arrays are outside the domain;
/// zero is admitted for every in-range shift. Visits all N digits from high to
/// low, preserving source digits not yet consumed, and zero-fills low positions.
/// The intra-word zero branch avoids a u64 shift by 64. Shift zero is identity.
///
/// Capacity and range are caller obligations, not runtime checks. An overwide
/// scaled input can lose high bits silently rather than return an error. The GCD
/// consumer proves the restored factor fits before using this exact path. No
/// allocation, saturation or resource-exhaustion result occurs here.
#[cfg_attr(verus_keep_ghost, verus_spec(
    requires N <= u32::MAX / 64, shift < 64 * N,
        value(words@) * pow2(shift as nat) < weight(N as nat),
    ensures value(final(words)@) == value(old(words)@) * pow2(shift as nat),
))]
#[inline]
pub(crate) fn shift_left<const N: usize>(words: &mut [u64; N], shift: u32) {
    proof_decl! { let ghost original = words@; }
    let word_shift = (shift / 64) as usize;
    let bit_shift = shift % 64;
    let mut index = N;
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant
            index <= N, N <= u32::MAX / 64,
            word_shift < N, bit_shift < 64,
            original.len() == N,
            value(original) * pow2(shift as nat) < weight(N as nat),
            forall|done: int| index <= done < N ==> #[trigger] words@[done]
                == left_shifted_limb(original, done - word_shift, bit_shift),
            forall|pending: int| 0 <= pending < index
                ==> #[trigger] words@[pending] == original[pending],
        decreases index,
    ))]
    while index != 0 {
        index -= 1;
        let shifted = if index < word_shift {
            0
        } else if bit_shift == 0 {
            words[index - word_shift]
        } else {
            let source = index - word_shift;
            let previous = if source > 0 { words[source - 1] } else { 0 };
            (words[source] << bit_shift) | (previous >> (64 - bit_shift))
        };
        proof! {
            if index < word_shift && bit_shift > 0 {
                assert((0u64 << bit_shift) | (0u64 >> ((64 - bit_shift) as u32)) == 0)
                    by (bit_vector);
            }
            assert(shifted == left_shifted_limb(original, index as int - word_shift, bit_shift));
        }
        words[index] = shifted;
    }
    proof! {
        let source = word_left_shifted(original, word_shift as nat);
        let factor = pow2(bit_shift as nat);
        let total = pow2(shift as nat);
        let low_weight = weight((N - word_shift) as nat);
        let high_weight = weight(word_shift as nat);
        lemma_pow2_pos(bit_shift as nat);
        weight_step(word_shift as nat);
        weight_add((N - word_shift) as nat, word_shift as nat);
        lemma_fundamental_div_mod(shift as int, 64);
        lemma_pow2_adds(64 * word_shift as nat, bit_shift as nat);
        assert(value(original) < low_weight) by (nonlinear_arith)
            requires value(original) * total < weight(N as nat),
                total == high_weight * factor, factor >= 1, high_weight > 0,
                weight(N as nat) == low_weight * high_weight, value(original) >= 0;
        prefix_is_value_below_bound(original, N as int - word_shift);
        word_left_shift_value(original, word_shift as nat);
        assert(value(source) * factor == value(original) * total) by (nonlinear_arith)
            requires value(source) == value(original) * high_weight,
                total == high_weight * factor;
        if bit_shift == 0 {
            prefix_equal(words@, source, N as int);
            lemma2_to64();
            assert(value(source) * factor == value(source)) by (nonlinear_arith)
                requires factor == 1;
        } else {
            let other = pow2((64 - bit_shift) as nat);
            lemma_pow2_pos((64 - bit_shift) as nat);
            assert forall|i: int| 0 <= i < source.len() implies
                #[trigger] words@[i] + ((source[i] as nat) / other) * radix()
                    == source[i] * factor + (limb(source, i - 1) as nat) / other by {
                left_shifted_digit_correct(limb(original, i - word_shift),
                    limb(original, i - word_shift - 1), bit_shift);
            }
            left_shift_value(source, words@, factor, other);
        }
    }
}

/// Narrow exactly when every digit above the low two is zero.
///
/// Requires `N >= 2`. Scans higher digits in increasing index order and returns
/// None at the first nonzero one; otherwise packs the low two digits into u128.
/// For admitted dimensions, Some is the unchanged value, including zero, and
/// None means the unsigned value exceeds u128::MAX. Input storage is unchanged.
///
/// The minimum dimension is not checked as a typed error: N=0 or N=1 reaches
/// out-of-bounds indexing and panics, even for zero. This helper neither pads short
/// arrays nor truncates an overwide value; resize supplies a separate checked
/// change of limb width.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires N >= 2,
    ensures
        result.is_none() <==> value(words@) > u128::MAX,
        match result { Some(word) => word == value(words@), None => true },
))]
#[inline]
pub(crate) fn to_u128<const N: usize>(words: &[u64; N]) -> Option<u128> {
    proof! {
        two_limbs_value(words@);
        prefix_split(words@, 2, N as int);
    }
    let mut index = 2;
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant
            2 <= index <= N,
            forall|higher: int| 2 <= higher < index ==> #[trigger] words@[higher] == 0,
        decreases N - index,
    ))]
    while index < N {
        if words[index] != 0 {
            proof! {
                let upper = words@.subrange(2, N as int);
                prefix_zero(upper, upper.len() as int);
                assert(upper[index as int - 2] == words[index as int]);
                if value(upper) == 0 {
                    assert(upper[index as int - 2] == 0);
                }
                assert(value(upper) > 0);
                two_limbs_value(words@);
                prefix_split(words@, 2, N as int);
                assert(value(words@) > u128::MAX) by (nonlinear_arith)
                    requires value(words@) == prefix(words@, 2) + weight(2) * value(upper),
                        prefix(words@, 2) >= 0, value(upper) > 0,
                        weight(2) == u128::MAX as nat + 1;
            }
            return None;
        }
        index += 1;
    }
    proof! {
        extend_prefix_zero(words@, 2, N as int);
        two_limbs_value(words@);
    }
    Some(words[0] as u128 | (words[1] as u128) << 64)
}

/// Copy an unsigned value into O limbs exactly when it fits, with zero high padding.
///
/// Admits all source/output dimensions, including zero. When shrinking, checks
/// the discarded high source digits before creating output; any nonzero digit
/// returns None. Otherwise copies the retained low digits and fills added high
/// positions with zero. Some preserves the exact value and every retained digit;
/// None is precisely `value(words) >= 2^(64*O)`, not an exhausted resource budget.
///
/// An empty source grows to zeros. O=0 accepts only a zero value and returns an
/// empty array; equal dimensions preserve all digits. The source is read-only,
/// and the O-element output is fixed storage with no heap allocation in this body.
/// Dimensions determine scan/copy work and storage without a local work/stack cap.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures result.is_none() <==> value(words@) >= weight(O as nat),
        match result {
            Some(output) => value(output@) == value(words@)
                && forall|index: int| 0 <= index < O ==> #[trigger] output[index] == limb(words@, index),
            None => true,
        },
))]
#[inline]
pub(crate) fn resize<const N: usize, const O: usize>(words: &[u64; N]) -> Option<[u64; O]> {
    if O < N {
        let mut index = O;
        #[cfg_attr(verus_keep_ghost, verus_spec(
            invariant O <= index <= N,
                forall|higher: int| O <= higher < index ==> #[trigger] words[higher] == 0,
            decreases N - index,
        ))]
        while index < N {
            if words[index] != 0 {
                proof! {
                    prefix_split(words@, O as int, N as int);
                    let upper = words@.subrange(O as int, N as int);
                    prefix_zero(upper, upper.len() as int);
                    assert(upper[index as int - O] == words[index as int]);
                    assert(value(upper) > 0);
                    weight_step(O as nat);
                    assert(value(words@) >= weight(O as nat)) by (nonlinear_arith)
                        requires value(words@) == prefix(words@, O as int) + weight(O as nat) * value(upper),
                            prefix(words@, O as int) >= 0, value(upper) > 0, weight(O as nat) > 0;
                }
                return None;
            }
            index += 1;
        }
    }
    let mut output = [0_u64; O];
    let mut index = 0;
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant index <= O,
            forall|done: int| 0 <= done < index ==> #[trigger] output[done] == limb(words@, done),
            forall|higher: int| O <= higher < N ==> #[trigger] words[higher] == 0,
        decreases O - index,
    ))]
    while index < O {
        output[index] = if index < N { words[index] } else { 0 };
        index += 1;
    }
    proof! {
        if O <= N {
            prefix_equal(output@, words@, O as int);
            extend_prefix_zero(words@, O as int, N as int);
        } else {
            prefix_equal(output@, words@, N as int);
            extend_prefix_zero(output@, N as int, O as int);
        }
        prefix_bound(output@, O as int);
    }
    Some(output)
}
