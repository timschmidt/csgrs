#[cfg(verus_keep_ghost)]
use vstd::arithmetic::{div_mod::*, power2::*};
#[cfg(verus_keep_ghost)]
use vstd::prelude::*;
#[cfg(verus_keep_ghost)]
use vstd::std_specs::bits::u64_trailing_zeros;

use core::cmp::Ordering;

#[cfg(verus_keep_ghost)]
verus! {
pub(crate) open spec fn radix() -> nat { 0x1_0000_0000_0000_0000 }

pub(crate) open spec fn weight(index: nat) -> nat {
    pow2(64 * index)
}

/// Denotation of the first `length` little-endian 64-bit limbs.
pub(crate) open spec fn prefix(words: Seq<u64>, length: int) -> nat
    recommends 0 <= length <= words.len(),
    decreases length,
{
    if length <= 0 { 0 } else {
        prefix(words, length - 1) + (words[length - 1] as nat) * weight((length - 1) as nat)
    }
}

pub(crate) open spec fn value(words: Seq<u64>) -> nat {
    prefix(words, words.len() as int)
}

pub(crate) proof fn weight_step(index: nat)
    ensures weight(index) > 0, weight(index + 1) == weight(index) * radix(),
{
    lemma_pow2_pos(64 * index);
    lemma_pow2_adds(64 * index, 64);
    lemma2_to64();
}

pub(crate) proof fn weight_add(left: nat, right: nat)
    ensures weight(left + right) == weight(left) * weight(right),
{
    lemma_pow2_adds(64 * left, 64 * right);
}

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

/// Equal higher limbs and unequal current limbs determine integer ordering,
/// regardless of all lower limbs.
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

pub(crate) open spec fn limb(words: Seq<u64>, index: int) -> u64 {
    if 0 <= index < words.len() { words[index] } else { 0 }
}

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

spec fn word_shifted(words: Seq<u64>, shift: nat) -> Seq<u64> {
    Seq::new(words.len(), |index: int| limb(words, index + shift))
}

spec fn shifted_limb(words: Seq<u64>, index: int, shift: u32) -> u64 {
    if shift == 0 { limb(words, index) }
    else { (limb(words, index) >> shift) | (limb(words, index + 1) << (64 - shift)) }
}

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

pub(crate) open spec fn word_left_shifted(words: Seq<u64>, shift: nat) -> Seq<u64> {
    Seq::new(words.len(), |index: int| limb(words, index - shift))
}

pub(crate) open spec fn left_shifted_limb(words: Seq<u64>, index: int, shift: u32) -> u64 {
    if shift == 0 { limb(words, index) }
    else { (limb(words, index) << shift) | (limb(words, index - 1) >> (64 - shift)) }
}

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

/// Subtract a no-larger integer in place, carrying borrow across every limb.
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

/// Shift in place from low limbs upward, so unread source limbs survive.
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

/// Zero has the full buffer width as its count; every nonzero value factors
/// into exactly this power of two times a positive odd integer.
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

/// Restore a power-of-two factor without discarding any significant bits.
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

/// Copy into a different limb width, rejecting exactly the values that do
/// not fit. Larger output buffers are filled with zero above the input.
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
