//! Direct binary alignment and checked accumulation for dyadic products.

#[cfg(verus_keep_ghost)]
use super::limbs::*;
#[cfg(verus_keep_ghost)]
use vstd::arithmetic::{div_mod::*, power2::*};
#[cfg(verus_keep_ghost)]
use vstd::prelude::*;

#[cfg(verus_keep_ghost)]
verus! {
pub(crate) open spec fn aligned(source: Seq<u64>, word_shift: nat, bit_shift: u32, length: nat) -> Seq<u64> {
    Seq::new(length, |index: int| left_shifted_limb(source, index - word_shift, bit_shift))
}

proof fn weight_monotone(lower: nat, higher: nat)
    requires lower <= higher,
    ensures weight(lower) <= weight(higher),
{
    if lower < higher { lemma_pow2_strictly_increases(64 * lower, 64 * higher); }
}

/// The highest retained digit, including a possible shifted-out high part,
/// determines exactly the first buffer width that can hold the value.
pub(crate) proof fn shifted_extent(source: Seq<u64>, last: int, shift: u64)
    requires 0 <= last < source.len(), source[last] > 0,
        forall|index: int| last < index < source.len() ==> #[trigger] source[index] == 0,
    ensures ({
        let bits = (shift % 64) as u32;
        let high = bits > 0 && source[last] >> ((64 - bits) as u32) != 0;
        let end = last + (if high { 1int } else { 0int }) + shift / 64;
        weight(end as nat) <= value(source) * pow2(shift as nat) < weight((end + 1) as nat)
    }),
{
    extend_prefix_zero(source, last + 1, source.len() as int);
    prefix_bound(source, last);
    prefix_bound(source, last + 1);
    let bits = (shift % 64) as u32;
    let words = (shift / 64) as nat;
    let factor = pow2(bits as nat);
    let other = pow2((64 - bits) as nat);
    let high = bits > 0 && source[last] >> ((64 - bits) as u32) != 0;
    let top = source[last] as nat;
    let low = prefix(source, last);
    let w = weight(last as nat);
    lemma2_to64();
    weight_step(last as nat);
    weight_step(last as nat + 1);
    lemma_pow2_pos(bits as nat);
    lemma_pow2_pos((64 - bits) as nat);
    lemma_pow2_adds(bits as nat, (64 - bits) as nat);
    lemma_pow2_strictly_increases(bits as nat, 64);
    if bits > 0 {
        vstd::bits::lemma_u64_shr_is_div(source[last], (64 - bits) as u64);
        lemma_fundamental_div_mod(top as int, other as int);
    }
    assert(value(source) == low + top * w);
    if high {
        if top < other { lemma_basic_div(top as int, other as int); }
        assert(top * factor >= radix()) by (nonlinear_arith)
            requires top >= other, factor > 0, other * factor == radix();
        assert(weight(last as nat + 1) <= value(source) * factor < weight(last as nat + 2))
            by (nonlinear_arith)
            requires value(source) == low + top * w, low >= 0, w > 0,
                top * factor >= radix(), 0 < factor < radix(),
                value(source) < weight(last as nat + 1),
                weight(last as nat + 1) == w * radix(),
                weight(last as nat + 2) == weight(last as nat + 1) * radix();
    } else {
        assert(top < other);
        assert(w <= value(source) * factor < weight(last as nat + 1)) by (nonlinear_arith)
            requires value(source) == low + top * w, 0 <= low < w, w > 0,
                0 < top < other, factor >= 1, other * factor == radix(),
                weight(last as nat + 1) == w * radix();
    }
    let offset = (last + if high { 1int } else { 0int }) as nat;
    lemma_fundamental_div_mod(shift as int, 64);
    lemma_pow2_adds(64 * words, bits as nat);
    weight_step(words);
    weight_add(offset, words);
    weight_add(offset + 1, words);
    assert(weight(offset + words) <= value(source) * pow2(shift as nat)
        < weight(offset + words + 1)) by (nonlinear_arith)
        requires weight(offset) <= value(source) * factor < weight(offset + 1),
            weight(words) > 0, pow2(shift as nat) == weight(words) * factor,
            weight(offset + words) == weight(offset) * weight(words),
            weight(offset + words + 1) == weight(offset + 1) * weight(words);
}

pub(crate) proof fn aligned_value(source: Seq<u64>, last: int, shift: u64, length: nat)
    requires 0 <= last < source.len(), source[last] > 0,
        forall|index: int| last < index < source.len() ==> #[trigger] source[index] == 0,
        ({ let bits = (shift % 64) as u32;
           last + (if bits > 0 && source[last] >> ((64 - bits) as u32) != 0 { 1int } else { 0int })
               + shift / 64 < length }),
    ensures value(aligned(source, (shift / 64) as nat, (shift % 64) as u32, length))
        == value(source) * pow2(shift as nat),
{
    let words = (shift / 64) as nat;
    let bits = (shift % 64) as u32;
    let high = bits > 0 && source[last] >> ((64 - bits) as u32) != 0;
    let end = (last + if high { 1int } else { 0int }) as nat + words;
    let padded = Seq::new(length, |index: int| limb(source, index));
    prefix_equal(padded, source, last + 1);
    extend_prefix_zero(source, last + 1, source.len() as int);
    extend_prefix_zero(padded, last + 1, length as int);
    extend_prefix_zero(padded, (length - words) as int, length as int);
    let moved = word_left_shifted(padded, words);
    word_left_shift_value(padded, words);
    let output = aligned(source, words, bits, length);
    let factor = pow2(bits as nat);
    let other = pow2((64 - bits) as nat);
    shifted_extent(source, last, shift);
    weight_monotone(end + 1, length);
    lemma_fundamental_div_mod(shift as int, 64);
    lemma_pow2_adds(64 * words, bits as nat);
    lemma_pow2_pos(bits as nat);
    lemma_pow2_pos((64 - bits) as nat);
    assert(value(moved) * factor == value(source) * pow2(shift as nat)) by (nonlinear_arith)
        requires value(moved) == value(source) * weight(words),
            pow2(shift as nat) == weight(words) * factor;
    if bits == 0 {
        lemma2_to64();
        prefix_equal(output, moved, length as int);
        assert(value(moved) * factor == value(moved)) by (nonlinear_arith) requires factor == 1;
    } else {
        assert forall|index: int| 0 <= index < length implies
            #[trigger] output[index] + ((moved[index] as nat) / other) * radix()
                == moved[index] * factor + (limb(moved, index - 1) as nat) / other by {
            assert(limb(moved, index - 1) == limb(source, index - words - 1));
            left_shifted_digit_correct(moved[index], limb(moved, index - 1), bits);
        }
        left_shift_value(moved, output, factor, other);
    }
}

proof fn aligned_support(source: Seq<u64>, last: int, words: nat, bits: u32, length: nat)
    requires bits < 64, 0 <= last < source.len(),
        forall|index: int| last < index < source.len() ==> #[trigger] source[index] == 0,
    ensures
        forall|index: int| 0 <= index < words && index < length
            ==> #[trigger] aligned(source, words, bits, length)[index] == 0,
        forall|index: int| last + (if bits > 0 && source[last] >> ((64 - bits) as u32) != 0 { 1int } else { 0int })
                + words < index < length
            ==> #[trigger] aligned(source, words, bits, length)[index] == 0,
{
    let output = aligned(source, words, bits, length);
    let high = bits > 0 && source[last] >> ((64 - bits) as u32) != 0;
    assert forall|index: int| 0 <= index < words && index < length implies
        #[trigger] output[index] == 0 by {
        if bits > 0 {
            assert((0u64 << bits) | (0u64 >> ((64 - bits) as u32)) == 0) by (bit_vector);
        }
    }
    assert forall|index: int| last + (if high { 1int } else { 0int }) + words < index < length implies
        #[trigger] output[index] == 0 by {
        let offset = index - words;
        assert(limb(source, offset) == 0);
        if bits > 0 {
            let previous = limb(source, offset - 1);
            if offset != last + 1 {
                assert(previous == 0);
                assert(0u64 >> ((64 - bits) as u32) == 0) by (bit_vector);
            }
            assert(previous >> ((64 - bits) as u32) == 0);
            assert((0u64 << bits) | (previous >> ((64 - bits) as u32)) == 0) by (bit_vector)
                requires previous >> ((64 - bits) as u32) == 0;
        }
    }
}

proof fn addition_update(words: Seq<u64>, index: int, digit: u64, addend: u64, carry: bool, next: bool)
    requires 0 <= index < words.len(),
        digit as int + (if next { radix() as int } else { 0int })
            == words[index] as int + addend as int + (if carry { 1int } else { 0int }),
    ensures value(words.update(index, digit)) + (if next { weight(index as nat + 1) } else { 0nat })
        == value(words) + addend * weight(index as nat) + (if carry { weight(index as nat) } else { 0nat }),
{
    super::product::prefix_update(words, index, digit, words.len() as int);
    weight_step(index as nat);
    assert(value(words.update(index, digit)) + (if next { weight(index as nat + 1) } else { 0nat })
        == value(words) + addend * weight(index as nat) + (if carry { weight(index as nat) } else { 0nat }))
        by (nonlinear_arith)
        requires value(words.update(index, digit)) + words[index] * weight(index as nat)
                == value(words) + digit * weight(index as nat),
            digit as int + (if next { radix() as int } else { 0int })
                == words[index] as int + addend as int + (if carry { 1int } else { 0int }),
            weight(index as nat + 1) == weight(index as nat) * radix();
}
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures result.is_none() <==> value(words@) == 0,
        match result {
            Some(index) => index < N && words[index as int] > 0
                && forall|higher: int| index < higher < N ==> #[trigger] words[higher] == 0,
            None => true,
        },
))]
#[inline]
fn highest_nonzero<const N: usize>(words: &[u64; N]) -> Option<usize> {
    let mut index = N;
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant index <= N,
            forall|higher: int| index <= higher < N ==> #[trigger] words[higher] == 0,
        decreases index,
    ))]
    while index > 0 {
        index -= 1;
        if words[index] != 0 {
            proof! { prefix_zero(words@, N as int); }
            return Some(index);
        }
    }
    proof! { prefix_zero(words@, N as int); }
    None
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires bits < 64, index <= N,
    ensures result == left_shifted_limb(words@, index as int, bits),
))]
#[inline]
fn aligned_digit<const N: usize>(words: &[u64; N], index: usize, bits: u32) -> u64 {
    let current = if index < N { words[index] } else { 0 };
    let mut digit = current << bits;
    if bits != 0 && index != 0 {
        digit |= words[index - 1] >> (64 - bits);
    }
    proof! {
        if bits == 0 { assert(current << 0 == current) by (bit_vector); }
        if index == 0 && bits > 0 {
            assert(current << bits == (current << bits) | (0u64 >> ((64 - bits) as u32))) by (bit_vector);
        }
    }
    digit
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures result.0 as int + (if result.1 { radix() as int } else { 0int })
        == left as int + right as int + (if carry { 1int } else { 0int }),
))]
#[inline]
fn add_word(left: u64, right: u64, carry: bool) -> (u64, bool) {
    let first = left.wrapping_add(right);
    let first_carry = first < left;
    proof! {
        assert(first as int + (if first_carry { radix() as int } else { 0int }) == left as int + right as int);
        assert(first_carry ==> first < u64::MAX);
    }
    let carry_word = if carry { 1 } else { 0 };
    let sum = first.wrapping_add(carry_word);
    let second_carry = sum < first;
    proof! { assert(!(first_carry && second_carry)); }
    (sum, first_carry || second_carry)
}

/// Add a shifted product directly into the occupied target limbs, then
/// propagate carry only as far as needed. Failure means the exact sum does
/// not fit; as in the original accumulator, a late overflow may mutate limbs.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires N <= u32::MAX / 64, M <= u32::MAX / 64,
    ensures
        result.is_none() <==> value(old(accumulator)@) + value(product@) * pow2(shift as nat) >= weight(M as nat),
        result.is_some() ==> value(final(accumulator)@)
            == value(old(accumulator)@) + value(product@) * pow2(shift as nat),
        value(product@) * pow2(shift as nat) >= weight(M as nat)
            ==> final(accumulator)@ == old(accumulator)@,
        value(product@) * pow2(shift as nat) < weight(M as nat)
            ==> value(final(accumulator)@)
                == (value(old(accumulator)@) + value(product@) * pow2(shift as nat)) % weight(M as nat),
))]
#[inline]
pub(crate) fn add_shifted<const N: usize, const M: usize>(
    accumulator: &mut [u64; M],
    product: &[u64; N],
    shift: u64,
) -> Option<()> {
    proof_decl! { let ghost original = accumulator@; }
    let Some(last_source) = highest_nonzero(product) else {
        proof! {
            prefix_bound(accumulator@, M as int);
            assert(value(product@) * pow2(shift as nat) == 0) by (nonlinear_arith)
                requires value(product@) == 0;
            lemma_small_mod(value(original), weight(M as nat));
        }
        return Some(());
    };
    let word_shift = shift / 64;
    let bit_shift = (shift % 64) as u32;
    let has_high_limb = bit_shift != 0 && product[last_source] >> (64 - bit_shift) != 0;
    let last_offset = last_source + if has_high_limb { 1 } else { 0 };
    let last_target = word_shift + last_offset as u64;
    proof! { shifted_extent(product@, last_source as int, shift); }
    if last_target >= M as u64 {
        proof! { weight_monotone(M as nat, last_target as nat); }
        return None;
    }
    let word_shift = word_shift as usize;
    let last_target = last_target as usize;
    proof_decl! { let ghost aligned_words = aligned(product@, word_shift as nat, bit_shift, M as nat); }
    proof! {
        aligned_value(product@, last_source as int, shift, M as nat);
        aligned_support(product@, last_source as int, word_shift as nat, bit_shift, M as nat);
        prefix_zero(aligned_words, word_shift as int);
        extend_prefix_zero(aligned_words, last_target as int + 1, M as int);
        prefix_bound(aligned_words, M as int);
    }
    let mut carry = false;
    let mut offset = 0;
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant
            N <= u32::MAX / 64, M <= u32::MAX / 64,
            last_offset <= N, offset <= last_offset + 1,
            word_shift + last_offset == last_target, last_target < M, bit_shift < 64,
            original.len() == M, aligned_words.len() == M,
            original == old(accumulator)@,
            aligned_words == aligned(product@, word_shift as nat, bit_shift, M as nat),
            value(aligned_words) == value(product@) * pow2(shift as nat),
            value(product@) * pow2(shift as nat) < weight(M as nat),
            prefix(aligned_words, last_target as int + 1) == value(aligned_words),
            value(accumulator@) + (if carry { weight((word_shift + offset) as nat) } else { 0nat })
                == value(original) + prefix(aligned_words, (word_shift + offset) as int),
        decreases last_offset + 1 - offset,
    ))]
    while offset <= last_offset {
        let addend = aligned_digit(product, offset, bit_shift);
        let target = word_shift + offset;
        proof_decl! {
            let ghost before = accumulator@;
            let ghost incoming = carry;
        }
        let (sum, next_carry) = add_word(accumulator[target], addend, carry);
        accumulator[target] = sum;
        proof! {
            addition_update(before, target as int, sum, addend, incoming, next_carry);
            assert(addend == aligned_words[target as int]);
        }
        carry = next_carry;
        offset += 1;
    }
    let mut target = last_target + 1;
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant
            target <= M, M <= u32::MAX / 64, original.len() == M,
            original == old(accumulator)@,
            value(product@) * pow2(shift as nat) < weight(M as nat),
            value(accumulator@) + (if carry { weight(target as nat) } else { 0nat })
                == value(original) + value(product@) * pow2(shift as nat),
        decreases M - target,
    ))]
    while carry {
        if target == M {
            proof! {
                prefix_bound(accumulator@, M as int);
                lemma_fundamental_div_mod_converse_mod(
                    (value(original) + value(product@) * pow2(shift as nat)) as int,
                    weight(M as nat) as int, 1, value(accumulator@) as int);
            }
            return None;
        }
        proof_decl! { let ghost before = accumulator@; }
        let (sum, next_carry) = add_word(accumulator[target], 0, true);
        accumulator[target] = sum;
        proof! {
            addition_update(before, target as int, sum, 0, true, next_carry);
            assert(0nat * weight(target as nat) == 0) by (nonlinear_arith);
        }
        carry = next_carry;
        target += 1;
    }
    proof! {
        prefix_bound(accumulator@, M as int);
        lemma_small_mod(value(accumulator@), weight(M as nat));
    }
    Some(())
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires M <= u32::MAX / 64,
    ensures result.is_none() <==>
            value(old(accumulator)@) + left as nat * right as nat * pow2(shift as nat) >= weight(M as nat),
        result.is_some() ==> value(final(accumulator)@)
            == value(old(accumulator)@) + left as nat * right as nat * pow2(shift as nat),
        left as nat * right as nat * pow2(shift as nat) >= weight(M as nat)
            ==> final(accumulator)@ == old(accumulator)@,
        left as nat * right as nat * pow2(shift as nat) < weight(M as nat)
            ==> value(final(accumulator)@)
                == (value(old(accumulator)@) + left as nat * right as nat * pow2(shift as nat)) % weight(M as nat),
))]
#[inline]
pub(crate) fn add_product<const M: usize>(
    accumulator: &mut [u64; M],
    left: u128,
    right: u128,
    shift: u64,
) -> Option<()> {
    let product = super::product::multiply_u128(left, right);
    add_shifted(accumulator, &product, shift)
}

/// Retain the single-word scalar path while proving both product widths and
/// their subsequent alignment against one mathematical sum.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires M <= u32::MAX / 64,
    ensures result.is_none() <==>
            value(old(accumulator)@) + value(left@) * right as nat * pow2(shift as nat) >= weight(M as nat),
        result.is_some() ==> value(final(accumulator)@)
            == value(old(accumulator)@) + value(left@) * right as nat * pow2(shift as nat),
        value(left@) * right as nat * pow2(shift as nat) >= weight(M as nat)
            ==> final(accumulator)@ == old(accumulator)@,
        value(left@) * right as nat * pow2(shift as nat) < weight(M as nat)
            ==> value(final(accumulator)@)
                == (value(old(accumulator)@) + value(left@) * right as nat * pow2(shift as nat)) % weight(M as nat),
))]
#[inline]
pub(crate) fn add_wide_product<const M: usize>(
    accumulator: &mut [u64; M],
    left: &[u64; 4],
    right: u128,
    shift: u64,
) -> Option<()> {
    if right <= u64::MAX as u128 {
        let scalar = [right as u64];
        proof! {
            lemma2_to64();
            assert(scalar[0] == right);
            assert(prefix(scalar@, 0) == 0);
            assert(prefix(scalar@, 1) == prefix(scalar@, 0) + scalar[0] * weight(0));
            assert(value(scalar@) == right) by (nonlinear_arith)
                requires value(scalar@) == 0nat + right * weight(0), weight(0) == 1;
        }
        let product = super::product::multiply::<4, 1, 5>(left, &scalar);
        return add_shifted(accumulator, &product, shift);
    }
    let scalar = super::product::split_u128(right);
    let product = super::product::multiply::<4, 2, 6>(left, &scalar);
    add_shifted(accumulator, &product, shift)
}
