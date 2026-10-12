//! Connect the verified limb denotation to mathematical bit lengths.
//! BigUint storage normalization and the runtime import still need refinement.
use crate::magnitude_model::has_bit_length;
use crate::verified::limbs::{prefix, prefix_bound, value, weight};
use vstd::arithmetic::div_mod::*;
use vstd::arithmetic::power2::*;
use vstd::prelude::*;
use vstd::std_specs::bits::*;

verus! {

/// The recursive standard-library specification determines the exact binade,
/// including a word whose top bit occupies the entire native width.
pub(crate) proof fn leading_zeros_determine_word_bit_length(word: u64)
    ensures
        0 <= u64_leading_zeros(word) <= 64,
        word == 0 <==> u64_leading_zeros(word) == 64,
        word > 0 ==> has_bit_length(word as nat, (64 - u64_leading_zeros(word)) as nat),
    decreases word,
{
    axiom_u64_leading_zeros(word);
    reveal(u64_leading_zeros);
    if word == 1 {
        reveal(u64_leading_zeros);
        lemma2_to64();
    } else if word > 1 {
        let half = word / 2;
        leading_zeros_determine_word_bit_length(half);
        let previous = (64 - u64_leading_zeros(half)) as nat;
        assert(previous > 0);
        lemma_fundamental_div_mod(word as int, 2);
        lemma_mod_bound(word as int, 2);
        lemma_pow2_unfold(previous);
        lemma_pow2_unfold(previous + 1);
        assert(pow2(previous) <= word && word < pow2(previous + 1)) by (nonlinear_arith)
            requires
                pow2(previous) == 2 * pow2((previous - 1) as nat),
                pow2(previous + 1) == 2 * pow2(previous),
                pow2((previous - 1) as nat) <= half, half < pow2(previous),
                word as int == 2 * (half as int) + (word as int) % 2,
                0 <= (word as int) % 2 < 2;
    }
}

pub(crate) open spec fn normalized_words(words: Seq<u64>) -> bool {
    words.len() == 0 || words[words.len() - 1] > 0
}

pub(crate) open spec fn limb_bit_length(words: Seq<u64>) -> nat {
    if words.len() == 0 { 0 }
    else { (64 * (words.len() - 1) + 64 - u64_leading_zeros(words[words.len() - 1])) as nat }
}

/// There is no machine-width truncation in this mathematical length. The
/// dependency import still must establish canonical top limbs and range checks.
pub(crate) proof fn normalized_limb_bit_length(words: Seq<u64>)
    requires normalized_words(words),
    ensures
        words.len() == 0 ==> value(words) == 0 && limb_bit_length(words) == 0,
        words.len() > 0 ==> has_bit_length(value(words), limb_bit_length(words)),
{
    if words.len() > 0 {
        let last = words.len() as int - 1;
        let top = words[last];
        leading_zeros_determine_word_bit_length(top);
        let bits = (64 - u64_leading_zeros(top)) as nat;
        let lower = prefix(words, last);
        let unit = weight(last as nat);
        prefix_bound(words, last);
        lemma_pow2_pos(64 * last as nat);
        lemma_pow2_adds(64 * last as nat, (bits - 1) as nat);
        lemma_pow2_adds(64 * last as nat, bits);
        assert(value(words) == lower + top * unit);
        assert(pow2((limb_bit_length(words) - 1) as nat) <= value(words)
            && value(words) < pow2(limb_bit_length(words))) by (nonlinear_arith)
            requires
                unit > 0, 0 <= lower < unit,
                pow2((bits - 1) as nat) <= top, top < pow2(bits),
                value(words) == lower + top * unit,
                pow2((limb_bit_length(words) - 1) as nat) == unit * pow2((bits - 1) as nat),
                pow2(limb_bit_length(words)) == unit * pow2(bits);
    }
}

/// A positive normalized integer's exact bit length grows by the complete
/// mathematical shift. This is the width test used before a borrowed scan.
pub(crate) proof fn binary_shift_bit_length(magnitude: nat, bits: nat, shift: nat)
    requires has_bit_length(magnitude, bits),
    ensures has_bit_length(magnitude * pow2(shift), bits + shift),
{
    lemma_pow2_pos(shift);
    lemma_pow2_adds((bits - 1) as nat, shift);
    lemma_pow2_adds(bits, shift);
    assert(pow2((bits + shift - 1) as nat) <= magnitude * pow2(shift)
        && magnitude * pow2(shift) < pow2(bits + shift)) by (nonlinear_arith)
        requires pow2(shift) > 0,
            pow2((bits + shift - 1) as nat) == pow2((bits - 1) as nat) * pow2(shift),
            pow2(bits + shift) == pow2(bits) * pow2(shift),
            pow2((bits - 1) as nat) <= magnitude < pow2(bits);
}

/// Distinct exact widths determine magnitude order without scanning limbs.
pub(crate) proof fn bit_lengths_determine_order(left: nat, right: nat, left_bits: nat, right_bits: nat)
    requires has_bit_length(left, left_bits), has_bit_length(right, right_bits), left_bits < right_bits,
    ensures left < right,
{
    if left_bits < right_bits - 1 {
        lemma_pow2_strictly_increases(left_bits, (right_bits - 1) as nat);
    }
}


pub(crate) proof fn trailing_zeros_factor_u32(value: u32)
    requires value > 0,
    ensures
        u32_trailing_zeros(value) < 32,
        0 < (value >> u32_trailing_zeros(value)) <= value,
        (value >> u32_trailing_zeros(value)) % 2 == 1,
        value == (value >> u32_trailing_zeros(value)) * pow2(u32_trailing_zeros(value) as nat),
{
    axiom_u32_trailing_zeros(value);
    let shift = u32_trailing_zeros(value) as u32;
    let odd = value >> shift;
    assert((value >> shift) % 2 == 1) by (bit_vector)
        requires (value >> shift) & 1u32 == 1u32;
    assert(0 < odd <= value) by (bit_vector)
        requires odd == value >> shift, odd % 2 == 1;
    vstd::bits::lemma_u32_mul_pow2_le_max_iff_max_shr(odd, shift, value);
    vstd::bits::lemma_u32_shl_is_mul(odd, shift);
    assert((value >> shift) << shift == value) by (bit_vector)
        requires shift < 32, value << ((32 - shift) as u32) == 0;
}

pub(crate) proof fn trailing_ones_factor_u32(word: u32)
    ensures
        u32_trailing_ones(word) <= 32,
        ({let unit = pow2(u32_trailing_ones(word) as nat);
          let magnitude = word as nat + 1;
          &&& magnitude / unit > 0
          &&& (magnitude / unit) % 2 == 1
          &&& magnitude == (magnitude / unit) * unit}),
{
    axiom_u32_trailing_ones(word);
    let complement = !word;
    axiom_u32_trailing_zeros(complement);
    let shift = u32_trailing_ones(word) as nat;
    lemma2_to64();
    lemma_pow2_pos(shift);
    let unit = pow2(shift);
    let magnitude = word as nat + 1;
    assert((word as u128) + (complement as u128) == u32::MAX as u128) by (bit_vector)
        requires complement == !word;
    if complement == 0 {
        assert(shift == 32);
        assert(magnitude == unit);
        lemma_div_by_self(unit as int);
    } else {
        trailing_zeros_factor_u32(complement);
        let odd = (complement >> u32_trailing_zeros(complement)) as nat;
        let other = pow2((32 - shift) as nat);
        lemma_pow2_adds(shift, (32 - shift) as nat);
        assert(magnitude == (other as int - odd as int) * unit) by (nonlinear_arith)
            requires word as nat + complement as nat == u32::MAX,
                magnitude == word as nat + 1,
                complement == odd * unit,
                pow2(32) == u32::MAX as nat + 1,
                pow2(32) == unit * other;
        assert(other > odd) by (nonlinear_arith)
            requires magnitude > 0, unit > 0,
                magnitude == (other as int - odd as int) * unit;
        let quotient = (other - odd) as nat;
        lemma_fundamental_div_mod_converse_div(magnitude as int, unit as int, quotient as int, 0);
        lemma_pow2_unfold((32 - shift) as nat);
        let half = pow2((31 - shift) as nat);
        lemma_fundamental_div_mod(odd as int, 2);
        assert(quotient as int == 2 * (half as int - (odd as int) / 2 - 1) + 1);
        lemma_mod_multiples_vanish(half as int - (odd as int) / 2 - 1, 1, 2);
    }
}


pub(crate) proof fn trailing_ones_factor_u64(word: u64)
    ensures
        u64_trailing_ones(word) <= 64,
        ({let unit = pow2(u64_trailing_ones(word) as nat);
          let magnitude = word as nat + 1;
          &&& magnitude / unit > 0
          &&& (magnitude / unit) % 2 == 1
          &&& magnitude == (magnitude / unit) * unit}),
{
    axiom_u64_trailing_ones(word);
    let complement = !word;
    axiom_u64_trailing_zeros(complement);
    let shift = u64_trailing_ones(word) as nat;
    lemma2_to64();
    lemma_pow2_pos(shift);
    let unit = pow2(shift);
    let magnitude = word as nat + 1;
    assert((word as u128) + (complement as u128) == u64::MAX as u128) by (bit_vector)
        requires complement == !word;
    if complement == 0 {
        assert(shift == 64);
        assert(magnitude == unit);
        lemma_div_by_self(unit as int);
    } else {
        crate::verified::gcd::trailing_zeros_factor_u64(complement);
        let odd = (complement >> u64_trailing_zeros(complement)) as nat;
        let other = pow2((64 - shift) as nat);
        lemma_pow2_adds(shift, (64 - shift) as nat);
        assert(magnitude == (other as int - odd as int) * unit) by (nonlinear_arith)
            requires word as nat + complement as nat == u64::MAX,
                magnitude == word as nat + 1,
                complement == odd * unit,
                pow2(64) == u64::MAX as nat + 1,
                pow2(64) == unit * other;
        assert(other > odd) by (nonlinear_arith)
            requires magnitude > 0, unit > 0,
                magnitude == (other as int - odd as int) * unit;
        let quotient = (other - odd) as nat;
        lemma_fundamental_div_mod_converse_div(magnitude as int, unit as int, quotient as int, 0);
        lemma_pow2_unfold((64 - shift) as nat);
        let half = pow2((63 - shift) as nat);
        lemma_fundamental_div_mod(odd as int, 2);
        assert(quotient as int == 2 * (half as int - (odd as int) / 2 - 1) + 1);
        lemma_mod_multiples_vanish(half as int - (odd as int) / 2 - 1, 1, 2);
    }
}

// These arithmetic bridges consume an odd-factor certificate. Actual dependency
// storage, first-match scans and machine-width metadata imports remain open.
pub(crate) proof fn odd_factor_of_power_is_unit(odd: nat, shift: nat, exponent: nat)
    requires odd > 0, odd % 2 == 1,
        odd * pow2(shift) == pow2(exponent),
    ensures odd == 1, shift == exponent,
{
    lemma_pow2_pos(shift);
    lemma_pow2_pos(exponent);
    if shift > exponent {
        lemma_pow2_strictly_increases(exponent, shift);
        assert(false) by (nonlinear_arith)
            requires odd > 0, pow2(shift) > pow2(exponent),
                pow2(shift) > 0, odd * pow2(shift) == pow2(exponent);
    } else if shift < exponent {
        let difference = (exponent - shift) as nat;
        lemma_pow2_adds(shift, difference);
        assert(odd == pow2(difference)) by (nonlinear_arith)
            requires pow2(shift) > 0,
                odd * pow2(shift) == pow2(exponent),
                pow2(exponent) == pow2(shift) * pow2(difference);
        lemma_pow2_unfold(difference);
        let half = pow2((difference - 1) as nat);
        lemma_mod_multiples_vanish(half as int, 0, 2);
        assert(odd % 2 == 0);
    } else {
        assert(odd == 1) by (nonlinear_arith)
            requires pow2(shift) > 0, odd * pow2(shift) == pow2(shift);
    }
}

pub(crate) proof fn trailing_zeros_determine_power_of_two(
    magnitude: nat, bits: nat, zeros: nat, odd: nat,
)
    requires has_bit_length(magnitude, bits),
        odd > 0, odd % 2 == 1, magnitude == odd * pow2(zeros),
    ensures (zeros == (bits - 1) as nat) == (magnitude == pow2((bits - 1) as nat)),
{
    let exponent = (bits - 1) as nat;
    lemma_pow2_pos(exponent);
    lemma_pow2_unfold(bits);
    if zeros == exponent {
        assert(odd < 2) by (nonlinear_arith)
            requires magnitude == odd * pow2(exponent),
                magnitude < 2 * pow2(exponent), pow2(exponent) > 0;
        assert(odd == 1);
    } else if magnitude == pow2(exponent) {
        odd_factor_of_power_is_unit(odd, zeros, exponent);
    }
}

pub(crate) proof fn trailing_ones_determine_all_ones(
    magnitude: nat, bits: nat, ones: nat, odd: nat,
)
    requires has_bit_length(magnitude, bits),
        odd > 0, odd % 2 == 1, magnitude + 1 == odd * pow2(ones),
    ensures (ones == bits) == (magnitude + 1 == pow2(bits)),
{
    lemma_pow2_pos(bits);
    if ones == bits {
        assert(odd <= 1) by (nonlinear_arith)
            requires magnitude + 1 == odd * pow2(bits),
                magnitude + 1 <= pow2(bits), pow2(bits) > 0;
        assert(odd == 1);
    } else if magnitude + 1 == pow2(bits) {
        odd_factor_of_power_is_unit(odd, ones, bits);
    }
}

}
