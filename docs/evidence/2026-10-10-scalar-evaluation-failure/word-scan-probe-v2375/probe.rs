//! Conditional typed word views; the actual dependency storage import remains open.
#![feature(proc_macro_hygiene)]
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/src/verified/mod.rs"]
mod verified;
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/magnitude_model.rs"]
mod magnitude_model;
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/bit_length_model.rs"]
mod bit_length_model;
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/digit_scan_model.rs"]
mod digit_scan_model;
use crate::digit_scan_model::*;
use crate::magnitude_model::has_bit_length;
use vstd::prelude::*;
use vstd::arithmetic::power2::*;
use vstd::std_specs::bits::*;

verus! {

pub proof fn trimmed_zero_suffix_preserves_value(digits: Seq<nat>, width: nat, length: int)
    requires 0 <= length <= digits.len(),
        forall|index: int| length <= index < digits.len() ==> #[trigger] digits[index] == 0,
    ensures digit_value(digits.subrange(0, length), width) == digit_value(digits, width),
{
    let suffix = digits.subrange(length, digits.len() as int);
    assert forall|index: int| 0 <= index < suffix.len() implies #[trigger] suffix[index] == 0 by {
        assert(suffix[index] == digits[length + index]);
    }
    digit_prefix_zero(suffix, width, suffix.len() as int);
    digit_prefix_split(digits, width, length, digits.len() as int);
    digit_prefix_equal(digits, digits.subrange(0, length), width, length);
    assert(digit_weight(width, length as nat) * 0nat == 0) by (nonlinear_arith);
}

pub proof fn u64_digit_prefix_matches_production(words: Seq<u64>, length: int)
    requires 0 <= length <= words.len(),
    ensures digit_prefix(word_digits_u64(words), 64, length)
        == crate::verified::limbs::prefix(words, length),
    decreases length,
{
    if length > 0 {
        u64_digit_prefix_matches_production(words, length - 1);
    }
}

pub proof fn u64_digit_value_matches_production(words: Seq<u64>)
    ensures word_value_u64(words) == crate::verified::limbs::value(words),
{
    u64_digit_prefix_matches_production(words, words.len() as int);
}

pub proof fn digit_scan_count_fits_u64(length: nat, width: nat, index: nat, within: nat)
    requires width > 0, index < length, within < width,
        width * length <= u64::MAX,
    ensures index <= u64::MAX, width * index <= u64::MAX,
        width * index + within < width * length,
        width * index + within <= u64::MAX,
{
    assert(index <= u64::MAX && width * index <= u64::MAX
        && width * index + within < width * length
        && width * index + within <= u64::MAX) by (nonlinear_arith)
        requires width > 0, index < length, within < width,
            width * length <= u64::MAX;
}

pub open spec fn word_digits_u32(words: Seq<u32>) -> Seq<nat> {
    Seq::new(words.len(), |index: int| words[index] as nat)
}

pub open spec fn word_value_u32(words: Seq<u32>) -> nat {
    digit_value(word_digits_u32(words), 32)
}

pub open spec fn normalized_u32(words: Seq<u32>) -> bool {
    words.len() == 0 || words[words.len() - 1] > 0
}

pub open spec fn word_bit_length_u32(words: Seq<u32>) -> nat {
    if words.len() == 0 { 0 }
    else { (32 * words.len() - u32_leading_zeros(words[words.len() - 1])) as nat }
}

pub open spec fn first_nonzero_u32(words: Seq<u32>, index: int) -> bool {
    0 <= index < words.len() && words[index] != 0
        && forall|lower: int| 0 <= lower < index ==> #[trigger] words[lower] == 0
}

pub open spec fn first_nonmaximal_u32(words: Seq<u32>, index: int) -> bool {
    0 <= index < words.len() && words[index] != u32::MAX
        && forall|lower: int| 0 <= lower < index ==> #[trigger] words[lower] == u32::MAX
}

pub proof fn typed_digit_bounds_u32(words: Seq<u32>)
    ensures word_digits_u32(words).len() == words.len(),
        forall|index: int| 0 <= index < words.len()
            ==> #[trigger] word_digits_u32(words)[index] < pow2(32),
        normalized_digits(word_digits_u32(words), 32) == normalized_u32(words),
{
    lemma2_to64();
}

pub proof fn typed_normalized_bit_length_u32(words: Seq<u32>)
    requires normalized_u32(words),
    ensures word_value_u32(words) == 0 <==> words.len() == 0,
        word_bit_length_u32(words) == 0 <==> words.len() == 0,
        word_bit_length_u32(words) <= 32 * words.len(),
        words.len() > 0 ==> has_bit_length(word_value_u32(words), word_bit_length_u32(words)),
{
    typed_digit_bounds_u32(words);
    normalized_zero_iff_empty(word_digits_u32(words), 32);
    if words.len() > 0 {
        let top = words[words.len() - 1];
        crate::digit_scan_model::u32_digit_bit_length(top);
        let bits = (32 - u32_leading_zeros(top)) as nat;
        normalized_digit_bit_length(word_digits_u32(words), 32, bits);
    }
}

pub proof fn typed_zero_scan_u32(words: Seq<u32>, index: int)
    requires first_nonzero_u32(words, index),
    ensures has_odd_factor(word_value_u32(words),
            32 * index as nat + u32_trailing_zeros(words[index]) as nat),
        32 * index as nat + (u32_trailing_zeros(words[index]) as nat) < 32 * words.len(),
{
    let digits = word_digits_u32(words);
    let word = words[index];
    crate::bit_length_model::trailing_zeros_factor_u32(word);
    let zeros = u32_trailing_zeros(word) as nat;
    let odd = (word >> u32_trailing_zeros(word)) as nat;
    assert forall|lower: int| 0 <= lower < index implies #[trigger] digits[lower] == 0 by {
        assert(words[lower] == 0);
    }
    first_nonzero_digit_factor(digits, 32, index, zeros, odd);
}

pub proof fn typed_one_scan_u32(words: Seq<u32>, index: int)
    requires first_nonmaximal_u32(words, index),
    ensures has_odd_factor(word_value_u32(words) + 1,
            32 * index as nat + u32_trailing_ones(words[index]) as nat),
        32 * index as nat + (u32_trailing_ones(words[index]) as nat) < 32 * words.len(),
{
    let digits = word_digits_u32(words);
    let word = words[index];
    crate::bit_length_model::trailing_ones_factor_u32(word);
    axiom_u32_trailing_ones(word);
    lemma2_to64();
    let ones = u32_trailing_ones(word) as nat;
    let odd = (word as nat + 1) / pow2(ones);
    assert forall|lower: int| 0 <= lower < index implies #[trigger] digits[lower] + 1 == pow2(32) by {
        assert(words[lower] == u32::MAX);
    }
    first_nonmaximal_digit_factor(digits, 32, index, ones, odd);
}

pub proof fn typed_zero_scan_none_u32(words: Seq<u32>)
    requires forall|index: int| 0 <= index < words.len() ==> #[trigger] words[index] == 0,
    ensures word_value_u32(words) == 0,
{
    let digits = word_digits_u32(words);
    assert forall|index: int| 0 <= index < digits.len() implies #[trigger] digits[index] == 0 by {
        assert(words[index] == 0);
    }
    digit_prefix_zero(digits, 32, digits.len() as int);
}

pub proof fn typed_one_scan_none_u32(words: Seq<u32>)
    requires forall|index: int| 0 <= index < words.len() ==> #[trigger] words[index] == u32::MAX,
    ensures has_odd_factor(word_value_u32(words) + 1, 32 * words.len()),
{
    let digits = word_digits_u32(words);
    lemma2_to64();
    assert forall|index: int| 0 <= index < digits.len() implies #[trigger] digits[index] + 1 == pow2(32) by {
        assert(words[index] == u32::MAX);
    }
    all_full_digits_factor(digits, 32);
}

pub proof fn typed_zero_suffix_truncation_u32(words: Seq<u32>, length: int)
    requires 0 <= length <= words.len(),
        length == 0 || words[length - 1] > 0,
        forall|index: int| length <= index < words.len() ==> #[trigger] words[index] == 0,
    ensures word_value_u32(words.subrange(0, length)) == word_value_u32(words),
        normalized_u32(words.subrange(0, length)),
{
    let digits = word_digits_u32(words);
    assert forall|index: int| length <= index < digits.len() implies #[trigger] digits[index] == 0 by {
        assert(words[index] == 0);
    }
    trimmed_zero_suffix_preserves_value(digits, 32, length);
    assert(word_digits_u32(words.subrange(0, length)) =~= digits.subrange(0, length));
}

pub open spec fn word_digits_u64(words: Seq<u64>) -> Seq<nat> {
    Seq::new(words.len(), |index: int| words[index] as nat)
}

pub open spec fn word_value_u64(words: Seq<u64>) -> nat {
    digit_value(word_digits_u64(words), 64)
}

pub open spec fn normalized_u64(words: Seq<u64>) -> bool {
    words.len() == 0 || words[words.len() - 1] > 0
}

pub open spec fn word_bit_length_u64(words: Seq<u64>) -> nat {
    if words.len() == 0 { 0 }
    else { (64 * words.len() - u64_leading_zeros(words[words.len() - 1])) as nat }
}

pub open spec fn first_nonzero_u64(words: Seq<u64>, index: int) -> bool {
    0 <= index < words.len() && words[index] != 0
        && forall|lower: int| 0 <= lower < index ==> #[trigger] words[lower] == 0
}

pub open spec fn first_nonmaximal_u64(words: Seq<u64>, index: int) -> bool {
    0 <= index < words.len() && words[index] != u64::MAX
        && forall|lower: int| 0 <= lower < index ==> #[trigger] words[lower] == u64::MAX
}

pub proof fn typed_digit_bounds_u64(words: Seq<u64>)
    ensures word_digits_u64(words).len() == words.len(),
        forall|index: int| 0 <= index < words.len()
            ==> #[trigger] word_digits_u64(words)[index] < pow2(64),
        normalized_digits(word_digits_u64(words), 64) == normalized_u64(words),
{
    lemma2_to64();
}

pub proof fn typed_normalized_bit_length_u64(words: Seq<u64>)
    requires normalized_u64(words),
    ensures word_value_u64(words) == 0 <==> words.len() == 0,
        word_bit_length_u64(words) == 0 <==> words.len() == 0,
        word_bit_length_u64(words) <= 64 * words.len(),
        words.len() > 0 ==> has_bit_length(word_value_u64(words), word_bit_length_u64(words)),
{
    typed_digit_bounds_u64(words);
    normalized_zero_iff_empty(word_digits_u64(words), 64);
    if words.len() > 0 {
        let top = words[words.len() - 1];
        crate::bit_length_model::leading_zeros_determine_word_bit_length(top);
        let bits = (64 - u64_leading_zeros(top)) as nat;
        normalized_digit_bit_length(word_digits_u64(words), 64, bits);
    }
}

pub proof fn typed_zero_scan_u64(words: Seq<u64>, index: int)
    requires first_nonzero_u64(words, index),
    ensures has_odd_factor(word_value_u64(words),
            64 * index as nat + u64_trailing_zeros(words[index]) as nat),
        64 * index as nat + (u64_trailing_zeros(words[index]) as nat) < 64 * words.len(),
{
    let digits = word_digits_u64(words);
    let word = words[index];
    crate::verified::gcd::trailing_zeros_factor_u64(word);
    let zeros = u64_trailing_zeros(word) as nat;
    let odd = (word >> u64_trailing_zeros(word)) as nat;
    assert forall|lower: int| 0 <= lower < index implies #[trigger] digits[lower] == 0 by {
        assert(words[lower] == 0);
    }
    first_nonzero_digit_factor(digits, 64, index, zeros, odd);
}

pub proof fn typed_one_scan_u64(words: Seq<u64>, index: int)
    requires first_nonmaximal_u64(words, index),
    ensures has_odd_factor(word_value_u64(words) + 1,
            64 * index as nat + u64_trailing_ones(words[index]) as nat),
        64 * index as nat + (u64_trailing_ones(words[index]) as nat) < 64 * words.len(),
{
    let digits = word_digits_u64(words);
    let word = words[index];
    crate::bit_length_model::trailing_ones_factor_u64(word);
    axiom_u64_trailing_ones(word);
    lemma2_to64();
    let ones = u64_trailing_ones(word) as nat;
    let odd = (word as nat + 1) / pow2(ones);
    assert forall|lower: int| 0 <= lower < index implies #[trigger] digits[lower] + 1 == pow2(64) by {
        assert(words[lower] == u64::MAX);
    }
    first_nonmaximal_digit_factor(digits, 64, index, ones, odd);
}

pub proof fn typed_zero_scan_none_u64(words: Seq<u64>)
    requires forall|index: int| 0 <= index < words.len() ==> #[trigger] words[index] == 0,
    ensures word_value_u64(words) == 0,
{
    let digits = word_digits_u64(words);
    assert forall|index: int| 0 <= index < digits.len() implies #[trigger] digits[index] == 0 by {
        assert(words[index] == 0);
    }
    digit_prefix_zero(digits, 64, digits.len() as int);
}

pub proof fn typed_one_scan_none_u64(words: Seq<u64>)
    requires forall|index: int| 0 <= index < words.len() ==> #[trigger] words[index] == u64::MAX,
    ensures has_odd_factor(word_value_u64(words) + 1, 64 * words.len()),
{
    let digits = word_digits_u64(words);
    lemma2_to64();
    assert forall|index: int| 0 <= index < digits.len() implies #[trigger] digits[index] + 1 == pow2(64) by {
        assert(words[index] == u64::MAX);
    }
    all_full_digits_factor(digits, 64);
}

pub proof fn typed_zero_suffix_truncation_u64(words: Seq<u64>, length: int)
    requires 0 <= length <= words.len(),
        length == 0 || words[length - 1] > 0,
        forall|index: int| length <= index < words.len() ==> #[trigger] words[index] == 0,
    ensures word_value_u64(words.subrange(0, length)) == word_value_u64(words),
        normalized_u64(words.subrange(0, length)),
{
    let digits = word_digits_u64(words);
    assert forall|index: int| length <= index < digits.len() implies #[trigger] digits[index] == 0 by {
        assert(words[index] == 0);
    }
    trimmed_zero_suffix_preserves_value(digits, 64, length);
    assert(word_digits_u64(words.subrange(0, length)) =~= digits.subrange(0, length));
}

}
