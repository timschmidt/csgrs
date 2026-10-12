//! Conditional multiword arithmetic for a radix of 2 raised to a digit width.
//! Actual BigUint storage, iterators and machine-width counts are not imported.
#![feature(proc_macro_hygiene)]
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/src/verified/mod.rs"]
mod verified;
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/magnitude_model.rs"]
mod magnitude_model;
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/bit_length_model.rs"]
mod bit_length_model;
use vstd::prelude::*;
use crate::magnitude_model::has_bit_length;
use vstd::std_specs::bits::*;
use vstd::arithmetic::{div_mod::*, power2::*};

verus! {

pub open spec fn digit_weight(width: nat, index: nat) -> nat {
    pow2(width * index)
}

pub open spec fn digit_prefix(digits: Seq<nat>, width: nat, length: int) -> nat
    recommends 0 <= length <= digits.len(),
    decreases length,
{
    if length <= 0 { 0 } else {
        digit_prefix(digits, width, length - 1)
            + digits[length - 1] * digit_weight(width, (length - 1) as nat)
    }
}

pub open spec fn digit_value(digits: Seq<nat>, width: nat) -> nat {
    digit_prefix(digits, width, digits.len() as int)
}

pub open spec fn has_odd_factor(value: nat, exponent: nat) -> bool {
    let unit = pow2(exponent);
    let quotient = value / unit;
    quotient > 0 && quotient % 2 == 1 && value == quotient * unit
}

pub proof fn digit_weight_step(width: nat, index: nat)
    ensures digit_weight(width, index) > 0,
        digit_weight(width, index + 1) == digit_weight(width, index) * pow2(width),
{
    lemma_pow2_pos(width * index);
    lemma_pow2_adds(width * index, width);
    assert(width * (index + 1) == width * index + width) by (nonlinear_arith);
}

pub proof fn digit_weight_add(width: nat, left: nat, right: nat)
    ensures digit_weight(width, left + right)
        == digit_weight(width, left) * digit_weight(width, right),
{
    lemma_pow2_adds(width * left, width * right);
    assert(width * (left + right) == width * left + width * right) by (nonlinear_arith);
}

pub proof fn digit_prefix_bound(digits: Seq<nat>, width: nat, length: int)
    requires 0 <= length <= digits.len(),
        forall|index: int| 0 <= index < length ==> #[trigger] digits[index] < pow2(width),
    ensures digit_prefix(digits, width, length) < digit_weight(width, length as nat),
    decreases length,
{
    if length == 0 {
        lemma2_to64();
    } else {
        digit_prefix_bound(digits, width, length - 1);
        digit_weight_step(width, (length - 1) as nat);
        assert(digit_prefix(digits, width, length) < digit_weight(width, length as nat))
            by (nonlinear_arith)
            requires digit_prefix(digits, width, length)
                    == digit_prefix(digits, width, length - 1)
                        + digits[length - 1] * digit_weight(width, (length - 1) as nat),
                digit_prefix(digits, width, length - 1) < digit_weight(width, (length - 1) as nat),
                0 <= digits[length - 1] < pow2(width),
                digit_weight(width, (length - 1) as nat) > 0,
                digit_weight(width, length as nat) == digit_weight(width, (length - 1) as nat) * pow2(width);
    }
}

pub proof fn digit_prefix_equal(left: Seq<nat>, right: Seq<nat>, width: nat, length: int)
    requires 0 <= length <= left.len(), length <= right.len(),
        forall|index: int| 0 <= index < length ==> #[trigger] left[index] == right[index],
    ensures digit_prefix(left, width, length) == digit_prefix(right, width, length),
    decreases length,
{
    if length > 0 {
        digit_prefix_equal(left, right, width, length - 1);
    }
}

pub proof fn digit_prefix_split(digits: Seq<nat>, width: nat, start: int, end: int)
    requires 0 <= start <= end <= digits.len(),
    ensures digit_prefix(digits, width, end) == digit_prefix(digits, width, start)
        + digit_weight(width, start as nat) * digit_value(digits.subrange(start, end), width),
    decreases end - start,
{
    if start == end {
        assert(digit_value(digits.subrange(start, end), width) == 0);
        assert(digit_weight(width, start as nat) * 0nat == 0) by (nonlinear_arith);
    } else {
        digit_prefix_split(digits, width, start, end - 1);
        let tail = digits.subrange(start, end);
        let previous = digits.subrange(start, end - 1);
        digit_prefix_equal(tail, previous, width, end - start - 1);
        digit_weight_add(width, start as nat, (end - start - 1) as nat);
        assert(digit_prefix(digits, width, end) == digit_prefix(digits, width, start)
            + digit_weight(width, start as nat) * digit_value(tail, width)) by (nonlinear_arith)
            requires digit_prefix(digits, width, end) == digit_prefix(digits, width, end - 1)
                    + digits[end - 1] * digit_weight(width, (end - 1) as nat),
                digit_prefix(digits, width, end - 1) == digit_prefix(digits, width, start)
                    + digit_weight(width, start as nat) * digit_value(previous, width),
                digit_value(tail, width) == digit_value(previous, width)
                    + digits[end - 1] * digit_weight(width, (end - start - 1) as nat),
                digit_weight(width, (end - 1) as nat) == digit_weight(width, start as nat)
                    * digit_weight(width, (end - start - 1) as nat);
    }
}

pub proof fn digit_prefix_zero(digits: Seq<nat>, width: nat, length: int)
    requires 0 <= length <= digits.len(),
    ensures digit_prefix(digits, width, length) == 0 <==>
        forall|index: int| 0 <= index < length ==> #[trigger] digits[index] == 0,
    decreases length,
{
    if length > 0 {
        digit_prefix_zero(digits, width, length - 1);
        digit_weight_step(width, (length - 1) as nat);
        assert(digit_prefix(digits, width, length) == 0 <==>
            digit_prefix(digits, width, length - 1) == 0 && digits[length - 1] == 0)
            by (nonlinear_arith)
            requires digit_prefix(digits, width, length) == digit_prefix(digits, width, length - 1)
                    + digits[length - 1] * digit_weight(width, (length - 1) as nat),
                digit_prefix(digits, width, length - 1) >= 0, digits[length - 1] >= 0,
                digit_weight(width, (length - 1) as nat) > 0;
    }
}

pub proof fn digit_prefix_full(digits: Seq<nat>, width: nat, length: int)
    requires 0 <= length <= digits.len(),
        forall|index: int| 0 <= index < length ==> #[trigger] digits[index] + 1 == pow2(width),
    ensures digit_prefix(digits, width, length) + 1 == digit_weight(width, length as nat),
    decreases length,
{
    if length == 0 {
        lemma2_to64();
    } else {
        digit_prefix_full(digits, width, length - 1);
        digit_weight_step(width, (length - 1) as nat);
        assert(digit_prefix(digits, width, length) + 1 == digit_weight(width, length as nat))
            by (nonlinear_arith)
            requires digit_prefix(digits, width, length) == digit_prefix(digits, width, length - 1)
                    + digits[length - 1] * digit_weight(width, (length - 1) as nat),
                digit_prefix(digits, width, length - 1) + 1 == digit_weight(width, (length - 1) as nat),
                digits[length - 1] + 1 == pow2(width),
                digit_weight(width, length as nat) == digit_weight(width, (length - 1) as nat) * pow2(width);
    }
}

pub proof fn upper_digits_preserve_odd_factor(
    odd: nat, shift: nat, width: nat, upper: nat, offset: nat,
)
    requires odd > 0, odd % 2 == 1, shift < width,
    ensures has_odd_factor((odd * pow2(shift) + pow2(width) * upper) * pow2(offset), offset + shift),
{
    let lower_unit = pow2(shift);
    let other = pow2((width - shift) as nat);
    let unit = pow2(offset + shift);
    let quotient = odd + other * upper;
    let value = (odd * lower_unit + pow2(width) * upper) * pow2(offset);
    lemma_pow2_pos((width - shift) as nat);
    lemma_pow2_pos(offset + shift);
    lemma_pow2_adds(shift, (width - shift) as nat);
    lemma_pow2_adds(offset, shift);
    assert(value == quotient * unit) by (nonlinear_arith)
        requires quotient == odd + other * upper,
            value == (odd * lower_unit + pow2(width) * upper) * pow2(offset),
            pow2(width) == lower_unit * other, unit == pow2(offset) * lower_unit;
    assert(quotient > 0) by (nonlinear_arith)
        requires quotient == odd + other * upper, odd > 0, other > 0, upper >= 0;
    lemma_pow2_unfold((width - shift) as nat);
    let half = pow2((width - shift - 1) as nat);
    assert(quotient == 2 * (half * upper) + odd) by (nonlinear_arith)
        requires quotient == odd + other * upper, other == 2 * half;
    lemma_mod_multiples_vanish((half * upper) as int, odd as int, 2);
    lemma_fundamental_div_mod_converse_div(value as int, unit as int, quotient as int, 0);
}

// This is the None branch of a first-nonmaximal-digit scan, including empty storage.
pub proof fn all_full_digits_factor(digits: Seq<nat>, width: nat)
    requires forall|index: int| 0 <= index < digits.len()
        ==> #[trigger] digits[index] + 1 == pow2(width),
    ensures has_odd_factor(digit_value(digits, width) + 1, width * digits.len()),
{
    digit_prefix_full(digits, width, digits.len() as int);
    lemma_pow2_pos(width * digits.len());
    lemma_div_by_self(pow2(width * digits.len()) as int);
}

pub proof fn first_nonzero_digit_factor(
    digits: Seq<nat>, width: nat, index: int, zeros: nat, odd: nat,
)
    requires width > 0, 0 <= index < digits.len(), zeros < width,
        forall|lower: int| 0 <= lower < index ==> #[trigger] digits[lower] == 0,
        odd > 0, odd % 2 == 1, digits[index] == odd * pow2(zeros),
    ensures has_odd_factor(digit_value(digits, width), width * index as nat + zeros),
{
    digit_prefix_zero(digits, width, index);
    digit_prefix_split(digits, width, index + 1, digits.len() as int);
    digit_weight_step(width, index as nat);
    let upper = digit_value(digits.subrange(index + 1, digits.len() as int), width);
    let offset = width * index as nat;
    assert(digit_value(digits, width)
        == (odd * pow2(zeros) + pow2(width) * upper) * pow2(offset))
        by (nonlinear_arith)
        requires digit_prefix(digits, width, index) == 0,
            digit_value(digits, width) == digit_prefix(digits, width, index + 1)
                + digit_weight(width, index as nat + 1) * upper,
            digit_prefix(digits, width, index + 1) == digit_prefix(digits, width, index)
                + digits[index] * digit_weight(width, index as nat),
            digits[index] == odd * pow2(zeros), digit_weight(width, index as nat) == pow2(offset),
            digit_weight(width, index as nat + 1) == digit_weight(width, index as nat) * pow2(width);
    upper_digits_preserve_odd_factor(odd, zeros, width, upper, offset);
}

pub proof fn first_nonmaximal_digit_factor(
    digits: Seq<nat>, width: nat, index: int, ones: nat, odd: nat,
)
    requires width > 0, 0 <= index < digits.len(), ones <= width,
        forall|lower: int| 0 <= lower < index ==> #[trigger] digits[lower] + 1 == pow2(width),
        digits[index] + 1 < pow2(width),
        odd > 0, odd % 2 == 1, digits[index] + 1 == odd * pow2(ones),
    ensures has_odd_factor(digit_value(digits, width) + 1, width * index as nat + ones),
{
    lemma_pow2_pos(width);
    if ones == width {
        assert(false) by (nonlinear_arith)
            requires digits[index] + 1 == odd * pow2(width),
                digits[index] + 1 < pow2(width), odd > 0, pow2(width) > 0;
    }
    digit_prefix_full(digits, width, index);
    digit_prefix_split(digits, width, index + 1, digits.len() as int);
    digit_weight_step(width, index as nat);
    let upper = digit_value(digits.subrange(index + 1, digits.len() as int), width);
    let offset = width * index as nat;
    assert(digit_value(digits, width) + 1
        == (odd * pow2(ones) + pow2(width) * upper) * pow2(offset))
        by (nonlinear_arith)
        requires digit_prefix(digits, width, index) + 1 == digit_weight(width, index as nat),
            digit_value(digits, width) == digit_prefix(digits, width, index + 1)
                + digit_weight(width, index as nat + 1) * upper,
            digit_prefix(digits, width, index + 1) == digit_prefix(digits, width, index)
                + digits[index] * digit_weight(width, index as nat),
            digits[index] + 1 == odd * pow2(ones), digit_weight(width, index as nat) == pow2(offset),
            digit_weight(width, index as nat + 1) == digit_weight(width, index as nat) * pow2(width);
    upper_digits_preserve_odd_factor(odd, ones, width, upper, offset);
}

pub open spec fn normalized_digits(digits: Seq<nat>, width: nat) -> bool {
    (forall|index: int| 0 <= index < digits.len() ==> #[trigger] digits[index] < pow2(width))
        && (digits.len() == 0 || digits[digits.len() - 1] > 0)
}

pub proof fn normalized_zero_iff_empty(digits: Seq<nat>, width: nat)
    requires normalized_digits(digits, width),
    ensures digit_value(digits, width) == 0 <==> digits.len() == 0,
{
    digit_prefix_zero(digits, width, digits.len() as int);
    if digits.len() > 0 && digit_value(digits, width) == 0 {
        assert(digits[digits.len() - 1] == 0);
    }
}

pub proof fn normalized_digit_bit_length(digits: Seq<nat>, width: nat, top_bits: nat)
    requires normalized_digits(digits, width), digits.len() > 0,
        crate::magnitude_model::has_bit_length(digits[digits.len() - 1], top_bits),
    ensures crate::magnitude_model::has_bit_length(digit_value(digits, width),
        width * (digits.len() - 1) + top_bits),
{
    let last = digits.len() as int - 1;
    let top = digits[last];
    let lower = digit_prefix(digits, width, last);
    let offset = width * last as nat;
    let unit = digit_weight(width, last as nat);
    let bits = offset + top_bits;
    digit_prefix_bound(digits, width, last);
    lemma_pow2_pos(offset);
    lemma_pow2_adds(offset, (top_bits - 1) as nat);
    lemma_pow2_adds(offset, top_bits);
    assert(pow2((bits - 1) as nat) <= digit_value(digits, width)
        && digit_value(digits, width) < pow2(bits)) by (nonlinear_arith)
        requires unit > 0, 0 <= lower < unit,
            pow2((top_bits - 1) as nat) <= top, top < pow2(top_bits),
            digit_value(digits, width) == lower + top * unit,
            pow2((bits - 1) as nat) == unit * pow2((top_bits - 1) as nat),
            pow2(bits) == unit * pow2(top_bits);
}

pub proof fn zero_scan_determines_power_of_two(
    digits: Seq<nat>, width: nat, index: int, zeros: nat, odd: nat, bits: nat,
)
    requires width > 0, 0 <= index < digits.len(), zeros < width,
        forall|lower: int| 0 <= lower < index ==> #[trigger] digits[lower] == 0,
        odd > 0, odd % 2 == 1, digits[index] == odd * pow2(zeros),
        crate::magnitude_model::has_bit_length(digit_value(digits, width), bits),
    ensures (width * index as nat + zeros == (bits - 1) as nat)
        == (digit_value(digits, width) == pow2((bits - 1) as nat)),
{
    first_nonzero_digit_factor(digits, width, index, zeros, odd);
    let count = width * index as nat + zeros;
    let value = digit_value(digits, width);
    crate::bit_length_model::trailing_zeros_determine_power_of_two(
        value, bits, count, value / pow2(count));
}

pub proof fn one_scan_determines_all_ones(
    digits: Seq<nat>, width: nat, index: int, ones: nat, odd: nat, bits: nat,
)
    requires width > 0, 0 <= index < digits.len(), ones <= width,
        forall|lower: int| 0 <= lower < index ==> #[trigger] digits[lower] + 1 == pow2(width),
        digits[index] + 1 < pow2(width),
        odd > 0, odd % 2 == 1, digits[index] + 1 == odd * pow2(ones),
        crate::magnitude_model::has_bit_length(digit_value(digits, width), bits),
    ensures (width * index as nat + ones == bits)
        == (digit_value(digits, width) + 1 == pow2(bits)),
{
    first_nonmaximal_digit_factor(digits, width, index, ones, odd);
    let count = width * index as nat + ones;
    let value = digit_value(digits, width);
    crate::bit_length_model::trailing_ones_determine_all_ones(
        value, bits, count, (value + 1) / pow2(count));
}

pub proof fn full_digit_scan_determines_all_ones(digits: Seq<nat>, width: nat, bits: nat)
    requires forall|index: int| 0 <= index < digits.len()
        ==> #[trigger] digits[index] + 1 == pow2(width),
        crate::magnitude_model::has_bit_length(digit_value(digits, width), bits),
    ensures (width * digits.len() == bits) == (digit_value(digits, width) + 1 == pow2(bits)),
{
    all_full_digits_factor(digits, width);
    let count = width * digits.len();
    let value = digit_value(digits, width);
    crate::bit_length_model::trailing_ones_determine_all_ones(
        value, bits, count, (value + 1) / pow2(count));
}

pub proof fn u32_digit_bit_length(word: u32)
    ensures
        0 <= u32_leading_zeros(word) <= 32,
        word == 0 <==> u32_leading_zeros(word) == 32,
        word > 0 ==> has_bit_length(word as nat, (32 - u32_leading_zeros(word)) as nat),
    decreases word,
{
    axiom_u32_leading_zeros(word);
    reveal(u32_leading_zeros);
    if word == 1 {
        reveal(u32_leading_zeros);
        lemma2_to64();
    } else if word > 1 {
        let half = word / 2;
        u32_digit_bit_length(half);
        let previous = (32 - u32_leading_zeros(half)) as nat;
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

}
