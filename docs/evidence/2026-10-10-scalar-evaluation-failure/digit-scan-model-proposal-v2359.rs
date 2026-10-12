//! Conditional multiword arithmetic for a radix of 2 raised to a digit width.
//! Actual BigUint storage, iterators and machine-width counts are not imported.
use vstd::prelude::*;
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

}
