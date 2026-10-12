#[cfg(verus_keep_ghost)]
use super::division::{gcd, gcd_characterization};
#[cfg(verus_keep_ghost)]
use vstd::arithmetic::{div_mod::*, power::lemma_pow0, power2::*};
#[cfg(verus_keep_ghost)]
use vstd::prelude::*;
#[cfg(verus_keep_ghost)]
use vstd::std_specs::bits::{axiom_u64_trailing_zeros, u64_trailing_zeros};

#[cfg(verus_keep_ghost)]
verus! {
pub(crate) proof fn gcd_symmetric(left: nat, right: nat)
    ensures gcd(left, right) == gcd(right, left),
{
    if left < right {
        lemma_small_mod(left, right);
    } else if right < left {
        lemma_small_mod(right, left);
    }
}

pub(crate) proof fn gcd_subtract(left: nat, right: nat)
    requires 0 < left <= right,
    ensures gcd(left, (right - left) as nat) == gcd(left, right),
{
    gcd_symmetric(left, right);
    gcd_symmetric(left, (right - left) as nat);
    lemma_mod_sub_multiples_vanish(right as int, left as int);
}

pub(crate) proof fn gcd_scaled(left: nat, right: nat, factor: nat)
    requires factor > 0,
    ensures gcd(left * factor, right * factor) == gcd(left, right) * factor,
    decreases right,
{
    if right != 0 {
        let remainder = left % right;
        let quotient = left / right;
        lemma_fundamental_div_mod(left as int, right as int);
        assert(0 <= remainder * factor < right * factor) by (nonlinear_arith)
            requires 0 <= remainder < right, factor > 0;
        assert(left * factor == quotient * (right * factor) + remainder * factor)
            by (nonlinear_arith)
            requires left == quotient * right + remainder;
        lemma_fundamental_div_mod_converse_mod(
            (left * factor) as int, (right * factor) as int,
            quotient as int, (remainder * factor) as int);
        gcd_scaled(right, remainder, factor);
    }
}

/// Cancel the complete common factor without changing the represented ratio.
/// This also covers a zero numerator, whose reduced denominator is one.
pub(crate) proof fn gcd_reduction(numerator: nat, denominator: nat)
    requires denominator > 0,
    ensures
        gcd(numerator, denominator) > 0,
        numerator / gcd(numerator, denominator) * gcd(numerator, denominator) == numerator,
        denominator / gcd(numerator, denominator) * gcd(numerator, denominator) == denominator,
        0 < denominator / gcd(numerator, denominator) <= denominator,
        numerator / gcd(numerator, denominator) <= numerator,
        gcd(numerator / gcd(numerator, denominator), denominator / gcd(numerator, denominator)) == 1,
        numerator / gcd(numerator, denominator) * denominator
            == numerator * (denominator / gcd(numerator, denominator)),
        (numerator / gcd(numerator, denominator) == 0 <==> numerator == 0),
        numerator == 0 ==> denominator / gcd(numerator, denominator) == 1,
{
    gcd_characterization(numerator, denominator);
    let divisor = gcd(numerator, denominator);
    let reduced_numerator = numerator / divisor;
    let reduced_denominator = denominator / divisor;
    lemma_fundamental_div_mod(numerator as int, divisor as int);
    lemma_fundamental_div_mod(denominator as int, divisor as int);
    assert(numerator == reduced_numerator * divisor
        && denominator == reduced_denominator * divisor) by (nonlinear_arith)
        requires numerator == divisor * reduced_numerator, denominator == divisor * reduced_denominator;
    assert(0 < reduced_denominator <= denominator && reduced_numerator <= numerator
        && (reduced_numerator == 0 <==> numerator == 0)) by (nonlinear_arith)
        requires divisor > 0, denominator > 0, reduced_numerator >= 0, reduced_denominator >= 0,
            numerator == reduced_numerator * divisor, denominator == reduced_denominator * divisor;
    gcd_scaled(reduced_numerator, reduced_denominator, divisor);
    assert(gcd(reduced_numerator, reduced_denominator) == 1) by (nonlinear_arith)
        requires divisor > 0, divisor == gcd(reduced_numerator, reduced_denominator) * divisor;
    assert(reduced_numerator * denominator == numerator * reduced_denominator) by (nonlinear_arith)
        requires numerator == reduced_numerator * divisor, denominator == reduced_denominator * divisor;
    if numerator == 0 {
        lemma_small_mod(0, denominator);
        reveal_with_fuel(gcd, 2);
        assert(divisor == denominator);
        lemma_div_by_self(denominator as int);
    }
}

pub(crate) proof fn gcd_bound(left: nat, right: nat)
    ensures
        left > 0 ==> gcd(left, right) <= left,
        right > 0 ==> gcd(left, right) <= right,
{
    gcd_characterization(left, right);
    let divisor = gcd(left, right);
    if left > 0 && divisor > left {
        lemma_small_mod(left, divisor);
    }
    if right > 0 && divisor > right {
        lemma_small_mod(right, divisor);
    }
}

proof fn divisor_of_odd_is_odd(value: nat, divisor: nat)
    requires value % 2 == 1, divisor > 0, value % divisor == 0,
    ensures divisor % 2 == 1,
{
    lemma_fundamental_div_mod(value as int, divisor as int);
    assert(value == (value / divisor) * divisor) by (nonlinear_arith)
        requires value == divisor * (value / divisor) + value % divisor, value % divisor == 0;
    lemma_mul_mod_noop((value / divisor) as int, divisor as int, 2);
    if divisor % 2 == 0 {
        assert(((value / divisor) % 2) * (divisor % 2) == 0) by (nonlinear_arith)
            requires divisor % 2 == 0;
        assert(value % 2 == 0);
    }
}

proof fn odd_divisor_cancels_two(value: nat, divisor: nat)
    requires divisor > 0, divisor % 2 == 1, (2 * value) % divisor == 0,
    ensures value % divisor == 0,
{
    let quotient = 2 * value / divisor;
    lemma_fundamental_div_mod((2 * value) as int, divisor as int);
    lemma_mul_mod_noop(quotient as int, divisor as int, 2);
    lemma_mod_multiples_basic(value as int, 2);
    lemma_fundamental_div_mod(quotient as int, 2);
    assert(value == (quotient / 2) * divisor) by (nonlinear_arith)
        requires 2 * value == quotient * divisor, quotient == 2 * (quotient / 2);
    lemma_mod_multiples_basic((quotient / 2) as int, divisor as int);
}

proof fn gcd_odd_double(left: nat, right: nat)
    requires left % 2 == 1,
    ensures gcd(left, 2 * right) == gcd(left, right),
{
    gcd_characterization(left, right);
    gcd_characterization(left, 2 * right);
    let original = gcd(left, right);
    let doubled = gcd(left, 2 * right);
    lemma_mul_mod_noop_right(2, right as int, original as int);
    lemma_small_mod(0, original);
    assert((2 * right) % original == 0);
    divisor_of_odd_is_odd(left, doubled);
    odd_divisor_cancels_two(right, doubled);
    assert(original <= doubled);
    assert(doubled <= original);
}

pub(crate) proof fn gcd_odd_power_two(left: nat, right: nat, shift: nat)
    requires left % 2 == 1,
    ensures gcd(left, right * pow2(shift)) == gcd(left, right),
    decreases shift,
{
    if shift == 0 {
        lemma_pow0(2);
        assert(pow2(shift) == 1);
        assert(right * pow2(shift) == right) by (nonlinear_arith)
            requires pow2(shift) == 1;
    } else {
        lemma_pow2_unfold(shift);
        let previous = pow2((shift - 1) as nat);
        assert(right * pow2(shift) == 2 * (right * previous)) by (nonlinear_arith)
            requires pow2(shift) == 2 * previous;
        gcd_odd_double(left, right * previous);
        gcd_odd_power_two(left, right, (shift - 1) as nat);
        assert(gcd(left, right * pow2(shift)) == gcd(left, right * previous));
        assert(gcd(left, right * previous) == gcd(left, right));
    }
}

proof fn gcd_power_two_factors_ordered(left: nat, right: nat, low: nat, high: nat)
    requires left % 2 == 1, low <= high,
    ensures gcd(left * pow2(low), right * pow2(high)) == gcd(left, right) * pow2(low),
{
    let extra = (high - low) as nat;
    lemma_pow2_pos(low);
    lemma_pow2_adds(extra, low);
    assert(right * pow2(high) == (right * pow2(extra)) * pow2(low))
        by (nonlinear_arith) requires pow2(high) == pow2(extra) * pow2(low);
    gcd_scaled(left, right * pow2(extra), pow2(low));
    gcd_odd_power_two(left, right, extra);
}

pub(crate) proof fn gcd_power_two_factors(left: nat, right: nat, left_shift: nat, right_shift: nat)
    requires left % 2 == 1, right % 2 == 1,
    ensures gcd(left * pow2(left_shift), right * pow2(right_shift))
        == gcd(left, right) * pow2(if left_shift < right_shift { left_shift } else { right_shift }),
{
    if left_shift <= right_shift {
        gcd_power_two_factors_ordered(left, right, left_shift, right_shift);
    } else {
        gcd_power_two_factors_ordered(right, left, right_shift, left_shift);
        gcd_symmetric(left * pow2(left_shift), right * pow2(right_shift));
        gcd_symmetric(left, right);
    }
}

/// Connect the standard-library trailing-zero specification to exact division
/// by a mathematical power of two, including the value's odd remaining part.
pub(crate) proof fn trailing_zeros_factor_u64(value: u64)
    requires value > 0,
    ensures
        u64_trailing_zeros(value) < 64,
        0 < (value >> u64_trailing_zeros(value)) <= value,
        (value >> u64_trailing_zeros(value)) % 2 == 1,
        value == (value >> u64_trailing_zeros(value)) * pow2(u64_trailing_zeros(value) as nat),
{
    axiom_u64_trailing_zeros(value);
    let shift = u64_trailing_zeros(value) as u64;
    let odd = value >> shift;
    assert((value >> shift) % 2 == 1) by (bit_vector)
        requires (value >> shift) & 1u64 == 1u64;
    assert(0 < odd <= value) by (bit_vector)
        requires odd == value >> shift, odd % 2 == 1;
    vstd::bits::lemma_u64_mul_pow2_le_max_iff_max_shr(odd, shift, value);
    vstd::bits::lemma_u64_shl_is_mul(odd, shift);
    assert((value >> shift) << shift == value) by (bit_vector)
        requires shift < 64, value << ((64 - shift) as u64) == 0;
}

pub(crate) open spec fn trailing_zeros_u128_spec(value: u128) -> u32 {
    if value as u64 != 0 {
        u64_trailing_zeros(value as u64)
    } else {
        (64 + u64_trailing_zeros((value >> 64) as u64)) as u32
    }
}

pub(crate) proof fn trailing_zeros_factor_u128(value: u128)
    ensures
        trailing_zeros_u128_spec(value) <= 128,
        (trailing_zeros_u128_spec(value) == 128 <==> value == 0),
        value > 0 ==> (
            0 < (value >> trailing_zeros_u128_spec(value)) <= value
            && (value >> trailing_zeros_u128_spec(value)) % 2 == 1
            && value == (value >> trailing_zeros_u128_spec(value))
                * pow2(trailing_zeros_u128_spec(value) as nat)),
{
    let low = value as u64;
    let high = (value >> 64) as u64;
    axiom_u64_trailing_zeros(low);
    axiom_u64_trailing_zeros(high);
    let shift = trailing_zeros_u128_spec(value);
    assert(value == 0 <==> low == 0 && high == 0) by (bit_vector)
        requires low == value as u64, high == (value >> 64) as u64;
    if value > 0 {
        if low != 0 {
            assert(value << ((128 - shift) as u32) == 0) by (bit_vector)
                requires shift < 64, low == value as u64,
                    low << ((64 - shift) as u64) == 0;
            assert((value >> shift) & 1u128 == 1u128) by (bit_vector)
                requires shift < 64, low == value as u64,
                    (low >> shift) & 1u64 == 1u64;
        } else {
            let upper_shift = u64_trailing_zeros(high);
            assert(value << ((128 - shift) as u32) == 0) by (bit_vector)
                requires upper_shift < 64, shift == 64 + upper_shift,
                    value as u64 == 0, high == (value >> 64) as u64,
                    high << ((64 - upper_shift) as u64) == 0;
            assert((value >> shift) & 1u128 == 1u128) by (bit_vector)
                requires upper_shift < 64, shift == 64 + upper_shift,
                    high == (value >> 64) as u64,
                    (high >> upper_shift) & 1u64 == 1u64;
        }
        let odd = value >> shift;
        assert(0 < odd <= value && odd % 2 == 1) by (bit_vector)
            requires odd == value >> shift, odd & 1u128 == 1u128;
        lemma_pow2_pos(shift as nat);
        vstd::bits::lemma_u128_shr_is_div(value, shift as u128);
        lemma_fundamental_div_mod(value as int, pow2(shift as nat) as int);
        assert(odd * pow2(shift as nat) <= value) by (nonlinear_arith)
            requires odd == value as nat / pow2(shift as nat),
                value == pow2(shift as nat) * (value as nat / pow2(shift as nat))
                    + value as nat % pow2(shift as nat),
                value as nat % pow2(shift as nat) >= 0;
        super::word::shift_left_u128_is_mul(odd, shift);
        assert((value >> shift) << shift == value) by (bit_vector)
            requires shift < 128, value << ((128 - shift) as u32) == 0;
    }
}

proof fn power_of_two_has_unit_odd_part(value: u64)
    requires value > 0, value & ((value - 1) as u64) == 0,
    ensures (value >> u64_trailing_zeros(value)) == 1,
{
    axiom_u64_trailing_zeros(value);
    let shift = u64_trailing_zeros(value);
    assert((value >> shift) == 1) by (bit_vector)
        requires value > 0, value & ((value - 1) as u64) == 0,
            shift < 64, (value >> shift) & 1u64 == 1u64;
}

/// Any common prefix of the two trailing-zero counts is an exact common
/// factor. This also covers the two-limb path that keeps non-odd quotients.
proof fn common_shift_preserves_gcd(left: u128, right: u128, shift: u32)
    requires left > 0, right > 0,
        shift <= trailing_zeros_u128_spec(left), shift <= trailing_zeros_u128_spec(right),
    ensures shift < 128,
        gcd(left as nat, right as nat)
            == gcd((left >> shift) as nat, (right >> shift) as nat) * pow2(shift as nat),
{
    trailing_zeros_factor_u128(left);
    trailing_zeros_factor_u128(right);
    divide_trailing_factor(left, shift);
    divide_trailing_factor(right, shift);
    lemma_pow2_pos(shift as nat);
    gcd_scaled((left >> shift) as nat, (right >> shift) as nat, pow2(shift as nat));
}

pub(crate) proof fn divide_trailing_factor(value: u128, shift: u32)
    requires value > 0, shift <= trailing_zeros_u128_spec(value),
    ensures shift < 128, 0 < value >> shift,
        value == (value >> shift) * pow2(shift as nat),
{
    trailing_zeros_factor_u128(value);
    let total = trailing_zeros_u128_spec(value);
    let odd = (value >> total) as nat;
    let extra = (total - shift) as nat;
    lemma_pow2_adds(extra, shift as nat);
    lemma_pow2_pos(shift as nat);
    assert(value == (odd * pow2(extra)) * pow2(shift as nat)) by (nonlinear_arith)
        requires value == odd * pow2(total as nat),
            pow2(total as nat) == pow2(extra) * pow2(shift as nat);
    lemma_fundamental_div_mod_converse(value as int, pow2(shift as nat) as int,
        (odd * pow2(extra)) as int, 0);
    vstd::bits::lemma_u128_shr_is_div(value, shift as u128);
    lemma_pow2_pos(extra);
    assert(odd * pow2(extra) > 0) by (nonlinear_arith)
        requires odd > 0, pow2(extra) > 0;
}

proof fn gcd_power_of_two_word(left: u64, right: u128, shift: u32)
    requires left > 0, right > 0, left & ((left - 1) as u64) == 0,
        shift == if u64_trailing_zeros(left) < trailing_zeros_u128_spec(right) {
            u64_trailing_zeros(left)
        } else { trailing_zeros_u128_spec(right) },
    ensures shift < 64,
        gcd(left as nat, right as nat) == pow2(shift as nat),
{
    trailing_zeros_factor_u64(left);
    trailing_zeros_factor_u128(right);
    power_of_two_has_unit_odd_part(left);
    let odd = (right >> trailing_zeros_u128_spec(right)) as nat;
    gcd_power_two_factors(1, odd, u64_trailing_zeros(left) as nat,
        trailing_zeros_u128_spec(right) as nat);
    gcd_symmetric(1, odd);
    assert(gcd(odd, 1) == gcd(1, 0));
    assert(gcd(1, odd) == 1);
    assert(1 * pow2(u64_trailing_zeros(left) as nat) == left) by (nonlinear_arith)
        requires left == (left >> u64_trailing_zeros(left)) * pow2(u64_trailing_zeros(left) as nat),
            (left >> u64_trailing_zeros(left)) == 1;
    assert(gcd(1, odd) * pow2(shift as nat) == pow2(shift as nat)) by (nonlinear_arith)
        requires gcd(1, odd) == 1;
}
}

/// Count zero bits using the two hardware-width halves of a `u128`.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures result == trailing_zeros_u128_spec(value), result <= 128,
        (result == 128 <==> value == 0),
))]
#[inline]
pub(crate) fn trailing_zeros_u128(value: u128) -> u32 {
    proof! { trailing_zeros_factor_u128(value); }
    let low = value as u64;
    if low != 0 {
        low.trailing_zeros()
    } else {
        64 + ((value >> 64) as u64).trailing_zeros()
    }
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures result == gcd(left as nat, right as nat),
    decreases right,
))]
const fn small_gcd(left: u8, right: u8) -> u8 {
    if right == 0 {
        left
    } else {
        small_gcd(right, left % right)
    }
}

// A bounded-depth traversal makes the compile-time initializer's contracts
// available to both Rust const evaluation and Verus's const-function proxy.
#[cfg_attr(verus_keep_ghost, verus_spec(
    requires start <= end <= 4096,
    ensures forall|index: int| 0 <= index < 4096 ==> (
        if start <= index < end {
            #[trigger] final(table)@[index] == gcd((index / 64) as nat, (index % 64) as nat)
        } else {
            final(table)@[index] == old(table)@[index]
        }),
    decreases end - start,
))]
const fn fill_small_gcd_table(table: &mut [u8; 4096], start: usize, end: usize) {
    if start < end {
        if end - start == 1 {
            table[start] = small_gcd((start / 64) as u8, (start % 64) as u8);
        } else {
            let middle = start + (end - start) / 2;
            fill_small_gcd_table(table, start, middle);
            fill_small_gcd_table(table, middle, end);
        }
    }
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures forall|index: int| 0 <= index < 4096 ==> #[trigger] result@[index]
        == gcd((index / 64) as nat, (index % 64) as nat),
))]
const fn build_small_gcd_table() -> [u8; 4096] {
    let mut table = [0_u8; 4096];
    fill_small_gcd_table(&mut table, 0, 4096);
    table
}

#[cfg_attr(verus_keep_ghost, verus_spec(
    ensures forall|index: int| 0 <= index < 4096 ==> #[trigger] SMALL_GCD_TABLE@[index]
        == gcd((index / 64) as nat, (index % 64) as nat),
))]
const SMALL_GCD_TABLE: [u8; 4096] = build_small_gcd_table();

/// Binary GCD keeps an odd left operand, removes powers of two from the right,
/// and subtracts the smaller value. The sum of the operands strictly decreases.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires input_left > 0, input_right > 0,
    ensures result == gcd(input_left as nat, input_right as nat),
))]
#[inline]
fn binary_gcd(input_left: u64, input_right: u64) -> u64 {
    let left_shift = input_left.trailing_zeros();
    let right_shift = input_right.trailing_zeros();
    let common_shift = left_shift.min(right_shift);
    let mut left = input_left >> left_shift;
    let mut right = input_right;
    proof! {
        trailing_zeros_factor_u64(input_left);
        trailing_zeros_factor_u64(input_right);
        let odd_right = (input_right >> right_shift) as nat;
        gcd_power_two_factors(left as nat, odd_right, left_shift as nat, right_shift as nat);
        gcd_odd_power_two(left as nat, odd_right, right_shift as nat);
        gcd_bound(input_left as nat, input_right as nat);
    }
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant
            left > 0, right > 0, left % 2 == 1,
            common_shift < 64,
            gcd(input_left as nat, input_right as nat) <= u64::MAX,
            gcd(left as nat, right as nat) * pow2(common_shift as nat)
                == gcd(input_left as nat, input_right as nat),
        decreases left as int + right as int,
    ))]
    loop {
        proof! { trailing_zeros_factor_u64(right); }
        let shift = right.trailing_zeros();
        proof! { gcd_odd_power_two(left as nat, (right >> shift) as nat, shift as nat); }
        right >>= shift;
        if left > right {
            proof! { gcd_symmetric(left as nat, right as nat); }
            core::mem::swap(&mut left, &mut right);
        }
        proof! { gcd_subtract(left as nat, right as nat); }
        right -= left;
        if right == 0 {
            proof! { vstd::bits::lemma_u64_shl_is_mul(left, common_shift as u64); }
            return left << common_shift;
        }
    }
}

/// The table and binary paths both return Euclid's greatest common divisor,
/// including the conventional zero cases.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures result == gcd(left as nat, right as nat),
))]
#[inline]
pub(crate) fn gcd_u64(left: u64, right: u64) -> u64 {
    if left == 0 {
        proof! { gcd_symmetric(left as nat, right as nat); }
        return right;
    }
    if right == 0 {
        return left;
    }
    if left < 64 && right < 64 {
        let index = left as usize * 64 + right as usize;
        proof! {
            lemma_fundamental_div_mod_converse(index as int, 64, left as int, right as int);
        }
        return SMALL_GCD_TABLE[index] as u64;
    }
    binary_gcd(left, right)
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires left > 0, right > 0,
    ensures result == gcd(left as nat, right as nat),
))]
#[inline]
fn gcd_small_wide(left: u64, right: u128) -> u128 {
    if left & (left - 1) == 0 {
        let shift = left.trailing_zeros().min(trailing_zeros_u128(right));
        proof! {
            gcd_power_of_two_word(left, right, shift);
            super::word::one_shift_is_power_of_two(shift);
        }
        return 1_u128 << shift;
    }
    proof! { gcd_symmetric(left as nat, right as nat); }
    gcd_u64(left, (right % left as u128) as u64) as u128
}

/// Exact GCD across the zero, table, binary, power-of-two and balanced two-limb
/// paths. Every helper precondition and reconstruction shift is discharged.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures result == gcd(left as nat, right as nat),
))]
#[inline]
pub(crate) fn gcd_u128(left: u128, right: u128) -> u128 {
    if left == 0 {
        proof! { gcd_symmetric(left as nat, right as nat); }
        return right;
    }
    if right == 0 {
        return left;
    }
    if left <= u64::MAX as u128 && right <= u64::MAX as u128 {
        return gcd_u64(left as u64, right as u64) as u128;
    }
    if left <= u64::MAX as u128 {
        return gcd_small_wide(left as u64, right);
    }
    if right <= u64::MAX as u128 {
        proof! { gcd_symmetric(left as nat, right as nat); }
        return gcd_small_wide(right as u64, left);
    }

    // Stop the u128 Euclidean reducer once its divisor fits a hardware word;
    // the remaining binary GCD avoids compiler-rt's two-limb remainder calls.
    let common_shift = trailing_zeros_u128(left).min(trailing_zeros_u128(right));
    proof! {
        common_shift_preserves_gcd(left, right, common_shift);
        gcd_bound(left as nat, right as nat);
    }
    let (larger, smaller) =
        super::division::reduce_gcd_to_word(left >> common_shift, right >> common_shift);
    if smaller == 0 {
        proof! { super::word::shift_left_u128_is_mul(larger, common_shift); }
        return larger << common_shift;
    }
    let remainder = (larger % smaller) as u64;
    let divisor = gcd_u64(smaller as u64, remainder) as u128;
    proof! {
        assert(divisor == gcd(larger as nat, smaller as nat));
        super::word::shift_left_u128_is_mul(divisor, common_shift);
    }
    divisor << common_shift
}
