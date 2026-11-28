//! Exact native GCD dispatch, binary-factor helpers and their shared proof identities.
//!
//! The u64 entry point handles zero, a table of small pairs and binary subtraction.
//! The u128 entry point reuses that path, specializes mixed-width inputs and reduces
//! two wide operands to a machine-word divisor before finishing. Each path returns
//! the same unsigned mathematical GCD, with gcd(0,0)=0. Rational reduction,
//! fixed-limb GCD and dyadic normalization consume these arithmetic facts.
//!
//! The crate-visible entry points accept every value of their native types.
//! Private binary and mixed-width helpers require positive inputs, and partial
//! table filling requires a valid index range; those contracts are not checked
//! error interfaces. The table is built by const evaluation, without runtime lazy
//! initialization or a mutable cache. Runtime arithmetic allocates no heap and
//! has no configurable work/cancellation budget or exhaustion outcome.
//!
//! Ghost identities use the Euclidean GCD specification to justify cancellation,
//! odd parts, exact shifts and reconstruction. Ordinary builds erase those proofs;
//! source contracts and documentation do not substitute for a fresh verifier run.

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
/// Prove symmetry of the Euclidean GCD for arbitrary natural inputs, including zero.
///
/// For unequal inputs, the smaller is the first remainder after swapping; equal
/// inputs need no rewrite. Consumers use this identity when ordering magnitudes
/// or choosing a mixed-width arithmetic path.
pub(crate) proof fn gcd_symmetric(left: nat, right: nat)
    ensures gcd(left, right) == gcd(right, left),
{
    if left < right {
        lemma_small_mod(left, right);
    } else if right < left {
        lemma_small_mod(right, left);
    }
}

/// Preserve GCD when subtracting a positive smaller operand from the larger.
///
/// Requires `0 < left <= right`. Symmetry and subtraction of a multiple give
/// `gcd(left, right-left) == gcd(left, right)`, including a zero difference for
/// equal operands. The hypotheses establish nonnegative subtraction, not that
/// the difference is positive for a subsequent trailing-zero removal.
pub(crate) proof fn gcd_subtract(left: nat, right: nat)
    requires 0 < left <= right,
    ensures gcd(left, (right - left) as nat) == gcd(left, right),
{
    gcd_symmetric(left, right);
    gcd_symmetric(left, (right - left) as nat);
    lemma_mod_sub_multiples_vanish(right as int, left as int);
}

/// Factor a shared positive multiplier out of the GCD of natural operands.
///
/// Requires factor positive; either operand may be zero. Euclidean induction
/// scales each quotient/remainder relation to prove
/// `gcd(left*factor, right*factor) == gcd(left,right)*factor`. This mathematical
/// identity places no native-width bound on the products.
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

/// Establish exact reduced fraction parts after cancelling their complete GCD.
///
/// Requires a positive denominator and permits numerator zero. The GCD is positive,
/// both divisions reconstruct their originals exactly, the new denominator stays
/// positive, neither part increases, and the resulting parts are coprime. Cross
/// multiplication preserves the fraction. A zero numerator remains zero and gives
/// denominator one. The lemma does not define a fraction with denominator zero.
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

/// Bound the GCD by each operand that is positive.
///
/// Admits all natural inputs. The bounds are conditional because gcd(0,right)
/// may exceed the zero operand; both-zero GCD is still zero. These inequalities
/// justify representable reconstruction shifts and fixed-buffer capacity.
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

/// Prove that a positive divisor of an odd natural number is odd.
///
/// Exact divisibility reconstructs the value as quotient times divisor. An even
/// divisor would make the product even, contradicting the input parity. Used to
/// show that a GCD with an odd operand has no factor of two.
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

/// Cancel a factor of two when testing divisibility by a positive odd divisor.
///
/// If divisor divides `2*value`, parity forces the corresponding quotient to be
/// even, so divisor also divides value. Value may be zero. This supplies the
/// reverse divisibility implication needed when removing binary factors from GCD.
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

/// Show that doubling the other operand does not change a GCD with an odd operand.
///
/// Requires left odd, hence positive; right may be zero or even. Common-divisor
/// characterization and cancellation of two through odd divisors establish the
/// two inequalities and therefore equality of the GCDs.
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

/// Ignore any power of two multiplying the other operand of an odd-input GCD.
///
/// Requires left odd, while right and shift are arbitrary naturals. Induction on
/// shift applies the doubling identity, including shift zero and right zero.
/// This justifies stripping binary factors from a positive residual without
/// requiring that both residuals already be odd.
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

/// Restore the smaller binary exponent when its associated residual is odd.
///
/// Requires left odd and `low <= high`; right need not be odd or nonzero. Factors
/// out `2^low` and removes the remaining power of two from the other argument,
/// yielding `gcd(left,right)*2^low`. This is the ordered case used by the symmetric
/// two-factor lemma, not a native shift or capacity guarantee.
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

/// Reconstruct GCD from two odd residuals and their separate binary exponents.
///
/// Both residuals must be odd, so neither is zero. The result is their GCD times
/// the power of two at the smaller exponent. Choosing the ordered proof and
/// transporting symmetry covers either exponent order without changing meaning.
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

/// Connect a positive u64's trailing-zero count to its exact positive odd quotient.
///
/// Requires value nonzero, giving a count below 64. The shifted quotient is odd,
/// positive and no greater than value, and multiplying it by the removed power
/// reconstructs value exactly. This lemma does not cover the zero sentinel count
/// or authorize shifting a runtime u64 by 64.
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

/// Specify a u128 trailing-zero count through its low and high u64 halves.
///
/// A nonzero low half supplies the count directly. Otherwise adds 64 to the high
/// half's count, giving 128 exactly for zero. This ghost definition mirrors the
/// runtime helper; the zero count is a sentinel, not a valid u128 shift amount.
pub(crate) open spec fn trailing_zeros_u128_spec(value: u128) -> u32 {
    if value as u64 != 0 {
        u64_trailing_zeros(value as u64)
    } else {
        (64 + u64_trailing_zeros((value >> 64) as u64)) as u32
    }
}

/// Prove the full count range and nonzero factorization of the two-half u128 count.
///
/// All inputs are admitted: the count is at most 128 and equals 128 exactly for
/// zero. Only positive inputs are shifted in the factorization claim; their
/// quotient is positive, odd and reconstructs the original by an exact power of
/// two. Low-half and high-half cases are connected to the u64 bit specification.
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

/// Identify one as the odd part of a positive u64 having a single set bit.
///
/// Requires both positivity and the clear-lowest-bit test to yield zero. The
/// trailing-zero specification then makes the shifted value exactly one. Zero
/// is excluded even though a wrapping version of the bit test could accept it.
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

/// Cancel any available common binary factor from two positive u128 operands.
///
/// Requires shift no greater than either trailing-zero count. Proves shift below
/// 128 and reconstructs the original GCD from that of the shifted operands times
/// `2^shift`. Removing less than a full trailing count may leave an even quotient;
/// the identity does not require or conclude that both outputs are odd.
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

/// Remove an available binary factor from a positive u128 without losing information.
///
/// Requires shift no greater than the value's trailing-zero count. Proves a valid
/// shift below 128, a positive quotient and exact reconstruction. The remaining
/// quotient need not be odd unless all trailing factors were removed. Used by
/// dyadic normalization and shared-factor GCD reduction.
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

/// Reduce a positive u64 power of two against a positive u128 to a pure binary GCD.
///
/// Requires the one-set-bit test on left and shift equal to the minimum of both
/// trailing-zero counts. The GCD is exactly `2^shift`, with shift below 64 because
/// left fits u64. This is the proof for the mixed-width power-of-two shortcut.
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

/// Count low zero bits of any u128 using its two u64 halves.
///
/// Returns a count in 0..=128, with 128 exactly for zero. Inspects the low half
/// first; when it is zero, adds 64 to the high half's count. For nonzero input,
/// removing this binary factor yields a positive odd integer. Callers must handle
/// the zero sentinel before using the count as a runtime shift; no allocation,
/// normalization or input validation occurs here.
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

/// Compute the exact GCD of any two u8 values by const-capable Euclidean recursion.
///
/// Returns left at a zero right operand, including zero for two zeros; otherwise
/// recurses on right and its smaller remainder. The domain is all u8 pairs even
/// though table construction only calls it with values below 64. No lookup table
/// is consulted and the nonzero guard prevents remainder by zero.
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

/// Fill a selected half-open table range with GCDs, preserving every other entry.
///
/// Requires `start <= end <= 4096`. Entry i receives the GCD of row `i/64` and
/// column `i%64`, each below 64; an empty range is unchanged. Splits nontrivial
/// ranges at their midpoint and evaluates small_gcd at single-entry leaves.
/// Balanced recursion keeps table-traversal depth logarithmic rather than a chain
/// of one call per entry, apart from the Euclidean recursion inside each leaf.
///
/// The same const function is available to Rust initialization and the Verus
/// const-function proxy. Bounds are caller obligations, not typed validation:
/// invalid reversed ranges can do nothing and out-of-bounds leaves can panic.
/// No heap allocation or runtime cache initialization is performed.
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

/// Construct all 4,096 small-pair GCD entries using the balanced const fill.
///
/// Starts from a zeroed u8 array and fills its entire valid range. Entry `a*64+b`
/// is gcd(a,b) for a,b below 64, including zero rows/columns and gcd(0,0)=0.
/// Returns the complete array for constant initialization without mutable global
/// state; it does not select runtime dispatch or accept external dimensions.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures forall|index: int| 0 <= index < 4096 ==> #[trigger] result@[index]
        == gcd((index / 64) as nat, (index % 64) as nat),
))]
const fn build_small_gcd_table() -> [u8; 4096] {
    let mut table = [0_u8; 4096];
    fill_small_gcd_table(&mut table, 0, 4096);
    table
}

/// Const-evaluated row-major GCD values for every pair in 0..64.
///
/// Contains 4,096 u8 entries indexed by `left*64+right`; the greatest entry is 63.
/// The u64 dispatcher uses it only after zero handling and bounds checks. This
/// is an immutable constant, not a lazy cache. The array's logical size does not
/// establish how often a compiler emits it or its final shipped binary footprint.
#[cfg_attr(verus_keep_ghost, verus_spec(
    ensures forall|index: int| 0 <= index < 4096 ==> #[trigger] SMALL_GCD_TABLE@[index]
        == gcd((index / 64) as nat, (index % 64) as nat),
))]
const SMALL_GCD_TABLE: [u8; 4096] = build_small_gcd_table();

/// Compute a positive u64 pair's exact GCD by odd normalization and subtraction.
///
/// Requires both inputs positive. Retains their common trailing-zero count, makes
/// left odd, and repeatedly removes right's powers of two, orders the operands
/// and subtracts the smaller from the larger. A zero difference ends the loop
/// before another trailing-zero shift. Restoring the common factor is lossless
/// because the GCD fits each original positive operand.
///
/// The residual sum strictly decreases on continued iterations. There is no
/// runtime positivity check: zero can lead to a shift by 64. The caller handles
/// zero first. No allocation, approximate result or budget-exhaustion outcome
/// occurs in this private helper.
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

/// Return the exact unsigned GCD of every u64 pair, including conventional zero cases.
///
/// Checks left zero first, then right zero; either returns the other value and
/// two zeros return zero. Positive pairs below 64 use the row-major table; all
/// other positive pairs use binary_gcd. Both paths preserve Euclid's value and
/// the result cannot exceed either positive input. No input is mutated, no heap
/// is allocated, and no fallible or approximate outcome is introduced.
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

/// Compute a positive u64/u128 pair's GCD with a power-of-two shortcut.
///
/// Requires both inputs positive, without requiring right to exceed u64::MAX.
/// A one-set-bit left operand returns the power of two at the smaller trailing
/// count. Otherwise takes right modulo left, whose remainder fits u64, and calls
/// gcd_u64 with the left word and that remainder. The complete result fits left.
///
/// Preconditions are not runtime validation; left zero can underflow the bit test,
/// while some zero-right calls may happen to return a value outside the declared
/// domain. The u128 dispatcher handles both zero cases before selecting this helper.
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

/// Return the exact unsigned GCD of every u128 pair through native-width specializations.
///
/// Checks left zero then right zero. If both positive operands fit u64, delegates
/// to gcd_u64; if only one does, passes that operand first to gcd_small_wide.
/// When both exceed u64::MAX, removes their minimum trailing-zero count and asks
/// the shared Euclidean reducer for an ordered pair with a u64-sized second value.
///
/// If that second value is zero, the first is the reduced GCD. Otherwise one
/// remainder supplies a u64 pair for gcd_u64. Restoring the saved binary factor
/// is exact and fits u128 because the result is a GCD of the original operands.
/// The shifted operands need not both be odd. All helper domains are established
/// before the corresponding call; no errors, heap allocation, approximation or
/// configurable work/cancellation result is exposed by this entry point.
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
