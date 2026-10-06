//! Integer rounding and cache coarsening over mathematical integers.
//! The BigInt shift/add implementation and cache state remain to be refined.
use vstd::prelude::*;
use vstd::arithmetic::power2::*;
use vstd::arithmetic::div_mod::*;
use crate::magnitude_model::{binary_numerator, binary_denominator};
verus! {
pub(crate) open spec fn rounded_after_shift(value: int, preliminary_bits: nat) -> int {
    (value / pow2(preliminary_bits) as int + 1) / 2
}

/// Integer right shifts are modeled by Euclidean division, including negative
/// inputs. The strict lower endpoint determines halfway rounding toward +inf.
pub(crate) proof fn rounded_shift_error(value: int, preliminary_bits: nat)
    ensures
        -(pow2(preliminary_bits + 1) as int)
            < 2 * (rounded_after_shift(value, preliminary_bits) * pow2(preliminary_bits + 1) - value)
            <= pow2(preliminary_bits + 1),
{
    let unit = pow2(preliminary_bits) as int;
    let quotient = value / unit;
    let remainder = value % unit;
    let result = (quotient + 1) / 2;
    let parity = (quotient + 1) % 2;
    lemma_pow2_pos(preliminary_bits);
    lemma_pow2_unfold(preliminary_bits + 1);
    lemma_fundamental_div_mod(value, unit);
    lemma_mod_bound(value, unit);
    lemma_fundamental_div_mod(quotient + 1, 2);
    lemma_mod_bound(quotient + 1, 2);
    assert(-2 * unit < 2 * (result * (2 * unit) - value) <= 2 * unit) by (nonlinear_arith)
        requires unit > 0, 0 <= remainder < unit, 0 <= parity < 2,
            value == quotient * unit + remainder,
            quotient + 1 == 2 * result + parity;
}

pub(crate) open spec fn planned_approximation(value: int, round: bool, bits: u32) -> int {
    if round { rounded_after_shift(value, bits as nat) }
    else { value * pow2(bits as nat) }
}

/// The production plan's complete contract suffices for nearest integer
/// rounding at every public precision, without restricting the magnitude.
pub(crate) proof fn precision_plan_is_nearest(value: int, precision: i32, round: bool, bits: u32)
    requires
        round == (precision > 0),
        if precision <= 0 { bits as int == -(precision as int) }
        else { bits as int == precision as int - 1 },
    ensures
        -(binary_numerator(precision as int) as int)
            < 2 * (planned_approximation(value, round, bits) * binary_numerator(precision as int)
                - value * binary_denominator(precision as int))
            <= binary_numerator(precision as int),
{
    vstd::arithmetic::power::lemma_pow0(2);
    if round {
        rounded_shift_error(value, bits as nat);
        assert(binary_numerator(precision as int) == pow2(bits as nat + 1));
        assert(binary_denominator(precision as int) == 1);
        assert(-(binary_numerator(precision as int) as int)
            < 2 * (planned_approximation(value, round, bits) * binary_numerator(precision as int)
                - value * binary_denominator(precision as int))
            <= binary_numerator(precision as int)) by (nonlinear_arith)
            requires binary_numerator(precision as int) == pow2(bits as nat + 1),
                binary_denominator(precision as int) == 1,
                planned_approximation(value, round, bits) == rounded_after_shift(value, bits as nat),
                -(pow2(bits as nat + 1) as int) < 2 * (rounded_after_shift(value, bits as nat) * pow2(bits as nat + 1) - value) <= pow2(bits as nat + 1);
    } else {
        assert(binary_numerator(precision as int) == 1);
        assert(binary_denominator(precision as int) == pow2(bits as nat));
        assert(planned_approximation(value, round, bits) * binary_numerator(precision as int)
                - value * binary_denominator(precision as int) == 0) by (nonlinear_arith)
            requires binary_numerator(precision as int) == 1,
                binary_denominator(precision as int) == pow2(bits as nat),
                planned_approximation(value, round, bits) == value * pow2(bits as nat);
    }
}

/// Coarsening a cached value by at least one bit preserves its one-unit error
/// guarantee. The rational center and cached integer can have either sign.
pub(crate) proof fn cache_coarsening_preserves_unit_error(
    cached: int, numerator: int, denominator: nat, gap: nat,
)
    requires denominator > 0, gap > 0,
        -(denominator as int) <= cached * denominator - numerator <= denominator,
    ensures
        -(denominator * pow2(gap) as int)
            <= rounded_after_shift(cached, (gap - 1) as nat) * denominator * pow2(gap) - numerator
            <= denominator * pow2(gap),
{
    let result = rounded_after_shift(cached, (gap - 1) as nat);
    let factor = pow2(gap);
    rounded_shift_error(cached, (gap - 1) as nat);
    lemma_pow2_pos((gap - 1) as nat);
    lemma_pow2_unfold(gap);
    assert(factor >= 2);
    assert(-(denominator * factor as int) <= result * denominator * factor - numerator
        <= denominator * factor) by (nonlinear_arith)
        requires denominator > 0, factor >= 2,
            -(factor as int) < 2 * (result * factor - cached) <= factor,
            -(denominator as int) <= cached * denominator - numerator <= denominator;
}
}
