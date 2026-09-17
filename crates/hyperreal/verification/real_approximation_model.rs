//! Compositional error bounds for exact real denotations.
//! Production graph, BigInt, precision arithmetic and cache refinement remain open.
use vstd::prelude::*;
use vstd::arithmetic::power2::*;
use crate::magnitude_model::{binary_numerator, binary_denominator, binary_scales_compose};
use crate::integer_approximation_model::{rounded_after_shift, rounded_shift_error, planned_approximation, precision_plan_is_nearest};
verus! {
pub(crate) open spec fn binary_unit(precision: int) -> real {
    binary_numerator(precision) as real / binary_denominator(precision) as real
}

pub(crate) open spec fn approximates(value: real, approximation: int, unit: real) -> bool {
    unit > 0real
        && -unit <= approximation as real * unit - value <= unit
}

pub(crate) proof fn binary_unit_positive(precision: int)
    ensures binary_unit(precision) > 0real,
        binary_numerator(precision) > 0, binary_denominator(precision) > 0,
{
    lemma_pow2_pos(if precision >= 0 { precision as nat } else { 0 });
    lemma_pow2_pos(if precision < 0 { (-precision) as nat } else { 0 });
    assert(binary_unit(precision) > 0real) by (nonlinear_arith)
        requires binary_numerator(precision) as real > 0real,
            binary_denominator(precision) as real > 0real;
}

proof fn cast_product(left: int, right: int)
    ensures (left * right) as real == left as real * right as real,
{
    assert((left * right) as real == left as real * right as real) by (nonlinear_arith);
}

pub(crate) proof fn binary_unit_composes(left: int, right: int)
    ensures binary_unit(left + right) == binary_unit(left) * binary_unit(right),
{
    binary_unit_positive(left); binary_unit_positive(right); binary_unit_positive(left + right);
    binary_scales_compose(left, right);
    let a = binary_numerator(left) as real; let b = binary_denominator(left) as real;
    let c = binary_numerator(right) as real; let d = binary_denominator(right) as real;
    let e = binary_numerator(left + right) as real; let f = binary_denominator(left + right) as real;
    cast_product(binary_denominator(left) as int, binary_denominator(right) as int);
    cast_product(binary_numerator(left + right) as int, (binary_denominator(left) * binary_denominator(right)) as int);
    cast_product(binary_numerator(left) as int, binary_numerator(right) as int);
    cast_product((binary_numerator(left) * binary_numerator(right)) as int, binary_denominator(left + right) as int);
    assert(e * (b * d) == (a * c) * f);
    assert(e / f == (a / b) * (c / d)) by (nonlinear_arith)
        requires b > 0real, d > 0real, f > 0real, e * (b * d) == (a * c) * f;
}

proof fn rounded_shift_real_error(value: int, preliminary_bits: nat)
    ensures -(pow2(preliminary_bits + 1) as real)
        < 2real * (rounded_after_shift(value, preliminary_bits) as real * pow2(preliminary_bits + 1) as real - value as real)
        <= pow2(preliminary_bits + 1) as real,
{
    rounded_shift_error(value, preliminary_bits);
    let result = rounded_after_shift(value, preliminary_bits);
    let factor = pow2(preliminary_bits + 1);
    cast_product(result, factor as int);
    assert(-(factor as real) < 2real * (result as real * factor as real - value as real) <= factor as real);
}

/// Relate the executable integer plan's postcondition to an exact real leaf.
pub(crate) proof fn integer_plan_denotes_real(value: int, precision: i32, round: bool, bits: u32)
    requires round == (precision > 0),
        if precision <= 0 { bits as int == -(precision as int) }
        else { bits as int == precision as int - 1 },
    ensures approximates(value as real, planned_approximation(value, round, bits), binary_unit(precision as int)),
{
    precision_plan_is_nearest(value, precision, round, bits);
    binary_unit_positive(precision as int);
    let result = planned_approximation(value, round, bits);
    let numerator = binary_numerator(precision as int);
    let denominator = binary_denominator(precision as int);
    cast_product(result, numerator as int);
    cast_product(value, denominator as int);
    assert(-(numerator as real) < 2real * (result as real * numerator as real - value as real * denominator as real) <= numerator as real);
    let unit = binary_unit(precision as int);
    assert(-unit <= result as real * unit - value as real <= unit) by (nonlinear_arith)
        requires numerator as real > 0real, denominator as real > 0real,
            unit == numerator as real / denominator as real,
            -(numerator as real) < 2real * (result as real * numerator as real - value as real * denominator as real) <= numerator as real;
}

pub(crate) proof fn negation_preserves_error(value: real, approximation: int, unit: real)
    requires approximates(value, approximation, unit),
    ensures approximates(-value, -approximation, unit),
{
    assert(-unit <= (-approximation) as real * unit + value <= unit) by (nonlinear_arith)
        requires unit > 0real, -unit <= approximation as real * unit - value <= unit;
}

pub(crate) proof fn binary_offset_preserves_error(value: real, approximation: int, precision: int, offset: int)
    requires approximates(value, approximation, binary_unit(precision - offset)),
    ensures approximates(value * binary_unit(offset), approximation, binary_unit(precision)),
{
    binary_unit_positive(precision - offset); binary_unit_positive(offset);
    binary_unit_composes(precision - offset, offset);
    assert(approximates(value * binary_unit(offset), approximation, binary_unit(precision))) by (nonlinear_arith)
        requires binary_unit(precision - offset) > 0real, binary_unit(offset) > 0real,
            binary_unit(precision) == binary_unit(precision - offset) * binary_unit(offset),
            -binary_unit(precision - offset) <= approximation as real * binary_unit(precision - offset) - value <= binary_unit(precision - offset);
}

pub(crate) proof fn addition_with_guard_bits_preserves_error(left: real, right: real, a: int, b: int, unit: real)
    requires approximates(left, a, unit / 4real), approximates(right, b, unit / 4real),
    ensures approximates(left + right, rounded_after_shift(a + b, 1), unit),
{
    rounded_shift_real_error(a + b, 1);
    lemma2_to64();
    let r = rounded_after_shift(a + b, 1) as real;
    assert(-unit <= r * unit - (left + right) <= unit) by (nonlinear_arith)
        requires unit > 0real,
            -4real < 2real * (r * 4real - (a as real + b as real)) <= 4real,
            -unit / 4real <= a as real * (unit / 4real) - left <= unit / 4real,
            -unit / 4real <= b as real * (unit / 4real) - right <= unit / 4real;
}

pub(crate) proof fn cache_coarsening_preserves_real_error(value: real, cached: int, unit: real, gap: nat)
    requires gap > 0, approximates(value, cached, unit),
    ensures approximates(value, rounded_after_shift(cached, (gap - 1) as nat), unit * pow2(gap) as real),
{
    rounded_shift_real_error(cached, (gap - 1) as nat);
    lemma_pow2_pos((gap - 1) as nat); lemma_pow2_unfold(gap);
    let factor = pow2(gap) as real;
    let result = rounded_after_shift(cached, (gap - 1) as nat) as real;
    assert(factor >= 2real);
    assert(unit * factor > 0real && -(unit * factor) <= result * (unit * factor) - value <= unit * factor) by (nonlinear_arith)
        requires unit > 0real, factor >= 2real,
            -factor < 2real * (result * factor - cached as real) <= factor,
            -unit <= cached as real * unit - value <= unit;
}

pub(crate) proof fn near_integer_is_adjacent(value: real, approximation: int)
    requires approximates(value, approximation, 0.25real),
    ensures rounded_after_shift(approximation, 1) == value.floor()
        || rounded_after_shift(approximation, 1) == -(-value).floor(),
{
    rounded_shift_real_error(approximation, 1); lemma2_to64();
    let result = rounded_after_shift(approximation, 1);
    assert(-1real < result as real - value < 1real);
    let lower = value.floor(); let upper = -(-value).floor();
    assert(lower <= result <= upper);
    assert(upper <= lower + 1);
}

pub(crate) proof fn separated_approximation_certifies_sign(value: real, approximation: int, unit: real)
    requires approximates(value, approximation, unit),
    ensures approximation > 1 ==> value > 0real,
        approximation < -1 ==> value < 0real,
{
    assert(approximation > 1 ==> value > 0real) by (nonlinear_arith)
        requires unit > 0real, -unit <= approximation as real * unit - value <= unit;
    assert(approximation < -1 ==> value < 0real) by (nonlinear_arith)
        requires unit > 0real, -unit <= approximation as real * unit - value <= unit;
}
}
