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

pub(crate) open spec fn strictly_approximates(value: real, approximation: int, unit: real) -> bool {
    unit > 0real
        && -unit < approximation as real * unit - value < unit
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

/// Mathematical counterpart of `scale`, including its signed right-shift
/// rounding. Connecting this specification to BigInt operations remains open.
pub(crate) open spec fn scaled_integer(value: int, shift: int) -> int {
    if shift >= 0 { value * pow2(shift as nat) }
    else { rounded_after_shift(value, (-shift - 1) as nat) }
}

pub(crate) proof fn binary_scaling_has_half_unit_error(value: int, shift: int)
    ensures -0.5real <= scaled_integer(value, shift) as real
        - value as real * binary_unit(shift) <= 0.5real,
{
    // Explicit exact-zero and halfway cases also make the rounding convention
    // independently visible to the arithmetic mutation checks.
    vstd::arithmetic::power::lemma_pow0(2);
    assert(scaled_integer(0, -1) == 0);
    assert(scaled_integer(1, -1) == 1);
    assert(scaled_integer(-1, -1) == 0);
    binary_unit_positive(shift);
    if shift >= 0 {
        cast_product(value, pow2(shift as nat) as int);
        assert(binary_denominator(shift) == 1);
        assert(binary_unit(shift) == pow2(shift as nat) as real) by (nonlinear_arith)
            requires binary_unit(shift) == pow2(shift as nat) as real / 1real;
        assert(scaled_integer(value, shift) as real == value as real * binary_unit(shift));
    } else {
        rounded_shift_real_error(value, (-shift - 1) as nat);
        let factor = pow2((-shift) as nat) as real;
        let result = scaled_integer(value, shift) as real;
        assert(-0.5real <= result - value as real / factor <= 0.5real)
            by (nonlinear_arith)
            requires factor > 0real,
                -factor < 2real * (result * factor - value as real) <= factor;
        assert(binary_unit(shift) == 1real / factor);
        assert(result - value as real * binary_unit(shift) == result - value as real / factor)
            by (nonlinear_arith)
            requires factor > 0real, binary_unit(shift) == 1real / factor;
    }
}

/// A cached integer's bit length supplies the upper bound required by the
/// multiplication anchor even when its binade differs from the exact value's.
pub(crate) proof fn cached_bit_length_bounds_real(value: real, approximation: int, precision: int, bits: nat)
    requires approximates(value, approximation, binary_unit(precision)),
        -(pow2(bits) as int) < approximation < pow2(bits),
    ensures -binary_unit(precision + bits) <= value <= binary_unit(precision + bits),
{
    binary_unit_positive(precision);
    binary_unit_composes(precision, bits as int);
    vstd::arithmetic::power::lemma_pow0(2);
    let factor = pow2(bits) as real;
    assert(binary_unit(bits as int) == factor) by (nonlinear_arith)
        requires binary_unit(bits as int) == factor / 1real;
    assert(-factor + 1real <= approximation as real <= factor - 1real);
    assert(-binary_unit(precision + bits) <= value <= binary_unit(precision + bits))
        by (nonlinear_arith)
        requires binary_unit(precision) > 0real,
            binary_unit(precision + bits) == binary_unit(precision) * factor,
            -factor + 1real <= approximation as real <= factor - 1real,
            -binary_unit(precision) <= approximation as real * binary_unit(precision) - value
                <= binary_unit(precision);
}

proof fn bounded_real_product(left: real, right: real, left_bound: real, right_bound: real)
    requires left_bound >= 0real, right_bound >= 0real,
        -left_bound <= left <= left_bound, -right_bound <= right <= right_bound,
    ensures -(left_bound * right_bound) <= left * right <= left_bound * right_bound,
{
    assert(-(left_bound * right_bound) <= left * right <= left_bound * right_bound)
        by (nonlinear_arith)
        requires left_bound >= 0real, right_bound >= 0real,
            -left_bound <= left <= left_bound, -right_bound <= right <= right_bound;
}

/// Three guard bits leave a quarter of the requested unit for each operand's
/// error. The known operand needs only this upper bound, not an exact binade.
pub(crate) proof fn asymmetric_product_before_rounding(
    left: real, right: real, a: int, b: int, left_unit: real, right_unit: real,
    left_bound: real, right_bound: real, unit: real,
)
    requires unit > 0real, left_bound > 0real, right_bound > 0real,
        approximates(left, a, left_unit), approximates(right, b, right_unit),
        -2real * left_bound <= left <= 2real * left_bound,
        -2real * right_bound <= b as real * right_unit <= 2real * right_bound,
        8real * left_unit * right_bound == unit,
        8real * right_unit * left_bound == unit,
    ensures -unit / 2real <= (a * b) as real * (left_unit * right_unit) - left * right
        <= unit / 2real,
        (-2real * right_bound < b as real * right_unit < 2real * right_bound)
            ==> -unit / 2real < (a * b) as real * (left_unit * right_unit) - left * right
                < unit / 2real,
{
    let first_error = a as real * left_unit - left;
    let second_error = b as real * right_unit - right;
    bounded_real_product(first_error, b as real * right_unit, left_unit, 2real * right_bound);
    bounded_real_product(left, second_error, 2real * left_bound, right_unit);
    cast_product(a, b);
    assert((a * b) as real * (left_unit * right_unit) - left * right
        == first_error * (b as real * right_unit) + left * second_error) by (nonlinear_arith)
        requires (a * b) as real == a as real * b as real,
            first_error == a as real * left_unit - left,
            second_error == b as real * right_unit - right;
    assert(-unit / 2real <= (a * b) as real * (left_unit * right_unit) - left * right
        <= unit / 2real) by (nonlinear_arith)
        requires 8real * left_unit * right_bound == unit,
            8real * right_unit * left_bound == unit,
            -(left_unit * (2real * right_bound)) <= first_error * (b as real * right_unit)
                <= left_unit * (2real * right_bound),
            -((2real * left_bound) * right_unit) <= left * second_error
                <= (2real * left_bound) * right_unit,
            (a * b) as real * (left_unit * right_unit) - left * right
                == first_error * (b as real * right_unit) + left * second_error;
    if -2real * right_bound < b as real * right_unit < 2real * right_bound {
        assert(-(left_unit * (2real * right_bound)) < first_error * (b as real * right_unit)
            < left_unit * (2real * right_bound)) by (nonlinear_arith)
            requires left_unit > 0real, right_bound > 0real,
                -left_unit <= first_error <= left_unit,
                -2real * right_bound < b as real * right_unit < 2real * right_bound;
        assert(-unit / 2real < (a * b) as real * (left_unit * right_unit) - left * right
            < unit / 2real) by (nonlinear_arith)
            requires 8real * left_unit * right_bound == unit,
                8real * right_unit * left_bound == unit,
                -(left_unit * (2real * right_bound)) < first_error * (b as real * right_unit)
                    < left_unit * (2real * right_bound),
                -((2real * left_bound) * right_unit) <= left * second_error
                    <= (2real * left_bound) * right_unit,
                (a * b) as real * (left_unit * right_unit) - left * right
                    == first_error * (b as real * right_unit) + left * second_error;
    }
}

proof fn three_guard_bits(precision: int, magnitude: int)
    ensures 8real * binary_unit(precision - magnitude - 3) * binary_unit(magnitude)
        == binary_unit(precision),
{
    lemma2_to64();
    vstd::arithmetic::power::lemma_pow0(2);
    binary_unit_composes(precision - magnitude - 3, magnitude);
    binary_unit_composes(precision - 3, 3);
    assert(binary_unit(3) == 8real) by (nonlinear_arith)
        requires binary_numerator(3) == 8, binary_denominator(3) == 1,
            binary_unit(3) == binary_numerator(3) as real / binary_denominator(3) as real;
    assert(8real * binary_unit(precision - magnitude - 3) * binary_unit(magnitude)
        == binary_unit(precision)) by (nonlinear_arith)
        requires binary_unit(precision - magnitude - 3) * binary_unit(magnitude)
                == binary_unit(precision - 3),
            binary_unit(precision - 3) * 8real == binary_unit(precision);
}

/// The asymmetric production schedule, with unbounded precision arithmetic
/// and input approximation/size obligations made explicit.
pub(crate) proof fn asymmetric_multiplication_preserves_error(
    left: real, right: real, a: int, b: int, precision: int,
    left_magnitude: int, right_magnitude: int,
)
    requires
        approximates(right, b, binary_unit(precision - left_magnitude - 3)),
        approximates(left, a, binary_unit(precision - right_magnitude - 3)),
        -2real * binary_unit(left_magnitude) <= left <= 2real * binary_unit(left_magnitude),
        -2real * binary_unit(right_magnitude)
            <= b as real * binary_unit(precision - left_magnitude - 3)
            <= 2real * binary_unit(right_magnitude),
    ensures approximates(left * right,
        scaled_integer(a * b, precision - left_magnitude - right_magnitude - 6),
        binary_unit(precision)),
        (-2real * binary_unit(right_magnitude)
            < b as real * binary_unit(precision - left_magnitude - 3)
            < 2real * binary_unit(right_magnitude))
            ==> strictly_approximates(left * right,
                scaled_integer(a * b, precision - left_magnitude - right_magnitude - 6),
                binary_unit(precision)),
{
    let lp = precision - right_magnitude - 3;
    let rp = precision - left_magnitude - 3;
    let shift = lp + rp - precision;
    binary_unit_positive(precision); binary_unit_positive(left_magnitude);
    binary_unit_positive(right_magnitude);
    three_guard_bits(precision, left_magnitude);
    three_guard_bits(precision, right_magnitude);
    asymmetric_product_before_rounding(left, right, a, b, binary_unit(lp), binary_unit(rp),
        binary_unit(left_magnitude), binary_unit(right_magnitude), binary_unit(precision));
    binary_scaling_has_half_unit_error(a * b, shift);
    binary_unit_composes(lp, rp);
    binary_unit_composes(shift, precision);
    let product = (a * b) as real;
    let result = scaled_integer(a * b, shift) as real;
    let unit = binary_unit(precision);
    assert(-unit <= result * unit - left * right <= unit) by (nonlinear_arith)
        requires unit > 0real,
            -0.5real <= result - product * binary_unit(shift) <= 0.5real,
            -unit / 2real <= product * (binary_unit(lp) * binary_unit(rp)) - left * right
                <= unit / 2real,
            binary_unit(shift) * unit == binary_unit(lp) * binary_unit(rp);
    if -2real * binary_unit(right_magnitude) < b as real * binary_unit(rp)
        < 2real * binary_unit(right_magnitude) {
        assert(-unit < result * unit - left * right < unit) by (nonlinear_arith)
            requires unit > 0real,
                -0.5real <= result - product * binary_unit(shift) <= 0.5real,
                -unit / 2real < product * (binary_unit(lp) * binary_unit(rp)) - left * right
                    < unit / 2real,
                binary_unit(shift) * unit == binary_unit(lp) * binary_unit(rp);
    }
}

pub(crate) proof fn asymmetric_zero_product_preserves_error(
    left: real, right: real, precision: int, left_magnitude: int,
)
    requires approximates(right, 0, binary_unit(precision - left_magnitude - 3)),
        -2real * binary_unit(left_magnitude) <= left <= 2real * binary_unit(left_magnitude),
    ensures approximates(left * right, 0, binary_unit(precision)),
{
    binary_unit_positive(precision); binary_unit_positive(left_magnitude);
    three_guard_bits(precision, left_magnitude);
    let bound = binary_unit(left_magnitude);
    let other_unit = binary_unit(precision - left_magnitude - 3);
    assert(-other_unit <= right <= other_unit) by (nonlinear_arith)
        requires -other_unit <= 0real * other_unit - right <= other_unit;
    bounded_real_product(left, right, 2real * bound, other_unit);
    assert(-binary_unit(precision) <= -left * right <= binary_unit(precision)) by (nonlinear_arith)
        requires binary_unit(precision) > 0real, 8real * other_unit * bound == binary_unit(precision),
            -(2real * bound * other_unit) <= left * right <= 2real * bound * other_unit;
    assert(approximates(left * right, 0, binary_unit(precision))) by (nonlinear_arith)
        requires binary_unit(precision) > 0real,
            -binary_unit(precision) <= -left * right <= binary_unit(precision);
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

/// The comparison kernel's two-integer gap certifies strict order only with
/// strict input errors. An inclusive one-unit contract alone is insufficient.
pub(crate) proof fn separated_strict_approximations_certify_order(left: real, right: real, a: int, b: int, unit: real)
    requires strictly_approximates(left, a, unit), strictly_approximates(right, b, unit),
    ensures a >= b + 2 ==> left > right,
        b >= a + 2 ==> right > left,
{
    if a >= b + 2 {
        assert(left > right) by (nonlinear_arith)
            requires unit > 0real, a as real >= b as real + 2real,
                -unit < a as real * unit - left < unit,
                -unit < b as real * unit - right < unit;
    }
    if b >= a + 2 {
        assert(right > left) by (nonlinear_arith)
            requires unit > 0real, b as real >= a as real + 2real,
                -unit < a as real * unit - left < unit,
                -unit < b as real * unit - right < unit;
    }
}

/// A strictly finer valid cache entry cannot lose the nonzero sign established
/// by a previous approximation separated from zero. This supports the two
/// cache reads in MSD queries, conditional on the cache refinement invariant.
pub(crate) proof fn finer_cache_preserves_separated_sign(
    value: real, cached: int, refined: int, old_unit: real, new_unit: real,
)
    requires approximates(value, cached, old_unit), approximates(value, refined, new_unit),
        new_unit < old_unit,
    ensures cached > 1 ==> refined > 0,
        cached < -1 ==> refined < 0,
{
    if cached > 1 {
        assert(refined as real > 0real) by (nonlinear_arith)
            requires old_unit > new_unit > 0real, cached as real >= 2real,
                -old_unit <= cached as real * old_unit - value <= old_unit,
                -new_unit <= refined as real * new_unit - value <= new_unit;
    }
    if cached < -1 {
        assert((refined as real) < 0real) by (nonlinear_arith)
            requires old_unit > new_unit > 0real, cached as real <= -2real,
                -old_unit <= cached as real * old_unit - value <= old_unit,
                -new_unit <= refined as real * new_unit - value <= new_unit;
    }
}
}
