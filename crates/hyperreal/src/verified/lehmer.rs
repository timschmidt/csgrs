//! Lehmer batch selection and signed row coefficients shared with wide GCD.
//! The matrix algebra is verified for arbitrary mathematical magnitudes;
//! the production BigUint extraction and row arithmetic remain separate.

#[cfg(verus_keep_ghost)]
use super::division::{gcd, gcd_characterization};
#[cfg(verus_keep_ghost)]
use vstd::arithmetic::{div_mod::*, power2::*};
#[cfg(verus_keep_ghost)]
use vstd::{math::abs, prelude::*};

#[cfg(verus_keep_ghost)]
verus! {
pub(crate) open spec fn determinant(a: int, b: int, c: int, d: int) -> int {
    a * d - b * c
}

proof fn abs_divisibility(value: int, divisor: nat)
    requires divisor > 0,
    ensures abs(value) % divisor == 0 <==> value % (divisor as int) == 0,
{
    if value < 0 {
        lemma_mul_mod_noop_right(-1, value, divisor as int);
        lemma_mul_mod_noop_right(-1, -value, divisor as int);
        lemma_small_mod(0, divisor);
    }
}

proof fn linear_combination_divisible(left: int, right: int, a: int, b: int, divisor: nat)
    requires divisor > 0, left % (divisor as int) == 0, right % (divisor as int) == 0,
    ensures (a * left + b * right) % (divisor as int) == 0,
{
    lemma_fundamental_div_mod(left, divisor as int);
    lemma_fundamental_div_mod(right, divisor as int);
    let l = left / (divisor as int);
    let r = right / (divisor as int);
    assert(a * left + b * right == (a * l + b * r) * divisor) by (nonlinear_arith)
        requires left == divisor * l, right == divisor * r;
    lemma_mod_multiples_basic(a * l + b * r, divisor as int);
}

/// A unit determinant supplies an integer inverse, so taking the magnitudes
/// of the transformed rows preserves exactly the original greatest divisor.
pub(crate) proof fn unimodular_preserves_gcd(left: nat, right: nat, a: int, b: int, c: int, d: int)
    requires determinant(a, b, c, d) == 1 || determinant(a, b, c, d) == -1,
    ensures gcd(abs(a * left + b * right), abs(c * left + d * right)) == gcd(left, right),
{
    let first = a * left + b * right;
    let second = c * left + d * right;
    let det = determinant(a, b, c, d);
    assert(d * (a * left + b * right) - b * (c * left + d * right)
        == (a * d - b * c) * left) by (nonlinear_arith);
    assert(-c * (a * left + b * right) + a * (c * left + d * right)
        == (a * d - b * c) * right) by (nonlinear_arith);
    assert(abs(det * left) == left && abs(det * right) == right) by (nonlinear_arith)
        requires det == 1 || det == -1, left >= 0, right >= 0;
    gcd_characterization(left, right);
    gcd_characterization(abs(first), abs(second));
    if left == 0 && right == 0 {
        assert(first == 0 && second == 0) by (nonlinear_arith)
            requires first == a * left + b * right, second == c * left + d * right,
                left == 0, right == 0;
    } else {
        assert(first != 0 || second != 0) by (nonlinear_arith)
            requires d * first - b * second == det * left,
                -c * first + a * second == det * right,
                det == 1 || det == -1, left != 0 || right != 0;
        let before = gcd(left, right);
        let after = gcd(abs(first), abs(second));
        linear_combination_divisible(left as int, right as int, a, b, before);
        linear_combination_divisible(left as int, right as int, c, d, before);
        abs_divisibility(first, before);
        abs_divisibility(second, before);
        assert(before <= after);
        abs_divisibility(first, after);
        abs_divisibility(second, after);
        linear_combination_divisible(first, second, d, -b, after);
        linear_combination_divisible(first, second, -c, a, after);
        assert(d * first + (-b) * second == det * left) by (nonlinear_arith)
            requires d * first - b * second == det * left;
        abs_divisibility(det * left, after);
        abs_divisibility(det * right, after);
        assert(left % after == 0 && right % after == 0);
        assert(after <= before);
    }
}

pub(crate) proof fn magnitude_row(left: nat, right: nat, a: int, b: int)
    ensures abs(a * left + b * right) == if (a < 0) == (b < 0) {
        abs(a) * left + abs(b) * right
    } else {
        abs(abs(a) * left - abs(b) * right)
    },
{
    assert(abs(a * left + b * right) == if (a < 0) == (b < 0) {
        abs(a) * left + abs(b) * right
    } else {
        abs(abs(a) * left - abs(b) * right)
    }) by (nonlinear_arith) requires left >= 0, right >= 0;
}

pub(crate) open spec fn alternating(a: int, b: int, c: int, d: int) -> bool {
    (a >= 0 && b <= 0 && c <= 0 && d >= 0)
        || (a <= 0 && b >= 0 && c >= 0 && d <= 0)
}

proof fn opposite_coefficient_growth(a: int, c: int, quotient: int)
    requires quotient > 0, (a >= 0 && c <= 0) || (a <= 0 && c >= 0),
    ensures abs(a - quotient * c) == abs(a) + quotient * abs(c),
{
    assert(abs(a - quotient * c) == abs(a) + quotient * abs(c)) by (nonlinear_arith)
        requires quotient > 0, (a >= 0 && c <= 0) || (a <= 0 && c >= 0);
}

proof fn row_growth(a: int, b: int, c: int, d: int, quotient: int, steps: nat)
    requires quotient > 0, alternating(a, b, c, d),
        abs(a) + abs(b) >= pow2(steps / 2),
        abs(c) + abs(d) >= pow2((steps + 1) / 2),
    ensures
        alternating(c, d, a - quotient * c, b - quotient * d),
        abs(c) + abs(d) >= pow2((steps + 1) / 2),
        abs(a - quotient * c) + abs(b - quotient * d) >= pow2((steps + 2) / 2),
{
    opposite_coefficient_growth(a, c, quotient);
    opposite_coefficient_growth(b, d, quotient);
    assert(alternating(c, d, a - quotient * c, b - quotient * d)) by (nonlinear_arith)
        requires quotient > 0, alternating(a, b, c, d);
    if steps % 2 == 0 {
        lemma_pow2_unfold((steps + 2) / 2);
    }
    let top = abs(a) + abs(b);
    let bottom = abs(c) + abs(d);
    let next = abs(a - quotient * c) + abs(b - quotient * d);
    assert(next == top + quotient * bottom) by (nonlinear_arith)
        requires abs(a - quotient * c) == abs(a) + quotient * abs(c),
            abs(b - quotient * d) == abs(b) + quotient * abs(d),
            next == abs(a - quotient * c) + abs(b - quotient * d),
            top == abs(a) + abs(b), bottom == abs(c) + abs(d);
    if steps % 2 == 0 {
        assert(next >= pow2((steps + 2) / 2)) by (nonlinear_arith)
            requires next == top + quotient * bottom, quotient >= 1,
                top >= pow2(steps / 2), bottom >= pow2(steps / 2), bottom >= 0,
                pow2((steps + 2) / 2) == 2 * pow2(steps / 2);
    } else {
        assert(next >= bottom) by (nonlinear_arith)
            requires next == top + quotient * bottom, quotient >= 1, top >= 0, bottom >= 0;
    }
}

/// The bounded coefficients double in magnitude at least every two steps.
/// Their word-size bound therefore keeps the existing u8 counter below 129.
proof fn step_counter_bound(c: int, d: int, steps: u8)
    requires abs(c) <= u64::MAX, abs(d) <= u64::MAX,
        abs(c) + abs(d) >= pow2((steps as nat + 1) / 2),
    ensures steps < 129,
{
    if steps >= 129 {
        let exponent = (steps as nat + 1) / 2;
        lemma2_to64();
        lemma_pow2_unfold(65);
        if exponent > 65 { lemma_pow2_strictly_increases(65, exponent); }
        assert(pow2(exponent) >= pow2(65));
    }
}

pub(crate) open spec fn valid_matrix(matrix: [i128; 4]) -> bool {
    (determinant(matrix[0] as int, matrix[1] as int, matrix[2] as int, matrix[3] as int) == 1
        || determinant(matrix[0] as int, matrix[1] as int, matrix[2] as int, matrix[3] as int) == -1)
        && forall|index: int| 0 <= index < 4 ==> abs(#[trigger] matrix[index] as int) <= u64::MAX
}

proof fn determinant_step(a: int, b: int, c: int, d: int, quotient: int)
    ensures determinant(c, d, a - quotient * c, b - quotient * d) == -determinant(a, b, c, d),
{
    assert(c * (b - quotient * d) - d * (a - quotient * c) == -(a * d - b * c))
        by (nonlinear_arith);
}

pub(crate) open spec fn finish(a: int, b: int, c: int, d: int, steps: nat) -> Option<(int, int, int, int)> {
    if steps >= 2 { Some((a, b, c, d)) } else { None }
}

/// A bounded recursive model of endpoint agreement using mathematical
/// integers. Coefficient growth proves that 129 guard evaluations suffice;
/// the executable loop never exhausts this model's recursion budget.
#[verifier::opaque]
pub(crate) open spec fn batch_spec(
    larger: int, smaller: int, a: int, b: int, c: int, d: int, steps: nat, fuel: nat,
) -> Option<(int, int, int, int)>
    decreases fuel,
{
    if fuel == 0 || larger < 0 || smaller <= 0
        || larger + a < 0 || larger + b < 0 || smaller + c <= 0 || smaller + d <= 0 {
        finish(a, b, c, d, steps)
    } else {
        let quotient = (larger + a) / (smaller + c);
        let next_c = a - quotient * c;
        let next_d = b - quotient * d;
        let next_smaller = larger - quotient * smaller;
        if quotient == 0 || quotient != (larger + b) / (smaller + d)
            || next_smaller < 0 || abs(c) > u64::MAX || abs(d) > u64::MAX
            || abs(next_c) > u64::MAX || abs(next_d) > u64::MAX {
            finish(a, b, c, d, steps)
        } else {
            batch_spec(smaller, next_smaller, c, d, next_c, next_d, steps + 1, (fuel - 1) as nat)
        }
    }
}

pub(crate) open spec fn matrix_view(result: Option<[i128; 4]>) -> Option<(int, int, int, int)> {
    match result {
        Some(matrix) => Some((matrix[0] as int, matrix[1] as int, matrix[2] as int, matrix[3] as int)),
        None => None,
    }
}

proof fn quotient_bound(larger: int, smaller: int, a: int, b: int, c: int, d: int)
    requires larger >= 0, smaller >= 0, alternating(a, b, c, d),
        larger + a >= 0, larger + b >= 0, smaller + c > 0, smaller + d > 0,
        (larger + a) / (smaller + c) == (larger + b) / (smaller + d),
    ensures 0 <= (larger + a) / (smaller + c) <= larger,
{
    if a <= 0 && c >= 0 {
        lemma_div_nonincreasing(larger + a, smaller + c);
    } else {
        lemma_div_nonincreasing(larger + b, smaller + d);
    }
}
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures result <==> abs(value as int) <= u64::MAX,
))]
#[inline]
fn coefficient_fits(value: i128) -> bool {
    value >= -(u64::MAX as i128) && value <= u64::MAX as i128
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures result.is_none() <==> abs(value as int) > u64::MAX,
        match result { Some(magnitude) => magnitude == abs(value as int), None => true },
))]
#[inline]
fn coefficient_magnitude(value: i128) -> Option<u64> {
    if !coefficient_fits(value) {
        return None;
    }
    Some(if value < 0 {
        (-value) as u64
    } else {
        value as u64
    })
}

/// Select exact word magnitudes and the addition/difference branch for a
/// signed matrix row, rejecting precisely the coefficients that do not fit.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures result.is_none() <==> (abs(left as int) > u64::MAX || abs(right as int) > u64::MAX),
        match result {
            Some((a, b, add)) => a == abs(left as int) && b == abs(right as int)
                && add == ((left < 0) == (right < 0)),
            None => true,
        },
))]
#[inline]
pub(crate) fn row_coefficients(left: i128, right: i128) -> Option<(u64, u64, bool)> {
    let left_magnitude = coefficient_magnitude(left)?;
    let right_magnitude = coefficient_magnitude(right)?;
    Some((left_magnitude, right_magnitude, (left < 0) == (right < 0)))
}

/// Build a word-bounded unimodular transform from the leading 62 bits. The
/// endpoint agreement guard and checked arithmetic retain the existing batch
/// selection; every accepted update strictly decreases the leading pair.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires 0 <= input_smaller <= input_larger < 0x4000_0000_0000_0000i128,
    ensures match result {
        Some(matrix) => valid_matrix(matrix)
            && 0 <= matrix[0] as int * input_larger + matrix[1] as int * input_smaller < input_larger
            && 0 <= matrix[2] as int * input_larger + matrix[3] as int * input_smaller < input_larger,
        None => true,
    },
        matrix_view(result) == batch_spec(input_larger as int, input_smaller as int, 1, 0, 0, 1, 0, 129),
))]
pub(crate) fn matrix(input_larger: i128, input_smaller: i128) -> Option<[i128; 4]> {
    if input_smaller == 0 {
        proof! { reveal(batch_spec); }
        return None;
    }
    let mut high_larger = input_larger;
    let mut high_smaller = input_smaller;
    let (mut a, mut b, mut c, mut d) = (1_i128, 0_i128, 0_i128, 1_i128);
    let mut steps = 0_u8;
    proof! { lemma2_to64(); }
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant
            0 <= input_smaller <= input_larger < 0x4000_0000_0000_0000int,
            0 <= high_larger <= input_larger,
            0 <= high_smaller <= input_larger,
            steps > 0 ==> high_smaller < input_larger,
            steps >= 2 ==> high_larger < input_larger,
            high_larger == a as int * input_larger + b as int * input_smaller,
            high_smaller == c as int * input_larger + d as int * input_smaller,
            determinant(a as int, b as int, c as int, d as int) == 1
                || determinant(a as int, b as int, c as int, d as int) == -1,
            alternating(a as int, b as int, c as int, d as int),
            abs(a as int) <= u64::MAX, abs(b as int) <= u64::MAX,
            abs(c as int) <= u64::MAX, abs(d as int) <= u64::MAX,
            abs(a as int) + abs(b as int) >= pow2(steps as nat / 2),
            abs(c as int) + abs(d as int) >= pow2((steps as nat + 1) / 2),
            steps < 129,
            batch_spec(high_larger as int, high_smaller as int, a as int, b as int, c as int, d as int,
                steps as nat, (129 - steps) as nat)
                == batch_spec(input_larger as int, input_smaller as int, 1, 0, 0, 1, 0, 129),
        ensures
            finish(a as int, b as int, c as int, d as int, steps as nat)
                == batch_spec(input_larger as int, input_smaller as int, 1, 0, 0, 1, 0, 129),
        decreases high_larger as int + high_smaller as int,
    ))]
    while let Some(numerator_low) = high_larger.checked_add(a) {
        proof! { reveal(batch_spec); }
        let Some(numerator_high) = high_larger.checked_add(b) else {
            break;
        };
        let Some(denominator_low) = high_smaller.checked_add(c) else {
            break;
        };
        let Some(denominator_high) = high_smaller.checked_add(d) else {
            break;
        };
        if numerator_low < 0 || numerator_high < 0 || denominator_low <= 0 || denominator_high <= 0
        {
            break;
        }
        let quotient = numerator_low / denominator_low;
        if quotient == 0 || quotient != numerator_high / denominator_high {
            break;
        }
        proof! {
            quotient_bound(high_larger as int, high_smaller as int, a as int, b as int, c as int, d as int);
            assert(i128::MIN <= quotient as int * c as int <= i128::MAX
                && i128::MIN <= quotient as int * d as int <= i128::MAX
                && i128::MIN <= quotient as int * high_smaller as int <= i128::MAX
                && i128::MIN <= a as int - quotient as int * c as int <= i128::MAX
                && i128::MIN <= b as int - quotient as int * d as int <= i128::MAX
                && i128::MIN <= high_larger as int - quotient as int * high_smaller as int <= i128::MAX)
                by (nonlinear_arith)
                requires 0 <= quotient <= high_larger < 0x4000_0000_0000_0000int,
                    0 <= high_smaller < 0x4000_0000_0000_0000int,
                    -(u64::MAX as int) <= a <= u64::MAX,
                    -(u64::MAX as int) <= b <= u64::MAX,
                    -(u64::MAX as int) <= c <= u64::MAX,
                    -(u64::MAX as int) <= d <= u64::MAX;
        }
        let Some(next_c) = a.checked_sub(quotient.checked_mul(c)?) else {
            break;
        };
        let Some(next_d) = b.checked_sub(quotient.checked_mul(d)?) else {
            break;
        };
        let Some(next_high_smaller) = high_larger.checked_sub(quotient.checked_mul(high_smaller)?)
        else {
            break;
        };
        if next_high_smaller < 0 {
            break;
        }
        if !coefficient_fits(c)
            || !coefficient_fits(d)
            || !coefficient_fits(next_c)
            || !coefficient_fits(next_d)
        {
            break;
        }
        proof! {
            assert(high_smaller > 0);
            assert(next_high_smaller < high_larger) by (nonlinear_arith)
                requires high_smaller > 0, quotient > 0,
                    next_high_smaller == high_larger - quotient * high_smaller;
            assert((high_smaller as int + next_high_smaller as int) < (high_larger as int + high_smaller as int))
                by (nonlinear_arith)
                requires high_smaller > 0, quotient > 0,
                    next_high_smaller == high_larger - quotient * high_smaller;
            assert(next_high_smaller == next_c as int * input_larger + next_d as int * input_smaller)
                by (nonlinear_arith)
                requires high_larger == a as int * input_larger + b as int * input_smaller,
                    high_smaller == c as int * input_larger + d as int * input_smaller,
                    next_c == a - quotient * c, next_d == b - quotient * d,
                    next_high_smaller == high_larger - quotient * high_smaller;
            determinant_step(a as int, b as int, c as int, d as int, quotient as int);
            row_growth(a as int, b as int, c as int, d as int, quotient as int, steps as nat);
            step_counter_bound(c as int, d as int, steps);
            step_counter_bound(next_c as int, next_d as int, (steps + 1) as u8);
        }
        a = c;
        b = d;
        c = next_c;
        d = next_d;
        high_larger = high_smaller;
        high_smaller = next_high_smaller;
        steps += 1;
    }
    proof! { reveal(batch_spec); }
    if steps >= 2 { Some([a, b, c, d]) } else { None }
}
