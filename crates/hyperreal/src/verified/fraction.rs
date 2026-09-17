//! Exact native fraction reduction shared with production arithmetic.

#[cfg(verus_keep_ghost)]
use super::division::gcd;
#[cfg(verus_keep_ghost)]
use vstd::prelude::*;

/// Divide both parts by their complete GCD, including `0 / denominator`.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires denominator > 0,
    ensures
        result.0 == numerator as nat / gcd(numerator as nat, denominator as nat),
        result.1 == denominator as nat / gcd(numerator as nat, denominator as nat),
        result.0 <= numerator,
        0 < result.1 <= denominator,
        gcd(result.0 as nat, result.1 as nat) == 1,
        result.0 as nat * denominator as nat == numerator as nat * result.1 as nat,
        (result.0 == 0 <==> numerator == 0),
        numerator == 0 ==> result.1 == 1,
))]
#[inline]
pub(crate) fn reduce(numerator: u128, denominator: u128) -> (u128, u128) {
    let divisor = super::gcd::gcd_u128(numerator, denominator);
    proof! { super::gcd::gcd_reduction(numerator as nat, denominator as nat); }
    (numerator / divisor, denominator / divisor)
}

#[cfg(verus_keep_ghost)]
use vstd::arithmetic::div_mod::*;

#[cfg(verus_keep_ghost)]
verus! {
pub(crate) open spec fn bezout_witness(left: nat, right: nat, x: int, y: int) -> bool {
    gcd(left, right) == x * left + y * right
}

/// Euclid's GCD has an integer linear-combination witness.
pub(crate) proof fn gcd_bezout(left: nat, right: nat)
    ensures exists|x: int, y: int| #[trigger] bezout_witness(left, right, x, y),
    decreases right,
{
    if right == 0 {
        assert(gcd(left, right) == 1 * left + 0 * right) by (nonlinear_arith)
            requires right == 0, gcd(left, right) == left;
        assert(bezout_witness(left, right, 1, 0));
    } else {
        let quotient = left / right;
        let remainder = left % right;
        gcd_bezout(right, remainder);
        let (x, y) = choose|x: int, y: int| #[trigger] bezout_witness(right, remainder, x, y);
        lemma_fundamental_div_mod(left as int, right as int);
        let coefficient = x - y * quotient;
        assert(y * left + coefficient * right == x * right + y * remainder) by (nonlinear_arith)
            requires left == right * quotient + remainder, coefficient == x - y * quotient;
        assert(bezout_witness(left, right, y, coefficient));
    }
}

proof fn linear_combination_divisible(left: int, right: int, divisor: int, x: int, y: int)
    requires divisor > 0, left % divisor == 0, right % divisor == 0,
    ensures (x * left + y * right) % divisor == 0,
{
    lemma_fundamental_div_mod(left, divisor);
    lemma_fundamental_div_mod(right, divisor);
    let coefficient = x * (left / divisor) + y * (right / divisor);
    assert(x * left + y * right == coefficient * divisor) by (nonlinear_arith)
        requires left == divisor * (left / divisor), right == divisor * (right / divisor),
            coefficient == x * (left / divisor) + y * (right / divisor);
    lemma_mod_multiples_basic(coefficient, divisor);
}

/// A divisor coprime to one factor must divide the other factor.
pub(crate) proof fn coprime_divides_product(left: nat, right: nat, divisor: nat)
    requires divisor > 0, gcd(left, divisor) == 1, (left * right) % divisor == 0,
    ensures right % divisor == 0,
{
    gcd_bezout(left, divisor);
    let (x, y) = choose|x: int, y: int| #[trigger] bezout_witness(left, divisor, x, y);
    lemma_mod_self_0(divisor as int);
    linear_combination_divisible((left * right) as int, divisor as int, divisor as int, x, y * right);
    assert(x * (left * right) + (y * right) * divisor == right) by (nonlinear_arith)
        requires x * left + y * divisor == 1;
}

/// Multiplication preserves coprimality to a common positive denominator.
pub(crate) proof fn coprime_product(left: nat, right: nat, denominator: nat)
    requires denominator > 0, gcd(left, denominator) == 1, gcd(right, denominator) == 1,
    ensures gcd(left * right, denominator) == 1,
{
    super::division::gcd_characterization(left * right, denominator);
    super::division::gcd_characterization(right, denominator);
    let divisor = gcd(left * right, denominator);
    gcd_bezout(left, denominator);
    let (x, y) = choose|x: int, y: int| #[trigger] bezout_witness(left, denominator, x, y);
    linear_combination_divisible((left * right) as int, denominator as int, divisor as int, x, y * right);
    assert(x * (left * right) + (y * right) * denominator == right) by (nonlinear_arith)
        requires x * left + y * denominator == 1;
    assert(right % divisor == 0);
    assert(divisor <= gcd(right, denominator));
}

/// Dividing either part cannot introduce a common factor into a reduced pair.
pub(crate) proof fn coprime_quotients(numerator: nat, denominator: nat, left_divisor: nat, right_divisor: nat)
    requires denominator > 0, gcd(numerator, denominator) == 1,
        left_divisor > 0, right_divisor > 0,
        numerator % left_divisor == 0, denominator % right_divisor == 0,
    ensures gcd(numerator / left_divisor, denominator / right_divisor) == 1,
{
    lemma_fundamental_div_mod(numerator as int, left_divisor as int);
    lemma_fundamental_div_mod(denominator as int, right_divisor as int);
    let reduced_numerator = numerator / left_divisor;
    let reduced_denominator = denominator / right_divisor;
    assert(reduced_denominator > 0) by (nonlinear_arith)
        requires denominator > 0, denominator == right_divisor * reduced_denominator;
    super::division::gcd_characterization(reduced_numerator, reduced_denominator);
    let divisor = gcd(reduced_numerator, reduced_denominator);
    gcd_bezout(numerator, denominator);
    let (x, y) = choose|x: int, y: int| #[trigger] bezout_witness(numerator, denominator, x, y);
    linear_combination_divisible(reduced_numerator as int, reduced_denominator as int, divisor as int,
        x * left_divisor, y * right_divisor);
    assert((x * left_divisor) * reduced_numerator + (y * right_divisor) * reduced_denominator == 1)
        by (nonlinear_arith) requires x * numerator + y * denominator == 1,
            numerator == left_divisor * reduced_numerator, denominator == right_divisor * reduced_denominator;
    assert(1nat % divisor == 0);
    if divisor > 1 {
        lemma_small_mod(1, divisor);
    }
}
}
