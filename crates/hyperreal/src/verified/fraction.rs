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

/// Prefix products retain the ordering relevant to checked native multiplication.
pub(crate) open spec fn factors_product(factors: Seq<u128>, length: int) -> nat
    recommends 0 <= length <= factors.len(),
    decreases length,
{
    if length <= 0 { 1 } else {
        factors_product(factors, length - 1) * factors[length - 1] as nat
    }
}

pub(crate) proof fn factors_positive(factors: Seq<u128>, length: int)
    requires 0 <= length <= factors.len(),
        forall|i: int| 0 <= i < length ==> #[trigger] factors[i] > 0,
    ensures factors_product(factors, length) > 0,
    decreases length,
{
    if length > 0 {
        factors_positive(factors, length - 1);
        assert(factors_product(factors, length) > 0) by (nonlinear_arith)
            requires factors_product(factors, length - 1) > 0, factors[length - 1] > 0,
                factors_product(factors, length)
                    == factors_product(factors, length - 1) * factors[length - 1] as nat;
    }
}

pub(crate) proof fn factors_unit(factors: Seq<u128>, length: int)
    requires 0 <= length <= factors.len(),
        forall|i: int| 0 <= i < length ==> #[trigger] factors[i] == 1,
    ensures factors_product(factors, length) == 1,
    decreases length,
{
    if length > 0 { factors_unit(factors, length - 1); }
}

/// An exact factor division divides every containing prefix by the same amount.
/// This formulation also covers a zero factor, without cancelling that zero.
pub(crate) proof fn factors_update_quotient(
    factors: Seq<u128>, index: int, quotient: u128, divisor: nat, length: int,
)
    requires 0 <= index < factors.len(), 0 <= length <= factors.len(),
        factors[index] as nat == quotient as nat * divisor,
    ensures
        index < length ==> factors_product(factors.update(index, quotient), length) * divisor
            == factors_product(factors, length),
        index >= length ==> factors_product(factors.update(index, quotient), length)
            == factors_product(factors, length),
    decreases length,
{
    if length > 0 {
        factors_update_quotient(factors, index, quotient, divisor, length - 1);
        let updated = factors.update(index, quotient);
        if index < length {
            assert(factors_product(updated, length) * divisor == factors_product(factors, length))
                by (nonlinear_arith) requires
                    index < length,
                    factors_product(updated, length)
                        == factors_product(updated, length - 1) * updated[length - 1] as nat,
                    factors_product(factors, length)
                        == factors_product(factors, length - 1) * factors[length - 1] as nat,
                    index == length - 1 ==> updated[length - 1] == quotient
                        && factors_product(updated, length - 1) == factors_product(factors, length - 1)
                        && factors[length - 1] as nat == quotient as nat * divisor,
                    index < length - 1 ==> updated[length - 1] == factors[length - 1]
                        && factors_product(updated, length - 1) * divisor
                            == factors_product(factors, length - 1);
        }
    }
}

/// Cross-cancelling any numerator/denominator pair preserves the whole fraction.
pub(crate) proof fn cancel_preserves_product_fraction(
    numerators: Seq<u128>, denominators: Seq<u128>, ni: int, di: int,
)
    requires 0 <= ni < numerators.len(), 0 <= di < denominators.len(), denominators[di] > 0,
    ensures ({
        let divisor = gcd(numerators[ni] as nat, denominators[di] as nat);
        let ns = numerators.update(ni, (numerators[ni] as nat / divisor) as u128);
        let ds = denominators.update(di, (denominators[di] as nat / divisor) as u128);
        factors_product(ns, ns.len() as int) * factors_product(denominators, denominators.len() as int)
            == factors_product(numerators, numerators.len() as int) * factors_product(ds, ds.len() as int)
    }),
{
    let divisor = gcd(numerators[ni] as nat, denominators[di] as nat);
    super::gcd::gcd_reduction(numerators[ni] as nat, denominators[di] as nat);
    let n = (numerators[ni] as nat / divisor) as u128;
    let d = (denominators[di] as nat / divisor) as u128;
    lemma_fundamental_div_mod(numerators[ni] as int, divisor as int);
    lemma_fundamental_div_mod(denominators[di] as int, divisor as int);
    factors_update_quotient(numerators, ni, n, divisor, numerators.len() as int);
    factors_update_quotient(denominators, di, d, divisor, denominators.len() as int);
    let ns = numerators.update(ni, n);
    let ds = denominators.update(di, d);
    assert(factors_product(ns, ns.len() as int) * factors_product(denominators, denominators.len() as int)
        == factors_product(numerators, numerators.len() as int) * factors_product(ds, ds.len() as int))
        by (nonlinear_arith) requires
            factors_product(ns, ns.len() as int) * divisor == factors_product(numerators, numerators.len() as int),
            factors_product(ds, ds.len() as int) * divisor == factors_product(denominators, denominators.len() as int);
}

pub(crate) proof fn coprime_factors(factors: Seq<u128>, denominator: nat, length: int)
    requires denominator > 0, 0 <= length <= factors.len(),
        forall|i: int| 0 <= i < length ==> gcd(#[trigger] factors[i] as nat, denominator) == 1,
    ensures gcd(factors_product(factors, length), denominator) == 1,
    decreases length,
{
    if length == 0 {
        super::gcd::gcd_symmetric(1, denominator);
        assert(gcd(denominator, 1) == gcd(1, 0));
    } else {
        coprime_factors(factors, denominator, length - 1);
        coprime_product(factors_product(factors, length - 1), factors[length - 1] as nat, denominator);
    }
}

/// Pairwise cross-coprimality gives a reduced product, including zero numerators.
pub(crate) proof fn mutually_coprime_products(numerators: Seq<u128>, denominators: Seq<u128>)
    requires
        forall|j: int| 0 <= j < denominators.len() ==> #[trigger] denominators[j] > 0,
        forall|i: int, j: int| 0 <= i < numerators.len() && 0 <= j < denominators.len()
            ==> gcd(#[trigger] numerators[i] as nat, #[trigger] denominators[j] as nat) == 1,
    ensures gcd(factors_product(numerators, numerators.len() as int),
        factors_product(denominators, denominators.len() as int)) == 1,
{
    factors_positive(denominators, denominators.len() as int);
    let denominator = factors_product(denominators, denominators.len() as int);
    assert forall|i: int| 0 <= i < numerators.len() implies
        gcd(#[trigger] numerators[i] as nat, denominator) == 1 by {
        let numerator = numerators[i] as nat;
        if numerator == 0 {
            assert forall|j: int| 0 <= j < denominators.len() implies #[trigger] denominators[j] == 1 by {
                assert(gcd(numerator, denominators[j] as nat) == 1);
                lemma_small_mod(0, denominators[j] as nat);
                reveal_with_fuel(gcd, 2);
                assert(gcd(0, denominators[j] as nat) == denominators[j]);
            }
            factors_unit(denominators, denominators.len() as int);
            assert(gcd(0, 1) == 1) by { reveal_with_fuel(gcd, 2); }
        } else {
            assert forall|j: int| 0 <= j < denominators.len() implies
                gcd(#[trigger] denominators[j] as nat, numerator) == 1 by {
                super::gcd::gcd_symmetric(numerator, denominators[j] as nat);
            }
            coprime_factors(denominators, numerator, denominators.len() as int);
            super::gcd::gcd_symmetric(denominator, numerator);
        }
    }
    coprime_factors(numerators, denominator, numerators.len() as int);
}
}
