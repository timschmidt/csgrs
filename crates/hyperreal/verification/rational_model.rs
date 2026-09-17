//! Unbounded rational semantics for the implementation refinement proofs.
//! Representation equality is deliberately distinct from rational equivalence:
//! Hyperreal may retain unreduced internal fractions.
use vstd::prelude::*;
use vstd::arithmetic::mul::*;
use vstd::arithmetic::div_mod::{lemma_mod_multiples_basic, lemma_small_mod};
use crate::verified::division::gcd;
use crate::verified::gcd::gcd_reduction;

verus! {

proof fn interchange_products(a: int, b: int, c: int, d: int)
    ensures (a * b) * (c * d) == (a * c) * (b * d),
{
    lemma_mul_is_associative(a, b, c * d);
    lemma_mul_is_associative(b, c, d);
    lemma_mul_is_commutative(b, c);
    lemma_mul_is_associative(c, b, d);
    lemma_mul_is_associative(a, c, b * d);
}

pub struct Fraction {
    pub numerator: int,
    pub denominator: int,
}

pub open spec fn valid(x: Fraction) -> bool {
    x.denominator > 0
}

pub(crate) open spec fn magnitude(numerator: int) -> nat {
    if numerator < 0 { (-numerator) as nat } else { numerator as nat }
}

pub(crate) open spec fn canonical(x: Fraction) -> bool {
    valid(x) && gcd(magnitude(x.numerator), x.denominator as nat) == 1
}

pub(crate) open spec fn reduce(x: Fraction) -> Fraction {
    let divisor = gcd(magnitude(x.numerator), x.denominator as nat);
    let reduced_magnitude = magnitude(x.numerator) / divisor;
    Fraction {
        numerator: if x.numerator < 0 { -(reduced_magnitude as int) } else { reduced_magnitude as int },
        denominator: x.denominator / (divisor as int),
    }
}

/// The native reducer's quotient contract yields canonical signed fractions.
pub(crate) proof fn reduction_preserves_value(x: Fraction)
    requires valid(x),
    ensures
        canonical(reduce(x)),
        equivalent(x, reduce(x)),
        (reduce(x).numerator == 0 <==> x.numerator == 0),
        x.numerator == 0 ==> reduce(x).denominator == 1,
{
    let numerator = magnitude(x.numerator);
    let denominator = x.denominator as nat;
    gcd_reduction(numerator, denominator);
    let divisor = gcd(numerator, denominator);
    let reduced_numerator = numerator / divisor;
    let reduced_denominator = denominator / divisor;
    if x.numerator < 0 {
        assert(equivalent(x, reduce(x))) by (nonlinear_arith)
            requires numerator == -x.numerator, reduced_numerator * denominator == numerator * reduced_denominator,
                reduce(x).numerator == -(reduced_numerator as int), reduce(x).denominator == reduced_denominator,
                x.denominator == denominator;
    }
    assert(magnitude(reduce(x).numerator) == reduced_numerator);
}

proof fn canonical_denominator_divides_other(x: Fraction, y: Fraction)
    requires canonical(x), valid(y), equivalent(x, y),
    ensures y.denominator % x.denominator == 0,
{
    let numerator = magnitude(x.numerator);
    let coefficient = if x.numerator < 0 { -y.numerator } else { y.numerator };
    if x.numerator < 0 {
        assert(numerator * y.denominator == coefficient * x.denominator) by (nonlinear_arith)
            requires numerator == -x.numerator, coefficient == -y.numerator,
                x.numerator * y.denominator == y.numerator * x.denominator;
    } else {
        assert(numerator * y.denominator == coefficient * x.denominator);
    }
    lemma_mod_multiples_basic(coefficient, x.denominator);
    crate::verified::fraction::coprime_divides_product(numerator, y.denominator as nat, x.denominator as nat);
}

/// Equivalent reduced fractions with positive denominators have identical parts.
pub(crate) proof fn canonical_unique(x: Fraction, y: Fraction)
    requires canonical(x), canonical(y), equivalent(x, y),
    ensures x.numerator == y.numerator, x.denominator == y.denominator,
{
    canonical_denominator_divides_other(x, y);
    equivalence_symmetric(x, y);
    canonical_denominator_divides_other(y, x);
    if x.denominator < y.denominator {
        lemma_small_mod(x.denominator as nat, y.denominator as nat);
    } else if y.denominator < x.denominator {
        lemma_small_mod(y.denominator as nat, x.denominator as nat);
    }
    assert(x.numerator == y.numerator) by (nonlinear_arith)
        requires x.denominator > 0, x.denominator == y.denominator,
            x.numerator * y.denominator == y.numerator * x.denominator;
}

pub(crate) proof fn reduction_is_idempotent(x: Fraction)
    requires valid(x),
    ensures reduce(reduce(x)).numerator == reduce(x).numerator,
        reduce(reduce(x)).denominator == reduce(x).denominator,
{
    reduction_preserves_value(x);
    reduction_preserves_value(reduce(x));
    canonical_unique(reduce(x), reduce(reduce(x)));
}

/// Canonical parts characterize value equality even for unreduced inputs.
pub(crate) proof fn canonical_form_characterizes_equivalence(x: Fraction, y: Fraction)
    requires valid(x), valid(y),
    ensures equivalent(x, y) <==> (reduce(x).numerator == reduce(y).numerator
        && reduce(x).denominator == reduce(y).denominator),
{
    reduction_preserves_value(x);
    reduction_preserves_value(y);
    let left = reduce(x);
    let right = reduce(y);
    if equivalent(x, y) {
        equivalence_symmetric(x, left);
        equivalence_transitive(left, x, y);
        equivalence_transitive(left, y, right);
        canonical_unique(left, right);
    }
    if left.numerator == right.numerator && left.denominator == right.denominator {
        assert(equivalent(left, right));
        equivalence_transitive(x, left, right);
        equivalence_symmetric(y, right);
        equivalence_transitive(x, right, y);
    }
}

pub open spec fn equivalent(x: Fraction, y: Fraction) -> bool {
    x.numerator * y.denominator == y.numerator * x.denominator
}

pub open spec fn less(x: Fraction, y: Fraction) -> bool {
    x.numerator * y.denominator < y.numerator * x.denominator
}

pub open spec fn integer(n: int) -> Fraction {
    Fraction { numerator: n, denominator: 1 }
}

pub open spec fn negate(x: Fraction) -> Fraction {
    Fraction { numerator: -x.numerator, denominator: x.denominator }
}

pub open spec fn add(x: Fraction, y: Fraction) -> Fraction {
    Fraction {
        numerator: x.numerator * y.denominator + y.numerator * x.denominator,
        denominator: x.denominator * y.denominator,
    }
}

pub open spec fn multiply(x: Fraction, y: Fraction) -> Fraction {
    Fraction {
        numerator: x.numerator * y.numerator,
        denominator: x.denominator * y.denominator,
    }
}

pub open spec fn reciprocal(x: Fraction) -> Fraction {
    if x.numerator < 0 {
        Fraction { numerator: -x.denominator, denominator: -x.numerator }
    } else {
        Fraction { numerator: x.denominator, denominator: x.numerator }
    }
}

pub proof fn equivalence_reflexive(x: Fraction)
    ensures equivalent(x, x),
{}

pub proof fn equivalence_symmetric(x: Fraction, y: Fraction)
    requires equivalent(x, y),
    ensures equivalent(y, x),
{}

pub proof fn equivalence_transitive(x: Fraction, y: Fraction, z: Fraction)
    requires valid(x), valid(y), valid(z), equivalent(x, y), equivalent(y, z),
    ensures equivalent(x, z),
{
    assert(x.numerator * z.denominator == z.numerator * x.denominator) by (nonlinear_arith)
        requires
            y.denominator > 0,
            x.numerator * y.denominator == y.numerator * x.denominator,
            y.numerator * z.denominator == z.numerator * y.denominator;
}

pub proof fn order_transitive(x: Fraction, y: Fraction, z: Fraction)
    requires valid(x), valid(y), valid(z), less(x, y), less(y, z),
    ensures less(x, z),
{
    assert(x.numerator * z.denominator < z.numerator * x.denominator) by (nonlinear_arith)
        requires
            x.denominator > 0, y.denominator > 0, z.denominator > 0,
            x.numerator * y.denominator < y.numerator * x.denominator,
            y.numerator * z.denominator < z.numerator * y.denominator;
}

pub proof fn order_trichotomy(x: Fraction, y: Fraction)
    ensures
        less(x, y) || equivalent(x, y) || less(y, x),
        !(less(x, y) && equivalent(x, y)),
        !(less(x, y) && less(y, x)),
        !(equivalent(x, y) && less(y, x)),
{}

pub proof fn rescaling_preserves_value(x: Fraction, scale: int)
    requires valid(x), scale > 0,
    ensures
        valid(Fraction { numerator: x.numerator * scale, denominator: x.denominator * scale }),
        equivalent(x, Fraction { numerator: x.numerator * scale, denominator: x.denominator * scale }),
{
    assert(x.denominator * scale > 0) by (nonlinear_arith)
        requires x.denominator > 0, scale > 0;
    assert(x.numerator * (x.denominator * scale) == (x.numerator * scale) * x.denominator)
        by (nonlinear_arith);
}

pub proof fn zero_iff_zero_numerator(x: Fraction)
    ensures equivalent(x, integer(0)) <==> x.numerator == 0,
{}

pub proof fn arithmetic_preserves_validity(x: Fraction, y: Fraction)
    requires valid(x), valid(y),
    ensures valid(negate(x)), valid(add(x, y)), valid(multiply(x, y)),
{
    assert(x.denominator * y.denominator > 0) by (nonlinear_arith)
        requires x.denominator > 0, y.denominator > 0;
}

pub proof fn addition_commutes(x: Fraction, y: Fraction)
    ensures equivalent(add(x, y), add(y, x)),
{
    assert((x.numerator * y.denominator + y.numerator * x.denominator) * (y.denominator * x.denominator)
        == (y.numerator * x.denominator + x.numerator * y.denominator) * (x.denominator * y.denominator))
        by (nonlinear_arith);
}

pub proof fn addition_associates(x: Fraction, y: Fraction, z: Fraction)
    ensures equivalent(add(add(x, y), z), add(x, add(y, z))),
{
    assert(add(add(x, y), z).denominator == add(x, add(y, z)).denominator) by (nonlinear_arith);
    assert(add(add(x, y), z).numerator == add(x, add(y, z)).numerator) by (nonlinear_arith);
}

pub proof fn multiplication_commutes(x: Fraction, y: Fraction)
    ensures equivalent(multiply(x, y), multiply(y, x)),
{
    assert(equivalent(multiply(x, y), multiply(y, x))) by (nonlinear_arith);
}

pub proof fn multiplication_associates(x: Fraction, y: Fraction, z: Fraction)
    ensures equivalent(multiply(multiply(x, y), z), multiply(x, multiply(y, z))),
{
    assert(equivalent(multiply(multiply(x, y), z), multiply(x, multiply(y, z)))) by (nonlinear_arith);
}

pub proof fn multiplication_distributes(x: Fraction, y: Fraction, z: Fraction)
    ensures equivalent(multiply(x, add(y, z)), add(multiply(x, y), multiply(x, z))),
{
    let left = multiply(x, add(y, z));
    let right = add(multiply(x, y), multiply(x, z));
    interchange_products(x.numerator, y.numerator, x.denominator, z.denominator);
    interchange_products(x.numerator, z.numerator, x.denominator, y.denominator);
    lemma_mul_is_distributive_add(x.numerator * x.denominator,
        y.numerator * z.denominator, z.numerator * y.denominator);
    lemma_mul_is_associative(x.numerator, x.denominator, add(y, z).numerator);
    lemma_mul_is_associative(x.numerator, add(y, z).numerator, x.denominator);
    lemma_mul_is_commutative(x.denominator, add(y, z).numerator);
    assert(right.numerator == left.numerator * x.denominator);
    interchange_products(x.denominator, y.denominator, x.denominator, z.denominator);
    lemma_mul_is_associative(x.denominator, x.denominator, y.denominator * z.denominator);
    lemma_mul_is_associative(x.denominator, y.denominator * z.denominator, x.denominator);
    lemma_mul_is_commutative(x.denominator, y.denominator * z.denominator);
    assert(right.denominator == left.denominator * x.denominator);
    lemma_mul_is_associative(left.numerator, left.denominator, x.denominator);
    lemma_mul_is_associative(left.numerator, x.denominator, left.denominator);
    lemma_mul_is_commutative(left.denominator, x.denominator);
}

pub proof fn identities(x: Fraction)
    ensures
        equivalent(add(x, integer(0)), x),
        equivalent(multiply(x, integer(1)), x),
        equivalent(add(x, negate(x)), integer(0)),
        equivalent(negate(negate(x)), x),
{
    assert(equivalent(add(x, negate(x)), integer(0))) by (nonlinear_arith);
}

pub proof fn reciprocal_is_inverse(x: Fraction)
    requires valid(x), x.numerator != 0,
    ensures valid(reciprocal(x)), equivalent(multiply(x, reciprocal(x)), integer(1)),
{
    if x.numerator < 0 {
        assert(x.numerator * -x.denominator == x.denominator * -x.numerator) by (nonlinear_arith);
    } else {
        assert(x.numerator * x.denominator == x.denominator * x.numerator) by (nonlinear_arith);
    }
}

pub proof fn addition_respects_equivalence(x: Fraction, y: Fraction, z: Fraction)
    requires equivalent(x, y),
    ensures equivalent(add(x, z), add(y, z)),
{
    lemma_mul_is_distributive_add_other_way(y.denominator * z.denominator,
        x.numerator * z.denominator, z.numerator * x.denominator);
    lemma_mul_is_distributive_add_other_way(x.denominator * z.denominator,
        y.numerator * z.denominator, z.numerator * y.denominator);
    interchange_products(x.numerator, z.denominator, y.denominator, z.denominator);
    interchange_products(y.numerator, z.denominator, x.denominator, z.denominator);
    interchange_products(z.numerator, y.denominator, x.denominator, z.denominator);
}

pub proof fn multiplication_respects_equivalence(x: Fraction, y: Fraction, z: Fraction)
    requires equivalent(x, y),
    ensures equivalent(multiply(x, z), multiply(y, z)),
{
    assert(equivalent(multiply(x, z), multiply(y, z))) by (nonlinear_arith)
        requires x.numerator * y.denominator == y.numerator * x.denominator;
}

} // verus!
