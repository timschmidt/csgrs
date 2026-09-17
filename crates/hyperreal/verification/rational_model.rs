//! Unbounded rational semantics for the implementation refinement proofs.
//! Representation equality is deliberately distinct from rational equivalence:
//! Hyperreal may retain unreduced internal fractions.
use vstd::prelude::*;
use vstd::arithmetic::mul::*;

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
