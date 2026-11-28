//! Exact unsigned fraction reduction and native cross-cancelled products.
//!
//! The rational adapters use these shared executable kernels after converting
//! magnitudes and positive denominators to u128; signs belong to those callers.
//! Reduction returns canonical coprime parts. Products cancel opposing factors
//! before checked multiplication, retaining the original prefix-overflow fallback
//! even when a later zero makes the mathematical result small.
//!
//! Ghost definitions describe Bezout witnesses, coprimality, prefix products and
//! the precise cancellation traversal. Ordinary builds erase those declarations
//! and proof annotations, not the executable arithmetic. These private kernels
//! trust denominator preconditions rather than returning typed validation errors.
//! Native overflow requests a wider exact path; it is neither approximation nor
//! resource exhaustion. No configurable work, stack or cancellation budget is
//! implemented here.

#[cfg(verus_keep_ghost)]
use super::division::gcd;
#[cfg(verus_keep_ghost)]
use vstd::prelude::*;

/// Reduce an unsigned native fraction by its complete positive GCD.
///
/// Requires denominator > 0; numerator may be zero or any u128 value. Returns
/// coprime numerator/denominator parts with positive denominator, the same exact
/// ratio and neither part increased. Zero reduces to (0,1). Calls the shared native
/// GCD once, then divides both inputs; no multiplication or heap allocation is
/// needed and the returned pair can be reduced again unchanged.
///
/// This helper does not validate its precondition. With denominator zero, two
/// zeros divide by zero while a positive numerator can return (1,0). Neither is
/// an admitted fraction, typed failure or supported extension of the domain.
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
/// Describe an integer linear combination equal to the natural GCD.
///
/// The relation is `gcd(left,right) = x*left + y*right`, with natural operands
/// and signed integer coefficients. Zero operands are allowed, including two
/// zeros. This ghost predicate records a witness relation; it does not compute
/// runtime coefficients or imply their native-width representability.
pub(crate) open spec fn bezout_witness(left: nat, right: nat, x: int, y: int) -> bool {
    gcd(left, right) == x * left + y * right
}

/// Establish existence of signed Bezout coefficients for any natural pair.
///
/// Uses Euclidean recursion on right, with the zero-right witness (1,0), and
/// transports the recursive coefficients through quotient and remainder. Both
/// operands may be zero. The result is an existential ghost witness, not a
/// runtime extended-GCD implementation or a bound on coefficient size.
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

/// Preserve divisibility through a signed integer linear combination.
///
/// Requires a positive divisor dividing both integer operands exactly. Arbitrary
/// signed coefficients x and y then give a divisible `x*left + y*right`, including
/// negative operands and coefficients. The proof rewrites the operands as exact
/// multiples; positivity is needed for the modular lemmas. Used to transport
/// Bezout identities into coprimality arguments.
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

/// Cancel a coprime factor from exact product divisibility.
///
/// Requires a positive divisor, gcd(left,divisor)=1, and divisibility of the
/// natural product `left*right`. Concludes that divisor divides right. Zero
/// factors are allowed when these hypotheses hold; no prime-divisor assumption
/// is needed. A Bezout witness supplies the integer linear combination proving
/// the cancellation, without a runtime factorization.
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

/// Keep a product coprime to a shared positive denominator.
///
/// If each natural factor is coprime to denominator > 0, their mathematical
/// product is also coprime to it. Zero factors are possible only with denominator
/// one under these hypotheses. The argument uses a common divisor and a Bezout
/// identity; it does not require or establish a native-width product bound.
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

/// Preserve reducedness when dividing each fraction part exactly.
///
/// Requires a positive denominator, an initially coprime natural pair, and
/// positive divisors that exactly divide their respective parts. The quotients
/// remain coprime and the denominator quotient is positive. Numerator zero is
/// allowed; no division-by-zero or inexact-division case is admitted. Cancellation
/// uses this fact to retain coprimality of previously processed cross pairs.
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

/// Define the mathematical product of a sequence prefix in stored order.
///
/// The recommended domain is `0 <= length <= factors.len()`. The empty prefix
/// is one, and each recursive step appends the next u128 factor in natural
/// arithmetic, without native overflow. Sequence order matters when all prefix
/// bounds are checked by factors_fit, although the full mathematical product is
/// commutative. This definition alone does not establish any capacity bound.
pub(crate) open spec fn factors_product(factors: Seq<u128>, length: int) -> nat
    recommends 0 <= length <= factors.len(),
    decreases length,
{
    if length <= 0 { 1 } else {
        factors_product(factors, length - 1) * factors[length - 1] as nat
    }
}

/// Show that a valid prefix of positive factors has positive product.
///
/// Requires every factor before length to be positive and length within the
/// sequence bounds. Induction includes the empty product one. Factors beyond
/// the prefix are unconstrained. The result supports positive denominator
/// products and exact cross-multiplication; it does not bound their magnitude.
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

/// Show that a valid prefix containing only ones has product one.
///
/// Requires an in-range length and unit factors throughout that prefix, with no
/// restriction on later entries. The empty prefix is included. Used when a zero
/// numerator is coprime to every positive denominator factor, forcing all those
/// factors to one and hence the canonical zero denominator to one.
pub(crate) proof fn factors_unit(factors: Seq<u128>, length: int)
    requires 0 <= length <= factors.len(),
        forall|i: int| 0 <= i < length ==> #[trigger] factors[i] == 1,
    ensures factors_product(factors, length) == 1,
    decreases length,
{
    if length > 0 { factors_unit(factors, length - 1); }
}

/// Transport an exact factor relation through every containing prefix.
///
/// Requires a valid update index, an in-range prefix length and the relation
/// `old_factor = quotient*divisor` in natural arithmetic. A prefix containing
/// the index satisfies `new_product*divisor = old_product`; an earlier prefix
/// is unchanged. Neither divisor positivity nor nonzero factors are assumed.
/// In particular divisor zero is permitted when the old factor is zero: the
/// conclusion is a multiplication identity, not cancellation or division by zero.
/// The relation is used after a separately justified positive GCD division.
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

/// Preserve the whole product's cross-multiplied relation after one cancellation.
///
/// Requires valid numerator/denominator indices and a positive selected
/// denominator. Divide the selected pair by its positive GCD and leave other
/// entries alone. The old and new full products obey exact cross-multiplication,
/// including a selected zero numerator. Other denominator entries are not
/// required positive by this lemma, so interpreting both products as fractions
/// requires that additional caller fact. No native product bound is assumed.
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

/// Lift prefix-wise coprimality to coprimality of the prefix product.
///
/// Requires denominator > 0, an in-range length and every selected factor
/// coprime to that denominator. Induction uses coprime_product; the empty prefix
/// one is coprime as well. Zero factors are allowed if the hypotheses hold.
/// The statement concerns exact natural products without a native-width limit.
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

/// Turn pairwise cross-coprimality into a reduced product fraction.
///
/// All denominator factors must be positive and every numerator factor must be
/// coprime to each denominator factor. The two full mathematical products are
/// then coprime, including empty sequences. A zero numerator forces every
/// denominator factor to one and the denominator product to one. The sequences
/// need not have equal lengths here, and no runtime overflow admission follows.
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

/// Advance the numerator-major cancellation invariant by one selected pair.
///
/// Requires valid indices, all denominators positive and coprimality of every
/// pair in earlier numerator rows or earlier columns of the current row. Dividing
/// the selected pair by its GCD preserves denominator positivity and all prior
/// coprimality, adding the current pair. coprime_quotients handles previously
/// processed pairs sharing a changed factor. This lemma neither changes later
/// pairs' obligations nor establishes native product bounds.
pub(crate) proof fn cancel_preserves_processed_coprimality(
    numerators: Seq<u128>, denominators: Seq<u128>, ni: int, di: int,
)
    requires
        0 <= ni < numerators.len(), 0 <= di < denominators.len(),
        forall|j: int| 0 <= j < denominators.len() ==> #[trigger] denominators[j] > 0,
        forall|i: int, j: int| 0 <= i < numerators.len() && 0 <= j < denominators.len()
            && (i < ni || i == ni && j < di)
            ==> gcd(#[trigger] numerators[i] as nat, #[trigger] denominators[j] as nat) == 1,
    ensures ({
        let divisor = gcd(numerators[ni] as nat, denominators[di] as nat);
        let ns = numerators.update(ni, (numerators[ni] as nat / divisor) as u128);
        let ds = denominators.update(di, (denominators[di] as nat / divisor) as u128);
        &&& forall|j: int| 0 <= j < ds.len() ==> #[trigger] ds[j] > 0
        &&& forall|i: int, j: int| 0 <= i < ns.len() && 0 <= j < ds.len()
            && (i < ni || i == ni && j <= di)
            ==> gcd(#[trigger] ns[i] as nat, #[trigger] ds[j] as nat) == 1
    }),
{
    let numerator = numerators[ni] as nat;
    let denominator = denominators[di] as nat;
    let divisor = gcd(numerator, denominator);
    super::gcd::gcd_reduction(numerator, denominator);
    super::division::gcd_characterization(numerator, denominator);
    let n = (numerator / divisor) as u128;
    let d = (denominator / divisor) as u128;
    let ns = numerators.update(ni, n);
    let ds = denominators.update(di, d);
    assert forall|j: int| 0 <= j < ds.len() implies #[trigger] ds[j] > 0 by {
        if j != di { assert(ds[j] == denominators[j]); }
    }
    assert forall|i: int, j: int| 0 <= i < ns.len() && 0 <= j < ds.len()
        && (i < ni || i == ni && j <= di)
        implies gcd(#[trigger] ns[i] as nat, #[trigger] ds[j] as nat) == 1 by {
        if i == ni && j == di {
            assert(gcd(n as nat, d as nat) == 1);
        } else {
            assert(gcd(numerators[i] as nat, denominators[j] as nat) == 1);
            if i == ni {
                coprime_quotients(numerator, denominators[j] as nat, divisor, 1);
            } else if j == di {
                coprime_quotients(numerators[i] as nat, denominator, 1, divisor);
            }
        }
    }
}
}

#[cfg(verus_keep_ghost)]
verus! {
/// Require every stored-order prefix product to fit u128.
///
/// Includes the empty prefix one and the full product. This is exactly the
/// admission condition for checked_factors_product, not just a bound on the final
/// mathematical product. For example, factors (u128::MAX,2,0) fail because the
/// second prefix overflows, whereas (u128::MAX,0,2) fit. The predicate describes
/// native capacity, not a total-work or memory budget.
pub(crate) open spec fn factors_fit(factors: Seq<u128>) -> bool {
    forall|length: int| 0 <= length <= factors.len()
        ==> #[trigger] factors_product(factors, length) <= u128::MAX
}

/// Model the exact numerator-major traversal and its unit shortcuts.
///
/// Recommended indices range from zero through each sequence length. At the end
/// of a numerator row, or when its current factor is one, continue with the next
/// row at column zero. Skip a unit denominator; otherwise divide the pair by
/// its GCD and continue with the next column. Returns both resulting sequences
/// without multiplying them. Different sequence lengths are allowed here.
///
/// Correspondence with the runtime fraction kernel additionally requires positive
/// denominators; the index recommendations alone do not establish that domain.
/// This ghost recursion is neither runtime input validation nor permission to
/// reorder cancellation or bypass checked multiplication when a zero is present.
pub(crate) open spec fn cancelled_factors(
    numerators: Seq<u128>, denominators: Seq<u128>, ni: int, di: int,
) -> (Seq<u128>, Seq<u128>)
    recommends 0 <= ni <= numerators.len(), 0 <= di <= denominators.len(),
    decreases numerators.len() - ni, denominators.len() - di,
{
    if ni >= numerators.len() {
        (numerators, denominators)
    } else if numerators[ni] == 1 || di >= denominators.len() {
        cancelled_factors(numerators, denominators, ni + 1, 0)
    } else if denominators[di] == 1 {
        cancelled_factors(numerators, denominators, ni, di + 1)
    } else {
        let divisor = gcd(numerators[ni] as nat, denominators[di] as nat);
        let ns = numerators.update(ni, (numerators[ni] as nat / divisor) as u128);
        let ds = denominators.update(di, (denominators[di] as nat / divisor) as u128);
        cancelled_factors(ns, ds, ni, di + 1)
    }
}

/// Discharge the skipped-unit numerator's coprimality obligation.
///
/// For every positive natural denominator, gcd(1,denominator)=1. Uses symmetry
/// and the Euclidean remainder of division by one. The cancellation loop invokes
/// it when a numerator becomes one before visiting all denominator columns.
proof fn unit_is_coprime(denominator: nat)
    requires denominator > 0,
    ensures gcd(1, denominator) == 1,
{
    super::gcd::gcd_symmetric(1, denominator);
    assert(gcd(denominator, 1) == gcd(1, 0));
}

/// Compose two cross-multiplication identities through a positive middle denominator.
///
/// Requires d1 > 0, `n1*d0 = n0*d1` and `n2*d1 = n1*d2`, and concludes
/// `n2*d0 = n0*d2`. All six values are natural; only the middle denominator is
/// required positive in this lemma. Interpreting the other pairs as fractions
/// needs their separate positivity facts. The loop uses the identity to connect
/// each cancellation back to its original input products.
proof fn product_fraction_transitive(
    n0: nat, d0: nat, n1: nat, d1: nat, n2: nat, d2: nat,
)
    requires d1 > 0, n1 * d0 == n0 * d1, n2 * d1 == n1 * d2,
    ensures n2 * d0 == n0 * d2,
{
    assert(n2 * d0 == n0 * d2) by (nonlinear_arith)
        requires d1 > 0, n1 * d0 == n0 * d1, n2 * d1 == n1 * d2;
}
}

/// Multiply a fixed array in stored order while checking every u128 prefix.
///
/// Accepts every factor value and every array length, including an empty array
/// whose product is Some(1). Returns the exact product only if each prefix fits;
/// the first overflowing multiplication returns None immediately. A later zero
/// does not rescue that failure: (u128::MAX,2,0) fails but (u128::MAX,0,2) gives
/// Some(0). The input is borrowed unchanged, with no reordering or allocation.
///
/// At most N native multiplications are attempted. None denotes this schedule's
/// capacity failure rather than a claim that the final mathematical value is
/// overwide, an approximation or a configured resource-budget exhaustion.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures result.is_none() <==> !factors_fit(factors@),
        match result {
            Some(value) => value == factors_product(factors@, N as int),
            None => true,
        },
))]
#[inline]
#[allow(
    clippy::question_mark,
    reason = "The overflow branch proves the exact prefix-overflow fallback condition."
)]
fn checked_factors_product<const N: usize>(factors: &[u128; N]) -> Option<u128> {
    let mut value = 1_u128;
    let mut index = 0;
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant index <= N, value == factors_product(factors@, index as int),
            forall|length: int| 0 <= length <= index
                ==> #[trigger] factors_product(factors@, length) <= u128::MAX,
        decreases N - index,
    ))]
    while index < N {
        let Some(next) = value.checked_mul(factors[index]) else {
            proof! {
                assert(factors_product(factors@, index as int + 1) > u128::MAX);
                assert(!factors_fit(factors@));
            }
            return None;
        };
        value = next;
        index += 1;
    }
    Some(value)
}

/// Cross-cancel unsigned factors, then multiply each array in its original order.
///
/// Requires every denominator positive; numerator factors may be zero. Visits
/// numerators first, then denominators, skipping unit denominators and stopping
/// a row when its numerator becomes one. Every other pair is reduced by its full
/// GCD. Input arrays are owned local copies; signs and conversion of arbitrary
/// precision inputs belong to the rational adapter.
///
/// After cancellation, checks numerator prefixes before denominator prefixes.
/// Some((n,d)) is the exact original product with coprime parts and d > 0; a
/// successful zero is (0,1), and empty arrays give (1,1). None means at least one
/// reduced prefix does not fit u128. It need not mean the final value is too large:
/// (u128::MAX,2,0) over unit denominators fails before reaching zero, while moving
/// the zero to the second position succeeds. The complete cancellation pass
/// precedes these checks; no early-zero shortcut changes that schedule.
///
/// Denominator positivity is a caller obligation, not a runtime check. Invalid
/// zeros may bypass cancellation or yield zero denominators, and a zero/zero pair
/// can panic. No typed domain error or partial product is returned. There are at
/// most N squared pair visits and two N-step product scans, with native GCD work
/// per nonunit pair. The owned arrays occupy space proportional to N, without a
/// practical stack cap, heap allocation or configurable work/cancellation budget.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires forall|j: int| 0 <= j < N ==> #[trigger] denominators[j] > 0,
    ensures ({
        let reduced = cancelled_factors(numerators@, denominators@, 0, 0);
        &&& (result.is_none() <==> !factors_fit(reduced.0) || !factors_fit(reduced.1))
        &&& match result {
            Some((n, d)) => d > 0 && gcd(n as nat, d as nat) == 1
                && n == factors_product(reduced.0, N as int)
                && d == factors_product(reduced.1, N as int)
                && n as nat * factors_product(denominators@, N as int)
                    == factors_product(numerators@, N as int) * d as nat
                && (n == 0 <==> factors_product(numerators@, N as int) == 0)
                && (n == 0 ==> d == 1),
            None => true,
        }
    }),
))]
#[inline]
pub(crate) fn cross_cancelled_product<const N: usize>(
    mut numerators: [u128; N],
    mut denominators: [u128; N],
) -> Option<(u128, u128)> {
    proof_decl! { let ghost original_ns = numerators@; }
    proof_decl! { let ghost original_ds = denominators@; }
    proof_decl! { let ghost original_n = factors_product(original_ns, N as int); }
    proof_decl! { let ghost original_d = factors_product(original_ds, N as int); }
    proof_decl! { let ghost expected = cancelled_factors(original_ns, original_ds, 0, 0); }
    let mut ni = 0;
    proof! { factors_positive(original_ds, N as int); }
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant ni <= N, original_ns.len() == N, original_ds.len() == N,
            original_n == factors_product(original_ns, N as int),
            original_d == factors_product(original_ds, N as int), original_d > 0,
            expected == cancelled_factors(original_ns, original_ds, 0, 0),
            expected == cancelled_factors(numerators@, denominators@, ni as int, 0),
            forall|j: int| 0 <= j < N ==> #[trigger] denominators[j] > 0,
            forall|i: int, j: int| 0 <= i < ni && 0 <= j < N
                ==> gcd(#[trigger] numerators[i] as nat, #[trigger] denominators[j] as nat) == 1,
            factors_product(numerators@, N as int) * original_d
                == original_n * factors_product(denominators@, N as int),
        decreases N - ni,
    ))]
    while ni < N {
        let mut di = 0;
        #[cfg_attr(verus_keep_ghost, verus_spec(
            invariant ni < N, di <= N, original_ns.len() == N, original_ds.len() == N,
                original_n == factors_product(original_ns, N as int),
                original_d == factors_product(original_ds, N as int), original_d > 0,
                expected == cancelled_factors(original_ns, original_ds, 0, 0),
                expected == cancelled_factors(numerators@, denominators@, ni as int, di as int),
                forall|j: int| 0 <= j < N ==> #[trigger] denominators[j] > 0,
                forall|i: int, j: int| 0 <= i < N && 0 <= j < N
                    && (i < ni || i == ni && j < di)
                    ==> gcd(#[trigger] numerators[i] as nat, #[trigger] denominators[j] as nat) == 1,
                factors_product(numerators@, N as int) * original_d
                    == original_n * factors_product(denominators@, N as int),
            decreases N - di,
        ))]
        while di < N && numerators[ni] != 1 {
            if denominators[di] != 1 {
                proof_decl! { let ghost before_ns = numerators@; }
                proof_decl! { let ghost before_ds = denominators@; }
                let (n, d) = reduce(numerators[ni], denominators[di]);
                proof! {
                    cancel_preserves_processed_coprimality(before_ns, before_ds, ni as int, di as int);
                    cancel_preserves_product_fraction(before_ns, before_ds, ni as int, di as int);
                    factors_positive(before_ds, N as int);
                }
                numerators[ni] = n;
                denominators[di] = d;
                proof! {
                    assert(numerators@ == before_ns.update(ni as int, n));
                    assert(denominators@ == before_ds.update(di as int, d));
                    product_fraction_transitive(original_n, original_d,
                        factors_product(before_ns, N as int), factors_product(before_ds, N as int),
                        factors_product(numerators@, N as int), factors_product(denominators@, N as int));
                    assert(expected == cancelled_factors(numerators@, denominators@, ni as int, di as int + 1));
                }
            } else {
                proof! {
                    assert(gcd(numerators[ni as int] as nat, 1) == gcd(1, 0));
                    assert forall|i: int, j: int| 0 <= i < N && 0 <= j < N
                        && (i < ni || i == ni && j <= di)
                        implies gcd(#[trigger] numerators[i] as nat, #[trigger] denominators[j] as nat) == 1 by {
                        if i == ni && j == di { assert(denominators[j] == 1); }
                    }
                    assert(expected == cancelled_factors(numerators@, denominators@, ni as int, di as int + 1));
                }
            }
            di += 1;
        }
        proof! {
            assert forall|i: int, j: int| 0 <= i <= ni && 0 <= j < N
                implies gcd(#[trigger] numerators[i] as nat, #[trigger] denominators[j] as nat) == 1 by {
                if i == ni && di < N {
                    assert(numerators[i] == 1);
                    unit_is_coprime(denominators[j] as nat);
                }
            }
            assert(expected == cancelled_factors(numerators@, denominators@, ni as int + 1, 0));
        }
        ni += 1;
    }
    proof! {
        assert(expected == (numerators@, denominators@));
        factors_positive(denominators@, N as int);
        mutually_coprime_products(numerators@, denominators@);
    }
    let magnitude = checked_factors_product(&numerators)?;
    let denominator = checked_factors_product(&denominators)?;
    proof! {
        assert((magnitude == 0) == (original_n == 0)) by (nonlinear_arith)
            requires denominator > 0, original_d > 0,
                magnitude as nat * original_d == original_n * denominator as nat;
        if magnitude == 0 {
            super::gcd::gcd_symmetric(0, denominator as nat);
            assert(gcd(denominator as nat, 0) == denominator);
        }
    }
    Some((magnitude, denominator))
}
