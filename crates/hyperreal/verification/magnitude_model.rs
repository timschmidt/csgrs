//! Mathematical binary-magnitude certificates for nonzero rational magnitudes.
//! BigUint bit-length and shifted-comparison refinement remains a separate obligation.
use vstd::prelude::*;
use vstd::arithmetic::power2::{pow2, lemma_pow2_pos, lemma_pow2_unfold, lemma_pow2_adds, lemma_pow2_strictly_increases};

verus! {

pub open spec fn binary_numerator(exponent: int) -> nat {
    pow2(if exponent >= 0 { exponent as nat } else { 0 })
}

pub open spec fn binary_denominator(exponent: int) -> nat {
    pow2(if exponent < 0 { (-exponent) as nat } else { 0 })
}

pub open spec fn has_bit_length(value: nat, bits: nat) -> bool {
    bits > 0 && pow2((bits - 1) as nat) <= value && value < pow2(bits)
}

pub open spec fn in_binade(numerator: nat, denominator: nat, exponent: int) -> bool {
    denominator > 0
        && binary_numerator(exponent) * denominator <= numerator * binary_denominator(exponent)
        && numerator * binary_denominator(exponent) < 2 * binary_numerator(exponent) * denominator
}

pub open spec fn is_binary_scale(numerator: nat, denominator: nat, exponent: int) -> bool {
    denominator > 0
        && numerator * binary_denominator(exponent) == binary_numerator(exponent) * denominator
}

pub open spec fn below_aligned(numerator: nat, denominator: nat, numerator_bits: nat, denominator_bits: nat) -> bool {
    let difference = numerator_bits as int - denominator_bits as int;
    numerator * binary_denominator(difference) < binary_numerator(difference) * denominator
}

pub open spec fn aligned_msd(numerator: nat, denominator: nat, numerator_bits: nat, denominator_bits: nat) -> int {
    numerator_bits as int - denominator_bits as int
        - if below_aligned(numerator, denominator, numerator_bits, denominator_bits) { 1int } else { 0int }
}

proof fn binary_positive(exponent: int)
    ensures binary_numerator(exponent) > 0, binary_denominator(exponent) > 0,
{
    lemma_pow2_pos(if exponent >= 0 { exponent as nat } else { 0 });
    lemma_pow2_pos(if exponent < 0 { (-exponent) as nat } else { 0 });
}

proof fn binary_step(exponent: int)
    ensures binary_numerator(exponent) * binary_denominator(exponent - 1)
        == 2 * binary_numerator(exponent - 1) * binary_denominator(exponent),
{
    vstd::arithmetic::power::lemma_pow0(2);
    if exponent > 0 {
        lemma_pow2_unfold(exponent as nat);
    } else if exponent == 0 {
        lemma_pow2_unfold(1);
    } else {
        lemma_pow2_unfold((1 - exponent) as nat);
    }
    assert(binary_numerator(exponent) * binary_denominator(exponent - 1)
        == 2 * binary_numerator(exponent - 1) * binary_denominator(exponent)) by (nonlinear_arith)
        requires pow2(0) == 1,
            exponent > 0 ==> binary_denominator(exponent) == 1 && binary_denominator(exponent - 1) == 1
                && binary_numerator(exponent) == 2 * binary_numerator(exponent - 1),
            exponent <= 0 ==> binary_numerator(exponent) == 1 && binary_numerator(exponent - 1) == 1
                && binary_denominator(exponent - 1) == 2 * binary_denominator(exponent);
}

/// Aligned bit lengths bound the ratio strictly between one half and two.
proof fn aligned_bit_length_bounds(numerator: nat, denominator: nat, numerator_bits: nat, denominator_bits: nat)
    requires has_bit_length(numerator, numerator_bits), has_bit_length(denominator, denominator_bits),
    ensures numerator > 0, denominator > 0,
        ({let e = numerator_bits as int - denominator_bits as int;
            &&& binary_numerator(e) * denominator < 2 * numerator * binary_denominator(e)
            &&& numerator * binary_denominator(e) < 2 * binary_numerator(e) * denominator}),
{
    vstd::arithmetic::power::lemma_pow0(2);
    lemma_pow2_pos((numerator_bits - 1) as nat);
    lemma_pow2_pos((denominator_bits - 1) as nat);
    lemma_pow2_unfold(numerator_bits);
    lemma_pow2_unfold(denominator_bits);
    let low_n = pow2((numerator_bits - 1) as nat);
    let low_d = pow2((denominator_bits - 1) as nat);
    if numerator_bits >= denominator_bits {
        let shift = (numerator_bits - denominator_bits) as nat;
        lemma_pow2_pos(shift);
        lemma_pow2_adds((denominator_bits - 1) as nat, shift);
        assert(low_n == low_d * pow2(shift));
        assert(pow2(shift) * denominator < 2 * numerator
            && numerator < 2 * pow2(shift) * denominator) by (nonlinear_arith)
            requires low_d > 0, pow2(shift) > 0, low_n == low_d * pow2(shift),
                low_n <= numerator, numerator < 2 * low_n,
                low_d <= denominator, denominator < 2 * low_d;
    } else {
        let shift = (denominator_bits - numerator_bits) as nat;
        lemma_pow2_pos(shift);
        lemma_pow2_adds((numerator_bits - 1) as nat, shift);
        assert(low_d == low_n * pow2(shift));
        assert(denominator < 2 * numerator * pow2(shift)
            && numerator * pow2(shift) < 2 * denominator) by (nonlinear_arith)
            requires low_n > 0, pow2(shift) > 0, low_d == low_n * pow2(shift),
                low_n <= numerator, numerator < 2 * low_n,
                low_d <= denominator, denominator < 2 * low_d;
    }
    let e = numerator_bits as int - denominator_bits as int;
    if numerator_bits >= denominator_bits {
        let shift = (numerator_bits - denominator_bits) as nat;
        assert(binary_numerator(e) == pow2(shift));
        assert(binary_denominator(e) == 1);
        assert(binary_numerator(e) * denominator < 2 * numerator * binary_denominator(e)
            && numerator * binary_denominator(e) < 2 * binary_numerator(e) * denominator) by (nonlinear_arith)
            requires binary_numerator(e) == pow2(shift), binary_denominator(e) == 1,
                pow2(shift) * denominator < 2 * numerator,
                numerator < 2 * pow2(shift) * denominator;
    } else {
        let shift = (denominator_bits - numerator_bits) as nat;
        assert(binary_numerator(e) == 1);
        assert(binary_denominator(e) == pow2(shift));
        assert(binary_numerator(e) * denominator < 2 * numerator * binary_denominator(e)
            && numerator * binary_denominator(e) < 2 * binary_numerator(e) * denominator) by (nonlinear_arith)
            requires binary_numerator(e) == 1, binary_denominator(e) == pow2(shift),
                denominator < 2 * numerator * pow2(shift),
                numerator * pow2(shift) < 2 * denominator;
    }
}

/// The one-comparison correction returns the exact floor binary exponent.
/// Equality of the aligned operands certifies precisely a binary scale,
/// including unreduced numerator and denominator pairs.
pub proof fn bit_lengths_determine_binade(numerator: nat, denominator: nat, numerator_bits: nat, denominator_bits: nat)
    requires has_bit_length(numerator, numerator_bits), has_bit_length(denominator, denominator_bits),
    ensures
        in_binade(numerator, denominator, aligned_msd(numerator, denominator, numerator_bits, denominator_bits)),
        (is_binary_scale(numerator, denominator, aligned_msd(numerator, denominator, numerator_bits, denominator_bits))
            <==> numerator * binary_denominator(numerator_bits as int - denominator_bits as int)
                == binary_numerator(numerator_bits as int - denominator_bits as int) * denominator),
{
    aligned_bit_length_bounds(numerator, denominator, numerator_bits, denominator_bits);
    let difference = numerator_bits as int - denominator_bits as int;
    binary_positive(difference);
    if below_aligned(numerator, denominator, numerator_bits, denominator_bits) {
        binary_positive(difference - 1);
        binary_step(difference);
        let a = binary_numerator(difference);
        let b = binary_denominator(difference);
        let previous_a = binary_numerator(difference - 1);
        let previous_b = binary_denominator(difference - 1);
        assert(previous_a * denominator < numerator * previous_b
            && numerator * previous_b < 2 * previous_a * denominator) by (nonlinear_arith)
            requires a > 0, b > 0, previous_a > 0, previous_b > 0,
                a * previous_b == 2 * previous_a * b,
                a * denominator < 2 * numerator * b, numerator * b < a * denominator;
    }
}

/// Signed binary scales compose for arbitrary integer exponents.
pub proof fn binary_scales_compose(left: int, right: int)
    ensures binary_numerator(left + right) * (binary_denominator(left) * binary_denominator(right))
        == (binary_numerator(left) * binary_numerator(right)) * binary_denominator(left + right),
{
    let lp = if left >= 0 { left as nat } else { 0 };
    let ln = if left < 0 { (-left) as nat } else { 0 };
    let rp = if right >= 0 { right as nat } else { 0 };
    let rn = if right < 0 { (-right) as nat } else { 0 };
    let sp = if left + right >= 0 { (left + right) as nat } else { 0 };
    let sn = if left + right < 0 { (-left - right) as nat } else { 0 };
    assert(sp + ln + rn == lp + rp + sn);
    lemma_pow2_adds(ln, rn);
    lemma_pow2_adds(sp, ln + rn);
    lemma_pow2_adds(lp, rp);
    lemma_pow2_adds(lp + rp, sn);
}

/// Binary scaling moves a binade by exactly the signed scale exponent.
pub proof fn binary_scale_preserves_binade(numerator: nat, denominator: nat, exponent: int, offset: int)
    requires in_binade(numerator, denominator, exponent),
    ensures in_binade(numerator * binary_numerator(offset), denominator * binary_denominator(offset), exponent + offset),
        (is_binary_scale(numerator, denominator, exponent)
            <==> is_binary_scale(numerator * binary_numerator(offset), denominator * binary_denominator(offset), exponent + offset)),
{
    binary_positive(exponent);
    binary_positive(offset);
    binary_positive(exponent + offset);
    binary_scales_compose(exponent, offset);
    let a = binary_numerator(exponent);
    let b = binary_denominator(exponent);
    let c = binary_numerator(offset);
    let d = binary_denominator(offset);
    let p = binary_numerator(exponent + offset);
    let q = binary_denominator(exponent + offset);
    assert(denominator * d > 0) by (nonlinear_arith) requires denominator > 0, d > 0;
    assert(p * (denominator * d) <= (numerator * c) * q
        && (numerator * c) * q < 2 * p * (denominator * d)) by (nonlinear_arith)
        requires a > 0, b > 0, c > 0, d > 0, p > 0, q > 0,
            p * (b * d) == (a * c) * q,
            a * denominator <= numerator * b, numerator * b < 2 * a * denominator;
    assert((numerator * b == a * denominator)
        <==> (numerator * c) * q == p * (denominator * d)) by (nonlinear_arith)
        requires a > 0, b > 0, c > 0, d > 0, p > 0, q > 0,
            p * (b * d) == (a * c) * q;
}


proof fn separated_binary_scales(left: int, right: int)
    requires left < right,
    ensures 2 * binary_numerator(left) * binary_denominator(right)
        <= binary_numerator(right) * binary_denominator(left),
{
    let lp = if left >= 0 { left as nat } else { 0 };
    let ln = if left < 0 { (-left) as nat } else { 0 };
    let rp = if right >= 0 { right as nat } else { 0 };
    let rn = if right < 0 { (-right) as nat } else { 0 };
    assert(lp + rn + 1 <= rp + ln);
    lemma_pow2_adds(lp, rn);
    lemma_pow2_adds(rp, ln);
    lemma_pow2_unfold(lp + rn + 1);
    if lp + rn + 1 < rp + ln {
        lemma_pow2_strictly_increases(lp + rn + 1, rp + ln);
    }
    assert(2 * binary_numerator(left) * binary_denominator(right)
        <= binary_numerator(right) * binary_denominator(left)) by (nonlinear_arith)
        requires pow2(lp + rn + 1) == 2 * pow2(lp + rn),
            pow2(lp + rn + 1) <= pow2(rp + ln),
            pow2(lp + rn) == binary_numerator(left) * binary_denominator(right),
            pow2(rp + ln) == binary_numerator(right) * binary_denominator(left);
}

/// Disjoint adjacent power-of-two intervals make the floor exponent unique.
pub proof fn binade_is_unique(numerator: nat, denominator: nat, left: int, right: int)
    requires in_binade(numerator, denominator, left), in_binade(numerator, denominator, right),
    ensures left == right,
{
    binary_positive(left);
    binary_positive(right);
    if left != right {
        let smaller = if left < right { left } else { right };
        let larger = if left < right { right } else { left };
        separated_binary_scales(smaller, larger);
        let a = binary_numerator(smaller);
        let b = binary_denominator(smaller);
        let c = binary_numerator(larger);
        let d = binary_denominator(larger);
        assert(false) by (nonlinear_arith)
            requires denominator > 0, b > 0, d > 0,
                2 * a * d <= c * b,
                numerator * b < 2 * a * denominator,
                c * denominator <= numerator * d;
    }
}

/// Connect the executable helper's shared exponent specification to the
/// rational interval. The BigUint caller must still prove the two bit lengths
/// and the meaning of its borrowed shifted comparison.
pub(crate) proof fn checked_exponent_denotes_scaled_binade(
    numerator: nat, denominator: nat, numerator_bits: u64, denominator_bits: u64,
    below: bool, offset: i32,
)
    requires has_bit_length(numerator, numerator_bits as nat),
        has_bit_length(denominator, denominator_bits as nat),
        below == below_aligned(numerator, denominator, numerator_bits as nat, denominator_bits as nat),
    ensures in_binade(numerator * binary_numerator(offset as int), denominator * binary_denominator(offset as int),
        crate::verified::word::bit_length_msd(numerator_bits, denominator_bits, below, offset)),
{
    bit_lengths_determine_binade(numerator, denominator, numerator_bits as nat, denominator_bits as nat);
    let exponent = aligned_msd(numerator, denominator, numerator_bits as nat, denominator_bits as nat);
    binary_scale_preserves_binade(numerator, denominator, exponent, offset as int);
}

} // verus!
