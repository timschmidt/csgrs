#[cfg(verus_keep_ghost)]
use vstd::arithmetic::div_mod::*;
#[cfg(verus_keep_ghost)]
use vstd::prelude::*;

#[cfg(verus_keep_ghost)]
verus! {
/// Euclid's mathematical GCD over unbounded natural numbers.
pub(crate) open spec fn gcd(left: nat, right: nat) -> nat
    decreases right,
{
    if right == 0 { left } else { gcd(right, left % right) }
}

proof fn common_divisors_step(left: nat, right: nat, divisor: nat)
    requires right > 0, divisor > 0, right % divisor == 0,
    ensures left % divisor == (left % right) % divisor,
{
    lemma_fundamental_div_mod(left as int, right as int);
    lemma_mul_mod_noop_right((left / right) as int, right as int, divisor as int);
    lemma_mod_twice((left % right) as int, divisor as int);
    lemma_add_mod_noop((left / right * right) as int, (left % right) as int, divisor as int);
    lemma_small_mod(0, divisor);
}

pub(crate) proof fn gcd_preserves_common_divisors(left: nat, right: nat)
    ensures forall|divisor: nat| divisor > 0 ==>
        (#[trigger] (gcd(left, right) % divisor) == 0
            <==> left % divisor == 0 && right % divisor == 0),
    decreases right,
{
    if right != 0 {
        gcd_preserves_common_divisors(right, left % right);
        assert forall|divisor: nat| divisor > 0 implies
            (#[trigger] (gcd(left, right) % divisor) == 0
                <==> left % divisor == 0 && right % divisor == 0) by {
            assert(gcd(left, right) == gcd(right, left % right));
            assert((gcd(right, left % right) % divisor == 0)
                <==> right % divisor == 0 && (left % right) % divisor == 0);
            if right % divisor == 0 {
                common_divisors_step(left, right, divisor);
            }
        }
    }
}

pub(crate) proof fn gcd_positive(left: nat, right: nat)
    requires left != 0 || right != 0,
    ensures gcd(left, right) > 0,
    decreases right,
{
    if right != 0 {
        gcd_positive(right, left % right);
    }
}

/// The recursive specification really is the greatest common divisor, with
/// the conventional gcd(0, 0) = 0. Thus preserving it has a mathematical meaning
/// independent of the optimized machine implementation.
pub(crate) proof fn gcd_characterization(left: nat, right: nat)
    ensures
        gcd(left, right) == 0 <==> left == 0 && right == 0,
        gcd(left, right) > 0 ==> left % gcd(left, right) == 0 && right % gcd(left, right) == 0,
        forall|divisor: nat| divisor > 0 && #[trigger] (left % divisor) == 0 && right % divisor == 0
            && gcd(left, right) > 0 ==> divisor <= gcd(left, right),
{
    let result = gcd(left, right);
    gcd_preserves_common_divisors(left, right);
    if left != 0 || right != 0 {
        gcd_positive(left, right);
        lemma_mod_self_0(result as int);
        assert forall|divisor: nat| divisor > 0 && #[trigger] (left % divisor) == 0 && right % divisor == 0
            && result > 0 implies divisor <= result by {
            if divisor > result {
                lemma_small_mod(result, divisor);
                assert(result % divisor == 0);
            }
        }
    }
}

proof fn gcd_swap_increasing(left: nat, right: nat)
    requires left < right,
    ensures gcd(left, right) == gcd(right, left),
{
    lemma_small_mod(left, right);
}

/// An upper bound on a divisor's high limb gives a lower bound on the quotient.
proof fn high_quotient_bound(left: u128, right: u128)
    requires
        right > u64::MAX,
        left >= 5 * right as int,
    ensures
        (right >> 64) < u64::MAX,
        0 <= ((left >> 64) as int / ((right >> 64) as int + 1)) * right <= left,
{
    let base: int = 0x1_0000_0000_0000_0000;
    assert((left >> 64) == left / 0x1_0000_0000_0000_0000u128) by (bit_vector);
    assert((right >> 64) == right / 0x1_0000_0000_0000_0000u128) by (bit_vector);
    let high_left = (left >> 64) as int;
    let high_right = (right >> 64) as int;
    let quotient = high_left / (high_right + 1);
    lemma_fundamental_div_mod(left as int, base);
    lemma_fundamental_div_mod(right as int, base);
    lemma_fundamental_div_mod(high_left, high_right + 1);
    assert(high_right < u64::MAX) by (nonlinear_arith)
        requires
            0 <= left <= u128::MAX,
            5 * right <= left,
            0 <= (right as int) % base < base,
            right == high_right * base + (right as int) % base,
            base == 0x1_0000_0000_0000_0000,
    ;
    assert(0 <= quotient * right <= left) by (nonlinear_arith)
        requires
            0 <= quotient,
            0 < high_right + 1,
            0 <= high_left % (high_right + 1),
            high_left == quotient * (high_right + 1) + high_left % (high_right + 1),
            high_left * base <= left,
            0 <= right < (high_right + 1) * base,
            base > 0,
    ;
}
}

/// Exact remainder step used by the two-limb GCD reducer.
/// Small quotients use subtraction; larger quotients use the high-limb estimate
/// before falling back to `%`. Every intermediate subtraction stays nonnegative.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires left >= right, right > u64::MAX,
    ensures result == left % right, result < right,
))]
#[inline]
pub(crate) fn gcd_remainder(left: u128, right: u128) -> u128 {
    let difference = left - right;
    proof! { lemma_mod_sub_multiples_vanish(left as int, right as int); }
    if difference < right {
        proof! { lemma_small_mod(difference as nat, right as nat); }
        return difference;
    }
    let second_difference = difference - right;
    proof! { lemma_mod_sub_multiples_vanish(difference as int, right as int); }
    if second_difference < right {
        proof! { lemma_small_mod(second_difference as nat, right as nat); }
        return second_difference;
    }
    let third_difference = second_difference - right;
    proof! { lemma_mod_sub_multiples_vanish(second_difference as int, right as int); }
    if third_difference < right {
        proof! { lemma_small_mod(third_difference as nat, right as nat); }
        return third_difference;
    }
    let fourth_difference = third_difference - right;
    proof! { lemma_mod_sub_multiples_vanish(third_difference as int, right as int); }
    if fourth_difference < right {
        proof! { lemma_small_mod(fourth_difference as nat, right as nat); }
        return fourth_difference;
    }
    proof! {
        high_quotient_bound(left, right);
        assert((left >> 64) <= u64::MAX) by (bit_vector);
    }
    let high_quotient = ((left >> 64) as u64) / (((right >> 64) as u64) + 1);
    proof! {
        assert(right as int * high_quotient as int == high_quotient as int * right as int)
            by (nonlinear_arith);
    }
    let approximate = left - right * high_quotient as u128;
    proof! {
        lemma_mod_multiples_vanish(-(high_quotient as int), left as int, right as int);
        assert(right as int * -(high_quotient as int) + left as int
            == left as int - right as int * high_quotient as int) by (nonlinear_arith);
    }
    if approximate < right {
        proof! { lemma_small_mod(approximate as nat, right as nat); }
        approximate
    } else {
        let corrected = approximate - right;
        proof! { lemma_mod_sub_multiples_vanish(approximate as int, right as int); }
        if corrected < right {
            proof! { lemma_small_mod(corrected as nat, right as nat); }
            corrected
        } else {
            corrected % right
        }
    }
}

/// Reduce an arbitrary pair to a pair with a machine-word second operand,
/// preserving its mathematical GCD. The loop discharges every precondition of
/// `gcd_remainder` and strictly decreases the second operand.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures
        result.0 >= result.1,
        result.1 <= u64::MAX,
        gcd(result.0 as nat, result.1 as nat) == gcd(left as nat, right as nat),
))]
#[inline]
pub(crate) fn reduce_gcd_to_word(left: u128, right: u128) -> (u128, u128) {
    let (mut larger, mut smaller) = if left < right {
        proof! { gcd_swap_increasing(left as nat, right as nat); }
        (right, left)
    } else {
        (left, right)
    };
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant
            larger >= smaller,
            gcd(larger as nat, smaller as nat) == gcd(left as nat, right as nat),
        decreases smaller,
    ))]
    while smaller > u64::MAX as u128 {
        let remainder = gcd_remainder(larger, smaller);
        proof! {
            assert(gcd(larger as nat, smaller as nat) == gcd(smaller as nat, remainder as nat));
        }
        larger = smaller;
        smaller = remainder;
    }
    (larger, smaller)
}
