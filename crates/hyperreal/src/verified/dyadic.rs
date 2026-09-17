//! Signed differences and canonical power-of-two denominator reduction.

#[cfg(verus_keep_ghost)]
use super::division::gcd;
#[cfg(verus_keep_ghost)]
use super::limbs::value;
#[cfg(verus_keep_ghost)]
use vstd::arithmetic::{div_mod::*, power2::*};
#[cfg(verus_keep_ghost)]
use vstd::{math::abs, prelude::*};

use super::limbs;
use core::cmp::Ordering;

#[cfg(verus_keep_ghost)]
verus! {
pub(crate) open spec fn signed(negative: bool, magnitude: nat) -> int {
    if negative { -(magnitude as int) } else { magnitude as int }
}

/// Cancel any prefix of a known exact binary factor, retaining a positive
/// quotient and exposing oddness when the entire factor is removed.
pub(crate) proof fn cancel_power_two(magnitude: nat, trailing: nat, removed: nat)
    requires removed <= trailing,
        magnitude / pow2(trailing) > 0,
        (magnitude / pow2(trailing)) % 2 == 1,
        magnitude == (magnitude / pow2(trailing)) * pow2(trailing),
    ensures magnitude / pow2(removed) > 0,
        magnitude == (magnitude / pow2(removed)) * pow2(removed),
        removed == trailing ==> (magnitude / pow2(removed)) % 2 == 1,
{
    let odd = magnitude / pow2(trailing);
    let remaining = pow2((trailing - removed) as nat);
    let factor = pow2(removed);
    lemma_pow2_pos(removed);
    lemma_pow2_pos((trailing - removed) as nat);
    lemma_pow2_adds(removed, (trailing - removed) as nat);
    assert(magnitude == factor * (odd * remaining)) by (nonlinear_arith)
        requires magnitude == odd * pow2(trailing), pow2(trailing) == factor * remaining;
    assert(magnitude == (odd * remaining) * factor) by (nonlinear_arith)
        requires magnitude == factor * (odd * remaining);
    lemma_fundamental_div_mod_converse_div(magnitude as int, factor as int,
        (odd * remaining) as int, 0);
    assert(odd * remaining > 0) by (nonlinear_arith) requires odd > 0, remaining > 0;
    assert(magnitude == (magnitude / factor) * factor) by (nonlinear_arith)
        requires magnitude == factor * (odd * remaining), magnitude / factor == odd * remaining;
}

/// Exact cancellation preserves the signed fraction by cross multiplication.
pub(crate) proof fn reduction_preserves_fraction(
    negative: bool, before: nat, after: nat, before_shift: nat, after_shift: nat,
)
    requires after_shift <= before_shift,
        before == after * pow2((before_shift - after_shift) as nat),
    ensures signed(negative, after) * pow2(before_shift)
        == signed(negative, before) * pow2(after_shift),
{
    lemma_pow2_adds(after_shift, (before_shift - after_shift) as nat);
    assert(signed(negative, after) * pow2(before_shift)
        == signed(negative, before) * pow2(after_shift)) by (nonlinear_arith)
        requires before == after * pow2((before_shift - after_shift) as nat),
            pow2(before_shift) == pow2(after_shift) * pow2((before_shift - after_shift) as nat);
}

pub(crate) proof fn canonical_is_reduced(magnitude: nat, shift: nat)
    requires shift == 0 || magnitude % 2 == 1,
    ensures gcd(magnitude, pow2(shift)) == 1,
{
    assert(magnitude % 1 == 0);
    assert(gcd(1, 0) == 1);
    assert(gcd(magnitude, 1) == 1);
    if shift == 0 {
        lemma2_to64();
    } else {
        super::gcd::gcd_odd_power_two(magnitude, 1, shift);
        assert(1nat * pow2(shift) == pow2(shift)) by (nonlinear_arith);
    }
}
}

/// Zero is represented by None; every nonzero difference has an exact sign
/// and magnitude, with comparison discharging subtraction's precondition.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures result.is_none() <==> value(positive@) == value(negative@),
        match result {
            Some((minus, magnitude)) => value(magnitude@) > 0
                && minus == (value(positive@) < value(negative@))
                && signed(minus, value(magnitude@)) == value(positive@) as int - value(negative@) as int,
            None => true,
        },
))]
#[inline]
pub(crate) fn difference<const N: usize>(
    positive: [u64; N],
    negative: [u64; N],
) -> Option<(bool, [u64; N])> {
    match limbs::compare(&positive, &negative) {
        Ordering::Equal => None,
        Ordering::Greater => {
            let mut magnitude = positive;
            limbs::subtract(&mut magnitude, &negative);
            Some((false, magnitude))
        }
        Ordering::Less => {
            let mut magnitude = negative;
            limbs::subtract(&mut magnitude, &positive);
            Some((true, magnitude))
        }
    }
}

/// Remove the common binary factor of a nonzero magnitude and its dyadic
/// denominator. The result is an integer or has an odd numerator.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires N <= u32::MAX / 64, value(magnitude@) > 0,
    ensures result <= denominator_shift,
        value(final(magnitude)@) > 0,
        value(old(magnitude)@)
            == value(final(magnitude)@) * pow2((denominator_shift - result) as nat),
        result == 0 || value(final(magnitude)@) % 2 == 1,
        gcd(value(final(magnitude)@), pow2(result as nat)) == 1,
))]
#[inline]
pub(crate) fn normalize<const N: usize>(magnitude: &mut [u64; N], denominator_shift: u64) -> u64 {
    proof_decl! { let ghost original = magnitude@; }
    let trailing = limbs::trailing_zeros(magnitude);
    let common = (trailing as u64).min(denominator_shift);
    proof! {
        cancel_power_two(value(original), trailing as nat, common as nat);
        if common == 0 {
            lemma2_to64();
            lemma_div_basics(value(original) as int);
        }
    }
    if common != 0 {
        limbs::shift_right(magnitude, common as u32);
    }
    let reduced_shift = denominator_shift - common;
    proof! { canonical_is_reduced(value(magnitude@), reduced_shift as nat); }
    reduced_shift
}

/// Finish positive/negative accumulation at one binary scale, preserving the
/// exact fraction and selecting its canonical nonzero dyadic representation.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires N <= u32::MAX / 64,
    ensures result.is_none() <==> value(positive@) == value(negative@),
        match result {
            Some((minus, magnitude, reduced_shift)) => reduced_shift <= denominator_shift
                && minus == (value(positive@) < value(negative@))
                && value(magnitude@) > 0
                && (reduced_shift == 0 || value(magnitude@) % 2 == 1)
                && gcd(value(magnitude@), pow2(reduced_shift as nat)) == 1
                && value(magnitude@) * pow2((denominator_shift - reduced_shift) as nat)
                    == abs(value(positive@) as int - value(negative@) as int)
                && signed(minus, value(magnitude@)) * pow2(denominator_shift as nat)
                    == (value(positive@) as int - value(negative@) as int) * pow2(reduced_shift as nat),
            None => true,
        },
))]
#[inline]
pub(crate) fn finish<const N: usize>(
    positive: [u64; N],
    negative: [u64; N],
    denominator_shift: u64,
) -> Option<(bool, [u64; N], u64)> {
    let (minus, mut magnitude) = difference(positive, negative)?;
    proof_decl! { let ghost original = magnitude@; }
    let reduced_shift = normalize(&mut magnitude, denominator_shift);
    proof! {
        reduction_preserves_fraction(minus, value(original), value(magnitude@),
            denominator_shift as nat, reduced_shift as nat);
    }
    Some((minus, magnitude, reduced_shift))
}
