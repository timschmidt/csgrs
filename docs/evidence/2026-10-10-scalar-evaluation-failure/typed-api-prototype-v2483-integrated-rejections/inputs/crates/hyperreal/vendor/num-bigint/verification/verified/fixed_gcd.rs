#[cfg(verus_keep_ghost)]
use super::division::gcd;
#[cfg(verus_keep_ghost)]
use super::gcd::{
    gcd_bound, gcd_odd_power_two, gcd_power_two_factors, gcd_subtract, gcd_symmetric,
};
#[cfg(verus_keep_ghost)]
use super::limbs::{prefix_bound, value, weight};
#[cfg(verus_keep_ghost)]
use vstd::arithmetic::{div_mod::*, power2::*};
#[cfg(verus_keep_ghost)]
use vstd::prelude::*;

use super::limbs;
use core::cmp::Ordering;

/// Keep the word fast path available to the caller's BigUint construction.
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub(crate) enum FixedGcd<const N: usize> {
    Word { value: u128, shift: u32 },
    Limbs([u64; N]),
}

#[cfg(verus_keep_ghost)]
verus! {
pub(crate) open spec fn result_value<const N: usize>(result: FixedGcd<N>) -> nat {
    match result {
        FixedGcd::Word { value: word, shift } => (word as nat) * pow2(shift as nat),
        FixedGcd::Limbs(words) => value(words@),
    }
}
}

/// Binary GCD for a bounded little-endian buffer, with a verified scalar exit.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires 2 <= N <= u32::MAX / 64,
    ensures result_value(result) == gcd(value(input_left@), value(input_right@)),
))]
pub(crate) fn gcd_fixed<const N: usize>(
    input_left: [u64; N],
    input_right: [u64; N],
) -> FixedGcd<N> {
    let mut left = input_left;
    let mut right = input_right;
    let width = N as u32 * 64;
    let left_shift = limbs::trailing_zeros(&left);
    if left_shift == width {
        proof! { gcd_symmetric(value(left@), value(right@)); }
        return FixedGcd::Limbs(right);
    }
    let right_shift = limbs::trailing_zeros(&right);
    if right_shift == width {
        return FixedGcd::Limbs(left);
    }
    let common_shift = left_shift.min(right_shift);
    limbs::shift_right(&mut left, left_shift);
    limbs::shift_right(&mut right, right_shift);
    proof! {
        gcd_power_two_factors(value(left@), value(right@), left_shift as nat, right_shift as nat);
        gcd_bound(value(input_left@), value(input_right@));
        prefix_bound(input_left@, N as int);
    }
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant
            2 <= N <= u32::MAX / 64,
            value(left@) > 0, value(right@) > 0,
            value(left@) % 2 == 1, value(right@) % 2 == 1,
            common_shift < 64 * N,
            gcd(value(input_left@), value(input_right@)) < weight(N as nat),
            gcd(value(left@), value(right@)) * pow2(common_shift as nat)
                == gcd(value(input_left@), value(input_right@)),
        decreases value(left@) + value(right@),
    ))]
    loop {
        // Verus does not yet support Rust's combined if-let chains.
        #[allow(clippy::collapsible_if)]
        if let Some(left_word) = limbs::to_u128(&left) {
            if let Some(right_word) = limbs::to_u128(&right) {
                return FixedGcd::Word {
                    value: super::gcd::gcd_u128(left_word, right_word),
                    shift: common_shift,
                };
            }
        }
        let ordering = limbs::compare(&left, &right);
        if matches!(ordering, Ordering::Equal) {
            proof! {
                lemma_mod_self_0(value(left@) as int);
                assert(gcd(value(left@), value(right@)) == value(left@));
            }
            proof! {
                if common_shift == 0 {
                    lemma2_to64();
                    assert(value(left@) * pow2(common_shift as nat) == value(left@))
                        by (nonlinear_arith) requires pow2(common_shift as nat) == 1;
                }
            }
            if common_shift != 0 {
                limbs::shift_left(&mut left, common_shift);
            }
            return FixedGcd::Limbs(left);
        }
        if matches!(ordering, Ordering::Greater) {
            proof! { gcd_symmetric(value(left@), value(right@)); }
            core::mem::swap(&mut left, &mut right);
        }
        proof! { gcd_subtract(value(left@), value(right@)); }
        limbs::subtract(&mut right, &left);
        let shift = limbs::trailing_zeros(&right);
        proof! {
            lemma_pow2_pos(shift as nat);
            let normalized = value(right@) / pow2(shift as nat);
            gcd_odd_power_two(value(left@), normalized, shift as nat);
            assert(normalized <= value(right@)) by (nonlinear_arith)
                requires value(right@) == normalized * pow2(shift as nat),
                    normalized >= 0, pow2(shift as nat) >= 1;
        }
        limbs::shift_right(&mut right, shift);
    }
}
