//! Exact unsigned binary GCD in fixed limb buffers with a retained native exit.
//!
//! Inputs use base-2^64 digits, least significant first, with high zero padding
//! permitted. The kernel removes powers of two, subtracts ordered odd magnitudes
//! and reuses the native GCD once both residuals fit u128. The result retains
//! either a native factor plus a binary shift or the complete limb value; the
//! rational adapter materializes that result as a BigUint.
//!
//! The kernel has no width-decline Option: for its admitted dimensions every
//! input pair, including zero, has a result. Width admission of arbitrary-precision
//! inputs belongs to the caller. Kernel bodies use owned fixed arrays and native
//! arithmetic without heap allocation; later result materialization can allocate.
//! Array width bounds storage for a chosen instantiation, while reductions depend
//! on the operands. No local total-work, stack or cancellation budget is enforced.
//! Ghost relations and loop invariants describe the integer GCD, not a measured
//! runtime bound or a fresh verification result.

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

/// Two exact result encodings that preserve the kernel's chosen exit path.
///
/// Word denotes `value * 2^shift`; Limbs contains the complete unsigned GCD in
/// numerical little-endian order. A Word result can represent a GCD larger than
/// u128 because its common binary factor is retained separately. Conversely,
/// Limbs may contain a small value or zero when an input was zero. The tag is
/// therefore not a unique encoding determined only by result bit length.
///
/// The enum can be constructed with arbitrary raw fields; it performs no
/// validation. Properties such as a positive odd Word factor and a result fitting
/// N limbs are established by gcd_fixed for admitted inputs, not by the enum
/// definition. Consumers must preserve the shift when materializing Word.
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub(crate) enum FixedGcd<const N: usize> {
    /// A native GCD factor with its common power of two still factored out.
    ///
    /// The kernel selects this exit before checking limb equality whenever both
    /// positive odd residuals fit u128. Its produced value is positive and odd;
    /// shift is the minimum of the original nonzero operands' trailing-zero counts.
    /// The complete represented result fits the original buffer but need not fit a
    /// native word. Raw construction does not enforce these producer invariants.
    Word { /** Unshifted factor. */ value: u128, /** Power-of-two exponent. */ shift: u32 },
    /// The complete GCD magnitude with any removed binary factor already restored.
    ///
    /// Zero-input exits copy the other operand, including high zero padding and the
    /// all-zero result for two zeros. Nonzero limb-equality exits restore the common
    /// shift in place after the native exit did not apply. No further shift should
    /// be applied by the consumer.
    Limbs(/** Complete base-2^64 magnitude, least significant digit first. */ [u64; N]),
}

#[cfg(verus_keep_ghost)]
verus! {
/// Decode either result encoding into one mathematical unsigned integer.
///
/// Word multiplies its native factor by `2^shift`; Limbs uses the shared positional
/// value relation. This ghost denotation accepts arbitrary raw fields and lengths,
/// without proving that the value is a GCD, that the native factor is odd, or that
/// the result fits N limbs. The gcd_fixed postcondition supplies the GCD identity;
/// its producer paths establish the encoding-specific invariants.
pub(crate) open spec fn result_value<const N: usize>(result: FixedGcd<N>) -> nat {
    match result {
        FixedGcd::Word { value: word, shift } => (word as nat) * pow2(shift as nat),
        FixedGcd::Limbs(words) => value(words@),
    }
}
}

/// Compute the exact GCD of two same-width unsigned limb magnitudes.
///
/// Requires `2 <= N <= u32::MAX / 64`. These dimension bounds are not validated
/// inside the kernel; the rational adapter enforces them for its instantiations.
/// All values in admitted arrays are supported, including zero and high padding.
/// If left is zero, returns Limbs(right); otherwise, if right is zero, returns
/// Limbs(left). Thus two zeros return a complete zero magnitude.
///
/// For nonzero inputs, removes each operand's trailing binary factor and retains
/// the smaller count as the common shift. Each iteration first tries to narrow
/// left, then right; if both fit, returns their native GCD with that shift. If
/// not, equal odd residuals return Limbs after exact shift restoration. Unequal
/// residuals are ordered, the smaller is subtracted from the larger, and powers
/// of two are removed from the positive difference before the next iteration.
/// The residual sum decreases and the common-factor GCD invariant is preserved.
///
/// The final GCD fits the original N limbs, which justifies lossless restoration;
/// the Word encoding can still represent more than 128 bits. Input arrays are
/// passed by value and local mutation does not alter caller-owned arrays. There
/// is no allocation, None result or exhausted-budget outcome in this kernel.
/// Dimensions outside the contract may bypass checks on zero inputs or panic on
/// native narrowing; apparent success there does not extend the admitted domain.
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
