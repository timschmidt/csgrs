//! Exact unsigned schoolbook products for fixed-width dyadic and comparison carriers.
//!
//! Arrays contain base-2^64 digits with the least significant limb first. This is
//! numeric limb order, independent of host byte order, not a serialized byte format.
//! The kernels preserve the complete integer product; signs, binary scales, rational
//! reduction and subsequent accumulator overflow belong to their callers.
//!
//! Capacity is a caller precondition of the generic multiply routine, not a typed
//! runtime check. Fixed arrays avoid heap allocation in these bodies, but larger
//! const-generic dimensions increase output storage and arithmetic work without a
//! local stack/work/cancellation budget. Cargo and the verifier import the same
//! bodies; conditional loop contracts and ghost lemmas are erased from ordinary
//! execution and do not by themselves establish a fresh formal proof result.

#[cfg(verus_keep_ghost)]
use super::limbs::{
    prefix, prefix_equal, prefix_zero, radix, value, weight, weight_add, weight_step,
};
#[cfg(verus_keep_ghost)]
use vstd::prelude::*;

#[cfg(verus_keep_ghost)]
verus! {
/// Relate one updated limb to the exact value of a selected little-endian prefix.
///
/// Requires 0 <= index < length <= words.len(). The new prefix plus the old
/// indexed digit's weight equals the old prefix plus the replacement's weight;
/// all other digit contributions cancel. Induction shortens length down to
/// index+1, where equality of the untouched earlier prefix closes the identity.
/// Product and shifted-accumulator proofs use this after individual digit writes.
/// This ghost sequence relation neither mutates a runtime buffer nor proves its
/// caller's capacity/carry invariants without the separate preconditions.
pub(crate) proof fn prefix_update(words: Seq<u64>, index: int, digit: u64, length: int)
    requires 0 <= index < length <= words.len(),
    ensures prefix(words.update(index, digit), length) + words[index] * weight(index as nat)
        == prefix(words, length) + digit * weight(index as nat),
    decreases length - index,
{
    if length == index + 1 {
        prefix_equal(words.update(index, digit), words, index);
    } else {
        prefix_update(words, index, digit, length - 1);
    }
}
}

/// Compute one product digit and carry from four unrestricted u64 inputs.
///
/// Returns (low, high) satisfying
/// `low + high * 2^64 = addend + left * right + carry`.
/// The maximum total is `(2^64-1)^2 + 2*(2^64-1) = 2^128-1`, so the u128
/// expression cannot overflow and both returned digits fit u64. The caller's
/// current output digit and incoming row carry need no additional bound.
/// This inner schoolbook step allocates nothing and has no decline/error case.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures result.0 as nat + result.1 as nat * radix()
        == addend as nat + left as nat * right as nat + carry as nat,
))]
#[inline]
fn multiply_accumulate(addend: u64, left: u64, right: u64, carry: u64) -> (u64, u64) {
    proof! {
        assert(addend as int + left as int * right as int + carry as int <= u128::MAX)
            by (nonlinear_arith)
            requires 0 <= addend <= u64::MAX, 0 <= left <= u64::MAX,
                0 <= right <= u64::MAX, 0 <= carry <= u64::MAX;
    }
    let total = addend as u128 + left as u128 * right as u128 + carry as u128;
    proof! {
        assert(total == (total as u64) as u128
            + ((total >> 64) as u64) as u128 * 0x1_0000_0000_0000_0000u128) by (bit_vector);
    }
    (total as u64, (total >> 64) as u64)
}

/// Multiply two little-endian limb arrays into a full, zero-padded unsigned product.
///
/// Requires O >= L+R, interpreted as mathematical lengths. Each input limb may
/// be any u64; leading zeros and empty operands are permitted. The complete
/// product fits L+R limbs, and every output limb at index >= L+R stays zero.
/// The result is exact even when it exceeds a native u128. No truncation,
/// normalization, sign handling or arbitrary-width allocation is performed.
///
/// The source initializes O limbs, performs L*R multiply-accumulate steps and
/// writes one final carry per left row. The next carry position is still zero,
/// so it is assigned rather than added. With R=0, rows write zero carries; with
/// L=0, the initialized zero output returns without entering either loop.
///
/// The capacity precondition is not checked up front. Undersized output can panic
/// on indexing even when the mathematical product is zero; an empty-left invalid
/// shape can instead bypass the accesses. Such cases are outside the contract,
/// not typed failures or supported truncating multiplication. Large compile-time
/// arrays still require output/stack storage; there is no local resource cap.
/// Conditional proof invariants relate each row/digit prefix and carry to the
/// same exact product, without replacing the executable loops.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires L as int + R as int <= O,
    ensures value(result@) == value(left@) * value(right@),
        forall|index: int| L + R <= index < O ==> #[trigger] result[index] == 0,
))]
#[inline]
pub(crate) fn multiply<const L: usize, const R: usize, const O: usize>(
    left: &[u64; L],
    right: &[u64; R],
) -> [u64; O] {
    let mut product = [0_u64; O];
    let mut left_index = 0;
    proof! {
        prefix_zero(product@, O as int);
        assert(0nat * value(right@) == 0nat) by (nonlinear_arith);
    }
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant
            left_index <= L, L as int + R as int <= O,
            value(product@) == prefix(left@, left_index as int) * value(right@),
            forall|higher: int| left_index + R <= higher < O
                ==> #[trigger] product[higher] == 0,
        decreases L - left_index,
    ))]
    while left_index < L {
        let mut carry = 0_u64;
        let mut right_index = 0;
        proof! {
            assert(0nat * weight(left_index as nat) == 0nat) by (nonlinear_arith);
            assert(left[left_index as int] * 0nat * weight(left_index as nat) == 0nat)
                by (nonlinear_arith);
        }
        #[cfg_attr(verus_keep_ghost, verus_spec(
            invariant
                left_index < L, right_index <= R, L as int + R as int <= O,
                value(product@) + carry as nat * weight((left_index + right_index) as nat)
                    == prefix(left@, left_index as int) * value(right@)
                        + left[left_index as int] * prefix(right@, right_index as int) * weight(left_index as nat),
                forall|higher: int| left_index + R <= higher < O
                    ==> #[trigger] product[higher] == 0,
            decreases R - right_index,
        ))]
        while right_index < R {
            let index = left_index + right_index;
            proof_decl! {
                let ghost before = product@;
                let ghost incoming = carry;
            }
            let (digit, next_carry) =
                multiply_accumulate(product[index], left[left_index], right[right_index], carry);
            product[index] = digit;
            proof! {
                prefix_update(before, index as int, digit, O as int);
                weight_step(index as nat);
                weight_add(left_index as nat, right_index as nat);
                let w = weight(index as nat);
                let l = left[left_index as int] as nat;
                let r = right[right_index as int] as nat;
                assert(digit * w + next_carry * weight(index as nat + 1)
                    == before[index as int] * w + l * r * w + incoming * w)
                    by (nonlinear_arith)
                    requires
                        digit as nat + next_carry as nat * radix()
                            == before[index as int] + l * r + incoming,
                        weight(index as nat + 1) == w * radix();
                assert(l * prefix(right@, right_index as int + 1) * weight(left_index as nat)
                    == l * prefix(right@, right_index as int) * weight(left_index as nat) + l * r * w)
                    by (nonlinear_arith)
                    requires
                        prefix(right@, right_index as int + 1)
                            == prefix(right@, right_index as int) + r * weight(right_index as nat),
                        w == weight(left_index as nat) * weight(right_index as nat);
            }
            carry = next_carry;
            right_index += 1;
        }
        let index = left_index + R;
        proof_decl! { let ghost before = product@; }
        product[index] = carry;
        proof! {
            prefix_update(before, index as int, carry, O as int);
            assert(value(product@) == prefix(left@, left_index as int + 1) * value(right@))
                by (nonlinear_arith)
                requires before[index as int] == 0,
                    value(product@) + before[index as int] * weight(index as nat)
                        == value(before) + carry * weight(index as nat),
                    value(before) + carry * weight(index as nat)
                        == prefix(left@, left_index as int) * value(right@)
                            + left[left_index as int] * value(right@) * weight(left_index as nat),
                    prefix(left@, left_index as int + 1)
                        == prefix(left@, left_index as int) + left[left_index as int] * weight(left_index as nat);
        }
        left_index += 1;
    }
    product
}

/// Split any u128 into its low and high u64 limbs, preserving the exact magnitude.
///
/// Returns [word mod 2^64, floor(word/2^64)], including zero and a zero high limb.
/// This fixed numerical decomposition is independent of host byte order and
/// requires neither allocation nor a range check. Product and wide-scalar
/// accumulation callers consume the same little-endian two-limb representation.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures value(result@) == word,
))]
#[inline]
pub(crate) fn split_u128(word: u128) -> [u64; 2] {
    let limbs = [word as u64, (word >> 64) as u64];
    proof! {
        super::limbs::two_limbs_value(limbs@);
        assert(word == (word as u64) as u128 | ((((word >> 64) as u64) as u128) << 64)) by (bit_vector);
    }
    limbs
}

/// Return the complete unsigned product of two u128 words as four little-endian limbs.
///
/// Splits both operands into two limbs and invokes the generic kernel with exactly
/// four output limbs, satisfying its capacity precondition for every input pair.
/// Zero and products exceeding u128::MAX succeed exactly; there is no overflow
/// Option, saturation, sign interpretation or heap allocation in this adapter.
/// The result is reused by dyadic quotient comparisons and shifted accumulators.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures value(result@) == left as nat * right as nat,
))]
#[inline]
pub(crate) fn multiply_u128(left: u128, right: u128) -> [u64; 4] {
    multiply(&split_u128(left), &split_u128(right))
}
