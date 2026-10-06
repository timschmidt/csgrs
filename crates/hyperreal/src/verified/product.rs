//! Exact schoolbook products for the fixed-width dyadic carriers.

#[cfg(verus_keep_ghost)]
use super::limbs::{
    prefix, prefix_equal, prefix_zero, radix, value, weight, weight_add, weight_step,
};
#[cfg(verus_keep_ghost)]
use vstd::prelude::*;

#[cfg(verus_keep_ghost)]
verus! {
/// Replacing one positional digit changes only that digit's contribution.
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

/// Multiply every pair of input limbs, retaining all carries in an output
/// with room for the complete product. Unused high output limbs stay zero.
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

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures value(result@) == left as nat * right as nat,
))]
#[inline]
pub(crate) fn multiply_u128(left: u128, right: u128) -> [u64; 4] {
    multiply(&split_u128(left), &split_u128(right))
}
