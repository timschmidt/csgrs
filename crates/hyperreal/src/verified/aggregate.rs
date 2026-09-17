//! Signed dyadic product sums with checked scale planning and fixed buffers.

#[cfg(verus_keep_ghost)]
use super::{
    division::gcd,
    dyadic::signed,
    limbs::{prefix_bound, prefix_zero, value, weight},
};
#[cfg(verus_keep_ghost)]
use vstd::{arithmetic::power2::*, math::abs, prelude::*};

#[cfg_attr(verus_keep_ghost, verus_verify)]
#[derive(Clone, Copy)]
pub(crate) enum Magnitude {
    Word(u128),
    Wide([u64; 4]),
}

#[cfg_attr(verus_keep_ghost, verus_verify)]
#[derive(Clone, Copy)]
pub(crate) struct Product {
    pub active: bool,
    pub negative: bool,
    pub left: Magnitude,
    pub right: u128,
    pub left_shift: u64,
    pub right_shift: u64,
}

#[cfg(verus_keep_ghost)]
verus! {
pub(crate) open spec fn magnitude(product: Product) -> nat {
    (match product.left {
        Magnitude::Word(word) => word as nat,
        Magnitude::Wide(words) => value(words@),
    }) * product.right as nat
}

pub(crate) open spec fn exponent(product: Product) -> nat {
    product.left_shift as nat + product.right_shift as nat
}

pub(crate) open spec fn common_scale(products: Seq<Product>, length: int) -> nat
    recommends 0 <= length <= products.len(),
    decreases length,
{
    if length <= 0 { 0 } else {
        let previous = common_scale(products, length - 1);
        let current = exponent(products[length - 1]);
        if previous > current { previous } else { current }
    }
}

pub(crate) open spec fn aligned(product: Product, scale: nat) -> nat {
    magnitude(product) * pow2((scale - exponent(product)) as nat)
}

pub(crate) open spec fn subtotal(products: Seq<Product>, scale: nat, negative: bool, length: int) -> nat
    recommends 0 <= length <= products.len(),
    decreases length,
{
    if length <= 0 { 0 } else {
        subtotal(products, scale, negative, length - 1)
            + if products[length - 1].active && products[length - 1].negative == negative {
                aligned(products[length - 1], scale)
            } else { 0 }
    }
}

pub(crate) open spec fn signed_sum(products: Seq<Product>, scale: nat, length: int) -> int
    recommends 0 <= length <= products.len(),
    decreases length,
{
    if length <= 0 { 0 } else {
        signed_sum(products, scale, length - 1)
            + if products[length - 1].active {
                signed(products[length - 1].negative, aligned(products[length - 1], scale))
            } else { 0 }
    }
}

pub(crate) proof fn scale_bounds(products: Seq<Product>, length: int)
    requires 0 <= length <= products.len(),
    ensures forall|index: int| 0 <= index < length
        ==> exponent(#[trigger] products[index]) <= common_scale(products, length),
    decreases length,
{
    if length > 0 {
        scale_bounds(products, length - 1);
        assert forall|index: int| 0 <= index < length implies
            exponent(#[trigger] products[index]) <= common_scale(products, length) by {
            if index < length - 1 {
                assert(exponent(products[index]) <= common_scale(products, length - 1));
            }
        }
    }
}

pub(crate) proof fn subtotal_monotone(products: Seq<Product>, scale: nat, negative: bool, lower: int, upper: int)
    requires 0 <= lower <= upper <= products.len(),
    ensures subtotal(products, scale, negative, lower) <= subtotal(products, scale, negative, upper),
    decreases upper - lower,
{
    if upper > lower { subtotal_monotone(products, scale, negative, lower, upper - 1); }
}

pub(crate) proof fn subtotals_denote_sum(products: Seq<Product>, scale: nat, length: int)
    requires 0 <= length <= products.len(),
    ensures signed_sum(products, scale, length)
        == subtotal(products, scale, false, length) as int - subtotal(products, scale, true, length) as int,
    decreases length,
{
    if length > 0 { subtotals_denote_sum(products, scale, length - 1); }
}

/// Each aligned numerator denotes the original product over its binary denominator.
pub(crate) proof fn alignment_preserves_fraction(product: Product, scale: nat)
    requires exponent(product) <= scale,
    ensures pow2(exponent(product)) == pow2(product.left_shift as nat) * pow2(product.right_shift as nat),
        signed(product.negative, aligned(product, scale)) * pow2(exponent(product))
        == signed(product.negative, magnitude(product)) * pow2(scale),
{
    lemma_pow2_adds(product.left_shift as nat, product.right_shift as nat);
    lemma_pow2_adds(exponent(product), (scale - exponent(product)) as nat);
    assert(signed(product.negative, aligned(product, scale)) * pow2(exponent(product))
        == signed(product.negative, magnitude(product)) * pow2(scale)) by (nonlinear_arith)
        requires pow2(scale) == pow2(exponent(product)) * pow2((scale - exponent(product)) as nat);
}

/// Raising the common denominator rescales the complete numerator exactly.
pub(crate) proof fn rescale_sum(products: Seq<Product>, lower: nat, upper: nat, length: int)
    requires 0 <= length <= products.len(), common_scale(products, length) <= lower <= upper,
    ensures signed_sum(products, upper, length)
        == signed_sum(products, lower, length) * pow2((upper - lower) as nat),
    decreases length,
{
    if length > 0 {
        rescale_sum(products, lower, upper, length - 1);
        let product = products[length - 1];
        let lower_factor = pow2((lower - exponent(product)) as nat);
        let factor = pow2((upper - lower) as nat);
        let upper_factor = pow2((upper - exponent(product)) as nat);
        lemma_pow2_adds((lower - exponent(product)) as nat, (upper - lower) as nat);
        assert(aligned(product, upper) == aligned(product, lower) * factor) by (nonlinear_arith)
            requires upper_factor == lower_factor * factor,
                aligned(product, upper) == magnitude(product) * upper_factor,
                aligned(product, lower) == magnitude(product) * lower_factor;
        let lower_term = if product.active { signed(product.negative, aligned(product, lower)) } else { 0int };
        let upper_term = if product.active { signed(product.negative, aligned(product, upper)) } else { 0int };
        assert(upper_term == lower_term * factor) by (nonlinear_arith)
            requires aligned(product, upper) == aligned(product, lower) * factor,
                upper_term == (if product.active { signed(product.negative, aligned(product, upper)) } else { 0int }),
                lower_term == (if product.active { signed(product.negative, aligned(product, lower)) } else { 0int });
        assert(signed_sum(products, upper, length)
            == signed_sum(products, lower, length) * factor) by (nonlinear_arith)
            requires signed_sum(products, upper, length - 1)
                == signed_sum(products, lower, length - 1) * factor,
                signed_sum(products, upper, length) == signed_sum(products, upper, length - 1) + upper_term,
                signed_sum(products, lower, length) == signed_sum(products, lower, length - 1) + lower_term,
                upper_term == lower_term * factor;
    } else {
        assert(0int * pow2((upper - lower) as nat) == 0int) by (nonlinear_arith);
    }
}
}

/// The common scale includes inactive terms, matching the carrier's planning
/// rule. Exponents are checked before any product or accumulator is evaluated.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures result.is_none() <==> common_scale(products@, N as int) > u64::MAX,
        match result {
            Some((shifts, maximum)) => maximum == common_scale(products@, N as int)
                && forall|index: int| 0 <= index < N
                    ==> #[trigger] shifts[index] == exponent(products[index]) && shifts[index] <= maximum,
            None => true,
        },
))]
#[inline]
pub(crate) fn plan<const N: usize>(products: &[Product; N]) -> Option<([u64; N], u64)> {
    let mut shifts = [0_u64; N];
    let mut maximum = 0_u64;
    let mut index = 0;
    proof! { scale_bounds(products@, N as int); }
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant index <= N, maximum == common_scale(products@, index as int),
            forall|done: int| 0 <= done < index ==> #[trigger] shifts[done] == exponent(products[done]),
            forall|term: int| 0 <= term < N
                ==> exponent(#[trigger] products[term]) <= common_scale(products@, N as int),
        decreases N - index,
    ))]
    while index < N {
        let shift = products[index]
            .left_shift
            .checked_add(products[index].right_shift)?;
        shifts[index] = shift;
        maximum = maximum.max(shift);
        index += 1;
    }
    Some((shifts, maximum))
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires M <= u32::MAX / 64,
    ensures result.is_none() <==>
            value(old(accumulator)@) + magnitude(product) * pow2(shift as nat) >= weight(M as nat),
        result.is_some() ==> value(final(accumulator)@)
            == value(old(accumulator)@) + magnitude(product) * pow2(shift as nat),
        magnitude(product) * pow2(shift as nat) >= weight(M as nat)
            ==> final(accumulator)@ == old(accumulator)@,
        magnitude(product) * pow2(shift as nat) < weight(M as nat)
            ==> value(final(accumulator)@)
                == (value(old(accumulator)@) + magnitude(product) * pow2(shift as nat)) % weight(M as nat),
))]
#[inline]
fn add_product<const M: usize>(
    accumulator: &mut [u64; M],
    product: Product,
    shift: u64,
) -> Option<()> {
    match product.left {
        Magnitude::Word(left) => {
            super::accumulate::add_product(accumulator, left, product.right, shift)
        }
        Magnitude::Wide(left) => {
            super::accumulate::add_wide_product(accumulator, &left, product.right, shift)
        }
    }
}

/// Evaluate the complete signed product sum, including planning and reduction.
/// Failure is exactly exponent overflow or overflow of either unsigned subtotal;
/// cancellation happens only after both subtotals have fit the fixed buffers.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires M <= u32::MAX / 64,
    ensures ({
        let scale = common_scale(products@, N as int);
        let total = signed_sum(products@, scale, N as int);
        &&& (result.is_none() <==> scale > u64::MAX
            || subtotal(products@, scale, false, N as int) >= weight(M as nat)
            || subtotal(products@, scale, true, N as int) >= weight(M as nat))
        &&& match result {
            Some((negative, words, shift)) => shift <= scale && negative == (total < 0)
                && (value(words@) == 0 <==> total == 0)
                && (value(words@) == 0 ==> shift == 0)
                && (shift == 0 || value(words@) % 2 == 1)
                && gcd(value(words@), pow2(shift as nat)) == 1
                && value(words@) * pow2((scale - shift) as nat) == abs(total)
                && signed(negative, value(words@)) * pow2(scale) == total * pow2(shift as nat),
            None => true,
        }
    }),
))]
#[inline]
pub(crate) fn sum_products<const N: usize, const M: usize>(
    products: &[Product; N],
) -> Option<(bool, [u64; M], u64)> {
    let (shifts, maximum) = plan(products)?;
    let mut positive = [0_u64; M];
    let mut negative = [0_u64; M];
    let mut index = 0;
    proof! {
        prefix_zero(positive@, M as int);
        prefix_zero(negative@, M as int);
    }
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant index <= N, M <= u32::MAX / 64,
            maximum == common_scale(products@, N as int),
            forall|term: int| 0 <= term < N
                ==> #[trigger] shifts[term] == exponent(products[term]) && shifts[term] <= maximum,
            value(positive@) == subtotal(products@, maximum as nat, false, index as int),
            value(negative@) == subtotal(products@, maximum as nat, true, index as int),
        decreases N - index,
    ))]
    while index < N {
        let product = products[index];
        if product.active {
            let shift = maximum - shifts[index];
            proof! { subtotal_monotone(products@, maximum as nat, product.negative, index as int + 1, N as int); }
            if product.negative {
                add_product(&mut negative, product, shift)?;
            } else {
                add_product(&mut positive, product, shift)?;
            }
        }
        index += 1;
    }
    proof! {
        subtotals_denote_sum(products@, maximum as nat, N as int);
        prefix_bound(positive@, M as int);
        prefix_bound(negative@, M as int);
    }
    match super::dyadic::finish(positive, negative, maximum) {
        Some(result) => Some(result),
        None => {
            let zero = [0_u64; M];
            proof! {
                prefix_zero(zero@, M as int);
                lemma2_to64();
                super::dyadic::canonical_is_reduced(0, 0);
                assert(0nat * pow2(maximum as nat) == 0nat) by (nonlinear_arith);
            }
            Some((false, zero, 0))
        }
    }
}
