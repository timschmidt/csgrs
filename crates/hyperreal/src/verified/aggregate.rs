//! Exact signed sums of dyadic products using native words or fixed limb buffers.
//!
//! A descriptor retains two unsigned factors, a sign, activity and two binary
//! denominator exponents. Planning includes inactive terms; accumulation skips
//! them. Exponent overflow therefore declines even for an inactive or zero term.
//! The rational layer constructs these descriptors from its signs and polarities
//! and decides which wider exact path to try when this layer returns None.
//!
//! The two-term native path requires each active factor tag, product and alignment
//! to pass its gates before signed addition. The limb path accepts both magnitude
//! tags but requires separate unsigned positive and negative subtotals to fit
//! before cancellation. A small final value alone guarantees neither path succeeds.
//! Successful results retain exact reduced dyadic values, including canonical zero.
//!
//! No heap-backed denominator is constructed. Planning stores N exponents; the
//! limb path owns two M-limb subtotals plus fixed product temporaries and reduction
//! scratch. Work depends on term count, occupied limbs and carries. There is no
//! local total-work, stack or cancellation budget; None reports the documented
//! fixed-representation decline, not exhaustion or mathematical undecidability.
//! Ghost relations state the arithmetic and are erased from ordinary builds.

#[cfg(verus_keep_ghost)]
use super::{
    division::gcd,
    dyadic::signed,
    limbs::{prefix_bound, prefix_zero, value, weight},
};
#[cfg(verus_keep_ghost)]
use vstd::{arithmetic::power2::*, math::abs, prelude::*};

/// Storage choice for the unsigned left factor of a dyadic product.
///
/// Word stores up to 128 bits; Wide stores up to 256 bits in numerical little-endian
/// limb order, independently of host byte order. Both permit zero and high zero
/// padding. The tag controls dispatch: an active Wide term is ineligible for the
/// native sum even when its value fits u128. No automatic narrowing occurs.
/// Derived Clone and Copy preserve the tag and digits without normalization.
#[cfg_attr(verus_keep_ghost, verus_verify)]
#[derive(Clone, Copy)]
pub(crate) enum Magnitude {
    /// A native unsigned factor, eligible for the checked two-term native path.
    Word(/** The complete factor value, including zero; no sign or denominator is encoded. */ u128),
    /// A four-limb factor consumed by the fixed-buffer path, retaining its storage tag.
    Wide(/** Base-2^64 magnitude, least significant limb first; all four digits are retained. */ [u64; 4]),
}

/// Unreduced signed product descriptor shared by both accumulation schedules.
///
/// When active, denotes the signed product of left and right divided by
/// `2^(left_shift + right_shift)`; otherwise contributes zero to the sum. The
/// exponents are still planned for inactive terms. Raw fields need not encode a
/// canonical zero: active zero factors and negative zero metadata are permitted.
/// Construction, Clone and Copy preserve all fields without validating exponent
/// addition, reducing a fraction or changing a Wide tag to Word. Result-producing
/// sum functions establish their own canonical output.
#[cfg_attr(verus_keep_ghost, verus_verify)]
#[derive(Clone, Copy)]
pub(crate) struct Product {
    /// Whether this term contributes arithmetic value; false still retains both exponents for planning.
    pub active: bool,
    /// Sign of an active product; ignored for inactive terms and cleared on zero output.
    pub negative: bool,
    /// Unsigned left factor and its dispatch tag, retained even for inactive terms.
    pub left: Magnitude,
    /// Unsigned u128 right factor; zero is permitted without making activity canonical.
    pub right: u128,
    /// Left factor's binary denominator exponent, added with overflow checking during planning.
    pub left_shift: u64,
    /// Right factor's binary denominator exponent; participates even when this term is inactive.
    pub right_shift: u64,
}

#[cfg(verus_keep_ghost)]
verus! {
/// Denote the raw unsigned product independently of sign, activity and exponents.
///
/// Decodes either left storage tag and multiplies by right in mathematical natural
/// numbers, so the specification has no native overflow. Inactivity is applied
/// by subtotal and signed_sum rather than by this relation.
pub(crate) open spec fn magnitude(product: Product) -> nat {
    (match product.left {
        Magnitude::Word(word) => word as nat,
        Magnitude::Wide(words) => value(words@),
    }) * product.right as nat
}

/// Add both denominator exponents in mathematical natural numbers.
///
/// The result can exceed u64::MAX. It ignores activity and magnitude; plan checks
/// whether every such sum can be stored in runtime exponent metadata.
pub(crate) open spec fn exponent(product: Product) -> nat {
    product.left_shift as nat + product.right_shift as nat
}

/// Take the maximum raw product exponent over a sequence prefix, with zero for an empty prefix.
///
/// The recommended domain is `0 <= length <= products.len()`. Every descriptor
/// participates, including inactive and zero terms. Recursion removes the last
/// term until length is nonpositive; the recommended bounds are not a runtime
/// validation interface. The result may exceed the runtime u64 exponent range.
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

/// Denote the unsigned numerator of a product at a chosen binary denominator scale.
///
/// Its fraction interpretation requires `scale >= exponent(product)`: multiply
/// the raw magnitude by `2^(scale - exponent(product))`. The definition itself
/// has no such precondition, so callers must establish it before claiming that
/// the original fraction is preserved. Sign and activity are not applied here.
pub(crate) open spec fn aligned(product: Product, scale: nat) -> nat {
    magnitude(product) * pow2((scale - exponent(product)) as nat)
}

/// Describe native_product eligibility, including tag and intermediate-width gates.
///
/// Inactive descriptors satisfy this relation unconditionally. An active term
/// requires a Word tag, an exponent no greater than scale, displacement below
/// 128, and both raw and aligned magnitudes at most u128::MAX. Active Wide zero
/// and active Word zero displaced by 128 or more therefore fail. This describes
/// a specific native schedule, not mathematical representability of the final sum.
pub(crate) open spec fn native_product_fits(product: Product, scale: nat) -> bool {
    !product.active || (
        (match product.left { Magnitude::Word(_) => true, Magnitude::Wide(_) => false })
        && exponent(product) <= scale && scale - exponent(product) < 128
        && magnitude(product) <= u128::MAX && aligned(product, scale) <= u128::MAX
    )
}

/// Sum aligned unsigned numerators of active terms with the selected sign in a prefix.
///
/// The recommended prefix bounds are `0 <= length <= products.len()`; an empty
/// prefix contributes zero. Terms of the other sign and inactive terms contribute
/// nothing. A common scale at least every included exponent gives these numerators
/// the intended fraction meaning. The natural-number sum has no runtime limb cap;
/// its comparison with a buffer weight defines admission before cancellation.
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

/// Sum signed aligned numerators of active terms over a prefix.
///
/// Uses the recommended bounds `0 <= length <= products.len()` and zero for an
/// empty prefix. Inactive terms contribute zero regardless of their raw factors.
/// For a scale no smaller than every included exponent, dividing by `2^scale`
/// yields the intended dyadic sum. This mathematical integer can exceed any
/// native or limb-buffer capacity and can include exact cancellation.
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

/// Prove that every exponent in a valid prefix is bounded by its common scale.
///
/// Induction follows the maximum recurrence, including inactive descriptors.
/// This supplies nonnegative runtime alignment differences after successful
/// planning; it does not prove that the common scale fits u64.
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

/// Show that extending a valid prefix cannot decrease either unsigned subtotal.
///
/// Requires `0 <= lower <= upper <= products.len()`. Each added selected term is
/// a nonnegative aligned magnitude, regardless of whether scale has the intended
/// fraction interpretation. The result lets an overflowing intermediate subtotal
/// witness overflow of the complete subtotal; it does not bound the signed sum.
pub(crate) proof fn subtotal_monotone(products: Seq<Product>, scale: nat, negative: bool, lower: int, upper: int)
    requires 0 <= lower <= upper <= products.len(),
    ensures subtotal(products, scale, negative, lower) <= subtotal(products, scale, negative, upper),
    decreases upper - lower,
{
    if upper > lower { subtotal_monotone(products, scale, negative, lower, upper - 1); }
}

/// Identify a valid prefix's signed sum with positive subtotal minus negative subtotal.
///
/// Induction partitions active terms by their sign and ignores inactive terms.
/// The identity holds for the same aligned numerators on both sides and gives
/// the final subtraction its signed-sum meaning. It does not establish that either
/// unsigned subtotal fits a runtime buffer before cancellation.
pub(crate) proof fn subtotals_denote_sum(products: Seq<Product>, scale: nat, length: int)
    requires 0 <= length <= products.len(),
    ensures signed_sum(products, scale, length)
        == subtotal(products, scale, false, length) as int - subtotal(products, scale, true, length) as int,
    decreases length,
{
    if length > 0 { subtotals_denote_sum(products, scale, length - 1); }
}

/// Prove that numerator alignment preserves a product's dyadic fraction.
///
/// Requires the common scale to be no smaller than the sum of both exponents.
/// Power-of-two factorization connects the two factor denominators to the product
/// denominator, then cross multiplication relates the original and aligned signed
/// numerators. This statement concerns the raw product even if inactive; callers
/// apply the activity convention separately. It establishes no native-width bound.
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

/// Prove that raising a common denominator rescales the entire signed numerator.
///
/// Requires a valid prefix and `common_scale <= lower <= upper`. The upper-scale
/// sum equals the lower-scale sum times `2^(upper - lower)`, including inactive
/// terms and cancellation. Induction uses the per-term power identity. This ghost
/// relation imposes no u64/limb limits and allocates no runtime shifted integer.
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

/// Check every product exponent and return all sums together with their maximum.
///
/// Adds left_shift and right_shift with checked u64 arithmetic in input order.
/// Returns None on the first overflowing sum, including for inactive or zero
/// terms. Otherwise returns the N stored exponents and their maximum; an empty
/// array returns an empty exponent array and maximum zero.
///
/// All raw descriptor values and tags are accepted. No factors are multiplied,
/// no terms are aligned, and no caller data is mutated. Planning uses an N-element
/// local array and at most N checked additions, with no local stack/work budget.
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

/// Dispatch one raw magnitude product into a caller-owned fixed accumulator.
///
/// Uses the Word or Wide multiplication path and the supplied nonnegative binary
/// shift. It ignores the descriptor's activity, sign and stored denominator
/// exponents; callers must select the target subtotal and alignment first.
/// Requires `M <= u32::MAX / 64`, without runtime dimension validation.
///
/// For original buffer S, shifted magnitude A and `B = 2^(64*M)`, succeeds exactly
/// when `S+A < B`. If `A >= B`, returns None before mutation; otherwise stores
/// `(S+A) modulo B`, including on late overflow. The aggregate caller owns these
/// scratch buffers and discards them on decline. Zero succeeds unchanged for all
/// u64 shifts, including M=0. There is no fallback or policy decision here.
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

/// Evaluate a signed dyadic product sum with separate fixed-width unsigned subtotals.
///
/// Requires `M <= u32::MAX / 64`, without runtime dimension validation. First
/// plans all N exponents, including inactive terms. Then visits active terms in
/// input order, aligning them to the maximum exponent and accumulating by sign.
/// Returns None exactly on exponent-sum overflow or when either complete unsigned
/// subtotal is at least `2^(64*M)`. Opposite signs cancel only after both subtotals
/// fit; they cannot rescue an overflowing intermediate positive or negative total.
///
/// Success returns `(negative, magnitude_limbs, denominator_shift)`. The limbs
/// are an unsigned little-endian magnitude; the value is its signed numerator
/// over `2^denominator_shift`. Common powers of two are removed, so a nonzero
/// result is an integer or has an odd numerator. Zero returns `(false, [0; M], 0)`,
/// including empty input or equal subtotals. M=0 admits only zero subtotals after
/// successful planning, not cancellation of nonzero terms.
///
/// Input descriptors remain unchanged. Partial overflow state is local scratch
/// and is discarded. Large exponents remain metadata, so canonical reduction does
/// not materialize an enormous denominator. None is fixed-tier decline, not a
/// zero answer, an arbitrary-width impossibility or a reached resource budget.
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

/// Attempt one active product and its alignment entirely within u128.
///
/// Requires the mathematical sum of stored exponents to be no greater than maximum;
/// plan establishes this for the caller. Inactive terms immediately return
/// `Some((false, 0))`. Active Wide terms return None before multiplication even
/// when their magnitude is zero. Active Word terms must have a u128 product and
/// a displacement below 128 whose scaled value also fits u128. The shift gate
/// rejects zero at displacement 128 or greater.
///
/// Success returns sign and aligned magnitude, clearing the sign for zero. This
/// helper neither reduces the denominator nor falls back to limbs. Its scale
/// precondition is not runtime-validated; violating it can underflow subtraction.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires exponent(product) <= maximum,
    ensures result.is_none() <==> !native_product_fits(product, maximum as nat),
        match result {
            Some((negative, magnitude)) => (magnitude == 0 ==> !negative)
                && super::word::signed(negative, magnitude)
                    == (if product.active { signed(product.negative, aligned(product, maximum as nat)) } else { 0int }),
            None => true,
        },
))]
#[inline]
fn native_product(product: Product, maximum: u64) -> Option<(bool, u128)> {
    if !product.active {
        return Some((false, 0));
    }
    let left = match product.left {
        Magnitude::Word(left) => left,
        Magnitude::Wide(_) => return None,
    };
    let magnitude = left.checked_mul(product.right)?;
    let shift = maximum - product.left_shift - product.right_shift;
    let scaled = super::word::checked_shift_left_u64(magnitude, shift)?;
    Some((product.negative && scaled != 0, scaled))
}

/// Attempt exactly two signed dyadic products using native arithmetic and reduction.
///
/// Plans both exponent sums, including inactive terms, then checks the first
/// native product, the second and their signed addition in that order. Returns
/// None for exponent overflow, an active Wide tag, an active displacement at least
/// 128, raw/aligned product overflow or final signed magnitude above u128::MAX.
/// The signed range is symmetric about zero and is not restricted to i128.
///
/// Opposite signs may cancel at the final addition, but both individual active
/// terms must first pass their gates. Inactive terms skip product/alignment gates
/// after planning. Success returns a canonical dyadic Word with common binary
/// factors removed; zero has inactive/positive metadata, magnitude zero and shift
/// zero. No input is mutated, no large denominator is allocated, and None leaves
/// the rational caller free to try a wider exact schedule.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures ({
        let scale = common_scale(products@, 2);
        let total = signed_sum(products@, scale, 2);
        &&& (result.is_none() <==> scale > u64::MAX
            || !native_product_fits(products[0], scale) || !native_product_fits(products[1], scale)
            || total > u128::MAX || total < -(u128::MAX as int))
        &&& match result {
            Some(word) => super::dyadic::canonical_word(word) && word.shift <= scale
                && word.negative == (total < 0)
                && word.magnitude as nat * pow2((scale - word.shift) as nat) == abs(total)
                && super::dyadic::word_value(word) * pow2(scale) == total * pow2(word.shift as nat),
            None => true,
        }
    }),
))]
#[inline]
pub(crate) fn sum_products_word(products: &[Product; 2]) -> Option<super::dyadic::Word> {
    let (_shifts, maximum) = plan(products)?;
    proof! {
        assert(_shifts[0] == exponent(products[0]));
        assert(_shifts[1] == exponent(products[1]));
        reveal_with_fuel(signed_sum, 3);
    }
    let (left_negative, left) = native_product(products[0], maximum)?;
    let (right_negative, right) = native_product(products[1], maximum)?;
    let (negative, magnitude) =
        super::word::signed_add(left_negative, left, right_negative, right)?;
    Some(super::dyadic::normalize_word(negative, magnitude, maximum))
}
