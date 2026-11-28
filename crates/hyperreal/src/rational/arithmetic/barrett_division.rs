/// Owned reciprocal data reused while dividing by one fixed integer d >= 2.
///
/// With k=bits(d) and B=2^k, preparation computes mu=floor(B^2/d) using one
/// integer division. A dividend is then processed from high to low in k-bit
/// blocks. Each combined block x<d*B has a quotient estimate no greater than
/// floor(x/d) and at most two below it, so correction uses at most two subtractions.
/// This is exact integer arithmetic throughout, with no approximate reciprocal.
///
/// The fields form one invariant set and must not be changed independently.
/// Derived Clone copies the owned BigUint divisor/reciprocal/mask and width;
/// Debug formats those stored parameters without new reciprocal preparation.
/// This private benchmark helper does not introduce a global cache or a production
/// division selector. Preparation and per-block temporaries allocate without
/// explicit memory/work admission, cancellation or typed exhaustion.
#[derive(Clone, Debug)]
struct PreparedBarrettDivisor {
    /// The fixed divisor d >= 2 copied at preparation; all remainder bounds refer to this value.
    divisor: BigUint,
    /// Exact truncated reciprocal mu=floor(2^(2*chunk_bits)/divisor), used for quotient estimates.
    reciprocal: BigUint,
    /// Divisor bit width k, at least two; chooses block base B=2^k and both estimate shifts.
    chunk_bits: usize,
    /// Low-k-bit mask B-1 used to extract each dividend digit without changing its value.
    chunk_mask: BigUint,
}

/// Prepare and consume a coherent reciprocal for exact block-wise quotient/remainder division.
///
/// Methods preserve dividend=q*d+r with 0<=r<d. The block-size and reciprocal
/// relations are supplied by new; div_rem maintains the per-block range invariant.
/// Debug assertions diagnose invariant violations but are not runtime resource
/// limits or typed validation for arbitrary manually assembled field values.
impl PreparedBarrettDivisor {
    /// Prepare a reciprocal for d >= 2, returning None for zero, one or an unrepresentable bit width.
    ///
    /// Convert bits(d) to usize, copy d, and allocate B=2^k, mask B-1 and the
    /// reciprocal floor(2^(2k)/d). None is an eligibility/conversion outcome, not proof
    /// of resource exhaustion. Saturating 2*k shift arithmetic is not a resource
    /// admission check; representable shifts and successful allocation are required.
    /// The benchmark wrappers reject zero before calling this and use the backend
    /// when preparation declines, notably for divisor one.
    fn new(divisor: &BigUint) -> Option<Self> {
        if divisor.bits() < 2 {
            return None;
        }
        let chunk_bits = usize::try_from(divisor.bits()).ok()?;
        let chunk_base = BigUint::one() << chunk_bits;
        Some(Self {
            divisor: divisor.clone(),
            reciprocal: (BigUint::one() << chunk_bits.saturating_mul(2)) / divisor,
            chunk_bits,
            chunk_mask: &chunk_base - 1_u8,
        })
    }

    /// Divide one combined block x<d*2^k using the prepared exact reciprocal.
    ///
    /// Estimate q0=floor(floor(x/2^(k-1))*mu/2^(k+1)), then form r0=x-q0*d.
    /// Both truncations round down. Since x<d*2^k<2^(2k), the reciprocal truncation
    /// loses less than one quotient unit; truncating the input prefix loses less than
    /// 2^(k-1)/d <= 1. Thus q0<=floor(x/d)<=q0+2. Subtract d and increment q0
    /// until the remainder is below d. The returned quotient fits one k-bit digit.
    ///
    /// Input range, nonnegative subtraction and the two-correction bound are checked
    /// only by debug assertions. The loop has no hard release-mode iteration limit;
    /// invalid state/input may panic or fail to satisfy that bound. Valid calls use
    /// multiplication, shifts and subtraction, without a new integer division or
    /// modular wraparound. Temporaries allocate without a resource/cancellation limit.
    fn div_rem_block(&self, value: &BigUint) -> (BigUint, BigUint) {
        debug_assert!(value < &(&self.divisor << self.chunk_bits));

        // HAC 14.42 with radix 2 and k equal to the divisor bit width.
        let quotient_prefix = value >> (self.chunk_bits - 1);
        let quotient = (&quotient_prefix * &self.reciprocal) >> (self.chunk_bits + 1);
        // Both truncations in the estimate round down, so `quotient` cannot
        // exceed the exact quotient. Subtract the full product directly;
        // unlike the textbook fixed-width form, `BigUint` does not need a
        // masked modular subtraction to avoid overflow.
        let product = &quotient * &self.divisor;
        debug_assert!(value >= &product);
        let mut remainder = value - product;
        let mut quotient = quotient;
        let mut corrections = 0_u8;
        while remainder >= self.divisor {
            remainder -= &self.divisor;
            quotient += 1_u8;
            corrections += 1;
            debug_assert!(corrections <= 2, "Barrett estimate exceeded its error bound");
        }
        (quotient, remainder)
    }

    /// Return the exact quotient and remainder of a nonnegative dividend by the prepared d.
    ///
    /// For dividend<d, return zero and a cloned dividend. Otherwise visit k-bit digits
    /// from most significant to least significant. Combine the previous remainder r<d
    /// with the next digit t<B as x=r*B+t<d*B, divide that block, and append its
    /// quotient digit. Leading zero quotient digits are naturally normalized by BigUint.
    ///
    /// The number of blocks is ceil(bits(dividend)/k), with an expect-based usize
    /// conversion. Shift indexes saturate rather than enforce a work or memory budget.
    /// The prepared divisor is borrowed and reusable; quotient/remainder and growing
    /// intermediates are materialized for each call. There is no cancellation or
    /// partial-output/typed-exhaustion contract.
    fn div_rem(&self, dividend: &BigUint) -> (BigUint, BigUint) {
        if dividend < &self.divisor {
            return (BigUint::ZERO, dividend.clone());
        }

        let block_count = usize::try_from(dividend.bits())
            .expect("BigUint bit width fits usize")
            .div_ceil(self.chunk_bits);
        let mut quotient = BigUint::ZERO;
        let mut remainder = BigUint::ZERO;
        for block in (0..block_count).rev() {
            let shift = block.saturating_mul(self.chunk_bits);
            let digit = (dividend >> shift) & &self.chunk_mask;
            let combined = (remainder << self.chunk_bits) + digit;
            let (quotient_digit, next_remainder) = self.div_rem_block(&combined);
            quotient = (quotient << self.chunk_bits) + quotient_digit;
            remainder = next_remainder;
        }
        (quotient, remainder)
    }
}

/// Benchmark interfaces comparing prepared Barrett division with the native integer backend.
///
/// These methods divide unsigned magnitudes, not signed Rational values. They
/// return exact quotient/remainder pairs but panic on zero divisors. One-shot
/// preparation is included in each call; batch preparation is shared within one
/// call only. No reciprocal is retained in Rational or in a cross-call cache.
impl Rational {
    /// Prepare one divisor and return the exact nonnegative quotient/remainder pair.
    ///
    /// Zero divisor panics before tracing. With dispatch-trace recording active, emit
    /// rational_algorithm/division-candidate/block-wise-barrett, then attempt preparation.
    /// Divisor one (or an unrepresentable preparation width) falls back to native
    /// BigUint div_rem, so the trace labels the requested wrapper rather than proving
    /// that a prepared Barrett kernel ran. For d>=2 preparation occurs even when the
    /// dividend is zero or smaller than d. No total-work/memory budget, cancellation
    /// or typed resource failure is provided; all results are fully materialized.
    pub fn div_rem_magnitudes_barrett_candidate(
        dividend: &BigUint,
        divisor: &BigUint,
    ) -> (BigUint, BigUint) {
        assert!(!divisor.is_zero(), "division by zero");
        #[cfg(feature = "dispatch-trace")]
        crate::trace_dispatch!(
            "rational_algorithm",
            "division-candidate",
            "block-wise-barrett"
        );
        if let Some(prepared) = PreparedBarrettDivisor::new(divisor) {
            prepared.div_rem(dividend)
        } else {
            use num::Integer as _;
            dividend.div_rem(divisor)
        }
    }

    /// Return input-ordered quotient/remainder pairs, preparing one reciprocal per batch call.
    ///
    /// Reject zero divisor even for an empty slice. For eligible d>=2, preparation
    /// occurs before iteration, including an empty batch, and is reused for all values.
    /// If preparation declines, delegate to the backend batch; divisor one follows
    /// that path. Inputs are borrowed and the output vector owns every result pair.
    /// Unlike the one-shot wrapper, this path emits no division-candidate trace event.
    /// There is no retained cross-call cache, total-resource admission or cancellation.
    pub fn div_rem_magnitudes_barrett_batch_candidate(
        dividends: &[BigUint],
        divisor: &BigUint,
    ) -> Vec<(BigUint, BigUint)> {
        assert!(!divisor.is_zero(), "division by zero");
        if let Some(prepared) = PreparedBarrettDivisor::new(divisor) {
            dividends
                .iter()
                .map(|dividend| prepared.div_rem(dividend))
                .collect()
        } else {
            Self::div_rem_magnitudes_backend_batch(dividends, divisor)
        }
    }

    /// Materialize native BigUint quotient/remainder pairs in input order for a fixed positive divisor.
    ///
    /// Zero divisor panics even for an empty input; otherwise empty input yields an
    /// empty vector. Each element independently calls the backend div_rem, without
    /// preparing or retaining a Barrett reciprocal or emitting the candidate event.
    /// Used as the batch comparison and preparation-decline fallback. Allocations and
    /// work have no explicit bound, cancellation or typed exhaustion result.
    pub fn div_rem_magnitudes_backend_batch(
        dividends: &[BigUint],
        divisor: &BigUint,
    ) -> Vec<(BigUint, BigUint)> {
        assert!(!divisor.is_zero(), "division by zero");
        use num::Integer as _;
        dividends
            .iter()
            .map(|dividend| dividend.div_rem(divisor))
            .collect()
    }
}
