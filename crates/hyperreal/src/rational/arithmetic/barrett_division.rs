/// A reusable Rust-native Barrett reciprocal for one positive magnitude.
///
/// The representation stays in `BigUint`; GMP/Rug is used only by the separate
/// dev benchmark oracle. One preparation division computes `floor(2^(2k)/d)`.
/// Subsequent divisions consume the dividend in `k`-bit blocks and use only
/// multiplication, shifts, masking, and at most two correction subtractions per
/// block.
#[derive(Clone, Debug)]
struct PreparedBarrettDivisor {
    divisor: BigUint,
    reciprocal: BigUint,
    chunk_bits: usize,
    chunk_mask: BigUint,
}

impl PreparedBarrettDivisor {
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

impl Rational {
    /// Benchmark probe for the one-shot Rust-native block-wise Barrett route.
    #[doc(hidden)]
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

    /// Benchmark probe that amortizes one native reciprocal over many values.
    #[doc(hidden)]
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

    /// Native `num-bigint` batch baseline for the Barrett crossover benchmark.
    #[doc(hidden)]
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
