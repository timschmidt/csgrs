impl Rational {
    const TOOM4_MULTIPLICATION_THRESHOLD_BITS: u64 = 1_048_576;

    fn should_use_toom4_multiplication(left: &BigUint, right: &BigUint) -> bool {
        let shorter = left.bits().min(right.bits());
        let longer = left.bits().max(right.bits());
        shorter >= Self::TOOM4_MULTIPLICATION_THRESHOLD_BITS
            && longer <= shorter.saturating_add(shorter / 4)
    }

    fn multiply_magnitudes(
        operation: &'static str,
        left: &BigUint,
        right: &BigUint,
    ) -> BigUint {
        if Self::should_use_toom8_multiplication(left, right) {
            #[cfg(feature = "dispatch-trace")]
            crate::trace_dispatch!("rational_algorithm", operation, "rust-native-toom8");
            Self::multiply_magnitudes_toom8(left, right)
        } else if Self::should_use_toom6_multiplication(left, right) {
            #[cfg(feature = "dispatch-trace")]
            crate::trace_dispatch!("rational_algorithm", operation, "rust-native-toom6");
            Self::multiply_magnitudes_toom6(left, right)
        } else if Self::should_use_toom4_multiplication(left, right) {
            #[cfg(feature = "dispatch-trace")]
            crate::trace_dispatch!("rational_algorithm", operation, "rust-native-toom4");
            Self::multiply_magnitudes_toom4(left, right)
        } else {
            #[cfg(feature = "dispatch-trace")]
            Self::trace_backend_multiplication(operation, left, right);
            #[cfg(not(feature = "dispatch-trace"))]
            let _ = operation;
            left * right
        }
    }

    /// Benchmark probe for the production multiplication selector.
    #[doc(hidden)]
    pub fn multiply_magnitudes_selected(left: &BigUint, right: &BigUint) -> BigUint {
        Self::multiply_magnitudes("multiplication-benchmark", left, right)
    }

    fn toom4_chunks(value: &BigUint, chunk_bits: usize) -> [BigInt; 4] {
        let mask = (BigUint::one() << chunk_bits) - 1_u8;
        std::array::from_fn(|index| {
            BigInt::from((value >> index.saturating_mul(chunk_bits)) & &mask)
        })
    }

    fn toom4_evaluate(chunks: &[BigInt; 4], point: i64) -> BigInt {
        chunks
            .iter()
            .rev()
            .fold(BigInt::ZERO, |value, chunk| value * point + chunk)
    }

    fn toom4_exact_linear_combination(
        values: &[BigInt; 5],
        coefficients: [i64; 5],
        divisor: i64,
    ) -> BigInt {
        let value = values
            .iter()
            .zip(coefficients)
            .fold(BigInt::ZERO, |sum, (value, coefficient)| {
                sum + value * coefficient
            });
        let divisor = BigInt::from(divisor);
        let remainder = &value % &divisor;
        debug_assert!(remainder.is_zero(), "Toom-4 interpolation must be exact");
        value / divisor
    }

    /// Seven-product, Rust-native Toom-4 benchmark candidate.
    ///
    /// Four balanced binary chunks are evaluated at 0, ±1, ±2, 3, and
    /// infinity. The degree-six product is reconstructed with exact `BigInt`
    /// interpolation; no GMP representation or release dependency is used.
    #[doc(hidden)]
    pub fn multiply_magnitudes_toom4_candidate(left: &BigUint, right: &BigUint) -> BigUint {
        #[cfg(feature = "dispatch-trace")]
        crate::trace_dispatch!(
            "rational_algorithm",
            "multiplication-candidate",
            "rust-native-toom4"
        );
        Self::multiply_magnitudes_toom4(left, right)
    }

    fn multiply_magnitudes_toom4(left: &BigUint, right: &BigUint) -> BigUint {
        if left.is_zero() || right.is_zero() {
            return BigUint::ZERO;
        }

        let max_bits = left.bits().max(right.bits());
        let chunk_bits = usize::try_from(max_bits.div_ceil(4))
            .expect("BigUint bit width fits usize")
            .max(1);
        let left_chunks = Self::toom4_chunks(left, chunk_bits);
        let right_chunks = Self::toom4_chunks(right, chunk_bits);

        let coefficient_zero = &left_chunks[0] * &right_chunks[0];
        let coefficient_six = &left_chunks[3] * &right_chunks[3];
        let points = [1_i64, -1, 2, -2, 3];
        let evaluated: [BigInt; 5] = points.map(|point| {
            let product = Self::toom4_evaluate(&left_chunks, point)
                * Self::toom4_evaluate(&right_chunks, point);
            product
                - &coefficient_zero
                - &coefficient_six * BigInt::from(point.pow(6_u32))
        });

        // Inverse of the five-by-five Vandermonde matrix for powers x¹..x⁵
        // at [1, -1, 2, -2, 3], arranged to keep every division exact.
        let coefficients = [
            coefficient_zero,
            Self::toom4_exact_linear_combination(&evaluated, [60, -30, -15, 3, 2], 60),
            Self::toom4_exact_linear_combination(&evaluated, [16, 16, -1, -1, 0], 24),
            Self::toom4_exact_linear_combination(&evaluated, [-14, -1, 7, -1, -1], 24),
            Self::toom4_exact_linear_combination(&evaluated, [-4, -4, 1, 1, 0], 24),
            Self::toom4_exact_linear_combination(&evaluated, [10, 5, -5, -1, 1], 120),
            coefficient_six,
        ];

        coefficients
            .into_iter()
            .enumerate()
            .fold(BigUint::ZERO, |product, (index, coefficient)| {
                assert!(
                    coefficient.sign() != Minus,
                    "Toom-4 coefficient must be nonnegative"
                );
                product
                    + (coefficient.magnitude()
                        << index.saturating_mul(chunk_bits))
            })
    }
}
