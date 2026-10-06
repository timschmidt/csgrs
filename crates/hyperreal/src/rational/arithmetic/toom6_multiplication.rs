impl Rational {
    const TOOM6_MULTIPLICATION_THRESHOLD_BITS: u64 = 524_288;

    fn should_use_toom6_multiplication(left: &BigUint, right: &BigUint) -> bool {
        let shorter = left.bits().min(right.bits());
        let longer = left.bits().max(right.bits());
        shorter >= Self::TOOM6_MULTIPLICATION_THRESHOLD_BITS
            && longer <= shorter.saturating_add(shorter / 6)
    }

    fn toom6_chunks(value: &BigUint, chunk_bits: usize) -> [BigInt; 6] {
        let mask = (BigUint::one() << chunk_bits) - 1_u8;
        std::array::from_fn(|index| {
            BigInt::from((value >> index.saturating_mul(chunk_bits)) & &mask)
        })
    }

    fn toom6_evaluate(chunks: &[BigInt; 6], point: i64) -> BigInt {
        chunks
            .iter()
            .rev()
            .fold(BigInt::ZERO, |value, chunk| value * point + chunk)
    }

    fn toom6_exact_linear_combination(
        values: &[BigInt; 9],
        coefficients: [i64; 9],
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
        debug_assert!(remainder.is_zero(), "Toom-6 interpolation must be exact");
        value / divisor
    }

    /// Eleven-product, Rust-native Toom-6 benchmark candidate.
    ///
    /// Six balanced binary chunks are evaluated at 0, ±1, ±2, ±3, ±4, 5,
    /// and infinity. Static exact interpolation reconstructs all eleven product
    /// coefficients without introducing a GMP-backed release representation.
    #[doc(hidden)]
    pub fn multiply_magnitudes_toom6_candidate(left: &BigUint, right: &BigUint) -> BigUint {
        #[cfg(feature = "dispatch-trace")]
        crate::trace_dispatch!(
            "rational_algorithm",
            "multiplication-candidate",
            "rust-native-toom6"
        );
        Self::multiply_magnitudes_toom6(left, right)
    }

    fn multiply_magnitudes_toom6(left: &BigUint, right: &BigUint) -> BigUint {
        if left.is_zero() || right.is_zero() {
            return BigUint::ZERO;
        }

        let max_bits = left.bits().max(right.bits());
        let chunk_bits = usize::try_from(max_bits.div_ceil(6))
            .expect("BigUint bit width fits usize")
            .max(1);
        let left_chunks = Self::toom6_chunks(left, chunk_bits);
        let right_chunks = Self::toom6_chunks(right, chunk_bits);
        let coefficient_zero = &left_chunks[0] * &right_chunks[0];
        let coefficient_ten = &left_chunks[5] * &right_chunks[5];
        let points = [1_i64, -1, 2, -2, 3, -3, 4, -4, 5];
        let evaluated: [BigInt; 9] = points.map(|point| {
            Self::toom6_evaluate(&left_chunks, point)
                * Self::toom6_evaluate(&right_chunks, point)
                - &coefficient_zero
                - &coefficient_ten * BigInt::from(point.pow(10_u32))
        });

        // Inverse Vandermonde rows for x¹..x⁹ at the nine finite points.
        let coefficients = [
            coefficient_zero,
            Self::toom6_exact_linear_combination(
                &evaluated,
                [2520, -1680, -840, 360, 240, -60, -45, 5, 4],
                2520,
            ),
            Self::toom6_exact_linear_combination(
                &evaluated,
                [8064, 8064, -1008, -1008, 128, 128, -9, -9, 0],
                10_080,
            ),
            Self::toom6_exact_linear_combination(
                &evaluated,
                [-56_574, 13_524, 38_514, -13_914, -11_916, 2691, 2286, -236, -205],
                90_720,
            ),
            Self::toom6_exact_linear_combination(
                &evaluated,
                [-1952, -1952, 676, 676, -96, -96, 7, 7, 0],
                5760,
            ),
            Self::toom6_exact_linear_combination(
                &evaluated,
                [2334, 396, -1716, 156, 684, -99, -141, 11, 13],
                17_280,
            ),
            Self::toom6_exact_linear_combination(
                &evaluated,
                [116, 116, -52, -52, 12, 12, -1, -1, 0],
                2880,
            ),
            Self::toom6_exact_linear_combination(
                &evaluated,
                [-714, -336, 504, 96, -216, -9, 51, -1, -5],
                60_480,
            ),
            Self::toom6_exact_linear_combination(
                &evaluated,
                [-56, -56, 28, 28, -8, -8, 1, 1, 0],
                40_320,
            ),
            Self::toom6_exact_linear_combination(
                &evaluated,
                [126, 84, -84, -36, 36, 9, -9, -1, 1],
                362_880,
            ),
            coefficient_ten,
        ];

        coefficients
            .into_iter()
            .enumerate()
            .fold(BigUint::ZERO, |product, (index, coefficient)| {
                assert!(
                    coefficient.sign() != Minus,
                    "Toom-6 coefficient must be nonnegative"
                );
                product
                    + (coefficient.magnitude()
                        << index.saturating_mul(chunk_bits))
            })
    }
}
