impl Rational {
    const TOOM8_MULTIPLICATION_THRESHOLD_BITS: u64 = 262_144;

    fn should_use_toom8_multiplication(left: &BigUint, right: &BigUint) -> bool {
        let shorter = left.bits().min(right.bits());
        let longer = left.bits().max(right.bits());
        shorter >= Self::TOOM8_MULTIPLICATION_THRESHOLD_BITS
            && longer <= shorter.saturating_add(shorter / 8)
    }

    const TOOM8_INTERPOLATION: [([i64; 13], i64); 13] = [
        ([360_360, -270_270, -135_135, 75_075, 50_050, -20_020, -15_015, 4095, 3276, -546, -455, 35, 30], 360_360),
        ([1_425_600, 1_425_600, -222_750, -222_750, 44_000, 44_000, -7425, -7425, 864, 864, -50, -50, 0], 1_663_200),
        ([-12_658_536, 5_217_102, 9_825_651, -4_864_695, -3_958_130, 1_477_652, 1_217_964, -315_972, -268_524, 43_026, 37_483, -2791, -2478], 19_958_400),
        ([-4_585_248, -4_585_248, 1_809_945, 1_809_945, -397_520, -397_520, 69_444, 69_444, -8208, -8208, 479, 479, 0], 10_886_400),
        ([3_273_936, -162_828, -2_856_717, 782_645, 1_463_380, -426_344, -483_796, 106_692, 109_848, -15_572, -15_553, 1049, 1036], 21_772_800),
        ([2_992_320, 2_992_320, -1_523_385, -1_523_385, 481_760, 481_760, -93_750, -93_750, 11_616, 11_616, -695, -695, 0], 43_545_600),
        ([-743_412, -190_521, 639_207, -16_585, -358_265, 46_954, 132_218, -19_014, -31_638, 3337, 4601, -247, -311], 43_545_600),
        ([-69_912, -69_912, 39_825, 39_825, -15_100, -15_100, 3606, 3606, -492, -492, 31, 31, 0], 14_515_200),
        ([14_172, 6849, -11_619, -2395, 6645, 362, -2602, 54, 666, -29, -101, 3, 7], 14_515_200),
        ([6480, 6480, -3915, -3915, 1640, 1640, -450, -450, 72, 72, -5, -5, 0], 43_545_600),
        ([-12_804, -8217, 9999, 4015, -5665, -1342, 2266, 282, -606, -31, 97, 1, -7], 479_001_600),
        ([-792, -792, 495, 495, -220, -220, 66, 66, -12, -12, 1, 1, 0], 479_001_600),
        ([1716, 1287, -1287, -715, 715, 286, -286, -78, 78, 13, -13, -1, 1], 6_227_020_800),
    ];

    fn toom8_chunks(value: &BigUint, chunk_bits: usize) -> [BigInt; 8] {
        let mask = (BigUint::one() << chunk_bits) - 1_u8;
        std::array::from_fn(|index| {
            BigInt::from((value >> index.saturating_mul(chunk_bits)) & &mask)
        })
    }

    fn toom8_evaluate(chunks: &[BigInt; 8], point: i64) -> BigInt {
        chunks
            .iter()
            .rev()
            .fold(BigInt::ZERO, |value, chunk| value * point + chunk)
    }

    fn toom8_exact_linear_combination(
        values: &[BigInt; 13],
        coefficients: [i64; 13],
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
        debug_assert!(remainder.is_zero(), "Toom-8 interpolation must be exact");
        value / divisor
    }

    /// Fifteen-product, Rust-native Toom-8 benchmark candidate.
    #[doc(hidden)]
    pub fn multiply_magnitudes_toom8_candidate(left: &BigUint, right: &BigUint) -> BigUint {
        #[cfg(feature = "dispatch-trace")]
        crate::trace_dispatch!(
            "rational_algorithm",
            "multiplication-candidate",
            "rust-native-toom8"
        );
        Self::multiply_magnitudes_toom8(left, right)
    }

    fn multiply_magnitudes_toom8(left: &BigUint, right: &BigUint) -> BigUint {
        if left.is_zero() || right.is_zero() {
            return BigUint::ZERO;
        }

        let max_bits = left.bits().max(right.bits());
        let chunk_bits = usize::try_from(max_bits.div_ceil(8))
            .expect("BigUint bit width fits usize")
            .max(1);
        let left_chunks = Self::toom8_chunks(left, chunk_bits);
        let right_chunks = Self::toom8_chunks(right, chunk_bits);
        let coefficient_zero = &left_chunks[0] * &right_chunks[0];
        let coefficient_fourteen = &left_chunks[7] * &right_chunks[7];
        let points = [1_i64, -1, 2, -2, 3, -3, 4, -4, 5, -5, 6, -6, 7];
        let evaluated: [BigInt; 13] = points.map(|point| {
            Self::toom8_evaluate(&left_chunks, point)
                * Self::toom8_evaluate(&right_chunks, point)
                - &coefficient_zero
                - &coefficient_fourteen * BigInt::from(point.pow(14_u32))
        });
        let middle: [BigInt; 13] = Self::TOOM8_INTERPOLATION.map(|(coefficients, divisor)| {
            Self::toom8_exact_linear_combination(&evaluated, coefficients, divisor)
        });

        let mut product = coefficient_zero.magnitude().clone();
        for (index, coefficient) in middle.into_iter().enumerate() {
            assert!(
                coefficient.sign() != Minus,
                "Toom-8 coefficient must be nonnegative"
            );
            product += coefficient.magnitude() << (index + 1).saturating_mul(chunk_bits);
        }
        product + (coefficient_fourteen.magnitude() << 14_usize.saturating_mul(chunk_bits))
    }
}
