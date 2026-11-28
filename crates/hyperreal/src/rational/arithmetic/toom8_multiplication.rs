/// Exact Toom-8 magnitude multiplication and its selection/interpolation helpers.
///
/// Split each nonnegative integer into 8 base-B coefficients, B=2^chunk_bits.
/// Multiplying the degree-7 polynomials gives degree 14; 15 point products
/// recover it at 0, +/-1, +/-2, +/-3, +/-4, +/-5, +/-6, 7 and the leading coefficient (infinity).
/// Signed BigInt intermediates handle negative evaluation points. Recombination
/// at B materializes the exact nonnegative integer product, including carries.
/// Point products themselves use the integer backend rather than recursively
/// calling Hyperreal's magnitude selector.
///
/// These kernels support rational arithmetic and half-GCD matrix operations.
/// They do not operate on rational signs/denominators or preserve a lazy recipe.
/// There is no work/memory budget, cancellation or typed exhaustion outcome;
/// large operands allocate proportionally growing intermediates. Internal width,
/// exact-divisibility and coefficient-sign checks are described on the helpers.
impl Rational {
    /// Minimum shorter-operand bit width (262,144) for the Toom-8 selector gate.
    ///
    /// This tuning threshold works together with the balance predicate and selector
    /// priority; it is neither a mathematical validity limit nor a resource bound.
    const TOOM8_MULTIPLICATION_THRESHOLD_BITS: u64 = 262_144;

    /// Test the Toom-8 size/balance gate without multiplying or allocating chunk buffers.
    ///
    /// Require shorter >= 262,144 bits and longer <= shorter+floor(shorter/8).
    /// The upper bound uses saturating addition. Zero operands fail the gate. The
    /// predicate does not select this path by itself: earlier eligible families in
    /// the production selector take priority, and candidates bypass these gates.
    fn should_use_toom8_multiplication(left: &BigUint, right: &BigUint) -> bool {
        let shorter = left.bits().min(right.bits());
        let longer = left.bits().max(right.bits());
        shorter >= Self::TOOM8_MULTIPLICATION_THRESHOLD_BITS
            && longer <= shorter.saturating_add(shorter / 8)
    }

    /// Inverse Vandermonde rows for the thirteen unknown middle product coefficients.
    ///
    /// Row k-1 recovers the coefficient of x^k for k=1..13. Each pair stores integer
    /// weights and their positive common divisor, in point order
    /// [1,-1,2,-2,3,-3,4,-4,5,-5,6,-6,7]. Inputs have the constant and x^14 terms
    /// removed. Applied to a valid degree-14 product, every weighted sum is exactly
    /// divisible; the helper's debug assertion alone is not a release-mode proof.
    /// The table is interpolation data, not a measured crossover or approximation.
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

    /// Extract 8 low-to-high nonnegative base-2^chunk_bits coefficients as BigInts.
    ///
    /// Mask each shifted window; bits above 8 windows are silently discarded.
    /// A zero width returns all zeros. The multiplication caller chooses a positive
    /// ceil(max_operand_bits/8) width, making extraction lossless for both inputs.
    /// Index-times-width shifts saturate rather than validate oversized requests;
    /// mask construction and chunk allocation have no explicit resource limit.
    fn toom8_chunks(value: &BigUint, chunk_bits: usize) -> [BigInt; 8] {
        let mask = (BigUint::one() << chunk_bits) - 1_u8;
        std::array::from_fn(|index| {
            BigInt::from((value >> index.saturating_mul(chunk_bits)) & &mask)
        })
    }

    /// Evaluate a degree-at-most-7 coefficient array at an i64 point by Horner's rule.
    ///
    /// Coefficients are ordered from constant upward; signed BigInt arithmetic handles
    /// negative points exactly. This accepts arbitrary signed coefficients, while the
    /// multiplication caller supplies nonnegative chunks. Intermediate products use
    /// the integer backend and allocate without an explicit work or memory bound.
    fn toom8_evaluate(chunks: &[BigInt; 8], point: i64) -> BigInt {
        chunks
            .iter()
            .rev()
            .fold(BigInt::ZERO, |value, chunk| value * point + chunk)
    }

    /// Compute a weighted sum of 13 signed values and divide by a nonzero i64 divisor.
    ///
    /// Interpolation callers supply weights that make the quotient exact. The remainder
    /// is computed unconditionally, but only debug assertions reject nondivisibility;
    /// release division would truncate toward zero for invalid interpolation data.
    /// A zero divisor panics during remainder evaluation. This helper does not return
    /// a typed validation error or impose a budget on the BigInt intermediates.
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

    /// Force the exact Toom-8 kernel for benchmark comparison, bypassing selection gates.
    ///
    /// Accepts arbitrary nonnegative BigUint operands, including zero and unbalanced
    /// pairs. Records rational_algorithm/multiplication-candidate/rust-native-toom8
    /// when tracing is enabled, even for zero inputs. The 15 point products and exact
    /// interpolation return a fully materialized BigUint; scalar-weight multiplications
    /// and recombination are additional work. No GMP representation is introduced.
    /// Internal invariant failures may panic; work, memory and cancellation are not
    /// bounded by this interface, and there is no typed exhaustion result.
    pub fn multiply_magnitudes_toom8_candidate(left: &BigUint, right: &BigUint) -> BigUint {
        #[cfg(feature = "dispatch-trace")]
        crate::trace_dispatch!(
            "rational_algorithm",
            "multiplication-candidate",
            "rust-native-toom8"
        );
        Self::multiply_magnitudes_toom8(left, right)
    }

    /// Multiply arbitrary nonnegative magnitudes by 8-chunk polynomial interpolation.
    ///
    /// Return zero before splitting if either operand is zero. Otherwise choose
    /// chunk_bits=max(1,ceil(max_operand_bits/8)), converting the width to usize with
    /// an expect. Recover endpoint coefficients directly and the 13 middle coefficients
    /// from signed evaluations after removing the endpoint contributions. The static
    /// inverse Vandermonde weights make all interpolation divisions exact for these
    /// inputs; integer convolution makes the recovered coefficients nonnegative.
    ///
    /// Recombine by shifted addition, so coefficients may exceed one base-B digit
    /// and carries are retained. Negative recovered coefficients panic; inexact
    /// division is asserted only in debug builds. The kernel does not consult its
    /// selection threshold or emit its own path event. Integer-backend temporaries
    /// are owned and dropped normally, with no total-work/memory bound or typed
    /// resource/cancellation contract.
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
