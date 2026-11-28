/// Exact Toom-6 magnitude multiplication and its selection/interpolation helpers.
///
/// Split each nonnegative integer into 6 base-B coefficients, B=2^chunk_bits.
/// Multiplying the degree-5 polynomials gives degree 10; 11 point products
/// recover it at 0, +/-1, +/-2, +/-3, +/-4, 5 and the leading coefficient (infinity).
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
    /// Minimum shorter-operand bit width (524,288) for the Toom-6 selector gate.
    ///
    /// This tuning threshold works together with the balance predicate and selector
    /// priority; it is neither a mathematical validity limit nor a resource bound.
    const TOOM6_MULTIPLICATION_THRESHOLD_BITS: u64 = 524_288;

    /// Test the Toom-6 size/balance gate without multiplying or allocating chunk buffers.
    ///
    /// Require shorter >= 524,288 bits and longer <= shorter+floor(shorter/6).
    /// The upper bound uses saturating addition. Zero operands fail the gate. The
    /// predicate does not select this path by itself: earlier eligible families in
    /// the production selector take priority, and candidates bypass these gates.
    fn should_use_toom6_multiplication(left: &BigUint, right: &BigUint) -> bool {
        let shorter = left.bits().min(right.bits());
        let longer = left.bits().max(right.bits());
        shorter >= Self::TOOM6_MULTIPLICATION_THRESHOLD_BITS
            && longer <= shorter.saturating_add(shorter / 6)
    }

    /// Extract 6 low-to-high nonnegative base-2^chunk_bits coefficients as BigInts.
    ///
    /// Mask each shifted window; bits above 6 windows are silently discarded.
    /// A zero width returns all zeros. The multiplication caller chooses a positive
    /// ceil(max_operand_bits/6) width, making extraction lossless for both inputs.
    /// Index-times-width shifts saturate rather than validate oversized requests;
    /// mask construction and chunk allocation have no explicit resource limit.
    fn toom6_chunks(value: &BigUint, chunk_bits: usize) -> [BigInt; 6] {
        let mask = (BigUint::one() << chunk_bits) - 1_u8;
        std::array::from_fn(|index| {
            BigInt::from((value >> index.saturating_mul(chunk_bits)) & &mask)
        })
    }

    /// Evaluate a degree-at-most-5 coefficient array at an i64 point by Horner's rule.
    ///
    /// Coefficients are ordered from constant upward; signed BigInt arithmetic handles
    /// negative points exactly. This accepts arbitrary signed coefficients, while the
    /// multiplication caller supplies nonnegative chunks. Intermediate products use
    /// the integer backend and allocate without an explicit work or memory bound.
    fn toom6_evaluate(chunks: &[BigInt; 6], point: i64) -> BigInt {
        chunks
            .iter()
            .rev()
            .fold(BigInt::ZERO, |value, chunk| value * point + chunk)
    }

    /// Compute a weighted sum of 9 signed values and divide by a nonzero i64 divisor.
    ///
    /// Interpolation callers supply weights that make the quotient exact. The remainder
    /// is computed unconditionally, but only debug assertions reject nondivisibility;
    /// release division would truncate toward zero for invalid interpolation data.
    /// A zero divisor panics during remainder evaluation. This helper does not return
    /// a typed validation error or impose a budget on the BigInt intermediates.
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

    /// Force the exact Toom-6 kernel for benchmark comparison, bypassing selection gates.
    ///
    /// Accepts arbitrary nonnegative BigUint operands, including zero and unbalanced
    /// pairs. Records rational_algorithm/multiplication-candidate/rust-native-toom6
    /// when tracing is enabled, even for zero inputs. The 11 point products and exact
    /// interpolation return a fully materialized BigUint; scalar-weight multiplications
    /// and recombination are additional work. No GMP representation is introduced.
    /// Internal invariant failures may panic; work, memory and cancellation are not
    /// bounded by this interface, and there is no typed exhaustion result.
    pub fn multiply_magnitudes_toom6_candidate(left: &BigUint, right: &BigUint) -> BigUint {
        #[cfg(feature = "dispatch-trace")]
        crate::trace_dispatch!(
            "rational_algorithm",
            "multiplication-candidate",
            "rust-native-toom6"
        );
        Self::multiply_magnitudes_toom6(left, right)
    }

    /// Multiply arbitrary nonnegative magnitudes by 6-chunk polynomial interpolation.
    ///
    /// Return zero before splitting if either operand is zero. Otherwise choose
    /// chunk_bits=max(1,ceil(max_operand_bits/6)), converting the width to usize with
    /// an expect. Recover endpoint coefficients directly and the 9 middle coefficients
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
