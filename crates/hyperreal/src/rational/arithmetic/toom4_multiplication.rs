/// Exact Toom-4 magnitude multiplication and its selection/interpolation helpers.
///
/// Split each nonnegative integer into 4 base-B coefficients, B=2^chunk_bits.
/// Multiplying the degree-3 polynomials gives degree 6; 7 point products
/// recover it at 0, 1, -1, 2, -2, 3 and the leading coefficient (infinity).
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
    /// Minimum shorter-operand bit width (1,048,576) for the Toom-4 selector gate.
    ///
    /// This tuning threshold works together with the balance predicate and selector
    /// priority; it is neither a mathematical validity limit nor a resource bound.
    const TOOM4_MULTIPLICATION_THRESHOLD_BITS: u64 = 1_048_576;

    /// Test the Toom-4 size/balance gate without multiplying or allocating chunk buffers.
    ///
    /// Require shorter >= 1,048,576 bits and longer <= shorter+floor(shorter/4).
    /// The upper bound uses saturating addition. Zero operands fail the gate. The
    /// predicate does not select this path by itself: earlier eligible families in
    /// the production selector take priority, and candidates bypass these gates.
    fn should_use_toom4_multiplication(left: &BigUint, right: &BigUint) -> bool {
        let shorter = left.bits().min(right.bits());
        let longer = left.bits().max(right.bits());
        shorter >= Self::TOOM4_MULTIPLICATION_THRESHOLD_BITS
            && longer <= shorter.saturating_add(shorter / 4)
    }

    /// Materialize the exact nonnegative product through the ordered magnitude selector.
    ///
    /// Try Toom-8, then Toom-6, then Toom-4, then the native BigUint product. The order
    /// resolves overlapping size/balance gates and is observable in dispatch traces.
    /// Each selected path records rational_algorithm under the supplied static
    /// operation label when tracing is enabled; otherwise that label is unused.
    ///
    /// Callers include rational products/cross differences and half-GCD matrices.
    /// Signs, cancellation and rational normalization remain their responsibility.
    /// No retained-product cache or work/cancellation limit is introduced here.
    /// Zero reaches the backend fallback; successful output is a fully materialized
    /// BigUint, with backend/allocation failures rather than a typed resource result.
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

    /// Benchmark entry point for the production nonnegative-integer product selector.
    ///
    /// Uses the same ordered size/balance gates as rational arithmetic and attributes
    /// tracing to multiplication-benchmark. Both zero and arbitrary finite BigUint
    /// operands are admitted; the result is exact and fully materialized. This does
    /// not force a particular Toom family or reset caches, and has no work/memory
    /// budget, cancellation or typed exhaustion result.
    pub fn multiply_magnitudes_selected(left: &BigUint, right: &BigUint) -> BigUint {
        Self::multiply_magnitudes("multiplication-benchmark", left, right)
    }

    /// Extract 4 low-to-high nonnegative base-2^chunk_bits coefficients as BigInts.
    ///
    /// Mask each shifted window; bits above 4 windows are silently discarded.
    /// A zero width returns all zeros. The multiplication caller chooses a positive
    /// ceil(max_operand_bits/4) width, making extraction lossless for both inputs.
    /// Index-times-width shifts saturate rather than validate oversized requests;
    /// mask construction and chunk allocation have no explicit resource limit.
    fn toom4_chunks(value: &BigUint, chunk_bits: usize) -> [BigInt; 4] {
        let mask = (BigUint::one() << chunk_bits) - 1_u8;
        std::array::from_fn(|index| {
            BigInt::from((value >> index.saturating_mul(chunk_bits)) & &mask)
        })
    }

    /// Evaluate a degree-at-most-3 coefficient array at an i64 point by Horner's rule.
    ///
    /// Coefficients are ordered from constant upward; signed BigInt arithmetic handles
    /// negative points exactly. This accepts arbitrary signed coefficients, while the
    /// multiplication caller supplies nonnegative chunks. Intermediate products use
    /// the integer backend and allocate without an explicit work or memory bound.
    fn toom4_evaluate(chunks: &[BigInt; 4], point: i64) -> BigInt {
        chunks
            .iter()
            .rev()
            .fold(BigInt::ZERO, |value, chunk| value * point + chunk)
    }

    /// Compute a weighted sum of 5 signed values and divide by a nonzero i64 divisor.
    ///
    /// Interpolation callers supply weights that make the quotient exact. The remainder
    /// is computed unconditionally, but only debug assertions reject nondivisibility;
    /// release division would truncate toward zero for invalid interpolation data.
    /// A zero divisor panics during remainder evaluation. This helper does not return
    /// a typed validation error or impose a budget on the BigInt intermediates.
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

    /// Force the exact Toom-4 kernel for benchmark comparison, bypassing selection gates.
    ///
    /// Accepts arbitrary nonnegative BigUint operands, including zero and unbalanced
    /// pairs. Records rational_algorithm/multiplication-candidate/rust-native-toom4
    /// when tracing is enabled, even for zero inputs. The 7 point products and exact
    /// interpolation return a fully materialized BigUint; scalar-weight multiplications
    /// and recombination are additional work. No GMP representation is introduced.
    /// Internal invariant failures may panic; work, memory and cancellation are not
    /// bounded by this interface, and there is no typed exhaustion result.
    pub fn multiply_magnitudes_toom4_candidate(left: &BigUint, right: &BigUint) -> BigUint {
        #[cfg(feature = "dispatch-trace")]
        crate::trace_dispatch!(
            "rational_algorithm",
            "multiplication-candidate",
            "rust-native-toom4"
        );
        Self::multiply_magnitudes_toom4(left, right)
    }

    /// Multiply arbitrary nonnegative magnitudes by 4-chunk polynomial interpolation.
    ///
    /// Return zero before splitting if either operand is zero. Otherwise choose
    /// chunk_bits=max(1,ceil(max_operand_bits/4)), converting the width to usize with
    /// an expect. Recover endpoint coefficients directly and the 5 middle coefficients
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
