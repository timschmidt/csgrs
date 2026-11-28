#[cfg(any(feature = "dispatch-trace", test))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
/// Diagnostic multiplication label inferred from the original operand widths.
///
/// The thresholds mirror num-bigint 0.4.6's four-family schedule, but the locked
/// backend strips low zero limbs before choosing its full-product kernel and has
/// separate zero/scalar exits. This classifier does neither, so its label is not
/// proof of the actual backend path. Rational-level cancellation and Hyperreal's
/// Toom selector also remain separate from this diagnostic.
///
/// Compiled for dispatch tracing or unit tests. Derived Clone/Copy duplicate the
/// fieldless tag, Debug prints its variant, and Eq/PartialEq compare tags; none
/// establish operand equality, arithmetic correctness or equal computational work.
enum BackendMultiplicationAlgorithm {
    /// Original shorter operand has at most 32 native-word limbs, including zero/scalar cases.
    ///
    /// The label groups these shapes under the backend long-multiplication family;
    /// it does not prove that a long-multiplication loop is entered.
    Basecase,
    /// Original shorter width exceeds 32 limbs and the longer is at least twice it; predicts unbalanced splitting.
    HalfKaratsuba,
    /// Original shorter width is 33..256 limbs and the longer is less than twice it; predicts balanced Toom-2.
    Karatsuba,
    /// Original shorter width exceeds 256 limbs and the longer is less than twice it; predicts the backend Toom-3 family.
    Toom3,
}

#[cfg(feature = "dispatch-trace")]
/// Map multiplication diagnostic tags to the existing stable trace strings when tracing is compiled.
impl BackendMultiplicationAlgorithm {
    /// Return the literal backend-* trace label for this tag without allocation or arithmetic.
    ///
    /// The string identifies the classifier result, not an observation inside num-bigint.
    const fn trace_path(self) -> &'static str {
        match self {
            Self::Basecase => "backend-basecase",
            Self::HalfKaratsuba => "backend-half-karatsuba",
            Self::Karatsuba => "backend-karatsuba",
            Self::Toom3 => "backend-toom3",
        }
    }
}

#[cfg(any(feature = "dispatch-trace", test))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
/// Coarse unsigned division/remainder label inferred before invoking the backend.
///
/// The classifier checks zero/small/equal dividends before divisor limb width;
/// num-bigint's div_rem checks a one-limb divisor earlier, and remainder has an
/// additional u32-divisor shortcut. Consequently this is not an exact backend
/// control-flow certificate. Zero divisors are not rejected by classification.
///
/// Available for tracing or tests. Clone/Copy preserve the fieldless tag, Debug
/// names it, and Eq/PartialEq compare tags rather than division values or validity.
enum BackendDivisionAlgorithm {
    /// Dividend is zero, less than divisor or equal to divisor under the classifier's first branch.
    ///
    /// Includes invalid 0/0. Small/equal positive operands can still take a one-limb
    /// backend routine, so this label is not a successful-division certificate.
    TrivialOrSmallQuotient,
    /// After the early branch misses, divisor has at most one native-word limb; includes a zero divisor with positive dividend.
    SingleLimb,
    /// Dividend exceeds a multi-limb divisor; predicts normalized Knuth division without observing backend execution.
    KnuthBasecase,
}

#[cfg(any(feature = "dispatch-trace", test))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
/// Size-only diagnostic for magnitude-to-text work, not an exact conversion-path report.
///
/// The local crossover is 32 native-word limbs. The locked num-bigint 0.4.6
/// non-power-of-two conversion uses its large-base path at 64 limbs, so values
/// with 32..63 limbs are mislabeled by this diagnostic. No radix is supplied;
/// zero and power-of-two conversions have separate backend paths. Current callers
/// use it before rational integer/mixed-fraction decimal formatting.
///
/// Available for tracing or tests. Derived Clone/Copy duplicate the tag, Debug
/// prints its name and Eq/PartialEq compare tags, without validating formatted output.
enum BackendRadixOutputAlgorithm {
    /// Fewer than 32 native-word limbs, including zero; predicts repeated small-divisor conversion without inspecting radix.
    RepeatedSingleLimbDivision,
    /// At least 32 native-word limbs under the local rule; the locked backend large-base threshold is 64, not 32.
    DivideAndConquer,
}

#[cfg(feature = "dispatch-trace")]
/// Map radix-output diagnostic tags to retained trace strings without changing their imperfect classification.
impl BackendRadixOutputAlgorithm {
    /// Return the static radix-output label for this tag; naming a family does not validate its execution or output.
    const fn trace_path(self) -> &'static str {
        match self {
            Self::RepeatedSingleLimbDivision => "backend-repeated-single-limb-division",
            Self::DivideAndConquer => "backend-divide-and-conquer",
        }
    }
}

#[cfg(feature = "dispatch-trace")]
/// Map coarse division tags to stable diagnostic paths when dispatch tracing is compiled.
impl BackendDivisionAlgorithm {
    /// Return the static division label, including labels that classification can assign to invalid zero-divisor inputs.
    const fn trace_path(self) -> &'static str {
        match self {
            Self::TrivialOrSmallQuotient => "backend-trivial-or-small-quotient",
            Self::SingleLimb => "backend-single-limb",
            Self::KnuthBasecase => "backend-knuth-basecase",
        }
    }
}

/// Classify operand shapes for diagnostics and wrap exact backend remainder arithmetic.
///
/// Classification and trace helpers exist only in tracing/test configurations as
/// specified on each item; remainder_magnitudes remains available without tracing.
/// These labels do not affect kernel selection, validate inputs or supply numerical
/// certainty. Known differences from the locked backend are described on the tags.
/// Feature-enabled trace arguments are evaluated before the recorder decides whether
/// recording is active. No resource budget or cancellation is introduced here.
impl Rational {
    #[cfg(any(feature = "dispatch-trace", test))]
    #[inline]
    /// Count native-word limbs as ceil(bits(value)/usize::BITS), with zero yielding zero.
    ///
    /// Matches num-bigint 0.4.6's digit width on 32/64-bit pointer targets. The count
    /// includes low zero limbs and does not model any backend trimming; it is not an
    /// allocation or temporary-memory measurement. Conversion to usize uses expect.
    /// Available for dispatch tracing or unit tests.
    fn backend_limb_count(value: &BigUint) -> usize {
        usize::try_from(value.bits().div_ceil(u64::from(usize::BITS)))
            .expect("BigUint limb count fits usize")
    }

    #[cfg(any(feature = "dispatch-trace", test))]
    /// Classify original limb counts with the basecase, half-Karatsuba, Karatsuba, Toom-3 order.
    ///
    /// Sort the two counts. Choose basecase at shorter<=32, then half-Karatsuba if
    /// 2*shorter<=longer (saturating multiplication), then Karatsuba at shorter<=256,
    /// otherwise Toom-3. No multiplication or low-zero-limb trimming occurs. For
    /// example, two 40-limb powers of the native-word base classify as Karatsuba even
    /// though the backend strips them to one limb and takes its basecase kernel.
    /// This diagnostic does not invoke Hyperreal's higher-Toom production selector.
    fn backend_multiplication_algorithm(
        left: &BigUint,
        right: &BigUint,
    ) -> BackendMultiplicationAlgorithm {
        let left = Self::backend_limb_count(left);
        let right = Self::backend_limb_count(right);
        let (shorter, longer) = if left <= right {
            (left, right)
        } else {
            (right, left)
        };

        if shorter <= 32 {
            BackendMultiplicationAlgorithm::Basecase
        } else if shorter.saturating_mul(2) <= longer {
            BackendMultiplicationAlgorithm::HalfKaratsuba
        } else if shorter <= 256 {
            BackendMultiplicationAlgorithm::Karatsuba
        } else {
            BackendMultiplicationAlgorithm::Toom3
        }
    }

    #[cfg(any(feature = "dispatch-trace", test))]
    /// Classify original magnitudes by ordered early comparisons and divisor limb count.
    ///
    /// Compute divisor width first, then test dividend<divisor, zero dividend and
    /// equality in that short-circuit order. A hit yields TrivialOrSmallQuotient;
    /// otherwise <=1 divisor limb yields SingleLimb and larger divisors yield
    /// KnuthBasecase. No division or nonzero-divisor validation is performed: 0/0
    /// classifies as trivial and positive/0 as single-limb. Backend one-limb and u32
    /// remainder shortcuts have different ordering, so the label can differ from the
    /// executed kernel even on valid input.
    fn backend_division_algorithm(
        dividend: &BigUint,
        divisor: &BigUint,
    ) -> BackendDivisionAlgorithm {
        let divisor_limbs = Self::backend_limb_count(divisor);
        if dividend < divisor || dividend.is_zero() || dividend == divisor {
            BackendDivisionAlgorithm::TrivialOrSmallQuotient
        } else if divisor_limbs <= 1 {
            BackendDivisionAlgorithm::SingleLimb
        } else {
            BackendDivisionAlgorithm::KnuthBasecase
        }
    }

    #[cfg(any(feature = "dispatch-trace", test))]
    /// Apply the local 32-native-limb radix-output threshold without formatting the value.
    ///
    /// Return DivideAndConquer at >=32 limbs, otherwise RepeatedSingleLimbDivision.
    /// The locked backend's non-power-of-two conversion threshold is 64; this known
    /// 32..63-limb discrepancy is preserved here. Zero/radix-specific shortcuts are
    /// not represented, and no radix, output validation or performance evidence is read.
    fn backend_radix_output_algorithm(value: &BigUint) -> BackendRadixOutputAlgorithm {
        if Self::backend_limb_count(value) >= 32 {
            BackendRadixOutputAlgorithm::DivideAndConquer
        } else {
            BackendRadixOutputAlgorithm::RepeatedSingleLimbDivision
        }
    }

    #[cfg(feature = "dispatch-trace")]
    /// Evaluate the multiplication classifier and submit its label under rational_algorithm/operation.
    ///
    /// The classifier executes even if feature-enabled recording is currently inactive;
    /// the recorder then decides whether to retain the event. This does not multiply,
    /// select a backend kernel or certify that the predicted path was used.
    fn trace_backend_multiplication(operation: &'static str, left: &BigUint, right: &BigUint) {
        crate::trace_dispatch!(
            "rational_algorithm",
            operation,
            Self::backend_multiplication_algorithm(left, right).trace_path()
        );
    }

    #[cfg(feature = "dispatch-trace")]
    /// Submit a coarse division label under rational_algorithm and the caller's operation name.
    ///
    /// Classification occurs before the recorder's active-scope check. This helper
    /// does not divide or reject zero; a later failed arithmetic attempt can therefore
    /// have a recorded label. No claim of backend completion follows from the event.
    fn trace_backend_division(operation: &'static str, dividend: &BigUint, divisor: &BigUint) {
        crate::trace_dispatch!(
            "rational_algorithm",
            operation,
            Self::backend_division_algorithm(dividend, divisor).trace_path()
        );
    }

    #[inline]
    /// Return dividend modulo a positive divisor through the borrowed BigUint remainder operator.
    ///
    /// When dispatch tracing is compiled, classify/submit the attempt before executing
    /// remainder; otherwise ignore the operation label. Divisor zero panics in the
    /// backend, possibly after a trace event. For a valid divisor the result is exact,
    /// nonnegative and less than divisor. Used by Rational's fractional-part path,
    /// which handles sign and normalization separately. Backend temporaries have no
    /// explicit work/memory admission, cancellation or typed exhaustion result here.
    fn remainder_magnitudes(
        operation: &'static str,
        dividend: &BigUint,
        divisor: &BigUint,
    ) -> BigUint {
        #[cfg(feature = "dispatch-trace")]
        Self::trace_backend_division(operation, dividend, divisor);
        #[cfg(not(feature = "dispatch-trace"))]
        let _ = operation;
        dividend % divisor
    }

    #[cfg(feature = "dispatch-trace")]
    /// Submit the size-only radix classification under rational_algorithm/binary-to-radix.
    ///
    /// Classification executes before the recorder's active-scope check. This neither
    /// formats the value nor observes the backend converter; the local 32-limb label
    /// must not be treated as proof of its 64-limb large-base path.
    fn trace_backend_radix_output(value: &BigUint) {
        crate::trace_dispatch!(
            "rational_algorithm",
            "binary-to-radix",
            Self::backend_radix_output_algorithm(value).trace_path()
        );
    }

}
