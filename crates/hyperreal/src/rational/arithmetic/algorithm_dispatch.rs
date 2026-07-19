/// Algorithm families selected by `num-bigint` 0.4.6.
///
/// Hyperreal owns the rational-level cross-cancellation and retained-fact
/// dispatch. Once it delegates a magnitude product, these thresholds describe
/// the exact backend selected by the locked dependency. Keeping the classifier
/// here makes that otherwise opaque choice available to dispatch traces and
/// regression tests.
#[cfg(any(feature = "dispatch-trace", test))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum BackendMultiplicationAlgorithm {
    Basecase,
    HalfKaratsuba,
    Karatsuba,
    Toom3,
}

#[cfg(feature = "dispatch-trace")]
impl BackendMultiplicationAlgorithm {
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
enum BackendDivisionAlgorithm {
    TrivialOrSmallQuotient,
    SingleLimb,
    KnuthBasecase,
}

#[cfg(any(feature = "dispatch-trace", test))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum BackendRadixOutputAlgorithm {
    RepeatedSingleLimbDivision,
    DivideAndConquer,
}

#[cfg(feature = "dispatch-trace")]
impl BackendRadixOutputAlgorithm {
    const fn trace_path(self) -> &'static str {
        match self {
            Self::RepeatedSingleLimbDivision => "backend-repeated-single-limb-division",
            Self::DivideAndConquer => "backend-divide-and-conquer",
        }
    }
}

#[cfg(feature = "dispatch-trace")]
impl BackendDivisionAlgorithm {
    const fn trace_path(self) -> &'static str {
        match self {
            Self::TrivialOrSmallQuotient => "backend-trivial-or-small-quotient",
            Self::SingleLimb => "backend-single-limb",
            Self::KnuthBasecase => "backend-knuth-basecase",
        }
    }
}

impl Rational {
    #[cfg(any(feature = "dispatch-trace", test))]
    #[inline]
    fn backend_limb_count(value: &BigUint) -> usize {
        usize::try_from(value.bits().div_ceil(u64::from(usize::BITS)))
            .expect("BigUint limb count fits usize")
    }

    #[cfg(any(feature = "dispatch-trace", test))]
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
    fn backend_radix_output_algorithm(value: &BigUint) -> BackendRadixOutputAlgorithm {
        if Self::backend_limb_count(value) >= 32 {
            BackendRadixOutputAlgorithm::DivideAndConquer
        } else {
            BackendRadixOutputAlgorithm::RepeatedSingleLimbDivision
        }
    }

    #[cfg(feature = "dispatch-trace")]
    fn trace_backend_multiplication(operation: &'static str, left: &BigUint, right: &BigUint) {
        crate::trace_dispatch!(
            "rational_algorithm",
            operation,
            Self::backend_multiplication_algorithm(left, right).trace_path()
        );
    }

    #[cfg(feature = "dispatch-trace")]
    fn trace_backend_division(operation: &'static str, dividend: &BigUint, divisor: &BigUint) {
        crate::trace_dispatch!(
            "rational_algorithm",
            operation,
            Self::backend_division_algorithm(dividend, divisor).trace_path()
        );
    }

    #[inline]
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
    fn trace_backend_radix_output(value: &BigUint) {
        crate::trace_dispatch!(
            "rational_algorithm",
            "binary-to-radix",
            Self::backend_radix_output_algorithm(value).trace_path()
        );
    }

    #[cfg(feature = "dispatch-trace")]
    fn trace_backend_gcd_algorithm() {
        crate::trace_dispatch!("rational_algorithm", "gcd", "backend-binary");
    }

}
