//! Exact-rational facts for borrowed sets of [`Real`] values.
//!
//! These facts sit in `hyperreal` because they are about scalar representation,
//! not about any particular vector, matrix, or predicate object. Geometry crates
//! can carry the summary as object metadata without inspecting rational storage.

use crate::{Rational, RationalStorageClass, Real, RealSign, ZeroKnowledge};

/// Coarse denominator class for an exact borrowed set with a shared scale.
///
/// The enum is intentionally storage-free: it tells callers whether a
/// shift-only dyadic schedule is eligible or whether the shared scale is an
/// ordinary exact rational denominator. It does not expose the denominator.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RealExactSetDenominatorKind {
    /// All values are exact rationals with power-of-two denominators.
    Dyadic,
    /// All values are exact rationals and share a non-dyadic denominator.
    SharedNonDyadic,
}

/// Coarse class for the largest dyadic denominator exponent in a borrowed set.
///
/// A dyadic rational is `n / 2^k`. This enum exposes only a bucket for the
/// largest `k` seen in the set, not the denominator itself. Geometry kernels
/// can use it to distinguish integer-grid, small shift, and large shift
/// schedules while keeping scalar storage private.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RealExactSetDyadicExponentClass {
    /// All dyadic denominators are `2^0`.
    Integer,
    /// The largest dyadic denominator shift is at most 32 bits.
    Small,
    /// The largest dyadic denominator shift is at most 128 bits.
    Medium,
    /// The largest dyadic denominator shift is larger than 128 bits.
    Large,
}

/// Conservative sign pattern for a borrowed set of [`Real`] values.
///
/// This is a scheduling hint, not a predicate result. It lets vector, matrix,
/// and predicate code select sparse or same-sign arithmetic packages when the
/// signs are already structurally certified, while unknown values remain
/// explicit. The boundary follows Yap's EGC model: preserve cheap object facts
/// and defer topological decisions to exact predicates.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RealExactSetSignPattern {
    /// The borrowed set was empty.
    Empty,
    /// Every value is structurally known to be exactly zero.
    AllZero,
    /// Every value is structurally known to be strictly positive.
    AllPositive,
    /// Every value is structurally known to be strictly negative.
    AllNegative,
    /// All signs are known and the set contains more than one sign class.
    MixedKnown,
    /// At least one value has unknown sign or zero status.
    Unknown,
}

/// Exact-rational structure shared by a borrowed set of [`Real`] values.
///
/// This type deliberately exposes facts rather than numerator or denominator
/// storage. Higher crates can choose dyadic or shared-denominator exact
/// schedules, while [`crate::Rational`] keeps ownership of its representation
/// and reduction policy. This follows Yap's exact-geometric-computation
/// guidance to preserve object-level rational structure before scalar
/// expansion; see Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RealExactSetFacts {
    /// Number of values examined.
    pub len: usize,
    /// Number of examined values currently represented as exact rationals.
    pub exact_rational_count: usize,
    /// Number of exact rational values whose reduced denominator is one.
    ///
    /// Integer-grid predicates and transforms can use this to select cheaper
    /// determinant/product-sum packages without inspecting scalar fields. This
    /// is the object-structure preservation advocated by Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997), expressed
    /// as a storage-free fact.
    pub exact_integer_count: usize,
    /// Number of exact rational values that are signed powers of two.
    ///
    /// Signed power-of-two coordinates often reduce multiplication and scaling
    /// to sign and shift work. The count is deliberately coarse: callers learn
    /// that the shape exists, not the exponent or numerator.
    pub exact_power_of_two_count: usize,
    /// Number of values structurally known to be exactly one.
    pub known_one_count: usize,
    /// Number of values structurally known to be exactly minus one.
    pub known_minus_one_count: usize,
    /// Whether every examined value is represented as an exact rational.
    pub all_exact_rational: bool,
    /// Whether every examined value is an exact rational with a power-of-two
    /// denominator.
    pub all_dyadic: bool,
    /// Whether every examined value is exact rational and all reduced
    /// denominators are equal.
    ///
    /// This is a common-scale eligibility fact only. It does not reveal the
    /// denominator itself, so callers remain on the right side of the scalar
    /// abstraction boundary.
    pub shared_denominator: bool,
    /// Largest exact-rational storage bucket seen in the borrowed set.
    ///
    /// This is a coarse bit-size class, not a numerator or denominator leak.
    /// Callers can use it to avoid repeatedly selecting expensive exact routes
    /// for very large scalar coordinates. The bucket is collected
    /// opportunistically during the same scan as the denominator facts.
    pub max_rational_storage: Option<RationalStorageClass>,
    /// Coarse largest dyadic denominator exponent when all values are dyadic.
    ///
    /// This is a bucketed scale fact rather than a denominator leak. It lets
    /// object-level kernels decide whether dyadic exact arithmetic will remain
    /// a cheap shift schedule or should be treated as a larger exact-rational
    /// route. The design follows Yap's EGC recommendation to preserve
    /// numerical object structure before scalar expansion.
    pub max_dyadic_exponent_class: Option<RealExactSetDyadicExponentClass>,
    /// Number of values structurally known to be exactly zero.
    ///
    /// Sparse exact kernels can use this as a numerator-sparsity hint without
    /// reaching into rational storage. See Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997), for the rationale
    /// behind keeping object structure visible until an exact arithmetic
    /// package is selected.
    pub known_zero_count: usize,
    /// Number of values structurally known to be nonzero.
    pub known_nonzero_count: usize,
    /// Number of values whose zero status is not structurally certified.
    pub unknown_zero_count: usize,
    /// Number of values structurally known to be strictly positive.
    pub known_positive_count: usize,
    /// Number of values structurally known to be strictly negative.
    pub known_negative_count: usize,
}

impl RealExactSetFacts {
    /// Returns facts for a borrowed set of reals.
    ///
    /// Empty sets are not considered all-exact: callers use these facts to
    /// select a concrete exact schedule, and no schedule is meaningful without
    /// at least one scalar.
    pub fn from_reals<'a, I>(values: I) -> Self
    where
        I: IntoIterator<Item = &'a Real>,
    {
        crate::trace_dispatch!("real", "exact_set_facts", "scan");
        let mut len = 0_usize;
        let mut exact_rational_count = 0_usize;
        let mut exact_integer_count = 0_usize;
        let mut exact_power_of_two_count = 0_usize;
        let mut known_one_count = 0_usize;
        let mut known_minus_one_count = 0_usize;
        let mut all_dyadic = true;
        let mut shared_denominator = true;
        let mut first_rational = None::<&Rational>;
        let mut max_rational_storage = None::<RationalStorageClass>;
        let mut max_dyadic_shift = Some(0_u64);
        let mut known_zero_count = 0_usize;
        let mut known_nonzero_count = 0_usize;
        let mut unknown_zero_count = 0_usize;
        let mut known_positive_count = 0_usize;
        let mut known_negative_count = 0_usize;

        for value in values {
            len += 1;
            let structural = value.structural_facts();
            match structural.zero {
                ZeroKnowledge::Zero => known_zero_count += 1,
                ZeroKnowledge::NonZero => known_nonzero_count += 1,
                ZeroKnowledge::Unknown => unknown_zero_count += 1,
            }
            match structural.sign {
                Some(RealSign::Positive) => known_positive_count += 1,
                Some(RealSign::Negative) => known_negative_count += 1,
                Some(RealSign::Zero) | None => {}
            }

            let Some(rational) = value.exact_rational_ref() else {
                all_dyadic = false;
                shared_denominator = false;
                max_dyadic_shift = None;
                continue;
            };

            exact_rational_count += 1;
            let rational_facts = rational.detailed_rational_facts();
            if rational_facts.exact_integer {
                exact_integer_count += 1;
            }
            if rational_facts.power_of_two {
                exact_power_of_two_count += 1;
            }
            if rational.is_one() {
                known_one_count += 1;
            }
            if rational.is_minus_one() {
                known_minus_one_count += 1;
            }
            let dyadic_shift = rational.dyadic_denominator_shift();
            all_dyadic &= dyadic_shift.is_some();
            max_dyadic_shift = match (max_dyadic_shift, dyadic_shift) {
                (Some(current), Some(next)) => Some(current.max(next)),
                _ => None,
            };
            max_rational_storage = max_storage_class(max_rational_storage, rational_facts.storage);
            if let Some(first) = first_rational {
                shared_denominator &= first.same_denominator(rational);
            } else {
                first_rational = Some(rational);
            }
        }

        let all_exact_rational = len != 0 && exact_rational_count == len;
        Self {
            len,
            exact_rational_count,
            exact_integer_count,
            exact_power_of_two_count,
            known_one_count,
            known_minus_one_count,
            all_exact_rational,
            all_dyadic: all_exact_rational && all_dyadic,
            shared_denominator: all_exact_rational && shared_denominator,
            max_rational_storage,
            max_dyadic_exponent_class: (all_exact_rational && all_dyadic)
                .then(|| dyadic_exponent_class(max_dyadic_shift.unwrap_or(0))),
            known_zero_count,
            known_nonzero_count,
            unknown_zero_count,
            known_positive_count,
            known_negative_count,
        }
    }

    /// Returns true when the set contains at least one value and all values are
    /// exact rationals.
    #[inline]
    pub fn is_nonempty_exact_rational(self) -> bool {
        self.len != 0 && self.all_exact_rational
    }

    /// Returns true when a dyadic exact schedule can be selected directly.
    #[inline]
    pub fn has_dyadic_schedule(self) -> bool {
        self.is_nonempty_exact_rational() && self.all_dyadic
    }

    /// Returns true when every value is an exact rational integer.
    ///
    /// This is a coordinate-set fact for integer-grid kernels. It is stronger
    /// than [`Self::has_dyadic_schedule`] and avoids rescanning denominators in
    /// determinant-heavy callers.
    #[inline]
    pub fn has_integer_grid_schedule(self) -> bool {
        self.is_nonempty_exact_rational() && self.exact_integer_count == self.len
    }

    /// Returns true when every value is structurally zero, one, or minus one.
    ///
    /// Signed-permutation and axis-aligned transforms can use this as a cheap
    /// route selector while still leaving geometric decisions to exact
    /// predicates.
    #[inline]
    pub fn has_signed_unit_schedule(self) -> bool {
        self.len != 0
            && self.unknown_zero_count == 0
            && self.known_zero_count + self.known_one_count + self.known_minus_one_count == self.len
    }

    /// Returns true when an equal-denominator exact schedule can be selected
    /// directly.
    #[inline]
    pub fn has_shared_denominator_schedule(self) -> bool {
        self.is_nonempty_exact_rational() && self.shared_denominator
    }

    /// Returns the coarse denominator kind when a shared-scale exact schedule is
    /// available.
    ///
    /// This is the scalar fact that vector, matrix, and predicate objects carry
    /// upward for Yap-style exact geometric computation: choose the arithmetic
    /// package from preserved object facts before expanding scalar arithmetic.
    #[inline]
    pub fn shared_denominator_kind(self) -> Option<RealExactSetDenominatorKind> {
        if !self.has_shared_denominator_schedule() {
            return None;
        }
        if self.all_dyadic {
            Some(RealExactSetDenominatorKind::Dyadic)
        } else {
            Some(RealExactSetDenominatorKind::SharedNonDyadic)
        }
    }

    /// Returns the conservative sign pattern for the borrowed set.
    ///
    /// This method derives its answer only from cached/structural sign and zero
    /// facts gathered during [`Self::from_reals`]. It does not refine
    /// computable values, so callers can use it speculatively in hot exact
    /// dispatch code.
    #[inline]
    pub fn sign_pattern(self) -> RealExactSetSignPattern {
        if self.len == 0 {
            return RealExactSetSignPattern::Empty;
        }
        if self.unknown_zero_count != 0
            || self.known_zero_count + self.known_positive_count + self.known_negative_count
                != self.len
        {
            return RealExactSetSignPattern::Unknown;
        }
        if self.known_zero_count == self.len {
            RealExactSetSignPattern::AllZero
        } else if self.known_positive_count == self.len {
            RealExactSetSignPattern::AllPositive
        } else if self.known_negative_count == self.len {
            RealExactSetSignPattern::AllNegative
        } else {
            RealExactSetSignPattern::MixedKnown
        }
    }
}

fn max_storage_class(
    current: Option<RationalStorageClass>,
    next: RationalStorageClass,
) -> Option<RationalStorageClass> {
    Some(match current {
        Some(current) if storage_rank(current) >= storage_rank(next) => current,
        _ => next,
    })
}

fn storage_rank(storage: RationalStorageClass) -> u8 {
    match storage {
        RationalStorageClass::Zero => 0,
        RationalStorageClass::WordSized => 1,
        RationalStorageClass::MultiLimb => 2,
        RationalStorageClass::VeryLarge => 3,
    }
}

fn dyadic_exponent_class(shift: u64) -> RealExactSetDyadicExponentClass {
    match shift {
        0 => RealExactSetDyadicExponentClass::Integer,
        1..=32 => RealExactSetDyadicExponentClass::Small,
        33..=128 => RealExactSetDyadicExponentClass::Medium,
        _ => RealExactSetDyadicExponentClass::Large,
    }
}
