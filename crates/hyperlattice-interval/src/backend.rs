//! Sealed scalar forwarding for Cartesian interval and retained-affine consumers.
//!
//! Implementations cover signed i32/i64 fixed-point intervals, rational intervals,
//! budgeted dyadic intervals, and f32/f64 certified enclosures under their features.
//! Scalar algorithms, domain errors and floating-environment qualification remain
//! in Hyperinterval. No native Real, integer-set, proposal-float or symbolic-ball
//! implementation is supplied. Default features select no concrete backend.
//!
//! Unit contexts add no work accounting to fixed, rational or floating arithmetic;
//! dyadic calls forward the same mutable budget. Constants and explicit Clone are
//! outside that budget. Errors do not imply that prior context charges are refunded.
//! Widths retain exact endpoint distances separately from rounded midpoint/radius
//! enclosures. Singleton-interval width comparisons do not validate nonnegativity
//! or prove that the operands originated from a width query.

use hyperinterval::{FiniteRealInterval, IntervalError};

/// Private implementation permit restricting the arithmetic trait to audited scalar families.
mod sealed {
    /// Marker with no methods; the private module prevents downstream implementations.
    pub trait Sealed {}
}

/// Narrow sealed adapter to Hyperinterval's certified scalar operations.
///
/// No scalar algorithm lives here. Integer sets, proposal floats, and symbolic
/// balls are deliberately not finite real-interval arithmetic backends.
/// Operations preserve scalar errors, rounding and context accounting rather than
/// adding predicate certainty. Clone remains an explicit unbudgeted ownership copy;
/// use copy when the backend's retention accounting must be charged.
pub trait IntervalArithmetic: sealed::Sealed + FiniteRealInterval + Clone {
    /// Caller-owned arithmetic state: `()` or a shared `DyadicBudget`.
    type Context;
    /// Exact width value: a scale-tagged unsigned fixed value, retained float
    /// difference, or rational/dyadic singleton interval. Widths never round.
    type Width;
    /// Exact endpoint distance; `None` for empty. Scalar work shares the context.
    fn width(&self, context: &mut Self::Context) -> Result<Option<Self::Width>, IntervalError>;
    /// Exact order of width values. Interval-valued width backends require
    /// singleton operands; this compares values, not interval set inclusion.
    fn compare_widths(
        a: &Self::Width,
        b: &Self::Width,
        context: &mut Self::Context,
    ) -> Result<core::cmp::Ordering, IntervalError>;
    /// Outward enclosure of the exact midpoint; empty maps to empty.
    fn midpoint_enclosure(&self, context: &mut Self::Context) -> Result<Self, IntervalError>;
    /// Outward enclosure of the exact half-width; empty maps to empty.
    fn radius_enclosure(&self, context: &mut Self::Context) -> Result<Self, IntervalError>;
    /// Construct and discard an empty scalar to validate type-level family constraints.
    ///
    /// Consumers use this even for zero-dimensional products. It does not certify
    /// that every arithmetic operation or floating environment will be supported.
    fn validate_family() -> Result<(), IntervalError> {
        Self::empty().map(|_| ())
    }
    /// Canonical empty scalar set, validating the backend family.
    fn empty() -> Result<Self, IntervalError>;
    /// Exact scalar zero, validating the backend family.
    fn zero() -> Result<Self, IntervalError>;
    /// Exact scalar one, or typed overflow if this family cannot represent it.
    /// Like `zero`, this constant construction is outside dyadic work accounting.
    fn one() -> Result<Self, IntervalError>;
    /// Scalar sum enclosure.
    fn add(&self, other: &Self, context: &mut Self::Context) -> Result<Self, IntervalError>;
    /// Scalar difference enclosure.
    fn sub(&self, other: &Self, context: &mut Self::Context) -> Result<Self, IntervalError>;
    /// Scalar product enclosure.
    fn mul(&self, other: &Self, context: &mut Self::Context) -> Result<Self, IntervalError>;
    /// Exact reflected set, using native negation without aligning a zero operand.
    fn neg(&self, context: &mut Self::Context) -> Result<Self, IntervalError>;
    /// Absolute-value image, including an interior minimum at zero.
    fn abs(&self, context: &mut Self::Context) -> Result<Self, IntervalError>;
    /// Image of the maximum of independently chosen operand members.
    fn max_image(&self, other: &Self, context: &mut Self::Context) -> Result<Self, IntervalError>;
    /// Smallest closed interval containing both inputs.
    fn hull(&self, other: &Self, context: &mut Self::Context) -> Result<Self, IntervalError>;
    /// Exact scalar intersection.
    fn intersection(
        &self,
        other: &Self,
        context: &mut Self::Context,
    ) -> Result<Self, IntervalError>;
    /// Exact inclusion in the other scalar set.
    fn subset_of(&self, other: &Self, context: &mut Self::Context) -> Result<bool, IntervalError>;
    /// Exact inclusion in the real interior of the other scalar set.
    fn interior_subset_of(
        &self,
        other: &Self,
        context: &mut Self::Context,
    ) -> Result<bool, IntervalError>;
    /// Construct an empty scalar, then take its hull with self for an accounted owned copy.
    ///
    /// Empty construction errors precede hull errors. Dyadic work uses the supplied
    /// context; ordinary Clone has no such context and is a separate operation.
    fn copy(&self, context: &mut Self::Context) -> Result<Self, IntervalError> {
        self.hull(&Self::empty()?, context)
    }
}

/// Emit exactly matching set-relation adapters for unit-context backends.
///
/// Wraps native hull/intersection/inclusion results in Ok without adding a scalar
/// decision or work budget. Rational owned results can allocate. Failure-producing
/// arithmetic and dyadic context-bearing relations remain separate implementations.
#[cfg(any(feature = "fixed", feature = "rational", feature = "float-enclosure"))]
macro_rules! unbudgeted_relations {
    () => {
        /// Wrap the native smallest containing interval, ignoring the unit context.
        fn hull(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
            Ok(Self::hull(self, other))
        }
        /// Wrap the native exact set intersection, including successful empty results.
        fn intersection(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
            Ok(Self::intersection(self, other))
        }
        /// Wrap native closed-set inclusion without adding a predicate policy or receipt.
        fn subset_of(&self, other: &Self, _: &mut ()) -> Result<bool, IntervalError> {
            Ok(Self::subset_of(self, other))
        }
        /// Wrap native inclusion in the other's real interior, preserving empty-set rules.
        fn interior_subset_of(&self, other: &Self, _: &mut ()) -> Result<bool, IntervalError> {
            Ok(Self::interior_subset_of(self, other))
        }
    };
}

#[cfg(feature = "fixed")]
/// Emit the i32 or i64 fixed-point adapter for every type-level fractional-bit count.
///
/// Native constructors validate scale; checked arithmetic owns outward rounding and
/// representability failures. The unit context supplies no additional accounting.
macro_rules! fixed {
    ($ty:ty) => {
        /// Admit this signed fixed-point family to the sealed arithmetic adapter.
        impl<const F: u32> sealed::Sealed for hyperinterval::FixedInterval<$ty, F> {}
        /// Forward fixed-point operations while retaining the F-bit binary scale.
        impl<const F: u32> IntervalArithmetic for hyperinterval::FixedInterval<$ty, F> {
            /// No caller-owned work counter; native fixed-width limits still apply.
            type Context = ();
            /// Unsigned exact endpoint distance in units of 2^-F.
            type Width = hyperinterval::FixedWidth<F>;
            /// Return the exact unsigned endpoint distance tagged with F, or None for empty.
            fn width(&self, _: &mut ()) -> Result<Option<Self::Width>, IntervalError> {
                Ok(Self::width(self))
            }
            /// Compare same-scale unsigned widths exactly; no interval projection is needed.
            fn compare_widths(
                a: &Self::Width,
                b: &Self::Width,
                _: &mut (),
            ) -> Result<core::cmp::Ordering, IntervalError> {
                Ok(a.cmp(b))
            }
            /// Forward the outward fixed-grid midpoint enclosure and scalar failures.
            fn midpoint_enclosure(&self, _: &mut ()) -> Result<Self, IntervalError> {
                Self::midpoint_enclosure(self)
            }
            /// Forward the outward fixed-grid half-width enclosure and scalar failures.
            fn radius_enclosure(&self, _: &mut ()) -> Result<Self, IntervalError> {
                Self::radius_enclosure(self)
            }
            /// Validate the fixed family and construct its canonical empty scalar set.
            fn empty() -> Result<Self, IntervalError> {
                Self::empty()
            }
            /// Construct the exact zero point, propagating an invalid type-level scale.
            fn zero() -> Result<Self, IntervalError> {
                Self::point(0)
            }
            /// Import the exact ratio 1/1; invalid scale or unrepresentable one stays typed.
            fn one() -> Result<Self, IntervalError> {
                Self::from_ratio(1, 1)
            }
            /// Forward checked fixed-point addition, preserving empty operands and overflow errors.
            fn add(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
                self.checked_add(other)
            }
            /// Forward checked independent-operand subtraction with native overflow handling.
            fn sub(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
                self.checked_sub(other)
            }
            /// Forward outward fixed-grid multiplication with native overflow handling.
            fn mul(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
                self.checked_mul(other)
            }
            /// Forward exact reflection; an unrepresentable negated endpoint returns overflow.
            fn neg(&self, _: &mut ()) -> Result<Self, IntervalError> {
                self.checked_neg()
            }
            /// Forward the absolute-value image and any endpoint overflow.
            fn abs(&self, _: &mut ()) -> Result<Self, IntervalError> {
                Self::abs(self)
            }
            /// Wrap the native maximum image of independent scalar members in Ok.
            fn max_image(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
                Ok(Self::max_image(self, other))
            }
            unbudgeted_relations!();
        }
    };
}
#[cfg(feature = "fixed")]
fixed!(i32);
#[cfg(feature = "fixed")]
fixed!(i64);

#[cfg(feature = "rational")]
/// Admit the rational interval family to the sealed adapter.
impl sealed::Sealed for hyperinterval::RationalInterval {}
#[cfg(feature = "rational")]
/// Forward rational interval operations without adding a second scalar kernel.
impl IntervalArithmetic for hyperinterval::RationalInterval {
    /// Unit state: arbitrary-precision rational work and allocation have no local budget.
    type Context = ();
    /// Singleton interval carrying the exact distance; comparison also accepts other singleton values.
    type Width = Self;
    /// Wrap the exact rational endpoint distance as a singleton, or return None for empty.
    fn width(&self, _: &mut ()) -> Result<Option<Self::Width>, IntervalError> {
        Ok(Self::width(self).map(Self::point))
    }
    /// Reject empty/nonsingleton operands, then compare their rational lower endpoints exactly.
    /// No positivity or width-origin check is performed, and the unit context adds no budget.
    fn compare_widths(
        a: &Self,
        b: &Self,
        _: &mut (),
    ) -> Result<core::cmp::Ordering, IntervalError> {
        if !a.is_singleton() || !b.is_singleton() {
            return Err(IntervalError::NonSingletonEndpoint);
        }
        Ok(a.bounds()
            .expect("singleton")
            .0
            .partial_cmp(b.bounds().expect("singleton").0)
            .expect("rational order is total"))
    }
    /// Wrap the native exact singleton midpoint; empty input remains empty.
    fn midpoint_enclosure(&self, _: &mut ()) -> Result<Self, IntervalError> {
        Ok(Self::midpoint_enclosure(self))
    }
    /// Wrap the native exact singleton half-width; empty input remains empty.
    fn radius_enclosure(&self, _: &mut ()) -> Result<Self, IntervalError> {
        Ok(Self::radius_enclosure(self))
    }
    /// Wrap the canonical empty rational interval without arithmetic accounting.
    fn empty() -> Result<Self, IntervalError> {
        Ok(Self::empty())
    }
    /// Construct the exact rational zero singleton outside any work budget.
    fn zero() -> Result<Self, IntervalError> {
        Ok(Self::point(0.into()))
    }
    /// Construct the exact rational one singleton outside any work budget.
    fn one() -> Result<Self, IntervalError> {
        Ok(Self::point(1.into()))
    }
    /// Wrap the exact rational sum interval; allocation and coefficient growth are unbounded here.
    fn add(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
        Ok(Self::add(self, other))
    }
    /// Wrap the exact independent-operand difference interval with no context accounting.
    fn sub(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
        Ok(Self::sub(self, other))
    }
    /// Wrap the exact rational product interval with no context accounting.
    fn mul(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
        Ok(Self::mul(self, other))
    }
    /// Wrap exact reflection of the interval, preserving emptiness.
    fn neg(&self, _: &mut ()) -> Result<Self, IntervalError> {
        Ok(Self::neg(self))
    }
    /// Wrap the native exact absolute-value image, including an interior minimum of zero.
    fn abs(&self, _: &mut ()) -> Result<Self, IntervalError> {
        Ok(Self::abs(self))
    }
    /// Wrap the native maximum image of independently chosen rational members.
    fn max_image(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
        Ok(Self::max_image(self, other))
    }
    unbudgeted_relations!();
}

#[cfg(feature = "dyadic")]
/// Admit the budgeted dyadic interval family to the sealed adapter.
impl sealed::Sealed for hyperinterval::DyadicInterval {}
#[cfg(feature = "dyadic")]
/// Forward budgeted dyadic interval operations without adding a second scalar kernel.
impl IntervalArithmetic for hyperinterval::DyadicInterval {
    /// Caller-owned work and intermediate-size limits shared across nested operations.
    type Context = hyperinterval::DyadicBudget;
    /// Singleton interval carrying the exact distance; comparison also accepts other singleton values.
    type Width = Self;
    /// Return None immediately for empty; otherwise request the budgeted exact width singleton.
    fn width(&self, context: &mut Self::Context) -> Result<Option<Self>, IntervalError> {
        if self.is_empty() {
            Ok(None)
        } else {
            Self::width(self, context).map(Some)
        }
    }
    /// Forward budgeted singleton-value comparison, preserving error order.
    /// Empty operands fail before reservation; nonempty nonsingletons can exhaust resources
    /// before NonSingletonEndpoint is reached. No nonnegative-width check is added.
    fn compare_widths(
        a: &Self,
        b: &Self,
        context: &mut Self::Context,
    ) -> Result<core::cmp::Ordering, IntervalError> {
        a.compare_singleton(b, context)
    }
    /// Forward the budgeted midpoint enclosure, exact when the half is representable.
    /// At the minimum exponent, native code rounds outward on the finest stored grid.
    fn midpoint_enclosure(&self, context: &mut Self::Context) -> Result<Self, IntervalError> {
        Self::midpoint_enclosure(self, context)
    }
    /// Forward the budgeted half-width enclosure, exact when the half is representable.
    /// At the minimum exponent, native code rounds outward on the finest stored grid.
    fn radius_enclosure(&self, context: &mut Self::Context) -> Result<Self, IntervalError> {
        Self::radius_enclosure(self, context)
    }
    /// Wrap canonical empty dyadic storage without consuming caller work.
    fn empty() -> Result<Self, IntervalError> {
        Ok(Self::empty())
    }
    /// Construct mantissa zero at exponent zero outside the caller budget.
    fn zero() -> Result<Self, IntervalError> {
        Self::point(0.into(), 0)
    }
    /// Construct mantissa one at exponent zero outside the caller budget.
    fn one() -> Result<Self, IntervalError> {
        Self::point(1.into(), 0)
    }
    /// Forward interval addition through the same mutable dyadic budget.
    fn add(&self, other: &Self, context: &mut Self::Context) -> Result<Self, IntervalError> {
        Self::add(self, other, context)
    }
    /// Forward independent-operand interval subtraction through the shared budget.
    fn sub(&self, other: &Self, context: &mut Self::Context) -> Result<Self, IntervalError> {
        Self::sub(self, other, context)
    }
    /// Forward interval multiplication through the shared budget without resetting it.
    fn mul(&self, other: &Self, context: &mut Self::Context) -> Result<Self, IntervalError> {
        Self::mul(self, other, context)
    }
    /// Forward native reflection with budgeted storage, avoiding a synthetic zero subtraction.
    fn neg(&self, context: &mut Self::Context) -> Result<Self, IntervalError> {
        Self::neg(self, context)
    }
    /// Forward the absolute-value image with shared scalar work/size accounting.
    fn abs(&self, context: &mut Self::Context) -> Result<Self, IntervalError> {
        Self::abs(self, context)
    }
    /// Forward the maximum image with shared scalar work/size accounting.
    fn max_image(&self, other: &Self, context: &mut Self::Context) -> Result<Self, IntervalError> {
        Self::max_image(self, other, context)
    }
    /// Forward the smallest containing interval, including budgeted owned retention.
    fn hull(&self, other: &Self, context: &mut Self::Context) -> Result<Self, IntervalError> {
        Self::hull(self, other, context)
    }
    /// Forward exact intersection and scalar resource errors without a separate budget.
    fn intersection(
        &self,
        other: &Self,
        context: &mut Self::Context,
    ) -> Result<Self, IntervalError> {
        Self::intersection(self, other, context)
    }
    /// Forward exact set inclusion and shared-budget failures without interpreting failure as false.
    fn subset_of(&self, other: &Self, context: &mut Self::Context) -> Result<bool, IntervalError> {
        Self::subset_of(self, other, context)
    }
    /// Forward real-interior inclusion with the same mutable arithmetic budget.
    fn interior_subset_of(
        &self,
        other: &Self,
        context: &mut Self::Context,
    ) -> Result<bool, IntervalError> {
        Self::interior_subset_of(self, other, context)
    }
}

#[cfg(feature = "float-enclosure")]
/// Emit an IEEE enclosure adapter with its matching exact retained-width type.
///
/// Native scalar code owns environment qualification and outward rounding. Retained
/// widths are exact measurements even when projection to a finite scalar would fail.
macro_rules! float {
    ($ty:ty, $width:ty) => {
        /// Admit this certified IEEE enclosure, not the proposal-interval family.
        impl sealed::Sealed for $ty {}
        /// Forward primitive enclosures without introducing a rounding-mode or work context.
        impl IntervalArithmetic for $ty {
            /// No mutable caller state; scalar environment qualification remains authoritative.
            type Context = ();
            /// Exact retained endpoint difference paired with this enclosure precision.
            type Width = $width;
            /// Retain exact endpoint distance without primitive subtraction, or return None for empty.
            fn width(&self, _: &mut ()) -> Result<Option<Self::Width>, IntervalError> {
                Ok(Self::width(self))
            }
            /// Compare retained endpoint differences exactly without first rounding them to intervals.
            fn compare_widths(
                a: &Self::Width,
                b: &Self::Width,
                _: &mut (),
            ) -> Result<core::cmp::Ordering, IntervalError> {
                Ok(a.cmp(b))
            }
            /// Forward the tight outward midpoint projection and native scalar failures.
            fn midpoint_enclosure(&self, _: &mut ()) -> Result<Self, IntervalError> {
                Self::midpoint_enclosure(self)
            }
            /// Forward the tight outward half-width projection and native scalar failures.
            fn radius_enclosure(&self, _: &mut ()) -> Result<Self, IntervalError> {
                Self::radius_enclosure(self)
            }
            /// Wrap canonical empty storage; this does not separately qualify arithmetic support.
            fn empty() -> Result<Self, IntervalError> {
                Ok(Self::empty())
            }
            /// Construct the exact floating zero singleton through native validated point ingress.
            fn zero() -> Result<Self, IntervalError> {
                Self::point(0.0)
            }
            /// Construct the exact floating one singleton through native validated point ingress.
            fn one() -> Result<Self, IntervalError> {
                Self::point(1.0)
            }
            /// Forward certified outward addition and its scalar environment/representability failures.
            fn add(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
                Self::add(self, other)
            }
            /// Forward certified independent-operand subtraction and native scalar failures.
            fn sub(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
                Self::sub(self, other)
            }
            /// Forward certified outward multiplication and native scalar failures.
            fn mul(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
                Self::mul(self, other)
            }
            /// Wrap exact native reflection without an additional arithmetic context.
            fn neg(&self, _: &mut ()) -> Result<Self, IntervalError> {
                Ok(Self::neg(self))
            }
            /// Wrap the native absolute-value image without selecting a midpoint.
            fn abs(&self, _: &mut ()) -> Result<Self, IntervalError> {
                Ok(Self::abs(self))
            }
            /// Wrap the native maximum image of independently chosen members.
            fn max_image(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
                Ok(Self::max_image(self, other))
            }
            unbudgeted_relations!();
        }
    };
}
#[cfg(feature = "float-enclosure")]
float!(hyperinterval::F32Enclosure, hyperinterval::F32Width);
#[cfg(feature = "float-enclosure")]
float!(hyperinterval::F64Enclosure, hyperinterval::F64Width);
