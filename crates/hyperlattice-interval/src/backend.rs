use hyperinterval::{FiniteRealInterval, IntervalError};

mod sealed {
    pub trait Sealed {}
}

/// Narrow sealed adapter to Hyperinterval's certified scalar operations.
///
/// No scalar algorithm lives here. Integer sets, proposal floats, and symbolic
/// balls are deliberately not finite real-interval arithmetic backends.
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
    /// Validates type-level constraints even for zero-dimensional products.
    fn validate_family() -> Result<(), IntervalError> {
        Self::empty().map(|_| ())
    }
    /// Canonical empty scalar set, validating the backend family.
    fn empty() -> Result<Self, IntervalError>;
    /// Exact scalar zero, validating the backend family.
    fn zero() -> Result<Self, IntervalError>;
    /// Scalar sum enclosure.
    fn add(&self, other: &Self, context: &mut Self::Context) -> Result<Self, IntervalError>;
    /// Scalar difference enclosure.
    fn sub(&self, other: &Self, context: &mut Self::Context) -> Result<Self, IntervalError>;
    /// Scalar product enclosure.
    fn mul(&self, other: &Self, context: &mut Self::Context) -> Result<Self, IntervalError>;
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
    /// Owned copy with the backend's normal retention accounting.
    fn copy(&self, context: &mut Self::Context) -> Result<Self, IntervalError> {
        self.hull(&Self::empty()?, context)
    }
}

// Only exactly matching pass-throughs share an adapter. Scalar failure and
// budget-sensitive cases remain explicit below.
#[cfg(any(feature = "fixed", feature = "rational", feature = "float-enclosure"))]
macro_rules! unbudgeted_relations {
    () => {
        fn hull(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
            Ok(Self::hull(self, other))
        }
        fn intersection(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
            Ok(Self::intersection(self, other))
        }
        fn subset_of(&self, other: &Self, _: &mut ()) -> Result<bool, IntervalError> {
            Ok(Self::subset_of(self, other))
        }
        fn interior_subset_of(&self, other: &Self, _: &mut ()) -> Result<bool, IntervalError> {
            Ok(Self::interior_subset_of(self, other))
        }
    };
}

#[cfg(feature = "fixed")]
macro_rules! fixed {
    ($ty:ty) => {
        impl<const F: u32> sealed::Sealed for hyperinterval::FixedInterval<$ty, F> {}
        impl<const F: u32> IntervalArithmetic for hyperinterval::FixedInterval<$ty, F> {
            type Context = ();
            type Width = hyperinterval::FixedWidth<F>;
            fn width(&self, _: &mut ()) -> Result<Option<Self::Width>, IntervalError> {
                Ok(Self::width(self))
            }
            fn compare_widths(
                a: &Self::Width,
                b: &Self::Width,
                _: &mut (),
            ) -> Result<core::cmp::Ordering, IntervalError> {
                Ok(a.cmp(b))
            }
            fn empty() -> Result<Self, IntervalError> {
                Self::empty()
            }
            fn zero() -> Result<Self, IntervalError> {
                Self::point(0)
            }
            fn add(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
                self.checked_add(other)
            }
            fn sub(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
                self.checked_sub(other)
            }
            fn mul(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
                self.checked_mul(other)
            }
            fn abs(&self, _: &mut ()) -> Result<Self, IntervalError> {
                Self::abs(self)
            }
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
impl sealed::Sealed for hyperinterval::RationalInterval {}
#[cfg(feature = "rational")]
impl IntervalArithmetic for hyperinterval::RationalInterval {
    type Context = ();
    type Width = Self;
    fn width(&self, _: &mut ()) -> Result<Option<Self::Width>, IntervalError> {
        Ok(Self::width(self).map(Self::point))
    }
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
    fn empty() -> Result<Self, IntervalError> {
        Ok(Self::empty())
    }
    fn zero() -> Result<Self, IntervalError> {
        Ok(Self::point(0.into()))
    }
    fn add(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
        Ok(Self::add(self, other))
    }
    fn sub(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
        Ok(Self::sub(self, other))
    }
    fn mul(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
        Ok(Self::mul(self, other))
    }
    fn abs(&self, _: &mut ()) -> Result<Self, IntervalError> {
        Ok(Self::abs(self))
    }
    fn max_image(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
        Ok(Self::max_image(self, other))
    }
    unbudgeted_relations!();
}

#[cfg(feature = "dyadic")]
impl sealed::Sealed for hyperinterval::DyadicInterval {}
#[cfg(feature = "dyadic")]
impl IntervalArithmetic for hyperinterval::DyadicInterval {
    type Context = hyperinterval::DyadicBudget;
    type Width = Self;
    fn width(&self, context: &mut Self::Context) -> Result<Option<Self>, IntervalError> {
        if self.is_empty() {
            Ok(None)
        } else {
            Self::width(self, context).map(Some)
        }
    }
    fn compare_widths(
        a: &Self,
        b: &Self,
        context: &mut Self::Context,
    ) -> Result<core::cmp::Ordering, IntervalError> {
        a.compare_singleton(b, context)
    }
    fn empty() -> Result<Self, IntervalError> {
        Ok(Self::empty())
    }
    fn zero() -> Result<Self, IntervalError> {
        Self::point(0.into(), 0)
    }
    fn add(&self, other: &Self, context: &mut Self::Context) -> Result<Self, IntervalError> {
        Self::add(self, other, context)
    }
    fn sub(&self, other: &Self, context: &mut Self::Context) -> Result<Self, IntervalError> {
        Self::sub(self, other, context)
    }
    fn mul(&self, other: &Self, context: &mut Self::Context) -> Result<Self, IntervalError> {
        Self::mul(self, other, context)
    }
    fn abs(&self, context: &mut Self::Context) -> Result<Self, IntervalError> {
        Self::abs(self, context)
    }
    fn max_image(&self, other: &Self, context: &mut Self::Context) -> Result<Self, IntervalError> {
        Self::max_image(self, other, context)
    }
    fn hull(&self, other: &Self, context: &mut Self::Context) -> Result<Self, IntervalError> {
        Self::hull(self, other, context)
    }
    fn intersection(
        &self,
        other: &Self,
        context: &mut Self::Context,
    ) -> Result<Self, IntervalError> {
        Self::intersection(self, other, context)
    }
    fn subset_of(&self, other: &Self, context: &mut Self::Context) -> Result<bool, IntervalError> {
        Self::subset_of(self, other, context)
    }
    fn interior_subset_of(
        &self,
        other: &Self,
        context: &mut Self::Context,
    ) -> Result<bool, IntervalError> {
        Self::interior_subset_of(self, other, context)
    }
}

#[cfg(feature = "float-enclosure")]
macro_rules! float {
    ($ty:ty, $width:ty) => {
        impl sealed::Sealed for $ty {}
        impl IntervalArithmetic for $ty {
            type Context = ();
            type Width = $width;
            fn width(&self, _: &mut ()) -> Result<Option<Self::Width>, IntervalError> {
                Ok(Self::width(self))
            }
            fn compare_widths(
                a: &Self::Width,
                b: &Self::Width,
                _: &mut (),
            ) -> Result<core::cmp::Ordering, IntervalError> {
                Ok(a.cmp(b))
            }
            fn empty() -> Result<Self, IntervalError> {
                Ok(Self::empty())
            }
            fn zero() -> Result<Self, IntervalError> {
                Self::point(0.0)
            }
            fn add(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
                Self::add(self, other)
            }
            fn sub(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
                Self::sub(self, other)
            }
            fn mul(&self, other: &Self, _: &mut ()) -> Result<Self, IntervalError> {
                Self::mul(self, other)
            }
            fn abs(&self, _: &mut ()) -> Result<Self, IntervalError> {
                Ok(Self::abs(self))
            }
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
