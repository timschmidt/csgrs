//! Scalar capabilities used by geometry predicates.

use core::ops::{Add, Mul, Sub};

use crate::predicate::{Sign, SignKnowledge};

/// Conservative magnitude bounds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagnitudeBounds {
    /// Lower bound on absolute value.
    pub abs_lower: f64,
    /// Upper bound on absolute value.
    pub abs_upper: f64,
}

impl MagnitudeBounds {
    /// Construct exact lower and upper absolute-value bounds.
    pub const fn exact(abs: f64) -> Self {
        Self {
            abs_lower: abs,
            abs_upper: abs,
        }
    }
}

/// Structural facts a scalar may expose without full evaluation.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ScalarFacts {
    /// Known exact sign, if available.
    pub sign: Option<Sign>,
    /// Whether the scalar is exactly zero, if known.
    pub exact_zero: Option<bool>,
    /// Whether zero has been ruled out, if known.
    pub provably_nonzero: Option<bool>,
    /// Whether the scalar representation is exact, if known.
    pub exact: Option<bool>,
    /// Whether the scalar is known to be rational-only, if known.
    pub rational_only: Option<bool>,
    /// Conservative magnitude bounds, if available.
    pub magnitude: Option<MagnitudeBounds>,
}

impl ScalarFacts {
    /// Convert structural facts into the public sign-knowledge model.
    #[inline(always)]
    pub fn sign_knowledge(self) -> SignKnowledge {
        if let Some(sign) = self.sign {
            SignKnowledge::exact(sign)
        } else if self.exact_zero == Some(true) {
            SignKnowledge::exact(Sign::Zero)
        } else if self.provably_nonzero == Some(true) {
            SignKnowledge::NonZero
        } else {
            SignKnowledge::Unknown
        }
    }
}

/// Minimal structural interface. Backends can implement only what they know.
pub trait StructuralScalar: Sized {
    /// Return all cheap structural facts known for this scalar.
    #[inline(always)]
    fn scalar_facts(&self) -> ScalarFacts {
        crate::trace_dispatch!("hyperlimit", "structural_scalar", "default-facts");
        ScalarFacts::default()
    }

    /// Return known sign information without forcing full evaluation.
    #[inline(always)]
    fn known_sign(&self) -> SignKnowledge {
        crate::trace_dispatch!("hyperlimit", "structural_scalar", "default-known-sign");
        self.scalar_facts().sign_knowledge()
    }

    /// Return whether the scalar is exactly zero, if known.
    #[inline(always)]
    fn is_exact_zero(&self) -> Option<bool> {
        crate::trace_dispatch!("hyperlimit", "structural_scalar", "default-exact-zero");
        self.scalar_facts().exact_zero
    }

    /// Return whether the scalar is known not to be zero, if known.
    #[inline(always)]
    fn is_provably_nonzero(&self) -> Option<bool> {
        crate::trace_dispatch!(
            "hyperlimit",
            "structural_scalar",
            "default-provably-nonzero"
        );
        self.scalar_facts().provably_nonzero
    }

    /// Return whether the scalar is known to be rational-only, if known.
    #[inline(always)]
    fn is_rational_only(&self) -> Option<bool> {
        crate::trace_dispatch!("hyperlimit", "structural_scalar", "default-rational-only");
        self.scalar_facts().rational_only
    }

    /// Return conservative magnitude bounds, if available.
    #[inline(always)]
    fn magnitude_bounds(&self) -> Option<MagnitudeBounds> {
        crate::trace_dispatch!("hyperlimit", "structural_scalar", "default-magnitude");
        self.scalar_facts().magnitude
    }

    /// Try to refine the sign without going below `min_precision`.
    #[inline(always)]
    fn refine_sign_until(&self, _min_precision: i32) -> SignKnowledge {
        crate::trace_dispatch!("hyperlimit", "structural_scalar", "default-refine-unknown");
        SignKnowledge::Unknown
    }
}

/// Numeric operations needed by the starter predicates.
///
/// This is deliberately small. Rich backends should add capabilities by
/// implementing additional traits, not by growing this trait into a full CAS.
pub trait PredicateScalar:
    StructuralScalar
    + Clone
    + core::fmt::Debug
    + Add<Output = Self>
    + Sub<Output = Self>
    + Mul<Output = Self>
{
    /// Convert to `f64` for cheap filters. Returning `None` disables that stage.
    fn to_f64(&self) -> Option<f64>;

    /// Whether predicate kernels should try `f64` filters before constructing
    /// scalar arithmetic expressions.
    #[inline]
    fn prefer_f64_filter_before_arithmetic() -> bool {
        crate::trace_dispatch!("hyperlimit", "predicate_scalar", "default-no-prefilter");
        false
    }
}

/// Predicate scalar with borrowed arithmetic.
///
/// Predicate kernels use this trait to build intermediate determinant terms
/// from borrowed operands. This avoids cloning every coordinate and
/// subexpression for scalar backends where cloning is materially more expensive
/// than primitive numeric copies.
pub trait BorrowedPredicateScalar: PredicateScalar {
    /// Add two borrowed scalar values.
    fn add_ref(&self, rhs: &Self) -> Self;

    /// Subtract two borrowed scalar values.
    fn sub_ref(&self, rhs: &Self) -> Self;

    /// Multiply two borrowed scalar values.
    fn mul_ref(&self, rhs: &Self) -> Self;
}

impl<T> BorrowedPredicateScalar for T
where
    T: PredicateScalar,
    for<'a, 'b> &'a T: Add<&'b T, Output = T> + Sub<&'b T, Output = T> + Mul<&'b T, Output = T>,
{
    #[inline]
    fn add_ref(&self, rhs: &Self) -> Self {
        crate::trace_dispatch!("hyperlimit", "borrowed_scalar_op", "add-ref-default");
        self + rhs
    }

    #[inline]
    fn sub_ref(&self, rhs: &Self) -> Self {
        crate::trace_dispatch!("hyperlimit", "borrowed_scalar_op", "sub-ref-default");
        self - rhs
    }

    #[inline]
    fn mul_ref(&self, rhs: &Self) -> Self {
        crate::trace_dispatch!("hyperlimit", "borrowed_scalar_op", "mul-ref-default");
        self * rhs
    }
}

macro_rules! impl_float_scalar {
    ($ty:ty, $prefer_filter:expr) => {
        impl StructuralScalar for $ty {
            #[inline(always)]
            fn scalar_facts(&self) -> ScalarFacts {
                crate::trace_dispatch!("hyperlimit", "float_scalar", "structural-facts");
                ScalarFacts {
                    sign: None,
                    exact_zero: None,
                    provably_nonzero: None,
                    exact: Some(false),
                    rational_only: Some(false),
                    magnitude: if self.is_nan() {
                        None
                    } else {
                        Some(MagnitudeBounds::exact(self.abs() as f64))
                    },
                }
            }
        }

        impl PredicateScalar for $ty {
            #[inline(always)]
            fn to_f64(&self) -> Option<f64> {
                if self.is_nan() {
                    crate::trace_dispatch!("hyperlimit", "float_scalar", "to-f64-nan");
                    None
                } else {
                    crate::trace_dispatch!("hyperlimit", "float_scalar", "to-f64");
                    Some(*self as f64)
                }
            }

            #[inline(always)]
            fn prefer_f64_filter_before_arithmetic() -> bool {
                crate::trace_dispatch!("hyperlimit", "float_scalar", "prefilter-policy");
                $prefer_filter
            }
        }
    };
}

impl_float_scalar!(f32, false);
impl_float_scalar!(f64, false);
