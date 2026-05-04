//! Scalar capabilities used by geometry predicates.

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
    pub sign: Option<Sign>,
    pub exact_zero: Option<bool>,
    pub provably_nonzero: Option<bool>,
    pub exact: Option<bool>,
    pub rational_only: Option<bool>,
    pub magnitude: Option<MagnitudeBounds>,
}

impl ScalarFacts {
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
    fn scalar_facts(&self) -> ScalarFacts {
        ScalarFacts::default()
    }

    fn known_sign(&self) -> SignKnowledge {
        self.scalar_facts().sign_knowledge()
    }

    fn is_exact_zero(&self) -> Option<bool> {
        self.scalar_facts().exact_zero
    }

    fn is_provably_nonzero(&self) -> Option<bool> {
        self.scalar_facts().provably_nonzero
    }

    fn is_rational_only(&self) -> Option<bool> {
        self.scalar_facts().rational_only
    }

    fn magnitude_bounds(&self) -> Option<MagnitudeBounds> {
        self.scalar_facts().magnitude
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
    + core::ops::Add<Output = Self>
    + core::ops::Sub<Output = Self>
    + core::ops::Mul<Output = Self>
{
    /// Convert to `f64` for cheap filters. Returning `None` disables that stage.
    fn to_f64(&self) -> Option<f64>;
}

macro_rules! impl_float_scalar {
    ($ty:ty) => {
        impl StructuralScalar for $ty {
            fn scalar_facts(&self) -> ScalarFacts {
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
            fn to_f64(&self) -> Option<f64> {
                if self.is_nan() {
                    None
                } else {
                    Some(*self as f64)
                }
            }
        }
    };
}

impl_float_scalar!(f32);
impl_float_scalar!(f64);
