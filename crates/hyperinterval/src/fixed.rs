use crate::IntervalError;
use crate::bounds::{Bounds, set_relations};

mod sealed {
    pub trait Sealed {}
    impl Sealed for i32 {}
    impl Sealed for i64 {}
}

/// Signed fixed-point endpoint family, sealed to i32 and i64.
pub trait FixedEndpoint: sealed::Sealed + Copy + Ord {
    /// Number of bits including the sign bit.
    const BITS: u32;
    /// Exact zero lattice numerator.
    const ZERO: Self;
}

impl FixedEndpoint for i32 {
    const BITS: u32 = 32;
    const ZERO: Self = 0;
}
impl FixedEndpoint for i64 {
    const BITS: u32 = 64;
    const ZERO: Self = 0;
}

/// Closed real interval `[lower / 2^FRAC, upper / 2^FRAC]`, or empty.
///
/// The endpoints are exact lattice numerators, not floating-point estimates.
/// All constructors, including `empty`, require `FRAC < I::BITS`. The i32
/// family includes Q2.30; i64 includes Q31.32. Storage and set relations are
/// allocation-free `no_std`. Empty is a separate state, never reversed bounds.
/// Serde uses null or a numerator pair and validates scale and order on import.
/// Hyperlimit, not this carrier, owns any final scalar predicate decision.
///
/// Only qualified endpoint widths can instantiate the carrier:
///
/// ```compile_fail
/// use hyperinterval::FixedInterval;
/// let _ = FixedInterval::<u64, 32>::point(0);
/// ```
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FixedInterval<I: FixedEndpoint, const FRAC: u32> {
    bounds: Option<Bounds<I>>,
}

impl<I: FixedEndpoint, const FRAC: u32> FixedInterval<I, FRAC> {
    // Arithmetic callers establish endpoint order before reaching this path.
    // Scale validation remains mandatory; public ingress always uses new().
    pub(crate) fn from_ordered(lower: I, upper: I) -> Result<Self, IntervalError> {
        Self::validate_scale()?;
        Ok(Self {
            bounds: Some(Bounds::ordered(lower, upper)),
        })
    }

    fn validate_scale() -> Result<(), IntervalError> {
        if FRAC < I::BITS {
            Ok(())
        } else {
            Err(IntervalError::InvalidScale)
        }
    }

    /// Constructs a nonempty interval from exact lattice numerators.
    ///
    /// Rejects unsupported scales before checking endpoint order.
    pub fn new(lower: I, upper: I) -> Result<Self, IntervalError> {
        Self::validate_scale()?;
        Ok(Self {
            bounds: Some(Bounds::new(lower, upper)?),
        })
    }

    /// Constructs an empty set after validating the type's fixed-point scale.
    pub fn empty() -> Result<Self, IntervalError> {
        Self::validate_scale()?;
        Ok(Self { bounds: None })
    }

    /// Constructs a singleton from an exact lattice numerator.
    pub fn point(value: I) -> Result<Self, IntervalError> {
        Self::validate_scale()?;
        Ok(Self {
            bounds: Some(Bounds::point(value)),
        })
    }

    set_relations!();

    /// Converts endpoint storage exactly, retaining the fixed-point scale.
    ///
    /// The destination scale is validated even for empty sets. An endpoint
    /// outside the destination range returns `Overflow`.
    pub fn convert<J: FixedEndpoint + TryFrom<I>>(
        &self,
    ) -> Result<FixedInterval<J, FRAC>, IntervalError> {
        let empty = FixedInterval::<J, FRAC>::empty()?;
        match self.bounds() {
            // Successful integer conversion is exact and preserves order.
            Some((lower, upper)) => FixedInterval::from_ordered(
                J::try_from(lower).map_err(|_| IntervalError::Overflow)?,
                J::try_from(upper).map_err(|_| IntervalError::Overflow)?,
            ),
            None => Ok(empty),
        }
    }
}

#[cfg(feature = "serde")]
impl<I: FixedEndpoint + serde::Serialize, const FRAC: u32> serde::Serialize
    for FixedInterval<I, FRAC>
{
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.bounds().serialize(serializer)
    }
}

#[cfg(feature = "serde")]
impl<'de, I: FixedEndpoint + serde::Deserialize<'de>, const FRAC: u32> serde::Deserialize<'de>
    for FixedInterval<I, FRAC>
{
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        match Option::<(I, I)>::deserialize(deserializer)? {
            None => Self::empty(),
            Some((lower, upper)) => Self::new(lower, upper),
        }
        .map_err(serde::de::Error::custom)
    }
}

/// An exact nonnegative width `numerator / 2^FRAC`, including a full signed range.
///
/// Unlike a signed interval endpoint, this stores every u64 lattice distance.
/// Equality and ordering compare mathematical values within the fixed scale.
/// Serialization stores the numerator; the receiving type supplies the scale.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct FixedWidth<const FRAC: u32> {
    numerator: u64,
}
impl<const FRAC: u32> FixedWidth<FRAC> {
    /// Constructs a width, requiring the shared fixed families' `FRAC < 64`.
    pub fn new(numerator: u64) -> Result<Self, IntervalError> {
        if FRAC >= 64 {
            return Err(IntervalError::InvalidScale);
        }
        Ok(Self { numerator })
    }
    /// Exact distance in units of `2^-FRAC`, with no signed narrowing.
    pub const fn numerator(&self) -> u64 {
        self.numerator
    }
    /// Number of binary fractional places in this value's type.
    pub const fn fraction_bits(&self) -> u32 {
        FRAC
    }
    /// Exact singleton projection into the signed i64 fixed family, or typed
    /// overflow. Failure leaves this exact unsigned width available for use.
    pub fn to_interval(&self) -> Result<FixedInterval<i64, FRAC>, IntervalError> {
        let numerator = i64::try_from(self.numerator).map_err(|_| IntervalError::Overflow)?;
        FixedInterval::<i64, FRAC>::point(numerator)
    }
}
#[cfg(feature = "serde")]
impl<const F: u32> serde::Serialize for FixedWidth<F> {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.numerator.serialize(serializer)
    }
}
#[cfg(feature = "serde")]
impl<'de, const F: u32> serde::Deserialize<'de> for FixedWidth<F> {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        Self::new(u64::deserialize(deserializer)?).map_err(serde::de::Error::custom)
    }
}

#[cfg(any(feature = "dyadic", feature = "rational"))]
impl<I: FixedEndpoint + Into<i64>, const FRAC: u32> FixedInterval<I, FRAC> {
    /// Converts both exact lattice endpoints directly to a normalized dyadic interval.
    ///
    /// Before materializing the mantissas, reserves two linear stages at their
    /// maximum magnitude bit length. Empty requires no work.
    #[cfg(feature = "dyadic")]
    pub fn to_dyadic(
        &self,
        budget: &mut crate::DyadicBudget,
    ) -> Result<crate::DyadicInterval, IntervalError> {
        let Some((lower, upper)) = self.bounds() else {
            return Ok(crate::DyadicInterval::empty());
        };
        let (lower, upper) = (lower.into(), upper.into());
        let magnitude = lower.unsigned_abs().max(upper.unsigned_abs());
        budget.reserve(u64::from(u64::BITS - magnitude.leading_zeros()), 2, false)?;
        crate::DyadicInterval::new(lower.into(), upper.into(), -(FRAC as i32))
    }

    /// Converts the whole closed interval exactly, using the lattice denominator
    /// for both rational endpoints. Empty maps to empty.
    #[cfg(feature = "rational")]
    pub fn to_rational(&self) -> crate::RationalInterval {
        let Some((lower, upper)) = self.bounds() else {
            return crate::RationalInterval::empty();
        };
        // FixedInterval construction proves FRAC < I::BITS <= 64.
        let denominator = 1_u64 << FRAC;
        let endpoint = |n: I| {
            hyperreal::Rational::fraction(n.into(), denominator)
                .expect("fixed-point denominator is a positive power of two")
        };
        let a = endpoint(lower);
        if lower == upper {
            crate::RationalInterval::point(a)
        } else {
            crate::RationalInterval::ordered(a, endpoint(upper))
        }
    }
}
