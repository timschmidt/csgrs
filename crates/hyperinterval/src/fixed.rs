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
            Some((lower, upper)) => FixedInterval::new(
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
