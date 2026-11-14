use crate::IntervalError;
use crate::bounds::{Bounds, set_relations};

mod sealed {
    pub trait Sealed {}
    macro_rules! seal {
        ($($t:ty),*) => { $(impl Sealed for $t {})* };
    }
    seal!(
        i8, i16, i32, i64, i128, isize, u8, u16, u32, u64, u128, usize
    );
}

/// Primitive integer endpoint family. Sealed to prevent unchecked order claims.
pub trait IntegerEndpoint: sealed::Sealed + Copy + Ord {
    /// Exact additive identity.
    const ZERO: Self;
}

macro_rules! endpoints {
    ($($t:ty),*) => { $(impl IntegerEndpoint for $t { const ZERO: Self = 0; })* };
}
endpoints!(
    i8, i16, i32, i64, i128, isize, u8, u16, u32, u64, u128, usize
);

/// Exact discrete integer set `{lower, ..., upper}`, or the empty set.
///
/// This is not a continuous real interval. Endpoints are exact primitive
/// integers; construction rejects reversed endpoints. Storage and relations
/// allocate nothing and require no `std`. There is no implicit midpoint or
/// scalar order. Serde encodes null for empty and an ordered pair otherwise,
/// rebuilding through checked construction on import.
///
/// Downstream endpoint implementations cannot assert certified semantics:
///
/// ```compile_fail
/// use hyperinterval::IntegerEndpoint;
/// #[derive(Clone, Copy, Eq, PartialEq, Ord, PartialOrd)]
/// struct UserEndpoint(i32);
/// impl IntegerEndpoint for UserEndpoint {
///     const ZERO: Self = Self(0);
/// }
/// ```
///
/// Set relations must be named; scalar ordering is intentionally unavailable:
///
/// ```compile_fail
/// use hyperinterval::IntegerRange;
/// let a = IntegerRange::<i32>::point(1);
/// let b = IntegerRange::<i32>::point(2);
/// let _ = a < b;
/// ```
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct IntegerRange<I: IntegerEndpoint> {
    bounds: Option<Bounds<I>>,
}

impl<I: IntegerEndpoint> IntegerRange<I> {
    pub(crate) fn from_bounds(bounds: Bounds<I>) -> Self {
        Self {
            bounds: Some(bounds),
        }
    }

    /// Constructs an exact nonempty range, rejecting reversed endpoints.
    pub fn new(lower: I, upper: I) -> Result<Self, IntervalError> {
        Ok(Self {
            bounds: Some(Bounds::new(lower, upper)?),
        })
    }

    /// Constructs the empty set without endpoint sentinels.
    pub fn empty() -> Self {
        Self { bounds: None }
    }

    /// Constructs a singleton discrete set.
    pub fn point(value: I) -> Self {
        Self {
            bounds: Some(Bounds::point(value)),
        }
    }

    set_relations!(#[inline(always)]);

    /// Converts endpoints exactly to another primitive width or signedness.
    ///
    /// Widening is lossless. A narrowing or signedness change that cannot
    /// represent either endpoint returns `Overflow`, never saturation.
    pub fn convert<J: IntegerEndpoint + TryFrom<I>>(
        &self,
    ) -> Result<IntegerRange<J>, IntervalError> {
        match self.bounds() {
            Some((lower, upper)) => IntegerRange::new(
                J::try_from(lower).map_err(|_| IntervalError::Overflow)?,
                J::try_from(upper).map_err(|_| IntervalError::Overflow)?,
            ),
            None => Ok(IntegerRange::empty()),
        }
    }
}

#[cfg(feature = "serde")]
impl<I: IntegerEndpoint + serde::Serialize> serde::Serialize for IntegerRange<I> {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.bounds().serialize(serializer)
    }
}

#[cfg(feature = "serde")]
impl<'de, I: IntegerEndpoint + serde::Deserialize<'de>> serde::Deserialize<'de>
    for IntegerRange<I>
{
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        match Option::<(I, I)>::deserialize(deserializer)? {
            None => Ok(Self::empty()),
            Some((lower, upper)) => Self::new(lower, upper).map_err(serde::de::Error::custom),
        }
    }
}
