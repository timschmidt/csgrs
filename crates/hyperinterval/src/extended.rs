//! Closed real enclosures with explicit unbounded ends, never infinite members.
#[cfg(feature = "dyadic")]
mod dyadic_overlap;

use crate::{IntervalError, SignSet};

mod sealed {
    pub trait Sealed {}
}
/// A sealed finite real-interval backend. Integer sets and proposals are excluded.
///
/// Downstream types cannot claim certified endpoint semantics through this trait.
/// ```compile_fail
/// use hyperinterval::{FiniteRealInterval,SignSet};
/// struct Forged;
/// impl FiniteRealInterval for Forged {
///     fn is_empty(&self)->bool { false }
///     fn is_singleton(&self)->bool { true }
///     fn signs(&self)->SignSet { SignSet::Zero }
/// }
/// ```
pub trait FiniteRealInterval: sealed::Sealed {
    /// Whether the finite real set is empty.
    fn is_empty(&self) -> bool;
    /// Whether the finite real set contains exactly one point.
    fn is_singleton(&self) -> bool;
    /// Possible signs of represented real members.
    fn signs(&self) -> SignSet;
}
macro_rules! backend {
    ($feature:literal,$ty:ty) => {
        #[cfg(feature=$feature)]
        impl sealed::Sealed for $ty {}
        #[cfg(feature=$feature)]
        impl FiniteRealInterval for $ty {
            fn is_empty(&self) -> bool {
                self.is_empty()
            }
            fn is_singleton(&self) -> bool {
                self.is_singleton()
            }
            fn signs(&self) -> SignSet {
                self.signs()
            }
        }
    };
}
backend!("rational", crate::RationalInterval);
backend!("dyadic", crate::DyadicInterval);
backend!("float-enclosure", crate::F32Enclosure);
backend!("float-enclosure", crate::F64Enclosure);
#[cfg(feature = "fixed")]
impl<T: crate::FixedEndpoint, const F: u32> sealed::Sealed for crate::FixedInterval<T, F> {}
#[cfg(feature = "fixed")]
impl<T: crate::FixedEndpoint, const F: u32> FiniteRealInterval for crate::FixedInterval<T, F> {
    fn is_empty(&self) -> bool {
        self.is_empty()
    }
    fn is_singleton(&self) -> bool {
        self.is_singleton()
    }
    fn signs(&self) -> SignSet {
        self.signs()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
enum State<I> {
    Empty,
    Finite(I),
    Below(I),
    Above(I),
    Entire,
}

/// An empty, finite, one-sided unbounded, or entire closed real enclosure.
/// Finite ends are included; infinity is a bound direction, never a member.
/// Ray bounds must be singleton intervals. Construction and replay validate
/// that invariant through the sealed finite backend.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ExtendedInterval<I: FiniteRealInterval> {
    state: State<I>,
}

/// Borrowed inspection of an extended interval. A view is not a constructor.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExtendedIntervalView<'a, I> {
    /// The empty set.
    Empty,
    /// A nonempty finite interval.
    Finite(&'a I),
    /// All real values at or below this singleton upper bound.
    Below(&'a I),
    /// All real values at or above this singleton lower bound.
    Above(&'a I),
    /// All real values.
    Entire,
}
impl<I: FiniteRealInterval> ExtendedInterval<I> {
    /// Canonical empty real set.
    pub const fn empty() -> Self {
        Self {
            state: State::Empty,
        }
    }
    /// Entire real line, with no infinite members.
    pub const fn entire() -> Self {
        Self {
            state: State::Entire,
        }
    }
    /// Adopts the exact finite carrier, canonicalizing its empty state.
    pub fn from_finite(range: I) -> Self {
        if range.is_empty() {
            Self::empty()
        } else {
            Self {
                state: State::Finite(range),
            }
        }
    }
    /// Closed ray at or below a finite singleton; empty/nonpoint bounds fail.
    pub fn below(upper: I) -> Result<Self, IntervalError> {
        if !upper.is_singleton() {
            return Err(IntervalError::NonSingletonEndpoint);
        }
        Ok(Self {
            state: State::Below(upper),
        })
    }
    /// Closed ray at or above a finite singleton; empty/nonpoint bounds fail.
    pub fn above(lower: I) -> Result<Self, IntervalError> {
        if !lower.is_singleton() {
            return Err(IntervalError::NonSingletonEndpoint);
        }
        Ok(Self {
            state: State::Above(lower),
        })
    }
    /// Whether no real members are represented.
    pub fn is_empty(&self) -> bool {
        matches!(self.state, State::Empty)
    }
    /// Whether all real values are represented.
    pub fn is_entire(&self) -> bool {
        matches!(self.state, State::Entire)
    }
    /// Whether exactly one real member is represented.
    pub fn is_singleton(&self) -> bool {
        matches!(&self.state,State::Finite(x) if x.is_singleton())
    }
    /// Borrowed nonempty finite carrier; inspect `view` to distinguish other states.
    pub fn as_finite(&self) -> Option<&I> {
        if let State::Finite(x) = &self.state {
            Some(x)
        } else {
            None
        }
    }
    /// Explicit finite/unbounded/empty state inspection.
    pub fn view(&self) -> ExtendedIntervalView<'_, I> {
        match &self.state {
            State::Empty => ExtendedIntervalView::Empty,
            State::Finite(x) => ExtendedIntervalView::Finite(x),
            State::Below(x) => ExtendedIntervalView::Below(x),
            State::Above(x) => ExtendedIntervalView::Above(x),
            State::Entire => ExtendedIntervalView::Entire,
        }
    }
    /// Possible signs of real members; this is not a scalar equality decision.
    pub fn signs(&self) -> SignSet {
        match &self.state {
            State::Empty => SignSet::Empty,
            State::Entire => SignSet::Mixed,
            State::Finite(x) => x.signs(),
            State::Below(x) => {
                if x.signs().contains_positive() {
                    SignSet::Mixed
                } else {
                    x.signs().union(SignSet::Negative)
                }
            }
            State::Above(x) => {
                if x.signs().contains_negative() {
                    SignSet::Mixed
                } else {
                    x.signs().union(SignSet::Positive)
                }
            }
        }
    }
}
#[cfg(feature = "serde")]
impl<I: FiniteRealInterval + serde::Serialize> serde::Serialize for ExtendedInterval<I> {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.state.serialize(serializer)
    }
}
#[cfg(feature = "serde")]
impl<'de, I: FiniteRealInterval + serde::Deserialize<'de>> serde::Deserialize<'de>
    for ExtendedInterval<I>
{
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        use serde::de::Error;
        match State::<I>::deserialize(deserializer)? {
            State::Empty => Ok(Self::empty()),
            State::Entire => Ok(Self::entire()),
            State::Finite(x) => Ok(Self::from_finite(x)),
            State::Below(x) => Self::below(x).map_err(D::Error::custom),
            State::Above(x) => Self::above(x).map_err(D::Error::custom),
        }
    }
}
