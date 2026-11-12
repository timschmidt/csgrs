//! At most two normalized closed real components, with no union allocation.
use crate::{ExtendedInterval, FiniteRealInterval, SignSet};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
enum State<I: FiniteRealInterval> {
    Empty,
    One(ExtendedInterval<I>),
    #[cfg(feature = "dyadic")]
    Two([ExtendedInterval<I>; 2]),
}
/// A normalized union of at most two closed extended real intervals.
/// Components are nonempty, sorted and strictly separated. Touching components
/// merge. Infinity is a bound direction, never a member. The container itself
/// allocates nothing and is Copy when its backend is Copy. Two-component
/// construction and validated replay are currently provided for dyadic data.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct IntervalSet2<I: FiniteRealInterval> {
    state: State<I>,
}
impl<I: FiniteRealInterval> IntervalSet2<I> {
    /// Canonical empty real set.
    pub const fn empty() -> Self {
        Self {
            state: State::Empty,
        }
    }
    /// Adopts one connected real interval, canonicalizing its empty state.
    pub fn from_interval(interval: ExtendedInterval<I>) -> Self {
        if interval.is_empty() {
            Self::empty()
        } else {
            Self {
                state: State::One(interval),
            }
        }
    }
    /// Borrowed nonempty components in strictly increasing order.
    pub fn components(&self) -> &[ExtendedInterval<I>] {
        match &self.state {
            State::Empty => &[],
            State::One(x) => core::slice::from_ref(x),
            #[cfg(feature = "dyadic")]
            State::Two(x) => x,
        }
    }
    /// Whether no real members are represented.
    pub fn is_empty(&self) -> bool {
        matches!(self.state, State::Empty)
    }
    /// Whether exactly one real member is represented.
    pub fn is_singleton(&self) -> bool {
        matches!(&self.state,State::One(x) if x.is_singleton())
    }
    /// Exact possible signs of members, including Nonzero for separated rays.
    pub fn signs(&self) -> SignSet {
        self.components()
            .iter()
            .fold(SignSet::Empty, |signs, x| signs.union(x.signs()))
    }
}

#[cfg(feature = "dyadic")]
mod dyadic {
    use super::*;
    use crate::{DomainCoverage, DyadicBudget, DyadicInterval, IntervalError, RangeEnclosure};
    use num_bigint::BigInt;
    type Set = IntervalSet2<DyadicInterval>;
    type Extended = ExtendedInterval<DyadicInterval>;
    impl Set {
        /// Normalizes the exact union, with all comparison/merge work budgeted.
        /// Rounding performed by an earlier operation may close a gap; these
        /// are the actual represented closed components being normalized.
        pub fn new(
            first: Extended,
            second: Extended,
            budget: &mut DyadicBudget,
        ) -> Result<Self, IntervalError> {
            if first.is_empty() {
                return Ok(Self::from_interval(second));
            }
            if second.is_empty() {
                return Ok(Self::from_interval(first));
            }
            Ok(if first.strictly_left_of(&second, budget)? {
                Self {
                    state: State::Two([first, second]),
                }
            } else if second.strictly_left_of(&first, budget)? {
                Self {
                    state: State::Two([second, first]),
                }
            } else {
                Self::from_interval(first.hull(&second, budget)?)
            })
        }
        /// Exact reflection. Component order is reversed by the same proof.
        pub fn neg(&self, budget: &mut DyadicBudget) -> Result<Self, IntervalError> {
            Ok(match &self.state {
                State::Empty => Self::empty(),
                State::One(x) => Self::from_interval(x.neg(budget)?),
                State::Two([a, b]) => Self {
                    state: State::Two([b.neg(budget)?, a.neg(budget)?]),
                },
            })
        }
        /// Smallest connected closed enclosure. This explicitly fills any gap.
        pub fn hull(&self, budget: &mut DyadicBudget) -> Result<Extended, IntervalError> {
            match &self.state {
                State::Empty => Ok(Extended::empty()),
                State::One(x) => x.hull(&Extended::empty(), budget),
                State::Two([a, b]) => a.hull(b, budget),
            }
        }
        /// Exact membership of a finite dyadic point in either component.
        pub fn contains_point(
            &self,
            mantissa: &BigInt,
            exponent: i32,
            budget: &mut DyadicBudget,
        ) -> Result<bool, IntervalError> {
            for x in self.components() {
                if x.contains_point(mantissa, exponent, budget)? {
                    return Ok(true);
                }
            }
            Ok(false)
        }
        /// Exact set inclusion. A connected component cannot cross a real gap
        /// in the target; every source component must fit one target component.
        pub fn subset_of(
            &self,
            other: &Self,
            budget: &mut DyadicBudget,
        ) -> Result<bool, IntervalError> {
            for x in self.components() {
                let mut contained = false;
                for y in other.components() {
                    if x.subset_of(y, budget)? {
                        contained = true;
                        break;
                    }
                }
                if !contained {
                    return Ok(false);
                }
            }
            Ok(true)
        }
        /// Outward exponential of each component, then exact normalization.
        /// Coverage is locally Complete; keep earlier Partial domain receipts.
        pub fn exp(
            &self,
            precision: i32,
            budget: &mut DyadicBudget,
        ) -> Result<RangeEnclosure<Self>, IntervalError> {
            let range = match &self.state {
                State::Empty => Self::empty(),
                State::One(x) => Self::from_interval(x.exp(precision, budget)?.range),
                State::Two([a, b]) => Self::new(
                    a.exp(precision, budget)?.range,
                    b.exp(precision, budget)?.range,
                    budget,
                )?,
            };
            Ok(RangeEnclosure {
                range,
                domain: DomainCoverage::Complete,
            })
        }
    }
    #[cfg(feature = "serde")]
    impl<'de> serde::Deserialize<'de> for Set {
        fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
            use serde::de::Error;
            match State::<DyadicInterval>::deserialize(deserializer)? {
                State::Empty => Ok(Self::empty()),
                State::One(x) => Ok(Self::from_interval(x)),
                State::Two([a, b]) => {
                    // Validate canonical ordering without materializing a hull
                    // or an exponent-sized alignment. Parsing bounds remain
                    // caller-owned, as for the finite dyadic deserializer.
                    let mut budget = DyadicBudget::new(u64::MAX, u64::MAX);
                    if !a
                        .strictly_left_of(&b, &mut budget)
                        .map_err(D::Error::custom)?
                    {
                        return Err(D::Error::custom(
                            "split components must be nonempty, sorted and strictly disjoint",
                        ));
                    }
                    Ok(Self {
                        state: State::Two([a, b]),
                    })
                }
            }
        }
    }
}
#[cfg(feature = "serde")]
impl<I: FiniteRealInterval + serde::Serialize> serde::Serialize for IntervalSet2<I> {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.state.serialize(serializer)
    }
}
