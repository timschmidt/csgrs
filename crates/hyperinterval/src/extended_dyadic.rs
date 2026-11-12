use crate::{
    DomainCoverage, DyadicBudget, DyadicInterval, ExtendedInterval, ExtendedIntervalView as View,
    IntervalError, RangeEnclosure,
};
use core::cmp::Ordering;
use num_bigint::BigInt;
use num_traits::Zero;
type Extended = ExtendedInterval<DyadicInterval>;
#[derive(Clone, Copy)]
enum Bound<'a> {
    Below,
    Finite(&'a BigInt, i32),
    Above,
}
fn bounds(x: &Extended) -> Option<(Bound<'_>, Bound<'_>)> {
    Some(match x.view() {
        View::Empty => return None,
        View::Entire => (Bound::Below, Bound::Above),
        View::Finite(x) => {
            let (a, b, e) = x.bounds().expect("nonempty finite state");
            (Bound::Finite(a, e), Bound::Finite(b, e))
        }
        View::Below(x) => {
            let (_, b, e) = x.bounds().expect("singleton bound");
            (Bound::Below, Bound::Finite(b, e))
        }
        View::Above(x) => {
            let (a, _, e) = x.bounds().expect("singleton bound");
            (Bound::Finite(a, e), Bound::Above)
        }
    })
}
fn compare(
    a: Bound<'_>,
    b: Bound<'_>,
    budget: &mut DyadicBudget,
) -> Result<Ordering, IntervalError> {
    Ok(match (a, b) {
        (Bound::Below, Bound::Below) | (Bound::Above, Bound::Above) => Ordering::Equal,
        (Bound::Below, _) | (_, Bound::Above) => Ordering::Less,
        (Bound::Above, _) | (_, Bound::Below) => Ordering::Greater,
        (Bound::Finite(a, e), Bound::Finite(b, f)) => {
            crate::dyadic_order::compare(a, e, b, f, budget)?
        }
    })
}
fn point(a: &BigInt, e: i32, budget: &mut DyadicBudget) -> Result<DyadicInterval, IntervalError> {
    budget.reserve(a.bits().max(1), 2, false)?;
    DyadicInterval::point(a.clone(), e)
}
fn from_bounds(
    a: Bound<'_>,
    b: Bound<'_>,
    budget: &mut DyadicBudget,
) -> Result<Extended, IntervalError> {
    if compare(a, b, budget)?.is_gt() {
        return Ok(Extended::empty());
    }
    match (a, b) {
        (Bound::Below, Bound::Above) => Ok(Extended::entire()),
        (Bound::Below, Bound::Finite(b, f)) => Extended::below(point(b, f, budget)?),
        (Bound::Finite(a, e), Bound::Above) => Extended::above(point(a, e, budget)?),
        (Bound::Finite(a, e), Bound::Finite(b, f)) => {
            let range = if a.is_zero() || b.is_zero() {
                budget.reserve(a.bits().max(b.bits()).max(1), 2, false)?;
                DyadicInterval::new(a.clone(), b.clone(), if a.is_zero() { f } else { e })?
            } else {
                point(a, e, budget)?.hull(&point(b, f, budget)?, budget)?
            };
            Ok(Extended::from_finite(range))
        }
        _ => unreachable!("lower cannot be positive infinity; upper cannot be negative infinity"),
    }
}
pub(crate) fn closed_bounds(
    a: &BigInt,
    e: i32,
    b: &BigInt,
    f: i32,
    budget: &mut DyadicBudget,
) -> Result<Extended, IntervalError> {
    from_bounds(Bound::Finite(a, e), Bound::Finite(b, f), budget)
}
impl Extended {
    /// Copies an empty/finite range into its finite carrier. Unbounded images
    /// fail explicitly; no infinite end is replaced by a large finite value.
    pub fn finite_range(&self, budget: &mut DyadicBudget) -> Result<DyadicInterval, IntervalError> {
        match self.view() {
            View::Empty => Ok(DyadicInterval::empty()),
            View::Finite(x) => {
                let (a, b, _) = x.bounds().expect("nonempty finite state");
                budget.reserve(a.bits().max(b.bits()), 2, false)?;
                Ok(x.clone())
            }
            _ => Err(IntervalError::UnboundedImage),
        }
    }
    /// Exact reflected real set, including reversal of unbounded directions.
    pub fn neg(&self, budget: &mut DyadicBudget) -> Result<Self, IntervalError> {
        match self.view() {
            View::Empty => Ok(Self::empty()),
            View::Entire => Ok(Self::entire()),
            View::Finite(x) => Ok(Self::from_finite(x.neg(budget)?)),
            View::Below(x) => Self::above(x.neg(budget)?),
            View::Above(x) => Self::below(x.neg(budget)?),
        }
    }
    /// Smallest closed interval containing both represented sets.
    pub fn hull(&self, other: &Self, budget: &mut DyadicBudget) -> Result<Self, IntervalError> {
        let (a, b, c, d) = match (bounds(self), bounds(other)) {
            (None, None) => return Ok(Self::empty()),
            (Some((a, b)), None) | (None, Some((a, b))) => return from_bounds(a, b, budget),
            (Some((a, b)), Some((c, d))) => (a, b, c, d),
        };
        from_bounds(
            if compare(a, c, budget)?.is_gt() { c } else { a },
            if compare(b, d, budget)?.is_lt() { d } else { b },
            budget,
        )
    }
    /// Exact intersection, with canonical empty output when disjoint.
    pub fn intersection(
        &self,
        other: &Self,
        budget: &mut DyadicBudget,
    ) -> Result<Self, IntervalError> {
        let (Some((a, b)), Some((c, d))) = (bounds(self), bounds(other)) else {
            return Ok(Self::empty());
        };
        from_bounds(
            if compare(a, c, budget)?.is_lt() { c } else { a },
            if compare(b, d, budget)?.is_gt() { d } else { b },
            budget,
        )
    }
    /// Exact set inclusion with explicitly charged finite endpoint comparisons.
    pub fn subset_of(
        &self,
        other: &Self,
        budget: &mut DyadicBudget,
    ) -> Result<bool, IntervalError> {
        let Some((a, b)) = bounds(self) else {
            return Ok(true);
        };
        let Some((c, d)) = bounds(other) else {
            return Ok(false);
        };
        Ok(!compare(c, a, budget)?.is_gt() && !compare(b, d, budget)?.is_gt())
    }
    /// Exact membership for a finite dyadic point; infinity is never a member.
    pub fn contains_point(
        &self,
        mantissa: &BigInt,
        exponent: i32,
        budget: &mut DyadicBudget,
    ) -> Result<bool, IntervalError> {
        let Some((a, b)) = bounds(self) else {
            return Ok(false);
        };
        let point = Bound::Finite(mantissa, exponent);
        Ok(!compare(a, point, budget)?.is_gt() && !compare(point, b, budget)?.is_gt())
    }
    /// Outward exponential image, retaining unbounded ends explicitly.
    /// A lower end at negative infinity has infimum zero; zero is included in
    /// the closed enclosure. Domain coverage is Complete for all source sets.
    pub fn exp(
        &self,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<RangeEnclosure<Self>, IntervalError> {
        let range = match self.view() {
            View::Empty => Self::empty(),
            View::Entire => Self::above(DyadicInterval::point(BigInt::zero(), 0)?)?,
            View::Finite(x) => Self::from_finite(x.exp(precision, budget)?.range),
            View::Below(x) => {
                let result = x.exp(precision, budget)?.range;
                let (_, b, e) = result.bounds().expect("nonempty point exponential");
                budget.reserve(b.bits().max(1), 1, false)?;
                Self::from_finite(DyadicInterval::new(BigInt::zero(), b.clone(), e)?)
            }
            View::Above(x) => {
                let result = x.exp(precision, budget)?.range;
                let (a, _, e) = result.bounds().expect("nonempty point exponential");
                Self::above(point(a, e, budget)?)?
            }
        };
        Ok(RangeEnclosure {
            range,
            domain: DomainCoverage::Complete,
        })
    }
}
