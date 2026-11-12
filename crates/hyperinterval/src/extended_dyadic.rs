use crate::{
    DomainCoverage, DyadicBudget, DyadicInterval, ExtendedInterval, ExtendedIntervalView as View,
    IntervalError, IntervalSet2, RangeEnclosure, SignSet,
};
use core::cmp::Ordering;
use num_bigint::BigInt;
use num_integer::Integer;
use num_traits::{One, Zero};
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
    /// Whether every member is strictly less than every member of `other`.
    /// Empty operands return false; touching endpoints are not strictly ordered.
    pub fn strictly_left_of(
        &self,
        other: &Self,
        budget: &mut DyadicBudget,
    ) -> Result<bool, IntervalError> {
        let (Some((_, b)), Some((c, _))) = (bounds(self), bounds(other)) else {
            return Ok(false);
        };
        Ok(compare(b, c, budget)?.is_lt())
    }
    /// Closed outward reciprocal image of the defined source, retaining both
    /// branches around zero. Zero-only input returns EmptyDomain; crossing or
    /// touching zero reports Partial. Unattained zero limits can enter the
    /// closed enclosure. Empty maps to empty with Complete local coverage.
    pub fn reciprocal(
        &self,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<RangeEnclosure<IntervalSet2<DyadicInterval>>, IntervalError> {
        if self.signs() == SignSet::Zero {
            return Err(IntervalError::EmptyDomain);
        }
        if self.is_empty() {
            return Ok(RangeEnclosure {
                range: IntervalSet2::empty(),
                domain: DomainCoverage::Complete,
            });
        }
        budget.reserve(1, 2, false)?;
        Self::from_finite(DyadicInterval::point(BigInt::one(), 0)?).div(self, precision, budget)
    }
    /// Outward quotient image on the defined real Cartesian domain, with at
    /// most two normalized closed components. A zero-only denominator fails
    /// before an empty-numerator shortcut. Other empty operands yield empty.
    /// A nonempty zero-containing denominator gives Partial local coverage.
    /// Finite bounds are rounded directly from exact quotients, not via a
    /// rounded reciprocal. All numerical work uses the caller's shared budget.
    pub fn div(
        &self,
        other: &Self,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<RangeEnclosure<IntervalSet2<DyadicInterval>>, IntervalError> {
        let signs = other.signs();
        if signs == SignSet::Zero {
            return Err(IntervalError::EmptyDomain);
        }
        if self.is_empty() || other.is_empty() {
            return Ok(RangeEnclosure {
                range: IntervalSet2::empty(),
                domain: DomainCoverage::Complete,
            });
        }
        let positive = if signs.contains_positive() {
            positive_quotient(self, other, precision, budget)?
        } else {
            Self::empty()
        };
        let negative = if signs.contains_negative() {
            positive_quotient(&self.neg(budget)?, &other.neg(budget)?, precision, budget)?
        } else {
            Self::empty()
        };
        Ok(RangeEnclosure {
            range: IntervalSet2::new(negative, positive, budget)?,
            domain: if signs.contains_zero() {
                DomainCoverage::Partial
            } else {
                DomainCoverage::Complete
            },
        })
    }
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

// A selected quotient extremum. Infinite values describe bound directions.
enum Limit {
    Below,
    Finite(BigInt, i32),
    Above,
}
impl Limit {
    fn bound(&self) -> Bound<'_> {
        match self {
            Self::Below => Bound::Below,
            Self::Above => Bound::Above,
            Self::Finite(n, e) => Bound::Finite(n, *e),
        }
    }
}
// Restrict a denominator to its strictly positive part. The lower bound zero
// is a pole limit, not a permitted denominator member. Monotonicity selects
// each exact quotient extremum before rounding or materializing coordinates.
fn positive_quotient(
    numerator: &Extended,
    denominator: &Extended,
    precision: i32,
    budget: &mut DyadicBudget,
) -> Result<Extended, IntervalError> {
    let (a, b) = bounds(numerator).expect("nonempty numerator");
    let (c, d) = bounds(denominator).expect("positive denominator part exists");
    let zero = BigInt::zero();
    let z = Bound::Finite(&zero, 0);
    let c = if compare(c, z, budget)?.is_lt() { z } else { c };
    let lower_divisor = if compare(a, z, budget)?.is_lt() { c } else { d };
    let upper_divisor = if compare(b, z, budget)?.is_gt() { c } else { d };
    let lower = quotient_bound(a, lower_divisor, true, precision, budget)?;
    let upper = quotient_bound(b, upper_divisor, false, precision, budget)?;
    from_bounds(lower.bound(), upper.bound(), budget)
}
fn quotient_bound(
    numerator: Bound<'_>,
    denominator: Bound<'_>,
    lower: bool,
    precision: i32,
    budget: &mut DyadicBudget,
) -> Result<Limit, IntervalError> {
    let (n, e) = match numerator {
        Bound::Below => return Ok(Limit::Below),
        Bound::Above => return Ok(Limit::Above),
        Bound::Finite(n, e) => (n, e),
    };
    let (d, f) = match denominator {
        Bound::Above => return Ok(Limit::Finite(BigInt::zero(), 0)),
        Bound::Finite(d, f) => (d, f),
        Bound::Below => unreachable!("positive branch divisor is nonnegative"),
    };
    if n.is_zero() {
        return Ok(Limit::Finite(BigInt::zero(), 0));
    }
    if d.is_zero() {
        return Ok(if n < &BigInt::zero() {
            Limit::Below
        } else {
            Limit::Above
        });
    }
    budget.reserve(n.bits().max(d.bits()).max(1), 1, false)?;
    let scale = i64::from(e) - i64::from(f);
    if n.magnitude() == d.magnitude()
        && i64::from(precision) <= scale
        && let Ok(scale) = i32::try_from(scale)
    {
        budget.reserve(1, 1, false)?;
        return Ok(Limit::Finite(
            if n < &BigInt::zero() {
                (-1).into()
            } else {
                1.into()
            },
            scale,
        ));
    }
    let shift = scale - i64::from(precision);
    if i128::from(n.bits()) + i128::from(shift) < i128::from(d.bits()) {
        // Strictly sub-grid magnitude. Avoid expanding an irrelevant gap.
        budget.reserve(1, 1, false)?;
        let negative = n < &BigInt::zero();
        let bound = match (negative, lower) {
            (true, true) => -1,
            (false, false) => 1,
            _ => 0,
        };
        return Ok(Limit::Finite(bound.into(), precision));
    }
    // Retain exact dyadic quotients before expanding the requested grid.
    // Reserve deterministic GCD/reduction work and owned results before the
    // backend call; these accounting units are not CPU or heap-byte bounds.
    let bits = n.bits().max(d.bits());
    budget.reserve(bits, bits.saturating_mul(2).saturating_add(5), true)?;
    let gcd = n.gcd(d);
    let n = n / &gcd;
    let d = d / gcd;
    let twos = d.trailing_zeros().expect("positive reduced denominator");
    if twos.checked_add(1) == Some(d.bits()) {
        let exact_scale = scale
            .checked_sub(i64::try_from(twos).map_err(|_| IntervalError::Overflow)?)
            .ok_or(IntervalError::Overflow)?;
        if i64::from(precision) <= exact_scale
            && let Ok(exact_scale) = i32::try_from(exact_scale)
        {
            return Ok(Limit::Finite(n, exact_scale));
        }
    }
    let (a, b) = crate::dyadic::scaled_quotient(&n, &d, shift, budget)?;
    Ok(Limit::Finite(if lower { a } else { b }, precision))
}
