//! Budgeted monotone arctangent, with exact reductions and alternating tails.
use crate::dyadic::scaled_quotient;
use crate::{
    DomainCoverage, DyadicBudget, DyadicInterval, ExtendedInterval, ExtendedIntervalView as View,
    IntervalError, RangeEnclosure,
};
use num_bigint::BigInt;
use num_traits::{One, Signed, Zero};
type Image = RangeEnclosure<DyadicInterval>;

impl DyadicInterval {
    /// Tight outward principal arctangent on the grid `2^precision`.
    /// Empty maps to empty; domain coverage is always Complete. Exact source
    /// reduction, alternating-series bounds and Machin's identity share the
    /// caller's size/work budget. Exhaustion publishes no partial image.
    /// This is deterministic accounting, not a heap-byte or cancellation bound.
    pub fn atan(&self, precision: i32, budget: &mut DyadicBudget) -> Result<Image, IntervalError> {
        let Some((a, b, e)) = self.bounds() else {
            return complete(Self::empty());
        };
        budget.reserve(1, 1, false)?;
        let one = BigInt::one();
        let lower = endpoint(a, &one, e, precision, budget)?;
        if a == b {
            return complete(lower);
        }
        assemble(lower, endpoint(b, &one, e, precision, budget)?, budget)
    }
}

#[cfg(feature = "rational")]
impl crate::RationalInterval {
    /// Tight outward principal arctangent with dyadic spacing `2^precision`.
    /// Requires rational+dyadic. The exact rational source remains authoritative
    /// through every refinement attempt; local coverage is Complete.
    pub fn atan(&self, precision: i32, budget: &mut DyadicBudget) -> Result<Image, IntervalError> {
        let Some((a, b)) = self.bounds() else {
            return complete(DyadicInterval::empty());
        };
        let point = |x: &hyperreal::Rational, budget: &mut DyadicBudget| {
            budget.reserve(x.numerator().bits().max(x.denominator().bits()), 2, false)?;
            endpoint(
                &BigInt::from_biguint(x.sign(), x.numerator().clone()),
                &BigInt::from(x.denominator().clone()),
                0,
                precision,
                budget,
            )
        };
        let lower = point(a, budget)?;
        if a == b {
            return complete(lower);
        }
        assemble(lower, point(b, budget)?, budget)
    }
}

impl ExtendedInterval<DyadicInterval> {
    /// Closed principal-arctangent enclosure, using +/-pi/2 as unbounded-end
    /// limits. The finite image may include unattained limit values through
    /// outward rounding. Coverage is locally Complete; retain earlier receipts.
    pub fn atan(&self, precision: i32, budget: &mut DyadicBudget) -> Result<Image, IntervalError> {
        match self.view() {
            View::Empty => complete(DyadicInterval::empty()),
            View::Finite(x) => x.atan(precision, budget),
            View::Entire => {
                let limit = half_pi(precision, budget)?;
                assemble(limit.neg(budget)?, limit, budget)
            }
            View::Below(x) => assemble(
                half_pi(precision, budget)?.neg(budget)?,
                x.atan(precision, budget)?.range,
                budget,
            ),
            View::Above(x) => assemble(
                x.atan(precision, budget)?.range,
                half_pi(precision, budget)?,
                budget,
            ),
        }
    }
}

fn complete(range: DyadicInterval) -> Result<Image, IntervalError> {
    Ok(RangeEnclosure {
        range,
        domain: DomainCoverage::Complete,
    })
}
fn assemble(
    lower: DyadicInterval,
    upper: DyadicInterval,
    budget: &mut DyadicBudget,
) -> Result<Image, IntervalError> {
    let (a, _, e) = lower.bounds().expect("nonempty monotone endpoint image");
    let (_, b, f) = upper.bounds().expect("nonempty monotone endpoint image");
    complete(crate::extended_dyadic::closed_bounds(a, e, b, f, budget)?.finite_range(budget)?)
}

// A positive nonzero arctangent is strictly between zero and min(x, 2).
// These inequalities also handle extreme scales without materializing them.
fn endpoint(
    n: &BigInt,
    d: &BigInt,
    e: i32,
    precision: i32,
    budget: &mut DyadicBudget,
) -> Result<DyadicInterval, IntervalError> {
    budget.reserve(n.bits().max(d.bits()).max(1), 2, false)?;
    if n.is_zero() {
        return DyadicInterval::point(BigInt::zero(), 0);
    }
    let negative = n.is_negative();
    let magnitude = n.abs();
    if precision >= 1 || !crate::dyadic_order::compare(&magnitude, e, d, precision, budget)?.is_gt()
    {
        return if negative {
            DyadicInterval::new((-1).into(), BigInt::zero(), precision)
        } else {
            DyadicInterval::new(BigInt::zero(), BigInt::one(), precision)
        };
    }
    let inverse = crate::dyadic_order::compare(&magnitude, e, d, 0, budget)?.is_gt();
    let near_one = if inverse {
        crate::dyadic_order::compare(&magnitude, e, d, 1, budget)?.is_lt()
    } else {
        crate::dyadic_order::compare(&magnitude, e, d, -1, budget)?.is_gt()
    };
    let (n, d, e) = if inverse {
        (d, &magnitude, -i64::from(e))
    } else {
        (&magnitude, d, i64::from(e))
    };
    let mut working = precision
        .min(-4)
        .checked_sub(12)
        .ok_or(IntervalError::Overflow)?;
    loop {
        let shift = e - i64::from(working);
        let (lo, hi) = if i128::from(n.bits()) + i128::from(shift) < i128::from(d.bits()) {
            // The positive reduced argument is below a working-grid unit.
            budget.reserve(1, 2, false)?;
            (BigInt::zero(), BigInt::one())
        } else {
            scaled_quotient(n, d, shift, budget)?
        };
        let mut argument = DyadicInterval::new(lo, hi, working)?;
        if near_one {
            budget.reserve(1, 1, false)?;
            let one = DyadicInterval::point(BigInt::one(), 0)?;
            argument =
                one.sub(&argument, budget)?
                    .div(&one.add(&argument, budget)?, working, budget)?;
        }
        // The exact argument is in [0,1/2]; reductions preserve this bound
        // after outward grid rounding because 1/2 is on the working grid.
        let small = series(&argument, working, budget)?;
        let value = if near_one {
            let quarter = quarter_pi(working, budget)?;
            if inverse {
                quarter.add(&small, budget)?
            } else {
                quarter.sub(&small, budget)?
            }
        } else if inverse {
            budget.reserve(2, 1, false)?;
            quarter_pi(working, budget)?
                .mul(&DyadicInterval::point(2.into(), 0)?, budget)?
                .sub(&small, budget)?
        } else {
            small
        };
        // Exact monotonicity supplies the source-side sign despite cancellation.
        let value = value.max_image(&DyadicInterval::point(BigInt::zero(), 0)?, budget)?;
        if let Some(rounded) = tight(&value, precision, budget)? {
            return if negative {
                rounded.neg(budget)
            } else {
                Ok(rounded)
            };
        }
        working = working.checked_mul(2).ok_or(IntervalError::Overflow)?;
    }
}

fn tight(
    value: &DyadicInterval,
    precision: i32,
    budget: &mut DyadicBudget,
) -> Result<Option<DyadicInterval>, IntervalError> {
    let rounded = value.regrid(precision, budget)?;
    let (a, b, e) = rounded.bounds().expect("nonempty arctangent bound");
    budget.reserve(a.bits().max(b.bits()).saturating_add(1), 1, false)?;
    let gap = b - a;
    Ok(if gap.is_zero() || (gap.is_one() && e == precision) {
        Some(rounded)
    } else {
        None
    })
}

// Machin: pi/4 = 4 atan(1/5) - atan(1/239). Both arguments are in
// the alternating-series domain. This does not use a global pi cache.
pub(crate) fn quarter_pi(
    working: i32,
    budget: &mut DyadicBudget,
) -> Result<DyadicInterval, IntervalError> {
    budget.reserve(8, 3, false)?;
    let one = BigInt::one();
    let (a, b) = scaled_quotient(&one, &BigInt::from(5), -i64::from(working), budget)?;
    let first = series(&DyadicInterval::new(a, b, working)?, working, budget)?;
    let (a, b) = scaled_quotient(&one, &BigInt::from(239), -i64::from(working), budget)?;
    first
        .mul(&DyadicInterval::point(4.into(), 0)?, budget)?
        .sub(
            &series(&DyadicInterval::new(a, b, working)?, working, budget)?,
            budget,
        )
}
fn half_pi(precision: i32, budget: &mut DyadicBudget) -> Result<DyadicInterval, IntervalError> {
    if precision >= 1 {
        budget.reserve(1, 2, false)?;
        return DyadicInterval::new(BigInt::zero(), BigInt::one(), precision);
    }
    let mut working = precision
        .min(-4)
        .checked_sub(12)
        .ok_or(IntervalError::Overflow)?;
    loop {
        budget.reserve(2, 1, false)?;
        let value =
            quarter_pi(working, budget)?.mul(&DyadicInterval::point(2.into(), 0)?, budget)?;
        if let Some(rounded) = tight(&value, precision, budget)? {
            return Ok(rounded);
        }
        working = working.checked_mul(2).ok_or(IntervalError::Overflow)?;
    }
}

// For 0<=x<=1/2, alternating odd powers decrease. The remainder after
// any partial sum has the sign of the next term and magnitude at most that
// term. Directed interval operations enclose every term for the exact source.
fn series(
    argument: &DyadicInterval,
    working: i32,
    budget: &mut DyadicBudget,
) -> Result<DyadicInterval, IntervalError> {
    budget.reserve(1, 4, false)?;
    let zero = DyadicInterval::point(BigInt::zero(), 0)?;
    let tolerance = DyadicInterval::new(BigInt::zero(), BigInt::one(), working)?;
    let ratio = argument.square(budget)?.regrid(working, budget)?;
    let mut power = argument.add(&zero, budget)?;
    let mut sum = zero.clone();
    let mut denominator = 1u64;
    let mut subtract = false;
    loop {
        budget.reserve(u64::from(u64::BITS - denominator.leading_zeros()), 2, false)?;
        let term = power.div(
            &DyadicInterval::point(denominator.into(), 0)?,
            working,
            budget,
        )?;
        let stop = term.subset_of(&tolerance, budget)?;
        let signed = if subtract { term.neg(budget)? } else { term };
        if stop {
            return sum.add(&signed.hull(&zero, budget)?, budget);
        }
        sum = sum.add(&signed, budget)?;
        power = power.mul(&ratio, budget)?.regrid(working, budget)?;
        subtract = !subtract;
        // Each iteration costs >2 of at most u64::MAX units. Exhaustion
        // therefore precedes counter overflow.
        denominator += 2;
    }
}
