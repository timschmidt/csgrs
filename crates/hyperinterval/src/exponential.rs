//! Budgeted interval evaluation; no unaccounted computable-graph evaluation.
use crate::dyadic::scaled_quotient;
use crate::{DomainCoverage, DyadicBudget, DyadicInterval, IntervalError, RangeEnclosure};
use num_bigint::BigInt;
use num_traits::{One, Signed, Zero};

impl DyadicInterval {
    /// Tight outward exponential image on the grid `2^precision`.
    ///
    /// The exponential is defined everywhere; empty maps to empty and domain
    /// coverage is always `Complete`. Certified Taylor tails, directed squaring
    /// and reciprocal arithmetic refine until each endpoint has adjacent grid
    /// bounds. All stages share the caller's size/work budget. Exhaustion
    /// publishes no partial result; there is no cache to poison a later retry.
    /// These are deterministic accounting units, not CPU/heap-byte bounds or
    /// cancellation support. See [`DyadicBudget`].
    pub fn exp(
        &self,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<RangeEnclosure<Self>, IntervalError> {
        let Some((a, b, e)) = self.bounds() else {
            return complete(Self::empty());
        };
        budget.reserve(1, 1, false)?;
        let denominator = BigInt::one();
        let lower = endpoint(a, &denominator, e, precision, budget)?;
        if a == b {
            return complete(lower);
        }
        let upper = endpoint(b, &denominator, e, precision, budget)?;
        complete(outer_bounds(lower, upper, budget)?)
    }
}

#[cfg(feature = "rational")]
impl crate::RationalInterval {
    /// Tight outward exponential image with dyadic spacing `2^precision`.
    ///
    /// Requires `rational` and `dyadic`. The complete exact rational source is
    /// retained through argument bounds; it is not replaced by an adopted
    /// rounded model. Directed refinement and all interval arithmetic share
    /// the caller's budget. Empty maps to empty with `Complete` coverage.
    pub fn exp(
        &self,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<RangeEnclosure<DyadicInterval>, IntervalError> {
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
        complete(outer_bounds(lower, point(b, budget)?, budget)?)
    }
}

// Select only the monotone image extrema before normalization/alignment.
// The other half of either point enclosure need not occur in the region image
// representation; retaining it can cause needless scale-sized allocations.
fn outer_bounds(
    lower: DyadicInterval,
    upper: DyadicInterval,
    budget: &mut DyadicBudget,
) -> Result<DyadicInterval, IntervalError> {
    let (a, _, e) = lower.bounds().expect("nonempty lower endpoint image");
    let (_, b, f) = upper.bounds().expect("nonempty upper endpoint image");
    budget.reserve(a.bits().max(b.bits()).max(1), 4, false)?;
    if a.is_zero() {
        // Zero needs no aligned mantissa, even at the minimum grid exponent.
        return DyadicInterval::new(BigInt::zero(), b.clone(), f);
    }
    DyadicInterval::point(a.clone(), e)?.hull(&DyadicInterval::point(b.clone(), f)?, budget)
}

fn complete(range: DyadicInterval) -> Result<RangeEnclosure<DyadicInterval>, IntervalError> {
    Ok(RangeEnclosure {
        range,
        domain: DomainCoverage::Complete,
    })
}

// n*2^exponent/d is a single exact source; d is strictly positive.
fn endpoint(
    n: &BigInt,
    d: &BigInt,
    exponent: i32,
    precision: i32,
    budget: &mut DyadicBudget,
) -> Result<DyadicInterval, IntervalError> {
    budget.reserve(n.bits().max(d.bits()).max(1), 3, false)?;
    let one = DyadicInterval::point(BigInt::one(), 0)?;
    if n.is_zero() {
        return if precision <= 0 {
            Ok(one)
        } else {
            DyadicInterval::new(BigInt::zero(), BigInt::one(), precision)
        };
    }
    let negative = n.is_negative();
    let magnitude = n.abs();
    // For p<=0 and x<=p, exp(x)<=exp(p)<=2^p. For p>=0 and
    // x<0, exp(x)<1<=2^p. No magnitude-sized expansion is required.
    if negative
        && (precision >= 0
            || at_least_integer(&magnitude, d, exponent, precision.unsigned_abs(), budget)?)
    {
        return DyadicInterval::new(BigInt::zero(), BigInt::one(), precision);
    }
    // n<2^N, d>=2^(D-1); division by 2^s puts the magnitude below 1/2.
    let reduction = (i64::try_from(n.bits()).map_err(|_| IntervalError::Overflow)?
        - i64::try_from(d.bits()).map_err(|_| IntervalError::Overflow)?
        + i64::from(exponent)
        + 2)
    .max(0);
    budget.reserve(1, reduction as u64, false)?;
    let mut working = precision
        .min(-4)
        .checked_sub(12)
        .ok_or(IntervalError::Overflow)?;
    loop {
        let shift = i64::from(exponent) - reduction - i64::from(working);
        let (lo, hi) = if i128::from(magnitude.bits()) + i128::from(shift) < i128::from(d.bits()) {
            // Positive value below one grid unit; avoid a huge denominator.
            budget.reserve(1, 2, false)?;
            (BigInt::zero(), BigInt::one())
        } else {
            scaled_quotient(&magnitude, d, shift, budget)?
        };
        let argument = DyadicInterval::new(lo, hi, working)?;
        let mut value = small_exp(&argument, working, budget)?;
        for _ in 0..reduction {
            value = value.square(budget)?.regrid(working, budget)?;
        }
        if negative {
            value = value.reciprocal(working, budget)?;
        }
        let rounded = value.regrid(precision, budget)?;
        let (lo, hi, e) = rounded.bounds().expect("nonempty exponential");
        budget.reserve(lo.bits().max(hi.bits()).saturating_add(1), 1, false)?;
        let gap = hi - lo;
        if gap.is_zero() || (gap.is_one() && e == precision) {
            return Ok(rounded);
        }
        // All failed attempts remain charged to the same budget. Doubling the
        // guard precision eventually separates nonzero rational exponentials
        // from dyadic grid boundaries, subject to resource/storage limits.
        working = working.checked_mul(2).ok_or(IntervalError::Overflow)?;
    }
}

// Prove n*2^e/d >= threshold with bit lengths before aligned comparison.
// This shortcut is distinct from ordinary quotient materialization: a huge
// negative exponential can have a one-unit enclosure even at extreme e/p.
fn at_least_integer(
    n: &BigInt,
    d: &BigInt,
    e: i32,
    threshold: u32,
    budget: &mut DyadicBudget,
) -> Result<bool, IntervalError> {
    budget.reserve(d.bits().saturating_add(32), 1, true)?;
    let right = d * threshold;
    let left_bits = i128::from(n.bits()) + i128::from(e);
    if left_bits != i128::from(right.bits()) {
        return Ok(left_bits > i128::from(right.bits()));
    }
    budget.reserve(n.bits().max(right.bits()), 2, false)?;
    Ok(if e >= 0 {
        n << e as usize >= right
    } else {
        n >= &(right << e.unsigned_abs() as usize)
    })
}

// 0<=x<=1/2. After S_n, the remaining positive Taylor tail is at most
// 2*t_(n+1), since every subsequent term ratio is at most 1/2. Directed
// arithmetic encloses every term and sum, including argument-grid uncertainty.
fn small_exp(
    argument: &DyadicInterval,
    working: i32,
    budget: &mut DyadicBudget,
) -> Result<DyadicInterval, IntervalError> {
    budget.reserve(2, 4, false)?;
    let one = DyadicInterval::point(BigInt::one(), 0)?;
    let two = DyadicInterval::point(BigInt::from(2), 0)?;
    let zero = DyadicInterval::point(BigInt::zero(), 0)?;
    let tolerance = DyadicInterval::new(BigInt::zero(), BigInt::one(), working)?;
    let mut sum = one.clone();
    let mut term = one;
    let mut k = 1u64;
    loop {
        budget.reserve(u64::from(u64::BITS - k.leading_zeros()), 2, false)?;
        let divisor = DyadicInterval::point(BigInt::from(k), 0)?;
        term = term.mul(argument, budget)?.div(&divisor, working, budget)?;
        if term.subset_of(&tolerance, budget)? {
            return sum.add(&term.mul(&two, budget)?.hull(&zero, budget)?, budget);
        }
        sum = sum.add(&term, budget)?;
        // Every iteration consumes more than one of at most u64::MAX work
        // units, so work exhaustion precedes overflow of this index.
        k += 1;
    }
}
