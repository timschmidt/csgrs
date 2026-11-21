//! Budgeted logarithm from positive atanh series and exact binary reduction.
use crate::dyadic::scaled_quotient;
use crate::{
    DomainCoverage, DyadicBudget, DyadicInterval, ExtendedInterval, IntervalError, RangeEnclosure,
};
use num_bigint::BigInt;
use num_traits::{One, Zero};
type Image = RangeEnclosure<ExtendedInterval<DyadicInterval>>;
impl DyadicInterval {
    /// Tight outward natural logarithm on the grid `2^precision`.
    /// Positive sources have Complete coverage. A nonpositive lower endpoint
    /// with positive upper endpoint returns a lower-unbounded image with
    /// Partial coverage; no positive source member returns `EmptyDomain`.
    /// Empty maps to empty with Complete coverage. All numerical stages share
    /// the caller's deterministic size/work budget; no partial result escapes
    /// exhaustion. Cancellation and actual heap-byte bounds are not provided.
    pub fn log(&self, precision: i32, budget: &mut DyadicBudget) -> Result<Image, IntervalError> {
        let Some((a, b, e)) = self.bounds() else {
            return image(ExtendedInterval::empty(), DomainCoverage::Complete);
        };
        if b <= &BigInt::zero() {
            return Err(IntervalError::EmptyDomain);
        }
        budget.reserve(1, 1, false)?;
        let one = BigInt::one();
        let upper = endpoint(b, &one, e, precision, budget)?;
        if a <= &BigInt::zero() {
            return partial(upper, budget);
        }
        if a == b {
            return image(
                ExtendedInterval::from_finite(upper),
                DomainCoverage::Complete,
            );
        }
        assemble(endpoint(a, &one, e, precision, budget)?, upper, budget)
    }
}
#[cfg(feature = "rational")]
impl crate::RationalInterval {
    /// Tight natural-logarithm enclosure of the defined real source domain.
    /// Requires rational+dyadic. Intervals reaching zero from above return an
    /// explicit lower-unbounded enclosure with Partial coverage. Exact rational
    /// inputs are retained through every budgeted refinement attempt.
    /// Stored fractions and singleton comparisons follow the budget in [`Self::exp`].
    pub fn log(&self, precision: i32, budget: &mut DyadicBudget) -> Result<Image, IntervalError> {
        let Some((a, b)) = self.bounds() else {
            return image(ExtendedInterval::empty(), DomainCoverage::Complete);
        };
        if !b.is_positive() {
            return Err(IntervalError::EmptyDomain);
        }
        let point = |x: &hyperreal::Rational, budget: &mut DyadicBudget| {
            crate::rational_admission::reserve_monotone_source(x, budget)?;
            endpoint(
                &BigInt::from(x.numerator().clone()),
                &BigInt::from(x.denominator().clone()),
                0,
                precision,
                budget,
            )
        };
        let upper = point(b, budget)?;
        if !a.is_positive() {
            return partial(upper, budget);
        }
        if crate::rational_admission::same_monotone_source(b, a, budget)? {
            return image(
                ExtendedInterval::from_finite(upper),
                DomainCoverage::Complete,
            );
        }
        assemble(point(a, budget)?, upper, budget)
    }
}
fn image(
    range: ExtendedInterval<DyadicInterval>,
    domain: DomainCoverage,
) -> Result<Image, IntervalError> {
    Ok(RangeEnclosure { range, domain })
}
fn partial(upper: DyadicInterval, budget: &mut DyadicBudget) -> Result<Image, IntervalError> {
    let (_, b, e) = upper.bounds().expect("nonempty point image");
    budget.reserve(b.bits().max(1), 2, false)?;
    image(
        ExtendedInterval::below(DyadicInterval::point(b.clone(), e)?)?,
        DomainCoverage::Partial,
    )
}
fn assemble(
    lower: DyadicInterval,
    upper: DyadicInterval,
    budget: &mut DyadicBudget,
) -> Result<Image, IntervalError> {
    let (a, _, e) = lower.bounds().expect("nonempty point image");
    let (_, b, f) = upper.bounds().expect("nonempty point image");
    image(
        crate::extended_dyadic::closed_bounds(a, e, b, f, budget)?,
        DomainCoverage::Complete,
    )
}
// Positive exact source n/d * 2^e. Normalize it to m*2^k, 1<=m<2.
fn endpoint(
    n: &BigInt,
    d: &BigInt,
    e: i32,
    precision: i32,
    budget: &mut DyadicBudget,
) -> Result<DyadicInterval, IntervalError> {
    let sign = crate::dyadic_order::compare(n, e, d, 0, budget)?;
    let zero = DyadicInterval::point(BigInt::zero(), 0)?;
    if sign.is_eq() {
        return Ok(zero);
    }
    let offset = i64::try_from(n.bits()).map_err(|_| IntervalError::Overflow)?
        - i64::try_from(d.bits()).map_err(|_| IntervalError::Overflow)?;
    let mut offset = i32::try_from(offset).map_err(|_| IntervalError::Overflow)?;
    if crate::dyadic_order::compare(n, 0, d, offset, budget)?.is_lt() {
        offset = offset.checked_sub(1).ok_or(IntervalError::Overflow)?;
    }
    let k = i64::from(e) + i64::from(offset);
    let bits = n
        .bits()
        .saturating_add(if offset < 0 {
            u64::from(offset.unsigned_abs())
        } else {
            0
        })
        .max(
            d.bits()
                .saturating_add(if offset > 0 { offset as u64 } else { 0 }),
        );
    budget.reserve(bits, 2, false)?;
    let (n, d) = if offset >= 0 {
        (n.clone(), d << offset as usize)
    } else {
        (n << offset.unsigned_abs() as usize, d.clone())
    };
    budget.reserve(bits.saturating_add(1), 2, false)?;
    let numerator = &n - &d;
    let denominator = &n + &d;
    let coefficient = if k == 0 {
        None
    } else {
        budget.reserve(
            u64::from(u64::BITS - k.unsigned_abs().leading_zeros()),
            2,
            false,
        )?;
        Some(DyadicInterval::point(BigInt::from(k), 0)?)
    };
    let mut working = precision
        .min(-4)
        .checked_sub(12)
        .ok_or(IntervalError::Overflow)?;
    loop {
        let mut reduced = if numerator.is_zero() {
            None
        } else {
            Some(series(&numerator, &denominator, working, budget)?)
        };
        if let Some(coefficient) = &coefficient {
            budget.reserve(2, 2, false)?;
            let log_two = series(&BigInt::one(), &BigInt::from(3), working, budget)?
                .mul(coefficient, budget)?;
            reduced = Some(match reduced {
                None => log_two,
                Some(value) => value.add(&log_two, budget)?,
            });
        }
        let value = reduced.expect("nonunit input has a reduced or scale contribution");
        // Transport the exact source-side relation through log's monotonicity.
        // This avoids rediscovering a known sign after cancelling k*log(2).
        let value = if sign.is_lt() {
            value.min_image(&zero, budget)?
        } else {
            value.max_image(&zero, budget)?
        };
        let rounded = value.regrid(precision, budget)?;
        let (a, b, e) = rounded.bounds().expect("nonzero logarithm construction");
        budget.reserve(a.bits().max(b.bits()).saturating_add(1), 1, false)?;
        let gap = b - a;
        if gap.is_zero() || (gap.is_one() && e == precision) {
            return Ok(rounded);
        }
        working = working.checked_mul(2).ok_or(IntervalError::Overflow)?;
    }
}
// log(m)=2*atanh(t), t=(m-1)/(m+1). Here 0<t<=1/3, and
// directed argument rounding keeps t<=1/2. Successive positive series terms
// have ratio <=1/4, so twice the next term bounds the entire omitted tail.
fn series(
    n: &BigInt,
    d: &BigInt,
    working: i32,
    budget: &mut DyadicBudget,
) -> Result<DyadicInterval, IntervalError> {
    let (lo, hi) = scaled_quotient(n, d, -i64::from(working), budget)?;
    let argument = DyadicInterval::new(lo, hi, working)?;
    let ratio = argument.square(budget)?.regrid(working, budget)?;
    budget.reserve(2, 4, false)?;
    let zero = DyadicInterval::point(BigInt::zero(), 0)?;
    let two = DyadicInterval::point(BigInt::from(2), 0)?;
    let tolerance = DyadicInterval::new(BigInt::zero(), BigInt::one(), working)?;
    let mut power = argument;
    let mut sum = zero.clone();
    let mut denominator = 1u64;
    loop {
        budget.reserve(u64::from(u64::BITS - denominator.leading_zeros()), 2, false)?;
        let divisor = DyadicInterval::point(BigInt::from(denominator), 0)?;
        let term = power.div(&divisor, working, budget)?;
        if term.subset_of(&tolerance, budget)? {
            return sum
                .add(&term.mul(&two, budget)?.hull(&zero, budget)?, budget)?
                .mul(&two, budget);
        }
        sum = sum.add(&term, budget)?;
        power = power.mul(&ratio, budget)?.regrid(working, budget)?;
        // Each iteration consumes >2 of at most u64::MAX work units, so
        // exhaustion precedes overflow of this odd denominator counter.
        denominator += 2;
    }
}
