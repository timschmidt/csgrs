//! Pole incidence and monotone branches reuse certified trigonometric bounds.
use crate::trigonometry::{Function, Reduced, Source, extrema, reduce, series, wide};
#[cfg(feature = "rational")]
use crate::trigonometry::{reserve_rational_sources, signed_rational_source};
use crate::{
    DomainCoverage, DyadicBudget, DyadicInterval, ExtendedInterval, ExtendedIntervalView as View,
    IntervalError, IntervalSet2, RangeEnclosure,
};
use num_bigint::BigInt;
use num_traits::{One, Zero};
type Extended = ExtendedInterval<DyadicInterval>;
type Image = RangeEnclosure<IntervalSet2<DyadicInterval>>;
#[derive(Clone, Copy)]
enum PoleFunction {
    Tan,
    Cot,
}
impl PoleFunction {
    fn cot(self) -> bool {
        matches!(self, Self::Cot)
    }
}
impl DyadicInterval {
    /// Tight closed tangent enclosure on the requested grid, retaining split
    /// images around poles. Empty maps to empty/Complete; excluded poles report
    /// Partial. Certified phase selection distinguishes no pole, one pole and
    /// full-branch coverage. An unresolved numerical denominator causes further
    /// refinement or typed exhaustion, never a whole-range fallback.
    /// All nested work uses the caller's DyadicBudget; its exclusions apply.
    pub fn tan(&self, precision: i32, budget: &mut DyadicBudget) -> Result<Image, IntervalError> {
        self.pole_trig(PoleFunction::Tan, precision, budget)
    }
    /// Tight closed cotangent enclosure. Zero-only input returns EmptyDomain;
    /// endpoint/interior poles use one-sided limits and Partial coverage.
    /// Other semantics and resource accounting match [`Self::tan`].
    pub fn cot(&self, precision: i32, budget: &mut DyadicBudget) -> Result<Image, IntervalError> {
        self.pole_trig(PoleFunction::Cot, precision, budget)
    }
    fn pole_trig(
        &self,
        function: PoleFunction,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<Image, IntervalError> {
        let Some((a, b, e)) = self.bounds() else {
            return image(IntervalSet2::empty(), DomainCoverage::Complete);
        };
        if function.cot() && a.is_zero() && b.is_zero() {
            return Err(IntervalError::EmptyDomain);
        }
        budget.reserve(1, 1, false)?;
        let one = BigInt::one();
        range(
            Source { n: a, d: &one, e },
            Source { n: b, d: &one, e },
            function,
            precision,
            budget,
        )
    }
}
#[cfg(feature = "rational")]
impl crate::RationalInterval {
    /// Tangent of the exact rational source, with certified pole incidence,
    /// outward dyadic components and domain coverage. Stored rational parts
    /// use the same ingress budget as [`Self::sin`]. Requires rational+dyadic.
    pub fn tan(&self, precision: i32, budget: &mut DyadicBudget) -> Result<Image, IntervalError> {
        self.pole_trig(PoleFunction::Tan, precision, budget)
    }
    /// Cotangent of the exact rational source, including the exact zero pole.
    /// See [`DyadicInterval::cot`] for limits and shared-budget semantics.
    pub fn cot(&self, precision: i32, budget: &mut DyadicBudget) -> Result<Image, IntervalError> {
        self.pole_trig(PoleFunction::Cot, precision, budget)
    }
    fn pole_trig(
        &self,
        function: PoleFunction,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<Image, IntervalError> {
        let Some((a, b)) = self.bounds() else {
            return image(IntervalSet2::empty(), DomainCoverage::Complete);
        };
        if function.cot() && a.is_zero() && b.is_zero() {
            return Err(IntervalError::EmptyDomain);
        }
        reserve_rational_sources(a, b, budget)?;
        let (an, ad) = signed_rational_source(a);
        let (bn, bd) = signed_rational_source(b);
        range(
            Source {
                n: &an,
                d: &ad,
                e: 0,
            },
            Source {
                n: &bn,
                d: &bd,
                e: 0,
            },
            function,
            precision,
            budget,
        )
    }
}
impl Extended {
    /// Tangent including unbounded connected sources: each contains full
    /// branches and poles, so its image is the entire line with Partial coverage.
    pub fn tan(&self, precision: i32, budget: &mut DyadicBudget) -> Result<Image, IntervalError> {
        match self.view() {
            View::Empty => image(IntervalSet2::empty(), DomainCoverage::Complete),
            View::Finite(x) => x.tan(precision, budget),
            _ => whole(),
        }
    }
    /// Cotangent with unbounded-source and pole semantics as in [`Self::tan`].
    /// Preserve earlier domain receipts when consuming this local result.
    pub fn cot(&self, precision: i32, budget: &mut DyadicBudget) -> Result<Image, IntervalError> {
        match self.view() {
            View::Empty => image(IntervalSet2::empty(), DomainCoverage::Complete),
            View::Finite(x) => x.cot(precision, budget),
            _ => whole(),
        }
    }
}
fn image(
    range: IntervalSet2<DyadicInterval>,
    domain: DomainCoverage,
) -> Result<Image, IntervalError> {
    Ok(RangeEnclosure { range, domain })
}
fn whole() -> Result<Image, IntervalError> {
    image(
        IntervalSet2::from_interval(Extended::entire()),
        DomainCoverage::Partial,
    )
}
fn range(
    a: Source<'_>,
    b: Source<'_>,
    function: PoleFunction,
    precision: i32,
    budget: &mut DyadicBudget,
) -> Result<Image, IntervalError> {
    if wide(a, b, budget)? {
        return whole();
    }
    if a.n.is_zero() && b.n.is_zero() {
        // Cotangent's exact zero pole was validated at ingress.
        return image(
            IntervalSet2::from_interval(Extended::from_finite(DyadicInterval::point(0.into(), 0)?)),
            DomainCoverage::Complete,
        );
    }
    let left_pole = function.cot() && a.n.is_zero();
    let right_pole = function.cot() && b.n.is_zero();
    let size = |x: Source<'_>| {
        (i128::from(x.n.bits()) - i128::from(x.d.bits()) + i128::from(x.e) + 1).max(0)
    };
    let magnitude = size(a).max(size(b));
    let initial = i128::from(precision.min(-4)) - 16 - magnitude;
    budget.reserve(
        u64::try_from(magnitude - initial + 4).map_err(|_| IntervalError::SizeExhausted)?,
        1,
        false,
    )?;
    let mut working = i32::try_from(initial).map_err(|_| IntervalError::Overflow)?;
    loop {
        budget.reserve(2, 1, false)?;
        let half_pi = crate::arctangent::quarter_pi(working, budget)?
            .mul(&DyadicInterval::point(2.into(), 0)?, budget)?;
        if let Some(left) = reduce(a, &half_pi, working, budget)? {
            if a.n == b.n && a.d == b.d {
                if let Some(value) = evaluate(left, function, precision, working, budget)? {
                    return image(
                        IntervalSet2::from_interval(Extended::from_finite(value)),
                        DomainCoverage::Complete,
                    );
                }
            } else if let Some(right) = reduce(b, &half_pi, working, budget)? {
                // Extrema of sine locate tangent poles; extrema of cosine
                // locate cotangent poles. Exclude an exact right-end zero
                // from interior incidence; it is handled by a one-sided limit.
                budget.reserve(1, 1, false)?;
                let before_zero = BigInt::from(-1);
                let (first, second) = extrema(
                    &left.index,
                    if right_pole {
                        &before_zero
                    } else {
                        &right.index
                    },
                    if function.cot() {
                        Function::Cos
                    } else {
                        Function::Sin
                    },
                    budget,
                )?;
                let interior = first || second;
                if (first && second) || (interior && (left_pole || right_pole)) {
                    return whole();
                }
                let left = if left_pole {
                    None
                } else {
                    evaluate(left, function, precision, working, budget)?
                };
                let right = if right_pole {
                    None
                } else {
                    evaluate(right, function, precision, working, budget)?
                };
                if (left_pole || left.is_some()) && (right_pole || right.is_some()) {
                    return assemble(left, right, function, interior, budget);
                }
            }
        }
        working = working.checked_mul(2).ok_or(IntervalError::Overflow)?;
    }
}
fn endpoint(
    x: &DyadicInterval,
    lower: bool,
    budget: &mut DyadicBudget,
) -> Result<DyadicInterval, IntervalError> {
    let (a, b, e) = x.bounds().expect("nonempty point enclosure");
    let n = if lower { a } else { b };
    budget.reserve(n.bits().max(1), 1, false)?;
    DyadicInterval::point(n.clone(), e)
}
fn assemble(
    left: Option<DyadicInterval>,
    right: Option<DyadicInterval>,
    function: PoleFunction,
    interior: bool,
    budget: &mut DyadicBudget,
) -> Result<Image, IntervalError> {
    // Cotangent decreases on each branch; tangent increases.
    let (lower, upper) = if function.cot() {
        (right, left)
    } else {
        (left, right)
    };
    match (lower, upper) {
        (Some(lower), Some(upper)) if !interior => {
            let (a, _, e) = lower.bounds().expect("nonempty image");
            let (_, b, f) = upper.bounds().expect("nonempty image");
            image(
                IntervalSet2::from_interval(crate::extended_dyadic::closed_bounds(
                    a, e, b, f, budget,
                )?),
                DomainCoverage::Complete,
            )
        }
        (Some(lower), Some(upper)) => image(
            IntervalSet2::new(
                Extended::above(endpoint(&lower, true, budget)?)?,
                Extended::below(endpoint(&upper, false, budget)?)?,
                budget,
            )?,
            DomainCoverage::Partial,
        ),
        (Some(lower), None) => image(
            IntervalSet2::from_interval(Extended::above(endpoint(&lower, true, budget)?)?),
            DomainCoverage::Partial,
        ),
        (None, Some(upper)) => image(
            IntervalSet2::from_interval(Extended::below(endpoint(&upper, false, budget)?)?),
            DomainCoverage::Partial,
        ),
        (None, None) => unreachable!("zero-only cotangent was rejected at ingress"),
    }
}
fn evaluate(
    source: Reduced,
    function: PoleFunction,
    precision: i32,
    working: i32,
    budget: &mut DyadicBudget,
) -> Result<Option<DyadicInterval>, IntervalError> {
    if source.zero {
        return Ok(Some(DyadicInterval::point(0.into(), 0)?));
    }
    let (a, b, _) = source
        .argument
        .bounds()
        .expect("certified residual enclosure");
    budget.reserve(a.bits().max(b.bits()).max(1), 2, false)?;
    let unit = DyadicInterval::new(0.into(), 1.into(), 0)?;
    let denominator_cosine = source.quadrant.is_multiple_of(2) != function.cot();
    let denominator = series(source.argument.clone(), denominator_cosine, working, budget)?
        .intersection(&unit, budget)?;
    if denominator.signs().contains_zero() {
        return Ok(None);
    }
    let numerator = series(source.argument, !denominator_cosine, working, budget)?
        .intersection(&unit, budget)?;
    let value = numerator.div(&denominator, precision, budget)?;
    let value = if (source.quadrant % 2 == 1) ^ source.negative {
        value.neg(budget)?
    } else {
        value
    };
    let (a, b, e) = value
        .bounds()
        .expect("positive divisor and nonempty numerator");
    budget.reserve(a.bits().max(b.bits()).saturating_add(1), 1, false)?;
    let gap = b - a;
    Ok(if gap.is_zero() || (gap.is_one() && e == precision) {
        Some(value)
    } else {
        None
    })
}
