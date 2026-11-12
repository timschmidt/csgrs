//! Certified quadrant selection, interior extrema and budgeted Taylor tails.
use crate::dyadic::scaled_quotient;
use crate::{
    DomainCoverage, DyadicBudget, DyadicInterval, ExtendedInterval, ExtendedIntervalView as View,
    IntervalError, RangeEnclosure,
};
use num_bigint::BigInt;
use num_integer::Integer;
use num_traits::{One, Signed, ToPrimitive, Zero};
type Image = RangeEnclosure<DyadicInterval>;
#[derive(Clone, Copy)]
pub(crate) enum Function {
    Sin,
    Cos,
}
#[derive(Clone, Copy)]
pub(crate) struct Source<'a> {
    pub(crate) n: &'a BigInt,
    pub(crate) d: &'a BigInt,
    pub(crate) e: i32,
}
pub(crate) struct Reduced {
    pub(crate) index: BigInt,
    pub(crate) quadrant: u8,
    pub(crate) negative: bool,
    pub(crate) zero: bool,
    pub(crate) argument: DyadicInterval,
}

impl DyadicInterval {
    /// Tight outward sine image on the requested dyadic grid. Certified phase
    /// reduction and interior extrema are included; this is not endpoint-only.
    /// All work shares the caller's size/work budget. Empty maps to empty and
    /// local coverage is Complete. Exhaustion never publishes a partial image.
    pub fn sin(&self, precision: i32, budget: &mut DyadicBudget) -> Result<Image, IntervalError> {
        self.trig(Function::Sin, precision, budget)
    }
    /// Tight outward cosine image, including every interior extremum. See
    /// [`Self::sin`] and [`DyadicBudget`] for coverage and resource semantics.
    pub fn cos(&self, precision: i32, budget: &mut DyadicBudget) -> Result<Image, IntervalError> {
        self.trig(Function::Cos, precision, budget)
    }
    fn trig(
        &self,
        function: Function,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<Image, IntervalError> {
        let Some((a, b, e)) = self.bounds() else {
            return complete(Self::empty());
        };
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
    /// Tight outward sine image of the exact rational source. Requires
    /// rational+dyadic; phase selection, extrema and all refinements are budgeted.
    pub fn sin(&self, precision: i32, budget: &mut DyadicBudget) -> Result<Image, IntervalError> {
        self.trig(Function::Sin, precision, budget)
    }
    /// Tight outward cosine image of the exact rational source, including
    /// interior extrema and Complete local domain coverage.
    pub fn cos(&self, precision: i32, budget: &mut DyadicBudget) -> Result<Image, IntervalError> {
        self.trig(Function::Cos, precision, budget)
    }
    fn trig(
        &self,
        function: Function,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<Image, IntervalError> {
        let Some((a, b)) = self.bounds() else {
            return complete(DyadicInterval::empty());
        };
        budget.reserve(
            a.numerator()
                .bits()
                .max(a.denominator().bits())
                .max(b.numerator().bits())
                .max(b.denominator().bits()),
            4,
            false,
        )?;
        let (an, ad, bn, bd) = (
            BigInt::from_biguint(a.sign(), a.numerator().clone()),
            BigInt::from(a.denominator().clone()),
            BigInt::from_biguint(b.sign(), b.numerator().clone()),
            BigInt::from(b.denominator().clone()),
        );
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
impl ExtendedInterval<DyadicInterval> {
    /// Tight outward sine image. Every nonempty unbounded connected source
    /// reaches both extrema; finite sources use certified phase selection.
    pub fn sin(&self, precision: i32, budget: &mut DyadicBudget) -> Result<Image, IntervalError> {
        match self.view() {
            View::Empty => complete(DyadicInterval::empty()),
            View::Finite(x) => x.sin(precision, budget),
            _ => complete(whole(precision, budget)?),
        }
    }
    /// Tight outward cosine image, including unbounded-source extrema.
    /// Coverage is locally Complete; retain earlier operations' receipts.
    pub fn cos(&self, precision: i32, budget: &mut DyadicBudget) -> Result<Image, IntervalError> {
        match self.view() {
            View::Empty => complete(DyadicInterval::empty()),
            View::Finite(x) => x.cos(precision, budget),
            _ => complete(whole(precision, budget)?),
        }
    }
}
fn complete(range: DyadicInterval) -> Result<Image, IntervalError> {
    Ok(RangeEnclosure {
        range,
        domain: DomainCoverage::Complete,
    })
}
fn whole(precision: i32, budget: &mut DyadicBudget) -> Result<DyadicInterval, IntervalError> {
    budget.reserve(1, 2, false)?;
    DyadicInterval::new((-1).into(), 1.into(), precision.max(0))
}
// Both public source families share an exponent. Width >=8 proves a complete
// period since 2*pi<8, even when resolving either endpoint phase is expensive.
pub(crate) fn wide(
    a: Source<'_>,
    b: Source<'_>,
    budget: &mut DyadicBudget,
) -> Result<bool, IntervalError> {
    if a.d == b.d {
        budget.reserve(a.n.bits().max(b.n.bits()).saturating_add(1), 1, false)?;
        return Ok(!crate::dyadic_order::compare(&(b.n - a.n), a.e, a.d, 3, budget)?.is_lt());
    }
    let bits =
        b.n.bits()
            .saturating_add(a.d.bits())
            .max(a.n.bits().saturating_add(b.d.bits()))
            .max(a.d.bits().saturating_add(b.d.bits()))
            .saturating_add(1);
    budget.reserve(bits, 4, true)?;
    let width = b.n * a.d - a.n * b.d;
    let denominator = a.d * b.d;
    Ok(!crate::dyadic_order::compare(&width, 0, &denominator, 3, budget)?.is_lt())
}
fn range(
    a: Source<'_>,
    b: Source<'_>,
    function: Function,
    precision: i32,
    budget: &mut DyadicBudget,
) -> Result<Image, IntervalError> {
    if wide(a, b, budget)? {
        return complete(whole(precision, budget)?);
    }
    if a.n.is_zero() && b.n.is_zero() {
        budget.reserve(1, 2, false)?;
        return complete(match function {
            Function::Sin => DyadicInterval::point(0.into(), 0)?,
            Function::Cos if precision <= 0 => DyadicInterval::point(1.into(), 0)?,
            Function::Cos => DyadicInterval::new(0.into(), 1.into(), precision)?,
        });
    }
    let size = |x: Source<'_>| {
        (i128::from(x.n.bits()) - i128::from(x.d.bits()) + i128::from(x.e) + 1).max(0)
    };
    let magnitude = size(a).max(size(b));
    let initial = i128::from(precision.min(-4)) - 16 - magnitude;
    // Both the quotient and the cancellation residual need magnitude bits.
    let reservation =
        u64::try_from(magnitude - initial + 4).map_err(|_| IntervalError::SizeExhausted)?;
    budget.reserve(reservation, 1, false)?;
    let mut working = i32::try_from(initial).map_err(|_| IntervalError::Overflow)?;
    loop {
        budget.reserve(2, 1, false)?;
        let half_pi = crate::arctangent::quarter_pi(working, budget)?
            .mul(&DyadicInterval::point(2.into(), 0)?, budget)?;
        if let Some(left) = reduce(a, &half_pi, working, budget)? {
            if a.n == b.n && a.d == b.d {
                if let Some(value) = evaluate(left, function, precision, working, budget)? {
                    return complete(value);
                }
            } else if let Some(right) = reduce(b, &half_pi, working, budget)? {
                let (minimum, maximum) = extrema(&left.index, &right.index, function, budget)?;
                if minimum && maximum {
                    return complete(whole(precision, budget)?);
                }
                if let (Some(left), Some(right)) = (
                    evaluate(left, function, precision, working, budget)?,
                    evaluate(right, function, precision, working, budget)?,
                ) {
                    let mut image = left.hull(&right, budget)?;
                    if minimum {
                        budget.reserve(1, 1, false)?;
                        image = image.hull(&DyadicInterval::point((-1).into(), 0)?, budget)?;
                    }
                    if maximum {
                        budget.reserve(1, 1, false)?;
                        image = image.hull(&DyadicInterval::point(1.into(), 0)?, budget)?;
                    }
                    // Extrema already lie on any grid of precision <=0.
                    return complete(if precision > 0 {
                        image.regrid(precision, budget)?
                    } else {
                        image
                    });
                }
            }
        }
        working = working.checked_mul(2).ok_or(IntervalError::Overflow)?;
    }
}
pub(crate) fn reduce(
    source: Source<'_>,
    half_pi: &DyadicInterval,
    working: i32,
    budget: &mut DyadicBudget,
) -> Result<Option<Reduced>, IntervalError> {
    budget.reserve(source.n.bits().max(source.d.bits()).max(1), 3, false)?;
    if source.n.is_zero() {
        return Ok(Some(Reduced {
            index: 0.into(),
            quadrant: 0,
            negative: false,
            zero: true,
            argument: DyadicInterval::point(0.into(), 0)?,
        }));
    }
    let negative = source.n.is_negative();
    let magnitude = source.n.abs();
    let shift = i64::from(source.e) - i64::from(working);
    let (a, b) = if i128::from(magnitude.bits()) + i128::from(shift) < i128::from(source.d.bits()) {
        budget.reserve(1, 2, false)?;
        (0.into(), 1.into())
    } else {
        scaled_quotient(&magnitude, source.d, shift, budget)?
    };
    let argument = DyadicInterval::new(a, b, working)?;
    let quotient = argument.div(half_pi, 0, budget)?;
    let (a, b, e) = quotient
        .bounds()
        .expect("positive divisor and nonempty argument");
    budget.reserve(a.bits().max(b.bits()).saturating_add(1), 3, false)?;
    if e != 0 || !(b - a).is_one() {
        return Ok(None);
    }
    // A nonzero rational divided by pi/2 is not an integer. Even if an
    // enclosure end is an integer, adjacent integer bounds select its floor.
    let k = a.clone();
    budget.reserve(k.bits().max(3), 2, false)?;
    let quadrant = k.mod_floor(&4.into()).to_u8().expect("residue in 0..4");
    let residual = if k.is_zero() {
        argument
    } else {
        argument.sub(
            &half_pi.mul(&DyadicInterval::point(k.clone(), 0)?, budget)?,
            budget,
        )?
    };
    budget.reserve(2, 2, false)?;
    let residual = residual.intersection(&DyadicInterval::new(0.into(), 2.into(), 0)?, budget)?;
    let index = if negative { -k - 1 } else { k };
    Ok(Some(Reduced {
        index,
        quadrant,
        negative,
        zero: false,
        argument: residual,
    }))
}
pub(crate) fn extrema(
    a: &BigInt,
    b: &BigInt,
    function: Function,
    budget: &mut DyadicBudget,
) -> Result<(bool, bool), IntervalError> {
    budget.reserve(a.bits().max(b.bits()).saturating_add(3), 6, false)?;
    if b - a >= BigInt::from(4) {
        return Ok((true, true));
    }
    let mut minimum = false;
    let mut maximum = false;
    for offset in 1..=3 {
        let index: BigInt = a + offset;
        if &index > b {
            break;
        }
        let residue = index.mod_floor(&4.into()).to_u8().expect("residue in 0..4");
        minimum |= residue
            == match function {
                Function::Sin => 3,
                Function::Cos => 2,
            };
        maximum |= residue
            == match function {
                Function::Sin => 1,
                Function::Cos => 0,
            };
    }
    Ok((minimum, maximum))
}
fn evaluate(
    source: Reduced,
    function: Function,
    precision: i32,
    working: i32,
    budget: &mut DyadicBudget,
) -> Result<Option<DyadicInterval>, IntervalError> {
    budget.reserve(1, 2, false)?;
    let one = DyadicInterval::point(1.into(), 0)?;
    let value = if source.zero {
        match function {
            Function::Sin => DyadicInterval::point(0.into(), 0)?,
            Function::Cos => one,
        }
    } else {
        let cosine = match function {
            Function::Sin => source.quadrant % 2 == 1,
            Function::Cos => matches!(source.quadrant, 0 | 2),
        };
        let negative = match function {
            Function::Sin => (source.quadrant >= 2) ^ source.negative,
            Function::Cos => source.quadrant == 1 || source.quadrant == 2,
        };
        let value = series(source.argument, cosine, working, budget)?
            .intersection(&DyadicInterval::new(0.into(), 1.into(), 0)?, budget)?;
        if negative { value.neg(budget)? } else { value }
    };
    let rounded = if source.zero && precision <= 0 {
        value
    } else {
        value.regrid(precision, budget)?
    };
    let (a, b, e) = rounded
        .bounds()
        .expect("selected quadrant retains a real function value");
    budget.reserve(a.bits().max(b.bits()).saturating_add(1), 1, false)?;
    let gap = b - a;
    Ok(if gap.is_zero() || (gap.is_one() && e == precision) {
        Some(rounded)
    } else {
        None
    })
}
// Residuals lie in [0,pi/2] subset [0,2]. Beyond the initial cosine term,
// alternating factorial terms decrease. The signed next term bounds the tail.
pub(crate) fn series(
    argument: DyadicInterval,
    cosine: bool,
    working: i32,
    budget: &mut DyadicBudget,
) -> Result<DyadicInterval, IntervalError> {
    budget.reserve(1, 4, false)?;
    let zero = DyadicInterval::point(0.into(), 0)?;
    let tolerance = DyadicInterval::new(0.into(), 1.into(), working)?;
    let square = argument.square(budget)?.regrid(working, budget)?;
    let mut term = if cosine {
        DyadicInterval::point(1.into(), 0)?
    } else {
        argument
    };
    let mut sum = zero.clone();
    let mut subtract = false;
    let mut n = if cosine { 0u64 } else { 1u64 };
    loop {
        let stop = term.subset_of(&tolerance, budget)?;
        if stop {
            let tail = if subtract { term.neg(budget)? } else { term };
            return sum.add(&tail.hull(&zero, budget)?, budget);
        }
        sum = if subtract {
            sum.sub(&term, budget)?
        } else {
            sum.add(&term, budget)?
        };
        // Work exhaustion precedes counter overflow; multiply as BigInts so
        // the two-index product has no machine-word overflow boundary.
        budget.reserve(
            u64::from(u64::BITS - (n + 2).leading_zeros()).saturating_mul(2),
            3,
            true,
        )?;
        let denominator = BigInt::from(n + 1) * BigInt::from(n + 2);
        term = term.mul(&square, budget)?.div(
            &DyadicInterval::point(denominator, 0)?,
            working,
            budget,
        )?;
        subtract = !subtract;
        n += 2;
    }
}
