//! Real rational powers from exact integer relations and sign monotonicity.
use crate::{
    DomainCoverage, DyadicBudget, DyadicInterval, ExtendedInterval, ExtendedIntervalView as View,
    IntervalError, IntervalSet2, RangeEnclosure, SignSet,
};
use num_bigint::{BigInt, BigUint};
use num_integer::Integer;
use num_traits::{One, Pow, Signed, ToPrimitive, Zero};
type Extended = ExtendedInterval<DyadicInterval>;
type Image = RangeEnclosure<IntervalSet2<DyadicInterval>>;

// Endpoints borrow the authoritative source. No preliminary dyadic conversion.
#[derive(Clone, Copy)]
enum End<'a> {
    Infinity,
    Finite(&'a BigInt, &'a BigInt, i32),
}
struct Power {
    numerator: BigUint,
    denominator: BigUint,
    negative: bool,
}
impl Power {
    fn new(n: &BigInt, d: &BigInt, budget: &mut DyadicBudget) -> Result<Self, IntervalError> {
        let bits = n.bits().max(d.bits()).max(1);
        budget.reserve(bits, bits.saturating_mul(2).saturating_add(5), true)?;
        let gcd = n.magnitude().gcd(d.magnitude());
        Ok(Self {
            numerator: n.magnitude() / &gcd,
            denominator: d.magnitude() / gcd,
            negative: n.is_negative() != d.is_negative(),
        })
    }
}
impl DyadicInterval {
    /// Tight closed outward image of the real power `x^(numerator/denominator)`.
    ///
    /// Both exponent integers may have arbitrary finite size. A zero denominator
    /// returns ContainsZero, including for empty sources. The exponent is reduced
    /// before branch selection. Odd denominators admit negative bases; even ones
    /// exclude them. Negative exponents exclude zero. Excluded source members
    /// report Partial, and no defined member returns EmptyDomain. Empty returns
    /// empty/Complete; exponent zero maps all real members, including zero, to one.
    ///
    /// Outputs retain up to two closed components around a pole. Infinity is a
    /// limit; closure may include unattained zero. Integer relations certify the
    /// requested grid bounds. All arithmetic shares the supplied deterministic
    /// size/work budget; see DyadicBudget for its units and exclusions.
    pub fn pow_rational(
        &self,
        numerator: &BigInt,
        denominator: &BigInt,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<Image, IntervalError> {
        if let Some(result) = trivial(self.signs(), numerator, denominator, precision, budget)? {
            return Ok(result);
        }
        let (a, b, e) = self.bounds().expect("nonempty source");
        budget.reserve(1, 1, false)?;
        let one = BigInt::one();
        image(
            End::Finite(a, &one, e),
            End::Finite(b, &one, e),
            self.signs(),
            &Power::new(numerator, denominator, budget)?,
            precision,
            budget,
        )
    }
}
impl Extended {
    /// Real rational power with the same domain and grid contract as
    /// [`DyadicInterval::pow_rational`], including infinite-end limits.
    pub fn pow_rational(
        &self,
        numerator: &BigInt,
        denominator: &BigInt,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<Image, IntervalError> {
        if let Some(result) = trivial(self.signs(), numerator, denominator, precision, budget)? {
            return Ok(result);
        }
        budget.reserve(1, 1, false)?;
        let one = BigInt::one();
        let (a, b) = match self.view() {
            View::Empty => unreachable!("handled by trivial"),
            View::Entire => (End::Infinity, End::Infinity),
            View::Finite(x) => {
                let (a, b, e) = x.bounds().expect("nonempty source");
                (End::Finite(a, &one, e), End::Finite(b, &one, e))
            }
            View::Below(x) => {
                let (b, _, e) = x.bounds().expect("singleton bound");
                (End::Infinity, End::Finite(b, &one, e))
            }
            View::Above(x) => {
                let (a, _, e) = x.bounds().expect("singleton bound");
                (End::Finite(a, &one, e), End::Infinity)
            }
        };
        image(
            a,
            b,
            self.signs(),
            &Power::new(numerator, denominator, budget)?,
            precision,
            budget,
        )
    }
}
#[cfg(feature = "rational")]
impl crate::RationalInterval {
    /// Real rational power retaining exact rational endpoints through root
    /// isolation. Requires rational+dyadic; see [`DyadicInterval::pow_rational`]
    /// for branch, pole, empty, domain-coverage and shared-budget semantics.
    pub fn pow_rational(
        &self,
        numerator: &BigInt,
        denominator: &BigInt,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<Image, IntervalError> {
        if let Some(result) = trivial(self.signs(), numerator, denominator, precision, budget)? {
            return Ok(result);
        }
        let (a, b) = self.bounds().expect("nonempty source");
        let convert = |x: &hyperreal::Rational, budget: &mut DyadicBudget| {
            budget.reserve(x.numerator().bits().max(x.denominator().bits()), 2, false)?;
            Ok::<_, IntervalError>((
                BigInt::from(x.numerator().clone()),
                BigInt::from(x.denominator().clone()),
            ))
        };
        let (an, ad) = convert(a, budget)?;
        let (bn, bd) = convert(b, budget)?;
        image(
            End::Finite(&an, &ad, 0),
            End::Finite(&bn, &bd, 0),
            self.signs(),
            &Power::new(numerator, denominator, budget)?,
            precision,
            budget,
        )
    }
}
fn trivial(
    signs: SignSet,
    n: &BigInt,
    d: &BigInt,
    precision: i32,
    budget: &mut DyadicBudget,
) -> Result<Option<Image>, IntervalError> {
    if d.is_zero() {
        return Err(IntervalError::ContainsZero);
    }
    if signs == SignSet::Empty {
        return Ok(Some(RangeEnclosure {
            range: IntervalSet2::empty(),
            domain: DomainCoverage::Complete,
        }));
    }
    if n.is_zero() {
        budget.reserve(1, 1, false)?;
        return Ok(Some(RangeEnclosure {
            range: IntervalSet2::from_interval(Extended::from_finite(if precision > 0 {
                DyadicInterval::new(BigInt::zero(), BigInt::one(), precision)?
            } else {
                DyadicInterval::point(BigInt::one(), 0)?
            })),
            domain: DomainCoverage::Complete,
        }));
    }
    Ok(None)
}
fn image(
    a: End<'_>,
    b: End<'_>,
    signs: SignSet,
    power: &Power,
    precision: i32,
    budget: &mut DyadicBudget,
) -> Result<Image, IntervalError> {
    let admit_negative = power.denominator.is_odd() && signs.contains_negative();
    let admit_positive = signs.contains_positive();
    let admit_zero = !power.negative && signs.contains_zero();
    if !admit_negative && !admit_positive && !admit_zero {
        return Err(IntervalError::EmptyDomain);
    }
    let domain = if (signs.contains_negative() && !admit_negative)
        || (signs.contains_zero() && !admit_zero)
    {
        DomainCoverage::Partial
    } else {
        DomainCoverage::Complete
    };
    budget.reserve(1, 1, false)?;
    let zero = BigInt::zero();
    let one = BigInt::one();
    let z = End::Finite(&zero, &one, 0);
    let negative = if admit_negative {
        let range = magnitude(
            if signs.contains_zero() { z } else { b },
            a,
            power,
            precision,
            budget,
        )?;
        if power.numerator.is_odd() {
            range.neg(budget)?
        } else {
            range
        }
    } else {
        Extended::empty()
    };
    let positive = if admit_positive {
        magnitude(
            if signs.contains_zero() { z } else { a },
            b,
            power,
            precision,
            budget,
        )?
    } else if admit_zero {
        Extended::from_finite(DyadicInterval::point(zero, 0)?)
    } else {
        Extended::empty()
    };
    Ok(RangeEnclosure {
        range: IntervalSet2::new(negative, positive, budget)?,
        domain,
    })
}
// On either sign half-line, absolute source endpoints are nonnegative and
// ordered. Negative powers reverse magnitude order; odd numerators reflect it.
fn magnitude(
    a: End<'_>,
    b: End<'_>,
    power: &Power,
    precision: i32,
    budget: &mut DyadicBudget,
) -> Result<Extended, IntervalError> {
    let (a, b) = if power.negative { (b, a) } else { (a, b) };
    let lower = endpoint(a, power, precision, budget)?.expect("lower magnitude bound is finite");
    let upper = endpoint(b, power, precision, budget)?;
    let (lo, _, e) = lower.bounds().expect("nonempty point image");
    match upper {
        None => {
            budget.reserve(lo.bits().max(1), 1, false)?;
            Extended::above(DyadicInterval::point(lo.clone(), e)?)
        }
        Some(upper) => {
            let (_, hi, f) = upper.bounds().expect("nonempty point image");
            crate::extended_dyadic::closed_bounds(lo, e, hi, f, budget)
        }
    }
}
// None denotes positive infinity, not an omitted result.
fn endpoint(
    x: End<'_>,
    power: &Power,
    precision: i32,
    budget: &mut DyadicBudget,
) -> Result<Option<DyadicInterval>, IntervalError> {
    match x {
        End::Infinity if !power.negative => Ok(None),
        End::Infinity => Ok(Some(DyadicInterval::point(BigInt::zero(), 0)?)),
        End::Finite(n, _, _) if n.is_zero() && power.negative => Ok(None),
        End::Finite(n, _, _) if n.is_zero() => Ok(Some(DyadicInterval::point(BigInt::zero(), 0)?)),
        End::Finite(n, d, e) => point(n, d, e, power, precision, budget).map(Some),
    }
}
fn point(
    n: &BigInt,
    d: &BigInt,
    e: i32,
    power: &Power,
    precision: i32,
    budget: &mut DyadicBudget,
) -> Result<DyadicInterval, IntervalError> {
    let nt = n.trailing_zeros().expect("nonzero numerator");
    let dt = d.trailing_zeros().expect("positive denominator");
    budget.reserve(n.bits().max(d.bits()).max(65), 4, false)?;
    let n = n.abs() >> usize::try_from(nt).map_err(|_| IntervalError::Overflow)?;
    let d = d >> usize::try_from(dt).map_err(|_| IntervalError::Overflow)?;
    let scale = BigInt::from(e) + BigInt::from(nt) - BigInt::from(dt);
    let (nr, ne) = root(&n, &power.denominator, budget)?;
    let (dr, de) = root(&d, &power.denominator, budget)?;
    budget.reserve(
        scale.bits().max(power.denominator.bits()).saturating_add(1),
        4,
        true,
    )?;
    let degree = BigInt::from(power.denominator.clone());
    let (root_scale, remainder) = scale.div_rem(&degree);
    let exact_root = ne && de && remainder.is_zero();
    let (n, d, scale) = if exact_root {
        (nr, dr, root_scale)
    } else {
        (n, d, scale)
    };
    let n = integer_power(&n, &power.numerator, budget)?;
    let d = integer_power(&d, &power.numerator, budget)?;
    budget.reserve(
        scale
            .bits()
            .saturating_add(power.numerator.bits())
            .saturating_add(1),
        3,
        true,
    )?;
    let scale = scale * BigInt::from(power.numerator.clone());
    let (n, d, scale) = if power.negative {
        (d, n, -scale)
    } else {
        (n, d, scale)
    };
    if exact_root
        && d.is_one()
        && let Some(e) = scale.to_i32()
        && precision <= e
    {
        return DyadicInterval::point(n, e);
    }
    budget.reserve(
        scale
            .bits()
            .max(power.denominator.bits().saturating_add(32))
            .saturating_add(2),
        4,
        true,
    )?;
    let shift = if exact_root {
        scale - precision
    } else {
        scale - degree * precision
    };
    // If strictly smaller than one grid unit, no exponent-sized denominator is needed.
    if &shift + BigInt::from(n.bits()) < BigInt::from(d.bits()) {
        budget.reserve(1, 2, false)?;
        return DyadicInterval::new(BigInt::zero(), BigInt::one(), precision);
    }
    // Check the declared size reservation even when the shared quotient's
    // signed scale cannot represent this request. Overflow remains distinct.
    let shift = match shift.to_i64() {
        Some(shift) => shift,
        None => {
            if shift.abs() + BigInt::from(n.bits().max(d.bits())) > BigInt::from(budget.max_bits())
            {
                return Err(IntervalError::SizeExhausted);
            }
            return Err(IntervalError::Overflow);
        }
    };
    let (floor, ceil) = crate::dyadic::scaled_quotient(&n, &d, shift, budget)?;
    if exact_root {
        return DyadicInterval::new(floor, ceil, precision);
    }
    let (lower, exact) = root(&floor, &power.denominator, budget)?;
    budget.reserve(lower.bits().saturating_add(1), 2, false)?;
    let upper = if exact && floor == ceil {
        lower.clone()
    } else {
        &lower + 1
    };
    DyadicInterval::new(lower, upper, precision)
}
// Exact nonnegative integer power, with arbitrary finite exponent. Bounds are
// reserved before the backend call, including binary exponentiation stages.
fn integer_power(
    x: &BigInt,
    p: &BigUint,
    budget: &mut DyadicBudget,
) -> Result<BigInt, IntervalError> {
    if x.bits() <= 1 || p.is_one() {
        budget.reserve(x.bits().max(1), 1, false)?;
        return Ok(x.clone());
    }
    let bits = x
        .bits()
        .checked_mul(p.to_u64().ok_or(IntervalError::SizeExhausted)?)
        .ok_or(IntervalError::SizeExhausted)?;
    budget.reserve(bits, p.bits().saturating_mul(2), true)?;
    Ok(Pow::pow(x, p))
}
// floor(q-th root(x)) and an exactness witness. Bisection maintains
// lo^q <= x < hi^q. Degree comparisons avoid materializing enormous powers
// when the root is one. Every iteration consumes the shared work budget.
fn root(
    x: &BigInt,
    q: &BigUint,
    budget: &mut DyadicBudget,
) -> Result<(BigInt, bool), IntervalError> {
    budget.reserve(x.bits().max(q.bits()).max(1), 2, false)?;
    if x.bits() <= 1 || q.is_one() {
        return Ok((x.clone(), true));
    }
    let degree = q.to_u64().unwrap_or(u64::MAX);
    if degree >= x.bits() {
        return Ok((BigInt::one(), false));
    }
    let bits = x.bits();
    let shift = (bits - 1) / degree;
    budget.reserve(shift.saturating_add(2), 4, false)?;
    let mut lo = BigInt::one() << usize::try_from(shift).map_err(|_| IntervalError::Overflow)?;
    let mut hi = &lo << 1usize;
    loop {
        budget.reserve(hi.bits().saturating_add(1), 4, false)?;
        if &hi - &lo == BigInt::one() {
            break;
        }
        let mid = (&lo + &hi) >> 1usize;
        if integer_power(&mid, q, budget)? <= *x {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let exact = integer_power(&lo, q, budget)? == *x;
    Ok((lo, exact))
}
