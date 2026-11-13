//! Monotone inverse images assembled from certified native scalar images.
use crate::{
    DomainCoverage, DyadicBudget, DyadicInterval, ExtendedInterval, IntervalError, RangeEnclosure,
};
use num_bigint::BigInt;
use num_traits::{One, Zero};

type FiniteImage = RangeEnclosure<DyadicInterval>;

#[derive(Clone, Copy)]
enum Function {
    Asin,
    Acos,
    Acosh,
    Atanh,
}

impl DyadicInterval {
    /// Outward principal inverse-sine image, on the requested dyadic grid.
    ///
    /// The real domain is [-1,1]. Excluded members report Partial coverage;
    /// no defined member returns EmptyDomain. Empty maps to empty/Complete.
    /// Endpoint monotonicity and `2*atan(x/(1+sqrt(1-x*x)))` certify the image,
    /// including both endpoints. Bounds need not be tight. All arithmetic
    /// shares the supplied DyadicBudget; its accounting exclusions still apply.
    pub fn asin(
        &self,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<FiniteImage, IntervalError> {
        self.inverse_finite(Function::Asin, precision, budget)
    }

    /// Outward principal inverse-cosine image, with domain [-1,1].
    ///
    /// Uses decreasing endpoint order and `pi/2 - asin(x)`. Coverage, empty,
    /// budget and non-tight enclosure semantics are those of [`Self::asin`].
    pub fn acos(
        &self,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<FiniteImage, IntervalError> {
        self.inverse_finite(Function::Acos, precision, budget)
    }

    /// Outward principal inverse-hyperbolic-cosine image, with domain [1,infinity).
    ///
    /// Uses monotonicity and `log(x + sqrt(x-1)*sqrt(x+1))`. Coverage, empty,
    /// budget and non-tight enclosure semantics are those of [`Self::asin`].
    pub fn acosh(
        &self,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<FiniteImage, IntervalError> {
        self.inverse_finite(Function::Acosh, precision, budget)
    }

    fn inverse_finite(
        &self,
        function: Function,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<FiniteImage, IntervalError> {
        if self.is_empty() {
            return Ok(RangeEnclosure {
                range: Self::empty(),
                domain: DomainCoverage::Complete,
            });
        }
        let source = restrict(self, function, budget)?;
        let (a, b, exponent) = source.range.bounds().expect("nonempty restricted domain");
        let lower = endpoint(a, exponent, function, precision, budget)?;
        let upper = if a == b {
            None
        } else {
            Some(endpoint(b, exponent, function, precision, budget)?)
        };
        let upper = upper.as_ref().unwrap_or(&lower);
        let (lo, hi) = if matches!(function, Function::Acos) {
            (upper, &lower)
        } else {
            (&lower, upper)
        };
        let (a, e) = lo.lower().expect("nonempty lower image");
        let (b, f) = hi.upper().expect("nonempty upper image");
        Ok(RangeEnclosure {
            range: crate::extended_dyadic::closed_bounds(a, e, b, f, budget)?
                .finite_range(budget)?,
            domain: source.domain,
        })
    }

    /// Outward inverse-hyperbolic-tangent image on its open real domain (-1,1).
    ///
    /// Uses monotonicity and `(log(1+x)-log(1-x))/2`. Inputs reaching a domain
    /// endpoint from inside have an explicit unbounded limit and Partial
    /// coverage. No defined source member returns EmptyDomain; empty maps to
    /// empty/Complete. Finite bounds lie on the requested grid but need not be
    /// tight. All arithmetic shares the caller's DyadicBudget.
    pub fn atanh(
        &self,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<RangeEnclosure<ExtendedInterval<Self>>, IntervalError> {
        if self.is_empty() {
            return Ok(RangeEnclosure {
                range: ExtendedInterval::empty(),
                domain: DomainCoverage::Complete,
            });
        }
        let source = restrict(self, Function::Atanh, budget)?;
        let (a, b, exponent) = source.range.bounds().expect("nonempty restricted domain");
        budget.reserve(1, 2, false)?;
        let one = BigInt::one();
        let minus_one = -BigInt::one();
        let at_lower = crate::dyadic_order::compare(a, exponent, &minus_one, 0, budget)?.is_eq();
        let at_upper = crate::dyadic_order::compare(b, exponent, &one, 0, budget)?.is_eq();
        if a == b && (at_lower || at_upper) {
            return Err(IntervalError::EmptyDomain);
        }
        if a == b {
            return Ok(RangeEnclosure {
                range: ExtendedInterval::from_finite(endpoint(
                    a,
                    exponent,
                    Function::Atanh,
                    precision,
                    budget,
                )?),
                domain: source.domain,
            });
        }
        let lower = if at_lower {
            None
        } else {
            Some(endpoint(a, exponent, Function::Atanh, precision, budget)?)
        };
        let upper = if at_upper {
            None
        } else {
            Some(endpoint(b, exponent, Function::Atanh, precision, budget)?)
        };
        let range = match (lower, upper) {
            (None, None) => ExtendedInterval::entire(),
            (None, Some(upper)) => {
                let (b, e) = upper.upper().expect("nonempty upper image");
                ExtendedInterval::below(copy_point(b, e, budget)?)?
            }
            (Some(lower), None) => {
                let (a, e) = lower.lower().expect("nonempty lower image");
                ExtendedInterval::above(copy_point(a, e, budget)?)?
            }
            (Some(lower), Some(upper)) => {
                let (a, e) = lower.lower().expect("nonempty lower image");
                let (b, f) = upper.upper().expect("nonempty upper image");
                crate::extended_dyadic::closed_bounds(a, e, b, f, budget)?
            }
        };
        Ok(RangeEnclosure {
            range,
            domain: if at_lower || at_upper {
                DomainCoverage::Partial
            } else {
                source.domain
            },
        })
    }
}

fn restrict(
    source: &DyadicInterval,
    function: Function,
    budget: &mut DyadicBudget,
) -> Result<FiniteImage, IntervalError> {
    budget.reserve(1, 2, false)?;
    let one = DyadicInterval::point(1.into(), 0)?;
    let range = if matches!(function, Function::Acosh) {
        if source.strictly_left_of(&one, budget)? {
            return Err(IntervalError::EmptyDomain);
        }
        source.max_image(&one, budget)?
    } else {
        budget.reserve(1, 2, false)?;
        source.intersection(&DyadicInterval::new((-1).into(), 1.into(), 0)?, budget)?
    };
    if range.is_empty() {
        return Err(IntervalError::EmptyDomain);
    }
    let domain = if source.subset_of(&range, budget)? {
        DomainCoverage::Complete
    } else {
        DomainCoverage::Partial
    };
    Ok(RangeEnclosure { range, domain })
}

fn copy_point(
    n: &BigInt,
    exponent: i32,
    budget: &mut DyadicBudget,
) -> Result<DyadicInterval, IntervalError> {
    budget.reserve(n.bits(), 2, false)?;
    DyadicInterval::point(n.clone(), exponent)
}

fn endpoint(
    n: &BigInt,
    exponent: i32,
    function: Function,
    precision: i32,
    budget: &mut DyadicBudget,
) -> Result<DyadicInterval, IntervalError> {
    let x = copy_point(n, exponent, budget)?;
    if n.is_zero() && matches!(function, Function::Asin | Function::Atanh) {
        return Ok(x);
    }
    budget.reserve(2, 4, false)?;
    let one = DyadicInterval::point(1.into(), 0)?;
    let two = DyadicInterval::point(2.into(), 0)?;
    if matches!(function, Function::Acos | Function::Acosh)
        && x.compare_singleton(&one, budget)?.is_eq()
    {
        return DyadicInterval::point(0.into(), 0);
    }
    let result = match function {
        Function::Asin | Function::Acos => {
            let root = one
                .sub(&x.square(budget)?, budget)?
                .sqrt(precision, budget)?;
            // Restriction proves 1-x^2 >= 0 before numerical enclosure.
            debug_assert_eq!(root.domain, DomainCoverage::Complete);
            let argument = x.div(&one.add(&root.range, budget)?, precision, budget)?;
            let asin = argument.atan(precision, budget)?.range.mul(&two, budget)?;
            if matches!(function, Function::Acos) {
                one.atan(precision, budget)?
                    .range
                    .mul(&two, budget)?
                    .sub(&asin, budget)?
            } else {
                asin
            }
        }
        Function::Acosh => {
            let left = x.sub(&one, budget)?.sqrt(precision, budget)?;
            let right = x.add(&one, budget)?.sqrt(precision, budget)?;
            debug_assert_eq!(left.domain, DomainCoverage::Complete);
            debug_assert_eq!(right.domain, DomainCoverage::Complete);
            let image = x
                .add(&left.range.mul(&right.range, budget)?, budget)?
                .log(precision, budget)?;
            debug_assert_eq!(image.domain, DomainCoverage::Complete);
            image.range.finite_range(budget)?
        }
        Function::Atanh => {
            let left = one.add(&x, budget)?.log(precision, budget)?;
            let right = one.sub(&x, budget)?.log(precision, budget)?;
            debug_assert_eq!(left.domain, DomainCoverage::Complete);
            debug_assert_eq!(right.domain, DomainCoverage::Complete);
            left.range
                .finite_range(budget)?
                .sub(&right.range.finite_range(budget)?, budget)?
                .div(&two, precision, budget)?
        }
    };
    result.regrid(precision, budget)
}
