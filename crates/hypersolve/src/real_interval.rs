//! Conservative intervals over arbitrary exact scalar endpoints.
//!
//! Enclosures use only certified STRICT sign and order decisions. An
//! unresolved enclosure returns `None`; terminal approximation belongs to
//! the consuming predicate, never to interval arithmetic.

use std::cmp::Ordering;

use hyperreal::{Rational as HyperRational, Real, RealSign, ZeroKnowledge};

use crate::curve_resultant::BivariatePolynomial;

/// Exact STRICT sign: a structural zero, else a certified predicate sign.
pub(crate) fn strict_real_sign(value: &Real) -> Option<RealSign> {
    if value.zero_status() == ZeroKnowledge::Zero {
        return Some(RealSign::Zero);
    }
    match crate::classify_real_sign_predicate(value, hyperlimit::PredicatePolicy::STRICT).value()? {
        crate::PredicateSign::Negative => Some(RealSign::Negative),
        crate::PredicateSign::Zero => Some(RealSign::Zero),
        crate::PredicateSign::Positive => Some(RealSign::Positive),
    }
}

/// Exact STRICT order: identity and rational shortcuts, then a certified
/// comparison predicate, then the certified sign of the difference.
pub(crate) fn strict_compare_reals(left: &Real, right: &Real) -> Option<Ordering> {
    if std::ptr::eq(left, right) {
        return Some(Ordering::Equal);
    }
    if let (Some(left), Some(right)) = (left.exact_rational_ref(), right.exact_rational_ref()) {
        return left.partial_cmp(right);
    }
    if let Some(ordering) =
        crate::compare_real_predicate(left, right, hyperlimit::PredicatePolicy::STRICT).value()
    {
        return Some(ordering);
    }
    strict_real_sign(&(left - right)).map(|sign| match sign {
        RealSign::Negative => Ordering::Less,
        RealSign::Zero => Ordering::Equal,
        RealSign::Positive => Ordering::Greater,
    })
}

/// A closed interval `[lower, upper]` with exact endpoints.
#[derive(Clone, Debug, PartialEq)]
pub struct RealInterval {
    /// Lower endpoint.
    pub lower: Real,
    /// Upper endpoint.
    pub upper: Real,
}

impl RealInterval {
    #[inline]
    /// Returns the sign of every value in a certified interval that excludes zero.
    pub fn strict_nonzero_sign(&self) -> Option<RealSign> {
        if Self::compare_to_zero(&self.lower) == Some(Ordering::Greater) {
            Some(RealSign::Positive)
        } else if Self::compare_to_zero(&self.upper) == Some(Ordering::Less) {
            Some(RealSign::Negative)
        } else {
            None
        }
    }

    fn compare_to_zero(value: &Real) -> Option<Ordering> {
        value
            .immediate_sign()
            .map(|sign| match sign {
                RealSign::Negative => Ordering::Less,
                RealSign::Zero => Ordering::Equal,
                RealSign::Positive => Ordering::Greater,
            })
            .or_else(|| strict_compare_reals(value, &Real::zero()))
    }

    /// Returns the exact interval sum.
    pub fn add(&self, other: &Self) -> Self {
        Self {
            lower: &self.lower + &other.lower,
            upper: &self.upper + &other.upper,
        }
    }

    /// Returns the exact interval difference.
    pub fn subtract(&self, other: &Self) -> Self {
        Self {
            lower: &self.lower - &other.upper,
            upper: &self.upper - &other.lower,
        }
    }

    /// Returns a certified product enclosure, or `None` when an endpoint order is undecided.
    pub fn multiply(&self, other: &Self) -> Option<Self> {
        let first_lower = Self::compare_to_zero(&self.lower);
        let first_upper = Self::compare_to_zero(&self.upper);
        let second_lower = Self::compare_to_zero(&other.lower);
        let second_upper = Self::compare_to_zero(&other.upper);
        let first_nonnegative = matches!(first_lower, Some(Ordering::Equal | Ordering::Greater));
        let first_nonpositive = matches!(first_upper, Some(Ordering::Less | Ordering::Equal));
        let second_nonnegative = matches!(second_lower, Some(Ordering::Equal | Ordering::Greater));
        let second_nonpositive = matches!(second_upper, Some(Ordering::Less | Ordering::Equal));
        let first_spans_zero =
            first_lower == Some(Ordering::Less) && first_upper == Some(Ordering::Greater);
        let second_spans_zero =
            second_lower == Some(Ordering::Less) && second_upper == Some(Ordering::Greater);
        if first_nonnegative {
            if second_nonnegative {
                return Some(Self {
                    lower: &self.lower * &other.lower,
                    upper: &self.upper * &other.upper,
                });
            } else if second_nonpositive {
                return Some(Self {
                    lower: &self.upper * &other.lower,
                    upper: &self.lower * &other.upper,
                });
            } else if second_spans_zero {
                return Some(Self {
                    lower: &self.upper * &other.lower,
                    upper: &self.upper * &other.upper,
                });
            }
        }
        if first_nonpositive {
            if second_nonnegative {
                return Some(Self {
                    lower: &self.lower * &other.upper,
                    upper: &self.upper * &other.lower,
                });
            } else if second_nonpositive {
                return Some(Self {
                    lower: &self.upper * &other.upper,
                    upper: &self.lower * &other.lower,
                });
            } else if second_spans_zero {
                return Some(Self {
                    lower: &self.lower * &other.upper,
                    upper: &self.lower * &other.lower,
                });
            }
        }
        if first_spans_zero && second_nonnegative {
            return Some(Self {
                lower: &self.lower * &other.upper,
                upper: &self.upper * &other.upper,
            });
        }
        if first_spans_zero && second_nonpositive {
            return Some(Self {
                lower: &self.upper * &other.lower,
                upper: &self.lower * &other.lower,
            });
        }

        // Both intervals cross zero. The sums are conservative exact bounds:
        // each lower product is nonpositive and each upper product is
        // nonnegative, so no comparison between deep exact expressions is
        // required merely to select a corner.
        if first_spans_zero && second_spans_zero {
            return Some(Self {
                lower: &self.lower * &other.upper + &self.upper * &other.lower,
                upper: &self.lower * &other.lower + &self.upper * &other.upper,
            });
        }

        let products = [
            &self.lower * &other.lower,
            &self.lower * &other.upper,
            &self.upper * &other.lower,
            &self.upper * &other.upper,
        ];
        Self::from_values(products)
    }

    /// Squares an interval without introducing the dependent cross-products
    /// of generic interval multiplication.
    pub fn square(&self) -> Option<Self> {
        let zero = Real::zero();
        let lower_sign = Self::compare_to_zero(&self.lower)?;
        let upper_sign = Self::compare_to_zero(&self.upper)?;
        if lower_sign != Ordering::Less {
            return Some(Self {
                lower: &self.lower * &self.lower,
                upper: &self.upper * &self.upper,
            });
        }
        if upper_sign != Ordering::Greater {
            return Some(Self {
                lower: &self.upper * &self.upper,
                upper: &self.lower * &self.lower,
            });
        }
        let lower_magnitude = -self.lower.clone();
        let lower_square = &self.lower * &self.lower;
        let upper_square = &self.upper * &self.upper;
        let upper = match strict_compare_reals(&lower_magnitude, &self.upper) {
            Some(Ordering::Greater) => lower_square,
            Some(Ordering::Less | Ordering::Equal) => upper_square,
            None => lower_square + upper_square,
        };
        Some(Self { lower: zero, upper })
    }

    /// Returns a certified quotient enclosure when the denominator excludes zero.
    pub fn divide(&self, other: &Self) -> Option<Self> {
        let denominator_is_positive =
            Self::compare_to_zero(&other.lower) == Some(Ordering::Greater);
        let denominator_is_negative = Self::compare_to_zero(&other.upper) == Some(Ordering::Less);
        if !denominator_is_positive && !denominator_is_negative {
            return None;
        }
        // Reciprocal is strictly decreasing on either side of zero.  Once the
        // denominator interval is certified not to cross zero, its endpoint
        // order therefore supplies the reciprocal bounds directly; asking the
        // generic exact comparator to rediscover that order only replays the
        // denominator DAG and can turn a proved division into uncertainty.
        let reciprocal = Self {
            lower: (Real::one() / &other.upper).ok()?,
            upper: (Real::one() / &other.lower).ok()?,
        };
        self.multiply(&reciprocal)
    }

    /// Encloses a power-basis polynomial over a parameter interval by Horner evaluation.
    pub fn evaluate_power_basis(coefficients: &[Real], parameter: &Self) -> Option<Self> {
        let mut value = Self {
            lower: Real::zero(),
            upper: Real::zero(),
        };
        for coefficient in coefficients.iter().rev() {
            value = value.multiply(parameter)?;
            value.lower += coefficient;
            value.upper += coefficient;
        }
        Some(value)
    }

    /// Encloses a bivariate power-basis polynomial over a parameter box.
    pub fn evaluate_bivariate_power_basis(
        polynomial: &BivariatePolynomial,
        first: &Self,
        second: &Self,
    ) -> Option<Self> {
        let mut value = Self {
            lower: Real::zero(),
            upper: Real::zero(),
        };
        for row in polynomial.coefficients.iter().rev() {
            value = value.multiply(first)?;
            value = value.add(&Self::evaluate_power_basis(row, second)?);
        }
        Some(value)
    }

    /// Encloses the nonnegative square root, optionally replacing symbolic
    /// root endpoints by certified dyadic rationals at `precision`.
    ///
    /// Interval arithmetic needs only outward bounds.  Retaining an exact
    /// `sqrt(Real)` endpoint causes every later product and comparison to
    /// replay that radical DAG; a certified dyadic enclosure carries the
    /// identical proof obligation with substantially smaller arithmetic.
    pub fn nonnegative_square_root(&self, precision: Option<i32>) -> Option<Self> {
        let zero = Real::zero();
        if Self::compare_to_zero(&self.upper)? == Ordering::Less {
            return None;
        }
        let endpoint = |value: &Real, lower: bool| {
            let root = value.clone().sqrt().ok()?;
            let Some(precision) = precision else {
                return Some(root);
            };
            let bounds = root.certified_dyadic_interval(precision)?;
            Some(Real::new(if lower {
                bounds[0].clone()
            } else {
                bounds[1].clone()
            }))
        };
        let lower = match Self::compare_to_zero(&self.lower)? {
            Ordering::Greater => endpoint(&self.lower, true)?,
            Ordering::Equal | Ordering::Less => zero,
        };
        let upper = match Self::compare_to_zero(&self.upper)? {
            Ordering::Greater => endpoint(&self.upper, false)?,
            Ordering::Equal => Real::zero(),
            Ordering::Less => return None,
        };
        Some(Self { lower, upper })
    }

    /// Returns the certified hull of exact values.
    pub fn from_values<const N: usize>(values: [Real; N]) -> Option<Self> {
        let mut values = values.into_iter();
        let first = values.next()?;
        let mut lower = first.clone();
        let mut upper = first;
        for value in values {
            if strict_compare_reals(&value, &lower)? == Ordering::Less {
                lower = value.clone();
            }
            if strict_compare_reals(&value, &upper)? == Ordering::Greater {
                upper = value;
            }
        }
        Some(Self { lower, upper })
    }
}

/// Certifies one strict sign of a polynomial whose Bernstein controls are
/// rational intervals `[lower, upper]`, by bounded de Casteljau subdivision.
/// Returns `None` when subdivision cannot certify a single sign.
pub fn rational_interval_bernstein_strict_sign(
    controls: Vec<[HyperRational; 2]>,
) -> Option<RealSign> {
    let zero = HyperRational::zero();
    let mut pending = vec![(controls, 0_u8)];
    let mut certified_sign = None;
    let mut visited = 0_usize;
    while let Some((controls, depth)) = pending.pop() {
        visited += 1;
        let first = controls.first()?;
        let last = controls.last()?;
        let endpoints_positive = first[0] > zero && last[0] > zero;
        let endpoints_negative = first[1] < zero && last[1] < zero;
        let segment_sign =
            if endpoints_positive && controls.iter().all(|control| control[0] >= zero) {
                Some(RealSign::Positive)
            } else if endpoints_negative && controls.iter().all(|control| control[1] <= zero) {
                Some(RealSign::Negative)
            } else {
                None
            };
        if let Some(segment_sign) = segment_sign {
            match certified_sign {
                Some(previous) if previous != segment_sign => return None,
                Some(_) => {}
                None => certified_sign = Some(segment_sign),
            }
            continue;
        }
        if depth == 10 || visited >= 256 {
            return None;
        }

        let mut work = controls;
        let degree = work.len().saturating_sub(1);
        let mut left = Vec::with_capacity(work.len());
        let mut right = Vec::with_capacity(work.len());
        left.push(work[0].clone());
        right.push(work[degree].clone());
        for level in 1..=degree {
            for index in 0..=degree - level {
                work[index] = [
                    HyperRational::average_pair(&work[index][0], &work[index + 1][0]),
                    HyperRational::average_pair(&work[index][1], &work[index + 1][1]),
                ];
            }
            left.push(work[0].clone());
            right.push(work[degree - level].clone());
        }
        right.reverse();
        pending.push((right, depth + 1));
        pending.push((left, depth + 1));
    }
    certified_sign
}

/// Whether two certified signs are strictly positive and strictly negative.
pub fn strict_signs_are_opposite(first: Option<RealSign>, second: Option<RealSign>) -> bool {
    matches!(
        (first, second),
        (Some(RealSign::Positive), Some(RealSign::Negative))
            | (Some(RealSign::Negative), Some(RealSign::Positive))
    )
}
