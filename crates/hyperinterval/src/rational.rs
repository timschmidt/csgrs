use crate::bounds::Bounds;
use crate::{IntervalError, SignSet};
use hyperreal::Rational;

/// An exact closed real interval with rational endpoints, or the empty set.
///
/// Arithmetic uses Hyperreal's arbitrary-precision rationals. It has no implicit
/// work limit; callers requiring a bounded arithmetic stage should use the
/// budgeted dyadic backend. There is no stored nominal value or scalar ordering.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RationalInterval {
    bounds: Option<Bounds<Rational>>,
}

impl RationalInterval {
    /// Checks endpoint order without changing either endpoint.
    pub fn new(lower: Rational, upper: Rational) -> Result<Self, IntervalError> {
        Ok(Self {
            bounds: Some(Bounds::new(lower, upper)?),
        })
    }

    /// The empty set.
    pub const fn empty() -> Self {
        Self { bounds: None }
    }

    /// A singleton at an exact rational value.
    pub fn point(value: Rational) -> Self {
        Self {
            bounds: Some(Bounds::point(value)),
        }
    }

    /// The set `nominal ± uncertainty`; negative uncertainty is invalid.
    pub fn from_nominal_uncertainty(
        nominal: &Rational,
        uncertainty: &Rational,
    ) -> Result<Self, IntervalError> {
        if uncertainty.is_negative() {
            return Err(IntervalError::NegativeRadius);
        }
        Ok(Self::ordered(nominal - uncertainty, nominal + uncertainty))
    }

    // Every caller proves ordering from already ordered inputs.
    pub(crate) fn ordered(lower: Rational, upper: Rational) -> Self {
        Self {
            bounds: Some(Bounds::ordered(lower, upper)),
        }
    }

    fn view(&self) -> Option<Bounds<&Rational>> {
        self.bounds.as_ref().map(Bounds::as_refs)
    }

    /// Whether the set is empty.
    pub fn is_empty(&self) -> bool {
        self.bounds.is_none()
    }

    /// Whether exactly one real value is represented.
    pub fn is_singleton(&self) -> bool {
        self.view().is_some_and(Bounds::is_singleton)
    }

    /// Borrowed exact endpoints, or `None` for empty.
    #[inline]
    pub fn bounds(&self) -> Option<(&Rational, &Rational)> {
        self.view().map(Bounds::endpoints)
    }

    /// Consumes the carrier and returns its owned exact endpoints, without cloning.
    #[inline]
    pub fn into_bounds(self) -> Option<(Rational, Rational)> {
        self.bounds.map(Bounds::endpoints)
    }

    /// Exact lower endpoint, or `None` for empty.
    pub fn lower(&self) -> Option<&Rational> {
        self.bounds().map(|b| b.0)
    }

    /// Exact upper endpoint, or `None` for empty.
    pub fn upper(&self) -> Option<&Rational> {
        self.bounds().map(|b| b.1)
    }

    /// Whether this set contains an exact rational point.
    pub fn contains_point(&self, point: &Rational) -> bool {
        self.view().is_some_and(|b| b.contains(point))
    }

    /// Set inclusion; empty is included in every set.
    pub fn subset_of(&self, other: &Self) -> bool {
        match (self.view(), other.view()) {
            (None, _) => true,
            (Some(_), None) => false,
            (Some(a), Some(b)) => a.subset(b),
        }
    }

    /// Strict endpoint inclusion in the real interior; empty is included.
    pub fn interior_subset_of(&self, other: &Self) -> bool {
        match (self.view(), other.view()) {
            (None, _) => true,
            (Some(_), None) => false,
            (Some(a), Some(b)) => a.interior_subset(b),
        }
    }

    /// Whether both sets are nonempty and this set lies strictly left of `other`.
    pub fn strictly_left_of(&self, other: &Self) -> bool {
        match (self.view(), other.view()) {
            (Some(a), Some(b)) => a.left_of(b),
            _ => false,
        }
    }

    /// Whether the intersection is empty, including when either operand is empty.
    pub fn disjoint(&self, other: &Self) -> bool {
        match (self.view(), other.view()) {
            (Some(a), Some(b)) => a.left_of(b) || b.left_of(a),
            _ => true,
        }
    }

    /// Whether the sets share a point, including a common endpoint.
    pub fn overlaps(&self, other: &Self) -> bool {
        !self.disjoint(other)
    }

    /// Smallest closed interval containing both sets.
    pub fn hull(&self, other: &Self) -> Self {
        Self {
            bounds: match (self.view(), other.view()) {
                (Some(a), Some(b)) => Some(a.hull(b).cloned()),
                (a, b) => a.or(b).map(Bounds::cloned),
            },
        }
    }

    /// Exact intersection; disjoint inputs produce empty.
    pub fn intersection(&self, other: &Self) -> Self {
        Self {
            bounds: match (self.view(), other.view()) {
                (Some(a), Some(b)) => a.intersection(b).map(Bounds::cloned),
                _ => None,
            },
        }
    }

    /// Signs of members of this set; this does not decide an enclosed scalar.
    pub fn signs(&self) -> SignSet {
        self.view()
            .map_or(SignSet::Empty, |b| b.signs(&Rational::zero()))
    }

    /// Exact endpoint distance, or `None` for empty.
    pub fn width(&self) -> Option<Rational> {
        self.bounds().map(|(a, b)| b - a)
    }

    /// Exact rational midpoint, or `None` for empty.
    pub fn midpoint(&self) -> Option<Rational> {
        self.bounds().map(|(a, b)| (a + b) / Rational::from(2))
    }

    /// Exact half-width, or `None` for empty.
    pub fn radius(&self) -> Option<Rational> {
        self.width().map(|w| w / Rational::from(2))
    }

    /// Exact pointwise absolute-value image, with zero minimum across zero.
    pub fn abs(&self) -> Self {
        match self.bounds() {
            None => Self::empty(),
            Some((a, _)) if !a.is_negative() => self.clone(),
            Some((_, b)) if b.is_negative() => self.neg(),
            Some((a, b)) => {
                let upper = -a;
                Self::ordered(
                    Rational::zero(),
                    if upper >= *b { upper } else { b.clone() },
                )
            }
        }
    }

    /// Exact image of `min(x, y)` for independently chosen members.
    /// Either empty operand gives empty; no real-domain points are excluded.
    pub fn min_image(&self, other: &Self) -> Self {
        Self {
            bounds: match (self.view(), other.view()) {
                (Some(a), Some(b)) => Some(a.min_image(b).cloned()),
                _ => None,
            },
        }
    }

    /// Exact image of `max(x, y)` for independently chosen members.
    /// Either empty operand gives empty; this differs from hull.
    pub fn max_image(&self, other: &Self) -> Self {
        Self {
            bounds: match (self.view(), other.view()) {
                (Some(a), Some(b)) => Some(a.max_image(b).cloned()),
                _ => None,
            },
        }
    }

    /// Exact Minkowski sum.
    pub fn add(&self, other: &Self) -> Self {
        match (self.bounds(), other.bounds()) {
            (Some((a, b)), Some((c, d))) => Self::ordered(a + c, b + d),
            _ => Self::empty(),
        }
    }

    /// Exact Minkowski difference; repeated operands are independent sets.
    pub fn sub(&self, other: &Self) -> Self {
        match (self.bounds(), other.bounds()) {
            (Some((a, b)), Some((c, d))) => Self::ordered(a - d, b - c),
            _ => Self::empty(),
        }
    }

    /// Exact reflected set.
    pub fn neg(&self) -> Self {
        self.bounds()
            .map_or_else(Self::empty, |(a, b)| Self::ordered(-b, -a))
    }

    /// Exact product hull, using the sign-specialized rational kernel.
    #[inline]
    pub fn mul(&self, other: &Self) -> Self {
        match (self.bounds(), other.bounds()) {
            (Some((a, b)), Some((c, d))) => {
                let (lower, upper) = product_hull(a, b, c, d);
                Self::ordered(lower, upper)
            }
            _ => Self::empty(),
        }
    }

    /// Exact image of pointwise squaring, retaining dependency at zero.
    pub fn square(&self) -> Self {
        match self.bounds() {
            None => Self::empty(),
            Some((a, b)) => {
                let aa = a * a;
                let bb = b * b;
                if self.contains_point(&Rational::zero()) {
                    Self::ordered(Rational::zero(), if aa >= bb { aa } else { bb })
                } else if a.is_negative() {
                    Self::ordered(bb, aa)
                } else {
                    Self::ordered(aa, bb)
                }
            }
        }
    }

    /// Exact image of a signed integer power, preserving pointwise dependence.
    ///
    /// Empty maps to empty. Exponent zero maps nonempty input to one, including
    /// zero. Negative powers reject every zero-containing interval. A connected
    /// nonzero interval stays on one real half-line, so endpoint monotonicity
    /// gives its exact image. Positive even powers include zero when needed.
    ///
    /// Reuses Hyperreal's exact eager-power kernel and growth guard. Refusals
    /// retain the original `Problem` in `IntervalError::Scalar` and its source.
    /// Currently nontrivial exponents above magnitude 255 require an estimated
    /// combined numerator/denominator size of at most 65,536 bits. Smaller
    /// exponents bound amplification relative to the input; this is not a total
    /// work/heap-byte limit or cancellation contract. Zero/unit endpoints have
    /// constant-size results even at i64 exponent extremes. No failure publishes
    /// a partial image, and no approximate power replaces an exhausted result.
    pub fn checked_pow(&self, exponent: i64) -> Result<Self, IntervalError> {
        let Some((a, b)) = self.bounds() else {
            return Ok(Self::empty());
        };
        match exponent {
            0 => return Ok(Self::point(Rational::one())),
            1 => return Ok(self.clone()),
            2 => return Ok(self.square()),
            -1 => return self.reciprocal(),
            _ => {}
        }
        if exponent < 0 && self.contains_point(&Rational::zero()) {
            return Err(IntervalError::ContainsZero);
        }
        let even = exponent.unsigned_abs().is_multiple_of(2);
        if exponent > 0 && even && self.contains_point(&Rational::zero()) {
            // Zero is the minimum. Select the maximum magnitude before powering:
            // an irrelevant endpoint can exceed the eager guard even when the
            // complete image has small exact endpoints (e.g. [-1/3,1]^large_even).
            let reflected = -a;
            let maximum = if reflected >= *b {
                reflected
            } else {
                b.clone()
            };
            let upper = maximum
                .powi(exponent.into())
                .map_err(IntervalError::Scalar)?;
            return Ok(Self::ordered(Rational::zero(), upper));
        }
        let aa = a
            .clone()
            .powi(exponent.into())
            .map_err(IntervalError::Scalar)?;
        if a == b {
            return Ok(Self::point(aa));
        }
        let bb = b
            .clone()
            .powi(exponent.into())
            .map_err(IntervalError::Scalar)?;
        // Positive odd powers increase everywhere. Positive even powers
        // decrease on the negative half-line. Negative odd powers decrease on
        // either admitted half-line; negative even powers increase only on the
        // negative half-line. No output comparison is needed on these branches.
        let increasing = if exponent > 0 {
            !even || !a.is_negative()
        } else {
            even && a.is_negative()
        };
        Ok(if increasing {
            Self::ordered(aa, bb)
        } else {
            Self::ordered(bb, aa)
        })
    }

    /// Exact reciprocal; a zero-containing input is rejected.
    pub fn reciprocal(&self) -> Result<Self, IntervalError> {
        if self.contains_point(&Rational::zero()) {
            return Err(IntervalError::ContainsZero);
        }
        Ok(self.bounds().map_or_else(Self::empty, |(a, b)| {
            Self::ordered(Rational::one() / b, Rational::one() / a)
        }))
    }

    /// Exact quotient hull; validates the denominator even for an empty numerator.
    pub fn div(&self, other: &Self) -> Result<Self, IntervalError> {
        Ok(self.mul(&other.reciprocal()?))
    }

    /// Exact product for strictly positive inputs; nonpositive lower bounds fail.
    pub fn positive_product(&self, other: &Self) -> Result<Self, IntervalError> {
        self.require_positive()?;
        other.require_positive()?;
        Ok(match (self.bounds(), other.bounds()) {
            (Some((a, b)), Some((c, d))) => Self::ordered(a * c, b * d),
            _ => Self::empty(),
        })
    }

    /// Exact quotient for strictly positive inputs; empty remains a valid set.
    pub fn positive_quotient(&self, other: &Self) -> Result<Self, IntervalError> {
        self.require_positive()?;
        other.require_positive()?;
        Ok(match (self.bounds(), other.bounds()) {
            (Some((a, b)), Some((c, d))) => Self::ordered(a / d, b / c),
            _ => Self::empty(),
        })
    }

    fn require_positive(&self) -> Result<(), IntervalError> {
        if self.lower().is_some_and(|a| !a.is_positive()) {
            Err(IntervalError::NonPositiveLowerBound)
        } else {
            Ok(())
        }
    }

    /// Exact image under `scale * x + offset`.
    pub fn affine(&self, scale: &Rational, offset: &Rational) -> Self {
        self.mul(&Self::point(scale.clone()))
            .add(&Self::point(offset.clone()))
    }
}

// Preserve the sequence of signs, products and comparisons from Hypersolve's
// current kernel. Consumer migration is gated by differential/dispatch evidence.
#[inline]
fn product_hull(
    left_lower: &Rational,
    left_upper: &Rational,
    right_lower: &Rational,
    right_upper: &Rational,
) -> (Rational, Rational) {
    if !left_lower.is_negative() {
        if !right_lower.is_negative() {
            return (left_lower * right_lower, left_upper * right_upper);
        }
        if !right_upper.is_positive() {
            return (left_upper * right_lower, left_lower * right_upper);
        }
        return (left_upper * right_lower, left_upper * right_upper);
    }
    if !left_upper.is_positive() {
        if !right_lower.is_negative() {
            return (left_lower * right_upper, left_upper * right_lower);
        }
        if !right_upper.is_positive() {
            return (left_upper * right_upper, left_lower * right_lower);
        }
        return (left_lower * right_upper, left_lower * right_lower);
    }
    if !right_lower.is_negative() {
        return (left_lower * right_upper, left_upper * right_upper);
    }
    if !right_upper.is_positive() {
        return (left_upper * right_lower, left_lower * right_lower);
    }
    let first_lower = left_lower * right_upper;
    let second_lower = left_upper * right_lower;
    let first_upper = left_lower * right_lower;
    let second_upper = left_upper * right_upper;
    let lower = if first_lower <= second_lower {
        first_lower
    } else {
        second_lower
    };
    let upper = if first_upper >= second_upper {
        first_upper
    } else {
        second_upper
    };
    (lower, upper)
}

#[cfg(feature = "serde")]
impl serde::Serialize for RationalInterval {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.bounds().serialize(serializer)
    }
}

#[cfg(feature = "serde")]
impl<'de> serde::Deserialize<'de> for RationalInterval {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        match Option::<(Rational, Rational)>::deserialize(deserializer)? {
            None => Ok(Self::empty()),
            Some((lower, upper)) => Self::new(lower, upper),
        }
        .map_err(serde::de::Error::custom)
    }
}
