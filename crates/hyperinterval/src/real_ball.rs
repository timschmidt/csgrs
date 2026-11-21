use crate::{DyadicBudget, DyadicInterval, IntervalError, RationalInterval};
use hyperreal::{Rational, Real};

/// A certified relation of a queried exact point to a closed real ball.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BallRelation {
    /// The queried point belongs to the ball, including its boundary.
    Contained,
    /// The queried point is strictly outside the ball.
    Excluded,
    /// Current certified bounds do not decide membership.
    Unresolved,
}

/// A symbolic exact center with an exact nonnegative rational radius.
///
/// The center remains authoritative; enclosure conversion never replaces it.
/// This initial carrier deliberately has no general product or quotient:
/// those operations need certified magnitude bounds on the centers first.
#[derive(Clone, Debug)]
pub struct RealBall {
    center: Real,
    radius: Rational,
}

impl RealBall {
    /// Validates the radius without asking for an approximation to the center.
    pub fn new(center: Real, radius: Rational) -> Result<Self, IntervalError> {
        if radius.is_negative() {
            return Err(IntervalError::NegativeRadius);
        }
        Ok(Self { center, radius })
    }

    /// The singleton at a symbolic exact center.
    pub fn point(center: Real) -> Self {
        Self {
            center,
            radius: Rational::zero(),
        }
    }

    /// The retained exact center, including Hyperreal's shared construction.
    pub fn center(&self) -> &Real {
        &self.center
    }

    /// The validated nonnegative exact radius.
    pub fn radius(&self) -> &Rational {
        &self.radius
    }

    /// Exact Minkowski sum, with no enclosure or scalar decision.
    pub fn add(&self, other: &Self) -> Self {
        Self {
            center: &self.center + &other.center,
            radius: &self.radius + &other.radius,
        }
    }

    /// Exact Minkowski difference; the radii add because inputs are independent.
    pub fn sub(&self, other: &Self) -> Self {
        Self {
            center: &self.center - &other.center,
            radius: &self.radius + &other.radius,
        }
    }

    /// Exact affine image `scale * x + offset`, with radius `abs(scale) * radius`.
    pub fn affine(&self, scale: &Rational, offset: &Real) -> Self {
        let magnitude = if scale.is_negative() {
            -scale
        } else {
            scale.clone()
        };
        Self {
            center: Real::from(scale.clone()) * &self.center + offset,
            radius: magnitude * &self.radius,
        }
    }

    /// Rational enclosure from Hyperreal's certified center bounds, expanded by
    /// the exact radius. Precision is center-grid spacing `2^precision`, not a
    /// promise about output width or radius. The radius need not be dyadic.
    ///
    /// Evaluation retains Hyperreal's existing cancellation/precision behavior;
    /// this method has no total-work bound and adds no separate cache.
    pub fn to_rational(&self, precision: i32) -> Result<RationalInterval, IntervalError> {
        let [lower, upper] = self
            .center
            .certified_dyadic_interval(precision)
            .ok_or(IntervalError::EnclosureUnavailable)?;
        // Validate the external enclosure's order before transporting it.
        let center = RationalInterval::new(lower, upper)?;
        Ok(center.add(&RationalInterval::from_nominal_uncertainty(
            &Rational::zero(),
            &self.radius,
        )?))
    }

    /// Outward dyadic enclosure at grid spacing `2^precision`.
    /// The budget covers the rational-to-dyadic conversion, not center evaluation
    /// or the arbitrary-precision rational radius expansion.
    pub fn to_dyadic(
        &self,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<DyadicInterval, IntervalError> {
        budget.reserve(i64::from(precision).unsigned_abs() + 1, 1, false)?;
        self.to_rational(precision)?.to_dyadic(precision, budget)
    }

    /// Classifies exact point membership using a certified bound on
    /// `point - center`. Overlap alone returns `Unresolved`; it cannot certify
    /// either membership or equality. An exact rational displacement resolves
    /// either side of a closed boundary even when it is off the dyadic grid.
    /// Unavailable evaluation is a typed error.
    pub fn classify_point(
        &self,
        point: &Real,
        precision: i32,
    ) -> Result<BallRelation, IntervalError> {
        let mut translated = point - &self.center;
        // Rational translation does not carry the point's abort flag, so
        // reattach it before refining the displacement.
        if let Some(signal) = point.abort_signal()
            && translated.exact_rational_ref().is_none()
        {
            translated.abort(signal.clone());
        }
        let displacement = Self::point(translated);
        let difference = displacement.to_rational(precision)?;
        let allowed = RationalInterval::from_nominal_uncertainty(&Rational::zero(), &self.radius)?;
        Ok(if difference.subset_of(&allowed) {
            BallRelation::Contained
        } else if difference.disjoint(&allowed) {
            BallRelation::Excluded
        } else if let Some(exact) = displacement.center().exact_rational_ref() {
            if allowed.contains_point(exact) {
                BallRelation::Contained
            } else {
                BallRelation::Excluded
            }
        } else {
            BallRelation::Unresolved
        })
    }
}
