//! Exact inverse-square metaballs and a continuous, sign-equivalent level field.

use core::cmp::Ordering;
use hyperlattice::{Aabb, Point3, Real};
use hyperreal::Problem;

use crate::evaluation::decide;
use crate::policy::{CONSTRUCTION_PREDICATE_POLICY, compare_reals_for_construction};
use crate::{SdfEvaluationError, SdfMetricStatus};

/// An inverse-square source with influence `radius² / |point - center|²`.
#[derive(Clone, Debug, PartialEq)]
pub struct Metaball {
    pub center: Point3,
    pub radius: Real,
}

impl Metaball {
    /// Retain authored parameters; evaluation validates the strictly positive radius.
    pub const fn new(center: Point3, radius: Real) -> Self {
        Self { center, radius }
    }

    /// Evaluate the exact inverse-square influence without a distance cutoff.
    ///
    /// The center is a pole and returns a scalar `DivideByZero` error. Use
    /// [`MetaballField::evaluate_point`] for a continuous level field at centers.
    pub fn influence(&self, point: &Point3) -> Result<Real, SdfEvaluationError> {
        require_positive(&self.radius, "metaball radius")?;
        let distance = self.distance_squared(point);
        match distance_sign(&distance)? {
            Ordering::Equal => Err(scalar_error("metaball influence", Problem::DivideByZero)),
            _ => self.influence_at_distance(distance),
        }
    }

    fn distance_squared(&self, point: &Point3) -> Real {
        point.to_vector().squared_distance(&self.center.to_vector())
    }

    fn influence_at_distance(&self, distance: Real) -> Result<Real, SdfEvaluationError> {
        ((&self.radius * &self.radius) / distance)
            .map_err(|source| scalar_error("metaball influence", source))
    }
}

/// A validated, borrowed inverse-square field at a positive iso level.
///
/// With `I = sum(radius² / distance²)`, evaluation returns `(iso - I)/(iso + I)`.
/// This is negative inside, zero exactly at `I = iso`, and positive outside.
/// It lies in `[-1, 1]` and extends continuously to `-1` at every center, including
/// coincident centers. It is a sign-equivalent field, not a distance or a mesh
/// fidelity certificate. Sampling it is an explicit choice of interpolation
/// field; it does not reproduce interpolation of singular `I` values.
///
/// Validation and scalar construction are strict. Borrowing prevents parameter
/// mutation while the validated field is in use. Work is linear in the source
/// count per sample, but scalar expression size, work and memory are not bounded.
#[derive(Debug)]
pub struct MetaballField<'a> {
    balls: &'a [Metaball],
    iso_value: Real,
}

impl<'a> MetaballField<'a> {
    /// Validate every radius and the iso level before any center/empty shortcut.
    pub fn new(balls: &'a [Metaball], iso_value: Real) -> Result<Self, SdfEvaluationError> {
        require_positive(&iso_value, "metaball iso value")?;
        for ball in balls {
            require_positive(&ball.radius, "metaball radius")?;
        }
        Ok(Self { balls, iso_value })
    }

    /// Retained sources whose parameters were validated at construction.
    pub const fn balls(&self) -> &'a [Metaball] {
        self.balls
    }

    /// Positive iso value retained by this field.
    pub const fn iso_value(&self) -> &Real {
        &self.iso_value
    }

    /// The normalization preserves the source's level-set sign, not metric distance.
    pub const fn metric_status(&self) -> SdfMetricStatus {
        SdfMetricStatus::SignEquivalent
    }

    /// Evaluate the continuous normalized level field exactly, including at poles.
    pub fn evaluate_point(&self, point: &Point3) -> Result<Real, SdfEvaluationError> {
        let mut influence = Real::zero();
        for ball in self.balls {
            let distance = ball.distance_squared(point);
            if distance_sign(&distance)? == Ordering::Equal {
                // Every radius and iso value was validated before this shortcut.
                // I tends to +infinity, so (iso-I)/(iso+I) tends to -1.
                return Ok(-Real::one());
            }
            influence += ball.influence_at_distance(distance)?;
        }
        ((&self.iso_value - &influence) / (&self.iso_value + &influence))
            .map_err(|source| scalar_error("normalized metaball field", source))
    }

    /// Conservative bounds with a strictly exterior boundary, or `None` for no sources.
    ///
    /// Each source gets radius `101/100 * r * sqrt(n/iso) + padding`.
    /// Beyond this box every contribution is less than `iso/n`, so their sum
    /// is less than `iso`. The exact 101/100 sampling margin provides exterior
    /// samples; it is not a tolerance used in a predicate or coordinate merge.
    /// Padding must be nonnegative, including for an empty field.
    pub fn sampling_bounds(&self, padding: &Real) -> Result<Option<Aabb>, SdfEvaluationError> {
        if decide(
            compare_reals_for_construction(padding, &Real::zero()),
            "metaball padding",
        )? == Ordering::Less
        {
            return Err(SdfEvaluationError::InvalidDomain {
                operation: "metaball padding",
            });
        }
        let Some((first, rest)) = self.balls.split_first() else {
            return Ok(None);
        };
        let count = u64::try_from(self.balls.len())
            .map_err(|_| scalar_error("metaball source count", Problem::Exhausted))?;
        let scale = (Real::from(count) / &self.iso_value)
            .map_err(|source| scalar_error("metaball bounds", source))?
            .sqrt()
            .map_err(|source| scalar_error("metaball bounds", source))?;
        let scale = ((scale * Real::from(101)) / Real::from(100))
            .map_err(|source| scalar_error("metaball bounds", source))?;
        let bounds_for = |ball: &Metaball| {
            let extent = &ball.radius * &scale + padding;
            Aabb::new(
                Point3::new(
                    &ball.center.x - &extent,
                    &ball.center.y - &extent,
                    &ball.center.z - &extent,
                ),
                Point3::new(
                    &ball.center.x + &extent,
                    &ball.center.y + &extent,
                    &ball.center.z + &extent,
                ),
            )
        };
        let mut bounds = bounds_for(first);
        for ball in rest {
            let next = bounds_for(ball);
            for (current, other) in [
                (&mut bounds.mins.x, next.mins.x),
                (&mut bounds.mins.y, next.mins.y),
                (&mut bounds.mins.z, next.mins.z),
            ] {
                *current = decide(
                    hyperlimit::real_min(current, &other, CONSTRUCTION_PREDICATE_POLICY),
                    "metaball minimum bound",
                )?
                .clone();
            }
            for (current, other) in [
                (&mut bounds.maxs.x, next.maxs.x),
                (&mut bounds.maxs.y, next.maxs.y),
                (&mut bounds.maxs.z, next.maxs.z),
            ] {
                *current = decide(
                    hyperlimit::real_max(current, &other, CONSTRUCTION_PREDICATE_POLICY),
                    "metaball maximum bound",
                )?
                .clone();
            }
        }
        Ok(Some(bounds))
    }
}

fn require_positive(value: &Real, operation: &'static str) -> Result<(), SdfEvaluationError> {
    if decide(
        compare_reals_for_construction(value, &Real::zero()),
        operation,
    )? == Ordering::Greater
    {
        Ok(())
    } else {
        Err(SdfEvaluationError::InvalidDomain { operation })
    }
}

fn distance_sign(distance: &Real) -> Result<Ordering, SdfEvaluationError> {
    let sign = decide(
        compare_reals_for_construction(distance, &Real::zero()),
        "metaball squared distance",
    )?;
    if sign == Ordering::Less {
        Err(SdfEvaluationError::InvalidDomain {
            operation: "metaball squared distance",
        })
    } else {
        Ok(sign)
    }
}

const fn scalar_error(operation: &'static str, source: Problem) -> SdfEvaluationError {
    SdfEvaluationError::Scalar { operation, source }
}
