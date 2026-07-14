//! Exact contact materials and first contact-query reports.
//!
//! Runtime engines usually build contact manifolds with margins, tolerances,
//! and iterative solvers. This module keeps the first contact boundary exact:
//! material coefficients are validated as exact values and AABB/AABB contact
//! state is classified as separated, touching, or intersecting without an
//! epsilon. Approximate manifold proposals may be useful, but accepted
//! topological contact decisions need exact or certified replay.
//!
//! The report surface is intentionally pre-solver. Complementarity and impulse
//! policies are named in the integration module, while this module only
//! provides the exact geometric and material facts those solvers consume.

use hyperlattice::Vector3;
use hyperlimit::{Aabb3Intersection, PredicateOutcome};
use hyperreal::{Real, RealSign};

use crate::{AxisAlignedBox3, PhysicsError, PhysicsResult};

/// Exact contact material coefficients.
#[derive(Clone, Debug, PartialEq)]
pub struct ContactMaterial {
    /// Provenance label or material handle.
    pub source: String,
    /// Coulomb friction coefficient.
    pub friction: Real,
    /// Normal restitution coefficient in `[0, 1]`.
    pub restitution: Real,
}

/// Contact classification for two exact shapes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContactClassification {
    /// Certified strict separation.
    Separated,
    /// Certified boundary contact without volume overlap.
    Touching,
    /// Certified overlap/intersection.
    Intersecting,
    /// Query could not certify a decision.
    Unknown,
}

/// Exact AABB/AABB contact report.
#[derive(Clone, Debug, PartialEq)]
pub struct AabbContactReport3 {
    /// Contact classification.
    pub classification: ContactClassification,
    /// Per-axis overlap/penetration interval lengths.
    pub overlaps: [Real; 3],
    /// Axis with minimum positive overlap when intersecting.
    pub minimum_overlap_axis: Option<usize>,
}

impl ContactMaterial {
    /// Creates a contact material after validating coefficient domains.
    pub fn new(
        source: impl Into<String>,
        friction: Real,
        restitution: Real,
    ) -> PhysicsResult<Self> {
        require_nonnegative(&friction, PhysicsError::NegativeFrictionCoefficient)?;
        require_fraction(&restitution)?;
        Ok(Self {
            source: source.into(),
            friction,
            restitution,
        })
    }
}

impl AabbContactReport3 {
    /// Classifies exact AABB/AABB contact without tolerance inflation.
    pub fn classify(left: &AxisAlignedBox3, right: &AxisAlignedBox3) -> PhysicsResult<Self> {
        let mut overlaps = [Real::zero(), Real::zero(), Real::zero()];
        let relation = decide(hyperlimit::classify_aabb3_intersection(
            &point3_from_vector(&left.min),
            &point3_from_vector(&left.max),
            &point3_from_vector(&right.min),
            &point3_from_vector(&right.max),
        ))?;

        if relation == Aabb3Intersection::Disjoint {
            return Ok(Self {
                classification: ContactClassification::Separated,
                overlaps,
                minimum_overlap_axis: None,
            });
        }
        for (axis, overlap) in overlaps.iter_mut().enumerate() {
            let left_before_right = left.max[axis].clone() - right.min[axis].clone();
            let right_before_left = right.max[axis].clone() - left.min[axis].clone();
            *overlap = min_real(left_before_right, right_before_left)?;
        }

        if relation == Aabb3Intersection::Touching {
            Ok(Self {
                classification: ContactClassification::Touching,
                overlaps,
                minimum_overlap_axis: None,
            })
        } else {
            Ok(Self {
                minimum_overlap_axis: minimum_overlap_axis(&overlaps)?,
                classification: ContactClassification::Intersecting,
                overlaps,
            })
        }
    }
}

fn minimum_overlap_axis(overlaps: &[Real; 3]) -> PhysicsResult<Option<usize>> {
    let mut best = 0_usize;
    for axis in 1..3 {
        if less(&overlaps[axis], &overlaps[best])? {
            best = axis;
        }
    }
    Ok(Some(best))
}

fn min_real(left: Real, right: Real) -> PhysicsResult<Real> {
    if less(&left, &right)? {
        Ok(left)
    } else {
        Ok(right)
    }
}

fn require_nonnegative(value: &Real, error: PhysicsError) -> PhysicsResult<()> {
    match sign(value)? {
        RealSign::Positive | RealSign::Zero => Ok(()),
        RealSign::Negative => Err(error),
    }
}

fn require_fraction(value: &Real) -> PhysicsResult<()> {
    let lower = sign(value)?;
    let upper = sign(&(Real::one() - value.clone()))?;
    match (lower, upper) {
        (RealSign::Positive | RealSign::Zero, RealSign::Positive | RealSign::Zero) => Ok(()),
        _ => Err(PhysicsError::InvalidRestitutionCoefficient),
    }
}

fn less(left: &Real, right: &Real) -> PhysicsResult<bool> {
    Ok(sign(&(left.clone() - right.clone()))? == RealSign::Negative)
}

fn sign(value: &Real) -> PhysicsResult<RealSign> {
    match hyperlimit::compare_reals(value, &Real::zero()).value() {
        Some(core::cmp::Ordering::Less) => Ok(RealSign::Negative),
        Some(core::cmp::Ordering::Equal) => Ok(RealSign::Zero),
        Some(core::cmp::Ordering::Greater) => Ok(RealSign::Positive),
        None => Err(PhysicsError::UnknownShapeQuery),
    }
}

fn decide<T>(outcome: PredicateOutcome<T>) -> PhysicsResult<T> {
    outcome.value().ok_or(PhysicsError::UnknownShapeQuery)
}

fn point3_from_vector(vector: &Vector3) -> hyperlimit::Point3 {
    hyperlimit::Point3::new(vector[0].clone(), vector[1].clone(), vector[2].clone())
}
