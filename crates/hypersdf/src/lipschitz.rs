//! Local Lipschitz-bound reports for retained SDF expressions.
//!
//! These bounds are conservative exact `Real` values over a closed AABB query.
//! They are scheduling and solver facts, not topology decisions. The formulas
//! follow classical Lipschitz arithmetic rules over interval enclosures: sums
//! add bounds, products use `|f| L_g + |g| L_f`, and min/max CSG composition
//! takes the larger active-family bound. Unsupported expressions return
//! explicit unknown evidence, preserving Yap's exact-geometric-computation
//! separation between certified facts and missing optimizations.

use core::cmp::Ordering;

use hyperlimit::{Certainty, Escalation, Point3, PredicateOutcome, RefinementNeed, compare_reals};
use hyperreal::Real;

use crate::expr::SdfExpr;
use crate::interval::{SdfInterval, interval_expr_cell};
use crate::primitive::{SdfPrimitive, radius_squared_domain, squared_distance3};
use crate::status::{SdfEvidenceStatus, SdfFreshness, SdfLipschitzStatus};

/// Conservative local Lipschitz-bound report over a closed AABB/cell.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfLipschitzReport {
    /// Minimum corner of the closed query cell.
    pub min: Point3,
    /// Maximum corner of the closed query cell.
    pub max: Point3,
    /// Certified conservative Lipschitz bound when available.
    pub bound: Option<Real>,
    /// Structural availability class for Lipschitz evidence.
    pub lipschitz_status: SdfLipschitzStatus,
    /// Exact/certified evidence status for the bound.
    pub evidence: SdfEvidenceStatus,
    /// Prepared-source freshness.
    pub freshness: SdfFreshness,
}

impl SdfLipschitzReport {
    /// Returns whether a certified bound is present.
    pub const fn is_certified(&self) -> bool {
        self.bound.is_some() && self.evidence.is_certified()
    }

    /// Validate that bound presence agrees with evidence status.
    pub const fn is_self_consistent(&self) -> bool {
        match self.evidence {
            SdfEvidenceStatus::Certified { .. } => self.bound.is_some(),
            SdfEvidenceStatus::Unknown { .. } => self.bound.is_none(),
        }
    }
}

pub(crate) fn lipschitz_expr_cell(
    expr: &SdfExpr,
    min: &Point3,
    max: &Point3,
) -> PredicateOutcome<Real> {
    match expr {
        SdfExpr::Constant(_) => decided(Real::zero()),
        SdfExpr::Coordinate(_) => decided(Real::one()),
        SdfExpr::Linear { coefficients, .. } => vector_norm(&[
            coefficients.0[0].clone(),
            coefficients.0[1].clone(),
            coefficients.0[2].clone(),
        ]),
        SdfExpr::Primitive(primitive) => lipschitz_primitive_cell(primitive, min, max),
        SdfExpr::Union(left, right) | SdfExpr::Intersection(left, right) => combine_lipschitz_pair(
            lipschitz_expr_cell(left, min, max),
            lipschitz_expr_cell(right, min, max),
            max_real,
        ),
        SdfExpr::Add(left, right) | SdfExpr::Sub(left, right) => combine_lipschitz_pair(
            lipschitz_expr_cell(left, min, max),
            lipschitz_expr_cell(right, min, max),
            |a, b| Some(&a + &b),
        ),
        SdfExpr::Mul(left, right) => lipschitz_mul_cell(left, right, min, max),
        SdfExpr::Abs(inner) | SdfExpr::Complement(inner) | SdfExpr::Offset { child: inner, .. } => {
            lipschitz_expr_cell(inner, min, max)
        }
        SdfExpr::Sqrt(inner) => lipschitz_sqrt_cell(inner, min, max),
        SdfExpr::Sin(inner) | SdfExpr::Cos(inner) => lipschitz_expr_cell(inner, min, max),
        SdfExpr::Tan(_) => unsupported(),
        SdfExpr::Transform { child, transform } => {
            let Some((child_min, child_max)) = transform.inverse_aabb(min, max) else {
                return unsupported();
            };
            lipschitz_expr_cell(child, &child_min, &child_max)
        }
    }
}

fn lipschitz_primitive_cell(
    primitive: &SdfPrimitive,
    min: &Point3,
    max: &Point3,
) -> PredicateOutcome<Real> {
    match primitive {
        SdfPrimitive::Plane { plane } => vector_norm(&[
            plane.normal.x.clone(),
            plane.normal.y.clone(),
            plane.normal.z.clone(),
        ]),
        SdfPrimitive::Sphere {
            center,
            radius_squared,
        } => match radius_squared_domain(radius_squared) {
            PredicateOutcome::Decided { value: true, .. } => {
                let farthest = match farthest_squared_distance(center, min, max) {
                    Some(value) => value,
                    None => return unsupported(),
                };
                match farthest.sqrt() {
                    Ok(distance) => decided(Real::from(2_i32) * &distance),
                    Err(_) => unsupported(),
                }
            }
            PredicateOutcome::Decided { .. } => unsupported(),
            PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
        },
        SdfPrimitive::Aabb { .. } => decided(Real::one()),
        SdfPrimitive::RoundedAabb { .. } => unsupported(),
        SdfPrimitive::Slab { plane, .. } => vector_norm(&[
            plane.normal.x.clone(),
            plane.normal.y.clone(),
            plane.normal.z.clone(),
        ]),
        SdfPrimitive::Cylinder { .. }
        | SdfPrimitive::Capsule { .. }
        | SdfPrimitive::Torus { .. } => unsupported(),
    }
}

fn lipschitz_mul_cell(
    left: &SdfExpr,
    right: &SdfExpr,
    min: &Point3,
    max: &Point3,
) -> PredicateOutcome<Real> {
    let left_l = match lipschitz_expr_cell(left, min, max) {
        PredicateOutcome::Decided { value, .. } => value,
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };
    let right_l = match lipschitz_expr_cell(right, min, max) {
        PredicateOutcome::Decided { value, .. } => value,
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };
    let left_interval = match interval_expr_cell(left, min, max) {
        PredicateOutcome::Decided { value, .. } => value,
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };
    let right_interval = match interval_expr_cell(right, min, max) {
        PredicateOutcome::Decided { value, .. } => value,
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };
    let Some(left_abs) = interval_abs_upper(&left_interval) else {
        return unsupported();
    };
    let Some(right_abs) = interval_abs_upper(&right_interval) else {
        return unsupported();
    };
    decided(&(&left_abs * &right_l) + &(&right_abs * &left_l))
}

fn lipschitz_sqrt_cell(inner: &SdfExpr, min: &Point3, max: &Point3) -> PredicateOutcome<Real> {
    let child_l = match lipschitz_expr_cell(inner, min, max) {
        PredicateOutcome::Decided { value, .. } => value,
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };
    let interval = match interval_expr_cell(inner, min, max) {
        PredicateOutcome::Decided { value, .. } => value,
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };
    match compare_reals(&interval.lower, &Real::zero()) {
        PredicateOutcome::Decided {
            value: Ordering::Greater,
            ..
        } => {
            let Ok(root) = interval.lower.sqrt() else {
                return unsupported();
            };
            let denominator = Real::from(2_i32) * &root;
            match (&child_l / &denominator).ok() {
                Some(bound) => decided(bound),
                None => unsupported(),
            }
        }
        PredicateOutcome::Decided { .. } => unsupported(),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn vector_norm(components: &[Real; 3]) -> PredicateOutcome<Real> {
    let x2 = &components[0] * &components[0];
    let y2 = &components[1] * &components[1];
    let z2 = &components[2] * &components[2];
    match (&(&x2 + &y2) + &z2).sqrt() {
        Ok(value) => decided(value),
        Err(_) => unsupported(),
    }
}

fn interval_abs_upper(interval: &SdfInterval) -> Option<Real> {
    let neg_lower = -&interval.lower;
    max_real(neg_lower, interval.upper.clone())
}

fn farthest_squared_distance(center: &Point3, min: &Point3, max: &Point3) -> Option<Real> {
    let corners = corners(min, max);
    let mut farthest = squared_distance3(center, &corners[0]);
    for corner in &corners[1..] {
        farthest = max_real(farthest, squared_distance3(center, corner))?;
    }
    Some(farthest)
}

fn corners(min: &Point3, max: &Point3) -> [Point3; 8] {
    [
        Point3::new(min.x.clone(), min.y.clone(), min.z.clone()),
        Point3::new(max.x.clone(), min.y.clone(), min.z.clone()),
        Point3::new(min.x.clone(), max.y.clone(), min.z.clone()),
        Point3::new(max.x.clone(), max.y.clone(), min.z.clone()),
        Point3::new(min.x.clone(), min.y.clone(), max.z.clone()),
        Point3::new(max.x.clone(), min.y.clone(), max.z.clone()),
        Point3::new(min.x.clone(), max.y.clone(), max.z.clone()),
        Point3::new(max.x.clone(), max.y.clone(), max.z.clone()),
    ]
}

fn combine_lipschitz_pair<F>(
    left: PredicateOutcome<Real>,
    right: PredicateOutcome<Real>,
    combine: F,
) -> PredicateOutcome<Real>
where
    F: FnOnce(Real, Real) -> Option<Real>,
{
    match (left, right) {
        (
            PredicateOutcome::Decided { value: a, .. },
            PredicateOutcome::Decided { value: b, .. },
        ) => match combine(a, b) {
            Some(value) => decided(value),
            None => unsupported(),
        },
        (PredicateOutcome::Unknown { needed, stage }, _)
        | (_, PredicateOutcome::Unknown { needed, stage }) => {
            PredicateOutcome::unknown(needed, stage)
        }
    }
}

fn max_real(left: Real, right: Real) -> Option<Real> {
    match compare_reals(&left, &right) {
        PredicateOutcome::Decided {
            value: Ordering::Less,
            ..
        } => Some(right),
        PredicateOutcome::Decided { .. } => Some(left),
        PredicateOutcome::Unknown { .. } => None,
    }
}

fn decided(value: Real) -> PredicateOutcome<Real> {
    PredicateOutcome::decided(value, Certainty::Exact, Escalation::Exact)
}

fn unsupported() -> PredicateOutcome<Real> {
    PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided)
}
