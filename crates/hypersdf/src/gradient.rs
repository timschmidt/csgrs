//! Exact gradient reports for retained SDF expressions.
//!
//! Gradients are advisory differential facts, not topology evidence. This
//! module only returns a vector when the retained expression has a certified
//! symbolic route at the query point. Branching constructs such as CSG min/max
//! are rejected at ties, and piecewise primitives are rejected at unresolved
//! branches. This follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): differential data may guide
//! adapters and solvers, but topology still replays through predicates.

use core::cmp::Ordering;

use hyperlattice::Vector3;
use hyperlimit::{Escalation, Point3, PredicateOutcome, RefinementNeed, compare_reals};
use hyperreal::Real;

use crate::expr::{SdfCoordinate, SdfExpr};
use crate::primitive::SdfPrimitive;
use crate::sampling::scalar_expr_point;
use crate::status::{SdfEvidenceStatus, SdfFreshness, SdfGradientStatus, SdfNormalStatus};

/// Point-gradient report for a retained expression.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfGradientReport {
    /// Query point.
    pub point: Point3,
    /// Exact symbolic gradient when certified at the query point.
    pub gradient: Option<Vector3>,
    /// Structural availability and provenance for the gradient.
    pub gradient_status: SdfGradientStatus,
    /// Exact/certified evidence status.
    pub evidence: SdfEvidenceStatus,
    /// Prepared-source freshness.
    pub freshness: SdfFreshness,
}

impl SdfGradientReport {
    /// Returns whether a certified exact gradient vector is present.
    pub const fn is_certified(&self) -> bool {
        self.gradient.is_some() && self.evidence.is_certified()
    }

    /// Validate that gradient presence agrees with evidence status.
    pub const fn is_self_consistent(&self) -> bool {
        match self.evidence {
            SdfEvidenceStatus::Certified { .. } => self.gradient.is_some(),
            SdfEvidenceStatus::Unknown { .. } => self.gradient.is_none(),
        }
    }
}

/// Point-normal report derived from a certified exact gradient.
///
/// The normal is intentionally unnormalized. Normalizing would require a
/// square root and possible division by an undecidable zero; for topology and
/// solver replay, the exact nonzero direction is the useful certificate. This
/// keeps the differential package aligned with Yap's exact-geometric-
/// computation model: a missing or zero normal is an explicit report state,
/// not a tolerance fallback.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfNormalReport {
    /// Query point.
    pub point: Point3,
    /// Exact unnormalized normal direction when certified nonzero.
    pub normal: Option<Vector3>,
    /// Validity of the reported normal direction.
    pub normal_status: SdfNormalStatus,
    /// Gradient availability and provenance for the source expression.
    pub gradient_status: SdfGradientStatus,
    /// Exact/certified evidence status for the normal validity decision.
    pub evidence: SdfEvidenceStatus,
    /// Prepared-source freshness.
    pub freshness: SdfFreshness,
}

impl SdfNormalReport {
    /// Returns whether a certified exact nonzero normal direction is present.
    pub const fn is_certified_direction(&self) -> bool {
        self.normal.is_some() && self.normal_status.has_direction() && self.evidence.is_certified()
    }

    /// Validate that normal presence agrees with status and evidence.
    pub const fn is_self_consistent(&self) -> bool {
        match self.normal_status {
            SdfNormalStatus::ExactDirection => {
                self.normal.is_some() && self.evidence.is_certified()
            }
            SdfNormalStatus::ZeroGradient => self.normal.is_none() && self.evidence.is_certified(),
            SdfNormalStatus::Unknown => self.normal.is_none(),
        }
    }
}

pub(crate) fn normal_from_gradient_report(report: SdfGradientReport) -> SdfNormalReport {
    let Some(gradient) = report.gradient.clone() else {
        return SdfNormalReport {
            point: report.point,
            normal: None,
            normal_status: SdfNormalStatus::Unknown,
            gradient_status: report.gradient_status,
            evidence: report.evidence,
            freshness: report.freshness,
        };
    };
    let norm_squared = vector_norm_squared(&gradient);
    match compare_reals(&norm_squared, &Real::zero()) {
        PredicateOutcome::Decided {
            value: Ordering::Greater,
            certainty,
            stage,
        } => SdfNormalReport {
            point: report.point,
            normal: Some(gradient),
            normal_status: SdfNormalStatus::ExactDirection,
            gradient_status: report.gradient_status,
            evidence: SdfEvidenceStatus::Certified { certainty, stage },
            freshness: report.freshness,
        },
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            certainty,
            stage,
        } => SdfNormalReport {
            point: report.point,
            normal: None,
            normal_status: SdfNormalStatus::ZeroGradient,
            gradient_status: report.gradient_status,
            evidence: SdfEvidenceStatus::Certified { certainty, stage },
            freshness: report.freshness,
        },
        PredicateOutcome::Decided { .. } => SdfNormalReport {
            point: report.point,
            normal: None,
            normal_status: SdfNormalStatus::Unknown,
            gradient_status: report.gradient_status,
            evidence: SdfEvidenceStatus::Unknown {
                needed: RefinementNeed::ExactArithmetic,
                stage: Escalation::Undecided,
            },
            freshness: report.freshness,
        },
        PredicateOutcome::Unknown { needed, stage } => SdfNormalReport {
            point: report.point,
            normal: None,
            normal_status: SdfNormalStatus::Unknown,
            gradient_status: report.gradient_status,
            evidence: SdfEvidenceStatus::Unknown { needed, stage },
            freshness: report.freshness,
        },
    }
}

pub(crate) fn gradient_expr_point(expr: &SdfExpr, point: &Point3) -> PredicateOutcome<Vector3> {
    match expr {
        SdfExpr::Constant(_) => decided(zero_vector()),
        SdfExpr::Coordinate(axis) => decided(axis_gradient(*axis)),
        SdfExpr::Linear { coefficients, .. } => decided(coefficients.clone()),
        SdfExpr::Primitive(primitive) => gradient_primitive_point(primitive, point),
        SdfExpr::Union(left, right) => gradient_minmax_point(left, right, point, true),
        SdfExpr::Intersection(left, right) => gradient_minmax_point(left, right, point, false),
        SdfExpr::Add(left, right) => combine_gradient_pair(
            gradient_expr_point(left, point),
            gradient_expr_point(right, point),
            add_vectors,
        ),
        SdfExpr::Sub(left, right) => combine_gradient_pair(
            gradient_expr_point(left, point),
            gradient_expr_point(right, point),
            sub_vectors,
        ),
        SdfExpr::Mul(left, right) => gradient_mul_point(left, right, point),
        SdfExpr::Abs(inner) => gradient_abs_point(inner, point),
        SdfExpr::Sqrt(_) | SdfExpr::Sin(_) | SdfExpr::Cos(_) | SdfExpr::Tan(_) => unsupported(),
        SdfExpr::Complement(inner) => map_gradient(gradient_expr_point(inner, point), neg_vector),
        SdfExpr::Offset { child, .. } => gradient_expr_point(child, point),
        SdfExpr::Transform { child, transform } => match transform {
            crate::transform::SdfTransform::Translation { .. } => {
                gradient_expr_point(child, &transform.inverse_point(point))
            }
            crate::transform::SdfTransform::Affine { .. } => unsupported(),
        },
    }
}

fn gradient_primitive_point(primitive: &SdfPrimitive, point: &Point3) -> PredicateOutcome<Vector3> {
    match primitive {
        SdfPrimitive::Plane { plane } => decided(Vector3([
            plane.normal.x.clone(),
            plane.normal.y.clone(),
            plane.normal.z.clone(),
        ])),
        SdfPrimitive::Sphere { center, .. } => {
            let two = Real::from(2_i32);
            decided(Vector3([
                &two * &(&point.x - &center.x),
                &two * &(&point.y - &center.y),
                &two * &(&point.z - &center.z),
            ]))
        }
        SdfPrimitive::Aabb { .. }
        | SdfPrimitive::RoundedAabb { .. }
        | SdfPrimitive::Cylinder { .. }
        | SdfPrimitive::Capsule { .. }
        | SdfPrimitive::Torus { .. } => unsupported(),
        SdfPrimitive::Slab { plane, .. } => {
            let value = plane_value(plane, point);
            match compare_reals(&value, &Real::zero()) {
                PredicateOutcome::Decided {
                    value: Ordering::Less,
                    ..
                } => decided(Vector3([
                    -&plane.normal.x,
                    -&plane.normal.y,
                    -&plane.normal.z,
                ])),
                PredicateOutcome::Decided {
                    value: Ordering::Greater,
                    ..
                } => decided(Vector3([
                    plane.normal.x.clone(),
                    plane.normal.y.clone(),
                    plane.normal.z.clone(),
                ])),
                PredicateOutcome::Decided { .. } => unsupported(),
                PredicateOutcome::Unknown { needed, stage } => {
                    PredicateOutcome::unknown(needed, stage)
                }
            }
        }
    }
}

fn gradient_minmax_point(
    left: &SdfExpr,
    right: &SdfExpr,
    point: &Point3,
    choose_min: bool,
) -> PredicateOutcome<Vector3> {
    let Some(left_value) = scalar_expr_point(left, point) else {
        return unsupported();
    };
    let Some(right_value) = scalar_expr_point(right, point) else {
        return unsupported();
    };
    match compare_reals(&left_value, &right_value) {
        PredicateOutcome::Decided {
            value: Ordering::Less,
            ..
        } => {
            if choose_min {
                gradient_expr_point(left, point)
            } else {
                gradient_expr_point(right, point)
            }
        }
        PredicateOutcome::Decided {
            value: Ordering::Greater,
            ..
        } => {
            if choose_min {
                gradient_expr_point(right, point)
            } else {
                gradient_expr_point(left, point)
            }
        }
        PredicateOutcome::Decided { .. } => unsupported(),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn gradient_mul_point(
    left: &SdfExpr,
    right: &SdfExpr,
    point: &Point3,
) -> PredicateOutcome<Vector3> {
    let Some(left_value) = scalar_expr_point(left, point) else {
        return unsupported();
    };
    let Some(right_value) = scalar_expr_point(right, point) else {
        return unsupported();
    };
    combine_gradient_pair(
        map_gradient(gradient_expr_point(left, point), |gradient| {
            scale_vector(gradient, &right_value)
        }),
        map_gradient(gradient_expr_point(right, point), |gradient| {
            scale_vector(gradient, &left_value)
        }),
        add_vectors,
    )
}

fn gradient_abs_point(inner: &SdfExpr, point: &Point3) -> PredicateOutcome<Vector3> {
    let Some(value) = scalar_expr_point(inner, point) else {
        return unsupported();
    };
    match compare_reals(&value, &Real::zero()) {
        PredicateOutcome::Decided {
            value: Ordering::Less,
            ..
        } => map_gradient(gradient_expr_point(inner, point), neg_vector),
        PredicateOutcome::Decided {
            value: Ordering::Greater,
            ..
        } => gradient_expr_point(inner, point),
        PredicateOutcome::Decided { .. } => unsupported(),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn combine_gradient_pair<F>(
    left: PredicateOutcome<Vector3>,
    right: PredicateOutcome<Vector3>,
    combine: F,
) -> PredicateOutcome<Vector3>
where
    F: FnOnce(Vector3, Vector3) -> Vector3,
{
    match (left, right) {
        (
            PredicateOutcome::Decided { value: a, .. },
            PredicateOutcome::Decided { value: b, .. },
        ) => decided(combine(a, b)),
        (PredicateOutcome::Unknown { needed, stage }, _)
        | (_, PredicateOutcome::Unknown { needed, stage }) => {
            PredicateOutcome::unknown(needed, stage)
        }
    }
}

fn map_gradient<F>(outcome: PredicateOutcome<Vector3>, map: F) -> PredicateOutcome<Vector3>
where
    F: FnOnce(Vector3) -> Vector3,
{
    match outcome {
        PredicateOutcome::Decided { value, .. } => decided(map(value)),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn axis_gradient(axis: SdfCoordinate) -> Vector3 {
    match axis {
        SdfCoordinate::X => Vector3([Real::one(), Real::zero(), Real::zero()]),
        SdfCoordinate::Y => Vector3([Real::zero(), Real::one(), Real::zero()]),
        SdfCoordinate::Z => Vector3([Real::zero(), Real::zero(), Real::one()]),
    }
}

fn zero_vector() -> Vector3 {
    Vector3([Real::zero(), Real::zero(), Real::zero()])
}

fn add_vectors(left: Vector3, right: Vector3) -> Vector3 {
    Vector3([
        &left.0[0] + &right.0[0],
        &left.0[1] + &right.0[1],
        &left.0[2] + &right.0[2],
    ])
}

fn sub_vectors(left: Vector3, right: Vector3) -> Vector3 {
    Vector3([
        &left.0[0] - &right.0[0],
        &left.0[1] - &right.0[1],
        &left.0[2] - &right.0[2],
    ])
}

fn neg_vector(vector: Vector3) -> Vector3 {
    Vector3([-&vector.0[0], -&vector.0[1], -&vector.0[2]])
}

fn scale_vector(vector: Vector3, scale: &Real) -> Vector3 {
    Vector3([
        &vector.0[0] * scale,
        &vector.0[1] * scale,
        &vector.0[2] * scale,
    ])
}

fn vector_norm_squared(vector: &Vector3) -> Real {
    let x2 = &vector.0[0] * &vector.0[0];
    let y2 = &vector.0[1] * &vector.0[1];
    let z2 = &vector.0[2] * &vector.0[2];
    &(&x2 + &y2) + &z2
}

fn plane_value(plane: &hyperlimit::Plane3, point: &Point3) -> Real {
    let nx = &plane.normal.x * &point.x;
    let ny = &plane.normal.y * &point.y;
    let nz = &plane.normal.z * &point.z;
    &(&(&nx + &ny) + &nz) + &plane.offset
}

fn decided(value: Vector3) -> PredicateOutcome<Vector3> {
    PredicateOutcome::decided(value, hyperlimit::Certainty::Exact, Escalation::Exact)
}

fn unsupported() -> PredicateOutcome<Vector3> {
    PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided)
}
