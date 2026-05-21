//! Sketch entity degeneracy preflight certificates.
//!
//! Degeneracy explanations belong to retained sketch objects, not only to the
//! residual rows they later lower into. This module checks simple entity-level
//! degeneracies with exact arithmetic and returns report rows instead of
//! applying epsilon repairs. That follows Yap's Exact Geometric Computation
//! boundary: topology/validity decisions consume certified facts or remain
//! explicit uncertainty. See Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997).

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals_with_policy};
use hyperreal::Real;

use crate::sketch::{
    SketchArcOfCircle2, SketchCircle2, SketchDistance, SketchEntityHandle, SketchEntityKind,
    SketchLineSegment2, SketchNormal2, SketchNormal3, SketchParameterHandle, SketchSolveProblem,
    SketchWorkplane,
};

/// Entity-level degeneracy family checked during sketch preflight.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SketchDegeneracyKind {
    /// A 2D line segment has equal start and end coordinates.
    ZeroLengthLineSegment2,
    /// A circle's retained distance radius is zero.
    ZeroRadiusCircle2,
    /// A circular arc has equal start and end coordinates.
    FullCircleArc2,
    /// A 2D normal does not have squared length exactly one.
    NonunitNormal2,
    /// A 3D normal/quaternion does not have squared length exactly one.
    NonunitNormal3,
    /// A workplane references a missing or nonunit normal.
    SingularWorkplane,
}

/// Proof status for one sketch degeneracy check.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SketchDegeneracyStatus {
    /// The checked entity was proved nondegenerate for this family.
    CertifiedNondegenerate,
    /// The checked entity was proved degenerate for this family.
    CertifiedDegenerate,
    /// Exact comparison was not decided under the supplied policy.
    Unknown,
    /// A referenced entity handle was missing.
    MissingEntity {
        /// Missing entity handle.
        handle: SketchEntityHandle,
    },
    /// A referenced parameter handle was missing.
    MissingParameter {
        /// Missing parameter handle.
        handle: SketchParameterHandle,
    },
    /// A referenced entity had the wrong retained family.
    WrongEntityKind {
        /// Rejected entity handle.
        handle: SketchEntityHandle,
        /// Expected retained family.
        expected: &'static str,
    },
}

impl SketchDegeneracyStatus {
    /// Returns whether this status proves a degeneracy.
    pub const fn is_certified_degenerate(&self) -> bool {
        matches!(self, Self::CertifiedDegenerate)
    }
}

/// One entity-level degeneracy proof row.
#[derive(Clone, Debug, PartialEq)]
pub struct SketchDegeneracyCheck {
    /// Source entity being checked.
    pub entity: SketchEntityHandle,
    /// Source entity name.
    pub entity_name: String,
    /// Degeneracy family.
    pub kind: SketchDegeneracyKind,
    /// Related entity handles used by the check.
    pub related_entities: Vec<SketchEntityHandle>,
    /// Exact witness value when the check reduced to a scalar expression.
    pub witness: Option<Real>,
    /// Proof status.
    pub status: SketchDegeneracyStatus,
}

/// Report for retained sketch entity degeneracy preflight.
#[derive(Clone, Debug, PartialEq)]
pub struct SketchDegeneracyReport {
    /// Per-entity degeneracy checks.
    pub checks: Vec<SketchDegeneracyCheck>,
    /// Number of retained entities examined.
    pub entity_count: usize,
    /// Number of checks proved nondegenerate.
    pub certified_nondegenerate_checks: usize,
    /// Number of checks proved degenerate.
    pub certified_degenerate_checks: usize,
    /// Number of checks left unresolved.
    pub unknown_checks: usize,
    /// Number of checks blocked by stale or wrong retained references.
    pub invalid_reference_checks: usize,
}

impl SketchDegeneracyReport {
    /// Returns true when at least one entity has a certified degeneracy.
    pub fn has_certified_degeneracy(&self) -> bool {
        self.certified_degenerate_checks > 0
    }
}

/// Certify simple retained sketch entity degeneracies with the default
/// predicate policy.
pub fn preflight_sketch_degeneracies(sketch: &SketchSolveProblem) -> SketchDegeneracyReport {
    preflight_sketch_degeneracies_with_policy(sketch, PredicatePolicy::default())
}

/// Certify simple retained sketch entity degeneracies with an explicit
/// predicate policy.
pub fn preflight_sketch_degeneracies_with_policy(
    sketch: &SketchSolveProblem,
    policy: PredicatePolicy,
) -> SketchDegeneracyReport {
    let mut checks = Vec::new();
    for entity in sketch.entities() {
        match &entity.kind {
            SketchEntityKind::LineSegment2(line) => checks.push(check_line_segment2(
                sketch,
                entity.handle,
                entity.name.clone(),
                line,
                policy,
            )),
            SketchEntityKind::Circle2(circle) => checks.push(check_circle2(
                sketch,
                entity.handle,
                entity.name.clone(),
                circle,
                policy,
            )),
            SketchEntityKind::ArcOfCircle2(arc) => checks.push(check_arc2(
                sketch,
                entity.handle,
                entity.name.clone(),
                arc,
                policy,
            )),
            SketchEntityKind::Normal2D(normal) => checks.push(check_normal2(
                sketch,
                entity.handle,
                entity.name.clone(),
                normal,
                policy,
            )),
            SketchEntityKind::Normal3D(normal) => checks.push(check_normal3(
                sketch,
                entity.handle,
                entity.name.clone(),
                normal,
                policy,
            )),
            SketchEntityKind::Workplane(workplane) => checks.push(check_workplane(
                sketch,
                entity.handle,
                entity.name.clone(),
                workplane,
                policy,
            )),
            SketchEntityKind::Point2D(_)
            | SketchEntityKind::Point3D(_)
            | SketchEntityKind::Distance(_)
            | SketchEntityKind::Cubic2(_) => {}
        }
    }

    let certified_nondegenerate_checks = checks
        .iter()
        .filter(|check| check.status == SketchDegeneracyStatus::CertifiedNondegenerate)
        .count();
    let certified_degenerate_checks = checks
        .iter()
        .filter(|check| check.status.is_certified_degenerate())
        .count();
    let unknown_checks = checks
        .iter()
        .filter(|check| check.status == SketchDegeneracyStatus::Unknown)
        .count();
    let invalid_reference_checks = checks
        .iter()
        .filter(|check| {
            matches!(
                check.status,
                SketchDegeneracyStatus::MissingEntity { .. }
                    | SketchDegeneracyStatus::MissingParameter { .. }
                    | SketchDegeneracyStatus::WrongEntityKind { .. }
            )
        })
        .count();

    SketchDegeneracyReport {
        checks,
        entity_count: sketch.entities().len(),
        certified_nondegenerate_checks,
        certified_degenerate_checks,
        unknown_checks,
        invalid_reference_checks,
    }
}

fn check_line_segment2(
    sketch: &SketchSolveProblem,
    entity: SketchEntityHandle,
    entity_name: String,
    line: &SketchLineSegment2,
    policy: PredicatePolicy,
) -> SketchDegeneracyCheck {
    let (witness, status) = match (
        point2_coordinates(sketch, line.start),
        point2_coordinates(sketch, line.end),
    ) {
        (Ok(start), Ok(end)) => {
            let squared = squared_distance2(&start, &end);
            (
                Some(squared.clone()),
                classify_zero_is_degenerate(&squared, policy),
            )
        }
        (Err(status), _) | (_, Err(status)) => (None, status),
    };
    SketchDegeneracyCheck {
        entity,
        entity_name,
        kind: SketchDegeneracyKind::ZeroLengthLineSegment2,
        related_entities: vec![line.start, line.end],
        witness,
        status,
    }
}

fn check_circle2(
    sketch: &SketchSolveProblem,
    entity: SketchEntityHandle,
    entity_name: String,
    circle: &SketchCircle2,
    policy: PredicatePolicy,
) -> SketchDegeneracyCheck {
    let (witness, status) = match distance_value(sketch, circle.radius) {
        Ok(radius) => {
            let squared = radius.clone() * radius;
            (
                Some(squared.clone()),
                classify_zero_is_degenerate(&squared, policy),
            )
        }
        Err(status) => (None, status),
    };
    SketchDegeneracyCheck {
        entity,
        entity_name,
        kind: SketchDegeneracyKind::ZeroRadiusCircle2,
        related_entities: vec![circle.center, circle.radius],
        witness,
        status,
    }
}

fn check_arc2(
    sketch: &SketchSolveProblem,
    entity: SketchEntityHandle,
    entity_name: String,
    arc: &SketchArcOfCircle2,
    policy: PredicatePolicy,
) -> SketchDegeneracyCheck {
    let (witness, status) = match (
        point2_coordinates(sketch, arc.start),
        point2_coordinates(sketch, arc.end),
    ) {
        (Ok(start), Ok(end)) => {
            let squared = squared_distance2(&start, &end);
            (
                Some(squared.clone()),
                classify_zero_is_degenerate(&squared, policy),
            )
        }
        (Err(status), _) | (_, Err(status)) => (None, status),
    };
    SketchDegeneracyCheck {
        entity,
        entity_name,
        kind: SketchDegeneracyKind::FullCircleArc2,
        related_entities: vec![arc.center, arc.start, arc.end, arc.radius],
        witness,
        status,
    }
}

fn check_normal2(
    sketch: &SketchSolveProblem,
    entity: SketchEntityHandle,
    entity_name: String,
    normal: &SketchNormal2,
    policy: PredicatePolicy,
) -> SketchDegeneracyCheck {
    let (witness, status) = match (
        parameter_value(sketch, normal.x),
        parameter_value(sketch, normal.y),
    ) {
        (Ok(x), Ok(y)) => {
            let squared_norm = x.clone() * x + y.clone() * y;
            (
                Some(squared_norm.clone()),
                classify_one_is_nondegenerate(&squared_norm, policy),
            )
        }
        (Err(status), _) | (_, Err(status)) => (None, status),
    };
    SketchDegeneracyCheck {
        entity,
        entity_name,
        kind: SketchDegeneracyKind::NonunitNormal2,
        related_entities: Vec::new(),
        witness,
        status,
    }
}

fn check_normal3(
    sketch: &SketchSolveProblem,
    entity: SketchEntityHandle,
    entity_name: String,
    normal: &SketchNormal3,
    policy: PredicatePolicy,
) -> SketchDegeneracyCheck {
    let values = [
        parameter_value(sketch, normal.w),
        parameter_value(sketch, normal.x),
        parameter_value(sketch, normal.y),
        parameter_value(sketch, normal.z),
    ];
    let (witness, status) =
        if let Some(error) = values.iter().find_map(|value| value.as_ref().err()) {
            (None, error.clone())
        } else {
            let mut squared_norm = Real::zero();
            for value in values.into_iter().map(Result::unwrap) {
                squared_norm = squared_norm + value.clone() * value;
            }
            (
                Some(squared_norm.clone()),
                classify_one_is_nondegenerate(&squared_norm, policy),
            )
        };
    SketchDegeneracyCheck {
        entity,
        entity_name,
        kind: SketchDegeneracyKind::NonunitNormal3,
        related_entities: Vec::new(),
        witness,
        status,
    }
}

fn check_workplane(
    sketch: &SketchSolveProblem,
    entity: SketchEntityHandle,
    entity_name: String,
    workplane: &SketchWorkplane,
    policy: PredicatePolicy,
) -> SketchDegeneracyCheck {
    let (witness, status) = match entity_kind(sketch, workplane.normal) {
        Ok(SketchEntityKind::Normal2D(normal)) => {
            let check = check_normal2(sketch, entity, entity_name.clone(), normal, policy);
            (check.witness, normal_status_to_workplane(check.status))
        }
        Ok(SketchEntityKind::Normal3D(normal)) => {
            let check = check_normal3(sketch, entity, entity_name.clone(), normal, policy);
            (check.witness, normal_status_to_workplane(check.status))
        }
        Ok(_) => (
            None,
            SketchDegeneracyStatus::WrongEntityKind {
                handle: workplane.normal,
                expected: "normal",
            },
        ),
        Err(status) => (None, status),
    };
    SketchDegeneracyCheck {
        entity,
        entity_name,
        kind: SketchDegeneracyKind::SingularWorkplane,
        related_entities: vec![workplane.origin, workplane.normal],
        witness,
        status,
    }
}

fn normal_status_to_workplane(status: SketchDegeneracyStatus) -> SketchDegeneracyStatus {
    match status {
        SketchDegeneracyStatus::CertifiedNondegenerate => {
            SketchDegeneracyStatus::CertifiedNondegenerate
        }
        SketchDegeneracyStatus::CertifiedDegenerate => SketchDegeneracyStatus::CertifiedDegenerate,
        other => other,
    }
}

fn classify_zero_is_degenerate(value: &Real, policy: PredicatePolicy) -> SketchDegeneracyStatus {
    match compare_reals_with_policy(value, &Real::zero(), policy).value() {
        Some(Ordering::Equal) => SketchDegeneracyStatus::CertifiedDegenerate,
        Some(Ordering::Less | Ordering::Greater) => SketchDegeneracyStatus::CertifiedNondegenerate,
        None => SketchDegeneracyStatus::Unknown,
    }
}

fn classify_one_is_nondegenerate(value: &Real, policy: PredicatePolicy) -> SketchDegeneracyStatus {
    match compare_reals_with_policy(value, &Real::one(), policy).value() {
        Some(Ordering::Equal) => SketchDegeneracyStatus::CertifiedNondegenerate,
        Some(Ordering::Less | Ordering::Greater) => SketchDegeneracyStatus::CertifiedDegenerate,
        None => SketchDegeneracyStatus::Unknown,
    }
}

fn point2_coordinates(
    sketch: &SketchSolveProblem,
    handle: SketchEntityHandle,
) -> Result<[Real; 2], SketchDegeneracyStatus> {
    match entity_kind(sketch, handle)? {
        SketchEntityKind::Point2D(point) => Ok([
            parameter_value(sketch, point.x)?,
            parameter_value(sketch, point.y)?,
        ]),
        _ => Err(SketchDegeneracyStatus::WrongEntityKind {
            handle,
            expected: "2D point",
        }),
    }
}

fn distance_value(
    sketch: &SketchSolveProblem,
    handle: SketchEntityHandle,
) -> Result<Real, SketchDegeneracyStatus> {
    match entity_kind(sketch, handle)? {
        SketchEntityKind::Distance(SketchDistance { value }) => parameter_value(sketch, *value),
        _ => Err(SketchDegeneracyStatus::WrongEntityKind {
            handle,
            expected: "distance",
        }),
    }
}

fn entity_kind(
    sketch: &SketchSolveProblem,
    handle: SketchEntityHandle,
) -> Result<&SketchEntityKind, SketchDegeneracyStatus> {
    sketch
        .entities()
        .get(handle.0 as usize)
        .filter(|entity| entity.handle == handle)
        .map(|entity| &entity.kind)
        .ok_or(SketchDegeneracyStatus::MissingEntity { handle })
}

fn parameter_value(
    sketch: &SketchSolveProblem,
    handle: SketchParameterHandle,
) -> Result<Real, SketchDegeneracyStatus> {
    sketch
        .parameters()
        .get(handle.0 as usize)
        .filter(|parameter| parameter.handle == handle)
        .map(|parameter| parameter.value.clone())
        .ok_or(SketchDegeneracyStatus::MissingParameter { handle })
}

fn squared_distance2(a: &[Real; 2], b: &[Real; 2]) -> Real {
    let dx = a[0].clone() - b[0].clone();
    let dy = a[1].clone() - b[1].clone();
    dx.clone() * dx + dy.clone() * dy
}
