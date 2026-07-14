//! Sketch entity-domain preflight certificates.
//!
//! Entity domains are the retained-object counterpart to scalar parameter
//! domains. They certify conditions such as unit normals, positive radii, and
//! nonzero tangent carriers before numerical proposal engines run. The checks
//! return explicit proof rows instead of normalizing data or applying epsilon
//! repairs, following the exact-geometric-computation boundary: construction
//! inputs must be certified or carried forward as uncertainty.
//! the exact-geometric-computation model. The unit-normal and tangent-carrier obligations also mirror the
//! retained normal and endpoint/orientation data exposed by SolveSpace's
//! public sketch entity/constraint model.

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals_with_policy};
use hyperreal::Real;

use crate::sketch::{
    SketchArcOfCircle2, SketchCircle2, SketchDistance, SketchEntityDomain, SketchEntityHandle,
    SketchEntityKind, SketchLineSegment2, SketchLineSegment3, SketchNormal2, SketchNormal3,
    SketchParameterHandle, SketchPoint2, SketchSolveProblem,
};

/// Stable family name for a retained sketch entity-domain check.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SketchEntityDomainKind {
    /// Unit squared length for 2D/3D normals.
    UnitNormal,
    /// Strictly positive retained distance/radius.
    PositiveRadius,
    /// Nonzero retained 2D line length.
    NonzeroLengthLineSegment2,
    /// Nonzero retained 3D line length.
    NonzeroLengthLineSegment3,
    /// Distinct retained start/end points for a 2D arc.
    NondegenerateArc2,
    /// Nonzero retained 2D tangent-carrier direction.
    NonzeroTangentLineSegment2,
}

/// Proof status for one retained entity-domain check.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SketchEntityDomainStatus {
    /// The entity-domain condition was proved exactly.
    CertifiedValid,
    /// The entity-domain condition was proved false.
    CertifiedInvalid,
    /// Exact comparison was not decided under the supplied policy.
    Unknown,
    /// The checked entity handle was missing.
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

impl SketchEntityDomainStatus {
    /// Returns whether this status proves the entity-domain condition.
    pub const fn is_certified_valid(&self) -> bool {
        matches!(self, Self::CertifiedValid)
    }

    /// Returns whether this status proves an entity-domain violation.
    pub const fn is_certified_invalid(&self) -> bool {
        matches!(self, Self::CertifiedInvalid)
    }

    /// Returns whether this status is caused by stale or incompatible retained
    /// references rather than an exact value comparison.
    pub const fn is_invalid_reference(&self) -> bool {
        matches!(
            self,
            Self::MissingEntity { .. }
                | Self::MissingParameter { .. }
                | Self::WrongEntityKind { .. }
        )
    }
}

/// One exact preflight check for a retained sketch entity.
#[derive(Clone, Debug, PartialEq)]
pub struct SketchEntityDomainCheck {
    /// Source entity handle.
    pub entity: SketchEntityHandle,
    /// Source entity name.
    pub entity_name: String,
    /// Domain ordinal in the retained entity's domain list.
    pub domain_index: usize,
    /// Checked domain payload.
    pub domain: SketchEntityDomain,
    /// Compact domain family.
    pub kind: SketchEntityDomainKind,
    /// Related entity handles consumed by the check.
    pub related_entities: Vec<SketchEntityHandle>,
    /// Exact witness value when the check reduces to a scalar expression.
    pub witness: Option<Real>,
    /// Proof status.
    pub status: SketchEntityDomainStatus,
}

/// Report for sketch entity-domain preflight.
#[derive(Clone, Debug, PartialEq)]
pub struct SketchEntityDomainReport {
    /// Per-domain proof rows.
    pub checks: Vec<SketchEntityDomainCheck>,
    /// Number of retained entities examined.
    pub entity_count: usize,
    /// Number of entity-domain checks proved valid.
    pub certified_valid_checks: usize,
    /// Number of entity-domain checks proved invalid.
    pub certified_invalid_checks: usize,
    /// Number of entity-domain checks left unresolved.
    pub unknown_checks: usize,
    /// Number of checks blocked by stale or wrong retained references.
    pub invalid_reference_checks: usize,
}

impl SketchEntityDomainReport {
    /// Returns true when at least one entity-domain check proves invalid
    /// retained data.
    pub fn has_certified_invalid_domain(&self) -> bool {
        self.certified_invalid_checks > 0
    }

    /// Returns true when every retained entity-domain check has a validity
    /// proof and no stale references were encountered.
    pub fn all_certified_valid(&self) -> bool {
        self.certified_invalid_checks == 0
            && self.unknown_checks == 0
            && self.invalid_reference_checks == 0
    }
}

/// Certify retained sketch entity-domain obligations with the default
/// predicate policy.
pub fn preflight_sketch_entity_domains(sketch: &SketchSolveProblem) -> SketchEntityDomainReport {
    preflight_sketch_entity_domains_with_policy(sketch, PredicatePolicy)
}

/// Certify retained sketch entity-domain obligations with an explicit
/// predicate policy.
///
/// These checks inspect the exact retained entity parameters, not a later
/// proposed candidate. Candidate-time entity-domain obligations should be
/// represented as residual or predicate certificates alongside that candidate.
pub fn preflight_sketch_entity_domains_with_policy(
    sketch: &SketchSolveProblem,
    policy: PredicatePolicy,
) -> SketchEntityDomainReport {
    let mut checks = Vec::new();
    for entity in sketch.entities() {
        for (domain_index, domain) in entity.domains.iter().cloned().enumerate() {
            checks.push(check_entity_domain(
                sketch,
                entity.handle,
                entity.name.clone(),
                domain_index,
                domain,
                &entity.kind,
                policy,
            ));
        }
    }

    let certified_valid_checks = checks
        .iter()
        .filter(|check| check.status.is_certified_valid())
        .count();
    let certified_invalid_checks = checks
        .iter()
        .filter(|check| check.status.is_certified_invalid())
        .count();
    let unknown_checks = checks
        .iter()
        .filter(|check| check.status == SketchEntityDomainStatus::Unknown)
        .count();
    let invalid_reference_checks = checks
        .iter()
        .filter(|check| check.status.is_invalid_reference())
        .count();

    SketchEntityDomainReport {
        checks,
        entity_count: sketch.entities().len(),
        certified_valid_checks,
        certified_invalid_checks,
        unknown_checks,
        invalid_reference_checks,
    }
}

fn check_entity_domain(
    sketch: &SketchSolveProblem,
    entity: SketchEntityHandle,
    entity_name: String,
    domain_index: usize,
    domain: SketchEntityDomain,
    kind: &SketchEntityKind,
    policy: PredicatePolicy,
) -> SketchEntityDomainCheck {
    let domain_kind = domain_kind(&domain);
    let (related_entities, witness, status) = match (&domain, kind) {
        (SketchEntityDomain::UnitNormal, SketchEntityKind::Normal2D(normal)) => {
            check_normal2(sketch, normal, policy)
        }
        (SketchEntityDomain::UnitNormal, SketchEntityKind::Normal3D(normal)) => {
            check_normal3(sketch, normal, policy)
        }
        (SketchEntityDomain::UnitNormal, _) => (
            Vec::new(),
            None,
            SketchEntityDomainStatus::WrongEntityKind {
                handle: entity,
                expected: "normal",
            },
        ),
        (SketchEntityDomain::PositiveRadius, SketchEntityKind::Distance(distance)) => {
            check_distance_positive(sketch, distance, policy)
        }
        (SketchEntityDomain::PositiveRadius, SketchEntityKind::Circle2(circle)) => {
            check_circle_radius_positive(sketch, circle, policy)
        }
        (SketchEntityDomain::PositiveRadius, SketchEntityKind::ArcOfCircle2(arc)) => {
            check_arc_radius_positive(sketch, arc, policy)
        }
        (SketchEntityDomain::PositiveRadius, _) => (
            Vec::new(),
            None,
            SketchEntityDomainStatus::WrongEntityKind {
                handle: entity,
                expected: "distance, circle, or arc",
            },
        ),
        (
            SketchEntityDomain::NonzeroLengthLineSegment2
            | SketchEntityDomain::NonzeroTangentLineSegment2,
            SketchEntityKind::LineSegment2(line),
        ) => check_line_segment_nonzero(sketch, line, policy),
        (SketchEntityDomain::NonzeroLengthLineSegment3, SketchEntityKind::LineSegment3(line)) => {
            check_line_segment3_nonzero(sketch, line, policy)
        }
        (SketchEntityDomain::NonzeroLengthLineSegment2, _) => (
            Vec::new(),
            None,
            SketchEntityDomainStatus::WrongEntityKind {
                handle: entity,
                expected: "2D line segment",
            },
        ),
        (SketchEntityDomain::NonzeroLengthLineSegment3, _) => (
            Vec::new(),
            None,
            SketchEntityDomainStatus::WrongEntityKind {
                handle: entity,
                expected: "3D line segment",
            },
        ),
        (SketchEntityDomain::NonzeroTangentLineSegment2, _) => (
            Vec::new(),
            None,
            SketchEntityDomainStatus::WrongEntityKind {
                handle: entity,
                expected: "2D tangent line segment",
            },
        ),
        (SketchEntityDomain::NondegenerateArc2, SketchEntityKind::ArcOfCircle2(arc)) => {
            check_arc_nondegenerate(sketch, arc, policy)
        }
        (SketchEntityDomain::NondegenerateArc2, _) => (
            Vec::new(),
            None,
            SketchEntityDomainStatus::WrongEntityKind {
                handle: entity,
                expected: "2D circular arc",
            },
        ),
    };

    SketchEntityDomainCheck {
        entity,
        entity_name,
        domain_index,
        domain,
        kind: domain_kind,
        related_entities,
        witness,
        status,
    }
}

fn domain_kind(domain: &SketchEntityDomain) -> SketchEntityDomainKind {
    match domain {
        SketchEntityDomain::UnitNormal => SketchEntityDomainKind::UnitNormal,
        SketchEntityDomain::PositiveRadius => SketchEntityDomainKind::PositiveRadius,
        SketchEntityDomain::NonzeroLengthLineSegment2 => {
            SketchEntityDomainKind::NonzeroLengthLineSegment2
        }
        SketchEntityDomain::NonzeroLengthLineSegment3 => {
            SketchEntityDomainKind::NonzeroLengthLineSegment3
        }
        SketchEntityDomain::NondegenerateArc2 => SketchEntityDomainKind::NondegenerateArc2,
        SketchEntityDomain::NonzeroTangentLineSegment2 => {
            SketchEntityDomainKind::NonzeroTangentLineSegment2
        }
    }
}

fn check_normal2(
    sketch: &SketchSolveProblem,
    normal: &SketchNormal2,
    policy: PredicatePolicy,
) -> (
    Vec<SketchEntityHandle>,
    Option<Real>,
    SketchEntityDomainStatus,
) {
    match (
        parameter_value(sketch, normal.x),
        parameter_value(sketch, normal.y),
    ) {
        (Ok(x), Ok(y)) => {
            let witness = x.clone() * x + y.clone() * y - Real::one();
            let status = classify_zero_is_valid(&witness, policy);
            (Vec::new(), Some(witness), status)
        }
        (Err(status), _) | (_, Err(status)) => (Vec::new(), None, status),
    }
}

fn check_normal3(
    sketch: &SketchSolveProblem,
    normal: &SketchNormal3,
    policy: PredicatePolicy,
) -> (
    Vec<SketchEntityHandle>,
    Option<Real>,
    SketchEntityDomainStatus,
) {
    match (
        parameter_value(sketch, normal.w),
        parameter_value(sketch, normal.x),
        parameter_value(sketch, normal.y),
        parameter_value(sketch, normal.z),
    ) {
        (Ok(w), Ok(x), Ok(y), Ok(z)) => {
            let witness =
                w.clone() * w + x.clone() * x + y.clone() * y + z.clone() * z - Real::one();
            let status = classify_zero_is_valid(&witness, policy);
            (Vec::new(), Some(witness), status)
        }
        (Err(status), _, _, _)
        | (_, Err(status), _, _)
        | (_, _, Err(status), _)
        | (_, _, _, Err(status)) => (Vec::new(), None, status),
    }
}

fn check_distance_positive(
    sketch: &SketchSolveProblem,
    distance: &SketchDistance,
    policy: PredicatePolicy,
) -> (
    Vec<SketchEntityHandle>,
    Option<Real>,
    SketchEntityDomainStatus,
) {
    match parameter_value(sketch, distance.value) {
        Ok(value) => {
            let status = classify_positive_is_valid(&value, policy);
            (Vec::new(), Some(value), status)
        }
        Err(status) => (Vec::new(), None, status),
    }
}

fn check_circle_radius_positive(
    sketch: &SketchSolveProblem,
    circle: &SketchCircle2,
    policy: PredicatePolicy,
) -> (
    Vec<SketchEntityHandle>,
    Option<Real>,
    SketchEntityDomainStatus,
) {
    match distance_value(sketch, circle.radius) {
        Ok(value) => {
            let status = classify_positive_is_valid(&value, policy);
            (vec![circle.radius], Some(value), status)
        }
        Err(status) => (vec![circle.radius], None, status),
    }
}

fn check_arc_radius_positive(
    sketch: &SketchSolveProblem,
    arc: &SketchArcOfCircle2,
    policy: PredicatePolicy,
) -> (
    Vec<SketchEntityHandle>,
    Option<Real>,
    SketchEntityDomainStatus,
) {
    match distance_value(sketch, arc.radius) {
        Ok(value) => {
            let status = classify_positive_is_valid(&value, policy);
            (vec![arc.radius], Some(value), status)
        }
        Err(status) => (vec![arc.radius], None, status),
    }
}

fn check_line_segment_nonzero(
    sketch: &SketchSolveProblem,
    line: &SketchLineSegment2,
    policy: PredicatePolicy,
) -> (
    Vec<SketchEntityHandle>,
    Option<Real>,
    SketchEntityDomainStatus,
) {
    match (
        point2_coordinates(sketch, line.start),
        point2_coordinates(sketch, line.end),
    ) {
        (Ok(start), Ok(end)) => {
            let witness = squared_distance2(&start, &end);
            let status = classify_positive_is_valid(&witness, policy);
            (vec![line.start, line.end], Some(witness), status)
        }
        (Err(status), _) | (_, Err(status)) => (vec![line.start, line.end], None, status),
    }
}

fn check_line_segment3_nonzero(
    sketch: &SketchSolveProblem,
    line: &SketchLineSegment3,
    policy: PredicatePolicy,
) -> (
    Vec<SketchEntityHandle>,
    Option<Real>,
    SketchEntityDomainStatus,
) {
    match (
        point3_coordinates(sketch, line.start),
        point3_coordinates(sketch, line.end),
    ) {
        (Ok(start), Ok(end)) => {
            let witness = squared_distance3(&start, &end);
            let status = classify_positive_is_valid(&witness, policy);
            (vec![line.start, line.end], Some(witness), status)
        }
        (Err(status), _) | (_, Err(status)) => (vec![line.start, line.end], None, status),
    }
}

fn check_arc_nondegenerate(
    sketch: &SketchSolveProblem,
    arc: &SketchArcOfCircle2,
    policy: PredicatePolicy,
) -> (
    Vec<SketchEntityHandle>,
    Option<Real>,
    SketchEntityDomainStatus,
) {
    match (
        point2_coordinates(sketch, arc.start),
        point2_coordinates(sketch, arc.end),
    ) {
        (Ok(start), Ok(end)) => {
            let witness = squared_distance2(&start, &end);
            let status = classify_positive_is_valid(&witness, policy);
            (vec![arc.start, arc.end], Some(witness), status)
        }
        (Err(status), _) | (_, Err(status)) => (vec![arc.start, arc.end], None, status),
    }
}

fn classify_zero_is_valid(value: &Real, policy: PredicatePolicy) -> SketchEntityDomainStatus {
    match compare_reals_with_policy(value, &Real::zero(), policy).value() {
        Some(Ordering::Equal) => SketchEntityDomainStatus::CertifiedValid,
        Some(Ordering::Less | Ordering::Greater) => SketchEntityDomainStatus::CertifiedInvalid,
        None => SketchEntityDomainStatus::Unknown,
    }
}

fn classify_positive_is_valid(value: &Real, policy: PredicatePolicy) -> SketchEntityDomainStatus {
    match compare_reals_with_policy(value, &Real::zero(), policy).value() {
        Some(Ordering::Greater) => SketchEntityDomainStatus::CertifiedValid,
        Some(Ordering::Less | Ordering::Equal) => SketchEntityDomainStatus::CertifiedInvalid,
        None => SketchEntityDomainStatus::Unknown,
    }
}

fn point2_coordinates(
    sketch: &SketchSolveProblem,
    handle: SketchEntityHandle,
) -> Result<[Real; 2], SketchEntityDomainStatus> {
    match entity_kind(sketch, handle)? {
        SketchEntityKind::Point2D(point) => point2_values(sketch, point),
        _ => Err(SketchEntityDomainStatus::WrongEntityKind {
            handle,
            expected: "2D point",
        }),
    }
}

fn point3_coordinates(
    sketch: &SketchSolveProblem,
    handle: SketchEntityHandle,
) -> Result<[Real; 3], SketchEntityDomainStatus> {
    match entity_kind(sketch, handle)? {
        SketchEntityKind::Point3D(point) => point3_values(sketch, point),
        _ => Err(SketchEntityDomainStatus::WrongEntityKind {
            handle,
            expected: "3D point",
        }),
    }
}

fn point2_values(
    sketch: &SketchSolveProblem,
    point: &SketchPoint2,
) -> Result<[Real; 2], SketchEntityDomainStatus> {
    Ok([
        parameter_value(sketch, point.x)?,
        parameter_value(sketch, point.y)?,
    ])
}

fn point3_values(
    sketch: &SketchSolveProblem,
    point: &crate::sketch::SketchPoint3,
) -> Result<[Real; 3], SketchEntityDomainStatus> {
    Ok([
        parameter_value(sketch, point.x)?,
        parameter_value(sketch, point.y)?,
        parameter_value(sketch, point.z)?,
    ])
}

fn distance_value(
    sketch: &SketchSolveProblem,
    handle: SketchEntityHandle,
) -> Result<Real, SketchEntityDomainStatus> {
    match entity_kind(sketch, handle)? {
        SketchEntityKind::Distance(distance) => parameter_value(sketch, distance.value),
        _ => Err(SketchEntityDomainStatus::WrongEntityKind {
            handle,
            expected: "distance",
        }),
    }
}

fn entity_kind(
    sketch: &SketchSolveProblem,
    handle: SketchEntityHandle,
) -> Result<&SketchEntityKind, SketchEntityDomainStatus> {
    sketch
        .entities()
        .get(handle.0 as usize)
        .filter(|entity| entity.handle == handle)
        .map(|entity| &entity.kind)
        .ok_or(SketchEntityDomainStatus::MissingEntity { handle })
}

fn parameter_value(
    sketch: &SketchSolveProblem,
    handle: SketchParameterHandle,
) -> Result<Real, SketchEntityDomainStatus> {
    sketch
        .parameters()
        .get(handle.0 as usize)
        .filter(|parameter| parameter.handle == handle)
        .map(|parameter| parameter.value.clone())
        .ok_or(SketchEntityDomainStatus::MissingParameter { handle })
}

fn squared_distance2(start: &[Real; 2], end: &[Real; 2]) -> Real {
    let dx = start[0].clone() - end[0].clone();
    let dy = start[1].clone() - end[1].clone();
    dx.clone() * dx + dy.clone() * dy
}

fn squared_distance3(start: &[Real; 3], end: &[Real; 3]) -> Real {
    let dx = start[0].clone() - end[0].clone();
    let dy = start[1].clone() - end[1].clone();
    let dz = start[2].clone() - end[2].clone();
    dx.clone() * dx + dy.clone() * dy + dz.clone() * dz
}
