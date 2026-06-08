//! Certified closed-convex solid facts.
//!
//! The routines in this module are intentionally narrow: they certify a closed
//! triangular mesh as an oriented convex polyhedron and classify points or
//! vertex sets with exact oriented halfspace predicates. They do not implement
//! arbitrary winding. Unsupported cases stay explicit rather than hidden behind
//! approximate representative points.

use std::cmp::Ordering;

use hyperlimit::{PlaneSide, Point3, compare_reals, orient3d_report};

use super::mesh::ExactMesh;
use hyperlimit::PredicateUse;
use hyperreal::Real;

/// Certified orientation of a closed triangular surface.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClosedMeshOrientation {
    /// The mesh is not a closed two-manifold under exact validation facts.
    NotClosed,
    /// The signed volume was certified positive.
    Positive,
    /// The signed volume was certified negative.
    Negative,
    /// The signed volume sign could not be certified or was zero.
    Unknown,
}

/// Certified convexity state for a closed triangular surface.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConvexSolidClassification {
    /// The mesh is not a closed two-manifold under exact validation facts.
    NotClosed,
    /// Every vertex is on or inside every oriented face halfspace.
    Convex,
    /// At least one vertex was certified outside an oriented face halfspace.
    NonConvex,
    /// A required orientation or halfspace predicate was undecided.
    Unknown,
}

/// Structural inconsistency found in a convex-solid report.
///
/// These checks validate the report model itself, not the geometry from
/// scratch. They are intentionally tied to the certificate-carrying APIs in
/// justified by retained exact facts. A report that contradicts those retained
/// facts must be treated as invalid before a shortcut consumes it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConvexSolidReportError {
    /// `NotClosed` orientation and convexity states were not paired.
    NotClosedStateMismatch,
    /// Unknown orientation was paired with a decided convexity state.
    UnknownOrientationHasDecidedConvexity,
    /// A certified orientation was paired with `Unknown` or `NotClosed`
    /// convexity in a way this module never constructs.
    OrientedStateHasUnsupportedConvexity,
    /// A non-certified state retained point halfspace predicates.
    NonCertifiedPointHasPredicates,
    /// A mesh/solid report claims a certified relation without certified
    /// convex solid facts.
    CertifiedMeshRelationWithoutCertifiedSolid,
    /// A non-certified mesh/solid report retained per-vertex classifications.
    NonCertifiedMeshHasVertices,
    /// A per-vertex relation cannot appear in a certified mesh/solid summary.
    UnexpectedVertexRelation,
    /// The per-vertex relations do not support the retained mesh/solid
    /// summary relation.
    MeshRelationMismatch,
    /// A nested solid-facts or point-classification report was invalid.
    NestedReport,
    /// The retained report no longer matches facts recomputed from the
    /// supplied source solid, point, or subject mesh.
    SourceReplayMismatch,
}

/// Freshness status for retained convex-solid reports.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConvexSolidReportFreshness {
    /// The report is locally valid and replays from source evidence.
    Current,
    /// Retained solid orientation/convexity facts are internally inconsistent.
    StaleSolidFacts,
    /// Retained point/solid predicate evidence is internally inconsistent.
    StalePointEvidence,
    /// Retained mesh/solid summary evidence is internally inconsistent.
    StaleMeshEvidence,
    /// A nested retained convex report failed its own audit.
    InvalidNestedReport,
    /// The report is locally valid but no longer replays from source evidence.
    SourceReplayMismatch,
}

impl From<ConvexSolidReportError> for ConvexSolidReportFreshness {
    fn from(error: ConvexSolidReportError) -> Self {
        match error {
            ConvexSolidReportError::NotClosedStateMismatch
            | ConvexSolidReportError::UnknownOrientationHasDecidedConvexity
            | ConvexSolidReportError::OrientedStateHasUnsupportedConvexity => Self::StaleSolidFacts,
            ConvexSolidReportError::NonCertifiedPointHasPredicates => Self::StalePointEvidence,
            ConvexSolidReportError::CertifiedMeshRelationWithoutCertifiedSolid
            | ConvexSolidReportError::NonCertifiedMeshHasVertices
            | ConvexSolidReportError::UnexpectedVertexRelation
            | ConvexSolidReportError::MeshRelationMismatch => Self::StaleMeshEvidence,
            ConvexSolidReportError::NestedReport => Self::InvalidNestedReport,
            ConvexSolidReportError::SourceReplayMismatch => Self::SourceReplayMismatch,
        }
    }
}

/// Exact facts retained while certifying a closed convex solid.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConvexSolidFacts {
    /// Certified closed-surface orientation.
    pub orientation: ClosedMeshOrientation,
    /// Certified convexity state.
    pub convexity: ConvexSolidClassification,
    /// Predicate certificates used by face/vertex halfspace tests.
    pub predicates: Vec<PredicateUse>,
}

impl ConvexSolidFacts {
    /// Return whether the mesh is certified as an oriented convex closed solid.
    pub const fn is_certified_convex(&self) -> bool {
        matches!(
            (self.orientation, self.convexity),
            (
                ClosedMeshOrientation::Positive | ClosedMeshOrientation::Negative,
                ConvexSolidClassification::Convex
            )
        )
    }

    /// Return whether every retained predicate route was proof-producing.
    pub fn all_proof_producing(&self) -> bool {
        self.predicates
            .iter()
            .copied()
            .all(PredicateUse::is_proof_producing)
    }

    /// Validate report invariants retained by [`certify_convex_solid`].
    ///
    /// This does not recompute convexity. It checks that the state tuple and
    /// predicate retention are consistent with the certified-construction
    /// contract used by the exact boolean shortcuts.
    pub fn validate(&self) -> Result<(), ConvexSolidReportError> {
        match (self.orientation, self.convexity) {
            (ClosedMeshOrientation::NotClosed, ConvexSolidClassification::NotClosed) => Ok(()),
            (ClosedMeshOrientation::NotClosed, _) | (_, ConvexSolidClassification::NotClosed) => {
                Err(ConvexSolidReportError::NotClosedStateMismatch)
            }
            (ClosedMeshOrientation::Unknown, ConvexSolidClassification::Unknown) => Ok(()),
            (ClosedMeshOrientation::Unknown, _) => {
                Err(ConvexSolidReportError::UnknownOrientationHasDecidedConvexity)
            }
            (
                ClosedMeshOrientation::Positive | ClosedMeshOrientation::Negative,
                ConvexSolidClassification::Convex | ConvexSolidClassification::NonConvex,
            ) => Ok(()),
            (
                ClosedMeshOrientation::Positive | ClosedMeshOrientation::Negative,
                ConvexSolidClassification::Unknown,
            ) => Ok(()),
        }
    }

    /// Validate these retained facts against their source mesh.
    ///
    /// The local validator checks only the state tuple and retained predicate
    /// shape. This replay check recomputes convex-solid certification from the
    /// boundary between a coherent certificate object and one still attached
    /// to the particular geometry whose predicates produced it.
    pub fn validate_against_source(&self, mesh: &ExactMesh) -> Result<(), ConvexSolidReportError> {
        self.validate()?;
        if self == &certify_convex_solid(mesh) {
            Ok(())
        } else {
            Err(ConvexSolidReportError::SourceReplayMismatch)
        }
    }

    /// Classify whether these retained convex-solid facts are fresh for `mesh`.
    pub fn freshness_against_source(&self, mesh: &ExactMesh) -> ConvexSolidReportFreshness {
        match self.validate_against_source(mesh) {
            Ok(()) => ConvexSolidReportFreshness::Current,
            Err(error) => error.into(),
        }
    }
}

/// Certified point/solid classification with retained predicate provenance.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConvexSolidPointClassification {
    /// Exact point/solid relation.
    pub relation: ConvexSolidPointRelation,
    /// Predicate certificates used by the halfspace tests for this point.
    pub predicates: Vec<PredicateUse>,
}

impl ConvexSolidPointClassification {
    /// Return whether every retained predicate route was proof-producing.
    pub fn all_proof_producing(&self) -> bool {
        self.predicates
            .iter()
            .copied()
            .all(PredicateUse::is_proof_producing)
    }

    /// Validate point/solid classification report invariants.
    ///
    /// Non-certified point reports are produced before any face halfspace
    /// predicate is meaningful, so they must not carry predicate evidence.
    /// Decided and unknown certified relations may retain a prefix of exact
    /// face predicates because outside/unknown exits short-circuit.
    pub fn validate(&self) -> Result<(), ConvexSolidReportError> {
        if matches!(self.relation, ConvexSolidPointRelation::NotCertifiedConvex)
            && !self.predicates.is_empty()
        {
            return Err(ConvexSolidReportError::NonCertifiedPointHasPredicates);
        }
        Ok(())
    }

    /// Validate this point/solid classification against its source objects.
    ///
    /// This recomputes the point halfspace walk from `point` and `solid` after
    /// the local report audit succeeds. Keeping the retained relation and
    /// predicate certificates replayable against the original source objects
    /// from the exact predicates that justified it.
    pub fn validate_against_sources(
        &self,
        point: &Point3,
        solid: &ExactMesh,
    ) -> Result<(), ConvexSolidReportError> {
        self.validate()?;
        if self == &classify_point_against_convex_solid_report(point, solid) {
            Ok(())
        } else {
            Err(ConvexSolidReportError::SourceReplayMismatch)
        }
    }

    /// Classify whether this retained point/solid report is fresh.
    pub fn freshness_against_sources(
        &self,
        point: &Point3,
        solid: &ExactMesh,
    ) -> ConvexSolidReportFreshness {
        match self.validate_against_sources(point, solid) {
            Ok(()) => ConvexSolidReportFreshness::Current,
            Err(error) => error.into(),
        }
    }
}

/// Exact relation between a point and a certified convex solid.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConvexSolidPointRelation {
    /// The containing mesh was not certified as a convex closed solid.
    NotCertifiedConvex,
    /// The point is strictly inside all oriented halfspaces.
    Inside,
    /// The point is on at least one boundary plane and outside none.
    Boundary,
    /// The point is certified outside at least one oriented halfspace.
    Outside,
    /// A required predicate was undecided.
    Unknown,
}

/// Exact relation between one mesh's vertices and a certified convex solid.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConvexSolidMeshRelation {
    /// The containing mesh was not certified as a convex closed solid.
    NotCertifiedConvex,
    /// Every subject vertex is strictly inside the solid.
    StrictlyInside,
    /// No subject vertex is inside the solid.
    Outside,
    /// Subject vertices touch the boundary, mix boundary/interior states, or
    /// otherwise require a full winding/surface-overlap policy.
    BoundaryOrMixed,
    /// A required predicate was undecided.
    Unknown,
}

/// Certified mesh/solid vertex classification with retained predicate provenance.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConvexSolidMeshClassification {
    /// Exact relation between the subject vertices and the convex solid.
    pub relation: ConvexSolidMeshRelation,
    /// Convexity facts certified for the containing solid.
    pub solid_facts: ConvexSolidFacts,
    /// Per-subject-vertex classifications.
    pub vertices: Vec<ConvexSolidPointClassification>,
}

impl ConvexSolidMeshClassification {
    /// Return whether every retained predicate route was proof-producing.
    pub fn all_proof_producing(&self) -> bool {
        self.solid_facts.all_proof_producing()
            && self
                .vertices
                .iter()
                .all(ConvexSolidPointClassification::all_proof_producing)
    }

    /// Validate mesh/solid vertex-classification report invariants.
    ///
    /// The mesh summary must be derivable from the retained per-vertex point
    /// reports. This is the local audit check for convex-containment boolean
    /// shortcuts; it keeps the topological decision coupled to the exact
    /// certified decisions and explicit uncertainty.
    pub fn validate(&self) -> Result<(), ConvexSolidReportError> {
        self.solid_facts
            .validate()
            .map_err(|_| ConvexSolidReportError::NestedReport)?;

        if !self.solid_facts.is_certified_convex() {
            return if matches!(self.relation, ConvexSolidMeshRelation::NotCertifiedConvex)
                && self.vertices.is_empty()
            {
                Ok(())
            } else if !self.vertices.is_empty() {
                Err(ConvexSolidReportError::NonCertifiedMeshHasVertices)
            } else {
                Err(ConvexSolidReportError::CertifiedMeshRelationWithoutCertifiedSolid)
            };
        }

        if matches!(self.relation, ConvexSolidMeshRelation::NotCertifiedConvex) {
            return Err(ConvexSolidReportError::CertifiedMeshRelationWithoutCertifiedSolid);
        }

        if matches!(self.relation, ConvexSolidMeshRelation::Unknown) {
            if self.vertices.is_empty() {
                return Err(ConvexSolidReportError::MeshRelationMismatch);
            }
            for (index, vertex) in self.vertices.iter().enumerate() {
                vertex
                    .validate()
                    .map_err(|_| ConvexSolidReportError::NestedReport)?;
                match vertex.relation {
                    ConvexSolidPointRelation::Inside
                    | ConvexSolidPointRelation::Boundary
                    | ConvexSolidPointRelation::Outside => {}
                    ConvexSolidPointRelation::Unknown => {
                        return if index + 1 == self.vertices.len() {
                            Ok(())
                        } else {
                            Err(ConvexSolidReportError::MeshRelationMismatch)
                        };
                    }
                    ConvexSolidPointRelation::NotCertifiedConvex => {
                        return Err(ConvexSolidReportError::UnexpectedVertexRelation);
                    }
                }
            }
            return Err(ConvexSolidReportError::MeshRelationMismatch);
        }

        let mut inside = 0_usize;
        let mut boundary = 0_usize;
        let mut outside = 0_usize;
        for vertex in &self.vertices {
            vertex
                .validate()
                .map_err(|_| ConvexSolidReportError::NestedReport)?;
            match vertex.relation {
                ConvexSolidPointRelation::Inside => inside += 1,
                ConvexSolidPointRelation::Boundary => boundary += 1,
                ConvexSolidPointRelation::Outside => outside += 1,
                ConvexSolidPointRelation::Unknown => {
                    return Err(ConvexSolidReportError::MeshRelationMismatch);
                }
                ConvexSolidPointRelation::NotCertifiedConvex => {
                    return Err(ConvexSolidReportError::UnexpectedVertexRelation);
                }
            }
        }

        let derived = match (inside, boundary, outside) {
            (_, 0, 0) if inside == self.vertices.len() => ConvexSolidMeshRelation::StrictlyInside,
            (0, 0, _) => ConvexSolidMeshRelation::Outside,
            _ => ConvexSolidMeshRelation::BoundaryOrMixed,
        };
        if self.relation == derived {
            Ok(())
        } else {
            Err(ConvexSolidReportError::MeshRelationMismatch)
        }
    }

    /// Validate this mesh/solid classification against its source meshes.
    ///
    /// The local audit proves that the summary relation follows from the
    /// retained per-vertex point classifications. This replay check
    /// recomputes the whole report from `subject` and `solid`, catching stale
    /// report objects that remain internally coherent but no longer belong to
    pub fn validate_against_sources(
        &self,
        subject: &ExactMesh,
        solid: &ExactMesh,
    ) -> Result<(), ConvexSolidReportError> {
        self.validate()?;
        if self == &classify_mesh_vertices_against_convex_solid_report(subject, solid) {
            Ok(())
        } else {
            Err(ConvexSolidReportError::SourceReplayMismatch)
        }
    }

    /// Classify whether this retained mesh/solid report is fresh.
    pub fn freshness_against_sources(
        &self,
        subject: &ExactMesh,
        solid: &ExactMesh,
    ) -> ConvexSolidReportFreshness {
        match self.validate_against_sources(subject, solid) {
            Ok(()) => ConvexSolidReportFreshness::Current,
            Err(error) => error.into(),
        }
    }
}

/// Certify whether a closed triangular mesh is an oriented convex solid.
///
/// Convexity is tested by replaying every mesh vertex against every oriented
/// face plane with `hyperlimit::orient3d_report`. `hyperlimit` uses the
/// translated determinant convention where the canonical tetrahedron
/// `(0,0,0),(1,0,0),(0,1,0),(0,0,1)` has negative orientation; with the signed
/// volume convention used here, interior points of a positively oriented
/// closed surface therefore lie on the above side of every face. The signed
/// volume orientation is exact `Real` arithmetic and is compared through
pub fn certify_convex_solid(mesh: &ExactMesh) -> ConvexSolidFacts {
    if !mesh.facts().mesh.closed_manifold {
        return ConvexSolidFacts {
            orientation: ClosedMeshOrientation::NotClosed,
            convexity: ConvexSolidClassification::NotClosed,
            predicates: Vec::new(),
        };
    }

    let orientation = exact_mesh_orientation(mesh);
    if !matches!(
        orientation,
        ClosedMeshOrientation::Positive | ClosedMeshOrientation::Negative
    ) {
        return ConvexSolidFacts {
            orientation,
            convexity: ConvexSolidClassification::Unknown,
            predicates: Vec::new(),
        };
    }

    let mut predicates = Vec::new();
    let mut saw_unknown = false;
    for triangle in mesh.triangles() {
        let tri = triangle.0;
        let a = mesh.vertices()[tri[0]].clone();
        let b = mesh.vertices()[tri[1]].clone();
        let c = mesh.vertices()[tri[2]].clone();

        for (vertex, point) in mesh.vertices().iter().enumerate() {
            if tri.contains(&vertex) {
                continue;
            }
            let report = orient3d_report(&a, &b, &c, &point.clone());
            predicates.push(PredicateUse::from_certificate(report.certificate));
            let Some(side) = report.value().map(PlaneSide::from) else {
                saw_unknown = true;
                continue;
            };
            if side_is_outside(orientation, side) {
                return ConvexSolidFacts {
                    orientation,
                    convexity: ConvexSolidClassification::NonConvex,
                    predicates,
                };
            }
        }
    }

    ConvexSolidFacts {
        orientation,
        convexity: if saw_unknown {
            ConvexSolidClassification::Unknown
        } else {
            ConvexSolidClassification::Convex
        },
        predicates,
    }
}

/// Classify one point against a certified convex solid.
pub fn classify_point_against_convex_solid(
    point: &Point3,
    solid: &ExactMesh,
) -> ConvexSolidPointRelation {
    classify_point_against_convex_solid_report(point, solid).relation
}

/// Classify one point against a certified convex solid and retain predicates.
///
/// This is the auditable form of [`classify_point_against_convex_solid`].
/// Each face halfspace query records the `hyperlimit::orient3d_report`
/// certificate that drove the relation. Keeping those certificates near the
/// the exact predicates they consumed rather than returning only a collapsed
/// boolean-like answer.
pub fn classify_point_against_convex_solid_report(
    point: &Point3,
    solid: &ExactMesh,
) -> ConvexSolidPointClassification {
    let facts = certify_convex_solid(solid);
    classify_point_with_convex_facts_report(point, solid, &facts)
}

/// Classify every vertex of `subject` against a certified convex solid.
///
/// This is a containment precondition for simple named-boolean shortcuts. It
/// is not a substitute for general winding: boundary and mixed relations remain
/// explicit so coplanar overlaps, shared faces, and partial intersections are
/// rejected by higher-level boolean policy.
pub fn classify_mesh_vertices_against_convex_solid(
    subject: &ExactMesh,
    solid: &ExactMesh,
) -> ConvexSolidMeshRelation {
    classify_mesh_vertices_against_convex_solid_report(subject, solid).relation
}

/// Classify every vertex of `subject` against a convex solid and retain predicates.
///
/// This report-returning API is the boolean-shortcut audit artifact. It
/// separates the solid certification predicates from the per-vertex halfspace
/// predicates so callers can inspect whether a containment/disjoint shortcut
/// contract used throughout the port: predicates and uncertainty stay explicit
/// at API boundaries.
pub fn classify_mesh_vertices_against_convex_solid_report(
    subject: &ExactMesh,
    solid: &ExactMesh,
) -> ConvexSolidMeshClassification {
    let facts = certify_convex_solid(solid);
    if !facts.is_certified_convex() {
        return ConvexSolidMeshClassification {
            relation: ConvexSolidMeshRelation::NotCertifiedConvex,
            solid_facts: facts,
            vertices: Vec::new(),
        };
    }

    let mut inside = 0_usize;
    let mut boundary = 0_usize;
    let mut outside = 0_usize;
    let mut vertices = Vec::with_capacity(subject.vertices().len());
    for vertex in subject.vertices() {
        let classification =
            classify_point_with_convex_facts_report(&vertex.clone(), solid, &facts);
        match classification.relation {
            ConvexSolidPointRelation::Inside => inside += 1,
            ConvexSolidPointRelation::Boundary => boundary += 1,
            ConvexSolidPointRelation::Outside => outside += 1,
            ConvexSolidPointRelation::Unknown => {
                vertices.push(classification);
                return ConvexSolidMeshClassification {
                    relation: ConvexSolidMeshRelation::Unknown,
                    solid_facts: facts,
                    vertices,
                };
            }
            ConvexSolidPointRelation::NotCertifiedConvex => {
                vertices.push(classification);
                return ConvexSolidMeshClassification {
                    relation: ConvexSolidMeshRelation::NotCertifiedConvex,
                    solid_facts: facts,
                    vertices,
                };
            }
        }
        vertices.push(classification);
    }

    let relation = match (inside, boundary, outside) {
        (_, 0, 0) if inside == subject.vertices().len() => ConvexSolidMeshRelation::StrictlyInside,
        (0, 0, _) => ConvexSolidMeshRelation::Outside,
        _ => ConvexSolidMeshRelation::BoundaryOrMixed,
    };
    ConvexSolidMeshClassification {
        relation,
        solid_facts: facts,
        vertices,
    }
}

fn classify_point_with_convex_facts_report(
    point: &Point3,
    solid: &ExactMesh,
    facts: &ConvexSolidFacts,
) -> ConvexSolidPointClassification {
    if !facts.is_certified_convex() {
        return ConvexSolidPointClassification {
            relation: ConvexSolidPointRelation::NotCertifiedConvex,
            predicates: Vec::new(),
        };
    }

    let mut touches_boundary = false;
    let mut predicates = Vec::with_capacity(solid.triangles().len());
    for triangle in solid.triangles() {
        let tri = triangle.0;
        let a = solid.vertices()[tri[0]].clone();
        let b = solid.vertices()[tri[1]].clone();
        let c = solid.vertices()[tri[2]].clone();
        let report = orient3d_report(&a, &b, &c, point);
        predicates.push(PredicateUse::from_certificate(report.certificate));
        let Some(side) = report.value().map(PlaneSide::from) else {
            return ConvexSolidPointClassification {
                relation: ConvexSolidPointRelation::Unknown,
                predicates,
            };
        };
        if side_is_outside(facts.orientation, side) {
            return ConvexSolidPointClassification {
                relation: ConvexSolidPointRelation::Outside,
                predicates,
            };
        }
        touches_boundary |= side == PlaneSide::On;
    }

    let relation = if touches_boundary {
        ConvexSolidPointRelation::Boundary
    } else {
        ConvexSolidPointRelation::Inside
    };
    ConvexSolidPointClassification {
        relation,
        predicates,
    }
}

pub(crate) fn exact_mesh_orientation(mesh: &ExactMesh) -> ClosedMeshOrientation {
    let signed_volume = mesh
        .triangles()
        .iter()
        .map(|triangle| {
            let tri = triangle.0;
            determinant_from_origin(
                &mesh.vertices()[tri[0]].clone(),
                &mesh.vertices()[tri[1]].clone(),
                &mesh.vertices()[tri[2]].clone(),
            )
        })
        .fold(Real::from(0), |sum, det| &sum + &det);

    match compare_reals(&signed_volume, &Real::from(0)).value() {
        Some(Ordering::Greater) => ClosedMeshOrientation::Positive,
        Some(Ordering::Less) => ClosedMeshOrientation::Negative,
        _ => ClosedMeshOrientation::Unknown,
    }
}

fn determinant_from_origin(a: &Point3, b: &Point3, c: &Point3) -> Real {
    let by_cz = &b.y * &c.z;
    let bz_cy = &b.z * &c.y;
    let bx_cz = &b.x * &c.z;
    let bz_cx = &b.z * &c.x;
    let bx_cy = &b.x * &c.y;
    let by_cx = &b.y * &c.x;

    let x_minor = &by_cz - &bz_cy;
    let y_minor = &bx_cz - &bz_cx;
    let z_minor = &bx_cy - &by_cx;

    let x_term = &a.x * &x_minor;
    let y_term = &a.y * &y_minor;
    let z_term = &a.z * &z_minor;

    &(&x_term - &y_term) + &z_term
}

fn side_is_outside(orientation: ClosedMeshOrientation, side: PlaneSide) -> bool {
    matches!(
        (orientation, side),
        (ClosedMeshOrientation::Positive, PlaneSide::Below)
            | (ClosedMeshOrientation::Negative, PlaneSide::Above)
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn certified_facts() -> ConvexSolidFacts {
        ConvexSolidFacts {
            orientation: ClosedMeshOrientation::Positive,
            convexity: ConvexSolidClassification::Convex,
            predicates: Vec::new(),
        }
    }

    fn point(relation: ConvexSolidPointRelation) -> ConvexSolidPointClassification {
        ConvexSolidPointClassification {
            relation,
            predicates: Vec::new(),
        }
    }

    #[test]
    fn convex_mesh_report_accepts_unknown_prefix_evidence() {
        let inside = point(ConvexSolidPointRelation::Inside);
        let unknown = point(ConvexSolidPointRelation::Unknown);
        let outside = point(ConvexSolidPointRelation::Outside);

        let report = ConvexSolidMeshClassification {
            relation: ConvexSolidMeshRelation::Unknown,
            solid_facts: certified_facts(),
            vertices: vec![inside.clone(), unknown.clone()],
        };
        assert!(report.validate().is_ok());

        let stale_prefix = ConvexSolidMeshClassification {
            relation: ConvexSolidMeshRelation::Unknown,
            solid_facts: certified_facts(),
            vertices: vec![inside],
        };
        assert_eq!(
            stale_prefix.validate(),
            Err(ConvexSolidReportError::MeshRelationMismatch)
        );

        let stale_suffix = ConvexSolidMeshClassification {
            relation: ConvexSolidMeshRelation::Unknown,
            solid_facts: certified_facts(),
            vertices: vec![unknown, outside],
        };
        assert_eq!(
            stale_suffix.validate(),
            Err(ConvexSolidReportError::MeshRelationMismatch)
        );
    }
}
