//! Exact winding classification for triangulated volumetric split regions.
//!
//! The construction graph already supplies exact split face regions. This
//! module adds the named-boolean semantic layer for closed volumetric meshes:
//! choose an exact interior representative point from each triangulated source
//! cell, classify that point by the closed opposite mesh's exact ray-parity
//! winding report, and retain the classification beside the output assembly.
//! The classifier tries the centroid first, then deterministic exact
//! barycentric interior witnesses if that centroid lies on the opposite mesh
//! boundary. Output orientation is changed only from replayable exact
//! evidence, never from a primitive-float sample or tolerance nudge.

use hyperlimit::Point3;

use super::graph::MeshSide;
use super::mesh::ExactMesh;
use super::region::{FaceRegionTriangulation, boundary_node_point};
use super::winding::{
    ClosedMeshWindingRelation, PointMeshWindingReport, WindingReportError,
    classify_point_against_closed_mesh_winding_report,
};
use super::witness::{
    EXACT_TRIANGLE_INTERIOR_WITNESSES, ExactTriangleInteriorWitness,
    ExactTriangleInteriorWitnessError,
};

/// Exact relation between one triangulated split cell and the opposite closed
/// mesh.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactVolumetricRegionRelation {
    /// The opposite mesh was not a closed two-manifold.
    NotClosed,
    /// The region representative was certified strictly inside the opposite
    /// mesh.
    Inside,
    /// The region representative was certified outside the opposite mesh.
    Outside,
    /// The representative lies on the opposite boundary.
    Boundary,
    /// Exact ray parity could not decide the representative.
    Unknown,
}

impl ExactVolumetricRegionRelation {
    /// Return whether this relation can directly drive named volumetric
    /// union/intersection/difference assembly.
    pub const fn is_strictly_decided(self) -> bool {
        matches!(self, Self::Inside | Self::Outside)
    }

    /// Return whether this relation is decided enough to drive a conservative
    /// coplanar-boundary materialization policy.
    ///
    /// Boundary classifications are not strict winding facts, but they are
    /// exact outcomes: every deterministic interior witness for the retained
    /// source cell replayed to the opposite closed mesh boundary. The boolean
    /// pipeline may consume that state only through an explicit topology
    /// policy; it must not relabel it as inside/outside.
    pub const fn is_materialization_decided(self) -> bool {
        matches!(self, Self::Inside | Self::Outside | Self::Boundary)
    }
}

/// Retained winding evidence for one triangulated split cell.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactVolumetricRegionClassification {
    /// Mesh side owning the split source face.
    pub region_side: MeshSide,
    /// Source face index on [`Self::region_side`].
    pub region_face: usize,
    /// Triangle indices into the retained [`FaceRegionTriangulation`] that
    /// produced [`Self::representative`].
    ///
    /// A single source face can be divided by several exact intersection
    /// segments. Retaining the local triangle handles makes the winding
    /// decision a per-cell certificate instead of a face-wide approximation,
    pub triangle: [usize; 3],
    /// Exact interior representative point used for winding parity.
    ///
    /// The classifier records the first deterministic barycentric witness that
    /// gives a strict inside/outside relation, falling back to the first
    /// boundary/unknown witness only when every candidate remains non-strict.
    /// This avoids treating an unlucky centroid-on-boundary event as a
    /// semantic blocker while keeping the chosen point replayable.
    pub representative: Point3,
    /// Exact barycentric witness that produced [`Self::representative`].
    ///
    /// Retaining the integer weights keeps the representative tied to the
    /// source triangle rather than to an opaque coordinate. Replaying those
    /// itself auditable.
    pub representative_witness: ExactTriangleInteriorWitness,
    /// Relation derived from [`Self::winding`].
    pub relation: ExactVolumetricRegionRelation,
    /// Exact closed-mesh ray-parity report for [`Self::representative`].
    pub winding: PointMeshWindingReport,
    /// Ordered exact witness attempts that justify the retained
    /// representative.
    ///
    /// Strict inside/outside classifications retain the centroid and every
    /// retry up to the first strict witness. Boundary/unknown classifications
    /// retain the full deterministic witness lattice, proving that no hidden
    /// perturbation was used after all exact candidates remained non-strict.
    /// carries replayable failed exact attempts instead of only a status bit.
    pub witness_attempts: Vec<ExactVolumetricWitnessAttempt>,
}

/// One exact barycentric representative tried for a volumetric cell.
///
/// The attempt retains both the exact barycentric witness and the winding
/// report produced by the materialized point. Keeping the unsuccessful
/// boundary/unknown attempts is important evidence: when the final
/// classification is non-strict, the caller can distinguish "one sample was
/// boundary" from "the deterministic exact witness lattice was exhausted."
#[derive(Clone, Debug, PartialEq)]
pub struct ExactVolumetricWitnessAttempt {
    /// Exact barycentric witness used for this attempt.
    pub witness: ExactTriangleInteriorWitness,
    /// Exact representative point materialized from [`Self::witness`].
    pub representative: Point3,
    /// Relation derived from [`Self::winding`].
    pub relation: ExactVolumetricRegionRelation,
    /// Exact closed-mesh ray-parity report for [`Self::representative`].
    pub winding: PointMeshWindingReport,
}

impl ExactVolumetricRegionClassification {
    /// Validate local consistency between retained representative, relation,
    /// and winding report.
    ///
    /// The representative is not a free coordinate: source replay recomputes it
    /// from a retained triangulation. This local audit checks the part that can
    /// be verified without the source mesh, namely that the relation mirrors
    /// the retained exact winding report and that the winding report is itself
    /// local certificate shape and source-object replay are both explicit.
    pub fn validate(&self) -> Result<(), ExactVolumetricRegionError> {
        self.representative_witness
            .validate()
            .map_err(ExactVolumetricRegionError::InvalidRepresentativeWitness)?;
        self.winding
            .validate()
            .map_err(ExactVolumetricRegionError::Winding)?;
        if self.relation != relation_from_winding(self.winding.relation) {
            return Err(ExactVolumetricRegionError::RelationMismatch);
        }
        validate_witness_attempts(self)?;
        Ok(())
    }

    /// Validate retained representative points against the retained
    /// triangulation cell without replaying the opposite source mesh.
    pub fn validate_representatives_against_triangulation(
        &self,
        triangulation: &FaceRegionTriangulation,
    ) -> Result<(), ExactVolumetricRegionError> {
        self.validate()?;
        if triangulation.side != self.region_side || triangulation.face != self.region_face {
            return Err(ExactVolumetricRegionError::InvalidTriangulation);
        }
        triangulation
            .validate()
            .map_err(|_| ExactVolumetricRegionError::InvalidTriangulation)?;
        if !triangulation
            .triangles
            .chunks_exact(3)
            .any(|candidate| candidate == self.triangle)
        {
            return Err(ExactVolumetricRegionError::InvalidTriangleIndex);
        }
        let (a, b, c) = triangle_points(triangulation, self.triangle)?;
        for attempt in &self.witness_attempts {
            let expected = attempt
                .witness
                .point_for_triangle(a, b, c)
                .map_err(ExactVolumetricRegionError::InvalidRepresentativeWitness)?;
            if expected != attempt.representative {
                return Err(ExactVolumetricRegionError::RepresentativeAttemptMismatch);
            }
        }
        Ok(())
    }

    /// Validate this classification by recomputing it from the retained
    /// triangulation cell and target mesh.
    pub fn validate_against_sources(
        &self,
        triangulation: &FaceRegionTriangulation,
        target: &ExactMesh,
    ) -> Result<(), ExactVolumetricRegionError> {
        self.validate_representatives_against_triangulation(triangulation)?;
        let replay = classify_triangulated_region_triangle_against_closed_mesh(
            triangulation,
            self.triangle,
            target,
        )?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactVolumetricRegionError::SourceReplayMismatch)
        }
    }

    /// Classify whether this retained volumetric region classification is fresh.
    pub fn freshness_against_sources(
        &self,
        triangulation: &FaceRegionTriangulation,
        target: &ExactMesh,
    ) -> ExactVolumetricRegionFreshness {
        match self.validate_against_sources(triangulation, target) {
            Ok(()) => ExactVolumetricRegionFreshness::Current,
            Err(error) => error.into(),
        }
    }
}

/// Validation or source-replay failure for volumetric region classifications.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactVolumetricRegionError {
    /// The retained triangulation did not pass its exact handoff audit.
    InvalidTriangulation,
    /// The triangulation produced no output triangle from which to choose an
    /// exact representative.
    EmptyTriangulation,
    /// The chosen triangulation triangle referenced a missing boundary node.
    InvalidTriangleIndex,
    /// The retained exact barycentric witness was not a strict interior point.
    InvalidRepresentativeWitness(ExactTriangleInteriorWitnessError),
    /// A classification did not retain the witness attempt that produced it.
    MissingRepresentativeAttempt,
    /// Retained witness attempts did not follow the deterministic lattice.
    RepresentativeAttemptOrderMismatch,
    /// A retained witness attempt did not match its winding report.
    RepresentativeAttemptRelationMismatch,
    /// The retained representative fields did not match the chosen attempt.
    RepresentativeAttemptMismatch,
    /// A non-strict classification did not retain the full exhausted lattice.
    RepresentativeAttemptNotExhausted,
    /// A retained strict attempt appeared before the chosen representative.
    RepresentativeAttemptSkippedStrict,
    /// A retained winding report failed its local audit.
    Winding(WindingReportError),
    /// The retained relation did not match the retained winding report.
    RelationMismatch,
    /// Recomputed representative or winding evidence did not match.
    SourceReplayMismatch,
}

/// Freshness status for retained volumetric winding region classifications.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactVolumetricRegionFreshness {
    /// The classification is locally valid and replays from source evidence.
    Current,
    /// Retained triangulation references or topology are invalid.
    InvalidTriangulation,
    /// Retained representative witness evidence is missing, out of order, or stale.
    InvalidRepresentativeEvidence,
    /// Retained winding evidence is internally inconsistent.
    InvalidWindingEvidence,
    /// Retained relation does not match retained winding evidence.
    StaleRelationEvidence,
    /// The classification is locally valid but no longer replays from sources.
    SourceReplayMismatch,
}

impl From<ExactVolumetricRegionError> for ExactVolumetricRegionFreshness {
    fn from(error: ExactVolumetricRegionError) -> Self {
        match error {
            ExactVolumetricRegionError::InvalidTriangulation
            | ExactVolumetricRegionError::EmptyTriangulation
            | ExactVolumetricRegionError::InvalidTriangleIndex => Self::InvalidTriangulation,
            ExactVolumetricRegionError::InvalidRepresentativeWitness(_)
            | ExactVolumetricRegionError::MissingRepresentativeAttempt
            | ExactVolumetricRegionError::RepresentativeAttemptOrderMismatch
            | ExactVolumetricRegionError::RepresentativeAttemptRelationMismatch
            | ExactVolumetricRegionError::RepresentativeAttemptMismatch
            | ExactVolumetricRegionError::RepresentativeAttemptNotExhausted
            | ExactVolumetricRegionError::RepresentativeAttemptSkippedStrict => {
                Self::InvalidRepresentativeEvidence
            }
            ExactVolumetricRegionError::Winding(_) => Self::InvalidWindingEvidence,
            ExactVolumetricRegionError::RelationMismatch => Self::StaleRelationEvidence,
            ExactVolumetricRegionError::SourceReplayMismatch => Self::SourceReplayMismatch,
        }
    }
}

/// Classify one exact triangulated source-face cell against a closed mesh.
///
/// The representative search starts with the exact centroid of the supplied
/// local triangle, then tries fixed exact barycentric interior witnesses if the
/// centroid is on the opposite boundary or otherwise undecided. Per-cell
/// representatives are the necessary semantic unit once a source face has been
/// subdivided by constrained intersection segments; using one sample for the
/// entire face would make inside/outside topology depend on an arbitrary
/// triangulator order. Retrying with retained exact interior witnesses follows
/// representational accident such as "centroid is on the boundary" should not
/// force an approximate perturbation.
pub(crate) fn classify_triangulated_region_triangle_against_closed_mesh(
    triangulation: &FaceRegionTriangulation,
    triangle: [usize; 3],
    target: &ExactMesh,
) -> Result<ExactVolumetricRegionClassification, ExactVolumetricRegionError> {
    triangulation
        .validate()
        .map_err(|_| ExactVolumetricRegionError::InvalidTriangulation)?;
    if !triangulation
        .triangles
        .chunks_exact(3)
        .any(|candidate| candidate == triangle)
    {
        return Err(ExactVolumetricRegionError::InvalidTriangleIndex);
    }
    let (a, b, c) = triangle_points(triangulation, triangle)?;
    let mut attempts = Vec::new();
    for witness in EXACT_TRIANGLE_INTERIOR_WITNESSES.iter().copied() {
        let representative = witness
            .point_for_triangle(a, b, c)
            .map_err(ExactVolumetricRegionError::InvalidRepresentativeWitness)?;
        let attempt = classify_witness_attempt(witness, representative, target)?;
        attempts.push(attempt);
        if attempts
            .last()
            .is_some_and(|attempt| attempt.relation.is_strictly_decided())
        {
            return classification_from_attempts(triangulation, triangle, attempts);
        }
    }
    classification_from_attempts(triangulation, triangle, attempts)
}

/// Classify every split-region triangle against its opposite closed mesh.
pub(crate) fn classify_triangulated_regions_against_opposite_meshes(
    triangulations: &[FaceRegionTriangulation],
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Vec<ExactVolumetricRegionClassification>, ExactVolumetricRegionError> {
    let mut classifications = Vec::new();
    for triangulation in triangulations {
        let target = match triangulation.side {
            MeshSide::Left => right,
            MeshSide::Right => left,
        };
        for triangle in triangulation.triangles.chunks_exact(3) {
            classifications.push(classify_triangulated_region_triangle_against_closed_mesh(
                triangulation,
                [triangle[0], triangle[1], triangle[2]],
                target,
            )?);
        }
    }
    Ok(classifications)
}

fn relation_from_winding(relation: ClosedMeshWindingRelation) -> ExactVolumetricRegionRelation {
    match relation {
        ClosedMeshWindingRelation::NotClosed => ExactVolumetricRegionRelation::NotClosed,
        ClosedMeshWindingRelation::Inside => ExactVolumetricRegionRelation::Inside,
        ClosedMeshWindingRelation::Outside => ExactVolumetricRegionRelation::Outside,
        ClosedMeshWindingRelation::Boundary => ExactVolumetricRegionRelation::Boundary,
        ClosedMeshWindingRelation::Unknown => ExactVolumetricRegionRelation::Unknown,
    }
}

fn triangle_points(
    triangulation: &FaceRegionTriangulation,
    triangle: [usize; 3],
) -> Result<(&Point3, &Point3, &Point3), ExactVolumetricRegionError> {
    let a = boundary_node_point(
        triangulation
            .boundary
            .get(triangle[0])
            .ok_or(ExactVolumetricRegionError::InvalidTriangleIndex)?,
    );
    let b = boundary_node_point(
        triangulation
            .boundary
            .get(triangle[1])
            .ok_or(ExactVolumetricRegionError::InvalidTriangleIndex)?,
    );
    let c = boundary_node_point(
        triangulation
            .boundary
            .get(triangle[2])
            .ok_or(ExactVolumetricRegionError::InvalidTriangleIndex)?,
    );
    Ok((a, b, c))
}

fn classify_representative(
    triangulation: &FaceRegionTriangulation,
    triangle: [usize; 3],
    attempt: &ExactVolumetricWitnessAttempt,
    attempts: Vec<ExactVolumetricWitnessAttempt>,
) -> Result<ExactVolumetricRegionClassification, ExactVolumetricRegionError> {
    Ok(ExactVolumetricRegionClassification {
        region_side: triangulation.side,
        region_face: triangulation.face,
        triangle,
        representative: attempt.representative.clone(),
        representative_witness: attempt.witness,
        relation: attempt.relation,
        winding: attempt.winding.clone(),
        witness_attempts: attempts,
    })
}

fn classify_witness_attempt(
    witness: ExactTriangleInteriorWitness,
    representative: Point3,
    target: &ExactMesh,
) -> Result<ExactVolumetricWitnessAttempt, ExactVolumetricRegionError> {
    let winding = classify_point_against_closed_mesh_winding_report(&representative, target);
    winding
        .validate()
        .map_err(ExactVolumetricRegionError::Winding)?;
    Ok(ExactVolumetricWitnessAttempt {
        witness,
        representative,
        relation: relation_from_winding(winding.relation),
        winding,
    })
}

fn classification_from_attempts(
    triangulation: &FaceRegionTriangulation,
    triangle: [usize; 3],
    attempts: Vec<ExactVolumetricWitnessAttempt>,
) -> Result<ExactVolumetricRegionClassification, ExactVolumetricRegionError> {
    if attempts.is_empty() {
        return Err(ExactVolumetricRegionError::EmptyTriangulation);
    }
    let chosen = attempts
        .iter()
        .position(|attempt| attempt.relation.is_strictly_decided())
        .or_else(|| {
            attempts
                .iter()
                .position(|attempt| attempt.relation == ExactVolumetricRegionRelation::Boundary)
        })
        .unwrap_or(0);
    let attempt = attempts[chosen].clone();
    classify_representative(triangulation, triangle, &attempt, attempts)
}

fn validate_witness_attempts(
    classification: &ExactVolumetricRegionClassification,
) -> Result<(), ExactVolumetricRegionError> {
    if classification.witness_attempts.is_empty() {
        return Err(ExactVolumetricRegionError::MissingRepresentativeAttempt);
    }
    if classification.witness_attempts.len() > EXACT_TRIANGLE_INTERIOR_WITNESSES.len() {
        return Err(ExactVolumetricRegionError::RepresentativeAttemptOrderMismatch);
    }

    for (index, attempt) in classification.witness_attempts.iter().enumerate() {
        let expected = EXACT_TRIANGLE_INTERIOR_WITNESSES[index];
        if attempt.witness != expected {
            return Err(ExactVolumetricRegionError::RepresentativeAttemptOrderMismatch);
        }
        attempt
            .witness
            .validate()
            .map_err(ExactVolumetricRegionError::InvalidRepresentativeWitness)?;
        attempt
            .winding
            .validate()
            .map_err(ExactVolumetricRegionError::Winding)?;
        if attempt.relation != relation_from_winding(attempt.winding.relation) {
            return Err(ExactVolumetricRegionError::RepresentativeAttemptRelationMismatch);
        }
    }

    let chosen = classification
        .witness_attempts
        .iter()
        .position(|attempt| {
            attempt.witness == classification.representative_witness
                && attempt.representative == classification.representative
                && attempt.relation == classification.relation
                && attempt.winding == classification.winding
        })
        .ok_or(ExactVolumetricRegionError::RepresentativeAttemptMismatch)?;

    if classification.relation.is_strictly_decided() {
        if classification.witness_attempts[..chosen]
            .iter()
            .any(|attempt| attempt.relation.is_strictly_decided())
        {
            return Err(ExactVolumetricRegionError::RepresentativeAttemptSkippedStrict);
        }
        if chosen + 1 != classification.witness_attempts.len() {
            return Err(ExactVolumetricRegionError::RepresentativeAttemptSkippedStrict);
        }
        return Ok(());
    }

    if classification
        .witness_attempts
        .iter()
        .any(|attempt| attempt.relation.is_strictly_decided())
    {
        return Err(ExactVolumetricRegionError::RepresentativeAttemptSkippedStrict);
    }
    if classification.witness_attempts.len() != EXACT_TRIANGLE_INTERIOR_WITNESSES.len() {
        return Err(ExactVolumetricRegionError::RepresentativeAttemptNotExhausted);
    }
    if classification.relation == ExactVolumetricRegionRelation::Boundary {
        let first_boundary = classification
            .witness_attempts
            .iter()
            .position(|attempt| attempt.relation == ExactVolumetricRegionRelation::Boundary)
            .ok_or(ExactVolumetricRegionError::RepresentativeAttemptMismatch)?;
        if chosen != first_boundary {
            return Err(ExactVolumetricRegionError::RepresentativeAttemptMismatch);
        }
    } else if chosen != 0 {
        return Err(ExactVolumetricRegionError::RepresentativeAttemptMismatch);
    }
    Ok(())
}
