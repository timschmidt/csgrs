//! Retained-state audit reports for exact meshes.
//!
//! [`ExactMeshAuditReport`] is a compact replay artifact for downstream crates
//! that need to consume an [`ExactMesh`] without trusting
//! incidental caches. It summarizes the validated object state after
//! [`ExactMesh::validate_retained_state`](super::ExactMesh::validate_retained_state)
//! has replayed bounds, topology facts, and predicate provenance from the exact
//! proof-bearing and cache-bearing boundaries explicit instead of accepting
//! rounded representatives or stale facts as topology.

use super::{ExactMesh, ExactMeshValidationError, MeshSource, ValidationPolicy};

/// Compact validated summary of an exact mesh artifact.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactMeshAuditReport {
    /// Source category declared by the mesh provenance.
    pub source: MeshSource,
    /// Human-readable source label retained by the mesh provenance.
    pub source_label: String,
    /// Construction version retained by the mesh provenance.
    pub construction_version: u64,
    /// Number of exact vertices retained by the mesh.
    pub vertex_count: usize,
    /// Number of triangular faces retained by the mesh.
    pub face_count: usize,
    /// Number of undirected edges retained by the mesh facts.
    pub edge_count: usize,
    /// Number of retained predicate-use records.
    pub predicate_uses: usize,
    /// Number of retained predicate-use records that are proof-producing.
    pub proof_predicates: usize,
    /// Whether retained facts certify a closed two-manifold.
    pub closed_manifold: bool,
    /// Whether every retained vertex has exact-rational coordinates.
    pub fixed_coordinates_exact_rational: bool,
    /// Validation policy used when the exact mesh was constructed.
    pub validation_policy: ValidationPolicy,
}

/// Error returned when a retained audit report no longer matches a mesh.
///
/// The report is intentionally small enough to cross serialization, fuzzing,
/// benchmark, voxelization, and physics handoff boundaries. Revalidating it
/// against the source mesh prevents downstream code from consuming a relabeled
/// or stale audit summary as exact evidence.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactMeshAuditError {
    /// The source mesh failed its whole-object retained-state replay.
    RetainedState(ExactMeshValidationError),
    /// A count stored in the audit report disagrees with the mesh.
    CountMismatch {
        /// Name of the mismatched field.
        field: &'static str,
        /// Count derived from the mesh.
        expected: usize,
        /// Count stored in the report.
        actual: usize,
    },
    /// The retained source category changed.
    SourceMismatch {
        /// Source category derived from the mesh.
        expected: MeshSource,
        /// Source category stored in the report.
        actual: MeshSource,
    },
    /// The retained source label changed.
    SourceLabelMismatch,
    /// The retained construction version changed.
    ConstructionVersionMismatch {
        /// Version derived from the mesh.
        expected: u64,
        /// Version stored in the report.
        actual: u64,
    },
    /// The closed-manifold summary changed.
    ClosedManifoldMismatch {
        /// Value derived from the mesh.
        expected: bool,
        /// Value stored in the report.
        actual: bool,
    },
    /// The exact-rational coordinate summary changed.
    FixedCoordinatesMismatch {
        /// Value derived from the mesh.
        expected: bool,
        /// Value stored in the report.
        actual: bool,
    },
    /// The retained validation policy changed.
    ValidationPolicyMismatch {
        /// Policy derived from the mesh.
        expected: ValidationPolicy,
        /// Policy stored in the report.
        actual: ValidationPolicy,
    },
}

/// Freshness status for a retained exact mesh audit.
///
/// This is advisory cache metadata, not a topology certificate. It gives
/// downstream voxel, physics, export, and boolean staging code a stable way to
/// reject stale retained facts before falling back to recomputation. The split
/// views, and proof-producing predicates should remain separately auditable.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactMeshAuditFreshness {
    /// The audit replays exactly against the current mesh.
    Current,
    /// The mesh itself failed retained-state validation.
    InvalidMeshState,
    /// Counts or proof-producing predicate totals changed.
    StaleCounts,
    /// Source category or source label changed.
    StaleSource,
    /// Construction version changed.
    StaleConstructionVersion,
    /// Closed-manifold or exact-coordinate summary changed.
    StaleSummaryFacts,
    /// Validation policy changed.
    StaleValidationPolicy,
}

impl ExactMeshAuditReport {
    /// Build an audit report after replaying every retained exact mesh fact.
    pub fn from_mesh(mesh: &ExactMesh) -> Result<Self, ExactMeshValidationError> {
        mesh.validate_retained_state()?;
        let predicate_uses = mesh.provenance().predicates.len();
        let proof_predicates = mesh
            .provenance()
            .predicates
            .iter()
            .filter(|predicate| predicate.is_proof_producing())
            .count();
        Ok(Self {
            source: mesh.provenance().source.source,
            source_label: mesh.provenance().source.label.clone(),
            construction_version: mesh.provenance().construction_version,
            vertex_count: mesh.vertices().len(),
            face_count: mesh.triangles().len(),
            edge_count: mesh.facts().mesh.edge_count,
            predicate_uses,
            proof_predicates,
            closed_manifold: mesh.facts().mesh.closed_manifold,
            fixed_coordinates_exact_rational: mesh.facts().mesh.fixed_coordinates_exact_rational,
            validation_policy: mesh.validation_policy(),
        })
    }

    /// Validate that this report still replays against `mesh`.
    pub fn validate_against_mesh(&self, mesh: &ExactMesh) -> Result<(), ExactMeshAuditError> {
        mesh.validate_retained_state()
            .map_err(ExactMeshAuditError::RetainedState)?;
        expect_count("vertex_count", mesh.vertices().len(), self.vertex_count)?;
        expect_count("face_count", mesh.triangles().len(), self.face_count)?;
        expect_count("edge_count", mesh.facts().mesh.edge_count, self.edge_count)?;
        expect_count(
            "predicate_uses",
            mesh.provenance().predicates.len(),
            self.predicate_uses,
        )?;
        let proof_predicates = mesh
            .provenance()
            .predicates
            .iter()
            .filter(|predicate| predicate.is_proof_producing())
            .count();
        expect_count("proof_predicates", proof_predicates, self.proof_predicates)?;
        if self.source != mesh.provenance().source.source {
            return Err(ExactMeshAuditError::SourceMismatch {
                expected: mesh.provenance().source.source,
                actual: self.source,
            });
        }
        if self.source_label != mesh.provenance().source.label {
            return Err(ExactMeshAuditError::SourceLabelMismatch);
        }
        if self.construction_version != mesh.provenance().construction_version {
            return Err(ExactMeshAuditError::ConstructionVersionMismatch {
                expected: mesh.provenance().construction_version,
                actual: self.construction_version,
            });
        }
        if self.closed_manifold != mesh.facts().mesh.closed_manifold {
            return Err(ExactMeshAuditError::ClosedManifoldMismatch {
                expected: mesh.facts().mesh.closed_manifold,
                actual: self.closed_manifold,
            });
        }
        if self.fixed_coordinates_exact_rational
            != mesh.facts().mesh.fixed_coordinates_exact_rational
        {
            return Err(ExactMeshAuditError::FixedCoordinatesMismatch {
                expected: mesh.facts().mesh.fixed_coordinates_exact_rational,
                actual: self.fixed_coordinates_exact_rational,
            });
        }
        if self.validation_policy != mesh.validation_policy() {
            return Err(ExactMeshAuditError::ValidationPolicyMismatch {
                expected: mesh.validation_policy(),
                actual: self.validation_policy,
            });
        }
        Ok(())
    }

    /// Classify whether this retained audit is fresh for `mesh`.
    ///
    /// Callers that only need a reuse/recompute decision can use this compact
    /// status instead of matching every validation error. A `Current` result
    /// still means "the audit is fresh"; topology decisions remain tied to the
    /// retained exact mesh and its predicate evidence.
    pub fn freshness_against_mesh(&self, mesh: &ExactMesh) -> ExactMeshAuditFreshness {
        match self.validate_against_mesh(mesh) {
            Ok(()) => ExactMeshAuditFreshness::Current,
            Err(ExactMeshAuditError::RetainedState(_)) => ExactMeshAuditFreshness::InvalidMeshState,
            Err(ExactMeshAuditError::CountMismatch { .. }) => ExactMeshAuditFreshness::StaleCounts,
            Err(ExactMeshAuditError::SourceMismatch { .. })
            | Err(ExactMeshAuditError::SourceLabelMismatch) => ExactMeshAuditFreshness::StaleSource,
            Err(ExactMeshAuditError::ConstructionVersionMismatch { .. }) => {
                ExactMeshAuditFreshness::StaleConstructionVersion
            }
            Err(ExactMeshAuditError::ClosedManifoldMismatch { .. })
            | Err(ExactMeshAuditError::FixedCoordinatesMismatch { .. }) => {
                ExactMeshAuditFreshness::StaleSummaryFacts
            }
            Err(ExactMeshAuditError::ValidationPolicyMismatch { .. }) => {
                ExactMeshAuditFreshness::StaleValidationPolicy
            }
        }
    }

    /// Return whether every retained predicate listed in the audit was
    /// proof-producing.
    pub const fn all_predicates_proof_producing(&self) -> bool {
        self.predicate_uses == self.proof_predicates
    }
}

/// Build a retained-state audit report for an exact mesh.
pub fn audit_exact_mesh(
    mesh: &ExactMesh,
) -> Result<ExactMeshAuditReport, ExactMeshValidationError> {
    ExactMeshAuditReport::from_mesh(mesh)
}

fn expect_count(
    field: &'static str,
    expected: usize,
    actual: usize,
) -> Result<(), ExactMeshAuditError> {
    if expected == actual {
        Ok(())
    } else {
        Err(ExactMeshAuditError::CountMismatch {
            field,
            expected,
            actual,
        })
    }
}
