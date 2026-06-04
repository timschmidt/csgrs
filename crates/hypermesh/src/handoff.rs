//! Exact downstream handoff reports.
//!
//! Hypermesh owns mesh topology and boolean-construction evidence, while
//! `hypervoxel` and `hyperphysics` consume that evidence for voxelization,
//! broad-phase grids, mass properties, and simulation fixtures. This module
//! keeps that boundary explicit: a mesh is not a solid handoff merely because
//! it has triangles; it must replay retained state, certify closed manifold
//! topology, retain exact rational coordinates, and expose exact face-plane
//! views, and certified predicate decisions are distinct artifacts.

use super::{ExactMesh, ExactMeshAuditReport, MeshSource, ValidationPolicy, audit_exact_mesh};

/// Exact solid handoff report for downstream crates.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactSolidHandoffReport {
    /// Whole-mesh retained-state audit that was replayed before handoff.
    pub audit: ExactMeshAuditReport,
    /// Number of retained exact face-plane equations.
    pub retained_face_planes: usize,
    /// Whether mesh-wide exact AABB bounds are retained.
    pub retained_mesh_bounds: bool,
    /// Whether this handoff has nonempty topology evidence.
    pub nonempty_topology: bool,
    /// Whether every retained predicate is proof-producing.
    pub proof_predicate_ready: bool,
}

/// Exact surface handoff report for downstream crates.
///
/// This report is the non-solid counterpart to [`ExactSolidHandoffReport`].
/// It accepts closed shells and boundary-allowed open surfaces, but still
/// requires exact retained mesh state, exact rational coordinates, face-plane
/// object evidence should stay separate from consumer-domain interpretation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactSurfaceHandoffReport {
    /// Whole-mesh retained-state audit that was replayed before handoff.
    pub audit: ExactMeshAuditReport,
    /// Number of retained exact face-plane equations.
    pub retained_face_planes: usize,
    /// Whether mesh-wide exact AABB bounds are retained.
    pub retained_mesh_bounds: bool,
    /// Whether this handoff has at least one face and one vertex.
    pub nonempty_topology: bool,
    /// Whether the retained topology is a closed manifold.
    pub closed_manifold: bool,
    /// Validation policy retained at construction.
    pub validation_policy: ValidationPolicy,
    /// Whether every retained predicate is proof-producing.
    pub proof_predicate_ready: bool,
}

/// Error returned when a mesh cannot be handed off as an exact solid.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactSolidHandoffError {
    /// The retained mesh audit failed.
    Audit(super::ExactMeshValidationError),
    /// The mesh has no vertices or no faces.
    EmptyTopology,
    /// Retained facts do not certify a closed two-manifold.
    NotClosedManifold,
    /// Coordinates are not all exact rational values.
    NonExactRationalCoordinates,
    /// Retained face-plane facts do not cover every face.
    FacePlaneCountMismatch {
        /// Face count derived from the mesh.
        expected: usize,
        /// Retained face-plane count.
        actual: usize,
    },
    /// Mesh-wide exact bounds are missing.
    MissingMeshBounds,
    /// A downstream handoff report no longer matches the source mesh.
    ReportMismatch {
        /// Name of the mismatched field.
        field: &'static str,
    },
}

/// Error returned when a mesh cannot be handed off as an exact surface.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactSurfaceHandoffError {
    /// The retained mesh audit failed.
    Audit(super::ExactMeshValidationError),
    /// The mesh has no vertices or no faces.
    EmptyTopology,
    /// Coordinates are not all exact rational values.
    NonExactRationalCoordinates,
    /// Retained face-plane facts do not cover every face.
    FacePlaneCountMismatch {
        /// Face count derived from the mesh.
        expected: usize,
        /// Retained face-plane count.
        actual: usize,
    },
    /// Mesh-wide exact bounds are missing.
    MissingMeshBounds,
    /// A downstream handoff report no longer matches the source mesh.
    ReportMismatch {
        /// Name of the mismatched field.
        field: &'static str,
    },
}

/// Freshness/readiness status for a retained exact solid handoff.
///
/// This is a downstream scheduling diagnostic, not a new proof. `Current`
/// means the handoff report still replays as exact closed-solid evidence;
/// other variants tell a caller whether to recompute, reject as non-solid, or
/// object facts and consumer-domain readiness separate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactSolidHandoffFreshness {
    /// The handoff report replays exactly against the current mesh.
    Current,
    /// The retained mesh audit failed before solid readiness could be checked.
    InvalidMeshState,
    /// The mesh is a valid exact artifact but not closed-solid evidence.
    NotSolidReady,
    /// Retained face-plane evidence no longer covers every face.
    StaleFacePlanes,
    /// Retained mesh bounds are missing.
    MissingBounds,
    /// The retained handoff report differs from replayed source evidence.
    StaleReport,
}

/// Freshness/readiness status for a retained exact surface handoff.
///
/// `Current` means the report still replays as exact surface evidence. It is
/// not a closed-solid certificate unless [`ExactSurfaceHandoffReport::closed_manifold`]
/// is also true and the caller's domain accepts closed shells. Keeping this
/// status separate from [`ExactSolidHandoffFreshness`] prevents open-surface
/// display, slicing, and surface-boolean outputs from being mislabeled as
/// volume evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactSurfaceHandoffFreshness {
    /// The handoff report replays exactly against the current mesh.
    Current,
    /// The retained mesh audit failed before surface readiness could be checked.
    InvalidMeshState,
    /// The mesh is a valid exact artifact but lacks surface topology evidence.
    NotSurfaceReady,
    /// Retained face-plane evidence no longer covers every face.
    StaleFacePlanes,
    /// Retained mesh bounds are missing.
    MissingBounds,
    /// The retained handoff report differs from replayed source evidence.
    StaleReport,
}

impl ExactSolidHandoffReport {
    /// Build an exact solid handoff report after retained-state replay.
    ///
    /// The report is intentionally conservative. Boundary meshes, empty
    /// meshes, and meshes with stale face-plane or bounds facts are valid
    /// hypermesh artifacts under some policies, but they are not accepted as
    /// exact closed-solid evidence for downstream physical or voxel semantics.
    pub fn from_mesh(mesh: &ExactMesh) -> Result<Self, ExactSolidHandoffError> {
        let audit = audit_exact_mesh(mesh).map_err(ExactSolidHandoffError::Audit)?;
        if audit.vertex_count == 0 || audit.face_count == 0 {
            return Err(ExactSolidHandoffError::EmptyTopology);
        }
        if !audit.closed_manifold {
            return Err(ExactSolidHandoffError::NotClosedManifold);
        }
        if audit.validation_policy != super::ValidationPolicy::CLOSED {
            return Err(ExactSolidHandoffError::NotClosedManifold);
        }
        if !audit.fixed_coordinates_exact_rational {
            return Err(ExactSolidHandoffError::NonExactRationalCoordinates);
        }
        let retained_face_planes = mesh.facts().faces.len();
        if retained_face_planes != audit.face_count {
            return Err(ExactSolidHandoffError::FacePlaneCountMismatch {
                expected: audit.face_count,
                actual: retained_face_planes,
            });
        }
        let retained_mesh_bounds = mesh.bounds().mesh.is_some();
        if !retained_mesh_bounds {
            return Err(ExactSolidHandoffError::MissingMeshBounds);
        }
        let proof_predicate_ready = audit.all_predicates_proof_producing();
        Ok(Self {
            audit,
            retained_face_planes,
            retained_mesh_bounds,
            nonempty_topology: true,
            proof_predicate_ready,
        })
    }

    /// Validate that this handoff report still replays against `mesh`.
    pub fn validate_against_mesh(&self, mesh: &ExactMesh) -> Result<(), ExactSolidHandoffError> {
        let replay = Self::from_mesh(mesh)?;
        if self != &replay {
            return Err(ExactSolidHandoffError::ReportMismatch {
                field: "exact_solid_handoff",
            });
        }
        Ok(())
    }

    /// Classify whether this retained handoff is fresh for `mesh`.
    pub fn freshness_against_mesh(&self, mesh: &ExactMesh) -> ExactSolidHandoffFreshness {
        match self.validate_against_mesh(mesh) {
            Ok(()) => ExactSolidHandoffFreshness::Current,
            Err(ExactSolidHandoffError::Audit(_)) => ExactSolidHandoffFreshness::InvalidMeshState,
            Err(ExactSolidHandoffError::EmptyTopology)
            | Err(ExactSolidHandoffError::NotClosedManifold)
            | Err(ExactSolidHandoffError::NonExactRationalCoordinates) => {
                ExactSolidHandoffFreshness::NotSolidReady
            }
            Err(ExactSolidHandoffError::FacePlaneCountMismatch { .. }) => {
                ExactSolidHandoffFreshness::StaleFacePlanes
            }
            Err(ExactSolidHandoffError::MissingMeshBounds) => {
                ExactSolidHandoffFreshness::MissingBounds
            }
            Err(ExactSolidHandoffError::ReportMismatch { .. }) => {
                ExactSolidHandoffFreshness::StaleReport
            }
        }
    }

    /// Return whether this handoff came directly from exact caller data.
    pub const fn source_is_exact(&self) -> bool {
        matches!(self.audit.source, MeshSource::Exact)
    }
}

impl ExactSurfaceHandoffReport {
    /// Build an exact surface handoff report after retained-state replay.
    ///
    /// Unlike [`ExactSolidHandoffReport::from_mesh`], this constructor does
    /// not require closed two-manifold topology. It is intended for exact
    /// open-surface artifacts whose downstream consumer needs surface geometry
    /// rather than volume semantics. The report remains conservative about
    /// retained evidence: missing face planes, missing bounds, non-exact
    /// coordinates, or stale audit state reject the handoff.
    pub fn from_mesh(mesh: &ExactMesh) -> Result<Self, ExactSurfaceHandoffError> {
        let audit = audit_exact_mesh(mesh).map_err(ExactSurfaceHandoffError::Audit)?;
        if audit.vertex_count == 0 || audit.face_count == 0 {
            return Err(ExactSurfaceHandoffError::EmptyTopology);
        }
        if !audit.fixed_coordinates_exact_rational {
            return Err(ExactSurfaceHandoffError::NonExactRationalCoordinates);
        }
        let retained_face_planes = mesh.facts().faces.len();
        if retained_face_planes != audit.face_count {
            return Err(ExactSurfaceHandoffError::FacePlaneCountMismatch {
                expected: audit.face_count,
                actual: retained_face_planes,
            });
        }
        let retained_mesh_bounds = mesh.bounds().mesh.is_some();
        if !retained_mesh_bounds {
            return Err(ExactSurfaceHandoffError::MissingMeshBounds);
        }
        let proof_predicate_ready = audit.all_predicates_proof_producing();
        Ok(Self {
            retained_face_planes,
            retained_mesh_bounds,
            nonempty_topology: true,
            closed_manifold: audit.closed_manifold,
            validation_policy: audit.validation_policy,
            proof_predicate_ready,
            audit,
        })
    }

    /// Validate that this surface handoff report still replays against `mesh`.
    pub fn validate_against_mesh(&self, mesh: &ExactMesh) -> Result<(), ExactSurfaceHandoffError> {
        let replay = Self::from_mesh(mesh)?;
        if self != &replay {
            return Err(ExactSurfaceHandoffError::ReportMismatch {
                field: "exact_surface_handoff",
            });
        }
        Ok(())
    }

    /// Classify whether this retained surface handoff is fresh for `mesh`.
    pub fn freshness_against_mesh(&self, mesh: &ExactMesh) -> ExactSurfaceHandoffFreshness {
        match self.validate_against_mesh(mesh) {
            Ok(()) => ExactSurfaceHandoffFreshness::Current,
            Err(ExactSurfaceHandoffError::Audit(_)) => {
                ExactSurfaceHandoffFreshness::InvalidMeshState
            }
            Err(ExactSurfaceHandoffError::EmptyTopology)
            | Err(ExactSurfaceHandoffError::NonExactRationalCoordinates) => {
                ExactSurfaceHandoffFreshness::NotSurfaceReady
            }
            Err(ExactSurfaceHandoffError::FacePlaneCountMismatch { .. }) => {
                ExactSurfaceHandoffFreshness::StaleFacePlanes
            }
            Err(ExactSurfaceHandoffError::MissingMeshBounds) => {
                ExactSurfaceHandoffFreshness::MissingBounds
            }
            Err(ExactSurfaceHandoffError::ReportMismatch { .. }) => {
                ExactSurfaceHandoffFreshness::StaleReport
            }
        }
    }

    /// Return whether this handoff came directly from exact caller data.
    pub const fn source_is_exact(&self) -> bool {
        matches!(self.audit.source, MeshSource::Exact)
    }

    /// Return whether this surface may be consumed as boundary-bearing data.
    pub const fn boundary_allowed(&self) -> bool {
        matches!(
            self.validation_policy.boundary,
            super::BoundaryPolicy::AllowBoundary
        )
    }
}

/// Build a downstream exact-solid handoff report.
pub fn exact_solid_handoff(
    mesh: &ExactMesh,
) -> Result<ExactSolidHandoffReport, ExactSolidHandoffError> {
    ExactSolidHandoffReport::from_mesh(mesh)
}

/// Build a downstream exact-surface handoff report.
pub fn exact_surface_handoff(
    mesh: &ExactMesh,
) -> Result<ExactSurfaceHandoffReport, ExactSurfaceHandoffError> {
    ExactSurfaceHandoffReport::from_mesh(mesh)
}
