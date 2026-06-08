//! Consumer-readiness summaries for exact mesh artifacts.
//!
//! Downstream crates often need a fast routing decision before requesting a
//! specific artifact: surface geometry, closed-solid evidence, or a lossy
//! preview/export view. [`ExactMeshConsumerReadinessReport`] is that routing
//! record. It is deliberately a summary, not a replacement for the underlying
//! geometric systems should expose object facts, adapter boundaries, and
//! cached readiness separately so approximate or domain-specific consumers
//! cannot silently reinterpret topology evidence.

use super::{
    ExactMesh, ExactMeshAuditReport, ValidationPolicy, approximate_mesh_f64_view, audit_exact_mesh,
    exact_solid_handoff, exact_surface_handoff,
};
use hyperlimit::MeshSource;

/// Compact readiness summary for common exact-mesh consumers.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactMeshConsumerReadinessReport {
    /// Whole-mesh retained-state audit used to derive the readiness summary.
    pub audit: ExactMeshAuditReport,
    /// Whether exact surface handoff currently replays for this mesh.
    pub surface_handoff_ready: bool,
    /// Whether exact closed-solid handoff currently replays for this mesh.
    pub solid_handoff_ready: bool,
    /// Whether a lossy `f64` display/export view currently replays.
    pub approximate_f64_view_ready: bool,
    /// Number of retained exact face-plane equations available for handoff.
    pub retained_face_planes: usize,
    /// Whether mesh-wide exact AABB bounds are retained for handoff.
    pub retained_mesh_bounds: bool,
    /// Whether the artifact has at least one vertex and one face.
    pub nonempty_topology: bool,
    /// Whether retained topology is closed-manifold evidence.
    pub closed_manifold: bool,
    /// Whether the construction policy allows boundary edges.
    pub boundary_allowed: bool,
    /// Whether all retained coordinates are exact rational values.
    pub exact_rational_coordinates: bool,
    /// Whether this artifact came from exact caller input.
    pub exact_source: bool,
}

/// Error returned when a retained consumer-readiness summary is stale.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactMeshConsumerReadinessError {
    /// The source mesh failed retained-state audit.
    Audit(super::ExactMeshValidationError),
    /// A downstream summary no longer matches replayed source evidence.
    ReportMismatch {
        /// Name of the mismatched field.
        field: &'static str,
    },
}

/// Freshness status for a retained consumer-readiness summary.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactMeshConsumerReadinessFreshness {
    /// The readiness report replays exactly against the current mesh.
    Current,
    /// The mesh failed retained-state audit.
    InvalidMeshState,
    /// The retained readiness report differs from replayed source evidence.
    StaleReport,
}

impl ExactMeshConsumerReadinessReport {
    /// Build a consumer-readiness summary after retained-state replay.
    ///
    /// This constructor probes exact surface handoff, exact solid handoff, and
    /// lossy `f64` view construction only as availability checks. Consumers
    /// that need the actual artifact should still request and validate that
    /// artifact directly. The summary prevents downstream code from treating
    /// "mesh exists" as equivalent to "solid exists" or "lossy view exists."
    pub fn from_mesh(mesh: &ExactMesh) -> Result<Self, ExactMeshConsumerReadinessError> {
        let audit = audit_exact_mesh(mesh).map_err(ExactMeshConsumerReadinessError::Audit)?;
        let retained_face_planes = mesh.facts().faces.len();
        let retained_mesh_bounds = mesh.bounds().mesh.is_some();
        Ok(Self {
            surface_handoff_ready: exact_surface_handoff(mesh).is_ok(),
            solid_handoff_ready: exact_solid_handoff(mesh).is_ok(),
            approximate_f64_view_ready: approximate_mesh_f64_view(mesh).is_ok(),
            retained_face_planes,
            retained_mesh_bounds,
            nonempty_topology: audit.vertex_count > 0 && audit.face_count > 0,
            closed_manifold: audit.closed_manifold,
            boundary_allowed: audit.validation_policy == ValidationPolicy::ALLOW_BOUNDARY,
            exact_rational_coordinates: audit.fixed_coordinates_exact_rational,
            exact_source: matches!(audit.source, MeshSource::Exact),
            audit,
        })
    }

    /// Validate that this readiness summary still replays against `mesh`.
    pub fn validate_against_mesh(
        &self,
        mesh: &ExactMesh,
    ) -> Result<(), ExactMeshConsumerReadinessError> {
        self.validate()?;
        let replay = Self::from_mesh(mesh)?;
        if self != &replay {
            return Err(ExactMeshConsumerReadinessError::ReportMismatch {
                field: "exact_mesh_consumer_readiness",
            });
        }
        Ok(())
    }

    /// Validate readiness-internal consistency without access to the source mesh.
    pub fn validate(&self) -> Result<(), ExactMeshConsumerReadinessError> {
        expect_summary_bool(
            "nonempty_topology",
            self.audit.vertex_count > 0 && self.audit.face_count > 0,
            self.nonempty_topology,
        )?;
        expect_summary_usize(
            "retained_face_planes",
            self.audit.face_count,
            self.retained_face_planes,
        )?;
        expect_summary_bool("retained_mesh_bounds", true, self.retained_mesh_bounds)?;
        expect_summary_bool(
            "closed_manifold",
            self.audit.closed_manifold,
            self.closed_manifold,
        )?;
        expect_summary_bool(
            "boundary_allowed",
            self.audit.validation_policy == ValidationPolicy::ALLOW_BOUNDARY,
            self.boundary_allowed,
        )?;
        expect_summary_bool(
            "exact_rational_coordinates",
            self.audit.fixed_coordinates_exact_rational,
            self.exact_rational_coordinates,
        )?;
        expect_summary_bool(
            "exact_source",
            matches!(self.audit.source, MeshSource::Exact),
            self.exact_source,
        )?;
        expect_summary_bool(
            "surface_handoff_ready",
            self.expected_surface_handoff_ready(),
            self.surface_handoff_ready,
        )?;
        expect_summary_bool(
            "solid_handoff_ready",
            self.expected_solid_handoff_ready(),
            self.solid_handoff_ready,
        )?;
        Ok(())
    }

    /// Classify whether this retained readiness summary is fresh for `mesh`.
    pub fn freshness_against_mesh(&self, mesh: &ExactMesh) -> ExactMeshConsumerReadinessFreshness {
        match self.validate_against_mesh(mesh) {
            Ok(()) => ExactMeshConsumerReadinessFreshness::Current,
            Err(ExactMeshConsumerReadinessError::Audit(_)) => {
                ExactMeshConsumerReadinessFreshness::InvalidMeshState
            }
            Err(ExactMeshConsumerReadinessError::ReportMismatch { .. }) => {
                ExactMeshConsumerReadinessFreshness::StaleReport
            }
        }
    }

    fn expected_surface_handoff_ready(&self) -> bool {
        self.nonempty_topology
            && self.exact_rational_coordinates
            && self.retained_face_planes == self.audit.face_count
            && self.retained_mesh_bounds
    }

    fn expected_solid_handoff_ready(&self) -> bool {
        self.expected_surface_handoff_ready()
            && self.closed_manifold
            && self.audit.validation_policy == ValidationPolicy::CLOSED
    }
}

/// Build a common-consumer readiness summary for an exact mesh.
pub fn exact_mesh_consumer_readiness(
    mesh: &ExactMesh,
) -> Result<ExactMeshConsumerReadinessReport, ExactMeshConsumerReadinessError> {
    ExactMeshConsumerReadinessReport::from_mesh(mesh)
}

fn expect_summary_bool(
    field: &'static str,
    expected: bool,
    actual: bool,
) -> Result<(), ExactMeshConsumerReadinessError> {
    if expected == actual {
        Ok(())
    } else {
        Err(ExactMeshConsumerReadinessError::ReportMismatch { field })
    }
}

fn expect_summary_usize(
    field: &'static str,
    expected: usize,
    actual: usize,
) -> Result<(), ExactMeshConsumerReadinessError> {
    if expected == actual {
        Ok(())
    } else {
        Err(ExactMeshConsumerReadinessError::ReportMismatch { field })
    }
}
