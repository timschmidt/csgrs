//! Exact mesh construction and storage.
//!
//! `ExactMesh` stores coordinates as `hyperlimit::Point3` over
//! `hyperreal::Real`. Primitive-float construction is a named lossy adapter
//! and validates every coordinate before import.

use super::adapter::{
    ExactI64MeshInputReport, LossyF64MeshInputReport, inspect_f64_mesh_input,
    inspect_i64_mesh_input,
};
use super::audit::{ExactMeshAuditReport, audit_exact_mesh};
use super::boolean::{
    ExactBooleanOperation, ExactBooleanRequest, materialize_boolean_exact_request,
};
use super::bounds::{BoundsValidationError, MeshBounds};
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::facts::{MeshFactsValidationError, MeshValidationFacts};
use super::scalar::LossyF64Import;
use super::validation::{ValidationPolicy, ValidationReport, validate_triangles_with_policy};
use super::view::ExactMeshRef;
use hyperlimit::{
    ConstructionProvenance, ConstructionProvenanceValidationError, Point3, PredicateUse,
    SourceProvenance,
};
use hyperreal::Real;

/// Triangle index triplet.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Triangle(pub [usize; 3]);

/// Exact triangular mesh with retained validation facts.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactMesh {
    vertices: Vec<Point3>,
    triangles: Vec<Triangle>,
    bounds: MeshBounds,
    facts: MeshValidationFacts,
    validation_policy: ValidationPolicy,
    provenance: ConstructionProvenance,
}

fn point_from_f64_lossy(values: [f64; 3], first_coordinate: usize) -> Result<Point3, MeshError> {
    let x = LossyF64Import::new(values[0], first_coordinate).map_err(MeshError::one)?;
    let y = LossyF64Import::new(values[1], first_coordinate + 1).map_err(MeshError::one)?;
    let z = LossyF64Import::new(values[2], first_coordinate + 2).map_err(MeshError::one)?;
    Ok(Point3::new(x.value, y.value, z.value))
}

/// Error returned when an [`ExactMesh`] retained-state audit fails.
///
/// This is a whole-object consistency check over topology facts, exact bounds,
/// object facts and proof-producing predicate provenance as part of the
/// certified mesh state rather than as incidental cache entries.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactMeshValidationError {
    /// The retained vertex count disagrees with the vertex buffer length.
    VertexCountMismatch {
        /// Vertex buffer length.
        expected: usize,
        /// Retained mesh-fact count.
        actual: usize,
    },
    /// The retained face count disagrees with the triangle buffer length.
    FaceCountMismatch {
        /// Triangle buffer length.
        expected: usize,
        /// Retained mesh-fact count.
        actual: usize,
    },
    /// Retained bounds failed their own validation.
    Bounds(BoundsValidationError),
    /// Retained mesh facts failed their own validation.
    Facts(MeshFactsValidationError),
    /// Retained provenance failed its own validation.
    Provenance(ConstructionProvenanceValidationError),
    /// Predicate provenance no longer mirrors the retained face predicate
    /// certificates.
    PredicateRetentionMismatch,
}

impl ExactMesh {
    /// Construct an exact mesh from exact vertices and triangle indices.
    pub fn new(
        vertices: Vec<Point3>,
        triangles: Vec<Triangle>,
        source: SourceProvenance,
    ) -> Result<Self, MeshError> {
        Self::new_with_policy(vertices, triangles, source, ValidationPolicy::CLOSED)
    }

    /// Construct an exact mesh with an explicit validation policy.
    pub fn new_with_policy(
        vertices: Vec<Point3>,
        triangles: Vec<Triangle>,
        source: SourceProvenance,
        policy: ValidationPolicy,
    ) -> Result<Self, MeshError> {
        let index_diagnostics = validate_indices(vertices.len(), &triangles);
        if !index_diagnostics.is_empty() {
            return Err(MeshError::new(index_diagnostics));
        }

        let points = vertices.to_vec();
        let triangle_indices = triangles.iter().map(|tri| tri.0).collect::<Vec<_>>();
        let bounds = MeshBounds::from_triangles(&points, &triangle_indices);
        let report = validate_triangles_with_policy(&points, &triangle_indices, policy);
        if !report.is_valid() {
            return Err(MeshError::new(report.diagnostics));
        }

        let mut provenance = ConstructionProvenance::new(source);
        retain_predicates(&mut provenance, &report);

        Ok(Self {
            vertices,
            triangles,
            bounds,
            facts: report.facts,
            validation_policy: policy,
            provenance,
        })
    }

    /// Construct an exact mesh from flat primitive-float coordinates.
    ///
    /// The `f64` values are checked for finiteness and imported as exact dyadic
    /// `Real` values. They are not used later as tolerance-bearing floats.
    pub fn from_f64_triangles(pos: &[f64], idx: &[usize]) -> Result<Self, MeshError> {
        Self::from_f64_triangles_with_policy(pos, idx, ValidationPolicy::CLOSED)
    }

    /// Construct an exact mesh from flat hyperreal coordinates.
    pub fn from_real_triangles(pos: &[Real], idx: &[usize]) -> Result<Self, MeshError> {
        Self::from_real_triangles_with_policy(pos, idx, ValidationPolicy::CLOSED)
    }

    /// Construct an exact mesh from flat hyperreal coordinates with an explicit
    /// validation policy.
    pub fn from_real_triangles_with_policy(
        pos: &[Real],
        idx: &[usize],
        policy: ValidationPolicy,
    ) -> Result<Self, MeshError> {
        if !pos.len().is_multiple_of(3) {
            return Err(MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::VertexBufferArity,
                "position buffer length must be a multiple of 3",
            )));
        }
        if !idx.len().is_multiple_of(3) {
            return Err(MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::IndexBufferArity,
                "index buffer length must be a multiple of 3",
            )));
        }

        let vertices = pos
            .chunks_exact(3)
            .map(|coords| Point3::new(coords[0].clone(), coords[1].clone(), coords[2].clone()))
            .collect::<Vec<_>>();
        let triangles = idx
            .chunks_exact(3)
            .map(|tri| Triangle([tri[0], tri[1], tri[2]]))
            .collect::<Vec<_>>();

        Self::new_with_policy(
            vertices,
            triangles,
            SourceProvenance::exact("flat hyperreal triangle mesh"),
            policy,
        )
    }

    /// Construct an exact mesh from flat primitive-float coordinates with an
    /// explicit validation policy.
    pub fn from_f64_triangles_with_policy(
        pos: &[f64],
        idx: &[usize],
        policy: ValidationPolicy,
    ) -> Result<Self, MeshError> {
        if !pos.len().is_multiple_of(3) {
            return Err(MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::VertexBufferArity,
                "position buffer length must be a multiple of 3",
            )));
        }
        if !idx.len().is_multiple_of(3) {
            return Err(MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::IndexBufferArity,
                "index buffer length must be a multiple of 3",
            )));
        }

        let mut vertices = Vec::with_capacity(pos.len() / 3);
        for (vertex, coords) in pos.chunks_exact(3).enumerate() {
            let point = point_from_f64_lossy([coords[0], coords[1], coords[2]], vertex * 3)?;
            vertices.push(point);
        }

        let triangles = idx
            .chunks_exact(3)
            .map(|tri| Triangle([tri[0], tri[1], tri[2]]))
            .collect::<Vec<_>>();

        Self::new_with_policy(
            vertices,
            triangles,
            SourceProvenance::lossy_f64("flat f64 triangle mesh"),
            policy,
        )
    }

    /// Audit a flat primitive-float mesh input stream without constructing a mesh.
    pub fn inspect_f64_triangles(pos: &[f64], idx: &[usize]) -> LossyF64MeshInputReport {
        inspect_f64_mesh_input(pos, idx)
    }

    /// Construct an exact mesh from flat integer coordinates.
    ///
    /// Integer grid input is lifted directly into `hyperreal::Real` without a
    /// primitive-float edge. Keeping grid coordinates exact and structurally
    /// structure for downstream exact predicates and determinant schedules.
    pub fn from_i64_triangles(pos: &[i64], idx: &[usize]) -> Result<Self, MeshError> {
        Self::from_i64_triangles_with_policy(pos, idx, ValidationPolicy::CLOSED)
    }

    /// Construct an exact mesh from integer coordinates with an explicit
    /// validation policy.
    pub fn from_i64_triangles_with_policy(
        pos: &[i64],
        idx: &[usize],
        policy: ValidationPolicy,
    ) -> Result<Self, MeshError> {
        if !pos.len().is_multiple_of(3) {
            return Err(MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::VertexBufferArity,
                "position buffer length must be a multiple of 3",
            )));
        }
        if !idx.len().is_multiple_of(3) {
            return Err(MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::IndexBufferArity,
                "index buffer length must be a multiple of 3",
            )));
        }

        let vertices = pos
            .chunks_exact(3)
            .map(|coords| {
                Point3::new(
                    Real::from(coords[0]),
                    Real::from(coords[1]),
                    Real::from(coords[2]),
                )
            })
            .collect::<Vec<_>>();
        let triangles = idx
            .chunks_exact(3)
            .map(|tri| Triangle([tri[0], tri[1], tri[2]]))
            .collect::<Vec<_>>();

        Self::new_with_policy(
            vertices,
            triangles,
            SourceProvenance::exact("flat i64 triangle mesh"),
            policy,
        )
    }

    /// Audit a flat exact-integer mesh input stream without constructing a mesh.
    pub fn inspect_i64_triangles(pos: &[i64], idx: &[usize]) -> ExactI64MeshInputReport {
        inspect_i64_mesh_input(pos, idx)
    }

    /// Return exact vertices.
    pub fn vertices(&self) -> &[Point3] {
        &self.vertices
    }

    /// Return triangle indices.
    pub fn triangles(&self) -> &[Triangle] {
        &self.triangles
    }

    /// Return retained exact broad-phase bounds.
    ///
    /// The bounds can safely reject disjoint pairs. Non-disjoint box relations
    /// are only candidates for exact narrow-phase predicates and must not be
    /// treated as topology decisions.
    pub const fn bounds(&self) -> &MeshBounds {
        &self.bounds
    }

    /// Return retained validation facts.
    pub const fn facts(&self) -> &MeshValidationFacts {
        &self.facts
    }

    /// Return the validation policy retained at construction.
    ///
    /// The policy is part of the exact artifact boundary: an open-surface mesh
    /// constructed with [`ValidationPolicy::ALLOW_BOUNDARY`] must not later be
    /// mistaken for closed-solid evidence merely because its structural facts
    /// approximation and domain policies visible at API boundaries.
    pub const fn validation_policy(&self) -> ValidationPolicy {
        self.validation_policy
    }

    /// Return construction provenance.
    pub const fn provenance(&self) -> &ConstructionProvenance {
        &self.provenance
    }

    /// Borrow this exact mesh through the lightweight query view API.
    pub const fn view(&self) -> ExactMeshRef<'_> {
        ExactMeshRef::new(self)
    }

    /// Validate all retained state stored on this exact mesh.
    ///
    /// Mesh construction already validates inputs before returning `Ok`. This
    /// method exists for tests, fuzzing, serialization boundaries, and
    /// downstream exact algorithms that receive an `ExactMesh` artifact and
    /// want to audit that its retained bounds, topology facts, and provenance
    /// still agree before consuming them. The bounds and topology facts are
    /// replayed from the exact vertices and triangle rows before acceptance;
    /// structure as valid only while it reproduces from the exact source
    /// object it summarizes.
    pub fn validate_retained_state(&self) -> Result<(), ExactMeshValidationError> {
        if self.vertices.len() != self.facts.mesh.vertex_count {
            return Err(ExactMeshValidationError::VertexCountMismatch {
                expected: self.vertices.len(),
                actual: self.facts.mesh.vertex_count,
            });
        }
        if self.triangles.len() != self.facts.mesh.face_count {
            return Err(ExactMeshValidationError::FaceCountMismatch {
                expected: self.triangles.len(),
                actual: self.facts.mesh.face_count,
            });
        }
        let points = self.vertices.to_vec();
        let triangles = self
            .triangles
            .iter()
            .map(|triangle| triangle.0)
            .collect::<Vec<_>>();
        self.bounds
            .validate_against_sources(&points, &triangles)
            .map_err(ExactMeshValidationError::Bounds)?;
        self.facts
            .validate_against_sources_with_policy(&points, &triangles, self.validation_policy)
            .map_err(ExactMeshValidationError::Facts)?;
        self.provenance
            .validate()
            .map_err(ExactMeshValidationError::Provenance)?;

        let retained_predicates = self
            .facts
            .faces
            .iter()
            .flat_map(|face| face.triangle.degeneracy_predicates.iter().copied())
            .collect::<Vec<_>>();
        if self.provenance.predicates != retained_predicates {
            return Err(ExactMeshValidationError::PredicateRetentionMismatch);
        }
        Ok(())
    }

    /// Build a replayed retained-state audit report from this exact mesh.
    pub fn audit_report(&self) -> Result<ExactMeshAuditReport, ExactMeshValidationError> {
        audit_exact_mesh(self)
    }

    /// Build a bundled report-bearing handoff package for downstream consumers.
    ///
    /// This compatibility API is moving to csgrs with product-facing handoff
    /// routing. The default hypermesh API keeps exact mesh facts and kernel
    /// methods only.
    #[cfg(feature = "legacy-public-api")]
    pub fn handoff_package(
        &self,
    ) -> Result<super::package::ExactMeshHandoffPackage, super::package::ExactMeshHandoffPackageError>
    {
        super::package::exact_mesh_handoff_package(self)
    }

    /// Build a replayed proposal report from this accepted exact mesh.
    #[cfg(feature = "legacy-public-api")]
    pub fn proposal_report(
        &self,
    ) -> Result<
        super::proposal::ExactMeshProposalReport,
        super::proposal::ExactMeshProposalReportError,
    > {
        super::proposal::ExactMeshProposalReport::from_mesh(self)
    }

    /// Build an artifact manifest from this accepted exact mesh.
    #[cfg(feature = "legacy-public-api")]
    pub fn artifact_manifest(
        &self,
    ) -> Result<super::artifact::MeshArtifactManifest, ExactMeshValidationError> {
        super::artifact::MeshArtifactManifest::from_exact_mesh(self)
    }

    /// Materialize the exact closed union of this mesh and `right`.
    ///
    /// This is the mesh-kernel convenience entry point for named booleans. It
    /// returns only the output mesh; callers that need retained arrangement
    /// evidence should use the lower-level internal kernel stages from csgrs.
    pub fn union(&self, right: &ExactMesh) -> Result<ExactMesh, MeshError> {
        self.named_boolean_mesh(
            right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
        )
    }

    /// Materialize the exact closed intersection of this mesh and `right`.
    ///
    /// Lower-dimensional contact is regularized into the representable triangle
    /// mesh result for the default closed output contract.
    pub fn intersection(&self, right: &ExactMesh) -> Result<ExactMesh, MeshError> {
        self.named_boolean_mesh(
            right,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        )
    }

    /// Materialize the exact closed difference of this mesh minus `right`.
    pub fn difference(&self, right: &ExactMesh) -> Result<ExactMesh, MeshError> {
        self.named_boolean_mesh(
            right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
        )
    }

    fn named_boolean_mesh(
        &self,
        right: &ExactMesh,
        operation: ExactBooleanOperation,
        validation: ValidationPolicy,
    ) -> Result<ExactMesh, MeshError> {
        let request = ExactBooleanRequest::new(operation, validation);
        materialize_boolean_exact_request(self, right, request).map(|result| result.mesh().clone())
    }

    /// Build an artifact manifest for a proposal report that replays against
    /// this accepted exact mesh.
    #[cfg(feature = "legacy-public-api")]
    pub fn proposal_artifact_manifest(
        &self,
        proposal: &super::proposal::ExactMeshProposalReport,
    ) -> Result<super::artifact::MeshArtifactManifest, super::proposal::ExactMeshProposalReportError>
    {
        super::artifact::MeshArtifactManifest::from_exact_mesh_proposal(self, proposal)
    }
}

fn validate_indices(vertex_count: usize, triangles: &[Triangle]) -> Vec<MeshDiagnostic> {
    let mut diagnostics = Vec::new();
    for (face, triangle) in triangles.iter().enumerate() {
        let [a, b, c] = triangle.0;
        for vertex in [a, b, c] {
            if vertex >= vertex_count {
                diagnostics.push(
                    MeshDiagnostic::new(
                        Severity::Error,
                        DiagnosticKind::IndexOutOfBounds,
                        format!(
                            "face {face} references vertex {vertex}, but only {vertex_count} vertices exist"
                        ),
                    )
                    .with_face(face)
                    .with_vertex(vertex),
                );
            }
        }
        if a == b || b == c || c == a {
            diagnostics.push(
                MeshDiagnostic::new(
                    Severity::Error,
                    DiagnosticKind::DegenerateTriangle,
                    format!("face {face} repeats a vertex"),
                )
                .with_face(face),
            );
        }
    }
    diagnostics
}

fn retain_predicates(provenance: &mut ConstructionProvenance, report: &ValidationReport) {
    for face in &report.facts.faces {
        for predicate in &face.triangle.degeneracy_predicates {
            provenance.push_predicate(PredicateUse::from_certificate(predicate.certificate));
        }
    }
}
