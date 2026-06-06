//! Shared mesh artifact vocabulary for Hyper geometry crates.
//!
//! `hypermesh` is the acceptance boundary for trusted mesh topology, but SDF,
//! voxel, BREP, export, and external-adapter crates also need to describe
//! mesh-shaped artifacts. This module provides that common vocabulary without
//! promoting preview data to topology. A producer reports source kind,
//! coordinate evidence, face records, source freshness, and numeric adapter
//! policy; exact consumers then decide whether the artifact may be used as a
//! validation handoff or only as preview/proposal data.
//!
//! The source kinds name common meshing routes rather than hiding them behind
//! anonymous triangles. Preview/export routes stay distinct from exact replay
//! routes so consumers can reject topology that was not retained as exact
//! evidence.

use super::proposal::{ExactMeshProposalReport, ExactMeshProposalReportError};
use super::{ExactMesh, ExactMeshValidationError, ValidationPolicy, audit_exact_mesh};
use hyperlimit::MeshSource;

/// Producer family for a mesh-shaped artifact.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MeshArtifactSourceKind {
    /// An accepted [`ExactMesh`] built from exact caller-owned coordinates.
    HypermeshExact,
    /// An accepted [`ExactMesh`] built from finite primitive floats lifted as
    /// exact dyadics and replayed by exact validation.
    HypermeshLossyF64Replay,
    /// An accepted [`ExactMesh`] whose source was a hypermesh adapter.
    HypermeshAdapterReplay,
    /// An accepted [`ExactMesh`] whose source was an external adapter.
    HypermeshExternalAdapterReplay,
    /// BREP planar/analytic tessellation manifest with exact source replay.
    BrepTessellation,
    /// BREP retained triangle handoff derived from source shell loops.
    BrepExactTriangleHandoff,
    /// SDF Surface Nets preview/export mesh.
    SdfSurfaceNetsPreview,
    /// Voxel exposed-face rows before lossy mesh export.
    VoxelExposedFaces,
    /// Voxel greedy/quad mesh preview or export adapter.
    VoxelGreedyPreview,
    /// External importer/exporter or runtime display adapter.
    ExternalAdapter,
    /// Producer did not preserve a known mesh artifact family.
    Unknown,
}

impl MeshArtifactSourceKind {
    /// Return whether this producer family is intrinsically preview/export
    /// evidence.
    ///
    /// Preview routes are never validation handoffs, even when their counts and
    /// records are internally well shaped.
    pub const fn is_preview_or_export_source(self) -> bool {
        matches!(self, Self::SdfSurfaceNetsPreview | Self::VoxelGreedyPreview)
    }

    /// Return whether the producer failed to retain a useful route identity.
    pub const fn is_unknown(self) -> bool {
        matches!(self, Self::Unknown)
    }
}

/// Evidence attached to emitted mesh coordinates.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MeshCoordinateEvidence {
    /// Coordinates are retained exact rationals from source object facts.
    ExactRational,
    /// Coordinates are exact integer-grid addresses.
    ExactIntegerGrid,
    /// Coordinates came from finite primitive floats but were lifted as exact
    /// dyadics before exact replay.
    ExactDyadicFromLossyFloat,
    /// Coordinates were derived by a source replay report outside hypermesh.
    CertifiedDerivedExact,
    /// Coordinates are primitive-float preview/export values only.
    LossyPrimitiveFloat,
    /// Records carry more than one evidence family; per-record audit decides
    /// whether exact replay is still possible.
    Mixed,
    /// Producer did not provide coordinate evidence.
    Unknown,
}

impl MeshCoordinateEvidence {
    /// Return whether these coordinates may participate in exact replay.
    pub const fn supports_exact_replay(self) -> bool {
        matches!(
            self,
            Self::ExactRational
                | Self::ExactIntegerGrid
                | Self::ExactDyadicFromLossyFloat
                | Self::CertifiedDerivedExact
        )
    }
}

/// Evidence attached to emitted mesh topology.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MeshTopologyEvidence {
    /// Topology replayed through [`ExactMesh::validate_retained_state`].
    ExactMeshRetainedReplay,
    /// Surface handoff replayed source-object evidence but is not a solid.
    ExactSurfaceHandoff,
    /// Solid handoff replayed source-object evidence.
    ExactSolidHandoff,
    /// Derived surface mesh whose source-face replay accepted the triangles.
    DerivedExactSurfaceHandoff,
    /// Exact voxel exposed-face rows before preview/export lowering.
    ExactVoxelFaceRows,
    /// Candidate topology still needs exact replay before acceptance.
    ProposalAwaitingReplay,
    /// Preview/export topology only.
    PreviewOnly,
    /// Producer did not provide topology evidence.
    Unknown,
}

impl MeshTopologyEvidence {
    /// Return whether this topology evidence can be a validation handoff.
    pub const fn supports_validation_handoff(self) -> bool {
        matches!(
            self,
            Self::ExactMeshRetainedReplay
                | Self::ExactSurfaceHandoff
                | Self::ExactSolidHandoff
                | Self::DerivedExactSurfaceHandoff
                | Self::ExactVoxelFaceRows
        )
    }
}

/// Intended role of a mesh-shaped artifact.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MeshArtifactRole {
    /// Accepted mesh topology owned by `hypermesh`.
    AcceptedTopology,
    /// Surface handoff to another exact geometry/domain crate.
    SurfaceHandoff,
    /// Solid handoff to another exact geometry/domain crate.
    SolidHandoff,
    /// Derived mesh that still points back to source geometry.
    DerivedHandoff,
    /// Candidate produced by an algorithm or external adapter.
    Proposal,
    /// Display or authoring preview.
    Preview,
    /// File/export view.
    Export,
}

impl MeshArtifactRole {
    /// Return whether the role is preview/export adapter data by definition.
    pub const fn is_preview_or_export(self) -> bool {
        matches!(self, Self::Preview | Self::Export)
    }
}

/// Numeric adapter contract for a mesh-shaped artifact.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MeshNumericAdapterContract {
    /// Coordinate evidence common to the artifact.
    pub coordinate_evidence: MeshCoordinateEvidence,
    /// Whether source coordinates replay as exact values at the artifact
    /// boundary.
    pub exact_coordinate_replay: bool,
    /// Whether primitive-float lowering happened for this artifact.
    pub primitive_float_lowering: bool,
    /// Whether the artifact came through a lossy adapter route.
    pub lossy_adapter_route: bool,
    /// Whether source object facts replayed at the producer boundary.
    pub source_replay_ready: bool,
    /// Whether the artifact is explicitly preview/export only.
    pub preview_only: bool,
}

impl MeshNumericAdapterContract {
    /// Contract for accepted exact coordinates.
    pub const fn exact(coordinate_evidence: MeshCoordinateEvidence) -> Self {
        Self {
            coordinate_evidence,
            exact_coordinate_replay: true,
            primitive_float_lowering: false,
            lossy_adapter_route: false,
            source_replay_ready: true,
            preview_only: false,
        }
    }

    /// Contract for exact dyadic coordinates lifted from primitive floats and
    /// replayed as an accepted mesh.
    pub const fn dyadic_lossy_replayed() -> Self {
        Self {
            coordinate_evidence: MeshCoordinateEvidence::ExactDyadicFromLossyFloat,
            exact_coordinate_replay: true,
            primitive_float_lowering: true,
            lossy_adapter_route: true,
            source_replay_ready: true,
            preview_only: false,
        }
    }

    /// Contract for preview/export meshes whose coordinates are lossy views.
    pub const fn preview(coordinate_evidence: MeshCoordinateEvidence) -> Self {
        Self {
            coordinate_evidence,
            exact_coordinate_replay: false,
            primitive_float_lowering: true,
            lossy_adapter_route: true,
            source_replay_ready: false,
            preview_only: true,
        }
    }
}

/// One vertex record in a shared mesh artifact manifest.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MeshArtifactVertexRecord {
    /// Manifest-local vertex index.
    pub index: usize,
    /// Coordinate evidence for this vertex.
    pub coordinate_evidence: MeshCoordinateEvidence,
}

/// One face record in a shared mesh artifact manifest.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MeshArtifactFaceRecord {
    /// Manifest-local face index.
    pub index: usize,
    /// Vertex references retained by this face record.
    pub vertices: Vec<usize>,
    /// Topology evidence for this face.
    pub topology_evidence: MeshTopologyEvidence,
}

/// Shared mesh artifact manifest emitted by a producer crate.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MeshArtifactManifest {
    /// Producer family.
    pub source_kind: MeshArtifactSourceKind,
    /// Human-readable source label.
    pub source_label: String,
    /// Source/construction version retained by the producer.
    pub source_version: u64,
    /// Expected current source version, when a consumer knows it.
    pub expected_source_version: Option<u64>,
    /// Intended role of the artifact.
    pub role: MeshArtifactRole,
    /// Numeric adapter contract.
    pub numeric_contract: MeshNumericAdapterContract,
    /// Declared vertex count.
    pub declared_vertex_count: usize,
    /// Declared face count.
    pub declared_face_count: usize,
    /// Optional per-vertex records.
    pub vertices: Vec<MeshArtifactVertexRecord>,
    /// Optional per-face records.
    pub faces: Vec<MeshArtifactFaceRecord>,
}

/// Explicit blocker found while auditing a shared mesh artifact manifest.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MeshArtifactBlocker {
    /// Source label is empty after trimming.
    EmptySourceLabel,
    /// Source version is zero.
    InvalidSourceVersion,
    /// Expected and actual source versions do not match.
    SourceVersionMismatch,
    /// Artifact declares zero vertices.
    EmptyVertexSet,
    /// Artifact declares zero faces.
    EmptyFaceSet,
    /// Declared vertex count and vertex records disagree.
    MissingOrMismatchedVertexRecords,
    /// Declared face count and face records disagree.
    MissingOrMismatchedFaceRecords,
    /// A vertex record index does not match its position.
    VertexIndexMismatch,
    /// A face record index does not match its position.
    FaceIndexMismatch,
    /// A face has fewer than three vertex references.
    FaceArityTooSmall,
    /// A face references a missing vertex.
    FaceVertexOutOfRange,
    /// Coordinates cannot be replayed as exact values.
    MissingExactCoordinateReplay,
    /// Topology evidence is not acceptable as a validation handoff.
    MissingExactTopologyReplay,
    /// Artifact is explicitly preview/export only.
    PreviewOrExportOnly,
    /// Producer family is intrinsically preview/export evidence.
    PreviewOrExportSource,
    /// Producer family did not retain a known mesh artifact route.
    UnknownSourceKind,
    /// Source-object replay was not retained by the producer.
    MissingSourceReplay,
    /// Manifest-level coordinate evidence disagrees with vertex records.
    CoordinateEvidenceMismatch,
}

/// Audited shared mesh artifact report.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MeshArtifactReport {
    /// Producer family.
    pub source_kind: MeshArtifactSourceKind,
    /// Source label.
    pub source_label: String,
    /// Artifact role.
    pub role: MeshArtifactRole,
    /// Declared vertex count.
    pub vertex_count: usize,
    /// Declared face count.
    pub face_count: usize,
    /// Numeric contract copied from the manifest.
    pub numeric_contract: MeshNumericAdapterContract,
    /// Whether source freshness was checked and accepted.
    pub source_current: bool,
    /// Whether coordinates can participate in exact replay.
    pub coordinates_exact_replay_ready: bool,
    /// Whether topology can participate in exact validation handoff.
    pub topology_validation_replay_ready: bool,
    /// Whether this artifact is preview/export only.
    pub preview_only: bool,
    /// Whether a downstream exact mesh validation handoff may consume this
    /// artifact as evidence.
    pub validation_handoff_ready: bool,
    /// Explicit blockers.
    pub blockers: Vec<MeshArtifactBlocker>,
}

impl MeshArtifactManifest {
    /// Build a manifest from an accepted [`ExactMesh`].
    ///
    /// This is the canonical `hypermesh` producer adapter. It replays the mesh
    /// audit first, then emits exact vertex and triangle records. Lossy-float
    /// inputs remain visible as `HypermeshLossyF64Replay`; exact dyadic replay
    /// is not relabeled as caller-owned exact input.
    pub fn from_exact_mesh(mesh: &ExactMesh) -> Result<Self, ExactMeshValidationError> {
        let audit = audit_exact_mesh(mesh)?;
        let source_kind = source_kind_from_mesh_source(audit.source);
        let numeric_contract = numeric_contract_from_mesh_source(audit.source);
        let coordinate_evidence = numeric_contract.coordinate_evidence;
        let vertices = (0..audit.vertex_count)
            .map(|index| MeshArtifactVertexRecord {
                index,
                coordinate_evidence,
            })
            .collect::<Vec<_>>();
        let faces = mesh
            .triangles()
            .iter()
            .enumerate()
            .map(|(index, triangle)| MeshArtifactFaceRecord {
                index,
                vertices: triangle.0.to_vec(),
                topology_evidence: MeshTopologyEvidence::ExactMeshRetainedReplay,
            })
            .collect::<Vec<_>>();
        Ok(Self {
            source_kind,
            source_label: audit.source_label,
            source_version: audit.construction_version,
            expected_source_version: Some(audit.construction_version),
            role: role_for_validation_policy(
                mesh.validation_policy(),
                mesh.facts().mesh.closed_manifold,
            ),
            numeric_contract,
            declared_vertex_count: audit.vertex_count,
            declared_face_count: audit.face_count,
            vertices,
            faces,
        })
    }

    /// Build a manifest from an accepted exact mesh proposal report.
    ///
    /// This is the proposal-facing adapter for hypermesh-derived topology,
    /// primitive-float imports, and external producers that have already crossed
    /// [`certify_exact_mesh_proposal`](crate::proposal::certify_exact_mesh_proposal).
    /// The proposal report must replay against `mesh` before the shared
    /// artifact is emitted. That keeps the accepted topology, source route, and
    /// lossy-adapter status coupled to the exact mesh audit, as required by
    pub fn from_exact_mesh_proposal(
        mesh: &ExactMesh,
        proposal: &ExactMeshProposalReport,
    ) -> Result<Self, ExactMeshProposalReportError> {
        proposal.validate_against_mesh(mesh)?;
        Self::from_exact_mesh(mesh).map_err(ExactMeshProposalReportError::Audit)
    }

    /// Build a manifest from explicit producer records.
    pub fn new(
        source_kind: MeshArtifactSourceKind,
        source_label: impl Into<String>,
        source_version: u64,
        role: MeshArtifactRole,
        numeric_contract: MeshNumericAdapterContract,
        vertices: Vec<MeshArtifactVertexRecord>,
        faces: Vec<MeshArtifactFaceRecord>,
    ) -> Self {
        Self {
            source_kind,
            source_label: source_label.into(),
            source_version,
            expected_source_version: Some(source_version),
            role,
            numeric_contract,
            declared_vertex_count: vertices.len(),
            declared_face_count: faces.len(),
            vertices,
            faces,
        }
    }

    /// Build a BREP tessellation handoff from explicit exact records.
    ///
    /// The caller must supply source-object replay evidence for the BREP face
    /// or shell that produced these records. This helper only packages that
    /// evidence in the shared Hyper mesh vocabulary; it does not promote a
    /// sampled preview tessellation to accepted topology.
    pub fn brep_tessellation_handoff(
        source_label: impl Into<String>,
        source_version: u64,
        vertices: Vec<MeshArtifactVertexRecord>,
        faces: Vec<MeshArtifactFaceRecord>,
    ) -> Self {
        Self::new(
            MeshArtifactSourceKind::BrepTessellation,
            source_label,
            source_version,
            MeshArtifactRole::DerivedHandoff,
            MeshNumericAdapterContract::exact(MeshCoordinateEvidence::CertifiedDerivedExact),
            vertices,
            faces,
        )
    }

    /// Build a BREP exact-triangle handoff from explicit exact records.
    ///
    /// This route is for retained shell/face triangulations whose source loops
    /// model treats approximate views and proof-bearing objects as distinct
    /// artifacts.
    pub fn brep_exact_triangle_handoff(
        source_label: impl Into<String>,
        source_version: u64,
        vertices: Vec<MeshArtifactVertexRecord>,
        faces: Vec<MeshArtifactFaceRecord>,
    ) -> Self {
        Self::new(
            MeshArtifactSourceKind::BrepExactTriangleHandoff,
            source_label,
            source_version,
            MeshArtifactRole::DerivedHandoff,
            MeshNumericAdapterContract::exact(MeshCoordinateEvidence::CertifiedDerivedExact),
            vertices,
            faces,
        )
    }

    /// Build an exact voxel exposed-face handoff.
    ///
    /// Exposed voxel faces are retained as exact integer-grid evidence rather
    /// than as a lossy display mesh. Greedy display/export meshes should use
    /// [`Self::voxel_greedy_preview`] instead; this distinction follows the
    pub fn voxel_exposed_face_handoff(
        source_label: impl Into<String>,
        source_version: u64,
        vertices: Vec<MeshArtifactVertexRecord>,
        faces: Vec<MeshArtifactFaceRecord>,
    ) -> Self {
        Self::new(
            MeshArtifactSourceKind::VoxelExposedFaces,
            source_label,
            source_version,
            MeshArtifactRole::DerivedHandoff,
            MeshNumericAdapterContract::exact(MeshCoordinateEvidence::ExactIntegerGrid),
            vertices,
            faces,
        )
    }

    /// Build a count-only SDF Surface Nets preview manifest.
    ///
    /// Surface Nets, after Gibson, "Constrained Elastic Surface Nets" (1998),
    /// is a useful preview/extraction route, but this constructor deliberately
    /// emits preview evidence. Exact acceptance requires a separate replaying
    /// handoff rather than relabeling the preview mesh.
    pub fn sdf_surface_nets_preview(
        source_label: impl Into<String>,
        vertex_count: usize,
        face_count: usize,
    ) -> Self {
        Self::preview_summary(
            MeshArtifactSourceKind::SdfSurfaceNetsPreview,
            source_label,
            vertex_count,
            face_count,
        )
    }

    /// Build a count-only voxel greedy-mesh preview manifest.
    ///
    /// Greedy voxel meshing is treated as display/export adapter evidence in
    /// the sense of Lysenko, "Meshing in a Minecraft Game" (2012). Exact voxel
    /// cell rows should use [`Self::voxel_exposed_face_handoff`].
    pub fn voxel_greedy_preview(
        source_label: impl Into<String>,
        vertex_count: usize,
        face_count: usize,
    ) -> Self {
        Self::preview_summary(
            MeshArtifactSourceKind::VoxelGreedyPreview,
            source_label,
            vertex_count,
            face_count,
        )
    }

    /// Build a count-only preview/export manifest.
    ///
    /// Count-only preview reports are useful for SDF and voxel exporters that
    /// already own their local payload. They intentionally lack vertex/face
    /// replay records and therefore cannot become validation handoffs.
    pub fn preview_summary(
        source_kind: MeshArtifactSourceKind,
        source_label: impl Into<String>,
        vertex_count: usize,
        face_count: usize,
    ) -> Self {
        Self {
            source_kind,
            source_label: source_label.into(),
            source_version: 1,
            expected_source_version: Some(1),
            role: MeshArtifactRole::Preview,
            numeric_contract: MeshNumericAdapterContract::preview(
                MeshCoordinateEvidence::LossyPrimitiveFloat,
            ),
            declared_vertex_count: vertex_count,
            declared_face_count: face_count,
            vertices: Vec::new(),
            faces: Vec::new(),
        }
    }

    /// Audit this manifest into a report.
    pub fn report(&self) -> MeshArtifactReport {
        let mut blockers = Vec::new();
        if self.source_label.trim().is_empty() {
            blockers.push(MeshArtifactBlocker::EmptySourceLabel);
        }
        if self.source_version == 0 {
            blockers.push(MeshArtifactBlocker::InvalidSourceVersion);
        }
        if self.source_kind.is_unknown() {
            blockers.push(MeshArtifactBlocker::UnknownSourceKind);
        }
        let source_current = self
            .expected_source_version
            .is_none_or(|expected| expected == self.source_version);
        if !source_current {
            blockers.push(MeshArtifactBlocker::SourceVersionMismatch);
        }
        if self.declared_vertex_count == 0 {
            blockers.push(MeshArtifactBlocker::EmptyVertexSet);
        }
        if self.declared_face_count == 0 {
            blockers.push(MeshArtifactBlocker::EmptyFaceSet);
        }
        if self.vertices.len() != self.declared_vertex_count {
            blockers.push(MeshArtifactBlocker::MissingOrMismatchedVertexRecords);
        }
        if self.faces.len() != self.declared_face_count {
            blockers.push(MeshArtifactBlocker::MissingOrMismatchedFaceRecords);
        }
        for (expected, vertex) in self.vertices.iter().enumerate() {
            if vertex.index != expected {
                blockers.push(MeshArtifactBlocker::VertexIndexMismatch);
                break;
            }
        }
        for (expected, face) in self.faces.iter().enumerate() {
            if face.index != expected {
                blockers.push(MeshArtifactBlocker::FaceIndexMismatch);
            }
            if face.vertices.len() < 3 {
                blockers.push(MeshArtifactBlocker::FaceArityTooSmall);
            }
            if face
                .vertices
                .iter()
                .any(|&vertex| vertex >= self.declared_vertex_count)
            {
                blockers.push(MeshArtifactBlocker::FaceVertexOutOfRange);
            }
        }

        let coordinate_evidence_consistent = coordinate_evidence_consistent(
            self.numeric_contract.coordinate_evidence,
            &self.vertices,
        );
        if !coordinate_evidence_consistent {
            blockers.push(MeshArtifactBlocker::CoordinateEvidenceMismatch);
        }

        if !self.numeric_contract.source_replay_ready {
            blockers.push(MeshArtifactBlocker::MissingSourceReplay);
        }

        let coordinates_exact_replay_ready = self.numeric_contract.exact_coordinate_replay
            && self.numeric_contract.source_replay_ready
            && coordinate_evidence_consistent
            && contract_coordinate_evidence_supports_exact_replay(
                self.numeric_contract.coordinate_evidence,
                &self.vertices,
            )
            && self
                .vertices
                .iter()
                .all(|vertex| vertex.coordinate_evidence.supports_exact_replay());
        if !coordinates_exact_replay_ready {
            blockers.push(MeshArtifactBlocker::MissingExactCoordinateReplay);
        }

        let topology_validation_replay_ready = self
            .faces
            .iter()
            .all(|face| face.topology_evidence.supports_validation_handoff())
            && !self.faces.is_empty();
        if !topology_validation_replay_ready {
            blockers.push(MeshArtifactBlocker::MissingExactTopologyReplay);
        }

        let preview_source = self.source_kind.is_preview_or_export_source();
        if preview_source {
            blockers.push(MeshArtifactBlocker::PreviewOrExportSource);
        }

        let preview_only = self.role.is_preview_or_export()
            || self.numeric_contract.preview_only
            || preview_source;
        if preview_only {
            blockers.push(MeshArtifactBlocker::PreviewOrExportOnly);
        }

        blockers.sort_by_key(|blocker| *blocker as u8);
        blockers.dedup();

        let validation_handoff_ready = blockers.is_empty()
            && source_current
            && coordinates_exact_replay_ready
            && topology_validation_replay_ready
            && !preview_only;

        MeshArtifactReport {
            source_kind: self.source_kind,
            source_label: self.source_label.clone(),
            role: self.role,
            vertex_count: self.declared_vertex_count,
            face_count: self.declared_face_count,
            numeric_contract: self.numeric_contract,
            source_current,
            coordinates_exact_replay_ready,
            topology_validation_replay_ready,
            preview_only,
            validation_handoff_ready,
            blockers,
        }
    }
}

/// Build a shared mesh artifact report from an accepted exact mesh.
pub fn mesh_artifact_from_exact_mesh(
    mesh: &ExactMesh,
) -> Result<MeshArtifactReport, ExactMeshValidationError> {
    Ok(MeshArtifactManifest::from_exact_mesh(mesh)?.report())
}

/// Build a shared mesh artifact report from an accepted exact mesh proposal.
pub fn mesh_artifact_from_exact_mesh_proposal(
    mesh: &ExactMesh,
    proposal: &ExactMeshProposalReport,
) -> Result<MeshArtifactReport, ExactMeshProposalReportError> {
    Ok(MeshArtifactManifest::from_exact_mesh_proposal(mesh, proposal)?.report())
}

fn source_kind_from_mesh_source(source: MeshSource) -> MeshArtifactSourceKind {
    match source {
        MeshSource::Exact => MeshArtifactSourceKind::HypermeshExact,
        MeshSource::LossyF64 => MeshArtifactSourceKind::HypermeshLossyF64Replay,
        MeshSource::HypermeshAdapter => MeshArtifactSourceKind::HypermeshAdapterReplay,
        MeshSource::ExternalAdapter => MeshArtifactSourceKind::HypermeshExternalAdapterReplay,
    }
}

fn numeric_contract_from_mesh_source(source: MeshSource) -> MeshNumericAdapterContract {
    match source {
        MeshSource::Exact => {
            MeshNumericAdapterContract::exact(MeshCoordinateEvidence::ExactRational)
        }
        MeshSource::LossyF64 => MeshNumericAdapterContract::dyadic_lossy_replayed(),
        MeshSource::HypermeshAdapter | MeshSource::ExternalAdapter => MeshNumericAdapterContract {
            coordinate_evidence: MeshCoordinateEvidence::CertifiedDerivedExact,
            exact_coordinate_replay: true,
            primitive_float_lowering: false,
            lossy_adapter_route: true,
            source_replay_ready: true,
            preview_only: false,
        },
    }
}

fn role_for_validation_policy(policy: ValidationPolicy, closed: bool) -> MeshArtifactRole {
    if policy == ValidationPolicy::CLOSED && closed {
        MeshArtifactRole::SolidHandoff
    } else {
        MeshArtifactRole::SurfaceHandoff
    }
}

fn coordinate_evidence_consistent(
    contract: MeshCoordinateEvidence,
    vertices: &[MeshArtifactVertexRecord],
) -> bool {
    match contract {
        MeshCoordinateEvidence::Mixed => true,
        evidence => vertices
            .iter()
            .all(|vertex| vertex.coordinate_evidence == evidence),
    }
}

fn contract_coordinate_evidence_supports_exact_replay(
    contract: MeshCoordinateEvidence,
    vertices: &[MeshArtifactVertexRecord],
) -> bool {
    match contract {
        MeshCoordinateEvidence::Mixed => {
            !vertices.is_empty()
                && vertices
                    .iter()
                    .all(|vertex| vertex.coordinate_evidence.supports_exact_replay())
        }
        evidence => evidence.supports_exact_replay(),
    }
}
