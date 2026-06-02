//! Report-bearing acceptance boundary for exact mesh proposals.
//!
//! External importers, boolmesh adapters, SDF tessellators, and primitive-float
//! paths may all produce candidate meshes. In the Hyper exact stack those
//! candidates are proposals until an [`ExactMesh`] has replayed its retained
//! topology, bounds, provenance, and predicate evidence. This module packages
//! that replay into a small report so downstream code cannot silently treat an
//! adapter output as trusted topology.
//!
//! Approximate or external computation may help generate objects, but
//! combinatorial use is guarded by exact object validation and certificate
//! replay. The report keeps validity and source semantics explicit.

use super::audit::{ExactMeshAuditError, ExactMeshAuditReport, audit_exact_mesh};
use super::{
    ApproximationPolicy, ConstructionProvenanceValidationError, ExactMesh,
    ExactMeshValidationError, MeshSource, ValidationPolicy,
};

/// Source class retained by an accepted mesh proposal report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactMeshProposalSourceKind {
    /// Caller-owned exact coordinates, not a proposal adapter.
    ExactConstruction,
    /// Primitive-float coordinates were lifted as exact dyadic values and then
    /// replayed by exact mesh validation.
    LossyPrimitiveFloatProposal,
    /// Boolmesh-derived topology was retained only as an approximate adapter
    /// proposal.
    BoolmeshAdapterProposal,
    /// External importer or display/runtime adapter proposal.
    ExternalAdapterProposal,
}

impl ExactMeshProposalSourceKind {
    /// Return whether this source kind is an adapter proposal rather than an
    /// exact construction boundary.
    pub const fn is_adapter_proposal(self) -> bool {
        !matches!(self, Self::ExactConstruction)
    }
}

/// Mesh-proposal acceptance status after exact replay.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactMeshProposalAcceptance {
    /// The mesh was exact input and replayed as a retained exact object.
    ExactInputReplayed,
    /// A lossy, external, or boolmesh proposal replayed through exact mesh
    /// validation and retained predicate evidence.
    ProposalAcceptedAfterExactReplay,
}

/// Validation failure for a retained mesh proposal report.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactMeshProposalReportError {
    /// Source kind and source approximation policy are inconsistent.
    SourcePolicyMismatch,
    /// Source kind and acceptance status are inconsistent.
    AcceptanceMismatch,
    /// The report does not retain an accepted topology claim.
    TopologyNotAccepted,
    /// The report claims accepted topology without exact replay.
    MissingExactReplay,
    /// Adapter-proposal flag disagrees with source kind.
    AdapterFlagMismatch,
    /// The mesh failed retained-state audit construction.
    Audit(ExactMeshValidationError),
    /// The embedded audit report no longer matches the accepted mesh.
    AuditReplay(ExactMeshAuditError),
    /// Mesh provenance failed its exact/source policy validation.
    Provenance(ConstructionProvenanceValidationError),
    /// Report metadata no longer matches the mesh source provenance.
    SourceReplayMismatch,
}

/// Replayable acceptance report for an exact mesh candidate.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactMeshProposalReport {
    /// Source kind derived from the candidate mesh provenance.
    pub source_kind: ExactMeshProposalSourceKind,
    /// Source label retained by the candidate mesh provenance.
    pub source_label: String,
    /// Approximation policy retained by the candidate mesh provenance.
    pub approximation: ApproximationPolicy,
    /// Whether this report came from an adapter proposal rather than exact
    /// caller-owned input.
    pub adapter_proposal: bool,
    /// Whether exact retained-state replay was performed.
    pub exact_replay_performed: bool,
    /// Whether the candidate is accepted as topology after exact replay.
    pub accepted_topology: bool,
    /// Acceptance class after exact replay.
    pub acceptance: ExactMeshProposalAcceptance,
    /// Validation policy used to construct the retained mesh.
    pub validation_policy: ValidationPolicy,
    /// Compact retained-state audit that must replay against the mesh.
    pub audit: ExactMeshAuditReport,
}

impl ExactMeshProposalReport {
    /// Build a proposal report after exact retained-state replay.
    ///
    /// The returned report is positive evidence: the mesh has already passed
    /// [`ExactMesh::validate_retained_state`](crate::exact::ExactMesh::validate_retained_state)
    /// through [`audit_exact_mesh`]. Approximate or external source categories
    /// remain visible in the report instead of being rewritten as exact input.
    pub fn from_mesh(mesh: &ExactMesh) -> Result<Self, ExactMeshProposalReportError> {
        mesh.provenance()
            .validate()
            .map_err(ExactMeshProposalReportError::Provenance)?;
        let audit = audit_exact_mesh(mesh).map_err(ExactMeshProposalReportError::Audit)?;
        let source_kind = source_kind_for_mesh_source(mesh.provenance().source.source);
        let adapter_proposal = source_kind.is_adapter_proposal();
        let acceptance = if adapter_proposal {
            ExactMeshProposalAcceptance::ProposalAcceptedAfterExactReplay
        } else {
            ExactMeshProposalAcceptance::ExactInputReplayed
        };
        let report = Self {
            source_kind,
            source_label: mesh.provenance().source.label.clone(),
            approximation: mesh.provenance().source.approximation,
            adapter_proposal,
            exact_replay_performed: true,
            accepted_topology: true,
            acceptance,
            validation_policy: mesh.validation_policy(),
            audit,
        };
        report.validate()?;
        Ok(report)
    }

    /// Validate the report shape without access to the source mesh.
    ///
    /// Local validation rejects relabeled source kinds, adapter flags, and
    /// accepted-topology claims that do not retain exact replay evidence.
    /// Source replay against an [`ExactMesh`] is handled by
    /// [`Self::validate_against_mesh`].
    pub fn validate(&self) -> Result<(), ExactMeshProposalReportError> {
        if !self.accepted_topology {
            return Err(ExactMeshProposalReportError::TopologyNotAccepted);
        }
        if !self.exact_replay_performed {
            return Err(ExactMeshProposalReportError::MissingExactReplay);
        }
        if self.adapter_proposal != self.source_kind.is_adapter_proposal() {
            return Err(ExactMeshProposalReportError::AdapterFlagMismatch);
        }
        let expected_acceptance = if self.adapter_proposal {
            ExactMeshProposalAcceptance::ProposalAcceptedAfterExactReplay
        } else {
            ExactMeshProposalAcceptance::ExactInputReplayed
        };
        if self.acceptance != expected_acceptance {
            return Err(ExactMeshProposalReportError::AcceptanceMismatch);
        }
        if !source_policy_matches(self.source_kind, self.approximation) {
            return Err(ExactMeshProposalReportError::SourcePolicyMismatch);
        }
        if self.audit.source != mesh_source_for_source_kind(self.source_kind)
            || self.audit.source_label != self.source_label
            || self.audit.validation_policy != self.validation_policy
        {
            return Err(ExactMeshProposalReportError::SourceReplayMismatch);
        }
        Ok(())
    }

    /// Validate the report by replaying it against the accepted exact mesh.
    ///
    /// This is the acceptance boundary external proposal engines must cross:
    /// the retained audit must still match the mesh, and reconstructing the
    /// proposal report from the mesh must reproduce the copied report exactly.
    pub fn validate_against_mesh(
        &self,
        mesh: &ExactMesh,
    ) -> Result<(), ExactMeshProposalReportError> {
        self.validate()?;
        self.audit
            .validate_against_mesh(mesh)
            .map_err(ExactMeshProposalReportError::AuditReplay)?;
        let replay = Self::from_mesh(mesh)?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactMeshProposalReportError::SourceReplayMismatch)
        }
    }
}

/// Certify a mesh proposal after exact retained-state replay.
pub fn certify_exact_mesh_proposal(
    mesh: &ExactMesh,
) -> Result<ExactMeshProposalReport, ExactMeshProposalReportError> {
    ExactMeshProposalReport::from_mesh(mesh)
}

fn source_kind_for_mesh_source(source: MeshSource) -> ExactMeshProposalSourceKind {
    match source {
        MeshSource::Exact => ExactMeshProposalSourceKind::ExactConstruction,
        MeshSource::LossyF64 => ExactMeshProposalSourceKind::LossyPrimitiveFloatProposal,
        MeshSource::BoolmeshAdapter => ExactMeshProposalSourceKind::BoolmeshAdapterProposal,
        MeshSource::ExternalAdapter => ExactMeshProposalSourceKind::ExternalAdapterProposal,
    }
}

fn mesh_source_for_source_kind(source_kind: ExactMeshProposalSourceKind) -> MeshSource {
    match source_kind {
        ExactMeshProposalSourceKind::ExactConstruction => MeshSource::Exact,
        ExactMeshProposalSourceKind::LossyPrimitiveFloatProposal => MeshSource::LossyF64,
        ExactMeshProposalSourceKind::BoolmeshAdapterProposal => MeshSource::BoolmeshAdapter,
        ExactMeshProposalSourceKind::ExternalAdapterProposal => MeshSource::ExternalAdapter,
    }
}

fn source_policy_matches(
    source_kind: ExactMeshProposalSourceKind,
    approximation: ApproximationPolicy,
) -> bool {
    matches!(
        (source_kind, approximation),
        (
            ExactMeshProposalSourceKind::ExactConstruction,
            ApproximationPolicy::ExactOnly
        ) | (
            ExactMeshProposalSourceKind::LossyPrimitiveFloatProposal,
            ApproximationPolicy::EdgeOnly
        ) | (
            ExactMeshProposalSourceKind::ExternalAdapterProposal,
            ApproximationPolicy::EdgeOnly
        ) | (
            ExactMeshProposalSourceKind::BoolmeshAdapterProposal,
            ApproximationPolicy::ExplicitApproximateDecision
        )
    )
}
