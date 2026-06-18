//--- Copyright (C) 2025 Saki Komikado <komietty@gmail.com>,
//--- This Source Code Form is subject to the terms of the Mozilla Public License v.2.0.

#![allow(clippy::too_many_arguments)]
#![allow(clippy::cast_abs_to_unsigned)]
#![forbid(unsafe_code)]

//! Exact-facing mesh API for the hyper geometry stack.
//!
//! Mesh coordinates are carried as [`hyperlimit::Point3`] over
//! [`hyperreal::Real`]. Topology-affecting decisions are exposed through exact
//! predicate reports, certified outputs, or explicit blockers when the
//! implementation cannot prove a requested operation.

mod adapter;
mod adjacent;
pub(crate) mod adjacent_polygon;
mod affine_box;
mod affine_solid;
mod arrangement2d;
mod arrangement3d;
mod artifact;
mod audit;
mod boolean;
mod bounds;
mod box_solid;
mod cell_complex;
mod cells;
mod construction;
mod contained_adjacent;
mod convex;
mod error;
mod exact_key;
mod facts;
mod graph;
mod handoff;
mod intersection;
pub(crate) mod loop_triangulation;
mod mesh;
mod narrow;
mod orthogonal_solid;
mod package;
mod proposal;
mod readiness;
mod region;
mod regularization;
mod reports;
mod scalar;
mod simplify;
mod solid;
mod topology;
mod validation;
mod view;
mod volumetric;
mod volumetric_cells;
mod winding;
mod witness;
mod workspace;

pub use adapter::{
    ExactI64MeshInputReadiness, ExactI64MeshInputReportValidationError, LossyF64MeshInputReadiness,
    LossyF64MeshInputReportValidationError,
};
pub use artifact::{
    MeshArtifactBlocker, MeshArtifactFaceRecord, MeshArtifactManifest, MeshArtifactReport,
    MeshArtifactReportError, MeshArtifactRole, MeshArtifactSourceKind, MeshArtifactVertexRecord,
    MeshCoordinateEvidence, MeshNumericAdapterContract, MeshTopologyEvidence,
};
pub use audit::ExactMeshAuditError;
pub use boolean::{
    ExactArrangementBooleanAttempt, ExactBooleanEvaluation, ExactBooleanOperation,
    ExactBooleanRequest, ExactBoundaryBooleanPolicy,
};
pub use error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
pub use facts::{
    EdgeFacts, FaceFacts, FacePlaneFacts, MeshFacts, MeshFactsValidationError, MeshValidationFacts,
    OrientedFaceFacts, TriangleFacts, VertexFacts, VertexLinkKind,
};
pub use mesh::{ExactMesh, ExactMeshValidationError, Triangle};
pub use package::{
    ExactMeshConsumerDomain, ExactMeshDomainSummaryError, ExactMeshHandoffPackage,
    ExactMeshHandoffPackageError,
};
pub use proposal::{
    ExactMeshProposalAcceptance, ExactMeshProposalReport, ExactMeshProposalReportError,
    ExactMeshProposalSourceKind,
};
pub use region::{ExactOutputTriangleOrientation, ExactRegionSelection};
pub use regularization::ExactRegularizationPolicy;
pub use reports::{ExactBooleanResult, ExactReportFreshness, ExactReportValidationError};
pub use validation::ValidationPolicy;
pub use workspace::ExactBooleanWorkspace;
