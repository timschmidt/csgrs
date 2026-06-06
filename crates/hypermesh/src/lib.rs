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
pub mod artifact;
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
mod facts;
mod graph;
mod handoff;
mod intersection;
pub(crate) mod loop_triangulation;
mod mesh;
mod narrow;
mod orthogonal_solid;
mod package;
pub mod proposal;
mod readiness;
mod region;
mod regularization;
mod reports;
mod scalar;
mod simplify;
mod solid;
mod support;
mod topology;
mod validation;
mod view;
mod volumetric;
mod volumetric_cells;
mod winding;
mod witness;

pub use adapter::{
    ExactI64MeshInputReadiness, ExactI64MeshInputReport, ExactI64MeshInputReportValidationError,
    LossyF64MeshInputReadiness, LossyF64MeshInputReport, LossyF64MeshInputReportValidationError,
    inspect_f64_mesh_input, inspect_i64_mesh_input,
};
pub use arrangement2d::{
    ExactArrangement2d, ExactArrangement2dBlocker, ExactArrangement2dEdge, ExactArrangement2dFace,
    ExactArrangement2dInputSegment, ExactArrangement2dOutputComponent,
    ExactArrangement2dOutputLoop, ExactArrangement2dOverlay, ExactArrangement2dOverlayFace,
    ExactArrangement2dRegion, ExactArrangement2dRegionRing, ExactArrangement2dSegmentSource,
    ExactArrangement2dSetOperation, ExactArrangement2dVertex, build_exact_arrangement2d,
    build_exact_arrangement2d_overlay, exact_arrangement2d_face_witness,
};
pub use arrangement3d::{
    ArrangementCarrierPlaneOverlay, ArrangementEdge, ArrangementEdgeProvenance,
    ArrangementFaceCarrier, ArrangementFaceCell, ArrangementFaceCellNode,
    ArrangementFacePlaneArrangement, ArrangementLowerDimensionalArtifact,
    ArrangementOppositeClassification, ArrangementRegion, ArrangementRegionEdgeIncidence,
    ArrangementRegionSide, ArrangementVertex, ArrangementVertexProvenance,
    ArrangementVolumeAdjacency, ArrangementVolumeFaceSide, ArrangementVolumeRegion,
    ExactArrangement, ExactArrangement3d,
};
pub use audit::{
    ExactMeshAuditError, ExactMeshAuditFreshness, ExactMeshAuditReport, audit_exact_mesh,
};
pub use boolean::{
    ExactArrangementBooleanAttempt, ExactArrangementBooleanDecline, ExactArrangementBooleanStage,
    ExactBooleanOperation, ExactBooleanPolicy, ExactBoundaryBooleanPolicy, boolean_exact,
    boolean_exact_with_boundary_policy, boolean_selected_regions, certify_boundary_touching_report,
    certify_open_surface_disjoint_report, certify_planar_arrangement_report,
    certify_refinement_report, certify_same_surface_report,
    certify_volumetric_boundary_closure_report, certify_winding_readiness_report,
    exact_arrangement_boolean_attempt_report, preflight_boolean_exact,
    preflight_boolean_exact_with_boundary_policy, preflight_boolean_exact_with_validation,
};
pub use bounds::{AabbIntersectionKind, BoundsValidationError, ExactAabb3, MeshBounds};
pub use cell_complex::{
    ExactCellComplex, ExactCellComplexFace, ExactCellComplexVolumeRegion, ExactCellRegionLabel,
    ExactLabeledCellComplex, ExactOppositeRegionLabel, ExactSelectedCellComplex,
    ExactSelectedFaceOrientation,
};
pub use construction::{
    intersect_segment_with_face_plane, intersect_segment_with_retained_face_plane,
};
pub use convex::{
    ConvexSolidDifference, ConvexSolidIntersection, ConvexSolidUnion,
    intersect_closed_convex_solids, subtract_closed_convex_solids, union_closed_convex_solids,
};
pub use error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
pub use facts::{
    EdgeFacts, FaceFacts, FacePlaneFacts, MeshFacts, MeshFactsValidationError, MeshValidationFacts,
    OrientedFaceFacts, TriangleFacts, VertexFacts, VertexLinkKind,
};
pub use graph::build_intersection_graph;
pub use handoff::{
    ExactSolidHandoffError, ExactSolidHandoffFreshness, ExactSolidHandoffReport,
    ExactSurfaceHandoffError, ExactSurfaceHandoffFreshness, ExactSurfaceHandoffReport,
    exact_solid_handoff, exact_surface_handoff,
};
pub use intersection::{
    MeshFacePairClassification, MeshFacePairRelation, MeshFacePairValidationError,
    classify_mesh_face_pair, classify_mesh_face_pairs,
};
pub use mesh::{ExactMesh, ExactMeshValidationError, Triangle};
pub use narrow::{
    TriangleTriangleClassification, TriangleTriangleRelation, TriangleTriangleValidationError,
    classify_mesh_triangle_against_retained_face_plane, classify_triangle_against_face_plane,
    classify_triangle_triangle,
};
pub use package::{
    ExactMeshConsumerDomain, ExactMeshDomainReportRef, ExactMeshDomainSummary,
    ExactMeshDomainSummaryError, ExactMeshDomainSummaryFreshness, ExactMeshHandoffPackage,
    ExactMeshHandoffPackageError, ExactMeshHandoffPackageFreshness, exact_mesh_handoff_package,
};
pub use readiness::{
    ExactMeshConsumerReadinessError, ExactMeshConsumerReadinessFreshness,
    ExactMeshConsumerReadinessReport, exact_mesh_consumer_readiness,
};
pub use regularization::{
    ExactArrangementBlocker, ExactLowerDimensionalPolicy, ExactRegularizationPolicy,
    ExactUnresolvedPolicy,
};
pub use reports::{
    ExactBooleanBlocker, ExactBooleanBlockerKind, ExactBooleanPreflight, ExactBooleanResult,
    ExactBooleanResultKind, ExactBooleanShortcutKind, ExactBooleanSupport,
    ExactBoundaryTouchingReport, ExactBoundaryTouchingStatus, ExactOpenSurfaceDisjointReport,
    ExactOpenSurfaceDisjointStatus, ExactPlanarArrangementReport, ExactPlanarArrangementStatus,
    ExactRefinementReport, ExactRefinementStatus, ExactReportFreshness, ExactReportValidationError,
    ExactSameSurfaceReport, ExactSameSurfaceStatus, ExactVolumetricBoundaryClosureReport,
    ExactVolumetricBoundaryClosureStatus, ExactWindingReadinessReport, ExactWindingReadinessStatus,
};
pub use scalar::LossyF64Import;
pub use simplify::{ExactSimplifiedCellComplex, ExactSimplifiedFaceCell};
pub use solid::{
    ClosedMeshOrientation, ConvexSolidClassification, ConvexSolidFacts,
    ConvexSolidMeshClassification, ConvexSolidMeshRelation, ConvexSolidPointClassification,
    ConvexSolidPointRelation, ConvexSolidReportError, certify_convex_solid,
    classify_mesh_vertices_against_convex_solid,
    classify_mesh_vertices_against_convex_solid_report, classify_point_against_convex_solid,
    classify_point_against_convex_solid_report,
};
pub use support::support_dop_for_mesh;
pub use validation::{
    BoundaryPolicy, ValidationPolicy, ValidationReport, validate_triangles,
    validate_triangles_with_policy,
};
pub use view::{
    ApproximateMeshF64View, ApproximateMeshF64ViewError, ApproximateMeshF64ViewFreshness,
    approximate_mesh_f64_view,
};
pub use winding::{
    ClosedMeshWindingMeshRelation, ClosedMeshWindingMeshReport, ClosedMeshWindingRelation,
    PointMeshWindingReport, WindingRayAxis, WindingReportError,
    classify_mesh_vertices_against_closed_mesh_winding,
    classify_mesh_vertices_against_closed_mesh_winding_report,
    classify_point_against_closed_mesh_winding, classify_point_against_closed_mesh_winding_report,
};
