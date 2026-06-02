//! Exact-facing mesh API for the hyper geometry stack.
//!
//! Mesh coordinates are carried as [`hyperlimit::Point3`] over
//! [`hyperreal::Real`]. Topology-affecting decisions are exposed through exact
//! predicate reports, certified outputs, or explicit blockers when the
//! implementation cannot prove a requested operation.

pub mod adapter;
#[doc(hidden)]
pub mod adjacent;
pub(crate) mod adjacent_polygon;
#[doc(hidden)]
pub mod affine_box;
#[doc(hidden)]
pub mod affine_solid;
#[doc(hidden)]
pub mod affine_surface;
#[doc(hidden)]
pub mod arrangement2d;
pub mod arrangement3d;
pub mod artifact;
pub mod audit;
pub mod boolean;
#[doc(hidden)]
pub mod boolmesh;
pub mod bounds;
#[doc(hidden)]
pub mod box_solid;
pub mod cell_complex;
#[doc(hidden)]
pub mod cells;
pub mod construction;
#[doc(hidden)]
pub mod contained_adjacent;
pub mod convex;
pub mod coplanar;
pub mod error;
pub mod facts;
pub mod graph;
pub mod handoff;
pub mod intersection;
pub mod mesh;
pub mod narrow;
#[doc(hidden)]
pub mod orthogonal_solid;
#[doc(hidden)]
pub mod orthogonal_surface;
pub mod package;
#[doc(hidden)]
pub mod planar;
pub mod predicates;
pub mod proposal;
pub mod provenance;
pub mod readiness;
#[doc(hidden)]
pub mod region;
pub mod regularization;
pub mod reports;
pub mod scalar;
pub mod simplify;
pub mod solid;
pub mod support;
#[doc(hidden)]
pub mod surface;
pub mod validation;
pub mod view;
#[doc(hidden)]
pub mod volumetric;
#[doc(hidden)]
pub mod volumetric_cells;
pub mod winding;
#[doc(hidden)]
pub mod witness;

pub use adapter::{
    ExactI64MeshInputReadiness, ExactI64MeshInputReport, ExactI64MeshInputReportValidationError,
    LossyF64MeshInputReadiness, LossyF64MeshInputReport, LossyF64MeshInputReportValidationError,
    inspect_f64_mesh_input, inspect_i64_mesh_input,
};
#[cfg(feature = "internal-fuzzing")]
pub use adjacent_polygon::{
    polygon_patch_candidate_face_sets_for_internal_fuzz, polygon_patch_pairs_for_internal_fuzz,
};
pub use arrangement2d::{
    ExactArrangement2d, ExactArrangement2dBlocker, ExactArrangement2dEdge, ExactArrangement2dFace,
    ExactArrangement2dInputSegment, ExactArrangement2dOutputLoop, ExactArrangement2dOverlay,
    ExactArrangement2dOverlayFace, ExactArrangement2dRegion, ExactArrangement2dRegionRing,
    ExactArrangement2dSegmentSource, ExactArrangement2dSetOperation, ExactArrangement2dVertex,
    build_exact_arrangement2d, build_exact_arrangement2d_overlay, exact_arrangement2d_face_witness,
};
pub use arrangement3d::{
    ArrangementCarrierPlaneOverlay, ArrangementEdge, ArrangementEdgeProvenance,
    ArrangementFaceCarrier, ArrangementFaceCell, ArrangementFaceCellNode,
    ArrangementFacePlaneArrangement, ArrangementLowerDimensionalArtifact,
    ArrangementOppositeClassification, ArrangementRegion, ArrangementVertex,
    ArrangementVertexProvenance, ExactArrangement, ExactArrangement3d,
};
pub use audit::{
    ExactMeshAuditError, ExactMeshAuditFreshness, ExactMeshAuditReport, audit_exact_mesh,
};
pub use boolean::{
    ExactArrangementBooleanAttempt, ExactArrangementBooleanDecline, ExactArrangementBooleanStage,
    ExactBooleanOperation, ExactBooleanPolicy, ExactBoundaryBooleanPolicy, boolean_exact,
    boolean_exact_with_boundary_policy, boolean_selected_regions, certify_boundary_touching_report,
    certify_open_surface_disjoint_report, certify_planar_arrangement_report,
    certify_refinement_report, certify_same_surface_report, certify_winding_readiness_report,
    exact_arrangement_boolean_attempt_report, preflight_boolean_exact,
};
pub use bounds::{AabbIntersectionKind, BoundsValidationError, ExactAabb3, MeshBounds};
pub use cell_complex::{
    ExactCellComplex, ExactCellComplexFace, ExactCellRegionLabel, ExactLabeledCellComplex,
    ExactOppositeRegionLabel, ExactSelectedCellComplex,
};
pub use construction::{
    SegmentPlaneConstructionFailure, SegmentPlaneIntersection, SegmentPlaneParameterRatio,
    SegmentPlaneRelation, SegmentPlaneValidationError, intersect_segment_with_face_plane,
    intersect_segment_with_oriented_plane, intersect_segment_with_retained_face_plane,
};
pub use convex::{
    ConvexSolidIntersection, ConvexSolidSingleCapDifference, intersect_closed_convex_solids,
    subtract_closed_convex_solids_single_cap,
};
pub use coplanar::{
    CoplanarProjection, CoplanarTriangleClassification, CoplanarTriangleRelation,
    CoplanarTriangleValidationError, classify_coplanar_triangles,
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
    TrianglePlaneClassification, TrianglePlaneRelation, TrianglePlaneValidationError,
    TriangleTriangleClassification, TriangleTriangleRelation, TriangleTriangleValidationError,
    classify_mesh_triangle_against_retained_face_plane, classify_triangle_against_face_plane,
    classify_triangle_triangle,
};
pub use package::{
    ExactMeshConsumerDomain, ExactMeshDomainReportRef, ExactMeshDomainSummary,
    ExactMeshDomainSummaryError, ExactMeshDomainSummaryFreshness, ExactMeshHandoffPackage,
    ExactMeshHandoffPackageError, ExactMeshHandoffPackageFreshness, exact_mesh_handoff_package,
};
pub use predicates::{TriangleDegeneracy, TrianglePredicateReport};
pub use provenance::{
    ApproximationPolicy, ConstructionProvenance, ConstructionProvenanceValidationError, MeshSource,
    PredicateUse, SourceProvenance,
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
    ExactSameSurfaceReport, ExactSameSurfaceStatus, ExactWindingReadinessReport,
    ExactWindingReadinessStatus,
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
pub use support::{
    SupportDop3, SupportDopAxis3, SupportDopExpansionKind, SupportDopExpansionReport,
    SupportDopRefreshReport, SupportDopValidationError, SupportSlab3, SupportWitness3,
    support_dop_for_mesh,
};
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
