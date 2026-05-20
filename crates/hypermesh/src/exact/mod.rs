//! Exact-facing mesh API for the hyper geometry stack.
//!
//! This module is the hypermesh boundary promised by the porting plan: mesh
//! state is carried with [`hyperreal::Real`] scalars and [`hyperlattice`]
//! vectors, while topology-affecting geometric decisions go through
//! [`hyperlimit`] predicate reports. That separation follows Yap, "Towards
//! Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997): local
//! caches and approximate views may improve performance, but combinatorial
//! mesh decisions must be certified or explicitly reported as unknown.

pub mod adapter;
pub mod audit;
pub mod boolean;
pub mod bounds;
#[cfg(feature = "exact-triangulation")]
pub mod box_solid;
#[cfg(feature = "exact-triangulation")]
pub mod cells;
pub mod construction;
pub mod convex;
pub mod coplanar;
pub mod error;
pub mod facts;
pub mod graph;
pub mod handoff;
pub mod intersection;
pub mod mesh;
pub mod narrow;
pub mod package;
pub mod predicates;
pub mod provenance;
pub mod readiness;
pub mod region;
pub mod reports;
pub mod scalar;
pub mod solid;
pub mod support;
pub mod surface;
pub mod validation;
pub mod view;
#[cfg(feature = "exact-triangulation")]
pub mod volumetric;
pub mod winding;

pub use adapter::{
    ExactI64MeshInputReadiness, ExactI64MeshInputReport, ExactI64MeshInputReportValidationError,
    LossyF64MeshInputReadiness, LossyF64MeshInputReport, LossyF64MeshInputReportValidationError,
    inspect_f64_mesh_input, inspect_i64_mesh_input,
};
pub use audit::{
    ExactMeshAuditError, ExactMeshAuditFreshness, ExactMeshAuditReport, audit_exact_mesh,
};
#[cfg(feature = "exact-triangulation")]
pub use boolean::{
    ExactBooleanOperation, ExactBooleanPolicy, ExactBoundaryBooleanPolicy, boolean_exact,
    boolean_exact_with_boundary_policy, boolean_selected_regions, certify_boundary_touching_report,
    certify_open_surface_disjoint_report, certify_planar_arrangement_report,
    certify_refinement_report, certify_same_surface_report, certify_winding_readiness_report,
    preflight_boolean_exact,
};
pub use bounds::{AabbIntersectionKind, BoundsValidationError, ExactAabb3, MeshBounds};
#[cfg(feature = "exact-triangulation")]
pub use cells::{ExactFaceCellTriangulationPlan, triangulate_all_face_cells_with_cdt};
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
pub use graph::{
    CoplanarArrangementReadinessReport, CoplanarArrangementReadinessStatus,
    CoplanarArrangementReadinessValidationError, CoplanarEdgeInterval, CoplanarEdgeOverlap,
    CoplanarEdgeSplitConstruction, CoplanarEdgeSplitPoint, CoplanarOverlapGraph,
    CoplanarOverlapGraphValidationError, CoplanarOverlapSplitGraph, CoplanarOverlapSplitPlan,
    CoplanarOverlapSplitValidationError, CoplanarVertexOverlap, EdgeSplit, EdgeSplitPoint,
    ExactEdgeSplitPlan, ExactFaceRegionPlan, ExactFaceSplitGeometryPlan, ExactFaceSplitPlan,
    ExactGraphVertex, ExactGraphVertexPlan, ExactGraphVertexUse, ExactIntersectionGraph,
    ExactSplitTopologyPlan, FacePairEvents, FaceRegionBoundary, FaceSplitBoundaryChain,
    FaceSplitBoundaryNode, FaceSplitEdge, FaceSplitGeometry, FaceSplitPlan, IntersectionEvent,
    IntersectionGraphValidationError, MeshSide, SplitEdgeChain, SplitEdgeNode, SplitPlanDiagnostic,
    SplitPlanDiagnosticKind, SplitPlanReportValidationError, SplitPlanValidationReport,
    build_intersection_graph,
};
pub use handoff::{
    ExactSolidHandoffError, ExactSolidHandoffFreshness, ExactSolidHandoffReport,
    ExactSurfaceHandoffError, ExactSurfaceHandoffFreshness, ExactSurfaceHandoffReport,
    exact_solid_handoff, exact_surface_handoff,
};
pub use intersection::{
    MeshFacePairClassification, MeshFacePairRelation, MeshFacePairValidationError,
    classify_mesh_face_pair, classify_mesh_face_pairs,
};
pub use mesh::{ExactMesh, ExactMeshValidationError, ExactPoint3, Triangle};
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
#[cfg(feature = "exact-triangulation")]
pub use region::{
    ExactBooleanAssemblyPlan, ExactOutputTriangle, ExactOutputTriangleOrientation,
    ExactOutputVertex, ExactRegionRetention, ExactRegionSelection, FaceRegionTriangulation,
    build_selected_region_mesh, checked_classify_face_regions_against_opposite_planes,
    checked_triangulate_face_regions_with_earcut, triangulate_face_regions_with_earcut,
};
pub use region::{
    FaceRegionPlaneClassification, FaceRegionPlaneRelation, FaceRegionPlaneValidationError,
    classify_face_regions_against_opposite_planes,
};
#[cfg(feature = "exact-triangulation")]
pub use reports::{
    ExactBooleanBlocker, ExactBooleanBlockerKind, ExactBooleanPreflight, ExactBooleanResult,
    ExactBooleanResultKind, ExactBooleanShortcutKind, ExactBooleanSupport,
    ExactBoundaryTouchingReport, ExactBoundaryTouchingStatus, ExactOpenSurfaceDisjointReport,
    ExactOpenSurfaceDisjointStatus, ExactPlanarArrangementReport, ExactPlanarArrangementStatus,
    ExactRefinementReport, ExactRefinementStatus, ExactReportValidationError,
    ExactSameSurfaceReport, ExactSameSurfaceStatus, ExactWindingReadinessReport,
    ExactWindingReadinessStatus,
};
pub use scalar::{ExactReal, LossyF64Import};
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
#[cfg(feature = "exact-triangulation")]
pub use surface::{
    CoplanarArrangementOperation, CoplanarConvexArrangement, CoplanarConvexHoledArrangement,
    CoplanarConvexMultiArrangement, CoplanarConvexMultiHoledArrangement,
    CoplanarConvexSurfaceContainment, CoplanarConvexSurfaceContainmentCertificate,
    CoplanarConvexSurfaceEquivalence, CoplanarConvexSurfaceReport,
    CoplanarConvexSurfaceReportError, CoplanarConvexSurfaceReportStatus,
    CoplanarTriangleArrangement, CoplanarTriangleHoledArrangement,
    arrange_coplanar_convex_surface_component_union, arrange_coplanar_convex_surface_difference,
    arrange_coplanar_convex_surface_holed_difference, arrange_coplanar_convex_surface_intersection,
    arrange_coplanar_convex_surface_multi_difference,
    arrange_coplanar_convex_surface_multi_holed_difference,
    arrange_coplanar_convex_surface_multi_intersection,
    arrange_coplanar_convex_surface_multi_union, arrange_coplanar_convex_surface_union,
    arrange_single_triangle_coplanar_difference, arrange_single_triangle_coplanar_holed_difference,
    arrange_single_triangle_coplanar_union, certify_coplanar_convex_surface_containment,
    certify_coplanar_convex_surface_equivalence, certify_coplanar_convex_surface_report,
};
pub use surface::{
    CoplanarSurfaceContainment, CoplanarSurfaceContainmentReport,
    CoplanarSurfaceContainmentReportError, CoplanarSurfaceContainmentStatus,
    CoplanarTriangleDifference, CoplanarTriangleIntersection, CoplanarTriangleUnion,
    certify_single_triangle_coplanar_containment,
    certify_single_triangle_coplanar_containment_report,
    difference_single_triangle_coplanar_surfaces, intersect_single_triangle_coplanar_surfaces,
    union_single_triangle_coplanar_surfaces,
};
pub use validation::{
    BoundaryPolicy, ValidationPolicy, ValidationReport, validate_triangles,
    validate_triangles_with_policy,
};
pub use view::{
    ApproximateMeshF64View, ApproximateMeshF64ViewError, ApproximateMeshF64ViewFreshness,
    approximate_mesh_f64_view,
};
#[cfg(feature = "exact-triangulation")]
pub use volumetric::{
    ExactVolumetricRegionClassification, ExactVolumetricRegionError, ExactVolumetricRegionRelation,
    classify_triangulated_region_against_closed_mesh,
    classify_triangulated_region_triangle_against_closed_mesh,
    classify_triangulated_regions_against_opposite_meshes,
};
pub use winding::{
    ClosedMeshWindingMeshRelation, ClosedMeshWindingMeshReport, ClosedMeshWindingRelation,
    PointMeshWindingReport, WindingRayAxis, WindingReportError,
    classify_mesh_vertices_against_closed_mesh_winding,
    classify_mesh_vertices_against_closed_mesh_winding_report,
    classify_point_against_closed_mesh_winding, classify_point_against_closed_mesh_winding_report,
};
