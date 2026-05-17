//! Exact-facing mesh API for the hyper geometry stack.
//!
//! This module is the hypermesh boundary promised by the porting plan: mesh
//! state is carried with [`hyperreal::Real`] scalars and [`hyperlattice`]
//! vectors, while topology-affecting geometric decisions go through
//! [`hyperlimit`] predicate reports. That separation follows Yap, "Towards
//! Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997): local
//! caches and approximate views may improve performance, but combinatorial
//! mesh decisions must be certified or explicitly reported as unknown.

pub mod boolean;
pub mod bounds;
pub mod construction;
pub mod coplanar;
pub mod error;
pub mod facts;
pub mod graph;
pub mod intersection;
pub mod mesh;
pub mod narrow;
pub mod predicates;
pub mod provenance;
pub mod region;
pub mod scalar;
pub mod validation;

#[cfg(feature = "exact-triangulation")]
pub use boolean::{
    ExactBooleanOperation, ExactBooleanPolicy, ExactBooleanPreflight, ExactBooleanResult,
    ExactBooleanSupport, boolean_exact, boolean_selected_regions, preflight_boolean_exact,
};
pub use bounds::{AabbIntersectionKind, ExactAabb3, MeshBounds};
pub use construction::{
    SegmentPlaneIntersection, SegmentPlaneRelation, intersect_segment_with_face_plane,
    intersect_segment_with_oriented_plane,
};
pub use coplanar::{
    CoplanarProjection, CoplanarTriangleClassification, CoplanarTriangleRelation,
    classify_coplanar_triangles,
};
pub use error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
pub use facts::{
    EdgeFacts, FaceFacts, MeshFacts, MeshValidationFacts, OrientedFaceFacts, TriangleFacts,
    VertexFacts, VertexLinkKind,
};
pub use graph::{
    EdgeSplit, EdgeSplitPoint, ExactEdgeSplitPlan, ExactFaceRegionPlan, ExactFaceSplitGeometryPlan,
    ExactFaceSplitPlan, ExactGraphVertex, ExactGraphVertexPlan, ExactGraphVertexUse,
    ExactIntersectionGraph, ExactSplitTopologyPlan, FacePairEvents, FaceRegionBoundary,
    FaceSplitBoundaryChain, FaceSplitBoundaryNode, FaceSplitEdge, FaceSplitGeometry, FaceSplitPlan,
    IntersectionEvent, MeshSide, SplitEdgeChain, SplitEdgeNode, SplitPlanDiagnostic,
    SplitPlanDiagnosticKind, SplitPlanValidationReport, build_intersection_graph,
};
pub use intersection::{
    MeshFacePairClassification, MeshFacePairRelation, classify_mesh_face_pair,
    classify_mesh_face_pairs,
};
pub use mesh::{ExactMesh, ExactPoint3, Triangle};
pub use narrow::{
    TrianglePlaneClassification, TrianglePlaneRelation, TriangleTriangleClassification,
    TriangleTriangleRelation, classify_triangle_against_face_plane, classify_triangle_triangle,
};
pub use predicates::{TriangleDegeneracy, TrianglePredicateReport};
pub use provenance::{
    ApproximationPolicy, ConstructionProvenance, MeshSource, PredicateUse, SourceProvenance,
};
#[cfg(feature = "exact-triangulation")]
pub use region::{
    ExactBooleanAssemblyPlan, ExactOutputTriangle, ExactOutputVertex, ExactRegionSelection,
    FaceRegionTriangulation, build_selected_region_mesh, triangulate_face_regions_with_earcut,
};
pub use region::{
    FaceRegionPlaneClassification, FaceRegionPlaneRelation,
    classify_face_regions_against_opposite_planes,
};
pub use scalar::{ExactReal, LossyF64Import};
pub use validation::{
    BoundaryPolicy, ValidationPolicy, ValidationReport, validate_triangles,
    validate_triangles_with_policy,
};
