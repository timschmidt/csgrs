//! Hyperreal-backed mesh boolean primitives.
//!
//! This crate keeps primitive coordinates at API boundaries only. Core
//! geometric state uses [`Real`] as its scalar and provides borrowed slice APIs
//! before owned convenience wrappers.
//!
//! The boolean kernel targets finite closed PWN triangle meshes represented
//! with exact [`Real`] coordinates. Unsupported or uncertifiable configurations
//! are reported as [`HypermeshError::UnknownClassification`] rather than being
//! guessed with approximate topology. By default, boolean operations run the
//! general EMBER subdivision/BSP/classification path; special-case boolean
//! shortcuts are not used to rescue uncertified general results. Public boolean
//! operations certify that the classified arrangement is already closed after
//! exact duplicate/T-junction resolution before returning a result.
//!
//! Use [`triangulate_and_resolve_certified`] to triangulate a boolean result
//! while preserving the invariant that open or zero-volume output is rejected
//! rather than repaired.

#![warn(missing_docs)]

pub mod bvh;
pub mod clip;
pub mod error;
pub mod geometry;
pub mod intersection;
pub mod local_bsp;
pub mod mesh;
pub mod operations;
pub mod output;
pub mod polygon;
pub mod segment_trace;
pub mod subdivision;
pub mod winding;

pub use bvh::{ExactBvh, PolygonBounds};
pub use error::{HypermeshError, HypermeshResult};
pub use geometry::{Aabb, Classification, Plane, classify_point, classify_projective_point};
pub use hyperlattice::{Point3, Real, Vector3};
pub use intersection::{
    IntersectionSegment, OverlapInfo, PairwiseIntersection, PairwiseIntersectionType,
    intersect_polygons,
};
pub use local_bsp::{BspLeaf, LocalBsp};
pub use mesh::{
    InputMesh, MeshRef, OutputVertex, PolygonSoup, Triangle, prepare_input, prepare_input_meshes,
    prepare_input_refs,
};
pub use operations::{
    EmberConfig, boolean_difference, boolean_difference_refs, boolean_intersection,
    boolean_intersection_refs, boolean_operation, boolean_operation_refs, boolean_union,
    boolean_union_refs,
};
pub use output::{
    BooleanResult, ClassifiedPolygon, OutputPolygon, TriangleSoup, TriangleSoupClosureReport,
    extract_output, triangle_soup_closure_report, triangle_soup_is_closed,
    triangulate_and_resolve_certified,
};
pub use polygon::{ApproxBounds, ConvexPolygon, make_quad, make_triangle};
pub use segment_trace::{
    TraceAxisSegmentResult, classify_leaf_polygon, trace_axis_segment, trace_segment,
};
pub use subdivision::{
    DEFAULT_LEAF_THRESHOLD, DEFAULT_MAX_DEPTH, LeafProcessingStats, SubdivisionConfig,
    SubdivisionTask, process_leaf, process_leaf_into, subdivide, subdivide_into,
};
pub use winding::{
    BooleanOp, Indicator, WindingNumberTransitionVector, WindingNumberVector, WindingPair,
    classify_polygon_output, make_indicator, propagate_wnv,
};
