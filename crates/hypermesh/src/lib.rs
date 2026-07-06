//! Hyperreal-backed mesh boolean primitives.
//!
//! This crate keeps primitive coordinates at API boundaries only. Core
//! geometric state uses [`Real`] as its scalar and exposes borrowed slice APIs.
//!
//! The supported input model is finite, closed, piecewise-winding-number
//! triangle meshes represented with exact [`Real`] coordinates through
//! `hyperlattice::Point3`. Disconnected closed components and nested closed
//! components are part of that model. Empty meshes, degenerate source
//! triangles, open triangle soups, invalid triangle indices, and arbitrary
//! non-PWN surface collections are outside the supported model and are
//! rejected before the general boolean path runs.
//!
//! Predicate decisions are routed through the strict `hyperlimit` /
//! `hyperlattice` exact-predicate stack. Unsupported or uncertifiable
//! configurations are reported as explicit [`HypermeshError`] values rather
//! than being guessed with approximate topology. In particular, arbitrary
//! undecidable computable [`Real`] values remain outside any completeness claim
//! when strict bounded refinement cannot certify the sign or incidence fact
//! needed by subdivision, reference propagation, or leaf classification.
//!
//! By default, boolean operations run the general EMBER
//! subdivision/BSP/classification path; special-case boolean shortcuts are not
//! used to rescue uncertified general results. Public boolean operations
//! certify that the classified arrangement is already closed after exact
//! duplicate/T-junction resolution before returning a result. If subdivision
//! reaches its configured depth budget before a task is certified complete, the
//! operation fails with [`HypermeshError::SubdivisionDepthLimit`] instead of
//! guessing through the unfinished branch.
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
pub use mesh::{InputMesh, MeshRef, OutputVertex, PolygonSoup, Triangle, prepare_input};
pub use operations::{
    EmberConfig, boolean_difference, boolean_intersection, boolean_operation, boolean_union,
};
pub use output::{
    BooleanResult, OutputPolygon, TriangleSoup, TriangleSoupClosureReport, extract_output,
    triangle_soup_closure_report, triangle_soup_is_closed, triangulate_and_resolve_certified,
};
pub use polygon::{ApproxBounds, ConvexPolygon, make_quad, make_triangle};
pub use segment_trace::{
    TraceAxisSegmentResult, classify_leaf_polygon, trace_axis_segment, trace_segment,
};
pub use subdivision::{
    DEFAULT_MAX_DEPTH, LeafProcessingStats, SubdivisionConfig, SubdivisionTask, process_leaf,
    process_leaf_into, subdivide, subdivide_into,
};
pub use winding::{
    BooleanOp, Indicator, WindingNumberTransitionVector, WindingNumberVector, WindingPair,
    classify_polygon_output, make_indicator, propagate_wnv,
};
