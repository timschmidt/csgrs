#![allow(clippy::too_many_arguments, clippy::type_complexity)]
#![allow(clippy::arc_with_non_send_sync)]

//! Hyperreal-backed mesh boolean primitives.
//!
//! This crate keeps primitive coordinates at API boundaries only. Core
//! geometric state uses [`Real`] as its scalar and exposes borrowed slice APIs.
//!
//! The intended input model is finite, closed, piecewise-winding-number
//! triangle meshes represented with exact [`Real`] coordinates through
//! `hyperlattice::Point3`. Disconnected closed components and nested closed
//! components are part of that model. Empty meshes, degenerate source
//! triangles, open triangle soups, signed edge boundaries, invalid triangle
//! indices, and arbitrary non-PWN surface collections are outside the
//! supported model and are rejected before the general boolean path runs.
//!
//! Completeness is claimed for that finite closed-PWN model when every strict
//! exact predicate required by the operation is decidable under the strict
//! bounded refinement policy:
//!
//! - If a boolean operation returns [`BooleanResult`], the classified
//!   arrangement and its winding data were certified by the general EMBER
//!   subdivision/BSP/classification path; the public API does not rely on
//!   special-case boolean shortcuts or output repair to turn an uncertified
//!   branch into success.
//! - If the current search cannot certify a required sign, incidence,
//!   reachability, reference-propagation step, leaf classification, or output
//!   closure fact, the operation returns an explicit [`HypermeshError`] such as
//!   [`HypermeshError::UnknownClassification`],
//!   [`HypermeshError::ReferencePropagationFailed`], or
//!   [`HypermeshError::SubdivisionDepthLimit`] instead of guessing through the
//!   unresolved branch.
//! - Reference propagation and leaf classification exhaust finite exact
//!   support-plane arrangements, canonical strict cell witnesses, retained
//!   plane-replacement orderings, and bounded detour cells. They do not rely on
//!   random or finite candidate sampling for completeness.
//!
//! Predicate decisions are routed through the strict `hyperlimit` /
//! `hyperlattice` exact-predicate stack. Unsupported or uncertifiable
//! configurations are reported as explicit [`HypermeshError`] values rather
//! than being guessed with approximate topology. In particular, arbitrary
//! undecidable computable [`Real`] values are outside this completeness boundary
//! when strict bounded refinement cannot certify the sign, incidence, or
//! ordering fact needed by subdivision, reference propagation, or leaf
//! classification. An explicitly configured finite subdivision depth remains a
//! caller-selected certification budget, not part of the completeness claim.
//!
//! By default, boolean operations run the general EMBER
//! subdivision/BSP/classification path; special-case boolean shortcuts are not
//! used to rescue uncertified general results. Public boolean operations
//! certify that the classified polygon arrangement has no singleton edges and
//! has exact forward/reverse edge cancellation before duplicate/T-junction
//! triangulation cleanup runs. Open or directionally unbalanced arrangements
//! are rejected rather than repaired. If subdivision
//! reaches an explicitly configured finite depth budget before a task is
//! certified complete, the operation fails with
//! [`HypermeshError::SubdivisionDepthLimit`] instead of guessing through the
//! unfinished branch. Default configurations have no arbitrary depth cap;
//! their subdivision branches terminate by exhausting the finite root split
//! basis.
//!
//! Use [`triangulate_and_resolve_certified`] to triangulate a boolean result
//! while preserving the invariant that open or zero-volume output is rejected
//! rather than repaired. Use [`certify_output_polygon_closure`] to validate
//! that invariant directly on the classified polygon arrangement before any
//! triangulation cleanup runs.

#![deny(dead_code)]
#![warn(missing_docs)]

mod trace;
pub(crate) use trace::trace_dispatch;

pub mod bvh;
pub mod clip;
pub mod convex_hull;
pub mod error;
pub mod geometry;
mod halfspace;
pub mod intersection;
pub mod local_bsp;
pub mod mesh;
pub mod operations;
pub mod output;
pub mod polygon;
mod predicate;
pub mod segment_trace;
pub mod subdivision;
pub mod winding;

pub use bvh::{ExactBvh, ExactPointBvh, PolygonBounds};
pub use convex_hull::{
    convex_hull, convex_hull_with_coplanar_groups, convex_hull_with_retained_facts,
};
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
    BooleanResult, OutputPolygon, TriangleSoup, TriangleSoupClosureReport, TriangleSource,
    certify_output_polygon_closure, extract_output, triangle_soup_closure_report,
    triangle_soup_is_closed, triangulate_and_resolve_certified,
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
