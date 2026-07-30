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
//! Every predicate-bearing API receives an immutable [`MeshContext`]. The
//! context selects either [`PredicatePolicy::STRICT`], which consumes only
//! certified decisions, or [`PredicatePolicy::APPROXIMATE_512`], which permits
//! Hyperlimit's deterministic 512-bit terminal equality/sign interpretation
//! after certification is exhausted. Successful operations return a
//! [`MeshOutcome`] whose aggregate [`MeshCertainty`] reports whether that
//! terminal was consumed.
//!
//! Completeness is claimed for the finite closed-PWN model under the selected
//! predicate policy:
//!
//! - If a general arrangement operation returns [`BooleanResult`], the classified
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
//! Predicate decisions are routed through one operation-local adapter into
//! `hyperlimit`; no global or hidden default selects topology semantics.
//! Under `STRICT`, unsupported or uncertifiable configurations are reported as
//! explicit [`HypermeshError`] values. Under `APPROXIMATE_512`, a result that
//! consumes the policy-authorized terminal remains `Real`-backed and is marked
//! [`MeshCertainty::Approximate512Consumed`] rather than relabeled as strictly
//! certified. An explicitly configured finite subdivision depth remains a
//! caller-selected certification budget, not part of the completeness claim.
//!
//! By default, boolean operations run the general EMBER
//! subdivision/BSP/classification path; special-case boolean shortcuts are not
//! used to rescue uncertified general results. The reusable carrier-level
//! [`boolean_triangle_meshes`] entry point may resolve exact algebraic cases
//! before invoking that path. General arrangement operations
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
//! triangulation cleanup runs. [`BooleanMesh::try_to_gpu_mesh_f32`] is the
//! explicit approximation boundary for backend-neutral finite `f32` position,
//! normal, and index buffers; the parallel
//! [`BooleanMesh::try_to_gpu_mesh_f64`] adapter retains binary64 precision.

#![deny(dead_code)]
#![warn(missing_docs)]

mod trace;
pub(crate) use trace::trace_dispatch;
pub mod context;
mod storage_hash;
#[cfg(test)]
mod test_support;

pub mod bvh;
pub mod clip;
pub mod convex_hull;
pub mod error;
pub mod geometry;
pub mod gpu;
mod halfspace;
pub mod intersection;
mod local_bsp;
pub mod mesh;
pub mod operations;
pub mod output;
pub mod polygon;
mod predicate;
pub mod segment_trace;
pub mod subdivision;
pub mod winding;

pub use bvh::{ExactBvh, ExactPointBvh, PolygonBounds};
pub use context::{MeshCertainty, MeshContext, MeshOutcome};
pub use convex_hull::{
    convex_hull, convex_hull_with_coplanar_groups, convex_hull_with_retained_facts,
};
pub use error::{HypermeshError, HypermeshResult};
pub use geometry::{Aabb, Classification, Plane, classify_point, classify_projective_point};
pub use gpu::{
    ExactGpuMeshBuffers, ExactGpuVertex, GpuMeshBuffersF32, GpuMeshBuffersF64, GpuMeshError,
    GpuVertexAttribute, InterleavedGpuMeshBuffersF32, InterleavedGpuMeshBuffersF64,
    approximate_gpu_mesh_f32, approximate_gpu_mesh_f32_or_zero, approximate_gpu_mesh_f64,
    approximate_gpu_mesh_f64_or_zero, approximate_interleaved_gpu_mesh_f32,
    approximate_interleaved_gpu_mesh_f64,
};
pub use hyperlattice::{Point3, Real, Vector3};
pub use hyperlimit::PredicatePolicy;
pub use intersection::{
    IntersectionSegment, OverlapInfo, PairwiseIntersection, PairwiseIntersectionType,
    intersect_polygons,
};
pub use mesh::{
    OutputVertex, PolygonSoup, Triangle, TriangleMesh, TriangleMeshRef, certify_convex_mesh,
    polygon_soup,
};
pub use operations::{EmberConfig, boolean_mesh, boolean_operation, boolean_triangle_meshes};
pub use output::{
    BooleanMesh, BooleanMeshClosureEvidence, BooleanResult, OutputPolygon, TriangleSource,
    boolean_mesh_closure_evidence, boolean_mesh_is_closed, certify_output_polygon_closure,
    extract_output, triangulate_and_resolve_certified,
};
pub use polygon::{ApproxBounds, ConvexPolygon, convex_quad, convex_triangle};
pub use segment_trace::{
    TraceAxisSegmentResult, classify_leaf_polygon, trace_axis_segment, trace_segment,
};
pub use subdivision::{
    DEFAULT_MAX_DEPTH, LeafProcessingStats, SubdivisionConfig, SubdivisionTask, process_leaf,
    process_leaf_into, subdivide, subdivide_into,
};
pub use winding::{
    BooleanOp, WindingNumberTransitionVector, WindingNumberVector, WindingPair,
    classify_polygon_output, propagate_wnv,
};
