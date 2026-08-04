#![allow(clippy::too_many_arguments, clippy::type_complexity)]
#![allow(clippy::arc_with_non_send_sync)]

//! Exact `hyperreal::Real` triangle-mesh geometry and Boolean operations.
//!
//! [`boolean`] validates a non-empty slice of finite closed
//! piecewise-winding-number mesh views, constructs one exact source-face and
//! radial-cell arrangement, evaluates a [`BooleanProgram`], and returns a
//! closure-certified [`BooleanMeshBatch`]. All requested roots share the same
//! compact exact vertex arena and do not repeat corefinement.
//!
//! Every predicate-bearing API receives an immutable [`MeshContext`].
//! [`PredicatePolicy::STRICT`] consumes only certified decisions;
//! [`PredicatePolicy::APPROXIMATE_512`] may consume Hyperlimit's deterministic
//! 512-bit terminal decision. Successful [`MeshOutcome`] values preserve that
//! distinction through aggregate [`MeshCertainty`].
//!
//! Boolean output is selected directly from exact cell truth. Degenerate,
//! duplicate, open, or directionally unbalanced output is rejected rather than
//! repaired. Expressions containing the exterior cell remain representable as
//! oriented finite boundaries and are marked by
//! [`BooleanMeshResult::exterior_inside`]; converting such a result to a finite
//! [`TriangleMesh`] is a typed error.

#![deny(dead_code)]
#![warn(missing_docs)]

mod trace;
pub(crate) use trace::trace_dispatch;
mod boolean;
pub mod context;
mod storage_hash;
mod surface_arrangement;
#[cfg(test)]
mod test_support;

pub mod bvh;
pub mod clip;
pub mod convex_hull;
pub mod error;
pub mod geometry;
pub mod gpu;
pub mod intersection;
pub mod mesh;
pub mod output;
mod point_interner;
pub mod polygon;
mod predicate;
pub mod winding;

pub use boolean::boolean;
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
    IntersectionPoint, IntersectionSegment, OverlapInfo, PairwiseIntersection, intersect_polygons,
};
pub use mesh::{
    PolygonSoup, Triangle, TriangleMesh, TriangleMeshRef, certify_convex_mesh, polygon_soup,
};
pub use output::{
    BooleanMeshBatch, BooleanMeshClosureEvidence, BooleanMeshResult, TriangleSource,
    boolean_mesh_closure_evidence, boolean_mesh_is_closed,
};
pub use polygon::{ApproxBounds, ConvexPolygon, convex_quad, convex_triangle};
pub use winding::{
    BooleanExpression, BooleanOp, BooleanProgram, WindingNumberTransitionVector,
    WindingNumberVector, WindingPair, classify_polygon_output, propagate_wnv,
};
