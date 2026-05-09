//! Geometry-oriented robust predicates with structural scalar awareness.
//!
//! `liminal` is intentionally positioned between scalar semantics and
//! application geometry code. It asks backends for facts such as known sign,
//! exact zero, magnitude bounds, and refinement capability before falling back
//! to generic robust predicate machinery.

#[cfg(feature = "dispatch-trace")]
macro_rules! trace_dispatch {
    ($layer:expr, $operation:expr, $path:expr) => {
        ::hyperreal::dispatch_trace::record($layer, $operation, $path);
    };
}

#[cfg(not(feature = "dispatch-trace"))]
macro_rules! trace_dispatch {
    ($layer:expr, $operation:expr, $path:expr) => {};
}

pub(crate) use trace_dispatch;

pub mod backend;
pub mod batch;
pub mod classify;
pub mod error;
pub mod filter;
pub mod orient;
pub mod plane;
pub mod predicate;
mod resolve;
pub mod scalar;

pub use batch::{
    Incircle2dCase, Insphere3dCase, Orient2dCase, Orient3dCase, PointPlaneCase,
    classify_point_line_batch, classify_point_line_batch_with_policy,
    classify_point_oriented_plane_batch, classify_point_oriented_plane_batch_with_policy,
    classify_point_plane_batch, classify_point_plane_batch_with_policy, incircle2d_batch,
    incircle2d_batch_with_policy, insphere3d_batch, insphere3d_batch_with_policy, orient2d_batch,
    orient2d_batch_with_policy, orient3d_batch, orient3d_batch_with_policy,
};
#[cfg(feature = "parallel")]
pub use batch::{
    classify_point_line_batch_parallel, classify_point_line_batch_parallel_with_policy,
    classify_point_oriented_plane_batch_parallel,
    classify_point_oriented_plane_batch_parallel_with_policy, classify_point_plane_batch_parallel,
    classify_point_plane_batch_parallel_with_policy, incircle2d_batch_parallel,
    incircle2d_batch_parallel_with_policy, insphere3d_batch_parallel,
    insphere3d_batch_parallel_with_policy, orient2d_batch_parallel,
    orient2d_batch_parallel_with_policy, orient3d_batch_parallel,
    orient3d_batch_parallel_with_policy,
};
pub use classify::{LineSide, PlaneSide};
pub use orient::{Point2, Point3, classify_point_line, incircle2d, insphere3d, orient2d, orient3d};
pub use plane::{Plane3, classify_point_oriented_plane, classify_point_plane};
pub use predicate::{
    Certainty, Escalation, PredicateOutcome, PredicatePolicy, Sign, SignKnowledge,
};
pub use scalar::{
    BorrowedPredicateScalar, MagnitudeBounds, PredicateScalar, ScalarFacts, StructuralScalar,
};
