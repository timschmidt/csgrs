//! Geometry-oriented robust predicates with structural scalar awareness.
//!
//! `predicated` is intentionally positioned between scalar semantics and
//! application geometry code. It asks backends for facts such as known sign,
//! exact zero, magnitude bounds, and refinement capability before falling back
//! to generic robust predicate machinery.

pub mod backend;
pub mod classify;
pub mod error;
pub mod filter;
pub mod orient;
pub mod plane;
pub mod predicate;
mod resolve;
pub mod scalar;

pub use classify::{LineSide, PlaneSide};
pub use orient::{Point2, Point3, classify_point_line, incircle2d, insphere3d, orient2d, orient3d};
pub use plane::{Plane3, classify_point_oriented_plane, classify_point_plane};
pub use predicate::{
    Certainty, Escalation, PredicateOutcome, PredicatePolicy, Sign, SignKnowledge,
};
pub use scalar::{MagnitudeBounds, PredicateScalar, ScalarFacts, StructuralScalar};
