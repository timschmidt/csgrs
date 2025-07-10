//! Mesh convex hull algorithms.

pub mod serial;
pub mod traits;

#[cfg(feature = "parallel")]
pub mod parallel;

// Re-export core types
pub use traits::ConvexHullOps;

#[cfg(not(feature = "parallel"))]
pub use serial::SerialConvexHullOps;

#[cfg(feature = "parallel")]
pub use parallel::ParallelConvexHullOps;
