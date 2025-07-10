//! Mesh smoothing algorithms.

pub mod serial;
pub mod traits;

#[cfg(feature = "parallel")]
pub mod parallel;

// Re-export core types
pub use traits::SmoothingOps;

#[cfg(not(feature = "parallel"))]
pub use serial::SerialSmoothingOps;

#[cfg(feature = "parallel")]
pub use parallel::ParallelSmoothingOps;
