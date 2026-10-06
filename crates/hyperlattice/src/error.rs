//! Error and cancellation types shared by lattice operations.

use std::sync::Arc;
use std::sync::atomic::AtomicBool;

/// Shared cancellation signal used by abort-aware APIs.
///
/// The signal is attached to cloned [`hyperreal::Real`] values before
/// operations that may evaluate opaque computable reals.
pub type AbortSignal = Arc<AtomicBool>;

pub use hyperreal::Problem;

/// Result type used by fallible operations in this crate.
pub type BlasResult<T> = Result<T, Problem>;

/// Result type used by APIs that reject unknown-zero conditions.
pub type CheckedBlasResult<T> = BlasResult<T>;
