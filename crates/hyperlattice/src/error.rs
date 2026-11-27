//! Native scalar error/result aliases and a shared cooperative cancellation flag.
//!
//! Problem is re-exported unchanged from Hyperreal. The result aliases add neither
//! predicate certainty nor validation. AbortSignal shares a flag; propagation and
//! polling remain responsibilities of the operations that attach and consume it.

use std::sync::Arc;
use std::sync::atomic::AtomicBool;

/// Shared AtomicBool handle passed to abort-aware lattice APIs.
///
/// Cloning the Arc shares the same flag. Helpers attach it to Real computable
/// payloads; this alias itself performs no polling. Setting the flag does not force
/// every exact/structural shortcut to stop, bound nested work, or roll back effects.
pub type AbortSignal = Arc<AtomicBool>;

pub use hyperreal::Problem;

/// Native Result<T, hyperreal::Problem> for fallible lattice arithmetic; no extra error wrapper or certainty receipt is introduced.
pub type BlasResult<T> = Result<T, Problem>;

/// Naming alias for BlasResult used by structural-domain-checked APIs.
///
/// It is the same Rust type, not a stronger result carrier. The called function,
/// not this alias, determines whether unknown-zero inputs are rejected.
pub type CheckedBlasResult<T> = BlasResult<T>;
