//! Lazy exact-real expression graph and approximation kernels.
//!
//! The implementation is intentionally split by semantic pressure rather than
//! by one node type per file. `node` owns the compact expression enum, caches,
//! structural facts, and graph rewrites because those pieces are tightly coupled
//! in hot constructors. `approximation` owns the precision-refinement kernels
//! for elementary functions. The smaller sibling modules are documentation
//! anchors for planned split points where moving code has not yet measured
//! neutral.

mod approximation;
mod budget;
mod constants;
mod format;
mod node;

pub use budget::EvaluationBudget;
pub use node::Computable;
pub use node::Precision;
pub(crate) use node::{
    Signal, checked_approximation_precision, evaluation_should_stop, precision_from_i64, scale,
    scale_at_precision_ref, shift, signed, unsigned,
};
