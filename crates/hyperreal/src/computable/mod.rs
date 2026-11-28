//! Retained exact-real expressions, numerical evaluation and diagnostic formatting.
//!
//! [`node`] owns [`Computable`], shared node state, constructors, structural
//! recognition and the outer approximation/cache machinery. [`approximation`]
//! owns the retained expression enum and its integer evaluation kernels. The
//! files included by each facade share its namespace and private invariants.
//! [`mod@format`] implements decimal/scientific display; [`constants`] is a
//! documentation anchor, with actual constant construction and evaluation in
//! the two implementation namespaces. This module reexports the graph handle,
//! precision alias and crate-internal helpers without adding a second kernel.
//!
//! A retained recipe preserves exact input meaning; it is not a guarantee that
//! every native constructor validates its mathematical domain or that every
//! predicate/evaluation completes. [`crate::Real`] supplies checked modeling
//! entry points. Native precision requests are binary accuracy scales, not
//! total-work or memory budgets. Interrupted integer returns carry no completion
//! receipt; see [`Computable::approx_signal`] before consuming them as evidence.

mod approximation;
mod constants;
mod format;
mod node;

pub use node::Computable;
pub use node::Precision;
pub(crate) use node::{Signal, scale, scale_at_precision, shift, should_stop, signed, unsigned};
