//! Exact rational values and their shared arithmetic/conversion authority.
//!
//! [`crate::Rational`] stores a signed fraction with arbitrary-width integer
//! magnitudes. Its clones share exact storage and retained evidence; marked
//! internal unreduced values canonicalize when a consumer requires reduced parts.
//! Exact equality concerns values, independently of cache or allocation identity.
//!
//! [`arithmetic`] includes the representation, operations, predicates, text I/O
//! and queries in one implementation scope. [`convert`] supplies primitive
//! adapters; [`parse`] is a documentation anchor, not another parser. Public
//! arithmetic items are re-exported here for the crate facade. [`crate::Real`]
//! builds broader scalar representations on this rational layer.
//!
//! Exactness does not imply a uniform work or memory bound. Individual APIs state
//! their domains and failures: fraction construction and inversion return typed
//! zero-denominator errors, while the division operator panics on a zero divisor.
//! Finite float ingress preserves the binary value; lossy views and truncated
//! display are explicit output boundaries, not replacements for exact storage.

/// Own rational storage, exact operations, queries and included implementation fragments.
/// Text parsing and formatting share this module's canonicalization and scalar helpers.
mod arithmetic;
/// Adapt primitive integers and IEEE values without introducing another arithmetic kernel.
/// Checked integer export and nonfinite float rejection preserve distinct error causes.
mod convert;
/// Preserve a private documentation destination for text parsing and display.
/// The actual implementations remain included in the arithmetic module.
mod parse;

pub use arithmetic::*;
