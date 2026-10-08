//! Constructor-focused helpers for [`Real`].
//!
//! The current constructor behavior remains in `arithmetic.rs`; this module
//! keeps the requested split layout.
//! Public and internal `Real` constructors live in [`super::arithmetic`].
//!
//! Constructor simplification is intentionally close to arithmetic rewrites and
//! structural facts. Moving it out before the representation settles risks
//! hiding performance-sensitive invariants that keep exact forms symbolic.
