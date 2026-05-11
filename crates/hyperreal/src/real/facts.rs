//! Structural fact and zero/sign queries for [`Real`].
//!
//! The implementations currently live in [`super::arithmetic`] because fact
//! queries read the same representation invariants that arithmetic constructors
//! maintain. Keeping them adjacent avoids accidental approximation and keeps
//! predicate-only callers on cheap symbolic paths.
