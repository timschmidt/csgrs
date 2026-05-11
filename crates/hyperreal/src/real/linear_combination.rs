//! Hot-path linear-combination and transform-style helpers for [`Real`].
//!
//! The implementations currently live in [`super::arithmetic`] near the exact
//! rational and symbolic constructors they reuse. Dot products and affine
//! combinations deliberately reduce symbolic structure before approximation and
//! share exact-rational denominators where possible.
