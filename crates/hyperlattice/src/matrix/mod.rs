//! Fixed-size matrix support.
//!
//! The performance-sensitive implementation is kept in [`core`]. The sibling
//! modules document the semantic areas of that implementation so the matrix
//! directory remains navigable without moving hot kernels across more module
//! boundaries than necessary.

/// Matrix storage, structural facts, arithmetic, and transform implementation.
mod core;

/// Navigation notes for immediate batch operations implemented in `core`.
mod batch;
/// Navigation notes for determinant kernels implemented in `core`.
mod determinant;
/// Navigation notes for inverse kernels implemented in `core`.
mod inverse;
/// Navigation notes for matrix operators implemented in `core`.
mod ops;
/// Navigation notes for matrix-vector transforms implemented in `core`.
mod transform;
/// Navigation notes for row-major matrix types implemented in `core`.
mod types;

pub use core::{
    Matrix3, Matrix3StructuralFacts, Matrix3TransformKind, Matrix4, Matrix4StructuralFacts,
    Matrix4TransformKind, MatrixDeterminantScheduleHint, SignedAxis4,
};
