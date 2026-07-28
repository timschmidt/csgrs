//! Fixed-size matrix support.
//!
//! The performance-sensitive implementation is kept in [`core`]. The sibling
//! modules document the semantic areas of that implementation so the matrix
//! directory remains navigable without moving hot kernels across more module
//! boundaries than necessary.

mod core;

mod batch;
mod determinant;
mod inverse;
mod ops;
mod transform;
mod types;

pub use core::{
    CachedMatrix3, CachedMatrix4, Matrix3, Matrix3StructuralFacts, Matrix3TransformKind, Matrix4,
    Matrix4StructuralFacts, Matrix4TransformKind, MatrixCacheState, MatrixDeterminantScheduleHint,
    RightDivisor3, RightDivisor4, SignedAxis4,
};
