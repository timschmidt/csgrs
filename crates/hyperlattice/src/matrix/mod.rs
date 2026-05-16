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
    Matrix3, Matrix3StructuralFacts, Matrix3TransformKind, Matrix4, Matrix4StructuralFacts,
    Matrix4TransformKind, MatrixDeterminantScheduleHint, MatrixPreparedCacheState, PreparedMatrix3,
    PreparedMatrix4, PreparedRightDivisor3, PreparedRightDivisor4, SignedAxis4,
};
