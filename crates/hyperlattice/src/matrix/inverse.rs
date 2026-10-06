//! Matrix inverse and reciprocal kernels live in [`super::core`].
//!
//! The implementation keeps the fixed 3x3/4x4 adjugate paths close to the
//! determinant helpers because those paths intentionally share factors and
//! delayed scaling, especially when the structural determinant is already known
//! or when the overall matrix is structurally singular from the outset.
