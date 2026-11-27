//! Determinant kernels live in [`crate::matrix::core`].
//!
//! The implementation uses fixed-size cofactor formulas and signed product
//! reductions so exact Real kernels can defer normalization and approximation.
