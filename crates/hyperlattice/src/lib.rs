//! Real-centered linear algebra primitives backed by exact hyperreal arithmetic.
//!
//! `hyperlattice` exposes complex numbers, 2D/3D/4D vectors, and 3x3/4x4
//! matrices whose coordinate and scalar type is [`Real`]. Primitive
//! floating-point values are supported only at API edges: finite `f32`/`f64`
//! inputs are lifted through checked [`Real`] constructors, and
//! [`Real::to_f64_approx`] provides a named lossy export for rendering, IO, and
//! other external libraries.
//!
//! Most arithmetic that can fail returns [`BlasResult`]. Checked APIs use
//! [`ZeroStatus`] and reject both definite zero and unknown-zero divisors,
//! returning [`Problem::UnknownZero`] for the latter.
//!
//! # Examples
//!
//! ```
//! use hyperlattice::{Matrix3, Real, Vector3, sqrt};
//!
//! fn r(value: i32) -> Real {
//!     value.into()
//! }
//!
//! let v = Vector3::new([r(3), r(4), r(0)]);
//! assert_eq!(v.dot(&v), r(25));
//! assert_eq!(sqrt(v.dot(&v)).unwrap(), r(5));
//!
//! let identity = Matrix3::identity();
//! assert_eq!(identity * v.clone(), v);
//! ```

#![warn(missing_docs)]

pub use hyperreal::{
    DomainFacts as RealDomainFacts, DomainStatus as RealDomainStatus,
    ExpressionDegree as RealExpressionDegree, MagnitudeBits as RealMagnitudeBits, Rational,
    RationalStorageClass, Real, RealExactSetDenominatorKind, RealExactSetDyadicExponentClass,
    RealExactSetSignPattern, RealSign, RealStructuralFacts as RealFacts,
    SymbolicDependencyMask as RealSymbolicDependencyMask, ZeroKnowledge as ZeroStatus,
    ZeroOneMinusOneStatus as RealZeroOneMinusOneStatus,
};

mod trace;
pub(crate) use trace::trace_dispatch;

mod error;
pub use error::{AbortSignal, BlasResult, CheckedBlasResult, Problem};

mod kernels;
pub use kernels::ExactRealSetFacts;
pub(crate) use kernels::{ExactRationalKind, RealKernelExt};

mod algebra2;
pub use algebra2::{
    Displacement2Facts, Orient2Facts, ProductSum2Facts, ProductTerm2Facts, displacement2,
    displacement2_facts, dot2, orient2_expr, orient2_expr_facts, positive_product_sum2,
    product_sum2_facts, product_term2_facts, signed_product_sum2, squared_distance2, squared_norm2,
    wedge2,
};

mod scalar;
pub use scalar::{
    acos, acos_with_abort, acosh, acosh_with_abort, asin, asin_with_abort, asinh, asinh_with_abort,
    atan, atan_with_abort, atanh, atanh_with_abort, cos, cosh, e, exp, i, ln, log10,
    log10_with_abort, one, pi, pow, powi, reciprocal, reciprocal_checked,
    reciprocal_checked_with_abort, reciprocal_ref, reciprocal_ref_checked, sin, sinh, sqrt, tan,
    tanh, tau, zero, zero_status, zero_status_with_abort,
};

mod complex;
pub use complex::Complex;

mod vector;
pub use vector::{
    Axis2, SharedScaleVec, SignedAxis2, Vector2, Vector2Facts, Vector3, Vector3Facts, Vector4,
    Vector4Facts, Vector4HomogeneousKind, VectorSharedScaleView,
};

mod matrix;
pub use matrix::{
    Matrix3, Matrix3StructuralFacts, Matrix4, Matrix4StructuralFacts,
    MatrixDeterminantScheduleHint, PreparedMatrix3, PreparedMatrix4, PreparedRightDivisor3,
    PreparedRightDivisor4, SignedAxis4,
};

#[cfg(feature = "arbitrary")]
mod arbitrary_impls;
