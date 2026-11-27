//! Linear algebra and coordinate expressions over exact native Hyperreal scalars.
//!
//! [`Complex`], [`Vector2`]/[`Vector3`]/[`Vector4`], [`Matrix3`]/[`Matrix4`],
//! and Cartesian points store [`Real`] coordinates. Matrix storage is row-major
//! and transforms act on column vectors. Raw homogeneous point/line/plane carriers
//! and [`Aabb`] also accept degenerate data; constructing them does not establish
//! geometric validity, incidence, topology, or a selected intersection solution.
//!
//! Scalar arithmetic and domain decisions remain with Hyperreal. Fallible lattice
//! operations generally return [`BlasResult`]; [`CheckedBlasResult`] is the same
//! native Result type, with no extra validation or certainty receipt in the alias.
//! Checked entry points guard selected divisors using structural/cached
//! [`ZeroKnowledge`]. Where such a guard runs, zero gives [`Problem::DivideByZero`]
//! and unknown gives [`Problem::UnknownZero`] without starting refinement there.
//! Exact shortcuts and later native reciprocal operations have their own contracts.
//! [`AbortSignal`] is cooperative and path-dependent, not a deadline or total-work bound.
//!
//! Exact expressions remain authoritative without eager canonicalization. Native
//! scalar equality, and coordinatewise equality of these carriers, is structural;
//! unequal representations need not mean unequal mathematical values. Scheduling
//! facts about sparsity, shared denominators and transform kinds describe queried
//! coordinates, not proof of filled-set equality or topology. Publicly editable
//! fact packets are not self-validating and detached facts must be reconsidered
//! after input changes. This crate adds no geometry policy or provenance receipt.
//!
//! # Exact ingress and explicit projections
//!
//! Finite `f32`/`f64` inputs are imported as their exact binary values through checked
//! conversions; NaN and infinity fail. In particular, `0.1_f64` is not one tenth.
//! Use [`Rational::fraction`] for an exact ratio. Named lossy exports such as
//! [`Real::to_f64_lossy`] are intended for display and external numeric interfaces;
//! diagnostic formatting is not a lossless interchange contract.
//!
//! ```
//! use hyperlattice::{Rational, Real};
//!
//! let tenth = Real::new(Rational::fraction(1, 10).unwrap());
//! let binary_tenth = Real::try_from(0.1_f64).unwrap();
//! assert_ne!(binary_tenth, tenth); // Both are exact rational values.
//! assert!(Real::try_from(f64::NAN).is_err());
//! ```
//!
//! # Features
//!
//! No optional features are enabled by default. Hyperreal remains a required
//! dependency; this facade is not the companion's primitive `no_std` build.
//!
//! | Feature | Effect |
//! | --- | --- |
//! | `arbitrary` | Fuzz generation for Complex, Vector2/3/4 and Matrix3/4 using finite binary64 or integer seeds; nonfinite float seeds are sanitized to zero. |
//! | `serde` | Enables native Real serialization and Point2 field derives; forwards Serde to the interval companion only when that dependency is enabled. It does not add serialization to every lattice carrier. |
//! | `hyperreal-dispatch-trace` | Enables forwarding to Hyperreal's opt-in, thread-local recorder; compiling the feature alone does not start recording. |
//! | `interval` | Reexports the companion's generic interval/affine carriers and error/backend traits, without selecting a scalar backend. |
//! | `interval-fixed` | Enables `interval` and the companion's fixed-point backend. |
//! | `interval-dyadic` | Enables `interval` and the companion's budgeted arbitrary-precision dyadic backend. |
//! | `interval-rational` | Enables `interval` and exact rational interval arithmetic. |
//! | `interval-hyperreal` | Enables `interval` and Hyperreal enclosure integration, including the companion's rational and dyadic dependencies. |
//! | `interval-float-enclosure` | Enables `interval` and primitive floating enclosures, subject to Hyperinterval's environment qualification. |
//!
//! Interval vectors/matrices denote Cartesian products; their arithmetic returns
//! enclosures. Retained affine-image carriers preserve source/map dependence for
//! further affine composition. Use the companion package directly for its primitive
//! allocation-free `no_std` configuration; rational/Hyperreal backends have different
//! dependency and resource contracts. Backend details belong to the companion docs.
//!
//! Private implementation documentation can be generated from the workspace with
//! `cargo doc -p hyperlattice --all-features --no-deps --document-private-items`.
//!
//! # Examples
//!
//! ```
//! use hyperlattice::{Matrix3, Real, Vector3};
//!
//! /// Import the integer fixture without a floating-point intermediate.
//! fn r(value: i32) -> Real {
//!     value.into()
//! }
//!
//! let v = Vector3::new([r(3), r(4), r(0)]);
//! assert_eq!(v.dot(&v), r(25));
//! assert_eq!(Real::sqrt(v.dot(&v)).unwrap(), r(5));
//!
//! let identity = Matrix3::identity();
//! assert_eq!(identity * v.clone(), v);
//! ```

#![warn(missing_docs)]

/// The companion's Cartesian interval algebra and retained affine-image carriers.
///
/// This reexport preserves type identity and backend contracts. Depend directly on
/// the companion for its primitive allocation-free no_std configuration; enabling
/// this facade feature retains Hyperlattice's required Real/std dependencies.
#[cfg(feature = "interval")]
pub use hyperlattice_interval as interval;
#[cfg(feature = "interval")]
pub use hyperlattice_interval::{
    AffineBox, AffineChain, AffineImage, AffineMap, AffineSource, BoxN, CenteredMatrix,
    CenteredVector, IntervalArithmetic, IntervalLatticeError, IntervalMatrix, IntervalVector,
    MidpointRadiusMatrix, MidpointRadiusVector, TranslatedBox,
};

pub use hyperreal::{
    DomainFacts, DomainStatus, ExpressionDegree, MagnitudeBits, Rational, RationalStorageClass,
    Real, RealExactSetDenominatorKind, RealExactSetDyadicExponentClass, RealExactSetFacts,
    RealExactSetSignPattern, RealSign, RealStructuralFacts, SymbolicDependencyMask, ZeroKnowledge,
    ZeroOneMinusOneStatus,
};

/// Feature-gated forwarding of lattice dispatch events to the native scalar recorder.
mod trace;
pub(crate) use trace::trace_dispatch;

/// Native scalar result/error aliases and a shared cooperative cancellation handle.
mod error;
pub use error::{AbortSignal, BlasResult, CheckedBlasResult, Problem};

/// Private arithmetic adapters and rational-storage dispatch, retaining scalar-owned authority.
mod kernels;
pub(crate) use kernels::{ExactRationalKind, RealKernelExt};

/// Exact planar expressions and conservative support facts, without geometric sign decisions.
mod algebra2;
pub use algebra2::{
    Displacement2Facts, Orient2Facts, ProductSum2Facts, ProductTerm2Facts, displacement2,
    displacement2_facts, dot2, orient2_expr, orient2_expr_facts, positive_product_sum2,
    product_sum2_facts, product_term2_facts, signed_product_sum2, squared_distance2, squared_norm2,
    wedge2,
};

/// Structural divisor guards and cooperative abort attachment used by lattice arithmetic.
mod scalar;
pub use scalar::{reciprocal_checked, reciprocal_ref_checked};

/// Complex expression pairs, arithmetic and elementary/Jacobi continuation over native Reals.
mod complex;
pub use complex::Complex;

/// Fixed-size vector arithmetic, structural metadata and equal-denominator coordinate views.
mod vector;
pub use vector::{
    Axis2, SharedScaleVec, SignedAxis2, Vector2, Vector2Facts, Vector3, Vector3Facts, Vector4,
    Vector4Facts, Vector4HomogeneousKind, VectorSharedScaleFacts, VectorSharedScaleView,
};

/// Cartesian point carriers, vector conversions and borrowed coordinate-fact snapshots.
mod point;
pub use point::{
    Point2, Point2Facts, Point3, Point3Facts, PointSharedScaleFacts, PointSharedScaleView,
};

/// Raw minimum/maximum corner storage without ordering, emptiness or containment validation.
mod aabb;
pub use aabb::Aabb;

/// Unnormalized homogeneous carriers and division-free intersection expressions.
mod projective;
pub use projective::{
    HomogeneousLine3, HomogeneousPoint3, Plane3Coefficients, ProjectivePlane3,
    homogeneous_point_plane_expression, intersect_homogeneous_line_plane, intersect_three_planes,
    intersect_two_planes,
};

/// Row-major fixed-size matrices, arithmetic dispatch and column-vector transforms.
mod matrix;
pub use matrix::{
    Matrix3, Matrix3StructuralFacts, Matrix3TransformKind, Matrix4, Matrix4StructuralFacts,
    Matrix4TransformKind, MatrixDeterminantScheduleHint, SignedAxis4,
};

/// Optional fuzz generators for complex/vector/matrix carriers using sanitized numeric seeds.
#[cfg(feature = "arbitrary")]
mod arbitrary_impls;
