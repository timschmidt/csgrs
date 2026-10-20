# Changelog

## [0.7.0] - Unreleased

### Breaking

- The scalar free functions are removed in favour of the native
  `hyperreal::Real` methods: the constants `zero`, `one`, `e`, `pi`, `tau`
  and `i`; `sqrt`, `exp`, `ln`, `log10`, `pow`, `powi`, `sin`, `cos`, `tan`,
  `sinh`, `cosh`, `tanh`, `asin`, `acos`, `atan`, `asinh`, `acosh`, `atanh`;
  `reciprocal`, `reciprocal_ref` and `zero_status`; and every `*_with_abort`
  variant of them. Use `Real::sqrt`, `Real::pi`, `Complex::i`,
  `Real::inverse`, `Real::inverse_ref`, `Real::powi_i64`,
  `Real::zero_status` and so on, and attach cancellation with `Real::abort`
  before the operation. `reciprocal_checked` and `reciprocal_ref_checked`
  remain because they reject unknown-zero divisors without refinement;
  `reciprocal_checked_with_abort` is removed.
- Re-exported Hyperreal types now use Hyperreal's own names: `ZeroStatus` is
  `ZeroKnowledge`, `RealFacts` is `RealStructuralFacts`, `RealDomainFacts`
  is `DomainFacts`, `RealDomainStatus` is `DomainStatus`,
  `RealExpressionDegree` is `ExpressionDegree`, `RealMagnitudeBits` is
  `MagnitudeBits`, `RealSymbolicDependencyMask` is `SymbolicDependencyMask`,
  `RealZeroOneMinusOneStatus` is `ZeroOneMinusOneStatus`, and
  `ExactRealSetFacts` is `RealExactSetFacts`. Public fields and return types
  of the `*Facts` structs change accordingly.
- The prepared-matrix API is replaced by immediate operations.
  `PreparedMatrix3`, `PreparedMatrix4`, `PreparedRightDivisor3`,
  `PreparedRightDivisor4`, `MatrixPreparedCacheState`, and
  `Matrix3`/`Matrix4::{prepare, prepare_right_divisor,
  div_matrix_with_prepared, div_matrix_checked_with_prepared,
  div_matrix_checked_with_prepared_with_abort}` (plus
  `Matrix4::div_exact_rational_matrix_with_prepared`) are removed; call
  `inverse`, `inverse_checked`, `div_matrix_checked` or
  `div_matrix_checked_with_abort` on the matrix directly.
- The transform-handle API is removed: `Matrix3::transform_vec3_handle`,
  `Matrix3::transform_vec3_with`, `Matrix4::transform_vec4_handle` and
  `Matrix4::transform_vec4_with`. Use `Matrix3::transform_vec3`,
  `Matrix3::transform_vec3_batch`, `Matrix4::transform_vec4`,
  `Matrix4::transform_vec4_batch` and the `transform_vec4_point*` and
  `transform_vec4_direction*` methods.
- `Matrix3`/`Matrix4::{upper,lower}_triangular_inverse_checked_with_abort`
  and `div_{upper,lower}_triangular_checked_with_abort` are removed. They
  never refined, so the corresponding `*_checked` methods are equivalent.
- `SharedScaleVec` no longer has public `exact` and `facts` fields; read them
  through `SharedScaleVec::facts()`, whose `exact` field holds the exact-set
  summary.
- `Complex::powi(0)` and `Complex::powi_checked(0)` return
  `Err(Problem::UnknownZero)` when the base is not proven nonzero. 0.6.0
  returned one unless the base was structurally known to be zero.
- `Real`, `Rational` and the other re-exported types come from Hyperreal
  0.14.

### Added

- `Complex::jacobi_sncndn` and `Complex::jacobi_epsilon` evaluate Jacobi's
  sn, cn, dn and epsilon at complex arguments for a real parameter `m < 1`,
  including negative parameters. Poles return `Problem::DivideByZero` and an
  unresolved denominator returns `Problem::UnknownZero`.
- `Complex::atanh` evaluates the principal inverse hyperbolic tangent off the
  real cuts `(-∞,-1]` and `[1,∞)`. The endpoints return `Problem::Infinity`,
  other cut points `Problem::NotANumber`, and undecided cut membership
  `Problem::Exhausted`.
- `Matrix3::transform_vec3`, `Matrix4::transform_vec4`,
  `Matrix4::transform_point3_batch` and `Matrix4::transform_direction3_batch`.
- A `serde` feature that derives `Serialize` and `Deserialize` for `Point2`
  and enables `hyperreal/serde`.

### Changed

- `Matrix4::rotation_axis_angle` with an axis along a coordinate axis (any
  nonzero length, either direction) now returns the same matrix as
  `rotation_x`, `rotation_y` or `rotation_z`, keeping the exact zero entries,
  instead of normalizing the axis.
- Exact scalar, vector, complex and matrix kernels were reworked for speed.
  Each retained change and its measurement is recorded in
  [`PERFORMANCE.md`](PERFORMANCE.md).
- Declares a minimum supported Rust version (MSRV) of 1.95.0; 0.6.0 declared
  none.
- Development moved to the
  [csgrs repository](https://github.com/timschmidt/csgrs), which the
  `repository` field now names.

For 0.6.0 and earlier, see [crates.io](https://crates.io/crates/hyperlattice/versions).
