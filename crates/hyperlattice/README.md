<h1>
  hyperlattice
  <img src="./doc/hyperlattice.png" alt="Hyper, a clever mathematician" width="144" align="right">
</h1>

`hyperlattice` provides small fixed-size linear algebra over
`hyperreal::Real`: complex numbers, 2D/3D/4D vectors, points, 3×3 and 4×4
matrices, affine and projective transforms, and reusable structural facts.

It is the carrier layer between Hyperreal scalars and geometry predicates. It
does not try to replace a general BLAS package, classify geometry, or own mesh
topology.

## Why exact-aware linear algebra?

Small matrices and vectors appear at nearly every geometry branch point.
Floating-point linear algebra can hide a singular pivot or turn a zero
determinant into a small nonzero value. Fully expanding exact expressions, on
the other hand, can do expensive work before a caller knows whether a zero
mask, transform kind, or shared scale already answers the scheduling question.

Hyperlattice retains that object-level structure:

```text
hyperreal::Real coordinates
           │
           ▼
 point / vector / matrix carriers
           │
   sparse support, zero masks,
   shared scales, transform kinds
           │
           ├──────────► exact algebra result
           └──────────► facts for Hyperlimit predicates
```

Facts are conservative scheduling evidence. A topology-changing decision still
belongs to an exact or explicitly uncertain predicate.

## Primary types

| Type | Purpose |
| --- | --- |
| `Complex` | Exact complex arithmetic, integer powers and Jacobi sn/cn/dn and epsilon for real m < 1. |
| `Vector2`, `Vector3`, `Vector4` | Fixed-size exact vectors with dot, norm, interpolation, and checked normalization operations. |
| `Point2`, `Point3` | Affine points kept distinct from displacement vectors. |
| `Matrix3`, `Matrix4` | Fixed-size exact matrices, transforms, determinants, and checked inverses. |
| `ProjectivePlane3` | Plane coefficients for projective intersection construction. |
| `HomogeneousPoint3`, `HomogeneousLine3` | Exact projective results that delay affine division. |
| `SharedScaleVec`, shared-scale views | Original rational coordinates with retained common-denominator facts; no rescaling is performed. |
| `*Facts`, transform-kind, and schedule types | Reusable zero, support, homogeneous, exact-set, and determinant metadata. |
| `BlasResult<T>`, `Problem`, `AbortSignal` | Checked failures and cooperative cancellation. |

`Rational`, `Real`, and their principal structural types are re-exported from
Hyperreal for convenience.

The workspace's `Complex::jacobi_sncndn(&m)` continues native real Jacobi
functions to complex arguments, including negative real parameters. Known
poles fail with `DivideByZero`; an unproved denominator fails with `UnknownZero`.
The components retain `Real` expressions and their scalar relations. Complex dn
can vanish and has no real-axis positivity restriction. See the
[admission](docs/admission-complex-jacobi.md) for the domain and verification
contract. These APIs are included in the checkout's unreleased 0.7.0 changes.

## Quick start

This checkout declares hyperlattice 0.7.0 and requires Rust 1.95 or newer.
The examples below are checked in; from the root of a
[CSGRS source checkout](../../readme.md), run:

```sh
cargo run -p hyperlattice --example readme_quickstart
```

The checked-in example is:

<!-- quickstart:start -->
```rust
//! Runnable exact-integer vector and identity-matrix quick start.
//!
//! The README contains this whole file verbatim between its quick-start markers;
//! the integration test checks trimmed text equality. The four assertions exercise
//! one rational fixture, not arbitrary symbolic equality or predicate certainty.

use hyperlattice::{Matrix3, Real, Vector3};

/// Import a signed 32-bit integer exactly into Real, without a floating-point conversion.
fn r(value: i32) -> Real {
    value.into()
}

/// Check squared length 25, principal length 5, and identity action/inversion.
///
/// Builds (3, 4, 0), checks its self-dot product and square root, then checks a 3x3
/// identity matrix acting on it and inverting to itself. Assertions use structural
/// PartialEq for these exact rational values. Native scalar errors propagate via
/// BlasResult; this example adds no policy or certainty receipt and prints nothing.
fn main() -> hyperlattice::BlasResult<()> {
    let vector = Vector3::new([r(3), r(4), r(0)]);
    assert_eq!(vector.dot(&vector), r(25));
    assert_eq!(Real::sqrt(vector.dot(&vector))?, r(5));

    let identity = Matrix3::identity();
    assert_eq!(identity.clone() * vector.clone(), vector);
    assert_eq!(identity.inverse()?, Matrix3::identity());
    Ok(())
}
```
<!-- quickstart:end -->

The same source is checked in as
[`examples/readme_quickstart.rs`](examples/readme_quickstart.rs), compiled by
the test suite, and compared with the README block.

## API guide

`Complex::jacobi_epsilon(&m)` supplies the meromorphic primitive of `dn²`
that vanishes at zero. It preserves additive quasi-periods and supports the
same real parameter domain and pole diagnostics as the triplet. Its argument
is the Jacobi argument; the real-axis implementation uses
`Real::jacobi_epsilon`, composing Legendre E with the continuous amplitude.
See the [epsilon admission](docs/admission-jacobi-epsilon.md) for the native
construction and independent development oracle.

`Complex::atanh()` evaluates the principal analytic inverse hyperbolic tangent.
It excludes the real cuts outside (-1,1), reports the poles as `Infinity`,
and keeps unresolved domain decisions typed. Components remain native Real
expressions. The [atanh admission](docs/admission-complex-atanh.md) records
independent series and reciprocal references, cut-bank tests, and the
[checked scalar quadrant extension](../hyperreal/docs/admission-atan2-right-half-plane.md).

### Scalar and complex operations

| Task | API |
| --- | --- |
| Constants | `Real::zero`, `Real::one`, `Real::e`, `Real::pi`, `Real::tau`, `Complex::i` |
| Zero knowledge | `Real::zero_status`, returning `ZeroKnowledge` without refinement |
| Reciprocal and powers | `Real::inverse`, `Real::inverse_ref`, `Real::pow`, `Real::powi_i64` |
| Structural checked reciprocal | `reciprocal_checked`, `reciprocal_ref_checked` |
| Elementary functions | Native `Real` methods: `sqrt`, `exp`, `ln`, `log10`, `sin`, `cos`, `tan`, `sinh`, `cosh`, `tanh` |
| Inverse functions | Native `Real` methods: `asin`, `acos`, `atan`, `asinh`, `acosh`, `atanh` |
| Complex values | `Complex::new`, `zero`, `one`, `i`, `conjugate`, `norm_squared`, `reciprocal`, `powi`, checked division variants, `atanh` |

Hyperreal owns scalar arithmetic, domain checks, and structural type names;
Hyperlattice does not wrap or rename those APIs. Attach cancellation directly
with `Real::abort` before invoking a scalar operation. The two structural checked
reciprocal functions reject unknown-zero divisors without refinement, unlike
`Real::inverse`, which may refine to establish a nonzero denominator.

### Vectors

The following lists operations across the vector family; dimension-specific
methods are identified where they differ.

| Task | API |
| --- | --- |
| Construct | `Vector2::from_xy`, `Vector3::new`/`from_xyz`, `Vector4::new`/`from_xyzw`, `zero`/`zeros` |
| Import/export | `try_from_f32_array`, `try_from_f64_array`, `to_f32_array_lossy`, `to_f64_array_lossy` |
| Read components | Array indexing or the public `.0` array; `SharedScaleVec::components`, `into_components` |
| Construct basis vectors | `Vector2::x/y`, `Vector3::x/y/z` |
| Measure | `dot`, `norm_squared`, `magnitude`, `norm`; `Vector2/3::squared_distance`, `Vector2::wedge`, `Vector3::cross`; `SharedScaleVec::squared_norm` |
| Normalize and divide | `normalize`, `normalize_checked`, abort-aware checked variants, `div_scalar_checked` |
| Combine | `lerp`, `step`, `mean`, `weighted_sum` |
| 3D frames | `unit_cross_checked`, `orthonormal_basis_checked`, `angle_to` |
| Retain structure | `structural_facts`, `Vector3/4::exact_facts`, `into_shared_scale`, `shared_scale_view` |

`magnitude` returns a fallible Euclidean norm. `norm` unwraps that result and
can panic on a scalar error; `norm_squared` avoids the square root.

`Vector4HomogeneousKind`, `Axis2`, `SignedAxis2`, and `SignedAxis4` describe
common exact shapes without asking users to decode coordinate patterns.

### Points and bounds

| Task | API |
| --- | --- |
| Construct | `Point2::new`, `Point3::new`, `origin` |
| Import/export | `try_from_f32_array`, `try_from_f64_array`, `to_f32_array_lossy`, `to_f64_array_lossy` |
| Convert | `to_vector`, `into_vector` |
| Combine | `lerp`, `centroid`, `weighted_sum` |
| Inspect structure | `structural_facts`, `shared_scale_view` |
| Create an origin bound | `Aabb::origin` |

Point/vector arithmetic follows affine semantics: subtracting points yields a
vector, while translating a point by a vector yields a point.
`Aabb` stores raw corners without validating their ordering. `Aabb::origin`
has coincident zero corners and is not a canonical empty-set sentinel.

### Exact 2D algebra

| Task | API |
| --- | --- |
| Displacement and facts | `displacement2`, `displacement2_facts` |
| Dot, wedge, and norms | `dot2`, `wedge2`, `squared_norm2`, `squared_distance2` |
| Product reducers | `signed_product_sum2`, `positive_product_sum2`, `product_term2_facts`, `product_sum2_facts` |
| Orientation expression | `orient2_expr`, `orient2_expr_facts` |

These functions construct exact expressions and facts. Hyperlimit owns the
policy that turns an orientation expression into a certified classification.

### Projective construction

| Task | API |
| --- | --- |
| Define a plane | `ProjectivePlane3::new`, the `Plane3Coefficients` trait |
| Intersect planes | `intersect_two_planes`, `intersect_three_planes` |
| Intersect a homogeneous line and plane | `intersect_homogeneous_line_plane`, `HomogeneousLine3::intersect_plane` |
| Evaluate incidence | `homogeneous_point_plane_expression`, `HomogeneousPoint3::plane_expression` |
| Convert a finite point | `HomogeneousPoint3::to_affine_point` |

Homogeneous results intentionally postpone division. `to_affine_point` divides
by the homogeneous weight and propagates scalar failures; unlike the structural
checked reciprocal, it can use native scalar nonzero reasoning. Raw plane, line,
and point constructors accept degenerate coefficients and do not establish
unique intersections or incidence proofs.

### Matrices and transforms

Both matrix types provide `new`, `zero`, `identity`, `transpose`, `determinant`,
`inverse`, `inverse_checked`, `powi`, checked scalar/matrix division, and
`structural_facts`. Storage is row-major; transforms act on column vectors.

| Task | API |
| --- | --- |
| Construct `Matrix3` structure | `diagonal`, `uniform_scale` |
| Construct `Matrix4` transforms | `from_row_major`, `from_row_slice`, `affine_translation`, `affine_nonuniform_scale`, `affine_orthonormal` |
| Scale all homogeneous coordinates | `Matrix3/4::uniform_scale` (including the weight coordinate) |
| Construct `Matrix4` rotations | `rotation_x`, `rotation_y`, `rotation_z`, `rotation_axis_angle`, `rotation_between_vectors`, `rotation_between_unit_vectors` |
| Construct `Matrix4` signed permutations | `signed_permutation` |
| Use specialized inverses | `diagonal_inverse`, triangular inverse methods, `affine_translation_inverse`, `affine_orthonormal_inverse`, `signed_permutation_inverse`, `uniform_scale_inverse` |
| Transform values | `transform_vec3`, `transform_vec4`, `transform_point3`, `transform_direction3` and corresponding batch methods |
| Inspect scheduling | `exact_facts`, `structural_facts` and its `determinant_schedule_hint` |

Specialized methods retain their stated shape preconditions. For example,
`affine_orthonormal` embeds the supplied block without proving orthonormality,
and `rotation_between_unit_vectors` requires caller-certified unit inputs.
Checked variants reject zero or unresolved selected divisors where their
structural guards run. General matrix operations provide
`inverse_checked_with_abort` and `div_matrix_checked_with_abort` with
path-dependent signal attachment; exact shortcuts can complete with a set flag.

### Facts and cancellation

Point, vector, matrix, displacement, product, and orientation fact objects
expose known-zero/nonzero/unknown masks and counts, sparse-support tests,
shared-denominator and dyadic schedule tests, transform kinds, and determinant
schedule hints. These describe the queried coordinates. Detached packets do not
track later coordinate mutation; recollect them after changing the source.
Many fact fields are public and queries trust their stored contents. An unknown
zero fact is not a proof of zero.

Create an `AbortSignal` with `Arc<AtomicBool>` and pass it to `*_with_abort`
methods for cooperative cancellation. Attachment and polling depend on the
operation's path; the signal is not a deadline or total-work bound.

## Features

| Feature | Default | Effect |
| --- | --- | --- |
| `arbitrary` | no | Implements `arbitrary::Arbitrary` for `Complex`, `Vector2/3/4`, and `Matrix3/4`, using finite binary64 or integer seeds. |
| `hyperreal-dispatch-trace` | no | Enables Hyperreal dispatch instrumentation for development and benchmarks. |
| `serde` | no | Enables native `Real` serialization and `Point2` derives; forwards Serde to the interval companion when enabled. |
| `interval` | no | Reexports generic interval and affine carriers without selecting a scalar backend. |
| `interval-fixed` | no | Enables `interval` with the fixed-point backend. |
| `interval-dyadic` | no | Enables `interval` with budgeted dyadic arithmetic. |
| `interval-rational` | no | Enables `interval` with exact rational interval arithmetic. |
| `interval-hyperreal` | no | Enables `interval` with Hyperreal enclosure integration and its rational/dyadic dependencies. |
| `interval-float-enclosure` | no | Enables `interval` with primitive floating enclosures, subject to scalar environment qualification. |

Hyperlattice has no default features.

## Guarantees and boundaries

- Native coordinates and matrix entries are `Real`; no primitive float is used
  as an internal algebra fallback.
- Finite `f32`/`f64` imports preserve the exact represented binary value; NaN and
  infinity are rejected. `0.1_f64` is not one tenth; use `Rational::fraction`
  for an exact ratio.
- Lossy exports are named and return `None` if a coordinate cannot produce a
  finite primitive value.
- Checked guards reject known-zero and unknown-zero divisors without refinement
  at that guard. Exact shortcuts and subsequent native scalar operations retain
  their own failure and cancellation contracts.
- Collected structural facts are conservative for unchanged source coordinates;
  mutable or detached fact packets are not self-validating certificates.
- Native coordinate equality is structural. Distinct representations need not
  mean distinct mathematical values, and carrier results add no predicate-policy
  or inherited-certainty receipt.
- Hyperlattice constructs expressions and carriers. Hyperlimit owns predicate
  escalation and classifications; geometry crates own curves and topology.

## Validation and performance tooling

The deterministic representation suite constructs 22 named positive recipes,
checks all eight public structural kinds, and exercises selected operations
across scalar, complex, vector, point, matrix, and projective carriers. All 484
ordered recipe pairs pass through selected arithmetic identities; the
Serde-enabled test also checks their serialized private tags. Its comparison
helper accepts overlapping enclosures when exact equality is not established,
so passing is not a general exact-equality or replay certificate. Four selected
feature/cache configurations are available from the crate directory with:

```sh
scripts/representation_coverage.sh
```

The matching fuzz target builds 22 labeled recipes and four opaque expressions,
then uses input bytes to scale values and rotate pairings. Construction depth
is limited, but evaluation work is not bounded by that limit. Its assertions
and interval-overlap fallback are described in [`fuzz/README.md`](fuzz/README.md):

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo +nightly fuzz run hyperreal_representations --fuzz-dir fuzz -- -max_total_time=30
```

`scripts/coverage.sh` collects LLVM execution coverage for selected no-feature
and all-feature configurations, including all-feature test targets. It requests
benchmark-report suppression and writes text and HTML under `target/coverage`
by default. The allocation profile reports allocator calls, requested-byte
traffic, and window-relative live balances for four workloads over 22 labeled
recipes. These counters do not measure total live memory; limitations are
described in [`PERFORMANCE.md`](PERFORMANCE.md#representation-coverage-and-allocation-gates).

```sh
scripts/coverage.sh
scripts/allocation_profile.sh
```

The Criterion `real_representations` group measures the same 22 labeled recipes;
the labels do not verify distinct runtime representation classes. The broader
`mathbench` includes similarly named scalar, complex, vector, and matrix rows for
native carriers, SymbolicaFloat (labeled Numerica 128), GMP/MPFR 128 through Rug,
and Symbolica's symbolic adapter. Inputs, guards, cancellation handling, and output
materialization differ across rows; matching names do not establish equivalent
completed workloads. Methodology and retained results live in
[`PERFORMANCE.md`](PERFORMANCE.md).

## Ecosystem and further documentation

- [Hyperreal](https://github.com/timschmidt/csgrs/tree/main/crates/hyperreal) supplies exact-aware
  scalars and scalar certificates.
- [Hyperlimit](https://github.com/timschmidt/csgrs/tree/main/crates/hyperlimit) supplies certified
  geometric predicates over these carriers.
- [Hypercurve](https://github.com/timschmidt/csgrs/tree/main/crates/hypercurve),
  [Hypertri](https://github.com/timschmidt/csgrs/tree/main/crates/hypertri), and
  [Hypermesh](https://github.com/timschmidt/csgrs/tree/main/crates/hypermesh) own higher geometry.

[`PERFORMANCE.md`](PERFORMANCE.md) records benchmark methodology and retained
optimization evidence. The shared report writer updates [`benchmarks.md`](benchmarks.md)
from retained Criterion results and the manifest's benchmark catalog. Automatic
callbacks honor the report-suppression gate; stored results may come from older
runs. Regenerate the report without measuring, from the workspace root, with
`cargo run -p hyperlattice --example write_benchmarks_md`.

Generate public API documentation with `cargo doc -p hyperlattice --no-deps --open`.
To include private library items and all feature-gated APIs, use
`cargo doc -p hyperlattice --no-deps --all-features --document-private-items --open`.

## References

- Bareiss, Erwin H. “Sylvester's Identity and Multistep Integer-Preserving
  Gaussian Elimination.” *Mathematics of Computation*, vol. 22, no. 103,
  1968, pp. 565–578.
  [doi:10.1090/S0025-5718-1968-0226829-0](https://doi.org/10.1090/S0025-5718-1968-0226829-0).
- Berkowitz, Stuart J. “On Computing the Determinant in Small Parallel Time
  Using a Small Number of Processors.” *Information Processing Letters*,
  vol. 18, no. 3, 1984, pp. 147–150.
  [doi:10.1016/0020-0190(84)90018-8](https://doi.org/10.1016/0020-0190(84)90018-8).
- Gustavson, Fred G. “Two Fast Algorithms for Sparse Matrices:
  Multiplication and Permuted Transposition.” *ACM Transactions on
  Mathematical Software*, vol. 4, no. 3, 1978, pp. 250–269.
  [doi:10.1145/355791.355796](https://doi.org/10.1145/355791.355796).
- Hou, Shui-Hung. “A Simple Proof of the Leverrier-Faddeev Characteristic
  Polynomial Algorithm.” *SIAM Review*, vol. 40, no. 3, 1998, pp. 706–709.
  [doi:10.1137/S003614459732076X](https://doi.org/10.1137/S003614459732076X).
- Yap, Chee K. “Towards Exact Geometric Computation.” *Computational
  Geometry*, vol. 7, 1997, pp. 3–23.
  [doi:10.1016/0925-7721(95)00040-2](https://doi.org/10.1016/0925-7721(95)00040-2).

These references provide background on exact elimination and determinants,
sparse multiplication, characteristic polynomials, and exact geometric
computation. Individual API documentation describes the implemented schedules.

## Acknowledgements

Hyperlattice is developed by Timothy Schmidt as part of the Hyper ecosystem.
The repository history also records contributions from TimTheBig. Its scalar
model and many exact reducers are built on Hyperreal.

## License and contributing

Hyperlattice is distributed under the MIT License; see [`LICENSE`](LICENSE).
Changes should preserve point/vector distinctions, explicit lossy boundaries,
and conservative structural facts. Before submitting a change, run:

```sh
cargo fmt --all -- --check
cargo test --all-targets --all-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features
```

## Extension protocol

Extensions follow the stack-wide [admission protocol](../../docs/admission.md).
It defines the exactness, certainty, boundary, cache, validation, and evidence
requirements shared by the Hyper crates.

## Interval algebra

Enable `interval` for generic `IntervalVector`, `IntervalMatrix` and `BoxN`
carriers, or select a backend with `interval-fixed`, `interval-dyadic`,
`interval-rational`, or `interval-float-enclosure`.
`interval-hyperreal` additionally exposes certified exact-Real-to-dyadic
conversion through the same scalar re-export.
These are Cartesian enclosures with explicit empty sets and checked scalar
arithmetic. Dyadic operations share a caller-owned budget across lanes; rational
arithmetic has no internal work/heap bound. Scalar backends are available under
`hyperlattice::interval::hyperinterval`. The companion `hyperlattice-interval`
declares Rust 1.88 and offers a primitive allocation-free `no_std` build.
Hyperreal-backed configurations retain the Rust 1.95 requirement; enabling an
interval feature on Hyperlattice does not remove its required Hyperreal dependency.

`AffineMap::image` retains exact affine definitions and shared source parameters.
Further maps, parameter restrictions and validated replay preserve correlations;
`AffineImage::enclosure` explicitly projects the result to a Cartesian box.
