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
| `Complex` | Exact complex arithmetic and integer powers. |
| `Vector2`, `Vector3`, `Vector4` | Fixed-size exact vectors with dot, norm, interpolation, and checked normalization operations. |
| `Point2`, `Point3` | Affine points kept distinct from displacement vectors. |
| `Matrix3`, `Matrix4` | Fixed-size exact matrices, transforms, determinants, and checked inverses. |
| `ProjectivePlane3` | Plane coefficients for projective intersection construction. |
| `HomogeneousPoint3`, `HomogeneousLine3` | Exact projective results that delay affine division. |
| `SharedScaleVec`, shared-scale views | Factored coordinate carriers for common rational scales. |
| `*Facts`, transform-kind, and schedule types | Reusable zero, support, homogeneous, exact-set, and determinant metadata. |
| `BlasResult<T>`, `Problem`, `AbortSignal` | Checked failures and cooperative cancellation. |

`Rational`, `Real`, and their principal structural types are re-exported from
Hyperreal for convenience.

## Quick start

Create a project and add the crate:

```sh
cargo new exact-linear-algebra
cd exact-linear-algebra
cargo add hyperlattice
```

Equivalent manifest entry:

```toml
[dependencies]
hyperlattice = "0.6.0"
```

Replace `src/main.rs` with:

<!-- quickstart:start -->
```rust
use hyperlattice::{Matrix3, Real, Vector3, sqrt};

fn r(value: i32) -> Real {
    value.into()
}

fn main() -> hyperlattice::BlasResult<()> {
    let vector = Vector3::new([r(3), r(4), r(0)]);
    assert_eq!(vector.dot(&vector), r(25));
    assert_eq!(sqrt(vector.dot(&vector))?, r(5));

    let identity = Matrix3::identity();
    assert_eq!(identity.clone() * vector.clone(), vector);
    assert_eq!(identity.inverse()?, Matrix3::identity());
    Ok(())
}
```
<!-- quickstart:end -->

Run it with `cargo run`. The same source is checked in as
[`examples/readme_quickstart.rs`](examples/readme_quickstart.rs), compiled by
the test suite, and compared with the README block.

## API guide

### Scalar and complex operations

| Task | API |
| --- | --- |
| Constants | `zero`, `one`, `e`, `pi`, `tau`, `i` |
| Zero knowledge | `zero_status`, `zero_status_with_abort` |
| Reciprocal and powers | `reciprocal`, `reciprocal_ref`, checked variants, `pow`, `powi` |
| Elementary functions | `sqrt`, `exp`, `ln`, `log10`, `sin`, `cos`, `tan`, `sinh`, `cosh`, `tanh` |
| Inverse functions | `asin`, `acos`, `atan`, `asinh`, `acosh`, `atanh`, including abort-aware variants |
| Complex values | `Complex::new`, `zero`, `one`, `i`, `conjugate`, `norm_squared`, `reciprocal`, `powi`, checked division variants |

Scalar wrappers keep a consistent `BlasResult` surface for linear-algebra
callers. More specialized scalar functions remain in Hyperreal.

### Vectors

| Task | API |
| --- | --- |
| Construct | `Vector2::from_xy`, `Vector3::new`/`from_xyz`, `Vector4::new`/`from_xyzw`, `zero`/`zeros` |
| Import/export | `try_from_f32_array`, `try_from_f64_array`, `to_f32_array_lossy`, `to_f64_array_lossy` |
| Read components | `x`, `y`, `z`, `components`, `into_components` |
| Measure | `dot`, `norm_squared`, `squared_norm`, `magnitude`/`norm`, `squared_distance`, `wedge`, `cross` |
| Normalize and divide | `normalize`, `normalize_checked`, abort-aware checked variants, `div_scalar_checked` |
| Combine | `lerp`, `step`, `mean`, `weighted_sum` |
| 3D frames | `unit_cross_checked`, `orthonormal_basis_checked`, `angle_to` |
| Retain structure | `structural_facts`, `exact_facts`, `into_shared_scale`, `shared_scale_view` |

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

Homogeneous results intentionally postpone division. `to_affine_point` is the
checked boundary at which a nonzero homogeneous scale is required.

### Matrices and transforms

Both matrix types provide `new`, `zero`, `identity`, `transpose`, `determinant`,
`inverse`, `inverse_checked`, `powi`, checked scalar/matrix division, and
`structural_facts`.

| Task | API |
| --- | --- |
| Construct `Matrix3` structure | `diagonal`, `uniform_scale` |
| Construct `Matrix4` transforms | `from_row_major`, `from_row_slice`, `affine_translation`, `affine_nonuniform_scale`, `uniform_scale` |
| Construct rotations | `rotation_x`, `rotation_y`, `rotation_z`, `rotation_axis_angle`, `rotation_between_vectors`, `affine_orthonormal` |
| Construct signed permutations | `signed_permutation` |
| Use specialized inverses | `diagonal_inverse`, triangular inverse methods, `affine_translation_inverse`, `affine_orthonormal_inverse`, `signed_permutation_inverse`, `uniform_scale_inverse` |
| Transform values | `transform_vec3`, `transform_vec4`, `transform_point3`, `transform_direction3` and corresponding batch methods |
| Inspect scheduling | `exact_facts`, `structural_facts`, `determinant_schedule_hint` |

Specialized division and inverse methods also have checked and abort-aware
variants where a divisor or pivot can be unresolved.

### Facts and cancellation

Point, vector, matrix, displacement, product, and orientation fact objects
expose known-zero/nonzero/unknown masks and counts, sparse-support tests,
shared-denominator and dyadic schedule tests, transform kinds, and determinant
schedule hints. They are reusable metadata, not proof that an unresolved
coordinate is zero.

Create an `AbortSignal` with `Arc<AtomicBool>` and pass it to `*_with_abort`
methods when long exact refinement must be cancellable.

## Features

| Feature | Default | Effect |
| --- | --- | --- |
| `arbitrary` | no | Implements `arbitrary::Arbitrary` for lattice-owned types. |
| `hyperreal-dispatch-trace` | no | Enables Hyperreal dispatch instrumentation for development and benchmarks. |

Hyperlattice has no default features.

## Guarantees and boundaries

- Native coordinates and matrix entries are `Real`; no primitive float is used
  as an internal algebra fallback.
- Finite `f32`/`f64` imports preserve the exact represented binary value.
- Lossy exports are named and return `None` if a coordinate cannot produce a
  finite primitive value.
- Checked division, normalization, and inversion reject definite-zero and
  unresolved-zero divisors or pivots.
- Structural facts are conservative and remain attached to their owning
  point, vector, matrix, or projective object.
- Hyperlattice constructs expressions and carriers. Hyperlimit owns predicate
  escalation and classifications; geometry crates own curves and topology.

## Ecosystem and further documentation

- [Hyperreal](https://github.com/timschmidt/hyperreal) supplies exact-aware
  scalars and scalar certificates.
- [Hyperlimit](https://github.com/timschmidt/hyperlimit) supplies certified
  geometric predicates over these carriers.
- [Hypercurve](https://github.com/timschmidt/hypercurve),
  [Hypertri](https://github.com/timschmidt/hypertri), and
  [Hypermesh](https://github.com/timschmidt/hypermesh) own higher geometry.

[`PERFORMANCE.md`](PERFORMANCE.md) records benchmark methodology and retained
optimization evidence. Generate the complete signatures and trait
implementations with `cargo doc --open`.

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

Bareiss and Berkowitz motivate exact determinant schedules; Gustavson motivates
sparse-support dispatch; Hou covers a small-matrix characteristic-polynomial
route; Yap establishes the exact-computation boundary used by downstream
geometry.

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
