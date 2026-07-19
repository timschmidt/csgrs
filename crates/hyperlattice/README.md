<h1>
  hyperlattice
  <img src="./doc/hyperlattice.png" alt="Hyper, a clever mathematician" width="144" align="right">
</h1>

`hyperlattice` provides small fixed-size linear algebra over `hyperreal::Real`: complex
numbers, 2D/3D/4D vectors, 3x3/4x4 matrices, transforms, and object-level structural
facts.

The crate is not a general BLAS replacement. It focuses on the small exact vector and
matrix objects that geometry, predicates, solvers, and domain crates repeatedly need.

## Typical Linear-Algebra Problems

Small linear algebra sits on the fault line between performance and exactness. Floating
matrices are fast but can hide singular pivots, near-zero determinants, and
transform-kind assumptions. Full symbolic expansion preserves meaning but can grow
before a caller knows whether a cheap structural fact was enough.

`hyperlattice` keeps objects small and facts local. Zero masks, homogeneous
point/direction tags, determinant schedule hints, sparse support, shared-scale views,
and prepared matrix cache summaries let callers skip known-zero work, choose exact
reducers, and delay scalar canonicalization until a result is needed.

## Main Types

- `Complex` provides exact complex arithmetic and integer powers.
- `Vector2`, `Vector3`, `Vector4`, homogeneous vector facts, shared-scale views, and
  signed-axis helpers describe small exact vectors.
- `Point2`, `Point3`, `Point2Facts`, `Point3Facts`, `PointSharedScaleView`, and
  `PointSharedScaleFacts` keep point coordinates separate from vectors while preserving
  sparse support, shared-denominator schedules, known-zero masks, and one-hot facts.
- `ProjectivePlane3`, `Plane3Coefficients`, `HomogeneousPoint3`, `HomogeneousLine3`,
  `intersect_two_planes`, `intersect_three_planes`, and
  `intersect_homogeneous_line_plane` keep exact 3D plane intersections in homogeneous
  form so affine division is delayed until a caller proves the point is finite.
- `Matrix3`, `Matrix4`, transform handles, transformed-vector/matrix views, prepared
  matrix handles, and prepared right-divisor handles describe small exact matrices.
- `Matrix3StructuralFacts`, `Matrix4StructuralFacts`, transform-kind enums, determinant
  schedule hints, and cache summaries preserve matrix structure.
- `Displacement2Facts`, `ProductTerm2Facts`, `ProductSum2Facts`, and `Orient2Facts`
  expose exact 2D algebra facts for predicate and curve callers.
- `AbortSignal`, `BlasResult`, checked result types, zero-status helpers, and scalar
  function wrappers provide fallible exact operations.

## Precision Model

All native scalar, vector, complex, and matrix operations use `Real`. Primitive floats
should appear only at named import/export, rendering, diagnostics, or interop edges.
Checked operations reject definite-zero and unknown-zero divisors or pivots instead of
rounding through a singular path.

`hyperlattice` preserves object facts that `hyperreal` cannot know by itself: coordinate
zero masks, homogeneous shape, shared scale, affine/translation/diagonal/projective
transform kind, determinant schedule, and prepared cache availability.
Point and projective carriers live here rather than in `hyperlimit` so predicate
policy can classify them without becoming the storage owner.

## Numerical Explosion

`hyperlattice` combats numerical explosion by keeping object-level facts beside exact
scalars. Zero masks, one-hot coordinates, determinant schedules, shared scales,
homogeneous facts, and prepared matrix caches let callers avoid expanding matrix,
projective, and transform expressions until an exact predicate or solve actually needs
the scalar terms.

## Performance Model

The crate reduces exact cost by exploiting fixed sizes and retained structure. Matrix
multiplication is unrolled, small powers are specialized before exponentiation by
squaring, borrowed arithmetic avoids unnecessary cloning, and product-sum reducers
preserve rational structure. Repeated exact scalar products reuse Hyperreal's bounded,
cycle-free rational product retention. Prepared matrix and right-divisor handles let callers reuse
determinant, adjugate, reciprocal, minor, and inverse work without exposing internal
cache storage.

Benchmarks track scalar, vector, matrix, prepared-cache, and dispatch-trace behavior so
shortcuts can be accepted only when they help the target surface without destabilizing
nearby Hyper predicate paths.

The paper-by-paper implementation map, accepted/rejected experiments, and reproducible
measurements are recorded in [`PERFORMANCE.md`](./PERFORMANCE.md).

## Current Status

Implemented today:

- `Real` constants, zero-status helpers, and elementary-function wrappers;
- `Complex` arithmetic and integer powers;
- `Vector2`, `Vector3`, `Vector4`, shared-scale views, homogeneous facts, dot products,
  normalization, and checked/abort-aware operations;
- `Point2`, `Point3`, point fact packets, shared-scale point views, sparse point facts,
  and point/vector conversions;
- homogeneous 3D points, Pluecker lines, projective plane coefficients, exact
  two-plane and three-plane intersections, and line/plane homogeneous intersections;
- exact 2D algebra helpers and facts for displacement, wedge/dot, product sums, and
  orientation expressions;
- `Matrix3`, `Matrix4`, determinant, inverse, transpose, multiplication, powers, checked
  paths, transform handles, prepared matrix/right-divisor handles, and structural facts;
- `RealFacts`, sign/magnitude facts, abort signals, `arbitrary` support, regression
  sentinels, and benchmark hooks.

Fallible operations return `BlasResult<T>` or checked variants. Checked operations
reject definite zero and unknown-zero divisors or pivots.

## Installation

```toml
[dependencies]
hyperlattice = "0.6.0"
```

For sibling checkouts:

```toml
[dependencies]
hyperlattice = { path = "../hyperlattice" }
```

Feature summary:

- `arbitrary`: implements `arbitrary::Arbitrary` for lattice-owned types.
- `hyperreal-dispatch-trace`: enables scalar dispatch tracing during benchmarks.

## Usage

```rust
use hyperlattice::{intersect_three_planes, Matrix3, Point3, ProjectivePlane3, Real, Vector3};

fn r(value: i32) -> Real {
    value.into()
}

fn main() {
    let v = Vector3::new([r(3), r(4), r(0)]);
    assert_eq!(v.dot(&v), r(25));

    let m = Matrix3::identity();
    assert_eq!(m.clone() * m.inverse().unwrap(), Matrix3::identity());

    let x = ProjectivePlane3::new(Point3::new(r(1), r(0), r(0)), r(-2));
    let y = ProjectivePlane3::new(Point3::new(r(0), r(1), r(0)), r(-3));
    let z = ProjectivePlane3::new(Point3::new(r(0), r(0), r(1)), r(-4));
    let p = intersect_three_planes(&x, &y, &z);
    assert_eq!(p.to_affine_point().unwrap(), Point3::new(r(2), r(3), r(4)));
}
```

## Development

```sh
cargo fmt --all -- --check
cargo test --locked
cargo check --benches --locked
cargo clippy --all-targets --locked -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --locked
cargo bench --bench mathbench
cargo bench --bench regression_sentinels
```

## References

Implementation comments describe local structure and dispatch choices; the numerical
background is consolidated here.

- Bareiss, Erwin H. "Sylvester's Identity and Multistep Integer-Preserving Gaussian
  Elimination." *Mathematics of Computation*, vol. 22, no. 103, 1968,
  https://doi.org/10.2307/2004533.
- Berkowitz, Stuart J. "On Computing the Determinant in Small Parallel Time Using a
  Small Number of Processors." *Information Processing Letters*, vol. 18, no. 3,
  1984, https://doi.org/10.1016/0020-0190(84)90018-8.
- Gustavson, Fred G. "Two Fast Algorithms for Sparse Matrices: Multiplication and
  Permuted Transposition." *ACM Transactions on Mathematical Software*, vol. 4,
  no. 3, 1978, https://doi.org/10.1145/355791.355796.
- Hou, Shui-Hung. "A Simple Proof of the Leverrier-Faddeev Characteristic Polynomial
  Algorithm." *SIAM Review*, vol. 40, no. 3, 1998,
  https://doi.org/10.1137/S003614459732076X.
- Yap, Chee K. "Towards Exact Geometric Computation." *Computational Geometry*,
  vol. 7, nos. 1-2, 1997, https://doi.org/10.1016/0925-7721(95)00040-2.

## Source Layout

- `src/scalar.rs`: `Real` constants, functions, facts, and zero status
- `src/complex.rs`: `Complex`
- `src/algebra2.rs`: exact 2D expressions and displacement facts
- `src/vector.rs`: `Vector2`, `Vector2Facts`, `Vector3`, and `Vector4`
- `src/point.rs`: `Point2`, `Point3`, point facts, and shared-scale point views
- `src/projective.rs`: homogeneous points, Pluecker lines, and plane intersections
- `src/matrix`: `Matrix3`, `Matrix4`, and transform handles
- `src/kernels.rs`: crate-private `Real` product-sum and structural helpers

## Hyper Ecosystem

`hyperlattice` builds fixed-size exact objects over
[hyperreal](https://github.com/timschmidt/hyperreal) for predicate consumers such as
[hyperlimit](https://github.com/timschmidt/hyperlimit) and geometry owners such as
[hypercurve](https://github.com/timschmidt/hypercurve),
[hypertri](https://github.com/timschmidt/hypertri), and
[hypermesh](https://github.com/timschmidt/hypermesh). Solver and domain consumers
include [hypersolve](https://github.com/timschmidt/hypersolve),
[hyperpath](https://github.com/timschmidt/hyperpath), and
[hyperphysics](https://github.com/timschmidt/hyperphysics).
