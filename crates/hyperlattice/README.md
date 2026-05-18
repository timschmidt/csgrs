<h1>
  hyperlattice
  <img src="./doc/hyperlattice.png" alt="Hyper, a clever mathematician" width="144" align="right">
</h1>

`hyperlattice` provides small fixed-size linear algebra over `hyperreal::Real`: complex
numbers, 2D/3D/4D vectors, 3x3/4x4 matrices, transforms, and object-level structural
facts.

The crate is not a general BLAS replacement. It focuses on the small exact vector and
matrix objects that geometry, predicates, solvers, and domain crates repeatedly need.

## Hyper Ecosystem

`hyperlattice` is the object-algebra layer between scalar facts and topology/domain
crates.

- [hyperreal](https://github.com/timschmidt/hyperreal): exact scalar values and
  structural facts.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): predicate layer that consumes
  point, vector, determinant, and shared-scale facts.
- [hypercurve](https://github.com/timschmidt/hypercurve),
  [hypertri](https://github.com/timschmidt/hypertri), and
  [hypermesh](https://github.com/timschmidt/hypermesh): geometry crates that reuse
  exact small-vector and transform structure.
- [hypersolve](https://github.com/timschmidt/hypersolve): residual and linear-algebra
  preparation over exact scalars.
- [hyperphysics](https://github.com/timschmidt/hyperphysics) and
  [hypervoxel](https://github.com/timschmidt/hypervoxel): domain crates that need exact
  vectors, transforms, and object-level facts.

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

## Performance Model

The crate reduces exact cost by exploiting fixed sizes and retained structure. Matrix
multiplication is unrolled, small powers are specialized before exponentiation by
squaring, borrowed arithmetic avoids unnecessary cloning, and product-sum reducers
preserve rational structure. Prepared matrix and right-divisor handles let callers reuse
determinant, adjugate, reciprocal, minor, and inverse work without exposing internal
cache storage.

Benchmarks track scalar, vector, matrix, prepared-cache, and dispatch-trace behavior so
shortcuts can be accepted only when they help the target surface without destabilizing
nearby Hyper predicate paths.

## Current Status

Implemented today:

- `Real` constants, zero-status helpers, and elementary-function wrappers;
- `Complex` arithmetic and integer powers;
- `Vector2`, `Vector3`, `Vector4`, shared-scale views, homogeneous facts, dot products,
  normalization, and checked/abort-aware operations;
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
hyperlattice = "0.5.0"
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
use hyperlattice::{Matrix3, Real, Vector3};

fn r(value: i32) -> Real { value.into() }

let v = Vector3::new([r(3), r(4), r(0)]);
assert_eq!(v.dot(&v), r(25));

let m = Matrix3::identity();
assert_eq!(m.clone() * m.inverse().unwrap(), Matrix3::identity());
```

## Development

Useful local checks:

```sh
cargo test
cargo bench --bench mathbench
cargo bench --bench regression_sentinels
```

## References

Bareiss, Erwin H. "Sylvester's Identity and Multistep Integer-Preserving
Gaussian Elimination." *Mathematics of Computation*, vol. 22, no. 103, 1968,
pp. 565-578.

Yap, Chee K. "Towards Exact Geometric Computation." *Computational Geometry*,
vol. 7, nos. 1-2, 1997, pp. 3-23.

## Source Layout

- `src/scalar.rs`: `Real` constants, functions, facts, and zero status
- `src/complex.rs`: `Complex`
- `src/algebra2.rs`: exact 2D expressions and displacement facts
- `src/vector.rs`: `Vector2`, `Vector2Facts`, `Vector3`, and `Vector4`
- `src/matrix`: `Matrix3`, `Matrix4`, and transform handles
- `src/kernels.rs`: crate-private `Real` product-sum and structural helpers
