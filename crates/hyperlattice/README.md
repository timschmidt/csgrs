<h1>
  hyperlattice
  <img src="./doc/hyperlattice.png" alt="Hyper, a clever mathematician" width="144" align="right">
</h1>

`hyperlattice` provides small fixed-size linear algebra over
`hyperreal::Real`. `Real` is the coordinate and scalar type for complex
numbers, 2D/3D/4D vectors, and 3x3/4x4 matrices.

Primitive `f32` and `f64` are accepted only at named boundaries for checked
input lifting, rendering, IO, diagnostics, or third-party interop. Lossy output
is explicit through `Real::to_f64_lossy`.

## Hyper Ecosystem

`hyperlattice` is the object-algebra layer: small vectors, matrices, transforms,
and product-sum structure over `hyperreal::Real`.

- [hyperreal](https://github.com/timschmidt/hyperreal): exact rational, symbolic, and computable
  real arithmetic.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): exact predicate policy and certified
  geometric decisions.
- [hyperlattice](https://github.com/timschmidt/hyperlattice): small exact vector, matrix, and
  transform algebra.
- [hypercurve](https://github.com/timschmidt/hypercurve): planar curve, contour, region, and
  boolean geometry.
- [hypertri](https://github.com/timschmidt/hypertri): exact polygon triangulation and constrained
  Delaunay topology.
- [hypermesh](https://github.com/timschmidt/boolmesh): 3D mesh boolean experiments and the
  future exact-aware mesh-topology layer.
- [hypersolve](https://github.com/timschmidt/hypersolve): experimental exact-aware solver layer.
- [hyperdrc](https://github.com/timschmidt/hyperdrc): PCB design-readiness checks over exact-aware
  geometry adapters.
- [hyperphysics](https://github.com/timschmidt/hyperphysics): placeholder physics-domain crate
  for the exact geometry stack.
- [csgrs](https://github.com/timschmidt/csgrs): constructive solid geometry and polygon boolean
  engine used by HyperDRC and available as an interop target.

## Traditional Numerical Problems

Small linear algebra sits directly on the fault line between performance and
exactness. Floating matrices are fast but can hide singular pivots, near-zero
determinants, and transform-kind assumptions. Full symbolic expansion keeps
meaning but can explode operand size before a caller knows whether a cheap
structural fact was enough.

`hyperlattice` approaches that tradeoff by keeping objects small and facts
local. It carries zero masks, homogeneous point/direction tags, determinant
schedule categories, sparse-support hints, shared-scale views, and prepared
matrix cache summaries. Those facts let callers skip known-zero terms, choose
fraction-free or shared-denominator reducers, reuse inverse/cofactor work, and
delay scalar canonicalization until the result is actually needed.

## Structural Facts

`hyperlattice` carries cheap numerical structure discovered by constructors and
hot kernels: known coordinate zeros, point/direction tags, homogeneous
coordinate shape, sparsity masks, determinant/cofactor state, and exact rational
facts. Those facts select faster exact vector and matrix paths without becoming
geometry predicates.

Exactness is not implemented by eagerly canonicalizing every coordinate after
each operation. In the sense of Yap's exact geometric computation model,
`hyperlattice` preserves conservative object-level structure so later exact
reducers and predicate crates can either certify a decision or report
uncertainty. Missing facts are missed optimizations; false facts are bugs.

Future APIs should expose structural metadata in stable value objects so higher
crates can reuse it without reinterpreting internal layouts.

## Current State

Implemented:

- `Real` constants and elementary functions
- `Complex` arithmetic and integer powers
- `Vector2`, `Vector3`, and `Vector4` arithmetic, dot products, magnitude,
  normalization, checked division, and abort-aware variants
- `Vector2Facts` and `Axis2` for coordinate zero-mask metadata
- exact 2D algebra helpers and facts used by predicates and curves
- `Matrix3` and `Matrix4` arithmetic, multiplication, determinant, inverse,
  transpose, reciprocal, integer powers, checked variants, and transform handles
- `RealFacts`, `RealSign`, `RealMagnitudeBits`, `ZeroStatus`, and `AbortSignal`

Fallible operations return `BlasResult<T>`. Checked operations reject both
definite zero and unknown-zero divisors or pivots.

## Installation

```toml
[dependencies]
hyperlattice = "0.4.0"
```

From sibling checkouts:

```toml
[dependencies]
hyperlattice = { path = "../hyperlattice" }
```

Features:

| Feature | Default | Purpose |
| --- | --- | --- |
| `arbitrary` | no | Implements `arbitrary::Arbitrary` for lattice-owned types. |
| `hyperreal-dispatch-trace` | no | Enables hyperreal dispatch tracing during benchmarks. |

## Examples

### Real Values

```rust
use hyperlattice::{Real, ln, log10, pi, sqrt, tau};

fn r(value: i32) -> Real {
    value.into()
}

let nine = r(9);
assert_eq!(sqrt(nine).unwrap(), r(3));
assert_eq!(tau(), r(2) * pi());
assert_eq!(ln(hyperlattice::e()).unwrap(), r(1));
assert_eq!(log10(r(100)).unwrap(), r(2));
```

### Vectors

```rust
use hyperlattice::{Rational, Real, Vector3, one};

fn r(value: i32) -> Real {
    value.into()
}

let v = Vector3::new([r(3), r(4), r(0)]);
let offset = v.clone() + r(10);

assert_eq!(v.dot(&v), r(25));
assert_eq!(offset, Vector3::new([r(13), r(14), r(10)]));
assert_eq!(v.normalize().unwrap().dot(&v.normalize().unwrap()), one());

let half = Rational::fraction(1, 2).unwrap().into();
let displayed = Vector3::new([half, r(2), r(3)]);
assert_eq!(format!("{displayed}"), "[1/2, 2, 3]");
assert_eq!(format!("{displayed:#}"), "[0.5, 2, 3]");
```

### Matrices

```rust
use hyperlattice::{Matrix3, Real};

fn r(value: i32) -> Real {
    value.into()
}

let matrix = Matrix3::new([
    [r(1), r(2), r(3)],
    [r(0), r(1), r(4)],
    [r(5), r(6), r(0)],
]);

assert_eq!(matrix.determinant(), r(1));
assert_eq!(matrix.clone() * matrix.clone().inverse().unwrap(), Matrix3::identity());
assert_eq!((matrix ^ 0).unwrap(), Matrix3::identity());
```

### Structural Facts

```rust
use hyperlattice::{RealSign, ZeroStatus, pi};

let facts = pi().structural_facts();
assert_eq!(facts.sign, Some(RealSign::Positive));
assert_eq!(facts.zero, ZeroStatus::NonZero);
assert!(!facts.exact_rational);

let approx = pi().to_f64_lossy().unwrap();
assert!(approx > 3.0 && approx < 4.0);
```

## Performance Notes

The crate is optimized for small fixed-size algebra over rich exact values:

- borrowed arithmetic keeps expression cloning low
- short product-sum reducers preserve exact-rational structure
- vector, matrix, and complex operations use owned-left/borrowed-right hot paths
- small powers are specialized before exponentiation by squaring
- 3x3 and 4x4 matrix multiplication is unrolled
- checked inverses use exact zero-status paths
- matrix structural facts expose semantic row/column zero certificates and
  determinant schedule categories, so consumers can pick exact kernels without
  depending on mask layouts
- prepared matrix and right-divisor handles expose `MatrixPreparedCacheState`,
  a docs.rs-visible cache availability summary for determinant, reciprocal,
  minor-factor, adjugate, and inverse reuse without exposing cached scalar
  storage
- homogeneous matrix facts carry conservative transform-kind provenance for
  affine, translation, diagonal-linear, signed-permutation, and projective
  dispatch
- structural facts are forwarded by borrow so `hyperlimit` can query them cheaply

Run the benchmark suite:

```sh
cargo bench --bench mathbench
```

Run dispatch tracing separately:

```sh
cargo bench --bench mathbench --features hyperreal-dispatch-trace -- --write-dispatch-trace-md
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
