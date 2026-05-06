# realistic_blas

`realistic_blas` is a small Rust linear algebra library built around a
crate-owned `Scalar` type. It is the vector/matrix layer in the local geometry
stack, sitting between semantics-rich scalar backends such as `hyperreal` and
geometry-policy crates such as `predicated`.

The crate provides scalar helpers, complex numbers, 3D/4D vectors, and 3x3/4x4
matrices using `Scalar` throughout. `Scalar`, `Complex`, `Vector3`, `Vector4`,
`Matrix3`, and `Matrix4` are generic over a backend marker and default to the
feature-selected `DefaultBackend`. By default, `Scalar` is backed by
`hyperreal::Real` from the published `hyperreal` crate. The approximate backend
is also available explicitly. `approx-backend` uses an `f64` value plus an `f64`
epsilon to model approximate error bounds and unknown-zero conditions.

The crate intentionally does not own robust geometry predicates such as
`orient2d`, `orient3d`, or point/plane classification. Instead it exposes
backend-neutral scalar facts so a predicate layer can exploit known sign,
zero/non-zero status, exact-rational availability, magnitude hints, and
borrowed finite approximations without depending directly on a backend's
private representation.

In the current stack:

- `hyperreal` supplies the default exact/symbolic scalar backend.
- `realistic_blas` owns complex, vector, and matrix algebra over that scalar
  abstraction, plus an explicit approximate backend for comparison and
  lighter-weight workflows.
- `predicated` consumes scalar facts from `realistic_blas::Scalar<B>` when
  geometric predicates need policy, robust fallback, or exact sign escalation.

This crate deliberately preserves those boundaries. It forwards introspection
facts from scalar backends, but it does not decide geometry topology or robust
predicate escalation policy.

## Current status

`realistic_blas` is an experimental but actively benchmarked `0.3.1` crate. The
current focus is ergonomic exact/symbolic algebra over small fixed-size
objects, not raw dense linear algebra throughput. Recent work has reduced clone
pressure in borrowed and mixed owned/borrowed scalar paths, added backend hooks
for borrowed arithmetic, and made scalar introspection cheap enough for
predicate filters to call frequently.

The benchmark suite compares:

- exact/symbolic hyperreal-backed scalar, vector, matrix, and trig operations
- the approximate backend
- external numeric libraries used as throughput reference points
- borrowed, owned, and mixed scalar operation forms

See [`benchmarks.md`](./benchmarks.md) for the generated Criterion summary.

## Features

- Re-exports `hyperreal::{Real, Rational}` for explicit construction and interop
  when `hyperreal-backend` is enabled. Library operations use crate-owned
  `Scalar` and `Problem`.
- Exposes `HyperrealBackend`, `ApproxBackend`, and `DefaultBackend` markers so
  both backends can be used in one build when both backend features are enabled.
- Constants and scalar helpers: `zero`, `one`, `e`, `pi`, `tau`, `i`,
  `reciprocal`, `reciprocal_checked`, `pow`, `powi`.
- Elementary functions: `exp`, `ln`, `log10`, `sqrt`, `sin`, `cos`, `tan`.
- Hyperbolic functions: `sinh`, `cosh`, `tanh`.
- Inverse trigonometric and hyperbolic helpers: `asin`, `acos`, `atan`, `asinh`,
  `acosh`, `atanh`.
- `ZeroStatus`, `Problem`, and `CheckedBlasResult` for APIs that reject unknown
  zero conditions instead of proceeding optimistically.
- `ScalarSign`, `ScalarFacts`, and `ScalarMagnitudeBits` for conservative
  backend-neutral structural facts used by robust predicate layers.
- Borrowed scalar inspection via `Scalar::structural_facts`,
  `Scalar::refine_sign_until`, and `Scalar::to_f64_approx`. These APIs are the
  current handoff surface for `predicated`.
- `AbortSignal` and `_with_abort` variants for zero-sensitive or conversion
  APIs that may need cancellable `Real` evaluation.
- `Complex` with arithmetic, reciprocal, checked reciprocal, conjugate, and
  integer powers, plus symbolic and alternate decimal display formatting.
- `Vector3` and `Vector4` with componentwise vector/vector arithmetic,
  componentwise vector/scalar addition and subtraction, scalar multiplication
  and division, checked scalar division, dot product, magnitude, normalization,
  checked normalization, abort-aware checked division/normalization, and
  symbolic and alternate decimal display formatting.
- `Matrix3` and `Matrix4` with componentwise matrix/matrix arithmetic,
  componentwise matrix/scalar addition and subtraction, matrix multiplication,
  scalar division, checked scalar division, matrix division, checked matrix
  division, integer powers via `^`, checked integer powers, transpose,
  determinant, inverse, checked inverse, reciprocal, checked reciprocal, and
  abort-aware checked division/inversion/power helpers, and symbolic and
  alternate decimal display formatting.

## Install

From the local sibling checkout used by this stack:

```toml
[dependencies]
realistic_blas = { path = "../realistic_blas" }
```

When using a published release, depend on the matching crate version:

```toml
[dependencies]
realistic_blas = "0.3.1"
```

The default feature set enables both backends. In this workspace the
`hyperreal-backend` feature depends on the sibling `hyperreal` crate; when
published, it should use the matching published `hyperreal` version. The
current local stack is aligned with:

```toml
hyperreal = "0.10.4"
num = "0.4.3"
```

The approximate `f64 + epsilon` backend has no normal dependencies on
`hyperreal` or `num`. To use it:

```toml
[dependencies]
realistic_blas = {
    path = "../realistic_blas",
    default-features = false,
    features = ["approx-backend"],
}
```

Backend features gate availability rather than changing the shared API shape.
When both `hyperreal-backend` and `approx-backend` are enabled,
`DefaultBackend` remains `HyperrealBackend` and approximate values can be
requested explicitly with types such as `Scalar<ApproxBackend>` or
`Vector3<ApproxBackend>`.

## Examples

### Scalars

```rust
use realistic_blas::{ln, log10, pi, sqrt, tau, Scalar};

fn s(value: i32) -> Scalar {
    value.into()
}

let nine: Scalar = 9.into();
let three = sqrt(nine).unwrap();
assert_eq!(three, s(3));

assert_eq!(tau(), s(2) * pi());
assert_eq!(ln(realistic_blas::e()).unwrap(), s(1));
assert_eq!(log10(s(100)).unwrap(), s(2));
```

### Explicit Backends

```rust
use realistic_blas::{ApproxBackend, HyperrealBackend, Scalar, Vector3};

let exact: Scalar<HyperrealBackend> = Scalar::try_from(1.25).unwrap();
let approx: Scalar<ApproxBackend> = Scalar::<ApproxBackend>::approx(1.25, 0.01).unwrap();

let exact_vector = Vector3::<HyperrealBackend>::new([exact.clone(), exact.clone(), exact]);
let approx_vector = Vector3::<ApproxBackend>::new([approx.clone(), approx.clone(), approx]);

assert_eq!(exact_vector.0.len(), approx_vector.0.len());
```

Many operations are fallible because scalar arithmetic can fail for invalid
domains, division by zero, unknown zero conditions, or unsupported conversions.
Fallible helpers return:

```rust
type BlasResult<T> = Result<T, realistic_blas::Problem>;
```

Checked helpers reject definite zero and unknown-zero cases:

```rust
type CheckedBlasResult<T> = Result<T, realistic_blas::Problem>;
```

For computations that may force hyperreal backend evaluation, callers can attach
a cancellation flag before calling into `realistic_blas`, or use the provided
abort-aware checked helpers. The approx backend accepts these APIs as no-ops.

```rust
use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use realistic_blas::{AbortSignal, Vector3};

let signal: AbortSignal = Arc::new(AtomicBool::new(false));
let vector = Vector3::new([3.into(), 4.into(), 0.into()]);
let unit = vector.normalize_checked_with_abort(&signal).unwrap();

signal.store(true, std::sync::atomic::Ordering::Relaxed);
```

### Structural Scalar Facts

`Scalar` exposes backend-neutral facts for downstream robust predicate crates.
These APIs are borrowed and conservative: missing sign or magnitude information
means the backend did not prove the fact cheaply.

```rust
use realistic_blas::{ScalarSign, ZeroStatus, pi};

let facts = pi().structural_facts();
assert_eq!(facts.sign, Some(ScalarSign::Positive));
assert_eq!(facts.zero, ZeroStatus::NonZero);
assert!(!facts.exact_rational);

let approx = pi().to_f64_approx().unwrap();
assert!(approx > 3.0 && approx < 4.0);
```

The Hyperreal backend forwards `Real::structural_facts`,
`Real::refine_sign_until`, and `Real::to_f64_approx`. The approx backend derives
the same public facts from its stored `value +/- epsilon` interval.

`realistic_blas` only forwards facts. Predicate escalation policy, robust
fallbacks, exact determinant paths, and topology classification belong in
`predicated` or another geometry-specific crate.

## Relationship to the other crates

- Use `hyperreal` directly when you need exact rational, symbolic, or computable
  scalar arithmetic without vectors or matrices.
- Use `realistic_blas` when you need small fixed-size vectors, matrices,
  complex numbers, and scalar functions over either exact/symbolic or
  approximate scalar backends.
- Use `predicated` when you need geometry predicates, classification, and
  provenance for how a sign/topology decision was made.

### Complex Numbers

```rust
use realistic_blas::{i, Complex};

let minus_one = Complex::new((-1).into(), 0.into());
assert_eq!((i() ^ 2).unwrap(), minus_one);
```

### Vectors

```rust
use realistic_blas::{one, Rational, Scalar, Vector3};

fn s(value: i32) -> Scalar {
    value.into()
}

let v = Vector3::new([s(3), s(4), s(0)]);
let offset = v.clone() + s(10);

assert_eq!(v.dot(&v), s(25));
assert_eq!(offset, Vector3::new([s(13), s(14), s(10)]));

let unit = v.normalize().unwrap();
assert_eq!(unit.dot(&unit), one());

let half = Rational::fraction(1, 2).unwrap().into();
let displayed = Vector3::new([half, s(2), s(3)]);
assert_eq!(format!("{displayed}"), "[1/2, 2, 3]");
assert_eq!(format!("{displayed:#}"), "[0.5, 2, 3]");
```

### Matrices

```rust
use realistic_blas::{Matrix3, Scalar};

fn s(value: i32) -> Scalar {
    value.into()
}

let matrix = Matrix3::new([
    [s(1), s(2), s(3)],
    [s(0), s(1), s(4)],
    [s(5), s(6), s(0)],
]);
let incremented = matrix.clone() + s(1);

assert_eq!(matrix.determinant(), s(1));
assert_eq!(
    incremented,
    Matrix3::new([
        [s(2), s(3), s(4)],
        [s(1), s(2), s(5)],
        [s(6), s(7), s(1)],
    ])
);
assert_eq!(matrix.clone() * matrix.clone().inverse().unwrap(), Matrix3::identity());
assert_eq!((matrix.clone() ^ 0).unwrap(), Matrix3::identity());
```

## Formatting

`Complex`, `Vector3`, `Vector4`, `Matrix3`, and `Matrix4` implement `Display`.
With the realistic backend, normal formatting forwards each component to
`Real`'s symbolic display, while alternate formatting forwards to `Real`'s
decimal display. With the approx backend, both forms display the approximate
center value.

```rust
use realistic_blas::{Matrix3, Rational, Scalar};

fn s(value: i32) -> Scalar {
    value.into()
}

let half = Rational::fraction(1, 2).unwrap().into();
let matrix = Matrix3::new([[half, s(2), s(3)], [s(4), s(5), s(6)], [s(7), s(8), s(9)]]);

assert_eq!(format!("{matrix}"), "[[1/2, 2, 3], [4, 5, 6], [7, 8, 9]]");
assert_eq!(format!("{matrix:#}"), "[[0.5, 2, 3], [4, 5, 6], [7, 8, 9]]");
```

The formatting examples above use the default hyperreal backend. With the approx
backend, `Rational` is not available and normal formatting prints approximate
decimal center values.

## Source Layout

The crate root re-exports the public API from focused modules:

- `src/scalar.rs`: scalar constants, functions, zero status, and structural
  fact types around `Scalar`.
- `src/complex.rs`: `Complex` and complex arithmetic.
- `src/vector.rs`: `Vector3`, `Vector4`, and vector operations.
- `src/matrix.rs`: `Matrix3`, `Matrix4`, and matrix operations.
- `src/backend/hyperreal`: hyperreal-backed `Scalar` implementation.
- `src/backend/approx`: approximate `f64 + epsilon` `Scalar` implementation.

## Notes

### Current Integration State

- `realistic_blas` has implemented its part of the predicate-integration plan:
  scalar fact forwarding through `Scalar<B>`.
- The Hyperreal backend uses the upstream structural and inverse elementary
  function APIs from `hyperreal`.
- The approx backend maps its interval model into conservative `ScalarFacts`.
- Full robust predicate implementation is not in this crate. The next layer is
  expected to consume these APIs from `predicated`.

When the hyperreal backend is selected, inverse trigonometric and inverse
hyperbolic helpers dispatch to native `hyperreal::Real` methods, preserving
exact symbolic paths where `hyperreal` can represent them. The approx backend
implements the same helpers with its local `f64 + epsilon` interval model.

The approx backend stores a center value and an absolute error bound. A scalar
with an interval containing zero reports `ZeroStatus::Unknown`, so checked
division, normalization, and matrix inversion exercise the same unknown-zero API
surface as the hyperreal backend.

Division-sensitive operations have two API paths. The checked path uses
`zero_status` and rejects both definite zero and `ZeroStatus::Unknown`.
Abort-aware checked variants attach an `AbortSignal` before running those zero
classification checks. The default hyperreal backend keeps the ordinary path
optimistic where possible; the approx backend may return `Problem::UnknownZero`
from ordinary arithmetic when an interval contains zero.

Matrix inversion uses Gauss-Jordan elimination. Ordinary inversion picks a pivot
that is not definitely zero. Checked inversion requires a pivot classified as
`ZeroStatus::NonZero`.

Scalar addition and subtraction are implemented as `Vector3 + Scalar`,
`Vector4 - Scalar`, `Matrix3 + Scalar`, and similar left-hand vector/matrix
forms. The reverse forms, such as `Scalar + Vector3`, cannot be implemented
directly because Rust's orphan rules forbid implementing a standard-library
trait for an external left-hand type.

## Development

Run the standard checks:

```sh
cargo fmt --check
cargo test --all-targets
cargo test --all-targets --all-features
cargo test --all-targets --no-default-features --features approx-backend
cargo clippy --all-targets -- -D warnings
cargo clippy --all-targets --all-features -- -D warnings
cargo clippy --all-targets --no-default-features --features approx-backend -- -D warnings
```

Use `--all-features` to validate that explicit backend type parameters can use
the realistic and approximate backends in the same build.

Run the Criterion benchmark suite:

```sh
cargo bench --bench mathbench
```

See [benchmarks.md](benchmarks.md) for operation coverage and benchmark
results. A completed `cargo bench --bench mathbench` run rewrites that file
from Criterion's saved median estimates.
