# realistic_blas

`realistic_blas` is a small Rust linear algebra library built around a
crate-owned `Scalar` type.

The crate provides scalar helpers, complex numbers, 3D/4D vectors, and 3x3/4x4
matrices using `Scalar` throughout. `Scalar`, `Complex`, `Vector3`, `Vector4`,
`Matrix3`, and `Matrix4` are generic over a backend marker and default to the
feature-selected `DefaultBackend`. By default, `Scalar` is backed by
[`realistic::Real`](https://crates.io/crates/realistic), while the approximate
backend is also available explicitly. `approx-backend` uses an `f64` value plus
an `f64` epsilon to model approximate error bounds and unknown-zero conditions.

## Features

- Re-exports `realistic::{Real, Rational}` for explicit construction and interop
  when `realistic-backend` is enabled. Library operations use crate-owned
  `Scalar` and `Problem`.
- Exposes `RealisticBackend`, `ApproxBackend`, and `DefaultBackend` markers so
  both backends can be used in one build when both backend features are enabled.
- Constants and scalar helpers: `zero`, `one`, `e`, `pi`, `tau`, `i`,
  `reciprocal`, `reciprocal_checked`, `pow`, `powi`.
- Elementary functions: `exp`, `ln`, `log10`, `sqrt`, `sin`, `cos`, `tan`.
- Hyperbolic functions: `sinh`, `cosh`, `tanh`.
- Inverse trigonometric and hyperbolic helpers: `asin`, `acos`, `atan`, `asinh`,
  `acosh`, `atanh`.
- `ZeroStatus`, `Problem`, and `CheckedBlasResult` for APIs that reject unknown
  zero conditions instead of proceeding optimistically.
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

Add the crate to your project:

```toml
[dependencies]
realistic_blas = { path = "path/to/realistic_blas" }
```

The default feature set enables both backends. The realistic backend depends on:

```toml
realistic = "0.8.1"
num = "0.4.3"
```

The approximate `f64 + epsilon` backend has no normal dependencies on
`realistic` or `num`. To use it:

```toml
[dependencies]
realistic_blas = {
    path = "path/to/realistic_blas",
    default-features = false,
    features = ["approx-backend"],
}
```

Backend features gate availability rather than changing the shared API shape.
When both `realistic-backend` and `approx-backend` are enabled,
`DefaultBackend` remains `RealisticBackend` and approximate values can be
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
use realistic_blas::{ApproxBackend, RealisticBackend, Scalar, Vector3};

let exact: Scalar<RealisticBackend> = Scalar::try_from(1.25).unwrap();
let approx: Scalar<ApproxBackend> = Scalar::<ApproxBackend>::approx(1.25, 0.01).unwrap();

let exact_vector = Vector3::<RealisticBackend>::new([exact.clone(), exact.clone(), exact]);
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

For computations that may force realistic backend evaluation, callers can attach
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

The formatting examples above use the default realistic backend. With the approx
backend, `Rational` is not available and normal formatting prints approximate
decimal center values.

## Source Layout

The crate root re-exports the public API from focused modules:

- `src/scalar.rs`: scalar constants and functions around `Scalar`.
- `src/complex.rs`: `Complex` and complex arithmetic.
- `src/vector.rs`: `Vector3`, `Vector4`, and vector operations.
- `src/matrix.rs`: `Matrix3`, `Matrix4`, and matrix operations.
- `src/backend/realistic`: realistic-backed `Scalar` implementation.
- `src/backend/approx`: approximate `f64 + epsilon` `Scalar` implementation.

## Notes

When the realistic backend is selected, `realistic::Real` does not currently
expose native inverse trigonometric or inverse hyperbolic methods. The inverse
helper functions convert through `f64` and then back into `Scalar`, so they are
approximate rather than symbolic.

The approx backend stores a center value and an absolute error bound. A scalar
with an interval containing zero reports `ZeroStatus::Unknown`, so checked
division, normalization, and matrix inversion exercise the same unknown-zero API
surface as the realistic backend.

Division-sensitive operations have two API paths. The checked path uses
`zero_status` and rejects both definite zero and `ZeroStatus::Unknown`.
Abort-aware checked variants attach an `AbortSignal` before running those zero
classification checks. The default realistic backend keeps the ordinary path
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
