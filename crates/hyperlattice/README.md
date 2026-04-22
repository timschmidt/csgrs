# realistic_blas

`realistic_blas` is a small Rust linear algebra library built around a
crate-owned `Scalar` type.

The crate provides scalar helpers, complex numbers, 3D/4D vectors, and 3x3/4x4
matrices using `Scalar` throughout. By default, `Scalar` is backed by
[`realistic::Real`](https://crates.io/crates/realistic). The optional
`approx-backend` uses an `f64` value plus an `f64` epsilon to model approximate
error bounds and unknown-zero conditions.

## Features

- Re-exports `realistic::{Real, Rational}` for explicit construction and interop
  when `realistic-backend` is enabled. Library operations use crate-owned
  `Scalar` and `Problem`.
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

The default feature set enables the realistic backend, which depends on:

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

The `realistic-backend` and `approx-backend` features are mutually exclusive.
`cargo --all-features` is therefore expected to fail with a feature-selection
error.

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
cargo test --all-targets --no-default-features --features approx-backend
cargo clippy --all-targets -- -D warnings
cargo clippy --all-targets --no-default-features --features approx-backend -- -D warnings
```

Do not use `--all-features` for normal validation because the backend features
are intentionally mutually exclusive.

Run the Criterion benchmark suite:

```sh
cargo bench --bench mathbench
cargo bench --bench mathbench --no-default-features --features approx-backend
```

The benchmark names mirror a small mathbench-style subset: vector dot,
magnitude, normalize, matrix determinant, inverse, matrix multiplication,
matrix-vector transforms for 3x3/4x4 types, and scalar trigonometric functions.

### Benchmark Results

The following Criterion median estimates were collected on an AMD Ryzen 7
5800X3D on Fedora. They compare this crate's two scalar backends with
`astro-float` and `arpfloat` comparison backends. The `mathbench` comparison
suite runs both comparison backends at 128-bit precision.

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.57 ns | 4.38 us | 6.32 us | 301.70 ns | 764.51 ns | 666.32x | 14.50x | 5.72x |
| `vec3 mag` | 10.73 ns | 7.64 us | 10.15 us | 5.88 us | 15.03 us | 711.73x | 1.30x | 0.51x |
| `vec3 norm` | 21.95 ns | 13.04 us | 13.20 us | 6.35 us | 19.98 us | 594.17x | 2.05x | 0.65x |
| `mat3 det` | 15.51 ns | 13.78 us | 3.42 us | 1.05 us | 2.71 us | 888.37x | 13.11x | 5.08x |
| `mat3 inv` | 62.15 ns | 59.15 us | 14.05 us | 3.31 us | 12.29 us | 951.72x | 17.86x | 4.81x |
| `mat3 mul` | 79.12 ns | 41.61 us | 13.27 us | 2.90 us | 8.79 us | 525.89x | 14.33x | 4.73x |
| `mat3 x vec3` | 14.87 ns | 15.20 us | 12.39 us | 1.13 us | 2.87 us | 1021.72x | 13.45x | 5.30x |
| `mat4 det` | 46.62 ns | 5.40 us | 5.36 us | 4.65 us | 9.31 us | 115.79x | 1.16x | 0.58x |
| `mat4 inv` | 131.64 ns | 19.92 us | 19.39 us | 11.77 us | 35.04 us | 151.35x | 1.69x | 0.57x |
| `mat4 mul` | 138.62 ns | 15.51 us | 15.14 us | 6.19 us | 16.93 us | 111.90x | 2.51x | 0.92x |
| `mat4 x vec4` | 24.61 ns | 5.14 us | 5.06 us | 1.90 us | 4.31 us | 208.65x | 2.70x | 1.19x |
| `sin 0.1` | 10.84 ns | 2.25 us | 2.40 us | 10.42 us | 124.97 us | 207.12x | 0.22x | 0.02x |
| `cos 0.1` | 11.57 ns | 225.34 ns | 224.45 ns | 10.05 us | 25.53 us | 19.48x | 0.02x | 0.01x |
| `sin 1.23456789` | 11.70 ns | 1.88 us | 1.89 us | 11.89 us | 264.47 us | 160.72x | 0.16x | 0.01x |
| `cos 1.23456789` | 11.97 ns | 415.14 ns | 420.92 ns | 10.16 us | 171.65 us | 34.67x | 0.04x | 0.00x |
| `sin 1e6` | 12.56 ns | 35.17 us | 35.24 us | 15.92 us | 267.64 us | 2800.21x | 2.21x | 0.13x |
| `cos 1e6` | 12.32 ns | 21.64 us | 22.01 us | 13.54 us | 171.76 us | 1756.35x | 1.60x | 0.13x |
| `sin 1e30` | 65.99 ns | 186.54 us | 185.02 us | 18.71 us | 275.44 us | 2826.92x | 9.97x | 0.68x |
| `cos 1e30` | 68.18 ns | 139.53 us | 137.80 us | 15.35 us | 172.42 us | 2046.61x | 9.09x | 0.81x |
| `sin pi_7` | 11.67 ns | 2.41 us | 4.35 us | 11.59 us | 124.49 us | 206.50x | 0.21x | 0.02x |
| `cos pi_7` | 11.62 ns | 214.98 ns | 4.49 us | 10.32 us | 27.50 us | 18.51x | 0.02x | 0.01x |
| `sin 1000pi_eps` | 11.84 ns | 23.04 us | 33.87 us | 16.00 us | 264.31 us | 1945.37x | 1.44x | 0.09x |
| `cos 1000pi_eps` | 12.29 ns | 13.79 us | 24.52 us | 13.54 us | 155.22 us | 1122.38x | 1.02x | 0.09x |
