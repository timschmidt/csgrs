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

| Benchmark | Approx backend | Realistic backend | astro-float 128 | arpfloat 128 | Realistic / approx |
| --- | ---: | ---: | ---: | ---: | ---: |
| `vectors/vec3 dot` | 6.9478 ns | 5.0714 us | 301.84 ns | 745.19 ns | 729.93x |
| `vectors/vec3 magnitude` | 10.498 ns | 8.4877 us | 5.8792 us | 14.950 us | 808.51x |
| `vectors/vec3 normalize` | 22.012 ns | 15.371 us | 6.4802 us | 19.569 us | 698.30x |
| `matrix3/mat3 determinant` | 15.824 ns | 13.482 us | 1.0505 us | 2.5926 us | 852.00x |
| `matrix3/mat3 inverse` | 141.12 ns | 79.398 us | 3.3588 us | 10.558 us | 562.63x |
| `matrix3/mat3 mul mat3` | 82.903 ns | 47.638 us | 2.8978 us | 8.0788 us | 574.62x |
| `matrix3/mat3 transform vec3` | 16.140 ns | 17.211 us | 1.1502 us | 2.6311 us | 1066.36x |
| `matrix4/mat4 determinant` | 76.050 ns | 10.043 us | 4.5701 us | 8.6799 us | 132.06x |
| `matrix4/mat4 inverse` | 212.69 ns | 36.395 us | 11.709 us | 33.774 us | 171.12x |
| `matrix4/mat4 mul mat4` | 93.995 ns | 16.655 us | 6.2529 us | 15.411 us | 177.19x |
| `matrix4/mat4 transform vec4` | 40.296 ns | 5.3458 us | 1.8886 us | 4.3025 us | 132.66x |
| `scalar_trig/sin` | 11.624 ns | 1.8208 us | 11.773 us | 262.82 us | 156.64x |
| `scalar_trig/cos` | 11.920 ns | 441.43 ns | 10.167 us | 170.19 us | 37.03x |
