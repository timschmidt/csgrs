# realistic_blas

`realistic_blas` is a small Rust linear algebra library built around
[`realistic::Real`](https://crates.io/crates/realistic), a real-number type with
symbolic and computable-number behavior.

The crate provides scalar helpers, complex numbers, 3D/4D vectors, and 3x3/4x4
matrices using `Real` throughout.

## Features

- Re-exports `realistic::{Real, Rational, Problem}`.
- Constants and scalar helpers: `zero`, `one`, `e`, `pi`, `tau`, `i`,
  `reciprocal`, `reciprocal_checked`, `pow`, `powi`.
- Elementary functions: `exp`, `ln`, `sqrt`, `sin`, `cos`, `tan`.
- Hyperbolic functions: `sinh`, `cosh`, `tanh`.
- Inverse trigonometric and hyperbolic helpers: `asin`, `acos`, `atan`, `asinh`,
  `acosh`, `atanh`.
- `ZeroStatus`, `BlasProblem`, and `CheckedBlasResult` for APIs that reject
  unknown zero conditions instead of proceeding optimistically.
- `Complex` with arithmetic, reciprocal, checked reciprocal, conjugate, and
  integer powers.
- `Vector3` and `Vector4` with componentwise vector/vector arithmetic,
  componentwise vector/scalar addition and subtraction, scalar multiplication
  and division, checked scalar division, dot product, magnitude, normalization,
  and checked normalization.
- `Matrix3` and `Matrix4` with componentwise matrix/matrix arithmetic,
  componentwise matrix/scalar addition and subtraction, matrix multiplication,
  scalar division, checked scalar division, matrix division, checked matrix
  division, integer powers via `^`, checked integer powers, transpose,
  determinant, inverse, checked inverse, reciprocal, and checked reciprocal.

## Install

Add the crate to your project:

```toml
[dependencies]
realistic_blas = { path = "path/to/realistic_blas" }
```

This project depends on:

```toml
realistic = "0.8.1"
```

## Examples

### Scalars

```rust
use realistic_blas::{ln, pi, sqrt, tau, Real};

fn r(value: i32) -> Real {
    value.into()
}

let nine: Real = 9.into();
let three = sqrt(nine).unwrap();
assert_eq!(three, r(3));

assert_eq!(tau(), r(2) * pi());
assert_eq!(ln(realistic_blas::e()).unwrap(), r(1));
```

Many operations are fallible because `Real` arithmetic can fail for invalid
domains, division by zero, or unsupported conversions. Fallible helpers return:

```rust
type BlasResult<T> = Result<T, realistic_blas::Problem>;
```

Checked helpers reject definite zero and unknown-zero cases:

```rust
type CheckedBlasResult<T> = Result<T, realistic_blas::BlasProblem>;
```

### Complex Numbers

```rust
use realistic_blas::{i, Complex};

let minus_one = Complex::new((-1).into(), 0.into());
assert_eq!((i() ^ 2).unwrap(), minus_one);
```

### Vectors

```rust
use realistic_blas::{one, Real, Vector3};

fn r(value: i32) -> Real {
    value.into()
}

let v = Vector3::new([r(3), r(4), r(0)]);
let offset = v.clone() + r(10);

assert_eq!(v.dot(&v), r(25));
assert_eq!(offset, Vector3::new([r(13), r(14), r(10)]));

let unit = v.normalize().unwrap();
assert_eq!(unit.dot(&unit), one());
```

### Matrices

```rust
use realistic_blas::{Matrix3, Real};

fn r(value: i32) -> Real {
    value.into()
}

let matrix = Matrix3::new([
    [r(1), r(2), r(3)],
    [r(0), r(1), r(4)],
    [r(5), r(6), r(0)],
]);
let incremented = matrix.clone() + r(1);

assert_eq!(matrix.determinant(), r(1));
assert_eq!(
    incremented,
    Matrix3::new([
        [r(2), r(3), r(4)],
        [r(1), r(2), r(5)],
        [r(6), r(7), r(1)],
    ])
);
assert_eq!(matrix.clone() * matrix.clone().inverse().unwrap(), Matrix3::identity());
assert_eq!((matrix.clone() ^ 0).unwrap(), Matrix3::identity());
```

## Source Layout

The crate root re-exports the public API from focused modules:

- `src/scalar.rs`: scalar constants and functions around `Real`.
- `src/complex.rs`: `Complex` and complex arithmetic.
- `src/vector.rs`: `Vector3`, `Vector4`, and vector operations.
- `src/matrix.rs`: `Matrix3`, `Matrix4`, and matrix operations.

## Notes

`realistic::Real` does not currently expose native inverse trigonometric or
inverse hyperbolic methods. The inverse helper functions in this crate convert
through `f64` and then back into `Real`, so they are approximate rather than
symbolic.

Division-sensitive operations have two API paths. The ordinary path rejects
values that are definitely zero and proceeds otherwise. The checked path uses
`zero_status` and rejects both definite zero and `ZeroStatus::Unknown`.

Matrix inversion uses Gauss-Jordan elimination. Ordinary inversion picks a pivot
that is not definitely zero. Checked inversion requires a pivot classified as
`ZeroStatus::NonZero`.

Scalar addition and subtraction are implemented as `Vector3 + Real`,
`Vector4 - Real`, `Matrix3 + Real`, and similar left-hand vector/matrix forms.
The reverse forms, such as `Real + Vector3`, cannot be implemented directly
because Rust's orphan rules forbid implementing a standard-library trait for an
external left-hand type.

## Development

Run the standard checks:

```sh
cargo fmt --check
cargo test
cargo clippy --all-targets --all-features -- -D warnings
```
