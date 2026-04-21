# realistic_blas

`realistic_blas` is a small Rust linear algebra library built around
[`realistic::Real`](https://crates.io/crates/realistic), a real-number type with
symbolic and computable-number behavior.

The crate provides scalar helpers, complex numbers, 3D/4D vectors, and 3x3/4x4
matrices using `Real` throughout.

## Features

- Re-exports `realistic::{Real, Rational, Problem}`.
- Constants and scalar helpers: `zero`, `one`, `e`, `pi`, `tau`, `i`,
  `reciprocal`, `pow`, `powi`.
- Elementary functions: `exp`, `ln`, `sqrt`, `sin`, `cos`, `tan`.
- Hyperbolic functions: `sinh`, `cosh`, `tanh`.
- Inverse trigonometric and hyperbolic helpers: `asin`, `acos`, `atan`, `asinh`,
  `acosh`, `atanh`.
- `Complex` with arithmetic, reciprocal, conjugate, and integer powers.
- `Vector3` and `Vector4` with arithmetic, scalar division, dot product,
  magnitude, and normalization.
- `Matrix3` and `Matrix4` with arithmetic, multiplication, scalar division,
  matrix division, integer powers via `^`, transpose, determinant, inverse, and
  reciprocal.

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

let nine: Real = 9.into();
let three = sqrt(nine).unwrap();
assert_eq!(three, 3.into());

assert_eq!(tau(), 2.into() * pi());
assert_eq!(ln(realistic_blas::e()).unwrap(), 1.into());
```

Many operations are fallible because `Real` arithmetic can fail for invalid
domains, division by zero, or unsupported conversions. Fallible helpers return:

```rust
type BlasResult<T> = Result<T, realistic_blas::Problem>;
```

### Complex Numbers

```rust
use realistic_blas::{i, Complex};

let minus_one = Complex::new((-1).into(), 0.into());
assert_eq!((i() ^ 2).unwrap(), minus_one);
```

### Vectors

```rust
use realistic_blas::{one, Vector3};

let v = Vector3::new([3.into(), 4.into(), 0.into()]);

assert_eq!(v.dot(&v), 25.into());

let unit = v.normalize().unwrap();
assert_eq!(unit.dot(&unit), one());
```

### Matrices

```rust
use realistic_blas::Matrix3;

let matrix = Matrix3::new([
    [1.into(), 2.into(), 3.into()],
    [0.into(), 1.into(), 4.into()],
    [5.into(), 6.into(), 0.into()],
]);

assert_eq!(matrix.determinant(), 1.into());
assert_eq!(matrix.clone() * matrix.clone().inverse().unwrap(), Matrix3::identity());
assert_eq!((matrix.clone() ^ 0).unwrap(), Matrix3::identity());
```

## Notes

`realistic::Real` does not currently expose native inverse trigonometric or
inverse hyperbolic methods. The inverse helper functions in this crate convert
through `f64` and then back into `Real`, so they are approximate rather than
symbolic.

Matrix inversion uses Gauss-Jordan elimination. A matrix is treated as singular
when no definitely non-zero pivot can be found.

## Development

Run the standard checks:

```sh
cargo fmt --check
cargo test
cargo clippy --all-targets --all-features -- -D warnings
```
