<h1>
  hyperlattice
  <img src="./doc/hyperlattice.png" alt="Hyper, a clever mathematician" width="144" align="right">
</h1>

`hyperlattice` provides small fixed-size linear algebra over a crate-owned
`Scalar` type.

It includes scalars, complex numbers, 3D/4D vectors, and 3x3/4x4 matrices. The
public types are generic over backend markers:

- `HyperrealBackend`: exact/symbolic scalars backed by `hyperreal::Real`
- `ApproxBackend`: an `f64` center value plus an absolute `f64` error bound
- `DefaultBackend`: `HyperrealBackend` when `hyperreal-backend` is enabled,
  otherwise `ApproxBackend` when only `approx-backend` is enabled

The default feature set enables both backends. The default scalar representation
remains `HyperrealBackend`.

## Relationship to Other Crates

- `hyperreal` supplies the default exact/symbolic scalar representation.
- `hyperlattice` owns complex, vector, and matrix algebra over backend-neutral
  `Scalar<B>` values.
- `liminal` can consume `hyperlattice::Scalar<B>` structural facts when
  geometry predicates need sign provenance, filtering, refinement, or robust
  fallback.

`hyperlattice` forwards scalar facts. It does not own robust predicate policy
or geometry topology.

## Current State

Version `0.3.3` is experimental, benchmarked, and intended for small-object
algebra over rich scalar backends, not high-throughput dense BLAS.

Implemented:

- `Scalar<B>` constants and elementary functions
- `Complex<B>` arithmetic and integer powers
- `Vector3<B>` and `Vector4<B>` componentwise arithmetic, scalar operations,
  dot products, magnitude, normalization, and checked variants
- `Matrix3<B>` and `Matrix4<B>` componentwise arithmetic, multiplication,
  scalar division, matrix division, determinant, inverse, transpose,
  reciprocal, integer powers, and checked/abort-aware variants
- `ZeroStatus`, `ScalarFacts`, `ScalarSign`, and `ScalarMagnitudeBits`
- `AbortSignal` and `_with_abort` APIs for computations that may refine
  hyperreal values
- symbolic and alternate decimal formatting for hyperreal-backed values

Fallible operations return `BlasResult<T>`. Checked operations reject both
definite zero and unknown-zero divisors or pivots.

## Installation

```toml
[dependencies]
hyperlattice = "0.3.3"
```

From sibling checkouts:

```toml
[dependencies]
hyperlattice = { path = "../hyperlattice" }
```

The hyperreal-backed feature pulls in the matching `hyperreal` and `num`
dependencies. Applications only need direct `hyperreal` or `num` dependencies
when they use those crates outside `hyperlattice`.

Approx-only build:

```toml
[dependencies]
hyperlattice = {
    version = "0.3.3",
    default-features = false,
    features = ["approx-backend"],
}
```

Features:

| Feature | Default | Purpose |
| --- | --- | --- |
| `hyperreal-backend` | yes | Enables `HyperrealBackend`, `Real`/`Rational` re-exports, and exact/symbolic scalars. |
| `approx-backend` | yes | Enables `ApproxBackend` with `f64 +/- epsilon` scalar intervals. |

## Examples

### Scalars

```rust
use hyperlattice::{Scalar, ln, log10, pi, sqrt, tau};

fn s(value: i32) -> Scalar {
    value.into()
}

let nine: Scalar = 9.into();
assert_eq!(sqrt(nine).unwrap(), s(3));
assert_eq!(tau(), s(2) * pi());
assert_eq!(ln(hyperlattice::e()).unwrap(), s(1));
assert_eq!(log10(s(100)).unwrap(), s(2));
```

### Explicit Backends

```rust
use hyperlattice::{ApproxBackend, HyperrealBackend, Scalar, Vector3};

let exact: Scalar<HyperrealBackend> = Scalar::try_from(1.25).unwrap();
let approx: Scalar<ApproxBackend> = Scalar::<ApproxBackend>::approx(1.25, 0.01).unwrap();

let exact_vector = Vector3::<HyperrealBackend>::new([exact.clone(), exact.clone(), exact]);
let approx_vector = Vector3::<ApproxBackend>::new([approx.clone(), approx.clone(), approx]);

assert_eq!(exact_vector.0.len(), approx_vector.0.len());
```

### Vectors

```rust
use hyperlattice::{Rational, Scalar, Vector3, one};

fn s(value: i32) -> Scalar {
    value.into()
}

let v = Vector3::new([s(3), s(4), s(0)]);
let offset = v.clone() + s(10);

assert_eq!(v.dot(&v), s(25));
assert_eq!(offset, Vector3::new([s(13), s(14), s(10)]));
assert_eq!(v.normalize().unwrap().dot(&v.normalize().unwrap()), one());

let half = Rational::fraction(1, 2).unwrap().into();
let displayed = Vector3::new([half, s(2), s(3)]);
assert_eq!(format!("{displayed}"), "[1/2, 2, 3]");
assert_eq!(format!("{displayed:#}"), "[0.5, 2, 3]");
```

### Matrices

```rust
use hyperlattice::{Matrix3, Scalar};

fn s(value: i32) -> Scalar {
    value.into()
}

let matrix = Matrix3::new([
    [s(1), s(2), s(3)],
    [s(0), s(1), s(4)],
    [s(5), s(6), s(0)],
]);

assert_eq!(matrix.determinant(), s(1));
assert_eq!(matrix.clone() * matrix.clone().inverse().unwrap(), Matrix3::identity());
assert_eq!((matrix ^ 0).unwrap(), Matrix3::identity());
```

### Structural Facts

```rust
use hyperlattice::{ScalarSign, ZeroStatus, pi};

let facts = pi().structural_facts();
assert_eq!(facts.sign, Some(ScalarSign::Positive));
assert_eq!(facts.zero, ZeroStatus::NonZero);
assert!(!facts.exact_rational);

let approx = pi().to_f64_approx().unwrap();
assert!(approx > 3.0 && approx < 4.0);
```

The hyperreal backend forwards `Real::structural_facts`,
`Real::refine_sign_until`, and `Real::to_f64_approx`. The approx backend derives
facts from its stored interval.

Two backend details are intentionally visible at the type boundary:

- `Scalar<HyperrealBackend>` inherits `hyperreal::Real` structural equality.
  `PartialEq` is not a full symbolic-equivalence prover, so borrowed and owned
  operations that build semantically equivalent computable expressions can
  differ structurally for symbolic values. Exact rationals and dyadic imports
  are the right inputs for strict borrowed/owned equality tests; use facts or
  approximation when comparing symbolic construction histories.
- `Scalar::try_from(-0.0_f32)` and `Scalar::try_from(-0.0_f64)` import through
  exact rational zero on the hyperreal backend. The numeric value round-trips,
  but the IEEE signed-zero bit is intentionally not represented.

### Abort-Aware Checked Operations

```rust
use hyperlattice::{AbortSignal, Vector3};
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

let signal: AbortSignal = Arc::new(AtomicBool::new(false));
let vector = Vector3::new([3.into(), 4.into(), 0.into()]);

let unit = vector.normalize_checked_with_abort(&signal).unwrap();
assert_eq!(unit.dot(&unit), 1.into());
```

## Performance Notes

The crate is optimized for small fixed-size algebra over rich scalars:

- backend hooks for borrowed add, subtract, multiply, divide, inverse, and dot
  products reduce cloning of hyperreal expression graphs
- hyperreal-backed constants and identities delegate to `hyperreal`
  constructors
- vector, matrix, and complex operations use owned-left/borrowed-right forms in
  hot paths
- small scalar powers are specialized before exponentiation by squaring
- 3x3 and 4x4 borrowed matrix multiplication is unrolled
- matrix division and inversion use checked zero-status paths where requested
- scalar facts are forwarded by borrow so `liminal` can query them cheaply
- the approx backend mirrors the API with a lower-cost interval representation

Run the benchmark suite:

```sh
cargo bench --bench mathbench
```

The generated benchmark summary is in [`benchmarks.md`](benchmarks.md).

Run dispatch tracing separately:

```sh
cargo bench --bench mathbench --features hyperreal-dispatch-trace -- --write-dispatch-trace-md
```

The generated trace summary is in [`dispatch_trace.md`](dispatch_trace.md).

## Source Layout

- `src/scalar.rs`: scalar constants, functions, facts, and zero status
- `src/complex.rs`: `Complex`
- `src/vector.rs`: `Vector3` and `Vector4`
- `src/matrix.rs`: `Matrix3` and `Matrix4`
- `src/backend/hyperreal`: hyperreal-backed scalar implementation
- `src/backend/approx`: approximate scalar implementation

## Development

```sh
cargo fmt --check
cargo test --all-targets
cargo test --all-targets --all-features
cargo test --all-targets --no-default-features --features approx-backend
cargo clippy --all-targets -- -D warnings
cargo clippy --all-targets --all-features -- -D warnings
```

Use `--all-features` when checking code that uses explicit
`HyperrealBackend` and `ApproxBackend` type parameters in the same build.

## License

MIT.
