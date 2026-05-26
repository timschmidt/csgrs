<h1>
  hyperreal
  <img src="./doc/hyper.png" alt="Hyper, a clever mathematician" width="144" align="right">
</h1>

`hyperreal` provides exact rational arithmetic, symbolic real values, lazy computable
real approximation, and conservative structural facts for the Hyper ecosystem. It is
the scalar layer used by the surrounding exact-aware geometry, solver, physics, circuit,
routing, packing, voxel, and design-readiness crates.

The crate is not a full computer algebra system, and it does not try to canonicalize
every real expression eagerly. Its job is narrower: preserve enough exact, symbolic,
and refinement-ready structure that higher layers can make certified decisions or
report explicit uncertainty without quietly falling back to primitive floating point.

## Hyper Ecosystem

`hyperreal` is the scalar substrate. The rest of the Hyper stack should keep its own
object-level facts, but use `hyperreal` values and certificates when scalar exactness
matters.

Core layers:

- [hyperreal](https://github.com/timschmidt/hyperreal): exact rational, symbolic, and
  computable real arithmetic.
- [hyperlattice](https://github.com/timschmidt/hyperlattice): small exact vector,
  matrix, transform, and shared-scale algebra.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): exact predicate policy,
  escalation, and result provenance.

Geometry and solver layers:

- [hypercurve](https://github.com/timschmidt/hypercurve): planar curves, contours,
  regions, offsets, and boolean-boundary work.
- [hypertri](https://github.com/timschmidt/hypertri): exact polygon triangulation,
  Delaunay, and constrained Delaunay topology.
- [hypermesh](https://github.com/timschmidt/hypermesh): 3D mesh validation, topology,
  and exact-aware boolean preflight.
- [hyperbrep](https://github.com/timschmidt/hyperbrep): retained BREP topology,
  planar surfaces, trim evidence, tessellation manifests, and mesh handoff reports.
- [hypersdf](https://github.com/timschmidt/hypersdf): signed-distance and implicit-field
  carriers with exact-aware sampling, classification, solver, mesh, and voxel handoffs.
- [hypersolve](https://github.com/timschmidt/hypersolve): symbolic residuals,
  solver preparation, and candidate certification.
- [csgrs](https://github.com/timschmidt/csgrs/tree/hyperreal): Multi-modal CAD kernel, owns CSG

Domain and proposal layers:

- [hyperpath](https://github.com/timschmidt/hyperpath): routing, toolpath, tangent,
  clearance, and path-provenance carriers.
- [hyperdrc](https://github.com/timschmidt/hyperdrc): PCB design-readiness checks and
  manufacturing package evidence.
- [hyperphysics](https://github.com/timschmidt/hyperphysics): exact-aware materials,
  mass properties, contact, field, and simulation handoff reports.
- [hypercircuit](https://github.com/timschmidt/hypercircuit): circuit MNA carriers,
  residual replay, and coupled electrothermal reports.
- [hyperparts](https://github.com/timschmidt/hyperparts): source-attributed part,
  interface, process, and compatibility facts.
- [hyperpack](https://github.com/timschmidt/hyperpack): exact-aware packing models and
  feasibility replay.
- [hypervoxel](https://github.com/timschmidt/hypervoxel): exact-aware voxel grid
  frames, sparse-grid facts, and adapter manifests.
- [hyperevolution](https://github.com/timschmidt/hyperevolution): exact-aware search,
  fitness, archive, and replay-policy carriers.

## Typical Real-Number Problems

Most numerical programs choose between fixed-size floats, full rational arithmetic,
symbolic algebra, or local epsilon rules.

Floats are fast and hardware-friendly, but they round nearly every nontrivial value.
That approximation becomes part of the value: `0.1` is not exactly one tenth, cancellation
can manufacture near-zero signs, and equality answers a machine-representation question
rather than a mathematical one. In geometry, routing, and constraint code, one wrong sign
can move a point to the wrong side of a line, change a triangulation, or accept the wrong
solver branch.

Rationals give exact division, exact finite-float import, and exact determinant or
dot-product results, but numerator and denominator growth can dominate runtime if every
intermediate is fully canonicalized. Symbolic representations preserve more meaning, but
they are incomplete unless the program becomes a full algebra system.

`hyperreal` takes the middle path used by the Hyper stack: keep exact and symbolic
structure alive, expose cheap conservative facts, refine only when a caller asks for a
decision, and make lossy primitive-float export a named edge API rather than a hidden
fallback.

## Main Types

- [`Rational`](src/rational/README.md) is the exact arithmetic base. It stores
  arbitrary-precision numerator/denominator values and supports exact reduction,
  dyadic detection, square extraction, shared-denominator product sums, decimal/fraction
  parsing, and exact finite IEEE-754 import.
- [`Real`](src/real/README.md) is the public scalar for the stack. It combines an exact
  rational scale with a compact symbolic/computable class so common values such as
  exact ones, powers/products of `pi` and `e`, selected roots, logarithms, and trig forms
  can expose facts before approximation.
- [`Computable`](src/computable/README.md) is the lazy approximation layer. It stores an
  exact-real expression graph and computes scaled integer approximations only at a
  requested binary precision.
- `RealStructuralFacts`, `RealDetailedFacts`, `CertifiedRealSign`,
  `CertifiedRealOrdering`, and `CertifiedRealEquality` are conservative certificates
  for sign, zero status, magnitude, rational state, primitive-float status, domain
  status, and comparison outcomes.
- `RealExactSetFacts` records useful exact-set summaries such as denominator class,
  dyadic exponent class, and sign pattern for higher-level reducers.
- `Simple`, enabled by the `simple` feature, is a small Lisp-like expression parser and
  calculator surface for tests, examples, and configuration-facing formulas.
- `Problem` is the crate-local error type used by fallible scalar operations.

## Precision Model

`hyperreal` treats precision as an API contract, not an implementation accident.

- Finite `f32` and `f64` imports decode the exact IEEE-754 value. They do not reinterpret
  a binary float as a decimal measurement.
- Decimal and fraction strings parse through `Rational`, so text inputs such as
  `12.125` and `97/8` remain exact.
- `Real::structural_facts()` exposes conservative sign, zero, nonzero, magnitude,
  exact-rational, symbolic, primitive-float, and domain facts without requiring a full
  equality proof.
- `refine_sign_until`, `sign_until`, and related certification APIs request bounded
  refinement only when a caller needs more precision.
- `PartialEq` on `Real` is structural. It is not a complete algebraic equality proof.
  Use exact-rational extraction, structural certificates, or explicit refinement when
  semantic equality is the question.
- `Real::to_f32_lossy()` and `Real::to_f64_lossy()` are named edge exports for rendering,
  IO, diagnostics, and third-party interop. They are not predicate, ordering, equality,
  or topology decisions.

In Yap's exact geometric computation sense, exactness here means preserving enough
certified structure to decide later predicates exactly or report uncertainty. It does
not mean expanding every scalar expression to its largest possible canonical form.

## Numerical Explosion

`hyperreal` combats numerical explosion by preserving factored structure and asking for
bounded refinement only at decision boundaries. Dyadic schedules, shared-denominator
reducers, cached constants, computable approximation caches, exact-set facts, and
lossy export labels keep large rationals and symbolic graphs from becoming the default
representation for every operation.

## Performance Model

Exact arithmetic is only useful in a systems stack if it is measured and kept under
control. `hyperreal` uses several small performance strategies together:

- Preserve factored structure. Rationals, dyadics, constants, roots, logarithms, and
  selected trig forms stay classified so later reducers can avoid expanding large
  numerators, denominators, or expression graphs.
- Normalize at useful boundaries. Dyadic denominators reduce by shifts, product sums can
  share denominators, and matrix/vector callers can delay full rational canonicalization
  until accumulated terms have had a chance to combine.
- Remove trivial work early. Canonical zeros, ones, identity constructors, all-zero sums,
  exact endpoints, and known domain facts avoid constructing larger expressions.
- Prefer structural facts before scalar probing. Many hot branches need sign, zero,
  nonzero, magnitude, dyadic, or exact-rational facts rather than a fresh approximation.
- Approximate at requested precision. `Computable` nodes use argument reduction,
  prescaled kernels, cancellation-aware transforms, shared constants, and precision-aware
  caches so refinement grows with caller demand.
- Keep hot kernels predictable. Borrowed arithmetic, shared-denominator/product-sum
  reducers, cached constants, and capability-gated symbolic shortcuts are preferred over
  speculative approximation in dense loops.
- Measure regressions directly. Dispatch tracing and benchmark families track GCDs,
  rational temporaries, peak operand sizes, repeated approximation, exact reducer use,
  cache pressure, and stack-facing behavior for `hyperlattice` and `hyperlimit`.

## Current Status

Version `0.13.0` is active and benchmark-driven. Current implementation work includes:

- exact rational and dyadic fast paths;
- dedicated constructors and cached constants for common zeros, ones, `pi`, `tau`, `e`,
  common square roots, and common logarithms;
- symbolic classes for selected `pi`, `e`, `sqrt`, `ln`, `sin(pi*q)`, and `tan(pi*q)`
  forms;
- exact trig, inverse-trig, logarithm, exponential, and inverse-hyperbolic shortcuts
  where the input structure is recognizable;
- argument reduction and prescaled kernels for transcendental approximation;
- structural sign, zero, nonzero, magnitude, exact-rational, exact-set, and domain
  queries;
- bounded sign refinement and certified equality/ordering/sign reports;
- cached approximation and structural-fact propagation through computable nodes;
- borrowed arithmetic paths for `Rational` and `Real`;
- shared-denominator and signed-product-sum hooks used by matrix/vector callers;
- `serde` support for expression structure, excluding transient caches and abort
  signals;
- dispatch tracing and targeted benchmark suites for scalar, approximation, symbolic,
  adversarial, and stack-facing regressions;
- source-level READMEs for `Rational`, `Real`, and `Computable`, plus
  `structural_facts.txt` for planned and implemented fact propagation.

Known limits: some equality questions remain undecidable without more context, and some
symbolic expressions eventually require refinement. Higher crates should preserve
object-level facts rather than asking the scalar layer to infer geometry or topology.

## Installation

```toml
[dependencies]
hyperreal = "0.12.0"
```

With the `Simple` parser and calculator binary:

```toml
[dependencies]
hyperreal = { version = "0.12.0", features = ["simple"] }
```

Feature flags:

| Feature | Default | Purpose |
| --- | --- | --- |
| `simple` | no | Enables `Simple` and the package calculator binary. |
| `cached-f32-approx` | no | Caches selected `f32` approximation paths. |
| `cached-f64-approx` | no | Caches selected `f64` approximation paths. |
| `dispatch-trace` | no | Records scalar dispatch and rational-growth counters. |

## Examples

### Exact Rationals

`Rational` is useful for measurements, fixtures, coefficients, and imported finite
values that must not become binary floating-point noise before the rest of the stack
sees them.

```rust
use hyperreal::Rational;
use std::convert::TryFrom;

let a = Rational::fraction(7, 8).unwrap();
let b = Rational::fraction(9, 10).unwrap();

assert_eq!(a + b, Rational::fraction(79, 40).unwrap());

let half = Rational::try_from(0.5_f64).unwrap();
assert_eq!(half, Rational::fraction(1, 2).unwrap());

let decimal: Rational = "12.125".parse().unwrap();
let fraction: Rational = "97/8".parse().unwrap();
assert_eq!(decimal, fraction);
```

### Symbolic Reals And Facts

`Real` keeps a rational scale plus symbolic or computable structure. Recognizable forms
can expose facts before a caller asks for a high-precision approximation.

```rust
use hyperreal::{Rational, Real, RealSign, ZeroKnowledge};

let x = Real::new(Rational::new(2)).sqrt().unwrap();
let y = Real::new(Rational::new(3)).sqrt().unwrap();
let z = x * y;

let approx = z.to_f64_lossy().unwrap();
assert!(approx > 2.44 && approx < 2.45);

let half = Real::new(Rational::fraction(1, 2).unwrap());
let cosine = (half * Real::pi()).cos().unwrap();
let facts = cosine.structural_facts();

assert_eq!(facts.zero, ZeroKnowledge::Zero);
assert_eq!(cosine.refine_sign_until(-32), Some(RealSign::Zero));
```

### Computable Approximation

`Computable` stores an expression graph and only computes a scaled integer
approximation when a precision is requested.

```rust
use hyperreal::{Computable, Rational, RealSign};

let x = Computable::rational(Rational::fraction(7, 5).unwrap()).sin();
let scaled = x.approx(-40);
assert_ne!(scaled, 0.into());

let near_pi = Computable::pi().add(Computable::rational(
    Rational::fraction(-22, 7).unwrap(),
));
assert_eq!(near_pi.sign_until(-8), Some(RealSign::Positive));
```

### Stack-Facing Decisions

Higher crates should ask for cheap facts first, then request bounded refinement only
when a decision needs a sign.

```rust
use hyperreal::{Rational, Real, RealSign};

fn classify_positive(value: &Real) -> Option<bool> {
    if let Some(sign) = value.structural_facts().sign {
        return Some(sign == RealSign::Positive);
    }

    value
        .refine_sign_until(-80)
        .map(|sign| sign == RealSign::Positive)
}

let offset = Real::pi() - Real::new(Rational::fraction(22, 7).unwrap());
assert_eq!(classify_positive(&offset), Some(true));
```

### Simple Expressions

Requires the `simple` feature.

```rust
use hyperreal::{Rational, Simple};

let expr: Simple = "(sqrt (/ 49 64))".parse().unwrap();
let value = expr.evaluate(&Default::default()).unwrap();

assert_eq!(value.exact_rational(), Some(Rational::fraction(7, 8).unwrap()));
```

`Simple` supports arithmetic, roots, powers, logs, exponentials, trig, inverse trig,
inverse hyperbolic functions, integers, decimals, fractions, `pi`, and `e`.

## Conversions

Supported conversions include:

- integer types to `Rational` and `Real`;
- finite `f32`/`f64` to `Rational` and `Real` by exact IEEE-754 decoding;
- `Real` to `f32`/`f64` by approximation;
- `Real::to_f32_lossy()` and `Real::to_f64_lossy()` for borrowed primitive exports;

Float import rejects `NaN` and infinities. Borrowed lossy export returns `None` when no
finite primitive-float approximation can be produced. Scientific notation is not a
supported exact text format. `-0.0` imports as exact rational zero, so IEEE signed-zero
information is not preserved.

## Documentation And Benchmarks

Useful local checks:

```sh
cargo fmt --check
cargo test
RUSTDOCFLAGS=-Dwarnings cargo doc --no-deps
cargo bench --bench numerical_micro
cargo bench --bench borrowed_ops
cargo bench --bench float_convert
cargo bench --bench scalar_micro
cargo bench --bench library_perf --features simple
cargo bench --bench adversarial_transcendentals
```

Run dispatch tracing separately:

```sh
cargo bench --bench dispatch_trace --features dispatch-trace
```

The generated benchmark summary is in [`benchmarks.md`](./benchmarks.md). Profiling
anchors and regression goals for `Rational`, `Real`, and `Computable` are in
[`PERFORMANCE.md`](./PERFORMANCE.md). Dispatch summaries are written to
[`dispatch_trace.md`](./dispatch_trace.md) when tracing is enabled.

When adding a shortcut, add a focused correctness test and a benchmark row for the
smallest affected surface. Keep the shortcut only if it improves the target without
regressing broader stack-facing paths.

## Provenance and Acknowledgements

`hyperreal` descends from the [`realistic`](https://github.com/tialaramex/realistic/)
project and continues that project's interest in practical computable real arithmetic.
Special thanks to [siefkenj](https://github.com/siefkenj), whose contributions improved
realistic and hyperreal.

## References

These are the papers and books which contribute ideas or methods to this crate.
They are listed here in MLA style for easy citation.

- Bareiss, Erwin H. "[Sylvester's Identity and Multistep Integer-Preserving
  Gaussian Elimination](https://www.ams.org/mcom/1968-22-103/S0025-5718-1968-0226829-0/)."
  *Mathematics of Computation*, vol. 22, no. 103, 1968, pp. 565-578.
  American Mathematical Society, https://doi.org/10.1090/S0025-5718-1968-0226829-0.
- Boehm, Hans-Juergen, Robert Cartwright, Mark Riggle, and Michael J.
  O'Donnell. "[Exact Real Arithmetic: A Case Study in Higher Order
  Programming](https://doi.org/10.1145/319838.319860)." *Proceedings of the
  1986 ACM Conference on LISP and Functional Programming*, ACM, 1986,
  pp. 162-173.
- Boehm, Hans-J. "[Towards an API for the Real
  Numbers](https://doi.org/10.1145/3385412.3386037)." *Proceedings of the
  41st ACM SIGPLAN International Conference on Programming Language Design and
  Implementation*, ACM, 2020, pp. 562-576.
- Brent, Richard P. "[Fast Multiple-Precision Evaluation of Elementary
  Functions](https://doi.org/10.1145/321941.321944)." *Journal of the ACM*,
  vol. 23, no. 2, 1976, pp. 242-251.
- Brent, Richard P., and Paul Zimmermann.
  "[Modern Computer Arithmetic](https://doi.org/10.1017/CBO9780511921698)."
  Cambridge University Press, 2010.
- Middeke, Johannes, David J. Jeffrey, and Christoph Koutschan.
  "[Common Factors in Fraction-Free Matrix
  Decompositions](https://doi.org/10.1007/s11786-020-00495-9)."
  *Mathematics in Computer Science*, vol. 15, 2021, pp. 589-608.
- Odrzywołek, Andrzej. "[All Elementary Functions from a Single Binary
  Operator](https://arxiv.org/abs/2603.21852)." *arXiv*, 2026,
  arXiv:2603.21852. Related implementation: Schmidt, Tim.
  "[emlmath](https://github.com/timschmidt/emlmath)." GitHub, 2026. Relevant
  to `Computable` graph evaluation and expression-tree lowering.
- Payne, Mary H., and Robert N. Hanek.
  "[Radian Reduction for Trigonometric
  Functions](https://doi.org/10.1145/1057600.1057602)." *ACM SIGNUM
  Newsletter*, vol. 18, no. 1, 1983, pp. 19-24.
- Shewchuk, Jonathan Richard. "[Adaptive Precision Floating-Point Arithmetic
  and Fast Robust Geometric
  Predicates](https://doi.org/10.1007/PL00009321)." *Discrete &
  Computational Geometry*, vol. 18, no. 3, 1997, pp. 305-363.
- Smith, Luke, and Joan Powell. "[An Alternative Method to Gauss-Jordan
  Elimination: Minimizing Fraction
  Arithmetic](https://doi.org/10.63301/tme.v20i2.1957)." *The Mathematics
  Educator*, vol. 20, no. 2, 2011, pp. 44-50.
- Yap, Chee-Keng. "[Towards Exact Geometric
  Computation](https://doi.org/10.1016/0925-7721(95)00040-2)." *Computational
  Geometry*, vol. 7, nos. 1-2, 1997, pp. 3-23.

## License

(C) https://github.com/timschmidt Apache-2.0 / MIT

(C) https://github.com/tialaramex/realistic/ Apache-2.0

(C) https://github.com/siefkenj Apache-2.0
