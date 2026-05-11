# hyperreal source map

This directory contains the implementation behind the public scalar API. The
main design rule is to preserve exact or symbolic structure for as long as it is
cheap and useful, approximate only when a caller asks for numeric precision, and
cache approximations/facts that are expensive to rediscover.

## Project layout

- `lib.rs`: public exports and crate-level API surface.
- `problem.rs`: domain and arithmetic errors shared by public types.
- `structural.rs`: conservative public sign, zero, magnitude, and exactness
  facts.
- `trace.rs` and `dispatch_trace`: optional instrumentation used by targeted
  dispatch tracing.
- `serde.rs`: serialization for durable expression structure. Transient caches
  and abort signals are intentionally not serialized.
- `simple.rs`: optional expression parser and evaluator behind the `simple`
  feature.
- `rational/`: exact arbitrary-precision rational arithmetic. See
  [`rational/README.md`](./rational/README.md).
- `real/`: public symbolic scalar type layered over rational and computable
  values. See [`real/README.md`](./real/README.md).
- `computable/`: lazy exact-real expression graph and approximation kernels. See
  [`computable/README.md`](./computable/README.md).

## Numerical expectations

- Exact rationals stay exact until an API explicitly asks for approximation or
  conversion to a primitive float.
- Symbolic constants and recognizable forms stay symbolic when doing so avoids
  precision loss or repeated approximation.
- Approximation is requested by binary precision. More negative precision means
  more bits after the binary point.
- Approximation may cache intermediate values. Cache hits must not change
  semantics, structural facts, or later higher-precision approximations.
- Structural facts are conservative. Unknown means "not cheaply proven", not
  false.
- `Real` equality is structural, not a complete computer-algebra theorem prover.

## Error model

Public fallible operations return `Problem` rather than panicking for ordinary
numeric domain failures:

- divide by zero
- square root of known-negative values
- logarithm of non-positive values
- inverse trig / inverse hyperbolic domain failures
- invalid primitive float import such as `NaN` or infinity

Internal `expect` calls are reserved for representation invariants that should
already have been proven by construction.

## Performance and tracing expectations

Performance work should follow the same priorities as the implementation:

- defer approximation as long as exact or symbolic structure can answer the
  question
- reduce symbolic or rational structure before constructing generic computable
  graphs
- reuse costly approximations and named constants
- avoid unnecessary allocation and cloning on borrowed arithmetic paths
- use inexpensive structural facts before falling back to approximation
- specialize only paths that have stable targeted benchmark evidence

Performance-driven choices should have adjacent comments, especially when the
code shape is less direct than the mathematical formula. Existing comments cite
papers where an algorithmic principle matters, for example fraction-free
elimination and exact-real arithmetic.

Use dispatch tracing to explain a path and Criterion benchmarks to decide
whether a change is worth keeping. Trace reductions without Criterion runtime
improvements are not enough.

## Type-level documentation

- [`Rational`](./rational/README.md): exact storage and arithmetic layer.
- [`Real`](./real/README.md): public symbolic scalar and structural fact layer.
- [`Computable`](./computable/README.md): lazy expression graph and numeric
  approximation layer.

