# Computable

`Computable` is the lazy exact-real expression graph. It is used when `Real`
needs a numeric certificate or an approximation path that cannot remain purely
rational/symbolic.

## Representation model

The internal node graph represents constants, exact rational leaves, arithmetic
operations, elementary functions, scale/shift nodes, and specialized kernels.
Nodes carry caches for approximations and conservative facts so repeated
queries do not rebuild work.

`Computable` follows the exact-real arithmetic model: callers request an
approximation at a binary precision, and the graph refines only as much as is
needed for that request.

## Module map

- `mod.rs`: public export and private shared node helpers.
- `node.rs`: compact expression enum, caches, structural facts, graph rewrites,
  constructors, and most high-level methods.
- `approximation.rs`: precision refinement kernels for elementary functions and
  argument reduction.
- `constants.rs`: shared constants and cached named values.
- `format.rs`: formatting support.
- `symbolic.rs`: symbolic helper routines and split points.

## API expectations

- `approx(precision)` returns a scaled integer approximation for the requested
  binary precision.
- repeated approximation requests may use caches, but lower-precision cache
  hits must not corrupt later higher-precision requests.
- sign and magnitude helpers are conservative unless exact structure proves the
  answer.
- constructors should simplify obvious identities before allocating generic
  graph nodes.
- abort-aware paths must check signals at bounded points without changing
  ordinary non-abort semantics.

## Numerical expectations

Approximation kernels should:

- reduce arguments before entering expensive series or transcendental kernels
- use exact/symbolic endpoints where possible
- avoid cancellation-prone forms when a stable transform is available
- reuse shared constants such as `pi`, `tau`, `e`, `sqrt(2)`, `sqrt(3)`, and
  common logarithms
- keep approximation precision explicit rather than silently falling back to
  primitive floating point

## Performance expectations

The fastest `Computable` path is the one never entered because `Rational` or
`Real` answered the question structurally. When a computable graph is required,
prefer shallow rewrites, cached constants, and bounded precision refinement over
eager high-precision evaluation.

