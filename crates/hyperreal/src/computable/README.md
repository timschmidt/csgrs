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

## Complex graph examples

The generator in [`../../examples/computable_graphs.rs`](../../examples/computable_graphs.rs)
builds intentionally large public `Computable` expressions, emits Mermaid graph
diagrams, and asks each root for an `approx(-80)` value:

```sh
cargo run --example computable_graphs
```

The generator keeps a tiny parallel graph builder next to the actual
`Computable` values:

```rust
let huge_pi = graph.binary("multiply", &pi, &huge, Computable::multiply);
let phase = graph.binary("add residual", &huge_pi, &seven_fifths, Computable::add);
let phase_plus_tiny = graph.binary("add tiny perturbation", &phase, &tiny, Computable::add);

let sin_phase = graph.unary("sin", &phase_plus_tiny, Computable::sin);
let cos_phase = graph.unary("cos", &phase, Computable::cos);
let atan = graph.unary("atan", &seven_tenths, Computable::atan);
let tan_atan = graph.unary("tan", &atan, Computable::tan);
let asin = graph.unary("asin", &three_fifths, Computable::asin);
let asin_sin = graph.unary("sin", &asin, Computable::sin);

let sin_sq = graph.unary("square", &sin_phase, Computable::square);
let cos_sq = graph.unary("square", &cos_phase, Computable::square);
let trig_norm = graph.binary(
    "add",
    &sin_sq,
    &cos_sq,
    Computable::add,
);
let inverse_norm = graph.unary("inverse", &trig_norm, Computable::inverse);
let numerator = graph.binary("add", &tan_atan, &asin_sin, Computable::add);
let product = graph.binary("multiply", &numerator, &inverse_norm, Computable::multiply);
let root = graph.unary(
    "sqrt",
    &product,
    Computable::sqrt,
);
```

### Argument-reduction tower

This expression stresses large argument reduction and inverse-trig composition:

```text
sqrt((tan(atan(7/10)) + sin(asin(3/5)))
     / (sin(10^30*pi + 7/5 + 2^-40)^2 + cos(10^30*pi + 7/5)^2))
```

```mermaid
flowchart TD
    n0["pi shared constant"]
    n1["10^30 exact integer"]
    n2["7/5 exact rational"]
    n3["3/5 exact rational"]
    n4["7/10 exact rational"]
    n5["1/2^40 exact rational"]
    n6["multiply"]
    n7["add residual"]
    n8["add tiny perturbation"]
    n9["sin"]
    n10["cos"]
    n11["atan"]
    n12["tan"]
    n13["asin"]
    n14["sin"]
    n15["square"]
    n16["square"]
    n17["add"]
    n18["inverse"]
    n19["add"]
    n20["multiply"]
    n21["sqrt"]
    n0 --> n6
    n1 --> n6
    n6 --> n7
    n2 --> n7
    n7 --> n8
    n5 --> n8
    n8 --> n9
    n7 --> n10
    n4 --> n11
    n11 --> n12
    n3 --> n13
    n13 --> n14
    n9 --> n15
    n10 --> n16
    n15 --> n17
    n16 --> n17
    n17 --> n18
    n12 --> n19
    n14 --> n19
    n19 --> n20
    n18 --> n20
    n20 --> n21
    n21:::root
    classDef root fill:#f7f3c6,stroke:#8a6d00,stroke-width:2px
```

Evaluation with `root.approx(-80)` returns the scaled integer
`1398739548397216159170853`, representing roughly:

```text
1.157010236445354561840032
```

The evaluator first uses constructor-time rewrites and structural facts where
available, then the trig kernels reduce the huge `10^30*pi + residual` argument
before requesting the precision needed by the final square-root node. Repeating
the same approximation reuses cached subresults.

### Cancellation and nested-inverse tower

This expression combines exact square rewrites, logs/exponentials, inverse
hyperbolic functions near difficult points, and a near-cancellation that is
inverted twice:

```text
sqrt(
    exp(ln(sqrt(12) + e))
  + exp(ln(45/14))
  + atanh(999999/1000000)
  + asinh(1/2)
  + acosh(sqrt(2)^2 + 1/2)
  + inverse(inverse(pi + 2^-50 - pi))
)
```

```mermaid
flowchart TD
    n0["pi shared constant"]
    n1["e shared constant"]
    n2["2 exact integer"]
    n3["12 exact integer"]
    n4["45/14 exact rational"]
    n5["999999/1000000 exact rational"]
    n6["1/2 exact rational"]
    n7["1/2^50 exact rational"]
    n8["sqrt"]
    n9["square"]
    n10["sqrt"]
    n11["add"]
    n12["ln"]
    n13["exp"]
    n14["ln"]
    n15["exp"]
    n16["atanh"]
    n17["asinh"]
    n18["add"]
    n19["acosh"]
    n20["add"]
    n21["add"]
    n22["add"]
    n23["negate"]
    n24["add"]
    n25["inverse"]
    n26["inverse"]
    n27["add"]
    n28["add"]
    n29["add"]
    n30["sqrt"]
    n2 --> n8
    n8 --> n9
    n3 --> n10
    n10 --> n11
    n1 --> n11
    n11 --> n12
    n12 --> n13
    n4 --> n14
    n14 --> n15
    n5 --> n16
    n6 --> n17
    n9 --> n18
    n6 --> n18
    n18 --> n19
    n16 --> n20
    n17 --> n20
    n20 --> n21
    n19 --> n21
    n0 --> n22
    n7 --> n22
    n0 --> n23
    n22 --> n24
    n23 --> n24
    n24 --> n25
    n25 --> n26
    n13 --> n27
    n15 --> n27
    n27 --> n28
    n21 --> n28
    n28 --> n29
    n26 --> n29
    n29 --> n30
    n30:::root
    classDef root fill:#f7f3c6,stroke:#8a6d00,stroke-width:2px
```

Evaluation with `root.approx(-80)` returns the scaled integer
`5227679412026104074933468`, representing roughly:

```text
4.324235058270604318885090
```

The `sqrt(2)^2` and double inverse shapes are candidates for structural
rewrites, while `atanh(999999/1000000)` and the `pi + 2^-50 - pi` branch force
precision planning to respect near-boundary and near-cancellation behavior. The
final `sqrt` asks each child only for enough precision to produce the requested
root approximation.

## Performance expectations

The fastest `Computable` path is the one never entered because `Rational` or
`Real` answered the question structurally. When a computable graph is required,
prefer shallow rewrites, cached constants, and bounded precision refinement over
eager high-precision evaluation.
