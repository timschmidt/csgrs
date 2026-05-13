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

### Refinement walk

The generator in
[`../../examples/computable_refinement_steps.rs`](../../examples/computable_refinement_steps.rs)
walks the argument-reduction tower through the public inspection and evaluation
steps:

```sh
cargo run --example computable_refinement_steps
```

It builds the same shape as the first graph:

```text
sqrt((tan(atan(7/10)) + sin(asin(3/5)))
     / (sin(10^30*pi + 7/5 + 2^-40)^2 + cos(10^30*pi + 7/5)^2))
```

The example demonstrates most of the Computable lifecycle:

- exact rational leaves report sign, zero status, exact-rational status, and
  exact magnitude without approximation
- the huge phase `10^30*pi + 7/5` is structurally nonzero and positive, with a
  known high most-significant bit
- trig evaluation reduces the huge `pi` multiple to the residual argument
- inverse-function compositions refine back to their rational inputs
- intermediate sums refine at progressively finer binary precisions
- the final root is evaluated at the requested precision and a repeat request
  reuses the cache

The walkthrough prints staged graphs before the numeric output. Nodes in red are
the expression shape being replaced or reduced; green nodes are the retained
reduced form; blue nodes are refinement requests; purple is a repeated cached
request.

Symbolic stage 1 starts with the full expression graph:

```mermaid
flowchart TD
    phase["10^30*pi + 7/5"]
    tiny["2^-40"]
    phaseTiny["add tiny"]
    sinHuge["sin"]
    phaseCos["10^30*pi + 7/5"]
    cosHuge["cos"]
    sinSq["square"]
    cosSq["square"]
    norm["add: trig_norm"]
    sevenTenths["7/10"]
    atan["atan"]
    tan["tan"]
    threeFifths["3/5"]
    asin["asin"]
    sinAsin["sin"]
    numerator["add: numerator"]
    invNorm["inverse"]
    product["multiply"]
    root["sqrt root"]
    phase --> phaseTiny
    tiny --> phaseTiny
    phaseTiny --> sinHuge --> sinSq --> norm
    phaseCos --> cosHuge --> cosSq --> norm
    sevenTenths --> atan --> tan --> numerator
    threeFifths --> asin --> sinAsin --> numerator
    norm --> invNorm --> product
    numerator --> product --> root
    root:::root
    classDef root fill:#f7f3c6,stroke:#8a6d00,stroke-width:2px
```

Symbolic stage 2 shows inverse-function reduction checks. These are not
floating-point identities; `compare_absolute(..., -64)` asks the computable
values to refine enough to prove equality at that tolerance:

```mermaid
flowchart TD
    sevenTenths["7/10"]
    atanTan["tan(atan(7/10))"]
    sevenReduced["7/10 retained"]
    threeFifths["3/5"]
    asinSin["sin(asin(3/5))"]
    threeReduced["3/5 retained"]
    numerator["add: numerator"]
    sevenTenths --> atanTan --> sevenReduced --> numerator
    threeFifths --> asinSin --> threeReduced --> numerator
    atanTan:::changed
    asinSin:::changed
    sevenReduced:::result
    threeReduced:::result
    classDef changed fill:#ffe3dc,stroke:#b5472f,stroke-width:2px
    classDef result fill:#dcfce7,stroke:#15803d,stroke-width:2px
```

Numeric stage 3 shows argument reduction for the huge trigonometric inputs. The
`10^30*pi` term is reduced away modulo the trig period before the approximation
kernels spend precision on the residual:

```mermaid
flowchart TD
    hugeSin["sin(10^30*pi + 7/5 + 2^-40)"]
    reducedSin["sin(7/5 + 2^-40)"]
    hugeCos["cos(10^30*pi + 7/5)"]
    reducedCos["cos(7/5)"]
    sinSq["square"]
    cosSq["square"]
    norm["add: trig_norm"]
    hugeSin --> reducedSin --> sinSq --> norm
    hugeCos --> reducedCos --> cosSq --> norm
    hugeSin:::changed
    hugeCos:::changed
    reducedSin:::result
    reducedCos:::result
    classDef changed fill:#ffe3dc,stroke:#b5472f,stroke-width:2px
    classDef result fill:#dcfce7,stroke:#15803d,stroke-width:2px
```

Numeric stage 4 shows precision refinement and cache reuse. `approx(p)` returns
an integer scaled by `2^-p`, and each request may refine only as far as needed
for that precision:

```mermaid
flowchart LR
    root["sqrt root"]
    p8["approx(-8)"]
    p16["approx(-16)"]
    p32["approx(-32)"]
    p64["approx(-64)"]
    p80["approx(-80)"]
    cached["second approx(-80): cache hit"]
    root --> p8 --> p16 --> p32 --> p64 --> p80 --> cached
    p8:::changed
    p16:::changed
    p32:::changed
    p64:::changed
    p80:::result
    cached:::cache
    classDef changed fill:#e0f2fe,stroke:#0369a1,stroke-width:2px
    classDef result fill:#dcfce7,stroke:#15803d,stroke-width:2px
    classDef cache fill:#ede9fe,stroke:#6d28d9,stroke-width:2px
```

Representative output:

```text
residual 7/5:
  facts: sign=positive, zero=nonzero, exact_rational=true, magnitude=msd 0 (exact=true)
  sign_until(-24): Some(Positive)
  approx(-12): 5734
  approx(-24): 23488102

tiny 2^-40:
  facts: sign=positive, zero=nonzero, exact_rational=true, magnitude=msd -40 (exact=true)
  sign_until(-24): Some(Positive)
  approx(-12): 0
  approx(-24): 0

phase = 10^30*pi + 7/5:
  facts: sign=positive, zero=nonzero, exact_rational=false, magnitude=msd 101 (exact=true)
  zero_status: NonZero
  sign_until(0): Some(Positive)

trig_norm:
  facts: sign=unknown, zero=unknown, exact_rational=false, magnitude=unknown
  sign_until(-24): Some(Positive)
  approx(-12): 3978
  approx(-24): 16292542

numerator:
  facts: sign=unknown, zero=unknown, exact_rational=false, magnitude=unknown
  sign_until(-24): Some(Positive)
  approx(-12): 5325
  approx(-24): 21810381

trig_norm refinement:
  approx( -8) = 249
  approx(-16) = 63643
  approx(-32) = 4170890717
  approx(-64) = 17913839226283551985
  decimal = 0.971111170334633746241117

numerator refinement:
  approx( -8) = 333
  approx(-16) = 85197
  approx(-32) = 5583457485
  approx(-64) = 23980767295822417101
  decimal = 1.300000000000000000000000

product before sqrt:
  facts: sign=unknown, zero=unknown, exact_rational=false, magnitude=unknown
  sign_until(-24): None

root:
  facts: sign=unknown, zero=unknown, exact_rational=false, magnitude=unknown
  sign_until(-24): None

final root:
  approx(-80) = 1398739548397216159170853
  decimal = 1.157010236445354561840032

large-argument reduction checks:
  sin(10^30*pi + 7/5 + 2^-40) vs sin(7/5 + 2^-40): equal within tolerance
  cos(10^30*pi + 7/5) vs cos(7/5): equal within tolerance

inverse-function reduction checks:
  tan(atan(7/10)) vs 7/10: equal within tolerance
  sin(asin(3/5)) vs 3/5: equal within tolerance

cache demonstration:
  first approx(-80) = 1398739548397216159170853
  second approx(-80) = 1398739548397216159170853
```

The coarse `tiny` approximations round to zero because `approx(p)` returns the
integer scaled by `2^-p`; the exact structural facts still preserve its positive
nonzero status. The later refinement rows show the same expression family
requesting more binary digits only where the caller asks for them.

## Performance expectations

The fastest `Computable` path is the one never entered because `Rational` or
`Real` answered the question structurally. When a computable graph is required,
prefer shallow rewrites, cached constants, and bounded precision refinement over
eager high-precision evaluation.
