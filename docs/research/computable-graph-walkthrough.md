# Computable graph walkthrough

These examples illustrate native scalar construction and query order. Their
Mermaid diagrams record authored operations or fixed explanatory stages, not
the rewritten internal DAG or observed dispatch. Printed comparisons and
repeated samples are observations; neither program contains an independent
oracle or a performance benchmark.

The output below was reproduced on revision
`fad0c11f1` during the documentation audit, using the all-feature build on the
current host. The documentation-only changes preserve both examples' output.
Exact integer samples are not canonical across implementations or cache histories;
completed approximation error, rather than equality of arbitrary sample integers,
is the numerical contract. See the [audit ledger](rustdoc-audit.md) for the
focused validation and independent mathematical enclosure checks.

The generator in [`../../crates/hyperreal/examples/computable_graphs.rs`](../../crates/hyperreal/examples/computable_graphs.rs)
builds intentionally large public `Computable` expressions, emits Mermaid graph
diagrams, and asks each root for an `approx(-80)` value:

```sh
cargo run -p hyperreal --example computable_graphs
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
`1378387510292223641544648`, representing roughly:

```text
1.140175425098964290542469
```

The two trig arguments differ by `2^-40`, so their squared sum is not the
identity one. Because `10^30` is even, periodicity removes its pi multiple,
leaving `sin(7/5+2^-40)^2+cos(7/5)^2`. The numerator is mathematically `13/10`.
Native constructors may simplify these expressions before evaluation. A repeated
request can reuse retained samples; the example does not instrument cache hits.

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
`5227679412026104074933469`, representing roughly:

```text
4.324235058270604318885090
```

Every source here is in its real domain. The radicand simplifies mathematically
to `sqrt(12)+e+45/14+atanh(999999/1000000)+asinh(1/2)+acosh(5/2)+2^-50`.
In particular, shared pi cancellation can preserve the tiny residual exactly;
the authored inverse nodes do not force numerical evaluation of both inverses.
Kernel guard precision and shared-cache history remain separate from the outer
`-80` request. This example makes no total-work or minimal-precision claim.

### Refinement walk

The generator in
[`../../crates/hyperreal/examples/computable_refinement_steps.rs`](../../crates/hyperreal/examples/computable_refinement_steps.rs)
walks the argument-reduction tower through the public inspection and evaluation
steps:

```sh
cargo run -p hyperreal --example computable_refinement_steps
```

It builds the same shape as the first graph:

```text
sqrt((tan(atan(7/10)) + sin(asin(3/5)))
     / (sin(10^30*pi + 7/5 + 2^-40)^2 + cos(10^30*pi + 7/5)^2))
```

The walkthrough reports exact-leaf facts, sign queries, samples, display,
tolerance comparisons and repeated requests. Queries run in the printed order,
so later observations can consume earlier evidence. Its helpers sometimes build
new root graphs, but process-wide constant caches remain shared. The routine
named `print_facts_only` also calls `sign_until`, which may refine numerically.

The walkthrough prints fixed staged diagrams before the numeric output. Red
and green illustrate mathematical reductions, blue illustrates precision requests,
and purple illustrates a repeated request. These colors and the cache-hit label
are explanatory text, not evidence of a particular runtime path.

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

Symbolic stage 2 illustrates inverse-function identities. The program checks
them with `compare_absolute(..., -64)`, which can decide through exact structure
or numerical refinement. Its `Equal` result is not generally an exact equality
certificate, and the printed diagram is not a trace of the chosen proof:

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

Numeric stage 3 illustrates periodic reduction of the huge trigonometric
inputs. The mathematical equivalence is exact; the program's tolerance
comparisons do not count native reduction steps or internal precision:

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

Numeric stage 4 illustrates precision requests and possible cache reuse.
`approx(p)` returns an integer representing the value in units of `2^p`.
Internal guard precision can be finer, and decimal formatting can warm caches
between printed queries. No minimal-refinement or cache-hit count is asserted:

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

Observed numeric output for the query order above:

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
  facts: sign=positive, zero=nonzero, exact_rational=false, magnitude=unknown
  sign_until(-24): Some(Positive)
  approx(-12): 4096
  approx(-24): 16777216

numerator:
  facts: sign=unknown, zero=unknown, exact_rational=false, magnitude=unknown
  sign_until(-24): Some(Positive)
  approx(-12): 5325
  approx(-24): 21810381

trig_norm refinement:
  approx( -8) = 256
  approx(-16) = 65536
  approx(-32) = 4294967296
  approx(-64) = 18446744073715171785
  decimal = 1.000000000000304669947723

numerator refinement:
  approx( -8) = 333
  approx(-16) = 85197
  approx(-32) = 5583457485
  approx(-64) = 23980767295822417101
  decimal = 1.300000000000000000000000

product before sqrt:
  facts: sign=positive, zero=nonzero, exact_rational=false, magnitude=unknown
  sign_until(-24): Some(Positive)

root:
  facts: sign=positive, zero=nonzero, exact_rational=false, magnitude=unknown
  sign_until(-24): Some(Positive)

final root:
  approx(-80) = 1378387510292223641544648
  decimal = 1.140175425098964290542469

large-argument reduction checks:
  sin(10^30*pi + 7/5 + 2^-40) vs sin(7/5 + 2^-40): equal within tolerance
  cos(10^30*pi + 7/5) vs cos(7/5): equal within tolerance

inverse-function reduction checks:
  tan(atan(7/10)) vs 7/10: equal within tolerance
  sin(asin(3/5)) vs 3/5: equal within tolerance

cache demonstration:
  first approx(-80) = 1378387510292223641544648
  second approx(-80) = 1378387510292223641544648
```

The coarse samples of `2^-40` are zero at the requested scales while structural
facts retain its positive nonzero value. This is permitted approximation, not
loss of the exact source. Subsequent requests and decimal formatting refine or
reuse evidence according to their own needs; an outer precision request is not
a bound on all nested work.
