# Real

`Real` is the public symbolic scalar. It combines an exact rational scale, a
compact symbolic class, an optional lazy computable certificate, and an optional
abort signal.

## Stored parts

```text
+-----------------------------------------------------------------------+
| Real                                                                  |
|                                                                       |
|  rational: Rational                                                   |
|  + exact signed scale                                                  |
|  + zero is represented here                                            |
|  + multiplies every nonzero symbolic/computable class                  |
|                                                                       |
|  class: Class                                                         |
|  + One                         exact rational only                     |
|  + Pi, PiPow, PiInv            pi-family certificates                  |
|  + Exp, PiExp, ConstProduct    e/pi product certificates               |
|  + Sqrt, PiSqrt, ...Sqrt       factored square-root certificates       |
|  + Ln, LnAffine, LnProduct     logarithm certificates                  |
|  + Log10                       base-10 logarithm certificate           |
|  + SinPi, TanPi                rational trig certificates              |
|  + Irrational                  opaque computable value                 |
|                                                                       |
|  computable: Option<Computable>                                       |
|  + lazy approximation graph                                            |
|  + shared constants and cached approximations live inside Computable   |
|  + absent only when the rational/class certificate is sufficient       |
|                                                                       |
|  signal: Option<Signal>                                               |
|  + optional abort hook for bounded/refinement callers                  |
+-----------------------------------------------------------------------+
```

The `Class` value is a certificate, not the entire number. The mathematical
value is `rational * class_value`, with `Computable` available when numeric
approximation is required.

## Module map

- `mod.rs`: public export and semantic module split.
- `arithmetic.rs`: representation, symbolic classes, constructors,
  simplification, arithmetic, elementary functions, display, and most tests.
- `constructors.rs`: public constructor grouping.
- `facts.rs`: structural fact API grouping.
- `approximation.rs`: approximation-facing API grouping.
- `linear_combination.rs`: exact linear combination and product-sum helpers.
- `convert.rs`: primitive and rational conversions.
- `tests.rs`: semantic and regression tests.

Most implementation still lives in `arithmetic.rs` because private fields and
hot simplification paths are tightly coupled. Avoid moving code just for file
shape unless benchmarks show no cost.

## API expectations

- `Real::new(Rational)` creates exact rational values.
- named constants such as `pi`, `e`, and `tau` use cached/shared construction.
- arithmetic preserves recognizable symbolic structure where benchmarks justify
  it.
- fallible methods return `Problem` for known domain errors.
- structural queries return conservative facts and should not force expensive
  approximation when representation facts are enough.
- borrowed arithmetic should avoid unnecessary expression cloning.
- conversion to primitive floats approximates; conversion from finite primitive
  floats is exact.

## Numerical expectations

`Real` should prefer this order:

1. answer from exact rational structure
2. answer from symbolic class facts
3. simplify symbolically into a smaller exact/certified form
4. construct or reuse a `Computable`
5. approximate only at requested precision

This is why many methods contain special cases for exact rationals, dyadics,
pi/e products, square roots, logarithms, and rational trig endpoints.

## Error expectations

Errors are semantic domain failures, not "could not prove cheaply" failures.
For example, a known-negative square root fails. A value whose sign is not
cheaply known may move to a computable path or bounded refinement path instead
of immediately failing, depending on the method.

## Performance expectations

Performance-sensitive code should document why a non-obvious representation is
kept. Typical reasons:

- preserving exact rational access for matrix/vector kernels
- keeping `pi`, `e`, and `sqrt` factors separate so later operations cancel
  them before approximation
- using cached computable constants rather than rebuilding kernels
- avoiding generic computable graphs for exact endpoints
- keeping direct expression shapes when Criterion shows they inline better

