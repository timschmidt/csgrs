# Rational

`Rational` is the exact fraction carrier beneath `hyperreal::Real` and the Hyper
geometry kernels. It represents arbitrary-width rational numbers using a sign,
an unsigned numerator and a nonzero unsigned denominator. It does not represent
irrational numbers; `Real` supplies the broader scalar representations.

## Architecture

```text
integer / finite IEEE input ── convert.rs ──┐
                                          ▼
text input ── arithmetic/format_parse.rs ── Rational ── Real / geometry consumers
                                          ▲
                   arithmetic/*.rs ────────┘
                   exact operations, queries and retained evidence
```

| Source | Responsibility |
| --- | --- |
| [mod.rs](mod.rs) | Connect the private modules and re-export arithmetic items to the crate facade. |
| [arithmetic.rs](arithmetic.rs) | Include implementation fragments into one module with shared representation and helper authority. |
| [arithmetic/representation.rs](arithmetic/representation.rs) | Shared payload, retained facts, cache layouts and optional Serde interchange. |
| [arithmetic/construction.rs](arithmetic/construction.rs) | Constructors, exact reduction, canonicalization and shared-value reuse. |
| [arithmetic/ops.rs](arithmetic/ops.rs), [aggregate_products.rs](arithmetic/aggregate_products.rs) | Scalar operations, balanced iterator products and fused exact reductions. |
| [arithmetic/comparison.rs](arithmetic/comparison.rs), [queries_conversion.rs](arithmetic/queries_conversion.rs) | Exact equality/order, hash, canonical observations and explicit float views. |
| [convert.rs](convert.rs) | Primitive integer ingress/egress and exact finite-float imports. |
| [arithmetic/format_parse.rs](arithmetic/format_parse.rs) | Text parsing and display; [parse.rs](parse.rs) is only a private documentation anchor. |

Clones share the payload and its retained evidence. Zero has sign `NoSign`,
numerator zero and denominator one. Some internal constructions retain a marked
unreduced ratio with its exact value intact. Canonical observations such as
numerator/denominator access, hashing and formatting obtain a shared reduced
value; exact comparisons can also use the stored ratio directly. A cache entry
or allocation identity is not the definition of rational equality.

## Build and use

From the repository root:

```sh
cargo build -p hyperreal --lib
cargo doc -p hyperreal --lib --no-deps --document-private-items
```

Open `target/doc/hyperreal/index.html` with the default Cargo target directory.
The private docs include `rational` and its implementation modules. Add
`--all-features` to include optional APIs such as Serde. Test functions and
function-local declarations are not automatically exposed by ordinary library
Rustdoc; their coverage is tracked in the [audit ledger](../../../../docs/research/rustdoc-audit.md).

```rust
use hyperreal::{Problem, Rational};

/// Construct exact tenths and use a checked reciprocal before multiplication.
fn main() -> Result<(), Problem> {
    let tenth = Rational::fraction(1, 10)?;
    let sum = &tenth + "0.2".parse::<Rational>()?;
    assert_eq!(sum, Rational::fraction(3, 10)?);
    assert_ne!(Rational::try_from(0.1_f64)?, tenth);
    let quotient = sum * Rational::new(3).inverse()?;
    assert_eq!(quotient, tenth);
    Ok(())
}
```

## Current behavior and limits

Integer inputs and ordinary ASCII integer, decimal, scientific and fraction
literals preserve their exact values. Finite binary32/binary64 imports preserve
the encoded binary value, including subnormals; `0.1_f64` is not decimal 1/10.
Both signed zeros become canonical rational zero. NaNs and infinities return
`Problem::NotANumber` and `Problem::Infinity` respectively.

`fraction` rejects a zero denominator, including 0/0, and `inverse` rejects zero
with `Problem::DivideByZero`. The `/` operator instead panics on a zero divisor,
including 0/0. Integer export rejects fractions before checking the target range;
it does not truncate. Root and other specialized helpers have their own domains
and decline/error contracts, documented with their implementations.

Display is presentation, not general exact replay: an improper fraction can
print as `1 2/3`, which the parser rejects. Alternate decimal formatting truncates
at its digit limit. Backend-accepted separator syntax also has known parser
defects: `1.2_5` currently parses as 41/40, not 1.25. See the
[parser implementation documentation](arithmetic/format_parse.rs) for the grammar
and other limitations. These defects are not additional exact syntax guarantees.

Shared reductions, dyadic paths and retained products can avoid repeated work,
but arbitrary-width arithmetic and retained result graphs have no general local
work, memory or cancellation budget. The scientific parser's exponent-expansion
guard reports `Problem::Exhausted` for excessive nonzero scale; it is not a bound
on all parsing or allocation. Cache-slot counts and inline node sizes likewise
do not bound retained bytes.

The [crate overview](../../README.md) introduces the public scalar API.
Measurements and optimization history belong in
[PERFORMANCE.md](../../PERFORMANCE.md); open work is tracked in
[FUTURE_WORK.md](../../FUTURE_WORK.md) and the repository audit ledger.
