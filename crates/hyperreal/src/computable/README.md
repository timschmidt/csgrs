# Computable

`Computable` is Hyperreal's retained exact-real expression graph. `Real` uses it
when a value or decision cannot remain in a cheaper rational or recognized
symbolic form. Exact recipes remain authoritative while constructors and queries
reuse structural facts and numerical samples.

## Architecture

| Source | Responsibility |
| --- | --- |
| `mod.rs` | Connect the implementation modules and reexport the graph handle and shared helpers. |
| `node.rs`, `node/` | Handles, shared caches/facts, constructors, structural recognizers, comparison and outer evaluation. Included files share one private namespace. |
| `approximation.rs`, `approximation/` | Retained expression enum, integer kernel dispatch, argument reduction and numerical evaluation. |
| `constants.rs` | Documentation anchor; actual constant construction and evaluation live in the two namespaces above. |
| `format.rs` | Decimal and scientific display, including rounding at the formatting boundary. |

Cloned handles share immutable recipes and node-owned accelerator state. A handle
can also carry its own abort signal. Named constants use process-wide caches;
a newly constructed result can therefore reuse evidence warmed by another graph.
The storage and publication layers have distinct responsibilities: cache storage
trusts its caller, while ordinary evaluation checks its signal before publishing.
A returned integer alone does not indicate whether interrupted work completed.

## Current behavior and limits

The graph includes exact rational leaves, shared constants, arithmetic, elementary
functions, selected algebraic-root nodes, elliptic and statistical constructions,
and specialized stable forms. Constructors can preserve identities and simplify
compositions, but can also evaluate operands; construction is not uniformly lazy.
Internal/native constructors assume the domains documented on their methods.
Use checked `Real` entry points for scalar admission rather than treating a raw
recipe or successful native deserialization as a general domain proof.

On valid, completed evaluation, `approx(p)` returns an integer representing the
value in units of `2^p`, with the method's one-unit error contract. It is not a
canonical or generally correctly rounded integer. More negative `p` requests
finer absolute accuracy; guard bits, child requests and formatting can require
additional refinement. Extreme exponents are not uniformly checked, and precision
is not a cumulative work, coefficient-size or memory budget.

Structural recognition may certify a sign or identity, or decline without proving
inequality. `compare_absolute` compares signed values at an absolute tolerance;
its `Equal` result is not generally an exact equality certificate. Cache facts
must retain their exactness qualification and cannot replace the original recipe.
Local depth, visit and degree limits are documented where they apply; they do not
establish a total resource bound or complete support for arbitrary expressions.

The governing contracts are in [properties.md](../../../../properties.md) and
[the admission protocol](../../../../docs/admission.md). The
[Rustdoc audit](../../../../docs/research/rustdoc-audit.md) records reviewed domains,
finite validation and remaining implementation gaps; these are distinct from
universal mathematical completeness.

## Build and inspect

From the repository root:

```sh
cargo check -p hyperreal --all-features
cargo doc -p hyperreal --all-features --no-deps --document-private-items
cargo run -p hyperreal --example computable_graphs
cargo run -p hyperreal --example computable_refinement_steps
```

Private library docs expose implementation details, but ordinary Rustdoc omits
`cfg(test)` fixtures and function-local items. The audit tracks those separately.
The examples print authored diagrams, facts and samples; their output is not an
internal graph trace, an independent oracle or a benchmark. See the
[graph walkthrough](../../../../docs/research/computable-graph-walkthrough.md)
for the expressions, diagrams, observed output and interpretation.
