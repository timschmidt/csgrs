# Hyperreal fuzzing

These targets keep their generated inputs in bounded exact-rational form. They
exercise construction, arithmetic, structural/certified queries, fused linear
algebra, lazy elementary-function evaluation, serialization, and direct
`Computable` approximation without treating a primitive-float result as proof.

Compile every target:

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
```

Run a bounded smoke pass from the repository root:

```sh
cargo +nightly fuzz run rational_arithmetic --fuzz-dir fuzz -- -max_total_time=30
cargo +nightly fuzz run real_exact --fuzz-dir fuzz -- -max_total_time=30
cargo +nightly fuzz run real_elementary --fuzz-dir fuzz -- -max_total_time=30
cargo +nightly fuzz run computable_approximation --fuzz-dir fuzz -- -max_total_time=30
cargo +nightly fuzz run structural_representations --fuzz-dir fuzz -- -max_total_time=30
cargo +nightly fuzz run string_parsing --fuzz-dir fuzz -- -max_total_time=30
```

Long-running campaigns should retain each target's corpus separately. A crash
is a semantic regression until minimized and promoted to a deterministic test.

`structural_representations` constructs all 20 private optimized `Real`
certificate forms, which span all eight public `StructuralKind` values.
Twenty seed strides cover the complete ordered 20x20 binary-dispatch matrix;
mutated bytes also build shared `Computable` DAGs with up to 32 variable
nodes. The deterministic serde inventory test covers every finite
`Computable` node and shared-constant tag, while this target covers the
unbounded graph-topology dimension.

`string_parsing` checks that every accepted exact `Rational` literal is
also accepted by `Real`. Its seed corpus includes signed wide integers,
general fractions, long decimals, scientific notation, and invalid
zero-denominator input.
