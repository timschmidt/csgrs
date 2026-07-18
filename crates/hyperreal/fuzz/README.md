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
```

Long-running campaigns should retain each target's corpus separately. A crash
is a semantic regression until minimized and promoted to a deterministic test.
