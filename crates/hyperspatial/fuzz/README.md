# Hyperspatial directed inputs

The `forward_local` target checks positive rational sphere/oblate/prolate
surfaces at exact special angles, both local-axis conventions, orthonormality,
and repeated source/evidence-preserving composition. It is a separate,
unpublished development package; libFuzzer is not a production dependency.

From the repository root:

```sh
cargo check --locked --manifest-path crates/hyperspatial/fuzz/Cargo.toml --bins
cargo +nightly fuzz run forward_local --fuzz-dir crates/hyperspatial/fuzz -- -max_total_time=30 -timeout=10 -rss_limit_mb=2048
```

Retain every crash/timeout and its minimized counterpart, then promote the
failure to a deterministic regression without weakening the original case.
The input generator is bounded; arbitrary native scalar allocations and time
inside a scalar call remain unbounded. The libFuzzer containment flags are
therefore part of the campaign command. A build alone establishes neither an
ASan campaign nor coverage. Lexicase selection across representation families,
dispatch tracing, and comparative benchmarks remain separate admission work.
