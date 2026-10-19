# Hyperspatial directed inputs

The `forward_local` target checks positive rational sphere/oblate/prolate
surfaces at exact special angles, closest-point inversion with certified
latitude/height/forward-residual enclosures, both local-axis conventions,
orthonormality, and repeated source/evidence-preserving composition. It is a separate,
unpublished development package; libFuzzer is not a production dependency.
The `normal_offset` target uses Hyperreal's 22 shared representation recipes
for positive axes and heights, checks the independently constructed normal foot,
then consumes the inverse in fresh forward evaluation and local vector operations.
The `mercator` target uses the same scalar inventory for independently
parameterized latitudes and longitudes, all three longitude policies, false
origins, scales, signed heights and oblate/prolate/spherical source ellipsoids.
It checks fresh inverse/forward arithmetic and downstream ECEF/local use.

From the repository root:

```sh
cargo check --locked --manifest-path crates/hyperspatial/fuzz/Cargo.toml --bins
python3 crates/hyperspatial/scripts/directed.py
python3 crates/hyperspatial/scripts/directed.py --workload mercator --output target/hyperspatial-directed-mercator
python3 crates/hyperspatial/scripts/fuzz-smoke.py --selected-corpus target/hyperspatial-directed/corpus --mercator-corpus target/hyperspatial-directed-mercator/corpus
```

Retain every crash/timeout and its minimized counterpart, then promote the
failure to a deterministic regression without weakening the original case.
The input generator is bounded; arbitrary native scalar allocations and time
inside a scalar call remain unbounded. The libFuzzer containment flags are
therefore part of the campaign command. Install nightly Rust and the pinned
development tool with `cargo install --locked cargo-fuzz --version 0.13.2`.
The smoke driver supplies 918 initial forward/local cases, 44 normal-offset
cases, 44 projection cases, and any cases selected by the directed driver.
All three targets use ASan,
a 60-second libFuzzer time budget, a 10-second per-input timeout, and a 2048 MiB
RSS limit. Initialization can consume or extend the time budget; the raw log
distinguishes corpus replay from mutation and any newly discovered paths.

The directed driver runs two generations of four candidates in each of 22
independent first-scalar-family buckets. Exact lexicase selection shuffles the
criteria (checked workflow time, dispatch count, work checkpoints), retains the
largest observed score at each step and breaks remaining ties randomly. The
second generation retains and mutates the selected parent without changing its
family. Failed/undecided/timed-out cases are retained and fail the campaign;
they cannot win as fast successes. These are cost-discovery samples, not a
statistical performance comparison. JSON reports and compressed raw dispatch
observations record all inputs, outcomes, scores, seeds, source digests and
tool configuration under `target/hyperspatial-directed` and
`target/hyperspatial-asan`. Override destinations with `--output`.

The [directed-testing admission](../docs/admission/e1-directed-testing.md) records
actual executed evidence and remaining limits. A build alone establishes
neither an ASan campaign nor coverage.
