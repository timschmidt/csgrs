# Hyperlimit fuzzing

`predicate_invariants` exercises selected public 2D/3D predicates, scalar and
Rayon-batch agreement, retained evidence/report replay, and metamorphic laws.
Four planar and five spatial points use exact rational coordinates with i16
numerators and denominators 1–16. Fixed boundary and separation fixtures extend
those generated cases. Most calls use `APPROXIMATE_512`; planar orientation also
checks `STRICT` agreement when the approximate-policy call decides. Many laws
and report checks are conditional on a decided result, and most assertions
compare values without certainty receipts.

`hyperreal_representations` constructs 20 fixed recipes spanning all eight public
Hyperreal structural kinds, plus four opaque computable graphs. It checks kinds
before nonzero rational scaling; it does not assert every private certificate.
Each normally decoded execution visits all 24 carriers, pairing each with one
rotated partner. Across possible stride values every rotation is available;
this is not a claim that a particular fuzz run visits every pair. The stride
also selects policy and graph depth, so those controls are correlated.
Orientation reversal and scalar/batch/Rayon values must agree, and a source
point must classify as on its line. Sign and ordering are invoked without result
assertions. Matching uncertainty can satisfy orientation comparisons.

The opaque recipes include sine/cosine of e, a mathematically zero identity,
and a shared graph with 1–8 update rounds. This bounds construction depth, not
total evaluation work or memory. The separate
[representation integration corpus](../tests/real_representations.rs) contains
22 certificate recipes, including `Pow10` and `Pow2`, and serde-gated inventory
checks for 70 node tags and 18 shared-constant tags. Finite recipe/tag coverage
does not establish coverage of arbitrary values or recursive compositions.

Run from `crates/hyperlimit`. The fuzz workspace enables `std` and `parallel`
on Hyperlimit. Compilation checks the targets but does not execute assertions;
fuzz runs need external time and memory containment. Neither target implements
a whole-execution work, memory, or cancellation budget.

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo +nightly fuzz run predicate_invariants --fuzz-dir fuzz -- -max_total_time=30
cargo +nightly fuzz run hyperreal_representations --fuzz-dir fuzz -- -max_total_time=30
```

The bins have `doc = false` to exclude them from ordinary package documentation.
Private source documentation can still be requested explicitly, for example:

```sh
cargo rustdoc --manifest-path fuzz/Cargo.toml --bin predicate_invariants -- --document-private-items
cargo rustdoc --manifest-path fuzz/Cargo.toml --bin hyperreal_representations -- --document-private-items
```

Nested helpers and the libFuzzer functions generated inside an anonymous
constant are not all exposed as item pages. Their behavior is described in
source/module docs; attached documentation for generated items remains an open
coverage gap tracked in the [audit ledger](../../../docs/research/rustdoc-audit.md).

## Retained performance offenders

`cargo bench --bench retained_fuzz` uses its own deterministic seed generators
under the two registered target names. It does not decode libFuzzer bytes or
execute these targets' invariant checks. By default it probes 20,000 cases total
across both targets and merges maximum historical target/seed probe times into
`slow_performers.txt`. Rotation releases the fastest existing promotion and
backfills from eligible history toward at most 100 cases. Each retained case
gets a Criterion row; the aggregate row's fixed name does not guarantee 100 cases.

With report writes enabled, refresh also updates `promoted_slow_offenders.txt`
and the score section in `benchmarks.md`. The score is an arithmetic mean of
fresh minimum probe times, despite the report's lexicase terminology. Its delta
and change in delta do not establish comparable workloads or significant speed
changes. Writes are best-effort and not transactional.
`HYPERLIMIT_SKIP_BENCHMARK_REPORTS` suppresses persistence and score probes, but
initial collection and promotion re-probes still run. These timings do not
check predicate completion or certainty.
