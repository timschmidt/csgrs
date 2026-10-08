# Hyperlimit fuzzing

`predicate_invariants` exercises the public 2D/3D predicate families, scalar
and Rayon-batch agreement, retained evidence/report replay, and metamorphic
laws over generated exact rationals.
`hyperreal_representations` constructs all eight public Hyperreal structural
kinds and all twenty-two optimized scalar certificates (`One` through
`Irrational`). Every execution crosses each certificate through orientation,
sidedness, sign, scalar/batch/Rayon parity, and ordering; a fuzzed rotation
eventually pairs every representation with every other representation without
an expensive quadratic inner loop. Fuzzer bytes also vary rational
offsets/scales, triangle steps, shear, and strict versus terminal-approximation
policy. The target also grows shared opaque computable DAGs to fuzz variable
depth and operation sequences. The serde-enabled integration matrix separately
drift-checks all 57 finite computable node variants and all 18 shared-constant
payloads; recursive graph topology itself is unbounded.

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo +nightly fuzz run predicate_invariants --fuzz-dir fuzz -- -max_total_time=30
cargo +nightly fuzz run hyperreal_representations --fuzz-dir fuzz -- -max_total_time=30
```

## Retained performance offenders

`cargo bench --bench retained_fuzz` deterministically replays inputs shaped by
both registered fuzz targets. Each run merges worst-ever target/seed timings
into `slow_performers.txt`, rotates the worst eligible offender into the
100-case `promoted_slow_offenders.txt` lexicase set, emits a dedicated Criterion
row for every promoted case, and refreshes the score, delta, and delta
derivative in `benchmarks.md`.
