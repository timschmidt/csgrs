# Hypersolve fuzzing

The suite covers algebraic transforms, exact linear algebra, curve resultants
and substitutions, active-set regeneration, failed constraints, and the
public sketch constraint families. `hyperreal_representations` is fail-closed
over all 22 optimized finite Hyperreal certificate classes (covering all eight
public structural kinds). Every execution sends every class through symbolic
analysis, retained and tree evaluation, candidate/domain certification, exact
dense and sparse replay, and a rotated cross-class expression pair. Input bytes
select the ordered-pair stride and also construct variable-depth, shared opaque
computable DAGs, covering the unbounded representation space beyond the finite
certificate enum.

```sh
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo +nightly fuzz run hyperreal_representations --fuzz-dir fuzz -- -max_total_time=30
```

For a deterministic smoke run that still executes every finite class:

```sh
cargo +nightly fuzz run hyperreal_representations --fuzz-dir fuzz -- -runs=100
```

Some ptrace-restricted hosts make LeakSanitizer abort before the target runs.
On those hosts, keep AddressSanitizer enabled and disable only leak detection:

```sh
ASAN_OPTIONS=detect_leaks=0 \
  cargo +nightly fuzz run hyperreal_representations --fuzz-dir fuzz -- -runs=100
```

Use `scripts/allocation_profile.sh` separately for steady-state allocation and
end-of-epoch live-byte accounting across every finite certificate class.

## Retained performance offenders

Run `cargo bench --bench retained_fuzz` to replay deterministic inputs for every
target registered in `fuzz/Cargo.toml`. The refresh merges each target/seed
pair's worst-ever timing into `slow_performers.txt`, rotates the worst eligible
offender into the 100-case `promoted_slow_offenders.txt` lexicase set, emits a
dedicated Criterion row for every promoted case, and writes the suite's score,
delta, and delta derivative into `benchmarks.md`.
