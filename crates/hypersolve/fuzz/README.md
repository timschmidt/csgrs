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

Run `cargo bench --bench retained_fuzz --features interval-lattice` to replay
every target registered in `fuzz/Cargo.toml`. The vertical-component and
Jacobian targets run their fuzz assertions on deterministic seed bytes. The
older adapters use target-shaped workloads, including shared sketch fixtures.
`cargo test --bench retained_fuzz --features interval-lattice` exercises one
seed per target without updating the reports. The refresh merges each target/seed
pair's worst-ever timing into `slow_performers.txt`, rotates the worst eligible
offender into the 100-case `promoted_slow_offenders.txt` lexicase set, emits a
dedicated Criterion row for every promoted case, and writes the suite's score,
delta, and delta derivative into `benchmarks.md`.

## Interval Krawczyk certificates

The interval conformance workspace provides the opt-in
`krawczyk_certificates` target. The oracle lives in Hypersolve and invokes
the affine, univariate quadratic and coupled multivariate quadratic public
certifiers under both predicate policies. Inputs use small integer coefficients
and independent quarter-unit radii.

The two-variable systems use `u = x + s*y` and `v = y - k*(u-p) - y₀`.
An invertible integer matrix mixes two residuals. In the affine case those
residuals are `u-p` and `v`. In the quadratic case they are `(u-p)*(u-q)`
and `v`. The roots can therefore be enumerated in the original coordinates,
including at box faces and on zero-width axes. The oracle also checks the
reported Newton steps, quadratic remainder bounds and dimensionless derivative
bounds by integer arithmetic and Cramer's rule. A box can contain a unique
root while failing this sufficient certificate.

Run the deterministic fixtures or Linux ASan campaign from `crates/hypersolve`.

```sh
cargo test --locked --test krawczyk_certificates
RUSTFLAGS='-C link-arg=-Wl,--compress-debug-sections=zlib' \
  cargo +nightly fuzz run krawczyk_certificates --fuzz-dir ../hyperinterval/fuzz \
  --features solver-certificates -- -runs=10000 -max_len=8 -max_total_time=90 -seed=20261119
```

The checked-in eight-byte seeds in `tests/fixtures/krawczyk/` run in the
native fixture suite and seed CI. They include both singular pivot positions
and two inputs that ranked first and second by median native replay time on
each of two pinned CPUs. To reproduce one input, pass its file path immediately
after the fuzz target name in the command above.

## Quadratic residuals over rectangles

The `conic_box_certificates` target checks the multivariate quadratic package
against extrema of `ax² + bxy + cy² + dx + ey + f` on a closed rectangle.
It evaluates corners, stationary points along each edge, and the interior
stationary point using integer arithmetic with exact rational coordinates.
A stationary line meets the boundary at the same value. The oracle checks
the signed center, row certainty and aggregate verdict under both policies.
Every singleton residual ball and every Taylor enclosure strictly separated
from zero requires a decision.

The eleven input bytes encode six signed coefficients, four endpoint bytes
and one mode byte. The signed endpoints are reduced modulo nine and ordered
on each axis. The low two mode bits select the constraint kind. The next two
select either the supplied constant or a shift of ±1,048,576. The checked-in
seeds include non-dyadic interior and edge extrema and a cross term that
changes the sign reached at a corner.

From `crates/hypersolve`, run the deterministic suite with
`cargo test --locked --test conic_box_certificates`. Copy
`tests/fixtures/conic/*.bin` into a writable corpus directory and pass that
directory to the sanitizer run.

```sh
RUSTFLAGS='-C link-arg=-Wl,--compress-debug-sections=zlib' \
  cargo +nightly fuzz run conic_box_certificates "$corpus" \
  --fuzz-dir ../hyperinterval/fuzz --features solver-certificates \
  -- -runs=10000 -max_len=11 -max_total_time=90 -seed=20261119
```
