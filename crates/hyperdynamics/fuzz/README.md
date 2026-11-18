# Hyperdynamics fuzz targets

`coordinate_change` checks the Hypersolve chart and Hyperdynamics consumer together.
Eight bytes select a unimodular shear, translation, polynomial evaluation point,
time and constant velocity. Independent integer formulas check transformed
nonlinear values and Jacobians. Each input also rejects a wrong inverse and a
short graph budget, replays the field/chart, and certifies then transports a
constant-flow step and one to four whole-horizon segments with independent exact
endpoints in both coordinate systems. All selected cases must
complete; deliberate failure controls do not count as geometric success.

```sh
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo clippy --locked --manifest-path fuzz/Cargo.toml --bins -- -D warnings
cargo +nightly fuzz build coordinate_change --fuzz-dir fuzz
cargo +nightly fuzz run coordinate_change --fuzz-dir fuzz -- \
  -runs=10000 -max_len=8 -max_total_time=60 -seed=20261113
```

The input cap and libFuzzer deadline contain the campaign. They do not establish
whole-work bounds for arbitrary exact scalars or arbitrary coordinate dimension.

## Reachability policies

`reachability_policy` exercises the public cover search. The high bit of the
eighth byte selects one of two independent integer oracles.

- Constant two-dimensional velocities translate a rectangle. The oracle
  reconstructs the partition from its widest-axis rule and checks the entire
  final Cartesian grid. It verifies each native Picard endpoint and the box
  carried into the next step. Generated policies reach or fall one short of
  the required cell, depth, evolution and segment counts.
- For a point evolving under `x' = a*t + b`, the exact solution increment
  is the integral of that affine function. On a uniform mesh the left and
  right rectangle sums enclose the integral and differ by `|a|*h*u` after
  elapsed time `u` with step `h`. The oracle checks those two values at
  each prefix, every discarded coarse attempt, and completion at the exact
  requested width. One fewer time refinement must withhold the final cover.

Both families retry limited requests with the same field and remaining
arithmetic budget. The ordinary integration suite exercises 126 deterministic
configurations, including ties, signed time, and capped last steps. From
`crates/hyperdynamics`, run the fixtures or the ASan campaign with:

```sh
cargo test --locked --features reachability --test reachability_policy
RUSTFLAGS='-C link-arg=-Wl,--compress-debug-sections=zlib' \
  cargo +nightly fuzz run reachability_policy -- -runs=10000 -max_len=8 -max_total_time=60 -seed=20261118
```

The Linux CI campaign compresses ELF debug sections at link time to retain
source locations within its executable size allowance.
