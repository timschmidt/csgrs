# Coordinate-change fuzzing

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
