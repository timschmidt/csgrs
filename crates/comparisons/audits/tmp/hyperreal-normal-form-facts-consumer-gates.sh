#!/usr/bin/env bash
set -euo pipefail
for crate in hypersolve hypertri hypermesh; do
    cd "/home/tim/Documents/GitHub/workspace/$crate"
    cargo test --release --locked --all-features --no-fail-fast -- --test-threads=2 > "/tmp/$crate-normal-form-facts-all-features.log" 2>&1
    cargo test --release --locked --no-default-features --no-fail-fast -- --test-threads=2 > "/tmp/$crate-normal-form-facts-minimal.log" 2>&1
    cargo clippy --locked --all-features --all-targets -- -D warnings > "/tmp/$crate-normal-form-facts-clippy.log" 2>&1
done
cd /home/tim/Documents/GitHub/workspace/hypermesh
cargo test --release --locked --all-features --test competitive dense_crossing_grid_large_policy_outputs_are_exactly_equal -- --ignored --exact --nocapture --test-threads=1 > /tmp/hypermesh-normal-form-facts-dense-crossing.log 2>&1
