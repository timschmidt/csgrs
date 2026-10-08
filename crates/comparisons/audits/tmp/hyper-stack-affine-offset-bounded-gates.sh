#!/usr/bin/env bash
set -euo pipefail
export HYPERLATTICE_SKIP_BENCHMARK_REPORTS=1
export HYPERTRI_SKIP_BENCHMARK_REPORTS=1
export HYPERMESH_SKIP_BENCHMARK_REPORTS=1
for crate in hyperlimit hyperlattice hypertri hypermesh; do
    cd "/home/tim/Documents/GitHub/workspace/${crate}"
    cargo test --release --locked --all-features --no-fail-fast -- --test-threads=2 > "/tmp/${crate}-affine-offset-bounded-all-features.log" 2>&1
    cargo check --locked --no-default-features > "/tmp/${crate}-affine-offset-bounded-minimal-check.log" 2>&1
    cargo clippy --locked --all-targets --all-features -- -D warnings > "/tmp/${crate}-affine-offset-bounded-clippy.log" 2>&1
    echo "${crate} affine-offset cross-stack gates complete"
done
