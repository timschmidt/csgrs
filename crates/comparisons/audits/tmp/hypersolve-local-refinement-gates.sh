#!/usr/bin/env bash
set -euo pipefail
cd /home/tim/Documents/GitHub/workspace/hypersolve
export HYPERSOLVE_SKIP_BENCHMARK_REPORTS=1
cargo fmt --all -- --check > /tmp/hypersolve-local-refinement-fmt.log 2>&1
rustfmt --edition 2024 --check fuzz/fuzz_targets/hyperreal_representations.rs >> /tmp/hypersolve-local-refinement-fmt.log 2>&1
cargo check --locked --all-targets --all-features > /tmp/hypersolve-local-refinement-check.log 2>&1
cargo test --locked --no-default-features > /tmp/hypersolve-local-refinement-minimal.log 2>&1
cargo test --locked --all-targets --all-features > /tmp/hypersolve-local-refinement-all-targets.log 2>&1
scripts/representation_coverage.sh > /tmp/hypersolve-local-refinement-representations.log 2>&1
env HYPERSOLVE_ALLOCATION_ITERATIONS=4 scripts/allocation_profile.sh > /tmp/hypersolve-local-refinement-allocations.log 2>&1
cargo check --locked --manifest-path fuzz/Cargo.toml --bins > /tmp/hypersolve-local-refinement-fuzz-check.log 2>&1
cargo clippy --locked --manifest-path fuzz/Cargo.toml --bin hyperreal_representations -- -D warnings > /tmp/hypersolve-local-refinement-fuzz-clippy.log 2>&1
cargo clippy --locked --all-targets --all-features -- -D warnings > /tmp/hypersolve-local-refinement-clippy.log 2>&1
env RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features > /tmp/hypersolve-local-refinement-docs.log 2>&1
cargo test --release --locked --all-features > /tmp/hypersolve-local-refinement-release.log 2>&1
cargo bench --locked --all-features --no-run > /tmp/hypersolve-local-refinement-bench-build.log 2>&1
echo 'Hypersolve local-refinement gates complete'
