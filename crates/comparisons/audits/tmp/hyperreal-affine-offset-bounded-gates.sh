#!/usr/bin/env bash
set -euo pipefail
cd /home/tim/Documents/GitHub/workspace/hyperreal
export HYPERREAL_SKIP_BENCHMARK_REPORTS=1
cargo fmt --all -- --check > /tmp/hyperreal-affine-offset-bounded-fmt.log 2>&1
cargo check --locked --all-targets > /tmp/hyperreal-affine-offset-bounded-check.log 2>&1
cargo test --locked --lib --tests --examples > /tmp/hyperreal-affine-offset-bounded-default.log 2>&1
cargo clippy --locked --all-targets -- -D warnings > /tmp/hyperreal-affine-offset-bounded-default-clippy.log 2>&1
cargo test --locked --all-features --lib --tests --examples > /tmp/hyperreal-affine-offset-bounded-all-targets.log 2>&1
cargo clippy --locked --all-targets --all-features -- -D warnings > /tmp/hyperreal-affine-offset-bounded-clippy.log 2>&1
scripts/representation_coverage.sh > /tmp/hyperreal-affine-offset-bounded-representations.log 2>&1
cargo check --locked --manifest-path fuzz/Cargo.toml --bins > /tmp/hyperreal-affine-offset-bounded-fuzz-check.log 2>&1
env RUSTDOCFLAGS='-D warnings' cargo doc --locked --all-features --no-deps > /tmp/hyperreal-affine-offset-bounded-docs.log 2>&1
cargo test --release --locked --all-features --lib --tests > /tmp/hyperreal-affine-offset-bounded-release.log 2>&1
cargo bench --locked --all-features --no-run > /tmp/hyperreal-affine-offset-bounded-bench-build.log 2>&1
scripts/memory_profile.sh 4 > /tmp/hyperreal-affine-offset-bounded-allocations.log 2>&1
echo 'Hyperreal bounded affine-offset gates complete'
