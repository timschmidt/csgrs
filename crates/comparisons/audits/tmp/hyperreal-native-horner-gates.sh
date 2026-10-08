#!/usr/bin/env bash
set -euo pipefail
cd /home/tim/Documents/GitHub/workspace/hyperreal
export HYPERREAL_SKIP_BENCHMARK_REPORTS=1
cargo fmt --all -- --check > /tmp/hyperreal-native-horner-fmt.log 2>&1
cargo check --locked --all-targets > /tmp/hyperreal-native-horner-check.log 2>&1
cargo test --locked --all-targets > /tmp/hyperreal-native-horner-default.log 2>&1
cargo clippy --locked --all-targets -- -D warnings > /tmp/hyperreal-native-horner-default-clippy.log 2>&1
cargo test --locked --all-features --all-targets > /tmp/hyperreal-native-horner-all-targets.log 2>&1
cargo clippy --locked --all-targets --all-features -- -D warnings > /tmp/hyperreal-native-horner-clippy.log 2>&1
scripts/representation_coverage.sh > /tmp/hyperreal-native-horner-representations.log 2>&1
cargo check --locked --manifest-path fuzz/Cargo.toml --bins > /tmp/hyperreal-native-horner-fuzz-check.log 2>&1
env RUSTDOCFLAGS='-D warnings' cargo doc --locked --all-features --no-deps > /tmp/hyperreal-native-horner-docs.log 2>&1
cargo test --release --locked --all-features --lib --tests > /tmp/hyperreal-native-horner-release.log 2>&1
cargo bench --locked --all-features --no-run > /tmp/hyperreal-native-horner-bench-build.log 2>&1
scripts/memory_profile.sh 4 > /tmp/hyperreal-native-horner-allocations.log 2>&1
echo 'Hyperreal native-Horner gates complete'
