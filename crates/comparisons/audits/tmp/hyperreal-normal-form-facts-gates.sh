#!/usr/bin/env bash
set -euo pipefail
cargo clippy --locked --no-default-features --all-targets -- -D warnings > /tmp/hyperreal-normal-form-facts-clippy-minimal.log 2>&1
cargo check --locked --all-targets > /tmp/hyperreal-normal-form-facts-check.log 2>&1
cargo check --locked --manifest-path fuzz/Cargo.toml --bins > /tmp/hyperreal-normal-form-facts-fuzz.log 2>&1
env RUSTDOCFLAGS='-D warnings' cargo doc --locked --all-features --no-deps > /tmp/hyperreal-normal-form-facts-docs.log 2>&1
bash scripts/representation_coverage.sh > /tmp/hyperreal-normal-form-facts-layouts.log 2>&1
cargo run --release --locked --example allocation_profile -- 64 > /tmp/hyperreal-normal-form-facts-allocations.log 2>&1
cargo fmt --all -- --check > /tmp/hyperreal-normal-form-facts-fmt.log 2>&1
