#!/usr/bin/env bash
set -euo pipefail
cd /home/tim/Documents/GitHub/workspace/hypercurve
cargo test --release --locked --all-features --no-fail-fast -- --test-threads=2 > /tmp/hypercurve-normal-form-facts-all-features.log 2>&1
cargo test --release --locked --no-default-features --no-fail-fast -- --test-threads=2 > /tmp/hypercurve-normal-form-facts-minimal.log 2>&1
cargo test --release --locked --all-features --lib -- --ignored --nocapture --test-threads=2 > /tmp/hypercurve-normal-form-facts-ignored-units.log 2>&1
cargo test --release --locked --all-features --test hypercurve_curve_region_stroke -- --ignored --nocapture --test-threads=2 > /tmp/hypercurve-normal-form-facts-ignored-stroke.log 2>&1
cargo clippy --locked --all-targets --all-features -- -D warnings > /tmp/hypercurve-normal-form-facts-clippy.log 2>&1
cargo check --locked --manifest-path fuzz/Cargo.toml --bins > /tmp/hypercurve-normal-form-facts-fuzz-check.log 2>&1
env RUSTDOCFLAGS='-D warnings' cargo doc --locked --all-features --no-deps > /tmp/hypercurve-normal-form-facts-docs.log 2>&1
cargo bench --locked --all-features --no-run > /tmp/hypercurve-normal-form-facts-bench-build.log 2>&1
cargo fmt --all -- --check > /tmp/hypercurve-normal-form-facts-fmt.log 2>&1
cargo test --locked --manifest-path examples/hypercurve_ui/Cargo.toml --no-fail-fast > /tmp/hypercurve-normal-form-facts-ui-tests.log 2>&1
cargo clippy --locked --manifest-path examples/hypercurve_ui/Cargo.toml --all-targets -- -D warnings > /tmp/hypercurve-normal-form-facts-ui-clippy.log 2>&1
cargo build --locked --manifest-path examples/hypercurve_ui/Cargo.toml --release --target wasm32-unknown-unknown > /tmp/hypercurve-normal-form-facts-ui-wasm.log 2>&1
