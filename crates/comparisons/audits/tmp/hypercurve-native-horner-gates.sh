#!/usr/bin/env bash
set -euo pipefail
cd /home/tim/Documents/GitHub/workspace/hypercurve
cargo test --release --locked --all-features --no-fail-fast -- --test-threads=2 > /tmp/hypercurve-native-horner-all-features.log 2>&1
cargo test --release --locked --no-default-features --no-fail-fast -- --test-threads=2 > /tmp/hypercurve-native-horner-minimal.log 2>&1
cargo test --release --locked --all-features --lib -- --ignored --nocapture --test-threads=2 > /tmp/hypercurve-native-horner-ignored-units.log 2>&1
cargo test --release --locked --all-features --test hypercurve_curve_region_stroke -- --ignored --nocapture --test-threads=2 > /tmp/hypercurve-native-horner-ignored-stroke.log 2>&1
cargo clippy --locked --all-targets --all-features -- -D warnings > /tmp/hypercurve-native-horner-clippy.log 2>&1
cargo check --locked --manifest-path fuzz/Cargo.toml --bins > /tmp/hypercurve-native-horner-fuzz-check.log 2>&1
env RUSTDOCFLAGS='-D warnings' cargo doc --locked --all-features --no-deps > /tmp/hypercurve-native-horner-docs.log 2>&1
cargo bench --locked --all-features --no-run > /tmp/hypercurve-native-horner-bench-build.log 2>&1
cargo fmt --all -- --check > /tmp/hypercurve-native-horner-fmt.log 2>&1
cargo test --locked --manifest-path examples/hypercurve_ui/Cargo.toml --no-fail-fast > /tmp/hypercurve-native-horner-ui-tests.log 2>&1
cargo clippy --locked --manifest-path examples/hypercurve_ui/Cargo.toml --all-targets -- -D warnings > /tmp/hypercurve-native-horner-ui-clippy.log 2>&1
cargo build --locked --manifest-path examples/hypercurve_ui/Cargo.toml --release --target wasm32-unknown-unknown > /tmp/hypercurve-native-horner-ui-wasm.log 2>&1
echo 'Hypercurve native-horner gates complete'
