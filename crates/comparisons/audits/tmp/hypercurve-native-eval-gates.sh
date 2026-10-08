#!/usr/bin/env bash
set -euo pipefail
cd /home/tim/Documents/GitHub/workspace/hypercurve
cargo test --release --locked --no-default-features --no-fail-fast -- --test-threads=2 > /tmp/hypercurve-native-eval-minimal.log 2>&1
cargo test --release --locked --all-features --lib -- --ignored --nocapture --test-threads=2 > /tmp/hypercurve-native-eval-ignored-units.log 2>&1
cargo test --release --locked --all-features --test hypercurve_curve_region_stroke -- --ignored --nocapture --test-threads=2 > /tmp/hypercurve-native-eval-ignored-stroke.log 2>&1
cargo clippy --locked --all-targets --all-features -- -D warnings > /tmp/hypercurve-native-eval-clippy.log 2>&1
cargo check --locked --manifest-path fuzz/Cargo.toml --bins > /tmp/hypercurve-native-eval-fuzz-check.log 2>&1
env RUSTDOCFLAGS='-D warnings' cargo doc --locked --all-features --no-deps > /tmp/hypercurve-native-eval-docs.log 2>&1
cargo bench --locked --all-features --no-run > /tmp/hypercurve-native-eval-bench-build.log 2>&1
cargo fmt --all -- --check > /tmp/hypercurve-native-eval-fmt.log 2>&1
cargo test --locked --manifest-path examples/hypercurve_ui/Cargo.toml --no-fail-fast > /tmp/hypercurve-native-eval-ui-tests.log 2>&1
cargo clippy --locked --manifest-path examples/hypercurve_ui/Cargo.toml --all-targets -- -D warnings > /tmp/hypercurve-native-eval-ui-clippy.log 2>&1
cargo build --locked --manifest-path examples/hypercurve_ui/Cargo.toml --release --target wasm32-unknown-unknown > /tmp/hypercurve-native-eval-ui-wasm.log 2>&1
echo 'Hypercurve native-eval gates complete'
