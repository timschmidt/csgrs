#!/usr/bin/env bash
set -euo pipefail
cd /home/tim/Documents/GitHub/workspace/hypercurve
cargo test --release --locked --all-features --no-fail-fast -- --test-threads=2 > /tmp/hypercurve-affine-offset-all-features.log 2>&1
cargo test --release --locked --no-default-features --no-fail-fast -- --test-threads=2 > /tmp/hypercurve-affine-offset-minimal.log 2>&1
cargo test --release --locked --all-features --lib -- --ignored --nocapture --test-threads=2 > /tmp/hypercurve-affine-offset-ignored-units.log 2>&1
cargo test --release --locked --all-features --test hypercurve_curve_region_stroke -- --ignored --nocapture --test-threads=2 > /tmp/hypercurve-affine-offset-ignored-stroke.log 2>&1
cargo clippy --locked --all-targets --all-features -- -D warnings > /tmp/hypercurve-affine-offset-clippy.log 2>&1
cargo check --locked --manifest-path fuzz/Cargo.toml --bins > /tmp/hypercurve-affine-offset-fuzz-check.log 2>&1
env RUSTDOCFLAGS='-D warnings' cargo doc --locked --all-features --no-deps > /tmp/hypercurve-affine-offset-docs.log 2>&1
cargo bench --locked --all-features --no-run > /tmp/hypercurve-affine-offset-bench-build.log 2>&1
cargo fmt --all -- --check > /tmp/hypercurve-affine-offset-fmt.log 2>&1
cargo test --locked --manifest-path examples/hypercurve_ui/Cargo.toml --no-fail-fast > /tmp/hypercurve-affine-offset-ui-tests.log 2>&1
cargo clippy --locked --manifest-path examples/hypercurve_ui/Cargo.toml --all-targets -- -D warnings > /tmp/hypercurve-affine-offset-ui-clippy.log 2>&1
cargo build --locked --manifest-path examples/hypercurve_ui/Cargo.toml --release --target wasm32-unknown-unknown > /tmp/hypercurve-affine-offset-ui-wasm.log 2>&1
echo 'Hypercurve affine-offset gates complete'
