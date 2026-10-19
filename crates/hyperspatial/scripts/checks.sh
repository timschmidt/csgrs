#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$repo_dir"

cargo fmt --all -- --check
cargo fmt --manifest-path crates/hyperspatial/fuzz/Cargo.toml -- --check
cargo check --locked -p hyperspatial --all-targets --all-features
cargo test --locked -p hyperspatial --no-default-features
cargo test --locked -p hyperspatial --all-targets --all-features
cargo test --locked -p hyperspatial --doc --all-features
cargo clippy --locked -p hyperspatial --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="${RUSTDOCFLAGS:-} -D warnings" cargo doc --locked -p hyperspatial --no-deps --all-features
python3 crates/hyperspatial/scripts/source-consumer.py
cargo check --locked --manifest-path crates/hyperspatial/fuzz/Cargo.toml --bins
cargo clippy --locked --manifest-path crates/hyperspatial/fuzz/Cargo.toml --bins -- -D warnings
