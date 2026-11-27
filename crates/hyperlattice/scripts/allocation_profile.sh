#!/usr/bin/env bash
set -euo pipefail

# Run the example from this crate directory with release/all-feature Cargo settings.
# Iterations come only from the environment (unset/empty means 128); positional
# script arguments are ignored. The example validates positivity and parsing.
# This wrapper supplies no isolation, timeout, memory bound, or output-file capture.
# Inherit Cargo/Rust flags and target storage; the printed counters are windowed
# allocator observations, not total live memory. See the example's private docs.

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
iterations="${HYPERLATTICE_ALLOCATION_ITERATIONS:-128}"

cd "${repo_dir}"
cargo run --release --all-features --example allocation_profile -- "${iterations}"
