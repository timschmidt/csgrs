#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

# Hyperreal's primitive approximation caches are dependency-only features.
# Exercise every cache layout while keeping all native triangulation surfaces
# available; the final configuration also verifies serde's private class tags.
cargo test --no-default-features \
    --features all-algorithms,runtime-select \
    --test real_representations
cargo test --no-default-features \
    --features all-algorithms,runtime-select,hyperreal/cached-f32-approx \
    --test real_representations
cargo test --no-default-features \
    --features all-algorithms,runtime-select,hyperreal/cached-f64-approx \
    --test real_representations
cargo test --all-features \
    --features hyperreal/cached-f32-approx,hyperreal/cached-f64-approx \
    --test real_representations
