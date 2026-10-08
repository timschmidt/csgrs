#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

# Hyperreal's primitive approximation caches are dependency-only features.
# Exercise every compile-time cache layout as well as both Hyperlattice feature
# surfaces so a representation cannot be validated under only one storage form.
cargo test --no-default-features --test real_representations
cargo test --no-default-features \
    --features hyperreal/cached-f32-approx \
    --test real_representations
cargo test --no-default-features \
    --features hyperreal/cached-f64-approx \
    --test real_representations
cargo test --all-features \
    --features hyperreal/cached-f32-approx,hyperreal/cached-f64-approx \
    --test real_representations
