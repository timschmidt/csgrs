#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

# Hyperreal's primitive approximation caches are dependency-only features.
# Exercise every cache layout so solver proofs are not validated under only
# one `Real` storage configuration.
cargo test --locked --no-default-features --test real_representations
cargo test --locked --no-default-features \
    --features hyperreal/cached-f32-approx \
    --test real_representations
cargo test --locked --no-default-features \
    --features hyperreal/cached-f64-approx \
    --test real_representations
cargo test --locked --all-features \
    --features hyperreal/cached-f32-approx,hyperreal/cached-f64-approx \
    --test real_representations
