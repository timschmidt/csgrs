#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

# Hyperreal's primitive approximation caches are dependency-only features.
# Run four selected feature/cache configurations sequentially and stop on failure.
# This visits the four explicit optional cache combinations under the crate's
# current empty defaults; it is not the Cartesian product with all lattice features.
# It runs the real_representations integration target, not a line-coverage report.
# Cargo environment/toolchain settings are inherited; no timeout or resource bound
# is added, and no per-run feature-isolation or exhaustive semantic claim follows.
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
