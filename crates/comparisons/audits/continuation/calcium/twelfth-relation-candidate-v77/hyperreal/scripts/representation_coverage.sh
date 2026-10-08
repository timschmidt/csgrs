#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

run_matrix() {
    local expected_f32="$1"
    local expected_exact_clone="$2"
    local label="$3"
    shift 3
    echo "Representation configuration: ${label}"
    HYPERREAL_EXPECT_F32_CACHE="${expected_f32}" \
        HYPERREAL_EXPECT_EXACT_CLONE_CACHE="${expected_exact_clone}" \
        cargo test --no-default-features "$@" --test real_representations --quiet
}

run_matrix absent absent "no primitive caches"
run_matrix present present "binary32 cache only" --features cached-f32-approx
run_matrix absent present "binary64 cache only" --features cached-f64-approx
run_matrix present present "both primitive caches" \
    --features cached-f32-approx,cached-f64-approx
run_matrix present present "all features plus private serde inventory" --all-features
