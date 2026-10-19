#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$repo_dir"

# The exact scalar pair corpus is expensive without optimization. Keep the
# test profile's assertions and overflow checks in all four configurations;
# checks.sh separately exercises the complete unoptimized debug corpus.
export CARGO_PROFILE_TEST_OPT_LEVEL=1
export CARGO_PROFILE_TEST_DEBUG_ASSERTIONS=true
export CARGO_PROFILE_TEST_OVERFLOW_CHECKS=true

# These dependency features change Real's storage/cache layout. All existing
# strict, qualified, failure and oracle cases run in each configuration.
run_matrix() {
    local expected_f32="$1"
    local expected_clone="$2"
    shift 2
    HYPERREAL_EXPECT_F32_CACHE="$expected_f32" \
        HYPERREAL_EXPECT_EXACT_CLONE_CACHE="$expected_clone" \
        cargo test --locked -p hyperspatial --tests "$@"
    cargo test --locked -p hyperreal --test jacobi --test jacobi_prepared --features serde "$@"
    cargo test --locked -p hyperlattice --test complex_jacobi --test jacobi_epsilon --features serde "$@"
    cargo test --locked -p hyperlimit --test real_representations --features serde "$@"
    cargo clippy --locked -p hyperspatial --all-targets "$@" -- -D warnings
}

run_matrix absent absent --no-default-features
run_matrix present present --no-default-features --features hyperreal/cached-f32-approx
run_matrix absent present --no-default-features --features hyperreal/cached-f64-approx
run_matrix present present --all-features --features hyperreal/cached-f32-approx,hyperreal/cached-f64-approx
