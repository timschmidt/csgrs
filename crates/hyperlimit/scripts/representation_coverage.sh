#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

# Hyperreal's cache features change which private primitive-cache states can be
# retained and cloned. Test every compile-time combination instead of assuming
# that Hyperlimit's own --all-features switch activates dependency-only flags.
HYPERLIMIT_EXPECT_F32_CACHE=absent \
    cargo test --no-default-features --test real_representations
HYPERLIMIT_EXPECT_F32_CACHE=present \
    cargo test --no-default-features \
    --features hyperreal/cached-f32-approx \
    --test real_representations
HYPERLIMIT_EXPECT_F32_CACHE=absent \
    cargo test --no-default-features \
    --features hyperreal/cached-f64-approx \
    --test real_representations
HYPERLIMIT_EXPECT_F32_CACHE=present \
    cargo test --all-features \
    --features hyperreal/cached-f32-approx,hyperreal/cached-f64-approx \
    --test real_representations
