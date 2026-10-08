#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
iterations="${HYPERTRI_ALLOCATION_ITERATIONS:-64}"

cd "${repo_dir}"
cargo run --release --example allocation_profile \
    --features all-algorithms,runtime-select -- "${iterations}"
