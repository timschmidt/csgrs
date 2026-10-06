#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
iterations="${HYPERSOLVE_ALLOCATION_ITERATIONS:-32}"

cd "${repo_dir}"
cargo run --locked --release --all-features --example allocation_profile -- "${iterations}"
