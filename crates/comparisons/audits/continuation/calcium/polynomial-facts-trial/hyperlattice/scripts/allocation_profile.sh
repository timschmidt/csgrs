#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
iterations="${HYPERLATTICE_ALLOCATION_ITERATIONS:-128}"

cd "${repo_dir}"
cargo run --release --all-features --example allocation_profile -- "${iterations}"
