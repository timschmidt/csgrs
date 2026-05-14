#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

run() {
  printf '\n==> %s\n' "$*"
  "$@"
}

run bash scripts/adversarial-medium.sh

if command -v cargo-fuzz >/dev/null 2>&1; then
  RUN_FUZZ_SECONDS="${RUN_FUZZ_SECONDS:-30}" run bash scripts/adversarial.sh
else
  printf '\n==> skipping cargo-fuzz campaigns because cargo-fuzz is not installed\n'
fi

if command -v cargo-miri >/dev/null 2>&1; then
  run cargo miri test --lib
else
  printf '\n==> skipping miri because cargo-miri is not installed\n'
fi

if [[ "${RUN_SANITIZERS:-0}" != "0" ]]; then
  run cargo test --tests --target x86_64-unknown-linux-gnu
fi
