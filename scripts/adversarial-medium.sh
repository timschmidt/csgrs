#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

run() {
  printf '\n==> %s\n' "$*"
  "$@"
}

run cargo test --test adversarial
run cargo test --test adversarial_deep
run cargo test --test adversarial_extrusions
run cargo test --test adversarial_stress
run cargo test --test adversarial_fixtures
run cargo test --doc
run cargo check --features "parallel"
