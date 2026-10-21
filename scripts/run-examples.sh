#!/usr/bin/env bash
# Run every example binary from a scratch directory, so generated files stay
# out of the checkout. Examples are part of the shipped user path (P37).
# Each known failure is an open target in properties.md. A known failure that
# starts passing also fails this script, so the list stays accurate.
set -euo pipefail
root=$(git rev-parse --show-toplevel)
cd "${root}"

# Strict 32x16 sphere coplanarity is undecidable (boolean_operations,
# convex_hull).
known_failures=(boolean_operations convex_hull)

# Build every example: an example whose required features are not enabled is
# silently skipped by `cargo build --examples`, leaving no fresh binary.
features=$(cargo metadata --locked --no-deps --format-version 1 | python3 -c '
import json, sys
package = next(p for p in json.load(sys.stdin)["packages"] if p["name"] == "csgrs")
print(",".join(sorted({feature for target in package["targets"]
                       if "example" in target["kind"]
                       for feature in target.get("required-features", [])})))')
cargo build --locked --examples --features "${features}"
examples_dir="${CARGO_TARGET_DIR:-${root}/target}/debug/examples"
work=$(mktemp -d)
trap 'rm -rf "${work}"' EXIT

status=0
for source in examples/*.rs; do
  name=$(basename "${source}" .rs)
  expected=pass
  for known in "${known_failures[@]}"; do
    [[ "${name}" == "${known}" ]] && expected=fail
  done
  if (cd "${work}" && "${examples_dir}/${name}" >"${work}/${name}.log" 2>&1); then
    actual=pass
  else
    actual=fail
  fi
  echo "${name}: ${actual} (expected ${expected})"
  if [[ "${actual}" != "${expected}" ]]; then
    tail -n 20 "${work}/${name}.log"
    status=1
  fi
done
exit "${status}"
