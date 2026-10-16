#!/usr/bin/env bash
# Exercise the generated WASM ABI and TypeScript declarations with real clients.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

bindgen_bin="${WASM_BINDGEN:-wasm-bindgen}"
tsc_bin="${TSC:-tests/wasm/node_modules/.bin/tsc}"
locked_version="$(python3 - <<'PY'
import tomllib
with open('Cargo.lock', 'rb') as lock:
    packages = tomllib.load(lock)['package']
print(next(p['version'] for p in packages if p['name'] == 'wasm-bindgen'))
PY
)"
if [[ "$("${bindgen_bin}" --version)" != "wasm-bindgen ${locked_version}" ]]; then
    printf 'Use wasm-bindgen-cli %s, matching Cargo.lock (WASM_BINDGEN may select its binary).\n' "${locked_version}" >&2
    exit 1
fi

wasm_target_dir="$(cargo metadata --locked --no-deps --format-version 1 |
    python3 -c 'import json, sys; print(json.load(sys.stdin)["target_directory"])')"
wasm_bindings_dir="${wasm_target_dir}/wasm-bindings-node"
cargo build --locked --profile wasm --target wasm32-unknown-unknown \
    --no-default-features --features wasm
"${bindgen_bin}" --target nodejs --out-name csgrs --out-dir "${wasm_bindings_dir}" \
    "${wasm_target_dir}/wasm32-unknown-unknown/wasm/csgrs.wasm"
node tests/wasm/geometry_certainty.cjs "${wasm_bindings_dir}/csgrs.js"
node tests/wasm/export_projection.cjs "${wasm_bindings_dir}/csgrs.js"
cp tests/wasm/geometry_certainty.ts "${wasm_bindings_dir}/geometry_certainty.ts"
"${tsc_bin}" --noEmit --strict --target ES2020 --lib ES2020,DOM,ESNext.Disposable \
    --module Node16 --moduleResolution Node16 \
    "${wasm_bindings_dir}/geometry_certainty.ts"
