#!/usr/bin/env bash
# Exercise the generated WASM ABI and TypeScript declarations with real clients.
# Run from any directory; paths below are resolved after changing to the repository.
# Requires Python 3 with tomllib, Cargo's wasm32 target, Node, wasm-bindgen-cli
# matching Cargo.lock and the pinned test TypeScript dependency already installed.
# WASM_BINDGEN and TSC override executable paths; this script installs no tools.
# Cargo metadata selects the target directory, honoring Cargo configuration such
# as CARGO_TARGET_DIR. Generated bindings and copied/compiled consumers live there.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

# Select tools without changing their versions; the bindgen CLI must match its Rust crate.
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

# Build the actual wasm profile and generate Node glue, rather than substituting a native loader.
wasm_target_dir="$(cargo metadata --locked --no-deps --format-version 1 |
    python3 -c 'import json, sys; print(json.load(sys.stdin)["target_directory"])')"
wasm_bindings_dir="${wasm_target_dir}/wasm-bindings-node"
cargo build --locked --profile wasm --target wasm32-unknown-unknown \
    --no-default-features --features wasm
"${bindgen_bin}" --target nodejs --out-name csgrs --out-dir "${wasm_bindings_dir}" \
    "${wasm_target_dir}/wasm32-unknown-unknown/wasm/csgrs.wasm"
node tests/wasm/geometry_certainty.cjs "${wasm_bindings_dir}/csgrs.js"
node tests/wasm/export_projection.cjs "${wasm_bindings_dir}/csgrs.js"
node tests/wasm/io_contracts.cjs "${wasm_bindings_dir}/csgrs.js"
node tests/wasm/mesh_errors.cjs "${wasm_bindings_dir}/csgrs.js"
node tests/wasm/containment_winding.cjs "${wasm_bindings_dir}/csgrs.js"
# This consumer is type-checked only; the CJS witness above supplies runtime coverage.
cp tests/wasm/geometry_certainty.ts "${wasm_bindings_dir}/geometry_certainty.ts"
"${tsc_bin}" --noEmit --strict --target ES2020 --lib ES2020,DOM,ESNext.Disposable \
    --module Node16 --moduleResolution Node16 \
    "${wasm_bindings_dir}/geometry_certainty.ts"

# Compile and execute the native format pipeline through generated declarations.
cp tests/wasm/io_contracts.ts "${wasm_bindings_dir}/io_contracts.ts"
"${tsc_bin}" --strict --target ES2020 --lib ES2020,DOM,ESNext.Disposable \
    --module Node16 --moduleResolution Node16 --outDir "${wasm_bindings_dir}" \
    "${wasm_bindings_dir}/io_contracts.ts"
node "${wasm_bindings_dir}/io_contracts.js"

# Exercise typed mesh failures through the generated TypeScript declarations.
cp tests/wasm/mesh_errors.ts "${wasm_bindings_dir}/mesh_errors.ts"
"${tsc_bin}" --strict --target ES2020 --lib ES2020,DOM,ESNext.Disposable \
    --module Node16 --moduleResolution Node16 --outDir "${wasm_bindings_dir}" \
    "${wasm_bindings_dir}/mesh_errors.ts"
node "${wasm_bindings_dir}/mesh_errors.js"

# Retain native construction qualifications through extraction and bound STL artifacts.
cp tests/wasm/construction_evidence.ts "${wasm_bindings_dir}/construction_evidence.ts"
"${tsc_bin}" --strict --target ES2020 --lib ES2020,DOM,ESNext.Disposable \
    --module Node16 --moduleResolution Node16 --outDir "${wasm_bindings_dir}" \
    "${wasm_bindings_dir}/construction_evidence.ts"
node "${wasm_bindings_dir}/construction_evidence.js"
