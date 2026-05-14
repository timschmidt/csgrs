#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

run() {
  printf '\n==> %s\n' "$*"
  "$@"
}

expect_fail() {
  printf '\n==> expecting failure: %s\n' "$*"
  if "$@"; then
    printf 'command unexpectedly succeeded: %s\n' "$*" >&2
    exit 1
  fi
}

run cargo test --test adversarial
run cargo test --test adversarial_deep
run cargo test --test adversarial_extrusions
run cargo test --test adversarial_stress
run cargo test --test adversarial_fixtures

feature_sets=(
  "f64 delaunay"
  "f64 earcut"
  "f64 delaunay-rs"
  "f64 parallel delaunay"
  "f64 parallel earcut"
  "f32 delaunay"
  "f32 earcut"
  "f32 delaunay-rs"
  "f64 delaunay stl-io dxf-io obj-io ply-io amf-io gltf-io gerber-io chull-io metaballs sdf offset bmesh"
  "f32 earcut stl-io dxf-io obj-io ply-io amf-io gltf-io gerber-io chull-io metaballs sdf offset bmesh"
)

for features in "${feature_sets[@]}"; do
  run cargo check --no-default-features --features "$features"
done

expect_fail cargo check --no-default-features --features "f32 f64 delaunay"
expect_fail cargo check --no-default-features --features "f64"
expect_fail cargo check --no-default-features --features "f64 delaunay earcut"
expect_fail cargo check --no-default-features --features "f64 delaunay delaunay-rs"

run cargo check --manifest-path fuzz/Cargo.toml --locked

if [[ "${RUN_FUZZ_SECONDS:-0}" != "0" ]]; then
  cargo_fuzz=(cargo fuzz)
  if cargo +nightly fuzz --help >/dev/null 2>&1; then
    cargo_fuzz=(cargo +nightly fuzz)
  elif ! command -v cargo-fuzz >/dev/null 2>&1; then
    printf 'RUN_FUZZ_SECONDS was set, but cargo-fuzz is not installed.\n' >&2
    exit 1
  fi

  fuzz_targets=(
    fuzz_mesh_bytecode
    fuzz_sketch_polygon_triangulate
    fuzz_obj_import
    fuzz_svg_import
    fuzz_gerber_import
    fuzz_dxf_import
    fuzz_transform_matrix
    fuzz_export_names
    fuzz_plane_split_polygon
    fuzz_toolpath_from_sketch
    fuzz_mesh_primitive_catalog
    fuzz_sketch_shape_catalog
    fuzz_sdf_tpms
    fuzz_vertex_arithmetic
    fuzz_sketch_extrude_revolve_sweep
    fuzz_mesh_polyhedron_constructor
    fuzz_mesh_boolean_pair
    fuzz_sketch_boolean_pair
    fuzz_vertex_quality
  )

  for target in "${fuzz_targets[@]}"; do
    run "${cargo_fuzz[@]}" run "$target" -- -max_total_time="$RUN_FUZZ_SECONDS"
  done
fi
