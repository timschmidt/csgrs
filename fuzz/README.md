# CSGRS fuzz targets

This unpublished `cargo-fuzz` package stress-tests CSGRS parsers, constructors,
transforms, profile-to-solid grammar, Hypermesh/Hypercurve handoffs, and
export boundaries. It is a developer tool, not a library dependency.

## Run

Install the nightly fuzz driver and list targets:

```sh
cargo install cargo-fuzz
cd fuzz
cargo fuzz list
```

Run one bounded campaign:

```sh
cargo fuzz run fuzz_mesh_boolean_pair -- -max_total_time=60
```

Reproduce a saved input with:

```sh
cargo fuzz run fuzz_mesh_boolean_pair path/to/artifact
```

## Target groups

- grammar and topology: `fuzz_mesh_bytecode`,
  `fuzz_sketch_polygon_triangulate`, `fuzz_plane_split_polygon`,
  `fuzz_mesh_boolean_pair`, `fuzz_sketch_boolean_pair`;
- formats: `fuzz_obj_import`, `fuzz_svg_import`, `fuzz_gerber_import`,
  `fuzz_dxf_import`, `fuzz_export_names`;
- construction: `fuzz_mesh_primitive_catalog`,
  `fuzz_sketch_shape_catalog`, `fuzz_mesh_polyhedron_constructor`,
  `fuzz_sketch_extrude_revolve_sweep`;
- Hyper handoffs and implicit geometry: `fuzz_mesh_hypermesh_adapter`,
  `fuzz_sdf_tpms`, `fuzz_metaballs_boundary`;
- local carriers: `fuzz_transform_matrix`, `fuzz_vertex_arithmetic`,
  `fuzz_vertex_quality`, `fuzz_part_blueprint`.

Targets bound tessellation, recursion, bytecode length, and other generated
work before entering expensive exact geometry. A timeout is not automatically
a correctness bug; minimize and replay it under the corresponding semantic or
adversarial test before promoting it.

Keep crash artifacts private until their input provenance and licensing are
understood. Add minimized, license-clean regressions to ordinary tests when
possible. This package is `publish = false` and follows the CSGRS MIT license.
