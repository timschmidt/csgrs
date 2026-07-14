# VTM Export

`vtm-export` converts a Voxelis `.vtm` model to a level-zero OBJ surface mesh.

```text
vtm-export <input.vtm> <output.obj>
```

Run it from the workspace with:

```sh
cargo run -p vtm-export --release -- model.vtm model.obj
```

The tool loads the model with a 1 GiB interner budget and exports a render mesh;
OBJ output is a sampled interchange artifact, not exact source geometry. Enable
the `tracy` feature for profiling.

See [Voxelis](../voxelis/README.md) for VTM and mesh APIs and
[HyperVoxel](../README.md) for exact-aware handoffs and the Hyper ecosystem.
