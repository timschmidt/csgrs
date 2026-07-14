# Voxelis Voxelize

`voxelis-voxelize` converts OBJ triangle meshes into sampled Voxelis models.
It partitions faces by chunk, tests triangles against candidate cells in
parallel, and writes occupied cells through `VoxModel<i32>` batches.

## API

- `Voxelizer::new` sizes the model from the parsed mesh.
- `Voxelizer::empty` starts with an unbounded empty model.
- `build_face_to_chunk_map` computes the chunk work schedule.
- `voxelize_mesh` consumes a prepared schedule; `voxelize` performs the full
  workflow; `simple_voxelize` provides the direct comparison path.
- `Voxelizer::model` contains the resulting `VoxModel<i32>`.

```rust,no_run
use std::path::Path;
use voxelis::{MaxDepth, io::Obj};
use voxelis_voxelize::Voxelizer;

let mesh = Obj::parse(Path::new("model.obj"));
let mut voxelizer = Voxelizer::new(
    MaxDepth::new(8),
    1.0,
    mesh,
    512 * 1024 * 1024,
);
voxelizer.voxelize();
println!("{} chunks", voxelizer.model.chunks.len());
```

The triangle/cube tests use floating tolerances. The result is appropriate for
legacy storage, rendering, and comparisons, not exact source-geometry proof.

Features `memory_stats` and `tracy` forward the corresponding Voxelis
instrumentation.

## References

- Tomas Akenine-Möller, [“Fast 3D Triangle-Box Overlap Testing”](https://doi.org/10.1145/1198555.1198748), 2005.
- Viktor Kämpe, Erik Sintorn, and Ulf Assarsson, [“High Resolution Sparse Voxel DAGs”](https://doi.org/10.1145/2461912.2462024), 2013.

See [Voxelis](../voxelis/README.md) for storage APIs and [HyperVoxel](../README.md)
for exact-aware voxelization.
