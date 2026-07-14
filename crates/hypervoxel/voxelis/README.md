# Voxelis

`voxelis` is a sparse voxel octree/DAG engine. It interns identical leaves and
branches, path-copies edits through integer voxel addresses, supports batched
updates, and generates naive or greedy surface meshes.

This directory is harvested from the upstream
[Voxelis project](https://github.com/WildPixelGames/voxelis). The surrounding
repository also contains `hypervoxel`, an exact-aware semantic layer; Voxelis
itself remains a high-performance sampled storage and rendering engine.

## Core API

- `VoxTree<T>` stores one fixed-depth voxel volume.
- `VoxInterner<T>` owns shared DAG nodes under an explicit memory budget.
- `MaxDepth`, `TraversalDepth`, `Lod`, and `BlockId` make tree depth and node
  identity explicit.
- `VoxOpsRead`, `VoxOpsWrite`, `VoxOpsBulkWrite`, and `VoxOpsBatch` provide the
  storage operations implemented by trees, chunks, and models.
- `VoxChunk<T>`, `VoxModel<T>`, and `VoxWorld<T>` organize trees in world space.
- `MeshData` and the mesh helpers produce naive or greedy render geometry.

## Example

```rust
use glam::IVec3;
use voxelis::{MaxDepth, VoxInterner};
use voxelis::spatial::{VoxOpsRead, VoxOpsWrite, VoxTree};

let mut interner = VoxInterner::with_memory_budget(1024 * 1024);
let mut tree = VoxTree::<u8>::new(MaxDepth::new(3));
let position = IVec3::new(2, 1, 0);

assert!(tree.set(&mut interner, position, 7));
assert_eq!(tree.get(&interner, position), Some(7));
```

Coordinates passed to a `VoxTree` must be nonnegative and less than
`2^max_depth` on every axis. The voxel type must implement `VoxelTrait`; with
the default `numeric_voxel_impls` feature, common numeric types are supported.

## Features

- `numeric_voxel_impls` implements `VoxelTrait` for numeric primitives.
- `vtm` enables VTM import/export and its compression dependencies.
- `memory_stats` records allocator/interner statistics.
- `tracy` enables Tracy instrumentation.
- `debug_trace_ref_counts` and `trace_greedy_timings` enable specialized
  diagnostics.

## Development

```sh
cargo test -p voxelis --all-features --locked
cargo clippy -p voxelis --all-targets --all-features --locked -- -D warnings
cargo bench -p voxelis --bench voxtree_bench
```

## References

- Viktor Kämpe, Erik Sintorn, and Ulf Assarsson, [“High Resolution Sparse Voxel DAGs”](https://doi.org/10.1145/2461912.2462024), 2013.
- Samuli Laine and Tero Karras, [“Efficient Sparse Voxel Octrees”](https://doi.org/10.1145/1730804.1730814), 2010.
- Mikola Lysenko, [“Meshing in a Minecraft Game”](https://0fps.net/2012/06/30/meshing-in-a-minecraft-game/), 2012.

See the repository-level [HyperVoxel guide](../README.md) for exact grid facts,
proof-bearing classification, and links to the wider Hyper geometry stack.
