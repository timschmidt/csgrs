# VTM Voxelize

`vtm-voxelize` samples an OBJ triangle mesh into a Voxelis model and writes the
result as `.vtm`.

```text
vtm-voxelize <max_depth> <chunk_size_in_m> <input.obj> <output.vtm>
```

For example:

```sh
cargo run -p vtm-voxelize --release -- 8 1.0 model.obj model.vtm
```

The current CLI reserves a 16 GiB interner budget. Voxelization uses
floating-point triangle/cube tests and produces sampled storage rather than an
exact source-geometry certificate. Enable `memory_stats` (default) for allocator
statistics or `tracy` for profiling.

## References

- Tomas Akenine-Möller, [“Fast 3D Triangle-Box Overlap Testing”](https://doi.org/10.1145/1198555.1198748), 2005.
- Viktor Kämpe, Erik Sintorn, and Ulf Assarsson, [“High Resolution Sparse Voxel DAGs”](https://doi.org/10.1145/2461912.2462024), 2013.

See [Voxelis Voxelize](../voxelis-voxelize/README.md) for the library workflow
and [HyperVoxel](../README.md) for exact-aware voxelization.

## Acknowledgements and license

This tool is bundled from
[WildPixelGames/Voxelis](https://github.com/WildPixelGames/voxelis), created by
Artur Wyszyński and its contributors. It retains the upstream
[MIT](../LICENSE-MIT) OR [Apache-2.0](../LICENSE-APACHE) license.
