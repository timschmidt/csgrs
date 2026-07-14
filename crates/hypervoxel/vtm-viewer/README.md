# VTM Viewer

`vtm-viewer` is an interactive Bevy viewer for Voxelis `.vtm` models. It loads
a selected LOD, generates a greedy render mesh, and provides orbit-camera,
wireframe, lighting, and diagnostic controls.

```text
vtm-viewer <input.vtm> [chunk_size_in_m] [lod]
```

Run it from the workspace with:

```sh
cargo run -p vtm-viewer --release -- model.vtm 1.28 0
```

The default chunk size is `1.28` metres and the default LOD is `0`.

The viewer requires a graphics-capable desktop environment and its Bevy assets.
It is a visualization adapter: the displayed primitive-float mesh is not exact
source geometry. Features `memory_stats` and `tracy` enable storage statistics
and profiling.

## References

- [Bevy 0.16 documentation](https://docs.rs/bevy/0.16).
- Mikola Lysenko, [“Meshing in a Minecraft Game”](https://0fps.net/2012/06/30/meshing-in-a-minecraft-game/), 2012.

See [Voxelis](../voxelis/README.md) for model and greedy-mesh APIs and
[HyperVoxel](../README.md) for exact-aware preview contracts.
