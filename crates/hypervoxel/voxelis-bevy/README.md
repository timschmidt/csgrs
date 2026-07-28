# Voxelis Bevy

`voxelis-bevy` contains interactive Bevy examples for Voxelis LOD selection and
greedy-mesh generation. The library target currently exports no stable API;
use the examples as integration references.

```sh
cargo run -p voxelis-bevy --example lod --release
cargo run -p voxelis-bevy --example greedy_meshing --release
```

The examples demonstrate `VoxModel`, `Lod`, greedy mesh arrays, Bevy mesh
assets, orbit controls, wireframes, and runtime memory/performance diagnostics.
They require a graphics-capable desktop environment and the assets expected by
the example code.

Features:

- `memory_stats` enables Voxelis allocator statistics and is on by default.
- `tracy` enables Tracy instrumentation.
- `trace_greedy_timings` exposes greedy-meshing timing details.

## References

- Mikola Lysenko, [“Meshing in a Minecraft Game”](https://0fps.net/2012/06/30/meshing-in-a-minecraft-game/), 2012.
- [Bevy documentation](https://docs.rs/bevy/0.16).

See [Voxelis](../voxelis/README.md) for storage and mesh types and
[HyperVoxel](../README.md) for the exact-aware layer.

## Acknowledgements and license

This package is bundled from
[WildPixelGames/Voxelis](https://github.com/WildPixelGames/voxelis), created by
Artur Wyszyński and its contributors. It retains the upstream
[MIT](../LICENSE-MIT) OR [Apache-2.0](../LICENSE-APACHE) license.
