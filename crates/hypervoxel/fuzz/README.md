# HyperVoxel Fuzz Targets

This `cargo-fuzz` package exercises exact grid addressing, continuous-field
materialization, triangle surface and solid voxelization, scheduled-triangle
reuse, and the retained `hypermesh` adapter.

Run one target from the repository root, for example:

```sh
cargo fuzz run grid_address
cargo fuzz run triangle_solid_voxelization
cargo fuzz run hypermesh_adapter
```

The targets treat panics, invalid certificates, inconsistent round trips, and
violated exact/conservative report invariants as failures. They use local Hyper
crates so a run covers the current ecosystem checkout rather than published
versions.

## Targets

- `grid_address` covers frames, addresses, sparse storage, serialization, LOD,
  side tables, and handoff reports.
- `continuous_field_materialization` covers certified continuous-field sampling.
- `triangle_surface_voxelization` and `triangle_solid_voxelization` cover direct
  mesh conversion.
- `triangle_solid_voxelization` covers reusable exact geometry and
  row caches.
- `hypermesh_adapter` covers retained-mesh validation and conversion.
- `hyperreal_representations` crosses every pair of the eight public Hyperreal
  structural kinds through grid frames, address bounds, exact boxes, bounded
  sign certification, and voxelization.

## References

- [The Rust Fuzz Book](https://rust-fuzz.github.io/book/) documents `cargo-fuzz` workflows and corpus management.
- [HyperVoxel](../README.md) documents the invariants and references exercised by these targets.

HyperVoxel integrates [hyperreal](https://github.com/timschmidt/hyperreal),
[hyperlattice](https://github.com/timschmidt/hyperlattice),
[hyperlimit](https://github.com/timschmidt/hyperlimit), and
[hypermesh](https://github.com/timschmidt/hypermesh).
