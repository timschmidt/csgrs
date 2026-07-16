<h1>
  hypervoxel
  <img src="./docs/voxelis_logo.png" alt="hypervoxel logo" width="144" align="right">
</h1>

`hypervoxel` is the exact-aware voxel layer of the Hyper geometry stack. It
owns grid frames, integer cell addresses, sparse voxel facts, exact
classification reports, and explicit handoffs to storage, meshing, simulation,
and preview adapters.

The repository also contains the harvested [Voxelis](https://github.com/WildPixelGames/voxelis)
SVO-DAG workspace. Voxelis supplies useful storage and rendering foundations;
the root `hypervoxel` crate supplies the semantic contract that prevents those
representations from silently becoming source geometry.

## Design

Voxel pipelines often collapse coordinate frames, quantization, boundary
policy, occupancy, material labels, compression, meshing, and rendering into
one sampled array. `hypervoxel` keeps those decisions observable:

- frame coordinates use `hyperreal::Real`, while `VoxelAddress` uses exact
  integer grid coordinates;
- occupancy distinguishes filled, empty, boundary, mixed, unknown, and lossy
  adapter states;
- exact classifiers return predicate accounting and provenance alongside the
  grid;
- empty reports do not become vacuous evidence;
- compression, paging, previews, and legacy imports must replay retained object
  facts before an exact-readiness flag can become true.

This separation also keeps exact arithmetic compact. Frames, integer addresses,
payload IDs, aggregate facts, and replay reports are retained instead of
expanding every cell into coordinate geometry.

## API Overview

| Task | Primary types and functions |
| --- | --- |
| Define a grid | `GridFrame::builder`, `GridSource`, `LengthUnit`, `GridFrameFacts` |
| Address cells | `VoxelAddress`, `CellBounds`, `ChunkAddress`, `ChunkShape` |
| Store facts | `VoxelCell`, `VoxelPayload`, `SparseVoxelGrid`, `VoxelEditBatch` |
| Attach domain records | `VoxelSideTables`, `MaterialRegionRecord`, `FieldSampleRecord`, `ProcessStateRecord` |
| Voxelize primitives | `ExactBox`, `ExactHalfSpace`, `ExactConvexHalfSpaceSet`, and their `voxelize_*` functions |
| Voxelize triangles | `ExactTriangleSurfaceMesh`, `ExactTriangleSolidMesh`, `PreparedExactTriangleSolidMesh` |
| Query and summarize | `VoxelAggregateFacts`, `VoxelSpatialAggregateFacts`, `select_lod_cells`, `voxel_neighbors6` |
| Extract exact surfaces | `extract_exposed_faces_with_report`, `sparse_exact_surface_triangle_mesh_with_report` |
| Use compressed storage | `ChunkPagedSparseGrid`, `SvoVoxelGrid`, deterministic snapshot and replay reports |
| Build adapters | `AdapterNumericContract`, `VoxelIoReport`, `PreviewExportReport`, `VoxelHandoffReport` |

Most operations return a result plus a report. Inspect readiness methods such as
`VoxelizationReport::exact_topology_ready` rather than inferring exactness from
a nonempty grid or a successful function return.

## Installation

For a sibling Hyper checkout:

```toml
[dependencies]
hypervoxel = { path = "../hypervoxel" }
```

The crate manifest is currently version `0.3.0`; once that release is available
from the selected registry, use:

```toml
[dependencies]
hypervoxel = "0.3"
```

Optional features:

- `hypermesh-adapter` validates and imports retained `hypermesh::InputMesh`
  solids.
- `legacy-voxelis` enables sampled interoperability with the harvested Voxelis
  storage backend. It does not promote legacy samples to exact source geometry.

## Quick Start

Build an exact frame, voxelize a box, and inspect the report before consuming
its topology:

```rust
use hyperreal::Real;
use hypervoxel::{
    ExactBox, GridFrame, GridSource, HypervoxelResult, LengthUnit,
    MaterialRegionId, VoxelizationPolicy, voxelize_exact_box,
};

fn main() -> HypervoxelResult<()> {
    let source = GridSource::new("example:box", 1);
    let frame = GridFrame::builder()
        .units(LengthUnit::Millimeter)
        .origin([0.into(), 0.into(), 0.into()])
        .pitch([1.into(), 1.into(), 1.into()])
        .depth(3)
        .source(source.clone())
        .build()?;
    let solid = ExactBox::new(
        [Real::from(1), Real::from(1), Real::from(1)],
        [Real::from(3), Real::from(3), Real::from(3)],
        Some(source),
    );

    let (grid, report) = voxelize_exact_box(
        frame,
        &solid,
        MaterialRegionId(7),
        VoxelizationPolicy::conservative_cover(),
    )?;

    assert_eq!(grid.len(), 8);
    assert!(report.exact_topology_ready());
    Ok(())
}
```

For direct sparse edits, validate the address against the frame and retain the
edit report:

```rust
use hypervoxel::{
    GridFrame, HypervoxelResult, MaterialRegionId, SparseVoxelGrid,
    VoxelAddress, VoxelCell,
};

fn edit_one_cell() -> HypervoxelResult<()> {
    let frame = GridFrame::builder()
        .pitch([1.into(), 1.into(), 1.into()])
        .depth(3)
        .build()?;
    let mut grid = SparseVoxelGrid::new(frame);
    let address = VoxelAddress::new(3, [2, 1, 0])?;
    let edit = grid.set(address, VoxelCell::material(MaterialRegionId(4)))?;
    assert!(edit.exact_edit_replay_ready);
    Ok(())
}
```

Runnable versions are in [`examples/exact_box.rs`](examples/exact_box.rs) and
[`examples/sparse_grid.rs`](examples/sparse_grid.rs).

## Common Workflows

- Use `voxelize_exact_halfspace` or
  `voxelize_exact_convex_halfspace_set` for proof-producing linear predicates.
- Prepare a closed triangle solid with `PreparedExactTriangleSolidMesh::prepare`
  before selecting a per-cell, component, or axis-sweep voxelization schedule.
- Call `query_material_regions`, `query_field_samples`, and the side-table audit
  functions before handing payload IDs to a domain crate.
- Use `extract_exposed_faces_with_report` for exact lattice faces. Primitive
  OBJ/quad output is deliberately named as a lossy adapter.
- Use `ChunkPagedSparseGrid` or `SvoVoxelGrid` when storage scale matters, then
  consume their replay reports rather than treating page or DAG layout as
  topology.
- Use `diff_sparse_grids`, deterministic snapshots, and trace reports for
  reproducible backend comparisons and optimization work.

## Status and Scope

Implemented today:

- exact frames, addresses, cells, side tables, edit batches, and sparse storage;
- exact box, half-space, convex-half-space, triangle-surface, and closed-triangle
  solid classification;
- prepared triangle schedules with explicit acceleration and fallback evidence;
- aggregate, LOD, neighbor, connected-component, broad-phase, support, path,
  and Manhattan-distance reports;
- exact exposed-face and indexed lattice-surface handoffs;
- chunk-paged and SVO-DAG replay, deterministic snapshots, compression, memory,
  IO, artifact, coupling, and domain-handoff reports;
- feature-gated `hypermesh` and legacy Voxelis adapters.

Production out-of-core pipelines, GPU renderers, general implicit-field
voxelization, and complete mesh/physics/process bridges remain future work.
Preview and legacy routes are intentionally not substitutes for exact source
replay.

## Development

The root workspace defaults to the harvested Voxelis members, so select the
HyperVoxel package explicitly:

```sh
cargo fmt --all -- --check
cargo test -p hypervoxel --all-features --locked
cargo clippy -p hypervoxel --all-targets --all-features --locked -- -D warnings
cargo doc -p hypervoxel --all-features --no-deps --locked
cargo run -p hypervoxel --example exact_box --locked
cargo run -p hypervoxel --example sparse_grid --locked
cargo bench -p hypervoxel --bench grid_frame
```

Benchmark methodology and results are in [`docs/benches.md`](docs/benches.md).
The root crate's reference-audit measurements and retained changes are in
[`PERFORMANCE.md`](PERFORMANCE.md).

The optional `dispatch-trace` feature forwards `hyperreal`'s exact-dispatch
instrumentation. Its integration test requires representative rational
voxelization to produce no approximation or unknown-fact events.

## Reference-Guided Design

The references define algorithm choices and exactness boundaries; listing a
paper does not imply that every algorithm in it is implemented:

- Yap supplies the exact-geometric-computation rule: combinatorial voxel and
  mesh decisions use certified predicates, while previews remain named as
  lossy or non-topological.
- Moore, Kearfott, and Cloud motivate certified intervals for occupancy,
  fields, transformed bounds, and unresolved boundary evidence.
- Rosenfeld and Pfaltz's sequential picture operations motivate exact
  six-neighbor components; their distance-functions paper motivates the
  separable integer Manhattan transform and source-aware readiness report.
- Bresenham motivates integer address-segment stepping. Amanatides and Woo
  motivate bounded grid-ray traversal, while the API deliberately distinguishes
  this address traversal from a continuous geometric ray predicate.
- Bentley and Ottmann motivate separating candidate reporting from exact
  intersection decisions. Kay and Kajiya motivate retained AABB hierarchy and
  page evidence as broad-phase acceleration rather than object truth.
- Möller's triangle-triangle test and Möller–Trumbore ray-triangle test provide
  fast proposal structures; Guigue and Devillers motivates the accepted exact
  path based on orientation/determinant signs without fragile constructed
  floating-point intersections.
- Lorensen and Cline motivate explicit sampled isosurface adapters. Marching
  Cubes is not used to certify source topology; exact voxel shells instead come
  from replayed exposed lattice faces.
- Kämpe, Sintorn, and Assarsson motivate bottom-up SVO-DAG interning and replay
  of shared subtrees rather than inferring geometry from compression.
- Botsch and coauthors motivate indexed vertex/edge/face vocabularies, manifold
  incidence audits, and distinct exact and preview mesh handoffs.
- Lysenko motivates greedy coplanar face patches, whose compressed cover is
  expanded and compared with the exact shell before becoming ready.
- Arvo supplies the per-axis affine AABB transform used instead of transforming
  eight corners.
- ISO 10303-242 motivates explicit product/process provenance, coordinate
  systems, versioned artifacts, and domain handoff manifests. The crate does
  not claim that its voxel formats are STEP AP242 encodings.

## References

- Chee-Keng Yap, [“Towards Exact Geometric Computation”](https://doi.org/10.1016/0925-7721%2895%2900040-2), *Computational Geometry* 7(1–2), 1997.
- Ramon E. Moore, R. Baker Kearfott, and Michael J. Cloud, [*Introduction to Interval Analysis*](https://doi.org/10.1137/1.9780898717716), SIAM, 2009.
- Azriel Rosenfeld and John L. Pfaltz, [“Sequential Operations in Digital Picture Processing”](https://doi.org/10.1145/321356.321357), *JACM* 13(4), 1966.
- Azriel Rosenfeld and John L. Pfaltz, [“Distance Functions on Digital Pictures”](https://doi.org/10.1016/0031-3203%2868%2990013-7), *Pattern Recognition* 1(1), 1968.
- Jack E. Bresenham, [“Algorithm for Computer Control of a Digital Plotter”](https://doi.org/10.1147/sj.41.0025), *IBM Systems Journal* 4(1), 1965.
- Jon L. Bentley and Thomas A. Ottmann, [“Algorithms for Reporting and Counting Geometric Intersections”](https://doi.org/10.1109/TC.1979.1675432), *IEEE Transactions on Computers* C-28(9), 1979.
- Timothy L. Kay and James T. Kajiya, [“Ray Tracing Complex Scenes”](https://doi.org/10.1145/15886.15916), *SIGGRAPH Computer Graphics* 20(4), 1986.
- John Amanatides and Andrew Woo, [“A Fast Voxel Traversal Algorithm for Ray Tracing”](https://doi.org/10.2312/egtp.19871000), *Eurographics ’87*.
- Tomas Möller, [“A Fast Triangle-Triangle Intersection Test”](https://doi.org/10.1080/10867651.1997.10487472), *Journal of Graphics Tools* 2(2), 1997.
- Tomas Möller and Ben Trumbore, [“Fast, Minimum Storage Ray-Triangle Intersection”](https://doi.org/10.1080/10867651.1997.10487468), *Journal of Graphics Tools* 2(1), 1997.
- Philippe Guigue and Olivier Devillers, [“Fast and Robust Triangle-Triangle Overlap Test Using Orientation Predicates”](https://doi.org/10.1080/10867651.2003.10487580), *Journal of Graphics Tools* 8(1), 2003.
- William E. Lorensen and Harvey E. Cline, [“Marching Cubes”](https://doi.org/10.1145/37402.37422), *SIGGRAPH Computer Graphics* 21(4), 1987.
- Viktor Kämpe, Erik Sintorn, and Ulf Assarsson, [“High Resolution Sparse Voxel DAGs”](https://doi.org/10.1145/2461912.2462024), *ACM Transactions on Graphics* 32(4), 2013.
- Mario Botsch et al., [*Polygon Mesh Processing*](https://doi.org/10.1201/b10688), A K Peters/CRC Press, 2010.
- Mikola Lysenko, [“Meshing in a Minecraft Game”](https://0fps.net/2012/06/30/meshing-in-a-minecraft-game/), 0fps, 2012.
- James Arvo, [“Transforming Axis-Aligned Bounding Boxes”](https://www.realtimerendering.com/resources/GraphicsGems/gems/TransBox.c), in *Graphics Gems*, 1990.
- ISO, [ISO 10303-242:2025, STEP AP242](https://www.iso.org/standard/84300.html).

## Hyper Ecosystem

Core dependencies: [hyperreal](https://github.com/timschmidt/hyperreal),
[hyperlattice](https://github.com/timschmidt/hyperlattice), and
[hyperlimit](https://github.com/timschmidt/hyperlimit). Related geometry and
consumer crates: [hypermesh](https://github.com/timschmidt/hypermesh),
[hypertri](https://github.com/timschmidt/hypertri),
[hyperbrep](https://github.com/timschmidt/hyperbrep), and
[hypersdf](https://github.com/timschmidt/hypersdf). See the
[remaining Hyper repositories](https://github.com/timschmidt?tab=repositories&q=hyper)
for solver and domain consumers.
