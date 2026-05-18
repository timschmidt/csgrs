<h1>
  hypervoxel
  <img src="./docs/voxelis_logo.png" alt="hypervoxel logo" width="144" align="right">
</h1>

`hypervoxel` owns exact-aware voxel grid frames, sparse-grid facts, voxelization
reports, and adapter manifests for the Hyper ecosystem. The repository still carries
harvested `voxelis` SVO-DAG storage code, but this crate is the Hyper semantic layer:
grid frames are expressed with `hyperreal::Real`, addresses are integer grid
coordinates, and lossy voxelizers or renderers must report their numeric contract.

The crate is not a production renderer or full voxelization suite yet. It is the place
where voxel evidence, aggregate facts, grid provenance, and adapter status are preserved
before storage, meshing, simulation, or visualization layers consume them.

## Hyper Ecosystem

`hypervoxel` consumes exact scalar, vector, and predicate facts from the core stack.

- [hyperreal](https://github.com/timschmidt/hyperreal): exact grid-frame origin, scale,
  spacing, and policy values.
- [hyperlattice](https://github.com/timschmidt/hyperlattice): vector and transform
  values for frame, AABB, affine, and field reports.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): exact classification policy
  for boxes, half-spaces, and future solid predicates.
- [hypermesh](https://github.com/timschmidt/hypermesh) and
  [hyperphysics](https://github.com/timschmidt/hyperphysics): mesh, mass, collision,
  support, field, and simulation handoff consumers.
- [hyperpath](https://github.com/timschmidt/hyperpath) and
  [hyperdrc](https://github.com/timschmidt/hyperdrc): process, routing, and
  manufacturing contexts that can use sparse-grid evidence.

## Typical Voxel Problems

Voxel systems often collapse many decisions into one sampled grid: coordinate frame,
quantization, boundary policy, material labels, LOD aggregation, compression, meshing,
and rendering. Once that happens, it is hard to tell whether a cell is truly occupied,
conservatively covered, unknown, or merely a display approximation. Compression and
meshing can also erase provenance unless their replay status is recorded.

`hypervoxel` treats a grid as evidence rather than just pixels in 3D. It keeps exact
frames and integer addresses, stores conservative aggregate facts, distinguishes
occupied/empty/mixed/unknown states, and records adapter, quantization, compression,
export, and handoff reports. Readiness flags are intentionally non-vacuous: empty
batches, empty snapshots, empty sample declarations, collapsed empty SVO roots, and
zero-byte memory routes are absence reports rather than exact evidence.

## Main Types

- `GridFrame`, `GridAxis`, `GridBasis`, `GridFrameFacts`, and frame manifests describe
  exact grid coordinate systems.
- `VoxelAddress`, `CellBounds`, `VoxelCell`, `VoxelPayload`, `SparseVoxelGrid`, and
  edit batches describe sparse voxel data.
- `VoxelAggregateFacts`, `VoxelSpatialAggregateFacts`, `LodSelectionReport`, and
  prepared-query reports preserve conservative grid summaries.
- `ExactBox`, `ExactHalfSpace`, `ExactConvexHalfSpaceSet`, and classifier reports
  provide the current exact voxelization surfaces.
- `VoxelSideTables`, material/field/process records, and query reports connect sparse
  cells to domain data.
- Mesh, support, path trace, compression, IO, artifact, coupling, export, and handoff
  report types preserve adapter status.

## Precision Model

Grid frames use `Real` values; voxel addresses are integer grid coordinates. Exact box,
half-space, and convex-half-space classification use exact cell bounds and return
inside/outside/boundary or mixed states as appropriate. Source preflight rejects
inverted or zero-extent boxes, zero or structurally unknown half-space normals, and
empty convex predicate sets before topology can be promoted. Quantization, preview
export, legacy storage, and lossy mesh output are report-bearing adapter surfaces, not
silent replacements for exact occupancy evidence.

## Performance Model

`hypervoxel` keeps dense data out of the semantic layer where possible. Sparse grids,
chunk summaries, aggregate facts, LOD selection, deterministic snapshots, side tables,
and adapter manifests let callers reason about large grids without materializing every
downstream representation. Exposed-face extraction and greedy face patch planning are
kept as explicit lossy/export steps.

The harvested `voxelis` SVO-DAG code remains available behind `legacy-voxelis` for
storage experiments. The feature exposes sampled `VoxTree<u8>` storage diffs where
default/nonzero semantics match Hyper cells, but the adapter remains lossy provenance
under `LegacyAdapterKind::VoxelisStorage` and cannot stand in for exact voxelization.

## Current Status

Implemented today:

- exact grid frames, source units, manifests, and frame facts;
- voxel addresses, sparse grids, edit batches, chunk summaries, side tables, and
  deterministic snapshots;
- occupancy, material, field, process, aggregate, LOD, neighbor, connected-component,
  and prepared-query reports;
- exact box, half-space, and convex-half-space-set voxelization/classification;
- AABB, affine, axis-permutation, support-mask, ray/path trace, distance-field preview,
  sparse-grid diff, mesh export, compression, memory-budget, IO, artifact, coupling, and
  handoff reports, including non-vacuous sample, memory, query, handoff, artifact, and
  process provenance gates.
- feature-gated sampled legacy `voxelis` storage differential reports that keep storage
  agreement separate from exact source-geometry replay.

Known limits: full production voxelizers, out-of-core pipelines, GPU renderers, and
complete mesh/field solver bridges remain adapter work.

## Installation

```toml
[dependencies]
hypervoxel = "0.2.0"
```

For sibling checkouts:

```toml
[dependencies]
hypervoxel = { path = "../hypervoxel" }
```

Feature summary:

- `legacy-voxelis`: enables the harvested `voxelis` integration.

## Usage

Start with an exact frame, then classify or store cells with explicit reports:

```rust,ignore
use hypervoxel::{
    ExactBox, GridFrameBuilder, LengthUnit, SparseVoxelGrid, VoxelAddress, VoxelCell,
    voxelize_exact_box,
};
use hyperreal::Real;

let frame = GridFrameBuilder::default()
    .unit(LengthUnit::Millimeter)
    .origin([Real::from(0), Real::from(0), Real::from(0)])
    .pitch(Real::from(1))
    .build()?;

let solid = ExactBox::new(
    [Real::from(0), Real::from(0), Real::from(0)],
    [Real::from(2), Real::from(2), Real::from(2)],
    None,
);

let report = voxelize_exact_box(&frame, &solid, 2)?;
let mut grid = SparseVoxelGrid::new(frame);
grid.insert(VoxelAddress::new(1, [0, 0, 0])?, VoxelCell::occupied());
```

Half-space and convex-set voxelizers, sparse-grid diffs, deterministic snapshots,
distance previews, exposed-face extraction, greedy patch planning, compression reports,
IO manifests, support masks, and legacy `voxelis` storage diffs follow the same rule:
they are report-bearing handoffs, not silent replacements for exact grid evidence.

## Development

Useful local checks:

```sh
cargo test
cargo bench --bench grid_frame
```

The legacy workspace members retain their own crate metadata and documentation for
`voxelis` internals.
