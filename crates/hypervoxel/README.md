<h1>
  Hypervoxel
  <img src="./docs/voxelis_logo.png" alt="Hypervoxel and Voxelis voxel mark" width="144" align="right">
</h1>

Exact-aware voxel frames, integer addresses, conservative occupancy facts, and
sparse-grid handoffs for the Hyper geometry stack.

Hypervoxel separates geometric meaning from storage layout. It owns exact grid
frames, integer cell addresses, occupancy and payload semantics, primitive and
triangle classifiers, topology reports, and explicit boundaries around
compression, paging, meshing, simulation, preview, and legacy adapters.

The repository also contains the harvested
[Voxelis](https://github.com/WildPixelGames/voxelis) SVO-DAG workspace.
Voxelis supplies storage and rendering foundations; the root `hypervoxel`
package supplies the Hyper exactness contract. See
[Repository layout](#repository-layout) when selecting a Cargo package.

This README describes `hypervoxel` version `0.3.0`.

## Primary types

| Type | Role |
| --- | --- |
| `GridFrame`, `GridFrameFacts`, `LengthUnit` | Exact origin, pitch, depth, unit, and arithmetic schedule |
| `VoxelAddress`, `CellBounds` | Integer octree address and exact model-space bounds |
| `VoxelCell`, `VoxelPayload`, `OccupancyState` | Cell fact and optional material/field/process payload |
| `SparseVoxelGrid`, `VoxelEditBatch` | Canonical sparse editing and replay layer |
| `VoxelAggregateFacts`, `VoxelSpatialAggregateFacts` | Conservative summaries over cells and hierarchy |
| `ExactBox`, `ExactHalfSpace`, `ExactConvexHalfSpaceSet` | Exact analytic voxelization inputs |
| `ExactTriangleSurfaceMesh`, `ExactTriangleSolidMesh`, `ExactTriangleSolid` | Exact triangle carriers with increasing topology guarantees |
| `ChunkPagedSparseGrid`, `SvoVoxelGrid` | Replayable compressed storage |
| `VoxelizationReport`, `VoxelPredicateCertificateReport` | Classification and exact-readiness evidence |
| `VoxelSideTables` | Material, field-sample, and process-state domain records |

## Install

```toml
[dependencies]
hypervoxel = "0.3.0"
```

The root workspace defaults to the harvested Voxelis members. In a checkout,
select the Hypervoxel package explicitly with `-p hypervoxel`.

## Quick start

This checked example creates an exact millimetre grid, voxelizes an exact box,
and verifies the report before using its topology.

<!-- quickstart:start -->
```rust
use hyperreal::Real;
use hypervoxel::{
    ExactBox, GridFrame, HypervoxelResult, LengthUnit, MaterialRegionId, VoxelizationPolicy,
    voxelize_exact_box,
};

fn main() -> HypervoxelResult<()> {
    let frame = GridFrame::new(
        [0.into(), 0.into(), 0.into()],
        [1.into(), 1.into(), 1.into()],
        3,
        LengthUnit::Millimeter,
    )?;
    let solid = ExactBox::new(
        [Real::from(1), Real::from(1), Real::from(1)],
        [Real::from(3), Real::from(3), Real::from(3)],
    );

    let (grid, report) = voxelize_exact_box(
        frame,
        &solid,
        MaterialRegionId(7),
        VoxelizationPolicy::conservative_cover(),
    )?;

    assert_eq!(grid.len(), 8);
    assert!(report.exact_topology_ready());
    println!("stored {} exact cells", grid.len());
    Ok(())
}
```
<!-- quickstart:end -->

Run it with:

```sh
cargo run -p hypervoxel --example exact_box
```

The companion
[`examples/sparse_grid.rs`](examples/sparse_grid.rs) demonstrates direct sparse
edits and exact address bounds.

## Model and evidence

```text
GridFrame + VoxelAddress
           │
       VoxelCell ── VoxelSideTables
           │
     SparseVoxelGrid
           │
   ┌───────┼──────────────────┐
queries  exact surface    replayable storage
          handoff         chunk pages / SVO-DAG
```

Frames use `hyperreal::Real`; addresses use exact integers. Occupancy
distinguishes filled, empty, boundary, mixed, unknown, and lossy-adapter states.
A payload ID is interpreted only through its side-table record.

Most operations return data plus a report. Check readiness methods such as
`VoxelizationReport::exact_topology_ready`; neither a nonempty grid nor a
successful storage conversion proves exact topology by itself.

## API guide

### Frames, addresses, cells, and edits

- `GridFrame::{new, unit, origin, pitches, depth, units, cells_per_axis,
  facts}` constructs and inspects the exact coordinate frame.
  `is_exact_rational_frame`, `has_dyadic_schedule`,
  `has_shared_denominator_schedule`, and `has_integer_grid_schedule` expose
  useful arithmetic routes.
- `VoxelAddress::{new, root, child, parent, morton_code, from_morton_code,
  child_path, from_child_path, bounds}` provides checked integer addressing.
- `CellBounds::{extent, center, corners}` produces exact model-space geometry.
- `VoxelCell::{empty, material, field_sample, process_state, boundary,
  unknown, lossy_adapter_value, report}` makes occupancy and provenance
  explicit.
- `SparseVoxelGrid::{new, frame, get, set, stored_aggregate, iter, len,
  is_empty}` is the main sparse carrier.
- `VoxelEditBatch::{new, push, iter, apply_to}` retains an ordered group of
  edits. `diff_sparse_grids` compares canonical sparse states.
- `VoxelSideTables`, `query_material_regions`, `lookup_material_display_colors`,
  and `query_field_samples` manage domain records without embedding them in
  geometry.

### Exact voxelization

- `voxelize_exact_box`, `voxelize_exact_halfspace`, and
  `voxelize_exact_convex_halfspace_set` classify analytic inputs.
- `classify_cell_against_halfspace` and
  `classify_cell_against_convex_halfspace_set` expose the per-cell forms.
- `ExactTriangle3`, `ExactTriangleSurfaceMesh`, and
  `ExactTriangleSolidMesh` retain triangle input and validation reports.
- `voxelize_exact_triangle_surface_mesh` and
  `classify_cell_against_triangle_surface_mesh` retain conservative
  surface-contact evidence.
- `ExactTriangleSolid::new` validates a closed triangle solid.
  `classify_cell_against_exact_triangle_solid` and
  `voxelize_exact_triangle_solid` provide the general route.
- Component, local-consensus, axis-sweep, consensus-axis-sweep, and adaptive
  voxelizers are exposed by the
  `voxelize_exact_triangle_solid_by_*` functions. Their reports identify the
  schedule, predicate counts, accelerations, and fallbacks used.

With `hypermesh-adapter`, `adapt_hypermesh_exact_solid` validates and imports a
retained Hypermesh solid. It is a conversion boundary, not shared ownership of
mesh topology.

### Queries and exact surface output

- `voxel_neighbors6`, `select_lod_cells`, `classify_support_mask`,
  `trace_address_segment`, `trace_address_ray`, and `sweep_address_segment`
  cover neighborhood, LOD, support, and integer-address traversal.
- Region aggregate, connected-component, broad-phase AABB, and Manhattan-band
  queries are available on `ChunkPagedSparseGrid`.
- `sample_manhattan_distance_field` and
  `sample_signed_manhattan_distance_field` return explicitly named preview
  distance samples.
- `extract_exposed_faces`, `greedy_face_patches`, and
  `exact_voxel_surface_triangle_mesh_from_faces` build lattice-aligned surface
  evidence.
- `sparse_exact_surface_triangle_mesh`,
  `chunk_paged_exact_surface_triangle_mesh`, and
  `svo_exact_surface_triangle_mesh` replay the corresponding storage carrier
  into an exact indexed triangle surface.
- `lossy_quad_mesh_from_faces` and `lossy_obj_from_quad_mesh` are explicit
  preview/export adapters.

### Storage, replay, and serialization

- `ChunkPagedSparseGrid::{from_sparse_grid, report, get, page, pages, iter}`
  retains page-level replay evidence. `diff_chunk_paged_sparse_grids` compares
  two paged states.
- `SvoVoxelGrid::{new, from_sparse_grid, to_sparse_grid, get, set, aggregate,
  stats}` provides bottom-up SVO-DAG storage with sparse replay.
- `DeterministicSnapshot::{text_v1, binary_v1, run_length_binary_v1}` emits
  stable snapshots with an explicit `SnapshotFormat`.
- `VoxelAggregateFacts::from_cells` and
  `VoxelSpatialAggregateFacts::from_grid` summarize storage conservatively;
  parent facts are not averaged material values.

### Adapter boundaries

- `ContinuousFieldVoxelBatch` is the intake form for exact-aware implicit-field
  classifications such as those produced by Hypersdf.
- `ExactAffineTransform`, `AxisPermutationTransform`, and `SignedAxis` retain
  supported grid-space transforms.
- With `legacy-voxelis`,
  `materialize_legacy_voxelis_u8_chunk_paged_storage` and
  `materialize_legacy_voxelis_u8_exact_surface_triangle_mesh` expose sampled
  interoperability. Legacy samples remain marked adapter data and are not
  promoted to source geometry.

## Guarantees and boundaries

- Grid coordinates and cell bounds are exact `Real` values; cell identities
  are integer addresses.
- Empty input is not treated as vacuous proof. Reports distinguish no evidence
  from certified empty geometry.
- Quantization and boundary policy are explicit in `VoxelizationPolicy`.
- Candidate acceleration may reduce work only when its report preserves the
  same exact cell decision.
- Compression, paging, greedy meshing, snapshots, and DAG interning must replay
  retained facts before an exact-readiness flag becomes true.
- Preview meshes, primitive-float exports, renderers, and legacy imports do not
  define source topology.

General implicit-field voxelization is supplied through the classified
continuous-field intake rather than duplicated here. Production out-of-core
execution and GPU rendering remain storage/application concerns, not exact
geometry guarantees.

## Feature flags

| Feature | Default | Purpose |
| --- | --- | --- |
| `dispatch-trace` | no | Forward Hyperreal exact-dispatch instrumentation |
| `hypermesh-adapter` | no | Validate/import Hypermesh exact solids |
| `legacy-voxelis` | no | Materialize sampled data in harvested Voxelis storage |

## Repository layout

The root package is `hypervoxel`. The workspace also contains the upstream
Voxelis packages `voxelis`, `voxelis-math`, `voxelis-memory`,
`voxelis-voxelize`, `voxelis-bevy`, and the `vtm-*` tools. Their README files
describe those packages and retain upstream licensing/provenance. Hypervoxel’s
API does not require a rendering package.

## Validation and performance

```sh
cargo fmt --all -- --check
cargo test -p hypervoxel --all-features --locked
cargo clippy -p hypervoxel --all-targets --all-features --locked -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc -p hypervoxel --all-features --no-deps --locked
cargo check -p hypervoxel --benches --all-features
```

Benchmark definitions are in [docs/benches.md](docs/benches.md); the root
crate’s reference audit and measured optimization record are in
[PERFORMANCE.md](PERFORMANCE.md).

## References

These sources describe algorithms or evidence boundaries used by the root
crate. Listing a source does not imply every algorithm in it is implemented.

- Yap, C. K. “Towards Exact Geometric Computation.” *Computational Geometry*
  7(1–2), 1997, 3–23.
  [DOI: 10.1016/0925-7721(95)00040-2](https://doi.org/10.1016/0925-7721(95)00040-2).
- Moore, R. E., Kearfott, R. B., and Cloud, M. J. *Introduction to Interval
  Analysis*. SIAM, 2009.
  [DOI: 10.1137/1.9780898717716](https://doi.org/10.1137/1.9780898717716).
- Rosenfeld, A., and Pfaltz, J. L. “Sequential Operations in Digital Picture
  Processing.” *JACM* 13(4), 1966.
  [DOI: 10.1145/321356.321357](https://doi.org/10.1145/321356.321357).
- Rosenfeld, A., and Pfaltz, J. L. “Distance Functions on Digital Pictures.”
  *Pattern Recognition* 1(1), 1968.
  [DOI: 10.1016/0031-3203(68)90013-7](https://doi.org/10.1016/0031-3203(68)90013-7).
- Bresenham, J. E. “Algorithm for Computer Control of a Digital Plotter.”
  *IBM Systems Journal* 4(1), 1965.
  [DOI: 10.1147/sj.41.0025](https://doi.org/10.1147/sj.41.0025).
- Amanatides, J., and Woo, A. “A Fast Voxel Traversal Algorithm for Ray
  Tracing.” *Eurographics ’87*.
  [DOI: 10.2312/egtp.19871000](https://doi.org/10.2312/egtp.19871000).
- Kay, T. L., and Kajiya, J. T. “Ray Tracing Complex Scenes.”
  *SIGGRAPH Computer Graphics* 20(4), 1986.
  [DOI: 10.1145/15886.15916](https://doi.org/10.1145/15886.15916).
- Möller, T. “A Fast Triangle-Triangle Intersection Test.”
  *Journal of Graphics Tools* 2(2), 1997.
  [DOI: 10.1080/10867651.1997.10487472](https://doi.org/10.1080/10867651.1997.10487472).
- Möller, T., and Trumbore, B. “Fast, Minimum Storage Ray-Triangle
  Intersection.” *Journal of Graphics Tools* 2(1), 1997.
  [DOI: 10.1080/10867651.1997.10487468](https://doi.org/10.1080/10867651.1997.10487468).
- Guigue, P., and Devillers, O. “Fast and Robust Triangle-Triangle Overlap Test
  Using Orientation Predicates.” *Journal of Graphics Tools* 8(1), 2003.
  [DOI: 10.1080/10867651.2003.10487580](https://doi.org/10.1080/10867651.2003.10487580).
- Lorensen, W. E., and Cline, H. E. “Marching Cubes.”
  *SIGGRAPH Computer Graphics* 21(4), 1987.
  [DOI: 10.1145/37402.37422](https://doi.org/10.1145/37402.37422).
- Kämpe, V., Sintorn, E., and Assarsson, U. “High Resolution Sparse Voxel
  DAGs.” *ACM Transactions on Graphics* 32(4), 2013.
  [DOI: 10.1145/2461912.2462024](https://doi.org/10.1145/2461912.2462024).
- Botsch, M., Kobbelt, L., Pauly, M., Alliez, P., and Lévy, B.
  *Polygon Mesh Processing*. A K Peters/CRC Press, 2010.
  [DOI: 10.1201/b10688](https://doi.org/10.1201/b10688).
- Arvo, J. “Transforming Axis-Aligned Bounding Boxes.” In *Graphics Gems*,
  Academic Press, 1990. [Reference implementation](https://www.realtimerendering.com/resources/GraphicsGems/gems/TransBox.c).
- ISO. *ISO 10303-242:2025, Industrial automation systems and integration —
  Product data representation and exchange — Part 242*.
  [ISO catalogue](https://www.iso.org/standard/84300.html).

## Acknowledgements and provenance

Hypervoxel builds on
[Hyperreal](https://github.com/timschmidt/hyperreal),
[Hyperlattice](https://github.com/timschmidt/hyperlattice), and
[Hyperlimit](https://github.com/timschmidt/hyperlimit), with optional
[Hypermesh](https://github.com/timschmidt/hypermesh) integration.

The bundled Voxelis workspace was created by Artur Wyszyński and contributors
and is retained under its upstream MIT/Apache-2.0 terms. Hypervoxel-specific
work is by Timothy Schmidt, with repository contributions also credited in the
Git history. The upstream project and its authorship must remain identified
when redistributing harvested packages.

## License and contributing

The root Hypervoxel crate is licensed under the
[Apache License 2.0](LICENSE-APACHE). Bundled Voxelis packages retain their
declared MIT/Apache-2.0 licensing; see their manifests and repository license
files.

Bug reports should include the exact frame, addresses or input geometry,
voxelization policy, enabled features, and complete report. Before proposing a
root-crate change, run formatting, the focused regression, the all-feature
suite, and strict Clippy with `-p hypervoxel`.
