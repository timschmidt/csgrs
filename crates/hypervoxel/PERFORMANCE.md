# HyperVoxel Performance Record

The retained measurements below use Criterion's release profile and the
existing `grid_frame` benchmark. Each before/after pair was collected on the
same host and fixture with 100 samples:

```sh
cargo bench -p hypervoxel --bench grid_frame --all-features -- <benchmark-name>
```

| Benchmark | Baseline estimate | Retained estimate | Change |
| --- | ---: | ---: | ---: |
| `triangle_solid_construction` | 8.313 µs | 7.148 µs | 14.0% faster |
| `triangle_solid_voxelization` | 5.388 ms | 5.298 ms | 1.7% faster |
| `region_aggregate` | 663.3 ns | 537.0 ns | 19.0% faster |
| `manhattan_distance_preview_32_cube_512_sources` | 5.170 ms | 0.908 ms | 82.4% faster |
| `semantic_connected_component` | 227.1 µs | 95.8 µs | 57.8% faster |
| `address_ray_trace` | 216.7 ns | 166.3 ns | 23.3% faster |
| `exact_voxel_surface_triangle_mesh_handoff` | 67.1 µs | 53.7 µs | 20.0% faster |
| `chunk_paged_connected_component` | 7.420 µs | 6.562 µs | 11.6% faster |
| `exact_exposed_face_extraction_report` | 467.4 µs | 432.6 µs | 7.4% faster |
| `grid_frame_construction` | 258.52 ns | 220.67 ns | 14.6% faster |
| `exact_cell_bounds` | 1.0300 µs | 1.0316 µs | 0.2% slower (noise) |
| `exact_box_voxelization` | 149.20 ms | 142.67 ms | 4.4% faster |
| `greedy_face_patches` | 65.493 µs | 62.988 µs | 3.8% faster |

## Retained Changes

`ExactTriangleSolid` now validates and retains exact scheduling facts during
ordinary construction, so callers cannot receive a successful but unusable
intermediate object. Its facet table indexes retained source triangles instead
of cloning their exact coordinates, and construction shares each triangle's
predicate points with its readiness audit. HyperBrep's downstream exact
geometry and materialization sentinels measured 247.8 µs and 376.4 µs after
the migration.

Sparse-grid queries are now inherent operations on `SparseVoxelGrid`; they no
longer require a wrapper that duplicates the grid frame and aggregate. Region
aggregation also streams matching cells directly into the fact accumulator
instead of first collecting a temporary vector. The retained benchmark uses
the same populated depth-eight grid and query region as the former wrapper.

The exact Manhattan transform already used six linear relaxations. Result
assembly nevertheless inserted every sample into a `BTreeMap` and then queried
the sparse grid again to decide whether each sample was occupied. Samples are
now emitted directly in `VoxelAddress` order, and exact occupancy comes from
the transform invariant `distance == 0`. A regression test compares that fact
with sparse storage and checks strict output ordering.

Sparse and chunk-paged component traversals retain ordered maps for public
results but use `FxHashSet` for membership-only visited state. Sparse traversal
also accumulates exact-readiness while cells are first read instead of replaying
every reached address after BFS. These changes preserve Rosenfeld–Pfaltz
six-neighbor semantics and deterministic report order.

Address rays reserve the exact smaller of the step-limit and boundary-limited
visit counts. Start addresses are now validated before address arithmetic, so
the faster path also rejects forged out-of-frame addresses instead of risking
underflow or returning invalid trace evidence.

Exact surface extraction reserves a conservative face count and lazily caches
one rational `CellBounds` value per surface cell. Indexed triangle mesh output
still derives deterministic vertices from the topology audit's ordered set,
but its lookup-only vertex index uses `FxHashMap`. Neither hash table is exposed
as ordering or topology evidence.

Grid frames are now created immediately with `GridFrame::new` or the concise
`GridFrame::unit` constructor. The public builder and its separately wrapped
axis objects were removed; validated pitches are retained directly by the
frame. The construction sentinel improved while exact bounds and box
voxelization preserved or improved their established performance.

Greedy face merging now returns `Vec<GreedyFacePatch>` immediately. The public
`GreedyFacePatchPlan` wrapper and sparse/chunk-paged `*_plan` lifecycle names
were removed: the wrapper retained only the completed patch vector and a
redundant copy of the input face count. The same 2,304-face exact box shell was
measured serially with 100 Criterion samples before and after; its interval
improved from 65.298--65.719 µs to 62.882--63.116 µs.

## Exactness and Scope Checks

`tests/dispatch_trace.rs` exercises a fractional exact box and requires nonzero
exact dispatch/reduction activity with zero approximation and zero unknown-fact
events. Existing generated and antagonistic tests cover prime denominators,
large negative origins, boundary contacts, triangle degeneracy, component
consensus, SVO replay, greedy cover replay, and lossy-adapter blockers.

Several references correctly remain boundary guidance rather than new code.
Marching Cubes is a sampled isosurface method and is not promoted to exact
source topology. Möller and Möller–Trumbore supply useful fast constructions,
but topology-changing acceptance uses the Guigue–Devillers/Yap determinant-sign
discipline. Bentley–Ottmann and Kay–Kajiya motivate candidate acceleration;
page, AABB, and schedule hits still require retained object replay. AP242
motivates provenance and interchange manifests without claiming STEP encoding
conformance.

## Nested Crate Reference Audit

The workspace crates repeat several root references and add two voxel-specific
sources. Each distinct nested reference has the following disposition:

| Source | Referencing crates | Audited disposition |
| --- | --- | --- |
| Akenine-Möller, *Fast 3D Triangle-Box Overlap Testing* | `voxelis-math`, `voxelis-voxelize`, `vtm-voxelize` | Retained in the exact root handoff as the 13-axis separating-axis decomposition over rational predicates. A direct floating-point replacement in the legacy sampler was rejected below. |
| Guigue–Devillers, *Fast and Robust Triangle-Triangle Overlap Test Using Orientation Predicates* | `voxelis-math` | Retained as determinant-sign guidance in the exact root handoff; the legacy math crate remains explicitly tolerance-based and non-certifying. |
| Kämpe–Sintorn–Assarsson, *High Resolution Sparse Voxel DAGs* | `voxelis`, `voxelis-voxelize`, `vtm-voxelize` | Existing bottom-up subtree interning and replay are retained. The root audit keeps compression evidence separate from geometry truth. |
| Laine–Karras, *Efficient Sparse Voxel Octrees* | `voxelis` | Existing sparse octree traversal, child masks, and compact nodes are retained; exact root APIs replay addresses before certifying geometry. |
| Lysenko, *Meshing in a Minecraft Game* | `voxelis`, `voxelis-bevy`, `vtm-export`, `vtm-viewer` | Existing greedy render meshes remain preview output. The root exact-face patches expand and compare the compressed cover against the exact shell. |
| Bevy 0.16 documentation | `voxelis-bevy`, `vtm-viewer` | Adapter and renderer API guidance only; it does not own topology or a computational kernel. |
| Wavefront OBJ format description | `vtm-export` | Interchange syntax guidance only. OBJ output remains a named lossy adapter, not exact source evidence. |

## Rejected Nested Prototype

`voxelis-math/benches/overlap_audit.rs` fixes 2,048 mixed triangle/cube
queries and records both time and hit count. Five sequential release runs of
the existing conservative sampler had a median of **24.492 ms** and **484,000**
hits over 1,000 repetitions. A complete floating-point Akenine-Möller 13-axis
SAT prototype had a median of **84.095 ms** and **274,000** hits: **3.43×
slower**, with a different occupancy result. The prototype was removed. The
legacy function intentionally treats any cube straddling the triangle plane
after its AABB rejection as occupied; changing that documented overfill is a
voxelization-policy migration, not a semantics-preserving optimization. Exact
callers already have the rational SAT path in `src/triangle_mesh.rs`.
