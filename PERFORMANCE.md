# Reference-Guided Performance Notes

These measurements retain only changes that preserve exact rows, output size,
and benchmark checksums. They were collected on 15 July 2026 with the release
benchmark profile. Times are medians after two warmup batches.

| Workload | Before | After | Change | Preserved evidence |
|---|---:|---:|---:|---|
| `mesh_queries/graphics_buffers` | 1.170 ms/op | 0.139 ms/op | 88.1% faster | 1,584 indices and exact position/normal regression |
| `mesh_io/all_exporters` | 87.602 ms | 81.581 ms | 6.9% faster | 385,166 output bytes and checksum |
| `mesh_boolean/prepare_and_extract_four` | 31.542 ms | 9.606 ms | 69.5% faster | 120 polygons, 120 corners, exact buffers and metadata |
| `mesh_boolean/extract_four_prebuilt` | 31.542 ms | 2.174 ms | 93.1% faster | Same four closure-certified outputs |
| `mesh_queries/ray_intersections` | 32.851 ms/op | 30.819 ms/op | 6.2% faster | Same exact ordered hit points, distances, and checksum |

## Retained changes

`Mesh::build_graphics_mesh` previously called `Mesh::triangulate`, which built
temporary triangle polygons, recomputed support planes, allocated metadata and
IDs, and then discarded that intermediate mesh. The renderer boundary now
streams the same certified triangle indices directly from each source polygon
into exact `Real` position/normal rows. A regression test compares every row and
the sequential index buffer with the former materialized path.

`Mesh::indexed_triangles` now reserves position, normal, face, and lookup
storage from source topology bounds before exporters populate it. Deduplication
still uses retained position and plane identities; output order and serialized
bytes are unchanged.

`Mesh::try_prepare_boolean` now adapts and subdivides one mesh pair into a
borrowed `PreparedMeshBoolean` once, then extracts union, difference,
intersection, and symmetric difference from retained exact winding evidence.
Each extraction still closure-certifies the HyperMesh result and restores
source polygon metadata. Direct methods now use the same pipeline with a
single-operation scope, preserving operation-specific winding-reachability
pruning without retaining a separate one-shot implementation. Empty,
adapter-empty, exactly disjoint, and identical inputs are prepared as exact
set-identity states, preserving the former direct polygonization and metadata.

Nine-sample release medians characterize the unified path:

| Workload | Scoped direct | Prepare + extract | Prebuilt extract |
|---|---:|---:|---:|
| overlapping, one operation | 7.90 ms | 7.94 ms | 0.59 ms |
| overlapping, two operations | 15.60 ms | 8.35 ms | 0.98 ms |
| overlapping, four operations | 31.55 ms | 9.61 ms | 2.18 ms |
| disjoint, four operations | 20.9 us | 20.2 us | 20.0 us |
| identical, four operations | 154.1 us | 44.6 us | 8.2 us |
| contained, four operations | 26.29 ms | 7.50 ms | 1.16 ms |
| face-touching, four operations | 24.53 ms | 6.80 ms | 0.76 ms |

Every row retains the same polygon/corner checksum. Focused differential tests
also compare exact vertex rows, source metadata, and shortcut polygonization.

`Mesh::ray_intersections` now streams triangle vertex references from each
polygon's certified triangulation indices instead of cloning a temporary
`[Vertex; 3]` for every ray test. The HyperLimit ray/triangle predicate, exact
hit ordering, and exact deduplication are unchanged. A differential test replays
the former materialized enumeration against the streaming result.

## End-to-end dispatch evidence

Dispatch recording now occurs inside each selected benchmark closure, so setup
for unselected feature rows cannot contaminate a trace. On the overlapping-cube
four-operation sentinel, ordinary calls recorded 1,012,924 dispatch events,
56,224 point classifications, four HyperMesh preparations, and four
subdivisions. Build-once/extract-four recorded 354,164 events, 14,056 point
classifications, one preparation, and one subdivision. Both traces recorded
four output-closure certifications, zero approximation events, zero refinement
events, and zero unknown-fact events.

Replay the focused measurements with:

```sh
CSGRS_BENCH_FILTER=mesh_queries/graphics_buffers \
CSGRS_BENCH_SAMPLES=9 \
CSGRS_BENCH_WARMUP=2 \
cargo bench --bench feature_pipeline --features offset,bevymesh

CSGRS_BENCH_FILTER=mesh_io/all_exporters \
CSGRS_BENCH_SAMPLES=7 \
CSGRS_BENCH_WARMUP=2 \
cargo bench --bench feature_pipeline --features offset,bevymesh

CSGRS_BENCH_FILTER=mesh_boolean \
CSGRS_BENCH_SAMPLES=9 \
CSGRS_BENCH_WARMUP=2 \
cargo bench --bench feature_pipeline --features offset,bevymesh

CSGRS_BENCH_FILTER=mesh_queries/ray_intersections \
CSGRS_BENCH_SAMPLES=9 \
CSGRS_BENCH_WARMUP=2 \
cargo bench --bench feature_pipeline --features offset,bevymesh
```

## Audited non-adoptions

The reference section's cubic Bezier GPU distance algorithm is a finite
nearest-distance technique. It is not substituted for exact/adaptive topology
predicates, and `csgrs` has no current public curve-distance API that would
justify adding a rendering-specific approximation. Likewise, the CAD/CAM
hyperbook's interval, subdivision, and offset methods are not claimed as
certificates where the current implementation explicitly crosses a finite
adapter boundary.

Taubin's two-pass smoothing filter and the uniform Laplacian formulation in
Botsch et al. keep topology and per-valence weights fixed across iterations. An
experiment therefore retained the lambda/mu stencils and alternated two reusable
position buffers instead of rebuilding scale divisions and allocating each
pass. On `mesh_refinement/subdivide_and_smooth`, nine-sample medians were
113.870 ms for the existing implementation and 113.933 ms for the experiment;
the identical 276,830,400 checksum confirmed output preservation, but the timing
was indistinguishable and slightly worse. The production change was removed.
The existing end-to-end benchmark remains the regression sentinel; exact `Real`
neighbor accumulation, not vector allocation or valence-scale division, dominates
this workload.

The isolated feature traces also keep the remaining large rows visible without
weakening their contracts. Symbolic profile offsets recorded 24,005 refinement
events because many trigonometric segment bounds cannot be decided structurally;
TPMS sampling recorded 22,784 refinements before its documented Surface Nets
preview boundary; smoothing recorded 68,074 rational reductions and 102,010
GCDs. Replacing these with primitive-float topology decisions, skipping offset
self-contact validation, or changing the smoothing stencil would change quality
or exactness rather than optimize the same result, so those substitutions were
not attempted.
