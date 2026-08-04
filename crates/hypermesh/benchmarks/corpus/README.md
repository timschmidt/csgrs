# Hypermesh Boolean fixture corpus

`fixtures.toml` is the authoritative registry for correctness, competitive,
scaling, fuzz-seed, and heap fixtures used during the surface-arrangement
replacement. Every record has a stable ID, provenance, coordinate and topology
tags, policy expectations, size tier, operation set, and CGAL-common-contract
status.

The registry is monotonic. A failing or pathological input may be minimized or
replaced by a deterministic generator with the same semantic path, but it is
not removed to make a test or benchmark pass. Large external assets remain
content-addressed and opt-in; their URLs, byte lengths, hashes, and derivation
are versioned rather than embedding them in test binaries.

Current tiers are intentionally explicit:

- `micro`: fast exact feature/contact and Boolean truth cases;
- `regression`: a historical bug or behavior contract;
- `competitive`: an input in the shared CGAL EPECK contract;
- `scaling`: a deterministic family with multiple complexity points;
- `heap`: a large or XL process/allocator measurement; and
- `fuzz-seed`: a permanent seed for mutation/reduction campaigns.

`tests/corpus_manifest.rs` rejects duplicate/incomplete records, missing exact
assets, competitive cases absent from the Rust benchmark corpus, and heap rows
without a large size tier. `implementation-test-migration.toml` records the
committed historical source snapshot and maps the removed engine-specific test
families to current public behavior cases or stronger arrangement invariants.

`tests/intersection_corpus.rs` permanently exercises every public pairwise
intersection class across both predicate policies, both operand orders, and
both polygon orientations. Its exact rectangle oracle distinguishes disjoint,
point, segment, and positive-area coplanar intersections, and the
`polygon_predicates` fuzz target mutates the same dimensional contract while
checking geometry, order/orientation invariance, policy certainty, and payload
indices.

The public Boolean matrix separately retains edge-touching, vertex-touching,
and face-tangent containment cases. Their inputs are ordinary closed
manifolds, while some exact closed-PWN results are intentionally non-manifold
at the lower-dimensional contact. They remain mandatory Hypermesh cases but
are not mislabeled as shared boolmesh/Manifold/CGAL output-contract rows.

`clipped_voxel_torus_9`, `clipped_voxel_torus_33`, and
`clipped_voxel_torus_65` form one deterministic indexed high-genus family.
They use the same exact symmetry-plane clipping operation at 460, 6,412, and
25,100 input triangles, respectively. The medium point runs in the shared
correctness and competitive suite; the large and XL points have unique direct
kernel/process heap selectors under both policies.

`dense_coplanar_boxes_4`, `dense_coplanar_boxes_16`, and
`dense_coplanar_boxes_32` are geometrically identical box pairs whose two
surface grids use opposite face diagonals. Every face therefore enters the
cross-operand coplanar overlay instead of relying on coincident triangulation.
The family has 384, 6,144, and 24,576 input triangles while every authored
input coordinate stays on the same denominator-at-most-eight dyadic lattice,
separating mesh growth from input scalar-storage growth. The large and XL
siblings have distinct both-policy kernel/process heap selectors.

`large_mesh_heap_probe` exposes both `boxes-3072` and
`boxes-3072-general`. They contain the same 6,144 exact input triangles; the
first primes the native policy-qualified closed-PWN fact, while the second uses
raw borrowed views so repeated-input cache effects are visible without
selecting a different engine. Every heap gate runs both
`STRICT` and `APPROXIMATE_512` and records output certainty and topology with
the process-memory result.

`large_mesh_kernel_heap_probe` accepts every manifested `heap_probe_modes`
selector and wraps the system allocator only in that measurement executable.
It records exact requested-payload bytes retained by prepared inputs, the peak
while `boolean` runs, the incremental kernel peak above those live inputs,
allocation/reallocation churn, post-Boolean retention, output-live payload,
input-attached retained-fact growth after output drop, and the residual after
input drop. Authoritative rows are one-thread runs so interval snapshots have a
single allocation schedule.
The ordinary `large_mesh_heap_probe` remains allocator-uninstrumented for
Heaptrack, Massif, RSS, and hardware-counter runs. The probes select fixtures,
input ownership, policy, and requested operation only; neither can select a
Boolean implementation or bypass a production path.
