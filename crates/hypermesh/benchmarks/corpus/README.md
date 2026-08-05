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

`crossing_octahedra` preserves the historical no-contained-source-vertex
regression: the two convex boundaries cross even though vertex containment
cannot discover their intersection. `affine_boxes` applies a determinant-eight
integer shear/scale to the ordinary overlap case, retaining exact volume and
topology oracles on non-axis-aligned support planes. Both are named executable
fixtures in the shared competitive corpus rather than placeholders for an old
implementation test family.

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

`sparse_multishell_tetrahedra_8`, `sparse_multishell_tetrahedra_64`, and
`sparse_multishell_tetrahedra_512` place independently overlapping
tetrahedron pairs in a 2x2x2, 4x4x4, or 8x8x8 integer grid. Each operand is one
mesh with many disconnected closed shells. Corresponding shells retain the
same exact local Boolean topology, while distinct grid cells are separated by
an exact gap. The family therefore scales component/cell assembly and sparse
broad-phase work from 64 through 4,096 input triangles without coplanar
overlap or arbitrary-rational width growth. The largest member has a unique
both-policy kernel/process heap selector and every member is exact-CGAL
exportable.

`transverse_self_pwn_clusters_8`, `transverse_self_pwn_clusters_64`, and
`transverse_self_pwn_clusters_512` scale same-operand corefinement. Each
separated cluster contains two transversely crossing tetrahedral shells in one
PWN, so winding multiplicity two and internal arrangement cells are exercised
without changing local topology or scalar width. A distant tetrahedron keeps
all five bounded Boolean expressions meaningful. The 4,100-triangle member is
a dedicated both-policy heap fixture.

`deep_symbolic_translated_boxes_1`, `_8`, `_32`, and `_128` keep box topology
and rational relative geometry fixed while increasing a shared nested
non-rational translation. The shallow case remains fully certified. At deeper
levels STRICT preserves an unresolved exact predicate as
`PredicateUndecided`, while APPROXIMATE_512 alone may consume its terminal and
must reproduce the same oriented geometry and source provenance with
`Approximate512Consumed` certainty. This family tests Hyperreal retained facts
and policy propagation independently of mesh size.

`wide_rational_boxes_64`, `wide_rational_boxes_512`, and
`wide_rational_boxes_2048` hold the 6,144-triangle overlapping-box topology
fixed while applying the positive exact similarity
`(2^shift + 1) / 2^shift`. Their scale numerators and denominators occupy 65,
513, and 2,049 bits even though every binary64 approximation is one. This
separates arbitrary-rational scalar width from mesh/event growth, crosses the
512-bit policy precision without consuming an approximate terminal, and gives
each point a distinct both-policy kernel/process heap selector.

`thin_dyadic_boxes_64`, `thin_dyadic_boxes_512`, and
`thin_dyadic_boxes_2048` hold that same 6,144-triangle connectivity fixed while
applying the exact affine map `(x, y, z) -> (x + z, y, z / 2^shift)`. Its
determinant is `2^-shift`: exact topology and inverse-embedded geometry remain
constant while parallel supports and their surface triangles become
arbitrarily close. The 2,048-bit member's thin coordinate underflows binary64
to zero, but remains an ordinary exact dyadic `Real`; the family is therefore
an exact near-degenerate/extreme-exponent gate rather than a floating-point
epsilon test. All three points are exact-CGAL exportable and have distinct
both-policy kernel/process heap selectors.

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
