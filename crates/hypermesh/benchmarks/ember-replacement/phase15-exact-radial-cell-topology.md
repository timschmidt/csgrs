# Phase 15 checkpoint: exact radial cells and winding truth

Date: 2026-08-03

Implementation: `c512e7f959c085fe4a68dc652dbe5f33c7236f01`

Seed-scheduling optimization: `a8e4a0e9169883aecc78de16423880891eaf97946`

Single-bound BVH optimization: `a727778eac25435dc6bf299b498497bfaf41484c`

Topology-corpus expansion: `3020c2b1f554101c29a09784e7bc7580994c2049`

Shared compact source hierarchy: `fa86ea482c65354cab834439c19e4c441ff48832`

Status: retained, test-gated Phase 15 core. This checkpoint does not claim a
Phase 15 exit, production ownership, an EMBER cutover, output certification, or
CGAL parity. `surface_arrangement` remains compiled only for tests, so no
second Boolean engine ships.

## Decision

Retain the exact radial topology, volumetric cell assembly, absolute winding
classification, and batched expression evaluator built directly over the
Phase 14 corefinement. The checkpoint replaces the mathematical duties of
EMBER reference propagation and whole-mesh segment tracing in the staged
engine without reusing their recursive machinery or data model.

Canonical geometric triangles are bundled once. Each bundle retains a compact
source-contribution range and one checked transition vector. Exact cyclic
radial order around every undirected edge connects the two sides of each facet
into volumetric cells, including coincident rays and nonmanifold edge valence.
One absolute seed winding is computed for each disconnected cell component;
all remaining windings propagate across reciprocal facet incidence with cycle
validation and checked `i32` arithmetic.

A flat truth DAG evaluates constants, operands, NOT, AND, OR, and XOR for any
number of requested roots. Union, intersection, either difference, symmetric
difference, and arbitrary multi-operand expressions therefore reuse the same
intersection graph, corefinement, cells, and winding table.

One staged orchestrator now builds the exact source BVH once. Pairwise
intersection borrows it, after which ownership is converted into a compact
query hierarchy used by absolute winding seeds. The conversion keeps checked
`u32` face order, 12-byte preorder nodes, six source-face extremum IDs per
node, and exact bounds only for the sparse faces that do not already own them.
It does not rebuild a hierarchy or retain the full node `Real` bounds across
corefinement.

## Exactness, termination, and policy

- Every geometric coordinate remains `hyperreal::Real`. Radial triple products,
  perpendicular dot products, ray/triangle relations, bounds, and winding
  transitions use the operation's `DecisionContext`.
- `STRICT` returns `PredicateUndecided` when a symbolic sign cannot be
  certified. `APPROXIMATE_512` can terminate only through Hyperlimit and its
  certainty is absorbed into the operation result.
- The generic seed directions are the finite family
  `(1,t,t^2)`, up to coordinate permutation. Every facet-parallel or edge-hit
  constraint excludes at most two parameters, so `2 * constraints + 1`
  distinct parameters guarantee an admissible ray when the arrangement is
  valid. There is no precision, pass, depth, or fixture-sized retry cap.
- Three axis rays are tried first as performance schedules. Their order uses
  lossy scene extents only to choose work; every accepted or rejected relation
  is still decided exactly. The complete finite family remains the fallback.
- Exact source-face BVH bounds restrict winding-ray candidates. The ray ends
  beyond the exact scene maximum on its unit coordinate, so it cannot omit a
  positive-parameter surface hit.
- Every compact node extremum is the source primitive ID recorded by the
  original certified bound union. Compaction therefore performs no scalar
  equality or ordering decision. In particular it does not rely on
  `Real::PartialEq`, which is intentionally best-effort, and cannot consume an
  `APPROXIMATE_512` terminal or change aggregate certainty.
- Open radial incidence, malformed point IDs, inconsistent propagation cycles,
  winding dimension mismatch, winding overflow, malformed truth DAGs, and an
  exhausted exact direction family are typed failures. No partial output or
  old-engine retry exists.

## Topology and failure-path corpus

Seventeen Phase 15 cases, together with the eight Phase 14 cases, now cover:

1. one tetrahedron producing exactly two reciprocal cells and windings 0/1;
2. disconnected nested shells receiving global rather than component-local
   absolute windings;
3. coincident shells bundled once with independent operand multiplicity;
4. transverse shells producing all four two-operand winding states and closed
   union/intersection/difference/XOR selections;
5. three nested operands and seven batched expression roots, including
   constants and arbitrary DAG composition;
6. edge-tangent closed shells with radial degree four and no fabricated
   intersection volume;
7. opposite-side shared-face coincidence classified as one interface, with
   both directed differences checked;
8. configurable disconnected-shell scaling under both policies;
9. an open triangle rejected by exact radial topology;
10. malformed incidence and mismatched winding dimensions rejected before
    cell propagation;
11. coincident transition overflow reported without wrapping; and
12. symbolic radial equality proving strict indeterminacy versus
    approximate-512 terminal certainty.
13. a nested negative-winding shell producing an exact cavity and two repeated
    global winding states;
14. a 64-triangle integer voxel ring producing one genus-one inside/outside
    radial component with 32 points and 96 edges;
15. transversely self-intersecting shells in one PWN operand, preserving the
    full `0/1/1/2` winding multiplicity;
16. exact scale-7 reflected embedding, complete face-order reversal, and
    operand permutation preserving cell truth and closed selections; and
17. 40 disjoint operands evaluating union, intersection, and parity roots from
    one 80-cell winding table and one arrangement.

Five corresponding permanent manifest records carry generators, policy and
CGAL eligibility, topology tags, and property oracles. The focused matrix
passes 25 tests. The complete all-feature matrix passes 1,115 unit and 134
integration tests with seven expected manual/benchmark
ignores.

Four compact-hierarchy tests add 690 fixed/exhaustive query comparisons against
the full exact BVH across both policies, including tied extrema and a face with
no retained bounds. They also cover empty input, sparse-bound fallback,
certainty preservation, count mismatch, absent primitive/polygon/node/extrema,
invalid preorder/range, and stale bound storage. Every malformed path returns
a typed error.

## Large-fixture runtime and scaling

The permanent generator was run with 1,536 disconnected tetrahedral shells:
6,144 input facets, 6,144 exact corefined facets, 9,216 radial edges, 3,072
cells, and 1,536 disconnected components. Each process performs the complete
intersection/corefinement/topology/classification/closed-selection pipeline
under both `STRICT` and `APPROXIMATE_512`; both remain `Certified`.

The first seed schedule always used `+X`. On this intentionally aligned family
that ray crossed every later component, creating a quadratic exact scan. The
axis-first schedule removes that accidental correlation:

| Revision | 6,144-facet wall time | Maximum RSS |
| --- | ---: | ---: |
| `c512e7f9`, fixed `+X` first | 7.29 s | 37,908 KiB |
| `a8e4a0e9`, narrow axis first | 0.18 s | 38,420 KiB |

Wall time falls 97.53% (40.5x). RSS is effectively flat at this sampling
resolution; Massif below is authoritative for heap. The 1,024-facet row is
0.03 s after the change versus 0.21 s before it.

Callgrind 3.27.0, pinned to CPU 11, records:

| Facets | Policies | Instructions | Instructions/facet |
| ---: | ---: | ---: | ---: |
| 1,024 | 2 | 271,903,497 | 265,531 |
| 6,144 | 2 | 1,719,503,571 | 279,867 |

Six times as many facets retire 6.324x as many instructions; per-facet cost
rises 5.40%, consistent with the intended hierarchy/sort overhead rather than
the removed quadratic ray scan. In the large row, exact corefinement accounts
for 711,357,006 inclusive instructions (41.37%), cell assembly for 528,294,615
(30.72%), and pairwise discovery for 265,249,016 (15.43%). These are the next
profile-guided ownership targets. The transverse two-shell both-policy case
retires 8,947,999 instructions after scheduling, down from the initial
10,656,974-instruction checkpoint measurement.

The production `ExactBvh` follow-up removes its private second copy of every
primitive's exact bounds and makes construction and leaf queries borrow the
canonical `PolygonBounds` array. On the same 6,144-facet, both-policy topology
row, Callgrind falls from 1,719,503,571 to 1,702,480,107 instructions: a
17,023,464-instruction or 0.9900% reduction. Exact hierarchy and leaf rejection
still use the same `DecisionContext`; approximate centers remain scheduling
hints only.

The shared-hierarchy follow-up reduces the same 6,144-facet row from
1,702,480,107 to 1,542,502,286 instructions, a further 159,977,821
(9.3968%). Its current 1,024-facet row is 247,177,831 instructions, so six
times the input retires 6.2405x the work and per-facet work grows 4.01%.
Recorded extremum provenance removes the former descendant scans; compact
preorder topology adds a smaller 160,375-instruction improvement on top.

## Large-fixture heap

Massif 3.27.0 used `--stacks=yes` on the 6,144-facet row:

| Measurement | Useful heap | Total including allocator/stack |
| --- | ---: | ---: |
| Fixture constructed, before Boolean stages | 6,545,203 | 6,970,224 |
| First topology-heavy detailed snapshot | 25,957,087 | 27,417,712 |
| Whole-process maximum | 28,587,215 | 30,217,232 |

The topology-heavy increment over retained input is 19.41 MB (18.51 MiB)
useful; the worst stage increment is 22.04 MB (21.02 MiB) useful. Maximum RSS
is 38,420 KiB with zero swaps. The whole-process maximum occurs during
corefinement, not truth-table evaluation. Arrangement point/provenance arenas
are the largest owners; the topology stage also rebuilds a source BVH, which
should be retained from the production orchestrator rather than duplicated at
cutover.

The single-bound BVH follow-up reduces the topology-heavy useful snapshot from
25,957,087 to 24,187,795 bytes (1,769,292 bytes, 6.8162%) and its total from
27,417,712 to 25,651,352 bytes (1,766,360 bytes, 6.4424%). The process-wide
useful maximum remains exactly 28,587,215 bytes because earlier corefinement,
not that retained BVH, owns the peak; total peak changes by only -16 bytes.
On the separate 6,144-triangle production general fixture, useful peak remains
exactly 28,730,032 bytes and total changes from 29,956,456 to 29,957,160 bytes,
a 704-byte allocator-metadata variation. This establishes a real stage-local
heap win without moving either governing useful-heap maximum.

The shared orchestrator and unique-source-identity arena capacity move the
topology process-wide useful maximum from 28,587,215 to 21,858,803 bytes
(-6,728,412, 23.5364%) and total peak from 30,217,216 to 23,533,872 bytes
(-22.1177%). The retained-input useful snapshot remains 6,545,203 bytes, so
the arrangement increment is now 15,313,600 bytes. Explicit fallible node
compaction is material: iterator in-place collection had retained the original
671,400-byte full-node allocation even after changing its logical element
type.

The separate shipped 6,144-triangle general path remains certified with
2,410 vertices and 4,816 triangles under both policies. Its useful peak is
28,721,584 bytes, 8,448 below the prior 28,730,032-byte checkpoint; strict and
approximate-512 totals are 29,964,728 and 29,964,712 bytes. Thus the test-gated
orchestrator adds no production heap burden.

## Source, binary size, and call graph

Relative to the Phase 14 evidence commit, the topology checkpoint adds 1,829
and removes three lines in the test-gated module. It therefore adds no linked
production general engine or compatibility surface. The later single-bound
BVH change is production code (28 insertions, 23 deletions), but shrinks every
measured native and optimized-WASM consumer relative to the Phase 14 artifacts:

| Consumer/profile | Native text, before -> after | Optimized WASM, before -> after |
| --- | ---: | ---: |
| General/release | 4,118,732 -> 4,118,236 (-496) | 2,780,336 -> 2,779,836 (-500) |
| Immediate/release | 4,151,956 -> 4,151,460 (-496) | 2,794,896 -> 2,794,396 (-500) |
| General/size | 1,899,850 -> 1,899,410 (-440) | 1,189,737 -> 1,189,546 (-191) |
| Immediate/size | 1,911,998 -> 1,911,566 (-432) | 1,199,904 -> 1,199,714 (-190) |

The largest native reduction is 0.0232%; the largest optimized-WASM reduction
is 0.0180%. The test-gated topology engine still contributes no production
binary bytes, and no alternate public representation or compatibility path was
introduced.

Relative to the single-bound-BVH checkpoint, the shared-hierarchy follow-up
moves the linked matrix only within 320 bytes:

| Consumer/profile | Native text, before -> after | Optimized WASM, before -> after |
| --- | ---: | ---: |
| General/release | 4,118,236 -> 4,118,204 (-32) | 2,779,836 -> 2,779,870 (+34) |
| Immediate/release | 4,151,460 -> 4,151,428 (-32) | 2,794,396 -> 2,794,424 (+28) |
| General/size | 1,899,410 -> 1,899,546 (+136) | 1,189,546 -> 1,189,866 (+320) |
| Immediate/size | 1,911,566 -> 1,911,694 (+128) | 1,199,714 -> 1,200,027 (+313) |

The maximum movement is 0.0269%; performance is materially better and the
arrangement remains test-gated, so this bounded size trade is retained.

The workspace call-graph utility reports 20,770 nodes/41,280 edges for the five
selected crate source graph and 27,227/51,297 with tests, benches, examples,
and fuzz targets. It inventories 545 arrangement nodes and the unchanged 4,437
historical subdivision/segment-trace/local-BSP nodes. There is no static edge
between the staged arrangement and EMBER machinery.

After the shared-hierarchy commit, the corresponding graphs contain
20,884/41,505 and 27,341/51,522 nodes/edges. They inventory 575 arrangement
nodes and the same 4,437 historical EMBER nodes, with zero edge in either
direction between the two implementations.

The incomplete first evidence serialization was deleted from `/tmp` after a
temporary quota failure; the complete evidence graph was then regenerated
successfully.

## Historical and competitive gauge

The established favorable common-contract row remains Hypermesh exact-box
union at 8.1114 us versus Boolmesh 73.329 us, Manifold-rust 60.191 us, and CGAL
6.0.3 EPECK 156.159 us. The governing historical deficit remains the
full-resolution EMBER result at 3,312.66 seconds and 329,352 KiB RSS versus
approximately 0.09 seconds and 15,516 KiB for CGAL EPECK.

The new 0.18-second disconnected-shell row is deliberately not presented as a
CGAL win: it is a different generated topology/scaling fixture and does not
yet pass through the pinned competitive adapter. Phase 17 must run the same
common-contract corpus through both production engines, case by case, after
Phase 16 cutover. The historical deficit stays open until then.

## Remaining work

Phase 15 still needs production promotion and larger real-world/pathological
siblings for the newly permanent high-genus, self-intersecting/PWN, cavity,
high-operand, and exact-embedding microcases. The staged orchestrator already
builds the source BVH once and transfers compact ownership between intersection
and seed classification; it must become the sole production owner only in the
atomic Phase 16 cutover. Phase 16 must materialize and independently certify
selected facets, migrate all controlled callers directly, and delete EMBER
atomically. No compatibility shim or dual engine will be added.

## Reproduction

```sh
cargo test --locked --all-features surface_arrangement --lib
cargo test --locked --all-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features
cargo test --locked --release --all-features --no-run --lib
HYPERMESH_TOPOLOGY_SHELLS=1536 taskset -c 11 \
  target/release/deps/hypermesh-<hash> \
  surface_arrangement::tests::disconnected_shell_scaling_preserves_every_exact_component --exact
HYPERMESH_TOPOLOGY_SHELLS=1536 valgrind --tool=massif --stacks=yes \
  --massif-out-file=/tmp/hypermesh-phase15-6144.massif \
  target/release/deps/hypermesh-<hash> \
  surface_arrangement::tests::disconnected_shell_scaling_preserves_every_exact_component --exact
HYPERMESH_TOPOLOGY_SHELLS=1536 taskset -c 11 \
  valgrind --tool=callgrind \
  --callgrind-out-file=/tmp/hypermesh-phase15-6144.callgrind \
  target/release/deps/hypermesh-<hash> \
  surface_arrangement::tests::disconnected_shell_scaling_preserves_every_exact_component --exact
cargo run --manifest-path ../tools/hyper-callgraph/Cargo.toml --release -- \
  --root .. --out-dir /tmp/hypermesh-phase15-callgraph \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh --format json
./benchmarks/size-harness/measure.sh default
```
