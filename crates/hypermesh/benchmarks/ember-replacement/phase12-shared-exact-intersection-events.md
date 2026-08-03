# Phase 12/13 checkpoint: shared exact intersection events

Status: retained

Hypermesh implementation: `a824644a76887e53b87d462b6e290f75f4967689`

Hyperlimit implementation: `bb1c5e7c418bb8af25701c8201f833fa485b9e9a`

Direct Hypermesh parent: `816bb81f`

This checkpoint removes the duplicated exact geometry retained by the compact
face-intersection graph. It is compact arrangement/intersection scaffolding,
not a claim that Phase 12, Phase 13, EMBER removal, or the CGAL target is
complete.

## Representation and failure contract

Each undirected non-coplanar intersection now owns one exact endpoint record.
Its two directed face events store only a shared checked segment ID and the
other checked face ID. Coplanar events store only the other face ID; edge and
support planes are borrowed from the immutable source polygon instead of being
cloned into every event. The encoded node is three `u32` words (`next`, other
face, and segment-or-sentinel), so it occupies 12 bytes on the measured target.
The public standalone polygon-intersection result no longer clones support or
edge planes that its caller can obtain from the supplied polygons.

Paired insertion validates distinct/in-range faces, checked `u32` IDs, row
counts, segment capacity, node capacity, and allocation reservations before
changing any logical graph state. A failed pair therefore cannot leave a half
edge or orphaned exact segment. Polygon-order remapping validates its
permutation, every linked node, every segment ID, and every remapped face ID.
There is no slice/view adapter, old event representation, or dual path.

The graph carries no policy-sensitive conclusion. All intersection, overlap,
ordering, containment, and BSP decisions still use the operation's
`DecisionContext`; `STRICT` and `APPROXIMATE_512` therefore retain Hyperlimit's
existing terminal semantics and aggregate certainty.

## Cross-layer broad-phase code generation

The first release build exposed an unrelated code-generation instability:
consumer-side inlining expanded Hyperlimit's complete Real comparison cascade
inside `bounds_overlap_decision`. Deterministic Callgrind showed the same
6,433,159 AABB calls but 117,385,002 versus 169,607,777 self instructions, and
the symbol grew from `0x2aa` to `0x578` bytes. Hyperlimit `bb1c5e7c` adds one
five-byte, non-inlined tail-call boundary around the unchanged policy-aware
comparison. Both parent and candidate were rebuilt with that same boundary for
the controlled graph A/B. The AABB wrapper then measured `0x2b0` bytes in both
revisions, with no change to the certified/terminal policy cascade.

## Correctness and path evidence

The default Hypermesh suite passed with 1,074 library tests, 4 active
competitive tests (6 opt-in tests ignored), 59 core tests, 4 corpus tests, 8
predicate-policy tests, 2 README tests, and 48 regression tests (1 benchmark
test ignored). Minimal features passed the same matrix. All features passed
1,075 library and 60 core tests plus the same integration rows. Warning-denied
Clippy and rustdoc, formatting, both native/WASM feature checks, and every fuzz
binary check passed.

Hyperlimit passed its default, minimal, and all-feature suites (142/142/150
library tests respectively plus every integration row), warning-denied Clippy
and rustdoc, formatting, both native/WASM feature checks, and every fuzz binary
check. Pre-existing untracked Hyperlimit fuzz artifacts/corpus data were not
modified or committed.

The `boxes-3072-general` large fixture contains 6,144 input triangles and
deliberately enters the general path. Both policies produced an identical
`Certified` result with 2,410 vertices and 4,816 triangles.

The regenerated five-crate call graph contains 19,941 production nodes and
39,811 edges; its test/bench/example/fuzz graph contains 26,359 nodes and
49,757 edges. It records
`bounds_overlap_decision -> ordered_aabb3s_intersect -> compare_aabb_reals -> compare_reals_with_policy`
and the streamed intersection route into `PairwiseIntersectionGraphBuilder`.
No `PairwiseIntersectionView` or `Vec<Vec<PairwiseIntersection>>` compatibility
representation exists. Hypercurve and HyperSolve were excluded and untouched.

## Large-fixture heap

Valgrind 3.27.0 Massif used `--time-unit=B --detailed-freq=1`. These are direct
process maxima on the large fixture.

| Policy | Revision | Useful heap (B) | Extra heap (B) | Total heap (B) |
| --- | --- | ---: | ---: | ---: |
| `STRICT` | parent | 29,450,268 | 1,227,836 | 30,678,104 |
| `STRICT` | shared events | 28,864,564 | 1,227,340 | 30,091,904 |
| `STRICT` | change | -585,704 (-1.9888%) | -496 | -586,200 (-1.9108%) |
| `APPROXIMATE_512` | parent | 29,450,268 | 1,228,316 | 30,678,584 |
| `APPROXIMATE_512` | shared events | 28,864,564 | 1,226,092 | 30,090,656 |
| `APPROXIMATE_512` | change | -585,704 (-1.9888%) | -2,224 | -587,928 (-1.9164%) |

Heaptrack independently reported 844,727 versus 844,735 allocation calls and
33,802 versus 33,794 temporary allocations. The eight growth allocations are
accepted because useful peak falls 585,704 bytes, total peak falls about 1.91%,
and deterministic work also improves. Massif is authoritative for byte-level
peak comparisons.

## Runtime

Two alternating five-process `perf stat` batches were pinned to CPU 11. Both
revisions used Hyperlimit `bb1c5e7c`; values are the mean of the two batch means
for the `STRICT` large general-path fixture.

| Revision | Task clock (ms) | Cycles | Instructions | Branches | Branch misses |
| --- | ---: | ---: | ---: | ---: | ---: |
| parent | 2,091.385 | 8,464,117,377 | 24,422,459,824 | 4,693,546,069 | 14,382,991 |
| shared events | 2,071.195 | 8,413,046,371 | 24,416,096,898 | 4,692,496,542 | 14,293,052 |
| change | -0.9654% | -0.6034% | -0.0261% | -0.0224% | -0.6253% |

Clock and cycle readings remain frequency/order sensitive; the smaller stable
instruction and branch counts establish that the compact representation does
not buy heap by adding deterministic work.

## Native and WASM size

The dependency-only harness was clean-built with rustc 1.97.0. The controlled
table rebuilds both parent and candidate with the same Hyperlimit outlining
boundary. Native code is linked `.text`; WASM is `wasm-opt -Oz`.

| Consumer/profile | Parent (B) | Shared events (B) | Change |
| --- | ---: | ---: | ---: |
| General release native `.text` | 4,080,804 | 4,083,004 | +2,200 (+0.0539%) |
| Immediate release native `.text` | 4,113,860 | 4,116,028 | +2,168 (+0.0527%) |
| General release optimized WASM | 2,753,327 | 2,755,317 | +1,990 (+0.0723%) |
| Immediate release optimized WASM | 2,767,885 | 2,769,936 | +2,051 (+0.0741%) |
| General size native `.text` | 1,878,874 | 1,880,338 | +1,464 (+0.0779%) |
| Immediate size native `.text` | 1,890,678 | 1,892,134 | +1,456 (+0.0770%) |
| General size optimized WASM | 1,173,589 | 1,174,206 | +617 (+0.0526%) |
| Immediate size optimized WASM | 1,183,734 | 1,184,346 | +612 (+0.0517%) |

Against the checked-in `816bb81f` size rows before the Hyperlimit boundary,
the combined stack grows only 0.0484--0.0835%; the boundary itself offsets part
of the graph code. This remains a temporary sub-0.09% cost. Phase 16 must delete
the historical subdivision/trace/BSP implementation, and Phase 17 must recover
the remaining linked-size delta without giving back runtime or heap.

## Reproduction

The parent was an isolated archive of `816bb81f` with absolute paths to the
same workspace Hyperreal, Hyperlattice, Hyperlimit, and Hypertri sources.

```sh
cargo test
cargo test --no-default-features
cargo test --all-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --no-deps --all-features
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo build --locked --release --example large_mesh_heap_probe
valgrind --tool=massif --time-unit=B --detailed-freq=1 \
  --massif-out-file=/tmp/hypermesh-shared-events.massif \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
taskset -c 11 perf stat -r 5 -x, \
  -e task-clock,cycles,instructions,branches,branch-misses \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
HYPERMESH_SIZE_TARGET_DIR=/tmp/hypermesh-size-shared-events \
  benchmarks/size-harness/measure.sh default
tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir target/hypermesh-ember-phase12-shared-segments-production \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --per-library
```
