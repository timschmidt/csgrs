# Phase 12 checkpoint: compact streamed face-intersection graph

Status: retained

Implementation: `584444cc4a1846c501587fa8944dbd4bd9b02842`

Direct parent: `dad851519ae299d4af353514aa88a76084dd8c00`

This checkpoint replaces `Vec<Vec<PairwiseIntersection>>` with one compact,
face-indexed node arena and consumes BVH candidates directly. It is scaffolding
for the replacement arrangement kernel, not a claim that Phase 12 or the EMBER
removal is complete.

## Representation and path contract

The retained graph stores two `u32` words per face (`head` and `count`) and one
contiguous node arena. Construction temporarily adds one `u32` tail per face;
the tail array is dropped by `finish`. An empty face therefore costs eight
retained bytes instead of the 24-byte `Vec` header on this x86-64 target, and
245 nonempty face-row allocations disappear on the large fixture.

Each node uses a checked `u32` ID and reserves `u32::MAX` as the end marker.
Append reports capacity or face-index errors before mutating the node arena.
Rows are copyable exact-size iterators and preserve callback discovery order.
Cached polygon-family remapping emits directly into a new graph rather than
recreating per-face vectors. No candidate-pair buffer, slice/view compatibility
adapter, dual representation, or policy-dependent storage path remains.

The graph stores topology candidates only. Every intersection predicate and
construction still runs through the operation's `DecisionContext`, so `STRICT`
and `APPROXIMATE_512` retain the same Hyperlimit terminal semantics and
aggregate certainty behavior.

## Correctness and topology

The full default suite passed at the implementation commit:

- 1,071 library tests;
- 4 active competitive tests, with 6 documented opt-in tests ignored;
- 59 core tests;
- 4 corpus-manifest tests;
- 8 predicate-policy tests;
- 2 README tests;
- 48 regression tests, with 1 benchmark-style test ignored; and
- 0 doc-test failures.

The new `boxes-3072-general` heap mode intentionally withholds certified-convex
input facts from the same 6,144-triangle fixture used by `boxes-3072`. Both
policies produced an identical certified result:

| Policy | Certainty | Input triangles | Output vertices | Output triangles |
| --- | --- | ---: | ---: | ---: |
| `STRICT` | `Certified` | 6,144 | 2,410 | 4,816 |
| `APPROXIMATE_512` | `Certified` | 6,144 | 2,410 | 4,816 |

## Large-fixture heap

Valgrind 3.27.0 Massif used `--time-unit=B --detailed-freq=1`; useful heap is
the allocation payload and total includes allocator overhead. These are direct
process maxima, not samples reconstructed from Heaptrack's time series.

| Policy | Revision | Useful heap (B) | Extra heap (B) | Total heap (B) |
| --- | --- | ---: | ---: | ---: |
| `STRICT` | parent | 29,540,348 | 1,229,772 | 30,770,120 |
| `STRICT` | compact graph | 29,450,268 | 1,228,428 | 30,678,696 |
| `STRICT` | change | -90,080 (-0.3049%) | -1,344 | -91,424 (-0.2971%) |
| `APPROXIMATE_512` | parent | 29,540,348 | 1,230,092 | 30,770,440 |
| `APPROXIMATE_512` | compact graph | 29,450,268 | 1,227,804 | 30,678,072 |
| `APPROXIMATE_512` | change | -90,080 (-0.3049%) | -2,288 | -92,368 (-0.3002%) |

Heaptrack independently reports 844,972 parent versus 844,727 candidate
allocation calls (-245, -0.0290%), 34,039 temporary allocations for both, and
rounded peak heap of 29.61 MB versus 29.52 MB. The policy rows have the same
allocation count. Massif is authoritative for byte-level peak comparisons.

## Runtime

Two alternating five-process `perf stat` batches were pinned to CPU 11. Values
below are the mean of the two batch means for the `STRICT` general-path fixture.
Wall time and cycles improved while the counter-stable branch total was flat;
the 0.0718% instruction increase is retained because measured runtime, peak
heap, allocations, and the replacement architecture all improve.

| Revision | Task clock (ms) | Cycles | Instructions | Branches | Branch misses |
| --- | ---: | ---: | ---: | ---: | ---: |
| parent | 1,980.195 | 8,188,558,194 | 24,402,968,350 | 4,693,198,302 | 14,579,496 |
| compact graph | 1,932.325 | 7,995,551,846 | 24,420,486,900 | 4,693,159,825 | 14,203,656 |
| change | -2.4174% | -2.3570% | +0.0718% | -0.0008% | -2.5779% |

Clock readings remained placement/frequency sensitive across additional
alternating batches; those batches ranged from a small candidate loss to a
candidate gain. The decision therefore does not claim a 2.4% universal
speedup. It establishes no material runtime regression while removing heap and
allocation work.

## Native and WASM size

The dependency-only canonical size harness was clean-built for both revisions
with rustc 1.97.0. Native code is linked `.text`; WASM is `wasm-opt -Oz`.

| Consumer/profile | Parent | Compact graph | Change |
| --- | ---: | ---: | ---: |
| General release native `.text` | 4,078,364 | 4,080,980 | +2,616 (+0.0641%) |
| Immediate release native `.text` | 4,111,404 | 4,114,004 | +2,600 (+0.0632%) |
| General release optimized WASM | 2,749,908 | 2,753,443 | +3,535 (+0.1285%) |
| Immediate release optimized WASM | 2,764,465 | 2,767,993 | +3,528 (+0.1276%) |
| General size native `.text` | 1,877,562 | 1,878,770 | +1,208 (+0.0643%) |
| Immediate size native `.text` | 1,889,366 | 1,890,558 | +1,192 (+0.0631%) |
| General size optimized WASM | 1,171,509 | 1,173,631 | +2,122 (+0.1811%) |
| Immediate size optimized WASM | 1,181,650 | 1,183,773 | +2,123 (+0.1797%) |

This is an explicit temporary code-size cost: production source grows by 192
net lines and the canonical linked consumers grow below 0.19%. It is accepted
because performance has priority, the representation lowers large-path heap,
and it is the compact shared substrate needed before deleting 30,578 lines of
historical subdivision/trace/BSP production code. Phase 16 and Phase 17 must
recover this size before final acceptance.

## Reproduction

The parent was an isolated archive of `dad85151`. Its diagnostic example alone
was updated with the candidate's `boxes-3072-general` fixture selector; no
parent library source changed. Workspace path dependencies were pinned to the
same Hyperreal, Hyperlattice, Hyperlimit, and Hypertri revisions for both builds.

```sh
cargo test
cargo build --release --example large_mesh_heap_probe
target/release/examples/large_mesh_heap_probe boxes-3072-general strict
target/release/examples/large_mesh_heap_probe boxes-3072-general approximate-512
valgrind --tool=massif --time-unit=B --detailed-freq=1 \
  --massif-out-file=/tmp/hypermesh-compact-graph.massif \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
taskset -c 11 perf stat -r 5 -x, \
  -e task-clock,cycles,instructions,branches,branch-misses \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
HYPERMESH_SIZE_TARGET_DIR=/tmp/hypermesh-size-compact \
  benchmarks/size-harness/measure.sh default
```
