# Phase 17 canonical graph-point transfer

Date: 2026-08-05

Status: accepted checkpoint; Phases 17 and 18 remain open

Implementation: Hypermesh `c94b4fd3cdfdb479fecf4075aee997b5a608dd46`

Direct parent/evidence: compact source-face work `cf07cbaf` / `38c92caf`

## Result

The pairwise intersection graph already assigns one operation-wide ID to each
exact construction point, but directed per-face events previously transferred
the same point into the surface-arrangement arena repeatedly. The arrangement
now allocates one fallible transient graph-ID-to-arrangement-ID table, imports
each construction point once on first use, and reuses the resulting canonical
arrangement ID for all later directed events.

Source boundary points still enter first and therefore preserve the existing
source-vertex construction-identity precedence. A first graph-point import
still passes the unchanged exact point and construction identity through
`ArrangementPointArena::insert`: structural equality is checked first, exact
rational identities are checked next, and general equality reaches the
existing policy-owned `PointInterner`/Hyperlimit cascade. The table caches only
that completed canonical transfer. It is dropped before retained-source-edge
propagation and per-face CDT, so its stage-local payload is absent from the
large-mesh peak.

This is a general ownership correction, not a benchmark shortcut. Production
contains no fixture, triangle-count, coordinate, operation, topology, result,
competitor, or policy-name branch; no compatibility shim or second Boolean
engine was added. A wrapper experiment and an unconditional-inline experiment
were removed after measurement because they increased deterministic work.

## Exactness and policy behavior

- `STRICT` imports only a point accepted by the unchanged certified/exact
  equality schedule. An unresolved equality remains a typed
  `PredicateUndecided` result.
- `APPROXIMATE_512` can resolve an otherwise unresolved equality only through
  Hyperlimit's terminal 512-bit decision. The operation context continues to
  publish `Approximate512Consumed` if that terminal is used.
- The remap cannot manufacture equality: each entry is written only after the
  graph point and its exact construction identity have completed canonical
  arena insertion.
- Missing graph IDs, allocation failure, and malformed event references remain
  typed errors. A focused regression proves both repeat reuse and the malformed
  table boundary.
- Every deterministic and large-fixture policy pair retains identical exact
  output. The ordinary large corpus remains `Certified`; the depth-128 symbolic
  control remains `Approximate512Consumed` only under its intended approximate
  policy.

## Retired-work measurements

The accepted implementation was rebuilt from a clean release artifact and
compared with the committed compact-face-work executable. Measurements are
CPU-11 pinned Linux `perf stat` instruction/branch counts. Output values,
topology summaries, and certainty are identical.

| Fixture / workload | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| overlapping boxes, all four ×1000 | 3,988,598,107 | 3,948,258,013 | -1.0114% | 688,092,798 | 682,280,329 | -0.8447% |
| sparse 512 shells, all four ×5 | 2,750,323,478 | 2,725,502,212 | -0.9025% | 494,207,703 | 490,560,781 | -0.7379% |
| dense coplanar 32, all four ×1 | 10,289,216,560 | 10,073,488,474 | -2.0966% | 1,755,112,191 | 1,722,415,548 | -1.8629% |
| clipped voxel torus 33, all four ×3 | 2,646,100,131 | 2,645,739,046 | -0.0136% | 444,099,185 | 444,073,164 | -0.0059% |
| 2,049-bit wide boxes, union ×5 | 11,836,833,066 | 11,824,555,926 | -0.1037% | 2,179,694,362 | 2,177,751,509 | -0.0891% |
| full YeahRight, intersection ×1 | 10,404,981,358 | 10,394,943,637 | -0.0965% | 1,786,026,592 | 1,784,402,889 | -0.0909% |

The improvement is broad and largest where many directed events share graph
points. Torus, wide-rational, and full controls are intentionally retained even
though their gains are small: they show that the general transfer schedule does
not trade away work on different topology or scalar shapes.

Symbolic controls also improve:

| Fixture / policy / workload | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change | Certainty |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| depth 1 / `STRICT` / all four ×20 | 4,295,790,760 | 4,285,247,794 | -0.2454% | 938,413,856 | 936,210,808 | -0.2348% | `Certified` |
| depth 128 / `APPROXIMATE_512` / all four ×5 | 1,505,498,946 | 1,505,351,515 | -0.0098% | 274,793,231 | 274,782,583 | -0.0039% | `Approximate512Consumed` |

## Large-fixture heap matrix

All fifteen selectors ran in independent processes under both `STRICT` and
`APPROXIMATE_512`. Each policy pair is byte-for-byte equal and `Certified`.
Every peak is exactly unchanged from the direct parent. The table adds one
allocation per operation; its exact payload is proportional to the operation's
graph-point count and is freed before the measured peak.

| Selector | Peak bytes | Parent/current calls | Parent/current cumulative bytes | Added table payload |
| --- | ---: | ---: | ---: | ---: |
| boxes-3072 | 12,383,968 | 19,227 / 19,228 | 16,571,238 / 16,571,814 | 576 |
| boxes-3072-general | 12,630,096 | 38,413 / 38,414 | 18,901,086 / 18,901,662 | 576 |
| dense-coplanar-16 | 14,893,071 | 276,942 / 276,943 | 66,303,658 / 66,315,962 | 12,304 |
| dense-coplanar-32 | 59,391,879 | 1,106,674 / 1,106,675 | 265,072,426 / 265,121,594 | 49,168 |
| sparse-shells-512 | 11,888,929 | 213,485 / 213,486 | 35,113,460 / 35,125,748 | 12,288 |
| self-PWN-clusters-512 | 12,284,773 | 215,479 / 215,480 | 35,709,262 / 35,721,550 | 12,288 |
| wide-rational-64 | 17,861,062 | 384,611 / 384,612 | 32,608,494 / 32,609,070 | 576 |
| wide-rational-512 | 18,750,079 | 1,295,425 / 1,295,426 | 104,895,582 / 104,896,158 | 576 |
| wide-rational-2048 | 27,423,760 | 1,567,489 / 1,567,490 | 385,301,166 / 385,301,742 | 576 |
| voxel-torus-33 | 12,784,184 | 28,308 / 28,309 | 23,173,108 / 23,174,148 | 1,040 |
| voxel-torus-65 | 51,784,828 | 102,729 / 102,730 | 105,624,183 / 105,626,263 | 2,080 |
| yeahright | 5,161,051 | 167,362 / 167,363 | 11,457,989 / 11,458,693 | 704 |
| yeahright-4 | 19,676,989 | 728,413 / 728,414 | 45,781,994 / 45,783,466 | 1,472 |
| yeahright-8 | 76,258,109 | 3,315,751 / 3,315,752 | 188,359,212 / 188,362,156 | 2,944 |
| yeahright-full-rotated | 59,440,454 | 9,926,707 / 9,926,708 | 588,741,037 / 588,769,229 | 28,192 |

The full row's kernel peak remains 52,317,092 bytes and its reallocation count
remains 1,610,293. The final Heaptrack capture is
`target/phase17-canonical-graph-point-transfer-full-strict.zst.zst`; its peak
flame stacks are `/tmp/hypermesh-canonical-graph-point-transfer-peak.stacks`.
It records 11,645,453 allocation-function calls and 2,669,071 temporary
allocations. The exact Heaptrack total remains 59,514,182 bytes, including
73,728 bytes of profiler/process overhead, versus 59,440,454 allocator bytes.
The direct parent has the same peak and one fewer allocation-function call.

## Historical and competitive boundaries

Fresh full processes preserve exact certified-empty output:

| Policy | Wall time | Maximum RSS |
| --- | ---: | ---: |
| `STRICT` | 1.00 s | 69,316 KiB |
| `APPROXIMATE_512` | 1.00 s | 69,140 KiB |

These one-process clocks are advisory; retired instructions are the stable
control above. Against historical EMBER's 3,312.66 s / 329,352 KiB, current
strict is about 3,313× faster and uses 78.95% less maximum RSS. Historical CGAL
at 0.09 s / 15,516 KiB remains about 11.11× faster and uses 4.47× less RSS.

Pinned CGAL 6.0.3 EPECK exact-OFF medians are unchanged. Hypermesh medians are
63 fresh CPU-11-pinned processes per policy. Inputs and outputs remain exact,
valid, closed, structurally valid, and topology-identical:

| Fixture | CGAL outside / inside | Parent `STRICT` | Current `STRICT` | Ratio | Parent `APPROXIMATE_512` | Current `APPROXIMATE_512` | Ratio |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| crossing octahedra | 112,972 / 122,712 ns | 433,890 ns | 405,112 ns | 3.586× | 430,771 ns | 411,392 ns | 3.642× |
| affine boxes | 376,124 / 383,553 ns | 819,744 ns | 789,356 ns | 2.099× | 815,734 ns | 792,286 ns | 2.106× |

The four Hypermesh medians improve 2.87–6.63% from the direct parent. All CGAL
gaps remain explicit Phase 17 work; no parity or superiority claim is made.

## Code, binary size, and call graph

The implementation changes one production/test file by 73 insertions and 9
deletions. Performance priority accepts bounded artifact growth:

| Configuration | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| default release native text | 1,984,978 | 1,986,058 | +1,080 (+0.0544%) |
| default release optimized WASM | 1,403,288 | 1,403,447 | +159 (+0.0113%) |
| default size native text | 1,089,703 | 1,091,151 | +1,448 (+0.1329%) |
| default size optimized WASM | 678,998 | 679,616 | +618 (+0.0910%) |
| all-feature release native text | 2,120,359 | 2,121,287 | +928 (+0.0438%) |
| all-feature release optimized WASM | 1,477,155 | 1,477,270 | +115 (+0.0078%) |
| all-feature size native text | 1,091,959 | 1,093,391 | +1,432 (+0.1311%) |
| all-feature size optimized WASM | 679,187 | 679,649 | +462 (+0.0680%) |

Immediate-consumer rows are recorded in the machine-readable companion file;
their largest movement is also below 0.133%.

The regenerated five-crate graphs contain 15,107 nodes / 25,193 edges for
production, 17,427 / 28,509 with tests and examples, and 21,473 / 34,610 with
all tests, examples, benches, and fuzz targets. They contain zero removed
EMBER/local-BSP/segment-trace node. There is exactly one
`append_intersection_constraints -> map_graph_point` edge and one
`ArrangementPointArena::with_capacity -> PointInterner::try_with_capacity`
edge. Hypercurve and HyperSolve are excluded and untouched.

## Validation

The following pass on the accepted source:

```sh
cargo fmt --all -- --check
cargo test --locked
cargo test --locked --all-features
cargo test --locked --lib --no-default-features
cargo check --locked --no-default-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-Dwarnings' cargo doc --locked --all-features --no-deps
cargo check --locked --manifest-path fuzz/Cargo.toml --all-targets
cargo check --locked --benches --all-features
cargo check --locked --examples --all-features
benchmarks/size-harness/measure.sh default
benchmarks/size-harness/measure.sh all
```

The all-feature suite passes 199 tests with six documented manual/opt-in
ignores: 152 unit, 8 Boolean, 11 competitive, 14 corpus-manifest, 3
intersection-corpus, 9 policy, and 2 README tests. The minimal library passes
151 tests. Fuzz targets, benches, examples, clippy, and rustdoc compile cleanly.

Phases 17 and 18 remain open for the external real-world corpus, remaining
per-case CGAL gaps, further measured arrangement/corefinement lifetime and
allocation reduction, and the final removal/policy/caller audit.
