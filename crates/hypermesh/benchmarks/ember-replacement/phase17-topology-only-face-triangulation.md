# Phase 17 topology-only exact face triangulation checkpoint

Captured 2026-08-03 on the established Ryzen 7 5800X3D / CPU 11 fixture
protocol. This is a retained Phase 17 optimization checkpoint, not a Phase 17
or Phase 18 exit and not a CGAL-parity claim.

## Result

Hypermesh no longer asks Hypertri to perform constrained Delaunay quality work
when a Boolean source face only needs a conforming triangulation. Hypertri now
has one explicit topology-only convex-hull PSLG entry point. It uses exact
orientation, point-location, constraint recovery, cavity, and flip predicates,
then deterministically stabilizes unconstrained diagonals without evaluating an
empty circle. Existing Delaunay entry points retain their quality contract for
callers that request it.

The change is general. It does not inspect fixture names, coordinates, triangle
counts, operation kind, or benchmark state. Every nontrivial bounded source
face enters the same topology API. The existing unchanged-source-face path
continues to use its construction-proven boundary fact.

The full 11,894-by-11,894 exact-empty YeahRight case falls from 48.02 seconds
and 634.365 billion instructions to 9.59 seconds and 122.117 billion
instructions. Linked Hypermesh consumers also shrink by 1.42–3.18% native
`.text` and 1.90–2.33% optimized WASM. The permanent 6,144-triangle control
adds less than 0.05% instructions and no material peak-heap movement.

## Exact algorithm and API

Hypertri production commits `9bbf89c279e54b41f227331484a74fe5ac743f12`
and `40c0783503c27a65a36d7be158130230055c6f4f` provide:

- `cdt::constrained_triangulation_convex_hull`, returning the renamed
  `ConstrainedTriangulation` result type;
- an exact fallible lexicographic point order, exact monotone convex hull,
  orientation-only point insertion, and checked boundary/interior edge splits;
- the existing exact flip/cavity constraint-recovery implementation;
- a strictly descending lexicographic unconstrained-edge stabilization rule;
- separate structural and Delaunay postprocessing call paths; and
- the corrected `2T + 1` unique-edge capacity bound for a triangulated disk.

There is no alias for the former `ConstrainedDelaunayTriangulation` name and no
forwarder or compatibility feature. Controlled callers were migrated directly.
The topology construction is isolated in `src/cdt/topology.rs`; shared recovery
and validation remain single implementations.

Hypermesh commit `1a3bfd099647d9286e78a2a43d38bffddc899533`
routes bounded Boolean face PSLGs to the topology-only API and absorbs
Hypertri's aggregate certainty exactly as before.

Both policies use the same algorithm. A direct symbolic regression proves that
`STRICT` returns `PredicateUndecided` at a terminal orientation while
`APPROXIMATE_512` completes with `Approximate512Consumed`. Exact rational work
remains `Certified` under both policies. No scalar representation, approximate
comparison, or epsilon is used to choose topology.

## Validation

At Hypertri evidence head `afdaf6386098914f854d1027e695f08d1f898bb4`:

- 72 unit, 25 adversarial, 6 differential, 10 property, 5 policy, 2 README,
  and 4 doctests pass (124 total);
- the topology-only property coverage includes opposite diagonals and separated
  closed cycles, and its libFuzzer target shares the constraint invariants;
- a 30-second ASan run completed 10,429 `topology_invariants` executions after
  disabling only LeakSanitizer's ptrace-incompatible final scan;
- every fuzz binary builds; and
- all-target/all-feature Clippy, warning-denied rustdoc, formatting, and diff
  checks pass.

At Hypermesh `1a3bfd09`, 140 tests pass with six documented opt-in/manual
ignores. The suite covers the overlapping coplanar-cell conformity that first
exposed inconsistent diagonals, disconnected inner cells, dense proper
crossings, isolated contacts, nonmanifold radial topology, all built-in and
batched outputs, and both terminal policies. All-target/all-feature Clippy,
warning-denied rustdoc, fuzz-bin compilation, formatting, and diff checks pass.
The full ignored YeahRight oracle was also run repeatedly and remained a
`Certified` empty result.

## Call-graph audit

The exact six-crate production scope (Hyperreal, Hyperlattice, Hyperlimit,
Hypertri, Hypermesh, and CSGRS) contains 18,035 function nodes and 29,542
edges. Starting at `hypermesh::surface_arrangement::corefine_face`, the static
reachable set contains 371 nodes. Starting at
`hypertri::cdt::constrained_triangulation_convex_hull`, it contains 242.
Neither reachable set contains an incircle call, Delaunay point construction,
constrained-Delaunay dispatch, or Delaunay legalization node.

Hypercurve and HyperSolve were not included in the graph scope and were not
edited.

## Full-resolution pathological case

The permanent manual case compares the 11,894-triangle YeahRight control mesh
with a rotated copy under `APPROXIMATE_512`. The output remains exact-empty and
`Certified`.

| Row | Wall | Task clock | Instructions | Maximum RSS |
| --- | ---: | ---: | ---: | ---: |
| Previous retained checkpoint | 48.02 s | 48,977.98 ms | 634,364,981,061 | 203,356 KiB |
| Topology-only final | 9.59 s | 9,628.02 ms | 122,117,257,187 | 200,988 KiB |

The direct wall reduction is 80.03% (5.01x), and deterministic instructions
fall 80.75% (5.19x). A three-repetition pinned counter run reported effectively
zero instruction variation; one wall repetition was preempted, so task clock
is the counter-comparison authority. The unprofiled final run used 9.38 s user,
0.17 s system, no swaps, and no major faults.

Against the historical EMBER row (3,312.66 s / 329,352 KiB), this is 345.43x
faster and 38.97% lower RSS. Against the established historical CGAL EPECK row
(0.09 s / 15,516 KiB), Hypermesh remains 106.56x slower and 12.95x larger by
RSS. That deficit remains open.

The final pinned perf mean also reports 40,081,006,293 cycles,
19,802,265,485 branches, 180,598,937 branch misses, and 56,911,702 cache misses.

## Exact overlapping boxes versus CGAL EPECK

Both engines produce valid closed outputs with expected union/intersection/
difference/reverse-difference volumes 84/12/52/20 and triangle counts
48/24/40/32. Hypermesh evaluates all four outputs from one shared arrangement.

| Engine / policy | Median | Ratio to CGAL copy outside | Ratio to CGAL copy inside |
| --- | ---: | ---: | ---: |
| Hypermesh `STRICT` | 985.67 us | 8.24x | 7.64x |
| Hypermesh `APPROXIMATE_512` | 995.83 us | 8.33x | 7.72x |
| CGAL 6.0.3 EPECK, copy outside | 119.5965 us | 1.00x | — |
| CGAL 6.0.3 EPECK, copy inside | 128.9760 us | — | 1.00x |

The CGAL rows are 30 warmed, CPU-11-pinned repetitions. Copy-outside min/mean/
max are 112.232/123.425/157.599 us; copy-inside values are
116.882/142.051/220.685 us. Small-case parity remains open.

## Permanent 6,144-triangle runtime and heap control

Every row returns the same `Certified` 2,410-vertex/4,816-triangle union.
Eleven pinned perf repetitions show the topology-only path is essentially flat
on this easy control:

| Input path | Policy | Task clock | Instructions | Delta from prior checkpoint |
| --- | --- | ---: | ---: | ---: |
| Native retained | `STRICT` | 162.71 ms | 1,761,491,358 | +0.0489% |
| Native retained | `APPROXIMATE_512` | 164.74 ms | 1,761,325,176 | +0.0339% |
| Raw/general | `STRICT` | 153.93 ms | 1,635,920,499 | +0.0379% |
| Raw/general | `APPROXIMATE_512` | 152.59 ms | 1,635,812,887 | +0.0333% |

Sequential Heaptrack recordings of this large fixture report:

| Input path | Policy | Allocations | Recorder temporary | Peak heap | Heaptrack RSS |
| --- | --- | ---: | ---: | ---: | ---: |
| Native retained | `STRICT` | 331,636 | 84,883 | 16,424,002 B | 23.77 MB |
| Native retained | `APPROXIMATE_512` | 331,636 | 84,883 | 16,423,650 B | 23.75 MB |
| Raw/general | `STRICT` | 294,754 | 84,883 | 16,423,658 B | 23.78 MB |
| Raw/general | `APPROXIMATE_512` | 294,754 | 84,883 | 16,424,010 B | 23.79 MB |

The 360-byte peak spread is below 0.0022% and brackets the previous exact
16,423,650/16,423,658-byte rows. Constraint topology adds 558 allocation calls
and 99 recorder-temporary calls on the fixture; the live-heap envelope does not
move materially. No heap conclusion is inferred by subtracting unrelated
process snapshots.

## Linked and source size

The split lets Hypermesh consumers discard Delaunay incircle/legalization code.
Every canonical linked row shrinks relative to the previous retained evidence:

| Features/profile/consumer | Native `.text` | Delta | Optimized WASM | Delta |
| --- | ---: | ---: | ---: | ---: |
| default/release/general | 1,932,490 | -3.0605% | 1,342,455 | -2.2723% |
| default/release/immediate | 1,935,642 | -3.0557% | 1,344,303 | -2.2693% |
| default/size/general | 1,034,303 | -1.4250% | 635,291 | -1.9072% |
| default/size/immediate | 1,035,243 | -1.4245% | 635,688 | -1.9062% |
| all/release/general | 2,067,727 | -3.1776% | 1,420,724 | -2.3311% |
| all/release/immediate | 2,070,575 | -3.1730% | 1,422,691 | -2.3280% |
| all/size/general | 1,035,895 | -1.4176% | 635,325 | -1.9038% |
| all/size/immediate | 1,036,843 | -1.4163% | 635,596 | -1.9047% |

The general topology API is real source, not a benchmark deletion trick.
Hypertri production source adds 555 physical lines and removes 42 versus
`74d739f`; its current `src` has 8,470 Tokei code lines. Hypermesh production
source changes by one net physical line and has 15,138 Tokei code lines. The
new code is kept because it closes a general topology/quality ownership error,
delivers the dominant runtime win, and reduces every shipped binary row. No
shim, duplicate recovery engine, or benchmark selector was added.

## Open work

CGAL parity is not reached. The post-change profile moves the dominant hard-case
cost to exact radial ray/dot classification and pairwise orientation work;
bounded triangulation is no longer the leading cost. Phase 17 must continue
with clean retained-fact scheduling there. Corpus completion, direct
kernel-lifetime heap boundaries, broader per-case CGAL execution, and the Phase
18 completion audit remain open.

## Reproduction

```sh
cargo test --all-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS=-Dwarnings cargo doc --no-deps --all-features
cargo +nightly check --manifest-path fuzz/Cargo.toml --bins
cargo fmt --all -- --check

taskset -c 11 cargo bench --bench competitive -- \
  'competitive_shared_arrangement/hypermesh/overlapping_boxes'
taskset -c 11 perf stat -r 11 -x, \
  -e task-clock,cycles,instructions,branches,branch-misses \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
heaptrack --record-only \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict

env YEAHRIGHT_BENCH=1 /usr/bin/time -v taskset -c 11 \
  target/release/deps/competitive-92c64513605410c9 \
  full_resolution_yeahright_rotated_intersection_certifies_empty \
  --ignored --exact --nocapture

benchmarks/size-harness/measure.sh default
benchmarks/size-harness/measure.sh all

tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir hypermesh/target/hypermesh-topology-final-callgraph \
  --format json \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh,csgrs \
  --per-library
```
