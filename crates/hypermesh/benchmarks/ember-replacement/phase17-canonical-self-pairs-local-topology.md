# Phase 17 canonical self-pairs and local topology checkpoint

Captured 2026-08-03 on the established Ryzen 7 5800X3D / CPU 11
single-thread protocol at Hypermesh `9ebefb108b8eea6863b9ef04b0916d72103825f3`
and Hypertri `186a29eb74ade9126f605184d1275a504450b9ec`.
This is a retained Phase 17 optimization checkpoint. Corpus completion, CGAL
EPECK parity, linked-size recovery, and the Phase 18 audit remain open.

## Result

Two general scheduling changes remove repeated topology discovery without
changing a geometric decision:

- Hypermesh traverses one exact BVH as canonical node pairs, so each distinct
  primitive pair is considered once instead of querying the same hierarchy
  once per primitive and later discarding the reverse pair.
- Hypertri sorts the three edge uses of every current triangle once per pass
  and carries the two exact owners with each interior edge. Constraint
  recovery, cavity construction, legalization, and canonicalization no longer
  rescan all triangles to rediscover adjacency for every selected edge.

The changes inspect no fixture name, coordinate magnitude, triangle count,
operation, policy, expected output, benchmark state, or competitor. Exact
predicates remain the only topology decisions. The BVH is a conservative work
scheduler; primitive AABBs are checked by the existing policy-aware exact
predicate before a pair is emitted. Hypertri still requires the same strict
convex-quadrilateral and Delaunay predicates before replacing a diagonal.

On the permanent 6,412/25,100-triangle torus family, warm `STRICT` time falls
15.87%/23.35%, instructions fall 14.65%/24.08%, allocation calls fall
16.57%/25.12%, and reallocations fall 97.53%/98.83%. Exact topology, volume,
and `Certified` certainty are unchanged under both policies. Incremental peak
heap is essentially unchanged because a later exact-geometry stage remains the
governing live set.

## Canonical self-overlap traversal

For a node paired with itself, the traversal schedules only `(left,left)`,
`(left,right)`, and `(right,right)`. For distinct internal nodes it schedules
their four child products; mixed internal/leaf pairs split only the internal
side. Leaves in the same node use the strict upper triangle of their local
primitive range. Distinct leaves use their ordinary Cartesian product.

These invariants make every primitive pair unique by construction. Polygon IDs
are normalized before callback, and equal IDs are discarded. There is no
pair-result hash set, per-primitive query stack, per-query match vector, reverse
pair, or post-hoc deduplication. Exact node and primitive bounds remain
conservative policy-aware gates, never topology authorities.

Brute-force differential tests compare the emitted set with every exact
primitive-AABB pair under `STRICT` and `APPROXIMATE_512`. Empty and singleton
trees produce no pair. Corrupt child indices, leaf ranges, and primitive
indices return typed `SurfaceArrangementFailed` errors instead of indexing or
silently omitting work. The public two-hierarchy BVH API is unchanged; only the
internal self-overlap owner uses the new traversal.

## One topology stream per Hypertri pass

`TopologyEdges` stores one 24-byte `(canonical edge, triangle owner)` record for
each triangle side, checks the `3T` capacity multiplication, rejects repeated
triangle vertices, sorts deterministically, and exposes either one boundary
owner or two interior owners. More than two uses is a nonmanifold error;
duplicate ownership by one triangle is also rejected.

The selected edge therefore carries its current adjacent triangle IDs and
opposite vertices through the exact flippability/illegality test to the local
write. No global scan or second set of four exact side predicates occurs
between proof and replacement. Each mutation ends the current iteration, so a
fresh topology stream is constructed before examining the next triangulation
state. This preserves the previous deterministic order and validation
contract while changing repeated `O(E*T)` adjacency discovery into one
`O(T log T)` sort per pass plus local owner lookups.

## Policy and exactness

Both policies use identical algorithms and outputs on all measured exact
rational fixtures:

- `STRICT` permits only certified comparisons.
- `APPROXIMATE_512` reaches the same centralized Hyperlimit terminal if an
  exact comparison remains undecided and aggregates any consumed certainty.
- Every checkpoint row remains `Certified`; the approximate terminal is not
  needed for these rational inputs.
- No cache or retained topology fact crosses policy contexts.
- No epsilon, primitive float comparison, alternate topology, or approximation
  is introduced by either scheduler.

The four direct large/XL heap executions are byte-for-byte identical between
policies, including allocation, deallocation, reallocation, and peak counters.

## Current exact torus comparison

Hypermesh timing is aggregate elapsed divided by 101/31/11 warmed production
calls after exact import and policy-qualified PWN priming. The pinned CGAL 6.0.3
EPECK values are unchanged 21-call medians over the identical reduced-rational
OFF inputs, with required input copies outside the timed interval.

| Input triangles | Policy | Prior Hypermesh | Current Hypermesh | Change | CGAL median | Current / CGAL |
| ---: | --- | ---: | ---: | ---: | ---: | ---: |
| 460 | `STRICT` | 9.745 ms | 8.800 ms | -9.70% | 0.493 ms | 17.85x |
| 460 | `APPROXIMATE_512` | 9.836 ms | 8.690 ms | -11.65% | 0.493 ms | 17.63x |
| 6,412 | `STRICT` | 127.903 ms | 107.600 ms | -15.87% | 3.504 ms | 30.71x |
| 6,412 | `APPROXIMATE_512` | 128.980 ms | 108.688 ms | -15.73% | 3.504 ms | 31.02x |
| 25,100 | `STRICT` | 602.395 ms | 461.710 ms | -23.35% | 11.839 ms | 39.00x |
| 25,100 | `APPROXIMATE_512` | 596.550 ms | 465.820 ms | -21.91% | 11.839 ms | 39.35x |

The scale-dependent deficit is smaller but remains substantial and is not
relabeled competitive.

## Fresh-process counters and slope

Eleven pinned repetitions include deterministic fixture construction, exact
import, policy-qualified PWN priming, and one `STRICT` Boolean call.

| Input triangles | Task clock | Instructions | Branches | Branch misses | Cache misses |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 6,412 | 131.11 ms | 1,473,347,411 | 263,198,394 | 1,846,180 | 1,269,313 |
| 25,100 | 560.07 ms | 6,309,046,742 | 1,126,465,365 | 6,693,610 | 5,904,766 |

Relative to the preceding corpus checkpoint, instructions fall 14.65% and
24.08%; branches fall 14.76% and 21.45%. Large cache misses move +0.76%, within
the noisy row, while XL cache misses fall 8.05%. For 3.9145x input triangles,
current task clock/instructions/branches grow 4.2718x/4.2821x/4.2799x rather
than 4.6912x/4.8142x/4.6446x. Scaling remains superlinear and open.

## Direct requested-payload heap

Each fixture was run in a fresh serialized process under both policies. The
table shows the policy-identical values and changes from the preceding corpus
checkpoint.

| Input triangles | Incremental peak | Alloc calls | Change | Realloc calls | Change | Added bytes | Change |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 6,412 | 16,387,156 B | 255,263 | -16.57% | 927 | -97.53% | 90,078,606 B | -3.23% |
| 25,100 | 65,352,128 B | 984,907 | -25.12% | 1,778 | -98.83% | 371,452,377 B | -3.97% |

The large peak falls only 1,400 bytes (0.0085%); XL peak is byte-identical.
Prepared input, output-live payload, and input-fact growth also remain
unchanged: 656,792/3,210,704 bytes, 375,368/1,623,160 bytes, and
42,840/699,616 bytes respectively. The input-fact lifetime cliff therefore
remains open.

Fresh-process maximum RSS is 21,280/73,912 KiB versus the retained CGAL
9,556/20,020 KiB rows, or 2.23x/3.69x. Requested-payload allocation work
improves materially, but the RSS target is not closed.

## Historical and small-case controls

The full 11,894-by-11,894 YeahRight intersection remains exact-empty and
`Certified` under `APPROXIMATE_512`:

| Row | Task clock | Instructions | Maximum RSS |
| --- | ---: | ---: | ---: |
| Fused construction/local-flip checkpoint | 3,380.23 ms | 36,921,998,673 | 192,376 KiB |
| Canonical pairs/local topology | 3,219.37 ms | 35,257,061,862 | 192,164 KiB |

This is another 4.76% task-clock and 4.51% instruction reduction. The current
pinned counter row is 1,028.98x faster than historical EMBER's 3,312.66
seconds, but still 35.77x slower and 12.38x larger by RSS than the established
historical CGAL 0.09-second/15,516-KiB row.

The shared exact-box Criterion point estimates improve to 857.12 us
(`STRICT`) and 857.68 us (`APPROXIMATE_512`), 7.17x CGAL with copy outside and
6.65x with copy inside. All four exact volumes and output topologies remain
unchanged. Small-case parity is open.

## Profile ownership

On the 25,100-triangle torus profile, inclusive samples now attribute 58.75%
to pairwise traversal, 57.65% to the canonical self-BVH subtree, 44.52% to
polygon intersection, and 34.98% to coplanar intersection. Exact AABB work
falls from 9.77% before the self traversal to 4.75%. Hypertri CDT falls from
20.97% to 12.68%; topology insertion is 11.63%, exact flippability 4.69%, and
the sorted topology stream 4.44%.

The profile therefore points next to general coplanar classification and
construction scheduling. It does not support a fixture-specific shortcut or a
different benchmark algorithm.

## Linked size

All four default-feature canonical consumers were rebuilt. Relative to the
preceding fused/corpus baseline:

| Profile / consumer | Native `.text` | Change | `wasm-opt -Oz` | Change |
| --- | ---: | ---: | ---: | ---: |
| release / general | 2,029,570 B | -0.1700% | 1,439,074 B | -0.1391% |
| release / immediate | 2,032,714 B | -0.1701% | 1,440,931 B | -0.1385% |
| size / general | 1,078,207 B | +0.0906% | 673,274 B | -0.2624% |
| size / immediate | 1,079,147 B | +0.0898% | 673,560 B | -0.2803% |

Release performance consumers and every WASM row shrink. The two sub-0.1%
size-profile native increases remain visible. Hypermesh adds 211 net source
lines and Hypertri adds 95 net source lines, including differential,
malformed-storage, and local-topology tests. There is still one Boolean engine,
no compatibility layer, and no feature or runtime selector.

## Rejected eager dyadic AABB schedule

A Hyperlimit prototype extracted all twelve coordinate views and attempted an
exact-dyadic AABB decision before the retained rational branch. AABB exactness
tests passed, but the eager extraction raised torus instructions by 3.0–3.7%
and slowed XL. It was removed completely.

This result reinforces Hyperreal's actual strength: the existing rational path
benefits from retained facts and delayed work. Flattening or eagerly exposing
representations is not automatically faster, even when the replacement kernel
is arithmetically smaller.

## Validation and call graph

Hypertri passes all 256 test feature combinations, all 256 all-target compile
combinations, no-default/all-algorithm and all-feature tests, warning-denied
Clippy and rustdoc, the UI build, and formatting. Hypermesh passes 151 executed
tests with six documented ignores, no-default/all-feature builds, warning-
denied Clippy and rustdoc, every fuzz-bin check, benchmark compilation,
formatting, the full ignored YeahRight oracle, and the three fixed native fuzz
seed executions.

The regenerated five-crate graph contains 15,393 nodes and 25,185 edges for
Hyperreal, Hyperlattice, Hyperlimit, Hypertri, and Hypermesh with examples.
Hypercurve and HyperSolve were neither scanned nor edited.

## Open work

Phase 11 still needs legally distributable external real-world pathologies,
dense-coplanar and fixed-coordinate-complexity scaling siblings, and broader
high-bit/deep-symbolic families. Phase 17 must reduce coplanar and pairwise
work, the remaining superlinear slope, input fact lifetime, peak/RSS, and every
current CGAL deficit without making the algorithm less general or less clean.
Source/native-size consolidation and the full Phase 18 path audit also remain
open.

## Reproduction

```sh
cargo test --all-features
cargo check --no-default-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --no-deps --all-features
cargo +nightly check --manifest-path fuzz/Cargo.toml --bins
cargo bench --no-run
cargo fmt --all -- --check

taskset -c 11 target/release/examples/competitive_arrangement_probe \
  clipped_voxel_torus_65 intersection strict 11
taskset -c 11 perf stat -r 11 -x, \
  -e task-clock,cycles,instructions,branches,branch-misses,cache-misses \
  target/release/examples/large_mesh_heap_probe voxel-torus-65 strict
taskset -c 11 target/release/examples/large_mesh_kernel_heap_probe \
  voxel-torus-65 approximate-512

YEAHRIGHT_BENCH=1 taskset -c 11 perf stat -r 3 -x, \
  -e task-clock,cycles,instructions,branches,branch-misses,cache-misses \
  target/release/deps/competitive-92c64513605410c9 \
  full_resolution_yeahright_rotated_intersection_certifies_empty \
  --ignored --exact --nocapture

benchmarks/size-harness/measure.sh default

tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir target/hypermesh-path-callgraph-phase17-canonical-pairs-topology \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --include-examples --format json,dot --per-library
```
