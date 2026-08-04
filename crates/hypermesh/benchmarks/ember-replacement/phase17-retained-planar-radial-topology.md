# Phase 17 retained planar radial-topology checkpoint

Captured 2026-08-03 on the established Ryzen 7 5800X3D / CPU 11 protocol at
Hypermesh `349dddd24109e70f616ad6496a0dd16e3a2773c9` and Hyperreal
`c2217bae132b725e29faeb89495b8cd51012e1bc`.

This checkpoint stops rediscovering radial geometry that the validated planar
face arrangements already prove. It is one scheduling improvement inside the
path-complete surface-arrangement engine. It does not recognize a fixture,
coordinate range, triangle count, operation, expected output, or policy; it
does not add a Boolean engine, compatibility path, retry, or semantic cap.

Phase 17 and Phase 18 remain open. The full case is materially faster, but it
still loses to the established CGAL EPECK row in runtime and RSS, and the
remaining corpus and final path-audit gates are not complete.

## Retained topological facts

`corefine_face` produces a validated, nonoverlapping planar triangulation for
each source face. `bundle_surface_facets` then records the exact source-face
contributions of every geometric facet. Two consequences can be consumed
without rebuilding materialized vectors and exact scalar predicates:

1. Two distinct facets contributed by the same validated source-face
   triangulation occupy opposite rays around their shared degree-two edge.
   This source ownership is a complete separation proof, so the former
   orientation-plus-perpendicular-dot equality test is unnecessary.
2. A degree-four edge with four unbundled facets split two-and-two between two
   source faces consists of two antipodal ray pairs. One policy-aware nonzero
   orientation orders representatives of those two pairs and therefore proves
   the complete alternating cycle `a, b, -a, -b`. The other three geometric
   relations and a four-element exact sort are redundant.

The second rule declines on a zero/undecided orientation, bundled
contributions, any face multiplicity other than two-and-two, and every degree
other than four. Coplanar, coincident, nonmanifold, and higher-degree cases
continue through the complete exact radial sorter. The existing retained
authored-adjacency proof remains unchanged.

The rule relies on a general arrangement invariant rather than an assumed
manifold input. A source face's triangulation is validated before radial
assembly; duplicate geometric facets are bundled; and a planar triangulation
cannot place two distinct incident triangles on the same side of an interior
edge. The new focused test compares the retained cycle against the complete
radial sorter for both policies, every permutation of four uses, orthogonal
rays, an oblique edge, unequal antipodal scales, and axial offsets. Coplanar
incidence directly proves decline without mutating the cell sets.

## Policy and exactness

No scalar comparison moved outside `DecisionContext`:

- `STRICT` still rejects an orientation that Hyperlimit cannot certify;
- `APPROXIMATE_512` may consume only Hyperlimit's terminal 512-bit decision;
- an exact zero declines to the complete radial path; and
- retained source incidence is policy-independent by construction.

All measured exact fixtures produce identical values and allocator metrics
under both policies with `Certified` certainty. The approximate terminal is
not consumed. Dispatch tracing reports zero unknown-fact events and zero
fallback-or-abort events.

## Full-resolution paired performance

The paired baseline was rebuilt with only the new radial schedules disabled;
the helper body was unreachable and removed by release linking. Both binaries
otherwise used the same current five-crate source and toolchain. Five fresh
processes per side include fixture loading, exact import, PWN work, one
23,788-triangle rotated YeahRight intersection, result validation, and
destruction. Runs were serialized and pinned to CPU 11.

| Metric | Paired baseline | Current | Movement |
| --- | ---: | ---: | ---: |
| Median test time | 3.14 s | 2.36 s | -24.84% |
| Cycles | 12,670,748,577 | 9,311,200,332 | -26.51% |
| Instructions | 35,244,558,675 | 24,604,987,622 | -30.19% |
| Branches | 5,966,280,753 | 4,371,171,205 | -26.74% |
| Cache misses | 22,644,345 | 21,275,315 | -6.05% |

The degree-four retained cycle alone measured 28,592,673,643 instructions and
4,974,755,917 branches. The same-face degree-two proof then removed another
13.95% and 12.13%, respectively. A temporary diagnostic, removed before the
commit, observed 48,910 degree-two and 4,924 degree-four edges in this fixture;
38,796 degree-two edges already used the earlier authored-adjacency proof.

The exact result remains empty and `Certified`. At 2.36 seconds the current
row is approximately 1,403.7x faster than historical EMBER's 3,312.66 seconds.
It remains approximately 26.22x slower than the established 0.09-second CGAL
EPECK row, so the competitive gate is explicitly open.

## Large-fixture heap and RSS

The allocator-instrumented boundary excludes fixture preparation, keeps inputs
alive through the Boolean, and drops output and inputs separately. Both policy
runs are byte- and count-identical.

| Metric | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| Incremental kernel peak | 158,258,204 B | 158,258,204 B | unchanged |
| Allocation calls | 39,175,099 | 28,631,893 | -26.91% |
| Reallocation calls | 4,027,656 | 3,113,322 | -22.70% |
| Allocated-byte churn | 2,445,957,944 B | 1,447,448,648 B | -40.82% |
| Input-attached fact growth | 24,389,848 B | 24,389,848 B | unchanged |
| Output-live payload | 56 B | 56 B | unchanged |
| Post-input residual | 10,792 B | 10,792 B | unchanged |

The ordinary process reaches 192,156 KiB maximum RSS with zero swaps, within
noise of the prior 192,304 KiB checkpoint. Peak live storage therefore does
not improve merely because churn does. The established CGAL row is 15,516
KiB, leaving a 12.38x RSS deficit.

## Independent controls

These controls ensure the retained topology rule is not a hard-case-only
shortcut. Unless otherwise stated, they are directional reruns against the
most recent pinned evidence rather than new confidence intervals.

- The 2,049-bit, 6,144-triangle wide-rational union is effectively neutral at
  4,195,599,349 instructions versus 4,195,442,185 (+0.0037%). Its both-policy
  direct heap remains exactly 31,038,074 peak bytes, 2,109,647 allocations,
  228,217 reallocations, 463,318,390 added bytes, 26,568 retained-fact bytes,
  and 520,472 output bytes.
- The 6,412- and 25,100-triangle clipped voxel-torus rows improve from
  1,307,616,836 to 1,303,920,505 instructions (-0.28%) and from
  5,602,180,760 to 5,592,161,042 (-0.18%).
- The 6,144-triangle bounded-coordinate dense-coplanar row moves from the
  pinned 3.622 billion to 3,616,449,330 instructions (about -0.15%).
- A warm 101-call exact-box all-operation mean improves from 864.20 to
  818.84 microseconds under `STRICT` (about -5.25%). Against the pinned CGAL
  row it remains about 6.85x slower.
- A warm 11-call medium dense-coplanar intersection measures 16.663 ms versus
  pinned CGAL's 16.036 ms, a 1.039x ratio inside the plan's 1.05 noise
  boundary for this directional run. The larger members' existing CGAL wins
  remain intact.

No control changes output topology or certainty.

## Linked size

Performance has priority, but all canonical native and WASM consumers remain
measured. Relative to the exact linear-arithmetic checkpoint:

| Profile / consumer | Parent native text | Current native text | Movement | Parent optimized WASM | Current optimized WASM | Movement |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| release / general | 2,020,214 B | 2,022,254 B | +2,040 B (+0.101%) | 1,437,333 B | 1,439,446 B | +2,113 B (+0.147%) |
| release / immediate | 2,023,358 B | 2,025,398 B | +2,040 B (+0.101%) | 1,439,188 B | 1,441,304 B | +2,116 B (+0.147%) |
| size / general | 1,084,423 B | 1,085,687 B | +1,264 B (+0.117%) | 679,361 B | 680,458 B | +1,097 B (+0.161%) |
| size / immediate | 1,085,371 B | 1,086,635 B | +1,264 B (+0.116%) | 679,768 B | 680,869 B | +1,101 B (+0.162%) |

This is an explicit size cost, retained because the general rule removes
30.19% of full-case instructions, 40.82% of allocated-byte churn, and also
improves or preserves the independent controls. Production adds one bounded
stack classifier and no arena, map, dependency, feature, or public API.

## New profile ownership

The frame-pointer profile at
`/tmp/hypermesh-current-retained-radial-topology-fp.data` contains 2,427
samples with none lost. Radial equality is no longer a top inclusive owner.
The new inclusive profile is led by face corefinement (41.71%), pairwise
intersection (40.16%), edge-plane crossing collection (26.41%), constrained
triangulation (17.85%), and exact GCD dispatch (17.71%). Exact 2D construction
after already-certified proper constraint crossings is the next clean
retained-work candidate; no future optimization may weaken its complete
symbolic or policy path.

The regenerated five-crate call graph has 14,689 function nodes and 24,441
edges at
`/tmp/hypermesh-retained-planar-radial-topology-callgraph-2026-08-03`.
Hypercurve and HyperSolve are excluded.

## Validation

Hypermesh passes 123 unit tests, 8 Boolean tests, 8 executed competitive tests,
11 manifest tests, 2 intersection-corpus tests, 9 policy tests, and 2 README
tests: 163 executed tests with six documented ignores. The focused retained
cycle test performs 96 complete-sort equivalence comparisons across both
policies and two exact embeddings.

The crate also passes no-default checking, warning-denied all-target/all-
feature Clippy, warning-denied rustdoc, every fuzz-target check, benchmark
compilation, formatting, the complete native/WASM size harness, the ignored
full-resolution oracle, both-policy full and wide direct heap probes, dispatch
trace, and diff checks.

## Open work

The full case remains 26.22x slower and 12.38x larger in RSS than the
established CGAL EPECK row. Exact boxes, every wide-rational row, and both
torus rows remain absolute runtime losses. Peak live memory is unchanged even
though churn falls sharply. External real-world/deeper-symbolic corpus work,
stage-specific lifetime attribution, broader current CGAL execution, and the
complete Phase 18 audit remain open.

## Reproduction

```sh
cargo test --all-features
cargo check --no-default-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --all-features --no-deps
cargo check --manifest-path fuzz/Cargo.toml --all-targets
cargo bench --no-run --all-features

YEAHRIGHT_BENCH=1 taskset -c 11 perf stat -r 5 -x, \
  -e cycles,instructions,branches,cache-misses \
  target/release/deps/competitive-92c64513605410c9 \
  --ignored --exact full_resolution_yeahright_rotated_intersection_certifies_empty

YEAHRIGHT_BENCH=1 target/release/examples/large_mesh_kernel_heap_probe \
  yeahright-full-rotated strict
YEAHRIGHT_BENCH=1 target/release/examples/large_mesh_kernel_heap_probe \
  yeahright-full-rotated approximate-512
target/release/examples/large_mesh_kernel_heap_probe wide-rational-2048 strict
target/release/examples/large_mesh_kernel_heap_probe \
  wide-rational-2048 approximate-512

benchmarks/size-harness/measure.sh

tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir /tmp/hypermesh-retained-planar-radial-topology-callgraph-2026-08-03 \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format dot,json,mermaid,graphml \
  --per-library
```
