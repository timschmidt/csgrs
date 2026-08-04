# Phase 17 retained crossing identities and shared planar carrier

Captured 2026-08-03 on the established Ryzen 7 5800X3D / CPU 11 protocol.
The implementation spans Hyperlattice `a475bb75`, Hyperlimit `6e4d68c8`,
Hypertri `3b436192`, and Hypermesh `e2bdff30`; the controlled Hyperpath caller
is migrated directly at `e6550627`. Hyperreal remains at `c2217bae`.

This checkpoint removes two forms of repeated exact work from the one
path-complete surface-arrangement engine:

1. Hyperlimit now separates policy-aware finite-segment classification from
   policy-free exact supporting-line construction. Callers that have already
   certified a proper crossing no longer classify the same pair again.
2. Hypermesh gives each proper crossing a canonical operation-wide geometric
   identity. Once one source face constructs the exact 3-D point, another face
   meeting the same crossing reuses that retained point and projection instead
   of reconstructing equal `Real` coordinates.

Hypertri also now reexports the canonical `hyperlattice::Point2` rather than
maintaining a byte-identical private carrier. This removes coordinate clones
at every Hypertri-to-Hyperlimit predicate call and two temporary indexed-ring
vectors. Serde is forwarded through the existing optional feature.

These are general retained-fact and ownership improvements. No code recognizes
a fixture, coordinate range, scalar width, topology, triangle count, Boolean
operation, expected output, or policy. There is no retry, compatibility shim,
second engine, hidden work limit, or CGAL-shaped shortcut. Phase 17 and Phase
18 remain open.

## Completeness and policy semantics

Finite-segment topology remains owned by Hyperlimit's `DecisionContext`.
`STRICT` may use only certified exact decisions; `APPROXIMATE_512` may resolve
an otherwise undecided comparison only through Hyperlimit's terminal 512-bit
policy. Exact supporting-line construction follows classification and is
therefore deliberately policy-free.

The retained crossing key is valid independently of face traversal order:

- a source edge and an intersecting split plane identify one
  `SourceEdgePlane` point;
- three independent split planes identify one `PlaneTriple` point; and
- the canonical pair of generic supporting edges identifies their unique line
  intersection.

The arena is consulted only after the current finite segments have been
certified as a proper crossing. A miss still performs exact 2-D construction,
lifts into the source face, validates the result against both supports, and
inserts it through the existing canonical point arena. Parallel, collinear,
endpoint, lower-dimensional, degenerate, and undecided cases keep their
complete paths. A focused regression proves that three incident source faces
share one exact triple point and its constraints.

All measured exact workloads are byte-identical between `STRICT` and
`APPROXIMATE_512`, remain `Certified`, and do not consume the approximate
terminal. Dispatch tracing records zero unknown-fact and zero
fallback-or-abort events.

## Full-resolution performance

Five fresh-process repetitions per row include fixture preparation, exact
import, PWN work, one 23,788-triangle rotated YeahRight intersection, result
validation, and destruction. Runs are serialized and pinned to CPU 11. The
parent is the retained-planar-radial checkpoint.

| Metric | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| Median test time | 2.36 s | 2.18 s | -7.63% |
| Cycles | 9,311,200,332 | 8,528,856,984 | -8.40% |
| Instructions | 24,604,987,622 | 22,237,885,130 | -9.62% |
| Branches | 4,371,171,205 | 3,988,513,749 | -8.75% |
| Current cache misses | — | 20,080,741 | — |

The exact result remains empty and `Certified`. The current row is about
1,519.6x faster than historical EMBER's 3,312.66 seconds, but still 24.22x
slower than the established 0.09-second CGAL EPECK row. The competitive full
case therefore remains explicitly open.

## Large-fixture heap and RSS

The direct allocator boundary excludes fixture preparation, keeps input meshes
alive through the Boolean, and drops output and inputs separately. Both
policies have identical byte and call counts.

| Metric | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| Incremental kernel peak | 158,258,204 B | 158,258,204 B | unchanged |
| Allocation calls | 28,631,893 | 26,006,589 | -9.17% |
| Reallocation calls | 3,113,322 | 2,525,282 | -18.89% |
| Allocated-byte churn | 1,447,448,648 B | 1,279,258,296 B | -11.62% |
| Input-attached fact growth | 24,389,848 B | 24,389,848 B | unchanged |
| Output-live payload | 56 B | 56 B | unchanged |
| Post-input residual | 10,792 B | 10,792 B | unchanged |

Compared with the earlier exact-linear-arithmetic checkpoint, allocation calls
are down 33.61% and allocated-byte churn is down 47.70%. The current process
reaches 192,196 KiB maximum RSS with zero swaps. That is still 12.39x CGAL's
established 15,516 KiB row, and the identical peak shows that shorter-lived
temporaries alone do not close the live-memory target.

The 2,049-bit, 6,144-triangle wide-rational union is also policy-identical:
31,038,074 peak bytes, 2,107,571 allocations, 227,677 reallocations,
462,626,870 allocated bytes, 26,568 input-attached fact bytes, 520,472 output
bytes, and 96,192 post-input residual bytes.

## Independent controls and CGAL

The controls demonstrate general behavior rather than one hard-case win:

| Workload | Parent instructions | Current instructions | Movement |
| --- | ---: | ---: | ---: |
| 2,049-bit wide rational, 6,144 triangles | 4,195,599,349 | 4,190,199,389 | -0.13% |
| Clipped voxel torus, 6,412 triangles | 1,303,920,505 | 1,246,596,747 | -4.40% |
| Clipped voxel torus, 25,100 triangles | 5,592,161,042 | 5,334,327,874 | -4.61% |
| Dense coplanar, 6,144 triangles | 3,616,449,330 | 3,268,905,691 | -9.61% |

Three independent 101-call exact-box aggregates give a 674.542 microsecond
median for `STRICT` and 734.314 microseconds for `APPROXIMATE_512`. Against
the pinned CGAL copy-outside/copy-inside rows of 119.5965/128.976
microseconds, `STRICT` remains 5.64x/5.23x slower.

Three independent 31-call medium dense-coplanar aggregates give 14.974266 ms
for `STRICT` and 14.979057 ms for `APPROXIMATE_512`. Both are approximately
6.6% faster than pinned CGAL's 16.036 ms. This closes the medium member's
runtime gate without changing its algorithm; its large and XL siblings were
already faster than CGAL.

## Linked size

Deleting duplicate carriers and adapters more than pays for the retained-point
lookup. Every canonical linked artifact shrinks from the radial checkpoint.

| Profile / consumer | Parent native | Current native | Movement | Parent WASM | Current WASM | Movement |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| release / general | 2,022,254 B | 2,019,526 B | -2,728 B (-0.135%) | 1,439,446 B | 1,437,843 B | -1,603 B (-0.111%) |
| release / immediate | 2,025,398 B | 2,022,670 B | -2,728 B (-0.135%) | 1,441,304 B | 1,439,697 B | -1,607 B (-0.111%) |
| size / general | 1,085,687 B | 1,082,351 B | -3,336 B (-0.307%) | 680,458 B | 679,625 B | -833 B (-0.122%) |
| size / immediate | 1,086,635 B | 1,083,315 B | -3,320 B (-0.306%) | 680,869 B | 680,032 B | -837 B (-0.123%) |

## Profile and call graph

The frame-pointer profile at
`/tmp/hypermesh-retained-crossing-shared-point-fp.data` contains 2,251 samples
with none lost. Inclusive ownership is led by surface construction (85.67%),
pairwise intersection (44.50%), BVH self-pair traversal (44.11%), pair append
(40.15%), polygon intersection (37.93%), face corefinement (35.24%), and
edge-plane crossing collection (28.54%). Constrained triangulation is 17.15%,
the 8-by-3 signed-product ordering fallback is 11.37%, exact line construction
is 3.92%, and point-on-segment classification is 4.65%.

The next clean target is therefore retained scheduling inside general
edge-plane crossings, especially repeated exact signed-product work. Any
change must preserve the generic decline paths and both Hyperlimit policies.

The regenerated five-crate call graph has 14,672 function nodes and 24,403
edges at
`/tmp/hypermesh-retained-crossing-shared-point-callgraph-2026-08-03`.
Hypercurve and HyperSolve are excluded.

## Validation and open work

Hyperlattice, Hyperlimit, Hypertri, and Hypermesh pass their all-feature tests,
no-default checks, warning-denied Clippy and rustdoc, formatting, fuzz-target
checks, and relevant docs. Hypermesh passes 163 executed tests with six
documented ignores, benchmark compilation, both-policy full and wide heap
probes, the complete native/WASM size harness, dispatch tracing, and the
ignored full-resolution oracle.

Hyperpath's controlled caller was migrated directly with no deprecated alias.
Its full build currently stops before that source at a concurrently changing,
out-of-scope HyperSolve `primitive_integer_ratio` call-site mismatch. No
Hypercurve or HyperSolve source was touched.

The full CGAL runtime and RSS deficits, exact-box and wide-rational absolute
losses, peak-live-memory ownership, external real-world/deeper-symbolic corpus,
stage-specific lifetime attribution, current broad CGAL confidence runs, and
the complete Phase 18 path audit remain open.

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
  --out-dir /tmp/hypermesh-retained-crossing-shared-point-callgraph-2026-08-03 \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format dot,json,mermaid,graphml \
  --per-library
```
