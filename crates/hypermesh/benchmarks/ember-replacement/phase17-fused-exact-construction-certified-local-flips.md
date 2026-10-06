# Phase 17 fused exact construction and certified local-flip checkpoint

Captured 2026-08-03 on the established Ryzen 7 5800X3D / CPU 11 protocol.
This is a retained Phase 17 optimization checkpoint. It does not claim Phase
17 corpus completion, Phase 18 completion, or CGAL EPECK parity.

## Result

The surface-arrangement Boolean engine now retains exact edge/support values
through edge-plane containment and construction, forms exact-rational
intersection coordinates only after the topology requires them, and reuses the
certified local topology of a flippable CDT diagonal instead of rescanning the
entire face point/constraint set.

Relative to the preceding retained-radial checkpoint, the 11,894-by-11,894
YeahRight exact-empty case falls from 4.82 seconds and 50.394 billion
instructions to 3.38 seconds and 36.922 billion instructions. That is 29.88%
less wall time and 26.73% fewer instructions with the same empty mesh and
`Certified` certainty. The new engine is now 980.08x faster than the historical
EMBER row, while the remaining historical CGAL EPECK gap is 37.56x.

No fixture name, coordinate value, triangle count, operation, expected result,
benchmark state, or competitor selects any path in this checkpoint. Dispatch
depends only on retained `Real` facts, exact construction identity, certified
predicate results, and valid triangulation topology.

## Exact construction cascade

Proper 3D edge-plane crossings now follow one clean cascade:

1. Retain the two already-required endpoint support values.
2. Classify polygon containment from the implicit intersection ratio before
   constructing an affine point.
3. For exact-rational edge/support data, classify the complete eight-term,
   three-factor edge numerator by sign without materializing a dead `Real`.
4. If containment accepts an exact-dyadic crossing, construct all three
   coordinates from the retained parameter ratio with one complete numerator
   and one quotient reduction per coordinate.
5. If implicit containment is genuinely undecided, construct the point and
   retain the former affine and projective fallbacks. If all of them remain
   undecided, return the original typed policy failure.

Hyperlimit proper 2D segment-crossing construction uses the corresponding
layered schedule after its unchanged four-orientation classifier certifies a
proper crossing:

- fixed-stack exact dyadic;
- fixed-wide exact dyadic;
- arbitrary exact-rational fused point construction;
- unchanged general symbolic `Real` arithmetic.

The arbitrary exact-rational kernel expands the two line determinants and each
affine coordinate polynomial before reducing them. It does not replace the
general symbolic path and cannot decide topology; it is only a construction
schedule after Hyperlimit has already certified the crossing.

## Certified local flips

Hypertri previously ran a global point-on-segment scan and a global
new-edge/constraint intersection scan after every diagonal had already passed
four strict orientation tests. Those tests certify a convex quadrilateral, so
the replacement diagonal lies inside the union of the two adjacent triangles.
Because every input point is represented by the triangulation and every
recovered constraint is a triangulation edge, the local certificate proves that
the replacement cannot pass through another point or cross a protected edge.

The redundant scan and its `flip_preserves_constraints` function are removed.
Exact PSLG input validation, local convexity predicates, constraint recovery,
and the full final constrained-topology validator remain. The approximate point
buffer used during recovery is now dropped before canonicalization/legalization
instead of being retained solely for the deleted scan. This change deletes 103
net Hypertri source lines.

The retained-topology step alone changes the hard case from 3,694.69 ms and
41,713,058,428 instructions to 3,380.23 ms and 36,921,998,673 instructions:
8.51% less task clock and 11.49% fewer instructions.

## Policy and path coverage

- Exact-rational work remains `Certified` under both `STRICT` and
  `APPROXIMATE_512`.
- Symbolic edge-numerator coverage proves that `STRICT` returns
  `PredicateUndecided` while `APPROXIMATE_512` records
  `Approximate512Consumed` at the same terminal predicate.
- Hyperlimit tests run a non-dyadic exact-rational proper crossing under both
  policies and require identical exact coordinates and exact certainty.
- The exact edge numerator is differentially checked over 512 generated cases.
- The arbitrary exact-rational 2D line kernel is differentially checked over
  512 generated non-dyadic line pairs, plus parallel and symbolic decline
  cases.
- The exact-dyadic 3D interpolator is checked over 256 generated cases and a
  zero-denominator failure.
- Hyperreal's exact fuzz target compares both new kernels with their expanded
  arithmetic oracles.
- Hypertri's generated constrained-CDT tests validate recovered constraints and
  complete final topology under both policies after the global rescan removal.

## Validation

At Hyperreal `bf31dea557880036ea172472bcba094fe7c0698b`:

- 704 unit/integration tests and 24 documentation tests pass;
- the GMP API coverage gate and the new fuzz-bin check pass;
- all-target/all-feature Clippy, warning-denied rustdoc, formatting, and diff
  checks pass.

At Hyperlimit `e9d58cf4ae46a29d35884a9fb50e91a1be3a526b`:

- 152 unit and 75 integration tests pass (227 total);
- all-target/all-feature Clippy, warning-denied rustdoc, every fuzz-bin check,
  formatting, and diff checks pass.

At Hypertri `513433d3c427f8f9a917875f4bbdde6a00231cfa`:

- 72 unit and 48 integration tests pass (120 total), plus four documentation
  tests;
- the generated constraint-recovery and topology-validation campaigns pass;
- all-target/all-feature Clippy, warning-denied rustdoc, every fuzz-bin check,
  formatting, and diff checks pass.

At Hypermesh `e8857586113a6967985113bf26f4ac4bc690937a`:

- 112 unit, 8 Boolean, 5 executed competitive, 6 corpus, 2 intersection, 9
  policy, and 2 README tests pass (144 executed total);
- six documented opt-in/manual competitive tests remain ignored by the normal
  suite, and the full ignored exact-empty oracle passes separately;
- all-target/all-feature Clippy, warning-denied rustdoc, every fuzz-bin check,
  formatting, and diff checks pass.

The six-crate production call graph contains 18,119 nodes and 29,662 edges. It
contains the Hypermesh-to-Hyperreal dyadic interpolation route, the
Hyperlimit-to-Hyperreal arbitrary exact-rational line-point route, and the
Hypertri local flippability predicate. The removed global flip-preservation
scan is absent. Hypercurve and HyperSolve were neither included nor edited.

## Full-resolution pathological case

Every repetition intersects the 11,894-triangle YeahRight control mesh with a
rotated copy under `APPROXIMATE_512` and returns an empty mesh with `Certified`
certainty.

| Row | Wall | Task clock | Instructions | Maximum RSS |
| --- | ---: | ---: | ---: | ---: |
| Retained radial/sign-only checkpoint | 4.82 s | 4,766.29 ms | 50,394,157,929 | 201,544 KiB |
| Fused exact construction, before local topology reuse | 3.70 s | 3,694.69 ms | 41,713,058,428 | 192,244 KiB |
| Fused construction plus certified local flips | 3.38 s | 3,380.23 ms | 36,921,998,673 | 192,376 KiB |

The final three-repetition pinned mean also reports 13,643,808,314 cycles,
6,309,879,530 branches, 66,118,865 branch misses, and 30,630,212 cache misses.
The unprofiled row used 3.19 seconds user time, 0.17 seconds system time, no
major faults, and no swaps.

Maximum RSS is 4.55% below the preceding checkpoint and 41.59% below
historical EMBER. It remains 12.40x the historical CGAL row's 15,516 KiB.

The final frame-pointer profile is led by:

| Production subtree | Children samples |
| --- | ---: |
| Pairwise intersection traversal | 33.24% |
| Face corefinement | 29.06% |
| Edge-plane crossing collection | 14.71% |
| Hyperlimit `orient3d` | 17.76% |
| Radial ring assembly | 17.18% |
| Topology-only CDT | 12.57% |
| Hyperlimit `orient2d` | 12.04% |

The deleted global flip scan was 9.51% of the preceding profile. The profile
now points back toward pairwise/radial exact predicates and `orient3d`, not a
replacement triangulation algorithm.

## Competitive exact boxes

All four shared-arrangement outputs retain the established exact volumes and
triangle counts. The CGAL rows are the unchanged pinned CGAL 6.0.3 EPECK
measurements from the shared adapter.

| Engine / policy | Median | Ratio to CGAL copy outside | Ratio to CGAL copy inside |
| --- | ---: | ---: | ---: | ---: |
| Hypermesh `STRICT` | 892.71 us | 7.46x | 6.92x |
| Hypermesh `APPROXIMATE_512` | 898.06 us | 7.51x | 6.96x |
| CGAL 6.0.3 EPECK, copy outside | 119.5965 us | 1.00x | — |
| CGAL 6.0.3 EPECK, copy inside | 128.9760 us | — | 1.00x |

This improves the Hypermesh medians 6.50% and 4.70% from the preceding
checkpoint. Small-case parity remains open.

The two new scalar rows are also present in Hyperreal's GMP/MPFR harness. Exact
dyadic 3D interpolation measures 339.31 ns versus 211.65 ns for 128-bit MPFR;
the arbitrary exact-rational 2D line point measures 894.57 ns versus 431.78 ns.
These are not like-for-like exactness levels: Hyperreal returns unbounded exact
rationals, while the comparator is fixed 128-bit MPFR. The rows are retained to
make the local cost visible; the mesh wins come from avoiding multiple larger
exact intermediate reductions.

## Permanent 6,144-triangle runtime and heap control

Every row returns the same `Certified` 2,410-vertex/4,816-triangle union.
Eleven pinned repetitions report:

| Input path | Policy | Task clock | Instructions |
| --- | --- | ---: | ---: |
| Native retained | `STRICT` | 157.43 ms | 1,662,403,460 |
| Native retained | `APPROXIMATE_512` | 155.99 ms | 1,662,193,173 |
| Raw/general | `STRICT` | 143.77 ms | 1,535,534,493 |
| Raw/general | `APPROXIMATE_512` | 143.98 ms | 1,535,546,632 |

The deterministic instruction changes versus the preceding checkpoint range
from +0.0035% to -0.0249%; this fixture is effectively neutral while the
pathological crossing-heavy fixture improves substantially.

Sequential Heaptrack recordings report:

| Input path | Policy | Allocations | Recorder temporary | Peak heap | Heaptrack RSS |
| --- | --- | ---: | ---: | ---: | ---: |
| Native retained | `STRICT` | 323,213 | 84,886 | 16,497,378 B | 24.00 MB |
| Native retained | `APPROXIMATE_512` | 323,213 | 84,886 | 16,497,378 B | 23.97 MB |
| Raw/general | `STRICT` | 286,331 | 84,886 | 16,497,738 B | 23.79 MB |
| Raw/general | `APPROXIMATE_512` | 286,331 | 84,886 | 16,497,738 B | 24.08 MB |

Native peak heap is byte-identical. General peak heap increases by 352 bytes
(0.0021%); allocation counts are unchanged.

## Linked and source size

Performance is the priority, but all canonical native and WASM rows were
measured. Relative to the preceding checkpoint, release native `.text` grows
3.92–3.98% and optimized release WASM grows 5.34–5.39%. Size-profile native
`.text` grows 2.96–3.03% and optimized size-profile WASM grows 4.25–4.35%.

| Features/profile/consumer | Native `.text` | Delta | Optimized WASM | Delta |
| --- | ---: | ---: | ---: | ---: |
| default/release/general | 2,033,026 | +3.923% | 1,441,079 | +5.388% |
| default/release/immediate | 2,036,178 | +3.916% | 1,442,929 | +5.380% |
| default/size/general | 1,077,231 | +3.025% | 675,045 | +4.253% |
| default/size/immediate | 1,078,179 | +3.022% | 675,453 | +4.252% |
| all/release/general | 2,175,799 | +3.979% | 1,523,884 | +5.346% |
| all/release/immediate | 2,178,647 | +3.974% | 1,525,861 | +5.339% |
| all/size/general | 1,078,199 | +2.961% | 675,798 | +4.348% |
| all/size/immediate | 1,079,155 | +2.957% | 676,070 | +4.346% |

The size growth is material and remains an explicit optimization target. It is
retained at this checkpoint because the same clean general schedules remove
26.73% of hard-case instructions and because the topology change simultaneously
deletes 103 net source lines. There is still one Boolean engine and no
compatibility layer.

Current Tokei code counts are 46,616 Hyperreal, 18,059 Hyperlimit, 8,366
Hypertri, and 15,567 Hypermesh lines. The implementation commits add 424
Hyperreal lines (including tests, fuzz, and GMP rows), 88 Hyperlimit lines,
delete 103 net Hypertri lines, and add 180 net Hypermesh lines.

## Rejected experiments

- Returning and materializing a general parameter before 3D point construction
  was slower than constructing only the demanded point.
- A delta-based dyadic affine form was slower than the direct endpoint form.
- Eagerly caching every triangle support value increased hard-case instructions
  from 42.555B to 42.636B (+0.188%) and was removed.
- Extending retained radial separation into higher-degree rings produced zero
  safe hits and remains removed.

## Open work

CGAL parity is not reached. Phase 17 still needs the remaining exhaustive,
pathological, scaling, competitive, and kernel-lifetime heap corpus expansion.
The current profile supports another clean retained-fact/sign-only pass through
pairwise/radial predicates and `orient3d`, followed by size recovery that does
not give back material performance. Phase 18 must still audit every exit
condition and completion gate.

## Reproduction

```sh
cargo test --all-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS=-Dwarnings cargo doc --no-deps --all-features
cargo +nightly check --manifest-path fuzz/Cargo.toml --bins
cargo fmt --all -- --check

taskset -c 11 cargo bench --bench competitive -- \
  competitive_shared_arrangement/hypermesh/overlapping_boxes
taskset -c 11 perf stat -r 11 -x, \
  -e task-clock,cycles,instructions,branches,branch-misses,cache-misses \
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
  --out-dir hypermesh/target/phase17-fused-topology-callgraph \
  --format json \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh,csgrs \
  --per-library
```
