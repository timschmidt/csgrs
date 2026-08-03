# Phase 13 checkpoint: dimension-complete pairwise intersections

Date: 2026-08-03

Implementation: `43f9c85385f34ca541e0afc554e69d6ae316a3fb`

Source parent: `e04b73d32a5eab14a61b2c7c32e68d7fa34ff2ac`
(the intervening `9c383dd1` commit contains evidence only)

Status: retained Phase 12–13 checkpoint. This is not completion of Phase 13,
the arrangement engine, EMBER removal, or CGAL parity.

## Decision

Retain the dimension-complete pairwise classifier, exact coplanar
separating-axis path, lower-dimensional retained events, provenance-identical
same-mesh feature elision, and polygon-inversion invariant fix.

The large general path removes 1,065,254,481 Callgrind instructions (4.4572%),
154,950,617 modeled branches (4.3392%), and 9,372,009 modeled mispredictions
(8.3258%) versus the exact fixed parent executable. The useful Massif maximum
grows 19,032 bytes (0.0663%); the certified path useful maximum is unchanged.
Maximum linked growth is 0.482% native and 0.396% after `wasm-opt -Oz`.
Performance has priority over size, and the additional representation is
required for path completeness, so the bounded size/heap growth is accepted
provisionally. Phase 16 still deletes the historical implementation and Phase
17 still owns recovery of all avoidable size and memory debt.

## Exact representation and semantics

The public intersection result is now one direct enum with six realizable
closed-set outcomes:

- `Disjoint`;
- `NonCoplanarPoint` and `NonCoplanarSegment`;
- `CoplanarPoint` and `CoplanarSegment`; and
- `CoplanarOverlap` for positive planar area.

The old independent `kind`, optional segment, and optional overlap fields could
represent contradictions and discarded exact point coordinates. They are
deleted. Every controlled caller was migrated directly; there is no adapter,
compatibility shim, deprecated duplicate, or alternate shipped API.

The retained graph uses the existing eight-byte directed event. Two high bits
encode coplanarity and dimension, the remaining 30 bits index the shared point
or segment arena, and `u32::MAX` denotes positive-area coplanar overlap. The
pending event remains 12 bytes. Segment endpoints and isolated exact-rational
points use the structural exact interner. Symbolic isolated points are appended
without adding a policy-sensitive equality decision.

Point and boundary-segment contacts are retained for the replacement engine,
but the current open-face consumer counts only non-coplanar segments and
positive-area coplanar overlaps as partition-changing. This prevents a tangent
contact from fabricating volume while later arrangement consumers are built.

Same-mesh contacts that exactly match a shared source vertex or edge identity
are not duplicated in the graph because the input topology already owns that
feature. Geometrically coincident features with distinct provenance remain
events. This is exact provenance filtering, not coordinate tolerance or a
topology heuristic.

## Coplanar path and inversion defect

The former allocation-heavy polygon clipping used only to answer whether two
convex coplanar interiors share positive area. It is replaced by the exact
convex separating-axis theorem over both polygons' edge planes. When positive
area is rejected, contained input vertices determine the remaining convex
point or collinear segment; unusual retained collinear cycles use exact
lexicographic extrema rather than discovery order.

The first bounded fuzz execution found a real independent contract defect with
input bytes `[121, 112, 104, 1, 0, 114, 126, 0]`: a positive-area pair of
oppositely oriented rectangles was reported as a segment.
`ConvexPolygon::inverted()` had inverted edge halfspaces even though the type
requires its interior on every edge's non-positive side. Inversion now reverses
support orientation and winding while preserving edge halfspaces. The exact
reproducer, a focused interior-halfspace unit test, the generalized rectangle
matrix, and 46,891 subsequent ASan/libFuzzer executions all pass.

## Hyperlimit policy contract

All geometry is over `hyperreal::Real`. Required comparisons go through the
operation-local decision context and Hyperlimit policy:

- `STRICT` returns `PredicateUndecided` if exact/certified proof does not decide;
- `APPROXIMATE_512` may consume only Hyperlimit's terminal 512-bit decision;
- `MeshOutcome` aggregates `Approximate512Consumed` when that terminal is used;
- exact rational cases remain `Certified` under both policies; and
- graph packing, remapping, and finalization make no scalar decision.

The permanent symbolic `pi + e` versus `e + pi` contact proves the distinction:
strict declines, approximate-512 returns `CoplanarPoint`, and its aggregate
certainty records terminal consumption.

## Permanent corpus

`tests/intersection_corpus.rs` adds:

- 12 exact triangle/quad cases spanning all six public variants;
- both policies, both operand orders, and both orientations: 192 deterministic
  class-matrix executions;
- 256 generated exact rectangle cases, each with eight inversion/order
  variants under both policies: 4,096 exact dimensional-oracle executions; and
- unordered exact segment-geometry and payload-index checks.

The corpus manifest now records the six classes, T-junctions, partial/full edge
contact, coincident area, crossing area without a contained vertex, symbolic
terminal equality, and orientation inversion. The polygon-predicate fuzz target
checks the same disjoint/point/segment/area oracle with exact geometry and
certainty. The registry remains monotonic; the fuzz failure was promoted into
the generalized deterministic path rather than discarded.

## Rejected naive retention

The first complete graph stored every same-mesh triangulation contact and used
the old coplanar construction path. Its general-path useful Massif maximum was
30,014,784 bytes, 1,307,392 bytes (4.5542%) above the 28,707,392-byte parent.
That design was rejected. Exact source-feature provenance filtering and the
allocation-free separating-axis proof reduce the final useful delta to 19,032
bytes (0.0663%) without removing topologically distinct coincident events.

## Large-fixture topology and heap

Both policy rows preserve certified topology:

| Probe | Input triangles | Output vertices | Output triangles | Certainty |
| --- | ---: | ---: | ---: | --- |
| `boxes-3072-general` | 6,144 | 2,410 | 4,816 | `Certified` |
| `boxes-3072` | 6,144 | 27 | 50 | `Certified` |

Direct Massif maxima (`--time-unit=B --detailed-freq=1`) are:

| Path/policy | Parent useful | Candidate useful | Useful delta | Candidate total |
| --- | ---: | ---: | ---: | ---: |
| General strict | 28,707,392 | 28,726,424 | +19,032 (+0.0663%) | 29,953,232 |
| General approximate-512 | 28,707,392 | 28,726,424 | +19,032 (+0.0663%) | 29,953,280 |
| Certified strict | 1,063,718 | 1,063,718 | 0 | 1,064,976 |
| Certified approximate-512 | 1,063,718 | 1,063,718 | 0 | 1,064,976 |

Versus the parent totals, general strict grows 17,584 bytes (0.0587%) and
general approximate-512 grows 18,304 bytes (0.0611%). Massif allocator
bookkeeping varies slightly between direct runs; useful bytes are the stable
carrier result.

## Deterministic performance

Fixed release executables were measured on CPU 11 of the Ryzen 7 5800X3D.
The parent SHA-256 is
`3cda9842a356a6112b4eccd0e04db5c9e82180e2efefbdd2cb9f5f3693f64465`;
the candidate is
`853caa6c409adb8b5b125ab609044a7394d2c3dad1bb28dce07906f530315241`.

| General strict Callgrind event | Parent | Candidate | Delta |
| --- | ---: | ---: | ---: |
| Instructions | 23,899,825,951 | 22,834,571,470 | -4.4572% |
| Total branches | 3,570,969,378 | 3,416,018,761 | -4.3392% |
| Mispredictions | 112,565,300 | 103,193,291 | -8.3258% |

Hardware counters independently place candidate instructions around 22.8295B
and parent instructions around 23.8830B, approximately 4.41% lower, with
approximately 3.28% fewer hardware branches. Wall-clock batches were not
stable enough for a universal speed claim: five-run internal deviation reached
13.65% during the final alternation. The deterministic reduction is retained;
no noisy clock percentage is promoted.

The certified large path moves from 14,973,358 to 14,987,994 Callgrind
instructions (+0.0977%) while branches fall 0.1326% and useful heap stays flat.
The dependency-only small exact union moves from 7,692,545 to 7,586,410
instructions (-1.3797%), with identical 10-polygon/10-vertex/16-triangle
output. These fixed-binary controls reject noisy Criterion regression labels.

## Historical and competitive gauge

The current shared exact overlapping-box Criterion centers were 8.1114 us,
1.7562 us, and 4.2271 us for Hypermesh union/intersection/difference. In the
same session Boolmesh reported 73.329/40.740/60.085 us and Manifold-rust
60.191/39.348/92.946 us. Hypermesh therefore remained 7.42–23.20x faster than
those non-exact competitors on these three rows.

The pinned CGAL 6.0.3 EPECK adapter reproduced valid, closed, structurally
valid exact outputs from the same OFF pair. Copy-outside medians were 156.159
us union, 149.119 us intersection, 157.659 us difference, and 154.050 us
reverse-difference. The separately compiled harnesses make ratios indicative,
but Hypermesh is ahead on this favorable convex exact-cell case.

On the 3,072-triangle-per-operand certified union, current centers were 780.22
us Hypermesh, 8.1325 ms Boolmesh, and 4.6426 ms Manifold-rust: 10.42x and 5.95x
advantages. The session was noisy—unrelated competitor changes ranged from
-5% to +87%—while fixed-binary certified Callgrind moved only +0.098%, so the
historical 706.69-us center is retained as context, not treated as a revision
regression.

The pinned full-resolution YeahRight comparison remains the opposite extreme:
Hypermesh's certified 3,312.66-second result versus about 0.09 seconds for CGAL
EPECK, with a 21.23x RSS deficit. This checkpoint neither reruns that
multi-hour unchanged output path nor claims CGAL parity. Per-case parity or
superiority remains the explicit Phase 17 gate.

## Native and WASM size

The dependency-only harness was clean-built with rustc 1.97.0. Native values
are linked `.text`; WASM values are after `wasm-opt -Oz`.

| Consumer/profile | Parent | Candidate | Delta |
| --- | ---: | ---: | ---: |
| General release native | 4,091,844 | 4,111,564 | +19,720 (+0.4819%) |
| General release WASM | 2,760,635 | 2,770,580 | +9,945 (+0.3602%) |
| Immediate release native | 4,125,060 | 4,144,780 | +19,720 (+0.4781%) |
| Immediate release WASM | 2,775,185 | 2,785,134 | +9,949 (+0.3585%) |
| General size native | 1,885,330 | 1,891,578 | +6,248 (+0.3314%) |
| General size WASM | 1,178,835 | 1,183,507 | +4,672 (+0.3963%) |
| Immediate size native | 1,897,478 | 1,903,606 | +6,128 (+0.3230%) |
| Immediate size WASM | 1,188,976 | 1,193,551 | +4,575 (+0.3848%) |

Production source grows by 963 insertions and 234 deletions in this checkpoint;
test/corpus/fuzz/benchmark evidence grows by 595 insertions and 35 deletions.
The representation is deliberately singular and compact, but this is still
provisional growth before historical-engine deletion.

## Call graph and validation

The workspace utility, restricted to Hyperreal, Hyperlattice, Hyperlimit,
Hypertri, and Hypermesh, reports:

| Graph | Nodes | Edges |
| --- | ---: | ---: |
| Production | 20,113 | 40,129 |
| Tests/bench/examples/fuzz evidence | 26,570 | 50,146 |

The production route is one streamed candidate admission into
`intersect_polygons_with_vertices`, one exact coplanar/non-coplanar classifier,
one checked graph builder, and direct consumers. Search and graph inspection
show no old public type, compatibility adapter, or second graph representation.

Final gates passed:

- default and no-default: 1,216 passed, zero failed, seven ignored;
- all features: 1,218 passed, zero failed, seven ignored;
- formatting, Clippy all-target/all-feature `-D warnings`, and rustdoc
  all-feature `-D warnings`;
- default and all-feature benchmark compilation, including dispatch trace;
- all fuzz binaries and a 46,891-execution ASan/libFuzzer campaign;
- minimal/all-feature `wasm32-unknown-unknown` checks;
- eight native UI tests and locked release Trunk build;
- both-policy large topology and Massif runs;
- native/WASM size harnesses, fixed-binary Callgrind, competitive Criterion,
  pinned CGAL EPECK, and five-crate call graphs; and
- `git diff --check`.

## Remaining Phase 13 work

The classifier still needs the replacement engine's canonical incidence
records and fully lazy construction recipes for transverse, tangent,
shared-feature, and coplanar-overlay scheduling. Lower-dimensional retained
facts are not yet consumed by a completed radial arrangement. Phases 14–16
must build coplanar corefinement, radial/cell topology, exact winding output,
and then atomically remove subdivision, segment trace, local BSP, and EMBER
entry/configuration code. No completion or CGAL-parity claim is made here.

## Reproduction

```sh
cargo test --locked
cargo test --locked --no-default-features
cargo test --locked --all-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features
cargo bench --locked --no-run --all-features
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo check --locked --target wasm32-unknown-unknown --no-default-features
cargo check --locked --target wasm32-unknown-unknown --all-features
ASAN_OPTIONS=detect_leaks=0 cargo +nightly fuzz run polygon_predicates \
  --fuzz-dir fuzz -- -max_total_time=30
valgrind --tool=massif --time-unit=B --detailed-freq=1 \
  --massif-out-file=/tmp/hypermesh-pathcomplete-final.massif \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
valgrind --tool=callgrind --cache-sim=no --branch-sim=yes \
  --callgrind-out-file=/tmp/hypermesh-pathcomplete-final.callgrind \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
HYPERMESH_SIZE_TARGET_DIR=/tmp/hypermesh-pathcomplete-size \
  benchmarks/size-harness/measure.sh default
tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir hypermesh/target/hypermesh-ember-phase13-path-complete-pairwise-production \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format json,dot \
  --per-library
```
