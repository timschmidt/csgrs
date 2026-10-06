# Phase 12/13 checkpoint: compact polygon vertex arena

Status: retained

Hypermesh implementation: `dcc16bfd03bcc2ea15270a84fa7416ef5d1f8f1a`

Direct Hypermesh parent: `3057c2f8`

Shared Hyperlimit implementation: `bb1c5e7c418bb8af25701c8201f833fa485b9e9a`

This checkpoint replaces the pairwise classifier's `Vec<Vec<Point3>>` face
vertex cache with one contiguous point arena and checked 32-bit face offsets.
It is compact arrangement/intersection substrate, not a claim that Phase 12,
Phase 13, EMBER removal, or the CGAL target is complete.

## Representation, exactness, and failure contract

`PolygonVertexArena` performs a checked face and total-point census, reserves
both allocations before materialization, stores every face range as adjacent
`u32` offsets, and validates every row range before returning a slice. Values
above the compact index domain, arithmetic overflow, allocation failure, an
invalid face, a reversed/out-of-bounds range, or derived-vertex failure returns
a typed error with no arena escaping. There is no unchecked narrowing.

Retained vertices are cloned in their existing cycle order. A polygon without
retained vertices is materialized in the same index order as
`vertices_decision`, through the operation's existing `DecisionContext`.
Consequently the arena adds no equality or topology decision: `STRICT` remains
unbounded certified evaluation, `APPROXIMATE_512` still terminates only through
Hyperlimit's approximate 512-bit policy, and aggregate certainty is unchanged.
The BVH callback and exact polygon classifier borrow the arena rows directly;
there is no per-face adapter, compatibility shim, alternate wide
representation, or dual engine.

For 6,144 triangular faces, the point payload remains 18,432 `Point3` values
(884,736 bytes on x86-64), while 6,144 inner `Vec` headers (147,456 bytes) are
replaced by 6,145 `u32` offsets (24,580 bytes). The live vertex-cache carrier
therefore loses exactly 122,876 useful bytes, or 11.904% of its complete
header-plus-point payload, and removes the separate ownership/allocation of
every face row.

Physical source in `intersection.rs` grows by 81 production lines and 53 test
lines. This is accepted provisionally because all release consumers shrink,
and Phase 16 deletes the much larger historical subdivision/trace/BSP source.

## Correctness and path evidence

Default and minimal-feature suites each passed with 1,079 library tests, 4
active competitive tests (6 opt-in tests ignored), 59 core tests, 4 corpus
tests, 8 predicate-policy tests, 2 README tests, and 48 regression tests (1
benchmark test ignored). All features passed 1,080 library and 60 core tests
plus the same integration rows. The arena test covers retained, empty, and
policy-materialized face rows; preserves numeric geometry and face ordering;
checks compact offset layout; and rejects ordinary, overflowing, and corrupted
row lookups.

Formatting, warning-denied all-target/all-feature Clippy and rustdoc, benchmark
compilation, every fuzz binary, WASM minimal/all-feature checks, clean
native/WASM size builds, and the release Trunk demo passed.

Both permanent large fixtures and both runtime policies produced identical
certified topology:

| Fixture | Policy | Input triangles | Certainty | Output vertices | Output triangles |
| --- | --- | ---: | --- | ---: | ---: |
| `boxes-3072-general` | `STRICT` | 6,144 | `Certified` | 2,410 | 4,816 |
| `boxes-3072-general` | `APPROXIMATE_512` | 6,144 | `Certified` | 2,410 | 4,816 |
| `boxes-3072` | `STRICT` | 6,144 | `Certified` | 27 | 50 |
| `boxes-3072` | `APPROXIMATE_512` | 6,144 | `Certified` | 27 | 50 |

The regenerated five-crate production graph contains 20,022 nodes and 39,960
edges; the graph including tests, examples, benches, and fuzz targets contains
26,440 nodes and 49,906 edges. It shows one production owner:
`pairwise_intersections_by_polygon_with_certified_embedded_inputs` builds the
arena, and the streamed candidate callback reads checked rows. Hypercurve and
HyperSolve were excluded and untouched.

## Large-fixture heap and allocations

Valgrind 3.27.0 Massif used `--time-unit=B --detailed-freq=1`. Process-wide
useful-heap maxima are flat because the face-vertex arena is released before a
later subdivision/output peak. The exact live carrier reduction above applies
during the intersection stage; allocator-overhead changes at the later maximum
are below 624 bytes and change sign by policy, so they are noise rather than a
peak-heap claim.

| Fixture/policy | Revision | Useful heap (B) | Extra heap (B) | Total heap (B) |
| --- | --- | ---: | ---: | ---: |
| general / `STRICT` | parent | 28,739,660 | 1,227,708 | 29,967,368 |
| general / `STRICT` | vertex arena | 28,739,660 | 1,227,084 | 29,966,744 |
| general / `APPROXIMATE_512` | parent | 28,739,660 | 1,227,692 | 29,967,352 |
| general / `APPROXIMATE_512` | vertex arena | 28,739,660 | 1,227,996 | 29,967,656 |
| certified / `STRICT` | parent | 1,063,718 | 1,258 | 1,064,976 |
| certified / `STRICT` | vertex arena | 1,063,718 | 1,258 | 1,064,976 |
| certified / `APPROXIMATE_512` | parent | 1,063,718 | 1,258 | 1,064,976 |
| certified / `APPROXIMATE_512` | vertex arena | 1,063,718 | 1,258 | 1,064,976 |

Heaptrack reports 844,759 parent versus 838,605 candidate allocation calls,
a reduction of 6,154 (0.7285%), with 34,039 temporary allocations in each.
Its rounded peak heap remains 28.81 MB; peak RSS including profiler overhead
moves from 40.26 MB to 38.60 MB. RSS is supporting evidence only because that
single profiler run has no confidence interval.

## Runtime

Six five-process `perf stat` batches per revision were pinned to CPU 11 for the
`STRICT` general fixture (30 processes per revision). The fixed parent binary
was sampled before, between, and after candidate experiments. One additional
parent batch with 11.55% internal task-clock deviation was excluded; including
it would make the candidate look artificially better. Every retained batch had
at most 3.36% internal deviation.

| Revision | Task clock (ms) | Cycles | Instructions | Branches | Branch misses |
| --- | ---: | ---: | ---: | ---: | ---: |
| parent | 1,949.100 | 7,972,275,711 | 23,881,142,170 | 4,604,487,211 | 13,630,737 |
| vertex arena | 1,924.403 | 7,887,936,419 | 23,896,746,787 | 4,633,286,121 | 14,237,004 |
| change | -1.2671% | -1.0579% | +0.0653% | +0.6255% | +4.4478% |

The address/allocation-sensitive hardware counters get worse while actual
task clock and cycles improve. This is reported rather than hidden. Callgrind
with branch simulation provides the deterministic-work control: it traces
23,902,510,549 parent versus 23,900,826,011 candidate instructions, a reduction
of 1,684,538 (0.0070%). Total branches fall 303,148 (0.0085%), and modeled
mispredictions fall 527,598 (0.4678%). Together with fewer allocations and
better locality, the real clock/cycle result supports retention.

## Native and WASM size

The canonical dependency-only harness was clean-built with rustc 1.97.0.
Native code is linked `.text`; WASM is `wasm-opt -Oz`.

| Consumer/profile | Parent (B) | Vertex arena (B) | Change |
| --- | ---: | ---: | ---: |
| General release native `.text` | 4,085,052 | 4,081,972 | -3,080 (-0.0754%) |
| General release optimized WASM | 2,757,749 | 2,754,491 | -3,258 (-0.1181%) |
| Immediate release native `.text` | 4,118,284 | 4,115,188 | -3,096 (-0.0752%) |
| Immediate release optimized WASM | 2,772,304 | 2,769,105 | -3,199 (-0.1154%) |
| General size native `.text` | 1,883,674 | 1,883,746 | +72 (+0.0038%) |
| General size optimized WASM | 1,176,941 | 1,177,387 | +446 (+0.0379%) |
| Immediate size native `.text` | 1,895,822 | 1,895,902 | +80 (+0.0042%) |
| Immediate size optimized WASM | 1,187,078 | 1,187,530 | +452 (+0.0381%) |

All release consumers shrink. Size-profile growth is at most 0.0381%, accepted
because runtime, allocations, live stage memory, and release linked size all
improve while exactness and paths remain unchanged.

## Reproduction

The parent executable is SHA-256
`127446b8a8c84967a5674d319b9defa2d5d40a9c2d5d04d4046cad9afb0e63ed`
and was built from `3057c2f8` against the same workspace Hyperreal,
Hyperlattice, Hyperlimit, and Hypertri sources.

```sh
cargo test
cargo test --no-default-features
cargo test --all-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --no-deps --all-features
cargo bench --no-run
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo check --target wasm32-unknown-unknown --no-default-features
cargo check --target wasm32-unknown-unknown --all-features
valgrind --tool=massif --time-unit=B --detailed-freq=1 \
  --massif-out-file=/tmp/hypermesh-vertex-arena-current-strict.massif \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
valgrind --tool=callgrind --cache-sim=no --branch-sim=yes \
  --callgrind-out-file=/tmp/hypermesh-vertex-arena-current.callgrind \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
taskset -c 11 perf stat -r 5 -x, \
  -e task-clock,cycles,instructions,branches,branch-misses \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
HYPERMESH_SIZE_TARGET_DIR=/tmp/hypermesh-vertex-arena-size-current \
  benchmarks/size-harness/measure.sh default
tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir hypermesh/target/hypermesh-ember-phase12-compact-polygon-vertex-arena-production \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --per-library
```
