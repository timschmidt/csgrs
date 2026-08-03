# Phase 12/13 checkpoint: interned exact intersection endpoints

Status: retained

Hypermesh implementation: `80aced02c431890b44e4861f6d0e8dd95ce0c558`

Direct Hypermesh parent: `3465b8db`

Shared Hyperlimit implementation: `bb1c5e7c418bb8af25701c8201f833fa485b9e9a`

This checkpoint replaces the 96-byte inline endpoint pair in every retained
pairwise-intersection segment with two checked 32-bit point-arena IDs. Exact
rational endpoint coordinates share one arena record by structural
Hyperreal identity. It is compact arrangement substrate, not a claim that
Phase 12, Phase 13, EMBER removal, or the CGAL target is complete.

## Representation, policy, and failure contract

`PairwiseIntersectionSegment` is 8 bytes on the measured x86-64 target, down
from 96 bytes. `PairwiseIntersectionGraph` owns one `Vec<Point3>` arena and
resolves IDs to borrowed points at its internal iterator boundary, so
controlled callers do not receive a second or compatibility representation.
The existing shared undirected segment and compact directed-event arenas are
unchanged.

Only points whose three coordinates are exact rational Hyperreal values are
interned. Their full exact storage identity and exact fingerprint drive lookup;
no floating approximation or policy-sensitive conclusion enters the key. If
either endpoint is symbolic/non-rational, both endpoints are appended
unmerged. That conservative branch deliberately adds no equality decision.
Consequently `STRICT` remains unbounded certified evaluation and
`APPROXIMATE_512` still terminates only through Hyperlimit's approximate
512-bit equality policy. Aggregate certainty remains owned by the operation's
unchanged `DecisionContext`.

The builder checks face, row, node, segment, and point capacities before
mutation. It reserves the point vector, exact-chain vector, and exact maps for
both endpoints before inserting either, then appends the already-reserved
segment and two directed nodes. An allocation or capacity failure therefore
cannot retain one endpoint, an orphan segment, or a half edge. Endpoint IDs
are converted with checked `u32::try_from` after the arena-length proof.
Polygon-order remapping pre-reserves and copies the point arena, registers its
unindexed length with the builder, and validates every copied endpoint ID.
There is no unchecked narrowing, wide-ID adapter, compatibility shim, or dual
engine.

## Correctness and path evidence

Default and minimal-feature suites each passed with 1,078 library tests, 4
active competitive tests (6 opt-in tests ignored), 59 core tests, 4 corpus
tests, 8 predicate-policy tests, 2 README tests, and 48 regression tests (1
benchmark test ignored). All features passed 1,079 library and 60 core tests
plus the same integration rows. New unit coverage proves exact endpoint
sharing, deliberately unmerged symbolic endpoints, remap preservation, the
8-byte carrier layout, symmetric event sharing, and failure/self-pair paths
with no point/segment/node mutation.

Formatting, warning-denied all-target/all-feature Clippy and rustdoc, native
minimal/all-feature checks, benchmark compilation, every fuzz binary, WASM
minimal/all-feature checks, clean native/WASM size builds, and the release
Trunk demo passed. The opt-in YeahRight probe correctly declined because no
local asset was configured; no network request or substitute fixture was used.

Both permanent large fixtures and both runtime policies produced identical
certified topology:

| Fixture | Policy | Input triangles | Certainty | Output vertices | Output triangles |
| --- | --- | ---: | --- | ---: | ---: |
| `boxes-3072-general` | `STRICT` | 6,144 | `Certified` | 2,410 | 4,816 |
| `boxes-3072-general` | `APPROXIMATE_512` | 6,144 | `Certified` | 2,410 | 4,816 |
| `boxes-3072` | `STRICT` | 6,144 | `Certified` | 27 | 50 |
| `boxes-3072` | `APPROXIMATE_512` | 6,144 | `Certified` | 27 | 50 |

The regenerated five-crate production graph contains 19,998 nodes and 39,923
edges; the graph including tests, examples, benches, and fuzz targets contains
26,416 nodes and 49,869 edges. Its only calls into the new storage operations
are `PairwiseIntersectionGraphBuilder::append_segment_pair` to
`intern_exact_pair_or_append` and `remap_polygon_order` to
`register_unindexed_existing`. Existing graph iterators resolve compact IDs
directly. Hypercurve and HyperSolve were excluded and untouched.

## Endpoint density and large-fixture heap

Temporary diagnostic instrumentation of the same 6,144-triangle general
fixture counted 288 undirected segments, 576 endpoint occurrences, and 72
unique exact endpoints. The instrumentation was removed before the retained
commit; the permanent large-fixture probe and heap measurements exercise the
same production route.

Valgrind 3.27.0 Massif used `--time-unit=B --detailed-freq=1`. The general
fixture's process-wide useful-heap maximum falls exactly 124,904 bytes
(0.4327%) under each policy. The certified-convex control never reaches the
general intersection graph, so its useful peak is intentionally flat.

| Fixture/policy | Revision | Useful heap (B) | Extra heap (B) | Total heap (B) |
| --- | --- | ---: | ---: | ---: |
| general / `STRICT` | parent | 28,864,564 | 1,228,684 | 30,093,248 |
| general / `STRICT` | endpoint arena | 28,739,660 | 1,227,708 | 29,967,368 |
| general / `APPROXIMATE_512` | parent | 28,864,564 | 1,226,780 | 30,091,344 |
| general / `APPROXIMATE_512` | endpoint arena | 28,739,660 | 1,227,692 | 29,967,352 |
| certified / `STRICT` | parent | 1,063,718 | 1,274 | 1,064,992 |
| certified / `STRICT` | endpoint arena | 1,063,718 | 1,258 | 1,064,976 |
| certified / `APPROXIMATE_512` | parent | 1,063,718 | 1,274 | 1,064,992 |
| certified / `APPROXIMATE_512` | endpoint arena | 1,063,718 | 1,258 | 1,064,976 |

Heaptrack reports 844,735 parent versus 844,759 candidate allocation calls
(+24, +0.0028%) and 34,039 temporary allocations for both. The extra calls
are the transient exact-interner storage; Massif shows that the compact
retained layout more than repays them at peak. Heaptrack's rounded peak moves
from 28.94 MB to 28.81 MB.

## Runtime

Two alternating five-process `perf stat` batches per revision were pinned to
CPU 11 for the `STRICT` general fixture. Clock and cycles are frequency/order
sensitive, but instructions agree closely with deterministic Callgrind.

| Revision | Task clock (ms) | Cycles | Instructions | Branches | Branch misses |
| --- | ---: | ---: | ---: | ---: | ---: |
| parent | 2,142.155 | 8,695,571,828 | 24,405,410,507 | 4,682,044,920 | 14,179,520 |
| endpoint arena | 1,983.830 | 8,066,425,251 | 23,896,240,950 | 4,633,564,728 | 14,283,520 |
| change | -7.3909% | -7.2353% | -2.0863% | -1.0354% | +0.7335% |

Callgrind with branch simulation traced 24,413,066,684 parent versus
23,902,510,549 candidate instructions, a deterministic reduction of
510,556,135 instructions (2.0913%). Total branches fall 8,831,361 (0.2464%)
and conditional branches fall 8,822,113 (0.2918%). Estimated total branch
mispredictions rise 1,076,596 (0.9638%), which is reported rather than hidden;
the whole-workload instruction and pinned-time improvements dominate on this
fixture.

## Native and WASM size

The canonical dependency-only harness was clean-built with rustc 1.97.0.
Native code is linked `.text`; WASM is `wasm-opt -Oz`.

| Consumer/profile | Parent (B) | Endpoint arena (B) | Change |
| --- | ---: | ---: | ---: |
| General release native `.text` | 4,082,772 | 4,085,052 | +2,280 (+0.0558%) |
| General release optimized WASM | 2,754,397 | 2,757,749 | +3,352 (+0.1217%) |
| Immediate release native `.text` | 4,116,004 | 4,118,284 | +2,280 (+0.0554%) |
| Immediate release optimized WASM | 2,769,016 | 2,772,304 | +3,288 (+0.1187%) |
| General size native `.text` | 1,880,730 | 1,883,674 | +2,944 (+0.1565%) |
| General size optimized WASM | 1,174,827 | 1,176,941 | +2,114 (+0.1799%) |
| Immediate size native `.text` | 1,892,902 | 1,895,822 | +2,920 (+0.1543%) |
| Immediate size optimized WASM | 1,184,967 | 1,187,078 | +2,111 (+0.1781%) |

The maximum linked growth is 0.180%. It is accepted because the same retained
checkpoint saves 0.433% peak useful heap and 2.091% deterministic instructions
on the general large fixture, while exactness, paths, and both-policy output
remain unchanged. Phase 16 must delete the historical subdivision/trace/BSP
implementation, and Phase 17 must re-optimize the final linked result.

## Reproduction

The parent was an isolated archive of `3465b8db` with absolute paths to the
same workspace Hyperreal, Hyperlattice, Hyperlimit, and Hypertri sources.

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
  --massif-out-file=/tmp/hypermesh-endpoint-current-strict.massif \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
valgrind --tool=callgrind --cache-sim=no --branch-sim=yes \
  --callgrind-out-file=/tmp/hypermesh-endpoint-current.callgrind \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
taskset -c 11 perf stat -r 5 -x, \
  -e task-clock,cycles,instructions,branches,branch-misses \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
HYPERMESH_SIZE_TARGET_DIR=/tmp/hypermesh-endpoint-size-current \
  benchmarks/size-harness/measure.sh default
tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir hypermesh/target/hypermesh-ember-phase12-interned-endpoints-production \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --per-library
```
