# Phase 12/13 checkpoint: contiguous intersection rows

Status: retained

Hypermesh implementation: `e04b73d32a5eab14a61b2c7c32e68d7fa34ff2ac`

Direct Hypermesh parent: `019131518445c8e2eca756d51fab753ada67cdf9`

Shared Hyperlimit implementation: `bb1c5e7c418bb8af25701c8201f833fa485b9e9a`

This checkpoint replaces the retained intersection graph's linked face rows
with one contiguous offset/event representation. It is compact arrangement
substrate, not a claim that Phase 12, Phase 13, EMBER removal, or the CGAL
target is complete.

## Representation and failure contract

The parent graph retains two `u32` words per face (`head` and `count`) and one
12-byte node per directed event (`next`, `other_polygon`, and `segment`). Its
builder additionally retains one `u32` tail per face. The new graph retains
one `u32` offset per face plus one terminal offset and one 8-byte event
(`other_polygon` and `segment`) per directed event. Row iteration is therefore
sequential rather than pointer-index chasing.

Construction appends only admitted intersection events to one pending arena;
it never retains broad-phase candidate pairs. A fallible final counting
distribution preserves each face's BVH discovery order. Builder creation,
face IDs, row counts, event/segment/point growth, prefix sums, both final
allocations, pending event ranges, other-face IDs, segment IDs, and final
cursor positions are checked. Failure consumes and drops the private builder;
no partial graph escapes. Symmetric pair insertion remains preflighted and
atomic, so allocation or capacity failure cannot leave a half edge, orphan
segment, or orphan endpoint.

For the 6,144-face general fixture, 288 undirected segments produce 576
directed events. The logical retained row carrier falls from 56,064 bytes
(`2 * 6,144 * 4 + 576 * 12`) to 29,188 bytes
(`6,145 * 4 + 576 * 8`), an exact reduction of 26,876 bytes (47.938%). The
Massif reduction is larger because the old linked node vector retains spare
capacity and the new graph has a smaller owner payload.

The new builder's logical face/event carrier is 31,488 bytes before
finalization. Even while its final 24,580-byte offset and 4,608-byte event
allocations coexist with pending storage, the logical carrier is 60,676 bytes,
below the parent's 80,640-byte builder carrier. The endpoint interner is
dropped before those final allocations.

There is no compatibility shim, view adapter, alternate representation, wide
fallback, feature-selected engine, or unchecked narrowing. Physical
pre-test-module source in `intersection.rs` grows from 1,321 to 1,403 lines;
the test module grows from 254 to 262 lines, and controlled subdivision tests
grow by one net line. This temporary source/linked-size cost is owed back when
the historical machinery is deleted.

## Exactness and policy

This change records and reorders no geometric fact. Endpoint `Point3` values,
shared segment IDs, other-face IDs, discovery order, and every event consumed
by subdivision are unchanged. Every intersection predicate and endpoint
construction continues through the operation's existing `DecisionContext`.

`STRICT` therefore remains unbounded certified evaluation.
`APPROXIMATE_512` still terminates only through Hyperlimit's approximate
512-bit policy, and the operation-local aggregate certainty is unchanged.
Graph finalization performs no scalar comparison, equality, incidence, or
topology decision, so it cannot contaminate later strict work.

## Correctness and path evidence

Default and minimal-feature suites each passed with 1,079 library tests, 4
active competitive tests (6 opt-in tests ignored), 59 core tests, 4 corpus
tests, 8 predicate-policy tests, 2 README tests, and 48 regression tests (1
benchmark test ignored). All features passed 1,080 library and 60 core tests
plus the same integration rows.

Focused tests cover noncontiguous streamed append order, exact CSR offsets,
empty rows, compact carrier sizes, shared exact endpoint IDs, deliberately
unmerged symbolic endpoints, polygon-order remapping, self pairs, invalid and
overflowing face domains, corrupted row counts, and atomic pair failure.

Formatting, warning-denied all-target/all-feature Clippy and rustdoc, benchmark
compilation, every fuzz binary, minimal/all-feature WASM checks, clean
native/WASM size builds, and the release Trunk UI passed.

Both permanent large fixtures and both runtime policies produced identical
certified topology:

| Fixture | Policy | Input triangles | Certainty | Output vertices | Output triangles |
| --- | --- | ---: | --- | ---: | ---: |
| `boxes-3072-general` | `STRICT` | 6,144 | `Certified` | 2,410 | 4,816 |
| `boxes-3072-general` | `APPROXIMATE_512` | 6,144 | `Certified` | 2,410 | 4,816 |
| `boxes-3072` | `STRICT` | 6,144 | `Certified` | 27 | 50 |
| `boxes-3072` | `APPROXIMATE_512` | 6,144 | `Certified` | 27 | 50 |

The regenerated five-crate production graph contains 20,042 nodes and 39,993
edges; the graph including examples, tests, benches, and fuzz targets contains
26,460 nodes and 49,939 edges. The production route constructs one builder,
appends admitted pairs, performs one checked finalization, and exposes direct
contiguous row iterators. Hypercurve and HyperSolve were excluded and
untouched.

## Large-fixture heap and allocations

Valgrind 3.27.0 Massif used `--time-unit=B --detailed-freq=1`. Unlike the
short-lived polygon vertex arena, the retained graph remains live at the later
process maximum. Useful peak therefore falls exactly 32,268 bytes (0.1123%)
under both policies. The 5,392-byte difference beyond logical live-length
savings reflects removed spare linked-node capacity and the smaller graph
owner payload.

| Fixture/policy | Revision | Useful heap (B) | Extra heap (B) | Total heap (B) |
| --- | --- | ---: | ---: | ---: |
| general / `STRICT` | parent | 28,739,660 | 1,227,788 | 29,967,448 |
| general / `STRICT` | contiguous rows | 28,707,392 | 1,227,008 | 29,934,400 |
| general / `APPROXIMATE_512` | parent | 28,739,660 | 1,227,996 | 29,967,656 |
| general / `APPROXIMATE_512` | contiguous rows | 28,707,392 | 1,227,936 | 29,935,328 |
| certified / `STRICT` | parent | 1,063,718 | 1,258 | 1,064,976 |
| certified / `STRICT` | contiguous rows | 1,063,718 | 1,258 | 1,064,976 |
| certified / `APPROXIMATE_512` | parent | 1,063,718 | 1,258 | 1,064,976 |
| certified / `APPROXIMATE_512` | contiguous rows | 1,063,718 | 1,258 | 1,064,976 |

Heaptrack 1.5.0 reports 838,604 parent versus 838,605 candidate allocation
calls and 34,038 versus 34,039 temporary allocations. The observed one-call
delta is within the one-call variation already seen between same-source probe
builds; final row allocations replace the removed head/tail ownership. Its
rounded peak falls from 28.81 to 28.78 MB.
Peak RSS including profiler overhead moves from 40.49 to 40.61 MB and is
supporting noise only; Massif supplies the byte-level heap claim.

## Runtime and deterministic work

Six retained five-process `perf stat` batches per revision were pinned to CPU
11 for the `STRICT` general fixture (30 processes per revision). A fixed parent
executable was sampled throughout. One parent batch with 11.64% internal task
clock deviation and its paired candidate batch were excluded before analysis;
the maximum retained internal deviation was 2.13%.

| Revision | Task clock (ms) | Cycles | Instructions | Branches | Branch misses |
| --- | ---: | ---: | ---: | ---: | ---: |
| parent | 1,938.872 | 7,915,324,059 | 23,882,247,674 | 4,604,782,728 | 13,659,334 |
| contiguous rows | 1,913.473 | 7,830,848,281 | 23,898,221,126 | 4,633,681,027 | 14,221,669 |
| change | -1.3100% | -1.0672% | +0.0669% | +0.6276% | +4.1169% |

The paired batch-mean clock interval is broad (-4.63% to +2.01% at 95%), so
the table is not a universal 1.31% speed claim. It establishes a favorable
sample mean without a demonstrated regression. Hardware branch counters are
address/layout-sensitive and worsen; this is reported rather than hidden.

Callgrind branch simulation compares the exact fixed binaries. The candidate
executes 23,903,497,834 versus 23,898,239,510 instructions (+5,258,324,
+0.0220%) and 3,576,124,518 versus 3,570,892,768 total branches (+0.1465%),
but modeled mispredictions fall from 113,853,710 to 111,140,278
(-2,713,432, -2.3833%). Sequential row scans trade a very small counting-pass
cost for lower retained memory and better modeled predictability. The real
clock/cycle mean, exact peak reduction, and replacement architecture support
retention; Phase 17 still owes recovery of the added deterministic work.

This is a direct historical-parent comparison. CGAL EPECK was not rerun for an
internal carrier-only checkpoint because operations and output semantics are
unchanged; attributing a competitive delta here would be misleading. The
pinned shared-contract CGAL comparison remains a Phase 17 per-case gate.

## Native and WASM size

The canonical dependency-only harness was clean-built with rustc 1.97.0.
Native code is linked `.text`; WASM is `wasm-opt -Oz`.

| Consumer/profile | Parent (B) | Contiguous rows (B) | Change |
| --- | ---: | ---: | ---: |
| General release native `.text` | 4,081,972 | 4,091,844 | +9,872 (+0.2418%) |
| General release optimized WASM | 2,754,491 | 2,760,635 | +6,144 (+0.2231%) |
| Immediate release native `.text` | 4,115,188 | 4,125,060 | +9,872 (+0.2399%) |
| Immediate release optimized WASM | 2,769,105 | 2,775,185 | +6,080 (+0.2196%) |
| General size native `.text` | 1,883,746 | 1,885,330 | +1,584 (+0.0841%) |
| General size optimized WASM | 1,177,387 | 1,178,835 | +1,448 (+0.1230%) |
| Immediate size native `.text` | 1,895,902 | 1,897,478 | +1,576 (+0.0831%) |
| Immediate size optimized WASM | 1,187,530 | 1,188,976 | +1,446 (+0.1218%) |

Maximum linked growth is 0.2418%. It is accepted provisionally because the
large-path clock/cycle means and actual peak heap improve, performance has
priority over size, and Phase 16 removes the much larger historical
subdivision/trace/BSP implementation. No size debt is relabeled complete.

## Reproduction

The fixed parent executable SHA-256 is
`f58a3b7d2b5f3c6810c470c71b34e266017db19cee0a1726bef7364afcc33274`;
the candidate executable SHA-256 is
`3cda9842a356a6112b4eccd0e04db5c9e82180e2efefbdd2cb9f5f3693f64465`.
Both use the same workspace Hyperreal, Hyperlattice, Hyperlimit, and Hypertri
sources.

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
  --massif-out-file=/tmp/hypermesh-csr-current-strict.massif \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
valgrind --tool=callgrind --cache-sim=no --branch-sim=yes \
  --callgrind-out-file=/tmp/hypermesh-csr-current.callgrind \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
taskset -c 11 perf stat -r 5 -x, \
  -e task-clock,cycles,instructions,branches,branch-misses \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
HYPERMESH_SIZE_TARGET_DIR=/tmp/hypermesh-csr-size-current \
  benchmarks/size-harness/measure.sh default
tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir hypermesh/target/hypermesh-ember-phase12-contiguous-intersection-rows-production \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --per-library
```
