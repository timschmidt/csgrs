# Phase 12/13 checkpoint: compact exact construction identities

Status: retained

Hypermesh implementation: `55d6af84c1f629e5939f714495687c349980ebd0`

Direct Hypermesh parent: `833bedcb`

Shared Hyperlimit implementation: `bb1c5e7c418bb8af25701c8201f833fa485b9e9a`

This checkpoint moves projective construction planes, source vertices, source
edges, and retained source-triangle identities into one checked 32-bit index
domain. It is compact arrangement storage for the replacement engine, not a
claim that Phase 12, Phase 13, EMBER removal, or the CGAL target is complete.

## Representation and failure contract

On the measured x86-64 target, `ConstructionPlaneIdentity` falls from 16 to 8
bytes, `ConstructionEdgeIdentity` from 40 to 20 bytes,
`ConstructionVertexIdentity` from 56 to 28 bytes, and the retained source
triangle descriptor from 40 to 32 bytes. The projective source-edge key falls
from 24 to 12 bytes. The 6,144 input polygons in either large-box fixture
therefore retain 49,152 fewer bytes of source-triangle identity payload during
polygon-soup construction.

Narrowing occurs only in fallible canonical constructors at input and
projective-construction boundaries. Mesh, plane, vertex, edge-endpoint, and
source-triangle values greater than `u32::MAX` produce `CapacityOverflow`.
The source-triangle setter converts every field before mutating the polygon, so
an error cannot leave a partial identity cycle. Array endpoints are sorted only
after both conversions succeed. Internal indexing widens a previously checked
`u32` losslessly. There is no wide-ID adapter, compatibility shim, alternate
identity representation, or unchecked truncation.

The identity arena records construction topology, not policy-sensitive
conclusions. Predicates and constructions still use the operation's
`DecisionContext`; `STRICT` remains unbounded exact evaluation and
`APPROXIMATE_512` retains Hyperlimit's terminal approximate 512-bit equality
evaluation and aggregate-certainty behavior.

## Correctness and path evidence

Default and minimal-feature suites each passed with 1,075 library tests, 4
active competitive tests (6 opt-in tests ignored), 59 core tests, 4 corpus
tests, 8 predicate-policy tests, 2 README tests, and 48 regression tests (1
benchmark test ignored). All features passed 1,076 library and 60 core tests
plus the same integration rows. The new 64-bit overflow test proves every
fallible identity constructor rejects an out-of-domain value and proves failed
source-triangle conversion leaves the polygon unmodified.

Formatting, warning-denied all-target/all-feature Clippy and rustdoc, native
minimal/all-feature checks, benchmark compilation, every fuzz binary, WASM
minimal/all-feature checks, native/WASM size harnesses, and the release Trunk
demo build passed. The inherited `NO_COLOR=1` had to be unset for Trunk 0.21.14
because that version accepts `true`/`false`, not `1`; the repository's
`TRUNK_COLOR=never` setting was preserved.

Both large fixtures and both policies produced identical certified topology:

| Fixture | Policy | Input triangles | Certainty | Output vertices | Output triangles |
| --- | --- | ---: | --- | ---: | ---: |
| `boxes-3072-general` | `STRICT` | 6,144 | `Certified` | 2,410 | 4,816 |
| `boxes-3072-general` | `APPROXIMATE_512` | 6,144 | `Certified` | 2,410 | 4,816 |
| `boxes-3072` | `STRICT` | 6,144 | `Certified` | 27 | 50 |
| `boxes-3072` | `APPROXIMATE_512` | 6,144 | `Certified` | 27 | 50 |

The regenerated five-crate production graph contains 19,963 nodes and 39,852
edges; the graph including tests, examples, benches, and fuzz targets contains
26,381 nodes and 49,798 edges. It records all checked identity constructors,
the polygon-soup and projective call paths, and the Hyperlimit policy-aware
comparison routes. Hypercurve and HyperSolve were excluded and untouched.

## Large-fixture heap

Valgrind 3.27.0 Massif used `--time-unit=B --detailed-freq=1`. Process-wide
useful-heap maxima are unchanged because larger exact geometry and output
arenas dominate at the sampled peaks. Allocator-overhead deltas are below 1.4
KiB and change sign by policy, so they are treated as noise rather than a peak
heap claim.

| Fixture/policy | Revision | Useful heap (B) | Extra heap (B) | Total heap (B) |
| --- | --- | ---: | ---: | ---: |
| general / `STRICT` | parent | 28,864,564 | 1,228,044 | 30,092,608 |
| general / `STRICT` | compact IDs | 28,864,564 | 1,227,532 | 30,092,096 |
| general / `APPROXIMATE_512` | parent | 28,864,564 | 1,226,812 | 30,091,376 |
| general / `APPROXIMATE_512` | compact IDs | 28,864,564 | 1,228,204 | 30,092,768 |
| certified / `STRICT` | parent | 1,063,718 | 1,274 | 1,064,992 |
| certified / `STRICT` | compact IDs | 1,063,718 | 1,258 | 1,064,976 |
| certified / `APPROXIMATE_512` | parent | 1,063,718 | 1,274 | 1,064,992 |
| certified / `APPROXIMATE_512` | compact IDs | 1,063,718 | 1,258 | 1,064,976 |

Heaptrack reports exactly 844,735 allocation calls for both general-path
revisions. Its `build_polygon_soup_internal` carrier row falls by 49.15 KiB,
matching the exact 8-byte descriptor reduction across 6,144 polygons. Temporary
allocation counts differ by three (33,789 parent, 33,792 candidate); no new
logical allocation site was added.

## Runtime

Four alternating five-process `perf stat` batches were pinned to CPU 11 for
the `STRICT` general-path fixture (20 processes per revision). Clock and cycles
remain frequency/order sensitive; their aggregate is slightly favorable.

| Revision | Task clock (ms) | Cycles | Instructions | Branches | Branch misses |
| --- | ---: | ---: | ---: | ---: | ---: |
| parent | 2,040.950 | 8,333,696,171 | 24,410,469,086 | 4,687,262,324 | 14,134,600 |
| compact IDs | 2,033.085 | 8,308,323,882 | 24,416,645,267 | 4,692,634,155 | 14,267,861 |
| change | -0.3854% | -0.3045% | +0.0253% | +0.1146% | +0.9428% |

Deterministic Callgrind traced 24,412,416,094 parent versus 24,424,410,247
candidate instructions. Of the 11,994,153-instruction delta, 10,842,564 comes
from libc `memcmp` executing a different internal alignment/content-prefix
route even though its two dominant caller counts are identical (91,587,384
rational equality and 23,088,776 rational ordering calls). Excluding libc
`memcmp` self work, traced instructions increase 0.0052% and conditional
branches increase 0.0083%. Dominant exact predicate, subdivision, trace, BVH,
and output call counts are unchanged. The representation is retained because
measured clock/cycles, exact retained identity memory, and replacement
architecture improve while added deterministic engine work is immaterial.

## Native and WASM size

The canonical dependency-only harness was clean-built with rustc 1.97.0.
Native code is linked `.text`; WASM is `wasm-opt -Oz`.

| Consumer/profile | Parent (B) | Compact IDs (B) | Change |
| --- | ---: | ---: | ---: |
| General release native `.text` | 4,083,164 | 4,082,772 | -392 (-0.0096%) |
| General release optimized WASM | 2,755,551 | 2,754,397 | -1,154 (-0.0419%) |
| Immediate release native `.text` | 4,116,220 | 4,116,004 | -216 (-0.0052%) |
| Immediate release optimized WASM | 2,770,102 | 2,769,016 | -1,086 (-0.0392%) |
| General size native `.text` | 1,880,546 | 1,880,730 | +184 (+0.0098%) |
| General size optimized WASM | 1,174,389 | 1,174,827 | +438 (+0.0373%) |
| Immediate size native `.text` | 1,892,326 | 1,892,902 | +576 (+0.0304%) |
| Immediate size optimized WASM | 1,184,523 | 1,184,967 | +444 (+0.0375%) |

Release consumers shrink; size-profile consumers grow by at most 0.038%.
This mixed sub-0.04% result is accepted provisionally. Phase 16 must remove the
historical subdivision/trace/BSP implementation, and Phase 17 must optimize the
final linked result without sacrificing path completeness or runtime.

## Reproduction

The parent was an isolated archive of `833bedcb` with absolute paths to the
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
cargo build --locked --manifest-path benchmarks/size-harness/Cargo.toml --profile size
cargo build --locked --manifest-path benchmarks/size-harness/Cargo.toml \
  --profile size --target wasm32-unknown-unknown
valgrind --tool=massif --time-unit=B --detailed-freq=1 \
  --massif-out-file=/tmp/hypermesh-compact-ids.massif \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
taskset -c 11 perf stat -r 5 -x, \
  -e task-clock,cycles,instructions,branches,branch-misses \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
HYPERMESH_SIZE_TARGET_DIR=/tmp/hypermesh-size-compact-ids \
  benchmarks/size-harness/measure.sh default
tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir hypermesh/target/hypermesh-ember-phase12-compact-construction-identities-production \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --per-library
```
