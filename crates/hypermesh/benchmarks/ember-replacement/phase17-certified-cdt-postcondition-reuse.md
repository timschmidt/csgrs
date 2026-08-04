# Phase 17 certified CDT postcondition reuse

Date: 2026-08-03

Implementation revisions: Hypertri `41631f6`, Hypermesh `272185db`

Status: retained Phase 17 checkpoint; Phase 17 and Phase 18 remain open.

## Result

Hypermesh now consumes the exact topology postconditions established by
Hypertri's checked, topology-only constrained-triangulation entry point.  The
surface-arrangement algorithm, accepted input domain, output topology, policy
decisions, and error paths are unchanged.  This is cross-crate proof scheduling,
not a mesh/operation/size/coordinate-width fixture dispatch.

Hypertri's final constrained-convex-hull validation already evaluates every
triangle orientation, proves consistent nonzero winding, builds the complete
edge-use table, proves every protected constraint is a returned edge, and proves
complete convex-hull coverage.  It now carries the already-computed common
winding out of that pass and rejects a negative result.  Consequently the
successful API contract explicitly guarantees strictly positive triangles and
preserved constraints under the selected policy and returned aggregate
certainty.  A permanent regression constructs a consistently negative complete
triangulation and verifies that the checked contract rejects it.

After absorbing Hypertri's certainty, Hypermesh retains its one exact source-face
orientation decision, maps each positive local triangle into the shared point
arena, and reverses every triangle only when the source projection is negative.
It no longer repeats one orientation predicate per returned triangle or builds
three ordered sets to re-prove constraint equality and edge membership.  The
test-only returned constraint list is still populated directly from the checked
topology.  An invalid or indeterminate Hypertri result still returns through the
existing typed error path before Hypermesh consumes any postcondition.

`STRICT` therefore cannot consume a terminal approximation.
`APPROXIMATE_512` still terminates only through Hyperlimit's 512-bit policy, and
Hypermesh absorbs `Approximate512Consumed` before using the result.  All measured
exact-rational fixtures remain byte-for-byte policy-equal and `Certified`.

## Full-resolution performance

The final source was measured in fresh processes pinned to CPU 11.  The test
imports the exact 11,894-by-11,894 rotated YeahRight pair, performs one exact
intersection, validates the certified empty result, destroys it, and repeats the
whole process five times.

| Metric | `7129b524` | `272185db` | Movement |
| --- | ---: | ---: | ---: |
| Median wall time | 1.93 s | 1.92 s | -0.52% |
| Cycles | 7,497,540,892 | 7,464,605,759 | -0.44% |
| Instructions | 19,222,742,051 | 19,133,466,915 | -0.46% |
| Branches | 3,433,388,544 | 3,414,974,772 | -0.54% |
| Cache misses | 19,858,900 | 19,189,528 | -3.37% |

The five final wall samples were 1.94, 1.92, 1.90, 1.91, and 1.94 seconds.
Historical EMBER remains 3,312.66 seconds, so the current row is about 1,725.3x
faster.  Pinned CGAL EPECK remains 0.09 seconds and 15,516 KiB RSS; Hypermesh is
still about 21.33x slower and 12.24x larger in fresh-process RSS.  Those absolute
gates remain open.

## General controls and exact-box A/B

Three `perf stat` repetitions of four independent fixtures confirm that the
change is not bought from a different path:

| Fixture | Input triangles | Parent instructions | Current instructions | Movement |
| --- | ---: | ---: | ---: | ---: |
| 2,049-bit wide rational | 6,144 | 4,160,089,259 | 4,159,897,802 | -0.005% |
| Clipped voxel torus 33 | 6,412 | 1,216,283,938 | 1,214,500,564 | -0.147% |
| Clipped voxel torus 65 | 25,100 | 5,220,285,535 | 5,218,831,419 | -0.028% |
| Dense coplanar 16 | 6,144 | 3,205,690,332 | 3,181,158,003 | -0.765% |

An isolated Criterion run was frequency-sensitive, so the small-case gate was
also tested as an interleaved same-machine parent/current A/B.  Each process
performed 1,000 complete shared-arrangement evaluations over ordinary
overlapping boxes; five parent/current pairs ran on CPU 11.  Both binaries used
the same current Hypertri, isolating the Hypermesh consumption change.

| Policy | Parent median | Current median | Time | Instructions | Branches |
| --- | ---: | ---: | ---: | ---: | ---: |
| `STRICT` | 637.360 us | 614.860 us | -3.53% | -1.41% | -1.81% |
| `APPROXIMATE_512` | 639.840 us | 617.034 us | -3.56% | -1.43% | -1.85% |

All four output meshes retain exactly 28 shared vertices and
48/24/40/32 triangles.  Directionally, the current rows remain about 5.14x and
4.78x the pinned CGAL copy-outside/copy-inside times, so exact-box parity stays
open.

## Large-fixture heap

The direct global-allocator probe excludes fixture construction and reports the
Boolean kernel's requested-payload boundary.  Both policies are exactly equal.

| Fixture | Metric | `7129b524` | `272185db` | Movement |
| --- | --- | ---: | ---: | ---: |
| Full YeahRight | Incremental peak | 158,258,204 B | 158,258,204 B | unchanged |
| Full YeahRight | Allocation calls | 19,290,062 | 19,273,900 | -16,162 |
| Full YeahRight | Reallocations | 2,133,982 | 2,133,982 | unchanged |
| Full YeahRight | Allocated bytes | 1,017,109,024 B | 1,015,039,056 B | -2,069,968 B |
| Wide rational | Incremental peak | 31,038,074 B | 31,038,074 B | unchanged |
| Wide rational | Allocation calls | 2,107,571 | 2,107,379 | -192 |
| Wide rational | Reallocations | 227,677 | 227,677 | unchanged |
| Wide rational | Allocated bytes | 462,626,870 B | 462,613,302 B | -13,568 B |

The full result still retains 24,389,848 input-fact bytes, 56 output bytes, and
10,792 bytes after input drop.  The wide result retains 26,568 input-fact bytes,
520,472 output bytes, and 96,192 bytes after input drop.  This checkpoint removes
temporary proof duplication; the dominant live peak belongs to a later stage
and therefore does not move.

Fresh-process full-fixture RSS is 189,952 KiB under `STRICT` and 189,892 KiB
under `APPROXIMATE_512`; this allocator/loader-level metric is effectively flat
and remains an absolute CGAL loss.

## Linked size

Every canonical linked row shrinks.  Native values are `.text`; WASM values are
`wasm-opt -Oz` bytes.

| Profile / consumer | Parent native | Current native | Parent WASM | Current WASM |
| --- | ---: | ---: | ---: | ---: |
| release / general | 2,028,558 | 2,024,622 (-0.194%) | 1,446,143 | 1,443,031 (-0.215%) |
| release / immediate | 2,031,702 | 2,027,766 (-0.194%) | 1,447,997 | 1,444,878 (-0.215%) |
| size / general | 1,086,399 | 1,084,319 (-0.191%) | 682,891 | 680,776 (-0.310%) |
| size / immediate | 1,087,355 | 1,085,259 (-0.193%) | 683,301 | 681,393 (-0.279%) |

Across production source, Hypermesh removes 29 net lines while Hypertri adds the
checked contract and its test.  No compatibility layer, alternate engine,
persistent fact carrier, or dependency is introduced.

## Validation and graph

- Hypertri: 74 unit, 48 integration, and 4 doctests pass under all features;
  the new negative-winding contract regression passes.
- Hypermesh: 167 tests pass and the six documented external/manual tests remain
  ignored; the full-resolution exact oracle passes explicitly.
- Both crates pass no-default checks, all-target/all-feature Clippy with warnings
  denied, rustdoc with warnings denied, and formatting/diff checks.
- Hypermesh fuzz targets and all benchmark targets compile.
- Full and wide direct heap probes pass under both policies with identical
  certified outputs.
- The full dispatch corpus reports zero unknown-fact events and zero
  fallback/abort events.
- The regenerated source graph covers only Hyperreal, Hyperlattice, Hyperlimit,
  Hypertri, and Hypermesh: 14,687 function nodes and 24,433 edges.  Hypercurve and
  HyperSolve are excluded.

## Open work

This checkpoint does not close Phase 17 or Phase 18.  Full YeahRight runtime/RSS
and exact boxes remain absolute CGAL losses; the full live-heap peak is unchanged;
real-world and deeper-symbolic corpus breadth, stage-specific heap attribution,
and the final path/requirement audit remain open.  The retained profile still
points to general corefinement/CDT work and scalar fact/lifetime ownership.

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
/usr/bin/time -v env YEAHRIGHT_BENCH=1 \
  target/release/examples/large_mesh_heap_probe yeahright-full-rotated strict
/usr/bin/time -v env YEAHRIGHT_BENCH=1 \
  target/release/examples/large_mesh_heap_probe \
  yeahright-full-rotated approximate-512

benchmarks/size-harness/measure.sh
YEAHRIGHT_BENCH=1 cargo bench --bench dispatch_trace --features dispatch-trace

tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir /tmp/hypermesh-certified-cdt-postcondition-reuse-callgraph-2026-08-03 \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format json
```
