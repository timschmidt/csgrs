# Phase 17 checkpoint: path-complete performance and retained-fact scheduling

Captured 2026-08-03 at Hypermesh implementation commits `09dc9bb1` and
`8976c0f9`. The machine-readable companion is
`phase17-path-complete-performance-checkpoint.toml`.

## Outcome

The sole production surface-arrangement engine now covers two additional
same-operand paths found by the topology-aware Boolean fuzzer:

- a coplanar overlay vertex synthesized on an authored source edge is fanned
  out to every incident source face before bounded face corefinement; and
- a non-coplanar same-operand segment is no longer discarded merely because
  its exact endpoints lie on triangle boundaries. Only a genuinely shared
  authored feature, proved by retained construction identity, is elided.

The first regression combines two overlapping closed shells in one operand
and unions them with an octahedron. Both policies and operand orders now
produce a closed `Certified` boundary with exact signed six-volume `21763/48`.
The second combines a subdivided and unsubdivided overlapping shell in one
operand and intersects it with a disjoint octahedron. Both policies and orders
produce the exact empty result. Both cases are permanent manifest records and
fuzz seeds.

The fixes are arrangement invariants, not fixture branches. Coplanar-overlay
source-edge membership is scheduled only at the overlay stage; ordinary point
insertion has no corresponding branch. Authored manifold neighbors are now
recognized from retained vertex and edge identities instead of reconstructing
that fact from `Point3<Real>` comparisons. Degree-two radial edges, unchanged
source faces, and exact radial orientation also use direct general structural
paths. No benchmark name, coordinate, triangle count, or fixture shape is
inspected by production code.

## Exactness and policy

- `hyperreal::Real` remains the only scalar used for construction and accepted
  topology.
- One operation-local `DecisionContext` carries either `STRICT` or
  `APPROXIMATE_512` through input validation, intersections, overlay,
  Hypertri, radial topology, winding, and output certification.
- Retained identities prove equality/orientation only when they denote the
  same authored construction. Numeric aliases still reach the policy-aware
  point interner.
- `STRICT` never consumes the approximate terminal. Only Hyperlimit's
  approximate-512 terminal may do so, and its consumption reaches aggregate
  `MeshCertainty`.
- All exact rational fixtures in this checkpoint remain `Certified` under both
  policies.

## Validation and fuzzing

`cargo test --all-features` passes 108 unit and 32 integration tests, with six
documented opt-in/manual ignores and no failures. Clippy with warnings denied,
warning-denied rustdoc, rustfmt, fuzz-bin compilation, and `git diff --check`
all pass.

The final libFuzzer/AddressSanitizer smoke matrix is:

| Target | Executions | Result |
| --- | ---: | --- |
| `polygon_predicates` | 6,619 | pass |
| `bvh_queries` | 13,165 | pass |
| `mesh_and_hull` | 4,481 | pass |
| `boolean_pipeline` | 1,352 | pass; all 1,159 seeds replayed |
| `boolean_box_oracle` | 1,232 | pass; all 962 seeds replayed |
| `boolean_input_validation` | 41,508 | pass |
| `boolean_hyperreal_representations` | 242 | pass; all 185 seeds replayed |
| `boolean_transformations` | 591 | pass; all 463 seeds replayed |

AddressSanitizer remained enabled. LeakSanitizer alone was disabled with
`ASAN_OPTIONS=detect_leaks=0` because the managed environment prevents its
ptrace-based final scan. Both original crash artifacts replay cleanly under
the same sanitizer binary; each is generalized into a deterministic public
regression rather than committed as an opaque crash blob.

## Competitive exact boxes

The common exact input is the versioned overlapping-box OFF pair. Hypermesh
constructs one arrangement and materializes union, intersection, left
difference, and right difference together. On CPU 11 of the Ryzen 7 5800X3D,
Criterion reports:

| Policy | Hypermesh 95% interval | Midpoint |
| --- | ---: | ---: |
| `STRICT` | 1.0305–1.0362 ms | 1.0330 ms |
| `APPROXIMATE_512` | 1.0374–1.0465 ms | 1.0422 ms |

A separate pinned 10,000-iteration loop reports 1.031642 ms strict and
1.034962 ms approximate-512. Both produce one 28-vertex shared arena and
48/24/40/32 triangles. The single-result union Criterion midpoint is
1.0169 ms.

The pinned CGAL 6.0.3 EPECK adapter ran 31 repetitions per copy boundary. With
the first cold repetition excluded, its four-result warmed medians are
130.0865 us with copies outside the timed interval and 128.2615 us with copies
inside it. Every output is valid, closed, and structurally valid, with exact
volumes 84/12/52/20 and 48/24/40/32 triangles. Hypermesh therefore remains
7.94–8.13x slower on this case. This is an open Phase 17 deficit, not parity.

For context only, the same-day non-exact competitor union midpoints are
63.618 us for Boolmesh and 56.747 us for Manifold-rust. Their scalar and
semantic contracts differ, so they are throughput controls rather than exact
oracles.

## Full-resolution historical hard case

The checksum-pinned YeahRight control mesh was acquired through the opt-in
fixture path and used in the 11,894-by-11,894 rotated intersection. A fresh,
single-CPU release process returned the exact empty result with
`MeshCertainty::Certified`:

| Engine/checkpoint | Wall time | Maximum RSS | Result |
| --- | ---: | ---: | --- |
| Historical EMBER | 3,312.66 s | 329,352 KiB | exact empty |
| Current arrangement | 48.02 s | 203,356 KiB | exact empty, `Certified` |
| Historical CGAL EPECK | 0.09 s | 15,516 KiB | exact empty |

The replacement is 68.99x faster than EMBER and lowers maximum RSS 38.26%.
The remaining deficits versus the established CGAL row are 533.56x runtime
and 13.11x RSS. A separate pinned counter run took 48.9965 s and retired
634,364,981,061 instructions, 98,972,369,150 branches, and 849,811,467 branch
misses (0.8586%). This row is now practical and correct, but plainly remains
the leading optimization target.

## Large-mesh runtime and heap

The permanent large fixture contains 3,072 triangles per operand, 6,144 total,
and produces the same `Certified` 2,410-vertex/4,816-triangle union under both
policies. The native mode primes only the policy-qualified closed-PWN fact;
the raw/general mode uses borrowed views with no reusable input fact. Both
enter the same Boolean engine.

Eleven pinned `perf stat` repetitions report:

| Input path | Policy | Task clock | Instructions |
| --- | --- | ---: | ---: |
| Native retained | `STRICT` | 163.99 ms | 1,760,630,809 |
| Native retained | `APPROXIMATE_512` | 168.82 ms | 1,760,728,824 |
| Raw/general | `STRICT` | 155.78 ms | 1,635,299,914 |
| Raw/general | `APPROXIMATE_512` | 154.79 ms | 1,635,268,690 |

Switching authored manifold-neighbor scheduling from point comparison to
retained identity removes 4.28% of native and 4.57% of raw/general
instructions versus the immediately preceding correct build. It also removes
10,371 dispatch events from the complete trace without changing topology.

Sequential Heaptrack recordings avoid the corruption observed when profiler
instances were launched concurrently:

| Input path | Policy | Allocations | Temporary | Exact peak heap | Peak RSS including Heaptrack |
| --- | --- | ---: | ---: | ---: | ---: |
| Native retained | `STRICT` | 331,078 | 84,784 | 16,423,650 B | 24.17 MB |
| Native retained | `APPROXIMATE_512` | 331,078 | 84,784 | 16,423,650 B | 23.98 MB |
| Raw/general | `STRICT` | 294,196 | 84,784 | 16,423,658 B | 24.22 MB |
| Raw/general | `APPROXIMATE_512` | 294,196 | 84,784 | 16,423,658 B | 24.05 MB |

These are total-process rows and include exact fixture storage. A separate
input-only/kernel-lifetime measurement remains open; no kernel-only claim is
derived by subtracting unrelated profiler snapshots.

## Source and linked size

Current `src` contains 15,137 Tokei code lines and 603,467 tracked bytes,
78.14% fewer code lines and 76.50% fewer bytes than the Phase 0 source tree.
This checkpoint adds a net 139 physical source lines and 6,349 bytes versus
the atomic-cutover evidence parent while closing two fuzz paths and adding the
retained-fact schedules.

The linked matrix below compares current artifacts with the same-toolchain
atomic-cutover measurement. Native values are ELF `.text`; WASM values are
`wasm-opt --all-features -Oz` bytes.

| Features/profile/consumer | Native text | Optimized WASM | Native delta | WASM delta |
| --- | ---: | ---: | ---: | ---: |
| default/release/general | 1,993,502 | 1,373,669 | +0.0325% | +0.4341% |
| default/release/immediate | 1,996,654 | 1,375,517 | +0.0325% | +0.4344% |
| default/size/general | 1,049,255 | 647,643 | +0.4534% | +0.1184% |
| default/size/immediate | 1,050,203 | 648,041 | +0.4538% | +0.1179% |
| all/release/general | 2,135,587 | 1,454,633 | +0.0247% | +0.3966% |
| all/release/immediate | 2,138,427 | 1,456,600 | +0.0247% | +0.3969% |
| all/size/general | 1,050,791 | 647,655 | +0.4658% | +0.1436% |
| all/size/immediate | 1,051,739 | 647,937 | +0.4646% | +0.1440% |

The largest growth is 0.466%, accepted provisionally for the 4.3–4.6%
large-fixture instruction reduction and newly complete paths. Further size
consolidation remains a Phase 17 task.

## Runtime evidence and removal audit

The dispatch executable covers 20 workloads and emits 1,402 summarized lines:
1,234,049 dispatch events, 80,674 predicate events, and 97,441 rational
temporaries, with zero unknown-fact and zero fallback/abort events.

The exact six-crate call-graph scope is Hypermesh, Hyperreal, Hyperlimit,
Hyperlattice, Hypertri, and CSGRS:

| Graph | Nodes | Edges | Hypermesh nodes | Hypermesh internal edges | Removed namespace nodes |
| --- | ---: | ---: | ---: | ---: | ---: |
| Production | 17,961 | 29,417 | 2,596 | 4,086 | 0 |
| Tests/benches/examples/fuzz | 25,107 | 39,662 | 3,549 | 5,236 | 0 |

Nightly rustdoc reports 282 Hypermesh public items, including 122 callables.
The static evidence matcher finds any-static test/benchmark/fuzz/trace evidence
for 60/35/61/25 callables and qualified evidence for 21/7/9/4. Static matching
is intentionally reported as incomplete; runtime and semantic tests, not name
matching, are the authority.

Current production/tests/examples/benches/fuzz/Cargo manifests and controlled
CSGRS source contain none of the removed public names or namespaces. The only
remaining source-tree mentions are three clearly historical paths/selectors in
the hash-pinned implementation-test migration ledger. No compatibility shim,
deprecated forwarder, feature selector, repair retry, linked old engine, or
second Boolean entry point exists. Hypercurve and HyperSolve were not edited.

## Open work

This is a retained checkpoint, not Phase 17 or Phase 18 completion. The small
exact box and full YeahRight rows remain slower and larger than CGAL EPECK.
The competitive corpus still needs case-by-case CGAL execution beyond the
current exact boxes and established hard-case row; the real/pathological and
medium/large/XL fixture families need further expansion; total versus
kernel-only heap lifetime needs a direct measurement boundary; and the full
CSGRS build remains deferred while its Hypercurve/HyperSolve dependencies are
under concurrent ownership. The goal remains active.

## Reproduction

```sh
cargo test --all-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-Dwarnings' cargo doc --no-deps --all-features
cargo +nightly check --manifest-path fuzz/Cargo.toml --bins
cargo fmt --all -- --check

taskset -c 11 cargo bench --bench competitive -- \
  'competitive_shared_arrangement/hypermesh/overlapping_boxes'
target/competitive/cgal-epeck/hypermesh_cgal_epeck \
  benchmarks/corpus/exact/overlapping-boxes-left.off \
  benchmarks/corpus/exact/overlapping-boxes-right.off all 31 outside

env YEAHRIGHT_BENCH=1 /usr/bin/time -v taskset -c 11 \
  target/release/deps/competitive-92c64513605410c9 \
  full_resolution_yeahright_rotated_intersection_certifies_empty \
  --ignored --exact --nocapture

benchmarks/size-harness/measure.sh default
benchmarks/size-harness/measure.sh all
```
