# Sparse exact-CDT adjacency checkpoint — 2026-08-03

This checkpoint compares Hypertri `42f82979cf4e` with its direct parent
`c47601266e0b` on the same dependency revisions and host recorded in the
companion TOML file. It supports Phases 13–14 of the workspace-root
`HYPERMESH_PATH_COMPLETE_IMPLEMENTATION_PLAN.md`; it does not claim that the
Hypermesh surface-arrangement engine or the CGAL EPECK parity gate is complete.

## Result

Located Bowyer-Watson insertion rebuilt triangle adjacency after every point by
materializing all three directed edge occurrences and sorting them. The direct
parent therefore paid an `O(T log T)` rebuild even though the active
super-triangle complex is one connected triangulated disk.

The implementation now builds one deterministic sparse open-addressed table of
unique undirected edges. A compact 24-byte slot stores the canonical endpoints
and an encoded triangle/edge owner; a paired sentinel avoids retaining a second
edge record. Capacity follows the exact disk invariant and every overflow,
occupancy violation, and third edge incidence returns a typed error. Focused
tests prove reciprocal half-edge construction and non-manifold rejection.

Adjacency construction makes no scalar decision and therefore introduces no
new policy terminal. The surrounding Delaunay operation still routes
orientation, in-circle, equality, and validation through its existing
`ExactKernel` and one Hyperlimit accumulator. The complete STRICT and
APPROXIMATE_512 policy suite passes; exact-rational outcomes remain Certified.

## Pinned large exact-set performance

The fixture is the permanent 400-point located Delaunay set in
`cdt::tests::located_cavity_triangulates_and_validates_large_exact_set`.
Callgrind 3.27.0 used branch simulation with cache simulation disabled, and
Massif used allocation-byte time with every snapshot detailed.

| Counter | Direct parent | Current | Change |
| --- | ---: | ---: | ---: |
| Instructions | 198,595,355 | 89,325,448 | 55.02% fewer |
| Branches | 33,027,262 | 19,660,067 | 40.47% fewer |
| Branch mispredicts | 1,778,211 | 486,058 | 72.67% fewer |
| Useful peak heap | 400,798 B | 376,151 B | 6.15% lower |
| Total peak heap | 436,264 B | 411,512 B | 5.67% lower |

The Criterion row was pinned to core 11. Its 5.2675 ms median
(`5.2331–5.3064 ms`) is 67.86% below the July 16.390 ms checkpoint and 86.50%
below the retained 39.026 ms historical row. Callgrind is the direct-parent
comparison; the older Criterion rows measure the same input but also include
the intervening exact-scalar stack improvements.

## Dependency-only artifact size

### CDT consumer

| Profile/artifact | Direct parent | Current | Change |
| --- | ---: | ---: | ---: |
| Release native file | 1,540,968 | 1,535,704 | -0.34% |
| Release native `.text` | 1,129,958 | 1,125,926 | -0.36% |
| Release raw WASM | 932,782 | 926,146 | -0.71% |
| Release `wasm-opt -Oz` | 701,789 | 697,013 | -0.68% |
| Size native file | 872,640 | 869,296 | -0.38% |
| Size native `.text` | 652,439 | 649,239 | -0.49% |
| Size raw WASM | 405,539 | 402,972 | -0.63% |
| Size `wasm-opt -Oz` | 341,778 | 342,176 | +398 B (+0.12%) |

### All-algorithm/runtime-selection consumer

| Profile/artifact | Direct parent | Current | Change |
| --- | ---: | ---: | ---: |
| Release native file | 1,780,792 | 1,775,464 | -0.30% |
| Release native `.text` | 1,343,690 | 1,339,682 | -0.30% |
| Release raw WASM | 1,176,748 | 1,170,305 | -0.55% |
| Release `wasm-opt -Oz` | 814,960 | 810,283 | -0.57% |
| Size native file | 959,760 | 956,784 | -0.31% |
| Size native `.text` | 735,619 | 732,787 | -0.38% |
| Size raw WASM | 482,287 | 479,951 | -0.48% |
| Size `wasm-opt -Oz` | 409,465 | 409,551 | +86 B (+0.02%) |

All speed-profile and raw size-profile artifacts shrink. The two optimized
size-profile WASM movements are retained rather than rounded away; performance
has priority, and the net linked-code evidence is favorable.

## Rejected Hypermesh whole-domain CDT experiment

This primitive was discovered while prototyping replacement of Hypermesh's
custom coplanar-output planarization with a full Hypertri CDT rebuild. The
prototype preserved topology and aggregate certainty under both policies on
the 6,144-triangle large fixtures. Useful peak heap was unchanged at
28,730,032 B on the general fixture and 1,063,718 B on the certified fixture.

It nevertheless failed the dense-repair performance gate:

| Dense crossing-discovery path | Instructions | Versus retained path |
| --- | ---: | ---: |
| Retained Hypermesh face-local path | 369,110,783 | baseline |
| Whole-domain Hypertri CDT | 2,746,096,252 | +643.98% |
| Sparse-adjacency CDT prototype | 1,425,968,819 | +286.33% |

The Hypermesh change was rejected and completely reverted. A global
convex-hull Delaunay rebuild performs work outside the authored output domain
and remains the wrong Phase-14 architecture even after removing the dominant
sort. Phase 14 must construct conforming exact face cells over the retained
intersection graph, using Hypertri only as a bounded triangulation primitive
where its domain matches the cell being emitted.

## Correctness, fuzzing, and call graph

- All 115 all-feature runtime tests and 4 doctests pass.
- Warning-denied all-target/all-feature Clippy and rustdoc pass.
- The final ASan/libFuzzer topology target executed 11,783 inputs in 31 seconds
  from the 966-file exact corpus, reached coverage 4,141/features 15,041, and
  found no failure. Leak detection was disabled because LeakSanitizer cannot
  run under the managed process tracer; AddressSanitizer remained enabled.
- Native and WASM minimal/all-feature builds, fuzz-bin checks, and the release
  benchmark build pass.
- The regenerated five-crate graph contains 20,215 nodes and 40,325 edges;
  Hypertri's per-library slice contains 1,298 nodes and 1,892 edges.
- The production checkpoint is +99/-22 source lines. No Hypermesh production
  change from the rejected experiment remains.

## Reproduction

```sh
cargo test --locked --all-features
cargo clippy --locked --all-features --all-targets -- -D warnings
RUSTDOCFLAGS=-Dwarnings cargo doc --locked --all-features --no-deps
ASAN_OPTIONS=detect_leaks=0 cargo +nightly fuzz run topology_invariants -- \
  -max_total_time=30
taskset -c 11 cargo bench --locked --bench exact \
  --features all-algorithms,runtime-select -- \
  exact_delaunay_400_located_insertions
./benchmarks/size-harness/measure.sh cdt
./benchmarks/size-harness/measure.sh all
```

The call graph was regenerated from the workspace root with:

```sh
tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir /tmp/hypertri-sparse-adjacency-callgraph \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --per-library \
  --format json
```
