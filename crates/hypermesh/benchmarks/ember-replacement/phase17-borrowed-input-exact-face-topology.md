# Phase 17 borrowed-input exact face topology

Date: 2026-08-05

Status: accepted checkpoint; Phases 17 and 18 remain open

Implementation: Hypertri `9e9b7ef`, Hypermesh `6dddeba2`

Direct parent/evidence: Hypertri `73a3d9e3`, Hypermesh `43fc1bcd`

## Result

The checked Hypertri entry point used by every changed Hypermesh source face
already received a borrowed exact point slice and a borrowed planar constraint
slice, performed the complete input and output validation, and inserted no
Steiner points. It nevertheless cloned the exact points once and the constraints
twice into a general `ConstrainedTriangulation` result. Hypermesh then used only
the returned triangle indices.

Hypertri now returns the checked `Vec<Triangle>` topology directly. Its public
contract states that the indices address the caller's point slice, and its
existing exact validators still establish unique input points, a planar PSLG,
valid indices, positive triangles, complete convex-hull coverage, and the
presence of every constraint edge before returning.

Hypermesh consumes that narrower result directly. It clears and reuses the
already allocated, sorted arrangement-point ID schedule, transfers the
corresponding `ExactPoint` values into Hypertri's input vector, and maps the
returned local indices through the reused ID schedule. Moving the exact
`hyperreal::Real` coordinates preserves their retained facts and avoids a
second exact-coordinate owner. The source-face orientation proof borrows the
same moved values after Hypertri returns.

This is one general ownership and API correction. It does not inspect a
fixture, size, coordinate width, operation, topology, result, policy name,
competitor, or benchmark. There is no threshold, compatibility shim, alternate
engine, or incomplete fast path. Hypercurve and HyperSolve have no caller of
the changed API and remain untouched.

## Exactness, completeness, and policy

- Hypertri still validates constraint indices, unique exact points, complete
  planar constraint geometry, recovered constraints, positive orientation, and
  final constrained convex-hull topology under the caller's policy.
- No construction, predicate, retry, fallback, or validation schedule was
  removed. Only redundant result ownership was removed.
- Triangle indices cannot refer to a hidden result-owned point set; the checked
  API returns only indices into the borrowed caller slice and never inserts a
  Steiner point.
- Hypermesh retains the exact point objects themselves rather than reconstructing
  them. Hyperreal's retained structural and arithmetic facts therefore remain
  available to the downstream source-orientation proof.
- Every capacity reservation and global-to-local translation remains checked
  and typed. Missing arrangement IDs still produce
  `SurfaceArrangementFailed`; reservation failure still produces
  `CapacityOverflow`.
- `STRICT` remains exact-only. `APPROXIMATE_512` can terminate only in
  Hyperlimit, and aggregate certainty is absorbed before Hypermesh consumes the
  topology. The depth-128 symbolic control still reports
  `Approximate512Consumed`; all exact large rows remain `Certified`.

An intermediate implementation moved exact points but allocated a second ID
vector. It improved runtime while adding one allocation per changed face. The
retained form reuses the pre-existing ID vector and recovers all of that
allocation cost. The earlier ownership-only prototype and the extra-vector
prototype are absent from production.

## Deterministic retired-work measurements

The current release probe is compared directly with the saved `43fc1bcd`
executable. Measurements are CPU-11-pinned `perf stat -r 3` instruction and
branch counts. Every output summary and certainty is identical.

| Fixture / workload | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| crossing octahedra, all four x1000 | 2,934,169,930 | 2,891,218,303 | -1.464% | 502,808,936 | 492,931,474 | -1.964% |
| affine boxes, all four x1000 | 6,922,022,653 | 6,841,402,361 | -1.165% | 1,192,635,961 | 1,173,401,314 | -1.613% |
| sparse 512 shells, all four x5 | 2,645,206,539 | 2,603,240,577 | -1.586% | 474,270,159 | 464,566,045 | -2.046% |
| dense coplanar 32, all four x1 | 9,921,940,124 | 9,846,370,964 | -0.762% | 1,689,388,655 | 1,671,099,939 | -1.083% |
| clipped voxel torus 33, all four x3 | 2,623,902,085 | 2,620,895,878 | -0.115% | 439,811,535 | 439,469,506 | -0.0778% |
| 2,049-bit wide boxes, union x5 | 11,796,309,821 | 11,795,647,823 | -0.00561% | 2,173,196,800 | 2,173,053,738 | -0.00658% |
| full YeahRight instrumented kernel x1 | 10,602,595,275 | 10,564,860,172 | -0.356% | 1,790,804,982 | 1,782,367,806 | -0.471% |
| symbolic depth 1, `STRICT`, all four x20 | 4,276,931,047 | 4,275,583,325 | -0.0315% | 936,271,091 | 933,999,224 | -0.2426% |
| symbolic depth 128, `APPROXIMATE_512`, all four x5 | 1,504,899,064 | 1,504,246,957 | -0.0433% | 274,750,543 | 274,531,456 | -0.0797% |

The first symbolic row remains `Certified`; the second remains
`Approximate512Consumed`. The broad improvement is consistent with removing
general exact-object copies rather than favoring one corpus topology.

## Large-fixture heap matrix

All fifteen selectors ran in fresh processes under both policies. Every policy
pair has byte-identical output, certainty, peak, allocation count,
reallocation count, and allocated bytes. Every peak is byte-identical to the
direct parent.

| Selector | Peak bytes | Parent/current calls | Parent/current cumulative bytes |
| --- | ---: | ---: | ---: |
| boxes-3072 | 12,383,968 | 16,637 / 16,541 | 15,498,934 / 15,482,550 |
| boxes-3072-general | 12,630,096 | 35,823 / 35,727 | 17,828,782 / 17,812,398 |
| dense-coplanar-16 | 14,893,071 | 240,070 / 221,638 | 57,283,178 / 53,940,842 |
| dense-coplanar-32 | 59,391,879 | 959,208 / 885,480 | 228,990,666 / 215,621,322 |
| sparse-shells-512 | 11,888,929 | 201,188 / 195,044 | 31,502,760 / 30,077,352 |
| self-PWN-clusters-512 | 12,284,773 | 203,182 / 197,038 | 32,098,566 / 30,673,158 |
| wide-rational-64 | 17,861,062 | 382,021 / 381,925 | 31,536,190 / 31,519,806 |
| wide-rational-512 | 18,750,079 | 1,292,835 / 1,292,739 | 103,823,278 / 103,806,894 |
| wide-rational-2048 | 27,423,760 | 1,564,899 / 1,564,803 | 384,228,862 / 384,212,478 |
| voxel-torus-33 | 12,784,184 | 27,430 / 27,040 | 22,712,960 / 22,608,640 |
| voxel-torus-65 | 51,784,828 | 100,979 / 100,205 | 104,232,547 / 104,025,315 |
| yeahright | 5,161,051 | 166,768 / 166,504 | 11,276,143 / 11,205,743 |
| yeahright-4 | 19,676,989 | 727,161 / 726,609 | 45,338,168 / 45,190,968 |
| yeahright-8 | 76,258,109 | 3,313,269 / 3,312,165 | 187,244,826 / 186,950,426 |
| yeahright-full-rotated | 59,440,454 | 9,911,765 / 9,906,413 | 584,399,479 / 582,162,615 |

Dense-32 removes 73,728 allocation calls and 13,369,344 bytes of cumulative
traffic. The full 23,788-triangle row removes 5,352 calls and 2,236,864 bytes;
its incremental kernel peak remains 52,317,092 bytes.

Heaptrack artifact
`target/phase17-borrowed-input-topology-full-strict.zst.zst` reports an exact
59,514,182-byte peak, unchanged from the parent, 11,627,764 allocation-function
calls (-5,352), and 2,669,159 temporary allocations (unchanged). Peak stacks
are `/tmp/hypermesh-borrowed-input-topology-peak.stacks`.

## Historical and competitive boundaries

The direct historical full-resolution boundaries remain EMBER at 3,312.66 s /
329,352 KiB, historical CGAL at 0.09 s / 15,516 KiB, and the preceding current
Hypermesh advisory process at 1.13 s / 68,864 KiB. This checkpoint removes
0.356% of full instrumented instructions but does not claim a new full
wall-clock ratio from a sub-percent change on a frequency-variable host.

Pinned CGAL 6.0.3 EPECK was refreshed from exact rational OFF for 63 internal
repetitions in each copy mode. Hypermesh used 63 fresh CPU-11-pinned processes
per policy. These absolute medians gauge the remaining competitive boundary;
deterministic counters, exact outputs, policy evidence, and heap are the
checkpoint gates.

| Fixture | CGAL outside / inside | Hypermesh `STRICT` | Ratio to outside | Hypermesh `APPROXIMATE_512` | Ratio to outside |
| --- | ---: | ---: | ---: | ---: | ---: |
| crossing octahedra | 115,432 / 120,011 ns | 407,402 ns | 3.529x | 409,192 ns | 3.545x |
| affine boxes | 370,524 / 375,384 ns | 779,916 ns | 2.105x | 781,116 ns | 2.108x |

Hypermesh ranges were 389,774--611,008 ns, 390,893--590,489 ns,
745,059--1,247,754 ns, and 746,239--3,793,629 ns respectively. CGAL ranges
were 109,993--337,517 ns, 108,143--343,936 ns, 360,745--555,592 ns, and
364,215--530,684 ns. Every per-case competitive gap remains open; no aggregate
or favorable clock sample is substituted for a losing case.

## Code, binary size, and call graph

Across both crates the implementation and directly adjusted tests/fuzz target
change 58 lines in and 62 lines out. Production changes are 55 insertions and
49 deletions. The breaking API replaces redundant ownership rather than
shipping a forwarding result type.

| Configuration | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| default release native general text | 1,959,578 | 1,957,586 | -1,992 |
| default release native immediate text | 1,962,722 | 1,960,730 | -1,992 |
| default release optimized WASM general | 1,382,028 | 1,381,518 | -510 |
| default release optimized WASM immediate | 1,383,876 | 1,383,359 | -517 |
| default size native general text | 1,068,847 | 1,068,935 | +88 |
| default size native immediate text | 1,069,811 | 1,069,891 | +80 |
| default size optimized WASM general | 662,218 | 661,997 | -221 |
| default size optimized WASM immediate | 662,648 | 662,410 | -238 |
| all-feature release native general text | 2,095,063 | 2,093,063 | -2,000 |
| all-feature release native immediate text | 2,097,919 | 2,095,919 | -2,000 |
| all-feature release optimized WASM general | 1,455,985 | 1,455,501 | -484 |
| all-feature release optimized WASM immediate | 1,457,954 | 1,457,473 | -481 |
| all-feature size native general text | 1,071,095 | 1,071,175 | +80 |
| all-feature size native immediate text | 1,072,059 | 1,072,147 | +88 |
| all-feature size optimized WASM general | 662,267 | 662,255 | -12 |
| all-feature size optimized WASM immediate | 662,541 | 662,294 | -247 |

Performance-priority release native and WASM artifacts all shrink. The
80--88-byte size-native growth is retained because every deterministic runtime
control improves, heap traffic falls, and the breaking result type removes
ownership and API surface.

The regenerated five-crate graphs contain 15,140 nodes / 25,245 edges for
production, 17,460 / 28,561 with tests and examples, and 21,505 / 34,661 with
all tests, examples, benches, and fuzz targets. Relative to the direct parent,
those are reductions of 1/3, 1/3, and 2/4 nodes/edges. There is one exact
`corefine_face -> constrained_triangulation_convex_hull` edge; the topology
entry point reaches its exact final validator and has no edge to
`ConstrainedTriangulation::from_parts_with_constraint_edges`. General
Delaunay-quality entry points retain their distinct result ownership. Removed
EMBER, subdivision-engine, segment-trace, and local-BSP namespaces remain
absent. Hypercurve and HyperSolve are excluded.

## Validation and reproduction

Hypermesh passes 200 default tests with six opt-in ignores, 201 all-feature
tests with six ignores, and 153 minimal library tests. Hypertri passes its
default/all-algorithm suites and the complete 256-combination feature-power-set
check and test matrices. Both crates pass warning-denied all-target/all-feature
Clippy and rustdoc, fuzz-target checks, bench/example checks, formatting, and
diff checks. Hypertri's UI test also passes.

```sh
cargo test --locked
cargo test --locked --all-features
cargo test --locked --lib --no-default-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS=-Dwarnings cargo doc --locked --all-features --no-deps
cargo check --locked --manifest-path fuzz/Cargo.toml --all-targets
cargo check --locked --benches --examples --all-features
cargo hack check --feature-powerset --exclude-features all-algorithms --all-targets
cargo hack test --feature-powerset --exclude-features all-algorithms
benchmarks/size-harness/measure.sh default
benchmarks/size-harness/measure.sh all
```

Phases 17 and 18 remain open for external real-world corpus completion, every
remaining per-case CGAL gap, additional clean arrangement/scalar scheduling,
and the final removal/policy/caller audit.
