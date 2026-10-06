# Exact axis-aligned-box cell checkpoint — 2026-07-30

This checkpoint measures Hypermesh commit `e1260726` against its direct parent
`0507e622` with the same current Hyperreal, Hyperlattice, and Hyperlimit
dependencies. The dependency-only immediate consumer is checked in at
`benchmarks/size-harness/src/bin/immediate.rs`.

## Correctness and policy

The exact cell arrangement uses at most 27 occupancy cells and 64 shared grid
vertices, all on the stack. It was exercised over all four Boolean operations
for disjoint, equal, left-contained, right-contained, adjacent, partially
touching, and volume-overlapping boxes. Each of the 28 results checks:

- the public and direct result agree;
- exact signed volume is correct;
- triangles are valid, nondegenerate, and unique;
- directed boundary uses cancel, including closed-PWN non-manifold cases; and
- original versus synthesized provenance is explicit.

The box fact now rejects duplicated half-faces and inward face winding.
A terminal equality fixture proves that `STRICT` declines the shortcut without
consuming approximation, while `APPROXIMATE_512` completes it and returns
`Approximate512Consumed`. Approximate coordinate orders are checked for
pairwise consistency before materialization.

## Runtime

Criterion was pinned to CPU 0. Values are medians from the final four-operation
run.

| Immediate certified box Boolean | Median |
| --- | ---: |
| Union | 3.5641 µs |
| Intersection | 1.5434 µs |
| Difference | 2.6468 µs |
| Symmetric difference | 3.8192 µs |

The union is 173.56× faster than Hypermesh's 618.60 µs historical row, 18.18×
faster than the 64.777 µs boolmesh reference, and 17.47× faster than the
62.279 µs manifold-rust reference. The competitors remain throughput
references, not exactness or policy oracles.

## Dispatch and memory

| Union execution evidence | Parent general path | Exact cell path |
| --- | ---: | ---: |
| Dispatch events | 31,797 | 234 |
| Predicate events | 96 | 0 |
| Exact `Real` comparisons | 7,230 | 213 |
| Rational temporaries | 6,712 | 0 |
| Cache events | — | 0 |

Heaptrack used identical release consumers, including process setup and output:

| Memory evidence | Parent | Exact cell | Change |
| --- | ---: | ---: | ---: |
| Allocation calls | 7,028 | 45 | -99.36% |
| Peak heap | 346.70 KiB | 94.61 KiB | -72.71% |
| Returned vertices | 20 | 44 | +24 |
| Returned triangles | 36 | 84 | +48 |

The arrangement eliminates nearly all transient work, but its unit-cell
boundary retains more output rows than the polygon path on the diagonally
overlapping-box fixture. Exact coplanar boundary coalescing is therefore still
a measured retained-memory optimization target; it must preserve shared edge
segmentation and closure.

## Linked artifact size

The same runtime-selected immediate consumer was built at the parent and
checkpoint revisions. Percentages are checkpoint growth over the parent.
The isolated A/B harness used the checked-in source verbatim under one stable
package name at both revisions, so Cargo package metadata did not affect the
comparison.

| Profile | Target | Parent raw | Current raw | Raw change | Parent `.text` / `wasm-opt -Oz` | Current `.text` / `wasm-opt -Oz` | Code change |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Release | Native | 4,316,600 | 4,353,864 | +0.863% | 3,702,373 | 3,737,509 | +0.949% |
| Release | WASM | 3,332,846 | 3,353,967 | +0.634% | 2,590,840 | 2,608,785 | +0.693% |
| Size | Native | 1,907,088 | 1,921,712 | +0.767% | 1,671,031 | 1,685,063 | +0.840% |
| Size | WASM | 1,275,767 | 1,288,414 | +0.991% | 1,084,570 | 1,096,003 | +1.054% |

Machine-readable raw, compressed, optimized, runtime, dispatch, and memory
values are in `exact-box-cell-2026-07-30.toml`.

## Verification

- `cargo test --all-features`: all 1,150 executed tests passed; 7 ignored.
- `cargo test --no-default-features`: all 1,148 executed tests passed; 7 ignored.
- all-target checks passed with all features and without default features.
- every fuzz target compiled.
- the dependency-only general and immediate size consumers compiled.
- formatting and `git diff --check` passed.
