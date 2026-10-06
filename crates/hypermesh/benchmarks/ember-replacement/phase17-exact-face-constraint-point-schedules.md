# Phase 17: exact face-constraint point schedules

Date: 2026-08-04

Status: accepted

Implementation: `c0d52f7b73f7dbf927e9eeea2ee646e9e58d4cfa`

Paired parent: `d103dcf8d484995343de580c38e6ea5eb7b532a9`

## Result

Face corefinement now constructs each exact constraint's point schedule with
its two defining endpoints already present and reuses one fallible scratch
allocation for every constraint on the face. Only other projected points need
the policy-aware exact point-on-closed-segment predicate that discovers
crossings, contacts, and overlaps.

This is a construction invariant, not a guessed geometric shortcut. Degenerate
constraints are removed before `RawConstraint` construction, both retained
endpoints are checked to exist in the face's projected point map, and an
authored segment is definitionally the closed segment between those endpoints.
The prior loop rediscovered the same two facts by evaluating its endpoint
predicate. Exact sorting still canonicalizes the complete resulting set, so
iteration and insertion order do not affect topology.

The scratch vector reserves the number of projected face points once through
`try_reserve_exact`, reports `CapacityOverflow` before mutation if allocation
cannot be represented, clears without losing capacity, and can never contain
more than that reserved number of unique point IDs. Every non-endpoint route
still reaches the unchanged complete exact predicate and every later split,
CDT, radial, winding, and output certification path is unchanged.

There is no fixture, mesh size, component count, Boolean operation, expected
result, competitor, policy name, topology, or output branch. No carrier, cache,
dependency, feature, public API, compatibility surface, or alternate engine is
added. The implementation adds fourteen lines and removes one line of Rust.

`STRICT` therefore remains exact-only. `APPROXIMATE_512` remains available only
at Hyperlimit's terminal decision and is still aggregated if any remaining
non-endpoint incidence test consumes it.

## Paired protocol

The committed parent executable was retained separately and compared with the
current executable on CPU 11, Rust 1.97.0, the same release profile, lockfile,
fixture construction, input PWN prime, and current Hyper stack. Three
`perf stat` processes report retired user instructions and branches for the
whole process. The fixture/import prime is inside the counters and amortized
over the stated Boolean repetitions.

## Retired work

| Fixture / workload | Parent instructions | Current instructions | Movement | Parent branches | Current branches | Movement |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| sparse 8, `all` x100 | 1,004,931,478 | 929,187,510 | -7.537% | 183,040,375 | 171,973,732 | -6.046% |
| sparse 64, `all` x20 | 1,705,229,624 | 1,583,118,694 | -7.161% | 312,037,033 | 294,944,516 | -5.478% |
| sparse 512, `all` x5 | 3,751,762,174 | 3,510,641,996 | -6.427% | 690,381,417 | 654,784,180 | -5.156% |
| ordinary overlapping boxes, `all` x1000 | 5,027,744,462 | 4,716,159,792 | -6.197% | 882,117,599 | 837,183,845 | -5.094% |
| torus 33, `all` x3 | 2,908,720,768 | 2,899,121,949 | -0.330% | 498,746,407 | 498,088,512 | -0.132% |
| torus 65, `all` x1 | 4,727,570,484 | 4,721,093,152 | -0.137% | 811,363,150 | 811,072,902 | -0.036% |
| subdivided boxes, union x5 | 4,445,861,799 | 4,398,720,901 | -1.060% | 770,401,491 | 763,410,893 | -0.907% |
| dense coplanar 32, `all` x1 | 11,867,423,054 | 11,003,524,059 | -7.280% | 2,028,346,077 | 1,894,405,557 | -6.603% |
| wide rational 2,049-bit, union x5 | 14,047,214,473 | 13,919,478,158 | -0.909% | 2,563,418,095 | 2,537,133,583 | -1.025% |
| full rotated YeahRight | 13,661,181,260 | 13,255,354,775 | -2.971% | 2,390,074,727 | 2,318,691,378 | -2.987% |

Every measured row improves in both counters. The gain follows face-constraint
incidence density: it is largest on dense coplanar work and remains material on
sparse shells, ordinary boxes, and the full external mesh. Independent torus,
subdivided, and wide-rational controls also improve, which rejects a
fixture-specific explanation.

One-process sparse-512 Callgrind falls from 861,194,927 to 812,872,428
instructions (-5.611%). Inclusive `build_surface_arrangement` work falls from
631,603,902 to 583,280,912 (-7.651%), and `corefine_surface` falls from
418,808,194 to 370,491,357 (-11.537%). Exact point-on-segment calls fall from
49,152 to 30,720. The current profile is
`target/phase17-face-constraint-schedule-sparse-512.callgrind`.

Corefinement remains the largest arrangement target. Its current profile still
contains about 149.25 million inclusive instructions in Hypertri constrained
triangulation, so that path remains open for clean retained-fact and scheduling
work rather than workload-specific bypasses.

## Paired clocks and CGAL boundary

Eleven adjacent parent/current pairs were alternated in order on CPU 11. The
table reports independent batch medians and the median of the eleven paired
current/parent ratios. Pairing is used because unpaired clocks on this host
showed visible frequency drift; retired work above remains the primary
acceptance signal.

| Shells | Policy | Parent median | Current median | Median paired movement |
| ---: | --- | ---: | ---: | ---: |
| 8 | STRICT | 0.920 ms | 0.867 ms | -6.606% |
| 8 | APPROXIMATE_512 | 0.892 ms | 0.846 ms | -5.671% |
| 64 | STRICT | 7.133 ms | 6.896 ms | -2.761% |
| 64 | APPROXIMATE_512 | 6.991 ms | 6.693 ms | -4.731% |
| 512 | STRICT | 61.072 ms | 58.386 ms | -4.800% |
| 512 | APPROXIMATE_512 | 61.738 ms | 58.843 ms | -3.922% |

The exact OFF inputs and CGAL 6.0.3 EPECK executable are unchanged from the
immediately preceding fresh 126-record run, whose 21-call outside-copy medians
are 0.288, 2.027, and 18.228 ms. Using the current Hypermesh batch medians,
`STRICT` remains 3.01x, 3.40x, and 3.20x slower; `APPROXIMATE_512` remains
2.94x, 3.30x, and 3.23x slower. All CGAL rows and all four outputs in each row
were valid, closed, and structurally valid. The CGAL parity gate remains open.

## Heap

All fourteen designated large selectors ran under both policies on the final
implementation. Every pair produced identical geometry, a byte-identical
allocator row, and `Certified` aggregate certainty.

The reusable scratch primarily removes allocation churn, not retained state.
Representative paired parent/current results are:

| Fixture | Parent / current total peak | Parent / current kernel peak | Allocation calls | Added payload |
| --- | ---: | ---: | ---: | ---: |
| sparse 512 | 15,540,295 / 15,540,315 B | 14,578,138 / 14,578,158 B | 470,296 / 463,128 | 50,905,532 / 50,801,084 B |
| dense coplanar 32 | 78,750,901 / 78,750,901 B | 74,724,848 / 74,724,848 B | 2,049,708 / 1,926,828 | 326,293,786 / 324,327,706 B |
| wide rational 2,049-bit | 31,898,360 / 31,898,360 B | 31,234,658 / 31,234,658 B | 1,947,027 / 1,923,383 | 438,743,478 / 432,389,422 B |
| full rotated YeahRight | 165,381,598 / 165,353,622 B | 158,258,204 / 158,230,228 B | 15,558,650 / 14,972,165 | 863,068,648 / 834,548,872 B |

The sparse peak grows by only the scratch vector's 20-byte requested payload;
dense and wide peaks are unchanged, and the full external fixture drops 27,976
bytes. Allocation calls fall by 7,168, 122,880, 23,644, and 586,485 in those
four rows. Full-fixture cumulative allocation falls by 28.52 MB.

## Source and binary size

Canonical linked-size movement is bounded to 264 native-text bytes and 166
optimized-WASM bytes in release, and 208/106 bytes in the size profile:

| Profile / consumer | Parent native `.text` | Current native `.text` | Parent `wasm-opt -Oz` | Current `wasm-opt -Oz` |
| --- | ---: | ---: | ---: | ---: |
| release / general | 1,984,718 B | 1,984,454 B | 1,408,990 B | 1,409,155 B |
| release / immediate | 1,987,862 B | 1,987,598 B | 1,410,857 B | 1,411,023 B |
| size / general | 1,080,047 B | 1,080,247 B | 674,493 B | 674,595 B |
| size / immediate | 1,081,003 B | 1,081,211 B | 674,902 B | 675,008 B |

The small mixed movement is retained because broad deterministic runtime and
allocation work improve materially, performance has priority, and the proof
and implementation remain compact.

## Call graph and path audit

The five-crate production graph contains 14,908 nodes and 24,870 edges. The
examples/tests graph contains 17,190 nodes and 28,097 edges. Hypercurve and
HyperSolve are excluded. The singular production route remains
`build_surface_arrangement -> corefine_surface -> corefine_face ->
planar_point_on_segment`; there is no alternate engine or policy-free route.
Artifacts are under
`target/callgraph-hypermesh-face-constraint-schedule{,-production}`.

## Validation

The final default suite executes 185 tests and the all-feature suite executes
186; all pass, with six documented external/manual YeahRight tests ignored in
ordinary runs. Warning-denied all-target/all-feature Clippy, no-default
checking, warning-denied rustdoc, all fuzz binaries, all benchmark targets,
formatting, diff checks, the canonical size harness, both call graphs, the
focused 33-test surface suite, Callgrind, paired perf/clocks, and every
two-policy large heap selector pass.

## Reproduction

```sh
cargo test --locked --all-features
cargo clippy --locked --all-targets --all-features -- -D warnings

taskset -c 11 perf stat -r 3 -x, \
  -e instructions:u,branches:u \
  target/release/examples/competitive_arrangement_probe \
  sparse_multishell_tetrahedra_512 all strict 5

target/release/examples/large_mesh_kernel_heap_probe \
  sparse-shells-512 approximate-512

benchmarks/size-harness/measure.sh default

tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir hypermesh/target/callgraph-hypermesh-face-constraint-schedule \
  --format json \
  --crate-name hypermesh,hyperreal,hyperlimit,hypertri,hyperlattice \
  --include-examples --include-tests --per-library
```
