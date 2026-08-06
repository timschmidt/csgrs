# Phase 17 retained point-insertion topology

Date: 2026-08-05

Status: retained; Phases 11, 17, and 18 remain open

Implementation: Hypertri `824bd3a`, Hypermesh `4aa51efc`

Direct parent: Hypertri `72ddcdc`, Hypermesh `4aa51efc`

## Purpose and algorithm guard

This checkpoint carries one checked exact triangle topology from nontrivial
orientation-only point insertion into constraint recovery. It removes repeated
complete adjacency construction from the dense face-corefinement path without
changing the Boolean algorithm, the exact predicates, the output contract, or
the selected Hyperlimit terminal policy.

The only scheduling crossover is the cardinality of the triangulation already
being processed. Below 16 triangles, the compact sparse adjacency rebuild is
cheaper. At 16 triangles, the implementation constructs one reciprocal
`TriangleTopology` and thereafter updates only the local point split. This is a
general data-structure crossover. There is no branch on fixture identity,
coordinate value or width, Boolean operation, expected result, policy name,
competitor, or measured benchmark outcome. There is no alternate Boolean
engine and no compatibility shim.

The retained path handles every exact point-insertion topology:

- an interior point replaces one triangle with three;
- a point on a hull edge replaces one triangle with two;
- a point on an interior edge replaces two incident triangles with four; and
- a point on an existing vertex remains a typed invalid-input path after exact
  numeric uniqueness checks.

`TriangleTopology::replace_point_region` proves the old and new local boundary,
allows only the authored hull-edge split through the inserted point, checks
manifold incidence and reciprocal ownership, patches every outside neighbor,
and updates vertex representatives atomically. One reusable scratch object owns
old-boundary, edge-use, neighbor, and outside-update storage. Appended triangle
slots are derived from the original triangle count rather than retained in a
second index vector.

The topology is boxed only after the crossover, returned with the point
triangulation, and moved directly into exact constraint recovery and
unconstrained-edge canonicalization. Small faces retain the prior compact path
and do not allocate a topology object.

## Exactness and path coverage

The retained local topology is an optimization of already-proved combinatorial
facts, not an approximate geometric cache. Orientation, point/triangle
classification, point-on-segment, edge legality, and terminal equality remain
owned by Hyperlimit over `hyperreal::Real`.

`STRICT` remains exact-only. `APPROXIMATE_512` can terminate only through
Hyperlimit's approximate 512-bit equality policy, and Hypermesh continues to
aggregate the resulting certainty. The retained topology contains no policy
branch. Every measured row in this checkpoint is `Certified`, so both policies
produce byte-for-byte equal `Real` values, topology, boundaries, and exact
volumes without consuming an approximate terminal.

Focused tests cover all three replacement shapes, complete-adjacency
equivalence, a mixed 25-point interior/boundary sequence, exact convex-hull
coverage, reciprocal edge-flip updates, atomic changed-boundary rejection, and
both policies. The existing adversarial, differential, fuzz-property,
constraint-cavity, policy, and public Boolean suites remain the broader gate.

## Dense exact performance

Measurements are CPU-11-pinned five-run `perf stat` medians over one shared
four-result arrangement of `dense_crossing_grid_17`. The current and saved
parent executables consume identical input, policy, operation set, and output
contract.

| Metric | Direct parent | Current | Change |
| --- | ---: | ---: | ---: |
| instructions | 22,355,999,159 | 21,536,995,472 | -3.663% |
| branches | 2,924,725,135 | 2,817,740,957 | -3.658% |
| cycles | 5,955,766,172 | 5,649,794,117 | -5.137% |
| median Boolean time | 1.404054762 s | 1.334104449 s | -4.982% |

Both executables produce the same `Certified` shared arena and result triangle
counts: 9,464 vertices and 11,308 / 11,908 / 10,764 / 12,452 triangles for
union, intersection, difference, and reverse difference.

Two small-path controls expose the complete trade rather than hiding it. Each
row is five runs of 1,000 retained arrangements against the same direct parent.
Neither fixture reaches the retained-topology crossover.

| Fixture | Instruction change | Branch change | Cycle change | Median time change |
| --- | ---: | ---: | ---: | ---: |
| crossing octahedra | +0.071% | +0.007% | -0.421% | -0.555% |
| affine boxes | +0.085% | +0.177% | +2.774% | +3.080% |

The small instruction movement is the linked-layout and predictable state cost
of carrying the general topology result; the affine clock row remains an open
regression control. It is not addressed with a fixture shortcut. The dense
instruction and allocation reductions justify retaining this clean structural
change while subsequent work continues to recover broad performance and size.

## Large-fixture heap and policy gate

`dense_crossing_grid_65` contains 1,572 input triangles and 16,900 exact grid
crossings. The allocator probe measures the complete kernel and requested
output, not a reduced microkernel. `STRICT` and `APPROXIMATE_512` report the
same values in every row:

| Metric | Direct parent | Current | Change |
| --- | ---: | ---: | ---: |
| total process peak | 178,914,697 B | 178,914,697 B | unchanged |
| incremental kernel peak | 178,125,996 B | 178,125,996 B | unchanged |
| post-Boolean incremental | 57,917,376 B | 57,917,376 B | unchanged |
| output live payload | 57,784,192 B | 57,784,192 B | unchanged |
| retained input-fact growth | 133,184 B | 133,184 B | unchanged |
| allocation calls | 47,668,590 | 32,817,512 | -31.155% |
| deallocation calls | 47,010,732 | 32,159,654 | -31.591% |
| reallocations | 2,531,566 | 2,132,870 | -15.749% |
| cumulative bytes added | 95,666,427,841 | 9,761,722,369 | -89.796% |
| cumulative bytes removed | 95,608,510,465 | 9,703,804,993 | -89.850% |

The useful peak is unchanged because exact output ownership dominates the high
water mark; the large reduction is temporary topology churn. The output is
`Certified` with 73,844 vertices and 164,068 triangles. An uninstrumented
`STRICT` process improves from 80.35 to 69.68 seconds wall time (-13.279%) and
79.90 to 63.97 seconds user time (-19.937%), while maximum RSS moves from
191,856 to 191,772 KiB (-84 KiB) with zero swaps.

The opt-in full-value two-policy gate compares every output `Real`, topology,
boundary balance, and exact six-volume 12,805. It passes in 129.93 seconds
versus 163.90 seconds for the direct parent (-20.726%).

## Profile

A final CPU-11, 999 Hz DWARF profile captured 1,341 samples with none lost in
`target/phase17-retained-point-insertion-topology-final.perf.data`.

| Self time | Symbol |
| ---: | --- |
| 25.41% | `Real::exact_rational_homogeneous_point2_i128` |
| 19.84% | `hyperlimit::orient2d_with_policy` |
| 6.92% | `Real::exact_dyadic_f64` |
| 5.28% | `surface_arrangement::projected_segment` |
| 4.53% | `Rational::denominator` |
| 4.07% | `Rational::numerator` |
| 1.66% | `TriangleTopology::replace_region` |
| 0.98% | `triangle_neighbors` |
| 0.68% | `TriangleTopology::neighbor_across` |

`triangle_neighbors` falls from 7.98% self in the direct-parent profile to
0.98%. The next evidence-led target is exact homogeneous-point construction
and orientation scheduling, where Hyperreal's retained facts can remove work
without copying CGAL's scalar strategy.

## Source and linked size

The three Hypertri source files change by 514 insertions and 99 deletions,
including the complete checked patch machinery and focused tests. The direct
default-feature competitive executable grows 5,964 text bytes (+0.216%).

The canonical size harness was rebuilt from the final source with Rust 1.97.0.
Values are linked text bytes for native artifacts and `wasm-opt -Oz` file bytes
for WASM artifacts.

| Features/profile | Current native general / immediate | Current WASM general / immediate | Parent-relative range |
| --- | ---: | ---: | ---: |
| default release | 1,986,190 / 1,989,342 | 1,407,703 / 1,409,543 | +0.242% to +0.312% |
| default size | 1,091,103 / 1,092,059 | 678,869 / 679,454 | +0.395% to +0.519% |
| all-feature release | 2,121,083 / 2,123,947 | 1,481,754 / 1,483,724 | +0.226% to +0.297% |
| all-feature size | 1,093,439 / 1,094,411 | 678,899 / 679,353 | +0.393% to +0.493% |

The maximum canonical growth is 3,507 bytes / 0.519%. Performance has priority,
but linked-size recovery remains an explicit Phase 17 gate.

## Current CGAL EPECK boundary

The pinned CGAL 6.0.3 EPECK adapter consumed freshly exported reduced-rational
OFF inputs. Every union, intersection, difference, and reverse-difference
output is valid, closed, structurally valid, and agrees with Hypermesh's exact
triangle-count and volume oracle.

The fresh-process protocol uses 63 CGAL iterations in each copy mode and 63
alternating-policy Hypermesh processes on CPU 11. Values are nanoseconds. These
wide clock ranges are frequency-sensitive; the policies use one topology path,
so their small separation is not an algorithmic policy effect.

| Fixture | CGAL outside median (range) | CGAL inside median (range) | Hypermesh STRICT median (range), ratio | Hypermesh APPROXIMATE_512 median (range), ratio |
| --- | ---: | ---: | ---: | ---: |
| crossing octahedra | 116,262 (110,323--333,227) | 117,962 (108,343--348,906) | 545,353 (427,591--874,250), 4.691x | 536,783 (427,321--737,980), 4.617x |
| affine boxes | 379,684 (362,775--582,860) | 381,824 (366,745--645,656) | 996,042 (784,446--1,559,783), 2.623x | 961,074 (787,946--1,490,667), 2.531x |

Five paired retained-input trials use 1,000 arrangements per engine, CGAL
copies outside the timer, one retained Hypermesh `MeshContext`, and unchanged
exact inputs. Median paired ratios are 2.538x for crossing and 1.826x for
affine. Hyperreal's retained ownership narrows the cold gap but does not close
either competitive case; both remain explicit Phase 17 failures.

## Call graph and validation

The final five-crate call graphs contain 15,309 nodes / 25,529 edges in
production, 17,656 / 28,899 with tests and examples, and 21,701 / 34,999 across
all evidence sources. This is +54 nodes / +79 edges in every scope. The
production graph proves the one-engine path:

```text
surface_arrangement::corefine_face
  -> constrained_triangulation_convex_hull
  -> topology::triangulate_point_set
  -> topology::insert_point
  -> TriangleTopology::replace_point_region
  -> insert_constraints_topology
  -> recover_constraints
  -> canonicalize_unconstrained_edges
```

There are zero exact EMBER, `segment_trace`, or `local_bsp` namespace nodes or
source identifiers and one `surface_arrangement` Boolean engine. The call-graph
artifacts are:

- `target/phase17-retained-point-topology-callgraph-production/callgraph.json`
- `target/phase17-retained-point-topology-callgraph-tests/callgraph.json`
- `target/phase17-retained-point-topology-callgraph-all/callgraph.json`

Final validation passes Hypertri's complete all-feature unit, adversarial,
differential, fuzz-property, policy, README, and doctest suites; Hypermesh's
complete all-feature release suite; warning-denied all-target/all-feature
Clippy for both crates; formatting; diff checks; the full large policy gate;
allocator and process probes; the profile; both canonical size matrices; and
all three call graphs.

Phases 11, 17, and 18 remain open. Remaining work includes broader external
real-world and generated pathology coverage, every losing current CGAL row,
stage-specific lifetime attribution, broad small-path performance recovery,
linked-size recovery, further retained Hyperreal fact scheduling, and the final
path-completeness/removal/release audit.
