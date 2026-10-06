# Phase 17 retained face-line orientations

Date: 2026-08-05

Status: retained; Phases 11, 17, and 18 remain open

Implementation: Hyperlimit `c2ca7be`, Hypertri `824bd3a`, Hypermesh
`13a3f83e`

Direct parent: Hyperlimit `d2dee8fb`, Hypertri `824bd3a`, Hypermesh
`c860e09a`

## Purpose and algorithm guard

The exact face-arrangement core repeatedly asks which side of one authored
projected line contains many projected points. The direct parent rebuilt the
fixed-endpoint dyadic and exact-word orientation schedules for every query.
This checkpoint compiles Hyperlimit's existing `Line2Orientation` once per
authored line and reuses it for pairwise proper-crossing tests and closed
point-on-segment incidence.

Retained evidence is enabled only when the points already present on the face
prove a conservative floor of sixteen non-endpoint queries per authored line.
Crossing points discovered later and pairwise line tests can only add reuse;
they are deliberately excluded from the gate. Below that structural crossover,
the original direct predicate path remains in use.

The crossover depends only on exact work already required by the general face
arrangement. There is no branch on fixture identity, coordinate values or
width, Boolean operation, expected result, policy name, competitor, or measured
output. The arrangement algorithm, crossing early exits, incidence rules,
triangulation, winding, and output construction are unchanged. There is one
Boolean engine and no compatibility shim.

A lower three-query experiment was rejected. It reduced dense-17 work slightly
further (15,483,312,974 instructions) but eagerly compiled evidence on ordinary
small faces and raised the crossing control to 3,357,932,962 instructions,
about fifteen percent above the direct parent. The retained threshold of
sixteen gives up roughly one half percent on dense-17 to keep the rule a clean
measured amortization boundary and ordinary inputs on the compact path.

## Exactness and policy

Hyperlimit now exposes retained-orientation variants of complete point/segment
classification and closed point-on-segment incidence. They enter the same
cascade as the direct predicate:

1. certified dyadic determinant filter;
2. checked exact-word homogeneous determinant;
3. certified rational determinant filter;
4. arbitrary-precision exact rational kernel; and
5. the existing policy-owned `Real` refinement fallback.

The retained package schedules proofs; it is not itself an incidence fact.
Every inconclusive filter falls through to the same complete path. Interval
containment remains an independent exact, policy-aware decision.

`STRICT` remains exact-only. `APPROXIMATE_512` may terminate only in
Hyperlimit's approximate 512-bit terminal, and Hypermesh continues to aggregate
certainty from every decision. On the large dense gate both policies remain
`Certified` and compare equal for every output `Real`, triangle, boundary
balance, and exact volume.

Hyperlimit's full all-feature suite passes 230 tests plus doctests, including a
new direct-versus-retained point/segment matrix over non-dyadic exact rationals,
all four point/segment location classes, and both policies. Hypermesh's full
release all-feature suite passes 207 tests plus doctests, with seven manual
stress tests ignored; its small
arrangements exercise the direct schedule and the permanent dense-crossing
family exercises retained scheduling.

## Dense exact performance

Measurements are five serialized CPU-11-pinned `perf stat` runs of one shared
union/intersection/difference/reverse-difference arrangement over
`dense_crossing_grid_17`. Medians are compared with the immediately preceding
retained-point-topology checkpoint.

| Metric | Direct parent | Current | Change |
| --- | ---: | ---: | ---: |
| instructions | 21,536,995,472 | 15,563,736,923 | -27.735% |
| branches | 2,817,740,957 | 2,144,853,300 | -23.880% |
| cycles | 5,649,794,117 | 4,402,175,133 | -22.083% |
| median Boolean time | 1.334104449 s | 1.043314770 s | -21.797% |

Both executables produce the same `Certified` shared arena and result triangle
counts: 9,464 vertices and 11,308 / 11,908 / 10,764 / 12,452 triangles.

Two five-run, 1,000-arrangement controls stay below the retained-evidence
crossover and expose the linked-layout/direct-dispatch cost rather than hiding
it:

| Fixture | Instructions | Branches | Cycles | Median time |
| --- | ---: | ---: | ---: | ---: |
| crossing octahedra | +0.331% | +0.291% | +2.213% | +2.091% |
| affine boxes | +0.301% | +0.258% | -0.363% | -0.578% |

These small deterministic regressions remain open performance controls. They
are not addressed with a fixture shortcut.

## Large-fixture heap, RSS, and policy gate

`dense_crossing_grid_65` contains 1,572 input triangles and 16,900 exact grid
crossings. The allocator probe measures the complete Boolean kernel and its
requested output. `STRICT` and `APPROXIMATE_512` produce exactly the same
values in every row:

| Metric | Direct parent | Current | Change |
| --- | ---: | ---: | ---: |
| total process peak | 178,914,697 B | 178,914,697 B | unchanged |
| incremental kernel peak | 178,125,996 B | 178,125,996 B | unchanged |
| post-Boolean incremental | 57,917,376 B | 57,917,376 B | unchanged |
| output live payload | 57,784,192 B | 57,784,192 B | unchanged |
| retained input-fact growth | 133,184 B | 133,184 B | unchanged |
| allocation calls | 32,817,512 | 32,818,562 | +0.003% |
| deallocation calls | 32,159,654 | 32,160,704 | +0.003% |
| reallocations | 2,132,870 | 2,132,870 | unchanged |
| cumulative bytes added | 9,761,722,369 | 9,841,052,545 | +0.813% |
| cumulative bytes removed | 9,703,804,993 | 9,783,135,169 | +0.818% |

The useful peak is unchanged because exact output ownership still determines
the high-water mark. The extra 79,330,176 cumulative bytes are short-lived
per-face line schedules; they do not survive the face or alter retained facts.
The output remains `Certified` with 73,844 vertices and 164,068 triangles.

The uninstrumented strict process completes in 46.37 seconds wall / 46.08
seconds user at 191,796 KiB maximum RSS and zero swaps. Relative to the direct
parent this is -33.453% wall, -27.966% user, and +24 KiB (+0.013%) RSS. The
complete two-policy value/topology/boundary/volume equality gate passes in
113.18 seconds, down 12.892% from 129.93 seconds.

## Profile

The final CPU-11, 999 Hz frame-pointer profile captured 1,100 samples with none
lost in `target/phase17-retained-line-orientation.perf.data`.

| Self time | Symbol |
| ---: | --- |
| 21.34% | `Real::exact_rational_homogeneous_point2_i128` |
| 10.34% | direct `orient2d_with_policy` |
| 6.26% | retained `orient2d_with_orientation_and_policy` |
| 5.81% | `Rational::denominator` |
| 5.64% | `surface_arrangement::projected_segment` |
| 2.86% | `Real::exact_dyadic_f64` |
| 2.77% | `Rational::numerator` |
| 2.50% | `TriangleTopology::replace_region` |
| 2.03% | `approximate_constraint_bounds_overlap` |
| 1.85% | `triangle_neighbors` |

The parent's direct orientation was 19.84% self. The current direct and
retained paths total 16.60%, while homogeneous point conversion falls from
25.41% to 21.34%. The next clean target is retaining query-point conversion
evidence across multiple authored lines, not weakening the determinant or
copying CGAL's scalar schedule.

## Source and linked size

The implementation changes four source files across Hyperlimit and Hypermesh by
201 insertions and 24 deletions, including API documentation and focused tests.
The direct competitive executable grows 24,452 text bytes (+0.886%).

The canonical size harness was rebuilt from final source with Rust 1.97.0.
Values are linked native text bytes and `wasm-opt -Oz` bytes.

| Features/profile | Current native general / immediate | Current WASM general / immediate | Parent-relative range |
| --- | ---: | ---: | ---: |
| default release | 2,004,634 / 2,007,786 | 1,421,118 / 1,422,956 | +0.927% to +0.953% |
| default size | 1,097,895 / 1,098,859 | 684,610 / 685,029 | +0.621% to +0.846% |
| all-feature release | 2,140,227 / 2,143,091 | 1,495,837 / 1,497,809 | +0.901% to +0.950% |
| all-feature size | 1,100,231 / 1,101,203 | 684,831 / 684,873 | +0.621% to +0.874% |

The largest native text increase is 19,144 bytes, and the maximum canonical
percentage growth is 0.953%. Performance has priority, but this is material and
linked-size recovery stays open.

## Current CGAL EPECK boundary

The pinned CGAL 6.0.3 EPECK executable and exact rational OFF inputs are
unchanged. Five paired CPU-11 trials use 900 retained iterations per engine,
CGAL copies outside the timer, and one retained Hypermesh `MeshContext`.
Every output remains valid, closed, structurally valid, and agrees with the
exact triangle-count and volume oracle.

| Fixture | CGAL median | Hypermesh median | Median paired ratio |
| --- | ---: | ---: | ---: |
| crossing octahedra | 114,422 ns | 294,877 ns | 2.587x |
| affine boxes | 369,954 ns | 680,832 ns | 1.846x |

Both gaps remain open. The preceding 63-process cold boundary is retained as
the current cold reference: 4.617--4.691x CGAL for crossing and
2.531--2.623x for affine. This checkpoint does not relabel either a win. The
historical 3,312.66-second EMBER / 0.09-second CGAL full-case boundary also
remains an explicit open historical target; this face-local checkpoint makes
no unsupported full-case claim.

## Call graph and validation

The final five-crate call graphs contain 15,323 nodes / 25,560 edges in
production, 17,669 / 28,930 with tests and examples, and 21,714 / 35,030 across
all evidence sources. The production graph directly records:

```text
surface_arrangement::corefine_face
  -> hyperlimit::line2_orientation
  -> surface_arrangement::segments_properly_cross
  -> surface_arrangement::planar_orientation_with_line
  -> hyperlimit::classify_point_line_with_orientation

surface_arrangement::corefine_face
  -> surface_arrangement::planar_point_on_segment
  -> hyperlimit::point_on_segment_with_orientation
  -> hyperlimit::classify_point_segment_with_orientation
  -> hyperlimit::orient2d_with_orientation_and_policy
```

There are zero exact EMBER, `segment_trace`, or `local_bsp` identifiers in
production/test/bench/example/fuzz sources or graph nodes, and there remains
one `surface_arrangement` Boolean engine. Graph artifacts are:

- `target/phase17-retained-line-orientation-callgraph-production/callgraph.json`
- `target/phase17-retained-line-orientation-callgraph-tests/callgraph.json`
- `target/phase17-retained-line-orientation-callgraph-all/callgraph.json`

Final validation passes Hyperlimit's complete all-feature suite, Hypermesh's
complete release all-feature suite, warning-denied all-target/all-feature
Clippy for both crates, formatting, diff checks, the large two-policy equality
gate, both-policy allocator probes, the uninstrumented process probe, canonical
size matrices, the final profile, current paired CGAL controls, and all three
call graphs.

Phases 11, 17, and 18 remain open. Remaining work includes retained query-point
evidence, broad direct-path recovery, external real-world and fuzz-source
corpus completion, stage-specific lifetime attribution, every losing CGAL row,
linked-size recovery, and the final path-completeness/removal/release audit.
