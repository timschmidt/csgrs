# Phase 17 retained mutable exact CDT topology

Date: 2026-08-04

Status: retained. Phases 11, 17, and 18 remain open.

## Outcome

Hypertri `69a18e9` replaces the remaining repeated whole-triangulation
incidence rebuild in exact constraint recovery with one checked mutable
topology for the recovery batch. Reciprocal triangle neighbors and one incident
triangle per vertex are constructed only when the first missing PSLG segment
requires recovery. Each subsequent segment walks its endpoint fan and exact
crossed dual corridor locally, retriangulates the complete cavity, and patches
the fixed triangle slots only after proving that the replacement has exactly
the old region boundary.

Final unconstrained-edge restoration uses the same retained adjacency and a
deterministic smallest-edge worklist. Stale work entries are validated against
the current triangles, and a flip enqueues only edges of the two changed
triangles. The old complete sorted scan remains the compact route when every
constraint is already represented, so a topology sidecar is not allocated for
the common no-recovery case. A differential test proves that retained and scan
schedules converge to the same canonical topology from two different convex
fans under both policies.

This is one general algorithm. There is no fixture, coordinate, result,
operation, policy-name, competitor, or workload-cardinality dispatch; no
compatibility shim; and no second Boolean engine.

## Checked invariants and path completeness

`TriangleTopology::replace_region` rejects unsorted or repeated slots, invalid
vertices, degenerate triangles, changed or duplicated boundaries,
non-reciprocal exterior ownership, and non-manifold replacement incidence
before mutating any triangle or adjacency row. Accepted patches update local
reciprocal neighbors, exterior owners, and vertex representatives together.
Tests pin reciprocal updates after a flip and prove that boundary rejection is
atomic.

The recovery walk keeps the complete typed paths for an absent endpoint,
multiple outgoing crossings, boundary escape, constrained-edge crossing,
cycles, non-disk cavities, omitted endpoints, changed triangle cardinality,
and omitted target edges. Exact cavity construction, collinear-vertex
reinsertion, and protected hole barriers remain unchanged from the preceding
correctness checkpoint.

`STRICT` and `APPROXIMATE_512` use identical topology and schedules. Every
geometric decision still goes through the policy-owned Hyperlimit predicates;
`STRICT` is exact-only, and `APPROXIMATE_512` can terminate only at
Hyperlimit's existing approximate 512-bit boundary. The large fixture required
no approximate termination: both rows are `Certified`.

## Deterministic CPU work

The immediate parent is the complete-cavity checkpoint at Hypertri `aa99d16`.
The historical scheduling parent is the correctness-complete iterative global
scanner retained in the preceding evidence. Rows are medians of three fresh
CPU-11-pinned processes and include fixture construction plus the requested
Boolean operation.

| Dense crossing 17 | Iterative scanner | Complete cavity | Retained topology |
| --- | ---: | ---: | ---: |
| instructions | 59,778,316,406 | 30,797,543,593 | 24,083,182,461 |
| branches | 7,783,893,606 | 4,080,896,229 | 3,125,077,444 |
| cycles | 16,178,219,472 | 8,626,788,491 | 6,495,638,376 |
| median Boolean time | 3.826149526 s | 2.040986333 s | 1.543316730 s |

Against the immediate parent, instructions fall 21.80%, branches 23.42%,
cycles 24.70%, and Boolean time 24.38%. Against the iterative scanner they fall
59.71%, 59.85%, 59.85%, and 59.66%. Every executable produces the same
certified 5,444-vertex / 11,908-triangle output.

The three-run 1,000-arrangement controls remain bounded. Crossing octahedra
instructions/branches move +0.995%/+0.877%; affine boxes move
+0.114%/+0.123%. Their complete exact summaries remain respectively 24 shared
vertices with 44/20/32/32 triangles and 24 shared vertices with 44/28/20/20
triangles. Crossing cycle movement was noisy at +7.86%, while affine cycles
fell 1.24%; deterministic counts, not this short clock window, are the gate.

## Large-fixture heap, traffic, and RSS

The requested-payload allocator surrounds only the Boolean kernel for the
1,572-input-triangle / 16,900-crossing `dense_crossing_grid_65` fixture.
Committed `STRICT` and `APPROXIMATE_512` runs agree exactly in every counter,
not merely output cardinality.

| Metric | Iterative scanner | Complete cavity | Retained topology |
| --- | ---: | ---: | ---: |
| incremental kernel peak | 178,757,268 B | 178,125,996 B | 178,125,996 B |
| post-Boolean incremental | 58,463,480 B | 57,917,376 B | 57,917,376 B |
| output-live payload | 58,415,720 B | 57,784,192 B | 57,784,192 B |
| retained fact growth | 47,760 B | 133,184 B | 133,184 B |
| allocation calls | 35,040,748 | 47,470,284 | 48,160,094 |
| reallocations | 1,656,922 | 3,090,101 | 3,875,143 |
| cumulative added bytes | 362,954,861,088 B | 151,292,361,689 B | 97,355,251,074 B |

Useful peak, post-Boolean state, output payload, and retained-fact growth are
byte-identical to the immediate parent. Local adjacency/worklist ownership adds
1.45% allocation calls and 25.41% reallocations, but eliminates another 35.65%
of cumulative requested-byte traffic. Relative to the iterative scanner,
cumulative traffic is down 73.18% and useful peak remains 631,272 bytes lower.
The remaining small-allocation count is reported as an open target.

The committed uninstrumented strict process completes in 1:43.52 with 191,708
KiB maximum RSS and zero swaps. Time improves 40.27% from the immediate
2:53.30 row and 85.02% from the iterative 11:31.26 row. RSS is 824 KiB (+0.43%)
above the immediate run while the direct allocator maximum is identical, so no
heap reduction is claimed for this checkpoint.

The opt-in large both-policy gate compares every output `Real`, complete
topology, balanced boundaries, and exact rational six-volume. It passes in
210.11 seconds, 41.75% faster than its preceding 360.71-second run; both
policies produce 73,844 vertices, 164,068 triangles, and exact volume 12,805.

## Profile and next target

A 999 Hz DWARF profile of dense crossing 17 captured 1,561 samples with none
lost. The former global `EdgeUse` quicksort/small-sort pair, previously 20.73%
of self samples, is absent. Current self leaders are exact homogeneous 2D point
construction (21.66%), policy-owned orientation (18.79%), protected cavity-hole
closure (8.86%), initial sparse triangle adjacency construction (6.01%), and
projected-segment preparation (4.39%). Mutable neighbor lookup and atomic local
replacement account for 1.88% and 1.48%.

This profile rejects weakened predicates and workload special cases. The next
clean topology target is a local proof that an already-simple corridor boundary
needs no whole-topology protected-hole search, while retaining the complete
closure path for weakly simple cavities. The scalar leaders should be addressed
through retained Hyperreal construction/predicate facts, not CGAL-shaped eager
arithmetic.

## Code and binary size

The direct default-feature competitive and allocator probes grow 27,432 bytes
(1.006%) and 27,420 bytes (0.954%) of linked text. Across the sixteen canonical
general/immediate, default/all-feature, release/size, native/optimized-WASM
artifacts, growth ranges from 1.02% to 1.90%. This explicit footprint cost is
retained for the 21.80% dense instruction reduction; performance has priority,
and the implementation keeps invariant checks factored rather than compressing
them into opaque control flow.

| Features/profile | Native general / immediate | Optimized WASM general / immediate | Maximum movement |
| --- | ---: | ---: | ---: |
| default release | 1,982,014 / 1,985,158 | 1,404,400 / 1,406,229 | +1.49% |
| default size | 1,086,423 / 1,087,387 | 675,613 / 676,026 | +1.89% |
| all-feature release | 2,116,907 / 2,119,763 | 1,478,558 / 1,480,516 | +1.42% |
| all-feature size | 1,088,767 / 1,089,731 | 675,643 / 675,917 | +1.90% |

## Call graph

Final five-crate graphs contain 15,248 nodes / 25,430 edges in production,
17,595 / 28,800 with tests and examples, and 21,640 / 34,900 across all evidence
sources. The new static spine is:

```text
hypermesh::surface_arrangement::corefine_face
  -> hypertri::cdt::constrained_triangulation_convex_hull
  -> hypertri::cdt_insert::recover_constraints
     -> TriangleTopology::new
     -> recover_constraint_cavity
        -> TriangleTopology::neighbor_across
        -> TriangleTopology::replace_region
  -> restore_unconstrained_edges
     -> TriangleTopology::replace_region
```

There are zero retired global constraint-scanner nodes, zero actual EMBER
namespace nodes, and one Boolean arrangement engine.

## Validation

- Hypertri's 82 unit, 25 adversarial, 6 differential, 10 fuzz-property, 5
  policy, 2 README, and 4 doctest cases pass with all features.
- Hypertri and Hypermesh warning-denied all-target/all-feature Clippy pass.
- Hypermesh's complete all-feature release suite passes, including 156 library,
  8 allocation-failure, 13 competitive, 16 corpus-manifest, 3 public API, 9
  regression, and 2 README cases; seven opt-in expensive cases remain ignored
  by the normal matrix.
- The opt-in large full-value policy gate passes separately. Formatting, diff,
  docs, native/WASM size builds, allocator rows, profile, and five-crate graphs
  pass.

Phases 11, 17, and 18 remain open for external real-world corpus admission,
complete fuzz-seed audit, local cavity-hole scheduling, every shared CGAL row,
large RSS targets, footprint recovery where it is performance-neutral, and the
final requirement audit.
