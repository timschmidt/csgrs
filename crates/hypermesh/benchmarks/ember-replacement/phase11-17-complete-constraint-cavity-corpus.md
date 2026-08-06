# Phase 11/17 complete exact constraint-cavity corpus

Date: 2026-08-04

Status: retained. Phases 11, 17, and 18 remain open.

## Outcome

Hypermesh's permanent corpus grows from 49 to 57 records and from eighteen to
nineteen isolated large-heap selectors. Five bounded-face microcases now pin
disconnected inner cells, nested loops, crossing constraints, opposite-winding
coincidence, and collinear overlap/T-junction behavior. A deterministic
`dense_crossing_grid_{5,17,65}` family scales one ordinary integer arrangement
from 100 through 16,900 proper line crossings without changing local shell
topology or scalar width.

The family exposed two general Hypertri defects in exact constraint recovery:

1. the dual crossed-triangle corridor can surround an unselected interior
   component, making its combinatorial boundary unsuitable for a one-disk ear
   schedule; and
2. removing a newly collinear vertex after an ear was already emitted can
   retain that vertex in the emitted triangle while silently omitting its newly
   exposed sliver.

Hypertri `d42d89a` closes only unprotected enclosed components, leaves the
convex-hull exterior and prior constraints as barriers, prunes straight chains
before emitting an ear, and reinserts every omitted indexed point through the
general exact topology insertion routine. Hypertri `ab37e54` then replaces the
repeated global flip loop with one complete crossed corridor per missing PSLG
segment. A one-edge corridor reuses its already-certified proper crossing as
the convex-quadrilateral proof; a multi-edge corridor initializes one cavity
bitset on first use and follows the same complete retriangulation path.
Hypertri `aa99d16` retains that bitset once per recovery batch and clears it
only when a multi-edge corridor needs it, avoiding repeated heap traffic while
keeping the one-edge route allocation-free.

There is no fixture, coordinate, operation, result, policy-name, or competitor
dispatch in production. There is no compatibility shim or alternate Boolean
engine.

## Exactness and policy ownership

The small and medium family members run the public Boolean engine under both
`STRICT` and `APPROXIMATE_512`, require `Certified`, compare complete output
values, prove balanced boundaries, and compare exact rational six-volumes.
The opt-in 65-line release gate performs the same full policy/output/property
checks for the 1,572-input-triangle, 16,900-crossing member. Its expected
intersection volume is exactly 12,805.

The isolated large allocator rows agree under both policies in every reported
metric and topology summary: 73,844 output vertices, 164,068 output triangles,
178,914,697 total requested-payload bytes at peak, and `Certified` certainty.
No terminal approximation is used. `STRICT` remains exact-only, and
`APPROXIMATE_512` can still terminate only through Hyperlimit's existing
512-bit policy boundary.

## Dense-corridor performance

The paired parent is a preserved release executable containing the completed
hole/collinearity correctness fixes but retaining the historical per-flip
global topology rebuild. This isolates scheduling from correctness. Each row
is three CPU-11-pinned fresh processes; counts include fixture construction and
one strict intersection.

| Dense 17 row | Iterative parent | Complete cavity | Change |
| --- | ---: | ---: | ---: |
| instructions | 59,778,316,406 | 30,797,543,593 | -48.48% |
| branches | 7,783,893,606 | 4,080,896,229 | -47.57% |
| cycles | 16,178,219,472 | 8,626,788,491 | -46.68% |
| median Boolean time | 3.826 s | 2.041 s | -46.66% |

Both executables produce the same certified 5,444-vertex / 11,908-triangle
summary. The complete-cavity algorithm is a simpler unit of work: it sorts
incidence once per segment instead of once per accepted flip, and performs one
topology replacement.

Ordinary controls remain essentially flat. In a three-run 1,000-arrangement
crossing-octahedra row, current instructions/branches move about
+0.09%/+0.09%, while cycles fall 0.37%; the exact output remains 24 shared vertices and
44/20/32/32 triangles for the four requested results. Affine boxes remain
neutral in instructions and branches within the paired run. These bounded
movements retain the general path-completeness fix and the roughly halved
dense work.

## Large heap, allocation traffic, and RSS

The 65-line heap row uses the requested-payload tracking allocator around only
the Boolean kernel. Both current policies produce identical values below.

| Metric | Iterative parent | Complete cavity | Change |
| --- | ---: | ---: | ---: |
| incremental kernel peak | 178,757,268 B | 178,125,996 B | -0.35% |
| post-Boolean incremental | 58,463,480 B | 57,917,376 B | -0.93% |
| output-live payload | 58,415,720 B | 57,784,192 B | -1.08% |
| allocation calls | 35,040,748 | 47,470,284 | +35.47% |
| reallocations | 1,656,922 | 3,090,101 | +86.50% |
| cumulative added bytes | 362,954,861,088 B | 151,292,361,689 B | -58.32% |

The cavity schedule intentionally trades more small exact temporaries for far
less aggregate allocation traffic and much lower CPU work. Reusing the cavity
bitset removes another 165,403 allocation calls and 258,027,572 cumulative
requested bytes from the intermediate complete-cavity build. The live maximum
is output-dominated and still falls by 631,272 bytes. Retaining mutable
adjacency across successive constraints is the next clean optimization target
because it can address call count and the remaining sort without changing
exact predicates.

The final uninstrumented strict process completes in 2:53.30 with 190,884 KiB
maximum RSS and zero swaps. The iterative parent requires 11:31.26 and
190,964 KiB on the same fixture, so complete-cavity scheduling removes 74.93%
of whole-process wall time while also lowering peak RSS by 80 KiB. Input
construction and retained facts are included.

## Profile and next algorithmic target

A 999 Hz DWARF profile of dense 17 captured 2,516 samples with none lost.
`EdgeUse` quicksort and small-sort account for 10.39% and 10.34% self time.
Exact homogeneous 2D point construction and policy-owned orientation account
for 15.92% and 13.45%; constraint geometry validation is only 0.84%.

This evidence rejects validation weakening or scalar approximation as the next
step. The next implementation should retain checked triangle adjacency across
constraint insertions and walk/update only the crossed corridor. That plays to
Hyperreal's retained facts and preserves the exact decline path while removing
the remaining global incidence sort.

## Historical and competitive boundary

The dense crossing family deliberately uses a generalized-winding self-PWN
operand and is outside the pinned shared CGAL contract, so it is not counted as
a CGAL win or loss. The ordinary exact shared-contract controls remain open at
approximately 3.53x CGAL EPECK for crossing octahedra and 2.11x for affine
boxes. The 512-bit thin-dyadic boundary remains approximately 6.48x/6.11x/
6.59x CGAL instructions/branches/cycles. This checkpoint does not claim parity.

Historically, the medium member exceeds every retired fixed repair/pass count;
the new engine exhausts it without a pass limit. No deleted EMBER engine is
restored for an artificial timing row. The preserved correctness-complete
iterative executable is the honest direct scheduling parent.

## Code and binary size

The directly affected competitive and allocator probes each shrink 1,852 bytes
of ELF `.text` because the repeated flip scanner is deleted. The new general
cavity-completion cases add source and bounded canonical linked code. Across
the sixteen default/all-feature release/size native and optimized-WASM
artifacts, the maximum growth is 0.2104%; performance has priority and the
roughly 46--48% dense CPU reduction justifies that bounded cost.

Current general/immediate measurements are:

| Feature/profile | Native text | Optimized WASM |
| --- | ---: | ---: |
| default release | 1,960,058 / 1,963,210 | 1,383,742 / 1,385,579 |
| default size | 1,070,871 / 1,071,835 | 663,053 / 663,466 |
| all-feature release | 2,095,471 / 2,098,335 | 1,457,902 / 1,459,873 |
| all-feature size | 1,073,215 / 1,074,179 | 663,072 / 663,528 |

## Call graph

Final five-crate graphs contain 15,182 nodes / 25,305 edges in production,
17,529 / 28,675 with tests and examples, and 21,574 / 34,775 across all
evidence sources. The relevant static spine is:

```text
hypermesh::surface_arrangement::corefine_face
  -> hypertri::cdt::constrained_triangulation_convex_hull
  -> hypertri::cdt_insert::recover_missing_constraint
  -> hypertri::cdt_insert::recover_constraint_cavity
     -> mark_cavity_triangles
        | close_constraint_cavity_holes
        | replace_adjacent_edge
```

The removed `first_flippable_edge_crossing_constraint` node is absent. There
are zero actual EMBER namespace nodes and one Boolean arrangement engine.

## Validation

- Hypertri all-feature unit, integration, fuzz-property, policy, README, and
  doctest suites pass.
- Hypermesh's release suite, 38 focused surface-arrangement tests, 16 manifest
  tests, small/medium public both-policy gate, and opt-in large both-policy gate
  pass.
- Warning-denied all-target/all-feature Clippy passes in both crates; Hypermesh
  documentation, formatting, and diff checks pass.
- Default and all-feature native/WASM release/size matrices pass.
- The permanent manifest and migration ledger remain monotonic.

Phases 11, 17, and 18 remain open for external real-world fixtures, complete
fuzz-seed admission, retained mutable CDT topology, every shared CGAL row,
large RSS targets, and the final requirement audit.
