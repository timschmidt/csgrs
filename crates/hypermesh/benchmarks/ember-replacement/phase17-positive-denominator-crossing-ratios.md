# Phase 17: policy-oriented positive crossing ratios

Captured 2026-08-04. The retained implementation is Hypermesh `f3683ba9`,
based on the exact support-line interval checkpoint `8689f3c9` (implementation
`cfc5ae62`).

## Result

Every proper edge/plane crossing in the nonparallel support-line slice is now
stored from the endpoint classified positive by the operation's
`DecisionContext` to the endpoint classified negative. Its retained coordinate
and interpolation ratios therefore share one policy-positive denominator.
Downstream ratio ordering, affine/ratio ordering, certified enclosure
construction, and final materialization no longer carry or branch on a
duplicate denominator-sign flag.

This is an algebraic invariant of the complete convex-slice algorithm. It is
not a fixture shortcut: there is no branch on mesh identity, triangle count,
coordinate width, Boolean operation, policy name, output expectation, or
competitor. The general exact `Real` comparison and Hyperlimit terminal remain
the only declined paths.

The change removes 24 net production lines, one call-graph node, and one edge.
Paired deterministic work improves on the 23,788-triangle full row, every
independent large control, and both ordinary-box policy rows except for a
0.031% approximate branch movement. Full and 2,049-bit large-fixture heap
counters are byte-identical. Linked-size movement is mixed and bounded to 94
bytes; performance has priority, and the smaller source/invariant is retained.

Phase 17 and Phase 18 remain open. This checkpoint does not change the
previously measured 19.00x full-row runtime and 12.25x fresh-process RSS loss
to pinned CGAL 6.0.3 EPECK, nor the 4.81x/4.53x ordinary-box losses.

## Exactness and policy argument

For a proper crossing, the already-computed endpoint classifications are
opposite. Naming their support values `p` and `n` and their selected line
coordinates `xp` and `xn`, the crossing coordinate is retained exactly as

`(p * xn - n * xp) / (p - n)`.

The interpolation parameter is retained as `p / (p - n)` from the positive
endpoint toward the negative endpoint. Reversing an input edge negates both
the former numerator and denominator, so the represented exact point and its
canonical construction identity are unchanged.

Under `STRICT`, the positive/negative classifications are certified or the
operation returns `PredicateUndecided`; no terminal approximation is consumed.
Under `APPROXIMATE_512`, only Hyperlimit may terminate an unresolved sign at
512 bits. The crossing orientation reuses that already-selected topology fact
and does not introduce another decision. If terminal interpretation was
needed, the existing aggregate `Approximate512Consumed` marker remains on the
result. Exact rational paths remain `Certified` under both policies.

The previous carrier stored the same policy-selected sign as a Boolean and
reversed every later comparison conditionally. Orienting the retained algebra
once is behaviorally equivalent while avoiding repeated control flow. Exact
materialization still uses `Real` numerator and denominator values; certified
binary64 enclosures may prove ordering only and never replace the exact point.

## Paired retired-work measurements

Candidate and parent were built in equal clean worktrees and measured with
`perf stat` on CPU 11. Instructions and branches are the acceptance signal;
wall time during this window moved with host frequency/load and is not claimed
as evidence.

| Workload | Parent instructions | Current instructions | Movement | Branch movement |
| --- | ---: | ---: | ---: | ---: |
| Full rotated YeahRight, 23,788 triangles | 16,530,262,352 | 16,526,259,516 | -0.0242% | -0.0047% |
| 2,049-bit wide rational, 6,144 triangles | 3,824,038,509 | 3,823,820,374 | -0.0057% | -0.0039% |
| Clipped voxel torus 33, 6,412 triangles | 1,134,371,933 | 1,134,237,758 | -0.0118% | -0.0098% |
| Clipped voxel torus 65, 25,100 triangles | 4,915,377,049 | 4,914,361,414 | -0.0207% | -0.0247% |
| Dense coplanar 16, 6,144 triangles | 3,016,824,494 | 3,016,050,293 | -0.0257% | -0.0271% |

The ordinary exact-box probe runs 1,000 complete shared arrangements per
process:

| Policy | Parent instructions | Current instructions | Movement | Branch movement |
| --- | ---: | ---: | ---: | ---: |
| `STRICT` | 5,681,276,501 | 5,678,137,657 | -0.0552% | -0.0372% |
| `APPROXIMATE_512` | 5,680,869,170 | 5,680,660,061 | -0.0037% | +0.0307% |

The tiny approximate branch movement is 307,907 branches over 1,000 complete
arrangements and does not outweigh lower instructions, consistent large-row
improvements, simpler source, and removal of the sign field.

## Large-fixture heap

The direct global-allocator probe excludes fixture construction and measures
the Boolean boundary. Both policies remain exactly equal for output,
certainty, peak, allocation/deallocation/reallocation counts, and byte totals.

| Selector | Policy | Incremental peak | Allocations | Reallocations | Added bytes |
| --- | --- | ---: | ---: | ---: | ---: |
| `yeahright-full-rotated` | both | 158,258,204 B | 16,928,390 | 2,385,161 | 913,357,128 B |
| `wide-rational-2048` | both | 31,234,658 B | 2,092,630 | 227,677 | 459,671,086 B |

Every counter is byte-for-byte identical to `8689f3c9`. The complete
13-selector, both-policy matrix from the parent checkpoint remains applicable:
this change removes a stack Boolean and branches only, allocates nothing, and
the full and widest direct sentinels confirm no lifetime movement.

## Source and linked size

Production source changes from 63 removed to 39 added lines, for a net removal
of 24 lines. Native values are `.text`; WASM values are `wasm-opt -Oz` bytes.

| Profile / consumer | Parent native | Current native | Parent WASM | Current WASM |
| --- | ---: | ---: | ---: | ---: |
| release / general | 1,968,566 | 1,968,582 (+16) | 1,391,337 | 1,391,425 (+88) |
| release / immediate | 1,971,710 | 1,971,726 (+16) | 1,393,195 | 1,393,283 (+88) |
| size / general | 1,072,063 | 1,071,807 (-256) | 668,360 | 668,454 (+94) |
| size / immediate | 1,073,019 | 1,072,771 (-248) | 668,771 | 668,865 (+94) |

The largest increase is 94 bytes (0.0141%). No dependency, compatibility shim,
duplicate engine, or new public API is added.

## Corpus and validation

The permanent exact support-line corpus exercises the normalized orientation
under both policies, both operand orders, and all four winding combinations:
80 static and 4,096 generated exact-oracle executions. Unit coverage includes
every empty/point/interval overlap order, affine/ratio mixtures, reversed
synthetic ratios, 1,024 random exact-rational comparisons, canonical equal
endpoints, retained materialization, all three support axes, symbolic terminal
policy behavior, and the carrier size bound.

- The all-feature suite passes 179 executed tests and the default suite passes
  178; six documented opt-in external/manual stress tests are ignored in each.
- No-default-feature checking, all-target/all-feature warning-denied Clippy,
  warning-denied rustdoc, every fuzz binary, formatting, and diff checks pass.
- Every benchmark target builds, and the locked release Trunk demo builds.
- Full and wide large-fixture heap probes pass under both policies with exact
  policy equality.

## Call-graph audit

The focused Hyperreal/Hyperlattice/Hyperlimit/Hypertri/Hypermesh graph contains
14,792 nodes and 24,633 edges. Hypermesh contributes 2,913 nodes and 4,668
edges, one node and one edge fewer than the parent. The same sole production
route enters closed support-line slicing; no deleted candidate-containment,
projective retry, four-plane determinant, EMBER, compatibility, or second
Boolean-engine route reappears. Hypercurve and HyperSolve are excluded.

## Rejected alternatives

Two follow-up attempts were measured and fully removed:

- Dropping retained certified endpoint enclosures increased the full row to
  16,843,535,511 instructions and 3,001,120,026 branches, about 1.90% more
  instructions than the retained parent.
- Replacing the exact `Rational::to_f64_enclosure` result with a coarse cached
  relative enclosure increased the full row to 16,545,002,359 instructions
  and 2,953,286,360 branches (+0.092%/+0.155%).

The retained design therefore keeps Hyperreal's certified facts close to the
exact ratio, orients the algebra once, and leaves the general fallback intact.

## Open work

The CGAL runtime/RSS gates, deeper symbolic and real-world corpus families,
stage-specific arena attribution, current confidence runs, and final Phase 18
requirements audit remain open. This checkpoint claims only the measured
crossing-ratio simplification and does not hide losing competitive rows.
