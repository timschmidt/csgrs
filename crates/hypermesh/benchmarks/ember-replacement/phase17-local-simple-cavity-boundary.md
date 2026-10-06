# Phase 17 local simple-cavity boundary scheduling

Date: 2026-08-05

Status: retained. Phases 11, 17, and 18 remain open.

## Outcome

Hypertri `72ddcdc` makes the exact cavity boundary itself decide whether
protected-hole closure is necessary. Constraint recovery already needs the
sorted local boundary to split and retriangulate a crossed corridor. It now
builds that boundary first and walks the exact vertex adjacency. A single
simple cycle containing both constraint endpoints proceeds directly. Only a
weak, disconnected, or multiply incident boundary invokes the complete search
for enclosed unprotected triangle components; after closure, the same local
cycle proof is required again before any topology mutation.

This is a general topological fact, not a workload heuristic. There is no
fixture, coordinate, line count, operation, output, policy-name, competitor, or
benchmark dispatch. The hull boundary and existing constraints remain barriers
to closure, all malformed-boundary errors remain typed, and the atomic checked
`TriangleTopology` replacement from the parent checkpoint is unchanged.

## Completeness and policy behavior

The focused grid-ring test proves all three structural paths:

1. an unselected inner component initially produces more than one boundary
   cycle;
2. the complete closure search absorbs it when no hull/protected barrier is
   reachable, after which the local cycle proof succeeds; and
3. a protected diagonal keeps the inner component separate and the cycle proof
   continues to reject it.

The complete cavity/collinearity corpus, canonical retained-versus-scan
differential, and all Hypertri policy/property tests pass. `STRICT` and
`APPROXIMATE_512` run identical topology code and continue to obtain every
geometric decision from Hyperlimit. `STRICT` remains exact-only, and only
Hyperlimit's approximate 512-bit terminal can consume approximation. Both
large rows remain `Certified`, so this fixture consumes no terminal
approximation.

## Deterministic performance

The direct parent is retained mutable topology at Hypertri `69a18e9` and
Hypermesh evidence `4f80b1c5`. Each dense row is the median of three fresh
CPU-11-pinned processes and includes fixture construction plus one exact
intersection.

| Dense crossing 17 | Parent | Local simple-boundary schedule | Change |
| --- | ---: | ---: | ---: |
| instructions | 24,083,182,461 | 22,227,815,623 | -7.704% |
| branches | 3,125,077,444 | 2,906,838,361 | -6.983% |
| cycles | 6,495,638,376 | 5,871,677,803 | -9.606% |
| median Boolean time | 1.543316730 s | 1.387561232 s | -10.092% |

Both versions produce the same certified 5,444-vertex / 11,908-triangle
output. Relative to the correctness-complete iterative scanner, the cumulative
reductions are 62.82% instructions, 62.66% branches, 63.71% cycles, and 63.73%
Boolean time.

Broad controls do not pay for the scheduling proof. The three-run
1,000-arrangement crossing-octahedra medians improve 0.120% instructions and
0.111% branches; affine boxes improve 0.0146% and 0.0248%. Their exact outputs
remain 24 shared vertices with 44/20/32/32 and 44/28/20/20 triangles.

## Large heap, traffic, and RSS

The requested-payload allocator surrounds only the Boolean kernel of the
1,572-input-triangle / 16,900-crossing `dense_crossing_grid_65` fixture.
Committed `STRICT` and `APPROXIMATE_512` runs are identical in every value:

| Metric | Parent | Local simple-boundary schedule | Change |
| --- | ---: | ---: | ---: |
| incremental kernel peak | 178,125,996 B | 178,125,996 B | 0 |
| post-Boolean incremental | 57,917,376 B | 57,917,376 B | 0 |
| output-live payload | 57,784,192 B | 57,784,192 B | 0 |
| retained fact growth | 133,184 B | 133,184 B | 0 |
| allocation calls | 48,160,094 | 47,668,590 | -1.021% |
| reallocations | 3,875,143 | 2,531,566 | -34.672% |
| cumulative added bytes | 97,355,251,074 B | 95,666,427,841 B | -1.735% |

The useful heap boundary and retained Hyperreal facts are byte-identical, while
skipped component work removes 491,504 allocations, 1,343,577 reallocations,
and 1,688,823,233 cumulative requested bytes.

The fresh strict process completes in 1:20.35 with 191,856 KiB maximum RSS and
zero swaps, 22.38% faster than the 1:43.52 parent. RSS moves only +148 KiB
(+0.077%) while the direct allocator peak is identical, so no RSS reduction is
claimed. Relative to the iterative scanner, wall time is down 88.38%.

The opt-in large gate compares every output `Real`, complete topology,
balanced boundaries, and exact rational six-volume under both policies. It
passes in 163.90 seconds, 21.99% faster than the 210.11-second parent. Both
policies produce 73,844 vertices, 164,068 triangles, and exact volume 12,805.

## Profile

A 999 Hz DWARF profile captures 1,422 samples with none lost. The parent's
8.86%-self `close_constraint_cavity_holes` node is absent from the sampled
dense route. Current self leaders are exact homogeneous 2D point construction
(22.89%), policy-owned orientation (21.38%), initial sparse triangle adjacency
(7.98%), exact rational denominator access (4.78%), exact dyadic export (4.25%),
and projected-segment preparation (3.97%). Mutable topology replacement and
neighbor lookup fall to 0.78% and 0.71%.

The result supports the architectural rule: schedule from retained structural
proofs, preserve the complete exact decline, and leave scalar semantics in
Hyperreal/Hyperlimit. The next topology opportunity is avoiding reconstruction
of adjacency that the exact initial triangulation already establishes, if it
can be transferred without duplicating ownership or increasing broad-path
cost.

## Code and binary size

The direct default-feature competitive and allocator probes each shrink 376
bytes of linked text. Every release-profile native and optimized-WASM canonical
consumer shrinks. Across all sixteen canonical consumers, movements range from
-0.081% to +0.037%; the maximum growth is 400 bytes in all-feature size-native
immediate code.

| Features/profile | Native general / immediate | Optimized WASM general / immediate | Parent-relative range |
| --- | ---: | ---: | ---: |
| default release | 1,981,398 / 1,984,542 | 1,403,334 / 1,405,165 | -0.076% to -0.031% |
| default size | 1,086,807 / 1,087,763 | 675,721 / 675,947 | -0.012% to +0.035% |
| all-feature release | 2,116,291 / 2,119,147 | 1,477,366 / 1,479,327 | -0.081% to -0.029% |
| all-feature size | 1,089,159 / 1,090,131 | 675,570 / 676,034 | -0.011% to +0.037% |

## Call graph and validation

The five-crate graphs contain 15,255 nodes / 25,450 edges in production,
17,602 / 28,820 with tests and examples, and 21,647 / 34,920 across all
evidence sources. The exact path is:

```text
recover_constraint_cavity
  -> collect_constraint_cavity_boundary
  -> constraint_cavity_cycle
  -> close_constraint_cavity_holes  # only after local proof declines
  -> collect_constraint_cavity_boundary
  -> constraint_cavity_cycle
  -> TriangleTopology::replace_region
```

There are zero retired global scanner nodes, zero actual EMBER namespace nodes,
and one Boolean arrangement engine.

- Hypertri's complete all-feature unit, adversarial, differential,
  fuzz-property, policy, README, and doctest suites pass.
- Hypermesh's complete all-feature release suite passes; seven opt-in expensive
  cases remain ignored by the normal matrix.
- The opt-in large full-value policy gate passes separately.
- Warning-denied all-target/all-feature Clippy, formatting, diff checks,
  allocator rows, profile, native/WASM size matrix, and call graphs pass.

Phases 11, 17, and 18 remain open for external real-world/fuzz-source corpus
completion, retained initial adjacency evaluation, every losing CGAL row,
large RSS targets, and the final requirement audit.
