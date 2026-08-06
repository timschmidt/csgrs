# Phases 11/17 path-complete exact line corridors

Date: 2026-08-06

Status: retained; Phases 11, 17, and 18 remain open

Implementation heads: Hyperreal `492882c`, Hyperlimit `b0418bd`, Hyperlattice
`a475bb7`, Hypertri `210a66c`, Hypermesh `90a5c833`

Direct measured parent: the retained projected-query checkpoint at Hypertri
`824bd3a` and Hypermesh `fd482a3e`

## Outcome

This checkpoint closes a valid constrained-triangulation path that the prior
engine rejected. Hypertri now recovers a missing PSLG edge by walking its exact
crossed-triangle corridor in order and constructing the two half-hole chains
directly. A folded same-side chain may contain a protected chord or enclose an
unprotected face. Both are handled by topology proofs inside the same general
algorithm:

- a protected backtracking spike remains in the ordered half-hole and every
  protected chain edge is proved present in the candidate replacement;
- an unprotected component is absorbed only when traversal proves that it
  reaches neither the convex-hull boundary nor a protected edge; and
- only repeated chain detours proved internal to that enlarged cavity are
  pruned before exact retriangulation.

The replacement is checked completely before one atomic topology mutation.
There is no retry through another triangulator, repair pass, fixture case,
coordinate threshold, Boolean-operation branch, expected-result branch,
policy-name branch, competitor branch, or benchmark branch. The finite dual
walk has no pass limit: exceeding the number of triangles proves a revisit or
malformed topology and returns a typed error.

The full-resolution 11,894-by-11,894-triangle rotated YeahRight intersection,
which exposed the missing corridor path, now returns the exact empty boundary
with `Certified` certainty under both policies. The same generator is shared by
the correctness test, allocator and process probes, Hypermesh competitive
probe, and exact CGAL exporter. This is benchmark/corpus infrastructure; it is
not production dispatch.

## Playing to retained `Real` facts

The checkpoint also completes the preceding retained-query work. Hyperreal's
`AffineDet2ExactWordQuery` stores one checked homogeneous exact-word form of a
projected point. Hyperlimit's `Line2OrientationQuery` schedules that carrier
beside its retained fixed-line evidence. Hypermesh builds the query schedule
once per projected point on a changed face when the already established
sixteen-query amortization floor selects retained line evidence, then refreshes
it once after new exact crossing points are admitted.

This is retained computation, not a cached geometric conclusion. If a point is
not an exact rational that fits the checked word carrier, or a determinant
overflows, the schedule is empty or declines and the unchanged complete
Hyperlimit cascade runs. The cascade remains:

1. certified dyadic determinant filter;
2. checked exact-word homogeneous determinant;
3. certified rational determinant filter;
4. arbitrary-precision exact rational evaluation; and
5. the existing policy-owned `Real` refinement path.

`STRICT` remains exact-only. `APPROXIMATE_512` may terminate only in
Hyperlimit's 512-bit terminal, and every such consumption is aggregated into
the Hypermesh result certainty. Neither query scheduling nor corridor topology
branches on policy.

## Exact corridor algorithm and failure coverage

For each absent constraint, the implementation:

1. locates every triangle incident to the first endpoint and proves either an
   existing target edge or exactly one outgoing proper crossing;
2. walks the reciprocal triangle topology across every proper crossing,
   reusing the two exact endpoint-side signs already needed to prove that
   crossing;
3. appends each advancing vertex to the exact left or right half-hole chain;
4. handles the one-crossing convex quadrilateral using that same crossing
   certificate;
5. triangulates both chains with exact ear tests, pruning straight chain
   vertices before ear emission and reinserting any omitted cavity vertex;
6. proves triangle-count conservation, target-edge presence, and retention of
   every protected chain edge; and
7. only if a weak chain prevents that proof, closes topology-proved unprotected
   enclosed components, prunes only newly internal detours, repeats exact
   triangulation and validation, and atomically replaces the region.

The left/right chain vectors are retained across the complete constraint batch.
That removes repeated allocation without changing ownership, ordering, or any
decision.

Every decline is explicit. Unsplit collinear corridor vertices, multiple exits,
crossing an existing constraint, and a revisited corridor are invalid input;
crossing the hull or ending before the target is unsupported geometry; a
nontriangulable exact chain is `NoEarFound`; malformed reciprocal topology,
changed Euler triangle count, or a missing target edge is rejected. No failure
returns an unchanged mesh and no partial region mutation occurs.

Focused both-policy tests pin the two formerly missing shapes:

- a folded corridor containing a non-crossed protected chord; and
- a corridor enclosing an unprotected triangle component.

The existing adversarial, differential, fuzz-property, policy, and final
topology validators cover ordinary, degenerate, malformed, fan-independent,
and canonical-restoration paths.

## Dense exact performance

Five serialized CPU-11 `perf stat` runs measure one four-result arrangement of
`dense_crossing_grid_17`. The direct parent already handled this fixture; this
is therefore a valid performance A/B after the correctness expansion.

| Metric | Retained-query parent | Current | Change |
| --- | ---: | ---: | ---: |
| instructions | 13,300,545,627 | 13,282,680,222 | -0.134% |
| branches | 1,856,541,387 | 1,845,801,877 | -0.579% |
| cycles | 3,706,695,668 | 3,690,398,367 | -0.440% |
| median Boolean time | 878,599,642 ns | 875,376,458 ns | -0.367% |

Relative to the earlier retained-line checkpoint, the combined query and
corridor work is down 14.656% instructions, 13.943% branches, 16.169% cycles,
and 16.097% median time. Every run returns the same `Certified` arena of 9,464
vertices and 11,308 / 11,908 / 10,764 / 12,452 result triangles.

The broad small controls openly regress relative to the incomplete direct
parent:

| Fixture, 1,000 arrangements | Instructions | Branches | Cycles | Median time |
| --- | ---: | ---: | ---: | ---: |
| crossing octahedra | +8.247% | +7.804% | +10.213% | +10.244% |
| affine boxes | +6.075% | +6.317% | +5.826% | +6.318% |

Those costs remain Phase 17 failures. The predecessor rejected the valid full
case, so restoring its shortcut would violate path completeness. Recovery must
come from cleaner retained evidence, topology scheduling, and lower constant
factors shared by all inputs—not fixture dispatch.

## Large dense heap and policy equality

`dense_crossing_grid_65` has 1,572 input triangles and 16,900 exact line
crossings. Both policies have counter-identical allocator rows and return
73,844 vertices / 164,068 triangles with `Certified` certainty.

| Metric | Current |
| --- | ---: |
| total process peak | 178,914,529 B |
| incremental Boolean-kernel peak | 178,125,828 B |
| post-Boolean incremental ownership | 57,827,760 B |
| live output payload | 57,782,288 B |
| retained input-fact growth | 45,472 B |
| allocations / deallocations / reallocations | 32,141,979 / 31,485,216 / 345,511 |
| cumulative bytes added / removed | 6,777,925,892 / 6,720,098,132 |

Reusing the corridor side storage leaves the output-dominated peak unchanged
but removes 475,814 allocations, 475,814 deallocations, 998,676 reallocations,
and 108,282,560 bytes in each cumulative traffic direction versus the
otherwise identical path-complete build.

A final CPU-11 process retires 605,478,868,303 instructions,
85,049,599,718 branches, and 151,874,719,725 cycles in 35.870 seconds. A
separate uninstrumented run records 192,000 KiB maximum RSS and zero swaps;
wall-clock variation is broad, so the deterministic counters are the primary
performance evidence. The post-change complete `Real`/topology/boundary/volume
policy-equality gate passes in 73.47 seconds.

## Full-resolution YeahRight heap and historical boundary

The shared full fixture contains 23,788 input triangles. `STRICT` and
`APPROXIMATE_512` are again counter-identical and return exactly zero vertices
and triangles with `Certified` certainty.

| Metric | Current |
| --- | ---: |
| retained input / payload | 7,123,362 B / 7,122,720 B |
| total process peak | 59,462,878 B |
| incremental Boolean-kernel peak | 52,339,516 B |
| post-Boolean incremental ownership | 2,638,136 B |
| live output payload | 56 B |
| retained input-fact growth | 2,638,080 B |
| allocations / deallocations / reallocations | 10,804,017 / 10,768,551 / 1,775,803 |
| cumulative bytes added / removed | 609,213,930 / 606,575,794 |

Fresh uninstrumented `STRICT` and `APPROXIMATE_512` processes both complete in
1.05 seconds at 69,528 and 69,156 KiB maximum RSS respectively, with zero
swaps. Historical EMBER required 3,312.66 seconds and 329,352 KiB: the current
exact engine is about 3,154.9x faster and uses 78.89% less process RSS. This is
a historical replacement result, not a CGAL-parity claim.

Five current Hypermesh `perf stat` runs have medians of 11,029,601,764
instructions, 1,881,123,239 branches, 4,304,739,936 cycles, and
1,013,468,411 ns.

## Current CGAL 6.0.3 EPECK boundary

The exact exporter writes the same shared full fixture used by Hypermesh. Its
left/right OFF SHA-256 identities are
`5352f571af1df60fd5acc8015aae82f5542c229e683e9f689b03fe452305e310`
and
`c10b3a3d0774126a664e8880c01f38a6a9765c2ab1a6be7dc3a93894a7ebeaa1`.
Every CGAL output is valid, closed, structurally valid, and exactly empty.

Twenty-one outside-copy CGAL intersections have a 30,267,445 ns median
(29,891,601–40,101,471 ns). A fresh process records 31,359,430 ns internally,
0.09 seconds wall / 0.08 seconds user, 17,352 KiB maximum RSS, and zero swaps.

Five paired trials use 21 CGAL intersections and three Hypermesh arrangements
per trial:

| Trial | CGAL median | Hypermesh median | Ratio |
| ---: | ---: | ---: | ---: |
| 1 | 30,710,915 ns | 999,688,898 ns | 32.552x |
| 2 | 30,265,415 ns | 999,323,760 ns | 33.019x |
| 3 | 30,307,572 ns | 1,006,322,340 ns | 33.204x |
| 4 | 30,380,237 ns | 989,776,558 ns | 32.580x |
| 5 | 30,183,621 ns | 994,276,173 ns | 32.941x |

The median paired full-case loss is 32.941x; process RSS is 4.007x CGAL. Both
remain explicit Phase 17 failures.

The refreshed 900-iteration retained-input small controls also remain losses:

| Fixture | CGAL median | Hypermesh median | Median paired ratio |
| --- | ---: | ---: | ---: |
| crossing octahedra | 114,032 ns | 305,184 ns | 2.680x |
| affine boxes | 371,064 ns | 703,351 ns | 1.904x |

No favorable aggregate hides those rows.

## Profile and next clean targets

The final CPU-11 dense-17 frame-pointer profile contains 894 samples with none
lost. Leading self costs are:

| Self | Symbol |
| ---: | --- |
| 14.56% | `Real::exact_rational_homogeneous_point2_i128` |
| 12.71% | direct `orient2d_with_policy` |
| 7.32% | retained orientation path |
| 4.49% | `Real::exact_dyadic_f64` |
| 3.23% | `TriangleTopology::replace_region` |
| 3.08% | `memcmp` |
| 2.85% | `surface_arrangement::corefine_face` |
| 2.84% / 2.65% | rational numerator / denominator |
| 2.75% | approximate constraint bounds |
| 2.31% | constraint validation |
| 2.29% | triangle-neighbor construction |
| 1.30% | retained point-on-segment query |
| 1.28% | `recover_constraints` |
| 0.92% | cavity-side triangulation |
| 0.12% | proper-crossing side classifier |

The corridor itself is no longer a leading dense self cost. The clean next
targets are sharing more exact projected-point conversion across line queries,
reducing the combined direct/retained orientation cost, and reducing topology
validation/replacement work while preserving the same proofs. The full-fixture
profile is retained separately, but its opt-in checksum verification spawned a
hashing child; dense-17 is the uncontaminated optimization profile.

Artifacts:

- `target/phase17-complete-corridor-dense17.perf.data`, SHA-256
  `1255d7c0a332558410c157cb5882257d724a8f799c1f6be9e704ebd06b3be28e`
- `target/phase17-complete-corridor-full.perf.data`, SHA-256
  `3e43530905468ef4ab0fcf2defee1ed3cd5452a4479e61a9b00bdb84b5c72f34`

## Source and linked size

Across the four changed crates and benchmark/corpus support, this combined
checkpoint adds 884 and removes 358 source lines, including API documentation,
proof checks, and focused regressions. Current Tokei counts are 11,668 Rust code
lines for Hypertri and 28,355 across the measured Hypermesh production,
tests, benches, examples, competitive support, and fuzz targets. Source
consolidation remains open.

The canonical Rust 1.97.0 consumer matrix is effectively flat versus the
retained-query parent. Values are linked native text bytes and optimized
`wasm-opt -Oz` bytes:

| Features/profile | Native general / immediate | WASM general / immediate | Movement range |
| --- | ---: | ---: | ---: |
| default release | 2,008,074 / 2,011,226 | 1,424,277 / 1,426,113 | +0.011% native; +0.029–0.030% WASM |
| default size | 1,099,823 / 1,100,803 | 687,273 / 687,690 | -0.116% native; +0.077–0.099% WASM |
| all-feature release | 2,143,875 / 2,146,723 | 1,498,946 / 1,500,919 | +0.010–0.011% native; +0.024–0.025% WASM |
| all-feature size | 1,102,159 / 1,103,131 | 687,100 / 687,511 | -0.117% native; +0.058–0.099% WASM |

Performance takes priority, but the small release/WASM increases and source
growth stay open. The direct competitive probe is not a canonical size row:
its new opt-in full-fixture selector intentionally links the external asset
decoder so Hypermesh and CGAL can consume one exact fixture generator.

## Call graph and removal audit

The refreshed five-crate static graphs contain:

| Scope | Nodes | Edges | SHA-256 |
| --- | ---: | ---: | --- |
| production | 15,360 | 25,646 | `ae504c3c801353f59604f6e8647da06ce356e4d5aced85b9a801d26596bc0ed9` |
| tests and examples | 17,705 | 29,018 | `2297740696d7458fd1904c8830f1c17a12feb5e524354998efa8337fc951b7bc` |
| all evidence | 21,750 | 35,118 | `a2ce59ac2dcc7cf70640442ec95dc19a908c3df2e5261da230c00e9c03f21e3c` |

The production graph records one exact route:

```text
surface_arrangement::corefine_face
  -> constrained_triangulation_convex_hull
  -> topology::triangulate_point_set
  -> insert_constraints_topology
  -> recover_constraints
  -> recover_constraint
  -> locate_constraint_from_endpoint
  -> recover_constraint_cavity
  -> triangulate_cavity_region
  -> TriangleTopology::replace_region
  -> TriangleTopology::replace_region_with_scratch

insert_constraints_topology
  -> canonicalize_unconstrained_edges
```

Exact source and graph searches find zero EMBER, `segment_trace`, or
`local_bsp` identifiers across production, tests, examples, benches, and fuzz
sources. There is one `surface_arrangement` Boolean engine and no compatibility
shim.

Graph artifacts are:

- `target/phase17-complete-corridor-callgraph-production/callgraph.json`
- `target/phase17-complete-corridor-callgraph-tests/callgraph.json`
- `target/phase17-complete-corridor-callgraph-all/callgraph.json`

## Validation and open exits

Final validation passes:

- Hypertri's 85 all-feature unit tests, 25 adversarial tests, six differential
  tests, ten fuzz-property tests, five policy tests, README tests, and doctests;
- Hypermesh's 156 all-feature unit tests, eight Boolean tests, thirteen active
  competitive tests, seventeen corpus-manifest tests, three intersection-corpus
  tests, nine policy tests, README tests, and doctests;
- default/all-feature and no-default-feature configurations;
- warning-denied all-target/all-feature Clippy for Hypertri and Hypermesh;
- warning-denied rustdoc, formatting, and both fuzz-bin build matrices;
- all-feature Hypermesh `end_to_end`, `competitive`, and `dispatch_trace`
  benchmark targets;
- post-change ASan/libFuzzer campaigns completing 13,325 Hypertri
  `topology_invariants` and 5,252 Hypermesh `boolean_pipeline` executions with
  no failure (LeakSanitizer disabled because the managed runner uses ptrace);
- the post-change dense-65 complete policy-equality gate;
- both-policy dense-65 and full-YeahRight allocator probes;
- the release full-YeahRight exact oracle;
- current small and full CGAL EPECK trials;
- both canonical size matrices; and
- all three call-graph scopes and exact removed-name searches.

Phases 11, 17, and 18 remain open. The corpus still needs more legally
distributable external real-world and generated pathological meshes plus a
completed fuzz-mutation-source audit. Every current CGAL runtime loss, the
full-case RSS gap, the broad small-path regression, source and optimized-WASM
recovery, deeper stage-lifetime attribution, the deferred controlled-caller
matrix, and the final requirement-by-requirement release audit remain open.

Machine-readable raw samples and exact flags are in
`phase11-17-path-complete-line-corridors.toml`.
