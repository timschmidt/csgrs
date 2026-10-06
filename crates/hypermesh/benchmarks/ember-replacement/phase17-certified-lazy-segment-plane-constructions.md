# Phase 17 checkpoint: certified lazy segment/plane constructions

Date: 2026-08-05

Implementation parent: Hypermesh `a59bc843`

Implementation commits: Hyperreal `f09c147`; Hypermesh `39250eb2`

Companion data: `phase17-certified-lazy-segment-plane-constructions.toml`

Status: exact construction-scheduling, runtime, heap, and size checkpoint;
Phases 17 and 18 remain open

## Outcome

The one surface-arrangement Boolean engine no longer constructs every exact
segment/plane crossing coordinate before it knows whether that crossing will
survive interval clipping. Expensive exact-rational source planes can instead
retain a certified binary64 coordinate enclosure and an exact construction
recipe. Enclosure-separated points compare without allocating the exact
coordinate; an overlapping enclosure or a surviving output point constructs
the same exact `hyperreal::Real` ratio on demand.

The schedule is driven only by reusable representation facts:

1. the polygon is a triangle with exactly two proper plane crossings, so one
   shared three-query filter can amortize both crossings;
2. the supporting plane is not backed by the already-cheap lossless primitive
   source-plane representation;
3. at least one stored exact plane coefficient is multi-limb or very large;
4. all three retained source-point queries and the plane filter are available.

Every declined condition takes the existing eager exact path. There is no
fixture, coordinate, triangle-count, operation, expected-result, policy-name,
competitor, or benchmark dispatch. General polygons, one-crossing boundary
configurations, compact primitive planes, symbolic planes, failed filters, and
unsafe floating ranges all retain the complete exact algorithm.

This is one representation-level schedule inside the production engine. It is
not a compatibility shim, second Boolean engine, retry, topology shortcut, or
benchmark-specific implementation.

## Scalar proof and retained exact recipe

Hyperreal now exposes `Rational::storage_class()`, a constant-time coarse cost
fact over the stored numerator and denominator bit lengths. It does not
canonicalize, approximate, or expose a numeric value. The mesh layer uses only
`WordSized` versus `MultiLimb`/`VeryLarge` to decide whether the interval work
can plausibly repay deferred exact construction.

`RationalLinearForm4Filter::segment_intersection_coordinate_enclosure` reuses
the fixed plane filter and the two retained homogeneous point queries. Each
query's positive homogeneous scale is divided out independently. The plane's
positive normalization scale remains common to the two support values and
therefore cancels from the segment parameter. For certified support intervals
`p > 0` and `n < 0`, the coordinate enclosure evaluates

```text
(p * x_negative + |n| * x_positive) / (p + |n|)
```

with outward-rounded interval addition, multiplication, reciprocal, and
division. Opposite signs must be interval-certified. Non-finite operands,
zero-spanning divisors, underflow/overflow boundaries, an invalid axis, an
on-plane endpoint, or overlapping support signs return `None`.

The exact recipe stores the borrowed plane, positive and negative endpoints,
and support-line axis. An exact comparison constructs only the coordinate
numerator and positive denominator. Output materialization constructs only the
segment parameter numerator and denominator, avoiding the otherwise unused
coordinate numerator. Both paths use the existing fused exact product-sum
operations and produce the same `Real` point as the eager path.

The four-term sign filter and the enclosure filter share one always-inlined
value/error kernel. Ordinary inlining had caused LLVM to outline the hot sign
path; explicit always-inlining recovered that control regression without
duplicating the arithmetic source.

## Exactness and policy

- `hyperreal::Real` remains the canonical construction scalar.
- `STRICT` has no approximate terminal.
- `APPROXIMATE_512` can terminate only through Hyperlimit's existing 512-bit
  terminal.
- The operation-local `DecisionContext` still aggregates terminal use into one
  `MeshCertainty`.
- A certified enclosure affects scheduling and ordering only. A filter miss
  reaches exact construction; it never changes accepted topology.
- Every rational and large-mesh row below is `Certified` and byte-identical
  across the two policies.
- The existing deep-symbolic approximate-policy case still reports
  `Approximate512Consumed`; this change adds no second approximation boundary.

Hyperreal tests include 20,000 deterministic random rational planes and
segments. More than 4,000 proper crossings must certify, and every returned
interval must contain the exact rational coordinate enclosure. Separate tests
cover reversed endpoint signs, on-plane and same-side endpoints, invalid axes,
unrepresentable ranges, zero coordinates, and exact fallback. Hypermesh tests
materialize a lazy crossing under both policies and verify the exact point and
`Certified` certainty.

## Full-case dispatch evidence

The feature-gated competitive probe can now record the full fixture without
changing the default binary. One strict full-resolution rotated YeahRight
intersection reports:

| Dispatch path | Count |
| --- | ---: |
| proper segment/plane crossings | 130,379 |
| eager exact schedule | 72,673 |
| certified lazy schedule | 57,706 |
| lazy coordinate enclosure available | 57,411 |
| lazy enclosure unavailable, exact fallback retained | 295 |
| lazy crossings materialized into exact points | 5,536 |
| segment/segment comparisons resolved by enclosures | 72,173 |
| segment/segment exact comparisons | 548 |
| mixed affine/segment comparisons resolved by enclosures | 144,821 |
| mixed exact comparisons | 40 |

Thus 52,170 of the 57,706 scheduled crossings avoid exact point
materialization. The remaining paths are explicit and measured; none is
discarded.

## Validation and corpus

The final source passes:

- Hyperreal `cargo test --all-features` (656 primary library tests plus every
  integration and documentation test);
- Hypermesh `cargo test --all-features` (216 executed tests and seven
  documented opt-in/manual ignores);
- both crates' no-default-feature checks, all-target/all-feature clippy with
  warnings denied, all-feature rustdoc, formatting, and diff checks;
- both crates' all-feature benchmark builds;
- the Hypermesh fuzz workspace build;
- AddressSanitizer/libFuzzer `boolean_pipeline`: 2,182 seed files discovered,
  3,815 executions in 31 seconds, 496 MiB maximum sanitizer RSS, no failure.

LeakSanitizer alone remains disabled with `ASAN_OPTIONS=detect_leaks=0` because
the managed environment prevents its final ptrace scan. The fuzzer ran against
a temporary corpus copy, so no generated artifact or corpus file entered this
checkpoint.

Both policies also completed the representative cross-family fixture set:

| Fixture | Output vertices | Output triangles | Certainty |
| --- | ---: | ---: | --- |
| dense coplanar boxes 16 | 3,074 | 6,144 | Certified |
| dense crossing grid 17 | 5,444 | 11,908 | Certified |
| sparse multishell tetrahedra 64 | 640 | 1,024 | Certified |
| transverse self-PWN clusters 64 | 644 | 1,028 | Certified |
| wide rational boxes 64 | 2,410 | 4,816 | Certified |
| thin dyadic boxes 64 | 2,410 | 4,816 | Certified |
| clipped voxel torus 33 | 1,730 | 3,456 | Certified |
| subdivided overlapping boxes, 3,072 each | 2,410 | 4,816 | Certified |
| full rotated YeahRight, 11,894 each | 0 | 0 | Certified |

The largest dense-crossing heap fixture independently produced 73,844 vertices
and 164,068 triangles under both policies.

## Deterministic performance controls

The small-control table reports five-run means for 1,000 strict arrangements
materializing union, intersection, difference, and reverse difference. Parent
and current binaries were run adjacently on CPU 11. Retired instructions are
the control authority; clocks and cycles are frequency-sensitive.

| Fixture | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| crossing octahedra | 2,856,120,360 | 2,860,759,250 | +0.162% | 483,841,689 | 483,892,347 | +0.010% |
| overlapping boxes | 3,629,023,122 | 3,636,107,362 | +0.195% | 616,862,720 | 616,135,705 | -0.118% |
| affine boxes | 6,970,505,217 | 6,981,948,247 | +0.164% | 1,187,302,596 | 1,187,019,004 | -0.024% |
| identical boxes | 3,765,819,087 | 3,763,428,054 | -0.063% | 639,861,472 | 636,462,898 | -0.531% |

The schedule does not activate on these cheap representations. Their bounded
instruction movement is the measured code-layout/check cost, not a hidden
fast-path substitution.

## Full-resolution historical and competitive boundary

Five strict runs of the checksum-pinned 11,894-by-11,894 rotated YeahRight
intersection give:

| Metric | `a59bc843` | Current | Change |
| --- | ---: | ---: | ---: |
| retired instructions | 10,343,264,422 | 9,786,228,610 | -5.385% |
| retired branches | 1,762,073,145 | 1,649,958,392 | -6.363% |
| branch misses | 23,414,063 | 22,569,040 | -3.609% |
| cycles | 4,044,737,521 | 3,815,823,327 | -5.660% |
| wall median | 950.602 ms | 899.417 ms | -5.385% |

Direct final samples were 897.774 ms / 70,524 KiB strict and 912.780 ms /
70,600 KiB approximate-512. Both returned the exact empty result with
`Certified` certainty.

Historical EMBER required 3,312.66 seconds and 329,352 KiB. The current strict
sample is about 3,683x faster and uses 78.59% less maximum RSS. The pinned CGAL
6.0.3 EPECK result remains 0.09 seconds and 15,516 KiB, leaving open deficits
of about 9.99x runtime and 4.55x process RSS.

Using the established independently validated exact-input CGAL EPECK medians,
the adjacent current Hypermesh wall medians remain 2.393x crossing, 2.733x
overlapping, and 1.926x affine. These are improvements over prior Hypermesh
clock checkpoints but are still explicit competitive gaps; the deterministic
retired-work rows remain the regression authority.

## Large-fixture heap

The allocator probe measures requested payload during one complete Boolean and
subtracts retained decoded input. Every current strict/approximate-512 pair is
identical.

| Fixture | Input triangles | Incremental peak | Allocations | Added bytes | Change from parent |
| --- | ---: | ---: | ---: | ---: | --- |
| general subdivided boxes | 6,144 | 12,132,042 | 35,729 | 17,985,166 | identical |
| dense coplanar boxes 32 | 24,576 | 55,759,202 | 926,776 | 220,619,719 | identical |
| transverse self-PWN clusters 512 | 4,100 | 11,451,636 | 197,040 | 30,927,366 | identical |
| wide rational boxes 2,048 | 6,144 | 26,899,026 | 1,564,805 | 384,385,246 | identical |
| thin dyadic boxes 2,048 | 6,144 | 37,980,930 | 1,037,874 | 356,750,078 | identical |
| dense crossing grid 65 | 1,572 | 197,318,844 | 33,193,334 | 6,852,763,150 | identical |
| full rotated YeahRight | 23,788 | 53,217,804 | 9,571,455 | 556,743,274 | traffic reduced |

The full-case peak remains governed by other simultaneous owners and is
unchanged. Allocation calls fall 11.41%, reallocations fall 17.42%, and added
payload traffic falls 8.74% relative to `a59bc843`.

## Source and linked size

Hypermesh `src` is 20,641 Tokei code lines, +290 (+1.43%) from the parent.
Hyperreal production source adds a net 199 lines for the stored-cost fact and
certified interval kernel. The canonical linked consumers move as follows:

| Features/profile | Parent native text | Current native text | Change | Parent optimized WASM | Current optimized WASM | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| default release | 2,024,242 | 2,038,986 | +0.728% | 1,441,682 | 1,456,412 | +1.022% |
| all-feature release | 2,159,459 | 2,176,715 | +0.799% | 1,516,808 | 1,532,843 | +1.057% |
| default size | 1,115,895 | 1,123,855 | +0.713% | 699,921 | 706,902 | +0.997% |
| all-feature size | 1,117,975 | 1,125,951 | +0.713% | 699,916 | 706,185 | +0.896% |

The size cost is accepted for this performance checkpoint because the
representative wide case removes 5.39% instructions and 8.74% allocation
traffic. Recovering source, native, and WASM size without losing the general
schedule remains open.

## Call graph and removal audit

The workspace utility scanned exactly Hyperreal, Hyperlattice, Hyperlimit,
Hypertri, and Hypermesh; Hypercurve and HyperSolve were excluded:

- production: 15,540 nodes / 25,984 edges;
- tests, examples, benches, and fuzz included: 21,943 nodes / 35,499 edges;
- 49 direct Hypermesh/Hypertri-to-Hyperlimit predicate boundaries, unchanged;
- one `build_surface_arrangement -> assemble_surface_cells` edge;
- direct schedule edges through retained queries, the certified Hyperreal
  enclosure, exact comparison fallback, and exact point materialization;
- zero exact EMBER, `segment_trace`, `local_bsp`, or `SurfaceSheet` nodes.

The generated artifacts are under
`target/callgraph-phase17-lazy-crossing-{production,all}`. Static graph
resolution is navigation/removal evidence, not a substitute for runtime,
policy, corpus, or sanitizer gates.

## Measured alternatives

- A cache of exact support results was removed: repeated observations already
  reuse Hyperreal retained facts and the cache added ownership/lookup cost.
- Making every rational crossing lazy greatly reduced exact construction but
  regressed cheap controls; the retained storage-cost crossover replaced it.
- Refactoring the hot sign filter through an ordinarily inlined helper caused
  LLVM outlining; one shared always-inlined kernel retained clean source and
  recovered the regression.
- Outlining the enclosure evaluator did not shrink linked text and added
  branches; it was removed.
- Reciprocal-plus-multiply interval division is retained because alternating
  paired measurements reduced full-case cycles by about 0.18--0.20% versus
  four direct endpoint divisions. A hand-written min/max rewrite was slower
  and was removed.

## Open work

This checkpoint does not declare Phase 17 or Phase 18 complete. All CGAL
runtime/RSS deficits, the small cheap-control instruction cost, ordinary
large-fixture peak recovery versus older checkpoints, linked/source size,
broader real-world and generated pathology admission, fuzz mutation-source
audit, and the final requirement-by-requirement release audit remain open.

Any later optimization must preserve the contribution-complete topology, one
exact decline path, both Hyperlimit policies, aggregate certainty, and the
prohibition on fixture- or competitor-aware dispatch.
