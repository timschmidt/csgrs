# Performance and Reference Audit

This document records the reference-driven optimization audit for `hyperlimit`.
Changes are retained only when the exact report contract remains intact and a
focused Criterion comparison shows a meaningful improvement.

## Retained optimization

### Collapse redundant Real sign-resolution passes

`resolve_real_sign` already asks `decide_real_sign` for the value's structural
facts before predicate-specific filters. Exact-rational structural facts always
carry `sign: Some(_)`, so they decide on that first pass. The former
`exact_real_sign` stage then re-read the same facts: it was unreachable for
exact rationals and returned `None` for every unresolved symbolic value. The
stage and its duplicate structural traversal are now removed; predicate
filters, exact callbacks, bounded refinement, certainty, and escalation remain
in the same order.

Scalar sign classification and Real ordering have no predicate-specific filter
or exact callback. They now use `resolve_real_sign_direct`, which calls
hyperreal's certificate-bearing `certified_sign_until` once and maps
`StructuralFacts`/`ExactZeroScale` to the structural stage and
`BoundedRefinement` to the refined stage. The exact-rational ordering branch is
unchanged. A close `pi < 355/113` regression proves that symbolic ordering still
reports `PredicateCertificate::BoundedRefinement`.

On the csgrs Reuleaux region-Boolean workload, a 30-sample, four-iteration A/B
measured 14.610 ms/op before and 14.402 ms/op after, a 1.43% reduction. The
patched interquartile range was 14.350--14.455 ms/op versus
14.502--14.722 ms/op before. Dispatch tracing removed 21,759 events
(206,175 to 184,416): 7,253 redundant `exact_real_sign` calls and the two
associated fact-query events for each call. All 7,251 successful refinements,
two unknown outcomes, and predicate decisions were unchanged.

The direct certificate route was measured separately against that already
simplified resolver: 14.792 ms/op before and 14.125 ms/op after, a 4.51%
reduction in the paired run. Its 14.027--14.250 ms/op interquartile range did not
overlap the 14.733--14.927 ms/op control range. The final trace contains 141,208
events, 64,967 fewer than the original 206,175, while retaining the same 7,251
refined decisions and two explicit unknown outcomes.

### Reuse point/ring edge orientations

`classify_point_ring_even_odd_report` formerly evaluated `orient2d(a, b,
point)` once while classifying the point against every edge, then evaluated the
same determinant again for each y-straddling edge. The report requires the full
`OffLine` versus `CollinearOutside` distinction, so the Hormann--Agathos idea of
skipping most boundary predicates cannot be applied without weakening retained
evidence. The implementation now certifies orientation once, reuses its sign for
crossing parity, and invokes only the collinear interval classifier when the
sign is zero.

Focused benchmark:

```sh
cargo bench --bench predicates -- \
  'exact_rational_kernels/ring/even_odd_reports' \
  --warm-up-time 1 --measurement-time 3 --sample-size 50
```

| Variant | Mean per 512 queries | Change |
| --- | ---: | ---: |
| Recompute orientation on straddling edges | 634.59 us | baseline |
| Reuse certified orientation | 504.88 us | -20.41% |

Criterion reported the improvement as statistically significant (`p = 0.00`).
The focused ring tests include inside, outside, edge boundary, vertex straddle,
indexed topology, repeated closing vertices, source replay, and the retained
`CollinearOutside` edge classification.

### Reuse triangle/plane vertex sides across edge reports

The 3D triangle/triangle classifier first certifies all three vertices of each
triangle against the opposite supporting plane. Its non-coplanar path then
classifies all six edges against the opposite triangle. Previously every edge
classifier recomputed the two endpoint `orient3d` signs, so each retained vertex
side was evaluated twice more. The Guigue--Devillers orientation decomposition
supports carrying those signs forward: the implementation now keeps both
three-element side arrays, passes each edge's certified endpoint pair into the
unchanged segment/triangle tail, and reuses the already prepared supporting
plane for the exact crossing construction.

The same prepared oriented-plane filters also replace a redundant second linear-
form preparation during the initial plane tests. Both coplanar and non-coplanar
sentinels improved:

| Workload | Before | After | Change |
| --- | ---: | ---: | ---: |
| Non-coplanar triangle report replay | 26.547 us | 20.609 us | -21.62% |
| Coplanar triangle report replay | about 20.07 us | 14.898 us | -25.76% |

Criterion reported both changes as statistically significant (`p = 0.00`). The
public report remains unchanged: all six non-coplanar edge relations are still
retained, and degeneracy, separation, boundary, proper crossing, and coplanar
tests replay successfully.

## Rejected experiments

### Search short Farkas certificates before active sets

After the origin candidate failed, an experimental schedule searched all one-
and two-plane Farkas certificates before constructing the remaining geometric
active-set candidates. It quickly handled opposed slabs but imposed quadratic
proof-search work on the shifted feasible case.

| Variant | Mean per mixed feasible/infeasible query | Change |
| --- | ---: | ---: |
| Existing active-set-first schedule | 16.51 us | baseline |
| Early one/two-plane certificate search | 18.37 us | +10.69% |

Criterion reported a significant regression (`p = 0.00`), so the experiment
was removed completely.

### Reorder k-DOP axes

Klosowski et al. observe that testing widely separated directions in sequence
may improve early exits, but explicitly leave a specially designed ordering as
future work and provide no evaluated order. `SupportDop3` also preserves caller
slab order and reports terminal source indices. No speculative reorder was
introduced.

## Reference-to-implementation audit

| Reference | Relevant idea | Result in `hyperlimit` |
| --- | --- | --- |
| Arvo, *Transforming Axis-Aligned Bounding Boxes* | Select min/max affine contributions rather than transform every corner. | `classify_plane_aabb3_report` already performs the corresponding exact term-interval reduction and retains selected-corner evidence. A general affine box-transform carrier is outside this crate's present API. |
| Bareiss, integer-preserving elimination | Fraction-free cubic determinant evaluation with controlled intermediate growth. | Already used by the exact-rational `orient_d` and `insphere_d` paths. |
| Bentley--Ottmann, geometric intersections | Event queue plus ordered sweep status gives output-sensitive batch segment intersection. | This crate supplies the exact segment predicates needed by a sweep. Arrangement/event ownership belongs in `hypercurve` or `hypertri`, not a per-pair predicate API. |
| de Berg et al., *Computational Geometry* | Robust plane sweep, randomized low-dimensional LP, convex hull, point location, and ownership-aware planar subdivisions. | Confirms the separation between exact primitive decisions here and topology/data structures in higher crates. The randomized LP alternative is covered below. |
| Ericson, *Real-Time Collision Detection* | Prepared bounding volumes, separating axes, early rejection, degeneracy handling, and robustness. | Prepared lines, planes, segments, circles, spheres, DOPs, and exact interval/SAT-style predicates already implement these principles without epsilon decisions. |
| Guigue--Devillers, triangle overlap | Boolean triangle overlap using orientation signs only, minimizing intermediate constructions and `orient3d` calls. | The public triangle/triangle report cannot adopt the paper's boolean-only output, but it now reuses the six initial vertex/plane signs across all edge reports, producing the 21.62% non-coplanar improvement above without discarding evidence. |
| Gustavson, sparse matrix algorithms | Row-wise sparse multiplication through an unordered accumulator/merge. | Hyperlimit determinants are tiny dense matrices with structural sparse-coordinate schedules, not general SpGEMM. Introducing sparse matrix storage would add overhead at present dimensions. |
| Hormann--Agathos, point in polygon | Half-open y-straddles, determinant-based crossings, integrated boundary handling, and cheap rejection before division. | Half-open straddles and exact orientation crossings were already present. Reusing the retained edge orientation produced the measured 20.41% improvement above. |
| Moore, *Interval Analysis* | Inclusion-preserving interval enclosures can certify results that exclude zero. | `certified_interval_sign`, certified balls, and determinant filters already use enclosures only as proofs; intervals crossing zero escalate rather than guess. |
| Möller, triangle intersection | Plane-side rejection, line-of-intersection projection on the largest component, then one-dimensional interval overlap. | Hyperlimit uses exact plane classifications and projection-aware segment/triangle composition, but also handles degeneracy and coplanarity and preserves replayable reports that Möller's fast boolean path omits. |
| Klosowski et al., k-DOP BVHs | Fixed-direction support intervals, early separation, tightness/cost tradeoffs, and hierarchy construction. | Exact witnessed support slabs and conservative overlap reports are implemented. BVH construction and temporal updates belong to higher crates. Axis-order speculation was rejected above. |
| Seidel, low-dimensional LP | Randomized incremental LP with recursive boundary subproblems and expected linear time for fixed dimension. | Current feasibility reports are deterministic and preserve exact witnesses or Farkas certificates. Seidel requires canonical lexicographic degeneracy handling and randomized scheduling; its paper also notes the implementation is not necessarily practical. No workload currently justifies replacing the exact active-set solver. |
| Schrijver, linear/integer programming | Polyhedral feasibility, duality, and Farkas certificates. | Infeasible 3D halfspace reports search support sets of at most four planes and replay exact nonnegative multiplier certificates. The attempted certificate reschedule regressed and was removed. |
| Shewchuk, adaptive robust predicates | Fast filters followed by exact/adaptive stages, with degeneracy decided exactly. | Dispatch tracing sends the standard 512-case easy and near-degenerate `orient2d`, `orient3d`, `incircle2d`, and `insphere3d` workloads through the certified Real filter with no fallback traffic. An additional expansion stage is not supported by current traces. |
| Yap, exact geometric computation | Separate combinatorial decisions from numeric approximation and refine only when certification requires it. | `PredicateOutcome`, certainty/escalation metadata, prepared exact objects, and replayable reports follow this architecture throughout the crate. |

## Trace evidence

The generated `dispatch_trace.md` standard workload contains 512 easy and 512
near-degenerate cases for each core predicate. All terminate at the certified
Real filter; no exact-rational, adaptive-refinement, or unknown fallback route
is exercised by those workloads. Transformed exact-rational cases separately
exercise the Bareiss and lifted determinant routes.

The triangle/triangle trace separately records
`reuse-plane-sides-for-edges`, making the retained Guigue--Devillers scheduling
choice distinguishable from the coplanar projection and early plane-separation
paths.

## Verification

The retained implementation is checked with:

```sh
cargo test --all-targets
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --no-deps
cargo check --examples --benches
cargo fmt --all -- --check
git diff --check
```
