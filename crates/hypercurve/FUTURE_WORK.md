# Future work: exact-composition performance and S5–S7 consolidation

This file records the remaining directions from the 2026 API and machinery review: what is still open, the evidence behind each direction, and approaches that were measured and rejected, so they are not repeated.

The detailed chronological log, including commit hashes, timings and samples, is `exactcore-hyper-comparison/audits/hypercurve-api-review-2026-09-12/implementation.md`. The original review is `review.md` beside it.

## Current state

The exact-only principal API, the `CurveRegion2` migration, and S1 (`SelectedScalar2` dispatch) are complete. S6 is partly done; the remaining sites are listed below.

The ignored composition harness (`tests/hypercurve_composed_workloads.rs` and `tests/hypercurve_open_path_compositions.rs`) has 9 of 24 cases completing under a 300 s budget:

| Case | Time |
| --- | --- |
| `repeated_inward_miter` | 21 s |
| `beveled_rational_seed` | 56 s |
| `chamfered_quadratic_bevel_stroke_round_trips` | 61 s (section 8) |
| `weighted_conic` | 61 s |
| `booleans_then_double` | 76 s |
| `bevel_then_round` | 107 s |
| `mixed_family` | 123 s |
| `mixed_cubic_arc` | 203 s |
| `nested_xor` | 290 s |

The other 15 still exceed the budget, and the hyperbola-pair stroke is a known stall. `round_stroke_boundary_of_a_filleted_arc_corner_refillets_and_erodes` is a separate open case: it ends in a typed blocker (section 8). All of them stay `#[ignore]`d, because they run far above the suite's per-test scale.

Sample a stalled case with `gdb -p <pid> -batch -ex "thread apply all bt"` against the test binary. Run it directly, for example `target/release/deps/hypercurve_open_path_compositions-* <name> --ignored --exact`, rather than through `cargo test`. Take the PID from `ps`, not `pgrep -f`, which matches the calling shell.

## 1. Signing radical tower values without squared dense polynomials

**Where the stalls are.** Most remaining stalls end in `RecursiveQuadraticValue::sign` → `hypersolve::dense_two_positive_square_root_sum_sign`. This affects the two-root group (`filleted_union`, `wide_weighted`, the chamfered strokes) and the tail of `refilleting`.

**Why the rule is not the problem.** The recursive rule (sign of a + b√d from the signs of a, b and a² − b²d) is sound. Every leaf sign, though, goes through `dense_polynomial_tuple_sign` on a fully expanded tensor:
- coefficients reach about 25,000 bits;
- degrees reach 15–30 per axis;
- nonzero values then need about 2,000 bits of root refinement to separate, roughly the coefficient size, so the cost is coefficient growth, not geometry.

**Proposed design** (from a code-grounded planning pass): keep the recursive rule, and change only the leaves.
1. **Lazy leaves.** Represent a leaf as an expression DAG over the reduced inputs (r, f, g, p, s1, s2) instead of an expanded tensor.
2. **Interval step.** Evaluate the DAG by interval arithmetic over the shared `RecursiveQuadraticBaseField::source_box` refinement.
3. **Modular nonzero certificate.** Compute the leaf's norm modulo a few primes, directly from the inputs, reducing modulo the selected root's polynomial after each product. Use a good prime: leading coefficients and denominators must be units. If gcd(m mod p, N mod p) is 1, the leaf is nonzero, and interval refinement is complete: refine until it separates, as `sign_with_nonzero_certificate_over_source_box` already does. The exact 25k-bit norm is never built.
4. **Exact fallback.** Use exact local-field arithmetic only for zeros or a likely common factor. One source uses `AlgebraicField`. Two sources use a nested `LocalFieldElement` polynomial ring over the lower-degree root, reusing `predicate_vanishes_at_isolated_root` and `predicate_sign_by_isolation`.

The new path would plug into `RecursiveQuadraticValue::sign` after the existing interval and compact-witness fast paths, keeping the dense route as the final fallback until it is proven. Under a bounded predicate budget, run only the interval and modular steps.

**Measured negatives, so they are not repeated:**
- **Signing the two-radical expression in `AlgebraicField`** (exact reduced field elements, no modular step). On lazy-`Real` coefficients it moved the cost into a field gcd. On rational coefficients it took 94 s for 10 calls, against about 40 s on the dense route with modular-first filter signs. Field reduction alone does not remove the cancellation.
- **A rational interval box enclosure of P(β, x)** after the modular nonzero certificate: no gain.
- **Dyadic shortening of wide rational brackets** before the Bernstein basis change: no gain.
- **A mean-value enclosure** (exact midpoint plus a Horner bound on P′) in the coprime bisection: slower. Exact interval Horner over endpoints of about 2,000 bits inflates intermediates.

The modular step is what is missing from all of these. Implement it first, then measure.

## 2. Exact representation of generated geometry

**Fixed.** A fillet against a line used the unit direction d/|d|, so √|d|² leaked into the fillet contact's defining polynomial as lazy `Real` DAGs. The fix (cb372d03) builds the rational norm of the offset-line incidence, R² − sd²|d|²X₀², and filters candidates to the authored factor.

**Look for the same pattern elsewhere**: any generated object whose `Real` coordinates come from `sqrt` or division of algebraic projections. A strong symptom is that `format!("{:?}", real)` on a coefficient takes seconds. Candidates:
- line offsets in stroke, chamfer and bevel construction (`curve_fillet_centers.rs` `FilletOffsetCarrier2::Line` offset, `line_unit_direction`);
- concentric-arc frames;
- any `LineSeg2::translated` by a unit normal.

**Degree-214 fillet centre in `round_stroke_of_a_filleted_nurbs_pair`.**
- The fillet centre's parameter (`anchor_evidence.center_parallel`, built in `curve_fillet_centers.rs` and reconstructed through `curve_corner_chain.rs` `RetainedFilletRadialFrame2::ParallelNormal`) is a degree-214 global algebraic root of the offset-offset intersection resultant.
- It is square-free, so the degree is probably intrinsic.
- Every later kernel (`selected_parallel_normal_parallel_intersections` → selected-fiber isolation over a degree-214 field) pays for it.
- **Direction:** keep that centre as a selected fiber over the other offset's parameter, with low-degree field arithmetic, instead of publishing a global `BezierParameter2::Algebraic`. This means changing how fillet centres are authored, not adding a local fast path.

## 3. Selected-fiber common-root counting

**Where it stalls.** `filleted_boolean_miter_erosion` and `stroke_fillet_stroke` stall in `count_bivariate_fiber_system_roots`. The stuck query counts common roots of fiber polynomials of degree 36 and 40, with coefficients up to 389 bits, over a degree-14 retained field. The existing modular coprimality certificate does not decide it, so the two probably share a factor.

**Measured negative.** Sending large fibers to the reducing Euclidean chain, instead of `rational_local_subresultant_rows`, only moved the cost into `normalize_local_polynomial` field divisions.

**Ideas:**
- A modular-plus-CRT gcd over Q(α) (gcd modulo primes at α-images, then rational reconstruction verified by exact division). This is the standard cure for coefficient swell.
- A square-free, Descartes or Bernstein count of the gcd's roots in the interval, instead of a full Sturm chain.

## 4. S6 remaining sites (`promote_curve_region_bezier_parameter`)

| Status | Sites |
| --- | --- |
| Done | group A, site 5, group C sites 7, 8 and 11b |
| Deliberate cold fallbacks | cusp_overlap.rs:167 and :208, cusp_semicircle.rs:3709 |
| Group C, two-radical cut relations | sites 6 (cusp_semicircle.rs:3870) and 10 (parameters.rs:4924) |
| Group B | site 4 (parallel_kernel.rs:2820) |
| Group D, frame centres seeding fields or kernels | parameters.rs:2944, fragment.rs:1293, frames.rs:578, chord_kernel.rs:6190, rational_kernel.rs:1486, parallel_kernel.rs:1304, cusp_semicircle.rs:7387 |
| Genuinely global | site 9 (cusp_semicircle.rs:6024), site 18 (tangent.rs:1395) |

**Group C, sites 6 and 10.** These flow through `independent_diameter_sum_is_zero` and need a selected-fiber two-radical zero test (cf. `square_root_sum_sign`) threaded through four `BezierParameter2`-typed functions.

**Group B, site 4.** Widening `BezierAlgebraicCuspSemicircleParallelContact2.parallel_parameter` to `CurveParameter2` gives 32 compile errors.
- About 19 are constructors that need only `.into()`.
- About 13 are readers of Bezier-only APIs.
- Do this after moving those readers to region-parameter APIs.

**Group D.**
- These need a `parallel_normal_center_field(frame)` returning the centre's own field: a Bezier root, a selected fiber's retained parameter, or a projective scalar's denominator field extended by the speed radical.
- The selected-fiber kernels then need porting to the recursive line-kernel form. chord_kernel already owns the smaller exact construction, so start there.
- In the measured stall the centre was already a degree-214 Bezier parameter, so group D alone will not speed it up; see section 2.

## 5. S5 and S7

**S5: one comparison ladder.** Embed, then join base fields, then return `Uncertain`, with no global promotion.

**S7: collapse duplicated cusp chart variants.** The duplicated variants are `SelectedFiberRational` and `SelectedFiberParallel`, and the corresponding `*ParameterMap2` pairs: about 106 references, concentrated in `cusp_semicircle/parameters.rs`. This is structure and code size, with no performance effect.

## 6. Public geometry for restricted boolean fragments

Boolean results can carry Bezier and conic fragments that cut their source at algebraic, non-rational parameters. Such a fragment is a `CurveCarrier2::Restricted` or `CurveCarrier2::SourceRange` carrier, and the public API cannot describe it:
- `Curve2::geometry()` returns `None`.
- `Curve2::native_bezier_fragments()` returns `Blocked(ExactCurveBlocker { operation: NativeTopology, family: Some(QuadraticBezier), reason: Unsupported, .. })`.
- `source_range()` and `retained_fragment()`, which hold the source curve and its exact parameter interval, are `pub(crate)`.

**Observed failure, now repaired in the demo.** Two `hypercurve_ui` tests previously failed with "the demo cannot yet edit a QuadraticBezier boolean fragment" (likewise for `CubicBezier` and `RationalQuadraticBezier`):
- `scenes::tests::multi_boolean_defaults_resolve_all_boolean_modes`;
- `scenes::tests::shared_polyline_boolean_state_resolves_all_boolean_modes`.

The [retained demo-result record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hypercurve-demo-retained-results/manifest.json)
repairs both using the existing `project_to_finite_curve_paths` display boundary,
while retaining the original exact region and paths for subsequent operations.
The original three full scene pairs decide all four Boolean modes; seven formerly
blocked operation rows now succeed, and all five prior successful preview byte
streams are unchanged. All 42 demo tests, strict Clippy, formatting, and native
and WASM release builds pass. This does not provide general exact control-point
access for these carriers. Finite JSON/URL editor serialization and legacy
sampled/Geo adapters remain separate boundaries requiring further work.

**Options:**
- A public accessor for the source curve plus its exact parameter interval as `CurveParameter2` endpoints. The consumer subdivides itself, or approximates for display.
- Promotion of a restricted fragment to a native Bezier whose control points are exact algebraic `Real`s, so that `native_bezier_fragments` succeeds.
- An explicit display-only approximation, named as such, that cannot re-enter exact operations.

The first keeps the exact-only principal API smallest. The second makes the existing accessor total for these carriers.

## 7. Concurrent contacts across independent radicals

**Resolved for one shared circle and one shared line (2026-10-22).** hypercircuit's Easyduino `stm32f103` board (`benches/easyduino_full_pipeline.rs`, `EASYDUINO_BOARD=stm32f103`) failed in the copper-layer union with "exact Construction for RationalQuadraticBezier was blocked by RealSign". The failure reproduced at `282a3eac52` and now completes; `PERFORMANCE.md` records the runs.

**Where.** `CurveRegionBooleanContext::build_split_topology` found two contacts on the corner arc of pad U2:8, a circular conic whose center carries the radicals of two 135-degree rotations. The first is with a 45-degree route edge `A`. The zero-clearance GND zone is cut by that route, so its boundary keeps a fragment `B` of the same edge, cut from the same support-line allocation. The second contact is with `B`, at the same point. Both are exact `Point2` values with their own square roots, and `B`'s ends are line-circle crossings, so `same_point` could not prove them equal and the pair stayed in `deferred_contact_matches`.

**Fix.** `contacts_decided_same_from_shared_circle_and_line` applies when both contacts share one circle (one carrier or one retained circle) and one line (one carrier or one support allocation, `LineSeg2::shares_support_line`). The line meets the circle in at most the two points `M ± τu`, where `M` is the projection of the center `C`, so the sign of `(P - C)·u` selects the point: equal signs prove equality and different signs prove distinctness. Both signs are exact predicates. `tests/hypercurve_pcb_boolean_regressions.rs` keeps the minimized board case, a 3-4-5 rotation and a reflection, crossings on both sides of the projection, near-tangent crossings 2.4e-5 apart and an exact tangency.

**Already fixed on this board's path (commit 5a4e8aebd).**
- Reversing a straight quadratic fragment kept its retained line support.
- A line-circle relation deflated an endpoint certified on the circle, even when it is not an arc endpoint.

**Still open.**
- Shared supports are recognized by allocation identity. If the route and the zone are transformed separately after the cut, their supports are separate allocations and the case is undecided again; the regressions transform the inputs before the cut.
- The earlier record listed six cases on this board, four with nonparallel `A` and `B` crossing on the arc. They do not occur on the current board. If such a case recurs: `A` and `B` meet in one point `Q` in their own field. The contacts are equal exactly when `Q` lies on the circle, an exact zero test of `|Q - C|² - r²`, and the branch sign above then selects `Q` on each line.
- The earlier options remain for such cases: line-conic contacts on a circular conic in tower-representable form, from the circle's exact center and radius as `line_circle_relation_at_endpoints` does for native arcs; `quadratic_tower` beyond one outer radical; or a zero-separation bound, which `algebraic_separation_bound_bits` does not give past 256 nodes or at `Inverse` nodes without an exact child sign.

## 8. Polynomial-branch sections and filled arrangements

**Admitted.** An unmapped graph branch meets an authored line with rational
endpoints, or a returned restriction of one, in both intersection APIs and
both operand orders, including tangencies; both operands split and the
pieces re-enter the same query (`PERFORMANCE.md`, 2026-10-22). A branch's
other intersections, its self-intersections and point location still report
typed `Unsupported` blockers.

**Open, in dependency order:**
- **Split cost.** Splitting a branch at `n` cuts isolates each cut fiber twice,
  once per adjacent piece, although the contact already holds the point. The
  vertical quintic topology costs about three times the section alone.
  Computing each cut point once in the branch split, or passing the contact
  points to it, would remove those isolations. This is a cost, not a defect.
- **Line components.** When the line is a component of the zero set, the
  substituted polynomial vanishes identically and the pair reports
  `Unsupported`, for every branch of that zero set. Admitting it needs overlap
  evidence on the branch that lies on the line and the finite contacts of the
  others.
- **Horizontal-line replay.** On a horizontal line the line parameter is the
  branch abscissa's own root, so a returned line piece's endpoint is a line
  point that is proved equal to, not stored as, the branch point. A
  selected-fiber image of the abscissa would keep the replay identity at the
  cost of fiber-field arithmetic.
- **Selected abscissas and mapped branches (H06, milestones 2 and 3).** An
  irrational `c`, algebraic line coefficients, and branches with a similarity
  or rational image need the query pulled back through the image map;
  source-x monotonicity is not world-x monotonicity. Branch pairs need
  resultant events and exact fiber signs.
- **Repeated and vertical components (H07).** See `hypersolve/FUTURE_WORK.md`.
- **Filled regions and BRep trims (H08).** `CurveRegion2` construction and
  `CurvePath2::simple_loop_orientation` with polynomial boundary pieces return
  `Blocked(Construction, ImplicitPolynomial, Unsupported)`; a test records
  this. Source-span and boundary ownership, pair incidence, point location,
  local sectors and BRep consumers must all retain the graph, selector and
  domain. The chart-qualified views and shared incidence of H01–H04 come
  first.

## 9. Selected-root comparison, fillet centres and tangent events (2026-10-23)

Items H09–H12 of the 2026-10-22 research queue, rechecked at `916e1e5f41`. `PERFORMANCE.md` records the measurements.

**Done: overlapping local polynomial roots (H10).** `cmp_by_refinement` orders two recursive roots whose isolators still overlap after 32 bisection steps by one sign of either defining polynomial at the other root (`order_against_polynomial_root`). Before, the 64 such comparisons in the release suite all ran 512 more bisection steps and then promoted both roots to global parameters, and a pair 2^-603 apart stayed undecided. A recursive root also keeps native algebraic roots it has been proved equal to. Together these let `chamfered_quadratic_bevel_stroke_round_trips` complete in about a minute; it did not finish in forty.

**Open: projective-scalar against polynomial roots.** The 12 remaining comparisons of a formula root with a polynomial root (10 `JP`, 2 `PJ` in the release suite) still take the 512-step route, about 0.1 s each. The certificate declines in both directions: a formula root has no defining polynomial to sign at the other root, and the polynomial root's relation signed at the formula root does not decide in the bounded pass. Homogeneous evaluation of that relation at `n / d` over a joined field (`recursive_projective_polynomial_value` lifts only ancestor coefficients) is the likely extension; check which step declines first.

**Open: equality with a high-degree eliminant root.** The remaining time of the chamfered case is one 47.6 s proof that a degree-6 local root over a two-source, one-extension tower equals a root of the degree-192 polynomial the chord/parallel system projects for its candidates (`parameter_is_in_ordered_range`). The gcd starts with a pseudo-remainder of degree 192 by degree 6 over the tower. Promoting the local root first and comparing the two native roots is no faster (62–82 s per equality; rejected). A modular gcd (H14), or a candidate filter that does not first project the system to degree 192, is the next step.

**Not implemented: two-sided field joins (H09).** `recursive_polynomial_sign_joined` returns `Unsupported` when a query's coefficient field has a selected source that the root's base lacks; there is no fallback inside it. The internal regression (`t^2 = 2 + sqrt 2` in the root's field; `beta = sqrt 3` in an independently built field, in both source orders) reproduces this and is kept as the ignored test `recursive_polynomial_root_signs_queries_over_an_independent_source`. A join over the union of both source tuples, factored from `recursive_merge_projective_point_fields`, decides all of its queries locally. With route tracing over the full release suite and the four open-path workflows, the only queries that reached the missing-source case came from that test, so the join was not kept. Implement it when a supported operation reaches the case.

**Not the cost: centre promotion in the selected parallel-normal kernel (H11).** In `round_stroke_of_a_filleted_nurbs_pair_completes`, the fillet centre reaching `selected_parallel_normal_parallel_intersections` is already a `BezierParameter2` of degree 214, so its promotion there is a no-op. Of five calls, four took 0.35 s, 12.1 s, 0.36 s and 110 s, and the fifth had not finished at the 40-minute limit; the time is in selected-fiber isolation over the degree-214 field (`LocalAlgebraicField::reduce`, local subresultants and Sturm sequences). Widening the consumer to `CurveParameter2` does not change this; authoring the centre as a low-degree selected fiber does (section 2), after its selected-factor degree is established (H15).

**Open: tangent events (H12).**
- `refilleting_a_filleted_vertex_completes`: the second fillet signs point-incidence predicates at recursive roots through `dense_two_positive_square_root_sum_sign` (section 1). It did not finish in forty minutes.
- `round_stroke_boundary_of_a_filleted_arc_corner_refillets_and_erodes`: fillet, round stroke, boundary reconstruction, reversal and a second fillet all complete; eroding the result then blocks with `RealSign`. At a tangent junction left by the second fillet, one offset span keeps an analytic parallel's retained tangent and the other the fillet arc's represented tangent. Their cross product is zero, but neither it nor the equality of the two offset ends is decided. The fillet certified that tangency when it was built; the rebuilt region does not keep it. The same workflow from a line/line corner completes (`round_stroke_boundary_of_a_filleted_line_corner_refillets_and_erodes`). Either the boundary keeps the tangency certificate or `curve_tangent_cross_sign` proves this zero exactly.

## 10. Box containment and CAM-derived contracts

**Admitted.** `Curve2::box_containment` and `CurvePath2::box_containment`
decide whether a curve image lies in a closed axis-aligned box, with an exact
witness point when it does not (`PERFORMANCE.md`, 2026-10-23). The cached
`bounds()` enclosure is the fast path; unsettled sides use the exact extremum
candidates of lines, arcs, polynomial quadratic and cubic Béziers, rational
quadratics and spline spans of those kinds.

**Open:**
- **General rational spans.** A rational span of degree three or more, or a
  polynomial spline span of degree four or more, contributes only when its
  coordinate is certified monotone; otherwise a crossing enclosure reports
  `Unsupported`. `RationalBezier2::axis_monotonicity_classified` already
  isolates the quotient-derivative roots. Deciding the coordinate at each
  isolated root against the bound would complete these spans, including
  rational cubic NURBS.
- **Retained, source-range and implicit-polynomial carriers** contribute only
  their endpoints. A source-range carrier could reuse the derivative roots
  that `retained_algebraic_source_extrema_bounds` certifies inside its
  parameter range.
- **Witness parameter.** The witness is a point. An arc's native fragment
  parameter is not its curve chart, so a curve parameter is not published.

**Rejected: hull subdivision as the refinement.** Halving until each piece's
control hull lies in the box decides the 9/4 fixture after one split, but a
bound `2^-k` above an irrational peak needs about `k` levels (201 hull tests
and 10.7 ms for `k = 200`, against 42 µs for the derivative roots), and a
tangent bound at an irrational parameter is never decided, because a hull
exceeds the curve wherever the curve's maximum is interior. Derivative roots
decide both exactly.

**Rechecked, no change (C07 fixtures 2 to 4, Alumina `ef94d9e`).**
- *Set distance and same-parameter distance.* `BezierFlatteningCertificate`
  bounds the distance between the curve and its chords as point sets: each
  control lies in the capsule around its chord, so every curve point is
  within the bound of the chord and, by continuity of the projection on the
  chord, every chord point is within the bound of the curve. It is not a
  same-parameter bound: `C(t) = (t^3, 0)` flattens to one chord at any bound,
  while `L(t) = (t, 0)` is 3/8 away at `t = 1/2` (a test records this). No
  API names it as one, and the offset verifiers name their same-parameter
  bounds explicitly, so there is no conflation to fix. The polyline does not
  publish each chord's source parameter interval; a consumer that needs a
  pointwise certificate builds it from the control net, as Alumina does.
- *Fidelity across split and copy.* `Curve2` lineage keeps source parameter
  identity through split, reversal and transform, but there is no fidelity
  classification on curves. `CertifiedCurvePathParallel2` keeps only aggregate
  exact and approximated counts: after `path()` or `into_path()` a split or
  copied path cannot tell exact parallel curves from verified approximations.
  A per-output-curve source index and fidelity record would preserve it.
  Alumina's moving-join policy uses its own span classification, so this is
  recorded, not changed.
- *Step quantization.* No Hypercurve or Hyperpath API rounds to a machine
  lattice or writes commands back into source geometry. Alumina rounds with
  `Real::round_certified` and keeps source and canonical-command travel checks
  separate; `8 * 1600/3 = 12800/3` rounds to 4267 steps, `8.000625` mm.

## 11. Resource experiments H16–H20 (2026-10-23)

Items H16–H20 of the 2026-10-22 research queue, rechecked at `d3d5e6ebef`. `PERFORMANCE.md` records the measurements.

**Done: deferred Bezier carrier preparation (H18).** The pair builders prepare a Bezier carrier's top-level curve only when a Bezier-Bezier pair that survives the broad phase, or the carrier's self context, reads it. Before, a union of a board's copper aggregate with one feature prepared every carrier of the aggregate: 99% of the 256,000–554,000 cross-operand preparations per Easyduino board were never read, almost all of them in the CAM round-trip audit. Allocation calls fall 0.7–2.7% and the peak of live bytes up to 5.6% on the six boards, and nano's CAM audit runs 6.1% fewer instructions; findings are unchanged.

## Fast paths added in this round

Keep these in mind when measuring:
- **Hypersolve:**
  - irreducible recursive towers (generator reuse);
  - primitive local-field remainders;
  - two-root sign by Bernstein exclusion, with a rational-norm and modular zero/nonzero certificate run before the basis change;
  - lazy centred coefficients in `algebraic_root_affine_relation`;
  - sign-change refinement for admitted roots.
- **Hypercurve:**
  - fuzzy bracket split;
  - bounded sign-change ordering;
  - exact Horner enclosure before Bernstein restriction;
  - filter-side Sturm exclusion when the defining polynomial dominates;
  - modular-first filter signs from defining degree 16 (`MODULAR_FIRST_DEFINING_DEGREE`). Below that degree the Sturm–Tarski chain is cheaper; a version without the gate regressed two passing cases by about 25%.
  - rational fillet line-offset contacts.

## Native circular endpoint projection follow-up (2026-10-09)

The [retained circular-area record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-09-hypercurve-retained-circular-area/manifest.json) certifies the original full
CPArti A5/A6 overlap and adds shared native arc lowering for retained endpoints.
The narrow quadratic view strictly replays its selected root and does not enter
the general scalar-witness cache. General implicit projection was rejected after
chord/offset field regressions; the failing sources and logs remain available.
Similarity transforms also preserve original retained carriers after native-view
projection, with the existing round-offset regression extended to cold/warm views.
That checkpoint retained two restricted-fragment demo editability failures; an
isolated committed baseline reproduced both with identical diagnostics. The
subsequent demo-result record above repairs those gates. This does not close
sections 1–7, public
control-point access for general restricted fragments, retained higher moments,
exterior traversal, or the ignored compositions.

## Downstream workload findings (2026-10-22)

Found by `tests/downstream_profiles.rs` (repository root), which migrates
SynapsCAD and exactcore-hyper-comparison workloads. Both are typed or
documented outcomes, not wrong answers.

**Eroded polygons with normalized edge offsets.** `circle(15, 40)` from
csgrs, offset by -1 with round corners (the inner half of SynapsCAD's
`offset.scad` wall), is the regular 40-gon with every edge moved inward by 1.
Its area agrees with `4500 sin(pi/20) (1 - sec(pi/40)/15)^2` to 10^-30, and
every probe point away from its boundary classifies, but `bounds()` and the
classification of its own vertex `(15 - sec(pi/40), 0)` end in
`ExactCurveBlocker { reason: Ordering }`. The vertices are intersections of
lines offset along unit normals, so they carry the square roots of the
squared edge lengths `450 (1 - cos(pi/20))`. Coordinates equal by symmetry,
or equal to an independently built `15 - sec(pi/40)`, are then not proved
equal. This is the pattern of section 2 (line offsets by a unit normal):
recognizing `sqrt(900 sin^2(pi/40)) = 30 sin(pi/40)`, or offsetting along an
unnormalized normal, would keep the coordinates in Q(zeta_80). The erosion
itself takes 12.9 s in a debug build; the outward offset takes 54 ms.

**Bounds of regions with algebraic cut fragments are envelopes.** SynapsCAD's
quadratic Bezier region `{x^2 <= y <= 4}` minus `[-3, 3] x [2, 5]` is
`{x^2 <= y <= 2}`, with corners `(+-sqrt 2, 2)`. Its area, membership and
transformed copies are exact, but `bounds()` reports x in `[-3/2, 3/2]`:
each cut fragment ends at an algebraic parameter, and its envelope is the
exact bound of the source curve over a rational parameter interval around
that parameter (`bezier_retained_measure.rs`). The documented contract is an
enclosure, so this is not wrong, but a consumer that centres a 2D region by
its bounds, as OpenSCAD's `center = true` does, uses the envelope's midpoint,
which for an asymmetric cut is not the set's. Tight bounds need the cut
endpoints' exact coordinates; refining the parameter interval only narrows
the envelope.
