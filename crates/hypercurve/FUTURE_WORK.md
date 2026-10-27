# Future work: exact-composition performance and S5–S7 consolidation

This file records the remaining directions from the 2026 API and machinery review: what is still open, the evidence behind each direction, and approaches that were measured and rejected, so they are not repeated.

The detailed chronological log, including commit hashes, timings and samples, is `exactcore-hyper-comparison/audits/hypercurve-api-review-2026-09-12/implementation.md`. The original review is `review.md` beside it.

## Current state

The initial [executable capability matrix](qualification/README.md) ties named
public fixtures to separate operation stages and asserted outcomes. Its bounded
catalog distinguishes exact successes from typed gaps and inherited qualification;
unspecified stages and the complete algebraic target remain open.

The initial M0 increment qualifies 18 public fixtures and 16 evidence-guard
regressions against `fa312fd849` plus the catalog/driver patch. Both the direct
invocation and the new CI commands pass with stable source and binary digests.
Formatting, typos, repository budget and `zizmor --pedantic --strict-collection .`
pass. Zizmor used its default offline mode; online-only audits were unavailable
without an API token. Hosted artifact upload is inspected, not locally executed.
No Rust kernel or geometry fixture changed, and the full pre-push matrix for
this increment is pending. The scoped report, source patch and raw checks are
in `docs/evidence/2026-10-26-hypercurve-capability-matrix`, with its digest in
`docs/evidence-archive.tsv`.

The exact-only principal API, the `CurveRegion2` migration, and S1 (`SelectedScalar2` dispatch) are complete. S6 is partly done; the remaining sites are listed below.

The historical ignored composition harness (`tests/hypercurve_composed_workloads.rs` and `tests/hypercurve_open_path_compositions.rs`) records 10 of 24 cases completing under a 300 s budget:

| Case | Time |
| --- | --- |
| `chamfered_quadratic_bevel_stroke_round_trips` | 15 s (section 9) |
| `repeated_inward_miter` | 21 s |
| `beveled_rational_seed` | 56 s |
| `weighted_conic` | 61 s |
| `booleans_then_double` | 76 s |
| `bevel_then_round` | 107 s |
| `mixed_family` | 123 s |
| `mixed_cubic_arc` | 203 s |
| `nested_xor` | 290 s |
| `refilleting_a_filleted_vertex_completes` | 163–296 s (section 1) |

Only the first and last rows were remeasured on 2026-10-23. The other 14 were recorded beyond that budget; this historical table has not been fully rerun. The hyperbola-pair stroke remains a recorded stall. The separate `round_stroke_boundary_of_a_filleted_arc_corner_refillets_and_erodes` now completes in 13.64 s and runs as an ordinary test; the M1 native-parallel-pair record below gives its scope and evidence.

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

This was the original hypothesis, not a prerequisite for further work. H13
below disproved it for the captured refilleting leaves. A different residual
stall must first supply its actual coefficients, selected sources and cost
trace (R16 of the 25 October handoff) before another modular experiment.

**Rechecked for `refilleting` (H13, 2026-10-23).** That case's leaves do not
match the design's premise. After affine source collapse each leaf is a
univariate polynomial of degree 27 in one selected source (coefficients of
1,400 to 64,000 bits), the existing modular coprimality certificate already
proves it nonzero, and its value is large but cancels 340–2,080 bits against
its coefficients. The cost was the certified refinement after that
certificate: exact rational interval Horner and Bernstein enclosures,
normalizing a wide fraction at every operation. Both now run over integer
common denominators, with the same decisions, and the case completes in three
to five minutes (`PERFORMANCE.md`). A modular norm before expansion would not
have changed it, so none was built. The lazy-leaf design above is untested on
the multi-source leaves of `filleted_union`, `wide_weighted` and the chamfered
strokes; capture their leaves before implementing it.

## 2. Exact representation of generated geometry

**Fixed.** A fillet against a line used the unit direction d/|d|, so √|d|² leaked into the fillet contact's defining polynomial as lazy `Real` DAGs. The fix (cb372d03) builds the rational norm of the offset-line incidence, R² − sd²|d|²X₀², and filters candidates to the authored factor.

**Look for the same pattern elsewhere**: any generated object whose `Real` coordinates come from `sqrt` or division of algebraic projections. A strong symptom is that `format!("{:?}", real)` on a coefficient takes seconds. Candidates:
- line offsets in stroke, chamfer and bevel construction (`curve_fillet_centers.rs` `FilletOffsetCarrier2::Line` offset, `line_unit_direction`);
- concentric-arc frames;
- any `LineSeg2::translated` by a unit normal.

**Degree-214 fillet centre in `round_stroke_of_a_filleted_nurbs_pair`.**
- The fillet centre's parameter (`anchor_evidence.center_parallel`, built in `curve_fillet_centers.rs` and reconstructed through `curve_corner_chain.rs` `RetainedFilletRadialFrame2::ParallelNormal`) is a degree-214 global algebraic root of the offset-offset intersection resultant.
- It is square-free but not irreducible (H15, 2026-10-23). Over Q it factors as degrees 2, 8, 40 and 164, and the selected root belongs to the degree-164 factor, which is irreducible (`PERFORMANCE.md` gives the certificate). The intrinsic degree of the centre is 164.
- Every later kernel (`selected_parallel_normal_parallel_intersections` → selected-fiber isolation over a degree-214 field) pays for it.
- Substituting the degree-164 factor, at the kernel or at the resultant, did not complete the round stroke and made three of its first four kernel calls slower; the stack has no factorization over Q to obtain the factor cheaply. The longest call now spends its time in the bivariate subresultant pseudo-remainders of `local_sturm_sequence`, after local-field reduction moved to integer arithmetic.
- **Direction:** keep that centre as a selected fiber over the other offset's parameter, with low-degree field arithmetic, instead of publishing a global `BezierParameter2::Algebraic`. This means changing how fillet centres are authored, not adding a local fast path. A selected fiber does not lower the degree-164 intrinsic degree; it can only avoid the 50 extraneous degrees and the global arithmetic.

## 3. Selected-fiber common-root counting

**Reproduced and captured (R16, 2026-10-25).** The two original STRICT workflows
reach different selected fields. Their uninstrumented baselines both exceed
external 300-second / 8-GiB containment; that is neither typed exhaustion nor
proof that the mathematical operation cannot complete.

- `filleted_boolean_miter_erosion_completes` supplies degree-36 incidence and
  degree-40 predicate polynomials over an irreducible degree-14 rational base.
  Independent exact division proves that the predicate is the incidence times
  a quartic. Divisibility and coprime cofactors establish the maximal GCD of
  degree 36. The diagnostic GCD finishes in about 26 ms; its following Sturm
  sequence remains unfinished at the external limit. Faster GCD construction
  does not address this captured stall.
- `stroke_fillet_stroke_of_a_cubic_arc_path_completes` supplies degrees 4 and
  40 over a degree-10 carrier with coefficients in `Q(sqrt(89))`. Its selected
  root has irreducible rational degree 20. An independent positive-embedding
  proof and exact division establish a maximal degree-4 GCD, with the
  predicate again a multiple of incidence. The general coefficient-field GCD
  takes about 192 seconds in the diagnostic run. These polynomial facts do
  not establish geometric contact or replace the callers' unsquared guards.

The existing rational-function reduction API already proves the first identity.
A private consumer tried that complete-quotient zero proof before common-root
counting. Both its broad and rational-only variants complete the first original
workflow, but neither completes the second within 300 seconds. The broad pass
spends about 200 diagnostic seconds before returning to the original count.
The rational-only variant preserves exact predicate outcomes but fails the
protected timing gate with eight simultaneous adverse intervals in the
controlled same-checkout comparison. Neither
runtime change is retained; the [cost record](PERFORMANCE.md#selected-fiber-quotient-rejection-r16-2026-10-25)
preserves their gains, regressions and limits. Eight independent predicate
tests accompany the captured evidence.

**Measured negative.** Sending large fibers to the reducing Euclidean chain, instead of `rational_local_subresultant_rows`, only moved the cost into `normalize_local_polynomial` field divisions.

Further work needs the actual coefficient field, selected-root identity and
cost, followed by unchanged whole-workflow and protected-control gates. A
modular/CRT strategy has not been measured on the recovered nonrational case.
Avoiding repeated zero/root-count proofs, or a cheaper certified count of the
GCD's roots, remain experiments. Multi-source `filleted_union`/`wide_weighted`
leaves, complete Pell-denominator outcomes and internal work/memory/cancellation
bounds remain open.

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

**Independent transforms, M1 increment (2026-10-26).** The retained line owner
now recognizes equal exact support endpoints, including reversal, across
independent allocations. Quadratic and rational Bézier line images map their
retained supports instead of rebuilding them from trimmed endpoints. Region
affine transforms use the same owning implementation. This repairs evidence
transport for algebraically trimmed lines under similarities and general
affine maps; it does not establish full composition closure.

The acceptance witnesses failed on `f612fbfe13` and pass after this change:
`support_line_identity_survives_independent_maps_and_reversal` and
`transforming_a_straight_fragment_keeps_its_exact_line_support`. They check exact
support direction, algebraic endpoints, both traversals, independent maps,
reflection, a nonsimilarity, and distinct or undecided nearby supports. The
owning layer is Hypercurve's line support and transform machinery; the affected
contracts are P12, P17–P19 and closure-plan M1. No tolerance or scalar fallback
was added.

Final local checks passed: formatting, Hypercurve Clippy with all features and
targets and `-D warnings`, the three focused unit tests, 22 rational Bézier
tests, 15 reconstruction tests, and 19 adapter/downstream/mixed-profile tests.
The earlier downstream run also passed 24 exact-profile-sweep tests before
the rational-line transform follow-up. The public PCB suite has nine prior
successes and one new composition failure: a separately transformed cut can
enter the compound region, but its next strict XOR is blocked by
`Intersection/Line/Ordering`. The full all-features run started before the
rational follow-up subsequently finished with 2,635 passes, this one failure,
and 35 existing ignored tests. These are
correctness observations, not performance or completed-closure claims.
The reproduction patch and command logs are retained in evidence record
`docs/evidence/2026-10-26-hypercurve-closure-m1`, with its digest in
`docs/evidence-archive.tsv`.

**Circle transforms, M1 follow-up (2026-10-26).** The complete independently
transformed PCB chain now succeeds, including reconstruction in three operand
orders and the next strict XOR against the corresponding union. Similarity
transforms had discarded retained circle definitions, forcing later contacts
through a general conic calculation with wider endpoint expressions. The
rational quadratic and general rational owners now transport the circle,
tangent contacts, source parameter frame and restriction, and certified
injectivity. Region transforms delegate to those owners. General affine maps
still invalidate circular metric evidence. No sign limit or policy changed.

The domain is retained circular rational spans, including algebraic centers,
trims, degree elevation, negative homogeneous weights, reversal, and certified
similarities; the properties are P12 and P17–P19. The missing-circle witness
failed on `e002496280`. Independent circle parametrization and exact radius,
tangency and parameter relations now pass, alongside near-distinct, repeated
visit and pole controls. Final local checks passed: formatting, strict
all-target/all-feature Hypercurve Clippy, six circle-incidence tests, all ten
PCB tests, and 43 adapter/profile/sweep/mixed-modeling tests. The final
full-suite and pre-push qualification is recorded below; no performance
improvement is claimed. The evidence record is
`docs/evidence/2026-10-26-hypercurve-closure-m1-circle`.

The subsequent full unit run found one storage-dependent assertion in the
original CPArti area fixture: preserved circles now permit native output. The
unchanged original geometry still has exact area. Its qualification now also
constructs an explicit retained boundary, checks the original area bounds for
both views, and requires each reconstructed contour's next XOR to be empty.
All four retained-circular tests, strict Hypercurve Clippy and formatting pass;
the complete suite subsequently passed. Production code is unchanged by this
test extension. Evidence: `docs/evidence/2026-10-26-hypercurve-closure-native-area`.

**Native circle reconstruction, M1 increment (2026-10-26).** Native arcs now
retain the existing shared circular support certificate through promotion,
restriction, corner reconstruction, reversal and rational decomposition.
Similarities use the circle owner's proof transport. The owning layer is
Hypercurve's native circular support and reconstruction machinery; the affected
contracts are P12 and P17–P19.

The new acceptance witness fails on `74fd8fc89c` after asserting that its input
has both contact proofs: native promotion discards them. The repaired path
preserves line and parallel contacts for an algebraic center, negative weights,
quadratic and degree-elevated charts, restricted contact membership, both
traversals, rotation and reflection. All seven circle-incidence tests, strict
all-target/all-feature Clippy and formatting pass. The complete release suite
passes 2,639 tests (35 existing ignored), and all 43 downstream
adapter/profile/sweep/mixed-model tests pass.
The original arc-corner erosion witness still reports `Offset/Line/RealSign`.
This increment establishes neither completed closure nor a performance gain.
Evidence: `docs/evidence/2026-10-26-hypercurve-closure-native-circle`.

**Pre-push qualification (2026-10-26).** Code `4b5bcdde50` completes all 129
locally executable workflow steps and all 196 required strict Clippy
configurations, including feature combinations, stable/MSRV packaged consumers,
coverage, full sanitizer campaigns, examples, fuzz-target checks and release
WASM demos. Per-command records verify unchanged source and dependency inputs;
failed sandbox/offline attempts and cancelled duplicates are retained separately
from completed replacements. Four hosted configuration/upload/deployment
actions were inspected and excluded from local execution. No CI configuration
changed. The concurrent benchmark runs establish no performance improvement.
Evidence: `docs/evidence/2026-10-26-hypercurve-closure-validation`.
This qualifies the committed increments; the remaining closure gaps stay open.

**Scoped capability matrix at `4b5bcdde50`.** Each cell describes only the
named witness below. A dash means this increment does not qualify that role,
not that the target contract excludes it. The target remains the complete
algebraic domain in `AGENTS.md`; arbitrary-degree closure, total resource
accounting and fresh-process replay are not established by these rows.

| Witness | Representation | Evaluation/differentials | Incidence/order | Region admission | Booleans | Further offset | Corner edits | Reconstruction | Exact replay |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| L: algebraically trimmed lines under independent affine maps | Support retained | — | Exact shared support; distinct controls | — | — | — | — | — | — |
| C: rational circle charts under similarities | Circle and selected domain retained | Independent point oracle | Exact inverse parameters; poles and repeated visits retained | — | — | — | — | — | — |
| N: native circle contact transport | Shared circle certificate | — | Line/parallel contact membership | — | — | — | — | Restrict, reverse, decompose | — |
| P: independently mapped PCB cut | Mixed retained supports | — | Circle/line contacts | Certified | Difference, union, XOR | — | — | Next XOR empty | — |
| A: original CPArti overlap | Native and retained views | Exact area bounds | — | Certified | Intersection, next XOR | — | — | Next XOR empty | — |
| O: original arc-corner composition | Generated mixed boundary | — | Earlier contacts complete | Certified before erosion | Rebuild XOR and fillet set differences | Blocked: Offset/Line/RealSign | Both fillets complete | Reversal and rebuild complete | — |

Witnesses: L is `support_line_identity_survives_independent_maps_and_reversal`
and `transforming_a_straight_fragment_keeps_its_exact_line_support`
in `src/bezier.rs`; C is
`similarity_preserves_retained_circle_and_selected_chart` and
`circle_incidence_preserves_subranges_reversal_elevation_and_certainty`;
N is `native_arc_reconstruction_preserves_circle_tangent_contacts` in
`src/rational_bezier_general/circle_incidence_tests.rs`. P is
`concurrent_circle_contacts_survive_independent_maps_after_cutting` in
`tests/hypercurve_pcb_boolean_regressions.rs`;
A is `original_cparti_pad_overlap_has_an_exact_area`; O is
`round_stroke_boundary_of_a_filleted_arc_corner_refillets_and_erodes`.
O remains ignored and was executed explicitly to reproduce its blocker;
its partial stages are not counted as completed erosion.

**Fillet contact and circular incidence evidence, M1 increment (2026-10-26).**
The unchanged arc-corner chain on `4b5bcdde50` / `4ace9a71a2` loses a represented
fillet's source contacts before erosion. Publication now retains each admitted
scalar contact parameter, its owning source chart and exact point. Noncollapsed
concentric offsets share the original circle certificate and compose their
scales. Tangent parallelism is kept separate from incidence: a shifted circle
is incident to the corresponding shifted parallel only after the original
normal-side equation and distance equality are certified. Similarities transport
these relations; changed source, parameter and point controls remain distinct.

The owning layers are Hypercurve's fillet publication, supporting-circle
certificates, normal-parallel incidence and arrangement contact handoff
(P12, P15, P17–P20). The optional circular incidence path reuses the existing
circle relation, parameter inverse and exact concentric construction for a
certified quadratic chart with active domain inside its pole-free unit interval.
It preserves the represented contact point through the next conic inverse.
Extensions, retracing, collapse, coincidence and unrecognized charts retain the
general path. No tolerance, policy limit, public API or archive format changed.

Independent witnesses cover algebraic centers, degree elevation, negative
weights, reversal, negative normal sheets, exact tangency and nearby distinct
circles. They check exact points, original parameters and oriented crossings.
The selected trim witness completes with an explicit 4,096-refinement /
128-promotion budget. A distinct rational tangent against an isolated algebraic
trim exhausts a zero refinement budget, then retries successfully with a larger
budget. These counters do not bound total work, expression growth or heap bytes.
Selected non-scalar fillet contacts and fresh-process certificate replay remain
outside this increment's demonstrated coverage.

Final checks passed: `cargo fmt --all -- --check`; MSRV Hypercurve Clippy with
`--all-targets --all-features -- -D warnings`; the complete release all-features
suite (72 suites, 2,645 passed, 35 existing ignored); and all 43 downstream
adapter/profile/sweep/mixed-model tests. The earlier full retry found one stale
tangent-frontier assertion: represented circular tangents now decide exact
answers. Its geometry is unchanged and the assertion now checks independent
cross/dot expectations. The initial truncated full run, diagnostic storage
exhaustion, read-only ccache startup failure and intermediate failed attempts
are retained separately from successful checks.

The original full arc-corner witness still fails at
`Offset/RationalQuadraticBezier/Predicate` during erosion. Its 12.50-second
failed probe is not a completed-workload cost or speedup claim. This source
increment has not yet completed the full repository pre-push matrix.
Evidence: `docs/evidence/2026-10-26-hypercurve-closure-fillet-contacts`, with
its digest in `docs/evidence-archive.tsv`.

| Scoped witness | Representation | Evaluation/differentials | Incidence/order | Region admission | Booleans | Further offset | Corner edits | Reconstruction | Exact replay |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| T: concentric contact transport | Shared source and composed scale | Exact tangent direction and opposition | Changed-distance incidence checked separately | — | — | Noncollapsed native circle certificate retained | — | Similarity and reflection | — |
| I: circular-source parallel incidence | Exact quadratic chart, degrees 2/5 | Independent circle point oracle | Exact parameters, crossings, trim and retry | — | — | Signed distances through normal reversal | — | Reversal and degree elevation | — |
| O: unchanged original arc-corner chain | Generated mixed boundary | — | Earlier circle/parallel contacts complete | Certified before erosion | Rebuild XOR and fillet set differences | Blocked: Offset/RationalQuadraticBezier/Predicate | Both fillets complete | Reversal and rebuild complete | — |

T is `concentric_circle_contacts_keep_directions_without_false_incidence`;
I comprises the five tests in `src/bezier_offset/parallel/circular_incidence_tests.rs`.
O is the unchanged full witness named above. These rows extend demonstrated
coverage only; the algebraic target contract and other matrix rows remain open.

**Offset band endpoint transport, M1 increment (2026-10-26).** At `532e517b07`,
separate endpoint representations at a certified smooth line/circle junction
can leave a band connector's circular incidence undecided. Band assembly now
shares the represented circle endpoint only after the existing strict tangent
junction proof establishes equality. Parallel source tangents at a common
vertex give equal normals or opposite normals; certified coordinate enclosures
exclude the latter's distance of `2|d|`. This is an exact proof, with no tolerance
merge. Original support fragments, charts and traversal remain authoritative.
The owner is Hypercurve offset band assembly (P12, P15, P17–P19); the domain is
connected certified source loops, rational nonzero offsets and represented
circular endpoints. Unproved joins retain their existing behavior.

The independent witness
`offset_band_endpoints_share_only_certified_smooth_circle_contacts` exercises
smooth and opposed line/quarter-circle junctions at distances `±1/4` and
`±1/10^9`. It checks the exact expected common point and verifies that opposite
normals remain distinct. Final formatting, MSRV all-target/all-feature Clippy
with `-D warnings`, the complete Hypercurve release suite (72 suites, 2,646
passed, 35 existing ignored), and all 43 downstream adapter/profile/sweep/mixed
checks pass. The original full fixture is unchanged and still fails during
erosion at `Offset/QuadraticBezier` against `RationalBezier/RealSign` (11.89 s).
This failed probe is not evidence of completed erosion or a speedup. The
increment establishes no new replay, total-work, heap-byte or cancellation
bound, and has not completed the full repository pre-push matrix. Raw checks,
contract and source patch are in
`docs/evidence/2026-10-26-hypercurve-closure-band-endpoints`; its digest is in
`docs/evidence-archive.tsv`.

| Scoped witness | Representation | Evaluation/differentials | Incidence/order | Region admission | Booleans | Further offset | Corner edits | Reconstruction | Exact replay |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| B: smooth/opposed band junctions | Original fragments and shared proved point | Exact signed normal endpoints | Strict equality or preserved distinction | Certified source loop | — | Signed line/circle bands | — | — | — |
| O after B: unchanged arc-corner chain | Generated mixed boundary | — | Earlier native line/circle band contacts complete | Certified before erosion | Earlier set identities complete | Blocked: QuadraticBezier/RationalBezier/RealSign | Both fillets complete | Reversal and rebuild complete | — |

**Native line endpoint proof transport, M1 increment (2026-10-26).** The
`LineSeg2::with_endpoint_representations` owner on `532e517b07` / `07f1157275`
returns a fresh segment when no retained trim support is present, discarding
independent offset provenance. The new direct regression reproduces that loss:
a proved distinct parallel relation becomes absent after equal endpoint
replacement. The helper now preserves support, signed offset source/distance,
orientation and existing endpoint-distinctness evidence together. Equal endpoint
values remain its caller-certified precondition. P15, P18 and P19 apply; this
changes no public type, serialization format or predicate policy.

The witness
`equal_line_endpoint_representations_preserve_offset_provenance` uses algebraic
coordinates, both trimmed and untrimmed offset lines, coincident/distinct support
relations, and a subsequent signed offset with an independently expected total
distance. The isolated final change passes formatting, strict all-target and
all-feature MSRV Clippy, all 2,647 Hypercurve release tests (72 suites, 35 existing
ignored), and all 43 downstream adapter/profile/sweep/mixed tests. The unchanged
full arc-corner probe still reports the band-only `QuadraticBezier/RationalBezier`
`RealSign` blocker (12.14 s); this repair alone does not complete erosion. No
performance, native replay or total resource-bound claim follows from these
checks, and the full repository pre-push matrix is pending. Raw baseline,
qualification logs and patch are in
`docs/evidence/2026-10-26-hypercurve-closure-line-endpoints`, with its digest in
`docs/evidence-archive.tsv`.

| Scoped witness | Representation | Evaluation/differentials | Incidence/order | Region admission | Booleans | Further offset | Corner edits | Reconstruction | Exact replay |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| L: native offset line endpoints | Support, offset and distinctness proofs retained | Original orientation retained | Coincident/distinct parallel relations preserved | — | — | Exact signed distance composition | — | Certified equal endpoint substitution | — |

**Native oriented contact evidence, M1 increment (2026-10-26).** On
`904e5408e1`, native line/line and line/arc intersections discard exact tangent
signs available from their supporting directions. The focused regression
reproduces an absent positive crossing sign. The native intersection evidence
owner now retains structural signs of the directed support cross product;
for a circle, it uses `cross(L, J(P-C)) = dot(L, P-C)`. Native endpoint
parameters retain the stored arc point identity. Arc reversal and operand
exchange transport the oriented sign. A proved nonzero sign certifies
transversality; a proved zero remains a tangency fact; an unavailable sign
remains absent. P15, P18 and P19 apply without a new public type or format.

`native_contacts_retain_exact_oriented_tangent_facts` covers rational and
algebraically translated centers, crossings, tangencies, collinear endpoint
contacts, reversals, exchanged operands, and lines separated from tangency by
`1/10^12`. Completed native line evidence publishes under zero refinement and
promotion budgets. The optional scalar fact query requests no such work, but
may traverse a shared expression graph: this is not a total-work or memory
bound. No new cancellation or exact replay claim follows.

The final isolated increment passes formatting, strict all-target and
all-feature MSRV Clippy, all 2,648 Hypercurve release tests (72 suites,
35 existing ignored), and all 43 downstream adapter/profile/sweep/mixed tests.
The original full arc-corner fixture remains unchanged and still reports
`Offset/QuadraticBezier` against `RationalBezier/RealSign` (14.80 s). That
failed probe is not completed erosion or a performance comparison. The full
repository pre-push matrix for this increment remains pending. The separate
local evidence branch retains the baseline, checks and source patch under
`docs/evidence/2026-10-26-hypercurve-closure-native-contact-evidence`; its
digest is in `docs/evidence-archive.tsv`.

| Scoped witness | Representation | Evaluation/differentials | Incidence/order | Region admission | Booleans | Further offset | Corner edits | Reconstruction | Exact replay |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| N: completed native contacts | Original lines/arcs and endpoint identity | Directed support tangent signs | Certified crossing/tangency facts retained | Existing corpus protected | Existing corpus protected | — | — | — | — |

**Retained line fragments, M1 increment (2026-10-26).** Exact affine line
subdivision, selected trimming, boundary endpoint sharing and native contour
materialization rebuilt lines from their endpoints, losing known supporting
lines and signed offset relations. The focused baselines reproduce the missing
support and offset facts. A separate `13c3e18375` regression proves that the
first offset of a reversed retained line recorded its source direction
backwards even though its coordinates were correct.

The owning `LineSeg2` restriction helper now transports support, traversal,
existing endpoint-distinctness evidence and signed offset provenance. Its first
offset orients the retained source with the current fragment. Quadratic affine
splits transport each noncollapsed child independently; cuts at zero or one
keep the complete surviving source, and reversed parameter intervals transport
oriented endpoint contacts. Public native subcurves, CurveString trims, equal
boundary vertices and retained native contour materialization use that same
owner. Existing endpoint/range/image validation remains in force. P15 and
P17–P20 apply; no new public carrier or interchange format is introduced.

Seven regressions cover rational and algebraic origins, oblique offsets,
interior and exterior algebraic cuts, collapsed children, independently
represented equal points, both traversal directions, tangent contact incidence,
nearby distinct parallel lines and further signed offsets. Materialization
covers quadratic and rational affine spans in authored, retained and analytic
parallel fragments; the resulting contour retains those proofs. A strict
scalar direction query can transport the sign beyond structural cases. When
broader symbolic direction remains unproved, the exact curve is retained but
oriented contact proof is unavailable. This is not complete algebraic closure.

Final checks pass: `cargo fmt --all -- --check`; MSRV Hypercurve Clippy with
`--all-targets --all-features -- -D warnings`; all 2,655 Hypercurve release
all-features tests (72 suites, 35 existing ignored); and all 43 downstream
adapter/profile/sweep/mixed tests. The unchanged full arc-corner probe still
reports `Offset/QuadraticBezier` against `RationalBezier/RealSign` (13.20 s).
Initial harness/fixture errors remain separate from reproduced proof failures.
The separately captured combined prototype completes the original arc-corner
workflow, but includes native pair changes outside this increment. That
completion is not attributed to this isolated line patch. Refinement/promotion
counts do not bound total graph work or memory; no speedup, new cancellation or
native replay claim follows. The full repository pre-push matrix for this
increment is pending. Baselines, source patches and checks are retained in
`docs/evidence/2026-10-26-hypercurve-closure-line-fragments`, with its digest in
`docs/evidence-archive.tsv`.

| Scoped witness | Representation | Evaluation/differentials | Incidence/order | Region admission | Booleans | Further offset | Corner edits | Reconstruction | Exact replay |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| F: selected native line fragments | Shared support and signed offset proofs | Reversal and affine split directions | Endpoint tangencies and distinct parallels | Native contour materialization | Existing corpus | Exact signed distance composition | — | Trimmed/equal boundary endpoints retain proof | — |

**Native parallel pairs, M1 increment (2026-10-26).** At `515e7647a9`, the
unchanged arc-corner workflow still blocks during erosion with
`Offset/QuadraticBezier` against `RationalBezier/RealSign`. The region pair
owner now consumes retained native affine lines and regular quadratic circular
charts without discarding their support, signed offset or selected parameter
relations. Adjacent certified loop endpoints retain their original parameters;
independent exact contacts are mapped back into the selected source range.
Concentric circular components preserve their circle evidence. Native disjoint
circle supports can prove an empty pair, while tangency and coincidence retain
the complete intersection path. Existing general algorithms remain available.

P12, P15 and P17–P20 govern this extension. Its domain is certified native
line sources and retained quadratic rational circle charts on admitted unit
subranges, with algebraic endpoints, signed normal offsets and both traversal
directions. A circular recognition alone does not prove normal regularity:
an injective degree-six chart with a stationary parameter at one half exposed
an incorrect early shortcut. The regression preserves that excluded parameter;
the new whole-domain shortcut requires the existing quadratic-chart and range
proofs. Zero-distance behavior and general higher-degree paths remain intact.

Four focused witnesses check original algebraic trim parameters, operand
exchange and reversal, tangent and coincident circles, negative offsets and
normal reversal, exact `1/10^12` separation, the stationary chart, and a reached
zero refinement budget followed by a successful larger-budget retry. Optional
native paths preserve invalid and exhausted outcomes when falling back.

The original `round_stroke_boundary_of_a_filleted_arc_corner_refillets_and_erodes`
now completes, with its geometry and assertions unchanged. It fillets, builds
a round stroke, reconstructs and reverses the boundary, checks an empty XOR,
refillets, checks added/removed material, erodes by `1/16`, and checks a further
empty difference. It is an ordinary test; only its ignore annotation and stale
blocker explanation were removed. A separate fresh-process run completes in
13.64 s. This is qualification under concurrent load, not a speedup comparison.

Final checks pass: all 2,660 Hypercurve release all-features tests (72 suites,
34 remaining ignored), all 43 downstream adapter/profile/sweep/mixed tests,
MSRV Clippy with `--all-targets --all-features -- -D warnings`, formatting,
typos and repository-budget checks. Source fingerprints establish that the
isolated commit has exactly the source and test bytes of the combined checkout
that ran these checks. Full repository pre-push qualification for this increment
remains pending; the frozen `904e5408e1` run does not cover it. Evidence is
`docs/evidence/2026-10-26-hypercurve-closure-native-pairs`, including the original
blocked baseline, rejected stationary shortcut, final checks and source patches;
its digest is in `docs/evidence-archive.tsv`.

| Scoped witness | Representation | Evaluation/differentials | Incidence/order | Region admission | Booleans | Further offset | Corner edits | Reconstruction | Exact replay |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| P: native parallel pairs | Retained affine lines and quadratic circle charts | Normal-side and stationary-domain guards | Original selected parameters and directed contacts | Arc-corner result admitted | XOR and difference identities | Completed erosion and subsequent consumption | Completed refillet | Rebuilt/reversed stroke boundary | — |

This completes the named arc-corner witness. M1, general algebraic closure,
generated native save/reload and total work/memory/cancellation bounds remain
open. Counted refinement exhaustion is not a bound on all computation.

**Still open.**
- The completed arc-corner witness does not establish M1 or general closure.
  Broader mixed-family compositions, transformed generated chains and native
  exact replay still need their own acceptance witnesses and algorithms.
- The earlier record listed six cases on this board, four with nonparallel `A` and `B` crossing on the arc. They do not occur on the current board. If such a case recurs: `A` and `B` meet in one point `Q` in their own field. The contacts are equal exactly when `Q` lies on the circle, an exact zero test of `|Q - C|² - r²`, and the branch sign above then selects `Q` on each line.
- The earlier options remain for such cases: line-conic contacts on a circular conic in tower-representable form, from the circle's exact center and radius as `line_circle_relation_at_endpoints` does for native arcs; `quadratic_tower` beyond one outer radical; or a zero-separation bound, which `algebraic_separation_bound_bits` does not give past 256 nodes or at `Inverse` nodes without an exact child sign.

## 8. Polynomial-branch sections and filled arrangements

**Admitted.** A graph branch and its retained invertible rational similarities
meet authored native lines with exact algebraic coordinates, or returned restrictions of them,
in both intersection APIs and both operand orders. Parameters remain in their
original source charts. Both operands split and the pieces re-enter the same
query; selected algebraic trims, reversal, tangent contacts and singular-event
contacts retain their meanings. A selected common line keeps its overlap
correspondence, while other branches retain their finite contacts with that
line. The recorded scalar and selected-field decisions must succeed exactly;
unresolved predicates retain their typed blockers. Selected polynomial pairs
also retain finite contacts and common components in a shared source chart or
equal rational similarity image. Singular tangent signs may remain unclassified.
Generated chords additionally retain certified support views and point-based
parameters for native supports, constant-abscissa charts and proved constant
selected components. Selected graph boundaries also retain these supports in
the shared region arrangement. The bounded region witnesses cover exact
classification, Booleans, reconstruction, open clipping, rational similarities,
mixed circular boundaries and cusp/node incidence. Broader algebraic
similarities, different pair images, nonlinear maps, general unrelated selected
chord supports and completion of the full algebraic region domain remain gaps.

**Rational-similarity section increment (2026-10-26).** Against `b1af6465b3`,
the original translated quintic fixture reproduced `Unsupported`. Pulling the
line back through the retained rational similarity reuses the source section
authority. Native line evaluation retains an affine image of the same selected
fiber, and transformed coordinate comparison consumes its existing field
relations. The independent sloped-line case also reproduced an undecided
point comparison before those relations were consumed. No coordinate
materialization or new implicit equation is required by these paths.

The retained witnesses substitute the transformed quintic independently, check
isolating bounds, both operand orders and reversals, selected trims through a
further transform, subdivision/requery, point-definition replay and distinct
features separated by `2^-80`. They exercise reached counted exhaustion,
larger-budget retry and strict/approximate/strict policy changes. The final
release suite passes 2,664 tests (34 existing ignored; 72 suites), all 43
affected downstream tests, strict all-target/all-feature MSRV Clippy,
formatting, 16 matrix guards and 22 capability witnesses. Typos, the repository
budget and diff checks pass. Raw source attribution, baseline failures and
qualification commands are retained in evidence record
`docs/evidence/2026-10-26-hypercurve-closure-mapped-polynomial`, digest
`81fd5fbcd665b34092c65e8e914400cdb35157a285d7545e84d3e45700f6f91d`.
Point replay here is in memory, and counted refinements/promotions are not a
total-work, memory or cancellation bound. Concurrent timings establish no
performance improvement. A2, full algebraic closure and this increment's full
repository pre-push qualification remain open.

**Line-component section increment (2026-10-26).** Against `45947259af`,
the original shared-line inputs and three new component witnesses reproduced
`Unsupported`. A global line identity and the existing checked slab/fiber
selection now distinguish the selected line branch from the other branches.
The original polynomial remains the point and branch authority. For a different
branch, exact division by the line supplies finite candidates and regular
tangent signs, including a transverse node or tangent contact where the full
polynomial's gradient vanishes.

The existing overlap correspondence clips both original parameter domains;
it retains algebraic trims, traversal and endpoint inclusion without rebasing
the map. Singleton intersections and disjoint same-support intervals remain
distinct. Five new witnesses cover these outcomes, both operand orders and
reversals, rational rotations/reflections, exact restricted parameter pairs,
distinct irrational branches, subdivision/requery, selected-point replay and
reached counted exhaustion followed by a larger-budget retry and policy
changes. A reviewed prototype used closed isolator membership; the retained
implementation delegates endpoint ownership to the checked fiber authority.
The irrational-branch fixture passed both versions and is not evidence of a
runtime failure in that prototype.

The final release suite passes 2,669 tests (34 existing ignored; 73 suites),
all 43 affected downstream tests, strict all-target/all-feature MSRV Clippy,
formatting, 16 matrix guards and all 27 capability witnesses. Typos, repository
budget and diff checks pass. Source snapshots, original failures and exact
commands are retained in
`docs/evidence/2026-10-26-hypercurve-closure-line-components`, digest
`9bc06b8764a2564c2ccd5f4ce42b3e952db4d0edc5e254116c273c3c0cfcf5c4`.
This does not complete A2, repeated/vertical component topology, filled
polynomial regions or fresh-process native replay. Counted work is not a
total-work, memory or cancellation bound. No performance improvement is claimed;
full repository pre-push qualification for this increment remains pending.

**Algebraic native-line increment (2026-10-26).** The original irrational
vertical and sloped lines reproduced `Unsupported` against the frozen
line-component candidate. Direct evaluation at algebraic abscissas already
passed and remains a control. Native line admission now uses exact scalar
decisions for chart direction and substituted coefficients. An initial
prototype falsely reported an empty intersection for the component
`y = -(sqrt(2) + sqrt(3))` of `y^4 - 10y^2 + 1`; the retained implementation
certifies coefficient zeros before distinguishing finite sections, empty
sections and common supports. The failure and candidate are archived.

Seven witnesses cover the original cases, cubic and seventh-root coordinates,
mixed radicals, selected common components and distinct nearby supports.
Independent defining equations, both operand orders and traversals, rational
rotations/reflections, subdivision/requery, retained-point replay and a reached
counted budget followed by retry verify the admitted composition. Policy
changes retain certified results. Selected chord carriers and broader
algebraic maps remain outside this increment's qualification.

The final release suite passes 2,676 tests (34 existing ignored; 74 suites),
all 43 affected downstream tests, strict all-target/all-feature MSRV Clippy,
formatting, 16 matrix guards and all 34 capability witnesses. Typos, repository
budget and diff checks pass. Qualification used `45947259af` plus the frozen
component and algebraic-line inputs; their bytes were verified unchanged when
advancing the commit parent to `9da58d3ac3`. Raw commands, snapshots and failures
are retained in `docs/evidence/2026-10-26-hypercurve-closure-algebraic-lines`,
digest `3404dc901761f282c45021ad7d399095a624a2f5961203a38cdc45734a88aec2`.
This does not complete A2, filled polynomial regions or fresh-process native
replay. Counted work is not a total-work, memory or cancellation bound. No
performance improvement is claimed; full repository pre-push qualification
for this increment remains pending.

**Selected-fiber ordering increment (2026-10-26).** At `fd1313a23a9`,
the independently retained interior point of `(y-x)(y+x)` and event endpoint
of `(y-x)(2y-1)` at `(1/2,1/2)` exhaust all 512 counted refinements without a
promotion. Both base parameters are already exact `1/2`. An interior isolator
can converge to an exact singleton without ever fitting inside it; the missing
step was consuming the retained exact value. This reproduces independently of
the polynomial-pair prototype that exposed the comparison.

P4, P15, P19 and P20 govern the scalar fix. Existing interval proofs run first;
then either operand's retained native or exact singleton evidence can decide
the comparison through the existing scalar authority. Non-budget unknowns keep
the original fallback. A second regression over retained `sqrt(2)` compares
`sqrt(3) +- 2^-80` with independently certified native `sqrt(3)`. When both the
incidence and query are structurally independent of that base, the existing
exact univariate path now decides without refining the irrelevant base. The
original fiber, selectors, multiplicity and actual query dependencies remain
intact; no global coordinate projection or new scalar authority is introduced.

Three public witnesses and two private scalar regressions check both operand
orders, distinct neighbors, negative conjugates, repeated roots, dependent
queries and equations, checked point-definition replay, policy changes and
actual zero-budget exhaustion followed by a larger retry. Final checks pass:
2,681 Hypercurve tests (75 suites, 34 existing ignored), 43 root downstream
tests, and 3,009 tests across Hyperbrep, Hyperbrep IO, Hypercircuit, Hyperdrc,
Hypergraphics and Hyperpack (0 existing ignored). Strict all-target/all-feature MSRV
Clippy, formatting, 16 matrix guards, 37 capability witnesses, typos, repository
budget and diff checks pass. An initial exact unit-test filter matched zero
tests; its retained log is not credited, and the corrected test and full suite
both execute the regression.

Raw baselines, source hashes and commands are retained in
`docs/evidence/2026-10-26-hypercurve-closure-fiber-order`, digest
`a82d1aafbb3ce0ce84f909eb99da5e5ae35b9bf76c82c2382847d05ae64d4435`.
Point replay is in memory; counted refinements/promotions do not bound total
arithmetic, coefficient growth, memory or cancellation. This does not complete
A2, general polynomial operations or fresh-process native archive replay. No
performance improvement is claimed. Full repository pre-push qualification
for this increment remains pending.

**Polynomial-pair incidence increment (2026-10-26).** At `fd1313a23a9`,
three original pair witnesses reproduce `Unsupported`. The original positive
parabola/quintic frontier remains intact as a successful empty result in both
operand orders and APIs. Selected graph branches now intersect in a shared
source chart or equal rational similarity image. Existing Hypersolve complete
resultants propose abscissas; both original selected fibers decide incidence.
Common factors retain selected overlap correspondences, residual roots and
mixed-component contacts at original slab boundaries. Original source equations,
root selections and active trim parameters remain authoritative.

P15 and P17–P20 govern this extension. A same-root sample establishes overlap
only on the checked event-free source slabs. Correspondence uses the original
source abscissa chart, including negative and nonunit ranges. The first
prototype incorrectly clipped a negative overlap to empty with a unit chart;
that failure and its exact regression are retained. Shared similarities are
compared by rational coefficient value, and cross signs account for determinant
orientation, traversal and operand order. Singular incidence may complete with
an absent tangent sign; a vanishing gradient is never published as tangency.
The mixed contact at `(1/2,1/2)` exposed the independently qualified selected-
fiber ordering defect recorded above.

Nine public witnesses cover the original cases, common nonlinear and linear
components, residual and mixed contacts, algebraic trims, exact negative-range
correspondences, tangent and singular contacts, distinct `2^-100` supports,
shared rational transforms, actual reached counted exhaustion and retry, policy
changes, in-memory point replay and split/requery. A regular `y=x^37` section
explicitly exceeds the ordinary bounded resultant degree schedule, then
completes through the existing continuation. This one degree-37 case is not an
arbitrary-degree completion proof. Different image transforms retain their
explicit `Unsupported` control; filled polynomial region admission remains
blocked in the unchanged original fixture.

The final release suite passes 2,690 tests (76 suites, 34 existing ignored),
all 43 root downstream tests and 3,009 tests across six direct dependent
packages (0 existing ignored). Strict all-target/all-feature MSRV Clippy,
formatting, 16 matrix guards, all 46 capability witnesses, typos, repository
budget and diff checks pass. Qualification used `fd1313a23a9` plus the frozen
selected-fiber prerequisite and pair inputs; non-receipt source bytes were
verified unchanged when advancing the commit parent to `f0fa18b7b8`.
Raw failures, source snapshots and commands are retained in
`docs/evidence/2026-10-26-hypercurve-closure-polynomial-pairs`, digest
`3a1cc48424006b37ad2c2be99587216ef49b9d48e0db8acd31fcdea986ef7b13`.

A2, general algebraic closure, broader coefficient fields and images, repeated
or vertical topology components, filled polynomial operations and fresh-process
native replay remain open. Counted refinements/promotions and external fixture
timeouts are not total-work, memory or cancellation bounds. No performance
improvement is claimed. Full repository pre-push qualification for this
increment remains pending.

**Generated polynomial-line increment (2026-10-26).** At `fd1313a23a9`,
three original generated-line queries return `Unsupported` although exact
native `LineSeg2` controls succeed. The retained cases use rational endpoints,
selected `+-sqrt(2)` endpoints without represented Cartesian coordinates, and
a selected common-line component. Generated chords now reuse certified support
views while retaining their own point-based parameters and finite-domain proof.
Neither selected coordinates nor an authored-family conversion are required
when the existing source-chart relation supplies the support.

P15 and P17–P20 govern this extension. A native retained support or a represented
constant abscissa can certify a line in the world or rational-similarity source
chart. A polynomial independent of source x proves a constant selected support;
exact ordinate comparison selects its common or disjoint branch. Matching
endpoints alone never proves polynomial linearity. Overlaps keep a shared
source/chord correspondence with original domains, repeated clipping, traversal
and endpoint-only contacts. An independent public point-location regression
found a lower-layer omission: chord side refinement bypassed its already
certified cardinal direction. It now consumes the existing exact coordinate
predicate before box refinement, preserving uncertain and exhausted outcomes.

Ten witnesses retain the original failures and native controls. They exercise
both orders and reversals, exact defining equations, finite contacts, tangency,
negative-range overlap clipping and requery, distinct `2^-80` supports, source
drop, in-memory point replay, rational rotations/reflections and the oblique
scale-five map `(3x-4y,4x+3y)`. Transformed contacts satisfy independently expanded
world equations. A reached zero work allowance remains typed exhaustion; a
larger retry and STRICT/APPROXIMATE_512/STRICT retain Certified results. Coincident
endpoints remain a typed invalid construction.

Final checks pass: 2,700 Hypercurve tests (77 suites, 34 existing ignored),
43 root downstream tests, 3,009 tests across six direct dependent packages
(0 existing ignored), strict all-target/all-feature MSRV Clippy, formatting,
16 matrix guards, 56 capability witnesses, typos, repository budget and diff
checks. Qualification used `f0fa18b7b8` plus frozen polynomial-pair inputs and
this patch; non-receipt bytes remain identical after advancing the commit
parent to `00dbdb152a`. Raw baselines and qualification are retained in
`docs/evidence/2026-10-26-hypercurve-closure-polynomial-chords`, digest
`07ec6f93dc278782911c1756d62550e489a493df418ab3efdfdb75193f6cc35d`.

General unrelated selected oblique lines, polynomial/rational and
polynomial/parallel pairs, broader coefficient/transform domains, filled
polynomial operations, A2 completion and fresh-process generated native replay
remain open. Counted refinements/promotions and external fixture timeouts are
not total-work, memory or cancellation bounds. No performance improvement is
claimed. Full repository pre-push qualification for this increment is pending.

**Selected polynomial-abscissa increment (2026-10-26).** The original
polynomial/rational intersection probes exposed an independent evaluation gap.
On `00dbdb152a` plus frozen generated-line inputs, a standalone `point_at`
witness reproduces `Evaluation/ImplicitPolynomial/Unsupported`: the retained
X of `C(t)=(t^2,-t)`, selected by `t^5+t^2+5t-5=0`, has no ordinary Bezier
parameter view. Radical and fifth-root controls already pass and remain intact.

P15 and P18–P20 govern this change. The polynomial fiber is now pulled back
through an abscissa's exact rational source expression over its original
selected algebraic base. Hypersolve owns the arithmetic substitution, clearing
only the active first-coordinate degree and retaining denominator factors.
Hypercurve certifies the original denominator, checks the source slab's ordered
fiber census, selects the original Y branch and retains the rational image.
A subsequent coordinate query can pull point maps back to the same base when
its numerator and denominator are structurally independent of the selected Y
fiber. It does not project unrelated dependent roots or approximate coordinates.

The expanded prototype first exposed a second `Unsupported` when consuming its
own output X. That failure remains alongside the original quintic. Ten public
witnesses now cover repeated output use, checked in-memory point replay, both
ordered parabola branches, both signs of rational denominators, rejected
removable poles, restriction, reversal and an independently expanded transformed
quintic. A retained zero-distance parallel, changed base selections and an exact
`2^-80` coordinate shift preserve their identities. A reached zero counted
allowance permits a larger retry; STRICT/APPROXIMATE_512/STRICT keeps Certified
results. Hypersolve's independent substitution identity test checks degree and
original pole factors.

Final checks pass: 1,213 Hypersolve tests (23 suites, none ignored), 2,710
Hypercurve tests (78 suites, 34 existing ignored), 43 root downstream tests and
3,009 tests across six dependent packages (0 existing ignored).
Strict all-target/all-feature MSRV Clippy for both owners, formatting, 16 matrix
guards, 66 capability witnesses, typos, repository budget and diff checks pass.
Qualification used `00dbdb152a` plus the frozen generated-line and abscissa
inputs; non-receipt bytes remain identical after advancing the commit parent to
`09ce677746`. Raw baselines and qualification are retained in
`docs/evidence/2026-10-26-hypercurve-closure-selected-abscissas`, digest
`1e45871c00e897e88d579812c2bb87e974e9ad9a356f6e36d6cae467992a4014`.

General nested/dependent-fiber charts, broader coefficient domains, polynomial
filled operations, A2 completion and fresh-process generated native archive
replay remain open. The separately tested rational-pair prototype is not
included in this increment. Counted refinements/promotions and external fixture
timeouts do not bound total arithmetic, coefficient growth, retained memory or
cooperative cancellation. No performance improvement is claimed. Full repository
pre-push qualification for this increment remains pending.

**Polynomial/rational section increment (2026-10-26).** Four original
pair fixtures reproduce typed `Unsupported` on the generated-line prerequisite:
a generic quintic against a quadratic and a rational circle, and parabola/circle
common components. An independent retained-abscissa evaluation gap was isolated
and qualified separately before this pair implementation consumed its output.

P15 and P17–P20 govern the extension. Exact homogeneous substitution isolates
finite candidates in the rational source parameter. Original denominators,
active source ranges, selected polynomial branches and exact point equality
validate each retained contact. Homogeneous gradients establish tangent signs.
For an identically vanishing substitution, certified monotone source-abscissa
pieces partition a pole-free rational trace; exact inversion and the source
slab's branch selection establish retained common components. Correspondences
keep both original charts and local span ranges through reversal, restriction,
algebraic clipping and later queries. Rational similarity charts pull back
through their exact inverse. Polynomial `parameter_from_span` now preserves the
original selected abscissa for span zero and rejects invalid span indices.

Expanded probes first exposed incorrect spline span ranges, missing polynomial
span conversion, transformed-chart blockers and a duplicate isolated cusp
contact already covered by an overlap. These failures remain archived alongside
the original four cases. Twenty public tests now pass, including three preserved
abscissa controls. Independent equations and topology assertions cover both
ordered branches, original selected roots, exchange and reversal, tangent and
singular contacts, spline/NURBS seams, multiple visits, cusps, rotation and
reflection, degree-seven and degree-37 sources, selected overlap trims,
no-enlargement checks and subsequent intersections. Checked in-memory point
replay, reached zero-budget exhaustion, larger retry and Certified policy
transitions preserve evidence. The capability catalog adds 17 distinct entries,
without duplicating the three existing controls.

Final checks pass: 2,730 Hypercurve tests (79 suites, 34 existing ignored), 43
root downstream tests and 3,009 tests across six dependent packages
(0 existing ignored). Strict all-target/all-feature MSRV Clippy,
formatting, 16 matrix guards, 83 capability witnesses, typos, repository budget
and diff checks pass. Qualification used `09ce677746` plus frozen abscissa and
rational-section inputs; non-receipt bytes remain identical after advancing the
commit parent to `80bcc08ae9`. Raw baselines and qualification are retained in
`docs/evidence/2026-10-26-hypercurve-closure-rational-sections`, digest
`c55babd43904c40db014bf6053f8aa1fceaad9a4d2a30fdc1d9f2156a8ab5e48`.

Constant-abscissa or collapsed common traces, nonlinear branch maps,
polynomial/normal-parallel pairs, general coefficient fields, polynomial regions,
A2 completion and fresh-process generated native archive replay remain open.
Counted refinements/promotions and external fixture timeouts do not bound all
arithmetic, coefficient growth, retained memory or cooperative cancellation.
Concurrent timings do not establish a performance improvement. Full repository
pre-push qualification for this increment remains pending.

**Polynomial point-location increment (2026-10-26).** Three public
fixtures reproduce `Intersection/ImplicitPolynomial/Unsupported`: a branch's own
selected fiber point, independent native parabola points and a generated
rational contact. These original failures remain alongside the expanded corpus.

P15 and P17–P20 govern the extension. The selected branch recovers a point's
original source abscissa, pulling an invertible similarity back exactly. It
checks the closed active domain and equality with the branch's ordered fiber;
a point on another root of the same polynomial is not incident. A parameter
proved equal to a boundary reuses that boundary's retained authority. No source
or ordinate is rounded or independently projected.

Eight public witnesses now cover selected and native points, both ordered
branches, checked point replay, exact `2^-80` off-support points, shared node
and cusp endpoints, selected trims, reversal, later intersections and location
on their split pieces. Quarter rotation/translation, reflection and uniform
`sqrt(2)` scaling use independently expanded world equations. Reached zero
counted work returns exhaustion; a larger retry and
STRICT/APPROXIMATE_512/STRICT transitions preserve Certified results.

Final checks pass: 2,738 Hypercurve tests (80 suites, 34 existing ignored), 43
root downstream tests and 3,009 tests across six dependent packages
(0 existing ignored). Strict all-target/all-feature MSRV Clippy,
formatting, 16 matrix guards, 91 capability witnesses, typos, repository budget
and diff checks pass. Qualification used `09ce677746` plus frozen abscissa,
rational-section and point-location inputs. All non-receipt bytes remain
identical after advancing the parent to `d7ae8ea218`. Raw evidence is in
`docs/evidence/2026-10-26-hypercurve-closure-polynomial-locations`, SHA-256
`18441451082feb7c408e2951a88f80a9b84e2a05eb699cb35de72f0e8d1433d9`.

Nonlinear rational maps may have multiple or continuous preimages and require
their own inverse/selection contract. Polynomial filled arrangements, A2/A3
completion, broader dependent fields and fresh-process generated native replay
remain open. Counted work and external timeouts do not bound all arithmetic,
coefficient growth, retained memory or cancellation. No performance improvement
is claimed. Full repository pre-push qualification remains pending.

**Polynomial shared-region increment (2026-10-27).** The unchanged H08
parabola boundary first reproduced `Construction/ImplicitPolynomial/Unsupported`
for orientation and admission. Its selected branches now remain supporting
fragments in the existing `CurveRegion2` arrangement. P12–P13, P15 and P17–P20
require the original equation, ordered root, exact domain, traversal and contact
points to survive splitting and publication. A Boolean introduces no new supports.
No rational fit or alternate region authority is introduced.

**API migration:** `CurveError` gains
`UnsupportedFiniteCurveProjection { family }`. External exhaustive Rust
matches must add this arm or an appropriate catch-all, preserving an unsupported
family outcome and returning no sampled geometry. Exact-operation adapters map
it to `ExactCurveError::Blocked` with `UncertaintyReason::Unsupported` and the
affected family. This is a breaking Rust API addition for exhaustive matches
and requires appropriate package release versioning; package publication is
not part of this increment.

Initial carrier admission still produced two loops instead of one for a clip.
The retained original fixture now passes through the existing exact boundary
probe. Selected point incidence is reused before unrelated probes; polynomial
fragment classification retains its exact interior point. The same authority
handles reconstructed path points and open authored/generated clipping. The
original second quintic cut at `x=3/4` is retained with its correct five-piece
expectation (`f(3/4)=-461/1024`), followed by a cut at `7/8` yielding four pieces
(`f(7/8)=21415/32768`). This corrects the test oracle without dropping the case.

Sixteen new public witnesses and the promoted original H08 fixture cover exact
point and path classification, all four Booleans, holes and reversal under both
fill rules, disconnected material, rational similarities, regularized empty
contact, `2^-80` separation, degree-five selected trims, checked point replay,
reconstruction and further operation, mixed circular supports, open clipping,
cusp and node incidence, zero offset, and actually reached budget exhaustion
with a larger retry. STRICT/APPROXIMATE_512/STRICT receipts remain Certified.
Independent equations, source/root/domain assertions and topology accompany the
set identities; sampled membership alone is not the oracle.

The first complete suite exposed one existing error-contract regression:
an unsupported polynomial pair returned a partial result instead of its prior
typed error after region-carrier admission. The generic support fallback now
preserves that representation failure. The original assertion remains unchanged,
and the failed full-suite output is retained with the other attempts.

Final checks on `5de7f0cb05` plus the frozen increment pass: 2,754 Hypercurve
tests (81 suites, 34 existing ignored), 43 root downstream tests and 3,009
tests across six dependent packages (0 existing ignored). Strict Hypercurve
all-target/all-feature MSRV Clippy, formatting, 16 matrix guards, 107 capability
witnesses, typos, repository budget and diff checks pass. All earlier failed
attempts, diagnostic sources and the interrupted prototype are retained. Raw
evidence is in `docs/evidence/2026-10-27-hypercurve-closure-polynomial-regions`, SHA-256
`6f2a9bdaf22a1b19e14126ff46325060a0532769608178d363979ea008f51c9b`. All non-receipt bytes remain identical after adding this record.

This is bounded A3 progress. Arbitrary coefficient fields, general image maps,
vertical/repeated components and every singular local sector remain open.
Nonzero polynomial-support offsets, fillets, chamfers and finite projection
still return typed Unsupported; those guard tests do not count as geometric
completion. Checked point definitions do not establish fresh-process native
replay or full BRep trim support. Counted work and external timeouts do not bound
all arithmetic, coefficient growth, retained memory or cancellation. No
performance improvement is claimed. The separate publication candidate through
`b1c011a070` excludes this increment; full repository pre-push qualification is
still required before publishing it.

**Horizontal boundary-component increment (2026-10-27).** On committed
`5de7f0cb05`, the original `y+2=0` graph in `[-2,2]^2` and both components
of `y^2-4=0` reproduced `BoundaryComponent`. Hypersolve now keeps a horizontal
boundary factor in the original equation and closed fibers. Such a component
has no discrete boundary-entry event. Contacts with other branches are already
zeros of `resultant(F,F_y)`; existing root-free separators and closed strip
counts establish event incidence. P11, P15 and P17–P20 remain owned there.

Ten solver witnesses cover the original lower/upper cases and the documented
circle-plus-boundary fixture, tangent parabola/circle contacts, crossings with
one side outside, algebraic event fibers, `2^-80` distinct components, checked
source replay, and actual counted exhaustion/retry. Independent rational Sturm
counts include closed endpoints. Six Hypercurve witnesses consume these graphs
through degree-five selected trims, finite intersections and overlap, point
location, singular endpoint incidence, reversal, similarity transport and
fresh-process diagnostic source replay followed by another operation. The
original HyperBRep boundary tensor is retained as a positive section witness,
with both pcurves, whole-image validation, native archive replay and affine
transport/revalidation; its upper-boundary counterpart also completes.

The stronger spatial fixture also reproduced a directed pcurve correspondence
mismatch. Matching directed endpoints fixes that error on both the new section
and an already admitted rational source restriction. A further plane contact
then exposed a rational-map dispatch gap: exact similarities stored as quotients
now supply a checked view to the existing line-section algorithm. Original map
definitions and denominator guards remain, including removable factors. This
does not complete general rational images or nonuniform affine incidence.

Final checks pass: 1,223 Hypersolve tests (24 suites), 2,744 Hypercurve tests
(81 suites, 34 existing ignored), 43 root downstream tests and 3,010 tests
across six dependent packages (0 existing ignored). Strict
all-target/all-feature MSRV Clippy for Hypersolve, Hypercurve and HyperBRep,
formatting, 16 matrix guards, 97 capability witnesses, typos, repository budget
and diff checks pass. All failed attempts remain. Raw evidence is in
`docs/evidence/2026-10-27-hypercurve-closure-horizontal-boundaries`, SHA-256 `3f1bfd095d5be2dfeca96045eaa0815ac6b5544b4e045fcfb4be5af0c9547b52`. All non-receipt bytes match the qualified
snapshot after this receipt.

No factor or multiplicity is discarded. Vertical/repeated components, broader
coefficient fields and general algebraic operations remain open. The diagnostic
source encoding is not a native Hypercurve archive format. This increment does
not include the separately qualified polynomial region work. Counted work and
external timeouts do not bound every arithmetic operation, coefficient growth,
retained byte or cancellation. No performance improvement is claimed; full
repository pre-push qualification remains pending.

**Algebraic similarity section increment (2026-10-27).** On committed
`5de7f0cb05`, native-line queries against the selected quintic scaled by
`sqrt(2)` or translated by `(sqrt(3),sqrt(2))` reproduce
`Intersection/ImplicitPolynomial/Unsupported`. Original source-line controls
already succeed. The polynomial owner now lets the exact inverse similarity
feed the existing native-line section authority. P15 and P17–P20 require the
same original source equation, ordered fiber, parameter domain, selected
endpoints, overlap correspondence and retained image to survive the query.

A nonsingular similarity is bijective: contact and overlap pullbacks preserve
the original source abscissa, and its determinant transports oriented crossing
signs. Removing the rational-coefficient restriction adds no geometric carrier
or elimination path. Exact scalar and selected-field decisions remain required;
their undecided or exhausted outcomes keep their typed meaning.

Seven public witnesses cover the original failures, a seventh-root scale,
oblique maps using both `sqrt(2)` and `sqrt(3)`, reflection and all traversal
orders, tangency, selected trim endpoints, excluded parameters, common line
components, repeated overlap clipping and distinct `2^-80` source shifts.
Independent expanded world equations, exact source parameters and topology
assertions accompany checked in-memory point replay and further intersection.
Reached zero counted work returns exhaustion; a larger retry succeeds, and
STRICT/APPROXIMATE_512/STRICT retains Certified receipts after source drop.

Final checks pass: 2,745 Hypercurve tests (81 suites, 34 existing ignored),
43 root downstream tests and 3,009 tests across six dependent packages
(0 existing ignored). Strict Hypercurve all-target/all-feature MSRV Clippy,
formatting, 16 matrix guards, 98 capability witnesses, typos, repository budget
and diff checks pass. Raw failures, source snapshots and exact commands are in
`docs/evidence/2026-10-27-hypercurve-closure-algebraic-similarities`, SHA-256 `cb775d3b839b79110b7dca2f73d291d91217f19dc11e1707d6d9ca10bdf99612`. All non-receipt source bytes remain identical
after this receipt.

General rational images, different-image polynomial pairs, broader topology
coefficient fields, repeated/vertical components and implicit offset/corner
closure remain open. Point-definition replay here is in memory, not a native
fresh-process archive. This branch excludes the separate region and horizontal
boundary increments. Counted work and external timeouts do not bound total
arithmetic, coefficient growth, retained memory or cancellation. No performance
improvement is claimed; full repository pre-push qualification remains pending.

**Selected-branch composition increment (2026-10-27).** This integration
retains all three qualified parents: polynomial regions (`b6da97710c`), closed
horizontal boundary components (`e49c24719f`) and algebraic similarity sections
(`f2d0071114`). Their individual qualification did not establish composition.
The combined closed-rectangle clip/reconstruct/subtract/intersect witness
reproduced an Unsupported Boolean face decision. Existing exterior probes now
also reuse an available exact coordinate while retaining its selected companion
and original point relations. Complete pair replay and the shared winding
authority still establish incidence and the retained face.

Three composition witnesses preserve both fill rules and traversal directions,
exact source and root identities, checked point replay, a selected quintic
ordinate and its quarter-turn image, disconnected side components, and canonical
empty results consumed by further union. A retained rational identity with
denominator guard `3+x`, followed by an algebraic similarity, preserves contact
coordinates, signs, overlap correspondence and subsequent intersection. The
original failing cases remain unchanged. Adding boundary-regularization probe
directions alone did not resolve the rotated case; that experiment was removed
and its failed results remain archived. The rotated failure occurs in the
APPROXIMATE_512 iteration: polynomial/chord intersection replaced the caller
context with its strict counterpart. It now uses the existing strict-predicate
scope, preserving replay authority while suppressing terminal approximation.
No construction-policy barrier is removed.

Final checks pass: 1,223 Hypersolve tests (24 suites), 2,770 Hypercurve tests
(84 suites, 34 existing ignored), 43 root downstream tests and 3,010 tests
across six dependent packages (0 existing ignored). Strict
all-target/all-feature MSRV Clippy for Hypersolve, Hypercurve and HyperBRep,
formatting, 16 matrix guards, 123 capability witnesses, typos, repository budget
and diff checks pass. Raw attempts and qualification are in `docs/evidence/2026-10-27-hypercurve-closure-selected-composition`,
SHA-256 `309880681d97f5bf8760866eea8009a38f0d1764bb4340f13e136d73e8da4499`. All non-receipt bytes remain identical to the qualified
snapshot after this receipt.

These are bounded composition cases, not general algebraic closure or a new
native region archive format. Broader selected supports, coefficient fields,
nonlinear maps, offsets and corner edits retain their recorded gaps. Counted
work and external fixture timeouts do not establish total arithmetic, expression
growth, retained-memory or cancellation bounds. No performance improvement is
claimed. The separate publication candidate is excluded, and full repository
pre-push qualification of this integration remains pending.

**Selected similarity point policy increment (2026-10-27).** On committed
`2395ba77d8`, two public witnesses produce Certified selected-quintic points
or similarity images under APPROXIMATE_512, then reproduce
`Classification/Unsupported` when the same returned points are queried under
STRICT. Independent STRICT reference constructions already succeed. P4, P8,
P9, P15 and P18–P20 require retained evidence to follow the actual consumed
policy and preserve selected source dependencies through subsequent operations.

The polynomial point and branch similarity construction sites now pass the
existing `retained_object_policy()` result to `BezierSimilarityPoint2::new`.
The generic constructor is unchanged. Locally exact images retain strict
replay authority; incompatible approximate source images remain nested, so a
new local certificate cannot upgrade inherited evidence. Checked point replay
continues to validate the original source definition and selected fiber.

Two public witnesses keep the same returned points across the policy boundary,
verify exact affine relations for the selected root of `y^5+5y+5`, replay point
definitions in memory, and consume them in generated-chord incidence and branch
subdivision. A new negative unit witness seeds an observed consumed-policy
receipt on a parent, constructs a locally Certified child, and verifies that
strict endpoint queries and replay remain blocked. The original generic
similarity policy-barrier regression passes unchanged. All four focused
checks pass; this negative witness tests receipt propagation, not a newly
rounded geometric construction.

Final checks pass: 2,773 Hypercurve tests (85 suites, 34 existing ignored),
43 root downstream tests and 3,010 tests across six dependent packages
(0 ignored). Strict Hypercurve all-target/all-feature MSRV Clippy,
formatting, 16 matrix guards, 125 capability witnesses, typos, repository budget
and diff checks pass. Raw failures, source snapshots and exact commands are in
`docs/evidence/2026-10-27-hypercurve-closure-similarity-policy`, SHA-256 `db95b27ddd9dff1a5f89d3e234a663ea7e66140af57edd77b708c033fb95537c`. All non-receipt source bytes remain identical
after this receipt.

This addresses selected polynomial similarity points, not every retained point
family. General algebraic operation closure and fresh-process native archive
replay remain open. Existing counted limits and external timeouts do not bound
total arithmetic, coefficient growth, retained memory or cancellation. No
performance improvement is claimed. Full repository pre-push qualification
remains pending; checks on the separate earlier publication candidate do not
validate this later source revision.

**Shared algebraic image incidence increment (2026-10-27).** On committed
`4519ec554d`, all eight public pair/chord witnesses reproduce the original
rational-only image restrictions. The composed seventh-root scale also
reproduces `Undecided` in the selected common-fiber solver on that revision.
Earlier prototypes that admitted the common similarity but lacked the solver
fallback exceeded the unchanged public hard case’s 300-second external allowance;
those source snapshots and timeouts are retained.
A mutated root interval is also accepted as field evidence without an actual
selected root. These are P15 and
P17–P20 failures at the curve incidence and selected-field admission boundaries.

Polynomial pairs now compare their shared invertible similarity coefficients
by strict value. Chords admit algebraic inverse coordinate forms and consume
an already proved source-chart support directly, preserving selected endpoints
without a forward/inverse expansion. After an undecided common-fiber GCD,
Hypersolve can reuse its existing projective normalization when every
coefficient ratio has an exact rational proof. A proved nonzero scalar factor
preserves roots and multiplicities; parameter factors are retained. Field and
quotient-ring projection admission recheck point membership or exactly one
owned root in `(lower, upper]`. Uncached singleton endpoints remain supported,
and unavailable selection predicates remain undecided.

Eight curve witnesses cover seventh-root scale/reflection, translation,
independently equal maps, algebraic trims and overlaps, tangent and near-distinct
branches, changed transforms, actual zero-budget exhaustion and retry, policy
transitions, both fill rules, reconstruction, further Booleans, and oblique
chords with selected endpoints. Ten public solver witnesses cover composed
coefficients, signed/transposed equations, nearby distinct equations, vanishing
parameter factors, valid and invalid selectors, singleton membership, and
structural-negation handoff. The first full qualification exposed four valid
singleton regressions and a structural-negation generator regression. Their
original tests and minimized seed remain unchanged; selection replay now lives
at field admission, while structural witness arithmetic keeps its contract.

Final checks pass: 1,233 Hypersolve tests (25 suites, none ignored), 2,781
Hypercurve tests (86 suites, 34 existing ignored), 43 root downstream tests, and
3,605 tests across nine dependent packages. The six curve consumers account
for 3,010 of those tests; additional direct solver consumers are Hyperpath,
Hyperphysics and the standalone exactCore comparison package. Its 19 existing
oracle-defect cases remain ignored. ExactCore source hashes are recorded before
and after execution. Strict Hypersolve/Hypercurve all-target/all-feature MSRV
Clippy, formatting, 16 matrix guards, 133 capability witnesses, typos, repository
budget and diff checks pass. Raw failed attempts, source snapshots and commands
are in `docs/evidence/2026-10-27-hypercurve-closure-algebraic-pair`, SHA-256 `af8f3c08003180038a0db21aa3bbdf5e78fb86a7861c5901c5b43248e2e8703d`. Non-receipt source bytes remain unchanged
after this receipt.

The comparison inventory guard also failed on the unchanged parent because it
omitted HyperBRep I/O and Hyperspatial. Their domain dispositions are now explicit,
with historical audit measurements preserved. Earlier kernel checks retain
their original source attribution: only those two audit metadata files changed.
The complete comparison suite, matrix and hygiene checks ran after that repair;
both failed qualification attempts are archived.

This is a shared-image chart extension and optional projective rationalization,
not a general algorithm for unrelated maps or arbitrary coefficient fields.
Structural root representation validation is not a complete field certificate.
Checked point replay is in memory; fresh-process native interchange and general
algebraic offset/corner closure remain open. Counted limits and external timeouts
do not bound all arithmetic, coefficient growth, memory or cancellation. No
performance improvement is claimed. Full repository pre-push qualification
remains pending for this later increment.

**Selected native source replay increment (2026-10-27).** On committed
`dca6233549`, an authored quadratic restricted at the selected positive root
of `2t^2-1` fails native SVG export. The unchanged library also rejects finite
projection of the retained line range `[1/4,3/4]`, whose independent endpoints
are `(2,0)` and `(6,0)`. Both failures and the library digest are retained.
The owning layers are Hypercurve exact interchange and finite path projection;
the root SVG adapter transports typed failures. P15, P18–P20 and P28–P30 apply.

Version 4 stroke metadata records native sources, selected parameters, original
domains and traversal in dependency order. Selected roots carry their defining
polynomials and isolators; projective charts reference the original selected
root. Replay re-establishes each definition through existing STRICT constructors
before assembling the path. Invalid, forward, cyclic and wrong-kind references
are rejected, including invalid definitions followed by an empty path. Local
allocation keys preserve existing sharing but are never serialized or treated
as mathematical equality. Authored-only paths still emit version 3, and both
versions remain readable. Finite projection consumes the existing retained
source spans through its fragment sampler; display commands never replace the
authoritative metadata. The existing maximum SVG segment count is checked
before publication, and exact replay errors retain their kernel category.

Nine curve witnesses include a separate-process reload followed by exact point
incidence, further restriction, another archive and reversed overlap; shared
degree-seven roots and projective charts; algebraic coefficients; similarity
and replay; rational trims of all eight authored families; invalid selectors,
ranges, references and chart poles; actual byte/segment limits with retry; and
exact finite-projection endpoints and traversal. Two root adapter witnesses
exercise public replay and typed invalid geometry paired with an empty path.
The all-family witness uses rational trims; selected algebraic trim success
is limited to the named cases.

Final checks pass: 2,790 Hypercurve tests (87 suites, 34 existing ignored),
60 root tests across seven affected suites, and 3,010 tests across six
dependent packages, none ignored. Strict Hypercurve/CSGRS all-target/all-feature
MSRV Clippy, the no-default-feature Hypercurve build, both API documentation
builds with warnings denied, formatting, 16 matrix guards, 142 capability
witnesses, typos, repository budget and diff checks pass. Raw failures, exact
commands and source snapshots are in `docs/evidence/2026-10-27-hypercurve-closure-native-replay`,
SHA-256 `14268be5ef2fa70f0f4a0ff8f5879aa52885e5b2c335c3f42cab6b7ce99bf2dd`. All non-receipt source bytes
remain unchanged after this receipt. The initial compilation errors, projection
failures and lint failure are retained; no existing test or hard fixture was
weakened.

This is a per-path authored-source restriction format. Other generated supports,
selected or joint fibers, filled-region native replay, cross-path sharing and
shared scalar-expression DAG serialization remain open. Byte and segment limits
do not bound total arithmetic, coefficient growth, memory or cancellation. No
performance improvement or universal closure is claimed. Full repository
pre-push qualification and published-package delivery remain pending for this
increment; checks on the earlier publication candidate do not validate these
source bytes.

**Procedural parallel replay increment (2026-10-27).** Committed
`8eea456cee` passes the authored-restriction replay control but rejects a
regular parabola parallel during SVG export. Its qualified unchanged library
also rejects finite projection of an ellipse parallel and an exterior `[2,3]`
line-parallel range. All three failures, source attribution, compiler commands
and the library digest are retained. Hypercurve exact interchange and finite
parallel projection own these changes; P15, P18–P20 and P28–P30 apply.

Version 4 adds shared source/normal-distance supports and selected finite range
records. Replay uses existing strict constructors to re-establish roots,
ascending ranges, denominator and normal guards, and interior cusp exclusions.
Every finite range is checked even when unused by an empty final path. Existing
version 3 and version 4 records remain supported. Local immutable allocation
keys retain sharing but are never serialized or treated as mathematical
equality. Explicit range order matters: the generic curve-range constructor
admits traversal in either direction and cannot certify an ascending domain.

Finite projection now evaluates interval bounds on the original retained range,
including exterior domains, and subdivides inconclusive Ordering enclosures
under the existing depth bound. Every accepted chord still needs a certified
box. Other error outcomes remain explicit, and display points never replace the
native source. A pole inside a retained rational range preserves the existing
typed Boundary blocker; it is not relabeled as invalid or counted as completion.

Nine curve witnesses cover parabola, nonrational ellipse, PH cubic and stationary
zero-distance sources with independently derived exact midpoint expectations;
shared cusp boundaries; separate-process reload before region admission,
Booleans and round offset; invalid references and singular/nonascending ranges;
rational pole guards and exterior domains; exact finite line-parallel endpoints;
reached byte limits and retry; algebraic trim, reflected algebraic similarity
and another archive; and distinct distances sharing one source. One root SVG
adapter witness consumes the replayed result in exact point incidence. The cap
fixture checks specified exact relations and operation consumption; its finite
membership assertions do not independently prove the entire expanded set.

Final checks pass: 2,799 Hypercurve tests (88 suites, 34 existing ignored),
61 root tests across eight affected suites, and 3,010 tests across six
dependent packages, none ignored. Strict Hypercurve/CSGRS all-target/all-feature
MSRV Clippy, the no-default-feature Hypercurve build, both API documentation
builds with warnings denied, formatting, 16 matrix guards, 151 capability
witnesses, typos, repository budget and diff checks pass. Raw failures, exact
commands and frozen source snapshots are in `docs/evidence/2026-10-27-hypercurve-closure-parallel-replay`,
SHA-256 `5b0a3f0c28cb0a70f0779f98ab2da108b2078fe5423008e50a92cd37f47f54fe`. All non-receipt source bytes
remain unchanged after this receipt. Failed compilation attempts, the original
projection failures and the descending-range replay failure remain available;
no existing test or hard fixture was weakened. The first downstream build failed
before tests when ccache attempted read-only storage. Its logs are retained;
a task-owned cache resolved the environment failure, and the remaining twelve
groups resumed on unchanged source. The first five passed groups keep their
original command and environment attribution.

Other generated supports, selected or joint fibers, filled-region archives,
cross-path sharing and scalar-expression DAG serialization remain open. Existing
byte, segment and subdivision-depth limits do not bound total arithmetic,
coefficient growth, temporary/retained memory or cancellation. No performance
improvement or universal closure is claimed. Full repository pre-push checks
and published-package delivery remain pending for this increment; the earlier
pushed candidate does not qualify these source bytes.

**Selected chord replay increment (2026-10-27).** Committed
`2eb55c5dd5` passes procedural-parallel export but rejects a degree-seven
selected chord after exact construction, midpoint incidence and split. On its
unchanged qualified library, exact incidence queries also change the same path's
finite projection from 82 to 83 points at tolerance 0.01. The original programs,
commands, library digest and source attribution are retained. Hypercurve chord
and point definitions, native SVG interchange and finite projection own this
increment; P15, P18–P20, P27 and P28–P30 apply.

Version 4 adds shared point, supporting-chord, selected-range and curve-endpoint
records. Replay covers exact coordinates, rational source images, rational
expressions and native parallel evaluations at Bezier scalar or selected-root
parameters. The chord owner proves support incidence and strict coordinate
order, then retains the supplied chart and directed endpoints. Valid exterior
ranges and alternative charts survive. The original support remains distinct
from its finite selection; a storage reversal flag alone cannot recover a
split reversed chord's chart. Invalid unused definitions still fail before an
empty final path. Local address keys retain their immutable owners and never
become serialized identities or mathematical equality.

A prior-curve endpoint reference preserves known incidence. Projection uses the
requested analytic-point refinement independently of stronger cached predicate
bounds, including through lazy endpoints. Exact predicates retain their existing
cache. No tolerance, source definition or acceptance assertion was weakened.
The new graph tags, shared retained-curve records and affected display bytes are
an explicit serialization migration in `docs/MIGRATIONS.md`; existing version 3
and version 4 inputs remain supported.

Nine public curve witnesses cover separate-process degree-seven replay before
incidence, overlap, reversal, split and another archive; algebraic-coordinate
region construction and round offset; hand-built chart and exterior-range
records; invalid unused supports and references; correlated rational-expression
endpoints; mixed parallel/chord source and selector sharing with independent
exact coordinates; reached byte limits and retry; original denominator poles;
and mixed native/generated boundaries with a circular hole and disconnected
material through reconstruction, Boolean identities and offset. A direct unit
witness retains the original cold/warm projection failure and checks
coarse/fine/coarse requests. One root SVG adapter test consumes a reloaded
selected chord in split and a second archive. Finite point expectations alone
do not prove the entire intended offset set.

All 17 final groups pass: 2,809 Hypercurve tests across 89 suites, retaining 34
existing ignored cases; 62 root tests across nine affected suites; 3,010 tests
across six dependent packages; strict all-target/all-feature Hypercurve/CSGRS
MSRV Clippy; a Hypercurve build without optional features; both API documentation
builds with warnings denied; formatting; 16 matrix guards; 160 capability
witnesses; typos, repository budget and diff checks. Evidence is in `docs/evidence/2026-10-27-hypercurve-closure-chord-replay`,
SHA-256 `79716febf65ec101df08a8ed67ed3d5411e2b003c616c4a02acc5c04575e1d67`. Non-receipt source bytes
are unchanged after qualification. The initial mistaken point-variant setup
assertion is retained but not counted as a geometry failure. Failed prototypes
preserve the original SVG assertion; the curve endpoint reference alone did not
fix it, and the independent parent probe identified the bounds-cache cause. The initial
mixed-topology setup mistook the native-circle wire for a missing path; its
recovery consumes both exact carriers and retains every geometric assertion.

Recursive point definitions, selected joint fibers, auxiliary chord contacts,
generated circles, filled-region archives, scalar-expression DAGs and cross-path
sharing remain open. The projection regression qualifies the named analytic
endpoint domain, not canonical output for every equivalent parameterization.
Byte, segment and depth limits do not bound all arithmetic, coefficient growth,
expression size, temporary/retained memory or cancellation. Metadata growth is
observed only for the named repeated-piece fixture; no general performance or
closure claim follows. Full repository pre-push checks and published-package
delivery remain pending for this increment.

**Selected point graph replay increment (2026-10-27).** Committed
`78c661a8ae` exports a selected chord but rejects both the retained line
and complete circular fillet from a degree-seven selected corner. The original
positive assertions, exact cut expectations, commands, qualified-library digest
and committed-source attribution remain in the evidence. This increment resolves
the retained line. The unchanged complete-circle assertion still fails with an
unsupported generated-curve definition. Hypercurve point construction and native
SVG replay own the geometric work; Hyperbrep owns its planar-point archive and
record admission. P15, P18–P20, P27 and P28–P30 apply.

Version 4 adds polynomial-fiber, rational-image, exact-similarity and unique
supporting-line-intersection point definitions, plus shared chord-displacement
recipes and endpoint selections. Replay retains the correlated base parameter,
re-proves the selected fiber, checks original denominator guards and reconstructs
similarities and unique intersections. Chord displacements preserve the directed
source, signed tangent or left-normal distance, optional origin and exact
translation. A native line or selected chord supplies the original support;
independent scalar materialization is unnecessary. The existing owner validates
the construction, including inherited policy. Invalid unused definitions fail.

The graph writer visits point dependencies iteratively. Local address keys own
their immutable inputs and do not define mathematical equality. Reusing shared
map prefixes emits 14, 26 and 50 native records for the named 4, 8 and 16-span
fixtures; these measurements concern serialization growth, not total memory or
speed. A new configurable point-definition depth limit is propagated through
chord and endpoint indirection. Byte and depth exhaustion are typed and permit a
larger-budget retry. The decoder reuses only prior checked immutable definitions;
public point reconstruction continues to validate its dependencies.

Hyperbrep's new planar-point variant delegates to the same geometry owner. Its
resource pass counts the source and optional origin before scalar parsing. It
retains the existing nested point format. Explicit native-line and selected-chord
source syntax both replay. The API, SVG tags and archive variant are documented
in `docs/MIGRATIONS.md`; existing inputs remain supported. The SVG fuzz target
now exercises both metadata versions under its existing input limits.

Ten public Hypercurve witnesses cover fresh-process replay of the original
fillet-trimmed line before exact midpoint splitting and another archive;
correlated fiber/map/algebraic-similarity relations; independent selected
supporting lines and exterior intersections; invalid references, selectors,
matrices and original poles; reached depth and byte limits with retry; shared
map prefixes; signed tangent and normal displacements with source reversal,
zero distance, arbitrary origin and translation; and shared endpoint recipes.
A unit witness checks inherited approximate-policy rejection and strict recovery.
One CSGRS SVG test repeats the trimmed-line composition. Two public Hyperbrep
archive tests preserve exact displaced points and the original selected fillet
cut through reload and further split, with deterministic serialization checked
separately from those geometric expectations.

All 18 final groups pass: 2,820 Hypercurve tests across 90 suites, retaining 34
existing ignored cases; 63 root tests across ten affected suites; 3,012 tests
across six dependent packages; strict all-target/all-feature Hypercurve,
Hyperbrep and CSGRS MSRV Clippy; Hypercurve and Hyperbrep builds without optional
features; all three API documentation builds with warnings denied; fuzz-target
Clippy; formatting; 16 matrix guards; 170 capability witnesses; typos, repository
budget and diff checks. Evidence is in `docs/evidence/2026-10-27-hypercurve-closure-point-replay`, SHA-256
`e5473caf511936bc1704e8d8ed1251b73bf1e2b17ab078623375a2d8b08f932b`. All non-receipt source bytes remain
identical after qualification. Failed prototypes and their source snapshots are
retained. Those records distinguish construction/codec gaps from invalid fixture
setup, compile failures and filesystem exhaustion. The native-source fixture
now uses its intended native constructor without dropping its view assertion,
exact point expectations or budget checks.

Generated circles, auxiliary contact and other cusp-derived definitions,
general joint parameter charts, broader generated and filled-region archives,
scalar-expression DAGs and cross-path sharing remain open. The finite corpus is
not a universal closure proof. Point nesting, bytes and existing archive limits
do not bound all arithmetic, coefficient growth, native prefix-vector allocation,
temporary/retained memory or cancellation. Full repository pre-push checks and
published-package delivery remain pending for this increment.

**Selected circle frame replay increment (2026-10-27).** The preceding
point-graph increment `83fd8c03b0` can replay the line cut from the original
degree-seven fillet, while the complete generated circle still fails export.
The unchanged-source baseline and its original positive assertion are retained.
A separate baseline also rejects the independent retained circle point before
serialization. Hypercurve owns circle frames, selected angular domains, exact
point incidence and retained chord directions; native SVG and the CSGRS adapter
consume those owners. P15, P18–P20, P27 and P28–P30 apply.

Version 4 now preserves chord-normal circle frames with their exact center,
directed source chord, nonzero signed radius and orientation. Scalar angular
parameters, signed normal contacts and independently incident points select
finite ascending ranges with explicit traversal. The original point and shared
source definitions remain authoritative; there is no coordinate fitting or
forced independent scalar materialization. Replay proves the literal signed
normal relation, radius, incident-point equation and selected half. Every unused
record is checked, including typed references and cross-circle parameter owners.

Transported oblique chord frames retain a represented unit-direction witness.
The chord owner proves unit length, perpendicular equality and positive directed
displacement before retaining it. Supports and finite chord ranges remain
distinct. Represented endpoint directions already reconstructed by the ordinary
constructor keep their existing wire form. This restores the construction fact
needed for deterministic projection after algebraic similarities. Construction
tangent flags are not serialized; the retained normal relations and existing
arrangement predicates re-establish applicable contacts. A changed-neighbor
witness confirms both transverse roots instead of inheriting tangency.

Eleven public curve witnesses cover the complete original fillet in a fresh
process before exact incidence, splitting, reversal and a second archive;
algebraic rotation/scaling and reflection; region admission, all four empty-set
Boolean operations and boundary reconstruction; mixed native/generated boundaries
with a hole and disconnected material; two changed-neighbor crossings in both
orders; invalid unit directions and signed contacts; finite-arc clipping and
`2^-80` separation; malformed frames, selectors and ranges; and reached depth
and byte limits followed by larger retries. A unit witness checks consumed
approximate-policy barriers. The CSGRS SVG adapter repeats complete circle
replay followed by split, reversal and another archive. Exact expectations are
checked separately from deterministic complete SVG bytes.

All 18 final groups pass: 2,832 Hypercurve tests across 91 suites, with the same
34 existing ignored cases; 64 root tests across 11 affected suites; 3,012 tests
across six dependent packages; strict all-target/all-feature Hypercurve,
Hyperbrep and CSGRS MSRV Clippy; builds without optional features; API docs with
warnings denied; fuzz Clippy; formatting; 16 matrix guards; 181 capability
witnesses; typos, repository budget and diff checks. Evidence is in `docs/evidence/2026-10-27-hypercurve-closure-circle-replay`,
SHA-256 `f0d529e9c2ca5cb332cd43e9ae22f01c48bf6b665b2f5fddc946b2742631413d`. All non-receipt bytes remain
identical after qualification. A host filesystem quota truncated extraction of
a required Gerber fixture during the first consumer attempt. Its positive
coverage assertion failed; the full consumer command was rerun successfully on
unchanged source after storage recovery. No fixture or assertion was changed.
Original failures and every prototype source
snapshot are retained, including the selected angular-record gap, missing unit
direction evidence, support/range reference distinction, unchanged older chart
assertion and proved-invalid contact classification. No hard geometry assertion
was replaced by a blocker expectation.

Other generated circle frames, general mapped contacts, repeated-offset angular
transport and filled-region native archives remain required work. A separate
correlated degree-seven/five fiber-center probe imports and evaluates exact
endpoints, but its independently mapped midpoint returns `Blocked(Ordering)`.
That preserved positive failure is an implementation gap, not exhausted work or
completed support. This finite corpus is not a universal algebraic closure proof.
Frame dependency depth and extension bytes do not bound total solver work,
coefficient or scalar-expression growth, temporary/retained memory or cooperative
cancellation. Full pre-push qualification and published-package delivery remain
pending for this increment.

**Correlated point and circle predicate increment (2026-10-27).** The
preceding selected-circle increment `32b114ca03` retains a probe whose
correlated degree-seven/five center imports and evaluates exact endpoints, but
whose midpoint query returns `Blocked(Ordering)`. A fresh unchanged-source build
reproduces that original positive assertion. The existing rational-query API
already proves the radius squared exactly one; oblique affine comparison also
fails before this change. Hypercurve's common point-field authority, circle
incidence and half chart, and chord-direction replay own these changes under
P15, P18–P20, P27, P29 and P30.

Public rational-query arithmetic and geometric predicates now share the same
checked source pullback and common-field alignment. Both selected coordinates
retain their base/fiber relation. Oblique projections subtract before independent
coordinate refinement; circle residuals use nonzero checked denominators and a
positive squared denominator. Original map and operand guards remain owned.
When a certified incident point's displacement reduces to exact constants in a
represented normal frame, the existing rational half-chart inverse consumes
those displacements directly. Signed radii, traversal, diameter endpoints and
complementary-half ownership keep their established meaning. This is not an
algorithm for every independent selected angular field.

A reflected oblique-frame witness also reproduces loss of a compact rational
unit direction during native replay: geometry, metadata and viewBox agree, while
one display coordinate changes after reconstruction through an equal radical
expression. Source/cold/warm probes distinguish this from query mutation. The
existing checked unit-direction record is now omitted only when the ordinary
chord owner reconstructs identical scalar definitions. Readers retain validated
incoming components even when another vector is equal in value. Exact length,
parallelism and orientation predicates still establish validity; byte identity
only decides whether an encoding can omit a definition. No new wire tag or
format version is introduced.

Seven maintained witnesses cover the original selected center without independent
coordinates; exact oblique dot products in both operand orders; midpoint location
and evaluation followed by subdivision, reversal and another native archive;
both signed radii and traversal directions, diameter endpoints, quarter points,
excluded halves and `2^-80` separation; negative denominator maps and original
zero-pole rejection; algebraic similarities and reflected complete SVG replay;
and actually reached zero work budgets followed by successful 512-refinement,
zero-promotion retries. Strict/approximate/strict requests remain Certified when
these exact proofs suffice. Exact geometry and deterministic serialization are
separate assertions. The original external positive program succeeds unchanged,
and the earlier replay corpus remains intact.

All 18 final groups pass: 2,839 Hypercurve tests across 92 suites with the same
34 existing ignored cases, 64 affected root tests across 11 suites, 3,012 tests
across six dependent packages, strict MSRV Clippy for all targets/features of
Hypercurve/CSGRS/Hyperbrep, builds without optional features, denied-warning API
docs and fuzz Clippy, formatting, 16 matrix guards, 188 capability witnesses,
typos, repository budget and diff checks. Raw evidence is `docs/evidence/2026-10-27-hypercurve-closure-fiber-point-order`,
SHA-256 `6f36bec9d2a26a7b7830fdd268ede65c1260e6bfd538218d3f818efe7c43f08a`. The qualification ran on a frozen complete source snapshot
while the preceding increment completed its independent checks. Afterwards,
reparenting imported only the three parent receipts and proved all other source
bytes unchanged. Both the full qualified patch and the final parent-relative
patch are retained; receipt checks follow the final ledger edits.

A host filesystem quota interrupted one Hypercircuit command and its report.
The truncated attempt is retained without inferring success. After verified
storage recovery, the complete command and all remaining groups ran on unchanged
source; the first nine complete source-attributed records remain intact.

Original failures remain available, including the initial harness search-path
error, the distinct angular-inversion blocker after incidence was fixed, and
the reflected display assertion. General independent fields, nonconstant angular
inversion, other generated circle frames, repeated-offset chart transport and
filled-region native archives remain open. Counted refinements/promotions do
not bound total arithmetic, coefficient growth, expression size, heap use or
cancellation. A finite corpus does not prove universal closure or performance.
Full repository pre-push qualification and published-package delivery remain
pending for these bytes.

**Open, in dependency order:**
- **Split cost, remaining contact reuse.** R05 (2026-10-25) tested evaluating
  each cut once and sharing its selected point between adjacent pieces. The
  measured quintic split improved, but the section control regressed. The
  candidate was removed under the handoff's literal latency gate; its exact
  rows and measurements remain in [the experiment record](PERFORMANCE.md#shared-polynomial-cut-points-r05).
  Passing the original intersection contact's authority into subdivision
  remains a distinct experiment.
- **Broader common components.** Qualified polynomial pairs now retain selected
  common nonlinear supports and residual/mixed contacts in a shared source or
  rational similarity chart. Different images, broader coefficient fields and
  repeated/vertical topology components still need their own acceptance work.
- **Horizontal-line replay.** On a horizontal line the line parameter is the
  branch abscissa's own root, so a returned line piece's endpoint is a line
  point that is proved equal to, not stored as, the branch point. A
  selected-fiber image of the abscissa would keep the replay identity at the
  cost of fiber-field arithmetic.
- **Selected abscissas and broader maps (H06, milestones 2 and 3).** Native-line
  sections now consume retained algebraic similarities and checked rational-map
  similarities where the original exact source decisions complete. Their
  composition has a separate witness. General selected chord supports, nonlinear
  rational images, further pair images and selected coefficient fields still
  need their own source-domain and incidence proofs; source-x monotonicity is
  not world-x monotonicity.
- **Repeated and vertical components (H07).** See `hypersolve/FUTURE_WORK.md`.
- **Complete implicit regions and BRep trims (H08).** The unchanged H08 cap
  now passes loop orientation and shared `CurveRegion2` admission. The bounded
  region corpus exercises retained graph supports, holes, disconnected and
  touching material, mixed circles, exact trims and later operations. This
  does not complete arbitrary implicit branches, coefficient fields, singular
  sectors, image maps or BRep trimming. Nonzero polynomial-support offsets,
  corner edits, finite projection and fresh-process native replay still need
  their own acceptance work. Source/root/domain and contact evidence must
  remain available throughout those extensions.

## 9. Selected-root comparison, fillet centres and tangent events (2026-10-23)

Items H09–H12 of the 2026-10-22 research queue, rechecked at `916e1e5f41`. `PERFORMANCE.md` records the measurements.

**Done: overlapping local polynomial roots (H10).** `cmp_by_refinement` orders two recursive roots whose isolators still overlap after 32 bisection steps by one sign of either defining polynomial at the other root (`order_against_polynomial_root`). Before, the 64 such comparisons in the release suite all ran 512 more bisection steps and then promoted both roots to global parameters, and a pair 2^-603 apart stayed undecided. A recursive root also keeps native algebraic roots it has been proved equal to. Together these let `chamfered_quadratic_bevel_stroke_round_trips` complete in about a minute; it did not finish in forty.

**Equivalent-field replay milestone (R15, 2026-10-25).** Reproduction located all
12 mixed formula/polynomial fallbacks in the selected-field chord-overlap test.
Their coefficient fields have equivalent selected sources but distinct
allocations, so ancestor lifting declines. After that decline, the formula path
now uses the existing equivalent-base/tower embeddings and homogeneous
evaluation with its certified positive denominator. The twelve comparisons
decide locally. Independent tests also order the roots of `f` and `f - 2^-600`,
which the previous full comparison leaves undecided. Different sources and
unproved source permutations retain the old fallback; no source union was added.
The [cost record](PERFORMANCE.md#equivalent-field-root-comparisons-r15-2026-10-25)
reports the targeted improvement and adverse controls. Shared singleton
certificate factoring, general field joins, cold archive continuation and total
work/memory/cancellation guarantees remain open. Combined primary validation
is recorded in the research ledger.

**Done: equality with a high-degree eliminant root (H14, 2026-10-23).** The remaining time of the chamfered case was one 47.6 s proof that a degree-6 local root over a two-source, one-extension tower equals a root of the degree-192 polynomial the chord/parallel system projects for its candidates (`parameter_is_in_ordered_range`). The proof is the sign-preserving remainder of the eliminant by the local relation, which is identically zero; no gcd is computed. Hypersolve now builds that remainder along the eliminant's Horner sequence and reduces tower products without a content gcd, and the proof takes about 1.4 s; the case takes about 15 s. A modular gcd would not shorten a zero remainder. A candidate filter that does not first project the system to degree 192 remains a possible further step. Promoting the local root first and comparing the two native roots is no faster (62–82 s per equality; rejected).

**Not implemented: two-sided field joins (H09).** `recursive_polynomial_sign_joined` returns `Unsupported` when a query's coefficient field has a selected source that the root's base lacks; there is no fallback inside it. The internal regression (`t^2 = 2 + sqrt 2` in the root's field; `beta = sqrt 3` in an independently built field, in both source orders) reproduces this and is kept as the ignored test `recursive_polynomial_root_signs_queries_over_an_independent_source`. A join over the union of both source tuples, factored from `recursive_merge_projective_point_fields`, decides all of its queries locally. With route tracing over the full release suite and the four open-path workflows, the only queries that reached the missing-source case came from that test, so the join was not kept. Production integration remains route-gated. The 25 October experimental charter separately authorizes a distinct advanced design on selected/mapped CAD/CAM fixtures without requiring an existing production caller; the tested variant's negative result remains a control.

**Not the cost: centre promotion in the selected parallel-normal kernel (H11).** In `round_stroke_of_a_filleted_nurbs_pair_completes`, the fillet centre reaching `selected_parallel_normal_parallel_intersections` is already a `BezierParameter2` of degree 214, so its promotion there is a no-op. Of five calls, four took 0.35 s, 12.1 s, 0.36 s and 110 s, and the fifth had not finished at the 40-minute limit; the time is in selected-fiber isolation over the degree-214 field (`LocalAlgebraicField::reduce`, local subresultants and Sturm sequences). Widening the consumer to `CurveParameter2` does not change this; authoring the centre as a low-degree selected fiber does (section 2). Its selected-factor degree is 164 (H15, section 2).

**Open: tangent events (H12).**
- `refilleting_a_filleted_vertex_completes`: the second fillet signs point-incidence predicates at recursive roots through `dense_two_positive_square_root_sum_sign` (section 1). It did not finish in forty minutes; with integer-form enclosures in the certified leaf refinement it completes in three to five minutes (H13).
- `round_stroke_boundary_of_a_filleted_arc_corner_refillets_and_erodes`: fillet, round stroke, boundary reconstruction, reversal and a second fillet all complete; eroding the result then blocks with `RealSign`. At a tangent junction left by the second fillet, one offset span keeps an analytic parallel's retained tangent and the other the fillet arc's represented tangent. Their cross product is zero, but neither it nor the equality of the two offset ends is decided. The fillet certified that tangency when it was built; the rebuilt region does not keep it. The same workflow from a line/line corner completes (`round_stroke_boundary_of_a_filleted_line_corner_refillets_and_erodes`). Either the boundary keeps the tangency certificate or `curve_tangent_cross_sign` proves this zero exactly.

## 10. Box containment and CAM-derived contracts

**Admitted.** `Curve2::box_containment` and `CurvePath2::box_containment`
decide whether a curve image lies in a closed axis-aligned box, with an exact
witness point when it does not (`PERFORMANCE.md`, 2026-10-23). The cached
`bounds()` enclosure is the fast path; unsettled sides use the exact extremum
candidates of lines, arcs, polynomial quadratic and cubic Béziers, rational
quadratics and spline spans of those kinds.

The R23 implementation extends the query to nonmonotone rational spans, including
degree-four and NURBS inputs, and retained rational/source-range trims within
the unit source chart. Quotient-derivative roots are filtered by exact active-
range membership before their coordinates are compared. The denominator proof
belongs to the active range, so a source pole outside a valid trim is harmless.
Native, selected, transformed and periodic fixtures include exact tangency,
`2^-200` clearance, source-parameter replay, source drop and budget retry.
The owner and Alumina consumer gates pass. This is a completion improvement;
existing-query costs and remaining blockers are recorded in `PERFORMANCE.md`.

**Open:**
- **Broader derivative domains.** Retained extensions outside the unit source
  chart and analytic-parallel or implicit-polynomial interior extrema need their
  own complete candidate enumeration. Symbolic root-isolation/ordering blockers
  retain their typed meaning; no new internal total-work or heap bound is claimed.
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

**Not changed: the degree-elevation cache (H16).** A kept source retains every intermediate degree: 1.2–1.9 MB at degree 64, against 42–207 KB for the target alone. The direct formula gives the same exact controls and is 2–17 times faster for one cold request of degree 32 or 64, but 1.7–8.4 times slower when degrees are requested in turn, and no real path elevates beyond degree 5. Revisit with bounded checkpoints if a caller requests high degrees.

**Not changed: shared Boolean selection views (H17).** No board makes a nontrivial `boolean_regions` call, and the fragment walk, cloning and graph construction that views would share are at most 2 of the 39–52 samples of the per-operation stage in the all-four bench. The output-dependent costs of that stage are open leads that also apply to single Booleans:
- the coalescing pass asks every kept arc fragment for an exact line image (`retained_line_fragment_endpoints`, `fit_exact_line_image_with_policy`): half of the stage for two circles. A fragment of a certified circular conic carrier is never a line;
- `retained_loops_have_pairwise_disjoint_bounds` (34% of the stage on nano) and line-run merging (55%).

R11 (2026-10-25) tested retaining certified roles through strict, indexed
directed-image coalescing. Repeated queries improved with identical exact rows
and construction allocations, but control latency nonregression remained
unproved. The candidate was removed; see
[the inconclusive experiment](PERFORMANCE.md#retained-loop-roles-through-line-coalescing-r11).
Line-fit reuse between coalescing and all-line compaction, and the remaining
cold role proof, are separate open experiments.

**Not applicable: span-pair bounds (H19).** Region carriers are single spans, so region Booleans never form a span Cartesian product. Direct intersection of multi-span curves does, and already rejects certified-disjoint span pairs before building candidates: two 64-span cubic B-splines with 66 contacts intersect in 69 ms with 3.6 MB peak transient.

**Open: retained scalar caches (H20/R17).** The earlier H20 report projection
held 87.6 MiB. Its allocation-site attribution put Hypercurve structures at
about 12%, exact rationals at 30%, Hyperreal rational derived forms at 17% and
`Computable` evaluation cells at 9%. The inclusive π-Laurent category (21%)
overlaps those origins. Its pipeline peaks were mostly transient work
(201–308 MiB against 108–170 MiB live at the end).

R17 reproduces current ownership with a complete archived trace and a separate
cache census. A smaller rational-cache layout lowers Nano memory but fails its
protected native latency gate, so only regression tests are retained. Full
pipeline post-drop samples do not establish a byte bound. See
[Hyperreal's owner-specific evidence](../hyperreal/PERFORMANCE.md#retained-scalar-ownership-and-rejected-layout-r17-2026-10-25).

Any scalar retention change belongs in Hyperreal. The rejected per-call memo
variants under "Closed normal-form memo sharing" in `PERFORMANCE.md` do not
answer what outlives a call. In H20's small-region probe, query caches for exact
signed area, boundary path views and classification indexes added 27–240% to a
queried region; recomputing them repeats the same exact work, so no eviction was
implemented.

## 12. Consumers of approximately closed loops (2026-10-23)

Found while checking receipt composition (item A01; the root `PERFORMANCE.md` has the record). The fixture is a unit square path whose last vertex is the first one translated by `(z, 0)`, where `z = sin(e + 1) - sin(e) cos(1) - cos(e) sin(1)` is an exact zero that strict predicates cannot decide. Strict `try_from_boundary_paths` returns `Blocked(RealSign)`; inside `provisional` it succeeds and consumes the terminal. Its consumers then behave as follows. No wrong certified answer was found.

- **Replay diagnosis corrected in the R08 milestone.** Retained chord and
  selected-fiber validators share a private admissibility check and return
  `IncompatiblePredicatePolicy`. Principal classification, Boolean and affine
  operations on this fixture report `Blocked(Predicate)` under strict replay.
  This changes the failure category, not the accepted policy pairs. Other
  representation-specific replay guards remain to be audited.
- **Affine continuation restored; native line-profile extraction remains open.**
  A transform that consumed terminal evidence now keeps its authoritative
  boundary without forcing an optional strictly closed native contour cache.
  Translation, reflection and scaling continue under the original policy,
  with qualified receipts and strict replay still blocked. Extrusion continues
  to fail at native line-string connection. An isolated prototype replayed the
  closure and bypassed that check, but native contour/string views have no
  retained closure-policy field; that broader change was not adopted. The
  remaining design must bind qualification to the source without promoting
  closure to structural equality.
- **Finite projection reports `Certified` under both policies.** It projects the retained loop as built; the closure decision belongs to the construction, as the root's operation-local receipts document.

A region produced by an approximate union of two squares that touch up to `z`
has structural loop closure and remains a separate control. Hypercircuit now
offers outcome methods for its five explicit-policy board queries; the original
methods remain compatibility paths without a receipt. Cold archive/rebuild and
native qualified-contour consumption remain open R08 acceptance work.

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

## Guarded circular line fits (R28, 2026-10-25)

The private positive-radius/finite-connected/nonconstant circle shortcut is
rejected under the literal latency gate. Thirty balanced native pairs preserve
all 5,760 outcome rows and improve warm circle queries, but five simultaneous
protected intervals regress on ordinary point/line cases. Allocation reductions
and instrumented Nano dispatch counts do not override this veto. See
[`PERFORMANCE.md`](PERFORMANCE.md#guarded-circular-line-fit-rejection-r28-2026-10-25)
for timing boundaries, adverse controls and all evidence limitations.

The experiment also confirms that `NotLine` is not a nonlinear-image proof:
coincident endpoints return it even for a constant point. Its description is
corrected, and seven independent regression tests preserve circle restrictions,
collapsed point fitting, retracing, singular/mixed-weight chart declines,
similarities, tiny native arcs and policy qualification. No runtime shortcut is
retained. General similarity currently drops circular provenance and therefore
uses the generic path; the tests do not claim shortcut coverage there.

Reopen optimization only with a changed premise, such as producer-owned proof
or a dispatch shape that protects ordinary fit costs. The next candidate must
repeat protected direct-query checks and then complete full circle/board
geometry, provenance, receipt, continuation and resource gates. The current
single-board diagnostic run skips the all-board golden comparison and reports
six release blockers. Broader H17 selection APIs remain a separate experiment.
