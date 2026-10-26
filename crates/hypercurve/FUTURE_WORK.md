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
meet authored lines with rational endpoints, or returned restrictions of them,
in both intersection APIs and both operand orders. Parameters remain in their
original source charts. Both operands split and the pieces re-enter the same
query; selected algebraic trims, reversal, tangent contacts and singular-event
contacts retain their meanings. Other algebraic similarities, nonlinear maps,
algebraic-coordinate lines, branch pairs and filled polynomial arrangements
remain implementation gaps.

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

**Open, in dependency order:**
- **Split cost, remaining contact reuse.** R05 (2026-10-25) tested evaluating
  each cut once and sharing its selected point between adjacent pieces. The
  measured quintic split improved, but the section control regressed. The
  candidate was removed under the handoff's literal latency gate; its exact
  rows and measurements remain in [the experiment record](PERFORMANCE.md#shared-polynomial-cut-points-r05).
  Passing the original intersection contact's authority into subdivision
  remains a distinct experiment.
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
- **Selected abscissas and broader maps (H06, milestones 2 and 3).** Rational
  similarity pullback is admitted above. Irrational line coordinates and
  general algebraic similarities or nonlinear rational images still need
  their own section and source-domain proof; source-x monotonicity is not
  world-x monotonicity. Branch pairs need resultant events and exact fiber
  signs.
- **Repeated and vertical components (H07).** See `hypersolve/FUTURE_WORK.md`.
- **Filled regions and BRep trims (H08).** `CurveRegion2` construction and
  `CurvePath2::simple_loop_orientation` with polynomial boundary pieces return
  `Blocked(Construction, ImplicitPolynomial, Unsupported)`; a test records
  this. Source-span and boundary ownership, pair incidence, point location,
  local sectors and BRep consumers must all retain the graph, selector and
  domain. Compare the chart-qualified views and shared incidence of H01–H04
  with narrow family services; they are exploratory alternatives, not required
  predecessors. The 25 October handoff stages actual pair/component capability
  in R05–R07 and explicitly permits advanced API prototypes.

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
