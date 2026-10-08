<!-- current-continuation -->
Current continuation: Hypercurve2ff2b956a521e6e3baf778255a0abd92d18b4d00 is
committed/sealed. V489 outer95767 and review15317 are reaped0:349 selected release
tests,8 binaries,6 checks,2047 source hashes and4 staged/committed files verified;
30 repositories were clean. Both full re-offset cases pass113.46/113.41s with no
material timing regression. New24 regressions cover stationary contacts,
reversal/modes/policies, wrong-side exclusion, arbitrary exact irrational
coefficients, region/Boolean replay and further exact round offsets.
V492 outer33147 is reaped1: replacing the native line by an equivalent quadratic
line chart still blocks the stationary reparameterization and interior-contact
cases. A retained regular range also wrongly rejects ancestral stationarity
outside that range (both zero and nonzero offset carriers). All6 radius-only
finite fillets pass. These4 failures are immediate, not timeouts.
V493 outer66902 is reaped0: the new two-file production patch fixes all4 retained
range policy tests (0.004s zero-distance;0.164/0.214s nonzero). It makes finite
range explicit in supporting_line_incidence_with_direction and only builds the
original-source parallel on demand. src/curve.rs andsrc/bezier_offset.rs remain
uncommitted pending broader checks. Mirror/archive V493 match production. No
owned process remains. The quadratic-line pair failures remain unresolved.
V494 outer15293 is reaped0: the4 range tests cover16 policy/direction/mode
configurations and pass. They are promoted into the existing stationary-fillet
test file. V495 final qualification is running as exact outer74084, targeting
353 release tests and6 checks on2047 frozen files. Do not edit production,
archive, mirror, driver or live fixture inputs, start another driver, stage or
commit before reaping that exact outer. V495 review/seal scripts are prepared
only. Next unallocated artifactV496. Full implementation goal remains active.
<!-- /current-continuation -->

## Latest continuation: Hyperreal 320bfe90, wide exact-field proof reuse

Hypercurve remains `1bd0348bb7b848e85f9085932a30033675f59e2f`; Hypersolve is `4ba7f2508b8e38f60e66543af688a885cfaa6a8e`; Hyperreal is now `320bfe90edb8c0902f0f8b8985f7223c6a31afb7` (on b1beafe7). Scalar field arithmetic, cached proofs and compaction no longer have a coefficient-size gate. The explicit binary-shift guard measures newly allocated expansion, preserving small shifts of wide values. All callers retain the existing exact Real interface.

V374 passes 2,059 distinct runtime tests and nine static/downstream checks; 2,046 input hashes, nine main binaries, index and commit bytes verified. Outer 49937 is terminal/reaped; all 30 repositories clean. The repaired parallel-loop chamfer remains passing at 114.03 seconds. See implementation.md for the independent V366 cancellation and V373 small-shift counterexamples and the V367/V371 unresolved full re-offset timings.

V376 is complete/reaped (outer 23345): both original strict and APPROXIMATE_512 full re-offset cases still time out at 240.03 seconds against committed 320bfe90. They remain unresolved and outside the passing suite. V377 is also complete/reaped (outer 71672): its copy-only remainder-reuse/positive-normalization trial still times out at 240 seconds in corner 0/candidate 1, despite completing six selected-tuple signs. It is unpromoted. Next V378 restores production inputs and probes the native coincident-circle constraint completeness gap using an independent exact semicircle witness. Keep both computational failures tracked. V375 remains an unrun optional debugger probe. The full goal and the other recorded closure/consolidation gaps remain active. Source inputs must remain frozen while the next driver runs; reap its exact session before editing or starting another driver.

---

Latest qualified milestone (2026-09-27): Hypercurve `1bd0348bb7b848e85f9085932a30033675f59e2f` reuses coefficient generators for exact local-root ordering. The nonzero single-fragment parallel-loop chamfer extension now passes in 112.77 seconds after the prior 240-second timeouts. V348 binds 232 passing selected release tests, six checks, 2,046 inputs, seven binaries and committed blobs; all 30 repositories are clean and all owned processes are reaped. The full goal remains active. Two original full extended-fillet re-offset performance failures and the other recorded closure/consolidation work remain open. See the latest entry in implementation.md for evidence and next actions.

V344 performance recheck is complete and reaped (outer session 88492, exit 124). The frozen V343 binary at committed Hypercurve `68e1f2b01bc9741dd77e410e9f9ba6b4099c9499` still times out after 240 seconds in `bezier_region::tests::one_fragment_nonzero_parallel_loop_extends_chamfer_cuts_on_one_finite_envelope`. Source and executable hashes remain verified. This case remains unresolved alongside the two original full extended-fillet re-offset cases; none of the three is counted among V343's 210 passing tests. The new constrained selected-circle compositions pass, but the full implementation goal is still active. No owned build, test or probe process remains live.

---

### Selected-circle constraint milestone — 2026-09-27

Hypercurve `68e1f2b01bc9741dd77e410e9f9ba6b4099c9499` extends constrained collapsed-circle fillets to retained selected circles. Exact Point and Parameter contacts preserve their source domains, original center/frame and two half charts. Free contact axes still require sufficient constraints. Certified endpoint ownership is shared with point replay; direct normal/tangent displacement inversion preserves scalar parameters where available, and the radial-chord inverse retains general exact evidence otherwise. No public representation or compatibility layer was added.

V343: 210 selected release tests, seven binaries, six checks, 2,046 verified source inputs; outer session 13817 reaped exit0. Open and complete circular paths pass exact region normalization and Boolean comparison, while complete-circle outputs also pass chamfer, offset and containment/XOR checks under both policies and traversal directions. All 30 repositories are clean. Full closure is not complete; see implementation.md and the separately recorded V344 performance recheck.

---

V334 follow-up after ff5f6e8c: a public probe now reproduces the next selected-circle constraint gap. A generated radius-one line/quadratic fillet is subdivided into two retained circular pieces; exact interior point or parameter constraints on both pieces still return Unsupported, while radius-only requests correctly require more constraints. Both policies/directions/modes agree. See the latest implementation.md section and `collapsed-selected-circle-contact-20260927-v334.rs`. Outer 76693 is reaped; no production changes, no live process, all 30 repos clean. First gate: constrained_collapsed_fillet requires PreparedFilletCarrier2::Arc when the collapsed source is AlgebraicCusp. Preserve the existing selected circle for reconstruction instead of deriving a frame from its zero-radius center offset. Goal active.

---

## Latest continuation: ff5f6e8c, exact selected contacts on native circles

ff5f6e8ce3c30a928ab26a94064176f50f372df1 is committed and qualified by V331: 197 selected release tests, six static/downstream checks, 2,046 source hashes, seven binaries and all seven commit blobs. All 30 repositories are clean. Outer session 80153 and every earlier owned driver are terminal/reaped; build mirror matches V331, no live probe or instrumentation, no push. See the latest implementation.md entry and constrained-fillet-api-20260926.md.

Native circle cuts accept retained exact contact evidence. Shared rational-circle reconstruction preserves original endpoint/parameter identity without manufacturing Cartesian coordinates or a normal frame. Single-circle crossed trims are rejected geometrically; optional overlap maps no longer consume approximation during compact-map recognition. The NURBS chart-infinity regression from deferred-marker handling is fixed.

The implementation goal remains active. Remaining constraints: general collapsed selected-circle contacts, native coincident circular supports, mixed retained chord/parallel coincidence and stationary-source one-sided evidence. Computational failures now explicitly include the quartic parallel chamfer test timing out after 240 seconds on both the current change (V327) and committed 00650940 baseline (V328), plus both original whole re-offset timeouts (last V286). V331 excludes these three; do not claim them as passes or mark full closure complete.

---

## Next continuation: V319 retained native-circle contacts


V319 retained native-circle contact counterexample (completed/reaped):

- `retained-native-circle-contact-20260927-v319.rs` uses a path of adjacent native CCW unit quarter arcs (1,0)->(0,1)->(-1,0). Both contact constraints are retained algebraic points evaluated from rational circle charts [(s,0),(s,1),(0,1)], weights [1,1,2], at the selected positive root of 2t²-1, s=+1/-1. Their exact coordinates are (±1/3, 2sqrt(2)/3), but the stored CurvePoint2 coordinate views are absent. Radius one has the intended unit-circle fillet between them; this preserves the same half-disk when closed by its diameter.
- Current `00650940` returns Blocked(Unsupported) in all eight policy/reversal/trim-mode combinations. The probe links V318's qualified normal library and verifies its hash and all 2,046 inputs before/after. Outer session 25714 is reaped exit0; this is negative behavior evidence, not a passing closure test. No production changes followed commit00650940.
- First gate: constrained_collapsed_fillet's Native branch calls point.coordinates() and rejects missing storage. Fixing that gate alone is insufficient: cuts need actual projective source parameters, and collapsed Point offsets currently provide no retained normal frame. `FilletComponentReplay2` on explicit parametric parallel carriers already preserves this information. Consider demand-driven native arc chart replay through that shared machinery, preserving native authored chart maps, seam ownership, extensions and repeated visits. Do not merely coerce contacts to Cartesian/rational payloads.
- Relevant implementation: `RetainedRationalCornerArc2::parameter_at_incident_point/cut_at_incident_point`, `Curve2::finite_fillet_charts`, `CurvePath2::fillets_in_authored_domain`, `PathFilletPlacement2`, and `publish_fillet_corner`/`reconstruct_selected_fillet` in curve_corner_reconstruction.rs. Native arcs currently do not participate in authored multi-chart fillet enumeration. ExactCornerCarrier2::Bezier/NativeBezierSpan explicitly bypasses optional circular promotion and prepares Parallel offsets. Native paths with non-Cartesian cuts already enter retained reconstruction, which requires both cut parameters and a normal frame.
- Native coincident circular supports and mixed retained chord/parallel coincidence remain separate completeness gaps. The original two full extended-fillet re-offset timeouts remain unresolved. Build mirror matches V318, all 30 repositories clean, no owned driver live. Goal active.

---

## Latest continuation: 00650940, exact coincident linear constraints


Coincident linear fillet milestone (2026-09-27):

- Hypercurve `00650940024212c8909bb34f30fc0384121062ef` fixes admissible coincident native/retained line families. Radius-only requests require an exact center or contact; finite constraints retain both isolated and continuous-locus solutions. Remote endpoints stay excluded across trim/extension and source-chart modes.
- Mixed native line/parametric parallel coincidence re-enters the existing parameter-component solver once, preserving original source charts and normal constraints. The V316 line/NURBS witness now returns the intended family requirement and finite 2/2/1 center/line-contact/top-contact counts.
- V318 passes 134 selected release tests, both all-target Clippy configurations, formatting, editing fuzz build, docs, and Hyperbrep check. All 2,046 source inputs, seven binaries, staged index and committed blobs are verified. Outer session 41975 is terminal/reaped. All 30 repositories are clean; build mirror matches V318. V317's first regression failure is retained as negative evidence and is superseded by V318.
- Still open: general collapsed selected-circle contacts, native coincident circular supports, mixed retained chord/parallel coincidence, and both original full re-offset timeouts. Native collapsed circles also reject retained exact contacts without stored Cartesian coordinates; that representation gap is the next investigation. The full goal remains active.

---

## Latest continuation: 41bec6d7, borrowed component replay; V316 native-line counterexample

Hypercurve HEAD `41bec6d77f784288d3da68b4833aaa6cd0b1c385`, parent c986e238. FilletCandidates2 and FilletCornerFamily2 are gone. Exact/prepared solvers return finite CurveCornerSolutions2, apply request constraints in place, and replay parameter components against borrowed prepared offsets. No compatibility layer or cloned center-support groups. V315 passes 131 selected release tests and six checks; receipts verify 2,046 source files, seven binaries, index and commit. All 30 worktrees clean. Build mirror matches V315.

V316 proves the next native coincident-line defect with a line (-3,0)->(0,0) and a degree-one NURBS through (0,0),(0,2),(-3,2), knots [0,0,1,2,2], unit weights. At radius one, centers (x,1), -3<x<0, give valid semicircles touching both horizontal charts. Radius-only must require more input; center (-1,1) permits two finite candidates (quarter circle and semicircle); contact (-1,2), with or without the center, selects one semicircle. Current results are respectively 1, 1, 0, 0 in both policies and reversals. Probe constructs the independent exact semicircle path. See the V316 source/log/terminal receipt; library and qualified source hashes verified.

Next: add this as a regression and repair native coincident support handling with actual contact-domain/ownership and normal proofs. Native line/line support intersection currently returns empty on a parallel pair; native circle/circle coincidence still becomes DegenerateCandidate broadly. A fixed-center replay route may share/generalize constrained_collapsed_fillet and avoid unrelated elimination, but underconstrained nonzero families also need correct detection. Do not replace uncertainty with emptiness or require Cartesian projection of arbitrary exact points.

Remaining: general collapsed selected-circle contacts, retained points without Cartesian coordinates on native collapsed arcs, native coincident supports, and both original full extended-fillet re-offset timeouts. Goal active. Outer sessions 92008/V314, 77805/V315 and 58141/V316 are terminal/reaped; no owned process is live.

---

## Latest continuation: c986e238, fixed-center parallel contacts

Hypercurve HEAD `c986e23825bddef1b68cd822b5db53b41c4f406b`. V314 passes 131 selected release tests and six checks; all 2,046 sources, seven binaries, index and committed files verified. Outer session 92008 is reaped, all 30 worktrees clean, no owned process is live. Build mirror matches V314.

Mixed collapsed circle/direct, retained and selected analytic-parallel contacts now replay exact preimages, authored parameters and original normal branches. Regular isolated contacts survive unrelated stationary source parameters; stationary candidates retain Boundary uncertainty. General collapsed selected-circle contacts and native coincident supports remain open. Next remove FilletCandidates2/private owned family groups by resolving components inside prepared-carrier solving. Both original full re-offset timeouts remain unresolved and excluded. Goal active.

---

## Latest continuation: d4136f50, constrained collapsed circles

Hypercurve HEAD `d4136f50483dbd9e840e03644004a0ff29d14794`; parent `0f1c54de` implements the public constraint request. Collapsed offsets now borrow the original prepared source and bind exact contacts inside the carrier solver. Native circles and circular NURBS select exact nonzero fillets instead of false DegenerateCandidate. Underconstrained supported families require additional input; contradictions and finite-domain exclusions are distinct. Public family APIs remain removed with callers migrated directly.

V309 passes 121 selected release tests and six static/downstream checks. All 2,046 workspace/archive/build sources, seven binaries, staged and committed files verified. All 30 repos clean; no push. Outer sessions 28146/V307, 14448/V308, 55400/V309 are terminal/reaped. V306 is a completed frozen counterexample probe. No owned process is live. Current build mirror matches V309.

Next: see `constrained-fillet-api-20260926.md`. Complete mixed/selected collapsed contact replay and native coincident-support families, preserving source charts and normal-sheet constraints. Use the now-available prepared-solver request binding to resolve components before source lifetimes end; remove transient family/candidate copies where possible. Original two re-offset performance failures remain open and were excluded from V309. Goal active.

---

## Latest continuation: 0f1c54de, constrained public fillets

Public request migration committed; see `constrained-fillet-api-20260926.md` and the latest implementation.md entry. V305: 119 selected release tests, six checks, 2,046 source hashes, seven binaries, all 19 commit blobs verified; outer session 83446 reaped, 30 clean repositories, no push. User design choice is implemented for retained parameter-component families. Exact points keep all preimages, authored parameters select a visit, including repeated circular NURBS.

Next bounded work: native collapsed circle offsets and coincident supports must retain original contact domains and require sufficient exact constraints instead of treating every continuum as degenerate. Resolve while prepared carriers remain alive to avoid another owned family representation. The two original full re-offset performance failures remain open (last scoped V286 diagnostic); no whole-goal completion claim.

---

## Latest continuation: 3ffa46d0, borrowed point-incidence evidence

Hypercurve `3ffa46d0c48ebc61c5b14826b6a1c896ced4581d` adds crate-private
`BezierParallel2::visit_point_incidence_evidence`. It visits borrowed exact contact
parameters or an entire matching domain, retains early-exit membership, and shares
algebraic-ray/chord-pair replay. Complete callers must wait for a decided complete
visit and deduplicate repeated parameters from overlapping proof routes.

V295: 18 targeted release tests + rustfmt + both all-target Clippy configurations
pass. 2046 source hashes, frozen binary and committed tree bound in receipts.
All 30 worktrees clean, no push. Owned sessions 17207/V292 (test fixture
compile failure), 57137/V293, 50514/V294 (unchanged duplicate qualification after
an edit-script assertion), and 2146/V295 are terminal/reaped. Build mirror matches
V295 production. No instrumentation or probes are running.

Next: `constrained-fillet-api-20260926.md` now describes applying center/contact
constraints inside the carrier solver and removing public family-selection API.
Preserve arbitrary selected contact capability; point inversion still has known
unsupported branches, so reuse provenance/location evidence instead of narrowing
that capability. Collapsed offset Point must keep its original circle and contact
domain. Coincident native line/circle supports also need continuum semantics.
Public request migration is not done. Original whole re-offset timeouts remain open.

---

## Latest continuation: d49748d7, exact partial contact constraints

The user chose **additional exact center or contact constraints** for continuous
fillets. Public radius family selectors must be replaced by a constrained request
and finite corner results, with explicit additional-input requirements.

Hypercurve `d49748d7f67105b79610fadf6489e42d38795de9` adds the internal selection authority: a retained parameter
component accepts zero/one/two contact constraints and yields Empty, Selected(pair),
or NeedsConstraint. It transports through retained correspondence/charts, retains
exact supplied parameters, and preserves strict reversed-range endpoint and finite
owner semantics. Existing full-pair membership/fillet selection use this authority.

V291: 8 release tests + rustfmt + both all-target Clippy feature configurations pass.
2046-source manifest, frozen binary, staged and committed hashes bound in receipts.
All 30 repositories clean; no push. Last owned session 48824 fully reaped;
1560/V289 and 75600/V290 are also reaped. Build mirror matches V291 production.

Next: follow `constrained-fillet-api-20260926.md`. In particular, recover and retain
point-incidence parameters instead of dropping them to boolean membership, preserve
fast existence queries, and apply center/contact constraints in the carrier solver
before path/region reconstruction. Collapsed circles and coincident line/circle
supports need explicit contact-domain handling. Public API migration is not done.
The two original whole re-offset cases still time out (last unchanged V286 evidence).

---

Current continuation: Hypercurve f3ee9b129e5eae2805fdc700a6a52f2f01d0c84a, Hypersolve 539eb997d21c2193b759172862667f4bb4e555bd, Hyperreal f0a17ce20a7683751c7d278157614f36048fa239. Hypersolve 46ee306 first repaired narrow-isolator sign replay; V282/V283 passed 525 tests and the captured exact query. V286 then qualifies shared tuple signs and owned coefficient compaction with 693 tests and six static checks. Original source equations remain available after an undecided point-substitution proof. Both full extended-fillet re-offset policies still exceed 90 seconds. All 30 repositories are clean; the implementation goal remains active. A fresh stack trace is next.

Earlier continuation records:

Current continuation: Hyperreal f0a17ce20a7683751c7d278157614f36048fa239, Hypersolve 72691085d1f4d4079741a3030ae369b90440edba, Hypercurve d8f9614cc943b33dd40836aa6d86a138a25be029. V275/V276 qualify exact tower compaction and tensor coefficient replay with 560 cases and six static checks. Extended-fillet re-offset still times out after90 seconds. All 30 repositories are clean, no owned process is live, and the goal remains active. See the latest implementation.md entry and V275 receipts.

Current follow-up: Hypercurve d8f9614cc943b33dd40836aa6d86a138a25be029; Hypersolve 8af4c7f48a4da6d61db8b49b74bb3370d6e89057. V270 qualifies shared source refinement and exact replay against 158 cases and three static checks. Extended-fillet re-offset still times out at 90 seconds. V268 locates the later cost in tensor-image square-free/GCD replay. See implementation.md and the V270 receipts. All 30 repositories are clean; goal remains active.

Current continuation: Hypercurve 69f75cf5349fa938c2f5b35532c036ac13fee0c0 and Hypersolve 8af4c7f48a4da6d61db8b49b74bb3370d6e89057. The scalar selected-fiber isolation blocker below is repaired (two exact roots, no Sturm fallback; 522 Hypersolve tests pass). The full re-offset still stalls later during a recursive circle/chord discriminant sign and retained source-root refinement. V253–V257 recorded that diagnosis; scalar enclosure and optional finite-filter experiments were copy-only. V260 removes the unjustified center-locus regularity premise and qualifies 42 selected-circle/fillet regressions plus formatting and both feature Clippy checks. See the latest implementation.md entry. All sessions are reaped and all 30 repositories are clean as of this note. The implementation goal remains active.

Earlier handoff, retained for context (the scalar-blocker and center-locus-guard paragraphs are now superseded):

Current implementation: Hypercurve 13691647247ae191903c76160c760174cd28e4f0. The retained-domain layer was committed as 8b18d3f9; the public selectable-family layer is now committed. The implementation goal remains active.

Fillet queries expose isolated edits separately from continuous contact families. Families retain component correspondences, exact domains, original normal-sheet constraints, source charts and policy identity. Path and region selection share the isolated cut/reconstruction authority and do not store callback chains or choose arbitrary representative points. All callers are migrated directly. A family is a candidate contact locus: selection can reject excluded or coincident contacts, including an entire continuation locus whose inserted arc would collapse.

The rational joined witness uses P(t)=(3t/8,9t²/64), original left offsets41/64 and5/8 traversed backwards, radius1/128, center distance81/128 and contacts at t=u=5/9. The public path test checks the independent rational contacts, center, radius and both orientations. The normalized-region test selects this family and composes an actual chamfer, round offset and Boolean clip. Both policies certify the operations; certified families created through APPROXIMATE_512 remain STRICT-replayable.

V240 records635 passing and2 failing release cases across11 binaries, with all four static/downstream checks passing. The source/index/HEAD review receipts bind2046 inputs and all18 committed files;30 repositories are clean. This is NOT full closure qualification. Both failures are extended analytic-parallel region fillets re-offset by1/1000. A stale assertion that every fillet had one boundary loop was independently reproduced on8b18d3f9 and corrected; the test now reaches the remaining actual offset blocker.

Immediate blocker: selected-circle/analytic-parallel intersection during re-offset regularization. Diagnostics V237/V238 show a same-source, nonadjacent pair across loops0 and1, carriers0 and5. selected_fiber_parameters_in_range calls Hypersolve on a reduced bivariate incidence with6rows and21columns and non-rational Real coefficients. The report is Undecided after32Bernstein subdivisions and4retained-root refinements, with no Sturm sequence built. The generic message identifies incomplete local-field arithmetic or coefficient signing. A diagonal-multiplicity simplification was tried under V239, failed identically and was fully reverted. Export a bounded exact fiber fixture (scalar serialization, never recursive Debug) to diagnose Hypersolve directly. Do not convert uncertainty into an empty intersection or raise precision limits without evidence.

Remaining family coverage: the public retained representation currently covers parallel/parallel parameter components. PreparedFilletCarrier2::offsets still collapses native/rational arcs and retained algebraic semicircles to Point { point }, losing the source contact domain. Point-on-offset and other coincidence branches discard parameter evidence. Preserve free axes, fixed axes and complete products before selecting/reconstructing those families; do not call every coincident support degenerate. Existing component machinery already represents correspondence and product loci, including exact chart ownership and selected roots.

Other gaps: selection exactly at a center-locus cusp must be audited. from_selected_parallel_normal certifies the underlying source frame but additionally rejects a zero center-support derivative; that extra premise may be unnecessary for a circle frame and needs a regression before removal. Original source-stationary and parallel-cusp one-sided tangents remain distinct unresolved cases.

Useful next family regression: forward first offset41/64 on[-1,0], second offset5/8 on[1,2] still join at(0,41/64). With radius1/128, t=u=5/9 lies on both permitted extensions and should select one CCW family; reversing gives the CW case. TrimOnly must exclude this pair. Also select an arbitrary exact non-rational parameter inside the compatible band, such as sqrt(5)/4, to check that chart replay does not demand rational reconstruction accidentally.

Full completeness, coefficient-growth control, arbitrary finite operation composition and the larger API/machinery simplification remain open. Do not mark the goal complete. No production/test/probe input may change during an owned build/test/probe; reap the exact outer session before further edits, staging or commits. No process is currently owned or live after V240.


### Coincident circular fillet families — work in progress, 2026-09-27

V379 independently confirms the native-circle gap on committed Hypercurve 1bd0348b: two concentric source arcs have valid nonzero semicircle fillets of radius 1/2, but radius-only requests omit the continuous family and exact center/contact/parameter requests return no solution. The second source is an authored degree-two NURBS containing a line and a circular span. Both policies, directions and edit modes reproduce the failure. V378 failed only to link the standalone probe; V379 fixes its dependency search path and is terminal/reaped (outer 74993).

The current uncommitted fix preserves the radial correspondence of coincident noncollapsed circle supports. It shares fixed-center selection with coincident lines, accepts center/point/parameter constraints, and tests circular domain overlap without enumerating overlap intervals. An interior point is only an existence witness, never an arbitrary returned fillet. Source-chart incident endpoints still permit finite isolated contacts; authored/remote endpoints retain their exclusions.

V380 passes the initial circular and two existing line regressions (outer 78972). V381 additionally passes signed-radius offsets through the source center, full circles, disjoint/complementary sweeps, major-arc overlaps with two components, isolated source-chart contacts, both policies/directions/modes, independent-region Boolean equality and subsequent offset/classification. Four tests pass, outer 64598 terminal/reaped exit0. Broad qualification and commit remain pending. Retained contacts/centers without Cartesian coordinates are being added. The full goal and the two original extended-fillet re-offset performance failures remain open.


V382 broad qualification stopped at the new retained-contact case: the exact same point succeeds in Cartesian form but is blocked as retained evidence. Outer 74428 is terminal/reaped exit1. V383 copy-only instrumentation locates the Unsupported result in concentric circle-frame construction (`curve_corner_chain.rs:3379`), reached by the first retained point constraint; outer 16584 is terminal/reaped exit1. No diagnostic edits were promoted. V384 will test a shared reconstruction fallback: normalize and rotate the certified radial through a retained similarity and reuse the existing general chord-normal circle frame. The one-field and selected-radial fast paths remain. This extends the common concentric-normal constructor rather than adding a new public or private circle representation.


V384 gets past retained concentric-frame construction but exposes missing tangent evidence (outer 82382, reaped exit1). Coincident nondegenerate line/circle families have opposed source tangents; their shared center selector now retains cross=0/dot<0 for the semicircle instead of rediscovering it through parameter incidence. V385 then publishes a result, but the next contact is wrong (outer 65919, reaped exit1). V386 reproduces that exact failure with a backtrace. V387 copy-only bounds diagnostics locate it on the authored NURBS' line span: the returned contact is approximately (1.585786,0), while the certified expected circle contact is (1.2,-1.6). Outer 31179 is reaped exit1. No diagnostic edits were promoted.

The shared reconstruction function was selecting the previous source's last fragment and next source's first fragment, discarding the already solved fragment indices. V388 carries each selected fragment index with the complete authored domain, preserving local parameter ownership while still retaining all source fragments for circular extensions. This is a common reconstruction fix, not a NURBS-specific branch. The retained contact/center tests also include independent-region Boolean equality and re-offset. Qualification and commit remain pending; no owned diagnostic process remains live before V388 launch.


V388 passes all five focused tests, including retained contacts and centers, signed-radius sheets, strict and approximate requests, source-chart domain ownership, independent-region Boolean equality and subsequent offset/classification. The new circular composition regression takes 1.12 seconds. Outer 36231 is terminal/reaped exit0; the mirror again matches production, with no diagnostic instrumentation. V389 broad qualification will reuse these five exact-source/binary results and cover the full 234-case selected geometry scope plus six static/downstream checks. Four Hypercurve files remain uncommitted pending qualification.


V389 passes all 234 selected release tests across seven binaries and all six static/downstream checks. Outer 31836 is terminal/reaped exit0. The slow chamfer-extension case passes in 112.97 seconds; the new circular retained-contact composition is independently qualified in V388 and reused against identical source/binary hashes.

Final review found another domain issue before commit: a RetainedRationalCornerArc2 stores its parent support and its actual surviving interval separately. The initial family classifier used the parent sweep. The final adjustment maps actual native/retained endpoints into the common center circle and classifies them in the other source's actual interval. Its interior witness likewise comes from the surviving interval. A new regression covers disjoint/touching/overlapping restrictions of the same parent quarter, both directions/policies/chart ownerships, each extension permission, and an irrational bound. Only curve_fillet.rs changes after V389. V390 qualifies that final adjustment against the affected fillet/domain/chart tests and repeats all six checks; the three shared reconstruction/source files are unchanged from V389. No commit yet.


### 2026-09-27: constrained coincident circles and preserved source charts

Committed Hypercurve `1f29f69df38496b7941d3b1a897dcf609f78418d` on `1bd0348bb7b848e85f9085932a30033675f59e2f`. Coincident noncollapsed circular offsets now require an additional exact center/contact only when the actual admissible domains leave a nonzero family. Exact center, point and authored-parameter constraints return the selected finite fillets. Native full/major arcs, signed-radius offsets through the center, irrational rational-chart bounds, finite isolated source-chart contacts, and remote/authored endpoint exclusions retain their semantics.

The private fixed-center selector is shared with coincident lines and retains the proved opposed-tangent relation. General retained centers reuse the existing chord-normal circle representation; the one-field and selected-radial constructions keep their fast paths. Reconstruction now carries the selected fragment index with the complete authored domain, preventing a circular NURBS contact from being evaluated on a neighboring line span. Circular family existence uses surviving retained intervals rather than their larger parent supports. No new public representation or compatibility interface was introduced.

V389 passes 234 selected release tests across seven binaries and six checks for the common reconstruction repairs. After the final surviving-domain correction (only curve_fillet.rs), V390 passes 86 affected tests across seven binaries and repeats all six checks: all-target Clippy with/without default features, changed-file formatting, editing fuzz, denied-warning documentation, and Hyperbrep all targets/features. The circular retained-contact/center Boolean-and-offset regression takes 1.12 seconds; the clipped-interval regression takes 0.02 seconds. The existing slow chamfer extension remains passing at 112.97 seconds in V389.

Source archives and receipts bind 2,046 inputs, frozen executable hashes, index bytes and committed bytes. V389 outer 31836 and V390 outer 67326 are terminal/reaped exit0; review outer 83900 is also reaped. All 30 repositories are clean. Build mirror matches committed V390 source. No push and no owned driver remains live. The full goal remains active: both original extended-fillet re-offset performance cases remain open, along with other coincidence/source-frame cases, stationary one-sided evidence, and the larger representation/algebraic consolidation.

Next audit the mixed retained-chord/analytic-parallel coincidence path, whose native-line re-entry currently rejects an algebraic-chord source. Independently, repeated point similarities still nest in BezierSimilarityPoint2::new despite the existing certified Similarity2::then operation; normalization is a concrete computational simplification candidate after the remaining family correctness gaps.


V391 public probe confirms the next mixed-coincidence gap on committed 1f29f69d. Geometry: lower line (-3,0)->(0,0), then a degree-two NURBS with controls (0,0),(0,1),(0,2),(-1,2),(-3,2), unit weights and knots [0,0,0,1,1,2,2,2]. The upper straight span has x(u)=-2u-u². An independent radius-1 CCW semicircle has contacts (-2,0),(-2,2) and center (-2,1); the authored upper parameter is sqrt(3), reversed 2-sqrt(3).

Native-line radius requests correctly require a constraint. Forward TrimOnly point/parameter requests return one Certified result, but center-only is Unsupported; reversed point/center requests also expose Unsupported branches. Forward TrimOrExtend reports two results even for the parameter constraint; inspect those candidates before calling them duplicates or invalid. Replacing only the lower native line with public BezierAlgebraicChord2/Curve2::from makes all 32 requests Unsupported across both policies, directions and modes. The native-vs-retained comparison is an exact representation counterexample.

Outer 6212 is terminal/reaped exit0 as a collecting diagnostic, not a passing qualification. V391 source/library/binary hashes bind the committed V390 sources. All production repositories remain clean and the mirror still matches production. Next use bounded copy-only error-location tracing to separate the native center/reversal failures from the explicit retained-chord rejection before implementing the shared repair.

V392 completed and reaped its owned build/probe children: center-only and reversed point constraints fail at curve_corner_chain.rs retained_fillet_sweep (missing ParallelNormal tangent evidence); all retained chords are rejected by the mixed coincidence re-entry at curve.rs. V393 probe compile failed on private same_point; outer session 25116 reaped exit 1. V394 uses public coincides_with and completed, outer 74644 reaped exit 0. Both extension-mode solutions share all source cut endpoints; exact parameter constraints also duplicate. V395 introduces selected-pair deduplication and retains exact oriented tangent proof in component replay, with a nonlinear NURBS straight-span regression and independent semicircle Boolean witness.

V395 (89981 exit 1) and V396 (4164 exit 1) were fully reaped. The new component replay reached the correct constrained contacts but Boolean XOR of the center-only TrimOnly result failed. V397 public probe (67504 exit 0), then V398 dispatch probe (18973 exit 0), traced this to selected-parallel-normal circle versus rational-circle coincidence: the Exact parameter branch classified IdenticallyZero as DegenerateProjection, whereas the Algebraic branch retained a component. V399 routes the Exact zero-incidence case through existing represented exact-frame circle replay; the norm-conjugate issue is resolved by the unsquared exact frame rather than assuming overlap from a zero norm. Source changes are curve_fillet.rs and bezier_offset.rs only; diagnostics remain archived copies.

Committed Hypercurve 46f278dacb5b8556b4196a09e22d1721b20695cb (parent 1f29f69d): selected component pairs retain oriented tangent evidence, incident-chart duplicates compare source parameters, and exact scalar parallel-normal circles reuse represented-frame overlap replay. The nonlinear straight NURBS regression covers both policies, reversal, trim/extension, radius-only requirements, center/point/parameter constraints, exact contact positions, connectivity and independent Boolean equality.
V399 focused run had 26 actual passing tests and two misnamed zero-test filters. V400 (outer 63110, reaped exit 1) completed 170 valid selected unit tests and all six static/downstream checks before its reuse assertion rejected those zero-test records. V401 corrects the module names and enforces exactly one executed test per case; it binds 238 distinct release tests across seven frozen binaries and six checks to identical 2046-file source manifests. Only verified identical-source/binary records are reused. V401 outer 43598 reaped exit 0; reviewer 50148 reaped exit 0; staged and committed blobs verified; all 30 repositories clean. Artifacts: fillet-component-witnesses-20260927-v401-{sources,terminal,reviewed,staged,committed,repositories-after}.json. The two existing full extended-fillet re-offset performance failures remain excluded and unresolved. Next: retained chord versus nonlinear straight/parallel coincidence, using an exact affine solve chart and original chord cut parameters.


### Retained chords share constrained fillet component replay — 2026-09-27

Committed Hypercurve `9d1dc57db828847afc4dfec0b4ba5fbb4d0a6121` on `46f278dacb5b8556b4196a09e22d1721b20695cb`. The mixed coincident line/parallel path now admits retained chords with a represented affine support. An exact solve chart uses the original chord endpoints and their retained projected parameters. The shared component solver preserves every nonlinear companion parameter and normal sheet; reconstruction publishes original chord locations and rechecks the original constraint conjunction. Native isolated fast paths remain in place. This adds no public curve variant or compatibility interface.

The new chart exposed an existing zero-polynomial arithmetic defect: empty coefficient rows are valid zeros, but multiplication subtracted one from a zero dimension and panicked with capacity overflow. All three affected local multiplication paths now short-circuit structural zeros, including allocation-fallible and first-parameter multiplication. Hypersolve's existing omitted-coefficient convention is preserved.

V402 compile failure (outer 32133), V403 capacity-overflow regression (40367), and V404 frozen-binary backtrace (33726) are terminal/reaped. V405 passes 29 focused cases (97392); V406 adds exact rotation and passes the expanded regression in 5.92 seconds (48426). That regression covers 240 selected edits plus radius-only constraint requirements across native lines, retained chords, offset-returned non-Cartesian endpoints, both policies, both traversal directions, both trim modes, and all five center/contact/parameter selectors. Every result has exact original endpoints and contacts, connected pieces, and Boolean equality to an independent semicircle.

V407 passes 239 selected release tests across seven binaries and six checks: both all-target Clippy feature modes, formatting, editing fuzz compilation, denied-warning documentation, and Hyperbrep compilation. The slow chamfer-extension case passes in 110.50 seconds. The one reused V406 case has identical source and frozen binary hashes and exactly one executed test. Qualification outer 20622 and reviewer 3142 are terminal/reaped. All 2,046 source inputs, staged bytes and committed blobs are verified; all 30 repositories are clean. Nothing was pushed.

Artifacts: `retained-linear-component-replay-20260927-v407-{sources,terminal,reviewed,staged,committed,repositories-before,repositories-after}.json` and its qualification/review/seal drivers. The full goal remains active. Both original extended-fillet re-offset performance failures remain excluded and unresolved; general retained chords without a represented affine support, further independent-frame/stationary cases, and the broader representation/algebraic consolidation still require work. Repeated BezierSimilarityPoint2 construction currently nests compatible transform layers even though Similarity2::then already composes certified transforms; this is a concrete next computational normalization, with retained-policy barriers preserved explicitly.


### Compose retained point similarities at construction — 2026-09-27

Committed Hypercurve `922a155e059ce12feb5042ed8b65a4c2e26cafe4` on `9d1dc57db828847afc4dfec0b4ba5fbb4d0a6121`. BezierSimilarityPoint2::new now composes compatible similarity layers with the existing certified Similarity2::then operation and shares the original point field. Repeated transports no longer retain and replay one wrapper per operation, and bounds use the composed map instead of enclosing every intermediate transformed box. No new carrier, coordinate materialization, or repeated matrix/scale certification is introduced. Incompatible retained-policy layers remain visible; composition never makes approximate evidence available to strict predicates.

V408 independently fails the new field-sharing regression on the prior constructor at the second transport (outer 97980, terminal/reaped exit1). V409 passes both new tests and six focused similarity/contact tests (29051, reaped exit0). The new cycle regression covers 384 exact transports across both policies, noncommuting translation/reflection order, certified scale/orientation, independently constructed algebraic point images and shared original field identity. The policy regression checks both rejected and subsequently permitted compositions, including strict predicate and bounds rejection after approximate construction.

V410 passes 297 selected release tests across seven binaries (228 unit tests and 69 integration tests), including the full prior 239-case closure scope plus additional transform/circle/policy and public transform-and-offset cases. All six static/downstream checks pass again. The long chamfer extension passes in 111.46 seconds; the nonlinear retained-chord fillet/Boolean regression remains 5.92 seconds. Eight focused V409 tests are reused only after matching exact source and frozen binary hashes and confirming one executed test per filter. Qualification outer 62399 and reviewer 38261 are terminal/reaped. The 2,046 inputs, index bytes and committed blobs are verified, all 30 repositories are clean, and nothing was pushed. No driver remains live.

Artifacts: `point-similarity-composition-20260927-v410-{sources,terminal,reviewed,staged,committed,repositories-before,repositories-after}.json`, its qualification/review/seal drivers, and the V408/V409 baseline/focused receipts. The full goal stays active. Both original extended-fillet re-offset performance cases remain explicitly excluded and unresolved; this passing scope does not establish that they are repaired. General coincident retained-chord/parallel sources without a represented affine support, other independent-frame/stationary cases and broader API/algebraic consolidation remain open. A concrete next completeness probe is an oblique chord whose endpoints retain independent selected fields, paired with an authored nonlinear straight trace on its exact parallel; the current generic chord/parallel coincidence branch still reports Boundary uncertainty. Recheck the two original full re-offset cases when auditing this normalization's effect on their construction histories.


### Independent oblique chord families and shared predicate replay — 2026-09-27 (qualification pending)

The next counterexample uses a retained chord from (sqrt(1/2),0) to (0,3sqrt(1/6)), with independently selected endpoint fields and no represented affine support, alongside an obliquely transformed nonlinear straight NURBS trace. A certified coincidence now retains its regular sample. The fillet solver recovers a temporary affine source chart from that sample, preserving the companion derivative scale, reuses the existing component solver, and restores the original chord locations and constraint conjunction. This adds no public curve representation or compatibility interface.

V415 and copy-only V416 still fail after roughly 164–171 seconds in radical pair projection. Component queries now reuse the existing domain-certified rational PH images when both are available, retaining the original parameters and normal constraints. V417 passes its initial 16-case focused scope; the initial oblique fixture takes 0.25 seconds. That timing is for the smaller fixture, not the subsequently expanded composition regression.

V418 expands the fixture to both policies, representations, directions, modes and five selectors, plus independent Boolean equality and re-offset. V419–V421 copy-only tracing locates an unresolved retained-chord tangent linear form. Its preliminary evaluator now declines under strict suppression, then the existing recursive projective direction field decides the complete form before the approximate terminal. V422 passes the new low-level linear-form regression but exposes a later Boolean Ordering failure. V423–V425 copy-only tracing locates the reversed chord/rational overlap comparison: oblique displaced coordinates were using bounds without their available projective authority. The shared displaced-coordinate query now reuses the existing coordinate fallback. V426 passes the coordinate and tangent regressions and the previously failing reversed TrimOnly composition, then exposes a TrimOrExtend InvalidBezierParameter error.

V427 copy-only backtracing identifies retained_incident_ray_regular_anchor_from_polynomials: a certified exterior affine bridge was passed to the unit-domain interval constructor. The existing ordered-interval constructor now admits that bridge, after unchanged exact source-weight and speed-sign certification. A regression covers selected and recursive exterior endpoints on both sides of [0,1], both extension directions/policies, and exclusion at/across the exact rational pole.

V428 passes all 21 focused tests. The expanded oblique regression completes in 22.04 seconds: 16 radius-only requests require an exact constraint, 80 selected edits preserve original endpoints/contacts and connectivity, every edit matches an independent semicircle region by Boolean XOR, and all 16 center-selected cases re-offset and match the independently offset witness. Dedicated coordinate and tangent tests preserve exact zeros and nonzero separations below 2^-512; the expanded swapped-operand component test preserves original normal constraints through both rational-image variants and both component query modes.

All V415–V428 drivers are terminal/reaped, including V423 outer 91741, V424 87835, V425 86417, V426 43115, V427 17774 and V428 66400. Diagnostic edits remain in archived copies only. Five Hypercurve files remain uncommitted on 922a155e059ce12feb5042ed8b65a4c2e26cafe4. V429 broad qualification (outer 91397) is running against 2,046 frozen inputs, selecting 322 release tests across seven binaries and six checks. All six checks have passed; selected tests are still running. Sources, index and committed blobs have not yet received final review/sealing. The two original extended-fillet re-offset performance failures remain explicitly excluded and unresolved. The full architectural goal remains active.


V429 (outer 91397) is terminal/reaped exit 1: all six static/downstream checks and the first 77 selected cases passed, but nonlinear_selected_corner_chamfers_through_retained_fixed_distance_image hit its 240-second limit (241.91 seconds including cleanup), versus 4.72 seconds for the same case in V410. No commit was made. The new displaced-coordinate path entered common projective replay before exhausting the previous exact bounds path. The candidate now runs that existing bounds predicate under strict suppression first, returning every certified result and retaining the common projective fallback for unresolved comparisons. V430 (outer 59996) is running 22 focused tests, starting with that long chamfer case. This is a performance hypothesis under test; it is not yet a successful broad qualification. Production/archive/mirror inputs are frozen while the driver runs.


V430 (outer 59996) is terminal/reaped exit 1; restoring only the displaced-coordinate bounds path still times out the same chamfer case at 241.88 seconds. V431 also restores the native interval decisions before the new tangent linear-form field replay, keeping the replay before any approximate terminal. V431 (outer 54512) is terminal/reaped exit 0 with all 22 focused tests passing. The nonlinear selected-corner chamfer now passes in 4.77 seconds, compared with 4.72 seconds for the same test in V410; its original performance is restored, not improved. The separate one_fragment_nonzero_parallel_loop_extends_chamfer_cuts_on_one_finite_envelope case took 111.46 seconds in V410. The expanded 80-edit oblique fillet/Boolean/offset regression passes in 22.29 seconds. Dedicated exact coordinate and tangent tests pass in 0.21 and 0.41 seconds. These are observed matched test timings, not a uniform performance claim. V432 repeats the 322-case broad scope and all six checks on the final candidate, reusing focused records only with identical source and binary hashes. No commit yet; the two original extended-fillet re-offset performance cases remain excluded/unresolved.


V432 (outer 25306) is terminal/reaped exit 1: all six checks and 85 selected cases passed, including the now-restored nonlinear selected-corner chamfer. The distinct one_fragment_nonzero_parallel_loop_extends_chamfer_cuts_on_one_finite_envelope test hit 240.02 seconds; its matching V410 baseline is 111.46 seconds. The earlier 4.77-second result matches the nonlinear selected-corner test's 4.72-second V410 baseline. Timing attribution in the unexecuted review/commit description has been corrected. No staging, review seal, or commit occurred; review-independent-oblique-fillet-20260927-v432.py must not be run as though V432 had passed.

V433 copy-only instrumentation traces entry/completion of the new tangent and displaced-coordinate projective fallbacks, including bounded-predicate status and function-only backtraces, on the newly failing one-fragment nonzero parallel test. The first script attempt had a Python syntax error before any archive/build/child creation; the corrected script runs as outer 65365 against the unchanged V432 production manifest. Production remains the five-file candidate on 922a155e; only the diagnostic archive/mirror are instrumented. Reap 65365 before any production change or another qualification driver. Next artifact number is V434.

### V433/V434: distinguish replay cost from downstream ordering

V433 outer session 65365 was reaped with exit 1: the quartic one-fragment
chamfer still hit its 240-second limit. Its twelve retained tangent replays
were unbounded, exact, and inexpensive (22–51 ms); the displaced-coordinate
fallback was not entered. V434 (outer 85004, still live) instruments native
tangent stages and chamfer phases. The separate nonlinear selected-corner
case passes in 4.72 seconds. In the quartic case, all observed native bounds
and shared-field replays finish in at most 161 ms, and trimming finishes
in 313 ms. Extension then stalls downstream. Thus refining tangent bounds
is not the dominant regression. The angular comparator discards its already
proved collinearity to the reference ray and recomputes a pair determinant;
that is a candidate machinery simplification to verify after V434 is reaped.

V434 outer 85004 was reaped with exit 1 (the quartic extension timed out at
240.02 seconds). Production now retains each ray's collinearity bit alongside
its already decided half-plane. Within one half-plane, a reference-parallel
ray is last in clockwise order; two such rays are equal. This reuses existing
exact cross/dot evidence and avoids a redundant determinant between independent
candidate fields. No new curve representation, work cap, or uncertain topology
decision was introduced. V435 outer 50198 is running 24 focused cases, including
both performance regressions and the existing exhaustive retained/represented
angular ordering test. The pending slice now changes six Hypercurve files.

V435 outer 50198 was reaped with exit 0: all 24 focused tests pass. The
quartic one-fragment nonzero parallel chamfer passes in 81.02 seconds, versus
111.46 seconds in V410 and the intervening >240-second regression. The distinct
nonlinear selected-corner case passes in 4.82 seconds (V410 4.72). All angular
ordering combinations pass, and the 80-edit constrained oblique fillet workload
passes in 22.19 seconds. V436 broad qualification will cover 323 tests plus six
checks, reusing only V435's identical frozen source/binary passes.

### Committed independent oblique coincidence replay

Hypercurve `aa34aa573dbfb405d4589ab1899d3c4a873b65c6` is committed. V436 outer 33572 and review 41881 were
reaped successfully. Qualification passed 323 selected release tests across
seven binaries and six checks. All 2046 source files matched the frozen
archive and build mirror; staged and committed versions of all six changed
files were verified against the same hashes. All 30 repositories are clean.

The slice retains the certified coincidence sample, derives an exact affine
solve chart without flattening independent chord endpoint fields, reuses the
existing parameter-component kernel, and restores original contact identities.
Radius-only continuous families require an exact center/contact constraint.
The same change reuses domain-certified rational offset images, replays exact
linear forms and displaced coordinates, admits exterior regular affine
bridges, and avoids redundant tangent-field determinants in angular ordering.

The larger goal remains active. The two original extended-fillet re-offset
performance cases were excluded from this passing scope and remain open.
V437 will recheck both against the exact frozen V436 promotion binary, with
240 seconds per case and an assertion that each passing filter executes one
actual test. No prior timeout is being represented as a pass.

V437 outer 59500 was reaped with exit 1. Both original extended-fillet
re-offset tests still time out at 240.03 seconds against the frozen V436
production binary; neither passing filter ran zero tests. All production and
archive hashes remained unchanged. These are unresolved computational closure
failures, not part of the 323-test qualified commit. V438 will test a copy-only
Hypersolve change: reuse the existing exact polynomial interval enclosure before
constructing a selected-root Sturm-Tarski chain. The original chain remains the
fallback for unresolved enclosures; no policy terminal or field is changed.

V438 outer 28505 is reaped with exit 1: the copy-only Hypersolve interval
trial also timed out at 240.03 seconds. None of the eleven logged root queries
was decided by its interval. Eight degree-six-root / cubic-query chains took
2–3 ms; two quintic-query chains finished by 79 ms; a third quintic query
remained inside the chain for the rest of the time limit. The trial is rejected
for production. V439 restored the committed source in the build mirror and
reused the frozen V436 binary for GDB, but sandbox ptrace was denied. Outer
83095 is reaped, no stack was captured. V440 retries the same read-only debugger
probe with approved process-debugging access; outer 79067 is live. Source
inputs remain frozen. Next artifact is V441.

V440 outer 79067 was reaped with exit 0 after 45.47 seconds. GDB captured a
function-only stack from the frozen committed V436 production binary, then
killed and reaped its test process. This is a diagnostic capture, not a test
pass. The active stack is BigUint division/remainder -> Hyperreal Rational
wide-magnitude GCD/reduction -> Quad::mul -> tower_from_computable ->
quadratic_tower_sign -> Hyperlimit exact comparison -> Hypersolve
trim_polynomial / polynomial_div_rem_trimmed -> sign_at_selected_root /
sign_at_selected_tuple -> Hypercurve dense tuple signs -> six recursive
quadratic sign frames -> selected-circle/chord intersections -> Boolean
regularization during region offset. Thus the cost lies in coefficient
normalization and exact zero/sign reconstruction inside remainder division.

The common division kernel already removes its cancelled leading slot by
construction; do not propose that already implemented fix again. It still
policy-trims lower coefficients after each elimination. V377 previously tried
fixed-degree CertifiedPolynomialDivisor remainder replay plus monic/content
normalization in a copy-only root-sign trial and still timed out. A future
change must distinguish those experiments, preferably by isolating the
expensive coefficient/remainder state or improving its retained field authority.
No scalar/solver experiment from V438 was promoted. The build mirror has been
restored to the committed production inputs and verified against all 2046
source hashes. All drivers are reaped; no source edits are pending. Next
artifact is V441. The full goal and both re-offset failures remain active.

### Fixed-field root-sign diagnosis and scalar fusion trial

V441 outer 23777 and V442 outer 54322 are reaped. These are intentional
input-capture diagnostics (test exit101 after the third query), not passing
geometry tests. V442 exports all coefficient data as exact nested quadratic
parts; all three queries use Q(sqrt(5),sqrt(50-20sqrt(5))). Source coefficients
are at most 1594 display characters. V443's initial Fraction parser rejected
Hyperreal's mixed-number display; it was terminal before the corrected V444
ran. V444 outer 88513 is reaped successfully: independent exact field
arithmetic proves one defining root in each retained interval and signs
+1,-1,+1. Positive rational content normalization still leaves a 154279-bit
last remainder for query002, confirming substantial coefficient growth.

V445 outer 90737 is reaped with exit1: the standalone reconstructed query002
still times out at 90 seconds with production scalar arithmetic. The fixture
thus reproduces the defect without region machinery. V446 is a copy-only
Hyperreal trial: Quad multiplication and inversion use existing fused rational
product sums; Quad sign reuses structural same-sign decisions and the existing
product-sum ordering without materializing a rational norm. No public interface
or supported scalar family is changed. Its driver is now starting; production
remains clean at HC aa34aa57 / HR320bfe90 / HS4ba7f250.

V446 outer 94761 is reaped with exit0: standalone captured queries002/000/001
pass with oracle signs +1/+1/-1 in85.22/.46/.46seconds. The scalar fusion
trial is promising but not promoted. V447 will add copy-only positive common
rational content removal across each quadratic-tower polynomial, retaining
all root/interval authority and the general Real fallback. Production unchanged.

V447 outer35418 is reaped with exit0: common rational tower-content removal
passes all three oracle queries but costs41.97/13.13/10.58seconds for002/000/001,
regressing the two simpler queries substantially. Not promoted. V448 isolates
positive leading-unit normalization with the now-measured fused scalar kernel,
using the existing GCD normalization and ordinary division. Unlike V377 it
measures standalone inputs and omits the certified fixed-degree divisor change.
No new scalar ratio API is included in this trial.

V448 outer87829 is reaped with exit0: positive leading-unit normalization plus
scalar fusion passes captured002/000/001 in23.14/4.87/5.57seconds. It improves
the hardest query but regresses simpler ones relative to fusion alone. V449
outer9004 is live, checking the original two extended-fillet re-offset cases
against exactly this trial. Production and all driver source inputs are frozen.

V449 outer9004 is reaped with exit1: STRICT reaches a later assertion at
161.78seconds, corner1/candidate1, after the earlier hard re-offset completed.
The assertion inspects only boundary_loops()[0] for1-2 fillet arcs, although
the same test already allows incident extensions to regularize into multiple
loops. This is a new semantic/test-ownership question, not a passing case.
The APPROXIMATE case was not run after failure. V450 outer71777 is live on
standalone queries with the polynomial part of P'Q/P removed before building
the field chain. It retains P and its original singleton authority.

V450 outer71777 is reaped with exit0: removing the initial polynomial quotient
passes the three standalone oracle queries in21.79/4.32/5.07seconds. V451
outer63090 is live, adding same-sign coefficient branches before constructing
the outer quadratic norm. V452 is prepared but not run: it will report bounded
per-loop circle counts and check fillet chart ownership across all regularized
loops, then run both complete re-offset cases with a360-second per-case cap.

V451 outer63090 is reaped successfully; query002/000/001 take20.74/4.32/5.07s.
V452 outer26524 is reaped with exit1: STRICT still times out at360.03s, and
APPROXIMATE was not run. Bounded layout logs prove the chart assertion error:
extended candidate0 has loops[(4curves,2arcs),(8,0)], while its reflected
candidate1 has[(8,0),(4,2)]. Aggregate arc ownership is identical. Both original
full composition cases remain open. The incremental kernel/test slice will
now be qualified in production across the existing323-case scope plus full
scalar/solver suites. Initial query reduction is moved after zero trimming
to preserve zero queries when the defining leading degree is unresolved.
No timeout is being promoted to a passing qualification.

V453 outer50244 is reaped with exit1. Hyperreal Clippy and all933 tests pass.
Hypersolve Clippy caught six test-only callers of renamed monic_normalize in
root_isolation_monic_tests.rs; all are updated directly, no alias. V454 reuses
only the two identical Hyperreal checks (the sole source delta fromV453 is the
Hypersolve test file) and runs the remaining scalar/solver/geometry verification.
Five production files are pending across three repositories.

V454 outer99706 is live on frozen production sources. Hyperreal933 tests and
Hypersolve898 tests pass, including the new arbitrary-Real zero-query guard,
repeated-conjugate queries with2^512 scale separation, scalar cancellation,
and nested sign branches. Scalar/solver Clippy and both Hypercurve Clippy
feature modes pass; formatting passes. Remaining broad checks and323 selected
geometry tests are still running. No source/index edits until reaping99706.

V454 preflight is complete:14 checks pass, including933 Hyperreal tests and
898 Hypersolve tests. The broad Hypercurve binary is compiling, so no geometry
pass count is claimed yet. The pending slice reuses scalar product sums, one
private monic polynomial kernel, retained root/isolator authority, and the
existing regularized region; no public API or carrier was added. Review/seal
scripts and a post-commit function-only stack probe(V455) are prepared only.

V454 outer99706 is reaped successfully. All323 selected geometry tests pass
across7 binaries, plus14 checks with933 Hyperreal and898 Hypersolve tests.
Selected-circle constrained-contact replay improves41.17->28.30seconds;
nonlinear selected-corner chamfer4.82->4.87, retained parallel chamfer81.02->81.27,
and the80-edit constrained oblique fillet workload22.19->21.89. V455 outer8330
is live on an exact-source replay of the three independently certified captured
queries, because production moved quotient reduction after the zero-query
guard. Production sources stay frozen. V456 is the prepared post-commit
remaining-case function-only stack probe; no V455 stack probe was run.

### Committed normalized scalar and selected-root replay

- hypercurve: `0d5a2c0ff0f80833fd37d9fc31eec9ffd4f923d0`
- hyperreal: `a4b5a570155cb3bccbbe4679522b5cd3f962c174`
- hypersolve: `924762a75f2dbb0a0061ee78601f2d7e50c70b72`

V454 outer99706, V455 outer8330, and review26562 are reaped successfully.
All2046 source hashes, all five staged/committed blobs, and all30 repository
statuses were verified. Passing scope:933 Hyperreal tests,898 Hypersolve tests,
323 selected Hypercurve tests, three independent captured-query oracles, and
14 checks. Final-source captured002/000/001 times20.64/4.32/5.02seconds.
No public API, scalar carrier, compatibility shim, degree cap, or root authority
replacement was introduced. The two original full re-offset cases remain
unresolved; V452 STRICT timed out at360seconds, APPROXIMATE was not rerun.
The first-loop fillet chart assertion was independently shown incorrect and
now checks all regularized loops. The goal remains active. V456 will capture
a function-only stack at240seconds from the committed V454 promotion binary.
All source inputs will remain frozen while that owned diagnostic runs.

V456 outer9752 is live under approved GDB access. It owns the frozen committed
V454 STRICT promotion test, will interrupt at240seconds for function-only
backtraces, then kill/reap the inferior. No source/index edits or other probe
drivers until outer9752 is reaped. All three repositories remain committed;
no production edits are pending. Next unallocated artifact isV457.

V456 outer9752 is reaped with exit0 at240.47seconds. This is a successful
function-only diagnostic, not a passing test. The later stack remains in
Rational Lehmer GCD/reduction -> signed_product_sum2 -> Quad::mul -> tower
reduction -> compact_quadratic_tower -> Hypersolve compact_exact_coefficients
inside sign_at_selected_root -> six recursive quadratic sign frames ->
selected-circle/chord intersections during re-offset Boolean regularization.
V457 will capture later quintic-or-higher query inputs and bounded remainder
widths in a copy-only build. No production edits are pending.

V457 outer10251 is reaped with exit0: the240.03second test timeout is an
intentional diagnostic stop, not a pass. Eleven exact quintic queries008-018
were exported with no unsupported coefficients. All use the same quartic
field Q(sqrt5,sqrt(50-20sqrt5));008-010 are byte-identical to V442000-002.
Queries008-015 share one equation/isolator;016-018 use another selection.
Largest observed remainder widths are about76000bits, all exactly reducible
in the retained tower. Queries010,013,014,015 take about21-24seconds each.
V458 will independently prove signs and compare projective query identities
using Fraction arithmetic and positive leading-unit Sturm-Tarski chains.
No production edits are pending; the mirror still contains the archived V457
diagnostic until the next restoration, and the full goal remains active.

V458 outer48998 is reaped successfully. Independent exact field replay proves
signs for all11 captures: +,-,+,+,-,-,-,+,+,-,+. There are two source-root
selections and10 projective query classes;015 is a negative field-unit multiple
of014, though not a rational multiple. Monic normalization increases raw
coefficient heights1680/3380bits to about5780/11570bits. Raw coefficient
magnitudes are about2^740/2^1480 while retained root brackets carry roughly
512bits. V459 will test certified interval Newton with outward dyadic rounding
on these already-proven singleton inputs, checking every sign againstV458.
This differs from rejected V438, which used only the unchanged initial bracket.
No production changes are pending.

V459 outer48957 completed synchronously with exit0 (recovered from the command/tool record). All11 exact independent interval-Newton queries pass at1024/2048bits in0.015-0.048s each. V460 is a copy-only Rust trial using certified Real dyadic bounds, the existing shared rational interval product, outward rounding and original selected-root authority. It keeps the exact chain fallback and changes no production API. All30 repositories are clean before the trial.

V460 outer90093 is reaped with exit0. All11 captured quartic-field signs pass in0.032-0.064s each (fresh process wall time), using the original Real certified dyadic API and shared rational product; no field-specific export is used. V461 now tests both complete formerly unresolved re-offset cases on the same copy-only filter. Production remains unchanged.

V461 outer60360 is reaped with exit1. The STRICT full composition still times out at360seconds; APPROXIMATE was not run. The11 isolated signs are improved but this does not qualify either full case. Production remains unchanged. V463 will take a function-only stack at240seconds from the frozen V461 binary, retaining its copy-only source hashes. V462 production qualification is only prepared, not launched.

V463 outer4689 is reaped successfully. At240seconds the new stack is LocalFieldElement::divide_after_nonzero -> normalize_local_polynomial -> local polynomial GCD -> common bivariate fiber roots, while ordering selected semicircle parameters in re-offset split topology. The original root-query chain is no longer the sampled bottleneck. This does not support the speculative scalar-scale explanation; V464 was prepared but not run. The certified root filter is now promoted for V462 qualification with three fallback/cancellation regressions and the existing independent generated-point oracle. The two complete re-offset cases remain explicitly unresolved, excluded from the323-case passing scope. Only hypersolve/src/root_sign.rs is changed in production.

V462 outer27449 is live on frozen production. All901 Hypersolve tests and9 checks pass; the323-case geometry scope is now running after compilation. V465 final-source eleven-query replay and V462 review/seal scripts are prepared, not launched. V466 is a prepared later-stage common-fiber input capture; no further production edit or concurrent driver is permitted until outer27449 is reaped. V464 remains unrun because the function-only stack established a different bottleneck.

V462 outer27449 is reaped with exit0. All901 Hypersolve tests,323 selected Hypercurve tests across7 binaries, and9 checks pass. No material slowdown was seen in the completed geometry scope. The two original full re-offset tests remain explicitly unresolved afterV461; V463 places the later cost in common-fiber local-field inversion/GCD. V465 now replays all11 independent captured signs against the final production source, with no diagnostic scalar API. No production/index edits until its outer session is reaped.

V465 outer18815 is reaped with exit0. All11 final-source captured sign replays pass in0.032-0.064s fresh-process wall time; the mirror is restored and all2046 source hashes checked. V467 compares two shorter fillet cases against frozenV454 binaries after V462 observed3.02->4.17s and3.97->4.57s; these increases need paired evidence before the pending commit. Long cases remain close to baseline. V462/V465 qualification passes remain valid, but the earlier broad statement of no material slowdown should be read with this explicit timing follow-up.

V467 outer81039 is reaped with exit0. Three paired repetitions confirm median3.018->4.219s for joined-path family replay and3.969->4.469s for nonlinear linear components. The earlier new filter was doing extra work even where primitive integer chains are already cheap. Production dispatch now preserves that existing path for actual rational coefficients, leaving all general-Real values supported by Newton and the exact chain fallback. The4096-bit fallback regression now has nonrational defining coefficients so it exercises that branch. V468 checks the two timing regressions first, then repeats901 solver tests,323 geometry cases and9 checks if they return near baseline. V462/V465 remain predecessor qualification, not final-source claims. Only src/root_sign.rs is modified.

V468 outer53597 is live; it is also recorded under the functions store key hypercurve_current_driver. Production, archive, mirror and current driver remain frozen. V469 final-source query replay and V468 review/seal scripts are prepared, unrun. V466 common-fiber capture was retargeted to V468 and remains unrun. No commit has been made for the Newton slice yet. Reap53597 before any source/index edit or next driver.

Read-only follow-up for the later bottleneck: HC BezierAlgebraicSelectedFiberAuthorityData2 (bezier_offset.rs ~4014) already retains a STRICT AlgebraicFiberRootRefiner, whose LocalAlgebraicField caches signs/inverses. HC algebraic_selected_fiber_root_predicate_sign (~107937) nevertheless calls a fresh count_bivariate_common_fiber_roots_at_algebraic_parameter (~108138), which constructs a new LocalAlgebraicField for each common-root query. Also its exact_nonzero flag accepts only PredicateCertainty::Exact; Hyperlimit predicate.rs documents Filtered as conservative structural Real facts, not terminal approximation. V466 will log bounded result status/count/certainty to assess reuse and whether certified nonzero evidence is discarded. No change to this path is made yet; collect input evidence before choosing between shared authority, polynomial normalization, or a dispatch repair.

V468 outer53597 is reaped with exit0. Final dispatch passes901 Hypersolve tests,323 selected Hypercurve tests across7 binaries and9 checks. The paired-regression workloads take3.02/3.92s, restoring their3.02/3.97s baselines. V469 now replays the11 independently proved captures against this final source. No source/index edits until its outer session is reaped; review/seal scripts are ready. Both original full re-offset cases remain unresolved; no change was made yet to the later selected-fiber path.

### Committed certified selected-root filtering

Hypersolve547b0d4e9a1b3fe656f50957e5f89c17877ab53b is committed and sealed. V468 outer53597, V469 outer45920, and review3456 are reaped successfully; stage/commit seals completed synchronously. All2046 source hashes, the one staged/committed blob, and30 clean repositories are verified. Passing final scope:901 Hypersolve tests,323 selected Hypercurve tests in7 binaries,9 checks,11 independent captured signs at0.032-0.064s each. The initial1.2/0.5s fillet regressions were measured byV467 and eliminated by preserving primitive integer dispatch for genuinely rational coefficients; general Real coefficients retain interval Newton and exact chain fallback. No public API/carrier/shim was added. Both full re-offset cases remain unresolved. V466 will now capture exact common-fiber inputs and bounded result status/count/certainty from the later local-field GCD path; a240s diagnostic timeout will not be treated as a passing test. The full goal remains active.

V466 exact outer45767 reaped with exit0 (recovered transcript call_JgLQQt8FpPdQaEiHA5w6Cg9c). STRICT timed out at240.03s as an intentional diagnostic stop, not a passing test. One degree20/12 common-fiber input over a degree6 retained root was captured, all215 scalar exports exact. The retained-root interval is strictly above the fiber interval, so diagonal identity cannot explain this selection. Both diagnostic mirror files are restored to committed production. No production changes are pending.

V470 exact relation oracle completed synchronously with exit0; no tested simple parameter identity vanishes modulo the captured defining equation. V471 exact Fraction interval oracle completed synchronously with exit0 and proves the captured predicate strictly positive on the retained fiber box after an exact shift of polynomial coordinates (64-bit coefficient enclosures suffice). Unshifted Horner fails even at2048bits. V472 outer84579 is live on a copy-only Rust replay: reuse the already-built restricted polynomial for the full interval sign before GCD. Production and live source inputs remain frozen. V473 complete re-offset trial is prepared only.

V472 exact outer84579 reaped with exit0. The captured predicate has a certified Positive sign using the already-restricted unit-box polynomial in25ms (0.032s fresh-process wall time). No further root refinement, field GCD, new API, carrier, or arithmetic mechanism was required. V473 now runs both original complete re-offset compositions on that copy-only dispatch repair. Production remains unchanged; live archive/mirror inputs stay frozen.

V473 exact outer33869 reaped with exit0. Both original complete re-offset compositions pass: STRICT114.96s and APPROXIMATE_512114.81s, compared with the prior STRICT360s timeout. The same restriction reuse is promoted to production with explanatory comments; only hypercurve/src/bezier_offset.rs changed (11 additions,16 deletions), no new API or arithmetic mechanism. All30 repository heads match the prior seal. V474 final production qualification now includes325 selected geometry tests and6 directly affected checks; unchanged Hypersolve tests need not be repeated. Sources remain frozen until its exact outer is reaped. V475 final-source captured proof and review/seal scripts are prepared only.

V474 exact outer40182 reaped with exit0. All325 selected release geometry tests in7 binaries and6 affected checks pass. Both original complete re-offset cases now pass in the final production source: STRICT113.76s and APPROXIMATE_512114.56s. The two preserved fast-path workloads take3.07/3.97s; no existing case above0.1s exceeds its previous timing by25percent plus0.2s. V475 now performs the planned independent captured-box proof against final production, with only its test fixture appended in the copy. No production/index edits until that outer is reaped.

### Committed restricted selected-fiber interval replay

Hypercurve 0987b79aa72a020999a97125b2db5a15f6e32e7c is committed and sealed. V474 exact outer40182, V475 outer45111, and review15576 are reaped successfully. All2046 source hashes, the one staged/committed blob, archived executables, and30 clean repository statuses are verified. Qualification passes325 selected release geometry tests across7 binaries and6 affected checks; the independent captured-box proof takes0.032s. Both previously unresolved full extended-fillet re-offset cases are now included and pass (STRICT113.76s, APPROXIMATE_512114.56s). The earlier STRICT run timed out at360s. No existing case above0.1s exceeded its previous wall time by25percent plus0.2s; the two guarded fillet timings remain3.07/3.97s. No API, carrier, arithmetic mechanism, or compatibility shim was added, and the patch removes five net lines. No push occurred. The broader goal remains active. V476 is the next public diagnostic: regular vs stationary parabola parameterizations, a one-sided cusp at an authored endpoint, and a constrained interior stationary contact with an independently proved oriented rational tangent circle, under both policies. No further production change is pending.

V476 exact outer77463 reaped with exit1. The diagnostic compiled against the V474 normal library artifact with its frozen hash and all2046 qualified source inputs verified. Both regular parabola controls pass. Its stationary reparameterization, the authored-endpoint one-sided cusp, and the interior stationary contact all report Blocked(Boundary) under both STRICT and APPROXIMATE_512 (six failures, each0.003-0.004s). Each intended tangent circle and oriented contact is independently rationally certified in the probe. This is a new representational/semantic closure defect, not a timeout or a passing qualification. Production remains committed and all live inputs are released; next unallocated artifactV477.

V477 completed synchronously with exit1: GDB could not resolve the inlined const ExactCurveError::blocked constructor, so none of its three runs captured a stack; all debugger/inferior processes are reaped. V478 exact outer16103 is live on a copy-only diagnostic build: error.rs temporarily makes that private constructor non-const/non-inlined and prints at most16 function-only standard backtraces for Fillet/Boundary per test process. Production remains committed. Archive, mirror, fixture, driver and all production inputs are frozen until outer16103 is reaped. The diagnostic never formats recursive Real or geometry payloads. Next artifactV479.

V478 exact outer16103 is reaped with exit1: the diagnostic library built successfully in76.92s, but the standalone fixture link used release rather than release/deps for dependency lookup. V479 exact outer71464 is reaped with exit0 after correcting only that link against the frozen V478 library. All three function-only traces place Blocked(Boundary) in fillet_offset_centers -> solve_carrier_fillet_corner. Source inspection identifies its bare supporting_line_incidence_with_direction call (curve.rs~7103); the shared kernel requires raw source speed nonzero on the whole unit interval (bezier_offset.rs~116851), explaining even unrelated endpoint failures. Existing relation_to_supporting_line_on_regular_range and point_evidence_on_regular_range already retain cancelled source hodographs and oriented one-sided frames. The next repair must preserve regular-range/cut-side ownership through center enumeration, points, tangent signs and fillet reconstruction; removing the global check alone is unsafe. The copied error.rs is restored to production; no production change or owned process remains. Next artifactV480.

V495 outer74084 and review65745 reaped0. Hypercurve commit
9aad1db6734065752a9b6d20648aceecddfebeae (Keep fillet incidence inside the
retained source domain) is sealed against all2047 input hashes and3
committed paths.353 selected release tests,8 binaries,6 checks pass;30
repositories clean. No recorded performance regressions; full reoffsets
115.16/114.26s. V496 is now the only running driver: exact outer82723,
private copied-source regular-pair diagnostic,3 probes including a
center-cusp tangent-sign counterexample. Production, mirror, archive,
fixture and driver remain frozen until exact outer82723 is reaped.

V497 exact outer94382 reaped0. All4 normal production probes pass both
policies in .004-.032s, including the formerly wrong certified cross sign.
Removed one constant arithmetic assertion from its proof comment before
qualification; no production behavior changed after the probe. V498 broad
qualification is live, exact outer99281:357 selected tests,8 binaries,
6 static/downstream checks. Only bezier_offset.rs differs from committed
9aad1db6. Inputs are frozen until exact outer99281 is reaped. Review and
seal scripts for V498 are prepared but unrun. The stationary pair fillet
domain dispatch/frame repair is still explicitly open; this commit target
repairs the shared regular-range tangent evidence prerequisite.

V498 exact outer99281 and review4167 reaped0. Committed/sealed Hypercurve
07b77cbe2512f5293713344abc239777797581f2, Replay parallel tangent
orientation at each exact contact.357 selected release tests,8 binaries,
6 checks;2047 source hashes and the committed blob verified;30 repositories
clean. No recorded performance regressions; full reoffsets113.01/112.96s.
No outstanding production changes. V499 is now the only running driver,
exact outer97045, private copied-source exterior-domain and general-pair
sign diagnostics. All production/mirror/archive/fixture/driver inputs stay
frozen until that exact outer is reaped. Next unallocated artifact500.

V501 exact outer 35767 and review outer 70553 reaped with exit 0. Committed and sealed Hypercurve 689250e0bbb144fa68b3d22d124a68103b190d06: Keep regular pair intersections on their retained source domains. 364 selected release tests across eight binaries, six checks, 2047 source hashes and committed bytes verified; 30 repositories clean. No recorded performance changes; full reoffsets 113.61/113.41s. Original stationary pair fillet dispatch and source-frame repair remain open. Next driver is V502 private mixed-domain and normal-sheet baseline.

V513 exact outer18087 reaped1. Both requested ranges were proved inside unit; the native kernel was complete with zero contacts, one overlap and no products. Rebuilding that existing correspondence failed Unsupported in select_parameter_component_in_domain (checkpoint3), not frame or GCD construction. Audited all regular-range production callers: support intersection clips correspondence through analytic_overlap, region intersection/arrangement through clipped_overlap_ranges; the in_domain caller uses this route only for RetainFinite. Pending repair preserves native overlap evidence with its explicit enclosing-evidence contract, clips isolated contacts internally, and keeps arbitrary retained domains on the shared solver. Promoted the V512 two-contact test and added an end-to-end Curve2 overlap test for u=v^2 with positive, disjoint and singleton restrictions, reversed traversal and both orders/policies. V512 normal-production probe has16 cases. V511 broad/review scripts retargeted to V512 and368 tests; all unrun.

V512 exact outer54940 reaped0: all16 normal-production probes pass. Third-generation analytic fillet restored at.414s; PH branch configurations2.817s, general exterior1.616s, general sign1.015s; two new clipping tests .008s each. V511 broad qualification starts next with368 cases,8 binaries,6 checks. Production/mirror/archive inputs freeze through its exact outer reaping; only bezier_offset.rs is pending.

V511 exact outer76932 and review11248 reaped0. Committed/sealed Hypercurve24751d38e1c8b0e48a435192c3590c8e9458db4b: Share retained rational pair domains and source frames.368 selected release tests,8 binaries,6 checks,2047 source hashes and1 committed blob verified;30 repositories clean. No recorded performance changes; full reoffsets113.61/113.71s. New four regressions cover exterior mixed domains, exterior normal sheets, two-axis native contact clipping, and end-to-end nonlinear correspondence clipping including disjoint/singleton restrictions. Original stationary pair fillet dispatch remains open. V508 public quadratic-line-chart diagnostic is next; V514 PH tangent audit follows only after V508 reaped. V515 generic source-frame fixture is prepared but production unchanged.

V518 exact outer29934 and review4676 reaped0. Committed/sealed Hypercurvefad45f2b26d0c7a9c548fd9882a5bdd832105270: Preserve retained source frames at rational pair endpoints.373 selected release tests across8 binaries,6checks,2047source hashes,3committed blobs verified;30repositories clean. No performance flags; fullreoffset113.86/113.76s vs113.61/113.71s baseline. New general factory retains native/selected/recursive scalar authorities and explicit pole guard; original-source one-sided endpoint tangent replay handles missing or both-zero native signs. Overall goal active, stationary public parallel-pair fillets still open. V520 empty-ray copied-source counterexample next.

V523 exact outer46877 and review99356 reaped0. Committed/sealed Hypercurve061585222021cfbf56a531d4ef9a9dd3804e692b: Omit empty incident charts before component selection.19focused release domain/family regressions,6checks,2047source hashes,2committed blobs verified;30repositories clean. No timing flags. Regression preserves unconstrained NeedsConstraint, either-axis exact contact selection and finite endpoints with empty rays on either axis/direction, unit/exterior domains, and independently represented sqrt(2) barrier equality. Public stationary-pair outcomes remain to be remeasured viaV522; full goal active.

V525 exact outer34409 reaped0; read-only review completed0 in the initial exec (no running review session). Committed/sealed Hypercurve374dcfb8bd888ce00adcfe64de9a9925b8148576: Cover stationary pair fillets through quadratic line charts.30public stationary-fillet release tests, all/no-default-feature Clippy and formatting (3checks),2047source hashes,1committed test blob verified;30repos clean. New2tests each cover polynomial stationary reparameterization and one-sided cusp with a quadratic line, both path orientations, underSTRICT/APPROXIMATE_512 (8parameterized cases total). Each selects the independent exact center and verifies contacts. Production repair06158522 remains unchanged. No timing flags; newtest groups .464s each, existing offset compositions18.338/18.337s.

Current next work:8 remaining public quadratic-line parallel-pair failures fromV524:6TrimOnly Boundary and2interior-cusp TrimOrExtend falseempty/UnsatisfiedConstraints. The4 endpoint TrimOrExtend failures fromV508/V516 are fixed and permanently covered, now also reversed. Do not report full closure or mark the overall goal complete. All drivers/reviews/formatters are reaped; no production changes pending. No agents or pending approvals. Next free artifactV526.

2026-09-27 follow-up: Hypercurve a03c08bb shares regular one-sided source frames with All/FirstComponent queries; e932c8c8 retains exact numeric endpoint ownership through root, selected-axis and finite/incident component replay; a0492bd0 preserves regular cell decomposition under range reversal. V52664, V52768, V52819 release cases plus6checks each passed; all exact outer/review sessions reaped and30repos clean. Continuous stationary families still require explicit center/contact constraints and those constraints retain original parameter correspondence, including owned endpoints. Four public quadratic-line stationary endpoint TrimOnly failures fixed; four interior pair cases remain. Full integration notes in stationary-pair-cell-integration-next.md; overall implementation goal remains active.
