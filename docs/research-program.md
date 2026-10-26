# Research program, October 2026

The research program is paused at the maintainer's request on 26 October
2026. R19 is the final admitted experiment in this run; the remaining entries
keep their open scope. R20 received read-only preparation, with no candidate,
measurement campaign or admission. The exact curve and BRep closure plans and
agent instructions are retained as requirements, not completed implementations.
The milestone narratives below describe their original validation checkpoints;
publication status is maintained in the [evidence index](evidence-archive.md).
The separate repository-wide pre-push checks are part of closing out this run.

This ledger records the evaluation of every project in the consolidated
research handoff of 22 October 2026 (`docs/research/csgrs-remaining-research-projects.txt`
in the working tree that received it, baseline `e2def0b69f`). Each project is
rechecked on the current code, reproduced or measured, and accepted only when
it benefits the priority order: exactness, then completeness and operation
success, performance, low memory, binary size and source size. Rejected and
inapplicable leads are recorded with their evidence so they are not proposed
again. Detailed evidence lives in the owning crate's `PERFORMANCE.md`,
`FUTURE_WORK.md` and `CHANGELOG.md`.

Status values: **accepted** (merged), **rejected** (measured or proved not
beneficial), **not applicable** (the lead does not reproduce or is already
solved), **deferred** (requires work outside this program, with the reason),
**pending**.

| ID | Project | Status | Outcome |
| --- | --- | --- | --- |
| F01 | Reproducible completion and resource acceptance | accepted | 2aea178386. The successive-operations harness times step, validation and drop separately, has an `--untouched` mode, counts allocator calls and the live heap at each step, documents that `realloc` counts growth only, and dumps raw rows with revision and toolchain (`SUCCESSIVE_OPERATIONS_RAW`). Validated and untouched runs give identical rows, and 197 of 200 steps allocate identical bytes. Validation does not measurably warm later steps. RSS sampling and an alternating A/B driver were not adopted, with reasons. |
| F02 | Validate cold-imported expression domains | accepted | 3c1ab5ad7d. Reproduced: `Real` import accepted undefined operands, so `asin(pi)` and `atanh(pi)` were certified Positive, `NthRoot(sqrt2-2, 3)` and `0*sqrt(-pi)` certified Zero (also through STRICT `orient2`), some sign queries panicked or never ended, and a forged hyperbrep cuboid archive validated as trusted. Import now proves every partial operation's operand domain, operands first: exact identity, the 32-node binary64 filter, the pi-Laurent form for tan/cot poles, then the 2^-2048 sign floor that native `Real` operations use. Refuted is `ParseError`, undecided `UnknownCertificate`. Allocations unchanged; decode time +2.5-12%, worst row `asin(1/pi)` +1.2 us. |
| F03 | Stack-safe deep expression teardown | rejected | 69d5210175 records it. Reproduced: dropping an 80,000-term sum on a 2 MiB stack aborts. An iterative `Drop for Node` fixed it, but all five variants regressed small-value drops (best: +13-14% instructions, +16-30% drop time on fresh radical and mesh values). Remaining design: a saturating depth hint written at construction (FUTURE_WORK). |
| M01 | Bound optional output-coordinate sharing | accepted | 9952d7a7c6. Reproduced: 4,096 dyadics in one window took 8,386,560 comparisons and as many proofs (196 ms). Each coordinate now meets at most 4 groups, 8 structural comparisons and 2 proofs; rationals are never proved; all-rational outputs reserve nothing; the pass is infallible and keeps exact expressions when it declines. Fixture: 0.66 ms, 0 proofs. Corpus rows and allocation unchanged at 0.1 MB. Hypermesh has no cancellation, so the bounds are fixed counts. |
| M02 | Regression tests for the new mesh optimizations | accepted | 987e441c6c. Tests for (a) grouped broad phase against an ungrouped oracle, (b) retained versus rebuilt non-manifold outputs, (c) apex fans over every apex and rotation, (d) an undecided Cramer division falling back to interpolation, (e) retained supports on shared symbolic coordinates, (f) start==end in crease crossings. No defect found; (a) and (f) fail under their mutations. |
| M03 | Embedded-complex certificate on approximate outputs | accepted | 74f8c951ea. Counterexample: an APPROXIMATE_512 difference across a 2^-600 wall recorded a certified embedded complex, and a later STRICT union reported Certified where the rebuilt rows were undecided. The fact now carries the arrangement's certainty and is consumed like `valid_pwn`. Certified outputs and corpus chains are unchanged. |
| M04 | Reuse retained incidence proofs before numerical zero tests | accepted | c70769dbb8. Reproduced: `cylinder(8,40,4)` minus its copy rotated 0.0001 degrees ended `PredicateUndecided` because three points built on two coplanar-decided cap planes and their shared edge were tested collinear numerically (a 148-node zero). Arrangement points now keep every construction identity; on-plane and on-line facts from source vertices, edge-plane crossings, plane triples and plane classes decide face containment, point-on-constraint, radial coplanarity, the seed ray origin and (through new hypertri `CollinearPointFacts`) CDT orientation. Facts only decide "on"; positions and orientation are still compared. Also fixes `boundary_line_for_segment` naming the previous edge. All 200 successive-operation steps keep their rows; allocation 63,408 -> 45,125 MB (-28.8%), icosahedron unions 3,972 -> 1,998 MB. Skew-axis and binary64 compound rotations still need scalar identities (angle addition). |
| M05 | Fixed-field normalization behind `Real` | accepted in part | 955af21659 and 381732223f. Measured: coordinate depth stays flat on the 30/45/10-degree chains, but re-expanding arithmetic history into the pi-Laurent normal form is 42-67% of step time; the remaining rotation blockers were resource limits, not missing identities. (1) Hyperlimit's sign cascades try a final exact relation attempt (degree 12, 32,768 charges) after the towers decline, so skew-axis and compound-Euler near-coincident cylinder Booleans complete certified; cost under 0.001% on the corpus chains and +0.05 to +3.5% on suites that end undecided by design. (2) Segment-plane crossings store derived coordinates as their smaller exact normal form: 6/8-gon drills and 30/45-degree cuts -42 to -52% instructions and -46 to -57% allocation, corpus 45,125 -> 42,951 MB; rows change in expressions only (MIGRATIONS). The field descriptor was not built; binary64 compound rotations still need degree 24 and about 4M charges. |
| M06 | Coplanar scheduling beyond first-use fans | accepted in part | 030bcf8962. Measured: with the new operand first (`tool.union(&body)`), coplanar-overlap pairs grew to 26 and 8 unions allocated 9,714 MB against 2,755 MB in corpus order. Reduction now ranks vertices of operands that retain an earlier Boolean's facts first, so corpus chains are byte-identical and the swapped chain falls to 4,926 MB. A retained per-vertex age, the plane-class region view and alternating-end growth are deferred. |
| H01 | Chart-qualified restricted-support view | not applicable | 327288e1a1. A public-API probe over every curve family (line, arcs, Bezier, conics, rational cubic, spline split at a knot, NURBS, degree-elevated and retraced curves), cut at rational and algebraic parameters in all traversal combinations, operand orders and both policies, found no failing or undecided restrict/reverse/restrict or replay case. No abstraction was retained without a measured benefit. |
| H02 | Shared support-pair incidence and witnesses | deferred | 327288e1a1. Same probe: no support-pair defect or measured cost names a family to migrate first; deferred until one does. |
| H03 | Incidence-relation restriction and dimension changes | not applicable | 327288e1a1. The [0,1]/[1,2] clipped-overlap singleton is kept and replays to the shared point; an overlap plus an isolated contact coexist (arcs [0,270] and [180,360] degrees); point-touching region Booleans give the expected areas. |
| H04 | Point incidence below topology | accepted | d6373655ef. Reproduced: Easyduino stm32f103 ended in a typed RealSign termination after 169.6 s. Minimized to three of 655 copper profiles: a pad's corner arc meets a route edge and a zone-cut fragment sharing its support allocation at one point, computed with independent radicals. Two contacts on one circle and one shared line are now decided equal or distinct by the exact sign of (P - C)·u, before the general fallback. The board now completes (1,389 findings); the other five boards are unchanged. The broader point-incidence service was not needed. |
| H05 | Polynomial-branch sections through split continuation | accepted | 9b96cfcba2. Reproduced: an unmapped graph branch of y^5 + 5y + 5 - x against an authored rational vertical line returned a typed `Unsupported` blocker from `intersect_curve` and `intersection_topology` in both orders, and `split_curve` lowered polynomial cuts through Bezier spans. Sections now use the branch's own fiber point (stored event endpoints at boundaries), the line parameter is an affine form of that point, and graph branches split in their source chart. Vertical-tangent and cusp events stay unclassified. Shared paths allocate identically; the new quintic section is 654 allocator calls. |
| H06 | Polynomial queries beyond vertical rational sections | accepted in part | 4d95675657 accepts milestone 1: any line with rational endpoints. Candidates are the square-free roots of the substituted polynomial; a root is kept only when the branch's own fiber point lies on the line, tangencies publish a zero cross sign, and a line inside the zero set stays a typed blocker. Irrational or selected lines, mapped branches and branch pairs (milestones 2-3) remain deferred. |
| H07 | Finite-fiber topology domains and algebraic coefficients | deferred | 8b571c16f2 records it. Rootless x-content already succeeds; F^2 and (2y-1)^2 F give `RepeatedComponent`, (2x-1)F `VerticalComponent`, (y-1)F `BoundaryComponent`. Square-free reduction is exact, but admitting it needs a published per-branch multiplicity record (hyperbrep relies on `RepeatedComponent` for tangential plane sections) and a vertical-component carrier: new public API on a crate prepared for release. |
| H08 | Polynomial supports in filled arrangements and BRep trims | deferred | 4d95675657 pins the state: `CurveRegion2` construction and loop orientation with polynomial boundaries return typed `Unsupported`. Depends on H06 milestones 2-3 and H07; no separable beneficial piece. |
| H09 | Two-sided coefficient-field joins for polynomial roots | rejected | 1001b93a63 records it. Reproduced: `recursive_polynomial_sign_joined` returns `Uncertain(Unsupported)` for a query over an independently introduced radical. A source-union join decided every such query locally, but two traced release suites and all four workflows reached it only from the new test, so it was removed; the regression is kept as an ignored test. |
| H10 | Local selected-root equality without global promotion | accepted | 2429241e66. Reproduced: overlapping local polynomial roots took 512 more bisections and then promoted both globally (64 pairs, 31 s of suite thread time); roots of f and f - 2^-600 stayed undecided. A root whose bracket isolates one simple root of g with opposite endpoint signs now orders the other value by the exact sign of g there, and each local root retains up to 4 native roots proved exactly equal. `chamfered_quadratic_bevel_stroke_round_trips` went from killed at 2,400 s to 61-67 s; its allocation calls fell from 5.48 G to 0.84 G (certificate alone: 539-647 s). f vs 7f: 2.79 M -> 17.7 k allocations. |
| H11 | Selected fillet-center authority through consumers | deferred | 1001b93a63. In the filleted NURBS round stroke, the center reaching `selected_parallel_normal_parallel_intersections` is already a degree-214 `BezierParameter2`; the cost is selected-fiber isolation over that field (one call 110 s, another past 40 min). Widening the consumer cannot help; local center authoring waits on H15. |
| H12 | Tangent and multiple-root events as local certificates | deferred | 1001b93a63. Refilleting stalled in certified two-radical sign refinement; it now completes through the H13 replacement (670ea140e0). A new fillet -> round stroke -> rebuild -> reverse -> refillet -> erode workflow passes for a line/line corner; the arc/line corner blocks because the rebuilt region does not keep the fillet's tangency certificate (ignored test). |
| H13 | Lazy radical leaf signs with modular nonzero filters | rejected; replacement accepted | 670ea140e0. Captured leaves are univariate degree-27 values in one selected source; the existing modular coprimality certificate already proves them nonzero, and the time is in certified refinement (exact rational interval Horner and Bernstein restriction, 21 of 24 samples), not leaf expansion, so a modular norm before expansion would not help. Both enclosures now run over integer common denominators (positive multiples, identical decisions; 1,152-polynomial test). `refilleting_a_filleted_vertex_completes`, killed at 1,508 s before, now passes in 163-296 s. |
| H14 | Modular/CRT selected-field GCD with maximality certificates | rejected; replacement accepted | 5b2b53f042 and 281ee3690f. The 47.6 s equality in the chamfered case is not a gcd: it is the exact remainder of a degree-192 eliminant (644-bit coefficients) by a degree-6 local relation, which is identically zero, and modular images cannot prove zero. Exact replacements: the sign remainder follows the dividend's Horner sequence (tower products 19,635 -> 2,430), rational quotient-ring remainders skip the dividend's content gcd, and local-field reduction uses the same integer remainder. Remainder 48.4 -> 1.35 s; `chamfered_quadratic_bevel_stroke_round_trips` 62.8 -> 13.4 s. |
| H15 | Selected-factor degree before eliminant cost | rejected | a4a608bd19 records it. The degree-214 fillet center is square-free over Q with factors of degree 2, 8, 40 and 164; only the irreducible degree-164 factor changes sign on the isolating interval, so the intrinsic degree is 164. Substituting it at the kernel or at the resultant did not make the round stroke complete, and the stack has no factorization over Q. Local selected-fiber center authoring (H11) remains the direction. |
| H16 | Direct homogeneous degree elevation | rejected | 681213db25 records it. A kept source retains every intermediate degree (1.24-1.87 MB at D=64 against 42-207 KB for the target), and the direct formula gives identical controls and is 2-17x faster for one cold request at D>=32, but 1.7-8.4x slower for successive degrees. No real path elevates far: 0 calls on all six Easyduino boards; the highest production target in the release tests is degree 5. |
| H17 | Shared Boolean topology selection views | rejected | 681213db25 records it. No board makes a nontrivial `boolean_regions` call; in the all-four bench the shareable part (fragment walk, clone/reverse, graph) is at most 2 of 39-52 samples. The real output costs (kept-arc line-image tests, tangent-successor and loop-role certification, line merging) apply to single Booleans and are recorded as leads. |
| H18 | Deferred carrier preparation after broad phase | accepted | 3fbb0867dc. Reproduced: on the default Easyduino pipeline 99% of cross-operand Bezier carrier preparations were never read (nano 553,901 prepared, 5,473 read), mostly in CAM-audit unions of the copper aggregate with one feature. Infallible preparations now happen on first use by a surviving pair or self context; fallible ones stay eager and in order, so errors are unchanged. All six boards keep their findings; allocation calls -0.7 to -2.7%, peak live bytes -0.4 to -5.6%; callgrind on nano 432.03 G -> 413.78 G instructions (-4.2%). |
| H19 | Bounded span-pair preparation | not applicable | 681213db25 records it. Region carriers are single spans, so every `prepare_rational_pairs` call from region Booleans is 1x1 (boards, pathological bench, 1,638 unit-test calls); direct multi-span intersection already rejects certified-disjoint span pairs first (64x64 spans: 69 ms, 3.6 MB transient). |
| H20 | Retained authority versus accelerator memory | deferred | 681213db25 records the audit. In the small-region probe, clones cost 32 B and repeat runs return to <=208 B after drops (no cycle found); the nano board holds 80 MiB live after materialization and 111 MiB after DRC, with pipeline peaks set by transient work. By allocation site, exact rationals are 30% and hyperreal retained rational derived forms 17% of live bytes; a retention policy for those scalar caches belongs in hyperreal and is recorded as a lead. |
| C01 | Exact linear profiles through sweep, twist and revolution | accepted | 12262e54e6. Reproduced: sweep, twist and revolve sampled every profile to binary64, losing exact volumes and authored vertices; a 2^-80 sliver swept to a certified closed surface of zero volume; partial revolution at negative radius failed. All three now consume exact line profiles (as extrusion did), orient side walls from exact signed area, and decide radial signs. 15 tests, 13 failing before. Binary64-exact profiles give byte-identical rows. A 32-gon full revolution is 50-65% slower because its vertices are now the exact cos/sin values. |
| C02 | Certified monotone non-affine fixed-plane sweeps | accepted | 4e4c0c70ab, with 32e925ff42. Reproduced: z = (t+t^2)/2 gave `NonMonotoneSweepPath`, and a straight sweep along a degree-elevated path failed with `DisconnectedCurvePath` (the prism recognizer mixed side-patch charts into the cap frame). Fixed-frame sweeps are now admitted when D = N'W - NW' > 0 on [0,1], proved by Bernstein coefficients or an exact Sturm count after the affine identity; sections use the path's selected contact. Stationary and backtracking progress and moving axes stay rejected. 13 previously successful archives are byte-identical. |
| C03 | Placement covariance through BRep chart transport | accepted in part | 68c7c38442. Reproduced through public APIs: a sweep whose cap use shares the side patch's pcurve (pivots 0 and 1) validated and replayed but reflected to `UnsupportedTransform`. Non-axial reflection now keys transported pcurves by (source pcurve, pivot); covariance tests check T(A op B) = T(A) op T(B). The broader affine-image family is deferred. |
| C04 | General modeling domains with explicit semantics | accepted in part | 4f89616d66 and 01d997c3ab. (a) Zero-size primitives already return documented canonical empty geometry and other zero parameters typed errors; `extrude_twisted` now reports `ZeroParameter`/`NonPositiveParameter` instead of an undocumented `InvalidArguments`, and both end scales zero taper to an apex. (b) Reproduced a silent overcount: loops on opposite sides of the axis were revolved separately, so a full turn gave a closed mesh with winding number 2 (six-volume 264 instead of 228) or a `Geometry` error; full turns now revolve the profile's exact fold, one-sided rows hash identically. Partial turns beyond a half turn keep the overlap, now documented and pinned (needs a 3D union). (c) Loft docs now name the diagonal side split and its errors. (d) Sweep frames were already named. (e) No API fakes contact conversion; a BRep edge/vertex-contact union test is added. |
| C05 | Enclosure and blend workflows | accepted in part | 59667cd53d. The constant-section enclosure workflow works exactly through four public calls and is now pinned by `tests/enclosure_workflows.rs`: walls and windows, OBJ round trip, pocket -> opening -> restoration -> recut over 4 generations, wall thickness down to 2^-80, explicit zero conventions, clearance violations, corridor splits, bevelled corners (inner area 54+4*sqrt2). Round-cornered base-union-walls takes 42.6 s in release because coplanar caps triangulated differently go through hypertri constraint recovery (recorded in hypermesh FUTURE_WORK); 3D edge blends and shelling stay blocked on H11/H12. |
| C06 | Exact/approximate exchange continuation | accepted in part | 4e016a2ba5 and f229f472d8. STEP lead does not reproduce: 1/3 gives `NonTerminatingDecimal`, values without a rational view `NonRational`, bad unit factors and off-grid inch coordinates are rejected, and no partial artifact or binary float is emitted. Root AMF export, however, rounded silently to 17 fixed fractional digits (all 8 vertices of a 2^-70 cube at the origin); it now uses the shared exact `decimal_f64` formatter like OBJ, PLY and ASCII STL (`cube(2)` byte-identical, `sphere(4,20,10)` 43,099 -> 58,089 bytes). The per-entity STEP preflight report is deferred: new public API that improves diagnostics, not exactness. |
| C07 | Enclosure evidence, approximation metrics, machine projections | accepted in part | e342c62168. Reproduced: `Curve2`/`CurvePath2::bounds()` return control-hull boxes, so the cubic (8,0),(9,3),(11,3),(12,0) is rejected against y <= 5/2 although its maximum is 9/4, and the answer depends on where the curve is split. New `box_containment` returns `Contained` or `Escapes` with an exact witness point (from endpoints, derivative roots, arc cardinal points, certified monotone rational spans), or a typed blocker; split- and traversal-invariant; hull fast path unchanged (0 allocations warm). Hull subdivision was rejected (201 tests, 10.7 ms, undecided at tangency). Fixtures 2-4 rechecked with no conflation found; a per-curve fidelity map is future work. Alumina branch `agent/exact-travel-containment` (5b5bd5b) decides its travel envelope with `box_containment`. |
| S01 | Cross-predicate bounded normal-form reuse | deferred | 42aec58702 records it. In quadratic chains 80-85% of pi-Laurent node visits re-expand an atom-free subtree already completed, over 99% within one Boolean; a verified thread-local prototype gave -52 to -55% instructions when owned by the whole Boolean but only -4 to -5% when reachable from Hypermesh's own sign calls, because 84-90% of expansions come from Hypertri's triangulation through Hyperlimit predicates. Reaching them needs a workspace in Hypertri's evaluator and about eight Hyperlimit predicates, an API decision beyond the research's minimal entry point. Maintainer decision 2026-10-24: not adopted for now, since the M05 compaction captures most of the cost; the layer-by-layer design and contract are recorded in hyperreal `FUTURE_WORK.md` for future consideration. |
| S02 | Non-expanding certified sign preflight | rejected | 220e84e817 records it. Traces: sign expansion is 39-52% of CPU on the root successive-operation chains, but the existing filter cannot enclose any of those graphs (radical constants 63%, trigonometric 26%, `Sqrt` 12%). Only hypercurve's Boolean tests hit it (565 of 132,820), saving 10 ms against 135 ms of filter attempts. A useful preflight first needs certified filter enclosures for those node kinds. |
| A01 | Unified operation receipts | accepted in part | 6a0ae15dbd. The composition-defect lead does not reproduce: with the angle-addition zero as an undecidable fixture, strict Booleans after an approximate union report Certified only with rows identical to a rebuilt copy without retained facts, outputs keep `approximate_512_consumed` in construction evidence, an approximate Boolean reusing the M03 embedded-complex fact reports the terminal, and `Provisional::certified()` is `None` for approximate regions. Added `GeometryCertainty::combine` and `GeometryOutcome::and_then` for dependent operations, documented that `map` cannot see decisions inside its closure, and pinned the cases in `tests/receipt_composition.rs`. Unifying the four one-byte certainty enums was rejected (about 30 lines, no measurable code, public renames in five crates). Open: receipts for hypercircuit's explicit-policy queries, and hypercurve's terminal-closed loop consumers (FUTURE_WORK section 12). |
| A02 | Explicit planar domains and classifications | accepted in part | 61da737db3 accepts (c): OBJ, DXF and VRML face import projects along any certified nonzero normal component when the dominant axis is undecided; a VRML face rotated by pi/4 to 200 digits failed under STRICT before. Rows of decided faces are unchanged; release binaries shrink by 4-8 KB. (a) rejected: `curve::try_transformed_with_context` already is the checked affine adapter. (b) rejected: `contains_xy` maps `RegionPointLocation` one to one, and `classify_point` is public. |
| A03 | Fewer accidental API cross-products | accepted in part | 2e35417d96. Cap and side-wall assembly duplicated across `try_extrude`, `extrude_twisted`, `try_sweep` and `revolve` now goes through shared `emit_section_caps` (each operation names its capped sections) and `emit_section_walls` (open, closed or apex end), keeping each operation's frames, orientation and validation; extrusion's and revolution's own walls stay. 79 fixtures, including C04's apex tapers and fold revolutions, give identical rows; WASM code section -2,073 bytes (-1,810 after `wasm-opt`), source about +13 lines, times within noise. Not changed: the twelve `try_`/plain wrapper pairs (needs a naming decision), the SDF/metaball sampling entry points (their split states a per-call diagnostics contract), and `curve::circle` (documented sampled polygon, OpenSCAD name). |
| B01 | Dependency features at consumer boundaries | accepted in part | 064e1974ab and f2f479b660. `image-io` now requests no codecs (raster tracing takes a decoded `GrayImage`; the README render example gets PNG as a dev-dependency): default build 146 -> 84 packages, 703.9 -> 577.4 CPU s, `Cargo.lock` 616 -> 572 packages, linked artifacts effectively unchanged; recorded in MIGRATIONS. `uuid` is compiled on wasm32 only for `dxf-io` and `gerber-io`. Rejected: dropping svg/amf/gltf/sdf/serde_json from `wasm` (+511 bytes deployed, removes implied APIs). Sibling crates already have `default = []`. |
| B02 | Root, native and WASM size attribution | accepted | da84f70aef. `benchmarks/size-harness` measures primitive, composed and raster root consumers, `libcsgrs_ffi.so`, a static C consumer (plain and `--gc-sections`) and the WASM bindings, with section, symbol/debug, stripped, gzip/brotli and `wasm-opt` sizes; repeat builds are byte-identical. In `composed`, 63% of `.text` is Hypercurve. Opt-level z is 12-20% smaller natively and 17-27% in WASM code but 1.34-1.46x slower than s; `embedded` keeps z because it targets microcontroller flash (maintainer decision, 2026-10-23). |
| T01 | Downstream CAD/CAM workloads as acceptance gates | accepted | b8b231f23a and 759a1f674d. The eight SynapsCAD and exactCore workload families are now root tests (`tests/downstream_exact_inputs.rs`, `downstream_profiles.rs`, `downstream_solids.rs`) with exact oracles: literal bounds and volumes, algebraic region areas, empty offset collapses, a twisted holed extrusion with exact sections, the viral planter with every child accounted for, and line and circle relations against Cramer and discriminant oracles. Family 5 exposed a hyperreal gap: sin^2 + cos^2 = 1 was unprovable for rational turns beyond the cyclotomic order limit, because exact cos(pi r) is stored as sin(pi(1/2 - r)). The ring recognizer now pairs complementary sines and cosines, so strict Booleans of rotated cubes decide; worst declined query +9.6% bytes. Near-coincident rotated cylinders still stop at an incidence tested only numerically (M04), recorded in hypermesh FUTURE_WORK. Alumina fixtures are left to C07. |


## Follow-on program, 25 October 2026

The second handoff, `research/csgrs-remaining-research-projects2.txt`, adds
R01–R30 and nine bounded API alternatives. It supersedes conflicting future
work gates, without changing the historical results above. Baseline:
`3cca6a894858c8b98497a3cfe81cff5e17e1a7f2`. Priority remains exactness,
completeness and operation success, performance, memory, binary size, source
size; Hypercurve has special priority. A pending entry is not an evaluation
or an acceptance claim. The full input-domain, adverse-control and continuation
requirements remain in the handoff and `admission.md`.

R01 starts with a public-API production reproduction under P17/P18/P27 in the
CSGRS modeling layer: a full-turn negative rectangle must preserve its sampled
set when an unrelated positive rectangle is added. R25 accompanies each
experiment with retained commands, source/build identity, outcomes and raw
evidence; historical summaries are not reconstructed as historical samples.
The first execution tier is R01, R02, R03, R04, R05, R08 and R11, with R25
throughout. Remaining projects and advanced alternatives stay in scope.
S01 remains explicitly not adopted; the separately owned ABC corpus is not
part of this program's execution or coordination.

| ID | Experiment | Current disposition |
| --- | --- | --- |
| R01 | Make segmented revolution independent of unrelated profile components | in progress; mesh milestone kept for exactness, with measured cost. BRep support identity/parameter correspondence remains open. See admission below. |
| R02 | Keep fillet tangency authority through rebuilt boundaries | reproduced; two broad carrier-retention prototypes rejected. Keeping selected-circle tangents leaves the same erosion blocker; forcing retained fillet publication regresses refilleting to Unsupported. A source-bound certificate for the represented fillet remains open. |
| R03 | Author selected fillet centers without global eliminant flattening | gate reproduced: the unchanged NURBS fillet/round-stroke test exceeds an external 300 s budget. Local center authoring remains pending; no solver replacement accepted. |
| R04 | Preserve exact members of mixed nonlinear modeling profiles | repair kept after all 39 final checks pass on unchanged retained source, including the six original downstream profiles, symbolic-hole gate, full owner/modeling/BRep/PCB suites and real binding consumers. The isolated 38,400-result triangulation comparison preserves exact rows and allocation fields; adverse cold timings remain disclosed. No speedup or release claim. |
| R05 | Extend polynomial queries by selected and mapped source families | admission withdrawn for the shared cut-point experiment: exact definitions and split/topology costs improve, but the unchanged section control regresses. It does not satisfy the literal latency gate in the Hypercurve handoff. The production change has been removed. Selected/algebraic lines, mapped queries, branch pairs and common components remain open. |
| R06 | Publish component and multiplicity authority for polynomial topology | pending |
| R07 | Carry polynomial boundaries through filled arrangements and BRep trims | pending |
| R08 | Finish terminal-qualified consumers and explicit-policy receipts | in progress; typed replay, qualified affine continuation and five PCB query outcomes kept. Full Hypercurve and affected downstream checks pass. Native contour/extrusion qualification and cold archive/rebuild remain open. See admission below. |
| R09 | Overlay coplanar class boundaries before repeated triangulation | pending |
| R10 | Select stack-safe teardown using a construction-time depth hint | pending |
| R11 | Transport loop roles through exact line-boundary coalescing | admission reopened and inconclusive: repeated mixed-loop proofs disappear, but unchanged-control latency nonregression is unproved. The handoff does not permit treating noisy overlap as nonregression. The production change has been removed; evidence and the separate line-fit proposal remain open. |
| R12 | Share a private lazy tangent cross and direction result | rejected: isolated builds preserve 19,200 exact results and reduce analytic allocations, but two whole-workflow controls have adverse latency intervals. The 192-byte WASM saving cannot override that gate. The production change has been removed. |
| R13 | Reuse the forced successor bijection before tangent construction | rejected for the measured corpus: a dedicated build passes 186 tests and visits 133 reconstruction sites. None has one unmatched pair before or after tangent construction. Modeling and PCB consumers add no such sites. No production change retained; the separate H17 API alternative remains open. |
| R14 | Prepare rational integer polynomials once per refinement proof | evaluated; no production change retained. The proposed point kernel already exists in Hyperreal. Two private variants pass exact oracles and Clippy; preparation-only and baseline both pass the unchanged full refilleting gate. Preparation accounts for only 0.417% / 0.0102% of the captured proofs in phase diagnostics, with no established native latency benefit. Further work requires a changed premise at the midpoint-evaluation owner. |
| R15 | Finish local root comparisons and share singleton selection certificates | equivalent-field replay kept for exact-comparison completion after all eight final primary checks pass: 2,618 Hypercurve tests, 24 downstream tests, Clippy, docs and WASM compilation. The current primary counterfactual reproduces the undecided pair defined by f and f - 2^-600. Thirty native pairs preserve 26,880 Certified samples / 1,680 artifacts and reduce targeted overlap cost, but general performance admission fails, including one simultaneously adverse control. Costs are disclosed and measured on the private pre-R23 stack. Shared-certificate factoring, cancellation and broader R15/P06 remain open. |
| R16 | Capture the actual remaining algebraic stall before choosing a solver | captured and evaluated; both private quotient consumers rejected. Independent exact proofs establish different rational and quadratic coefficient fields and maximal common factors. The first stall follows a 26 ms GCD; the second spends about 192 diagnostic seconds in general coefficient arithmetic. The rational-only shortcut completes the first original workflow, but the second still times out and eight simultaneous protected timing intervals regress. Only eight predicate tests and corrected documentation are retained; all eight targeted checks pass (78 tests). Multi-source leaves, complete Pell outcomes and internal resource bounds remain open. See the [cost and capture record](../crates/hypercurve/PERFORMANCE.md#selected-fiber-quotient-rejection-r16-2026-10-25). |
| R17 | Reduce retained scalar accelerators at their actual owner | ownership reproduced; private lazy-slot layout rejected. Complete trace replay and a separate census locate held storage, with no decline-only evaluation cells. The candidate lowers Nano report payload by 5.3% and release peak by 4.6%, but a protected warm query regresses and all 64 latency upper bounds exceed 1. Only three retention/source-drop/concurrency tests are retained; all eight final primary checks pass (1,776 tests). Internal resource bounds remain open. See the [owner's evidence](../crates/hyperreal/PERFORMANCE.md#retained-scalar-ownership-and-rejected-layout-r17-2026-10-25). |
| R18 | Prove hard compound rotation relations without a global limit increase | kept for scoped exact completion after all 20 final primary/downstream checks pass on 2,352 unchanged source inputs. Late common-factor recovery completes the original binary64 and decimal workflows plus 18 axis/family/policy cells without raising expanded degree or work limits. Thirty native pairs preserve 227,028,480 predicate results; eight workflow pairs preserve 32 exact artifact pairs. Every protected simultaneous latency upper bound exceeds 1; two scalar controls are marginally adverse, and the deliberate degree-14 decline requests 2,096 more bytes per query. General performance admission fails; no speedup is claimed. See the [owner record](../crates/hyperreal/PERFORMANCE.md#factored-compound-rotation-completion-r18-2026-10-25) and evidence record `2026-10-25-research-r18`. |
| R19 | Audit cold proof continuation and real resource bounds at import boundaries | kept for scoped exact completion after all 20 final primary/downstream checks pass on 2,354 unchanged source inputs: 303 new exact import/consumer cells and cold exact BRep archive replay. Thirty scalar pairs and eight workflow pairs preserve all declared results/artifacts, but fail general performance admission: 101 of 118 simultaneous upper bounds exceed one, with three adverse native-construction metrics. Costs, intentional expression changes and still-missing cumulative resource contracts are recorded in the [owner record](../crates/hyperreal/PERFORMANCE.md#cold-domain-continuation-and-resource-audit-r19-2026-10-25) and evidence record `2026-10-25-research-r19`. |
| R20 | Separate partial swept boundaries from regularized revolution solids | pending |
| R21 | Canonicalize straight sweep images with isolated stationary parameters | pending |
| R22 | Extend one affine-image BRep family through all required consumers | pending |
| R23 | Complete box containment with selected extrema on retained spans | exact rational/selected-span milestone kept after all seven final primary checks pass. The identical private source passes 2,610 Hypercurve tests, 24 CSGRS tests and all twenty Alumina envelope cases versus six completed / fourteen unresolved on baseline. Thirty cost pairs preserve 1,080 old artifacts and add 12,000 successful queries. Nineteen adverse timing cells, eight extra bytes on polynomial extrema and native size growth are disclosed; no speedup claim. Public witness locations and broader derivative domains remain open. |
| R24 | Retain per-curve fidelity and source correspondence after extraction | pending |
| R25 | Close campaign evidence and independent completeness acceptance gaps | in progress; historical H13–H15 inputs recovered and checked. Successive-operation build attribution and per-row journaling are implemented and exercised with copied binaries and forced termination. Remaining completeness oracles and named gates stay open. |
| R26 | Attribute final HEAD symbols before choosing binary refactors | pending |
| R27 | Place each exact section vertex once while streaming wall rows | pending |
| R28 | Use certified nonconstant circular images to reject line fitting | shortcut rejected: thirty native pairs preserve 5,760 outcome-row pairs and improve circle queries, but five simultaneous protected intervals regress. Only seven independent fit/degeneracy/receipt tests and a correction to the misleading `NotLine` description are retained. All eight final checks pass, including 63 Hypercurve and 24 downstream tests. |
| R29 | Factor only measured duplicated SDF setup and finalization | pending |
| R30 | Make degenerate hull tie selection independent of cache warmth | pending |

The bounded alternatives are tracked separately so an old rejected variant
cannot silently close a different research hypothesis:

| Alternative | Current disposition |
| --- | --- |
| H01 restricted support/chart view | pending; compare with R05/R07/R24 |
| H02 pair capability API | pending; demanding selected/mapped pair fixture |
| H03 compositional relation/contact algebra | pending; residual contact and overlap replay |
| H09 local source-union/field join | pending; independent selected sources |
| H16 adaptive elevation/preparation | pending; sparse versus sequential degrees |
| H17 demand-driven arrangement/selection | pending; compare with R11–R13/R28 |
| AffineMap2/RegionPointLocation | pending; invertible affine composition |
| Certainty/provenance algebra | pending; independent fidelity, certainty and policy axes |
| Wrapper fallibility/policy vocabulary | pending; usable advanced operation chains |

### Hypercurve work-package coverage and admission reconciliation

The third input, [hypercurve_rd_program.md](research/hypercurve_rd_program.md),
adds P00–P29. These packages remain in scope alongside R01–R30; overlap with an
R project does not establish package completion. Its section 6.3 requires the
upper confidence bound of every protected latency ratio to be at most one for
a lower-priority change. Neither a lack of statistical significance nor instruction/allocation
savings is sufficient. The earlier R05/R11 admission narratives below are
historical experiment reports, superseded by this reconciliation: R05 has an
adverse section interval, and R11 has not established control nonregression.
R12 is rejected for its adverse whole-workflow intervals. All three production changes have been removed after the frozen combined
validation passed all 39 commands with unchanged compiled source. The patches,
test candidates and raw results remain available in the archive.
No existing supported success or assertion is weakened to retain them.
R04's sampled-profile repair has a higher-priority correctness/completion
claim and discloses its measured runtime cost separately.

| Package | Current evaluation and related work |
| --- | --- |
| P00 baseline and domains | In progress: source/build attribution and isolated targets established; full capability/cost matrix remains open. |
| P01 independent checkers | In progress: exact geometry, rational, incidence and replay oracles accompany individual experiments; broader completeness checking remains open. |
| P02 exact span adapter | Pending; H01 restricted-support pilot remains open. |
| P03 trims and traversal | Pending; compare H01 and R07/R24. |
| P04 selected-root authority | Pending; compare R03/R15. |
| P05 cusp-contact charts | Pending; source-bound contact evidence remains open under R02. |
| P06 local comparisons | Pending; compare R15/R16 and the recovered hard inputs. |
| P07 consumer promotion | Pending; R05/R07 intersection and arrangement families remain open. |
| P08 tangency and qualification | In progress: R02 prototypes rejected, R08 qualified affine continuation kept; native consumers remain open. |
| P09 support-pair relations | Pending; H02/H03 and R05 broader query families remain open. |
| P10 local offset/fillet fields | Pending; R03 gate reproduced without a replacement solver. |
| P11 lazy signs and modular filters | In progress: historical inputs recovered; current-stage benefit must still be established. |
| P12 modular GCD and root counts | In progress: degree-164 selected factor checked; independent root-count and solver work remain open. |
| P13 filter order and compaction | R12 rejected; R14 two-variant evaluation complete with no production adoption. The scalar midpoint kernel already exists, and preparation-only latency benefit remains unproved. Other filter/compaction hypotheses stay open. |
| P14 exact containment | Rational and selected-span milestone kept after independent oracles, full owner/downstream checks, a real Alumina consumer, paired costs and final primary validation. Published witness locations and derivative domains beyond the unit source chart remain open. |
| P15 distance and curvature | Pending; operation domains and independent checks must be frozen first. |
| P16 offset/stroke completion | In progress: existing hard composition gates reproduced; no broad new domain admitted. |
| P17 editable generated corners | In progress: R02/R03 blockers reproduced; solution completeness remains open. |
| P18 arrangement invariants | In progress: R13 shortcut rejected for measured sites; other degeneracies and H17 remain open. |
| P19 polynomial components | Pending; R06/R07 multiplicity and vertical-component prerequisites remain open. |
| P20 resources and cancellation | In progress: external containment and lifetime measurements recorded; R10/R17/R19 internal contracts remain open. |
| P21 preparation and pruning | Pending; broader retained-query/cache experiments remain open. |
| P22 certified finite projection | In progress: R04 repairs sampled modeling; it supplies no analytic fidelity certificate. R24 correspondence remains open. |
| P23 skeletons and Minkowski scope | Pending; no implementation or admission claimed. |
| P24 binary footprint | Pending; R26 must attribute the final retained stack. |
| P25 semantic organization | Pending; no cosmetic refactor adopted. |
| P26 formal proof boundaries | Pending; identify a concrete boundary and checker before adding tooling. |
| P27 integration and bindings | In progress: affected Rust/C/C++/Python/JS/TS checks run per milestone; packaged-consumer coverage remains open. |
| P28 alternative algorithms | In progress: individual negative experiments retained; no blanket disposition. |
| P29 cumulative closure | R19 combined owner/downstream suites pass. Broader interactions, holdout and release admission remain open; the research program is paused. |

### R01 mesh milestone and R25 evidence recovery

Owner: CSGRS modeling composition; P17/P18/P25/P27, with P9/P19 receipts.
The baseline public API loses the exact interior witness `(-3/2,1/2,0)` from
the revolution of `[-2,-1] × [0,1]` when the disjoint profile
`[1,2] × [2,3]` is added at three segments. The other 19 original profile
tests pass. The retained implementation splits mixed signed profiles at the
axis and uses the existing exact mesh union for odd full-turn lattices;
even lattices retain the equivalent planar fold. Integer full-turn multiples
canonicalize coverage, and `revolve_from` preserves an explicit start.
Oversized noninteger sweeps remain invalid. The intentional odd-lattice
geometry/row migration is in [MIGRATIONS.md](MIGRATIONS.md).

Acceptance actually executed after the final code edit: 18 native curve unit
tests; 56 affected integration tests, including 24 exact-profile tests;
the existing revolved-ring continuation gate with its required IO features;
13 FFI Rust tests; real C/C++/Python consumers; the complete generated JS/TS
WASM script; warning-free root docs, formatting, and Clippy across all root
targets/features with warnings denied. All pass. Independent polygon
half-space membership and oriented ray winding cover axis crossings, holes,
overlaps and disconnected pieces at N=3/4/5/6. A deliberate unregularized
overlap is detected as winding two. Tests retain source-drop, eight cut/restore
generations, empty feedback, reversed authoring, signed turns, explicit start
37, and the 360-degree boundary. A stale FFI test also fails on the unchanged
baseline: 13/-13 rotation now certifies. Its original exact set expectations
are retained as success controls, with a separate unresolved three-rotation
fixture exercising all four failure paths.

The 18-case, 540-operation allocation-instrumented paired probe preserves
exact rows and allocation totals on all 12 one-sided/even-lattice controls.
The changed odd mixed-sign contract has substantial extra cost; for example
the N=5 overlap requests 258 MB, peaks at 1.07 MB live growth, and takes about
268 ms in this probe. This is accepted for exactness, not as an optimization;
all adverse rows and linked probe sizes are in [PERFORMANCE.md](../PERFORMANCE.md#signed-revolution-research-r01-2026-10-25).
There is no new total-work, byte-budget or cancellation guarantee; probes use
external time containment. BRep canonical support/parameter retention,
broader continuation, and the remaining R01 acceptance work stay open.

Evidence record: `docs/evidence/2026-10-25-research` in the evidence repository,
commit `2f08663201ff859be36eeeabf5745d5916d2a555`, SHA-256
`7e9cd5280eefa5160ee60801ff2cb4efd7d1a2ae90d300db0f7a74c42d800643`.
It is retained on a local archive branch; publication status and replay
instructions are in [evidence-archive.md](evidence-archive.md#local-follow-on-records).
The same record preserves the R02 failing gate and its passing control.

R25 recovered the actual H13–H15 coefficients, factors, probes and historical
logs unchanged. New independent checks reproduce the degree-214 factor product,
square-freeness, the unique selected root by Bernstein sign variation, and
degree-164 irreducibility using good primes 19/29/31. The input digest matches
the owning historical performance record. A full-polynomial Sturm verification
timed out at 300 seconds; that adverse route is retained beside the successful
proof. This closes the captured-factor archival gap, not R25's remaining
harness/completeness work. No historical timings were fabricated. The older
mandatory modular/support-abstraction wording in Hypercurve FUTURE_WORK now
agrees with the handoff's experimental charter and measured negative results.

### R02 experiments and R25 harness milestone

R02's unchanged arc/line gate fails with `RealSign` at erosion; its line/line
control passes. Retaining every selected-circle tangent does not discharge
the failure (11.27 s). Bypassing represented fillet publication instead makes
the second fillet return `Unsupported` (0.32 s); this is a regression, not a
speedup. Both patches were removed. A bounded diagnostic finds a retained
degree-two parallel against the represented fillet tangent. Their scalar
graphs have 256/184 nodes and depths 76/70. A recursive Debug dump was stopped
because it expanded shared expressions; this was a diagnostic failure, not a
geometry timeout. The existing difficult test remains unchanged and ignored.

R25 ownership is the benchmark and its shared test runner (P27/P35–P39).
Before editing, a copied executable reported an unrelated checkout's revision;
killing it after it printed completed square-drill rows left no raw file.
The new driver embeds source and actual compiler metadata, retains dirty and
new input contents, checks source stability during compilation, hashes the
executed binary, and records incomplete/completed/interrupted/timed-out runs.
The runner reports records after operation/validation/drop timing and syncs
each TSV row before starting the next step. Direct Cargo builds explicitly
remain unattributed. Format and replay instructions are in
[successive-operations.md](successive-operations.md).

The copied-binary test runs from a new Git checkout with replacement `git`
and `rustc` programs on PATH. It preserves the embedded source/compiler record
and actual binary hash. Both validated and untouched 16-step controls complete
certified; validated geometry/receipt/size columns and operation allocation
totals match the baseline. SIGKILL during the next symbolic drill retains the
first certified row and records interruption without a completion marker.
All 20 successive-operation tests pass, including the three explicitly run long
chains and a new observer-order test covering certified success and typed
failure. Root all-target/all-feature Clippy with warnings denied and formatting
pass. Driver termination forwards the signal and reaps its child; timeout is
recorded separately. These checks address
process termination; they do not establish storage-device fault tolerance or
the separate R25 whole-component completeness gates.

R03's unchanged NURBS workflow was actually executed from the release test
binary with a 300 s external timeout and an 8 GiB address-space limit. It did
not complete; the last 30-second memory sample reported 893,304 KiB RSS at
270 s. This is observed process memory, not an allocation or algorithmic peak
bound. The timeout is external containment, not a new typed geometry budget.

R04's initial reproduction added four deterministic fixtures in
`tests/mixed_modeling_profiles.rs`. All were explicitly executed and failed
as expected on the unchanged modeling
bridge. An additional 40-operation probe records every row: 20 straight-only
controls preserve exact vertices; mixed third/sqrt2 cases lose them in all five
operations; mixed `2^-80` widths and `2^70` origins return `InvalidInput` after
finite projection collapses the ring. That evidence record preserves the
original ignored fixtures. The subsequent R04 admission below expands and
enables them without weakening their expectations or policy.

Evidence: `docs/evidence/2026-10-25-research-r02-r25`, local archive commit
`1a77f07e8f166bb2455b5f3f5117e5c51a3219eb`, SHA-256
`3715724b34398e9443db0d38aed749c3f2eba130b210c5db5e924b531d17cb6e`.
The archive includes rejected patches, full executed command records, exact
rows, final checks and limitations. No push or full pre-push matrix was run.

### R04 mixed-profile admission

**Repair retained after renewed admission.** Later R12 validation ran the unchanged
`downstream_profiles` target, which the original R04 selection omitted.
The algebraic parabola extrusion loses 3.7193 units of volume (required deficit
below 0.014), and the circle-wall extrusion fails a lexicographic point
comparison. A fresh, dedicated-target legacy → mixed → legacy comparison
without R12 reproduces both failures by changing only the modeling-profile
function, then restores all six passes. The original results below
remain evidence for their stated fixtures, but do not establish acceptance of
the combined change.

The repair candidate refines retained Bézier parameters in source coordinates
before finite sampling and uses Hyperreal's existing product-cancellation
primitive in orientation/normal construction. A fresh primary-only Cargo target
passes all six original downstream tests, plus the full predicate and mesh
suites. Retaining unresolved bridge priorities alone did not complete a new
triangular-hole fixture with a terminal-zero coordinate; extending bridge
search alone also failed. Both failed experiments remain in the raw evidence.
The subsequent candidate also retains vertices with unresolved collinearity,
distinguishes unresolved local convexity from a decided nonconvex turn, and
tries other ears before returning a retained undecided error. Every selected
bridge and ear still requires its geometric proofs. It passes the unchanged
fixture and all 213 Hypertri tests both in isolation and in the primary worktree,
with Clippy warnings denied. The primary predicate and mesh suites also pass.
After removing the R05/R11/R12 production candidates, all 39 final commands
pass on unchanged compiled source. Full suites pass: 376 predicate, 213
triangulation, 295 mesh, 2,600 curve, 676 modeling, 669 BRep and 354 PCB tests,
plus 20 successive-operation and 13 FFI tests. Existing ignored tests remain
7 mesh, 35 curve and 7 modeling. Real C/C++/Python consumers, generated JS/TS
WASM checks, affected all-feature/all-target Clippy, warning-free docs,
formatting, repository budget and diff checks pass. This is the affected
local/downstream matrix, not the full pre-push workflow matrix.

An isolated, workspace-locked comparison of the added triangulation proof
search records 38,400 certified results across 32 cases and 30 alternating
pairs. Exact triangle rows and every non-timing field are unchanged, including
allocation counts, bytes, peak/result live growth and post-drop retention.
It covers concavity, two holes, a comb, `2^-80`, symbolic translation and
40/20-vertex circular rings, both authoring directions and policies. All output
triangles have certified positive orientation, boundary incidence matches,
and exact areas agree with the independently computed ring areas. This is a
completion change, not a speedup claim: cold square/approximate and tiny/strict
medians rise about 3 microseconds, with paired intervals above 1; symbolic
translation/approximate rises about 9 microseconds. The native probe grows
448 text bytes and 648 file bytes. No new total-work, memory or cancellation
bound is claimed. The raw record retains every adverse row.

Build isolation is part of this renewed gate. A shared Cargo target allowed
exploratory dependency objects to enter a primary validation run. That run is
not accepted as source validation. Primary and experimental builds now use
separate target directories; the raw follow-up record preserves the failed
checks, build-isolation finding and clean reruns. R12's isolated,
path-normalized comparison is complete and rejects that separate optimization.

Owners: the CSGRS modeling bridge and Hypercurve's classified profile sampler;
P1/P16/P17/P18/P19/P25/P26. The previous whole-region `Option` collection made
one curved boundary quantize every component. The bridge now chooses exact
or sampled contours per already classified profile. The new explicit
`CurveRegionProfile2::sample_to_linear_contours` keeps certified line images,
native curve endpoints and material/hole ownership, samples curved interiors
with the existing projector, and reuses one coordinate authority per certified
join. Replayed approximate connectivity retains its receipt. Finite export
continues to quantize every coordinate.

All eight public fixtures fail on the baseline and pass after the change.
The 220-operation matrix covers thirds, sqrt(2), `2^-80`, `2^70` translation,
reversed authoring and component order, strict and approximate policies,
curved material with exact holes, curved holes with exact material, and mixed
spans sharing exact endpoints. Independent exact rectangle area and volume
formulae supplement vertex and topology checks. A curved enclosure with a
`2^-80` wall retains its independently known removed volume through four
cut/restore generations after dropping the source region, then composes
through empty. Kernel tests cover cubic endpoints and strict/approximate/strict
closure replay. No test, fixture, policy or validation requirement was weakened.

The paired allocation probe retains 1,350 results across 45 cases: all
candidate cases complete with the required exact vertex, compared with 15
correct baseline cases, 15 rounded successes and 15 invalid-input failures.
Fifteen controls retain identical exact geometry rows, certainty and fidelity
flags; the ten line-only controls also preserve cold and warm allocation
counts and bytes. Algebraic coordinates can cost more: mixed sqrt(2)
revolution rises from 1.61 to 3.81 ms and about 2.07 to 4.80 MB of allocation
traffic. This is retained for exactness and completion, with no speed claim;
[PERFORMANCE.md](../PERFORMANCE.md#mixed-modeling-profiles-r04-2026-10-25)
records the adverse rows and measurement limits.

The original, narrower admission ran 30 selected checks: 158 Rust tests
(7 projection, 8 enabled curve-continuation, 18 native modeling, 92 modeling
integration, 20 successive-operation and 13 FFI tests), real C/C++/Python
consumers, the full generated JavaScript/TypeScript WASM script, root and
Hypercurve all-target/all-feature Clippy with warnings denied, warning-free
docs for both crates, formatting, repository budget and diff checks. That
historical selection is superseded by the 39-command final repair matrix above.
The earlier R02/R03 failures are not closed by this change. This is local source validation,
without a push, complete pre-push matrix or release-publication claim.

Sampling remains a fidelity boundary, including endpoints that cannot inhabit
native coordinates. The `1e-3` chord-error request is not a newly certified
topology/error bound; the whole mesh retains sampling and coordinate-projection
qualification. No new total-work, retained-memory or cancellation bound is
introduced. Native mesh row changes are explicit in
[MIGRATIONS.md](MIGRATIONS.md). Per-curve correspondence and broader cold
archive guarantees remain separate R24/R19 work.

Evidence: `docs/evidence/2026-10-25-research-r04`, local archive commit
`a1c1182802aae8bd43ca825da95e67f262cd7a71`, SHA-256
`349cfa2256f7d156d1efd0629b1a3d4ef959b05734e8198a7e82c3aba5a67cb9`.
Raw baselines, prototype failures, final commands, exact output rows and
adverse resource measurements are retained; publication status is in
[evidence-archive.md](evidence-archive.md#local-follow-on-records).


Renewed admission evidence: `docs/evidence/2026-10-25-research-r04-followup`,
local archive commit `c0fd193305812e52a76a076b8bfce89ceb208e62`, SHA-256
`ef789e5a1b616bf86510bd17dda8f2c5fa522a44fe665a6015371550f074ee85`. This follow-up records the source
isolation failures and corrections, unchanged hard fixtures, paired cost rows,
and the final 39-command retained-stack validation with matching before/after
compiled-source inventories. No main-repository commit or push was performed.


### R11 loop-role transport experiment — admission withdrawn

The latency reconciliation above supersedes this original admission. The
production change has been removed; the following records the tested candidate.

Owner: Hypercurve Boolean output normalization, P4/P9/P15/P17/P19/P27/P36.
On the current target, a mixed cap/rectangle union with a remote circle proves
both output loops material, coalesces its lines, then repeats the same role
proof on every query. The new private rewrite retains that authority only
for a strict source and strict directed-image equivalence with unchanged loop
indices. Existing merge predicates and boundary provenance are unchanged;
qualified source decisions are replayed and cannot become strict proofs.
Geometry-dependent caches are recreated, while the immutable role Arc shares
the source proof for the output's lifetime. There is no new global cache.

The baseline regression fails at the repeated proof. Six public fixtures and
three unit guards now pass, with independent area/location expectations,
holes, point touches, backtracking, reversal/seams, selected endpoints,
source drop, later Booleans and rejected/qualified rewrites. Thirty alternating
pairs across eight cases preserve every exact geometry/role/area row and
construction allocation/peak count. Ten queries on two mixed components drop
from 65,780 allocated bytes to 20 (47.49 to 0.57 µs warm median); 32 components
drop from 886,880 to 320 bytes. The first prototype's extra construction
allocation was removed. Result retention grows by at most 48 bytes in these
cases; post-drop live bytes match. Construction timings include slower
controls, and native probe `.text` grows by 12,592 bytes. Full measurements
and limits are in [Hypercurve PERFORMANCE](../crates/hypercurve/PERFORMANCE.md#retained-loop-roles-through-line-coalescing-r11).

After the final implementation edit, 36 local/binding checks, a seeded Boolean
rerun and two downstream commands pass: 298 Rust tests plus the seeded
11-test rerun; real C/C++/Python and JS/TS WASM consumers; BRep extrusion,
axial-shell and retained-profile continuations; PCB materialization; root and
Hypercurve all-target/all-feature Clippy with warnings denied; docs and format.
The initial BRep build encountered a read-only ccache directory and passed
with the cache placed in `/tmp`; both logs are retained. Fourteen existing
ignored frontier tests remain open. No push, full pre-push matrix or published
package verification was performed. Line-fit sharing and the remaining cold
role-proof cost are separate, unfinished parts of R11.

Evidence record: `docs/evidence/2026-10-25-research-r11`, local archive commit
`14d94946da060d8cb215cb161f37efbe6c9bb314`, SHA-256
`e558e4a5395098e82ae0a2796cc58013ace58676d33466b1eb902ad358fc8b2c`.
Historical records and published `main` are unchanged; the branch has not
been pushed.


### R05 shared cut-point experiment — admission withdrawn

The latency reconciliation above supersedes this original admission. The
production change has been removed; the following records the tested candidate.

Owner: Hypercurve polynomial branch subdivision; P4/P10/P15/P17/P19/P20/P27
and P35–P39. On the current target the genuine quintic section isolates one
fiber, while section plus splitting isolates three. The new regression fails
because adjacent branch pieces independently reconstruct the selected point.
The retained producer evaluates each cut once, preserving its fiber relation,
source/slab/root, original chart, traversal and image maps. Public one-cut
splits and internal sorted multi-cut splits share the producer. There is no
new global cache or intersection family.

Acceptance includes independent polynomial residuals, selected and mapped
cuts, source drop and strict replay, reversal, duplicate/endpoint cuts, invalid
parameters and exhaustion/recovery. An exploratory replay assertion exposed
an existing approximate-policy similarity blocker. A separate baseline and
candidate probe gives identical strict/approximate/strict outcomes; the final
guard preserves that blocker rather than upgrading qualified evidence.

The 30-pair, ten-case, twenty-sample measurement completes 12,000 certified
operations with identical exact rows, selector/map definitions, sizes and
checksums. Warm split allocation traffic falls from 76,352 to 39,528 bytes;
vertical topology falls from 140,624 to 103,560 bytes. Steady cache retention
matches after the initially adverse warming behavior is accounted for.
Unchanged controls, including the slower section and cold line samples, are
retained in the [cost record](../crates/hypercurve/PERFORMANCE.md#shared-polynomial-cut-points-r05).
This admits the bounded cut-sharing optimization, not selected/mapped line
intersections, branch pairs or common components. The original topology
contact still has a separately constructed point. Resource limits and
unsupported families remain unchanged.

Final affected validation passes all 12 commands and 214 Rust tests, including
31 subdivision, 16 polynomial, 132 other curve/point, 14 BRep and 21 modeling
tests. Root/Hypercurve all-target/all-feature Clippy denies warnings; docs,
wasm32 compilation, formatting, repository budget and diff checks pass. No
polynomial binding API changed; this is not full pre-push or release validation.

Evidence record: `docs/evidence/2026-10-25-research-r05` in the local evidence
repository; digest and publication status are in
[evidence-archive.md](evidence-archive.md#local-follow-on-records).

### R08 replay, affine continuation and PCB receipt milestone

Owners: Hypercurve retained-policy validation and affine cache admission,
CSGRS error classification, and Hypercircuit's board query boundary;
P6/P8/P9/P14/P17/P18/P19/P25/P30. On the current target, a square closed by
`z = sin(e+1) - sin(e)cos(1) - cos(e)sin(1)` consumes the approximate terminal.
Strict classification, Boolean and translation then misreport policy replay
as invalid topology; approximate translation fails while building an optional
strict native contour cache. The independent addition formula establishes
the fixture's exact zero. A truly open `1/100` gap remains a separate invalid
control under both policies.

The retained validators now share an admissibility check and return
`CurveError::IncompatiblePredicatePolicy`, mapped to a principal
`Blocked(Predicate)` and CSGRS `Undecided`. Accepted policy pairs are unchanged;
the consumer still observes the decisions it uses. Four existing replay tests
now require that specific cause instead of a generic topology string. A
qualified affine transform keeps its authoritative boundary without demanding
or certifying the strict native accelerator. Translation, reflection and
anisotropic scaling pass two four-step chains with Boolean reuse, source
replacement and strict/approximate/strict checks. Strict sources retain their
certified native views under either policy.

Hypercircuit adds `*_outcome` methods for point, inset, disc, segment and box
queries, returning the complete query's `Provisional<Result<Classification<_>,
_>>`. The original signatures remain compatibility paths. All five consume
the terminal on the symbolic-zero fixture; ordinary exact queries stay
certified under the approximate policy. Nested scopes, source drop, mapped
receipts, malformed segments, curved boundaries, holes and `1/10^12`
near-boundary controls pass. The external baseline observer and the new
methods produce identical value/receipt logs for 30 queries. This does not
certify inherited construction fidelity.

Final validation: 2,607 passing Hypercurve tests across all features, with
seed `20261025` and 35 existing ignored tests; 31 focused commands with 350
passing Rust test executions (overlapping the full suite) and 25 existing
ignored tests. Modeling, BRep, PCB and work-budget consumers pass. Clippy
denies warnings for every target and feature of CSGRS, Hypercurve,
Hypercircuit and Hyperbrep. Docs, wasm32 checks, C/C++/Python consumers,
JavaScript/WASM and TypeScript consumers, formatting and repository-budget
checks pass. Earlier compiler/test-harness errors and the initial four stale
error assertions are retained in the evidence; the seeded rerun is clean.

The linked native loop probe grows by 20,312 text bytes and 12,424 file bytes;
the board-query probe grows by 5,116 and 8,624 bytes. These are probe-specific
costs, accepted for restored completion and observable certainty, not a
performance or release-size improvement. No new resource bound is claimed.

Native line-profile extrusion still reports `AmbiguousCurveStringConnection`
under either policy, including its current principal `Invalid` wrapper. The
isolated constructor-bypass experiment made extrusion complete but did not
retain a closure-policy dependency in native contour/string views; it is not
adopted. No wrong operation-local certified classification was established.
Qualified native consumption, its failure diagnosis, cold archive/rebuild and
the remaining representation-specific replay guards stay open. No push or
complete pre-push/release matrix was run.

Evidence record: `docs/evidence/2026-10-25-research-r08` in the local evidence
repository; digest and publication status are in
[evidence-archive.md](evidence-archive.md#local-follow-on-records).

### R13 successor complement evaluation

The proposed shortcut is rejected for the measured current workload set.
Hypercurve already uses the forced-complement theorem in unary regularization;
this experiment checks whether the binary Boolean caller has a useful missing
case under P13/P17/P18/P19. Equal incident counts are insufficient: selected
oriented boundary authority, same-vertex incidence, distinct active IDs and
valid distinct target claims remain prerequisites. No production helper or
instrumentation is retained.

A dedicated target and frozen source patch pass 186 tests, with three existing
stroke ignores. Boolean/stroke/seeded-fuzz tests visit 131 tangent reconstruction
sites and selected-curve fixtures visit two more. Every site starts with two
unmatched pairs; afterwards 132 are complete and one still has two unmatched
pairs. None has exactly one pair before or after tangent construction. The
26 modeling continuation tests and 33 PCB tests add no unresolved binary sites;
the latter include Easyduino full-fidelity Nano aggregation under both policies,
DRC handoff, curved outlines, routing and manufacturing workflows. All recorded
identity/incidence/claim checks pass. This is evidence of no benefit in these
completed workloads, not an impossibility proof for future selected or
high-valence cases. The H17 exploratory API register stays open.

The earlier shared-target trace is retained as exploratory evidence. The new
run records commands, seed, time containment, raw and parsed observations,
assertion results and an unchanged source digest. No timing, allocation or size
improvement is claimed. Evidence: `docs/evidence/2026-10-25-research-r13`, local
archive commit `e72d70d97cd87a590d17a0d6604b7d016be5be41`, SHA-256
`2ed9e47b8d6d725d80826b67196711a3aee70a36013de1101e879d323658dc7f`.
The archive branch has not been pushed.

### R12 tangent-sharing evaluation — rejected

The private candidate removes a repeated tangent-cross request and reuses the
cross/dot result for retained-parallel/represented-vector opposition. An
8,064-case independent derivative oracle, 1,152 ordered family outcomes and
24 public stroke/rebuild chains preserve signs, certainty, typed blockers and
source-drop continuation. Historical shared-target cost measurements were
superseded by separate targets with normalized source paths and workspace-locked
external dependencies. The frozen probe patches cover Hypercurve; other local
dependencies remain at HEAD, as recorded in the archive.

Thirty alternating process pairs complete 19,200 certified results. All exact
audit bytes and receipt/memory/continuation invariants match. Every analytic
sample saves allocation bytes/calls, while native line/quadratic allocations
are unchanged. Nonzero analytic warm stroke ratios are 0.9894–0.9928; zero-offset
intervals overlap one. The protected whole-workflow controls are adverse:
forward approximate line 1.0134 [1.0007, 1.0283], forward strict zero parallel
1.0057 [1.0003, 1.0120]. These exclude fixture construction and drop; no complete
user-workflow speedup is inferred. Instruction reductions and 192 bytes saved
in the matched WASM probe do not override the literal latency requirement.

The candidate passes all 39 frozen combined validation commands, including
2,612 Hypercurve, 676 modeling, 669 BRep and 354 PCB tests, with existing ignores
reported in the raw logs; affected predicates/triangulation/mesh, bindings,
Clippy and docs also pass. This establishes the measured correctness gates,
not performance admission. Its production change has been removed. The
independent tangent behavior oracles remain unchanged and are included in the
post-removal retained-stack validation. The R02/R03 hard gates remain open.

Evidence: `docs/evidence/2026-10-25-research-r12`, local archive commit
`8e48e5ce1c98476c61bfa406304dc3ce7158b324`, SHA-256
`db661d469da8d93c2a870847904ef4f1e397ef511045775afa5f7efb0a64e304`. The branch has not been pushed.

### R14 evaluation: no preparation change retained

The midpoint homogeneous-integer kernel already exists in Hyperreal. The two
private variants pass exact oracles and Clippy, and both baseline and the
preparation-only variant pass the unchanged ignored refilleting workflow.
Five alternating instrumented diagnostic pairs put repeated coefficient
preparation at 0.417% and 0.0102% of the two captured proofs; about 89–90%
is midpoint evaluation. This locates work but establishes no native latency
benefit. The duplicate point kernel is rejected and preparation-only adoption
is inconclusive under the handoff's literal nonregression rule. No production
change is retained. A further variant needs a changed premise at the midpoint
evaluation owner.

Evidence: `docs/evidence/2026-10-25-research-r14`, local archive commit
`5f5b31ac82d32f32ec004a901080de54b8693d9a`, SHA-256
`d2a04ded4c46f0eaea3959153e7d85dbceae00903ad3095951d2dbf56e2ba393`. Source attribution, independent normalized-rational
enclosure checks, original failed diagnostic assumptions and corrections,
6,336 point comparisons for the removed first variant, the unchanged
1,152-polynomial oracle, policy/fallback replays, and the two full refilleting
completions are retained. One candidate run at 161.61 s and baseline at
173.70 s are correctness completions only, not a paired speedup claim.


### R23 exact rational containment milestone

Kept under P8/P9/P14–P20/P25/P35/P36 for certified completion. Hypercurve owns
the rational quotient-derivative candidates, active finite-domain proof and
retained point comparison. Endpoints and all in-range stationary coordinates
establish containment or an exact escaping member. Source poles outside a
valid selected trim do not invalidate it. Roots outside the active range cannot
become witnesses. Conservative bounds are still the initial shortcut.

The independent extrema and continuation matrix covers rational/irrational
cubic and quartic peaks, repeated roots, constant coordinates, signed weights,
selected endpoints, NURBS knots, periodic seams, reversal, similarity transforms,
source drop, exact tangency, `2^-200` clearance and budget exhaustion/retry.
Witness definitions and source parameters replay. This does not supply a new
public source-location result or establish cold archive continuation.

Both private candidates pass 2,610 Hypercurve tests (35 ignored) and
24 CSGRS continuation tests. The final primary checkout passes seven commands:
format; Hypercurve/CSGRS all-feature/all-target Clippy with `-D warnings`;
both warning-free docs; eighteen containment tests; twenty-four downstream,
receipt and failure tests; repository budget; `git diff --check`. All 2,348
compiled-source files from the final private run match primary byte-for-byte,
and each validation run has identical before/after inventories.

Pinned Alumina interface `2c49c81aa45f498d3dea41beae9586963d67a1fb` and firmware
`bdee397d0b80841f16bf17e7bcd029b4feb2c212` private copies test the same twenty
travel-envelope cases: baseline completes six and reports fourteen unresolved;
candidate admits twelve and rejects eight with correct exact escapes. Three
existing travel and rounded-command controls pass in both. The original
consumer worktrees are untouched. Motion scheduling and release packaging are
not included in this result.

Each 30-pair native cost comparison has 67,200 outcomes. The retained candidate
adds 12,000 successful queries and preserves 1,080 old full artifact pairs plus
21,600 per-sample receipt/checksum/size comparisons. Nineteen old timing cells
have adverse unadjusted intervals; the [owning cost record](../crates/hypercurve/PERFORMANCE.md#rational-box-containment-completion-r23-2026-10-25)
reports representative medians and all measurement boundaries. Polynomial
stationary candidates add eight requested bytes, with no additional calls;
other old allocation sequences match. Native probe `.text` grows 8,160 bytes
and file size 9,584 bytes. This is not a speedup or latency-nonregression claim.
The initial duplicate endpoint allocation was removed and its measurements
remain preserved. Twenty-sample retained-state plateaus are not a heap bound.

Evidence record `docs/evidence/2026-10-25-research-r23` is on the local archive
branch at `ec280dbccf7ecb7e77c2f021efa1ea16963e7c18`; its manifest digest is
`9e409ecec0e53d725d217d5f0f17c029a898a4dfcac16922d19e9ecc493209d9`.
It includes failed prototypes, oracle/attribution corrections and the disk-quota
retry. No main-repository commit or push was performed. Broader derivative
domains, public witness locations, cold interchange and internal resource/
cancellation bounds remain open; this milestone does not close all of R23/P14.

### R15 equivalent-field comparison milestone

The equivalent-field replay is kept for exact-comparison completion, under
P1–P6, P8–P11, P17–P20 and P35/P36. Its owner is Hypercurve's retained recursive
root comparator. After the ordinary formula evaluator declines `Unsupported`,
existing Hypersolve embeddings replay a polynomial's coefficients into an
equivalent selected-source base and radical tower. Homogeneous evaluation
preserves sign through the formula's certified positive denominator. This adds
no source union, new radical, cache or global limit and keeps the old fallback.

The current primary counterfactual, including R23, reproduces both missing
test groups. In particular, the roots of `f` and `f - 2^-600` remain undecided
under both policies; the candidate orders them exactly without global
projection. Fourteen formula/polynomial cases and eight equivalent-tower cases
cover independent field allocations, gauges, conjugates, reciprocal/negative
charts and source drop. Other gates preserve foreign-source declines and
reordered-source fallback, 36 repeated-root cases, cache saturation and poison,
changed charts, budget exhaustion/retry and qualified terminals that must not
enter the exact equality cache. All twelve reproduced long overlap comparisons
use the local exact sign in the diagnostic candidate trace.

Final combined primary validation passes eight commands: formatting;
Hypercurve/CSGRS all-feature/all-target Clippy with warnings denied; both crates'
documentation with warnings denied; all 2,618 Hypercurve tests in 72 groups
(35 existing ignores); 24 CSGRS downstream composition/receipt/error tests;
default-feature Hypercurve WASM release compilation; repository budget; and
diff checks. Compiled-source inventories agree before and after the run.
The preceding private owner run passes 2,608 tests and the same 24 downstream
tests. This is affected-path validation, not the full pre-push workflow matrix
or a published-package/binding release claim. No push was made.

Thirty native process pairs preserve 26,880 Certified samples, 1,680 full
artifact pairs and every per-sample checksum/size. The targeted approximate
overlaps improve, but the general performance-only gate fails: all 96 other
control upper bounds exceed one; sixteen marginal intervals and one
simultaneous interval are adverse. The latter is a complete STRICT workflow
at ratio 1.0272 [1.0136, 1.0411]. Five independent allocation pairs preserve
4,480 Certified samples / 280 artifacts and reduce overlap query traffic from
about 476 MB to 14.5 MB. Other sampled allocation sequences match. Native
`.text` grows 2,096 bytes and file size 12,712 bytes. The
[owning cost record](../crates/hypercurve/PERFORMANCE.md#equivalent-field-root-comparisons-r15-2026-10-25)
gives timing boundaries, all adverse results, statistical treatment and limits.
That frozen cost base predates R23; it establishes no combined-stack native
performance claim. Exact completion, rather than general performance, is the
retention basis.

Evidence record: `docs/evidence/2026-10-25-research-r15` in the local evidence
repository, commit `d65d6245382cde6ef23b695608c2df7c9f9fef7b`.
The record contains 19,191 files / 60,600,432 bytes, SHA-256
`1384647865ae24a1ae53535fcca46ee43a23f0a3e30237db1055414fc7296600`; it remains unpublished.
Shared singleton-certificate factoring, general field joins, cancellation,
heap bounds, cold archive continuation and broader R15/P06 work remain open.

### R28 guarded circular line-fit experiment

The performance-only shortcut is rejected. It uses retained positive-radius
circle authority, same-sign weights and distinct active endpoints to prove
that a finite connected image is nonconstant. Both native quadratic and general
rational fit paths need the guard: current two-circle output uses the native
path. A general-only change would miss the reproduced cost. Collapsed support
tags, retracing, poles and unresolved endpoints retain the generic fallback.

Seven independent tests pass on both baseline and candidate. Thirty final
native pairs preserve 5,760 outcome-row pairs: 68,814,000 successful exact fit
queries and 13,762,800 typed unsupported queries. Warm circle ratios are
0.2512–0.3250, but all 32 noncircle control upper bounds exceed one and five
simultaneous intervals show regressions. Five allocation pairs reduce only the
eight circle/policy cells. Native probe text grows 23,472 bytes. The
[owning cost record](../crates/hypercurve/PERFORMANCE.md#guarded-circular-line-fit-rejection-r28-2026-10-25)
reports every adverse case, timing boundaries and the predeclared early veto.
Whole-workflow performance and exact-artifact admission were not run after
that veto; diagnostic Nano counts alone establish neither of those claims.

Only the seven independent regression tests and a documentation correction
remain: `NotLine` includes constant points and does not certify nonlinearity.
Runtime source is otherwise identical. All eight final checks pass:
formatting; Hypercurve/CSGRS all-feature/all-target Clippy; both crates' docs
with warnings denied; seven unit oracles; 56 fit/native-image integration tests;
24 downstream modeling/receipt/error tests; repository budget; and diff checks.
Compiled-source inventories agree before and after validation. This is scoped
retained-change validation, not the full pre-push or release matrix.

The evidence record is `docs/evidence/2026-10-25-research-r28` on the local
archive branch, with its digest in `docs/evidence-archive.tsv`. It includes
the unchanged original Nano pipeline with 268 DRC findings and six release
blockers; the board filter skips its all-board golden check. Large diagnostic
logs are losslessly compressed with original-content hashes. No push was made.
Reopening the shortcut requires a changed producer/dispatch premise that
protects ordinary fits, followed by direct-query and complete circle/board
workflow gates. The separate H17 API proposal remains open.

R28 archive commit: `bdded6ef9d6761918276852e96c64c4b2a986e91`;
4,248 files / 22,826,945 bytes, SHA-256
`65c784d87d3d6fea819f48c12de1d6100e1dc11a2489884426a90de31fe1e477`.
All thirteen local record digests verify; historical archive paths and the
published archive branch are unchanged.
