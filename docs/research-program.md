# Research program, October 2026

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
| M04 | Reuse retained incidence proofs before numerical zero tests | pending | |
| M05 | Fixed-field normalization behind `Real` | pending | |
| M06 | Coplanar scheduling beyond first-use fans | pending | |
| H01 | Chart-qualified restricted-support view | not applicable | 327288e1a1. A public-API probe over every curve family (line, arcs, Bezier, conics, rational cubic, spline split at a knot, NURBS, degree-elevated and retraced curves), cut at rational and algebraic parameters in all traversal combinations, operand orders and both policies, found no failing or undecided restrict/reverse/restrict or replay case. No abstraction was retained without a measured benefit. |
| H02 | Shared support-pair incidence and witnesses | deferred | 327288e1a1. Same probe: no support-pair defect or measured cost names a family to migrate first; deferred until one does. |
| H03 | Incidence-relation restriction and dimension changes | not applicable | 327288e1a1. The [0,1]/[1,2] clipped-overlap singleton is kept and replays to the shared point; an overlap plus an isolated contact coexist (arcs [0,270] and [180,360] degrees); point-touching region Booleans give the expected areas. |
| H04 | Point incidence below topology | accepted | d6373655ef. Reproduced: Easyduino stm32f103 ended in a typed RealSign termination after 169.6 s. Minimized to three of 655 copper profiles: a pad's corner arc meets a route edge and a zone-cut fragment sharing its support allocation at one point, computed with independent radicals. Two contacts on one circle and one shared line are now decided equal or distinct by the exact sign of (P - C)·u, before the general fallback. The board now completes (1,389 findings); the other five boards are unchanged. The broader point-incidence service was not needed. |
| H05 | Polynomial-branch sections through split continuation | pending | |
| H06 | Polynomial queries beyond vertical rational sections | pending | |
| H07 | Finite-fiber topology domains and algebraic coefficients | pending | |
| H08 | Polynomial supports in filled arrangements and BRep trims | pending | |
| H09 | Two-sided coefficient-field joins for polynomial roots | pending | |
| H10 | Local selected-root equality without global promotion | pending | |
| H11 | Selected fillet-center authority through consumers | pending | |
| H12 | Tangent and multiple-root events as local certificates | pending | |
| H13 | Lazy radical leaf signs with modular nonzero filters | pending | |
| H14 | Modular/CRT selected-field GCD with maximality certificates | pending | |
| H15 | Selected-factor degree before eliminant cost | pending | |
| H16 | Direct homogeneous degree elevation | pending | |
| H17 | Shared Boolean topology selection views | pending | |
| H18 | Deferred carrier preparation after broad phase | pending | |
| H19 | Bounded span-pair preparation | pending | |
| H20 | Retained authority versus accelerator memory | pending | |
| C01 | Exact linear profiles through sweep, twist and revolution | accepted | 12262e54e6. Reproduced: sweep, twist and revolve sampled every profile to binary64, losing exact volumes and authored vertices; a 2^-80 sliver swept to a certified closed surface of zero volume; partial revolution at negative radius failed. All three now consume exact line profiles (as extrusion did), orient side walls from exact signed area, and decide radial signs. 15 tests, 13 failing before. Binary64-exact profiles give byte-identical rows. A 32-gon full revolution is 50-65% slower because its vertices are now the exact cos/sin values. |
| C02 | Certified monotone non-affine fixed-plane sweeps | pending | |
| C03 | Placement covariance through BRep chart transport | pending | |
| C04 | General modeling domains with explicit semantics | pending | |
| C05 | Enclosure and blend workflows | pending | |
| C06 | Exact/approximate exchange continuation | pending | |
| C07 | Enclosure evidence, approximation metrics, machine projections | pending | |
| S01 | Cross-predicate bounded normal-form reuse | pending | |
| S02 | Non-expanding certified sign preflight | rejected | 220e84e817 records it. Traces: sign expansion is 39-52% of CPU on the root successive-operation chains, but the existing filter cannot enclose any of those graphs (radical constants 63%, trigonometric 26%, `Sqrt` 12%). Only hypercurve's Boolean tests hit it (565 of 132,820), saving 10 ms against 135 ms of filter attempts. A useful preflight first needs certified filter enclosures for those node kinds. |
| A01 | Unified operation receipts | pending | |
| A02 | Explicit planar domains and classifications | accepted in part | 61da737db3 accepts (c): OBJ, DXF and VRML face import projects along any certified nonzero normal component when the dominant axis is undecided; a VRML face rotated by pi/4 to 200 digits failed under STRICT before. Rows of decided faces are unchanged; release binaries shrink by 4-8 KB. (a) rejected: `curve::try_transformed_with_context` already is the checked affine adapter. (b) rejected: `contains_xy` maps `RegionPointLocation` one to one, and `classify_point` is public. |
| A03 | Fewer accidental API cross-products | pending | |
| B01 | Dependency features at consumer boundaries | pending | |
| B02 | Root, native and WASM size attribution | pending | |
| T01 | Downstream CAD/CAM workloads as acceptance gates | accepted | b8b231f23a and 759a1f674d. The eight SynapsCAD and exactCore workload families are now root tests (`tests/downstream_exact_inputs.rs`, `downstream_profiles.rs`, `downstream_solids.rs`) with exact oracles: literal bounds and volumes, algebraic region areas, empty offset collapses, a twisted holed extrusion with exact sections, the viral planter with every child accounted for, and line and circle relations against Cramer and discriminant oracles. Family 5 exposed a hyperreal gap: sin^2 + cos^2 = 1 was unprovable for rational turns beyond the cyclotomic order limit, because exact cos(pi r) is stored as sin(pi(1/2 - r)). The ring recognizer now pairs complementary sines and cosines, so strict Booleans of rotated cubes decide; worst declined query +9.6% bytes. Near-coincident rotated cylinders still stop at an incidence tested only numerically (M04), recorded in hypermesh FUTURE_WORK. Alumina fixtures are left to C07. |
