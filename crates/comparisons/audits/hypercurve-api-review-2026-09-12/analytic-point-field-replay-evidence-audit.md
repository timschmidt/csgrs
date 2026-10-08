# Analytic point equality through retained source fields

The implementation goal remains active. This change addresses the additional point replay stress preserved during the evaluable region carrier migration; it does not qualify the full curve-family closure goal or the existing failing and expensive cases.

## Finding and implementation

The extra assertion in `algebraic_chord_analytic_parallel_pair_replays_contacts_and_overlap` exceeded 300 seconds before this change. Localization showed successful point evaluations and comparisons for the earlier crossing, selected-fiber, zero-distance overlap, regular retracing branch, and opaque-coefficient overlap cases. The first stalled comparison was an algebraic point against an analytic endpoint on the opaque retracing source restricted from the selected positive root of `2t² - 1` to one.

Resolving the endpoint preserves the regular branch frame. The analytic equality dispatcher nevertheless reserved its existing recursive projective predicate for recursive parameters, skipping ordinary algebraic Bezier parameters. It then attempted independent Cartesian coordinate reconstruction. Calling the existing recursive predicate directly certified the same endpoint equality; subsequently entering the old dispatcher still timed out. The source parameter and positive frame speed already supply the needed correlated field.

`BezierAnalyticParallelPoint2::same_point_evidence` now uses that existing predicate for ordinary algebraic source parameters as well as recursive parameters. Exact scalar parameters retain their direct-coordinate route. There is no new API, compatibility adapter, point representation, or field arithmetic. Existing policy validation, denominator certification, positive radical selection, field caching, and the general fallback remain authoritative.

All contact and overlap endpoints in the shared chord/parallel test helper now replay the reported evaluable curve and require `CurvePoint2::coincides_with` to return Certified equality. The former duplicate assertion at the selected/exterior caller is removed.

## Independent regression obligations

A new test evaluates the non-PH source `P(t)=(t,t²)` with left offset `1/8` at the positive root of `2t²-1`. The independent comparison expression uses the identity `speed(alpha)=sqrt(3)`. It checks equality and inequality under both policies and operand orders, with positive and negative homogeneous denominators, the other selected root of the same polynomial, the opposite normal sheet, and coordinate displacements of `2^-600`. All 40 comparisons must be Certified. The existing regularized-frame and foreign-field axis-order regression is also retained.

The generic replay fixture includes arbitrary exact Real coefficients whose zero status is unresolved, selected/exterior domains, branch-local PH reduction, contacts, and overlaps. Its restored assertions no longer sit in a separate unqualified stress artifact.

## Attempts retained

- `region-carrier-evidence-equality-stress/reproducer.json`: original 300-second timeout, with its source and executable preserved.
- `analytic-point-replay-localization.json`: 30-second localization, identifying the first stalled comparison. The source before instrumentation is preserved separately.
- `analytic-point-replay-recursive-localization.json` and `.patch`: direct recursive equality succeeds on that comparison; the unchanged dispatcher then exceeds 45 seconds. Instrumentation has been removed.
- `analytic-point-field-replay-attempt1.json`: test compilation caught a missing unwrap of the fixture's parallel constructor result. No test ran.
- `analytic-point-field-replay-attempt2.json`: restored generic replay passes in 0.11 seconds; the new fixture initially requested a negative root interval through the unit-interval constructor, correctly receiving `InvalidBezierParameter`. The fixture now uses the existing general ordered-interval constructor; production validation is unchanged.
- `analytic-point-field-replay-attempt3.json`: all three focused tests pass. The generic replay takes 0.11 seconds, the 40 comparison cases 0.02 seconds, and the retained axis-order check 0.01 seconds. These are individual release-test observations, not general throughput, allocation, or memory measurements.

The subsequent frozen-source qualification strengthens the shared replay assertion to require Certified public equality and removes its duplicate caller assertion. Its final results, source hashes, normal-library hash, dependent checks, public matrices, and commit are recorded in `analytic-point-field-replay-qualification.json`.

## Final qualification and controlled timing

Committed Hypercurve `ef2114738d48439e9ce3f1ac787c974309b9d6e5`. All 49 targets complete with 2,210 passes (1,978 Hypercurve and 232 HyperBREP), five unchanged known failures, nine ignored tests, no new failures, and no timeouts. Eight unchanged expensive cases remain excluded; the full suite is not passing. Both all-target checks, separate fuzz/UI caller checks, formatting and whitespace checks pass. All 396 source hashes remain unchanged through the builds, tests, matrices, commit, and timing comparison. All 30 repositories are clean. Normal-library SHA: `71ce67bd166701aac15ea302f64d5c2bf15e92468ecc755567a4cecced1ecc23`.

All independent public matrices pass. The carrier matrix certifies 2,016 boundary replays, 432 analytic replays, 448 clipping queries, 2,208 source endpoint replays, and 1,008 repeated path trims. The generated-circle, general-trim, and constant-image matrices also retain their exact counts and certification.

A controlled comparison runs the preserved prior executable and the final executable in separate sequential processes using the unchanged public carrier matrix. The prior executable takes 198.369 seconds; three final runs take 1.415470, 1.415405, and 1.415494 seconds (median 1.415470). All outputs have identical counts and Certified status. The earlier independent qualification recorded 202.939 seconds for the prior version and 1.415420 seconds for the new version. Source, executable, normal-library hashes, commands, wall/CPU times and logs are retained in `analytic-point-field-replay-comparison.json`. These measurements apply to this matrix; no general throughput, allocation, or memory claim follows.

The next source audit is `analytic-common-dispatch-followup.json`. Common analytic intersections still lack several pair cases, and nonlinear parameter-component transport remains duplicated in region-specific overlap dispatch. The full architecture goal, known failures/exclusions, normalized region construction and broader algebraic machinery consolidation remain unfinished.
