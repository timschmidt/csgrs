# Selected-circle equations and unused native poles

This follow-up was executed against committed Hypercurve `3eabc047e4e826385aa6ab7720cdc966dc06f1a1`. The independent fixture and source/artifact-bound results below identify remaining closure work; they are not part of the passing finite-circle-domains qualification.

## Independent public fixture

Reuse the retained radius-1/4 fillet circle and the proven transverse contact with the left distance-1/64 parallel of `P(t)=(-1/8+t^2,t)`, `t in [0,1/8]`, from `finite-circle-analytic-domains-certificate.md`.

Use the increasing rational chart `t=(s-2)/(2s-1)` on `s in [2,5/2]`. Its derivative is `3/(2s-1)^2 > 0`, and its endpoints are 0 and 1/8. It preserves both the geometric image and the oriented unit normal, hence the exact parallel and the independent unique transverse contact. Its homogeneous quadratic coefficients are:

- `X(s)=31/8-(7/2)s+(1/2)s^2`;
- `Y(s)=2-5s+2s^2`;
- `W(s)=1-4s+4s^2=(2s-1)^2`.

The rational Bezier controls are `(31/8,2),(-17/8,1/2),(7/8,-1)`, with weights `1,-1,1`. At the unused native parameter `s=1/2`, `W=0` and `X=9/4`, so this is a genuine pole, not a removable projective zero. On the requested `[2,5/2]` interval, the source is finite and regular.

The equivalent compact native chart `s=2+u/2` has the controls `(-1/8,0),(-1/8,1/16),(-7/64,1/8)` and weights `9,12,16`. Its homogeneous coefficients are `X=-9/8-(3/4)u+(1/8)u^2`, `Y=(3/2)u+(1/2)u^2`, `W=9+6u+u^2`. Exact fraction arithmetic independently verified both Bernstein-to-power expansions. These controls happen to match the earlier native polynomial controls; the unequal weights give a different parameterization of the same parabola.

The public matrix has four charts: polynomial native, polynomial exterior, rational native, rational exterior. Both policies, both operand orders and both source reversals give 64 cases. The expected geometry is one transverse contact per case, with exact public location/point replay. The first combined run timed out before all chart combinations were visited; actual results are recorded below.

## Lower machinery audit

The circle geometric entry now keeps explicit finite ranges, but `parallel_system`, `represented_center_parallel_system`, `recursive_selected_radial_target_system` and `direct_pair_radial_parallel_fast_path` still invoke `certify_finite_source` and native regularity checks on their target source. The selected-normal fixed-distance builder also invokes native checks on the candidate source. Those predicates isolate unit-interval roots irrespective of the active domain.

Decide the ownership boundary carefully. Equation construction is formal homogeneous algebra and can often be domain-independent. Finiteness, normal-sheet selection and positivity must still be proved on consumed roots/cells or on the actual finite/ray domain. Inspect every contact and positive-dimensional replay consumer before removing a premise; a cached equation must not silently retain a proof that depended on a different domain. Existing finite ordinary-pair code already separates native discovery assumptions from exact candidate replay and may provide reusable machinery. Avoid adding exterior-only wrappers, duplicating a solver or weakening pole/zero-speed rejection at actual candidate points.

Selected centers need local frame proof, not whole-source finiteness. The existing certified-center branch in `parallel_fixed_distance_system` already expresses part of that distinction. Unrestricted center discovery and selected-point target incidence are related remaining audits, but need not be conflated with a completed bounded circle-equation migration.

Dependencies remain pinned. The other session owns Hyperreal verification and associated changes; leave them untouched.


## Executed baseline and unresolved cost

The frozen `finite-circle-native-poles-public.rs` has SHA-256 `4c0b3a6dd6472258d37dd226209c3261a7341ea0cbb149af77ac2b3de1ce552c`. Its first combined run completed the 16 strict-policy polynomial cases, then exceeded 120 seconds before completing the first compact rational case. `finite-circle-native-poles-baseline.json` records exit 124, the exact command, source, executable and archived normal-library hashes. It is not a complete 64-case matrix.

A second frozen source, `finite-circle-native-poles-charts-public.rs`, selects one chart per process and reports phase boundaries. `finite-circle-native-poles-charts-baseline.json` records:

- Chart 3, rational exterior: all 16 combinations reached intersection and returned `Uncertain(Boundary)` pair blockers, with no contacts or point replays; runtime about 0.008 seconds, exit 101 from the expected-completeness assertion. Finite endpoint evaluation, analytic-fragment admission and boundary-path export succeeded under both policies.
- Chart 2, rational native: finite endpoint evaluation, fragment admission, boundary closure and path export succeeded. The first strict-policy, unreversed, unswapped intersection exceeded 60 seconds. This is an observed intersection stall; its precise internal cause has not been localized.

Both executions use the archived normal library from the qualified commit, SHA-256 `8eb7cd1fbaa241bf01e599e07ca251959c800c77c62a8091232bad9d52006638`. No earlier commit was run on this new rational fixture, so these results do not establish that the last migration introduced the stall. No production source was changed after that commit.

The source-oriented regularized tangent worker already divides a common hodograph factor and selects its sign on a regular cell. It is a possible simplification to investigate for these rational charts, where the unreduced homogeneous tangent contains an extra factor. Reusing it would require a proof over the consumed finite/ray domain; choosing a sign at one arbitrary sample alone cannot certify a domain crossing a factor root. Likewise, clearing source-denominator factors from a projected incidence polynomial is valid only with explicit nonzero evidence on the consumed domain. These are hypotheses for the next implementation, not demonstrated performance repairs.


## Stack localization during the frame-replay implementation

A sandbox-approved debugger run of the same archived parent executable interrupted the native case after five seconds. The stack is bound to executable/library hashes in `selected-point-frame-replay-parent-stack.json` and retained in `finite-circle-native-poles-native-stack.log`. The contact had already been returned: the public test was comparing its point with the point evaluated from its location. Equality reached `BezierAnalyticParallelPoint2::represented_coordinates`, then selected-fiber global promotion and a costly local Sturm sequence. Thus the earlier phrase "intersection stall" refers to the composite intersection-and-replay query, not root discovery itself.

Inspection identified raw and reduced hodographs on the same support and retained parameter. `point_evidence_on_regular_range` retains a common-factor-reduced tangent field, while contact evidence can retain the raw differential. Structural equality alone does not identify these equivalent unit directions. The implementation now compares their cross and dot products in the existing source-parameter field, with strict decisions under the original policy. It also distinguishes opposite or rotated frames for nonzero displacement and handles zero displacement without a unit frame. The frame-replay fix is committed as `5c115f5171c0c90623cddaa7c268bac4d316d758` and qualified with 2,300 passing tests, six focused tests and 36 public probes, with unchanged known failures/exclusions. The unused-native-pole premise remains separate work.


The frame-replay candidate now completes the unchanged full 64-case source in 4.22 seconds: 48 contacts and 96 exact point replays succeed, and all 16 rational-exterior cases still return Boundary blockers. This is recorded in `selected-point-frame-replay-remaining-circle-domains.json`, separately from passing qualification. The earlier tangent-GCD/eliminant scheduling hypotheses do not explain the observed timeout; the stack and repaired public replay identify point-equality reconstruction as its cause.

For the next domain migration, separate formal equation construction from the finiteness/regularity evidence needed to consume it. Four selected-circle backends still build homogeneous equations behind unit-only target checks, and the selected-normal backend shares a fixed-distance builder with chamfer queries. Their equations can be shared across finite ranges if admission proves source weight and required source speed nonzero on the actual domain. Positive-dimensional replay also needs that range proof before a sample can certify a whole connected cell. Existing `polynomial_is_nonzero_on_parameter_range` supplies finite proof without promoting selected endpoints. Incident extensions must retain their open regular barrier semantics; a finite-only check is insufficient for a ray.

Inspect both standalone candidate replay and cached parameter-map consumers when moving a premise. Direct pair-radial candidate replay already has pointwise `target_is_regular` checks. Recursive systems retain weight and speed in their candidate evaluator. Rational-frame and selected-normal contact/overlap consumers require explicit review before relying on pointwise filtering alone. Keep formal equation caches independent of domain only when no domain-dependent reduction or proof is stored inside them. The public matrix is an exact 16-failure baseline for the remaining migration; broaden it with active pole/source-stationary rejection and other circle-frame families as those premises move.


Current authoritative parent for the next implementation: Hypercurve `5c115f5171c0c90623cddaa7c268bac4d316d758`. Use `selected-point-frame-replay-remaining-circle-domains.json` as the completed 64-case baseline (48 contacts, 96 point replays, 16 Boundary failures), bound to the final archived normal library. All build/test/probe sessions from this stage were reaped before committing. Hypercurve and HyperBREP are clean; no global workspace-clean claim is made.


## Resolved circle target-domain premise

Hypercurve `fedd740a91a89456b7e655db32f94dd8f702136c` resolves the recorded 16 rational-exterior blockers. The unchanged 64-case source now passes all contacts and 128 exact point replays; see `finite-circle-source-domain-poles-public.json`, bound to the final archived normal library. The formal-equation/range-admission migration is qualified with 2,301 passing tests, 11 focused checks and 37 public probes, with unchanged known failures/exclusions. Historical parent failures above remain authoritative for their own artifacts. Follow `finite-fixed-distance-domains-next.md` for the next unexecuted source audit.
