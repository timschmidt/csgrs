# Finite Bezier self topology and injectivity

Parent: hypercurve 84956dabadf9afcef4704f5b2dee880a282f1939. Goal active; this is one implementation stage.

## Baseline discovery

The independently derived cubic P(t)=(t²−1,t³−t), active [-2,2], has off-diagonal visits (-1,1) and (1,-1) to the origin. Its native unit chart is injective. Equivalent Q(s)=P(4s−2) uses the native unit interval. Both close with a vertical chord.

`finite-region-self-domain-baseline.json` is the set-only probe, source preserved as `finite-region-self-domain-set-only-public.rs`; it passed sampled point locations and Boolean lobe clips in both policies. Different normalized loop counts alone are not treated as a failure.

`finite-region-self-domain-ownership-baseline2.json` adds actual boundary ownership: export normalized paths, locate the regular leftmost point exactly, and test both normal sides against the certified filled-side flag. It has four failures, all on the exterior chart (both sides, both policies). Both wrong classifications were Certified. The native chart passes. Source SHA 90cb9bb76517881a9067f959549a04ddb2b0b89a4b595364c08cc6071fe1a8bf; archived parent normal library cfe74e6bb84e6907ba989a26023d587fb8d2fd4912961045b76c518bd98b1f39.

The first ownership probe assumed all published cuts had scalar payloads and stopped on a selected parameter. Its source is `finite-region-self-domain-ownership-attempt1.rs`; its runtime record is `finite-region-self-domain-ownership-baseline.json`. The corrected fixture uses exact incidence to find the known regular point without reconstructing selected cuts. An earlier compile-only attempt passed Real where CurveParameter2 was required, then corrected the caller before execution.

## Implementation and qualification history

Make the support injectivity shortcut require its active range. Bezier self-contact discovery uses the shared finite rational kernel, excluding the structural identity diagonal before projection. The existing symmetric native-unit solver remains an isolated-contact fast path under its domain premise. Common pair contact/component evidence is published once through CurveIntersectionContext. Remove RegionCarrierPairContext::BezierSelf, its cache, builder, adapter and unused initialization sites. Require exact quadratic reduction before arbitrary-degree circular provenance can certify injectivity.

Sources are frozen while owned builds, probes and tests run. No performance improvement claimed. Parallel self-domain ownership and remaining analytic range premises remain separate unfinished work.

The first candidate passes the corrected public ownership reproducer in both policies. Its exact source patch and normal library are archived in `finite-region-self-domain-attempt1/`, before adding production regressions. The test-only compile omission is preserved in `finite-region-self-domain-attempt2-check.log` and its source patch; it called an overlap field as a method. The warning exposed an unused native-axis convenience method, which was removed and its two tests migrated to the classified authority.

The frozen qualification candidate adds four tests: active-domain and circular-chart injectivity; self-contact endpoint and selected-root replay across degree 3/5 and reversal; one retained retracing component excluding the stationary diagonal; and normalized ownership through reversal, elevation, repeat regularization and Boolean self-union. The public reproducer also checks ordinary curve self-pair contacts and both lobe clips.

The first frozen full build completed (HC 225.12s, HB 113.04s), but the new region regression blocked on reversed traversal. Its matching sources, normal library and libtest executable plus focused results are archived in `finite-region-self-domain-attempt3-regression/`; no full suite or public matrix run was executed on that candidate. Three new lower-level regressions and all three selected existing regressions passed.

A standalone public diagnostic distinguishes chart, elevation, direction and generation. It reproduces the reversed unit-chart blocker on the qualified parent as well; the exterior parent skipped self-topology and returned the incorrect single-owned-side boundary. See `finite-region-self-domain-reentry-parent.json` and `finite-region-self-domain-reentry-attempt3.json`. Diagnostic returncode 0 means the report completed, not that its printed ERROR rows passed.

The follow-up source audit found two self-branch lookups pairing traversal-oriented topology endpoints with unadjusted source-range endpoints. Both now apply the carrier traversal direction when choosing the source cut. This is a topology parameter identity repair, not a change to curve equations or a coordinate approximation. Focused retesting is pending.

All seven exact focused checks passed after the source/traversal endpoint repair (`finite-region-self-domain-attempt4-focused.json`). The region matrix covers both directions, both charts, polynomial/degree-five representations and both policies, with repeat regularization and Boolean self-union. A final degeneracy guard also requires distinct endpoints before quadratic-circle provenance certifies injectivity, and tests a genuine public subcurve collapse that inherits circular support. This final change is part of the new frozen qualification.

The next frozen build also completed (HC 224.57s, HB 115.85s). All seven focused checks, 27 public matrices and three caller checks passed. Before full-suite execution, an additional asymmetric retracing probe P(t)=(t²,t⁴), t∈[-2,1], closed by the chord (1,1)→(4,16), exposed a retained-domain Ordering blocker. The unit-chart equivalent already normalized correctly. The qualified parent reports four native-chart blockers and eight incorrectly certified boundary classifications on the canceled exterior spur (12 failures total); the candidate has four exterior blockers (no incorrect classifications). See `finite-region-self-retrace-parent.json` and `finite-region-self-retrace-attempt5.json`. Matching sources and binaries plus executed checks are archived in `finite-region-self-domain-attempt5-retrace/`. No full suite was run on that candidate.

The blocker exposed finite support side sampling calling the unit-only first derivative evaluator. The rational quotient derivative computation is now shared with an affine support entry point, retaining denominator certification and the cheap first-order path. The support regression checks exact cubic derivatives at exterior parameters; the self-component regression now also requires regularized cancellation, including selected domain bounds. The final public matrix will include the asymmetric retracing probe.

All seven exact focused checks pass with the affine derivative repair, including actual asymmetric retracing cancellation for ordinary and selected finite bounds (`finite-region-self-domain-attempt6-focused.json`). Libtest SHA f9f496002e209743c785052b1e09be7f2b19ac461856012a5ee24089ce22fa83. Production and test sources are now frozen for final builds, public matrices, caller checks and the full baseline-qualified suite.

Final builds passed (HC release 133.93s, HB release 112.54s), as did all 28 public matrices and three caller checks. The new public ownership and asymmetric retracing probes each complete with zero failures. The seven focused checks were reused after matching their exact final libtest hash; none was rerun merely to regenerate a report. Full-suite execution is in progress; this paragraph does not claim full qualification.

## Final qualification and commit

Committed f46b62827a4c2c7fc99929243b18f45133a1324e. Final qualification: 2,274 passes across 49 targets (2,042 Hypercurve and 232 HyperBREP), five unchanged known failures, nine ignored tests, eight previously unqualified expensive exclusions, no new failures and no timeouts. All 28 public matrices, seven focused checks, three caller checks, formatting and all 396 frozen source hashes pass. Post-commit verification finds all thirty repositories clean. See `finite-region-self-domain-qualification.json` and `finite-region-self-domain-post-commit.json`. No performance or memory improvement is claimed. The original goal remains active; finite analytic self-domain ownership and the broader remaining work have not been declared complete.
