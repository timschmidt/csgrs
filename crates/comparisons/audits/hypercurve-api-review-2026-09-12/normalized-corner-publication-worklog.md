# Normalize corner operation publication

Parent: Hypercurve `07d6b300c109eaa304371ba61898b29cd262bc6d`, with pinned Hyperreal `a2da8e2b5de9a1a4653d3662f2f05cb6533fd7ef`. The other session's working Hyperreal changes and the unqualified working offset inverse are excluded from builds.

The bound public square/hole witness fails publication invariants in all four parent cases (two operations, two policies). Explicit unary normalization gives the intended set. The chamfer has exact area 51; obsolete hole edges cease to own boundary points. Candidate 1 retains the certified corner joins and invokes the existing normalization authority before returning.

Candidate 1 passes the new public regression and 92 other promotion tests. Seven assertions fail and three cases exceed the exploratory 90-second bound. Parent replay establishes five pre-existing assertion failures, including the projective fillet case that now reaches the time bound. The new findings are:

- The boundary-path comparison helper assumes one loop, although an extended circular chamfer can normalize into multiple components. Its independent expected native construction also normalizes; compare that geometry and reconstruct all boundary paths.
- A one-sided zero chamfer introduces a redundant collinear subdivision. The old five-segment assertion must become an exact identity/area/boundary check on the normalized four-sided square.
- A higher-order fillet fails with `algebraic cusp split boundaries are not increasing`. Diagnose the parameter/contact evidence; do not bypass normalization or silently discard a candidate.
- Arc/parabola and line/parabola extension tests that pass on the parent exceed 90 seconds on the candidate. Their previously uninspected corner candidates now require normalization too. The projective Bézier pair extension test also exceeds the same bound. A bound is not proof of nontermination; longer runs and targeted stage diagnostics are required before qualification.

Source, build, executable and per-case records: `normalized-corner-publication-candidate1-*`. No candidate is committed while these regressions remain unresolved.

The three slow cases also exceed 300 seconds. Diagnostic build 1 passes both corrected segmentation assertions. The higher-order fillet admits parameter 1 as interior to a selected-circle range whose mapped end brackets in [0, 1/16], then fails the ascending split check. The contradictory angular/physical incidence needs diagnosis; the check is retained. Line/parabola stalls in face classification after split topology completes. Arc/parabola progresses through several candidates and slows in both contact reconciliation and face classification. The projective pair stalls while reconciling its first circle/Bezier contact after pair replay completes.

Tracing the normal-angle predicates exposed a separate reproducible source/parallel direction error. It is fixed and independently committed as `5206f58`, with its own parent/candidate angular oracle and 1,188 passing library tests. The corner publication candidate remains uncommitted; further tracing will use this corrected predicate. Diagnostic instrumentation exists only in archived isolated snapshots, never in production commits.

The remaining local-orientation defect is independently fixed in `13b193f`: the new fillet center support need not be regular over the source range, so orientation must be replayed at each actual selected contact. Parent `5206f58` gives a positive source-tangent dot product for P(t)=(4-2t,8t(1-t)) against the positive x-axis, although P'(t) has x component -2. Candidate passes all 20 contact/traversal checks under both policies, plus unprojectable fiber replay; 1,190 library and 97 promotion tests pass. Five promotion failures are reproduced on the archived parent. This does not yet qualify normalized corner publication; diagnostic 3 reruns it with the committed local predicate.
