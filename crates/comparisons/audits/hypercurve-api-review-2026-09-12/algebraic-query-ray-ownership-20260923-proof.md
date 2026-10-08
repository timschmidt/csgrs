# Algebraic-query spatial endpoint ownership

Parent: afa52c676b7712e3114fcf1132102b969f04f238. Dependency sources are the pinned snapshots recorded in the candidate/focused source bindings; the other session's live Hyperreal worktree is excluded.

## Independent regression

Keep the query Q=(alpha,0), alpha=sqrt(1/2), as a rational-curve image of a selected algebraic root. The left ray has side predicate f=-y and forward predicate alpha-x. Three independently understood polynomial pieces lie wholly at x<=0:

- Quadratic controls (0,0),(-4,2),(-1,-1): y=4t-5t^2. Its start approaches the negative ray side, and its interior root t=4/5 crosses to the positive side. Winding contribution +1.
- Quadratic controls (0,0),(-1,0),(-2,-1): y=-t^2. Its tangent start owns the positive interior side. Contribution +1.
- Closing chord (-1,-1)->(0,0): positive interior side at its end. Contribution -1.

The first and third pieces close a simple loop. Its horizontal interior section is -48/25<x<0; thus Q is outside, (-alpha,0) is inside, and (-alpha,-alpha) lies on the diagonal boundary. These facts do not depend on Hypercurve classification. The tests compare retained-root and exact-Real query forms, both traversal directions, both predicate policies, and positive/negative homogeneous gauges. A cubic with y=(t-1/4)(t-1/2)(t-3/4), x=2-4t forces subdivision through the middle contact, with one root behind the query and two ahead whose contributions cancel. A separate retained-range test places alpha itself at either finite endpoint of an upward/downward vertical segment.

The baseline executable bccdd12d17bce30f208e5ef82e103bcd8efbef878fa7a7d20f37a5a4ef70e5a1 fails both new endpoint-ownership tests: native subdivision drops the positive tangent start, while retained-root winding incorrectly includes the arch's negative start crossing. The public algebraic classifier previously avoided such vertex rays using reconstructed endpoint images; this guard hid the inconsistent internal counting rule. This patch removes that rational-fragment restriction after repairing the rule.

## One rule, two exact mechanisms

For a finite contact, let b and a indicate a strict positive ray side immediately before and after the root. Its contribution is [not end and a]-[not start and b]. An interior root uses a-b, a start uses a, and an end uses -b. Traversal reversal negates the contribution without changing ownership. Represented-query contacts and retained algebraic-query roots now use the same helper. Closed interval incidence stays separate from winding ownership; the obsolete half-open/reversed options are removed.

For selected algebraic roots, the existing correlated first-nonzero derivative certificate supplies the after-side sign. Odd order reverses that sign on the before-side; even order preserves it. Thus even multiplicity still contributes zero in the interior, but finite endpoints correctly contribute their interior side. Origin skipping retains its separate transverse-contact requirement. No query/root coordinate is materialized or independently reconstructed.

For a rational Bernstein piece with one nonzero weight sign and every control at or ahead of the ray origin, every interior point lies strictly ahead whenever at least one control is strictly ahead. All finite contact contributions therefore telescope to [f(end)>0]-[f(start)>0], including zero endpoint values. Applying this to both children makes the artificial subdivision endpoint cancel automatically. The previous midpoint root/sign branch is unnecessary. Endpoint-origin coincidences and forward collinearity remain uncertain boundary events. The control-hull subdivision fast path is retained.

## Simplification and closure

The rational ray fragment no longer stores endpoint CurvePoint2 images solely for vertex-ray rejection, and preparation no longer constructs those images. Selected query/root proof authority stays in the existing Hypersolve-backed predicates. Scalar and selected-root queries now share finite ownership semantics. There is no new public API, alias or compatibility shim. Full-family Boolean/offset/corner closure is still an active goal; this patch does not resolve the other recorded baseline failures or time limits.

Qualification results will be recorded after all candidate processes are terminal/reaped.

The candidate removes 112 production lines from bezier_region.rs; test coverage grows by 239 lines. Focused1 passed all seven selected tests. Full qualification is still running under session 30134; no source changes are permitted until it is terminal/reaped.

Final qualification: broad1 session 30134 completed/reaped with 1,224 attempted, 1,197 passes, six ignored, nine unchanged assertion failures and twelve unchanged 75-second limits. No new or changed nonpasses. Both all-target configurations passed. The sole post-suite change deleted the now-unused retained_point_linear_difference_to_algebraic_sign import; an exact byte comparison proves this is the only difference from the frozen full-suite source. Clean1 session 94471 compiled all targets with all features and no default features without warnings, then exited/reaped. The final production reduction is 113 lines; no test or semantic code changed after broad1. All final main Hypercurve inputs match the clean snapshot.
