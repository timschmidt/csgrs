# Planned conic endpoint contact certificate

This is a mathematical candidate, not a qualified implementation.

For a pole-free rational span P=(X/W,Y/W) with homogeneous power degrees at
most two, its oriented incidence H against any fixed affine line has H=F/W,
where F has degree at most two. Suppose a caller already owns an exact contact
at one endpoint b of the finite parameter interval [a,b]. A strict derivative
sign on a nonempty terminal subinterval gives the sign of H immediately inside
b. If H(a) has that same strict sign, there is no other root: a second simple
root would reverse the sign, and another even-multiplicity root together with
b would require degree at least three. A double root at b itself is allowed.
Constant-sign W is required on the entire closed interval. Traversal reversal
and homogeneous gauge do not change the argument. A failed sufficient proof
must leave the full incidence kernel available.

The existing rational-source point evidence, construction-local chord side
predicate, strict interior scalar, and retained-range tangent-cross certificate
can supply these obligations. No new point or parameter representation is needed.
Checking actual homogeneous power degrees also admits degree-elevated conics
whose higher coefficients are structurally zero. Merely checking endpoint
sides or the authored unit interval would not prove this theorem.

Positive test: P(t)=(2t^2,t), endpoint u selected by 4u^2=alpha over
alpha^65=1/2, and chord direction (1,2). On [0,u], the derivative cross
1-8t changes sign, so full-span monotonicity cannot discharge the endpoint.
Nevertheless the other incidence root 1/4-u is negative. The local terminal
sign and opposite-endpoint side certify that u is the only contact.
The cache/global parameter representation should remain untouched.

Negative conic test: change the chord direction to (1,1). Its other root
1/2-u lies strictly inside [0,u]. A chord extending backwards by (1,1) from
P(u) contains that second contact. The sufficient certificate must decline,
and the complete solver must retain the second root.

Negative higher-degree test: X=t, Y=(t-1/10)(t-1/5)(t-1),
W=(t+1/100)^3. W is positive on [0,1]. The horizontal chord from P(1) to
(100,0) owns t=1 and also contains both contacts t=1/10 and t=1/5.
H(0) and H immediately below 1 are negative, and H' is strictly positive
on [1/2,1]. Its derivative numerator, after removing (t+1/100)^2, is
133t^2/100-333t/500+79/1250, positive and increasing on that subinterval.
Thus even a valid terminal monotonicity proof plus matching endpoint sides
would wrongly drop two contacts if the degree guard were omitted.

The existing selected-chamfer fixture's original closing chord also has a
second intersection: t=u^2/(u+5), in addition to its owned t=0 endpoint.
It must continue through the complete kernel. This candidate only aims to
avoid unnecessary reconstruction for genuinely complete owned contacts.

## Frozen v1 candidate

The private sufficient certificate now runs after complete finite-range pole
admission in the existing rational intersection entry point. It reads actual
homogeneous power degrees, verifies ownership at a range endpoint, obtains the
opposite side from construction-local bounds, and reuses the existing tangent
cross certificate on a terminal subinterval. A declined bounded proof leaves
the original complete intersection paths available. No representation or
compatibility API was added.

Four new regressions cover selected degree-65 endpoint evidence, a finite
extended range, double endpoint contact, reversed ranges/chords, gauges 1/-3,
degree elevation, a second conic root, the two cubic crossings, and an interior
pole. The cubic test independently verifies its terminal derivative sign and
opposite endpoint side before requiring both unowned contacts. The pole test
uses X=t, Y=(t-1/4)(t-2), W=t-3/4 on [0,2]; the entry point must decline
the non-finite range rather than discharge its owned endpoint.

The immutable snapshot is `/tmp/hypercurve-conic-owned-contact-v1-20260924`;
`conic-owned-contact-20260924-v1-sources.json` binds all 2,044 inputs. Both
all-target Clippy feature configurations pass with warnings denied. The release
build and runtime qualification are pending. Source bytes remain frozen while
the owned build/test driver is live. No conic certificate change is committed.

## V1 outcome and v2 hypothesis

V1 compiled cleanly but the positive local-evidence regression declined a valid
case. Its original driver stopped at that failure. A separate run of the same
immutable executable passes all three negative regressions. The existing
selected chamfer reaches a certified result in 1.61 seconds, then fails its
loop-zero-only retained-cut assertion; its former 75-second timeout is gone in
this bounded first-policy observation, not yet a full regression pass.

V2 tries up to four successively closer terminal neighborhoods before declining
the optional proof. A first scalar gap can land on another tangent zero (the
positive fixture's 1/8 is one such critical parameter), so a single terminal
interval is stronger than the local theorem requires. This is a diagnosis
hypothesis until the positive regression passes. The full isolator remains
available after the bounded attempts. Failure messages now identify the policy,
source degree, selected/scalar case, direction and endpoint position without
printing recursive values.

The selected-chamfer caller now searches every normalized boundary for its
original selected-fiber cut obligation, requires filled-left topology, and adds
certified point queries in both surviving lobes, in the removed corner, and
outside the source extent. The original local-image and no-global-center trace
assertions remain. All v1 processes were terminal and reaped before v2 edits.
The v2 build/test driver is live against its new immutable snapshot.

## V2/V3 diagnosis

V2 still declines the first positive case: STRICT, degree two, selected endpoint,
chord direction (1,2), owned endpoint at the range end. The four-neighborhood
hypothesis did not fix it. V3 changes only failure diagnostics in the test and
confirms the earlier blocker: comparison with the opposite exact-zero endpoint
is `Uncertain(Predicate)`, while comparison with the owned endpoint itself is
`Decided(Equal)`. The independent origin-side query is `Decided(Left)`, and
the first interior-scalar query declines. Thus the failing proof never reaches
the tangent query. The bounded selected-fiber comparator intentionally uses
only the stored singleton before declining; the raw singleton still touches zero.
The next revision must retain native refined endpoint envelopes, not add more
terminal-neighborhood retries. Those speculative retries will be removed.

All three negative regressions and the existing ownership, root-import and
monotonicity regressions pass in V3. The strengthened chamfer test exceeds 75s.
The eight-second capture `conic-owned-contact-20260924-stack1-{terminal.json,log}`
is bound to binary SHA
`dadc6d8d34a70e361bb60c76f1e5bff2c2bb30877384ecd027e9492b85194d57`.
It samples chamfer regularization's boundary-probe/chord intersection path,
where derived chord-pair endpoint bounds trigger recursive point reconstruction
and complete selected-fiber projection. It does not sample `classify_point`,
so attributing this cost to the added point queries is unsupported. The current
stack also does not identify the policy iteration. The debugger killed and
reaped its own inferior. V3's remaining focused driver is still live at writing.

The strengthened query expectations have independent exact-fraction bounds in
`conic-owned-contact-20260924-region-query-certificate.json`: 49/100<u<1/2;
the small-lobe closing x at y=3/200 exceeds 7203/5500000>1/1000;
the large-lobe closing x at y=1 is below 50/549<1/5; and the chamfer y
at x=12/25 exceeds 609/1100>1/2. These are geometric bounds, not output
layout assumptions.

## V4 native-envelope correction

All V3 processes are terminal and reaped. V4 removes the speculative four-
neighborhood loop. It retains the existing native endpoint refinement results
before asking bounded order, side, and terminal-tangent predicates. Four native
refinement steps are only this sufficient proof's local opportunity; any decline
still reaches the complete solver. No scalar or selected-fiber comparison API
changes, new representation, or cache is introduced. Temporary bounded scalar
failure diagnostics are removed. Four test-only phase markers identify the
policy and construction/query stage of the still-open chamfer replay. Those
markers are diagnostic and must be removed before final qualification/commit.
The new immutable v4 build/test driver is live.

## V4 observed progress and remaining cold bounds

All four new conic regressions pass, including the previously declined selected
endpoint; native endpoint refinement is the confirmed correction. The original
ownership, low-degree root import, and monotonicity regressions also pass.
Phase markers confirm that STRICT chamfer construction finishes, the retained
cut is found across the normalized loops, all four certified geometry queries
pass, and all original trace obligations pass before the test begins its
APPROXIMATE_512 iteration. The test then exceeds 75s during that second
chamfer construction. The point-classification suspicion was not confirmed.

The earlier stack's cold path is `chord_intersections_once` requesting
`conservative_bounds_refined(0)` before support predicates. In
`algebraic_chord_endpoint_bounds_refined_impl`, a composite point's unsuccessful
local zero-step enclosure immediately falls back to recursive global point-field
construction. The caller's later 2/4/8-step bounding opportunities are not reached.
A possible shared fix is to try a small increasing native refinement schedule
when the first composite enclosure declines, before retaining the existing
complete field fallback. Returning a tighter certified box than requested is
valid, and preserving the complete fallback avoids restricting any representation.
This remains a hypothesis awaiting a direct retained-intersection regression.

## V5 composite-bound candidate

All V4 processes were reaped. The common endpoint-bound dispatcher now tries
native refinements at 2/4/8/16 steps after a composite's requested enclosure
declines, skipping steps no finer than the requested refinement. Only success
returns early; the existing complete recursive-field fallback remains intact.
No bounds interface or representation changes.

A new regression retains alpha^65=1/2 and u^3=alpha, constructs the exact
intersection of P(u)->P(u)+(1,1), P=(2t^2,t), with y=3/2, and verifies
that the initial native determinant enclosure declines. Its complete bound
query must then succeed under the bounded exact policy at 0/8/32 requested
steps. The independent exact Real u=(1/2)^(1/195) supplies the geometric
oracle (2u^2-u+3/2,3/2). Global parameter and point-field caches must remain
absent. Both policies are covered. The physical v5 snapshot binds all 2,044
inputs; its ten focused build/test cases are pending. Diagnostic phase markers
in the region test still require removal before final qualification.

## V5 outcome and final V6 qualification

All V5 processes are terminal and reaped. The five new local-evidence tests
pass. The strengthened selected chamfer passes both policies, all four certified
point queries per candidate, and the retained cut and dispatch obligations in
5.24 seconds including process startup. The separate existing extended-fillet
test still exceeds its 75-second bound. Both all-target Clippy configurations
pass with warnings denied.

V6 removes the four temporary phase markers without changing production bytes.
The fresh immutable snapshot binds 2,044 inputs for final checks and broader
qualification. Rustfmt and diff checks pass before launch. Source bytes remain
frozen while any owned qualification process is live.

## Qualified commit

Committed `2e13ca8308747e25728af3c35ae35f39609418e3` after final V6 focused and broad qualification.
All nine focused cases pass. Broad: 436 attempts, 430 passes, three unchanged
ignored cases and three unchanged baseline nonpasses; all 121 public cases
pass. Source/index/HEAD bindings agree; all owned processes are reaped; all
thirty workspace repositories are clean. No push.
