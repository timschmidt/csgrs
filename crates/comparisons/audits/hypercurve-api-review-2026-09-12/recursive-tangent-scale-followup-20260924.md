# Recursive tangent combination: exact speed-scale mismatch

Discovered read-only while retained-parallel-expression V3 qualification is
live. No source changes or new probe has run for this finding yet.

The recursive selected-radial system stores circle/source tangent cross as
C=-turn*W*(R dot H), whereas its tangent-dot expression stores sqrt(S)*D,
with D=turn*(W*(R cross H)-d*W^2*center_denominator*sqrt(S)).
Its tangent_cross_dot_source_expression currently puts cross_scale*C in the
rational component. Its later native evaluator likewise adds polynomial C to
the speed-scaled dot expression. Thus a mixed combination uses unequal
positive scales. This can change its sign, even though cross-only and dot-only
queries are individually correct. The represented system places C in the
radical component; the pair-specialized system likewise multiplies it by the
candidate speed. Unit-target-speed rational cases hide the discrepancy.

Independent contact oracle: the fixture's recursive circle is the unit circle
centered at the origin. Let P(u)=(1/5+3u,17/20+4u+u^2), offset left by d=1/4.
At u=0, H=(3,4), sqrt(S)=5, Q=(0,1). The circle tangent is turn*(-1,0), so
cross=-4*turn and dot=-3*turn. Therefore 3*cross-4*dot is exactly zero, and
(3+epsilon)*cross-4*dot has sign -turn*sign(epsilon). Current recursive
assembly instead produces 48*turn at epsilon=0, apart from one common
strictly positive homogeneous factor. A direct pair accelerator can mask the
bug, so the regression should explicitly exercise both the available pair
backend and the recursive backend with that optional accelerator absent.

For a genuinely retained local parameter, compose u=t^2-2. Then
P(t)=(3t^2-29/5,t^4-63/20). At the positive local root t^2-2=0 in (1,2),
Q=(0,1), H=2t*(3,4), sqrt(S)=10t. The same cancellation and shifted-sign
oracle applies, with the extra strictly positive factor 2t. This also exposes
the current nonconstant-speed rejection in the generic parameter wrapper.

The coherent fix should accept CurveParameter2 at the two recursive tangent
and diameter query boundaries, keep their specialized native accelerators,
and build each exact predicate once for interval and retained/native replay.
Remove the superseded native/region overloads and migrate all callers. The
cross term belongs in the positive-speed radical component. Shared
A+B*sqrt(S) replay can then replace the separately implemented chord incidence
sign case tree without changing its source-weight orientation or admission.

V1 diagnostic on parent 714138215f94bfbe19f8a5555be28aa9e7f37714
reproduced a wrong strict sign in 0.01 library seconds: Positive versus the
independent Zero oracle. All production and original tests matched the parent.
The executable, all 2,044 source inputs and process termination are bound in
the V1 receipts; every owned process was reaped before editing.

V2 moves the cross term into the positive-speed radical component, matching
the represented and pair paths. One tangent and one diameter query now accept
CurveParameter2 directly. Their optional native pair and interval accelerators
remain, and both use the existing expression evaluator after those paths.
The separate native/region overloads and duplicated terminal evaluation are
removed, and the inverse-map replay no longer narrows a common parameter just
to perform a diameter predicate. The new test changes only the two migrated
method names and formatting; all mathematical inputs and assertions remain.
All source bytes are now frozen for V2 qualification.

V2 formatting, both all-target Clippy feature configurations, and all ten
focused release cases pass. The new regression takes 2.734 seconds including
startup, both parameter forms, both optional-accelerator settings, and both
policies. Exact cancellation, +/-2^-600 signs, strict circle incidence, diameter
ordering and Certified accounting all pass with global projection absent.
The 1,470-case broader qualification is live; source bytes remain frozen.

V2 completed and committed as 9be44c2eec926e71dc2ea0b140e768e9542008f2. Full qualification records
1,470 attempted cases: 1,460 passed, six ignored, and the same four 75-second
timeouts. All 225 public integration cases pass. The three isolated long
composition cases take 37.22, 144.47 and 59.50 seconds within their unchanged
limits. Every prior passing case remains passing. The staged/post-commit
receipts bind all 2,044 inputs, all executables, the index and committed bytes.
All thirty repositories are clean; every owned process is reaped; nothing was
pushed. The full goal remains active.

The change removes 140 production lines while adding the 130-line regression.
The source-factor correction is shared by interval and exact replay, and no
compatibility wrapper remains for the removed query overloads. Chord incidence
still has its separate positive-root case tree; its consolidation and local
all-roots enumeration remain follow-up work, not behavior claimed here.
