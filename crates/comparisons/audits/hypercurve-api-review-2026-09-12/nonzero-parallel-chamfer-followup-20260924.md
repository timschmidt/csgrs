# Nonzero-parallel closed-quartic chamfer follow-up

The remaining nonzero-offset chamfer regression uses a quartic with controls
(0,0),(3,0),(0,3),(-3,0),(0,0), unit weights, left offset 1/4 and Euclidean
setbacks 1/2. Its input constructor certifies one regular authored span. The
unchanged full qualification still reaches the 75-second timeout. The test
first constructs TrimOnly, then TrimOrExtend, under both policies and both
traversal directions; its current log does not locate the expensive phase.

Independent source algebra: u=t*(1-t), C(t)=(12*u*(1-2*t),18*u^2).
The tangent is (12*(1-6*u),36*u*(1-2*t)), whose squared norm is
144*(1-12*u+45*u^2-36*u^3). Thus x(1-t)=-x(t), y(1-t)=y(t), and the left offset
has the same reflection symmetry when the positive speed sheet is preserved.
The offset seam is (0,1/4). This algebra can check a future exact oracle, but
it does not locate the timeout or justify a fixture-specific fast path.

A bounded stack driver is prepared for the committed retained-root-replay
executable. It must run only after focused verification, qualification and
commit receipts exist and every previous owned process has been reaped. It
traces only its own child, disables network symbol lookup, prints bounded
frames without arguments, and kills/reaps both the child and debugger. No
source changes or diagnostic timing will count as production qualification.

The committed stack1 diagnostic completed and reaped its own child and debugger
at 20 seconds. The sample is in dense rational/tensor multiplication inside
BezierDenseTwoSquareRootExpression2::projection, reached through
recursive_quadratic_polynomial_projection and chord/parallel incidence system
construction during Boolean normalization. This is a sampled location, not a
complete profile. Sources and the committed executable remain byte-identical.

Simply deferring the projection is insufficient: the complete enumerator
currently requests native Bezier parameters and every candidate uses the
native speed-field embedding. Existing local ordered-field isolation can
retain new roots, but their A(t)+B(t)*sqrt(S(t)) predicates need general local
parameter replay first. The retained-parallel-expression regression addresses
that capability before changing all-root contact representation or scheduling.

The retained-expression follow-up makes nonconstant positive-speed replay
possible at a local recursive polynomial root. The remaining all-roots work
should change BezierAlgebraicChordParallelContact2's parallel parameter to
CurveParameter2 and update its Boolean/corner consumers directly. The separate
monotone contact currently omits tangent-dot evidence, so it is not a duplicate
merely because both carry the common parameter. Preserve its distinct proof
requirements until a shared contact contract can satisfy those consumers.

Chord system construction currently takes independent build_projection and
build_coordinate_differences flags. Global dense projection is consumed by
parameters(), a projected candidate-box certificate, and opposite-sheet
component residual enumeration. The projection can become demand-driven;
finite-domain local Bernstein isolation can run before it, with global
fallback retained for repeated roots, whole components, and unbounded rays.
Local isolation reports must cover the complete finite envelope and filter
to the authored retained range before they can claim completeness. A local
norm root is only a candidate: W!=0, S>0, the unsquared positive-speed
expression, finite chord placement, and derivative orientation must all replay.
The common point constructor already accepts CurveParameter2 plus a supplied
regularized tangent frame; local contact publication need not promote the root.

The existing recursive chord incidence-sign method separately implements the
same A+B*sqrt(S) sign algebra, including source-weight orientation. Share its
component/magnitude replay with the radial system rather than add another
copy for local chord candidates. Diameter comparison in the radial system
also has a native-only/nonconstant-speed restriction that can consume the
same expression mechanism once its exact expression is formed. These are
next implementation targets, not completed or qualified behavior.

Read-only identity audit during recursive-tangent-scale V2 qualification:
BezierRecursiveProjectiveParameter2 keeps Projective, Monotone, or Polynomial
authority. Monotone authority already owns the defining side chord and exact
parallel support, whereas the generic polynomial root owns only its field,
polynomial and singleton bounds. ChordRationalTangent identity currently keeps
a RationalBezier2 source, chord traversal cross sign and finite chord location.
Its use is bounded: three attachment sites, one CurveParameter2 forwarding
method, and several structural replay queries in bezier_offset.rs.

A future all-roots contact can preserve its already-proved incidence by
generalizing this contact identity to the actual parallel support instead of
adding another construction-specific variant. Ordinary rational contacts are
zero-distance parallels. However, this is not a mechanical field substitution:
the equal-normal-offset side theorem compares the point's total offset to
the chord displacement and assumes the defining contact is on the zero-offset
source. That theorem must keep an explicit zero-distance/rational guard until
its relative-offset and derivative-orientation proof is generalized. Other
consumers only require certified incidence, tangent orientation, and finite
chord location, so they can accept the broader contact identity directly.

certifies_monotone_chord_incidence currently consults only monotone authority;
a common certified-contact query could consume either existing monotone proof
or the generalized contact identity. Keep parameter policy, actual parallel
support, tangent displacement, translations, chord support identity, source
chart and any regularized one-sided frame in the proof preconditions. Avoid
claiming every polynomial norm root has an incidence identity: attach it only
after strict positive-speed unsquared replay and complete finite admission.

A bounded all-roots oracle for the future local enumerator:
P(t)=(t,t^2/2), left offset d=1/4, horizontal chord support y=1/2,
and finite target domain [-2,2]. Put S=1+t^2. Incidence is
1/4 + ((t^2-1)/2)*sqrt(S)=0. Its squared-magnitude polynomial,
multiplied by 16, is -4*t^6+4*t^4+4*t^2-3. In u=t^2 this has
one negative and two positive roots; the positive roots lie in (1/2,3/4)
and (1,2). Hence the finite target domain contains four real norm roots.
Only the two with |t|<1 satisfy the authored positive-speed equation; the
others have both unsquared terms positive. All are simple, S>0 and W=1.
The parallel derivative scale is 1-(1/4)/(1+t^2)^(3/2)>0 throughout.
The authored contacts have |Qx|>1/2, Qy=1/2, opposite nonzero tangent-cross
signs, and positive source tangent-dot sign for a rightward chord.

The full chord [-2,2] at y=1/2 therefore retains two contacts; its left
portion [-2,0] retains one, and [-1/2,1/2] retains none. Both traversal
directions and policies should replay with the global projection cache
untouched on the local path. This checks all-root completeness, conjugate
rejection, strict finite clipping, and orientation independently of the
closed-quartic chamfer fixture. Tangencies, whole components and unbounded
incident rays keep the existing complete global fallback until their local
certificates are supported. Any existing tests specifically targeting the
dense selected-fiber algorithm must continue to exercise that algorithm
directly if a new local accelerator bypasses their former public call path;
do not merely delete their certificate assertions.

The same four-norm-root oracle can use an entirely authored unit interval:
substitute t=4u-2 and use quadratic controls (-2,2),(0,-2),(2,2). The two
authored contacts have u in (1/4,1/2) and (1/2,3/4); the two rejected-sheet
roots lie outside those middle quarters, all within (0,1). This avoids
relying on exterior finite-domain admission just to exercise local enumeration.

The derivative-orientation audit found an existing common-parameter route:
BezierParallel2::parallel_derivative_scale_sign already accepts CurveParameter2
and signs speed, signed curvature and their squared comparison through
parameter.polynomial_sign. Therefore a local root need not be promoted for
this contact predicate. Its neighboring tangent_side_at still takes a native
BezierParameter2 even though both of its polynomial queries can use the same
common parameter authority; that narrower entry point belongs in the later
tangent-parameter migration. Source regularity and singular-germ handling
must keep their existing explicit preconditions.

V22 read-only follow-up: local all-root contacts now publish a common parameter
and point after exact sheet, finite-domain and tangent replay. The point still
does not retain a chord-incidence identity: certifies_monotone_chord_incidence
only consumes Monotone authority, while the new local root has Polynomial
authority. The existing ChordRationalTangent identity remains rational-only.
This is an actual evidence-retention gap in the current representation, but no
new stack sample yet establishes it as the remaining quartic timeout's cause.
A subsequent change must retain regularized tangent-frame preconditions and
keep the equal-normal-offset theorem's rational/zero-distance guard explicit.
Do not introduce a support/point reference cycle or an unlimited list of old
contact reports merely to avoid one repeated predicate.

The ordered-field Bernstein path is intentionally complete only for the roots
its report certifies. It rejects unresolved repeated roots and coefficient
signs to the full projection authority. Existing Hypersolve sign-remainder
and real-root linear quotient primitives are relevant to future deflation,
but a bound or guessed multiplicity must not replace the root certificate.
A fresh bounded sample after V22 qualification/commit should choose the next
implementation target instead of relying on the older projection stack.

The committed V22 bounded stack is terminal and outer handle 55670 is reaped.
It still reaches incidence_projection during Boolean regularization. A separate
physically copied diagnostic snapshot V1 leaves production unchanged and logs
only stage names, counts and status enums. Its 20-second diagnostic child and
outer handle 31611 are reaped. It reports sign-uncertain during local root
isolation, before any finite clipping or contact replay. Thus retained-contact
identity is not the immediate cause of this observed decline. Diagnostic V2
adds bounded coefficient-sign metadata and compares the existing bounded path
with a complete strict local attempt; neither is production qualification.
