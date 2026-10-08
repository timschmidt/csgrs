# One finite-chord point containment predicate

Parent Hypercurve: 3bf8ae4ec3587c6953ffc542d4d20755c8c1bd02.

BezierAlgebraicChord2 previously exposed a Point2 contains_point and a
CurvePoint2 contains_point_evidence for the same finite-chord question.
The represented path owned exact-line, local-box, certified-tangent and
native algebraic accelerators; the general path used a retained support-side
predicate followed by monotone parameter clipping. The native path also
returned from exact_line before checking the chord's retained policy.

The candidate keeps one contains_point taking CurvePoint2. It validates the
retained policy before dispatch, preserves the represented specializations,
and lets their uncertainty fall through to the shared support-side and finite
parameter authority. Optional exact-line, tangent and native algebraic sign
predicates run with the approximate terminal suppressed while retaining object
replay authority. The complete support predicate still observes the caller's
policy. No representation, cache, alias or forwarding overload is added.
All callers in region classification, fillet validation, curve evaluation,
support intersection and tests are migrated directly.

Independent endpoint fields select A=(sqrt(1/2),0), B=(0,sqrt(1/3)). The
extended regression queries retained endpoints, independently represented
endpoints, their midpoint, an off-line point inside their coordinate box,
and the collinear extrapolation 2A-B. Both traversal directions and policies
must produce certified finite-membership decisions. The existing policy-query
regression additionally checks a horizontal chord against an opaque exact zero:
strict remains uncertain, the approximate terminal is explicitly observed, and
a later forced-strict query remains uncertain. Repetition checks that the
terminal result does not become cached exact truth.

V1 stopped at two missing Real clones in the new sqrt test oracles. V2 stopped
at Clippy's rejection of an unrelated bounding-box test's redundant conversion.
Each driver was terminal and reaped, with source hashes verified, before editing
the next version. V3 removes that conversion; production is otherwise unchanged
from V1. Each physical snapshot binds 2,044 inputs and six changed source files.
V3 formatting, both all-target Clippy configurations and all ten focused
regressions pass. The complete library qualification and nine public targets
are running. The public targets extend the previous region/arrangement/path
coverage with curve, intersection, general point and parameter-range tests,
because curve evaluation and support-intersection callers are migrated here.
No failing version is claimed as qualified.

Final V3 qualification: 1,465 cases attempted, 1,451 passed, six ignored,
and eight unchanged 75-second timeouts. All 225 public integration cases pass.
Formatting and both all-target Clippy configurations pass. All 2,044 frozen
inputs, copied executables, index and HEAD bindings are verified; every owned
process is reaped. Committed as Hypercurve 1cf070a5d1ec2ebf2796177da784a2d5a0494ee2.
All thirty repositories are clean after commit; nothing was pushed. The full
implementation goal remains active. Adjacent V3 qualification, staged and
post-commit JSON receipts record these bindings.
