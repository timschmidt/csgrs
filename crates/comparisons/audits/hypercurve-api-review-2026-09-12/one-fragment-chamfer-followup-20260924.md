# Closed-cubic chamfer computational follow-up

The materialized and selected one-fragment fixtures use the same cubic with
controls (0,0),(4,0),(0,4),(0,0), and both Euclidean chord setbacks are 1/2.
Their trim-only tests exceed the unchanged 75-second limit. Existing logs
contain only the test-start line, so no algorithmic cause is established yet.
The intended next diagnostic is a bounded stack sample of the unchanged
qualified release executable, after the represented-circle qualification and
commit. No timing from the debugger will count as a qualification result.

Independent fixture algebra, useful for checking any proposed fix:

* C(t)=(12*t*(1-t)^2, 12*t^2*(1-t)), with t in [0,1].
* Setting u=t*(1-t), squared distance from the seam is 144*u^2*(1-2*u).
  A setback of 1/2 therefore solves 576*u^2*(1-2*u)=1.
* On 0<u<1/4, the left side is strictly increasing: its derivative is
  1152*u*(1-3*u)>0. It starts below 1 and ends above 1, so there is one u.
  The two finite cuts are t=alpha and t=1-alpha, with 0<alpha<1/2.
* C(t).x+C(t).y=12*t*(1-t). The cut chord is x+y=12*u. Its source-incidence
  equation is quadratic and has exactly the two selected cuts. Any generic
  replay must preserve those already constructed endpoint incidences.
* det(C'(t),C''(t))=288*(1-3*t+3*t^2)>0; the source has no stationary point
  on [0,1]. This is a strictly convex closed boundary with an authored corner
  at its seam. The trimmed result is the middle source interval plus its chord.

These facts do not establish where the present implementation spends time,
and they are not a proposal to add a special-case formula for this fixture.
The stack sample should first distinguish root isolation, endpoint incidence,
normalization, repeated evidence reconstruction and diagnostic formatting.
The existing fragment-count expectations in this simple trim-only fixture
are consistent with its geometry; they must not be weakened to hide the timeout.

The current source admits and prepares the same one-fragment carrier twice,
once for each corner side. Whether that causes material duplicate work needs
evidence. Likewise, conjugate selected parameters must retain their true
relations without treating independently allocated source axes as independent
mathematical variables. Fix the shared evidence/scheduling authority identified
by the diagnostic, and preserve the general exact fallback.

The first debugger attempt was denied ptrace before its inferior ran. Its
log and separate unavailable receipt are preserved. The separately approved
stack2 probe traced only its own child, interrupted at 20 seconds, printed
bounded frames without arguments or values, and killed/reaped the child and
debugger. All 2,044 source bindings and the committed executable hash match.

At the sample, the expensive stack is BigInt multiplication inside Hypersolve
quotient-ring resultant elimination and tagged tensor-fiber norm projection.
The caller is BezierRecursivePolynomialParameterAuthority2::promoted_parameter,
reached from polynomial_sign_at_parameter and the retained analytic point
coordinate comparison. The enclosing operation is finite chord clipping in
recursive_projective_rational_intersections during region normalization.
This is a sampled location, not a complete allocation/time profile. It does
rule out diagnostic formatting as the work occurring at the sample.

The scalar recursive parameter already supports a selected-generator identity:
a native parameter present in the coefficient field, lying strictly within
the certified singleton bracket and satisfying the defining polynomial, is
that selected root. The coordinate predicate previously proceeded directly
to a new polynomial sign query even when both rational coordinate functions
were identically the same.

The frozen retained-coordinate-identity V1 candidate adds one guarded reuse:
after certifying both denominators nonzero, cross-multiply the two univariate
coordinate charts, requiring every coefficient to be structurally zero. Only
a bounded certified Equal parameter comparison then proves coordinate Equal.
Every other case retains the existing sign and projection fallback. The older
point is taken from the existing imported Algebraic source, so endpoint views
need no new representation or normalization interface. All tests are unchanged.
The first focused cases are the previously timed-out materialized and selected
trim-only chamfers, with their original 75-second limits. Qualification is
in progress; no timeout improvement is yet claimed.

The V1 focused candidate did not resolve the first trim-only timeout at the
unchanged 75-second limit. Formatting and both all-target Clippy configurations
passed, but this is not a qualified production change. Its physical snapshot,
binary, logs and terminal source bindings remain intact.

A separate V2 diagnostic kept the V1 production candidate and added bounded
test-only carrier labels and polynomial degrees. It was stopped and reaped at
20 seconds, solely as a diagnostic. Both observed coordinate comparisons
imported an algebraic point and retained degree-three coordinate/defining
polynomials, but the native Bezier parameter wrapper was absent. The import
already requires retained coordinate polynomials and an exact solver root in
the coefficient field. Requiring the optional native wrapper prevented the
proposed reuse despite sufficient underlying evidence.

V3 removes all diagnostic instrumentation and generalizes the existing
singleton-root identity certificate to AlgebraicRootRepresentation. The native
parameter comparison now calls that same certificate, and the target embedding
primitive accepts solver root evidence directly, with all callers migrated.
After the coordinate-chart and denominator checks, the coordinate predicate
uses the older point's retained solver root. The certificate requires an
existing coefficient-field source, strict containment in the singleton bracket,
and a zero defining residual. A declined certificate proves no inequality and
preserves the existing complete fallback. All original tests are unchanged.
V3 qualification is in progress; no resolved timeout is claimed yet.

V3 passed the unchanged materialized trim-only regression (0.16 seconds reported
by libtest, 0.475 seconds including the driver and input verification), but the
selected-carrier regression still exceeded 75 seconds. Its bounded stack3
sample again found global promotion from the local polynomial sign query during
finite chord clipping. No original test assertion or deadline was changed.
V3 remains an unqualified intermediate candidate, with immutable receipts.

V4 moves the reuse into the local polynomial-sign authority. After short
interval separation attempts, it asks whether any already retained coefficient
root is the selected singleton. The shared certificate now returns the proved
field embedding, so a query can evaluate in that field without a second rebuild.
The native parameter comparison consumes the same certificate. Only decided
strict signs are accepted from this bounded reuse; all original native-field,
refinement and complete projection fallbacks remain. The point-coordinate
shortcut is removed entirely. This placement also handles recursive point
wrappers without recovering an optional native Bezier parameter. V4 focused
qualification is in progress, with all original test bytes unchanged.

The selected-root reuse is committed as 1210fbd67a43d910652d9a8cf6d5a8ac4127484f. Final V5 corrects only the
materialized extension test's source-range/traversal endpoint pairing. All four
materialized/selected trim and extension regressions now pass, with both policies
and both traversal directions. The final proof document records combined full
and focused qualification; four unrelated/pre-existing timeouts remain.
