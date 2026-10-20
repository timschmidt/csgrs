# atan2 on the positive half-plane

Baseline `7042b1ef2`, branch `hyperspatial`; Hyperreal owns the quadrant decision.
P2–P7 and P18–P20 apply. The new complex atanh test reproduces a panic at
`atan2` for x=1-(1/4)^2-y^2 and y=2*(sin(e)^2+cos(e)^2-1), although x can be
proved positive and atan(y/x) is analytic there. The checked scalar API also
returns Exhausted for opaque y with a certified positive x. That sign is not
needed to select the branch. Add the scalar regression before changing code.

Resolve x first after existing exact-axis shortcuts. For certified x>0,
return the existing exact atan(y/x) expression without classifying y. Keep
all negative-axis/origin decisions and their existing sign budget; unknown
y with x<=0 must remain Exhausted. Preserve every previously successful
serialized result. Make the infallible wrapper call this checked owner;
it retains its documented panic on unresolved quadrants. Complex atanh must
use the checked API and propagate typed uncertainty.

Test opaque zero and positive/negative tiny differences, structurally positive
and refinement-proven positive x, unresolved left/origin cases, exact axes,
previous quadrants, serialization, low/high/low refinement, cancellation and
concurrent readers. Retain the previous independent scalar oracle suite and
measure both complete atan2 functions in the later production coverage gate.
No new scalar node, precision budget, native work/heap guarantee or signed-zero
cut convention is introduced. This extends certified completion, not a speed
comparison against the old Exhausted result.

The unknown-x/known-nonzero-y case also reproduces Exhausted in the Real
wrapper. Reuse `Computable::try_atan2_until` and its existing checked half-angle
path for this case; do not duplicate the formula in Hyperlattice. The public
complex regression includes a trigonometric unit-circle argument whose zero
norm residual has no cheap certificate.

The cancellation regression isolates a separate unary construction defect:
input and divided-input certified intervals correctly reject an active abort,
but `atan` and `try_atan2` return intervals after the deferred atan node drops
the input's root signal. `Computable::atan_deferred` owns this propagation.
Retain the same signal at its new root as well as in the retained input graph;
verify cold/warm abort, recovery and cache integrity. This does not claim that
arbitrary distinct descendant cancellation signals are a combined controller.
The exact source and observed stage diagnostics are retained in the probe log.

The first small-offset test incorrectly expected 1e-320 differences to remain
unresolved: the existing public sign floor is -2048 bits and can certify those
signs. Keep them as certified upper/lower-left successes; supplement with
1e-1000 differences and assert unresolved sign before testing exhaustion.
Neither the operation's floor nor its successful input domain is reduced.

Focused coverage initially missed axes whose nonzero coordinate needs
refinement. Add cold inputs sin(e)^2+cos(e)^2-1 plus or minus 1/2, assert that
their structural sign is unknown, and require exact signed pi/2 (x=0) and pi
(y=0, x<0). This tests the post-refinement axis contract without changing the
implementation or excluding lines from the complete-function denominator.

Before integration, the positive-x regression and an isolated active-abort
atan probe reproduce their failures on `7042b1ef2`. The latter first confirms
that the input rejects refinement, then shows its atan output incorrectly
permits it. The full cancellation test also fails earlier at the old atan2
constructor; both diagnostics are retained rather than treating that earlier
failure as proof of the distinct signal defect. The final admitted regression
suite remains unchanged. Complete stable/MSRV, native/downstream, cache,
coverage and sanitizer checks pass; the
[verification record](../../hyperspatial/verification/2026-10-19-e2-complex-atanh/README.md)
retains all failures and commands. Complete-function line coverage is 42/43
for the changed Real atan2 functions and 13/13 for deferred atan. All 1,936
before/after serialized rows remain byte-identical. These results do not
establish native total-work or heap bounds.
