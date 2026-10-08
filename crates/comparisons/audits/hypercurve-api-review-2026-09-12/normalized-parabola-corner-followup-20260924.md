# Normalized parabola corner oracle

The existing retained_analytic_multifragment_corner_extends_chamfer_and_fillet
asserts an exterior construction cut remains on a boundary fragment. This is
not a semantic invariant after regularization, even after replacing loop zero
with all loops or broadening the accepted carrier representation.

The source is P(t)=(t,t^2), corner V=(1,1), and outgoing line slope
m=83549/38280 for the exact-cut cases. Its second intersection with the
parabola is B=P(m-1), with 1<m-1<6/5. The desired source cut
C=P(6/5)=(6/5,36/25) lies above the outgoing line, inside the old material.
The zero-next-setback chamfer from C back to V also lies above that line.
Consequently the extra positive-winding lobe overlaps the old material:
regularization may consume C and the chamfer, retaining source only to B.
Requiring C to remain a boundary endpoint is wrong for that result.

Independent points for the extended chamfer candidate: (11/10,121/100) is
on the newly exposed parabola, while (11/10,243/200) lies strictly between
that parabola and the old outgoing line (new material). C should be inside,
not boundary. Preserve the existing candidate counts, selected/native forms,
both policies and reversal. Require normalized topology for every candidate;
find the extended candidate by these geometric facts across the exact set.
Check old interior (-1,0), far exterior (10,10), and B's boundary ownership.

For the exact fillet radius 299/125, the circle tangent to the source at C
on its left has center (-126/125,59/25). Its distance to the outgoing line
is exactly 299/125 (the authored direction is the rational unit vector
(38280/91901,83549/91901)). C is again inside the old material. The oriented
C-to-forward-line contact can traverse the major circular arc; it may add
material beyond the old x=-2 wall while the cut itself is consumed. The
leftmost circle point (-17/5,59/25) and interior point (-3,2) distinguish that
exact circle extension. This branch geometry must be checked against the
actual result before replacing assertions; do not assume every candidate
uses this circle/arc or that an arc survives every regularization.

Independent Python Fraction arithmetic additionally verifies
|P(t)-center|^2-r^2=(t-6/5)^2*(t^2+(12/5)t+3/5), with coefficients
[108/125,252/125,-93/25,0,1]. The remaining circle/source contacts
t=(-6+sqrt(21))/5 and (-6-sqrt(21))/5 are both negative, outside the
retained source range [0,6/5]. The same calculation verifies the rational
unit line direction and exact circle-to-line tangent distance. The circular
major arc can still intersect other old boundaries, which regularization
must resolve; this factorization alone does not certify the whole filled set.

For the algebraic fillet case, the line slope is 12/5 and the second
source/line crossing is B=P(7/5)=(7/5,49/25). A left-side radius-1/2 circle
with an exterior source contact can lie entirely inside the old material.
Its source contact is beyond 7/5 because the signed contact equation is
(t-1)(t-7/5) + (1/2)*((1+(24/5)t)/sqrt(1+4t^2)-13/5)=0.
The second term is nonpositive by Cauchy-Schwarz; thus an exterior root
cannot lie in (1,7/5). The selected cut can be consumed and the normalized
boundary need not retain any algebraic endpoint. Candidate oracles include
P(13/10) on the newly exposed source and (13/10,341/200) in new material,
plus B, the old interior, and far exterior. Circle containment and branch
orientation still need explicit verification before making this a regression.

Further independent bounds complete the algebraic circle-containment argument.
On t in [7/5,141/100], put A=13/10-(t-1)(t-7/5),
B=(12/5)t+1/2, S=1+4t^2. Both A and B are positive; the contact
equation has the sign of B^2-A^2*S. Exact Fraction calculations give a
negative sign at 7/5 and a positive sign at 141/100. Its derivative is
2t-12/5+(12/5-2t)/S^(3/2), bounded below by
2/5-(21/50)/8>0, so this bracket contains one exterior contact.

The circle center is (t-t/sqrt(S), t^2+1/(2sqrt(S))). Since 2<sqrt(S)<3,
its disk lies at x>1/5 and x<(2/3)*(141/100)+1/2<23/13; its y maximum
is less than (141/100)^2+3/4<37/13, below the old top edge. Its y minimum
exceeds (7/5)^2-1/2>1, above the old parabola for x<=1. For x>=1 the
exact tangent-line equation puts the entire disk on the old material side
of the outgoing line. Thus the circle is contained in the old region. Its
CCW source-to-line tangent continuation is the major arc, but regularization
consumes it; the new visible boundary transfers from P(t) to the line at
the rational second crossing t=7/5. The source cut itself is strictly inside.

For the larger exact circle the CCW continuation is also the major arc:
the source unit tangent at 6/5 is (5,12)/13, and the forward line tangent
has slightly smaller angle. The major arc includes the stated leftmost
circle point outside the old x=-2 wall. Its independent point (-3,2)
lies strictly inside that added disk. Both cases retain the small exposed
parabola lobe below the old outgoing line.

## Implementation qualification

After committing selected-circle incidence as Hypercurve 91ddad8, V1 replaces
the old test with retained_analytic_corners_preserve_normalized_sets and removes
its two obsolete analytic-fragment inspection helpers. Every candidate must
have normalized filled-left topology. Independent point queries must decide
with certified certainty, and one candidate must satisfy every expected point
location for each construction. Both policies, traversal directions and
selected/native source forms remain covered.

The physical normalized-parabola-corners-20260924-v1 snapshot binds 2,044
inputs. A source comparison proves that only this test and its two helpers
changed; all production and unrelated tests match 91ddad8. Formatting and both
all-target Clippy configurations pass. All five focused corner regressions pass. The new normalized-set test takes
9.68 library seconds. Qualification verifies the source, executable, index and
HEAD bindings; the previous full production qualification remains applicable. These are independent
geometric obligations, not permission to weaken failed tests.

Committed as Hypercurve 3bf8ae4ec3587c6953ffc542d4d20755c8c1bd02. All thirty repositories are clean after commit; nothing was pushed. Eight existing bounded-workload timeouts remain. The full implementation goal is active.
