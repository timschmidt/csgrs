# Next normalized-corner caller checks

This is read-only planning while the scalar qualification sweep runs. No Hypercurve source has changed.

- The current sweep confirms `algebraic_endpoint_images_reenter_shared_corner_carriers` fails solely at the first `AlgebraicChord` count assertion (`bezier_region.rs:28811`). That assertion describes storage, not the exact line support. Later fillet assertions have not yet been reached. Replace it only with a genuine exact geometry/set oracle; do not merely delete the count or force a narrower native representation.
- `retained_algebraic_straights_trim_or_extend_chamfer_and_fillet` searches only loop zero for (3,0) and (2,-1). The fixture is the rectangle [sqrt(1/2),2] × [0,2]. Both exterior cuts may form a separate normalized lobe touching the rectangle at (2,0). The earlier bounded diagnostic found that lobe on another loop. Search all loops within the same candidate, retain both exact endpoint requirements and the candidate-completeness comparisons, and validate filled-left topology. Useful independent set probes are (1,1) inside and (5,5) outside for all candidates, and (17/8,-1/8) inside the candidate retaining both exterior cuts. Verify that last geometric assertion against the actual fillet branch before adopting it.
- The existing test helper `retained_fragment_has_exact_endpoint` checks only three payload variants and only `.coordinates()`. The production `curve_fragment_endpoint_point` already returns general CurvePoint2 evidence for every fragment family; exact `coincides_with` can test the endpoint without requiring a coordinate payload. Migrate all helper callers directly if replacing it, keep strict/certified success, and do not claim unavailable equality is false evidence.
- Single-fragment source fixtures can produce multiple normalized fragments or loops after editing and self-contact splitting. Their source interval coverage, exact endpoints, inserted geometric carrier, winding, and regularized set are the obligations. Current `assert_one_fragment_edit_shape` and several tests demand one stored interval or loop zero. A replacement must preserve those geometric obligations; a weaker fragment-count bound is insufficient.

The public root-certificate construction issue recorded in `biquadratic-basis-20260923-proof.md` is still separate API debt. No privacy/isolation-authority migration has been implemented in this scalar batch.

## Completed straight-corner migration

Hypercurve `7af48343b93c7f2110e60d26a94c316aad996795` implements the second and third bullets above. The unchanged (1,1) interior probe and the proposed exterior-lobe probe both pass under both policies and traversals for chamfers and fillets. All three test-helper callers pass. Source production is unchanged. The algebraic-endpoint payload assertion, PH interval-coverage assertion, and analytic corner endpoint lookup remain follow-up work; no conclusion about their later unchecked stages has been claimed.

## PH single-carrier interval oracle (read-only follow-up)

For P(t)=(t(1-t)(1-2t)/6,-sqrt(3)t(1-t)/6), the exterior cuts
at t=-1/2,3/2 are (-1/4,sqrt(3)/8),(1/4,sqrt(3)/8).
Regularization splits the t=0/1 self-contact, so neither loop zero nor a
single Materialized payload describes the complete normalized boundary.
A caller migration should find both exact endpoint positions in one candidate
over all normalized loops, require filled-left topology, and retain the original
policy, reversal, radius/setback and candidate obligations.

Independent source-boundary witnesses across all four interval parts are:
P(-1/4)=(-5/64,5sqrt(3)/96), P(1/4)=(1/64,-sqrt(3)/32),
P(3/4)=(-1/64,-sqrt(3)/32), P(5/4)=(5/64,5sqrt(3)/96).
These detect losing the authored middle loop or either exterior tail, without
requiring one stored subcurve. In a candidate retaining the specified cuts,
(0,-1/24) and (0,1/24) should lie inside the lower and upper lobes; (1,1)
is outside. Verify these through certified queries, not a fragment count.

The matching fillet has center (0,17sqrt(3)/48), radius 13sqrt(3)/48,
and its short lower arc reaches y=sqrt(3)/12. Thus both source-tail samples
and the positive interior probe lie below that arc; the chamfer is higher at
y=sqrt(3)/8. Arc-branch variations must be inspected before asserting that
any additional high point is outside. No test or production change has been
made for this plan, and no later PH branch has been claimed as passing.


A further exact PH check excludes hidden circle crossings in that oracle.
Put s=t-1/2. Then x=s*(4*s^2-1)/12 and y=sqrt(3)*(4*s^2-1)/24.
For center (0,17*sqrt(3)/48) and radius 13*sqrt(3)/48,

    |P(t)-center|^2-radius^2 = (s^2-1)^2*(4*s^2+9)/36.

This is strictly positive except at s=+/-1, the two requested extension cuts.
Thus the fillet circle has no additional source contact that could erase any
of the four proposed boundary sample points. The horizontal chamfer has
additional-source-contact equation y=sqrt(3)/8, also exactly s=+/-1.
The lower lobe vertical extent is [-sqrt(3)/24,0]; the upper circle's lowest
point is sqrt(3)/12 and the chamfer is at sqrt(3)/8. These bounds support
(0,-1/24) and (0,1/24) as interior probes in the two distinct lobes, and (1,1)
is outside the x-bounds of both the finite source interval and joining circle.
No production or test source was edited while scalar qualification ran.

V1 implementation is now frozen in normalized-ph-corner-20260924-v1-sources.json.
Only the PH test function changes; a prefix/suffix comparison with committed
Hypercurve HEAD proves all production and other test bytes unchanged. The
assertion now finds both exact cuts across all loops of the same candidate,
requires normalized topology for every candidate, and checks four independent
source boundary samples plus both lobe interiors and an exterior point.
Both policy modes, traversal directions, radii/setbacks, and operation modes
are unchanged. An independent Python Fraction polynomial expansion confirms
the circle/source factorization in the test comment. Both Clippy configurations
and five focused PH tests are pending; no success is inferred yet.
