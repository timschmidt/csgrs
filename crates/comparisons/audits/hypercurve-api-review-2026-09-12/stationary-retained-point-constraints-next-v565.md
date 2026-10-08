# Retained point constraints on stationary fillet families (unrun V565)

V565 is an independent copied-source fixture, not a confirmed failure. It
requires V563 to be fully qualified, committed and reaped before running.
Production remains frozen under V563. V564 source-factor reuse is prepared but
unrun; a demonstrated completeness failure takes priority over that optimization.

The existing stationary quartic family has independently known rational center
and contacts. Construct each same point as the selected algebraic image of
Q(s) = point + (2s² - 1)(1,2), at the positive root of 2s² - 1. Public point
construction retains the image without stored coordinates; first verify exact
coincidence with the independently known point. Then supply that retained image
as center, first contact or second contact, for both traversal directions and
both policies. Every case must select the same exact fillet. The fixture
collects bounded errors; it never prints recursive Real/geometry values.

Source inspection (hypothesis, pending execution):
- FilletComponentReplay2::point_parameters explicitly uses
  point_incidence_in_regular_domain for stored Cartesian points. Other points
  go through fillet_point_parameters and visit_point_incidence_evidence.
- The algebraic-point visitor constructs two raw parallel endpoint witnesses,
  then BezierParallelAlgebraicRay2. A stationary source endpoint can prevent
  those raw witnesses even though the requested contact is in the regular
  interior and the component already owns a regular source cell.
- The ray constructor certifies the raw source frame (None override). Its point
  membership machinery uses only the source, selected domain and two coordinate
  incidence systems; its endpoint geometry is needed for winding, not membership.

Potential shared simplification, only after the counterexample is measured:
separate retained algebraic point incidence from the winding-ray carrier. Reuse
its exact coordinate systems and selected-root replay with a borrowed source,
actual domain and certified normal field. Then point constraints can use the
component's regular-domain authority for every point representation, removing
the Cartesian-vs-retained dispatch in the fillet selector. Update winding-ray
callers to the shared system builder rather than adding forwarding aliases.
Do not grant a whole ray or a multi-cell domain one normal merely from a finite
midpoint; preserve source-factor sign, owned cusp limits, empty-ray behavior,
and the original/expanded incident domain distinction. General visitors still
need their original pointwise semantics on domains spanning source singularities.

Measured V565 (outer83712 reaped1): build134.664s; case15.435s. All
independent point/coincidence assertions pass.6/12filletcases failBoundary:
forwardcenter,forwardcontact0,reversedcontact1 underbothpolicies. The other6
select the expected fillet. Production9baa94d8 remainsclean beforethefix.

Planned implementation: move system/projection/sign and point-visit methods
from BezierParallelAlgebraicRay2 into a small private borrowed algebraic query
(parallel,range,optionalnormalframe). Winding retains its endpoint/sweep data
and calls that shared query. Membership constructs no endpoint witnesses.
The existing incidence visitor gets an explicit regular-cell mode for the
component selector; raw general callers retain pointwise/raw semantics.
Remove point_incidence_in_regular_domain's Cartesian-only helper and update
its sole caller to the shared visitor. Propagate the regular mode through
endpoint/analytic/similarity evidence resolution and correlated chord-point
incidence, certifying the actual finite/incident normal field with existing
source_tangent_field_in_regular_domain. Keep one-sided normals and ray
barriers; no new public curve/point carrier or compatibility interface.
