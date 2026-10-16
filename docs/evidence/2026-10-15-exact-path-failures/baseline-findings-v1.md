# Reproduced baseline

`reproduction-v1` and `reproduction-v3` execute the retained E13 library graph
whose production sources match revision `958d2fa76`. Their success means the
diagnostics ran and reproduced the current behavior, not that the requested
failure contract was satisfied. `reproduction-v2` preserves a probe compiler
mistake (`CurvePoint2` exposes `coordinates`, not the vector method
`represented_coordinates`) and the resulting missing executable.

Observed before implementation:

- Missing controls, nonclosed controls and zero display count all become a
  successful empty result through `bezier_region`; its checked counterpart
  rejects them as invalid input.
- A coincident two-point Bezier path returns `None` without its native cause.
- B-spline degree zero becomes `None`; the context variant stringifies its
  native invalid-construction error as generic `Geometry`, with no source.
- The cubic endpoint offset `sin(e)^2 + cos(e)^2 - 1` remains undecided in the
  strict/approximate/strict sequence. All three errors are flattened to generic
  `Geometry` with no source. Approximate policy does not complete this fixture.
- The valid cubic with controls `(0,0),(3,0),(0,3),(0,0)` succeeds Certified
  under both policies and has independently derived signed area `27/20`.
- The retraced quadratic `(0,0),(1,0),(0,0)` succeeds as a Certified empty set
  under both policies. Preserve this successful empty outcome separately from
  malformed construction.
- Line, quadratic, cubic and degree-four Bezier paths, and the matching clamped
  single-span polynomial B-splines, retain exact independently evaluated
  midpoint coordinates for display counts 1, 16 and `usize::MAX`. No sampling
  resolution reduction is needed for these successful fixtures.

The new tests preserve this certified corpus and add typed error assertions,
policy reuse, native cause checks, family tags, path reversal, region membership,
exact area and downstream empty/transform composition. This migration changes
the Rust return types and error categories explicitly. It does not claim that
the unresolved cubic now completes, nor that inherited qualification or finite
projection certifies an original source claim.
