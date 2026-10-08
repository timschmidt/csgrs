# General curve/path split topology

Base: Hypercurve `0facb05f1c0949c475a98be078dcbc389b522cfd`.
The original implementation goal remains active. Qualification and the incremental
commit are recorded separately in `general-split-topology-qualification.json`.

## API and representation

`CurveIntersectionTopology2::{first, second}` now borrow traversal-ordered
`Curve2` pieces. `CurvePathSplit2::curves` exposes the same pieces grouped by
authored curve. General topology no longer requires a native Bézier definition
or promotion of a selected cut to `BezierParameter2`. The removed
`materializations` and general `arrangement_graph_view` interfaces have no
compatibility wrappers; callers are migrated directly.

`arrangement_graph` borrows the graph shared with topology clones. Preparation
occurs within the topology operation's certainty tracking: lowering a retained
source range can make predicates, so doing it in an untracked lazy accessor
would lose evidence of any consumed terminal decision. The existing retained
fragment arrangement engine remains the only graph implementation. Graph source
indices now identify authored curves, with fragment indices in traversal order.
Path indices enumerate the first path followed by the second.

Batch subdivision uses the existing general parameter comparison and restriction
authorities. It removes duplicate and endpoint cuts, preserves reversed retained
source traversal, and flattens repeated restrictions through the existing shared
authored source. Intersection span charts supply the same affine mapping used by
public locations, including non-unit and descending spline charts.

## Semantic and evidence checks

Three new public tests plus two expanded regressions exercise 256 additional
curve/path topology queries, 96 piece-reentry intersections, and eight setup
intersections under both policies. Cases include:

- Generated parabola chamfer chords cut by an independent vertical line, both
  traversals and operand orders. Independent radical coordinates verify the
  internal endpoint; every returned piece reenters intersection.
- Selected upper tails of a quadratic, elevated rational curve, polynomial
  B-spline and NURBS, then a second intersection. Reversed retained traversal
  and spline source domains `[2,5]` must retain evaluable exact endpoints.
- Polynomial and weighted rational splines with a discontinuous internal knot.
  Two contacts share one source parameter but have different points. One split
  preserves both one-sided limits rather than fabricating a connecting edge;
  each piece reenters intersection independently. Graph provenance is checked.
- Full and selected overlaps between generated chords and independently authored
  radical lines. Both traversals and operand orders preserve exact overlap cuts.
- Collinear endpoint-only contacts after composed selected splits. No empty
  piece is emitted, and each original finite carrier remains whole.

Existing assertions about native materialization counts and storage variants are
replaced by exact endpoint checks and counts of actual cut pieces. Region Boolean
semantics and independent point expectations remain checked.

## Machinery and remaining limits

The native curve/region trim operation still needs span-local representative
points. Its remaining split preparation now consumes the native spans and Bézier
parameters it already prepared, avoiding repeated preparation, promotion and an
aggregate materialization vector. It does not retain a forwarding helper for the
removed general splitter. General trim closure is still unfinished.

The native `RationalBezierIntersectionTopology2` is a distinct lower-level API,
not a compatibility carrier for the removed general interfaces. Its existing
graph accessors are unchanged in this migration.

This migration does not establish complete selected-circle/parallel pair
dispatch, finite exterior-domain replay, retraced or partially coincident
non-injective component relations, or the broader original architecture goal.
It also does not seed arrangement topology vertices directly from every contact
identity; downstream graph predicates still consume existing retained evidence.

There is no measured throughput, allocation, expression-depth or memory-size
claim. Eager graph preparation trades demand-driven graph assembly for correctly
tracked operation certainty; source and topology clones share their authority,
but further scheduling and proof-reuse work remains warranted.
