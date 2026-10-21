# Future work

## Convex hulls of sampled spheres

`solid::convex_hull` of a csgrs `solid::sphere` stops with `ConvexHullPredicate { stage: "face outside linear query" }` for most sample counts.

**Why.** The four corners of each quad between two latitude rings form an isosceles trapezoid, which is exactly planar. The hull's `orient3d` against such a face is therefore an exact zero. With trigonometric coordinates that zero has no structural proof, and refinement cannot certify it.

**What already decides.** Exact sine and cosine are available at multiples of π/2, π/3, π/4, π/6 and π/12. The last adds the 15-degree values `(sqrt(6) ∓ sqrt(2))/4`. Spheres whose segment and stack counts divide those turns have coordinates in `Q(sqrt(2), sqrt(3))`. Their coplanarity zeros are multiquadratic identities, and the quadratic tower decides them: `sphere(8, 24, 12)` gives a certified hull with 528 triangles.

**What does not.** Other sample counts still produce opaque `SinPi` computables or nested radicals. Examples are the 32 × 16 sphere used by csgrs's `kernel_comparison` bench, and any count with an angle such as π/16 or π/5. Their hulls stay undecidable.

**Options:**
- **Hull from mesh structure.** The quads come from the sampled grid, so the generator knows which four vertices are coplanar. Carrying that combinatorial certificate into the hull's `memberships` would decide the zeros without arithmetic.
- **Larger exact trig tables.** Add closed forms where they are biquadratic or one nested radical deep, such as π/8 and π/5. These would only help where the quadratic tower already covers the field.
- **Hull of an exact-rational sphere.** Rational points on the sphere (for example from rational stereographic parameters) make every predicate rational, but they change the sampled geometry.

**Priority.** This is the highest-value fix in the stack (review of 2026-10-20). The same undecidable coplanar quads stop strict Booleans with the sampled sphere, not only hulls. The repository's own `boolean_operations` and `convex_hull` examples fail under `STRICT` for that reason, and `scripts/run-examples.sh` keeps them as tracked known failures. Of the options above, the generator-side coplanarity certificate keeps the sampled geometry. Rational stereographic sampling changes it, which would need an explicit migration. Acceptance: both examples pass under strict policy in CI, are removed from the known-failure list, and keep their exact set semantics.

## Successive operations

Found by `tests/successive_operations.rs` (repository root); measurements in
[`docs/successive-operations.md`](../../docs/successive-operations.md).

### Corefinement vertices accumulate

Each Boolean triangulates every source triangle separately and keeps every
arrangement vertex. A later cutter crosses the long fan edges left on earlier
coplanar faces, so the vertices where it meets them stay in the output. They
are not intersections of three source planes and the exact set does not need
them. Output size then grows faster than the set's complexity:

- 16 square (4-gon) drills: 1,600 triangles, against 12 + 20k = 332 needed.
- 3-4-5 rotated cube cuts: per-step increments 30, 50, 70, …, 150, so the
  triangle count grows quadratically.
- Two crossing bars with integer inputs give 86 vertices, 52 of them Steiner
  points such as (23/3, 4, 8) in the coincident faces at y = 4.

Repeating a cut that leaves the set unchanged is a fixed point, so the growth
comes from new cuts crossing old fan edges. **Options:** corefine coplanar
regions of one input plane as polygons, not per triangle; or, after the
Boolean, merge facets that share a source plane and remove vertices whose
incident facets lie in one plane, or in two planes along their crease, which
needs no arithmetic. Either changes output rows, so it is a serialization
migration under P27.

### Derived coordinates deepen with each generation

The extra vertices above are interpolated from earlier derived vertices.
For symbolic inputs their expression graphs deepen at every step. For 16-gon
drills the maximum depth goes 9, 19, 30; for 6-gon drills it reaches 58
after eight. Exact zeros among such values eventually exceed the strict
normal-form budgets:

- 16-gon drills: undecided at drill 4, 32-gon drills at drill 2, both
  `surface arrangement point on segment`. `corefine_face` tests every
  projected point against each constraint segment. Points constructed on one
  drill-wall plane and the face plane are collinear by construction. Their
  construction identities show this, but the test needs a numeric zero proof.
  `hypertri`'s constraint validation repeats it.
- Cubes rotated by 10 degrees: undecided at the second cut
  (`segment_intersection`). `sin(pi/18)` is cubic, so no radical tower decides
  the zeros.

Without the growth above, most of these points would not exist. Two further
fixes would apply anyway. Incidence can be certified from construction
identity before numeric tests run. A point can be represented by its defining
planes, so that its coordinates do not depend on previous outputs.

### Kept: embedded-complex evidence and exact axis-plane coordinates

Boolean outputs now retain that their triangles meet only in shared vertices
or edges. A later arrangement then records radial separation for shared
reversed edges from identity and skips the remaining self tests on that
operand. On ten corpus chains this published identical rows to the full self
test. Crossings on an axis-aligned rational plane now take the plane's exact
coordinate instead of an interpolated expression. Before that, a revolved ring
cut twice in the same plane was undecided at the second cut.
