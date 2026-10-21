# Future work

## Convex hulls of sampled spheres

Strict hulls and Booleans of csgrs `solid::sphere` meshes used to stop on exact zeros that no recognizer could prove. The README examples `boolean_operations` and `convex_hull` failed under `STRICT`. Since 2026-10-20 they complete, and `scripts/run-examples.sh` no longer lists them as known failures.

**Why they failed.** Reproduction on 2026-10-20 found a different cause at each call site. The quad zero itself was not one of them.

- The `orient3d` zero of the four corners of a latitude quad already decided. The quad is an isosceles trapezoid, and its planarity is a polynomial identity in independent sine and cosine atoms, which the ring recognizer proves.
- Booleans stopped at `surface arrangement retained line orientation`. A cube face crosses both triangles of one quad, and the three crossing points are collinear. The zero is a rational function of the atoms. Its degree and term count exceed the ring recognizer's limits (degree 8, 64 terms).
- Hulls stopped at `face outside linear query`. The 32 × 16 south pole and vertices (longitude, ring) = (5, 14), (13, 8) and (11, 8) are coplanar, but they are not a quad. The reason is `cot(pi/16) = tan(7 pi/16)`: stereographic projection from the pole maps the four points onto one line. This identity relates trigonometric values at different angles, and no identity in independent atoms implies it. A generator-side coplanarity certificate for quads would therefore not have completed the hull.

**What decides now.** `hyperreal`'s exact rational normal form gained a cyclotomic zero proof, tried right after the ring attempt. Rationals, sines and cosines of rational multiples of pi, `sqrt(2)`, `sqrt(3)` and `sqrt(6)` lie in one cyclotomic field. The proof reduces the expression modulo primes at every conjugate of the root of unity, and a norm bound turns zero residues into an exact proof. It applies after any rational affine transform and to the outputs of later operations, because it reads the values, not how the mesh was generated. Examples: the 32 × 16 sphere lies in Q(zeta_32) of degree 16. The 17 × 9 sphere lies in Q(zeta_612) of degree 192. The `convex_hull` example's cube is rotated by 35 degrees, which with the 32 × 16 sphere needs Q(zeta_288). `tests/sampled_sphere_strict.rs` checks strict Booleans of the README cube with the 32 × 16 and 17 × 9 spheres, strict hulls of both, and a strict difference, union and hull in sequence.

**What remains.**
- **Cost.** These workloads complete, but slowly. Release builds take about 9 s for the four Booleans of `boolean_operations` and about 7 s for `convex_hull`; debug builds take about 70 s and 55 s. A strict hull of the 32 × 16 sphere alone takes 17 s in release. The proof itself is a small part of that: 5.5% of retired instructions in `boolean_operations`. Structural sign facts (`exact_quadratic_surd_parts` through the Laurent normal form) take about 38%, and bounded refinement about 29%. A generator-side certificate could still save work, but it is no longer needed for completion.
- **Hull row stability.** A second strict hull of the same 32 × 16 sphere mesh in one process can return different rows and a different triangulation of its coplanar faces, though the hull is the same set. This happens when the first hull starts from coordinates with no cached approximation. With the coordinates' binary64 projections computed first, which leaves those projections unchanged, three successive hulls agreed. Fresh processes agree. The 24 × 12 and 17 × 9 hulls and the Booleans gave identical rows. The earlier sampled-sphere trial traced this to eye selection: farthest-point ties between cospherical points are broken by the order in which points are found. P27 asks for deterministic rows. Breaking ties by point index would fix this, but it changes the certified rows of existing degenerate hulls, so it needs a recorded migration.
- **Domain limits.** The proof only shows that a value is zero. A nonzero value below the refinement limit stays undecided. Other square roots, field orders above 16384 or degrees above 1024, graphs above 4096 distinct nodes, and more than 2^25 work charges also stay undecided. These limits are typed declines, not answers.
- **Rational stereographic sampling** would make every predicate rational. It changes the sampled geometry, so it would need an explicit migration.

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
after eight. Exact zeros among such values exceeded the strict normal-form
budgets: 16-gon drills were undecided at drill 4 and 32-gon drills at drill 2
(`surface arrangement point on segment`), and cubes rotated by 10 degrees at
the second cut (`segment_intersection`; `sin(pi/18)` is cubic, so no radical
tower decides it). `hyperreal`'s cyclotomic zero proof now decides all of
them; the straight and 10-degree-tilted 16- and 32-gon chains complete four
cuts certified. The cost still rises steeply: the tilted 32-gon chain takes
5.0, 14.8, 66.4 and 158.7 s for cuts 1 to 4 in a release build. Points
constructed on one drill-wall plane and the face plane are collinear by
construction, but `corefine_face` and `hypertri`'s constraint validation
still prove it numerically.

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
