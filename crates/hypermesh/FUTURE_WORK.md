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

### Kept: results keep only the vertices their set needs

Each Boolean used to triangulate every source triangle separately and keep
every arrangement vertex, so a later cutter crossed the fan edges left on
earlier coplanar faces and those crossings stayed. Sixteen square drills left
1,600 triangles where 12 + 20k = 332 suffice, 3-4-5 rotated cuts grew
quadratically, and two crossing bars on integer inputs left 54 Steiner
vertices such as (23/3, 4, 8).

Since 2026-10-21 `surface_reduction` reduces every certified selection after
output certification. Facets join plane classes only through facts the
arrangement already holds: the source face itself, pairwise classifications
that found two faces coplanar, and plane classes retained on an unedited
earlier result. Pairwise classifications with non-parallel supports, or
retained facts, mark two classes as distinct planes. No plane predicate is
added. A vertex whose facets form one manifold fan is removed when they all
lie in one class, or when they lie in two classes that change at exactly two
edges known to be collinear. The embedded-complex fact then forces each side
to sweep a half-turn, so the vertex lies strictly between its two crease
neighbours. Hypertri retriangulates each side under the operation policy. A
first pass keys on source faces and restores every source triangle the
result keeps whole, so a cut that leaves the set unchanged reproduces its
input rows; a second pass keys on plane classes. The module documentation
gives the argument in full.

Certified results also retain each triangle's plane class, the distinct-plane
facts between classes, and the exact support plane of the source face it was
cut from. A reduced triangle spans earlier derived vertices, so the plane
through its own vertices, rebuilt by the next Boolean, deepened with every
generation: a 10-degree-tilted 32-gon drill chain became undecided at its
fourth cut. With the retained plane, every hole corner is built on the block's
own face plane and that chain keeps coordinate depth 15.

The represented set, closure, orientation, certainty and embedded-complex
fact are unchanged, and the remaining vertices keep their exact values.
Square and 4-gon drill chains now have exactly 12 + 20k triangles, crossing
bars keep only integer corners, rotated cuts add 12 triangles per cut, and
sampled-polygon drill chains keep coordinate depth 4 or 5 instead of reaching
58. Over the twenty harness chains measured before and after, allocation fell
from 113.8 GB to 33.7 GB. Changed rows are a P27 migration, recorded in
[`docs/MIGRATIONS.md`](../../docs/MIGRATIONS.md).

### Coplanar unions along a long crease concentrate fans

The fewest triangles for a long planar strip whose one side is a single
straight edge include a fan from that edge's endpoints: every triangle near
an end shares the end vertex. Unions of translated icosahedra build four such
strips, because four face planes contain the translation direction. Each new
icosahedron's coplanar face then overlaps every fan triangle at the growing
end, and its corefinement handles constraints from all of them. This chain
allocated 7.9 GB over six unions against 6.5 GB before reduction, although its
output is 164 triangles instead of 276, and its per-step cost still grows with
the number of unions. It is the only corpus chain whose cost grew beyond the
reduction's own bookkeeping.

Reduction fans each polygon around its earliest-constructed vertex, which puts
the fans at the old end of these strips. Against starting the ring at that
vertex, which fans around its predecessor, this cut the chain's allocation by
22% and the corpus total by 6% without changing any chain's output size
(measured before support planes were retained). It does not remove the cost.
**Options:** overlay a new coplanar face against the retained plane-class
region as one polygon instead of each of its triangles, which is the
arrangement-side alternative to output reduction; or keep a bounded number of
collinear vertices on long creases where the set does not need them, which
gives up minimal size.

### Derived coordinates still deepen where cuts overlap

Cube cuts rotated by 30, 45 and 10 degrees overlap the previous cut, so their
new corners are true intersections with earlier walls, and the crossed wall
edges run between earlier derived corners. Their depth still grows (to 29, 31
and 38), although less than before (53, 41 and 39), and their allocation fell
3.6 to 17 times. Points constructed on one drill-wall plane and the face plane
are collinear by construction, but `corefine_face` and Hypertri's constraint
validation still prove it numerically. **Options:** certify incidence from
construction identity before numeric tests run; represent a point by its
defining planes, so that its coordinates do not depend on previous outputs.

### Kept: embedded-complex evidence and exact axis-plane coordinates

Boolean outputs retain that their triangles meet only in shared vertices
or edges. A later arrangement then records radial separation for shared
reversed edges from identity and skips the remaining self tests on that
operand. On ten corpus chains this published identical rows to the full self
test; with output reduction a hypermesh test still pins the two paths to
identical rows. Crossings on an axis-aligned rational plane take the plane's
exact coordinate instead of an interpolated expression. Before that, a
revolved ring cut twice in the same plane was undecided at the second cut.
