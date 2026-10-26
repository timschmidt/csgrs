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
neighbours. Hypertri retriangulates each side under strict policy, whatever
the operation policy, and a vertex whose polygon strict policy cannot
triangulate stays. A
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

### Kept: coplanar strips fan from their oldest end

The fewest triangles for a long planar strip whose one side is a single
straight edge include fans from that edge's two ends. Unions of translated
icosahedra build four such strips, because four face planes contain the
translation direction. Wherever a strip fanned from its growing end, each new
icosahedron's coplanar face overlapped every fan triangle there: one more per
union, each crossing adding exact collinearity proofs to the face's
corefinement. Six unions allocated 7.9 GB, against 6.5 GB before reduction.

Reduction meant to fan each polygon around its least arena vertex, but it
relied on Hypertri's ear clipping starting beside that vertex, and ear
clipping fans around whichever vertex its first rejected ear leaves. Two of
the four strips fanned from their growing end. Since 2026-10-21 reduction uses
Hypertri's `triangulate_simple_ring_around`, which tries the ears beside the
chosen vertex first, so every strip fans from its least vertex; the newest
end keeps two triangles in each strip. The face splitting of each union then
takes the same work whatever the number of earlier unions: about 0.7 s from
the fourth union on, where it took 1.2 s at the fourth union and 0.4 s more
at each later one. Six unions allocate 4.2 GB.

The least arena index is the first-used vertex, not the oldest: the arena
numbers source vertices in operand and row order, then the points the
Boolean builds, and each operation rebuilds it. In `body.union(&tool)` the
previous result's vertices come first, which is why its strips fan from
their old end. Without vertices on the straight edge, which the set does not
need, no triangulation keeps both ends local.
Hypertri's `simple_ring` tests check every apex, rotation and orientation of
concave, deep-pocket, star, chevron, collinear and strip rings against the
ring alone (positive areas, each edge once, no edge over a vertex) and that
every rotation gives the same triangles.

**Measured (2026-10-23, research project M06).** A release probe (not
committed) ran eight unions of translated icosahedra in six ways and counted
the coplanar-overlap pairs each union's arrangement found, through a
temporary dispatch-trace label. Every variant gave certified closed
manifolds of 44 + 24k triangles, and the five with one set had exactly
equal volumes at every step.

| Variant | Overlap pairs, unions 1 to 8 | MB at union 8 | MB, 8 unions |
| --- | --- | ---: | ---: |
| `body.union(&tool)`, the corpus chain | 4, 8, 12, then 8 | 392 | 2,755 |
| `tool.union(&body)` | 4, 8, ... 26, one or two more per union | 2,209 | 9,714 |
| ends alternating, `body.union(&tool)` | 4, 8, 8, 12, 13, 15, 13, 16 | 911 | 4,414 |
| `body` rebuilt from its rows | as the corpus chain | 2,201 | 9,126 |
| `body`'s rows reversed | as the corpus chain | 2,252 | 9,123 |
| each row's vertices rotated | as the corpus chain | 2,113 | 8,974 |

Rebuilding the body from its own rows drops the facts a result retains
(plane classes, supports, the embedded-complex fact), and that, not the fan
apex, makes each union cost more; reversing or rotating the rows changes the
cost by -4% to +2% beside the rebuilt rows. Strips growing at both ends
overlap more fan triangles, as no triangulation without extra vertices can
avoid. Swapping the operands put every new tool's vertices first, so a
strip fanned from the growing end and each union met one or two more of its
triangles.

**Kept (2026-10-23).** Reduction now ranks the vertices of an operand that
retains an earlier Boolean's facts before the other operands' vertices and
the points the Boolean builds, and only then by arena index. When such
operands already come first, as in every corpus chain, the order is the
arena order and needs no storage, so the corpus keeps its rows and
allocation exactly. In `tool.union(&body)` the eighth union allocates
665 MB instead of 2,209 MB and the eight 4,926 MB instead of 9,714 MB, with
10 to 12 overlap pairs per union from the fourth on. `coplanar_union_strips_fan_from_their_oldest_end_with_the_new_operand_first`
checks the tetrahedron strips that way round: one strip kept `k + 1`
triangles at its newest end after union `k`, and now keeps three.

**Open.** The rank is an operand's provenance, not each vertex's age:
within the earlier result, first use follows that result's own rows, which
list the geometry its first operand brought first. That is why swapped
unions still meet more overlaps than the corpus chain (10 against 8) and
allocate 1.7 times as much, and why one tetrahedron strip keeps three
triangles at its newest end. A retained per-vertex age would fix this at
four bytes a vertex. An operation-local plane-class region view (overlay
each class's boundaries once, triangulate the selected output) would also
remove the dependence on fans, and retained collinear vertices could keep
both ends of a strip local at the cost of output size; neither is justified
by these numbers yet.

### Kept: crossings of retained creases are built from planes

Cube cuts rotated by 30, 45 and 10 degrees overlap the previous cut, so their
new corners lie where a cutting wall crosses an edge between a block face and
an earlier wall, and that edge runs between corners built by earlier cuts.
Interpolating between its endpoints deepened the coordinates with every
generation, to depth 29, 31 and 38. Such an edge joins two triangles whose
retained plane classes are distinct planes, so both retained supports contain
it and meet exactly in its line; since the crossing is proper, the two planes
and the cutting plane have independent normals and meet in exactly the
crossing point. Since 2026-10-21 that point is built from the three planes
by Cramer's rule when the endpoints' expression graphs are deeper than the
two planes', which leaves rational endpoints and unchanged source vertices on
interpolation. A coordinate that both endpoints store as one expression keeps
it, one that an exact axis plane fixes keeps its rational value, and a
division that does not decide falls back to interpolation. The three chains
keep depth 16, 17 and 18 from their fourth, third and first cut on, checked
through 16, 8 and 8 cuts, and allocate 13 to 27% less; with shared output
coordinates (below) the 30- and 45-degree chains reach at most 15 and 17.
Regressions (2026-10-22) cover a proper crossing whose determinant is
nonzero but too small for division to decide, which is interpolated, and a
crossing that keeps a symbolic coordinate both endpoints store as one
expression, which fails without that rule.

Points constructed on one wall plane and a face plane are collinear by
construction. `corefine_face` and Hypertri's constraint validation proved that
numerically, and these orientation zeros were most of the instructions of a
coplanar union step; since 2026-10-22 the arrangement takes them from
construction (next section).

### Kept: incidences from construction replace zero proofs

Every arrangement point has construction identities: a source vertex, a
source edge crossing a support plane, three support planes, or two lines of
one plane. Each puts the point exactly on planes and lines, because
interpolation and Cramer's rule solve the defining equations without
approximation. A face arrangement nevertheless proved every incidence again
by an exact zero, which the scalar layer must prove rather than refine: that
each point lies in its face's plane and on the edges it was built on, which
points lie on each authored line, and, in Hypertri, the orientation of every
three collinear points.

Since 2026-10-22 each point keeps all its identities, also those of an equal
point the interner merged it with, and plane classes are built before
corefinement, so support planes compare by class: a point built on one
face's plane lies on that of every face decided or retained coplanar with it.
A split line is the meeting of two faces decided non-coplanar, so a point on
both classes lies on it. From these facts the arrangement decides, without a
determinant:

- that a point lies in its face's plane, or on one of the face's edges, when
  checking that it lies inside the face;
- that a point lies on an authored line, leaving only its position along the
  line to exact comparisons, and that two constraints on one line, or with an
  endpoint on the other's line, cannot cross properly;
- for Hypertri's constrained triangulation, the points of each authored line,
  whether known from construction or decided collinear by the pass above,
  through `constrained_triangulation_convex_hull_with_collinear_facts`; three
  points of one line then have orientation zero;
- that two facets cut from faces of one class are coplanar around a shared
  edge, and that the seed of a cell, the centroid of its facet, lies in the
  plane of that facet and of every face of its class.

The facts only ever decide that a point is on a plane or line, never that it
is off one. Imported or edited meshes carry only source identities, so their
other incidences are proved as before. Two classes with a contradictory fact
give no incidence, which exact decisions cannot produce. Exact rational
arithmetic decides every zero of rational geometry cheaply, so the identity
bookkeeping is skipped when every input coordinate is rational. Hypertri
trusts the sets it receives and records their certainty: a set that consumed
the approximate terminal is used only under `APPROXIMATE_512`.

The change also fixed `boundary_line_for_segment`, which named a coplanar
contact segment by the edge before the one it lies on; edge plane `i` and
edge identity `i` describe the same edge. The wrong name reached only the
constraint deduplication, where either name is that of the same segment,
but the incidence facts rely on it.

On the successive-operation corpus every step keeps its outcome, triangle,
vertex and coordinate counts and graph sizes, and every validated step's
exact rows (triangle indices, rational coordinates, and the value and graph
size of each symbolic coordinate) are identical. Allocation over the corpus
falls from 63.4 GB to 45.1 GB: icosahedron unions 3,972 MB to 1,998 MB, the
tilted 32-gon drills 34.3 GB to 20.5 GB, the revolved ring 1,391 MB to
987 MB, and the other symbolic chains by 5 to 28%. Rational chains allocate
the same bytes when run alone; in the full run, four of their steps allocate
up to 296 bytes more, from state that the symbolic chains before them leave
in the process. On five traced symbolic chains Hyperlimit makes 7 to 22%
fewer `orient2d` calls and, on the icosahedron and ring chains, 30% fewer
`orient3d` calls; the rational square drills make the same calls. The
near-coincident cylinders below decide because of it. Measurements and commands are in
[`PERFORMANCE.md`](PERFORMANCE.md).

### Kept: equal output coordinates share one expression

Each Boolean computes exact bounds for every source triangle and every
hierarchy node. Two equal symbolic coordinates built by different operations
compare equal only through refinement to the maximum precision followed by an
exact zero proof, and every later Boolean repeated those proofs. Translated
icosahedra share their y and z values across copies, and the unions' crossings
reproduce earlier coordinates through other expressions: after six unions, 184
pairs of y coordinates were equal but stored differently, up from 4 after one.
The hierarchy build took 115 ms at the fourth union and 340 ms at the seventh,
and the last three of six unions allocated 714, 755 and 893 MB.

Since 2026-10-22 each output stores the coordinates of one axis that strict
policy proves equal as one expression, the smallest of them; hyperreal then
decides their comparisons by structural identity. Candidates are coordinates
within a narrow approximate window, a coordinate matching an expression its
group already holds needs no proof, and an undecided comparison keeps both
expressions, so values and certainty never change. A segment crossing also
keeps a coordinate that both endpoints store as one expression. The last three
of six unions now allocate 654, 681 and 686 MB, and the chain 3,972 MB in
total; depth falls from 29 to 27. Over twelve unions the per-step cost rises
from 712 MB to 812 MB and from 849 ms to 928 ms while the result grows from 116
to 308 triangles; the same probe before rose from 780 MB to 1,354 MB and from
915 ms to 1,554 ms. Rational chains allocate the same, since equal rationals
already compare structurally, and the sampled sphere cut allocates 0.3% more.

The candidate search was unbounded: distinct values crowded into one window,
such as 4,096 dyadics `i / 2^60`, cost a structural comparison and a strict
proof for every pair. Since 2026-10-22 a coordinate compares with the groups
of at most the four nearest approximations, structurally with at most eight
of their expressions, and attempts at most two proofs, none between two
exact rationals; the scratch is flat and reserved only for an axis with a
symbolic coordinate. No corpus chain had more than one candidate group in a
window, so its rows and allocation are unchanged. The bounds count
comparisons and proofs, not their cost: one structural comparison walks both
expression graphs, one proof refines to the maximum precision, each
coordinate is approximated once (possibly refining far), and each expression
in a group of several is measured up to 4,096 nodes. Boolean operations have
no cancellation, so none of these steps observes one.

What remains per step is work over the whole input, such as exact bounds and
the hierarchy build, at about 0.5 MB per input triangle on this chain.
**Option:** retain exact per-triangle extrema and node bounds with a result,
the way plane classes are retained, for the triangles the next Boolean reads
unchanged.

### Kept: embedded-complex evidence and exact axis-plane coordinates

Boolean outputs retain that their triangles meet only in shared vertices
or edges. A later arrangement then records radial separation for shared
reversed edges from identity and skips the remaining self tests on that
operand. On ten corpus chains this published identical rows to the full self
test; with output reduction a hypermesh test still pins the two paths to
identical rows. Since 2026-10-21 the broad phase no longer enumerates those
pairs at all: it skips node pairs inside one such operand and records the
radial separations from its edge adjacency. Enumerating them took exact
bounds tests that grew with the operand: on the icosahedron chain the
pairwise phase took 0.25 s at the third union and 1.1 s at the seventh, and
now takes about 0.1 s at each. The arrangement receives the same facts, and
the pinned paths still agree. Crossings on an axis-aligned rational plane
take the plane's exact coordinate instead of an interpolated expression.
Before that, a revolved ring cut twice in the same plane was undecided at the
second cut.

**What the fact rests on.** An output's facets are a subset of its
arrangement's facets, and output reduction triangulates under strict policy
without changing the complex. So the output meets only in shared vertices or
edges whenever the arrangement's pairwise, corefinement and triangulation
decisions are correct, which certified decisions prove. A decision that
consumed the `APPROXIMATE_512` terminal is not a proof: it reads a value
within 2^-512 of zero as zero, and the value may be a nonzero one. Until
2026-10-22 every output nevertheless recorded the fact as proved. A slot
whose front face lies `u + 2^-600` behind a block face (`u` an exact zero
that no proof decides) is undecided under strict policy; the approximate
difference is the exact one, with a wall `2^-600` thick, and recorded the
fact. A strict union of it with a disjoint box then completed as certified,
while the same rows rebuilt were undecided, and an approximate union reported
`Certified`. The output was in fact embedded, so no wrong set was shown, but
the receipt rested on the consumed terminal. The fact now keeps the
certainty of the arrangement's decisions, like the PWN validation fact:
strict policy uses it only when certified, and an approximate operation that
uses an approximate one consumes the terminal (`tests/boolean.rs`,
`approximate_results_do_not_certify_their_structure_for_later_operations`).
Certified outputs, and so every successive-operation chain, are unchanged.

Open: the certainty recorded is that of the whole operation, as for plane
classes, though embeddedness depends only on decisions up to corefinement;
an operation that consumed the terminal only to classify windings could
record a certified fact. Approximate outputs also publish different rows
when reused with the retained fact than when rebuilt: in the slot fixture
with `u` alone, later approximate Booleans gave 42, 30, 50, 78 and 74
triangles against 40, 28, 44, 64 and 60, because the full self test finds
coplanar faces that the retained path, without plane classes, never learns.

Independent regressions (2026-10-22) compare the grouped broad phase with
the plain traversal less same-group pairs, embedded-input groups (including
missing or mismatched source identities and one output as both operands)
with the plain traversal's pairwise graph, retained radial separations with
the reversed shared-edge pairs on a four-sheet edge, and retained with
rebuilt edge-touching, vertex-touching and balanced non-manifold unions in
later Booleans against box volumes. No disagreement was found.

## Coordinate-drop projection

`surface_arrangement::projection_axis` chooses the coordinate dropped when a
support plane is projected for Hypertri. It takes the first nonzero rational
normal component, then classifies the components in order and returns the
first undecided component's error. csgrs's `solid::certified_projection_axis`,
used by `polyhedron` and by OBJ, DXF and VRML face triangulation, instead
tries the later components, since any component proved nonzero gives an
invertible projection. A support plane whose first component is undecided and
whose later component is provably nonzero would fail here but not there. No
such input has been reproduced; sharing one primitive would need a public
Hypermesh entry point.

## Near-coincident rotations beyond the cyclotomic domain

Found by `tests/downstream_solids.rs` (repository root), migrating the
SynapsCAD fixture `cylinder(h = 40, r = 8, $fn = 8)` minus the same cylinder
rotated by 0.0001 degree. The sine and cosine of 1/10000 degree are rational
turns of order 3,600,000, beyond the cyclotomic zero proof's order 16384, so
exact zeros among the two cylinders' coordinates must come from identities in
independent atoms.

**Fixed below this layer.** The first zero was the order of two crossings on
the line shared by two side faces. They are mirror images across the
rotation's bisector plane, which contains that line, so they are one point;
proving it needs `sin^2 + cos^2 = 1`. Hyperreal stores exact `cos(pi * r)` as
`sin(pi * (1/2 - r))`, and its ring recognizer related only a sine and a
cosine of one argument. It now also relates two sines, or two cosines, whose
arguments sum to `pi / 2` (2026-10-22). That decides rotations of a cube by
1/10000 or 12.345 degree about any axis through a corner, and their round
trips, all of which ended `PredicateUndecided` before.

**Fixed in the arrangement (2026-10-22).** The cylinders then stopped at
`surface arrangement point on segment` or, for rotations about z, whose caps
are coplanar, `surface arrangement orientation`. In the smallest
reproduction, `solid::cylinder(8, 40, 4)` minus itself rotated by 0.0001
degree about y, a side face of the first prism meets both triangles of the
rotated top cap along one line. Its arrangement holds three points on that
line: one built on each cap triangle's plane and one on the triangles' shared
edge. The two triangles were decided coplanar, so the points are collinear
by construction, but the face arrangement tested each point against each
constraint numerically, and the collinearity was a 148-node expression that
refinement leaves at zero to 2^-400. Hypertri's triangulation of the face then
needed the orientation of the same three points. Larger fixtures met the same
kind of zero in a point's containment in its face, in the radial order of
coplanar facets and in the cell seed's ray parameter. All are now taken from
construction (above). The differences for rotations by 0.0001 degree about
each signed coordinate axis and by 0.000001 and 0.01 degree about y, and the
union, intersection and difference about y with and without centring, now
complete as certified closed manifolds. Every boundary triangle of each
separates the exact set from its complement, checked against the convex
operands' half-spaces. For the 4-gon prism `APPROXIMATE_512` gives the same
rows as `STRICT` without consuming its terminal.

**Fixed below this layer (2026-10-23).** About a skew axis,
`rotate(0.0001, [1, 1, 1])` and the compound Euler rotation `[a, -a, a]`
stopped earlier, in the pairwise face tests, at `Real sign classification`:
the order of two crossings of one face pair along their common line, and an
input vertex against the other cylinder's face plane. No construction
relates these points. They are zeros of degree above 8 in the rotation's
sines and cosines: Hyperreal's relation attempt proves the skew-axis zero
with 32,768 work charges instead of 8,192, and its quotient attempt proves
the compound zero at degree 12; neither needs angle addition. No number field
of small degree holds them, since 1/10000 degree is a rational turn of order
3,600,000. Hyperlimit now runs one extended relation attempt with both
limits after every other exact stage declines, and the union, intersection
and difference of both rotations complete as certified closed manifolds
(`tests/downstream_solids.rs`). Each takes about 2 s in release, against
about 0.2 s to fail before.

**Scoped completion (R18, 2026-10-25).** The original binary64 sequence
13.1, 17.3, -13.1 and -17.3 degrees about z now completes its strict cube
union. Captured predicates expose common-factor cancellations and a smaller
unit-circle residual. Hyperreal proves these only after the existing extended
expansion declines, using its remaining work and node budgets. Expanded
polynomials retain the degree-12 limit; no four-million-charge stage is added.
The independent cube half-space/volume oracle, distinct decimal control,
three angle families on each axis under both policies, and empty-result
continuation are in `tests/compound_rotations.rs` at the repository root.
General compound rotations and operation-wide resource bounds remain open.
See the [exact-completion and cost record](../hyperreal/PERFORMANCE.md#factored-compound-rotation-completion-r18-2026-10-25).

## Coplanar contact between differently triangulated caps

Found by `tests/enclosure_workflows.rs` (repository root, research project
C05, 2026-10-23). An enclosure with round plan corners is a base slab
`try_extrude(outer, 1)` united with walls `try_extrude(outer - inner, 5)`.
The rounded outer region is sampled by extrusion to 132 binary64 vertices,
so the base's bottom cap (a triangulated rounded rectangle) and the walls'
bottom cap (a triangulated annulus) are coplanar, overlap, and are
triangulated differently. In one release run of a temporary probe on a
shared host:

| Operation (524 and 544 triangles in, 540 out) | Time |
| --- | ---: |
| base union walls, both from z = 0 | 42.6 s |
| base union walls raised to z = 1 (contact at the slab top) | 37.5 s |
| xor of that result with the same set built below | 12.8 s |
| block `try_extrude(outer, 5)` minus the cavity raised to z = 1 | 38 ms |
| one window cut through the result | 2.6 ms |

Subtraction builds the same set (the xor is empty) without a coplanar
contact. Thirty stack samples of the first union were all in
`surface_arrangement::corefine_face`; 25 were in Hypertri's
`insert_constraints_topology`, 21 of them in `edge_proper_crossing_sides`,
evaluating exact `orient2d` on wide rationals. The same union with rational
corner points `((1 - t^2)/(1 + t^2), 2t/(1 + t^2))`, `t = k/n`, grows with
the number of cap vertices and with their coordinate size:

| Chords per corner | Base triangles | Union |
| ---: | ---: | ---: |
| 1 | 28 | 6.1 ms |
| 4 | 76 | 25 ms |
| 8 | 140 | 77 ms |
| 16 | 268 | 0.33-0.38 s |
| 32 | 524 | 2.56 s |

Each doubling past 8 costs 4-7 times as much, and binary64 sample points of
the same count are about 17 times slower than these small rationals. Every
result was certified, so this is a cost, not a completeness, limit. The
root enclosure test builds its rounded case by subtraction and records why.
The operation-local plane-class region view noted above (overlay each class's
boundaries once, then triangulate the selected output) is the candidate fix;
it was not attempted here.
