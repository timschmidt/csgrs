# Reference-Guided Performance Notes

These measurements retain only changes that preserve exact rows, output size,
and benchmark checksums. They were collected on 15, 17, and 18 July 2026 with
the release benchmark profile. Times are medians after two warmup batches.

| Workload | Before | After | Change | Preserved evidence |
|---|---:|---:|---:|---|
| `mesh_queries/graphics_buffers` | 1.170 ms/op | 0.139 ms/op | 88.1% faster | 1,584 indices and exact position/normal regression |
| `mesh_io/all_exporters` | 87.602 ms | 81.581 ms | 6.9% faster | 385,166 output bytes and checksum |
| `mesh_boolean/prepare_and_extract_four` | 31.542 ms | 9.606 ms | 69.5% faster | 120 polygons, 120 corners, exact buffers and metadata |
| `mesh_boolean/extract_four_prebuilt` | 31.542 ms | 2.174 ms | 93.1% faster | Same four closure-certified outputs |
| `mesh_queries/ray_intersections` | 32.851 ms/op | 30.819 ms/op | 6.2% faster | Same exact ordered hit points, distances, and checksum |
| `kernel/center/translated_box` (cold) | 23.768 us/op | 6.339 us/op | 73.3% faster | Same 12-coordinate checksum; 1.060x faster than CGAL EPECK |
| `kernel/translate/sphere_medium` (cold) | 0.2513 ms/op | 0.1865 ms/op | 25.8% faster | Same 960-coordinate checksum; 2.095x faster than CGAL EPECK |
| `kernel/affine_transform/sphere_shear` (cold) | 0.1288 ms/op | 0.1069 ms/op | 17.0% faster | Same 960-coordinate checksum; 5.125x faster than CGAL EPECK |
| `profile_primitives/constructor/circle_with_keyway` | 4.72 ms/op | 0.209 ms/op | 95.6% faster | Same contour roles, area, exact containment oracle, and checksum |
| `profile_primitives/constructor/keyhole` | 5.58 ms/op | 0.161 ms/op | 97.1% faster | Same contour roles, area, exact containment oracle, and checksum |
| `profile_primitives/constructor/crescent` | 10.9 ms/op | 1.42 ms/op | 87.0% faster | Same contour roles, area, 605 exact containment oracle decisions, and checksum |
| `profile_primitives/constructor/circle` | 0.129 ms/op | 0.0791 ms/op | 38.7% faster | Exact former `Region2`, point cache, and symbolic unit-normal cache at five tessellations |
| `profile_primitives/constructor/ellipse` | 0.171 ms/op | 0.0452 ms/op | 73.6% faster | Exact former `Region2` equality across five aspect-ratio/tessellation cases |
| `profile_primitives/constructor/star` | 0.128 ms/op | 0.0539 ms/op | 57.8% faster | Exact former `Region2` equality across five point-count/radius-ratio cases |
| `profile_primitives/constructor/regular_ngon` | 0.0368 ms/op | 0.0201 ms/op | 45.3% faster | Exact former `Region2` equality at six side counts |
| `profile_primitives/constructor/teardrop` | 0.170 ms/op | 0.0606 ms/op | 64.4% faster | Exact former `Region2` equality across five dimension/tessellation cases |
| `profile_primitives/constructor/egg` | 0.118 ms/op | 0.0628 ms/op | 46.9% faster | Legacy finite coordinates within 1e-12 plus exact requested bounds |
| `profile_primitives/constructor/squircle` | 0.138 ms/op | 0.0634 ms/op | 54.0% faster | Exact former `Region2` equality across six aspect-ratio/tessellation cases |
| `profile_primitives/constructor/ring` | 0.263 ms/op | 0.0868 ms/op | 66.9% faster | Exact former annular `Region2` equality across six dimension/tessellation cases |
| `profile_primitives/constructor/supershape` | 0.0280 ms/op | 0.0692 ms/op | 147.5% exactness cost | Legacy finite coordinates within 1e-11; no internal binary64 demotion |
| `profile_primitives/constructor/airfoil_naca4` | 1.686 ms/op | 1.184 ms/op | 29.8% faster | Exact former-region equality at three tessellations and same checksum |
| `profile_primitives/constructor/involute_gear` | 0.637 ms/op | 0.389 ms/op | 38.9% faster | Legacy coordinates within 2e-12, symbolic samples, and exact beyond-binary64 module retention |
| `profile_primitives/constructor/cycloidal_gear` | 0.391 ms/op | 0.202 ms/op | 48.3% faster | Legacy coordinates within 2e-12, symbolic samples, and exact beyond-binary64 module retention |
| `profile_primitives/constructor/cycloidal_rack` | 0.170 ms/op | 0.0419 ms/op | 75.4% faster | Exact former `Region2` equality across three parameter sets |
| `profile_primitives/constructor/involute_rack` | 0.0809 ms/op | 0.0309 ms/op | 61.8% faster | Exact former `Region2` equality plus zero-root-space boundary oracle |
| `profile_primitives/constructor/reuleaux` | 12.409 ms/op | 0.0650 ms/op | 99.5% faster | Exact former Boolean boundary and same contour checksum |
| `profile_primitives/constructor/heart` | 0.3378 ms/op | 0.3130 ms/op | 7.3% faster | Legacy finite coordinates within 1e-12; exact requested bounds and symbolic samples |

## Retained changes

`Mesh::build_graphics_mesh` previously called `Mesh::triangulate`, which built
temporary triangle polygons, recomputed support planes, allocated metadata and
IDs, and then discarded that intermediate mesh. The renderer boundary now
streams the same certified triangle indices directly from each source polygon
into exact `Real` position/normal rows. A regression test compares every row and
the sequential index buffer with the former materialized path.

`Mesh::indexed_triangles` now reserves position, normal, face, and lookup
storage from source topology bounds before exporters populate it. Deduplication
still uses retained position and plane identities; output order and serialized
bytes are unchanged.

`Mesh::try_prepare_boolean` now adapts and subdivides one mesh pair into a
borrowed `PreparedMeshBoolean` once, then extracts union, difference,
intersection, and symmetric difference from retained exact winding evidence.
Each extraction still closure-certifies the HyperMesh result and restores
source polygon metadata. Direct methods now use the same pipeline with a
single-operation scope, preserving operation-specific winding-reachability
pruning without retaining a separate one-shot implementation. Empty,
adapter-empty, exactly disjoint, and identical inputs are prepared as exact
set-identity states, preserving the former direct polygonization and metadata.

Nine-sample release medians characterize the unified path:

| Workload | Scoped direct | Prepare + extract | Prebuilt extract |
|---|---:|---:|---:|
| overlapping, one operation | 7.90 ms | 7.94 ms | 0.59 ms |
| overlapping, two operations | 15.60 ms | 8.35 ms | 0.98 ms |
| overlapping, four operations | 31.55 ms | 9.61 ms | 2.18 ms |
| disjoint, four operations | 20.9 us | 20.2 us | 20.0 us |
| identical, four operations | 154.1 us | 44.6 us | 8.2 us |
| contained, four operations | 26.29 ms | 7.50 ms | 1.16 ms |
| face-touching, four operations | 24.53 ms | 6.80 ms | 0.76 ms |

Every row retains the same polygon/corner checksum. Focused differential tests
also compare exact vertex rows, source metadata, and shortcut polygonization.

Cold HyperMesh extraction now reuses each certified source triangle's retained
exact support normal when materializing CSGRS shading normals. Previously every
used source polygon rebuilt an exact cross product even though HyperMesh had
already constructed the same oriented support plane during preparation. The
retained area vector passes through the existing finite shading-normal boundary
and CSGRS normal cache; symbolic or unavailable source normals retain the full
exact recomputation fallback. Cache lookup precedes source-vector cloning, so
repeated extraction keeps the prior warmed path.

Seven counter runs over 500 fresh sphere/box operations measured the extraction
change after the source-centroid optimization:

| operation | recomputed source normal | retained support normal | result |
| --- | ---: | ---: | ---: |
| union | 9,925,432,002 | 9,481,373,714 | 4.47% fewer instructions |
| difference | 8,295,496,270 | 7,883,059,273 | 4.97% fewer instructions |

Cycles fell 7.16% for union and 8.47% for difference. In a 15-sample matched
cross-kernel run, cold exact difference measured 1.908 ms versus CGAL EPECK's
1.896 ms, and cold union measured 2.693 ms versus 2.480 ms. Retained CSGRS was
20.49x faster than CGAL for difference and 17.55x for union, while both cold
and retained rows remained well ahead of tight OpenCascade. Focused tests
verify retained source orientation, invalid-source rejection, output flat-normal
orientation, exact buffers, and metadata.

Validation passed the locked default and all-feature suites (306/370 library
tests plus all integration tests), every locked feature check, all-target and
all-feature Clippy with warnings denied, warning-denied rustdoc, spelling, both
adversarial scripts, and the benchmark and fuzz-target builds. The retained
Boolean adapter and mesh-pair sanitizer targets each completed 1,000 ASAN runs
without failure.

Cold HyperMesh output materialization now appends all triangle vertices to one
preallocated buffer and gives each CSGRS polygon an exact shared range. Source
metadata, fresh plane identities, flat shading normals, and independent
copy-on-write mutation are unchanged. This replaces a three-vertex `Vec` and
its separate `Arc` allocation for every emitted triangle with one result-wide
buffer.

Across 30 counter runs of 500 alternating-input sphere/box operations, union
instructions fell from 10,034,937,171 to 9,950,943,190 (0.84%) and cycles fell
0.55%. Difference instructions fell from 7,935,683,222 to 7,872,306,550
(0.80%) and cycles fell 0.36%. A 50-union heap trace recorded 1,060,054
allocations versus 1,084,445, eliminating 24,391 allocations, or about 488 per
union. A direct regression verifies that every materialized output polygon
shares the retained vertex buffer while the existing exact-output and metadata
oracles remain unchanged.

`Mesh::ray_intersections` now streams triangle vertex references from each
polygon's certified triangulation indices instead of cloning a temporary
`[Vertex; 3]` for every ray test. The HyperLimit ray/triangle predicate, exact
hit ordering, and exact deduplication are unchanged. A differential test replays
the former materialized enumeration against the streaming result.

The exact translation path now materializes its complete finite position view
once, shares that retained allocation with both the coordinate cache and the
result layout, and skips construction of an indexed vertex pool unless the
already-known triangle, normal, and coordinate-layout preconditions can accept
it. Polygon corner counts survive the generic translated fallback because
translation changes neither polygon order nor arity. That retained fact also
lets the HyperMesh adapter triangulate translated cuboids directly from their
canonical position slots instead of requiring the triangle-only pool. A
differential regression compares every exact triangle coordinate row, source
polygon row, and position-ID mapping against generic triangulation.
Independently cached constructors may assign different canonical slot numbers
to the same triangle rows, so the identical-input shortcut now compares those
exact indexed coordinate rows instead of treating raw local indices as global
geometry identity. Seven cold
cross-kernel samples put translated-box centering at 6.339 us, medium-sphere
translation at 0.1865 ms, and sphere shear at 0.1069 ms; all three beat both
CGAL EPECK and OpenCascade while preserving their prior checksums.

AddressSanitizer completed 10,000 transform-matrix executions and 2,000
HyperMesh-adapter executions with leak detection disabled for the ptrace-hosted
runner. The adapter campaign exposed two stale harness assumptions rather than
production faults: the native mesh-buffer view intentionally expands triangle
corners while connectivity canonicalizes exact positions, and an open or
otherwise Boolean-invalid triangle stream can still have valid native buffers
and partial connectivity. The corrected oracle now checks bidirectional exact
position coverage and reserves closed-manifold claims for successful HyperMesh
imports.

`Profile::circle_with_keyway` now recognizes the exactly certified case where
both left cutter corners lie in the convex sampled circle. It constructs that
single attached notch with one ring traversal and a logarithmic convex-point
certificate instead of invoking the general region Boolean. Deep or wide
cutters whose topology is not covered by that certificate retain the former
Boolean path. Four fast-path geometries match the Boolean oracle's contour
roles, signed area, and 484 exact-rational containment classifications; a
separate deep/wide regression proves the fallback remains selected.

`Profile::keyhole` applies the dual construction when a centered rectangular
handle reaches at least the sampled circle radius and its bottom corner is
certified inside the convex ring. The height proof ensures the rectangle covers
the complete replaced upper arc; a logarithmic convex-point query and two exact
line intersections then build the union boundary in one traversal. Short and
overly wide handles retain the general Boolean. Four direct geometries match
the Boolean oracle's contour roles, signed area, and 572 exact-rational
containment classifications; two fallback regressions cover both excluded
topology classes.

`Profile::crescent` now classifies the complete topology before invoking a
general Boolean. Properly crossing homothetic regular polygons are joined from
their two exactly certified boundary transitions: the outer outside arc is
spliced to the reversed inner inside arc. X-axis reflection reuses lower-half
membership decisions. Exact radius/offset identities handle containment as an
explicit material-plus-hole region and disjointness as the unchanged outer
ring, including internal and external tangency. This removes the former
`RealSign` panic at internal tangency. Five crossing geometries, including odd
segments and negative offset, match the Boolean oracle's contour roles, signed
area, and 605 exact-rational containment classifications; contained and
disjoint identities have separate regressions.

`Profile::circle` now materializes its regular sampled polygon from the
constructor's own topology certificate. Positive radius and at least three
cyclic samples prove distinct neighboring vertices, nonzero chords, simple
closure, and consistent winding, so the generic exact distance predicates do
not need to rediscover those properties. The convex tessellation cache remains
the exact authored point sequence. A differential regression compares both the
complete former `Region2` and point cache at 3, 4, 5, 16, and 31 segments.
The retained side-normal cache now also samples exact half-step angles instead
of promoting binary64 sine/cosine values; the same differential proves every
normal against its symbolic construction. Matched 30-sample medians fell from
0.129 to 0.0791 ms/op (38.7%), with non-overlapping 0.125--0.155 and
0.0788--0.0799 ms interquartile ranges. A one-call trace moved from 1,752 to
2,068 dispatch events because the formerly invisible normal sampling is now
exact and recorded. Unknown and fallback/abort events each fell from 88 to
zero, with no approximation or refinement events before or after.

`Profile::ellipse` applies the same certified materialization after its positive
width and height admission. Independent positive axis scaling is an
orientation-preserving affine map of the regular sampled polygon, so it
preserves distinct neighboring vertices, nonzero edges, simplicity, closure,
and winding. A differential regression compares the complete former exact
`Region2` at five portrait, landscape, and strongly eccentric tessellations.
Matched 30-sample medians fell from 0.171 to 0.0452 ms/op (73.6%), with
0.136--0.179 and 0.0441--0.0455 ms interquartile ranges. A one-call trace fell
from 1,647 to 1,175 dispatch events (28.7%); unknown and fallback/abort events
each fell from 88 to zero, with no approximation or refinement events before
or after.

`Profile::star` now uses the direct line-ring path after its strict positive
inner-radius and larger-outer-radius admission. Alternating samples lie on
strictly increasing rays; each chord is contained in its own angular wedge, so
nonadjacent chords cannot cross and every authored edge is nonzero with
consistent winding. A differential regression compares the complete former
exact `Region2` at five point counts and radius ratios, including nearly equal
radii and strong contrast. Matched 30-sample medians fell from 0.128 to 0.0539
ms/op (57.8%), with 0.121--0.143 and 0.0519--0.0571 ms interquartile ranges. A
one-call trace fell from 1,658 to 1,190 dispatch events (28.2%); unknown and
fallback/abort events each fell from 88 to zero, with no approximation or
refinement events before or after.

`Profile::regular_ngon` reuses the same positive-radius regular-polygon proof
as `Profile::circle`, without adding the circle-only convex-normal cache. A
differential regression compares the complete former exact `Region2` at 3, 4,
5, 7, 16, and 31 sides. Matched 30-sample medians fell from 0.0368 to 0.0201
ms/op (45.3%), with 0.0360--0.0431 and 0.0174--0.0211 ms interquartile ranges.
A one-call trace fell from 723 to 589 dispatch events (18.5%); unknown and
fallback/abort events each fell from 26 to zero, with no approximation or
refinement events before or after.

`Profile::teardrop` now materializes its sampled semicircle-and-tip boundary
from the constructor's existing strict `length > width / 2` certificate. The
positive cap-center height places the entire semicircle on or above its
endpoint chord while both tip edges stay below it except at shared endpoints,
proving nonzero edges, simple closure, and consistent winding. A differential
regression compares the complete former exact `Region2` across five dimension
and tessellation regimes, including the minimum two-segment cap. Matched
30-sample medians fell from 0.170 to 0.0606 ms/op (64.4%), with 0.166--0.177
and 0.0566--0.0672 ms interquartile ranges. A one-call trace fell from 2,112
to 1,436 dispatch events (32.0%); unknown and fallback/abort events each fell
from 92 to zero, with no approximation or refinement events before or after.

`Profile::egg` no longer demotes every symbolic sine/cosine sample to binary64
for internal normalization. The discrete extrema are known from uniform sample
indices: x extrema are nearest the quarter turns, while
`d(c + c^2/5)/dc = 1 + 2c/5` is strictly positive on `[-1, 1]`, so y extrema
are nearest 0 and pi. Two exact span divisions are hoisted outside the point
loop, and analytically tied extrema are canonicalized to the exact requested
half-dimensions. The raw curve's signed curvature
`1 + (2/5) cos(theta)^3 >= 3/5` certifies the normalized sampled ring as
distinct, convex, simple, and consistently wound. Across seven tessellations,
the new path stays within `1e-12` of every legacy finite coordinate, preserves
non-rational exact coordinates, and proves the requested width and length
exactly. Matched 30-sample medians fell from 0.118 to 0.0628 ms/op (46.9%),
with 0.100--0.121 and 0.0618--0.0644 ms interquartile ranges. A one-call trace
fell from 2,083 to 1,377 dispatch events (33.9%); approximation, refinement,
fallback, and unknown events remained zero.

`Profile::squircle` now materializes its exact signed-square-root samples from
the constructor's Lamé topology certificate. Positive width and height scale
the strictly convex boundary `x^4/rx^4 + y^4/ry^4 = 1`, so cyclic samples are
distinct and form a simple, consistently wound convex polygon without generic
edge validation. A differential regression compares the complete former exact
`Region2` across six aspect-ratio and tessellation cases. Matched 30-sample
medians fell from 0.138 to 0.0634 ms/op (54.0%), with 0.126--0.156 and
0.0587--0.0683 ms interquartile ranges. A one-call trace fell from 2,135 to
1,711 dispatch events (19.9%); unknown events fell from 96 to zero and
fallback/abort events from 160 to 64, with approximation and refinement events
remaining zero. The retained 64 fallback events belong to the intrinsic
signed-square-root classification, not ring topology validation.

`Profile::ring` now applies the same analytic topology certificate separately
to its two concentric regular polygons. Positive input dimensions and at least
three samples make both boundaries simple and consistently wound; the larger
outer radius makes the inner polygon a strict positive homothety inside it, so
the material and hole roles need no generic ring validation. A differential
regression compares the complete former exact annular `Region2` across six
diameter, thickness, and tessellation cases. Matched 30-sample medians fell
from 0.263 to 0.0868 ms/op (66.9%), with 0.260--0.265 and 0.0858--0.0885 ms
interquartile ranges. A one-call trace fell from 3,646 to 2,174 dispatch events
(40.4%); unknown and fallback/abort events both fell from 176 to zero, while
approximation and refinement events remained zero.

`Profile::supershape` no longer demotes its six parameters, symbolic sample
angles, powered superformula terms, radius, and final coordinates to binary64.
General exponents now use Hyperreal powers throughout. The common
`a = b = 1`, `n2 = n3 = 2` identity is reduced exactly through
`cos(theta)^2 + sin(theta)^2 = 1`, avoiding the general power graph while
retaining exact circle samples. Positive radii and uniform angular wedges
certify the resulting polygon directly. Symmetric and asymmetric differential
cases retain every legacy finite coordinate within `1e-11` and preserve
non-rational exact coordinates. Removing the internal approximation raises the
matched 30-sample median from 0.0280 to 0.0692 ms/op (2.48x), with
0.0266--0.0327 and 0.0688--0.0702 ms interquartile ranges. The one-call trace
rises from 788 to 1,999 events because the exact sine/cosine path is now
observable; fallback, approximation, and refinement events remain zero.

`Profile::airfoil_naca4` previously evaluated every interior chord station
twice: once while constructing the upper surface and again while traversing the
lower surface in reverse. It now evaluates each exact station once, appends the
upper point immediately, and retains the corresponding lower point for the
same reverse-order assembly. Matched 15-sample medians fell from 1.686 to 1.184
ms/op (29.8%). A one-call trace fell from 22,928 to 14,441 dispatch events;
generic sine, cosine, and arctangent evaluations each fell from 158 to 80 and
refinements remained zero. A differential oracle reconstructs the former
two-pass algorithm and compares the complete exact `Region2` for symmetric and
cambered airfoils at 10, 24, and 80 samples. The shared sketch-constructor fuzz
target completed 1,000 AddressSanitizer-instrumented executions after the
change with no failure.

`Profile::involute_gear` no longer demotes module, pressure angle, clearance,
backlash, analytic radii, or sampled angles to binary64. Hyperreal square root,
inverse tangent, and trigonometry retain the analytic samples directly. The
constructor evaluates each radial flank sample once for one local tooth, reuses
that retained sequence in reverse for the opposite flank, and assembles the
gear with exact pitch rotations. Even tooth counts reuse half-turn symmetry;
multiples of four reuse exact coordinate swaps and sign changes for three
quadrants. The admitted analytic ranges also certify the line ring without a
second generic validation pass. A differential oracle compares every boundary
coordinate with the former finite implementation within 2e-12 across three
pressure-angle, clearance, backlash, tooth-count, and tessellation cases; an
adversarial module one unit beyond binary64 integer resolution remains exactly
distinguishable after construction. Matched 30-sample medians are 0.389 ms/op
with a 0.384--0.396 ms interquartile range: 38.9% faster than the original
0.637 ms generic path, while paying a documented 125% exactness cost against
the intermediate 0.173 ms finite sampler. The optimized exact one-call trace
emits 1,213 events versus 24,619 originally and 943 for the invisible finite
sampler, with six arctangent and five generic square-root evaluations and no
unknown or approximation events.

`Profile::cycloidal_gear` now obtains its first-lobe tip and root parameters
from exact closed-form radial-square identities and retained inverse cosine
nodes, replacing the former binary64 conversion and 64-step floating bisection.
It samples one oriented epicycloid and hypocycloid per tooth shape, reuses those
samples for both flanks, and assembles repeated teeth through shared exact
rotational symmetry. Tip and root arcs remain exact `Real` polar samples. A
differential oracle compares every boundary coordinate with the former finite
implementation within 2e-12 across three lobe and symmetry cases; a module one
unit beyond binary64 integer resolution remains distinguishable after complete
construction. Matched 30-sample release medians are 0.202 ms/op with a
0.201--0.205 ms interquartile range: 48.3% faster than the original 0.391 ms
path, while paying a documented 71% exactness cost against the intermediate
0.118 ms finite sampler. Its one-call trace fell from 12,839 to 3,292 dispatch
events (74.4%), with zero approximation, refinement, fallback, unknown-fact,
or rational-GCD events.

`Profile::cycloidal_rack` now evaluates its exact cycloid lobe once and reuses
that symbolic `Real` boundary under pitch translations for every tooth. Its
constructor also owns a direct topology certificate: positive admitted radius
gives `x'(theta) = r(1 - cos(theta))`, so every positive sample interval is
strictly x-monotone; positive pitch proves the bottom edge nonzero, and the
strictly negative root line proves both side edges nonzero. The shared endpoint
assembly is therefore connected, closed, simple, and consistently wound without
repeating generic exact distance predicates. A copied former implementation
matches the complete exact `Region2` across three tooth/resolution/clearance
sets. Matched 30-sample medians fell from 0.170 to 0.0419 ms/op (75.4%), with
non-overlapping 0.162--0.185 and 0.0325--0.0469 ms interquartile ranges. The
one-call trace fell from 1,848 to 667 events (63.9%); sine and cosine calls each
fell from 33 to 9, while 128 unknown/fallback events fell to zero.

`Profile::involute_rack` uses the same certified line-ring materialization when
its already-computed tip width and root space are strictly positive. Those
inequalities, together with positive module, dedendum, and pressure-angle
tangent, make each authored x coordinate strictly increase and every tooth
slope nonzero; the closing root edge spans the complete rack width. Exact zero
root space formerly authored one duplicate edge and returned empty, so it is
still rejected before the certified path. The complete exact `Region2` matches
the former generic constructor across three valid parameter sets, and a
symbolically constructed zero-root-space case matches its empty result. Matched
30-sample medians fell from 0.0809 to 0.0309 ms/op (61.8%), with non-overlapping
0.0796--0.0834 and 0.0305--0.0318 ms interquartile ranges. The one-call trace
fell from 1,003 to 499 events (50.2%); unknown events fell from 92 to zero and
fallback/abort events from 93 to one.

`Profile::reuleaux` now recognizes the exact sampling alignment
`circle_segments % (4 * sides) == 0`. Under that certificate, every analytic
arc endpoint is already one of the regular-polygon centers, so the intersection
boundary is assembled directly from those shared centers and the intervening
source-circle samples. This visits the retained half-circle boundary once and
does not ask the general arrangement engine to rediscover known arc topology;
unaligned tessellations retain the former fallible Boolean path. The canonical
3-sided, 24-segment boundary matches the former Boolean's exact segment cycle.
An aligned 5-sided regression checks its single 10-segment material ring and
width. Across 30 samples, the median fell from the retained 12.409 ms/op
baseline to 0.0650 ms/op (99.5%), with a 0.0643--0.0685 ms interquartile range
and the same checksum. A one-call trace fell from 117,970 to 797 dispatch events
(99.3%); approximations and refinements remained zero.

`Profile::heart` no longer evaluates five exact trigonometric expressions per
sample, demotes them to binary64 for normalization, and promotes the results
back into `Real`. The multiple-angle terms are the exact polynomial
`y(c) = 4 + 19c - 2c^2 - 8c^3 - 8c^4`, so each station now evaluates one sine
and one cosine. Quarter-turn symmetry supplies the sampled x extrema, the
sample nearest pi supplies y-min, and the single derivative sign change makes
y-max an exact logarithmic unimodal search rather than a linear four-comparison
scan. Across 30 samples, the median fell from 0.3378 to 0.3130 ms/op (7.3%),
with non-overlapping 0.3366--0.3387 and 0.3068--0.3302 ms interquartile ranges.
The one-call trace fell from 3,857 to 2,742 events (28.9%), and cosine calls
from 128 to 32. Seven tessellation counts from 8 through 63 retain every legacy
finite coordinate within 1e-12 while now proving exact requested width and
height and retaining symbolic sample dependencies.

The shared sketch-constructor target then completed 1,000
AddressSanitizer-instrumented executions (350 coverage points and 573 feature
edges). Supershape inputs retain negative, zero, and positive exact powers in
the bounded integer range -4 through 4 so fuzz throughput cannot be dominated
by arbitrarily large symbolic exponent expansion; this is a harness-only bound,
and the public constructor remains unrestricted. Its generated corpus exposed
exact-Boolean uncertainty in the legacy
fallbacks for a 3-segment keyhole and a 15-sided Reuleaux polygon. Those public
constructors now use the fallible Boolean surfaces and fail closed instead of
reaching the panic-based convenience methods; both minimized four-byte inputs
have permanent unit regressions and replay cleanly.

## End-to-end dispatch evidence

Dispatch recording now occurs inside each selected benchmark closure, so setup
for unselected feature rows cannot contaminate a trace. On the overlapping-cube
four-operation sentinel, ordinary calls recorded 1,012,924 dispatch events,
56,224 point classifications, four HyperMesh preparations, and four
subdivisions. Build-once/extract-four recorded 354,164 events, 14,056 point
classifications, one preparation, and one subdivision. Both traces recorded
four output-closure certifications, zero approximation events, zero refinement
events, and zero unknown-fact events.

The public free writer surfaces now have their own in-memory benchmark rather
than inheriting evidence only from the `Mesh` string-returning wrappers. The
nine-sample `mesh_io/public_writer_exporters` median is 45.934 ms for 314,636
materialized OBJ, PLY, plain AMF, colored AMF, and glTF bytes. Its isolated
trace records 151,935 dispatch events and one benchmark-entry marker, with zero
approximation, refinement, or unknown-fact events and checksum 314,636. Direct
writer-versus-string equality tests cover each format; the export-name fuzz
target completed 1,010 AddressSanitizer-instrumented nightly executions with
leak detection disabled for the ptrace-hosted runner. Separate nine-sample rows
retain format-level baselines: OBJ 8.996 ms for 44,308 bytes, PLY 10.647 ms for
137,970 bytes, AMF 8.621 ms, and glTF 8.838 ms for 41,659 bytes.

Replay the focused measurements with:

```sh
CSGRS_BENCH_FILTER=mesh_queries/graphics_buffers \
CSGRS_BENCH_SAMPLES=9 \
CSGRS_BENCH_WARMUP=2 \
cargo bench --bench feature_pipeline --features offset,bevymesh

CSGRS_BENCH_FILTER=mesh_io/all_exporters \
CSGRS_BENCH_SAMPLES=7 \
CSGRS_BENCH_WARMUP=2 \
cargo bench --bench feature_pipeline --features offset,bevymesh

CSGRS_BENCH_FILTER=mesh_boolean \
CSGRS_BENCH_SAMPLES=9 \
CSGRS_BENCH_WARMUP=2 \
cargo bench --bench feature_pipeline --features offset,bevymesh

CSGRS_BENCH_FILTER=mesh_queries/ray_intersections \
CSGRS_BENCH_SAMPLES=9 \
CSGRS_BENCH_WARMUP=2 \
cargo bench --bench feature_pipeline --features offset,bevymesh
```

## Audited non-adoptions

The reference section's cubic Bezier GPU distance algorithm is a finite
nearest-distance technique. It is not substituted for exact/adaptive topology
predicates, and `csgrs` has no current public curve-distance API that would
justify adding a rendering-specific approximation. Likewise, the CAD/CAM
hyperbook's interval, subdivision, and offset methods are not claimed as
certificates where the current implementation explicitly crosses a finite
adapter boundary.

Taubin's two-pass smoothing filter and the uniform Laplacian formulation in
Botsch et al. keep topology and per-valence weights fixed across iterations. An
experiment therefore retained the lambda/mu stencils and alternated two reusable
position buffers instead of rebuilding scale divisions and allocating each
pass. On `mesh_refinement/subdivide_and_smooth`, nine-sample medians were
113.870 ms for the existing implementation and 113.933 ms for the experiment;
the identical 276,830,400 checksum confirmed output preservation, but the timing
was indistinguishable and slightly worse. The production change was removed.
The existing end-to-end benchmark remains the regression sentinel; exact `Real`
neighbor accumulation, not vector allocation or valence-scale division, dominates
this workload.

The single-pointer linear convex-polygon intersection algorithm of O'Rourke,
Chien, Olson, and Naddor
(<https://asignatura.us.es/fgcitig/Articulos/09-A%20New%20Linear%20Algorithm%20for%20Intersecting%20Convex%20Polygons.pdf>)
was also evaluated as a shared Hypercurve specialization for the sampled
Reuleaux constructor. The retained general-Boolean baseline is 12.409 ms/op.
Exact strict-convexity certificates based on all half planes, prepared
self-contact plus local turns, and local turns plus a rotation index measured
26.32, 21.30, and 19.66 ms/op respectively. A final translated-source
provenance experiment reached 18.25 ms/op but still could not certify later
symbolic trigonometric edge-head relations, so it paid the specialization and
fell back. Its one-call trace remained effectively the general path (119,927
dispatch events and 1,930 AABB overlaps versus 117,970 and 1,930 at baseline),
with the same one-contour checksum. All variants and temporary provenance code
were removed. The retained aligned-arc optimization above succeeds by proving
the constructor's endpoint indices before any clipping and therefore does not
solve—or pay for—the shared predicate gap. Unaligned convex intersections still
need an algebraic trigonometric zero/sign certificate or source-direction facts
that survive emitted intersection fragments.

The glTF public writer was also changed experimentally from serializing a
pretty JSON string and copying its bytes to calling `serde_json`'s pretty
writer directly. Across 15 samples, its median moved from 8.838 ms to 8.791 ms
(0.5%); the baseline and experiment interquartile ranges overlapped, while the
direct path required an extra document-builder layer and writer-error mapping.
The experiment was removed. The format-level writer benchmark and byte-equality
test remain, so a future retry must demonstrate a material improvement while
preserving byte-identical output.

The isolated feature traces also keep the remaining large rows visible without
weakening their contracts. Symbolic profile offsets recorded 24,005 refinement
events because many trigonometric segment bounds cannot be decided structurally;
TPMS sampling recorded 22,784 refinements before its documented Surface Nets
preview boundary; smoothing recorded 68,074 rational reductions and 102,010
GCDs. Replacing these with primitive-float topology decisions, skipping offset
self-contact validation, or changing the smoothing stencil would change quality
or exactness rather than optimize the same result, so those substitutions were
not attempted.
