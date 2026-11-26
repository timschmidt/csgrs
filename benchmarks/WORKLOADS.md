# Cross-kernel workload definitions

These are current source-defined scenarios, not a claim of matched completed work.
CSGRS boxes start at the origin; CGAL/OCCT boxes are centered. CSGRS sphere samples
use exact trigonometric coordinates with Y poles, CGAL uses double samples with Z
poles, and OCCT uses analytic spheres. Nominal dimensions and CSV names therefore
do not establish equal sets. The execution/validation contract must also match
before treating a timing ratio as an implementation comparison.

| Benchmark/case | Fixed scenario | Timed result |
|---|---|---|
| `construct_box/unit` | side 2; CSGRS `[0,2]^3`, native `[-1,1]^3` | constructed geometry plus facet count |
| `construct_sphere/medium` | radius 10; polygon runners 32×16, OCCT deflections 0.25/0.2 | constructed/tessellated geometry |
| `construct_sphere/large` | radius 10; polygon runners 64×32, OCCT deflections 0.125/0.1 | constructed/tessellated geometry |
| `precision/construct_sphere/high_resolution` | radius 10; csgrs/CGAL use 128×64, OCCT uses `0.005` linear and `0.07` angular deflection | exact-kernel construction or tight OCCT tessellation plus facet count |
| `translate/sphere_medium` | medium sphere translated by `(3,-2,5)` | translated solid |
| `rotate_xyz/sphere_medium` | medium sphere rotated X/Y/Z by 17°/29°/43° | rotated solid |
| `scale_nonuniform/sphere_medium` | medium sphere scaled by `(2,1/2,3/2)` | scaled solid |
| `mirror/sphere_across_x_eq_1` | medium sphere reflected across the plane `x=1`, with outward orientation preserved | mirrored solid |
| `affine_transform/sphere_shear` | fixed XY/YZ shears plus translation `(2,-3,4)`; exact rational coefficients in CSGRS/CGAL, double coefficients in OCCT | transformed per-engine geometry |
| `inverse/sphere_orientation` | medium sphere with every face orientation reversed | inverted solid |
| `boolean_union/sphere_box` | radius-10 sphere (12×6 polygon or analytic); side-14 box at CSGRS minimum `(3,2,1)` versus native center `(3,2,1)` | different-input unions plus output measurement |
| `boolean_difference/sphere_box` | same per-engine operands | sphere minus box plus materialized output |
| `boolean_intersection/sphere_box` | same per-engine operands | intersection plus materialized output |
| `boolean_xor/sphere_box` | same per-engine operands; CGAL appends two differences, OCCT fuses them | symmetric-difference construction plus output measurement |
| `precision/boolean_sliver/overlap_1e-6` | side-2 cubes shifted by exact `1999999/1000000` in CSGRS/CGAL, rounded double in OCCT | CSGRS checks X thickness, CGAL exact volume, OCCT tolerance/volume range; output measured |
| `extrude/circle_64` | radius-6 sampled 64-gon extruded 20 units; exact-expression versus double vertices | constructed prism plus output measurement |
| `triangulate/sphere_medium` | per-engine medium sphere | CSGRS clone, CGAL copy/triangulate call, OCCT remesh |
| `bounding_box/sphere_medium` | sampled or analytic sphere | bounds and selected projected checksum |
| `mass_properties/sphere_medium` | sampled or analytic sphere, unit density | CSGRS mass report, CGAL volume alone, OCCT volume-properties call |
| `stl_write/sphere_medium` | per-engine tessellated sphere | in-memory binary STL in CSGRS/CGAL; fixed temporary file in OCCT |
| `corpus/*/deterministic_concave_labyrinth_31x31x6` | serialized orthogonal labyrinth, 4,012 positions and 8,020 triangles | import, translate, bounds and engine-specific graphics/connectivity/validity rows |
| `corpus/*/sierpinski_foam_level3` | contact-filled Sierpiński-derived foam, 15,232 positions and 36,096 triangles | the same named rows, with engine-specific implementations |

Further differences apply even when row names match. Native graphics cases count
corners (OCCT first remeshes); they do not export the CSGRS graphics buffers. OCCT
subdivision is finer remeshing, and connectivity keeps per-face nodes separate.
CGAL ray/segment queries count intersected triangles; OCCT's ray row uses the same
finite interval as its segment row. Native identical-box intersection is an identity
shortcut. For contained boxes, CSGRS `[0,4]^3 - [0,2]^3` removes a corner, while the
native centered boxes form an enclosed cavity. None of these differences is detected
by engine-label parity or count fingerprints.

## Published Solidean requests

These opt-in rows use the unmodified version-1 request files pinned by
`benchmarks/support/solidean.rs`. Their local CSV rows are not part of native
CGAL/OCCT parity because the comparison values were published by the kernel
authors on a different host.

| Benchmark/case | Fixed scenario | Timed result |
|---|---|---|
| `published-solidean/iterated_difference/primitives_A_minus_B_to_J` | convert A–J and perform 9 ordered differences | final mesh; published area 13.4224, volume 2.3513 |
| `published-solidean/iterated_difference/terrain_0.2_-y_224` | convert the workpiece and 224 tools, then perform 224 ordered differences | final mesh; published area 6.1648, volume 0.5477 |
| `published-solidean/iterated_union_difference/checker_grid_n10_1999` | convert 1,000 cubes, perform 999 unions followed by 1,000 differences | intended empty final mesh; local check uses near-zero projected measures |
| `published-solidean/iterated_difference/dome_carve_N` | convert one block and N capsule tools, then perform N ordered differences for N = 10, 100, 250, 1,000, or 5,000 | final mesh; published area/volume oracle for N ≥ 100 |

## csgrs-specific transform helpers

The broader `feature_pipeline` additionally times API helpers that do not have
a single equivalent operation in all three native kernels:

| Benchmark/case | Covered operations |
|---|---|
| `profile_transform/all_csg_helpers` | two translations, a quarter-turn affine map and non-uniform scaling of one rectangle |
| `profile_distribution/arc_linear_grid` | fourteen grid-translated squares, counted separately without union |
| `mesh_positioning/center_float_vector` | bounding-box center, float-to-Z-zero, and vector translation on `TriangleMesh` |
| `mesh_distribution/arc_linear_grid` | twenty-two grid-translated cubes combined by concatenation |

CGAL's sphere/extrusion builders create double polygon samples before promotion
into `Surface_mesh`. OCCT constructs analytic round solids, but its extrusion case
uses a sampled polygon prism. Its sliver scale is compile-time checked against the
installed confusion tolerance; mass/tolerance checks do not establish exact set
equality. This catalog records current coverage gaps without changing the fixtures
or the repository's requirement to compare the same completed contract.
