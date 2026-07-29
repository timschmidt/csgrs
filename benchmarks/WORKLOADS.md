# Cross-kernel workload definitions

| Benchmark/case | Fixed scenario | Timed result |
|---|---|---|
| `construct_box/unit` | centered cube, side 2 | constructed solid plus facet count |
| `construct_sphere/medium` | radius 10, 32 segments, 16 stacks | constructed/tessellated solid |
| `construct_sphere/large` | radius 10, 64 segments, 32 stacks | constructed/tessellated solid |
| `precision/construct_sphere/high_resolution` | radius 10; csgrs/CGAL use 128×64, OCCT uses `0.005` linear and `0.07` angular deflection | exact-kernel construction or tight OCCT tessellation plus facet count |
| `translate/sphere_medium` | medium sphere translated by `(3,-2,5)` | translated solid |
| `rotate_xyz/sphere_medium` | medium sphere rotated X/Y/Z by 17°/29°/43° | rotated solid |
| `scale_nonuniform/sphere_medium` | medium sphere scaled by `(2,1/2,3/2)` | scaled solid |
| `mirror/sphere_across_x_eq_1` | medium sphere reflected across the plane `x=1`, with outward orientation preserved | mirrored solid |
| `affine_transform/sphere_shear` | medium sphere transformed by fixed exact/rational XY and YZ shears plus translation `(2,-3,4)` | arbitrarily transformed solid |
| `inverse/sphere_orientation` | medium sphere with every face orientation reversed | inverted solid |
| `boolean_union/sphere_box` | radius-10 sphere at 12×6; side-14 box centered at `(3,2,1)` | union plus materialized output |
| `boolean_difference/sphere_box` | same operands | sphere minus box plus materialized output |
| `boolean_intersection/sphere_box` | same operands | intersection plus materialized output |
| `boolean_xor/sphere_box` | same operands; native kernels compose two differences and a union | symmetric difference plus materialized output |
| `precision/boolean_sliver/overlap_1e-6` | side-2 cubes with exact/rational shift `1.999999`; OCCT uses the same binary value with zero added fuzzy tolerance | nonempty `1e-6`-thick intersection plus materialized output |
| `extrude/circle_64` | radius-6, 64-segment circle extruded 20 units | closed solid plus materialized output |
| `triangulate/sphere_medium` | medium sphere | triangulated/tessellated output |
| `bounding_box/sphere_medium` | medium sphere or equivalent analytic sphere | six bounds |
| `mass_properties/sphere_medium` | medium sphere or equivalent analytic sphere, unit density | volume/mass properties |
| `stl_write/sphere_medium` | medium tessellated sphere | in-memory binary STL (csgrs and CGAL) |
| `corpus/*/deterministic_concave_labyrinth_31x31x6` | deterministically serialized closed orthogonal labyrinth, 4,012 positions and 8,020 triangles | OBJ import, exact translation, bounds, graphics buffers, connectivity, and manifold validation |
| `corpus/*/sierpinski_foam_level3` | deterministically serialized closed level-3 Sierpiński foam, 15,232 positions and 36,096 triangles | OBJ import, exact translation, bounds, graphics buffers, connectivity, and manifold validation |

## Published Solidean requests

These opt-in rows use the unmodified version-1 request files pinned by
`benchmarks/support/solidean.rs`. Their local CSV rows are not part of native
CGAL/OCCT parity because the comparison values were published by the kernel
authors on a different host.

| Benchmark/case | Fixed scenario | Timed result |
|---|---|---|
| `published-solidean/iterated_difference/primitives_A_minus_B_to_J` | convert A–J and perform 9 ordered differences | final mesh; published area 13.4224, volume 2.3513 |
| `published-solidean/iterated_difference/terrain_0.2_-y_224` | convert the workpiece and 224 tools, then perform 224 ordered differences | final mesh; published area 6.1648, volume 0.5477 |
| `published-solidean/iterated_union_difference/checker_grid_n10_1999` | convert 1,000 cubes, perform 999 unions followed by 1,000 differences | empty final mesh |
| `published-solidean/iterated_difference/dome_carve_N` | convert one block and N capsule tools, then perform N ordered differences for N = 10, 100, 250, 1,000, or 5,000 | final mesh; published area/volume oracle for N ≥ 100 |

## csgrs-specific transform helpers

The broader `feature_pipeline` additionally times API helpers that do not have
a single equivalent operation in all three native kernels:

| Benchmark/case | Covered operations |
|---|---|
| `profile_transform/all_csg_helpers` | vector translation, Z rotation, non-uniform scaling, plane mirror, center, float, arbitrary affine transform, and inverse on `CurveRegion2` |
| `profile_distribution/arc_linear_grid` | arc, linear, and grid distribution on `CurveRegion2`, including profile unions |
| `mesh_positioning/center_float_vector` | bounding-box center, float-to-Z-zero, and vector translation on `TriangleMesh` |
| `mesh_distribution/arc_linear_grid` | arc, linear, and grid distribution, including the transforms and unions used to materialize every copy |

CGAL's sphere/extrusion builders in the benchmark source deliberately construct
the prescribed polygon soup before promotion into `Surface_mesh`; this avoids
benchmarking an unrelated convenience primitive. OCCT uses analytic sphere and
prism construction because B-rep modeling is the kernel's native contract.
The sliver thickness is deliberately ten times OCCT's `1e-7` confusion
tolerance. It probes the tight end of OCCT's documented modeling contract while
remaining an exact rational construction in csgrs and CGAL EPECK.
