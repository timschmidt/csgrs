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

## csgrs-specific transform helpers

The broader `feature_pipeline` additionally times API helpers that do not have
a single equivalent operation in all three native kernels:

| Benchmark/case | Covered operations |
|---|---|
| `profile_transform/all_csg_helpers` | vector translation, Z rotation, non-uniform scaling, plane mirror, center, float, arbitrary affine transform, and inverse on `Profile` |
| `profile_distribution/arc_linear_grid` | arc, linear, and grid distribution on `Profile`, including profile unions |
| `mesh_positioning/center_float_vector` | bounding-box center, float-to-Z-zero, and vector translation on `Mesh` |
| `mesh_distribution/arc_linear_grid` | arc, linear, and grid distribution, including the transforms and unions used to materialize every copy |

CGAL's sphere/extrusion builders in the benchmark source deliberately construct
the prescribed polygon soup before promotion into `Surface_mesh`; this avoids
benchmarking an unrelated convenience primitive. OCCT uses analytic sphere and
prism construction because B-rep modeling is the kernel's native contract.
The sliver thickness is deliberately ten times OCCT's `1e-7` confusion
tolerance. It probes the tight end of OCCT's documented modeling contract while
remaining an exact rational construction in csgrs and CGAL EPECK.
