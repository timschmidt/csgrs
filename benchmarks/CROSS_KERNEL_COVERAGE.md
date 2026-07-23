# Cross-kernel benchmark coverage

`kernel_comparison` measures the portable solid-kernel surface shared by
CSGRS, CGAL EPECK, and OpenCascade. Every `kernel`, `precision`, and `corpus`
workload is required to occur in all three runners; `benchmarks/summarize.py
--require-engine-parity` rejects missing or extra engine rows. Opt-in `stress`
and `dangerous` rows are diagnostic ceilings outside the portable parity set.

## Covered portable API

| CSGRS surface | Cross-kernel workloads |
|---|---|
| Primitive construction | cube, cuboid, cylinder, frustum, octahedron, icosahedron, sphere at three resolutions, ellipsoid, torus |
| Solid transforms | translation, XYZ rotation, uniform and nonuniform scale, mirror, arbitrary affine transform, centering, orientation inversion |
| Regularized Booleans | union, difference, intersection, and XOR on sphere/box; disjoint, contained, face-touching, identical, and exact-sliver topology classes |
| Profile-to-solid modeling | 64-segment circle extrusion |
| Multi-instance modeling | linear, grid, and landmark-angle arc distribution |
| Mesh refinement and normalization | triangulation, one-level triangle subdivision, normal reconstruction, finite-coordinate materialization |
| Spatial and physical queries | bounding box, mass properties, point containment, ray intersections, polyline intersections, dihedral angle |
| Topology and carrier queries | vertex enumeration, graphics buffers, connectivity, manifold validation |
| Interchange | binary STL writing; YeahRight arbitrary-polygon OBJ ingestion with scientific-notation coordinates |
| YeahRight corpus | full-resolution transform, bounds, carrier, connectivity, and manifold checks; combined clipping-box Booleans on a 1,128-triangle convex-hull proxy |

The runners use equivalent geometric inputs and operation semantics. Output
facet counts are reported but are not required to match because OpenCascade
tessellates analytic B-reps, CGAL operates on exact `Surface_mesh`, and CSGRS
retains its exact polygon mesh.

The full 11,894-triangle-per-operand YeahRight intersection is retained as the
disabled-by-default `dangerous` tier. Enable it only with
`CSGRS_BENCH_ENABLE_DANGEROUS=1`; it has been observed to consume about 116 GiB
RSS before the Linux OOM killer terminated CSGRS. The opt-in `stress` tier uses
the committed 1,146-triangle proxy, whose Euler characteristic preserves the
source model's genus 131; enable it with `CSGRS_BENCH_ENABLE_STRESS=1`.

## Deliberately outside the three-kernel matrix

These APIs remain exercised by `feature_pipeline`, tests, or dependency
benchmarks. They are not assigned misleading native no-ops where one of the
comparison kernels has no corresponding abstraction.

| CSGRS-only or nonportable surface | Reason it is not a three-kernel workload |
|---|---|
| Metadata filtering, mapping, and source-face propagation | CGAL and OpenCascade carriers do not expose CSGRS metadata semantics |
| Hyperreal expression evaluation and finite-boundary conversion | Scalar/exactness substrate rather than a shared solid-kernel operation |
| HyperMesh exact metadata-preserving output adaptation | No equivalent source-metadata contract exists in both native kernels |
| Convex hull and Minkowski sum | CGAL has direct facilities, but the installed OpenCascade modeling API has no corresponding general mesh operator |
| Laplacian/Taubin smoothing | No common operator with matching boundary and weighting semantics |
| Loft, twist, revolve, and path sweep | No single high-level operator with matching profile, cap, seam, and discretization contracts across all three kernels |
| Gear, egg, teardrop, arrow, text, and raster-derived constructors | Domain constructors, not common kernel primitives |
| Profile Boolean, offset, skeleton, flatten, and slice APIs | Planar/profile topology has no common representation in this 3D kernel harness |
| SDF, TPMS, and metaball meshing | Sampling algorithms rather than shared boundary-representation kernel calls |
| AMF, DXF, Gerber, glTF, PLY, SVG, and application adapters | Optional format or application layers; STL and the focused YeahRight OBJ ingestion case are the shared interchange cases |
| Bevy, WASM, and JavaScript adapters | Host integration rather than geometric-kernel work |

When a genuinely equivalent operation becomes available in all three kernels,
it should move into `kernel_comparison` and will then be protected by the
engine-parity check.
