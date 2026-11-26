# Cross-kernel benchmark coverage

`kernel_comparison` groups nominal solid-kernel scenarios across CSGRS, CGAL EPECK
and OpenCascade. The target comparison contract requires every `kernel`, `precision`
and `corpus` workload in all three runners. `benchmarks/summarize.py --require-engine-parity`
checks engine-label sets for observed keys; it cannot find workloads absent from
all inputs or validate equivalent geometry/work. Opt-in `stress`
and `dangerous` rows are diagnostic ceilings outside the portable parity set.

## Represented scenario families

| CSGRS surface | Cross-kernel workloads |
|---|---|
| Primitive construction | cube, cuboid, cylinder, frustum, octahedron, icosahedron, sphere at three resolutions, ellipsoid, torus |
| Solid transforms | translation, XYZ rotation, uniform and nonuniform scale, mirror, arbitrary affine transform, centering, orientation inversion |
| Regularized Booleans | union, difference, intersection, and XOR on sphere/box; disjoint, contained, face-touching, identical, and exact-sliver topology classes |
| CurveRegion2-to-solid modeling | 64-segment circle extrusion |
| Multi-instance modeling | linear, grid, and landmark-angle arc distribution |
| TriangleMesh refinement and normalization | triangulation, one-level triangle subdivision, normal reconstruction, finite-coordinate materialization |
| Spatial and physical queries | bounding box, mass properties, point containment, ray intersections, polyline intersections, dihedral angle |
| Topology and carrier queries | vertex enumeration, graphics buffers, connectivity, manifold validation |
| Interchange | binary STL writing; optional arbitrary-polygon OBJ corpus ingestion |
| Deterministic complex corpus | OBJ import, translation, bounds, graphics buffers, connectivity, and manifold validation for an 8,020-triangle concave labyrinth and a 36,096-triangle level-3 Sierpiński foam |
| Optional external corpus | full-resolution transform, bounds, carrier, connectivity, and manifold checks; combined clipping-box Booleans on a generated convex hull |

The current runners do not establish equivalent inputs or operation semantics for
every row. CSGRS boxes are origin-anchored and native boxes centered, affecting
sphere/box overlap and contained-box subtraction. Sphere sampling axes/arithmetic
differ, and OCCT uses analytic geometry. Native graphics rows only count corners;
OCCT subdivision remeshes, while native identical-box intersection uses a shortcut.
The table is a scenario index, not certified API coverage. See
[workload definitions](WORKLOADS.md) for current per-row differences. Output counts
and matching labels cannot discharge the repository's matched-contract requirement.

The full external-corpus intersection remains a disabled-by-default
`dangerous` tier. It additionally requires `YEAHRIGHT_BENCH=1`; enable it only
on a disposable, memory-limited benchmark host.

## Deliberately outside the three-kernel matrix

These surfaces require feature-specific tests or benchmarks outside this matrix.
The table explains intended scope; it does not certify that every named API exists,
is currently exercised, or completes. Native placeholders must not be interpreted
as equivalent operations when their implemented work differs.

| CSGRS-only or nonportable surface | Reason it is not a three-kernel workload |
|---|---|
| Metadata filtering, mapping, and source-face propagation | Native runners do not implement the CSGRS provenance contract |
| Hyperreal expression evaluation and finite-boundary conversion | Scalar/exactness substrate rather than a shared solid-kernel operation |
| HyperMesh exact metadata-preserving output adaptation | No matched source-metadata adaptation is implemented in the native runners |
| Convex hull and Minkowski sum | No corresponding OCCT workload is implemented here |
| Laplacian/Taubin smoothing | No common operator with matching boundary and weighting semantics |
| Loft, twist, revolve, and path sweep | No single high-level operator with matching profile, cap, seam, and discretization contracts across all three kernels |
| Gear, egg, teardrop, arrow, text, and raster-derived constructors | Domain constructors, not common kernel primitives |
| CurveRegion2 Boolean, offset, flatten and slice APIs; planned skeleton work | Planar/profile topology has no common implementation in this 3D kernel harness |
| SDF, TPMS, and metaball meshing | Sampling algorithms rather than shared boundary-representation kernel calls |
| AMF, DXF, Gerber, glTF, PLY, SVG, and application adapters | Optional format or application layers; STL and the optional OBJ corpus case are the shared interchange cases |
| Bevy, WASM, and JavaScript adapters | Host integration rather than geometric-kernel work |

An operation can enter a matched comparison after its input sets, semantics,
certainty, validation and measured materialization agree. Engine-label parity is
then an additional presence check, not the proof of that agreement.
