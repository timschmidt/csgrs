# YeahRight competitive fixture

`yeahright_boolean_hull.obj` is the 1,128-triangle convex hull derived from
Keenan Crane's YeahRight model and used by the csgrs cross-kernel benchmark.
The competitive harness subdivides it deterministically to 4,512 triangles
before benchmarking mesh-carrier construction and clipping-box Booleans.

`controlmesh.obj` is the original 5,687-vertex, 5,845-polygon genus-131
control mesh. Fan triangulation produces 11,894 triangles. It remains an
always-on full-resolution carrier/import test and benchmark, and an ignored
rotated-copy intersection preserves the explicit memory-ceiling hard test.

Keenan Crane released the original YeahRight model and all of its meshes into
the public domain.
