# YeahRight competitive fixture

`yeahright_boolean_hull.obj` is the 1,128-triangle convex hull derived from
Keenan Crane's YeahRight model and used by the csgrs cross-kernel benchmark.
The competitive harness subdivides it deterministically to 4,512 triangles
before benchmarking mesh-carrier construction and clipping-box Booleans.

Keenan Crane released the original YeahRight model and all of its meshes into
the public domain.
