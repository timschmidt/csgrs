# YeahRight benchmark corpus

Keenan Crane released the YeahRight model and every mesh in the original
archive into the public domain. The original statement and dataset description
are preserved in `README.txt`; the five original OBJ source assets are
committed alongside it.

The cross-kernel benchmark uses:

- `controlmesh.obj` for exact OBJ ingestion, winding normalization, transforms,
  bounds, graphics buffers, connectivity, and manifold validation. It contains
  5,687 vertices and 5,845 arbitrary polygon faces, which triangulate to 11,894
  faces.
- `yeahright_boolean_hull.obj` for the portable combined clipping-box Boolean
  row. It is the convex hull of the 188,672-triangle tessellation and contains
  566 vertices, 1,692 edges, and 1,128 triangles.
- `controlmesh_boolean_proxy.obj` for the opt-in rotated-copy stress suite. It
  is a topology-preserving CGAL edge-collapse simplification of the
  oriented control mesh with 313 vertices, 1,719 edges, and 1,146 triangles.
  Its Euler characteristic remains -260, so its genus remains 131.
- `controlmesh.obj` itself for the opt-in `dangerous` full-resolution
  intersection. On the development host, CSGRS preparation reached about
  116 GiB RSS and was terminated by the Linux OOM killer. Do not enable this
  workload on a machine that cannot tolerate memory exhaustion.

The stress copy is rotated exactly 90 degrees around Y and translated by
`(1, 12, 1)`. The landmark rotation avoids introducing trigonometric sampling
into the exact kernels while producing a non-identical overlap.
