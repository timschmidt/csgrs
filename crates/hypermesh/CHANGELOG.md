# Changelog

## [0.1.0] - Unreleased

Initial release.

Hypermesh provides exact Boolean operations on 3D triangle meshes for the
Hyper geometry stack. It validates finite, closed piecewise-winding-number
(PWN) meshes, builds one exact surface arrangement for all requested
expressions, and returns closure-certified triangle boundaries. Coordinates
remain `hyperreal::Real` values throughout; file formats and solid-modeling
grammar belong to adapters such as csgrs.

- `boolean` evaluates a `BooleanProgram`: one variadic union, intersection,
  difference, or symmetric difference, or a topologically ordered
  `BooleanExpression` program with several output roots. All roots share one
  set of intersections, cells, windings, and exact vertices in a
  `BooleanMeshBatch`.
- Every predicate-bearing operation takes a `MeshContext` with an explicit
  Hyperlimit `PredicatePolicy` (`STRICT` or `APPROXIMATE_512`). Results are
  `MeshOutcome<T>` values whose `MeshCertainty` reports whether the 512-bit
  approximate terminal decision was consumed.
- Disconnected, nested, coincident, balanced nonmanifold, and
  winding-multiplicity inputs are supported. Invalid indices, degenerate or
  open meshes, non-PWN input, malformed programs, policy-undecidable
  predicates, and invalid output are typed `HypermeshError` failures; empty
  results are valid.
- `BooleanMeshBatch::into_triangle_meshes` returns `UnboundedBooleanOutput`
  for results that contain the exterior (for example, from negation) instead
  of presenting them as finite solids.
- Boolean results can be fed into further Booleans. An unedited result
  retains that its triangles meet only in shared vertices or edges, so a later
  Boolean does not test it against itself; the published rows are the same as
  with the full self test. Crossings on an axis-aligned plane with rational
  coefficients store that coordinate exactly, so a later cut in the same plane
  decides even when the crossed edges have symbolic coordinates.
- Boolean results keep only the vertices their represented set needs.
  Coplanar pieces of each result are grouped into plane classes from
  coplanarity the arrangement already decided or that an earlier result
  retained, without new plane tests. A vertex inside one class, or on a
  straight crease between two classes, is removed and the space around it is
  retriangulated by Hypertri under the operation's policy; corners, crease
  ends and non-manifold contacts stay. Represented sets, closure, orientation,
  the embedded-complex fact and certainty are unchanged, and no vertex is
  added. Sixteen square holes through a block give 332 triangles instead of
  1,600, and later cuts no longer interpolate new vertices from earlier
  crossings, so sampled-trigonometric drill chains keep constant coordinate
  size. A result triangle covering several coplanar input triangles names the
  least of them in `TriangleSource`. Certified results retain their plane
  classes for the next Boolean. This changes published rows; see the csgrs
  migration notes.
- `minkowski_sum` computes the regularized Minkowski sum of bounded
  closed-PWN solids; `minkowski_sum_with_options` adds count limits and
  cooperative cancellation, which do not bound total work or memory.
  `minkowski_hull` computes the convex hull of pairwise point sums.
- Further exact operations include `convex_hull`, oriented horizontal
  sections with source provenance (`TriangleMesh::slice_z`),
  `TriangleMesh::validate_closed_pwn`, `TriangleMesh::try_concatenate`,
  `intersect_polygons`, and the `ExactBvh` and `ExactPointBvh` acceleration
  structures.
- `surface_nets` and `surface_nets_standalone` build Surface Nets meshes
  directly from `Real` samples on a regular grid. The reported certainty
  covers the sampled construction only; it does not certify that the grid
  captures the topology of the underlying continuous field.
- `f32` and `f64` GPU buffers are explicit approximate export boundaries.
  Exact render-row export reports degenerate triangles and undecided normals
  as distinct `GpuMeshError` variants and publishes no partial buffer.
- Strict convex hulls and Booleans of sampled spheres complete: their
  coplanar quads, collinear plane crossings and coincidental coplanarities are
  exact zeros that hyperreal's cyclotomic zero proof decides.
- There are no default features; `dispatch-trace` enables correlated
  Hyperreal, Hyperlattice, and Hyperlimit dispatch tracing.
- Requires Rust 1.95.0 or later.
