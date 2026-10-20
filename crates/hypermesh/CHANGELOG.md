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
- There are no default features; `dispatch-trace` enables correlated
  Hyperreal, Hyperlattice, and Hyperlimit dispatch tracing.
- Requires Rust 1.95.0 or later.
