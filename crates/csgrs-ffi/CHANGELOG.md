# Changelog

## [0.1.0] - Unreleased

Initial release.

`csgrs-ffi` exposes CSGRS triangle-mesh and filled-region operations through a
C ABI declared in `include/csgrs.h`, for C, C++, Go, Python and native
JavaScript/TypeScript loaders. Geometry is Hyperreal-backed internally;
primitive values are converted only at ABI ingress and egress. The crate
builds `cdylib`, `staticlib` and `rlib` artifacts and requires Rust 1.95.0 or
later.

- Four scalar families selected by function prefix: `f32`, `f64`, `i128` and
  opaque `real` values (`csgrs_triangle_mesh_<family>_*` and
  `csgrs_curve_region_<family>_*`). Passing a handle of another family
  reports `CSGRS_STATUS_WRONG_SCALAR_FAMILY`.
- Opaque `CsgrsReal`, `CsgrsTriangleMesh` and `CsgrsCurveRegion` handles,
  each freed once with its matching `*_free` function, plus value records
  for vectors, matrices, bounds, triangles and graphics buffers.
- Triangle meshes: cube, cuboid, sphere, cylinder and polyhedron
  constructors; union, difference, intersection and xor; transform,
  translate, scale, rotate, inverse, center and float; bounds, indexed
  buffers and graphics meshes.
- Filled regions: square, rectangle, circle and polygon constructors;
  Booleans; transforms; bounds; finite region profiles; and extrude, vector
  extrude and revolve into triangle meshes.
- Fallible calls return `CsgrsStatus`. Invalid input, undecided predicates,
  resource exhaustion, unsupported representations, other geometry failures
  and unbounded transform images have distinct statuses, and a completed
  bounds query on empty geometry reports `CSGRS_STATUS_NO_BOUNDS`. A failed
  call publishes no output handle.
- `csgrs_last_error_message()` returns a borrowed, thread-local diagnostic
  valid until the next ABI call on that thread or thread exit. Status-returning
  calls catch unwinding panics in their actions as `CSGRS_STATUS_PANIC`;
  aborts, invalid-pointer access and failures in diagnostic formatting are
  outside that catcher, and free functions have no unwind wrapper.
- Float ingress rejects non-finite values and preserves finite binary values
  exactly. Float egress is lossy; integer egress requires a recognized in-range
  integer. Real mesh-coordinate export clones native scalars, while region
  profiles pass through sampled binary64 points even in the real family.
- The C++, Python and Go wrappers in the CSGRS repository's `bindings/`
  directory keep the C status in their exceptions or errors.
