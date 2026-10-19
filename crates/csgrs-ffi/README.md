# csgrs FFI

`csgrs-ffi` exposes CSGRS native triangle and filled-region operations through
a stable-shape C ABI. It is the shared binding spine for C, C++, Go, Python,
and native JavaScript/TypeScript loaders.

The ABI uses opaque handles for triangle meshes, filled curve regions, and raw
Hyperreal values. Scalar families are selected by function prefix:

- `csgrs_triangle_mesh_f32_*`
- `csgrs_triangle_mesh_f64_*`
- `csgrs_triangle_mesh_i128_*`
- `csgrs_triangle_mesh_real_*`
- `csgrs_curve_region_f32_*`
- `csgrs_curve_region_f64_*`
- `csgrs_curve_region_i128_*`
- `csgrs_curve_region_real_*`

All geometry is Hyperreal-backed internally. Primitive values are converted
only at ABI ingress and egress.

The public C declarations live in [`include/csgrs.h`](include/csgrs.h).

## Build and link

This README describes `csgrs-ffi 0.1.0` in the
[CSGRS source checkout](../../readme.md). Build the dynamic and static libraries
from the monorepo root:

```sh
cargo build -p csgrs-ffi --release --locked
```

The crate lives in `crates/csgrs-ffi/`; its dependencies use the matching
monorepo sources. Current source APIs are not a claim of registry availability.

The artifacts are named `libcsgrs_ffi` on Unix-like targets. Include
`crates/csgrs-ffi/include/csgrs.h` from the monorepo root, link the library appropriate for the target, and keep
the Rust dynamic library discoverable at runtime when using the shared form.

## API shape

- `CsgrsStatus` is returned by fallible operations. Read
  `csgrs_last_error_message()` immediately after a failure on the same thread.
- `CsgrsScalarFamily` records the boundary scalar family of a geometry handle.
- `CsgrsReal`, `CsgrsTriangleMesh`, and `CsgrsCurveRegion` are opaque. Never inspect,
  allocate, or copy their storage from C.
- `CsgrsVec*`, `CsgrsAabb3*`, `CsgrsMatrix4*`, `CsgrsTriangleU32`, and
  graphics-buffer records are value ABI types defined by the header.
- Constructors and operations write handles or value records through explicit
  output pointers.

The header groups scalar conversion, triangle-mesh and curve-region lifetime,
constructors, transforms, Booleans, bounds, indexed/graphics buffers,
finite region projections, and curve-to-solid operations.

## Safety, ownership, and error rules

- Input pointers must be null where allowed or valid for every documented read;
  output pointers must be valid for one write.
- Array pointers may be null only for a zero-length input. Polyhedron face
  offsets must cover the complete index buffer from zero through its length.
- Free every successful `CsgrsReal`, `CsgrsTriangleMesh`, and `CsgrsCurveRegion` handle
  exactly once with its matching `*_free` function.
- Free heap-backed real-valued bounds, buffers, graphics meshes, and region
  projections with the matching generated free function.
- Do not mix scalar families in one operation; query a handle with
  `csgrs_triangle_mesh_family` or `csgrs_curve_region_family` when its origin is uncertain.
- Float ingress rejects non-finite values. Integer and opaque-real egress
  remains fallible when a value cannot be represented.
- A non-success status leaves the documented output unowned.
- Mesh primitive and planar square/rectangle/circle/polygon constructors distinguish invalid input (`VALIDATION_FAILED`),
  undecided predicates (`PREDICATE_UNDECIDED`), resource/capacity failure
  (`RESOURCE_EXHAUSTED`), unsupported representations (`UNSUPPORTED`), and other
  native geometry failure (`GEOMETRY_FAILED`). These statuses are additive;
  existing numeric values are unchanged. Proven zero-size solids succeed as
  empty geometry after all constructor parameters have been validated.
  Nonnegative zero-size planar primitives also succeed as canonical empty
  regions; an empty or malformed polygon ring fails. Resolution and companion
  dimensions are validated before a zero-size shortcut. No ABI signature or
  numeric status changed in this planar failure migration.
- Mesh transforms retain native failure categories. A projective image that
  reaches or crosses infinity reports `UNBOUNDED` (9). Invalid transforms and
  undecided predicates remain distinct failures, and publish no model handle.
- Mesh union, difference, intersection and xor retain native failure categories.
  Undecided Booleans now report `PREDICATE_UNDECIDED` (5), replacing the previous
  generic `VALIDATION_FAILED` (4); invalid inputs still report (4), even when
  paired with empty geometry. Diagnostic text is retained and failed Booleans
  publish no model handle. Signatures and numeric status values are unchanged.
- Mesh and curve bounds return `NO_BOUNDS` (10) for a completed empty query.
  They require a valid output pointer but leave its record untouched; no
  opaque-real endpoints are allocated. Only `OK` writes an owned bounds record.
  Nonempty zero-size bounds, invalid input, and undecided predicates remain
  distinct. Centering and floating preserve canonical empty geometry and
  propagate bounds errors without publishing a model handle.
- C++ exceptions retain the C status in `error::status()`; Python exceptions
  retain it in `CsgrsError.status`; Go retains it in `StatusError.Status`.
  WASM primitive constructors, transforms, bounds, and positioning throw a
  `GeometryError` with a string `code` (`invalid_input`, `undecided`,
  `resource_exhausted`, `unsupported`, `unbounded`, or `geometry_failure`) and a diagnostic
  message. Other binding operations that still flatten native error categories
  remain outside this migration.
- The last-error pointer is borrowed, thread-local, and valid only until the
  next ABI call on that thread.

## Validation and license

```sh
cargo test -p csgrs-ffi --locked
cargo clippy -p csgrs-ffi --locked --all-targets -- -D warnings
```

Binding smoke tests in the
[`csgrs` crate](https://github.com/timschmidt/csgrs)
exercise the same header. The crate is MIT-licensed, matching CSGRS.

## References

- The checked-in [`include/csgrs.h`](include/csgrs.h) is the normative ABI
  declaration for this crate.
- The [CSGRS API documentation](https://docs.rs/csgrs) defines the geometry
  behavior exposed by the ABI.
- ISO/IEC 9899 (C) defines the language-level calling declarations used by the
  public header; platform ABI documents govern concrete layout and linkage.

## Acknowledgements

This crate is maintained with CSGRS by Timothy Schmidt and packages the work
of the CSGRS and Hyper geometry contributors behind a language-neutral
boundary.

## Extension protocol

Extensions follow the stack-wide [admission protocol](../../docs/admission.md).
It defines the exactness, certainty, boundary, cache, validation, and evidence
requirements shared by the Hyper crates.
