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
at the adapter boundaries. Float ingress preserves the exact finite binary
value: `0.1_f64` is not one tenth. Float exports are lossy projections; integer
exports require a recognized in-range integer. Opaque real mesh-coordinate
exports retain native scalars, but region profiles first pass through sampled
binary64 points in **every** scalar family. Those profiles are display/interchange
output, not exact native curve replay.

Geometry and validation remain in the shared scalar adapters and Hyper kernels.
The ABI has no policy selector or separate operation-local certainty receipt.
Most decision-bearing paths use strict predicates; region Booleans use the active
Hypercurve scope, strict outside a preview/provisional scope. Mesh construction
history is available through `csgrs_triangle_mesh_construction_evidence`; it does
not establish topology or analytic-source fidelity. Extrusion, revolution and
profile extraction discard their separate outcome receipts at this boundary.

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
  output pointers must be valid for one write without aliasing borrowed inputs.
  Non-null addresses, alignment, allocation extent and lifetimes are caller
  obligations. Null checks and unwind catching do not validate them.
- Array pointers may be null only for a zero-length input. Polyhedron face
  offsets must cover the complete index buffer from zero through its length.
- Free every successful `CsgrsReal`, `CsgrsTriangleMesh`, and `CsgrsCurveRegion` handle
  exactly once with its matching `*_free` function.
- Free heap-backed real-valued bounds, buffers, graphics meshes, and region
  projections with the matching generated free function.
  Primitive-family buffer arrays also require their matching free function.
  Preserve the returned pointers and element counts; copying a value record
  does not duplicate ownership. Real components supplied as inputs are borrowed.
- Do not mix scalar families in one operation; query a handle with
  `csgrs_triangle_mesh_family` or `csgrs_curve_region_family` when its origin is uncertain.
- Float ingress rejects non-finite values. Integer egress can fail on
  noninteger, unrecognized or out-of-range values. Real scalar export clones
  native values, though the preceding geometry/projection operation can fail.
- `OK` transfers any new handle/buffer ownership and overwrites the output slot
  without releasing a previous value. Ordinary errors leave the slot untouched;
  a caller's pre-existing ownership remains theirs. Use only successful records
  with the matching free function.
- Mesh primitive and planar square/rectangle/circle/polygon constructors preserve
  native invalid, undecided, resource, unsupported and other geometry failure
  categories. Proven zero-size solids succeed as empty geometry after all
  constructor parameters have been validated.
  Nonnegative zero-size planar primitives also succeed as canonical empty
  regions; an empty or malformed polygon ring fails. Resolution and companion
  dimensions are validated before a zero-size shortcut.
- Unavailable storage or unaddressable storage dimensions report
  `RESOURCE_UNAVAILABLE` (11), separately from a reached configured allowance
  (`RESOURCE_EXHAUSTED`, 6).
  Neither failure publishes a partial model; a larger-budget or later retry
  remains possible where its native cause permits it.
- Mesh transforms retain native failure categories. A projective image that
  reaches or crosses infinity reports `UNBOUNDED` (9). Invalid transforms and
  undecided predicates remain distinct failures, and publish no model handle.
- Mesh union, difference, intersection and xor retain native failure categories.
  Undecided Booleans report `PREDICATE_UNDECIDED` (5); invalid inputs report
  `VALIDATION_FAILED` (4), even when
  paired with empty geometry. Diagnostic text is retained and failed Booleans
  publish no model handle.
- Curve union, difference, intersection, xor, revolve and profile extraction
  also retain native failure categories. Undecided operations report
  `PREDICATE_UNDECIDED` (5). Invalid revolve parameters and non-finite profile
  projections report (4). Failures retain diagnostic text and leave output
  handles and profile records untouched.
- Mesh and curve bounds return `NO_BOUNDS` (10) for a completed empty query.
  They require a valid output pointer but leave its record untouched; no
  opaque-real endpoints are allocated. Only `OK` writes an owned bounds record.
  Nonempty zero-size bounds, invalid input, and undecided predicates remain
  distinct. Centering and floating preserve canonical empty geometry and
  propagate bounds errors without publishing a model handle.
- Polyhedron and graphics-export errors preserve native categories. Undecided
  triangulation and normal certification return `PREDICATE_UNDECIDED` (5).
  Invalid rings, indices and proven degenerate normals report (4).
  Failure leaves output handles and buffer
  records untouched; successful empty graphics buffers have zero row counts.
- C++ exceptions retain the C status in `error::status()`; Python exceptions
  retain it in `CsgrsError.status`; Go retains it in `StatusError.Status`.
  WASM primitive constructors, transforms, bounds, and positioning throw a
  `GeometryError` with a string `code` (`invalid_input`, `undecided`,
  `resource_exhausted`, `resource_unavailable`, `unsupported`, `unbounded`, or `geometry_failure`) and a diagnostic
  message. Some other WASM operations still flatten native error categories;
  see the separate [WASM guide](../../bindings/js/WASM.md).
- The last-error pointer is borrowed, thread-local, and valid only until the
  next ABI call on that thread or thread exit. Status-returning calls replace
  the diagnostic on success as well as failure; free functions leave it unchanged.
- Status-returning calls catch unwinding panics in their actions. Aborts,
  allocation aborts, invalid memory access and failures in error reporting are
  not caught; free functions have no unwind wrapper. There is no ABI-wide
  work, memory or cancellation budget.

Boxes and rectangles begin at the origin. Sphere/cylinder/circle constructors
produce sampled polygonal geometry, with no implied analytic fidelity bound.
Cylinder height is signed. `inverse` reverses triangle winding rather than
constructing an unbounded complement. Revolution is segmented about Y;
`revolve_solid` regularizes the sampled material while admitting closed PWN
point/edge contacts, which need not have a manifold boundary.

## Validation and license

```sh
cargo test -p csgrs-ffi --locked
cargo clippy -p csgrs-ffi --locked --all-targets -- -D warnings
```

The repository's [release-consumer checks](../../scripts/release-check.sh)
compile/link C and C++ consumers of this header and exercise native bindings.
Those checks are separate from `cargo test -p csgrs-ffi`. The crate is
MIT-licensed, matching CSGRS.

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
