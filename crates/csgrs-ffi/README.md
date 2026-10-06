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

Build the dynamic and static libraries from the crate checkout:

```sh
cargo build --release --locked
```

Repository development uses the same sibling checkout layout as CSGRS: place
`csgrs-ffi`, `csgrs`, and the Hyper geometry crates beside one another. Published
packages use the matching version requirements from `Cargo.toml`.

The artifacts are named `libcsgrs_ffi` on Unix-like targets. Include
`include/csgrs.h`, link the library appropriate for the target, and keep
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
- The last-error pointer is borrowed, thread-local, and valid only until the
  next ABI call on that thread.

## Validation and license

```sh
cargo test --locked
cargo clippy --locked --all-targets -- -D warnings
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
