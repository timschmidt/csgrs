# csgrs FFI

`csgrs-ffi` exposes the Rust scalar adapter facade through a C ABI. It is the
shared binding spine for C, C++, Go, Python, and native JavaScript/TypeScript
loaders.

The ABI uses opaque handles for meshes, profiles, and raw hyperreals. Scalar
families are selected by function prefix:

- `csgrs_mesh_f32_*`
- `csgrs_mesh_f64_*`
- `csgrs_mesh_i128_*`
- `csgrs_mesh_real_*`
- `csgrs_profile_f32_*`
- `csgrs_profile_f64_*`
- `csgrs_profile_i128_*`
- `csgrs_profile_real_*`

All handles still contain hyperreal-backed `csgrs` core objects. Primitive
values are converted only at ABI ingress and egress.

The public C declarations live in [`include/csgrs.h`](include/csgrs.h).

## Build and link

Build the dynamic and static libraries from the CSGRS checkout:

```sh
cargo build --release --manifest-path ffi/Cargo.toml
```

The artifacts are named `libcsgrs_ffi` on Unix-like targets. Include
`ffi/include/csgrs.h`, link the library appropriate for the target, and keep
the Rust dynamic library discoverable at runtime when using the shared form.

## API shape

- `CsgrsStatus` is returned by fallible operations. Read
  `csgrs_last_error_message()` immediately after a failure on the same thread.
- `CsgrsScalarFamily` records which scalar facade owns a mesh or profile.
- `CsgrsReal`, `CsgrsMesh`, and `CsgrsProfile` are opaque. Never inspect,
  allocate, or copy their storage from C.
- `CsgrsVec*`, `CsgrsAabb3*`, `CsgrsMatrix4*`, `CsgrsTriangleU32`, and
  graphics-buffer records are value ABI types defined by the header.
- Constructors and operations write handles or value records through explicit
  output pointers.

The header groups scalar construction/conversion, mesh/profile lifetime,
primitives, transforms, Booleans, bounds, indexed and graphics buffers,
profile regions, and profile-to-solid operations.

## Safety, ownership, and error rules

- Input pointers must be null where allowed or valid for every documented read;
  output pointers must be valid for one write.
- Free every successful `CsgrsReal`, `CsgrsMesh`, and `CsgrsProfile` handle
  exactly once with its matching `*_free` function.
- Free heap-backed real-valued bounds, buffers, graphics meshes, and region
  projections with the matching generated free function.
- Do not mix scalar families in one operation; query a handle with
  `csgrs_mesh_family` or `csgrs_profile_family` when its origin is uncertain.
- Float ingress rejects non-finite values. Integer and opaque-real egress
  remains fallible when a value cannot be represented.
- A non-success status leaves the documented output unowned.
- The last-error pointer is borrowed, thread-local, and valid only until the
  next ABI call on that thread.

## Validation and license

```sh
cargo test --manifest-path ffi/Cargo.toml
cargo clippy --manifest-path ffi/Cargo.toml --all-targets -- -D warnings
```

Binding smoke tests under [`../bindings`](../bindings/README.md) exercise the
same header. The crate is MIT-licensed, matching CSGRS.
