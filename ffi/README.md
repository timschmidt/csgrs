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
