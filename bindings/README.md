# csgrs language bindings

Language bindings sit above `csgrs-ffi`, not above the geometry core. This keeps
the maintenance shape flat:

- Rust users choose `csgrs` for raw hyperreals or `csgrs-adapter` for scalar
  families.
- C users include `../ffi/include/csgrs.h` and link `csgrs-ffi`.
- C++ uses `cpp/csgrs.hpp`, a RAII wrapper over the C ABI.
- Python uses `python/csgrs.py`, a `ctypes` wrapper over the C ABI.
- Go uses `go/csgrs.go`, a `cgo` wrapper over the C ABI.
- JavaScript/TypeScript uses `js/csgrs.ts`, a typed wrapper around a native
  loader that exposes the C ABI operation names.

Every binding preserves the same scalar families:

- `f32`
- `f64`
- `i128`
- `real`

The outward API is intentionally generated-shaped. Add or rename operations in
the Rust adapter facade first, mirror them in `csgrs-ffi`, then update the thin
language wrappers. Do not put geometry logic in language bindings.
