# csgrs language bindings

Language bindings sit above `csgrs-ffi`, not above the geometry core. This keeps
the maintenance shape flat:

- Rust users choose exact core types or the namespaced `csgrs::adapter` scalar
  families from the same crate.
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
`csgrs::adapter` first, mirror them in `csgrs-ffi`, then update the thin
language wrappers. Do not put geometry logic in language bindings.

## Build order

Build the ABI once before running native wrappers:

```sh
cargo build --release --manifest-path ../ffi/Cargo.toml
```

Then validate the wrapper you ship:

- C++: include `cpp/csgrs.hpp`, point the compiler at
  `../ffi/include`, link `csgrs_ffi`, and compile a program that constructs and
  frees at least one object;
- Python: set `CSGRS_LIBRARY` or pass the built library path to `load()`, then
  run a small construction/operation/free cycle;
- Go: make the header and library visible to cgo, add the package to a Go
  module, and run `go test`;
- JavaScript/TypeScript: provide a native loader implementing the C ABI names
  consumed by `js/csgrs.ts`, then run `tsc --noEmit` and a loader integration
  test.

Repository-level syntax checks that need no live native loader are:

```sh
python3 -m py_compile bindings/python/csgrs.py
c++ -std=c++17 -fsyntax-only -x c++ bindings/cpp/csgrs.hpp
tsc --noEmit --target ES2020 --strict bindings/js/csgrs.ts
```

The C header is the ABI authority. Language wrappers own lifetime convenience,
exceptions/results, and idiomatic naming only. Scalar conversion, Boolean
behavior, metadata rules, and topology stay in the Rust layers.

## Compatibility

Opaque handles may cross only the ABI version that created them. Callers must
use the matching free function, must not mix scalar families, and must copy any
borrowed last-error text before another ABI call. The wrappers are currently
developed with the repository and do not promise independent semantic
versioning.

All binding code follows the CSGRS MIT license.
