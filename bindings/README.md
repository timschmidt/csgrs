# csgrs language bindings

Language bindings sit above `csgrs-ffi`, not above the geometry core. This keeps
the maintenance shape flat:

- Rust users choose exact core types or the namespaced `csgrs::adapter` scalar
  families from the same crate.
- C users include the sibling `csgrs-ffi/include/csgrs.h` header and link
  `csgrs-ffi`.
- C++ uses `cpp/csgrs.hpp`, a RAII wrapper over the C ABI.
- Python uses `python/csgrs.py`, a `ctypes` wrapper over the C ABI.
- Go uses `go/csgrs.go`, a `cgo` wrapper over the C ABI.
- JavaScript/TypeScript uses `js/csgrs.ts`, a typed wrapper around a native
  loader that exposes the C ABI operation names.

Browser and Node.js consumers can instead use the generated
[WebAssembly bindings](js/WASM.md), with named certainty receipts and explicit
point-query status. These wrappers are generated directly from the `wasm`
feature and do not require a C-ABI loader.

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
cargo build --release --manifest-path crates/csgrs-ffi/Cargo.toml
```

Then validate the wrapper you ship:

- C++: include `cpp/csgrs.hpp`, point the compiler at
  `crates/csgrs-ffi/include`, link `csgrs_ffi`, and compile a program that
  constructs and frees at least one object;
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

Planar transforms share the native matrix contract. A matrix must act as a
nonsingular affine map on the XY plane, preserve that plane, and have constant
nonzero homogeneous weight; the weight is divided out. Perspective and
out-of-plane maps return `UNSUPPORTED` (7); constant zero weight with a nonzero
coordinate image returns `UNBOUNDED` (9); singular planar maps or an all-zero
homogeneous image return `VALIDATION_FAILED` (4).
Unresolved predicates remain `PREDICATE_UNDECIDED` (5), with diagnostic text.
These checks also apply to empty regions. C++/Python exceptions and Go errors
retain the C status. Scalar conversion still has its own failure category.

Three-component planar transform methods now validate every component and
preserve its meaning. Nonzero Z translation and rotations leaving the XY plane
are rejected. Z scale has no effect on XY geometry. Plane-preserving X/Y
half-turns perform their actual reflections; rotation order is X, then Y, then Z.
These corrections replace the earlier ignored components without changing
binding signatures. WASM transforms expose matching `GeometryError.code`
categories. Strict WASM membership returns no value only at a proved boundary;
an undecided query throws `GeometryError` with code `undecided`.

Planar square, rectangle, circle and polygon constructors preserve native
failure through C status codes, C++/Python exceptions and Go errors. Negative
dimensions, invalid resolutions and malformed polygon rings no longer publish
empty model handles. A certified zero-size primitive is successful empty
geometry after every argument has been checked. WASM's corresponding
constructors (also ellipse, right triangle, regular polygon, ring and crescent)
throw `GeometryError` with a machine-readable `code`. Signatures at these
language boundaries are unchanged.

Bounds are optional. The C ABI returns `CSGRS_STATUS_NO_BOUNDS` (10) for a
completed empty query and leaves the output record untouched. C++ returns
`std::optional`, Python returns `None`, and Go returns a nil bounds pointer with
a nil error. Nonempty results retain their previous coordinate types. Invalid
input and unresolved predicates remain errors; Go's `StatusError` retains both
the C status and diagnostic message. Go callers must now check the bounds
pointer before dereferencing it. Native JavaScript loaders must map `NO_BOUNDS`
to absence and preserve all other status categories; the loader implementation
is supplied by the consumer and is not included here.

Opaque handles may cross only the ABI version that created them. Callers must
use the matching free function, must not mix scalar families, and must copy any
borrowed last-error text before another ABI call. The wrappers are currently
developed with the repository and do not promise independent semantic
versioning.

All binding code follows the CSGRS MIT license.
