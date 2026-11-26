# csgrs language bindings

Native language bindings sit above `csgrs-ffi`; geometry remains in the shared
Rust adapters and kernels:

- Rust users choose exact core types or the namespaced `csgrs::adapter` scalar
  families from the same crate.
- C users include [`crates/csgrs-ffi/include/csgrs.h`](../crates/csgrs-ffi/include/csgrs.h) and link
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

The native interfaces name four boundary scalar families:

- `f32`
- `f64`
- `i128`
- `real`

These labels do not establish identical operation coverage, input coercion or
ownership across wrappers. WASM has its own generated API, not this four-family
native-loader surface.

| Native wrapper | Current lifetime and conversion limits |
| --- | --- |
| C++ | Geometry/scalars use RAII. Exported buffers and real-valued bounds require manual native frees; copying their records does not duplicate ownership. |
| Python | Handles/buffers use best-effort finalizers, with no explicit close protocol. Real bounds are raw owning records. Integer-family coercion can truncate via `int` and wraps modulo 2^128; matrix packing bypasses that coercion. |
| Go | Owners require explicit `Free`; copying owning structs is unsafe. Real bounds have no Go free wrapper, and aliased C fields remain unexported. Several declared graphics/profile types have no matching operation wrappers. |
| Native TypeScript | Only a typed facade is provided. The consumer supplies marshalling, output slots, status/error translation and raw-buffer cleanup. `free()` neither clears handles nor prevents repeated calls; replacing the global loader does not migrate live handles. |

Float egress is lossy and integer egress can fail. Native region profiles pass
through sampled binary64 points even in the real family; they are not exact curve
replay. The C ABI has no policy selector or separate operation-local receipt.
Mesh construction history is queryable through C, C++ and Python, but does not
certify topology or fidelity; the current Go/native-TypeScript wrappers omit that
query. The generated WASM API exposes its own receipts and history accessors.

Add or rename operations in
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
  exercise construction, operations and ownership cleanup. Its automatic search
  still uses legacy `ffi/target/release` paths; finalizers are not an explicit
  close API;
- Go: make the header and library visible to cgo, add the package to a Go
  module, and run `go test`;
- JavaScript/TypeScript: provide a native loader implementing the C ABI names
  consumed by `js/csgrs.ts` with its logical argument/return protocol, then run
  `tsc --noEmit` and a loader integration test. A raw symbol lookup alone is
  insufficient: the facade omits C output pointers and array lengths and expects
  decoded results rather than status integers.

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
Unresolved predicates remain `PREDICATE_UNDECIDED` (5), with native diagnostic text.
These checks also apply to empty regions. C++/Python exceptions and Go errors
retain the C status. Scalar conversion still has its own failure category.
Go fetches thread-local text in a separate cgo call without OS-thread pinning;
without caller thread affinity, its copied message need not belong to the status.

Three-component planar transform methods validate every component and
preserve its meaning. Nonzero Z translation and rotations leaving the XY plane
are rejected. Z scale has no effect on XY geometry. Plane-preserving X/Y
half-turns perform their actual reflections; rotation order is X, then Y, then Z.
WASM transforms expose matching `GeometryError.code`
categories. Strict WASM membership returns no value only at a proved boundary;
an undecided query throws `GeometryError` with code `undecided`.

Planar square, rectangle, circle and polygon constructors preserve native
failure through C status codes, C++/Python exceptions and Go errors. Negative
dimensions, invalid resolutions and malformed polygon rings do not publish
empty model handles. A certified zero-size primitive is successful empty
geometry after every argument has been checked. WASM's corresponding
constructors (also ellipse, right triangle, regular polygon, ring and crescent)
throw `GeometryError` with a machine-readable `code`.

Bounds are optional. The C ABI returns `CSGRS_STATUS_NO_BOUNDS` (10) for a
completed empty query and leaves the output record untouched. C++ returns
`std::optional`, Python returns `None`, and Go returns a nil bounds pointer with
a nil error. Nonempty results use the selected family coordinates. Invalid
input and unresolved predicates remain errors; Go's `StatusError` retains the
numeric status with the diagnostic limitation above. Check the bounds pointer
before dereferencing it. Native JavaScript loaders must map `NO_BOUNDS`
to absence and preserve all other status categories; the loader implementation
is supplied by the consumer and is not included here.

Mesh union, difference, intersection and xor preserve native failure categories
through C status codes, C++/Python exceptions and Go errors. An undecided
Boolean returns `PREDICATE_UNDECIDED` (5). Invalid input reports
`VALIDATION_FAILED` (4), including malformed operands paired with empty geometry.
Failed operations publish no model handle.

Curve Booleans, revolve and profile extraction preserve the same native
categories. Undecided curve operations report status (5);
invalid parameters and non-finite profile projections remain (4). C++ and
Python expose those statuses with diagnostic text. Failures leave
output handles and profile records untouched. Rust scalar adapters retain
`AdapterError::Construction` and its native cause instead of a string-only
`AdapterError::Validation`.

Polyhedron construction and graphics export also preserve native failure
categories. Undecided triangulation or normal certification reports (5);
malformed rings, invalid indices and proven degenerate normals report (4).
Failures leave output handles and graphics records untouched. Successful empty
exports contain zero vertices and indices. Rust point-transform adapters retain their
original `hyperreal::Problem` through `AdapterError::Hyperreal`.

Opaque handles may cross only the ABI version that created them. Callers must
use the matching free function, must not mix scalar families, and must copy any
borrowed last-error text before another ABI call. The wrappers are currently
developed with the repository and do not promise independent semantic
versioning.

All binding code follows the CSGRS MIT license.
