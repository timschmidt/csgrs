# csgrs adapters

`csgrs-adapter` is the primitive-scalar boundary crate for the hyperreal
`csgrs` core.

Use this crate when a Rust-facing API must accept `f32`, `f64`, or `i128`
without changing CSGRS's internal scalar model. Rust code that can use
`hyperreal::Real` directly should depend on `csgrs` instead.

## Installation

```toml
[dependencies]
csgrs-adapter = "0.1.0"
```

The rule is simple: `csgrs`, `hyperlattice`, `hyperlimit`, `hypercurve`,
`hypertri`, and `hypermesh` own geometry as raw `hyperreal::Real`. Primitive
types are accepted only by adapter crates and file/language IO boundaries. An
adapter converts values on ingress, calls the exact core, then converts results
on egress.

## Rust scalar adapters

The Rust facade is generic over a marker type:

```rust
use csgrs_adapter::{F64, Mesh};

type MeshF64 = Mesh<F64, ()>;

fn main() -> Result<(), csgrs_adapter::AdapterError> {
    let cube = MeshF64::cube(2.0, ())?;
    let shifted = cube.translate(1.0, 0.0, 0.0)?;
    let _bounds = shifted.bounding_box()?;
    Ok(())
}
```

The same aliases are exported directly as `MeshF32`, `MeshF64`, `MeshI128`,
`RawMesh`, `ProfileF32`, `ProfileF64`, `ProfileI128`, and `RawProfile`.

Run the example from a checkout with:

```sh
cargo run --manifest-path adapters/Cargo.toml --example basic
```

Available markers:

- `RawReal`: accepts and returns `hyperreal::Real` with no primitive conversion.
- `F32`: accepts finite `f32`, stores exact dyadic hyperreals, returns lossy finite `f32`.
- `F64`: accepts finite `f64`, stores exact dyadic hyperreals, returns lossy finite `f64`.
- `I128`: accepts exact `i128`, stores exact hyperreals, returns only exact in-range integers.

All wrapper types contain the raw core object:

```rust
use csgrs_adapter::{F64, Mesh};

fn main() -> Result<(), csgrs_adapter::AdapterError> {
    let mesh = Mesh::<F64, ()>::cube(1.0, ())?;
    let _raw: csgrs_adapter::core::mesh::Mesh<()> = mesh.into_raw();
    Ok(())
}
```

## Useful API

- scalar policy: `ScalarAdapter`, `RawReal`, `F32`, `F64`, `I128`,
  `AdapterError`, and `AdapterResult`;
- mesh facade: `Mesh<A, M>`, `RawMesh`, `MeshF32`, `MeshF64`, `MeshI128`,
  `MeshVertex`, `IndexedMeshBuffers`, and `GraphicsMesh`;
- profile facade: `Profile<A>`, `RawProfile`, `ProfileF32`, `ProfileF64`,
  `ProfileI128`, and `RegionProfile`;
- shared results: `Aabb3<S>` and the `core` re-export for deliberate access to
  raw CSGRS types.

The mesh facade covers construction, transforms, Booleans, bounds, topology,
graphics/indexed buffers, and raw-carrier conversion. The profile facade
covers construction, transforms, Booleans, region projection, extrusion,
revolution, sweep, loft, and raw-carrier conversion.

## Maintenance model

The adapter matrix should stay two-dimensional:

- Add a scalar by implementing `ScalarAdapter` once.
- Add a Rust operation by forwarding through `Mesh<A, M>` or `Profile<A>` once.
- Generate language bindings from the Rust adapter facade or a C ABI layer.

Do not add crates per scalar-language pair. The intended layout is:

```text
csgrs                 exact Rust core, raw hyperreals only
csgrs/adapters        Rust scalar adapter facade
csgrs/ffi             optional generated C ABI over adapters
bindings/cpp          C++ wrapper over the C ABI
bindings/go           Go wrapper over the C ABI
bindings/python       Python wrapper over the C ABI or PyO3 facade
bindings/js           wasm-bindgen/napi wrapper over the adapter facade
```

Language bindings should expose scalar families as feature-selected concrete
entry points, for example `csgrs_f64_mesh_translate` and
`csgrs_i128_mesh_translate`, while sharing the same generated operation list.
Raw hyperreals crossing a non-Rust FFI boundary should be opaque handles or
serialized values, never Rust struct layout.

## Conversion policy

- Float input rejects `NaN` and infinities through `hyperreal::Real::try_from`.
- Float input is exact dyadic import, not decimal interpretation.
- Float output is explicitly lossy and fails if no finite primitive value exists.
- Graphics export converts the core's retained HyperMesh render rows directly
  into the requested interleaved scalar layout without temporary split buffers.
- Integer input is exact.
- Integer output is fallible unless the value is structurally rational,
  integral, and inside the target range.
- Core geometry decisions remain in hyperreal/hyperlimit space.

## Features, testing, and license

`mesh` and `sketch` are enabled by default; `sketch` includes the mesh surface
needed by profile-to-solid operations.

```sh
cargo test --manifest-path adapters/Cargo.toml --all-features
cargo clippy --manifest-path adapters/Cargo.toml --all-features --all-targets -- -D warnings
cargo bench --manifest-path adapters/Cargo.toml --bench graphics_export
```

The crate is MIT-licensed, matching CSGRS.
