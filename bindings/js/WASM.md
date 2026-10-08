# JavaScript and TypeScript through WASM

The `wasm` feature exports native mesh, curve, point, matrix, and plane
wrappers. `wasm-bindgen` generates their JavaScript and TypeScript declarations
from Rust; `bindings/js/csgrs.ts` is the separate native C-ABI loader adapter.

Build a browser package with the command in the [root README](../../readme.md).
Use a `wasm-bindgen-cli` version matching the `wasm-bindgen` entry in
`Cargo.lock`. The generated wrappers own native allocations. Call `free()`
when a wrapper is no longer needed; consuming methods such as `intoMesh()`,
`intoRegion()`, `intoPlane()`, and `intoMatrix()` already release their input
wrapper and transfer ownership to the returned wrapper.
The generated declarations also support `Symbol.dispose`; TypeScript clients
should include `ESNext.Disposable` in their `lib` setting.

## Certainty and query status

Completed operation results expose `certainty` as `GeometryCertaintyJs`:

- `Certified`: every consumed topology decision was certified.
- `Approximate512Consumed`: a decision used the explicitly authorized
  512-bit terminal. Merely requesting approximate policy does not set this.

The existing `approximate512Consumed` Boolean remains available. Read a
result's receipt before consuming its wrapper. A mesh, curve, plane, or matrix
returned by a consuming method is the native value; keep the receipt alongside
it when passing the operation's evidence to another consumer. Certainty
describes predicate decisions, not the fidelity of `f64` display buffers.
Curve `toArrays` and `toRegionProfiles` also return the named `certainty` field.

`containsXYWithContext` and `containsPointWithContext` return a
`GeometryBoolResultJs`. Its `status` distinguishes:

| Status | `value` | `certainty` | `reason` |
| --- | --- | --- | --- |
| `Decided` | `true` or `false` | Named receipt | `undefined` |
| `Boundary` | `undefined` | Named receipt | `undefined` |
| `Uncertain` (curve) | `undefined` | `undefined` | Diagnostic string |
| `Unknown` (mesh) | `undefined` | `undefined` | Diagnostic string |

Curve boundary membership is decided under the selected policy and carries
that decision's certainty; it must not be confused with an unresolved
predicate. The empty curve region classifies every point
as outside. Mesh membership uses strict solid containment: a point on a mesh's
surface is a decided `false`, matching the native Hypermesh contract.
Invalid inputs still throw. The strict `containsXY` convenience method returns
`undefined` only for a decided boundary and throws if classification is
unresolved; it no longer silently conflates uncertainty with a boundary.

```ts
import { CurveRegionJs, GeometryQueryStatusJs } from "./pkg/csgrs";

const region = CurveRegionJs.square(1);
const query = region.containsXYWithContext(0, 0.5, false);
try {
  switch (query.status) {
    case GeometryQueryStatusJs.Decided:
      console.log(query.value, query.certainty);
      break;
    case GeometryQueryStatusJs.Boundary:
      console.log("boundary", query.certainty);
      break;
    case GeometryQueryStatusJs.Uncertain:
    case GeometryQueryStatusJs.Unknown:
      console.log("unresolved", query.reason);
      break;
  }
} finally {
  query.free();
  region.free();
}
```

## Reproduce the boundary checks

With `wasm32-unknown-unknown`, the matching `wasm-bindgen` CLI, and Node.js
installed, install the locked TypeScript compiler and run:

```sh
npm ci --ignore-scripts --no-audit --no-fund --prefix tests/wasm
bash scripts/test-wasm-bindings.sh
cargo test --locked --no-default-features --features wasm --test wasm_certainty
```

The script builds the locked WASM artifact, generates real Node.js bindings,
exercises geometry/receipt/invalid-input behavior, and checks a client against
the generated declarations. Native Rust tests additionally construct exact
undecidable scalar expressions, checking strict/approximate/strict query
replay without inventing a float representation of those expressions.
