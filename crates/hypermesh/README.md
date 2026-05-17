# hypermesh

`hypermesh` is the experimental 3D mesh-boolean crate in the Hyper workspace.
It carries a legacy float-oriented mesh boolean engine, inspired by
Manifold-style robust mesh CSG, while the new `hypermesh::exact` API ports mesh
validation toward Hyper's exact predicate stack.

The current implementation exposes a simple `Manifold` buffer and
`compute_boolean` entry point for union, subtraction, and intersection over
closed manifold triangle meshes. It uses `glam` vectors, optional `rayon`, a
Morton broad phase, triangle intersection kernels, topology simplification, and
ear-clipping triangulation. Inputs are expected to be manifold; boundaries,
overlapping geometry, and non-manifold cases remain outside the accepted
contract.

## Hyper Ecosystem

`hypermesh` is the experimental 3D mesh-topology layer: legacy mesh booleans,
exact-stack validation, retained mesh facts, and future exact construction
geometry.

- [hyperreal](https://github.com/timschmidt/hyperreal): exact rational, symbolic, and computable
  real arithmetic.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): exact predicate policy and certified
  geometric decisions.
- [hyperlattice](https://github.com/timschmidt/hyperlattice): small exact vector, matrix, and
  transform algebra.
- [hypercurve](https://github.com/timschmidt/hypercurve): planar curve, contour, region, and
  boolean geometry.
- [hypertri](https://github.com/timschmidt/hypertri): exact polygon triangulation and constrained
  Delaunay topology.
- [hypermesh](https://github.com/timschmidt/hypermesh): 3D mesh boolean experiments and the
  future exact-aware mesh-topology layer.
- [hypersolve](https://github.com/timschmidt/hypersolve): experimental exact-aware solver layer.
- [hyperdrc](https://github.com/timschmidt/hyperdrc): PCB design-readiness checks over exact-aware
  geometry adapters.
- [hyperphysics](https://github.com/timschmidt/hyperphysics): placeholder physics-domain crate
  for the exact geometry stack.
- [csgrs](https://github.com/timschmidt/csgrs): constructive solid geometry and polygon boolean
  engine used by HyperDRC and available as an interop target.

## Traditional Mesh Boolean Problems

Mesh booleans are hard because they combine geometry, topology, and numerical
cleanup in one pipeline. Triangle intersections can be nearly coplanar,
vertices can lie exactly on faces or edges, duplicate edges can create
non-manifold output, and small gaps or slivers can be artifacts of tolerance
rather than real features. Performance pressure pushes engines toward spatial
indexes and floating filters, while correctness pressure demands exact
incidence decisions and careful topology reconstruction.

The long-term Hyper approach is to split those concerns. Broad-phase and local
mesh structure should prune work aggressively; exact predicates should certify
orientation, coplanarity, segment/triangle hits, and winding decisions; and
topology repair should carry provenance for inserted vertices, split faces, and
protected edges. Numerical explosion should be controlled by using exact
reducers only at branch points, preserving shared-scale facts, and avoiding
global canonicalization of every coordinate before the mesh topology actually
needs it.

## Current Status

Implemented in the current engine:

- feature-gated exact validation (`exact`) and legacy boolean
  (`legacy-boolean`) paths so users do not build algorithms they do not use;
- `Manifold::new` for vertex/index buffers with manifold validation;
- `compute_boolean` for add, subtract, and intersect operations;
- Morton-code broad-phase collision candidate generation;
- triangle/triangle intersection kernels and half-edge style topology records;
- topology simplification passes for duplicated, collapsed, and degenerate
  edges;
- ear-clipping triangulation for reconstructed polygonal faces;
- optional `rayon` and demo-only `bevy` features;
- exact `f64` edge import through `hyperreal::Real` dyadic lifting;
- `MeshFacts`, vertex/edge/face/triangle fact carriers, predicate provenance,
  and structured validation diagnostics in `hypermesh::exact`;
- unit tests, proptests, a fuzz target, and benchmarks for exact validation.

Known limits:

- primitive floating-point coordinates are still the operational model;
- input meshes must already be closed and manifold;
- exact Hyper predicates, structural facts, provenance records, and exact-aware
  validation are implemented for the new validation boundary but not yet for
  every boolean/intersection/simplification kernel;
- primitive generation and transformation helpers intentionally live outside
  the mesh boolean core.

## Usage

```rust
use hypermesh::prelude::*;

let left = Manifold::new(&positions_a, &indices_a)?;
let right = Manifold::new(&positions_b, &indices_b)?;
let result = compute_boolean(&left, &right, OpType::Subtract)?;
# Ok::<(), String>(())
```

Run the demo examples from this crate:

```sh
cargo run --package hypermesh --release --example menger_sponge --features bevy,rayon,f32
```

Build only the exact validation boundary without legacy boolean kernels:

```sh
cargo test --no-default-features --features exact
```

## Roadmap

- Replace local float predicates at topology branch points with `hyperlimit`
  predicates over exact-aware coordinates.
- Preserve mesh facts: manifoldness certificates, face planes, edge incidence,
  inserted-intersection provenance, bounds, and transform/source-grid facts.
- Add adversarial tests for coplanar, near-coplanar, sliver, shared-edge,
  shared-vertex, and non-manifold rejection cases.
- Add benchmarks that separate broad-phase, intersection, topology rebuild, and
  simplification costs.
