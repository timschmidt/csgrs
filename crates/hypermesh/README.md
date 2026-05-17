# hypermesh

`hypermesh` is the experimental 3D mesh-boolean crate in the Hyper workspace.
Today it is still packaged internally as `boolmesh` and carries the original
float-oriented mesh boolean engine, inspired by Manifold-style robust mesh CSG.
This README describes its role in the Hyper ecosystem rather than as a
standalone upstream release.

The current implementation exposes a simple `Manifold` buffer and
`compute_boolean` entry point for union, subtraction, and intersection over
closed manifold triangle meshes. It uses `glam` vectors, optional `rayon`, a
Morton broad phase, triangle intersection kernels, topology simplification, and
ear-clipping triangulation. Inputs are expected to be manifold; boundaries,
overlapping geometry, and non-manifold cases remain outside the accepted
contract.

## Hyper Ecosystem Role And Links

`hypermesh` is intended to become the 3D counterpart to `hypercurve` and
`hypertri`: solid mesh topology, manifold validation, triangle/plane
intersection, remeshing, and boolean result reconstruction. The lower layers
should eventually supply exact scalar values (`hyperreal`), vector/matrix facts
(`hyperlattice`), and orientation/incidence predicates (`hyperlimit`).
`hypersolve` and `hyperphysics` can then consume prepared mesh facts for
constraints and simulation adapters.

The current code does not yet use those lower Hyper layers for its core boolean
decisions. Treat it as an imported/experimental engine whose public surface is
being evaluated before exact-aware predicates and retained topology facts are
threaded through it.

Stack links:

- [hyperreal](../hyperreal/README.md): exact rational, symbolic, and computable
  real arithmetic.
- [hyperlimit](../hyperlimit/README.md): exact predicate policy and certified
  geometric decisions.
- [hyperlattice](../hyperlattice/README.md): small exact vector, matrix, and
  transform algebra.
- [hypercurve](../hypercurve/README.md): planar curve, contour, region, and
  boolean geometry.
- [hypertri](../hypertri/README.md): exact polygon triangulation and constrained
  Delaunay topology.
- [hypermesh](../hypermesh/README.md): 3D mesh boolean experiments and the
  future exact-aware mesh-topology layer.
- [hypersolve](../hypersolve/README.md): experimental exact-aware solver layer.
- [hyperdrc](../hyperdrc/README.md): PCB design-readiness checks over exact-aware
  geometry adapters.
- [hyperphysics](../hyperphysics/README.md): placeholder physics-domain crate
  for the exact geometry stack.
- [csgrs](../csgrs/readme.md): constructive solid geometry and polygon boolean
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

- `Manifold::new` for vertex/index buffers with manifold validation;
- `compute_boolean` for add, subtract, and intersect operations;
- Morton-code broad-phase collision candidate generation;
- triangle/triangle intersection kernels and half-edge style topology records;
- topology simplification passes for duplicated, collapsed, and degenerate
  edges;
- ear-clipping triangulation for reconstructed polygonal faces;
- optional `rayon` and demo-only `bevy` features;
- unit tests and example models inherited from the imported engine.

Known limits:

- the crate package metadata still says `boolmesh`;
- primitive floating-point coordinates are still the operational model;
- input meshes must already be closed and manifold;
- exact Hyper predicates, structural facts, provenance records, and exact-aware
  validation are future integration work;
- primitive generation and transformation helpers intentionally live outside
  the mesh boolean core.

## Usage

```rust
use boolmesh::{compute_boolean, Manifold, OpType};

let left = Manifold::new(&positions_a, &indices_a)?;
let right = Manifold::new(&positions_b, &indices_b)?;
let result = compute_boolean(&left, &right, OpType::Subtract)?;
# Ok::<(), String>(())
```

Run the demo examples from this crate:

```sh
cargo run --package boolmesh --release --example menger_sponge --features bevy,rayon,f32
```

## Roadmap

- Rename/package-align the crate once its Hyper-facing API is stable.
- Replace local float predicates at topology branch points with `hyperlimit`
  predicates over exact-aware coordinates.
- Preserve mesh facts: manifoldness certificates, face planes, edge incidence,
  inserted-intersection provenance, bounds, and transform/source-grid facts.
- Add adversarial tests for coplanar, near-coplanar, sliver, shared-edge,
  shared-vertex, and non-manifold rejection cases.
- Add benchmarks that separate broad-phase, intersection, topology rebuild, and
  simplification costs.
