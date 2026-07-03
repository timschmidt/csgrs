# hypermesh

`hypermesh` is a Rust experiment for exact triangle-mesh boolean operations over
hyperreal-backed coordinates.


## Features

- Exact input coordinates through `hyperreal` / `hyperlattice`.
- Triangle mesh boolean operations: union, intersection, difference, and
  symmetric difference.
- Borrowed and owned mesh APIs.
- Focused Rust regression tests for topology, classification, and output mesh
  regularization.
- An egui / WASM demo using the shared `hypergraphics` renderer.

## Algorithmic Support Boundary

The intended input model is finite, closed, piecewise-winding-number (PWN)
triangle meshes. Vertex coordinates are `hyperreal::Real` values carried
through `hyperlattice::Point3`; the boolean kernel does not downcast geometry
to primitive floats. Meshes may contain disconnected closed components and
nested components when the caller either supplies or assumes the corresponding
NSI/NNC facts. Open triangle soups, invalid triangle indices, and arbitrary
non-PWN surface collections are outside the supported model.

Predicate decisions are routed through the strict exact-predicate stack
(`hyperlimit`, `hyperlattice`, and `hypertri` as support crates). A predicate,
path trace, or classification that cannot be certified returns
`UnknownClassification`; the algorithm must not silently use an approximate
answer. In particular, arbitrary undecidable computable `Real` values remain
outside any completeness claim when strict bounded refinement cannot decide the
required sign.

The implementation is being aligned with the EMBER algorithm in `ember.pdf`.
Completion is not yet claimed. Current general-path coverage includes
subdivision, face-local BSP splitting, exact pairwise intersection handling,
certified winding-vector propagation by segment traces, and no-repair
triangulation checks for the regression cases that have been promoted to the
general path. Remaining gaps are tracked by code paths that can still return
`UnknownClassification`, and by compatibility fallbacks described below.

Leaf classification currently searches certified off-face probes from exact
leaf interior points. If a probe lies on a traced surface, cannot reach the
adjacent cell, or cannot be traced from the reference point, that probe is
discarded. If no certified probe path remains, the leaf reports
`UnknownClassification`; there is no silent fallback to the reference winding
number.

Subdivision reference propagation currently accepts the EMBER projection of the
parent reference point onto a child AABB only when the projected point and trace
are certified valid. If the projected point or direct trace is degenerate, the
implementation tries local axis-aligned escape targets and their multi-axis
combinations inside certified open intervals before the next surface hit or
AABB boundary. If none trace cleanly, it reports `UnknownClassification`
instead of using finite random/interior sampling. The full EMBER
plane-replacement reference construction remains unfinished.

`EmberConfig::default()` runs the general subdivision/BSP/classification path
with shortcut fallbacks disabled. If a caller explicitly sets
`use_proven_shortcuts: true`, the implementation still attempts the general
path first. Shortcut results are used only when the general path errors or its
classified output fails `triangulate_and_resolve_certified`. Current fallback
families are exact same-surface equivalence, disjoint-bound proofs, strict
containment proofs, boundary-only contact proofs, and same-basis oriented-box
cell decompositions. These are compatibility fallbacks, not the primary
algorithmic route.

Subdivision depth is a certification budget, not a permission to guess. If a
task reaches `max_depth` while it still contains more polygons than the leaf
threshold and the bounds remain splittable, the operation reports
`UnknownClassification` instead of forcing an oversized leaf. Full
arrangement-isolation termination is still an implementation target.

`triangulate_and_resolve_certified` resolves exact duplicate vertices,
duplicate faces, and T-junctions, but refuses non-empty outputs with boundary
edges or zero signed volume instead of capping or peeling them. Non-manifold
edge valence is allowed for closed PWN output. The broader
`triangulate_and_resolve` compatibility helper still performs boundary cleanup
for legacy consumers and for cases whose classified arrangement is not yet
emitted closed by construction.

## Building

```bash
cargo check
cargo test
```

## Demo

```bash
cd examples/hypermesh_ui
trunk serve --address 127.0.0.1 --port 8082
```

Then open:

```text
http://127.0.0.1:8082/hypermesh/
```

## Layout

```text
src/                    Rust boolean kernel
tests/                  Rust unit and regression tests
examples/hypermesh_ui/ egui/WASM demo
```

## License

MIT
