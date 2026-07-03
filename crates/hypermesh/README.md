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

The boolean kernel targets closed PWN triangle meshes whose coordinates are
represented as `hyperreal::Real` values. Predicate decisions are routed through
the strict `hyperlimit` pipeline and may return `UnknownClassification` when a
sign, path, or classification cannot be certified under that pipeline.

The implementation is being aligned with the EMBER algorithm in `ember.pdf`.
It currently favors explicit uncertainty over silent topology guesses: failed
leaf classification or reference-path construction must either find another
certified path or report `UnknownClassification`. Arbitrary computable `Real`
values are outside the current completeness claim when bounded strict
refinement cannot decide the required predicate.

Exact same-surface two-mesh booleans are handled as a proven equivalence before
subdivision: union/intersection preserve the surface, while difference and
symmetric difference are empty.

Meshes with provably disjoint AABB interiors are also handled before
subdivision for regularized operations where that proof is sufficient:
intersection is empty, and difference preserves the left operand. This includes
boundary-plane touching boxes.

Strict containment between two non-intersecting mesh surfaces is handled before
subdivision when every candidate vertex has certified nonzero winding inside
the container and no candidate vertex lies on the container surface.

Boundary-only contact for two meshes is also handled before subdivision for
intersection and difference when no input vertex is strictly inside the other
mesh and no pair of faces has a certified transverse crossing through both
face interiors. In that case, regularized intersection is empty and difference
preserves the left operand.

Same-basis oriented boxes are handled by exact cell decompositions before
subdivision. Ordered axis-aligned box fixtures are covered by that same path.
The regression suite verifies this through the certified no-repair
triangulation helper.

Subdivision depth is a certification budget, not a permission to guess. If a
task reaches `max_depth` while it still contains more polygons than the leaf
threshold and the bounds remain splittable, the operation reports
`UnknownClassification` instead of forcing an oversized leaf.

`triangulate_and_resolve_certified` resolves exact duplicate vertices,
duplicate faces, and T-junctions, but refuses non-empty open or zero-volume
triangle soups instead of capping or peeling them. The broader
`triangulate_and_resolve` compatibility helper still performs boundary cleanup
for cases whose classified arrangement is not yet emitted closed by
construction.

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
