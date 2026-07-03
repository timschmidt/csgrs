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
nested closed components. Open triangle soups, invalid triangle indices, and
arbitrary non-PWN surface collections are outside the supported model.

Predicate decisions are routed through the strict exact-predicate stack
(`hyperlimit` and `hyperlattice` as support crates). A predicate, path trace,
or classification that cannot be certified returns
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
`UnknownClassification`.

Leaf classification currently searches certified off-face probes from exact
leaf interior points by stepping into the open interval before the nearest
crossed local surface or AABB boundary. If a probe lies on a traced surface,
cannot reach the adjacent cell, or cannot be traced from the reference point,
that probe is discarded. If no certified probe path remains, the leaf reports
`UnknownClassification`; there is no silent fallback to the reference winding
number. There are no input-assumption bypass flags; leaves run pairwise
intersection discovery and classify each direct polygon separately.

Subdivision reference propagation currently accepts the EMBER projection of the
parent reference point onto a child AABB only when the projected point and trace
are certified valid. Existing references are reused only when they are strict
child-cell interior points and not on local surfaces. If the projected point or
direct trace is degenerate, the implementation tries local axis-aligned escape
targets and their multi-axis combinations inside certified open intervals
before the next surface hit or AABB boundary. If those targets are degenerate,
it scans a deterministic `(n + 1)^3` strict interior grid for `n` local support
planes; in a positive-volume child cell, this grid has a point off every finite
support plane unless a predicate is undecidable. Segment tracing uses
arrangement-coordinate endpoint-box detours, cut by local vertex coordinates
and exact endpoint-box surface crossings, when direct axis-ordered paths hit
surfaces. If none trace cleanly, it reports `UnknownClassification` instead of
using random/interior sampling. The full EMBER plane-replacement reference
path construction remains unfinished.

`EmberConfig::default()` runs only the general subdivision/BSP/classification
path. The previous same-surface, disjoint-bound, strict-containment,
boundary-contact, and oriented-box compatibility fallbacks have been removed,
so public boolean results either certify through the general path or return an
error. Bounded WNV-reachability early termination is not exposed; subdivision
continues unless a leaf or certified failure is reached.

Subdivision depth is a certification budget, not a permission to guess. Bounds
remain splittable whenever any axis has certified positive extent; there is no
coordinate-scale cutoff. If a task reaches `max_depth` while it still contains
more polygons than the leaf threshold and the bounds remain splittable,
hypermesh attempts to certify the current task as a leaf using the same exact
BSP/classification path. Enabled BSP leaves are rejected unless exact pairwise
checks prove they have no remaining interior segment intersections with local
polygons. Hypermesh reports `UnknownClassification` if leaf classification or
this isolation check fails before appending output. Full arrangement-isolation
termination is still an implementation target.

`triangulate_and_resolve_certified` resolves exact duplicate vertices,
duplicate faces, and T-junctions, but refuses non-empty outputs with boundary
edges or zero signed volume instead of capping or peeling them. Non-manifold
edge valence is allowed for closed PWN output. If exact T-junction/crossing
resolution does not converge within its certification budget, it reports
`UnknownClassification`. Hypermesh does not expose a repairing triangulation
path; if the classified arrangement is not emitted closed by construction, the
operation fails certification.

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
