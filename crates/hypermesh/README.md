# hypermesh

`hypermesh` is a Rust experiment for exact triangle-mesh boolean operations over
hyperreal-backed coordinates.


## Features

- Exact input coordinates through `hyperreal` / `hyperlattice`.
- Triangle mesh boolean operations: union, intersection, difference, and
  symmetric difference.
- Borrowed boolean and preparation APIs over `MeshRef` views.
- Focused Rust regression tests for topology, classification, and output mesh
  regularization.
- An egui / WASM demo using the shared `hypergraphics` renderer.

## Algorithmic Support Boundary

The intended input model is finite, closed, piecewise-winding-number (PWN)
triangle meshes. Vertex coordinates are `hyperreal::Real` values carried
through `hyperlattice::Point3`; the boolean kernel does not downcast geometry
to primitive floats. Meshes may contain disconnected closed components and
nested closed components. Empty meshes, degenerate source triangles, open
triangle soups, invalid triangle indices, and arbitrary non-PWN surface
collections are outside the supported model and are rejected before the boolean
subdivision path.

Predicate decisions are routed through the strict exact-predicate stack
(`hyperlimit` and `hyperlattice` as support crates). A scalar predicate, path
trace, reference propagation step, or classification that cannot be certified
returns an explicit `HypermeshError`; the algorithm must not silently use an
approximate answer. In particular, arbitrary undecidable computable `Real`
values remain outside any completeness claim when strict bounded refinement
cannot decide the required sign.

The implementation is being aligned with the EMBER algorithm in `ember.pdf`.
Completion is not yet claimed. Current general-path coverage includes
subdivision, face-local BSP splitting, exact pairwise intersection handling,
certified winding-vector propagation by segment traces, and no-repair
triangulation checks for the regression cases that have been promoted to the
general path. Remaining gaps are tracked by code paths that can still return
explicit certification errors.

Leaf classification currently searches certified off-face probes from exact
leaf interior points by building strict witness cells along the support normal
or a support-axis direction inside the open interval before the nearest crossed
local surface or AABB boundary. The centroid is still used as a deterministic
seed when needed, but replayable leaf interior targets now prefer strict
halfspace witnesses and deterministic EMBER-style points formed by shifting
adjacent edge planes inward and intersecting them with the support plane. If a
probe lies on a traced surface,
cannot reach the adjacent cell, or cannot be traced from the reference point,
that probe is discarded. If no certified probe path remains, the leaf reports
`UnknownClassification`; there is no silent fallback to the reference winding
number. There are no input-assumption bypass flags; leaves run pairwise
intersection discovery across all local polygons, including same-mesh
self-intersections, and classify each direct polygon separately. Normal probes
derived from retained leaf definitions and strict normal-corridor witnesses
retain their defining plane triples, and leaf classification retries EMBER
plane-replacement traces across all retained reference and probe definitions
before failing. Leaf interior
construction also asks `hyperlimit` for a strict replayable halfspace witness
inside the leaf so probe generation can retain multiple certified plane
definitions even when the affine interior point itself came from
centroid-style construction, and those retained definitions now include every
exact witness-active leaf halfspace we can verify rather than only the
feasibility basis planes. Normal- and axis-direction probe witnesses now do
the same for their strict witness cells instead of keeping only a hand-built
definition family, and they now reuse any strict closed-cell feasibility
witness that `hyperlimit` already provides before falling back to additional
shifted-cell witnesses. Axis-direction probes are now constructed from strict
witness search in the closed axis corridor and desired support-side cell,
rather than by midpoint sampling. Full
plane-replacement coverage for every reference/probe construction remains
unfinished, though probe fallback now also retries from the reference point's
exact axis-plane definition even when other retained start definitions exist,
and duplicate certified probe witnesses now merge their retained definition
families instead of dropping later constructions.

Subdivision reference propagation currently accepts the EMBER projection of the
parent reference point onto a child AABB only when the projected point and trace
are certified valid. Existing references are reused only when they are strict
child-cell interior points and not on local surfaces. If the projected point or
direct trace is degenerate, the implementation first tries local axis-aligned
escape corridors inside certified open intervals before the next surface hit or
AABB boundary, using `hyperlimit` witness search instead of midpoint sampling.
If those direct one-axis corridors still cannot be traced, it builds the
certified axis-aligned escape box bounded by the nearest exact axis crossings
or child AABB faces around that projection, then asks `hyperlimit` for a
replayable halfspace-feasibility witness inside that box while backtracking
over certified slack sides of local support planes. If that tighter escape cell
still cannot be traced, it falls back to the full child-cell support search.
Segment tracing uses
direct paths and arrangement-coordinate endpoint-box detours, cut by local
vertex coordinates and exact endpoint-box surface crossings, when axis-ordered
paths hit surfaces. Those detour points are now constructed from each
certified cut endpoint box by first reusing any strict closed-cell feasibility
witness that `hyperlimit` already provides, then retaining the strict interior
cell seed and any additional strict shifted-cell witness instead of midpoint
Cartesian sampling.
Chosen detour legs now retry the same axis-ordered then
direct certified path search before the detour is abandoned. If none trace
cleanly, it reports
`ReferencePropagationFailed` instead of using random/interior sampling. The
reference point carries retained plane triples; projected/escaped references
carry axis-plane triples, and support-cell witnesses are now constructed from
closed support-side cells by enumerating exact feasible cell vertices, taking a
strict interior centroid seed, and then asking `hyperlimit` for a replayable
witness inside the inward-shifted strict cell. When `hyperlimit` already
provides a strict feasible witness for the closed support cell, hypermesh now
tries that direct certified target before the shifted replayable witness, so
reference propagation can backtrack across multiple certified targets inside
one feasible support-side cell instead of collapsing the cell to one point.
Support-cell retained definitions now include every exact witness-active
halfspace we can verify, not just the feasibility basis planes returned by
`hyperlimit`. When direct tracing cannot certify a reference step, hypermesh
retries certified plane-replacement traces between retained definitions, with
exact axis-plane definitions appended for both endpoints before giving up. The
same retained definitions are used again during leaf classification for
plane-defined probes, and duplicate certified target points merge their
retained definition sets instead of dropping later constructions. The
support-cell fallback backtracks across alternate feasible support-side cells
when a candidate target cannot be traced. Full EMBER plane-replacement
coverage for every reference construction remains unfinished.

`EmberConfig::default()` runs only the general subdivision/BSP/classification
path. The previous same-surface, disjoint-bound, strict-containment,
boundary-contact, and oriented-box shortcuts have been removed, so public
boolean results either certify through the general path or return an error.
Subdivision first applies exact local WNV-transition reachability when the
reachable state set stays small, and otherwise falls back to conservative
per-component reachable winding ranges. A task is discarded only when those
exact or range-based bounds make the Boolean indicator impossible for every
reachable winding vector in the approximation. Full arrangement-complete
finite-automaton WNV reachability remains an implementation target.

Subdivision depth is a certification budget, not a permission to guess. Bounds
remain splittable whenever any axis has certified positive extent; there is no
coordinate-scale cutoff. When local polygon vertices provide an exact interior
arrangement gap, subdivision now prefers that gap over a pure AABB midpoint for
the next split plane, and when vertex-only candidates do not improve the split
it now also considers exact local pairwise-intersection segment endpoints
before falling back to the midpoint. If a task reaches `max_depth` while it still contains
more polygons than the leaf threshold and the bounds remain splittable,
hypermesh attempts to certify the current task as a leaf using the same exact
BSP/classification path. Enabled BSP leaves are rejected unless exact pairwise
checks prove they have no remaining interior segment intersections with local
polygons. Coplanar overlap and effective-delta checks now use the same
certified strict leaf interior witness family instead of a centroid-only test
point, and pairwise coplanar overlap detection now reuses that certified
convex-polygon interior witness construction instead of a standalone centroid
witness when no strict contained vertex exists. Hypermesh reports `SubdivisionDepthLimit` if the configured depth
budget is reached before the current task can be certified as a leaf, and it
reports `UnknownClassification` if leaf classification or this isolation check
fails before appending output outside the depth-limit branch. Full
arrangement-isolation termination is still an implementation target.

`triangulate_and_resolve_certified` resolves exact duplicate vertices,
duplicate faces, and T-junctions, but refuses non-empty outputs with boundary
edges or zero signed volume instead of capping or peeling them. Non-manifold
edge valence is allowed for closed PWN output. If exact T-junction/crossing
resolution does not converge within its certification budget, it reports
`OutputResolutionLimit`. Hypermesh does not expose a repairing triangulation
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
