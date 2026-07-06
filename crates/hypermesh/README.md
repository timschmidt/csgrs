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
local surface or AABB boundary. Leaf interior targets now first come from the
closed leaf halfspace cell and its shifted strict witness family, and that
same witness family now also includes stricter replayable constructions seeded
from the exact leaf-vertex barycenter instead of running a separate centroid
fallback branch afterward. The implementation no longer falls back to treating
the naked centroid as a certified leaf witness. If a
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
before failing. When a straight interior-to-probe reachability segment is
blocked, leaf classification now also retries retained plane-replacement paths
between the interior and probe definitions before discarding that probe, and it
now also retries the same certified endpoint-box detour family used by segment
tracing when straight interior-to-probe reachability is blocked; those probe
reachability detours now also allow the same bounded nested-detour retry on
detour legs instead of collapsing every chosen leg to the no-detour family
immediately. Retained plane-replacement fallback now also allows those
individual replacement steps to use the same bounded detour family, while still
stopping short of nested plane-replacement recursion, and those probe
reachability replacement steps now keep their actual intermediate plane triples
instead of collapsing each step back to axis-only endpoint definitions. Probe winding fallback
from retained reference/probe definitions now uses that same bounded detour
family on its replacement steps as well, again without recursing into another
plane-replacement layer. Leaf interior
construction now asks `hyperlimit` for strict replayable witnesses from the
closed leaf cell and its shifted witness family before it falls back to a
centroid seed, so probe generation can retain multiple certified plane
definitions across those witness families. Shifted leaf, probe, and detour
witness expansions now also backtrack past uncertified shifted seeds instead of
aborting the whole local witness family, and strict halfspace-cell seed
collection now does the same for uncertified strictness checks on candidate
direct witnesses.
definitions, and those retained definitions now include every
exact witness-active leaf halfspace we can verify rather than only the
feasibility basis planes. The shifted strict leaf cell now also contributes
its own strict feasibility witness and exact feasible vertices rather than a
centroid seed family or one chosen feasibility witness. Normal- and axis-direction probe
witnesses now do the same for their strict witness cells instead of keeping
only a hand-built definition family, and they now reuse any strict closed-cell
feasibility witness that `hyperlimit` already provides before falling back to
additional shifted-cell witnesses. Those shifted witness searches now also
start from the exact feasible vertices of the local witness cell rather than a
centroid seed family, and each shifted witness cell now contributes its own
strict witness family and exact feasible vertices instead of collapsing to one
chosen feasibility witness. Axis-direction probes are now
constructed from strict
witness search in the closed axis corridor and desired support-side cell,
rather than by midpoint sampling. Full
plane-replacement coverage for every reference/probe construction remains
unfinished, though probe fallback now also retries from the reference point's
exact axis-plane definition even when other retained start definitions exist,
and duplicate certified probe witnesses now merge their retained definition
families instead of dropping later constructions. Strict leaf-cell witness
points whose richer active-plane replay fails are now still retained as exact
axis-defined interior witnesses rather than forcing an immediate centroid-seed
fallback, and the same axis-defined retention now applies to certified normal
and axis probe witnesses instead of discarding them when richer probe-plane
replay fails. Certified endpoint-box detour witnesses now also retain exact
axis-defined target definitions instead of being dropped when richer probe
replay reconstruction fails. Definition-preserving normal-probe search now
also augments, rather than suppresses, the broader certified normal-corridor
witness family when both are available, and axis-direction probe search now
does the same for retained interior definitions whose non-support planes
preserve the moved axis. Both retained-definition probe families now also
backtrack past uncertified local candidate searches instead of aborting the
whole local probe-family search on the first `UnknownClassification`. The
probe witness build steps inside those families now likewise skip
`UnknownClassification` candidate points instead of aborting the whole local
probe witness set when later certified witnesses still exist. The
stricter replayable leaf cell
built from an interior witness now also contributes its own shifted witness
family and shifted exact vertices instead of collapsing to one witness plus
raw cell vertices, and the direct strict leaf witness family now also expands
through that stricter replayable leaf-cell construction instead of leaving it
only to the barycenter branch. Those stricter leaf-cell and shifted-edge
interior witness expansions now likewise backtrack past uncertified local
candidate searches instead of aborting the whole leaf witness family on the
first `UnknownClassification`, and the underlying strict leaf-witness build
steps now do the same candidate-locally for direct, shifted, and shifted
vertex witness points.

Subdivision reference propagation currently accepts certified projected-child
reference targets, not just a single midpoint-filled representative point.
Existing references are reused only when they are strict child-cell interior
points and not on local surfaces. Otherwise hypermesh builds the projected
child cell that preserves every axis already strict in the parent reference,
then asks `hyperlimit` for strict witnesses and exact feasible vertices in that
projected cell before tracing from the parent reference. If the first
projected target family is exhausted, later certified projected escape
witnesses now augment that direct projected target family rather than being
discarded when direct projected targets already exist, and they are retried by
direct tracing before the implementation
relaxes into escape search. If those projected
targets still cannot be traced directly, the implementation next tries local
support-side cell search inside that same certified projected cell before it
starts relaxing the geometry into broader escape families, and uncertified
projected target-family construction, projected escape-family construction, or
projected support-cell searches now fall through to later certified escape
families instead of aborting the whole propagation step. When projected target
construction yields no certified
targets at all, those later escape families now fall back to the certified
projected-cell witness family itself rather than an old-reference clamp point
or being skipped entirely. If those projected
support cells still cannot be certified, the implementation now also
backtracks past uncertified shifted projected seeds and projected vertices
instead of aborting the whole projected target family, and witness points whose
retained plane-definition reconstruction is uncertified are now still retained
as exact axis-defined targets rather than being discarded candidate-locally.
If a whole projected witness or seed family has no certified target and only
uncertified candidates, that local family now returns
`UnknownClassification` so later projected/support/escape backtracking can
decide whether to keep searching or fail, instead of silently collapsing the
local family to an empty target set.
It then tries local
axis-aligned
escape corridors across the ordered exact stop family from the next surface hit
out to the child AABB boundary, using `hyperlimit` witness search instead of
midpoint sampling and backtracking past uncertified or empty earlier corridor
searches. If those direct one-axis corridors still cannot be traced, it next
searches the ordered exact escape-box family bounded by certified axis stop
values and child AABB faces around that projected target, asking `hyperlimit`
for a replayable halfspace-feasibility witness inside each box while
backtracking over certified slack sides of local support planes. If that
tighter escape-cell family is uncertified or still cannot be traced, it falls
back to the full child-cell support search.
Segment tracing uses
direct paths and arrangement-coordinate endpoint-box detours, cut by local
vertex coordinates and exact endpoint-box surface crossings, when axis-ordered
paths hit surfaces. Those detour points are now constructed from each
certified cut endpoint box by first reusing any strict closed-cell feasibility
witness that `hyperlimit` already provides, then retaining the remaining
strict shifted-cell witness family instead of midpoint Cartesian sampling.
Those detour witnesses now also retain replayable plane definitions, and
chosen detour legs retry certified plane-replacement traces from those
definitions after the axis-ordered/direct leg search fails. Retained-definition
segment tracing now also allows a bounded nested detour retry on those legs
instead of collapsing every detour leg to the no-detour family immediately. If none trace
cleanly, it reports
`ReferencePropagationFailed` instead of using random/interior sampling. The
reference point carries retained plane triples, and projected / projected-escape
references now keep certified halfspace-derived plane definitions when those
can be reconstructed instead of collapsing every witness back to one axis-plane
triple. Closed projected child cells now also retain any remaining strict
direct witnesses of the cell, including strict exact feasible vertices, instead
of using those points only as shifted-seed sources. The projected escape-target
family now retains those same remaining strict direct projected witnesses before
it widens into shifted projected escape cells. Support-cell witnesses are now constructed from
closed support-side cells by enumerating exact feasible cell vertices and
asking `hyperlimit` for replayable witnesses inside inward-shifted strict
cells. When `hyperlimit` already provides a strict feasible witness for the
closed support cell, hypermesh now tries that richer direct witness first, then
shifted replayable witnesses built from every available strict support-cell
witness and from every exact feasible support-cell vertex, and finally any
remaining strict direct witnesses of the closed cell. That lets reference
propagation backtrack across multiple certified direct and shifted targets
inside one feasible support-side cell instead of collapsing the cell to one
point. Each shifted support cell now also contributes every strict target
recovered from its own certified witness family and its own exact feasible
vertices instead of only the first feasibility witness selected by the
halfspace predicate, and uncertified shifted support seeds or shifted support
vertices now no longer abort the whole support-cell target family. Witness
points whose retained plane-definition reconstruction is uncertified are now
also skipped candidate-locally instead of aborting those support-cell target
families. Direct projected/support seed collection now also skips uncertified
strictness checks candidate-locally instead of aborting the whole seed family.
As with the projected-child families, those candidate-local skips now surface
`UnknownClassification` when an entire local projected/support seed or target
family is uncertified and no certified witness survives, rather than silently
degenerating that family to empty.
Support-cell retained definitions now include every exact witness-active
halfspace we can verify, not just the feasibility basis planes returned by
`hyperlimit`. When direct tracing cannot certify a reference step, hypermesh
retries the same retained-definition segment path family used by leaf probes:
direct tracing, certified endpoint-box detours, and plane-replacement traces
between retained definitions, with exact axis-plane definitions appended for
both endpoints before giving up. Those retained plane-replacement steps now
also allow the same bounded detour family used by probe winding fallback,
and each such winding/reference replacement step now keeps the actual
intermediate plane triples of the current and next replacement points rather
than collapsing them back to axis-only definitions. Each such step also gets
one bounded lower definition-based segment trace of its own before the search
gives up, without recursing into another step-detoured plane-replacement
layer. The same retained
definitions are used again during leaf classification for plane-defined
probes, and duplicate certified target points merge their retained definition
sets instead of dropping later constructions. The
support-cell fallback now also backtracks across alternate feasible
support-side cells when one candidate branch returns
`UnknownClassification`, rather than aborting the whole search on the first
uncertified cell. Full EMBER plane-replacement
coverage for every reference construction remains unfinished.
Leaf probe reachability now also gives retained plane-replacement steps one
lower definition-based reachability retry of their own before the search gives
up, without opening another nested step-detoured plane-replacement layer, and
arrangement-detour reachability now backtracks past uncertified detour legs
instead of aborting the whole probe search on the first uncertified detour.

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
coordinate-scale cutoff. The fallback midpoint split is now chosen by actual
child clip counts across every positive-extent axis rather than by longest-axis
geometry alone. When local polygon vertices provide an exact interior
arrangement gap, subdivision now prefers that gap over that midpoint baseline
for the next split plane, and it now continues to consider exact local
pairwise-intersection segment endpoints even after an arrangement-gap candidate
has already improved that midpoint baseline, before keeping the best remaining
split. Split ranking now also penalizes empty-child cuts explicitly, so a cut
that leaves all polygons on one side is no longer preferred over a non-empty
branching cut with the same maximum child load just because it duplicates fewer
polygons. The recursion now backtracks across that ordered exact local split
family instead of committing to one chosen split candidate: if a higher-ranked
split hits `UnknownClassification`, `ReferencePropagationFailed`, or
`SubdivisionDepthLimit`, later exact local split candidates are still tried
before the task gives up. If a task reaches `max_depth` while the bounds remain
splittable, hypermesh attempts to certify the current task as a leaf using the
same exact
BSP/classification path. Enabled BSP leaves are rejected unless exact pairwise
checks prove they have no remaining interior segment intersections with local
polygons; segment intersections are now checked by exact open-interval
feasibility along the certified intersection segment instead of by midpoint
sampling. Coplanar overlap and effective-delta checks now use the same
certified strict leaf interior witness family instead of a centroid-only test
point, and pairwise coplanar overlap detection now reuses that certified
convex-polygon interior witness construction instead of a standalone centroid
witness when no strict contained vertex exists. Face-local BSP duplicate-overlap
suppression now uses the same certified leaf witness-family relation instead of
one centroid-style representative point. Splittable tasks now also try
that same certified leaf path exactly once before subdivision, regardless of
former leaf-threshold sizing, and if that exact leaf attempt still returns
`UnknownClassification` while the bounds remain splittable, hypermesh keeps
subdividing instead of treating heuristic leaf sizing as a hard completeness
boundary. That lets exact local arrangement isolation continue until the depth
budget or a certified leaf result stops the branch. Hypermesh reports
`SubdivisionDepthLimit` if the configured depth budget is reached before the
current task can be certified as a leaf, and it reports
`UnknownClassification` if leaf classification or this isolation check fails
before appending output outside the depth-limit branch. Full
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
