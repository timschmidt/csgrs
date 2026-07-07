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

Within that model, the current implementation makes a narrower runtime claim:

- Certified results are exact with respect to the strict `hyperlimit` /
  `hyperlattice` predicate stack; the algorithm must not guess through an
  uncertified sign, incidence, reachability, reference-propagation, or output
  closure decision.
- When the current EMBER search is incomplete for a branch, hypermesh reports
  an explicit error such as `UnknownClassification`,
  `ReferencePropagationFailed`, `SubdivisionDepthLimit`, `OpenOutput`, or
  `OutputResolutionLimit` instead of silently widening the support claim.
- Completion is not yet claimed for the whole intended closed-PWN model. The
  remaining finite-family search structure and depth-budgeted termination mean
  that some intended-model inputs can still fail with those explicit errors
  even when a complete EMBER implementation would certify them.

Predicate decisions are routed through the strict exact-predicate stack
(`hyperlimit` and `hyperlattice` as support crates). A scalar predicate, path
trace, reference propagation step, or classification that cannot be certified
returns an explicit `HypermeshError`; the algorithm must not silently use an
approximate answer. In particular, arbitrary undecidable computable `Real`
values remain outside any completeness claim when strict bounded refinement
cannot decide the required sign, incidence, or ordering fact needed by the
current bounded search.

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
from exact closed leaf halfspace-cell geometry: strict direct witnesses,
shifted witnesses, exact shifted vertices, and strict centroids of every
feasible closed-cell vertex subset of size two or greater instead of running a separate centroid
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
reachability detours now also allow the same cycle-guarded nested-detour retry
on detour legs instead of collapsing every chosen leg to the no-detour family
immediately. Retained plane-replacement fallback now also allows those
individual replacement steps to follow their own exact local detour-point
family with the same visited-point cycle guard, while still stopping short of
nested plane-replacement recursion, and those probe
reachability replacement steps now keep their actual intermediate plane triples
instead of collapsing each step back to axis-only endpoint definitions; if an
intermediate replacement ordering itself cannot be reconstructed as an affine
point, that local reachability search now surfaces `UnknownClassification`
instead of silently treating the ordering as merely blocked, and probe
reachability now also backtracks across uncertified start/end definition pairs
before giving up on the whole local plane-replacement family, while skipping
exact duplicate retained-definition plane triples before expanding those local
pair searches. Probe-family collection now also skips duplicate retained
definition triples before rerunning the same constrained corridor searches.
Probe winding fallback
from retained reference/probe definitions now uses that same bounded detour
family on its replacement steps as well, again without recursing into another
plane-replacement layer, and that retained-definition segment search now also
backtracks across uncertified direct definition pairs and continues into
certified detour families instead of collapsing an all-uncertified direct
family to `None`. The retained-definition entry path above the step-detoured
plane-replacement fallback now also surfaces an uncertified direct trace family
explicitly and lets that broader replacement search decide the fallback,
instead of flattening the uncertified direct trace to absence first. The plain
no-detour segment tracer below that retained path
family now also surfaces `UnknownClassification` when its local axis/direct
trace family is uncertified, and the broader retained-definition and detour
search layers make the fallback decision explicitly instead of treating that
uncertified local trace as absent. Leaf probe winding now likewise surfaces
`UnknownClassification` when every retained-definition path is uncertified
instead of silently treating that probe winding trace as absent. Leaf interior
construction now asks `hyperlimit` for strict replayable witnesses from the
closed leaf cell and its shifted witness family before it falls back to a
centroid seed, so probe generation can retain multiple certified plane
definitions across those witness families. Shifted leaf, probe, and detour
witness expansions now also backtrack past uncertified shifted seeds instead of
aborting the whole local witness family, and strict halfspace-cell seed
collection now does the same for uncertified strictness checks on candidate
direct witnesses. Direct detour-target construction from those strict witnesses
now also backtracks past uncertified target builds instead of aborting before
later certified direct or shifted detour targets run, and the surrounding
endpoint-box detour family now also backtracks past uncertified local boxes
instead of aborting before later certified detour boxes run. The earlier
axis-interval surface-cut construction feeding those endpoint boxes now also
skips past partially uncertified local surface crossings instead of aborting
the whole detour-box family before later exact boxes are even formed.
If one of those local witness or seed families is entirely uncertified and no
certified candidate survives, it now returns `UnknownClassification` instead of
quietly collapsing that local family to an empty witness set.
definitions, and those retained definitions now include every
exact witness-active leaf halfspace we can verify rather than only the
feasibility basis planes. The shifted strict leaf cell now also contributes
its own strict feasibility witness, exact feasible vertices, and exact
closed-cell geometry seeds from every feasible vertex subset of size two or
greater, including strict edge midpoints, rather than a centroid seed family or one chosen
feasibility witness. Normal- and axis-direction probe witnesses now do the
same for their strict witness cells instead of keeping only a hand-built
definition family, and they now reuse any strict closed-cell feasibility
witness that `hyperlimit` already provides before falling back to additional
shifted-cell witnesses. Those shifted witness searches now also start from the
witness-specific active-plane family only when the witness is the actual
`hyperlimit` report witness; later strict seeds stay on the generic exact-cell
family instead of inheriting a mismatched active basis.
exact feasible vertices and exact closed-cell geometry seeds of the local
witness cell rather than a centroid seed family, and each shifted witness cell
now contributes its own strict witness family, exact feasible vertices, and
raw exact geometry seeds
while backtracking past wholly uncertified strict-seed subfamilies before
giving up on the later exact-vertex and raw geometry-seed families,
deduping overlapping strict-seed, exact-vertex, and geometry-seed inputs in
first-occurrence order before rerunning the same shifted witness-cell
construction from equivalent points, and the halfspace-cell seed builders now
reuse one exact feasible-vertex / geometry-seed family per local cell before
widening into shifted witness-cell search. The leaf-witness builders now reuse
that same raw local family instead of recomputing feasible vertices and
geometry seeds after deriving strict leaf seeds, and shifted leaf/probe/detour
witness builders now also dedupe later shifted seed families against the
report witness itself so they do not rebuild that same direct witness through
later strict-seed,
exact-vertex, or geometry-seed families,
and equivalent certified probes inside one leaf classification now reuse the
same retained-definition winding trace instead of retracing the same exact
probe point/definition family repeatedly across probe sides or repeated probe
families. Equivalent per-leaf probe surface-hit checks and retained-definition
probe reachability checks are now likewise cached by exact probe and
interior/probe definition families instead of being recomputed for repeated
local probe families, and repeated detour-point surface-hit checks inside the
live segment/probe detour searches now reuse the same exact local surface test
across failed sibling detour branches instead of rerunning it for duplicate
detour points in each recursive branch, while axis-ordered
segment tracing now also caches exact intermediate surface-hit checks across
the repeated ordering family instead of rechecking the same intermediate point
in each ordering,
instead of collapsing to one chosen feasibility witness. Axis-direction probes are now
constructed from strict
witness search in the closed axis corridor and desired support-side cell,
rather than by midpoint sampling, and that axis probe path now also walks the
ordered exact stop family out to the child boundary instead of stopping at the
first certified crossing. Partially uncertified local axis crossings no longer
abort the whole corridor family before later exact corridors run. Full
Normal-direction probes now do the same: they walk the ordered exact stop
family out to the child boundary instead of stopping at one certified normal
corridor, and partially uncertified local normal crossings no longer abort the
whole corridor family before later exact corridors run. Full
plane-replacement coverage for every reference/probe construction remains
unfinished, though probe and reference fallback now both retry from the
reference point's exact axis-plane definition even when other retained start
definitions exist, and duplicate certified probe witnesses now merge their retained definition
families instead of dropping later constructions. Strict leaf-cell witness
points whose richer active-plane replay fails are now still retained as exact
axis-defined interior witnesses rather than forcing an immediate centroid-seed
fallback, and the same axis-defined retention now applies to certified normal
and axis probe witnesses instead of discarding them when richer probe-plane
replay fails. Certified endpoint-box detour witnesses now also retain exact
axis-defined target definitions instead of being dropped when richer probe
replay reconstruction fails. Definition-preserving normal-probe search now
also ignores stale active-plane indices and still salvages coincident local
halfspace definitions before collapsing all the way to axis-only replay, and
the same recovery now applies to reference-target definition reconstruction.
Strict leaf-witness replay now does the same for stale active-plane indices
instead of collapsing immediately to support-plus-axis interior replay when
coincident leaf-cell planes are still available.
If leaf-interior or probe definition reconstruction is itself uncertified, the
fallback axis-defined candidate is still explored, but if no later certified
probe family or probe path succeeds that branch now surfaces
`UnknownClassification` instead of being flattened into plain absence.
The same now applies to fallback-built detour targets: if an uncertified
axis-defined detour is later skipped or cannot certify either leg, that local
detour family also surfaces `UnknownClassification` instead of reading as an
ordinary missing detour. The same rule now also holds in the cycle-guarded
step-detour helper used by plane-replacement reachability, so fallback-built
step detours no longer collapse back to plain `false` when they are skipped or
their later legs cannot certify a path. The cycle-guarded runtime detour paths
now also preserve that uncertainty when a fallback-built detour is skipped
because it revisits the current start/end or an already-visited detour point,
instead of flattening that skip back into ordinary absence.
Definition-preserving normal-probe search also now
also augments, rather than suppresses, the broader certified normal-corridor
witness family when both are available, and axis-direction probe search now
does the same for retained interior definitions whose non-support planes
preserve the moved axis. Both retained-definition probe families now also
backtrack past uncertified local candidate searches instead of aborting the
whole local probe-family search on the first `UnknownClassification`. The
probe witness build steps inside those families now likewise skip
`UnknownClassification` candidate points instead of aborting the whole local
probe witness set when later certified witnesses still exist, and the
top-level bounded probe search from one interior witness now also backtracks
past uncertified normal or axis probe families instead of aborting before
later certified probe directions are tried. Leaf classification now also
backtracks past uncertified interior/side probe families and uncertified
probe reachability checks instead of aborting before later certified interior
witnesses or probe paths are tried. The
stricter replayable leaf cell
built from an interior witness now also contributes its own shifted witness
family and shifted exact vertices instead of collapsing to one witness plus
raw cell vertices, and the direct strict leaf witness family now also expands
through that stricter replayable leaf-cell construction instead of leaving it
only to the closed-cell geometry seed branch. Those stricter leaf-cell and shifted-edge
interior witness expansions now likewise backtrack past uncertified local
candidate searches instead of aborting the whole leaf witness family on the
first `UnknownClassification`, and the underlying strict leaf-witness build
steps now do the same candidate-locally for direct, shifted, shifted-vertex,
and shifted-geometry witness points. Shifted witness cells now also backtrack
past uncertified strictness checks on their raw shifted-vertex and raw
shifted-geometry seed sources instead of aborting that whole shifted witness
family before later certified candidates run. The remaining strict leaf/probe/
detour seed builders now also continue past an uncertified root halfspace
feasibility report by still searching the later exact-vertex and closed-cell
geometry seed families, instead of treating that root report as a hard local
failure before those exact seed families run. If one of those later
leaf/probe/detour witness families is itself uncertified after earlier
certified candidates already exist, the surviving candidates now keep that
uncertainty attached so later failure still surfaces
`UnknownClassification` instead of being flattened back into ordinary absence.
The same uncertainty now also stays attached across shifted witness-cell
construction itself, so later leaf/probe/detour targets built from a surviving
shifted witness still surface `UnknownClassification` if that local shifted
family was only partially certified.

Subdivision reference propagation currently accepts certified projected-child
reference targets, not just a single midpoint-filled representative point.
Existing references are reused only when they are strict child-cell interior
points and not on local surfaces. Otherwise hypermesh builds the projected
child cell that preserves every axis already strict in the parent reference,
then asks `hyperlimit` for strict witnesses, exact feasible vertices, and exact
closed-cell geometry seeds derived from those feasible vertices in that
projected cell, using every feasible vertex subset centroid of size two or
greater before tracing from the parent reference. If the first
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
If one of those fallback axis-defined targets later proves unusable and no
other certified reference target succeeds, that uncertified reconstruction now
still surfaces `UnknownClassification` instead of being flattened back into a
plain absent target.
If a whole projected witness or seed family has no certified target and only
uncertified candidates, that local family now returns
`UnknownClassification` so later projected/support/escape backtracking can
decide whether to keep searching or fail, instead of silently collapsing the
local family to an empty target set. The same rule now also applies across the
ordered shifted projected/support subfamilies inside one retained target
builder: an earlier uncertified strict-seed expansion no longer prevents later
raw shifted-vertex expansions from contributing certified targets, and only an
entirely uncertified local multi-family search surfaces `UnknownClassification`.
The deferred direct strict-seed pass now also backtracks past uncertified
strict seeds instead of aborting before later direct projected/support targets
run.
Direct feasibility witnesses inside those projected/support target builders now
participate in that same family backtracking order instead of being tried as a
special pre-pass that could cut off later strict-seed or exact-vertex families.
Those projected/support target builders now also continue past an uncertified
root halfspace report by still searching the later exact-vertex and closed-cell
geometry seed families, instead of requiring a certified report witness before
those exact seed families run.
If one of those projected/support local target families is uncertified after
earlier certified targets already exist, the surviving targets now keep that
uncertainty attached so a later trace failure still surfaces
`UnknownClassification` instead of being flattened back into ordinary absence.
The top-level projected direct/escape search now does the same: if every
projected direct trace, projected-support search, and projected escape search
is uncertified, that local projected-family search returns
`UnknownClassification` instead of silently collapsing to `None`, and only the
broader full child-cell support fallback boundary intentionally downgrades that
local failure so a later certified support-cell construction can still run.
Projected target-family construction now also tracks that same uncertified
state explicitly at the `compute_new_reference(...)` boundary: if projected
direct/escape target-family construction was uncertified and support fallback
still finds no witness, the final result is now
`UnknownClassification` instead of a plain
`ReferencePropagationFailed`. In particular, the root projected direct/escape
target-family builders now keep that uncertified state even when they return
an empty family and let support fallback continue, instead of surfacing
`UnknownClassification` before the support-side search runs at all, and when
those root or shifted projected families do return surviving targets after a
later uncertified local family, those survivors now keep that uncertainty
attached so a later failed trace does not flatten back into ordinary absence.
At the target level, a projected/support reference point that is strictly
inside the child cell but whose retained-definition trace is uncertified now
surfaces `UnknownClassification` instead of being treated like a simple absent
target. Within one feasible support cell, that same rule now also applies
across the whole retained target family: one uncertified target trace no
longer aborts the cell before later certified targets are tried. The outer
projected direct/escape search also no longer retraces the exact same
projected target just because it appears in both the direct and escape
families; it skips that duplicate direct trace and proceeds straight to the
escape-specific searches for that target. Within one live root projected
reference update, projected direct-target tracing now also reuses point-level
reference-validity checks for repeated target points and full retained-target
traces for repeated exact projected targets instead of repaying those exact
queries before the later escape layers run. For one projected target, the later
axis-corridor and tight escape-box searches now also reuse the same exact
axis stop families instead of recomputing those surface-crossing families
independently before each escape layer, and identical escape `Aabb` support
searches are now reused across those later escape layers instead of rerunning
the same support-cell search for duplicate bounds. At the root projected-cell
boundary, direct projected targets and projected-escape targets now also share
one exact halfspace report and one exact projected seed-family construction
instead of rebuilding that same root projected-cell evidence twice before
search begins, and for one shifted projected seed they now also share one
exact shifted projected-cell report and shifted projected seed-family build
across both the shifted direct-target layer and the shifted projected-escape
layer instead of recomputing that shifted-cell evidence twice per seed. The
support/reference fallback path now also reuses full reference-validity checks
for repeated target points across converged support states instead of repaying
the same `is_valid_reference_for_bounds(...)` query before each retained-target
trace attempt. The outer support-cell wrapper now also treats an uncertified
root feasibility check as another local backtracking point instead of aborting
support propagation before later support-side branches are explored.
It then tries local
axis-aligned
escape corridors across the ordered exact stop family from the next surface hit
out to the child AABB boundary, using `hyperlimit` witness search instead of
midpoint sampling and backtracking past uncertified or empty earlier corridor
searches; if every corridor in that local family is uncertified, that family
now surfaces `UnknownClassification` instead of collapsing to `None`. The
underlying exact stop family now also skips past partially uncertified local
surface crossings instead of aborting corridor construction before later exact
stops run, and that uncertainty is preserved through both the later
axis-corridor search and the tighter escape-box search. If those
direct one-axis corridors still cannot be traced, it next searches the ordered
exact escape-box family bounded by certified axis stop values and child AABB
faces around that projected target, asking `hyperlimit` for a replayable
halfspace-feasibility witness inside each box while backtracking over certified
slack sides of local support planes; if every escape box in that local family
is uncertified, it now also surfaces `UnknownClassification` instead of
silently collapsing. That tighter escape-box family now also skips past
partially uncertified local boxes instead of aborting before later exact boxes
run, and if only those later boxes remain but none certifies a witness, that
local family still preserves `UnknownClassification`. If that tighter escape-cell family is uncertified or still
cannot be traced, it falls
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
segment tracing and probe reachability now both follow the finite exact local
detour-point family with cycle-guarded nested detour retries on those legs
instead of collapsing every detour leg to the no-detour family immediately or
stopping at a generic local polygon-count recursion cap. Individual
plane-replacement steps now do the same within one replacement step while still
stopping short of recursive step-detoured plane-replacement. If none trace
cleanly, it reports
`ReferencePropagationFailed` instead of using random/interior sampling. The
reference point carries retained plane triples, and projected / projected-escape
references now keep certified halfspace-derived plane definitions when those
can be reconstructed instead of collapsing every witness back to one axis-plane
triple. Closed projected child cells now also retain any remaining strict
direct witnesses of the cell, including strict exact feasible vertices, instead
of using those points only as shifted-seed sources. The projected escape-target
family now retains those same remaining strict direct projected witnesses before
it widens into shifted projected escape cells, and it no longer rebuilds direct
targets whose points are already present in the projected target family.
Shifted projected/support direct-target builders now also dedupe later shifted
seed families against the report witness itself, so they do not rebuild that same direct
target again through later strict-seed, exact-vertex, or geometry-seed families.
The top-level projected/support target builders now do the same before they
widen into shifted target search from those same exact seed families.
Support-cell witnesses are now constructed from
closed support-side cells by enumerating exact feasible cell vertices and
asking `hyperlimit` for replayable witnesses inside inward-shifted strict
cells. When `hyperlimit` already provides a strict feasible witness for the
closed support cell, hypermesh now tries that richer direct witness first, and
support-cell search now also accepts the current feasible child/support cell
before forcing any further support-side assignment, then shifted replayable
witnesses built from every available strict support-cell
witness, every exact feasible support-cell vertex, and exact closed-cell
geometry seeds derived from every feasible support-cell vertex subset of size
two or greater, and finally any
remaining strict direct witnesses of the closed cell. That lets reference
propagation backtrack across multiple certified direct and shifted targets
inside one feasible support-side cell instead of collapsing the cell to one
point. Each shifted support cell now also contributes every strict target
recovered from its own certified witness family and its own exact feasible
vertices and raw geometry seeds instead of only the first feasibility witness
selected by the halfspace predicate, and uncertified shifted support seeds or shifted support
vertices now no longer abort the whole support-cell target family. Witness
points whose retained plane-definition reconstruction is uncertified are now
also skipped candidate-locally instead of aborting those support-cell target
families. Direct projected/support seed collection now also skips uncertified
strictness checks candidate-locally instead of aborting the whole seed family.
Projected/support target construction now also backtracks across wholly
uncertified strict-seed and raw-vertex subfamilies instead of aborting the
whole local target builder before later exact families run. As with the
projected-child families, those candidate-local skips now surface
`UnknownClassification` when an entire local projected/support seed or target
family is uncertified and no certified witness survives, rather than silently
degenerating that family to empty. When those shifted target builders receive
overlapping strict-seed, exact-vertex, and geometry-seed families, they now
dedupe those seeds in first-occurrence order before rerunning the same shifted
cell construction from multiple equivalent points, and the projected-side seed
builders now reuse one exact feasible-vertex / geometry-seed family per cell
instead of recomputing those raw families separately for strict and shifted
search. Projected escape-target construction now reuses that same per-cell
projected seed family instead of rebuilding it before widening into escape
targets, and support-side seed builders now do the same before widening into
shifted support-cell target search.
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
support-cell fallback now also backtracks across alternate support-side
branches when one candidate branch returns `UnknownClassification`, rather
than aborting the whole search on the first uncertified cell. An uncertified
feasibility report on the current support cell now also falls through to
later support-side branches instead of cutting off that broader search at the
current node, the current support cell is still allowed to attempt certified
target construction without a report witness when later exact seed families
suffice, and support-side branches whose feasibility precheck is
uncertified are now searched instead of being discarded immediately. When
support fallback does branch on a support plane, it now tries the side
containing the current reference point first before backtracking across the
opposite side, and repeated identical support-side halfspaces no longer
spawn redundant duplicate branch states. Exact opposite support halfspace
pairs are now also skipped before any feasibility query or deeper recursion.
Support-cell search now also prunes any state that already forces the
reference onto a local support plane before report or acceptance queries run,
and once the current halfspace state already fixes a polygon's support side,
later unchanged recursion through repeated fixed support-plane states is
skipped entirely. Identical support halfspace states now also reuse cached
feasibility/report queries at the live support-reference boundary instead of
reissuing the same exact halfspace query family each time that state is
revisited, and repeated retained-definition trace attempts on the same support
target are now reused at that same boundary instead of retracing identical
reference targets when support states converge. Repeated support-surface
rejection of the same target point is now also cached across that live
support-reference search boundary, so converged support-target families do not
repay the same local support-plane test on each revisit. Identical support
states also
reuse the exact support target-family construction itself before those traces
run, instead of rebuilding the same retained target family on each revisit, and
the full accepted support-reference result is now reused on identical
support-state/report revisits instead of rerunning that whole acceptance layer.
Identical `(polygon_index, support halfspace state)` recursive support-search
states are also now reused instead of replaying the same downstream accept and
branch search after converging to that exact state again.
The
support/reference target trace search now also skips support-surface targets
before retained-definition tracing runs, instead of paying for a trace only to
reject that target afterward. The
exact support/projected vertex family now also skips candidate-local
`UnknownClassification` membership checks instead of aborting the whole
vertex family on the first uncertified candidate, and the leaf/probe-side
exact halfspace-cell vertex family now does the same. The
projected-escape direct-seed family now also backtracks past uncertified
membership checks instead of letting one uncertified deferred direct seed cut
off later certified direct escape targets. The
focused reference tests now also cover this support-cell fallback on prepared
closed-mesh polygons, not only on synthetic support-plane fixtures. Full
EMBER plane-replacement
coverage for every reference construction remains unfinished.
The subdivision-entry support-fallback slice is also now checked against the
public boolean path on the prepared closed-face union fixture, so that
alternate support-reference propagation is covered above the private helper
layer as well.
Leaf probe reachability now also gives retained plane-replacement steps one
lower definition-based reachability retry of their own before the search gives
up, without opening another nested step-detoured plane-replacement layer, and
arrangement-detour reachability now backtracks past uncertified detour legs
instead of aborting the whole probe search on the first uncertified detour.
An uncertified retained-definition no-detour reachability family now also
falls through to the detour search layer instead of aborting before any
certified detour family is tried, and only surfaces
`UnknownClassification` if no certified detour path succeeds.
Likewise, an uncertified direct geometric probe-reachability check no longer
blocks retained-definition plane-replacement reachability from being tried;
that direct layer now leaves the fallback decision to the retained-definition
search instead of failing one layer too early.

`EmberConfig::default()` runs only the general subdivision/BSP/classification
path. The previous same-surface, disjoint-bound, strict-containment,
boundary-contact, and oriented-box shortcuts have been removed, so public
boolean results either certify through the general path or return an error.
The public `boolean_operation` entry now certifies closure directly on the
classified polygon arrangement instead of requiring triangulation cleanup to
succeed before returning that arrangement.
The public regression suite now also exercises crossing octahedra and
affine-box overlap through that normal subdivision/reference path, not only
through the root one-leaf classifier. The suite now also pins the
"no strict contained source vertex" precondition on the crossing-octahedra
fixture explicitly, alongside those public Boolean-path regressions.
Shared-face box contact is now likewise covered through that public general
path as separate per-op regressions, so each Boolean op stays individually
bounded and debuggable instead of hiding behind one oversized bundled case.
The regression suite now also forces the root certified leaf classifier
(`max_depth: 0`) through same-surface contact, shared-face/shared-edge/shared-vertex
box contact, partial-face contact, nested closed containment, disconnected
closed containment, crossing octahedra, and affine-box overlap cases instead
of relying on deeper subdivision to rescue those paths.
Subdivision first applies conservative per-component reachable winding ranges.
If those ranges still allow the Boolean indicator, it then checks exact local
WNV-transition reachability across the full transition family instead of
stopping at a fixed reachable-state cutoff. A task is discarded only when the
range precheck or the exact reachable winding set makes the Boolean indicator
impossible. Full arrangement-complete finite-automaton WNV reachability
remains an implementation target.

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
polygons. When those child-count metrics tie, split ranking now prefers the
candidate with the lower exact post-split child intersection load before it
falls back to source kind. The recursion now backtracks across that ordered exact local split
family instead of committing to one chosen split candidate: if a higher-ranked
split hits `UnknownClassification`, `ReferencePropagationFailed`, or
`SubdivisionDepthLimit`, later exact local split candidates are still tried
before the task gives up. When split counts tie, exact arrangement/intersection
candidates now win over raw midpoint cuts instead of inheriting the old
midpoint-first insertion order, and duplicate midpoint-valued candidates are
now promoted when a later exact arrangement/intersection source reaches the
same split plane. If a task reaches `max_depth` while the bounds remain
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
one centroid-style representative point, and repeated coplanar overlap splits
now stop at an existing matching local BSP split plane instead of replaying the
same exact branch split through both subtrees again. The leaf BSP builder also
dedupes overlap boundary planes across the local pairwise overlap family before
submitting them to the BSP at all. Splittable tasks now also try
that same certified leaf path exactly once before subdivision, regardless of
former leaf-threshold sizing, and if that exact leaf attempt still returns
`UnknownClassification` while the bounds remain splittable, hypermesh keeps
subdividing instead of treating heuristic leaf sizing as a hard completeness
boundary. When a split would otherwise recurse on the exact same polygon family
with an empty opposite child, the surviving child now contracts to the exact
local polygon-family bounds instead of repeatedly bisecting empty space around
that unchanged arrangement. Unsplittable tasks now also run the exact leaf
processor directly once instead of first retrying the same uncertified path
through the certified leaf-output helper. That lets exact local arrangement
isolation continue until the depth budget or a certified leaf result stops the
branch. Hypermesh reports
`SubdivisionDepthLimit` if the configured depth budget is reached before the
current task can be certified as a leaf, and it reports
`UnknownClassification` if leaf classification or this isolation check fails
before appending output outside the depth-limit branch. Certified BSP leaf
validation and coplanar effective-`delta_w` accumulation now also share one
exact leaf-analysis pass instead of rebuilding the same local leaf polygon,
witness family, and per-polygon leaf-test relations twice per fragment, and the
same certified leaf interior witness family is now reused directly by BSP-fragment
leaf classification instead of being rebuilt again inside `classify_leaf_polygon`.
Exact repeated direct/BSP fragment classifications with the same support, edge
cycle, and `delta_w` inside one subdivision task now also reuse the same
certified winding trace instead of retracing equivalent fragments before output
dedupe removes them, and exact duplicate BSP leaf edge cycles are now skipped
before leaf certification and coplanar `delta_w` analysis run at all.
Full
arrangement-isolation termination is still an implementation target.

`triangulate_and_resolve_certified` resolves exact duplicate vertices,
duplicate faces, and T-junctions, but refuses non-empty outputs with boundary
edges or zero signed volume instead of capping or peeling them. Non-manifold
edge valence is allowed for closed PWN output. If exact T-junction/crossing
resolution does not converge within its certification budget, it reports
`OutputResolutionLimit`. Hypermesh does not expose a repairing triangulation
path; if the classified arrangement is not emitted closed by construction, the
operation fails certification. The emitted polygon arrangement is now checked
for exact boundary closure before any triangulation cleanup runs, so
`resolve_tjunctions` only cleans triangle-soup representation artifacts and is
not allowed to turn an open polygon arrangement into a certified result.
Exact duplicate oriented output polygons are now also suppressed when the
classified arrangement is materialized into `BooleanResult`, so shared-face and
coplanar duplicate surfaces are reduced before closure checking and
triangulation cleanup run. The same exact-geometry duplicate suppression now
also runs earlier at subdivision emission time, so duplicate classified
polygons are merged before they ever reach the final classified arrangement.
The exact closure check now also caches split subedges per undirected polygon
edge, so repeated coincident segments do not rescan and re-sort the same merged
vertex chain before counting boundary usage.
`certify_output_polygon_closure` exposes that pre-triangulation check directly
for callers and regressions that want to validate closure on the classified
polygon arrangement itself.

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
