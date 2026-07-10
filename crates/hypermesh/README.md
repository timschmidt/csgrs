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

Closed-PWN validation is geometric and oriented: every exact undirected edge
class must have equal forward and reverse triangle uses. Ordinary singleton
boundary edges report `OpenInput`; closed-valence soups whose directed edge
multiplicities do not cancel report `NonPwnInput`. Balanced non-manifold edge
multiplicity remains supported by this boundary check.
The public general path now also exercises that support boundary with doubled
and opposite-oriented canceling closed tetrahedra: coincident strict segment
crossings retain every PWN transition, while coplanar shared-edge events pair
boundary incidences per sheet. Vertex and unmatched boundary crossings remain
uncertified and force alternate-path search instead of collapsing coincident
multiplicity. The classified arrangements are required to reduce the doubled
surface to one closed Boolean boundary with the exact tetrahedron volume and
the canceling surface to an empty boundary before cleanup.

Within that model, the current runtime claim is deliberately narrower than a
blanket completeness claim:

- If hypermesh returns `BooleanResult`, the result came from the general EMBER
  subdivision/BSP/classification path with certified winding data and certified
  arrangement closure; public success must not depend on special-case boolean
  shortcuts or output repair.
- If the current EMBER search cannot certify a required sign, incidence,
  reachability, reference-propagation step, leaf classification, or output
  closure fact, hypermesh reports an explicit error such as
  `UnknownClassification`, `ReferencePropagationFailed`,
  `SubdivisionDepthLimit`, `OpenOutput`, or `OutputResolutionLimit` instead of
  silently widening the support claim.
- Completion is not yet claimed for the whole intended closed-PWN model. The
  remaining finite-family search structure means that some intended-model
  inputs can still fail with those explicit errors even when a complete EMBER
  implementation would certify them.

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
plane-replacement tracer also now caches exact repeated intermediate steps
across axis orderings inside one retained path, so equivalent one-plane updates
do not retrace the same local segment family over and over before that broader
search moves on. The same plane-replacement trace caches are now also shared
across sibling retained-definition pair attempts in one step-detoured search,
so converged later replacement steps do not retrace the same local segment
family again after an earlier pair already explored them. The retained
definition no-detour plane-replacement trace now shares those same affine and
step caches across sibling definition-pair attempts too, instead of rebuilding
them for each pair. The retained-definition segment entry now also keeps its
definition-aware no-detour trace cache and endpoint-box detour-family cache
alive from the first direct retained query into the later step-detoured
replacement search, instead of rebuilding those top-level caches for every
later retained subquery. Before any retained plane triple enters those trace
or reachability searches, its affine reconstruction is checked against the
declared endpoint. A decidable mismatch is discarded so winding propagation
cannot silently start from a stale point; a singular or otherwise undecidable
triple is excluded from execution while its uncertainty is preserved if no
certified alternative succeeds, and the exact axis definition of the endpoint
is always included. Successful subdivision reference states now enforce the
corresponding storage invariant as well: inherited, projected, support, reused,
and cached child references retain only plane triples whose affine
reconstruction equals the stored point, collapse plane-set duplicates, and
synthesize the exact axis triple when no retained certificate survives. Stale
or singular triples can no longer persist as claimed construction metadata in
later child tasks. The reachability-side retained plane-replacement
fallback now keeps that same no-detour reachability cache and detour-family
cache alive across sibling retained replacement steps too, instead of
rebuilding them for each later subquery. The same local
plane-replacement walk now also reuses exact
`affine_from_planes(...)` results across sibling orderings, so equivalent
intermediate plane triples do not repay the same exact point reconstruction
before trace or reachability continues, and the reachability-side plane
replacement walk now also caches exact repeated intermediate step checks across
those sibling orderings instead of rerunning the same local adjacency test
family. Those reachability-side affine and step caches are now also shared
across sibling retained-definition pair attempts in one step-detoured search,
so converged later replacement steps do not recheck the same local adjacency
family again after an earlier pair already explored them. The retained-definition entry path
above the step-detoured
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
direct witnesses. When one strict halfspace-cell seed family already returns
surviving strict witnesses after a partially uncertified candidate-local
search, that uncertainty now stays attached across later sibling seed families
instead of being flattened back into an ordinary certified seed family.
Partially uncertified feasible halfspace-cell vertex families now also keep
that uncertainty attached when later exact vertices survive, so the later
strict halfspace seed families and geometry-seed families do not start from an
apparently fully certified shifted vertex family. The same now holds one layer
deeper for centroid-based geometry-seed subset families on both the leaf and
reference sides: one uncertified subset centroid no longer aborts the whole
local geometry-seed family before later exact subset centroids run.
The reference-side direct strict projected/support seed filters now also treat
exact child-boundary candidates as `UnknownClassification` instead of ordinary
non-strict seeds, so later certified strict seed families still run with that
uncertainty preserved. The same boundary-aware rule now also applies one layer
later in the direct report-witness and shifted target-family builders, so an
exact child-boundary witness no longer gets skipped as ordinary absence before
later certified projected/support targets run.
The leaf/probe halfspace-cell side now follows the same rule: exact probe-cell
boundary candidates in the direct strict seed filter and shifted witness-family
builder now count as `UnknownClassification` instead of ordinary non-strict
rejection before later certified witness families run.
Likewise, shifted halfspace witness collectors now treat surviving fallback-marked
shifted witnesses as uncertainty that carries forward across later sibling
seeds and sibling seed families, instead of only noticing hard
`UnknownClassification` returns. The downstream leaf/probe point collectors now
do the same for fallback-marked surviving interior/probe candidates, so later
sibling candidate families no longer look fully certified just because an
earlier partially uncertified build returned some output. The detour-target
collectors now do the same for fallback-marked surviving detour candidates and
detour families, so valid-path fallback does not silently treat later sibling
detours as fully certified after an earlier partially uncertified local build.
That carry-forward rule now uses one terminal family invariant for shifted
halfspace witnesses, leaf points, probes, and detours: a hard unknown or any
nonredundant fallback marks every materialized survivor, even when another
sibling was built exactly. A complete membership/reachability/winding path can
still certify its chosen survivor, while an all-survivor failure returns
`UnknownClassification`. Exact same-state fallback duplicates are merged into
their certified construction instead of creating artificial family
uncertainty.
Direct detour-target construction from those strict witnesses
now also backtracks past uncertified target builds instead of aborting before
later certified direct or shifted detour targets run, and the surrounding
endpoint-box detour family now also backtracks past uncertified local boxes
instead of aborting before later certified detour boxes run. The earlier
axis-interval surface-cut construction feeding those endpoint boxes now also
skips past partially uncertified local surface crossings instead of aborting
the whole detour-box family before later exact boxes are even formed, and
exact boundary contacts on those local polygon cuts now count as
`UnknownClassification` for that cut candidate instead of ordinary accepted
surface membership. The same detour-box layer now treats exact start-point and
endpoint surface contact the same way, so start-on-plane and endpoint-on-plane
local cuts no longer disappear as ordinary no-crossing before later exact
detour boxes run.
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
witness builders now also promote the report witness into the shifted seed
root when it is not already a strict direct seed, while still deduping later
strict-seed, exact-vertex, and geometry-seed families against that promoted
root so they do not rebuild the same direct witness again,
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
abort the whole corridor family before later exact corridors run, and exact
start-on-plane, boundary, and endpoint-on-plane contacts on those local axis
corridors now count as
`UnknownClassification` for that corridor candidate instead of ordinary stop
membership. Full
Normal-direction probes now do the same: they walk the ordered exact stop
family out to the child boundary instead of stopping at one certified normal
corridor, and partially uncertified local normal crossings no longer abort the
whole corridor family before later exact corridors run, while exact boundary
contacts at the start point or along a local normal crossing now count as
`UnknownClassification` for that corridor candidate instead of ordinary stop
membership. The live leaf-classification path now also searches those normal
corridors progressively, trying each strict retained-definition family before
building later corridor families, so successful BSP-fragment leaf
classifications no longer materialize the full normal-probe family up front.
Full
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
The direct axis-ordered segment path now also keeps trying later orderings
when one intermediate surface-membership check or one axis-aligned leg trace
is uncertified, instead of aborting the whole ordered-path search before those
later exact orderings run.
Direct segment tracing now also treats unmatched on-edge crossings as
`UnknownClassification` instead of flattening them into an ordinary invalid
path result. Exact leg endpoints that land on a traced polygon now also count
as `UnknownClassification` instead of being silently skipped by the direct
crossing collector. Zero-length direct traces and zero-length axis-segment
legs now also reject exact traced-surface contact as
`UnknownClassification` instead of silently passing it through as a valid
empty trace. Axis-ordered trace search now applies the same rule before any
legs run, so a zero-length retained path cannot bypass traced-surface
uncertainty either. The traced-surface rejection layer used by axis-ordered
direct search and detour fallback now also treats exact boundary contact on a
traced polygon as `UnknownClassification` instead of ordinary surface
membership, so later exact orderings or detours still run.
Direct adjacent-cell reachability now does the same for exact blocker-surface
contacts: boundary hits surface as `UnknownClassification`, while strict
interior blocker crossings remain ordinary blocked paths. Zero-length probe
reachability checks now also reject exact blocker-surface contact as
`UnknownClassification` instead of silently collapsing to an ordinary
unreachable result. Retained-definition plane-replacement trace and
reachability steps now also consult their step tracers even when two distinct
definition triples land on the same affine point, so same-point definition
updates cannot silently bypass uncertified local contact.
The live cycle-guarded detour layers now also allow same-point detours when
they introduce a new retained-definition family at the current endpoint,
instead of treating every revisited point as an automatic skip before that
zero-length definition transition can be tried.
The reachability-side plane-replacement walk now does the same for its
intermediate adjacency checks: one uncertified replacement leg only invalidates
that ordering, rather than cutting off later exact plane-replacement orderings.
Strict leaf-witness replay now does the same for stale active-plane indices
instead of collapsing immediately to support-plus-axis interior replay when
coincident leaf-cell planes are still available.
If leaf-interior or probe definition reconstruction is itself uncertified, the
fallback axis-defined candidate is still explored, but if no later certified
probe family or probe path succeeds that branch now surfaces
`UnknownClassification` instead of being flattened into plain absence.
Once a fallback-marked interior/probe pair passes strict leaf membership,
off-surface probe validation, certified adjacent-cell reachability, and the
complete reference-to-probe winding trace, the resulting winding now counts as
certified. Uncertainty from an unused richer point-definition replay no longer
rejects that complete proof. A fallback candidate that is skipped, unreachable,
or fails to trace still contributes `UnknownClassification` if no later probe
path succeeds.
Redundant same-point fallback duplicates in the live leaf/probe/detour
collectors also no longer poison an exact certified duplicate with the same
retained definitions; fallback at one point now survives only when it still
contributes unresolved local state there.
The same now applies to fallback-built detour targets: if an uncertified
axis-defined detour is later skipped or cannot certify either leg, that local
detour family also surfaces `UnknownClassification` instead of reading as an
ordinary missing detour. The same rule now also holds in the cycle-guarded
step-detour helper used by plane-replacement reachability, so fallback-built
step detours no longer collapse back to plain `false` when they are skipped or
their later legs cannot certify a path. The cycle-guarded runtime detour paths
now also preserve that uncertainty when a fallback-built detour is skipped
because it revisits the current start/end or an already-visited detour point,
instead of flattening that skip back into ordinary absence. Once a
fallback-built detour passes strict-cell construction, off-surface validation,
and every winding or reachability leg in its complete path, that path now
counts as certified. Uncertainty from unused richer detour definitions no
longer rejects a proven recursive, budgeted, progressive, or breadth-first
detour path. The same
cycle-guarded reachability layer now also keeps searching detours after an
uncertified direct no-detour adjacency check, instead of aborting detour
fallback before any later certified detour path runs.
Definition-preserving normal-probe search also now
also augments, rather than suppresses, the broader certified normal-corridor
witness family when both are available, and axis-direction probe search now
does the same for retained interior definitions whose non-support planes
preserve the moved axis. Both retained-definition probe families now also
backtrack past uncertified local candidate searches instead of aborting the
whole local probe-family search on the first `UnknownClassification`. The
probe witness build steps inside those families now likewise skip
`UnknownClassification` candidate points instead of aborting the whole local
probe witness set when later certified witnesses still exist, and exact
support-plane contact in the strict probe and strict axis-probe builders now
also counts as `UnknownClassification` instead of silently disappearing as
ordinary absence. Exact halfspace-cell boundary contact in those same strict
probe and strict axis-probe witness builders now also counts as
`UnknownClassification` instead of flattening to ordinary non-membership. The
top-level bounded probe search from one interior witness now also backtracks
past uncertified normal or axis probe families instead of aborting before
later certified probe directions are tried. Leaf classification now also
backtracks past uncertified interior/side probe families and uncertified
probe reachability checks instead of aborting before later certified interior
witnesses or probe paths are tried. Normal- and axis-probe corridor search now
retains an exact interior boundary hit as a stop barrier while preserving its
uncertain winding transition, so candidate probes can be placed strictly before
a shared triangle edge instead of overshooting it. Normal corridors try the
exact halfway point to each ordered stop before expanding the broader strict
witness family; the ordinary surface, reachability, and winding checks still
certify that candidate before it can classify a leaf. Those corridor searches
also keep exact bound-stop support-plane contacts visible to the local polygon
classifier instead of dropping them as missing crossings, and exact zero-room
bound-start contacts now count as local `UnknownClassification` instead of
ordinary empty corridor families. Later certified corridors still run and the
surviving family keeps uncertainty attached when those endpoint or bound-start
contacts are exact. The
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
and shifted-geometry witness points. Exact leaf-boundary contact in the strict
leaf witness builders now also counts as `UnknownClassification` instead of
silently disappearing as ordinary absence, and the direct strict leaf seed
filter and shifted-edge replay layer now do the same for exact leaf-boundary
candidates before later certified witness families run. Shifted witness cells now also backtrack
past uncertified strictness checks on their raw shifted-vertex and raw
shifted-geometry seed sources instead of aborting that whole shifted witness
family before later certified candidates run. When that direct strict leaf
witness seed-family construction is only partially certified, the surviving
leaf witnesses now keep that uncertainty attached instead of being flattened
back to ordinary certified witnesses. The remaining strict leaf/probe/
detour seed builders now also continue past an uncertified root halfspace
feasibility report by still searching the later exact-vertex and closed-cell
geometry seed families, instead of treating that root report as a hard local
failure before those exact seed families run. If one of those later
leaf/probe/detour witness families is itself uncertified after earlier
certified candidates already exist, the surviving candidates now keep that
uncertainty attached so later failure still surfaces
`UnknownClassification` instead of being flattened back into ordinary absence.
The leaf-witness layer now does the same one level higher too: if the direct
strict leaf seed family is empty and partially uncertified, later shifted
vertex or shifted-geometry seed sources still run instead of being cut off
before shifted witness construction starts.
The same uncertainty now also stays attached across shifted witness-cell
construction itself, so later leaf/probe/detour targets built from a surviving
shifted witness still surface `UnknownClassification` if that local shifted
family was only partially certified. When duplicate shifted witness cells
rediscover the same strict point, hypermesh now also keeps every distinct
active-plane/halfspace family for that point instead of collapsing the witness
back to one first-arrival family, while still deduping geometrically identical
families when the same shifted halfspace state is rediscovered in a different
local halfspace order. The direct retained-definition builders behind those
leaf/probe witnesses now also keep partial uncertainty when one candidate
plane triple cannot be replayed exactly but a later exact definition still can,
instead of flattening that local replay search back into an apparently fully
certified witness.

Subdivision reference propagation currently accepts certified projected-child
reference targets, not just a single midpoint-filled representative point.
Existing references are reused only when they are strict child-cell interior
points and not on local surfaces. Otherwise hypermesh builds the projected
child cell that preserves every axis already strict in the parent reference,
then asks `hyperlimit` for strict witnesses, exact feasible vertices, and exact
closed-cell geometry seeds derived from those feasible vertices in that
projected cell, using every feasible vertex subset centroid of size two or
greater before tracing from the parent reference.

Reference-target traces are confined to the exact child or local search AABB,
expanded only as needed to include the inherited reference point. Retained
plane-replacement orderings whose affine start or intermediate point leaves
that trace box are skipped, and detour targets outside the same box are treated
as blocked. The detour search exhausts the endpoint-derived boxes first, then
adds exact candidates from the full trace box; when polygon clipping leaves no
non-endpoint arrangement cell, it also searches the unsplit trace box for a
certified route around finite polygons. Cached detour families include that
trace AABB in their key, so a family built for one local domain cannot certify
another.

If the first
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
plain absent target. Conversely, once strict target validity and the complete
reference winding trace both succeed, the exact axis-defined target is promoted
to a certified reference: its retained definitions are normalized to triples
that reconstruct the point, and uncertainty from an unused richer definition
family no longer rejects that proven propagation path.
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
That now also applies when one sibling projected/support family already
returns surviving fallback-marked targets: later sibling families inherit that
same uncertainty instead of being misread as fully certified just because the
partially uncertified family produced output.
The candidate-local projected/support target collectors now do the same before
those sibling families are even formed: if one candidate build already returns
a fallback-marked target, later candidate targets inherit that uncertainty
instead of being treated as fully certified siblings of a partially uncertified
candidate. The direct retained-definition replay inside
`reference_target_from_halfspace_witness(...)` now follows the same rule:
later exact active-halfspace definitions still survive after an earlier
unreplayable plane triple, and the resulting `ReferenceTarget` keeps that
uncertainty attached instead of looking fully certified.
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
stops run, and exact boundary contacts on those local axis-surface crossings
now count as `UnknownClassification` for that stop candidate instead of
ordinary stop membership. Exact start-on-plane and endpoint-on-plane contacts
now count the same way instead of disappearing as ordinary no-crossing before
later exact corridors run. That uncertainty is preserved through both the
later axis-corridor search and the tighter escape-box search. If those
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
paths hit surfaces. Equivalent axis-ordered segment legs now also reuse the
same exact `trace_axis_segment(...)` result across sibling orderings instead of
rerunning the same leg trace before detour fallback begins. Those detour
points are now constructed from each
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
stopping short of recursive step-detoured plane-replacement, and the live
cycle-guard entry now also reuses identical definition-aware no-detour trace
and reachability queries across revisited branches instead of reissuing those
same retained-definition base checks. The same entry layer now also reuses
identical endpoint-box detour target families for repeated `(start, end)` or
reversed `(end, start)` queries instead of rebuilding the same local detour
boxes across revisited branches. Complete local detour construction is now
resumable by endpoint-box cell, direct and deferred target phase, and individual
shifted seed. Both winding-carrying segment traces and no-plane-replacement probe
reachability consume those batches through breadth-first path queues, so later
target phases and shallower sibling paths run before one blocked leg can
recursively monopolize the search. Segment-path expansion propagates the exact
winding vector across every already-certified leg and accepts a fallback-built
target only after the complete path succeeds. The strict-AABB detour cursor now
exhausts its already-built direct witnesses before expanding shifted seed
families, while retaining every shifted family as a later fallback. If that
cursor emitted earlier target batches but also skipped an uncertified family,
exhaustion still surfaces `UnknownClassification` after the emitted paths fail;
partial emission no longer flattens the omitted family into ordinary absence.
Fully materialized detour families carry the same uncertainty on every
surviving target, where a complete successful path discharges it and an
all-target failure preserves it.
Detour paths now also use each unique local polygon support plane as an exact
arrangement-cell signature.
Those open cells are convex and disjoint from every local polygon surface, so
the breadth-first search globally enqueues only the first certified-preferred
geometric target for each cell while same-point definition transitions remain
available. A winding trace whose endpoints already have the same open-cell
signature now certifies the unchanged winding directly; points on any support
plane remain excluded and still require retained-definition propagation. Exact
plane/AABB extrema also discard endpoint boxes whose strict
interiors lie entirely in an endpoint cell before their witness families are
generated. This makes negative searches finite without an arbitrary path-depth
or work cap. The top-level retained-definition probe-reachability entry now uses
that same batched breadth-first arrangement-cell search instead of recursively
exhausting one progressive endpoint-box family before its siblings. The live step-detour
reachability entry now
also reuses those same definition-aware no-step-detour checks and endpoint-box
detour families across failed sibling branches instead of rebuilding them on
each revisit. If none trace
cleanly, it reports
`ReferencePropagationFailed` instead of using random/interior sampling. The
reference point carries retained plane triples, and projected / projected-escape
references now keep certified halfspace-derived plane definitions when those
can be reconstructed instead of collapsing every witness back to one axis-plane
triple. Closed projected child cells now also retain any remaining strict
direct witnesses of the cell, including strict exact feasible vertices, instead
of using those points only as shifted-seed sources. The projected escape-target
family now retains those same remaining strict direct projected witnesses before
it widens into shifted projected escape cells, and same-point report/direct
escape witnesses are now still allowed to merge additional retained plane
definitions into an existing projected target instead of being dropped early on
point equality alone.
The same point-level rule now also applies to the direct projected/support
target family itself: a strict direct seed that lands on the report witness no
longer gets skipped before it can merge extra retained plane definitions into
that same-point `ReferenceTarget`.
And if a fallback-marked same-point `ReferenceTarget` later gets an exact
certified duplicate with the same retained definitions, that redundant fallback
copy no longer poisons the merged target or the surrounding family collector.
And when the report witness is not itself a strict direct seed, it is now still
used as a shifted-search root in the projected/support target family instead of
being treated only as a direct witness. That keeps later shifted target
construction from silently missing alternate reference paths rooted at the same
exact witness point.
The same promotion now also applies to the projected escape-target family:
when the report witness is not itself a strict direct projected seed, it is
still used as a shifted projected-escape root instead of being treated only as
the direct report witness.
And if one projected escape sibling family is only fallback-certified, later
projected escape siblings now inherit that uncertainty after the merged
same-point target set is reconciled, instead of looking certified unless a hard
`UnknownClassification` happened.
And if one of those projected escape targets is already fallback-marked, later
axis-escape and tight-escape failure now preserve that uncertainty instead of
flattening the exhausted escape branch into ordinary absence.
Fallback-marked projected/support `ReferenceTarget`s are also no longer
accepted as final success just because their winding trace happens to succeed;
the search now keeps looking for a certified target and only returns success on
that certified target. The same contract now also applies to the indirect
projected-support, axis-escape, and tight-escape branches: if one of those
helpers produces a fallback-marked success tuple, projected reference search
keeps looking for a certified later result and otherwise returns
`UnknownClassification`. The lower axis-escape and tight-box helper searches now
also enforce that rule locally, so a fallback-marked corridor or escape-box
result can no longer look like certified helper success before the broader
projected-reference search sees it.
Projected axis-stop corridor search now also treats exact zero-room bound-start
contact as local `UnknownClassification` instead of ordinary empty stop family,
matching the later start- and endpoint-boundary handling in that same
reference-side search layer.
Shifted projected/support direct-target builders now also dedupe later shifted
seed families against the report witness itself, so they do not rebuild that same direct
target again through later strict-seed, exact-vertex, or geometry-seed families.
The top-level projected/support target builders now do the same before they
widen into shifted target search from those same exact seed families.
And inside the already-shifted projected/support target-family builders, a
fallback-marked report witness can now still be replayed as a same-point
strict seed, so a later certified duplicate at that witness point can clear
redundant fallback state instead of leaving the merged shifted target
uncertified forever.
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
builders now reuse one exact feasible-vertex family per cell when deriving raw
geometry seeds instead of recomputing the same feasible vertex enumeration for
both shifted-vertex and geometry-seed paths. Projected escape-target
construction now reuses that same per-cell projected seed family instead of
rebuilding it before widening into escape targets, and support-side seed
builders now do the same before widening into shifted support-cell target
search.
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
revisited. Feasibility-first support states now also prime and reuse that same
report cache before the later accept/trace layer asks for the full witness, so
one exact support state no longer pays both a feasibility predicate and then a
separate full report query. Repeated retained-definition trace attempts on the
same support
target are now reused at that same boundary instead of retracing identical
reference targets when support states converge. Repeated support-surface
rejection of the same target point is now also cached across that live
support-reference search boundary, so converged support-target families do not
repay the same local support-plane test on each revisit. Within one
`compute_new_reference(...)` update, projected support, axis-escape support,
tight-escape support, and final support fallback now also share those same
safe support-reference query caches instead of rebuilding the same top-level
halfspace/report/trace state for each later support-side attempt, and the same
bounds-aware support target-family, accept, recursive support-search,
reference-validity, and retained-definition trace caches now also stay alive
across those later support-side calls. Equivalent support halfspace families
now also hit those support-side caches when the same local halfspaces arrive in
a different order, and the report-sensitive target/accept layers now also treat
active-plane index permutations on the same geometric halfspace witness as the
same support state instead of missing reuse on representation-only differences.
When root projected-cell setup already classified the projected halfspace state,
that exact report/unknown result now also primes the shared support-side
halfspace caches before projected-support fallback begins, so the same
projected root state is not reclassified again just to enter support search.
The same per-update projected/support support-search cache set now also reuses
the feasible support-cell vertex family and derived geometry-seed family for
identical halfspace states, so projected root-family construction and later
projected/support target construction do not rebuild that exact seed geometry
from scratch on each revisit. The centroid-subset geometry-seed family is now
also memoized by the exact feasible support-vertex family itself, so distinct
projected/support halfspace states that collapse to the same support vertices
reuse the same subset-centroid seed construction instead of recomputing it.
Support-side seed/direct/target/accept caches now also collapse all halfspace
reports that do not carry a usable feasible witness into one shared cache
state, so infeasible certificate variation does not fragment later support
fallback reuse when the builders would ignore that report payload anyway. The
seed-family and direct-target layers now also treat two feasible reports with
the same witness point as the same cache state even if their active-plane
metadata differs, because those two builders only use the witness itself.
Partially uncertified feasible support-cell
vertex families now also keep that uncertainty attached when later exact
vertices survive, so the cached seed-geometry state does not flatten that
branch back into an apparently fully certified seed family. The support side
now also caches shifted
support-cell halfspace/report/seed families by exact `(bounds, halfspaces,
seed)` state, so repeated shifted support-target construction does not rebuild
the same shifted local search stack after earlier support attempts already
explored it.
Support-side witness replay now also caches
`reference_target_from_halfspace_witness(...)` by exact witness point plus
geometric halfspace/active-plane state, so repeated strict and shifted support
target construction does not rebuild the same retained-definition target at one
support witness over and over.
Projected root-family and projected-escape witness replay now do the same
inside one projected reference update, so repeated report/direct/shifted
projected witness families reuse the same retained-definition target
construction before support fallback even starts. The live support query cache
now shares that same witness-target memo across the later support fallback
phase too, so one `compute_new_reference(...)` update does not rebuild the same
retained-definition target separately in projected root search and then again
when support fallback revisits the same witness state.
The same projected/reference cache set now also memoizes the full projected
root family assembly by `(bounds, projected halfspace state)`, so repeated
child reference calls do not rebuild the same projected feasibility report,
strict target family, and projected escape family before trace-time search
starts.
The same per-update projected/support query cache now also memoizes exact
strict point-vs-reference-halfspace-cell containment checks by
`(bounds, point, halfspaces)`, so repeated projected/support target families do
not repay the same boundary-aware strict-cell membership predicate each time
they revisit the same local witness or seed state.
Those geometry-only support/reference caches now also stay alive across
recursive child `compute_new_reference(...)` calls in subdivision, while the
old-ref/polygon-specific trace, validity, accept, and recursive support-search
layers are reset per call, so sibling child-reference propagation reuses the
same exact report/seed/witness/containment work instead of rebuilding it from
scratch on each new child bound.
Projected direct target tracing now
shares those same bounds-aware validity and trace caches too, so one reference
update does not repay the same exact validity or retained-definition check when
a projected direct target later reappears in support-side search. The
support/projected reference side now also treats retained definition triples as
set-equal up to plane permutation when it merges target families and when it
hits the retained-definition trace cache, so equivalent target definitions do
not repay the same trace just because the local plane order differs. The same
trace cache now also ignores redundant fallback/certified duplication on an
otherwise identical target, because the retained-definition trace itself does
not depend on that bookkeeping bit. Identical support states also
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
reject that target afterward. Uncertified support-surface rejection or
reference-validity checks now also invalidate only the current target and let
later exact targets run, instead of aborting the whole target search before
those siblings get traced. Exact boundary contact on a local polygon now also
counts as an uncertified reference-validity check in that layer instead of
ordinary invalid target rejection. Support-surface rejection now also classifies
against the actual support polygon instead of only its support plane, so exact
boundary contact there counts as `UnknownClassification` and coplanar points
outside the polygon are no longer rejected as ordinary surface hits. The same
certified local-polygon validity rule now also applies to the inherited
reference fast path in `compute_new_reference(...)`, so a boundary old
reference no longer degrades into ordinary “try another sampled reference”
failure if every later projected/support path also exhausts. The
cycle-guarded detour layers now treat uncertified traced-surface checks the
same way: one uncertified detour-point surface query no longer aborts the
whole detour family before later certified detours run. The
exact support/projected vertex family now also skips candidate-local
`UnknownClassification` membership checks instead of aborting the whole
vertex family on the first uncertified candidate, and the leaf/probe-side
exact halfspace-cell vertex family now does the same. The
projected-escape direct-seed family now also backtracks past uncertified
membership checks instead of letting one uncertified deferred direct seed cut
off later certified direct escape targets. The
projected-escape report-witness and shifted-family include checks now apply
the same strict halfspace rule, so a witness on a non-equality halfspace
boundary is treated as `UnknownClassification` locally instead of ordinary
containment. That same projected-escape layer now also memoizes exact pure
halfspace containment checks by `(point, halfspaces)` across its report,
direct-seed, and shifted-family builders, so one projected reference update
does not repay the same strict escape-membership predicate on each sibling
revisit. The
focused reference tests now also cover this support-cell fallback on prepared
closed-mesh polygons, not only on synthetic support-plane fixtures. An
inherited reference on a source surface can now be normalized before
projected/support search when the available source polygons certify exact
boundary-free closure for every winding component inside the task bounds.
Strict coplanar face interiors keep the direct two-normal-side path. Edge,
vertex, and non-coplanar multi-surface contacts instead enumerate the finite
arrangement of incident support planes in bounded direction space, construct a
strict witness for each feasible direction cell, and advance it only to the
first exact polygon or bounds barrier. Hypermesh traces each resulting adjacent
point from an exact exterior point with zero winding and accepts only the open
cell whose independently certified winding matches the inherited `ref_wnv`; it
does not infer a side from face orientation. The exterior-zero proof now also
requires every winding-vector mesh to be represented, so an omitted enclosing
mesh cannot be silently treated as zero. Clipped-open and missing-mesh surface
families remain explicit certification failures. Full EMBER plane-replacement
coverage for every reference construction therefore remains unfinished beyond
this closed-family surface-departure case.
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
Property coverage also generates bounded integer-coordinate closed-box pairs,
runs every Boolean operator through the public general path, certifies polygon
closure before triangulation cleanup, and compares the certified triangle
soup against the exact analytic Boolean volume.
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
coordinate-scale cutoff. Raw AABB-midpoint splitting is no longer a subdivision
fallback. A top-level subdivision call constructs one finite split basis from
exact interior gaps between root polygon coordinates and exact root
pairwise-intersection segment endpoints. Every descendant task can use only
basis planes strictly inside its current bounds; recursively clipped geometry
cannot introduce a new split plane. After a plane is selected, it is a boundary
of, or outside, each child and therefore cannot be selected again on either
branch. The number of root-basis planes strictly inside the bounds decreases on
every recursive split, which bounds branch depth by the finite root-basis size.
This supplies the global subdivision-finiteness proof independently of
`max_depth`; finite values remain a user-selected certification budget, while
the default `usize::MAX` leaves the finite split basis as the termination bound.
Every task now attempts its exact BSP leaf-completeness proof before constructing
or traversing ordered split attempts. A certified leaf therefore terminates at
its current arrangement cell without unnecessary split-child reference
propagation; subdivision runs only after that proof remains uncertified. Public
coverage includes a union whose first input contains overlapping closed
tetrahedron components and whose root leaf certifies a closed exact-volume
result directly.
If no root-basis plane remains, hypermesh attempts the certified leaf once and
returns `UnknownClassification` if it cannot prove BSP completeness; it does not
repeatedly bisect event-free bounds toward `max_depth`. Split ranking penalizes
empty-child cuts explicitly, so a cut
that leaves all polygons on one side is no longer preferred over a non-empty
branching cut with the same maximum child load just because it duplicates fewer
polygons. When those child-count metrics tie, split ranking now prefers the
candidate with the lower exact post-split child intersection load before it
falls back to source kind. The recursion now backtracks across the ordered
currently available root-basis family instead of committing to one chosen split
candidate: if a higher-ranked
split hits `UnknownClassification`, `ReferencePropagationFailed`, or
`SubdivisionDepthLimit`, later root-basis split candidates are still tried
before the task gives up. Exact intersection candidates win arrangement-gap
ties, and duplicate arrangement-gap candidates are promoted when an exact
intersection endpoint reaches the same split plane. If a task reaches
`max_depth` while the bounds remain
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
submitting them to the BSP at all. When the highest-ranked split strictly
reduces the polygon family in every non-empty child, a splittable task tries
that one family-reducing branch before materializing the dense leaf BSP.
If the preferred branch fails certification, the task tries the certified leaf
path exactly once and then backtracks through every remaining split; tasks with
no strictly reducing split still try the leaf first. This changes only work
ordering: no leaf or split certification path is dropped. When a split would
otherwise recurse on the exact same polygon family
with an empty opposite child, the surviving child now contracts to the exact
local polygon-family bounds instead of repeatedly bisecting empty space around
that unchanged arrangement. Different split planes that contract to the same
child polygon partition are now also skipped before reference propagation and
recursion rerun that identical branch state, and identical child states now
also reuse cached reference propagation instead of recomputing the same child
reference witness each time that state reappears. That child reference cache
and the matching child subdivision cache are now shared across the whole
top-level subdivision call, not only one parent split search, but the child
reference memo is keyed by the full parent reference state, the parent/source
polygon family that actually drives `compute_new_reference(...)`, and the
child bounds so recursive reuse stays exact. Equivalent parent retained
definition families now also hit that child-reference memo even when the same
three planes arrive in a different local order, and the matching child
subdivision memo now treats equivalent retained parent definition families the
same way instead of missing recursive branch reuse on plane-order-only
differences. The exact ordered split-candidate family is now also cached by
child bounds plus polygon family, so recursive tasks that differ only in
reference state no longer repay the same arrangement/intersection split search
before reference propagation starts. Those same cached split candidates now
also share their exact clipped child polygon partitions with the later split
attempt loop, so one recursive task no longer clips the same candidate family
twice just to rank and then execute the same split. The live split-ranking path
now also collects pairwise segment intersections only once per polygon family
and reuses that exact segment set across all three axis-specific intersection
candidate scans, instead of rebuilding the same BVH and polygon-pair segment
intersections once per axis. The same top-level subdivision runtime now also
keeps one exact pairwise polygon-intersection family per repeated local polygon
family up to permutation and reuses it across split child-intersection load
and later direct leaf/BSP processing, instead of rebuilding the same pairwise
polygon relation again when recursive branches revisit that exact local family
in a different polygon order. Split ranking now also derives its intersection
segment endpoints from that cached pairwise polygon relation instead of paying
another full BVH/polygon-pair intersection pass just to recover the same split
candidates. It also caches the exact sorted per-axis vertex coordinates for
each repeated polygon family, so arrangement split search under different child
bounds no longer recomputes the same affine polygon vertices and axis ordering
before filtering those values back down to the active bounds. Cached split
child partitions now also reuse the
same clipped child polygon families across those permuted parent orders instead
of fragmenting on order-only differences before later recursion and child-state
reuse. Recursive branches that
converge back to the same exact child task can therefore reuse the
already-certified child result instead of replaying that whole branch again.
Repeated BSP leaf certification on the same host polygon, rotated-equivalent
leaf edge cycle, and repeated local polygon family up to permutation now also
reuses one exact certified leaf-analysis result across recursive branches
instead of rebuilding the same BSP leaf witness family and effective-`delta_w`
state again. The same recursive leaf path now also reuses the exact enabled
face-local BSP leaf family for the same host polygon and repeated local polygon
family up to permutation, instead of rebuilding the same local BSP split tree
from cached pairwise intersections every time those branches reappear.
Unsplittable tasks now also run the exact leaf
processor directly once instead of first retrying the same uncertified path
through the certified leaf-output helper, but they only succeed if that leaf
result is explicitly marked `certified_complete`; an unsplittable task whose
leaf processor returns a non-certified `Ok(...)` now surfaces
`UnknownClassification` instead of leaking partial output. That lets exact local arrangement
isolation continue until the exact split family, depth budget, or a certified
leaf result stops the branch. Hypermesh reports `SubdivisionDepthLimit` only if
the configured depth budget is reached while an exact arrangement split remains
available. If the split family is exhausted and leaf classification or its
isolation check still fails, it reports `UnknownClassification` without
appending partial output. Certified BSP leaf
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

`triangulate_and_resolve_certified` resolves exact duplicate vertices,
duplicate faces, and T-junctions, but refuses non-empty outputs with singleton
edges, directed edge imbalances, or zero signed volume instead of capping or
peeling them. Balanced non-manifold edge valence is allowed for closed PWN
output. If exact T-junction/crossing
resolution does not converge within its certification budget, it reports
`OutputResolutionLimit`. Hypermesh does not expose a repairing triangulation
path; if the classified arrangement is not emitted closed by construction, the
operation fails certification. The emitted polygon arrangement is now checked
for exact signed boundary closure before any triangulation cleanup runs: after
exact T-junction subdivision, every geometric subedge must have equal forward
and reverse polygon uses. A reversed face or duplicate same-oriented open face
therefore fails with an `OpenOutput` report containing `unbalanced_edges` even
when every undirected edge has valence two. Thus,
`resolve_tjunctions` only cleans triangle-soup representation artifacts and is
not allowed to turn an open polygon arrangement into a certified result.
Exact duplicate oriented output polygons are now also suppressed when the
classified arrangement is materialized into `BooleanResult`, so shared-face and
coplanar duplicate surfaces are reduced before closure checking and
triangulation cleanup run. The same exact-geometry duplicate suppression now
also runs earlier at subdivision emission time, so duplicate classified
polygons are merged before they ever reach the final classified arrangement.
That final `BooleanResult` materialization now also buckets classified polygons
by `(classification, support plane, edge count)` before exact edge-cycle
comparison, so large classified outputs do not rescan the entire emitted
polygon list for every later duplicate candidate.
The exact closure check now also caches canonically directed split subedges per
undirected polygon edge, so repeated coincident segments do not rescan and
re-sort the same merged vertex chain before counting forward and reverse uses.
Its exact duplicate-vertex merge
step now also groups vertices by exact lexicographic ordering and still assigns
merged vertex ids by first appearance, instead of string-keying every
coordinate triple or linearly rescanning every merged vertex for each new
polygon vertex. The same closure pass now also keeps
one exact per-axis ordering of merged output vertices, so each undirected edge
only checks vertices whose dominant-axis coordinate lies inside that edge span
instead of rescanning the entire merged vertex set before exact on-segment
certification.
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
