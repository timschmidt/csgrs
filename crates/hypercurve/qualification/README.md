# Executable capability witnesses

This is the initial bounded matrix for M0 of the
[exact closure plan](../../../docs/research/hypercurve_exact_curve_closure_plan.txt).
It records what named public tests assert on the source revision actually built.
The full algebraic domain in [AGENTS.md](../../../AGENTS.md) remains the target;
this catalog does not establish universal closure or complete M0 coverage.

From the repository root, with Python 3.11 or later and the repository's Rust
toolchain installed:

```sh
python3 crates/hypercurve/scripts/test_capability_matrix.py
python3 crates/hypercurve/scripts/capability_matrix.py --output /tmp/hypercurve-capabilities
```

The output directory must be new and outside the source tree. `--fixture ID`
selects one row and can be repeated. Omitting it runs all catalog rows.
`--build-timeout SECONDS` controls build containment separately from each row's
published timeout. Cargo uses `--locked --release --all-features`; normal Cargo
build-cache reuse is allowed. Each fixture runs alone in a fresh test process,
with one test thread. A fresh-process archive replay claim additionally requires
the witness itself to save and reload geometry in separate processes.

The [catalog](capabilities.json) separates authored, generated and selected
implicit inputs. Each row names its owner, properties, scalar/input domain,
policies, operation conventions, asserted outcomes and oracle. The source test
remains the authority for construction and assertions. Changing a fixture's
meaning requires reviewing its catalog row as well. These are reviewed witness
claims, not an automatic inference of geometry semantics from a zero exit code.

The report keeps separate stages for representation, evaluation, differentials,
incidence/order, classification, region admission, Booleans, offsets, repeated
offsets, corner edits, restriction/reconstruction, transforms, exact replay and
resource recovery. An empty stage list means **not qualified by this witness**.
It does not mean unsupported or successful. A report for selected IDs says
nothing about rows that were not run.

| Recorded outcome | Meaning when the witness passes |
| --- | --- |
| `exact_success` | The named exact assertions pass within their stated scope. |
| `successful_empty` | The fixture explicitly asserts a successful empty result. |
| `unsupported`, `invalid`, `exhausted` | The fixture asserts that specific public failure category. |
| `blocked_unspecified` | The fixture asserts `Blocked` without checking its reason; no narrower cause is claimed. |
| `qualified_approximation` | The fixture retains the consumed approximation receipt. |

The schema also reserves distinct outcomes for proved no solution,
underconstrained input, undecided predicates and unbounded output. An outcome
not present in the catalog has no new witness here. The polynomial frontier test now preserves both the original disjoint pair
and the unchanged H08 boundary as positive incidence/admission witnesses.
The original authored SVG round trips exercise eight authored families in one
process. Selected source-restriction and procedural-parallel replay have separate
witnesses below; these do not establish all generated definitions or filled-region
replay.
Point-definition selector rejection is distinct from native archive replay.
Membership samples are only the finite assertions listed, not a proof of the
entire represented set.

The selected SVG witnesses retain native sources, original ranges, traversal,
selected polynomial roots and projective charts in version 4 metadata. One
fixture saves a quadratic trim, reloads it in another process, then checks exact
point incidence, further restriction, another archive and reversed overlap.
Separate witnesses cover a shared degree-seven root, algebraic coefficients,
similarities, rational trims of all eight authored families, invalid selectors,
references and chart poles, and reached byte/segment limits followed by retry.
The version 3 fixtures remain unchanged. Finite projection consumes the existing
retained source spans, and its output never replaces authoritative metadata.
Shared scalar expression DAG serialization, cross-path dependencies, other
generated carriers, filled-region replay and total resource bounds remain open.

The parallel SVG witnesses retain quadratic, cubic and rational Bezier sources,
signed distances, selected ranges and traversal in the same version 4 graph.
The named parabola, nonrational ellipse offset, Pythagorean-hodograph cubic and
zero-distance stationary source have independently derived midpoint expectations.
Cusp-split pieces retain one source and one selected boundary; distinct distances
on one source retain that sharing through a second archive. A separate process
reloads a closed path with a parallel boundary, admits it through the existing
region authority, then consumes it in Booleans and a round offset. Its finite
membership assertions are not an independent proof of the entire offset set.
Other witnesses cover an algebraic trim followed by a reflected algebraic
similarity and another archive, exterior source ranges, invalid references and
singular ranges even with an empty final path, and reached byte limits followed
by retry. A rational pole outside the retained range does not prevent replay;
a pole inside it retains the existing typed Boundary blocker.

Finite parallel projection now subdivides inconclusive interval bounds and
accepts a chord only after obtaining a certified enclosure. Power-basis bounds
apply to the retained finite range, including ranges outside the unit chart.
Display points still cannot supply exact incidence or replace the native
definition. The existing subdivision depth and extension-byte limits are not
total-work, coefficient-growth, retained-memory or cancellation bounds. These
witnesses do not qualify selected joint fibers, other generated supports,
filled-region archives, general offset closure or complete resource control.

The selected-chord SVG witnesses preserve supporting lines, directed endpoints,
validated coordinate charts and shared point definitions in version 4 metadata.
They cover exact coordinates, rational source images, rational expressions and
native parallel evaluations at retained Bezier parameters. A degree-seven root
is reloaded in a separate process before incidence, split, reversal, overlap and
another archive. Alternative charts and exterior ranges are checked separately.
Invalid charts, point/support references and original denominator poles fail
even with an empty final path. Reached byte limits permit larger retries; the
named repeated-piece fixture checks shared roots and linear metadata growth,
without claiming general arithmetic or memory bounds.

A mixed parallel/chord fixture checks independent exact endpoint coordinates,
shared source/root definitions and complete SVG determinism after incidence
queries, for both a lazy endpoint and a standalone point definition. The direct
finite-projection unit regression also retains the original cold/warm case and
coarse/fine/coarse requests. Another witness reloads mixed native/generated
boundaries with a circular hole and disconnected material, then checks loop
roles, hole ownership, boundary reconstruction, empty-set Boolean continuation
and a round offset's specified exact hole boundary. Point-location assertions
are finite expectations, not a standalone proof of the entire offset set.
The additional point-graph witnesses below extend that initial point vocabulary.
Auxiliary contact records, other generated circle frames, filled-region archives
and total resource bounds remain unqualified.

The point-graph SVG witnesses encode polynomial fibers, rational images,
similarities and supporting-line intersections as prior validated dependencies.
The original degree-seven fillet yields a trimmed line whose selected contact
replays in another process before a further exact split and another archive.
This initial line-piece witness is extended by the selected-circle witnesses
below; the original complete-circle failure remains in the evidence record.
Other witnesses retain a correlated cubic fiber over a quadratic base, an
original rational denominator, an algebraic reflection and independently
selected degree-seven/degree-five line supports with exterior intersections.
Defining relations and exact point queries are checked separately from complete
SVG determinism. Invalid references, multiple or missing fiber roots, false
exact-root witnesses, nonsimilar matrices, degenerate or parallel line pairs
and original poles are rejected even for an empty final path.

The point-definition depth limit is separate from extension bytes. A raw graph
with 300 nested similarities checks the default limit and a larger retry; an
endpoint/chord indirection cannot reset depth. Rational-map chains exercise
export limits and recovery. The 4/8/16-span prefix fixture checks linear native
record growth and byte exhaustion/retry. These checks do not bound solver work,
existing native prefix-vector allocations, scalar expression trees or total
memory. Chord-displacement witnesses additionally check signed tangent and normal
distances, arbitrary origins, exact translations, reversed sources, paired
endpoint sharing and malformed recipe references. Joint point definitions are
distinct from general joint parameter
charts, whose interchange is still unsupported.

The displacement definition also passes through Hyperbrep's native planar-point
archive. Public model tests retain the original selected fillet cut before
another split and archive, and check a native-coordinate source with and without
an explicit origin. The source and origin consume archive record budgets before
scalar decoding; exhausted attempts allow a larger retry. Degenerate or nonlinear
source substitutions remain invalid. These tests do not turn Hyperbrep's nested
point format into a shared dependency graph or qualify generated circle replay.

The rational-similarity section witnesses retain the original translated
quintic case as a positive fixture. They also check transformed selected-line
evaluation, cusps, tangencies, distinct nearby supports, and a counted-budget
retry. The component witnesses additionally separate a selected common line
from the other branches of its original polynomial. They retain algebraic
trim correspondences, orientation, singleton contacts, transverse and tangent
contacts, and disjoint irrational branches. Their point-definition replay is
in memory; native generated archive replay remains unqualified.

The algebraic-coordinate line witnesses retain the original irrational
vertical and sloped frontier inputs as positive assertions. Native line
coordinates include cubic, seventh-root and mixed quadratic expressions.
Substituted coefficients require certified zero decisions: a quartic's line
component must not become an empty result because its zero expression is not
structurally obvious. These witnesses cover native `LineSeg2` inputs; selected
chord carriers and general branch pairs need their own qualification. Algebraic
similarity coefficients are exercised separately below. They do not establish universal algebraic closure.

The point-order witnesses compare an interior polynomial fiber with an exact
event singleton, retain distinct neighbors separated by `2^-100`, and replay
checked point definitions after source drop. The original equality must finish
within 512 counted refinements and one promotion; a reached zero-budget
attempt must permit a larger retry. These are selected-point and scalar
evidence checks, with in-memory replay only.
Related unit witnesses preserve repeated supports and verify that a
base-independent root relation never discards a query's actual base dependency.

The polynomial-pair witnesses qualify selected graph branches in a common
source chart, including equal rational similarities. Original quintic/parabola
inputs, common components, mixed and residual contacts, algebraic trims,
negative parameter ranges, tangencies and distinct nearby supports remain
exact. Retained splits and overlap restrictions are consumed by another
intersection. A degree-37 graph checks continuation beyond the bounded
resultant's ordinary degree schedule. Singular incidence may complete without
a certified tangent sign; that is not singular differential completion.
Those pair witnesses do not qualify different image transforms, broader
coefficient domains or filled polynomial regions. Point-definition replay
remains in memory, and the 300-second fixture allowance is external containment.

The generated-line witnesses retain polynomial-fiber endpoints and chord-owned
point parameters. They exercise finite sections, constant selected components,
reversal, repeated overlap clipping, exact nearby separation, tangency, rational
similarities and a reached zero-budget retry. Construction-certified axis
supports also qualify public point location independently of pair dispatch.
The original native-line controls and failing generated inputs remain in the
corpus. General oblique lines with unrelated selected endpoints and broader
coefficient/transform domains remain unqualified. These checks do not complete
polynomial filled regions or fresh-process generated archive replay.

The selected-abscissa witnesses require polynomial branches to consume retained
rational coordinates over one selected algebraic parameter. They preserve the
generic quintic failure alongside radical and fifth-root controls. Output X
coordinates are repeatedly consumed, with branch order, both denominator signs,
original pole guards, restrictions, transforms, distinct selections and exact
nearby values checked separately. Zero counted allowance and a larger retry keep
explicit outcomes. General dependent-fiber charts, filled-region operations and
fresh-process generated archive replay remain unqualified.

The rational-section witnesses retain the original quintic/polynomial and
quintic/rational failures as positive incidence and continuation cases. Common
components keep distinct monotone visits, exact overlap clipping, local span
charts, cusp endpoint ownership and other-branch boundary contacts. Native
circular, polynomial B-spline and NURBS charts, rational similarities and all
traversal combinations are exercised. Degree-seven and degree-37 sources retain
their original equations through replay and another intersection. Selected trim
and retry witnesses test certainty and explicit reached exhaustion separately.
General coefficient fields, collapsed/vertical common traces, other image maps,
nonrational parallels and polynomial filled operations remain unqualified.

Polynomial point-location witnesses consume native and generated points in
selected graph charts and invertible rational/algebraic similarities. They
check exact root selection, closed algebraic trims, shared singular endpoints,
nearby distinct points, changed transforms, output-to-input intersections and
recovery after reached exhaustion. General rational image preimages and filled
polynomial arrangements remain outside this increment. Checked point definitions
here are in-memory replay, not fresh-process native archive qualification.

Selected polynomial region witnesses now retain graph supports in the shared
arrangement. They qualify the original H08 cap, native and selected point
classification, all four Booleans, holes, disconnected components, reversal,
similarity transport, exact contact-only empty results, `2^-80` separation,
degree-five trims, open-curve clipping, mixed circular boundaries, cusp and node
incidence, and output reconstruction followed by further operations. Independent
equations and source/root/domain assertions accompany topology and set identities.
Zero offset succeeds; nonzero offsets, corner edits and finite projection retain
explicit implementation gaps. Reached exhaustion, larger retry and certainty
transitions are separate claims. This does not establish the entire algebraic
region domain, total resource bounds or fresh-process native replay.

The algebraic similarity witnesses extend native-line sections to retained
images with quadratic and seventh-root scale/translation coefficients and an
oblique map using both sqrt(2) and sqrt(3). Independently expanded world
equations, original source abscissas, reflection and traversal signs, tangency,
selected trim endpoints, overlap continuation and exact nearby separation are
checked. Reached counted exhaustion remains distinct from a larger successful
retry and Certified policy transitions. These are similarity pullbacks through
the existing section authority; general rational images, different-image
polynomial pairs and fresh-process native archive replay remain unqualified.

The composition witnesses combine the independently qualified polynomial
regions, horizontal domain components and algebraic similarity sections. They
retain the original failing clip/reconstruct/subtract/intersect chain, exact
selected boundary coordinates, a selected quintic ordinate and its quarter-turn
image, plus a guarded rational map followed by an algebraic similarity. Existing
intersection and winding authorities consume exact-coordinate axis probes when
ordinary directions are unavailable. Polynomial/chord sections use the existing
strict-predicate scope with the original replay context. Both fill rules, traversal, source/root/map
identity, overlap continuation and Certified policy receipts are asserted within
the named cases. General algebraic closure, native archive replay and total
resource bounds remain unqualified.

The selected-polynomial policy witnesses retain the same point or transformed
branch produced with a Certified receipt under `APPROXIMATE_512`, then consume
it under `STRICT`. Exact coordinate relations, checked point definitions and
further chord/line incidence remain available. A separate unit witness keeps an
observed approximate dependency qualified through a later locally Certified
similarity; the original explicit similarity-policy barrier regression is
unchanged. This qualifies the named polynomial construction paths, not a new
native archive format or every point-family policy transition.

The driver obtains executables from Cargo's artifact messages, checks exact
registration and ignored status, and requires one successful test summary.
Missing tests, ignored tests, zero-test passes, failing builds or tests, changed
binaries, changed catalogs and source changes cannot publish successful claims.
The JSON report contains separate build and fixture commands/times, compiler
and relevant build settings, binary and log digests, the source revision,
tracked/input hashes, dirty patch and new source inputs. It reuses the
repository's existing source-attribution helpers. This is an attributed local
build, not a hermetic-build claim.

Timeout and interruption are incomplete external outcomes, never inferred
`BudgetExhausted`. On timeout or SIGTERM the runner stops the owned process
group, including descendants. This tests the qualification harness's cleanup;
it does not establish kernel cancellation. The per-fixture 300-second allowance
is elapsed-time containment, not a total-work or memory limit. The catalog
states the remaining resource gaps explicitly. Timing under CI or concurrent
work is qualification data, not a controlled performance comparison.

Hypercurve CI executes the evidence-guard tests and the catalog, then retains
its report and logs as an artifact. For a retained implementation or performance
claim, archive the relevant raw record under the repository's
[admission protocol](../../../docs/admission.md) and record its digest. A green
matrix is evidence for these witnesses only; broader transforms, singularities,
nonrational offset chains, generated replay, general algebraic regions and
shipped consumers still require their own algorithms and acceptance evidence.

The eight `polynomial_algebraic_pairs` witnesses extend the catalog to shared
algebraic similarities, selected overlaps, seventh-root scales, reflected
region intersections and oblique chords with selected endpoints. They retain
source equations, exact trim parameters and checked in-memory point replay.
Ten companion Hypersolve acceptance tests cover projective coefficient
proofs, distinct nearby equations, selected parameter factors, uncached
singleton points and strict field admission of mutable root selectors.
Structural witness negation keeps its existing contract and does not certify
field admission. Full qualification and its source attribution are
recorded in the increment ledger; these named cases do not establish arbitrary
algebraic operation closure or fresh-process native interchange.


The selected-circle graph witnesses retain a chord-normal frame and selected
angular domain without requiring independently materialized center coordinates.
The original degree-seven circular fillet replays in a separate process before
exact point location, splitting, reversal and a second archive. Algebraic
rotation/scaling and reflection preserve exact endpoints and an independently
transformed interior point, separately from full SVG determinism. Replayed
boundaries reenter the shared region arrangement, empty-set Boolean identities,
boundary reconstruction, and a mixed authored/generated fixture with a hole and
disconnected material. Changing a tangent neighbor to a transverse line must
recover both independently derived crossings in both operand orders.

Handwritten records reject wrong, forward and cross-circle references, zero
radii, invalid angular ranges, off-circle points, wrong half selections, incorrect
signed normal displacements and nonunit, nonparallel or reversed chord directions.
These checks include unused definitions. A reached circle dependency-depth or
extension-byte limit prevents publication; a larger retry succeeds. A private
unit test additionally checks consumed approximate-policy barriers.

This remains an incremental support qualification. Rational, parallel-normal
and selected-radial generated circle frames, other mapped angular contacts,
repeated-offset chart transport and filled-region archives remain open. A retained
probe with a correlated degree-seven/five fiber center originally imported and
evaluated its endpoints but failed its midpoint query with `Blocked(Ordering)`.
The following common-field witnesses close that retained case; the original
positive failure remains in the evidence record. The
finite witnesses and depth/byte limits do not prove universal algebraic closure,
total solver work, coefficient growth, heap bounds or cooperative cancellation.


The common-field point witnesses reuse the rational-query source pullback for
oblique linear comparison and selected-circle residuals. Their center retains
`2*beta^7-1=0` with `3/4<beta<1` and the correlated fiber
`gamma^5+beta-2=0` with `1<gamma<2`. The independently defined displacement
`(3/5,4/5)` is exactly on the unit circle. Its midpoint query now continues
through evaluation, subdivision, reversal and a second native archive. Negative
radii, both traversal directions, diameter endpoints, an interior quarter point,
the opposite half and a `2^-80` displacement verify chart selection separately
from support incidence. Negative denominator maps retain their signs, and a
vanishing original denominator remains invalid even with a cancellable quotient.

Algebraic rotation/scaling and reflection retain independent point incidence and
complete SVG determinism. A reflected oblique frame originally lost its compact
rational unit direction during replay; an equal radical expression changed one
display coordinate. The direction record now preserves its checked definition
when the ordinary constructor would rebuild different scalar bytes. Byte
identity controls omission only; exact value predicates still prove validity.
The original comparison, source/cold/warm diagnostic, and older replay corpus
are retained.

A reached zero solver-work budget prevents a completed point query; retries on
the same objects succeed within 512 counted root refinements and zero algebraic
promotions. Strict/approximate/strict requests remain certified when these exact
proofs suffice. These counters do not bound total arithmetic, coefficient growth,
heap use or cancellation. General independent point fields, other circle frames
and the complete operation-closure target remain open. The following extension
covers nonconstant angular selections in a shared field; neither corpus proves
universal completion or a performance advantage.


Selected circle-point parameters retain an independently defined point and its
original selected base/fiber instead of reconstructing a scalar angle. The
qualification center satisfies `2*beta^7=1` and `gamma^5+beta=2`; the independent
point uses the rational half chart at `u=beta`. For radius `r`, unit normal `N`,
and selected turn, let `n=(P-C).N` and `b=turn*cross(N,P-C)`. On the selected half,
comparison with a represented parameter `v` uses the exact sign of
`r*((1-2*v)*b-2*v*(1-v)*n)`. Two points on that same half are ordered by its strictly
monotone normal coordinate, accounting for the sign of `r`. Interior parameters
exclude diameter endpoints, which retain canonical zero/one ownership.

Both radial coordinates must share a valid field before publication. A prototype
that checked only one projection admitted an older fillet contact whose remaining
angular queries were unsupported; its unchanged fresh-process replay test caught
the error. The complete radial view shares the original immutable source.
Similarities transport both coordinates through their certified linear part.
Native replay reconstructs and checks point/circle incidence and the selected
half; the stored query view and live caches are not serialized proof.

Public fixtures cover independent definitions, cuts separated by `2^-80`, exact
restriction, split/reverse/replay, both signed radii and turns, algebraic rotation
and scaling, reflection, and another split after a fresh process imports the
archive. Private owner tests reject invalid incidence and complementary-half
records and preserve inherited frame and point qualification. Requesting an
approximate policy with strict dependencies does not itself consume approximation.
Actual zero work limits prevent completed queries; retries have 4096 counted
refinements available and zero promotions. These counters do not bound all
arithmetic or retained memory, and no new cancellation contract is claimed.

The close-cut prototype completes within its 300-second fixture envelope, but
ordinary point equality and display projection account for most of its runtime;
angular ordering uses the shared field. This is a cost frontier, not a speedup
claim. On that preceding revision, separate positive probes report
`Blocked(Unsupported)` for public derivatives and for filled-region admission of
a closed circle assembled from these pieces, before its round-offset chain.
The following differential extension resolves the first probe within its stated
frame and point-image domain; region admission remains open. These are implementation gaps, not exact completion,
no-solution proofs, or resource exhaustion. Mixed regions, repeated offsets and
corner edits still require their own completion evidence.


Selected circle differentials reuse the existing rational Leibniz recurrence,
with the center omitted from the chart being differentiated. For a represented
unit normal `N`, signed radius `r`, and turn `s`, homogeneous controls are
`(r*N,1)`, `(r*s*perp(N),0)`, and `(-r*N,1)`. Original chart polynomials are
differentiated before any selected-field reduction. A retained incident point
supplies its already aligned radial forms `n=(P-C).N` and
`b=s*cross(N,P-C)`. The exact inverse `u=b/(r+n+b)` has a second chart
`u=(r-n)/(r-n+b)` at its excluded endpoint. Original radial denominators and the
chosen inverse denominator are checked before composing derivative images in
the same joint point field. The guarded angular point `(u,0)` is shared by all
derivative maps, preserving the original map chain instead of permanently
expanding the same inverse denominator into each derivative. Neither angular
roots nor coordinate roots are independently materialized. A frame's inherited qualification survives the
mathematical cancellation of its center.

The original degree-seven/five positive first-derivative assertion now succeeds
with exact magnitude. Independent higher-order expectations use the closed
complex formula `z^(k)=2i*k!*(1+i)^(k-1)/(1-(1+i)*u)^(k+1)`, rather than the
implementation's derivative recurrence. Fixtures exercise three orders at
nonconstant selected points, eight orders at both endpoints and the midpoint,
signed radii and turns, restriction and reversal, algebraic similarities,
point-definition replay, and a derivative image used to construct a line whose
native archive is replayed in another process before another exact split.

The retained derivative and an exact vector comparison complete at zero charged
refinements/promotions. Initial test assumptions requiring their exhaustion were
incorrect; both successful zero-budget assertions and the raw failed attempts
are retained. The complete incidence-to-derivative-to-comparison chain reaches
the actual zero limit and succeeds on repeated larger retries. These counters
are not a total arithmetic, expression-growth, memory or cancellation bound.
An unrepresentable requested output count returns a typed implementation blocker;
a reproduced capacity panic is corrected using checked allocation guards shared
with the rational derivative owner. No such blocker counts as completed geometry.

Retained point bounds preserve the same selected roots and rational-map guards.
An initial isolator can straddle a map pole even when the selected point is
finite; local refinement must resolve that enclosure before native display can
consume it. The minimized `1/(beta-4/5)` fixture checks exact endpoint containment
and native replay independently. Curve and path bounds retain one successful
strict enclosure per immutable owner through the existing thread-safe policy
cache. Errors are not retained, so an actually exhausted ZERO request can retry
with 4,096 refinements and no promotions; a later warm ZERO request reuses the
proved enclosure. The entry lives with its owner, has no global retention or
eviction, and a new transformed owner starts with its own cache.
Cardinal coordinate order also reuses the existing shared-field linear predicate
before materializing scalar images. Independent cancelling translations and
signed `2^-80` differences require zero charged promotions/refinements; the same
proof is consumed by affine-form equality and native chord-range replay.
Field alignment removes retained maps and similarities before comparing their
unmapped sources. Those source points already retain both scalar selections and
use the existing comparison path, ensuring normalization makes progress even
when the two selected fibers differ. A distinct-fiber regression preserves equal
base coordinates, strict positive fifth-root order and translated-point identity. This is an
entry-count contract, not a heap-byte or total-work bound.

This corpus demonstrates represented frames with represented or shared
polynomial-point angular authority. Other point-image fields, nonrepresented
frames, filled-region admission, repeated offsets and corner edits remain open.
The finite tests and local rational identities do not establish universal closure,
a uniform practical resource bound or a comparative speedup.


Rational point images retain the original selected point and one immutable map
per node. Both coordinates share that point; their original denominator is
checked before admission and again before a pulled-back query is consumed.
For a polynomial query of total degree `d`, homogeneous substitution uses the
same `d` in its numerator and denominator. These temporary predicate forms do
not replace the original map graph or authorize cancellation at an original
pole. The existing recursive projective field consumes the same homogeneous
image and proves its denominator orientation. Native rational-point predicates
read the original chart polynomials before optional coordinate materialization.
No additional cache is introduced for the shared map node.

The `rational_point_images` target keeps the original degree-seven positive
witness: a rational parabola point `(a,a²)`, `2*a^7=1`, maps to
`(2*a/(1+a²),(1-a²)/(1+a²))`. An independently authored rational Bernstein circle
supplies its expected point. Checked replay and the inverse map recover the
parabola. Additional cases retain negative denominators, nearby distinct
points, original and downstream removable poles, exact bounds, similarities,
and a generated line whose fresh-process native replay precedes another exact
midpoint split and rational map. Geometric identities and deterministic native
serialization are checked separately.

A source-attributed positive witness requires map construction and independent
point comparison with 4,096 refinements and zero coordinate promotions. Actual
ZERO exhaustion is separately checked both inside the constructor and at the
outer shared-budget boundary; larger retries on the same source must succeed.
Private owner cases cover qualified identity and constant queries, as well as
strict native replay inside a provisional scope. The original unsupported
construction, unnecessary promotion request, incomplete exhaustion propagation,
policy failures and harness compiler failures remain in the evidence record.
These counters do not bound total polynomial arithmetic, coefficient growth,
expression size, retained memory, wall time or cancellation. Other selected
point fields, general region admission, offsets and corner chains remain open;
this corpus is not a proof of universal algebraic operation closure.
