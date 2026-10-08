# Local chord/parallel roots — qualification in progress

Parent: Hypercurve `9945504d571c15316b9fbca22aa689a63b6dddb2`.

The all-roots chord/parallel API required a dense global norm even when
Hypersolve could isolate every finite root in the selected coefficient field.
Its contact payload then narrowed every parameter to BezierParameter2. The
candidate changes retain CurveParameter2, remove the eager-projection constructor
flag and migrate its nine callers, and share exact contact replay between local
and global enumeration. The global norm is cached only when successfully needed.

The finite route accepts only a complete ordered-field isolation report over an
outward envelope, then tests each root against the original retained boundaries.
Strict source-frame admission remains before enumeration. Every retained contact
replays the positive-speed sheet, nonzero source weight, finite chord membership,
tangent cross and dot, and nonzero parallel derivative scale. Local uncertainty
falls through to the complete existing global route. Repeated roots, whole
components, and unbounded rays retain that route. No approximation selects a new
persistent root representation.

The independent parabola oracle has four simple norm roots, exactly two authored
positive-speed contacts, and finite chord counts 2/1/0. Both policies and both
traversal directions must preserve root representation, containment, and tangent
orientation with the global cache empty. The independent rational root-count and
clipping certificate is local-chord-parallel-roots-20260924-independent-algebra.json.

V1 is an immutable test-only snapshot. All production bytes match the parent.
Its new regression fails with Unsupported after certifying all four local norm
roots. The driver, source hashes, executable, logs, and terminal receipt bind that
failure. Its exact process handle was reaped before production edits.

V2 adds the implementation and caller migrations. All-target Clippy identified
one remaining identity conversion in curve_support_intersection.rs. Both Clippy
jobs and the outer driver were reaped before fixing that direct caller. V3 differs
from V2 only by this migration. No geometric assertion in the new oracle changed;
V2's oracle-binding receipt records its mechanical API changes from V1. The old
selected-conjugate-fiber regression continues to assert its dense certificate and
also exercises the public local dispatcher on the same fixtures.

V3 qualification is running. No candidate commit or full-goal completion is
claimed. The four earlier bounded-workload failures remain obligations until
measured against the new frozen snapshot.

V3 focused qualification is complete: all 23 focused cases, rustfmt, and both
all-target Clippy configurations pass. The new independent contact regression
finishes in 0.02 seconds of libtest time. Full 1471-case qualification is running.
The source freeze remains in effect until its exact outer handle is reaped.

V3 full qualification is terminal and all 1471 cases were attempted: 1460 pass,
six remain ignored, and five are nonpassing. All 225 public integration cases
pass. A previously passing nonrepresented chord/parallel fillet now times out
at 75 seconds; the old nonlinear endpoint fillet timeout becomes an explicit
Unsupported result. These block qualification and commit. The other three old
timeouts remain at 75 seconds. All source/executable bindings match, and the
full outer process was reaped before diagnostics. The first bounded debugger
launch was denied by sandbox ptrace restrictions and obtained no sample.

V4 adds an independent zero-norm circle oracle without changing V3 production.
The left unit offset of the clockwise rational unit-circle arc is Q=2P; its
opposite sheet collapses to the origin. A vertical chord has exactly one authored
contact at the midpoint, although the squared norm is identically zero. V4 fails
when that midpoint is used as the regular sample, incorrectly declaring a whole
component. The rational polynomial certificate is recorded separately in
zero-norm-midpoint-contact-20260924-algebra.json.

V5 proves the unsquared norm identity in the authored coefficient field and
compares the first nonzero common jet when the sample is a common zero. This
separates an isolated authored contact from the opposite-sheet component.
Three duplicate coefficient-identity methods are replaced by one strict helper.
Both independent regressions pass. Native promotion now clips projected
candidates to the retained isolator before expensive coefficient replay, and
uses uniqueness only when the complete projection leaves one candidate.
Formatting and both all-target Clippy configurations pass. The previously
passing fillet still times out at 75 seconds, and the nonlinear endpoint fillet
still returns Unsupported. V5 is not qualified for commit.

All V3/V5 bounded debugger probes are terminal and their exact outer handles
were reaped. The V5 fillet stack moved from root promotion to independent
Cartesian direction projection in represented circle/rational reconstruction.
The endpoint-fillet failure trace confirms local center isolation and chord-normal
frame selection, followed by the narrower companion-parameter requirement.
The existing recursive circle frame already admits this geometry.

V6 migrates retained selected parallel contacts, angular and tangent queries,
point publication, and fillet companion reconstruction to CurveParameter2.
Chord tangent predicates replay a local root in its joined coefficient field
before the existing native trivariate specialization. Three duplicated range
membership functions become one common-parameter predicate. Chord-normal
circle/rational dispatch now tries the general recursive frame before Cartesian
projection. Compiler qualification identified four redundant conversions/borrows;
its exact process was reaped before their V7 correction. V7 qualification is in
progress. Neither snapshot is claimed qualified or committed.

Read-only V7 review identified two additional boundaries to qualify before the
migration is considered complete. A local parallel-normal tangent represented
as a chord should retain center_parallel evidence on that frame; otherwise the
frame chooser treats it differently from its native ParallelNormal equivalent
and selects an unrelated chord coefficient field. The recursive circle kernel
can then needlessly join center and chord fields. Separately, a structurally
zero radical in an identically zero norm proves the rational incidence is zero
without any positive-speed premise. This matters for a zero-offset source with
a stationary component sample, whose domain correctly requires only finite
weight. These are review findings, not yet changes or passing claims.

V7 formatting and both all-target Clippy configurations pass, as do the two
independent root regressions. The same fillet times out at 75 seconds. Its
bounded stack now reaches local circle/rational isolation but repeatedly
refines the unnecessarily combined frame field. All driver and debugger handles
were reaped before V8. V8 retains center_parallel evidence when a parallel's
normal frame is represented as a chord, preserving native frame-selection
semantics for local roots. A new exact contact test uses P(t)=(0,t^2-1/2) at
sqrt(1/2), retaining root allocation identity through both circle frame kinds,
angular ordering, point publication and rejection of an unequal source parameter.
The stationary-sample review concern is already covered by construction: a
zero-distance chord system fixes target speed to one. No extra fix is needed.

V8 formatting, both all-target Clippy configurations, and all three independent
root/contact regressions pass. The nonrepresented fillet remains a 75-second
timeout, with repeated Sturm preparation in recursive coefficient intervals.
Its bounded debugger was reaped. The old nonlinear endpoint fillet gets beyond
Unsupported but remains a 75-second diagnostic timeout; that diagnostic
partly overlapped the debugger and is not a performance qualification.
Every outer handle was reaped before edits.

V9–V11 migrate the parallel-normal circle frame itself to CurveParameter2.
This removes the native-vs-local frame split and fallback chord construction
from the parallel fillet carrier. Bounds, analytic point factories, source
identity and affine boundary maps retain the common parameter. Only actual
univariate/bivariate projection kernels request ordinary-root evidence. Selected
chord/parallel-normal contact maps also retain the common parameter. Endpoint
analytic-source accessors now admit selected fragments directly. All constructor
callers, including trimming and subdivision, are migrated without shims.
The degree-135 selected-frame test now asserts ParallelNormal and its original
retained parameter, keeping its guarded-promotion assertions. The independent
contact regression adds a local normal-frame center and exact diameter endpoints;
none of its previous assertions are removed. V9/V10 compiler failures were
reaped before correction. Initial V11 setup exhausted disk quota before a build
started. Only rebuildable owned debug/release caches and the incomplete V11
copy were removed. Completed snapshots and archived executables are unchanged.
V11 is now frozen for 31 focused cases, including four existing normal/affine
frame cases. No candidate commit or full-goal completion is claimed.

V11 passes formatting, both Clippy configurations and its first seven focused
cases, but the prior passing fillet still times out at 75 seconds. Its bounded
debugger reaches mixed selected-fiber/local parameter ordering during Boolean
splitting. Both parameters were projected globally even though a selected
fiber already supports native-parameter comparison. Every handle was reaped.
V12 tries that direct comparison after projecting only the recursive scalar,
retaining both-projection fallback on uncertainty. The common parameter caller
is updated directly. The contact oracle adds equal and nearby shifted selected
fibers with a foreign repeated root, in both orders and policies. The existing
degree-135 mixed-gap test joins the focused set (32 cases). V12 is frozen.

V12 passes its first nine focused cases, including the prior passing fillet
in 66.46 library seconds (parent 4.32 process seconds). The nonlinear endpoint
fillet remains a 75-second timeout. Separate bounded debugger samples reach
mixed comparison: a large common-root proof in the first case, repeated fiber
refinement in the second. Both handles were reaped before edits. V13 removes
another native-only storage boundary: canonical parallel tangent authority now
retains CurveParameter2, so affine replacement can preserve the original
center alias for local cuts. Its native ray/contact consumers explicitly request
projection. The existing contact oracle adds exact positive-affine alias
identity coverage. No new cache is added. V13 is frozen for qualification.

V13 passes its first nine focused cases; the fillet remains 66.59 library
seconds and the nonlinear endpoint fillet still times out at 75 seconds. The
outer driver is reaped. V14 adds a native root identity proof: the same defining
polynomial and strict containment of one certified singleton interval in the
other prove the same selected root, even across independent allocations. No
cache, polynomial reconstruction, or equality inference from mere overlap is
introduced. The new regression covers identity both ways, affine chart
composition and overlapping isolators selecting opposite conjugates. All V13
contact/root regressions remain byte-identical. V14 is frozen for 33 focused
cases; complete qualification now contains 1249 library and 225 public cases.

V14 passes the new native root identity case and the previous focused cases
through the fillet (67.32 library seconds); nonlinear endpoint remains a
75-second timeout. The outer driver was reaped. Review corrected the new
conjugate test bracket: [-1,1/4] did not overlap the positive [1/2,1] bracket;
V15 uses [-1,2/3], which does overlap while still selecting only the negative
root. V15 also retains successful strict native projection with each recursive
selected root and shares it through refinement/identity-preserving embeddings.
Different roots and changed charts get distinct cells; no uncertainty is cached.
Mixed selected/local comparison moves to that root authority and reuses an
already-projected coefficient-field alias before interval refinement. Stored
separation remains first, unrelated roots retain local refinement and complete
projection fallback. The new two-conjugate regression checks projected order,
refinement, and complemented charts in both policies. The three earlier
contact/root regressions are byte-identical. V15 is frozen for 34 focused cases
and 1250 library plus 225 public cases; no qualification or commit is claimed.

V15 passes all ten focused cases preceding the nonlinear endpoint fillet,
including projection/refinement/chart values. The prior passing fillet remains
66.33 library seconds and the nonlinear endpoint still times out. Its exact
outer handle was reaped. V16 retains a single composed projective chart for
local polynomial roots, directly referencing their original unmapped selection.
Native demand projects that source once and replays the certified map; changed
polynomial coefficients are regenerated from the original relation and composed
map, preventing common-scale/history inflation. Inverse maps recover the
original selection and preserve any intervening refinement. The rational-scale
normalizer is shared with ordinary native charts. Mixed comparison may replay
a chart whose source already has a native certificate before refining bounds.
The projection regression now checks retained source maps and refinement progress;
all three earlier contact/root regressions remain byte-identical. V16 is frozen
for the same 34 focused and 1475 total cases. No candidate is qualified yet.

V16 passes formatting, both Clippy configurations and eleven focused cases.
The previously passing fillet falls to 10.25 process seconds. The nonlinear
endpoint fillet now reaches its old carrier-only assertion in 13.25 process
seconds instead of timing out. The exact outer handle 86685 is reaped. V17
changes only that existing test: local selected cuts are admitted through a
strict polynomial proof that their nonconstant quadratic source satisfies
X*W=Y^2, with zero offset, while also requiring normalized filled-left topology.
The fillet-presence and extension candidate assertions remain unchanged. All
production code and all five new regressions remain byte-identical to V16.
V17 is frozen for the same focused and full qualification.

V17 stops in Clippy: the new assertion treated Option<bool> as bool.
Outer handle 87710 is reaped; V18 requires Some(true) explicitly. Review also
preserves the mapped-polynomial projection fallback when original-source chart
replay is uncertain: the cheaper identity route cannot remove a prior complete
route. Formatter handle 83565 is reaped. All five new regressions are unchanged.
V18 is frozen for the same focused and full qualification.

V18 passes both Clippy configurations and eleven focused cases. The fillet
is 9.50 library seconds; nonlinear endpoint completes geometry in 12.93
seconds and passes the new exact parabola assertion, then reaches the second
old assertion requiring a circle specifically in loop zero. Outer handle
54420 is reaped. V19 changes only this test to inspect all normalized boundary
components, preserving the circle requirement and every geometric assertion.
Production and all five new regressions remain byte-identical. The required
new singleton-chart selector is corrected from tests to its actual module
conversion_tests; no required case or limit is removed. V19 is frozen.

V19 passes formatting, both Clippy configurations and all 34 focused cases.
The nonlinear endpoint fillet now passes with circle presence checked over
all boundary components; the prior assertion was restricted to loop zero.
Outer handle 15523 is reaped. V20 strengthens only that test with five exact
point witnesses, independently derived in endpoint-fillet-normalization-20260924.md.
They verify both positive and negative winding lobes survive nonzero
regularization, their boundary ownership, interiors and exterior. All previous
assertions, all production and all five new tests remain unchanged. V20 is frozen.

V20 passes Clippy and its preceding focused cases, then rejects the new
negative-contact oracle: no returned candidate contains that lower boundary.
Outer handle 29805 is reaped. Code and independent geometry show why: its
chord contact lies before the incoming segment start, while extension is
admissible only past the edited end. No production fix is indicated. V21
replaces that incorrect oracle with the left/left offset branch independently
isolated at 69/100<t<7/10. Exact Fraction bounds certify the contact, an
interior disk witness, and admissible line extension beyond alpha. Four further
witnesses check old/new boundaries, original interior and the exterior gap.
The derivation and rational bounds are in endpoint-fillet-normalization-20260924-v2.md
and its algebra JSON. Production instructions are unchanged; the intersection
method documentation is updated to describe local-first enumeration. All five
new regressions and all previous endpoint assertions remain. Formatter 60009
is reaped. V21 is frozen for qualification.

V21 passes formatting, both all-target Clippy configurations, and all 34
focused cases, including all five independent point witnesses on the admissible
exterior fillet branch under both policies and both traversal directions.
The exact outer handle 53025 is reaped. Fillet process timings: [('nonrepresented_chord_parallel_corner_fillets_without_reintersection', 9.84579551499337), ('nonlinear_algebraic_endpoint_fillet_uses_complete_incident_domain', 40.488849337096326)].
Full qualification has started with the same frozen 2,044 inputs; no commit
or overall completion is claimed yet.


V21 full qualification has exposed previously passing chord-normal rational
contact/overlap regressions. The source remains frozen while that driver runs.
The failures share the V6 recursive-before-represented chord-normal dispatch,
which is no longer required by the V9 common parallel-normal frame migration.
The recursive component publisher passes tangent_dot_source, which includes
turn, to a callback requiring the absolute angular sign cross(Q-C,Q'). The
clockwise case therefore applies traversal orientation twice. Point replay also
requires as_bezier_parameter even though the recursive contact retains a valid
CurveParameter2 accepted by rational_point_evidence_at_region_parameter.
The planned correction restores the previous demand-driven chord-normal
dispatch, fixes these two shared replay contracts, and exercises the recursive
route directly using the existing independent-anchor geometric fixtures.
No gate, timeout, existing oracle, or source is changed before driver reaping.

V21 full qualification is terminal and its exact outer handle 81542 was reaped
before edits. All 1,475 cases ran: 1,455 pass, six are ignored, three previous
timeouts remain and eleven previously passing chord-normal cases regress. All
225 public integration cases pass. V22 removes the obsolete recursive-first
chord-normal detour, corrects the recursive angular/traversal sign contract and
replays rational contact points through the common parameter authority. The
existing independent-anchor tests now also invoke recursive contact replay
directly; overlap covers both circle directions and both weight signs. All
previous mathematical assertions remain. The eleven V21 regressions are added
to the focused gate (45 cases); no case or bound is removed or relaxed. V22
is a separate physical copy of all 2,044 inputs, with no shared source inodes.

V22 focused qualification passes all 45 cases, rustfmt and both all-target
Clippy configurations. Its exact outer handle 80274 is reaped. All eleven V21
regressions pass, including the added direct recursive replay assertions and
both circle traversals. The nonlinear endpoint fillet remains passing. Full
1,475-case qualification now begins on the unchanged V22 source snapshot.

V22 full qualification and its exact outer handle 55496 are terminal and
reaped. All 1,475 cases were attempted: 1,466 pass, six remain ignored and the
three previous 75-second timeouts remain. All 225 public integration cases
pass, with no regression from the parent. The nonlinear endpoint fillet
finishes in 40.535 process seconds and the nonrepresented fillet in 9.788
seconds (still slower than the parent's 4.321 seconds). The commit gate passes.

Hypercurve commit bfa31c3370dee1cb9d57c246964cbb48d80d0424 contains the exact qualified bytes.
Staged and post-commit receipts bind all 2,044 source inputs, archived
executables, the index and HEAD. All thirty repositories are clean and every
owned process is reaped. Nothing was pushed. The goal remains active.
