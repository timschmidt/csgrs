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
with one test thread. This is not a fresh-process archive replay guarantee.

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
Authored SVG
round trips exercise eight authored families in one process; they do not cover
generated definitions, fresh-process native replay or later modeling operations.
Point-definition selector rejection is distinct from native archive replay.
Membership samples are only the finite assertions listed, not a proof of the
entire represented set.

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
chord carriers, broader transform coefficients and general branch pairs still
need separate qualification. They do not establish universal algebraic closure.

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
