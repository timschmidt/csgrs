# Affine axis projection: isolated owner extraction

The smaller Phase 11 candidate passes local functional qualification and remains
**unaccepted pending cost and distribution admission**. Primary production code
is unchanged. The full path extraction remains required; this does not replace
feed, lookahead or jerk scheduling with a generic ODE or declare Phase 11 complete.
The [previous full extraction failures](path-dynamics-current-admission.md) remain
historical evidence. The bounded memory exception does not waive those runtime
failures or the [cancellation candidate's remaining gates](cancellation-finish-resources.md).

## Contract and source

Owner: Hyperdynamics exact affine dense-axis projection, with a Hyperpath
compatibility adapter. Relevant properties are P1, P6, P8–P10, P19, P31 and P33.
The domain is finite nonempty affine spans with nonnegative exact absolute axis
derivatives, a positive derivative on each span, and dense positive velocity,
acceleration and jerk limits. Existing scalar failures, undecided signs, replay
reports, bottleneck ties and policy semantics remain unchanged. Curved spans and
nonlinear machine maps require their own higher-derivative proof.

The external direct-import witness fails on current baseline
`52b6d1bab627ab212eda80fbc0d4c08a4cd23176` because Hyperdynamics has no projection
API. Candidate `8f351042e1880a8d77b50606df8b387066522c45` moves the existing
algorithm and reports behind `axis-projection`, with thirteen narrow error
variants. A source comparison preserves all algorithm/report tokens except the
error type. Deprecated Hyperpath wrappers map errors and return the same reports
without deep cloning them. The later stack measurement below identifies a return-slot copy. Both root and routing-module legacy paths remain tested.
No arithmetic, sign-call order, dispatch choice or independent replay changed.

Interface candidate `e5af3b4383d33c440d087e0d2b933fccde766b9c`, based on
`5b532c803af86dd9e3d7a66f3651163fd2f7c68b`, imports the new owner directly and
maps failures back to its unchanged route-error boundary. Both repositories keep
`refs/interval-unaccepted/axis-projection-owner-20261117`; these commits are
isolated from their primary branches. Firmware remains at
`7e0d5a43d01706fe619234d8b174beb1ebe45dc6` without edits.

## Executed qualification

Rust 1.95.0, offline locked dependencies, native x86_64, twelve build jobs:

| Check | Result |
| --- | --- |
| Hyperpath and Hyperdynamics all-feature tests/doctests after final edit | 602 passed, 27 suite results |
| Minimal `axis-projection` feature test | passed |
| Interface core tests/doctests | 201 passed |
| External legacy/direct consumers, tracing off/on, both policies | four complete output comparisons match |
| Baseline-derived full dispatch snapshots | sixteen unchanged count assertions |
| Legacy typed failure conversion | all thirteen variants preserved |
| Strict owner/consumer Clippy, strict docs, formatting and typo checks | passed in recorded configurations |
| Repository budget and staged diff checks | passed |

The direct consumer dependency graph contains no geometry crate. Independent
expectations check selected exact limits, first-minimizer ties, unsafe proposals,
invalid inputs and subsequent replay. Complete output comparisons preserve
176,550 bytes without tracing and 215,437 bytes with tracing; they supplement
rather than replace those expectations. The existing Interface 41,795-byte
canonical planner transcript assertion and all its semantic assertions remain
unchanged and pass. Original Hyperpath test assertions are also unchanged.

The initial probe lock was regenerated during candidate feature resolution.
A reconstructed baseline lock on unchanged source retains the same package
versions, sources and checksums, and fresh locked builds reproduce both original
complete outputs. A preliminary movement-script assertion failed before writes;
its original source and correction are retained. The expected missing-API compile
failure is retained. Neither is hidden in a passing-only command inventory.

## Admission still open

This stage supplies no runtime nonregression, complete-lifetime memory/stack,
final native/WASM artifact, packaging, browser or firmware-fit qualification.
The new rational fixture set does not independently cover every algebraic or
undecided-input branch. Broader supported paths and the unchanged owner and
`successive_operations` benchmarks still require their applicable checks.
No memory or binary exception is proposed, accepted or charged here; the fixed
program baseline and every historical positive charge remain unchanged.

The recorded validation contains 53 child invocations and 720.397 seconds,
within its declared 120-command/3,600-second envelope. This is external campaign
containment, not an internal total-work or memory guarantee. The held release
queue and immutable DRC image remain unchanged. No push or release occurred.

Raw evidence: `docs/evidence/2026-11-17-interval-axis-projection-owner` in archive commit
`93f858d19f60ffcb3384fb262673fc19c1eb9493`; 3 files, 45,856 bytes,
record SHA-256 `1c5e1d7a0df68af347a9e993e101001c8c73d7fc248e8ad6bdc035d602ad1676`. All 82 original payload members,
decoded bytes and committed archive files were verified. The digest is registered
in [the evidence index](../evidence-archive.tsv).

## Matched owner timing and legacy trace repair

On Rust 1.95.0 the six previously selected Hyperpath benchmarks all had lower
candidate mean times in both ABBA and BAAB blocks. Each binary ran from the
same path on CPU 0 with ASLR disabled. Each run used a one-second Criterion
warmup, two-second measurement and 30 samples. The 1,440 samples cover
43,953,195 completed iterations. The entries below are percentage changes from
baseline in each block.

| Workload | ABBA | BAAB |
| --- | ---: | ---: |
| Axis-aligned offset | −1.02% | −0.62% |
| Quadratic hodograph | −5.19% | −4.38% |
| Rectangular-pad clearance | −5.21% | −5.08% |
| Additive bead schedule | −5.11% | −5.07% |
| Exact trace import | −0.87% | −0.83% |
| 256-record parser | −2.34% | −2.78% |

The final benchmark file grew from 13,312,800 to 13,323,776 bytes (+10,976),
exceeding the ordinary 4,096-byte incremental cap. The text section grew from
8,118,103 to 8,119,191 bytes. The union of defined function ranges grew from
8,044,187 to 8,045,207 bytes, with six unsized symbols on each side. These
measurements cover the owner benchmark. Full owner timing, direct workload
cost, complete heap/stack lifetimes, downstream artifacts, and cumulative
admission remain open.

The earlier external probe explicitly enabled Hyperdynamics tracing as well as
Hyperpath tracing. An external caller enabling only Hyperpath tracing exposed
an extra lattice matrix dispatch event after axis planning. Its baseline count
was zero. The extracted manifest enabled Hypersolve tracing through Hyperdynamics,
which in turn enabled lattice tracing for the entire consumer. Removing that
feature edge restored the baseline count under both policies. The complete
215,437-byte legacy probe also matched when built with Hyperpath tracing alone.
All 446 Hyperpath tests passed. The repaired owner bench executable is
byte-identical to the timed candidate. The feature fix is isolated in
`08c23797c51102f23367fc904144d1bbe4bcef93`.

Four standalone lockfiles needed updates for the candidate dependency graph.
Candidate `75dd0273eb7b9deddcee80fd98a7626a993a9070` records them and preserves
all registry package versions and checksums. The Hyperpath fuzz targets compile,
the native measurement tests pass, and the WASM measurement client type-checks.
The comparison harness initially lacked its external oracle. All 85 archived
input hashes matched NYU's exactCorelib revision
`e9b4e4af404e657d74741d0e1e6fb18ab19e0838`. With those sources and MPFR
4.2.2, seven path comparisons pass and one existing contract difference remains
ignored. The trace and size findings keep the original qualification and the
later correction distinguishable.

Two of the standalone locks already missed the clock and FOC firmware crates'
Hyperinterval dependencies on the primary branch. Primary commit `4e19d4d33`
restores those four dependency entries across the native and WASM conformance
clients. Native all-feature tests and strict Clippy passed. The stable release
WASM client built and its Fraction oracle matched all 4,369 calls, including
1,330 negative-radius and 24 invalid-denominator cases. The final module was
960,680 bytes with SHA-256
`923d17c99a7788e2ae817a898c7795551de48f1c6d3039c4ddfd345a90dd97da`.

The benchmark binaries, raw samples, feature witnesses, package checks and
lock comparisons are in evidence commit `803884ca5c88752269928634323026f258c30f94`,
record `docs/evidence/2026-11-17-interval-axis-owner-cost`. The WASM replay
above ran after archival using the existing interface-measurements Fraction
oracle and the release module identified by its hash.

The [native consumer memory and runtime review](axis-projection-memory.md) measures
report movement through the legacy wrapper and qualifies the revised boundary.

## Tangent replay findings at the Phase 11 boundary

At `143c712e5`, the contract for a G1 join requires coincident endpoints and
two parallel, nonzero tangents with the same orientation. With coincident
endpoints and tangents `(x, 0)` and `(y, 0)`, that orientation condition is
`xy > 0`. The solver model's cross product vanishes for every such pair and its
dot-product constraint accepts `xy >= 0`.

| Scalars `(x, y)` | Strict join classifier | Solver join, chain and oriented alignment |
| --- | --- | --- |
| (1, 2) | G1Continuous | All satisfied |
| (1, −2) | ReversedTangent | Rejected |
| (1, 0), (0, 1), (0, 0) | DegenerateTangent | All satisfied |

A separate direct-classifier witness uses an exact positive displacement
`h = 2^-2048` behind the sine addition identity. With identical tangents,
endpoints separated by `(h, 0)` were reported G1-continuous under
`APPROXIMATE_512`. With identical endpoints, tangents `(1, 0)` and `(1, h)`
were also reported G1-continuous. A tangent `(h, 0)` was reported degenerate.
The strict classifier returned `Unknown` for all three. The reports carry
no qualification for the terminal approximations. All three public solver
entry points and the direct classifier need their own acceptance witnesses
before tangent evidence can authorize downstream smoothing.

## Source inventory

The Phase 0 scanner now visits the full routing, tangent and solver modules
named in section 10.1. A fresh scan of those three files grew from one
declaration to 118. A separate Rust syntax-tree walk found exactly the same
64 routing, 34 tangent and 20 solver declarations, plus all 19 in the affine
axis-projection submodule. The inventory has an individual, source-bound
review for each of the 19 projection and 34 tangent declarations. The other
routing and solver entries remain lexical candidates. All 365 earlier manual
records survived two successive refreshes.

## Affine projection proof admission

On the primary branch, both public projection operations consume a sign or
comparison through `PredicateOutcome::value()`. This loses the distinction
between a certified zero and a terminal approximation. At `143c712e5` the
span with absolute derivatives `[1, −2^-2048]` and unit axis limits was
accepted under `APPROXIMATE_512` when the negative value was expressed using
the sine addition identity. Independent replay certified all six inequalities.
The positive counterpart was labeled `ZeroFeedRate`. An undecided ordering
was treated as a tie and reached `AxisProjectionProposalUncertified`.

Qualification commit `cce4eb9f9` requires certified decisions before either
public entry point validates its arguments. All three previously failing
witnesses then return `PredicateUnresolved` under both policies. Exact zero
still produces its distinct error, and a genuine tie retains the first row.
The zero-axis and exact-tie control kept three rational comparisons and 30
structural sign calls. Rust 1.95 and 1.99 each passed 450 all-feature tests,
three minimal-feature tests and strict rustdoc. The 199 Alumina Interface
core tests include all 18 machine-profile cases.

On both toolchains the guard preserved the complete output, dispatch, heap
and touched stack in each of the 1×1, 4×3 and 32×8 native workloads under
both policies and three trace modes. The two-fill bare-Wasm probes also
measured identical stack and linear pages for matching outcomes. Among 14
linked probe, example and benchmark images, its largest individual growth
was 64 file bytes and 32 executable-code bytes. The optimized Alumina browser
module remained 10,737,605 bytes and the desktop executable remained
13,931,312 bytes. The raw browser module grew by 12 bytes and its Brotli
delivery grew by 825 bytes.

The guard remains on the local `interval-axis-proof-qualification` branch.
On Rust 1.95 the complete 32×8 approximate workload with passive tracing
took 0.69% and 1.65% more thread CPU time on the two cores in the final
paired run. An earlier confirmation also found higher medians on both cores.
Moving the checks to each decision, reserving the exact row capacity, sharing
solver bindings, reusing row buffers, checking certainty receipts, and
iterating paired inputs were measured separately. Sharing bindings changed
the construction trace. Reusing buffers increased the Rust 1.99 shipped
example by 7,464 bytes against a 4,096-byte incremental cap. None of these
trials qualifies a path ownership transfer. The primary projection proof
gap and the tangent findings above remain open.

Measured sources, both exact oracles and raw trial records are archived in
evidence commit `1818ef32b8debc1724ae48647bd6562279928ba9`. [Evidence index](../evidence-archive.tsv)

## Direct G1 classification and positive orientation

Hyperpath `e8b1bef51` closes the three direct tangent failures recorded above.
Exact comparisons guard endpoint equality, squared norms, cross products and
orientation in the existing decision order. An unresolved alignment retains a
proved shared endpoint. Corner limits, schedule planning and independent
replay now return `PredicateUnresolved` for an unresolved join. Proved
degeneracy and mismatched endpoints still return `UnsupportedRouteGeometry`.
The regressions prime approximate caches and cover both public policies.
[Acceptance tests](../../crates/hyperpath/tests/tangent_certification.rs)

The nine solver-backed zero-tangent outcomes still reproduce on this revision.
The oriented problem currently permits a zero dot product. The csgrs trial
`7433a54bd` adds a strictly positive residual to the oriented model.
An independent point-ball case caught a false violation in its first version.
The corrected trial keeps a hidden positive or negative sign unresolved until
the sign is certified. Its Alumina Interface counterpart `19c9f6e` encodes
the new residual kind while preserving existing transcript tags. The older
exhaustive encoder fails to compile against the extension. Release coordination
and the broader solver resource checks remain open. The affine projection
proof and extraction findings above are separate open issues.
[Measurements and admission record](../../INTERVAL_REFACTOR_PROGRESS.md#phase-11-certified-tangent-decisions-and-schedule-replay)
