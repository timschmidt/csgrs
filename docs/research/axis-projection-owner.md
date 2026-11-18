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
without copying them. Both root and routing-module legacy paths remain tested.
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
