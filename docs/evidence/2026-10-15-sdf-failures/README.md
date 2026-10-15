# Typed SDF construction and standalone-grid composition

Implementation: `0a1ecb0ac`. The closure verifies 330 retained executable paths.

The authoritative qualification is [`closure-v1.json`](closure-v1.json),
produced by [`close_v1.py`](close_v1.py) after the implementation commit. It
records that commit, final source identities, executed commands, raw-output and
executable hashes, the unchanged 35 pending native files, and the restriction
of ledger edits to `current_work`. The overall properties plan remains open.

Hypersdf owns scalar construction and its domain/branch diagnostics. CSGRS owns
the public modeling contract and shared sampler. Hypermesh owns face incidence
and the distinction between chunk and standalone-grid extraction. Relevant
properties are P4, P6, P8, P9, P18, P20, P25, P35 and P37.

The four SDF convenience constructors now return `Result`. Invalid expression
domains, undecided branches or samples, arithmetic failure, invalid resolution,
capacity exhaustion and extraction failure cannot become successful empty
geometry. `SdfSample` preserves the failing grid index and native error chain.
`Sdf::evaluate_point` and `SdfPrimitive::try_scalar_value` retain strict native
construction checks; their existing classification/preview consumers share
that implementation. The older primitive `scalar_value` explicitly discards
errors into `None` for compatibility. Context modeling APIs retain the certainty
actually consumed; approximate meshing does not authorize approximate scalar
construction branches.

The new semantic plane test initially found vertices without any faces in the
last grid layer. Investigation showed that native `surface_nets` deliberately
uses a documented chunk ownership convention. That contract is retained.
The modeling sampler now selects `surface_nets_standalone`, which uses the same
implementation and includes every face whose four incident cells are present.
This is an explicit modeling-output migration. No performance-only or unchanged
serialization claim is made. The display preview keeps its existing convention
and remains separately labeled `PreviewOnly`.

Other migrations are explicit: resolution below two is rejected instead of
clamped, and diagnostics fail when requested signs or extrema cannot be
established. Existing valid formulas, policies and fixture resolutions remain.
Numerical certainty applies to the sampled level-surface proposal; it does not
prove closed topology or fidelity to the continuous field. The root and native
READMEs describe the consumer migration.

Acceptance evidence includes independently derived scalar values for all eight
primitives; exact rational plane positions, sample counts and extrema; exact
unit oriented area for first/last-layer planes on every axis; preservation of
the legacy chunk convention; typed domain/uncertainty/arithmetic causes;
recovery after partial sampling failure; empty union/subtraction composition;
and strict/approximate/strict certainty and preview checks. Existing extreme
scalar-magnitude, closed-sphere, downstream Boolean and TPMS fixtures remain.

| Trial | Result and scope |
| --- | --- |
| `baseline-v1` | Preparation failed before tests because the target parent directory was absent. The failure and runner identity are recorded. |
| `baseline-v2` | All eight regressions fail on `a8ca70322` plus the original in-scope native changes. |
| `candidate-v1` | Compilation fails on a missing qualified scalar type. |
| `candidate-v2` | Five native evaluation tests pass; CSGRS compilation exposes a now test-only mesh context constant. |
| `candidate-v3` | Five native tests and 38 focused modeling tests pass; the new plane test finds omitted final-layer faces. |
| `candidate-v4` | Three native grid tests, five evaluation tests and 46 focused modeling tests pass; strict Clippy rejects test loop style. |
| `native-v1` | Root/fuzz formatting passes; nested Hypersdf fuzz formatting fails. No build or target executable runs; its unused copied build cache is removed with a recorded audit. |
| `native-v2` | Runtime suites and root lint checks pass; nested Hypersdf fuzz Clippy rejects a pre-existing range loop. This trial predates the explicit native chunk/standalone split. |
| `native-v3` | Final qualification: all 17 commands pass on unchanged final sources. 213 CSGRS, 236 Hypermesh, 110 all-feature Hypersdf, 109 default Hypersdf and 10 FFI tests pass. Minimal-feature tests (38), SDF-only tests (18), strict all-target/minimal/SDF-only/fuzz Clippy, formatting, strict docs and one doctest pass. Feature configurations overlap. |

Each executed phase is immutable and retains its command list, environment,
source hashes, tracked patch, untracked-source archive, raw outputs and terminal
result. Failed phases are not resumed or pooled with later successes. Final
qualification uses a fresh named phase and build directory. `validate_v2.py`
creates the missing target parent; it does not revise the failed preparation.
The exact final documentation and runner inputs are recorded separately.

Eight existing tests remain ignored: one CSGRS rendering case and seven
Hypermesh external-fixture/manual scale or memory cases. Examples and benchmark
consumers compile under strict all-target Clippy; rendering and benchmark
measurement were not run. Fuzz targets are compiled/linted, not fuzzed for a
duration. There is no C/Go/Python/WASM SDF-construction binding to exercise;
FFI unit tests pass, but this phase does not repeat live binding consumers.
The full workflow matrix, sanitizers, clean external distribution, release/MSRV
validation and performance comparison remain open.

Primary sample, coordinate and preview buffers use checked reservations. Scalar
expression growth, secondary extraction/preview allocations, total computation,
peak heap usage and cancellation still lack internal bounds. Each command has
an external 1800-second wall limit; no internal resource bound is claimed.
Separate metaball fallbacks and approximation policy, other modeling failures,
end-to-end provenance, ordinary strict-workflow completion and earlier scalar
and native admission gaps remain open. No push was performed.
