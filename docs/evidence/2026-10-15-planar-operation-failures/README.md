# Planar operation failures and matrix meaning

Apply `properties.md` and `docs/admission.md`, chiefly P1, P4, P6, P8–P9,
P17–P20, P25–P30 and P35–P39. CSGRS owns public modeling failure semantics
and admission of a homogeneous matrix into a planar affine operation.
Hypercurve owns affine topology and classification. Reuse its native kernel;
do not duplicate determinant/topology machinery in the adapter.

Reproduce errors that become empty transforms, undecided queries that become
boundary results, stringified native blockers, and ignored homogeneous or
out-of-plane matrix components. Supported planar affine matrices must preserve
homogeneous meaning, including constant nonunit/negative weights. Perspective
and out-of-plane maps require an explicit unsupported outcome. Constant zero
weight with a nonzero coordinate image is unbounded; an all-zero homogeneous
image is invalid. Empty geometry must not bypass invalid argument checks.

Tests retain independent exact coordinate/area/volume expectations, manifold
validation and downstream composition. Preserve original examples and benchmark
inputs/resolutions. Stronger admissible cached proof may strengthen Unknown;
qualified matrix-admission evidence must not become Certified at an identity
kernel shortcut. Operation-local certainty is separate from source provenance.

Previous goal turn made progress in `1916037c7`, `fcb269765`, `98bda8a48`.
This slice starts from the actual worktree at `98bda8a48`. The 35 pending native
files remain byte-identical and require separate admission. Trials are immutable
and use fresh targets; all failures and raw measurements remain retained.
Internal work, heap-byte and cancellation bounds remain open; adversarial work
uses external wall/address-space containment. Full plan and release validation
remain open. No push or CI changes.

## Reproduction

`baseline-v1` retains a direct-rustc harness failure: `cfg(test)` was omitted
from the explicit check-cfg declarations. `baseline-v2` compiles the unchanged
13 regression tests and reports 3 pass/10 fail. `baseline-v3` adds four adapter
regressions and reports 3 pass/14 fail, still against the preserved library
from `98bda8a48` and the unchanged native worktree.

The scalar adapter also converts typed errors to strings and ignores Z
translation and X/Y rotation. Its non-context translate/scale ignore non-finite
third components altogether. These APIs reach the C ABI, C++, Python and Go,
so the repair must use the same matrix-admission implementation and exercise
real consumers. A plane-preserving X half-turn is a reflection, not identity;
a quarter-turn that leaves the XY plane is unsupported for this carrier.

The valid controls establish exact boundary/inside/outside classification,
empty-transform composition, and area 18/volume 36 with separate manifold
validation for reflected, translated and rotated regions with holes. No test
fixture or policy was reduced to obtain these results.

## Implementation and qualification

Primary planar transforms and membership now return typed errors. The shared
matrix admission preserves homogeneous meaning and uses native affine topology;
scalar adapters preserve all three input components and X/Y/Z rotation order.
Gerber, Hypercircuit, native bindings and WASM preserve the native cause. PCB
placement transforms use the selected materialization policy and aggregate
certainty. Fabrication failures cannot become empty manufacturing images.
The root and binding READMEs document the API and error-enum migration.

Twenty final regressions cover exact coordinates, holes, area 18, volume 36,
separate manifold validation, affine-cut commutation, empty composition,
qualified identity shortcuts, native blockers, mutation and recovery.
`candidate-v1` passed its earlier 17 cases. `native-v1` retains a failed FFI
test: the first matrix implementation classified an all-zero image as unbounded.
The implementation was corrected and the original invalid-input expectation
retained. Its profile benchmark filter selected zero cases and earns no
validation credit; `native-v2` uses the actual original case name.

All 22 final native commands pass: formatting, strict Clippy, root tests across
all features (313 pass, one existing ignored image-corpus test), minimal features
(38 pass) and curve-only (128 pass), fuzz-target Clippy, docs/doctests, original
benchmark consumers, renderer build, FFI checks/builds and Hypercircuit
docs/doctests. These counts overlap across configurations.

The full Hypercircuit library/integration execution has 341 passes and seven
fixture failures. The six Easyduino snapshots differ only in 160 DRC
`check_version` fields each. The manufacturing archive differs in
`evidence/hyperdrc.json` and its dependent manifest digests; reproducibility
assertions preceding that golden-fixture comparison pass. The reconstructed
starting revision, with the same 35 pending native files, reproduces all seven
actual and expected byte pairs exactly. `downstream-comparison-v1` retains the
decoded reports and archives and verifies all 2,593 reconstructed source inputs.
Neither trial is relabeled as passing and no expected fixture is updated.

Cargo initially stopped at Easyduino. The remaining 32 compiled integration
test executables were then run, including fabrication and materialization.
The first remainder setup encountered stale fingerprints without executable
files; its failed invocation is retained. The second records hashes and runs
the actual compiled executables. Hypercircuit all-feature/all-target strict
Clippy passes.

The immutable validation runner's early domain prose predates the all-zero
image correction and the downstream scope extension. This document and the
final closure qualify those details; the executed runner and its protocols
remain unchanged. All native sources, fixtures and policies stayed fixed during
the final validation phases. Evidence preparation did not alter those inputs.

All 43 native-binding commands pass, including actual C/C++/Python clients for
seven regression groups and the Go consumers. WASM-target strict Clippy and the
generated Node/TypeScript clients pass. The new planar WASM cases exercise 48
negative operations plus certified homogeneous normalization, membership and
extrusion under both policies.

## Completed-work cost probe

`cost-v1` stops after baseline compilation and a candidate Rust type error,
before any timing. The standalone library build selected a different
Hyperlattice instance from the baseline example build. The initial
`cost-build-identity-v1.json` comparison of compilation-unit settings and two
available libraries did not establish the actually resolved dependency graph;
it is insufficient evidence and is retained as part of that failed preparation.
`cost-build-v1` then uses the exact baseline renderer/example build command.
`cost-v2` checks that command, core library bytes, source identity and compiled
type compatibility before measuring. The harness and workload stay unchanged.

All 114 commands pass: two builds, 16 exact-output dumps and 96 fresh-process
samples. The probe completes 12,288 timed strict operations and preserves all
1,024 exact output variants per revision. Timing includes input construction,
operation, independent exact coordinate/area proofs, boundary materialization,
serialization, checksum and drop. The query case also checks independently
known inside, outside and boundary facts. Warm repeats one input; varied cycles
128 widths and does not imply globally cold scalar caches.

Median candidate latency change over three samples per case:

| Completed workload | Warm | Varied | Extra allocations / requested bytes per operation |
| --- | ---: | ---: | ---: |
| Translation | -8.9% | -6.4% | 0 / 0 |
| Scaling | +10.1% | +8.7% | 0 / 0 |
| Rotation | +7.5% | +12.9% | 0 / 0 |
| Affine matrix | +15.1% | -2.4% | 0 / 0 |
| Membership queries | -4.7% | +1.0% | 0 / 0 |
| Scalar-adapter translation | +15.2% | +6.4% | 0 / 0 |
| Scalar-adapter scaling | -4.0% | +3.2% | 0 / 0 |
| Scalar-adapter rotation | +10.3% | +1.8% | 15 / 792 |

The shared, unpinned host and allocation instrumentation limit timing claims.
All favorable and unfavorable samples, retained/peak allocation observations,
process resource logs and binary sizes remain available. The all-feature rlib
changes from 9,869,558 to 9,852,650 bytes. This is a correctness and API migration,
not an optimization or a claim of supported-corpus speedup. The extra rotation
allocations accompany preserving the formerly ignored X/Y components.

Internal total-work, expression-growth, retained-heap and cancellation bounds
remain open. Trials use external wall and per-process address-space containment;
Node/WASM has wall containment without an address-space limit. Three compiler
consolidations share only identical non-executable files in closed trials,
retaining every path and byte. Executed scripts, failures, fixtures and unique
artifacts are preserved. No clean external-package, full release, pre-push or
complete-properties qualification is claimed.
