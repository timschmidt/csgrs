# Downstream API admission, 4 November 2026

This completes the local review and commit of D01/D02/D04, with qualification
limits. It does not close the remaining research program. CSGRS code is
`d9b06483a396d68e16ef18438f38070e31854372`; SynapsCAD implementation is
`657e6b14768fe93f9c7149457fc1d1c817552e13`, followed by the documentation-only
migration note `4fcb1d3`. openscad-rs remains at `c0d92e6`.

## Contract and reproduced witnesses

The application evaluator and display/API boundary own this change: P6,
P25–P30, with P23 for shared arrangement evaluation and P35 for regressions.
The bounded domain is the existing supported OpenSCAD models and finite 3D
Boolean/transform subtrees, including failures, inherited qualification and
canonical empty geometry. Curve regions retain their kernel authority. The
application creates no replacement predicates or geometry kernel.

Deferred failures formerly acquired the enclosing union's span. The retained
kernel error path now selects matching shared source metadata through balanced
folds, transforms and repeated module calls. Unlocalized kernel errors retain
the enclosing statement rather than inventing an operand. Failed parts retain
their numbers, typed cause and location; successful parts retain construction
evidence. A failed second part no longer changes the third part's palette.

Missing acceptance witnesses exposed two further false successes: orphan
positions counted as canonical empty projection, and `compile_views_only`
returned empty or partial previews despite failed parts. They now fail
explicitly, while genuinely empty source succeeds. The full compilation API
retains typed failures alongside successful parts; callers must inspect both.
The terminal preview helper exposes text errors through its existing signature.

Compiler and phase benchmark reuse `part_mesh_data`. A custom `cube(-1)` input
formerly yielded benchmark exit zero and checksum zero; it now rejects the
incomplete run. An initial source-read hypothesis about lost lazy geometry did
not reproduce: evaluation already resolves roots before benchmarking. Shared
projection is retained for consistent failure/empty handling and metadata,
with no claim of recovered geometry or measured speedup.

Batching retains independent exact volumes 880 and 60, exact bounds, and the
four-child one-arrangement witness. It may change triangulation and vertex
ordering relative to eager evaluation. SynapsCAD documents this intentional
representation migration separately from exact-set and provenance contracts.

## Executed qualification

Raw commands, source stages and logs are in the evidence record below. Final
Rust source passed focused span, palette, canonical-empty, preview-helper,
multi-child and arrangement tests; all 32 binary/UI tests; warning-denied
all-target Clippy with `dispatch-trace`; locked fuzz-target compilation; WASM
binary compilation; warning-denied library rustdoc; native release build; and
format/whitespace checks. The pedantic strict-collection zizmor audit passed
after correcting six local-action reference findings. Original hard inputs,
assertions and workflow exclusions remain unchanged.

The unchanged configured command was executed with an external 1,200-second
containment limit:

```sh
CCACHE_DISABLE=1 CARGO_BUILD_JOBS=8 \
  CARGO_TARGET_DIR=/tmp/csgrs-research-target CARGO_NET_OFFLINE=true \
  timeout 1200 .github/scripts/test-ci.sh
```

It terminated with exit 101 after 800.90 seconds: **59 passed, two failed,
two ignored, 53 filtered**. This run preceded the late empty/preview-helper
guards, whose regressions subsequently passed separately. It is not final
whole-suite green evidence. The two failures remain open:

- `geb_letter_solids_are_closed`: B extrusion exceeds its existing one-minute
  cooperative deadline. A live stack with the verified executable supplied to
  the debugger locates work in Hypertri hole bridging and exact squared-distance
  comparisons. This single snapshot is diagnostic evidence, not a profile or
  general complexity claim. Cancellation does not interrupt that kernel call.
- `near_coincident_boolean_covers_signed_axes_and_angle_syntax`: triangle 25
  collapses at the finite f32 rendering boundary. It remains a typed failure.

Actual native UI inspection used isolated temporary settings and captured only
the launched app's own window: successful blue `@1`, failed `@2` with
kind/location, and sampled green `@3`. Production compiler previews of a
rectangular through-hole plus disconnected cube were visually compared with
OpenSCAD 2021.01 on the identical source. These bounded checks do not establish
the Manifold differential corpus or general geometric equivalence.

## D04 and retained work

Alumina Interface `efd379f831b66b61fa871a2137be9dca89235292` commits its
introductory README and `docs/CURRENT-BASELINE.md`. Against base `2c49c81`,
553 narrative lines match after canonical relative-link rebasing. Alumina
Firmware `62d3f6fb9c69846a4a9fdba7fc74f4aeec731392` commits its README and
`docs/CURRENT-STATUS.md`; 962 narrative lines match base `bdee397` after the
same rebasing. Every relative file target resolves. Scoped whitespace checks
pass. These are documentation-fidelity checks, not new application/firmware
runtime or hardware qualification. Other pending planning/contributor edits
remain untouched.

The integrated CSGRS tree was promoted to local `main`; the previous integration
tip has a dated archive tag. Ten checked-out historical branch controls remain
untouched; see the [consolidation record](consolidation-2026-11-04.md).
No source code changed in this receipt-only CSGRS update. Its affected checks
are formatting, links, whitespace, archive integrity and repository budget;
the earlier kernel qualification remains attributed to its original revision.

## Evidence and open scope

Local evidence branch `downstream-api-admission-2026-11-04`, commit
`668cde7f42cfb3ea8e86d09cff85058ff69a6c9f`, retains record
`docs/evidence/2026-11-04-downstream-api-admission`: 70 files, 468,721 bytes,
SHA-256 `22315100e54bfd9404f3b5cb2bccd1c88ec0b982afe1bf35107b80e172ec1d92`.
The record includes source revisions, stage patch, regression/validation logs,
visuals, replay helpers and bounded debugger receipts. Earlier evidence is
unchanged. The [archive index](../evidence-archive.md) supplies retrieval context.

No push is qualified by this work: configured CI still fails. Hosted dependency
pin availability is unestablished. Installed distribution, full browser runtime,
Manifold comparison, formal/coverage claims and broad operation closure remain
open. Next investigations must reproduce on the committed tree. Application
export's quantization/welding and retained-source contract require review before
relaxing the f32 boundary; hole-bridging cost needs measured attribution before
optimization. Source inspection alone establishes neither a fix nor admission.

A subsequent [export equality correction](export-equality-2026-11-04.md) at
SynapsCAD `124c31d` removes a reproduced tolerance merge of distinct projected
vertices. Its 35 binary/UI tests pass; native-source export and the two earlier
full-library CI failures remain open.

The subsequent [native-source admission](native-source-boundaries-2026-11-04.md)
at SynapsCAD `1f1ec3e` supersedes renderer-buffer export and resolves the
near-coincident failure. Configured CI now has 64 passes, one B-text failure and
two ignores; conversion costs and remaining boundary/visual limits are retained.
