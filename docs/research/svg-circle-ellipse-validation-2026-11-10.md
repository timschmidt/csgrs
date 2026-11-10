# SVG circle and ellipse validation (10 November 2026)

Status: **finite implementation qualified locally; publication pending**.
Findings were reproduced on `9490c8bb0d6044105c4fcd7202c679736c39b526`.
The shared-validator implementation passes 164 affected native reports, 34
selected capability records, strict owner/adapter lint, 35 reports and strict
lint on the declared minimum compiler, 99 zero-import WASM executions, and 996
BRep/IO reports with strict lint. These results do not qualify all 463 catalog
rows, general SVG source conformance or a new main publication.

## Contract and owner

Hypercurve owns primitive admission in `src/svg.rs`; CSGRS transports its result
through the SVG adapter. P1, P5, P6, P8–P10, P16–P20, P25–P27 and P39 apply
within the witnesses' separate scopes. The input domain is the existing finite
numeric attribute syntax for circle/ellipse centers and radii, with visible,
unpainted, zero-opacity, display-none and hidden selections. General SVG units
and visible zero-radius semantics remain outside this change.

Authored invalid fields must remain invalid before selection. Valid selected
empty geometry must not require unused sampled coordinates. A visible exact
positive radius that the legacy binary64 sampler cannot represent must report
`Unsupported`, distinct from malformed input, successful empty and exhaustion.
Previously successful hidden zero-radius empty results remain successful.

The shared numeric validator retains exact decimal meaning and calls the scalar
owner's sign machinery. Large scientific exponents use the existing bounded
rational parser and exact lazy power fallback. No new scalar decision algorithm
or region carrier is introduced. Visible output uses the original ellipse
sampling calculation, with its existing represented polygonal model.

## Current-revision reproduction

The original owner run reported one passing preservation control and three
failures: hidden negative radii bypassed validation, valid unpainted tiny shapes
attempted sampling, and visible positive underflow was reported as malformed.
The retained hidden zero-radius control already passed on the baseline.

Two CSGRS adapter witnesses separately failed on the unchanged producer. A fifth
owner witness, using radii `+/-1e-1000000000`, also failed on the unchanged
producer. These cases are retained alongside the original smaller inputs.

| Actual baseline run | Reports | Command-log SHA-256 |
| --- | --- | --- |
| `svg-circle-ellipse-validation-red-1-20261110` | 1 passed, 3 failed | `1cf66e87ab1b361af9215b4d97b4db2238310ddd2e0f1599a9dc31267cea27ab` |
| `svg-circle-ellipse-root-red-1-20261110` | 0 passed, 2 failed | `dc08201f14164b4f9f65ed8a34af07a48779f43437e3a380e12b8081e0131265` |
| `svg-circle-ellipse-lazy-scale-red-1-20261110` | 0 passed, 1 failed | `93bd8df8ea343c39e68a452beeeb9b6739302b3d3bffe128ff0ec81f72f7927f` |

Each actual Cargo exit was 101, with unchanged source and no external timeout.
The owner fixtures and their assertions were preserved when the large-exponent
case was appended. Production remained unchanged until all baseline runs ended.

## Ordinary-output control

Before the edit, four ordinary visible circle/ellipse documents completed under
both policies. The eight observations retain complete SVG artifacts, native
replay and deterministic re-export, an exact region XOR, certainty, dispatch and
rational counters, and time for the completed work. They do not compare the
sampled model to an analytic circle or ellipse. They run in one process; this
does not establish fresh-process replay of these sampled models.

The actual baseline run `svg-circle-ellipse-sampled-baseline-1-20261110` exited
zero with unchanged source; its command-log SHA-256 is
`a286fa8f3af905d206c4233102bbff1e0580f615fe7d6ce956b7df52c436e90d`.
All eight candidate artifacts and certainty receipts match in full. The extra
work is one exact radius parse and structural sign check per circle, and two
per ellipse. Independent literal fractions predict every dispatch delta. The
rational trace adds one temporary per radius; other rational counters,
including reductions, GCDs and peak operand width, are unchanged on these
controls. Other dispatch counts are unchanged. This is an intentional
validation correction, not a behavior-preserving source-size refactor.
One time observation per case is not a benchmark or a speedup claim.

## Qualified implementation and remaining work

The importer validates circle/ellipse fields before empty selection. It shares
the existing exact numeric validator, preserves hidden zero-radius empties,
and separates positive sampler underflow from invalid input. Sampling is
deferred until painted visible geometry is required. The original sampler body
is preserved byte for byte.

The first candidate's five owner tests passed, including the large-exponent
case, under both policies. Its actual run
`svg-circle-ellipse-owner-green-1-20261110` has command-log SHA-256
`1d122e7f207622016ab0d49c768757772717ff94143442f0db6929b9d8eb15b3`.
The final shared-validator source is qualified separately:

| Actual run | Completed scope | Command-log SHA-256 |
| --- | --- | --- |
| `svg-circle-ellipse-affected-qualification-2-20261110` | 12 SVG unit, 132 owner integration and 20 adapter reports; 17 evidence-runner unit tests; 34 selected capability records; eight ordinary controls; strict owner/root lint | `dbd49be83630bf9ae34ca36405ce0ec2ad8705af2fe8293afd003735800f4dbd` |
| `svg-circle-ellipse-msrv-1-20261110` | Rust 1.95.0: 33 maintained owner and two adapter bodies; strict owner/root lint | `b7f8b2a27e70d16f71ed60c4c3e3b2b588f32086fd5c3d160048026fdcfb380b` |
| `svg-circle-ellipse-wasm-2-20261110` | 33 maintained owner bodies in each of three fresh instances; 99 executions, zero imports | `33c467be2160a112d639ac6a2b9f5da3b322ea321405b45cf97f515c07cbbeae` |
| `svg-circle-ellipse-brep-downstream-1-20261110` | 110 BRep/IO harnesses, 996 reports, no failures/ignores; strict lint | `57c6a3234717f2cc7562684e1eb5e56e6be6158c79aa1d74827c2d0dc81c40b4` |

All four actual runs exited zero with unchanged 3,844-input source snapshots
and no external timeout. Portability adapters retain the original function
bodies, assertions and native process cfg guards. WASM uses the original
compiled stack and VM flags; it does not qualify the CSGRS adapter or a public
ABI on that target. Admission/index edits after these runs change documentation
only. A new source commit and complete main publication gate remain required.

Five catalog rows declare their different outcomes; the original 458 objects
remain unchanged. Successful empty replay/continuation uses an independently
authored unit-square boundary. Error assertions check the typed public variants
separately. No row claims analytic fidelity, general SVG semantics, complete
algebraic operation closure or total resource control.

The first affected attempt retained a read-only temporary-directory failure
from a fresh-process replay. The unchanged complete command passed after moving
scratch files to writable `/tmp` storage. No assertion or fixture changed.

The first WASM attempt compiled the module, then failed before any case when
the newly added 8 GiB virtual-address cap prevented Node from reserving memory.
The unchanged, previously qualified 28-case parent module reproduced that same
initial-instantiation failure under the cap. The successful retry uses the
identical module and fixtures with separate compiler/Node address-space limits:
8 GiB for compilation and 64 GiB for VM reservations. It preserves the original
VM flags, compiled stack and assertions. These failures are neither geometry
results nor typed native exhaustion; all failed attempts and the parent control
are retained.

The final local harnesses contain execution at 7,200 seconds, with the address
limits above. Native/MSRV/compiler processes use 8 GiB. The capability rows
declare a separate 300-second execution timeout after the build. These are
external containment, not total arithmetic, heap-memory or cancellation bounds.
Compiler-inclusive peak RSS and WASM linear memory are not geometry live-heap
measurements.

The local immutable record
`docs/evidence/2026-11-10-svg-circle-ellipse-validation` contains 190 files and
25,139,407 bytes, digest
`fe4bc4367d46392901541bea2607e676a1c58d856f7c0e7db12b5e49dc9e6c35`.
Its commit `0a1da64a17b4ff0912b58183d4bc42149511cf10` extends the consumer
cohort archive without changing prior records. All 457 manifest records verify
from local Git objects; the full-manifest SHA-256 is
`82125068993f9793ff1183d09b27f9d996c226df4efbc4aabe129159861b23d7`.
No evidence upload occurred. The source index accompanies this actual
implementation increment. New main publication remains pending; the earlier
B-text and NURBS completion gaps are unchanged.
