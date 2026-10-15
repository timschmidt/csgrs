# Slice and projection acceptance contract

Target baseline: `99849fe84`, plus the same 35 pending native source files
preserved by the preceding planar-operation closure. Those files remain in
scope for separate admission and must not change in this trial.

The missing acceptance tests are in `tests/slice_projection_contract.rs`.
P8, P13, P17–P20 and P25 require precise policy, winding, composition,
retention and typed failures. Hypermesh owns oriented mesh intersections;
Hypercurve owns boundary winding and regularization; CSGRS owns modeling
composition and error propagation. Existing implementations swallow errors
in the primary APIs and independently fill every closed slice loop. The
baseline trial must reproduce these findings before production changes.

The operation domain is finite indexed triangle geometry, with open-sheet
section wires retained. Closed sections use signed nonzero winding, including
holes, nested or duplicated material shells, global reversal and disconnected
components. Coplanar faces contribute projected area. Isolated point contacts
and zero-area projected faces are omitted by the existing regularized area/wire
carrier. This does not claim a full intersection carrier of every dimension.

Exact rational area, point membership, open-wire endpoints and determinant
volume are independent semantic oracles. A subsequent extrusion is separately
checked for manifoldness. Certainty is operation-local; none of these tests
certifies inherited analytic intent or source construction provenance.

Trial commands, toolchain, source snapshots, streams and resource measurements
are retained unchanged. Baseline Rust linking reuses the preserved all-feature
example library and its exact dependencies; it does not rebuild in a closed
target directory. Changed commands or source require a new trial. Each command
has 1,800 seconds wall containment and 8 GiB per-process virtual-address
containment. These are external test limits, not internal work, memory or
cancellation guarantees. No performance claim or full-plan completion is made.

## Baseline reproduction

`baseline-v1` compiled successfully against the preserved target revision.
Six tests passed and ten failed. Interior, top, bottom, reversed, disconnected
and cached hollow sections filled their holes (area 16 instead of 12, or 32
instead of 24 for two components). Four primary-API tests observed successful
empty geometry for invalid indices or undecided predicates. Empty composition,
open-wire endpoints, nested and duplicated positive shells, projection and
strict/approximate policy separation passed. Original streams and the direct
Rust executable are retained; no production code changed in this trial.

## Implementation and initial qualification

`baseline-v2` extended the reproduction to 21 cases (eight pass, thirteen
fail), linked against the same preserved baseline library even though its
snapshot also records the in-progress candidate source. The baseline binary
is authoritative for that trial; the source snapshot is not a claim that the
candidate had already passed. Original baseline-v1 remains unchanged.

The native Hypermesh section routine retains directed intersections, source
triangles, edge identities and exact parameters. It decomposes directed
chains for the limits approached from above and below. CSGRS delegates
nonzero winding across all contours to native Hypercurve; signed coplanar
faces contribute separately. Opposite traversals cancel and nested equal
orientations accumulate. Source witnesses are available in the native section
record; the planar return value does not expose these mesh-source records.
Primary slice/projection APIs return typed errors. Original example and
benchmark fixtures were migrated to explicit Result handling without changing
their inputs. Corrected topology and the public Result API are deliberate
contract changes, not a performance-only patch.

Candidate-v1 passed 21 modeling tests and 39 native tests selected by the
section filter (the filter also selects intersection tests). Candidate-v2
passed 23 modeling tests. Candidate-v3 added duplicated position rows and
randomized face ordering; 24 passed and one failed because construction
identity prevented matching distinct indices at the same coordinates. That
trial is retained as a failure. Candidate-v4 separates identity-based proof
reuse from geometric point interning and admits two-edge retraced contours
through native contour construction. All 25 modeling tests, the same 39
native tests, Hypermesh all-feature/all-target Clippy and CSGRS curve-only
all-target Clippy with warnings denied pass. No tests or policies were weakened.

Compiler consolidation v1 hard-links only byte-identical non-executable
compiler files in closed candidate-v1 through candidate-v3 targets. All paths,
bytes, unique files, executables and streams remain available. Fresh phases
copy seeds with independent inodes. Broader qualification and resource-cost
measurement remain pending; this initial success is not full-plan acceptance.

## Broader qualification

Implementation commit `07258a70d` passes all 14 commands in `native-v1`:
root/fuzz formatting; default and all-feature Hypermesh tests; Hypermesh docs;
root all-feature and minimal all-target Clippy with warnings denied; root
all-feature, minimal and curve-only library/integration tests; root docs and
doctests; the original projection benchmark; and the README renderer build.
Hypermesh totals are 237 and 238 passing tests, each with seven existing
ignored tests. Root totals are 338 passing plus one existing ignored test,
96 passing, and 231 passing respectively; the root doctest also passes.
`renderer-v1` executes the full unmodified-resolution example and produces all
73 PNGs in its preserved target directory. The original 24×12 sphere benchmark
executes all three projection cases (nine sample rows, 528 input triangles),
with successful nonempty results. Benchmark counts alone are not the semantic
oracle; the independent regressions establish the represented sets.

## Completed-operation resource costs

`cost-v1` passes 82 commands: two probe builds, eight exact-output dumps and
72 fresh-process timing samples, with 128 timed operations per sample (9,216
completed timed operations). Both revisions pass exact square-area, membership
and all-boundary-edge checks for 512 dumped variants per revision. Work includes
input construction, operation, validation, materialization, expression
serialization and output drop. Three samples per case alternate revision order.
Warm repeats width 2; varied cycles 128 widths with default modeling-cache
limits; disabled uses those widths with modeling retention disabled. Native and
scalar caches remain outside that control. Raw unfavorable samples are kept.

Observed median latency changes versus the preserved baseline:

| Operation | Warm | Varied | Modeling caches disabled |
| --- | ---: | ---: | ---: |
| Middle slice | -4.3% | +321.4% | +321.4% |
| Bottom slice | -15.0% | -4.0% | -3.8% |
| Top slice | +6.4% | -10.2% | -4.7% |
| Projection | -0.3% | -2.0% | -2.5% |

The middle-slice regression is substantial: about 600 → 3,300 allocations
and 82,013 → 370,358 requested bytes per varied/disabled completed operation.
Warm allocation counts are unchanged (57 per operation). Bottom/top slices
use fewer allocations; projection counts are unchanged. This is a correctness
repair with a performance follow-up, not a retained optimization. An obvious
follow-up to investigate is reusing the same winding proof when the above and
below directed sections are structurally identical; that change must preserve
this certified corpus, certainty and declared output contracts.

Middle/top/projection dumped rows remain byte-identical. Bottom-slice rows
change traversal while retaining the same byte count and independently proved
set; this is part of the documented correctness/API migration. All three timing
samples reproduce their revision's checksum and serialized size. The report
retains CSGRS and Hypermesh archive sizes and both probe executable sizes;
moving code into Hypermesh must not be mistaken for a binary-size optimization.

Build identity was checked before timing. `build-identity-v1` reads actual
Rust metadata; `build-identity-v2` confirms the same toolchain, 901 unchanged
core/configuration source files, Cargo compilation-unit configuration, target,
ABI and resolved dependency names. Fresh and seeded archives nevertheless have
different encoded crate hashes. The cause is not established; matching file
names or Cargo fingerprints alone were not treated as proof of actual linkage.
Each probe links its own preserved Rust dependencies and compiles against their
actual types. The raw archives, metadata and commands are retained. Rebuild
variance, allocator instrumentation and the shared unpinned host limit timing
inferences. No bit-identical-dependency or corpus-wide performance claim is made.

## Remaining admission

Native section records retain source indices and parameters relative to the
call input; planar results do not expose those mesh-source records. This does
not establish end-to-end construction provenance or analytic fidelity. Internal
total-work, heap-byte and cancellation limits remain unimplemented; external
trial containment is not a substitute for those contracts.

The 35 pre-existing native files remain byte-identical and unadmitted. Seven
previously reproduced Hypercircuit snapshot/golden failures remain unresolved;
this slice does not re-execute those unrelated consumers. Full CI and clean
external release/package admission remain pending. No push was performed.
Only `current_work` changes in the execution ledger; earlier status entries
and bytes outside that field remain intact. The full properties goal remains
active. Qualification and preservation manifests are in `qualification-v1.json`
and `closure-v1.json`.
