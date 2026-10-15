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
