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
