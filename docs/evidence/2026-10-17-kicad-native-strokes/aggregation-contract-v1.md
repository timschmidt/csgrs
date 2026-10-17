# Native board aggregation prerequisite

The executed candidate v1 has ten passing source-fidelity tests, but the
unchanged six-case range migration stops at `BoardModel::all_copper` on its
first fixture. `report-projection` rejects the width `1e-400` finite ring.
This is not a failed native construction. The owning invariant is shared
PCB profile aggregation (P1, P4, P6, P17–P20, P25, P35–P37): exact regions
must not require a finite report view before composing.

Before changing that implementation, run new public-entry-point regressions
against the completed candidate-v1 libraries, read-only. Cover both board
aggregation entry points, canonical empty, invalid inputs paired with empty,
non-projectable exact strokes, holes and overlapping material, disconnected
components, a native quadratic carrier, layer metadata/selection, reused
proofs across strict/approximate/strict scopes, mutation and budget recovery.
Area and point expectations are analytically derived, not wrapper agreement.
Validate every selected input before identity shortcuts. Native aggregation
must reuse the existing shared profile implementation and preserve typed
failure; do not introduce an adapter geometry kernel. Report conversion stays
an explicitly fallible boundary. Retain all earlier failed tests and sources.

This prerequisite does not waive any original workload, fixture, resource
assertion or downstream validation in `contract-v1.md`. Changes to report
representation require their own recorded migration and evidence. No total
work, retained-heap or cancellation bound is claimed beyond existing APIs;
diagnostics run under external containment.
