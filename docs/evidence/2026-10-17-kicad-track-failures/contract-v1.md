# KiCad track-source failure contract

Base revision: `ac14e99174318291a548749395552ab982a7d097`.
Ownership: Hyperdrc's KiCad source adapter, reusing SourceScalar parsing and
the existing guarded PcbRegion failure carrier. P1/P6/P8/P9/P17/P18/P19/P20/
P25/P26/P30/P35/P36 apply. No geometry or predicate kernel is added here.

Reproduce current public loading behavior before changing production source.
Required start/end coordinates and width must not be silently omitted,
defaulted, truncated, or treated as successful empty geometry when malformed.
The original malformed-track fixture remains intact; expand individual cases
so a first assertion failure cannot hide later input categories. Validate all
required source fields before accepting zero width as canonical empty.

Failure must survive ambient policy, cloning, source scope exit, emptiness,
area, bounds, point classification, Boolean composition, report projection,
native handoff and board aggregation. Retain feature identity and diagnosis;
failed source metadata must not become a successful origin marker. Exercise
budget exhaustion separately from malformed source and verify recovery.
Existing PcbGeometryUncertainty diagnostics are reused; a complete typed
cross-layer failure-category taxonomy remains a separate open requirement.

The current source and valid workload corpus are authoritative. Preserve the
original full two-board, copper-neck and Gerber fixtures, policies and limits.
This work makes source failures visible; it does not certify the legacy finite
track stroke's fidelity, round caps, extreme-coordinate handling, or all
previously proposed exact-stroke regressions. Never count a faster error as a
completed geometric workload or silently change those unresolved expectations.
Any unavoidable supported-contract migration must be stated before admission.

Run all Hyperdrc tests, original workload gates, warning-denied all-target
Clippy and docs, formatting, public API pipelines and actual CLI consumers.
Retain all attempts, full inputs, raw outputs, policies, source/dependency and
toolchain identity, elapsed/CPU/RSS and artifact sizes. Compare complete
deterministic reports for valid fixtures where applicable; inspect malformed
CLI coverage and error visibility. Use external 1800-second / 8-GiB process
limits; no new internal total-work, heap-byte or cancellation bound is claimed.

The primary format reference is KiCad's [track segment grammar](https://dev-docs.kicad.org/en/file-formats/sexpr-pcb/#_track_segment),
consulted 2026-10-17: start and end contain two coordinates, and width contains
one value. The adapter's zero-width empty behavior is its existing modeling
contract, not a claim that every authoring tool accepts such an object.

The full properties, distribution, pre-push/CI, coverage, provenance and
historical artifact audit remain open. The rejected 35-file native integration
stays archived and excluded; do not restore it wholesale or weaken its gates.
