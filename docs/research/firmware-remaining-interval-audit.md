# Remaining firmware interval inventory

This Phase 0/9 audit replaces the provisional classifications of **23 existing
firmware inventory entries** at firmware
`7e0d5a43d01706fe619234d8b174beb1ebe45dc6`. Fourteen are test declarations;
nine are production or host-analysis declarations. The other 1,509 inventory
rows, including the completed clock/FOC reviews, remain unchanged.

All 70 entries in the existing firmware subset now have reviewed dispositions.
This does not establish an exhaustive inventory of firmware, complete generic
arithmetic migration, or phase completion. In particular, earlier rejected FOC
and servo changes remain rejected. No production implementation changes here.

## Reviewed owners

| Declarations | Actual contract | Disposition |
| --- | --- | --- |
| Capability `read_verified_range`, `RangeSink` | Half-open byte windows over one canonical, digest-verified document; exact-end empty read is complete | Retain byte-stream ownership and receipts; these are not closed numerical sets |
| Diagnostic `validate_retained_interval` | Actual capture duration is bounded by the request; the truncation flag exactly describes a shorter capture | Retain evidence completeness check |
| Transport `validate_chunk_range` | Nonempty bounded waveform chunks, checked offset/length arithmetic and exact record extent | Retain byte transport validation and typed errors |
| Machine-IR `finite_difference_first_bounds` | Exact attained extrema of a discrete recurrence over a nonzero update count | Retain specialized recurrence proof and selection; generic interval evaluation cannot replace it |
| Runtime `queue_slot_range` | Half-open arena span with checked layout arithmetic; slice access rechecks the actual arena | Retain graph memory ownership and shape errors |
| Target `StackBounds` | Aligned linker/HAL stack extent, protected reserve and same-core access authority | Retain address provenance, live-stack and interrupt guards; no geometric interval substitution |
| Graph VCD `Range` | Extrema of strictly positive pulse widths and periods | Retain the positive-duration contract; do not merge with the PCM accumulator by spelling alone |
| PCM VCD `Range` | Optional extrema preserve observed zero separately from no observation | Retain explicit accumulation state and existing failure-report semantics |

The graph accumulator's zero initial minimum is not a reproduced zero-sample
bug: zero-width pulses fail before observation. Ordered events and a completed
positive pulse imply a positive interval between successive rises. The PCM
accumulator has a different domain and uses `Option<u64>` endpoints. Its report
boundary deliberately emits zero for absent measurements in supported failure
marker reports. Neither encoding is imported into the authoritative interval
kernel. Potential generic reuse still needs semantic and cost admission.

The fourteen tests remain witnesses in their existing owners, rather than being
misclassified as interval implementations. Their declaration, purpose and source
identity are recorded. This audit does not claim that all fourteen were rerun.
The exact executed scope is below.

## Independent discrete-extrema qualification

The public machine-IR validator rejects zero update counts before calling the
private bounds helper. For admitted counts, the first-difference sequence has
an affine second difference. Its extrema occur at an endpoint or beside the
second-difference sign transition. The existing implementation locates that
transition by bounded binary search and checks its neighboring discrete values;
it does not substitute a continuous polynomial extremum.

A qualification-only patch calls the original private helper while independently
stepping the machine recurrence to form expected minima and maxima. It exercises
**21,952 sequences / 362,208 values per compiler**, covering counts 1–32,
positive/negative/zero curvature, endpoint and interior extrema, and small and
full-width signed coefficients. The oracle does not call the production
closed-form evaluator, sign-transition test or binary search for its expectations.

On Rust 1.88 and 1.95, the oracle and all 23 original machine-IR library tests
pass. All 30 xtask tests pass on Rust 1.88, as does strict all-target Clippy for
machine-IR and xtask. The temporary patch is retained in the evidence record
and the isolated checkout is restored clean. It is **not a retained CI regression
test**; replayable qualification evidence does not claim continuous CI coverage.
The finite corpus does not prove arbitrary-input completeness or replace review
of the discrete-extrema argument.

No benchmark, allocation, MCU-cycle, whole-call stack or HIL measurement is
claimed. The inspected target carriers use primitive storage, but that source
observation is not a runtime memory bound. All 54 size accounts remain unchanged.
Current image/flash failures are tracked in the
[final-image qualification](firmware-current-image-qualification.md).

## Evidence and preservation

Current declaration locators and source hashes are validated against the pinned
firmware. The inventory verifier checks every row, verifies the fourteen test
attributes, and proves that only the 23 selected entries changed. The initial
mechanical checker incorrectly rejected descriptive wording containing
“requires”; its failure is retained. The corrected check recognizes the actual
unresolved placeholder values and does not relax an implementation assertion.

Evidence: `docs/evidence/2026-11-16-interval-firmware-remaining-inventory` in
csgrs-evidence, commit `5dc617121db7f0369ccb107610e1a9dc77ba86e7`: 56 files,
2,319,703 bytes, SHA-256
`73777cc34d4f5a1a0e4f1daec98d3e3e9c83153ed0c7dc10aefdb84353b51f34`.
The record includes source snapshots/manifests, both inventory versions through
the original table and reviewed rows, the qualification patch, exact commands,
logs and reproduction instructions. Earlier archive records are unchanged.
No push, publication, firmware image change or phase completion is claimed.
