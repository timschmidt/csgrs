# Exact KiCad straight-track ingress after the single-line kernel admission

Target revision: `a843bf79dfe329fb51ababaaf803752bb74a0f64`.
Parent admission: `docs/evidence/2026-10-17-native-stroke-completion`.

The owning layers are KiCad source admission and the shared native curve
construction. P1, P4, P6, P8–P9, P12–P20, P25–P27 and P35–P39 require exact
source values, round-cap fidelity, reusable certified output, visible failure,
and preservation of the declared resource contract. Reproduce the archived
26 regressions and all 39 current source-failure regressions on this revision
before modifying production code. No credit follows a failed assertion.

The input domain is the existing exact decimal/scientific KiCad `segment`
syntax: validated start/end pairs and nonnegative width. Zero width denotes
regularized empty geometry only after validating all required fields. Positive
coincident endpoints retain the exact disk already admitted. Positive distinct
endpoints must denote a round-capped exact segment stroke. Values must not be
rounded to binary64 before subtraction, normalization or width halving.

Preserve malformed-source categories, duplicate/arity validation, diagnostics,
segment indices, zero-width semantics and shared-budget denial/recovery. A
failed segment cannot publish its valid prefix as a complete board. Kernel
failure must remain a typed error; no empty or origin fallback is admissible.
Reuse native path/stroke construction and existing strict policy machinery.
Do not restore the archived failed-region source adapter.

This intentionally migrates the current finite rectangular approximation to
the intended round-capped source set, including previously unsupported exact
coordinate ranges. Preserve the original failure fixture and explicitly
replace its unsupported-range expectation with independently proved successful
geometry. Do not alter malformed-source or resource-exhaustion expectations.
Expected report changes require independently checked source/geometry evidence
and an explicit readiness/artifact migration, not snapshot-update flags.

Exercise exact analytic area, endpoint-cap membership, bounds, reversal, source
metadata, native handoff, reconstruction/aggregation, touching caps and actual
short checks. Compare strict→approximate→strict contexts without permitting an
approximate ambient policy to replace a strict proof. The finite `line_polygon`
adapter is a separate compatibility boundary; retain its unmodified failures
as an open part of the full objective, without claiming this ingress change
fixes it.

Keep all original full-resolution projects and every resource assertion.
The full-board worker CPU limit remains 120 seconds and copper-neck check-body
wall limit remains 60 seconds. Run those gates early. If exact ingestion cannot
complete them, investigate the owning bottleneck; do not reduce fixtures or
publish an easier input domain as completion. Requalify affected native and
Hypercircuit consumers, documentation, lint and final output contracts.

Retain each executed source, helper, runner, build target and raw outcome;
corrections use new versions. Completed Cargo targets remain read-only.
External elapsed/address-space containment is not an internal work, memory or
cancellation guarantee. Measure comparable completed work separately from
changed-geometry or newly completed workloads. Only the `current_work` JSON
value in the status ledger may change. Historical artifact loss remains
explicitly incomplete; this change does not retroactively establish recovery.
