# Rational Horner working-interval requalification

The Phase 7 production migration remains **rejected**. Current qualification
reproduces a downstream stack regression even though the proposed binary-size
caps permit the measured growth. The bounded size exception does not waive the
memory gate. No production source, dependency change, or size account is retained.

Hyperinterval owns rational product hulls and scalar translation. Hypersolve owns
Horner traversal, root selection, exclusion proofs and reports. P1, P4, P6, P10,
P36 and P37 apply. The experiment delegates three rational Horner adapters through
validated nonempty working intervals, preserving coefficient and predicate order.
The derivative path still multiplies before extracting the next coefficient.
Empty-capable intervals retain their semantics; selected tensor output remains
input to a subsequent exact evaluation.

The matched control is CSGRS `6e4e68f348fa6493f7164336f5ec8768aff71be3`.
Reviewed owner/dependency source, manifests, lock, build configuration and normal
benchmark paths agree with the fixed program control
`c9f1f9ac55216b762e9eb97672de1e66f3b8a3fb`. This is a current qualification,
not a replacement for the unfavorable historical Horner timing. Rust 1.99.0
and normal native release flags are recorded with the retained binaries.

Three unaccepted snapshots remain available:

| Snapshot | Disposition |
| --- | --- |
| `7f784f9e1f1e5d4b56a6e716a1a05767707a505a` | Borrowed translation increases the emitted evaluator frame by 16 bytes; timing held. |
| `80337035e8bb43f4293aa97e5cd547aec13c911f` | Consuming translation reduces that evaluator frame, but a completed cold tensor chain grows 80 stack bytes. |
| `0f604c194108d244c9cd709e06b3c8ff80b07ba3` | Inlining the two-add translation adapter removes 64 of those bytes; the same tensor chain still grows 16 bytes. |

Each consuming variant passes six migration/production differential tests over
the 702-case corpus, eight rational/serialization tests, affected strict Clippy
and formatting. Fresh-process comparisons preserve complete dispatch vectors,
rational counters and outputs, including cold/repeated operations. Thirty
serialized public reports and all 36 steady-state allocation rows match. These
checks do not establish universal equivalence or cold allocation bounds.

The unchanged public-consumer stack harness covers 50 cases under cold and warm
thread-cache conditions: polynomial evaluation, root-presence proofs, and tensor
construction followed by evaluation. Independent endpoint, status, sign and
provenance expectations are checked outside the measured callback. Each variant
has 12,800 observations against an equally sampled control: 128 per case/cache
state, two watermark patterns, eight fresh processes. All complete outcomes
match. The first cold narrow tensor chain reaches 11,784 bytes in the control,
11,864 in the consuming candidate and 11,800 after targeted inlining, in all
eight processes. The other 99 case/cache-state maxima do not grow. Cross-case
savings do not offset this persistent failure.

Diagnostic stale-stack captures and emitted call frames identify a 64-byte
translation-helper frame beneath a caller that grows by 16 bytes. The targeted
inline experiment confirms removal of the 64-byte component. Stale stack words
are not a live backtrace; the original uninstrumented campaigns decide admission.
An attempted inline campaign accidentally reused the diagnostic C archive from
a shared Cargo output directory and aborted. Its binary and raw rows remain
excluded from acceptance. Cleaning only the harness artifacts restored the
original C monitor; the corrected campaign ran within the remaining budget.
Earlier lock-resolution and compiler failures are also preserved.

| Final native certification artifact | Consuming delta | Inline delta | Proposed cap |
| --- | ---: | ---: | ---: |
| File | +9,112 B | +9,248 B | 16,384 B and 1% |
| Complete emitted code | +7,600 B | +7,920 B | 8,192 B and 1% |
| Address-deduplicated named code | +7,429 B | +7,750 B | 8,192 B and 1% |

These are prospective screens, not final consumer or cumulative size admission.
All 54 existing accounts retain their charges. No comparative runtime samples
were taken. Broader owner/downstream workflows, final consumer artifacts and
firmware headroom are not qualified by this experiment. Both isolated worktrees
are restored clean; source snapshots, original adverse data and all test sources
remain archived. A future repair must retain the cold tensor witness and supply
a concrete representation or lifetime justification before further tuning.

Evidence: `docs/evidence/2026-11-16-interval-horner-requalification` in
csgrs-evidence, commit `4d0e306d02fbafdd0e9a52e4a8c8274b3ff7aa71`: 380 files,
195,329,447 bytes, SHA-256
`54113bb2b11dc6d0488873a3f196390736d2a0cfcafd78a78a00d7755f7a559a`.
This frozen subrecord leaves coordinated Stage 92 and all fifteen phases open.
No push or full release qualification is claimed.
