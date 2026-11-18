# Rational Horner working-interval requalification

## Reassessment under the bounded memory exception, 18 November 2026

The targeted inline candidate fits the proposed native memory allowance on
`79b183c51`. The previously adverse case constructs a tensor image, consumes
the resulting root in a polynomial evaluation, and returns both reports.

| Cold tensor stack on x86_64, Rust 1.99 release | Fixed program | Current branch | Targeted inline |
| --- | ---: | ---: | ---: |
| Through materialization of both reports | 11,784 B | 11,784 B | 11,800 B |
| Through independent validation and destruction | 12,424 B | 12,424 B | 12,440 B |

The 16 B increase in the complete call is 0.129%. Its incremental limit was
32 B and 1%, its cumulative limit was 64 B and 2%, and its total limit was
32 KiB. The peak leaves 2,080,616 B in the 2 MiB thread stack after the
4,096 B guard. Each of the eight fresh processes reproduced the cold increase.
The other 99 case and cache-state combinations stayed level or used less stack.
Fixed `c9f1f9ac5` and current controls matched on every paired observation.

Each implementation ran 12,800 measurements with the original stack probe and
12,800 with the complete-lifetime probe. All original returned reports matched.
The latter probe checked construction, the next operation where applicable,
exact result validation and destruction on the measured thread. Caller-owned
inputs were prepared before measurement. Allocation counts, deallocations,
reallocations, total requested bytes, peak requested live bytes and final live
bytes matched on every paired observation. The cold tensor case peaked at
3,760 requested live bytes and retained up to 1,424 B after report destruction
in both controls and the inline candidate. These figures cover the single-thread
Rust allocator measurement. Deployment RSS, allocator metadata and additional
concurrent callers require separate measurements.

The alternative from `80337035e` uses the compiler's ordinary inline hint for
the same two additions. Rebuilding it on the current branch reproduced the
80 B increase on the cold tensor case. Complete-lifetime stack reached
12,504 B. That exceeds both the 32 B incremental cap and the 64 B cumulative
cap. Every returned report and requested allocation measurement still matched.
The two alternatives therefore have different memory dispositions under the
same allowance.

On the candidate, six native solver differential tests and eight rational
arithmetic and serialization tests passed. A further comparison exercised the
**actual candidate production routines** and the frozen routines in both
native x86_64 and WASI Preview 1 processes. All 1,404 polynomial-evaluation
and 2,808 zero-exclusion windows matched, including ordered results and
rational reducer counters. Both exact endpoint and exclusion tests passed in
the guest.

Two independent timing campaigns exercised the original nine completed solver
workloads. Each used eight ABBA/BAAB blocks, 40 Criterion samples per case per
invocation and a paired block bootstrap. The second reversed the variant order
and rotated core assignments. The exact rational point took 0.990× control
(95% interval 0.974–1.005) and 0.987× (0.968–1.007). No workload had a lower
95% bound above one in either campaign. The exact normal constant was faster
in both, by 1.4% and 1.2%. The archive preserves all nine results
individually.

The Rust 1.99 default-feature certification executable grew by 7,864 B in the
file, 6,816 B in allocated executable sections and 6,855 B in deduplicated
named code against the current branch. These increments fit that
configuration's prospective screen. Against the fixed build the differences
were 9,288 B, 7,024 B and 6,971 B respectively.

The program's [existing release size accounts](interval-size-accounts.json)
also cover Rust 1.95.0 with all features. The matched builds produced 42
accounted executables per revision. Thirty-three kept their recorded package
feature lists. Nine lattice executables had acquired a WASI feature since the
fixed baseline and were excluded from the like-for-like budget comparison.
Four Hypersolve artifacts exceeded their incremental caps.

| Rust 1.95.0 release artifact | File increase | `.text` increase | Named code increase |
| --- | ---: | ---: | ---: |
| Certification benchmark | 38,880 B | 30,864 B | 30,583 B |
| Dispatch trace benchmark | 16,000 B | 15,648 B | 15,498 B |
| Retained fuzz benchmark | 28,072 B | 23,312 B | 23,047 B |
| Solver unit tests | 8,592 B | 8,560 B | 8,489 B |
| Existing incremental cap | 4,096 B and 1% | 4,096 B and 1% | 1,024 B and 0.5% |

Only the unit test executable contains the two added production-dispatch tests.
The certification benchmark alone exceeds even its 16,384 B cumulative file
allowance with this increment. The migration remains **rejected on release
binary size**. No new memory or binary allowance was charged. Downstream
`successive_operations`, final Interface and firmware qualification were not
run after these failures.

The evidence record is `docs/evidence/2026-11-18-interval-horner-bounded-readmission`
in csgrs-evidence commit `3051ef201beb2eb2f97230de13e85804a857f2cc`.
It contains the two candidate patches, harnesses, all 102,400 memory probe
rows, all 576 Criterion sample files, release artifact measurements, complete
WASI production traces and replay commands. [Record digest](../evidence-archive.tsv)

## Original assessment, 16 November 2026

The production migration was rejected under the zero-growth stack gate in
force at the time. The proposed binary-size caps permitted the measured
growth. The production sources, dependency graph and size accounts retained
their preceding state.

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
