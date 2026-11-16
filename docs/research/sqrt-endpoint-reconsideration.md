# One-sided square-root extraction under bounded size admission

**Not retained.** The Phase 7 candidate was reproduced on CSGRS
`c17761095a4b24b055825383b780f2ae93dae0bc`, with Interface
`32a59de0d69436427e5b97297c4929fd908bf326`. The earlier
[72-byte size rejection](interval-real-adapter-audit.md#one-sided-square-root-relocation-rejected-on-2026-11-14)
predated the bounded-growth allowance. Current artifact screening permits
further evaluation, but repeated runtime regressions prevent retention.

Hyperinterval owns the generic rational/grid arithmetic; Hypersolve retains
strict endpoint comparisons, radicand enclosure, clipping, symbolic fallback,
reports and policy. Properties P1/P4/P6/P10/P17–P20/P35–P39. The missing shared
API was reproduced before production edits. The relocated body keeps its eager
sign, integer division/remainder, root, square-exactness and materialization
sequence for nonnegative rational inputs and nonpositive binary grid exponents.
Its typed owner errors preserve the solver's existing `None`/fallback behavior.
The existing two-sided budgeted square root has a different operation sequence
and resource contract and remains unchanged. No total-work, heap or cancellation
bound is added to the legacy one-sided path.

The proposed owner tests cover validation order, zero and exact grid squares,
and 3,315 rational/grid cases using shared independent GMP squared inequalities
and directed MPFR checks. Exact grid round trips prevent outward conversion from
hiding an off-grid result; generated results are consumed by interval squaring.
The complete Hyperinterval/Hypersolve run reports 1,770 passing completions in
109 result lines, including nested subprocess fixtures. All 199 Interface core
tests pass. Strict owner Clippy and workspace formatting pass.

Twelve public square-root cases preserve serialized endpoints, subsequent
squaring, both-policy comparison receipts, complete scalar/rational dispatch
traces and allocation/live/peak records. Cases include rational and symbolic
inputs, clipping, absent real domain, exact squares, positive-grid fallback,
no requested grid, 512-bit grids and large coefficients. Native/WASM Interface
budget transcripts and replay also match, with no WASM host calls. Lockfile
updates add the intended dependency edges without changing package versions.

Both native square-root consumer binaries shrink by 24 bytes; their complete
`.text` sizes are unchanged. Thirty of 57 owner artifacts are byte-identical,
and none exceeds the initial file allowance. Interface's final native budget
binary adds 128 file bytes and 32 `.text` bytes; WASM file/code payload sizes
remain unchanged. This is the initial artifact screen, not completed cumulative
size admission. The square-root method and closure retain their measured local
stack reservations; whole-call peak stack remains unqualified.

Two independent runs of the same binaries each contain twelve explicit warmup
processes and eight alternating ABBA/BAAB blocks. Each block compares means of
two process medians per variant, using the same executable path, CPU 0, disabled
ASLR and no concurrent agent builds/tests. Across both runs, the fractional
rational case is slower in all sixteen blocks: 0.77–1.37% initially and
0.83–1.81% on repeat. The exact-square fractional case is also consistently
slower: 0.93–2.42%, then 1.94–3.69%. Large-coefficient bounds improve
1.07–1.40%, then 0.93–1.37%. Other cases include mixed and noisy results; all
samples remain in evidence. No universal significance or compiler-causality
claim is made. These repeated adverse completed workloads reject retention
under the unchanged runtime gate.

The two runs record 3,047,424 timed calls: 2,793,472 successful enclosures and
253,952 expected absent-real-domain results. Warmup/allocation-control calls are
excluded. An earlier attempt stopped at the binary-hash assertion before any
measured block: an `objcopy` inspection had rewritten copied artifacts. The
mutated copies and failed warmups are preserved. Both variants were rebuilt to
reproduce all four originally recorded hashes before timing was repeated in a
new directory. No measured adverse sample was removed.

Snapshots remain at `refs/interval-unaccepted/sqrt-endpoint-20261116`: CSGRS
`4878c0fcddc1b005b667aa8c7d7dd4d46851942b`, Interface
`6a8f9eb07dd93f0bcc9c7defdde73e76538c4ecc`. All seven candidate repository paths
are restored. No helper is retained without its intended solver consumer.
All 45 cumulative accounts and the fixed program baseline remain unchanged.
Full owner timing, successive_operations, whole-stack memory/stack, platform
and release checks were not completed after the runtime rejection. No cap or
performance exception, layout/inlining retry, push or publication is adopted.
The full fifteen-phase goal remains active.

Raw evidence: `docs/evidence/2026-11-16-interval-sqrt-endpoint-reconsideration`
in the evidence repository, including exact patches, commands, raw timing,
artifact hashes, failed inspection attempt and restoration receipts.

Final restoration reports 1,768 passing owner completions in 108 result lines
(including nested subprocess fixtures), plus all 199 Interface tests. Strict
owner/Interface Clippy and workspace formatting pass. All 61 measured artifacts
reproduce baseline hashes, with identical complete transcripts and replay.
All 88 previous frozen stages verify unchanged.


Review commit `e8a3d4e16595ac8b7e98c2bdc60be156ada2c309`. Evidence archive commit
`12d5c1d22714f9abca6dff94f848a80f66c35b1f`: 459 files / 10,966,221 bytes,
SHA-256 `c01b12f6f809ac391c3afea734ee16fd68ac98c2bf9dd4c6d4fe2886f11149dc`.
