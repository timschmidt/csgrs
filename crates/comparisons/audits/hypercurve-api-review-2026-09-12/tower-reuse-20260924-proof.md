# Intrinsic quadratic-tower proof reuse — 2026-09-24

## Measured problem

The unchanged selected-fiber transverse mapped-cut regression passes its original and reversed chamfer inversions after the common-biquadratic-basis fix, then declines the similarity-transformed chamfer. The bounded diagnostic `similarity-point-20260923-probe5-case-0.log` reports two failed scalar parses with zero of 2,048 nodes remaining and memo lengths 2,028 and 2,036. No missing alternate witness, third independent square class, nested-radical mismatch, or rational-size rejection was reported. The provisional similarity point-image branches had no effect and are absent from this candidate.

## Candidate

The existing lazy synchronized approximation cache also retains one successful quadratic tower at a queried scalar root. The private cache type is renamed directly, with every production/test caller migrated. The node layout stays unchanged; approximation-only cache cells gain one optional pointer. Retained forms contain only rational coefficients, never expression-node references. Parsing checks retained forms within the unchanged 2,048-node budget; the 8,192-bit rational limit is unchanged. Only successful sign proofs are published, together with the existing intrinsic exact-sign fact. Failed attempts install no failure marker. Real sign queries factor the outer rational sign and query the shared payload, so the proof does not die with a temporary scaled expression.

The new wide-DAG regression first exceeds the fixed budget, then proves its 1,024 leaves and successfully replays the same root. It requires exact cancellation against an independently constructed sum, proof sharing across clones, no approximation cache, and rejection of a third independent radical. Further regressions exercise scaled Real proof retention and simultaneous approximation/tower publication. The existing size bound, serialization, rounding, and concurrent-cache regressions remain unchanged apart from the private type rename.

## Attempts and current validation

- v1: compile-only failure; two production references and five test references to the renamed private cache remained. No tests ran. The immutable snapshot and failure log are retained.
- v2: all 831 Hyperreal library tests pass, including 13 focused tower tests. Both all-target feature configurations are warning-free. Hypercurve all-target feature configurations also pass. Its release test executable is still being built at the time of this initial record.

This is a candidate record, not a completed closure or performance claim. Follow-up results will be appended only after exact process handles are reaped. Main production and all snapshot source bytes remain frozen during owned builds and tests.

## v2 geometry outcome

The fresh release executable (`7705297542dd9a23a59ace39dcdf5faae57e909a9e7faf6eec2ef95213b166a8`) passes five neighboring cases but the unchanged selected transverse regression still fails on the transformed chamfer with STRICT RealSign (4.04s reported by libtest). Retention only at queried sign roots is insufficient. All v2 processes were reaped. No claim is made that this candidate fixes the geometry composition.

The next candidate retains completed reductions of shared subexpressions during parsing, with an explicit bound on every retained rational coefficient and radicand. Such a normal form proves the value of that subexpression even if the enclosing parse later fails; no sign or success is asserted for that parent.

## Shared-form refinement

v3 passes all 831 scalar tests and both feature configurations. Review found that returning cache hits before local memo insertion could charge repeated references to one node more than once. Before testing geometry, v4 restores the existing per-call memoization path and avoids retaining cheap rational/constant leaves or re-publishing cache hits. A new 2,048-reference regression checks the fixed unique-node budget and exact cancellation. All 832 v4 scalar tests, including 14 focused tests, pass; both all-target feature configurations are warning-free. v4 geometry qualification is running.

## v4 geometry outcome and diagnostic follow-up

The v4 executable (`7b66b5dc088beb4b25b881dc2e06ab965b25fca37c42f82ba4c75cd021ae980a`) again passes five neighboring cases; the selected transformed-chamfer test remains a STRICT RealSign nonpass (3.94s reported by libtest). All v4 owned processes are terminal/reaped. A separate immutable diagnostic snapshot counts local memo hits, retained-form hits, and completed shared/unshared subexpressions, and reports existing algebraic/budget rejection reasons. It enables at the transformed-chamfer call only and emits at most 128 short lines. It changes no geometry operation or assertion. Diagnostic functions and counters are absent from main production.

## Root cause after shared-form reuse

The diagnostic confirms that reuse works, but most newly completed nodes in the two failing replays are unshared. The first failure exhausts the visit limit after 1,172 local memo hits, six retained-form hits, 284 completed shared and 1,543 completed unshared expressions. The second exhausts it after 1,015 local memo hits, 13 retained hits, 131 shared and 1,431 unshared completions. No algebraic-dimension or rational-size rejection was logged. The diagnostic executable SHA is `71ed5290f3bf0ba02929ee97ec9f373c1dcab1428eb771600461badaa541e0e4`; all processes are reaped.

## v5 algorithm

Replace the recursive parser and linear memo search with an iterative postorder traversal and hash-indexed memo of successful exact reductions. Each distinct reachable node is reduced once; unsupported operators stop the traversal. The artificial 2,048-visit cutoff is removed rather than raised. Traversal stack and local storage scale with the input DAG, and the existing field-dimension/coefficient guards still govern arithmetic. Shared intrinsic forms and exact queried signs remain reusable across later operations. No public API or new scalar expression variant is introduced.

The power-of-two factor now checks the rational-size guard before allocating its integer, covering even a one-node expression with an extreme shift. v5 passes 833 scalar tests, including 15 focused tests and cold wide, deep, exponentially shared, concurrent, extreme-shift, and independent exact-cancellation cases. Both scalar all-target feature configurations are warning-free. Geometry qualification is running. The earlier statement that the node budget is unchanged applies to v1–v4 only; v5 replaces that algorithmic restriction with iterative DAG traversal.

## v5 integration success and shift-guard review

The unchanged selected transverse/chamfer regression now passes completely, including the similarity-transformed chamfer under both policies (4.50s libtest time). All five neighboring cases pass. The executable SHA is `650d8d250dc7c3aed8d2102de205b6f72500e9cbd48fec8378c700dbd17bc6f0`; source bindings and both warning-free feature checks accompany the terminal report. All v5 owned processes are reaped.

Before broad qualification, review identified a representational restriction in the new shift guard: a shift larger than 8,192 can cancel existing numerator/denominator powers and leave coefficients inside the intended bound. v6 replaces factor construction with exact coefficient-wise dyadic scaling. It cancels powers of two, checks resulting numerator/denominator sizes, then allocates only those bounded integers. Zero coefficients require no shift allocation. Regression coverage adds both directions of a 16,000-bit shift that leaves an 8,001-bit coefficient, exact cancellation against independent expected values, and zero at extreme shifts. The v5 simple absolute-shift guard is not the proposed final implementation.

## Final qualification and commit

Committed Hyperreal `28eaac35e8c24006c532b25a393b7d430d03823c`. All 833 Hyperreal, 252 Hyperlimit and 507 Hypersolve library tests pass. Both all-target feature configurations are warning-free in Hyperreal, Hyperlimit, Hypersolve and Hypercurve. The complete 1,226-case Hypercurve library inventory was attempted: 1,205 passed, six ignored, and 15 existing nonpasses remain (11 timeouts at 75 seconds and four failing assertions). No prior passing case regressed. The selected-circle/chord case uses its documented 180-second bound and passes in 151.04s; it had already passed before this batch under a larger focused bound and is not a new speedup.

The complete transformed selected-chamfer round trip passes under both policies in 4.71s of libtest time. The unchanged PH self-contact fillet passes in 3.62s in the sweep; a direct replay on the source-bound parent executable exceeds 75s. This establishes a computational improvement for that regression, not a general benchmark claim. The five focused neighbors also pass.

Every one of the 2,044 snapshot inputs matches main. Source, executable, staged and committed bytes are bound by `tower-reuse-20260924-{qualification,staged,post-commit}.json` and the v6 per-layer reports. All owned processes are terminal/reaped. Hyperreal, Hypercurve and Hypersolve are clean. Hypercurve production and tests were unchanged by this batch. There is no new public API, compatibility alias, geometry-specific fallback, or expression-node variant; the existing node-size bounds pass.

The full goal is not complete. The 15 remaining nonpasses are listed in the qualification record. Caller follow-up notes are in `normalized-corner-followup-20260924.md`; certificate authority/privacy debt remains separate.
