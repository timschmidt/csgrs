# Finish-borrow complete consumer resources

This qualification evaluates the previously unmeasured lifetime, stack and final
native consumer costs of candidate `1abfbc3fc80c52d7d57be26ac031ef824ca92d9d`.
It follows the [functional/runtime review](cancellation-finish-borrow.md), under
Hypersolve work-budget ownership, P5/P6/P20/P36 and plan 14.4. The candidate
remains isolated; instrumented observations do not establish runtime performance.

## Sources and completed contract

Four boundaries retain the fixed program baseline `c9f1f9ac`, primary `e63cbf68`,
original publication implementation `8a454c860` and revised finish-borrow
implementation `1abfbc3fc`. Current primary `d8d58bdd7` differs from `e63cbf68`
only in documentation. The original publication implementation is an explicit
accounting boundary; its positive costs are not refunded by later reductions.

The campaign reuses 30 hash-verified final images for the first three boundaries
and builds twelve revised-candidate images against the exact original consumer
path, sources and dependency lock. Copies receive fresh section/symbol/image
measurements and fresh controls and resource observations. All fixtures,
operation code, assertions, stack probes and lifetime observers remain unchanged.
Four strict consumer Clippy configurations and formatting pass.

The complete four-case consumer covers 1/4/16 variables and separate 64-variable
stress, both policies, default/trace configurations, passive/active recording,
reused/rebuilt input, cold/warm operation and cleanup, three heap repetitions,
both stack fill patterns and three 32-cycle histories for each input mode.
It materializes full outputs, receipts and traces and checks exact expectations.
The older repair-dependent cases 4–6 retain their separate fixtures and obligations.

Primary's six cancellation/retry smoke attempts reproduce the publication
assertion failure. Both publication implementations complete that workload.
Their retry outputs must match ordinary outputs; their two-operation trace is
a separate domain. Failed primary attempts are not successful cost baselines.

## Accounting and scope

Raw accounts use the unchanged original numerical caps. Historical reconciliation
retains prior values, denominators, positive charges and verdicts on the primary
and original-publication path. The existing target-specific byte proposal is
then evaluated against the newly reconciled accounts, separately from the raw
and historical results. It does not automatically transfer an earlier verdict
to changed source. No other numerical gate is relaxed.

Added retry retains its absolute capacity envelope and no pre-publication primary
percentage denominator. The two correct implementations also have a separately
identified growth comparison using actual reference measurements and the ordinary
metric caps. That comparison does not invent a successful earlier primary outcome.

Requested and allocator-usable payload are distinct. The original guarded-stack
boundary, exclusions, fill patterns, worker reserves, persistent-state limit and
no-growth-after-cycle-two checks remain. These are finite single-worker native
consumer measurements, not RSS, firmware, arbitrary-input or concurrency bounds.
Final linked consumer images include their measurement apparatus; shipped
distribution and platform qualification remain separate obligations.

## Results and disposition

Collection completes 4,938 observations: 4,932 successes and six expected primary
failures. All 3,876 full output/trace and 4,404 output comparisons pass. The 888
complete histories cover 28,416 cycles. Independent audit verifies observation
coverage, source/image/log hashes, final-image measurements and exact outputs.

There are 288 resource accounts. Fourteen fresh accounts fail the initial growth
allowances. Historical reconciliation raises that to 25. Applying the existing
byte proposal leaves 15 failures: thirteen common active-trace accounts and two
added-retry comparisons between the correct implementations. Cumulative
allocation/free counts, peak/traffic bytes and cleanup stack remain outside
declared limits. This is a new source/accounting path; the earlier proposal's
one-failure verdict does not transfer to it.

For example, the reused-input, cold, 64-variable APPROXIMATE_512 active-trace
cleanup-stack account measures 1,712 bytes on fixed/primary/reference and 1,752
on the revised candidate. A preserved earlier 40-byte charge plus the new
40-byte charge totals 80 bytes. That exceeds 4% of 1,712 bytes (68.48 bytes),
even though each increment and the 256-byte absolute allowance pass. The prior
charge remains despite the current primary/reference observations being equal.

All absolute capacities, required headrooms and lifetime-retention checks pass.
The twelve final-image accounts have no failed growth or headroom checks.
Maximum observed requested/usable peaks are 577,406 / 613,232 bytes; persistent
footprints reach 12,696 / 12,888 bytes. Operation and cleanup stack reach
18,800 / 1,768 bytes. These measurements do not establish an arbitrary-input bound.

Historical review verifies 3,288 corresponding full output/trace pairs. It
retains 3,264 metric records with 744 positive historical entries, including
aggregate floors, and their original values, denominators and verdicts. Those
entries are not 744 independent measurements. The proposal changes 72 byte-metric
caps using the already declared numbers; an independent fraction-arithmetic
verifier checks those verdicts and the unchanged gates. No new cap is introduced.

The producer uses 5,219 direct children and about 1,186 seconds within its
6,500-child / 5,400-second limits. The three sequential audits share one
300-second window and pass in about 73 seconds. No retry, deadline reset or
unchanged measurement rerun follows. Source control is restored clean; the
producer and audits are terminal, workflow queue held and original watcher
terminal.

The revised candidate remains isolated. Its resource and full runtime gates
remain open, alongside shipped distribution and platform qualification. The
older solver bundle and rejected trace alternatives retain their separate failed
obligations. Primary production code, accepted totals of 57 size / zero memory
exceptions and the 1,644-row inventory are unchanged. No push; the full
fifteen-phase goal remains active.

Frozen evidence: `docs/evidence/2026-11-17-interval-cancellation-finish-consumer`,
archive commit `11a20d85962cf35c577cad25b61e0b9d367941c1`; 3 files / 32,476,138 bytes,
SHA-256 `e37499a3986a178c9c6c310691cc7e8a9d53f575c8ceaff0146241ec4338ee91`.
The lossless payload contains 5,313 members / 323,661,870 raw bytes and
42 validated images. Original, decoded and committed hashes are verified;
earlier archive paths are unchanged.
