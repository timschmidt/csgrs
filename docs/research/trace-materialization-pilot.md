# Trace materialization pilot

Two isolated collector alternatives preserve complete outputs but fail the local
cost screen. Neither implementation is retained. The independent regression
for ordered, content-keyed snapshots and clearing `take()` behavior is retained;
production trace code is unchanged.

This P4/P10/P36 investigation follows the
[cancellation qualification](solver-cancellation-qualification.md). Its remaining
wrapper-to-scope failures all use active trace recording. The owning function is
Hyperreal's private `collect_dispatch_counts`; the recording hot path, predicate
order, public API and rational counters remain unchanged in both alternatives.
The baseline is scope candidate `ab81708aa` plus the new test at
`4f41bd58415d936968e1955dbd431f0cb5e988f0`.

The new test independently specifies exact ordered rows and counts, merges equal
text held at distinct static addresses, checks repeated nondestructive snapshots,
checks that `take()` clears counters, and verifies that an earlier report stays
valid after recording resumes. The existing duplicate-label and concurrent-scope
regressions remain intact.

## Measurements and disposition

The same native Rust 1.95 consumer runs 276 configurations per source: identity
counts 0, 1, 2, 12, 64, 65, 256 and 4,096; one, seventeen, half as many or all
unique textual groups where applicable; snapshot/take; cold/eight warm calls;
and three fresh repetitions. Every output is checked against independent integer
count sums and exact expected labels/order. All 828 controls pass, with 736 full
output comparisons over 92 domains. Nine owner tests pass on each source, along
with strict all-target/all-feature owner Clippy and consumer Clippy.

| Alternative | Implementation | Failed accounts / 92 | Disposition |
| --- | --- | ---: | --- |
| Batch, `4e405deede718963aa7d846592b32c76f128926d` | Collect identity entries, sort by text, merge, shrink | 68 | Rejected: alias-heavy temporary growth and added reallocations |
| Incremental, `c80583b1070dae0356ec7bda236fa6bbcacab4bb` | Insert/merge into an ordered vector, then shrink | 88 | Rejected: traffic growth and added reallocations |

The screen uses unchanged diagnostic limits: peak/traffic 4,096 bytes AND 2%,
retained bytes 512 AND 2%, allocation/reallocation counts 16 AND 1%. Zero
baselines permit no positive growth under this screen. These are local diagnostic
comparisons, not replacements for fixed-program cumulative accounts or evidence
that either implementation meets the whole-consumer contract.

Batch sorting reduces requested peak by up to 359,912 bytes for other inputs,
but 4,096 identity-distinct labels with one textual value increase requested
peak from 688 to 229,376 bytes and traffic from 688 to 229,432 bytes. The favorable
unique-label cases cannot erase this supported-input regression. Byte metrics
alone fail in 56 accounts; 68 have a new reallocation.

Incremental aggregation never increases observed peak or retained output storage.
It nonetheless increases requested traffic from 9,424 to 17,752 bytes for 65
unique labels and introduces up to ten reallocations where the baseline has none.
Byte metrics fail in 24 accounts; 88 have additional reallocations. No allowance
is enlarged to retain it.

All output storage is released by cleanup. Requested/usable retained output
sizes agree across sources. The original collector varies requested/usable
peak, traffic and allocation/deallocation counts across repetitions in 22 domains;
neither alternative varies these observed metrics in this finite corpus. This
is a reproducibility observation, not a universal resource bound or runtime win.

## Scope and execution limits

The allocator monitor is copied unchanged from the complete cancellation
consumer. Static-label input preparation, counter recording, collector
materialization and cleanup have separate observations. The harness deliberately
creates a bounded set of process-lifetime static labels to exercise the public
`&'static str` contract; their storage is explicit and is not claimed released.
Collector measurements exclude input creation, validation and counter-table
retention. This pilot does not establish whole-call stack, process RSS, full
geometry lifetime, runtime, distribution or final shipped-image admission.

All 868 direct children are counted at one spawn site; compiler descendants are
explicitly outside that count. Execution completes in about 366 seconds within
the original 3,600-second window and 2,000-child ceiling. Owned process cleanup,
per-file limits, staging reservations and the development reserve remain binding.
The source worktree is restored to the tested control commit. No queue, previous
source image, historical charge or earlier fixture changes.

The first independent verifier incorrectly required every command name to be
unique. Two counted `git rev-parse HEAD` reads used the same log name. Their
arguments, output lengths and raw/stored hashes are identical, so both records
are fully backed by the preserved bytes. The corrected verifier permits only
this proven-identical reuse, checks both records and all other log hashes, and
finishes within the original 300-second offline window. The failed verifier,
log, correction and original deadline remain preserved. Geometry is not rerun.

## Next investigation

The batch regression arises because the live table distinguishes addresses while
the public report merges textual labels. A content-keyed recording table could
avoid that temporary expansion and the second aggregation structure. Such a
change affects event-recording cost and must be reviewed and measured there;
these collector-only results cannot qualify it. The historical constant-keying
change `ea649ddb9` is inspected as design context, not treated as current runtime
evidence. The governing order and all other gates remain intact.

Accepted totals remain 57 size accounts and zero memory exceptions. Neither
prototype is merged, and the full fifteen-phase goal remains active. No push.

Frozen evidence: `docs/evidence/2026-11-17-interval-trace-materialization`,
archive commit `3cc14e7745bec36f5c6a0eebbf17587600aea49e`; 917 files / 4,824,326 bytes,
SHA-256 `e1c880a1930c61cdd0c3a8ddd4b7d62a6a68020ef9d1740a70f1c3caf66b7800`.
All source, stored and committed hashes are verified, including three executable
images and the primary-worktree checks for the retained test. Those nine owner
tests, strict all-target/all-feature Clippy and workspace formatting pass after
the final source edit. The production prefix before the test module is verified
byte-identical to its prior primary revision. Earlier archive paths are unchanged.
