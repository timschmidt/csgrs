# Certified divisor-zero-factor WASM qualification — checkpoint 72

Outcome: the isolated checkpoint-70 candidate preserves its native mathematical
results under actual WASM execution. No live source is changed or promoted.
The six retained continuation transfers and donor-reading totals are unchanged.
This checkpoint does not establish WASM performance or close the ecosystem audit.

## Mathematical and state coverage

The rational importer is byte-identical to the checkpoint-71 native importer.
The unchanged history bridge uses the same work implementation as the earlier
native history harness; this identity is checked before and after execution.
Every comparison includes the full report, not only a checksum, status or sampled
polynomial value. Input wire identities, signed coefficients, intervals, exact
witnesses, metadata and certification outcomes are preserved.

| Corpus | Instance lifetime | WASM observations, both variants |
| --- | --- | ---: |
| 8,346 authored rational cases, both policies | One persistent instance per variant | 33,384 |
| 114 cost cases, both policies/lifecycles, batches 1/8 | Fresh instance per observation | 1,824 |
| 48 nonrational cases, both policies, four histories, both lifecycles, batches 1/8 | Fresh instance per observation | 3,072 |

All 38,280 observations match independently qualified native records. The
16,692 paired rational queries retain exactly 184 new answers and 16,508 unchanged
reports: 152 previously Undecided results and 32 degree rejections become
Transformed. Repeated policies, scales and carriers are not independent defects.
Denominator guards and nonisolating-image results are unchanged.

The independent extended-field oracle performs 89,984 checks on the 3,072 history
observations. The checkpoint-70 rational/signed-polynomial checker is also rerun;
its 128,652 checks are prior-corpus revalidation, not newly read donor lines.
An independent streaming pass replays every saved observation in its prescribed
order, checks persistent-instance sequence and memory continuity, and reproduces
all counts, statuses and full-report comparisons. No row is omitted or trimmed.

## Bridge controls and execution environment

Seventy-six fresh-instance negative controls trap as expected: malformed or
truncated payloads, input bounds, invalid metadata/operation, invalid indices and
policies, and illegal prepare/batch/finish ordering. Four positive sequences
confirm legal reuse behavior, including two additional expected traps when a
completed history instance is prepared again. The rational bridge resets work;
the reused history bridge permits repeated finish, not a second prepare. These
are collector checks, not hostile-input hardening of a production scalar API.

Two valid saved-record controls pass and 21 deliberate corruptions are rejected,
including signed coefficients, endpoints, source/completion flags, checksums,
policy/history, instance sequencing, clock, memory and allocation-field changes.

Runtime: Node v22.22.2, V8 12.4.254.21-node.39, CPU affinity 2, with
--expose-gc --no-liftoff --no-wasm-tier-up --no-wasm-lazy-compilation. All four
modules have no host imports and the expected export sets. Source/module hashes,
commands, complete output and terminal exit records are preserved. All four
offline locked release builds and Clippy checks pass, as does Rust formatting.
Both baseline/candidate dependency graphs contain 33 packages/nodes and match
fully after path normalization; all four lockfiles are identical.

The main runtime capture passes from 2026-09-11T20:46:20.889Z to
20:50:12.611Z, code 0 / null signal, empty stderr and an empty failure log.
ABI controls pass at 20:59:09.914Z; record/corruption replays pass at
21:01:21.042Z / 21:01:20.686Z. Independent evidence assembly passes at
21:03:33.060Z. Expected test traps are not failed execution gates. Earlier
checkpoint failures remain preserved; this is not an all-green audit history.

## Memory, storage and limits

Each rational full-corpus instance uses 33,816,576 bytes of linear-memory
capacity after initialization, with no later sampled growth in either variant.
The smaller cost-corpus instances use 1,703,936 bytes. History observations end
between 1,310,720 and 1,572,864 bytes, with the same range in both variants.
These are allocator-backed WASM memory capacities, not per-query requests,
live allocations, peak heap, process RSS or proof of equal memory costs.

The four modules occupy 5,818,881 bytes in /tmp/calcium-zero-wasm.ATQP07.
The complete raw observation file occupies 53,098,727 workspace bytes, with
additional small controls/evidence files. Existing scalar/solver trees and the
build cache are reused: no new solver-tree copy or deletion. /tmp availability
was 15,031,947,264 bytes at checkpoint start and 15,003,926,528 at the recorded
post-check; this includes shared-cache and other temporary usage, not just modules.

Qualification batch times are retained only as diagnostics. There are no paired
WASM timing claims. The rational bridge drops every timed report and requeries
outside the batch; the reused history bridge retains the last report until
finish, so the two timing boundaries are not equivalent. Checksums alone do not
validate every discarded internal batch result; full pre/post reports, source
checks, independent oracles and multiple batch lengths provide the recorded
qualification scope.

Next: predeclare matched WASM costs, then representative consumer and stripped-size
gates before deciding retention. The separate power-sum candidate, unread donor
files/supporting references, unresolved historical transfers and full requested
inventory reconciliation remain open. No general speed, memory, binary-size or
source-size improvement is claimed by this checkpoint.

Recheck with `node verify-zero-factor-wasm-v72.mjs`. This replays saved numerical
evidence and hashes the prior native cost evidence; it does not rerun the original
WASM batch campaign or recompute checkpoint-71 timing statistics.
