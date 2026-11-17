# Trace key representations

Content-keyed event recording is rejected on runtime even after its local memory
screen passes. Borrowed report keys preserve outputs and reduce temporary bytes,
but remain unadmitted: three original allocation-count accounts fail, and full
consumer lifetime, stack, runtime and distribution checks remain open.

This follows the [collector pilot](trace-materialization-pilot.md) and
[cancellation qualification](solver-cancellation-qualification.md), under
P4/P10/P36 and the unchanged admission gates. The control is
`4f41bd58415d936968e1955dbd431f0cb5e988f0`: the scope-local cancellation candidate
plus the independent trace regression retained in primary `3fa808111`.
No production change from these new candidates is merged into the primary tree.

The [runtime follow-up](trace-borrowed-runtime.md) now records the large-report
non-regression gap and a retained unwind-cleanup witness.

## Source candidates and local memory

Each source runs the original 276-case collector corpus, with independent ordered
labels/counts and complete output comparisons against the immutable control.
The corpus covers empty, unique and identity-distinct equal labels, snapshot/take,
cold/eight warm calls, and three repetitions. Nine owner tests, strict
all-feature/all-target owner Clippy, consumer Clippy and formatting pass for each
source. Recording and report materialization have separate accounts.

| Source | Change | Local result |
| --- | --- | --- |
| `48c1fbe426c7ad8bba08441b9e93a2930e2651dc` | Content equality/hash in the live table; collect unique entries and sort | Recording passes; 32 retained-report accounts fail |
| `61298adca134f0c2cb7181b015e9d4d4ccc6684c` | Preallocate the exact iterator length before extending/sorting | All 184 local accounts pass; rejected by runtime below |
| `d2848f1a083f8fe25ef8ea98a62cd7ad7c2feaf0` | Preserve identity recording; borrow keys during BTreeMap report aggregation | All byte accounts pass; three allocation/free accounts fail |

The first content-keyed collector uses generic vector collection, which retains
capacity four for a one-row report: 224 requested / 232 usable bytes instead of
56 / 56. Requiring the already-available exact-size iterator and preallocating
its length repairs that specific issue without a shrink or reallocation. This
new source retains the earlier failure and its evidence; no allowance changes.

Borrowed report keys leave identity equality/hash, the hasher, recording guard,
recording functions, TLS and rational counters byte-identical. Report ordering
compares layer, operation and path in the original lexicographic order. Returned
rows own static string handles and counts; they do not borrow the live table.
The temporary keys borrow it while the state borrow remains held. `take()` uses
a drop guard to clear on normal return or unwinding, retaining table capacity
as the original drain did. This is an implementation review, not an additional
runtime claim about untested failure paths.

The borrowed-key source reduces requested materialization peak/traffic by up to
248,360 bytes and usable peak/traffic by up to 243,976 bytes. Retained report sizes
and reallocations never increase; recording observations are unchanged. Three
cross-process accounts fail allocations and frees by one: 64 identities / 32 text
values with cold take, 65 / 32 with cold snapshot, and 256 / 256 with warm take.
Their requested peaks still fall from 5,048 to 3,040 bytes for the first two and
37,568 to 21,920 bytes for the third. These failures remain recorded.

## Event-time rejection

The exact-capacity content-keyed table hashes complete label text for every
recorded event, using the unchanged byte-wise hash writer. A separate consumer
measures four completed rounds of 4,096 events, including reset, take, exact row
validation, empty-snapshot checks, report cleanup and serialization. Input
preparation, printing, serialization-buffer destruction and TLS teardown are
outside timing. No allocator monitor is present in this timing executable.

Both source images run with the same argv, working directory, environment and
one pinned CPU. Sixteen domains combine label lengths 5, 64, 4,096 and 32,768,
one or two distinct addresses with equal text, and active/passive recording.
Twelve paired repetitions alternate source order. All 384 controls complete;
all 192 full-output pairs and 384 independent expectations match.

For one address and active recording, observed medians in milliseconds are:

| Label bytes | Control CPU | Candidate CPU | Paired CPU increase |
| ---: | ---: | ---: | ---: |
| 5 | 0.841 | 1.180 | 0.347 |
| 64 | 0.837 | 2.452 | 1.614 |
| 4,096 | 0.883 | 89.294 | 88.408 |
| 32,768 | 1.057 | 704.753 | 703.695 |

All 96 active pairs are slower. For 32,768-byte labels across both identity
counts, every paired increase lies between about 701 and 727 milliseconds.
The material absolute increase rejects this source; no runtime allowance or
precise speedup ratio is inferred from short control timings. Passive results,
clock resolution, wall times and every raw sample remain available. This is a
rejection screen, not a replacement for owning benchmarks or geometry workloads.

## Shared-input allocation diagnostic

Pointer-based hashing makes the live table's iteration depend on label addresses.
The original collector already varies allocation counts across fresh processes.
To distinguish iteration effects from borrowed-key costs, a diagnostic compiles
both unchanged production prefixes into one process and passes exactly the same
label addresses, insertion sequence and counts to both. Instrumentation exposes
the actual identity-table iteration; complete vectors are retained and compared.
The original owner tests remain with their source crates. Diagnostic modules
exclude those test modules and do not claim scalar/geometry qualification.

All 276 configurations run in both source orders: 552 completed pairs. Every
actual iteration vector and full report matches; 1,104 independent output
expectations pass. Allocation, free and reallocation counts agree in every pair.
All 1,104 paired recording/materialization accounts pass the original local caps.
Requested materialization peak/traffic decreases by up to 244,640 bytes, usable
by up to 240,192 bytes; report retention remains equal. Requested recording
tuples match exactly, while usable peak/traffic varies by up to 4,056 bytes in
either direction. Those positive observations remain in the evidence.

This finite controlled result supports separating address-dependent iteration
from key storage. It does not erase the three original failures, establish a
universal allocator/ASLR explanation, or qualify the full consumer. For example,
the shared-input control itself reaches 38 allocations in the 256-label warm-take
case, while the earlier standalone control maximum was 37; both diagnostic
collectors have 38 in that same process.

## Execution and remaining gates

Each source-memory pilot uses 293 direct children and completes within its
1,800-second / 500-child envelope. The runtime screen uses 400 direct children
in about 168 seconds within its 1,200-second / 500-child envelope. The shared-input
diagnostic has two zero-control harness failures: strict Clippy finds a private
hook whose normal scalar caller is absent, then formatting requires alphabetical
module declarations. A diagnostic forwarding function preserves reachability
without suppressing warnings; declaration order is corrected. Both failed
attempts, their seven commands and corrections remain preserved. The final
556-command run completes within the original deadline: 563 aggregate direct
children and about 143 seconds, below the 700-child / 1,200-second limits.
No geometric control is retried and no deadline resets.

All independent offline verifiers pass within their 300-second bounds. The
source worktree is restored to the control, queue parent remains held and its
original watcher remains terminal. No prior account, cap, fixture or charge is
changed. Accepted totals stay 57 size accounts and zero memory exceptions;
inventory stays 1,644 rows. No push; the full fifteen-phase goal remains active.

Further borrowed-key qualification is deferred while the runtime follow-up is
unresolved. Whole recording/materialization/reset lifetimes, original whole-call
stack, owning benchmarks, `successive_operations`, both-policy geometry chains,
exact traces/certainty, fixed-baseline cumulative costs and final native/WASM/
package measurements remain required before retention. Collector-only results
and shared-input diagnostics do not satisfy those gates.

Frozen evidence: `docs/evidence/2026-11-17-interval-trace-key-representations`,
archive commit `5fd724d2341e4ad19fed277370f27e2bb66ccf0b`; 2,011 files / 19,028,727 bytes,
SHA-256 `f39d6584b892a9d999a06cafaf00e4b3922f14d35346987b6201af07cf279e9c`.
All original, stored and committed hashes are verified, including six validated
native images. Earlier archive paths are unchanged. Initial failed diagnostic
builds have no claimed runtime controls or frozen image. The preceding collector
record supplies the immutable baseline logs referenced by these verifiers.
