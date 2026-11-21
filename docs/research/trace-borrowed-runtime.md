# Borrowed trace keys: runtime qualification remains open

Borrowed report keys remain unadmitted. They reduce temporary bytes, but the
completed-workload timing screen shows higher median times for large unique-label
reports. A shared-input diagnostic retains that signal. These observations do
not establish non-regression, a universal slowdown, or a specific CPU/cache cause.
The retained change is an independent regression for cleanup when `take()` unwinds;
primary production code remains unchanged.

This follows the [key-representation review](trace-key-representations.md), under
P4/P10/P36 and the unchanged performance gate. The runtime sources are control
`4f41bd58415d936968e1955dbd431f0cb5e988f0` and borrowed-key candidate
`d2848f1a083f8fe25ef8ea98a62cd7ad7c2feaf0`. Their previous byte results, three
cross-process allocation-count failures and shared-input allocation diagnostics
remain preserved. No memory exception is used to waive runtime qualification.

## Completed source-runtime workloads

An identical consumer built against each source completes 1,344 fresh-process
controls: 56 domains, twelve paired repetitions, alternating source order. Each
sample completes at least 1,048,576 events across repeated reset, recording,
snapshot/take, exact validation of every returned field and count, serialization,
report destruction, counter reset, empty checks and recording-scope cleanup.
There are 224,027,904 completed rounds across all controls. The final full reports
match in all 672 pairs and satisfy 1,344 independent exact expectations.

Active domains combine 1, 12, 65, 256 or 4,096 identity-distinct labels; one,
seventeen or all distinct text values where applicable; 5- or 256-byte labels;
and snapshot/take. Eight passive domains check the recording guard. Both images
use the same argv, working directory, environment and pinned CPU. Process CPU,
wall time and clock resolution are retained for every sample; none is discarded.
Active CPU times range from about 62 milliseconds to 1.13 seconds. Some passive
controls take about 4 milliseconds; no speedup claim is made from those timings.

Timing includes validation and serializing every round into a reused string.
Input creation, final printing, text-buffer destruction and retained TLS teardown
are outside timing. No allocator interposer is present. This does not measure
whole-process memory or qualify geometry, owning benchmarks or shipped binaries.

The 4,096-unique-label snapshot cases have median paired CPU increases of 6.34%
for short labels and 7.31% for long labels, about 34.3 and 40.5 milliseconds per
completed workload. Ten and eleven of twelve pairs, respectively, are slower.
Several smaller/shared-label domains favor the candidate; those observations
cannot cancel the unresolved large-report signal. Substantial variation and
contrary samples remain visible. No automatic timing allowance is introduced.

## Same addresses and actual iteration

A second consumer compiles the unchanged production prefixes and diagnostic
accessors from the earlier shared-input allocation experiment. Both collectors
receive exactly the same static input addresses, insertion order and counts.
Their actual identity-table iteration vectors are retained and equal in all 192
fresh-process pairs. Both tables are filled once before timing, so this diagnostic
starts with retained table capacity; it is not a replacement for the source screen.

Sixteen domains combine 256/4,096 identities, seventeen/all text values, both label
lengths and both collectors. Twelve paired repetitions alternate execution order.
The timed sequence otherwise matches the source screen. All 192 full report pairs
and 384 independent exact expectations pass. Every sample exceeds 130 milliseconds
of process CPU time. No allocator monitor is compiled into this executable.

For 4,096 unique labels, observed median paired CPU changes are:

| Label bytes | Collector | Median change | Slower pairs / 12 |
| ---: | --- | ---: | ---: |
| 5 | snapshot | +5.47% | 9 |
| 5 | take | +4.55% | 9 |
| 256 | snapshot | +1.38% | 8 |
| 256 | take | +0.46% | 6 |

Each of these domains also has a positive median within each source-order group.
The short-label snapshot medians are about +5.63% with control first and +5.34%
with borrowed keys first. Thus equal iteration and alternating order do not
establish non-regression. This finite diagnostic does not identify an underlying
hardware/allocator mechanism or prove a universal performance ordering. Full
lifetime/stack and distribution campaigns are deferred while runtime is unresolved.

## Retained cleanup witness

The new debug regression constructs two identity-distinct static labels with equal
text and privately seeds counts whose sum overflows `u64`. This reaches a panic
during aggregation without executing an unbounded event stream. The test checks
that `take()` leaves no counts after unwinding, the recording scope is restored,
an outside-scope record is ignored, and a later scope produces a fresh exact
report. It is enabled only with debug assertions, matching its overflow trigger;
release wrapping behavior is unchanged.

All ten affected debug tests pass on both the primary implementation and the
borrowed implementation, along with strict all-target/all-feature owner Clippy
and workspace formatting. The candidate-plus-test commit is isolated at
`024e1c1dbc5cf1035d47a985f5dd0aa7e5e7b4f8`. Both production prefixes are verified
byte-identical to their respective pre-test sources. Only the regression is
retained on primary; the borrowed implementation remains unmerged.

## Execution, evidence and disposition

The source-runtime producer uses 1,360 direct children and about 521 seconds,
within 1,600 children / 2,400 seconds. Shared-input runtime uses 197 children and
about 199 seconds, within 300 / 1,200. The owner-test producer uses 17 children
and about 122 seconds, within 100 / 1,800. Each producer counts direct spawns at
one site; compiler descendants are explicitly excluded. Runtime begins without
competing owned compiler work. Offline verifiers pass within their 300-second
bounds. The initial inline unwind audit and its durable verifier replay are both
retained; no tests or geometry are rerun for that bookkeeping step.

The first archive staging check rejects an intentional trailing separator in the
saved test insertion fragment. The fragment is retained losslessly compressed,
with unchanged raw hash and explicit encoding; full compiled sources stay raw
and unchanged. The failed archive command/log and reviewed continuation remain.
No source, measurement, protocol, test or cap changes to obtain archival success.

The source worktree is restored to control, all task producers are terminal,
queue parent remains held and its original watcher remains terminal. Accepted
totals remain 57 size accounts and zero memory exceptions; inventory remains
1,644 rows. Earlier charges and failures remain recorded. No push; the full
fifteen-phase objective remains active. The optional borrowed-key optimization
is not a prerequisite for further work on the functional migrations. Their
remaining admission evidence and bounded allowances must be assessed directly,
without silently discarding historical costs or weakening other gates.

Frozen evidence: `docs/evidence/2026-11-17-interval-trace-borrowed-runtime`,
archive commit `ab588c47a0cf2273a0164fb837cdacb93ebc1444`; 1,645 files / 10,550,284 bytes,
SHA-256 `0dd304266c9fdd74b8b04985f7034ef8f7e96eaffcb0dfaf331b68d882dc075a`.
All original, stored and committed hashes are verified, including three native
timing images. Earlier archive paths are unchanged.
