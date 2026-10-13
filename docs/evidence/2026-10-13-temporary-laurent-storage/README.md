All three tested storage variants are **rejected for runtime admission**. They
save allocation calls but introduce repeated scalar slowdowns and larger
temporary layouts. See [decision-v1.json](decision-v1.json) and the complete
[cost assessment](cost-assessment-v1.json).

This isolates one inline slot for private temporary Laurent terms, radical
entries, and both together, against a matched Vec baseline. The uninstrumented
endpoint/P4 combination supplies the parent. All four sources add the same
locked SmallVec dependency with no requested features and the same eight tests.
Persistent certificate fields and opaque atom vectors keep their existing
storage. Arithmetic, ordered insertion, original retries, recognition limits,
proof retention and original native performance assertions remain unchanged.

The preceding retained-projection diagnostic observed 35.24 million recursive
copies, 98.89% with one term. Those counts motivate this experiment; they do not
establish allocation savings or performance improvement. Inline slots may
increase temporary/memo/stack sizes, move costs and spilling costs. Qualification
records layouts separately from later requested-heap measurements.

Preparation froze four separate 10,132-file sources, each with exactly four
changed files. The baseline runtime body matches the parent; every candidate
recovers the original source tokens after its explicit storage replacements.
The lockfile changes only hyperreal's dependency list, preserving all package
versions and checksums. Sequential qualification completed in all four fresh targets. Eight shared tests cover entry limits, cancellation after spilling,
regrowth, retained copies, exact products/inverses, power limits, opaque owner
release, and cold/warm/cloned proof queries. Every variant also passes six cache controls, eleven P4 regressions, 882
all-feature and 795 default scalar library tests, 28 doctests, formatting, strict
default/all-feature/all-target Clippy and warning-denied docs. Focused and
configuration totals overlap. All 40 commands and their hashes are verified in
[correctness-v1.json](correctness-v1.json).

The coefficient/polynomial sizes are 56/24 bytes for baseline, 64/24 for radical
storage, 56/80 for term storage, and 64/88 for both. Persistent coefficient,
persistent polynomial, Computable and Node sizes remain 32/24/16/56 bytes. These
are host type layouts, not total-memory measurements.

The completed scalar comparison used three unchanged probes, two separate
resource modes and five four-variant blocks: 24 preflights and 120 measured
processes. The first four blocks balance positions; the fifth reverses the
first. Every build preceded measurements, each source has a dedicated target,
and the sample set is closed. Every mathematical witness and representation
inventory matches. All commands, sources, libraries, executables and raw sample
bindings were verified.

| Observation, out of 68 rows per variant | Radical slot | Term slot | Both |
| --- | ---: | ---: | ---: |
| Slower in all five timing blocks | 11 | 19 | 19 |
| Faster in all five timing blocks | 7 | 3 | 5 |
| Reduced allocation calls | 36 | 51 | 51 |
| Increased requested allocation bytes | 15 | 8 | 8 |
| Increased peak requested bytes | 29 | 20 | 18 |
| Probe executable increase | 9,800–10,864 B | 11,680–12,728 B | 11,544–12,208 B |

The radical slot slows the quadratic probe's warm multiradical-square query in
all five pairs, with a median paired ratio of 1.15363. The term slot slows the
rational probe's warm multiradical-square query in every pair, with a median
paired ratio of 1.34093. The combined variant slows the warm wide-integer affine
query in all pairs, with a median paired ratio of 1.17107. These are descriptive
sample ratios, not significance or application-speedup claims. Every faster and
slower row remains in the assessment.

Allocation-call counts never increase in these probes. Nevertheless, larger
elements and memo entries can cost more requested bytes: radical-only pi
transport adds 372,616 bytes per 1,000 first queries; term-only cloned opaque
declines add 265,152 bytes; combined cloned opaque declines add 329,152 bytes.
Retained requested-byte deltas match in every phase and pair. Peak increases
reach 104, 320 and 384 bytes respectively. Requested-heap counters exclude stack
frames, native C heaps and allocator bookkeeping; they do not establish total
memory or eventual process reclamation. Probe sizes are not production CLI
sizes.

These regressions settle rejection before native follow-up. No native timing
gate, endpoint/broad/abort control, full-report replay or production CLI build
was executed for these variants. The parent's original board failure remains
open; its timing is not a timing result for a rejected storage variant.

The next hypothesis keeps polynomial terms as Vec and investigates the radical
slot with SmallVec's `union` feature. Source review suggests this could avoid
the extra coefficient layout word, but actual layouts and costs remain to be
established. A separate Vec feature-only control is required because dependency
features unify across other native consumers. This future experiment is not
prepared or measured. The completed sources and sample set remain frozen.

The native parent is still unadmitted. The original 120-second board and
60-second neck limits, all 35 pending native entries and all 39 properties,
14 targets and crate-specific future work remain in scope.
