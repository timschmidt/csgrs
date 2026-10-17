# Lazy circular-support index qualification

**Unadmitted: lazy index completes local qualification; measured transition costs and downstream acceptance remain open.** Production is unchanged. The original
35-file native bundle and both earlier index proposals remain rejected; independently
admitted repairs remain intact. The full properties plan remains open.

The owning Hypercurve cache now avoids constructing keys or an index for small
batches. After 32 retained relations, an eligible exact query lazily indexes
prior entries while retaining first-match priority and symbolic/very-large
fallback. This isolated proposal is preserved for review and is not applied.

All eight previous cache regressions remain intact. A ninth checks 80 mixed
symbolic, million-bit and ordinary rational supports, rebuilt owners and
changed geometry across activation. Both matched builds pass all nine tests
and strict direct Clippy for the no-feature test harness.

| Full library configuration | Passed | Failed | Existing/diagnostic ignored |
| --- | ---: | ---: | ---: |
| corpus-v1 | 1743 | 0 | 4 |
| all-feature-library-v1 | 1750 | 0 | 9 |

All-feature diagnostic Clippy also passed. The no-feature corpus preserves
one original manual benchmark and three explicitly executed diagnostic tests;
the all-feature corpus preserves six original manual benchmarks and those
same three diagnostics. No source-mirror mutation or property-persistence
warning occurred. Counts overlap across configurations and do not measure
coverage. Full Cargo target/feature/manifest, integration, docs, examples,
benches, downstream native/report and real-binding gates remain unexecuted.

The complete small-batch comparison retained every earlier size and added
31, 32, 33 and 64. All 140 matched processes / 280 phases preserve the independent
3-4-5 oracle, strict certification and exact ordered rows. Each phase performs
at least 65,536 queries with full output materialization and validation.

| Pairs | Baseline cold s | Lazy cold s | Baseline warm s | Lazy warm s |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 0.097113507 | 0.095881001 | 0.050781100 | 0.049448532 |
| 4 | 0.113296274 | 0.110977924 | 0.064607460 | 0.065742222 |
| 8 | 0.133682933 | 0.131092621 | 0.078933006 | 0.078446899 |
| 9 | 0.135526397 | 0.134720022 | 0.079772718 | 0.079198028 |
| 16 | 0.173484308 | 0.169890025 | 0.100591508 | 0.100798163 |
| 31 | 0.213970016 | 0.207329193 | 0.135753231 | 0.135180761 |
| 32 | 0.217384292 | 0.208783073 | 0.139388501 | 0.088810407 |
| 33 | 0.215041133 | 0.235105254 | 0.138661381 | 0.090188042 |
| 64 | 0.294508042 | 0.204987244 | 0.211508425 | 0.090338712 |
| 128 | 0.435714758 | 0.187200456 | 0.357907654 | 0.092067363 |

The earlier 4/8/9-pair cold regressions are absent in this comparison.
Cold activation at 33 pairs is consistently 9.33% slower, with separated
sample ranges. That remains an unfavorable result despite falling below
the predeclared 10% decisive threshold. The small warm-4 median is also
1.76% higher with overlapping ranges; all raw observations are retained.

The original fifteen-case corpus also completed: 210 matched processes /
420 phases, all independently checked and Certified, with exact ordered rows
identical across variants and cache phases. Median phase seconds:

| Case | Baseline cold | Lazy cold | Baseline warm | Lazy warm |
| --- | ---: | ---: | ---: | ---: |
| rational 1 | 0.000077865 | 0.000071725 | 0.000005939 | 0.000002580 |
| rational 16 | 0.000161659 | 0.000163359 | 0.000041557 | 0.000041267 |
| rational 128 | 0.001183199 | 0.000733990 | 0.000803914 | 0.000227504 |
| rational 512 | 0.012048592 | 0.002113935 | 0.010116794 | 0.000850351 |
| rational 2048 | 0.160645231 | 0.008002110 | 0.154661842 | 0.003308252 |
| rational 8192 | 2.500912829 | 0.033308581 | 2.454941997 | 0.013959281 |
| symbolic 128 | 0.007744428 | 0.007606107 | 0.006548010 | 0.006599817 |
| symbolic 512 | 0.083992329 | 0.082634501 | 0.079254274 | 0.077767886 |
| mixed 128 | 0.009461300 | 0.008953905 | 0.008460239 | 0.008206676 |
| mixed 512 | 0.119571223 | 0.116430379 | 0.115672671 | 0.114397698 |
| superrange 128 | 0.002486220 | 0.001306150 | 0.001883141 | 0.000615498 |
| subrange 128 | 0.011196771 | 0.005601795 | 0.007514504 | 0.001687794 |
| shared-coefficient 64 | 0.000019149 | 0.000020038 | 0.000492866 | 0.000561201 |
| shared-coefficient 65536 | 0.000024848 | 0.000023188 | 0.000478737 | 0.000561621 |
| shared-coefficient 1048576 | 0.000036238 | 0.000035078 | 0.000395533 | 0.000451729 |

Single-query and other short phases are diagnostic controls, not precise
general speed factors. The shared-center warm controls suggested extra cost,
so a separately predeclared follow-up reused the exact same binaries and
20,000-query probe after both full-library processes were terminal.

It collected seven matched groups of eight fresh processes per variant:
112 processes, each requiring exact Disjoint rows and a strict receipt.
Each group sums 160,000 warm queries. This is an aggregate of independent
fresh-process phases, not one contiguous timing or an end-to-end workload.
The baseline median group took 0.003924050 s;
the candidate took 0.004314295 s (+9.94%).
Positive paired group differences: 7/7;
candidate range wholly above baseline: True.
The predeclared rejection rule is not met.

All 20 complete-process allocation observations preserve the uninstrumented
exact witnesses. Instrumented times are excluded from latency comparisons.
One-pair allocation counts now match. Heap results include construction,
queries, validation, output and teardown; they do not isolate cache heap or
establish an internal bound. Diagnostic executable size is not package size.

| Workload | Baseline allocations / peak | Lazy allocations / peak |
| --- | --- | --- |
| rational 1 | 786 / 374.57K | 786 / 374.57K |
| rational 512 | 817906 / 2.56M | 41497 / 2.48M |
| rational 2048 | 12706548 / 9.96M | 159773 / 9.60M |
| symbolic 128 | 476069 / 814.81K | 476069 / 814.82K |
| mixed 128 | 606821 / 846.30K | 595429 / 854.12K |
| superrange 128 | 110967 / 855.15K | 17749 / 869.76K |
| subrange 128 | 375941 / 2.10M | 96230 / 2.07M |
| shared-coefficient 64 | 713 / 374.61K | 713 / 374.61K |
| shared-coefficient 65536 | 726 / 374.61K | 726 / 374.61K |
| shared-coefficient 1048576 | 726 / 754.99K | 726 / 754.99K |

The next implementation hypothesis is an early direct return for small
cache hits before index bookkeeping, combined with a separately evaluated
activation threshold. Neither change is implemented or admitted here.
The existing complete workload gates, hard fixtures and exact-output/report
contracts remain required before reconsidering native track integration.

Raw logs, commands, source/feature/dependency/toolchain records, all samples,
helpers and the reviewable proposal are retained in a verified lossless
archive with per-member hashes. Original physical outputs remain untouched.
Only the permitted current_work ledger value changes. The inherited 27,093
missing paths remain disclosed; this is not a whole-history audit, a release
claim or completion of the properties plan.
