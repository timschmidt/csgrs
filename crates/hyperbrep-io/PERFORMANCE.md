# HyperBRep IO performance

## Part 21 syntax — 2026-10-15

Retain flat borrowed syntax and reuse parser/writer frame buffers across records.
Three alternating pairs pinned to CPU 0 compare the initial implementation
(fresh buffers per record) with the retained implementation, including the later
lexical corrections. Both versions accept these valid syntax fixtures; the
initial version fails the new invalid-token regressions. These benchmarks
instrument Rust allocator calls; timings include that instrumentation. All four
case names, iterations, output bytes and checksums match. Expected output is
constructed independently of the production parser/writer.

| Case | Iterations/process | Baseline median | Retained median | Ratio | Allocation requests/cycle, before → after |
| --- | ---: | ---: | ---: | ---: | ---: |
| Small exact-lexeme record | 1,000 | 3.448 µs | 3.040 µs | 0.881671 | 21 → 15 |
| 10,000 referenced point records | 20 | 8.898959 ms | 7.371986 ms | 0.828410 | 20,078 → 74 |
| 20,000 nested aggregates | 20 | 1.128832 ms | 1.089528 ms | 0.965182 | 64 → 58 |
| Byte-limit rejection | 100,000 | 5 ns | 5 ns | 1.000000 | 0 → 0 |

Requested allocator bytes per cycle are 4,682 → 4,202, 18,508,330 → 16,908,010,
5,857,866 → 5,857,386, and zero respectively. These sum Rust allocation and
reallocation requests; they are not retained heap, foreign allocation or peak
memory measurements. Input/output sizes are respectively 197/207, 377,932/377,941
and 40,152/40,162 bytes. Input construction is outside timing. The byte case
rejects a 1 MiB input before UTF-8/token decoding and performs no allocations.

The release benchmark grows 280 bytes to 656,088. Median process peak RSS is
12,416 versus 13,148 KiB; all three raw samples per side remain available.
These descriptive trials do not prove significance or costs on arbitrary
STEP geometry. Numeric conversion, schema resolution and BREP validation are
separate workloads and are not included in these syntax timings.

The earlier three unpaired prototype trials and the matched comparison before
lexical corrections are retained separately and do not contribute to the final
paired comparison. The upstream-parser probes are correctness
comparisons only: their floating numeric carriers fail the exact-input contract,
so they are not interchangeable performance baselines.

[Reproduction, sources and raw evidence](../hyperbrep/docs/evidence/2026-10-15-step-syntax/README.md)
include native tests, four restored fault controls and execution of a WASM probe.
The older HyperBRep archive-admission timing issue remains open; this new-crate
comparison does not qualify that historical change.

## Faceted representation import — 2026-10-15

**Kept:** strict faceted mapping through the native builder; iterative cached
unit resolution; exact rational point/edge interning; hash-set membership checks;
and pre-product unit-scale bit admission. This is new functionality with no
equivalent previous importer. The table reports absolute costs, not a speedup.
Three fresh processes run the same allocation-instrumented benchmark on CPU 0.
Every iteration checks dimensional-recipe counts and exact volume. Geometry rows
include parse, header/unit/geometry mapping, full native validation, volume and
exact native JSON serialization, with complete output compared against a first
reference import. That first import and earlier cases can warm scalar caches;
these are repeated-operation costs, not isolated cold timings.

| Case | Iterations/process | Median per iteration | Allocation requests/iteration | Requested bytes/iteration |
| --- | ---: | ---: | ---: | ---: |
| Box | 100 | 1.144366 ms | 3,099 | 598,709 |
| Two boxes | 50 | 2.323810 ms | 6,487 | 1,239,093 |
| Through-hole | 30 | 3.566425 ms | 13,568 | 1,714,770 |
| Oblique tetrahedron | 50 | 925.084 µs | 1,965.68 | 351,216.52 |
| 2,000-link unit chain | 10 | 6.715144 ms | 4,113 | 5,553,196 |
| 20,000-link unit chain | 3 | 66.116269 ms | 40,141 | 55,501,196 |
| Unit-scale bit rejection | 100 | 229.821 µs | 1,523.01 | 74,368.32 |

Fractional counts are per-process averages followed by the median across three
processes. Requested bytes sum allocations/reallocations, not retained memory.
Unit rows include syntax/header/graph resolution and drop; bit rejection starts
from already parsed syntax. The tenfold-depth observation is not a general
linear-time bound. Geometry outputs remain 34,060 / 68,183 / 66,393 / 18,472 bytes
with matching complete output, FNV fingerprints and receipts across all trials.
The benchmark executable is 20,928,888 bytes; process peak RSS samples are
41,400 / 43,268 / 44,108 KiB. The executed WASM probe is 14,721,483 bytes with
zero host imports. The initial costs before the later text-decoding correction
are retained separately and are not a matched optimization baseline.

The unchanged syntax benchmark was also rebuilt from `cd1cc8d42` and the current
IO source in isolated equivalent Cargo contexts, then run in three alternating
CPU-0 pairs. Small/wide/deep/byte-rejection median ratios are
0.992636 / 1.044287 / 1.002761 / 1.020947. All four output receipts and existing
allocation counts are unchanged. Ratios use total elapsed nanoseconds divided
by iterations; the printed integer per-iteration values truncate (4 versus 5 ns
for byte rejection), so their quotient would distort the observed timing.
The isolated syntax executable grows from 656,008 to 3,020,240 bytes with native
geometry dependencies. This is a measured dependency cost, not a code-size win.

[Raw costs, sources, controls and reproduction](../hyperbrep/docs/evidence/2026-10-15-step-faceted/README.md)
support this selected-domain increment. Generic STEP documents, independent
exporter corpora, all native validation domains and global work/memory admission
remain open. The earlier archive-admission timing gate is unchanged.

## Exact native faceted export — 2026-10-15

**Kept as new functionality:** exact decimal admission, boundary-derived plane
frames with primitive integer direction ratios, native-identity point reuse,
complete directed-use accounting and ASCII metadata append. No equivalent old
geometry writer exists, so the following are absolute costs, not speedups.
Three fresh allocation-instrumented processes run on CPU 0. Native construction
and source-volume assertions are outside timing; each timed iteration includes
export, output/provenance counts, complete text comparison and drop. A first
reference export can warm scalar caches. The rejection row includes creating
then discarding the partial internal buffer, with no returned artifact.

| Case | Iterations/process | Median per iteration | Allocation requests/iteration | Requested bytes/iteration |
| --- | ---: | ---: | ---: | ---: |
| Box | 1,000 | 75.507363 µs | 736 | 21,780 |
| Through-hole prism | 500 | 134.104006 µs | 1,356.002 | 44,732.352 |
| Concave prism | 500 | 117.505948 µs | 1,131.002 | 39,850.352 |
| Split box edge | 1,000 | 79.357272 µs | 770 | 29,000 |
| 32 repeated body occurrences | 30 | 2.2138392 ms | 22,075 | 692,181 |
| Reject coordinate `1/3` | 1,000 | 4.152168 µs | 42 | 2,606 |

All six receipts and all complete output comparisons agree. Geometry output
sizes are 2,965 / 4,396 / 3,620 / 3,021 / 63,172 bytes, with stable recorded FNV
fingerprints. Requested bytes sum allocator requests, not retained/peak heap.
The export benchmark is 21,808,416 bytes; process peak RSS samples are
9,628 / 9,648 / 9,644 KiB, including untimed construction. The executed WASM
probe is 14,514,067 bytes with zero host imports. That probe has different code
coverage from the previous importer probe; its size is not a matched improvement.

The unchanged seven-row importer harness also runs in three alternating CPU-0
pairs against a frozen executable bound to the preceding `25a48c202` evidence.
Box / two bodies / hole / tetrahedron / 2k units / 20k units / scale-rejection
median elapsed-total ratios are 1.000413 / 1.007029 / 1.024714 / 1.005463 /
0.978652 / 0.970498 / 0.987288. Every receipt, output fingerprint and allocation
count is preserved. Importer executable sizes are 20,928,888 → 20,927,880 bytes;
median process RSS is 41,772 → 41,628 KiB. These samples do not establish a
statistically significant speedup or costs on large arbitrary models.

[Sources, raw trials, checks and reproduction](../hyperbrep/docs/evidence/2026-10-15-step-export/README.md)
bind this increment to its exact code. No additional optimization was benchmarked
and rejected in this increment. The prior archive-admission timing gate remains
open; new IO evidence does not qualify that historical change.
