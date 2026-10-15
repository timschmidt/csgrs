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
