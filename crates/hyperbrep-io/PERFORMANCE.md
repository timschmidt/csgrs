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
