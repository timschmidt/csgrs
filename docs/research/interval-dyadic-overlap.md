# Dyadic overlap and disjointness qualification

Commit `207e88c55` delivers the three dyadic query families. The original
review measurements below remain available for comparison.
[Retained implementation and limits](#retained-query-implementation)

Plan §§3.4–3.5 assigns closed-set queries to Hyperinterval. On `67b0a2e74`,
two finite singleton inputs at exponents `i32::MIN` and `i32::MAX` exhausted
a 128-bit, 4,096-work-unit budget in both common-grid intersection and finite
strict order. Extended strict order decided which singleton precedes the
other with one bit and one work unit. The named overlap and disjointness
queries were missing for finite, unbounded and split dyadic intervals.

## Retained coverage

Commit `4cd5a9eef` exercises every pair from 27 connected inputs with integer
endpoints in `[-2, 2]`, including both ray directions, the entire line and
the empty set. On this domain, five endpoint cells and six intervening or
exterior cells determine the represented set exactly. Maximal runs in an
independent bitmap give the expected number, order and exact value of the
normalized components. All 729 constructor inputs and all 12,544 inclusion
pairs among the 112 distinct unions passed. Swapping the two returned
components or accepting a component outside the target each failed the test
on the baseline. A second test exercises extreme-exponent strict order,
exhaustion and retry through the existing extended API.

The standalone native release test executable occupies 1,175,536 bytes.
The four existing release cost executables for order images, division,
tangent and fractional powers were byte-identical before and after this
test addition. The all-feature Hyperinterval and Hyperlattice Interval suites
passed 359 tests and doctests. The downstream reachability suite passed 11.
Rust 1.88 dyadic-only tests and Clippy, strict all-target/all-feature owner
Clippy, rustdoc and the WASM library check passed.

## Implementations held for review

Both local review commits are based on `67b0a2e74`. The validation includes
19,600 independent finite endpoint pairs, closed endpoint contact, disjoint
exponent extremes and retry, plus the affected ASan fuzz targets. The full
proposal also covers ray contact, gaps retained by division and subsequent
comparisons of normalized unions. Across two fresh WASM instances, 7,776
calls matched the independent exact-set expectations. Whole linear memory
stayed at 1,114,112 bytes per instance.

| Local review commit | Measured benefit | Remaining gate |
| --- | --- | --- |
| `6019a652f` on `review/interval-dyadic-overlap-20261118` | Named queries for all three carriers. A sorted scan can prove separation of two two-component unions in five work units. The wide-input peak fell from 1,320 to 264 requested bytes. | On six alternating native runs, median time per 100,000 ray queries rose from 1.71 to 3.36 ms. Split-union queries rose from 1.61 to 2.71 ms. |
| `83c36c6fd` on `review/interval-finite-overlap-20261118` | Named finite queries, including borrowed comparison of same-grid mantissas. With 2,049-bit mantissas, both touching and disjoint queries completed with zero heap allocations. The three timed finite cases improved on each of three pinned CPUs. | A mixed executable adopting only the finite API kept the existing union algorithm. Its union case ran about 6% slower on two pinned CPUs and about 3% faster on the third across ten alternating runs per CPU. |

Each proposal passed incremental and cumulative size checks against the
previous primary and fixed `c9f1f9ac5` baseline for four native cost programs
and two new-API clients. The full proposal grew the linked native client by
736 bytes and the WASM file by 975 bytes. The finite proposal grew them by
184 and 832 bytes. Another finite version missed the 1,024-byte incremental
WASM file cap by eight bytes. No size or memory allowance was debited for
these proposals. The retained tests introduce no production call path.

The archive includes cold and warm guarded-stack measurements, heap counts,
full linked-image deltas, comparison sources, rejected patches, raw alternating
timings, work receipts and mutation logs. These are single-threaded x86-64
Linux and WASM client measurements. Firmware deployment was outside this
qualification. Raw evidence is registered under
`docs/evidence/2026-11-18-interval-dyadic-overlap` at local evidence commit
`41499f327`. [Evidence index](../evidence-archive.tsv)

## Retained query implementation

Commit `207e88c55` adds `overlaps` and `disjoint` to finite dyadic intervals,
extended dyadic intervals and normalized dyadic unions. The two boundary
comparisons for finite intervals use the existing budgeted point-order kernel.
Two rays pointing in the same direction always intersect. An opposite-ray or
finite/ray pair has only one possible direction of strict separation. The set
query advances past a component only when the classification proves it cannot
meet a remaining component. Two two-component inputs require at most three
pair classifications. The implementation keeps the sign and work schedule of
the existing finite endpoint comparator. No caller needs to fill a gap or align
both intervals to one grid to ask whether the intersection is empty.
[Implementation](../../crates/hyperinterval/src/extended/dyadic_overlap.rs)

The eight new tests cover 19,600 finite endpoint pairs, 729 connected pairs,
12,544 union pairs, opposite exponent extremes, 2,049-bit inputs, closed
contact and typed exhaustion followed by retry. An independent ordering oracle
constructs all 112 normalized unions on the five-endpoint arrangement. A
separate comparison exercises the existing constructor at 27 combinations of
bit and work budgets for every connected pair. Three deliberate faults in
endpoint contact, component advancement and ray classification failed the
corpus. The restored source passed. Both arithmetic and split-division ASan
targets completed 30,000 inputs, including replay of quotient components into
the new queries. The owner and adapter suites passed 367 tests and doctests,
with 11 downstream reachability and 9 solver expression-interval tests.
Rust 1.88 dyadic-only tests and Clippy, strict owner Clippy and rustdoc,
formatting, release tests and the WASM library check passed.
[Conformance tests](../../crates/hyperinterval/tests/dyadic_overlap.rs)

| Linked query client | Native file change | WASM file change | WASM code change | Ray time relative to control | Split-union time relative to control |
| --- | ---: | ---: | ---: | ---: | ---: |
| Workspace lock, `num-bigint` 0.4.6 | −96 B | +285 B | +202 B | 94.6–94.8% | 70.8–70.9% |
| Standalone lock, `num-bigint` 0.4.8 | +536 B | +679 B | +596 B | 90.3–90.7% | 69.3–69.6% |

The timing ranges are ratios of medians for ten alternating process pairs
on each of three pinned CPUs. Each process completed 100,000 queries per
case. The three finite cases also improved on every measured CPU. For each
lock, 7,776 WASM cases in two fresh instances matched independent set
expectations. Linear memory stayed at 1,114,112 bytes per instance.
The nine comparable final binaries passed each applicable incremental and
cumulative gate against `aa92b60c0` and fixed `c9f1f9ac5`. The existing
order, division, tangent and fractional-power programs preserved all 2,820
completed rows and work receipts per block. Their individual raw timing
distributions remain in the archive. The production change is confined to
the named queries. The separately measured normalization and division
refactors remain review experiments. [Size accounts](interval-size-accounts.json)

Whole-call stack measurements used twelve fresh processes per revision and
lock, two stack-fill patterns and separate cold and warm thread observations.
The measured boundary includes construction, query, retry and destruction for
the composition workloads. Prepared queries and complete division were also
measured separately. The composition workload charged **16 stack bytes** to
each of 24 lock, input and temperature combinations. The closest case grew
from 952 to 968 bytes. Its effective allowance leaves three incremental bytes
and 22 cumulative bytes. The minimum observed thread headroom was 2,085,520
bytes. No workload increased its requested or allocator-usable heap peak.
All measured results returned to zero retained heap bytes after drop. The
host qualification covers a single worker on x86-64 Linux and includes
separate portable WASM execution. [Memory accounts](interval-memory-accounts.json)

The preserved controls, executable deltas, mutation checks, clocks, budgets
and raw resource observations are registered under
`docs/evidence/2026-11-18-interval-directed-overlap` at local evidence
commit `b6503355e`. [Evidence index](../evidence-archive.tsv)
