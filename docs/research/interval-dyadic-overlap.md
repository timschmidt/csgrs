# Dyadic overlap and disjointness qualification

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
