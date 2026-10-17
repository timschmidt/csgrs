# Reject the unguarded circle-support index

The E38 index is **rejected as written**. It remains isolated; no cache
implementation is applied to main. This continuation made progress by completing
additional consumer suites and finding a reproducible cost regression on valid
exact inputs. The original 35-file native bundle and E36 native-ingress rejection
remain in force, as do the useful components independently admitted since then.

## Decisive same-contract evidence

Two concentric circles with center `(2^b + 3, 0)` and radii 5 and 7 have no common
point: a point cannot simultaneously have squared distance 25 and 49 from that
center. Both implementations returned the complete Disjoint relation with a
Certified receipt, one retained cache entry, and identical ordered result rows.
All 42 fresh-process samples passed, producing 84 cold/warm observations.

The diagnostic used seven matched repetitions per case, shuffled case order,
alternating implementation order, identical source apart from the index,
identical diagnostic code, and the same verified dependency libraries and
compiler flags. Input construction precedes phase timing. The warm phase includes
20,000 lookups, relation validation, and complete result-row materialization.

| Center coefficient shift b | Baseline warm median | Indexed warm median |
| --- | ---: | ---: |
| 64 | 0.000462768 s | 0.003775640 s |
| 65,536 | 0.000432840 s | 0.053107920 s |
| 1,048,576 | 0.000453839 s | 0.781765218 s |

The baseline samples are sub-millisecond and their ratios should not be treated
as precise general speed factors. The large-coordinate case adds roughly
0.78 seconds for the same 20,000 successful queries, consistently across all
seven repetitions. Cold single-query timings are retained as diagnostics only.
This is private-kernel evidence, not an end-to-end or distributed-binary claim.

The source explains the regression: Point2 equality first recognizes shared
owned storage; the index instead hashes exact rational coefficients on every
lookup. Rational cloning shares an Arc and does not duplicate coefficient
buffers, but its Hash implementation reads numerator and denominator words.
No approximate evidence or changed mathematical result was observed.

Earlier favorable large-batch measurements and the unchanged native report
bytes remain valid within their recorded scopes. They do not justify retaining
this supported-input regression. Any replacement must preserve cheap shared
support reuse and the earlier symbolic-first matching semantics, and must add
these coefficient cases to its acceptance corpus. No supported-domain restriction,
policy change, smaller original fixture, or weaker assertion is authorized.

## Consumer qualification and resource outcome

On main revision 67a6ba54f plus the unchanged three-file E38 proposal:

| Suite | Passed | Failed | Existing ignored |
| --- | ---: | ---: | ---: |
| CSGRS all features, tests and docs | 458 | 0 | 1 |
| CSGRS no default features, tests and docs | 161 | 0 | 0 |
| Hypercircuit all features, tests and docs | 348 | 0 | 0 |

These are per-configuration totals, not distinct-test counts. All builds and
format checks completed. The one existing ignored case is the expensive
byte-for-byte README render regression; it was not run. The external storage
guard stopped the subsequent
strict Clippy command when the /dev/shm per-user quota crossed its 512-MiB
reserve. No compiler error or warning was reported before that stop. Documentation
generation, native binding builds/tests/consumers, and their Clippy checks were
not run. The prepared WASM helper was never invoked. No broader pass is claimed.

The fresh native Cargo target is closed and preserved, including incomplete
Clippy output. Normal sequential builds overwrote five example executable paths
inside that then-open target; their time-of-build hashes remain recorded, but
those intermediate versions are unavailable for later read-only reuse. The
closure manifest pins every final physical path separately, including hard-link
aliases. It does not claim preservation of every intermediate build version.

The coefficient diagnostic used a different private runtime directory with
1,800-second and 8-GiB per-command external containment. Its guards retained
256 MiB of runtime filesystem space, 768 MiB of repository space, and 2 GiB of
available host memory. No guard stopped those commands. Completed sources,
dependencies and binaries remained unchanged. These are external controls, not
internal expression-growth, heap-byte, total-work or cancellation guarantees.

## Acceptance boundary and next work

This evidence closes the retention decision for the current index; unfinished
binding and matched full-corpus cost qualification cannot admit it. Main source
bytes and all earlier admission decisions remain unchanged. Only current_work
in the execution ledger is updated. The inherited 27,093 missing artifact paths
remain disclosed; no whole-history preservation claim is made.

Continue with a smaller cache revision that avoids this repeated hashing cost,
or another independently justified fix for ordinary strict-mode completion.
Requalify any revised implementation, including original hard cases and real
bindings, in fresh guarded storage. Full CI/pre-push validation, clean external
distribution, native exact-ingress admission, coverage, directed campaigns,
formal verification and the rest of the properties plan remain open.
