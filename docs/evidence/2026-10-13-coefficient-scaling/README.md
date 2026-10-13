# Rational scaling inside normal-form coefficients

Both scaling variants are **rejected for runtime admission**. Their scalar
allocation savings do not carry into the native endpoint controls, which show
persistent slowdowns and four peak increases. Both also fail the original
120-second board assertion. The parent endpoint trial and its earlier tradeoffs
remain open. See [decision-v1.json](decision-v1.json).

`right` consumes the already owned left coefficient when the right coefficient is
purely rational. `both` additionally clones and scales the borrowed right
coefficient when the left is purely rational. Both use the existing exact scaling
helper. General multiplication, normal-form budgets, ordered insertion and the
mandatory original fallback remain byte-for-byte unchanged.

The shared four regression functions pass on the unchanged runtime baseline and
both candidates. Each candidate passes 867 scalar library results, 28 doctests,
strict all-feature/all-target scalar Clippy, formatting and warning-denied docs.
The four new and six retained-cache functions are included in the library totals.
See [correctness-v1.json](correctness-v1.json).

The unchanged rational, quadratic and integer probes ran 18 preflights followed
by 90 fixed measurement processes: five blocks, three variants and separate time
and allocation modes. Every mathematical witness and representation inventory
matches. Builds finished before measurement, and each source has a separate
target and verified library/binary identity.

| Observed cost | Right scaling | Both directions |
| --- | ---: | ---: |
| Rows with reduced allocation fields, out of 68 | 6 | 18 |
| Rows with an increased allocation field in any sample | 0 | 0 |
| Timing rows faster in all five blocks, out of 68 | 6 | 13 |
| Timing rows slower in all five blocks | 1 | 1 |
| Probe executable increase | 504–528 bytes | 1,368–1,376 bytes |

Both save 2,000 allocations per 1,000 inputs in each of six opaque query rows.
Right scaling saves 192,000 requested bytes per row; both directions save
264,000. Retained requested bytes match in every measured phase. These counters
exclude native C heaps and do not establish eventual process reclamation.

The all-five slower rows are right `wide_integer_affine_cloned` (median block
ratio 1.00630) and both `wide_integer_affine_warm` (1.01641). Radical-Laurent query
rows are slower in four blocks for both variants, with ratios up to 1.19825 and
1.17512 respectively. Their median paired total differences are 1,350–2,700 ns
per 4,096 inputs. Other adverse rows, every sample and all ranges are retained in
[cost-assessment-v1.json](cost-assessment-v1.json). The sample set is closed;
variation is substantial and these are descriptive results.

The [source review and decision](scalar-decision-v1.json) explains why skipping
coefficient reinsertion preserves existing identities: atom comparison reads
immutable expression structure and exact leaves, with an iterative fallback
after its short recursive walk. It reads no warmed facts or approximation
caches. This is source reasoning with regressions, not a universal formal proof.

Each variant also passes 2,494 curve results (33 existing ignores), 1,644 DRC
results excluding the two separately executed timing assertions, strict native
consumer Clippy and warning-denied scalar/curve/DRC docs. Source and artifact
identity remain guarded throughout. See [native-assessment-v1.json](native-assessment-v1.json).

| Original assertion | Right scaling | Both directions |
| --- | ---: | ---: |
| Complete boards, worker CPU < 120 seconds | **Fail: 134.799982328 s** | **Fail: 137.672192623 s** |
| Copper-neck check body, wall time < 60 seconds | Pass | Pass |

Both complete boards emit their reports with 12 and 197 findings. A passing
copper-neck assertion does not print its measured interval; whole process
durations include setup and are not that metric. These separate gate runs do
not establish a matched application speedup or complete report identity.

The native controls completed all 30 endpoint timing/allocation processes,
15 broad processes and nine final-coordinate abort processes. Every endpoint
witness and inventory matches; all 231 broad witnesses match each other and the
historical record. Every final-abort process preserves the same 210 receipts.
See [native-controls-assessment-v1.json](native-controls-assessment-v1.json).

| Native cost observation | Right scaling | Both directions |
| --- | ---: | ---: |
| Endpoint timing rows slower in all five blocks, out of 48 | 10 | 13 |
| Endpoint timing rows faster in all five blocks | 0 | 0 |
| Broad rows slower in all five blocks, out of 231 | 18 | 15 |
| Broad rows faster in all five blocks | 8 | 12 |
| Endpoint rows with reduced allocation fields | 0 | 0 |
| Endpoint rows with increased peak requested bytes | 4 | 4 |

Allocation calls, requested bytes and retained requested-byte deltas match in
every native endpoint sample. The four opaque-translated phases each add 24
peak requested bytes in all five blocks. Examples of timing regressions include
right rational non-endpoint first queries (median block ratio 1.10644) and
bidirectional aborted-unknown cloned queries (1.10190). The historical abort
cost label refers to an intermediate signal; effective final-coordinate
abortion is checked by the separate receipt probe. All faster and slower rows
remain in the closed sample set; these are descriptive observations.

Complete report replay and production CLI size remain **unexecuted** for these
rejected sources. The native costs and original board failures settle rejection
before admission, so the prepared replay runner was not started. Finding counts
are not claimed as complete report identity.

The next diagnostic returns to the unchanged endpoint parent and counts actual
linear coefficient products discarded under an already stored zero rational
scale. It must preserve every operation and use only integer counters and
stored-sign reads. Coverage is required before proposing a runtime change;
neither rejected scaling branch is inherited. Both original limits, all earlier
tradeoffs and the full properties plan remain open.
