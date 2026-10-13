# Rational scaling inside normal-form coefficients

The two isolated variants pass scalar correctness and have repeatable allocation
savings. Both advance to native qualification; neither is admitted to runtime.
The parent endpoint trial and its earlier tradeoffs remain open.

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

Native consumer checks, the original complete-board and copper-neck limits,
fixed endpoint/broad controls, complete report identity and production binary
cost remain required. The full properties plan remains open.
