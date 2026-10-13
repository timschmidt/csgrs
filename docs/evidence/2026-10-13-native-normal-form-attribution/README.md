The unchanged endpoint/P4 parent still spends substantial sampled work in scalar
normalization. Its single fixed profile records **7,071 samples, zero lost
samples and no empty stacks**. Unknown leaf symbols account for 0.327% of
sampled cycle weight. Both complete board reports finish with 12 and 197
findings; the original assertion fails at **141.705158972 worker-CPU seconds
against 120**. No runtime change is admitted.

| Recovered work | Inclusive share of total sampled cycle weight |
| --- | ---: |
| Polynomial normal form | 26.983% |
| Quadratic projection | 26.313% |
| Coefficient multiplication | 9.085% |
| Rational arithmetic inside normal form | 15.651% |
| Rational arithmetic inside coefficient multiplication | 8.186% |
| Declared allocator-leaf subset inside normal form | 7.893% |

These groups overlap. The exclusive partition assigns 17.055% of total weight
to normal-form stacks without recovered multiplication, 9.085% to normal-form
stacks containing coefficient multiplication, 0.844% to other recovered
polynomial multiplication within normal form, and 73.017% to other stacks.
Comparison and zero testing are the largest recovered direct native consumers
of normal-form work, at 7.247% and 6.064% of total weight respectively.

The [assessment](assessment-v1.json) verifies all six commands, the executable,
losslessly compressed logs, all 10,131 frozen parent files, the 1,134-file main
guard and all 35 pending native paths. The build emitted no compiler warnings.
The [complete attribution](profile-v1/observations.json), raw perf data and
source hashes are retained. The sample set is closed.

This build enables frame pointers and line tables. Its measurements do not
replace the parent's release failure at 133.31248412 worker-CPU seconds against
120. The separate copper-neck check-body gate was not rerun. Finding counts do
not establish complete report-byte identity. Sampled periods do not measure
invocation counts, wall time, duplicate requests or predicted savings; omitted
inline frames can hide work beneath other symbols. The older endpoint-only
profile is not a matched comparison.

The [decision](decision-v1.json) keeps the direct, outlined and borrowed
same-radicand helper series closed. Source review instead identifies a question
about eliminating an entire normalization pass: `immediate_sign` already
accepts a known sign in a retained root bound, while the unresolved exact-sign
walk calls quadratic normalization without consulting that bound at its final
fallback site. The profile does not reveal whether such a sign is available
there. Cached Unknown results and the direct predicate resolver already avoid
other repeated work; removing those protections is not proposed.

The [next diagnostic specification](next-diagnostic-v1.json) calls for fixed
counters at that existing fallback site. They will classify only existing
bound metadata, execute the original fallback, and record its existing result.
They must add no scalar queries, owners, cache writes or arithmetic changes.
Disagreements become correctness investigations; agreeing coverage alone does
not justify retention. No instrumented source or counter runner is prepared or
executed yet. Mandatory original retry, cache frontiers, proof receipts, P4
monotonicity, complete fixtures and both native limits remain requirements.

All 39 properties, 14 targets and crate future work remain in scope. The plan
is active and incomplete.
