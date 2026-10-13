The remaining full-board cost is concentrated in scalar normal-form work.
This fresh profile of the frozen endpoint candidate records 6,786 samples,
zero lost samples and no empty stacks. Both original board reports complete,
but the original 120-second worker-CPU assertion fails at 135.955714978 seconds.
The separate release result remains 138.312559051 / 120 seconds; profiling
changes code generation and does not replace that result.

| Recovered work | Inclusive sampled cycle weight |
| --- | ---: |
| Polynomial normal form | 27.404% |
| Quadratic projection | 26.921% |
| Radical-coefficient multiplication | 9.227% |
| Endpoint membership | 6.100% |

These categories overlap. They describe sampled cycle periods, not invocation
counts, elapsed time or predicted savings. Coefficient multiplication appears
in 619 samples; the declared allocator-leaf subset below it accounts for 1.947%
of total profile weight. Its largest recovered curve callers are zero testing
and real comparison. A missing separately recovered cache-reader symbol does
not mean cache reads are free: inline frames were not decoded.

[The assessment](assessment-v1.json) verifies source, command, executable and
artifact hashes, including all 35 pending main-worktree entries. The build has
its own Cargo target and uses the uninstrumented candidate. The raw profile,
losslessly compressed logs, complete symbol tables and
[multiplication attribution](profile-v1/multiply-costs.json) are retained.

Source review identifies a narrow hypothesis: multiplying an owned coefficient
by a purely rational right coefficient currently reconstructs its vectors,
while a separate scaling helper already updates them in place. The profile
does not establish how often that shape occurs. The next diagnostic will count
existing left/right vector shapes without adding scalar queries, cloning
owners or changing arithmetic. Any subsequent trial must preserve term order,
opaque ownership, cache facts, mandatory original fallback, abort/budget
receipts and the original cost gates.

The rejected coefficient-merge and eight-node quadratic-prepass trials remain
rejected. No new arithmetic change is admitted by this profile. The prior
endpoint timing tradeoffs, retained-memory increases, broader qualification
and complete properties/future-work plan remain open.
