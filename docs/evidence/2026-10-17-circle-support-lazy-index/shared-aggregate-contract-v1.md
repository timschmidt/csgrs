# Follow-up for the short shared-support timing

The complete original 15-case comparison is preserved. Its 64-bit shared-center
warm phase has seven candidate samples above every baseline sample, with a
13.86% median increase. The phase is about half a millisecond. This follow-up
checks whether the unfavorable observation persists across grouped independent
processes before making a retention decision. It adds evidence; it does not
replace the original matrix or its larger coefficient fixtures.

Use the existing, hash-verified matched binaries without changing source,
features, policy, dependencies, query count, output materialization or oracle.
Each process constructs concentric radius-5 and radius-7 circles sharing the
exact center `(2^64 + 3, 0)`, performs one cold query and 20,000 warm queries,
checks every result is Disjoint, materializes every output row, and requires a
strict Certified receipt. There are no points satisfying both circle equations.

Collect seven matched groups, each containing eight fresh processes per variant.
Alternate variant order for successive pairs. A group sums the eight internally
measured warm phases: 160,000 complete queries per variant. These are aggregated
fresh-process phases, not one contiguous timed call. Preserve individual cold
and warm durations, rows, process resource observations and all commands.
Printing, startup, input construction and cleanup remain outside phase timers
and inside process observations, exactly as in the original probe.

Run only after both full-library processes and all other local qualification
commands are terminal. Use the existing external limits and reserves. Reject
this proposal as written if the grouped median exceeds baseline by more than
10%, all seven paired group differences are positive, and the candidate group
range lies wholly above baseline. Do not infer precise end-to-end speed factors
from this private-kernel measurement. A negative or noisy result cannot prove
zero overhead or satisfy any other open admission gate.
