# Preserved grouped shared-support comparison

This predeclared follow-up tests the small-hit revision against the production
baseline using the same seven groups as E44. E44's grouped median increase was
9.9449548%, with seven positive paired differences and separate ranges; that
was below the strictly-greater-than-10% decisive rejection threshold. Keep
both the old observation and every new sample, favorable or unfavorable.

Use the existing, hash-verified matched binaries without changing source,
features, policy, dependencies, query count, output materialization or oracle.
Each process constructs concentric radius-5 and radius-7 circles sharing the
exact center `(2^64 + 3, 0)`, performs one cold query and 20,000 warm queries,
checks every result is Disjoint, materializes every output row, and requires a
strict Certified receipt. There are no points satisfying both circle equations.

Collect seven matched groups, each containing eight fresh processes per variant.
Alternate variant order for successive pairs. A group sums eight internally
measured warm phases: 160,000 complete queries per variant. These are aggregated
fresh-process phases, not one contiguous timed call. Preserve individual cold
and warm durations, rows, process resource observations and all commands.
Printing, startup, input construction and cleanup remain outside phase timers
and inside process observations, exactly as in the original probe.

Run only after both full-library processes and all other local qualification
commands are terminal. Use this experiment's quota-aware resource supervisor.
Reject the proposal as written if the grouped median exceeds baseline by more
than 10%, all seven paired group differences are positive, and the candidate
group range lies wholly above baseline. Do not infer precise end-to-end speed
factors from this private-kernel measurement. A negative or noisy result cannot
prove zero overhead or satisfy any other open admission gate.
