# E36: exact KiCad track ingress remains unadmitted

The prototype repairs exact source strokes and native board aggregation, but
fails the unchanged full-board and copper-neck resource contracts. No production
source change is applied. The original 35-file bundle remains rejected; earlier
independently admitted components remain in place.

On parent `a843bf79dfe329fb51ababaaf803752bb74a0f64`, the original archived
regressions still produce 5 passes and 21 failures. Current typed-source tests
pass all 39 cases. The new source acceptance tests fail 10/10 on that parent;
their candidate v4 counterparts pass, together with 39 source-failure tests and
12 native-aggregation tests. The latter cover exact out-of-f64-range geometry,
empty composition, malformed operands, holes, mixed curve families, budgets,
source diagnostic history, authored report order, ownership and concurrent use.

Candidate v4 records 1675 passing native results with the three workload gates
executed separately. Gerber passes. Full-board fails at 396.946659071 worker CPU
seconds against 120; copper-neck fails at 133.165601323 check-body wall seconds
against 60. Both original complete projects and assertions are unchanged.
The full-board worker had already used 204.33 CPU seconds before profiling;
its final time is a diagnostic observation, not a clean benchmark. Copper-neck
was not profiled. Documentation and CLI builds pass. Strict Clippy fails on a
new explicit clone iterator. Complete commands and raw outcomes are retained.

The earlier v3 run was cancelled after observing 150.11 worker CPU seconds;
its final assertion did not run. The attempted v4 cancellation found that its
target process had naturally exited and sent no signals. The subsequent stack
snapshot observes a mechanical test in the broad suite, despite stale neck
attribution in its preparation metadata. These are distinct observations.

The candidate migrates finite rectangles to exact capsules and changes report
counts from 12/201 to 12/197. An independent audit of those report changes is
still required. Cost probes are oracle preflights, not admission benchmarks.
The failed attempts, partial preparations and immutable source snapshots are
preserved; completed Cargo targets must not be reopened.

Next, isolate native aggregation and readiness-check costs on the original
projects before choosing a kernel repair. No input reduction, weaker policy,
warning suppression or relaxed runtime limit is accepted. The shipped user
path and full properties plan remain open. The historical artifact audit still
has 27,093 missing paths; new evidence cannot certify that lost history.
