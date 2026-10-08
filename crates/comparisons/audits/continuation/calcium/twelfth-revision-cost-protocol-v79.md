# Checkpoint 79: three-way native proof-cost comparison

Predeclared before timing collection. No production or previous-candidate edit.
Use the unchanged checkpoint 78 Rust harness, 96 inputs and 576 groups (two
precision budgets, three lifecycles). Compare live baseline, frozen candidate 77
(`prior`) and revision 79 (`candidate`). Require baseline/prior rebuilds to match
their frozen checkpoint 78 binaries byte-for-byte and verify all six executable
identities. Only two new frozen binaries; reuse the existing build cache.

Run every case; do not select only the previously bad tangent/cotangent groups.
Validate complete certificates against the original per-variant preflight and
require revised outputs to equal prior outputs. New exact answers versus baseline
are changed-work comparisons, not equal-work speedups.

Serialize processes pinned to CPU 2. Reuse the harness's four warmups. Pilot one
and sixteen iterations for all three variants; set the common iteration count
per group to ceil(2 ms / slowest of the six pilot per-query costs), bounded to
1..2000. Preserve all short/capped rows; the cap limits cold-pair setup memory.
Run 24 CPU triples, with all six variant permutations appearing four times and
their order randomized with the corrected AES rejection-sampled generator. Each
triple uses the same shuffled group order for all variants. Run six allocation
triples, all six permutations once, at both one and sixteen iterations. CPU and
counting-allocator binaries are separate. Record host snapshots before, between
and after phases; these are not proof of an idle or fixed-frequency host.

Recompute revised/baseline and revised/prior paired ratios from all raw rows.
Use the unchanged corrected 20,000-replicate bootstrap and independent median
order-statistic intervals. Both are conditional on sampling assumptions, not
multiplicity-adjusted. Perform statistical computation only after collection.
Allocation bytes/peak are allocator-level, exclude pair setup for cold/retained
queries, and are not RSS. Construction-inclusive means a warmed process, not
process startup. Report costs and changed answers even when unfavorable.

Expected records: 3,456 pilots + 41,472 CPU + 20,736 allocation = 65,664;
separate build preflight: 3,456 records. A source/hash/stream/chronology failure
invalidates qualification, regardless of an outer process's exit status.
No WASM runtime, consumer or representative linked-size claim follows from this
native campaign. Candidate retention remains a separate decision.
