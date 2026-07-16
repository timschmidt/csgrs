# Performance Record

This record ties the retained implementation changes to reproducible release
benchmarks. Measurements are medians of five runs of:

```sh
cargo bench --bench replay_micro
```

The benchmark fixes the item count at 100 and uses exact integer `Real` values.
Absolute timings vary by host; the before/after comparison was collected on the
same machine and checkout. Release builds use the documented measurement
counts. Custom benchmark binaries also execute under `cargo test --all-targets`,
so debug builds use one smoke iteration. The aggregate `feasibility` benchmark
uses 10 release iterations and one debug iteration.

## Retained Changes

| Workload | Baseline median | Retained median | Change |
| --- | ---: | ---: | ---: |
| 3D replay, 100 placements | 1.220 ms | 0.855 ms | 29.9% faster |
| 2D replay, 100 placements | 1.173 ms | 0.770 ms | 34.4% faster |
| 1D replay, 100 placements | 1.176 ms | 0.742 ms | 36.9% faster |
| 3D zero-clearance replay, 100 placements | 4.494 ms | 4.201 ms | 6.5% faster |
| Prepare 100 unique demand classes | 2.463 ms | 0.936 ms | 62.0% faster |
| Prepare 100 repeated-size items | 1.817 ms | 0.750 ms | 58.7% faster |

Replay now resolves each placement's item once, then reuses an aligned item
schedule in the quadratic pair loop. This removes two ordered-map lookups per
pair without changing check counts, evidence order, or exact predicate
semantics. The same schedule is used by clearance replay.

Pair-incompatibility preparation now stops checking axes as soon as one axis
certifies that the pair can be separated inside the bin. The criterion is a
disjunction, so later axes cannot change that pair's classification. This is
the principal preparation win.

Item-map construction also checks map cardinality and returns
`PackError::DuplicateItem` for conflicting declarations. The normal unique-ID
path still performs the same collection followed by one length comparison.

## Exactness Check

With `dispatch-trace` enabled, `tests/dispatch_trace.rs` exercises exact
rational 3D replay and pair-incompatibility bounds. It requires nonzero exact
dispatch/reduction activity and zero approximation or unknown-fact events.
This guards the optimization boundary: less lookup and predicate work must not
be achieved by lowering exact decisions to primitive floating point.

## Rejected Experiments

- Reserving the exact maximum number of clearance pair-evidence records moved
  the median from about 4.067 ms to 4.094 ms (about 0.7% slower). The allocator's
  ordinary growth strategy was retained.
- Sorting items by dimensions before building prepared demand classes regressed
  unique-class preparation from about 2.46 ms to 2.60 ms and repeated-class
  preparation from about 1.82 ms to 2.06 ms. The extra exact comparisons and
  clones outweighed locality benefits.
- Deferring demand-class ID sorting until the end also regressed both measured
  workloads (about 0.7% and 3.2%). Incremental deterministic ordering was
  restored.
- Pairing bounded-search placements with the DFS item prefix instead of looking
  up their IDs appeared simpler, but the 12-item search median moved from about
  288.1 µs to 291.5 µs (about 1.2% slower). The lookup implementation was
  restored and the microbenchmark retained to catch future improvements.

## Reference Connection

The replay schedule preserves Yap's exact-predicate discipline while reducing
administrative lookup work. The incompatibility short circuit follows the
necessary-bound reasoning central to knapsack/bin-packing work by Martello,
Toth, Pisinger, and Vigo. The benchmark deliberately covers 1D, 2D, and 3D
surfaces from Dyckhoff's problem typology, and the rejected universal reorderings
are consistent with the portfolio lesson suggested by the no-free-lunch and
stochastic-local-search references: measure on the relevant workload and keep
only demonstrated improvements.
