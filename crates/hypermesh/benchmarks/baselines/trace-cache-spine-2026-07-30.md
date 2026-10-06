# Hypermesh definition-reachability cache-spine checkpoint

Date: 2026-07-30

Direct parent: `cced5ea3` (`Consolidate policy-aware point interning`)

## Change

Definition reachability had two byte-for-byte-equivalent plain cache types,
two byte-for-byte-equivalent cycle-guard cache types, two plain lookup/begin
implementations, two cycle-guard lookup/begin implementations, and two
closure wrappers. The duplicated names described the caller's search strategy,
but their keys, reversal rules, in-progress sentinel, result semantics, and
storage layout were identical.

The trace engine now owns one `DefinitionReachabilityCache`, one
`DefinitionCycleGuardReachabilityCache`, and one lookup/begin path for each
shape. No alias, compatibility wrapper, or old implementation remains.
No-step, full-no-detour, no-plane-replacement, and detour-leg strategies still
own separate cache instances. The instance-local regression stores opposite
results for identical geometry in two owners and proves that the result sets
cannot leak through shared implementation.

The consolidation does not change predicate evaluation. Cache keys remain
exact `Real` points plus set-equivalent plane-definition families. Cycle-guard
keys still normalize visited definition points, reuse exact states, and apply
the same proven true-subset/false-superset implications. An in-progress query
still exposes `UnknownClassification`, so recursive search cannot mistake a
placeholder for a certified terminal.

## Correctness and path coverage

The focused cache suite covers identical, permuted-family, reversed-direction,
in-progress, normalized-endpoint, true-subset, false-superset, and
instance-local behavior. The complete all-feature suite passed with 1,043
library tests; the no-default suite passed with 1,042. The integration corpus
also passed under both configurations, including both-policy terminal tests,
general and exact-cell Booleans, trace/subdivision regressions, competitive
differentials, deterministic README examples, and closure validation.

Warning-denied all-target Clippy passed with all features and no default
features. Warning-denied rustdoc, every fuzz binary, formatting, and
`git diff --check` passed. The dispatch-trace executable retained a nonempty
`segment_and_winding` recording and the larger cube workloads retained
`breadth-first-detour-trace` events.

## Runtime

Two serialized, CPU-8, parent/current Criterion pairs exercised the
non-analytic overlapping-octahedron union. Both pairs stayed inside the
configured two-percent noise gate:

| Pair | Parent | Current | Center movement |
| --- | ---: | ---: | ---: |
| 1 | 2.8453 ms | 2.8352 ms | -0.35% |
| 2 | 2.8762 ms | 2.8583 ms | -0.62% |

A fixed release probe ran the same certified octahedron union 256 times.
Ten `perf stat` repetitions reported:

| Counter | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| Instructions | 6,276,869,737 | 6,277,526,527 | +0.0105% |
| Cycles | 3,033,975,073 | 3,049,724,526 | +0.52% |
| Task clock | 720.29 ms | 724.93 ms | +0.64% |
| Branches | 1,132,940,834 | 1,132,988,952 | +0.0042% |
| Branch misses | 25,557,769 | 25,337,806 | -0.86% |

The instruction delta is nearly one part in ten thousand and is treated as
code-layout noise, not a runtime improvement or regression.

## Memory

Heaptrack over the same 256-operation probe is heap-neutral; its 0.06 MiB RSS
movement is below profiler/process-layout noise:

| Counter | Parent | Current |
| --- | ---: | ---: |
| Allocations | 3,944,566 | 3,944,566 |
| Temporary allocations | 38,146 | 38,146 |
| Peak heap | 471.83 KiB | 471.83 KiB |
| Peak RSS, including Heaptrack | 9.20 MiB | 9.14 MiB |
| Leaked process-lifetime memory | 126.59 KiB | 126.59 KiB |

The common types have the same layout as the removed types, and every semantic
owner remains a separate instance, so neither retained nor transient cache
memory grows.

## Historical and competitive controls

The overlapping-box exact-cell control measured 4.9170 us in this frozen
dependency snapshot, versus 66.344 us for boolmesh and 59.579 us for
manifold-rust on the same throughput corpus. Hypermesh is therefore 13.49x and
12.12x faster, respectively, and 125.81x faster than the original 618.60 us
historical Hypermesh row. This analytic control does not exercise the changed
cache spine; it verifies that consolidation did not disturb the retained
competitive fast path.

## Source and call graph

Excluding five pre-existing formatting-only test hunks, the core consolidation
removes 609 lines and adds 212. The retained 29-line release probe makes the
checkpoint's total source movement -368 lines: production library source falls
by 274 lines, duplicated tests fall by 123 lines, and reproducible benchmark
support adds 31 lines.

The source call graph moves from 7,736 nodes and 19,134 edges to 7,716 nodes
and 19,064 edges. The strategy-specific plain and no-plane-replacement
lookup/begin/wrapper nodes are gone. All remaining callers enter the common
plain or cycle-guard spine.

## Native and WASM linked code

Native code is `.text`; WASM code is `wasm-opt -Oz`. Parent and current builds
used frozen source and dependency snapshots.

| Features | Consumer | Profile | Parent | Current | Movement |
| --- | --- | --- | ---: | ---: | ---: |
| Default | General native | Release | 3,727,325 | 3,722,117 | -0.1397% |
| Default | General WASM | Release | 2,616,459 | 2,615,280 | -0.0451% |
| Default | Immediate native | Release | 3,760,581 | 3,755,381 | -0.1383% |
| Default | Immediate WASM | Release | 2,631,476 | 2,630,366 | -0.0422% |
| Default | General native | Size | 1,695,299 | 1,694,939 | -0.0212% |
| Default | General WASM | Size | 1,103,049 | 1,102,521 | -0.0479% |
| Default | Immediate native | Size | 1,707,223 | 1,706,839 | -0.0225% |
| Default | Immediate WASM | Size | 1,113,204 | 1,112,674 | -0.0476% |
| All | General native | Release | 3,859,530 | 3,854,346 | -0.1343% |
| All | General WASM | Release | 2,695,379 | 2,694,197 | -0.0439% |
| All | Immediate native | Release | 3,893,346 | 3,888,162 | -0.1332% |
| All | Immediate WASM | Release | 2,710,804 | 2,709,631 | -0.0433% |
| All | General native | Size | 1,695,579 | 1,695,195 | -0.0226% |
| All | General WASM | Size | 1,100,611 | 1,099,685 | -0.0841% |
| All | Immediate native | Size | 1,707,455 | 1,707,111 | -0.0201% |
| All | Immediate WASM | Size | 1,110,668 | 1,109,628 | -0.0936% |

All sixteen linked-code rows shrink. Optimized-WASM compression is mixed in
three sub-0.15% rows because the smaller bytecode changes compressor layout;
the final packaging gate still owns compressed-size closure. The trace-heavy
probe itself loses 6,896 native `.text` bytes (-0.16%).

## Reproduction

```sh
cargo test --locked --all-features --no-fail-fast
cargo test --locked --no-default-features --no-fail-fast
cargo clippy --locked --all-targets --all-features -- -D warnings
cargo clippy --locked --all-targets --no-default-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo bench --locked --bench dispatch_trace --features dispatch-trace

cargo build --locked --release --example trace_cache_probe
taskset -c 8 perf stat -x, -r 10 \
  -e task-clock,cycles,instructions,branches,branch-misses,minor-faults -- \
  target/release/examples/trace_cache_probe 256
heaptrack --record-only target/release/examples/trace_cache_probe 256

./benchmarks/size-harness/measure.sh default
./benchmarks/size-harness/measure.sh all

tools/hyper-callgraph/target/release/hyper-callgraph \
  --root hypermesh \
  --out-dir /tmp/hypermesh-callgraph-cache-spine \
  --crate-name hypermesh \
  --per-library \
  --format json
```

Machine-readable values and artifact hashes are in
`trace-cache-spine-2026-07-30.toml`.
