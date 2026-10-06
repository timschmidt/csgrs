# Hypermesh policy-aware point-interner checkpoint

Date: 2026-07-30

Direct parent: `dc60a06c` (`Centralize halfspace policy decisions`)

## Change

One crate-private point/construction interner now owns the five former
deduplication implementations in mesh validation, convex-hull construction,
convex-polygon extraction, Boolean-mesh cleanup, and crossing-event insertion.

The exact-rational tier first recognizes retained storage, then uses a compact
full-rational fingerprint. Fingerprint collisions are chained and verified by
exact coordinate equality, so neither lossy projection nor a hash match can
merge unequal points. The general tier indexes certified dyadic coordinate
enclosures. A cell or interval miss rejects a candidate only when the
enclosures prove disjointness; every survivor reaches the centralized
policy-aware three-coordinate predicate. Candidate epochs bound the transient
candidate list to one index per retained point, including after `u32` wrap.

Retained construction identities are proof keys. Equal identities merge before
numeric evaluation, distinct retained identities are not collapsed by an
ambiguous coordinate predicate, and identity-free points remain eligible for
numeric merging. `STRICT` preserves an exhausted equality as
`PredicateUndecided`; `APPROXIMATE_512` may consume the terminal comparison and
marks the operation `Approximate512Consumed`.

Crossing-event insertion constructs the interner lazily on the first actual
crossing. No-crossing output therefore pays no indexing allocation. All
interner-owned growth and candidate accumulation use typed capacity failures.

## Correctness and path coverage

The focused regressions cover:

- exact values whose `f64` projections collide but whose rational values differ;
- equal rational values with distinct retained storage;
- exact-only construction followed by safe promotion to symbolic coordinates;
- certified interval pruning versus overlapping equivalent expressions;
- an undecided early candidate followed by a later structural match;
- construction-identity precedence over policy-aware numeric equality;
- candidate-epoch wrap; and
- strict versus approximate-512 terminal equality and aggregate certainty.

The complete library suite passed with 1,045 tests. The retained 17-by-17
crossing fixture still resolves all 289 proper crossings in one finite batch.

The final matrix also passed 1,046 all-feature library tests, all integration
and policy suites under all-feature and no-default configurations, all-target
checking, warnings-denied Clippy, warnings-denied rustdoc, all eight fuzz
targets, formatting, and `git diff --check`. Six external/manual competitive
stress tests and one benchmark-style regression remain explicitly ignored,
unchanged from the direct parent.

## Runtime

The ordinary exact-output and convex-hull controls remain within the paired
noise envelope:

| Row | Direct parent | Current | Center movement |
| --- | ---: | ---: | ---: |
| `output/cube_union_triangulate_certified` | 134.42 µs | 133.73 µs | -0.51% |
| `convex_hull/grid_4913` | 5.8565 ms | 5.8863 ms | +0.51% |

The output intervals were 134.07–134.83 µs and 133.40–134.05 µs. The final
serialized hull intervals were 5.8375–5.8752 ms and 5.8675–5.9050 ms;
Criterion reported no detectable change. A later output rerun was rejected
because the host load average exceeded 4 and 13% of samples were high outliers.

The fixed 289-crossing release test exposes the removed repeated equality scan:

| Counter, 20 repetitions | Direct parent | Current | Movement |
| --- | ---: | ---: | ---: |
| Instructions | 263,405,505 | 77,371,719 | -70.63% |
| Cycles | 81,441,968 | 25,798,023 | -68.32% |
| Task clock | 20.89 ms | 7.68 ms | -63.24% |
| Minor faults | 533 | 558 | +4.69% |

The test executables were pinned to CPU 8. Instructions are the stable
cross-host-load discriminator; time and cycles are retained as secondary
evidence.

## Memory

Heaptrack on the same fixed release test reports:

| Counter | Direct parent | Current | Movement |
| --- | ---: | ---: | ---: |
| Allocations | 39,195 | 39,211 | +16 (+0.041%) |
| Peak heap | 1.05 MB | 1.18 MB | +0.13 MB |
| Peak RSS, including Heaptrack | 11.05 MB | 10.90 MB | -0.15 MB |
| Heaptrack runtime | 0.076 s | 0.062 s | -18.4% |

The additional peak heap is the deliberately retained exact index for the
crossing batch. It is transient and released with output resolution. The
checkpoint is therefore a Pareto tradeoff, not an unconditional memory win.
The general tier's epoch array costs four bytes per indexed point but prevents
up to 64 duplicate candidate entries per point; the measured exact-only
crossing path does not allocate that array.

## Native and WASM linked code

Native code is `.text`; WASM code is `wasm-opt -Oz`. Both the default and
all-feature dependency-only consumers were built with release and size
profiles.

| Features | Consumer | Profile | Target | Parent code | Current code | Movement |
| --- | --- | --- | --- | ---: | ---: | ---: |
| Default | General | Release | Native | 3,710,877 | 3,722,325 | +0.3085% |
| Default | General | Release | WASM | 2,601,131 | 2,612,055 | +0.4200% |
| Default | Immediate | Release | Native | 3,744,701 | 3,755,597 | +0.2910% |
| Default | Immediate | Release | WASM | 2,616,379 | 2,627,138 | +0.4112% |
| Default | General | Size | Native | 1,678,963 | 1,692,315 | +0.7953% |
| Default | General | Size | WASM | 1,091,463 | 1,100,485 | +0.8266% |
| Default | Immediate | Size | Native | 1,690,951 | 1,704,215 | +0.7844% |
| Default | Immediate | Size | WASM | 1,101,285 | 1,111,220 | +0.9021% |
| All | General | Release | Native | 3,841,826 | 3,853,346 | +0.2999% |
| All | General | Release | WASM | 2,679,419 | 2,690,338 | +0.4075% |
| All | Immediate | Release | Native | 3,875,898 | 3,887,162 | +0.2906% |
| All | Immediate | Release | WASM | 2,694,985 | 2,705,768 | +0.4001% |
| All | General | Size | Native | 1,679,211 | 1,692,587 | +0.7966% |
| All | General | Size | WASM | 1,088,626 | 1,098,021 | +0.8630% |
| All | Immediate | Size | Native | 1,691,175 | 1,704,439 | +0.7843% |
| All | Immediate | Size | WASM | 1,098,762 | 1,108,100 | +0.8499% |

Every row stays below the one-percent implementation-checkpoint gate. This is
not the final Phase 8 size gate: the remaining 9–13 KiB of linked code must be
recovered by later consolidation or explicitly justified as the final Pareto
point.

## Source and call graph

The migrated files remove 516 lines and add 111. The shared module contains
595 production lines and 235 test lines, for a net change of +190 production
lines and +425 total lines at this checkpoint.

The source call graph moves from 7,679 nodes and 19,035 edges to 7,736 nodes
and 19,134 edges after the two rare-path regressions. The syntactic graph grows
because the proof tiers and tests are explicit, but the old position bucket,
exact-output merger, output bucket, mesh-local certified-cell index, and linear
crossing insertion functions are gone. Five consumers now enter one
policy/termination spine. Phase 6 source-reduction work remains open.

## Reproduction

```sh
cargo test --locked --lib --no-fail-fast
taskset -c 8 perf stat -x, -r 20 \
  -e task-clock,cycles,instructions,minor-faults -- \
  target/release/deps/hypermesh-a2607ff55afe136b \
  --exact output::tests::crossing_discovery_batches_more_than_the_historical_pass_limit
heaptrack --record-only -o /tmp/hypermesh-point-interner-current.heaptrack \
  target/release/deps/hypermesh-a2607ff55afe136b \
  --exact output::tests::crossing_discovery_batches_more_than_the_historical_pass_limit
./benchmarks/size-harness/measure.sh default
./benchmarks/size-harness/measure.sh all
tools/hyper-callgraph/target/release/hyper-callgraph \
  --root hypermesh \
  --out-dir /tmp/hypermesh-point-interner-current-callgraph \
  --crate-name hypermesh \
  --per-library \
  --format json
```

Machine-readable values and artifact hashes are in
`point-interner-2026-07-30.toml`.
