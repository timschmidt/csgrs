# Hypermesh leaf-probe cache-ownership checkpoint

Date: 2026-07-30

Direct parent: `90a75315` (`Consolidate definition reachability cache spine`)

## Change

The leaf-probe path previously destructured and forwarded roughly twenty
mutable cache references through several positional call boundaries. Many of
those caches have identical Rust types but intentionally represent different
search strategies. A swapped argument could therefore compile while allowing
one strategy's result to answer another strategy's query.

`LeafProbeQueryCaches` now owns one nested `AdjacentCellQueryCaches` value.
Adjacent-cell reachability, no-step boundary reachability, progressive normal
probes, and ordinary leaf probes borrow that aggregate instead of forwarding
long positional lists. The aggregate contains the existing independent cache
instances; it does not merge semantically distinct result sets.

`evaluate_leaf_probe_with_query_caches` also receives the owning leaf cache
aggregate and reuses the existing certified adjacent-probe winding entry
point. An unused `positive_side` argument and its forwarding chain are gone.
No alias, compatibility shim, deprecated entry point, or parallel old
implementation remains.

## Policy and exactness

This checkpoint changes ownership and call shape, not predicates or cache
semantics. Cache keys, exact `Real` coordinates, in-progress sentinels,
reversal rules, certified subset implications, and result values are
unchanged. Every call remains under the immutable operation `MeshContext`.

`STRICT` still rejects an unresolved terminal and
`APPROXIMATE_512` still reaches Hyperlimit's terminal 512-bit equality/sign
interpretation only after certification is exhausted. The same aggregate
certainty is returned by the canonical Boolean APIs. Moving the caches cannot
upgrade approximate evidence or make it visible to a strict operation.

## Correctness and path coverage

The complete all-feature library suite passed all 1,043 tests. The complete
integration corpus passed, including the eight explicit both-policy tests,
general leaf and subdivision paths, exact-cell and projective paths, closure
validation, competitive differentials, and README examples.

The corresponding 1,042-test no-default suite, warning-denied full/minimal
Clippy, warning-denied rustdoc, every fuzz binary, and the dispatch-trace
executable also passed. The trace retained `segment_and_winding` and
`breadth-first-detour-trace` events.

## Source and call graph

Excluding five pre-existing formatting-only test hunks, the checkpoint adds
245 lines and removes 455, for a net reduction of 210 lines. Production
library source falls by 217 lines and duplicated tests fall by 59. The
66-line large-fixture heap probe is the only net benchmark-support addition;
three existing benchmark helpers merely become visible to that private example
module.

The library source graph moves from 7,716 nodes and 19,064 edges to 7,723 nodes
and 19,056 edges. The seven syntactic nodes added by aggregate `Default` and
field-qualified test access replace eight call edges. The high-arity leaf
boundaries disappear without adding a semantic execution path.

## Trace-heavy runtime

A fixed release probe performs 256 non-analytic overlapping-octahedron unions.
Two serialized CPU-8 parent/current pairs, each containing ten `perf stat`
repetitions, reported:

| Counter | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| Task clock | 725.130 ms | 718.265 ms | -0.947% |
| Cycles | 3,052,769,111 | 3,025,037,744 | -0.908% |
| Instructions | 6,287,068,399 | 6,286,429,293 | -0.0102% |
| Branches | 1,135,839,940 | 1,135,754,902 | -0.0075% |
| Branch misses | 25,701,491 | 25,304,425 | -1.545% |

The instruction result is effectively neutral. The small time/cycle movement
is favorable but is not treated as an algorithmic speedup.

Two reverse-order Criterion pairs exercised the same octahedron union:

| Pair | Parent | Current | Center movement |
| --- | ---: | ---: | ---: |
| 1 | 2.8240 ms | 2.8489 ms | +0.882% |
| 2 | 2.8468 ms | 2.8327 ms | -0.495% |

The combined centers are 2.8354 and 2.8408 ms, a neutral +0.191%.

## Large-fixture runtime and heap

The checked-in `large_mesh_heap_probe` measures two deliberately different
large paths:

- 3,072 triangles per overlapping box, or 6,144 total input triangles, covers
  the specialized certified-convex/axis-aligned path.
- The retained YeahRight arrangement uses the historical 1,128-facet hull
  plus a 12-facet box. Exact midpoint subdivision presents 4,524 triangles
  while preserving the 1,140 exact supporting facets. This covers the
  projective convex path, exact output closure, and T-junction resolution.

Both revisions produce 27 vertices/50 triangles on the large boxes and
625 vertices/1,246 triangles on the retained YeahRight union.

Heaptrack is exactly neutral on every allocation measure:

| Fixture | Revision | Allocations | Temporary | Peak heap | Heaptrack RSS |
| --- | --- | ---: | ---: | ---: | ---: |
| Boxes, 6,144 triangles | Parent | 28,156 | 80 | 4.72 MiB | 10.53 MiB |
| Boxes, 6,144 triangles | Current | 28,156 | 80 | 4.72 MiB | 12.63 MiB |
| YeahRight, 4,524 triangles | Parent | 41,757,532 | 7,636,311 | 12.77 MiB | 22.14 MiB |
| YeahRight, 4,524 triangles | Current | 41,757,532 | 7,636,311 | 12.77 MiB | 22.20 MiB |

The boxes row is too short for its profiled RSS to be stable. On the
YeahRight row, unprofiled fresh-process RSS was 18,404 KiB parent and
18,644 KiB current. Neither RSS movement indicates retained-cache growth; the
allocation count and live-heap high-water mark are identical.

Two reverse-order five-run `perf stat` pairs on the retained fixture found
instruction-neutral execution but a linked-layout-sensitive time boundary:

| Counter | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| Task clock | 1,719.60 ms | 1,770.68 ms | +2.970% |
| Cycles | 7,158,174,966 | 7,383,026,532 | +3.141% |
| Instructions | 23,421,401,269 | 23,421,381,557 | -0.000084% |
| Branches | 3,902,960,862 | 3,902,956,757 | -0.000105% |
| Branch misses | 25,126,599 | 24,955,849 | -0.680% |

The semantic work counters and allocation trace are unchanged. The time row
sits at the three-percent large-workload gate and is retained as a measured
code-layout cost, not described as neutral noise. The next profile-driven
phase must recover it.

## Historical and competitive controls

The retained historical YeahRight gate was 944.8 ms, 67.74 MiB peak heap,
5,020,891 allocations, and 82.5 MiB maximum RSS. The current immediate mesh
pipeline uses 12.77 MiB peak heap, 81.15% below that heap ceiling and already
well below the plan's first 64 MiB target. Its 41,757,532 allocation calls are
731.68% above the historical count, and its 1,770.68 ms task-clock average is
87.41% above the historical time.

Those historical runtime and allocation comparisons are directional rather
than a frozen A/B: the old result recorded 5,152 classified polygons, whereas
the current canonical immediate API closure-certifies and resolves a
1,246-triangle mesh. The heap ceiling is nevertheless an end-to-end safety
gate and is passed with substantial margin.

Heaptrack identifies the remaining large-row cost rather than the ownership
refactor: 17,277,197 allocation calls arise in BigUint left shifts and
8,436,732 in BigUint division. Their stacks run through exact signed-product
sums in output crossing/T-junction resolution and Boolean mesh closure. These
are explicit Phase 7 targets; policy or exact closure checks may not be
weakened to remove them.

The frozen overlapping-box competitive control remains 4.917 us for
Hypermesh, 66.344 us for boolmesh, and 59.579 us for manifold-rust. Hypermesh
is 13.49x and 12.12x faster, respectively, and 125.81x faster than the
historical 618.60 us Hypermesh row.

## Native and WASM linked code

Native code is `.text`; WASM code is `wasm-opt -Oz`. All sixteen canonical
consumer rows shrink:

| Features | Consumer | Profile | Parent | Current | Movement |
| --- | --- | --- | ---: | ---: | ---: |
| Default | General native | Release | 3,739,125 | 3,733,357 | -0.1543% |
| Default | General WASM | Release | 2,632,311 | 2,630,695 | -0.0614% |
| Default | Immediate native | Release | 3,772,389 | 3,766,629 | -0.1527% |
| Default | Immediate WASM | Release | 2,647,396 | 2,645,782 | -0.0610% |
| Default | General native | Size | 1,704,315 | 1,701,275 | -0.1784% |
| Default | General WASM | Size | 1,111,161 | 1,109,838 | -0.1191% |
| Default | Immediate native | Size | 1,716,255 | 1,713,175 | -0.1795% |
| Default | Immediate WASM | Size | 1,121,456 | 1,120,006 | -0.1293% |
| All | General native | Release | 3,873,978 | 3,869,306 | -0.1206% |
| All | General WASM | Release | 2,712,359 | 2,710,742 | -0.0596% |
| All | Immediate native | Release | 3,907,794 | 3,903,122 | -0.1196% |
| All | Immediate WASM | Release | 2,727,805 | 2,726,170 | -0.0599% |
| All | General native | Size | 1,704,595 | 1,701,467 | -0.1835% |
| All | General WASM | Size | 1,108,352 | 1,107,473 | -0.0793% |
| All | Immediate native | Size | 1,716,487 | 1,713,375 | -0.1813% |
| All | Immediate WASM | Size | 1,118,343 | 1,117,470 | -0.0781% |

The trace-heavy release probe also falls by 7,212 native `.text` bytes
(-0.1669%) and 8,648 raw file bytes (-0.1575%).

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

cargo build --locked --release --example large_mesh_heap_probe
heaptrack --record-only \
  target/release/examples/large_mesh_heap_probe boxes-3072

git archive --format=tar --output=/tmp/yeahright-retained.tar \
  575c4d0a competitive/data/yeahright_boolean_hull.obj
tar -xf /tmp/yeahright-retained.tar -C /tmp
YEAHRIGHT_HULL_OBJ=/tmp/competitive/data/yeahright_boolean_hull.obj \
  heaptrack --record-only \
  target/release/examples/large_mesh_heap_probe yeahright

./benchmarks/size-harness/measure.sh default
./benchmarks/size-harness/measure.sh all

tools/hyper-callgraph/target/release/hyper-callgraph \
  --root hypermesh \
  --out-dir /tmp/hypermesh-callgraph-leaf-probe \
  --crate-name hypermesh \
  --per-library \
  --format json
```

The retained fixture blob has SHA-256
`5f1ac8a2f8bf2b0c67ce95dc8146cc783155dc9c34eeee164f03446a0830886c`.
Machine-readable values and artifact hashes are in
`leaf-probe-cache-ownership-2026-07-30.toml`.
