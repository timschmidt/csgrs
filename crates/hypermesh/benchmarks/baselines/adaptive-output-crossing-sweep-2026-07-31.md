# Hypermesh adaptive output crossing-sweep checkpoint

Date: 2026-07-31

Direct Hypermesh parent:
`993a63176bae9e59146de5b8ea1239c2d0c216b3`

Hypermesh implementation:
`1ff2d3ede6a1874a6668c5f6521265cdd5e7bde5`

## Outcome

Symbolized profiling of the retained YeahRight arrangement showed that output
crossing discovery spent 4.92% of samples in
`split_edge_crossing_events`. The fixture has no proper crossing events, but
the fixed-X sweep still admitted a large candidate set before exact bounds
could prove that each pair was disjoint.

Crossing discovery now chooses its sweep axis from a bounded sample of the
already available exact-rational binary64 enclosures:

- fewer than 256 unique edges retain the previous X-axis path;
- larger sets sample at most 32 evenly spaced edges from the deterministic,
  sorted edge sequence;
- the selector counts pairwise outward-interval overlaps independently on X,
  Y, and Z;
- the axis with the fewest sampled overlaps wins, with a stable X/Y/Z tie;
  and
- the selected axis controls only approximate sorting and the conservative
  sweep break.

The selector has fixed stack storage, bounded work, no allocation, and no
retained carrier. It adds no compatibility entry point or policy-free path.

## Exactness and policy contract

Axis selection cannot certify geometry. An approximate sweep rejects a pair
only when outward enclosures prove exact separation. Every surviving pair
still reaches the existing exact edge-bound and proper-intersection predicates
through the operation's `DecisionContext`.

The enclosure sweep is available only when every output coordinate is an
exact rational with a finite enclosure. Symbolic and out-of-range coordinates
continue through the exact policy-aware X-axis ordering. Therefore:

- `STRICT` never consumes a terminal approximation;
- `APPROXIMATE_512` can consume Hyperlimit's 512-bit terminal only at the
  same canonical predicate boundary as before;
- sampling changes candidate volume and work order, never equality,
  incidence, intersection, or topology; and
- both policies retain identical certified output on every measured fixture.

The new unit regression constructs 256 edges whose X and Z intervals all
overlap while Y intervals are pairwise disjoint. It proves the selector chooses
Y at the threshold and preserves X immediately below it. Existing symbolic,
out-of-range, binary64-collapse, fixed-point repair, and both-policy regressions
remain green.

## Direct-parent CPU results

The parent and candidate use identical Hyperreal, Hyperlattice, Hyperlimit,
and Hypertri sources. Release probes were pinned to CPU 8 and serialized.
Retained and generated rows use 61 interleaved parent/candidate repetitions;
the construction-heavy 6,144-triangle box control uses 201 repetitions.

Each cell shows `parent -> candidate (movement)`.

| Fixture / policy | Task clock | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Retained / `STRICT` | 106.494 -> 104.269 ms (-2.0893%) | 408,703,809 -> 399,889,715 (-2.1566%) | 1,070,971,536 -> 1,017,500,709 (-4.9927%) | 180,842,014 -> 173,663,795 (-3.9693%) | 1,484,848 -> 1,429,215 (-3.7467%) | 1,659,088 -> 1,644,597 (-0.8734%) |
| Retained / `APPROXIMATE_512` | 93.314 -> 90.215 ms (-3.3210%) | 362,815,851 -> 349,792,138 (-3.5896%) | 1,070,978,869 -> 1,017,502,413 (-4.9932%) | 180,843,851 -> 173,664,098 (-3.9701%) | 1,459,307 -> 1,399,058 (-4.1286%) | 1,489,494 -> 1,461,908 (-1.8521%) |
| Generated 13,452-t / `STRICT` | 80.445 -> 80.434 ms (-0.0137%) | 287,644,537 -> 287,775,418 (+0.0455%) | 671,905,037 -> 667,461,588 (-0.6613%) | 102,376,323 -> 101,837,662 (-0.5262%) | 868,582 -> 858,596 (-1.1496%) | 1,900,102 -> 1,884,494 (-0.8214%) |
| Generated 13,452-t / `APPROXIMATE_512` | 82.973 -> 82.454 ms (-0.6255%) | 291,478,358 -> 289,603,158 (-0.6433%) | 671,917,071 -> 667,415,489 (-0.6700%) | 102,379,388 -> 101,826,382 (-0.5402%) | 871,539 -> 862,244 (-1.0665%) | 1,934,162 -> 1,892,741 (-2.1416%) |
| 6,144-t boxes / `STRICT` | 7.494 -> 7.499 ms (+0.0667%) | 15,948,034 -> 16,142,203 (+1.2175%) | 36,272,042 -> 36,321,576 (+0.1366%) | 6,656,344 -> 6,660,070 (+0.0560%) | 70,691 -> 70,583 (-0.1534%) | 112,676 -> 112,194 (-0.4284%) |
| 6,144-t boxes / `APPROXIMATE_512` | 7.426 -> 7.397 ms (-0.3905%) | 15,863,640 -> 15,815,679 (-0.3023%) | 36,272,112 -> 36,322,129 (+0.1379%) | 6,656,352 -> 6,660,152 (+0.0571%) | 70,617 -> 70,595 (-0.0314%) | 111,995 -> 113,734 (+1.5527%) |

The retained hard path improves every measured counter under both policies;
instructions fall 4.99%. The generated hard row executes 0.66-0.67% fewer
instructions, with strict cycles effectively flat and approximate cycles down
0.64%. The sub-threshold box probe is candidly mixed: instructions grow about
0.14%, strict cycles grow 1.22%, and approximate cycles fall 0.30%. Adaptive
sampling is not executed below the threshold, so this measures only the
fixed-X dynamic-axis branch and linked layout rather than a changed candidate
set. Performance has priority, so the large retained gain outweighs the small
mixed control.

## Large-fixture heap

Heaptrack includes fixture construction and the complete immediate union. The
table uses Heaptrack's recording summary for allocation/temporary counts and
`heaptrack_print` for peak heap. Counts are identical between `STRICT` and
`APPROXIMATE_512` on every candidate row.

| Fixture / revision | Allocations | Temporary | Peak heap | Candidate Heaptrack RSS | Output |
| --- | ---: | ---: | ---: | ---: | --- |
| Retained parent | 1,254,715 | 173,184 | 12.70 MiB | 22.32-22.35 MiB | 625 v / 1,246 t |
| Retained candidate | 1,247,977 (-0.5370%) | 172,658 (-0.3037%) | 12.70 MiB | 22.51-22.60 MiB | 625 v / 1,246 t |
| 6,144-t boxes parent | 27,211 | 81 | 4.70 MiB | - | 27 v / 50 t |
| 6,144-t boxes candidate | 27,211 | 79 | 4.70 MiB | 13.22-13.31 MiB | 27 v / 50 t |
| Generated 13,452-t parent | 304,568 | 27,058 | 11.66 MiB | - | 154 v / 304 t |
| Generated 13,452-t candidate | 303,000 (-0.5148%) | 27,004 (-0.1996%) | 11.66 MiB | 23.48-23.88 MiB | 154 v / 304 t |

The selector itself allocates nothing. Fewer admitted pairs avoid downstream
exact-rational temporaries on both crossing-heavy meshes. No peak-heap row
moves. RSS includes profiler overhead and is informative rather than a
retained-memory gate.

## Historical and competitive controls

The frozen historical retained row was 944.8 ms, 67.74 MiB peak heap,
5,020,891 allocations, and 82.5 MiB maximum RSS. The current paired strict
candidate is 88.96% faster, retains 81.25% less peak heap, and performs 75.14%
fewer allocations. The historical implementation materialized different
polygon output, so this is directional rather than a direct correctness A/B.

Fresh Criterion rows were pinned to CPU 8. These competitors are throughput
references; they do not provide Hypermesh's exact `Real`, explicit terminal
policy, or certified output contract.

| Union workload | Hypermesh | boolmesh | manifold-rust | Relative result |
| --- | ---: | ---: | ---: | --- |
| Overlapping 12-triangle boxes | 5.0185 us | 65.166 us | 57.624 us | Hypermesh 12.99x / 11.48x faster |
| 3,072-triangle boxes per operand | 1.9852 ms | 7.6112 ms | 4.3480 ms | Hypermesh 3.83x / 2.19x faster |
| Dyadic YeahRight 840-triangle hull + box | 13.263 ms | 0.76597 ms | 0.67798 ms | boolmesh 17.32x and manifold-rust 19.56x faster |

Against the previous stored Hypermesh Criterion checkpoint, the small row is
8.77% faster and the projective row is 3.98% faster. The large exact-cell row
is 2.34% slower in this cross-run comparison; the paired probe above is the
stronger incremental signal and reports +0.067% task clock with mixed counters.

## Native and WASM linked code

Native code is `.text`; WASM code is `wasm-opt -Oz`. Clean dependency-only
default-feature consumers compare the committed parent checkpoint with the
adaptive candidate.

| Consumer | Profile | Parent | Candidate | Movement |
| --- | --- | ---: | ---: | ---: |
| General native | Release | 4,020,108 | 4,021,580 | +0.0366% |
| General WASM | Release | 2,696,430 | 2,697,828 | +0.0518% |
| Immediate native | Release | 4,053,724 | 4,055,196 | +0.0363% |
| Immediate WASM | Release | 2,711,466 | 2,712,803 | +0.0493% |
| General native | Size | 1,845,258 | 1,846,370 | +0.0603% |
| General WASM | Size | 1,144,334 | 1,144,901 | +0.0495% |
| Immediate native | Size | 1,857,758 | 1,858,862 | +0.0594% |
| Immediate WASM | Size | 1,155,297 | 1,155,863 | +0.0490% |

The largest canonical linked-code increase is 0.0603%. This is retained for
the 4.99% retained-fixture instruction reduction; the selector introduces no
new retained data or heap allocation.

## Source and call graph

The implementation changes one source file by +97/-12 lines: +73/-12 in
production and +24 in the focused unit regression. The net production increase
is 61 lines.

| Scope | Parent | Candidate | Movement |
| --- | ---: | ---: | ---: |
| Isolated Hypermesh | 7,953 nodes / 19,563 edges | 7,963 / 19,581 | +10 / +18 |
| Five Hyper crates | 19,531 nodes / 39,025 edges | 19,535 / 39,024 | +4 / -1 |

The graph utility is syntactic; closures, tests, and alias resolution can move
its counts even when dynamic production reachability does not. The new
production ownership is limited to two private selector helpers and one
constant.

## Rejected alternatives

- Hard-coding Y or Z improved the retained case inconsistently and had no
  workload-independent justification; both variants were removed.
- Counting exact overlaps by fully sorting all three axes spent about 9.2
  million instructions before crossing discovery; it was removed.
- Sampling 64 edges chose worse axes on retained and generated controls and
  raised retained instructions relative to the 32-edge selector; it was
  removed.
- Retaining one approximate sweep interval in every `ExactEdgeBounds` reduced
  some instruction counts but enlarged the hot carrier and worsened retained
  cycles/task clock; it was removed.
- Const-specialized axis comparators, separate X-only paths, and duplicated
  comparator pairs all worsened code layout or controls while increasing
  source/native size; they were removed.

No rejected carrier, helper, diagnostic environment lookup, or compatibility
shim remains in the source.

## Validation

All completed successfully after retrying the repaired Hyperreal/Hyperlimit
stack:

```text
# hyperreal, hyperlattice, hyperlimit, hypertri, and hypermesh
cargo test --locked
cargo test --locked --no-default-features
cargo test --locked --all-features
cargo clippy --locked --all-targets --all-features -- -D warnings
cargo fmt --all -- --check
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features
cargo check --locked --manifest-path fuzz/Cargo.toml --bins

# hypermesh release surfaces
cargo bench --locked --no-run
cargo build --locked --manifest-path benchmarks/size-harness/Cargo.toml \
  --profile size --target wasm32-unknown-unknown
./benchmarks/size-harness/measure.sh default

# representative policy probes
target/release/examples/large_mesh_heap_probe boxes-3072 strict
target/release/examples/large_mesh_heap_probe boxes-3072 approximate-512
YEAHRIGHT_HULL_OBJ=/path/to/yeahright_boolean_hull.obj \
  target/release/examples/large_mesh_heap_probe yeahright strict
YEAHRIGHT_HULL_OBJ=/path/to/yeahright_boolean_hull.obj \
  target/release/examples/large_mesh_heap_probe yeahright approximate-512
YEAHRIGHT_BENCH=1 \
  target/release/examples/large_mesh_heap_probe yeahright-8 strict
YEAHRIGHT_BENCH=1 \
  target/release/examples/large_mesh_heap_probe yeahright-8 approximate-512

../tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . --crate-name hypermesh --out-dir /tmp/hypermesh-adaptive-callgraph
../tools/hyper-callgraph/target/release/hyper-callgraph \
  --root .. \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --out-dir /tmp/hypermesh-adaptive-callgraph-five
```

Hypermesh passes 1,051 default/no-default library tests and 1,052 all-feature
library tests. All measured outputs are certified. The retained fixture has
SHA-256
`5f1ac8a2f8bf2b0c67ce95dc8146cc783155dc9c34eeee164f03446a0830886c`
and is presented as 4,524 input triangles after exact subdivision.
