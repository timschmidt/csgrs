# Phase 17: first-proof views and wide dyadic add/sub scheduling

Captured 2026-08-04. This checkpoint follows the retained rational line-filter
evidence at Hyperreal `8e2ed531`, Hyperlimit `76281a34`, and Hypermesh
`35a92dcd`. The two retained Hyperreal refinements are `68c6b846` and
`3582c1e6`; Hyperlimit and Hypermesh production code are unchanged.

## Result

Two scalar-owned scheduling refinements are retained.

First, a directly proved certified rational binary64 view is now published
after its first successful proof. The former second-use observation bit and
branch are gone. Publication still requires the same direct bounded rational
conversion, finite/normal relative-error certificate, exact-zero rule, and
thread-safe primitive cache. A generic lossy cache cannot establish predicate
eligibility. This returns bit 30 to the dyadic-shift cache, raising its cached
maximum from 4,194,302 to 8,388,606 without changing exact behavior above the
cache limit. The implementation removes 15 net lines and one fact.

Second, reduced arbitrary-width dyadic addition and subtraction no longer
enter the general denominator-GCD schedule after the existing checked `u128`
path declines. The canonical `Rational` core now:

1. asks each operand for its reduced-only retained dyadic denominator shift;
2. aligns the two numerators to the larger power-of-two scale;
3. combines their signs exactly;
4. removes a possible common factor with trailing-zero shifts only; and
5. reuses the larger input denominator allocation, or returns through the
   existing reduced-word constructor when the result narrows.

Unequal reduced dyadic scales align an odd numerator with an even numerator,
so the result is already reduced. Equal scales may expose a shared power of
two, which is removed directly. A zero result returns canonical zero. Lazy
internally unreduced values, non-dyadic values, conversion-width failure, and
every other decline enter the unchanged arbitrary-precision GCD/reduction
path. The existing word route remains first.

This is one general scalar algorithm. It does not inspect a mesh, fixture,
coordinate-width label, triangle count, topology, Boolean operation, policy
name, expected output, benchmark, or competitor. It adds no compatibility
shim, mesh cache, retained field, dependency, Boolean engine, or hidden work
limit. Hypercurve and HyperSolve are excluded and untouched.

## Exactness and policy coverage

The permanent scalar matrix compares 288 wide signed add/sub cases against
independent fraction construction. It spans denominator shifts 0, 1, 127,
128, 255, and 300, both signs, equal and unequal scales, reduction across the
word boundary, and exact cancellation. A dispatch regression proves that one
wide dyadic add and one wide dyadic subtraction perform zero rational GCDs.
A lazy unreduced dyadic-looking operand is proved to decline this route and
enter the general GCD path with the same exact result.

The line-filter regressions continue to cover exact boundaries, direct-proof
eligibility, balanced 2,049-bit conversion decline, independent wrappers,
prewarmed generic caches, and concurrent publication across eight wrappers
and 512 calls.

Neither refinement creates a policy terminal. Every result is exact rational
arithmetic or certified filter evidence. `STRICT` remains exact-only.
`APPROXIMATE_512` still terminates only in Hyperlimit's existing 512-bit
terminal after all exact routes decline. Full and wide fixture outputs remain
policy-identical and `Certified`.

## First-proof publication measurement

Independent release targets compare Hyperreal `8e2ed531` with `68c6b846`.
Three pinned repetitions use instructions and branches as acceptance metrics.

| Workload | Two-use instructions | First-proof instructions | Movement | Branch movement |
| --- | ---: | ---: | ---: | ---: |
| Full rotated YeahRight, 23,788 triangles | 14,590,011,452 | 14,585,287,840 | -0.03238% | -0.03826% |
| 2,049-bit rational boxes, five unions | 14,489,100,600 | 14,489,075,372 | -0.00017% | +0.00149% |
| Clipped voxel torus 33, three all-result arrangements | 3,133,923,082 | 3,134,101,450 | +0.00569% | -0.02625% |
| Ordinary overlapping boxes, 1,000 all-result arrangements | 5,678,047,675 | 5,678,381,134 | +0.00587% | +0.01136% |

The simplification is retained because the difficult row improves, code and
native examples shrink by 16 text bytes, one fact bit is returned, and the
broad movements are bounded. Full and wide allocator counters are exactly
unchanged from the two-use implementation under both policies.

## Wide dyadic paired deterministic work

The accepted first-proof target is the parent for the wide dyadic comparison.
Parent and candidate use the same compiler, lockfiles, fixture archive,
release configuration, CPU 11, and three repetitions. Wall-clock frequency
variation is not an acceptance metric.

| Workload | First-proof instructions | Current instructions | Movement | Branch movement |
| --- | ---: | ---: | ---: | ---: |
| Full rotated YeahRight, 23,788 triangles | 14,585,266,608 | 14,312,073,897 | -1.87307% | -2.20790% |
| 2,049-bit rational boxes, five unions | 14,489,064,501 | 14,120,772,065 | -2.54186% | -3.23321% |
| Clipped voxel torus 33, three all-result arrangements | 3,131,393,482 | 3,132,907,967 | +0.04836% | +0.07073% |
| Ordinary overlapping boxes, 1,000 all-result arrangements | 5,677,617,753 | 5,676,900,903 | -0.01263% | -0.01977% |

Every output is identical. The small torus cost is retained and disclosed:
the 1.87–2.54% difficult/wide reductions and large allocation reductions
materially outweigh its 0.05% instruction check. No workload branch is used
to remove that cost.

## Large-fixture heap

The direct global-allocator probe excludes fixture construction from the
incremental Boolean boundary. Parent and current rows are exact across both
`STRICT` and `APPROXIMATE_512`.

| Fixture | Metric | First-proof | Current | Movement |
| --- | --- | ---: | ---: | ---: |
| Full rotated YeahRight | incremental peak | 158,258,204 B | 158,258,204 B | equal |
| Full rotated YeahRight | allocations | 16,928,390 | 16,146,116 | -782,274 (-4.6211%) |
| Full rotated YeahRight | reallocations | 2,385,300 | 2,294,770 | -90,530 (-3.7953%) |
| Full rotated YeahRight | added bytes | 913,360,240 B | 893,400,104 B | -19,960,136 B (-2.1854%) |
| 2,049-bit rational boxes | incremental peak | 31,234,658 B | 31,234,658 B | equal |
| 2,049-bit rational boxes | allocations | 2,092,630 | 1,947,527 | -145,103 (-6.9340%) |
| 2,049-bit rational boxes | reallocations | 227,677 | 223,172 | -4,505 (-1.9787%) |
| 2,049-bit rational boxes | added bytes | 459,671,086 B | 438,862,678 B | -20,808,408 B (-4.5268%) |

Full input payload, input-fact growth, output payload, and post-drop residual
are unchanged. Wide fixture construction benefits before the measured kernel
boundary and retains 1,488 fewer input payload bytes; its incremental peak and
26,568-byte input-fact growth remain unchanged. Current policy pairs are
byte-for-byte equal for every reported counter.

## Source and linked size

The first-proof refinement changes 16 inserted and 31 deleted lines. The wide
dyadic refinement adds 73 production lines and 102 exactness/dispatch test
lines. It adds no field or dependency. Paired clean source archives compare
Hyperreal `68c6b846` with `3582c1e6`; native values are `.text`, and WASM
values are `wasm-opt -Oz` bytes.

| Profile / consumer | First-proof native | Current native | First-proof WASM | Current WASM |
| --- | ---: | ---: | ---: | ---: |
| release / general | 1,966,486 | 1,969,526 (+3,040; +0.1546%) | 1,391,777 | 1,394,092 (+2,315; +0.1663%) |
| release / immediate | 1,969,614 | 1,972,670 (+3,056; +0.1552%) | 1,393,631 | 1,395,935 (+2,304; +0.1653%) |
| size / general | 1,071,431 | 1,072,255 (+824; +0.0769%) | 667,694 | 668,390 (+696; +0.1042%) |
| size / immediate | 1,072,395 | 1,073,219 (+824; +0.0768%) | 668,100 | 668,796 (+696; +0.1042%) |

The bounded linked growth is accepted because performance has priority, both
large allocation rows materially improve, and the code is one complete
representation-driven algorithm. Explicitly forcing a separate no-inline
boundary produced byte-identical example text and was removed as redundant.

## Dispatch and call graph

The permanent dispatch-trace suite reaches `rational/sub/wide-dyadic` 83
times on the control YeahRight arrangement. The focused scalar regression
proves the new add/sub route emits no GCD event, while the unreduced decline
does. No production counter or diagnostic branch is retained.

The regenerated five-crate source graph at
`/tmp/hypermesh-wide-dyadic-addsub-callgraph-2026-08-04` contains 14,848
function nodes and 24,747 edges:

| Crate | Nodes | Edges |
| --- | ---: | ---: |
| Hyperreal | 7,258 | 12,498 |
| Hyperlattice | 1,370 | 2,560 |
| Hyperlimit | 1,938 | 2,982 |
| Hypertri | 1,375 | 2,009 |
| Hypermesh | 2,913 | 4,668 |

Direct edges show only `Rational::add_ref` and `Rational::subtract_ref`
entering `Rational::add_sub_wide_dyadic`, which in turn reads the canonical
reduced-only dyadic-shift fact and existing constructors. Hypercurve and
HyperSolve are excluded. Production searches still find no EMBER route or
alternate Boolean engine.

## Validation

- Hyperreal passes 651 all-feature and 573 default unit tests, every
  integration/oracle test, and 24/19 doctests.
- Hyperlimit passes 154 all-feature and 144 default unit tests plus every
  integration test.
- Hyperlattice's complete suite passes. Hypertri passes 74 unit tests plus all
  integrations and doctests.
- Hypermesh passes 179 all-feature and 178 default executions; six documented
  manual/external tests remain ignored.
- Hyperreal, Hyperlimit, and Hypermesh warning-denied all-target/all-feature
  Clippy pass. Hyperreal and Hypermesh no-default checks and warning-denied
  rustdoc pass.
- Every Hypermesh fuzz binary checks, every benchmark target builds, format
  and diff checks pass, and both large fixtures pass both policies.

## Competitive status and open work

This checkpoint is gauged against the permanent historical/competitive
ledger without relabeling old measurements as current. The last pinned CGAL
6.0.3 EPECK comparison reports a 19.00x full-row runtime loss and 12.25x
fresh-process RSS loss, with ordinary-box losses of 4.81x under `STRICT` and
4.53x under `APPROXIMATE_512`. CGAL was not rerun for this scalar checkpoint,
so no runtime or RSS ratio is inferred from deterministic instruction work.

Current CGAL confidence runs, external real-world and deeper-symbolic fixture
families, sparse/multi-shell/pathological expansion, stage-specific arena and
retained-fact lifetime attribution, every remaining per-case runtime/RSS
gate, corpus completion, and the Phase 18 audit remain open.

## Reproduction

```sh
(cd ../hyperreal && cargo test --locked --all-features && cargo test --locked)
(cd ../hyperlimit && cargo test --locked --all-features && cargo test --locked)
(cd ../hyperlattice && cargo test --locked --all-features)
(cd ../hypertri && cargo test --locked --all-features)
cargo test --locked --all-features
cargo test --locked
cargo clippy --locked --all-targets --all-features -- -D warnings
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo bench --locked --no-run

YEAHRIGHT_BENCH=1 taskset -c 11 perf stat -r 3 -x, \
  -e instructions:u,branches:u \
  target/release/examples/large_mesh_heap_probe \
  yeahright-full-rotated strict
taskset -c 11 perf stat -r 3 -x, -e instructions:u,branches:u \
  target/release/examples/competitive_arrangement_probe \
  wide_rational_boxes_2048 union strict 5
target/release/examples/large_mesh_kernel_heap_probe \
  wide-rational-2048 approximate-512
benchmarks/size-harness/measure.sh default

(cd .. && tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir /tmp/hypermesh-wide-dyadic-addsub-callgraph-2026-08-04 \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format dot,json,mermaid,graphml \
  --per-library)
```

