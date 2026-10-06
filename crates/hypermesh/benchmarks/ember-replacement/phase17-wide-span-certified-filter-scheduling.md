# Phase 17 wide-span certified-filter scheduling

Date: 2026-08-05

Status: accepted performance checkpoint; Phases 11, 17, and 18 remain open

Implementation: Hyperreal `14258dc`, Hypermesh `a9c1a00b`

Direct parent/evidence: Hyperreal `3582c1e6`, Hypermesh `fa7bcedf`

## Result

The 512-bit thin-dyadic corpus exposed two independent representation
boundaries that multiplied conservative work without contributing a topology
decision:

1. Hypermesh's 24-byte binary32 AABB carrier remained conservative below the
   normal binary32 range, but many distinct exact endpoints collapsed into a
   handful of subnormal buckets. The resulting broad phase admitted 261,232
   candidate pairs instead of the 69,712 pairs admitted by the exact borrowed
   bounds.
2. Hyperreal's rational four-lane linear-form filter rejected every normalized
   lane more than 500 exponents below its sibling, even when most products were
   safely representable. Those queries fell through to materially more exact
   word and arbitrary-precision product sums.

Hypermesh now marks the compact binary32 filter unavailable whenever a finite,
nonzero binary64 enclosure endpoint lies below `f32::MIN_POSITIVE`. Exact zero
remains representable. The existing BVH traversal then uses the already-owned
exact bounds and the canonical policy-aware overlap predicate. No additional
storage, tree, candidate representation, or fallback engine is introduced.

Hyperreal now retains every normalized nonzero lane that remains a normal
binary64 value. Its certified sign radius combines the existing relative
`82 * eps * magnitude_sum` proof with an absolute
`16 * f64::MIN_POSITIVE` floor. The floor covers at most four underflowing
products and seven rounded sum operations with margin; away from the underflow
boundary it is negligible relative to the existing normal-scale proof. A query
that cannot exclude zero still declines into the unchanged complete exact
product-sum cascade.

These are format- and proof-driven rules. Neither implementation inspects an
exponent span value, fixture name, mesh size, Boolean operation, result,
topology, policy name, competitor, or benchmark. There is no shift-512 branch,
expected-result shortcut, compatibility shim, alternate engine, or incomplete
algorithm. The retained form was selected only after broad exact, symbolic,
heap, size, and competitive controls.

## Exactness and policy

- The rational normalization oracle covers all 2,046 normal binary64 exponents
  and spans including 501, 511, 512, and 1,022.
- A generated exact-rational oracle compares 2,048 wide-exponent queries at
  spans 501, 512, 900, and 1,022 with
  `Rational::signed_product_sum_ordering`.
- A 501-bit safe pair is certified directly. A 512-bit pair whose only nonzero
  product underflows is deliberately declined by the absolute error floor.
- Binary64 zero/subnormal normalization failures and unsafe raw filters remain
  declined; no rounded sign is accepted.
- The binary32-bound regression uses exactly separated `2^-512` AABBs. The
  compact filter returns unavailable, and the borrowed exact overlap result is
  false and `Certified` under both `STRICT` and `APPROXIMATE_512`.
- Every thin-dyadic result remains 2,410 vertices and 4,816 union triangles.
  Both policies remain byte-identical and `Certified`; no terminal
  approximation is consumed.
- `STRICT` therefore remains exact-only. `APPROXIMATE_512` may still terminate
  only through Hyperlimit's terminal 512-bit policy, with aggregate certainty
  absorbed by the operation context.

## Deterministic thin-dyadic performance

CPU-11-pinned `perf stat -r 3` wraps one fresh release process and one union.
Setup is included. The parent is the committed `fa7bcedf` corpus baseline; the
current rows use the final source after both retained changes.

| Shift / `STRICT` | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change | Parent cycles | Current cycles | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 64 | 1,194,400,383 | 1,195,648,142 | +0.1045% | 205,221,436 | 205,357,311 | +0.0662% | 457,250,995 | 458,731,594 | +0.3238% |
| 512 | 3,429,452,625 | 1,644,137,045 | -52.0583% | 590,847,897 | 296,821,565 | -49.7635% | 1,118,393,189 | 644,195,524 | -42.3999% |
| 2,048 | 2,330,629,672 | 2,330,928,941 | +0.0128% | 445,770,817 | 445,828,609 | +0.0130% | 870,224,758 | 868,368,182 | -0.2133% |

The 64-bit increase is the explicit cost of checking whether six enclosure
endpoints remain suitable for the compact binary32 scheduler. The 2,048-bit
case already declined that carrier because its enclosure was unavailable, so
its deterministic retired work is effectively unchanged. The 512-bit case is
the general representable-but-collapsed interval: instructions fall 52.06%,
branches 49.76%, and cycles 42.40% without changing output or certainty.

Broad controls bound the cost of the general checks:

| Workload | Instruction change | Branch change | Certainty |
| --- | ---: | ---: | --- |
| crossing octahedra, all four x1000 | +0.0164% | +0.0018% | `Certified` |
| affine boxes, all four x1000 | +0.0882% | +0.0629% | `Certified` |
| sparse 512 shells, all four x5 | +0.0606% | +0.0719% | `Certified` |
| dense coplanar 32, all four x1 | +0.1307% | +0.0917% | `Certified` |
| clipped voxel torus 33, all four x3 | +0.1792% | +0.1606% | `Certified` |
| 2,049-bit wide boxes, union x5 | +0.0536% | +0.0389% | `Certified` |
| full YeahRight instrumented kernel x1 | +0.0303% | +0.0284% | `Certified` |
| symbolic depth 1, `STRICT`, all four x20 | +0.00002% | +0.00003% | `Certified` |
| symbolic depth 128, `APPROXIMATE_512`, all four x5 | +0.00004% | +0.00007% | `Approximate512Consumed` |

Performance is prioritized over size, so the sub-0.18% broad overhead is
accepted for a 52% instruction reduction in a permanent exact corpus cell.
The control rows also demonstrate that the change is not selecting the corpus
topology or its expected result.

## Dispatch evidence

One traced 512-bit union records the following parent/current movements:

| Dispatch count | Parent | Current | Change |
| --- | ---: | ---: | ---: |
| BVH candidate pairs | 261,232 | 69,712 | -73.31% |
| affine classifications | 1,848,988 | 632,360 | -65.80% |
| newly constructed point queries | 1,440,092 | 425,884 | -70.43% |
| retained rational point queries | 82,558 | 425,884 | +415.84% |
| floating certificates | 34,029 | 254,791 | +648.75% |
| arbitrary-precision product sums | 134,727 | 26,454 | -80.36% |
| word product sums | 1,700,273 | 364,155 | -78.58% |
| all-zero classifications | 467,616 | 137,520 | -70.59% |

The exact borrowed bounds remove conservative false positives before
intersection work. Among the surviving pairs, Hyperreal's normalized retained
rational facts certify substantially more linear forms before the exact word
and arbitrary-precision schedules. This is the intended layered use of
`hyperreal::Real`: retain exact structure, schedule a proved cheap query, and
fall through without semantic loss when the proof is inconclusive.

## Large-fixture heap and RSS

The 512-bit large fixture ran in a fresh instrumented process under both
policies. Every requested-payload metric is byte-identical between policies and
the output remains `Certified`.

| Metric | Parent | Current | Change |
| --- | ---: | ---: | ---: |
| input payload | 798,040 | 798,040 | 0 |
| total peak | 23,610,989 | 23,610,989 | 0 |
| incremental kernel peak | 22,812,314 | 22,812,314 | 0 |
| post-Boolean incremental | 548,712 | 548,712 | 0 |
| output-live payload | 520,472 | 520,472 | 0 |
| input fact growth | 28,240 | 28,240 | 0 |
| post-input-drop residual | 122,584 | 122,584 | 0 |
| allocation calls | 1,043,381 | 400,128 | -61.65% |
| deallocation calls | 1,043,104 | 399,851 | -61.67% |
| reallocations | 9,236 | 6,081 | -34.16% |
| cumulative added bytes | 113,027,238 | 46,478,238 | -58.88% |
| cumulative removed bytes | 112,478,526 | 45,929,526 | -59.17% |

The complete full-YeahRight heap control is exactly unchanged at a
59,440,454-byte requested-payload peak, 52,317,092-byte incremental kernel
peak, 9,906,413 calls, 1,612,899 reallocations, and 582,162,615 cumulative
added bytes. The new schedules therefore reduce the targeted fixture's churn
without growing any measured large-fixture peak.

Fresh maximum RSS for the 512-bit row is 28,580 KiB under `STRICT` and 28,952
KiB under `APPROXIMATE_512`, versus 10,016 KiB for CGAL EPECK. The strict ratio
improves from 2.928x to 2.853x, but both 2.853x/2.890x losses remain open.

## Pinned CGAL EPECK boundary

The exact reduced-rational OFF input and CGAL 6.0.3 EPECK output remain
unchanged at 2,410 vertices and 4,816 valid, closed triangles. Complete
21-union process counters include exact input generation or OFF parsing once.

| Engine / revision | Instructions | Branches | Cycles |
| --- | ---: | ---: | ---: |
| Hypermesh parent | 70,639,080,634 | 12,177,322,425 | 32,045,757,750 |
| Hypermesh current | 33,127,045,250 | 5,997,323,181 | 12,848,036,917 |
| CGAL EPECK | 4,854,933,811 | 893,866,332 | 1,893,795,616 |

The current process removes 53.10% of instructions, 50.75% of branches, and
59.91% of cycles from Hypermesh. Its remaining deterministic losses are
6.823x, 6.709x, and 6.784x respectively. Observed Hypermesh internal means of
143.18--146.57 ms remain roughly 22.75--23.28x the 6.295 ms CGAL median. Host
frequency makes those wall values advisory; the per-case gap is explicitly
open and is not called parity.

## Code and binary size

Production changes are 30 insertions and 12 deletions across Hyperreal and
Hypermesh. The remaining lines are exact regression/oracle tests. No data
field, dependency, feature, public API, or monomorphized policy type is added.

| Configuration | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| default release native general text | 1,957,586 | 1,957,890 | +304 |
| default release native immediate text | 1,960,730 | 1,961,034 | +304 |
| default release optimized WASM general | 1,381,518 | 1,381,629 | +111 |
| default release optimized WASM immediate | 1,383,359 | 1,383,470 | +111 |
| default size native general text | 1,068,935 | 1,069,047 | +112 |
| default size native immediate text | 1,069,891 | 1,070,003 | +112 |
| default size optimized WASM general | 661,997 | 662,037 | +40 |
| default size optimized WASM immediate | 662,410 | 662,450 | +40 |
| all-feature release native general text | 2,093,063 | 2,093,367 | +304 |
| all-feature release native immediate text | 2,095,919 | 2,096,223 | +304 |
| all-feature release optimized WASM general | 1,455,501 | 1,455,608 | +107 |
| all-feature release optimized WASM immediate | 1,457,473 | 1,457,580 | +107 |
| all-feature size native general text | 1,071,175 | 1,071,287 | +112 |
| all-feature size native immediate text | 1,072,147 | 1,072,259 | +112 |
| all-feature size optimized WASM general | 662,255 | 662,295 | +40 |
| all-feature size optimized WASM immediate | 662,294 | 662,334 | +40 |

The maximum increase is 304 native text bytes; optimized WASM adds at most 111
bytes. This bounded growth is retained for the much larger deterministic
runtime and allocation-traffic win.

## Call graph and rejected alternative

Fresh five-crate graphs contain 15,149 production nodes / 25,270 edges, 16,603
nodes / 27,562 edges with tests, and 21,531 / 34,711 with tests, examples,
benches, and fuzz targets. The exact scope is Hyperreal, Hyperlattice,
Hyperlimit, Hypertri, and Hypermesh; Hypercurve and HyperSolve are excluded.

`CertifiedAabbFilter::from_bounds_ref` can only construct a conservative
carrier or return unavailable. `bounds_overlap_decision` still enters the one
borrowed exact AABB route. `RationalLinearForm4Filter::sign` reaches the one
certified Hyperreal sign function, and both filter/query constructors reach the
same normalization function. There are zero actual `ember` namespace nodes;
substring matches are only ordinary `membership` identifiers. No old engine,
new selector, or alternate predicate route appears.

An intermediate rational-filter form checked every lane product for normality.
It improved the 512-bit target but added roughly 2% instructions to the 64-bit
sibling. It was removed completely. The retained absolute error floor proves
the same underflow boundary once per query and keeps the broad overhead below
0.18%.

## Validation and reproduction

Hyperreal passes 652 all-feature and 574 no-default unit tests, all integration
suites, and 24/19 respective doctests. Hypermesh passes 203 default tests with
six ignores, 204 all-feature tests with six ignores, and 154 minimal library
tests. Warning-denied all-target/all-feature Clippy and rustdoc, fuzz-target
checks, bench/example builds, formatting, diff checks, both size matrices,
both-policy large heap probes, dispatch tracing, and all three call graphs pass.

```sh
cargo test --locked
cargo test --locked --all-features
cargo test --locked --no-default-features --lib
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS=-Dwarnings cargo doc --locked --all-features --no-deps
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo bench --locked --no-run --all-features
cargo check --locked --examples --all-features
benchmarks/size-harness/measure.sh default
benchmarks/size-harness/measure.sh all
```

Phases 11, 17, and 18 remain open. This checkpoint closes the anomalous
512-bit conservative-scheduling cell, but not its CGAL runtime/RSS deficits,
the smaller 64-bit overhead, external real-world corpus work, remaining
arrangement/corefinement ownership, or the final requirement audit.
