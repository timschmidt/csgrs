# Phase 17: retained-fact product-sum scheduling

Captured 2026-08-04. The mesh implementation remains Hypermesh `f3683ba9`;
the scalar scheduling implementation is Hyperreal `0a952182`, based on
Hyperreal `c2217bae`. The preceding Hypermesh evidence head is `43c8c170`.

## Result

`Real::signed_product_sum` and `Real::active_signed_product_sum` now retain the
constant-shape Pythagorean `SinPi` identity as their first dispatch, then try
the exact-rational whole-polynomial reducer before scanning for the general
orthonormal sine polynomial. The bounded expression tree remains the final
fallback.

This plays directly to Hyperreal's retained scalar facts. An all-rational
determinant no longer scans every factor merely to prove that no `SinPi` atom
exists before entering the shared-denominator reducer. The fixed two-term sine
identity keeps its original fastest route, and all other symbolic inputs still
reach the same recognizer or expression builder after the exact reducer
declines.

The implementation is a dispatch reorder at two public fixed-shape entry
points plus corrected documentation: 13 inserted and 13 deleted production
lines, no new function, API, dependency, allocation, graph node, compatibility
layer, or Boolean-engine path. Production contains no fixture, triangle-count,
coordinate-width, operation, expected-output, policy-name, benchmark, or
competitor dispatch.

Phase 17 and Phase 18 remain open. This checkpoint improves general exact
work, but it does not claim that the outstanding pinned CGAL EPECK runtime or
RSS gates have closed.

## Path completeness and policy argument

The order preserves every prior route:

1. The fixed Pythagorean recognizer still runs first and returns only after
   exact structural proof from retained `SinPi` arguments and rational scales.
2. The exact-rational reducer returns only when every factor exposes a borrowed
   exact `Rational`; otherwise it returns `None` without approximation.
3. The general orthonormal recognizer therefore still receives every input it
   could previously certify, including mixed rational/`SinPi` polynomials.
4. The same bounded expression-tree builder receives every input declined by
   all three exact recognizers.

The recognized classes are proof-bearing scalar shapes, not mesh cases. No new
numeric comparison or topology decision is introduced. `STRICT` therefore
still refuses unresolved decisions, while `APPROXIMATE_512` can terminate only
through Hyperlimit's existing 512-bit terminal. Existing aggregate certainty
propagation is unchanged, and exact-rational work remains `Certified` under
both policies.

## Paired mesh work

Parent and candidate were compiled from equal source states into independent
release targets and run on CPU 11. Instructions and branches are the acceptance
metrics; wall time moved with host frequency and is not used. The wide control
uses five complete shared arrangements to amplify a very small movement.

| Workload | Parent instructions | Current instructions | Movement | Branch movement |
| --- | ---: | ---: | ---: | ---: |
| Full rotated YeahRight, 23,788 triangles | 16,526,759,045 | 16,506,581,246 | -0.1221% | -0.1248% |
| 2,049-bit wide rational, 6,144 triangles, five arrangements | 14,668,312,208 | 14,668,171,266 | -0.0010% | -0.0030% |
| Clipped voxel torus 33, 6,412 triangles | 1,134,319,463 | 1,133,187,357 | -0.0998% | -0.1096% |
| Clipped voxel torus 65, 25,100 triangles | 4,914,802,919 | 4,908,029,439 | -0.1378% | -0.2476% |
| Dense coplanar 16, 6,144 triangles | 3,019,032,101 | 3,002,649,818 | -0.5426% | -0.8260% |

The ordinary exact-box control runs 1,000 complete shared arrangements per
process and exercises all four built-in results from one arrangement.

| Policy | Parent instructions | Current instructions | Movement | Branch movement |
| --- | ---: | ---: | ---: | ---: |
| `STRICT` | 5,677,972,759 | 5,666,961,740 | -0.1939% | -0.2425% |
| `APPROXIMATE_512` | 5,682,917,084 | 5,662,035,127 | -0.3675% | -0.4876% |

Every large control and both policy rows retire less deterministic work. The
wide row is intentionally reported as essentially neutral rather than
overstating a sub-0.001% instruction result.

## Scalar route audit

A fixed-iteration `perf stat` probe isolated the four affected scalar classes
before it was removed; it changed no production dispatch and shipped no probe
binary. The existing permanent scalar Criterion group covers rational and
mixed-symbolic determinant sums, while permanent unit tests cover both sine
identities.

| Public `Real::signed_product_sum` class | Parent instructions | Current instructions | Movement | Branch movement |
| --- | ---: | ---: | ---: | ---: |
| Rational determinant, 5,000,000 calls | 23,625,382,390 | 23,210,382,417 | -1.7566% | -3.2257% |
| Mixed symbolic determinant, 1,000,000 calls | 24,259,403,400 | 24,259,403,532 | +0.000001% | effectively zero |
| Pythagorean `SinPi`, 5,000,000 calls | 6,620,388,466 | 6,620,388,464 | effectively zero | effectively zero |
| General orthonormal `SinPi`, 500,000 calls | 3,747,390,067 | 3,758,390,006 | +0.2935% | +0.1776% |

The retained order deliberately preserves the fast Pythagorean route, makes
mixed symbolic work instruction-identical, and accepts one failed exact class
check before the rarer general orthonormal proof. That small local cost is
outweighed by the rational and end-to-end gains without adding a shape or
benchmark special case.

## Large-fixture heap

The direct global-allocator probe excludes fixture construction. Current and
paired parent counters are exactly equal, and `STRICT` and
`APPROXIMATE_512` are byte-for-byte equal for output, certainty, peak, call
counts, and byte totals.

| Selector | Policy | Incremental peak | Allocations | Reallocations | Added bytes |
| --- | --- | ---: | ---: | ---: | ---: |
| `yeahright-full-rotated` | both | 158,258,204 B | 16,928,390 | 2,385,300 | 913,360,240 B |
| `wide-rational-2048` | both | 31,234,658 B | 2,092,630 | 227,677 | 459,671,086 B |

The full row also retains 16,575,749 deallocations and 888,970,336 removed
bytes; the wide row retains 2,092,488 deallocations and 459,124,046 removed
bytes. The counter drift from older evidence predates this candidate; an
immediate parent rebuild reproduced the current counters exactly.

## Source and linked size

The scalar source has zero net line growth. Native values are `.text`; WASM
values are `wasm-opt -Oz` bytes.

| Profile / consumer | Parent native | Current native | Parent WASM | Current WASM |
| --- | ---: | ---: | ---: | ---: |
| release / general | 1,968,582 | 1,968,630 (+48) | 1,391,425 | 1,391,425 (equal) |
| release / immediate | 1,971,726 | 1,971,774 (+48) | 1,393,283 | 1,393,283 (equal) |
| size / general | 1,071,807 | 1,071,807 (equal) | 668,454 | 668,459 (+5) |
| size / immediate | 1,072,771 | 1,072,771 (equal) | 668,865 | 668,870 (+5) |

The largest movement is 48 native bytes (0.0024%); performance has priority,
and the WASM increase is five bytes only in the size profile.

## Validation and graph

- Hyperreal passes 649 all-feature unit tests plus every integration/oracle and
  24 doctests; its 572-test no-default suite and 19 doctests also pass.
- Hyperlimit, Hyperlattice, and Hypertri pass their complete all-feature
  suites. Hyperlimit's STRICT/APPROXIMATE_512 coverage remains green.
- Hypermesh passes 179 all-feature and 178 default executions; six documented
  external/manual stress tests remain ignored.
- No-default checking, warning-denied Clippy and rustdoc, every fuzz binary,
  every benchmark target, formatting, diff checks, and the locked release
  Trunk demo pass.
- Full and 2,049-bit heap sentinels pass under both policies with exact policy
  and parent equality.

The regenerated Hyperreal/Hyperlattice/Hyperlimit/Hypertri/Hypermesh graph is
structurally unchanged at 14,792 nodes and 24,633 edges. Hypermesh contributes
2,913 nodes and 4,668 edges; Hyperreal contributes 7,210 nodes and 12,416
edges. Hypercurve and HyperSolve are excluded. No EMBER, compatibility, or
second Boolean-engine route reappears.

## Rejected alternatives

- Moving the exact reducer ahead of the fixed Pythagorean identity produced a
  slightly lower full-row count, but needlessly charged the established fast
  sine route. The retained order gives up about 0.02% on that one mesh profile
  to preserve the clean proof-specific schedule.
- A mesh-local all-rational support-value helper saved about 0.05% on the full
  row but added roughly 1.7 KiB native and 2.6 KiB WASM and duplicated scalar
  fact scheduling. Outlining recovered only 232 native and 124 WASM bytes; the
  helper was fully removed.
- A two-product construction variant regressed end-to-end work and was fully
  removed.

## Competitive and open work

This checkpoint does not special-case or hide the difficult competitive rows.
The governing pinned CGAL 6.0.3 EPECK evidence still reports approximately a
19.00x full-row runtime loss and 12.25x fresh-process RSS loss, with ordinary
box losses of 4.81x (`STRICT`) and 4.53x (`APPROXIMATE_512`). Those historical
ratios are not relabeled as current measurements merely because deterministic
Hypermesh work improved by less than one percent.

Current CGAL confidence runs, external real-world and deeper-symbolic fixture
families, sparse/multi-shell/pathological expansion, stage-specific arena
attribution, remaining runtime/RSS gates, and the final Phase 18 audit remain
open.

## Reproduction

```sh
cargo test --locked --all-features
cargo test --locked
cargo check --locked --no-default-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --locked --all-features --no-deps
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo bench --locked --no-run

YEAHRIGHT_BENCH=1 taskset -c 11 perf stat -x, \
  -e instructions:u,branches:u \
  target/release/examples/large_mesh_heap_probe \
  yeahright-full-rotated strict
taskset -c 11 perf stat -x, -e instructions:u,branches:u \
  target/release/examples/competitive_arrangement_probe \
  overlapping_boxes all strict 1000
target/release/examples/large_mesh_kernel_heap_probe \
  <fixture-selector> <strict|approximate-512>
benchmarks/size-harness/measure.sh

tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir /tmp/hypermesh-exact-rational-product-sum-schedule-callgraph-2026-08-04 \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format dot,json,mermaid,graphml \
  --per-library
```
