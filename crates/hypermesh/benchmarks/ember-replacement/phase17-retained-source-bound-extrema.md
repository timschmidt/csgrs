# Phase 17 checkpoint: retained exact source-bound extrema

Date: 2026-08-05

Status: accepted checkpoint; Phases 17 and 18 remain open

Hypermesh implementation: `7d836392643a2a786e57db2aa06de9c0cd2e955c`

Hyperlimit implementation: `d2dee8fb801e3255dbcef93eb054d897003c28cf`

This checkpoint removes the remaining materialized per-source-triangle exact
AABB without weakening the canonical Boolean engine or its predicate policy.
Each source triangle now retains six triangle-local extrema selectors packed in
two bytes. Those selectors borrow the exact `Real` coordinates from the one
shared source-position owner introduced by the preceding checkpoint.

The representation is general: it has no fixture, triangle-count, expected
result, Boolean-operation, or competitor branch. Standalone and
arrangement-derived polygons retain their existing owned-bound behavior. No
compatibility surface or alternate engine was added.

## Exactness and policy ownership

Hyperlimit now exposes one borrowed-coordinate form of its existing ordered
3D AABB predicate. The point-based entry point delegates to it, and both forms
call the same `ordered_aabb3_pairwise_relation` cascade. Hypermesh never makes
a terminal equality decision itself.

The direct symbolic regression uses the exact identity
`(pi + e) - (e + pi)` on a touching box boundary:

- `STRICT` returns `Unknown`;
- `APPROXIMATE_512` returns `Decided(true)` with `Approximate` certainty;
- exact rational inputs remain `Exact` under both policies.

Source-face extrema are chosen through `compare_real_decision`, so construction
uses the selected mesh policy and contributes to the aggregate mesh certainty.
Inversion reverses the triangle-local selectors with the vertex cycle and is
checked against the original exact bounds. The packed descriptor keeps
`RetainedVertexCycle` at 24 bytes.

The compact query hierarchy also moves, rather than recomputes, the certified
binary32 primitive filters produced during BVH construction. These filters can
only prove disjointness. Missing or unavailable filters fall through to the
borrowed exact predicate, and malformed filter/polygon storage is rejected
explicitly.

## Broad performance controls

`perf stat -r 3` was pinned to CPU 11. The parent executable is the preceding
shared-source-position implementation. Instructions and branches improve on
all six topology/scalar-width controls:

| Fixture / workload | Instruction change | Branch change |
| --- | ---: | ---: |
| overlapping boxes, all four x1000 | -0.646% | -0.610% |
| sparse multishell 512, all four x5 | -2.064% | -2.239% |
| dense coplanar 32, all four x1 | -0.580% | -0.761% |
| clipped voxel torus 33, all four x3 | -0.898% | -0.851% |
| wide rational 2048, union x5 | -0.261% | -0.280% |
| full rotated YeahRight, intersection x1 | -0.185% | -0.211% |

This is the intended Hyperreal advantage: exact extrema and their certified
enclosures are retained once, then scheduled from the cheapest sufficient fact
to the canonical exact predicate. No benchmark is recognized or bypassed.

## Large-fixture heap matrix

All 15 selectors were measured under both `STRICT` and
`APPROXIMATE_512`. Every policy pair was byte-for-byte identical, every result
was `Certified`, every peak and allocation count fell, and reallocation counts
were unchanged.

| Selector | Parent peak | Current peak | Peak change | Allocation-call change | Added-byte change |
| --- | ---: | ---: | ---: | ---: | ---: |
| boxes-3072 | 14,349,904 | 12,580,432 | -1,769,472 (-12.331%) | -6,144 | -1,769,472 |
| boxes-3072-general | 14,399,504 | 12,630,032 | -1,769,472 (-12.288%) | -6,144 | -1,769,472 |
| dense-coplanar-16 | 17,046,229 | 15,424,213 | -1,622,016 (-9.515%) | -12,288 | -3,538,944 |
| dense-coplanar-32 | 67,937,525 | 61,449,461 | -6,488,064 (-9.550%) | -49,152 | -14,155,776 |
| sparse-shells-512 | 13,739,923 | 12,658,579 | -1,081,344 (-7.870%) | -4,096 | -1,179,648 |
| self-pwn-clusters-512 | 13,871,137 | 12,788,737 | -1,082,400 (-7.803%) | -4,100 | -1,180,800 |
| wide-rational-64 | 19,630,974 | 17,861,502 | -1,769,472 (-9.014%) | -6,144 | -1,769,472 |
| wide-rational-512 | 20,520,231 | 18,750,759 | -1,769,472 (-8.623%) | -6,144 | -1,769,472 |
| wide-rational-2048 | 29,195,064 | 27,425,592 | -1,769,472 (-6.061%) | -6,144 | -1,769,472 |
| voxel-torus-33 | 14,835,880 | 12,989,224 | -1,846,656 (-12.447%) | -6,412 | -1,846,656 |
| voxel-torus-65 | 59,816,684 | 52,587,884 | -7,228,800 (-12.085%) | -25,100 | -7,228,800 |
| yeahright | 5,596,865 | 5,371,937 | -224,928 (-4.019%) | -852 | -245,376 |
| yeahright-4 | 21,010,549 | 20,120,341 | -890,208 (-4.237%) | -3,372 | -971,136 |
| yeahright-8 | 80,135,413 | 76,261,237 | -3,874,176 (-4.835%) | -13,452 | -3,874,176 |
| yeahright-full-rotated | 154,886,838 | 148,606,806 | -6,280,032 (-4.055%) | -23,788 | -6,850,944 |

The dense fixtures construct the source cache twice, explaining their doubled
allocation and total-added-byte savings. Peak lifetime depends on the fixture:
when the compact query hierarchy is live at the global peak, its reused
24-byte certified filter offsets part of the 288-byte source-bound removal.

## Stage-specific ownership

Fresh Heaptrack captures of the full 11,894-by-11,894 rotated intersection
show exactly where that trade lands:

| Live owner at peak | Parent | Current | Change |
| --- | ---: | ---: | ---: |
| total | 154,960,566 | 148,680,534 | -6,280,032 (-4.05%) |
| polygon soup | 111,787,592 | 104,936,648 | -6,850,944 (-6.13%) |
| surface arrangement | 35,975,852 | 36,546,764 | +570,912 (+1.59%) |
| corefinement | 32,732,220 | 32,732,220 | 0 |
| pairwise intersection | 2,853,604 | 2,853,604 | 0 |

There are 23,788 source faces. Thus the source owner loses exactly 288 bytes
per face, the arrangement retains exactly 24 bytes per face of already-built
certified filter, and the global peak loses exactly 264 bytes per face. The
optimization does not displace memory into corefinement or intersection work.
Captures are
`target/phase17-parent-source-bound-views-full-strict.zst` and
`target/phase17-retained-source-bound-extrema-full-strict.zst`.

## Historical and CGAL boundary

A fresh CPU-11-pinned process produced the same certified exact-empty full
rotated result in 1.34 seconds and 173,040 KiB maximum RSS, versus 1.36 seconds
and 179,540 KiB for the parent. These single-process wall rows are advisory;
the hardware frequency was noisy, so instruction and branch counts above are
the performance acceptance authority. Relative to the established historical
EMBER row, current is about 2,472x faster and uses 47.5% less RSS. The
historical full-fixture CGAL EPECK row remains far ahead at 0.09 seconds and
15,516 KiB.

CGAL 6.0.3 EPECK was refreshed for 63 exact-OFF repetitions in both copy modes;
Hypermesh used 63 fresh-process samples. Every CGAL output was valid, closed,
structurally valid, and matched Hypermesh's topology and exact-volume oracle.

| Fixture | CGAL outside | CGAL inside | Hypermesh STRICT | Ratio to outside | Hypermesh APPROXIMATE_512 | Ratio to outside |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| crossing octahedra | 125,342 ns | 128,052 ns | 479,747 ns | 3.828x | 482,137 ns | 3.847x |
| affine boxes | 381,374 ns | 377,804 ns | 875,520 ns | 2.296x | 866,390 ns | 2.272x |

The CGAL gaps remain open. No small-case, competitor-specific, or expected
output path is introduced to close them.

## Code and binary size

The general retained-fact implementation adds 268 and removes 52 Hypermesh
lines; the canonical Hyperlimit API and its policy regressions add 76 and
remove 13. Performance was prioritized, but size remains bounded:

| Consumer/profile | Native text change | Optimized WASM change |
| --- | ---: | ---: |
| default release, general | +10,008 bytes (+0.504%) | +3,041 bytes (+0.215%) |
| default release, immediate | +10,008 bytes (+0.504%) | +3,051 bytes (+0.216%) |
| default size, general | +3,384 bytes (+0.313%) | +2,270 bytes (+0.335%) |
| default size, immediate | +3,384 bytes (+0.312%) | +2,270 bytes (+0.335%) |
| all-feature release, general | +9,656 bytes (+0.455%) | +2,397 bytes (+0.161%) |
| all-feature release, immediate | +9,656 bytes (+0.455%) | +2,413 bytes (+0.162%) |
| all-feature size, general | +3,480 bytes (+0.321%) | +2,727 bytes (+0.403%) |
| all-feature size, immediate | +3,488 bytes (+0.321%) | +2,727 bytes (+0.403%) |

Current all-feature release native text is 2,130,427/2,133,267 bytes and
optimized WASM is 1,492,610/1,494,583 bytes for general/immediate consumers.
Current all-feature size-profile native text is 1,088,047/1,089,011 bytes and
optimized WASM is 679,270/679,543 bytes.

## Call graph and validation

The refreshed five-crate graphs contain 14,997 nodes/25,023 edges for
production, 17,317/28,339 with tests and examples, and 21,363/34,440 with all
tests/examples/benches/fuzz targets. Hypermesh has one edge from borrowed
bounds overlap to Hyperlimit's borrowed AABB predicate, which has one edge to
the existing canonical relation cascade. There are zero production EMBER,
local-BSP, or segment-trace nodes.

Validation passed:

```sh
# Hyperlimit
cargo fmt --all -- --check
cargo test --locked --all-features
cargo test --locked --no-default-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --all-features --no-deps

# Hypermesh
cargo fmt --all -- --check
cargo test --locked --all-features
cargo test --locked --no-default-features --lib
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --all-features --no-deps
cargo check --locked --manifest-path fuzz/Cargo.toml --all-targets
cargo check --locked --benches --all-features
cargo check --locked --examples --all-features
```

The next measured polygon-soup target is exact source-plane ownership. It must
follow the same rule: retain one canonical exact representation and schedule
from Hyperreal facts, without replacing a clear general algorithm with fixture
or microbenchmark cases.
