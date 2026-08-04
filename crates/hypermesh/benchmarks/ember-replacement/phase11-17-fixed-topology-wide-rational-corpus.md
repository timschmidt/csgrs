# Phase 11/17 fixed-topology wide-rational corpus checkpoint

Captured 2026-08-03 on the established Ryzen 7 5800X3D / CPU 11 protocol at
Hypermesh `4408288fe5053d7c9da205583b6718b506e52afe`.

This checkpoint closes the first fixed-topology arbitrary-rational-width
corpus gap. It holds authored mesh connectivity, arrangement topology, bounds,
and binary64 approximations fixed while growing the exact similarity from a
one-bit identity through 65-, 513-, and 2,049-bit numerator/denominator
components. It adds exact both-policy oracles, identical reduced-rational CGAL
inputs, direct kernel heap boundaries, fresh-process RSS, hardware counters,
and a high-width profile. It does not claim Phase 11 corpus completion, Phase
17 performance completion, Phase 18 completion, or whole-corpus CGAL parity.

## Result

The permanent family begins with the existing 6,144-triangle subdivided
overlapping-box case and applies the positive exact similarity

`s(k) = (2^k + 1) / 2^k`

for `k = 64, 512, 2048`. Every transformed point remains an exact
`hyperreal::Real`; no decimal or binary64 round trip constructs the fixture.
All four siblings use exactly the same 3,072 triangles per operand and the same
triangle index arrays. The three wide scales all round to `1.0` in binary64,
so broad-phase geometry and floating filters see the same finite coordinates
while exact schedules must distinguish the growing rationals.

There is no production-code or Cargo change. Fixture selection exists only in
the deterministic corpus support, exporter, competitive probe, and heap
probe. The production Boolean engine does not inspect fixture identity,
coordinate width, triangle count, operation, policy, expected result, or
measurement state.

## Permanent geometry and policy contract

| Fixture | Scale component bits | Total input T | Union output V/T | Normalized exact volume |
| --- | ---: | ---: | ---: | ---: |
| `subdivided_overlapping_boxes_3072_each` | 1 | 6,144 | 2,410 / 4,816 | 84 |
| `wide_rational_boxes_64` | 65 | 6,144 | 2,410 / 4,816 | 84 |
| `wide_rational_boxes_512` | 513 | 6,144 | 2,410 / 4,816 | 84 |
| `wide_rational_boxes_2048` | 2,049 | 6,144 | 2,410 / 4,816 | 84 |

The permanent regression evaluates union, intersection, left-minus-right,
right-minus-left, and XOR together through one arrangement for all three wide
scales under `STRICT` and `APPROXIMATE_512`. It uses a smaller two-division
member so every operation can carry a full exact oracle in the ordinary test
suite. After division by the exact scale, all vertices and triangle arrays are
identical across widths. The exact volumes are respectively
`[84, 12, 52, 20, 72] * s(k)^3`; equivalently, the exact six-volume
coefficients are `[504, 72, 312, 120, 432]`.

Every output is finite, nondegenerate, directionally balanced, and
`Certified`. `STRICT` and `APPROXIMATE_512` return exactly equal batches; no
terminal approximate decision is consumed. The large union probes reproduce
the 2,410-vertex/4,816-triangle result under both policies with byte-identical
requested-payload metrics at each width.

## Identical reduced-rational CGAL inputs

`export_cgal_exact_off` now has one exact `TriangleMesh` serialization route
for ordinary, dense-coplanar, torus, and wide-rational fixtures. It writes the
reduced numerator and denominator from `Real::exact_rational`, never a display
or binary64 approximation. CGAL 6.0.3 EPECK at
`cefe3007d59cf4292a09da4fa8a35f38478a4e0b` therefore consumes the same exact
authored coordinates.

| Scale | Left SHA-256 | Right SHA-256 |
| ---: | --- | --- |
| 1 | `180adda7add394332e99ee76cf5eeb5638128abdc77c432bdf83b3dfd44b2c31` | `dc72da54edf2ccafc864dcfdd43a245056f7f63cc6b60d1e286ad3db560df039` |
| 65-bit | `d2ebe4956aa1ef23e21f7f79d923d35b6a794a146c22b5aa90c1a7b9b1796c2b` | `8cd482c9bd04e71128248998a378d96507368665ae1325f92f65fb2090eed1c4` |
| 513-bit | `d8f4e81f743403e2baa3e0adde96b78fff5ecd50396b0c7907c0af6311fd116d` | `c0942f9e5317819972271ed2d62401db71634cf9531aef52f87c4366292cbf01` |
| 2,049-bit | `64ee8735c5228ce1547294528d8f521dc1c2dd3bdcbb9a6ca4f600d4c2caa8b5` | `523e3b5b20e106fb0cf62cbdba08f49fae29e0b1cc0301c523d35294b9e5bd48` |

CGAL reports every union valid, closed, and structurally valid with the same
2,410 vertices and 4,816 triangles. Its diagnostic binary64 volume is 84 at
all widths because `s(k)` rounds to one; Hypermesh's oracle above verifies the
unrounded exact similarity volume.

## Current Hypermesh versus CGAL EPECK

Both executables use one thread pinned to CPU 11. The base, 65-bit, and
513-bit Hypermesh values are medians of three independent 11-call aggregate
means after exact import and policy-qualified PWN priming. The 2,049-bit value
uses three independent five-call aggregate means. CGAL is the median of 21
calls to `corefine_and_compute_boolean_operations`, with its required mutable
input copy outside the interval. The operation is union in both engines.

| Scale bits | Hypermesh STRICT | Hypermesh APPROXIMATE_512 | CGAL median (min-max) | STRICT/CGAL | APPROX/CGAL |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 94.123 ms | 93.283 ms | 4.673 ms (4.623-5.260) | 20.14x | 19.96x |
| 65 | 141.633 ms | 140.274 ms | 22.379 ms (22.292-23.929) | 6.33x | 6.27x |
| 513 | 187.713 ms | 192.244 ms | 29.266 ms (28.908-31.845) | 6.41x | 6.57x |
| 2,049 | 542.174 ms | 540.822 ms | 55.244 ms (54.942-57.859) | 9.81x | 9.79x |

Hypermesh is slower on every absolute row, so all four remain open per-case
runtime gates. Its `STRICT` growth from the identity is 1.505x, 1.994x, and
5.760x, versus CGAL's 4.789x, 6.263x, and 11.821x. Hyperreal's retained
expression facts and delayed exact schedules therefore provide a real width
slope advantage through this family, but the 513-to-2,049-bit allocation and
multiply-accumulate cliff widens the remaining absolute gap.

There is no historical EMBER row for this new exact-similarity family, so no
historical speedup is inferred. The established overlapping-box and YeahRight
rows remain the historical scorecards.

## Runtime slope and high-width profile

Eleven fresh-process `perf stat` repetitions use the ordinary uninstrumented
probe and include fixture construction, exact import, policy-qualified PWN
priming, and one production `STRICT` Boolean call.

| Scale bits | Task clock | Cycles | Instructions | Branches | Branch misses | Cache misses |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 114.92 ms | 454.470M | 1.258B | 222.734M | 1.753M | 1.135M |
| 65 | 180.93 ms | 712.134M | 1.884B | 341.588M | 2.273M | 3.132M |
| 513 | 253.99 ms | 991.591M | 2.581B | 481.725M | 2.594M | 3.542M |
| 2,049 | 744.52 ms | 2.970B | 8.612B | 1.398B | 5.923M | 5.871M |

At 2,049 bits, task clock/cycles/instructions/branches grow
6.479x/6.535x/6.847x/6.275x from the fixed-topology identity. Branch misses
grow only 3.380x and cache misses 5.173x; the primary excess is exact arithmetic
work and allocation rather than an arrangement-topology explosion.

A separate CPU-11 `perf record -F 999 -g --call-graph dwarf` run of the same
2,049-bit process captured 768 cycle samples with none lost. Self samples are
led by `num_bigint::biguint::multiplication::mac3` at 39.65%, followed by
`BigUint::trailing_zeros` at 4.20%, `memmove` at 3.98%, BigUint `sub_sign` at
3.51%, `Rational::to_f64_lossy` at 3.04%, and
`classify_exact_rational_coordinates` at 2.13%. The fused four-product
sign-ordering fallback contributes 1.55%, Rational comparison 1.32%, and
`realloc` 1.21%. This points to clean, general exact-arithmetic ownership,
normalization, and schedule reuse—not a mesh or fixture special case—as the
next optimization area.

## Direct large-mesh heap

The allocator-instrumented executable measures successful requested Rust
allocation payload only. Preparation and optional input priming finish before
the Boolean interval; output and input are dropped separately afterward. Each
row was reproduced in separate `STRICT` and `APPROXIMATE_512` processes with
identical bytes, event counts, topology, and `Certified` certainty.

| Scale bits | Prepared input | Incremental kernel peak | Output payload | Input fact growth | Alloc calls | Realloc calls | Allocated bytes |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 655,616 B | 15,807,426 B | 520,472 B | 13,536 B | 166,410 | 788 | 27,359,910 B |
| 65 | 617,304 B | 26,906,378 B | 520,472 B | 9,976 B | 1,076,635 | 2,035 | 68,227,422 B |
| 513 | 626,848 B | 46,033,482 B | 520,472 B | 12,744 B | 1,806,898 | 38,201 | 293,106,862 B |
| 2,049 | 664,368 B | 111,848,010 B | 520,472 B | 26,568 B | 3,183,205 | 955,733 | 1,488,168,086 B |

From identity to 2,049 bits, prepared input and output remain within 1.33% and
exactly flat respectively, while incremental kernel peak grows 7.0757x,
allocation calls 19.1287x, reallocations 1,212.8591x, and allocated-byte churn
54.3923x. Post-Boolean incremental ownership moves only from 534,008 to
547,040 bytes. Hyperreal's shared retained input representation is therefore
doing useful work; repeated temporary arbitrary-width construction and growth
inside the kernel is the measured memory target.

Fresh-process maximum RSS is 20,640/20,800 KiB for Hypermesh
`STRICT`/`APPROXIMATE_512` versus 9,780 KiB for CGAL at identity. At 65, 513,
and 2,049 bits, the rows are 33,572/33,588 versus 9,792 KiB,
51,852/51,832 versus 10,692 KiB, and 116,016/116,096 versus 13,672 KiB.
Hypermesh `STRICT` is 2.11x, 3.43x, 4.85x, and 8.49x CGAL. RSS includes
different fixture front ends, exact carriers, allocators, and executable maps;
the direct requested-payload boundary remains the authoritative Hypermesh
kernel measurement. A like-for-like CGAL kernel allocator boundary remains
open.

## Call graph, footprint, and validation

The workspace call-graph utility was regenerated over exactly Hyperreal,
Hyperlattice, Hyperlimit, Hypertri, and Hypermesh, excluding concurrently
modified Hypercurve and HyperSolve. The production-source graph contains
14,607 syntactic nodes and 24,276 edges. Because this checkpoint changes no
production source, it adds no production node or edge; the graph connects the
runtime profile to the existing general predicate and exact-construction
routes. As documented by the utility, its resolver is a navigation aid rather
than compiler reachability proof.

The checkpoint adds 405 and removes 43 lines across deterministic corpus
support, examples, documentation, and tests. No production Rust module,
dependency, Cargo feature, or canonical consumer changed. All canonical
size-harness rows are therefore byte-identical to the retained coplanar-pair
checkpoint:

| Profile/consumer | Native `.text` | `wasm-opt -Oz` |
| --- | ---: | ---: |
| release/general | 2,033,826 B | 1,441,629 B |
| release/immediate | 2,036,970 B | 1,443,484 B |
| size/general | 1,080,599 B | 675,111 B |
| size/immediate | 1,081,539 B | 675,517 B |

Validation includes 118 unit, 8 Boolean, 8 executed competitive, 11 manifest,
2 intersection, 9 policy, and 2 README tests (158 executed total; six
documented opt-in/manual ignores), no-default checking, warning-denied
all-target/all-feature Clippy, warning-denied rustdoc, every fuzz-bin check,
bench compilation, release probe/exporter builds, all wide-scale both-policy
heap executions, exact CGAL input/output validation, formatting, diff checks,
and the default native/WASM size harness.

## Open work

Phase 11 still needs deeper symbolic non-rational families, distributable
external real-world pathologies, further sparse/multi-shell/pathological scale
siblings, and stage-specific arena attribution. Phase 17 retains every
absolute CGAL runtime/RSS loss in this family, especially the high-width
temporary-allocation cliff, plus torus and full YeahRight losses, broader CGAL
heap comparison, and linked-size recovery. The clean next target is to reuse
normalized arbitrary-width operands, capacities, and retained exact facts
along the existing predicate/construction paths where profiles prove reuse;
the algorithm must remain complete for fixed-word, arbitrary-rational, and
symbolic fallbacks under both policies. Phase 18 must still perform the full
requirement and exit audit.

## Reproduction

```sh
cargo build --locked --release \
  --example competitive_arrangement_probe \
  --example export_cgal_exact_off \
  --example large_mesh_heap_probe \
  --example large_mesh_kernel_heap_probe

target/release/examples/export_cgal_exact_off \
  wide_rational_boxes_2048 /tmp/hypermesh-wide-rational

taskset -c 11 target/release/examples/competitive_arrangement_probe \
  wide_rational_boxes_2048 union strict 5
taskset -c 11 target/competitive/cgal-epeck/hypermesh_cgal_epeck \
  /tmp/hypermesh-wide-rational/wide_rational_boxes_2048-left.off \
  /tmp/hypermesh-wide-rational/wide_rational_boxes_2048-right.off \
  union 21 outside

taskset -c 11 target/release/examples/large_mesh_kernel_heap_probe \
  wide-rational-2048 approximate-512
taskset -c 11 perf stat -r 11 -x, \
  -e task-clock,cycles,instructions,branches,branch-misses,cache-misses \
  target/release/examples/large_mesh_heap_probe wide-rational-2048 strict
taskset -c 11 perf record -F 999 -g --call-graph dwarf \
  -o target/phase11-17-wide-rational-2048.data -- \
  target/release/examples/large_mesh_heap_probe wide-rational-2048 strict

tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir /tmp/hypermesh-wide-rational-callgraph-src \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --per-library \
  --format json

cargo test --all-features
cargo check --no-default-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --no-deps --all-features
cargo +nightly check --manifest-path fuzz/Cargo.toml --bins
cargo bench --no-run
cargo fmt --all -- --check
benchmarks/size-harness/measure.sh default
```
