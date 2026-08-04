# Phase 17 primitive wide-plane construction checkpoint

Captured 2026-08-03 on the established Ryzen 7 5800X3D / CPU 11 protocol at
Hyperreal `d5a07b65a6e7753a21fb1c8495c8bae75da8a33c` and Hypermesh
`da32f8eb10aaad8f82166e709004f5302c17688c`.

This checkpoint supersedes the retained-linear-form experiment recorded in
[`phase17-retained-wide-rational-linear-forms.md`](phase17-retained-wide-rational-linear-forms.md).
The exact-form carrier, width schedule, Hypermesh sidecar, and all supporting
dispatch are removed completely. The replacement attacks the cause at plane
construction: an exact dyadic cross product is reduced by its positive
projective scale before the plane offset is formed. Every later point/plane
predicate therefore sees the small equivalent plane instead of repeatedly
multiplying and dividing a mathematically dead common factor.

The rule is representation-driven and applies to every wide exact-dyadic plane
for which the fixed-stack constructor declines. It contains no fixture name,
operation, topology, bit threshold, expected result, or benchmark dispatch.
It is one clean construction schedule over `hyperreal::Real`, not another
Boolean path. This remains a Phase 17 optimization checkpoint; the remaining
corpus, absolute CGAL EPECK, size-recovery, stage-lifetime, Phase 17, and Phase
18 gates remain open.

## Exact construction rule

`Plane::from_points` retains its existing layered order:

1. the fixed-stack exact-dyadic constructor handles compact inputs without
   heap-backed wide arithmetic;
2. the complete exact-rational fallback forms the three cross-product
   components;
3. when those inputs are dyadic, the three-component normal is converted to a
   primitive integer ratio using one positive common scale; and
4. the exact offset is formed from that primitive normal and the already
   available exact point.

The non-dyadic rational path is unchanged. Degenerate all-zero normals remain
all zero. A single nonzero component becomes its signed unit directly, without
building a common denominator. Multi-component normals clear denominators and
remove integer content exactly. Because normalization uses one positive scale,
plane orientation, incidence, every Boolean result, and all symbolic or
Hyperlimit fallbacks are unchanged.

Hyperreal's `Rational::primitive_integer_ratio` now accepts and returns a
const-generic fixed array. This deliberate controlled-caller API replacement
eliminates the result `Vec`; no compatibility overload or shim is shipped.
The sparse one-component route also avoids a common-denominator allocation.
The same fixed-array representation serves axis-aligned and oblique planes.

`STRICT` still cannot consume a terminal approximation. `APPROXIMATE_512`
still terminates only through Hyperlimit's 512-bit policy when a predicate
genuinely reaches it. Every measured wide result is `Certified`, the terminal
is not consumed, and the two policies produce exactly equal mesh values in the
wide-rational differential corpus.

## Fixed-topology performance

The operation is union of the permanent 6,144-triangle overlapping-box family.
Current values are medians of three independent warm five-operation aggregate
means after exact import and policy-qualified PWN priming. Historical
Hypermesh and pinned CGAL EPECK values are the unchanged exact-input rows from
`phase11-17-fixed-topology-wide-rational-corpus`; CGAL was not rerun because
the checksum-pinned OFF assets and competitive executable did not change.

| Exact scale component | Historical STRICT | Current STRICT | Historical APPROXIMATE_512 | Current APPROXIMATE_512 | Current STRICT / CGAL |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 1 bit | 94.123 ms | 93.638 ms (-0.52%) | 93.283 ms | 93.761 ms (+0.51%) | 20.04x |
| 65 bits | 141.633 ms | 141.990 ms (+0.25%) | 140.274 ms | 140.747 ms (+0.34%) | 6.34x |
| 513 bits | 187.713 ms | 171.279 ms (-8.76%) | 192.244 ms | 171.123 ms (-10.99%) | 5.85x |
| 2,049 bits | 542.174 ms | 324.767 ms (-40.10%) | 540.822 ms | 323.745 ms (-40.14%) | 5.88x |

The 1- and 65-bit rows stay on the compact fixed-stack construction path and
are controls. Projective normalization begins when that constructor declines,
which explains the smooth improvement at 513 bits and the larger gain when
2,049-bit cross-product content would otherwise survive into the offset and
all repeated signs. Relative to the now-removed retained-form checkpoint, the
2,049-bit rows improve another 33.54% `STRICT` and 33.79%
`APPROXIMATE_512`.

The absolute loss to pinned CGAL remains open. The 2,049-bit ratio improves
from the historical 9.814x/9.790x and retained-form 8.846x/8.852x to
5.879x/5.860x, but this is not parity.

## Whole-process retired work

Eleven fresh-process `perf stat` repetitions of the 2,049-bit `STRICT` heap
probe include fixture preparation, exact import, PWN priming, and one Boolean.
Even with that setup dilution, the construction rule removes work throughout
the process.

| Counter | Historical | Current | Movement |
| --- | ---: | ---: | ---: |
| Task clock | 744.52 ms | 455.53 ms | -38.82% |
| Cycles | 2,970,098,453 | 1,828,497,133 | -38.44% |
| Instructions | 8,612,012,286 | 5,235,483,203 | -39.21% |
| Branches | 1,397,652,445 | 928,933,418 | -33.54% |
| Branch misses | 5,923,494 | 4,332,047 | -26.87% |
| Cache misses | 5,871,094 | 4,377,912 | -25.43% |

The movement is broad retired-work reduction, not a wall-clock-only artifact.

## Direct large-mesh heap

The allocator-instrumented kernel boundary excludes fixture preparation and
drops output and input separately. Both policies produce the same certified
2,410-vertex, 4,816-triangle union on every row.

| Scale bits | Incremental peak | Alloc calls | Realloc calls | Allocated-byte churn | Output payload | Input fact growth |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 15,807,426 B | 166,410 | 788 | 27,359,910 B | 520,472 B | 13,536 B |
| 65 | 26,906,378 B | 1,076,635 | 2,035 | 68,227,422 B | 520,472 B | 9,976 B |
| 513 | 30,816,842 B | 1,729,010 | 38,201 | 166,069,902 B | 520,472 B | 12,744 B |
| 2,049 historical | 111,848,010 B | 3,183,205 | 955,733 | 1,488,168,086 B | 520,472 B | 26,568 B |
| 2,049 current | 55,761,482 B | 2,387,525 | 477,205 | 687,328,086 B | 520,472 B | 26,568 B |

At 2,049 bits, incremental peak falls 50.15%, allocation calls fall 25.00%,
reallocations fall 50.07%, and allocated-byte churn falls 53.81%. Output
ownership and input-attached fact growth are unchanged. The 513-bit row also
falls from 46,033,482 to 30,816,842 peak bytes and from 293,106,862 to
166,069,902 churn bytes. Unlike the removed carrier, the replacement improves
peak memory as well as time.

The full-resolution rotated YeahRight intersection is an independent large
mesh control: 23,788 input triangles, 158,258,204 incremental peak bytes,
39,175,091 allocations, 4,027,656 reallocations, and 2,445,957,688 allocated
bytes. `STRICT` and `APPROXIMATE_512` both return the same empty certified
intersection. Three-run `perf stat` reports 3,048.95 ms and 34,868,105,605
instructions; one `/usr/bin/time -v` run reports 190,288 KiB maximum RSS.
Against the prior retained-coplanar-fact checkpoint, task clock improves about
4.80%, instructions improve 1.09%, and RSS falls 2,016 KiB.

## Unaffected controls

Fresh current guard timings are 0.890 ms `STRICT` and 0.843 ms
`APPROXIMATE_512` for overlapping-box union; 93.193 ms for ordinary
6,144-triangle subdivided-box union; 286.689 ms and 1,189.755 ms for the
6,144- and 24,576-triangle dense-coplanar intersections; and 94.547 ms and
405.539 ms for the 6,412- and 25,100-triangle clipped-torus intersections.
These are health controls, not comparative claims.

## Code, binary size, and graph

The two superseding implementation commits insert 120 and delete 422 lines
across Hyperreal and Hypermesh, including documentation and focused tests: a
net deletion of 302 lines. The complete default size harness is smaller than
the removed carrier checkpoint in all eight native/WASM artifacts.

| Profile / consumer | Historical native text | Current native text | Historical optimized WASM | Current optimized WASM |
| --- | ---: | ---: | ---: | ---: |
| release / general | 2,033,826 B | 2,039,878 B (+0.298%) | 1,441,629 B | 1,446,618 B (+0.346%) |
| release / immediate | 2,036,970 B | 2,043,022 B (+0.297%) | 1,443,484 B | 1,448,473 B (+0.346%) |
| size / general | 1,080,599 B | 1,082,343 B (+0.161%) | 675,111 B | 677,174 B (+0.306%) |
| size / immediate | 1,081,539 B | 1,083,291 B (+0.162%) | 675,517 B | 677,790 B (+0.336%) |

Relative to the retained-form checkpoint, current native text falls 732–1,600
bytes and optimized WASM falls 724–1,933 bytes. The residual historical growth
is retained for the much larger runtime and heap reductions and remains a
Phase 17 size-recovery target.

The five-crate call graph was regenerated over exactly Hyperreal,
Hyperlattice, Hyperlimit, Hypertri, and Hypermesh. It contains 14,631 function
nodes and 24,314 edges at
`/tmp/hypermesh-primitive-plane-final-callgraph-2026-08-03`, 38 nodes and 50
edges fewer than the removed-carrier graph. Hypercurve and HyperSolve remain
excluded.

## Removed and rejected implementations

Every losing experiment is absent from the shipped code:

- The retained `ExactRationalLinearForm4`, backend-width schedule, compact
  dyadic evaluator, exact-index sidecar, and decline sentinels are fully
  removed. Construction normalization is 33.5–33.8% faster at 2,049 bits,
  halves peak heap, and is smaller.
- A three-line equal-content quotient shortcut saved only about 0.15%
  instructions while adding 332 native text bytes. The big-integer division
  backend already handles equality; the duplicate shortcut was removed.
- A `Vec`-returning primitive-ratio prototype had the same peak but made 6,144
  extra allocations on the axis-plane corpus and carried conversion panic
  code. The fixed-array API replaced it directly with no shim.
- An iterator-based sparse scan added about 688 native text bytes. The simple
  explicit scan added about 68 bytes in the isolated artifact and was retained
  because it is clearer and smaller.
- All diagnostic dispatch counters and temporary binaries were removed.

No benchmark-specific threshold, expected-output shortcut, or distorted
algorithm remains.

## Validation

Hyperreal passes 647 unit tests and all integration and documentation targets,
no-default checking, warning-denied all-target/all-feature Clippy,
warning-denied rustdoc, all fuzz-target checks, benchmark compilation, format,
diff, and GMP public-API audit gates.

Hypermesh passes 120 unit, 8 Boolean, 8 executed competitive, 11 manifest,
2 intersection, 9 policy, and 2 README tests: 160 executed tests with six
documented competitive ignores. It also passes no-default checking,
warning-denied all-target/all-feature Clippy, warning-denied rustdoc, all fuzz
target checks, benchmark compilation, formatting, diff checks, both-policy
direct heap rows, wide/control measurements, the size harness, and the
exactness/certainty checks. Focused tests cover 2,049-bit axis and oblique
planes, primitive normals and offsets, sparse/zero/multi-component ratios, the
complete wide predicate fallback, both policies, and aggregate certainty.

## Open work

The 2,049-bit row is still roughly 5.9x pinned CGAL EPECK, ordinary exact-box
rows remain slower, and projective normalization still adds native/WASM code.
Phase 17 must continue through measured event, allocation, division, topology,
and locality costs; isolate stage lifetimes; and recover linked size without
flattening `Real` expressions or obscuring the surface-arrangement algorithm.
Phase 11 still needs external real-world, deeper-symbolic, sparse, multi-shell,
and further pathological fixtures. Phase 18 remains open until every
shared-contract competitive row and final requirement audit closes.

## Reproduction

```sh
cargo test --all-features
cargo check --no-default-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --all-features --no-deps

taskset -c 11 target/release/examples/competitive_arrangement_probe \
  wide_rational_boxes_2048 union strict 5
taskset -c 11 target/release/examples/competitive_arrangement_probe \
  wide_rational_boxes_2048 union approximate-512 5

taskset -c 11 target/release/examples/large_mesh_kernel_heap_probe \
  wide-rational-2048 strict
taskset -c 11 target/release/examples/large_mesh_kernel_heap_probe \
  wide-rational-2048 approximate-512

taskset -c 11 perf stat -r 11 -x, \
  -e task-clock,cycles,instructions,branches,branch-misses,cache-misses \
  target/release/examples/large_mesh_heap_probe wide-rational-2048 strict

benchmarks/size-harness/measure.sh

tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir /tmp/hypermesh-primitive-plane-final-callgraph-2026-08-03 \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format dot,json,mermaid,graphml \
  --per-library
```
