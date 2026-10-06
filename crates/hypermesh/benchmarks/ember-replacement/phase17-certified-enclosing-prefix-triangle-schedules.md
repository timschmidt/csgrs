# Phase 17: certified enclosing-prefix triangle schedules

Date: 2026-08-05

Status: accepted

Hypertri implementation: `73a3d9e34ee92c280731b4fcc1abb6e5515a3b5c`

Hypertri parent: `de12f48ef4ffb3c655838fdcf024879da8c12e60`

Hypermesh implementation baseline: `c0d52f7b73f7dbf927e9eeea2ee646e9e58d4cfa`

Hypermesh evidence checkpoint: `503646e41d8cadcaa5a3d7aba5c8e8d5fcaf7071`

## Outcome

Hypertri's topology-only point-set triangulation now consumes a useful order
that Hypermesh already retains: source-face vertices precede points constructed
inside or on that face. The first three points are used as a seed only after an
isolated `STRICT` three-halfspace proof establishes that they form a
nondegenerate triangle enclosing every remaining point. A negative side,
degenerate seed, multiply-zero point, or undecided strict proof declines to the
unchanged lexicographic hull-discovery algorithm.

This is a general invariant schedule, not a mesh-, operation-, size-, policy-,
or benchmark-specific dispatch. It changes neither the public topology
contract nor the final constrained-triangulation certificate. It also does not
introduce a second triangulation engine or compatibility surface.

The exact signs proved for point 3 are consumed directly to form either its
three interior triangles or its two edge-split triangles. Remaining points use
the existing exact insertion routine, constraint recovery is unchanged, and
the public API still certifies the complete constrained convex-hull topology.

## Exactness and policy behavior

The optional schedule is proved by a separate `STRICT` kernel. Therefore:

- an accepted schedule is based only on exact/certified signs;
- an undecided strict proof is only a scheduling miss and falls through;
- a declined probe cannot consume or hide an `APPROXIMATE_512` terminal;
- subsequent insertion, constraint recovery, and validation continue under the
  caller's selected policy;
- aggregate certainty remains owned by the caller's kernel and Hyperlimit;
- all 28 large-fixture policy runs completed as `Certified` with identical
  geometry and allocator metrics between `STRICT` and `APPROXIMATE_512`.

Focused tests exercise an interior fourth point, an edge fourth point, both
source windings, a later point on each boundary class, an outside-prefix
decline, and a collinear-prefix decline. Existing property, differential,
adversarial, policy, corpus, and arrangement suites exercise the complete
fallback and downstream certificate.

## Retired work

The saved parent and current release executables were run as adjacent whole
processes on CPU 11. Each row is the aggregate of three `perf stat -r 3`
processes. Counts include one fixture construction and PWN prime. Retired
instructions and branches are primary because wall clocks on this host still
show frequency drift.

| Fixture / workload | Parent instructions | Current instructions | Movement | Parent branches | Current branches | Movement |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Sparse shells 8, all ×100 | 929,721,274 | 872,559,792 | -6.148% | 172,044,137 | 161,774,415 | -5.969% |
| Sparse shells 64, all ×20 | 1,584,456,223 | 1,492,011,410 | -5.834% | 294,373,215 | 278,487,586 | -5.396% |
| Sparse shells 512, all ×5 | 3,505,894,790 | 3,322,012,546 | -5.245% | 653,684,218 | 620,090,294 | -5.139% |
| Exact boxes, all ×1000 | 4,715,173,898 | 4,453,872,949 | -5.542% | 836,824,865 | 789,920,833 | -5.605% |
| Voxel torus 33, all ×3 | 2,898,111,984 | 2,887,691,159 | -0.360% | 497,188,600 | 496,073,587 | -0.224% |
| Voxel torus 65, all ×1 | 4,719,894,211 | 4,713,254,420 | -0.141% | 810,165,703 | 809,828,482 | -0.042% |
| Subdivided boxes, union ×5 | 4,400,271,110 | 4,396,750,400 | -0.080% | 763,562,785 | 763,108,570 | -0.059% |
| Dense coplanar 32, all ×1 | 11,002,182,070 | 10,601,377,389 | -3.643% | 1,897,867,685 | 1,820,960,667 | -4.052% |
| 2,049-bit boxes, union ×5 | 13,919,908,066 | 13,913,960,935 | -0.043% | 2,537,356,998 | 2,535,887,467 | -0.058% |
| Full rotated YeahRight, intersection ×1 | 13,652,466,543 | 13,329,994,190 | -2.362% | 2,346,522,304 | 2,290,522,075 | -2.387% |

Every measured control improves. The high-genus, subdivided, and very-wide
controls remain effectively neutral where few faces satisfy the schedule;
there is no negative special case hidden behind the benchmark set.

## Callgrind attribution

One sparse-512 all-result arrangement was captured at both checkpoints. The
new artifact is
`target/phase17-strict-enclosing-prefix-triangle-sparse-512.callgrind`.

| Inclusive scope | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| Whole process | 812,872,428 | 776,316,650 | -4.497% |
| `build_surface_arrangement` | 583,280,912 | 546,726,173 | -6.267% |
| `corefine_surface` | 370,491,357 | 333,948,971 | -9.863% |
| Hypertri topology-only CDT | 149,251,219 | 112,727,597 | -24.471% |
| `triangulate_point_set` | 65,289,699 | 28,234,519 | -56.755% |

The schedule removes exact lexicographic sorting and hull discovery only after
the enclosure proof succeeds. Constraint recovery and full final validation
remain visible in the current 112.73-million-instruction CDT total.

## Paired clocks and CGAL boundary

Eleven adjacent parent/current pairs were alternated on CPU 11. Each process
ran 100, 20, or 5 operations for the 8-, 64-, or 512-shell case respectively.
The table reports independent batch medians and the median of paired movement.

| Shells | Policy | Parent median | Current median | Paired movement |
| ---: | --- | ---: | ---: | ---: |
| 8 | `STRICT` | 0.832 ms | 0.795 ms | -3.445% |
| 8 | `APPROXIMATE_512` | 0.835 ms | 0.814 ms | -3.652% |
| 64 | `STRICT` | 6.561 ms | 6.358 ms | -3.558% |
| 64 | `APPROXIMATE_512` | 6.621 ms | 6.324 ms | -4.090% |
| 512 | `STRICT` | 57.336 ms | 54.053 ms | -5.213% |
| 512 | `APPROXIMATE_512` | 56.781 ms | 53.934 ms | -4.865% |

The exact OFF inputs and pinned CGAL 6.0.3 EPECK executable are unchanged from
the immediately preceding fresh 126-record run. Its 21-call outside-copy
medians are 0.288, 2.027, and 18.228 ms; inside-copy medians are 0.279, 2.064,
and 18.231 ms. Against the outside-copy boundary, current Hypermesh is 2.77×,
3.14×, and 2.97× for `STRICT`, and 2.83×, 3.12×, and 2.96× for
`APPROXIMATE_512`. Every CGAL row and all four outputs per row were valid,
closed, and structurally valid. The CGAL parity gate remains open.

## Large-mesh heap

All fourteen designated selectors ran under both policies using the measured
`System` allocator wrapper. Every policy pair is byte-identical and
`Certified`. No peak increased. The schedule primarily removes transient exact
arithmetic and vector churn rather than retained arrangement state.

| Fixture | Total peak, parent → current | Kernel peak, parent → current | Allocations, parent → current | Reallocations, parent → current | Added bytes, parent → current |
| --- | ---: | ---: | ---: | ---: | ---: |
| `boxes-3072` | 17,053,704 → 17,053,704 | 16,004,178 → 16,004,178 | 158,745 → 158,393 | 788 → 788 | 27,268,118 → 27,248,150 |
| `boxes-3072-general` | 16,660,328 → 16,660,328 | 16,063,730 → 16,063,730 | 165,640 → 165,288 | 788 → 788 | 28,982,822 → 28,962,854 |
| `dense-coplanar-16` | 19,749,525 → 19,749,525 | 18,719,560 → 18,719,560 | 480,328 → 412,744 | 30,845 → 30,845 | 81,002,090 → 77,168,234 |
| `dense-coplanar-32` | 78,750,901 → 78,750,901 | 74,724,848 → 74,724,848 | 1,926,828 → 1,656,492 | 123,026 → 123,026 | 324,327,706 → 308,992,282 |
| `sparse-shells-512` | 15,540,315 → 15,540,195 | 14,578,158 → 14,578,038 | 463,128 → 440,600 | 14,099 → 12,563 | 50,801,084 → 48,862,652 |
| `wide-rational-64` | 22,334,270 → 22,334,270 | 21,716,146 → 21,716,146 | 648,761 → 648,217 | 4,903 → 4,903 | 48,724,182 → 48,693,462 |
| `wide-rational-512` | 23,223,527 → 23,223,527 | 22,596,194 → 22,596,194 | 1,651,394 → 1,650,338 | 39,812 → 39,812 | 127,939,350 → 127,866,646 |
| `wide-rational-2048` | 31,898,360 → 31,898,360 | 31,234,658 → 31,234,658 | 1,923,383 → 1,922,327 | 221,132 → 221,132 | 432,389,422 → 432,193,838 |
| `voxel-torus-33` | 17,657,096 → 17,657,096 | 16,589,238 → 16,589,238 | 179,307 → 177,877 | 925 → 795 | 34,732,040 → 34,587,868 |
| `voxel-torus-65` | 70,860,620 → 70,860,620 | 66,042,050 → 66,042,050 | 691,939 → 689,101 | 1,776 → 1,518 | 150,751,435 → 150,464,407 |
| `yeahright` | 5,971,801 → 5,971,681 | 5,054,548 → 5,054,428 | 209,892 → 206,147 | 10,611 → 10,494 | 13,660,389 → 13,418,317 |
| `yeahright-4` | 22,494,285 → 22,494,165 | 18,855,430 → 18,855,310 | 878,096 → 869,517 | 38,026 → 37,777 | 53,598,754 → 53,057,714 |
| `yeahright-8` | 86,054,229 → 86,054,229 | 71,528,974 → 71,528,974 | 3,869,544 → 3,851,796 | 146,205 → 145,554 | 217,024,316 → 215,926,164 |
| `yeahright-full-rotated` | 165,353,622 → 165,353,502 | 158,230,228 → 158,230,108 | 14,972,165 → 14,568,430 | 2,232,013 → 2,201,838 | 834,548,872 → 811,156,748 |

The full 23,788-triangle case saves 403,735 allocation calls, 30,175
reallocations, and 23,392,124 allocated payload bytes while reducing measured
peak by 120 bytes.

## Code and binary size

The implementation adds 139 Rust lines including its path tests and removes no
lines. Tests do not enter shipped artifacts. The canonical size harness reports
the following small production cost relative to the parent checkpoint.

| Profile / consumer | Native text, parent → current | Movement | `wasm-opt -Oz`, parent → current | Movement |
| --- | ---: | ---: | ---: | ---: |
| Release / general | 1,984,454 → 1,985,406 | +952 B (+0.048%) | 1,409,155 → 1,410,475 | +1,320 B (+0.094%) |
| Release / immediate | 1,987,598 → 1,988,550 | +952 B (+0.048%) | 1,411,023 → 1,412,345 | +1,322 B (+0.094%) |
| Size / general | 1,080,247 → 1,081,775 | +1,528 B (+0.141%) | 674,595 → 676,121 | +1,526 B (+0.226%) |
| Size / immediate | 1,081,211 → 1,082,739 | +1,528 B (+0.141%) | 675,008 → 676,756 | +1,748 B (+0.259%) |

The sub-0.1% release growth is accepted for a 24.47% CDT reduction and broad
whole-operation wins. No compatibility shim, alternate engine, or duplicated
algorithm is shipped.

## Call-graph and validation audit

The production graph contains 14,925 nodes and 24,904 edges. The graph with
examples and tests contains 17,207 nodes and 28,131 edges. The new edges show
one optional schedule from `triangulate_point_set`, its strict enclosure proof,
the existing `insert_point` continuation, and the unchanged fallback edges to
lexicographic order, convex hull, insertion, and validation.

The exact checkpoint passes:

- Hypertri: 78 library, 25 adversarial, 6 differential, 10 fuzz-property, 5
  policy, 2 README, and 4 rustdoc tests;
- Hypermesh: 185 default and 186 all-feature executed tests, with six documented
  opt-in/manual YeahRight tests ignored;
- warning-denied all-target/all-feature Clippy for both affected crates;
- no-default checks and warning-denied rustdoc for both crates;
- every Hypermesh fuzz binary and every Hypermesh/Hypertri benchmark target;
- formatting, diff, canonical size, paired perf, Callgrind, both call graphs,
  and all 14 × 2 large heap runs.

## Reproduction

```sh
cargo test --locked --all-features
cargo clippy --locked --all-targets --all-features -- -D warnings

taskset -c 11 perf stat -r 3 -x, \
  -e instructions:u,branches:u \
  target/release/examples/competitive_arrangement_probe \
  sparse_multishell_tetrahedra_512 all strict 5

valgrind --tool=callgrind \
  --callgrind-out-file=target/phase17-strict-enclosing-prefix-triangle-sparse-512.callgrind \
  target/release/examples/competitive_arrangement_probe \
  sparse_multishell_tetrahedra_512 all strict 1

env YEAHRIGHT_BENCH=1 \
  target/release/examples/large_mesh_kernel_heap_probe \
  yeahright-full-rotated approximate-512

benchmarks/size-harness/measure.sh default

tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir hypermesh/target/callgraph-hypermesh-enclosing-prefix-schedule \
  --format json \
  --crate-name hypermesh,hyperreal,hyperlimit,hypertri,hyperlattice \
  --include-examples --include-tests --per-library
```

Phase 17 and Phase 18 remain open. The next clean target is the remaining
112.73-million-instruction topology-only CDT/corefinement path, while CGAL
parity, broader real-world/deeper-symbolic corpus coverage, and stage-specific
heap attribution remain explicit gates.
