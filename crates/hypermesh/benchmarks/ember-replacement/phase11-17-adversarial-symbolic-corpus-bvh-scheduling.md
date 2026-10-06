# Phase 11/17: adversarial corpus, symbolic-policy scaling, and BVH scheduling

Date: 2026-08-05

Status: accepted checkpoint; Phases 11, 17, and 18 remain open

Hypermesh implementation: `f646d8f008c69d1cd65fd3876480af443de435e0`

Parent/evidence baseline: `209b2d22d34b02b53b4ac2d77ed765a4a17a6c47`

## Outcome

The permanent corpus now has 46 named records and gains four kinds of coverage
that were previously represented only by broad labels or one-off unit cases:

- crossing octahedra whose intersection contains no strictly contained source
  vertex, with exact topology/volume oracles and a pinned CGAL EPECK row;
- an integer affine image of overlapping boxes, which introduces oblique
  support planes while retaining exact topology/volume oracles and a pinned
  CGAL row;
- 8/64/512-cluster self-intersecting PWN siblings, each formed from separated
  pairs of transversely crossing tetrahedra, with all five Boolean outputs and
  an isolated 4,100-triangle heap selector; and
- depth 1/8/32/128 shared symbolic translations at fixed 24-triangle topology.
  Depth 1 remains `Certified`; deeper `STRICT` operations return typed
  `PredicateUndecided`, while only `APPROXIMATE_512` consumes the terminal and
  reports `Approximate512Consumed` on the correct translated arrangement.

The obsolete migration placeholder `general_nonconvex_regression_suite` is
replaced by named executable cases. The generators are ordinary deterministic
geometry constructors: production code never sees a fixture identity,
triangle count, expected answer, or competitor.

The production change removes exact `Real` comparison from BVH split-axis
selection. Node bounds remain exact. The split axis is now chosen from lossy
binary64 span estimates with a stable lower-axis tie; missing or NaN estimates
rank as zero. This choice only schedules the complete traversal. Exact node
bounds, certified filters, policy-owned consuming predicates, candidate
generation, and exhaustive leaf handling are unchanged. A poor estimate can
only produce a less balanced tree, never remove a candidate or change
certainty. This also prevents a scheduling heuristic from prematurely making
deep symbolic `STRICT` construction indeterminate.

## Correctness and policy behavior

The self-PWN family completes under both policies with byte-identical output
and `Certified` certainty. At 8/64/512 clusters, the five output triangle
counts are respectively `[132,0,128,4,132]`, `[1028,0,1024,4,1028]`, and
`[8196,0,8192,4,8196]`; exact signed volume, directed-edge balance, and shared
arrangement properties are checked for every result.

The symbolic family first proves both translated inputs are exact `Certified`
polygon soups. The depth-1 Boolean is `Certified` under both policies. At
depths 8/32/128, `STRICT` preserves the unresolved exact decision as
`PredicateUndecided`; `APPROXIMATE_512` alone produces the reference topology,
orientation, source provenance, and normalized exact volume while marking
`Approximate512Consumed`. The policy difference is therefore observed, not
normalized away.

All existing and new tests pass under default, all-feature, and
no-default-feature configurations. Warning-denied Clippy and rustdoc, every
fuzz target, every benchmark target, and every example compile. Six explicitly
documented resource-heavy tests remain ignored by their existing contract.

## Retired work against the saved parent

Saved parent and current release executables ran as adjacent whole processes
on CPU 11 with `perf stat -r 3`. Retired instructions and branches are the
primary signal because host clock frequency remains noisy. Output topology and
`Certified` certainty are identical.

| Fixture / workload | Parent instructions | Current instructions | Instruction movement | Branch movement |
| --- | ---: | ---: | ---: | ---: |
| Sparse shells 512, all four ×5 | 3,326,617,001 | 3,324,130,995 | -0.075% | -0.114% |
| Ordinary boxes, all four ×1000 | 4,454,100,972 | 4,451,946,283 | -0.048% | effectively flat |
| Dense coplanar 32, all four ×1 | 10,603,957,193 | 10,607,960,938 | +0.038% | -0.111% |
| Voxel torus 33, all four ×3 | 2,887,187,669 | 2,883,915,399 | -0.113% | +0.016% |
| 2,049-bit boxes, union ×5 | 13,913,868,176 | 13,897,147,768 | -0.120% | -0.131% |
| Full rotated YeahRight, intersection ×1 | 12,942,178,484 | 12,941,853,170 | -0.003% | -0.021% |

The scheduler is essentially neutral on these already-balanced inputs. Five
of six instruction rows improve; dense coplanar pays 0.038% instructions while
retiring 0.111% fewer branches. This bounded trade removes an exact scalar
predicate from every BVH construction and unlocks the required deep-symbolic
policy path without changing the geometric algorithm.

## New scaling families

The self-PWN family ran the shared five-result arrangement under both policies.

| Clusters | Repetitions | STRICT instructions | APPROXIMATE_512 instructions | Certainty |
| ---: | ---: | ---: | ---: | --- |
| 8 | 100 | 913,870,075 | 913,953,005 | `Certified` |
| 64 | 20 | 1,523,171,777 | 1,523,173,647 | `Certified` |
| 512 | 5 | 3,376,390,247 | 3,376,392,785 | `Certified` |

Normalized per Boolean call, retired work grows 8.333× and 8.867× for each 8×
cluster increase. The second step is still superlinear and remains a Phase 17
target.

The fixed-topology symbolic family ran the same five-result arrangement:

| Depth | Policy | Repetitions | Instructions | Per call | Result |
| ---: | --- | ---: | ---: | ---: | --- |
| 1 | `STRICT` | 100 | 22,059,480,390 | 220.59 M | `Certified` |
| 1 | `APPROXIMATE_512` | 100 | 22,059,480,569 | 220.59 M | `Certified` |
| 8 | `APPROXIMATE_512` | 50 | 10,375,726,092 | 207.51 M | terminal consumed |
| 32 | `APPROXIMATE_512` | 20 | 4,224,567,958 | 211.23 M | terminal consumed |
| 128 | `APPROXIMATE_512` | 5 | 1,545,672,369 | 309.13 M | terminal consumed |

These deliberately expose scalar construction/refinement cost: a 24-triangle
symbolic workload is expensive despite fixed topology. It is an optimization
target, not a benchmark to special-case.

## Fresh CGAL EPECK boundary

Both new common-contract cases were exported as exact OFF and run through the
pinned CGAL 6.0.3 EPECK executable for 21 repetitions. Every output was valid,
closed, structurally valid, and matched Hypermesh topology and exact volume.
Hypermesh medians cover the same shared four-result arrangement.

| Fixture | CGAL outside-copy | CGAL inside-copy | Hypermesh STRICT | Ratio | Hypermesh APPROXIMATE_512 | Ratio |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Crossing octahedra | 127,541 ns | 130,531 ns | 361,978 ns | 2.838× | 363,837 ns | 2.853× |
| Affine boxes | 384,674 ns | 386,213 ns | 802,762 ns | 2.087× | 792,780 ns | 2.061× |

The CGAL parity gate remains open. These gaps are reported directly and do not
justify a fixture-dependent path.

## Complete large-fixture heap gate

All 15 selectors ran under `STRICT` and `APPROXIMATE_512`. Every policy pair is
byte-identical and `Certified`. Against the saved parent, 13 established
selectors have flat or lower total peak; sparse shells rises 1,904 bytes
(0.0123%). Allocation calls fall for every established family. Representative
parent-to-current allocation counts are 158,393→157,412 (retained boxes),
1,656,492→1,654,929 (dense-32), 1,922,327→1,913,629 (wide-2048),
3,851,796→3,833,604 (YeahRight-8), and 14,568,430→14,542,139 (full).

The new 512-cluster self-PWN selector has 4,100 input and 8,196 union-output
triangles. Prepared input retains 964,369 bytes (963,728-byte payload), total
Boolean peak is 15,675,073 bytes, incremental kernel peak is 14,710,704 bytes,
post-Boolean retention is 1,073,024 bytes, live output payload is 1,032,968
bytes, input-attached fact growth is 40,056 bytes, and post-input residual is
50,080 bytes. It performs 441,106 allocations, 440,541 deallocations, 12,631
reallocations, adds 49,376,158 bytes, and removes 48,303,134 bytes.

## Stage-specific peak attribution

The full rotated 23,788-triangle `STRICT` row was captured in
`target/phase17-stage-attribution-full-strict.zst` and converted to Massif and
peak-stack views. Heaptrack reports 16,878,720 allocations, 3,681,081 temporary
allocations, a 165.43 MB peak, 205.41 MB peak RSS, and 6.923 seconds runtime.

At the exact 165,424,662-byte global peak (6.438 s):

- allocations retained from `build_polygon_soup_internal` account for
  122,254,248 bytes;
- `build_surface_arrangement` owns 35,975,860 live bytes, of which
  `corefine_surface` accounts for 32,732,060 bytes; and
- prepared inputs, runtime state, and the remaining arrangement machinery
  account for roughly 7.19 MB.

The later post-corefinement maximum is 158,143,590 bytes, only 7,281,072 bytes
below the global peak. The first general memory target is therefore retained
exact polygon-soup representation (122.25 MB), followed by corefinement
temporaries (32.73 MB); radial/cell machinery is not the dominant live owner at
the global peak. This is temporal ownership attribution, not an estimate from
source types.

## Code, binary size, and graph

The checkpoint changes 973 added and 80 removed lines across production,
tests, corpus data, and probes. Relative to the accepted parent, release native
text grows 848 bytes (0.043%) for both canonical consumers; optimized release
WASM grows 1,574 bytes (0.112%). The size profile improves: native text shrinks
296/304 bytes and optimized WASM shrinks 96 bytes for both consumers. Current
all-feature absolute release text is 2,123,239/2,126,079 bytes and optimized
WASM is 1,488,407/1,490,397 bytes for general/immediate consumers.

The five-crate production graph contains 14,926 nodes and 24,909 edges; the
examples/tests graph contains 17,246 nodes and 28,225 edges. The new BVH helper
has edges only to approximate-bound access, `Real::to_f64_lossy`, NaN testing,
and fixed-array construction. It has no edge to a consuming predicate or
Hyperlimit. The production graph still contains one arrangement/Boolean
engine and no EMBER path.

## Reproduction

```sh
cargo test --locked --all-features
cargo test --locked --no-default-features
cargo clippy --locked --all-targets --all-features -- -D warnings
env RUSTDOCFLAGS=-Dwarnings cargo doc --locked --no-deps --all-features
cargo check --locked --manifest-path fuzz/Cargo.toml --all-targets

taskset -c 11 perf stat -r 3 -x, -e instructions:u,branches:u \
  target/release/examples/competitive_arrangement_probe \
  transverse_self_pwn_clusters_512 all-five strict 5

env YEAHRIGHT_BENCH=1 \
  target/release/examples/large_mesh_kernel_heap_probe \
  transverse-self-pwn-clusters-512 approximate-512

benchmarks/size-harness/measure.sh default
```

The remaining corpus work is legally distributable external real-world
pathology, any uncovered matrix cells, and making every intended permanent
microcase an executable fuzz mutation source. Phase 17 next follows the
measured 122.25 MB polygon-soup ownership rather than inventing fixture-specific
shortcuts.
