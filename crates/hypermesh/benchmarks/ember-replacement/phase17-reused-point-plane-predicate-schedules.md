# Phase 17 checkpoint: reused point/plane predicate schedules

Date: 2026-08-05

Implementation parent: Hypermesh `39250eb2`

Implementation commit: Hypermesh `fe7f143c`

Companion data: `phase17-reused-point-plane-predicate-schedules.toml`

Status: exact predicate-scheduling, performance, heap, and size checkpoint;
Phases 17 and 18 remain open

## Outcome

One polygon-pair walk now prepares each required support-plane predicate once
and borrows that short-lived schedule for every vertex classification, slice
event, and certified lazy crossing enclosure in that direction. The schedule
retains references to the plane's four exact rational coefficients and one
copy of its already certified `RationalLinearForm4Filter`. It owns no mesh
geometry, classification, topology, heap allocation, or policy result.

The forward support schedule is prepared before its triangle separator. The
reverse schedule is prepared only after the forward separator succeeds, so a
rejected pair does not perform unused reverse cache work. General polygons use
the same two directional schedules. Public one-off classifications still use
the same complete point/plane implementation through a one-call schedule.

No fixture, coordinate range, triangle count, operation, expected result,
policy name, competitor, or benchmark selects this path. There is no second
engine, retry, compatibility shim, or topology shortcut.

## Exactness and policy

- `hyperreal::Real` remains the sole construction scalar.
- The borrowed schedule changes only reuse and evaluation order. It does not
  cache or substitute a geometric conclusion.
- An unavailable rational carrier takes the unchanged general `Real` path.
- An unavailable or inconclusive certified filter takes the unchanged exact
  rational path.
- `STRICT` has no approximate terminal.
- `APPROXIMATE_512` can terminate only in Hyperlimit's existing 512-bit
  terminal, and `DecisionContext` still aggregates that use once.
- Preparing the reverse predicate after a certified forward separator cannot
  remove a decision from a surviving pair. A rejected pair has no downstream
  topology and requires no reverse predicate.

The focused unit test classifies three rational points through one schedule,
checks the certified results against the public complete path, and verifies
that the plane-filter cache records one miss and no repeated lookup. Symbolic,
wide rational, projective, and terminal-policy tests continue to cover every
fallback.

## Dispatch evidence

The strict full-resolution rotated YeahRight four-output arrangement records
1,133,738 exact-rational point classifications, of which 852,825 are resolved
by the certified rational floating filter. Those counts are unchanged from the
parent. Cache misses and capacity clears are also unchanged because the same
set of support planes is required; repeated cache hits fall because one
borrowed predicate serves the complete directional walk:

| Dispatch path | Parent | Current | Change |
| --- | ---: | ---: | ---: |
| rational filter cache misses | 32,395 | 32,395 | 0 |
| rational filter cache clears | 3 | 3 | 0 |
| rational filter cache hits | 1,130,196 | 409,009 | -721,187 (-63.81%) |
| exact-rational classifications | 1,133,738 | 1,133,738 | 0 |
| certified floating-filter classifications | 852,825 | 852,825 | 0 |

The preceding lazy-construction dispatch is likewise unchanged: 57,706 lazy
crossings are scheduled, 57,411 receive certified coordinate enclosures, and
only 5,536 materialize exact points. Predicate reuse therefore composes with,
rather than replaces, Hyperreal's retained-fact construction schedule.

## Deterministic performance

Parent and current saved release binaries were run under identical invocations
on CPU 11. Each small row is the mean hardware count from three executions of
1,000 strict arrangements materializing union, intersection, difference, and
reverse difference. Retired instructions and branches are the authority;
wall time and cycles remain frequency-sensitive.

| Fixture | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| crossing octahedra | 2,862,200,085 | 2,850,595,931 | -0.405% | 484,084,264 | 482,833,653 | -0.258% |
| overlapping boxes | 3,634,114,850 | 3,605,333,188 | -0.792% | 615,308,210 | 613,261,134 | -0.333% |
| affine boxes | 6,979,289,415 | 6,940,133,815 | -0.561% | 1,185,105,645 | 1,181,946,816 | -0.267% |
| identical boxes | 3,764,582,122 | 3,720,753,198 | -1.164% | 636,530,753 | 632,935,727 | -0.565% |

Five full-resolution runs give:

| Metric | Parent | Current | Change |
| --- | ---: | ---: | ---: |
| retired instructions | 9,915,813,025 | 9,844,090,158 | -0.723% |
| retired branches | 1,669,089,202 | 1,662,998,078 | -0.365% |
| cycles | 3,961,353,473 | 3,865,004,261 | -2.432% |
| wall median | 932.584 ms | 908.550 ms | -2.577% |

Both binaries produce the same 14,626 shared vertices and output triangle
counts `[33,512, 0, 16,756, 16,756]`, with `Certified` certainty.

Historical EMBER required 3,312.66 seconds and 329,352 KiB on the exact empty
intersection. The current approximately 0.909-second sample is about 3,646x
faster. The pinned CGAL 6.0.3 EPECK result remains approximately 0.09 seconds
and 15,516 KiB, so the roughly 10.1x runtime and 4.55x process-RSS deficits
remain open. The established current small-case CGAL deficits also remain near
2.39x crossing, 2.73x overlapping, and 1.93x affine; this checkpoint is not
large enough to claim closure from frequency-sensitive clocks.

## Large-fixture heap

The schedule is stack-borrowed and performs no allocation. Fresh requested-
payload probes nevertheless re-ran both policies on the two most informative
large rows:

| Fixture | Policy | Output | Incremental peak | Allocations | Reallocations | Added bytes | Certainty |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| full rotated YeahRight, 23,788 input triangles | STRICT | 0 triangles | 53,217,804 | 9,571,455 | 1,466,413 | 556,743,274 | Certified |
| full rotated YeahRight, 23,788 input triangles | APPROXIMATE_512 | 0 triangles | 53,217,804 | 9,571,455 | 1,466,413 | 556,743,274 | Certified |
| dense crossing grid 65, 1,572 input triangles | STRICT | 73,844 vertices / 164,068 triangles | 197,318,844 | 33,193,334 | 345,512 | 6,852,763,150 | Certified |
| dense crossing grid 65, 1,572 input triangles | APPROXIMATE_512 | 73,844 vertices / 164,068 triangles | 197,318,844 | 33,193,334 | 345,512 | 6,852,763,150 | Certified |

Every counter is byte-identical between policies and matches the preceding
checkpoint. The dense peak remains output-dominated; the full peak remains
governed by other arrangement owners.

## Source and linked size

The implementation changes 198 added and 65 removed lines across the predicate
and pairwise walker; current Hypermesh production source is 20,754 Tokei code
lines. Canonical linked size is neutral-to-smaller except one 48-byte
all-feature native size-profile movement:

| Features/profile | Parent native text | Current native text | Change | Parent optimized WASM | Current optimized WASM | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| default release | 2,038,986 | 2,038,666 | -0.016% | 1,456,412 | 1,455,415 | -0.068% |
| all-feature release | 2,176,715 | 2,175,867 | -0.039% | 1,532,843 | 1,531,580 | -0.082% |
| default size | 1,123,855 | 1,123,807 | -0.004% | 706,902 | 706,837 | -0.009% |
| all-feature size | 1,125,951 | 1,125,999 | +0.004% | 706,185 | 706,120 | -0.009% |

## Call graph and removal audit

The workspace utility scanned exactly Hyperreal, Hyperlattice, Hyperlimit,
Hypertri, and Hypermesh; Hypercurve and HyperSolve were excluded:

- production: 15,550 nodes / 26,000 edges;
- tests, examples, benches, and fuzz included: 21,953 nodes / 35,515 edges;
- 49 direct Hypermesh/Hypertri-to-Hyperlimit predicate boundaries;
- one `build_surface_arrangement -> assemble_surface_cells` edge;
- direct production edges from the pairwise walker into
  `PointPlanePredicate::new`, its certified rational classifier, and the
  unchanged exact fallback;
- zero exact EMBER, `segment_trace`, `local_bsp`, or `SurfaceSheet` namespace
  nodes.

Compact JSON evidence was generated under `/tmp` for this checkpoint. Static
call-graph resolution is navigation and removal evidence, not a substitute for
runtime, policy, corpus, or sanitizer gates.

## Measured alternatives removed

- A face-indexed support-filter table reduced full instructions only 0.093%
  beyond this checkpoint, regressed overlapping instructions 0.053%, branches
  0.209%, and cycles about 1.6%, and added one `u32` per face plus retained
  filter storage. It was removed completely.
- Retaining filters beside every source support plane lowered instructions but
  raised representative small-case cycles about 0.7--0.9% and enlarged the
  hot plane owner. Both atomic-lazy and plain construction-time variants were
  removed completely.
- A retained rational projected-point/line query improved the full case by at
  most 0.32% but regressed dense-17 by up to 0.50%. Dense dispatch showed no
  certified retained rational line signs, so the unused carrier was removed.

These are general representation experiments, not fixture branches. No
rejected carrier, helper, or compatibility path remains in the source tree.

## Validation

The accepted source passed the complete all-feature Hypermesh suite, including
163 library tests and all integration tests; the competitive suite executed 13
tests with seven documented opt-in/manual ignores. Default, no-default,
warning-denied Clippy, rustdoc, formatting, benchmark-build, fuzz-build, broad
both-policy corpus, and preceding 2,182-seed sanitizer gates remain green.

## Open work

Phases 17 and 18 remain open. In particular, every CGAL loss, external
real-world/generated fixture expansion, fuzz mutation-source coverage,
stage-specific lifetime reduction, global allocation traffic, source/native/
WASM recovery, deferred callers, and the final path/removal audit remain open.

## Reproduction

```sh
cargo test --locked --all-features
cargo check --locked --no-default-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --locked --all-features --no-deps
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo bench --locked --no-run --all-features

taskset -c 11 perf stat -r 3 -x, -e instructions:u,branches:u \
  target/release/examples/competitive_arrangement_probe \
  overlapping_boxes all strict 1000
YEAHRIGHT_BENCH=1 taskset -c 11 perf stat -r 5 -x, \
  -e instructions:u,branches:u,branch-misses:u,cycles:u -- \
  target/release/examples/competitive_arrangement_probe \
  yeahright_full_resolution_rotated_intersection all strict 1

YEAHRIGHT_BENCH=1 target/release/examples/large_mesh_kernel_heap_probe \
  yeahright-full-rotated strict
target/release/examples/large_mesh_kernel_heap_probe dense-crossing-65 strict
benchmarks/size-harness/measure.sh default
benchmarks/size-harness/measure.sh all

(cd .. && tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir /tmp/hypermesh-point-plane-schedules-callgraph-2026-08-05 \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format json)
```
