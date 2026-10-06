# Phase 11/17 bounded-coordinate dense-coplanar corpus checkpoint

Captured 2026-08-03 on the established Ryzen 7 5800X3D / CPU 11 protocol at
Hypermesh `9b3dd0b981f67dc7471438f01c1d01c66096e76b`.

This checkpoint closes the named dense-coplanar and bounded-input-coordinate
scaling corpus gaps and adds current exact CGAL EPECK, whole-kernel heap, RSS,
and hardware-counter evidence. It does not claim Phase 11 corpus completion,
Phase 17 performance completion, Phase 18 completion, or whole-corpus CGAL
parity.

## Result

One deterministic generator builds two geometrically identical exact boxes.
The operands use opposite diagonals on every authored face and then the same
power-of-two triangular surface subdivision. Every face therefore enters the
ordinary positive-area cross-operand coplanar overlay; coincident authored
triangulation cannot satisfy the result by identity alone.

The three permanent points contain 384, 6,144, and 24,576 total input
triangles. Every authored input coordinate is a binary64 dyadic whose reduced
denominator is at most eight across the whole family. The family consequently
scales arrangement topology without crossing the growing-coordinate storage
threshold exposed by the voxel-torus family.

There is no production-code change in this checkpoint. Fixture selection
exists only in benchmark/export/heap executables. The production Boolean
engine does not inspect fixture names, coordinates, counts, operations,
policies, expected output, or measurement state.

## Permanent geometry and policy contract

For `d` surface divisions, each operand contains `12*d*d` triangles and the
pair contains `24*d*d`. The expected exact intersection and union are the same
4-by-4-by-4 box; both differences and XOR are empty. Opposite diagonals force
the shared arrangement to double each authored triangulation into the common
overlay shown by the output counts.

| Fixture | Tier | Total input T | Exact output V/T | Exact volumes U/I/L-R/R-L/XOR |
| --- | --- | ---: | ---: | --- |
| `dense_coplanar_boxes_4` | medium | 384 | 194 / 384 | 64 / 64 / 0 / 0 / 0 |
| `dense_coplanar_boxes_16` | large | 6,144 | 3,074 / 6,144 | 64 / 64 / 0 / 0 / 0 |
| `dense_coplanar_boxes_32` | XL | 24,576 | 12,290 / 24,576 | 64 / 64 / 0 / 0 / 0 |

The medium regression evaluates all five truth-table outputs through one
arrangement under `STRICT` and `APPROXIMATE_512`. It requires exact policy
output equality, `Certified` aggregate certainty, expected triangle counts,
finite nondegenerate facets, directed-edge balance, and exact volume. The
large and XL heap rows execute intersection under both policies and reproduce
the expected topology with policy-identical requested-payload metrics. No
terminal approximate decision is consumed.

CGAL's four-output shared call reports every requested result valid, closed,
and structurally valid at all three scales, with the same vertex, triangle,
and volume results. Its XOR adapter is represented by the two empty directed
difference outputs; Hypermesh additionally checks its single empty XOR mesh.

## Identical reduced-rational CGAL inputs

`export_cgal_exact_off` serializes `Real::exact_rational`, not a rounded
display approximation. CGAL 6.0.3 EPECK at
`cefe3007d59cf4292a09da4fa8a35f38478a4e0b` therefore consumes the same exact
authored coordinates.

| Divisions | Left SHA-256 | Right SHA-256 |
| ---: | --- | --- |
| 4 | `f48faf36d826aae30605c973bd89ca6560609250342e2a062e7e49757a4562f3` | `1153aa0089997699cbe18fa4557dc9d560ae1d1635e85d5daa87c067d7e1dca6` |
| 16 | `180adda7add394332e99ee76cf5eeb5638128abdc77c432bdf83b3dfd44b2c31` | `23c8030799fe5ee879fa0ba59088ff0708903050ece37690886a23d95c25ed59` |
| 32 | `eab79a262242a13460247a2595360e51984cc4b753d21d5192529dd7ebd68de1` | `9818395cb2ba5e89e518903cf4e480bbfd2f4eba2158125fbe3d542bacf1f732` |

## Current Hypermesh versus CGAL EPECK

Both executables use one thread pinned to CPU 11. Each Hypermesh value is the
median of three independent aggregate means after exact import and
policy-qualified PWN priming: 31 calls at the medium point, 11 at large, and 5
at XL. CGAL is the median of 31/11/5 calls to
`corefine_and_compute_boolean_operations`, with its required mutable input
copy outside the timed interval. The operation is intersection in both
engines.

| Input T | Hypermesh STRICT | Hypermesh APPROXIMATE_512 | CGAL median (min-max) | STRICT versus CGAL | APPROX versus CGAL |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 384 | 17.230 ms | 17.364 ms | 16.036 ms (15.910-17.749) | 7.44% slower | 8.28% slower |
| 6,144 | 289.770 ms | 289.386 ms | 330.925 ms (322.472-345.582) | 12.44% faster | 12.55% faster |
| 24,576 | 1,192.773 ms | 1,186.641 ms | 1,432.353 ms (1,424.605-1,484.692) | 16.73% faster | 17.15% faster |

The medium-to-large step grows input triangles 16x and `STRICT` time 16.818x;
the large-to-XL step grows triangles 4x and time 4.116x. CGAL grows 20.636x
and 4.328x over the same steps. Hypermesh's crossover is therefore a measured
scaling property of the existing general arrangement and retained-fact
schedule, not a benchmark-conditioned fast path.

The medium loss remains an open per-case runtime target. The large and XL
runtime targets are closed for this family only. The family did not exist in
the historical EMBER benchmark, so no historical speedup is invented. The
established historical YeahRight and overlapping-box rows remain the valid
historical scorecards, including their still-open CGAL deficits.

## Runtime slope

Eleven fresh-process `perf stat` repetitions use the ordinary uninstrumented
probe and include fixture construction, exact import, PWN priming, and one
production Boolean call.

| Input T | Task clock | Cycles | Instructions | Branches | Branch misses | Cache misses |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 6,144 | 311.57 ms | 1.279B | 3.622B | 646.214M | 3.818M | 2.184M |
| 24,576 | 1,285.52 ms | 5.287B | 14.654B | 2.617B | 13.734M | 9.208M |

For 4x input triangles, task clock grows 4.1259x, cycles 4.1323x,
instructions 4.0461x, branches 4.0493x, branch misses 3.5976x, and cache
misses 4.2163x. Retired work is close to linear; the cache and clock excess
remain measurable rather than being rounded away.

## Direct large-mesh heap

The allocator-instrumented executable measures successful requested Rust
allocation payload only. Preparation and optional input priming finish before
the Boolean interval; output and input are dropped separately afterward. Each
displayed row was reproduced in separate `STRICT` and `APPROXIMATE_512`
processes with identical bytes, event counts, topology, and `Certified`
certainty.

| Input T | Prepared input | Incremental kernel peak | Output payload | Input fact growth | Alloc calls | Realloc calls | Allocated bytes |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 6,144 | 636,048 B | 18,719,560 B | 663,896 B | 40,168 B | 568,287 | 30,845 | 84,818,330 B |
| 24,576 | 2,452,488 B | 74,724,848 B | 2,654,552 B | 72,368 B | 2,237,772 | 123,026 | 337,947,810 B |

For 4x triangles, incremental peak grows 3.9918x and falls slightly from
3,046.8 to 3,040.6 bytes per input triangle. Allocation calls grow 3.9377x,
reallocations 3.9885x, allocated-byte churn 3.9844x, and input-attached fact
growth only 1.8016x. This fixed-coordinate control therefore has a near-linear
whole-kernel requested-payload slope and no fact-lifetime cliff.

Fresh-process maximum RSS is 24,500/24,708 KiB for Hypermesh
`STRICT`/`APPROXIMATE_512` versus 27,056 KiB for CGAL at 6,144 triangles. At
24,576 triangles it is 83,652/83,520 KiB versus 89,212 KiB. Hypermesh's
`STRICT` rows use 9.45% and 6.23% less RSS. RSS includes different fixture
front ends, exact carriers, allocator implementations, and executable maps;
the direct requested-payload boundary above remains the authoritative
Hypermesh kernel measurement. A like-for-like CGAL kernel allocator boundary
is still open.

## Footprint and validation

No production Rust module, dependency, Cargo feature, or canonical consumer
changed. The checkpoint adds 284 and removes 7 lines in deterministic
benchmark support, examples, manifest/documentation, and tests. All canonical
size-harness rows are byte-identical to the retained coplanar-pair checkpoint:

| Profile/consumer | Native `.text` | `wasm-opt -Oz` |
| --- | ---: | ---: |
| release/general | 2,033,826 B | 1,441,629 B |
| release/immediate | 2,036,970 B | 1,443,484 B |
| size/general | 1,080,599 B | 675,111 B |
| size/immediate | 1,081,539 B | 675,517 B |

Validation includes 118 unit, 8 Boolean, 7 executed competitive, 10 manifest,
2 intersection, 9 policy, and 2 README tests (156 executed total; six
documented opt-in/manual ignores), no-default checking, warning-denied
all-target/all-feature Clippy, warning-denied rustdoc, every fuzz-bin check,
bench compilation, release probe/exporter builds, four large/XL both-policy
heap executions, exact CGAL output validation, formatting, diff checks, and
the default native/WASM size harness.

## Open work

Phase 11 still needs legally distributable external real-world pathologies,
broader high-bit/deep-symbolic families, further sparse/multi-shell/pathological
scale siblings, and stage-specific arena attribution. Phase 17 still has the
medium member's CGAL loss, large torus and full YeahRight runtime/RSS deficits,
broader per-case CGAL heap comparison, cache-slope cleanup, and linked-size
recovery. This family-level win does not close any losing case. Phase 18 must
still perform the complete requirement and exit audit.

## Reproduction

```sh
cargo build --locked --release \
  --example competitive_arrangement_probe \
  --example export_cgal_exact_off \
  --example large_mesh_heap_probe \
  --example large_mesh_kernel_heap_probe

target/release/examples/export_cgal_exact_off \
  dense_coplanar_boxes_32 /tmp/hypermesh-dense-coplanar

taskset -c 11 target/release/examples/competitive_arrangement_probe \
  dense_coplanar_boxes_32 intersection strict 5
taskset -c 11 target/competitive/cgal-epeck/hypermesh_cgal_epeck \
  /tmp/hypermesh-dense-coplanar/dense_coplanar_boxes_32-left.off \
  /tmp/hypermesh-dense-coplanar/dense_coplanar_boxes_32-right.off \
  intersection 5 outside

taskset -c 11 target/release/examples/large_mesh_kernel_heap_probe \
  dense-coplanar-32 approximate-512
taskset -c 11 perf stat -r 11 -x, \
  -e task-clock,cycles,instructions,branches,branch-misses,cache-misses \
  target/release/examples/large_mesh_heap_probe dense-coplanar-32 strict

cargo test --all-features
cargo check --no-default-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --no-deps --all-features
cargo +nightly check --manifest-path fuzz/Cargo.toml --bins
cargo bench --no-run
cargo fmt --all -- --check
benchmarks/size-harness/measure.sh default
```
