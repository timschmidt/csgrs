# Phase 17 affine wide edge-plane checkpoint

Captured 2026-08-03 on the established Ryzen 7 5800X3D / CPU 11 protocol at
Hyperreal `f59a6ce7206354f33f73f8956e3c1e5261a20893` and Hypermesh
`5f9e45d21ded8a6aad46764f82fb396c6930cc0a`.

This checkpoint extends the primitive support-plane construction recorded in
[`phase17-primitive-wide-plane-construction.md`](phase17-primitive-wide-plane-construction.md)
to the three oriented edge planes of every source triangle. An edge vector
formed from exact coordinates can retain the input mesh's common affine scale
even after its support plane has become primitive. Carrying that scale into an
edge-plane offset and every later classification is exact but needlessly
expensive. The retained rule removes it once, before the offset is formed.

The optimization is scheduled from one immutable representation fact: all
exact-dyadic displacements from a mesh anchor have a common numerator content
wider than one native word. This is translation invariant and independent of
fixture name, operation, topology, expected output, or a specific bit count.
A wide denominator alone is coordinate resolution, not evidence that
normalization will reduce later products, so it deliberately stays on the
general path. Declining the optimization never declines the Boolean operation;
the existing complete `Real` edge-plane construction runs unchanged.

This is one clean construction schedule inside the path-complete surface
arrangement. It is not another Boolean engine or a benchmark shortcut. Phase
17 remains open because every measured wide row still loses to pinned CGAL
EPECK, and Phase 18 remains open with the broader competitive and final-audit
gates.

## Exact construction and scheduling rule

For a native mesh, polygon-soup preparation computes and caches one ternary
fact: unknown, decline, or normalize. The fact scan:

1. chooses the first exact point as an affine anchor;
2. obtains the reduced unsigned numerator of each exact-dyadic coordinate
   displacement without entering Hyperreal's retained add/subtract path;
3. accumulates the exact numerator GCD through Hyperreal's existing
   mixed-width GCD schedule;
4. declines immediately once that GCD fits one native word, because a GCD can
   only shrink; and
5. enables edge-plane normalization only when the complete scan retains
   multi-limb common numerator content.

The cached state is one `AtomicU8` placed in existing `TriangleMeshFacts`
padding. Relaxed ordering is sufficient because the value is a deterministic
property of immutable positions and publishes no associated data. Concurrent
first consumers may repeat the read-only scan and store the same result. Raw
borrowed views compute the same conservative fact without a compatibility
carrier. Public triangle and quad constructors use the identical rule.

When enabled, each oriented edge plane retains the existing construction
order: subtract endpoints, cross the edge with the already primitive support
normal, convert the exact-dyadic normal to one primitive integer ratio using a
positive scale, and form the offset through the authored endpoint. Point-plane
incidence, orientation, interior halfspaces, and every downstream Boolean
decision are therefore unchanged.

Hyperreal adds two structural exact queries for this layered schedule:

- `dyadic_difference_numerator_magnitude` computes the reduced magnitude
  directly from canonical dyadic numerators, denominator shifts, and signs;
  it intentionally does not populate bounded retained linear-result slots;
- `numerator_magnitude_gcd` exposes the existing mixed-width GCD dispatcher as
  a positive exact integer content query without leaking `BigUint` ownership
  into Hypermesh.

Both queries are general exact-rational representation operations. They have
no mesh, fixture, policy, or benchmark knowledge.

`STRICT` still cannot consume terminal approximation. `APPROXIMATE_512` still
terminates only through Hyperlimit's approximate 512-bit equality policy when
a predicate actually reaches it. Every measured row in this checkpoint is
exactly equal between policies and remains `Certified`; the approximate
terminal is not consumed.

## Fixed-topology warm runtime

The operation is union of the permanent 6,144-triangle overlapping-box
family. Each Hypermesh value is the median of three independent warm
five-operation aggregate means after exact import and PWN priming. The parent
is the immediately preceding primitive-plane implementation at Hypermesh
`da32f8eb`. CGAL 6.0.3 EPECK was rerun for 21 repetitions with input copying
outside the interval on freshly exported exact rational OFF files.

| Exact scale component | Parent STRICT | Current STRICT | Parent APPROXIMATE_512 | Current APPROXIMATE_512 | CGAL EPECK | Current STRICT / CGAL |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 65 bits | 139.688 ms | 124.727 ms (-10.71%) | 144.948 ms | 125.214 ms (-13.61%) | 22.304 ms | 5.59x |
| 513 bits | 166.881 ms | 166.251 ms (-0.38%) | 167.066 ms | 163.551 ms (-2.10%) | 29.070 ms | 5.72x |
| 2,049 bits | 328.752 ms | 245.495 ms (-25.33%) | 323.994 ms | 245.597 ms (-24.20%) | 55.261 ms | 4.44x |

At 2,049 bits the original fixed-topology checkpoint was 542.174/540.822 ms,
so the complete support-plus-edge construction campaign improves warm
`STRICT`/`APPROXIMATE_512` by 54.72%/54.59%. The exact CGAL gap narrows from
9.81x historically and 5.88x at the preceding checkpoint to 4.44x. This is a
large slope improvement, not parity.

## Whole-process retired work

Eleven fresh-process `perf stat` repetitions include fixture construction,
exact import, the complete affine-content scan, PWN priming, and one 2,049-bit
`STRICT` union.

| Counter | Primitive-plane parent | Current | Movement |
| --- | ---: | ---: | ---: |
| Task clock | 455.39 ms | 370.23 ms | -18.70% |
| Cycles | 1,829,281,954 | 1,506,682,648 | -17.64% |
| Instructions | 5,235,520,029 | 4,195,402,389 | -19.87% |
| Branches | 928,942,604 | 780,805,097 | -15.95% |
| Branch misses | 4,322,528 | 3,949,145 | -8.64% |
| Cache misses | 4,263,749 | 2,878,577 | -32.49% |

The setup-inclusive counters confirm that the one-time exact representation
scan pays for itself; the result is not an artifact of excluding cold work.

## Direct large-mesh heap

The allocator-instrumented kernel boundary excludes fixture preparation and
drops output and input separately. Both policies are byte-identical on every
current row and produce the same certified 2,410-vertex, 4,816-triangle union.
Allocated-byte churn below is the allocator's total added bytes during the
Boolean interval.

| Scale bits | Metric | Primitive-plane parent | Current | Movement |
| ---: | --- | ---: | ---: | ---: |
| 65 | Incremental peak | 26,906,378 B | 21,519,562 B | -20.02% |
| 65 | Allocation calls | 1,076,635 | 693,333 | -35.60% |
| 65 | Reallocation calls | 2,035 | 4,903 | +140.93% |
| 65 | Allocated-byte churn | 68,227,422 B | 50,749,998 B | -25.62% |
| 513 | Incremental peak | 30,816,842 B | 22,399,610 B | -27.31% |
| 513 | Allocation calls | 1,729,010 | 1,833,830 | +6.06% |
| 513 | Reallocation calls | 38,201 | 44,345 | +16.08% |
| 513 | Allocated-byte churn | 166,069,902 B | 136,500,670 B | -17.81% |
| 2,049 | Incremental peak | 55,761,482 B | 31,038,074 B | -44.34% |
| 2,049 | Allocation calls | 2,387,525 | 2,109,647 | -11.64% |
| 2,049 | Reallocation calls | 477,205 | 228,217 | -52.18% |
| 2,049 | Allocated-byte churn | 687,328,086 B | 463,318,390 B | -32.59% |

Output payload stays exactly 520,472 bytes. Input-attached fact growth stays
9,976, 12,744, and 26,568 bytes. The 65-bit full affine scan retains 120 bytes
of canonical input facts before the kernel interval; the 513- and 2,049-bit
input payloads are byte-identical to the parent. `TriangleMeshFacts` itself
does not grow because the cached schedule occupies padding.

The 65- and 513-bit reallocation counts are explicit Pareto losses even though
peak and total churn improve materially; they remain visible rather than being
hidden by the stronger 2,049-bit row.

## Ordinary-width and heterogeneous controls

The representation schedule declines cleanly on the ordinary 6,144-triangle
row. Nine paired warm five-operation aggregates report 102.553 ms current
versus 102.019 ms parent (+0.52%). Eleven five-operation `perf stat` processes
report 6,022,350,821 versus 6,018,600,210 instructions (+0.0623%) and
1,070,736,284 versus 1,070,278,900 branches (+0.0427%). Task-clock movement is
+0.82%, inside the two measurements' approximately 0.61%/0.67% relative
intervals. The direct both-policy heap remains exactly 15,807,426 peak bytes,
166,410 allocations, 788 reallocations, 27,359,910 allocated bytes, 520,472
output bytes, and 13,536 input-fact bytes.

The full rotated YeahRight intersection is the independent heterogeneous
control: 23,788 input triangles and an exact empty certified result. Current
versus parent has identical 158,258,204-byte incremental peak, retained input,
output, reallocation, and drop boundaries. Current performs eight additional
allocation calls and adds 256 bytes, changes instructions by +0.0025% and
branches by -0.0018%, and measures 3,020.41 versus 3,038.25 ms in separate
three-run batches. There is no material full-mesh regression or claimed win.

## Rejected schedules and retained-fact discipline

Two correct prototypes were measured and completely removed:

- Enabling normalization for any wide dyadic coordinate improved the synthetic
  wide family but also normalized the heterogeneous YeahRight mesh, whose
  edge normals had wide denominators and only word-sized numerator content.
  It raised full-mesh instructions about 1.21%, branches 1.32%, allocation
  calls 1.90%, and allocated-byte churn 1.03%. The broad gate is absent.
- The first affine-content scan used ordinary rational subtraction. Hyperreal
  correctly treated those operations as reuse evidence and populated bounded
  retained linear-result caches on source coordinates, but the scheduling
  probe then displaced arithmetic the Boolean kernel reused. Wide-kernel
  allocations and churn rose about 6%. The structural dyadic-difference query
  replaces that probe and restores the improved kernel metrics exactly.

These rejections are why the final rule tests reusable numerator content and
why one-shot representation analysis bypasses retained arithmetic slots. The
design plays to Hyperreal's retained facts instead of flattening expressions or
polluting its scheduler.

No diagnostic dispatch counter, trace wrapper, fixture branch, output shortcut,
alternate edge-plane implementation, or compatibility shim remains.

## Code, linked size, and graph

The implementation commits add a net 308 source lines across production,
focused tests, documentation, and the GMP API audit. The performance win costs
0.20–0.32% in the canonical native/WASM consumers relative to the immediately
preceding checkpoint.

| Profile / consumer | Parent native text | Current native text | Parent optimized WASM | Current optimized WASM |
| --- | ---: | ---: | ---: | ---: |
| release / general | 2,039,878 B | 2,044,326 B (+0.218%) | 1,446,618 B | 1,450,878 B (+0.295%) |
| release / immediate | 2,043,022 B | 2,047,470 B (+0.218%) | 1,448,473 B | 1,452,733 B (+0.294%) |
| size / general | 1,082,343 B | 1,084,559 B (+0.205%) | 677,174 B | 679,339 B (+0.320%) |
| size / immediate | 1,083,291 B | 1,085,499 B (+0.204%) | 677,790 B | 679,746 B (+0.289%) |

The linked growth remains a Phase 17 recovery target and is retained because
the setup-inclusive 2,049-bit instruction, runtime, peak, and churn reductions
are much larger. The implementation uses one cached fact and one construction
route rather than a second narrow/wide Boolean engine.

The regenerated five-crate production/test graph contains 14,672 function
nodes and 24,391 edges at
`/tmp/hypermesh-affine-edge-plane-final-callgraph-2026-08-03`. It includes the
direct mesh-fact-to-affine-scan, structural Hyperreal queries, and edge-plane
constructor paths. Hypercurve and HyperSolve are excluded.

## Validation

Hyperreal passes 649 unit tests plus every integration and documentation target,
the GMP public-API audit, no-default checking, warning-denied all-target/all-
feature Clippy, docs, all fuzz-target checks, benchmark compilation, formatting,
and diff checks.

Hypermesh passes 122 unit, 8 Boolean, 8 executed competitive, 11 manifest,
2 intersection, 9 policy, and 2 README tests: 162 executed tests with six
documented competitive ignores. It also passes all-feature and no-default
builds, warning-denied all-target/all-feature Clippy, docs, all fuzz-target
checks, benchmark compilation, the complete size harness, current CGAL runs,
both-policy direct heap rows, formatting, and diff checks.

Focused tests cover positive projective equivalence of support and all three
edge planes, axis/sparse and multi-component normals, translated wide affine
coordinates, denominator-only decline, structural dyadic differences across
sign/denominator/zero/non-dyadic cases, concurrent fact safety by construction,
both policies, exact equality, and aggregate certainty.

## Open work

The 2,049-bit row remains 4.44x pinned CGAL EPECK; the 65- and 513-bit rows
remain 5.59x and 5.72x. Ordinary exact boxes remain a larger absolute ratio.
The 65- and 513-bit reallocation counts and the 0.20–0.32% linked growth are
also open Pareto dimensions. Phase 17 must continue with general candidate,
event, arithmetic, allocation, topology, locality, and stage-lifetime profiles
without obscuring the surface-arrangement algorithm. Phase 11 still needs the
remaining external real-world/deeper-symbolic/sparse/multi-shell corpus. Phase
18 remains open until every shared-contract competitive row and final audit
closes.

## Reproduction

```sh
cargo test --all-features
cargo check --no-default-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --all-features --no-deps
cargo check --manifest-path fuzz/Cargo.toml --all-targets
cargo bench --no-run --all-features

taskset -c 11 target/release/examples/competitive_arrangement_probe \
  wide_rational_boxes_2048 union strict 5
taskset -c 11 target/release/examples/competitive_arrangement_probe \
  wide_rational_boxes_2048 union approximate-512 5

target/release/examples/export_cgal_exact_off \
  wide_rational_boxes_2048 /tmp/hypermesh-affine-edge-cgal
taskset -c 11 target/competitive/cgal-epeck/hypermesh_cgal_epeck \
  /tmp/hypermesh-affine-edge-cgal/wide_rational_boxes_2048-left.off \
  /tmp/hypermesh-affine-edge-cgal/wide_rational_boxes_2048-right.off \
  union 21 outside

taskset -c 11 target/release/examples/large_mesh_kernel_heap_probe \
  wide-rational-2048 strict
taskset -c 11 target/release/examples/large_mesh_kernel_heap_probe \
  wide-rational-2048 approximate-512

taskset -c 11 perf stat -r 11 \
  -e task-clock,cycles,instructions,branches,branch-misses,cache-misses \
  target/release/examples/large_mesh_heap_probe wide-rational-2048 strict

benchmarks/size-harness/measure.sh

tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir /tmp/hypermesh-affine-edge-plane-final-callgraph-2026-08-03 \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format dot,json,mermaid,graphml \
  --per-library
```
