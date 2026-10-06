# Phase 17: exact support-line interval intersections

Captured 2026-08-04. The retained implementation is Hypermesh `cfc5ae62`,
based on the certified-BVH checkpoint `10dc6533`.

## Result

Nonparallel convex-polygon intersection now has one exact geometric route:

1. Prove a nonzero coordinate of the support-line direction from the cross
   product of the two support normals.
2. Intersect each closed convex polygon with the other polygon's support
   plane, producing one closed zero- or one-dimensional interval on the
   common support line.
3. Intersect those two closed intervals exactly.
4. Materialize only the surviving one or two endpoints.

This replaces the historical narrow-phase sequence that collected every
edge-plane candidate, constructed each affine point, tested each candidate
against the other polygon, retried through projective four-plane determinants,
and deduplicated the survivors. The replacement is a standard convex-slice
algorithm, not a benchmark shortcut. It contains no fixture, triangle-count,
coordinate-width, operation, expected-output, or alternate-engine branch.

The full 23,788-triangle YeahRight exact-empty row loses 9.38% of instructions,
8.28% of branches, and 9.04% of median wall time from the accepted parent.
All four independent controls improve. Release native text shrinks 61,192
bytes and optimized release WASM shrinks 56,252 bytes. Full and 2,049-bit
incremental heap peaks remain byte-identical. The complete 13-selector heap
matrix is exactly policy-equal.

Phase 17 and Phase 18 remain open. The current full row is still 19.00x slower
and 12.25x larger in fresh-process RSS than the pinned CGAL 6.0.3 EPECK row;
ordinary exact boxes remain 4.81x/4.53x slower at the two copy boundaries.

## Exactness and completeness argument

Let the two nonparallel support planes be `P` and `Q`, and let their common
line be `L`. For a closed convex polygon `A` contained in `P`, `A ∩ Q` equals
`A ∩ L` and is therefore empty, one point, or one closed interval. Its extrema
are exactly the on-plane vertices and proper edge/plane crossings encountered
by a complete boundary walk. The same holds for polygon `B`. Consequently,

`A ∩ B = (A ∩ L) ∩ (B ∩ L)`,

so the intersection of the two closed scalar intervals is the complete
noncoplanar result. This covers disjoint slices, endpoint contact, partial
overlap, containment, equality, reversed windings, collinear boundary
vertices, and arbitrary convex polygon degree without a retry or pass limit.

Any certified nonzero component of `normal(P) × normal(Q)` is a one-to-one
affine coordinate on `L`. Crossing coordinates are retained as the exact ratio

`(s0 * endpoint1 - s1 * endpoint0) / (s0 - s1)`,

where `s0` and `s1` are the already-demanded support values. Ratio/ratio and
ratio/affine comparisons use exact rational product-sum ordering first and the
general `Real`/Hyperlimit decision path otherwise. Finite binary64 enclosures
may prove strict separation or exact representable equality only; overlap
declines to the exact comparison. The endpoint's interpolation numerator and
denominator remain retained so the surviving `Point3` is constructed once.

Parallel support planes still enter the existing exact coplanar/disjoint path.
Empty polygons remain disjoint. Malformed/capacity paths retain their typed
errors. No projective fallback is necessary for the nonparallel slice because
every finite polygon vertex and every finite crossing has an affine closed-line
coordinate; the deleted projective containment machinery was duplicate work,
not a completeness route.

## Hyperlimit policy contract

Support parallelism is a composite three-component predicate. The new
scheduler first gives all three components their complete strict proof attempt.
A later certified nonzero component therefore returns `Certified` even if an
earlier component is symbolically unresolved. Only if no strict component
proves nonparallelism may `APPROXIMATE_512` ask Hyperlimit to terminate the
remaining unknown components. `STRICT` returns `PredicateUndecided` at that
same point.

Endpoint ordering follows the same contract. Exact rationals remain
`Certified` under both policies. Symbolic equality is undecided under `STRICT`
and records `Approximate512Consumed` only through Hyperlimit's 512-bit
terminal under `APPROXIMATE_512`. Aggregate certainty is unchanged.

## Carrier and construction schedule

The operation-local carrier stores either an affine point or one borrowed
source edge plus its exact coordinate ratio and interpolation numerator. A
closed span owns at most two such values. Equal endpoints merge canonical
construction identities before materialization. A layout regression fixes the
point carrier at at most 256 bytes and the two-endpoint span at at most 528
bytes.

Several measured alternatives were rejected and fully removed:

- eagerly materializing every affine candidate raised the full row to about
  21.4 billion instructions;
- the first by-value deferred span caused large stack moves;
- splitting coordinate and materialization data across duplicate tagged
  carriers increased the hot carrier size;
- recomputing the interpolation numerator for the final endpoint added about
  11 million full-row instructions.

The retained version extends spans in place, borrows source edge points, keeps
one shared denominator, and stores the interpolation numerator once. These are
general dataflow choices, not topology or fixture special cases.

## Permanent corpus additions

Five exact static cases add disjoint support-line intervals, crossing/crossing
point contact, contained intervals, a Z-directed support line, and convex-quad
containment. Each runs under both policies, both operand orders, and all four
winding combinations: 80 new static executions.

A 256-case property test compares axis-aligned noncoplanar rectangle slices to
an independent exact closed-interval oracle under both policies, both operand
orders, and all winding combinations: 4,096 executions. Unit tests additionally
cover every interval dimension/order, affine/ratio combinations, 1,024 random
exact-rational ratio comparisons, canonical equal-endpoint construction,
deferred materialization, carrier size, symbolic endpoint equality, symbolic
parallelism, and the strict-first later-component rule.

## Full-resolution performance

The current row is five fresh processes pinned to CPU 11. It includes fixture
preparation, exact import, validation, Boolean intersection, output checks, and
destruction. The established parent is the accepted `10dc6533` checkpoint on
the same host and protocol; deterministic instructions and branches are the
primary comparison.

| Metric | `10dc6533` | `cfc5ae62` | Movement |
| --- | ---: | ---: | ---: |
| Median wall | 1.88 s | 1.71 s | -9.04% |
| Cycles | 7,239,160,608 | 6,600,430,217 | -8.82% |
| Instructions | 18,514,197,608 | 16,777,620,124 | -9.38% |
| Branches | 3,286,637,679 | 3,014,526,912 | -8.28% |
| Cache misses | 19,792,580 | 18,491,840 | -6.57% |

Wall samples were 1.71, 1.72, 1.72, 1.71, and 1.71 seconds. The result was
empty and `Certified` in every run. Historical EMBER remains 3,312.66 seconds
and 329,352 KiB, making the current row about 1,937.2x faster and 42.28% lower
in fresh-process RSS. Pinned CGAL EPECK remains 0.09 seconds and 15,516 KiB;
current strict/approximate fresh-process RSS is 190,092/190,356 KiB.

## Independent controls and ordinary exact boxes

All controls are ordinary executions of the same direct release probe.

| Fixture | Triangles | Parent instructions | Current instructions | Movement | Branch movement |
| --- | ---: | ---: | ---: | ---: | ---: |
| 2,049-bit wide rational | 6,144 | 3,892,427,738 | 3,824,038,509 | -1.76% | -2.26% |
| Clipped voxel torus 33 | 6,412 | 1,157,159,946 | 1,134,371,933 | -1.97% | -1.98% |
| Clipped voxel torus 65 | 25,100 | 4,957,003,380 | 4,915,377,049 | -0.84% | -0.83% |
| Dense coplanar 16 | 6,144 | 3,051,772,507 | 3,016,824,494 | -1.15% | -1.12% |

Criterion's complete shared arrangement for ordinary overlapping boxes is
574.91 microseconds under `STRICT` and 583.89 microseconds under
`APPROXIMATE_512`, versus 632.628/629.603 microseconds at the accepted parent.
The pinned CGAL copy-outside/copy-inside rows are 119.5965/128.9760
microseconds, leaving explicit 4.81x/4.53x losses.

## Direct large-fixture heap matrix

The global-allocator probe excludes fixture construction and reports requested
payload at the Boolean boundary. Every row was run under both policies. For
each selector, output, certainty, peak, allocation/deallocation/reallocation
counts, added bytes, and removed bytes are exactly equal policy-for-policy; all
results are `Certified`.

| Selector | Input triangles | Output vertices/triangles | Incremental peak |
| --- | ---: | ---: | ---: |
| `boxes-3072` | 6,144 | 2,410 / 4,816 | 16,004,010 B |
| `boxes-3072-general` | 6,144 | 2,410 / 4,816 | 16,063,730 B |
| `dense-coplanar-16` | 6,144 | 3,074 / 6,144 | 18,719,560 B |
| `dense-coplanar-32` | 24,576 | 12,290 / 24,576 | 74,724,848 B |
| `wide-rational-64` | 6,144 | 2,410 / 4,816 | 21,716,146 B |
| `wide-rational-512` | 6,144 | 2,410 / 4,816 | 22,596,194 B |
| `wide-rational-2048` | 6,144 | 2,410 / 4,816 | 31,234,658 B |
| `voxel-torus-33` | 6,412 | 1,730 / 3,456 | 16,592,142 B |
| `voxel-torus-65` | 25,100 | 6,532 / 13,060 | 66,152,426 B |
| `yeahright` | 852 | 317 / 630 | 5,053,588 B |
| `yeahright-4` | 3,372 | 1,054 / 2,104 | 18,851,122 B |
| `yeahright-8` | 13,452 | 3,820 / 7,636 | 71,531,870 B |
| `yeahright-full-rotated` | 23,788 | 0 / 0 | 158,258,204 B |

The final full row performs 16,928,390 allocations, 2,385,161 reallocations,
and adds 913,357,128 bytes over the whole operation; all are identical to the
parent and between policies. The final 2,049-bit row likewise retains its
31,234,658-byte peak and policy-identical counters.

## Linked size

Native values are `.text`; WASM values are `wasm-opt -Oz` bytes.

| Profile / consumer | Parent native | Current native | Parent WASM | Current WASM |
| --- | ---: | ---: | ---: | ---: |
| release / general | 2,029,758 | 1,968,566 (-3.01%) | 1,447,589 | 1,391,337 (-3.89%) |
| release / immediate | 2,032,902 | 1,971,710 (-3.01%) | 1,449,447 | 1,393,195 (-3.88%) |
| size / general | 1,088,223 | 1,072,063 (-1.49%) | 684,175 | 668,360 (-2.31%) |
| size / immediate | 1,089,171 | 1,073,019 (-1.48%) | 684,362 | 668,771 (-2.28%) |

The shrink comes from deleting the candidate-containment/projective fallback
machinery; no dependency or compatibility layer is added.

## Dispatch and call-graph audit

The YeahRight trace records 3,491 forward and 3,491 reverse support-line
slices, 2,624 certified support-plane separations, zero unknown-fact events,
and zero fallback/abort events. Ordinary cube, nested, octahedral, subdivided,
and clipping workloads also exercise the same two slice events.

After removing the clean temporary parent worktree from discovery, the focused
five-crate graph contains 14,793 nodes and 24,634 edges. Hypermesh contributes
2,914 nodes and 4,669 edges. `intersect_polygons_with_vertices_constructed` is
the sole production caller of the new nonparallel routine; that routine is the
sole caller of both slice construction and interval intersection. The graph
contains none of the deleted edge-candidate collection, affine containment,
projective edge/polygon containment, four-plane determinant, or old parallel
support functions. Hypercurve and HyperSolve are excluded.

## Validation

- 179 all-feature tests pass; six documented external/manual tests remain
  ignored. The default suite also passes.
- No-default-feature checking, all-target/all-feature Clippy with warnings
  denied, rustdoc with warnings denied, formatting, and diff checks pass.
- Every fuzz binary checks and every benchmark target builds.
- Native and WASM size profiles build.
- The locked release Trunk demo builds.
- The full exact oracle passes repeatedly; all 13 heap selectors pass under
  both policies with exact policy equality.

## Open work

This checkpoint does not close Phase 17 or 18. Full and ordinary-box CGAL
runtime/RSS losses remain. The corpus still needs external real-world and
deeper-symbolic families, further sparse/multi-shell/pathological cases, and
stage-specific lifetime fixtures. Current CGAL confidence runs and per-stage
heap attribution remain incomplete. The final removal/requirements audit and
the deferred concurrent-dependent caller validation also remain open.

## Reproduction

```sh
cargo test --locked --all-features
cargo test --locked
cargo check --locked --no-default-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --locked --all-features --no-deps
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo bench --locked --no-run

YEAHRIGHT_BENCH=1 taskset -c 11 perf stat -r 5 -x, \
  -e cycles,instructions,branches,cache-misses \
  target/release/deps/competitive-92c64513605410c9 \
  --ignored --exact full_resolution_yeahright_rotated_intersection_certifies_empty

target/release/examples/large_mesh_kernel_heap_probe \
  <fixture-selector> <strict|approximate-512>
benchmarks/size-harness/measure.sh
YEAHRIGHT_BENCH=1 cargo bench --locked --bench dispatch_trace \
  --features dispatch-trace

tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir /tmp/hypermesh-exact-support-line-interval-callgraph-clean-2026-08-04 \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format dot,json,mermaid,graphml \
  --per-library
```
