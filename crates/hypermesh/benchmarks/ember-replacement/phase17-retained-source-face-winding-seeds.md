# Phase 17: retained source-face winding seeds

Date: 2026-08-04

Status: accepted

Implementation: `d103dcf8d484995343de580c38e6ea5eb7b532a9`

Paired parent: `611defa86758888c181b5642ecd784c6d0b4792e`

## Result

Absolute cell winding is now seeded against the original exact closed-PWN
source triangles, not against every triangle created by face corefinement.
Corefinement remains responsible for the complete surface arrangement and
radial topology. It does not change the boundary whose signed crossings define
the winding number: an exact ray crosses a convex source triangle at most once,
and that crossing applies the triangle's transition exactly once.

Both source and arranged triangles use one exact ray/triangle predicate over
borrowed `Point3` references. The previous implementation cloned three
`Point3` values, and therefore nine potentially fact-bearing `Real` values,
for every tested corefined triangle. The new source path reads the exact
retained input vertex cycle directly. This is a general retained-representation
optimization; it does not inspect fixture identity, triangle or component
count, Boolean operation, output topology, expected answers, or a competitor.

The complete path remains explicit:

- the unchanged exact BVH conservatively discovers candidate source faces;
- a missing, incomplete, or non-triangular retained source cycle is a typed
  arrangement failure rather than an unchecked indexing path;
- a ray hit at its origin is accepted only when that source face is in the
  arranged seed facet's checked contribution row;
- a hit on a true source edge or a ray parallel to a source support remains a
  degenerate direction and enters the existing finite exact retry schedule;
- artificial internal corefinement edges no longer cause such retries;
- each failed direction retains its isolated decision context, and only the
  successful direction's certainty is absorbed by the operation context.

Consequently `STRICT` remains exact-only. `APPROXIMATE_512` still reaches only
Hyperlimit's terminal decision and still updates the aggregate certainty if it
is consumed. The new exact-rational regression runs both policies and proves
that a ray through an artificial subdivision edge is an `Ahead` crossing of
the source face even though each subdivision reports `Degenerate`.

## Paired protocol

The committed parent was exported into an isolated source tree, built with the
same current Hyperreal, Hyperlattice, Hyperlimit, and Hypertri checkouts, the
same Cargo target cache, Rust 1.97.0, lockfile, release profile, and CPU 11.
Three `perf stat` processes report user instructions and branches. Fixture
construction and one source-soup/PWN prime are inside the process counters and
are amortized across the stated Boolean repetitions.

## Retired work

| Fixture / workload | Parent instructions | Current instructions | Movement | Parent branches | Current branches | Movement |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| sparse 8, `all` x100 | 1,087,048,451 | 1,004,682,832 | -7.577% | 197,773,290 | 183,061,918 | -7.439% |
| sparse 64, `all` x20 | 1,902,817,815 | 1,705,807,591 | -10.354% | 347,295,502 | 313,147,112 | -9.833% |
| sparse 512, `all` x5 | 4,447,657,362 | 3,750,586,752 | -15.673% | 815,062,679 | 689,826,654 | -15.365% |
| ordinary overlapping boxes, `all` x1000 | 5,205,693,181 | 5,027,801,344 | -3.417% | 915,433,758 | 882,399,897 | -3.609% |
| torus 33, `all` x3 | 2,908,883,293 | 2,908,661,626 | -0.008% | 498,761,016 | 499,389,796 | +0.126% |
| torus 65, `all` x1 | 4,728,932,526 | 4,728,428,061 | -0.011% | 811,515,533 | 812,182,990 | +0.082% |
| subdivided boxes, union x5 | 4,444,820,950 | 4,445,483,994 | +0.015% | 770,106,400 | 770,543,764 | +0.057% |
| dense coplanar 32, `all` x1 | 11,864,414,114 | 11,866,015,885 | +0.014% | 2,027,359,638 | 2,023,934,826 | -0.169% |
| wide rational 2,049-bit, union x5 | 14,047,710,894 | 14,047,202,624 | -0.004% | 2,563,506,103 | 2,563,300,061 | -0.008% |
| full rotated YeahRight | 13,661,036,892 | 13,661,218,319 | +0.001% | 2,389,997,025 | 2,390,076,100 | +0.003% |

The large win follows topology, not a fixture selector: every disconnected
surface-cell component needs an absolute winding seed, while connected torus,
wide-rational, subdivided, and YeahRight rows need very few seeds. The latter
controls move at most 0.169% in either counter. Ordinary overlapping boxes also
benefit materially.

Sparse instructions per arrangement are now 10.047M, 85.290M, and 750.117M.
The two 8x shell steps cost 8.489x and 8.795x, improved from 8.758x and 9.350x
at the parent checkpoint. The remaining superlinear component cost is still an
open Phase-17 target.

One-process Callgrind falls from 1,010,417,613 to 861,194,927 instructions
(-14.768%). Inclusive `classify_surface_cells` work falls from 222,586,741 to
83,020,645 (-62.702%), and `build_surface_arrangement` falls from 780,950,557
to 631,603,902 (-19.124%). Corefinement itself remains approximately 418.8M
instructions and is now the clearer next arrangement bottleneck. The current
profile is `target/phase17-source-face-winding-sparse-512.callgrind`.

## Current CGAL EPECK boundary

The pinned CGAL 6.0.3 EPECK adapter was rerun for 21 repetitions in both copy
modes on all three exact OFF siblings. Every one of the 126 records and all
four outputs in every record are valid, closed, and structurally valid.
Hypermesh medians below are the per-operation medians from the three pinned
`perf` processes used for the counter table; the repetitions are 100, 20, and
5 respectively.

| Shells | Hypermesh STRICT | Hypermesh APPROXIMATE_512 | CGAL outside-copy | CGAL inside-copy | STRICT / CGAL outside |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 8 | 0.880 ms | 0.868 ms | 0.288 ms | 0.279 ms | 3.06x |
| 64 | 6.917 ms | 6.854 ms | 2.027 ms | 2.064 ms | 3.41x |
| 512 | 60.584 ms | 57.567 ms | 18.228 ms | 18.231 ms | 3.32x |

The 8-shell clock is sensitive to host frequency and process noise. Retired
work is the primary historical acceptance signal. The competitive conclusion
is nevertheless materially better than the parent's approximately 4.1-4.3x
`STRICT` loss on this family, while the CGAL parity gate remains open.

## Heap

All fourteen designated large selectors were executed under both policies on
the final implementation. Every policy pair returned the same result and a
byte-identical allocator row with `Certified` aggregate certainty. Relative to
the paired parent, total and incremental peak heap are byte-identical for all
fourteen selectors.

The removed transient `Real` clones reduce lifetime work without adding a
carrier or cache. Sparse-512 allocation calls fall from 513,202 to 470,296
(-8.36%), cumulative added payload falls from 53,235,212 to 50,905,532 bytes
(-4.38%), and post-Boolean retained input-fact growth falls by 7,176 bytes.
Dense-16 and dense-32 lose 126 and 127 allocation calls respectively. The
remaining eleven selectors have byte-identical allocation totals as well as
byte-identical peaks.

Representative unchanged peaks are 15,540,295 bytes total / 14,578,138 bytes
incremental for sparse-512, 78,750,901 / 74,724,848 for dense-32,
70,860,620 / 66,042,050 for torus-65, and 165,381,598 / 158,258,204 for full
rotated YeahRight.

## Source and binary size

The implementation commit adds 105 and removes 29 Rust lines, including the
policy-paired regression, and adds no dependency, feature, public API, carrier,
or compatibility surface. Canonical linked size moves slightly downward in
release builds and is effectively flat in size builds:

| Profile / consumer | Parent native `.text` | Current native `.text` | Parent `wasm-opt -Oz` | Current `wasm-opt -Oz` |
| --- | ---: | ---: | ---: | ---: |
| release / general | 1,987,146 B | 1,984,718 B (-0.122%) | 1,410,174 B | 1,408,990 B (-0.084%) |
| release / immediate | 1,990,290 B | 1,987,862 B (-0.122%) | 1,412,032 B | 1,410,857 B (-0.083%) |
| size / general | 1,080,175 B | 1,080,047 B (-0.012%) | 674,376 B | 674,493 B (+0.017%) |
| size / immediate | 1,081,131 B | 1,081,003 B (-0.012%) | 674,786 B | 674,902 B (+0.017%) |

## Call graph and path audit

The five-crate production graph contains 14,905 nodes and 24,866 edges. The
examples/tests graph contains 17,187 nodes and 28,093 edges. Hypercurve and
HyperSolve are excluded. The production route is singular:

`classify_surface_cells -> seed_surface_cell_winding ->
try_seed_surface_cell_winding -> ray_source_polygon_relation ->
ray_triangle_relation`.

The arranged seed facet reaches the same final predicate through
`ray_facet_relation`. No removed EMBER, local-BSP, or segment-trace namespace
is present in production source or the graph. Artifacts are under
`target/callgraph-hypermesh-source-face-winding{,-production}`.

## Validation

The final default suite executes 185 tests and the all-feature suite executes
186; all pass, with the six documented external/manual YeahRight tests ignored
in ordinary runs. Warning-denied all-target/all-feature Clippy, no-default
checking, warning-denied rustdoc, all fuzz binaries, all benchmarks, formatting,
diff checks, the canonical size harness, both call graphs, the exact CGAL
corpus, and every two-policy large heap selector pass.

## Reproduction

```sh
cargo test --locked --all-features
cargo clippy --locked --all-targets --all-features -- -D warnings

taskset -c 11 perf stat -r 3 -x, \
  -e instructions:u,branches:u \
  target/release/examples/competitive_arrangement_probe \
  sparse_multishell_tetrahedra_512 all strict 5

target/release/examples/large_mesh_kernel_heap_probe \
  sparse-shells-512 approximate-512

benchmarks/size-harness/measure.sh default

tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir hypermesh/target/callgraph-hypermesh-source-face-winding \
  --format json \
  --crate-name hypermesh,hyperreal,hyperlimit,hypertri,hyperlattice \
  --include-examples --include-tests --per-library
```
