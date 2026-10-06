# Phase 17: lossless retained source planes

Date: 2026-08-04

Status: accepted

Implementation: `19b76441b000a2c5ecdba3ff2568ebcf58f3a25e`

Paired parent: `48a9e7dcac0381d19107a7825217d0f88fbb16c3`

## Result

Native immutable meshes now retain one certified, all-or-none source-plane
fact after canonical polygon construction proves that every support and edge
coefficient has a lossless primitive encoding. A complete mesh uses 64-byte
binary32 triangle rows when every coefficient survives a second exact
round-trip and 128-byte binary64 rows otherwise. General rational, symbolic,
out-of-range, borrowed, or already-approximate construction declines retention
and continues through the unchanged exact `Real` construction path.

The retained rows are representations, not predicate results. A cache hit:

- imports every coefficient back into an exact rational `Real`;
- recomputes policy-owned bounds from the immutable source points;
- restamps current mesh, global polygon, source-edge identity, and winding
  transition metadata;
- continues through PWN validation, intersection, arrangement, winding, and
  output certification normally.

Cache publication requires aggregate `Certified` certainty both before and
after source construction. A certified cache may be consumed under either
policy, while an already consumed `APPROXIMATE_512` context keeps its aggregate
certainty. `STRICT` still has no approximate terminal. No component count,
fixture identity, operation, expected topology, or competitor selects this
path.

The implementation deliberately retains only plane coefficients. A trial that
retained complete `ConvexPolygon` values reduced sparse retired work by roughly
10.7-12.6%, but raised 512-shell input payload from 699,312 B to 10,557,240 B
and total peak from 15.72 MB to approximately 18.14 MB. It was rejected. The first
binary64-only compact trial retained 524,336 additional bytes on that fixture;
the accepted binary32 tier halves that retained payload while improving cache
locality.

## Paired protocol

The parent commit was exported to an isolated `/tmp` tree and compiled against
the same current Hyperreal, Hyperlattice, Hyperlimit, and Hypertri checkouts as
the candidate. Both used Rust 1.97.0, locked dependencies, release settings,
CPU 11, and three `perf stat` repetitions. Primary measurements are retired
instructions and branches; wall time is secondary. Fixture preparation and one
PWN-prime pass are inside the whole-process counters and amortized over the
reported Boolean repetitions.

## Retired work

| Fixture / workload | Parent instructions | Current instructions | Movement | Parent branches | Current branches | Movement |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| sparse 8, `all` x100 | 1,168,218,243 | 1,086,859,715 | -6.964% | 211,796,723 | 197,831,069 | -6.594% |
| sparse 64, `all` x20 | 2,037,655,064 | 1,903,729,231 | -6.573% | 369,868,843 | 348,729,493 | -5.715% |
| sparse 512, `all` x5 | 4,734,236,442 | 4,449,797,082 | -6.008% | 859,970,070 | 815,189,317 | -5.207% |
| torus 33, `all` x3 | 3,126,387,839 | 2,907,846,314 | -6.990% | 534,398,036 | 499,160,407 | -6.594% |
| ordinary boxes, `all` x1000 | 5,474,831,546 | 5,207,832,630 | -4.877% | 960,247,114 | 916,089,869 | -4.599% |
| subdivided boxes, union x5 | 4,821,868,010 | 4,519,596,141 | -6.269% | 831,078,128 | 781,580,073 | -5.956% |
| torus 65, `all` x1 | 4,968,646,774 | 4,794,313,809 | -3.509% | 850,002,742 | 824,310,444 | -3.023% |
| dense coplanar 32, `all` x1 | 12,130,180,797 | 11,936,381,673 | -1.598% | 2,067,245,564 | 2,035,775,256 | -1.522% |
| full rotated YeahRight | 13,723,276,881 | 13,661,330,601 | -0.451% | 2,398,609,294 | 2,391,820,353 | -0.283% |
| wide rational 2,049-bit, union x5 | 14,119,169,725 | 14,047,329,052 | -0.509% | 2,573,353,291 | 2,563,295,551 | -0.391% |

The final two controls decline retained plane rows and show no counter
regression; there is no memory-intensive cache on either path.

Sparse instructions per arrangement are 10.869M, 95.186M, and 889.959M.
The two 8x shell steps still grow retired work by 8.758x and 9.350x. Absolute
work is lower at every scale, but the superlinear component slope remains open;
this checkpoint does not claim to solve component-local scheduling.

Paired median wall times from the same three `perf` processes improved 5.83%
on torus 65, 4.65% on subdivided boxes, and 3.50% on dense coplanar 32. They
support the counter direction but are not the primary acceptance metric.

## Current CGAL EPECK boundary

The pinned CGAL 6.0.3 EPECK adapter and the exact OFF corpus were rerun on the
same host. All 126 sparse output records were valid, closed, and structurally
valid. Current medians are:

| Shells | Hypermesh STRICT | Hypermesh APPROXIMATE_512 | CGAL outside-copy | CGAL inside-copy | STRICT / CGAL outside |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 8 | 1.159 ms | 1.375 ms | 0.268 ms | 0.289 ms | 4.32x |
| 64 | 8.445 ms | 8.531 ms | 2.046 ms | 2.077 ms | 4.13x |
| 512 | 75.974 ms | 77.296 ms | 17.631 ms | 17.782 ms | 4.31x |

The small case has visible host/process noise. Deterministic counters show the
accepted historical improvement, while the competitive conclusion remains
unchanged: Hypermesh is still about four times slower than CGAL EPECK on this
family and Phase 17 remains open.

## Heap and RSS

The allocator probe wraps `System` only in the measurement executable and
counts successful requested payload. Every row is a direct parent/candidate
pair. `Total peak` includes retained input; `kernel peak` is incremental above
the primed input.

| Fixture | Input retained parent / current | Total peak parent / current | Kernel peak parent / current | Allocation calls parent / current |
| --- | ---: | ---: | ---: | ---: |
| sparse shells 512 | 699,949 / 962,157 B | 15,717,359 / 15,540,295 B | 15,017,410 / 14,578,138 B | 516,658 / 513,202 |
| torus 33 | 657,426 / 1,067,858 B | 17,249,256 / 17,657,096 B | 16,591,830 / 16,589,238 B | 184,195 / 179,827 |
| torus 65 | 3,211,338 / 4,818,570 B | 69,363,596 / 70,860,620 B | 66,152,258 / 66,042,050 B | 701,588 / 692,975 |
| subdivided boxes | 656,246 / 1,049,526 B | 16,660,256 / 17,053,704 B | 16,004,010 / 16,004,178 B | 165,970 / 159,829 |
| dense coplanar 32 | 2,453,125 / 4,026,053 B | 77,177,973 / 78,750,901 B | 74,724,848 / 74,724,848 B | 2,068,267 / 2,049,835 |
| wide rational 2,049-bit | 663,686 / 663,702 B | 31,898,344 / 31,898,360 B | 31,234,658 / 31,234,658 B | 1,947,027 / 1,947,027 |
| full rotated YeahRight | 7,123,378 / 7,123,394 B | 165,381,582 / 165,381,598 B | 158,258,204 / 158,258,204 B | 15,558,650 / 15,558,650 |

Sparse total peak falls 1.127% because reduced transient construction is larger
than the retained binary32 rows. The other admitted large fixtures raise total
peak 2.04-2.36%, while their incremental kernel peak is flat or lower and
allocation calls fall 0.89-3.70%. Wide rational and full YeahRight decline the
cache and pay only the 16-byte two-mesh fact-header increase.

Fresh-process maximum RSS moved from 20,852 to 21,352 KiB on sparse 512,
73,592 to 75,944 KiB on torus 65, 83,344 to 85,308 KiB on dense 32, and 20,628
to 21,212 KiB on subdivided boxes. This 2.36-3.20% resident increase is accepted
because performance has priority, the exact heap boundary is explicit, and the
larger complete-polygon carrier was rejected.

All fourteen designated large heap selectors were executed on the final
implementation under `STRICT` and `APPROXIMATE_512`. Every policy pair produced
the same certified result and a byte-identical allocator row.

## Source and binary size

The implementation commit adds 411 and removes 58 Rust lines, including four
path-specific tests. The permanent carrier adds one fact-header slot and no
dependency or feature. Canonical size movement is:

| Profile / consumer | Parent native `.text` | Current native `.text` | Parent `wasm-opt -Oz` | Current `wasm-opt -Oz` |
| --- | ---: | ---: | ---: | ---: |
| release / general | 1,970,782 B | 1,987,146 B (+0.830%) | 1,394,510 B | 1,410,174 B (+1.123%) |
| release / immediate | 1,973,926 B | 1,990,290 B (+0.829%) | 1,396,354 B | 1,412,032 B (+1.123%) |
| size / general | 1,072,367 B | 1,080,175 B (+0.728%) | 668,271 B | 674,376 B (+0.914%) |
| size / immediate | 1,073,331 B | 1,081,131 B (+0.727%) | 668,678 B | 674,786 B (+0.913%) |

The sub-1.2% binary growth is accepted against the general 1.6-7.0% retired-work
gain. The mechanism remains private and replaces repeated construction rather
than adding a second Boolean engine or compatibility surface.

## Call graph and path audit

The workspace call-graph utility scanned Hypermesh, Hyperreal, Hyperlimit,
Hypertri, and Hyperlattice with examples and tests: 17,181 nodes and 28,077
edges. The retained path is confined to
`build_polygon_soup_internal -> CompactSourcePolygons::from_polygons` on first
certified construction and
`build_polygon_soup_internal -> append_compact_source_polygon ->
ConvexPolygon::from_triangle_planes` on reuse. The same soup boundary remains
reachable from public Boolean, polygon-soup, and convex-certification callers.
No competitor, fixture, Boolean operation, or result materializer reaches a
separate path. Runtime profiling still attributes the optimization to the
existing source-soup boundary, not an alternate arrangement engine.

The generated graph is
`target/callgraph-hypermesh-retained-source-planes/callgraph.json`; it is a
local navigation artifact, not committed proof of dynamic dispatch.

## Validation

Validation includes the default and all-feature suites, warning-denied
all-target/all-feature Clippy, no-default checking, warning-denied rustdoc,
every fuzz binary check, benchmark compilation, the canonical native/WASM size
harness, the pinned CGAL exact corpus, all fourteen two-policy heap probes,
formatting, and diff checks. The default suite executes 184 tests; all features
execute 185, with the six documented external/manual YeahRight tests ignored in
ordinary runs. Focused tests prove binary32 reuse, binary64 reuse, general
rational fallback, borrowed non-retention, no approximate fact publication,
aggregate certainty preservation, and complete operand/provenance restamping.

## Reproduction

```sh
cargo test --locked --all-features
cargo clippy --locked --all-targets --all-features -- -D warnings

taskset -c 11 perf stat -r 3 -x, \
  -e instructions:u,branches:u \
  target/release/examples/competitive_arrangement_probe \
  sparse_multishell_tetrahedra_512 all strict 5

target/release/examples/large_mesh_kernel_heap_probe \
  sparse-shells-512 strict

benchmarks/size-harness/measure.sh default

tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir target/callgraph-hypermesh-retained-source-planes \
  --format json \
  --crate-name hypermesh,hyperreal,hyperlimit,hypertri,hyperlattice \
  --include-examples --include-tests --per-library
```
