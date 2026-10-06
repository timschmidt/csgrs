# Phase 17 retained coplanar pair facts checkpoint

Captured 2026-08-03 on the established Ryzen 7 5800X3D / CPU 11
single-thread protocol at Hypermesh
`50b62e5da55f1b87cead99ef060cd771e3c9908b`. The parent is the canonical
self-pair/local-topology checkpoint at `9ebefb108b8eea6863b9ef04b0916d72103825f3`.
This is a retained Phase 17 optimization checkpoint. Corpus completion, linked
size recovery, CGAL EPECK parity, and the Phase 18 audit remain open.

## Result

One operation-local pair scratch now follows the canonical BVH candidate
stream. It retains capacity, exact point/edge classifications, and Hyperreal's
certified floating query facts while every topology decision remains exact or
policy-resolved through the existing cascade.

On the permanent 6,412/25,100-triangle torus siblings, deterministic
instructions fall 11.25%/11.20%, branches fall 11.69%/11.74%, allocation calls
fall 28.27%/28.99%, and requested allocation bytes fall 61.52%/59.52% from the
parent. Current warm `STRICT` time is 14.44%/12.43% lower. Exact topology,
volume, and `Certified` certainty are unchanged under both policies.

The reusable point query alone removes another 2.14% of instructions on both
large rows after the classification matrix was already present. This is the
intended Hyperreal advantage: a certified `RationalLinearForm4Query` retains
the expensive arbitrary-rational-to-binary64 normalization for one point,
then several edge planes reuse it. An uncertain filter still enters the same
unbounded exact-rational fallback.

No fixture name, coordinate magnitude, triangle count, operation, expected
output, benchmark state, or competitor is inspected. There is one Boolean
engine and no compatibility layer, alternate benchmark path, epsilon, or
primitive-float topology decision.

## Pairwise scratch and exact classification ownership

`PairwiseIntersectionScratch` is owned by the whole self-BVH traversal. Its
three vectors are cleared before each polygon pair but retain capacity:

- constructed point/contact candidates;
- the two directional edge-by-vertex classification matrices; and
- one optional compact rational query per vertex.

The direct pairwise API creates the same scratch locally, so there is no second
algorithm. Every capacity sum and product is checked before allocation.
Allocation failure and malformed indexing remain typed errors.

For a coplanar convex pair, the original separating-axis walk still visits
left edges/right vertices, then right edges/left vertices, in the original
short-circuit order. Only demanded classifications are stored. If positive
area is absent, lower-dimensional point/segment recovery consumes those same
facts instead of repeating the predicates.

A fully known non-positive vertex column proves containment. A cached positive
proves exclusion immediately only while aggregate certainty is still
`Certified`. After any `APPROXIMATE_512` terminal has been consumed, retained
structural vertex identity keeps its stronger original priority before that
cached exclusion is used. A permanent regression constructs this otherwise
contradictory cache state directly and proves the retained identity wins.

Failed `STRICT` classifications are not stored. Query facts and matrices live
only inside one operation and are cleared between polygon pairs, so neither
policy nor geometry can contaminate another operation.

The small candidate list is deduplicated stably in place, preserving vector
capacity and choosing the minimum canonical construction recipe. The identical
exact-rational point-contradiction proof is now centralized for the pair graph
and arrangement point arena; one known unequal rational coordinate is a sound
early inequality certificate even when other coordinate representations are
not retained rationals.

## Policy and exactness

Both policies run the same schedules:

- `STRICT` consumes only exact or certified predicates and returns typed
  indeterminacy when the cascade is exhausted.
- `APPROXIMATE_512` may consume only Hyperlimit's centralized 512-bit terminal,
  and the operation aggregates that fact.
- The rational point query is only a certified filter input. It cannot decide
  an uncertain sign and never replaces exact fallback arithmetic.
- Reusing a demanded classification consumes no new decision. The original
  predicate order is unchanged for classifications not already known.
- All measured rational rows are `Certified`; exact outputs are identical
  between policies.
- Large/XL direct heap counters are byte-for-byte identical between policies.

The full all-feature policy suite, symbolic terminal tests, pairwise closed-set
corpus, contact fuzz seeds, and the new approximate-cache/retained-identity
regression pass.

## Current exact torus comparison

Small and large Hypermesh rows are aggregate elapsed divided by 101/31 warmed
production calls after exact import and policy-qualified PWN priming. The XL
row is the median of three independent 11-call aggregate means. CGAL values
remain pinned CGAL 6.0.3 EPECK 21-call medians over identical reduced-rational
OFF inputs, with required input copies outside the timed interval.

| Input triangles | Policy | Parent Hypermesh | Current Hypermesh | Change | CGAL median | Current / CGAL |
| ---: | --- | ---: | ---: | ---: | ---: | ---: |
| 460 | `STRICT` | 8.800 ms | 8.033 ms | -8.72% | 0.493 ms | 16.29x |
| 460 | `APPROXIMATE_512` | 8.690 ms | 7.956 ms | -8.45% | 0.493 ms | 16.14x |
| 6,412 | `STRICT` | 107.600 ms | 92.058 ms | -14.44% | 3.504 ms | 26.28x |
| 6,412 | `APPROXIMATE_512` | 108.688 ms | 92.438 ms | -14.95% | 3.504 ms | 26.38x |
| 25,100 | `STRICT` | 461.710 ms | 404.326 ms | -12.43% | 11.839 ms | 34.15x |
| 25,100 | `APPROXIMATE_512` | 465.820 ms | 401.745 ms | -13.76% | 11.839 ms | 33.93x |

Every row improves, but Hypermesh is not yet competitive with CGAL on any
member of this family.

## Fresh-process counters and scaling

Eleven pinned repetitions include deterministic fixture construction, exact
import, policy-qualified PWN priming, and one `STRICT` Boolean call.

| Input triangles | Task clock | Instructions | Change | Branches | Change | Branch misses | Cache misses |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 6,412 | 114.38 ms | 1,307,616,836 | -11.25% | 232,438,083 | -11.69% | 1,647,082 | 1,199,260 |
| 25,100 | 494.98 ms | 5,602,180,760 | -11.20% | 994,234,204 | -11.74% | 5,979,365 | 5,895,897 |

For 3.9145x input triangles, instructions and branches grow 4.2843x and
4.2774x. Absolute work falls substantially, but the instruction slope is
effectively unchanged from the parent's 4.2821x and remains superlinear.
Cache-miss growth is noisy at 4.9163x. This checkpoint does not close the
scaling gate.

After reusable pair classifications but before retained point queries, the
large/XL instruction rows were 1,336,266,235 and 5,725,131,181. Retaining the
query therefore removes a further 2.1440% and 2.1476% respectively without an
additional topology path.

## Direct requested-payload heap

Each fixture ran in a fresh serialized process under both policies. Paired
policy rows are identical, so each is shown once.

| Input triangles | Incremental peak | Parent peak | Alloc calls | Change | Realloc calls | Added bytes | Change |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 6,412 | 16,387,766 B | 16,387,156 B | 183,110 | -28.27% | 927 | 34,661,744 B | -61.52% |
| 25,100 | 65,353,442 B | 65,352,128 B | 699,382 | -28.99% | 1,778 | 150,367,419 B | -59.52% |

The reusable query adds one bounded scratch allocation and 240 bytes beyond
the classification-only prototype. Relative to the parent, peak movement is
only +610/+1,314 bytes (+0.0037%/+0.0020%). Prepared input, output-live payload,
and input-fact growth remain 656,792/3,210,704 bytes,
375,368/1,623,160 bytes, and 42,840/699,616 bytes. The governing live set and
input-fact lifetime cliff are unchanged.

Fresh-process maximum RSS is 21,500/73,852 KiB versus retained CGAL
9,556/20,020 KiB, or 2.25x/3.69x. RSS remains open.

## Historical and small-case controls

The full 11,894-by-11,894 YeahRight intersection remains exact-empty and
`Certified` under `APPROXIMATE_512`:

| Row | Task clock | Instructions | Maximum RSS |
| --- | ---: | ---: | ---: |
| Parent | 3,219.37 ms | 35,257,061,862 | 192,164 KiB |
| Current | 3,202.70 ms | 35,250,840,270 | 192,304 KiB |

This is only a 0.52% clock and 0.018% instruction movement; RSS is noise. The
checkpoint is intentionally reported as neutral on this mostly noncoplanar
control. Current execution is 1,034.33x faster than historical EMBER's
3,312.66 seconds, but remains 35.59x slower and 12.39x larger by RSS than the
historical CGAL 0.09-second/15,516-KiB row.

The shared exact-box Criterion point estimates are 864.20 us (`STRICT`) and
858.94 us (`APPROXIMATE_512`), statistically neutral. They remain 7.23x/7.18x
CGAL with copy outside and 6.70x/6.66x with copy inside. Exact volumes and
topologies are unchanged.

## Profile ownership

A fresh frame-pointer XL profile attributes 46.25% inclusively to pairwise
intersection, 45.03% to exact self-BVH traversal, 38.28% to pair append,
34.67% to polygon intersection, and 25.40% to coplanar intersection. The
retained matrix's lower-dimensional containment is 14.84%. Reusable point
query classify/new are 9.15%/7.04%, and rational normalization falls from
4.84% before query retention to 3.62%.

Outside pair discovery, face corefinement is 22.59%, Hypertri CDT 12.72%,
constraint insertion 11.69%, local topology sorting 5.31%, arrangement point
insertion 5.14%, exact bound overlap 5.05%, local flippability 4.89%, and BVH
construction 4.16%. The next work remains clean reduction of pairwise/coplanar
and exact equality/construction work, followed by Hypertri/BVH locality. The
profile supports no fixture-specific shortcut.

## Linked and source size

Performance is the priority, but all four default-feature canonical consumers
were rebuilt. Relative to the parent:

| Profile / consumer | Native `.text` | Change | `wasm-opt -Oz` | Change |
| --- | ---: | ---: | ---: | ---: |
| release / general | 2,033,826 B | +0.210% | 1,441,629 B | +0.178% |
| release / immediate | 2,036,970 B | +0.209% | 1,443,484 B | +0.177% |
| size / general | 1,080,599 B | +0.222% | 675,111 B | +0.273% |
| size / immediate | 1,081,539 B | +0.222% | 675,517 B | +0.291% |

Hypermesh adds 350 net source lines, including direct malformed-capacity,
scratch-reset, canonical-deduplication, retained-query, and approximate-policy
tests. The bounded 0.18–0.29% linked growth remains an explicit Phase 17 size
recovery target and is retained for the 11.2% instruction and 28–62%
allocation-work reductions.

A rational prefilter before every retained vertex comparison was also tested.
It did not reduce deterministic work and was removed completely. The retained
certainty gate is narrower: it skips equality only when a classification
already demanded by the exact SAT walk is certified.

## Validation and call graph

Hypermesh passes 154 executed tests with six documented ignores, all-feature
and no-default builds, warning-denied Clippy and rustdoc, every fuzz-bin build,
benchmark compilation, formatting, the full ignored YeahRight oracle, and all
three permanent lower-dimensional-contact fuzz seeds under the current code.
Both policies pass every exact pairwise, Boolean, competitive, and policy
test.

The regenerated five-crate graph contains 15,460 nodes and 25,291 edges for
Hyperreal, Hyperlattice, Hyperlimit, Hypertri, and Hypermesh with examples.
It shows the direct classification-matrix-to-point-query edge and contains no
removed EMBER implementation node. Hypercurve and HyperSolve were neither
scanned nor edited.

## Open work

Phase 11 still needs legally distributable external real-world pathologies,
dense-coplanar and fixed-coordinate-complexity scaling siblings, and broader
high-bit/deep-symbolic families. Phase 17 must reduce the remaining 26–34x
torus deficit, 35.59x historical YeahRight deficit, superlinear slope,
input-fact lifetime, RSS, exact equality/construction work, and linked growth.
Phase 18 must still audit every exit condition and completion gate.

## Reproduction

```sh
cargo test --all-features
cargo check --no-default-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --no-deps --all-features
cargo +nightly check --manifest-path fuzz/Cargo.toml --bins
cargo bench --no-run
cargo fmt --all -- --check

taskset -c 11 target/release/examples/competitive_arrangement_probe \
  clipped_voxel_torus_65 intersection strict 11
taskset -c 11 perf stat -r 11 -x, \
  -e task-clock,cycles,instructions,branches,branch-misses,cache-misses \
  target/release/examples/large_mesh_heap_probe voxel-torus-65 strict
taskset -c 11 target/release/examples/large_mesh_kernel_heap_probe \
  voxel-torus-65 approximate-512

env YEAHRIGHT_BENCH=1 taskset -c 11 perf stat -r 3 -x, \
  -e task-clock,cycles,instructions,branches,branch-misses,cache-misses \
  target/release/deps/competitive-92c64513605410c9 \
  full_resolution_yeahright_rotated_intersection_certifies_empty \
  --ignored --exact --nocapture

benchmarks/size-harness/measure.sh default

../tools/hyper-callgraph/target/release/hyper-callgraph \
  --root .. \
  --out-dir ../target/hypermesh-path-callgraph-phase17-retained-coplanar-facts \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --include-examples --format json,dot --per-library
```
