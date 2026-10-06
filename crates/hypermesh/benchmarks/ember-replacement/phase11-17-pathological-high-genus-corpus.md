# Phase 11/17 pathological contact and high-genus corpus checkpoint

Captured 2026-08-03 on the established Ryzen 7 5800X3D / CPU 11 protocol at
Hypermesh `5992d713812b8e3ff6e898acc08fb897e98f4f48` (corpus implementation
`403ee0442828e3858ea95bad000485f798ab8004`).

This checkpoint adds permanent path-completeness, scaling, heap, fuzz, and
current CGAL EPECK evidence. It does not claim Phase 11 corpus completion,
Phase 17 performance completion, Phase 18 completion, or CGAL parity.

## Result

The manifest grows from 23 to 29 records with two complementary additions:

- exact edge-touching, vertex-touching, and face-tangent containment Boolean
  cases cover union, intersection, difference, reverse difference, and XOR
  under both policies; and
- one indexed clipped voxel-torus generator supplies 460-, 6,412-, and
  25,100-triangle medium/large/XL points with exact result oracles, direct
  large-mesh heap selectors, and exact CGAL inputs.

All successful Hypermesh rows are `Certified`. `APPROXIMATE_512` is exercised
through the same public entry point but consumes no terminal in these exact
rational cases. No production module, dependency, feature, engine selector,
or coordinate/triangle-count branch changes.

## Lower-dimensional closed-PWN contract

The first attempted edge-touch case correctly exposed a contract boundary:
two boxes meeting along an edge have ordinary closed-manifold inputs, but the
exact union boundary has four incident faces at that edge. It is a valid
directionally balanced closed PWN result and an intentional Hypermesh output,
not a two-manifold output accepted by the shared boolmesh/Manifold/CGAL result
contract.

The three cases therefore live in a separate permanent Hypermesh corpus rather
than weakening competitor validation or mislabeling them as shared-contract
wins. Every output is checked for finite nondegenerate triangles, directed-edge
balance, exact volume, certainty, and all five Boolean truth tables. The
face-tangent containment case additionally exercises partial coplanar overlay.

Each case has a named 32-byte `boolean_box_oracle` seed under `fuzz/seeds`.
The mutable libFuzzer corpus remains ignored. All three seeds execute their
shared four-operation program successfully; the integration test supplies the
reverse-difference row as well.

## Deterministic high-genus scaling family

`clipped_voxel_torus_case(outer)` constructs the indexed boundary of a
rectangular voxel annulus. Its wall and depth are `(outer - 1) / 4`; exposed
voxel quads are emitted once with shared vertex IDs and consistent outward
winding. A box clips the genus-one solid at its exact half-integer x symmetry
plane. This is one general generator and the ordinary production Boolean
engine—there is no benchmark-specific algorithm or engine dispatch.

| Fixture | Tier | Input vertices/triangles (torus) | Total input triangles | Intersection V/T | Exact volume |
| --- | --- | ---: | ---: | ---: | ---: |
| `clipped_voxel_torus_9` | medium | 224 / 448 | 460 | 148 / 292 | 56 |
| `clipped_voxel_torus_33` | large | 3,200 / 6,400 | 6,412 | 1,730 / 3,456 | 3,200 |
| `clipped_voxel_torus_65` | XL | 12,544 / 25,088 | 25,100 | 6,532 / 13,060 | 25,088 |

The medium point runs through both Hypermesh policies, boolmesh,
Manifold-rust, half-edge validation, and current CGAL EPECK. Large and XL run
through both Hypermesh policies, the direct requested-payload heap boundary,
fresh-process RSS, hardware counters, and CGAL EPECK.

## Identical exact CGAL inputs

`export_cgal_exact_off` writes every binary64 fixture coordinate as the reduced
`numerator/denominator` returned by `Real::exact_rational`. It never uses a
rounded display approximation. CGAL 6.0.3 EPECK therefore consumes the same
rational coordinates as Hyperreal.

The generated left/right SHA-256 pairs are:

| Extent | Left | Right |
| ---: | --- | --- |
| 9 | `cbdb30250dc6bbd07f5978a5962d54b19083b21c90ca927993c70aa656cfa257` | `ad5e42da04accef93c6b570cbc0b8a2581b6e14cccb3e9efc8d81eac02a8ba37` |
| 33 | `e957a01081b741947d3ce71956ece30f7863f6be26d4b798dd0e397b68546a99` | `c3983115300d1fc4ed2b96c73ce254816183b8b9e922495eab159734f5e96073` |
| 65 | `1669686d993366a7f0c2da5bb5ded0ed3672f039f5ce76e3eb2edb62042cf85b` | `19e5da2af93b0fa83365b4b4cd7de19ec0a0e07b648c25b82a1b5e22a04477a7` |

CGAL reports valid, closed, structurally valid output with the same vertex,
triangle, and volume results for every intersection row.

## Current Hypermesh versus CGAL EPECK

Both executables use one thread pinned to CPU 11. Hypermesh throughput is the
aggregate elapsed time divided by 101/31/11 repeated calls after exact import
and policy-qualified PWN priming. CGAL is the median of 21 calls with its
required mutable input copy outside the timed interval. Separate single-call
rows confirm that retained-input warming is not hiding the main gap.

| Input triangles | Hypermesh STRICT | Hypermesh APPROXIMATE_512 | CGAL median (min–max) | STRICT / CGAL | APPROX / CGAL |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 460 | 9.745 ms | 9.836 ms | 0.493 ms (0.484–0.659) | 19.77x | 19.95x |
| 6,412 | 127.903 ms | 128.980 ms | 3.504 ms (3.459–4.725) | 36.51x | 36.81x |
| 25,100 | 602.395 ms | 596.550 ms | 11.839 ms (11.473–13.665) | 50.88x | 50.39x |

Cold STRICT/CGAL calls are 10.329/0.837 ms, 130.908/4.195 ms, and
605.279/13.525 ms. The loss is therefore real in both cold and warm use. It
also grows with scale and is now an explicit per-case Phase 17 gate rather
than being hidden by the faster overlapping-box rows.

The new family had no historical EMBER row, so no historical ratio is
invented. The established full-YeahRight and box historical scorecards remain
the historical comparison. This checkpoint adds a broader current competitor
shape whose deficit must be closed.

## Runtime slope

Eleven fresh-process `perf stat` repetitions on the ordinary, uninstrumented
probe report:

| Fixture | Task clock | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: |
| extent 33 | 149.71 ms | 1.726B | 308.763M | 2.093M | 1.260M |
| extent 65 | 702.32 ms | 8.310B | 1.434B | 8.711M | 6.422M |

For 3.9145x input triangles, task clock grows 4.6912x, instructions 4.8142x,
branches 4.6446x, and cache misses 5.0977x. This is a useful failure gate, not
an approximately linear performance claim. Profiles must determine whether
candidate/event volume, face corefinement, radial work, or scalar scheduling
causes the growing deficit before changing the algorithm.

## Direct large-mesh heap

Each row below represents two serialized processes, one per policy. Every byte
and count is policy-identical and `Certified`.

| Fixture | Prepared input | Incremental kernel peak | Output-live payload | Input fact growth | Alloc calls | Realloc calls |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 6,412 triangles | 656,792 B | 16,388,556 B | 375,368 B | 42,840 B | 305,956 | 37,483 |
| 25,100 triangles | 3,210,704 B | 65,352,128 B | 1,623,160 B | 699,616 B | 1,315,382 | 152,323 |

The 3.9145x triangle increase produces 3.9877x incremental peak and changes
peak bytes per triangle only from 2,556 to 2,604. Kernel peak is therefore
close to linear. Allocation calls grow 4.2993x and total allocated byte churn
4.1551x, while input-attached fact growth rises 16.33x. That fact-lifetime row
is explicitly open.

This family also crosses Hyperreal's compact small-coordinate regime as its
integer grid grows. Prepared input payload rises 4.8885x, so it is not claimed
as a pure topology-only scalar-complexity control. A fixed-coordinate-complexity
sibling is an open corpus item. The current row is still valuable because the
threshold is a real Hyperreal characteristic that production meshes can cross;
it must not be hidden or special-cased away.

Fresh-process RSS is 21,304 versus 9,556 KiB for Hypermesh/CGAL at the large
point (2.23x), and 73,716 versus 20,020 KiB at XL (3.68x). The processes use
different fixture decoding and allocator implementations, so direct Hypermesh
requested payload remains separately reported. Both metrics miss their CGAL
targets and remain open.

## Rejected retained-fact experiment

A clean Hyperlimit prototype expanded
`det(a-d, b-d, c-d)` into four affine determinants (24 triple products) to
avoid constructing nine coordinate differences. Exact, translation,
degeneracy, property, and Shewchuk-parity tests passed, but full YeahRight rose
from 36.922B to 43.947B instructions (+19.03%) and from 3.380 to 3.855 seconds
(+14.06%) with materially flat RSS. It was completely removed.

The result is specific and useful: Hyperreal benefits more from retaining and
reusing the nine linear facts than from exposing a larger flat polynomial.
Future work should play to that reuse rather than globally suppressing facts.

## Size, source, and validation

No production or Cargo input changed. All four default-feature size-harness
consumers are byte-identical to the fused-kernel checkpoint:

| Profile/consumer | Native `.text` | `wasm-opt -Oz` |
| --- | ---: | ---: |
| release/general | 2,033,026 B | 1,441,079 B |
| release/immediate | 2,036,178 B | 1,442,929 B |
| size/general | 1,077,231 B | 675,045 B |
| size/immediate | 1,078,179 B | 675,453 B |

The two implementation commits add 699 and remove 123 lines across benchmark
support, examples, manifests, documentation, fuzz seeds, and tests. The box
corpus now derives its elementary AABB volume/bounds oracle from compact input
records instead of adding more copied magic-number carriers. None of this code
links into canonical native or WASM consumers.

Validation at `5992d713` includes 112 unit, 8 Boolean, 6 executed competitive,
9 manifest, 2 intersection, 9 policy, and 2 README tests (148 executed; six
opt-in/manual ignores), warning-denied all-target/all-feature Clippy, warning-
denied rustdoc, every fuzz-bin check, three fixed fuzz-seed executions, release
probe/exporter builds, formatting/diff checks, and the default size harness.

## Open work

Phase 11 still needs legally distributable real-world pathologies, dense
coplanar and fixed-coordinate-complexity scaling siblings, broader high-bit and
deep-symbolic families, and stage-specific heap attribution. Phase 17 must
reduce the torus runtime slope, allocation churn, and input fact-lifetime cliff
without giving back exactness or clean algorithms, then close current CGAL
runtime/RSS rows case by case. Linked-size recovery and the Phase 18 completion
audit also remain open.

## Reproduction

```sh
cargo build --release \
  --example competitive_arrangement_probe \
  --example export_cgal_exact_off \
  --example large_mesh_heap_probe \
  --example large_mesh_kernel_heap_probe

target/release/examples/export_cgal_exact_off \
  clipped_voxel_torus_65 /tmp/hypermesh-cgal

taskset -c 11 target/release/examples/competitive_arrangement_probe \
  clipped_voxel_torus_65 intersection strict 11
taskset -c 11 target/competitive/cgal-epeck/hypermesh_cgal_epeck \
  /tmp/hypermesh-cgal/clipped_voxel_torus_65-left.off \
  /tmp/hypermesh-cgal/clipped_voxel_torus_65-right.off \
  intersection 21 outside

taskset -c 11 target/release/examples/large_mesh_kernel_heap_probe \
  voxel-torus-65 approximate-512
taskset -c 11 perf stat -r 11 -x, \
  -e task-clock,cycles,instructions,branches,branch-misses,cache-misses \
  target/release/examples/large_mesh_heap_probe voxel-torus-65 strict

env ASAN_OPTIONS=detect_leaks=0 \
  fuzz/target/x86_64-unknown-linux-gnu/release/boolean_box_oracle \
  fuzz/seeds/boolean_box_oracle/edge_touching_boxes -runs=1

cargo test --all-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --no-deps --all-features
cargo +nightly check --manifest-path fuzz/Cargo.toml --bins
cargo fmt --all -- --check
benchmarks/size-harness/measure.sh default
```
