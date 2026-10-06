# Phase 17 retained radial facts and sign-only predicate checkpoint

Captured 2026-08-03 on the established Ryzen 7 5800X3D / CPU 11 protocol.
This is a retained Phase 17 optimization checkpoint. Phase 17 corpus and
kernel-lifetime heap work, Phase 18 completion, and per-case CGAL EPECK parity
remain open.

## Result

The exact Boolean engine now keeps a compact fact it had already proved while
classifying authored manifold adjacencies: two source faces meet only on
distinct radial rays along their common edge. Degree-two cell assembly reuses
that fact instead of reconstructing a perpendicular-dot equality. The proof is
stored as one canonical `u64` source-face-pair key. Higher-degree and
nonmanifold radial rings retain the complete exact ordering and equality path.

For exact-rational radial dot work that remains, Hypermesh asks Hyperreal for
the sign of the complete 12-term polynomial rather than constructing a dead
`Real` result. Hyperlimit commit
`9deee2642869735b9b77d333fab2be6f075904ea` applies the same ownership rule to
the wide exact-rational `orient3d` fallback: certified floating and word
filters are unchanged, while the six-term determinant reaches
`Rational::signed_product_sum_ordering` and returns only its sign. Symbolic
inputs continue through the existing policy-aware `Real` expressions.

Hypermesh implementation commit
`a8724b4bdfff3a85009c2d838ca2a6ba0f8a04c2` and that Hyperlimit commit reduce
the 11,894-by-11,894 YeahRight case from 9.59 seconds and 122.117 billion
instructions to 4.82 seconds and 50.394 billion instructions. That is 49.74%
less wall time and 58.73% fewer instructions, with the same exact-empty
`Certified` result. Relative to historical EMBER, the retained engine is now
687.27x faster on this case. Relative to the pinned historical CGAL EPECK row,
Hypermesh is still 53.56x slower; this checkpoint is not a parity claim.

No fixture name, coordinate value, triangle count, operation kind, benchmark
state, or output expectation selects these paths. The choices depend only on
facts carried by `hyperreal::Real`, exact construction identity, and radial
degree.

## Exactness and policy behavior

Both policies execute the same topology and algebra:

- `STRICT` still refuses a genuinely undecided symbolic terminal predicate.
- `APPROXIMATE_512` may terminate that same predicate at approximately 512
  bits and records `Approximate512Consumed` in the operation-wide certainty.
- Exact-rational work is `Certified` under both policies.
- A retained adjacency proof is produced through the same `DecisionContext` as
  the Boolean operation. If `APPROXIMATE_512` was needed to produce it, that
  certainty is recorded before the structural fact is reused. `STRICT` cannot
  silently acquire an approximate fact.
- The sign-only schedules change no polynomial and use no epsilon, primitive
  float ordering, or alternative topology.

The radial dot regression exhausts all 19,683 triples of vectors drawn from
`{-1, 0, 1}^3` and compares the sign-only expanded polynomial with the former
factored exact rational value. Hyperlimit adds 256 generated wide-integer
determinant cases beyond the word kernel and compares the sign-only result with
an independent `i128` determinant. Existing symbolic radial coverage continues
to prove `STRICT` undecided versus `APPROXIMATE_512` consumed.

Malformed retained-pair storage is rejected for equal face IDs, out-of-range
face IDs, and noncanonical ordering. A geometric contact with distinct
construction identity does not acquire the authored-adjacency fact.

## Retained-fact scope

A diagnostic run, removed before committing, counted:

| Radial work class | Count |
| --- | ---: |
| Degree-two edges discharged by the retained proof | 38,796 |
| Degree-two edges requiring exact radial classification | 10,114 |
| Higher-degree edges | 4,924 |
| Higher-degree post-sort exact adjacency classifications | 14,772 |
| Safe retained-proof hits in those higher-degree adjacencies | 0 |

The zero-hit higher-degree experiment was removed. Those rings remain on the
single complete exact algorithm rather than carrying dead special handling.

The pair arena reserves the ordinary closed-triangle incidence estimate
`face_count + face_count / 2`, but this is only a storage seed. Checked growth
remains available for nonmanifold inputs. Final sorting and deduplication make
the packed keys canonical before any binary search.

## Validation

At Hyperlimit `9deee264`:

- 151 unit and 75 integration tests pass (226 total);
- the new wide sign-only property executes 256 cases;
- all-target/all-feature Clippy, warning-denied rustdoc, all fuzz-bin checks,
  formatting, and diff checks pass.

At Hypermesh `a8724b4b`:

- 110 unit, 8 Boolean, 5 executed competitive, 6 corpus, 2 intersection, 9
  policy, and 2 README tests pass (142 executed total);
- six documented opt-in/manual competitive tests remain ignored by the normal
  suite;
- the full ignored YeahRight exact-empty oracle passes separately;
- all-target/all-feature Clippy, warning-denied rustdoc, every fuzz-bin check,
  formatting, and diff checks pass.

The six-crate production call graph (Hyperreal, Hyperlattice, Hyperlimit,
Hypertri, Hypermesh, and CSGRS) contains 18,080 nodes and 29,599 edges. It
records the producer edge from `append_pairwise_intersection` to the retained
pair append, the radial sign-only edge to
`Rational::signed_product_sum_ordering`, and Hyperlimit's exact `orient3d` edge
to the same scalar-owned sign primitive. Hypercurve and HyperSolve were neither
included nor edited.

## Full-resolution pathological case

The permanent manual case intersects the 11,894-triangle YeahRight control
mesh with a rotated copy under `APPROXIMATE_512`. All final repetitions return
an empty mesh with `Certified` certainty.

| Row | Wall | Task clock | Instructions | Maximum RSS |
| --- | ---: | ---: | ---: | ---: |
| Topology-only checkpoint | 9.59 s | 9,628.02 ms | 122,117,257,187 | 200,988 KiB |
| Retained/sign-only checkpoint | 4.82 s | 4,766.29 ms | 50,394,157,929 | 201,544 KiB |

The three-repetition pinned mean also reports 19,448,036,412 cycles,
8,684,791,554 branches, 94,790,398 branch misses, and 43,461,547 cache misses.
The unprofiled row used 4.61 seconds user time, 0.18 seconds system time, no
major faults, and no swaps.

Maximum RSS moves by 0.28% from the topology checkpoint. It remains 38.81%
below historical EMBER's 329,352 KiB, but 12.99x the historical CGAL row's
15,516 KiB.

The post-change frame-pointer profile moves the leading cost away from radial
classification:

| Production subtree | Children samples |
| --- | ---: |
| Pairwise intersection traversal | 42.95% |
| Edge-plane crossing construction | 31.11% |
| Face corefinement | 29.55% |
| Topology-only triangulation | 16.62% |
| Hyperlimit `orient3d` | 13.22% |
| Radial ring assembly | 12.65% |
| Remaining radial dot classification | 5.96% |

Pairwise edge-plane crossing construction is therefore the next profile-led
target. Before this checkpoint, radial dot/ray classification accounted for
about 42.7% of the hard-case profile.

## Exact boxes versus pinned CGAL EPECK

Both engines retain valid closed outputs with union/intersection/difference/
reverse-difference volumes 84/12/52/20 and triangle counts 48/24/40/32.
Hypermesh evaluates all four outputs from one shared arrangement.

| Engine / policy | Median | Ratio to CGAL copy outside | Ratio to CGAL copy inside |
| --- | ---: | ---: | ---: |
| Hypermesh `STRICT` | 954.77 us | 7.98x | 7.40x |
| Hypermesh `APPROXIMATE_512` | 942.37 us | 7.88x | 7.31x |
| CGAL 6.0.3 EPECK, copy outside | 119.5965 us | 1.00x | — |
| CGAL 6.0.3 EPECK, copy inside | 128.9760 us | — | 1.00x |

The Hypermesh medians improve 3.13% and 5.37% from the topology-only
checkpoint. Small-case parity remains open.

## Permanent 6,144-triangle runtime and heap control

Every row returns the same `Certified` 2,410-vertex/4,816-triangle union.
Eleven pinned repetitions report:

| Input path | Policy | Task clock | Instructions | Instruction reduction |
| --- | --- | ---: | ---: | ---: |
| Native retained | `STRICT` | 155.02 ms | 1,662,345,028 | 5.63% |
| Native retained | `APPROXIMATE_512` | 153.68 ms | 1,662,344,721 | 5.62% |
| Raw/general | `STRICT` | 145.52 ms | 1,535,917,001 | 6.11% |
| Raw/general | `APPROXIMATE_512` | 147.05 ms | 1,535,793,170 | 6.11% |

Sequential Heaptrack recordings of the same large fixture report:

| Input path | Policy | Allocations | Recorder temporary | Peak heap | Heaptrack RSS |
| --- | --- | ---: | ---: | ---: | ---: |
| Native retained | `STRICT` | 323,213 | 84,886 | 16,497,378 B | 24.12 MB |
| Native retained | `APPROXIMATE_512` | 323,213 | 84,886 | 16,497,378 B | 24.04 MB |
| Raw/general | `STRICT` | 286,331 | 84,886 | 16,497,386 B | 23.84 MB |
| Raw/general | `APPROXIMATE_512` | 286,331 | 84,886 | 16,497,386 B | 23.91 MB |

Each row removes 8,423 allocation calls (2.54% native, 2.86% general). The
compact proof arena raises peak heap by 73,376–73,728 bytes, at most 0.449%.
The first five-`u32` proof record raised the same fixture by roughly 326 KiB;
it was discarded in favor of the packed key.

## Linked and source size

Performance is the priority, but every canonical native and WASM row was
measured. Relative to the topology-only checkpoint, native `.text` grows
1.09–1.23% and optimized WASM grows 1.82–1.94%:

| Features/profile/consumer | Native `.text` | Delta | Optimized WASM | Delta |
| --- | ---: | ---: | ---: | ---: |
| default/release/general | 1,956,290 | +1.232% | 1,367,406 | +1.859% |
| default/release/immediate | 1,959,442 | +1.230% | 1,369,257 | +1.856% |
| default/size/general | 1,045,599 | +1.092% | 647,508 | +1.923% |
| default/size/immediate | 1,046,555 | +1.093% | 647,907 | +1.922% |
| all/release/general | 2,092,527 | +1.199% | 1,446,553 | +1.818% |
| all/release/immediate | 2,095,375 | +1.198% | 1,448,520 | +1.816% |
| all/size/general | 1,047,191 | +1.090% | 647,639 | +1.938% |
| all/size/immediate | 1,048,163 | +1.092% | 647,909 | +1.937% |

The retained runtime/heap trade is kept because it nearly halves the current
hard-case runtime and removes allocations without duplicating an engine. The
generic 12-by-4 `Real::signed_product_sum` candidate was rejected: it was
slower and linked a large symbolic specialization. The final exact-rational
sign path plus unchanged symbolic fallback is smaller and faster. A sign-only
`orient2d` experiment was also rejected: approximately 0.07% fewer hard-case
instructions did not justify an additional 8.7 KiB specialization.

Hypermesh `src` contains 15,393 Tokei code lines; Hyperlimit contains 17,974.
The commits add tests and explicit malformed-storage coverage as well as the
production schedules. There is no compatibility wrapper or benchmark-only
source.

## Open work

CGAL parity is not reached. The next implementation pass should follow the new
profile into exact edge-plane crossing construction, then revisit the smaller
remaining radial predicate share. The permanent corpus still needs the
remaining Phase 17 exhaustive/pathological/scaling fixtures, direct
kernel-lifetime heap boundaries, and broader current CGAL execution. Phase 18
must still audit every planned exit condition.

## Reproduction

```sh
cargo test --all-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS=-Dwarnings cargo doc --no-deps --all-features
cargo +nightly check --manifest-path fuzz/Cargo.toml --bins
cargo fmt --all -- --check

taskset -c 11 cargo bench --bench competitive -- \
  'competitive_shared_arrangement/hypermesh/overlapping_boxes'
taskset -c 11 perf stat -r 11 -x, \
  -e task-clock,cycles,instructions,branches,branch-misses,cache-misses \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
heaptrack --record-only \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict

env YEAHRIGHT_BENCH=1 /usr/bin/time -v taskset -c 11 \
  target/release/deps/competitive-5e62c5f2446653cb \
  full_resolution_yeahright_rotated_intersection_certifies_empty \
  --ignored --exact --nocapture

benchmarks/size-harness/measure.sh default
benchmarks/size-harness/measure.sh all

tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir hypermesh/target/hypermesh-radial-sign-final-callgraph \
  --format json \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh,csgrs \
  --per-library
```
