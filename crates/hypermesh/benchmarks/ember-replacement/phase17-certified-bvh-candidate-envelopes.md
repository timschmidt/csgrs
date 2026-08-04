# Phase 17 certified BVH candidate envelopes

Date: 2026-08-03

Implementation revisions: Hypertri `41631f6`, Hypermesh `7088b9a4`

Status: retained Phase 17 checkpoint; Phase 17 and Phase 18 remain open.

## Result

Hypermesh now schedules broad-phase work with compact, certified outward
binary32 envelopes while leaving every topology decision with the existing
exact Hyperlimit predicates. This is one general algorithm for every mesh,
operation, coordinate width, and policy. It adds no fixture dispatch, expected
result shortcut, work limit, compatibility layer, or second Boolean engine.

Each exact polygon AABB receives one contiguous 24-byte envelope. Construction
first asks `hyperreal::Real` for its retained exact-dyadic binary64 view, then
uses the borrowed exact-rational binary64 enclosure when available, and finally
rounds outward to binary32. A symbolic value or exact rational outside the
finite enclosure range marks the envelope unavailable and follows the unchanged
policy-aware exact AABB path. Therefore a filter can prove disjointness, but an
overlap is only a scheduling candidate.

The working BVH retains primitive and node envelopes because reconstructing
either hot set lost substantially more runtime than it saved in memory. The
compact source-query hierarchy does not duplicate them: it reconstructs leaf
and one-axis node envelopes from its already-retained exact extrema. Public BVH
queries still exact-classify leaves. The internal canonical self traversal now
states its conservative-candidate contract explicitly, and its only production
consumer immediately runs the complete exact polygon intersection predicate.

This plays to Hyperreal's retained facts and borrowed exact views without
pretending that its scalar has CGAL EPECK's cost model. The existing binary64
center array remains the BVH partition schedule; envelopes affect rejection
only.

## Exactness and policy

Permanent regressions cover outward conversion at ordinary, subnormal, and
overflow magnitudes; non-dyadic rational false positives; explicit binary32
coarsening at `2^30`; unavailable symbolic values; unavailable 1,101-bit exact
rationals; malformed envelope storage; and canonical self-candidate coverage
against exact brute force. Both `STRICT` and `APPROXIMATE_512` exercise every
policy-aware case. A separate intersection-graph regression proves that a
retained binary32 false positive is rejected by the exact narrow phase.

`STRICT` cannot consume a terminal approximation. `APPROXIMATE_512` still
terminates only through Hyperlimit's 512-bit policy and contributes to the same
aggregate certainty. Every measured rational fixture is policy-identical and
`Certified`; the full dispatch corpus has zero unknown-fact and zero
fallback/abort events.

The full YeahRight trace makes the scheduling effect concrete:

| Event | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| Exact ordered-AABB relations | 60,859 | 4 | -99.993% |
| BVH candidates | 8,023 | 8,023 | unchanged |
| Borrowed dyadic-rational comparisons | 145,630 | 56,906 | -60.92% |

No candidate is removed from this fixture; the saved work is repeated proof
scheduling around the same exact arrangement.

## Full-resolution performance

The stable five-run window used fresh processes pinned to CPU 11. Each process
imports the exact 11,894-by-11,894 rotated YeahRight pair, performs one exact
intersection, validates the certified empty result, and destroys it.

| Metric | `272185db` | `7088b9a4` | Movement |
| --- | ---: | ---: | ---: |
| Median wall time | 1.92 s | 1.88 s | -2.08% |
| Cycles | 7,464,605,759 | 7,239,160,608 | -3.02% |
| Instructions | 19,133,466,915 | 18,514,197,608 | -3.24% |
| Branches | 3,414,974,772 | 3,286,637,679 | -3.76% |
| Cache misses | 19,189,528 | 19,792,580 | +3.14% |

The five wall samples were 1.88, 1.87, 1.86, 1.89, and 1.90 seconds. Later
host activity made wall/cycle repetitions noisy, while instructions repeated
within 0.001%; the deterministic instruction and branch movements are the
stronger gate. The cache counter is a measured cost, not hidden.

Historical EMBER remains 3,312.66 seconds, so the retained row is about
1,762.1x faster. Pinned CGAL EPECK remains 0.09 seconds and 15,516 KiB RSS;
Hypermesh is still about 20.89x slower and 12.24x larger in fresh-process RSS.
Those absolute Phase 18 gates remain open.

## General controls and exact boxes

Three independent `perf stat` repetitions show improvements across topology,
coordinate width, and scale:

| Fixture | Input triangles | Parent instructions | Current instructions | Movement |
| --- | ---: | ---: | ---: | ---: |
| 2,049-bit wide rational | 6,144 | 4,159,897,802 | 3,892,427,738 | -6.43% |
| Clipped voxel torus 33 | 6,412 | 1,214,500,564 | 1,157,159,946 | -4.72% |
| Clipped voxel torus 65 | 25,100 | 5,218,831,419 | 4,957,232,851 | -5.01% |
| Dense coplanar 16 | 6,144 | 3,181,158,003 | 3,053,149,032 | -4.02% |

Branches improve by 9.66%, 6.11%, 6.57%, and 4.79%, respectively. These are
ordinary executions of the same code, not benchmark-specialized paths.

For 1,000 complete shared-arrangement evaluations of ordinary overlapping
boxes, instructions improve 0.98% under both policies and branches improve
1.25%. The five-run medians were 632.628 us (`STRICT`) and 629.603 us
(`APPROXIMATE_512`) during a frequency-sensitive window, so exact-box wall time
does not establish a win over the prior 614.860/617.034 us samples. The current
rows remain roughly 5.29x and 4.88x the pinned CGAL copy-outside/copy-inside
times. Exact-box parity remains open.

## Complete large-fixture heap matrix

The direct global-allocator probe excludes fixture construction and reports the
Boolean kernel's requested-payload boundary. Every selector was run under both
policies; output, certainty, and every heap counter are exactly equal
policy-for-policy.

| Selector | Input triangles | Incremental peak |
| --- | ---: | ---: |
| `boxes-3072` | 6,144 | 16,004,010 B |
| `boxes-3072-general` | 6,144 | 16,063,730 B |
| `dense-coplanar-16` | 6,144 | 18,719,560 B |
| `dense-coplanar-32` | 24,576 | 74,724,848 B |
| `wide-rational-64` | 6,144 | 21,716,146 B |
| `wide-rational-512` | 6,144 | 22,596,194 B |
| `wide-rational-2048` | 6,144 | 31,234,658 B |
| `voxel-torus-33` | 6,412 | 16,591,126 B |
| `voxel-torus-65` | 25,100 | 66,152,426 B |
| `yeahright` | 852 | 5,053,588 B |
| `yeahright-4` | 3,372 | 18,851,122 B |
| `yeahright-8` | 13,452 | 71,530,158 B |
| `yeahright-full-rotated` | 23,788 | 158,258,204 B |

Against the immediate parent, full YeahRight peak remains exactly
158,258,204 B; one contiguous allocation adds 767,496 total allocated bytes
(+0.076%) but is not live at the dominant peak. Wide rational peak rises by
196,584 B (+0.633%), with one allocation and 196,584 added bytes. Both retain
the same output and fact payloads as the parent.

Fresh-process full-fixture RSS is 189,768 KiB under `STRICT` and 189,940 KiB
under `APPROXIMATE_512`, effectively flat relative to the parent and still an
absolute CGAL loss.

## Linked size

The general filter adds a small amount of linked code. Native values are
`.text`; WASM values are `wasm-opt -Oz` bytes.

| Profile / consumer | Parent native | Current native | Parent WASM | Current WASM |
| --- | ---: | ---: | ---: | ---: |
| release / general | 2,024,622 | 2,029,758 (+0.254%) | 1,443,031 | 1,447,589 (+0.316%) |
| release / immediate | 2,027,766 | 2,032,902 (+0.253%) | 1,444,878 | 1,449,447 (+0.316%) |
| size / general | 1,084,319 | 1,088,223 (+0.360%) | 680,776 | 684,175 (+0.499%) |
| size / immediate | 1,085,259 | 1,089,171 (+0.360%) | 681,393 | 684,362 (+0.436%) |

Performance has priority over size, and the large-control work reductions are
materially larger than this linked-size cost. No dependency is added.

## Rejected layouts

- Binary64 envelopes doubled filter payload and were rejected.
- Retaining duplicate envelopes in the compact source hierarchy increased
  large-fixture live heap and was rejected.
- Reconstructing all working-tree envelopes on demand raised the full fixture
  to about 19.79 billion instructions; retaining nodes alone was still about
  19.70 billion. Both were rejected.
- Deriving partition centers from binary32 envelopes increased instructions;
  the existing general binary64 center schedule was restored.

These experiments changed storage/scheduling only. None introduced a
mesh/operation/size/coordinate branch.

## Validation and graph

- Hypermesh passes 173 all-feature tests; default and no-default suites pass,
  with six documented external/manual tests remaining ignored.
- The full-resolution exact oracle passes explicitly.
- All-target/all-feature Clippy and rustdoc pass with warnings denied;
  formatting and diff checks pass.
- Every fuzz target and benchmark target compiles.
- All thirteen large heap selectors pass under both policies with identical
  certified results and counters.
- The regenerated graph covers only Hyperreal, Hyperlattice, Hyperlimit,
  Hypertri, and Hypermesh: 14,732 function nodes and 24,516 edges. It confirms
  that the conservative self-candidate traversal has one production consumer,
  the exact intersection graph builder. Hypercurve and HyperSolve are excluded.

## Open work

This checkpoint does not close Phase 17 or Phase 18. Full YeahRight runtime and
RSS, exact boxes, deeper symbolic inputs, broader real-world corpus coverage,
and final path/requirement audits remain open. The next work should continue to
reduce exact arrangement and scalar-fact lifetime costs through general proof
scheduling, without benchmark dispatch or topology shortcuts.

## Reproduction

```sh
cargo test --all-features
cargo test --no-default-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --all-features --no-deps
cargo check --manifest-path fuzz/Cargo.toml --all-targets
cargo bench --no-run --all-features

YEAHRIGHT_BENCH=1 taskset -c 11 perf stat -r 5 -x, \
  -e cycles,instructions,branches,cache-misses \
  target/release/deps/competitive-92c64513605410c9 \
  --ignored --exact full_resolution_yeahright_rotated_intersection_certifies_empty

target/release/examples/large_mesh_kernel_heap_probe \
  <fixture-selector> <strict|approximate-512>
benchmarks/size-harness/measure.sh
YEAHRIGHT_BENCH=1 cargo bench --bench dispatch_trace --features dispatch-trace

tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir /tmp/hypermesh-certified-bvh-filters-callgraph-2026-08-03 \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format json
```
