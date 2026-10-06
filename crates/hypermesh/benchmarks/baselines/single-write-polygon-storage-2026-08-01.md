# Single-write source-polygon storage checkpoint

Date: 2026-08-01

Direct parent: Hypermesh `5b8f103a387b4f341366466dc3866f5bf05ce3d5`
(checkpoint-18 implementation `88c2bc47c6b25389200a5e6a0f68abc3045694b6`)

Implementation: Hypermesh `df75f3b04474b8cf4803ec9313048728e37fc5d8`

## Summary

`build_polygon_soup_with_edge_mode` constructed each 336-byte
`ConvexPolygon` on the stack and then used `Vec::push`. Optimized x86-64 code
materialized that move as two consecutive 0x150-byte copies: one from the
constructor's stack result to an argument temporary and another from that
temporary to the vector slot. This cost appeared on every admitted source
triangle, under every Boolean operation and both predicate policies.

The builder now reserves one slot, writes the fully validated polygon directly
into that slot, and immediately extends the vector length. Disassembly contains
one 0x150-byte move instead of two. The representative repeated-operation
binary is file-size neutral, loses 52 text bytes and 20 text/data/BSS bytes,
and the hot builder loses 38 bytes. Source grows by six net lines.

Instructions and branches improve on all six large-fixture/policy rows. The
source-heavy generated and box rows remove approximately 0.70%/2.62% of
instructions and 0.66%/2.40% of branches. Fresh APPROXIMATE_512 Criterion
centers improve 5.62% on the exact-cell control and 0.68% on the projective
control. Heap allocation counts and peaks remain exactly unchanged under both
policies.

## Safety, path, and policy proof

The single unsafe block has a local invariant:

1. `stored_polygon` is read from `polygons.len()`.
2. `polygons.reserve(1)` completes before any pointer is obtained. It therefore
   guarantees that `stored_polygon` names one aligned spare slot. If reserve
   panics, the local polygon and the unchanged vector retain ordinary drop
   behavior.
3. `as_mut_ptr().add(stored_polygon).write(polygon)` initializes exactly that
   slot and moves the complete polygon into it without reading or dropping the
   old uninitialized bytes.
4. There is no potentially panicking operation between `write` and `set_len`.
   `set_len(stored_polygon + 1)` makes exactly the initialized slot visible.
   Later adjacency-map work therefore sees an ordinary owned vector element,
   and any later unwind drops it normally.

The function already computes the checked sum of every input triangle count
and constructs the vector with exactly that capacity. Consequently the reserve
is a no-allocation capacity check on every valid loop iteration. It is retained
anyway so the unsafe proof is local and remains valid if the surrounding
preallocation is later restructured. Capacity overflow and allocation failure
retain `Vec::push`'s standard behavior.

All validation remains before storage: index bounds, the same exhaustive four
constructor cases, source-triangle identities, support validity, degeneracy,
and winding deltas. Every error exits before the vector length changes. All
post-storage paths are also unchanged: adjacent support registration, indexed
edge balance, non-PWN checks, fallback, output repair, triangulation, and exact
closure certification.

The change moves an initialized carrier only. It performs no scalar operation,
comparison, predicate, cache lookup, certainty merge, or terminal decision.
`STRICT` still permits no approximate terminal. `APPROXIMATE_512` still reaches
approximation only through Hyperlimit's 512-bit equality terminal. Both-policy
large probes return identical topology and `Certified` certainty. No public or
internal compatibility path was added.

## Serialized CPU work

Parent/candidate/candidate/parent processes were pinned to logical CPU 9.
Fixture construction occurs once per process and the complete immediate union
is repeated. Values are means across the two processes for each revision;
task time is reported per repeated operation. Instructions and branches are
the deterministic retention gate.

| Fixture / policy | Repetitions | Parent task ms | Candidate task ms | Task | Cycles | Instructions | Branches | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated 13,452 / `STRICT` | 101 | 11.713774 | 11.602085 | -0.953% | -0.976% | -0.696% | -0.651% | -6.441% |
| Generated / `APPROXIMATE_512` | 101 | 11.705132 | 11.783096 | +0.666% | +0.637% | -0.703% | -0.661% | -4.097% |
| Retained 4,524 / `STRICT` | 51 | 35.212783 | 35.080920 | -0.374% | -0.439% | -0.073% | -0.068% | -0.502% |
| Retained / `APPROXIMATE_512` | 51 | 34.970740 | 35.138934 | +0.481% | +0.457% | -0.076% | -0.073% | -1.832% |
| Boxes 6,144 / `STRICT` | 2,001 | 1.448041 | 1.397469 | -3.492% | -3.258% | -2.623% | -2.398% | -8.289% |
| Boxes / `APPROXIMATE_512` | 2,001 | 1.436324 | 1.437084 | +0.053% | +0.162% | -2.623% | -2.397% | -5.756% |

Branch misses improve on both box rows and the generated APPROXIMATE_512 row;
they move +0.329%, +0.539%, and +0.683% on the remaining rows while total
branch work and cache misses fall. The short task-clock rows remain sensitive
to order and frequency. In particular, no regression claim is inferred from
the three small positive APPROXIMATE_512 clock means: deterministic work falls
in every row, and the adjacent APPROXIMATE_512 Criterion controls improve.

## Large-fixture heap

The clean final-source release probe includes fixture construction and one
complete immediate union. Strict and approximate recordings match exactly:

| Fixture | Input triangles | Allocations | Reconstructed temporaries | Peak heap |
| --- | ---: | ---: | ---: | ---: |
| Generated projective | 13,452 | 200,753 | 10,359 | 10.69 MiB |
| Retained arrangement | 4,524 | 454,001 | 28,735 | 12.38 MiB |
| Subdivided boxes | 6,144 | 27,209 | 81 | 4.26 MiB |

Every allocation count, reconstructed temporary count, and peak reproduces
checkpoint 18 exactly. This is expected: the source vector was already
preallocated to the checked triangle total, so `reserve(1)` never allocates on
the measured path. The retained row uses the 1,140-facet YeahRight hull fixture
and one exact subdivision, as required by the memory gate.

## Competitive and historical controls

A fresh final-source Criterion session pinned to CPU 9 reports:

| Control | Hypermesh | Boolmesh | Manifold-rust | Classification |
| --- | ---: | ---: | ---: | --- |
| Exact-cell union, 3,072 triangles/operand | 1.3524--1.3560 ms (1.3547 center) | 7.4128--7.4529 ms (7.4327) | 4.2749--4.3012 ms (4.2850) | Hypermesh improved 5.17% in Criterion's paired estimate |
| Projective generated union | 6.3440--6.3771 ms (6.3595 center) | 744.79--748.42 us (746.05) | 661.16--665.20 us (663.50) | Hypermesh movement remained within Criterion's noise threshold |

Hypermesh is 5.49x faster than boolmesh and 3.16x faster than manifold-rust on
the exact-cell row. Its center is 5.62% below checkpoint 18's 1.4354 ms. On the
projective control, Hypermesh remains 8.52x and 9.58x slower, but its center is
0.68% below checkpoint 18's 6.4028 ms. The competitors do not retain Hyperreal
coordinates or expose Hyperlimit policy/certainty, so they are throughput
controls rather than exactness oracles.

The directional historical retained baseline remains 944.8 ms, 67.74 MiB peak
heap, and 5,020,891 allocations. The current strict row is 35.081 ms, 12.38 MiB,
and 454,001 allocations: approximately 96.29%, 81.72%, and 90.96% below those
historical values. Fixture and implementation evolution make this a trend, not
a direct A/B.

## Cycle profile

The final 100-operation generated-8 profile was sampled at 1,999 Hz on CPU 9.
It contains 2,350 samples, approximately 4,862,760,510 cycle events, and zero
lost samples. That is 2.08% fewer events than checkpoint 18's 4,965,995,227.

`__memmove_avx_unaligned_erms` falls from 6.00% to 4.31% self. The builder is
4.81% self. Disassembly explains the remaining move: the source polygon still
must travel once from its fully validated stack representation into its stable
vector address, but the intermediate argument copy is gone. The other leading
self heads remain exact product-sum ordering, projective construction, lossy
enclosure extraction, crossing splitting, GCD, rational classification, and
normalization. Sampling percentages are attribution evidence; serialized
instructions and branches are the retention gate.

## Source, linked code, and call graph

Production source changes are seven insertions and one deletion. This is
Hypermesh's only production unsafe block; its complete local proof is above.

Against checkpoint 18, the canonical linked text rows are:

| Consumer | Profile / format | Parent bytes | Candidate bytes | Movement |
| --- | --- | ---: | ---: | ---: |
| General | release native text | 4,034,636 | 4,034,636 | 0 |
| Immediate | release native text | 4,068,252 | 4,068,252 | 0 |
| General | release `wasm-opt -Oz` | 2,711,129 | 2,711,129 | 0 |
| Immediate | release `wasm-opt -Oz` | 2,726,164 | 2,726,164 | 0 |
| General | size native text | 1,855,802 | 1,855,938 | +136 (+0.0073%) |
| Immediate | size native text | 1,868,302 | 1,868,430 | +128 (+0.0069%) |
| General | size `wasm-opt -Oz` | 1,152,628 | 1,152,620 | -8 (-0.0007%) |
| Immediate | size `wasm-opt -Oz` | 1,163,596 | 1,163,587 | -9 (-0.0008%) |

The equal-build repeated release binary remains 6,369,296 file bytes. Text
falls 52 bytes, data is unchanged, and BSS grows 32 bytes through linked
layout, leaving text/data/BSS 20 bytes smaller. The hot builder falls from
0x324d to 0x3227 bytes, or 38 bytes. Performance is the primary gate; the
selected spelling also avoids release file/text growth and slightly shrinks
the representative linked total.

The source call graph moves from 8,014 nodes / 19,666 edges to 8,018 / 19,670
for isolated Hypermesh and from 19,664 / 39,256 to 19,668 / 39,260 for all five
crates. This is solely the utility's receiver-name heuristic: it removes the
synthetic `polygons::push` node/edge and adds synthetic `reserve`, `as_mut_ptr`,
`add`, `write`, and `set_len` node/edges. Optimized code contains no new
out-of-line helper and no new policy, predicate, allocation, fallback, or
topology spine.

## Rejected spellings

- Pushing before support validation and mutating the stable vector element was
  safe and reduced the copy count, but an extended box bracket made runtime and
  cycles approximately 1.4% worse. Keeping validation on the hot stack won.
- Validating on the stack, pushing, then setting identity still produced two
  0x150-byte copies.
- `spare_capacity_mut()[0].write` was correct and fast but added 56 release text
  bytes and 24 data bytes relative to the parent. The direct pointer spelling
  generates smaller code with the same invariant.
- `extend([polygon])` and `extend(std::iter::once(polygon))` both retained two
  copies and introduced out-of-line generic helpers. They added 476/212 text
  bytes and 1,112/624 file bytes, respectively.
- Omitting `reserve(1)` saved another 20 text bytes relative to the selected
  spelling and a small amount of deterministic work, but it made correctness
  depend on a distant preallocation invariant. It was rejected so every future
  path has a local capacity proof.

All experimental forms were fully removed.

## Validation

Final implementation results:

- default and no-default library suites: 1,057 / 1,057 passed;
- all-feature library suite: 1,058 passed;
- every integration suite passed in all three configurations;
- warning-denied all-target Clippy passed for all and no-default features;
- warning-denied rustdoc passed for all and no-default features;
- every fuzz binary checked and every benchmark target compiled;
- formatting and diff checks passed;
- AddressSanitizer passed all 14 internal mesh tests and seven polygon-soup
  integration paths, including owned, borrowed, empty, degenerate, open,
  non-PWN, and balanced non-manifold inputs;
- the opt-in YeahRight every-operation closure/degeneracy test passed;
- polygon and immediate APIs remained consistent for every operation;
- the 3,360/13,440-triangle stress test passed;
- the 11,894-triangle full-resolution input validation passed; and
- the all-family dispatch trace passed, with zero unknown-fact events and zero
  generated-projective fallback/abort events.

The approximately 56-minute full-resolution rotated Boolean was not rerun.
This change only changes one initialized carrier move and does not affect
normalization, scaling, predicate dispatch, terminal policy, or output topology;
the last certified-empty 319.07 MiB manual gate remains applicable under the
plan's rerun rule.

Representative commands:

```text
cargo test --locked --no-fail-fast
cargo test --locked --no-default-features --no-fail-fast
cargo test --locked --all-features --no-fail-fast
cargo clippy --locked --all-targets --all-features -- -D warnings
cargo clippy --locked --all-targets --no-default-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --no-default-features
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo bench --locked --all-features --no-run
YEAHRIGHT_BENCH=1 cargo bench --locked --bench dispatch_trace --features dispatch-trace
./benchmarks/size-harness/measure.sh default

CARGO_TARGET_DIR=/tmp/hypermesh-single-write-asan \
RUSTFLAGS='-Zsanitizer=address' ASAN_OPTIONS=detect_leaks=0 \
cargo +nightly test --locked --target x86_64-unknown-linux-gnu \
--lib 'mesh::tests::' -- --test-threads=1

CARGO_TARGET_DIR=/tmp/hypermesh-single-write-asan \
RUSTFLAGS='-Zsanitizer=address' ASAN_OPTIONS=detect_leaks=0 \
cargo +nightly test --locked --target x86_64-unknown-linux-gnu \
--test core polygon_soup -- --test-threads=1

taskset -c 9 cargo bench --locked --bench competitive -- \
subdivided_overlapping_boxes_3072_each/union
YEAHRIGHT_BENCH=1 taskset -c 9 cargo bench --locked --bench competitive -- \
yeahright_control_hull_subdivided_box/union
```
