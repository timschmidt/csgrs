# Native mesh position-storage reuse checkpoint

Date: 2026-08-01

Direct parent: Hypermesh `05bd7bbf58b7797a33b03cd7d98a72f2ae984ded`
(implementation `7a90f764951b24d1877fee156455bd2deea451ba`)

Implementation: Hypermesh `5b1c369da8cf42e6e013dd962149a72447e95f34`

## Summary

Certified-convex Boolean preparation retained every indexed source triangle
against one operation-owned `Arc<[Point3]>`. Canonical `TriangleMesh` inputs
already own exactly that immutable representation, but their borrowed
`TriangleMeshRef` views discarded the ownership fact and copied the complete
position slice into a second Arc on every operation.

`build_polygon_soup_with_edge_mode` now distinguishes the two existing view
families:

- a native view produced by `TriangleMesh::as_ref()` clones the mesh's existing
  position Arc; and
- a public slice-only view produced by `TriangleMeshRef::new()` still copies
  its borrowed positions into independently owned storage.

The operation therefore keeps the same exact `Real` values, indices, supports,
edge identities, predicates, and lifetime contract while removing one large
position copy and allocation per native operand. The usual two-input Boolean
removes two allocations. No cache, retained fact, mutable alias, public API,
compatibility path, policy state, or terminal was added.

## Ownership and path proof

`TriangleMesh::positions` is an immutable `Arc<[Point3]>`. Its `as_ref()`
constructor is the only source of `TriangleMeshRef::native = Some(mesh)`.
Cloning that Arc retains the same allocation beyond the borrowed input view,
so the returned polygon soup remains independently alive and sees byte-for-byte
the same canonical coordinates.

`TriangleMeshRef::new()` sets `native = None`. That public slice-only path has
no owning Arc to clone and retains the former `Arc::from(mesh.positions)` copy.
The focused regression proves native polygons are pointer-identical to the
input mesh's Arc, all native polygons share that Arc, and the borrowed path
uses a distinct allocation. Both paths materialize the same indexed vertices.

The storage selection executes only when both `defer_edges` and the existing
certified-convex input fact are true. Every other polygon-soup mode retains its
previous `None` path. Supplied input planes, axis-support reuse, adjacent-plane
reuse, support validation, non-PWN validation, Boolean fallback, output repair,
and closure certification are unchanged.

This is below the decision boundary. It calls no Hyperlimit predicate and
cannot consume a terminal. `STRICT` still permits no terminal approximation;
`APPROXIMATE_512` still terminates only at Hyperlimit's existing 512-bit
boundary and reports consumption through aggregate certainty. Every measured
output under both policies is identical and `Certified`.

## Serialized CPU work

Parent/candidate/candidate/parent invocations were pinned to logical CPU 9.
The probe constructs each fixture once and repeats only the complete immediate
Boolean. Values below are the mean per operation of the two parent and two
candidate processes. Instructions and branches are the primary deterministic
retention gate; task clock and cycles improve on every row as well.

| Fixture / policy | Repetitions | Parent task ms | Candidate task ms | Task | Instructions | Branches |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Retained 4,524 triangles / `STRICT` | 51 | 35.20461 | 34.96912 | -0.669% | -0.175% | -0.198% |
| Retained / `APPROXIMATE_512` | 51 | 35.21333 | 34.88108 | -0.944% | -0.171% | -0.193% |
| Generated 13,452 triangles / `STRICT` | 31 | 13.79419 | 13.49419 | -2.175% | -1.351% | -1.573% |
| Generated / `APPROXIMATE_512` | 31 | 13.88258 | 13.54500 | -2.432% | -1.351% | -1.571% |
| Boxes 6,144 triangles / `STRICT` | 1,001 | 1.77353 | 1.62961 | -8.115% | -4.838% | -5.504% |
| Boxes / `APPROXIMATE_512` | 1,001 | 1.80318 | 1.63179 | -9.505% | -4.863% | -5.520% |

The generated 852-triangle projective control targeted by Criterion also
improves: task clock -0.326%, cycles -0.248%, instructions -0.205%, and
branches -0.239% over 301 repetitions. Branch-miss counts on a few strict rows
move against the candidate while total branches, cycles, and task time improve;
the complete raw counter means are recorded in the TOML companion.

## Competitive and historical controls

A fresh same-session Criterion run reports the 3,072-triangle-per-operand
exact-cell union at 1.5972--1.5999 ms (center 1.5989 ms). Boolmesh reports
7.4098--7.4419 ms and manifold-rust 4.2665--4.2894 ms. Hypermesh is therefore
4.64x faster than boolmesh and 2.68x faster than manifold-rust on this
throughput row. The current center is 13.97% below the previously recorded
1.8585 ms Hypermesh row.

The generated projective union reports 6.4178--6.4485 ms (center 6.4374 ms),
0.401% below checkpoint 16's 6.4633 ms center. Boolmesh reports 750.08--755.48
us and manifold-rust 659.72--668.88 us in the same run, leaving Hypermesh 8.55x
and 9.68x slower. Those engines neither retain Hyperreal coordinates nor expose
Hyperlimit policy and aggregate certification, so they remain throughput
controls rather than exactness oracles.

Against the directional historical retained row of 944.8 ms, 67.74 MiB peak
heap, 5,020,891 allocations, and about 82.5 MiB RSS, the current strict row is
34.969 ms, 12.38 MiB, 454,000 allocations, and 18,448 KiB direct maximum RSS.
That is roughly 96.30%, 81.72%, 90.96%, and 78.16% below the historical values.
Fixture and implementation evolution make this a trend, not a direct A/B.

## Large-fixture heap and RSS

Heaptrack records fixture construction plus one complete immediate union.
Parent and candidate executable paths have equal length. Recorder and
reconstructed temporary-allocation counts remain identical. The final-source
candidate was rerun after the direct-`match` source simplification.

| Fixture | Parent allocations | Candidate allocations | Parent peak heap | Candidate peak heap | Peak change |
| --- | ---: | ---: | ---: | ---: | ---: |
| Boxes 6,144 | 27,210 | 27,208 | 4.70 MiB | 4.26 MiB | -9.36% |
| Retained 4,524 | 454,002 | 454,000 | 12.71 MiB | 12.38 MiB | -2.60% |
| Generated 13,452 | 200,754 | 200,752 | 11.66 MiB | 10.69 MiB | -8.32% |

Reconstructed temporary counts remain 80, 28,734, and 10,358 respectively.
Candidate `APPROXIMATE_512` recordings exactly match candidate `STRICT`
allocation, temporary, and peak-heap totals on all three fixtures.

Uninstrumented `/usr/bin/time -v` maximum RSS moves 10,008 to 9,364 KiB on
boxes, 18,672 to 18,448 KiB retained, and 17,868 to 16,872 KiB generated-8.
Heaptrack's own recorder RSS is reported separately in the TOML because its
injection overhead and allocator timing are not the normalized heap gate.

## Profiles

Matched 100-operation generated-8 profiles were recorded at 1,999 Hz. The
parent captured 2,502 samples with zero lost and about 5.206 billion cycle
events; the candidate captured 2,454 with zero lost and about 5.109 billion.
`__memmove_avx_unaligned_erms` falls from 7.79% to 6.64% self. Percentages for
neighboring symbols shift with the smaller denominator, so total deterministic
work and the direct A/B remain the attribution gate.

## Source, linked code, and call graph

Production is four insertions and two deletions. The expanded ownership
regression adds 19 test lines. The canonical linked-code movement is tiny:

| Consumer | Profile / format | Parent bytes | Candidate bytes | Movement |
| --- | --- | ---: | ---: | ---: |
| General | release native text | 4,034,476 | 4,034,572 | +96 (+0.0024%) |
| Immediate | release native text | 4,068,092 | 4,068,188 | +96 (+0.0024%) |
| General | release `wasm-opt -Oz` | 2,711,117 | 2,711,169 | +52 (+0.0019%) |
| Immediate | release `wasm-opt -Oz` | 2,726,152 | 2,726,204 | +52 (+0.0019%) |
| General | size native text | 1,855,890 | 1,855,914 | +24 (+0.0013%) |
| Immediate | size native text | 1,868,390 | 1,868,414 | +24 (+0.0013%) |
| General | size `wasm-opt -Oz` | 1,152,682 | 1,152,731 | +49 (+0.0043%) |
| Immediate | size `wasm-opt -Oz` | 1,163,651 | 1,163,700 | +49 (+0.0042%) |

The repeated release probe grows 136 file bytes and 48 text bytes; its BSS
shrinks 48 bytes, leaving the ELF text/data/BSS total unchanged. Performance
and heap reductions have priority over this bounded linked-layout cost.

Isolated Hypermesh moves from 8,011 nodes / 19,663 edges to 8,012 / 19,665.
The five-crate graph moves from 19,661 / 39,253 to 19,662 / 39,255. The single
new syntactic node is local test/selection structure; there is no policy,
predicate, fallback, ownership carrier, or topology spine.

An initial `Option::map_or_else` spelling was replaced by a direct `match`.
Both lowered within counter variation, while the direct form removed one
closure node from each graph scope. No alternate implementation or shim is
retained.

## Validation

Final-source results:

- default and no-default library suites: 1,057 / 1,057 passed;
- all-feature library suite: 1,058 passed;
- every integration suite passed in all three configurations;
- warning-denied all-target Clippy passed for all and no-default features;
- warning-denied rustdoc passed for all and no-default features;
- every fuzz binary checked and every benchmark target compiled;
- the focused native/borrowed ownership regression passed under AddressSanitizer;
- the opt-in YeahRight every-operation closure/degeneracy test passed;
- the polygon/immediate consistency test passed;
- the 3,360/13,440-triangle stress test passed;
- the 11,894-triangle full-resolution input validation passed; and
- the all-family dispatch trace passed, including the generated projective row,
  with zero unknown-fact or fallback/abort events there.

The approximately 56-minute full-resolution rotated Boolean was not rerun.
This checkpoint changes neither full-width normalization, candidate scaling,
predicate dispatch, nor output topology; its last certified-empty 319.07 MiB
manual gate therefore remains applicable under the plan's rerun rule.

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

CARGO_TARGET_DIR=/tmp/hypermesh-native-position-asan \
RUSTFLAGS='-Zsanitizer=address' ASAN_OPTIONS=detect_leaks=0 \
cargo +nightly test --locked --target x86_64-unknown-linux-gnu --lib \
mesh::tests::deferred_certified_triangles_share_one_indexed_position_pool \
-- --exact
```
