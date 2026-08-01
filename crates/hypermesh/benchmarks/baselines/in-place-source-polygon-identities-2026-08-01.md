# In-place source-polygon identity checkpoint

Date: 2026-08-01

Direct parent: Hypermesh `b8b38a7e55a85296dc67e7640f8ff6cbceb9cc82`
(checkpoint-17 implementation `5b1c369da8cf42e6e013dd962149a72447e95f34`)

Implementation: Hypermesh `88c2bc47c6b25389200a5e6a0f68abc3045694b6`

## Summary

Polygon-soup construction used to return each newly built 336-byte
`ConvexPolygon` through a consuming source-identity decorator. In optimized
code that shape left several 0x150-byte carrier copies around the four
constructor paths. It also forced the two infallible indexed constructors into
temporary `Result` values solely to share a final `?` and consuming method.

The source identity is now installed through one inlined `&mut self` setter on
the already initialized local polygon. The indexed constructor arms return the
polygon directly, while only the genuinely fallible owned-position arms apply
`?`. The old consuming method and every caller were removed; there is no
compatibility shim. The exact compact identity value is unchanged:
`RetainedIdentityCycles::SourceTriangle { mesh, vertices }`.

This removes deterministic work from all three representative mesh families.
The largest source-heavy rows improve by 2.40--8.58% in instructions and
2.04--7.12% in branches. Exact-cell wall time improves 9.86--12.14%, and the
generated 13,452-triangle projective row improves 3.48--4.26%. The retained
row removes about 0.27% of instructions; its clock measurements remain
order-sensitive, including a +0.65% APPROXIMATE_512 mean in the short bracket.
An extended bracket confirms the deterministic reduction under both policies.

## Path and policy proof

`build_polygon_soup_with_edge_mode` still covers the same four exhaustive
construction cases:

- retained indexed positions with supplied input planes;
- retained indexed positions with an axis, adjacent-support, or computed
  support hint and deferred edges;
- owned vertices with supplied input planes; and
- owned vertices with the ordinary fallible convex-triangle constructor.

Only the first two cases are infallible. The latter two propagate the same
errors at the same match arms. After construction, every case installs the
same source-triangle identity before the unchanged support-validity and
degenerate-triangle checks. Deferred-edge expansion, supplied-plane handling,
adjacent support reuse, non-PWN validation, Boolean fallback, output repair,
and closure certification are unchanged.

The setter assigns an existing exact provenance enum and performs no numeric
comparison. No Hyperreal representation, Hyperlattice construction,
Hyperlimit predicate, terminal, cache, certainty aggregation, or Hypertri
adapter changed. `STRICT` therefore still forbids terminal approximation;
`APPROXIMATE_512` still terminates only at Hyperlimit's existing 512-bit
equality boundary. All measured outputs under both policies have identical
topology and `Certified` certainty.

## Serialized CPU work

Parent/candidate/candidate/parent processes were pinned to logical CPU 9.
Fixture construction occurs once per process; only the complete immediate
union is repeated. Values are means per operation across two processes per
revision. Instructions and branches are the deterministic retention gate.

| Fixture / policy | Repetitions | Parent task ms | Candidate task ms | Task | Cycles | Instructions | Branches | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated 13,452 / `STRICT` | 101 | 12.119901 | 11.697673 | -3.484% | -3.496% | -2.405% | -2.052% | -2.012% |
| Generated / `APPROXIMATE_512` | 101 | 12.211337 | 11.691634 | -4.256% | -4.296% | -2.398% | -2.041% | -0.907% |
| Retained 4,524 / `STRICT` | 51 | 35.928824 | 35.315686 | -1.707% | -1.702% | -0.273% | -0.228% | -2.287% |
| Retained / `APPROXIMATE_512` | 51 | 35.017843 | 35.244804 | +0.648% | +0.624% | -0.262% | -0.215% | -0.032% |
| Boxes 6,144 / `STRICT` | 2,001 | 1.621467 | 1.461527 | -9.864% | -9.836% | -8.555% | -7.107% | -0.698% |
| Boxes / `APPROXIMATE_512` | 2,001 | 1.612501 | 1.416817 | -12.136% | -12.170% | -8.580% | -7.124% | +0.037% |

Branch misses also improve on both generated and both box rows. They move
+0.106% and +0.180% on the retained rows while total branch work falls.
Because retained wall time changes direction with order and frequency, a
101-repetition follow-up used instructions and branches as its gate: `STRICT`
remained approximately -0.270% / -0.223%, and `APPROXIMATE_512` approximately
-0.268% / -0.219%. No retained clock claim depends on the noisy bracket.

## Competitive and historical controls

A fresh final-source Criterion session reports the 3,072-triangle-per-operand
exact-cell union at 1.4256--1.4406 ms (center 1.4354 ms). Boolmesh reports a
7.4067 ms center and manifold-rust 4.3267 ms. Hypermesh is therefore 5.16x
faster than boolmesh and 3.01x faster than manifold-rust on this throughput
row. The Hypermesh center is 10.23% below checkpoint 17's 1.5989 ms.

The generated projective union reports 6.3740--6.4376 ms (center 6.4028 ms),
0.54% below checkpoint 17's 6.4374 ms. Isolated competitor reruns center at
745.37 us for boolmesh and 664.56 us for manifold-rust, leaving Hypermesh
8.59x and 9.63x slower. Those engines neither retain Hyperreal coordinates nor
expose Hyperlimit policy and aggregate certification, so they remain
throughput controls rather than exactness oracles.

The directional historical retained baseline remains 944.8 ms, 67.74 MiB
peak heap, and 5,020,891 allocations. The current strict row is 35.316 ms,
12.38 MiB, and 454,001 allocations: approximately 96.26%, 81.72%, and 90.96%
below those historical values. Fixture and implementation evolution make this
a trend, not a direct A/B.

## Large-fixture heap

Matched parent/candidate Heaptrack recordings use equal-length executable
paths and include fixture construction plus one complete immediate union. The
refactor is exactly allocation- and peak-neutral, as expected for a stack
carrier move removal:

| Fixture | Parent allocations | Candidate allocations | Parent peak | Candidate peak |
| --- | ---: | ---: | ---: | ---: |
| Generated 13,452 | 200,753 | 200,753 | 10.69 MiB | 10.69 MiB |
| Retained 4,524 | 454,001 | 454,001 | 12.38 MiB | 12.38 MiB |
| Boxes 6,144 | 27,209 | 27,209 | 4.26 MiB | 4.26 MiB |

Matched reconstructed temporary counts are also identical at 10,358, 28,734,
and 80. The clean final-source probe was rebuilt after validation and rerun
under both policies. Its allocation and peak totals reproduce the table
exactly, and `STRICT` / `APPROXIMATE_512` are identical on every fixture. The
clean probe reports 10,359, 28,735, and 81 reconstructed temporaries because
it restores the ordinary two-argument CLI in place of the temporary repeated
operation harness; this is outside the matched algorithm A/B.

## Profile

Matched 100-operation generated-8 profiles were recorded at 1,999 Hz with
zero lost samples. Checkpoint 17 captured 2,454 samples and about
5,109,455,973 cycle events; this implementation captures 2,429 samples and
4,965,995,227 events, a 2.81% event reduction.

`__memmove_avx_unaligned_erms` falls from 6.64% to 6.00% self and
`build_polygon_soup_with_edge_mode` from 6.63% to 4.17%. The former consuming
identity decorator and the inlined setter have no remaining symbol. The next
largest self heads are exact work: fixed product-sum ordering, projective
classification, lossy-enclosure extraction, split crossings, GCD, rational
classification, certified-line construction, and normalization. Sampling
percentages are attribution evidence; the serialized counters are the
retention gate.

## Source, linked code, and call graph

Production changes are 12 insertions and 12 deletions; internal test callers
account for the remaining 27 insertions and 27 deletions. The old consuming
API is fully removed.

| Consumer | Profile / format | Parent bytes | Candidate bytes | Movement |
| --- | --- | ---: | ---: | ---: |
| General | release native text | 4,034,572 | 4,034,636 | +64 (+0.0016%) |
| Immediate | release native text | 4,068,188 | 4,068,252 | +64 (+0.0016%) |
| General | release `wasm-opt -Oz` | 2,711,169 | 2,711,129 | -40 (-0.0015%) |
| Immediate | release `wasm-opt -Oz` | 2,726,204 | 2,726,164 | -40 (-0.0015%) |
| General | size native text | 1,855,914 | 1,855,802 | -112 (-0.0060%) |
| Immediate | size native text | 1,868,414 | 1,868,302 | -112 (-0.0060%) |
| General | size `wasm-opt -Oz` | 1,152,731 | 1,152,628 | -103 (-0.0089%) |
| Immediate | size `wasm-opt -Oz` | 1,163,700 | 1,163,596 | -104 (-0.0089%) |

The equal-path repeated release binary shrinks 400 file bytes and 168 text
bytes. BSS grows 176 bytes through linked layout, leaving text/data/BSS eight
bytes larger (+0.00015%). Performance remains the primary gate; six of the
eight canonical native/WASM text rows also improve.

The call-graph utility reports isolated Hypermesh moving from 8,012 nodes /
19,665 edges to 8,014 / 19,666, and the five-crate graph from 19,662 / 39,255
to 19,664 / 39,256. Its receiver-name heuristic removes four synthetic
`with_source_triangle_edge_identities` nodes and adds six synthetic
`set_source_triangle_edge_identities` nodes across production and tests. In
the optimized production binary the setter is inlined and the consuming node
is absent. No policy, predicate, fallback, allocation, or topology spine was
added.

An intermediate non-inlined mutable setter was smaller and faster than the
old consuming form. Adding `#[inline]` then removed its 179-byte symbol,
shrunk text another 140 bytes, and reduced generated instructions about 0.24%
and box instructions about 0.9%, with retained work neutral-to-improved.

Inlining the indexed deferred-triangle constructor was rejected and fully
removed. It eliminated that 489-byte symbol but grew its caller by 777 bytes,
increased net text by 116 bytes, made retained wall time about 0.9% worse, and
repeatedly raised box cache misses about 7.3%. Its modest instruction reduction
did not justify the runtime/cache/code trade.

## Validation

Final implementation results:

- default and no-default library suites: 1,057 / 1,057 passed;
- all-feature library suite: 1,058 passed;
- every integration suite passed in all three configurations;
- warning-denied all-target Clippy passed for all and no-default features;
- warning-denied rustdoc passed for all and no-default features;
- formatting and diff checks passed;
- every fuzz binary checked and every benchmark target compiled;
- the shared indexed-position/source construction regression passed under
  AddressSanitizer;
- the opt-in YeahRight every-operation closure/degeneracy test passed;
- the polygon/immediate consistency test passed;
- the 3,360/13,440-triangle stress test passed;
- the 11,894-triangle full-resolution input validation passed; and
- the all-family dispatch trace passed, including the generated projective row
  with zero unknown-fact or fallback/abort events.

The approximately 56-minute full-resolution rotated Boolean was not rerun.
This change moves only an initialized polygon carrier and changes neither
normalization, candidate scaling, predicate dispatch, terminal policy, nor
output topology; its last certified-empty 319.07 MiB manual gate remains
applicable under the plan's rerun rule.

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

CARGO_TARGET_DIR=/tmp/hypermesh-in-place-identities-asan \
RUSTFLAGS='-Zsanitizer=address' ASAN_OPTIONS=detect_leaks=0 \
cargo +nightly test --locked --target x86_64-unknown-linux-gnu --lib \
mesh::tests::deferred_certified_triangles_share_one_indexed_position_pool \
-- --exact
```
