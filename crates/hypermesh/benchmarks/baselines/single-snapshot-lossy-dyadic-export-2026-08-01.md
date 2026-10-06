# Single-snapshot lossy dyadic export

Date: 2026-08-01

Hypermesh evidence base: `13c9b99d8233`

Hyperreal parent: `e90f501b85c65a37050cdcee31c7c92fb36d7b24`

Hyperreal implementation: `8d6eafdbddbb847fcd5d6a1acc4d74e91b3f84ea`

## Outcome

`Rational::to_f64_lossy` now carries one relaxed retained-fact snapshot
through both representation admission and dyadic decoding. The previous hot
path loaded the same atomic word in `is_internally_unreduced()`, then entered
`dyadic_denominator_shift()`, which loaded it again. Reduced values therefore
paid two relaxed atomic loads and two representation decisions before lossy
conversion began.

The selected implementation directly tests the immutable internal-unreduced
bit in one snapshot and passes that word to the existing exact dyadic decoder.
Cold denominator inspection and monotonic fact learning are unchanged. The
internally unreduced path still canonicalizes and recursively exports the
canonical value; zero still returns before any dyadic learning; non-dyadic and
non-normal dyadic values still use the existing general conversion path.

There is no public API, compatibility shim, allocation, cache, carrier,
predicate, policy, or topology change. Production is seven insertions and two
deletions in one function, including the concurrency invariant comment. One
61-line regression covers the representation/fact paths.

Every serialized fixture/policy row retires fewer instructions and branches:
generated projective work falls about 0.52%/1.20%, retained-arrangement work
falls about 0.26%/0.65%, and boxes fall 0.07--0.10%/0.20--0.23%. Canonical
linked text moves by only -172 to +96 bytes depending on target/profile, and
large-fixture heap is exactly unchanged.

## Exactness and concurrency proof

The retained word belongs to an immutable rational representation. Its facts
are advisory and monotonic:

1. `RETAINED_UNREDUCED_INTERNAL` is set when an internal lazy coordinate is
   constructed and is never added later or cleared while that value exists.
2. Dyadic-known, dyadic-value, and encoded-shift facts describe the immutable
   denominator and are added with relaxed atomic `fetch_or`.
3. A stale relaxed snapshot can omit a concurrently learned dyadic fact, but
   the existing cold learner then inspects the same immutable denominator and
   reaches the same result. It can only repeat exact work.
4. The whole encoded shift is published in the same atomic word, so readers
   cannot observe a partially written shift.

For a single snapshot, every path remains exhaustive:

- an internally unreduced value enters the unchanged exact canonicalization
  path;
- zero returns `0.0` without inspecting or learning denominator facts;
- an encoded dyadic shift enters the unchanged normal binary64 conversion;
- a learned non-dyadic value skips denominator inspection and uses the general
  conversion;
- an unknown denominator is inspected exactly and its result retained;
- a dyadic outside the normal conversion range falls through to exact-MSD and
  general finite/range handling; and
- conversion failure still returns `None` so `Real` can use its general
  approximation path.

The regression exercises a cold 300-bit dyadic, its cached hit and unchanged
fact word, cold and cached non-dyadic fallback, internally unreduced
canonicalization with a retained canonical shift, and zero's no-learning fast
path. Existing randomized GMP cross-checks cover normal wide-dyadic rounding;
the complete conversion suites cover subnormal, overflow, non-dyadic, cached,
and concurrent `Real` views. The final 37-test AddressSanitizer dyadic sweep
passes.

The returned `f64` bits do not change. Lossy values remain non-certifying
filter inputs. `STRICT` still permits no terminal approximation, while
`APPROXIMATE_512` still delegates only an unresolved terminal equality to
Hyperlimit's 512-bit interpretation. All six large rows finish `Certified`
with identical topology, and the generated dispatch trace reports zero
unknown-fact and zero fallback/abort events.

## Serialized CPU work

Parent/candidate/candidate/parent processes were pinned to logical CPU 9. Each
process constructs its fixture once and repeats a complete immediate union.
Values below are means of the two processes for each revision. Task time is
per operation. Instructions and branches are the retention gate; task clock
and cycles remain secondary because host frequency and load vary.

| Fixture / policy | Repetitions | Parent task ms | Candidate task ms | Task | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated 13,452 / `STRICT` | 101 | 11.383564 | 11.360297 | -0.204% | -0.129% | -0.520% | -1.196% | -0.130% | -2.445% |
| Generated / `APPROXIMATE_512` | 101 | 11.450495 | 11.443911 | -0.058% | -0.059% | -0.524% | -1.200% | -0.110% | -0.950% |
| Retained 4,524 / `STRICT` | 51 | 34.970490 | 34.707843 | -0.751% | -0.689% | -0.258% | -0.649% | +0.730% | -3.324% |
| Retained / `APPROXIMATE_512` | 51 | 34.830882 | 34.937745 | +0.307% | +0.240% | -0.258% | -0.650% | -1.633% | -1.682% |
| Boxes 6,144 / `STRICT` | 2,001 | 1.393793 | 1.363023 | -2.208% | -2.179% | -0.066% | -0.201% | -0.096% | -5.384% |
| Boxes / `APPROXIMATE_512` | 2,001 | 1.368828 | 1.363603 | -0.382% | -0.208% | -0.099% | -0.225% | +0.018% | -4.598% |

Every deterministic work row improves. Five of six task-clock means improve;
the short retained approximate bracket moves +0.31% while its instructions,
branches, and cache misses improve. That isolated clock movement is below the
host's observed cross-session modes and does not override the uniform retired
work result.

## Large-fixture heap

Final-source Heaptrack recordings include fixture construction and one
complete immediate union. Strict and approximate recordings match exactly:

| Fixture | Input triangles | Allocations | Reconstructed temporaries | Peak heap |
| --- | ---: | ---: | ---: | ---: |
| Generated projective | 13,452 | 200,753 | 10,359 | 10.69 MiB |
| Retained arrangement | 4,524 | 454,001 | 28,735 | 12.38 MiB |
| Subdivided boxes | 6,144 | 27,209 | 81 | 4.26 MiB |

Every value reproduces checkpoint 20 exactly. The retained row uses the
1,140-facet hull with SHA-256
`5f1ac8a2f8bf2b0c67ce95dc8146cc783155dc9c34eeee164f03446a0830886c`
and one exact subdivision. The change owns no memory and adds no cache.

## Competitive and historical controls

One final Criterion session on CPU 9 reports:

| Control | Hypermesh | Boolmesh | Manifold-rust |
| --- | ---: | ---: | ---: |
| Exact-cell union, 3,072 triangles/operand | 1.3777--1.3906 ms (1.3836 center) | 7.4253--7.4570 ms (7.4395) | 4.2846--4.3030 ms (4.2928) |
| Projective generated union | 6.3661--6.3813 ms (6.3755 center) | 746.84--749.68 us (748.41) | 656.29--664.35 us (660.32) |

Hypermesh is 5.38x faster than boolmesh and 3.10x faster than manifold-rust on
the exact-cell control. Its center is 1.11% below checkpoint 20's 1.3992 ms,
and Criterion classifies the local movement as within the noise threshold.

The projective center is 0.35% above checkpoint 20's 6.3530 ms and Criterion
reports no performance change. Hypermesh remains 8.52x and 9.66x slower than
the competitors on that throughput control. The direct 13,452-triangle A/B is
the relevant retention evidence and retires 0.52% fewer instructions and 1.20%
fewer branches in both policies. Competitors do not retain Hyperreal exact
coordinates or expose Hyperlimit policy/certainty, so they are throughput
controls rather than exactness oracles.

The directional retained baseline remains 944.8 ms, 67.74 MiB, and 5,020,891
allocations. The current strict direct row is 34.708 ms, 12.38 MiB, and
454,001 allocations: 96.33%, 81.72%, and 90.96% below those historical
values. Fixture and implementation evolution make this a trend, not a direct
A/B.

## Cycle profile

The final 100-operation generated-8 profile was sampled at 1,999 Hz on CPU 9.
It contains 2,296 samples, approximately 4,767,140,061 cycle events, and zero
lost samples. The event count is 0.12% below checkpoint 20's 4,772,911,846.

The largest self owners are polygon-soup construction 5.06%, projective input
construction 4.51%, memmove 4.44%, lossy rational conversion 3.80%, six-product
ordering 3.52%, four-by-two product planning 2.60%, rational linear-form
normalization 2.54%, malloc 2.51%, crossing event splitting 2.47%, and
four-by-two word totals 2.26%. Sampling shares can move while total work falls;
serialized counters are the quantitative gate. Lossy export remains a scalar
architecture target, but its repeated retained-fact work is now removed.

## Source, linked code, and call graph

Canonical consumer sizes are nearly neutral:

| Consumer | Profile / format | Checkpoint 20 | Current | Movement |
| --- | --- | ---: | ---: | ---: |
| General | Release native text | 4,036,996 | 4,037,092 | +96 (+0.0024%) |
| Immediate | Release native text | 4,070,612 | 4,070,708 | +96 (+0.0024%) |
| General | Release WASM `wasm-opt -Oz` | 2,702,295 | 2,702,123 | -172 (-0.0064%) |
| Immediate | Release WASM `wasm-opt -Oz` | 2,717,330 | 2,717,158 | -172 (-0.0063%) |
| General | Size native text | 1,854,058 | 1,854,090 | +32 (+0.0017%) |
| Immediate | Size native text | 1,866,574 | 1,866,606 | +32 (+0.0017%) |
| General | Size WASM `wasm-opt -Oz` | 1,151,845 | 1,151,896 | +51 (+0.0044%) |
| Immediate | Size WASM `wasm-opt -Oz` | 1,162,204 | 1,162,254 | +50 (+0.0043%) |

The equal-length repeated-operation executable moves from 6,372,312 to
6,374,120 file bytes and from 5,055,522 to 5,057,314 text bytes. Its GNU
aggregate text/data/BSS total grows 4,080 bytes; 2,288 bytes are link-layout
padding reported through BSS rather than an added source-owned object. The
canonical consumers above are the release/size decision surface.

The Hypermesh-only graph remains 8,018 nodes / 19,670 edges because production
changes are in Hyperreal. The five-crate source graph moves from 19,671 / 39,266
to 19,679 / 39,275. The production edit creates no function. The focused
regression and the utility's receiver-name aliases account for the syntactic
movement; graph counts are audit signals, not machine-code counts. There is no
new policy or fallback spine.

## Rejected alternatives

- Forcing one shared out-of-line copy of the complete conversion routine
  reduced representative text 13,168 bytes from the parent and retired about
  0.64% fewer generated instructions. However, direct selected/shared
  alternation made the shared form 1.22% slower in strict and 0.72% slower in
  approximate mode, with roughly 4.5--6.2% more cache misses. Performance is
  the primary objective, so this size-first form was fully removed.
- Moving only internal-unreduced canonicalization to a cold out-of-line helper
  left representative aggregate size essentially neutral and still reduced
  generated instructions about 0.30%. Internally unreduced coordinates are
  frequent enough that the extra call surrendered about half the selected
  instruction/branch gain and made generated task time 0.53--0.79% slower.
  The helper was fully removed.

No rejected alternative, temporary repetition hook, diagnostic counter, or
compatibility path remains in either repository.

## Validation

The final implementation passes:

- default, no-default, and all-feature test matrices for Hyperreal,
  Hyperlattice, Hyperlimit, Hypertri, and Hypermesh;
- 559/559/636 Hyperreal library tests and all integration/doc tests;
- 1,057/1,057/1,058 Hypermesh library tests and every integration suite;
- warning-denied all-target Clippy for all and no-default features in all five
  crates;
- warning-denied rustdoc for both feature surfaces in all five crates;
- formatting and diff checks in all five crates;
- every Hyperreal and Hypermesh benchmark target and every Hypermesh fuzz bin;
- the final 37-test nightly AddressSanitizer dyadic sweep;
- the opt-in YeahRight every-operation closure/degeneracy gate;
- polygon/immediate API consistency;
- the 3,360/13,440-triangle stress gate;
- the 11,894-triangle full-resolution input-validation gate; and
- the all-family dispatch trace, including zero generated unknown-fact and
  fallback/abort events.

The approximately 56-minute full-resolution rotated Boolean was not rerun.
This checkpoint reuses advisory metadata in a bit-identical lossy conversion;
it changes no numeric result, predicate, terminal policy, certainty, topology,
allocation, carrier, normalization, or candidate scaling. The established
exact CGAL EPECK empty oracle and prior 3,357.09-second / 319.07-MiB
conservative Hypermesh gate therefore remain applicable. No final-source
full-resolution time or memory is claimed.

Representative commands:

```text
# each of hyperreal, hyperlattice, hyperlimit, hypertri, and hypermesh
cargo test --locked --no-fail-fast
cargo test --locked --no-default-features --no-fail-fast
cargo test --locked --all-features --no-fail-fast
cargo clippy --locked --all-targets --all-features -- -D warnings
cargo clippy --locked --all-targets --no-default-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --no-default-features
cargo fmt --all -- --check

# focused Hyperreal sanitizer
CARGO_TARGET_DIR=/tmp/hyperreal-lossy-snapshot-asan \
RUSTFLAGS='-Zsanitizer=address' ASAN_OPTIONS=detect_leaks=0 \
cargo +nightly test --locked --target x86_64-unknown-linux-gnu dyadic --lib

# Hypermesh build and path surfaces
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo bench --locked --all-features --no-run
YEAHRIGHT_BENCH=1 cargo bench --locked --bench dispatch_trace \
  --features dispatch-trace

# final-source heap; repeat for both policies and all three fixtures
cargo build --locked --release --example large_mesh_heap_probe
YEAHRIGHT_BENCH=1 heaptrack --record-only \
  target/release/examples/large_mesh_heap_probe yeahright-8 strict
YEAHRIGHT_HULL_OBJ=/path/to/yeahright_boolean_hull.obj \
  heaptrack --record-only \
  target/release/examples/large_mesh_heap_probe yeahright strict
heaptrack --record-only \
  target/release/examples/large_mesh_heap_probe boxes-3072 strict

# competitive and linked-size controls
taskset -c 9 cargo bench --locked --bench competitive -- \
  subdivided_overlapping_boxes_3072_each/union
YEAHRIGHT_BENCH=1 taskset -c 9 cargo bench --locked --bench competitive -- \
  yeahright_control_hull_subdivided_box/union
./benchmarks/size-harness/measure.sh default

# call graphs
../tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . --crate-name hypermesh --format json \
  --out-dir /tmp/hypermesh-lossy-snapshot-callgraph
../tools/hyper-callgraph/target/release/hyper-callgraph \
  --root .. \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format json --out-dir /tmp/hyperstack-lossy-snapshot-callgraph
```

Machine-readable values are in
`single-snapshot-lossy-dyadic-export-2026-08-01.toml`.
