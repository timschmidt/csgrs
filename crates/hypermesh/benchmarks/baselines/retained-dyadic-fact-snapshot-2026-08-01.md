# Single-snapshot retained dyadic facts

Date: 2026-08-01

Hypermesh evidence base: `9ee8e6a2d5e5`

Hyperreal parent: `7262d3037d056c9fee83b07d6d43cc3d7bf65277`

Hyperreal implementation: `e90f501b85c65a37050cdcee31c7c92fb36d7b24`

## Outcome

Exact dyadic product scheduling now reads each `Rational` retained-fact word
once. The previous aggregate paths first queried
`is_internally_unreduced()`, then called `dyadic_denominator_shift()`, which
queried the unreduced bit again before loading the same fact word a third time.
The wide-dyadic multiplication path had the same two-stage pattern. Reduced
factors could therefore require as many as three relaxed atomic loads before
the arithmetic began.

The selected implementation adds one crate-private reduced-only probe. It
takes one relaxed snapshot, rejects the immutable internal-unreduced bit, and
decodes the dyadic/non-dyadic fact from that same word. Hot decoding remains
inlined; genuinely cold denominator inspection and fact learning is one cold,
out-of-line function. The following complete exact paths use the probe:

- two-factor dyadic alignment;
- the unplanned fixed product-sum accumulator;
- the generic fixed product-sum planner; and
- wide-dyadic multiplication with word numerators.

There is no public API, allocation, carrier, compatibility shim, alternate
predicate, or policy branch. Production is 31 insertions and 19 deletions;
the representation/fallback regression is 48 test lines.

Against checkpoint 19, serialized generated-projective work falls about 2.80%
in instructions and 3.17--3.18% in branches. Retained-arrangement work falls
about 0.96% in instructions and 0.70--0.71% in branches. The representative
linked image grows 2,416 text bytes but only 16 aggregate text/data/BSS bytes;
canonical release WASM and every size-profile row shrink. Large-mesh heap is
exactly unchanged.

## Exactness and concurrency proof

The retained word is advisory metadata for an immutable rational value. Its
facts are monotonic and are learned with atomic `fetch_or`. In particular,
`RETAINED_UNREDUCED_INTERNAL` is established when the internal representation
is constructed and is never learned later or cleared while the value is
shared. A published reduced value can therefore never become unreduced, and a
published unreduced value cannot be mistaken for reduced by a later fact race.

For one relaxed snapshot `r`, the reduced-only probe is exhaustive:

1. If the unreduced bit is present, it returns `None`. Every selected caller
   takes its existing exact generic fallback. Original numerators are never
   combined with canonicalized denominator shifts.
2. If an encoded dyadic shift is present, it returns exactly that shift.
3. If the known/non-dyadic bit is present without an encoded shift, it returns
   `None` and the caller takes the same exact fallback as before.
4. If neither learned fact is present, the cold function inspects the immutable
   denominator, returns the exact result, and atomically retains that result.

A concurrent learner can make `r` stale only by adding a fact after the load.
That can cause a second exact denominator inspection, but cannot change the
answer. The whole fact update is one atomic word update, so no partial encoded
shift can be observed. The ordinary canonical probe still canonicalizes
internally unreduced rationals and returns their canonical shift.

The new regression exercises a cold 300-bit dyadic, its cached hit, a cached
non-dyadic, direct reduced-only rejection of an internally unreduced rational,
the ordinary canonical probe on that value, and an end-to-end wide unreduced
dyadic multiplication that must fall through to the generic exact kernel.
AddressSanitizer passes the complete 36-test dyadic selection.

No comparison result, equality result, sign, certainty, or topology is changed.
`STRICT` still permits no terminal approximation. `APPROXIMATE_512` still has
only Hyperlimit's terminal 512-bit equality interpretation. All six measured
rows finish as `Certified` with identical topology under both policies, and the
generated dispatch trace reports zero unknown-fact and zero fallback/abort
events.

## Serialized CPU work

Parent/candidate/candidate/parent processes were pinned to logical CPU 9. Each
process constructs its fixture once and repeats one complete immediate union.
Values are the means of the two processes for each revision. Task time is per
operation; instructions and branches are the retention gate because host load
and frequency varied during this checkpoint.

| Fixture / policy | Repetitions | Parent task ms | Candidate task ms | Task | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated 13,452 / `STRICT` | 101 | 12.095595 | 12.026277 | -0.573% | -0.576% | -2.794% | -3.169% | -0.010% | -0.282% |
| Generated / `APPROXIMATE_512` | 101 | 11.910807 | 12.027518 | +0.980% | +1.088% | -2.799% | -3.178% | -0.872% | -0.405% |
| Retained 4,524 / `STRICT` | 51 | 36.317728 | 36.297227 | -0.056% | -0.192% | -0.956% | -0.703% | -1.135% | -2.021% |
| Retained / `APPROXIMATE_512` | 51 | 35.795942 | 35.840003 | +0.123% | +0.080% | -0.963% | -0.715% | -0.898% | -0.331% |
| Boxes 6,144 / `STRICT` | 2,001 | 1.431646 | 1.454460 | +1.594% | +1.601% | -0.669% | -0.505% | -0.633% | +5.262% |
| Boxes / `APPROXIMATE_512` | 2,001 | 1.464516 | 1.425031 | -2.696% | -2.674% | -0.693% | -0.517% | -0.609% | +6.027% |

Every instruction and branch row improves, as do all branch-miss means. The
opposite-sign short clock movements and box cache-miss movements are not used
to infer regressions: the paired box policies execute the same exact topology,
the approximate task clocks improve while strict clocks move the other way,
and repeated Criterion sessions exposed discrete host-load modes. The direct
retention decision is based on retired work, the complete profiles, and the
absence of a semantic or memory tradeoff.

The final extra multiplication caller was measured independently against the
already selected aggregate candidate. It removed another 116 representative
text bytes and slightly reduced deterministic work, so it is a strict Pareto
extension rather than an unmeasured scope addition.

## Large-fixture heap

Final-source Heaptrack recordings include fixture construction and one complete
immediate union. Strict and approximate recordings match exactly:

| Fixture | Input triangles | Allocations | Reconstructed temporaries | Peak heap |
| --- | ---: | ---: | ---: | ---: |
| Generated projective | 13,452 | 200,753 | 10,359 | 10.69 MiB |
| Retained arrangement | 4,524 | 454,001 | 28,735 | 12.38 MiB |
| Subdivided boxes | 6,144 | 27,209 | 81 | 4.26 MiB |

Every value reproduces checkpoint 19 exactly. The retained row uses the
1,140-facet hull with SHA-256
`5f1ac8a2f8bf2b0c67ce95dc8146cc783155dc9c34eeee164f03446a0830886c`
and one exact subdivision. The change owns no memory and adds no cache.

## Competitive and historical controls

One final full Criterion session on CPU 9 reports:

| Control | Hypermesh | Boolmesh | Manifold-rust |
| --- | ---: | ---: | ---: |
| Exact-cell union, 3,072 triangles/operand | 1.3977--1.4014 ms (1.3992 center) | 7.4090--7.4274 ms (7.4175) | 4.2819--4.3337 ms (4.3055) |
| Projective generated union | 6.3383--6.3820 ms (6.3530 center) | 748.56--753.97 us (751.51) | 661.16--665.62 us (663.36) |

Hypermesh is 5.30x faster than boolmesh and 3.08x faster than manifold-rust on
the exact-cell control. It remains 8.45x and 9.58x slower on the generated
projective control, whose center is 0.10% below checkpoint 19 and whose paired
Criterion classification is no change. The competitors do not retain
Hyperreal coordinates or expose Hyperlimit policy/certainty, so these are
throughput controls rather than exactness oracles.

The exact-cell center is 3.28% above checkpoint 19's 1.3547 ms, but the host
did not support a stable cross-session clock conclusion. A same-source final
repeat immediately produced 1.3658--1.3695 ms, while a rejected contaminated
session produced 2.6733 ms for Hypermesh and 11.159 ms for boolmesh before
manifold returned to 4.2899 ms. At that point load averages were
5.87/6.90/5.74. The serialized approximate box bracket, which is the policy
used by the competitive harness, retires 0.69% fewer instructions and 0.52%
fewer branches while its task clock improves 2.70%. The evidence therefore
records the final full Criterion interval honestly but does not override the
direct A/B with a contaminated cross-session inference.

The directional historical retained baseline remains 944.8 ms, 67.74 MiB,
and 5,020,891 allocations. The current strict direct row is 36.297 ms,
12.38 MiB, and 454,001 allocations: 96.16%, 81.72%, and 90.96% below those
historical values. Fixture and implementation evolution make this a trend, not
a direct A/B.

## Cycle profile

The final 100-operation generated-8 profile was sampled at 1,999 Hz on CPU 9.
It contains 2,323 samples, approximately 4,772,911,846 cycle events, and zero
lost samples. That is 1.85% fewer events than checkpoint 19's 4,862,760,510.

The intended six-product ordering head falls from 4.55% to 3.56% self, and the
generic product-plan head from 2.97% to 2.64%. The four-by-two word-total owner
is visible at 2.62%; the crossing owner falls from 2.99% to 2.84%, and memmove
from 4.31% to 4.19%. Builder, projective construction, lossy conversion, GCD,
and exact normalization remain the next architecture targets. Sampling shares
move as attribution denominators change; the event total and serialized
counters are the quantitative gates.

## Source, linked code, and call graph

The selected hot/cold split pays a small native release text cost while
shrinking release WASM and all size-profile forms:

| Consumer | Profile / format | Checkpoint 19 | Current | Movement |
| --- | --- | ---: | ---: | ---: |
| General | Release native text | 4,034,636 | 4,036,996 | +2,360 (+0.0585%) |
| Immediate | Release native text | 4,068,252 | 4,070,612 | +2,360 (+0.0580%) |
| General | Release WASM `wasm-opt -Oz` | 2,711,129 | 2,702,295 | -8,834 (-0.3258%) |
| Immediate | Release WASM `wasm-opt -Oz` | 2,726,164 | 2,717,330 | -8,834 (-0.3240%) |
| General | Size native text | 1,855,938 | 1,854,058 | -1,880 (-0.1013%) |
| Immediate | Size native text | 1,868,430 | 1,866,574 | -1,856 (-0.0993%) |
| General | Size WASM `wasm-opt -Oz` | 1,152,620 | 1,151,845 | -775 (-0.0672%) |
| Immediate | Size WASM `wasm-opt -Oz` | 1,163,587 | 1,162,204 | -1,383 (-0.1189%) |

The equal-length repeated-operation executable moves from 6,369,272 to
6,372,312 file bytes and from 5,053,106 to 5,055,522 text bytes. Data falls 80
bytes and BSS falls 2,320, leaving aggregate text/data/BSS only 16 bytes larger.
This is the selected performance-first point; minimum-size variants were
measured and rejected below.

The Hypermesh-only graph remains 8,018 nodes / 19,670 edges because production
changes are in Hyperreal. The five-crate graph moves from 19,668 / 39,260 to
19,671 / 39,266: the reduced-only probe, cold learner, and regression add three
syntactic nodes and six edges. The utility's receiver-name heuristic also emits
synthetic aliases for method calls, so graph counts are structural audit
signals rather than machine-code counts. There is no new policy or fallback
spine.

## Rejected alternatives

- Fully inlining cold denominator inspection moved generated instructions and
  branches about -3.03%/-3.74%, but grew representative text by 6,924 bytes.
  It duplicated a genuinely cold branch into every planner for only a small
  incremental counter gain and was fully removed.
- Making the complete reduced-only probe out of line shrank text by 880 bytes
  and aggregate linked sections by 4,096 bytes, but retained only about
  -1.66% instructions and -0.73% branches. That surrendered too much hot-path
  throughput.
- Inlining only the six-by-two unplanned accumulator shrank text by 456 bytes,
  but retained only about -2.0% instructions and -1.4% branches.
- Inlining the unplanned and generic plan while outlining product alignment
  grew text by 2,228 bytes and moved one direct generated pair about
  -2.71%/-2.97%. It saved only 304 text bytes versus the selected aggregate
  coverage while losing deterministic work.

No rejected alternative, temporary repetition hook, or compatibility path
remains in either repository.

## Validation

The final implementation passes:

- default, no-default, and all-feature test matrices for Hyperreal,
  Hyperlattice, Hyperlimit, Hypertri, and Hypermesh;
- 558/558/635 Hyperreal library tests and all integration/doc tests;
- 1,057/1,057/1,058 Hypermesh library tests and every integration suite;
- warning-denied all-target Clippy for all and no-default features in all five
  crates;
- warning-denied rustdoc for both feature surfaces in all five crates;
- formatting and diff checks in all five crates;
- every Hyperreal and Hypermesh benchmark target and every Hypermesh fuzz bin;
- the final 36-test nightly AddressSanitizer dyadic sweep;
- the opt-in YeahRight every-operation closure/degeneracy gate;
- polygon/immediate API consistency;
- the 3,360/13,440-triangle stress gate;
- the 11,894-triangle full-resolution input-validation gate; and
- the all-family dispatch trace, including zero generated unknown-fact and
  fallback/abort events.

The approximately 56-minute full-resolution rotated Boolean was not rerun.
This checkpoint changes only immutable retained-fact load scheduling and exact
fallback admission; it changes no numeric result, predicate, equality policy,
certainty, topology, allocation, or carrier. The established exact CGAL EPECK
empty oracle and prior 3,357.09-second / 319.07-MiB conservative Hypermesh gate
remain applicable. No final-source full-resolution time or memory is claimed.

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
CARGO_TARGET_DIR=/tmp/hyperreal-retained-fact-asan \
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
  --out-dir /tmp/hypermesh-retained-fact-callgraph
../tools/hyper-callgraph/target/release/hyper-callgraph \
  --root .. \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format json --out-dir /tmp/hyperstack-retained-fact-callgraph
```

Machine-readable values are in
`retained-dyadic-fact-snapshot-2026-08-01.toml`.
