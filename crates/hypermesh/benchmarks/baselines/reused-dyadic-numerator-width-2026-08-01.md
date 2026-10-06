# Reused dyadic numerator width

Date: 2026-08-01

Hypermesh evidence base: `ea618ad6a69bce042927788dd18c8106ecff07b6`

Hyperreal parent: `8d6eafdbddbb847fcd5d6a1acc4d74e91b3f84ea`

Hyperreal implementation: `a1c524cb46da6d622d324af376cdd7b37a9aba1f`

## Outcome

Normal dyadic binary64 conversion already computed the exact numerator bit
width to derive the unbiased exponent. Its high-word normalizer immediately
called `BigUint::bits()` again on the same immutable numerator. The selected
implementation passes the first width into that private helper and removes the
second scan of the final limb.

The change is two insertions and three deletions in one file. It creates no
public API, compatibility shim, allocation, cache, carrier, predicate, policy,
or topology path. The normal conversion helper remains shared by lossy export,
exact dyadic export, and binary64 enclosure construction.

Every fixture/policy row retires fewer instructions: about 0.232% on generated
projective work, 0.109--0.113% on retained-arrangement work, and
0.018--0.045% on the construction-heavy box control. Branch movement is
0.0003--0.0192%, below the fixture's randomized-path variation. Four of six
task-clock brackets improve; pairing policies gives a lower mean on every
fixture family. Canonical release text shrinks 72 native bytes and 86
optimized-WASM bytes per consumer, size-native is unchanged, size-WASM
shrinks 8--10 bytes, and large-fixture heap is exactly unchanged.

## Exactness and path proof

`normal_dyadic_f64_magnitude` computes `numerator_bits` before any conversion:

1. `self.numerator` is behind a shared reference and cannot change between the
   width computation and high-word extraction.
2. The exponent range check observes only that width and the exact retained
   denominator shift.
3. The selected path passes the identical width value to
   `normalized_high_u64_round_to_odd`; its digit traversal, discarded-bit
   test, round-to-odd rule, scaling, finite check, and return values are
   unchanged.
4. There is one call site for the private normalizer. A zero numerator is
   rejected by each public caller before this normal path, while the helper
   still returns `None` if no highest digit exists.
5. Non-dyadic, subnormal, underflow, overflow, internally unreduced, and
   general approximation paths do not depend on the removed query.

The existing randomized GMP rounding regression directly covers normal wide
dyadics, including both highest-limb alignment shapes and discarded low words.
The conversion suites cover signed values, exact and inexact binary64 values,
subnormals, range failure, non-dyadics, cached facts, and concurrent views. The
final 37-test AddressSanitizer dyadic sweep passes.

The returned binary64 bits are unchanged. These values remain scheduling and
filter inputs: they cannot certify a topological decision. `STRICT` still
permits no terminal approximation, and `APPROXIMATE_512` still delegates only
an unresolved terminal decision to Hyperlimit's 512-bit interpretation. Every
large row finishes `Certified` with identical topology. The generated dispatch
trace reports zero unknown-fact and zero fallback/abort events.

## Serialized CPU work

Parent/candidate/candidate/parent processes were pinned to logical CPU 9. Each
process constructs its fixture once and repeats a complete immediate union.
Values are means of two processes for each revision. Generated and box rows
were lengthened to 501 and 10,001 operations; retained rows use 51 operations.
Instructions are the stable retention gate. Task clock, cycles, branch misses,
and cache misses remain secondary on this shared, frequency-varying host.

| Fixture / policy | Repetitions | Parent task ms/op | Candidate task ms/op | Task | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated 13,452 / `STRICT` | 501 | 10.943323 | 10.955609 | +0.112% | +0.123% | -0.23195% | +0.00146% | +0.285% | +5.485% |
| Generated / `APPROXIMATE_512` | 501 | 10.961257 | 10.898573 | -0.572% | -0.543% | -0.23221% | +0.00124% | +0.641% | +6.509% |
| Retained 4,524 / `STRICT` | 51 | 35.065882 | 34.870392 | -0.557% | -0.739% | -0.11341% | +0.00026% | -0.096% | -0.554% |
| Retained / `APPROXIMATE_512` | 51 | 35.271863 | 34.907647 | -1.033% | -0.993% | -0.10869% | +0.00526% | -0.191% | -1.912% |
| Boxes 6,144 / `STRICT` | 10,001 | 1.382903 | 1.392672 | +0.706% | +0.551% | -0.01758% | +0.01922% | +1.375% | -2.627% |
| Boxes / `APPROXIMATE_512` | 10,001 | 1.427291 | 1.405709 | -1.512% | -1.434% | -0.04537% | +0.00141% | +1.692% | -0.433% |

The strict/approximate clock brackets move in opposite directions on generated
and box work even though both policies consume the same certified path. Their
policy-paired means improve about 0.23% generated and 0.42% boxes; retained
improves about 0.80%. The generic cache-miss counter likewise rises on both
generated policy rows without a consistent clock regression. The source removes
one read-only width query and owns no cache or memory access, so these secondary
movements are recorded as code-layout/host variation rather than claimed as a
cache improvement.

## Large-fixture heap

Final-source Heaptrack recordings include fixture construction and one
complete immediate union. Strict and approximate recordings match exactly:

| Fixture | Input triangles | Allocations | Reconstructed temporaries | Peak heap |
| --- | ---: | ---: | ---: | ---: |
| Generated projective | 13,452 | 200,753 | 10,359 | 10.69 MiB |
| Retained arrangement | 4,524 | 454,001 | 28,735 | 12.38 MiB |
| Subdivided boxes | 6,144 | 27,209 | 81 | 4.26 MiB |

Every value reproduces checkpoints 20 and 21 exactly. The retained row uses
the 1,140-facet hull with SHA-256
`5f1ac8a2f8bf2b0c67ce95dc8146cc783155dc9c34eeee164f03446a0830886c`
and one exact subdivision. The implementation adds no allocation or retained
state.

## Competitive, scalar, and historical controls

One final Criterion session on CPU 9 reports:

| Control | Hypermesh / Hyperreal | Competitor 1 | Competitor 2 |
| --- | ---: | ---: | ---: |
| Exact-cell union, 3,072 triangles/operand | Hypermesh 1.3938--1.4119 ms (1.4009 center) | Boolmesh 7.4762--7.5618 ms (7.5222) | Manifold-rust 4.3332--4.3649 ms (4.3509) |
| Projective generated union | Hypermesh 6.4317--6.4679 ms (6.4500 center) | Boolmesh 768.91--799.99 us (778.90) | Manifold-rust 679.53--698.18 us (686.06) |
| Exact dyadic `to_f64_lossy` | Hyperreal 2.6019--2.6136 ns (2.6075 center) | GMP 7.8411--7.8832 ns (7.8614) | — |

Hypermesh is 5.37x faster than boolmesh and 3.11x faster than manifold-rust on
the exact-cell control. It remains 8.28x and 9.40x slower on the projective
throughput control. All three libraries moved upward in this session, and the
direct serialized A/B is the relevant retention evidence. Hyperreal's focused
dyadic export is 3.02x faster than GMP in the micro-control; this is a current
competitive ratio, not a parent/candidate comparison.

The directional retained baseline remains 944.8 ms, 67.74 MiB, and 5,020,891
allocations. Current strict direct work is 34.870 ms, 12.38 MiB, and 454,001
allocations: about 96.31%, 81.72%, and 90.96% below those historical values.
Fixture and implementation evolution make this a trend rather than a direct
A/B.

## Cycle profile

The final 100-operation generated-8 profile was sampled at 1,999 Hz on CPU 9.
It contains 2,324 samples, approximately 4,770,775,349 cycle events, and zero
lost samples. The event count is 0.08% above checkpoint 21 and 0.04% below
checkpoint 20.

The largest self owners are polygon-soup construction 5.78%, lossy rational
conversion 4.57%, memmove 4.14%, projective input construction 3.99%,
six-product ordering 3.81%, four-by-two product planning 2.51%, malloc 2.44%,
rational linear-form normalization 2.39%, crossing event splitting 2.26%, and
four-by-two word totals 2.25%. Sampling attribution moved while serialized
instructions fell; the counter A/B is the quantitative gate.

## Source, linked code, and call graph

Canonical default-feature consumer sizes improve or hold:

| Consumer | Profile / format | Checkpoint 21 | Current | Movement |
| --- | --- | ---: | ---: | ---: |
| General | Release native text | 4,037,092 | 4,037,020 | -72 (-0.0018%) |
| Immediate | Release native text | 4,070,708 | 4,070,636 | -72 (-0.0018%) |
| General | Release WASM `wasm-opt -Oz` | 2,702,123 | 2,702,037 | -86 (-0.0032%) |
| Immediate | Release WASM `wasm-opt -Oz` | 2,717,158 | 2,717,072 | -86 (-0.0032%) |
| General | Size native text | 1,854,090 | 1,854,090 | 0 |
| Immediate | Size native text | 1,866,606 | 1,866,606 | 0 |
| General | Size WASM `wasm-opt -Oz` | 1,151,896 | 1,151,886 | -10 (-0.0009%) |
| Immediate | Size WASM `wasm-opt -Oz` | 1,162,254 | 1,162,246 | -8 (-0.0007%) |

The equal-length repeated-operation executable moves from 6,374,120 to
6,374,104 file bytes and from 5,057,314 to 5,057,262 text bytes. GNU aggregate
text/data/BSS grows 12 bytes only because BSS padding moves by 64 bytes; no
source-owned object is added. Production source is +2/-3.

The Hypermesh-only graph remains 8,018 nodes / 19,670 edges. The five-crate
graph moves from 19,679 / 39,275 to 19,678 / 39,274. The removed node and edge
are exactly the utility's synthetic
`queries_conversion::value::bits` alias and its call from the normalizer. No
function, policy, fallback, or topology spine is added.

## Rejected alternative

A fused iterator form derived the bit width and highest word from one reverse
digit traversal. It reduced representative text 148 bytes from checkpoint 21,
but direct fused/selected alternation added about 0.094% generated instructions
and generally slowed task time. It also duplicated more limb-shape logic. The
form was fully removed; only the already-computed-width argument remains.

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
This checkpoint passes an already computed integer between two private steps;
it changes no numeric result, predicate, terminal policy, certainty, topology,
allocation, carrier, normalization, or candidate scaling. The established
exact CGAL EPECK empty oracle and prior 3,357.09-second / 319.07-MiB
conservative Hypermesh gate remain applicable. No final-source full-resolution
time or memory is claimed.

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
CARGO_TARGET_DIR=/tmp/hyperreal-bit-reuse-asan \
RUSTFLAGS='-Zsanitizer=address' ASAN_OPTIONS=detect_leaks=0 \
cargo +nightly test --locked --target x86_64-unknown-linux-gnu dyadic --lib

# Hypermesh path/build surfaces
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
taskset -c 9 cargo bench --locked --bench gmp_api -- to_f64_lossy
./benchmarks/size-harness/measure.sh default

# call graphs
../tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . --crate-name hypermesh --format json \
  --out-dir /tmp/hypermesh-bit-reuse-callgraph
../tools/hyper-callgraph/target/release/hyper-callgraph \
  --root .. \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format json --out-dir /tmp/hyperstack-bit-reuse-callgraph
```

Machine-readable values are in
`reused-dyadic-numerator-width-2026-08-01.toml`.
