# One-pass dyadic word totals

Date: 2026-08-01

Hypermesh evidence base: `a029d3c24540de8dd201c62c600ec555437bf801`

Hyperreal parent: `a1c524cb46da6d622d324af376cdd7b37a9aba1f`

Hyperreal implementation: `9b171c1231a993a65110cee06fa67ff655f7a4ed`

## Outcome

The specialized six-by-two signed-product ordering path previously built two
term-sized stack arrays, first recording every `u128` magnitude and denominator
shift and then walking them again to align and add the positive and negative
totals. The retained implementation keeps both totals aligned while it visits
each term. A later, larger denominator shift checked-rescales the accumulated
totals; an earlier-grid term is checked-scaled before addition; an already
aligned term avoids a no-op shift.

This removes the `[u128; TERMS]` magnitude and `[u64; TERMS]` shift arrays and
the second term loop. Production is 17 insertions and 16 deletions, including a
three-line invariant comment. A 40-line deterministic differential regression
was added. There is no public API, compatibility shim, allocation, cache,
carrier, predicate, policy, or topology change.

All six serialized fixture/policy rows retire fewer instructions: about
0.513% generated, 0.120--0.122% retained, and 0.446--0.482% on boxes. The
strict/approximate paired task-clock means improve 0.593%, 0.062%, and 0.107%
respectively. Large-fixture heap is exactly unchanged. Canonical release text
and optimized WASM shrink, as does the equal-length repeated-operation binary;
the separately optimized size profile grows 0.013--0.017%, which is retained
because runtime has priority and the canonical artifacts improve.

## Exactness and path proof

For every processed prefix, `positive` and `negative` are nonnegative exact
sums expressed on that prefix's maximum denominator-shift grid:

1. The first live term establishes the grid and is added without scaling.
2. A term below the current grid is multiplied by the same exact power of two
   that the old second pass used.
3. When a term raises the grid by `delta`, multiplying both accumulated sums
   by `2^delta` is distributive over all preceding term magnitudes and is
   therefore identical to scaling those terms separately after the final
   maximum is known.
4. Each side is a monotonic sum of nonnegative magnitudes. Consequently, a
   checked rescale or addition overflows exactly when the corresponding old
   final-grid total cannot fit in `u128`; no successful word path is lost and
   no former fallback becomes a word-path success with a different value.
5. Zero on one side is safe under every shift. If conversion of an oversized
   shift fails while the other side is live, the old second pass must also
   fail while scaling that live prefix. The specialized caller enters this
   helper only after finding at least two nonzero terms.

Factor magnitude multiplication, exact dyadic-shift extraction, shift
addition, sign dispatch, positive/negative checked addition, and the outer
`Option` fallback are unchanged. Equal-grid terms now skip only a multiplication
by one.

The new regression compares this unplanned schedule with the retained planned
two-pass implementation across 512 deterministic cases. It alternates a
narrow population required to succeed with full-word odd numerators and shifts
through 127 that exercise fallback, uses both signs, and presents raised,
lower, and equal alignment grids in arbitrary order. Both success and fallback
populations are asserted.

The word accumulator remains an exact fast path. Any `None` continues into the
existing wider exact arithmetic. `STRICT` still permits no terminal
approximation; `APPROXIMATE_512` still changes only Hyperlimit's unresolved
terminal 512-bit equality/sign interpretation. Every measured operation under
both policies finishes `Certified` with identical topology, and the generated
dispatch trace reports zero unknown-fact and zero fallback/abort events.

## Serialized CPU work

Parent/candidate/candidate/parent processes were pinned to logical CPU 9. Each
process constructs its fixture once and repeats a complete immediate union.
Values are means of two processes per revision. Generated and box controls use
501 and 10,001 operations; the much longer retained control uses 51.
Instructions are the primary retention gate. Task clock, cycles, branch
misses, and cache misses are secondary on this shared frequency-varying host.

| Fixture / policy | Repetitions | Parent task ms/op | Candidate task ms/op | Task | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated 13,452 / `STRICT` | 501 | 10.888573 | 10.832954 | -0.511% | -0.455% | -0.51263% | -0.21754% | +1.164% | -2.795% |
| Generated / `APPROXIMATE_512` | 501 | 10.965689 | 10.891806 | -0.674% | -0.692% | -0.51348% | -0.21794% | +1.133% | -4.080% |
| Retained 4,524 / `STRICT` | 51 | 34.761275 | 34.885392 | +0.357% | +0.389% | -0.12150% | -0.05180% | +2.036% | -0.777% |
| Retained / `APPROXIMATE_512` | 51 | 34.821667 | 34.654118 | -0.481% | -0.495% | -0.11953% | -0.05039% | +0.137% | -1.678% |
| Boxes 6,144 / `STRICT` | 10,001 | 1.382440 | 1.384724 | +0.165% | +0.444% | -0.48196% | -0.21271% | +0.196% | +9.671% |
| Boxes / `APPROXIMATE_512` | 10,001 | 1.372841 | 1.367618 | -0.380% | -0.448% | -0.44574% | -0.18309% | -0.386% | +10.823% |

Policy-paired task means move from 10.927131 to 10.862380 ms generated
(-0.5926%), 34.791471 to 34.769755 ms retained (-0.0624%), and 1.377640 to
1.376171 ms boxes (-0.1067%). The strict retained clock bracket is the sole
material individual regression, despite lower instructions, branches, and
cache misses. Box cache-miss percentages rise under both policies without a
consistent clock or cycle regression. These secondary movements are recorded
as host/code-layout variation, not claimed as cache improvements.

## Large-fixture heap

Final-source Heaptrack recordings include fixture construction and one
complete immediate union. Strict and approximate recordings match each other
and checkpoint 22 exactly:

| Fixture | Input triangles | Allocations | Reconstructed temporaries | Peak heap |
| --- | ---: | ---: | ---: | ---: |
| Generated projective | 13,452 | 200,753 | 10,359 | 10.69 MiB |
| Retained arrangement | 4,524 | 454,001 | 28,735 | 12.38 MiB |
| Subdivided boxes | 6,144 | 27,209 | 81 | 4.26 MiB |

The retained row uses the 1,140-facet hull with SHA-256
`5f1ac8a2f8bf2b0c67ce95dc8146cc783155dc9c34eeee164f03446a0830886c`
and one exact subdivision. Removing fixed local arrays changes no heap
allocation or retained result state.

## Competitive and historical controls

One final Criterion session on CPU 9 reports:

| Control | Hypermesh | Boolmesh | Manifold-rust |
| --- | ---: | ---: | ---: |
| Exact-cell union, 3,072 triangles/operand | 1.3585--1.3642 ms (1.3614 center) | 7.3858--7.4335 ms (7.4133) | 4.2528--4.2776 ms (4.2643) |
| Projective generated union | 6.3713--6.4050 ms (6.3912 center) | 746.21--750.91 us (748.22) | 649.04--653.23 us (651.28) |

Hypermesh is 5.45x faster than boolmesh and 3.13x faster than manifold-rust on
the exact-cell control. It remains 8.54x and 9.81x slower on the projective
throughput control, which remains the primary competitive gap. Because the
whole competitive session moved, the serialized parent/candidate alternation
is the retention evidence.

The directional retained baseline remains 944.8 ms, 67.74 MiB, and 5,020,891
allocations. Current strict direct work is 34.885 ms, 12.38 MiB, and 454,001
allocations: 96.31%, 81.72%, and 90.96% below those historical values. Fixture
and implementation evolution make this a trend, not a direct A/B.

## Cycle profile

The final 100-operation generated-8 profile was sampled at 1,999 Hz on CPU 9.
It contains 2,331 samples, approximately 4,826,908,725 cycle events, and zero
lost samples. The largest self owners are polygon-soup construction 6.45%,
projective construction 4.41%, lossy rational conversion 3.81%, memmove 3.47%,
six-by-two signed-product ordering 3.26%, four-by-two product planning 3.03%,
crossing-event splitting 2.70%, rational linear-form normalization 2.60%, and
mixed-width GCD about 2.56%. Sampling moves the ordering attribution from the
parent profile's 3.81%; serialized counters are the quantitative gate.

## Source, linked code, and call graph

Canonical consumer size movements from checkpoint 22 are:

| Consumer | Profile / format | Parent | Current | Movement |
| --- | --- | ---: | ---: | ---: |
| General | Release native text | 4,037,020 | 4,036,956 | -64 (-0.0016%) |
| Immediate | Release native text | 4,070,636 | 4,070,572 | -64 (-0.0016%) |
| General | Release WASM `wasm-opt -Oz` | 2,702,037 | 2,701,864 | -173 (-0.0064%) |
| Immediate | Release WASM `wasm-opt -Oz` | 2,717,072 | 2,716,899 | -173 (-0.0064%) |
| General | Size native text | 1,854,090 | 1,854,346 | +256 (+0.0138%) |
| Immediate | Size native text | 1,866,606 | 1,866,846 | +240 (+0.0129%) |
| General | Size WASM `wasm-opt -Oz` | 1,151,886 | 1,152,087 | +201 (+0.0175%) |
| Immediate | Size WASM `wasm-opt -Oz` | 1,162,246 | 1,162,447 | +201 (+0.0173%) |

The equal-length repeated-operation executable shrinks from 6,374,104 to
6,371,744 file bytes, 5,057,262 to 5,055,578 text bytes, and 5,317,241 to
5,313,141 aggregate text/data/BSS bytes. Its BSS shrinks from 4,347 to 1,931
bytes. This code-repetition control is a stronger ownership-shape signal than
the modest size-profile growth and supports retaining the faster form.

The Hypermesh-only graph remains 8,018 nodes / 19,670 edges. The five-crate
source graph moves from 19,678 / 39,274 to 19,683 / 39,286. All five nodes and
twelve edges are the new differential test and its test-only closures/calls;
the production edit creates no function or call edge and adds no policy,
terminal, fallback, allocation, ownership, or topology spine.

## Rejected alternatives

The first one-pass form conditionally rescaled each sign side. It reduced
generated instructions about 0.354% and branches about 0.194%, but grew the
representative frame-pointer text by 144 bytes. Rescaling both sides together
when the grid rises improved that form by another roughly 0.061% instructions
and 0.101% branches and put text 660 bytes below the parent. The selected form
also skips equal-grid no-op magnitude scaling, reducing generated work another
roughly 0.18%; direct alternation against the preceding form showed about
0.038% fewer instructions and favorable clocks. The two superseded forms were
fully removed.

No temporary repetition hook, diagnostic counter, comparison assertion,
rejected direct-IEEE conversion, or compatibility path remains in either
repository.

## Validation

The final implementation passes:

- default, no-default, and all-feature test matrices for Hyperreal,
  Hyperlattice, Hyperlimit, Hypertri, and Hypermesh;
- 560/560/637 Hyperreal library tests and all integration/doc tests;
- 1,057/1,057/1,058 Hypermesh library tests and every integration suite;
- warning-denied all-target Clippy for all and no-default features in all five
  crates;
- warning-denied rustdoc for both feature surfaces in all five crates;
- formatting and diff checks in all five crates;
- every Hyperreal and Hypermesh benchmark target and every Hypermesh fuzz bin;
- the final 38-test nightly AddressSanitizer dyadic sweep;
- opt-in YeahRight every-operation closure/degeneracy and polygon/immediate
  consistency gates;
- the 3,360/13,440-triangle stress and 11,894-triangle full-input validation
  gates; and
- the all-family dispatch trace with zero generated unknown-fact and
  fallback/abort events.

The approximately 56-minute full-resolution rotated Boolean was not rerun.
This checkpoint differentially proves the exact word schedule against the
retained two-pass schedule, including identical success/fallback decisions,
and changes no predicate, terminal policy, construction, candidate set, or
topology. The established exact CGAL EPECK empty oracle and prior
3,357.09-second / 319.07-MiB conservative Hypermesh gate remain applicable. No
final-source full-resolution time or memory is claimed.

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

# focused sanitizer and Hypermesh path/build surfaces
CARGO_TARGET_DIR=/tmp/hyperreal-one-pass-asan \
RUSTFLAGS='-Zsanitizer=address' ASAN_OPTIONS=detect_leaks=0 \
cargo +nightly test --locked --target x86_64-unknown-linux-gnu dyadic --lib
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

# competitive, size, and call-graph controls
taskset -c 9 cargo bench --locked --bench competitive -- \
  subdivided_overlapping_boxes_3072_each/union
YEAHRIGHT_BENCH=1 taskset -c 9 cargo bench --locked --bench competitive -- \
  yeahright_control_hull_subdivided_box/union
./benchmarks/size-harness/measure.sh default
../tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . --crate-name hypermesh --format json \
  --out-dir /tmp/hypermesh-one-pass-callgraph-final
../tools/hyper-callgraph/target/release/hyper-callgraph \
  --root .. \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format json --out-dir /tmp/hyperstack-one-pass-callgraph-final
```

Machine-readable values are in
`one-pass-dyadic-word-totals-2026-08-01.toml`.
