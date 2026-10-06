# Exact word dyadic export

Date: 2026-08-02

Status: retained as Phase 7 checkpoint 38

Revisions:

- Hyperreal parent: `22c3a445538c5bf6669d7a8bd0bc1bb04c414394`
- Hyperreal candidate: `33e7e4d620c0c95620c1045d09d0089ded105030`
- Hypermesh and evidence parent: `7b8af991ea3f55c626164f0d71960fa8a719cdd7`
- Hyperlattice: `d11ca2f0e825d8e26048cfda5d1101df21dcfef0`
- Hyperlimit: `3e5d8816cd32bba46f48e0c6c13ab7a9da227784`
- Hypertri: `c47601266e0b9b17d0c5a0764fa22b18168ada73`

## Outcome

Hyperreal's borrowed lossy binary64 export now recognizes a normal dyadic
whose numerator has at most 53 bits before normalizing that numerator into an
unsigned high word. It converts the proved-small numerator through the cheaper
signed-word instruction and scales it by an exact normal power of two. The
existing general normal-dyadic path remains the immediate fallback.

Across both Hyperlimit policies, this removes 1.2637% of generated-projective
instructions and 0.2497% of its branches. It removes 0.2659% of retained-mesh
instructions and 0.3645% / 0.2344% of dense-box instructions / branches.
Clean policy-paired clock controls improve 0.62%, 0.10%, and 0.68% for the
generated, retained, and dense fixtures respectively. The retained path adds
0.0767% branches, but the longer confirmation remains neutral on task clock
and cycles.

The production native probe adds 160 bytes of `.text` and file size while its
aggregate linked sections are unchanged. The repeated A/B probe adds 192
bytes. Canonical native text grows by 232--624 bytes and optimized WASM by
212--317 bytes. Large-fixture heap peaks remain 7.50, 11.66, and 1.14 MiB;
the fast path itself allocates nothing.

## Implementation and exact bound

`Rational::to_f64_lossy` already snapshots immutable/monotonic retained facts
once and selects its dyadic schedule before attempting a general conversion.
The new `exact_word_dyadic_f64_magnitude` helper is tried inside that existing
schedule only when:

- the reduced numerator has at most 53 bits; and
- the power-of-two denominator shift is at most 1022.

Those conditions prove that the numerator is losslessly representable through
`i64`, the scale is a finite normal binary64 power of two, and their product is
the exact normal binary value denoted by the rational. The signed lowering
avoids the wider unsigned-word-to-binary64 sequence used after high-word
normalization.

A numerator wider than 53 bits falls through to the unchanged round-to-odd
normalizer. A denominator shift of 1023 or greater also falls through, even
when a wider numerator keeps the result normal. Subnormal, overflowing,
non-dyadic, unreduced-internal, zero, and general rational behavior is
unchanged. Production source changes by 16 insertions and one deletion; the
focused proof test adds 65 lines.

The focused test covers positive and negative values at numerator widths 1,
2, 17, 33, and 53 and denominator shifts 0, 1, 17, 40, 511, and 1022. It checks
the helper bits, lossy output bits, and exact enclosure endpoints. It also
forces both fallthrough boundaries with a 54-bit numerator and a shift-1023
denominator.

## Exactness and policy proof

The helper computes a borrowed primitive view; it does not replace or mutate
the canonical exact `Rational`. Its successful domain is exactly representable,
so it cannot change a filter's mathematical value. Exact predicates continue
to consume the original `Real` / `Rational`, and enclosure users receive equal
exact endpoints.

No Hyperlimit resolver, policy branch, certainty aggregation, or predicate is
changed. `STRICT` continues to consume only structural, filtered, or exact
decisions. `APPROXIMATE_512` still differs only when Hyperlimit reaches its
terminal 512-bit equality/sign interpretation. Hypermesh continues to pass the
selected policy through its immutable `MeshContext`.

Both policies return `Certified` with identical topology on every large
fixture:

| Fixture | Input triangles | Output vertices | Output triangles | Result |
| --- | ---: | ---: | ---: | --- |
| Generated projective | 13,452 | 154 | 304 | `Certified` |
| Retained arrangement | 4,524 | 625 | 1,246 | `Certified` |
| Dense boxes | 6,144 | 27 | 50 | `Certified` |

## Serialized CPU A/B

Parent and candidate use equal native release flags and the same temporary
operation-repetition hook. The hook is absent from every production tree.
Runs are serialized on CPU 9 in parent/candidate/candidate/parent brackets;
the clean dense strict confirmation uses the reverse order. Instructions and
branches are the deterministic retention gate. Task clock and cycles confirm
direction after outliers are excluded.

Arithmetic means across the two policies:

| Fixture | Repetitions | Task clock | Cycles | Instructions | Branches |
| --- | ---: | ---: | ---: | ---: | ---: |
| Generated 13,452 | 1,001 | -0.6209% | -0.6330% | -1.2637% | -0.2497% |
| Retained 4,524 | 201 / 401 | -0.1009% | -0.1552% | -0.2659% | +0.0767% |
| Dense boxes 6,144 | 10,001 | -0.6764% | -0.5955% | -0.3645% | -0.2344% |

Per-policy clean brackets:

| Fixture / policy | Task clock | Cycles | Instructions | Branches |
| --- | ---: | ---: | ---: | ---: |
| Generated / `STRICT` | +0.1334% | -0.0155% | -1.2615% | -0.2463% |
| Generated / `APPROXIMATE_512` | -1.3758% | -1.2530% | -1.2660% | -0.2532% |
| Retained / `STRICT` | -0.1674% | -0.2343% | -0.2668% | +0.0760% |
| Retained / `APPROXIMATE_512`, 401 confirmation | -0.0345% | -0.0760% | -0.2650% | +0.0774% |
| Dense boxes / `STRICT`, reverse confirmation | -0.2200% | -0.1297% | -0.3662% | -0.2367% |
| Dense boxes / `APPROXIMATE_512` | -1.1328% | -1.0613% | -0.3627% | -0.2321% |

The first retained approximate and dense strict brackets contained large
clock/cycle outliers while their deterministic counters stayed aligned. The
longer/reverse confirmations above replace those clocks. Branch-miss counts
move about +1.1% on retained and +2.1--5.6% on dense, but lower total work wins
on every clean clock/cycle control.

## Profile movement

Equal-flag 501-operation CPU-9 profiles record 18,536 parent and 18,370
candidate cycle samples with zero loss. `Rational::to_f64_lossy` falls from
4.50% to 3.28% self, a 1.22 percentage-point or 27.1% sampled-self reduction.
It moves below both fixed product-sum ordering families and is no longer the
third-largest self owner.

The current leading sampled heads are `compute_boolean` 5.76%, four-by-two
product-sum ordering 4.95%, six-by-two ordering 4.40%, lossy export 3.28%,
word GCD 2.93%, crossing-event splitting 2.87%, allocator internals 2.73%,
projective input-soup construction 2.58%, fixed-512 GCD 2.50%, filter
normalization 2.41%, and the certified rational line filter 2.38%.

## Large-fixture heap

Production, no-hook Heaptrack recordings include fixture construction and one
complete immediate union. The retained OBJ is
`yeahright_boolean_hull.obj`, SHA-256
`5f1ac8a2f8bf2b0c67ce95dc8146cc783155dc9c34eeee164f03446a0830886c`.

| Fixture / policy | Parent allocations | Candidate allocations | Parent peak | Candidate peak | Parent RSS | Candidate RSS |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated / `STRICT` | 200,642 | 200,641 | 7.50 MiB | 7.50 MiB | 17.84 MiB | 17.82 MiB |
| Generated / `APPROXIMATE_512` | 200,641 | 200,641 | 7.50 MiB | 7.50 MiB | 17.81 MiB | 17.83 MiB |
| Retained / `STRICT` | 453,854 | 453,853 | 11.66 MiB | 11.66 MiB | 21.08 MiB | 21.18 MiB |
| Retained / `APPROXIMATE_512` | 453,853 | 453,853 | 11.66 MiB | 11.66 MiB | 21.03 MiB | 21.07 MiB |
| Dense boxes / `STRICT` | 2,148 | 2,147 | 1.14 MiB | 1.14 MiB | 9.06 MiB | 9.17 MiB |
| Dense boxes / `APPROXIMATE_512` | 2,147 | 2,147 | 1.14 MiB | 1.14 MiB | 9.16 MiB | 9.14 MiB |

Direct byte-level Massif A/B uses equal-flag repeated probes:

| Fixture | Parent total | Candidate total | Parent useful | Candidate useful |
| --- | ---: | ---: | ---: | ---: |
| Generated | 8,245,736 B | 8,245,736 B | 7,420,638 B | 7,420,638 B |
| Retained | 12,690,448 B | 12,690,584 B | 11,582,671 B | 11,582,375 B |
| Dense boxes | 1,064,992 B | 1,065,024 B | 1,063,742 B | 1,063,742 B |

The retained total movement is +136 bytes (+0.0011%) while useful heap falls
296 bytes. Dense useful heap is identical and allocator overhead moves 32
bytes. These byte-level movements and RSS noise do not change the checkpoint's
memory frontier.

## Dispatch, linked size, and call graph

The generated dispatch trace is unchanged from checkpoint 37: 97,321 dispatch
events, 676 predicates, 1,411 linear-algebra events, 6,341 cache events, and
12,775 rational temporaries, with zero unknown facts and zero fallback/abort
events. Topology and predicate count are unchanged.

Equal-flag repeated-probe size:

| Measure | Parent | Candidate | Movement |
| --- | ---: | ---: | ---: |
| `.text` section | 3,817,686 B | 3,817,878 B | +192 B |
| GNU text | 4,635,605 B | 4,635,789 B | +184 B |
| GNU aggregate | 4,887,168 B | 4,887,160 B | -8 B |
| File | 5,586,432 B | 5,586,624 B | +192 B |

The no-hook production probe grows 160 bytes in `.text` and file size while
its aggregate remains 4,883,052 bytes.

Canonical consumer sizes:

| Consumer | Native text movement | Native aggregate movement | File movement | Optimized WASM movement |
| --- | ---: | ---: | ---: | ---: |
| General release | +624 B | 0 B | +640 B | +317 B |
| Immediate release | +624 B | +4,096 B | +632 B | +317 B |
| General size profile | +248 B | -8 B | +256 B | +216 B |
| Immediate size profile | +232 B | -8 B | +240 B | +212 B |

The immediate release aggregate movement is BSS/link-layout movement; file
growth remains 632 bytes, and measured whole-process RSS does not regress
directionally. Performance remains the primary optimization objective.

The Hypermesh per-library call graph is byte-identical at 8,051 nodes / 19,877
edges, SHA-256
`efbebd2b6288e1bedefadc6082cac08d94d526383e28849e5d4c9908b570f617`.
The five-crate graph moves by the helper and focused test to 19,774 nodes /
39,555 edges, SHA-256
`ed1c224e43e32bc06ec2e858cb3579e288c281c7e93ea028966c9bf2911c1dc6`.
Hyperlattice, Hyperlimit, Hypertri, and Hypermesh per-library graphs are
byte-identical to checkpoint 37. No mesh path, policy terminal, predicate, or
fallback edge changes.

## Competitive and historical controls

Fresh CPU-9 Criterion centers retain the competitive shape. Hypermesh-only
confirmations are paired with the same-session competitor centers; Criterion
clocks are orientation rather than the direct revision A/B gate.

| Fixture / operation | Hypermesh | Boolmesh | Manifold-rust | Hypermesh relation |
| --- | ---: | ---: | ---: | --- |
| Projective / union | 6.1241 ms | 917.20 us | 812.59 us | 6.68x / 7.54x slower |
| Projective / intersection | 4.4722 ms | 736.87 us | 666.34 us | 6.07x / 6.71x slower |
| Projective / difference | 4.1653 ms | 767.44 us | 676.26 us | 5.43x / 6.16x slower |
| Dense boxes / union | 706.46 us | 6.8038 ms | 4.3413 ms | 9.63x / 6.15x faster |
| Dense boxes / intersection | 514.09 us | 3.8771 ms | 3.3471 ms | 7.54x / 6.51x faster |
| Dense boxes / difference | 645.76 us | 6.4471 ms | 4.0196 ms | 9.98x / 6.22x faster |

Against checkpoint 37 centers, projective Hypermesh union/intersection/
difference improve 2.66%, 6.33%, and 1.11%. Dense centers move +0.30%, +1.54%,
and +0.50%, all within the broader session's implementation-wide movement;
the serialized direct A/B still removes deterministic dense work and improves
clean union clocks.

The direct retained strict bracket averages 33.962 ms per union. Against the
directional historical 944.8 ms row, that is 96.405% lower or 27.82x faster.
The current 11.66 MiB peak, 453,853 allocations, and 21.18 MiB strict RSS are
82.79%, 90.96%, and 74.33% below the historical 67.74 MiB, 5,020,891, and
82.5 MiB. Fixture and implementation evolution make these trend controls,
not direct revision A/B.

## Rejected implementations

- A direct branch using unsigned-word conversion increased generated
  instructions about 0.29% and branches about 0.75%; it was removed.
- Gating the signed-word path on the retained exact-f64 marker removed about
  0.97% of generated instructions but added about 0.34% branches and 288 bytes
  of `.text`; the simpler general proof is better and was retained instead.
- Folding the signed path into the existing normalizer removed only about
  0.84% instructions and added about 0.54% branches on generated work. It also
  retires about 0.41% more instructions than the retained direct helper and was
  removed.
- A nested 64-bit/53-bit dispatch increased generated instructions about 0.26%
  and branches about 0.40%; it was removed.

No rejected implementation, diagnostic counter, temporary repetition hook,
or measurement-only source remains in a production tree.

## Validation

The retained implementation passes:

- Hyperreal default/minimal/all-feature full suites, with 564 / 564 / 641
  library tests, plus warning-denied Clippy/rustdoc and formatting;
- Hyperlattice, Hyperlimit, Hypertri, and Hypermesh default, minimal, and
  all-feature full suites, warning-denied Clippy/rustdoc, and formatting;
- the focused Hyperreal exact/fallback test under AddressSanitizer and all
  1,063 default Hypermesh library tests under AddressSanitizer, with leak
  detection disabled;
- every Hypermesh fuzz-bin check, all-feature benchmark compilation, and
  default/all-feature `wasm32-unknown-unknown` library checks;
- both-policy large-fixture topology/certainty, six production Heaptrack runs,
  three direct Massif A/B rows, serialized CPU controls, all-family dispatch,
  canonical native/WASM size, and five-crate/per-library call graphs; and
- exact policy regressions proving that terminal-consumption propagation still
  occurs only where Hyperlimit resolves an otherwise undecided result.

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

# sanitizer surfaces
CARGO_TARGET_DIR=/tmp/hyperreal-lossy-word-asan \
RUSTFLAGS='-Zsanitizer=address' ASAN_OPTIONS=detect_leaks=0 \
cargo +nightly test --locked --target x86_64-unknown-linux-gnu \
  exact_word_dyadic_view_is_bit_exact_across_the_fast_path --lib
CARGO_TARGET_DIR=/tmp/hypermesh-lossy-word-asan \
RUSTFLAGS='-Zsanitizer=address' ASAN_OPTIONS=detect_leaks=0 \
cargo +nightly test --locked --target x86_64-unknown-linux-gnu --lib

# Hypermesh path/build surfaces
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo bench --locked --all-features --no-run
cargo check --locked --target wasm32-unknown-unknown --no-default-features --lib
cargo check --locked --target wasm32-unknown-unknown --all-features --lib
YEAHRIGHT_BENCH=1 cargo bench --locked --bench dispatch_trace \
  --features dispatch-trace
HYPERMESH_SIZE_TARGET_DIR=/tmp/hypermesh-lossy-word-size-default \
  ./benchmarks/size-harness/measure.sh default

# call graph
../tools/hyper-callgraph/target/release/hyper-callgraph \
  --root .. \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format json --per-library \
  --out-dir /tmp/hypermesh-lossy-word-callgraph
```

Hyperlimit's pre-existing untracked `hyperlimit` executable is untouched.
Machine-readable raw and derived values are in
`exact-word-dyadic-export-2026-08-02.toml`.
