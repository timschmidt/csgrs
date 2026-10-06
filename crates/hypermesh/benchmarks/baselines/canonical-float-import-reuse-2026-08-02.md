# Canonical finite-float rational reuse

Date: 2026-08-02

Status: retained as Phase 7 checkpoint 37

Revisions:

- Hyperreal parent: `fde2f8d3ad12e314a706ae39ef3ba1805cdaea84`
- Hyperreal candidate: `22c3a445538c5bf6669d7a8bd0bc1bb04c414394`
- Hypermesh and evidence parent: `158f48915684d2bd5f05c0d1e31215fb6558bf06`
- Hyperlattice: `d11ca2f0e825d8e26048cfda5d1101df21dcfef0`
- Hyperlimit: `3e5d8816cd32bba46f48e0c6c13ab7a9da227784`
- Hypertri: `c47601266e0b9b17d0c5a0764fa22b18168ada73`

## Outcome

Hyperreal's exact finite-`f32`/`f64` decoder now reuses its existing bounded
canonical rational storage before allocating another `RationalData` and two
`BigUint` values. It retains the original exact allocation path outside that
bounded domain. No cache, dependency, public type, compatibility entry point,
policy branch, or fallback was added.

The strongest effect is the 6,144-triangle dense-box input, whose coordinates
are drawn heavily from the bounded canonical set. Across both Hyperlimit
policies, deterministic instructions fall 0.9533%, branches fall 1.5863%, and
production allocation calls fall 92.10%. Direct byte-level Massif peak falls
58.99%, useful heap falls 52.98%, and process RSS falls 19.79--21.35%.

The generated projective control moves only +0.0158% instructions and +0.0524%
branches across both policies; its reverse-order 13,452-triangle confirmation
remains below the one-percent clock gate. The retained arrangement removes
0.0481% instructions and 1.128% policy-paired cycles. These results retain the
candidate on the performance/memory Pareto frontier despite a 784-byte
path-specific `.text` increase. Every canonical native/WASM size-harness row
is unchanged.

## Implementation and bounded fallback

`Rational::from_reduced_dyadic_word` already receives an exact, nonzero,
reduced unsigned word numerator, sign, and power-of-two denominator shift from
IEEE-754 decoding. It now computes the retained dyadic facts once and probes
only the pre-existing canonical domains:

- denominator shift zero and magnitude one reuses `one()` or `minus_one()`;
- denominator shift zero and magnitude 2--64 reuses `small_integer`;
- denominator shift 1--63 and odd magnitude 1--63 reuses
  `small_reduced_dyadic`; and
- every other value follows the unchanged `BigUint`/`Arc` construction path.

The cache hit retains only monotonic facts already proved by decoding. The
bounded cache values are exactly representable as binary64, so the existing
`TryFrom<f64>` exact-view marker remains sound when the object is shared with
other exact constructors. Production source changes by 33 insertions and five
deletions; the focused boundary test adds 42 lines.

The focused test proves identity reuse for positive and negative one, integer
four, positive and negative `3/8`, and the shared `f32`/`f64` `3/8` value. It
also proves allocation-path fallthrough for integer 65, magnitude-65 dyadic
`65/2`, and denominator shift 64. Repeated values in each outside case remain
distinct allocations.

## Exactness and policy proof

This checkpoint changes representation identity only after exact IEEE-754
decomposition. A cache hit denotes the identical reduced rational number; it
does not decide a geometric comparison and it cannot turn an unknown result
into a certified result.

`STRICT` therefore continues to consume only structural, filtered, or exact
decisions. `APPROXIMATE_512` still differs only when Hyperlimit reaches its
terminal 512-bit equality/sign interpretation. Hypermesh receives the selected
policy through the same immutable `MeshContext`, and aggregate certainty is
unchanged. Both policies return `Certified` with identical topology on all
three large fixtures:

| Fixture | Input triangles | Output vertices | Output triangles | Result |
| --- | ---: | ---: | ---: | --- |
| Generated projective | 13,452 | 154 | 304 | `Certified` |
| Retained arrangement | 4,524 | 625 | 1,246 | `Certified` |
| Dense boxes | 6,144 | 27 | 50 | `Certified` |

The existing general construction path remains complete for larger
magnitudes, larger shifts, subnormals, and every finite decoded float outside
the bounded cache. Non-finite rejection is untouched.

## Serialized CPU A/B

The parent and candidate probes use equal release flags and the same temporary
operation-repetition hook. The hook was removed from the production tree after
measurement. Runs are serialized on CPU 9 in parent/candidate/candidate/parent
brackets. Task clock and cycles are directional; instructions and branches are
the retention gate.

Policy-paired deterministic work:

| Fixture | Repetitions | Instructions | Branches |
| --- | ---: | ---: | ---: |
| Generated 852 | 1,001 | +0.0158% | +0.0524% |
| Retained 4,524 | 201 | -0.0481% | +0.00002% |
| Dense boxes 6,144 | 10,001 | -0.9533% | -1.5863% |

Per-policy bracket means:

| Fixture / policy | Task clock | Cycles | Instructions | Branches |
| --- | ---: | ---: | ---: | ---: |
| Generated 852 / `STRICT` | -0.511% | -0.266% | +0.0143% | +0.0501% |
| Generated 852 / `APPROXIMATE_512` | outlier | outlier | +0.0173% | +0.0546% |
| Retained / `STRICT` | -2.675% | -2.438% | -0.0514% | -0.0046% |
| Retained / `APPROXIMATE_512` | +0.044% | +0.172% | -0.0448% | +0.0046% |
| Dense boxes / `STRICT` | +1.967% | +1.367% | -0.9498% | -1.5811% |
| Dense boxes / `APPROXIMATE_512` | -2.039% | -1.893% | -0.9568% | -1.5914% |

The approximate generated row contains one 7.097-second candidate sample; its
instruction and branch counts remain aligned with the other three samples.
Policy-paired retained task/cycles improve 1.311%/1.128%, while paired dense
cycles improve 0.250% and task clock is flat. This is consistent with less
deterministic work and with clock noise in the strict dense bracket.

The supplemental 13,452-triangle generated probe uses 501 repetitions for the
first bracket and 1,001 for a reverse strict confirmation:

| Bracket | Task clock | Cycles | Instructions | Branches |
| --- | ---: | ---: | ---: | ---: |
| Initial `STRICT` | +2.705% | +2.455% | +0.00818% | +0.03871% |
| Initial `APPROXIMATE_512` | +1.417% | +1.373% | +0.00938% | +0.04011% |
| Reverse `STRICT` | +0.823% | +0.759% | +0.01138% | +0.04211% |

The reverse bracket brings clocks below the checkpoint's one-percent gate,
while both orders show that deterministic work moves less than 0.05%.

## Large-fixture heap

Production, no-hook Heaptrack recordings include fixture construction and one
complete immediate union. The retained OBJ is
`yeahright_boolean_hull.obj`, SHA-256
`5f1ac8a2f8bf2b0c67ce95dc8146cc783155dc9c34eeee164f03446a0830886c`.

| Fixture / policy | Parent allocations | Candidate allocations | Movement | Parent peak | Candidate peak | Parent RSS | Candidate RSS |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated / `STRICT` | 200,743 | 200,642 | -0.050% | 7.50 MiB | 7.50 MiB | 18.35 MiB | 17.84 MiB |
| Generated / `APPROXIMATE_512` | 200,743 | 200,641 | -0.051% | 7.50 MiB | 7.50 MiB | 18.36 MiB | 17.81 MiB |
| Retained / `STRICT` | 453,990 | 453,854 | -0.030% | 11.67 MiB | 11.66 MiB | 21.51 MiB | 21.08 MiB |
| Retained / `APPROXIMATE_512` | 453,990 | 453,853 | -0.030% | 11.67 MiB | 11.66 MiB | 21.70 MiB | 21.03 MiB |
| Dense boxes / `STRICT` | 27,189 | 2,148 | -92.100% | 2.34 MiB | 1.14 MiB | 11.52 MiB | 9.06 MiB |
| Dense boxes / `APPROXIMATE_512` | 27,189 | 2,147 | -92.103% | 2.34 MiB | 1.14 MiB | 11.42 MiB | 9.16 MiB |

Direct byte-level Massif A/B uses equal-flag repeated probes:

| Fixture | Parent total | Candidate total | Movement | Parent useful | Candidate useful | Movement |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated | 8,245,672 B | 8,245,736 B | +64 B | 7,420,566 B | 7,420,638 B | +72 B |
| Retained | 12,698,576 B | 12,690,448 B | -8,128 B | 11,589,527 B | 11,582,671 B | -6,856 B |
| Dense boxes | 2,597,160 B | 1,064,992 B | -1,532,168 B | 2,262,518 B | 1,063,742 B | -1,198,776 B |

The 64-byte generated movement is 0.00078%. The dense-box total/useful
reductions are 58.994%/52.984%. The cache arrays already existed in static
storage; this change creates only values actually observed and reduces both
heap peak and whole-process RSS.

## Dispatch, linked size, and call graph

The generated dispatch trace retains zero unknown-fact and zero
fallback-or-abort events. The exact topology and predicate count are unchanged;
the canonical reuse removes 26 dispatch events, four cache events, and 19
counted rational temporaries:

| Counter | Parent | Candidate |
| --- | ---: | ---: |
| Dispatch | 97,347 | 97,321 |
| Predicates | 676 | 676 |
| Linear algebra | 1,411 | 1,411 |
| Cache | 6,345 | 6,341 |
| Rational temporaries | 12,794 | 12,775 |
| Unknown facts | 0 | 0 |
| Fallback or abort | 0 | 0 |

Canonical linked sections are unchanged from checkpoint 35:

| Consumer | Native text | Native aggregate | Optimized WASM |
| --- | ---: | ---: | ---: |
| General release | 4,064,708 B | 4,305,535 B | 2,740,169 B |
| Immediate release | 4,098,324 B | 4,338,303 B | 2,755,218 B |
| General size profile | 1,871,754 B | 2,114,124 B | 1,166,000 B |
| Immediate size profile | 1,884,222 B | 2,126,416 B | 1,176,378 B |

Those integer-input consumers do not retain the finite-float decoder. The
equal-flag repeated large-mesh probe does: its `.text` section grows from
3,816,902 to 3,817,686 bytes (+784 B, +0.0205%), GNU `size` text grows 896
bytes while BSS falls 896 bytes, aggregate remains 4,887,168 bytes, and file
size grows from 5,585,416 to 5,586,432 bytes (+1,016 B, +0.0182%).

The Hypermesh-only call graph is byte-identical to checkpoint 35 at 8,051
nodes / 19,877 edges, SHA-256
`efbebd2b6288e1bedefadc6082cac08d94d526383e28849e5d4c9908b570f617`.
The five-crate graph is 19,768 / 39,541, moving +3 / +13 for the constructor
branch and focused test, SHA-256
`81943958b7d104824d2cafdec511e5e8bfbfc34f33595fcd5393bb63d1d541df`.
No Hypermesh path, policy terminal, predicate, or fallback edge changes.

## Competitive and historical controls

Fresh CPU-9 Criterion centers retain the same competitive shape. These clocks
are context, not the direct A/B gate, because several competitor rows moved
materially in the same session.

| Fixture / operation | Hypermesh | Boolmesh | Manifold-rust | Hypermesh relation |
| --- | ---: | ---: | ---: | --- |
| Projective / union | 6.2912 ms | 769.28 us | 671.74 us | 8.18x / 9.37x slower |
| Projective / intersection | 4.7743 ms | 746.07 us | 660.81 us | 6.40x / 7.22x slower |
| Projective / difference | 4.2119 ms | 839.70 us | 676.28 us | 5.02x / 6.23x slower |
| Dense boxes / union | 704.35 us | 7.4792 ms | 4.8444 ms | 10.62x / 6.88x faster |
| Dense boxes / intersection | 506.28 us | 3.8772 ms | 3.4130 ms | 7.66x / 6.74x faster |
| Dense boxes / difference | 642.54 us | 6.1485 ms | 3.9795 ms | 9.57x / 6.19x faster |

Competitors remain throughput comparators, not exactness or policy oracles.

The direct retained strict bracket averages 34.281 ms per union. Against the
directional historical 944.8 ms row, that is 96.372% lower or 27.56x faster.
The current 11.66 MiB peak, 453,854 allocations, and 21.08 MiB strict RSS are
82.79%, 90.96%, and 74.45% below the historical 67.74 MiB, 5,020,891, and
82.5 MiB. Fixture and implementation evolution make these trend controls,
not a direct revision A/B.

## Rejected implementations

- Branchy Euclidean, subtract-Euclidean, unrolled, and hybrid word-GCD loops
  increased deterministic work and were removed.
- Projectively clearing dyadic affine-plane coefficient denominators increased
  13,452-triangle instructions about 8.05% and branches about 7.72%; it was
  removed.
- Carrying active limb counts through fixed-width GCD increased instructions
  about 0.56% and branches about 1.37%; it was removed.
- Packing cached crossing bounds for SSE2 improved one generated bracket but
  repeated a 1.36% strict dense-box cycle regression and slightly increased
  dense instructions/branches; it was removed.
- Reusing only fractional finite-float values gave up integer allocation
  savings and made the large generated deterministic counts slightly worse
  than the retained all-small form; it was removed.

No rejected implementation, diagnostic counter, temporary repetition hook, or
measurement-only source remains in a production tree.

## Validation

The retained implementation passes:

- Hyperreal default/minimal/all-feature full suites, with 563 / 563 / 640
  library tests, plus warning-denied Clippy/rustdoc and formatting in all and
  minimal configurations;
- Hyperlattice, Hyperlimit, Hypertri, and Hypermesh default, minimal, and
  all-feature full suites, warning-denied Clippy/rustdoc, and formatting;
- the focused Hyperreal cache/fallthrough test under AddressSanitizer and all
  1,063 default Hypermesh library tests under AddressSanitizer, with leak
  detection disabled;
- every Hypermesh fuzz-bin check, all-feature benchmark compilation, and
  default/all-feature `wasm32-unknown-unknown` library checks;
- both-policy large-fixture topology/certainty, six production Heaptrack runs,
  three direct Massif A/B rows, serialized CPU controls, all-family dispatch,
  canonical native/WASM size, and five-crate/per-library call graphs; and
- exact policy regressions proving certified work under both policies and
  terminal-consumption propagation only where Hyperlimit actually resolves an
  otherwise undecided result.

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
CARGO_TARGET_DIR=/tmp/hyperreal-float-import-asan \
RUSTFLAGS='-Zsanitizer=address' ASAN_OPTIONS=detect_leaks=0 \
cargo +nightly test --locked --target x86_64-unknown-linux-gnu \
  finite_float_imports_reuse_small_canonical_storage --lib
CARGO_TARGET_DIR=/tmp/hypermesh-float-import-asan \
RUSTFLAGS='-Zsanitizer=address' ASAN_OPTIONS=detect_leaks=0 \
cargo +nightly test --locked --target x86_64-unknown-linux-gnu --lib

# Hypermesh path/build surfaces
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo bench --locked --all-features --no-run
cargo check --locked --target wasm32-unknown-unknown --no-default-features --lib
cargo check --locked --target wasm32-unknown-unknown --all-features --lib
YEAHRIGHT_BENCH=1 cargo bench --locked --bench dispatch_trace \
  --features dispatch-trace
HYPERMESH_SIZE_TARGET_DIR=/tmp/hypermesh-float-import-size-default \
  ./benchmarks/size-harness/measure.sh default

# call graph
../tools/hyper-callgraph/target/release/hyper-callgraph \
  --root .. \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format json --per-library \
  --out-dir /tmp/hypermesh-float-import-callgraph
```

Hyperlimit's pre-existing untracked `hyperlimit` executable is untouched.
Machine-readable raw and derived values are in
`canonical-float-import-reuse-2026-08-02.toml`.
