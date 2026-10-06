# Hypermesh unsigned Lehmer-matrix application checkpoint

Date: 2026-07-31

Direct Hypermesh parent:
`8c9575f533f64f14ae4cf607a7d476df737a6e32`

Implementations:

- Hyperreal parent `e365a0153836393954d85a9e8988d9539b2068b0`
- Hyperreal candidate `d10a01f1b4a6ec202dff6c0fc2a726baa13cc841`
- Hyperlimit `3e5d8816cd32bba46f48e0c6c13ab7a9da227784`
- Hypermesh `8c9575f533f64f14ae4cf607a7d476df737a6e32`

Hypermesh production source is unchanged in this checkpoint.

## Outcome

Lehmer's leading-limb reducer batches exact Euclidean steps in a signed 2-by-2
matrix whose coefficients are bounded to one machine word. The prior matrix
application cloned both arbitrary-width magnitudes into `BigInt`, performed
four signed products and two additions, then cloned both result magnitudes back
to `BigUint`.

The canonical application path now stays in `BigUint`. It multiplies each wide
operand by the unsigned magnitude of its one-word coefficient, adds products
when coefficient signs agree, and takes their absolute difference when signs
differ. The existing progress check and full-width remainder fallback are
unchanged.

The final change removes 938 allocation calls from the retained 4,524-triangle
mesh and 42 from the generated 13,452-triangle mesh without changing their
12.69/11.66 MiB peak heaps. It substantially improves balanced scalar GCD rows
at 512 bits and above. The owning retained mesh is mixed but within noise:
strict task clock/cycles improve 1.18/1.21%, while approximate task
clock/cycles move +0.07/+0.20%; both execute fewer instructions and branches.

Six of eight linked-code rows shrink by 104-136 bytes. Release WASM grows only
52 bytes (0.0019%) in each consumer. No public API, retained carrier,
compatibility path, or production function is added.

## Exactness and policy contract

For nonnegative wide magnitudes `L` and `S` and signed coefficients `x` and
`y`, the result magnitude is exactly

```text
|xL + yS| = |x|L + |y|S                    when sign(x) = sign(y)
|xL + yS| = abs(|x|L - |y|S)               otherwise
```

This identity also covers a zero coefficient. `lehmer_gcd_matrix` already
proves that every retained coefficient magnitude fits `u64`; application uses
checked conversion anyway and returns `None` to the unchanged exact remainder
fallback if that private producer contract is ever violated. `i128::MIN` is
handled by `unsigned_abs` and rejected rather than overflowing.

The matrix and progress conditions are unchanged, so every accepted result is
the same magnitude that the signed implementation produced. A 625-matrix test
covers every combination of coefficients in `{-3,-1,0,1,3}` for both rows
against the removed signed implementation, plus `i128::MIN`/`MAX` rejection.
The existing randomized corpus compares selected GCD with
`num::Integer::gcd` on balanced and initially unbalanced values from the
191-bit crossover boundary through 4,096 bits.

This is policy-independent exact scalar arithmetic:

- `STRICT` never consumes an approximate decision;
- `APPROXIMATE_512` retains the same terminal 512-bit predicate boundary;
- matrix application cannot alter `Certainty` or escalation;
- canonical rational normalization is unchanged; and
- every measured mesh output remains certified and identical between policies.

## Scalar A/B

Criterion's named parent baseline and candidate were pinned to CPU 9. The
unchanged 128-bit word row moved -3.12% across the two serial runs, exposing a
frequency/background shift. Raw movements are retained below; the large
balanced improvements greatly exceed that control, while small dynamic-entry
movements do not and are not overstated.

| Selected GCD row | Signed parent | Unsigned candidate | Raw movement |
| --- | ---: | ---: | ---: |
| 128-bit word control | 136.686 ns | 132.429 ns | -3.12% |
| 192-bit balanced | 5.533 us | 5.370 us | -2.94% |
| 512-bit balanced | 20.017 us | 11.227 us | -43.91% |
| 1,024-bit balanced | 33.162 us | 21.747 us | -34.42% |
| 4,096-bit balanced | 200.289 us | 118.069 us | -41.05% |
| Unbalanced to 192-bit Lehmer | 8.632 us | 8.464 us | -1.94% |
| Unbalanced to 256-bit Lehmer | 8.438 us | 8.316 us | -1.45% |
| Unbalanced to 512-bit Lehmer | 12.994 us | 12.293 us | -5.40% |
| Unbalanced to 1,024-bit Lehmer | 25.687 us | 24.132 us | -6.05% |
| Unbalanced to 4,096-bit Lehmer | 133.007 us | 129.132 us | -2.91% |

The 512/1,024-bit dynamic-entry rows improve about 2.3/3.0 percentage points
beyond the unchanged control's cross-run movement. Balanced wide rows show the
largest benefit because they apply more Lehmer batches before reaching the word
tail.

## Direct-parent Hypermesh CPU results

Parent and candidate release probes use the same fixture and CPU 9. The final
committed retained rows use 201 repetitions. Generated rows use 101 and box
controls use 401. Each candidate cell includes movement from its direct parent.

| Fixture / policy | Task clock | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Retained / `STRICT` | 55.98 ms (-1.183%) | 206,674,465 (-1.210%) | 544,226,094 (-0.015%) | 92,498,793 (-0.019%) | 865,013 (+0.348%) | 1,413,660 (+1.162%) |
| Retained / `APPROXIMATE_512` | 55.48 ms (+0.072%) | 205,979,349 (+0.196%) | 544,225,309 (-0.017%) | 92,498,557 (-0.022%) | 865,496 (+0.746%) | 1,404,870 (+2.800%) |
| Generated 13,452-t / `STRICT` | 76.26 ms (-0.574%) | 267,413,040 (-0.489%) | 598,919,583 (-0.0004%) | 90,165,709 (-0.0026%) | 750,437 (+0.021%) | 1,823,605 (-1.537%) |
| Generated 13,452-t / `APPROXIMATE_512` | 76.44 ms (-1.100%) | 268,034,163 (-0.752%) | 598,895,859 (-0.0032%) | 90,159,918 (-0.0070%) | 750,186 (+0.028%) | 1,815,704 (-2.515%) |
| 6,144-t boxes / `STRICT` | 5.87 ms (-0.677%) | 14,288,773 (-0.284%) | 35,383,467 (+0.0044%) | 6,485,666 (+0.0081%) | 65,456 (+0.666%) | 119,530 (+3.889%) |
| 6,144-t boxes / `APPROXIMATE_512` | 5.92 ms (-0.337%) | 14,430,783 (-0.351%) | 35,383,840 (+0.0055%) | 6,485,754 (+0.0086%) | 65,578 (+0.836%) | 119,751 (+4.943%) |

The retained approximate time/cycle movement and box instruction/cache
movement are small linked-layout effects. They do not create a repeatable
task-clock regression: five of six task-clock rows improve, and the remaining
row moves +0.04 ms. Performance is the primary objective, so the large scalar
gains and lower owning-mesh allocation count outweigh those small control
movements.

Output topology is identical for parent, candidate, `STRICT`, and
`APPROXIMATE_512`:

- retained: 4,524 input triangles, 625 vertices / 1,246 triangles;
- generated: 13,452 input triangles, 154 / 304; and
- boxes: 6,144 input triangles, 27 / 50.

All six outcomes report `MeshCertainty::Certified`.

## Large-fixture heap

Heaptrack includes fixture construction and the complete immediate union.
Candidate counts are identical between policies. A direct rerun reproduced the
stored parent totals within one recorder-startup allocation, so movement uses
the committed parent ledger.

| Fixture / revision | Allocations | Heaptrack temporary classification | Peak heap | Candidate Heaptrack RSS |
| --- | ---: | ---: | ---: | ---: |
| Retained parent | 521,086 | 27,181 | 12.69 MiB | - |
| Retained candidate | 520,148 (-0.180%) | 27,637 (+1.678%) | 12.69 MiB | 22.18-22.24 MiB |
| Generated parent | 215,113 | 10,300 | 11.66 MiB | - |
| Generated candidate | 215,071 (-0.020%) | 10,317 (+0.165%) | 11.66 MiB | 23.28-23.41 MiB |
| Boxes parent | 27,211 | 79 | 4.70 MiB | - |
| Boxes candidate | 27,211 (unchanged) | 79 (unchanged) | 4.70 MiB | 10.33-11.68 MiB |

The temporary classification increases slightly because unsigned products are
destroyed/reused in a different order. Total allocation calls fall and peak
live heap is unchanged, which are the authoritative memory gates. No retained
allocation or field is introduced.

## Historical and competitive controls

The frozen retained historical row was 944.8 ms, 67.74 MiB peak heap,
5,020,891 allocations, and 82.5 MiB maximum RSS. The current strict row is
directionally 94.07% faster, retains 81.27% less peak heap, and performs 89.64%
fewer allocations. Historical polygon output differed, so this remains a
directional regression anchor rather than a direct correctness A/B.

The immediately preceding same-day competitive run remains the comparison
control; competitors do not provide Hypermesh's exact `Real`, selected
terminal policy, or certified-output contract.

| Union workload | Hypermesh | boolmesh | manifold-rust | Relative result |
| --- | ---: | ---: | ---: | --- |
| Overlapping 12-triangle boxes | 5.0998 us | 67.195 us | 60.512 us | Hypermesh 13.18x / 11.87x faster |
| 3,072-triangle boxes per operand | 1.8719 ms | 7.6133 ms | 4.4687 ms | Hypermesh 4.07x / 2.39x faster |
| Dyadic YeahRight hull + box | 7.8718 ms | 0.76936 ms | 0.84057 ms | boolmesh 10.23x / manifold-rust 9.36x faster |

The projective competitor gap remains the main competitive Phase 7 target.
This scalar change narrows normalization overhead but does not substitute a
different topology algorithm or weaken exactness.

## Full-resolution gate

The prior checkpoint corrected the exact oracle and completed the
11,894-by-11,894 rotated intersection as certified empty in 3,357.09 seconds
with 319.07 MiB maximum RSS on its conservative 512-bit dynamic-entry
candidate. The user stopped a redundant rerun, and this checkpoint does not
restart the approximately 56-minute test. No final-source full-resolution time
or memory is claimed here. Exact signed-reference equivalence, scalar crossover
coverage, both-policy retained/generated/box probes, and all five crate test
matrices validate the direct-unsigned path at practical resolution.

## Native and WASM linked code

Native code is `.text`; WASM code is `wasm-opt -Oz`. Clean default-feature
consumers compare Hyperreal `e365a015` with `d10a01f`.

| Consumer | Profile | Parent | Candidate | Movement |
| --- | --- | ---: | ---: | ---: |
| General native | Release | 4,032,340 | 4,032,236 | -104 / -0.0026% |
| General WASM | Release | 2,705,575 | 2,705,627 | +52 / +0.0019% |
| Immediate native | Release | 4,065,940 | 4,065,836 | -104 / -0.0026% |
| Immediate WASM | Release | 2,720,613 | 2,720,665 | +52 / +0.0019% |
| General native | Size | 1,850,730 | 1,850,610 | -120 / -0.0065% |
| General WASM | Size | 1,147,862 | 1,147,727 | -135 / -0.0118% |
| Immediate native | Size | 1,863,238 | 1,863,102 | -136 / -0.0073% |
| Immediate WASM | Size | 1,158,824 | 1,158,689 | -135 / -0.0117% |

Six rows shrink; the largest growth is 52 bytes. The candidate is therefore
also on the linked-size frontier.

## Source and call graph

Hyperreal production changes by +18/-6 lines, including the exact-invariant
comment. Tests add 49 lines; the benchmark ledger replaces ten result rows.
There is no new public or production function and no second GCD implementation.

| Scope | Direct parent | Candidate | Movement |
| --- | ---: | ---: | ---: |
| Isolated Hypermesh | 7,982 nodes / 19,609 edges | 7,982 / 19,609 | unchanged |
| Five Hyper crates | 19,582 nodes / 39,110 edges | 19,591 / 39,118 | +9 / +8 |

The graph utility is syntactic and counts the local hot closure, exhaustive
test loops, and signed-reference helper. Production ownership remains the one
private canonical Lehmer application path.

## Rejected source-shortening variant

An in-place fallthrough form saved three source lines. On the retained probe it
raised instructions from 544,166,586 to 544,215,743 (+0.0090%) and branches
from 92,491,597 to 92,497,532 (+0.0064%). Task clock and cycles moved within
noise. Because performance has priority over source length, that variant was
fully removed and the lower-instruction direct-return form was restored.

## Validation

Hyperreal passes default, no-default, and all-feature matrices (555, 555, and
632 unit tests plus integrations and doctests), warning-denied full/minimal
Clippy, warning-denied documentation, formatting, every fuzz binary check, and
benchmark compilation. Nightly AddressSanitizer passes the exhaustive unsigned
matrix test, randomized selected-GCD equivalence, and half-GCD/Lehmer reference
test. Leak detection is disabled for the ptrace sandbox. The first fresh ASan
target hit `/tmp` quota; only that incomplete 349 MiB build directory was
removed, and the successful run reused the existing instrumented cache.

Hypermesh passes default, no-default, and all-feature matrices (1,052, 1,052,
and 1,053 unit tests plus integrations and policy suites), warning-denied
full/minimal Clippy, warning-denied documentation, formatting, every fuzz
binary check, and benchmark compilation. Hyperlattice, Hyperlimit, and Hypertri
also pass their default, no-default, and all-feature test matrices against the
committed Hyperreal candidate.

```text
# each of hyperreal, hyperlattice, hyperlimit, hypertri, and hypermesh
cargo test --locked --no-fail-fast
cargo test --locked --no-default-features --no-fail-fast
cargo test --locked --all-features --no-fail-fast

# hyperreal and hypermesh build/lint surfaces
cargo clippy --locked --all-targets --all-features -- -D warnings
cargo clippy --locked --all-targets --no-default-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features
cargo fmt --all -- --check
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo bench --locked --no-run

# focused Hyperreal sanitizer
CARGO_TARGET_DIR=/tmp/hyperreal-dynamic-lehmer-asan \
  RUSTFLAGS='-Zsanitizer=address' ASAN_OPTIONS=detect_leaks=0 \
  cargo +nightly test --locked --target x86_64-unknown-linux-gnu lehmer --lib

# evidence surfaces
cargo build --locked --release --example large_mesh_heap_probe
heaptrack --record-only target/release/examples/large_mesh_heap_probe \
  boxes-3072 strict
./benchmarks/size-harness/measure.sh default
../tools/hyper-callgraph/target/release/hyper-callgraph \
  --root .. \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --out-dir /tmp/hypermesh-unsigned-matrix-callgraph-five
```
