# Fused exact product-sum sign planning

Date: 2026-08-02

Status: retained as Phase 7 checkpoint 40

Revisions:

- Hyperreal parent: `33e7e4d620c0c95620c1045d09d0089ded105030`
- Hyperreal implementation: `6302bbd848ad99cf192419c76e399f6e45cbdba3`
- Hypermesh implementation: `8d27c24a4a68caec9b3b72e1bdc0bf6143a6de4c`
- Hypermesh evidence parent: `4cbf48890f8a70e763d64bb6dd0e613ffe3dc1bb`
- Hyperlattice: `d11ca2f0e825d8e26048cfda5d1101df21dcfef0`
- Hyperlimit: `3e5d8816cd32bba46f48e0c6c13ab7a9da227784`
- Hypertri: `c47601266e0b9b17d0c5a0764fa22b18168ada73`

## Outcome

Hyperreal's common four- and six-product exact ordering schedules now compute
each product sign inside their existing one-pass dyadic word accumulator.
Successful probes no longer construct and rescan a separate sign array before
doing the exact aligned accumulation. If a probe declines, one outlined helper
reconstructs every sign from the original immutable inputs and enters the
unchanged zero-term, single-term, or arbitrary-width complete path.

This is a private schedule change: it adds no public API, dependency, heap
allocation, cache, compatibility shim, floating decision, policy branch, or
topology fallback. The implementation changes by 68 insertions and 22
deletions in one production file; 52 insertions update and extend exact tests.
All temporary repetition hooks and rejected experiments were removed before
the implementation commit.

Across both Hyperlimit policies, the equal-binary serialized A/B removes
0.620% of generated instructions and 0.491% of branches, while task clock and
cycles improve 0.646% and 0.500%. Dense task clock/cycles improve 1.751% and
1.548%; retained task clock/cycles improve 0.627% and 0.694%. Performance is
the retention priority. The production stripped probe grows 1,168 bytes of
`.text`; optimized WASM grows 57--460 bytes across the canonical matrix.

## Exactness and complete-path invariant

For every product term, `product_term_sign` multiplies the exact rational
factor signs and the caller's exact additive/subtractive sign. The fused loop
then performs the same work as the former preplanned loop:

- exact zero products are skipped;
- every nonzero factor must expose the same exact word-sized dyadic view;
- numerator multiplication, denominator alignment, shifts, and positive or
  negative accumulation remain checked `u128` operations;
- successful accumulation compares the exact positive and negative totals;
  and
- any unavailable dyadic view or checked overflow returns `None` without
  publishing partial state.

The declined path receives the original `positive_terms` and `terms` arrays,
not partial accumulator state. It recomputes the full exact sign array, handles
all-zero and single-nonzero schedules exactly, and calls the unchanged
arbitrary-width fallback for every other case. The four-term admission guard
and six-term admission domain are unchanged. Consequently every input that
formerly declined still reaches every former complete path.

The focused regression covers all-zero, single-negative, and mixed six-term
ordering. Existing randomized tests compare the one-pass accumulator against
the planned implementation across narrow successes and deliberate overflow or
representation declines. The full Hyperreal and Hypermesh suites exercise the
same scalar primitive through determinant, orientation, plane, subdivision,
trace, and output paths.

## STRICT and APPROXIMATE_512 policy proof

The changed functions use exact `Rational` signs and exact integer arithmetic
only. They do not call Hyperlimit, inspect `MeshContext`, consume certainty, or
produce approximate values. `STRICT` therefore still consumes only
structural, filtered, or exact decisions. `APPROXIMATE_512` still differs only
when Hyperlimit reaches its terminal 512-bit equality/sign interpretation.

Both policies return `Certified` with identical output topology:

| Fixture | Input triangles | Output vertices | Output triangles | STRICT | APPROXIMATE_512 |
| --- | ---: | ---: | ---: | --- | --- |
| Generated projective | 13,452 | 154 | 304 | `Certified` | `Certified` |
| Retained arrangement | 4,524 | 625 | 1,246 | `Certified` | `Certified` |
| Dense boxes | 6,144 | 27 | 50 | `Certified` | `Certified` |

The generated dispatch trace is unchanged at 97,321 events, including 676
predicates, 1,411 linear-algebra events, 6,341 cache events, and 12,775
rational temporaries. It records zero unknown facts and zero fallback/abort
events under both policies.

## Serialized CPU A/B

The checkpoint-39 parent and checkpoint-40 candidate use equal native release
flags and the same temporary operation-repetition hook. Runs are serialized on
CPU 9. STRICT uses parent/candidate/candidate/parent brackets. Generated
APPROXIMATE_512 combines that order with a reverse
candidate/parent/parent/candidate bracket to suppress order bias. Every run
produces the certified topology above. Percentages are candidate relative to
parent; negative is better.

| Fixture / policy | Repetitions | Task clock | Cycles | Instructions | Branches | Branch misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated / `STRICT` | 1,001 | -0.854% | -0.632% | -0.6190% | -0.4897% | +0.25% |
| Generated / `APPROXIMATE_512` | 1,001 | -0.437% | -0.368% | -0.6209% | -0.4918% | -0.22% |
| Retained / `STRICT` | 501 | +0.035% | -0.071% | -0.0389% | +0.0021% | -1.08% |
| Retained / `APPROXIMATE_512` | 501 | -1.289% | -1.317% | -0.0313% | +0.0107% | +0.08% |
| Dense boxes / `STRICT` | 10,001 | -1.354% | -1.148% | -0.1656% | -0.1013% | +3.75% |
| Dense boxes / `APPROXIMATE_512` | 10,001 | -2.148% | -1.948% | -0.1770% | -0.1179% | +0.83% |

Arithmetic means across the two policies:

| Fixture | Task clock | Cycles | Instructions | Branches |
| --- | ---: | ---: | ---: | ---: |
| Generated 13,452 | -0.646% | -0.500% | -0.620% | -0.491% |
| Retained 4,524 | -0.627% | -0.694% | -0.035% | +0.006% |
| Dense boxes 6,144 | -1.751% | -1.548% | -0.171% | -0.110% |

The generated STRICT parent/candidate means are 8,546.950 / 8,473.965 ms,
36.1550 / 35.9265 billion cycles, 100.9090 / 100.2844 billion instructions,
and 17.4006 / 17.3154 billion branches. The four-sample generated approximate
means are 8,546.595 / 8,509.220 ms and 36.1216 / 35.9885 billion cycles.

Dense STRICT parent/candidate means are 6,953.625 / 6,859.495 ms and 29.4040 /
29.0664 billion cycles. Retained STRICT's clock is neutral at 17,112.235 /
17,118.270 ms while its deterministic work improves; the opposite-policy
bracket and the policy mean show the retained runtime direction. Branch-miss
movement is noisy and does not override the instruction, branch, and cycle
gates.

## Profile movement

Equal-flag 1,001-operation CPU-9 cycle profiles record 17,436 parent and
17,013 candidate samples with zero loss. The candidate's principal sampled
heads are projective input-soup construction at 5.69%, four-by-two exact
product sums at 4.75%, six-by-two exact product sums at 4.30%,
`Rational::to_f64_lossy` at 3.07%, allocator internals at 2.91%, word GCD at
2.87%, crossing-event splitting at 2.75%, fixed GCD at 2.61%, and computed
input soup at 2.59%. Sampling is explanatory; deterministic counters are the
retention gate.

## Large-fixture heap

Production, no-hook Heaptrack recordings include fixture construction and one
complete immediate union. The retained OBJ is `yeahright_boolean_hull.obj`,
SHA-256
`5f1ac8a2f8bf2b0c67ce95dc8146cc783155dc9c34eeee164f03446a0830886c`.

| Fixture / policy | Parent allocations | Candidate allocations | Parent peak | Candidate peak | Parent RSS | Candidate RSS |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated / `STRICT` | 200,644 | 200,643 | 7.50 MiB | 7.50 MiB | 17.81 MiB | 17.79 MiB |
| Generated / `APPROXIMATE_512` | 200,644 | 200,643 | 7.50 MiB | 7.50 MiB | 17.77 MiB | 17.83 MiB |
| Retained / `STRICT` | 453,856 | 453,855 | 11.66 MiB | 11.66 MiB | 20.99 MiB | 20.98 MiB |
| Retained / `APPROXIMATE_512` | 453,856 | 453,855 | 11.66 MiB | 11.66 MiB | 20.99 MiB | 21.05 MiB |
| Dense boxes / `STRICT` | 2,148 | 2,147 | 1.14 MiB | 1.14 MiB | 9.05 MiB | 9.03 MiB |
| Dense boxes / `APPROXIMATE_512` | 2,148 | 2,147 | 1.14 MiB | 1.14 MiB | 9.06 MiB | 9.01 MiB |

The scalar change adds no allocation. The uniform one-call movement and small
RSS variation are process/layout noise; rounded Heaptrack peaks are unchanged.
Direct byte-level Massif A/B is likewise flat:

| Fixture | Parent total | Candidate total | Movement | Parent useful | Candidate useful |
| --- | ---: | ---: | ---: | ---: | ---: |
| Generated | 8,245,720 B | 8,245,736 B | +16 B | 7,420,614 B | 7,420,614 B |
| Retained | 12,691,568 B | 12,690,608 B | -960 B | 11,582,695 B | 11,582,335 B |
| Dense boxes | 1,065,008 B | 1,064,976 B | -32 B | 1,063,718 B | 1,063,718 B |

These measurements use all three large fixtures, including the retained
4,524-triangle arrangement requested as the heap gate.

## Linked size and call graph

Stripped production-probe movement:

| Measure | Parent | Candidate | Movement |
| --- | ---: | ---: | ---: |
| `.text` section | 3,817,766 B | 3,818,934 B | +1,168 B |
| GNU text | 4,634,485 B | 4,635,873 B | +1,388 B |
| GNU data | 247,512 B | 247,528 B | +16 B |
| GNU aggregate | 4,883,056 B | 4,887,132 B | +4,076 B |
| Stripped file | 4,885,880 B | 4,887,304 B | +1,424 B |

The aggregate movements near 4 KiB are section-alignment/layout changes; the
actual ELF `.bss` remains 227 bytes. Canonical consumer deltas:

| Features / consumer | Native text | Native aggregate | File | Optimized WASM |
| --- | ---: | ---: | ---: | ---: |
| Default general release | +1,480 B | +4,104 B | +1,816 B | +460 B |
| Default immediate release | +1,480 B | +8 B | +1,816 B | +399 B |
| Default general size | +416 B | 0 B | +464 B | +78 B |
| Default immediate size | +400 B | 0 B | +448 B | +79 B |
| All-feature general release | +1,312 B | +4,096 B | +1,784 B | +418 B |
| All-feature immediate release | +1,320 B | +8 B | +1,792 B | +331 B |
| All-feature general size | +416 B | 0 B | +464 B | +57 B |
| All-feature immediate size | +400 B | +4,096 B | +448 B | +57 B |

The runtime win is retained over this bounded code-size cost. No duplicate
implementation or compatibility surface was added: the common hot schedule
owns the fused loop and one outlined helper owns all declined cases.

The five-crate graph moves from 19,775 nodes / 39,556 edges to 19,778 /
39,568, SHA-256
`c1f42f93eca02d9d8e951bffc46537cd99d7140ff6984a4a56ed01b1a844a65c`.
Hyperreal moves from 7,153 / 12,275 to 7,156 / 12,287, SHA-256
`a465eff2e8af569c8825f7031025ff97a3c7e58941224c49db93925a513f1262`.
Hyperlattice, Hyperlimit, Hypertri, and Hypermesh per-library graphs are
byte-identical.

The three added nodes are the declined helper, its `Self` alias, and the
focused test. The twelve added edges connect the hot accumulator to exact sign
evaluation, the ordering entry to the declined helper, and that helper/test to
the already-existing exact sign and fallback operations. No policy terminal,
predicate family, mesh operation, or arbitrary-width fallback edge is removed.

## Competitive and historical controls

Fresh final-source Criterion rows were pinned to CPU 9. Projective competitors
remain stable enough to orient the comparison. Several dense competitor rows
shifted materially between sessions, so cross-session Criterion is not used as
the revision retention gate; the serialized same-binary counters above are.

| Fixture / operation | Hypermesh | Boolmesh | Manifold-rust | Hypermesh relation |
| --- | ---: | ---: | ---: | --- |
| Projective / union | 6.1469 ms | 758.52 us | 666.39 us | 8.10x / 9.22x slower |
| Projective / intersection | 4.4874 ms | 744.03 us | 665.45 us | 6.03x / 6.74x slower |
| Projective / difference | 4.0703 ms | 756.97 us | 667.90 us | 5.38x / 6.09x slower |
| Dense boxes / union | 711.94 us | 6.7685 ms | 4.3120 ms | 9.51x / 6.06x faster |
| Dense boxes / intersection | 503.18 us | 3.8837 ms | 3.4970 ms | 7.72x / 6.95x faster |
| Dense boxes / difference | 645.10 us | 6.1781 ms | 3.9831 ms | 9.58x / 6.17x faster |

Against checkpoint 39's stored Hypermesh centers, projective
union/intersection/difference improve 2.66%/1.04%/1.82%, while dense
union/intersection/difference move -2.35%/-2.26%/-0.03%. Criterion's live
paired estimates classify projective union (-3.82%) and dense intersection
(-2.19%) as improvements; the remaining rows are neutral or within its noise
threshold.

The retained STRICT bracket averages 34.1682 ms per union. Against the
directional historical 944.8 ms row, that is 96.384% lower or 27.65x faster.
The candidate's 11.66 MiB peak, 453,855 allocations, and 20.98 MiB strict RSS
are 82.79%, 90.96%, and 74.57% below the historical 67.74 MiB, 5,020,891, and
82.5 MiB. Fixture and implementation evolution make these historical trend
controls, not direct revision A/B.

## Rejected implementations

- An exact early-zero branch in `Rational::to_f64_lossy` saved 48 binary bytes
  and about 0.10% instructions in the hooked probe, but regressed generated
  STRICT task clock 1.30% and approximate clock 0.29%.
- Projective-adjacency parity head selection reduced visits but increased
  branch misses 2.3--2.5% and lost clock. Borrowed edge records and field
  reordering were neutral-to-slower.
- Fully inlining both the fused hot path and its complete declined path removed
  deterministic work but lost clock to layout growth.
- Outlining the accumulator added 1.77% instructions. Six-term-only and
  four-term-only shapes added 4.6% and 2.37% total instructions.
- Marking the declined helper `#[cold]` worsened generated clock. The retained
  helper is only `#[inline(never)]`.

Every rejected source variant, diagnostic counter, and repetition hook was
removed.

## Validation

The retained source passed:

- Hyperreal default/minimal/all-feature suites (565 / 565 / 642 library tests
  plus every integration and doc test);
- Hypermesh default/minimal/all-feature suites (1,063 / 1,063 / 1,064 library
  tests plus every integration and doc test);
- Hyperlattice, Hyperlimit, and Hypertri full default, minimal, and all-feature
  suites;
- warning-denied all-target Clippy, warning-denied minimal/all-feature rustdoc,
  and format checks for all five crates;
- the focused Hyperreal test under nightly ASAN and all 1,063 Hypermesh library
  tests under nightly ASAN, with leak detection disabled;
- every Hypermesh fuzz-bin check, all-feature benchmark compilation, and
  minimal/all-feature `wasm32-unknown-unknown` library checks;
- both-policy dispatch tracing and the five bounded ignored competitive release
  gates for input validity, every Boolean operation, immediate/polygon parity,
  large-fixture memory pressure, and bounded full-resolution validation; and
- diff whitespace checks plus randomized exact accumulator/fallback coverage.

The known approximately 56-minute
`full_resolution_yeahright_rotated_intersection_certifies_empty` control was
not rerun. This scalar schedule changes no mesh operation or fallback graph,
and its bounded full-resolution gate passed.

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

# sanitizer and Hypermesh build surfaces
RUSTFLAGS='-Zsanitizer=address' ASAN_OPTIONS=detect_leaks=0 \
cargo +nightly test --locked --target x86_64-unknown-linux-gnu --lib
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo bench --locked --all-features --no-run
cargo check --locked --target wasm32-unknown-unknown --no-default-features --lib
cargo check --locked --target wasm32-unknown-unknown --all-features --lib
YEAHRIGHT_BENCH=1 cargo bench --locked --bench dispatch_trace \
  --features dispatch-trace

# competitive and linked-size controls
YEAHRIGHT_BENCH=1 taskset -c 9 cargo bench --locked --bench competitive -- \
  yeahright_control_hull_subdivided_box
taskset -c 9 cargo bench --locked --bench competitive -- \
  subdivided_overlapping_boxes_3072_each
HYPERMESH_SIZE_TARGET_DIR=/tmp/hypermesh-fused-sign-size-default \
  ./benchmarks/size-harness/measure.sh default
HYPERMESH_SIZE_TARGET_DIR=/tmp/hypermesh-fused-sign-size-all \
  ./benchmarks/size-harness/measure.sh all

../tools/hyper-callgraph/target/release/hyper-callgraph \
  --root .. \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format json --per-library \
  --out-dir /tmp/hypermesh-fused-sign-callgraph
```

Hyperlimit's pre-existing untracked `hyperlimit` executable is untouched.
Machine-readable raw and derived values are in
`fused-product-sum-sign-planning-2026-08-02.toml`.
