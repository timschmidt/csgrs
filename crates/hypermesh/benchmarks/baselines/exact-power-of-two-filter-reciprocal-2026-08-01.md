# Exact power-of-two filter reciprocal — 2026-08-01

This is Phase 7 checkpoint 30 of the workspace Hypermesh path-completeness
plan. The retained implementation is Hyperreal
`f6071aa430ce6e8fb10ed7c37e3b821d7c5b9d50`, based on checkpoint 29's
Hyperreal `aa4b76b955ecdf46d133d91ef215b9f0c985c1d2` and Hypermesh evidence
revision `85edb3a56f4d58108670dd0bad7cb0ba89d90c09`. Hypermesh production
remains at `5c48a413d1895ccd8e5bc5d56a06f127d2ce82f6`; Hyperlattice, Hyperlimit,
and Hypertri remain at `d11ca2f0e825d8e26048cfda5d1101df21dcfef0`,
`3e5d8816cd32bba46f48e0c6c13ab7a9da227784`, and
`c47601266e0b9b17d0c5a0764fa22b18168ada73`.

## Outcome

The exact-rational four-term floating filter now constructs the reciprocal of
its binary64 power-of-two normalization scale from the biased exponent bits.
It no longer executes a scalar floating-point division. The production change
is four insertions and one deletion in one private function; the exhaustive
differential regression is 72 test-only lines.

The selected generated-projective hot path improves policy-paired task clock
0.126% and cycles 0.326%. `STRICT` improves 0.479% task / 0.485% cycles and
`APPROXIMATE_512` improves 0.167% cycles despite a noisy 0.230% task-clock
movement. The function's sampled self ownership falls from 2.62% to 2.24%.
The generated instruction count grows only 0.0166% because the integer bit
construction retires more instructions than one `divsd`, while completing in
fewer cycles.

Retained-arrangement and dense-box controls are neutral. Dense-box
policy-paired instructions and branches improve 0.0049% and 0.0034%; its
0.28--0.31% clock/cycle movement is below the noise seen in policy-opposed
rows. The retained approximate clock bracket was interrupted twice, but its
deterministic instruction and branch totals remain within 0.0008% and 0.0038%.

There is no allocation, cache, topology, public API, dependency, policy,
terminal, or compatibility-shim change. Large-fixture allocation streams and
exact Valgrind Massif maxima are identical. Equal-layout linked sections are
identical, and the five-crate source call graph is byte-for-byte identical.

## Exact invariant and complete input paths

`normalize_rational_linear_form4_values` first selects the greatest absolute
binary64 bit pattern and rejects a zero, subnormal, infinity, or NaN scale.
Every accepted scale is therefore exactly `2^(E - 1023)` for biased exponent
`E` in `1..=2046`.

- For `E` in `1..=2045`, the reciprocal is normal and has biased exponent
  `2046 - E`.
- For `E == 2046`, the exact reciprocal is the representable subnormal
  `2^-1023`, whose binary64 bits are `1 << 51`.
- The constructed value is positive and bit-identical to the former
  `1.0 / scale` for every accepted exponent.
- The zero fast return, rejected subnormal/non-finite scales, per-lane signed
  zero handling, multiplication, and safe-minimum rejection are unchanged.
- A scaled lane that would be nonzero subnormal still rejects the filter and
  falls through to exact rational evaluation. No rounded value certifies a
  sign outside the existing error proof.

The new differential test enumerates all 2,046 normal exponents, exponent
spans around the 500-bit safety boundary and its extremes, four significand
patterns, both signs, positive and negative zero, infinity, NaN, minimum and
maximum subnormals, minimum normal, and maximum normal. It compares the new
implementation with a local copy of the former division implementation.
Existing randomized and boundary tests continue to compare the filter result
with the materialized exact rational sum.

## Policy and topology behavior

This Hyperreal filter has no policy terminal. A successful result is a
certified sign under the existing 82-epsilon error radius; `None` continues to
fall through to exact rational arithmetic. Hypermesh still routes every
topology decision through its immutable `MeshContext` and Hyperlimit policy:

- `STRICT` cannot consume a terminal approximation; and
- `APPROXIMATE_512` may terminate only at Hyperlimit's final 512-bit
  equality/sign interpretation.

Release tests cover union, intersection, difference, symmetric difference,
exact closure, exact triangle nondegeneracy, and polygon/immediate agreement
under both policies. Both policies remain `Certified` and produce exactly the
same meshes: 154 vertices / 304 triangles generated, 625 / 1,246 retained,
and 27 / 50 dense boxes.

The generated dispatch trace is unchanged: 97,131 dispatch events, 676
predicate events, zero unknown facts, zero fallback/abort events, and 12,794
rational temporaries. It still reaches direct word, planned stack,
arbitrary-precision dyadic, equal-denominator, and LCM exact ordering paths.

## Serialized CPU work

Checkpoint-29 and checkpoint-30 executables use equal-layout five-crate source
worktrees, CPU 9, fixture-once construction, complete immediate unions, and
parent/candidate/candidate/parent process order. Every row is certified and
produces the exact output counts above.

| Fixture / policy | Repetitions | Task | Cycles | Instructions | Branches | Branch misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated / `STRICT` | 1,501 | -0.479% | -0.485% | +0.0168% | +0.0686% | +0.197% |
| Generated / `APPROXIMATE_512` | 1,501 | +0.230% | -0.167% | +0.0164% | +0.0675% | -0.253% |
| Retained / `STRICT` | 401 | +0.139% | +0.335% | -0.0001% | +0.0037% | +1.645% |
| Retained / `APPROXIMATE_512` | 401 | interrupted | interrupted | +0.0008% | +0.0038% | noisy |
| Dense boxes / `STRICT` | 20,001 | +0.062% | -0.026% | -0.0281% | -0.0389% | +0.170% |
| Dense boxes / `APPROXIMATE_512` | 20,001 | +0.559% | +0.580% | +0.0183% | +0.0322% | +2.154% |

Policy-paired generated movement is -0.126% task / -0.326% cycles. Dense-box
movement is +0.311% / +0.277%, while deterministic instructions and branches
move -0.0049% / -0.0034%. Branch misses and the policy-opposed clocks are too
noisy to interpret. The generated fixture is the measured owner of this
normalizer and is the retention gate; the other fixtures show that the change
does not materially tax paths that exercise it less often.

## Profile and machine code

The final CPU-9 frame-pointer profile covers 501 strict generated unions,
19,386 samples, zero lost samples, and approximately 19.386 billion cycles.
Exact filter normalization falls from checkpoint 29's 2.62% self ownership to
2.24%, a 0.38 percentage-point or roughly 14.5% sampled reduction.

Other leading self owners are the four-by-two product-sum dispatcher 5.02%,
compact projective input construction 4.63%, lossy rational export 4.28%,
crossing-event splitting 3.71%, the six-by-two dispatcher 3.70%, mixed-width
GCD 2.98%, and word GCD 2.82%.

In the equal-layout repeated executable, the normalizer grows from 326 to 332
bytes. Its `divsd` is replaced by one constant subtraction and a compare/CMOV
for the lone subnormal reciprocal. Whole-executable text/data/BSS sections are
exactly unchanged; the unstripped file grows 16 bytes.

The profile is `/tmp/hypermesh-exponent-reciprocal-final.data`.

## Rejected implementations

Three broader bit-level rewrites were fully removed:

| Candidate | Normalizer | Linked effect | Runtime reason for rejection |
| --- | ---: | ---: | --- |
| Complete bit-level lane normalization | 674 B | +384 B text, aggregate neutral, +464 B file | duplicates cold subnormal machinery and is much larger |
| Bit transform with defensive subnormal rejection | 439 B | +148 B text / -144 B BSS, +4 B aggregate | generated instructions +0.70%, branches +0.58% |
| Two-pass min/max bit transform | 386 B | +100 B text / -96 B BSS, +4 B aggregate | generated instructions +0.538%, cycles +0.204% |

The retained reciprocal-only form is the smallest measured rewrite: 332 bytes
for the function, section-neutral, and faster on the owning mesh path.

## Large-fixture heap and RSS

Heaptrack covers fixture construction plus one complete immediate union under
both policies. The allocation stream is identical between checkpoint 29 and
the retained implementation.

| Fixture | Input triangles | Allocations | Temporary allocations | Heaptrack peak | Heaptrack RSS range |
| --- | ---: | ---: | ---: | ---: | ---: |
| Generated projective | 13,452 | 200,749 | 10,358 | 7.50 MiB | 17.97–18.00 MiB |
| Dense boxes | 6,144 | 27,209 | 81 | 2.34 MiB | 10.70–10.89 MiB |

Independent Valgrind Massif parent/candidate runs give byte-identical maxima:
8,245,672 bytes generated and 2,597,144 bytes on dense boxes. This direct
maximum is the authoritative incremental heap comparison; Heaptrack RSS
includes profiler overhead and is retained only as a process-scale control.

Heaptrack recordings are
`/tmp/hypermesh-exponent-reciprocal-{generated,boxes}-{parent,candidate}-{strict,approximate-512}.heaptrack.zst`.
Valgrind recordings are
`/tmp/hypermesh-exponent-reciprocal-valgrind-{generated,boxes}-{parent,candidate}.massif`.

## Linked code and call graph

| Consumer | Checkpoint 29 | Checkpoint 30 | Movement |
| --- | ---: | ---: | ---: |
| General release native text | 4,065,252 | 4,065,252 | unchanged |
| Immediate release native text | 4,098,884 | 4,098,884 | unchanged |
| General release WASM `wasm-opt -Oz` | 2,740,378 | 2,740,649 | +271 / +0.0099% |
| Immediate release WASM `wasm-opt -Oz` | 2,755,421 | 2,755,692 | +271 / +0.0098% |
| General size native text | 1,871,306 | 1,871,338 | +32 / +0.0017% |
| Immediate size native text | 1,883,782 | 1,883,798 | +16 / +0.0008% |
| General size WASM `wasm-opt -Oz` | 1,165,586 | 1,165,612 | +26 / +0.0022% |
| Immediate size WASM `wasm-opt -Oz` | 1,175,965 | 1,175,991 | +26 / +0.0022% |
| Equal-layout repeated native text | 5,063,618 | 5,063,618 | unchanged |
| Equal-layout repeated aggregate | 5,321,325 | 5,321,325 | unchanged |

Release-native section aggregates are also exactly unchanged. The unstripped
canonical files grow 56 bytes because isolated target paths change metadata;
the equal-layout file comparison bounds the actual change at 16 bytes. In the
size-native rows, text growth is offset by equal BSS reduction, leaving both
aggregate section totals exactly unchanged.

The directly diffed five-crate graph remains 19,726 nodes / 39,456 edges. Its
JSON SHA-256 is identical to checkpoint 29:
`c937bcd90b749efa8a817f96c2ea0aabb10703562939fe2c77c5205b1c121457`.
There is no new policy, predicate, equality, topology, or terminal edge.

The final graph is
`/tmp/hyperstack-exponent-reciprocal-callgraph-final/callgraph.json`.

## Competitive and historical controls

The final CPU-9 Criterion rerun reports:

| Control | Hypermesh | Boolmesh | Manifold-rust |
| --- | ---: | ---: | ---: |
| Projective generated union | 6.1435–6.2477 ms (6.1849 center) | 749.89–753.70 us (752.03) | 654.29–664.94 us (659.01) |
| Dense exact-cell boxes | 725.84–731.83 us (729.42 center) | 6.7423–6.7655 ms (6.7502) | 4.2743–4.3102 ms (4.2941) |

Relative to checkpoint 29's centers, Hypermesh moves -0.097% generated and
+0.163% on boxes, both neutral at this session's noise scale. Hypermesh is
8.22x and 9.39x slower than Boolmesh and Manifold on the projective throughput
control. On dense exact-cell boxes it is 9.25x and 5.89x faster. Competitors
remain throughput comparators, not exactness oracles.

The first generated Criterion session was discarded: Hypermesh reported a
7.749–10.432 ms interval while both competitors remained at their normal
centers. A serialized rerun immediately returned the stable 6.185 ms center;
the outlier is recorded rather than averaged.

The retained-arrangement bracket gives a current stable-sample policy-paired
34.159 ms per union. Against the directional historical 944.8 ms row, that is
96.38% lower. Fixture and implementation evolution make this a trend, not a
direct A/B.

## Validation

The retained implementation passes:

- Hyperreal default, no-default, and all-feature suites (561 / 561 / 638
  library tests), including the exhaustive 2,046-exponent differential;
- Hyperlattice, Hyperlimit, Hypertri, and Hypermesh default, no-default, and
  all-feature suites; Hypermesh has 1,060 default/minimal and 1,061 all-feature
  library tests;
- five focused Hyperreal filter tests and all 1,060 default Hypermesh library
  tests under nightly AddressSanitizer;
- warning-denied all-feature and no-default Clippy and rustdoc across all five
  crates;
- Hyperreal and Hypermesh fuzz-bin checks and all-feature benchmark
  compilation;
- the complete Hypermesh dispatch-trace bench;
- release every-operation exactness under both policies, polygon/immediate
  agreement, 3,360/13,440-triangle stress, and complete 11,894-triangle input
  validation;
- eight Heaptrack recordings, four direct A/B Valgrind recordings, serialized
  CPU counters, a frame-pointer profile, native/WASM size consumers,
  competitive controls, and the five-crate call graph; and
- formatting, clean-diff, and post-commit worktree checks.

The approximately 56-minute full-resolution rotated Boolean was not rerun. It
enters the unchanged ordinary non-certified-input path, while this change only
constructs the same exact power-of-two scale inside a certified sign filter.
Its prior certified-empty result remains the relevant hard-path evidence.

All temporary repetition hooks and benchmark worktrees are removed during
checkpoint cleanup. Hyperlimit's pre-existing untracked `hyperlimit`
executable is not touched.
