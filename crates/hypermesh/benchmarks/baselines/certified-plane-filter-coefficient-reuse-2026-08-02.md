# Certified plane-filter coefficient reuse — 2026-08-02

This is Phase 7 checkpoint 31 of the workspace Hypermesh path-completeness
plan. The retained implementation is Hyperreal
`3d50951775764f6ca50f5805b149c54cc423432c` and Hypermesh
`7b21836c9f34f7d86c35d43c7669b9490f271807`, based on checkpoint 30's
Hyperreal `f6071aa430ce6e8fb10ed7c37e3b821d7c5b9d50` and Hypermesh evidence
revision `db7d97bb8adb7f3c630e10ab70d9868d5a1259a5`. Hyperlattice, Hyperlimit,
and Hypertri remain at `d11ca2f0e825d8e26048cfda5d1101df21dcfef0`,
`3e5d8816cd32bba46f48e0c6c13ab7a9da227784`, and
`c47601266e0b9b17d0c5a0764fa22b18168ada73`.

## Outcome

Hypermesh now reuses the certified rational four-term filter's already
computed positive-power-of-two normalized coefficients when it creates the
non-certifying projective plane schedule. Constructing the evidence also
warms the existing operation-local rational plane filter cache. This removes
a second lossy conversion of the same four exact coefficients and turns 216
later cache probes into hits on the generated-projective control.

The policy-paired direct A/B results improve:

- generated projective: 0.370% task, 0.453% cycles, 0.177% instructions, and
  0.154% branches;
- retained arrangement: 0.812% task, 0.730% cycles, 0.441% instructions, and
  0.616% branches; and
- dense boxes: 1.569% task, 1.586% cycles, 0.299% instructions, and 0.060%
  branches.

All six policy/fixture rows improve task clock, cycles, instructions, and
branches. Large-fixture allocation counts, Heaptrack peaks, and direct Massif
maxima are unchanged. Native release section aggregates are exactly
unchanged; optimized release WASM grows 215 bytes, and the size-profile
general native aggregate shrinks 8 bytes.

## Exactness and complete paths

`RationalPlane4PredicateEvidence::new` remains the entry gate. It succeeds
only when all four plane coefficients have exact rational representations.
Its optional filter has the existing Hyperreal proof: the exposed binary64
coefficients are scaled by an exact positive power of two and retain the
filter's error bound. They are explicitly not exact coefficients.

The values are used only for work scheduling:

1. A normalized binary64 bit key proposes an already-seen exact support
   plane. Exact plane equality verifies every approximate-key hit.
2. The active-plane schedule proposes and ranks cycles. Every proposed cycle
   is clipped exactly and verified against the complete candidate-plane set.
3. Empty, rejected, or incomplete proposals fall through to the full exact
   clipping path.
4. If the certified filter cannot be formed because the exponent span is
   unsafe, Hypermesh uses the former per-coefficient lossy conversion.
5. If any coefficient is not an exact rational, evidence creation returns
   `None`, so projective scheduling retains the former exact/certified
   fallback.

The new regressions cover both policies, the normalized small-integer result,
the unsafe `f64::MAX`/`MIN_POSITIVE` exponent-span fallback, and a non-exact
`Real::pi()` coefficient. The operation-local cache now also has a direct
capacity regression: insertion 8,193 clears its 8,192-entry/16,384-slot
bound, evicts the first entry, and retains only the newest entry. No cache or
evidence escapes the operation.

## Policy behavior

This change adds no policy branch and no terminal. The filter either supplies
certified evidence or declines; it cannot decide mesh equality or topology.
Cached filter entries are policy-independent because their result is proved,
not approximated.

- `STRICT` still forbids terminal approximation.
- `APPROXIMATE_512` still permits approximation only at Hyperlimit's final
  512-bit equality/sign interpretation.

Union, intersection, difference, and symmetric difference remain Certified
and byte-identical across both policies on every release control. Exact
closure and triangle nondegeneracy checks pass for the generated fixture, and
polygon/immediate APIs agree.

## Serialized CPU A/B

Parent and candidate executables use equal-layout five-crate source trees,
CPU 9, one fixture construction per process, complete immediate unions, and
parent/candidate/candidate/parent order. Negative values are improvements.

| Fixture / policy | Repetitions | Task | Cycles | Instructions | Branches | Branch misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated / `STRICT` | 1,501 | -0.662% | -0.596% | -0.178% | -0.155% | +0.188% |
| Generated / `APPROXIMATE_512` | 1,501 | -0.077% | -0.309% | -0.176% | -0.152% | +0.292% |
| Retained / `STRICT` | 401 | -0.674% | -0.553% | -0.444% | -0.619% | -0.666% |
| Retained / `APPROXIMATE_512` | 401 | -0.949% | -0.907% | -0.439% | -0.613% | +0.164% |
| Dense boxes / `STRICT` | 20,001 | -1.291% | -1.404% | -0.301% | -0.063% | -2.314% |
| Dense boxes / `APPROXIMATE_512` | 20,001 | -1.852% | -1.771% | -0.296% | -0.057% | -5.297% |

The deterministic instruction and branch reductions support the clock and
cycle movement. Branch misses are retained as a noisy secondary counter and
are not used as a gate.

## Dispatch and profile evidence

Generated dispatch totals change only where expected:

| Event | Parent | Candidate |
| --- | ---: | ---: |
| Dispatch | 97,131 | 97,347 |
| Predicate | 676 | 676 |
| Linear algebra | 1,411 | 1,411 |
| Cache | 6,129 | 6,345 |
| Cache hits | 5,891 | 6,107 |
| Cache misses | 216 | 216 |
| Active cycles proposed / certified | 45 / 45 | 45 / 45 |
| Rational temporaries | 12,794 | 12,794 |
| Unknown / fallback-or-abort | 0 / 0 | 0 / 0 |

The extra 216 dispatch/cache events are earlier certified cache hits. Exact
arithmetic, predicates, proposals, certifications, topology, and fallback
counts are unchanged.

The final frame-pointer profile covers 501 strict generated unions on CPU 9,
zero lost samples, and approximately 19.421 billion cycles. Leading self
owners remain the four-by-two product-sum dispatcher (5.00%), rational lossy
export (4.82%), projective input construction (4.57%), the six-by-two
dispatcher (3.91%), crossing splitting (3.81%), mixed-width GCD (3.02%),
filter normalization (2.61%), the projective builder (2.55%), word GCD
(2.43%), the line filter (2.41%), and rational-plane filter lookup (2.18%).
Sampling percentages are not used for a parent/candidate claim; the direct
CPU counters above are authoritative.

## Large-fixture heap

Heaptrack includes fixture construction and one complete immediate union for
both policies. Massif directly brackets parent/candidate strict runs.

| Fixture | Input triangles | Allocations | Temporary classification | Heaptrack peak | Massif maximum | Useful heap |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated projective | 13,452 | 200,749 → 200,749 | 10,357 → 10,357 | 7.50 → 7.50 MiB | 8,245,704 → 8,245,704 B | 7,420,566 → 7,420,566 B |
| Dense boxes | 6,144 | 27,209 → 27,209 | 80 → 79 | 2.34 → 2.34 MiB | 2,597,144 → 2,597,144 B | 2,262,518 → 2,262,518 B |

Generated parent/candidate RSS ranges are 17.99–18.06 and 17.94–18.12 MiB;
dense-box ranges are 10.82–10.95 and 10.92–11.01 MiB. These are profiler and
system noise. Identical Massif maxima are the authoritative peak-live-heap
result.

## Linked code and call graph

| Consumer | Checkpoint 30 | Checkpoint 31 | Movement |
| --- | ---: | ---: | ---: |
| General release native text | 4,065,252 | 4,065,604 | +352 / +0.0087% |
| General release native aggregate | 4,305,535 | 4,305,535 | unchanged |
| Immediate release native text | 4,098,884 | 4,099,252 | +368 / +0.0090% |
| Immediate release native aggregate | 4,342,399 | 4,342,399 | unchanged |
| General release WASM `wasm-opt -Oz` | 2,740,649 | 2,740,864 | +215 / +0.0078% |
| Immediate release WASM `wasm-opt -Oz` | 2,755,692 | 2,755,907 | +215 / +0.0078% |
| General size native text | 1,871,338 | 1,871,474 | +136 / +0.0073% |
| General size native aggregate | 2,114,124 | 2,114,116 | -8 |
| Immediate size native text | 1,883,798 | 1,883,926 | +128 / +0.0068% |
| Immediate size native aggregate | 2,126,408 | 2,126,408 | unchanged |
| General size WASM `wasm-opt -Oz` | 1,165,612 | 1,165,740 | +128 / +0.0110% |
| Immediate size WASM `wasm-opt -Oz` | 1,175,991 | 1,176,119 | +128 / +0.0109% |
| Equal-layout repeated native text | 5,064,586 | 5,064,994 | +408 / +0.0081% |
| Equal-layout repeated aggregate | 5,325,429 | 5,325,437 | +8 |
| Equal-layout repeated file | 6,387,208 | 6,387,184 | -24 |

The native text is balanced by BSS reduction because the optimizer changes
constant placement; neither release consumer's aggregate grows. The selected
runtime reduction is materially larger than the sub-0.011% linked-code cost.

The complete five-crate source graph is 19,743 nodes / 39,489 edges, up 17 /
33 for the new accessor and regressions. Its JSON SHA-256 is
`37568708858b8eb4725306952a007393473fe162cd95500bd4dec3200eb58c24`.
Inspection finds no new policy, terminal, equality, or topology edge.

## Competitive and historical controls

The final serialized CPU-9 Criterion controls report:

| Control | Hypermesh | Boolmesh | Manifold-rust |
| --- | ---: | ---: | ---: |
| Projective generated union | 6.2186–6.3160 ms (6.2610 center) | 759.49–767.84 us (764.26) | 673.02–679.50 us (675.84) |
| Dense exact-cell boxes | 745.73–749.70 us (747.24 center) | 6.9071–6.9749 ms (6.9370) | 4.3358–4.5746 ms (4.4209) |

Relative to checkpoint 30, all three generated centers move upward together
by 1.23–2.55%, and all three dense-box centers move upward by 2.44–2.95%.
Their ratios are stable: Hypermesh is 8.19x/9.26x slower on the deliberately
projective throughput control and 9.28x/5.92x faster on dense exact-cell
boxes. Competitors remain throughput comparators, not exactness oracles.

The retained-arrangement direct bracket gives 34.525 ms per current union.
Against the directional historical 944.8 ms row, that is 96.35% lower or
27.37x faster. Fixture and implementation evolution make this a trend, not a
direct A/B.

## Rejected implementations

- Hoisting checked polygon indices per triangle added about 206 bytes to the
  builder, 192 bytes of whole-executable text, and approximately 0.020%
  generated instructions. It was removed.
- Prewarming the cache while retaining raw lossy plane coefficients added 376
  bytes of repeated-executable text and improved generated strict
  instructions only about 0.013%, while branches rose about 0.11%. It was
  removed. This isolates normalized coefficient reuse as the useful work.

No rejected implementation remains in a production tree. No compatibility
shim or dependency was added.

## Validation

The retained implementation passes:

- Hyperreal default/no-default/all-feature suites with 562 / 562 / 639
  library tests;
- Hypermesh default/no-default/all-feature suites with 1,062 / 1,062 / 1,063
  library tests;
- Hyperlattice, Hyperlimit, and Hypertri default/no-default/all-feature
  matrices;
- warning-denied Clippy and rustdoc under all and minimal features for all five
  crates, plus formatting and diff checks;
- Hyperreal and Hypermesh fuzz-bin checks and all-feature benchmark
  compilation;
- six focused Hyperreal filter tests and all 1,062 default Hypermesh library
  tests under AddressSanitizer with leak detection disabled;
- release every-operation exactness under both policies, polygon/immediate
  agreement, 3,360/13,440-triangle stress, and complete 11,894-triangle input
  validation; and
- dispatch trace, high-repetition CPU A/B, eight Heaptrack runs, four direct
  Massif runs, native/WASM size consumers, competitive controls, profile, and
  five-crate call graph.

The approximately 56-minute full-resolution rotated Boolean was not rerun.
The change affects only a proposal schedule that is exact-verified and has a
complete exact fallback; the release stress/input gates and prior
certified-empty hard-path run cover the affected path without spending an
hour on an unchanged outcome.

All temporary repetition hooks, benchmark worktrees, profiler recordings, and
generated fixture copies are removed during checkpoint cleanup. Hyperlimit's
pre-existing untracked `hyperlimit` executable is not touched.
