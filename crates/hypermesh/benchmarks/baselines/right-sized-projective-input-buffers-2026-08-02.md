# Right-sized projective input buffers — 2026-08-02

This is Phase 7 checkpoint 32 of the workspace Hypermesh
path-completeness plan. The retained implementation is Hypermesh
`5b32e85ddb109371c39975ca423f11f36c311a11`, based on checkpoint 31
evidence revision `c993cb5f63cfe0086b1139a8db5450f610d200c3`. Hyperreal,
Hyperlattice, Hyperlimit, and Hypertri remain at
`3d50951775764f6ca50f5805b149c54cc423432c`,
`d11ca2f0e825d8e26048cfda5d1101df21dcfef0`,
`3e5d8816cd32bba46f48e0c6c13ab7a9da227784`, and
`c47601266e0b9b17d0c5a0764fa22b18168ada73`.

## Outcome

The compact projective input builder now gives its exact-dyadic scheduling
position array exactly one slot per source position. The former
`Option<Vec<_>>` collection grew geometrically through an adapter whose size
hint did not reach the vector; the 1,538-position dense-box rows therefore
retained 2,048 slots. The explicit complete scan retains exactly 1,538.

Support-plane storage now starts at six slots for the common axis-aligned box
path and at `min(triangle_count, 256)` for general input. The 256 value is a
seed, never a work or support limit: ordinary `Vec` growth still admits every
finite source support. This avoids the small allocation/move sequence without
reserving one `Plane` per triangle.

Policy-paired direct A/B results improve task clock and cycles on all six
fixture/policy rows:

- generated projective: 0.922% task, 0.877% cycles, 0.027% instructions,
  and 0.031% branches;
- retained arrangement: 2.727% task, 2.812% cycles, and effectively flat
  deterministic work (0.002% fewer instructions and 0.001% more branches);
  and
- dense boxes: 0.489% task, 0.617% cycles, 0.403% instructions, and 0.243%
  branches.

The 13,452-triangle generated and retained controls each remove eight
allocations. Dense boxes remove 20 allocations and 18 reconstructed temporary
allocations. Canonical native aggregate sections do not grow, optimized WASM
shrinks, and the equal-layout repeated executable shrinks 8,192 aggregate
bytes.

## Exactness, policy, and complete paths

Both changes affect allocation layout only.

1. Every successful exact-dyadic conversion produces the same binary64 value
   in the same source order as before.
2. If any coordinate lacks an exact dyadic representation, the partially
   filled cache is dropped and the former complete lossy-position scan runs.
   No converted prefix is published.
3. Approximate positions remain optional scheduling hints. They cannot certify
   a plane, equality, predicate, or topology result.
4. Every newly constructed support plane is still exact and passes the same
   policy-aware nondegeneracy validation.
5. Supplied supports, adjacent exact support reuse, invalid indices,
   non-dyadic rationals, and allocation growth retain their former complete
   paths.
6. The support seed is not a cap. Inputs with more supports grow normally; no
   hidden work limit or silent fallback was added.

The new regression fails the dyadic scan on a late `1/3` coordinate after a
valid prefix. It verifies exact incidence under both `STRICT` and
`APPROXIMATE_512`, and both rows remain `Certified`.

There is no policy branch, equality change, terminal, cache, or predicate
change. `STRICT` still forbids terminal approximation.
`APPROXIMATE_512` still permits approximation only at Hyperlimit's final
512-bit equality/sign interpretation.

## Serialized CPU A/B

Parent and candidate executables use equal-layout five-crate source trees,
CPU 9, one fixture construction per process, complete immediate unions, and
parent/candidate/candidate/parent order. Negative values are improvements.
The generated CPU control has 852 triangles; the separate large-heap control
has 13,452.

| Fixture / policy | Repetitions | Task | Cycles | Instructions | Branches | Branch misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated / `STRICT` | 1,501 | -1.006% | -1.035% | -0.027% | -0.032% | +0.318% |
| Generated / `APPROXIMATE_512` | 1,501 | -0.838% | -0.718% | -0.026% | -0.029% | -0.174% |
| Retained / `STRICT` | 401 | -2.752% | -2.906% | -0.002% | +0.003% | -1.637% |
| Retained / `APPROXIMATE_512` | 401 | -2.702% | -2.719% | -0.003% | -0.001% | -1.066% |
| Dense boxes / `STRICT` | 20,001 | -0.088% | -0.083% | -0.401% | -0.240% | -3.333% |
| Dense boxes / `APPROXIMATE_512` | 20,001 | -0.889% | -1.151% | -0.405% | -0.246% | -4.218% |

The retained clock gain is predominantly layout/cache behavior: its
instruction and branch totals are flat. The dense-box deterministic reduction
directly reflects avoided vector growth. Branch misses remain a noisy
secondary counter and are not an independent retention gate.

## Dispatch and profile evidence

The generated dispatch trace exactly reproduces checkpoint 31:

| Event | Count |
| --- | ---: |
| Dispatch | 97,347 |
| Predicate | 676 |
| Linear algebra | 1,411 |
| Cache / hits / misses | 6,345 / 6,107 / 216 |
| Active cycles proposed / certified | 45 / 45 |
| Rational temporaries | 12,794 |
| Unknown / fallback-or-abort | 0 / 0 |

Capacity changes therefore introduce no arithmetic, predicate, proposal,
terminal, or fallback event.

The final frame-pointer profile covers 501 strict generated unions on CPU 9,
13,585 samples, zero lost samples, and approximately 13.585 billion cycles.
The targeted `mesh::build_projective_input_soup` falls from checkpoint 31's
4.57% self to 0.54%. Current leading owners are exact crossing splitting
(5.67%), mixed-width GCD (4.00%), word GCD (3.88%), the certified rational
line filter (3.22%), projective convex-face computation (2.07%), and the two
lossy-export bodies (1.51% and 0.87%). Sampling percentages are trend
evidence; serialized counters above are authoritative.

## Large-fixture heap

Heaptrack includes fixture construction and one complete immediate union under
both policies. Massif directly brackets strict parent/candidate runs.

| Fixture | Input triangles | Allocations | Temporary allocations | Heaptrack peak | Massif maximum | Useful heap |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated projective | 13,452 | 200,748 → 200,740 | 10,357 → 10,356 | 7.50 → 7.50 MiB | 8,245,672 → 8,245,672 B | 7,420,542 → 7,420,542 B |
| Retained arrangement | 4,524 | 453,997 → 453,989 | 28,734 → 28,733 | 11.67 → 11.67 MiB | 12,699,320 → 12,701,088 B | 11,590,063 → 11,591,727 B |
| Dense boxes | 6,144 | 27,208 → 27,188 | 79 → 61 | 2.34 → 2.34 MiB | 2,597,128 → 2,597,128 B | 2,262,494 → 2,262,494 B |

The retained Massif maximum rises 1,768 bytes, or 0.0139%, while its rounded
Heaptrack peak remains unchanged and its allocation count falls. This tiny
allocator-layout cost is retained transparently because the same workload is
2.73% faster by task clock, 2.81% faster by cycles, and performance has the
stated priority. Generated and dense-box direct maxima are byte-identical.
Heaptrack RSS ranges remain profiler noise.

## Linked code and call graph

| Consumer | Checkpoint 31 | Checkpoint 32 | Movement |
| --- | ---: | ---: | ---: |
| General release native text | 4,065,604 | 4,065,204 | -400 |
| General release native aggregate | 4,305,535 | 4,305,535 | unchanged |
| Immediate release native text | 4,099,252 | 4,098,852 | -400 |
| Immediate release native aggregate | 4,342,399 | 4,342,399 | unchanged |
| General release WASM `wasm-opt -Oz` | 2,740,864 | 2,740,709 | -155 |
| Immediate release WASM `wasm-opt -Oz` | 2,755,907 | 2,755,752 | -155 |
| General size native text | 1,871,474 | 1,871,042 | -432 |
| General size native aggregate | 2,114,116 | 2,114,116 | unchanged |
| Immediate size native text | 1,883,926 | 1,883,510 | -416 |
| Immediate size native aggregate | 2,126,408 | 2,126,408 | unchanged |
| General size WASM `wasm-opt -Oz` | 1,165,740 | 1,165,389 | -351 |
| Immediate size WASM `wasm-opt -Oz` | 1,176,119 | 1,175,768 | -351 |
| Equal-layout repeated native text | 5,062,270 | 5,056,094 | -6,176 |
| Equal-layout repeated aggregate | 5,321,337 | 5,313,145 | -8,192 |
| Equal-layout repeated file | 6,381,240 | 6,373,288 | -7,952 |

The explicit scan removes the large generic `Option<Vec<_>>` iterator
instantiation. Native canonical text shrink is balanced by BSS placement, so
aggregate sections remain unchanged; both optimized-WASM profiles shrink.

The complete five-crate source graph is 19,744 nodes / 39,498 edges. The
one-node/nine-edge increase is exactly the new regression. Its JSON SHA-256 is
`89d03d7b45239310d4b96814538f8bd2ace4b3bcb962b5bdcf2443dc9bce32ba`.
No production policy, equality, terminal, or topology edge is added.

## Competitive and historical controls

Final serialized CPU-9 Criterion controls report:

| Control | Hypermesh | Boolmesh | Manifold-rust |
| --- | ---: | ---: | ---: |
| Projective generated union | 6.1914–6.2486 ms (6.2274 center) | 748.39–753.73 us (751.18) | 661.23–668.52 us (664.16) |
| Dense exact-cell boxes | 718.85–725.16 us (721.19 center) | 6.8425–6.9348 ms (6.8849) | 4.3141–4.3562 ms (4.3405) |

Relative to checkpoint 31, all three generated centers improve
0.54–1.73%, so the generated competitor ratios move to 8.29x and 9.38x
slower. Dense Hypermesh improves 3.49%, versus 0.75% for Boolmesh and 1.82%
for Manifold, and is now 9.55x and 6.02x faster on that control. Competitors
remain throughput comparators, not exactness oracles.

The retained policy-paired direct center is 34.309 ms per union. Against the
directional historical 944.8 ms row, that is 96.37% lower or 27.54x faster.
Fixture and implementation evolution make this a trend, not a direct A/B.

## Rejected implementations

- Generic lazy source approximation reduced only about 0.007% generated
  instructions and regressed dense-box clocks/cycles about 1.6–1.7%.
- Specialized inline lazy conversion improved generated cycles about 0.33%
  but regressed dense boxes about 0.45–0.51%.
- A whole-function `#[cold]` layout sometimes benchmarked favorably but
  falsely labeled a method called for every source vertex. Truthful hit/miss
  and no-inline splits did not clear dense-box clocks.
- Direct source-identity-cycle lookup added about 4 KiB repeated text and
  increased dense-box instructions about 0.85%.
- Exact-capacity position collection alone shrank code and improved dense
  deterministic work, but exposed a retained clock regression. The bounded
  support reserve is the cohesive shape that clears every runtime row.

No rejected implementation remains in a production tree. No compatibility
shim or dependency was added.

## Validation

The retained implementation passes:

- Hyperreal default/no-default/all-feature suites with 562 / 562 / 639
  library tests;
- Hypermesh default/no-default/all-feature suites with 1,063 / 1,063 / 1,064
  library tests and every integration suite;
- Hyperlattice, Hyperlimit, and Hypertri default/no-default/all-feature
  matrices;
- warning-denied Clippy under all and minimal features, warning-denied
  rustdoc, and formatting for all five crates;
- Hypermesh fuzz-bin checks and all-feature benchmark compilation;
- all 1,063 default Hypermesh library tests under AddressSanitizer with leak
  detection disabled;
- release every-operation exactness under both policies, polygon/immediate
  agreement, 3,360/13,440-triangle stress, and complete 11,894-triangle input
  validation; and
- dispatch trace, six high-repetition CPU rows, twelve Heaptrack recordings
  across three large fixtures, six direct Massif runs, native/WASM size
  consumers, competitive controls, profile, and five-crate call graph.

The first ASAN link attempt exhausted the temporary user quota. Removing only
the checkpoint worktree and partial sanitizer target recovered the quota; the
clean retry passed all 1,063 tests. This was an environment failure, not a
test failure.

The approximately 56-minute full-resolution rotated Boolean was not rerun.
This change only selects vector capacities: it cannot alter a predicate,
proposal, policy terminal, or fallback. The release stress/input gates and
prior certified-empty run cover the path without spending an hour on an
unchanged decision sequence.

All temporary repetition hooks and rejected worktrees are removed.
Hyperlimit's pre-existing untracked `hyperlimit` executable is untouched.
