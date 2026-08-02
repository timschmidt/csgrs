# Adaptive crossing bounds cache — 2026-08-02

This is Phase 7 checkpoint 33 of the workspace Hypermesh
path-completeness plan. The retained implementation is Hypermesh
`7acad3848913388b915747cb5d516c425c92865a`, based on checkpoint 32
evidence revision `0ad1717dcdfb254c397a350c72703b57aabaf78a`.
Hyperreal, Hyperlattice, Hyperlimit, and Hypertri remain at
`3d50951775764f6ca50f5805b149c54cc423432c`,
`d11ca2f0e825d8e26048cfda5d1101df21dcfef0`,
`3e5d8816cd32bba46f48e0c6c13ab7a9da227784`, and
`c47601266e0b9b17d0c5a0764fa22b18168ada73`.

## Outcome

The certified-enclosure crossing sweep now caches one complete approximate
edge-bound vector at the same 256-edge admission point that already selects
the adaptive sweep. The former 1,024-edge cache threshold made the canonical
456-edge generated output recompute a complete left bound for every left edge
and reread right endpoint enclosures throughout the nested sweep. Construction
of the exact-capacity side vector is cheaper once the adaptive tier is active.

The production change is three insertions and two deletions. It adds no type,
helper, dependency, compatibility path, index narrowing, policy branch, or
code-size movement. On the canonical generated control, policy-paired direct
A/B improves task clock 1.353%, cycles 1.407%, instructions 1.280%, and
branches 0.167%. Retained and dense-box controls remain neutral. The sampled
crossing owner falls from 5.54% to 4.42% self.

## Exactness, policy, and complete paths

The cache contains only the same outward-rounded binary64 edge enclosures that
the direct path already constructs.

1. An enclosure rejection remains a proof of exact separation. Every survivor
   still reaches the complete exact projected crossing predicate.
2. The cache covers all three axes and every bounded edge; it is not a sampled
   or partial topology test.
3. `try_reserve_exact` failure still selects the former direct, complete
   per-left/per-right enclosure path. Allocation failure cannot skip work.
4. Sweeps below 256 edges remain direct. Sweeps at or above 1,024 edges are
   behaviorally unchanged. The newly cached tier is exactly 256–1,023 edges.
5. Inputs without certified enclosures retain the complete symbolic/exact
   bounds sweep. Shared endpoints, exact separation hidden by binary64
   rounding, coplanar/projected crossings, T-junctions, and independent event
   batches retain their existing exact paths.
6. `STRICT` still forbids approximate decisions. `APPROXIMATE_512` can consume
   approximation only in Hyperlimit's terminal 512-bit equality/sign
   interpretation. This scheduling cache cannot set result certainty or
   certify a predicate.

The existing threshold regression constructs enough exact separated triangles
to enter the cached tier, runs both policies, verifies no topology mutation,
and requires `Certified`. Exact-rounding, symbolic, independent-batch, and
hidden-separation regressions also pass.

## Serialized CPU A/B

Parent and candidate executables use equal-layout five-crate trees, identical
`-C target-cpu=native -C codegen-units=1` flags, CPU 9, one fixture build per
process, and parent/candidate/candidate/parent order. The retained strict row
also includes a reverse candidate/parent/parent/candidate bracket to cancel a
first-bracket thermal drift. Negative values are improvements.

| Fixture / policy | Repetitions | Task | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated 852 / `STRICT` | 1,501 | -2.260% | -2.277% | -1.282% | -0.168% | +0.064% | -0.061% |
| Generated 852 / `APPROXIMATE_512` | 1,501 | -0.445% | -0.537% | -1.279% | -0.166% | +0.639% | +1.600% |
| Retained 4,524 / `STRICT` | 401, four runs per side | -0.069% | -0.107% | +0.001% | +0.002% | +0.039% | -0.221% |
| Retained 4,524 / `APPROXIMATE_512` | 401 | -0.011% | +0.039% | +0.000% | +0.001% | -1.078% | -0.512% |
| Dense boxes 6,144 / `STRICT` | 20,001 | +0.886% | +1.013% | +0.000% | +0.001% | +1.846% | -0.755% |
| Dense boxes 6,144 / `APPROXIMATE_512` | 20,001 | -0.769% | -0.723% | +0.004% | +0.006% | -0.697% | -0.125% |

Policy-paired means are:

- generated: -1.353% task, -1.407% cycles, -1.280% instructions, and
  -0.167% branches;
- retained: -0.040% task, -0.034% cycles, +0.0007% instructions, and
  +0.0012% branches; and
- dense boxes: +0.059% task, +0.145% cycles, +0.0023% instructions, and
  +0.0036% branches.

The opposite strict/approximate box clock movements expose thermal/run-order
noise; deterministic work is neutral, and neither row admits the cache. The
retained path already cached at 1,024, so its deterministic work is likewise
neutral. A supplemental 13,452-triangle generated-input bracket still improves
policy-paired task/cycles/instructions/branches by 2.391%/2.016%/0.818%/0.107%.

## Dispatch and profile evidence

The generated dispatch trace exactly reproduces checkpoint 32:

| Event | Count |
| --- | ---: |
| Dispatch | 97,347 |
| Predicate | 676 |
| Linear algebra | 1,411 |
| Cache / filter hits / filter misses | 6,345 / 6,107 / 216 |
| Active cycles proposed / certified | 45 / 45 |
| Rational temporaries | 12,794 |
| Unknown / fallback-or-abort | 0 / 0 |

The final frame-pointer profile covers 501 strict generated unions on CPU 9,
14,196 samples, zero lost samples, and approximately 14.196 billion cycle
events. `split_edge_crossing_events` is 4.42% self, down from the parent
profile's 5.54%. Current leading owners are crossing splitting (4.42%), word
GCD (4.09%), allocator internals (3.72%), the certified rational line filter
(3.46%), fixed 512-bit GCD (2.79%), and lossy rational export (2.49%). Sampling
percentages are directional; the serialized counters above are the retention
gate.

## Large-fixture heap

Heaptrack includes fixture construction and one complete immediate union under
both policies. Massif directly brackets strict parent/candidate runs.

| Fixture | Input triangles | Allocations | Temporary allocations | Heaptrack peak | Massif maximum | Useful heap |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated projective | 13,452 | 200,742 → 200,743 | 10,358 → 10,358 | 7.50 → 7.50 MiB | 8,245,672 → 8,245,672 B | 7,420,542 → 7,420,542 B |
| Retained arrangement | 4,524 | 453,990 → 453,990 | 28,734 → 28,734 | 11.67 → 11.67 MiB | 12,699,112 → 12,699,112 B | 11,590,135 → 11,590,135 B |
| Dense boxes | 6,144 | 27,189 → 27,189 | 62 → 62 | 2.34 → 2.34 MiB | 2,597,128 → 2,597,128 B | 2,262,494 → 2,262,494 B |

The generated mid-tier cache costs exactly one bounded allocation. It does not
change reconstructed temporary allocations or the exact measured peak. The
retained path already allocated its cache; the box path remains below the
threshold. Strict and approximate Heaptrack counts match. RSS ranges are
profiler noise: 17.83–17.87 MiB generated, 21.00–21.22 MiB retained, and
10.42–10.66 MiB boxes.

## Linked code and call graph

Every canonical size consumer exactly matches checkpoint 32:

| Consumer | Text or optimized WASM | Aggregate native sections |
| --- | ---: | ---: |
| General release native | 4,065,204 B | 4,305,535 B |
| Immediate release native | 4,098,852 B | 4,342,399 B |
| General release WASM `wasm-opt -Oz` | 2,740,709 B | — |
| Immediate release WASM `wasm-opt -Oz` | 2,755,752 B | — |
| General size native | 1,871,042 B | 2,114,116 B |
| Immediate size native | 1,883,510 B | 2,126,408 B |
| General size WASM `wasm-opt -Oz` | 1,165,389 B | — |
| Immediate size WASM `wasm-opt -Oz` | 1,175,768 B | — |
| Equal-layout repeated probe | 4,636,065 B | 4,887,164 B |

The repeated probe's unstripped file is also unchanged at 5,586,352 bytes.
The complete five-crate source graph remains exactly 19,744 nodes / 39,498
edges. Its JSON remains byte-identical with SHA-256
`89d03d7b45239310d4b96814538f8bd2ace4b3bcb962b5bdcf2443dc9bce32ba`.
The Hypermesh-only graph is 8,068 nodes / 19,864 edges. No policy, equality,
terminal, allocation-fallback, or topology edge was added.

## Competitive and historical controls

Fresh serialized CPU-9 Criterion controls report:

| Control | Hypermesh | Boolmesh | Manifold-rust |
| --- | ---: | ---: | ---: |
| Projective generated union | 6.5889–6.6647 ms (6.6253 center) | 791.77–809.74 us (797.89) | 713.15–734.75 us (721.65) |
| Dense exact-cell boxes | 761.84–777.83 us (769.81 center) | 7.7071–7.9063 ms (7.8133) | 5.0585–5.1129 ms (5.0869) |

The machine was uniformly slower than the immediately preceding Criterion
session: every engine moved upward, including competitors by 6–19%. Ratios
remain the meaningful same-session comparison. Hypermesh is 8.30x and 9.18x
slower than Boolmesh and Manifold on the projective control, while it is
10.15x and 6.61x faster on dense exact-cell boxes. Competitors remain
throughput comparators, not exactness oracles.

The final retained policy-paired direct center is 36.287 ms per union. Against
the directional historical 944.8 ms row, that is 96.16% lower or 26.04x
faster. Fresh cube-union and subdivided-192 centers are 2.423 ms and 83.922 ms;
both reflect the same machine-wide slowdown and remain tracked against the
2.284 ms closure target and 80.019 ms historical gate.

## Rejected implementations

- Lazy and eager per-left certified line-filter caches increased branches or
  clocks; filter evaluation, not carrier construction, remained the sampled
  cost.
- A Hyperreal magnitude-bit normal/zero classifier increased generated
  instructions 1.40% and was fully removed.
- Packing exact edge endpoint order into one bit mask reduced storage but
  added hot accessor work. Combined with the lower cache threshold it improved
  generated instructions about 1.33% but repeatedly regressed retained and box
  clocks by roughly 0.5–0.6%.
- Restoring direct sweep fields to that packed carrier still increased
  generated cycles about 1.0%, grew the repeated binary 960 bytes, and was
  fully removed.
- The retained two-line threshold change has the smallest source/binary shape,
  preserves direct endpoint loads, and clears the complete runtime and heap
  controls.

No diagnostic counters, measurement hooks, rejected representations, or
Hyperreal experiments remain in a production tree.

## Validation

The retained implementation passes:

- Hyperreal default/no-default/all-feature suites with 562 / 562 / 639 tests;
- Hyperlattice 19 / 19 / 19, Hyperlimit 142 / 142 / 150, and Hypertri
  3 / 3 / 66 default/no-default/all-feature library matrices;
- Hypermesh default/no-default/all-feature suites with 1,063 / 1,063 / 1,064
  library tests and every integration suite;
- all 1,063 default Hypermesh library tests under AddressSanitizer with leak
  detection disabled;
- warning-denied Clippy and rustdoc under all and minimal features, formatting,
  fuzz-bin checks, and all-feature benchmark compilation;
- release every-operation exactness under both policies, polygon/immediate
  agreement, 3,360/13,440-triangle stress, and full 11,894-triangle input
  validation; and
- dispatch trace, canonical and supplemental CPU rows, twelve Heaptrack
  recordings, six direct Massif runs, native/WASM size consumers, competitive
  and historical controls, frame-pointer profile, and both call graphs.

The first opt-in release-test attempt used an isolated target directory that
did not contain the already validated external fixture and failed before test
execution when the sandbox blocked a download. Reusing the canonical cached
fixture made the unchanged command pass; this was an environment-path error,
not a test failure.

The approximately 56-minute rotated 11,894-by-11,894 Boolean was not rerun.
The change neither alters output topology nor removes an exact/fallback path;
the full input validator, 13,440-triangle output stress, exact hidden-separation
tests, and byte-identical dispatch sequence exercise the affected schedule.

All temporary repetition hooks are removed. Hyperlimit's pre-existing
untracked `hyperlimit` executable is untouched.
