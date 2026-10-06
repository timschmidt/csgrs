# Out-of-line exact product-sum fallback — 2026-08-01

This is Phase 7 checkpoint 29 of the workspace Hypermesh path-completeness
plan. The retained implementation is Hyperreal
`aa4b76b955ecdf46d133d91ef215b9f0c985c1d2`, based on checkpoint 28's
Hyperreal `a90fd36aca8df4aab4661c068f2b29961d657da2` and Hypermesh evidence
revision `7179747e13cc4c35731a54ce63898ba83d2b5aba`. Hypermesh production
remains at `5c48a413d1895ccd8e5bc5d56a06f127d2ce82f6`; Hyperlattice, Hyperlimit,
and Hypertri remain at `d11ca2f0e825d8e26048cfda5d1101df21dcfef0`,
`3e5d8816cd32bba46f48e0c6c13ab7a9da227784`, and
`c47601266e0b9b17d0c5a0764fa22b18168ada73`.

## Outcome

The fixed signed-product ordering dispatcher keeps its all-zero, single-term,
and direct unplanned dyadic-word paths inline. Only after every direct path
declines does it call a private `#[inline(never)]` helper containing the
unchanged complete bit-width plan and arbitrary-precision reducers.

This reduces paired generated instructions 0.653%, retained instructions
0.127%, and dense-box instructions 0.279%. Policy-paired task clock improves
0.746%, 0.890%, and 0.435% respectively. The equal-layout repeated binary
shrinks 4,404 text bytes and 4,084 aggregate section bytes. Native canonical
text and size-profile optimized WASM also shrink; speed-profile optimized WASM
grows 12.4 KiB (about 0.45%). Runtime has priority, so that bounded WASM cost is
retained.

Exact Valgrind Massif maxima are byte-identical to checkpoint 28 on the
13,452-triangle generated and 6,144-triangle dense-box fixtures. Heaptrack
reports identical strict/approximate peaks; generated allocation calls improve
by one and dense-box calls are unchanged.

The production diff is one file, 61 insertions and 50 deletions. Almost all of
that movement is the existing fallback body. There is no public API, new
dependency, compatibility shim, predicate, cache, allocator, policy branch,
terminal, or topology path.

## Complete exact path invariant

`Rational::signed_product_sum_ordering` still executes the same ordered path
family:

1. derive every exact product sign;
2. return exact equality for an all-zero sum;
3. return the exact sign for a single nonzero product;
4. try the existing direct unplanned dyadic word accumulator for admitted
   four- and six-term/two-factor forms;
5. build the complete dyadic bit-width plan;
6. try the planned dyadic word accumulator when the plan does not prefer the
   wide path;
7. try the exact non-dyadic word accumulator when no dyadic plan exists;
8. try the bounded 384-bit stack accumulator for four-term/two-factor dyadic
   forms;
9. reduce an arbitrary-precision dyadic sum;
10. compare arbitrary-precision magnitudes directly for equal product
    denominators; or
11. construct the exact LCM scale and compare arbitrary-precision positive and
    negative totals.

Steps 5–11 moved as one body into the helper. Their condition order, trace
labels, exact arithmetic, return values, and fallback relationships are
unchanged. The reference arrays and one-byte signs are passed by value; no
rational is cloned or materialized at the boundary. A helper return is still
an exact `Ordering`, never a binary64 or approximate decision.

Existing differential tests cover zero/single/direct paths, overflow, wide
dyadic stack success and overflow, arbitrary dyadic, equal-denominator, mixed
LCM, and four- and six-term populations against a materialized exact rational.
The 512-case unplanned four/six-term differential remains green. The explicit
dispatch-trace regression still reaches the wide stack path.

## Policy and topology behavior

Hyperreal rational ordering has no policy terminal. Hypermesh continues to
route every topology decision through its immutable `MeshContext` and
Hyperlimit policy:

- `STRICT` cannot consume a terminal approximation; and
- `APPROXIMATE_512` may terminate only at Hyperlimit's final 512-bit
  equality/sign interpretation.

Moving an exact fallback cannot consume approximation or alter aggregate mesh
certainty. On every measured large fixture, both policies return `Certified`
and exactly the same output. The generated projective fixture remains
154 vertices / 304 triangles, the retained arrangement remains 625 / 1,246,
and the dense boxes remain 27 / 50. Release tests cover union, intersection,
difference, and symmetric difference, exact directed closure, exact
nondegeneracy, and polygon/immediate agreement under both policies.

The all-family runtime trace is unchanged on the projective control: 97,131
dispatch events, 676 predicate events, zero unknown facts, zero fallback/abort
events, 12,794 rational temporaries, and the same exact ordering paths. In
particular it reaches direct word, planned stack, arbitrary dyadic,
equal-denominator, and LCM ordering reducers without adding another policy or
terminal spine.

## Serialized CPU work

Checkpoint-28 and checkpoint-29 executables use equal-layout source worktrees,
CPU 9, fixture-once construction, complete immediate unions, and
parent/candidate/candidate/parent process order. Retired instructions are the
primary retention gate. Every row is certified and produces the output counts
above.

| Fixture / policy | Repetitions | Task | Cycles | Instructions | Branches | Branch misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated / `STRICT` | 501 | -0.655% | -0.664% | -0.6517% | -0.0678% | +0.519% |
| Generated / `APPROXIMATE_512` | 501 | -0.837% | -0.846% | -0.6538% | -0.0701% | +0.281% |
| Retained / `STRICT` | 201 | -0.959% | -0.891% | -0.1249% | -0.0163% | -0.608% |
| Retained / `APPROXIMATE_512` | 201 | -0.821% | -0.654% | -0.1285% | -0.0210% | -1.168% |
| Dense boxes / `STRICT` | 10,001 | -1.461% | -1.416% | -0.2779% | -0.0388% | -2.586% |
| Dense boxes / `APPROXIMATE_512` | 10,001 | +0.592% | +0.345% | -0.2802% | -0.0416% | +2.285% |

Policy-paired movements are -0.746% task / -0.653% instructions generated,
-0.890% / -0.127% retained, and -0.435% / -0.279% on boxes. Every policy row
retires fewer instructions and branches. The policy-opposed box clock and miss
movement is noise: exact retired work improves almost identically, and the
final Criterion center is neutral.

## Profile and generated layout

The CPU-9 frame-pointer profile covers 501 strict generated unions, 18,118
samples, zero lost samples, and approximately 18.980 billion cycles. The
four-by-two dispatcher falls from 6.01% self in checkpoint 28 to 4.72%; its
outlined complete helper owns 0.45%. Combined sampled ownership is 5.17%, while
the deterministic whole-program instruction reduction is 0.653%.

The six-by-two dispatcher/helper split is 3.68% / 0.36%. Other leading self
owners are projective input construction 4.57%, lossy rational export 4.39%,
crossing-event splitting 3.66%, mixed-width GCD 3.29%, exact filter
normalization 2.62%, and word GCD 2.62%. The next profile work therefore stays
with conversion, crossing resolution, compact preparation, and exact GCD/filter
families rather than rebuilding this dispatcher.

In the equal-layout repeated executable, each parent four-by-two monolith is
11,071 bytes. The current callers are 2,011 and 2,059 bytes and share a 9,923
byte helper. The parent six-by-two monolith is 8,652 bytes; the current caller
is 2,439 bytes and its helper is 7,897 bytes. Other const instantiations likewise
keep the common dispatcher small while retaining a complete specialized
fallback.

The profile is
`/tmp/hypermesh-outlined-fallback-exact-final.data`.

## Large-fixture heap and RSS

Heaptrack covers fixture construction plus one complete immediate union under
both policies. The 3,372-triangle generated row is an additional scale control;
the direct checkpoint comparison uses the 13,452- and 6,144-triangle rows.

| Fixture | Input triangles | Allocations, parent → current | Current temporary allocations | Heaptrack peak | Current RSS, strict / approximate |
| --- | ---: | ---: | ---: | ---: | ---: |
| Generated projective | 13,452 | 200,750 → 200,749 | 10,358 | 7.50 MiB | 12,788 / 12,984 KiB |
| Dense boxes | 6,144 | 27,209 → 27,209 | 81 | 2.34 MiB | 7,848 / 7,644 KiB |
| Generated scale control | 3,372 | n/a → 154,714 | 10,358 | 7.50 MiB | 12,684 / 12,724 KiB |

Strict and approximate rows have identical allocation counts and rounded peaks.
Independent Valgrind Massif runs on the exact checkpoint-28 and checkpoint-29
binaries report byte-identical maxima: 8,245,672 bytes generated and 2,597,144
bytes on boxes. This is the authoritative incremental peak comparison; exported
Heaptrack timeline snapshots vary with sampling position even when their exact
allocation stream and rounded peak agree.

Heaptrack recordings are
`/tmp/hypermesh-outlined-fallback-{generated,boxes,generated4}-{strict,approximate-512}.heaptrack.zst`.
Valgrind recordings are
`/tmp/hypermesh-outlined-fallback-valgrind-{generated,boxes}-{parent,candidate}.massif`.

## Linked code and call graph

| Consumer | Checkpoint 28 | Checkpoint 29 | Movement |
| --- | ---: | ---: | ---: |
| General release native text | 4,065,340 | 4,065,252 | -88 / -0.0022% |
| Immediate release native text | 4,098,972 | 4,098,884 | -88 / -0.0021% |
| General release WASM `wasm-opt -Oz` | 2,728,000 | 2,740,378 | +12,378 / +0.454% |
| Immediate release WASM `wasm-opt -Oz` | 2,743,045 | 2,755,421 | +12,376 / +0.451% |
| General size native text | 1,871,386 | 1,871,306 | -80 / -0.0043% |
| Immediate size native text | 1,883,854 | 1,883,782 | -72 / -0.0038% |
| General size WASM `wasm-opt -Oz` | 1,165,862 | 1,165,586 | -276 / -0.0237% |
| Immediate size WASM `wasm-opt -Oz` | 1,176,241 | 1,175,965 | -276 / -0.0235% |
| Equal-layout repeated native text | 5,067,282 | 5,062,878 | -4,404 / -0.0869% |
| Equal-layout repeated aggregate | 5,325,421 | 5,321,337 | -4,084 / -0.0767% |

General/immediate release-native file size falls 808 bytes. Their aggregate
sections move only +8 bytes because data grows 16 bytes and BSS grows 80 while
text falls. Size-native file size falls 80/64 bytes; its aggregate is unchanged
or eight bytes smaller. The speed-WASM growth is the only adverse artifact row
and is accepted for the consistent runtime win.

The directly diffed five-crate graph moves from 19,724 nodes / 39,453 edges to
19,726 / 39,456. The two nodes are the utility's `Rational::` and `Self::`
aliases for the one new private helper. Eighteen edges move into the helper,
fifteen leave the old monolith, and one dispatcher-to-helper edge is added.
Hypermesh-owned nodes and outgoing edges in that matched graph remain exactly
8,031 / 19,835. No policy, predicate, equality, topology, or terminal node is
added.

The final stack graph is
`/tmp/hyperstack-outlined-fallback-callgraph-final/callgraph.json`. A fresh
isolated invocation reports 8,059 / 19,835; its root-relative node count is not
used as an incremental baseline because byte-identical Hypermesh source reports
a different node scope than the prior isolated artifact. The matched graph JSON
diff above is the comparable result.

## Competitive and historical controls

The final CPU-9 Criterion rerun reports:

| Control | Hypermesh | Boolmesh | Manifold-rust |
| --- | ---: | ---: | ---: |
| Projective generated union | 6.1842–6.2020 ms (6.1909 center) | 750.31–755.43 us (752.95) | 668.76–674.71 us (671.12) |
| Dense exact-cell boxes | 725.80–731.56 us (728.23 center) | 6.7830–7.2648 ms (6.9827) | 4.3077–4.3354 ms (4.3242) |

Relative to checkpoint 28's stable centers, Hypermesh moves +0.575% generated
and +0.073% on boxes, both neutral at this session's noise scale. The serialized
instruction gates improve 0.653% and 0.279%. An earlier same-session generated
center of 6.2493 ms and box center of 746.39 us coincided with all competitors
slowing and is retained as environmental variance rather than averaged into the
final center.

Hypermesh is currently 8.22x and 9.22x slower than Boolmesh and Manifold on the
projective throughput control. On dense exact-cell boxes it is 9.59x and 5.94x
faster. Competitors remain throughput comparators, not exactness oracles.

The retained-arrangement CPU bracket gives a current policy-paired 33.803 ms per
union. Against the directional historical 944.8 ms row, that is 96.42% lower.
Fixture and implementation evolution make this a trend, not a direct A/B.

## Validation

The retained implementation passes:

- Hyperreal default, no-default, and all-feature suites (560 / 560 / 637
  library tests), plus the focused four-test nightly AddressSanitizer sweep;
- Hyperlattice, Hyperlimit, Hypertri, and Hypermesh default, no-default, and
  all-feature suites; Hypermesh has 1,060 default/minimal and 1,061 all-feature
  library tests;
- all 1,060 default Hypermesh library tests under nightly AddressSanitizer;
- warning-denied all-feature and no-default Clippy and rustdoc across all five
  crates;
- Hyperreal and Hypermesh fuzz-bin checks and all-feature benchmark
  compilation;
- the explicit signed-product stack dispatch test and the complete Hypermesh
  dispatch-trace bench;
- release every-operation exactness under both policies, polygon/immediate
  agreement, 3,360/13,440-triangle stress, and complete 11,894-triangle input
  validation;
- six new Heaptrack recordings, four direct A/B Valgrind recordings, serialized
  CPU counters, final frame-pointer profile, native/WASM size consumers,
  competitive controls, and the five-crate call graph; and
- formatting, clean-diff, and clean-worktree checks.

The approximately 56-minute full-resolution rotated Boolean was not rerun. It
enters the unchanged ordinary non-certified-input path, while this change only
moves an exact rational fallback body after the same direct admission checks.
Its prior certified-empty result remains the relevant hard-path evidence.

All temporary repetition hooks and benchmark worktrees were removed. Hyperlimit's
pre-existing untracked `hyperlimit` executable was not touched.
