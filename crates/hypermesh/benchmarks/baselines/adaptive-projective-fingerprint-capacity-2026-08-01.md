# Adaptive projective fingerprint capacity — 2026-08-01

This is Phase 7 checkpoint 28 of the workspace Hypermesh path-completeness
plan. The retained implementation is Hypermesh
`5c48a413d1895ccd8e5bc5d56a06f127d2ce82f6`, based on checkpoint 27 evidence
revision `99b54b772f55fdee1927c67ebc42ae9f88042ace` and implementation
`baccdc5c3a7f9174313311ec657670b8820d59f0`. The scalar and policy base remains
Hyperreal `a90fd36aca8df4aab4661c068f2b29961d657da2` and Hyperlimit
`3e5d8816cd32bba46f48e0c6c13ab7a9da227784`.

## Outcome

Projective coincidence resolution now creates its exact fingerprint table with
capacity for three quarters of the retained construction identities. This
matches the parent table's final capacity class on all three large fixtures
without paying its incremental allocation and rehash sequence. An all-unique
input remains complete: the table grows normally and needs at most one growth
from this initial bound.

The change removes six allocations from the generated 13,452-triangle fixture,
three from the 6,144-triangle dense boxes, and seven from the retained
4,524-triangle arrangement. Exact Massif maxima fall 273,824 bytes (3.699%)
generated and 23,112 bytes (1.015%) on boxes. The retained maximum moves only
+1,864 bytes (+0.016%) and is effectively flat.

Every serialized policy row retires fewer instructions and branches. Paired
generated task clock is neutral (-0.013%) while instructions fall 0.0236%; the
long retained row improves 1.288% in task clock while instructions fall
0.0138%. Dense-box task clock is noisy (+0.675% paired, with opposite policy
directions), but instructions fall 0.0246%, branch misses fall 0.923%, and its
independent Criterion center improves 0.362%. Final Criterion centers improve
0.546% generated and 0.362% on boxes relative to checkpoint 27.

The implementation is five inserted production lines and one deletion. It
adds no public API, policy branch, predicate, terminal, topology path, test-only
production hook, or compatibility layer. Canonical linked text grows 27–400
bytes depending on target/profile; the equal-layout repeated binary grows 116
text bytes while aggregate text/data/BSS shrinks 12 bytes.

## Capacity and exactness invariant

`resolve_vertex_coincidences` first drains and sorts every retained construction
identity. The new requested capacity is

```text
entries - floor(entries / 4)
```

using saturating subtraction. Capacity is only an allocation hint. It cannot
bound input, terminate a search, omit a candidate, or prove inequality.

The resolution paths are unchanged:

1. an exact projective-affine fingerprint schedules keyed candidates;
2. every same-key candidate is disambiguated by the existing policy-aware exact
   identity equality path;
3. an exact fingerprint collision that is not equal remains in the bucket;
4. keyed identities are still compared with every unkeyed identity; and
5. an unkeyed identity still takes the complete preceding-identity equality
   scan before joining the unkeyed set.

Disjoint-set merging, canonical representative selection, incidence merging,
and final identity installation are byte-for-byte unchanged. Existing
regressions exercise deliberately colliding modular fingerprints under both
policies and order-independent canonicalization across multiple construction
identities.

The measured table classes explain the three-quarter bound:

| Fixture | Entries | Fingerprint buckets | Requested | Parent final capacity | Current final capacity |
| --- | ---: | ---: | ---: | ---: | ---: |
| Generated projective | 179 | 135 | 135 | 224 | 224 |
| Dense boxes | 32 | 26 | 24 | 28 | 28 |
| Retained arrangement | 492 | 370 | 369 | 448 | 448 |

Reserving one slot per identity was rejected. It selected 56 slots instead of
28 on boxes and 896 instead of 448 on the retained fixture, and the retained
control regressed approximately 0.5–0.8% in stable cycles. That form was fully
removed along with all temporary capacity diagnostics.

## Policy and complete fallback behavior

No `Real` equality, predicate, topology, cache key, public API, or fallback
changes. Under `STRICT`, an unresolved predicate remains a typed indeterminate
result. Under `APPROXIMATE_512`, only Hyperlimit's terminal 512-bit
equality/sign interpretation may resolve an otherwise unresolved decision.
Table capacity cannot consume or invent approximation.

Both policies return `Certified` for the measured fixtures and produce exactly
equal meshes. The compact projective admission rules, ordinary full polygon
path, general subdivision engine, retryable compact rebuild, and non-retryable
error propagation are unchanged. No fixed capacity or allocator outcome is
translated into a successful mesh result.

## Exact output gates

| Fixture | Input triangles | Output vertices | Output triangles |
| --- | ---: | ---: | ---: |
| Generated projective | 13,452 | 154 | 304 |
| Retained arrangement | 4,524 | 625 | 1,246 |
| Dense subdivided boxes | 6,144 | 27 | 50 |

Union, intersection, difference, and symmetric difference pass under both
policies. Strict and approximate-policy meshes are exactly equal; each passes
exact directed closure and exact nondegeneracy. Polygon and immediate APIs
agree.

## Serialized CPU work

Checkpoint-27 and checkpoint-28 executables were built from equal-length
sibling worktree paths, pinned to logical CPU 9, and run in
parent/candidate/candidate/parent order. Each process constructs its fixture
once and repeats a complete immediate union. Retired instructions are the
deterministic retention gate; task clock and cycles are reported without
hiding their between-process variance.

| Fixture / policy | Repetitions | Task | Cycles | Instructions | Branches |
| --- | ---: | ---: | ---: | ---: | ---: |
| Generated / `STRICT` | 501 | +0.256% | +0.226% | -0.0263% | -0.0149% |
| Generated / `APPROXIMATE_512` | 501 | -0.282% | -0.241% | -0.0210% | -0.0073% |
| Dense boxes / `STRICT` | 10,001 | +2.101% | +1.708% | -0.0283% | -0.0142% |
| Dense boxes / `APPROXIMATE_512` | 10,001 | -0.751% | -0.704% | -0.0210% | -0.0023% |
| Retained / `STRICT` | 201 | -1.206% | -1.115% | -0.0105% | -0.0028% |
| Retained / `APPROXIMATE_512` | 201 | -1.370% | -1.316% | -0.0170% | -0.0103% |

Policy-paired movements are -0.013% task / -0.0236% instructions generated,
+0.675% / -0.0246% on boxes, and -1.288% / -0.0138% retained. Generated and
box branch misses fall 0.554% and 0.923%; retained branch misses move +0.616%.
The strict/approximate clock reversal on boxes cannot be caused by terminal
policy work because both runs remain certified and have nearly identical
retired work. The stable Criterion session below is the stronger elapsed-time
control for that fixture.

The companion TOML retains every raw bracket. It also names and excludes two
provenance errors caught before this checkpoint: an older artifact was the
checkpoint-26 parent rather than checkpoint 27, and an initial rebuilt
candidate came from a different-length manifest path. Neither measurement is
used here.

## Large-fixture heap

Heaptrack and Massif cover fixture construction plus one complete immediate
union under both policies.

| Fixture | Allocations, parent → current | Massif maximum, parent → current | Movement | Heaptrack peak | Current RSS range |
| --- | ---: | ---: | ---: | ---: | ---: |
| Generated projective | 200,756 → 200,750 | 7,401,790 → 7,127,966 B | -3.699% | 7.50 MiB | 17.67–17.88 MiB |
| Dense boxes | 27,212 → 27,209 | 2,276,472 → 2,253,360 B | -1.015% | 2.34 MiB | 10.59–10.81 MiB |
| Retained arrangement | 454,005 → 453,998 | 11,663,543 → 11,665,407 B | +0.016% | 11.67 MiB | 20.91–20.92 MiB |

The generated and box Massif reductions are the removed transient overlapping
hash-table growth allocations. The retained table selects the same final class
as its parent and remains flat at whole-process scale. Reconstructed temporary
counts stay 10,359 generated, 81 boxes, and 28,735 retained.

Heaptrack recordings are
`/tmp/hypermesh-fingerprint-adaptive-exact-{generated,boxes,retained}-{strict,approximate-512}.heaptrack.zst`.
Massif outputs are
`/tmp/hypermesh-fingerprint-adaptive-{generated,boxes,retained}.massif`. The
retained fixture SHA-256 is
`5f1ac8a2f8bf2b0c67ce95dc8146cc783155dc9c34eeee164f03446a0830886c`.

## Linked code and call graph

| Consumer | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| General release native text | 4,064,940 | 4,065,340 | +400 / +0.0098% |
| Immediate release native text | 4,098,588 | 4,098,972 | +384 / +0.0094% |
| General release WASM `wasm-opt -Oz` | 2,727,780 | 2,728,000 | +220 / +0.0081% |
| Immediate release WASM `wasm-opt -Oz` | 2,742,824 | 2,743,045 | +221 / +0.0081% |
| General size native text | 1,871,322 | 1,871,386 | +64 / +0.0034% |
| Immediate size native text | 1,883,790 | 1,883,854 | +64 / +0.0034% |
| General size WASM `wasm-opt -Oz` | 1,165,835 | 1,165,862 | +27 / +0.0023% |
| Immediate size WASM `wasm-opt -Oz` | 1,176,214 | 1,176,241 | +27 / +0.0023% |

General native aggregates are unchanged. The release immediate aggregate moves
one 4 KiB BSS/page class despite only 384 bytes of text growth; both
size-profile native aggregates are unchanged. The equal-layout repeated probe
moves from 6,386,464 to 6,386,568 file bytes and from 5,067,166 to 5,067,282
text bytes; BSS falls 128 bytes, leaving the aggregate 12 bytes smaller.

Hypermesh's comparable graph moves from 8,031 nodes / 19,827 edges to 8,034 /
19,829. The five-crate graph moves from 19,721 / 39,451 to 19,724 / 39,453.
The three nodes and two edges are capacity construction/arithmetic shape, not
another policy, predicate, equality, fallback, or terminal spine. The graph is
`/tmp/hypermesh-fingerprint-adaptive-callgraph/callgraph.json`.

## Cycle profile

The final CPU-9 frame-pointer profile covers 501 strict generated unions,
9,299 samples, zero lost samples, and approximately 19.536 billion cycles.
Largest self owners are four-by-two signed-product summation 6.01%, lossy
rational export 4.86%, compact input construction 4.66%, six-by-two summation
3.85%, crossing-event splitting 3.59%, mixed-width GCD 2.85%, word GCD 2.79%,
rational-filter normalization 2.67%, allocator work 2.57%, compact projective
preparation 2.41%, exact coordinate classification 2.22%, and the rational
filter 2.08%.

The parent profile's `[u64; 3]` fingerprint-table `reserve_rehash` owner is
absent from the current profile even at the 0.01% reporting cutoff. Sampling
attribution varies; paired retired instructions remain authoritative. The
profile is `/tmp/hypermesh-fingerprint-adaptive-exact-final.data`.

## Competitive and historical controls

The final stable CPU-9 Criterion session reports:

| Control | Hypermesh | Boolmesh | Manifold-rust |
| --- | ---: | ---: | ---: |
| Projective generated union | 6.1382–6.1782 ms (6.1555 center) | 746.65–751.39 us (748.16) | 653.52–659.96 us (657.04) |
| Dense exact-cell boxes | 725.92–729.73 us (727.70 center) | 6.7161–6.7349 ms (6.7248) | 4.2637–4.2718 ms (4.2673) |

The generated center is 0.546% below checkpoint 27. Hypermesh remains 8.23x
slower than Boolmesh and 9.37x slower than Manifold on this exact-heavy row.
The dense-box center is 0.362% below checkpoint 27; Hypermesh is 9.24x faster
than Boolmesh and 5.86x faster than Manifold. Competitors are throughput
comparators, not exactness oracles.

An earlier generated Criterion session spanning 6.1750–7.0796 ms was rejected
as unstable rather than averaged into the stable session.

The directional retained historical baseline remains 944.8 ms, 67.74 MiB,
and 5,020,891 allocations. Current direct work remains about 34.77 ms,
11.67 MiB, and now 453,998 allocations: 96.32%, 82.77%, and 90.96% lower.
Fixture and implementation evolution make this a trend, not a direct A/B.

## Validation

The final implementation passes:

- default and no-default test matrices with 1,060 library tests, and the
  all-feature/all-target matrix with 1,061;
- 1,060/1,060 default library tests under nightly AddressSanitizer;
- warning-denied all/no-default Clippy and rustdoc;
- every fuzz-bin check and all-feature benchmark compilation;
- release opt-in every-operation exactness under both policies,
  polygon/immediate agreement, 3,360/13,440-triangle stress, and complete
  11,894-triangle input validation;
- all-family dispatch tracing, with 97,131 dispatch events, 676 predicate
  events, zero unknown facts, and zero fallback/abort events on the generated
  row;
- six exact-source Heaptrack recordings, three Massif controls, serialized CPU
  counters, native/WASM size controls, competitive Criterion, the exact-source
  frame-pointer profile, and the five-crate call graph; and
- formatting and diff checks.

The temporary capacity diagnostics, repetition hooks, and benchmark worktrees
were removed. The approximately 56-minute full-resolution rotated Boolean was
not rerun because it enters the unchanged ordinary non-certified-input path;
this allocation-only construction change cannot alter that path's decisions.
