# Paired certified line-direction checkpoint

Date: 2026-08-01

Hyperreal direct parent: `dd226cb23b348b3983dc63142539c1c733c48757`

Hyperreal implementation: `7262d3037d056c9fee83b07d6d43cc3d7bf65277`

Hypermesh direct parent: `aa4bbc161f305ff4a103d529a44d27cd4e32c05c`

Hypermesh implementation: `f882f4310bd0f6d820c75c6a8e9bc58eaea682ad`

Other dependencies:

- Hyperlattice `d11ca2f0e825d8e26048cfda5d1101df21dcfef0`
- Hyperlimit `3e5d8816cd32bba46f48e0c6c13ab7a9da227784`
- Hypertri `c47601266e0b9b17d0c5a0764fa22b18168ada73`

This checkpoint shares one certified binary64 line direction across the two
query points tested against each projected segment during crossing discovery.
It changes neither the exact predicate nor its fallback: every certified sign
uses the same conservative error algebra as the prior single-query filter, and
every inconclusive sign reaches the same exact-rational orientation.

## Exactness and policy invariant

`RationalLine2Filter::sign_point3_pair` projects two already-certified rational
queries, computes the line's two differences and conservative errors once, and
then evaluates the two independent point differences and determinants. The
single-query API now calls the same factored direction/sign kernels, so there is
one certification formula rather than a second approximation family.

Hypermesh carries the paired results as `Option<Option<RealSign>>`:

- no outer value means that no paired query was available and permits the
  existing single-query filter;
- `Some(Some(sign))` is a proved sign and returns immediately; and
- `Some(None)` records an attempted but inconclusive proof and goes directly
  to the exact rational determinant without repeating approximate work.

Invalid projection axes return two inconclusive slots. A zero or uncertainty
boundary may leave either point inconclusive independently. Focused tests cover
positive/negative pairs, one boundary plus one proved sign, invalid axes, the
randomized exact-sign oracle, and crossing, same-side, and endpoint topologies.

The filter has no policy parameter because it can only certify a mathematical
sign. It cannot emit or consume a terminal approximation:

- `STRICT` still accepts certified or exact decisions only;
- `APPROXIMATE_512` still reaches Hyperlimit's terminal 512-bit interpretation
  only after the unchanged complete certification/exact stack is exhausted;
- no policy or certainty is cached in Hyperreal or in a mesh carrier; and
- every measured parent/candidate result under both policies has identical
  topology and `MeshCertainty::Certified`.

There is no epsilon, approximate equality, new pass/candidate limit, retained
allocation, cache, compatibility shim, or alternate topology path.

## Profile and path characterization

The retained profile immediately before this change attributes 6.00% self time
to `split_edge_crossing_events`; the certified rational line filter itself is
3.04%. Temporary exact diagnostics, removed before measurement, characterized
the complete sweep:

| Fixture | Sweep/bounds | Pair visits | Early breaks | Approximate rejects | Shared-endpoint rejects | Exact-bound rejects | Projected rejects | Events |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Retained 4,524 triangles | Z / cached | 217,963 | 1,848 | 187,488 | 10,952 | 0 | 17,675 | 0 |
| Generated 13,452 triangles | Y / direct | 22,163 | 435 | 16,387 | 2,487 | 0 | 2,854 | 0 |
| Boxes 6,144 triangles | X / direct | 1,655 | 50 | 1,075 | 365 | 0 | 165 | 0 |

All three cases have zero crossing events, which makes them strict rejection
controls: the optimization cannot hide construction or repair work behind a
changed result.

Measured alternatives were fully removed:

- sorting sweep edges with `sort_unstable` saved at most 0.306% instructions,
  worsened branches, and added 3,616 bytes of `.text`;
- omitting the already-proved sweep axis from later approximate overlap checks
  added 0.18--0.24% instructions; the constant-index form also added 0.51%
  branches;
- a per-vertex rational-query cache saved 1.16% retained instructions but
  added an allocation, 0.03 MiB peak heap, 2,240 bytes of `.text`, and regressed
  the projective Criterion row 6.67%;
- hoisting direction state into every 64-byte line-filter carrier added 0.11%
  retained instructions because single-query users could not amortize it; and
- the first paired implementation used `array::map`, whose generated drain
  machinery made the representative `.text` grow 3,568 bytes. Direct calls
  preserve the speedup and reduce that growth to 1,524 bytes.

No rejected implementation or dispatch remains.

## Retained-process CPU results

Fixtures are built once and each Boolean is repeated in one process. Runs are
serialized, pinned to CPU 9, and use reverse-order parent/candidate brackets.
Retained rows use two 51-operation measurements per revision, generated rows
two 101-operation measurements, and box rows two 501-operation measurements.
Values are means per operation.

| Fixture / policy | Task clock | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Retained / `STRICT` parent | 40.536 ms | 170,307,971 | 462,321,292 | 79,316,622 | 737,967 | 1,244,410 |
| Retained / `STRICT` candidate | 41.094 ms (+1.378%) | 172,389,632 (+1.222%) | 458,988,983 (-0.721%) | 78,711,185 (-0.763%) | 738,677 (+0.096%) | 1,269,895 (+2.048%) |
| Retained / `APPROXIMATE_512` parent | 41.269 ms | 173,169,761 | 462,318,194 | 79,315,830 | 745,803 | 1,254,911 |
| Retained / `APPROXIMATE_512` candidate | 40.925 ms (-0.834%) | 171,754,332 (-0.817%) | 459,005,328 (-0.717%) | 78,714,460 (-0.758%) | 739,152 (-0.892%) | 1,275,953 (+1.677%) |
| Generated / `STRICT` parent | 12.957 ms | 54,343,063 | 147,811,849 | 24,517,390 | 243,095 | 425,151 |
| Generated / `STRICT` candidate | 12.954 ms (-0.026%) | 54,294,198 (-0.090%) | 147,139,482 (-0.455%) | 24,391,633 (-0.513%) | 244,535 (+0.592%) | 450,440 (+5.948%) |
| Generated / `APPROXIMATE_512` parent | 13.565 ms | 56,320,508 | 147,819,608 | 24,519,318 | 243,280 | 434,957 |
| Generated / `APPROXIMATE_512` candidate | 13.044 ms (-3.841%) | 54,395,281 (-3.418%) | 147,139,319 (-0.460%) | 24,391,579 (-0.521%) | 242,240 (-0.427%) | 442,105 (+1.643%) |
| Boxes / `STRICT` parent | 1.8417 ms | 7,721,315 | 19,075,028 | 3,252,106 | 23,995 | 90,146 |
| Boxes / `STRICT` candidate | 1.8421 ms (+0.023%) | 7,744,049 (+0.294%) | 19,008,811 (-0.347%) | 3,238,314 (-0.424%) | 23,970 (-0.105%) | 85,373 (-5.295%) |
| Boxes / `APPROXIMATE_512` parent | 1.8416 ms | 7,716,030 | 19,074,752 | 3,252,053 | 24,241 | 90,165 |
| Boxes / `APPROXIMATE_512` candidate | 1.8368 ms (-0.262%) | 7,694,064 (-0.285%) | 19,007,728 (-0.351%) | 3,238,062 (-0.430%) | 23,966 (-1.133%) | 85,598 (-5.065%) |

Instructions and branches improve deterministically in every fixture and under
both policies. Retained strict task clock/cycles and the miss counters show the
usual frequency/cache variation; the reverse policy row and the longer
Criterion control do not reproduce a regression.

## Criterion, historical, and competitive controls

A clean temporary tree at the direct parent revisions avoids the contaminated
Criterion baseline left by a rejected cache experiment. In the final adjacent
candidate/parent/candidate bracket, generated projective union centers were
6.9898 ms, 7.0083 ms, and 6.9799 ms. The candidate bracket mean is 6.9849 ms,
0.335% below the direct parent; the 95% intervals overlap, so this row is timing
neutral rather than a claimed statistically significant speedup. Its 0.455--
0.460% large-generated and 0.837% competitive-control instruction reductions
are the stable evidence.

Current competitive union controls on the same pinned session are:

| Engine | Generated projective union | Relative to Hypermesh |
| --- | ---: | ---: |
| Hypermesh exact candidate | 6.9849 ms bracket mean | 1.00x |
| boolmesh | 748.12 us | Hypermesh 9.34x slower |
| manifold-rust | 658.77 us | Hypermesh 10.60x slower |

The competitors do not preserve Hyperreal coordinates, expose Hyperlimit
policy selection, or return Hypermesh's certification marker. They are
throughput controls, not exactness oracles. The preceding stored Hypermesh row
was 7.0621 ms, so the current absolute value is 1.09% lower across sessions.

Against the frozen historical retained row (944.8 ms, 67.74 MiB peak heap,
5,020,891 allocations, and about 82.5 MiB RSS), current strict retained work is
41.094 ms, 12.71 MiB, 454,003 allocations, and about 22.14 MiB RSS: a
directional 95.65% runtime, 81.24% peak-heap, 90.96% allocation, and 73.16% RSS
reduction. Fixture and timing differences make this a trend, not a direct A/B.

## Large-fixture heap

Heaptrack records fixture construction plus one complete immediate union.
Recorder totals and reconstructed peaks are unchanged from the direct parent:

| Fixture / policy | Allocations | Recorder temporary | Reconstructed temporary | Peak heap | Candidate RSS |
| --- | ---: | ---: | ---: | ---: | ---: |
| Retained / `STRICT` | 454,003 | 28,608 | 28,734 | 12.71 MiB | 22.14 MiB |
| Retained / `APPROXIMATE_512` | 454,003 | 28,608 | 28,734 | 12.71 MiB | 22.20 MiB |
| Generated / `STRICT` | 200,755 | 10,316 | 10,358 | 11.66 MiB | 23.26 MiB |
| Generated / `APPROXIMATE_512` | 200,755 | 10,316 | 10,358 | 11.66 MiB | 23.27 MiB |

Direct-parent strict RSS in the same recordings is 21.98 MiB retained and
23.27 MiB generated. RSS includes Heaptrack and process mapping noise; heap,
allocation, and temporary-allocation controls are exact matches.

## Native and WASM linked code

Native code is linked `.text`; WASM code is `wasm-opt -Oz`. The canonical
dependency-only consumers compare the committed sparse-cofactor checkpoint
with this candidate:

| Consumer | Profile / format | Parent | Candidate | Movement |
| --- | --- | ---: | ---: | ---: |
| General | Release native | 4,032,492 | 4,033,596 | +1,104 / +0.0274% |
| Immediate | Release native | 4,066,108 | 4,067,212 | +1,104 / +0.0272% |
| General | Release WASM | 2,707,649 | 2,709,462 | +1,813 / +0.0670% |
| Immediate | Release WASM | 2,722,688 | 2,724,501 | +1,813 / +0.0666% |
| General | Size native | 1,855,642 | 1,855,658 | +16 / +0.0009% |
| Immediate | Size native | 1,868,142 | 1,868,158 | +16 / +0.0009% |
| General | Size WASM | 1,151,698 | 1,152,464 | +766 / +0.0665% |
| Immediate | Size WASM | 1,162,667 | 1,163,433 | +766 / +0.0659% |

The repeated-probe release executable grows 912 file bytes and 1,524 `.text`
bytes. Hyperreal production/test/API classification changes by +100/-6;
Hypermesh production changes by +18/-1. No carrier grows.

The call-graph utility reports:

| Scope | Parent | Candidate | Movement |
| --- | ---: | ---: | ---: |
| Isolated Hypermesh | 7,994 nodes / 19,633 edges | 7,998 / 19,638 | +4 / +5 |
| Five Hyper crates | 19,638 nodes / 39,216 edges | 19,648 / 39,228 | +10 / +12 |

The added nodes are the paired proof API, factored arithmetic kernels, focused
tests, and four consumer calls. No second policy, topology, or fallback spine
is introduced.

## Validation

The final five-crate stack passes:

- default, no-default, and all-feature tests in Hyperreal, Hyperlattice,
  Hyperlimit, Hypertri, and Hypermesh;
- Hypermesh's 1,053 default/no-default and 1,054 all-feature unit tests plus all
  integration, policy, regression, and doctest surfaces;
- warning-denied all-target Clippy and warning-denied rustdoc under all and
  no-default features in all five crates;
- formatting, every fuzz workspace, and all-feature benchmark compilation in
  all five crates;
- Hypermesh's canonical WASM size harness;
- focused AddressSanitizer runs for Hyperreal's randomized certified line
  oracle and both Hypermesh projected-crossing policy tests; and
- opt-in release YeahRight checks for every Boolean operation's exact boundary
  output and polygon/immediate API consistency.

Representative commands:

```text
cargo test --locked --no-fail-fast
cargo test --locked --no-default-features --no-fail-fast
cargo test --locked --all-features --no-fail-fast
cargo clippy --locked --all-targets --all-features -- -D warnings
cargo clippy --locked --all-targets --no-default-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --no-default-features
cargo fmt --all -- --check
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo bench --locked --all-features --no-run

CARGO_TARGET_DIR=/tmp/<crate>-line-pair-asan \
  RUSTFLAGS='-Zsanitizer=address' ASAN_OPTIONS=detect_leaks=0 \
  cargo +nightly test --locked --target x86_64-unknown-linux-gnu <filter> --lib

./benchmarks/size-harness/measure.sh default
```
