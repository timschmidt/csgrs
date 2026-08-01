# Direct crossing-projection directions checkpoint

Date: 2026-08-01

Hypermesh direct parent: `c3ef5cfb`

Hypermesh implementation: `188d4d93ff9b51cdde4245ea7b82983058ab34fe`

Dependencies:

- Hyperreal `7262d3037d056c9fee83b07d6d43cc3d7bf65277`
- Hyperlattice `d11ca2f0e825d8e26048cfda5d1101df21dcfef0`
- Hyperlimit `3e5d8816cd32bba46f48e0c6c13ab7a9da227784`
- Hypertri `c47601266e0b9b17d0c5a0764fa22b18168ada73`

This checkpoint expands the two three-coordinate floating direction arrays in
`approximate_projection_axis` directly. It removes the generic array-map
machinery from a hot crossing selector while performing the same six binary64
subtractions in the same coordinate order. The existing normal calculation,
finite/nonzero filtering, magnitude comparison, and last-equal-axis tie rule
are unchanged.

## Exactness and policy invariant

The approximate normal only chooses which one of three complete exact
projections is attempted first. It never proves or rejects a crossing. Every
surviving projection still executes exact projected orientation for both
segments and exact 3-D coplanarity before an event can be constructed.

The direct expansion preserves all selector paths:

- finite nonzero normal components retain the same magnitude ordering;
- equal magnitudes retain `Iterator::max_by`'s last-equal-axis selection;
- zero and non-finite components remain excluded;
- an all-zero, all-non-finite, or parallel normal still returns `None`;
- `None` still tries all exact projections in their fixed fallback order;
- a selected axis changes only the first exact attempt, not the complete set;
- the symbolic no-enclosure path, shared-endpoint handling, exact
  coplanarity, intersection construction, repair, and closure checks are
  unchanged; and
- all parent/candidate and both-policy fixture results have identical topology
  and `MeshCertainty::Certified`.

`STRICT` therefore continues to accept only structural, filtered, or exact
decisions. `APPROXIMATE_512` still reaches Hyperlimit's terminal 512-bit
evaluation only after the unchanged certified/exact stack is exhausted. The
selector neither reads policy nor changes certainty.

A focused regression covers an equal-magnitude tie, one usable component next
to an overflowed component, no usable component, and parallel directions.
There is no epsilon, approximate equality, allocation, cache, carrier field,
pass limit, public API, alternate topology path, or compatibility shim.

## Work and profile characterization

The selector is entered once for every projected crossing survivor: 17,675
times on the retained arrangement, 2,854 times on generated YeahRight-8, and
165 times on boxes-3072. Parent and candidate both perform six subtractions
and the same cross-product and ordering work per call. Only the two generic
`[usize; 3]::map` state machines are removed.

The final retained profile uses 30 operations at 1,999 Hz, records 2,166
samples with zero lost, and has an event count of 4,479,318,857. The two
generic array-map symbols that accounted for 0.61% and 0.05% of the direct
parent receive no candidate samples. `split_edge_crossing_events` moves from
4.99% to 5.28%, the paired certified line-sign kernel from 2.29% to 2.38%, and
mixed-width GCD from 5.77% to 5.02%. Those whole-symbol movements are sampling
noise around unrelated retained work; serialized counters and the disjoint
Criterion intervals are the retention gates.

## Retained-process CPU results

Each fixture is built once and its Boolean union is repeated in one process.
Runs are serialized and pinned to CPU 9 in reverse-order parent/candidate
brackets. Values are the mean per operation of two measurements per revision.
Every row reports the same certified topology.

| Fixture / policy | Repetitions | Task clock | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Retained / `STRICT` parent | 51 | 35.831 ms | 150,975,138 | 419,745,995 | 71,307,674 | 678,032 | 1,219,632 |
| Retained / `STRICT` candidate | 51 | 35.645 ms (-0.520%) | 150,595,336 (-0.252%) | 418,296,807 (-0.345%) | 71,132,929 (-0.245%) | 682,917 (+0.720%) | 1,202,948 (-1.368%) |
| Retained / `APPROXIMATE_512` parent | 101 | 36.192 ms | 152,212,146 | 419,452,663 | 71,249,126 | 673,609 | 1,228,543 |
| Retained / `APPROXIMATE_512` candidate | 101 | 35.743 ms (-1.240%) | 150,900,339 (-0.862%) | 417,997,543 (-0.347%) | 71,073,703 (-0.246%) | 683,042 (+1.400%) | 1,205,760 (-1.855%) |
| Generated / `STRICT` parent | 301 | 12.183 ms | 51,475,359 | 139,288,493 | 23,011,510 | 231,729 | 440,060 |
| Generated / `STRICT` candidate | 301 | 12.073 ms (-0.905%) | 51,018,961 (-0.887%) | 139,040,217 (-0.178%) | 22,983,782 (-0.120%) | 231,546 (-0.079%) | 431,933 (-1.847%) |
| Generated / `APPROXIMATE_512` parent | 301 | 12.269 ms | 51,650,468 | 139,289,037 | 23,011,694 | 231,015 | 440,995 |
| Generated / `APPROXIMATE_512` candidate | 301 | 12.085 ms (-1.499%) | 51,001,329 (-1.257%) | 139,036,673 (-0.181%) | 22,982,985 (-0.125%) | 229,658 (-0.587%) | 430,643 (-2.347%) |
| Boxes / `STRICT` parent | 2,001 | 1.7907 ms | 7,585,381 | 18,874,781 | 3,206,412 | 21,269 | 83,837 |
| Boxes / `STRICT` candidate | 2,001 | 1.7827 ms (-0.446%) | 7,550,461 (-0.460%) | 18,861,532 (-0.070%) | 3,204,959 (-0.045%) | 22,231 (+4.524%) | 88,561 (+5.634%) |
| Boxes / `APPROXIMATE_512` parent | 4,001 | 1.8295 ms | 7,598,534 | 18,870,175 | 3,205,451 | 22,064 | 85,257 |
| Boxes / `APPROXIMATE_512` candidate | 4,001 | 1.8231 ms (-0.348%) | 7,611,105 (+0.165%) | 18,857,368 (-0.068%) | 3,204,167 (-0.040%) | 21,943 (-0.549%) | 87,916 (+3.120%) |

Instructions and branches fall on all three fixtures under both policies.
Five of six task-clock rows and five of six cycle rows improve; the small
approximate box cycle increase and small-fixture miss movements are layout and
frequency noise. The generated and retained rows, which execute substantially
more crossing-selection work, improve every primary metric.

## Criterion, historical, and competitive controls

A candidate/parent/candidate Criterion bracket on generated projective union
reports candidate intervals of 6.4657--6.5026 ms and 6.5050--6.5206 ms around
a parent interval of 6.5494--6.5662 ms. Both candidate upper bounds are below
the parent lower bound. Candidate centers are 6.4795 and 6.5142 ms; their
6.49685 ms mean is 0.910% below the 6.5565 ms direct parent and 1.012% below
the preceding checkpoint's 6.56325 ms bracket mean.

The same stored competitive controls are 749.98 us for boolmesh and 657.43 us
for manifold-rust. Hypermesh is therefore 8.66x and 9.88x slower on
throughput. Those engines do not preserve Hyperreal coordinates, expose
Hyperlimit policy, or report certification and are not exactness oracles.

Against the frozen historical retained row of 944.8 ms, current strict
retained work is 35.645 ms, a directional reduction of 96.23%. Fixture and
measurement evolution make that historical result a trend rather than a
direct A/B.

## Large-fixture heap

Heaptrack records fixture construction plus one complete immediate union.
Parent and candidate were invoked through equal-length executable names;
without this normalization, the longer candidate pathname itself causes one
extra process-startup allocation. With that measurement artifact removed,
allocation counts, recorder and reconstructed temporary counts, and peak heap
match the direct parent exactly. Both policies also match.

| Fixture / policy | Allocations | Recorder temporary | Reconstructed temporary | Peak heap | Direct max RSS |
| --- | ---: | ---: | ---: | ---: | ---: |
| Retained / `STRICT` | 454,003 | 28,608 | 28,734 | 12.71 MiB | 18.38 MiB |
| Retained / `APPROXIMATE_512` | 454,003 | 28,608 | 28,734 | 12.71 MiB | 18.25 MiB |
| Generated / `STRICT` | 200,755 | 10,316 | 10,358 | 11.66 MiB | 16.75 MiB |
| Generated / `APPROXIMATE_512` | 200,755 | 10,316 | 10,358 | 11.66 MiB | 16.94 MiB |

## Source, linked code, and call graph

Production changes are 12 insertions and two deletions; the focused regression
adds 27 lines. No public API or production carrier changes. Canonical linked
code relative to the direct parent shrinks in every measured cell:

| Consumer | Profile / format | Parent | Candidate | Movement |
| --- | --- | ---: | ---: | ---: |
| General | Release native text | 4,033,452 | 4,033,372 | -80 / -0.0020% |
| Immediate | Release native text | 4,067,068 | 4,066,988 | -80 / -0.0020% |
| General | Release WASM `wasm-opt -Oz` | 2,711,118 | 2,711,116 | -2 / -0.0001% |
| Immediate | Release WASM `wasm-opt -Oz` | 2,726,157 | 2,726,153 | -4 / -0.0001% |
| General | Size native text | 1,855,794 | 1,855,602 | -192 / -0.0103% |
| Immediate | Size native text | 1,868,302 | 1,868,094 | -208 / -0.0111% |
| General | Size WASM `wasm-opt -Oz` | 1,152,648 | 1,152,378 | -270 / -0.0234% |
| Immediate | Size WASM `wasm-opt -Oz` | 1,163,617 | 1,163,348 | -269 / -0.0231% |

The repeated release probe adds 832 file bytes, 1,608 `.text` bytes, and 2,496
BSS bytes because the surrounding example's optimized layout changes. The
canonical general/immediate consumers are the linked-size gate and all eight
of those cells improve.

The call-graph utility reports 8,008 nodes / 19,659 edges for isolated
Hypermesh and 19,658 / 39,249 for the five-crate scope. Relative to the direct
parent this is +2 nodes / +1 edge in either scope, exactly the new test and its
local point-construction closure. The production expansion introduces no
function, policy edge, terminal, or topology spine.

## Rejected experiment

An exploratory form also replaced `filter(...).max_by(...)` with a hand-written
best-axis loop. Although retained instructions fell 0.394%, branch misses rose
3.856%, cycles rose 0.286%, and task clock rose 0.261%. That loop was fully
removed. The retained form changes only direction-array construction.

## Validation

The committed Hypermesh source passes:

- default, no-default, and all-feature tests: 1,056 / 1,056 / 1,057 unit tests
  plus all integration, policy, regression, and doctest surfaces;
- warning-denied all-target Clippy and warning-denied rustdoc under all and
  no-default features;
- formatting, every fuzz binary check, all-feature benchmark compilation, and
  the canonical native/WASM release/size harness;
- AddressSanitizer runs of the new tie/non-finite selector regression and the
  both-policy paired-filter topology regression; and
- opt-in release YeahRight checks for every Boolean operation's exact closed
  boundary and polygon/immediate API consistency.

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
./benchmarks/size-harness/measure.sh default

CARGO_TARGET_DIR=/tmp/hypermesh-direct-projection-asan \
  RUSTFLAGS='-Zsanitizer=address' ASAN_OPTIONS=detect_leaks=0 \
  cargo +nightly test --locked --target x86_64-unknown-linux-gnu \
  --lib <full-test-name> -- --exact

YEAHRIGHT_BENCH=1 cargo test --locked --release --test competitive \
  <exact-test-name> -- --ignored --exact
```
