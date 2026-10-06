# Left-edge certified-query reuse checkpoint

Date: 2026-08-01

Hypermesh direct parent: `e2662ba2`

Hypermesh implementation: `eb17c478dbf2df676ea8d45331646e3af2ec241b`

Dependencies:

- Hyperreal `7262d3037d056c9fee83b07d6d43cc3d7bf65277`
- Hyperlattice `d11ca2f0e825d8e26048cfda5d1101df21dcfef0`
- Hyperlimit `3e5d8816cd32bba46f48e0c6c13ab7a9da227784`
- Hypertri `c47601266e0b9b17d0c5a0764fa22b18168ada73`

This checkpoint constructs the two certified rational queries for a crossing
sweep's left edge once per outer iteration instead of once for every surviving
right edge. Right-edge queries remain local to each survivor. The projected
predicate now receives four borrowed queries rather than copying them into one
temporary four-query array.

## Exactness and policy invariant

`RationalPoint3Query::from_certified_enclosures` is a pure, policy-free filter
constructor. Its result depends only on one immutable exact-rational output
vertex and its certified outward binary64 enclosures. For every fixed left
edge, rebuilding either endpoint query for a later right edge must therefore
return the same bits and the same `Option` result.

If both left queries exist, the candidate passes references to those exact
same filter values. If either construction returns `None`, the parent's
four-query construction also returns `None` for every pair sharing that left
edge; both revisions then use the unchanged exact-rational orientation path.
Right-query failure has the same local exact fallback as before. Borrowing the
queries changes neither their arithmetic nor their lifetime beyond the outer
sweep iteration.

Consequently:

- `STRICT` still admits only structural, filtered, or exact decisions;
- `APPROXIMATE_512` still reaches Hyperlimit's terminal 512-bit evaluation
  only after the unchanged certification/exact stack is exhausted;
- query construction cannot consume policy or change mesh certainty;
- the symbolic no-enclosure path still supplies no rational-query bundle and
  is unchanged;
- event construction, exact coplanarity, intersection construction, repair,
  and closure certification are unchanged; and
- every measured parent/candidate and both-policy result has identical
  topology and `MeshCertainty::Certified`.

There is no epsilon, approximate equality, new allocation, retained cache,
carrier field, pass/candidate limit, alternate topology path, policy-free
entry point, or compatibility shim.

## Work characterization

The retained controls have no crossing events, so every projected survivor is
a rejection and no changed construction work can conceal a topology change.
For a closed triangular output, the unique edge count is `3F/2`. The parent
constructs four queries per projected survivor; the candidate constructs two
per unique left edge and two per survivor.

| Fixture | Unique edges | Projected survivors | Parent query constructions | Candidate | Reduction |
| --- | ---: | ---: | ---: | ---: | ---: |
| Retained 4,524 triangles | 1,869 | 17,675 | 70,700 | 39,088 | -44.713% |
| Generated 13,452 triangles | 456 | 2,854 | 11,416 | 6,620 | -42.011% |
| Boxes 6,144 triangles | 75 | 165 | 660 | 480 | -27.273% |

The final retained profile uses 30 operations at 1,999 Hz, records 2,168
samples with zero lost and an event count of 4,517,347,295. Self time in
`split_edge_crossing_events` moves from 5.61% to 4.99%; the paired certified
line-sign kernel moves from 2.41% to 2.29%. Mixed-width GCD is now the leading
symbol at 5.77%. Sampling is descriptive; serialized counters are the
retention gate.

## Retained-process CPU results

Each fixture is built once and its Boolean union is repeated in one process.
Runs are serialized and pinned to CPU 9 in reverse-order parent/candidate
brackets. Values are the mean per operation of two measurements per revision.
Longer generated and box rows suppress fixture-construction and timing noise.

| Fixture / policy | Repetitions | Task clock | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Retained / `STRICT` parent | 51 | 37.439 ms | 156,358,323 | 422,858,233 | 71,874,618 | 680,724 | 1,247,711 |
| Retained / `STRICT` candidate | 51 | 36.492 ms (-2.529%) | 152,718,700 (-2.328%) | 419,743,486 (-0.737%) | 71,307,135 (-0.790%) | 677,829 (-0.425%) | 1,226,545 (-1.696%) |
| Retained / `APPROXIMATE_512` parent | 51 | 37.305 ms | 154,364,785 | 422,871,453 | 71,877,486 | 687,381 | 1,248,658 |
| Retained / `APPROXIMATE_512` candidate | 51 | 36.791 ms (-1.377%) | 152,286,340 (-1.346%) | 419,744,974 (-0.739%) | 71,306,985 (-0.794%) | 678,009 (-1.363%) | 1,229,366 (-1.545%) |
| Generated / `STRICT` parent | 301 | 12.638 ms | 52,348,103 | 139,766,691 | 23,101,448 | 230,273 | 440,890 |
| Generated / `STRICT` candidate | 301 | 12.485 ms (-1.209%) | 51,764,970 (-1.114%) | 139,290,139 (-0.341%) | 23,011,875 (-0.388%) | 231,937 (+0.723%) | 439,555 (-0.303%) |
| Generated / `APPROXIMATE_512` parent | 301 | 12.259 ms | 51,708,114 | 139,768,285 | 23,101,714 | 230,369 | 443,660 |
| Generated / `APPROXIMATE_512` candidate | 301 | 12.205 ms (-0.438%) | 51,485,039 (-0.431%) | 139,290,112 (-0.342%) | 23,011,977 (-0.388%) | 230,596 (+0.098%) | 443,485 (-0.039%) |
| Boxes / `STRICT` parent | 1,001 | 1.8162 ms | 7,607,363 | 18,897,905 | 3,210,850 | 22,790 | 85,398 |
| Boxes / `STRICT` candidate | 1,001 | 1.7937 ms (-1.237%) | 7,538,877 (-0.900%) | 18,882,646 (-0.081%) | 3,207,952 (-0.090%) | 22,670 (-0.528%) | 85,780 (+0.447%) |
| Boxes / `APPROXIMATE_512` parent | 1,001 | 1.7902 ms | 7,541,138 | 18,897,780 | 3,210,838 | 22,791 | 84,969 |
| Boxes / `APPROXIMATE_512` candidate | 1,001 | 1.7804 ms (-0.544%) | 7,518,553 (-0.299%) | 18,881,978 (-0.084%) | 3,207,822 (-0.094%) | 22,149 (-2.818%) | 85,254 (+0.336%) |

Instructions and branches fall on every fixture and policy. The small
generated branch-miss and box cache-miss movements are layout noise; extended
task-clock and cycle brackets remain favorable. Short exploratory rows with
contradictory timing but identical static work were discarded and rerun at
longer duration rather than selected into this table.

## Criterion, historical, and competitive controls

A stable candidate/parent/candidate Criterion bracket on generated projective
union reports candidate intervals of 6.5702--6.6181 ms and
6.5276--6.5577 ms around a 6.5674--6.6096 ms parent. Candidate centers are
6.5895 and 6.5370 ms; their 6.5633 ms mean is 0.341% below the 6.5857 ms
parent. Confidence intervals overlap, so deterministic mesh counters remain
the retention gate.

The same pinned competitive session's current controls remain 749.98 us for
boolmesh and 657.43 us for manifold-rust. Hypermesh is therefore 8.75x and
9.98x slower on throughput. Those engines do not preserve Hyperreal
coordinates, expose Hyperlimit policy, or report certification and are not
exactness oracles. The preceding stored Hypermesh center was 6.5823 ms; the
new bracket mean is 0.289% lower across sessions.

Against the frozen historical retained row of 944.8 ms, current strict
retained work is 36.492 ms, a directional reduction of 96.14%. Fixture and
measurement evolution make that historical result a trend rather than a
direct A/B.

## Large-fixture heap

Heaptrack records fixture construction plus one complete immediate union. The
change adds no allocation call. Total allocations and peak heap exactly match
the direct parent and the preceding checkpoint. Recorder/reconstruction
temporary classification falls by one in each fixture, which is lifetime
classification noise rather than an allocation-count change.

| Fixture / policy | Allocations | Recorder temporary | Reconstructed temporary | Peak heap | Direct max RSS |
| --- | ---: | ---: | ---: | ---: | ---: |
| Retained / `STRICT` | 454,003 | 28,608 | 28,734 | 12.71 MiB | 18.17 MiB |
| Retained / `APPROXIMATE_512` | 454,003 | 28,608 | 28,734 | 12.71 MiB | 18.26 MiB |
| Generated / `STRICT` | 200,755 | 10,316 | 10,358 | 11.66 MiB | 17.14 MiB |
| Generated / `APPROXIMATE_512` | 200,755 | 10,316 | 10,358 | 11.66 MiB | 17.06 MiB |

## Source, linked code, and call graph

The production change is one file, 31 insertions and 19 deletions. No public
API or production carrier changes. Canonical linked code relative to the
direct parent is:

| Consumer | Profile / format | Parent | Candidate | Movement |
| --- | --- | ---: | ---: | ---: |
| General | Release native text | 4,033,388 | 4,033,452 | +64 / +0.0016% |
| Immediate | Release native text | 4,067,004 | 4,067,068 | +64 / +0.0016% |
| General | Release WASM `wasm-opt -Oz` | 2,710,939 | 2,711,118 | +179 / +0.0066% |
| Immediate | Release WASM `wasm-opt -Oz` | 2,725,978 | 2,726,157 | +179 / +0.0066% |
| General | Size native text | 1,856,058 | 1,855,794 | -264 / -0.0142% |
| Immediate | Size native text | 1,868,566 | 1,868,302 | -264 / -0.0141% |
| General | Size WASM `wasm-opt -Oz` | 1,152,641 | 1,152,648 | +7 / +0.0006% |
| Immediate | Size WASM `wasm-opt -Oz` | 1,163,610 | 1,163,617 | +7 / +0.0006% |

The repeated release probe adds 896 file bytes and 512 `.text` bytes; its BSS
falls 528 bytes. Performance has priority, and the deterministic retained and
generated reductions comfortably exceed these linked movements.

The call-graph utility reports 8,006 nodes / 19,658 edges for isolated
Hypermesh and 19,656 / 39,248 for the five-crate scope. Relative to the direct
parent this is +1 node / +3 edges in either scope, from the left/right query
closures and borrowed bundle assembly. There is no new policy, terminal, or
topology spine.

## Validation

The committed Hypermesh source passes:

- default, no-default, and all-feature tests: 1,055 / 1,055 / 1,056 unit tests
  plus all integration, policy, regression, and doctest surfaces;
- warning-denied all-target Clippy and warning-denied rustdoc under all and
  no-default features;
- formatting, every fuzz binary check, all-feature benchmark compilation, and
  the canonical native/WASM release/size harness;
- AddressSanitizer runs of both-policy paired-filter topology and the rounded
  enclosure separation regression; and
- opt-in release YeahRight checks for every Boolean operation's exact closed
  boundary and polygon/immediate API consistency.

The dependency revisions are unchanged from the preceding fully validated
five-crate checkpoints and are rebuilt through Hypermesh's feature, benchmark,
size, and sanitizer surfaces.

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

CARGO_TARGET_DIR=/tmp/hypermesh-left-query-asan \
  RUSTFLAGS='-Zsanitizer=address' ASAN_OPTIONS=detect_leaks=0 \
  cargo +nightly test --locked --target x86_64-unknown-linux-gnu \
  --lib <full-test-name> -- --exact

YEAHRIGHT_BENCH=1 cargo test --locked --release --test competitive \
  <exact-test-name> -- --ignored --exact
```
