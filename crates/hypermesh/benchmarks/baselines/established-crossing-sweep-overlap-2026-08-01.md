# Established crossing-sweep overlap checkpoint

Date: 2026-08-01

Hypermesh direct parent: `d93e23db` (implementation `188d4d93`)

Hypermesh implementation: `7a90f764951b24d1877fee156455bd2deea451ba`

Dependencies:

- Hyperreal `7262d3037d056c9fee83b07d6d43cc3d7bf65277`
- Hyperlattice `d11ca2f0e825d8e26048cfda5d1101df21dcfef0`
- Hyperlimit `3e5d8816cd32bba46f48e0c6c13ab7a9da227784`
- Hypertri `c47601266e0b9b17d0c5a0764fa22b18168ada73`

This checkpoint stops the allocation-free output-crossing scan from testing
the sweep-axis enclosure overlap twice. The scan already sorts every edge by
its finite outward-enclosure minimum on the selected sweep axis. For a right
edge reached after the break check:

1. sorting establishes `right_min >= left_min`;
2. the failed break establishes `right_min <= left_max`; and
3. interval construction establishes `right_max >= right_min`.

The two sweep intervals therefore overlap. The direct overlap helper tests only
the other two axes. The cached tier for at least 1,024 edges deliberately keeps
its existing compiler-friendly three-axis check.

## Exactness and policy invariant

The changed check is a conservative broad-phase test over certified outward
binary64 enclosures. It may reject a pair only when one of the two remaining
axis intervals is disjoint. Every survivor still executes the complete exact
projected-orientation predicate and exact 3-D coplanarity predicate before an
event can be constructed.

All potentially relevant paths remain explicit:

- each of the three possible sweep axes uses the corresponding two other axes;
- equality at either outward interval endpoint remains overlap;
- the break condition and its sorted-minimum premise are unchanged;
- the cached-enclosure tier remains unchanged;
- the no-enclosure symbolic tier retains its exact bound comparisons;
- shared endpoints still bypass crossing construction only after broad-phase
  survival;
- exact separation hidden by overlapping binary64 bounds still reaches and is
  rejected by the exact projected predicate;
- exact intersection construction, interning, repair, closure validation, and
  triangulation are unchanged; and
- there is no epsilon, approximate equality, arbitrary pass limit, cache,
  allocation, carrier field, public API, or compatibility shim.

`STRICT` therefore continues to accept only structural, filtered, or exact
decisions. `APPROXIMATE_512` still reaches Hyperlimit's terminal 512-bit
evaluation only after the unchanged certified/exact stack is exhausted. The
overlap schedule neither reads policy nor changes `MeshCertainty`.

The focused regression constructs a sorted, sweep-overlapping pair for every
sweep axis, separates it on another axis, and proves that the two-axis helper
rejects it. Existing regressions retain exact separation hidden by floating
bounds and the cached-tier path.

## Dynamic path characterization

A temporary diagnostic counter was compiled into a measurement-only binary and
then fully removed. One strict operation reaches the new direct helper 21,728
times on generated YeahRight-8 and 1,605 times on boxes-3072. Each call is an
opportunity to avoid the redundant sweep-axis minimum, maximum, and comparisons;
short-circuiting means the old helper did not execute all of those operations
on every call. The 1,869-edge retained arrangement reports zero direct-helper
calls because it correctly uses the unchanged cached tier.

| Fixture | Input triangles | Output vertices | Output triangles | Direct checks |
| --- | ---: | ---: | ---: | ---: |
| Retained arrangement | 4,524 | 625 | 1,246 | 0 |
| Generated YeahRight-8 | 13,452 | 154 | 304 | 21,728 |
| Boxes-3072 | 6,144 | 27 | 50 | 1,605 |

Every parent/candidate and both-policy output has the topology above and
`MeshCertainty::Certified`.

## Retained-process CPU results

Each fixture is built once and its Boolean union is repeated in one process.
Runs are serialized and pinned to CPU 9 in parent/candidate/candidate/parent
order. Values are the mean per operation of two measurements per revision.

| Fixture / policy | Repetitions | Task clock | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Retained / `STRICT` parent | 51 | 35.4287 ms | 149,711,961 | 418,293,958 | 71,128,441 | 679,355 | 1,210,139 |
| Retained / `STRICT` candidate | 51 | 35.2019 ms (-0.640%) | 148,715,166 (-0.666%) | 418,147,160 (-0.035%) | 71,134,585 (+0.009%) | 676,037 (-0.488%) | 1,233,403 (+1.922%) |
| Retained / `APPROXIMATE_512` parent | 101 | 35.5973 ms | 150,581,468 | 418,010,781 | 71,072,901 | 672,076 | 1,222,413 |
| Retained / `APPROXIMATE_512` candidate | 101 | 34.9924 ms (-1.699%) | 148,198,044 (-1.583%) | 417,852,509 (-0.038%) | 71,075,883 (+0.004%) | 672,358 (+0.042%) | 1,210,263 (-0.994%) |
| Generated / `STRICT` parent | 301 | 12.0478 ms | 50,937,000 | 139,056,632 | 22,994,791 | 229,711 | 427,410 |
| Generated / `STRICT` candidate | 301 | 11.9953 ms (-0.436%) | 50,701,494 (-0.462%) | 138,879,006 (-0.128%) | 22,960,849 (-0.148%) | 229,978 (+0.116%) | 425,875 (-0.359%) |
| Generated / `APPROXIMATE_512` parent | 301 | 12.0471 ms | 50,934,763 | 139,060,900 | 22,995,710 | 228,710 | 431,955 |
| Generated / `APPROXIMATE_512` candidate | 301 | 11.9698 ms (-0.641%) | 50,608,109 (-0.641%) | 138,878,252 (-0.131%) | 22,960,356 (-0.154%) | 230,248 (+0.673%) | 428,862 (-0.716%) |
| Boxes / `STRICT` parent | 4,001 | 1.76101 ms | 7,461,255 | 18,855,138 | 3,203,657 | 21,823 | 81,769 |
| Boxes / `STRICT` candidate | 4,001 | 1.78530 ms (+1.380%) | 7,513,322 (+0.698%) | 18,829,650 (-0.135%) | 3,201,111 (-0.079%) | 22,151 (+1.502%) | 84,261 (+3.047%) |
| Boxes / `APPROXIMATE_512` parent | 2,001 | 1.76455 ms | 7,480,231 | 18,860,791 | 3,204,753 | 21,958 | 83,703 |
| Boxes / `APPROXIMATE_512` candidate | 2,001 | 1.77899 ms (+0.818%) | 7,538,771 (+0.783%) | 18,837,449 (-0.124%) | 3,202,201 (-0.080%) | 22,093 (+0.617%) | 84,182 (+0.571%) |

The targeted generated path improves every primary metric under both policies:
instructions fall 0.128--0.131% and branches 0.148--0.154%. Box instructions
and branches also fall, but its task clock and cycles regress 0.82--1.38% and
0.70--0.78%. This is an explicitly retained layout tradeoff: performance has
priority over size, the representative general path and Criterion improve, and
the deterministic box work count still falls. The cached retained tier is
instruction-neutral within 0.04%, as expected; its timing movement is layout
and system variation rather than work removed by this change.

## Criterion, historical, and competitive controls

A candidate/parent/candidate Criterion bracket on generated projective union
reports candidate intervals of 6.4554--6.5174 ms and 6.4288--6.4505 ms around a
parent interval of 6.4901--6.5473 ms. Candidate centers are 6.4845 and 6.4421
ms; their 6.4633 ms mean is 0.832% below the 6.5175 ms parent. The first
candidate interval overlaps the parent and the second lies wholly below it, so
the repeated static instruction/branch reductions are retained as the primary
work gate. The result is 0.516% below the preceding checkpoint's 6.49685 ms
candidate mean.

The stored competitive controls are 749.98 us for boolmesh and 657.43 us for
manifold-rust. Hypermesh is therefore 8.62x and 9.83x slower on throughput.
Those engines do not retain Hyperreal coordinates, expose Hyperlimit policy, or
report certification and are not exactness oracles.

Against the frozen historical retained row of 944.8 ms, current strict retained
work is 35.20194 ms, a directional reduction of 96.27%. Fixture and measurement
evolution make that historical result a trend rather than a direct A/B.

## Profiles

The final retained profile uses 30 operations at 1,999 Hz, records 2,163
samples with zero lost, and has an event count of 4,479,930,524. Mixed-width
GCD is 5.01% self, `split_edge_crossing_events` is 4.59%, and the paired
certified line-sign kernel is 2.44%. The retained fixture uses the unchanged
cached tier, so this profile is an architecture priority map rather than a
direct-helper attribution.

A generated direct-tier profile uses 100 operations at 1,999 Hz, records 2,583
samples with zero lost, and has an event count of 5,341,254,994.
`split_edge_crossing_events` is 3.23% self, mixed-width GCD is 2.04%, and the
paired line-sign kernel is 1.14%. The two-axis helper is inlined. The profile
also contains 1.31% one-time fixture SHA-256 validation, which is excluded from
algorithmic interpretation.

## Large-fixture heap

Heaptrack records fixture construction plus one complete immediate union.
Parent and candidate use equal-length executable names so the path string does
not create a process-startup allocation artifact. Allocation counts, recorder
and reconstructed temporary counts, and peak heap match exactly under both
policies.

| Fixture / policy | Allocations | Recorder temporary | Reconstructed temporary | Peak heap | Candidate direct max RSS |
| --- | ---: | ---: | ---: | ---: | ---: |
| Retained / `STRICT` | 454,003 | 28,608 | 28,734 | 12.71 MiB | 18.29 MiB |
| Retained / `APPROXIMATE_512` | 454,003 | 28,608 | 28,734 | 12.71 MiB | 18.30 MiB |
| Generated / `STRICT` | 200,755 | 10,316 | 10,358 | 11.66 MiB | 17.06 MiB |
| Generated / `APPROXIMATE_512` | 200,755 | 10,316 | 10,358 | 11.66 MiB | 16.74 MiB |

Heaptrack's own peak RSS varies from 22.07 to 23.26 MiB across these invocations;
the allocator and reconstructed heap totals are the normalized memory gate.
No allocation, cache, or retained carrier was added.

## Source, linked code, and call graph

Production changes are 17 insertions and four deletions; the focused regression
adds 29 lines. No public API or production carrier changes. Canonical linked
code relative to the direct parent moves as follows:

| Consumer | Profile / format | Parent | Candidate | Movement |
| --- | --- | ---: | ---: | ---: |
| General | Release native text | 4,033,372 | 4,034,476 | +1,104 / +0.0274% |
| Immediate | Release native text | 4,066,988 | 4,068,092 | +1,104 / +0.0271% |
| General | Release WASM `wasm-opt -Oz` | 2,711,116 | 2,711,117 | +1 / +0.00004% |
| Immediate | Release WASM `wasm-opt -Oz` | 2,726,153 | 2,726,152 | -1 / -0.00004% |
| General | Size native text | 1,855,602 | 1,855,890 | +288 / +0.0155% |
| Immediate | Size native text | 1,868,094 | 1,868,390 | +296 / +0.0158% |
| General | Size WASM `wasm-opt -Oz` | 1,152,378 | 1,152,682 | +304 / +0.0264% |
| Immediate | Size WASM `wasm-opt -Oz` | 1,163,348 | 1,163,651 | +303 / +0.0260% |

The repeated-probe executable itself shrinks by 200 file bytes and 172 text
bytes. Its ELF BSS grows by 176 bytes, leaving total text/data/BSS four bytes
larger. Canonical native growth is at most 0.0274%, canonical WASM is nearly
flat, and the runtime improvement justifies the bounded linked-size cost.

The call-graph utility reports 8,011 nodes / 19,663 edges for isolated
Hypermesh and 19,661 / 39,253 for the five-crate scope. Relative to the parent,
both scopes move +3 nodes / +4 edges: one net production closure node and edge
from the renamed two-axis helper, plus the focused regression's two nodes and
three edges. There is no new policy, terminal, fallback, allocation, or
topology spine.

## Rejected experiments

Five alternatives were measured and fully removed:

- manual finite-only `if` min/max lowered generated instructions about 0.36%
  but increased box instructions about 0.46%, box branches about 1.07%, and
  large-fixture branches;
- using a generic two-axis iterator in cached and direct tiers increased
  retained instructions 0.355%;
- explicit axis matches in both tiers increased retained instructions 0.26%
  and branches 0.52%;
- applying precomputed axes to both cached and direct tiers increased retained
  instructions 0.18%; and
- an adaptive-only split that left boxes on the old helper increased box
  instructions 0.022% and branches 0.064%, weakened the generated benefit, and
  worsened retained clocks.

Only the precomputed, direct-tier two-axis form remains.

## Validation

The committed Hypermesh source passes:

- default, no-default, and all-feature tests: 1,057 / 1,057 / 1,058 unit tests
  plus all integration, policy, regression, and doctest surfaces;
- warning-denied all-target Clippy and warning-denied rustdoc under all and
  no-default features;
- formatting, every fuzz binary check, all-feature benchmark compilation, and
  the canonical native/WASM release/size harness;
- AddressSanitizer runs of the new all-axis overlap proof and the exact
  separation-hidden-by-binary64 regression; and
- opt-in release YeahRight checks for every Boolean operation's exact closed
  boundary and polygon/immediate API consistency.

The four dependency revisions are unchanged from the preceding validated
five-crate checkpoint. Hyperlimit's pre-existing untracked local `hyperlimit`
binary is not part of this evidence and was not modified.

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

CARGO_TARGET_DIR=/tmp/hypermesh-overlap-proof-asan \
  RUSTFLAGS='-Zsanitizer=address' ASAN_OPTIONS=detect_leaks=0 \
  cargo +nightly test --locked --target x86_64-unknown-linux-gnu \
  --lib <full-test-name> -- --exact

YEAHRIGHT_BENCH=1 cargo test --locked --release --test competitive \
  <exact-test-name> -- --ignored --exact --test-threads=1

../tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . --crate-name hypermesh \
  --out-dir /tmp/hypermesh-overlap-proof-callgraph --format json
../tools/hyper-callgraph/target/release/hyper-callgraph \
  --root .. --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --out-dir /tmp/hyperstack-overlap-proof-callgraph --format json
```
