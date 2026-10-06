# Sparse exact three-plane cofactor checkpoint

Date: 2026-08-01

Hyperreal direct parent: `dc36d4d5dd1a299448536dced41fd044239b5e75`

Hyperreal implementation: `dd226cb23b348b3983dc63142539c1c733c48757`

Hyperlattice direct parent: `53fbdf6fd35e08b9d696f1e2c3c5e742ea261a96`

Hyperlattice implementation: `d11ca2f0e825d8e26048cfda5d1101df21dcfef0`

Hypermesh measurement source: `25332f79558b9d668577c87744a52f0508fcf460`

Hypermesh evidence parent: `7d266469dadc2f5cb7a7b4427dab3563530d7cc7`

Other dependencies:

- Hyperlimit `3e5d8816cd32bba46f48e0c6c13ab7a9da227784`
- Hypertri `c47601266e0b9b17d0c5a0764fa22b18168ada73`

This checkpoint replaces four independently expanded exact 3-by-3
determinants in Hyperlattice's three-plane intersection with three exact
2-by-2 minors whenever any plane row has one nonzero exact-rational
coefficient. The arbitrary exact and symbolic construction remains the
complete fallback. Hypermesh source is unchanged; its canonical Hyperreal and
Hyperlattice dependencies are the direct A/B variables.

## Exact construction and canonical identity

For a 3-by-4 coefficient matrix `M`, Hyperlattice's homogeneous intersection
is the signed cofactor tuple. If row `r` has one nonzero coefficient `a` in
column `s`, Laplace expansion proves:

- coordinate `s` is exactly zero;
- each other coordinate is one 2-by-2 minor of the other two rows; and
- the three retained minors share the exact factor `(-1)^(r+s) a`.

Hyperreal's crate-private rational kernel selects the two other rows and the
three other columns, canonicalizes the borrowed rational references, chooses
the determinant sign from the row/column parity, and evaluates only the three
cross minors. It uses the existing dyadic signed-product reducer when the six
participating coefficients are dyadic; irrelevant coefficients in the sparse
column do not affect that classification. Each minor is multiplied by the
sparse coefficient and the sparse coordinate is the canonical exact zero.

The public `Real` aggregate succeeds only after all twelve inputs have proved
to be exact rationals and a row has proved to contain exactly one nonzero.
Hyperlattice attempts that aggregate first and otherwise executes the prior
expanded determinant construction unchanged. There is no epsilon, floating
projection, cache, retained scalar field, recursion, arbitrary pass limit, or
compatibility shim.

Exact tuple identity matters beyond projective equivalence. Hypermesh uses the
canonical signed cofactor tuple in construction identity and degeneracy paths.
An initially tested variant omitted the common sparse coefficient because a
homogeneous point is projectively invariant under nonzero scaling. Although
large-fixture topology remained projectively correct, the full Hypermesh suite
exposed 18 failures, including `PointAtInfinity` and
`UnknownClassification` paths. Restoring `(-1)^(r+s) a` made the optimized
tuple exactly equal to the old expanded tuple and cleared every failure. That
proportional-only variant was removed rather than hidden behind a compatibility
path.

The exhaustive focused oracle covers every 3 row by 4 column sparse position,
positive and negative general rational scales, dyadic and non-dyadic retained
coefficients, multiple sparse rows, dependent rows, the all-zero homogeneous
result, nonsparse exact matrices, and symbolic matrices. Successful sparse
results are asserted equal component-by-component to the expanded cofactor
tuple, not merely projectively equivalent.

## Hyperlimit policy contract

The new schedule is an exact construction below the predicate-policy boundary.
It neither accepts nor stores a policy and cannot consume an approximate
terminal:

- `STRICT` continues to accept only certified decisions;
- `APPROXIMATE_512` continues to use Hyperlimit's terminal 512-bit
  equality/zero interpretation only after certification is exhausted;
- nonsparse and symbolic input reaches the same complete expanded fallback;
  and
- Hypermesh continues to propagate the selected `MeshContext` and aggregate
  certainty normally.

Every measured operation under both policies produced the same topology and
`MeshCertainty::Certified`; the approximate terminal was never consumed.

## Workload dispatch and topology

The retained fixture is the recreated one-subdivision YeahRight Boolean hull
with 4,524 input triangles. The generated `yeahright-8` fixture has 13,452
input triangles. `boxes-3072` has 6,144 input triangles and deliberately does
not contain a sparse plane row, making it a fallback control.

| Fixture / each policy | Sparse exact schedule | Expanded fallback | Output | Certainty |
| --- | ---: | ---: | --- | --- |
| Retained | 8,333 | 0 | 625 vertices / 1,246 triangles | Certified |
| Generated | 1,249 | 0 | 154 / 304 | Certified |
| Boxes | 0 | 26 | 27 / 50 | Certified |

Parent, candidate, `STRICT`, and `APPROXIMATE_512` have identical output
topology. Temporary fixture-repetition instrumentation was removed after the
binaries were built; the normal diagnostic dispatch labels remain part of the
existing trace framework.

## Direct scalar control

The permanent Criterion row
`exact_product_sums/exact_rational_sparse_homogeneous_plane_intersection3`
evaluates a canonical sparse dyadic matrix in 259.73 ns with a 259.36--260.13
ns 95% interval. The aggregate did not exist in the direct parent, so no
synthetic scalar speedup is claimed. There is also no single like-for-like GMP
API: the public API coverage ledger classifies this as a Hyperreal aggregate.
The retained-process mesh A/B below measures the replaced expanded schedule.

## Retained-process CPU results

The fixture is built once and the Boolean is repeated in one process. Runs are
serialized, pinned to CPU 9, and bracket parent/candidate in reverse order.
Retained rows use two 51-operation measurements per revision, generated rows
two 101-operation measurements, and box rows two 501-operation measurements.
Values below are means per operation.

| Fixture / policy | Task clock | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Retained / `STRICT` parent | 41.435 ms | 175,020,938 | 485,282,965 | 84,887,303 | 769,516 | 1,253,942 |
| Retained / `STRICT` candidate | 39.318 ms (-5.110%) | 166,065,334 (-5.117%) | 462,329,049 (-4.730%) | 79,318,633 (-6.560%) | 737,515 (-4.159%) | 1,221,082 (-2.621%) |
| Retained / `APPROXIMATE_512` parent | 41.567 ms | 175,529,490 | 485,294,446 | 84,889,380 | 780,890 | 1,259,374 |
| Retained / `APPROXIMATE_512` candidate | 39.452 ms (-5.088%) | 166,543,015 (-5.120%) | 462,321,323 (-4.734%) | 79,317,042 (-6.564%) | 737,641 (-5.539%) | 1,229,491 (-2.373%) |
| Generated / `STRICT` parent | 23.171 ms | 94,082,465 | 152,799,933 | 25,673,647 | 266,574 | 519,994 |
| Generated / `STRICT` candidate | 21.961 ms (-5.222%) | 88,847,880 (-5.564%) | 147,813,872 (-3.263%) | 24,517,953 (-4.501%) | 277,573 (+4.126%) | 614,740 (+18.221%) |
| Generated / `APPROXIMATE_512` parent | 18.091 ms | 73,622,663 | 152,802,320 | 25,674,352 | 269,496 | 535,128 |
| Generated / `APPROXIMATE_512` candidate | 17.160 ms (-5.149%) | 70,195,726 (-4.655%) | 147,814,225 (-3.264%) | 24,517,898 (-4.504%) | 257,528 (-4.441%) | 500,785 (-6.418%) |
| Boxes / `STRICT` parent | 1.985 ms | 8,218,480 | 19,072,192 | 3,251,942 | 25,732 | 84,344 |
| Boxes / `STRICT` candidate | 2.259 ms (+13.788%) | 9,312,782 (+13.315%) | 19,075,426 (+0.017%) | 3,252,167 (+0.007%) | 23,821 (-7.426%) | 88,527 (+4.960%) |
| Boxes / `APPROXIMATE_512` parent | 2.711 ms | 11,002,744 | 19,071,778 | 3,251,846 | 32,604 | 98,034 |
| Boxes / `APPROXIMATE_512` candidate | 2.756 ms (+1.662%) | 11,204,544 (+1.834%) | 19,075,012 (+0.017%) | 3,252,094 (+0.008%) | 33,262 (+2.019%) | 103,467 (+5.542%) |

The retained improvement repeats on every static counter and both policy rows.
Generated instructions and branches repeat the expected reduction; its strict
cache counters are noisy while its task clock and cycles remain favorable.
The box control executes no sparse schedule and changes only 0.017% in
instructions, so its wall-clock variation is treated as layout/system noise,
not as an optimized-path regression.

## Profile attribution

The direct parent profile has 2,583 samples and the candidate 2,398. Both cover
30 retained operations at 1,999 Hz with zero lost samples.

| Symbol family | Parent self | Candidate self |
| --- | ---: | ---: |
| Six-factor/three-product signed determinant owner | 3.79% | 0.63% |
| Sparse homogeneous intersection kernel | absent | 0.71% |
| Two-product signed-minor ordering | included above | 0.93% |
| Borrowed rational multiplication | distributed | 0.29% |
| Public `Real` sparse wrapper | absent | 0.21% |

Unrelated 3-by-3 determinant work still accounts for the candidate's residual
six-factor owner. Sampling percentages are descriptive, but together with the
4.73% instruction reduction they show that the targeted four-determinant
expansion no longer dominates this construction.

## Large-fixture heap

Heaptrack covers fixture construction and one complete immediate union for
every large fixture and both policies. Recorder allocation totals and recorder
temporary-allocation classifications are authoritative.

| Fixture | Parent allocations | Candidate allocations | Movement | Parent / candidate temporary | Peak heap | Candidate RSS, both policies |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Retained | 520,149 | 454,003 | -66,146 / -12.717% | 27,636 / 28,608 | 12.71 / 12.71 MiB | 22.04 / 22.02 MiB |
| Generated | 215,071 | 200,755 | -14,316 / -6.656% | 10,317 / 10,316 | 11.66 / 11.66 MiB | 23.26 / 23.15 MiB |
| Boxes | 27,211 | 27,211 | unchanged | 79 / 78 | 4.70 / 4.70 MiB | 12.91 / 12.91 MiB |

Parent RSS was 22.23/22.32 MiB retained, 23.25/23.31 MiB generated, and
12.36/12.75 MiB boxes. The retained recorder temporary category grows by 972
while total allocations fall by 66,146; no retained heap is added and peak
heap is unchanged. Heaptrack's reconstructed temporary counts are 27,762 vs
28,734 retained, 10,358 vs 10,358 generated, and 80 vs 80 boxes; they are
recorded separately and are not substituted for the recorder summaries.

## Native and WASM linked code

Native code is linked `.text`; WASM code is `wasm-opt -Oz`. Release/speed
artifacts all shrink. The size profile pays about 2.1--2.3 KiB for the new
public aggregate and branch while retaining its more aggressive size-oriented
inlining decisions.

| Consumer | Profile / format | Parent | Candidate | Movement |
| --- | --- | ---: | ---: | ---: |
| General | Release native | 4,035,028 | 4,032,492 | -2,536 / -0.0629% |
| Immediate | Release native | 4,068,628 | 4,066,108 | -2,520 / -0.0619% |
| General | Release WASM | 2,707,924 | 2,707,649 | -275 / -0.0102% |
| Immediate | Release WASM | 2,722,963 | 2,722,688 | -275 / -0.0101% |
| General | Size native | 1,853,362 | 1,855,642 | +2,280 / +0.1230% |
| Immediate | Size native | 1,865,862 | 1,868,142 | +2,280 / +0.1222% |
| General | Size WASM | 1,149,614 | 1,151,698 | +2,084 / +0.1813% |
| Immediate | Size WASM | 1,160,583 | 1,162,667 | +2,084 / +0.1796% |

Hyperreal changes by 130 insertions: 107 production, 22 permanent benchmark,
and one API-classification test line. Hyperlattice changes by +180/-28 in one
source file, including the fallback extraction and exhaustive focused tests.
The call-graph utility reports:

| Scope | Parent | Candidate | Movement |
| --- | ---: | ---: | ---: |
| Isolated Hypermesh | 7,994 nodes / 19,633 edges | 7,994 / 19,633 | unchanged |
| Five Hyper crates | 19,614 nodes / 39,166 edges | 19,638 / 39,216 | +24 / +50 |

The five-crate increase includes the public wrapper, rational kernel closure,
permanent benchmark, and exhaustive tests. No second policy spine or alternate
construction family is introduced.

## Historical and competitive controls

The frozen historical retained row was 944.8 ms, 67.74 MiB peak heap,
5,020,891 allocations, and about 82.5 MiB maximum RSS. The current retained
strict repeated row is 39.318 ms, 12.71 MiB peak heap, 454,003 allocations,
and about 22.04 MiB RSS: directionally 95.84% faster, 81.24% lower peak heap,
90.96% fewer allocations, and 73.30% lower RSS. The historical fixture used
5,152 polygons and a different timing setup, so this is a trend, not a direct
correctness A/B. The preceding one-shot strict checkpoint was 51.61 ms.

Competitive Criterion was rerun on the final exact-scale candidate:

| Union workload | Hypermesh | boolmesh | manifold-rust | Relative result |
| --- | ---: | ---: | ---: | --- |
| Overlapping 12-triangle boxes | 4.8574 us | 65.038 us | 57.943 us | Hypermesh 13.39x / 11.93x faster |
| 3,072-triangle boxes per operand | 1.8585 ms | 7.5188 ms | 4.3430 ms | Hypermesh 4.05x / 2.34x faster |
| Generated YeahRight projective union | 7.0621 ms | 0.76067 ms | 0.67402 ms | Hypermesh 9.28x / 10.48x slower |

The competitors do not provide Hyperreal exact coordinates, Hyperlimit policy
selection, or Hypermesh's certified-result contract. They remain throughput
and memory controls rather than exactness oracles. The projective fixture is
still the principal competitive performance target even after this 5% exact
construction improvement.

## Full-resolution gate

The established full-resolution oracle remains exact CGAL EPECK's valid empty
zero-vertex/zero-face result for the 11,894-by-11,894 rotated fixture. A prior
conservative intermediate 512-bit Hypermesh run completed as the same certified
empty result in 3,357.09 seconds with 319.07 MiB maximum RSS. This construction
checkpoint does not relabel or replace that source-specific result.

## Validation

The final five-crate stack passes:

- default, no-default, and all-feature test matrices in Hyperreal,
  Hyperlattice, Hyperlimit, Hypertri, and Hypermesh;
- Hyperreal and Hyperlattice all-target checks/tests, including every Criterion
  sentinel (the broad Hyperreal run completed normally);
- Hypermesh's 1,053 default/no-default unit tests and 1,054 all-feature unit
  tests plus integration, policy, regression, and doctest surfaces;
- warning-denied all-target Clippy and warning-denied rustdoc under all and
  no-default features in all five crates;
- formatting, every fuzz workspace, and all-feature benchmark compilation in
  all five crates;
- Hypertri's `all-algorithms` test matrix, UI manifest, and cargo-hack check
  and test powersets across all 256 feature configurations;
- Hypermesh's CI WASM size harness; and
- focused AddressSanitizer runs for Hypermesh's general affine-box path and
  all seven Hyperlattice projective tests.

Representative commands:

```text
cargo test --locked --no-fail-fast
cargo test --locked --no-default-features --no-fail-fast
cargo test --locked --all-features -q
cargo check --locked --all-targets
cargo test --locked --all-targets -q
cargo clippy --locked --all-targets --all-features -- -D warnings
cargo clippy --locked --all-targets --no-default-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --no-default-features
cargo fmt --all -- --check
cargo check --locked --manifest-path fuzz/Cargo.toml --all-targets
cargo bench --locked --no-run --all-features
```
