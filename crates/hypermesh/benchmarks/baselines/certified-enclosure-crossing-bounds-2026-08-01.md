# Certified-enclosure crossing-bounds checkpoint

Date: 2026-08-01

Hypermesh direct parent: `c2cd8989b77d92a995302965a4188c674ebeac91`

Hypermesh implementation: `231b185cbb1c0ce4863029725fda8d1049909383`

Dependencies:

- Hyperreal `7262d3037d056c9fee83b07d6d43cc3d7bf65277`
- Hyperlattice `d11ca2f0e825d8e26048cfda5d1101df21dcfef0`
- Hyperlimit `3e5d8816cd32bba46f48e0c6c13ab7a9da227784`
- Hypertri `c47601266e0b9b17d0c5a0764fa22b18168ada73`

This checkpoint removes a redundant exact AABB pass from output-edge crossing
discovery when certified outward binary64 enclosures are available. It also
chooses the approximate sweep axis before constructing exact edge ordering and
computes exact endpoint order only on that selected axis. The no-enclosure
path retains its complete exact sweep and all-axis exact bounds.

## Exactness and policy invariant

`exact_output_vertex_enclosures` exists only when every output coordinate is
an exact rational with a finite outward binary64 enclosure. Disjoint enclosure
intervals prove exact separation and remain the first broad phase. An overlap
does not prove intersection; every survivor still reaches the existing exact
projected-segment predicate.

The removed exact AABB pass was only another sufficient rejection. It was not
a premise needed by `proper_segment_intersection_after_bounds_overlap`:

- projected orientation tests reject a same-side endpoint pair exactly;
- opposite orientations for both segments prove a proper crossing in that
  coordinate projection;
- all coordinate projections are tried when a projection degenerates; and
- exact coplanarity is then required. For a nondegenerate proper projection of
  two coplanar 3D lines, the dropped coordinate is fixed by the common plane,
  so the projected crossing is the 3D crossing.

The focused rounding-boundary regression places two rational segments one
exact unit apart near `2^60`, where their certified binary64 AABBs overlap. It
first proves that the former exact AABB pass rejects the pair, then proves that
the complete projected predicate and the full crossing-discovery loop also
reject it under both policies. A second symbolic `sqrt(2)` regression forces
the no-enclosure path and proves that its exact bounds still discover a proper
crossing under both policies.

The selected policy is unchanged throughout:

- `STRICT` still accepts only structural, filtered, or exact decisions;
- `APPROXIMATE_512` still terminates only in Hyperlimit after the unchanged
  complete decision stack is exhausted;
- the removed comparisons cannot introduce an approximate decision;
- no policy or certainty is cached in an edge, scalar, or mesh carrier; and
- every parent and candidate control has identical topology and
  `MeshCertainty::Certified` under both policies.

There is no epsilon, new allocation, cache, pass/candidate limit, alternate
topology path, compatibility shim, or policy-free entry point.

## Profile and path characterization

The preceding exact diagnostics found zero exact-AABB rejections after the
certified enclosure pass on all three retained controls:

| Fixture | Pair visits | Approximate rejects | Shared endpoints | Exact-AABB rejects | Projected rejects | Events |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Retained 4,524 triangles | 217,963 | 187,488 | 10,952 | 0 | 17,675 | 0 |
| Generated 13,452 triangles | 22,163 | 16,387 | 2,487 | 0 | 2,854 | 0 |
| Boxes 6,144 triangles | 1,655 | 1,075 | 365 | 0 | 165 | 0 |

All three are strict rejection controls: no event construction or repair work
can conceal a changed result. The retained profile uses 30 operations at
1,999 Hz. The direct parent recorded 2,450 samples and 6.06% self in
`split_edge_crossing_events`; the candidate records 2,219 samples, zero lost,
and 5.61% self. Sampling is descriptive; the serialized counters below are the
retention gate.

## Retained-process CPU results

Each fixture is built once and its Boolean union is repeated in one process.
Runs are serialized and pinned to CPU 9 in reverse-order parent/candidate
brackets. Values are the mean per operation of two measurements per revision.

| Fixture / policy | Task clock | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Retained / `STRICT` parent | 40.545 ms | 168,891,735 | 459,006,580 | 78,715,192 | 738,885 | 1,258,568 |
| Retained / `STRICT` candidate | 37.333 ms (-7.923%) | 156,348,214 (-7.427%) | 422,851,064 (-7.877%) | 71,873,371 (-8.692%) | 680,916 (-7.846%) | 1,249,768 (-0.699%) |
| Retained / `APPROXIMATE_512` parent | 39.601 ms | 165,779,920 | 458,997,085 | 78,712,825 | 737,974 | 1,247,206 |
| Retained / `APPROXIMATE_512` candidate | 36.584 ms (-7.619%) | 152,957,440 (-7.735%) | 422,858,228 (-7.873%) | 71,874,510 (-8.688%) | 681,635 (-7.634%) | 1,230,382 (-1.349%) |
| Generated / `STRICT` parent | 13.153 ms | 54,726,821 | 147,138,280 | 24,391,334 | 244,054 | 447,770 |
| Generated / `STRICT` candidate | 12.642 ms (-3.885%) | 52,667,114 (-3.764%) | 142,669,627 (-3.037%) | 23,524,514 (-3.554%) | 232,987 (-4.535%) | 442,823 (-1.105%) |
| Generated / `APPROXIMATE_512` parent | 13.248 ms | 55,141,949 | 147,137,188 | 24,391,142 | 243,427 | 449,320 |
| Generated / `APPROXIMATE_512` candidate | 12.665 ms (-4.400%) | 52,741,605 (-4.353%) | 142,666,103 (-3.039%) | 23,523,746 (-3.556%) | 232,796 (-4.367%) | 447,825 (-0.333%) |
| Boxes / `STRICT` parent | 1.9094 ms | 7,865,814 | 19,008,459 | 3,238,265 | 23,212 | 84,068 |
| Boxes / `STRICT` candidate | 1.8569 ms (-2.750%) | 7,622,771 (-3.090%) | 18,913,749 (-0.498%) | 3,214,027 (-0.748%) | 23,289 (+0.332%) | 85,562 (+1.777%) |
| Boxes / `APPROXIMATE_512` parent | 1.8351 ms | 7,602,354 | 19,008,247 | 3,238,191 | 23,268 | 83,989 |
| Boxes / `APPROXIMATE_512` candidate | 1.8165 ms (-1.012%) | 7,519,426 (-1.091%) | 18,914,281 (-0.494%) | 3,214,149 (-0.742%) | 22,298 (-4.165%) | 84,840 (+1.013%) |

Instructions and branches fall deterministically on every fixture and policy.
The branch/cache-miss box movements are short-row layout noise; task clock and
cycles remain favorable in the final bracket.

## Criterion, historical, and competitive controls

A clean direct-parent tree and the candidate were run in an adjacent
candidate/parent/candidate Criterion bracket. Generated projective-union
centers were 6.5846 ms, 7.0251 ms, and 6.5799 ms. The candidate bracket mean is
6.5823 ms, 6.304% below the direct parent, and all candidate confidence
intervals are disjoint from the parent interval.

Current competitive controls on the same pinned session are:

| Engine | Generated projective union | Relative to Hypermesh |
| --- | ---: | ---: |
| Hypermesh exact candidate | 6.5823 ms bracket mean | 1.00x |
| boolmesh | 749.98 us | Hypermesh 8.78x slower |
| manifold-rust | 657.43 us | Hypermesh 10.01x slower |

The competitors do not preserve Hyperreal coordinates, expose Hyperlimit
policy selection, or report Hypermesh certification. They are throughput
controls, not exactness oracles. The preceding stored Hypermesh row was
6.9849 ms, so this checkpoint is 5.765% lower across sessions.

Against the frozen historical retained row (944.8 ms, 67.74 MiB peak heap,
5,020,891 allocations, and about 82.5 MiB RSS), current strict retained work is
37.333 ms, 12.71 MiB, 454,003 allocations, and 22.23 MiB RSS: directional
reductions of 96.05% runtime, 81.24% peak heap, 90.96% allocations, and 73.05%
RSS. Fixture and timing differences make this a trend rather than a direct A/B.

## Large-fixture heap

Heaptrack records fixture construction plus one complete immediate union.
Total allocations and peak heap are unchanged from the direct parent. The
recorder/reconstruction classifies one additional existing allocation as
temporary; no allocation is added and the total count is identical.

| Fixture / policy | Allocations | Recorder temporary | Reconstructed temporary | Peak heap | RSS |
| --- | ---: | ---: | ---: | ---: | ---: |
| Retained / `STRICT` | 454,003 | 28,609 | 28,735 | 12.71 MiB | 22.23 MiB |
| Retained / `APPROXIMATE_512` | 454,003 | 28,609 | 28,735 | 12.71 MiB | 22.09 MiB |
| Generated / `STRICT` | 200,755 | 10,317 | 10,359 | 11.66 MiB | 23.19 MiB |
| Generated / `APPROXIMATE_512` | 200,755 | 10,317 | 10,359 | 11.66 MiB | 23.24 MiB |

## Native and WASM linked code

Native code is linked `.text`; WASM code is `wasm-opt -Oz`. The canonical
dependency-only consumers compare the paired-line checkpoint with this change:

| Consumer | Profile / format | Parent | Candidate | Movement |
| --- | --- | ---: | ---: | ---: |
| General | Release native | 4,033,596 | 4,033,388 | -208 / -0.0052% |
| Immediate | Release native | 4,067,212 | 4,067,004 | -208 / -0.0051% |
| General | Release WASM | 2,709,462 | 2,710,939 | +1,477 / +0.0545% |
| Immediate | Release WASM | 2,724,501 | 2,725,978 | +1,477 / +0.0542% |
| General | Size native | 1,855,658 | 1,856,058 | +400 / +0.0216% |
| Immediate | Size native | 1,868,158 | 1,868,566 | +408 / +0.0218% |
| General | Size WASM | 1,152,464 | 1,152,641 | +177 / +0.0154% |
| Immediate | Size WASM | 1,163,433 | 1,163,610 | +177 / +0.0152% |

The repeated-probe release executable grows 328 file bytes and 688 `.text`
bytes. No production carrier grows.

The call-graph utility reports 8,005 nodes / 19,655 edges for isolated
Hypermesh and 19,655 / 39,245 for the five-crate scope. Relative to the parent
checkpoint this is +7 nodes / +17 edges in either scope, accounted for by the
focused exactness regressions, raw-edge selector shape, and exact-axis bounds
construction. No new policy, terminal, or topology spine is introduced.

## Rejected experiments

Every losing implementation was fully removed:

- forcing the mixed-width GCD dispatcher out of line added about 0.257%
  retained instructions and 52 bytes of representative `.text`;
- precomputing and passing BigUint bit/trailing-zero facts saved 1,012 bytes of
  representative `.text` but added 0.055--0.058% retained instructions;
- compacting six endpoint indices into a three-bit mask reduced the native
  edge carrier from 64 to 24 bytes and removed 2,884 bytes of representative
  `.text`, but changed sort code generation: generated instructions rose
  0.045%, generated branches 0.098%, and retained branches 0.109%; and
- retaining exact endpoint order on all three axes after the exact AABB pass
  was removed lost 0.03--0.08% of instructions across the owning controls, so
  the selected-axis-only construction was retained.

Performance has priority over size, so the compact carrier and GCD size wins
were rejected.

## Validation

The committed Hypermesh source passes:

- default, no-default, and all-feature tests: 1,055 / 1,055 / 1,056 unit tests
  plus all integration, policy, regression, and doctest surfaces;
- warning-denied all-target Clippy and warning-denied rustdoc under all and
  no-default features;
- formatting, every fuzz binary check, and all-feature benchmark compilation;
- the canonical native/WASM release/size harness;
- focused AddressSanitizer runs for the certified-enclosure rounding boundary
  and symbolic no-enclosure crossing under both policies; and
- opt-in release YeahRight checks for every Boolean operation's exact closed
  boundary and polygon/immediate API consistency.

The four dependency revisions are unchanged from the immediately preceding
five-crate checkpoint, where their default/no-default/all-feature, lint, docs,
fuzz, benchmark, and sanitizer surfaces passed. Hypermesh's all-feature build
recompiled the same revisions.

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

CARGO_TARGET_DIR=/tmp/hypermesh-certified-bounds-asan \
  RUSTFLAGS='-Zsanitizer=address' ASAN_OPTIONS=detect_leaks=0 \
  cargo +nightly test --locked --target x86_64-unknown-linux-gnu <filter> --lib

YEAHRIGHT_BENCH=1 cargo test --locked --release --test competitive \
  <exact-test-name> -- --ignored --exact

./benchmarks/size-harness/measure.sh default
```
