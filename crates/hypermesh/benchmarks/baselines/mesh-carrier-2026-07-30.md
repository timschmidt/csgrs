# Compact mesh-carrier checkpoint — 2026-07-30

This checkpoint measures Hypermesh `5df21f0f` against its direct parent
`1aef2f4f` with the same current Hyperreal, Hyperlattice, and Hyperlimit
dependencies. The checked-in `carrier_retention` and `native_repeat` consumers
make the retained-memory and fixed-work runtime measurements reproducible.

## Ownership and exactness

`TriangleMeshFacts` is now a 72-byte immutable proof header rather than a
1,048-byte cache container, a 93.13% reduction. It retains only construction
plane provenance, optional exact input polygons, lazily shared exact bounds,
and three one-byte proof facts. Adjacency, component counts, finite
materializations, GPU buffers, and reversed winding are caller-owned results
instead of permanent mesh storage.

- Exact and axis-aligned bounds share one lazily allocated exact payload.
- Connectivity counting no longer materializes a full adjacency graph.
- Laplacian and Taubin smoothing build adjacency once per operation; both
  Taubin passes also share one scratch buffer.
- Reversed winding shares exact positions and preserves only the PWN proof
  justified by reversal.
- Exact GPU conversion is explicitly materializing and returns owned buffers.
  Invalid or policy-unknown triangle normals return
  `TriangleNormalUnavailable`; they are never replaced by a fabricated normal.

All winding mutations now pass through one two-phase checked transition. A
dimension mismatch or any `i32` overflow is typed and leaves the entire
winding vector unchanged.

## Cold retained memory

Heaptrack measured 100,000 simultaneously retained triangle meshes created by
the checked-in `carrier_retention` consumer. The same 300,015 allocation calls
occur in both revisions; the improvement is retained bytes per carrier rather
than an allocator artifact.

| Evidence | Parent | Current | Change |
| --- | ---: | ---: | ---: |
| `TriangleMeshFacts` | 1,048 B | 72 B | -93.13% |
| Peak heap | 113.68 MiB | 16.08 MiB | -85.86% |
| Peak RSS including Heaptrack | 122.39 MiB | 24.88 MiB | -79.67% |
| Allocation calls | 300,015 | 300,015 | 0 |
| Temporary allocations | 2 | 2 | 0 |
| Heaptrack runtime | 0.165 s | 0.116 s | -29.70% |
| Leaked process-runtime memory | 544 B | 544 B | 0 |

Heaptrack's runtime is secondary evidence because the run is short and
instrumented; the fixed-work hardware-counter run below is the runtime gate.

## Hot-path memory and runtime

One immediate certified box union returns the same `Certified` outcome, 44
vertices, and 84 triangles at both revisions.

| Immediate union evidence | Parent | Current | Change |
| --- | ---: | ---: | ---: |
| Allocation calls | 43 | 45 | +2 |
| Temporary allocations | 2 | 2 | 0 |
| Peak heap | 94.61 KiB | 93.27 KiB | -1.42% |
| Peak RSS including Heaptrack | 6.09 MiB | 6.66 MiB | noisy process floor |

The two additional calls allocate the lazy exact-bound payloads. That bounded
one-shot cost replaces hundreds of bytes of unconditional cache state on every
mesh and does not change the exact output.

Criterion confidence intervals for the exact box union overlap:
3.7011–3.8540 µs at the parent and 3.6991–3.7589 µs at the checkpoint.
The exact path therefore retains its earlier 173.56× historical Hypermesh,
18.18× boolmesh, and 17.47× manifold-rust speed ratios without a resolved
regression.

Five CPU-0 `perf stat` repetitions over 256 general unions used the checked-in
`native_repeat` consumer:

| Evidence | Parent | Current | Change |
| --- | ---: | ---: | ---: |
| Instructions | 4,666,399,555 ±0.01% | 4,663,974,560 ±0.02% | -0.0520% |
| Cycles | 2,283,617,043 ±0.75% | 2,293,619,004 ±1.17% | +0.438% |
| Task clock | 542.79 ms ±0.72% | 546.63 ms ±1.22% | +0.707% |
| Elapsed | 543.927 ms ±0.71% | 547.612 ms ±1.22% | +0.677% |

Instructions decline slightly. The noisier cycle and clock confidence
intervals overlap, so no runtime regression is resolved.

## Linked artifact size

Percentages are checkpoint growth over the direct parent. `Native code` is
`.text`; `WASM code` is `wasm-opt -Oz`.

| Consumer | Profile | Target | Parent raw | Current raw | Raw change | Parent code | Current code | Code change |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| General | Release | Native | 4,315,008 | 4,315,328 | +0.0074% | 3,700,909 | 3,701,085 | +0.0048% |
| General | Release | WASM | 3,334,427 | 3,333,367 | -0.0318% | 2,591,788 | 2,591,488 | -0.0116% |
| Immediate | Release | Native | 4,350,800 | 4,350,800 | 0 | 3,735,173 | 3,734,901 | -0.0073% |
| Immediate | Release | WASM | 3,351,567 | 3,350,442 | -0.0336% | 2,607,123 | 2,606,677 | -0.0171% |
| General | Size | Native | 1,908,224 | 1,908,864 | +0.0335% | 1,672,059 | 1,672,707 | +0.0388% |
| General | Size | WASM | 1,276,690 | 1,277,958 | +0.0993% | 1,084,503 | 1,084,991 | +0.0450% |
| Immediate | Size | Native | 1,920,816 | 1,921,408 | +0.0308% | 1,684,127 | 1,684,711 | +0.0347% |
| Immediate | Size | WASM | 1,287,315 | 1,288,543 | +0.0954% | 1,094,922 | 1,095,173 | +0.0229% |

Every code movement is below 0.05%; the largest raw movement is +0.0993%.
Release WASM and both immediate release code artifacts shrink. `cargo bloat`
attributes approximately 763.9 KiB of the size-profile general consumer to
Hypermesh at the parent and 763.4 KiB at the checkpoint.

## Verification

- `cargo test --all-features`: 1,156 executed tests passed; 7 ignored.
- `cargo test --no-default-features`: 1,154 executed tests passed; 7 ignored.
- all-target checks passed with all features and without default features.
- every fuzz target compiled.
- `cargo clippy --all-targets --all-features -- -D warnings` passed.
- rustdoc with warnings denied passed.
- native/WASM release and size consumers compiled and were measured.
- formatting and `git diff --check` passed.

Machine-readable values are in `mesh-carrier-2026-07-30.toml`.
