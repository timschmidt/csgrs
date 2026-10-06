# Exact CDT path-completeness checkpoint — 2026-07-31

This checkpoint covers Hypertri `bf9a792e6a31` against its direct parent
`de99ec16f88b`, using the same current Hyperreal, Hyperlattice, and Hyperlimit
revisions recorded in the companion TOML file. It is Phase 4/7 evidence for the
workspace-root `HYPERMESH_PATH_COMPLETE_IMPLEMENTATION_PLAN.md`.

## Result

Two finite exact inputs exposed previously unsupported or incorrect paths:

- a five-point near-collinear set produced a locally legal triangulation that
  omitted one convex-hull wedge because a fixed 64× supertriangle was not large
  enough; and
- a six-point constraint crossed a non-convex strip with no legal next edge
  flip, causing the general CDT path to stop.

The Delaunay seed now expands its exact supertriangle until an exact
combinatorial/convex boundary proof succeeds. General constraint recovery now
falls back to exact crossed-cavity retriangulation, reinserts collinear cavity
vertices, protects prior constraints, and verifies the target edge before
return. Both recovery and unconstrained legalization terminate from their
monotone exact invariants rather than hidden flip budgets.

The general CDT return path proves complete convex-hull coverage. Validation
uses one sorted edge-use table for adjacency, constraint presence, connectivity,
and local Delaunay checks instead of repeated quadratic edge scans. Protected
edge membership is sorted and binary-searched. Empty constrained collinear
inputs remain valid empty certified triangulations.

Every new decision uses the operation's existing `ExactKernel`: cavity winding
goes through Hyperlimit `ring_area_sign(policy)`, and all orientation,
containment, segment, equality, and in-circle decisions retain the same
STRICT/APPROXIMATE_512 accumulator. Exact-rational regressions report
`Certified` under both policies.

## Correctness and policy evidence

- All 256 feature-power test configurations pass.
- The all-feature surface passes 113 runtime tests and 4 doctests.
- Warning-denied all-feature, CDT-only, and featureless Clippy pass; all-feature
  rustdoc and the standalone UI package pass.
- Two committed generators each pass 20,000 cases under both STRICT and
  APPROXIMATE_512: two arbitrary finite constraints (including crossings) and
  three protected edges sharing an endpoint. The committed-source replay
  therefore executes 80,000 new policy-specific CDT outcomes, in addition to
  the existing crossing, closed-cycle, edge-flip, and vertex-split generators.
- Deterministic regressions cover adaptive supertriangle expansion,
  non-flippable cavity recovery, collinear cavity-boundary reinsertion,
  incomplete-hull rejection, and empty constrained collinear input.

## Runtime

Criterion rows used the optimized `exact` benchmark. The direct parent was
built in a temporary checkout against the identical current dependency
revisions.

| Exact row | Direct parent | Current | Direct change |
| --- | ---: | ---: | ---: |
| 400-point located Delaunay | 16.387 ms | 16.390 ms | +0.02% |
| Two crossing constraints | 20.123 µs | 20.298 µs | +0.87% |
| Separated-cycle general PSLG | 44.059 µs | 43.046 µs | 2.30% faster |
| Non-convex cavity recovery | unsupported | 38.539 µs | newly complete |
| Adaptive supertriangle fixture | incomplete topology | 28.796 µs | newly complete |

The ordinary 400-point row is 6.74% faster than the July policy checkpoint
(17.575 ms) and 58.00% faster than the retained historical 39.026 ms row. This
confirms that the recent Hyperreal/Hyperlimit corrections materially improved
the stack without hiding policy terminals.

## Dependency-only artifact size

### CDT consumer

| Profile/artifact | Direct parent | Current | Change |
| --- | ---: | ---: | ---: |
| Release native file | 1,522,856 | 1,536,040 | +0.87% |
| Release native `.text` | 1,114,206 | 1,125,510 | +1.01% |
| Release raw WASM | 916,219 | 930,466 | +1.55% |
| Release `wasm-opt -Oz` | 687,376 | 699,243 | +1.73% |
| Size native file | 861,568 | 871,824 | +1.19% |
| Size native `.text` | 641,887 | 651,615 | +1.52% |
| Size raw WASM | 396,000 | 404,255 | +2.08% |
| Size `wasm-opt -Oz` | 335,399 | 340,340 | +1.47% |

### All-algorithm/runtime-selection consumer

| Profile/artifact | Direct parent | Current | Change |
| --- | ---: | ---: | ---: |
| Release native file | 1,744,488 | 1,773,408 | +1.66% |
| Release native `.text` | 1,310,541 | 1,337,242 | +2.04% |
| Release raw WASM | 1,057,177 | 1,173,337 | +10.99% |
| Release `wasm-opt -Oz` | 795,219 | 811,688 | +2.07% |
| Size native file | 950,192 | 958,304 | +0.85% |
| Size native `.text` | 726,571 | 734,155 | +1.04% |
| Size raw WASM | 470,178 | 479,737 | +2.03% |
| Size `wasm-opt -Oz` | 400,677 | 406,414 | +1.43% |

The raw speed-profile WASM compiler output expands the newly reachable general
CDT fallback aggressively; `wasm-opt` reduces that movement to 2.07%. The
optimized artifacts and native code are the relevant linked-code tradeoff for
two formerly missing correctness families. This growth remains a packaging
recovery row rather than being normalized away.

A cold-code annotation for the cavity was rejected: its forced-fallback
Criterion interval did not improve, while release all-feature native grew from
1,773,408 to 1,776,568 bytes and raw WASM from 1,173,337 to 1,179,321 bytes.
The earlier full Earcut reuse experiment was also removed because the
specialized exact cavity clip is about 5% faster while preserving CDT-only
feature independence.

## Source and call graph

The implementation commit is +971/-199 total lines, including benchmarks and
generators; production `src` is net +634 lines. The exact fallback is retained
because the smaller parent cannot complete both regression families.

The regenerated five-crate source graph contains 19,477 nodes and 38,894 edges.
Hypertri alone moves from 1,233 nodes/1,738 edges at the direct parent to 1,311
nodes/1,885 edges. There are 27 direct Hypertri→Hyperlimit edges; the added
cavity edge is the canonical policy-requiring `ring_area_sign` path.

## Reproduction

```sh
cargo test --all-features --no-fail-fast
cargo hack test --feature-powerset --exclude-features all-algorithms
PROPTEST_CASES=20000 cargo test --no-default-features \
  --features all-algorithms --test fuzz_properties fuzz_cdt_recovers_
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS=-Dwarnings cargo doc --no-deps --all-features
cargo bench --bench exact --features all-algorithms,runtime-select -- exact_cdt_
./benchmarks/size-harness/measure.sh cdt
./benchmarks/size-harness/measure.sh all
```

The source call graph was generated from the workspace root:

```sh
tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir /tmp/hypertri-cdt-path-complete-callgraph \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --per-library \
  --format json
```
