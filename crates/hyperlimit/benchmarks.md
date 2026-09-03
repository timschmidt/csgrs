<!-- BEGIN promoted_slow_offender_score -->
## `promoted_slow_offender_score`

Deterministic lexicase score for Hyperlimit's retained fuzz offenders. The score is the average current best-of-five replay time; lower is better. Delta compares with the previous score, and derivative is the change in delta.

<!-- promoted_slow_score_nanos: 19839 -->
<!-- promoted_slow_previous_score_nanos: 26579 -->
<!-- promoted_slow_score_delta_nanos: -6740 -->

| Metric | Value |
| --- | ---: |
| Cases scored | 100 |
| Average score | 19.839 us |
| Delta | -6.740 us |
| Delta derivative | -6.740 us |

| Rank | Current Time | Fuzz target | Input |
| ---: | ---: | --- | --- |
| 1 | 22.618 us | `predicate_invariants` | `seed[1498]` |
| 2 | 22.029 us | `predicate_invariants` | `seed[1196]` |
| 3 | 21.639 us | `predicate_invariants` | `seed[3034]` |
| 4 | 21.578 us | `predicate_invariants` | `seed[978]` |
| 5 | 21.329 us | `predicate_invariants` | `seed[1517]` |
| 6 | 21.329 us | `predicate_invariants` | `seed[1549]` |
| 7 | 21.249 us | `predicate_invariants` | `seed[2192]` |
| 8 | 21.199 us | `predicate_invariants` | `seed[1577]` |
| 9 | 21.169 us | `predicate_invariants` | `seed[1048]` |
| 10 | 21.078 us | `predicate_invariants` | `seed[919]` |

<!-- END promoted_slow_offender_score -->







# Hyperlimit Benchmarks

This file is updated automatically by the benchmark binaries.

<!-- BEGIN COMPLETE BENCHMARK REPORT -->
## Complete generated benchmark report

Every registered benchmark target is catalogued below. Every Criterion result found under `target/criterion` is included without a name or implementation filter; non-Criterion targets write their own linked reports. Each timing binary refreshes this section after it runs.

Run the complete non-instrumented timing set with:

```sh
cargo bench --features parallel
```

Regenerate this Markdown from stored Criterion data without rerunning benchmarks:

```sh
cargo run --example write_benchmarks_md
```

### Registered benchmark suites

| Target | Kind | Required features | Command | Generated report |
| --- | --- | --- | --- | --- |
| `predicates` | Criterion timing | `default` | `cargo bench --bench predicates` | this file |
| `retained_fuzz` | Criterion timing | `default` | `cargo bench --bench retained_fuzz` | this file |
| `predicates` trace mode | diagnostic | `dispatch-trace` | `cargo bench --all-features --bench predicates -- --write-dispatch-trace-md` | [dispatch_trace.md](dispatch_trace.md) |

### Comparative results

Rows sharing a Criterion group and input are compared when they expose distinct implementations. Ratios are elapsed time relative to the fastest stored row; they do not imply identical guarantees or output semantics.

| Group | Input | Implementation | Mean | Relative to fastest |
| --- | --- | --- | ---: | ---: |
| `incircle2d` | `easy` | `apfp` | 2.50 us | 1.00x |
| `incircle2d` | `easy` | `robust` | 2.91 us | 1.16x |
| `incircle2d` | `easy` | `geometry_predicates` | 2.92 us | 1.17x |
| `incircle2d` | `easy` | `hyperreal_evidence` | 18.06 us | 7.23x |
| `incircle2d` | `easy` | `hyperreal` | 24.98 us | 10.00x |
| `incircle2d` | `near_degenerate` | `apfp` | 2.63 us | 1.00x |
| `incircle2d` | `near_degenerate` | `robust` | 3.01 us | 1.14x |
| `incircle2d` | `near_degenerate` | `geometry_predicates` | 3.09 us | 1.18x |
| `incircle2d` | `near_degenerate` | `hyperreal_evidence` | 18.10 us | 6.88x |
| `incircle2d` | `near_degenerate` | `hyperreal` | 24.92 us | 9.48x |
| `insphere3d` | `easy` | `geometry_predicates` | 10.99 us | 1.00x |
| `insphere3d` | `easy` | `robust` | 14.21 us | 1.29x |
| `insphere3d` | `easy` | `hyperreal_evidence` | 41.97 us | 3.82x |
| `insphere3d` | `easy` | `hyperreal` | 71.31 us | 6.49x |
| `insphere3d` | `near_degenerate` | `geometry_predicates` | 10.82 us | 1.00x |
| `insphere3d` | `near_degenerate` | `robust` | 13.98 us | 1.29x |
| `insphere3d` | `near_degenerate` | `hyperreal_evidence` | 42.78 us | 3.96x |
| `insphere3d` | `near_degenerate` | `hyperreal` | 74.25 us | 6.86x |
| `orient2d` | `easy` | `apfp` | 986.89 ns | 1.00x |
| `orient2d` | `easy` | `geometry_predicates` | 1.23 us | 1.25x |
| `orient2d` | `easy` | `robust` | 1.30 us | 1.32x |
| `orient2d` | `easy` | `hyperreal` | 13.25 us | 13.42x |
| `orient2d` | `near_degenerate` | `apfp` | 966.41 ns | 1.00x |
| `orient2d` | `near_degenerate` | `robust` | 1.36 us | 1.41x |
| `orient2d` | `near_degenerate` | `geometry_predicates` | 1.52 us | 1.57x |
| `orient2d` | `near_degenerate` | `hyperreal` | 13.15 us | 13.61x |
| `orient3d` | `easy` | `geometry_predicates` | 3.49 us | 1.00x |
| `orient3d` | `easy` | `robust` | 8.40 us | 2.40x |
| `orient3d` | `easy` | `hyperreal` | 47.41 us | 13.57x |
| `orient3d` | `near_degenerate` | `geometry_predicates` | 3.58 us | 1.00x |
| `orient3d` | `near_degenerate` | `robust` | 8.75 us | 2.44x |
| `orient3d` | `near_degenerate` | `hyperreal` | 41.38 us | 11.56x |

### All Criterion results

| Benchmark | Mean | 95% CI | Median | Change vs baseline | Throughput |
| --- | ---: | ---: | ---: | ---: | ---: |
| `aabb_immediate/3d/intersection` | 85.30 ns | 84.51 ns - 86.23 ns | 85.01 ns | - | - |
| `batch_parallel/incircle2d/near_degenerate/rayon` | 135.66 us | 133.90 us - 137.41 us | 135.66 us | - | 8192 elements |
| `batch_parallel/incircle2d/near_degenerate/sequential` | 385.39 us | 380.32 us - 390.46 us | 385.39 us | - | 8192 elements |
| `batch_parallel/insphere3d/near_degenerate/rayon` | 275.90 us | 275.64 us - 276.17 us | 275.90 us | - | 8192 elements |
| `batch_parallel/insphere3d/near_degenerate/sequential` | 1.17 ms | 1.16 ms - 1.18 ms | 1.17 ms | - | 8192 elements |
| `batch_parallel/orient2d/near_degenerate/rayon` | 118.23 us | 118.19 us - 118.27 us | 118.23 us | - | 8192 elements |
| `batch_parallel/orient2d/near_degenerate/sequential` | 368.01 us | 358.47 us - 377.55 us | 368.01 us | - | 8192 elements |
| `batch_parallel/orient3d/near_degenerate/rayon` | 173.44 us | 172.19 us - 174.68 us | 173.44 us | - | 8192 elements |
| `batch_parallel/orient3d/near_degenerate/sequential` | 658.70 us | 645.72 us - 671.69 us | 658.70 us | - | 8192 elements |
| `exact_rational_kernels/convex/halfspace_feasibility3_active_sets` | 16.46 us | 16.43 us - 16.50 us | 16.44 us | +2.25% | - |
| `exact_rational_kernels/distance3/point_triangle_dyadic_thresholds` | 792.01 us | 788.79 us - 795.49 us | 788.48 us | +4.90% | - |
| `exact_rational_kernels/distance3/point_triangle_integer_thresholds` | 969.00 us | 963.20 us - 975.67 us | 962.96 us | -38.16% | - |
| `exact_rational_kernels/distance3/point_triangle_scaled_thresholds` | 1.23 ms | 1.23 ms - 1.24 ms | 1.23 ms | -23.82% | - |
| `exact_rational_kernels/homogeneous/three_plane_coordinate_triples` | 373.90 us | 373.36 us - 374.45 us | 373.64 us | - | - |
| `exact_rational_kernels/homogeneous/two_plane_line_then_plane` | 475.54 us | 454.86 us - 498.71 us | 432.24 us | - | - |
| `exact_rational_kernels/triangle3/ray_intersection_reports` | 1.37 ms | 1.37 ms - 1.37 ms | 1.37 ms | -0.51% | - |
| `hypermesh_port_helpers/segment_plane/determinant_ratio` | 1.82 us | 1.81 us - 1.82 us | 1.82 us | +4.15% | - |
| `incircle2d/apfp/easy` | 2.50 us | 2.47 us - 2.53 us | 2.50 us | - | - |
| `incircle2d/apfp/near_degenerate` | 2.63 us | 2.63 us - 2.63 us | 2.63 us | - | - |
| `incircle2d/geometry_predicates/easy` | 2.92 us | 2.90 us - 2.94 us | 2.92 us | - | - |
| `incircle2d/geometry_predicates/near_degenerate` | 3.09 us | 3.06 us - 3.13 us | 3.09 us | - | - |
| `incircle2d/hyperreal/easy` | 24.98 us | 24.34 us - 25.62 us | 24.98 us | - | - |
| `incircle2d/hyperreal/near_degenerate` | 24.92 us | 24.71 us - 25.13 us | 24.92 us | - | - |
| `incircle2d/hyperreal_evidence/easy` | 18.06 us | 18.00 us - 18.12 us | 18.06 us | - | - |
| `incircle2d/hyperreal_evidence/near_degenerate` | 18.10 us | 17.94 us - 18.26 us | 18.10 us | - | - |
| `incircle2d/robust/easy` | 2.91 us | 2.87 us - 2.95 us | 2.91 us | - | - |
| `incircle2d/robust/near_degenerate` | 3.01 us | 2.99 us - 3.02 us | 3.01 us | - | - |
| `insphere3d/geometry_predicates/easy` | 10.99 us | 10.90 us - 11.07 us | 10.99 us | - | - |
| `insphere3d/geometry_predicates/near_degenerate` | 10.82 us | 10.78 us - 10.86 us | 10.82 us | - | - |
| `insphere3d/hyperreal/easy` | 71.31 us | 71.11 us - 71.51 us | 71.31 us | - | - |
| `insphere3d/hyperreal/near_degenerate` | 74.25 us | 72.95 us - 75.54 us | 74.25 us | - | - |
| `insphere3d/hyperreal_evidence/easy` | 41.97 us | 41.86 us - 42.09 us | 41.97 us | - | - |
| `insphere3d/hyperreal_evidence/near_degenerate` | 42.78 us | 42.77 us - 42.79 us | 42.78 us | - | - |
| `insphere3d/robust/easy` | 14.21 us | 13.97 us - 14.45 us | 14.21 us | - | - |
| `insphere3d/robust/near_degenerate` | 13.98 us | 13.94 us - 14.01 us | 13.98 us | - | - |
| `orient2d/apfp/easy` | 986.89 ns | 962.76 ns - 1.01 us | 986.89 ns | - | - |
| `orient2d/apfp/near_degenerate` | 966.41 ns | 958.87 ns - 973.94 ns | 966.41 ns | - | - |
| `orient2d/geometry_predicates/easy` | 1.23 us | 1.21 us - 1.25 us | 1.23 us | - | - |
| `orient2d/geometry_predicates/near_degenerate` | 1.52 us | 1.51 us - 1.53 us | 1.52 us | - | - |
| `orient2d/hyperreal/easy` | 13.25 us | 13.04 us - 13.45 us | 13.25 us | - | - |
| `orient2d/hyperreal/near_degenerate` | 13.15 us | 12.79 us - 13.52 us | 13.15 us | - | - |
| `orient2d/robust/easy` | 1.30 us | 1.30 us - 1.30 us | 1.30 us | - | - |
| `orient2d/robust/near_degenerate` | 1.36 us | 1.35 us - 1.36 us | 1.36 us | - | - |
| `orient3d/geometry_predicates/easy` | 3.49 us | 3.46 us - 3.53 us | 3.49 us | - | - |
| `orient3d/geometry_predicates/near_degenerate` | 3.58 us | 3.54 us - 3.62 us | 3.58 us | - | - |
| `orient3d/hyperreal/easy` | 47.41 us | 46.20 us - 48.62 us | 47.41 us | - | - |
| `orient3d/hyperreal/near_degenerate` | 41.38 us | 41.28 us - 41.47 us | 41.38 us | - | - |
| `orient3d/robust/easy` | 8.40 us | 8.34 us - 8.45 us | 8.40 us | - | - |
| `orient3d/robust/near_degenerate` | 8.75 us | 8.72 us - 8.77 us | 8.75 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_0` | 40.39 us | 34.56 us - 46.88 us | 37.35 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1` | 33.19 us | 30.37 us - 37.31 us | 30.79 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_10` | 49.74 us | 36.97 us - 64.96 us | 36.48 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1008` | 19.91 us | 19.66 us - 20.21 us | 19.72 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1009` | 21.52 us | 20.81 us - 22.41 us | 20.83 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1048` | 22.50 us | 21.99 us - 23.06 us | 22.43 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_11` | 38.72 us | 34.11 us - 46.10 us | 34.64 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1117` | 22.40 us | 20.76 us - 24.27 us | 21.75 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1143` | 20.61 us | 19.77 us - 21.75 us | 19.85 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1162` | 21.78 us | 21.41 us - 22.29 us | 21.45 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1184` | 22.79 us | 22.26 us - 23.33 us | 22.68 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1196` | 23.44 us | 22.49 us - 24.89 us | 22.99 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_12` | 35.83 us | 32.07 us - 41.43 us | 32.55 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_13` | 32.33 us | 29.93 us - 36.71 us | 30.09 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1369` | 20.60 us | 19.99 us - 21.48 us | 20.16 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_14` | 103.58 us | 52.41 us - 166.54 us | 61.43 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1401` | 19.99 us | 19.47 us - 20.54 us | 19.90 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1402` | 20.05 us | 19.49 us - 20.96 us | 19.60 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1433` | 21.86 us | 21.43 us - 22.56 us | 21.60 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1437` | 21.61 us | 21.48 us - 21.75 us | 21.55 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1498` | 26.39 us | 24.40 us - 29.30 us | 25.61 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_15` | 33.51 us | 31.91 us - 35.85 us | 32.13 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1517` | 24.95 us | 21.90 us - 29.09 us | 21.90 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1539` | 20.48 us | 19.35 us - 22.31 us | 19.37 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1549` | 23.16 us | 22.44 us - 24.00 us | 22.66 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1557` | 20.81 us | 20.31 us - 21.32 us | 20.78 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1575` | 20.91 us | 20.32 us - 21.48 us | 21.07 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1576` | 21.71 us | 20.66 us - 22.91 us | 20.65 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1577` | 22.52 us | 21.99 us - 23.39 us | 22.19 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1586` | 19.73 us | 19.63 us - 19.84 us | 19.76 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_16` | 41.88 us | 34.75 us - 50.33 us | 35.23 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1600` | 20.17 us | 19.72 us - 20.69 us | 19.71 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1612` | 21.06 us | 20.73 us - 21.51 us | 20.93 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1626` | 20.42 us | 19.76 us - 21.18 us | 19.94 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1640` | 20.87 us | 20.22 us - 21.65 us | 20.30 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1673` | 22.49 us | 21.37 us - 24.05 us | 21.55 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1675` | 21.01 us | 20.32 us - 21.85 us | 20.47 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1682` | 22.01 us | 21.31 us - 23.07 us | 21.60 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_17` | 47.64 us | 36.94 us - 59.84 us | 41.12 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1701` | 21.70 us | 20.36 us - 23.36 us | 21.06 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1738` | 22.61 us | 21.56 us - 24.12 us | 21.79 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1750` | 21.51 us | 20.99 us - 22.15 us | 21.11 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_18` | 35.50 us | 29.50 us - 43.05 us | 31.11 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_19` | 33.63 us | 30.92 us - 37.91 us | 31.28 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1912` | 20.53 us | 19.78 us - 21.69 us | 19.92 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1914` | 21.35 us | 20.43 us - 22.50 us | 20.74 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1937` | 21.38 us | 21.24 us - 21.56 us | 21.24 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1946` | 23.18 us | 22.00 us - 24.61 us | 22.56 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1950` | 21.49 us | 20.42 us - 23.40 us | 20.51 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1956` | 22.62 us | 21.69 us - 23.72 us | 22.54 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1965` | 20.48 us | 20.03 us - 21.19 us | 20.13 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1967` | 21.37 us | 20.78 us - 22.32 us | 20.91 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1977` | 21.75 us | 21.43 us - 22.11 us | 21.61 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1993` | 20.76 us | 20.03 us - 21.74 us | 20.10 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1995` | 21.00 us | 20.81 us - 21.20 us | 20.98 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_2` | 54.17 us | 43.84 us - 63.92 us | 60.13 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_20` | 31.53 us | 29.79 us - 33.55 us | 29.72 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_2046` | 21.21 us | 20.62 us - 21.88 us | 20.82 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_2054` | 20.02 us | 19.81 us - 20.24 us | 19.98 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_21` | 102.72 us | 63.96 us - 146.25 us | 115.30 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_2186` | 21.56 us | 21.28 us - 21.94 us | 21.30 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_2192` | 23.44 us | 22.56 us - 24.65 us | 22.73 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_22` | 27.55 us | 26.53 us - 28.81 us | 27.34 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_2240` | 22.23 us | 21.42 us - 23.23 us | 21.44 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_2286` | 21.35 us | 21.11 us - 21.66 us | 21.21 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_2290` | 22.86 us | 21.62 us - 24.38 us | 22.06 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_2292` | 23.20 us | 21.59 us - 25.52 us | 21.72 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_23` | 37.93 us | 32.69 us - 47.51 us | 33.16 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_2365` | 20.21 us | 19.42 us - 21.31 us | 19.40 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_24` | 43.89 us | 36.27 us - 54.25 us | 37.40 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_25` | 55.15 us | 34.24 us - 78.30 us | 33.76 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_26` | 107.94 us | 63.40 us - 167.07 us | 104.49 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_27` | 25.84 us | 25.32 us - 26.33 us | 26.03 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_28` | 101.76 us | 69.76 us - 135.49 us | 100.22 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_29` | 26.45 us | 26.22 us - 26.71 us | 26.32 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_3` | 29.53 us | 29.35 us - 29.74 us | 29.40 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_30` | 37.50 us | 34.33 us - 41.58 us | 35.96 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_3034` | 20.01 us | 19.78 us - 20.29 us | 19.81 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_3035` | 21.66 us | 20.53 us - 22.92 us | 20.67 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_31` | 45.17 us | 34.23 us - 58.27 us | 40.65 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_32` | 32.66 us | 32.26 us - 33.04 us | 32.74 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_33` | 31.24 us | 30.73 us - 31.88 us | 31.17 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_34` | 39.78 us | 32.42 us - 49.02 us | 33.51 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_35` | 31.54 us | 27.73 us - 35.98 us | 27.68 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_36` | 56.10 us | 34.48 us - 87.48 us | 34.45 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_37` | 271.37 us | 29.57 us - 546.83 us | 28.75 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_38` | 31.46 us | 29.20 us - 34.27 us | 29.27 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_39` | 45.39 us | 33.02 us - 65.08 us | 33.73 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_4` | 37.57 us | 29.64 us - 46.66 us | 27.99 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_40` | 55.68 us | 41.61 us - 72.48 us | 51.72 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_41` | 32.42 us | 30.32 us - 35.98 us | 30.92 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_42` | 79.95 us | 49.06 us - 113.64 us | 68.43 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_43` | 31.17 us | 27.88 us - 35.07 us | 28.14 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_44` | 33.55 us | 29.31 us - 39.34 us | 31.09 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_45` | 54.14 us | 28.96 us - 99.73 us | 29.10 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_46` | 37.75 us | 30.97 us - 48.96 us | 31.31 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_47` | 40.49 us | 29.89 us - 55.72 us | 28.20 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_48` | 34.49 us | 32.93 us - 36.26 us | 34.04 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_49` | 33.84 us | 26.99 us - 43.05 us | 26.58 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_5` | 37.21 us | 32.25 us - 45.32 us | 33.62 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_50` | 38.63 us | 33.88 us - 44.57 us | 35.34 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_5022` | 40.98 us | 30.79 us - 59.74 us | 31.05 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_51` | 62.82 us | 40.48 us - 92.44 us | 49.97 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_52` | 32.75 us | 31.19 us - 35.35 us | 31.45 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_5279` | 20.23 us | 19.68 us - 20.87 us | 19.69 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_53` | 31.32 us | 29.33 us - 32.88 us | 32.65 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_532` | 21.08 us | 19.96 us - 22.26 us | 20.74 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_54` | 42.32 us | 32.75 us - 59.26 us | 32.53 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_5448` | 19.84 us | 19.48 us - 20.37 us | 19.66 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_5449` | 21.36 us | 20.70 us - 22.11 us | 21.34 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_55` | 66.87 us | 38.58 us - 101.62 us | 34.58 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_56` | 30.56 us | 30.18 us - 31.03 us | 30.40 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_57` | 71.75 us | 40.41 us - 123.29 us | 39.57 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_58` | 44.34 us | 32.86 us - 58.65 us | 32.31 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_59` | 40.62 us | 36.16 us - 46.49 us | 36.26 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_6` | 33.44 us | 31.92 us - 35.92 us | 32.12 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_60` | 36.43 us | 31.76 us - 43.66 us | 32.22 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_61` | 32.79 us | 28.02 us - 39.40 us | 31.46 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_62` | 31.54 us | 30.08 us - 33.49 us | 30.69 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_63` | 39.00 us | 30.55 us - 49.93 us | 31.41 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_64` | 106.80 us | 60.44 us - 157.76 us | 82.39 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_6445` | 21.52 us | 20.62 us - 22.60 us | 21.20 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_65` | 32.36 us | 31.09 us - 34.01 us | 31.21 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_66` | 134.54 us | 73.20 us - 222.54 us | 120.35 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_67` | 33.20 us | 32.18 us - 34.21 us | 33.30 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_68` | 45.63 us | 33.18 us - 66.80 us | 33.44 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_69` | 43.13 us | 32.80 us - 54.99 us | 32.82 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_7` | 32.09 us | 29.10 us - 36.05 us | 31.14 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_70` | 35.13 us | 34.28 us - 36.43 us | 34.33 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_71` | 67.02 us | 41.22 us - 107.20 us | 55.40 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_719` | 22.02 us | 20.95 us - 23.18 us | 21.99 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_72` | 31.90 us | 30.00 us - 34.71 us | 30.69 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_722` | 22.76 us | 21.58 us - 24.20 us | 21.59 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_73` | 77.18 us | 42.08 us - 127.83 us | 59.94 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_74` | 41.38 us | 33.34 us - 53.09 us | 34.32 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_75` | 48.11 us | 26.98 us - 88.01 us | 30.27 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_750` | 20.55 us | 20.18 us - 21.08 us | 20.34 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_753` | 20.52 us | 20.00 us - 21.10 us | 20.26 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_754` | 21.71 us | 20.52 us - 23.19 us | 20.69 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_758` | 22.31 us | 20.88 us - 23.86 us | 21.29 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_76` | 27.27 us | 26.86 us - 27.73 us | 27.08 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_761` | 21.68 us | 20.92 us - 22.53 us | 21.32 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_763` | 21.63 us | 21.46 us - 21.82 us | 21.55 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_77` | 36.76 us | 31.93 us - 43.24 us | 32.45 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_774` | 20.45 us | 19.61 us - 21.56 us | 19.72 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_775` | 20.91 us | 20.54 us - 21.44 us | 20.57 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_78` | 45.62 us | 36.10 us - 56.26 us | 42.24 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_781` | 22.19 us | 21.20 us - 23.38 us | 21.88 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_79` | 28.83 us | 27.42 us - 30.00 us | 29.60 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_799` | 20.31 us | 19.50 us - 21.34 us | 19.56 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_8` | 142.49 us | 61.00 us - 248.85 us | 68.17 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_80` | 55.74 us | 43.55 us - 68.74 us | 58.41 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_803` | 21.15 us | 21.03 us - 21.27 us | 21.09 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_806` | 20.96 us | 20.01 us - 22.42 us | 20.19 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_808` | 20.01 us | 19.87 us - 20.15 us | 19.99 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_81` | 50.36 us | 39.27 us - 61.92 us | 46.37 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_815` | 19.70 us | 19.58 us - 19.83 us | 19.71 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_82` | 32.59 us | 31.64 us - 33.71 us | 31.97 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_820` | 22.12 us | 21.95 us - 22.30 us | 22.10 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_828` | 22.13 us | 20.81 us - 23.64 us | 21.29 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_83` | 31.20 us | 28.12 us - 35.56 us | 28.38 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_831` | 20.74 us | 20.12 us - 21.64 us | 20.21 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_84` | 48.32 us | 35.83 us - 63.91 us | 38.24 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_842` | 20.19 us | 19.84 us - 20.69 us | 19.90 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_85` | 33.02 us | 31.73 us - 34.62 us | 32.41 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_86` | 34.43 us | 31.20 us - 37.21 us | 35.96 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_87` | 62.35 us | 44.25 us - 83.00 us | 62.90 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_870` | 22.81 us | 22.28 us - 23.59 us | 22.54 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_873` | 21.91 us | 21.65 us - 22.17 us | 21.71 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_874` | 20.22 us | 19.29 us - 21.51 us | 19.30 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_875` | 21.05 us | 20.28 us - 21.90 us | 21.05 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_877` | 20.39 us | 19.62 us - 21.36 us | 19.44 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_89` | 33.73 us | 30.90 us - 38.80 us | 31.24 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_895` | 25.01 us | 22.47 us - 28.23 us | 22.43 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_896` | 22.35 us | 22.06 us - 22.79 us | 22.25 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_9` | 32.88 us | 30.20 us - 37.13 us | 31.17 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_90` | 38.39 us | 27.62 us - 57.61 us | 28.57 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_904` | 21.85 us | 20.91 us - 22.95 us | 21.02 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_909` | 21.85 us | 20.94 us - 23.09 us | 21.41 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_91` | 31.07 us | 30.30 us - 32.22 us | 30.70 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_913` | 22.27 us | 21.41 us - 23.50 us | 21.55 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_916` | 21.11 us | 21.03 us - 21.20 us | 21.10 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_918` | 20.36 us | 20.21 us - 20.53 us | 20.37 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_919` | 21.36 us | 21.33 us - 21.39 us | 21.35 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_92` | 37.34 us | 34.01 us - 41.53 us | 34.95 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_93` | 33.48 us | 30.13 us - 37.74 us | 30.62 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_94` | 35.87 us | 25.17 us - 47.96 us | 29.18 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_95` | 32.49 us | 31.20 us - 34.18 us | 31.47 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_96` | 34.66 us | 30.86 us - 39.38 us | 30.93 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_97` | 53.12 us | 31.55 us - 82.49 us | 32.97 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_9729` | 20.50 us | 19.83 us - 21.25 us | 20.47 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_9730` | 23.47 us | 22.26 us - 24.94 us | 22.43 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_9731` | 20.35 us | 20.21 us - 20.49 us | 20.33 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_9732` | 20.59 us | 20.15 us - 21.12 us | 20.21 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_9763` | 22.97 us | 22.59 us - 23.36 us | 22.94 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_978` | 23.96 us | 22.68 us - 25.44 us | 22.50 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_98` | 53.56 us | 42.44 us - 65.21 us | 48.19 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_99` | 41.40 us | 32.22 us - 52.77 us | 32.22 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_991` | 23.25 us | 22.04 us - 24.84 us | 22.10 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_999` | 20.70 us | 19.76 us - 22.13 us | 19.99 us | - | - |
| `promoted_slow_offender_score/replay_promoted_100` | 2.83 ms | 2.73 ms - 2.93 ms | 2.80 ms | -15.54% | - |
| `real_sign_pair/composed_scalar_cascades` | 26.57 ns | 26.26 ns - 26.99 ns | 26.31 ns | - | - |
| `real_sign_pair/paired_cascade` | 9.64 ns | 9.55 ns - 9.76 ns | 9.64 ns | - | - |

<!-- END COMPLETE BENCHMARK REPORT -->
