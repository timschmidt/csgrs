<!-- BEGIN promoted_slow_offender_score -->
## `promoted_slow_offender_score`

Deterministic lexicase score for Hyperlimit's retained fuzz offenders. The score is the average current best-of-five replay time; lower is better. Delta compares with the previous score, and derivative is the change in delta.

<!-- promoted_slow_score_nanos: 17956 -->
<!-- promoted_slow_previous_score_nanos: 19839 -->
<!-- promoted_slow_score_delta_nanos: -1883 -->

| Metric | Value |
| --- | ---: |
| Cases scored | 100 |
| Average score | 17.956 us |
| Delta | -1.883 us |
| Delta derivative | 4.857 us |

| Rank | Current Time | Fuzz target | Input |
| ---: | ---: | --- | --- |
| 1 | 19.448 us | `predicate_invariants` | `seed[1162]` |
| 2 | 19.279 us | `predicate_invariants` | `seed[2192]` |
| 3 | 19.268 us | `predicate_invariants` | `seed[895]` |
| 4 | 19.208 us | `predicate_invariants` | `seed[991]` |
| 5 | 19.169 us | `predicate_invariants` | `seed[1196]` |
| 6 | 19.149 us | `predicate_invariants` | `seed[1184]` |
| 7 | 19.118 us | `predicate_invariants` | `seed[1498]` |
| 8 | 19.059 us | `predicate_invariants` | `seed[919]` |
| 9 | 18.999 us | `predicate_invariants` | `seed[9763]` |
| 10 | 18.909 us | `predicate_invariants` | `seed[1517]` |

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
| `classify_point_line_fixed` | `easy` | `hyperreal_oriented` | 8.85 us | 1.00x |
| `classify_point_line_fixed` | `easy` | `hyperreal` | 11.68 us | 1.32x |
| `classify_point_line_fixed` | `near_degenerate` | `hyperreal_oriented` | 8.49 us | 1.00x |
| `classify_point_line_fixed` | `near_degenerate` | `hyperreal` | 10.87 us | 1.28x |
| `classify_point_oriented_plane` | `easy` | `hyperreal_evidence` | 18.32 us | 1.00x |
| `classify_point_oriented_plane` | `easy` | `hyperreal` | 35.34 us | 1.93x |
| `classify_point_oriented_plane` | `near_degenerate` | `hyperreal_evidence` | 18.56 us | 1.00x |
| `classify_point_oriented_plane` | `near_degenerate` | `hyperreal` | 35.67 us | 1.92x |
| `classify_point_plane` | `easy` | `hyperreal_evidence` | 8.24 us | 1.00x |
| `classify_point_plane` | `easy` | `hyperreal` | 18.01 us | 2.19x |
| `classify_point_plane` | `near_degenerate` | `hyperreal_evidence` | 7.04 us | 1.00x |
| `classify_point_plane` | `near_degenerate` | `hyperreal` | 17.38 us | 2.47x |
| `incircle2d` | `easy` | `apfp` | 2.40 us | 1.00x |
| `incircle2d` | `easy` | `geometry_predicates` | 2.81 us | 1.17x |
| `incircle2d` | `easy` | `robust` | 2.86 us | 1.19x |
| `incircle2d` | `easy` | `hyperreal_evidence` | 16.76 us | 6.99x |
| `incircle2d` | `easy` | `hyperreal` | 20.19 us | 8.42x |
| `incircle2d` | `near_degenerate` | `apfp` | 2.55 us | 1.00x |
| `incircle2d` | `near_degenerate` | `robust` | 2.88 us | 1.13x |
| `incircle2d` | `near_degenerate` | `geometry_predicates` | 2.89 us | 1.13x |
| `incircle2d` | `near_degenerate` | `hyperreal_evidence` | 16.84 us | 6.59x |
| `incircle2d` | `near_degenerate` | `hyperreal` | 20.66 us | 8.09x |
| `insphere3d` | `easy` | `robust` | 13.33 us | 1.00x |
| `insphere3d` | `easy` | `geometry_predicates` | 20.32 us | 1.52x |
| `insphere3d` | `easy` | `hyperreal_evidence` | 39.46 us | 2.96x |
| `insphere3d` | `easy` | `hyperreal` | 59.21 us | 4.44x |
| `insphere3d` | `near_degenerate` | `robust` | 13.41 us | 1.00x |
| `insphere3d` | `near_degenerate` | `geometry_predicates` | 20.25 us | 1.51x |
| `insphere3d` | `near_degenerate` | `hyperreal_evidence` | 39.88 us | 2.97x |
| `insphere3d` | `near_degenerate` | `hyperreal` | 60.79 us | 4.53x |
| `orient2d` | `easy` | `apfp` | 907.12 ns | 1.00x |
| `orient2d` | `easy` | `robust` | 1.15 us | 1.27x |
| `orient2d` | `easy` | `geometry_predicates` | 1.25 us | 1.37x |
| `orient2d` | `easy` | `hyperreal` | 11.06 us | 12.20x |
| `orient2d` | `near_degenerate` | `apfp` | 950.86 ns | 1.00x |
| `orient2d` | `near_degenerate` | `robust` | 1.18 us | 1.24x |
| `orient2d` | `near_degenerate` | `geometry_predicates` | 1.60 us | 1.68x |
| `orient2d` | `near_degenerate` | `hyperreal` | 10.88 us | 11.44x |
| `orient3d` | `easy` | `geometry_predicates` | 3.50 us | 1.00x |
| `orient3d` | `easy` | `robust` | 8.14 us | 2.33x |
| `orient3d` | `easy` | `hyperreal` | 39.01 us | 11.16x |
| `orient3d` | `near_degenerate` | `geometry_predicates` | 3.38 us | 1.00x |
| `orient3d` | `near_degenerate` | `robust` | 8.08 us | 2.39x |
| `orient3d` | `near_degenerate` | `hyperreal` | 34.46 us | 10.20x |

### All Criterion results

| Benchmark | Mean | 95% CI | Median | Change vs baseline | Throughput |
| --- | ---: | ---: | ---: | ---: | ---: |
| `aabb_immediate/2d/intersection_with_facts` | 55.03 ns | 54.90 ns - 55.21 ns | 54.78 ns | - | - |
| `aabb_immediate/2d/ordered_intersection_coordinates` | 13.82 ns | 13.78 ns - 13.86 ns | 13.75 ns | - | - |
| `aabb_immediate/2d/ordered_point_coordinates` | 12.99 ns | 12.93 ns - 13.06 ns | 12.91 ns | - | - |
| `aabb_immediate/3d/intersection` | 84.51 ns | 84.22 ns - 84.83 ns | 84.28 ns | - | - |
| `aabb_immediate/3d/ordered_contains` | 26.96 ns | 26.78 ns - 27.26 ns | 26.79 ns | - | - |
| `aabb_immediate/3d/ordered_intersection` | 27.03 ns | 26.94 ns - 27.13 ns | 26.88 ns | - | - |
| `aabb_immediate/3d/point` | 56.46 ns | 55.92 ns - 57.16 ns | 55.48 ns | - | - |
| `aabb_immediate/3d/relative_interior` | 30.23 ns | 30.16 ns - 30.32 ns | 30.11 ns | - | - |
| `batch_parallel/incircle2d/near_degenerate/rayon` | 112.59 us | 107.77 us - 118.03 us | 108.76 us | - | 8192 elements |
| `batch_parallel/incircle2d/near_degenerate/sequential` | 351.99 us | 349.51 us - 354.99 us | 349.22 us | - | 8192 elements |
| `batch_parallel/insphere3d/near_degenerate/rayon` | 214.74 us | 208.84 us - 221.66 us | 213.58 us | - | 8192 elements |
| `batch_parallel/insphere3d/near_degenerate/sequential` | 1.01 ms | 1.00 ms - 1.02 ms | 1.01 ms | - | 8192 elements |
| `batch_parallel/orient2d/near_degenerate/rayon` | 73.67 us | 71.31 us - 76.30 us | 72.65 us | - | 8192 elements |
| `batch_parallel/orient2d/near_degenerate/sequential` | 167.49 us | 165.66 us - 169.72 us | 166.48 us | - | 8192 elements |
| `batch_parallel/orient3d/near_degenerate/rayon` | 137.77 us | 133.08 us - 142.53 us | 139.78 us | - | 8192 elements |
| `batch_parallel/orient3d/near_degenerate/sequential` | 559.08 us | 557.12 us - 561.41 us | 556.96 us | - | 8192 elements |
| `certified_filters/ball_sign/rational` | 35.43 us | 35.16 us - 35.75 us | 34.78 us | - | - |
| `classify_point_line/hyperreal/easy` | 10.90 us | 10.88 us - 10.91 us | 10.89 us | - | - |
| `classify_point_line/hyperreal/near_degenerate` | 10.92 us | 10.90 us - 10.94 us | 10.87 us | - | - |
| `classify_point_line_fixed/hyperreal/easy` | 11.68 us | 11.65 us - 11.70 us | 11.64 us | - | - |
| `classify_point_line_fixed/hyperreal/near_degenerate` | 10.87 us | 10.85 us - 10.90 us | 10.83 us | - | - |
| `classify_point_line_fixed/hyperreal_oriented/easy` | 8.85 us | 8.85 us - 8.86 us | 8.85 us | - | - |
| `classify_point_line_fixed/hyperreal_oriented/near_degenerate` | 8.49 us | 8.46 us - 8.54 us | 8.42 us | - | - |
| `classify_point_oriented_plane/hyperreal/easy` | 35.34 us | 35.15 us - 35.56 us | 34.89 us | - | - |
| `classify_point_oriented_plane/hyperreal/near_degenerate` | 35.67 us | 35.45 us - 35.91 us | 35.18 us | - | - |
| `classify_point_oriented_plane/hyperreal_evidence/easy` | 18.32 us | 18.26 us - 18.39 us | 18.21 us | - | - |
| `classify_point_oriented_plane/hyperreal_evidence/near_degenerate` | 18.56 us | 18.48 us - 18.65 us | 18.39 us | - | - |
| `classify_point_plane/hyperreal/easy` | 18.01 us | 17.97 us - 18.05 us | 17.93 us | - | - |
| `classify_point_plane/hyperreal/near_degenerate` | 17.38 us | 17.31 us - 17.45 us | 17.26 us | - | - |
| `classify_point_plane/hyperreal_evidence/easy` | 8.24 us | 8.21 us - 8.27 us | 8.20 us | - | - |
| `classify_point_plane/hyperreal_evidence/near_degenerate` | 7.04 us | 7.00 us - 7.08 us | 6.98 us | - | - |
| `evidence_derivation/affine_det2_exact_word_filter_only` | 44.93 ns | 44.49 ns - 45.46 ns | 44.00 ns | - | - |
| `evidence_derivation/affine_det2_filter_only` | 11.07 ns | 11.00 ns - 11.15 ns | 10.90 ns | - | - |
| `evidence_derivation/affine_det3_exact_word_filter_only` | 206.99 ns | 205.22 ns - 208.98 ns | 203.40 ns | - | - |
| `evidence_derivation/affine_det3_filter_only` | 16.54 ns | 16.51 ns - 16.58 ns | 16.49 ns | - | - |
| `evidence_derivation/incircle2/dyadic_filter` | 1.72 us | 1.70 us - 1.73 us | 1.69 us | - | - |
| `evidence_derivation/incircle2d_filter_only` | 11.69 ns | 11.65 ns - 11.74 ns | 11.63 ns | - | - |
| `evidence_derivation/insphere3/dyadic_filter` | 3.36 us | 3.35 us - 3.37 us | 3.34 us | - | - |
| `evidence_derivation/insphere3d_filter_only` | 21.20 ns | 21.07 ns - 21.35 ns | 20.97 ns | - | - |
| `evidence_derivation/line2/dyadic_filter` | 804.00 ns | 800.34 ns - 808.00 ns | 796.13 ns | - | - |
| `evidence_derivation/line2/exact_rational_word_filter` | 817.59 ns | 812.72 ns - 825.13 ns | 809.87 ns | - | - |
| `evidence_derivation/linear_form3_filter_only` | 9.92 ns | 9.88 ns - 9.97 ns | 9.84 ns | - | - |
| `evidence_derivation/oriented_plane3/dyadic_filter` | 1.31 us | 1.30 us - 1.32 us | 1.29 us | - | - |
| `evidence_derivation/oriented_plane3/exact_rational_word_filter` | 1.61 us | 1.60 us - 1.63 us | 1.59 us | - | - |
| `evidence_derivation/plane3/dyadic_filter` | 875.28 ns | 869.95 ns - 881.16 ns | 863.90 ns | - | - |
| `evidence_derivation/point2/displacement_facts` | 114.25 ns | 113.82 ns - 114.72 ns | 113.32 ns | - | - |
| `exact_rational_kernels/affine_independent_d/4d_common_denominator` | 588.31 us | 586.34 us - 590.55 us | 584.94 us | - | - |
| `exact_rational_kernels/circle2/line_and_segment_relations` | 386.80 us | 385.15 us - 388.63 us | 383.53 us | - | - |
| `exact_rational_kernels/convex/halfspace_feasibility3_active_sets` | 15.81 us | 15.65 us - 16.04 us | 15.56 us | - | - |
| `exact_rational_kernels/convex/point_halfspace_composition` | 362.03 us | 359.98 us - 364.26 us | 358.41 us | - | - |
| `exact_rational_kernels/distance3/point_feature_scaled_thresholds` | 1.20 ms | 1.20 ms - 1.21 ms | 1.19 ms | - | - |
| `exact_rational_kernels/distance3/point_triangle_dyadic_thresholds` | 746.83 us | 741.96 us - 752.49 us | 737.46 us | - | - |
| `exact_rational_kernels/distance3/point_triangle_integer_thresholds` | 885.18 us | 883.34 us - 887.43 us | 882.80 us | - | - |
| `exact_rational_kernels/distance3/point_triangle_scaled_thresholds` | 1.19 ms | 1.19 ms - 1.20 ms | 1.19 ms | - | - |
| `exact_rational_kernels/distance_ordering/point2_non_dyadic` | 159.45 ns | 156.61 ns - 163.01 ns | 154.92 ns | - | - |
| `exact_rational_kernels/distance_ordering/point3_non_dyadic` | 259.65 ns | 258.10 ns - 261.45 ns | 256.62 ns | - | - |
| `exact_rational_kernels/homogeneous/three_plane_coordinate_triples` | 359.33 us | 358.13 us - 360.71 us | 357.22 us | - | - |
| `exact_rational_kernels/homogeneous/two_plane_line_then_plane` | 408.40 us | 406.25 us - 410.91 us | 403.80 us | - | - |
| `exact_rational_kernels/incircle2d/common_denominator` | 257.72 us | 255.46 us - 260.24 us | 253.66 us | - | - |
| `exact_rational_kernels/incircle2d/larger_rational_near_degenerate` | 1.32 ms | 1.31 ms - 1.32 ms | 1.30 ms | - | - |
| `exact_rational_kernels/insphere3d/common_denominator` | 1.04 ms | 1.03 ms - 1.04 ms | 1.03 ms | - | - |
| `exact_rational_kernels/insphere_d/4d_common_denominator` | 3.69 ms | 3.67 ms - 3.70 ms | 3.66 ms | - | - |
| `exact_rational_kernels/orient2d/common_denominator` | 48.02 us | 47.69 us - 48.40 us | 47.41 us | - | - |
| `exact_rational_kernels/orient2d/larger_rational_near_degenerate` | 42.90 us | 42.59 us - 43.24 us | 42.54 us | - | - |
| `exact_rational_kernels/orient3d/common_denominator` | 143.86 us | 143.55 us - 144.19 us | 143.45 us | - | - |
| `exact_rational_kernels/orient_d/4d_common_denominator` | 594.89 us | 592.08 us - 598.27 us | 590.70 us | - | - |
| `exact_rational_kernels/plane/aabb3_reports` | 947.42 us | 942.86 us - 952.39 us | 940.53 us | - | - |
| `exact_rational_kernels/ring/area_sign_non_dyadic` | 6.05 us | 6.02 us - 6.08 us | 6.00 us | - | - |
| `exact_rational_kernels/ring/even_odd_reports` | 211.97 us | 210.87 us - 213.21 us | 210.30 us | - | - |
| `exact_rational_kernels/segment3_intersection/mixed_exact_rational` | 340.03 us | 338.42 us - 341.75 us | 337.40 us | - | - |
| `exact_rational_kernels/triangle3/ray_intersection_reports` | 1.26 ms | 1.25 ms - 1.27 ms | 1.24 ms | - | - |
| `exact_rational_kernels/triangle3/segment_and_ray_intersections` | 2.74 ms | 2.73 ms - 2.75 ms | 2.71 ms | - | - |
| `exact_rational_kernels/triangle3/segment_intersection_reports` | 1.92 ms | 1.92 ms - 1.93 ms | 1.91 ms | - | - |
| `explicit_sphere_immediate/point` | 114.03 ns | 113.74 ns - 114.35 ns | 113.61 ns | - | - |
| `explicit_sphere_immediate/point2_distance_ordering_dyadic` | 36.95 ns | 36.71 ns - 37.24 ns | 36.52 ns | - | - |
| `explicit_sphere_immediate/point3_distance_ordering_dyadic` | 53.36 ns | 53.18 ns - 53.55 ns | 53.08 ns | - | - |
| `filter_cost_breakdown/orient2d/certified_filter` | 8.57 us | 8.53 us - 8.60 us | 8.48 us | - | - |
| `filter_cost_breakdown/orient2d/exact_view_6` | 7.63 us | 7.59 us - 7.67 us | 7.65 us | - | - |
| `filter_cost_breakdown/orient2d/lossy_cached_view_6` | 1.98 us | 1.98 us - 1.99 us | 1.98 us | - | - |
| `filter_cost_breakdown/orient2d/public_predicate` | 11.13 us | 11.10 us - 11.17 us | 11.07 us | - | - |
| `filter_cost_breakdown/orient2d/robust_arithmetic` | 1.13 us | 1.12 us - 1.14 us | 1.11 us | - | - |
| `halfspace3_immediate/feasible` | 14.00 us | 13.96 us - 14.05 us | 13.91 us | - | - |
| `halfspace3_immediate/infeasible` | 1.65 us | 1.65 us - 1.66 us | 1.65 us | - | - |
| `hypermesh_port_helpers/coplanar_triangles/projected_overlap` | 1.57 us | 1.57 us - 1.58 us | 1.56 us | - | - |
| `hypermesh_port_helpers/projected_parameters/exact_ratio` | 426.17 ns | 422.80 ns - 430.05 ns | 422.30 ns | - | - |
| `hypermesh_port_helpers/segment_plane/determinant_ratio` | 1.69 us | 1.68 us - 1.70 us | 1.69 us | - | - |
| `hypermesh_port_helpers/support_dop3/build_and_aabb_project` | 5.34 us | 5.32 us - 5.36 us | 5.34 us | - | - |
| `hypermesh_port_helpers/support_dop3/build_and_aabb_report` | 5.50 us | 5.49 us - 5.52 us | 5.50 us | - | - |
| `hypermesh_port_helpers/support_dop3/build_and_plane_report` | 13.11 us | 13.06 us - 13.16 us | 13.07 us | - | - |
| `hypermesh_port_helpers/triangle3_degeneracy/projected_orientations` | 28.53 ns | 28.48 ns - 28.61 ns | 28.45 ns | - | - |
| `hypermesh_port_helpers/triangle_plane/report_replay` | 887.34 ns | 880.19 ns - 895.74 ns | 872.29 ns | - | - |
| `hypermesh_port_helpers/triangle_triangle3/noncoplanar_report_replay` | 13.58 us | 13.51 us - 13.65 us | 13.62 us | - | - |
| `hypermesh_port_helpers/triangle_triangle3/report_replay` | 4.54 us | 4.51 us - 4.57 us | 4.50 us | - | - |
| `incircle2_immediate/point_with_evidence` | 31.75 ns | 31.70 ns - 31.81 ns | 31.71 ns | - | - |
| `incircle2d/apfp/easy` | 2.40 us | 2.40 us - 2.40 us | 2.39 us | - | - |
| `incircle2d/apfp/near_degenerate` | 2.55 us | 2.54 us - 2.57 us | 2.53 us | - | - |
| `incircle2d/geometry_predicates/easy` | 2.81 us | 2.81 us - 2.82 us | 2.81 us | - | - |
| `incircle2d/geometry_predicates/near_degenerate` | 2.89 us | 2.88 us - 2.89 us | 2.88 us | - | - |
| `incircle2d/hyperreal/easy` | 20.19 us | 20.12 us - 20.27 us | 20.04 us | - | - |
| `incircle2d/hyperreal/near_degenerate` | 20.66 us | 20.56 us - 20.76 us | 20.57 us | - | - |
| `incircle2d/hyperreal_evidence/easy` | 16.76 us | 16.70 us - 16.81 us | 16.65 us | - | - |
| `incircle2d/hyperreal_evidence/near_degenerate` | 16.84 us | 16.78 us - 16.90 us | 16.73 us | - | - |
| `incircle2d/robust/easy` | 2.86 us | 2.85 us - 2.88 us | 2.84 us | - | - |
| `incircle2d/robust/near_degenerate` | 2.88 us | 2.88 us - 2.88 us | 2.88 us | - | - |
| `insphere3_immediate/point_with_evidence` | 75.05 ns | 74.79 ns - 75.41 ns | 74.74 ns | - | - |
| `insphere3d/geometry_predicates/easy` | 20.32 us | 20.27 us - 20.38 us | 20.29 us | - | - |
| `insphere3d/geometry_predicates/near_degenerate` | 20.25 us | 20.20 us - 20.31 us | 20.27 us | - | - |
| `insphere3d/hyperreal/easy` | 59.21 us | 58.77 us - 59.71 us | 58.18 us | - | - |
| `insphere3d/hyperreal/near_degenerate` | 60.79 us | 60.44 us - 61.20 us | 59.94 us | - | - |
| `insphere3d/hyperreal_evidence/easy` | 39.46 us | 39.40 us - 39.54 us | 39.37 us | - | - |
| `insphere3d/hyperreal_evidence/near_degenerate` | 39.88 us | 39.71 us - 40.07 us | 39.51 us | - | - |
| `insphere3d/robust/easy` | 13.33 us | 13.31 us - 13.37 us | 13.31 us | - | - |
| `insphere3d/robust/near_degenerate` | 13.41 us | 13.34 us - 13.49 us | 13.26 us | - | - |
| `line2_immediate/point_with_orientation` | 14.69 ns | 14.64 ns - 14.73 ns | 14.60 ns | - | - |
| `nd_symbolic_scale/insphere_d/4d_dense_pi_center` | 25.13 us | 25.07 us - 25.22 us | 25.06 us | - | - |
| `nd_symbolic_scale/orient_d/4d_dense_pi` | 5.01 us | 4.99 us - 5.04 us | 4.97 us | - | - |
| `orient2d/apfp/easy` | 907.12 ns | 902.57 ns - 912.40 ns | 897.50 ns | - | - |
| `orient2d/apfp/near_degenerate` | 950.86 ns | 949.34 ns - 952.48 ns | 949.21 ns | - | - |
| `orient2d/geometry_predicates/easy` | 1.25 us | 1.24 us - 1.25 us | 1.23 us | - | - |
| `orient2d/geometry_predicates/near_degenerate` | 1.60 us | 1.59 us - 1.61 us | 1.58 us | - | - |
| `orient2d/hyperreal/easy` | 11.06 us | 11.04 us - 11.09 us | 11.01 us | - | - |
| `orient2d/hyperreal/near_degenerate` | 10.88 us | 10.87 us - 10.89 us | 10.88 us | - | - |
| `orient2d/robust/easy` | 1.15 us | 1.14 us - 1.17 us | 1.12 us | - | - |
| `orient2d/robust/near_degenerate` | 1.18 us | 1.17 us - 1.18 us | 1.17 us | - | - |
| `orient3d/geometry_predicates/easy` | 3.50 us | 3.47 us - 3.52 us | 3.45 us | - | - |
| `orient3d/geometry_predicates/near_degenerate` | 3.38 us | 3.37 us - 3.39 us | 3.36 us | - | - |
| `orient3d/hyperreal/easy` | 39.01 us | 38.91 us - 39.13 us | 38.89 us | - | - |
| `orient3d/hyperreal/near_degenerate` | 34.46 us | 34.40 us - 34.53 us | 34.36 us | - | - |
| `orient3d/robust/easy` | 8.14 us | 8.12 us - 8.15 us | 8.11 us | - | - |
| `orient3d/robust/near_degenerate` | 8.08 us | 8.06 us - 8.10 us | 8.05 us | - | - |
| `plane_composition_filters/segment/dyadic` | 124.25 us | 123.54 us - 125.07 us | 123.19 us | - | - |
| `plane_composition_filters/segment/exact_rational` | 209.34 us | 208.33 us - 210.45 us | 207.49 us | - | - |
| `plane_composition_filters/triangle/dyadic` | 39.70 us | 39.50 us - 39.92 us | 39.35 us | - | - |
| `plane_composition_filters/triangle/exact_rational` | 298.65 us | 297.66 us - 299.72 us | 296.47 us | - | - |
| `point2_equality/equal_exact_rational` | 5.95 ns | 5.93 ns - 5.98 ns | 5.91 ns | - | - |
| `point2_equality/unequal_y_exact_rational` | 13.54 ns | 13.48 ns - 13.61 ns | 13.41 ns | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1008` | 18.93 us | 18.10 us - 19.96 us | 17.90 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1009` | 22.48 us | 20.11 us - 25.69 us | 20.38 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1048` | 19.73 us | 19.62 us - 19.85 us | 19.67 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1117` | 19.67 us | 18.59 us - 21.31 us | 18.80 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1143` | 18.14 us | 18.09 us - 18.18 us | 18.16 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1162` | 19.90 us | 19.80 us - 20.03 us | 19.85 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1184` | 20.19 us | 19.72 us - 20.84 us | 19.77 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1196` | 20.13 us | 19.86 us - 20.55 us | 19.96 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1369` | 19.21 us | 18.29 us - 20.28 us | 18.55 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1401` | 18.33 us | 17.56 us - 19.45 us | 17.51 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1402` | 19.13 us | 18.35 us - 20.12 us | 18.55 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1433` | 20.13 us | 19.64 us - 20.67 us | 19.52 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1437` | 19.51 us | 19.26 us - 19.86 us | 19.32 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1498` | 20.38 us | 19.94 us - 20.92 us | 19.99 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1517` | 20.88 us | 20.11 us - 21.93 us | 20.12 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1539` | 18.24 us | 17.99 us - 18.52 us | 18.11 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1549` | 20.28 us | 19.71 us - 21.21 us | 19.77 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1557` | 18.15 us | 18.00 us - 18.29 us | 18.22 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1575` | 18.47 us | 18.18 us - 18.78 us | 18.38 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1576` | 18.22 us | 18.13 us - 18.32 us | 18.19 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1577` | 20.54 us | 19.86 us - 21.36 us | 19.85 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1586` | 18.81 us | 18.32 us - 19.57 us | 18.46 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1600` | 18.12 us | 17.92 us - 18.41 us | 18.02 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1612` | 18.47 us | 18.35 us - 18.61 us | 18.38 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1626` | 18.39 us | 18.32 us - 18.46 us | 18.43 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1640` | 18.39 us | 18.35 us - 18.43 us | 18.40 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1673` | 19.72 us | 19.63 us - 19.81 us | 19.70 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1675` | 20.29 us | 18.80 us - 21.97 us | 19.43 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1682` | 19.70 us | 19.17 us - 20.33 us | 19.22 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1701` | 19.62 us | 18.31 us - 21.08 us | 18.89 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1738` | 20.53 us | 19.56 us - 21.69 us | 20.07 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1750` | 18.88 us | 18.83 us - 18.94 us | 18.87 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1912` | 18.34 us | 18.27 us - 18.45 us | 18.29 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1937` | 19.86 us | 19.75 us - 19.99 us | 19.81 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1946` | 19.64 us | 19.37 us - 19.91 us | 19.53 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1950` | 21.81 us | 20.35 us - 23.55 us | 21.71 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1956` | 19.19 us | 19.13 us - 19.26 us | 19.18 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1965` | 19.31 us | 18.61 us - 20.19 us | 18.74 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1967` | 20.82 us | 20.13 us - 21.53 us | 20.70 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1977` | 19.38 us | 19.34 us - 19.43 us | 19.37 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1993` | 18.42 us | 18.23 us - 18.75 us | 18.26 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_1995` | 20.06 us | 19.60 us - 20.50 us | 20.25 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_2046` | 19.54 us | 18.75 us - 20.44 us | 18.76 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_2054` | 19.71 us | 18.93 us - 20.70 us | 19.23 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_2186` | 19.87 us | 19.39 us - 20.61 us | 19.44 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_2192` | 19.69 us | 19.61 us - 19.78 us | 19.67 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_2240` | 21.50 us | 20.16 us - 23.12 us | 20.63 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_2286` | 19.04 us | 18.85 us - 19.38 us | 18.90 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_2290` | 20.56 us | 19.82 us - 21.45 us | 20.06 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_2292` | 20.20 us | 19.10 us - 21.92 us | 19.32 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_2365` | 17.92 us | 17.88 us - 17.97 us | 17.92 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_3034` | 18.52 us | 18.05 us - 19.42 us | 18.08 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_3035` | 17.91 us | 17.68 us - 18.29 us | 17.70 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_5279` | 18.19 us | 18.11 us - 18.30 us | 18.11 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_531` | 18.72 us | 18.00 us - 19.66 us | 18.04 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_532` | 17.23 us | 17.04 us - 17.47 us | 17.07 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_5448` | 17.98 us | 17.67 us - 18.40 us | 17.69 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_5449` | 17.88 us | 17.78 us - 18.00 us | 17.82 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_6445` | 18.02 us | 17.92 us - 18.15 us | 17.98 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_719` | 19.14 us | 18.93 us - 19.35 us | 19.17 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_722` | 21.45 us | 20.74 us - 22.28 us | 21.24 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_750` | 18.65 us | 18.37 us - 18.96 us | 18.44 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_753` | 18.94 us | 18.37 us - 19.66 us | 18.56 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_754` | 18.86 us | 18.04 us - 20.28 us | 18.20 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_758` | 17.89 us | 17.85 us - 17.93 us | 17.88 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_761` | 18.27 us | 18.15 us - 18.43 us | 18.18 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_763` | 19.85 us | 19.68 us - 20.03 us | 19.81 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_774` | 19.11 us | 18.43 us - 19.92 us | 18.80 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_775` | 18.55 us | 18.51 us - 18.59 us | 18.55 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_781` | 19.08 us | 18.15 us - 20.41 us | 18.17 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_799` | 18.22 us | 17.92 us - 18.56 us | 17.92 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_803` | 18.97 us | 18.86 us - 19.09 us | 18.87 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_806` | 20.76 us | 19.34 us - 22.46 us | 19.85 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_808` | 18.20 us | 18.14 us - 18.27 us | 18.17 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_815` | 19.70 us | 18.73 us - 20.90 us | 18.63 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_820` | 20.26 us | 19.76 us - 20.82 us | 19.72 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_828` | 18.02 us | 17.86 us - 18.19 us | 18.00 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_831` | 19.37 us | 18.42 us - 20.87 us | 18.56 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_842` | 18.46 us | 18.03 us - 19.27 us | 18.08 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_870` | 19.83 us | 19.59 us - 20.19 us | 19.62 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_873` | 19.99 us | 19.50 us - 20.70 us | 19.53 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_874` | 17.91 us | 17.88 us - 17.94 us | 17.89 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_875` | 18.02 us | 17.70 us - 18.41 us | 17.69 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_877` | 18.16 us | 17.96 us - 18.43 us | 17.97 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_895` | 20.56 us | 20.09 us - 21.24 us | 20.18 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_896` | 21.35 us | 19.69 us - 23.47 us | 19.58 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_904` | 18.64 us | 18.51 us - 18.78 us | 18.61 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_909` | 20.66 us | 19.76 us - 21.57 us | 20.32 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_913` | 20.52 us | 19.84 us - 21.43 us | 20.09 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_916` | 19.63 us | 19.41 us - 19.96 us | 19.46 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_918` | 19.30 us | 18.88 us - 19.78 us | 19.08 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_919` | 20.23 us | 19.92 us - 20.64 us | 19.98 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_9729` | 18.21 us | 17.87 us - 18.68 us | 17.91 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_9730` | 19.80 us | 19.64 us - 20.05 us | 19.67 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_9731` | 18.56 us | 18.35 us - 18.80 us | 18.48 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_9732` | 19.29 us | 18.79 us - 19.91 us | 18.98 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_9763` | 19.73 us | 19.64 us - 19.82 us | 19.73 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_978` | 19.98 us | 19.87 us - 20.12 us | 19.90 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_991` | 20.56 us | 20.34 us - 20.80 us | 20.50 us | - | - |
| `promoted_fuzz_worst_performers/predicate_invariants_seed_999` | 18.07 us | 17.98 us - 18.16 us | 18.03 us | - | - |
| `promoted_slow_offender_score/replay_promoted_100` | 2.50 ms | 2.45 ms - 2.56 ms | 2.47 ms | - | - |
| `real_sign_pair/composed_scalar_cascades` | 30.46 ns | 30.25 ns - 30.71 ns | 30.10 ns | - | - |
| `real_sign_pair/paired_cascade` | 9.74 ns | 9.67 ns - 9.81 ns | 9.63 ns | - | - |
| `segment2_immediate/intersection_with_facts` | 125.98 ns | 125.40 ns - 126.67 ns | 124.78 ns | - | - |
| `segment2_immediate/point_with_facts` | 120.69 ns | 120.19 ns - 121.27 ns | 119.67 ns | - | - |
| `segment3_immediate/intersection` | 864.89 ns | 861.70 ns - 868.52 ns | 859.35 ns | - | - |
| `segment3_immediate/point` | 224.92 ns | 224.08 ns - 225.86 ns | 223.93 ns | - | - |
| `shared_scale_views/incircle2d/common_denominator_evidence` | 89.32 us | 88.80 us - 89.92 us | 88.30 us | - | - |
| `shared_scale_views/incircle2d/common_denominator_predicate` | 250.17 us | 248.85 us - 251.77 us | 247.66 us | - | - |
| `shared_scale_views/insphere3d/common_denominator_evidence` | 90.19 us | 89.63 us - 90.82 us | 89.25 us | - | - |
| `shared_scale_views/insphere3d/common_denominator_predicate` | 900.66 us | 896.86 us - 904.54 us | 897.86 us | - | - |
| `shared_scale_views/point2/common_denominator` | 90.92 ns | 90.69 ns - 91.20 ns | 90.53 ns | - | - |
| `shared_scale_views/point3/common_denominator` | 131.81 ns | 130.76 ns - 133.03 ns | 129.84 ns | - | - |
| `transformed_predicates/classify_point_line/oriented_exact_rational_affine` | 16.72 us | 16.60 us - 16.85 us | 16.52 us | - | - |
| `transformed_predicates/classify_point_oriented_plane/evidence_exact_rational_affine` | 32.03 us | 31.86 us - 32.23 us | 31.75 us | - | - |
| `transformed_predicates/incircle2d/evidence_exact_rational_affine` | 124.09 us | 123.95 us - 124.24 us | 124.07 us | - | - |
| `transformed_predicates/incircle2d/exact_rational_affine` | 303.97 us | 302.37 us - 305.73 us | 301.34 us | - | - |
| `transformed_predicates/orient2d/exact_rational_affine` | 50.79 us | 50.46 us - 51.18 us | 50.13 us | - | - |
| `triangle2_immediate/point_with_orientation` | 63.04 ns | 62.89 ns - 63.22 ns | 62.75 ns | - | - |
| `triangle3_immediate/point_with_orientation` | 1.27 us | 1.27 us - 1.28 us | 1.27 us | - | - |

<!-- END COMPLETE BENCHMARK REPORT -->
