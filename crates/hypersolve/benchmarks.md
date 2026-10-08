<!-- BEGIN promoted_slow_offender_score -->
## `promoted_slow_offender_score`

Deterministic lexicase score for Hypersolve's retained fuzz offenders. The score is the average current best-of-five replay time; lower is better. Delta compares with the previous score, and derivative is the change in delta.

<!-- promoted_slow_score_nanos: 28867 -->
<!-- promoted_slow_previous_score_nanos: 29584 -->
<!-- promoted_slow_score_delta_nanos: -717 -->

| Metric | Value |
| --- | ---: |
| Cases scored | 100 |
| Average score | 28.867 us |
| Delta | -717 ns |
| Delta derivative | -1.868 us |

| Rank | Current Time | Fuzz target | Input |
| ---: | ---: | --- | --- |
| 1 | 30.798 us | `sketch_projected_line_radius_equality` | `seed[92]` |
| 2 | 30.378 us | `sketch_projected_cubic_curve_cubic_curve_g2` | `seed[5]` |
| 3 | 30.118 us | `sketch_projected_line_length_range` | `seed[5]` |
| 4 | 30.048 us | `sketch_projected_line_orientation` | `seed[92]` |
| 5 | 29.728 us | `sketch_projected_arc_cubic_curve_tangent` | `seed[50]` |
| 6 | 29.698 us | `sketch_projected_distance` | `seed[17]` |
| 7 | 29.678 us | `sketch_projected_point_on_cubic` | `seed[17]` |
| 8 | 29.658 us | `sketch_projected_line_arc_sweep_length` | `seed[17]` |
| 9 | 29.568 us | `sketch_projected_point_on_cubic_curve` | `seed[17]` |
| 10 | 29.518 us | `sketch_projected_arc_cubic_curve_second_order_contact` | `seed[21]` |

<!-- END promoted_slow_offender_score -->








# Hypersolve Benchmarks

This file is updated automatically by the benchmark binaries.

<!-- BEGIN COMPLETE BENCHMARK REPORT -->
## Complete generated benchmark report

Every registered benchmark target is catalogued below. Every Criterion result found under `target/criterion` is included without a name or implementation filter; non-Criterion targets write their own linked reports. Each timing binary refreshes this section after it runs.

Run the complete non-instrumented timing set with:

```sh
cargo bench
```

Regenerate this Markdown from stored Criterion data without rerunning benchmarks:

```sh
cargo run --example write_benchmarks_md
```

### Registered benchmark suites

| Target | Kind | Required features | Command | Generated report |
| --- | --- | --- | --- | --- |
| `algebraic_fiber` | custom timing | `default` | `cargo bench --bench algebraic_fiber` | [algebraic_fiber_benchmarks.md](algebraic_fiber_benchmarks.md) |
| `certification` | Criterion timing | `default` | `cargo bench --bench certification` | this file |
| `competitive` | Criterion timing | `default` | `cargo bench --bench competitive` | this file |
| `dispatch_trace` | diagnostic | `dispatch-trace` | `cargo bench --bench dispatch_trace --features dispatch-trace` | [dispatch_trace.md](dispatch_trace.md) |
| `representations` | Criterion timing | `default` | `cargo bench --bench representations` | this file |
| `retained_fuzz` | Criterion timing | `default` | `cargo bench --bench retained_fuzz` | this file |
| `cgal_quadratic` | external exact comparison | `CGAL, GMP` | `benches/competitors/run_cgal_quadratic.sh` | [cgal_quadratic_benchmarks.md](cgal_quadratic_benchmarks.md) |

### Comparative results

Rows sharing a Criterion group and input are compared when they expose distinct implementations. Ratios are elapsed time relative to the fastest stored row; they do not imply identical guarantees or output semantics.

| Group | Input | Implementation | Mean | Relative to fastest |
| --- | --- | --- | ---: | ---: |
| `competitive/dense_linear_4x4` | `-` | `nalgebra_f64_lu_proposal` | 64.97 ns | 1.00x |
| `competitive/dense_linear_4x4` | `-` | `hypersolve_exact_with_replay` | 5.26 us | 81.00x |
| `competitive/quadratic_roots` | `-` | `roots_f64_proposal` | 3.55 ns | 1.00x |
| `competitive/quadratic_roots` | `-` | `hypersolve_exact_candidates` | 227.55 ns | 64.06x |

### All Criterion results

| Benchmark | Mean | 95% CI | Median | Change vs baseline | Throughput |
| --- | ---: | ---: | ---: | ---: | ---: |
| `algebraic_root_rational_map_transform` | 1.96 us | 1.96 us - 1.96 us | 1.96 us | - | - |
| `analyze_exact_affine_rank` | 1.22 us | 1.22 us - 1.23 us | 1.22 us | -0.99% | - |
| `analyze_sparse_bareiss_cyclic_row_swaps_64` | 76.99 us | 76.76 us - 77.31 us | 76.73 us | -0.09% | - |
| `analyze_sparse_bareiss_elimination_pattern` | 925.24 ns | 923.79 ns - 927.00 ns | 924.83 ns | -0.37% | - |
| `apply_equality_substitution_classes` | 360.76 ns | 359.87 ns - 361.80 ns | 359.49 ns | +3.25% | - |
| `arithmetic_algebraic_root_representations` | 93.03 ns | 92.78 ns - 93.32 ns | 92.62 ns | - | - |
| `arithmetic_algebraic_root_representations/exact_normal_point_divide` | 177.80 ns | 177.68 ns - 177.92 ns | 177.88 ns | - | - |
| `arithmetic_algebraic_root_representations/exact_real_points` | 185.29 ns | 184.90 ns - 185.77 ns | 184.47 ns | - | - |
| `arithmetic_algebraic_root_representations/mixed_exact_real_scalar` | 3.40 us | 3.39 us - 3.40 us | 3.40 us | - | - |
| `arithmetic_algebraic_root_representations/negate` | 113.39 ns | 113.27 ns - 113.55 ns | 113.25 ns | - | - |
| `arithmetic_algebraic_root_representations/same_quadratic_affine_square` | 453.95 ns | 453.61 ns - 454.31 ns | 454.07 ns | - | - |
| `arithmetic_algebraic_root_representations/same_root_add` | 290.70 ns | 290.12 ns - 291.33 ns | 290.42 ns | - | - |
| `arithmetic_algebraic_root_representations/same_root_multiply` | 83.98 ns | 83.88 ns - 84.08 ns | 83.82 ns | - | - |
| `arithmetic_algebraic_root_representations/same_root_touching_zero_divide` | 61.32 ns | 61.23 ns - 61.42 ns | 61.23 ns | - | - |
| `arithmetic_algebraic_root_representations/zero_dividend` | 51.28 ns | 51.24 ns - 51.32 ns | 51.27 ns | - | - |
| `arithmetic_algebraic_root_representations_mixed_scalar` | 413.35 ns | 412.01 ns - 414.80 ns | 411.08 ns | - | - |
| `audit_active_set` | 2.86 us | 2.85 us - 2.87 us | 2.87 us | - | - |
| `bernstein_subdivision/cluster/degree_2/depth_32` | 14.40 us | 14.38 us - 14.42 us | 14.38 us | - | - |
| `bernstein_subdivision/cluster/degree_4/depth_0` | 8.06 us | 8.05 us - 8.07 us | 8.06 us | - | - |
| `bernstein_subdivision/cluster/degree_4/depth_32` | 27.22 us | 27.19 us - 27.25 us | 27.21 us | - | - |
| `bernstein_subdivision/cluster/degree_8/depth_32` | 103.68 us | 103.58 us - 103.78 us | 103.58 us | - | - |
| `bernstein_subdivision/endpoint/degree_2/depth_8` | 2.04 us | 2.04 us - 2.05 us | 2.04 us | - | - |
| `bernstein_subdivision/endpoint/degree_4/depth_8` | 8.28 us | 8.26 us - 8.31 us | 8.24 us | - | - |
| `bernstein_subdivision/endpoint/degree_8/depth_16` | 30.34 us | 30.29 us - 30.41 us | 30.26 us | - | - |
| `bernstein_subdivision/outside/degree_1/depth_8` | 1.02 us | 1.01 us - 1.02 us | 1.02 us | - | - |
| `bernstein_subdivision/positive/degree_2/depth_24` | 2.97 us | 2.96 us - 2.98 us | 2.95 us | - | - |
| `bernstein_subdivision/positive/degree_4/depth_24` | 8.20 us | 8.17 us - 8.25 us | 8.17 us | - | - |
| `bernstein_subdivision/repeated/degree_2/depth_16` | 12.30 us | 12.28 us - 12.32 us | 12.28 us | - | - |
| `bernstein_subdivision/repeated/degree_4/depth_16` | 18.79 us | 18.78 us - 18.81 us | 18.80 us | - | - |
| `bernstein_subdivision/spread/degree_16/depth_16` | 198.60 us | 198.38 us - 198.81 us | 198.63 us | - | - |
| `bernstein_subdivision/spread/degree_2/depth_8` | 3.14 us | 3.13 us - 3.14 us | 3.13 us | - | - |
| `bernstein_subdivision/spread/degree_4/depth_8` | 8.58 us | 8.56 us - 8.60 us | 8.55 us | - | - |
| `bernstein_subdivision/spread/degree_8/depth_16` | 33.94 us | 33.90 us - 33.99 us | 33.91 us | - | - |
| `build_equality_substitution_classes_exact` | 5.54 us | 5.53 us - 5.55 us | 5.53 us | - | - |
| `certify_affine_candidate_exact` | 2.90 us | 2.89 us - 2.91 us | 2.89 us | - | - |
| `certify_affine_krawczyk_box` | 1.48 us | 1.48 us - 1.48 us | 1.48 us | -2.10% | - |
| `certify_candidate_batch_affine` | 47.47 us | 47.38 us - 47.57 us | 47.41 us | - | - |
| `certify_candidate_domains` | 46.69 us | 46.45 us - 46.98 us | 46.32 us | - | - |
| `certify_direct_univariate_quadratic_roots` | 69.62 us | 69.56 us - 69.68 us | 69.52 us | +1.18% | - |
| `certify_interval_box_candidate_report` | 14.80 us | 14.79 us - 14.82 us | 14.80 us | - | - |
| `certify_multivariate_quadratic_interval_rows` | 32.71 us | 32.66 us - 32.76 us | 32.64 us | - | - |
| `certify_multivariate_quadratic_krawczyk_square/1` | 1.71 us | 1.70 us - 1.71 us | 1.70 us | - | - |
| `certify_multivariate_quadratic_krawczyk_square/2` | 4.37 us | 4.37 us - 4.38 us | 4.37 us | - | - |
| `certify_multivariate_quadratic_krawczyk_square/4` | 29.63 us | 29.59 us - 29.68 us | 29.57 us | - | - |
| `certify_multivariate_quadratic_krawczyk_square/8` | 163.60 us | 163.20 us - 164.01 us | 163.28 us | - | - |
| `certify_quadratic_interval_rows` | 14.84 us | 14.81 us - 14.87 us | 14.79 us | - | - |
| `certify_univariate_quadratic_alpha_rows` | 14.17 us | 14.13 us - 14.24 us | 14.12 us | - | - |
| `certify_univariate_quadratic_krawczyk_rows` | 13.03 us | 13.00 us - 13.06 us | 13.02 us | - | - |
| `compare_algebraic_root_representations` | 53.63 ns | 53.50 ns - 53.77 ns | 53.39 ns | - | - |
| `compare_algebraic_root_representations/disjoint_intervals` | 26.44 ns | 26.35 ns - 26.54 ns | 26.26 ns | - | - |
| `compare_algebraic_root_representations/exact_rational_points` | 52.22 ns | 51.98 ns - 52.51 ns | 51.82 ns | - | - |
| `compare_algebraic_root_representations/exact_real_points` | 107.12 ns | 106.61 ns - 107.70 ns | 106.06 ns | - | - |
| `compare_algebraic_root_representations/wide_exact_point` | 57.73 ns | 57.56 ns - 57.92 ns | 57.70 ns | - | - |
| `compare_algebraic_root_representations_by_difference` | 22.75 us | 22.68 us - 22.83 us | 22.60 us | - | - |
| `compare_algebraic_root_representations_by_difference/disjoint_intervals` | 262.87 ns | 262.62 ns - 263.13 ns | 262.96 ns | - | - |
| `compare_algebraic_root_representations_by_difference/exact_point_common_root` | 499.54 ns | 497.68 ns - 501.74 ns | 496.68 ns | - | - |
| `compare_algebraic_root_representations_by_difference/exact_rational_points` | 285.26 ns | 285.00 ns - 285.54 ns | 284.92 ns | - | - |
| `compare_algebraic_root_representations_by_difference/same_representation` | 279.99 ns | 279.50 ns - 280.58 ns | 279.32 ns | - | - |
| `compare_algebraic_root_representations_with_refinement` | 1.30 us | 1.29 us - 1.30 us | 1.29 us | - | - |
| `compare_algebraic_root_representations_with_refinement/exact_point_against_interval` | 3.05 us | 3.03 us - 3.07 us | 3.02 us | - | - |
| `compare_algebraic_root_representations_with_refinement/one_sided_exact_real_coefficients` | 4.53 us | 4.51 us - 4.54 us | 4.50 us | - | - |
| `competitive/dense_linear_4x4/hypersolve_exact_with_replay` | 5.26 us | 5.26 us - 5.27 us | 5.26 us | - | - |
| `competitive/dense_linear_4x4/nalgebra_f64_lu_proposal` | 64.97 ns | 64.85 ns - 65.14 ns | 64.84 ns | - | - |
| `competitive/quadratic_roots/hypersolve_exact_candidates` | 227.55 ns | 226.13 ns - 229.07 ns | 227.16 ns | - | - |
| `competitive/quadratic_roots/roots_f64_proposal` | 3.55 ns | 3.55 ns - 3.56 ns | 3.55 ns | - | - |
| `competitor_exact_quadratic_roots/hypersolve` | 217.18 ns | 217.05 ns - 217.32 ns | 217.15 ns | -0.67% | - |
| `count_bernstein_univariate_polynomial_interval_roots` | 42.89 us | 42.82 us - 42.98 us | 42.83 us | - | - |
| `count_bivariate_common_fiber_degree_drop` | 39.38 us | 39.34 us - 39.43 us | 39.36 us | - | - |
| `count_bivariate_fiber_roots_closed/repeated_endpoints` | 9.04 us | 9.03 us - 9.05 us | 9.03 us | - | - |
| `count_bivariate_fiber_roots_even_multiplicity` | 12.75 us | 12.71 us - 12.80 us | 12.70 us | - | - |
| `count_bivariate_fiber_roots_intervals/adjacent_64` | 320.38 us | 319.44 us - 321.52 us | 318.98 us | - | - |
| `count_bivariate_fiber_roots_intervals/lower_endpoint_64` | 4.40 us | 4.38 us - 4.42 us | 4.37 us | - | - |
| `count_bivariate_fiber_roots_intervals/shared_lower_64` | 296.83 us | 296.25 us - 297.50 us | 296.25 us | - | - |
| `count_descartes_univariate_polynomial_roots` | 18.06 us | 18.04 us - 18.09 us | 18.03 us | - | - |
| `deflate_bivariate_fiber_diagonal_root/triple_root` | 6.14 us | 6.13 us - 6.15 us | 6.13 us | +0.51% | - |
| `deflate_bivariate_fiber_diagonal_root/triple_root_second_parameter` | 5.98 us | 5.97 us - 5.99 us | 5.97 us | +1.00% | - |
| `dense_tensor_reduce_axis_modulo/already_reduced_2x64x64` | 95.69 us | 95.58 us - 95.85 us | 95.60 us | -1.77% | - |
| `dense_tensor_reduce_axis_modulo/compact_8x2x2` | 7.37 us | 7.36 us - 7.39 us | 7.36 us | +5.08% | - |
| `dense_tensor_reduce_axis_modulo/padded_8x64x64` | 135.25 us | 135.19 us - 135.31 us | 135.27 us | +2.27% | - |
| `determinant_bareiss` | 343.72 ns | 343.06 ns - 344.51 ns | 342.68 ns | -2.76% | - |
| `determinant_bareiss_pivot_free_terminal_2` | 139.01 ns | 138.94 ns - 139.09 ns | 138.96 ns | +0.72% | - |
| `diagnose_failed_constraints_affine` | 1.04 ms | 1.04 ms - 1.05 ms | 1.03 ms | - | - |
| `diagnose_sketch_failed_constraints` | 114.14 us | 113.96 us - 114.34 us | 113.97 us | - | - |
| `divide_bivariate_polynomial_exact` | 868.02 ns | 865.42 ns - 871.32 ns | 864.69 ns | -2.28% | - |
| `divide_univariate_polynomial_exact/dense_degree_48_by_16` | 33.08 us | 32.96 us - 33.21 us | 32.90 us | +2.07% | - |
| `divide_univariate_polynomial_exact/exact_real_quadratic_by_linear` | 6.90 us | 6.88 us - 6.93 us | 6.88 us | +0.00% | - |
| `domain_geometry_squared_distance_build` | 539.40 ns | 538.03 ns - 540.93 ns | 537.84 ns | - | - |
| `eliminate_affine_rows_with_substitution_classes` | 7.19 us | 7.17 us - 7.22 us | 7.17 us | -2.22% | - |
| `enumerate_direct_univariate_quadratic_branches` | 68.61 us | 68.51 us - 68.74 us | 68.48 us | -1.07% | - |
| `evaluate_polynomial_at_algebraic_root` | 316.53 ns | 315.86 ns - 317.32 ns | 315.78 ns | - | - |
| `evaluate_polynomial_at_algebraic_root/degree_16_interval` | 2.66 us | 2.66 us - 2.66 us | 2.65 us | - | - |
| `evaluate_polynomial_at_algebraic_root/exact_normal_constant` | 94.77 ns | 94.40 ns - 95.20 ns | 94.13 ns | - | - |
| `evaluate_polynomial_at_algebraic_root/exact_rational_point` | 210.42 ns | 210.23 ns - 210.62 ns | 210.30 ns | - | - |
| `evaluate_polynomial_at_algebraic_root/exact_real_point` | 243.18 ns | 242.60 ns - 243.85 ns | 242.15 ns | - | - |
| `evaluate_rational_expression_at_algebraic_root` | 522.33 ns | 520.77 ns - 524.18 ns | 520.01 ns | - | - |
| `evaluate_rational_expression_at_algebraic_root/exact_normal_denominator` | 238.83 ns | 238.61 ns - 239.07 ns | 238.79 ns | - | - |
| `evaluate_rational_expression_at_algebraic_root/exact_rational_point` | 260.38 ns | 260.14 ns - 260.65 ns | 260.19 ns | - | - |
| `evaluate_rational_expression_at_algebraic_root/exact_real_point` | 344.46 ns | 344.02 ns - 344.99 ns | 343.98 ns | - | - |
| `evaluate_rational_expression_at_algebraic_root/same_exact_real_value` | 307.43 ns | 307.11 ns - 307.83 ns | 307.04 ns | - | - |
| `evaluate_rational_expression_at_algebraic_root/zero_over_exact_normal_denominator` | 174.13 ns | 173.69 ns - 174.63 ns | 173.61 ns | - | - |
| `isolate_bivariate_fiber_roots/bernstein_rational_deflation` | 18.41 us | 18.36 us - 18.47 us | 18.32 us | -3.12% | - |
| `isolate_bivariate_fiber_roots/partitioned_rational_sturm_fallback` | 408.61 us | 407.57 us - 409.85 us | 407.03 us | -0.40% | - |
| `isolate_bivariate_fiber_roots/repeated_irrational_sturm_fallback` | 31.27 us | 31.24 us - 31.32 us | 31.24 us | -2.97% | - |
| `isolate_univariate_polynomial_roots_sturm` | 194.95 us | 194.72 us - 195.20 us | 194.63 us | -2.38% | - |
| `multivariate_quadratic_row_forms` | 28.81 us | 28.74 us - 28.90 us | 28.73 us | -5.64% | - |
| `polynomial_has_one_distinct_root_in_open_interval/bernstein_degree_16` | 18.44 us | 18.42 us - 18.46 us | 18.43 us | - | - |
| `polynomial_has_one_distinct_root_in_open_interval/endpoint_and_interior` | 217.68 ns | 217.38 ns - 218.04 ns | 217.41 ns | - | - |
| `polynomial_has_one_distinct_root_in_open_interval/monotone_degree_16` | 876.83 ns | 874.84 ns - 879.38 ns | 873.45 ns | - | - |
| `polynomial_has_one_distinct_root_in_open_interval/quadratic_sign` | 79.21 ns | 79.06 ns - 79.38 ns | 79.04 ns | - | - |
| `polynomial_has_one_distinct_root_in_open_interval/repeated_exact_real` | 7.59 us | 7.58 us - 7.61 us | 7.59 us | - | - |
| `polynomial_has_one_distinct_root_in_open_interval/repeated_quadratic` | 511.80 ns | 510.92 ns - 512.96 ns | 510.54 ns | - | - |
| `polynomial_has_one_distinct_root_in_open_interval/sturm_fallback_cubic` | 7.81 us | 7.80 us - 7.83 us | 7.80 us | - | - |
| `project_algebraic_fiber_polynomial_image` | 6.04 us | 6.03 us - 6.05 us | 6.03 us | - | - |
| `project_algebraic_fiber_polynomial_image/cubic_source_quadratic_image` | 19.83 us | 19.80 us - 19.87 us | 19.81 us | - | - |
| `project_algebraic_fiber_polynomial_image/saturated_conjugate_factor` | 86.79 us | 86.37 us - 87.27 us | 86.02 us | - | - |
| `project_algebraic_fiber_polynomial_image/saturated_fourfold_multicoefficient` | 2.23 ms | 2.22 ms - 2.23 ms | 2.21 ms | - | - |
| `project_algebraic_fiber_polynomial_image/strict_exact_zero_image_degree` | 11.94 us | 11.41 us - 12.98 us | 11.41 us | - | - |
| `project_bivariate_fiber/retained_first_quadratic` | 1.92 us | 1.92 us - 1.93 us | 1.91 us | -3.81% | - |
| `project_bivariate_fiber/retained_second_quadratic` | 1.77 us | 1.77 us - 1.77 us | 1.77 us | -5.51% | - |
| `project_selected_tensor_fiber_via_tagged_norm/distinct_quadratic_carriers` | 204.77 us | 204.22 us - 205.46 us | 204.08 us | -1.36% | - |
| `project_selected_tensor_fiber_via_tagged_norm/opposite_conjugate_cubic` | 199.53 us | 199.15 us - 199.98 us | 199.09 us | -2.76% | - |
| `project_selected_tensor_fiber_via_tagged_norm/repeated_degree_six_carriers` | 212.95 us | 212.63 us - 213.32 us | 212.44 us | -2.45% | - |
| `promoted_fuzz_worst_performers/failed_constraint_minimal_removals_seed_10` | 26.83 us | 26.82 us - 26.85 us | 26.84 us | - | - |
| `promoted_fuzz_worst_performers/failed_constraint_minimal_removals_seed_14` | 27.22 us | 26.33 us - 28.58 us | 26.36 us | - | - |
| `promoted_fuzz_worst_performers/failed_constraint_minimal_removals_seed_15` | 26.63 us | 26.58 us - 26.70 us | 26.58 us | - | - |
| `promoted_fuzz_worst_performers/failed_constraint_minimal_removals_seed_18` | 26.73 us | 26.68 us - 26.77 us | 26.71 us | - | - |
| `promoted_fuzz_worst_performers/failed_constraint_minimal_removals_seed_19` | 27.72 us | 26.99 us - 28.89 us | 27.13 us | - | - |
| `promoted_fuzz_worst_performers/failed_constraint_minimal_removals_seed_2` | 27.79 us | 27.12 us - 28.75 us | 27.36 us | - | - |
| `promoted_fuzz_worst_performers/failed_constraint_minimal_removals_seed_23` | 26.88 us | 26.52 us - 27.28 us | 26.48 us | - | - |
| `promoted_fuzz_worst_performers/failed_constraint_minimal_removals_seed_58` | 26.96 us | 26.84 us - 27.10 us | 26.87 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_second_order_contact_seed_12` | 31.77 us | 30.42 us - 33.81 us | 30.53 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_second_order_contact_seed_13` | 30.65 us | 30.57 us - 30.75 us | 30.63 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_second_order_contact_seed_20` | 29.93 us | 29.85 us - 30.02 us | 29.86 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_second_order_contact_seed_21` | 30.91 us | 30.39 us - 31.70 us | 30.48 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_second_order_contact_seed_22` | 32.62 us | 32.04 us - 33.26 us | 32.60 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_second_order_contact_seed_3` | 30.61 us | 29.83 us - 32.09 us | 29.95 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_second_order_contact_seed_7` | 30.44 us | 30.32 us - 30.56 us | 30.47 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_second_order_contact_seed_8` | 30.15 us | 30.06 us - 30.26 us | 30.11 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_second_order_contact_seed_9` | 30.11 us | 29.88 us - 30.35 us | 30.06 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_tangent_seed_21` | 30.00 us | 29.91 us - 30.12 us | 29.95 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_tangent_seed_22` | 30.17 us | 30.05 us - 30.30 us | 30.11 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_tangent_seed_23` | 30.25 us | 29.95 us - 30.73 us | 30.02 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_tangent_seed_50` | 30.32 us | 30.24 us - 30.41 us | 30.29 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_tangent_seed_6` | 30.32 us | 30.20 us - 30.45 us | 30.32 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_tangent_seed_9` | 30.75 us | 30.59 us - 30.93 us | 30.63 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_line_tangent_seed_12` | 30.15 us | 29.97 us - 30.46 us | 30.01 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_line_tangent_seed_6` | 29.99 us | 29.64 us - 30.44 us | 29.77 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_line_tangent_seed_7` | 30.03 us | 29.91 us - 30.19 us | 29.92 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_line_tangent_seed_9` | 31.03 us | 30.90 us - 31.18 us | 30.93 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_c2_seed_9` | 30.25 us | 29.76 us - 30.92 us | 29.84 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_g2_seed_22` | 32.08 us | 31.30 us - 33.04 us | 31.49 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_g2_seed_42` | 31.08 us | 30.45 us - 31.87 us | 30.60 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_g2_seed_5` | 31.22 us | 31.04 us - 31.42 us | 31.17 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_g2_seed_7` | 30.07 us | 29.45 us - 31.21 us | 29.48 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_g2_seed_9` | 32.70 us | 31.11 us - 34.83 us | 31.22 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_tangent_seed_21` | 31.59 us | 30.08 us - 33.71 us | 30.11 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_tangent_seed_6` | 30.98 us | 30.75 us - 31.31 us | 30.81 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_tangent_seed_9` | 30.77 us | 30.72 us - 30.81 us | 30.77 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_line_tangent_seed_17` | 30.31 us | 30.17 us - 30.49 us | 30.26 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_line_tangent_seed_21` | 30.90 us | 30.04 us - 32.05 us | 30.05 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_line_tangent_seed_47` | 30.36 us | 30.25 us - 30.52 us | 30.32 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_line_tangent_seed_9` | 30.50 us | 30.35 us - 30.64 us | 30.50 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_line_tangent_seed_20` | 30.87 us | 30.85 us - 30.89 us | 30.86 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_line_tangent_seed_21` | 30.63 us | 30.53 us - 30.72 us | 30.61 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_line_tangent_seed_22` | 30.79 us | 30.71 us - 30.88 us | 30.78 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_line_tangent_seed_23` | 32.06 us | 30.82 us - 33.52 us | 30.85 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_line_tangent_seed_6` | 30.58 us | 30.51 us - 30.65 us | 30.56 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_line_tangent_seed_9` | 31.64 us | 30.46 us - 33.28 us | 30.65 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_distance_range_seed_1` | 30.81 us | 29.90 us - 31.93 us | 30.06 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_distance_range_seed_10` | 30.43 us | 30.22 us - 30.72 us | 30.29 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_distance_range_seed_8` | 30.77 us | 30.71 us - 30.86 us | 30.73 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_distance_range_seed_9` | 31.03 us | 30.09 us - 32.09 us | 30.35 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_distance_seed_17` | 30.12 us | 29.94 us - 30.30 us | 30.08 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_distance_seed_22` | 30.22 us | 30.12 us - 30.35 us | 30.19 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_distance_seed_6` | 30.37 us | 29.87 us - 31.06 us | 29.93 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_distance_seed_9` | 31.04 us | 29.99 us - 32.65 us | 29.98 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_equal_length_seed_28` | 33.25 us | 31.60 us - 34.93 us | 32.63 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_equal_point_line_distances_seed_19` | 30.80 us | 30.63 us - 30.98 us | 30.68 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_equal_point_line_distances_seed_9` | 30.52 us | 30.42 us - 30.65 us | 30.44 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_equal_point_point_distances_seed_6` | 30.23 us | 30.08 us - 30.47 us | 30.16 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_equal_point_point_distances_seed_9` | 31.01 us | 30.73 us - 31.41 us | 30.77 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_difference_seed_20` | 30.33 us | 30.09 us - 30.70 us | 30.11 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_difference_seed_246` | 29.97 us | 29.90 us - 30.04 us | 29.93 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_difference_seed_6` | 29.43 us | 29.32 us - 29.59 us | 29.38 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_point_line_distance_seed_20` | 30.85 us | 30.79 us - 30.91 us | 30.81 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_point_line_distance_seed_6` | 31.53 us | 30.95 us - 32.18 us | 30.91 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_ratio_seed_17` | 30.58 us | 30.43 us - 30.79 us | 30.51 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_ratio_seed_22` | 30.43 us | 30.29 us - 30.58 us | 30.37 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_ratio_seed_7` | 29.56 us | 29.32 us - 29.85 us | 29.49 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_ratio_seed_71` | 30.10 us | 29.74 us - 30.55 us | 29.80 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_arc_sweep_length_seed_17` | 29.81 us | 29.77 us - 29.86 us | 29.81 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_arc_sweep_length_seed_19` | 29.95 us | 29.82 us - 30.12 us | 29.83 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_circle_tangent_seed_17` | 30.78 us | 30.68 us - 30.90 us | 30.70 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_length_range_seed_22` | 30.21 us | 30.15 us - 30.28 us | 30.19 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_length_range_seed_5` | 30.20 us | 30.15 us - 30.25 us | 30.15 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_length_range_seed_8` | 29.89 us | 29.71 us - 30.09 us | 29.80 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_orientation_seed_1` | 30.12 us | 29.93 us - 30.37 us | 29.95 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_orientation_seed_92` | 30.97 us | 30.85 us - 31.14 us | 30.91 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_radius_equality_seed_7` | 32.26 us | 30.33 us - 34.83 us | 30.51 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_radius_equality_seed_92` | 28.83 us | 28.77 us - 28.89 us | 28.80 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_oriented_angle_seed_121` | 30.09 us | 29.94 us - 30.28 us | 29.97 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_oriented_angle_seed_20` | 30.72 us | 30.61 us - 30.85 us | 30.73 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_concentric_seed_18` | 30.73 us | 30.66 us - 30.80 us | 30.70 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_distance_difference_seed_17` | 30.51 us | 30.23 us - 30.84 us | 30.24 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_distance_ratio_seed_136` | 30.91 us | 29.84 us - 32.18 us | 29.87 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_distance_ratio_seed_21` | 31.26 us | 30.93 us - 31.72 us | 31.09 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_distance_ratio_seed_22` | 30.30 us | 30.16 us - 30.49 us | 30.22 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_distance_ratio_seed_8` | 30.12 us | 29.86 us - 30.42 us | 29.94 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_line_distance_range_seed_2` | 31.71 us | 31.32 us - 32.30 us | 31.39 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_line_radius_equality_seed_7` | 29.84 us | 29.68 us - 30.02 us | 29.71 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_arc_seed_17` | 31.25 us | 30.88 us - 31.69 us | 31.03 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_arc_seed_8` | 30.05 us | 29.79 us - 30.39 us | 29.88 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_circle_seed_2` | 29.95 us | 29.86 us - 30.05 us | 29.91 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_circle_seed_6` | 30.83 us | 30.57 us - 31.14 us | 30.63 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_cubic_curve_seed_17` | 31.39 us | 31.31 us - 31.48 us | 31.37 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_cubic_curve_seed_21` | 30.43 us | 30.12 us - 30.92 us | 30.20 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_cubic_curve_seed_8` | 30.88 us | 29.94 us - 31.95 us | 29.88 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_cubic_seed_17` | 31.11 us | 31.08 us - 31.15 us | 31.12 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_cubic_seed_18` | 30.41 us | 30.23 us - 30.61 us | 30.35 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_cubic_seed_22` | 30.39 us | 30.22 us - 30.60 us | 30.37 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_cubic_seed_6` | 30.80 us | 30.59 us - 31.11 us | 30.68 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_radius_equality_seed_17` | 30.73 us | 30.47 us - 31.05 us | 30.56 us | - | - |
| `promoted_slow_offender_score/replay_promoted_100` | 3.19 ms | 3.19 ms - 3.20 ms | 3.19 ms | - | - |
| `propose_active_set_update` | 3.29 us | 3.28 us - 3.30 us | 3.27 us | - | - |
| `quadratic_extraction/analyze_multi16` | 28.62 us | 28.51 us - 28.75 us | 28.45 us | - | - |
| `quadratic_extraction/analyze_uni16` | 41.46 us | 41.38 us - 41.57 us | 41.38 us | - | - |
| `quadratic_extraction/multi_bad_degree` | 584.80 ns | 584.18 ns - 585.46 ns | 584.11 ns | - | - |
| `quadratic_extraction/multi_bad_monomial` | 463.12 ns | 462.29 ns - 464.44 ns | 462.18 ns | - | - |
| `quadratic_extraction/multi_cancellation` | 881.71 ns | 880.25 ns - 883.54 ns | 880.05 ns | - | - |
| `quadratic_extraction/multi_cross` | 1.16 us | 1.15 us - 1.16 us | 1.15 us | - | - |
| `quadratic_extraction/multi_dense32` | 159.92 us | 159.53 us - 160.29 us | 159.92 us | - | - |
| `quadratic_extraction/multi_dense8` | 8.97 us | 8.96 us - 8.98 us | 8.97 us | - | - |
| `quadratic_extraction/multi_pi` | 923.96 ns | 921.35 ns - 927.31 ns | 920.81 ns | - | - |
| `quadratic_extraction/multi_square` | 467.23 ns | 466.16 ns - 468.43 ns | 466.48 ns | - | - |
| `quadratic_extraction/multi_zero_terms` | 641.16 ns | 640.36 ns - 642.08 ns | 640.31 ns | - | - |
| `quadratic_extraction/uni_bad_degree` | 965.74 ns | 964.37 ns - 967.29 ns | 964.02 ns | - | - |
| `quadratic_extraction/uni_cancellation` | 1.48 us | 1.48 us - 1.49 us | 1.48 us | - | - |
| `quadratic_extraction/uni_factored` | 772.13 ns | 769.67 ns - 775.10 ns | 768.84 ns | - | - |
| `quadratic_extraction/uni_pi` | 809.96 ns | 807.09 ns - 813.50 ns | 807.34 ns | - | - |
| `quadratic_extraction/uni_scaled_cancellation` | 2.52 us | 2.51 us - 2.53 us | 2.51 us | - | - |
| `quadratic_extraction/uni_square` | 568.43 ns | 567.54 ns - 569.83 ns | 567.78 ns | - | - |
| `quadratic_extraction/uni_wide` | 620.24 ns | 619.45 ns - 621.05 ns | 620.58 ns | - | - |
| `quadratic_form_candidate_replay` | 9.04 us | 9.01 us - 9.09 us | 9.00 us | - | - |
| `real_representations/full_solver_boundary/ConstOffset` | 45.27 us | 45.15 us - 45.42 us | 45.16 us | - | - |
| `real_representations/full_solver_boundary/ConstProduct` | 11.74 us | 11.71 us - 11.78 us | 11.74 us | - | - |
| `real_representations/full_solver_boundary/ConstProductSqrt` | 12.50 us | 12.49 us - 12.52 us | 12.49 us | - | - |
| `real_representations/full_solver_boundary/Exp` | 9.11 us | 9.08 us - 9.15 us | 9.10 us | - | - |
| `real_representations/full_solver_boundary/Irrational` | 40.84 us | 40.68 us - 41.00 us | 40.83 us | - | - |
| `real_representations/full_solver_boundary/Ln` | 40.09 us | 39.88 us - 40.40 us | 39.91 us | - | - |
| `real_representations/full_solver_boundary/LnAffine` | 41.52 us | 41.40 us - 41.65 us | 41.48 us | - | - |
| `real_representations/full_solver_boundary/LnProduct` | 44.52 us | 43.82 us - 45.28 us | 44.31 us | - | - |
| `real_representations/full_solver_boundary/Log10` | 43.06 us | 42.22 us - 44.17 us | 42.61 us | - | - |
| `real_representations/full_solver_boundary/Log2` | 44.40 us | 42.86 us - 46.17 us | 43.26 us | - | - |
| `real_representations/full_solver_boundary/One` | 5.57 us | 5.56 us - 5.58 us | 5.57 us | - | - |
| `real_representations/full_solver_boundary/Pi` | 7.32 us | 7.30 us - 7.33 us | 7.31 us | - | - |
| `real_representations/full_solver_boundary/PiExp` | 9.90 us | 9.87 us - 9.93 us | 9.89 us | - | - |
| `real_representations/full_solver_boundary/PiInv` | 7.88 us | 7.83 us - 7.96 us | 7.85 us | - | - |
| `real_representations/full_solver_boundary/PiInvExp` | 10.15 us | 10.07 us - 10.27 us | 10.08 us | - | - |
| `real_representations/full_solver_boundary/PiPow` | 8.21 us | 8.20 us - 8.21 us | 8.21 us | - | - |
| `real_representations/full_solver_boundary/PiSqrt` | 8.33 us | 8.31 us - 8.35 us | 8.32 us | - | - |
| `real_representations/full_solver_boundary/Pow10` | 27.25 us | 26.68 us - 27.81 us | 27.46 us | - | - |
| `real_representations/full_solver_boundary/Pow2` | 27.48 us | 26.93 us - 28.05 us | 27.08 us | - | - |
| `real_representations/full_solver_boundary/SinPi` | 45.40 us | 43.84 us - 47.16 us | 44.69 us | - | - |
| `real_representations/full_solver_boundary/Sqrt` | 6.84 us | 6.80 us - 6.90 us | 6.81 us | - | - |
| `real_representations/full_solver_boundary/TanPi` | 42.74 us | 42.15 us - 43.40 us | 42.19 us | - | - |
| `reduce_bivariate_rational_function/dense_degree_15_proportional` | 7.15 us | 7.13 us - 7.16 us | 7.13 us | - | - |
| `reduce_bivariate_rational_function/nonmonic_retained_denominator` | 15.65 us | 15.61 us - 15.69 us | 15.59 us | - | - |
| `refine_isolated_univariate_polynomial_interval/exact_linear_witness` | 126.94 ns | 126.66 ns - 127.29 ns | 126.53 ns | - | - |
| `refine_isolated_univariate_polynomial_interval/exact_quadratic_witness` | 142.68 ns | 142.57 ns - 142.81 ns | 142.61 ns | - | - |
| `refine_isolated_univariate_polynomial_interval/exact_real_coefficients_steps_4` | 1.20 us | 1.20 us - 1.21 us | 1.20 us | - | - |
| `refine_isolated_univariate_polynomial_interval/lower_endpoint_root` | 1.40 us | 1.40 us - 1.41 us | 1.39 us | - | - |
| `refine_isolated_univariate_polynomial_interval/quadratic_steps_4` | 641.51 ns | 640.61 ns - 642.48 ns | 640.82 ns | - | - |
| `refine_isolated_univariate_polynomial_interval/repeated_quadratic_steps_4` | 3.93 us | 3.92 us - 3.93 us | 3.92 us | - | - |
| `refine_isolated_univariate_polynomial_interval/upper_endpoint_root` | 1.50 us | 1.50 us - 1.50 us | 1.50 us | - | - |
| `refine_isolated_univariate_polynomial_interval/wide_exact_quadratic_witness` | 336.87 ns | 336.43 ns - 337.31 ns | 337.06 ns | - | - |
| `refine_isolated_univariate_polynomial_interval/width_already_satisfied` | 315.23 ns | 314.60 ns - 315.97 ns | 314.54 ns | - | - |
| `regenerate_active_set_affine_candidate` | 3.32 us | 3.31 us - 3.32 us | 3.31 us | - | - |
| `regenerate_active_set_quadratic_candidates` | 3.45 us | 3.45 us - 3.46 us | 3.44 us | - | - |
| `replay_dense_linear_residuals` | 313.75 ns | 312.62 ns - 315.11 ns | 312.07 ns | +1.10% | - |
| `replay_sparse_linear_residuals` | 698.71 ns | 697.19 ns - 700.40 ns | 696.94 ns | +3.65% | - |
| `report_lossy_adapter_only_candidate` | 592.65 ns | 591.26 ns - 594.34 ns | 590.71 ns | - | - |
| `represent_algebraic_tensor_image/sum_four_roots` | 182.41 us | 181.76 us - 183.22 us | 181.49 us | +0.46% | - |
| `represent_algebraic_tensor_image/sum_four_roots_padded_256` | 1.25 ms | 1.25 ms - 1.25 ms | 1.25 ms | +0.30% | - |
| `represent_algebraic_tensor_image/two_conjugate_cubic_roots` | 126.41 us | 126.19 us - 126.67 us | 126.01 us | -1.33% | - |
| `represent_algebraic_tensor_image/two_conjugate_repeated_degree_six_roots` | 141.20 us | 140.96 us - 141.47 us | 140.93 us | -0.26% | - |
| `represent_univariate_algebraic_roots` | 218.53 us | 218.05 us - 219.07 us | 217.85 us | -0.27% | - |
| `represented_root_sign/exact_rational_point` | 23.51 ns | 23.43 ns - 23.59 ns | 23.77 ns | - | - |
| `represented_root_sign/exact_real_point` | 57.62 ns | 57.59 ns - 57.65 ns | 57.63 ns | - | - |
| `represented_root_sign/isolating_interval` | 13.10 ns | 13.03 ns - 13.17 ns | 12.98 ns | - | - |
| `resultant_constant_degree_64` | 1.03 us | 1.03 us - 1.03 us | 1.03 us | -1.62% | - |
| `resultant_parametric_curve_intersection` | 1.12 us | 1.12 us - 1.12 us | 1.12 us | -2.41% | - |
| `resultant_rational_parametric_curve_intersection` | 1.35 us | 1.35 us - 1.35 us | 1.34 us | -2.02% | - |
| `resultant_tensor_polynomial_univariate_constraint` | 1.04 us | 1.04 us - 1.04 us | 1.04 us | -1.73% | - |
| `resultant_tensor_polynomial_univariate_constraint/compact_grid_2x2` | 6.32 us | 6.32 us - 6.33 us | 6.32 us | +0.18% | - |
| `resultant_tensor_polynomial_univariate_constraint/direct_real_norm` | 991.11 ns | 989.77 ns - 992.71 ns | 991.11 ns | +6.14% | - |
| `resultant_tensor_polynomial_univariate_constraint/padded_closed_form_norm_1024` | 7.51 us | 7.46 us - 7.59 us | 7.43 us | +0.88% | - |
| `resultant_tensor_polynomial_univariate_constraint/padded_direct_norm_1024` | 10.36 us | 10.35 us - 10.37 us | 10.35 us | -0.25% | - |
| `resultant_tensor_polynomial_univariate_constraint/padded_grid_64x64` | 70.70 us | 70.58 us - 70.85 us | 70.59 us | -2.11% | - |
| `resultant_tensor_polynomial_univariate_constraint/strict_exact_normal_constraint` | 10.54 us | 10.52 us - 10.56 us | 10.51 us | -3.22% | - |
| `resultant_tensor_polynomial_univariate_constraint/strict_exact_zero_output_degree` | 4.83 us | 4.81 us - 4.86 us | 4.82 us | +0.96% | - |
| `resultant_trivariate_polynomial_univariate_constraint` | 23.99 us | 23.93 us - 24.06 us | 23.93 us | +0.99% | - |
| `resultant_trivariate_polynomial_univariate_constraint/strict_exact_normal_constraint` | 12.85 us | 12.81 us - 12.91 us | 12.80 us | -2.87% | - |
| `resultant_univariate_polynomials` | 1.23 us | 1.23 us - 1.23 us | 1.22 us | +1.50% | - |
| `resultant_univariate_polynomials/strict_exact_normal_trimming` | 8.88 us | 8.86 us - 8.90 us | 8.88 us | -2.80% | - |
| `run_active_set_update_loop` | 3.41 us | 3.40 us - 3.43 us | 3.40 us | - | - |
| `schedule_candidate_batch_predicates` | 1.80 us | 1.79 us - 1.80 us | 1.79 us | - | - |
| `schedule_univariate_resultant_pairs` | 2.82 us | 2.81 us - 2.82 us | 2.81 us | +2.34% | - |
| `search_failed_constraint_minimal_removals` | 24.42 us | 24.30 us - 24.57 us | 24.30 us | - | - |
| `search_failed_constraint_pair_removals` | 3.24 us | 3.23 us - 3.25 us | 3.22 us | - | - |
| `search_failed_constraint_set_removals` | 24.31 us | 24.21 us - 24.42 us | 24.15 us | - | - |
| `search_failed_constraint_single_removals` | 1.67 us | 1.67 us - 1.67 us | 1.67 us | - | - |
| `simplify_unary_endpoint_expression` | 24.37 us | 24.25 us - 24.50 us | 24.13 us | - | - |
| `sketch_arc_arc_tangent_lowering` | 98.35 us | 98.23 us - 98.49 us | 98.24 us | -1.37% | - |
| `sketch_arc_arc_tangent_residual_forms` | 47.42 us | 47.37 us - 47.48 us | 47.44 us | +1.11% | - |
| `sketch_arc_cubic_second_order_lowering` | 704.29 us | 703.35 us - 705.40 us | 703.43 us | -1.69% | - |
| `sketch_arc_cubic_second_order_residual_forms` | 533.36 us | 532.88 us - 533.86 us | 532.47 us | -0.72% | - |
| `sketch_arc_cubic_tangent_lowering` | 403.68 us | 403.24 us - 404.13 us | 403.58 us | -1.11% | - |
| `sketch_arc_cubic_tangent_residual_forms` | 324.74 us | 323.35 us - 326.23 us | 322.54 us | -0.95% | - |
| `sketch_arc_line_tangent_lowering` | 75.56 us | 75.50 us - 75.63 us | 75.52 us | -1.46% | - |
| `sketch_arc_line_tangent_residual_forms` | 40.60 us | 40.55 us - 40.65 us | 40.56 us | -0.62% | - |
| `sketch_axis_symmetry_lowering` | 33.20 us | 33.13 us - 33.27 us | 33.15 us | -0.41% | - |
| `sketch_circle_circle_tangent_lowering` | 33.48 us | 33.33 us - 33.67 us | 33.23 us | -3.24% | - |
| `sketch_circle_circle_tangent_residual_forms` | 15.87 us | 15.84 us - 15.91 us | 15.80 us | -1.97% | - |
| `sketch_circle_incidence_residual_forms` | 19.71 us | 19.68 us - 19.76 us | 19.66 us | +0.93% | - |
| `sketch_compatibility_fixture_replay` | 98.78 us | 98.61 us - 98.97 us | 98.66 us | -1.44% | - |
| `sketch_concentric_lowering` | 17.58 us | 17.45 us - 17.71 us | 17.19 us | -4.29% | - |
| `sketch_concentric_residual_forms` | 4.71 us | 4.70 us - 4.72 us | 4.69 us | -4.85% | - |
| `sketch_construction_certificate` | 406.74 us | 405.46 us - 408.24 us | 404.15 us | -2.12% | - |
| `sketch_cubic_cubic_c2_lowering` | 687.19 us | 685.58 us - 688.99 us | 684.11 us | +0.08% | - |
| `sketch_cubic_cubic_c2_residual_forms` | 486.05 us | 484.44 us - 487.87 us | 484.04 us | -0.12% | - |
| `sketch_cubic_cubic_g2_lowering` | 2.18 ms | 2.18 ms - 2.18 ms | 2.18 ms | +0.36% | - |
| `sketch_cubic_cubic_g2_residual_forms` | 1.58 ms | 1.57 ms - 1.58 ms | 1.58 ms | +1.21% | - |
| `sketch_cubic_cubic_tangent_lowering` | 704.33 us | 702.84 us - 705.83 us | 702.73 us | -1.41% | - |
| `sketch_cubic_cubic_tangent_residual_forms` | 508.85 us | 507.63 us - 510.34 us | 506.81 us | -1.97% | - |
| `sketch_cubic_line_tangent_lowering` | 374.72 us | 374.01 us - 375.47 us | 374.27 us | -0.89% | - |
| `sketch_cubic_line_tangent_residual_forms` | 261.29 us | 260.58 us - 262.19 us | 260.79 us | -1.87% | - |
| `sketch_degeneracy_preflight` | 16.42 us | 16.40 us - 16.43 us | 16.39 us | +1.27% | - |
| `sketch_distance_range_lowering` | 41.97 us | 41.80 us - 42.16 us | 41.79 us | -0.76% | - |
| `sketch_distance_residual_forms` | 19.41 us | 19.40 us - 19.42 us | 19.41 us | -0.67% | - |
| `sketch_entity_domain_preflight` | 13.72 us | 13.71 us - 13.75 us | 13.70 us | -2.30% | - |
| `sketch_equal_angle_lowering` | 127.33 us | 127.09 us - 127.60 us | 126.78 us | -2.03% | - |
| `sketch_equal_angle_residual_forms` | 122.62 us | 122.42 us - 122.86 us | 122.32 us | -0.11% | - |
| `sketch_equal_length_radius_lowering` | 46.18 us | 46.06 us - 46.31 us | 46.05 us | -4.30% | - |
| `sketch_equal_point_line_distance_lowering` | 170.86 us | 170.16 us - 171.73 us | 170.01 us | +0.36% | - |
| `sketch_length_difference_lowering` | 126.58 us | 126.23 us - 126.99 us | 126.14 us | -0.21% | - |
| `sketch_length_ratio_point_line_lowering` | 95.16 us | 94.76 us - 95.60 us | 94.28 us | -0.84% | - |
| `sketch_line_arc_length_lowering` | 161.23 us | 160.88 us - 161.65 us | 160.96 us | +0.58% | - |
| `sketch_line_arc_length_residual_forms` | 110.01 us | 109.83 us - 110.18 us | 109.95 us | +2.14% | - |
| `sketch_line_arc_sweep_length_lowering` | 177.57 us | 177.19 us - 178.00 us | 176.91 us | +0.73% | - |
| `sketch_line_arc_sweep_length_residual_forms` | 120.33 us | 119.49 us - 121.37 us | 119.11 us | +2.46% | - |
| `sketch_line_orientation_lowering` | 54.92 us | 54.87 us - 54.97 us | 54.87 us | +1.09% | - |
| `sketch_line_symmetry_lowering` | 55.09 us | 54.95 us - 55.25 us | 54.97 us | +0.57% | - |
| `sketch_lower_to_problem` | 49.26 us | 49.13 us - 49.40 us | 48.83 us | -2.57% | - |
| `sketch_midpoint_lowering` | 24.20 us | 24.09 us - 24.35 us | 24.00 us | -1.77% | - |
| `sketch_oriented_angle_lowering` | 136.77 us | 136.32 us - 137.45 us | 136.29 us | -2.39% | - |
| `sketch_oriented_angle_residual_forms` | 78.98 us | 78.90 us - 79.06 us | 79.00 us | -2.09% | - |
| `sketch_parameter_domain_preflight` | 2.69 us | 2.68 us - 2.69 us | 2.69 us | -1.30% | - |
| `sketch_parameter_margin_lowering` | 5.95 us | 5.94 us - 5.96 us | 5.94 us | +1.98% | - |
| `sketch_parameter_ordering_lowering` | 4.66 us | 4.65 us - 4.66 us | 4.65 us | -0.34% | - |
| `sketch_point_line_residual_forms` | 65.05 us | 64.99 us - 65.10 us | 65.05 us | -1.39% | - |
| `sketch_point_on_arc_lowering` | 142.98 us | 141.78 us - 144.39 us | 140.74 us | +3.95% | - |
| `sketch_point_on_arc_residual_forms` | 97.27 us | 97.12 us - 97.44 us | 97.19 us | +0.29% | - |
| `sketch_point_on_cubic_lowering` | 145.70 us | 145.44 us - 145.95 us | 146.07 us | +0.53% | - |
| `sketch_point_on_cubic_residual_forms` | 94.17 us | 93.84 us - 94.57 us | 93.92 us | -0.01% | - |
| `sketch_point_on_line_lowering` | 27.89 us | 27.83 us - 27.95 us | 28.03 us | -2.02% | - |
| `sketch_point_on_line_residual_forms` | 14.32 us | 14.28 us - 14.38 us | 14.26 us | +0.29% | - |
| `sketch_projected_arc_cubic_curve_second_order_lowering` | 6.18 ms | 6.17 ms - 6.20 ms | 6.17 ms | -1.47% | - |
| `sketch_projected_arc_cubic_curve_second_order_residual_forms` | 5.62 ms | 5.60 ms - 5.64 ms | 5.59 ms | -2.49% | - |
| `sketch_projected_arc_cubic_curve_tangent_lowering` | 3.73 ms | 3.72 ms - 3.75 ms | 3.72 ms | -1.86% | - |
| `sketch_projected_arc_cubic_curve_tangent_residual_forms` | 3.38 ms | 3.37 ms - 3.39 ms | 3.38 ms | -2.27% | - |
| `sketch_projected_arc_line_tangent_lowering` | 534.30 us | 533.28 us - 535.40 us | 533.26 us | -3.80% | - |
| `sketch_projected_arc_line_tangent_residual_forms` | 443.21 us | 442.05 us - 444.51 us | 441.91 us | -0.53% | - |
| `sketch_projected_cubic_curve_cubic_curve_c2_lowering` | 7.36 ms | 7.34 ms - 7.38 ms | 7.34 ms | -1.91% | - |
| `sketch_projected_cubic_curve_cubic_curve_c2_residual_forms` | 7.10 ms | 7.06 ms - 7.15 ms | 7.02 ms | -1.20% | - |
| `sketch_projected_cubic_curve_cubic_curve_g2_lowering` | 22.86 ms | 22.66 ms - 23.08 ms | 22.62 ms | -1.61% | - |
| `sketch_projected_cubic_curve_cubic_curve_g2_residual_forms` | 24.43 ms | 24.35 ms - 24.49 ms | 24.41 ms | -1.85% | - |
| `sketch_projected_cubic_curve_cubic_curve_tangent_lowering` | 7.51 ms | 7.50 ms - 7.52 ms | 7.52 ms | +0.20% | - |
| `sketch_projected_cubic_curve_cubic_curve_tangent_residual_forms` | 7.11 ms | 7.08 ms - 7.13 ms | 7.07 ms | -3.03% | - |
| `sketch_projected_cubic_curve_line_tangent_lowering` | 4.20 ms | 4.20 ms - 4.20 ms | 4.20 ms | -2.11% | - |
| `sketch_projected_cubic_curve_line_tangent_residual_forms` | 3.49 ms | 3.47 ms - 3.50 ms | 3.47 ms | -2.39% | - |
| `sketch_projected_cubic_line_tangent_lowering` | 877.47 us | 875.99 us - 879.08 us | 876.69 us | -2.20% | - |
| `sketch_projected_cubic_line_tangent_residual_forms` | 620.54 us | 619.47 us - 621.66 us | 619.74 us | -2.37% | - |
| `sketch_projected_distance_lowering` | 243.91 us | 243.36 us - 244.43 us | 244.27 us | +0.13% | - |
| `sketch_projected_distance_range_lowering` | 398.03 us | 396.77 us - 399.52 us | 396.25 us | -0.97% | - |
| `sketch_projected_distance_range_residual_forms` | 263.44 us | 262.20 us - 264.65 us | 265.93 us | -0.52% | - |
| `sketch_projected_distance_residual_forms` | 260.68 us | 259.70 us - 261.64 us | 261.42 us | +3.23% | - |
| `sketch_projected_equal_length_lowering` | 436.36 us | 434.72 us - 438.14 us | 433.65 us | +0.32% | - |
| `sketch_projected_equal_length_residual_forms` | 303.11 us | 301.87 us - 304.34 us | 305.97 us | -0.60% | - |
| `sketch_projected_equal_point_distances_lowering` | 462.58 us | 461.02 us - 464.30 us | 463.45 us | +1.71% | - |
| `sketch_projected_equal_point_distances_residual_forms` | 307.21 us | 306.04 us - 308.53 us | 307.22 us | -0.34% | - |
| `sketch_projected_equal_point_line_distances_lowering` | 1.08 ms | 1.08 ms - 1.08 ms | 1.08 ms | -2.10% | - |
| `sketch_projected_equal_point_line_distances_residual_forms` | 749.31 us | 745.27 us - 753.12 us | 760.97 us | +1.41% | - |
| `sketch_projected_length_difference_lowering` | 1.39 ms | 1.38 ms - 1.39 ms | 1.38 ms | -2.20% | - |
| `sketch_projected_length_difference_residual_forms` | 944.03 us | 939.51 us - 948.63 us | 938.05 us | -3.43% | - |
| `sketch_projected_length_point_line_distance_lowering` | 790.50 us | 787.75 us - 793.40 us | 787.15 us | -2.04% | - |
| `sketch_projected_length_point_line_distance_residual_forms` | 538.62 us | 536.27 us - 541.13 us | 537.31 us | +0.29% | - |
| `sketch_projected_length_ratio_lowering` | 439.36 us | 438.02 us - 440.79 us | 437.78 us | -1.12% | - |
| `sketch_projected_length_ratio_residual_forms` | 303.00 us | 302.25 us - 303.79 us | 301.34 us | +2.26% | - |
| `sketch_projected_line_arc_sweep_length_lowering` | 457.60 us | 456.89 us - 458.36 us | 457.06 us | -1.79% | - |
| `sketch_projected_line_arc_sweep_length_residual_forms` | 324.13 us | 323.24 us - 325.25 us | 322.71 us | +0.87% | - |
| `sketch_projected_line_circle_tangent_lowering` | 833.92 us | 831.59 us - 836.32 us | 832.64 us | -0.80% | - |
| `sketch_projected_line_circle_tangent_residual_forms` | 583.52 us | 581.75 us - 585.41 us | 581.17 us | -0.89% | - |
| `sketch_projected_line_length_range_lowering` | 404.59 us | 403.28 us - 406.16 us | 403.96 us | -1.43% | - |
| `sketch_projected_line_length_range_residual_forms` | 266.65 us | 265.62 us - 267.71 us | 267.77 us | -2.71% | - |
| `sketch_projected_line_orientation_lowering` | 1.58 ms | 1.57 ms - 1.59 ms | 1.57 ms | -1.69% | - |
| `sketch_projected_line_orientation_residual_forms` | 1.24 ms | 1.24 ms - 1.24 ms | 1.23 ms | -0.24% | - |
| `sketch_projected_line_radius_lowering` | 239.32 us | 238.57 us - 240.19 us | 238.24 us | +0.60% | - |
| `sketch_projected_line_radius_residual_forms` | 158.59 us | 158.28 us - 158.96 us | 158.23 us | +1.69% | - |
| `sketch_projected_line_symmetry_lowering` | 731.86 us | 730.02 us - 733.80 us | 728.87 us | -2.21% | - |
| `sketch_projected_line_symmetry_residual_forms` | 543.16 us | 540.67 us - 546.57 us | 542.00 us | +0.13% | - |
| `sketch_projected_oriented_angle_lowering` | 1.78 ms | 1.77 ms - 1.80 ms | 1.76 ms | +0.98% | - |
| `sketch_projected_oriented_angle_residual_forms` | 1.26 ms | 1.26 ms - 1.27 ms | 1.26 ms | +0.57% | - |
| `sketch_projected_point_concentric_lowering` | 229.49 us | 228.95 us - 230.06 us | 228.66 us | -1.16% | - |
| `sketch_projected_point_concentric_residual_forms` | 159.01 us | 158.54 us - 159.58 us | 158.47 us | +1.33% | - |
| `sketch_projected_point_distance_difference_lowering` | 1.44 ms | 1.44 ms - 1.44 ms | 1.44 ms | -0.89% | - |
| `sketch_projected_point_distance_difference_residual_forms` | 962.02 us | 957.96 us - 966.04 us | 971.20 us | +0.66% | - |
| `sketch_projected_point_distance_point_line_distance_lowering` | 780.47 us | 777.62 us - 783.52 us | 778.19 us | -1.25% | - |
| `sketch_projected_point_distance_point_line_distance_residual_forms` | 539.13 us | 536.97 us - 541.81 us | 536.25 us | +3.58% | - |
| `sketch_projected_point_distance_ratio_lowering` | 468.67 us | 467.50 us - 469.87 us | 467.96 us | -1.20% | - |
| `sketch_projected_point_distance_ratio_residual_forms` | 308.56 us | 307.14 us - 310.08 us | 307.43 us | +0.98% | - |
| `sketch_projected_point_line_distance_lowering` | 661.62 us | 659.88 us - 663.42 us | 656.76 us | -3.09% | - |
| `sketch_projected_point_line_distance_range_lowering` | 1.08 ms | 1.08 ms - 1.08 ms | 1.08 ms | +0.40% | - |
| `sketch_projected_point_line_distance_range_residual_forms` | 710.52 us | 707.24 us - 713.75 us | 714.45 us | +1.34% | - |
| `sketch_projected_point_line_distance_residual_forms` | 1.11 ms | 1.10 ms - 1.11 ms | 1.11 ms | +0.23% | - |
| `sketch_projected_point_line_radius_lowering` | 580.44 us | 577.69 us - 583.67 us | 577.40 us | -1.27% | - |
| `sketch_projected_point_line_radius_residual_forms` | 383.88 us | 382.38 us - 385.34 us | 385.99 us | +1.18% | - |
| `sketch_projected_point_on_arc_lowering` | 833.95 us | 832.31 us - 835.66 us | 832.01 us | +1.27% | - |
| `sketch_projected_point_on_arc_residual_forms` | 592.17 us | 591.17 us - 593.28 us | 592.10 us | +3.80% | - |
| `sketch_projected_point_on_circle_lowering` | 368.02 us | 367.13 us - 368.97 us | 366.73 us | -2.23% | - |
| `sketch_projected_point_on_circle_residual_forms` | 267.03 us | 266.38 us - 267.78 us | 266.84 us | -0.23% | - |
| `sketch_projected_point_on_cubic_curve_lowering` | 1.32 ms | 1.31 ms - 1.32 ms | 1.31 ms | -2.41% | - |
| `sketch_projected_point_on_cubic_curve_residual_forms` | 1.02 ms | 1.02 ms - 1.03 ms | 1.02 ms | -0.74% | - |
| `sketch_projected_point_on_cubic_lowering` | 354.13 us | 353.42 us - 354.95 us | 353.10 us | -0.30% | - |
| `sketch_projected_point_on_cubic_residual_forms` | 248.71 us | 248.18 us - 249.32 us | 247.63 us | +0.25% | - |
| `sketch_projected_point_on_line_lowering` | 397.58 us | 396.95 us - 398.22 us | 397.10 us | -2.05% | - |
| `sketch_projected_point_on_line_residual_forms` | 302.82 us | 302.35 us - 303.33 us | 302.22 us | -2.12% | - |
| `sketch_projected_point_radius_lowering` | 253.06 us | 251.90 us - 254.35 us | 252.10 us | -0.52% | - |
| `sketch_projected_point_radius_residual_forms` | 159.59 us | 159.21 us - 160.02 us | 158.90 us | +2.88% | - |
| `sketch_range_and_objective_lowering` | 10.86 us | 10.83 us - 10.90 us | 10.82 us | +1.06% | - |
| `sketch_round_trip_metadata_lowering` | 50.17 us | 50.00 us - 50.36 us | 49.91 us | -1.26% | - |
| `sketch_same_direction_lowering` | 42.31 us | 42.26 us - 42.36 us | 42.22 us | -3.49% | - |
| `sketch_tangent_residual_forms` | 22.77 us | 22.74 us - 22.81 us | 22.75 us | -1.43% | - |
| `sketch_tangent_same_direction_lowering` | 41.72 us | 41.69 us - 41.76 us | 41.67 us | -2.32% | - |
| `sketch_unit_tolerance_audit` | 12.31 us | 12.30 us - 12.33 us | 12.30 us | -0.17% | - |
| `sketch_workplane_frame` | 931.34 ns | 930.57 ns - 932.28 ns | 930.74 ns | -1.69% | - |
| `sketch_workplane_point_lifts` | 19.21 us | 19.08 us - 19.40 us | 19.07 us | +0.46% | - |
| `sketch_workplane_symmetry_lowering` | 221.63 us | 221.21 us - 222.07 us | 221.43 us | -3.01% | - |
| `sketch_workplane_symmetry_residual_forms` | 148.22 us | 147.64 us - 148.87 us | 147.63 us | +1.25% | - |
| `solve_bfgs_affine` | 4.65 us | 4.63 us - 4.66 us | 4.62 us | - | - |
| `solve_dense_bareiss_approximate_terminal_replay` | 1.49 us | 1.48 us - 1.49 us | 1.48 us | -0.24% | - |
| `solve_dense_bareiss_two_rhs_sequential` | 2.85 us | 2.85 us - 2.86 us | 2.85 us | -3.76% | - |
| `solve_dense_bareiss_two_rhs_shared` | 2.47 us | 2.46 us - 2.47 us | 2.46 us | -2.02% | - |
| `solve_dense_linear_system_bareiss` | 1.52 us | 1.52 us - 1.52 us | 1.52 us | -1.20% | - |
| `solve_dense_linear_system_bareiss_tridiagonal_8` | 20.49 us | 20.41 us - 20.58 us | 20.35 us | +2.67% | - |
| `solve_direct_affine_system` | 883.93 ns | 883.10 ns - 884.79 ns | 883.51 ns | -6.40% | - |
| `solve_direct_univariate_quadratic_rows` | 5.23 us | 5.23 us - 5.24 us | 5.24 us | +2.12% | - |
| `solve_dogleg_affine` | 4.64 us | 4.64 us - 4.64 us | 4.64 us | - | - |
| `solve_levenberg_marquardt_affine` | 4.61 us | 4.60 us - 4.62 us | 4.59 us | - | - |
| `solve_modified_newton_affine_seed` | 3.88 us | 3.88 us - 3.89 us | 3.88 us | - | - |
| `solve_modified_newton_bounded_quadratic_seed` | 13.46 us | 13.44 us - 13.48 us | 13.43 us | - | - |
| `solve_modified_newton_bounded_substitution_seed` | 5.74 us | 5.71 us - 5.77 us | 5.69 us | - | - |
| `solve_modified_newton_dragged_parameter` | 1.10 us | 1.10 us - 1.10 us | 1.10 us | - | - |
| `solve_modified_newton_least_squares_affine` | 15.74 us | 15.72 us - 15.76 us | 15.71 us | - | - |
| `solve_modified_newton_preprocessing` | 13.46 us | 13.42 us - 13.50 us | 13.41 us | - | - |
| `solve_modified_newton_quadratic_seed` | 12.79 us | 12.78 us - 12.80 us | 12.78 us | - | - |
| `solve_modified_newton_substitution_seed` | 8.75 us | 8.73 us - 8.77 us | 8.72 us | - | - |
| `solve_powell_hybrid_affine` | 4.60 us | 4.59 us - 4.62 us | 4.59 us | - | - |
| `solve_sparse_linear_system_bareiss` | 2.45 us | 2.44 us - 2.45 us | 2.44 us | -0.96% | - |
| `solve_sparse_linear_system_bareiss_pattern_preserving` | 3.62 us | 3.61 us - 3.63 us | 3.60 us | -1.04% | - |
| `solve_sparse_linear_system_bareiss_pattern_preserving/strict_exact_normal_pivot` | 5.94 us | 5.93 us - 5.96 us | 5.93 us | -0.35% | - |
| `solve_sqp_affine` | 4.60 us | 4.60 us - 4.61 us | 4.60 us | - | - |
| `solver_block_affine_rows` | 240.46 ns | 239.81 ns - 241.30 ns | 239.29 ns | -0.87% | - |
| `sparse_bareiss_arrowhead_32/authored_order` | 3.43 ms | 3.42 ms - 3.44 ms | 3.42 ms | +2.73% | - |
| `sparse_bareiss_arrowhead_32/minimum_degree` | 775.48 us | 773.32 us - 778.22 us | 772.24 us | +0.56% | - |
| `sparse_bareiss_tridiagonal_32/authored_order` | 234.99 us | 234.68 us - 235.32 us | 234.62 us | -3.56% | - |
| `sparse_bareiss_tridiagonal_32/minimum_degree` | 298.94 us | 298.55 us - 299.35 us | 298.44 us | -1.32% | - |
| `sparse_linear_batch_replay` | 6.16 us | 6.13 us - 6.20 us | 6.13 us | -0.40% | - |
| `square_free_part/degree_64_repeated` | 27.80 us | 27.76 us - 27.84 us | 27.76 us | -0.43% | - |
| `square_free_part/degree_64_square_free` | 7.08 us | 7.05 us - 7.11 us | 7.04 us | +1.55% | - |
| `square_free_part/exact_real_repeated_quadratic` | 1.22 us | 1.18 us - 1.30 us | 1.18 us | +2.28% | - |
| `square_root_algebraic_root` | 1.45 us | 1.44 us - 1.46 us | 1.43 us | - | - |
| `square_root_algebraic_root/adaptive_neighbor` | 3.28 us | 3.28 us - 3.29 us | 3.28 us | - | - |
| `square_root_algebraic_root/repeated_exact_witness` | 2.54 us | 2.54 us - 2.55 us | 2.54 us | - | - |
| `square_root_algebraic_root/repeated_rational_square_witness` | 645.79 ns | 645.03 ns - 646.62 ns | 644.78 ns | - | - |
| `subdivide_bernstein_univariate_polynomial_interval_roots` | 64.43 us | 64.36 us - 64.52 us | 64.38 us | - | - |
| `subresultant_chain_univariate_polynomials` | 920.01 ns | 918.83 ns - 921.21 ns | 919.73 ns | -0.85% | - |
| `substitute_bezier_power_basis` | 568.09 ns | 567.18 ns - 569.20 ns | 567.33 ns | +3.39% | - |
| `substitute_bspline_knot_span_power_basis` | 3.63 us | 3.62 us - 3.64 us | 3.62 us | -1.08% | - |
| `substitute_nurbs_knot_span_power_basis` | 9.71 us | 9.70 us - 9.73 us | 9.74 us | -0.10% | - |
| `substitute_rational_bezier_power_basis` | 1.43 us | 1.43 us - 1.43 us | 1.43 us | +0.82% | - |
| `transform_algebraic_root_affine` | 360.83 ns | 359.62 ns - 362.20 ns | 358.56 ns | - | - |
| `transform_algebraic_root_affine/degree_16` | 17.26 us | 17.21 us - 17.33 us | 17.18 us | - | - |
| `transform_algebraic_root_affine/degree_16_dense` | 17.96 us | 17.93 us - 17.99 us | 17.91 us | - | - |
| `transform_algebraic_root_affine/exact_real_source` | 758.01 ns | 756.27 ns - 760.03 ns | 755.14 ns | - | - |
| `transform_algebraic_root_affine/exact_witness` | 221.12 ns | 220.64 ns - 221.71 ns | 220.59 ns | - | - |
| `transform_algebraic_root_affine/negative_scaling` | 348.97 ns | 348.32 ns - 349.66 ns | 348.77 ns | - | - |
| `transform_algebraic_root_affine/negative_scaling_endpoint_refinement` | 9.11 us | 9.09 us - 9.13 us | 9.08 us | - | - |
| `transform_algebraic_root_affine/scaling` | 221.25 ns | 221.09 ns - 221.43 ns | 221.14 ns | - | - |
| `transform_algebraic_root_affine/translation` | 363.37 ns | 362.36 ns - 364.61 ns | 361.66 ns | - | - |
| `transform_algebraic_root_mobius` | 1.19 us | 1.19 us - 1.19 us | 1.19 us | - | - |
| `transform_algebraic_root_mobius/constant_denominator` | 474.98 ns | 473.57 ns - 476.56 ns | 474.45 ns | - | - |
| `transform_algebraic_root_mobius/exact_real_source` | 1.44 us | 1.43 us - 1.45 us | 1.43 us | - | - |
| `transform_algebraic_root_mobius/exact_witness` | 503.10 ns | 501.33 ns - 505.38 ns | 501.38 ns | - | - |
| `transform_algebraic_root_mobius/excluded_endpoint_pole_refinement` | 5.27 us | 5.26 us - 5.28 us | 5.25 us | - | - |
| `transform_algebraic_root_mobius/reciprocal` | 536.85 ns | 535.52 ns - 538.38 ns | 536.08 ns | - | - |
| `transform_algebraic_root_mobius/reversed_endpoint_refinement` | 9.53 us | 9.50 us - 9.57 us | 9.49 us | - | - |
| `transform_algebraic_root_polynomial_image` | 3.46 us | 3.45 us - 3.47 us | 3.45 us | - | - |
| `transform_algebraic_root_polynomial_image/constant` | 121.64 ns | 121.20 ns - 122.15 ns | 120.84 ns | - | - |
| `transform_algebraic_root_polynomial_image/exact_witness` | 460.82 ns | 459.28 ns - 462.69 ns | 459.19 ns | - | - |
| `transform_algebraic_root_polynomial_image/foreign_root_refinement` | 28.02 us | 27.97 us - 28.07 us | 27.96 us | - | - |
| `transform_algebraic_root_polynomial_image/rational_modulus_image` | 178.05 ns | 177.81 ns - 178.33 ns | 177.85 ns | - | - |
| `transform_algebraic_root_polynomial_image/repeated_degree_eight_carrier` | 19.38 us | 19.12 us - 19.76 us | 19.11 us | - | - |
| `transform_algebraic_root_polynomial_image/repeated_degree_six_carrier` | 50.39 us | 50.33 us - 50.44 us | 50.39 us | - | - |
| `transform_algebraic_root_polynomial_image/square_free_degree_six_carrier` | 31.12 us | 30.89 us - 31.39 us | 30.66 us | - | - |
| `transform_algebraic_root_polynomial_image/stationary` | 4.08 us | 4.07 us - 4.08 us | 4.07 us | - | - |
| `transform_algebraic_root_rational_image` | 2.64 us | 2.63 us - 2.64 us | 2.63 us | - | - |
| `transform_algebraic_root_rational_image/certified_algebraic_pole` | 899.07 ns | 897.23 ns - 900.87 ns | 902.07 ns | - | - |
| `transform_algebraic_root_rational_image/constant_degree_6` | 3.11 us | 3.11 us - 3.11 us | 3.11 us | - | - |
| `transform_algebraic_root_rational_image/dependency_broadened_denominator` | 9.88 us | 9.87 us - 9.89 us | 9.88 us | - | - |
| `transform_algebraic_root_rational_image/monotone_quadratic` | 1.81 us | 1.80 us - 1.82 us | 1.79 us | - | - |
| `transform_algebraic_root_rational_image/shared_linear_factor` | 13.75 us | 13.74 us - 13.76 us | 13.74 us | - | - |
| `transform_algebraic_root_rational_image/shared_nonmonic_linear_factor` | 14.64 us | 14.63 us - 14.65 us | 14.65 us | - | - |
| `transform_algebraic_root_rational_image/stationary_cubic` | 3.25 us | 3.23 us - 3.28 us | 3.21 us | - | - |
| `transform_algebraic_root_rational_image_degree_12_cubic` | 250.91 us | 250.35 us - 251.69 us | 250.49 us | - | - |
| `transform_algebraic_root_rational_images/batch_four_dependency_denominator` | 34.86 us | 34.49 us - 35.35 us | 34.43 us | - | - |
| `transform_algebraic_root_rational_images/batch_four_linear` | 10.49 us | 10.47 us - 10.50 us | 10.48 us | - | - |
| `transform_algebraic_roots_binary` | 18.20 us | 18.15 us - 18.25 us | 18.13 us | - | - |
| `transform_algebraic_roots_binary/repeated_degree_six_carriers` | 45.62 us | 45.57 us - 45.68 us | 45.55 us | - | - |
| `transform_algebraic_roots_binary/shared_repeated_degree_six_carrier` | 31.76 us | 31.72 us - 31.81 us | 31.75 us | - | - |
| `transform_algebraic_roots_binary_divide` | 17.23 us | 17.20 us - 17.28 us | 17.20 us | - | - |
| `transform_algebraic_roots_binary_multiply` | 16.32 us | 16.26 us - 16.38 us | 16.23 us | - | - |
| `transform_algebraic_roots_binary_subtract` | 19.67 us | 19.65 us - 19.70 us | 19.64 us | - | - |
| `univariate_quadratic_row_forms` | 41.49 us | 41.40 us - 41.57 us | 41.36 us | -1.29% | - |

<!-- END COMPLETE BENCHMARK REPORT -->
