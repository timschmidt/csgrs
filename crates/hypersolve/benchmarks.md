<!-- BEGIN promoted_slow_offender_score -->
## `promoted_slow_offender_score`

Deterministic lexicase score for Hypersolve's retained fuzz offenders. The score is the average current best-of-five replay time; lower is better. Delta compares with the previous score, and derivative is the change in delta.

<!-- promoted_slow_score_nanos: 29584 -->
<!-- promoted_slow_previous_score_nanos: 28433 -->
<!-- promoted_slow_score_delta_nanos: 1151 -->

| Metric | Value |
| --- | ---: |
| Cases scored | 100 |
| Average score | 29.584 us |
| Delta | 1.151 us |
| Delta derivative | 1.151 us |

| Rank | Current Time | Fuzz target | Input |
| ---: | ---: | --- | --- |
| 1 | 31.968 us | `sketch_projected_line_radius_equality` | `seed[92]` |
| 2 | 31.068 us | `sketch_projected_oriented_angle` | `seed[20]` |
| 3 | 30.968 us | `sketch_projected_arc_cubic_curve_tangent` | `seed[9]` |
| 4 | 30.948 us | `sketch_projected_cubic_line_tangent` | `seed[6]` |
| 5 | 30.908 us | `sketch_projected_point_on_cubic` | `seed[18]` |
| 6 | 30.818 us | `sketch_projected_arc_cubic_curve_tangent` | `seed[23]` |
| 7 | 30.778 us | `sketch_projected_line_length_range` | `seed[8]` |
| 8 | 30.628 us | `sketch_projected_arc_cubic_curve_second_order_contact` | `seed[12]` |
| 9 | 30.628 us | `sketch_projected_line_arc_sweep_length` | `seed[19]` |
| 10 | 30.618 us | `sketch_projected_length_point_line_distance` | `seed[6]` |

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
| `competitive/dense_linear_4x4` | `-` | `nalgebra_f64_lu_proposal` | 68.04 ns | 1.00x |
| `competitive/dense_linear_4x4` | `-` | `hypersolve_exact_with_replay` | 5.45 us | 80.07x |
| `competitive/quadratic_roots` | `-` | `roots_f64_proposal` | 3.87 ns | 1.00x |
| `competitive/quadratic_roots` | `-` | `hypersolve_exact_candidates` | 292.96 ns | 75.76x |

### All Criterion results

| Benchmark | Mean | 95% CI | Median | Change vs baseline | Throughput |
| --- | ---: | ---: | ---: | ---: | ---: |
| `algebraic_root_rational_map_transform` | 2.25 us | 2.24 us - 2.26 us | 2.23 us | - | - |
| `analyze_sparse_bareiss_elimination_pattern` | 946.32 ns | 942.43 ns - 950.30 ns | 942.74 ns | +1.57% | - |
| `arithmetic_algebraic_root_representations` | 119.68 ns | 119.46 ns - 119.98 ns | 119.40 ns | -6.82% | - |
| `arithmetic_algebraic_root_representations/exact_normal_point_divide` | 10.76 us | 10.73 us - 10.81 us | 10.71 us | -5.48% | - |
| `arithmetic_algebraic_root_representations/exact_real_points` | 393.80 ns | 391.51 ns - 396.41 ns | 389.16 ns | -19.94% | - |
| `arithmetic_algebraic_root_representations/mixed_exact_real_scalar` | 3.78 us | 3.77 us - 3.79 us | 3.76 us | -8.96% | - |
| `arithmetic_algebraic_root_representations/negate` | 122.30 ns | 122.06 ns - 122.57 ns | 121.92 ns | -19.99% | - |
| `arithmetic_algebraic_root_representations/same_quadratic_affine_square` | 443.39 ns | 441.62 ns - 445.34 ns | 440.94 ns | -4.96% | - |
| `arithmetic_algebraic_root_representations/same_root_add` | 381.34 ns | 380.10 ns - 382.76 ns | 379.29 ns | -10.94% | - |
| `arithmetic_algebraic_root_representations/same_root_multiply` | 103.45 ns | 103.29 ns - 103.62 ns | 103.23 ns | -0.99% | - |
| `arithmetic_algebraic_root_representations/same_root_touching_zero_divide` | 101.39 ns | 101.20 ns - 101.59 ns | 101.11 ns | -3.05% | - |
| `arithmetic_algebraic_root_representations/zero_dividend` | 91.04 ns | 90.53 ns - 91.64 ns | 89.93 ns | -1.28% | - |
| `arithmetic_algebraic_root_representations_mixed_scalar` | 517.76 ns | 515.80 ns - 519.93 ns | 515.19 ns | +4.80% | - |
| `certify_affine_krawczyk_box` | 1.49 us | 1.49 us - 1.49 us | 1.49 us | -3.31% | - |
| `certify_multivariate_quadratic_krawczyk_rows` | 192.15 ns | 191.87 ns - 192.44 ns | 192.15 ns | -1.97% | - |
| `certify_univariate_quadratic_krawczyk_rows` | 9.97 us | 9.95 us - 9.98 us | 9.96 us | +0.64% | - |
| `compare_algebraic_root_representations` | 42.01 ns | 41.83 ns - 42.21 ns | 41.93 ns | +3.11% | - |
| `compare_algebraic_root_representations/disjoint_intervals` | 25.26 ns | 25.08 ns - 25.49 ns | 25.00 ns | -15.03% | - |
| `compare_algebraic_root_representations/exact_rational_points` | 51.19 ns | 50.96 ns - 51.45 ns | 51.04 ns | -9.55% | - |
| `compare_algebraic_root_representations/exact_real_points` | 945.03 ns | 941.14 ns - 949.55 ns | 939.34 ns | -13.26% | - |
| `compare_algebraic_root_representations/wide_exact_point` | 57.25 ns | 57.11 ns - 57.40 ns | 57.16 ns | - | - |
| `compare_algebraic_root_representations_by_difference` | 21.68 us | 21.58 us - 21.78 us | 21.53 us | +0.73% | - |
| `compare_algebraic_root_representations_by_difference/disjoint_intervals` | 296.98 ns | 295.73 ns - 298.59 ns | 294.86 ns | - | - |
| `compare_algebraic_root_representations_by_difference/exact_point_common_root` | 473.43 ns | 470.91 ns - 476.44 ns | 468.68 ns | - | - |
| `compare_algebraic_root_representations_by_difference/exact_rational_points` | 317.83 ns | 317.48 ns - 318.24 ns | 317.25 ns | - | - |
| `compare_algebraic_root_representations_by_difference/same_representation` | 313.51 ns | 312.65 ns - 314.49 ns | 311.96 ns | - | - |
| `compare_algebraic_root_representations_with_refinement` | 3.34 us | 3.32 us - 3.35 us | 3.32 us | -87.91% | - |
| `compare_algebraic_root_representations_with_refinement/exact_point_against_interval` | 3.89 us | 3.88 us - 3.91 us | 3.87 us | -54.11% | - |
| `compare_algebraic_root_representations_with_refinement/one_sided_exact_real_coefficients` | 4.36 us | 4.35 us - 4.38 us | 4.35 us | -65.33% | - |
| `competitive/dense_linear_4x4/hypersolve_exact_with_replay` | 5.45 us | 5.41 us - 5.49 us | 5.43 us | +1.46% | - |
| `competitive/dense_linear_4x4/nalgebra_f64_lu_proposal` | 68.04 ns | 67.84 ns - 68.29 ns | 67.95 ns | +1.56% | - |
| `competitive/quadratic_roots/hypersolve_exact_candidates` | 292.96 ns | 289.64 ns - 297.00 ns | 289.26 ns | -4.34% | - |
| `competitive/quadratic_roots/roots_f64_proposal` | 3.87 ns | 3.80 ns - 3.95 ns | 3.85 ns | +5.25% | - |
| `count_bivariate_common_fiber_degree_drop` | 11.79 us | 11.72 us - 11.87 us | 11.63 us | -2.17% | - |
| `count_bivariate_fiber_roots_closed/repeated_endpoints` | 10.06 us | 9.87 us - 10.27 us | 9.62 us | -1.12% | - |
| `count_bivariate_fiber_roots_even_multiplicity` | 18.04 us | 17.99 us - 18.09 us | 17.98 us | -1.38% | - |
| `count_bivariate_fiber_roots_intervals/adjacent_64` | 439.12 us | 438.17 us - 440.20 us | 438.30 us | +0.26% | - |
| `count_bivariate_fiber_roots_intervals/lower_endpoint_64` | 4.93 us | 4.89 us - 4.98 us | 4.86 us | -97.80% | - |
| `count_bivariate_fiber_roots_intervals/shared_lower_64` | 419.67 us | 417.99 us - 421.55 us | 416.44 us | -0.90% | - |
| `deflate_bivariate_fiber_diagonal_root/triple_root` | 6.54 us | 6.51 us - 6.56 us | 6.49 us | +0.50% | - |
| `deflate_bivariate_fiber_diagonal_root/triple_root_second_parameter` | 6.42 us | 6.40 us - 6.45 us | 6.40 us | +1.90% | - |
| `dense_tensor_reduce_axis_modulo/already_reduced_2x64x64` | 100.09 us | 99.79 us - 100.44 us | 99.49 us | -70.80% | - |
| `dense_tensor_reduce_axis_modulo/compact_8x2x2` | 4.46 us | 4.44 us - 4.48 us | 4.44 us | -19.67% | - |
| `dense_tensor_reduce_axis_modulo/padded_8x64x64` | 129.58 us | 129.35 us - 129.82 us | 129.08 us | -90.62% | - |
| `divide_bivariate_polynomial_exact` | 861.50 ns | 847.03 ns - 877.92 ns | 831.72 ns | -10.06% | - |
| `divide_univariate_polynomial_exact/dense_degree_48_by_16` | 32.28 us | 32.14 us - 32.43 us | 32.02 us | -4.77% | - |
| `divide_univariate_polynomial_exact/exact_real_quadratic_by_linear` | 14.18 us | 14.07 us - 14.30 us | 13.94 us | -26.82% | - |
| `evaluate_polynomial_at_algebraic_root` | 365.47 ns | 364.72 ns - 366.33 ns | 364.08 ns | -61.47% | - |
| `evaluate_polynomial_at_algebraic_root/degree_16_interval` | 2.88 us | 2.88 us - 2.89 us | 2.87 us | -0.84% | - |
| `evaluate_polynomial_at_algebraic_root/exact_normal_constant` | 6.87 us | 6.85 us - 6.88 us | 6.85 us | - | - |
| `evaluate_polynomial_at_algebraic_root/exact_rational_point` | 261.13 ns | 260.68 ns - 261.62 ns | 260.35 ns | -32.97% | - |
| `evaluate_polynomial_at_algebraic_root/exact_real_point` | 469.20 ns | 468.28 ns - 470.25 ns | 468.01 ns | -11.81% | - |
| `evaluate_rational_expression_at_algebraic_root` | 710.44 ns | 707.19 ns - 715.58 ns | 706.49 ns | -61.92% | - |
| `evaluate_rational_expression_at_algebraic_root/exact_normal_denominator` | 9.06 us | 9.02 us - 9.09 us | 8.99 us | +1405.87% | - |
| `evaluate_rational_expression_at_algebraic_root/exact_rational_point` | 458.92 ns | 457.04 ns - 461.02 ns | 454.93 ns | -36.22% | - |
| `evaluate_rational_expression_at_algebraic_root/exact_real_point` | 709.19 ns | 706.44 ns - 712.42 ns | 704.41 ns | -27.86% | - |
| `evaluate_rational_expression_at_algebraic_root/same_exact_real_value` | 708.23 ns | 705.74 ns - 711.32 ns | 702.56 ns | - | - |
| `evaluate_rational_expression_at_algebraic_root/zero_over_exact_normal_denominator` | 7.13 us | 7.10 us - 7.18 us | 7.06 us | - | - |
| `isolate_bivariate_fiber_roots/bernstein_rational_deflation` | 30.82 us | 30.79 us - 30.86 us | 30.78 us | -1.83% | - |
| `isolate_bivariate_fiber_roots/partitioned_rational_sturm_fallback` | 563.70 us | 561.29 us - 566.42 us | 560.03 us | -0.94% | - |
| `isolate_bivariate_fiber_roots/repeated_irrational_sturm_fallback` | 57.04 us | 56.84 us - 57.25 us | 56.61 us | -3.11% | - |
| `isolate_univariate_polynomial_roots_sturm` | 116.79 us | 116.57 us - 117.02 us | 116.73 us | -7.11% | - |
| `polynomial_has_one_distinct_root_in_open_interval/bernstein_degree_16` | 18.44 us | 18.38 us - 18.51 us | 18.34 us | -13.12% | - |
| `polynomial_has_one_distinct_root_in_open_interval/endpoint_and_interior` | 227.45 ns | 226.87 ns - 228.10 ns | 226.22 ns | -72.90% | - |
| `polynomial_has_one_distinct_root_in_open_interval/monotone_degree_16` | 900.67 ns | 896.60 ns - 904.90 ns | 891.78 ns | -72.77% | - |
| `polynomial_has_one_distinct_root_in_open_interval/quadratic_sign` | 101.21 ns | 100.98 ns - 101.45 ns | 100.89 ns | -62.86% | - |
| `polynomial_has_one_distinct_root_in_open_interval/repeated_exact_real` | 5.34 us | 5.32 us - 5.36 us | 5.30 us | -58.46% | - |
| `polynomial_has_one_distinct_root_in_open_interval/repeated_quadratic` | 497.85 ns | 497.05 ns - 498.70 ns | 497.14 ns | -91.53% | - |
| `polynomial_has_one_distinct_root_in_open_interval/sturm_fallback_cubic` | 8.33 us | 8.31 us - 8.35 us | 8.29 us | -26.07% | - |
| `project_algebraic_fiber_polynomial_image` | 5.65 us | 5.61 us - 5.69 us | 5.58 us | +3.91% | - |
| `project_algebraic_fiber_polynomial_image/cubic_source_quadratic_image` | 21.98 us | 21.85 us - 22.13 us | 21.70 us | -0.78% | - |
| `project_algebraic_fiber_polynomial_image/saturated_conjugate_factor` | 94.84 us | 94.36 us - 95.40 us | 94.12 us | -8.39% | - |
| `project_algebraic_fiber_polynomial_image/saturated_fourfold_multicoefficient` | 2.24 ms | 2.22 ms - 2.26 ms | 2.20 ms | +2.38% | - |
| `project_algebraic_fiber_polynomial_image/strict_exact_zero_image_degree` | 88.03 us | 87.37 us - 88.79 us | 86.54 us | +0.28% | - |
| `project_bivariate_fiber/retained_first_quadratic` | 1.68 us | 1.67 us - 1.70 us | 1.66 us | -0.72% | - |
| `project_bivariate_fiber/retained_second_quadratic` | 1.57 us | 1.56 us - 1.58 us | 1.56 us | -4.07% | - |
| `project_selected_tensor_fiber_via_tagged_norm/distinct_quadratic_carriers` | 162.71 us | 162.04 us - 163.44 us | 162.06 us | +0.45% | - |
| `project_selected_tensor_fiber_via_tagged_norm/opposite_conjugate_cubic` | 139.91 us | 138.95 us - 141.04 us | 138.67 us | -1.69% | - |
| `project_selected_tensor_fiber_via_tagged_norm/repeated_degree_six_carriers` | 155.51 us | 155.06 us - 156.00 us | 154.65 us | +0.41% | - |
| `promoted_fuzz_worst_performers/failed_constraint_minimal_removals_seed_0` | 120.20 us | 79.05 us - 166.52 us | 91.32 us | - | - |
| `promoted_fuzz_worst_performers/failed_constraint_minimal_removals_seed_1` | 55.43 us | 49.67 us - 65.20 us | 52.35 us | - | - |
| `promoted_fuzz_worst_performers/failed_constraint_minimal_removals_seed_10` | 29.06 us | 28.37 us - 30.02 us | 28.47 us | - | - |
| `promoted_fuzz_worst_performers/failed_constraint_minimal_removals_seed_14` | 29.12 us | 28.36 us - 30.02 us | 28.29 us | - | - |
| `promoted_fuzz_worst_performers/failed_constraint_minimal_removals_seed_15` | 27.73 us | 27.57 us - 27.94 us | 27.66 us | - | - |
| `promoted_fuzz_worst_performers/failed_constraint_minimal_removals_seed_18` | 30.73 us | 28.71 us - 32.94 us | 28.96 us | - | - |
| `promoted_fuzz_worst_performers/failed_constraint_minimal_removals_seed_19` | 30.76 us | 28.51 us - 33.35 us | 29.17 us | - | - |
| `promoted_fuzz_worst_performers/failed_constraint_minimal_removals_seed_2` | 29.65 us | 28.31 us - 31.15 us | 29.01 us | -61.72% | - |
| `promoted_fuzz_worst_performers/failed_constraint_minimal_removals_seed_23` | 27.74 us | 27.44 us - 28.09 us | 27.55 us | - | - |
| `promoted_fuzz_worst_performers/failed_constraint_minimal_removals_seed_58` | 29.97 us | 27.78 us - 33.58 us | 27.96 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_second_order_contact_seed_0` | 48.42 us | 47.30 us - 49.72 us | 47.86 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_second_order_contact_seed_1` | 75.13 us | 52.25 us - 102.19 us | 51.30 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_second_order_contact_seed_12` | 31.74 us | 31.52 us - 32.03 us | 31.65 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_second_order_contact_seed_13` | 31.10 us | 30.84 us - 31.39 us | 31.05 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_second_order_contact_seed_2` | 48.08 us | 47.62 us - 48.53 us | 47.94 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_second_order_contact_seed_20` | 32.43 us | 31.39 us - 33.55 us | 32.17 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_second_order_contact_seed_21` | 33.12 us | 32.11 us - 34.13 us | 33.45 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_second_order_contact_seed_22` | 32.39 us | 32.01 us - 32.78 us | 32.26 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_second_order_contact_seed_3` | 32.80 us | 32.45 us - 33.19 us | 32.51 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_second_order_contact_seed_7` | 31.75 us | 31.56 us - 31.95 us | 31.65 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_second_order_contact_seed_8` | 31.24 us | 31.08 us - 31.40 us | 31.23 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_second_order_contact_seed_9` | 32.00 us | 31.54 us - 32.46 us | 31.88 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_tangent_seed_0` | 64.01 us | 47.30 us - 93.78 us | 47.93 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_tangent_seed_1` | 68.86 us | 54.05 us - 88.17 us | 54.71 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_tangent_seed_2` | 235.20 us | 128.31 us - 361.63 us | 194.29 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_tangent_seed_21` | 31.77 us | 30.95 us - 32.95 us | 31.00 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_tangent_seed_22` | 32.09 us | 30.58 us - 34.37 us | 31.05 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_tangent_seed_23` | 34.15 us | 32.42 us - 36.32 us | 32.49 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_tangent_seed_50` | 31.96 us | 31.74 us - 32.22 us | 31.79 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_tangent_seed_6` | 31.07 us | 30.73 us - 31.48 us | 30.79 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_cubic_curve_tangent_seed_9` | 30.25 us | 29.90 us - 30.69 us | 30.23 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_line_tangent_seed_0` | 49.13 us | 44.52 us - 55.86 us | 46.86 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_line_tangent_seed_1` | 78.04 us | 48.92 us - 113.59 us | 49.16 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_line_tangent_seed_12` | 32.60 us | 31.57 us - 33.88 us | 31.76 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_line_tangent_seed_2` | 74.85 us | 58.42 us - 92.31 us | 57.53 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_line_tangent_seed_6` | 31.40 us | 30.76 us - 32.22 us | 31.21 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_line_tangent_seed_7` | 31.67 us | 30.67 us - 32.75 us | 31.29 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_arc_line_tangent_seed_9` | 33.90 us | 32.48 us - 35.54 us | 32.90 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_c2_seed_0` | 60.37 us | 48.52 us - 79.35 us | 50.16 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_c2_seed_1` | 50.16 us | 49.20 us - 51.32 us | 49.66 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_c2_seed_2` | 62.18 us | 52.56 us - 74.22 us | 52.40 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_c2_seed_9` | 31.23 us | 30.82 us - 31.69 us | 31.02 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_g2_seed_0` | 60.06 us | 49.41 us - 74.10 us | 50.37 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_g2_seed_1` | 53.63 us | 52.44 us - 54.99 us | 53.65 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_g2_seed_2` | 111.88 us | 52.78 us - 190.36 us | 50.02 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_g2_seed_22` | 32.19 us | 31.54 us - 33.05 us | 31.62 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_g2_seed_5` | 33.13 us | 31.91 us - 35.13 us | 32.13 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_g2_seed_7` | 34.48 us | 32.11 us - 37.64 us | 32.90 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_g2_seed_9` | 31.21 us | 30.60 us - 31.89 us | 31.11 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_tangent_seed_0` | 168.80 us | 82.50 us - 289.15 us | 104.46 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_tangent_seed_1` | 87.88 us | 57.19 us - 124.48 us | 64.04 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_tangent_seed_2` | 143.69 us | 96.87 us - 189.69 us | 165.03 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_tangent_seed_21` | 33.20 us | 32.03 us - 34.59 us | 31.91 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_tangent_seed_6` | 32.64 us | 31.24 us - 34.41 us | 31.37 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_cubic_curve_tangent_seed_9` | 31.97 us | 31.18 us - 32.85 us | 31.50 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_line_tangent_seed_0` | 159.72 us | 107.88 us - 212.92 us | 167.51 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_line_tangent_seed_17` | 32.49 us | 31.22 us - 34.10 us | 31.24 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_line_tangent_seed_2` | 51.96 us | 49.82 us - 54.24 us | 52.86 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_line_tangent_seed_21` | 32.86 us | 31.91 us - 33.85 us | 33.03 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_line_tangent_seed_47` | 31.19 us | 30.97 us - 31.46 us | 31.08 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_curve_line_tangent_seed_9` | 33.20 us | 32.60 us - 33.88 us | 32.88 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_line_tangent_seed_0` | 53.61 us | 49.02 us - 61.14 us | 50.34 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_line_tangent_seed_1` | 50.78 us | 49.36 us - 52.32 us | 50.08 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_line_tangent_seed_20` | 34.80 us | 32.10 us - 38.00 us | 32.14 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_line_tangent_seed_21` | 32.93 us | 32.01 us - 34.08 us | 32.22 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_line_tangent_seed_22` | 31.37 us | 30.66 us - 32.19 us | 31.18 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_line_tangent_seed_23` | 31.13 us | 30.77 us - 31.47 us | 31.14 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_line_tangent_seed_6` | 31.64 us | 30.89 us - 32.95 us | 31.01 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_cubic_line_tangent_seed_9` | 33.26 us | 32.02 us - 34.81 us | 32.27 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_distance_range_seed_1` | 32.97 us | 32.08 us - 34.15 us | 32.22 us | -38.57% | - |
| `promoted_fuzz_worst_performers/sketch_projected_distance_range_seed_10` | 33.25 us | 32.09 us - 34.84 us | 32.23 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_distance_range_seed_2` | 65.27 us | 54.70 us - 78.05 us | 54.31 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_distance_range_seed_8` | 32.10 us | 31.98 us - 32.25 us | 32.06 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_distance_range_seed_9` | 33.58 us | 32.41 us - 35.03 us | 32.51 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_distance_seed_1` | 58.09 us | 51.99 us - 65.70 us | 54.97 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_distance_seed_17` | 31.25 us | 30.52 us - 32.36 us | 30.76 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_distance_seed_2` | 49.47 us | 48.14 us - 51.08 us | 49.05 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_distance_seed_22` | 31.15 us | 30.86 us - 31.45 us | 31.13 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_distance_seed_6` | 32.76 us | 32.04 us - 33.51 us | 32.50 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_distance_seed_9` | 32.94 us | 31.14 us - 36.25 us | 31.47 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_equal_length_seed_1` | 51.67 us | 46.94 us - 58.41 us | 49.45 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_equal_length_seed_2` | 54.22 us | 47.64 us - 63.92 us | 48.08 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_equal_length_seed_28` | 32.13 us | 31.57 us - 32.89 us | 31.75 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_equal_point_distance_point_line_distance_seed_0` | 45.90 us | 45.05 us - 47.02 us | 45.14 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_equal_point_distance_point_line_distance_seed_1` | 54.10 us | 47.41 us - 63.64 us | 47.65 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_equal_point_distance_point_line_distance_seed_2` | 51.54 us | 48.90 us - 54.83 us | 50.72 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_equal_point_line_distances_seed_0` | 53.49 us | 46.63 us - 66.37 us | 47.11 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_equal_point_line_distances_seed_1` | 69.95 us | 53.03 us - 90.57 us | 52.42 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_equal_point_line_distances_seed_19` | 32.32 us | 30.59 us - 34.69 us | 30.69 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_equal_point_line_distances_seed_2` | 45.92 us | 45.50 us - 46.33 us | 45.96 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_equal_point_line_distances_seed_9` | 33.69 us | 32.81 us - 34.91 us | 33.01 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_equal_point_point_distances_seed_0` | 50.71 us | 39.36 us - 66.33 us | 45.69 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_equal_point_point_distances_seed_1` | 54.92 us | 46.62 us - 65.83 us | 47.12 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_equal_point_point_distances_seed_2` | 51.88 us | 48.21 us - 54.20 us | 53.52 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_equal_point_point_distances_seed_6` | 34.69 us | 32.28 us - 37.47 us | 33.81 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_equal_point_point_distances_seed_9` | 32.36 us | 31.43 us - 33.68 us | 31.82 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_difference_seed_0` | 119.88 us | 64.66 us - 199.51 us | 68.04 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_difference_seed_1` | 72.87 us | 58.56 us - 88.85 us | 69.64 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_difference_seed_2` | 137.51 us | 81.43 us - 204.57 us | 106.77 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_difference_seed_20` | 32.37 us | 31.21 us - 33.78 us | 32.28 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_difference_seed_246` | 34.10 us | 33.14 us - 35.09 us | 34.17 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_difference_seed_6` | 32.19 us | 31.03 us - 33.46 us | 31.76 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_point_line_distance_seed_0` | 52.25 us | 47.79 us - 58.73 us | 48.07 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_point_line_distance_seed_1` | 56.62 us | 48.73 us - 69.97 us | 49.26 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_point_line_distance_seed_2` | 58.43 us | 51.05 us - 67.40 us | 52.70 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_point_line_distance_seed_20` | 32.99 us | 31.61 us - 34.48 us | 32.66 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_point_line_distance_seed_6` | 31.00 us | 30.57 us - 31.51 us | 30.87 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_ratio_seed_0` | 83.18 us | 48.39 us - 138.65 us | 48.91 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_ratio_seed_1` | 152.53 us | 84.03 us - 230.29 us | 75.74 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_ratio_seed_17` | 31.45 us | 30.77 us - 32.24 us | 31.13 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_ratio_seed_2` | 95.13 us | 56.30 us - 160.31 us | 58.47 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_ratio_seed_22` | 33.68 us | 31.87 us - 35.93 us | 31.84 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_ratio_seed_7` | 31.89 us | 30.97 us - 33.54 us | 31.04 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_length_ratio_seed_71` | 32.78 us | 31.33 us - 34.69 us | 31.38 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_arc_sweep_length_seed_0` | 143.23 us | 48.54 us - 311.32 us | 48.11 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_arc_sweep_length_seed_1` | 90.71 us | 57.09 us - 150.91 us | 58.52 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_arc_sweep_length_seed_17` | 33.53 us | 31.81 us - 35.42 us | 32.48 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_arc_sweep_length_seed_19` | 32.03 us | 31.08 us - 33.16 us | 31.55 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_arc_sweep_length_seed_2` | 53.88 us | 48.71 us - 62.67 us | 49.20 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_arc_sweep_length_seed_6` | 33.39 us | 31.77 us - 35.11 us | 33.39 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_circle_tangent_seed_0` | 82.36 us | 59.02 us - 113.03 us | 58.28 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_circle_tangent_seed_1` | 52.15 us | 47.39 us - 59.14 us | 48.03 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_circle_tangent_seed_17` | 32.96 us | 32.25 us - 33.70 us | 32.73 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_circle_tangent_seed_2` | 50.85 us | 47.67 us - 55.80 us | 48.03 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_circle_tangent_seed_78` | 63.36 us | 52.18 us - 77.77 us | 52.95 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_length_range_seed_0` | 48.22 us | 46.95 us - 49.53 us | 48.10 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_length_range_seed_1` | 58.12 us | 51.18 us - 69.98 us | 52.11 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_length_range_seed_2` | 83.81 us | 69.08 us - 97.60 us | 90.53 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_length_range_seed_22` | 33.32 us | 32.26 us - 34.59 us | 32.45 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_length_range_seed_5` | 32.69 us | 32.20 us - 33.40 us | 32.32 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_length_range_seed_8` | 32.30 us | 30.96 us - 33.85 us | 31.27 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_orientation_seed_0` | 75.19 us | 58.42 us - 93.55 us | 75.22 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_orientation_seed_1` | 35.07 us | 33.15 us - 37.17 us | 34.10 us | -56.58% | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_orientation_seed_2` | 56.63 us | 52.21 us - 62.07 us | 53.31 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_orientation_seed_92` | 34.43 us | 32.66 us - 36.62 us | 33.62 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_radius_equality_seed_0` | 44.35 us | 40.81 us - 47.58 us | 46.07 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_radius_equality_seed_1` | 208.11 us | 62.49 us - 369.28 us | 53.03 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_radius_equality_seed_2` | 226.40 us | 95.96 us - 398.01 us | 151.95 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_radius_equality_seed_7` | 32.08 us | 30.98 us - 33.74 us | 31.18 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_radius_equality_seed_92` | 30.70 us | 30.42 us - 31.05 us | 30.56 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_symmetry_seed_0` | 67.20 us | 50.40 us - 87.56 us | 51.80 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_symmetry_seed_1` | 46.72 us | 46.24 us - 47.21 us | 46.32 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_line_symmetry_seed_2` | 55.00 us | 48.80 us - 61.73 us | 52.53 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_oriented_angle_seed_0` | 67.99 us | 48.74 us - 96.64 us | 49.38 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_oriented_angle_seed_1` | 52.06 us | 47.07 us - 60.44 us | 47.74 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_oriented_angle_seed_121` | 32.57 us | 31.66 us - 33.64 us | 31.68 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_oriented_angle_seed_2` | 59.41 us | 49.28 us - 75.16 us | 51.37 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_oriented_angle_seed_20` | 32.38 us | 31.53 us - 33.44 us | 31.77 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_concentric_seed_0` | 64.04 us | 52.66 us - 80.54 us | 53.58 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_concentric_seed_1` | 75.06 us | 51.95 us - 103.02 us | 53.27 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_concentric_seed_18` | 30.82 us | 30.54 us - 31.05 us | 30.93 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_concentric_seed_2` | 116.46 us | 48.45 us - 192.03 us | 47.62 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_distance_difference_seed_0` | 149.72 us | 110.71 us - 186.66 us | 166.15 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_distance_difference_seed_1` | 134.91 us | 48.47 us - 260.23 us | 49.36 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_distance_difference_seed_17` | 33.51 us | 32.32 us - 34.79 us | 32.57 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_distance_difference_seed_2` | 70.81 us | 49.13 us - 95.39 us | 49.30 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_distance_ratio_seed_0` | 61.87 us | 52.41 us - 75.82 us | 53.36 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_distance_ratio_seed_1` | 110.65 us | 60.98 us - 174.40 us | 63.59 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_distance_ratio_seed_136` | 31.51 us | 30.83 us - 32.41 us | 31.36 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_distance_ratio_seed_2` | 50.13 us | 45.23 us - 58.16 us | 46.81 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_distance_ratio_seed_21` | 32.93 us | 31.99 us - 34.16 us | 32.14 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_distance_ratio_seed_22` | 32.67 us | 31.67 us - 33.83 us | 32.44 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_distance_ratio_seed_8` | 32.20 us | 31.29 us - 33.45 us | 31.49 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_line_distance_range_seed_0` | 48.87 us | 47.78 us - 50.46 us | 48.34 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_line_distance_range_seed_1` | 55.81 us | 50.69 us - 63.02 us | 52.52 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_line_distance_range_seed_2` | 31.74 us | 31.51 us - 31.99 us | 31.79 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_line_distance_seed_0` | 57.06 us | 51.26 us - 63.75 us | 54.95 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_line_distance_seed_1` | 64.82 us | 56.45 us - 76.14 us | 55.84 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_line_radius_equality_seed_0` | 50.41 us | 46.51 us - 57.45 us | 47.06 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_line_radius_equality_seed_1` | 73.59 us | 57.59 us - 92.22 us | 60.46 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_line_radius_equality_seed_7` | 31.09 us | 30.80 us - 31.41 us | 30.84 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_arc_seed_0` | 133.29 us | 48.94 us - 256.58 us | 50.97 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_arc_seed_1` | 48.35 us | 48.14 us - 48.61 us | 48.26 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_arc_seed_17` | 33.67 us | 32.98 us - 34.79 us | 33.13 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_arc_seed_2` | 51.86 us | 47.98 us - 58.48 us | 48.97 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_arc_seed_8` | 31.74 us | 30.77 us - 32.88 us | 31.10 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_circle_seed_0` | 40.88 us | 35.77 us - 45.93 us | 44.24 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_circle_seed_1` | 58.64 us | 49.84 us - 75.42 us | 50.53 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_circle_seed_2` | 32.79 us | 31.95 us - 33.73 us | 32.53 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_circle_seed_6` | 31.31 us | 30.77 us - 32.01 us | 30.92 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_cubic_curve_seed_0` | 81.29 us | 54.66 us - 113.80 us | 55.66 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_cubic_curve_seed_1` | 107.21 us | 71.24 us - 152.52 us | 90.67 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_cubic_curve_seed_17` | 34.34 us | 32.25 us - 36.62 us | 33.87 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_cubic_curve_seed_2` | 158.74 us | 80.37 us - 263.65 us | 122.62 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_cubic_curve_seed_21` | 32.79 us | 31.83 us - 33.73 us | 32.83 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_cubic_curve_seed_8` | 34.59 us | 32.32 us - 37.31 us | 32.72 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_cubic_seed_0` | 130.84 us | 68.96 us - 210.98 us | 87.10 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_cubic_seed_1` | 88.03 us | 62.29 us - 116.20 us | 66.54 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_cubic_seed_17` | 33.57 us | 32.26 us - 35.01 us | 33.36 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_cubic_seed_18` | 31.89 us | 31.16 us - 32.70 us | 31.44 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_cubic_seed_2` | 124.64 us | 91.53 us - 160.46 us | 101.18 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_cubic_seed_22` | 30.69 us | 30.30 us - 31.28 us | 30.51 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_on_cubic_seed_6` | 32.22 us | 31.31 us - 33.27 us | 31.44 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_radius_equality_seed_0` | 130.48 us | 95.08 us - 165.87 us | 132.61 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_radius_equality_seed_1` | 67.76 us | 60.57 us - 75.05 us | 66.97 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_radius_equality_seed_17` | 33.43 us | 32.25 us - 35.02 us | 32.80 us | - | - |
| `promoted_fuzz_worst_performers/sketch_projected_point_radius_equality_seed_2` | 48.24 us | 46.86 us - 49.98 us | 47.49 us | - | - |
| `promoted_slow_offender_score/replay_promoted_100` | 3.61 ms | 3.49 ms - 3.75 ms | 3.63 ms | -5.40% | - |
| `real_representations/full_solver_boundary/ConstOffset` | 40.31 us | 40.12 us - 40.50 us | 40.26 us | -4.43% | - |
| `real_representations/full_solver_boundary/ConstProduct` | 25.16 us | 24.86 us - 25.55 us | 24.95 us | -0.18% | - |
| `real_representations/full_solver_boundary/ConstProductSqrt` | 38.31 us | 37.48 us - 39.29 us | 37.55 us | +0.97% | - |
| `real_representations/full_solver_boundary/Exp` | 25.00 us | 24.66 us - 25.42 us | 24.70 us | +3.44% | - |
| `real_representations/full_solver_boundary/Irrational` | 39.21 us | 38.96 us - 39.42 us | 39.26 us | -4.68% | - |
| `real_representations/full_solver_boundary/Ln` | 40.53 us | 40.22 us - 40.89 us | 40.43 us | -2.84% | - |
| `real_representations/full_solver_boundary/LnAffine` | 85.30 us | 83.07 us - 87.96 us | 83.22 us | -0.41% | - |
| `real_representations/full_solver_boundary/LnProduct` | 56.73 us | 55.76 us - 57.83 us | 56.42 us | -1.20% | - |
| `real_representations/full_solver_boundary/Log10` | 73.05 us | 72.17 us - 73.96 us | 72.69 us | -1.65% | - |
| `real_representations/full_solver_boundary/Log2` | 73.84 us | 73.14 us - 74.98 us | 73.30 us | -1.62% | - |
| `real_representations/full_solver_boundary/One` | 6.22 us | 6.12 us - 6.32 us | 6.14 us | +4.46% | - |
| `real_representations/full_solver_boundary/Pi` | 10.08 us | 9.92 us - 10.24 us | 10.00 us | +4.41% | - |
| `real_representations/full_solver_boundary/PiExp` | 18.08 us | 17.92 us - 18.34 us | 17.98 us | -0.08% | - |
| `real_representations/full_solver_boundary/PiInv` | 11.75 us | 11.71 us - 11.78 us | 11.76 us | -1.19% | - |
| `real_representations/full_solver_boundary/PiInvExp` | 19.69 us | 19.55 us - 19.83 us | 19.60 us | -1.47% | - |
| `real_representations/full_solver_boundary/PiPow` | 15.39 us | 15.31 us - 15.47 us | 15.33 us | -0.87% | - |
| `real_representations/full_solver_boundary/PiSqrt` | 19.81 us | 19.57 us - 20.10 us | 19.79 us | +0.16% | - |
| `real_representations/full_solver_boundary/Pow10` | 241.38 us | 237.87 us - 245.28 us | 239.78 us | +1.95% | - |
| `real_representations/full_solver_boundary/Pow2` | 214.79 us | 211.17 us - 218.27 us | 214.78 us | +5.75% | - |
| `real_representations/full_solver_boundary/SinPi` | 196.43 us | 192.65 us - 200.20 us | 195.14 us | +0.51% | - |
| `real_representations/full_solver_boundary/Sqrt` | 12.85 us | 12.77 us - 12.95 us | 12.78 us | -3.65% | - |
| `real_representations/full_solver_boundary/TanPi` | 591.71 us | 584.08 us - 601.01 us | 587.62 us | -0.43% | - |
| `reduce_bivariate_rational_function/dense_degree_15_proportional` | 7.51 us | 7.45 us - 7.57 us | 7.40 us | -1.64% | - |
| `reduce_bivariate_rational_function/nonmonic_retained_denominator` | 1.69 us | 1.68 us - 1.71 us | 1.66 us | -24.01% | - |
| `refine_isolated_univariate_polynomial_interval/exact_linear_witness` | 230.50 ns | 229.88 ns - 231.29 ns | 229.53 ns | +59.28% | - |
| `refine_isolated_univariate_polynomial_interval/exact_quadratic_witness` | 290.36 ns | 289.69 ns - 291.10 ns | 289.02 ns | +100.78% | - |
| `refine_isolated_univariate_polynomial_interval/exact_real_coefficients_steps_4` | 2.07 us | 2.06 us - 2.09 us | 2.05 us | -87.83% | - |
| `refine_isolated_univariate_polynomial_interval/lower_endpoint_root` | 3.59 us | 3.58 us - 3.60 us | 3.58 us | +1.15% | - |
| `refine_isolated_univariate_polynomial_interval/quadratic_steps_4` | 957.29 ns | 950.58 ns - 965.24 ns | 943.78 ns | -88.52% | - |
| `refine_isolated_univariate_polynomial_interval/repeated_quadratic_steps_4` | 9.47 us | 9.43 us - 9.51 us | 9.40 us | -29.84% | - |
| `refine_isolated_univariate_polynomial_interval/upper_endpoint_root` | 3.97 us | 3.96 us - 3.99 us | 3.95 us | +4.07% | - |
| `refine_isolated_univariate_polynomial_interval/wide_exact_quadratic_witness` | 496.64 ns | 494.87 ns - 498.62 ns | 492.64 ns | -0.91% | - |
| `refine_isolated_univariate_polynomial_interval/width_already_satisfied` | 461.70 ns | 457.14 ns - 467.19 ns | 453.04 ns | -87.32% | - |
| `represent_algebraic_tensor_image/sum_four_roots` | 386.58 us | 385.35 us - 387.85 us | 385.77 us | -1.16% | - |
| `represent_algebraic_tensor_image/sum_four_roots_padded_256` | 494.35 us | 491.33 us - 498.34 us | 489.74 us | -7.05% | - |
| `represent_algebraic_tensor_image/two_conjugate_cubic_roots` | 167.07 us | 166.14 us - 168.16 us | 166.07 us | -0.52% | - |
| `represent_algebraic_tensor_image/two_conjugate_repeated_degree_six_roots` | 185.67 us | 184.56 us - 186.96 us | 184.06 us | -7.94% | - |
| `represented_root_sign/exact_rational_point` | 22.54 ns | 22.46 ns - 22.65 ns | 22.58 ns | -20.40% | - |
| `represented_root_sign/exact_real_point` | 58.48 ns | 58.41 ns - 58.56 ns | 58.37 ns | +0.39% | - |
| `represented_root_sign/isolating_interval` | 12.34 ns | 12.30 ns - 12.39 ns | 12.27 ns | -46.64% | - |
| `resultant_parametric_curve_intersection` | 2.47 us | 2.45 us - 2.49 us | 2.44 us | -2.07% | - |
| `resultant_rational_parametric_curve_intersection` | 2.39 us | 2.38 us - 2.40 us | 2.37 us | -7.74% | - |
| `resultant_tensor_polynomial_univariate_constraint` | 1.21 us | 1.21 us - 1.22 us | 1.20 us | -19.60% | - |
| `resultant_tensor_polynomial_univariate_constraint/compact_grid_2x2` | 6.66 us | 6.56 us - 6.78 us | 6.47 us | -7.43% | - |
| `resultant_tensor_polynomial_univariate_constraint/direct_real_norm` | 1.55 us | 1.54 us - 1.56 us | 1.53 us | +1.42% | - |
| `resultant_tensor_polynomial_univariate_constraint/padded_closed_form_norm_1024` | 8.50 us | 8.43 us - 8.58 us | 8.36 us | -98.30% | - |
| `resultant_tensor_polynomial_univariate_constraint/padded_direct_norm_1024` | 9.85 us | 9.75 us - 9.95 us | 9.71 us | -94.97% | - |
| `resultant_tensor_polynomial_univariate_constraint/padded_grid_64x64` | 69.35 us | 68.79 us - 70.02 us | 68.39 us | -93.86% | - |
| `resultant_tensor_polynomial_univariate_constraint/strict_exact_normal_constraint` | 202.58 us | 200.54 us - 204.75 us | 198.43 us | - | - |
| `resultant_tensor_polynomial_univariate_constraint/strict_exact_zero_output_degree` | 19.51 us | 19.43 us - 19.60 us | 19.40 us | +459.18% | - |
| `resultant_trivariate_polynomial_univariate_constraint` | 28.00 us | 27.82 us - 28.20 us | 27.65 us | +0.65% | - |
| `resultant_trivariate_polynomial_univariate_constraint/strict_exact_normal_constraint` | 460.90 us | 455.85 us - 466.75 us | 453.17 us | - | - |
| `resultant_univariate_polynomials` | 1.21 us | 1.19 us - 1.22 us | 1.19 us | +1.58% | - |
| `resultant_univariate_polynomials/strict_exact_normal_trimming` | 92.71 us | 92.21 us - 93.26 us | 91.69 us | - | - |
| `schedule_univariate_resultant_pairs` | 2.79 us | 2.77 us - 2.82 us | 2.75 us | +1.70% | - |
| `solve_sparse_linear_system_bareiss_pattern_preserving` | 3.71 us | 3.69 us - 3.73 us | 3.68 us | +1.65% | - |
| `solve_sparse_linear_system_bareiss_pattern_preserving/strict_exact_normal_pivot` | 456.30 us | 453.03 us - 459.89 us | 448.89 us | - | - |
| `sparse_bareiss_arrowhead_32/authored_order` | 3.41 ms | 3.40 ms - 3.43 ms | 3.38 ms | -0.83% | - |
| `sparse_bareiss_arrowhead_32/minimum_degree` | 783.36 us | 779.75 us - 787.38 us | 777.48 us | +0.48% | - |
| `sparse_bareiss_tridiagonal_32/authored_order` | 253.48 us | 251.77 us - 255.36 us | 250.41 us | -1.12% | - |
| `sparse_bareiss_tridiagonal_32/minimum_degree` | 322.78 us | 320.66 us - 325.19 us | 320.30 us | +1.09% | - |
| `square_free_part/degree_64_repeated` | 25.61 us | 25.11 us - 26.52 us | 25.05 us | -1.11% | - |
| `square_free_part/degree_64_square_free` | 5.81 us | 5.79 us - 5.83 us | 5.78 us | +1.73% | - |
| `square_free_part/exact_real_repeated_quadratic` | 1.58 us | 1.57 us - 1.59 us | 1.56 us | -50.81% | - |
| `square_root_algebraic_root` | 10.87 us | 10.81 us - 10.94 us | 10.77 us | -0.71% | - |
| `square_root_algebraic_root/adaptive_neighbor` | 55.49 us | 55.32 us - 55.67 us | 55.47 us | +2.92% | - |
| `square_root_algebraic_root/repeated_exact_witness` | 2.25 us | 2.24 us - 2.26 us | 2.23 us | -91.05% | - |
| `square_root_algebraic_root/repeated_rational_square_witness` | 1.16 us | 1.15 us - 1.16 us | 1.15 us | -3.75% | - |
| `subdivide_bernstein_univariate_polynomial_interval_roots` | 57.00 us | 56.71 us - 57.36 us | 56.62 us | -76.04% | - |
| `subresultant_chain_univariate_polynomials` | 852.84 ns | 845.57 ns - 860.69 ns | 837.21 ns | -1.37% | - |
| `transform_algebraic_root_affine` | 467.24 ns | 464.95 ns - 469.71 ns | 461.68 ns | -54.91% | - |
| `transform_algebraic_root_affine/degree_16` | 17.30 us | 17.26 us - 17.35 us | 17.23 us | -3.41% | - |
| `transform_algebraic_root_affine/degree_16_dense` | 17.84 us | 17.78 us - 17.90 us | 17.72 us | -7.86% | - |
| `transform_algebraic_root_affine/exact_real_source` | 774.88 ns | 772.04 ns - 778.14 ns | 769.60 ns | -43.33% | - |
| `transform_algebraic_root_affine/exact_witness` | 316.31 ns | 315.28 ns - 317.43 ns | 314.89 ns | +8.83% | - |
| `transform_algebraic_root_affine/negative_scaling` | 326.33 ns | 324.76 ns - 328.19 ns | 323.23 ns | +5.40% | - |
| `transform_algebraic_root_affine/negative_scaling_endpoint_refinement` | 12.35 us | 12.30 us - 12.40 us | 12.23 us | - | - |
| `transform_algebraic_root_affine/scaling` | 320.63 ns | 319.67 ns - 321.69 ns | 319.10 ns | -0.28% | - |
| `transform_algebraic_root_affine/translation` | 406.63 ns | 405.74 ns - 407.64 ns | 404.59 ns | -0.26% | - |
| `transform_algebraic_root_mobius` | 1.20 us | 1.19 us - 1.20 us | 1.19 us | -1.51% | - |
| `transform_algebraic_root_mobius/constant_denominator` | 598.52 ns | 596.42 ns - 600.80 ns | 595.37 ns | - | - |
| `transform_algebraic_root_mobius/exact_real_source` | 1.83 us | 1.83 us - 1.84 us | 1.82 us | - | - |
| `transform_algebraic_root_mobius/exact_witness` | 527.66 ns | 526.33 ns - 529.20 ns | 526.10 ns | -2.86% | - |
| `transform_algebraic_root_mobius/excluded_endpoint_pole_refinement` | 4.65 us | 4.63 us - 4.67 us | 4.62 us | - | - |
| `transform_algebraic_root_mobius/reciprocal` | 566.06 ns | 563.44 ns - 569.05 ns | 560.96 ns | +10.55% | - |
| `transform_algebraic_root_mobius/reversed_endpoint_refinement` | 12.97 us | 12.85 us - 13.15 us | 12.78 us | - | - |
| `transform_algebraic_root_polynomial_image` | 2.96 us | 2.95 us - 2.97 us | 2.94 us | +2.68% | - |
| `transform_algebraic_root_polynomial_image/constant` | 119.89 ns | 119.62 ns - 120.18 ns | 119.43 ns | -51.30% | - |
| `transform_algebraic_root_polynomial_image/exact_witness` | 438.86 ns | 437.48 ns - 440.33 ns | 436.41 ns | -6.25% | - |
| `transform_algebraic_root_polynomial_image/foreign_root_refinement` | 31.35 us | 31.00 us - 31.88 us | 30.83 us | +1.56% | - |
| `transform_algebraic_root_polynomial_image/rational_modulus_image` | 181.05 ns | 180.76 ns - 181.37 ns | 180.57 ns | -97.94% | - |
| `transform_algebraic_root_polynomial_image/repeated_degree_eight_carrier` | 14.22 us | 14.15 us - 14.30 us | 14.10 us | -1.78% | - |
| `transform_algebraic_root_polynomial_image/repeated_degree_six_carrier` | 43.66 us | 43.48 us - 43.85 us | 43.48 us | -6.95% | - |
| `transform_algebraic_root_polynomial_image/square_free_degree_six_carrier` | 27.87 us | 27.73 us - 28.03 us | 27.60 us | -8.10% | - |
| `transform_algebraic_root_polynomial_image/stationary` | 3.53 us | 3.52 us - 3.55 us | 3.51 us | -25.42% | - |
| `transform_algebraic_root_rational_image` | 2.93 us | 2.93 us - 2.94 us | 2.92 us | -30.48% | - |
| `transform_algebraic_root_rational_image/certified_algebraic_pole` | 1.21 us | 1.21 us - 1.22 us | 1.20 us | +1.40% | - |
| `transform_algebraic_root_rational_image/constant_degree_6` | 3.61 us | 3.60 us - 3.63 us | 3.58 us | -57.26% | - |
| `transform_algebraic_root_rational_image/dependency_broadened_denominator` | 10.15 us | 10.09 us - 10.22 us | 10.03 us | -0.46% | - |
| `transform_algebraic_root_rational_image/monotone_quadratic` | 2.05 us | 2.05 us - 2.06 us | 2.04 us | -1.74% | - |
| `transform_algebraic_root_rational_image/shared_linear_factor` | 6.92 us | 6.90 us - 6.94 us | 6.89 us | -1.24% | - |
| `transform_algebraic_root_rational_image/shared_nonmonic_linear_factor` | 7.51 us | 7.48 us - 7.54 us | 7.45 us | -58.99% | - |
| `transform_algebraic_root_rational_image/stationary_cubic` | 3.73 us | 3.72 us - 3.75 us | 3.71 us | -3.48% | - |
| `transform_algebraic_root_rational_image_degree_12_cubic` | 1.96 ms | 1.96 ms - 1.97 ms | 1.95 ms | - | - |
| `transform_algebraic_root_rational_images/batch_four_dependency_denominator` | 36.22 us | 36.10 us - 36.35 us | 36.04 us | -28.96% | - |
| `transform_algebraic_root_rational_images/batch_four_linear` | 12.52 us | 12.48 us - 12.56 us | 12.46 us | -4.43% | - |
| `transform_algebraic_roots_binary` | 24.72 us | 24.63 us - 24.82 us | 24.57 us | -32.50% | - |
| `transform_algebraic_roots_binary/repeated_degree_six_carriers` | 41.67 us | 41.46 us - 41.89 us | 41.29 us | -16.13% | - |
| `transform_algebraic_roots_binary/shared_repeated_degree_six_carrier` | 29.06 us | 29.00 us - 29.13 us | 28.96 us | -35.49% | - |
| `transform_algebraic_roots_binary_divide` | 21.53 us | 21.41 us - 21.67 us | 21.31 us | -24.33% | - |
| `transform_algebraic_roots_binary_multiply` | 20.23 us | 20.15 us - 20.32 us | 20.14 us | -25.03% | - |
| `transform_algebraic_roots_binary_subtract` | 28.63 us | 28.49 us - 28.77 us | 28.41 us | - | - |

<!-- END COMPLETE BENCHMARK REPORT -->
