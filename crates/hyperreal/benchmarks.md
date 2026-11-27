<!-- BEGIN promoted_slow_offender_score -->
## `promoted_slow_offender_score`

Deterministic lexicase score for the current 100 promoted slow offenders. The score is the average current best-of-five wall-clock probe across the promoted set; lower is better. Delta compares with the previous score recorded in this file, and derivative is the change in delta.

<!-- promoted_slow_score_nanos: 4879 -->
<!-- promoted_slow_previous_score_nanos: 4839 -->
<!-- promoted_slow_score_delta_nanos: 40 -->

| Metric | Value |
| --- | ---: |
| Cases scored | 100 |
| Average score | 4.879 us |
| Delta | 40 ns |
| Delta derivative | -177 ns |

| Rank | Current Time | Operation | Input |
| ---: | ---: | --- | --- |
| 1 | 10.389 us | `generated_tan_p96` | `generated[3486] -2 37/80` |
| 2 | 10.260 us | `generated_tan_p96` | `generated[18246] -1 187/188` |
| 3 | 10.129 us | `generated_tan_p96` | `generated[3756] -1 123/214` |
| 4 | 10.069 us | `generated_tan_p96` | `generated[5676] -1 215/229` |
| 5 | 9.979 us | `generated_tan_p96` | `generated[12081] -1 262/383` |
| 6 | 9.979 us | `generated_tan_p96` | `generated[13911] -2 134/427` |
| 7 | 9.970 us | `generated_tan_p96` | `generated[3591] -1 14/15` |
| 8 | 9.859 us | `generated_tan_p96` | `generated[14136] -1 79/106` |
| 9 | 9.849 us | `generated_tan_p96` | `generated[11691] 1 431/439` |
| 10 | 9.839 us | `generated_tan_p96` | `generated[8976] 1 71/73` |

<!-- END promoted_slow_offender_score -->

<!-- BEGIN numerical_micro -->
## `numerical_micro`

Low-level `Computable` microbenchmarks for approximation kernels, caches, structural facts, comparisons, and deep evaluator trees.

### `computable_cache`

Cold versus cached approximation of basic `Computable` expressions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_cache/ratio_approx_cold_p128` | 23.10 ns | 23.02 ns - 23.18 ns | Approximates a rational value at p=-128 from a fresh clone. |
| `computable_cache/ratio_approx_cached_p128` | 18.91 ns | 18.88 ns - 18.94 ns | Repeats an already cached rational approximation at p=-128. |
| `computable_cache/pi_approx_cold_p128` | 26.58 ns | 26.54 ns - 26.61 ns | Approximates pi at p=-128 from a fresh clone. |
| `computable_cache/pi_approx_cached_p128` | 19.08 ns | 18.98 ns - 19.20 ns | Repeats an already cached pi approximation at p=-128. |
| `computable_cache/pi_plus_tiny_cold_p128` | 27.16 ns | 27.04 ns - 27.31 ns | Approximates pi plus a tiny exact rational perturbation. |
| `computable_cache/pi_minus_tiny_cold_p128` | 27.12 ns | 27.02 ns - 27.25 ns | Approximates pi minus a tiny exact rational perturbation. |

### `computable_bounds`

Structural sign and bound discovery for deep or perturbed computable trees.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_bounds/deep_scaled_product_sign` | not run | not run | Finds the sign of a deep scaled product. |
| `computable_bounds/scaled_square_sign` | not run | not run | Finds the sign of repeated squaring with exact scale factors. |
| `computable_bounds/sqrt_scaled_square_sign` | not run | not run | Finds the sign after taking a square root of a scaled square. |
| `computable_bounds/deep_structural_bound_sign` | not run | not run | Finds sign through repeated multiply/inverse/negate structural transformations. |
| `computable_bounds/deep_structural_bound_sign_cached` | not run | not run | Reads the cached sign of the deep structural-bound chain. |
| `computable_bounds/deep_structural_bound_facts_cached` | 8.60 ns | 8.52 ns - 8.70 ns | Reads cached structural facts for the deep structural-bound chain. |
| `computable_bounds/perturbed_scaled_product_sign` | not run | not run | Finds sign for a deeply scaled value with a tiny perturbation. |
| `computable_bounds/perturbed_scaled_product_sign_until` | not run | not run | Refines sign for the perturbed scaled product only to p=-128. |
| `computable_bounds/pi_minus_tiny_sign` | not run | not run | Finds sign for pi minus a tiny exact rational. |
| `computable_bounds/pi_minus_tiny_sign_cached` | not run | not run | Reads cached sign for pi minus a tiny exact rational. |
| `computable_bounds/exp_unknown_sign_arg_sign` | not run | not run | Finds sign for exp(1 - pi), where exp can prove positivity structurally. |
| `computable_bounds/exp_unknown_sign_arg_sign_cached` | not run | not run | Reads cached sign for exp(1 - pi). |
| `computable_bounds/mixed_pi_e_sign_until_p0_cold` | 605.62 ns | 599.43 ns - 611.70 ns | Certifies a fresh mixed pi/e expression with the bounded binary64 interval filter. |
| `computable_bounds/near_pi_sign_until_p64_cold` | 525.02 ns | 517.52 ns - 532.66 ns | Certifies a fresh close pi/rational difference at its permitted p=-64 floor. |
| `computable_bounds/near_pi_sign_until_p0_inconclusive_cold` | 867.83 ns | 859.69 ns - 876.41 ns | Preserves an inconclusive p=0 result for the same close pi/rational difference. |
| `computable_bounds/unsupported_sin_difference_sign_until_p0_cold` | 1.029 us | 1.022 us - 1.037 us | Measures bounded-filter rejection and arbitrary-precision fallback for an unsupported sine expression. |
| `computable_bounds/unsupported_sin_difference_sign_until_p64_cold` | 4.380 us | 4.364 us - 4.401 us | Measures algebraic-certificate rejection after ordinary refinement reaches p=-64. |

### `computable_algebraic_roots`

Explicit bounded-degree roots and exact algebraic-zero certification.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_algebraic_roots/root5_construct` | 193.38 ns | 193.09 ns - 193.69 ns | Constructs the non-perfect fifth root of 17. |
| `computable_algebraic_roots/root5_interval_p128_cold` | 2.932 us | 2.920 us - 2.947 us | Computes a p=-128 certified interval for a freshly constructed fifth root. |
| `computable_algebraic_roots/root5_interval_p2048_cold` | 98.993 us | 98.868 us - 99.162 us | Computes a p=-2048 certified interval for a freshly constructed fifth root. |
| `computable_algebraic_roots/root9_interval_p128_cold` | 5.740 us | 5.716 us - 5.768 us | Computes a p=-128 certified interval at the explicit-root degree cap. |
| `computable_algebraic_roots/root9_interval_p2048_cold` | 272.922 us | 272.424 us - 273.477 us | Computes a p=-2048 certified interval at the explicit-root degree cap. |
| `computable_algebraic_roots/root10_interval_p128_fallback_cold` | 5.866 us | 5.858 us - 5.874 us | Exercises the retained exp/ln fallback just above the explicit-root degree cap. |
| `computable_algebraic_roots/eighth_root_near_dyadic_sign_p64_cold` | 5.963 us | 5.941 us - 5.989 us | Certifies an ordinary nonzero eighth-root difference before algebraic zero metadata is needed. |
| `computable_algebraic_roots/ramanujan_one_zero_sign_p2048_cold` | 43.311 us | 43.160 us - 43.476 us | Certifies the first archived Ramanujan radical identity as exactly zero. |
| `computable_algebraic_roots/ramanujan_two_zero_sign_p2048_cold` | 126.520 us | 126.210 us - 126.897 us | Certifies the nested archived Ramanujan radical identity as exactly zero. |
| `computable_algebraic_roots/many_digits_c10_zero_sign_p2048_cold` | 40.113 us | 40.029 us - 40.207 us | Certifies the archived mixed cube/fifth-root C10 identity as exactly zero. |

### `computable_compare`

Ordering and absolute-comparison shortcuts.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_compare/compare_to_opposite_sign` | 11.08 ns | 11.08 ns - 11.09 ns | Compares values with known opposite signs. |
| `computable_compare/compare_to_exact_msd_gap` | 18.55 ns | 18.49 ns - 18.62 ns | Compares values with a large exact magnitude gap. |
| `computable_compare/compare_to_clone_shared_composite` | 5.03 ns | 5.02 ns - 5.03 ns | Compares two handles that share one composite expression node. |
| `computable_compare/compare_absolute_exact_rational` | 4.61 ns | 4.58 ns - 4.64 ns | Compares exact rationals using an absolute error tolerance. |
| `computable_compare/compare_absolute_exact_rational_same_numerator` | 36.88 ns | 36.54 ns - 37.26 ns | Compares exact rationals with matching numerator magnitudes. |
| `computable_compare/compare_absolute_mixed_exact_leaf_kinds` | 24.18 ns | 24.10 ns - 24.27 ns | Compares opposite-sign exact values stored as `One` and `Ratio` leaves. |
| `computable_compare/compare_absolute_dominant_add` | 12.95 ns | 12.89 ns - 13.02 ns | Compares a dominant term against the same term plus a tiny addend. |
| `computable_compare/compare_absolute_exact_msd_gap` | 14.71 ns | 14.67 ns - 14.76 ns | Compares absolute values with a large exact magnitude gap. |

### `computable_transcendentals`

Low-level approximation kernels and deep expression-tree stress cases.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_transcendentals/e_constant_cold_p128` | 36.85 ns | 36.55 ns - 37.16 ns | Approximates the shared e constant from a fresh clone. |
| `computable_transcendentals/e_constant_cached_p128` | 19.28 ns | 19.25 ns - 19.32 ns | Repeats a cached approximation of e. |
| `computable_transcendentals/exp_cold_p128` | 4.080 us | 4.076 us - 4.085 us | Approximates exp(7/5) from a fresh clone. |
| `computable_transcendentals/exp_cached_p128` | 19.48 ns | 19.40 ns - 19.58 ns | Repeats a cached exp(7/5) approximation. |
| `computable_transcendentals/exp_deferred_constructor` | 29.80 ns | 29.62 ns - 30.02 ns | Constructs exp(7/2) without requesting an approximation. |
| `computable_transcendentals/exp_coarse_cold_p1` | 2.324 us | 2.308 us - 2.342 us | Approximates exp(3/2) at coarse precision, requiring certified range reduction. |
| `computable_transcendentals/exp_deferred_negative_cold_p128` | 97.109 us | 96.846 us - 97.418 us | Approximates exp(-10000), exercising deferred binary-scaling fallback. |
| `computable_transcendentals/exp_large_cold_p128` | 4.449 us | 4.440 us - 4.460 us | Approximates exp(128), exercising the bounded exact-integer power path. |
| `computable_transcendentals/expm1_tiny_cold_p128` | 836.57 ns | 833.67 ns - 839.96 ns | Approximates expm1(1/1000000) directly without subtractive cancellation. |
| `computable_transcendentals/expm1_coarse_cold_p1` | 3.075 us | 3.069 us - 3.081 us | Approximates expm1(3/2) at coarse precision after checking the argument range. |
| `computable_transcendentals/expm1_coarse_negative_cold_p1` | 53.77 ns | 53.05 ns - 54.49 ns | Approximates expm1(-32) at coarse precision, where zero is a valid result. |
| `computable_transcendentals/exp_negative_integer_cold_p128` | 2.129 us | 2.125 us - 2.134 us | Approximates exp(-32), retaining signed ln(2) range reduction. |
| `computable_transcendentals/exp_integer_limit_cold_p128` | 6.373 us | 6.356 us - 6.391 us | Approximates exp(256), guarding the binary e-power limit. |
| `computable_transcendentals/exp_integer_above_limit_cold_p128` | 11.888 us | 11.872 us - 11.906 us | Approximates exp(257), retaining the ln(2) range-reduction fallback. |
| `computable_transcendentals/exp_half_cold_p128` | 2.981 us | 2.971 us - 2.993 us | Approximates exp(1/2). |
| `computable_transcendentals/exp_near_limit_cold_p128` | 2.852 us | 2.849 us - 2.855 us | Approximates exp near a prescaling threshold. |
| `computable_transcendentals/exp_near_limit_cached_p128` | 18.96 ns | 18.94 ns - 18.99 ns | Repeats a cached near-threshold exp approximation. |
| `computable_transcendentals/exp_zero_cold_p128` | 57.85 ns | 57.26 ns - 58.44 ns | Approximates exp(0). |
| `computable_transcendentals/ln_cold_p128` | 3.096 us | 3.088 us - 3.108 us | Approximates ln(11/7). |
| `computable_transcendentals/ln_cached_p128` | 18.93 ns | 18.91 ns - 18.96 ns | Repeats a cached ln(11/7) approximation. |
| `computable_transcendentals/ln_smooth_rational_cold_p128` | 703.02 ns | 697.64 ns - 708.73 ns | Approximates ln(45/14), which can decompose into shared prime-log constants. |
| `computable_transcendentals/ln_nonsmooth_rational_cold_p128` | 2.524 us | 2.513 us - 2.535 us | Approximates ln(11/13), guarding the generic exact-rational log fallback. |
| `computable_transcendentals/ln_large_cold_p128` | 946.20 ns | 938.23 ns - 954.50 ns | Approximates ln(1024), exercising large-input reduction. |
| `computable_transcendentals/ln_large_cached_p128` | 19.06 ns | 19.01 ns - 19.11 ns | Repeats a cached ln(1024) approximation. |
| `computable_transcendentals/ln_tiny_cold_p128` | 212.91 ns | 211.12 ns - 214.68 ns | Approximates ln(2^-1024), exercising tiny-input reduction. |
| `computable_transcendentals/ln_near_limit_cold_p128` | 3.200 us | 3.197 us - 3.205 us | Approximates ln near the prescaled-ln limit. |
| `computable_transcendentals/ln_near_limit_cached_p128` | 18.95 ns | 18.92 ns - 18.99 ns | Repeats a cached near-limit ln approximation. |
| `computable_transcendentals/ln_one_cold_p128` | 21.36 ns | 21.13 ns - 21.56 ns | Approximates ln(1). |
| `computable_transcendentals/sqrt_cold_p128` | 743.12 ns | 740.41 ns - 745.76 ns | Approximates sqrt(2). |
| `computable_transcendentals/sqrt_squarefree_scaled_cold_p128` | 109.20 ns | 108.54 ns - 109.88 ns | Approximates sqrt(12), which can reduce to 2*sqrt(3). |
| `computable_transcendentals/sqrt_cached_p128` | 18.96 ns | 18.93 ns - 18.99 ns | Repeats a cached sqrt(2) approximation. |
| `computable_transcendentals/sqrt_single_scaled_square_cold_p128` | 838.16 ns | 836.70 ns - 839.77 ns | Builds and approximates sqrt((7*pi/8)^2). |
| `computable_transcendentals/sin_cold_p96` | 1.575 us | 1.570 us - 1.580 us | Approximates sin(7/5). |
| `computable_transcendentals/sin_cached_p96` | 21.90 ns | 21.83 ns - 21.97 ns | Repeats a cached sin(7/5) approximation. |
| `computable_transcendentals/cos_cold_p96` | 1.458 us | 1.454 us - 1.463 us | Approximates cos(7/5). |
| `computable_transcendentals/sin_f64_cold_p96` | 1.741 us | 1.735 us - 1.749 us | Approximates sin of the exact binary64-derived dyadic for 1.23456789. |
| `computable_transcendentals/cos_f64_cold_p96` | 1.650 us | 1.643 us - 1.657 us | Approximates cos of the exact binary64-derived dyadic for 1.23456789. |
| `computable_transcendentals/sin_1e6_cold_p96` | 2.311 us | 2.304 us - 2.319 us | Approximates sin(1000000). |
| `computable_transcendentals/cos_1e6_cold_p96` | 2.330 us | 2.317 us - 2.346 us | Approximates cos(1000000). |
| `computable_transcendentals/sin_1e30_cold_p96` | 2.138 us | 2.133 us - 2.145 us | Approximates sin(10^30). |
| `computable_transcendentals/cos_1e30_cold_p96` | 2.223 us | 2.217 us - 2.231 us | Approximates cos(10^30). |
| `computable_transcendentals/cos_cached_p96` | 18.93 ns | 18.90 ns - 18.96 ns | Repeats a cached cos(7/5) approximation. |
| `computable_transcendentals/tan_cold_p96` | 5.974 us | 5.965 us - 5.982 us | Approximates tan(7/5). |
| `computable_transcendentals/tan_cached_p96` | 18.94 ns | 18.92 ns - 18.97 ns | Repeats a cached tan(7/5) approximation. |
| `computable_transcendentals/sin_zero_cold_p96` | 21.87 ns | 21.59 ns - 22.13 ns | Approximates sin(0). |
| `computable_transcendentals/cos_zero_cold_p96` | 60.07 ns | 59.56 ns - 60.57 ns | Approximates cos(0). |
| `computable_transcendentals/tan_zero_cold_p96` | 21.61 ns | 21.35 ns - 21.85 ns | Approximates tan(0). |
| `computable_transcendentals/tan_near_half_pi_cold_p96` | 10.363 us | 10.300 us - 10.441 us | Approximates tangent near pi/2. |
| `computable_transcendentals/tan_near_half_pi_cached_p96` | 18.92 ns | 18.90 ns - 18.94 ns | Repeats cached tangent near pi/2. |
| `computable_transcendentals/sin_huge_cold_p96` | 1.563 us | 1.559 us - 1.568 us | Approximates sine of a huge pi multiple plus offset. |
| `computable_transcendentals/cos_huge_cold_p96` | 1.469 us | 1.464 us - 1.474 us | Approximates cosine of a huge pi multiple plus offset. |
| `computable_transcendentals/tan_huge_cold_p96` | 6.078 us | 6.052 us - 6.106 us | Approximates tangent of a huge pi multiple plus offset. |
| `computable_transcendentals/asin_cold_p96` | 6.278 us | 6.246 us - 6.315 us | Approximates a computable asin expression. |
| `computable_transcendentals/asin_cached_p96` | 19.19 ns | 19.07 ns - 19.33 ns | Repeats a cached computable asin approximation. |
| `computable_transcendentals/acos_cold_p96` | 5.599 us | 5.577 us - 5.625 us | Approximates a computable acos expression. |
| `computable_transcendentals/acos_cached_p96` | 19.04 ns | 18.99 ns - 19.10 ns | Repeats a cached computable acos approximation. |
| `computable_transcendentals/asin_tiny_cold_p96` | 397.70 ns | 396.05 ns - 399.35 ns | Approximates asin(1e-12), exercising the tiny-input series. |
| `computable_transcendentals/acos_tiny_cold_p96` | 719.31 ns | 715.76 ns - 723.00 ns | Approximates acos(1e-12), exercising the tiny-input complement. |
| `computable_transcendentals/asin_near_one_cold_p96` | 1.899 us | 1.891 us - 1.907 us | Approximates asin(0.999999), exercising the endpoint complement. |
| `computable_transcendentals/acos_near_one_cold_p96` | 1.514 us | 1.513 us - 1.516 us | Approximates acos(0.999999), exercising the endpoint transform. |
| `computable_transcendentals/atan_cold_p96` | 1.899 us | 1.895 us - 1.903 us | Approximates atan(7/10). |
| `computable_transcendentals/atan_cached_p96` | 19.09 ns | 19.02 ns - 19.16 ns | Repeats a cached atan(7/10) approximation. |
| `computable_transcendentals/atan_large_cold_p96` | 1.642 us | 1.635 us - 1.650 us | Approximates atan(8), exercising argument reduction. |
| `computable_transcendentals/asin_zero_cold_p96` | 21.37 ns | 21.12 ns - 21.59 ns | Approximates asin(0) expression. |
| `computable_transcendentals/atan_zero_cold_p96` | 21.68 ns | 21.37 ns - 21.97 ns | Approximates atan(0). |
| `computable_transcendentals/asinh_cold_p128` | 9.524 us | 9.470 us - 9.590 us | Approximates a computable asinh expression. |
| `computable_transcendentals/asinh_three_quarters_cold_p128` | 5.041 us | 5.033 us - 5.050 us | Approximates asinh(3/4) across the series/ln1p crossover. |
| `computable_transcendentals/asinh_cached_p128` | 18.90 ns | 18.87 ns - 18.93 ns | Repeats a cached computable asinh approximation. |
| `computable_transcendentals/acosh_cold_p128` | 37.35 ns | 37.11 ns - 37.57 ns | Approximates a computable acosh expression. |
| `computable_transcendentals/acosh_cached_p128` | 19.46 ns | 19.42 ns - 19.50 ns | Repeats a cached computable acosh approximation. |
| `computable_transcendentals/atanh_cold_p128` | 147.44 ns | 146.73 ns - 148.29 ns | Approximates a computable atanh expression. |
| `computable_transcendentals/atanh_cached_p128` | 18.87 ns | 18.85 ns - 18.89 ns | Repeats a cached computable atanh approximation. |
| `computable_transcendentals/atanh_tiny_cold_p128` | 484.02 ns | 482.70 ns - 485.41 ns | Approximates atanh(1e-12), exercising the tiny-input series. |
| `computable_transcendentals/atanh_near_one_cold_p128` | 2.190 us | 2.181 us - 2.200 us | Approximates atanh(0.999999), exercising the endpoint log transform. |
| `computable_transcendentals/asinh_zero_cold_p128` | 21.75 ns | 21.51 ns - 21.97 ns | Approximates asinh(0) expression. |
| `computable_transcendentals/atanh_zero_cold_p128` | 21.16 ns | 20.93 ns - 21.37 ns | Approximates atanh(0) expression. |
| `computable_transcendentals/deep_add_chain_cold_p128` | 42.77 ns | 42.70 ns - 42.85 ns | Approximates a 5000-node addition chain. |
| `computable_transcendentals/deep_multiply_chain_cold_p128` | 42.90 ns | 42.73 ns - 43.10 ns | Approximates a 5000-node multiply-by-one chain. |
| `computable_transcendentals/deep_multiply_identity_chain_cold_p128` | 66.44 ns | 66.26 ns - 66.66 ns | Approximates a deep identity multiplication chain around pi. |
| `computable_transcendentals/deep_scaled_product_chain_cold_p128` | 28.19 ns | 28.16 ns - 28.21 ns | Approximates a deep product of exact scale factors. |
| `computable_transcendentals/perturbed_scaled_product_chain_cold_p128` | 28.70 ns | 28.43 ns - 29.00 ns | Approximates a deep scaled product with a tiny perturbation. |
| `computable_transcendentals/scaled_square_chain_cold_p128` | 28.48 ns | 28.42 ns - 28.55 ns | Approximates repeated squaring of a scaled irrational. |
| `computable_transcendentals/asymmetric_product_bad_order_cold_p128` | 28.57 ns | 28.54 ns - 28.62 ns | Approximates an asymmetric product order stress case. |
| `computable_transcendentals/sqrt_scaled_square_chain_cold_p128` | 437.02 ns | 433.41 ns - 440.98 ns | Approximates sqrt of a scaled-square chain. |
| `computable_transcendentals/warmed_zero_product_cold_p128` | 15.57 ns | 15.50 ns - 15.67 ns | Approximates a product involving a warmed zero sum. |
| `computable_transcendentals/inverse_scaled_product_chain_cold_p128` | 28.41 ns | 28.38 ns - 28.44 ns | Approximates the inverse of a deep scaled product. |
| `computable_transcendentals/deep_inverse_pair_chain_cold_p128` | 66.14 ns | 66.01 ns - 66.32 ns | Approximates a chain of inverse(inverse(x)) pairs. |
| `computable_transcendentals/deep_negated_square_chain_cold_p128` | 66.33 ns | 66.20 ns - 66.48 ns | Approximates repeated negate-square-sqrt transformations. |
| `computable_transcendentals/deep_negative_one_product_chain_cold_p128` | 67.48 ns | 67.12 ns - 67.90 ns | Approximates repeated multiplication by -1. |
| `computable_transcendentals/deep_half_product_chain_cold_p128` | 17.64 ns | 17.56 ns - 17.74 ns | Approximates repeated multiplication by 1/2. |
| `computable_transcendentals/deep_half_square_chain_cold_p128` | 28.39 ns | 28.33 ns - 28.48 ns | Approximates repeated squaring after scaling by 1/2. |
| `computable_transcendentals/deep_sqrt_square_chain_cold_p128` | 42.74 ns | 42.58 ns - 42.98 ns | Approximates repeated sqrt-square simplification. |
| `computable_transcendentals/inverse_half_product_chain_cold_p128` | 29.22 ns | 29.03 ns - 29.44 ns | Approximates the inverse of a deep half-product chain. |

<!-- END numerical_micro -->

<!-- BEGIN scalar_micro -->
## `scalar_micro`

Microbenchmarks for scalar operations, structural queries, cache hits, and dense exact arithmetic.

### `construction_speed`

Cost of constructing common exact scalar identities and small integers.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `construction_speed/rational_one` | 3.09 ns | 3.08 ns - 3.10 ns | Constructs `Rational::one()`. |
| `construction_speed/rational_new_one` | 3.27 ns | 3.26 ns - 3.28 ns | Constructs one through `Rational::new(1)`. |
| `construction_speed/rational_from_u8_four` | 3.78 ns | 3.77 ns - 3.80 ns | Constructs positive four through unsigned primitive conversion. |
| `construction_speed/rational_from_i8_minus_four` | 3.96 ns | 3.95 ns - 3.96 ns | Constructs negative four through signed primitive conversion. |
| `construction_speed/computable_one` | 16.91 ns | 16.84 ns - 16.99 ns | Constructs `Computable::one()`. |
| `construction_speed/real_new_rational_one` | 9.52 ns | 9.50 ns - 9.54 ns | Constructs one through `Real::new(Rational::one())`. |
| `construction_speed/real_one` | 9.76 ns | 9.74 ns - 9.78 ns | Constructs one through `Real::one()`. |
| `construction_speed/real_from_i32_one` | 9.53 ns | 9.49 ns - 9.58 ns | Constructs one through integer conversion. |
| `construction_speed/real_from_u8_four` | 10.33 ns | 10.31 ns - 10.35 ns | Constructs positive four as an exact `Real` from `u8`. |
| `construction_speed/real_from_i8_minus_four` | 10.57 ns | 10.51 ns - 10.65 ns | Constructs negative four as an exact `Real` from `i8`. |

### `raw_cache_hit_cost`

Cost of cold and cached `Computable::approx` calls for simple values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `raw_cache_hit_cost/zero` | 9.38 ns | 9.36 ns - 9.41 ns | Cached approximation request for exact zero. |
| `raw_cache_hit_cost/one` | 32.26 ns | 32.17 ns - 32.37 ns | Cached approximation request for exact one. |
| `raw_cache_hit_cost/two` | 32.27 ns | 32.22 ns - 32.34 ns | Cached approximation request for exact two. |
| `raw_cache_hit_cost/e` | 58.89 ns | 58.73 ns - 59.10 ns | Cached approximation request for Euler's constant. |
| `raw_cache_hit_cost/pi` | 57.71 ns | 57.64 ns - 57.82 ns | Cached approximation request for pi. |
| `raw_cache_hit_cost/tau` | 57.42 ns | 57.37 ns - 57.49 ns | Cached approximation request for two pi. |

### `structural_query_speed`

Speed of public structural queries across exact, transcendental, and composite `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `structural_query_speed/zero_zero_status` | 0.82 ns | 0.81 ns - 0.82 ns | Checks zero/nonzero facts for exact zero. |
| `structural_query_speed/zero_sign_query` | 4.74 ns | 4.73 ns - 4.76 ns | Reads sign facts for exact zero. |
| `structural_query_speed/zero_msd_query` | 13.53 ns | 13.51 ns - 13.56 ns | Reads magnitude facts for exact zero. |
| `structural_query_speed/zero_structural_facts` | 8.04 ns | 8.03 ns - 8.05 ns | Computes full structural facts for exact zero. |
| `structural_query_speed/one_zero_status` | 1.08 ns | 1.08 ns - 1.08 ns | Checks zero/nonzero facts for exact one. |
| `structural_query_speed/one_sign_query` | 11.55 ns | 11.53 ns - 11.57 ns | Reads sign facts for exact one. |
| `structural_query_speed/one_msd_query` | 16.25 ns | 16.23 ns - 16.29 ns | Reads magnitude facts for exact one. |
| `structural_query_speed/one_structural_facts` | 11.90 ns | 11.82 ns - 12.02 ns | Computes full structural facts for exact one. |
| `structural_query_speed/negative_zero_status` | 1.08 ns | 1.08 ns - 1.08 ns | Checks zero/nonzero facts for an exact negative integer. |
| `structural_query_speed/negative_sign_query` | 13.32 ns | 13.24 ns - 13.42 ns | Reads sign facts for an exact negative integer. |
| `structural_query_speed/negative_msd_query` | 17.27 ns | 17.23 ns - 17.31 ns | Reads magnitude facts for an exact negative integer. |
| `structural_query_speed/negative_structural_facts` | 13.19 ns | 13.17 ns - 13.23 ns | Computes full structural facts for an exact negative integer. |
| `structural_query_speed/tiny_exact_zero_status` | 1.08 ns | 1.08 ns - 1.09 ns | Checks zero/nonzero facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_sign_query` | 17.00 ns | 16.97 ns - 17.04 ns | Reads sign facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_msd_query` | 19.73 ns | 19.65 ns - 19.81 ns | Reads magnitude facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_structural_facts` | 17.17 ns | 17.10 ns - 17.24 ns | Computes full structural facts for a tiny exact rational. |
| `structural_query_speed/pi_zero_status` | 1.08 ns | 1.08 ns - 1.08 ns | Checks zero/nonzero facts for pi. |
| `structural_query_speed/pi_sign_query` | 17.41 ns | 17.36 ns - 17.47 ns | Reads sign facts for pi. |
| `structural_query_speed/pi_msd_query` | 20.40 ns | 20.33 ns - 20.49 ns | Reads magnitude facts for pi. |
| `structural_query_speed/pi_structural_facts` | 17.33 ns | 17.29 ns - 17.38 ns | Computes full structural facts for pi. |
| `structural_query_speed/e_zero_status` | 1.08 ns | 1.08 ns - 1.09 ns | Checks zero/nonzero facts for e. |
| `structural_query_speed/e_sign_query` | 17.66 ns | 17.54 ns - 17.80 ns | Reads sign facts for e. |
| `structural_query_speed/e_msd_query` | 20.35 ns | 20.29 ns - 20.43 ns | Reads magnitude facts for e. |
| `structural_query_speed/e_structural_facts` | 17.30 ns | 17.24 ns - 17.38 ns | Computes full structural facts for e. |
| `structural_query_speed/tau_zero_status` | 1.09 ns | 1.09 ns - 1.09 ns | Checks zero/nonzero facts for tau. |
| `structural_query_speed/tau_sign_query` | 21.99 ns | 21.87 ns - 22.14 ns | Reads sign facts for tau. |
| `structural_query_speed/tau_msd_query` | 26.09 ns | 26.06 ns - 26.13 ns | Reads magnitude facts for tau. |
| `structural_query_speed/tau_structural_facts` | 21.79 ns | 21.73 ns - 21.85 ns | Computes full structural facts for tau. |
| `structural_query_speed/sqrt_two_zero_status` | 1.09 ns | 1.08 ns - 1.09 ns | Checks zero/nonzero facts for sqrt(2). |
| `structural_query_speed/sqrt_two_sign_query` | 17.41 ns | 17.35 ns - 17.48 ns | Reads sign facts for sqrt(2). |
| `structural_query_speed/sqrt_two_msd_query` | 20.36 ns | 20.28 ns - 20.47 ns | Reads magnitude facts for sqrt(2). |
| `structural_query_speed/sqrt_two_structural_facts` | 17.60 ns | 17.49 ns - 17.73 ns | Computes full structural facts for sqrt(2). |
| `structural_query_speed/pi_minus_three_zero_status` | 1.08 ns | 1.08 ns - 1.08 ns | Checks zero/nonzero facts for pi - 3. |
| `structural_query_speed/pi_minus_three_sign_query` | 17.48 ns | 17.41 ns - 17.55 ns | Reads sign facts for pi - 3. |
| `structural_query_speed/pi_minus_three_msd_query` | 20.47 ns | 20.40 ns - 20.56 ns | Reads magnitude facts for pi - 3. |
| `structural_query_speed/pi_minus_three_structural_facts` | 17.26 ns | 17.24 ns - 17.27 ns | Computes full structural facts for pi - 3. |
| `structural_query_speed/dense_expr_zero_status` | 3.54 ns | 3.53 ns - 3.54 ns | Checks zero/nonzero facts for a dense composite expression. |
| `structural_query_speed/dense_expr_sign_query` | 8.06 ns | 8.04 ns - 8.09 ns | Reads sign facts for a dense composite expression. |
| `structural_query_speed/dense_expr_msd_query` | 15.60 ns | 15.54 ns - 15.67 ns | Reads magnitude facts for a dense composite expression. |
| `structural_query_speed/dense_expr_structural_facts` | 10.83 ns | 10.80 ns - 10.86 ns | Computes full structural facts for a dense composite expression. |
| `structural_query_speed/structural_negation_match` | 13.08 ns | 13.04 ns - 13.12 ns | Certifies opposite rational scales over one shared exact symbolic basis. |
| `structural_query_speed/structural_negation_miss` | 3.78 ns | 3.78 ns - 3.79 ns | Rejects an unrelated exact symbolic basis without constructing a negation. |

### `pure_scalar_algorithm_speed`

Core scalar algorithms that do not require high-precision transcendental approximation.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `pure_scalar_algorithm_speed/rational_add` | 8.76 ns | 8.73 ns - 8.79 ns | Adds two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_sub` | 9.44 ns | 9.35 ns - 9.53 ns | Subtracts two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_add_wide_dyadic_cold` | 92.70 ns | 91.60 ns - 93.72 ns | Adds fresh integer and wide-dyadic operands without retained work. |
| `pure_scalar_algorithm_speed/rational_sub_wide_dyadic_cold` | 98.71 ns | 94.79 ns - 105.54 ns | Subtracts fresh integer and wide-dyadic operands without retained work. |
| `pure_scalar_algorithm_speed/rational_add_shared_cold` | 98.39 ns | 97.66 ns - 99.03 ns | Adds fresh operands whose storage is cloned but whose arithmetic pair is not yet observed. |
| `pure_scalar_algorithm_speed/rational_sub_shared_cold` | 97.91 ns | 97.07 ns - 98.70 ns | Subtracts fresh operands whose storage is cloned but whose arithmetic pair is not yet observed. |
| `pure_scalar_algorithm_speed/rational_scaled_difference_composed_cold` | 264.51 ns | 263.12 ns - 265.93 ns | Computes a fresh wide-integer scaled difference through multiply then subtract. |
| `pure_scalar_algorithm_speed/rational_scaled_difference_fused_cold` | 95.31 ns | 94.53 ns - 96.05 ns | Computes the same fresh wide-integer scaled difference with the fused integer kernel. |
| `pure_scalar_algorithm_speed/rational_cross_difference_unit_divisor_composed_cold` | 593.91 ns | 592.58 ns - 595.37 ns | Computes a fresh wide-integer cross difference and divides it by negative one through general operations. |
| `pure_scalar_algorithm_speed/rational_cross_difference_unit_divisor_fused_cold` | 193.14 ns | 186.03 ns - 206.13 ns | Computes the same cross difference through the checked fused unit-divisor path. |
| `pure_scalar_algorithm_speed/rational_mul` | 22.51 ns | 22.43 ns - 22.61 ns | Multiplies two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_mul_retained_general` | 11.67 ns | 11.61 ns - 11.74 ns | Reuses one retained exact product for an immutable rational operand pair. |
| `pure_scalar_algorithm_speed/rational_mul_wide_dyadic_cold` | 191.69 ns | 188.68 ns - 194.54 ns | Multiplies fresh wide-denominator dyadics whose numerators fit `u128`. |
| `pure_scalar_algorithm_speed/rational_mul_dyadic_general_cross_cancel` | 11.77 ns | 11.67 ns - 11.90 ns | Multiplies a wide dyadic rational by a general rational with a power-of-two numerator. |
| `pure_scalar_algorithm_speed/rational_div` | 162.70 ns | 162.23 ns - 163.28 ns | Divides two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_inverse_owned_cold` | 20.86 ns | 20.82 ns - 20.91 ns | Inverts a fresh uniquely owned nontrivial rational. |
| `pure_scalar_algorithm_speed/rational_inverse_retained` | 7.63 ns | 7.58 ns - 7.68 ns | Reuses the retained reciprocal of a shared nontrivial rational. |
| `pure_scalar_algorithm_speed/rational_neg_owned_cold` | 9.35 ns | 9.29 ns - 9.42 ns | Negates a fresh uniquely owned nontrivial rational in place. |
| `pure_scalar_algorithm_speed/rational_neg_retained` | 7.81 ns | 7.78 ns - 7.83 ns | Reuses the retained opposite sign of a shared nontrivial rational. |
| `pure_scalar_algorithm_speed/real_exact_powi_i64_owned_cold` | 255.83 ns | 254.32 ns - 257.69 ns | Raises a fresh uniquely owned exact rational Real to the fifth power. |
| `pure_scalar_algorithm_speed/real_exact_powi_i64_retained` | 58.43 ns | 58.03 ns - 58.90 ns | Reuses the bounded exact product chain for a shared fifth power. |
| `pure_scalar_algorithm_speed/real_exact_add` | 17.98 ns | 17.91 ns - 18.07 ns | Adds exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_average_pair` | 139.52 ns | 139.09 ns - 140.01 ns | Averages exact rational-backed `Real` values through the fused pair kernel. |
| `pure_scalar_algorithm_speed/real_exact_average_pair_expanded` | 223.63 ns | 223.06 ns - 224.23 ns | Averages exact rational-backed `Real` values through separate add and divide operations. |
| `pure_scalar_algorithm_speed/real_exact_sub` | 18.37 ns | 18.25 ns - 18.51 ns | Subtracts exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_mul` | 31.60 ns | 31.53 ns - 31.68 ns | Multiplies exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_mul_retained` | 20.66 ns | 20.62 ns - 20.70 ns | Reuses the retained exact product beneath rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_div` | 182.22 ns | 182.06 ns - 182.43 ns | Divides exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_sqrt_owned_cold` | 220.08 ns | 218.71 ns - 221.73 ns | Reduces a fresh uniquely owned exact square-root expression. |
| `pure_scalar_algorithm_speed/real_exact_sqrt_reduce` | 100.37 ns | 99.87 ns - 100.94 ns | Reuses the retained reduction of an exact square-root expression. |
| `pure_scalar_algorithm_speed/real_exact_dyadic_sqrt_reduce` | 95.35 ns | 94.92 ns - 95.83 ns | Reuses the square-root reduction of a large exact dyadic rational. |
| `pure_scalar_algorithm_speed/real_exact_general_sqrt_reduce` | 94.01 ns | 93.77 ns - 94.28 ns | Reuses the square-root reduction of a non-dyadic rational sum of squares. |
| `pure_scalar_algorithm_speed/real_exact_dyadic_radical_scale` | 36.33 ns | 36.24 ns - 36.43 ns | Scales an exact reciprocal radical by one exact binary64-derived dyadic coordinate. |
| `pure_scalar_algorithm_speed/real_exact_ln_reduce` | 81.77 ns | 81.15 ns - 82.53 ns | Reduces an exact logarithm of a power of two. |
| `pure_scalar_algorithm_speed/real_pow_small_integer_exponent` | 130.64 ns | 129.92 ns - 131.52 ns | Dispatches `Real::pow` with an exact small-integer exponent. |

### `rational_algorithm_dispatch_speed`

Cold backend algorithm families and retained rational fact dispatch selected from GMP-style operand shapes.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `rational_algorithm_dispatch_speed/dyadic_fact_cold` | 38.15 ns | 37.06 ns - 39.13 ns | Classifies a fresh non-dyadic denominator and retains the result. |
| `rational_algorithm_dispatch_speed/dyadic_fact_retained` | 1.95 ns | 1.92 ns - 1.98 ns | Reads an already-retained non-dyadic denominator classification. |
| `rational_algorithm_dispatch_speed/compare_leading_significand_retained_1024_bits` | 46.18 ns | 45.92 ns - 46.48 ns | Compares retained wide rational magnitudes through the certified leading-significand interval. |
| `rational_algorithm_dispatch_speed/compare_dyadic_shifted_retained_1024_bits` | 10.92 ns | 10.89 ns - 10.96 ns | Compares retained wide dyadics with equal scaled width and a five-bit denominator-shift difference. |
| `rational_algorithm_dispatch_speed/equality_leading_significand_retained_1024_bits` | 173.91 ns | 173.39 ns - 174.49 ns | Compares unequal retained wide fractions with separated leading significands. |
| `rational_algorithm_dispatch_speed/equality_dyadic_shifted_retained_1024_bits` | 10.93 ns | 10.91 ns - 10.96 ns | Rejects unequal retained wide dyadics without cross products. |
| `rational_algorithm_dispatch_speed/equality_shared_identity_retained` | 3.64 ns | 3.62 ns - 3.65 ns | Compares two references to one retained rational allocation. |
| `rational_algorithm_dispatch_speed/equality_same_denominator_retained` | 6.48 ns | 6.45 ns - 6.51 ns | Rejects unequal small numerators with a common denominator. |
| `rational_algorithm_dispatch_speed/equality_word_retained` | 10.50 ns | 10.42 ns - 10.59 ns | Rejects unequal small fractions with different non-dyadic denominators. |
| `rational_algorithm_dispatch_speed/equality_equal_distinct_retained` | 7.31 ns | 7.27 ns - 7.35 ns | Recognizes equal small fractions held in distinct allocations. |
| `rational_algorithm_dispatch_speed/equality_different_signs_retained` | 2.39 ns | 2.38 ns - 2.40 ns | Rejects wide fractions with opposite signs before magnitude comparison. |
| `rational_algorithm_dispatch_speed/equality_close_retained_1024_bits` | 182.84 ns | 182.34 ns - 183.42 ns | Rejects near-equal wide fractions through the exact cross-product fallback. |
| `rational_algorithm_dispatch_speed/mul_backend_basecase_cold` | 361.29 ns | 257.72 ns - 566.34 ns | Multiplies fresh balanced 16-limb integers through the backend basecase kernel. |
| `rational_algorithm_dispatch_speed/mul_backend_half_karatsuba_cold` | 625.75 ns | 499.12 ns - 876.30 ns | Multiplies fresh unbalanced 33-by-66-limb integers through half-Karatsuba. |
| `rational_algorithm_dispatch_speed/mul_backend_karatsuba_cold` | 816.38 ns | 813.13 ns - 819.81 ns | Multiplies fresh balanced 40-limb integers through Karatsuba. |
| `rational_algorithm_dispatch_speed/mul_backend_toom3_cold` | 8.407 us | 8.373 us - 8.442 us | Multiplies fresh balanced 257-limb integers through Toom-3. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_4096_bits` | 8.266 us | 8.203 us - 8.338 us | Runs Hyperreal's seven-product Rust-native Toom-4 candidate on balanced 4,096-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_4096_bits` | 2.801 us | 2.789 us - 2.816 us | Runs the native backend product on the same 4,096-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_16384_bits` | 34.689 us | 34.635 us - 34.757 us | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 16,384-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_16384_bits` | 26.388 us | 26.308 us - 26.482 us | Runs the native backend product on the same 16,384-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_65536_bits` | 225.446 us | 225.070 us - 225.991 us | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 65,536-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_65536_bits` | 203.027 us | 202.650 us - 203.449 us | Runs the native backend product on the same 65,536-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_262144_bits` | 1.646 ms | 1.639 ms - 1.654 ms | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 262,144-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_262144_bits` | 1.641 ms | 1.636 ms - 1.647 ms | Runs the native backend product on the same 262,144-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_524288_bits` | 4.599 ms | 4.585 ms - 4.615 ms | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 524,288-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_524288_bits` | 4.485 ms | 4.472 ms - 4.501 ms | Runs the native backend product on the same 524,288-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_1048576_bits` | 12.216 ms | 12.191 ms - 12.250 ms | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 1,048,576-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_1048576_bits` | 12.877 ms | 12.816 ms - 12.947 ms | Runs the native backend product on the same 1,048,576-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_2097152_bits` | 32.910 ms | 32.844 ms - 32.983 ms | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 2,097,152-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_2097152_bits` | 35.604 ms | 35.531 ms - 35.684 ms | Runs the native backend product on the same 2,097,152-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_1048576_bits` | 10.775 ms | 10.746 ms - 10.807 ms | Runs the retained production Toom-8 selector above its balanced crossover. |
| `rational_algorithm_dispatch_speed/mul_selected_2097152_bits` | 28.229 ms | 28.116 ms - 28.363 ms | Runs the retained production Toom-8 selector on balanced 2,097,152-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_1048576_bits` | 10.924 ms | 10.890 ms - 10.967 ms | Runs Hyperreal's eleven-product Rust-native Toom-6 candidate above its crossover. |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_131072_bits` | 595.264 us | 594.525 us - 596.063 us | Runs Hyperreal's Rust-native Toom-6 candidate on balanced 131,072-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_131072_bits` | 593.866 us | 591.529 us - 596.592 us | Runs the retained native backend selector on the same 131,072-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_262144_bits` | 1.586 ms | 1.581 ms - 1.592 ms | Runs Hyperreal's Rust-native Toom-6 candidate on balanced 262,144-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_524288_bits` | 4.143 ms | 4.132 ms - 4.156 ms | Runs Hyperreal's Rust-native Toom-6 candidate on balanced 524,288-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_524288_bits` | 4.005 ms | 3.990 ms - 4.022 ms | Runs the retained production Toom-8 selector above its balanced crossover. |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_2097152_bits` | 30.664 ms | 30.508 ms - 30.833 ms | Runs Hyperreal's Rust-native Toom-6 candidate on balanced 2,097,152-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_toom4_unbalanced_1258291_by_1048576` | 14.997 ms | 14.952 ms - 15.050 ms | Runs retained Toom-4 on a 6:5 operand pair outside Toom-6's balance band. |
| `rational_algorithm_dispatch_speed/mul_backend_unbalanced_1258291_by_1048576` | 16.312 ms | 16.252 ms - 16.380 ms | Runs the native backend on the same 6:5 operand pair. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_262144_bits` | 1.521 ms | 1.517 ms - 1.526 ms | Runs Hyperreal's fifteen-product Rust-native Toom-8 candidate on balanced 262,144-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_262144_bits` | 1.524 ms | 1.520 ms - 1.528 ms | Runs the retained production Toom-8 selector at its balanced crossover. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_65536_bits` | 258.966 us | 257.922 us - 260.121 us | Runs Hyperreal's Rust-native Toom-8 candidate on balanced 65,536-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_131072_bits` | 603.414 us | 602.105 us - 604.917 us | Runs Hyperreal's Rust-native Toom-8 candidate on balanced 131,072-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_524288_bits` | 4.004 ms | 3.989 ms - 4.022 ms | Runs Hyperreal's Rust-native Toom-8 candidate at the Toom-6 crossover. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_1048576_bits` | 10.765 ms | 10.739 ms - 10.794 ms | Runs Hyperreal's Rust-native Toom-8 candidate on balanced 1,048,576-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_2097152_bits` | 28.297 ms | 28.199 ms - 28.407 ms | Runs Hyperreal's Rust-native Toom-8 candidate on balanced 2,097,152-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_4194304_bits` | 74.186 ms | 74.018 ms - 74.376 ms | Runs Hyperreal's Rust-native Toom-8 candidate on balanced 4,194,304-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_4194304_bits` | 73.811 ms | 73.707 ms - 73.933 ms | Runs the retained production Toom-8 selector on the same 4,194,304-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_toom6_unbalanced_599186_by_524288` | 4.814 ms | 4.797 ms - 4.835 ms | Runs retained Toom-6 on an 8:7 operand pair outside Toom-8's balance band. |
| `rational_algorithm_dispatch_speed/mul_backend_unbalanced_599186_by_524288` | 5.364 ms | 5.347 ms - 5.383 ms | Runs the native backend on the same 8:7 operand pair. |
| `rational_algorithm_dispatch_speed/mul_ntt_candidate_262144_bits` | 16.200 ms | 16.152 ms - 16.257 ms | Runs Hyperreal's exact two-prime Rust-native NTT/CRT candidate on balanced 262,144-bit operands. |
| `rational_algorithm_dispatch_speed/mul_ntt_candidate_1048576_bits` | 73.253 ms | 73.092 ms - 73.431 ms | Runs the Rust-native NTT/CRT candidate on balanced 1,048,576-bit operands. |
| `rational_algorithm_dispatch_speed/mul_ntt_candidate_4194304_bits` | 328.426 ms | 327.815 ms - 329.071 ms | Runs the Rust-native NTT/CRT candidate on balanced 4,194,304-bit operands. |
| `rational_algorithm_dispatch_speed/reduce_backend_single_limb_cold` | 136.07 ns | 135.69 ns - 136.56 ns | Reduces a fresh wide fraction by a single-limb exact divisor. |
| `rational_algorithm_dispatch_speed/reduce_backend_knuth_cold` | 755.92 ns | 752.29 ns - 759.89 ns | Reduces a fresh wide fraction through normalized Knuth basecase division. |
| `rational_algorithm_dispatch_speed/reduce_backend_large_knuth_cold` | 10.482 us | 10.424 us - 10.551 us | Reduces a fresh 129-limb numerator by a 65-limb exact divisor through normalized Knuth division. |
| `rational_algorithm_dispatch_speed/reduce_fixed_512_coprime_cold` | 2.812 us | 2.789 us - 2.838 us | Reduces fresh balanced 512-bit operands through the fixed-limb rational-operation GCD. |
| `rational_algorithm_dispatch_speed/exact_remainder_large_knuth` | 5.093 us | 5.074 us - 5.114 us | Computes a wide rational fractional remainder through the traced normalized Knuth backend. |
| `rational_algorithm_dispatch_speed/division_trivial_small_quotient` | 83.35 ns | 82.72 ns - 84.06 ns | Exercises the backend's zero-quotient magnitude division exit on wide operands. |
| `rational_algorithm_dispatch_speed/gcd_selected_128_bits` | 129.68 ns | 129.35 ns - 130.07 ns | Runs selected magnitude GCD on an ascending balanced two-limb pair. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_128_bits` | 5.589 us | 5.542 us - 5.640 us | Runs the full-width Euclidean baseline on the same 128-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_192_bits` | 5.431 us | 5.394 us - 5.474 us | Runs selected magnitude GCD at the retained three-limb Lehmer crossover. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_192_bits` | 8.774 us | 8.734 us - 8.819 us | Runs the full-width Euclidean baseline on the same 192-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_512_bits` | 11.044 us | 10.983 us - 11.110 us | Runs selected magnitude GCD above the Lehmer crossover. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_512_bits` | 32.526 us | 32.315 us - 32.786 us | Runs the full-width Euclidean baseline on the same 512-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_1024_bits` | 21.325 us | 21.192 us - 21.468 us | Runs selected magnitude GCD above the Lehmer crossover. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_1024_bits` | 75.667 us | 75.260 us - 76.115 us | Runs the full-width Euclidean baseline on the same 1,024-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_4096_bits` | 116.449 us | 115.071 us - 118.035 us | Runs selected magnitude GCD well above the Lehmer crossover. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_4096_bits` | 496.054 us | 493.546 us - 498.810 us | Runs the full-width Euclidean baseline on the same 4,096-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_192_bits` | 8.502 us | 8.459 us - 8.552 us | Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 192 bits. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_192_bits` | 8.526 us | 8.442 us - 8.626 us | Runs the full-width Euclidean baseline on the same initially unbalanced pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_256_bits` | 8.086 us | 8.028 us - 8.160 us | Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 256 bits. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_256_bits` | 13.725 us | 13.674 us - 13.782 us | Runs the full-width Euclidean baseline on the same initially unbalanced pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_512_bits` | 12.216 us | 12.128 us - 12.320 us | Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 512 bits. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_512_bits` | 30.706 us | 30.668 us - 30.745 us | Runs the full-width Euclidean baseline on the same initially unbalanced pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_1024_bits` | 23.438 us | 23.372 us - 23.520 us | Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 1,024 bits. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_1024_bits` | 73.718 us | 73.538 us - 73.949 us | Runs the full-width Euclidean baseline on the same initially unbalanced pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_4096_bits` | 122.571 us | 122.409 us - 122.742 us | Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 4,096 bits. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_4096_bits` | 510.853 us | 509.312 us - 513.168 us | Runs the full-width Euclidean baseline on the same initially unbalanced pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_8192_bits` | 296.557 us | 296.141 us - 297.014 us | Runs the recursive half-GCD candidate below its provisional crossover. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_8192_bits` | 303.258 us | 301.453 us - 305.247 us | Runs the quadratic Lehmer baseline on the same 8,192-bit pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_16384_bits` | 3.231 ms | 3.218 ms - 3.245 ms | Runs the recursive half-GCD candidate at its provisional crossover. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_16384_bits` | 877.287 us | 873.885 us - 880.939 us | Runs the quadratic Lehmer baseline on the same 16,384-bit pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_65536_bits` | 11.356 ms | 11.316 ms - 11.403 ms | Runs the recursive half-GCD candidate well above its provisional crossover. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_65536_bits` | 8.931 ms | 8.909 ms - 8.957 ms | Runs the quadratic Lehmer baseline on the same 65,536-bit pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_262144_bits` | 272.155 ms | 271.688 ms - 272.654 ms | Runs recursive half-GCD with selected higher-Toom matrix products at 262,144 bits. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_262144_bits` | 126.535 ms | 126.281 ms - 126.870 ms | Runs the Lehmer baseline on the same 262,144-bit pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_1048576_bits` | 3.829 s | 3.827 s - 3.832 s | Runs recursive half-GCD with selected higher-Toom matrix products at 1,048,576 bits. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_1048576_bits` | 1.980 s | 1.977 s - 1.982 s | Runs the Lehmer baseline on the same 1,048,576-bit pair. |
| `rational_algorithm_dispatch_speed/barrett_one_shot_8192_by_1024` | 5.755 us | 5.745 us - 5.765 us | Prepares a Rust-native Barrett reciprocal and divides one 8,192-bit value by a 1,024-bit divisor. |
| `rational_algorithm_dispatch_speed/backend_one_shot_8192_by_1024` | 2.794 us | 2.789 us - 2.800 us | Runs the native backend div-rem baseline for the same one-shot operands. |
| `rational_algorithm_dispatch_speed/barrett_batch16_8192_by_1024` | 85.577 us | 85.484 us - 85.680 us | Amortizes one Rust-native Barrett reciprocal over sixteen 8,192-bit dividends. |
| `rational_algorithm_dispatch_speed/backend_batch16_8192_by_1024` | 49.515 us | 49.418 us - 49.637 us | Runs sixteen native backend div-rem operations on the same values. |
| `rational_algorithm_dispatch_speed/barrett_batch16_65536_by_4096` | 1.470 ms | 1.468 ms - 1.473 ms | Amortizes one Rust-native Barrett reciprocal over sixteen 65,536-bit dividends. |
| `rational_algorithm_dispatch_speed/backend_batch16_65536_by_4096` | 1.214 ms | 1.213 ms - 1.215 ms | Runs sixteen native backend div-rem operations on the same large values. |
| `rational_algorithm_dispatch_speed/perfect_power_factor_reject` | 70.91 ns | 70.81 ns - 71.05 ns | Rejects 12 after small-factor multiplicities collapse to gcd one. |
| `rational_algorithm_dispatch_speed/perfect_power_general_seventh` | 1.627 us | 1.626 us - 1.628 us | Discovers an exact rational seventh power whose base primes exceed the trial table. |
| `rational_algorithm_dispatch_speed/perfect_power_fixed_seventh` | 208.53 ns | 207.88 ns - 209.30 ns | Checks the same value when the seventh-root degree is already known. |
| `rational_algorithm_dispatch_speed/perfect_power_unfactored_reject` | 3.273 us | 3.264 us - 3.283 us | Rejects mismatched seventh- and fifth-power rational components beyond the trial table. |
| `rational_algorithm_dispatch_speed/radix_format_small_integer` | 948.97 ns | 941.01 ns - 958.28 ns | Formats a 16-limb integer using repeated single-limb radix division. |
| `rational_algorithm_dispatch_speed/radix_format_large_integer` | 2.996 us | 2.989 us - 3.004 us | Formats a 32-limb integer using divide-and-conquer radix conversion. |
| `rational_algorithm_dispatch_speed/radix_parse_short_decimal` | 81.78 ns | 81.55 ns - 82.05 ns | Parses a short exact decimal through the checked word-sized path. |
| `rational_algorithm_dispatch_speed/radix_parse_short_scientific` | 75.44 ns | 75.35 ns - 75.54 ns | Parses a representative file-I/O scientific literal through the checked word-sized path. |
| `rational_algorithm_dispatch_speed/radix_parse_wide_scientific` | 42.858 us | 42.796 us - 42.966 us | Parses a 5,120-digit significand with a negative decimal exponent exactly. |
| `rational_algorithm_dispatch_speed/radix_parse_wide_scientific_expanded` | 39.483 us | 39.455 us - 39.514 us | Parses the same exact wide value after expanding its decimal point as a baseline. |
| `rational_algorithm_dispatch_speed/radix_parse_large_integer` | 1.845 us | 1.843 us - 1.847 us | Parses a large below-threshold decimal fixture through chunked multiply-add conversion. |
| `rational_algorithm_dispatch_speed/radix_parse_divide_conquer_10240_digits` | 105.766 us | 105.652 us - 105.933 us | Parses 10,240 digits through the divide-and-conquer product tree. |
| `rational_algorithm_dispatch_speed/radix_parse_backend_chunked_10240_digits` | 105.253 us | 104.992 us - 105.593 us | Parses the same 10,240 digits with the backend chunked multiply-add baseline. |
| `rational_algorithm_dispatch_speed/radix_parse_divide_conquer_20480_digits` | 297.513 us | 296.739 us - 298.467 us | Parses 20,480 digits through the divide-and-conquer product tree. |
| `rational_algorithm_dispatch_speed/radix_parse_backend_chunked_20480_digits` | 380.912 us | 380.588 us - 381.284 us | Parses the same 20,480 digits with the backend chunked multiply-add baseline. |
| `rational_algorithm_dispatch_speed/radix_format_fraction_decimal` | 2.668 us | 2.665 us - 2.672 us | Formats a rational decimal through exact repeated digit division. |

### `borrowed_op_overhead`

Borrowed versus owned operation overhead for rational and real operands.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `borrowed_op_overhead/rational_clone_pair` | 7.35 ns | 7.34 ns - 7.36 ns | Clones two rational values. |
| `borrowed_op_overhead/rational_add_refs` | 8.78 ns | 8.75 ns - 8.81 ns | Adds rational references. |
| `borrowed_op_overhead/rational_add_owned` | 10.25 ns | 10.22 ns - 10.27 ns | Adds owned rational values. |
| `borrowed_op_overhead/real_clone_pair` | 37.29 ns | 37.23 ns - 37.38 ns | Clones two scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_unscaled_add_refs` | 90.20 ns | 90.07 ns - 90.36 ns | Adds borrowed unscaled transcendental `Real` values. |
| `borrowed_op_overhead/real_unscaled_add_owned` | 89.88 ns | 89.54 ns - 90.37 ns | Adds owned unscaled transcendental `Real` values. |
| `borrowed_op_overhead/real_add_refs` | 256.14 ns | 255.86 ns - 256.45 ns | Adds borrowed scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_add_owned` | 272.38 ns | 270.94 ns - 274.34 ns | Adds owned scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_dot2_refs_dense_symbolic` | 484.89 ns | 484.56 ns - 485.22 ns | Computes a borrowed two-lane symbolic dot product with no rational shortcut terms. |
| `borrowed_op_overhead/real_active_dot2_refs_dense_symbolic` | 494.39 ns | 492.74 ns - 496.52 ns | Computes a borrowed two-lane symbolic dot product after the caller has already classified every lane active. |
| `borrowed_op_overhead/real_dot2_refs_mixed_structural` | 26.98 ns | 26.96 ns - 27.00 ns | Computes a borrowed two-lane symbolic dot product with an exact zero lane and a rational scale lane. |
| `borrowed_op_overhead/real_dot3_refs_dense_symbolic` | 1.018 us | 1.013 us - 1.023 us | Computes a borrowed three-lane symbolic dot product with no rational shortcut terms. |
| `borrowed_op_overhead/real_active_dot3_refs_dense_symbolic` | 1.023 us | 1.023 us - 1.024 us | Computes a borrowed three-lane symbolic dot product after the caller has already classified every lane active. |
| `borrowed_op_overhead/real_dot3_refs_mixed_structural` | 203.49 ns | 203.10 ns - 204.03 ns | Computes a borrowed three-lane symbolic dot product with exact zero and rational scale terms. |
| `borrowed_op_overhead/real_dot4_refs_dense_symbolic` | 1.555 us | 1.553 us - 1.557 us | Computes a borrowed four-lane symbolic dot product with no rational shortcut terms. |
| `borrowed_op_overhead/real_active_dot4_refs_dense_symbolic` | 1.579 us | 1.568 us - 1.595 us | Computes a borrowed four-lane symbolic dot product after the caller has already classified every lane active. |
| `borrowed_op_overhead/real_dot4_refs_mixed_structural` | 242.50 ns | 241.92 ns - 243.17 ns | Computes a borrowed four-lane symbolic dot product with exact zero and rational scale terms. |

### `dense_algebra`

Small dense algebra kernels that stress repeated exact and symbolic operations.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `dense_algebra/rational_dot_64` | 1.689 us | 1.685 us - 1.693 us | Computes a 64-element rational dot product. |
| `dense_algebra/rational_matmul_8` | 54.981 us | 54.892 us - 55.085 us | Computes an 8x8 rational matrix multiply. |
| `dense_algebra/real_dot_36` | 4.416 us | 4.413 us - 4.418 us | Computes a 36-element dot product over symbolic `Real` values. |
| `dense_algebra/real_matmul_6` | 45.701 us | 45.663 us - 45.739 us | Computes a 6x6 matrix multiply over symbolic `Real` values. |
| `dense_algebra/real_sum_refs_64_symbolic` | 6.686 us | 6.674 us - 6.700 us | Constructs an arbitrary-length sum of 64 borrowed symbolic square roots. |
| `dense_algebra/real_sum_refs_64_symbolic_to_f64` | 32.265 us | 32.239 us - 32.292 us | Constructs and approximates the same arbitrary-length symbolic sum. |
| `dense_algebra/real_sum_refs_1024_symbolic_sequential` | 114.986 us | 114.735 us - 115.268 us | Constructs a 1,024-term symbolic sum with the former sequential fold. |
| `dense_algebra/real_sum_refs_1024_symbolic` | 151.653 us | 151.462 us - 151.875 us | Constructs the same 1,024-term symbolic sum through the balanced public aggregate. |
| `dense_algebra/real_sum_refs_1024_symbolic_to_f64_sequential` | 5.144 ms | 5.137 ms - 5.153 ms | Sequentially constructs and approximates a 1,024-term symbolic sum with cold child caches. |
| `dense_algebra/real_sum_refs_1024_symbolic_to_f64` | 1.224 ms | 1.220 ms - 1.228 ms | Constructs and approximates the same cold 1,024-term symbolic sum through the balanced public aggregate. |
| `dense_algebra/real_sum_refs_1024_rational_sequential` | 21.458 us | 21.370 us - 21.545 us | Constructs a 1,024-term exact rational sum with the former sequential fold. |
| `dense_algebra/real_sum_refs_1024_rational` | 35.742 us | 35.714 us - 35.775 us | Constructs the same exact rational sum through the homogeneous-prefix public aggregate. |
| `dense_algebra/real_sum_owned_1024_symbolic_former_clone_path` | 151.000 us | 150.422 us - 151.727 us | Consumes a 1,024-term symbolic sum while reproducing the former extra per-value clone. |
| `dense_algebra/real_sum_owned_1024_symbolic` | 122.156 us | 121.930 us - 122.398 us | Consumes the same symbolic sum by moving owned values into the balanced reducer. |

### `exact_transcendental_special_forms`

Construction-time shortcuts for exact rational multiples of pi and inverse compositions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `exact_transcendental_special_forms/sin_pi_7` | 782.38 ns | 780.01 ns - 784.94 ns | Builds the exact special form for sin(pi/7). |
| `exact_transcendental_special_forms/cos_pi_7` | 647.14 ns | 643.94 ns - 651.10 ns | Builds the exact special form for cos(pi/7). |
| `exact_transcendental_special_forms/tan_pi_7` | 235.23 ns | 233.97 ns - 236.76 ns | Builds the exact special form for tan(pi/7). |
| `exact_transcendental_special_forms/asin_sin_6pi_7` | 905.78 ns | 904.25 ns - 907.31 ns | Recognizes the principal branch of asin(sin(6pi/7)). |
| `exact_transcendental_special_forms/acos_cos_9pi_7` | 889.12 ns | 886.57 ns - 893.12 ns | Recognizes the principal branch of acos(cos(9pi/7)). |
| `exact_transcendental_special_forms/atan_tan_6pi_7` | 479.89 ns | 477.82 ns - 482.26 ns | Recognizes the principal branch of atan(tan(6pi/7)). |
| `exact_transcendental_special_forms/asinh_large` | 188.12 ns | 187.26 ns - 189.07 ns | Builds a large inverse hyperbolic sine without exact intermediate Reals. |
| `exact_transcendental_special_forms/atanh_sqrt_half` | 85.60 ns | 85.29 ns - 86.00 ns | Builds atanh(sqrt(2)/2) after exact structural domain checks. |
| `exact_transcendental_special_forms/atanh_sqrt_two_error` | 17.15 ns | 17.13 ns - 17.17 ns | Rejects atanh(sqrt(2)) through exact structural domain checks. |
| `exact_transcendental_special_forms/sinh_ln_two` | 125.63 ns | 125.11 ns - 126.28 ns | Folds sinh(ln(2)) to the exact rational 3/4 via the integer-log-collapse shortcut. |
| `exact_transcendental_special_forms/cosh_ln_two` | 131.53 ns | 131.21 ns - 131.86 ns | Folds cosh(ln(2)) to the exact rational 5/4 via the integer-log-collapse shortcut. |
| `exact_transcendental_special_forms/tanh_ln_two` | 249.37 ns | 248.77 ns - 250.10 ns | Folds tanh(ln(2)) to the exact rational 3/5 via the integer-log-collapse shortcut. |
| `exact_transcendental_special_forms/sinh_rational_one` | 419.64 ns | 409.40 ns - 438.52 ns | Builds sinh(1) through the generic (exp(x) - exp(-x))/2 identity path. |
| `exact_transcendental_special_forms/cosh_rational_one` | 355.12 ns | 353.69 ns - 356.72 ns | Builds cosh(1) through the generic (exp(x) + exp(-x))/2 identity path. |
| `exact_transcendental_special_forms/tanh_rational_one` | 581.22 ns | 572.99 ns - 595.70 ns | Builds tanh(1) through the generic (exp(x) - exp(-x))/(exp(x) + exp(-x)) identity path. |
| `exact_transcendental_special_forms/atan2_origin` | 18.42 ns | 18.41 ns - 18.44 ns | Hits the origin (0, 0) short-circuit returning exact zero. |
| `exact_transcendental_special_forms/atan2_axis_positive_y` | 31.89 ns | 31.81 ns - 31.99 ns | Hits the positive-y axis short-circuit returning exact pi/2. |
| `exact_transcendental_special_forms/atan2_axis_negative_x` | 29.75 ns | 29.69 ns - 29.83 ns | Hits the negative-x axis short-circuit returning exact pi. |
| `exact_transcendental_special_forms/atan2_quadrant_one_unit_diagonal` | 74.95 ns | 74.81 ns - 75.14 ns | Quadrant I unit diagonal reduces to atan(1) = pi/4 exact special form. |
| `exact_transcendental_special_forms/atan2_quadrant_two_pi_correction` | 775.64 ns | 773.77 ns - 777.75 ns | Quadrant II (1, -2) exercises atan(small ratio) + pi correction. |
| `exact_transcendental_special_forms/atan2_quadrant_three_negative_pi` | 700.39 ns | 698.71 ns - 702.30 ns | Quadrant III (-1, -2) exercises atan(small ratio) - pi correction. |
| `exact_transcendental_special_forms/log2_power_of_two` | 60.61 ns | 60.49 ns - 60.76 ns | Folds log2(1024) to the exact rational 10 via the integer-log-detection shortcut. |
| `exact_transcendental_special_forms/log2_rational_three` | 131.06 ns | 130.45 ns - 131.76 ns | Builds log2(3) as a lightweight Log2 symbolic certificate. |
| `exact_transcendental_special_forms/log2_ln_quotient_fold` | 145.50 ns | 144.98 ns - 146.08 ns | Folds ln(5) / ln(2) into a Log2 certificate via the divide-recognize shortcut. |

### `real_cotangent`

Public cotangent construction and cold refinement against the two compositional alternatives.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_cotangent/atan_two_direct_construct` | 34.78 ns | 34.67 ns - 34.91 ns | Folds cot(atan(2)) through the public exact inverse-trig rewrite. |
| `real_cotangent/atan_two_quotient_construct` | 1.406 us | 1.403 us - 1.409 us | Builds cot(atan(2)) as the control expression cos(x)/sin(x). |
| `real_cotangent/atan_two_inverse_tan_construct` | 4.416 us | 4.405 us - 4.427 us | Builds cot(atan(2)) as the incomplete control expression 1/tan(x). |
| `real_cotangent/tiny_direct_cold_p256` | 7.791 us | 7.765 us - 7.826 us | Constructs and certifies public cot(2^-10) at 256 fractional bits. |
| `real_cotangent/tiny_quotient_cold_p256` | 7.620 us | 7.611 us - 7.630 us | Constructs and certifies cos(2^-10)/sin(2^-10) at 256 fractional bits. |
| `real_cotangent/tiny_inverse_tan_cold_p256` | 10.004 us | 9.989 us - 10.020 us | Constructs and certifies 1/tan(2^-10) at 256 fractional bits. |

### `symbolic_reductions`

Existing symbolic constant algebra cases considered for additional reductions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `symbolic_reductions/sqrt_pi_square` | 279.51 ns | 278.29 ns - 281.04 ns | Reduces sqrt(pi^2). |
| `symbolic_reductions/sqrt_pi_e_square` | 546.99 ns | 546.21 ns - 547.81 ns | Reduces sqrt((pi * e)^2). |
| `symbolic_reductions/ln_scaled_e` | 60.78 ns | 60.45 ns - 61.17 ns | Reduces ln(2 * e). |
| `symbolic_reductions/sub_pi_three` | 43.30 ns | 43.11 ns - 43.51 ns | Builds the certified pi - 3 constant-offset form. |
| `symbolic_reductions/pi_minus_three_facts` | 17.32 ns | 17.27 ns - 17.37 ns | Reads structural facts for the cached pi - 3 offset form. |
| `symbolic_reductions/div_exp_exp` | 308.78 ns | 307.23 ns - 310.62 ns | Reduces e^3 / e. |
| `symbolic_reductions/div_pi_square_e` | 304.98 ns | 303.32 ns - 306.95 ns | Reduces pi^2 / e. |
| `symbolic_reductions/div_const_products` | 412.48 ns | 399.55 ns - 436.63 ns | Reduces (pi^3 * e^5) / (pi * e^2). |
| `symbolic_reductions/inverse_pi` | 30.93 ns | 30.78 ns - 31.10 ns | Builds the reciprocal of pi. |
| `symbolic_reductions/div_one_pi` | 62.66 ns | 62.39 ns - 62.95 ns | Reduces 1 / pi. |
| `symbolic_reductions/div_rational_exp` | 184.01 ns | 164.62 ns - 221.62 ns | Reduces 2 / e. |
| `symbolic_reductions/div_e_pi` | 120.16 ns | 119.25 ns - 121.22 ns | Reduces e / pi. |
| `symbolic_reductions/mul_pi_inverse_pi` | 67.28 ns | 66.98 ns - 67.59 ns | Multiplies pi by its reciprocal. |
| `symbolic_reductions/mul_pi_e_sqrt_two` | 188.81 ns | 187.06 ns - 190.68 ns | Builds the factored pi * e * sqrt(2) form. |
| `symbolic_reductions/mul_const_product_sqrt_sqrt` | 156.14 ns | 155.31 ns - 157.04 ns | Cancels sqrt(2) from (pi * e * sqrt(2)) * sqrt(2). |
| `symbolic_reductions/div_const_product_sqrt_e` | 73.03 ns | 72.48 ns - 73.64 ns | Reduces (pi * e * sqrt(2)) / e. |
| `symbolic_reductions/inverse_const_product_sqrt` | 297.30 ns | 295.14 ns - 299.72 ns | Builds a rationalized reciprocal of pi * e * sqrt(2). |
| `symbolic_reductions/inverse_sqrt_two` | 16.52 ns | 16.46 ns - 16.60 ns | Builds the rationalized reciprocal of unit-scaled sqrt(2). |
| `symbolic_reductions/div_sqrt_two_sqrt_three` | 47.00 ns | 46.94 ns - 47.05 ns | Rationalizes a quotient of two unit-scaled square roots. |

### `exact_product_sums`

Fixed product-sum reducers used by determinant and cofactor kernels.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `exact_product_sums/signed_product_sum_lcm_6x2` | 320.79 ns | 320.38 ns - 321.29 ns | Computes an exact rational six-term signed product sum with mixed denominators. |
| `exact_product_sums/signed_product_sum_common_scale_6x2` | 171.33 ns | 170.54 ns - 172.33 ns | Computes an exact rational six-term signed product sum through the carried common-scale reducer. |
| `exact_product_sums/signed_product_sum_sparse_single_6x2` | 102.84 ns | 102.58 ns - 103.12 ns | Computes a sparse exact rational six-term signed product sum with one active product. |
| `exact_product_sums/real_signed_product_sum_rational_det3` | 301.32 ns | 300.61 ns - 302.14 ns | Computes a 3x3 determinant-shaped signed product sum through the public `Real` builder. |
| `exact_product_sums/real_signed_product_sum_mixed_symbolic_det3` | 2.091 us | 2.087 us - 2.095 us | Computes the same determinant-shaped builder with symbolic factors and rational scales. |
| `exact_product_sums/exact_rational_sparse_homogeneous_plane_intersection3` | 209.96 ns | 208.91 ns - 211.12 ns | Computes a canonical exact three-plane cofactor tuple from one sparse dyadic row. |

<!-- END scalar_micro -->

<!-- BEGIN library_perf -->
## `library_perf`

Library-level Criterion benchmarks for public `Rational`, `Real`, and `Simple` behavior.

### `real_format`

Formatting costs for important irrational `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_format/pi_lower_exp_32` | 4.779 us | 4.774 us - 4.785 us | Formats pi with 32 digits in lower-exponential form. |
| `real_format/pi_display_alt_32` | 5.369 us | 5.336 us - 5.407 us | Formats pi with alternate decimal display at 32 digits. |
| `real_format/sqrt_two_display_alt_32` | 4.920 us | 4.910 us - 4.932 us | Formats sqrt(2) with alternate decimal display at 32 digits. |

### `real_constants`

Construction cost for shared mathematical constants.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_constants/pi` | 16.57 ns | 16.56 ns - 16.59 ns | Constructs the symbolic pi value. |
| `real_constants/e` | 21.14 ns | 21.11 ns - 21.17 ns | Constructs the symbolic Euler constant value. |

### `simple`

Parser and evaluator costs for the `Simple` expression language.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple/parse_nested` | 410.44 ns | 409.34 ns - 411.85 ns | Parses a nested expression with powers, trig, and constants. |
| `simple/eval_nested` | 1.706 us | 1.698 us - 1.718 us | Evaluates a parsed mixed symbolic/numeric expression. |
| `simple/eval_constants` | 1.234 us | 1.224 us - 1.244 us | Evaluates repeated built-in constants. |
| `simple/eval_exact` | 273.67 ns | 272.21 ns - 275.02 ns | Evaluates a rational-only expression through exact shortcuts. |
| `simple/eval_nested_exact` | 899.19 ns | 893.73 ns - 904.45 ns | Evaluates a nested rational-only expression through exact shortcuts. |

### `real_powi`

Integer exponentiation for exact and irrational `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_powi/exact_17` | 120.07 ns | 119.66 ns - 120.56 ns | Raises an exact rational-backed `Real` to the 17th power. |
| `real_powi/exact_17_i64` | 77.99 ns | 77.86 ns - 78.15 ns | Raises an exact rational-backed `Real` through the machine-sized exponent API. |
| `real_powi/irrational_17` | 153.05 ns | 151.71 ns - 154.64 ns | Raises sqrt(3) to the 17th power with symbolic simplification. |
| `real_powi/large_exact_lazy_20000` | 48.511 us | 48.330 us - 48.717 us | Routes an oversized exact rational power to its bounded lazy exact representation. |

### `rational_powi`

Integer exponentiation for `Rational`.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `rational_powi/exact_17` | 79.61 ns | 79.22 ns - 80.03 ns | Raises a rational value to the 17th power. |
| `rational_powi/oversized_20000_exhausted` | 18.29 ns | 18.25 ns - 18.33 ns | Rejects eager materialization before an oversized rational power allocates its result. |

### `iterator_products`

Sequential and balanced exact products over a 1,000-factor Wallis corpus.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `iterator_products/rational_wallis_sequential_1000` | 53.927 ms | 53.842 ms - 54.028 ms | Multiplies borrowed rational factors with a conventional left fold. |
| `iterator_products/rational_wallis_owned_1000` | 1.510 ms | 1.507 ms - 1.514 ms | Consumes rational factors through the balanced `Product` implementation. |
| `iterator_products/rational_wallis_borrowed_1000` | 1.557 ms | 1.551 ms - 1.564 ms | Clones borrowed rational factors into the balanced `Product` implementation. |
| `iterator_products/real_wallis_owned_1000` | 1.513 ms | 1.511 ms - 1.516 ms | Consumes exact rational-backed `Real` factors through their balanced `Product` implementation. |

### `real_exact_trig`

Exact and symbolic trig construction for known pi multiples.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_trig/sin_pi_6` | 517.86 ns | 515.87 ns - 520.42 ns | Computes sin(pi/6) via exact shortcut. |
| `real_exact_trig/cos_pi_3` | 429.80 ns | 428.85 ns - 430.81 ns | Computes cos(pi/3) via exact shortcut. |
| `real_exact_trig/tan_pi_5` | 234.35 ns | 233.56 ns - 235.18 ns | Builds tan(pi/5), a nontrivial symbolic tangent. |

### `real_general_trig`

General trig construction for irrational arguments.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_general_trig/tan_sqrt_2` | 411.75 ns | 410.87 ns - 412.83 ns | Builds tan(sqrt(2)). |
| `real_general_trig/tan_pi_sqrt_2_over_5` | 1.709 us | 1.701 us - 1.720 us | Builds tangent of an irrational multiple of pi. |

### `real_exact_inverse_trig`

Exact inverse trig shortcuts and symbolic inverse trig recognition.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_inverse_trig/asin_1_2` | 29.35 ns | 29.23 ns - 29.51 ns | Recognizes asin(1/2) as pi/6. |
| `real_exact_inverse_trig/asin_minus_1_2` | 41.08 ns | 40.99 ns - 41.17 ns | Recognizes asin(-1/2) as -pi/6. |
| `real_exact_inverse_trig/asin_sqrt_2_over_2` | 61.03 ns | 60.96 ns - 61.12 ns | Recognizes asin(sqrt(2)/2) as pi/4. |
| `real_exact_inverse_trig/asin_sin_pi_5` | 57.38 ns | 57.19 ns - 57.61 ns | Inverts a symbolic sin(pi/5). |
| `real_exact_inverse_trig/acos_1` | 18.47 ns | 18.37 ns - 18.59 ns | Recognizes acos(1) as zero. |
| `real_exact_inverse_trig/acos_minus_1` | 22.91 ns | 22.86 ns - 22.98 ns | Recognizes acos(-1) as pi. |
| `real_exact_inverse_trig/acos_1_2` | 30.27 ns | 30.10 ns - 30.48 ns | Recognizes acos(1/2) as pi/3. |
| `real_exact_inverse_trig/atan_1` | 23.67 ns | 23.64 ns - 23.71 ns | Recognizes atan(1) as pi/4. |
| `real_exact_inverse_trig/atan_sqrt_3_over_3` | 66.29 ns | 65.94 ns - 66.72 ns | Recognizes atan(sqrt(3)/3) as pi/6. |
| `real_exact_inverse_trig/atan_tan_pi_5` | 58.16 ns | 57.93 ns - 58.43 ns | Inverts a symbolic tan(pi/5). |

### `real_general_inverse_trig`

General inverse trig construction, domain errors, and atan range reduction.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_general_inverse_trig/asin_7_10` | 183.48 ns | 182.09 ns - 185.08 ns | Builds asin(7/10) through the rational-specialized path. |
| `real_general_inverse_trig/asin_near_one` | 181.84 ns | 181.50 ns - 182.22 ns | Builds a deferred exact-rational asin near the positive endpoint. |
| `real_general_inverse_trig/asin_near_minus_one` | 186.59 ns | 186.22 ns - 187.08 ns | Builds a deferred exact-rational asin near the negative endpoint. |
| `real_general_inverse_trig/asin_sqrt_2_over_3` | 325.87 ns | 324.79 ns - 327.17 ns | Builds asin(sqrt(2)/3) through the general path. |
| `real_general_inverse_trig/acos_7_10` | 178.53 ns | 177.54 ns - 179.75 ns | Builds acos(7/10) through the rational-specialized asin path. |
| `real_general_inverse_trig/acos_sqrt_2_over_3` | 729.87 ns | 728.27 ns - 731.54 ns | Builds acos(sqrt(2)/3) through the general path. |
| `real_general_inverse_trig/asin_11_10_error` | 23.26 ns | 23.18 ns - 23.36 ns | Rejects rational asin input outside [-1, 1]. |
| `real_general_inverse_trig/acos_11_10_error` | 21.11 ns | 21.09 ns - 21.14 ns | Rejects rational acos input outside [-1, 1]. |
| `real_general_inverse_trig/atan_8` | 372.79 ns | 371.61 ns - 374.14 ns | Builds atan(8), exercising large-argument reduction. |
| `real_general_inverse_trig/atan_sqrt_2` | 2.909 us | 2.903 us - 2.917 us | Builds atan(sqrt(2)). |

### `real_inverse_hyperbolic`

Inverse hyperbolic construction, exact exits, stable ln1p forms, and domain errors.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_inverse_hyperbolic/asinh_0` | 11.22 ns | 11.17 ns - 11.29 ns | Recognizes asinh(0) as zero. |
| `real_inverse_hyperbolic/asinh_1_2` | 184.78 ns | 184.06 ns - 185.65 ns | Builds asinh(1/2) through the stable moderate-input path. |
| `real_inverse_hyperbolic/asinh_sqrt_2` | 136.10 ns | 115.95 ns - 176.23 ns | Builds asinh(sqrt(2)) without cancellation-prone log construction. |
| `real_inverse_hyperbolic/asinh_minus_1_2` | 240.41 ns | 238.56 ns - 242.45 ns | Uses odd symmetry for negative asinh input. |
| `real_inverse_hyperbolic/asinh_1_000_000` | 194.89 ns | 194.37 ns - 195.48 ns | Builds asinh for a large positive rational. |
| `real_inverse_hyperbolic/acosh_1` | 12.46 ns | 12.45 ns - 12.48 ns | Recognizes acosh(1) as zero. |
| `real_inverse_hyperbolic/acosh_2` | 41.62 ns | 41.41 ns - 41.87 ns | Builds acosh(2) through the stable moderate-input path. |
| `real_inverse_hyperbolic/acosh_sqrt_2` | 107.81 ns | 99.55 ns - 124.14 ns | Builds acosh(sqrt(2)) through square-root domain specialization. |
| `real_inverse_hyperbolic/acosh_1_000_000` | 158.66 ns | 157.62 ns - 159.80 ns | Builds acosh for a large positive rational. |
| `real_inverse_hyperbolic/atanh_0` | 11.14 ns | 11.12 ns - 11.16 ns | Recognizes atanh(0) as zero. |
| `real_inverse_hyperbolic/atanh_1_2` | 25.15 ns | 25.11 ns - 25.20 ns | Builds exact-rational atanh(1/2). |
| `real_inverse_hyperbolic/atanh_minus_1_2` | 48.10 ns | 48.03 ns - 48.19 ns | Builds exact-rational atanh(-1/2). |
| `real_inverse_hyperbolic/atanh_sqrt_half` | 95.62 ns | 85.24 ns - 116.15 ns | Recognizes atanh(sqrt(2)/2) as asinh(1). |
| `real_inverse_hyperbolic/atanh_9_10` | 209.19 ns | 208.84 ns - 209.58 ns | Builds exact-rational atanh near the upper domain boundary. |
| `real_inverse_hyperbolic/atanh_1_error` | 10.38 ns | 10.34 ns - 10.41 ns | Rejects atanh(1) at the rational domain boundary. |

### `simple_inverse_functions`

Parsed/evaluated inverse trig and inverse hyperbolic expressions that should succeed.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple_inverse_functions/asin_1_2` | 66.31 ns | 66.07 ns - 66.54 ns | Evaluates `(asin 1/2)`. |
| `simple_inverse_functions/acos_1_2` | 65.94 ns | 65.70 ns - 66.19 ns | Evaluates `(acos 1/2)`. |
| `simple_inverse_functions/atan_1` | 64.09 ns | 63.85 ns - 64.30 ns | Evaluates `(atan 1)`. |
| `simple_inverse_functions/asin_general` | 214.22 ns | 213.18 ns - 215.52 ns | Evaluates `(asin 7/10)`. |
| `simple_inverse_functions/acos_general` | 215.01 ns | 213.44 ns - 216.82 ns | Evaluates `(acos 7/10)`. |
| `simple_inverse_functions/atan_general` | 402.07 ns | 400.92 ns - 403.91 ns | Evaluates `(atan 8)`. |
| `simple_inverse_functions/asinh_1_2` | 226.85 ns | 226.24 ns - 227.51 ns | Evaluates `(asinh 1/2)`. |
| `simple_inverse_functions/asinh_sqrt_2` | 297.36 ns | 296.91 ns - 297.81 ns | Evaluates `(asinh (sqrt 2))`. |
| `simple_inverse_functions/acosh_2` | 63.54 ns | 63.35 ns - 63.72 ns | Evaluates `(acosh 2)`. |
| `simple_inverse_functions/acosh_sqrt_2` | 249.27 ns | 248.05 ns - 250.90 ns | Evaluates `(acosh (sqrt 2))`. |
| `simple_inverse_functions/atanh_1_2` | 58.94 ns | 58.68 ns - 59.22 ns | Evaluates `(atanh 1/2)`. |
| `simple_inverse_functions/atanh_minus_1_2` | 75.84 ns | 75.56 ns - 76.11 ns | Evaluates `(atanh -1/2)`. |

### `simple_inverse_error_functions`

Parsed/evaluated inverse function expressions that should fail quickly with `NotANumber`.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple_inverse_error_functions/asin_11_10` | 55.61 ns | 55.40 ns - 55.82 ns | Rejects `(asin 11/10)`. |
| `simple_inverse_error_functions/acos_sqrt_2` | 264.09 ns | 263.16 ns - 265.20 ns | Rejects `(acos (sqrt 2))`. |
| `simple_inverse_error_functions/acosh_0` | 38.96 ns | 38.78 ns - 39.13 ns | Rejects `(acosh 0)`. |
| `simple_inverse_error_functions/acosh_minus_2` | 39.19 ns | 39.01 ns - 39.38 ns | Rejects `(acosh -2)`. |
| `simple_inverse_error_functions/atanh_1` | 42.92 ns | 42.73 ns - 43.10 ns | Rejects `(atanh 1)`. |
| `simple_inverse_error_functions/atanh_sqrt_2` | 159.13 ns | 158.82 ns - 159.42 ns | Rejects `(atanh (sqrt 2))`. |

### `real_exact_ln`

Exact logarithm construction and simplification for rational inputs.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_ln/ln_1024` | 82.28 ns | 81.97 ns - 82.65 ns | Recognizes ln(1024) as 10 ln(2). |
| `real_exact_ln/ln_1_8` | 77.19 ns | 76.96 ns - 77.50 ns | Recognizes ln(1/8) as -3 ln(2). |
| `real_exact_ln/ln_1000` | 51.13 ns | 51.05 ns - 51.23 ns | Simplifies ln(1000) via small integer logarithm factors. |

### `real_exact_exp_log10`

Exact inverse relationships among exp, ln, log2, and log10.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_exp_log10/exp_ln_1000` | 63.80 ns | 63.70 ns - 63.92 ns | Simplifies exp(ln(1000)) back to 1000. |
| `real_exact_exp_log10/exp_ln_1_8` | 67.06 ns | 66.89 ns - 67.27 ns | Simplifies exp(ln(1/8)) back to 1/8. |
| `real_exact_exp_log10/log10_1000` | 33.76 ns | 33.57 ns - 33.99 ns | Recognizes log10(1000) as 3. |
| `real_exact_exp_log10/log10_1_1000` | 62.63 ns | 62.56 ns - 62.70 ns | Recognizes log10(1/1000) as -3. |
| `real_exact_exp_log10/pow2_log2_3` | 55.12 ns | 55.07 ns - 55.18 ns | Builds 2 raised to the retained log2(3) certificate. |
| `real_exact_exp_log10/pow10_log10_2` | 55.06 ns | 55.03 ns - 55.09 ns | Builds 10 raised to the retained log10(2) certificate. |
| `real_exact_exp_log10/log2_exp2_1_7` | 18.66 ns | 18.61 ns - 18.72 ns | Recovers 1/7 from the exact algebraic value 2^(1/7). |
| `real_exact_exp_log10/log10_exp10_6411_4096` | 18.34 ns | 18.32 ns - 18.36 ns | Recovers Kahan's 6411/4096 exponent from its exact base-ten power. |

### `real_stable_scalar_substrate`

Stable scalar constructors that preserve small residuals, dominance, roots, rational powers, and certified integer decisions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_stable_scalar_substrate/ln_1p_tiny` | 54.88 ns | 54.70 ns - 55.07 ns | Builds ln(1 + tiny) without first adding one generically. |
| `real_stable_scalar_substrate/ln_1m_tiny` | 59.88 ns | 59.68 ns - 60.09 ns | Builds ln(1 - tiny) through the log1p companion path. |
| `real_stable_scalar_substrate/expm1_tiny` | 154.08 ns | 153.16 ns - 155.20 ns | Builds exp(tiny) - 1 through the dedicated expm1 node. |
| `real_stable_scalar_substrate/softplus_large_positive` | 2.077 us | 2.061 us - 2.100 us | Builds softplus for a dominant positive input. |
| `real_stable_scalar_substrate/softplus_large_negative` | 1.901 us | 1.891 us - 1.918 us | Builds softplus for a dominant negative input. |
| `real_stable_scalar_substrate/logaddexp_dominant` | 2.191 us | 2.188 us - 2.195 us | Builds logaddexp when one side is certifiably dominant. |
| `real_stable_scalar_substrate/logsubexp_near` | 291.53 ns | 290.56 ns - 292.70 ns | Builds logsubexp for a certifiably positive but small log-space difference. |
| `real_stable_scalar_substrate/sigmoid_large_positive` | 2.089 us | 2.076 us - 2.109 us | Builds a large positive sigmoid through the stable tail path. |
| `real_stable_scalar_substrate/logit_near_one` | 393.31 ns | 392.82 ns - 393.88 ns | Builds logit close to the upper probability boundary. |
| `real_stable_scalar_substrate/sqrt1pm1_tiny` | 704.75 ns | 703.74 ns - 706.04 ns | Builds sqrt(1 + tiny) - 1 through the stable helper. |
| `real_stable_scalar_substrate/sqrt1m1_tiny` | 724.02 ns | 722.96 ns - 725.36 ns | Builds sqrt(1 - tiny) - 1 through the stable helper. |
| `real_stable_scalar_substrate/sqrt_quadratic_surd_perfect_norm` | 1.441 us | 1.438 us - 1.445 us | Recovers the exact principal root of 3 + 2*sqrt(2) from its square conjugate norm. |
| `real_stable_scalar_substrate/certified_compare_nested_radical_identity` | 234.52 ns | 232.33 ns - 237.15 ns | Certifies sqrt(x)+sqrt(y) against the equivalent nested radical after exact construction. |
| `real_stable_scalar_substrate/cbrt_negative_perfect` | 141.28 ns | 140.92 ns - 141.64 ns | Collapses a negative perfect cube. |
| `real_stable_scalar_substrate/root_n_perfect_fourth` | 146.81 ns | 146.44 ns - 147.25 ns | Collapses an exact fourth root. |
| `real_stable_scalar_substrate/pow_rational_negative_odd_denominator` | 246.44 ns | 240.99 ns - 256.77 ns | Routes a negative rational base through odd-root symmetry. |
| `real_stable_scalar_substrate/near_integer_rational` | 74.65 ns | 74.33 ns - 75.04 ns | Chooses one adjacent integer from an exact rational without refinement. |
| `real_stable_scalar_substrate/near_integer_sqrt2` | 229.61 ns | 224.74 ns - 238.66 ns | Chooses one adjacent integer from a cold irrational expression with one bounded approximation. |
| `real_stable_scalar_substrate/floor_certified_sqrt2` | 3.055 us | 3.046 us - 3.064 us | Certifies the directional floor of the same cold irrational expression. |
| `real_stable_scalar_substrate/floor_certified_rational` | 74.93 ns | 74.80 ns - 75.09 ns | Certifies rational floor structurally. |
| `real_stable_scalar_substrate/rem_euclid_certified_rational` | 311.79 ns | 311.30 ns - 312.29 ns | Computes rational Euclidean remainder through certified quotient floor. |

### `real_geometry_polynomial_substrate`

Geometry-facing scalar helpers for rational-turn trig, removable small-angle limits, vectors, product sums, and polynomial forms.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_geometry_polynomial_substrate/sin_pi_one_sixth` | 78.12 ns | 77.52 ns - 78.85 ns | Uses exact rational-turn sine. |
| `real_geometry_polynomial_substrate/cos_pi_one_fourth` | 35.79 ns | 35.76 ns - 35.83 ns | Uses exact rational-turn cosine. |
| `real_geometry_polynomial_substrate/cos_pi_one_seventh` | 245.98 ns | 245.12 ns - 247.11 ns | Builds a non-tabulated rational-turn cosine certificate. |
| `real_geometry_polynomial_substrate/tan_pi_one_third` | 30.23 ns | 30.17 ns - 30.30 ns | Uses exact rational-turn tangent. |
| `real_geometry_polynomial_substrate/sinc_zero` | 10.80 ns | 10.79 ns - 10.81 ns | Returns the removable sinc limit at zero. |
| `real_geometry_polynomial_substrate/sinc_tiny` | 343.20 ns | 342.09 ns - 344.61 ns | Builds sinc for a tiny exact input. |
| `real_geometry_polynomial_substrate/sinc_pi_half` | 185.29 ns | 184.50 ns - 186.18 ns | Builds normalized sinc for an exact half turn. |
| `real_geometry_polynomial_substrate/cosc_tiny` | 598.44 ns | 597.44 ns - 599.56 ns | Builds the small-angle (1 - cos x) / x^2 helper. |
| `real_geometry_polynomial_substrate/sinc_opaque_zero` | 4.349 us | 4.310 us - 4.396 us | Continues sinc across a freshly built opaque trigonometric zero. |
| `real_geometry_polynomial_substrate/sinc_pi_opaque_zero` | 6.710 us | 6.671 us - 6.762 us | Continues normalized sinc across a freshly built opaque trigonometric zero. |
| `real_geometry_polynomial_substrate/cosc_opaque_zero` | 4.434 us | 4.416 us - 4.455 us | Continues cosc across a freshly built opaque trigonometric zero. |
| `real_geometry_polynomial_substrate/atan2_axis` | 30.01 ns | 29.94 ns - 30.10 ns | Classifies an axis-aligned atan2 input exactly. |
| `real_geometry_polynomial_substrate/atan2_quadrant` | 186.56 ns | 185.67 ns - 187.55 ns | Builds a quadrant-correct atan2 expression. |
| `real_geometry_polynomial_substrate/hypot2_3_4` | 81.21 ns | 80.92 ns - 81.60 ns | Collapses a 3-4-5 norm through exact dot products. |
| `real_geometry_polynomial_substrate/hypot3_2_3_6` | 138.76 ns | 138.62 ns - 138.91 ns | Collapses a 2-3-6 norm through exact dot products. |
| `real_geometry_polynomial_substrate/hypot_minus_tiny` | 2.006 us | 1.999 us - 2.016 us | Uses rationalized hypot-minus for cancellation resistance. |
| `real_geometry_polynomial_substrate/mul_add_zero_product` | 35.02 ns | 34.85 ns - 35.22 ns | Skips a known-zero product lane. |
| `real_geometry_polynomial_substrate/sum_products_dense` | 1.973 us | 1.965 us - 1.982 us | Builds a dense product sum. |
| `real_geometry_polynomial_substrate/diff_of_products_near_cancel` | 296.07 ns | 295.00 ns - 297.23 ns | Preserves determinant-like product difference structure. |
| `real_geometry_polynomial_substrate/eval_poly_horner` | 1.187 us | 1.183 us - 1.192 us | Evaluates a polynomial through Horner form. |
| `real_geometry_polynomial_substrate/eval_rational_poly` | 1.541 us | 1.537 us - 1.547 us | Evaluates numerator and denominator polynomial forms before division. |

### `real_normal_scientific_substrate`

Gaussian tail helpers and exact/finite scientific special-function forms added for higher numerical workloads.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_normal_scientific_substrate/erfc_zero` | 8.66 ns | 8.65 ns - 8.68 ns | Takes the exact erfc(0) exit. |
| `real_normal_scientific_substrate/erfcx_tail` | 1.595 us | 1.591 us - 1.598 us | Builds scaled erfc in a positive tail. |
| `real_normal_scientific_substrate/normal_sf_tail` | 369.96 ns | 368.98 ns - 371.13 ns | Builds standard-normal upper-tail probability. |
| `real_normal_scientific_substrate/pnorm_upper_tail` | 371.50 ns | 370.29 ns - 372.97 ns | Builds the upper-tail alias. |
| `real_normal_scientific_substrate/log_pnorm_tail` | 286.37 ns | 282.45 ns - 293.21 ns | Builds lower log-CDF tail form. |
| `real_normal_scientific_substrate/log_pnorm_zero` | 48.19 ns | 47.98 ns - 48.47 ns | Takes the exact log-CDF value at zero. |
| `real_normal_scientific_substrate/log_normal_sf_tail` | 304.49 ns | 303.35 ns - 305.79 ns | Builds upper log-survival tail form. |
| `real_normal_scientific_substrate/log_normal_sf_zero` | 48.55 ns | 48.25 ns - 48.90 ns | Takes the exact log-survival value at zero. |
| `real_normal_scientific_substrate/log_dnorm_large` | 144.25 ns | 143.43 ns - 145.22 ns | Builds analytic log-density at a large input. |
| `real_normal_scientific_substrate/normal_interval_narrow` | 759.05 ns | 755.85 ns - 762.60 ns | Builds a narrow interval mass without spelling pnorm subtraction. |
| `real_normal_scientific_substrate/erfinv_mid` | 1.447 us | 1.440 us - 1.455 us | Builds inverse error function through qnorm transform. |
| `real_normal_scientific_substrate/erfcinv_tail` | 1.642 us | 1.637 us - 1.646 us | Builds inverse complementary error function through tail qnorm transform. |
| `real_normal_scientific_substrate/qnorm_upper_tail` | 1.025 us | 1.021 us - 1.030 us | Builds inverse survival quantile. |
| `real_normal_scientific_substrate/normal_pdf_parametric` | 1.295 us | 1.291 us - 1.300 us | Standardizes exactly before density construction. |
| `real_normal_scientific_substrate/normal_survival_parametric` | 522.95 ns | 522.48 ns - 523.45 ns | Standardizes exactly before upper-tail construction. |
| `real_normal_scientific_substrate/normal_mills_tail` | 2.863 us | 2.855 us - 2.873 us | Builds Mills ratio through erfcx identity. |
| `real_normal_scientific_substrate/normal_mills_zero` | 15.87 ns | 15.82 ns - 15.93 ns | Takes the exact Mills ratio value at zero. |
| `real_normal_scientific_substrate/normal_hazard_tail` | 2.930 us | 2.926 us - 2.936 us | Builds reciprocal Mills hazard. |
| `real_normal_scientific_substrate/normal_hazard_zero` | 16.18 ns | 16.14 ns - 16.23 ns | Takes the exact hazard value at zero. |
| `real_normal_scientific_substrate/normal_inverse_mills_zero` | 16.02 ns | 15.97 ns - 16.08 ns | Takes the exact lower inverse Mills value at zero. |
| `real_normal_scientific_substrate/hermite_8` | 1.333 us | 1.331 us - 1.336 us | Builds an exact probabilists' Hermite polynomial. |
| `real_normal_scientific_substrate/dnorm_derivative_4` | 1.666 us | 1.663 us - 1.668 us | Combines exact Hermite polynomial with normal density. |
| `real_normal_scientific_substrate/standard_normal_moment_12` | 144.37 ns | 143.95 ns - 144.85 ns | Uses double-factorial closed form. |
| `real_normal_scientific_substrate/normal_interval_moment_3` | 2.457 us | 2.446 us - 2.469 us | Uses interval mass and density-boundary recurrence. |
| `real_normal_scientific_substrate/truncated_normal_mean` | 2.518 us | 2.511 us - 2.526 us | Builds truncated-normal mean from stable interval mass. |
| `real_normal_scientific_substrate/gamma_integer` | 102.80 ns | 102.48 ns - 103.21 ns | Uses exact integer gamma closed form. |
| `real_normal_scientific_substrate/gamma_half_integer` | 306.48 ns | 305.65 ns - 307.43 ns | Uses exact half-integer gamma closed form. |
| `real_normal_scientific_substrate/lgamma_half_integer` | 1.154 us | 1.152 us - 1.156 us | Logs the absolute half-integer gamma value. |
| `real_normal_scientific_substrate/beta_integer` | 132.71 ns | 132.38 ns - 133.09 ns | Builds integer beta through an exact factorial ratio. |
| `real_normal_scientific_substrate/ln_beta_half_integer` | 1.758 us | 1.752 us - 1.765 us | Builds log beta through lgamma sum. |
| `real_normal_scientific_substrate/regularized_beta_mid` | 1.235 us | 1.231 us - 1.241 us | Uses finite positive-integer beta binomial tail. |
| `real_normal_scientific_substrate/regularized_beta_uniform` | 160.41 ns | 159.28 ns - 161.68 ns | Takes the exact I_x(1, 1) identity. |
| `real_normal_scientific_substrate/regularized_beta_left_unity` | 329.26 ns | 328.64 ns - 329.96 ns | Reduces I_x(1, b) to one complement power. |
| `real_normal_scientific_substrate/regularized_beta_q_mid` | 840.73 ns | 836.93 ns - 845.50 ns | Uses finite positive-integer beta upper-tail form. |
| `real_normal_scientific_substrate/regularized_beta_q_uniform` | 175.20 ns | 174.78 ns - 175.68 ns | Takes the exact upper-tail I_x(1, 1) complement. |
| `real_normal_scientific_substrate/regularized_beta_q_left_unity` | 237.16 ns | 236.79 ns - 237.65 ns | Reduces the upper beta tail for a = 1 to one power. |
| `real_normal_scientific_substrate/regularized_gamma_p_half` | 1.852 us | 1.844 us - 1.862 us | Uses half-integer incomplete-gamma recurrence. |
| `real_normal_scientific_substrate/regularized_gamma_q_integer` | 659.09 ns | 657.60 ns - 660.87 ns | Uses integer incomplete-gamma recurrence. |
| `real_normal_scientific_substrate/chi_square_sf` | 1.573 us | 1.567 us - 1.580 us | Wraps regularized upper gamma for chi-square upper tail. |

### `simple_new_function_surface`

Parser and evaluator coverage for the newly exposed stable scalar, geometry, normal, and scientific functions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple_new_function_surface/stable_log_exp_bundle` | 8.792 us | 8.734 us - 8.861 us | Evaluates log1p/log1m/expm1/softplus/logaddexp/logsubexp/sigmoid/logit together. |
| `simple_new_function_surface/geometry_bundle` | 14.188 us | 14.139 us - 14.244 us | Evaluates rational-turn trig, small-angle helpers, vector norms, product sums, and polynomials together. |
| `simple_new_function_surface/normal_bundle` | 30.824 us | 30.735 us - 30.936 us | Evaluates normal tails, log tails, interval mass, inverse tails, and moments together. |
| `simple_new_function_surface/scientific_bundle` | 12.973 us | 12.928 us - 13.022 us | Evaluates gamma, beta, regularized gamma/beta, and chi-square forms together. |
| `simple_new_function_surface/error_bundle` | 175.92 ns | 173.07 ns - 178.44 ns | Exercises fast domain failures for new public functions. |

<!-- END library_perf -->

<!-- BEGIN adversarial_transcendentals -->
## `adversarial_transcendentals`

Adversarial transcendental benchmarks for `hyperreal` trig, inverse trig, and inverse hyperbolic construction and approximation paths.

### `trig_adversarial_approx`

Cold approximation of sine, cosine, and tangent at exact, tiny, huge, and near-singular arguments.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `trig_adversarial_approx/sin_tiny_rational_p96` | 466.96 ns | 461.60 ns - 472.42 ns | Approximates sin(1e-12), stressing direct tiny-argument setup. |
| `trig_adversarial_approx/cos_tiny_rational_p96` | 511.41 ns | 503.08 ns - 522.21 ns | Approximates cos(1e-12), stressing direct tiny-argument setup. |
| `trig_adversarial_approx/tan_tiny_rational_p96` | 1.761 us | 1.757 us - 1.766 us | Approximates tan(1e-12), stressing direct tiny-argument setup. |
| `trig_adversarial_approx/sin_medium_rational_p96` | 1.631 us | 1.626 us - 1.637 us | Approximates sin(7/5), a moderate non-pi rational. |
| `trig_adversarial_approx/cos_medium_rational_p96` | 1.555 us | 1.529 us - 1.588 us | Approximates cos(7/5), a moderate non-pi rational. |
| `trig_adversarial_approx/tan_medium_rational_p96` | 6.134 us | 6.085 us - 6.198 us | Approximates tan(7/5), a moderate non-pi rational. |
| `trig_adversarial_approx/sin_f64_exact_p96` | 1.874 us | 1.846 us - 1.921 us | Approximates sin(1.23456789 imported as an exact dyadic rational). |
| `trig_adversarial_approx/cos_f64_exact_p96` | 1.755 us | 1.751 us - 1.759 us | Approximates cos(1.23456789 imported as an exact dyadic rational). |
| `trig_adversarial_approx/sin_1e6_p96` | 2.492 us | 2.455 us - 2.542 us | Approximates sin(1000000), stressing integer argument reduction. |
| `trig_adversarial_approx/cos_1e6_p96` | 2.449 us | 2.415 us - 2.491 us | Approximates cos(1000000), stressing integer argument reduction. |
| `trig_adversarial_approx/tan_1e6_p96` | 7.813 us | 7.779 us - 7.866 us | Approximates tan(1000000), stressing integer argument reduction. |
| `trig_adversarial_approx/sin_1e30_p96` | 2.284 us | 2.271 us - 2.300 us | Approximates sin(10^30), stressing very large integer reduction. |
| `trig_adversarial_approx/cos_1e30_p96` | 2.374 us | 2.347 us - 2.422 us | Approximates cos(10^30), stressing very large integer reduction. |
| `trig_adversarial_approx/tan_1e30_p96` | 6.769 us | 6.634 us - 6.928 us | Approximates tan(10^30), stressing very large integer reduction. |
| `trig_adversarial_approx/sin_huge_pi_plus_offset_p96` | 2.225 us | 2.178 us - 2.282 us | Approximates sin(2^512*pi + 7/5), stressing exact pi-multiple cancellation. |
| `trig_adversarial_approx/cos_huge_pi_plus_offset_p96` | 2.060 us | 2.034 us - 2.093 us | Approximates cos(2^512*pi + 7/5), stressing exact pi-multiple cancellation. |
| `trig_adversarial_approx/tan_huge_pi_plus_offset_p96` | 6.361 us | 6.313 us - 6.433 us | Approximates tan(2^512*pi + 7/5), stressing exact pi-multiple cancellation. |
| `trig_adversarial_approx/tan_near_half_pi_p96` | 20.641 us | 20.354 us - 20.968 us | Approximates tan(pi/2 - 2^-40), stressing the cotangent complement path. |
| `trig_adversarial_approx/tan_promoted_generated_604_125_p96` | 6.480 us | 6.454 us - 6.520 us | Promoted slow-performer tan(604/125), a generated top offender from the library-wide fuzz history. |

### `inverse_trig_adversarial_approx`

Cold approximation of asin, acos, and atan near exact values, zero, endpoints, and large atan inputs.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `inverse_trig_adversarial_approx/asin_zero_p96` | 39.25 ns | 38.89 ns - 39.63 ns | Approximates asin(0), which should collapse before the generic inverse-trig path. |
| `inverse_trig_adversarial_approx/acos_zero_p96` | 173.03 ns | 172.45 ns - 173.63 ns | Approximates acos(0), which should reduce to pi/2. |
| `inverse_trig_adversarial_approx/atan_zero_p96` | 137.36 ns | 136.22 ns - 139.04 ns | Approximates atan(0), which should collapse to zero. |
| `inverse_trig_adversarial_approx/asin_tiny_positive_p96` | 401.46 ns | 397.76 ns - 405.78 ns | Approximates asin(1e-12), stressing the tiny odd series. |
| `inverse_trig_adversarial_approx/acos_tiny_positive_p96` | 1.558 us | 1.535 us - 1.589 us | Approximates acos(1e-12), stressing pi/2 minus the tiny asin path. |
| `inverse_trig_adversarial_approx/atan_tiny_positive_p96` | 515.64 ns | 502.31 ns - 529.85 ns | Approximates atan(1e-12), stressing direct tiny atan setup. |
| `inverse_trig_adversarial_approx/asin_mid_positive_p96` | 6.557 us | 6.418 us - 6.711 us | Approximates asin(7/10), a generic in-domain value. |
| `inverse_trig_adversarial_approx/acos_mid_positive_p96` | 5.721 us | 5.656 us - 5.798 us | Approximates acos(7/10), a generic in-domain value. |
| `inverse_trig_adversarial_approx/atan_mid_positive_p96` | 2.140 us | 2.115 us - 2.165 us | Approximates atan(7/10), a generic in-domain value. |
| `inverse_trig_adversarial_approx/atan_two_thirds_anchor_sweep_p96` | 9.688 us | 9.566 us - 9.838 us | Approximates atan at 11/20, 3/5, 7/10, and 4/5, covering the two-thirds table-reduction interval. |
| `inverse_trig_adversarial_approx/atan_two_thirds_anchor_sweep_p32` | 5.813 us | 5.779 us - 5.847 us | Repeats the two-thirds table-reduction interval sweep at 32-bit precision. |
| `inverse_trig_adversarial_approx/atan_two_thirds_anchor_sweep_p256` | 20.274 us | 20.046 us - 20.597 us | Repeats the two-thirds table-reduction interval sweep at 256-bit precision. |
| `inverse_trig_adversarial_approx/atan_two_thirds_anchor_upper_edge_p96` | 2.776 us | 2.760 us - 2.794 us | Approximates atan(4/5), guarding the upper edge of the two-thirds table-reduction interval against a local regression. |
| `inverse_trig_adversarial_approx/asin_near_one_p96` | 1.923 us | 1.912 us - 1.934 us | Approximates asin(0.999999), stressing endpoint transforms. |
| `inverse_trig_adversarial_approx/acos_near_one_p96` | 1.573 us | 1.566 us - 1.582 us | Approximates acos(0.999999), stressing endpoint transforms. |
| `inverse_trig_adversarial_approx/asin_near_minus_one_p96` | 1.892 us | 1.885 us - 1.899 us | Approximates asin(-0.999999), stressing odd symmetry near the endpoint. |
| `inverse_trig_adversarial_approx/acos_near_minus_one_p96` | 1.683 us | 1.678 us - 1.687 us | Approximates acos(-0.999999), stressing negative endpoint transforms. |
| `inverse_trig_adversarial_approx/atan_large_p96` | 1.860 us | 1.854 us - 1.867 us | Approximates atan(8), stressing reciprocal reduction. |
| `inverse_trig_adversarial_approx/atan_promoted_generated_783_412_p96` | 1.916 us | 1.907 us - 1.927 us | Promoted slow-performer atan(783/412), the generated exact-rational atan top offender. |
| `inverse_trig_adversarial_approx/ln_square_plus_one_promoted_generated_677_222_p96` | 31.54 ns | 31.08 ns - 32.07 ns | Promoted slow-performer ln((677/222)^2 + 1), the generated exact-rational log top offender. |
| `inverse_trig_adversarial_approx/atan_huge_p96` | 924.52 ns | 901.88 ns - 947.06 ns | Approximates atan(10^30), stressing very large reciprocal reduction. |

### `trig_fuzz_adversarial_approx`

Deterministic broad sweeps of sine, cosine, and tangent over tiny, ordinary, huge, pi-offset, and near-pole exact inputs.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `trig_fuzz_adversarial_approx/sin_sweep_768_p96` | 1.602 ms | 1.588 ms - 1.624 ms | Approximates sin over 768 deterministic exact inputs spanning tiny, ordinary, huge, dyadic, rational, and pi-offset cases. |
| `trig_fuzz_adversarial_approx/cos_sweep_768_p96` | 1.623 ms | 1.615 ms - 1.633 ms | Approximates cos over the same 768-input deterministic fuzz sweep. |
| `trig_fuzz_adversarial_approx/tan_sweep_768_p96` | 4.526 ms | 4.503 ms - 4.552 ms | Approximates tan over the same deterministic sweep, including near-half-pi stress cases. |
| `trig_fuzz_adversarial_approx/sin_promoted_slow_candidates_p96` | 17.023 us | 16.884 us - 17.179 us | Approximates sin over promoted slow candidates found by prior sweep-style runs. |
| `trig_fuzz_adversarial_approx/cos_promoted_slow_candidates_p96` | 17.560 us | 17.492 us - 17.634 us | Approximates cos over promoted slow candidates found by prior sweep-style runs. |
| `trig_fuzz_adversarial_approx/tan_promoted_slow_candidates_p96` | 77.763 us | 76.871 us - 79.152 us | Approximates tan over promoted near-pole and large-reduction slow candidates. |

### `promoted_library_slow_offenders_approx`

One hundred structurally varied worst offenders promoted from the library-wide slow-performer history.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `promoted_library_slow_offenders_approx/promoted_100_structural_slow_offenders_p96` | not run | not run | Approximates 100 individual promoted slow cases spanning ln(1+x^2), atan, tan, sin, and cos over varied exact-rational structures. |

### `inverse_hyperbolic_adversarial_approx`

Cold approximation of inverse hyperbolic functions at tiny, moderate, large, and endpoint-adjacent arguments.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `inverse_hyperbolic_adversarial_approx/asinh_tiny_positive_p128` | 550.40 ns | 543.80 ns - 560.72 ns | Approximates asinh(1e-12), stressing cancellation avoidance near zero. |
| `inverse_hyperbolic_adversarial_approx/asinh_mid_positive_p128` | 10.283 us | 10.069 us - 10.521 us | Approximates asinh(1/2), a moderate positive value. |
| `inverse_hyperbolic_adversarial_approx/asinh_large_positive_p128` | 5.826 us | 5.772 us - 5.888 us | Approximates asinh(10^6), stressing large-input logarithmic behavior. |
| `inverse_hyperbolic_adversarial_approx/asinh_large_negative_p128` | 6.304 us | 6.171 us - 6.486 us | Approximates asinh(-10^6), stressing odd symmetry for large inputs. |
| `inverse_hyperbolic_adversarial_approx/acosh_one_plus_tiny_p128` | 3.978 us | 3.965 us - 3.990 us | Approximates acosh(1 + 1e-12), stressing the near-one endpoint. |
| `inverse_hyperbolic_adversarial_approx/acosh_sqrt_two_p128` | 86.79 ns | 86.48 ns - 87.18 ns | Approximates acosh(sqrt(2)), a symbolic square-root input. |
| `inverse_hyperbolic_adversarial_approx/acosh_two_p128` | 47.93 ns | 47.83 ns - 48.03 ns | Approximates acosh(2), a moderate exact rational. |
| `inverse_hyperbolic_adversarial_approx/acosh_large_positive_p128` | 5.970 us | 5.754 us - 6.240 us | Approximates acosh(10^6), stressing large-input logarithmic behavior. |
| `inverse_hyperbolic_adversarial_approx/atanh_tiny_positive_p128` | 506.19 ns | 503.35 ns - 510.09 ns | Approximates atanh(1e-12), stressing the tiny odd series. |
| `inverse_hyperbolic_adversarial_approx/atanh_mid_positive_p128` | 198.68 ns | 196.92 ns - 200.67 ns | Approximates atanh(1/2), a moderate exact rational. |
| `inverse_hyperbolic_adversarial_approx/atanh_near_one_p128` | 3.008 us | 3.000 us - 3.017 us | Approximates atanh(0.999999), stressing endpoint logarithmic behavior. |
| `inverse_hyperbolic_adversarial_approx/atanh_near_minus_one_p128` | 3.246 us | 3.171 us - 3.339 us | Approximates atanh(-0.999999), stressing odd symmetry near the endpoint. |

### `real_shortcut_adversarial`

Public `Real` construction shortcuts and domain checks for the same transcendental families.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_shortcut_adversarial/sin_exact_pi_over_six` | 472.57 ns | 469.04 ns - 478.15 ns | Constructs sin(pi/6), which should return the exact rational 1/2. |
| `real_shortcut_adversarial/cos_exact_pi_over_three` | 428.94 ns | 426.47 ns - 431.77 ns | Constructs cos(pi/3), which should return the exact rational 1/2. |
| `real_shortcut_adversarial/tan_exact_pi_over_four` | 24.25 ns | 23.94 ns - 24.57 ns | Constructs tan(pi/4), which should return the exact rational 1. |
| `real_shortcut_adversarial/asin_exact_half` | 32.12 ns | 30.72 ns - 33.55 ns | Constructs asin(1/2), which should return pi/6. |
| `real_shortcut_adversarial/acos_exact_half` | 30.81 ns | 29.90 ns - 31.81 ns | Constructs acos(1/2), which should return pi/3. |
| `real_shortcut_adversarial/atan_exact_one` | 25.82 ns | 24.97 ns - 26.88 ns | Constructs atan(1), which should return pi/4. |
| `real_shortcut_adversarial/asin_domain_error` | 24.94 ns | 24.43 ns - 25.44 ns | Rejects asin(1 + 1e-12). |
| `real_shortcut_adversarial/acos_domain_error` | 22.85 ns | 22.11 ns - 23.70 ns | Rejects acos(1 + 1e-12). |
| `real_shortcut_adversarial/atanh_endpoint_infinity` | 10.49 ns | 10.30 ns - 10.73 ns | Rejects atanh(1) as an infinite endpoint. |
| `real_shortcut_adversarial/atanh_domain_error` | 10.89 ns | 10.87 ns - 10.90 ns | Rejects atanh(1 + 1e-12). |
| `real_shortcut_adversarial/acosh_domain_error` | 9.38 ns | 9.19 ns - 9.61 ns | Rejects acosh(1 - 1e-12). |

<!-- END adversarial_transcendentals -->

<!-- BEGIN borrowed_ops -->
## `borrowed_ops`

Compares owned arithmetic with borrowed arithmetic for exact and irrational values.

### `rational_ops`

Owned versus borrowed arithmetic for exact `Rational` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `rational_ops/add_owned` | 10.78 ns | 10.73 ns - 10.84 ns | Adds cloned owned operands. |
| `rational_ops/add_refs` | 9.33 ns | 9.25 ns - 9.42 ns | Adds borrowed operands without cloning both inputs. |
| `rational_ops/sub_owned` | 10.56 ns | 10.52 ns - 10.60 ns | Subtracts cloned owned operands. |
| `rational_ops/sub_refs` | 9.04 ns | 9.01 ns - 9.06 ns | Subtracts borrowed operands. |
| `rational_ops/mul_owned` | 24.44 ns | 24.38 ns - 24.52 ns | Multiplies cloned owned operands. |
| `rational_ops/mul_refs` | 23.06 ns | 22.95 ns - 23.18 ns | Multiplies borrowed operands. |
| `rational_ops/div_owned` | 182.78 ns | 181.90 ns - 183.76 ns | Divides cloned owned operands. |
| `rational_ops/div_refs` | 163.44 ns | 162.91 ns - 164.03 ns | Divides borrowed operands. |

### `real_ops`

Owned versus borrowed arithmetic for exact rational-backed `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_ops/add_owned` | 20.69 ns | 20.62 ns - 20.76 ns | Adds cloned owned operands. |
| `real_ops/add_refs` | 17.78 ns | 17.75 ns - 17.81 ns | Adds borrowed operands without cloning both inputs. |
| `real_ops/sub_owned` | 23.03 ns | 22.97 ns - 23.12 ns | Subtracts cloned owned operands. |
| `real_ops/sub_refs` | 17.94 ns | 17.86 ns - 18.05 ns | Subtracts borrowed operands. |
| `real_ops/mul_owned` | 34.19 ns | 33.99 ns - 34.40 ns | Multiplies cloned owned operands. |
| `real_ops/mul_refs` | 31.11 ns | 30.89 ns - 31.35 ns | Multiplies borrowed operands. |
| `real_ops/div_owned` | 193.04 ns | 192.28 ns - 193.97 ns | Divides cloned owned operands. |
| `real_ops/div_refs` | 183.46 ns | 183.26 ns - 183.67 ns | Divides borrowed operands. |

### `real_irrational_ops`

Owned versus borrowed arithmetic for symbolic irrational `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_irrational_ops/add_owned` | 67.05 ns | 66.31 ns - 67.85 ns | Adds cloned owned operands. |
| `real_irrational_ops/add_refs` | 57.81 ns | 57.53 ns - 58.20 ns | Adds borrowed operands without cloning both inputs. |
| `real_irrational_ops/sub_owned` | 106.58 ns | 106.01 ns - 107.19 ns | Subtracts cloned owned operands. |
| `real_irrational_ops/sub_refs` | 96.00 ns | 95.74 ns - 96.35 ns | Subtracts borrowed operands. |
| `real_irrational_ops/mul_owned` | 253.08 ns | 252.30 ns - 254.21 ns | Multiplies cloned owned operands. |
| `real_irrational_ops/mul_refs` | 223.71 ns | 222.80 ns - 224.75 ns | Multiplies borrowed operands. |
| `real_irrational_ops/div_owned` | 48.20 ns | 48.01 ns - 48.41 ns | Divides cloned owned operands. |
| `real_irrational_ops/div_refs` | 41.73 ns | 41.69 ns - 41.78 ns | Divides borrowed operands. |

<!-- END borrowed_ops -->

<!-- BEGIN float_convert -->
## `float_convert`

Covers exact import of floating-point values, including public `Real` conversion overhead.

### `float_convert`

Exact IEEE-754 imports and finite outward binary64 enclosures of rationals.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `float_convert/f32_normal` | 46.17 ns | 45.91 ns - 46.45 ns | Converts a normal `f32` into an exact `Rational`. |
| `float_convert/f64_normal` | 46.35 ns | 46.23 ns - 46.52 ns | Converts a normal `f64` into an exact `Rational`. |
| `float_convert/f64_binary_fraction` | 9.99 ns | 9.97 ns - 10.03 ns | Converts an exactly representable binary `f64` fraction into `Rational`. |
| `float_convert/f64_subnormal` | 54.41 ns | 53.98 ns - 54.91 ns | Converts a subnormal `f64` into an exact `Rational`. |
| `float_convert/real_f32_normal` | 65.22 ns | 65.15 ns - 65.28 ns | Converts a normal `f32` through the public `Real::try_from` path. |
| `float_convert/real_f64_normal` | 67.22 ns | 67.00 ns - 67.50 ns | Converts a normal `f64` through the public `Real::try_from` path. |
| `float_convert/real_f64_binary_fraction` | 22.05 ns | 22.01 ns - 22.11 ns | Converts an exactly representable binary `f64` fraction through the public `Real::try_from` path. |
| `float_convert/real_f64_subnormal` | 71.37 ns | 71.21 ns - 71.56 ns | Converts a subnormal `f64` through the public `Real::try_from` path. |
| `float_convert/f64_enclosure_exact` | 7.33 ns | 7.32 ns - 7.35 ns | Exports an exactly representable dyadic as a finite binary64 singleton. |
| `float_convert/f64_enclosure_rounded` | 10.23 ns | 10.20 ns - 10.28 ns | Exports a dyadic requiring outward binary64 rounding. |
| `float_convert/f64_enclosure_near_max` | 13.26 ns | 13.23 ns - 13.29 ns | Exports an exact value just below binary64 MAX without an infinite endpoint. |

<!-- END float_convert -->

<!-- BEGIN COMPLETE BENCHMARK REPORT -->
## Complete generated benchmark report

Every registered benchmark target is catalogued below. Every Criterion result found under `target/criterion` is included without a name or implementation filter; non-Criterion targets write their own linked reports. Each timing binary refreshes this section after it runs.

Run the complete non-instrumented timing set with:

```sh
cargo bench --features simple
```

Regenerate this Markdown from stored Criterion data without rerunning benchmarks:

```sh
cargo run --example write_benchmarks_md
```

### Registered benchmark suites

| Target | Kind | Required features | Command | Generated report |
| --- | --- | --- | --- | --- |
| `adversarial_library` | Criterion timing | `default` | `cargo bench --bench adversarial_library` | this file |
| `adversarial_transcendentals` | Criterion timing | `default` | `cargo bench --bench adversarial_transcendentals` | this file |
| `borrowed_ops` | Criterion timing | `default` | `cargo bench --bench borrowed_ops` | this file |
| `dispatch_trace` | diagnostic | `dispatch-trace` | `cargo bench --bench dispatch_trace --features dispatch-trace` | [dispatch_trace.md](dispatch_trace.md) |
| `float_convert` | Criterion timing | `default` | `cargo bench --bench float_convert` | this file |
| `gmp_api` | Criterion timing | `default` | `cargo bench --bench gmp_api` | this file |
| `library_perf` | Criterion timing | `simple` | `cargo bench --bench library_perf --features simple` | this file |
| `numerical_micro` | Criterion timing | `default` | `cargo bench --bench numerical_micro` | this file |
| `real_representations` | Criterion timing | `default` | `cargo bench --bench real_representations` | this file |
| `scalar_micro` | Criterion timing | `default` | `cargo bench --bench scalar_micro` | this file |

### Comparative results

Rows sharing a Criterion group and input are compared when they expose distinct implementations. Ratios are elapsed time relative to the fastest stored row; they do not imply identical guarantees or output semantics.

| Group | Input | Implementation | Mean | Relative to fastest |
| --- | --- | --- | ---: | ---: |
| `gmp_computable_api_p128` | `acos` | `gmp_mpfr128` | 3.24 us | 1.00x |
| `gmp_computable_api_p128` | `acos` | `hyperreal` | 6.36 us | 1.97x |
| `gmp_computable_api_p128` | `acosh` | `gmp_mpfr128` | 1.69 us | 1.00x |
| `gmp_computable_api_p128` | `acosh` | `hyperreal` | 12.87 us | 7.60x |
| `gmp_computable_api_p128` | `add` | `gmp_mpfr128` | 29.82 ns | 1.00x |
| `gmp_computable_api_p128` | `add` | `hyperreal` | 133.92 ns | 4.49x |
| `gmp_computable_api_p128` | `asin` | `gmp_mpfr128` | 3.25 us | 1.00x |
| `gmp_computable_api_p128` | `asin` | `hyperreal` | 6.56 us | 2.02x |
| `gmp_computable_api_p128` | `asinh` | `gmp_mpfr128` | 1.74 us | 1.00x |
| `gmp_computable_api_p128` | `asinh` | `hyperreal` | 6.20 us | 3.57x |
| `gmp_computable_api_p128` | `atan` | `hyperreal` | 2.74 us | 1.00x |
| `gmp_computable_api_p128` | `atan` | `gmp_mpfr128` | 2.85 us | 1.04x |
| `gmp_computable_api_p128` | `atan2` | `gmp_mpfr128` | 2.53 us | 1.00x |
| `gmp_computable_api_p128` | `atan2` | `hyperreal` | 6.24 us | 2.46x |
| `gmp_computable_api_p128` | `atanh` | `hyperreal` | 460.38 ns | 1.00x |
| `gmp_computable_api_p128` | `atanh` | `gmp_mpfr128` | 1.70 us | 3.68x |
| `gmp_computable_api_p128` | `compare_absolute` | `gmp_mpfr128` | 2.93 ns | 1.00x |
| `gmp_computable_api_p128` | `compare_absolute` | `hyperreal` | 7.62 ns | 2.60x |
| `gmp_computable_api_p128` | `cos` | `gmp_mpfr128` | 513.91 ns | 1.00x |
| `gmp_computable_api_p128` | `cos` | `hyperreal` | 2.12 us | 4.13x |
| `gmp_computable_api_p128` | `dnorm` | `gmp_mpfr128` | 1.21 us | 1.00x |
| `gmp_computable_api_p128` | `dnorm` | `hyperreal` | 7.44 us | 6.14x |
| `gmp_computable_api_p128` | `e` | `gmp_mpfr128` | 17.98 ns | 1.00x |
| `gmp_computable_api_p128` | `e` | `hyperreal` | 20.04 ns | 1.11x |
| `gmp_computable_api_p128` | `erf` | `gmp_mpfr128` | 3.34 us | 1.00x |
| `gmp_computable_api_p128` | `erf` | `hyperreal` | 31.16 us | 9.33x |
| `gmp_computable_api_p128` | `erfc` | `gmp_mpfr128` | 3.70 us | 1.00x |
| `gmp_computable_api_p128` | `erfc` | `hyperreal` | 32.56 us | 8.81x |
| `gmp_computable_api_p128` | `erfcx` | `gmp_mpfr128` | 4.76 us | 1.00x |
| `gmp_computable_api_p128` | `erfcx` | `hyperreal` | 66.96 us | 14.06x |
| `gmp_computable_api_p128` | `exp` | `gmp_mpfr128` | 925.33 ns | 1.00x |
| `gmp_computable_api_p128` | `exp` | `hyperreal` | 4.41 us | 4.77x |
| `gmp_computable_api_p128` | `expm1` | `gmp_mpfr128` | 1.05 us | 1.00x |
| `gmp_computable_api_p128` | `expm1` | `hyperreal` | 4.78 us | 4.57x |
| `gmp_computable_api_p128` | `inverse` | `gmp_mpfr128` | 60.16 ns | 1.00x |
| `gmp_computable_api_p128` | `inverse` | `hyperreal` | 127.52 ns | 2.12x |
| `gmp_computable_api_p128` | `ln` | `hyperreal` | 1.19 us | 1.00x |
| `gmp_computable_api_p128` | `ln` | `gmp_mpfr128` | 1.33 us | 1.12x |
| `gmp_computable_api_p128` | `log_dnorm` | `gmp_mpfr128` | 2.59 us | 1.00x |
| `gmp_computable_api_p128` | `log_dnorm` | `hyperreal` | 6.87 us | 2.65x |
| `gmp_computable_api_p128` | `log_normal_sf` | `gmp_mpfr128` | 5.68 us | 1.00x |
| `gmp_computable_api_p128` | `log_normal_sf` | `hyperreal` | 85.44 us | 15.05x |
| `gmp_computable_api_p128` | `log_pnorm` | `gmp_mpfr128` | 5.74 us | 1.00x |
| `gmp_computable_api_p128` | `log_pnorm` | `hyperreal` | 51.79 us | 9.02x |
| `gmp_computable_api_p128` | `multiply` | `gmp_mpfr128` | 37.47 ns | 1.00x |
| `gmp_computable_api_p128` | `multiply` | `hyperreal` | 145.23 ns | 3.88x |
| `gmp_computable_api_p128` | `negate` | `gmp_mpfr128` | 13.36 ns | 1.00x |
| `gmp_computable_api_p128` | `negate` | `hyperreal` | 112.28 ns | 8.41x |
| `gmp_computable_api_p128` | `normal_interval` | `gmp_mpfr128` | 7.98 us | 1.00x |
| `gmp_computable_api_p128` | `normal_interval` | `hyperreal` | 73.81 us | 9.25x |
| `gmp_computable_api_p128` | `normal_quantile` | `gmp_mpfr128` | 46.96 us | 1.00x |
| `gmp_computable_api_p128` | `normal_quantile` | `hyperreal` | 271.53 us | 5.78x |
| `gmp_computable_api_p128` | `normal_sf` | `gmp_mpfr128` | 3.26 us | 1.00x |
| `gmp_computable_api_p128` | `normal_sf` | `hyperreal` | 30.56 us | 9.38x |
| `gmp_computable_api_p128` | `pi` | `gmp_mpfr128` | 17.87 ns | 1.00x |
| `gmp_computable_api_p128` | `pi` | `hyperreal` | 58.64 ns | 3.28x |
| `gmp_computable_api_p128` | `pnorm` | `gmp_mpfr128` | 3.20 us | 1.00x |
| `gmp_computable_api_p128` | `pnorm` | `hyperreal` | 31.14 us | 9.72x |
| `gmp_computable_api_p128` | `sign_until` | `gmp_mpfr128` | 0.49 ns | 1.00x |
| `gmp_computable_api_p128` | `sign_until` | `hyperreal` | 4.08 ns | 8.37x |
| `gmp_computable_api_p128` | `sign_until_floor_2000` | `gmp_mpfr128` | 0.48 ns | 1.00x |
| `gmp_computable_api_p128` | `sign_until_floor_2000` | `hyperreal` | 4.08 ns | 8.52x |
| `gmp_computable_api_p128` | `sin` | `gmp_mpfr128` | 736.29 ns | 1.00x |
| `gmp_computable_api_p128` | `sin` | `hyperreal` | 2.11 us | 2.86x |
| `gmp_computable_api_p128` | `sqrt` | `gmp_mpfr128` | 105.46 ns | 1.00x |
| `gmp_computable_api_p128` | `sqrt` | `hyperreal` | 322.92 ns | 3.06x |
| `gmp_computable_api_p128` | `square` | `gmp_mpfr128` | 40.09 ns | 1.00x |
| `gmp_computable_api_p128` | `square` | `hyperreal` | 121.01 ns | 3.02x |
| `gmp_computable_api_p128` | `tan` | `gmp_mpfr128` | 991.94 ns | 1.00x |
| `gmp_computable_api_p128` | `tan` | `hyperreal` | 8.73 us | 8.81x |
| `gmp_computable_api_p128` | `tau` | `gmp_mpfr128` | 17.97 ns | 1.00x |
| `gmp_computable_api_p128` | `tau` | `hyperreal` | 20.68 ns | 1.15x |
| `gmp_computable_api_p128` | `try_compare_to` | `gmp_mpfr128` | 3.07 ns | 1.00x |
| `gmp_computable_api_p128` | `try_compare_to` | `hyperreal` | 17.43 ns | 5.68x |
| `gmp_computable_api_p128` | `zero_status` | `gmp_mpfr128` | 1.91 ns | 1.00x |
| `gmp_computable_api_p128` | `zero_status` | `hyperreal` | 2.66 ns | 1.40x |
| `gmp_magnitude_algorithms` | `16384` | `num_bigint_mul` | 9.58 us | 1.00x |
| `gmp_magnitude_algorithms` | `16384` | `gmp_mul` | 13.34 us | 1.39x |
| `gmp_magnitude_algorithms` | `16384` | `gmp_roundtrip_mul` | 29.81 us | 3.11x |
| `gmp_magnitude_algorithms` | `16384` | `gmp_div` | 31.54 us | 3.29x |
| `gmp_magnitude_algorithms` | `16384` | `gmp_roundtrip_div` | 48.14 us | 5.02x |
| `gmp_magnitude_algorithms` | `16384` | `num_bigint_div` | 73.08 us | 7.63x |
| `gmp_magnitude_algorithms` | `4096` | `gmp_mul` | 1.52 us | 1.00x |
| `gmp_magnitude_algorithms` | `4096` | `num_bigint_mul` | 2.16 us | 1.42x |
| `gmp_magnitude_algorithms` | `4096` | `gmp_div` | 2.66 us | 1.75x |
| `gmp_magnitude_algorithms` | `4096` | `num_bigint_div` | 4.92 us | 3.24x |
| `gmp_magnitude_algorithms` | `4096` | `gmp_roundtrip_mul` | 5.59 us | 3.67x |
| `gmp_magnitude_algorithms` | `4096` | `gmp_roundtrip_div` | 6.86 us | 4.51x |
| `gmp_magnitude_algorithms` | `65536` | `gmp_mul` | 81.81 us | 1.00x |
| `gmp_magnitude_algorithms` | `65536` | `num_bigint_mul` | 116.07 us | 1.42x |
| `gmp_magnitude_algorithms` | `65536` | `gmp_roundtrip_mul` | 144.76 us | 1.77x |
| `gmp_magnitude_algorithms` | `65536` | `gmp_div` | 250.49 us | 3.06x |
| `gmp_magnitude_algorithms` | `65536` | `gmp_roundtrip_div` | 314.85 us | 3.85x |
| `gmp_magnitude_algorithms` | `65536` | `num_bigint_div` | 1.12 ms | 13.73x |
| `gmp_rational_api` | `add` | `gmp` | 0.94 ns | 1.00x |
| `gmp_rational_api` | `add` | `hyperreal` | 6.16 ns | 6.57x |
| `gmp_rational_api` | `average_pair` | `hyperreal` | 79.58 ns | 1.00x |
| `gmp_rational_api` | `average_pair` | `gmp` | 81.95 ns | 1.03x |
| `gmp_rational_api` | `complex_product` | `hyperreal` | 203.94 ns | 1.00x |
| `gmp_rational_api` | `complex_product` | `gmp` | 398.44 ns | 1.95x |
| `gmp_rational_api` | `complex_quotient` | `hyperreal` | 213.01 ns | 1.00x |
| `gmp_rational_api` | `complex_quotient` | `gmp` | 733.55 ns | 3.44x |
| `gmp_rational_api` | `denominator` | `hyperreal` | 10.98 ns | 1.00x |
| `gmp_rational_api` | `denominator` | `gmp` | 11.93 ns | 1.09x |
| `gmp_rational_api` | `div` | `gmp` | 0.94 ns | 1.00x |
| `gmp_rational_api` | `div` | `hyperreal` | 92.80 ns | 99.17x |
| `gmp_rational_api` | `dot2` | `hyperreal` | 99.68 ns | 1.00x |
| `gmp_rational_api` | `dot2` | `gmp` | 161.65 ns | 1.62x |
| `gmp_rational_api` | `extract_square_reduced` | `gmp` | 73.77 ns | 1.00x |
| `gmp_rational_api` | `extract_square_reduced` | `hyperreal` | 121.98 ns | 1.65x |
| `gmp_rational_api` | `extract_square_will_succeed` | `hyperreal` | 3.24 ns | 1.00x |
| `gmp_rational_api` | `extract_square_will_succeed` | `gmp` | 78.84 ns | 24.36x |
| `gmp_rational_api` | `fract` | `hyperreal` | 34.14 ns | 1.00x |
| `gmp_rational_api` | `fract` | `gmp` | 108.90 ns | 3.19x |
| `gmp_rational_api` | `from_bigint` | `hyperreal` | 30.18 ns | 1.00x |
| `gmp_rational_api` | `from_bigint` | `gmp` | 50.52 ns | 1.67x |
| `gmp_rational_api` | `from_bigint_fraction` | `hyperreal` | 58.89 ns | 1.00x |
| `gmp_rational_api` | `from_bigint_fraction` | `gmp` | 129.72 ns | 2.20x |
| `gmp_rational_api` | `from_fraction` | `gmp` | 36.33 ns | 1.00x |
| `gmp_rational_api` | `from_fraction` | `hyperreal` | 68.97 ns | 1.90x |
| `gmp_rational_api` | `from_integer` | `hyperreal` | 3.62 ns | 1.00x |
| `gmp_rational_api` | `from_integer` | `gmp` | 17.05 ns | 4.71x |
| `gmp_rational_api` | `inverse` | `hyperreal` | 7.46 ns | 1.00x |
| `gmp_rational_api` | `inverse` | `gmp` | 30.02 ns | 4.02x |
| `gmp_rational_api` | `is_dyadic` | `hyperreal` | 1.80 ns | 1.00x |
| `gmp_rational_api` | `is_dyadic` | `gmp` | 5.00 ns | 2.78x |
| `gmp_rational_api` | `is_integer` | `gmp` | 0.47 ns | 1.00x |
| `gmp_rational_api` | `is_integer` | `hyperreal` | 4.24 ns | 9.03x |
| `gmp_rational_api` | `is_negative` | `hyperreal` | 0.47 ns | 1.00x |
| `gmp_rational_api` | `is_negative` | `gmp` | 2.17 ns | 4.63x |
| `gmp_rational_api` | `is_one` | `hyperreal` | 2.87 ns | 1.00x |
| `gmp_rational_api` | `is_one` | `gmp` | 8.35 ns | 2.91x |
| `gmp_rational_api` | `is_perfect_power` | `gmp` | 107.12 ns | 1.00x |
| `gmp_rational_api` | `is_perfect_power` | `hyperreal` | 266.23 ns | 2.49x |
| `gmp_rational_api` | `is_positive` | `hyperreal` | 0.48 ns | 1.00x |
| `gmp_rational_api` | `is_positive` | `gmp` | 2.39 ns | 4.99x |
| `gmp_rational_api` | `is_zero` | `hyperreal` | 0.47 ns | 1.00x |
| `gmp_rational_api` | `is_zero` | `gmp` | 2.39 ns | 5.05x |
| `gmp_rational_api` | `mean3_refs` | `gmp` | 129.29 ns | 1.00x |
| `gmp_rational_api` | `mean3_refs` | `hyperreal` | 207.61 ns | 1.61x |
| `gmp_rational_api` | `mean_refs` | `hyperreal` | 129.61 ns | 1.00x |
| `gmp_rational_api` | `mean_refs` | `gmp` | 238.15 ns | 1.84x |
| `gmp_rational_api` | `mul` | `gmp` | 0.95 ns | 1.00x |
| `gmp_rational_api` | `mul` | `hyperreal` | 11.92 ns | 12.56x |
| `gmp_rational_api` | `neg` | `gmp` | 0.47 ns | 1.00x |
| `gmp_rational_api` | `neg` | `hyperreal` | 8.91 ns | 19.05x |
| `gmp_rational_api` | `numerator` | `hyperreal` | 11.05 ns | 1.00x |
| `gmp_rational_api` | `numerator` | `gmp` | 12.00 ns | 1.09x |
| `gmp_rational_api` | `one` | `hyperreal` | 3.06 ns | 1.00x |
| `gmp_rational_api` | `one` | `gmp` | 17.24 ns | 5.63x |
| `gmp_rational_api` | `ordering` | `gmp` | 2.35 ns | 1.00x |
| `gmp_rational_api` | `ordering` | `hyperreal` | 3.58 ns | 1.52x |
| `gmp_rational_api` | `ordering_wide_dyadic` | `hyperreal` | 13.92 ns | 1.00x |
| `gmp_rational_api` | `ordering_wide_dyadic` | `gmp` | 38.67 ns | 2.78x |
| `gmp_rational_api` | `perfect_nth_root` | `gmp` | 106.79 ns | 1.00x |
| `gmp_rational_api` | `perfect_nth_root` | `hyperreal` | 163.13 ns | 1.53x |
| `gmp_rational_api` | `powi_17` | `gmp` | 101.43 ns | 1.00x |
| `gmp_rational_api` | `powi_17` | `hyperreal` | 307.39 ns | 3.03x |
| `gmp_rational_api` | `same_denominator` | `gmp` | 35.98 ns | 1.00x |
| `gmp_rational_api` | `same_denominator` | `hyperreal` | 71.59 ns | 1.99x |
| `gmp_rational_api` | `shifted_big_integer` | `hyperreal` | 49.32 ns | 1.00x |
| `gmp_rational_api` | `shifted_big_integer` | `gmp` | 75.53 ns | 1.53x |
| `gmp_rational_api` | `sign` | `gmp` | 0.47 ns | 1.00x |
| `gmp_rational_api` | `sign` | `hyperreal` | 0.47 ns | 1.01x |
| `gmp_rational_api` | `signed_product_sum` | `hyperreal` | 109.30 ns | 1.00x |
| `gmp_rational_api` | `signed_product_sum` | `gmp` | 217.51 ns | 1.99x |
| `gmp_rational_api` | `signed_product_sum_ordering` | `hyperreal` | 36.02 ns | 1.00x |
| `gmp_rational_api` | `signed_product_sum_ordering` | `gmp` | 212.32 ns | 5.90x |
| `gmp_rational_api` | `signed_product_sum_shared_denominator` | `hyperreal` | 100.61 ns | 1.00x |
| `gmp_rational_api` | `signed_product_sum_shared_denominator` | `gmp` | 216.24 ns | 2.15x |
| `gmp_rational_api` | `sub` | `gmp` | 0.94 ns | 1.00x |
| `gmp_rational_api` | `sub` | `hyperreal` | 7.63 ns | 8.14x |
| `gmp_rational_api` | `to_f64` | `hyperreal` | 3.34 ns | 1.00x |
| `gmp_rational_api` | `to_f64` | `gmp` | 23.98 ns | 7.17x |
| `gmp_rational_api` | `to_f64_enclosure` | `hyperreal` | 20.08 ns | 1.00x |
| `gmp_rational_api` | `to_f64_enclosure` | `gmp` | 128.23 ns | 6.39x |
| `gmp_rational_api` | `to_integer` | `gmp` | 1.19 ns | 1.00x |
| `gmp_rational_api` | `to_integer` | `hyperreal` | 4.66 ns | 3.91x |
| `gmp_rational_api` | `trunc` | `gmp` | 50.11 ns | 1.00x |
| `gmp_rational_api` | `trunc` | `hyperreal` | 51.19 ns | 1.02x |
| `gmp_rational_api` | `zero` | `hyperreal` | 3.82 ns | 1.00x |
| `gmp_rational_api` | `zero` | `gmp` | 11.10 ns | 2.90x |
| `gmp_real_arithmetic_api` | `abs` | `gmp_mpfr128` | 26.88 ns | 1.00x |
| `gmp_real_arithmetic_api` | `abs` | `hyperreal` | 31.45 ns | 1.17x |
| `gmp_real_arithmetic_api` | `add` | `hyperreal` | 34.30 ns | 1.00x |
| `gmp_real_arithmetic_api` | `add` | `gmp_mpfr128` | 44.67 ns | 1.30x |
| `gmp_real_arithmetic_api` | `ceil` | `gmp_mpfr128` | 45.57 ns | 1.00x |
| `gmp_real_arithmetic_api` | `ceil` | `hyperreal` | 167.87 ns | 3.68x |
| `gmp_real_arithmetic_api` | `div` | `hyperreal` | 62.36 ns | 1.00x |
| `gmp_real_arithmetic_api` | `div` | `gmp_mpfr128` | 90.51 ns | 1.45x |
| `gmp_real_arithmetic_api` | `floor` | `gmp_mpfr128` | 44.49 ns | 1.00x |
| `gmp_real_arithmetic_api` | `floor` | `hyperreal` | 113.00 ns | 2.54x |
| `gmp_real_arithmetic_api` | `fract` | `gmp_mpfr128` | 36.62 ns | 1.00x |
| `gmp_real_arithmetic_api` | `fract` | `hyperreal` | 159.79 ns | 4.36x |
| `gmp_real_arithmetic_api` | `inverse` | `hyperreal` | 25.97 ns | 1.00x |
| `gmp_real_arithmetic_api` | `inverse` | `gmp_mpfr128` | 61.37 ns | 2.36x |
| `gmp_real_arithmetic_api` | `mul` | `hyperreal` | 36.84 ns | 1.00x |
| `gmp_real_arithmetic_api` | `mul` | `gmp_mpfr128` | 58.89 ns | 1.60x |
| `gmp_real_arithmetic_api` | `neg` | `gmp_mpfr128` | 17.43 ns | 1.00x |
| `gmp_real_arithmetic_api` | `neg` | `hyperreal` | 21.80 ns | 1.25x |
| `gmp_real_arithmetic_api` | `pow` | `hyperreal` | 421.82 ns | 1.00x |
| `gmp_real_arithmetic_api` | `pow` | `gmp_mpfr128` | 2.94 us | 6.97x |
| `gmp_real_arithmetic_api` | `powi_17` | `hyperreal` | 81.65 ns | 1.00x |
| `gmp_real_arithmetic_api` | `powi_17` | `gmp_mpfr128` | 143.35 ns | 1.76x |
| `gmp_real_arithmetic_api` | `rem_euclid` | `gmp_mpfr128` | 154.23 ns | 1.00x |
| `gmp_real_arithmetic_api` | `rem_euclid` | `hyperreal` | 320.21 ns | 2.08x |
| `gmp_real_arithmetic_api` | `round` | `gmp_mpfr128` | 45.54 ns | 1.00x |
| `gmp_real_arithmetic_api` | `round` | `hyperreal` | 225.08 ns | 4.94x |
| `gmp_real_arithmetic_api` | `sub` | `hyperreal` | 34.35 ns | 1.00x |
| `gmp_real_arithmetic_api` | `sub` | `gmp_mpfr128` | 47.00 ns | 1.37x |
| `gmp_real_arithmetic_api` | `to_degrees` | `gmp_mpfr128` | 88.74 ns | 1.00x |
| `gmp_real_arithmetic_api` | `to_degrees` | `hyperreal` | 251.56 ns | 2.83x |
| `gmp_real_arithmetic_api` | `to_radians` | `gmp_mpfr128` | 85.26 ns | 1.00x |
| `gmp_real_arithmetic_api` | `to_radians` | `hyperreal` | 257.71 ns | 3.02x |
| `gmp_real_arithmetic_api` | `trunc` | `gmp_mpfr128` | 44.66 ns | 1.00x |
| `gmp_real_arithmetic_api` | `trunc` | `hyperreal` | 110.69 ns | 2.48x |
| `gmp_real_collection_and_conversion_api` | `affine` | `hyperreal` | 41.46 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `affine` | `gmp_mpfr128` | 60.62 ns | 1.46x |
| `gmp_real_collection_and_conversion_api` | `e` | `hyperreal` | 20.40 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `e` | `gmp_mpfr128` | 1.05 us | 51.40x |
| `gmp_real_collection_and_conversion_api` | `exact_rational_interpolate_point3_known_dyadic` | `gmp_mpfr128` | 220.83 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `exact_rational_interpolate_point3_known_dyadic` | `hyperreal` | 291.59 ns | 1.32x |
| `gmp_real_collection_and_conversion_api` | `exact_rational_line_intersection2_known_dyadic` | `hyperreal` | 480.25 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `exact_rational_line_intersection2_known_dyadic` | `gmp_mpfr128` | 602.98 ns | 1.26x |
| `gmp_real_collection_and_conversion_api` | `exact_rational_line_intersection2_point_known_exact` | `gmp_mpfr128` | 471.86 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `exact_rational_line_intersection2_point_known_exact` | `hyperreal` | 919.86 ns | 1.95x |
| `gmp_real_collection_and_conversion_api` | `exact_rational_parameterized_point2_known_dyadic` | `gmp_mpfr128` | 147.07 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `exact_rational_parameterized_point2_known_dyadic` | `hyperreal` | 166.78 ns | 1.13x |
| `gmp_real_collection_and_conversion_api` | `exact_rational_quotient_known_dyadic` | `hyperreal` | 27.36 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `exact_rational_quotient_known_dyadic` | `gmp_mpfr128` | 63.08 ns | 2.31x |
| `gmp_real_collection_and_conversion_api` | `integer_bigint` | `gmp_mpfr128` | 58.82 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `integer_bigint` | `hyperreal` | 76.13 ns | 1.29x |
| `gmp_real_collection_and_conversion_api` | `inverse_ref` | `hyperreal` | 15.99 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `inverse_ref` | `gmp_mpfr128` | 60.45 ns | 3.78x |
| `gmp_real_collection_and_conversion_api` | `is_finite` | `gmp_mpfr128` | 0.49 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `is_finite` | `hyperreal` | 0.50 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `is_integer` | `gmp_mpfr128` | 1.68 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `is_integer` | `hyperreal` | 5.86 ns | 3.49x |
| `gmp_real_collection_and_conversion_api` | `max` | `hyperreal` | 15.92 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `max` | `gmp_mpfr128` | 26.13 ns | 1.64x |
| `gmp_real_collection_and_conversion_api` | `mean` | `gmp_mpfr128` | 69.57 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `mean` | `hyperreal` | 118.87 ns | 1.71x |
| `gmp_real_collection_and_conversion_api` | `min` | `hyperreal` | 16.10 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `min` | `gmp_mpfr128` | 26.30 ns | 1.63x |
| `gmp_real_collection_and_conversion_api` | `new_rational` | `gmp_mpfr128` | 26.77 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `new_rational` | `hyperreal` | 54.81 ns | 2.05x |
| `gmp_real_collection_and_conversion_api` | `one` | `hyperreal` | 9.90 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `one` | `gmp_mpfr128` | 19.99 ns | 2.02x |
| `gmp_real_collection_and_conversion_api` | `pi` | `hyperreal` | 16.54 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `pi` | `gmp_mpfr128` | 17.92 ns | 1.08x |
| `gmp_real_collection_and_conversion_api` | `sample_stddev` | `gmp_mpfr128` | 481.62 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `sample_stddev` | `hyperreal` | 789.49 ns | 1.64x |
| `gmp_real_collection_and_conversion_api` | `sum_owned` | `gmp_mpfr128` | 61.73 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `sum_owned` | `hyperreal` | 147.07 ns | 2.38x |
| `gmp_real_collection_and_conversion_api` | `sum_refs` | `gmp_mpfr128` | 61.97 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `sum_refs` | `hyperreal` | 114.98 ns | 1.86x |
| `gmp_real_collection_and_conversion_api` | `tau` | `hyperreal` | 16.31 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `tau` | `gmp_mpfr128` | 26.00 ns | 1.59x |
| `gmp_real_collection_and_conversion_api` | `to_f32_lossy` | `hyperreal` | 5.13 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `to_f32_lossy` | `gmp_mpfr128` | 8.26 ns | 1.61x |
| `gmp_real_collection_and_conversion_api` | `to_f64_exact_dyadic` | `hyperreal` | 2.40 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `to_f64_exact_dyadic` | `gmp_mpfr128` | 8.00 ns | 3.33x |
| `gmp_real_collection_and_conversion_api` | `to_f64_lossy` | `hyperreal` | 0.86 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `to_f64_lossy` | `gmp_mpfr128` | 8.07 ns | 9.35x |
| `gmp_real_collection_and_conversion_api` | `zero` | `hyperreal` | 9.60 ns | 1.00x |
| `gmp_real_collection_and_conversion_api` | `zero` | `gmp_mpfr128` | 17.96 ns | 1.87x |
| `gmp_real_derived_api` | `chi_square_cdf_k5` | `hyperreal` | 4.31 us | 1.00x |
| `gmp_real_derived_api` | `chi_square_cdf_k5` | `gmp_mpfr128` | 56.22 us | 13.04x |
| `gmp_real_derived_api` | `chi_square_sf_k5` | `hyperreal` | 3.06 us | 1.00x |
| `gmp_real_derived_api` | `chi_square_sf_k5` | `gmp_mpfr128` | 56.15 us | 18.33x |
| `gmp_real_derived_api` | `dnorm` | `hyperreal` | 1.05 us | 1.00x |
| `gmp_real_derived_api` | `dnorm` | `gmp_mpfr128` | 1.29 us | 1.23x |
| `gmp_real_derived_api` | `dnorm_derivative_n6` | `gmp_mpfr128` | 1.63 us | 1.00x |
| `gmp_real_derived_api` | `dnorm_derivative_n6` | `hyperreal` | 2.41 us | 1.48x |
| `gmp_real_derived_api` | `erfcx` | `hyperreal` | 1.25 us | 1.00x |
| `gmp_real_derived_api` | `erfcx` | `gmp_mpfr128` | 33.48 us | 26.69x |
| `gmp_real_derived_api` | `gaussian_derivative_n6` | `gmp_mpfr128` | 1.65 us | 1.00x |
| `gmp_real_derived_api` | `gaussian_derivative_n6` | `hyperreal` | 2.29 us | 1.39x |
| `gmp_real_derived_api` | `hermite_probabilists_n6` | `gmp_mpfr128` | 337.59 ns | 1.00x |
| `gmp_real_derived_api` | `hermite_probabilists_n6` | `hyperreal` | 1.07 us | 3.16x |
| `gmp_real_derived_api` | `log_dnorm` | `hyperreal` | 146.55 ns | 1.00x |
| `gmp_real_derived_api` | `log_dnorm` | `gmp_mpfr128` | 2.53 us | 17.24x |
| `gmp_real_derived_api` | `log_normal_sf` | `hyperreal` | 291.56 ns | 1.00x |
| `gmp_real_derived_api` | `log_normal_sf` | `gmp_mpfr128` | 34.79 us | 119.31x |
| `gmp_real_derived_api` | `log_pnorm` | `hyperreal` | 281.08 ns | 1.00x |
| `gmp_real_derived_api` | `log_pnorm` | `gmp_mpfr128` | 34.02 us | 121.05x |
| `gmp_real_derived_api` | `logit` | `hyperreal` | 356.77 ns | 1.00x |
| `gmp_real_derived_api` | `logit` | `gmp_mpfr128` | 1.42 us | 3.97x |
| `gmp_real_derived_api` | `normal_cdf` | `gmp_mpfr128` | 3.34 us | 1.00x |
| `gmp_real_derived_api` | `normal_cdf` | `hyperreal` | 3.58 us | 1.07x |
| `gmp_real_derived_api` | `normal_hazard` | `hyperreal` | 3.16 us | 1.00x |
| `gmp_real_derived_api` | `normal_hazard` | `gmp_mpfr128` | 33.71 us | 10.67x |
| `gmp_real_derived_api` | `normal_interval` | `hyperreal` | 681.31 ns | 1.00x |
| `gmp_real_derived_api` | `normal_interval` | `gmp_mpfr128` | 5.57 us | 8.18x |
| `gmp_real_derived_api` | `normal_interval_moment_n4` | `hyperreal` | 5.97 us | 1.00x |
| `gmp_real_derived_api` | `normal_interval_moment_n4` | `gmp_mpfr128` | 10.12 us | 1.69x |
| `gmp_real_derived_api` | `normal_inverse_mills` | `hyperreal` | 26.31 ns | 1.00x |
| `gmp_real_derived_api` | `normal_inverse_mills` | `gmp_mpfr128` | 418.92 ns | 15.92x |
| `gmp_real_derived_api` | `normal_log_hazard` | `hyperreal` | 531.32 ns | 1.00x |
| `gmp_real_derived_api` | `normal_log_hazard` | `gmp_mpfr128` | 36.55 us | 68.78x |
| `gmp_real_derived_api` | `normal_mills` | `hyperreal` | 3.00 us | 1.00x |
| `gmp_real_derived_api` | `normal_mills` | `gmp_mpfr128` | 33.02 us | 10.99x |
| `gmp_real_derived_api` | `normal_pdf` | `gmp_mpfr128` | 1.35 us | 1.00x |
| `gmp_real_derived_api` | `normal_pdf` | `hyperreal` | 1.40 us | 1.04x |
| `gmp_real_derived_api` | `normal_quantile` | `hyperreal` | 1.07 us | 1.00x |
| `gmp_real_derived_api` | `normal_quantile` | `gmp_mpfr128` | 33.02 us | 30.75x |
| `gmp_real_derived_api` | `normal_sf` | `hyperreal` | 335.35 ns | 1.00x |
| `gmp_real_derived_api` | `normal_sf` | `gmp_mpfr128` | 33.10 us | 98.71x |
| `gmp_real_derived_api` | `normal_survival` | `hyperreal` | 616.64 ns | 1.00x |
| `gmp_real_derived_api` | `normal_survival` | `gmp_mpfr128` | 3.26 us | 5.28x |
| `gmp_real_derived_api` | `pnorm` | `gmp_mpfr128` | 3.95 us | 1.00x |
| `gmp_real_derived_api` | `pnorm` | `hyperreal` | 4.85 us | 1.23x |
| `gmp_real_derived_api` | `pnorm_diff` | `hyperreal` | 647.81 ns | 1.00x |
| `gmp_real_derived_api` | `pnorm_diff` | `gmp_mpfr128` | 5.59 us | 8.64x |
| `gmp_real_derived_api` | `pnorm_upper` | `hyperreal` | 359.22 ns | 1.00x |
| `gmp_real_derived_api` | `pnorm_upper` | `gmp_mpfr128` | 4.00 us | 11.14x |
| `gmp_real_derived_api` | `qnorm` | `hyperreal` | 734.96 ns | 1.00x |
| `gmp_real_derived_api` | `qnorm` | `gmp_mpfr128` | 32.46 us | 44.16x |
| `gmp_real_derived_api` | `qnorm_upper` | `hyperreal` | 863.59 ns | 1.00x |
| `gmp_real_derived_api` | `qnorm_upper` | `gmp_mpfr128` | 32.36 us | 37.47x |
| `gmp_real_derived_api` | `regularized_beta_integer` | `gmp_mpfr128` | 783.87 ns | 1.00x |
| `gmp_real_derived_api` | `regularized_beta_integer` | `hyperreal` | 3.48 us | 4.44x |
| `gmp_real_derived_api` | `regularized_beta_q_integer` | `gmp_mpfr128` | 844.92 ns | 1.00x |
| `gmp_real_derived_api` | `regularized_beta_q_integer` | `hyperreal` | 2.69 us | 3.18x |
| `gmp_real_derived_api` | `regularized_gamma_p` | `hyperreal` | 4.08 us | 1.00x |
| `gmp_real_derived_api` | `regularized_gamma_p` | `gmp_mpfr128` | 56.06 us | 13.73x |
| `gmp_real_derived_api` | `regularized_gamma_q` | `hyperreal` | 2.89 us | 1.00x |
| `gmp_real_derived_api` | `regularized_gamma_q` | `gmp_mpfr128` | 57.19 us | 19.77x |
| `gmp_real_derived_api` | `sigmoid` | `hyperreal` | 228.61 ns | 1.00x |
| `gmp_real_derived_api` | `sigmoid` | `gmp_mpfr128` | 1.05 us | 4.61x |
| `gmp_real_derived_api` | `softplus` | `hyperreal` | 2.08 us | 1.00x |
| `gmp_real_derived_api` | `softplus` | `gmp_mpfr128` | 2.50 us | 1.20x |
| `gmp_real_derived_api` | `sqrt1m1` | `gmp_mpfr128` | 165.21 ns | 1.00x |
| `gmp_real_derived_api` | `sqrt1m1` | `hyperreal` | 954.19 ns | 5.78x |
| `gmp_real_derived_api` | `sqrt1pm1` | `gmp_mpfr128` | 159.63 ns | 1.00x |
| `gmp_real_derived_api` | `sqrt1pm1` | `hyperreal` | 955.42 ns | 5.99x |
| `gmp_real_derived_api` | `standard_normal_moment_n8` | `gmp_mpfr128` | 19.83 ns | 1.00x |
| `gmp_real_derived_api` | `standard_normal_moment_n8` | `hyperreal` | 114.78 ns | 5.79x |
| `gmp_real_derived_api` | `truncated_normal_mean` | `hyperreal` | 3.15 us | 1.00x |
| `gmp_real_derived_api` | `truncated_normal_mean` | `gmp_mpfr128` | 9.51 us | 3.01x |
| `gmp_real_derived_api` | `truncated_normal_variance` | `hyperreal` | 3.92 us | 1.00x |
| `gmp_real_derived_api` | `truncated_normal_variance` | `gmp_mpfr128` | 9.82 us | 2.50x |
| `gmp_real_elementary_api` | `acos` | `hyperreal` | 176.68 ns | 1.00x |
| `gmp_real_elementary_api` | `acos` | `gmp_mpfr128` | 3.27 us | 18.52x |
| `gmp_real_elementary_api` | `acosh` | `hyperreal` | 192.28 ns | 1.00x |
| `gmp_real_elementary_api` | `acosh` | `gmp_mpfr128` | 1.64 us | 8.53x |
| `gmp_real_elementary_api` | `asin` | `hyperreal` | 183.77 ns | 1.00x |
| `gmp_real_elementary_api` | `asin` | `gmp_mpfr128` | 3.24 us | 17.61x |
| `gmp_real_elementary_api` | `asinh` | `hyperreal` | 174.41 ns | 1.00x |
| `gmp_real_elementary_api` | `asinh` | `gmp_mpfr128` | 1.72 us | 9.84x |
| `gmp_real_elementary_api` | `atan` | `hyperreal` | 245.93 ns | 1.00x |
| `gmp_real_elementary_api` | `atan` | `gmp_mpfr128` | 2.94 us | 11.97x |
| `gmp_real_elementary_api` | `atan2` | `hyperreal` | 762.80 ns | 1.00x |
| `gmp_real_elementary_api` | `atan2` | `gmp_mpfr128` | 2.86 us | 3.76x |
| `gmp_real_elementary_api` | `atanh` | `hyperreal` | 391.04 ns | 1.00x |
| `gmp_real_elementary_api` | `atanh` | `gmp_mpfr128` | 1.69 us | 4.33x |
| `gmp_real_elementary_api` | `beta` | `hyperreal` | 1.04 us | 1.00x |
| `gmp_real_elementary_api` | `beta` | `gmp_mpfr128` | 17.98 us | 17.23x |
| `gmp_real_elementary_api` | `cbrt` | `hyperreal` | 264.69 ns | 1.00x |
| `gmp_real_elementary_api` | `cbrt` | `gmp_mpfr128` | 388.84 ns | 1.47x |
| `gmp_real_elementary_api` | `cos` | `hyperreal` | 213.12 ns | 1.00x |
| `gmp_real_elementary_api` | `cos` | `gmp_mpfr128` | 533.56 ns | 2.50x |
| `gmp_real_elementary_api` | `cos_pi` | `hyperreal` | 236.36 ns | 1.00x |
| `gmp_real_elementary_api` | `cos_pi` | `gmp_mpfr128` | 921.75 ns | 3.90x |
| `gmp_real_elementary_api` | `cosc` | `hyperreal` | 517.45 ns | 1.00x |
| `gmp_real_elementary_api` | `cosc` | `gmp_mpfr128` | 638.37 ns | 1.23x |
| `gmp_real_elementary_api` | `cosh` | `hyperreal` | 353.69 ns | 1.00x |
| `gmp_real_elementary_api` | `cosh` | `gmp_mpfr128` | 1.10 us | 3.10x |
| `gmp_real_elementary_api` | `cot` | `hyperreal` | 712.51 ns | 1.00x |
| `gmp_real_elementary_api` | `cot` | `gmp_mpfr128` | 1.09 us | 1.53x |
| `gmp_real_elementary_api` | `cot_pi` | `hyperreal` | 118.50 ns | 1.00x |
| `gmp_real_elementary_api` | `cot_pi` | `gmp_mpfr128` | 1.78 us | 15.05x |
| `gmp_real_elementary_api` | `erf` | `hyperreal` | 919.56 ns | 1.00x |
| `gmp_real_elementary_api` | `erf` | `gmp_mpfr128` | 3.37 us | 3.67x |
| `gmp_real_elementary_api` | `erfc` | `hyperreal` | 132.28 ns | 1.00x |
| `gmp_real_elementary_api` | `erfc` | `gmp_mpfr128` | 3.61 us | 27.27x |
| `gmp_real_elementary_api` | `erfcinv` | `hyperreal` | 1.33 us | 1.00x |
| `gmp_real_elementary_api` | `erfcinv` | `gmp_mpfr128` | 32.46 us | 24.33x |
| `gmp_real_elementary_api` | `erfinv` | `hyperreal` | 1.43 us | 1.00x |
| `gmp_real_elementary_api` | `erfinv` | `gmp_mpfr128` | 32.89 us | 22.96x |
| `gmp_real_elementary_api` | `exp` | `hyperreal` | 101.60 ns | 1.00x |
| `gmp_real_elementary_api` | `exp` | `gmp_mpfr128` | 936.10 ns | 9.21x |
| `gmp_real_elementary_api` | `exp10` | `hyperreal` | 161.34 ns | 1.00x |
| `gmp_real_elementary_api` | `exp10` | `gmp_mpfr128` | 2.89 us | 17.89x |
| `gmp_real_elementary_api` | `exp2` | `hyperreal` | 163.46 ns | 1.00x |
| `gmp_real_elementary_api` | `exp2` | `gmp_mpfr128` | 1.20 us | 7.36x |
| `gmp_real_elementary_api` | `expm1` | `hyperreal` | 159.02 ns | 1.00x |
| `gmp_real_elementary_api` | `expm1` | `gmp_mpfr128` | 737.63 ns | 4.64x |
| `gmp_real_elementary_api` | `gamma` | `hyperreal` | 313.98 ns | 1.00x |
| `gmp_real_elementary_api` | `gamma` | `gmp_mpfr128` | 8.27 us | 26.32x |
| `gmp_real_elementary_api` | `hypot2` | `hyperreal` | 96.07 ns | 1.00x |
| `gmp_real_elementary_api` | `hypot2` | `gmp_mpfr128` | 196.68 ns | 2.05x |
| `gmp_real_elementary_api` | `hypot3` | `hyperreal` | 141.71 ns | 1.00x |
| `gmp_real_elementary_api` | `hypot3` | `gmp_mpfr128` | 426.79 ns | 3.01x |
| `gmp_real_elementary_api` | `hypot_minus` | `gmp_mpfr128` | 243.25 ns | 1.00x |
| `gmp_real_elementary_api` | `hypot_minus` | `hyperreal` | 524.58 ns | 2.16x |
| `gmp_real_elementary_api` | `lbeta` | `hyperreal` | 2.88 us | 1.00x |
| `gmp_real_elementary_api` | `lbeta` | `gmp_mpfr128` | 24.74 us | 8.60x |
| `gmp_real_elementary_api` | `lgamma` | `hyperreal` | 3.01 us | 1.00x |
| `gmp_real_elementary_api` | `lgamma` | `gmp_mpfr128` | 8.17 us | 2.71x |
| `gmp_real_elementary_api` | `ln` | `hyperreal` | 882.94 ns | 1.00x |
| `gmp_real_elementary_api` | `ln` | `gmp_mpfr128` | 1.33 us | 1.51x |
| `gmp_real_elementary_api` | `ln_1m` | `hyperreal` | 62.39 ns | 1.00x |
| `gmp_real_elementary_api` | `ln_1m` | `gmp_mpfr128` | 3.56 us | 57.07x |
| `gmp_real_elementary_api` | `ln_1p` | `hyperreal` | 58.68 ns | 1.00x |
| `gmp_real_elementary_api` | `ln_1p` | `gmp_mpfr128` | 349.18 ns | 5.95x |
| `gmp_real_elementary_api` | `ln_beta` | `hyperreal` | 2.84 us | 1.00x |
| `gmp_real_elementary_api` | `ln_beta` | `gmp_mpfr128` | 24.72 us | 8.72x |
| `gmp_real_elementary_api` | `log10` | `hyperreal` | 880.63 ns | 1.00x |
| `gmp_real_elementary_api` | `log10` | `gmp_mpfr128` | 3.05 us | 3.46x |
| `gmp_real_elementary_api` | `log1m` | `hyperreal` | 63.13 ns | 1.00x |
| `gmp_real_elementary_api` | `log1m` | `gmp_mpfr128` | 3.66 us | 57.98x |
| `gmp_real_elementary_api` | `log1p` | `hyperreal` | 58.68 ns | 1.00x |
| `gmp_real_elementary_api` | `log1p` | `gmp_mpfr128` | 341.18 ns | 5.81x |
| `gmp_real_elementary_api` | `log2` | `hyperreal` | 893.41 ns | 1.00x |
| `gmp_real_elementary_api` | `log2` | `gmp_mpfr128` | 1.53 us | 1.71x |
| `gmp_real_elementary_api` | `logaddexp` | `hyperreal` | 258.81 ns | 1.00x |
| `gmp_real_elementary_api` | `logaddexp` | `gmp_mpfr128` | 3.54 us | 13.69x |
| `gmp_real_elementary_api` | `logsubexp` | `hyperreal` | 262.89 ns | 1.00x |
| `gmp_real_elementary_api` | `logsubexp` | `gmp_mpfr128` | 3.55 us | 13.52x |
| `gmp_real_elementary_api` | `pow_rational_5_over_3` | `hyperreal` | 513.55 ns | 1.00x |
| `gmp_real_elementary_api` | `pow_rational_5_over_3` | `gmp_mpfr128` | 2.91 us | 5.67x |
| `gmp_real_elementary_api` | `root_n_5` | `hyperreal` | 156.06 ns | 1.00x |
| `gmp_real_elementary_api` | `root_n_5` | `gmp_mpfr128` | 487.17 ns | 3.12x |
| `gmp_real_elementary_api` | `sin` | `hyperreal` | 214.25 ns | 1.00x |
| `gmp_real_elementary_api` | `sin` | `gmp_mpfr128` | 733.75 ns | 3.42x |
| `gmp_real_elementary_api` | `sin_pi` | `hyperreal` | 90.81 ns | 1.00x |
| `gmp_real_elementary_api` | `sin_pi` | `gmp_mpfr128` | 1.53 us | 16.84x |
| `gmp_real_elementary_api` | `sinc` | `hyperreal` | 287.42 ns | 1.00x |
| `gmp_real_elementary_api` | `sinc` | `gmp_mpfr128` | 789.43 ns | 2.75x |
| `gmp_real_elementary_api` | `sinc_pi` | `hyperreal` | 384.33 ns | 1.00x |
| `gmp_real_elementary_api` | `sinc_pi` | `gmp_mpfr128` | 1.65 us | 4.29x |
| `gmp_real_elementary_api` | `sinh` | `hyperreal` | 393.45 ns | 1.00x |
| `gmp_real_elementary_api` | `sinh` | `gmp_mpfr128` | 1.14 us | 2.89x |
| `gmp_real_elementary_api` | `sqrt` | `hyperreal` | 102.56 ns | 1.00x |
| `gmp_real_elementary_api` | `sqrt` | `gmp_mpfr128` | 107.31 ns | 1.05x |
| `gmp_real_elementary_api` | `tan` | `hyperreal` | 57.72 ns | 1.00x |
| `gmp_real_elementary_api` | `tan` | `gmp_mpfr128` | 983.86 ns | 17.05x |
| `gmp_real_elementary_api` | `tan_pi` | `hyperreal` | 173.48 ns | 1.00x |
| `gmp_real_elementary_api` | `tan_pi` | `gmp_mpfr128` | 1.84 us | 10.63x |
| `gmp_real_elementary_api` | `tanh` | `hyperreal` | 575.71 ns | 1.00x |
| `gmp_real_elementary_api` | `tanh` | `gmp_mpfr128` | 1.18 us | 2.05x |
| `gmp_real_linear_algebra_api` | `active_dot2_refs` | `hyperreal` | 57.27 ns | 1.00x |
| `gmp_real_linear_algebra_api` | `active_dot2_refs` | `gmp_mpfr128` | 155.64 ns | 2.72x |
| `gmp_real_linear_algebra_api` | `active_dot3_refs` | `hyperreal` | 69.09 ns | 1.00x |
| `gmp_real_linear_algebra_api` | `active_dot3_refs` | `gmp_mpfr128` | 265.24 ns | 3.84x |
| `gmp_real_linear_algebra_api` | `active_dot4_refs` | `hyperreal` | 93.01 ns | 1.00x |
| `gmp_real_linear_algebra_api` | `active_dot4_refs` | `gmp_mpfr128` | 199.65 ns | 2.15x |
| `gmp_real_linear_algebra_api` | `active_linear_combination3_refs` | `hyperreal` | 70.56 ns | 1.00x |
| `gmp_real_linear_algebra_api` | `active_linear_combination3_refs` | `gmp_mpfr128` | 268.49 ns | 3.81x |
| `gmp_real_linear_algebra_api` | `active_linear_combination4_refs` | `hyperreal` | 90.62 ns | 1.00x |
| `gmp_real_linear_algebra_api` | `active_linear_combination4_refs` | `gmp_mpfr128` | 200.00 ns | 2.21x |
| `gmp_real_linear_algebra_api` | `active_signed_product_sum` | `hyperreal` | 54.31 ns | 1.00x |
| `gmp_real_linear_algebra_api` | `active_signed_product_sum` | `gmp_mpfr128` | 82.04 ns | 1.51x |
| `gmp_real_linear_algebra_api` | `affine_combination3_refs` | `hyperreal` | 88.37 ns | 1.00x |
| `gmp_real_linear_algebra_api` | `affine_combination3_refs` | `gmp_mpfr128` | 273.88 ns | 3.10x |
| `gmp_real_linear_algebra_api` | `affine_combination4_refs` | `hyperreal` | 115.59 ns | 1.00x |
| `gmp_real_linear_algebra_api` | `affine_combination4_refs` | `gmp_mpfr128` | 220.92 ns | 1.91x |
| `gmp_real_linear_algebra_api` | `diff_of_products` | `hyperreal` | 51.19 ns | 1.00x |
| `gmp_real_linear_algebra_api` | `diff_of_products` | `gmp_mpfr128` | 83.25 ns | 1.63x |
| `gmp_real_linear_algebra_api` | `dot2_refs` | `hyperreal` | 57.78 ns | 1.00x |
| `gmp_real_linear_algebra_api` | `dot2_refs` | `gmp_mpfr128` | 159.25 ns | 2.76x |
| `gmp_real_linear_algebra_api` | `dot3_refs` | `hyperreal` | 72.28 ns | 1.00x |
| `gmp_real_linear_algebra_api` | `dot3_refs` | `gmp_mpfr128` | 268.39 ns | 3.71x |
| `gmp_real_linear_algebra_api` | `dot4_refs` | `hyperreal` | 92.77 ns | 1.00x |
| `gmp_real_linear_algebra_api` | `dot4_refs` | `gmp_mpfr128` | 202.53 ns | 2.18x |
| `gmp_real_linear_algebra_api` | `eval_poly` | `hyperreal` | 99.02 ns | 1.00x |
| `gmp_real_linear_algebra_api` | `eval_poly` | `gmp_mpfr128` | 120.80 ns | 1.22x |
| `gmp_real_linear_algebra_api` | `eval_rational_poly` | `gmp_mpfr128` | 257.65 ns | 1.00x |
| `gmp_real_linear_algebra_api` | `eval_rational_poly` | `hyperreal` | 291.45 ns | 1.13x |
| `gmp_real_linear_algebra_api` | `linear_combination3_refs` | `hyperreal` | 73.37 ns | 1.00x |
| `gmp_real_linear_algebra_api` | `linear_combination3_refs` | `gmp_mpfr128` | 268.38 ns | 3.66x |
| `gmp_real_linear_algebra_api` | `linear_combination4_refs` | `hyperreal` | 91.32 ns | 1.00x |
| `gmp_real_linear_algebra_api` | `linear_combination4_refs` | `gmp_mpfr128` | 199.65 ns | 2.19x |
| `gmp_real_linear_algebra_api` | `mul_add` | `gmp_mpfr128` | 72.05 ns | 1.00x |
| `gmp_real_linear_algebra_api` | `mul_add` | `hyperreal` | 95.56 ns | 1.33x |
| `gmp_real_linear_algebra_api` | `signed_product_sum` | `hyperreal` | 54.12 ns | 1.00x |
| `gmp_real_linear_algebra_api` | `signed_product_sum` | `gmp_mpfr128` | 82.23 ns | 1.52x |
| `gmp_real_linear_algebra_api` | `sum_products` | `hyperreal` | 98.87 ns | 1.00x |
| `gmp_real_linear_algebra_api` | `sum_products` | `gmp_mpfr128` | 196.31 ns | 1.99x |
| `real_representation_construction_export` | `const_offset` | `mpfr192` | 60.71 ns | 1.00x |
| `real_representation_construction_export` | `const_offset` | `hyperreal_exact` | 107.15 ns | 1.77x |
| `real_representation_construction_export` | `const_product` | `hyperreal_exact` | 1.31 us | 1.00x |
| `real_representation_construction_export` | `const_product` | `mpfr192` | 1.42 us | 1.09x |
| `real_representation_construction_export` | `const_product_sqrt` | `mpfr192` | 1.55 us | 1.00x |
| `real_representation_construction_export` | `const_product_sqrt` | `hyperreal_exact` | 2.56 us | 1.65x |
| `real_representation_construction_export` | `exp` | `hyperreal_exact` | 189.99 ns | 1.00x |
| `real_representation_construction_export` | `exp` | `mpfr192` | 1.41 us | 7.42x |
| `real_representation_construction_export` | `irrational` | `mpfr192` | 1.03 us | 1.00x |
| `real_representation_construction_export` | `irrational` | `hyperreal_exact` | 2.15 us | 2.10x |
| `real_representation_construction_export` | `ln` | `hyperreal_exact` | 117.53 ns | 1.00x |
| `real_representation_construction_export` | `ln` | `mpfr192` | 1.72 us | 14.59x |
| `real_representation_construction_export` | `ln_affine` | `hyperreal_exact` | 155.93 ns | 1.00x |
| `real_representation_construction_export` | `ln_affine` | `mpfr192` | 3.17 us | 20.30x |
| `real_representation_construction_export` | `ln_product` | `hyperreal_exact` | 773.53 ns | 1.00x |
| `real_representation_construction_export` | `ln_product` | `mpfr192` | 3.58 us | 4.63x |
| `real_representation_construction_export` | `log10` | `hyperreal_exact` | 1.39 us | 1.00x |
| `real_representation_construction_export` | `log10` | `mpfr192` | 3.72 us | 2.68x |
| `real_representation_construction_export` | `log2` | `hyperreal_exact` | 1.24 us | 1.00x |
| `real_representation_construction_export` | `log2` | `mpfr192` | 1.86 us | 1.50x |
| `real_representation_construction_export` | `one` | `mpfr192` | 30.62 ns | 1.00x |
| `real_representation_construction_export` | `one` | `hyperreal_exact` | 62.03 ns | 2.03x |
| `real_representation_construction_export` | `pi` | `hyperreal_exact` | 27.74 ns | 1.00x |
| `real_representation_construction_export` | `pi` | `mpfr192` | 29.36 ns | 1.06x |
| `real_representation_construction_export` | `pi_exp` | `hyperreal_exact` | 138.87 ns | 1.00x |
| `real_representation_construction_export` | `pi_exp` | `mpfr192` | 1.40 us | 10.10x |
| `real_representation_construction_export` | `pi_inv` | `hyperreal_exact` | 56.85 ns | 1.00x |
| `real_representation_construction_export` | `pi_inv` | `mpfr192` | 95.57 ns | 1.68x |
| `real_representation_construction_export` | `pi_inv_exp` | `hyperreal_exact` | 143.18 ns | 1.00x |
| `real_representation_construction_export` | `pi_inv_exp` | `mpfr192` | 1.44 us | 10.06x |
| `real_representation_construction_export` | `pi_pow` | `mpfr192` | 55.46 ns | 1.00x |
| `real_representation_construction_export` | `pi_pow` | `hyperreal_exact` | 108.68 ns | 1.96x |
| `real_representation_construction_export` | `pi_sqrt` | `mpfr192` | 150.26 ns | 1.00x |
| `real_representation_construction_export` | `pi_sqrt` | `hyperreal_exact` | 386.99 ns | 2.58x |
| `real_representation_construction_export` | `sin_pi` | `mpfr192` | 1.23 us | 1.00x |
| `real_representation_construction_export` | `sin_pi` | `hyperreal_exact` | 3.75 us | 3.05x |
| `real_representation_construction_export` | `sqrt` | `hyperreal_exact` | 105.29 ns | 1.00x |
| `real_representation_construction_export` | `sqrt` | `mpfr192` | 116.64 ns | 1.11x |
| `real_representation_construction_export` | `tan_pi` | `mpfr192` | 1.48 us | 1.00x |
| `real_representation_construction_export` | `tan_pi` | `hyperreal_exact` | 12.07 us | 8.17x |
| `real_representation_prepared` | `const_offset` | `hyperreal_cached_f64` | 1.18 ns | 1.00x |
| `real_representation_prepared` | `const_offset` | `mpfr192_f64` | 8.03 ns | 6.82x |
| `real_representation_prepared` | `const_offset` | `mpfr192_clone` | 17.18 ns | 14.59x |
| `real_representation_prepared` | `const_offset` | `hyperreal_clone` | 28.19 ns | 23.95x |
| `real_representation_prepared` | `const_offset` | `hyperreal_certified_192` | 191.40 ns | 162.56x |
| `real_representation_prepared` | `const_product` | `hyperreal_cached_f64` | 1.18 ns | 1.00x |
| `real_representation_prepared` | `const_product` | `mpfr192_f64` | 8.05 ns | 6.81x |
| `real_representation_prepared` | `const_product` | `mpfr192_clone` | 17.24 ns | 14.58x |
| `real_representation_prepared` | `const_product` | `hyperreal_clone` | 22.49 ns | 19.02x |
| `real_representation_prepared` | `const_product` | `hyperreal_certified_192` | 225.82 ns | 191.02x |
| `real_representation_prepared` | `const_product_sqrt` | `hyperreal_cached_f64` | 1.18 ns | 1.00x |
| `real_representation_prepared` | `const_product_sqrt` | `mpfr192_f64` | 8.00 ns | 6.79x |
| `real_representation_prepared` | `const_product_sqrt` | `mpfr192_clone` | 17.22 ns | 14.61x |
| `real_representation_prepared` | `const_product_sqrt` | `hyperreal_clone` | 29.42 ns | 24.95x |
| `real_representation_prepared` | `const_product_sqrt` | `hyperreal_certified_192` | 189.59 ns | 160.81x |
| `real_representation_prepared` | `exp` | `hyperreal_cached_f64` | 1.18 ns | 1.00x |
| `real_representation_prepared` | `exp` | `mpfr192_f64` | 7.34 ns | 6.23x |
| `real_representation_prepared` | `exp` | `mpfr192_clone` | 17.23 ns | 14.61x |
| `real_representation_prepared` | `exp` | `hyperreal_clone` | 18.97 ns | 16.08x |
| `real_representation_prepared` | `exp` | `hyperreal_certified_192` | 227.82 ns | 193.14x |
| `real_representation_prepared` | `irrational` | `hyperreal_cached_f64` | 1.20 ns | 1.00x |
| `real_representation_prepared` | `irrational` | `mpfr192_f64` | 8.19 ns | 6.84x |
| `real_representation_prepared` | `irrational` | `hyperreal_clone` | 13.58 ns | 11.34x |
| `real_representation_prepared` | `irrational` | `mpfr192_clone` | 17.51 ns | 14.62x |
| `real_representation_prepared` | `irrational` | `hyperreal_certified_192` | 197.89 ns | 165.21x |
| `real_representation_prepared` | `ln` | `hyperreal_cached_f64` | 1.18 ns | 1.00x |
| `real_representation_prepared` | `ln` | `mpfr192_f64` | 7.34 ns | 6.23x |
| `real_representation_prepared` | `ln` | `mpfr192_clone` | 17.28 ns | 14.65x |
| `real_representation_prepared` | `ln` | `hyperreal_clone` | 19.01 ns | 16.13x |
| `real_representation_prepared` | `ln` | `hyperreal_certified_192` | 190.97 ns | 161.96x |
| `real_representation_prepared` | `ln_affine` | `hyperreal_cached_f64` | 1.18 ns | 1.00x |
| `real_representation_prepared` | `ln_affine` | `mpfr192_f64` | 7.33 ns | 6.19x |
| `real_representation_prepared` | `ln_affine` | `mpfr192_clone` | 17.23 ns | 14.54x |
| `real_representation_prepared` | `ln_affine` | `hyperreal_clone` | 28.35 ns | 23.93x |
| `real_representation_prepared` | `ln_affine` | `hyperreal_certified_192` | 189.90 ns | 160.28x |
| `real_representation_prepared` | `ln_product` | `hyperreal_cached_f64` | 1.17 ns | 1.00x |
| `real_representation_prepared` | `ln_product` | `mpfr192_f64` | 7.34 ns | 6.25x |
| `real_representation_prepared` | `ln_product` | `mpfr192_clone` | 17.23 ns | 14.67x |
| `real_representation_prepared` | `ln_product` | `hyperreal_clone` | 28.31 ns | 24.12x |
| `real_representation_prepared` | `ln_product` | `hyperreal_certified_192` | 193.24 ns | 164.60x |
| `real_representation_prepared` | `log10` | `hyperreal_cached_f64` | 1.18 ns | 1.00x |
| `real_representation_prepared` | `log10` | `mpfr192_f64` | 7.31 ns | 6.18x |
| `real_representation_prepared` | `log10` | `mpfr192_clone` | 17.22 ns | 14.56x |
| `real_representation_prepared` | `log10` | `hyperreal_clone` | 18.92 ns | 16.00x |
| `real_representation_prepared` | `log10` | `hyperreal_certified_192` | 228.18 ns | 193.01x |
| `real_representation_prepared` | `log2` | `hyperreal_cached_f64` | 1.19 ns | 1.00x |
| `real_representation_prepared` | `log2` | `mpfr192_f64` | 8.03 ns | 6.76x |
| `real_representation_prepared` | `log2` | `mpfr192_clone` | 17.18 ns | 14.45x |
| `real_representation_prepared` | `log2` | `hyperreal_clone` | 18.91 ns | 15.90x |
| `real_representation_prepared` | `log2` | `hyperreal_certified_192` | 227.05 ns | 190.95x |
| `real_representation_prepared` | `one` | `hyperreal_cached_f64` | 1.18 ns | 1.00x |
| `real_representation_prepared` | `one` | `mpfr192_f64` | 8.03 ns | 6.82x |
| `real_representation_prepared` | `one` | `hyperreal_clone` | 9.70 ns | 8.24x |
| `real_representation_prepared` | `one` | `hyperreal_certified_192` | 13.26 ns | 11.26x |
| `real_representation_prepared` | `one` | `mpfr192_clone` | 17.18 ns | 14.60x |
| `real_representation_prepared` | `pi` | `hyperreal_cached_f64` | 1.18 ns | 1.00x |
| `real_representation_prepared` | `pi` | `mpfr192_f64` | 8.01 ns | 6.80x |
| `real_representation_prepared` | `pi` | `hyperreal_clone` | 13.71 ns | 11.63x |
| `real_representation_prepared` | `pi` | `mpfr192_clone` | 17.17 ns | 14.57x |
| `real_representation_prepared` | `pi` | `hyperreal_certified_192` | 195.22 ns | 165.65x |
| `real_representation_prepared` | `pi_exp` | `hyperreal_cached_f64` | 1.19 ns | 1.00x |
| `real_representation_prepared` | `pi_exp` | `mpfr192_f64` | 7.33 ns | 6.18x |
| `real_representation_prepared` | `pi_exp` | `mpfr192_clone` | 17.26 ns | 14.55x |
| `real_representation_prepared` | `pi_exp` | `hyperreal_clone` | 18.81 ns | 15.86x |
| `real_representation_prepared` | `pi_exp` | `hyperreal_certified_192` | 190.67 ns | 160.76x |
| `real_representation_prepared` | `pi_inv` | `hyperreal_cached_f64` | 1.17 ns | 1.00x |
| `real_representation_prepared` | `pi_inv` | `mpfr192_f64` | 7.35 ns | 6.26x |
| `real_representation_prepared` | `pi_inv` | `hyperreal_clone` | 13.54 ns | 11.53x |
| `real_representation_prepared` | `pi_inv` | `mpfr192_clone` | 17.13 ns | 14.59x |
| `real_representation_prepared` | `pi_inv` | `hyperreal_certified_192` | 230.64 ns | 196.42x |
| `real_representation_prepared` | `pi_inv_exp` | `hyperreal_cached_f64` | 1.17 ns | 1.00x |
| `real_representation_prepared` | `pi_inv_exp` | `mpfr192_f64` | 8.05 ns | 6.86x |
| `real_representation_prepared` | `pi_inv_exp` | `mpfr192_clone` | 17.18 ns | 14.64x |
| `real_representation_prepared` | `pi_inv_exp` | `hyperreal_clone` | 18.78 ns | 16.00x |
| `real_representation_prepared` | `pi_inv_exp` | `hyperreal_certified_192` | 193.19 ns | 164.61x |
| `real_representation_prepared` | `pi_pow` | `hyperreal_cached_f64` | 1.18 ns | 1.00x |
| `real_representation_prepared` | `pi_pow` | `mpfr192_f64` | 8.05 ns | 6.81x |
| `real_representation_prepared` | `pi_pow` | `hyperreal_clone` | 13.68 ns | 11.58x |
| `real_representation_prepared` | `pi_pow` | `mpfr192_clone` | 17.19 ns | 14.54x |
| `real_representation_prepared` | `pi_pow` | `hyperreal_certified_192` | 228.63 ns | 193.48x |
| `real_representation_prepared` | `pi_sqrt` | `hyperreal_cached_f64` | 1.18 ns | 1.00x |
| `real_representation_prepared` | `pi_sqrt` | `mpfr192_f64` | 8.05 ns | 6.81x |
| `real_representation_prepared` | `pi_sqrt` | `mpfr192_clone` | 17.21 ns | 14.56x |
| `real_representation_prepared` | `pi_sqrt` | `hyperreal_clone` | 18.94 ns | 16.02x |
| `real_representation_prepared` | `pi_sqrt` | `hyperreal_certified_192` | 190.18 ns | 160.88x |
| `real_representation_prepared` | `sin_pi` | `hyperreal_cached_f64` | 1.18 ns | 1.00x |
| `real_representation_prepared` | `sin_pi` | `mpfr192_f64` | 7.44 ns | 6.31x |
| `real_representation_prepared` | `sin_pi` | `mpfr192_clone` | 17.15 ns | 14.55x |
| `real_representation_prepared` | `sin_pi` | `hyperreal_clone` | 19.00 ns | 16.12x |
| `real_representation_prepared` | `sin_pi` | `hyperreal_certified_192` | 233.21 ns | 197.88x |
| `real_representation_prepared` | `sqrt` | `hyperreal_cached_f64` | 1.18 ns | 1.00x |
| `real_representation_prepared` | `sqrt` | `mpfr192_f64` | 7.36 ns | 6.26x |
| `real_representation_prepared` | `sqrt` | `mpfr192_clone` | 17.26 ns | 14.67x |
| `real_representation_prepared` | `sqrt` | `hyperreal_clone` | 19.09 ns | 16.23x |
| `real_representation_prepared` | `sqrt` | `hyperreal_certified_192` | 190.16 ns | 161.67x |
| `real_representation_prepared` | `tan_pi` | `hyperreal_cached_f64` | 1.19 ns | 1.00x |
| `real_representation_prepared` | `tan_pi` | `mpfr192_f64` | 7.43 ns | 6.23x |
| `real_representation_prepared` | `tan_pi` | `mpfr192_clone` | 17.44 ns | 14.61x |
| `real_representation_prepared` | `tan_pi` | `hyperreal_clone` | 19.18 ns | 16.06x |
| `real_representation_prepared` | `tan_pi` | `hyperreal_certified_192` | 197.23 ns | 165.18x |

### All Criterion results

| Benchmark | Mean | 95% CI | Median | Change vs baseline | Throughput |
| --- | ---: | ---: | ---: | ---: | ---: |
| `adversarial_library_fuzz/collect_slow_performer_history` | 166.95 ms | 166.25 ms - 167.72 ms | 167.18 ms | -0.05% | - |
| `borrowed_op_overhead/rational_add_owned` | 10.25 ns | 10.22 ns - 10.27 ns | 10.22 ns | -0.50% | - |
| `borrowed_op_overhead/rational_add_refs` | 8.78 ns | 8.75 ns - 8.81 ns | 8.73 ns | -2.29% | - |
| `borrowed_op_overhead/rational_clone_pair` | 7.35 ns | 7.34 ns - 7.36 ns | 7.34 ns | -0.96% | - |
| `borrowed_op_overhead/real_active_dot2_refs_dense_symbolic` | 494.39 ns | 492.74 ns - 496.52 ns | 492.66 ns | -11.32% | - |
| `borrowed_op_overhead/real_active_dot3_refs_dense_symbolic` | 1.02 us | 1.02 us - 1.02 us | 1.02 us | -4.91% | - |
| `borrowed_op_overhead/real_active_dot4_refs_dense_symbolic` | 1.58 us | 1.57 us - 1.60 us | 1.56 us | -4.09% | - |
| `borrowed_op_overhead/real_add_owned` | 272.38 ns | 270.94 ns - 274.34 ns | 270.20 ns | -3.00% | - |
| `borrowed_op_overhead/real_add_refs` | 256.14 ns | 255.86 ns - 256.45 ns | 255.76 ns | -2.46% | - |
| `borrowed_op_overhead/real_clone_pair` | 37.29 ns | 37.23 ns - 37.38 ns | 37.21 ns | -1.03% | - |
| `borrowed_op_overhead/real_dot2_refs_dense_symbolic` | 484.89 ns | 484.56 ns - 485.22 ns | 484.94 ns | -14.19% | - |
| `borrowed_op_overhead/real_dot2_refs_mixed_structural` | 26.98 ns | 26.96 ns - 27.00 ns | 26.98 ns | -1.53% | - |
| `borrowed_op_overhead/real_dot3_refs_dense_symbolic` | 1.02 us | 1.01 us - 1.02 us | 1.01 us | -4.32% | - |
| `borrowed_op_overhead/real_dot3_refs_mixed_structural` | 203.49 ns | 203.10 ns - 204.03 ns | 203.14 ns | -3.71% | - |
| `borrowed_op_overhead/real_dot4_refs_dense_symbolic` | 1.56 us | 1.55 us - 1.56 us | 1.55 us | -4.96% | - |
| `borrowed_op_overhead/real_dot4_refs_mixed_structural` | 242.50 ns | 241.92 ns - 243.17 ns | 241.71 ns | -2.13% | - |
| `borrowed_op_overhead/real_unscaled_add_owned` | 89.88 ns | 89.54 ns - 90.37 ns | 89.32 ns | -10.38% | - |
| `borrowed_op_overhead/real_unscaled_add_refs` | 90.20 ns | 90.07 ns - 90.36 ns | 90.01 ns | -0.64% | - |
| `computable_algebraic_roots/eighth_root_near_dyadic_sign_p64_cold` | 5.96 us | 5.94 us - 5.99 us | 5.93 us | -1.47% | - |
| `computable_algebraic_roots/many_digits_c10_zero_sign_p2048_cold` | 40.11 us | 40.03 us - 40.21 us | 40.02 us | -0.54% | - |
| `computable_algebraic_roots/ramanujan_one_zero_sign_p2048_cold` | 43.31 us | 43.16 us - 43.48 us | 43.12 us | -0.69% | - |
| `computable_algebraic_roots/ramanujan_two_zero_sign_p2048_cold` | 126.52 us | 126.21 us - 126.90 us | 126.36 us | -2.93% | - |
| `computable_algebraic_roots/root10_interval_p128_fallback_cold` | 5.87 us | 5.86 us - 5.87 us | 5.86 us | +0.50% | - |
| `computable_algebraic_roots/root5_construct` | 193.38 ns | 193.09 ns - 193.69 ns | 193.36 ns | -2.16% | - |
| `computable_algebraic_roots/root5_interval_p128_cold` | 2.93 us | 2.92 us - 2.95 us | 2.92 us | +1.58% | - |
| `computable_algebraic_roots/root5_interval_p2048_cold` | 98.99 us | 98.87 us - 99.16 us | 98.94 us | -1.25% | - |
| `computable_algebraic_roots/root9_interval_p128_cold` | 5.74 us | 5.72 us - 5.77 us | 5.70 us | -0.28% | - |
| `computable_algebraic_roots/root9_interval_p2048_cold` | 272.92 us | 272.42 us - 273.48 us | 272.02 us | -0.72% | - |
| `computable_bounds/deep_scaled_product_sign_until_p2000` | 5.71 ns | 5.70 ns - 5.73 ns | 5.69 ns | -0.87% | - |
| `computable_bounds/deep_structural_bound_facts_cached` | 8.60 ns | 8.52 ns - 8.70 ns | 8.41 ns | +3.56% | - |
| `computable_bounds/deep_structural_bound_sign_until_cached` | 4.00 ns | 3.99 ns - 4.01 ns | 3.99 ns | -1.40% | - |
| `computable_bounds/deep_structural_bound_sign_until_p2000` | 5.75 ns | 5.73 ns - 5.78 ns | 5.71 ns | +0.72% | - |
| `computable_bounds/exp_unknown_sign_arg_sign_until_cached` | 4.01 ns | 4.00 ns - 4.02 ns | 4.00 ns | -0.80% | - |
| `computable_bounds/exp_unknown_sign_arg_sign_until_p2000` | 6.10 ns | 6.09 ns - 6.11 ns | 6.08 ns | +5.08% | - |
| `computable_bounds/mixed_pi_e_sign_until_p0_cold` | 605.62 ns | 599.43 ns - 611.70 ns | 628.89 ns | +0.56% | - |
| `computable_bounds/near_pi_sign_until_p0_inconclusive_cold` | 867.83 ns | 859.69 ns - 876.41 ns | 845.46 ns | +0.07% | - |
| `computable_bounds/near_pi_sign_until_p64_cold` | 525.02 ns | 517.52 ns - 532.66 ns | 505.43 ns | +2.90% | - |
| `computable_bounds/perturbed_scaled_product_sign_until_p128` | 5.75 ns | 5.73 ns - 5.78 ns | 5.72 ns | +0.11% | - |
| `computable_bounds/perturbed_scaled_product_sign_until_p2000` | 5.75 ns | 5.73 ns - 5.77 ns | 5.73 ns | +0.34% | - |
| `computable_bounds/pi_minus_tiny_sign_until_cached` | 4.00 ns | 3.99 ns - 4.00 ns | 3.99 ns | -0.69% | - |
| `computable_bounds/pi_minus_tiny_sign_until_p2000` | 6.09 ns | 6.05 ns - 6.14 ns | 6.01 ns | +2.15% | - |
| `computable_bounds/scaled_square_sign_until_p2000` | 5.73 ns | 5.71 ns - 5.76 ns | 5.71 ns | -1.71% | - |
| `computable_bounds/sqrt_scaled_square_sign_until_p2000` | 56.30 ns | 53.14 ns - 59.48 ns | 43.11 ns | +2.38% | - |
| `computable_bounds/unsupported_sin_difference_sign_until_p0_cold` | 1.03 us | 1.02 us - 1.04 us | 1.01 us | -0.54% | - |
| `computable_bounds/unsupported_sin_difference_sign_until_p64_cold` | 4.38 us | 4.36 us - 4.40 us | 4.37 us | +1.00% | - |
| `computable_cache/pi_approx_cached_p128` | 19.08 ns | 18.98 ns - 19.20 ns | 18.91 ns | -9.79% | - |
| `computable_cache/pi_approx_cold_p128` | 26.58 ns | 26.54 ns - 26.61 ns | 26.57 ns | -6.05% | - |
| `computable_cache/pi_minus_tiny_cold_p128` | 27.12 ns | 27.02 ns - 27.25 ns | 26.95 ns | -6.31% | - |
| `computable_cache/pi_plus_tiny_cold_p128` | 27.16 ns | 27.04 ns - 27.31 ns | 26.99 ns | -5.56% | - |
| `computable_cache/ratio_approx_cached_p128` | 18.91 ns | 18.88 ns - 18.94 ns | 18.88 ns | -4.39% | - |
| `computable_cache/ratio_approx_cold_p128` | 23.10 ns | 23.02 ns - 23.18 ns | 22.98 ns | -5.77% | - |
| `computable_compare/compare_absolute_dominant_add` | 12.95 ns | 12.89 ns - 13.02 ns | 12.79 ns | -0.11% | - |
| `computable_compare/compare_absolute_exact_msd_gap` | 14.71 ns | 14.67 ns - 14.76 ns | 14.62 ns | -0.39% | - |
| `computable_compare/compare_absolute_exact_rational` | 4.61 ns | 4.58 ns - 4.64 ns | 4.55 ns | -0.12% | - |
| `computable_compare/compare_absolute_exact_rational_same_numerator` | 36.88 ns | 36.54 ns - 37.26 ns | 36.07 ns | -4.45% | - |
| `computable_compare/compare_absolute_mixed_exact_leaf_kinds` | 24.18 ns | 24.10 ns - 24.27 ns | 24.08 ns | -0.81% | - |
| `computable_compare/compare_to_clone_shared_composite` | 5.03 ns | 5.02 ns - 5.03 ns | 5.03 ns | -1.95% | - |
| `computable_compare/compare_to_exact_msd_gap` | 18.55 ns | 18.49 ns - 18.62 ns | 18.44 ns | -1.20% | - |
| `computable_compare/compare_to_opposite_sign` | 11.08 ns | 11.08 ns - 11.09 ns | 11.07 ns | -1.00% | - |
| `computable_transcendentals/acos_cached_p96` | 19.04 ns | 18.99 ns - 19.10 ns | 18.97 ns | -3.78% | - |
| `computable_transcendentals/acos_cold_p96` | 5.60 us | 5.58 us - 5.63 us | 5.57 us | -0.73% | - |
| `computable_transcendentals/acos_near_one_cold_p96` | 1.51 us | 1.51 us - 1.52 us | 1.51 us | -3.09% | - |
| `computable_transcendentals/acos_tiny_cold_p96` | 719.31 ns | 715.76 ns - 723.00 ns | 708.57 ns | -1.69% | - |
| `computable_transcendentals/acosh_cached_p128` | 19.46 ns | 19.42 ns - 19.50 ns | 19.40 ns | -8.42% | - |
| `computable_transcendentals/acosh_cold_p128` | 37.35 ns | 37.11 ns - 37.57 ns | 37.35 ns | -1.55% | - |
| `computable_transcendentals/asin_cached_p96` | 19.19 ns | 19.07 ns - 19.33 ns | 18.95 ns | -3.47% | - |
| `computable_transcendentals/asin_cold_p96` | 6.28 us | 6.25 us - 6.32 us | 6.25 us | +1.48% | - |
| `computable_transcendentals/asin_near_one_cold_p96` | 1.90 us | 1.89 us - 1.91 us | 1.89 us | +1.47% | - |
| `computable_transcendentals/asin_tiny_cold_p96` | 397.70 ns | 396.05 ns - 399.35 ns | 397.50 ns | +0.90% | - |
| `computable_transcendentals/asin_zero_cold_p96` | 21.37 ns | 21.12 ns - 21.59 ns | 21.33 ns | -16.68% | - |
| `computable_transcendentals/asinh_cached_p128` | 18.90 ns | 18.87 ns - 18.93 ns | 18.86 ns | -3.76% | - |
| `computable_transcendentals/asinh_cold_p128` | 9.52 us | 9.47 us - 9.59 us | 9.41 us | +2.55% | - |
| `computable_transcendentals/asinh_three_quarters_cold_p128` | 5.04 us | 5.03 us - 5.05 us | 5.04 us | +1.08% | - |
| `computable_transcendentals/asinh_zero_cold_p128` | 21.75 ns | 21.51 ns - 21.97 ns | 21.70 ns | -13.97% | - |
| `computable_transcendentals/asymmetric_product_bad_order_cold_p128` | 28.57 ns | 28.54 ns - 28.62 ns | 28.52 ns | +0.98% | - |
| `computable_transcendentals/atan_cached_p96` | 19.09 ns | 19.02 ns - 19.16 ns | 18.97 ns | -4.51% | - |
| `computable_transcendentals/atan_cold_p96` | 1.90 us | 1.89 us - 1.90 us | 1.90 us | +0.73% | - |
| `computable_transcendentals/atan_large_cold_p96` | 1.64 us | 1.63 us - 1.65 us | 1.63 us | -1.64% | - |
| `computable_transcendentals/atan_zero_cold_p96` | 21.68 ns | 21.37 ns - 21.97 ns | 21.71 ns | -15.44% | - |
| `computable_transcendentals/atanh_cached_p128` | 18.87 ns | 18.85 ns - 18.89 ns | 18.87 ns | -3.85% | - |
| `computable_transcendentals/atanh_cold_p128` | 147.44 ns | 146.73 ns - 148.29 ns | 147.67 ns | -0.31% | - |
| `computable_transcendentals/atanh_near_one_cold_p128` | 2.19 us | 2.18 us - 2.20 us | 2.18 us | -0.08% | - |
| `computable_transcendentals/atanh_tiny_cold_p128` | 484.02 ns | 482.70 ns - 485.41 ns | 482.67 ns | -1.80% | - |
| `computable_transcendentals/atanh_zero_cold_p128` | 21.16 ns | 20.93 ns - 21.37 ns | 21.09 ns | -16.32% | - |
| `computable_transcendentals/cos_1e30_cold_p96` | 2.22 us | 2.22 us - 2.23 us | 2.21 us | -0.35% | - |
| `computable_transcendentals/cos_1e6_cold_p96` | 2.33 us | 2.32 us - 2.35 us | 2.30 us | +0.47% | - |
| `computable_transcendentals/cos_cached_p96` | 18.93 ns | 18.90 ns - 18.96 ns | 18.91 ns | -3.37% | - |
| `computable_transcendentals/cos_cold_p96` | 1.46 us | 1.45 us - 1.46 us | 1.45 us | -3.72% | - |
| `computable_transcendentals/cos_f64_cold_p96` | 1.65 us | 1.64 us - 1.66 us | 1.64 us | -3.83% | - |
| `computable_transcendentals/cos_huge_cold_p96` | 1.47 us | 1.46 us - 1.47 us | 1.46 us | -3.72% | - |
| `computable_transcendentals/cos_zero_cold_p96` | 60.07 ns | 59.56 ns - 60.57 ns | 59.63 ns | +9.63% | - |
| `computable_transcendentals/deep_add_chain_cold_p128` | 42.77 ns | 42.70 ns - 42.85 ns | 42.67 ns | +2.31% | - |
| `computable_transcendentals/deep_half_product_chain_cold_p128` | 17.64 ns | 17.56 ns - 17.74 ns | 17.48 ns | +9.38% | - |
| `computable_transcendentals/deep_half_square_chain_cold_p128` | 28.39 ns | 28.33 ns - 28.48 ns | 28.30 ns | +3.48% | - |
| `computable_transcendentals/deep_inverse_pair_chain_cold_p128` | 66.14 ns | 66.01 ns - 66.32 ns | 65.98 ns | +1.87% | - |
| `computable_transcendentals/deep_multiply_chain_cold_p128` | 42.90 ns | 42.73 ns - 43.10 ns | 42.60 ns | +2.08% | - |
| `computable_transcendentals/deep_multiply_identity_chain_cold_p128` | 66.44 ns | 66.26 ns - 66.66 ns | 66.13 ns | +1.55% | - |
| `computable_transcendentals/deep_negated_square_chain_cold_p128` | 66.33 ns | 66.20 ns - 66.48 ns | 66.12 ns | +2.79% | - |
| `computable_transcendentals/deep_negative_one_product_chain_cold_p128` | 67.48 ns | 67.12 ns - 67.90 ns | 66.70 ns | +0.43% | - |
| `computable_transcendentals/deep_scaled_product_chain_cold_p128` | 28.19 ns | 28.16 ns - 28.21 ns | 28.15 ns | +4.43% | - |
| `computable_transcendentals/deep_sqrt_square_chain_cold_p128` | 42.74 ns | 42.58 ns - 42.98 ns | 42.49 ns | +2.10% | - |
| `computable_transcendentals/e_constant_cached_p128` | 19.28 ns | 19.25 ns - 19.32 ns | 19.25 ns | -9.22% | - |
| `computable_transcendentals/e_constant_cold_p128` | 36.85 ns | 36.55 ns - 37.16 ns | 36.57 ns | -6.86% | - |
| `computable_transcendentals/exp_cached_p128` | 19.48 ns | 19.40 ns - 19.58 ns | 19.33 ns | -2.38% | - |
| `computable_transcendentals/exp_coarse_cold_p1` | 2.32 us | 2.31 us - 2.34 us | 2.29 us | +8.12% | - |
| `computable_transcendentals/exp_cold_p128` | 4.08 us | 4.08 us - 4.09 us | 4.08 us | -0.26% | - |
| `computable_transcendentals/exp_deferred_constructor` | 29.80 ns | 29.62 ns - 30.02 ns | 29.46 ns | -11.33% | - |
| `computable_transcendentals/exp_deferred_negative_cold_p128` | 97.11 us | 96.85 us - 97.42 us | 96.82 us | +0.47% | - |
| `computable_transcendentals/exp_half_cold_p128` | 2.98 us | 2.97 us - 2.99 us | 2.96 us | -1.20% | - |
| `computable_transcendentals/exp_integer_above_limit_cold_p128` | 11.89 us | 11.87 us - 11.91 us | 11.86 us | -1.92% | - |
| `computable_transcendentals/exp_integer_limit_cold_p128` | 6.37 us | 6.36 us - 6.39 us | 6.38 us | -0.18% | - |
| `computable_transcendentals/exp_large_cold_p128` | 4.45 us | 4.44 us - 4.46 us | 4.44 us | -2.14% | - |
| `computable_transcendentals/exp_near_limit_cached_p128` | 18.96 ns | 18.94 ns - 18.99 ns | 18.94 ns | -3.70% | - |
| `computable_transcendentals/exp_near_limit_cold_p128` | 2.85 us | 2.85 us - 2.85 us | 2.85 us | -4.82% | - |
| `computable_transcendentals/exp_negative_integer_cold_p128` | 2.13 us | 2.12 us - 2.13 us | 2.12 us | -0.58% | - |
| `computable_transcendentals/exp_zero_cold_p128` | 57.85 ns | 57.26 ns - 58.44 ns | 58.43 ns | +11.25% | - |
| `computable_transcendentals/expm1_coarse_cold_p1` | 3.07 us | 3.07 us - 3.08 us | 3.07 us | +2.17% | - |
| `computable_transcendentals/expm1_coarse_negative_cold_p1` | 53.77 ns | 53.05 ns - 54.49 ns | 54.00 ns | +13.01% | - |
| `computable_transcendentals/expm1_tiny_cold_p128` | 836.57 ns | 833.67 ns - 839.96 ns | 834.55 ns | +2.05% | - |
| `computable_transcendentals/inverse_half_product_chain_cold_p128` | 29.22 ns | 29.03 ns - 29.44 ns | 28.93 ns | +2.73% | - |
| `computable_transcendentals/inverse_scaled_product_chain_cold_p128` | 28.41 ns | 28.38 ns - 28.44 ns | 28.37 ns | +1.89% | - |
| `computable_transcendentals/ln_cached_p128` | 18.93 ns | 18.91 ns - 18.96 ns | 18.90 ns | -3.39% | - |
| `computable_transcendentals/ln_cold_p128` | 3.10 us | 3.09 us - 3.11 us | 3.09 us | +0.25% | - |
| `computable_transcendentals/ln_large_cached_p128` | 19.06 ns | 19.01 ns - 19.11 ns | 18.99 ns | -5.40% | - |
| `computable_transcendentals/ln_large_cold_p128` | 946.20 ns | 938.23 ns - 954.50 ns | 923.11 ns | +0.19% | - |
| `computable_transcendentals/ln_near_limit_cached_p128` | 18.95 ns | 18.92 ns - 18.99 ns | 18.90 ns | -4.60% | - |
| `computable_transcendentals/ln_near_limit_cold_p128` | 3.20 us | 3.20 us - 3.20 us | 3.19 us | -0.74% | - |
| `computable_transcendentals/ln_nonsmooth_rational_cold_p128` | 2.52 us | 2.51 us - 2.53 us | 2.50 us | -1.08% | - |
| `computable_transcendentals/ln_one_cold_p128` | 21.36 ns | 21.13 ns - 21.56 ns | 21.27 ns | -15.35% | - |
| `computable_transcendentals/ln_smooth_rational_cold_p128` | 703.02 ns | 697.64 ns - 708.73 ns | 689.07 ns | +3.84% | - |
| `computable_transcendentals/ln_tiny_cold_p128` | 212.91 ns | 211.12 ns - 214.68 ns | 215.51 ns | -2.81% | - |
| `computable_transcendentals/perturbed_scaled_product_chain_cold_p128` | 28.70 ns | 28.43 ns - 29.00 ns | 28.17 ns | +5.04% | - |
| `computable_transcendentals/scaled_square_chain_cold_p128` | 28.48 ns | 28.42 ns - 28.55 ns | 28.39 ns | -1.38% | - |
| `computable_transcendentals/sin_1e30_cold_p96` | 2.14 us | 2.13 us - 2.14 us | 2.13 us | +0.14% | - |
| `computable_transcendentals/sin_1e6_cold_p96` | 2.31 us | 2.30 us - 2.32 us | 2.30 us | +1.36% | - |
| `computable_transcendentals/sin_cached_p96` | 21.90 ns | 21.83 ns - 21.97 ns | 21.75 ns | +11.43% | - |
| `computable_transcendentals/sin_cold_p96` | 1.57 us | 1.57 us - 1.58 us | 1.57 us | -0.24% | - |
| `computable_transcendentals/sin_f64_cold_p96` | 1.74 us | 1.73 us - 1.75 us | 1.73 us | +1.33% | - |
| `computable_transcendentals/sin_huge_cold_p96` | 1.56 us | 1.56 us - 1.57 us | 1.56 us | -0.15% | - |
| `computable_transcendentals/sin_zero_cold_p96` | 21.87 ns | 21.59 ns - 22.13 ns | 21.90 ns | -13.67% | - |
| `computable_transcendentals/sqrt_cached_p128` | 18.96 ns | 18.93 ns - 18.99 ns | 18.92 ns | -3.20% | - |
| `computable_transcendentals/sqrt_cold_p128` | 743.12 ns | 740.41 ns - 745.76 ns | 744.01 ns | +2.42% | - |
| `computable_transcendentals/sqrt_scaled_square_chain_cold_p128` | 437.02 ns | 433.41 ns - 440.98 ns | 425.80 ns | -4.70% | - |
| `computable_transcendentals/sqrt_single_scaled_square_cold_p128` | 838.16 ns | 836.70 ns - 839.77 ns | 836.62 ns | +0.74% | - |
| `computable_transcendentals/sqrt_squarefree_scaled_cold_p128` | 109.20 ns | 108.54 ns - 109.88 ns | 109.96 ns | -1.30% | - |
| `computable_transcendentals/tan_cached_p96` | 18.94 ns | 18.92 ns - 18.97 ns | 18.91 ns | -3.38% | - |
| `computable_transcendentals/tan_cold_p96` | 5.97 us | 5.96 us - 5.98 us | 5.97 us | -1.79% | - |
| `computable_transcendentals/tan_huge_cold_p96` | 6.08 us | 6.05 us - 6.11 us | 6.05 us | +0.13% | - |
| `computable_transcendentals/tan_near_half_pi_cached_p96` | 18.92 ns | 18.90 ns - 18.94 ns | 18.89 ns | -3.34% | - |
| `computable_transcendentals/tan_near_half_pi_cold_p96` | 10.36 us | 10.30 us - 10.44 us | 10.26 us | +2.70% | - |
| `computable_transcendentals/tan_zero_cold_p96` | 21.61 ns | 21.35 ns - 21.85 ns | 21.47 ns | -14.54% | - |
| `computable_transcendentals/warmed_zero_product_cold_p128` | 15.57 ns | 15.50 ns - 15.67 ns | 15.46 ns | +13.43% | - |
| `construction_speed/computable_one` | 16.91 ns | 16.84 ns - 16.99 ns | 16.80 ns | -1.94% | - |
| `construction_speed/rational_from_i8_minus_four` | 3.96 ns | 3.95 ns - 3.96 ns | 3.95 ns | -0.60% | - |
| `construction_speed/rational_from_u8_four` | 3.78 ns | 3.77 ns - 3.80 ns | 3.76 ns | +1.13% | - |
| `construction_speed/rational_new_one` | 3.27 ns | 3.26 ns - 3.28 ns | 3.26 ns | -1.71% | - |
| `construction_speed/rational_one` | 3.09 ns | 3.08 ns - 3.10 ns | 3.10 ns | +0.06% | - |
| `construction_speed/real_from_i32_one` | 9.53 ns | 9.49 ns - 9.58 ns | 9.47 ns | -0.28% | - |
| `construction_speed/real_from_i8_minus_four` | 10.57 ns | 10.51 ns - 10.65 ns | 10.48 ns | +1.94% | - |
| `construction_speed/real_from_u8_four` | 10.33 ns | 10.31 ns - 10.35 ns | 10.30 ns | -0.13% | - |
| `construction_speed/real_new_rational_one` | 9.52 ns | 9.50 ns - 9.54 ns | 9.49 ns | +0.72% | - |
| `construction_speed/real_one` | 9.76 ns | 9.74 ns - 9.78 ns | 9.74 ns | +2.23% | - |
| `dense_algebra/rational_dot_64` | 1.69 us | 1.69 us - 1.69 us | 1.68 us | +5.34% | - |
| `dense_algebra/rational_matmul_8` | 54.98 us | 54.89 us - 55.08 us | 54.95 us | +0.13% | - |
| `dense_algebra/real_dot_36` | 4.42 us | 4.41 us - 4.42 us | 4.42 us | -1.86% | - |
| `dense_algebra/real_matmul_6` | 45.70 us | 45.66 us - 45.74 us | 45.68 us | -0.43% | - |
| `dense_algebra/real_sum_owned_1024_symbolic` | 122.16 us | 121.93 us - 122.40 us | 122.14 us | +0.28% | - |
| `dense_algebra/real_sum_owned_1024_symbolic_former_clone_path` | 151.00 us | 150.42 us - 151.73 us | 150.27 us | +0.75% | - |
| `dense_algebra/real_sum_refs_1024_rational` | 35.74 us | 35.71 us - 35.78 us | 35.72 us | -2.38% | - |
| `dense_algebra/real_sum_refs_1024_rational_sequential` | 21.46 us | 21.37 us - 21.55 us | 21.69 us | -2.61% | - |
| `dense_algebra/real_sum_refs_1024_symbolic` | 151.65 us | 151.46 us - 151.87 us | 151.60 us | -0.54% | - |
| `dense_algebra/real_sum_refs_1024_symbolic_sequential` | 114.99 us | 114.73 us - 115.27 us | 114.75 us | -2.58% | - |
| `dense_algebra/real_sum_refs_1024_symbolic_to_f64` | 1.22 ms | 1.22 ms - 1.23 ms | 1.23 ms | -2.27% | - |
| `dense_algebra/real_sum_refs_1024_symbolic_to_f64_sequential` | 5.14 ms | 5.14 ms - 5.15 ms | 5.13 ms | -0.63% | - |
| `dense_algebra/real_sum_refs_64_symbolic` | 6.69 us | 6.67 us - 6.70 us | 6.67 us | -1.06% | - |
| `dense_algebra/real_sum_refs_64_symbolic_to_f64` | 32.26 us | 32.24 us - 32.29 us | 32.26 us | -0.35% | - |
| `exact_product_sums/exact_rational_sparse_homogeneous_plane_intersection3` | 209.96 ns | 208.91 ns - 211.12 ns | 207.50 ns | +3.06% | - |
| `exact_product_sums/real_signed_product_sum_mixed_symbolic_det3` | 2.09 us | 2.09 us - 2.09 us | 2.09 us | -2.79% | - |
| `exact_product_sums/real_signed_product_sum_rational_det3` | 301.32 ns | 300.61 ns - 302.14 ns | 300.32 ns | -1.97% | - |
| `exact_product_sums/signed_product_sum_common_scale_6x2` | 171.33 ns | 170.54 ns - 172.33 ns | 170.01 ns | -1.82% | - |
| `exact_product_sums/signed_product_sum_lcm_6x2` | 320.79 ns | 320.38 ns - 321.29 ns | 320.31 ns | -2.27% | - |
| `exact_product_sums/signed_product_sum_sparse_single_6x2` | 102.84 ns | 102.58 ns - 103.12 ns | 102.51 ns | -3.04% | - |
| `exact_transcendental_special_forms/acos_cos_9pi_7` | 889.12 ns | 886.57 ns - 893.12 ns | 886.11 ns | -2.51% | - |
| `exact_transcendental_special_forms/asin_sin_6pi_7` | 905.78 ns | 904.25 ns - 907.31 ns | 905.61 ns | -1.01% | - |
| `exact_transcendental_special_forms/asinh_large` | 188.12 ns | 187.26 ns - 189.07 ns | 186.29 ns | -4.43% | - |
| `exact_transcendental_special_forms/atan2_axis_negative_x` | 29.75 ns | 29.69 ns - 29.83 ns | 29.70 ns | -5.90% | - |
| `exact_transcendental_special_forms/atan2_axis_positive_y` | 31.89 ns | 31.81 ns - 31.99 ns | 31.77 ns | -0.51% | - |
| `exact_transcendental_special_forms/atan2_origin` | 18.42 ns | 18.41 ns - 18.44 ns | 18.43 ns | -2.13% | - |
| `exact_transcendental_special_forms/atan2_quadrant_one_unit_diagonal` | 74.95 ns | 74.81 ns - 75.14 ns | 74.76 ns | +0.10% | - |
| `exact_transcendental_special_forms/atan2_quadrant_three_negative_pi` | 700.39 ns | 698.71 ns - 702.30 ns | 697.80 ns | -3.39% | - |
| `exact_transcendental_special_forms/atan2_quadrant_two_pi_correction` | 775.64 ns | 773.77 ns - 777.75 ns | 773.74 ns | -3.50% | - |
| `exact_transcendental_special_forms/atan_tan_6pi_7` | 479.89 ns | 477.82 ns - 482.26 ns | 476.74 ns | -7.82% | - |
| `exact_transcendental_special_forms/atanh_sqrt_half` | 85.60 ns | 85.29 ns - 86.00 ns | 85.07 ns | -1.95% | - |
| `exact_transcendental_special_forms/atanh_sqrt_two_error` | 17.15 ns | 17.13 ns - 17.17 ns | 17.12 ns | -5.97% | - |
| `exact_transcendental_special_forms/cos_pi_7` | 647.14 ns | 643.94 ns - 651.10 ns | 641.36 ns | -1.08% | - |
| `exact_transcendental_special_forms/cosh_ln_two` | 131.53 ns | 131.21 ns - 131.86 ns | 131.34 ns | +2.90% | - |
| `exact_transcendental_special_forms/cosh_rational_one` | 355.12 ns | 353.69 ns - 356.72 ns | 352.56 ns | -1.01% | - |
| `exact_transcendental_special_forms/log2_ln_quotient_fold` | 145.50 ns | 144.98 ns - 146.08 ns | 144.41 ns | -3.88% | - |
| `exact_transcendental_special_forms/log2_power_of_two` | 60.61 ns | 60.49 ns - 60.76 ns | 60.44 ns | -1.26% | - |
| `exact_transcendental_special_forms/log2_rational_three` | 131.06 ns | 130.45 ns - 131.76 ns | 129.84 ns | -5.61% | - |
| `exact_transcendental_special_forms/sin_pi_7` | 782.38 ns | 780.01 ns - 784.94 ns | 778.93 ns | -3.41% | - |
| `exact_transcendental_special_forms/sinh_ln_two` | 125.63 ns | 125.11 ns - 126.28 ns | 124.86 ns | -1.52% | - |
| `exact_transcendental_special_forms/sinh_rational_one` | 419.64 ns | 409.40 ns - 438.52 ns | 407.95 ns | +0.86% | - |
| `exact_transcendental_special_forms/tan_pi_7` | 235.23 ns | 233.97 ns - 236.76 ns | 233.42 ns | -1.24% | - |
| `exact_transcendental_special_forms/tanh_ln_two` | 249.37 ns | 248.77 ns - 250.10 ns | 248.67 ns | +0.87% | - |
| `exact_transcendental_special_forms/tanh_rational_one` | 581.22 ns | 572.99 ns - 595.70 ns | 571.24 ns | -1.60% | - |
| `float_convert/f32_normal` | 46.17 ns | 45.91 ns - 46.45 ns | 45.51 ns | +2.34% | - |
| `float_convert/f64_binary_fraction` | 9.99 ns | 9.97 ns - 10.03 ns | 9.96 ns | +2.57% | - |
| `float_convert/f64_enclosure_exact` | 7.33 ns | 7.32 ns - 7.35 ns | 7.31 ns | +0.61% | - |
| `float_convert/f64_enclosure_near_max` | 13.26 ns | 13.23 ns - 13.29 ns | 13.20 ns | +0.37% | - |
| `float_convert/f64_enclosure_rounded` | 10.23 ns | 10.20 ns - 10.28 ns | 10.17 ns | +0.99% | - |
| `float_convert/f64_normal` | 46.35 ns | 46.23 ns - 46.52 ns | 46.17 ns | -1.03% | - |
| `float_convert/f64_subnormal` | 54.41 ns | 53.98 ns - 54.91 ns | 53.50 ns | +2.26% | - |
| `float_convert/real_f32_normal` | 65.22 ns | 65.15 ns - 65.28 ns | 65.15 ns | +0.46% | - |
| `float_convert/real_f64_binary_fraction` | 22.05 ns | 22.01 ns - 22.11 ns | 21.98 ns | +0.80% | - |
| `float_convert/real_f64_normal` | 67.22 ns | 67.00 ns - 67.50 ns | 66.98 ns | +6.27% | - |
| `float_convert/real_f64_subnormal` | 71.37 ns | 71.21 ns - 71.56 ns | 71.11 ns | -0.66% | - |
| `gmp_computable_api_p128/gmp_mpfr128/acos` | 3.24 us | 3.22 us - 3.25 us | 3.21 us | +1.57% | - |
| `gmp_computable_api_p128/gmp_mpfr128/acosh` | 1.69 us | 1.68 us - 1.71 us | 1.67 us | +3.92% | - |
| `gmp_computable_api_p128/gmp_mpfr128/add` | 29.82 ns | 29.63 ns - 30.04 ns | 29.42 ns | +1.37% | - |
| `gmp_computable_api_p128/gmp_mpfr128/asin` | 3.25 us | 3.24 us - 3.27 us | 3.22 us | +1.08% | - |
| `gmp_computable_api_p128/gmp_mpfr128/asinh` | 1.74 us | 1.72 us - 1.75 us | 1.70 us | +4.42% | - |
| `gmp_computable_api_p128/gmp_mpfr128/atan` | 2.85 us | 2.84 us - 2.86 us | 2.83 us | +1.20% | - |
| `gmp_computable_api_p128/gmp_mpfr128/atan2` | 2.53 us | 2.52 us - 2.55 us | 2.50 us | +2.30% | - |
| `gmp_computable_api_p128/gmp_mpfr128/atanh` | 1.70 us | 1.68 us - 1.72 us | 1.67 us | +3.78% | - |
| `gmp_computable_api_p128/gmp_mpfr128/compare_absolute` | 2.93 ns | 2.87 ns - 3.00 ns | 2.79 ns | +12.29% | - |
| `gmp_computable_api_p128/gmp_mpfr128/cos` | 513.91 ns | 511.02 ns - 517.08 ns | 508.20 ns | +2.35% | - |
| `gmp_computable_api_p128/gmp_mpfr128/dnorm` | 1.21 us | 1.20 us - 1.22 us | 1.19 us | +0.70% | - |
| `gmp_computable_api_p128/gmp_mpfr128/e` | 17.98 ns | 17.89 ns - 18.07 ns | 17.81 ns | +1.43% | - |
| `gmp_computable_api_p128/gmp_mpfr128/erf` | 3.34 us | 3.33 us - 3.36 us | 3.32 us | +1.06% | - |
| `gmp_computable_api_p128/gmp_mpfr128/erfc` | 3.70 us | 3.67 us - 3.73 us | 3.62 us | +5.25% | - |
| `gmp_computable_api_p128/gmp_mpfr128/erfcx` | 4.76 us | 4.71 us - 4.82 us | 4.65 us | +4.17% | - |
| `gmp_computable_api_p128/gmp_mpfr128/exp` | 925.33 ns | 920.71 ns - 930.17 ns | 919.88 ns | -0.32% | - |
| `gmp_computable_api_p128/gmp_mpfr128/expm1` | 1.05 us | 1.04 us - 1.05 us | 1.04 us | -0.80% | - |
| `gmp_computable_api_p128/gmp_mpfr128/inverse` | 60.16 ns | 59.81 ns - 60.53 ns | 59.41 ns | +2.31% | - |
| `gmp_computable_api_p128/gmp_mpfr128/ln` | 1.33 us | 1.32 us - 1.34 us | 1.32 us | +2.28% | - |
| `gmp_computable_api_p128/gmp_mpfr128/log_dnorm` | 2.59 us | 2.57 us - 2.62 us | 2.55 us | +2.12% | - |
| `gmp_computable_api_p128/gmp_mpfr128/log_normal_sf` | 5.68 us | 5.65 us - 5.71 us | 5.61 us | +1.26% | - |
| `gmp_computable_api_p128/gmp_mpfr128/log_pnorm` | 5.74 us | 5.70 us - 5.78 us | 5.68 us | +2.29% | - |
| `gmp_computable_api_p128/gmp_mpfr128/multiply` | 37.47 ns | 37.24 ns - 37.75 ns | 37.09 ns | +1.17% | - |
| `gmp_computable_api_p128/gmp_mpfr128/negate` | 13.36 ns | 13.28 ns - 13.44 ns | 13.17 ns | +4.43% | - |
| `gmp_computable_api_p128/gmp_mpfr128/normal_interval` | 7.98 us | 7.89 us - 8.08 us | 7.78 us | +1.40% | - |
| `gmp_computable_api_p128/gmp_mpfr128/normal_quantile` | 46.96 us | 46.48 us - 47.53 us | 46.10 us | +1.29% | - |
| `gmp_computable_api_p128/gmp_mpfr128/normal_sf` | 3.26 us | 3.23 us - 3.29 us | 3.21 us | +3.95% | - |
| `gmp_computable_api_p128/gmp_mpfr128/pi` | 17.87 ns | 17.81 ns - 17.95 ns | 17.75 ns | +2.62% | - |
| `gmp_computable_api_p128/gmp_mpfr128/pnorm` | 3.20 us | 3.19 us - 3.22 us | 3.17 us | +2.83% | - |
| `gmp_computable_api_p128/gmp_mpfr128/sign_until` | 0.49 ns | 0.48 ns - 0.49 ns | 0.48 ns | +2.14% | - |
| `gmp_computable_api_p128/gmp_mpfr128/sign_until_floor_2000` | 0.48 ns | 0.48 ns - 0.48 ns | 0.48 ns | +1.38% | - |
| `gmp_computable_api_p128/gmp_mpfr128/sin` | 736.29 ns | 730.69 ns - 742.82 ns | 725.35 ns | +3.33% | - |
| `gmp_computable_api_p128/gmp_mpfr128/sqrt` | 105.46 ns | 104.82 ns - 106.20 ns | 104.32 ns | -4.68% | - |
| `gmp_computable_api_p128/gmp_mpfr128/square` | 40.09 ns | 39.95 ns - 40.26 ns | 39.81 ns | +2.12% | - |
| `gmp_computable_api_p128/gmp_mpfr128/tan` | 991.94 ns | 985.71 ns - 998.78 ns | 980.72 ns | +3.27% | - |
| `gmp_computable_api_p128/gmp_mpfr128/tau` | 17.97 ns | 17.85 ns - 18.10 ns | 17.71 ns | +1.80% | - |
| `gmp_computable_api_p128/gmp_mpfr128/try_compare_to` | 3.07 ns | 3.06 ns - 3.08 ns | 3.07 ns | -0.96% | - |
| `gmp_computable_api_p128/gmp_mpfr128/zero_status` | 1.91 ns | 1.84 ns - 1.97 ns | 1.72 ns | +14.28% | - |
| `gmp_computable_api_p128/hyperreal/acos` | 6.36 us | 6.31 us - 6.41 us | 6.28 us | +2.12% | - |
| `gmp_computable_api_p128/hyperreal/acosh` | 12.87 us | 12.80 us - 12.96 us | 12.76 us | +2.42% | - |
| `gmp_computable_api_p128/hyperreal/add` | 133.92 ns | 132.24 ns - 136.39 ns | 131.10 ns | +0.50% | - |
| `gmp_computable_api_p128/hyperreal/asin` | 6.56 us | 6.54 us - 6.59 us | 6.52 us | +1.45% | - |
| `gmp_computable_api_p128/hyperreal/asinh` | 6.20 us | 6.14 us - 6.27 us | 6.06 us | +2.64% | - |
| `gmp_computable_api_p128/hyperreal/atan` | 2.74 us | 2.73 us - 2.76 us | 2.71 us | +2.57% | - |
| `gmp_computable_api_p128/hyperreal/atan2` | 6.24 us | 6.21 us - 6.27 us | 6.19 us | +0.47% | - |
| `gmp_computable_api_p128/hyperreal/atanh` | 460.38 ns | 456.17 ns - 464.92 ns | 451.78 ns | -6.96% | - |
| `gmp_computable_api_p128/hyperreal/compare_absolute` | 7.62 ns | 7.57 ns - 7.67 ns | 7.51 ns | +9.97% | - |
| `gmp_computable_api_p128/hyperreal/cos` | 2.12 us | 2.11 us - 2.14 us | 2.10 us | +2.50% | - |
| `gmp_computable_api_p128/hyperreal/dnorm` | 7.44 us | 7.41 us - 7.48 us | 7.39 us | +1.09% | - |
| `gmp_computable_api_p128/hyperreal/e` | 20.04 ns | 19.95 ns - 20.13 ns | 19.94 ns | +3.85% | - |
| `gmp_computable_api_p128/hyperreal/erf` | 31.16 us | 31.06 us - 31.28 us | 31.04 us | +1.29% | - |
| `gmp_computable_api_p128/hyperreal/erfc` | 32.56 us | 32.46 us - 32.68 us | 32.38 us | +0.18% | - |
| `gmp_computable_api_p128/hyperreal/erfcx` | 66.96 us | 66.44 us - 67.59 us | 66.03 us | +2.29% | - |
| `gmp_computable_api_p128/hyperreal/exp` | 4.41 us | 4.39 us - 4.45 us | 4.36 us | +3.57% | - |
| `gmp_computable_api_p128/hyperreal/expm1` | 4.78 us | 4.73 us - 4.84 us | 4.67 us | +4.24% | - |
| `gmp_computable_api_p128/hyperreal/inverse` | 127.52 ns | 126.70 ns - 128.40 ns | 125.99 ns | -3.55% | - |
| `gmp_computable_api_p128/hyperreal/ln` | 1.19 us | 1.19 us - 1.20 us | 1.18 us | -4.34% | - |
| `gmp_computable_api_p128/hyperreal/log_dnorm` | 6.87 us | 6.82 us - 6.92 us | 6.76 us | +0.28% | - |
| `gmp_computable_api_p128/hyperreal/log_normal_sf` | 85.44 us | 84.54 us - 86.55 us | 83.62 us | +3.13% | - |
| `gmp_computable_api_p128/hyperreal/log_pnorm` | 51.79 us | 51.53 us - 52.08 us | 51.42 us | +2.82% | - |
| `gmp_computable_api_p128/hyperreal/multiply` | 145.23 ns | 143.85 ns - 146.78 ns | 142.60 ns | +5.77% | - |
| `gmp_computable_api_p128/hyperreal/negate` | 112.28 ns | 111.54 ns - 113.10 ns | 110.75 ns | +5.30% | - |
| `gmp_computable_api_p128/hyperreal/normal_interval` | 73.81 us | 73.12 us - 74.63 us | 72.35 us | +2.75% | - |
| `gmp_computable_api_p128/hyperreal/normal_quantile` | 271.53 us | 269.36 us - 274.01 us | 267.23 us | +2.44% | - |
| `gmp_computable_api_p128/hyperreal/normal_sf` | 30.56 us | 30.38 us - 30.76 us | 30.24 us | +0.69% | - |
| `gmp_computable_api_p128/hyperreal/pi` | 58.64 ns | 58.40 ns - 58.91 ns | 58.11 ns | +1.81% | - |
| `gmp_computable_api_p128/hyperreal/pnorm` | 31.14 us | 30.88 us - 31.44 us | 30.72 us | +5.68% | - |
| `gmp_computable_api_p128/hyperreal/sign_until` | 4.08 ns | 4.07 ns - 4.10 ns | 4.06 ns | -0.72% | - |
| `gmp_computable_api_p128/hyperreal/sign_until_floor_2000` | 4.08 ns | 4.07 ns - 4.10 ns | 4.06 ns | +0.91% | - |
| `gmp_computable_api_p128/hyperreal/sin` | 2.11 us | 2.10 us - 2.12 us | 2.09 us | +0.92% | - |
| `gmp_computable_api_p128/hyperreal/sqrt` | 322.92 ns | 319.54 ns - 326.73 ns | 316.55 ns | -0.84% | - |
| `gmp_computable_api_p128/hyperreal/square` | 121.01 ns | 120.34 ns - 121.75 ns | 119.95 ns | +7.11% | - |
| `gmp_computable_api_p128/hyperreal/tan` | 8.73 us | 8.69 us - 8.78 us | 8.69 us | +3.84% | - |
| `gmp_computable_api_p128/hyperreal/tau` | 20.68 ns | 20.52 ns - 20.86 ns | 20.50 ns | +5.68% | - |
| `gmp_computable_api_p128/hyperreal/try_compare_to` | 17.43 ns | 17.33 ns - 17.54 ns | 17.18 ns | -0.28% | - |
| `gmp_computable_api_p128/hyperreal/zero_status` | 2.66 ns | 2.65 ns - 2.68 ns | 2.64 ns | +2.54% | - |
| `gmp_magnitude_algorithms/gmp_div/16384` | 31.54 us | 31.40 us - 31.69 us | 31.25 us | +0.37% | - |
| `gmp_magnitude_algorithms/gmp_div/4096` | 2.66 us | 2.64 us - 2.67 us | 2.63 us | +1.76% | - |
| `gmp_magnitude_algorithms/gmp_div/65536` | 250.49 us | 249.69 us - 251.43 us | 249.06 us | -0.74% | - |
| `gmp_magnitude_algorithms/gmp_mul/16384` | 13.34 us | 13.18 us - 13.56 us | 13.06 us | +3.06% | - |
| `gmp_magnitude_algorithms/gmp_mul/4096` | 1.52 us | 1.51 us - 1.54 us | 1.50 us | +3.64% | - |
| `gmp_magnitude_algorithms/gmp_mul/65536` | 81.81 us | 81.77 us - 81.86 us | 81.77 us | -1.43% | - |
| `gmp_magnitude_algorithms/gmp_roundtrip_div/16384` | 48.14 us | 47.90 us - 48.41 us | 47.53 us | +0.97% | - |
| `gmp_magnitude_algorithms/gmp_roundtrip_div/4096` | 6.86 us | 6.82 us - 6.91 us | 6.79 us | +1.42% | - |
| `gmp_magnitude_algorithms/gmp_roundtrip_div/65536` | 314.85 us | 313.92 us - 315.96 us | 313.21 us | -0.51% | - |
| `gmp_magnitude_algorithms/gmp_roundtrip_mul/16384` | 29.81 us | 29.53 us - 30.11 us | 29.26 us | +1.99% | - |
| `gmp_magnitude_algorithms/gmp_roundtrip_mul/4096` | 5.59 us | 5.53 us - 5.65 us | 5.46 us | +1.62% | - |
| `gmp_magnitude_algorithms/gmp_roundtrip_mul/65536` | 144.76 us | 144.04 us - 145.93 us | 144.01 us | -0.29% | - |
| `gmp_magnitude_algorithms/num_bigint_div/16384` | 73.08 us | 72.80 us - 73.38 us | 72.74 us | +1.08% | - |
| `gmp_magnitude_algorithms/num_bigint_div/4096` | 4.92 us | 4.91 us - 4.95 us | 4.88 us | +0.35% | - |
| `gmp_magnitude_algorithms/num_bigint_div/65536` | 1.12 ms | 1.12 ms - 1.13 ms | 1.12 ms | +0.30% | - |
| `gmp_magnitude_algorithms/num_bigint_mul/16384` | 9.58 us | 9.54 us - 9.63 us | 9.49 us | -1.18% | - |
| `gmp_magnitude_algorithms/num_bigint_mul/4096` | 2.16 us | 2.15 us - 2.17 us | 2.14 us | +1.15% | - |
| `gmp_magnitude_algorithms/num_bigint_mul/65536` | 116.07 us | 115.44 us - 116.79 us | 114.67 us | +0.51% | - |
| `gmp_rational_api/gmp/add` | 0.94 ns | 0.94 ns - 0.94 ns | 0.94 ns | +0.45% | - |
| `gmp_rational_api/gmp/average_pair` | 81.95 ns | 81.48 ns - 82.44 ns | 81.14 ns | +2.67% | - |
| `gmp_rational_api/gmp/complex_product` | 398.44 ns | 394.79 ns - 402.46 ns | 390.16 ns | +3.99% | - |
| `gmp_rational_api/gmp/complex_quotient` | 733.55 ns | 729.35 ns - 738.29 ns | 726.19 ns | +1.29% | - |
| `gmp_rational_api/gmp/denominator` | 11.93 ns | 11.90 ns - 11.96 ns | 11.89 ns | -0.54% | - |
| `gmp_rational_api/gmp/div` | 0.94 ns | 0.93 ns - 0.94 ns | 0.93 ns | -1.54% | - |
| `gmp_rational_api/gmp/dot2` | 161.65 ns | 160.48 ns - 162.91 ns | 159.65 ns | +4.48% | - |
| `gmp_rational_api/gmp/extract_square_reduced` | 73.77 ns | 73.43 ns - 74.14 ns | 73.18 ns | +2.44% | - |
| `gmp_rational_api/gmp/extract_square_will_succeed` | 78.84 ns | 78.20 ns - 79.69 ns | 77.87 ns | +1.83% | - |
| `gmp_rational_api/gmp/fract` | 108.90 ns | 108.49 ns - 109.43 ns | 108.37 ns | -0.39% | - |
| `gmp_rational_api/gmp/from_bigint` | 50.52 ns | 50.17 ns - 50.89 ns | 49.74 ns | +4.41% | - |
| `gmp_rational_api/gmp/from_bigint_fraction` | 129.72 ns | 129.25 ns - 130.22 ns | 128.93 ns | -1.52% | - |
| `gmp_rational_api/gmp/from_fraction` | 36.33 ns | 36.19 ns - 36.49 ns | 36.09 ns | +1.49% | - |
| `gmp_rational_api/gmp/from_integer` | 17.05 ns | 16.99 ns - 17.11 ns | 16.95 ns | -0.38% | - |
| `gmp_rational_api/gmp/inverse` | 30.02 ns | 29.98 ns - 30.07 ns | 29.97 ns | -0.19% | - |
| `gmp_rational_api/gmp/is_dyadic` | 5.00 ns | 4.99 ns - 5.01 ns | 4.98 ns | +0.15% | - |
| `gmp_rational_api/gmp/is_integer` | 0.47 ns | 0.47 ns - 0.47 ns | 0.47 ns | -0.79% | - |
| `gmp_rational_api/gmp/is_negative` | 2.17 ns | 2.16 ns - 2.20 ns | 2.13 ns | -4.29% | - |
| `gmp_rational_api/gmp/is_one` | 8.35 ns | 8.31 ns - 8.39 ns | 8.26 ns | +1.22% | - |
| `gmp_rational_api/gmp/is_perfect_power` | 107.12 ns | 106.50 ns - 107.94 ns | 106.31 ns | -0.32% | - |
| `gmp_rational_api/gmp/is_positive` | 2.39 ns | 2.38 ns - 2.40 ns | 2.38 ns | -4.51% | - |
| `gmp_rational_api/gmp/is_zero` | 2.39 ns | 2.38 ns - 2.40 ns | 2.37 ns | +0.50% | - |
| `gmp_rational_api/gmp/mean3_refs` | 129.29 ns | 128.54 ns - 130.10 ns | 127.84 ns | +2.38% | - |
| `gmp_rational_api/gmp/mean_refs` | 238.15 ns | 236.71 ns - 239.72 ns | 235.54 ns | +2.60% | - |
| `gmp_rational_api/gmp/mul` | 0.95 ns | 0.94 ns - 0.96 ns | 0.94 ns | +1.75% | - |
| `gmp_rational_api/gmp/neg` | 0.47 ns | 0.47 ns - 0.47 ns | 0.47 ns | +0.56% | - |
| `gmp_rational_api/gmp/numerator` | 12.00 ns | 11.95 ns - 12.07 ns | 11.91 ns | +0.98% | - |
| `gmp_rational_api/gmp/one` | 17.24 ns | 17.16 ns - 17.33 ns | 17.08 ns | +0.80% | - |
| `gmp_rational_api/gmp/ordering` | 2.35 ns | 2.34 ns - 2.37 ns | 2.33 ns | +3.95% | - |
| `gmp_rational_api/gmp/ordering_wide_dyadic` | 38.67 ns | 38.47 ns - 38.89 ns | 38.43 ns | +4.20% | - |
| `gmp_rational_api/gmp/perfect_nth_root` | 106.79 ns | 106.32 ns - 107.34 ns | 105.93 ns | +0.82% | - |
| `gmp_rational_api/gmp/powi_17` | 101.43 ns | 101.15 ns - 101.78 ns | 101.10 ns | -0.42% | - |
| `gmp_rational_api/gmp/same_denominator` | 35.98 ns | 35.76 ns - 36.23 ns | 35.68 ns | +1.91% | - |
| `gmp_rational_api/gmp/shifted_big_integer` | 75.53 ns | 75.18 ns - 75.93 ns | 74.84 ns | +1.17% | - |
| `gmp_rational_api/gmp/sign` | 0.47 ns | 0.47 ns - 0.47 ns | 0.47 ns | +0.98% | - |
| `gmp_rational_api/gmp/signed_product_sum` | 217.51 ns | 215.62 ns - 219.52 ns | 212.22 ns | +4.65% | - |
| `gmp_rational_api/gmp/signed_product_sum_ordering` | 212.32 ns | 210.89 ns - 213.91 ns | 208.87 ns | +2.48% | - |
| `gmp_rational_api/gmp/signed_product_sum_shared_denominator` | 216.24 ns | 214.33 ns - 218.40 ns | 212.59 ns | +4.29% | - |
| `gmp_rational_api/gmp/sub` | 0.94 ns | 0.93 ns - 0.94 ns | 0.93 ns | +0.34% | - |
| `gmp_rational_api/gmp/to_f64` | 23.98 ns | 23.90 ns - 24.06 ns | 23.87 ns | +1.54% | - |
| `gmp_rational_api/gmp/to_f64_enclosure` | 128.23 ns | 127.67 ns - 128.84 ns | 127.37 ns | +2.53% | - |
| `gmp_rational_api/gmp/to_integer` | 1.19 ns | 1.19 ns - 1.20 ns | 1.19 ns | +1.70% | - |
| `gmp_rational_api/gmp/trunc` | 50.11 ns | 50.03 ns - 50.20 ns | 50.01 ns | +0.70% | - |
| `gmp_rational_api/gmp/zero` | 11.10 ns | 11.06 ns - 11.14 ns | 11.06 ns | +1.18% | - |
| `gmp_rational_api/hyperreal/add` | 6.16 ns | 6.11 ns - 6.21 ns | 6.06 ns | -1.24% | - |
| `gmp_rational_api/hyperreal/average_pair` | 79.58 ns | 79.08 ns - 80.16 ns | 78.69 ns | +3.36% | - |
| `gmp_rational_api/hyperreal/complex_product` | 203.94 ns | 202.90 ns - 205.07 ns | 201.57 ns | +14.23% | - |
| `gmp_rational_api/hyperreal/complex_quotient` | 213.01 ns | 210.93 ns - 215.48 ns | 209.19 ns | +2.97% | - |
| `gmp_rational_api/hyperreal/denominator` | 10.98 ns | 10.94 ns - 11.03 ns | 10.90 ns | +3.24% | - |
| `gmp_rational_api/hyperreal/div` | 92.80 ns | 92.38 ns - 93.26 ns | 91.89 ns | +5.03% | - |
| `gmp_rational_api/hyperreal/dot2` | 99.68 ns | 99.00 ns - 100.44 ns | 98.22 ns | +2.72% | - |
| `gmp_rational_api/hyperreal/extract_square_reduced` | 121.98 ns | 120.83 ns - 123.31 ns | 119.95 ns | +0.85% | - |
| `gmp_rational_api/hyperreal/extract_square_will_succeed` | 3.24 ns | 3.20 ns - 3.27 ns | 3.18 ns | +4.26% | - |
| `gmp_rational_api/hyperreal/fract` | 34.14 ns | 34.07 ns - 34.22 ns | 34.08 ns | +0.49% | - |
| `gmp_rational_api/hyperreal/from_bigint` | 30.18 ns | 30.04 ns - 30.33 ns | 29.87 ns | -2.22% | - |
| `gmp_rational_api/hyperreal/from_bigint_fraction` | 58.89 ns | 58.43 ns - 59.55 ns | 58.21 ns | -1.51% | - |
| `gmp_rational_api/hyperreal/from_fraction` | 68.97 ns | 68.64 ns - 69.35 ns | 68.47 ns | +0.37% | - |
| `gmp_rational_api/hyperreal/from_integer` | 3.62 ns | 3.59 ns - 3.65 ns | 3.55 ns | +2.34% | - |
| `gmp_rational_api/hyperreal/inverse` | 7.46 ns | 7.45 ns - 7.47 ns | 7.45 ns | -0.10% | - |
| `gmp_rational_api/hyperreal/is_dyadic` | 1.80 ns | 1.79 ns - 1.81 ns | 1.77 ns | -0.87% | - |
| `gmp_rational_api/hyperreal/is_integer` | 4.24 ns | 4.18 ns - 4.30 ns | 4.03 ns | -3.60% | - |
| `gmp_rational_api/hyperreal/is_negative` | 0.47 ns | 0.47 ns - 0.47 ns | 0.47 ns | +0.67% | - |
| `gmp_rational_api/hyperreal/is_one` | 2.87 ns | 2.85 ns - 2.88 ns | 2.84 ns | +0.92% | - |
| `gmp_rational_api/hyperreal/is_perfect_power` | 266.23 ns | 264.28 ns - 268.32 ns | 261.47 ns | +1.62% | - |
| `gmp_rational_api/hyperreal/is_positive` | 0.48 ns | 0.48 ns - 0.48 ns | 0.47 ns | +1.52% | - |
| `gmp_rational_api/hyperreal/is_zero` | 0.47 ns | 0.47 ns - 0.48 ns | 0.47 ns | +1.23% | - |
| `gmp_rational_api/hyperreal/mean3_refs` | 207.61 ns | 206.38 ns - 208.96 ns | 205.22 ns | -13.20% | - |
| `gmp_rational_api/hyperreal/mean_refs` | 129.61 ns | 128.67 ns - 130.67 ns | 127.84 ns | +1.71% | - |
| `gmp_rational_api/hyperreal/mul` | 11.92 ns | 11.89 ns - 11.97 ns | 11.88 ns | +3.22% | - |
| `gmp_rational_api/hyperreal/neg` | 8.91 ns | 8.88 ns - 8.95 ns | 8.86 ns | +2.10% | - |
| `gmp_rational_api/hyperreal/numerator` | 11.05 ns | 11.02 ns - 11.09 ns | 10.99 ns | +4.05% | - |
| `gmp_rational_api/hyperreal/one` | 3.06 ns | 3.05 ns - 3.07 ns | 3.07 ns | +0.32% | - |
| `gmp_rational_api/hyperreal/ordering` | 3.58 ns | 3.56 ns - 3.60 ns | 3.56 ns | +0.46% | - |
| `gmp_rational_api/hyperreal/ordering_wide_dyadic` | 13.92 ns | 13.82 ns - 14.03 ns | 13.71 ns | +1.96% | - |
| `gmp_rational_api/hyperreal/perfect_nth_root` | 163.13 ns | 161.63 ns - 164.78 ns | 159.81 ns | -1.21% | - |
| `gmp_rational_api/hyperreal/powi_17` | 307.39 ns | 306.27 ns - 308.66 ns | 307.55 ns | +1.64% | - |
| `gmp_rational_api/hyperreal/same_denominator` | 71.59 ns | 71.24 ns - 71.97 ns | 70.95 ns | +2.39% | - |
| `gmp_rational_api/hyperreal/shifted_big_integer` | 49.32 ns | 49.14 ns - 49.52 ns | 49.08 ns | +3.24% | - |
| `gmp_rational_api/hyperreal/sign` | 0.47 ns | 0.47 ns - 0.48 ns | 0.47 ns | +1.71% | - |
| `gmp_rational_api/hyperreal/signed_product_sum` | 109.30 ns | 108.56 ns - 110.11 ns | 108.19 ns | +13.16% | - |
| `gmp_rational_api/hyperreal/signed_product_sum_ordering` | 36.02 ns | 35.68 ns - 36.42 ns | 35.57 ns | +4.41% | - |
| `gmp_rational_api/hyperreal/signed_product_sum_shared_denominator` | 100.61 ns | 99.89 ns - 101.41 ns | 99.16 ns | +2.60% | - |
| `gmp_rational_api/hyperreal/sub` | 7.63 ns | 7.61 ns - 7.65 ns | 7.61 ns | +0.01% | - |
| `gmp_rational_api/hyperreal/to_f64` | 3.34 ns | 3.33 ns - 3.36 ns | 3.32 ns | +0.48% | - |
| `gmp_rational_api/hyperreal/to_f64_enclosure` | 20.08 ns | 20.00 ns - 20.17 ns | 19.93 ns | +1.18% | - |
| `gmp_rational_api/hyperreal/to_integer` | 4.66 ns | 4.64 ns - 4.68 ns | 4.63 ns | +2.84% | - |
| `gmp_rational_api/hyperreal/trunc` | 51.19 ns | 51.02 ns - 51.37 ns | 51.17 ns | +3.67% | - |
| `gmp_rational_api/hyperreal/zero` | 3.82 ns | 3.79 ns - 3.85 ns | 3.86 ns | -0.37% | - |
| `gmp_real_arithmetic_api/gmp_mpfr128/abs` | 26.88 ns | 26.79 ns - 26.99 ns | 26.73 ns | -0.65% | - |
| `gmp_real_arithmetic_api/gmp_mpfr128/add` | 44.67 ns | 44.37 ns - 45.05 ns | 44.05 ns | +2.67% | - |
| `gmp_real_arithmetic_api/gmp_mpfr128/ceil` | 45.57 ns | 45.30 ns - 45.86 ns | 45.07 ns | +3.82% | - |
| `gmp_real_arithmetic_api/gmp_mpfr128/div` | 90.51 ns | 90.17 ns - 90.88 ns | 90.02 ns | +34.08% | - |
| `gmp_real_arithmetic_api/gmp_mpfr128/floor` | 44.49 ns | 44.16 ns - 44.87 ns | 43.96 ns | +3.92% | - |
| `gmp_real_arithmetic_api/gmp_mpfr128/fract` | 36.62 ns | 36.48 ns - 36.77 ns | 36.38 ns | +3.00% | - |
| `gmp_real_arithmetic_api/gmp_mpfr128/inverse` | 61.37 ns | 61.06 ns - 61.71 ns | 60.86 ns | +2.17% | - |
| `gmp_real_arithmetic_api/gmp_mpfr128/mul` | 58.89 ns | 58.55 ns - 59.32 ns | 58.33 ns | +1.47% | - |
| `gmp_real_arithmetic_api/gmp_mpfr128/neg` | 17.43 ns | 17.30 ns - 17.57 ns | 17.20 ns | +3.04% | - |
| `gmp_real_arithmetic_api/gmp_mpfr128/pow` | 2.94 us | 2.90 us - 2.98 us | 2.87 us | +5.64% | - |
| `gmp_real_arithmetic_api/gmp_mpfr128/powi_17` | 143.35 ns | 142.28 ns - 144.55 ns | 141.80 ns | +3.94% | - |
| `gmp_real_arithmetic_api/gmp_mpfr128/rem_euclid` | 154.23 ns | 152.82 ns - 155.85 ns | 151.61 ns | +9.19% | - |
| `gmp_real_arithmetic_api/gmp_mpfr128/round` | 45.54 ns | 45.29 ns - 45.82 ns | 45.10 ns | +3.67% | - |
| `gmp_real_arithmetic_api/gmp_mpfr128/sub` | 47.00 ns | 46.74 ns - 47.28 ns | 46.35 ns | +5.97% | - |
| `gmp_real_arithmetic_api/gmp_mpfr128/to_degrees` | 88.74 ns | 88.30 ns - 89.22 ns | 87.88 ns | +1.78% | - |
| `gmp_real_arithmetic_api/gmp_mpfr128/to_radians` | 85.26 ns | 84.78 ns - 85.80 ns | 84.35 ns | +2.17% | - |
| `gmp_real_arithmetic_api/gmp_mpfr128/trunc` | 44.66 ns | 44.26 ns - 45.12 ns | 43.73 ns | +4.60% | - |
| `gmp_real_arithmetic_api/hyperreal/abs` | 31.45 ns | 31.24 ns - 31.68 ns | 30.97 ns | +2.33% | - |
| `gmp_real_arithmetic_api/hyperreal/add` | 34.30 ns | 34.14 ns - 34.48 ns | 34.03 ns | +3.36% | - |
| `gmp_real_arithmetic_api/hyperreal/ceil` | 167.87 ns | 166.15 ns - 169.87 ns | 164.90 ns | +3.70% | - |
| `gmp_real_arithmetic_api/hyperreal/div` | 62.36 ns | 61.99 ns - 62.79 ns | 61.64 ns | +9.79% | - |
| `gmp_real_arithmetic_api/hyperreal/floor` | 113.00 ns | 112.35 ns - 113.71 ns | 111.57 ns | +5.36% | - |
| `gmp_real_arithmetic_api/hyperreal/fract` | 159.79 ns | 158.11 ns - 161.89 ns | 157.04 ns | +3.66% | - |
| `gmp_real_arithmetic_api/hyperreal/inverse` | 25.97 ns | 25.85 ns - 26.10 ns | 25.74 ns | +2.76% | - |
| `gmp_real_arithmetic_api/hyperreal/mul` | 36.84 ns | 36.62 ns - 37.08 ns | 36.39 ns | +3.93% | - |
| `gmp_real_arithmetic_api/hyperreal/neg` | 21.80 ns | 21.71 ns - 21.91 ns | 21.63 ns | +2.12% | - |
| `gmp_real_arithmetic_api/hyperreal/pow` | 421.82 ns | 420.06 ns - 423.81 ns | 419.05 ns | -4.96% | - |
| `gmp_real_arithmetic_api/hyperreal/powi_17` | 81.65 ns | 81.04 ns - 82.33 ns | 80.20 ns | +3.99% | - |
| `gmp_real_arithmetic_api/hyperreal/rem_euclid` | 320.21 ns | 318.44 ns - 322.14 ns | 317.79 ns | +2.35% | - |
| `gmp_real_arithmetic_api/hyperreal/round` | 225.08 ns | 222.69 ns - 227.85 ns | 220.39 ns | +5.07% | - |
| `gmp_real_arithmetic_api/hyperreal/sub` | 34.35 ns | 34.02 ns - 34.74 ns | 33.72 ns | +2.72% | - |
| `gmp_real_arithmetic_api/hyperreal/to_degrees` | 251.56 ns | 249.50 ns - 253.77 ns | 247.36 ns | +2.08% | - |
| `gmp_real_arithmetic_api/hyperreal/to_radians` | 257.71 ns | 254.65 ns - 261.11 ns | 251.52 ns | +1.57% | - |
| `gmp_real_arithmetic_api/hyperreal/trunc` | 110.69 ns | 109.92 ns - 111.51 ns | 109.58 ns | +5.00% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/affine` | 60.62 ns | 60.31 ns - 60.95 ns | 60.09 ns | +0.58% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/e` | 1.05 us | 1.04 us - 1.06 us | 1.03 us | +2.15% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/exact_rational_interpolate_point3_known_dyadic` | 220.83 ns | 220.01 ns - 221.74 ns | 219.52 ns | +2.66% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/exact_rational_line_intersection2_known_dyadic` | 602.98 ns | 599.17 ns - 607.17 ns | 594.53 ns | +4.63% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/exact_rational_line_intersection2_point_known_exact` | 471.86 ns | 468.74 ns - 475.25 ns | 465.18 ns | +2.40% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/exact_rational_parameterized_point2_known_dyadic` | 147.07 ns | 146.05 ns - 148.21 ns | 145.08 ns | +0.89% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/exact_rational_quotient_known_dyadic` | 63.08 ns | 62.81 ns - 63.37 ns | 62.80 ns | +5.03% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/integer_bigint` | 58.82 ns | 58.53 ns - 59.12 ns | 58.48 ns | +1.57% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/inverse_ref` | 60.45 ns | 60.12 ns - 60.81 ns | 59.73 ns | +2.56% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/is_finite` | 0.49 ns | 0.49 ns - 0.50 ns | 0.48 ns | +4.72% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/is_integer` | 1.68 ns | 1.67 ns - 1.69 ns | 1.66 ns | -2.88% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/max` | 26.13 ns | 25.98 ns - 26.30 ns | 25.83 ns | +2.29% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/mean` | 69.57 ns | 69.15 ns - 70.06 ns | 68.58 ns | +1.11% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/min` | 26.30 ns | 26.18 ns - 26.43 ns | 26.11 ns | +2.67% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/new_rational` | 26.77 ns | 26.70 ns - 26.84 ns | 26.68 ns | +1.02% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/one` | 19.99 ns | 19.84 ns - 20.14 ns | 19.77 ns | +5.80% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/pi` | 17.92 ns | 17.84 ns - 18.00 ns | 17.82 ns | +4.39% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/sample_stddev` | 481.62 ns | 479.62 ns - 483.80 ns | 477.83 ns | +2.43% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/sum_owned` | 61.73 ns | 61.47 ns - 62.00 ns | 61.13 ns | +1.18% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/sum_refs` | 61.97 ns | 61.70 ns - 62.27 ns | 61.43 ns | +0.49% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/tau` | 26.00 ns | 25.89 ns - 26.10 ns | 25.85 ns | -0.81% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/to_f32_lossy` | 8.26 ns | 8.21 ns - 8.31 ns | 8.17 ns | +1.91% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/to_f64_exact_dyadic` | 8.00 ns | 7.96 ns - 8.04 ns | 7.92 ns | +0.70% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/to_f64_lossy` | 8.07 ns | 8.01 ns - 8.13 ns | 7.94 ns | +2.79% | - |
| `gmp_real_collection_and_conversion_api/gmp_mpfr128/zero` | 17.96 ns | 17.82 ns - 18.13 ns | 17.64 ns | +1.89% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/affine` | 41.46 ns | 41.19 ns - 41.74 ns | 41.02 ns | +3.06% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/e` | 20.40 ns | 20.32 ns - 20.47 ns | 20.27 ns | +1.94% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/exact_rational_interpolate_point3_known_dyadic` | 291.59 ns | 289.05 ns - 294.38 ns | 285.91 ns | +2.12% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/exact_rational_line_intersection2_known_dyadic` | 480.25 ns | 477.35 ns - 483.41 ns | 474.74 ns | -1.52% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/exact_rational_line_intersection2_point_known_exact` | 919.86 ns | 911.75 ns - 928.90 ns | 903.03 ns | +3.16% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/exact_rational_parameterized_point2_known_dyadic` | 166.78 ns | 166.01 ns - 167.63 ns | 165.66 ns | +3.21% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/exact_rational_quotient_known_dyadic` | 27.36 ns | 27.20 ns - 27.53 ns | 27.07 ns | +0.19% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/integer_bigint` | 76.13 ns | 75.25 ns - 77.09 ns | 74.07 ns | -8.98% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/inverse_ref` | 15.99 ns | 15.91 ns - 16.07 ns | 15.84 ns | +1.13% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/is_finite` | 0.50 ns | 0.49 ns - 0.50 ns | 0.49 ns | +5.49% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/is_integer` | 5.86 ns | 5.83 ns - 5.90 ns | 5.81 ns | -1.83% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/max` | 15.92 ns | 15.80 ns - 16.06 ns | 15.68 ns | +2.96% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/mean` | 118.87 ns | 118.07 ns - 119.74 ns | 117.44 ns | -0.21% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/min` | 16.10 ns | 15.96 ns - 16.25 ns | 15.77 ns | +3.30% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/new_rational` | 54.81 ns | 54.54 ns - 55.10 ns | 54.21 ns | +1.88% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/one` | 9.90 ns | 9.84 ns - 9.97 ns | 9.78 ns | +4.69% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/pi` | 16.54 ns | 16.43 ns - 16.67 ns | 16.34 ns | -0.19% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/sample_stddev` | 789.49 ns | 784.93 ns - 794.62 ns | 788.12 ns | -0.68% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/sum_owned` | 147.07 ns | 146.51 ns - 147.70 ns | 146.27 ns | -1.24% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/sum_refs` | 114.98 ns | 114.37 ns - 115.63 ns | 113.92 ns | +30.64% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/tau` | 16.31 ns | 16.23 ns - 16.39 ns | 16.16 ns | +2.22% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/to_f32_lossy` | 5.13 ns | 5.10 ns - 5.16 ns | 5.08 ns | +2.21% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/to_f64_exact_dyadic` | 2.40 ns | 2.39 ns - 2.41 ns | 2.38 ns | -5.50% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/to_f64_lossy` | 0.86 ns | 0.86 ns - 0.87 ns | 0.85 ns | +4.48% | - |
| `gmp_real_collection_and_conversion_api/hyperreal/zero` | 9.60 ns | 9.57 ns - 9.63 ns | 9.54 ns | +0.75% | - |
| `gmp_real_derived_api/gmp_mpfr128/chi_square_cdf_k5` | 56.22 us | 55.75 us - 56.73 us | 55.23 us | +3.28% | - |
| `gmp_real_derived_api/gmp_mpfr128/chi_square_sf_k5` | 56.15 us | 55.74 us - 56.60 us | 55.41 us | +1.61% | - |
| `gmp_real_derived_api/gmp_mpfr128/dnorm` | 1.29 us | 1.28 us - 1.30 us | 1.28 us | +4.06% | - |
| `gmp_real_derived_api/gmp_mpfr128/dnorm_derivative_n6` | 1.63 us | 1.62 us - 1.64 us | 1.61 us | +0.99% | - |
| `gmp_real_derived_api/gmp_mpfr128/erfcx` | 33.48 us | 33.26 us - 33.71 us | 33.21 us | +3.97% | - |
| `gmp_real_derived_api/gmp_mpfr128/gaussian_derivative_n6` | 1.65 us | 1.64 us - 1.66 us | 1.62 us | +2.60% | - |
| `gmp_real_derived_api/gmp_mpfr128/hermite_probabilists_n6` | 337.59 ns | 335.91 ns - 339.54 ns | 334.44 ns | +3.86% | - |
| `gmp_real_derived_api/gmp_mpfr128/log_dnorm` | 2.53 us | 2.51 us - 2.54 us | 2.50 us | +3.17% | - |
| `gmp_real_derived_api/gmp_mpfr128/log_normal_sf` | 34.79 us | 34.34 us - 35.32 us | 34.25 us | +6.12% | - |
| `gmp_real_derived_api/gmp_mpfr128/log_pnorm` | 34.02 us | 33.80 us - 34.25 us | 33.84 us | +5.17% | - |
| `gmp_real_derived_api/gmp_mpfr128/logit` | 1.42 us | 1.41 us - 1.42 us | 1.41 us | +1.79% | - |
| `gmp_real_derived_api/gmp_mpfr128/normal_cdf` | 3.34 us | 3.32 us - 3.37 us | 3.29 us | +5.43% | - |
| `gmp_real_derived_api/gmp_mpfr128/normal_hazard` | 33.71 us | 33.45 us - 34.00 us | 33.25 us | +4.74% | - |
| `gmp_real_derived_api/gmp_mpfr128/normal_interval` | 5.57 us | 5.51 us - 5.65 us | 5.46 us | +3.48% | - |
| `gmp_real_derived_api/gmp_mpfr128/normal_interval_moment_n4` | 10.12 us | 10.06 us - 10.19 us | 10.00 us | +1.60% | - |
| `gmp_real_derived_api/gmp_mpfr128/normal_inverse_mills` | 418.92 ns | 415.79 ns - 422.35 ns | 412.58 ns | +10.48% | - |
| `gmp_real_derived_api/gmp_mpfr128/normal_log_hazard` | 36.55 us | 36.29 us - 36.83 us | 36.03 us | +2.36% | - |
| `gmp_real_derived_api/gmp_mpfr128/normal_mills` | 33.02 us | 32.87 us - 33.18 us | 32.83 us | +2.60% | - |
| `gmp_real_derived_api/gmp_mpfr128/normal_pdf` | 1.35 us | 1.34 us - 1.36 us | 1.34 us | +3.64% | - |
| `gmp_real_derived_api/gmp_mpfr128/normal_quantile` | 33.02 us | 32.73 us - 33.34 us | 32.61 us | +3.65% | - |
| `gmp_real_derived_api/gmp_mpfr128/normal_sf` | 33.10 us | 32.85 us - 33.38 us | 32.88 us | +7.51% | - |
| `gmp_real_derived_api/gmp_mpfr128/normal_survival` | 3.26 us | 3.24 us - 3.27 us | 3.23 us | +1.54% | - |
| `gmp_real_derived_api/gmp_mpfr128/pnorm` | 3.95 us | 3.93 us - 3.97 us | 3.92 us | +3.20% | - |
| `gmp_real_derived_api/gmp_mpfr128/pnorm_diff` | 5.59 us | 5.55 us - 5.64 us | 5.52 us | +3.88% | - |
| `gmp_real_derived_api/gmp_mpfr128/pnorm_upper` | 4.00 us | 3.97 us - 4.04 us | 3.96 us | +4.98% | - |
| `gmp_real_derived_api/gmp_mpfr128/qnorm` | 32.46 us | 32.25 us - 32.69 us | 32.07 us | +2.85% | - |
| `gmp_real_derived_api/gmp_mpfr128/qnorm_upper` | 32.36 us | 32.14 us - 32.62 us | 31.97 us | +1.91% | - |
| `gmp_real_derived_api/gmp_mpfr128/regularized_beta_integer` | 783.87 ns | 780.15 ns - 787.99 ns | 778.62 ns | +3.21% | - |
| `gmp_real_derived_api/gmp_mpfr128/regularized_beta_q_integer` | 844.92 ns | 839.64 ns - 850.86 ns | 836.47 ns | -0.86% | - |
| `gmp_real_derived_api/gmp_mpfr128/regularized_gamma_p` | 56.06 us | 55.65 us - 56.50 us | 55.13 us | +2.77% | - |
| `gmp_real_derived_api/gmp_mpfr128/regularized_gamma_q` | 57.19 us | 56.28 us - 58.41 us | 55.62 us | +5.49% | - |
| `gmp_real_derived_api/gmp_mpfr128/sigmoid` | 1.05 us | 1.05 us - 1.06 us | 1.05 us | +3.77% | - |
| `gmp_real_derived_api/gmp_mpfr128/softplus` | 2.50 us | 2.49 us - 2.51 us | 2.48 us | +3.24% | - |
| `gmp_real_derived_api/gmp_mpfr128/sqrt1m1` | 165.21 ns | 164.11 ns - 166.55 ns | 163.00 ns | -11.22% | - |
| `gmp_real_derived_api/gmp_mpfr128/sqrt1pm1` | 159.63 ns | 159.02 ns - 160.31 ns | 158.39 ns | +1.61% | - |
| `gmp_real_derived_api/gmp_mpfr128/standard_normal_moment_n8` | 19.83 ns | 19.69 ns - 19.97 ns | 19.71 ns | +2.38% | - |
| `gmp_real_derived_api/gmp_mpfr128/truncated_normal_mean` | 9.51 us | 9.43 us - 9.60 us | 9.34 us | +0.83% | - |
| `gmp_real_derived_api/gmp_mpfr128/truncated_normal_variance` | 9.82 us | 9.76 us - 9.90 us | 9.70 us | +1.87% | - |
| `gmp_real_derived_api/hyperreal/chi_square_cdf_k5` | 4.31 us | 4.26 us - 4.37 us | 4.21 us | +2.30% | - |
| `gmp_real_derived_api/hyperreal/chi_square_sf_k5` | 3.06 us | 3.04 us - 3.09 us | 3.02 us | -0.02% | - |
| `gmp_real_derived_api/hyperreal/dnorm` | 1.05 us | 1.04 us - 1.05 us | 1.03 us | -3.44% | - |
| `gmp_real_derived_api/hyperreal/dnorm_derivative_n6` | 2.41 us | 2.39 us - 2.43 us | 2.36 us | +2.68% | - |
| `gmp_real_derived_api/hyperreal/erfcx` | 1.25 us | 1.25 us - 1.26 us | 1.24 us | -1.59% | - |
| `gmp_real_derived_api/hyperreal/gaussian_derivative_n6` | 2.29 us | 2.28 us - 2.31 us | 2.28 us | -3.08% | - |
| `gmp_real_derived_api/hyperreal/hermite_probabilists_n6` | 1.07 us | 1.06 us - 1.08 us | 1.05 us | -0.18% | - |
| `gmp_real_derived_api/hyperreal/log_dnorm` | 146.55 ns | 145.43 ns - 147.88 ns | 144.70 ns | -0.56% | - |
| `gmp_real_derived_api/hyperreal/log_normal_sf` | 291.56 ns | 289.75 ns - 293.53 ns | 288.97 ns | +0.63% | - |
| `gmp_real_derived_api/hyperreal/log_pnorm` | 281.08 ns | 279.39 ns - 282.84 ns | 277.95 ns | +1.60% | - |
| `gmp_real_derived_api/hyperreal/logit` | 356.77 ns | 354.51 ns - 359.21 ns | 352.78 ns | -1.50% | - |
| `gmp_real_derived_api/hyperreal/normal_cdf` | 3.58 us | 3.55 us - 3.60 us | 3.54 us | +3.77% | - |
| `gmp_real_derived_api/hyperreal/normal_hazard` | 3.16 us | 3.14 us - 3.19 us | 3.11 us | +3.35% | - |
| `gmp_real_derived_api/hyperreal/normal_interval` | 681.31 ns | 674.07 ns - 688.99 ns | 672.55 ns | +2.23% | - |
| `gmp_real_derived_api/hyperreal/normal_interval_moment_n4` | 5.97 us | 5.91 us - 6.04 us | 5.87 us | +1.60% | - |
| `gmp_real_derived_api/hyperreal/normal_inverse_mills` | 26.31 ns | 26.17 ns - 26.48 ns | 26.05 ns | +2.55% | - |
| `gmp_real_derived_api/hyperreal/normal_log_hazard` | 531.32 ns | 527.12 ns - 536.00 ns | 522.84 ns | -0.74% | - |
| `gmp_real_derived_api/hyperreal/normal_mills` | 3.00 us | 2.99 us - 3.02 us | 2.98 us | +2.61% | - |
| `gmp_real_derived_api/hyperreal/normal_pdf` | 1.40 us | 1.40 us - 1.41 us | 1.40 us | +0.03% | - |
| `gmp_real_derived_api/hyperreal/normal_quantile` | 1.07 us | 1.06 us - 1.09 us | 1.05 us | -0.30% | - |
| `gmp_real_derived_api/hyperreal/normal_sf` | 335.35 ns | 332.31 ns - 338.73 ns | 328.79 ns | +0.00% | - |
| `gmp_real_derived_api/hyperreal/normal_survival` | 616.64 ns | 613.60 ns - 619.93 ns | 609.93 ns | -1.63% | - |
| `gmp_real_derived_api/hyperreal/pnorm` | 4.85 us | 4.81 us - 4.88 us | 4.78 us | +4.62% | - |
| `gmp_real_derived_api/hyperreal/pnorm_diff` | 647.81 ns | 644.11 ns - 652.52 ns | 641.46 ns | -4.63% | - |
| `gmp_real_derived_api/hyperreal/pnorm_upper` | 359.22 ns | 356.84 ns - 361.87 ns | 354.57 ns | -0.51% | - |
| `gmp_real_derived_api/hyperreal/qnorm` | 734.96 ns | 728.50 ns - 741.86 ns | 725.09 ns | +3.62% | - |
| `gmp_real_derived_api/hyperreal/qnorm_upper` | 863.59 ns | 859.27 ns - 868.33 ns | 857.16 ns | -0.16% | - |
| `gmp_real_derived_api/hyperreal/regularized_beta_integer` | 3.48 us | 3.45 us - 3.51 us | 3.41 us | +0.55% | - |
| `gmp_real_derived_api/hyperreal/regularized_beta_q_integer` | 2.69 us | 2.65 us - 2.72 us | 2.64 us | +2.81% | - |
| `gmp_real_derived_api/hyperreal/regularized_gamma_p` | 4.08 us | 4.03 us - 4.14 us | 3.97 us | +4.15% | - |
| `gmp_real_derived_api/hyperreal/regularized_gamma_q` | 2.89 us | 2.86 us - 2.93 us | 2.81 us | +2.43% | - |
| `gmp_real_derived_api/hyperreal/sigmoid` | 228.61 ns | 227.67 ns - 229.64 ns | 227.10 ns | +0.55% | - |
| `gmp_real_derived_api/hyperreal/softplus` | 2.08 us | 2.07 us - 2.09 us | 2.06 us | -2.84% | - |
| `gmp_real_derived_api/hyperreal/sqrt1m1` | 954.19 ns | 946.28 ns - 962.84 ns | 933.83 ns | +5.80% | - |
| `gmp_real_derived_api/hyperreal/sqrt1pm1` | 955.42 ns | 945.30 ns - 966.71 ns | 931.75 ns | +4.65% | - |
| `gmp_real_derived_api/hyperreal/standard_normal_moment_n8` | 114.78 ns | 114.22 ns - 115.43 ns | 113.88 ns | +1.98% | - |
| `gmp_real_derived_api/hyperreal/truncated_normal_mean` | 3.15 us | 3.13 us - 3.18 us | 3.11 us | -1.03% | - |
| `gmp_real_derived_api/hyperreal/truncated_normal_variance` | 3.92 us | 3.89 us - 3.96 us | 3.86 us | -2.01% | - |
| `gmp_real_elementary_api/gmp_mpfr128/acos` | 3.27 us | 3.25 us - 3.29 us | 3.24 us | +3.23% | - |
| `gmp_real_elementary_api/gmp_mpfr128/acosh` | 1.64 us | 1.63 us - 1.65 us | 1.62 us | -0.60% | - |
| `gmp_real_elementary_api/gmp_mpfr128/asin` | 3.24 us | 3.22 us - 3.25 us | 3.22 us | +1.62% | - |
| `gmp_real_elementary_api/gmp_mpfr128/asinh` | 1.72 us | 1.70 us - 1.73 us | 1.68 us | +2.63% | - |
| `gmp_real_elementary_api/gmp_mpfr128/atan` | 2.94 us | 2.92 us - 2.97 us | 2.90 us | +4.85% | - |
| `gmp_real_elementary_api/gmp_mpfr128/atan2` | 2.86 us | 2.84 us - 2.89 us | 2.82 us | +4.40% | - |
| `gmp_real_elementary_api/gmp_mpfr128/atanh` | 1.69 us | 1.68 us - 1.71 us | 1.67 us | +3.56% | - |
| `gmp_real_elementary_api/gmp_mpfr128/beta` | 17.98 us | 17.89 us - 18.08 us | 17.85 us | +3.12% | - |
| `gmp_real_elementary_api/gmp_mpfr128/cbrt` | 388.84 ns | 387.10 ns - 390.88 ns | 386.36 ns | +2.49% | - |
| `gmp_real_elementary_api/gmp_mpfr128/cos` | 533.56 ns | 530.35 ns - 536.96 ns | 527.70 ns | +6.15% | - |
| `gmp_real_elementary_api/gmp_mpfr128/cos_pi` | 921.75 ns | 908.77 ns - 937.39 ns | 902.95 ns | +6.89% | - |
| `gmp_real_elementary_api/gmp_mpfr128/cosc` | 638.37 ns | 629.45 ns - 648.40 ns | 625.06 ns | +5.76% | - |
| `gmp_real_elementary_api/gmp_mpfr128/cosh` | 1.10 us | 1.09 us - 1.10 us | 1.08 us | +2.33% | - |
| `gmp_real_elementary_api/gmp_mpfr128/cot` | 1.09 us | 1.08 us - 1.10 us | 1.08 us | +1.92% | - |
| `gmp_real_elementary_api/gmp_mpfr128/cot_pi` | 1.78 us | 1.77 us - 1.80 us | 1.76 us | +4.30% | - |
| `gmp_real_elementary_api/gmp_mpfr128/erf` | 3.37 us | 3.36 us - 3.39 us | 3.35 us | +3.57% | - |
| `gmp_real_elementary_api/gmp_mpfr128/erfc` | 3.61 us | 3.59 us - 3.63 us | 3.56 us | +2.64% | - |
| `gmp_real_elementary_api/gmp_mpfr128/erfcinv` | 32.46 us | 32.19 us - 32.75 us | 31.88 us | +3.82% | - |
| `gmp_real_elementary_api/gmp_mpfr128/erfinv` | 32.89 us | 32.57 us - 33.25 us | 32.20 us | +6.01% | - |
| `gmp_real_elementary_api/gmp_mpfr128/exp` | 936.10 ns | 927.25 ns - 947.50 ns | 919.29 ns | +3.03% | - |
| `gmp_real_elementary_api/gmp_mpfr128/exp10` | 2.89 us | 2.86 us - 2.91 us | 2.85 us | +3.08% | - |
| `gmp_real_elementary_api/gmp_mpfr128/exp2` | 1.20 us | 1.20 us - 1.21 us | 1.19 us | +2.65% | - |
| `gmp_real_elementary_api/gmp_mpfr128/expm1` | 737.63 ns | 733.05 ns - 742.53 ns | 729.04 ns | +1.58% | - |
| `gmp_real_elementary_api/gmp_mpfr128/gamma` | 8.27 us | 8.19 us - 8.34 us | 8.13 us | +5.16% | - |
| `gmp_real_elementary_api/gmp_mpfr128/hypot2` | 196.68 ns | 195.49 ns - 197.97 ns | 196.03 ns | +3.13% | - |
| `gmp_real_elementary_api/gmp_mpfr128/hypot3` | 426.79 ns | 422.98 ns - 431.14 ns | 420.58 ns | +9.13% | - |
| `gmp_real_elementary_api/gmp_mpfr128/hypot_minus` | 243.25 ns | 240.81 ns - 246.07 ns | 238.62 ns | +5.53% | - |
| `gmp_real_elementary_api/gmp_mpfr128/lbeta` | 24.74 us | 24.56 us - 24.94 us | 24.25 us | +2.94% | - |
| `gmp_real_elementary_api/gmp_mpfr128/lgamma` | 8.17 us | 8.09 us - 8.27 us | 8.03 us | +3.93% | - |
| `gmp_real_elementary_api/gmp_mpfr128/ln` | 1.33 us | 1.32 us - 1.34 us | 1.32 us | +1.59% | - |
| `gmp_real_elementary_api/gmp_mpfr128/ln_1m` | 3.56 us | 3.54 us - 3.58 us | 3.52 us | +1.36% | - |
| `gmp_real_elementary_api/gmp_mpfr128/ln_1p` | 349.18 ns | 346.64 ns - 351.85 ns | 346.09 ns | +4.49% | - |
| `gmp_real_elementary_api/gmp_mpfr128/ln_beta` | 24.72 us | 24.54 us - 24.92 us | 24.50 us | +2.02% | - |
| `gmp_real_elementary_api/gmp_mpfr128/log10` | 3.05 us | 3.01 us - 3.10 us | 2.95 us | +4.92% | - |
| `gmp_real_elementary_api/gmp_mpfr128/log1m` | 3.66 us | 3.62 us - 3.70 us | 3.57 us | +4.40% | - |
| `gmp_real_elementary_api/gmp_mpfr128/log1p` | 341.18 ns | 339.13 ns - 343.54 ns | 336.68 ns | -2.73% | - |
| `gmp_real_elementary_api/gmp_mpfr128/log2` | 1.53 us | 1.52 us - 1.54 us | 1.51 us | +1.16% | - |
| `gmp_real_elementary_api/gmp_mpfr128/logaddexp` | 3.54 us | 3.51 us - 3.57 us | 3.50 us | +3.90% | - |
| `gmp_real_elementary_api/gmp_mpfr128/logsubexp` | 3.55 us | 3.52 us - 3.59 us | 3.51 us | +4.57% | - |
| `gmp_real_elementary_api/gmp_mpfr128/pow_rational_5_over_3` | 2.91 us | 2.88 us - 2.95 us | 2.86 us | +4.77% | - |
| `gmp_real_elementary_api/gmp_mpfr128/root_n_5` | 487.17 ns | 483.05 ns - 491.50 ns | 477.50 ns | +5.18% | - |
| `gmp_real_elementary_api/gmp_mpfr128/sin` | 733.75 ns | 727.59 ns - 740.73 ns | 719.66 ns | +3.23% | - |
| `gmp_real_elementary_api/gmp_mpfr128/sin_pi` | 1.53 us | 1.52 us - 1.54 us | 1.51 us | +2.82% | - |
| `gmp_real_elementary_api/gmp_mpfr128/sinc` | 789.43 ns | 784.12 ns - 795.16 ns | 779.60 ns | -0.36% | - |
| `gmp_real_elementary_api/gmp_mpfr128/sinc_pi` | 1.65 us | 1.63 us - 1.67 us | 1.62 us | +7.15% | - |
| `gmp_real_elementary_api/gmp_mpfr128/sinh` | 1.14 us | 1.12 us - 1.15 us | 1.11 us | +3.73% | - |
| `gmp_real_elementary_api/gmp_mpfr128/sqrt` | 107.31 ns | 106.82 ns - 107.83 ns | 106.43 ns | +2.36% | - |
| `gmp_real_elementary_api/gmp_mpfr128/tan` | 983.86 ns | 977.64 ns - 990.60 ns | 972.82 ns | +2.66% | - |
| `gmp_real_elementary_api/gmp_mpfr128/tan_pi` | 1.84 us | 1.84 us - 1.85 us | 1.83 us | +3.51% | - |
| `gmp_real_elementary_api/gmp_mpfr128/tanh` | 1.18 us | 1.17 us - 1.19 us | 1.17 us | +2.10% | - |
| `gmp_real_elementary_api/hyperreal/acos` | 176.68 ns | 175.80 ns - 177.76 ns | 175.29 ns | +0.62% | - |
| `gmp_real_elementary_api/hyperreal/acosh` | 192.28 ns | 191.06 ns - 193.66 ns | 189.92 ns | -1.67% | - |
| `gmp_real_elementary_api/hyperreal/asin` | 183.77 ns | 182.93 ns - 184.74 ns | 182.23 ns | -2.40% | - |
| `gmp_real_elementary_api/hyperreal/asinh` | 174.41 ns | 173.49 ns - 175.38 ns | 173.05 ns | +6.40% | - |
| `gmp_real_elementary_api/hyperreal/atan` | 245.93 ns | 245.03 ns - 246.95 ns | 244.33 ns | -1.75% | - |
| `gmp_real_elementary_api/hyperreal/atan2` | 762.80 ns | 755.86 ns - 770.30 ns | 746.36 ns | +4.12% | - |
| `gmp_real_elementary_api/hyperreal/atanh` | 391.04 ns | 388.83 ns - 393.60 ns | 387.35 ns | +0.64% | - |
| `gmp_real_elementary_api/hyperreal/beta` | 1.04 us | 1.04 us - 1.05 us | 1.04 us | +2.38% | - |
| `gmp_real_elementary_api/hyperreal/cbrt` | 264.69 ns | 262.72 ns - 266.81 ns | 259.82 ns | +0.61% | - |
| `gmp_real_elementary_api/hyperreal/cos` | 213.12 ns | 212.35 ns - 213.97 ns | 211.81 ns | +1.43% | - |
| `gmp_real_elementary_api/hyperreal/cos_pi` | 236.36 ns | 233.93 ns - 239.24 ns | 231.79 ns | -0.60% | - |
| `gmp_real_elementary_api/hyperreal/cosc` | 517.45 ns | 512.09 ns - 523.57 ns | 504.93 ns | +3.31% | - |
| `gmp_real_elementary_api/hyperreal/cosh` | 353.69 ns | 351.55 ns - 356.09 ns | 350.29 ns | -0.38% | - |
| `gmp_real_elementary_api/hyperreal/cot` | 712.51 ns | 707.12 ns - 718.23 ns | 699.24 ns | +3.69% | - |
| `gmp_real_elementary_api/hyperreal/cot_pi` | 118.50 ns | 117.65 ns - 119.46 ns | 117.06 ns | +2.23% | - |
| `gmp_real_elementary_api/hyperreal/erf` | 919.56 ns | 914.43 ns - 925.21 ns | 910.48 ns | -0.97% | - |
| `gmp_real_elementary_api/hyperreal/erfc` | 132.28 ns | 131.40 ns - 133.23 ns | 130.45 ns | +0.89% | - |
| `gmp_real_elementary_api/hyperreal/erfcinv` | 1.33 us | 1.32 us - 1.35 us | 1.32 us | +4.23% | - |
| `gmp_real_elementary_api/hyperreal/erfinv` | 1.43 us | 1.42 us - 1.45 us | 1.41 us | +0.69% | - |
| `gmp_real_elementary_api/hyperreal/exp` | 101.60 ns | 100.35 ns - 103.10 ns | 99.74 ns | +2.69% | - |
| `gmp_real_elementary_api/hyperreal/exp10` | 161.34 ns | 160.24 ns - 162.60 ns | 159.48 ns | +0.94% | - |
| `gmp_real_elementary_api/hyperreal/exp2` | 163.46 ns | 161.78 ns - 165.60 ns | 160.60 ns | +3.30% | - |
| `gmp_real_elementary_api/hyperreal/expm1` | 159.02 ns | 157.95 ns - 160.15 ns | 157.16 ns | +3.70% | - |
| `gmp_real_elementary_api/hyperreal/gamma` | 313.98 ns | 312.06 ns - 316.09 ns | 311.23 ns | +2.21% | - |
| `gmp_real_elementary_api/hyperreal/hypot2` | 96.07 ns | 95.58 ns - 96.62 ns | 95.05 ns | +2.25% | - |
| `gmp_real_elementary_api/hyperreal/hypot3` | 141.71 ns | 140.43 ns - 143.22 ns | 139.06 ns | +1.27% | - |
| `gmp_real_elementary_api/hyperreal/hypot_minus` | 524.58 ns | 521.15 ns - 528.60 ns | 518.19 ns | +3.56% | - |
| `gmp_real_elementary_api/hyperreal/lbeta` | 2.88 us | 2.85 us - 2.91 us | 2.81 us | -3.93% | - |
| `gmp_real_elementary_api/hyperreal/lgamma` | 3.01 us | 2.98 us - 3.04 us | 2.96 us | +0.93% | - |
| `gmp_real_elementary_api/hyperreal/ln` | 882.94 ns | 877.23 ns - 888.97 ns | 877.25 ns | -2.01% | - |
| `gmp_real_elementary_api/hyperreal/ln_1m` | 62.39 ns | 62.09 ns - 62.73 ns | 61.89 ns | +3.26% | - |
| `gmp_real_elementary_api/hyperreal/ln_1p` | 58.68 ns | 58.45 ns - 58.93 ns | 58.24 ns | +0.65% | - |
| `gmp_real_elementary_api/hyperreal/ln_beta` | 2.84 us | 2.82 us - 2.86 us | 2.81 us | -2.16% | - |
| `gmp_real_elementary_api/hyperreal/log10` | 880.63 ns | 876.08 ns - 885.84 ns | 873.02 ns | -8.62% | - |
| `gmp_real_elementary_api/hyperreal/log1m` | 63.13 ns | 62.88 ns - 63.39 ns | 62.70 ns | +3.23% | - |
| `gmp_real_elementary_api/hyperreal/log1p` | 58.68 ns | 58.45 ns - 58.94 ns | 58.26 ns | +0.51% | - |
| `gmp_real_elementary_api/hyperreal/log2` | 893.41 ns | 883.33 ns - 905.30 ns | 873.50 ns | -6.92% | - |
| `gmp_real_elementary_api/hyperreal/logaddexp` | 258.81 ns | 256.78 ns - 261.13 ns | 255.41 ns | +4.16% | - |
| `gmp_real_elementary_api/hyperreal/logsubexp` | 262.89 ns | 261.27 ns - 264.64 ns | 260.31 ns | +3.07% | - |
| `gmp_real_elementary_api/hyperreal/pow_rational_5_over_3` | 513.55 ns | 509.92 ns - 517.42 ns | 507.42 ns | -0.17% | - |
| `gmp_real_elementary_api/hyperreal/root_n_5` | 156.06 ns | 154.66 ns - 157.61 ns | 153.35 ns | -0.03% | - |
| `gmp_real_elementary_api/hyperreal/sin` | 214.25 ns | 213.11 ns - 215.50 ns | 211.56 ns | +2.02% | - |
| `gmp_real_elementary_api/hyperreal/sin_pi` | 90.81 ns | 89.79 ns - 91.94 ns | 89.05 ns | +5.96% | - |
| `gmp_real_elementary_api/hyperreal/sinc` | 287.42 ns | 285.48 ns - 289.53 ns | 284.20 ns | +4.19% | - |
| `gmp_real_elementary_api/hyperreal/sinc_pi` | 384.33 ns | 382.08 ns - 386.84 ns | 381.21 ns | +0.86% | - |
| `gmp_real_elementary_api/hyperreal/sinh` | 393.45 ns | 390.11 ns - 397.15 ns | 387.37 ns | -0.51% | - |
| `gmp_real_elementary_api/hyperreal/sqrt` | 102.56 ns | 102.15 ns - 103.04 ns | 101.78 ns | +3.40% | - |
| `gmp_real_elementary_api/hyperreal/tan` | 57.72 ns | 57.36 ns - 58.11 ns | 57.13 ns | +2.16% | - |
| `gmp_real_elementary_api/hyperreal/tan_pi` | 173.48 ns | 172.60 ns - 174.41 ns | 172.39 ns | +5.45% | - |
| `gmp_real_elementary_api/hyperreal/tanh` | 575.71 ns | 571.78 ns - 580.06 ns | 567.72 ns | -1.49% | - |
| `gmp_real_linear_algebra_api/gmp_mpfr128/active_dot2_refs` | 155.64 ns | 154.36 ns - 157.11 ns | 152.96 ns | -0.33% | - |
| `gmp_real_linear_algebra_api/gmp_mpfr128/active_dot3_refs` | 265.24 ns | 263.00 ns - 267.68 ns | 260.30 ns | -2.40% | - |
| `gmp_real_linear_algebra_api/gmp_mpfr128/active_dot4_refs` | 199.65 ns | 198.82 ns - 200.56 ns | 198.05 ns | -1.10% | - |
| `gmp_real_linear_algebra_api/gmp_mpfr128/active_linear_combination3_refs` | 268.49 ns | 267.34 ns - 269.75 ns | 266.62 ns | -0.34% | - |
| `gmp_real_linear_algebra_api/gmp_mpfr128/active_linear_combination4_refs` | 200.00 ns | 198.80 ns - 201.40 ns | 197.84 ns | -0.54% | - |
| `gmp_real_linear_algebra_api/gmp_mpfr128/active_signed_product_sum` | 82.04 ns | 81.55 ns - 82.57 ns | 80.95 ns | +0.79% | - |
| `gmp_real_linear_algebra_api/gmp_mpfr128/affine_combination3_refs` | 273.88 ns | 272.09 ns - 275.79 ns | 269.43 ns | -9.10% | - |
| `gmp_real_linear_algebra_api/gmp_mpfr128/affine_combination4_refs` | 220.92 ns | 219.97 ns - 221.91 ns | 219.26 ns | +1.40% | - |
| `gmp_real_linear_algebra_api/gmp_mpfr128/diff_of_products` | 83.25 ns | 82.65 ns - 83.89 ns | 82.21 ns | +0.39% | - |
| `gmp_real_linear_algebra_api/gmp_mpfr128/dot2_refs` | 159.25 ns | 157.92 ns - 160.71 ns | 157.06 ns | +0.34% | - |
| `gmp_real_linear_algebra_api/gmp_mpfr128/dot3_refs` | 268.39 ns | 266.79 ns - 270.14 ns | 265.37 ns | -0.15% | - |
| `gmp_real_linear_algebra_api/gmp_mpfr128/dot4_refs` | 202.53 ns | 201.01 ns - 204.26 ns | 199.15 ns | -0.46% | - |
| `gmp_real_linear_algebra_api/gmp_mpfr128/eval_poly` | 120.80 ns | 120.38 ns - 121.30 ns | 120.19 ns | +3.59% | - |
| `gmp_real_linear_algebra_api/gmp_mpfr128/eval_rational_poly` | 257.65 ns | 256.59 ns - 258.83 ns | 255.94 ns | +5.95% | - |
| `gmp_real_linear_algebra_api/gmp_mpfr128/linear_combination3_refs` | 268.38 ns | 266.61 ns - 270.32 ns | 264.96 ns | +0.01% | - |
| `gmp_real_linear_algebra_api/gmp_mpfr128/linear_combination4_refs` | 199.65 ns | 198.59 ns - 200.83 ns | 197.35 ns | -0.72% | - |
| `gmp_real_linear_algebra_api/gmp_mpfr128/mul_add` | 72.05 ns | 71.70 ns - 72.44 ns | 71.39 ns | +0.17% | - |
| `gmp_real_linear_algebra_api/gmp_mpfr128/signed_product_sum` | 82.23 ns | 81.88 ns - 82.63 ns | 81.43 ns | +0.55% | - |
| `gmp_real_linear_algebra_api/gmp_mpfr128/sum_products` | 196.31 ns | 195.60 ns - 197.10 ns | 195.55 ns | +3.08% | - |
| `gmp_real_linear_algebra_api/hyperreal/active_dot2_refs` | 57.27 ns | 56.90 ns - 57.70 ns | 56.50 ns | +3.09% | - |
| `gmp_real_linear_algebra_api/hyperreal/active_dot3_refs` | 69.09 ns | 68.69 ns - 69.56 ns | 68.30 ns | +0.90% | - |
| `gmp_real_linear_algebra_api/hyperreal/active_dot4_refs` | 93.01 ns | 92.09 ns - 94.13 ns | 91.09 ns | +4.28% | - |
| `gmp_real_linear_algebra_api/hyperreal/active_linear_combination3_refs` | 70.56 ns | 70.06 ns - 71.11 ns | 69.74 ns | +3.94% | - |
| `gmp_real_linear_algebra_api/hyperreal/active_linear_combination4_refs` | 90.62 ns | 90.18 ns - 91.10 ns | 89.83 ns | +1.42% | - |
| `gmp_real_linear_algebra_api/hyperreal/active_signed_product_sum` | 54.31 ns | 54.11 ns - 54.53 ns | 53.93 ns | +0.22% | - |
| `gmp_real_linear_algebra_api/hyperreal/affine_combination3_refs` | 88.37 ns | 87.50 ns - 89.40 ns | 86.92 ns | +2.09% | - |
| `gmp_real_linear_algebra_api/hyperreal/affine_combination4_refs` | 115.59 ns | 114.53 ns - 116.72 ns | 113.76 ns | +3.80% | - |
| `gmp_real_linear_algebra_api/hyperreal/diff_of_products` | 51.19 ns | 50.99 ns - 51.41 ns | 50.79 ns | +0.92% | - |
| `gmp_real_linear_algebra_api/hyperreal/dot2_refs` | 57.78 ns | 57.44 ns - 58.13 ns | 57.35 ns | +2.08% | - |
| `gmp_real_linear_algebra_api/hyperreal/dot3_refs` | 72.28 ns | 71.83 ns - 72.79 ns | 71.42 ns | +4.15% | - |
| `gmp_real_linear_algebra_api/hyperreal/dot4_refs` | 92.77 ns | 92.10 ns - 93.48 ns | 91.66 ns | +5.21% | - |
| `gmp_real_linear_algebra_api/hyperreal/eval_poly` | 99.02 ns | 98.35 ns - 99.75 ns | 97.88 ns | +2.32% | - |
| `gmp_real_linear_algebra_api/hyperreal/eval_rational_poly` | 291.45 ns | 289.87 ns - 293.27 ns | 289.71 ns | -0.19% | - |
| `gmp_real_linear_algebra_api/hyperreal/linear_combination3_refs` | 73.37 ns | 72.73 ns - 74.08 ns | 72.07 ns | +5.07% | - |
| `gmp_real_linear_algebra_api/hyperreal/linear_combination4_refs` | 91.32 ns | 90.76 ns - 91.93 ns | 90.20 ns | +2.58% | - |
| `gmp_real_linear_algebra_api/hyperreal/mul_add` | 95.56 ns | 95.08 ns - 96.07 ns | 95.03 ns | +3.85% | - |
| `gmp_real_linear_algebra_api/hyperreal/signed_product_sum` | 54.12 ns | 53.83 ns - 54.43 ns | 53.66 ns | -0.78% | - |
| `gmp_real_linear_algebra_api/hyperreal/sum_products` | 98.87 ns | 97.56 ns - 100.61 ns | 96.61 ns | +4.40% | - |
| `inverse_hyperbolic_adversarial_approx/acosh_large_positive_p128` | 5.97 us | 5.75 us - 6.24 us | 5.92 us | -0.74% | - |
| `inverse_hyperbolic_adversarial_approx/acosh_one_plus_tiny_p128` | 3.98 us | 3.97 us - 3.99 us | 3.98 us | -7.67% | - |
| `inverse_hyperbolic_adversarial_approx/acosh_sqrt_two_p128` | 86.79 ns | 86.48 ns - 87.18 ns | 86.69 ns | -1.16% | - |
| `inverse_hyperbolic_adversarial_approx/acosh_two_p128` | 47.93 ns | 47.83 ns - 48.03 ns | 47.94 ns | -2.80% | - |
| `inverse_hyperbolic_adversarial_approx/asinh_large_negative_p128` | 6.30 us | 6.17 us - 6.49 us | 6.22 us | +0.07% | - |
| `inverse_hyperbolic_adversarial_approx/asinh_large_positive_p128` | 5.83 us | 5.77 us - 5.89 us | 5.81 us | +1.12% | - |
| `inverse_hyperbolic_adversarial_approx/asinh_mid_positive_p128` | 10.28 us | 10.07 us - 10.52 us | 10.07 us | +1.01% | - |
| `inverse_hyperbolic_adversarial_approx/asinh_tiny_positive_p128` | 550.40 ns | 543.80 ns - 560.72 ns | 545.87 ns | +1.72% | - |
| `inverse_hyperbolic_adversarial_approx/atanh_mid_positive_p128` | 198.68 ns | 196.92 ns - 200.67 ns | 197.18 ns | -2.70% | - |
| `inverse_hyperbolic_adversarial_approx/atanh_near_minus_one_p128` | 3.25 us | 3.17 us - 3.34 us | 3.17 us | +1.85% | - |
| `inverse_hyperbolic_adversarial_approx/atanh_near_one_p128` | 3.01 us | 3.00 us - 3.02 us | 3.00 us | -2.65% | - |
| `inverse_hyperbolic_adversarial_approx/atanh_tiny_positive_p128` | 506.19 ns | 503.35 ns - 510.09 ns | 504.25 ns | -3.33% | - |
| `inverse_trig_adversarial_approx/acos_mid_positive_p96` | 5.72 us | 5.66 us - 5.80 us | 5.66 us | +1.50% | - |
| `inverse_trig_adversarial_approx/acos_near_minus_one_p96` | 1.68 us | 1.68 us - 1.69 us | 1.68 us | -0.58% | - |
| `inverse_trig_adversarial_approx/acos_near_one_p96` | 1.57 us | 1.57 us - 1.58 us | 1.57 us | -1.75% | - |
| `inverse_trig_adversarial_approx/acos_tiny_positive_p96` | 1.56 us | 1.53 us - 1.59 us | 1.54 us | +1.10% | - |
| `inverse_trig_adversarial_approx/acos_zero_p96` | 173.03 ns | 172.45 ns - 173.63 ns | 173.21 ns | -4.78% | - |
| `inverse_trig_adversarial_approx/asin_mid_positive_p96` | 6.56 us | 6.42 us - 6.71 us | 6.46 us | +2.17% | - |
| `inverse_trig_adversarial_approx/asin_near_minus_one_p96` | 1.89 us | 1.88 us - 1.90 us | 1.89 us | -2.56% | - |
| `inverse_trig_adversarial_approx/asin_near_one_p96` | 1.92 us | 1.91 us - 1.93 us | 1.92 us | -1.23% | - |
| `inverse_trig_adversarial_approx/asin_tiny_positive_p96` | 401.46 ns | 397.76 ns - 405.78 ns | 400.25 ns | -1.94% | - |
| `inverse_trig_adversarial_approx/asin_zero_p96` | 39.25 ns | 38.89 ns - 39.63 ns | 38.94 ns | +0.18% | - |
| `inverse_trig_adversarial_approx/atan_huge_p96` | 924.52 ns | 901.88 ns - 947.06 ns | 935.80 ns | -7.63% | - |
| `inverse_trig_adversarial_approx/atan_large_p96` | 1.86 us | 1.85 us - 1.87 us | 1.86 us | -0.79% | - |
| `inverse_trig_adversarial_approx/atan_mid_positive_p96` | 2.14 us | 2.12 us - 2.16 us | 2.15 us | -1.12% | - |
| `inverse_trig_adversarial_approx/atan_promoted_generated_783_412_p96` | 1.92 us | 1.91 us - 1.93 us | 1.91 us | -1.15% | - |
| `inverse_trig_adversarial_approx/atan_tiny_positive_p96` | 515.64 ns | 502.31 ns - 529.85 ns | 503.90 ns | +5.26% | - |
| `inverse_trig_adversarial_approx/atan_two_thirds_anchor_sweep_p256` | 20.27 us | 20.05 us - 20.60 us | 20.10 us | -2.32% | - |
| `inverse_trig_adversarial_approx/atan_two_thirds_anchor_sweep_p32` | 5.81 us | 5.78 us - 5.85 us | 5.81 us | -0.93% | - |
| `inverse_trig_adversarial_approx/atan_two_thirds_anchor_sweep_p96` | 9.69 us | 9.57 us - 9.84 us | 9.57 us | -4.89% | - |
| `inverse_trig_adversarial_approx/atan_two_thirds_anchor_upper_edge_p96` | 2.78 us | 2.76 us - 2.79 us | 2.78 us | -4.64% | - |
| `inverse_trig_adversarial_approx/atan_zero_p96` | 137.36 ns | 136.22 ns - 139.04 ns | 136.49 ns | -1.38% | - |
| `inverse_trig_adversarial_approx/ln_square_plus_one_promoted_generated_677_222_p96` | 31.54 ns | 31.08 ns - 32.07 ns | 31.34 ns | +0.96% | - |
| `iterator_products/rational_wallis_borrowed_1000` | 1.56 ms | 1.55 ms - 1.56 ms | 1.54 ms | +1.96% | - |
| `iterator_products/rational_wallis_owned_1000` | 1.51 ms | 1.51 ms - 1.51 ms | 1.50 ms | -0.83% | - |
| `iterator_products/rational_wallis_sequential_1000` | 53.93 ms | 53.84 ms - 54.03 ms | 53.80 ms | +0.19% | - |
| `iterator_products/real_wallis_owned_1000` | 1.51 ms | 1.51 ms - 1.52 ms | 1.51 ms | -1.21% | - |
| `promoted_library_slow_offenders_approx/atan_generated_10869_1_123_155_p96` | 2.18 us | 2.16 us - 2.20 us | 2.17 us | -1.81% | - |
| `promoted_library_slow_offenders_approx/atan_generated_11034_1_367_518_p96` | 2.34 us | 2.32 us - 2.37 us | 2.33 us | -2.17% | - |
| `promoted_library_slow_offenders_approx/atan_generated_15279_1_403_522_p96` | 2.22 us | 2.22 us - 2.23 us | 2.22 us | -4.96% | - |
| `promoted_library_slow_offenders_approx/atan_generated_15369_neg_1_74_93_p96` | 2.36 us | 2.32 us - 2.40 us | 2.33 us | -1.76% | - |
| `promoted_library_slow_offenders_approx/atan_generated_15474_neg_1_13_19_p96` | 2.47 us | 2.45 us - 2.48 us | 2.47 us | -8.19% | - |
| `promoted_library_slow_offenders_approx/atan_generated_2964_neg_1_146_373_p96` | 2.28 us | 2.25 us - 2.31 us | 2.26 us | -1.21% | - |
| `promoted_library_slow_offenders_approx/atan_generated_5094_neg_1_347_604_p96` | 2.13 us | 2.06 us - 2.20 us | 2.05 us | - | - |
| `promoted_library_slow_offenders_approx/atan_generated_5124_neg_1_237_523_p96` | 2.00 us | 1.98 us - 2.03 us | 2.00 us | -6.23% | - |
| `promoted_library_slow_offenders_approx/atan_generated_849_neg_1_391_600_p96` | 2.37 us | 2.33 us - 2.40 us | 2.34 us | -5.14% | - |
| `promoted_library_slow_offenders_approx/cos_generated_15110_7_5_27_p96` | 2.30 us | 2.29 us - 2.31 us | 2.31 us | -2.68% | - |
| `promoted_library_slow_offenders_approx/cos_generated_16610_7_4_19_p96` | 2.34 us | 2.31 us - 2.37 us | 2.32 us | +0.72% | - |
| `promoted_library_slow_offenders_approx/cos_generated_9365_7_14_139_p96` | 2.41 us | 2.37 us - 2.49 us | 2.37 us | -0.60% | - |
| `promoted_library_slow_offenders_approx/cos_generated_9950_neg_5_1_5_p96` | 2.20 us | 2.13 us - 2.30 us | 2.15 us | +0.00% | - |
| `promoted_library_slow_offenders_approx/ln_generated_10327_neg_1_19_732_p96` | 2.66 us | 2.62 us - 2.72 us | 2.63 us | -3.62% | - |
| `promoted_library_slow_offenders_approx/ln_generated_10702_2_201_218_p96` | 3.42 us | 3.31 us - 3.54 us | 3.38 us | +5.44% | - |
| `promoted_library_slow_offenders_approx/ln_generated_10732_6_6_137_p96` | 3.05 us | 3.03 us - 3.06 us | 3.04 us | +0.25% | - |
| `promoted_library_slow_offenders_approx/ln_generated_10927_neg_8_57_109_p96` | 3.03 us | 3.03 us - 3.04 us | 3.03 us | -2.67% | - |
| `promoted_library_slow_offenders_approx/ln_generated_11122_neg_8_13_27_p96` | 3.14 us | 3.08 us - 3.20 us | 3.11 us | +3.83% | - |
| `promoted_library_slow_offenders_approx/ln_generated_11317_neg_8_21_53_p96` | 2.92 us | 2.90 us - 2.92 us | 2.92 us | -1.40% | - |
| `promoted_library_slow_offenders_approx/ln_generated_11497_1_137_564_p96` | 3.61 us | 3.58 us - 3.65 us | 3.60 us | -0.15% | - |
| `promoted_library_slow_offenders_approx/ln_generated_1297_neg_1_83_188_p96` | 3.50 us | 3.49 us - 3.51 us | 3.50 us | -4.78% | - |
| `promoted_library_slow_offenders_approx/ln_generated_13537_neg_7_17_41_p96` | 3.02 us | 2.99 us - 3.07 us | 2.99 us | -0.73% | - |
| `promoted_library_slow_offenders_approx/ln_generated_1372_neg_1_309_484_p96` | 2.99 us | 2.95 us - 3.03 us | 2.97 us | +3.27% | - |
| `promoted_library_slow_offenders_approx/ln_generated_13837_neg_1_55_76_p96` | 2.32 us | 2.31 us - 2.34 us | 2.32 us | -2.62% | - |
| `promoted_library_slow_offenders_approx/ln_generated_14377_neg_1_189_764_p96` | 3.74 us | 3.65 us - 3.84 us | 3.70 us | +1.07% | - |
| `promoted_library_slow_offenders_approx/ln_generated_14947_3_11_222_p96` | 3.65 us | 3.56 us - 3.77 us | 3.59 us | +3.51% | - |
| `promoted_library_slow_offenders_approx/ln_generated_14977_6_22_141_p96` | 3.30 us | 3.25 us - 3.37 us | 3.27 us | +0.85% | - |
| `promoted_library_slow_offenders_approx/ln_generated_15082_1_181_356_p96` | 3.47 us | 3.46 us - 3.48 us | 3.46 us | +2.06% | - |
| `promoted_library_slow_offenders_approx/ln_generated_15472_neg_3_13_50_p96` | 3.98 us | 3.93 us - 4.04 us | 3.94 us | +3.28% | - |
| `promoted_library_slow_offenders_approx/ln_generated_16402_1_11_52_p96` | 3.44 us | 3.42 us - 3.47 us | 3.42 us | +0.60% | - |
| `promoted_library_slow_offenders_approx/ln_generated_16447_1_9_20_p96` | 3.53 us | 3.52 us - 3.53 us | 3.53 us | -4.31% | - |
| `promoted_library_slow_offenders_approx/ln_generated_16597_neg_1_15_188_p96` | 2.71 us | 2.71 us - 2.71 us | 2.71 us | -1.35% | - |
| `promoted_library_slow_offenders_approx/ln_generated_16642_3_13_22_p96` | 3.14 us | 3.13 us - 3.15 us | 3.14 us | -7.43% | - |
| `promoted_library_slow_offenders_approx/ln_generated_1687_1_27_44_p96` | 3.08 us | 3.01 us - 3.18 us | 3.01 us | +1.93% | - |
| `promoted_library_slow_offenders_approx/ln_generated_17197_neg_7_29_43_p96` | 2.66 us | 2.64 us - 2.69 us | 2.64 us | -3.21% | - |
| `promoted_library_slow_offenders_approx/ln_generated_17392_neg_7_41_101_p96` | 3.09 us | 3.08 us - 3.10 us | 3.08 us | -3.10% | - |
| `promoted_library_slow_offenders_approx/ln_generated_17587_neg_6_68_73_p96` | 3.53 us | 3.52 us - 3.55 us | 3.53 us | -1.70% | - |
| `promoted_library_slow_offenders_approx/ln_generated_17752_neg_3_11_18_p96` | 3.04 us | 3.03 us - 3.05 us | 3.04 us | +0.67% | - |
| `promoted_library_slow_offenders_approx/ln_generated_18352_neg_1_133_500_p96` | 3.76 us | 3.71 us - 3.82 us | 3.74 us | +0.47% | - |
| `promoted_library_slow_offenders_approx/ln_generated_1837_neg_1_107_724_p96` | 3.17 us | 3.15 us - 3.19 us | 3.16 us | -0.03% | - |
| `promoted_library_slow_offenders_approx/ln_generated_2242_5_103_129_p96` | 2.79 us | 2.73 us - 2.86 us | 2.75 us | -2.68% | - |
| `promoted_library_slow_offenders_approx/ln_generated_2632_neg_10_37_73_p96` | 3.08 us | 3.04 us - 3.16 us | 3.05 us | +1.84% | - |
| `promoted_library_slow_offenders_approx/ln_generated_3007_1_65_556_p96` | 3.08 us | 3.04 us - 3.13 us | 3.05 us | -1.80% | - |
| `promoted_library_slow_offenders_approx/ln_generated_322_1_95_164_p96` | 3.22 us | 3.14 us - 3.31 us | 3.17 us | -0.42% | - |
| `promoted_library_slow_offenders_approx/ln_generated_5812_neg_1_51_460_p96` | 3.08 us | 3.05 us - 3.13 us | 3.07 us | -0.55% | - |
| `promoted_library_slow_offenders_approx/ln_generated_6457_2_169_214_p96` | 2.95 us | 2.89 us - 3.02 us | 2.92 us | +4.11% | - |
| `promoted_library_slow_offenders_approx/ln_generated_6487_5_123_133_p96` | 3.02 us | 2.97 us - 3.07 us | 2.99 us | -2.69% | - |
| `promoted_library_slow_offenders_approx/ln_generated_6592_1_109_348_p96` | 3.89 us | 3.85 us - 3.94 us | 3.86 us | +1.68% | - |
| `promoted_library_slow_offenders_approx/ln_generated_6682_neg_9_8_35_p96` | 3.72 us | 3.67 us - 3.77 us | 3.71 us | +3.67% | - |
| `promoted_library_slow_offenders_approx/ln_generated_6877_neg_9_34_77_p96` | 3.99 us | 3.95 us - 4.05 us | 3.97 us | +2.66% | - |
| `promoted_library_slow_offenders_approx/ln_generated_7072_neg_9_44_49_p96` | 3.58 us | 3.53 us - 3.62 us | 3.58 us | +4.07% | - |
| `promoted_library_slow_offenders_approx/ln_generated_7447_1_53_76_p96` | 2.53 us | 2.52 us - 2.54 us | 2.53 us | -1.74% | - |
| `promoted_library_slow_offenders_approx/ln_generated_7567_neg_1_31_52_p96` | 3.02 us | 3.01 us - 3.04 us | 3.01 us | +0.56% | - |
| `promoted_library_slow_offenders_approx/ln_generated_7642_neg_1_25_36_p96` | 2.71 us | 2.69 us - 2.73 us | 2.70 us | +0.20% | - |
| `promoted_library_slow_offenders_approx/ln_generated_7912_1_93_772_p96` | 3.10 us | 3.05 us - 3.15 us | 3.08 us | +1.18% | - |
| `promoted_library_slow_offenders_approx/ln_generated_8152_3_11_62_p96` | 3.82 us | 3.74 us - 3.91 us | 3.81 us | +3.60% | - |
| `promoted_library_slow_offenders_approx/ln_generated_9457_neg_3_23_90_p96` | 4.04 us | 3.98 us - 4.11 us | 3.99 us | +3.03% | - |
| `promoted_library_slow_offenders_approx/ln_generated_9862_neg_1_221_492_p96` | 3.56 us | 3.54 us - 3.57 us | 3.55 us | -2.50% | - |
| `promoted_library_slow_offenders_approx/tan_generated_1011_2_58_181_p96` | 10.12 us | 10.02 us - 10.25 us | 10.06 us | -1.87% | - |
| `promoted_library_slow_offenders_approx/tan_generated_1071_neg_3_177_200_p96` | 3.32 us | 3.25 us - 3.41 us | 3.26 us | -2.49% | - |
| `promoted_library_slow_offenders_approx/tan_generated_11391_3_29_36_p96` | 3.75 us | 3.74 us - 3.77 us | 3.75 us | -0.73% | - |
| `promoted_library_slow_offenders_approx/tan_generated_11421_neg_4_55_57_p96` | 7.97 us | 7.71 us - 8.22 us | 8.05 us | +5.15% | - |
| `promoted_library_slow_offenders_approx/tan_generated_11691_1_431_439_p96` | 10.61 us | 10.51 us - 10.69 us | 10.65 us | -0.04% | - |
| `promoted_library_slow_offenders_approx/tan_generated_11841_neg_5_2_17_p96` | 8.41 us | 8.12 us - 8.71 us | 8.23 us | +4.94% | - |
| `promoted_library_slow_offenders_approx/tan_generated_12081_neg_1_262_383_p96` | 10.56 us | 10.50 us - 10.62 us | 10.56 us | -3.31% | - |
| `promoted_library_slow_offenders_approx/tan_generated_12111_neg_1_76_151_p96` | 10.36 us | 10.24 us - 10.52 us | 10.32 us | -6.04% | - |
| `promoted_library_slow_offenders_approx/tan_generated_12186_neg_1_189_299_p96` | 10.52 us | 10.47 us - 10.57 us | 10.53 us | -7.37% | - |
| `promoted_library_slow_offenders_approx/tan_generated_12216_neg_1_268_517_p96` | 10.44 us | 10.39 us - 10.49 us | 10.45 us | -0.91% | - |
| `promoted_library_slow_offenders_approx/tan_generated_12561_4_19_21_p96` | 7.23 us | 7.08 us - 7.39 us | 7.25 us | +5.08% | - |
| `promoted_library_slow_offenders_approx/tan_generated_1296_neg_3_71_91_p96` | 3.69 us | 3.67 us - 3.72 us | 3.68 us | -7.08% | - |
| `promoted_library_slow_offenders_approx/tan_generated_13446_neg_5_15_187_p96` | 8.10 us | 7.88 us - 8.35 us | 7.92 us | +2.36% | - |
| `promoted_library_slow_offenders_approx/tan_generated_13836_neg_3_73_131_p96` | 8.02 us | 7.94 us - 8.12 us | 7.97 us | -1.97% | - |
| `promoted_library_slow_offenders_approx/tan_generated_13866_neg_5_1_2_p96` | 2.52 us | 2.51 us - 2.52 us | 2.51 us | -3.97% | - |
| `promoted_library_slow_offenders_approx/tan_generated_13911_neg_2_134_427_p96` | 10.82 us | 10.74 us - 10.91 us | 10.77 us | +1.26% | - |
| `promoted_library_slow_offenders_approx/tan_generated_14136_neg_1_79_106_p96` | 10.71 us | 10.51 us - 11.03 us | 10.60 us | -0.65% | - |
| `promoted_library_slow_offenders_approx/tan_generated_14421_5_25_47_p96` | 3.25 us | 3.22 us - 3.30 us | 3.22 us | -4.64% | - |
| `promoted_library_slow_offenders_approx/tan_generated_14946_4_104_125_p96` | 6.65 us | 6.61 us - 6.69 us | 6.64 us | -8.68% | - |
| `promoted_library_slow_offenders_approx/tan_generated_15081_1_205_259_p96` | 10.38 us | 10.23 us - 10.55 us | 10.32 us | -3.86% | - |
| `promoted_library_slow_offenders_approx/tan_generated_15891_neg_5_23_33_p96` | 4.01 us | 3.99 us - 4.02 us | 4.00 us | -1.95% | - |
| `promoted_library_slow_offenders_approx/tan_generated_16806_5_3_22_p96` | 7.84 us | 7.78 us - 7.91 us | 7.82 us | -1.70% | - |
| `promoted_library_slow_offenders_approx/tan_generated_17331_4_66_83_p96` | 6.11 us | 6.09 us - 6.13 us | 6.12 us | -0.93% | - |
| `promoted_library_slow_offenders_approx/tan_generated_17496_3_190_219_p96` | 3.43 us | 3.38 us - 3.48 us | 3.42 us | -5.65% | - |
| `promoted_library_slow_offenders_approx/tan_generated_18246_neg_1_187_188_p96` | 11.02 us | 10.92 us - 11.09 us | 11.04 us | -1.12% | - |
| `promoted_library_slow_offenders_approx/tan_generated_18276_neg_1_77_107_p96` | 10.52 us | 10.48 us - 10.56 us | 10.55 us | -5.89% | - |
| `promoted_library_slow_offenders_approx/tan_generated_18666_5_15_17_p96` | 7.93 us | 7.82 us - 8.05 us | 7.89 us | -4.14% | - |
| `promoted_library_slow_offenders_approx/tan_generated_2016_1_101_141_p96` | 10.49 us | 10.13 us - 10.89 us | 10.17 us | +1.51% | - |
| `promoted_library_slow_offenders_approx/tan_generated_321_1_214_231_p96` | 10.52 us | 10.29 us - 10.78 us | 10.40 us | -1.38% | - |
| `promoted_library_slow_offenders_approx/tan_generated_3321_neg_4_17_107_p96` | 3.98 us | 3.96 us - 4.00 us | 3.98 us | -0.73% | - |
| `promoted_library_slow_offenders_approx/tan_generated_3486_neg_2_37_80_p96` | 11.37 us | 11.01 us - 11.75 us | 11.16 us | +5.68% | - |
| `promoted_library_slow_offenders_approx/tan_generated_3591_neg_1_14_15_p96` | 10.67 us | 10.60 us - 10.74 us | 10.66 us | - | - |
| `promoted_library_slow_offenders_approx/tan_generated_3756_neg_1_123_214_p96` | 10.82 us | 10.74 us - 10.89 us | 10.85 us | -3.60% | - |
| `promoted_library_slow_offenders_approx/tan_generated_4401_2_5_13_p96` | 10.16 us | 10.11 us - 10.19 us | 10.17 us | -6.08% | - |
| `promoted_library_slow_offenders_approx/tan_generated_486_1_53_71_p96` | 10.19 us | 10.08 us - 10.30 us | 10.20 us | -1.41% | - |
| `promoted_library_slow_offenders_approx/tan_generated_5676_neg_1_215_229_p96` | 10.75 us | 10.65 us - 10.83 us | 10.77 us | +0.05% | - |
| `promoted_library_slow_offenders_approx/tan_generated_5916_neg_1_337_578_p96` | 10.77 us | 10.60 us - 10.96 us | 10.70 us | -3.63% | - |
| `promoted_library_slow_offenders_approx/tan_generated_6426_1_15_22_p96` | 10.28 us | 10.17 us - 10.41 us | 10.25 us | +0.16% | - |
| `promoted_library_slow_offenders_approx/tan_generated_6906_neg_87_128_p96` | 8.12 us | 8.07 us - 8.17 us | 8.14 us | +0.75% | - |
| `promoted_library_slow_offenders_approx/tan_generated_8976_1_71_73_p96` | 10.56 us | 10.43 us - 10.73 us | 10.55 us | -0.86% | - |
| `promoted_library_slow_offenders_approx/tan_generated_9231_neg_7_5_6_p96` | 5.44 us | 5.41 us - 5.46 us | 5.45 us | -1.53% | - |
| `promoted_library_slow_offenders_approx/tan_generated_9396_neg_4_128_155_p96` | 6.73 us | 6.52 us - 6.98 us | 6.64 us | +2.43% | - |
| `promoted_library_slow_offenders_approx/tan_generated_9591_neg_3_125_127_p96` | 3.47 us | 3.40 us - 3.55 us | 3.44 us | -1.68% | - |
| `promoted_slow_offender_score/score_promoted_100` | 2.84 ms | 2.81 ms - 2.90 ms | 2.81 ms | -2.91% | - |
| `pure_scalar_algorithm_speed/rational_add` | 8.76 ns | 8.73 ns - 8.79 ns | 8.72 ns | -5.13% | - |
| `pure_scalar_algorithm_speed/rational_add_shared_cold` | 98.39 ns | 97.66 ns - 99.03 ns | 98.76 ns | -2.81% | - |
| `pure_scalar_algorithm_speed/rational_add_wide_dyadic_cold` | 92.70 ns | 91.60 ns - 93.72 ns | 94.08 ns | +0.94% | - |
| `pure_scalar_algorithm_speed/rational_cross_difference_unit_divisor_composed_cold` | 593.91 ns | 592.58 ns - 595.37 ns | 592.73 ns | +12.21% | - |
| `pure_scalar_algorithm_speed/rational_cross_difference_unit_divisor_fused_cold` | 193.14 ns | 186.03 ns - 206.13 ns | 185.49 ns | +2.14% | - |
| `pure_scalar_algorithm_speed/rational_div` | 162.70 ns | 162.23 ns - 163.28 ns | 162.06 ns | +0.94% | - |
| `pure_scalar_algorithm_speed/rational_inverse_owned_cold` | 20.86 ns | 20.82 ns - 20.91 ns | 20.82 ns | -1.14% | - |
| `pure_scalar_algorithm_speed/rational_inverse_retained` | 7.63 ns | 7.58 ns - 7.68 ns | 7.53 ns | +1.53% | - |
| `pure_scalar_algorithm_speed/rational_mul` | 22.51 ns | 22.43 ns - 22.61 ns | 22.40 ns | +5.49% | - |
| `pure_scalar_algorithm_speed/rational_mul_dyadic_general_cross_cancel` | 11.77 ns | 11.67 ns - 11.90 ns | 11.57 ns | +1.89% | - |
| `pure_scalar_algorithm_speed/rational_mul_retained_general` | 11.67 ns | 11.61 ns - 11.74 ns | 11.56 ns | +0.99% | - |
| `pure_scalar_algorithm_speed/rational_mul_wide_dyadic_cold` | 191.69 ns | 188.68 ns - 194.54 ns | 199.10 ns | -0.64% | - |
| `pure_scalar_algorithm_speed/rational_neg_owned_cold` | 9.35 ns | 9.29 ns - 9.42 ns | 9.26 ns | -0.99% | - |
| `pure_scalar_algorithm_speed/rational_neg_retained` | 7.81 ns | 7.78 ns - 7.83 ns | 7.77 ns | -0.15% | - |
| `pure_scalar_algorithm_speed/rational_scaled_difference_composed_cold` | 264.51 ns | 263.12 ns - 265.93 ns | 266.02 ns | +3.99% | - |
| `pure_scalar_algorithm_speed/rational_scaled_difference_fused_cold` | 95.31 ns | 94.53 ns - 96.05 ns | 95.71 ns | +1.44% | - |
| `pure_scalar_algorithm_speed/rational_sub` | 9.44 ns | 9.35 ns - 9.53 ns | 9.69 ns | +3.26% | - |
| `pure_scalar_algorithm_speed/rational_sub_shared_cold` | 97.91 ns | 97.07 ns - 98.70 ns | 97.91 ns | -0.58% | - |
| `pure_scalar_algorithm_speed/rational_sub_wide_dyadic_cold` | 98.71 ns | 94.79 ns - 105.54 ns | 96.81 ns | +3.60% | - |
| `pure_scalar_algorithm_speed/real_exact_add` | 17.98 ns | 17.91 ns - 18.07 ns | 17.88 ns | -1.01% | - |
| `pure_scalar_algorithm_speed/real_exact_average_pair` | 139.52 ns | 139.09 ns - 140.01 ns | 138.79 ns | +0.36% | - |
| `pure_scalar_algorithm_speed/real_exact_average_pair_expanded` | 223.63 ns | 223.06 ns - 224.23 ns | 222.58 ns | +1.89% | - |
| `pure_scalar_algorithm_speed/real_exact_div` | 182.22 ns | 182.06 ns - 182.43 ns | 182.04 ns | +1.18% | - |
| `pure_scalar_algorithm_speed/real_exact_dyadic_radical_scale` | 36.33 ns | 36.24 ns - 36.43 ns | 36.21 ns | -0.05% | - |
| `pure_scalar_algorithm_speed/real_exact_dyadic_sqrt_reduce` | 95.35 ns | 94.92 ns - 95.83 ns | 94.69 ns | +0.78% | - |
| `pure_scalar_algorithm_speed/real_exact_general_sqrt_reduce` | 94.01 ns | 93.77 ns - 94.28 ns | 93.71 ns | +0.41% | - |
| `pure_scalar_algorithm_speed/real_exact_ln_reduce` | 81.77 ns | 81.15 ns - 82.53 ns | 80.71 ns | +1.12% | - |
| `pure_scalar_algorithm_speed/real_exact_mul` | 31.60 ns | 31.53 ns - 31.68 ns | 31.60 ns | +3.68% | - |
| `pure_scalar_algorithm_speed/real_exact_mul_retained` | 20.66 ns | 20.62 ns - 20.70 ns | 20.60 ns | -2.61% | - |
| `pure_scalar_algorithm_speed/real_exact_powi_i64_owned_cold` | 255.83 ns | 254.32 ns - 257.69 ns | 253.30 ns | -1.10% | - |
| `pure_scalar_algorithm_speed/real_exact_powi_i64_retained` | 58.43 ns | 58.03 ns - 58.90 ns | 57.80 ns | -0.56% | - |
| `pure_scalar_algorithm_speed/real_exact_sqrt_owned_cold` | 220.08 ns | 218.71 ns - 221.73 ns | 217.43 ns | +0.61% | - |
| `pure_scalar_algorithm_speed/real_exact_sqrt_reduce` | 100.37 ns | 99.87 ns - 100.94 ns | 99.49 ns | +1.94% | - |
| `pure_scalar_algorithm_speed/real_exact_sub` | 18.37 ns | 18.25 ns - 18.51 ns | 18.16 ns | +0.68% | - |
| `pure_scalar_algorithm_speed/real_pow_small_integer_exponent` | 130.64 ns | 129.92 ns - 131.52 ns | 129.16 ns | +0.18% | - |
| `rational_algorithm_dispatch_speed/backend_batch16_65536_by_4096` | 1.21 ms | 1.21 ms - 1.22 ms | 1.21 ms | +0.54% | - |
| `rational_algorithm_dispatch_speed/backend_batch16_8192_by_1024` | 49.51 us | 49.42 us - 49.64 us | 49.43 us | +0.07% | - |
| `rational_algorithm_dispatch_speed/backend_one_shot_8192_by_1024` | 2.79 us | 2.79 us - 2.80 us | 2.79 us | +0.10% | - |
| `rational_algorithm_dispatch_speed/barrett_batch16_65536_by_4096` | 1.47 ms | 1.47 ms - 1.47 ms | 1.47 ms | -0.04% | - |
| `rational_algorithm_dispatch_speed/barrett_batch16_8192_by_1024` | 85.58 us | 85.48 us - 85.68 us | 85.48 us | -1.37% | - |
| `rational_algorithm_dispatch_speed/barrett_one_shot_8192_by_1024` | 5.75 us | 5.75 us - 5.77 us | 5.75 us | -1.23% | - |
| `rational_algorithm_dispatch_speed/compare_dyadic_shifted_retained_1024_bits` | 10.92 ns | 10.89 ns - 10.96 ns | 10.85 ns | -6.99% | - |
| `rational_algorithm_dispatch_speed/compare_leading_significand_retained_1024_bits` | 46.18 ns | 45.92 ns - 46.48 ns | 45.77 ns | +1.79% | - |
| `rational_algorithm_dispatch_speed/division_trivial_small_quotient` | 83.35 ns | 82.72 ns - 84.06 ns | 82.10 ns | +2.94% | - |
| `rational_algorithm_dispatch_speed/dyadic_fact_cold` | 38.15 ns | 37.06 ns - 39.13 ns | 39.57 ns | -0.46% | - |
| `rational_algorithm_dispatch_speed/dyadic_fact_retained` | 1.95 ns | 1.92 ns - 1.98 ns | 1.89 ns | +3.48% | - |
| `rational_algorithm_dispatch_speed/equality_close_retained_1024_bits` | 182.84 ns | 182.34 ns - 183.42 ns | 181.84 ns | +0.31% | - |
| `rational_algorithm_dispatch_speed/equality_different_signs_retained` | 2.39 ns | 2.38 ns - 2.40 ns | 2.37 ns | +0.60% | - |
| `rational_algorithm_dispatch_speed/equality_dyadic_shifted_retained_1024_bits` | 10.93 ns | 10.91 ns - 10.96 ns | 10.88 ns | -4.02% | - |
| `rational_algorithm_dispatch_speed/equality_equal_distinct_retained` | 7.31 ns | 7.27 ns - 7.35 ns | 7.29 ns | +19.01% | - |
| `rational_algorithm_dispatch_speed/equality_leading_significand_retained_1024_bits` | 173.91 ns | 173.39 ns - 174.49 ns | 173.35 ns | +0.20% | - |
| `rational_algorithm_dispatch_speed/equality_same_denominator_retained` | 6.48 ns | 6.45 ns - 6.51 ns | 6.43 ns | +1.31% | - |
| `rational_algorithm_dispatch_speed/equality_shared_identity_retained` | 3.64 ns | 3.62 ns - 3.65 ns | 3.63 ns | +0.36% | - |
| `rational_algorithm_dispatch_speed/equality_word_retained` | 10.50 ns | 10.42 ns - 10.59 ns | 10.31 ns | +0.53% | - |
| `rational_algorithm_dispatch_speed/exact_remainder_large_knuth` | 5.09 us | 5.07 us - 5.11 us | 5.06 us | +1.75% | - |
| `rational_algorithm_dispatch_speed/gcd_euclidean_1024_bits` | 75.67 us | 75.26 us - 76.12 us | 74.77 us | +1.42% | - |
| `rational_algorithm_dispatch_speed/gcd_euclidean_128_bits` | 5.59 us | 5.54 us - 5.64 us | 5.49 us | +0.96% | - |
| `rational_algorithm_dispatch_speed/gcd_euclidean_192_bits` | 8.77 us | 8.73 us - 8.82 us | 8.72 us | -1.20% | - |
| `rational_algorithm_dispatch_speed/gcd_euclidean_4096_bits` | 496.05 us | 493.55 us - 498.81 us | 493.22 us | +0.81% | - |
| `rational_algorithm_dispatch_speed/gcd_euclidean_512_bits` | 32.53 us | 32.32 us - 32.79 us | 32.17 us | +0.63% | - |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_1024_bits` | 73.72 us | 73.54 us - 73.95 us | 73.60 us | -0.84% | - |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_192_bits` | 8.53 us | 8.44 us - 8.63 us | 8.37 us | +1.56% | - |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_256_bits` | 13.73 us | 13.67 us - 13.78 us | 13.63 us | -0.82% | - |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_4096_bits` | 510.85 us | 509.31 us - 513.17 us | 508.91 us | +0.88% | - |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_512_bits` | 30.71 us | 30.67 us - 30.74 us | 30.63 us | -0.90% | - |
| `rational_algorithm_dispatch_speed/gcd_selected_1024_bits` | 21.33 us | 21.19 us - 21.47 us | 21.06 us | +1.99% | - |
| `rational_algorithm_dispatch_speed/gcd_selected_128_bits` | 129.68 ns | 129.35 ns - 130.07 ns | 129.17 ns | +0.50% | - |
| `rational_algorithm_dispatch_speed/gcd_selected_192_bits` | 5.43 us | 5.39 us - 5.47 us | 5.36 us | +1.88% | - |
| `rational_algorithm_dispatch_speed/gcd_selected_4096_bits` | 116.45 us | 115.07 us - 118.04 us | 113.47 us | +3.62% | - |
| `rational_algorithm_dispatch_speed/gcd_selected_512_bits` | 11.04 us | 10.98 us - 11.11 us | 10.94 us | +2.46% | - |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_1024_bits` | 23.44 us | 23.37 us - 23.52 us | 23.32 us | -1.08% | - |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_192_bits` | 8.50 us | 8.46 us - 8.55 us | 8.43 us | +0.36% | - |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_256_bits` | 8.09 us | 8.03 us - 8.16 us | 7.98 us | -0.42% | - |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_4096_bits` | 122.57 us | 122.41 us - 122.74 us | 122.54 us | +2.88% | - |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_512_bits` | 12.22 us | 12.13 us - 12.32 us | 12.09 us | +2.14% | - |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_1048576_bits` | 3.83 s | 3.83 s - 3.83 s | 3.83 s | -0.17% | - |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_16384_bits` | 3.23 ms | 3.22 ms - 3.24 ms | 3.21 ms | +0.33% | - |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_262144_bits` | 272.15 ms | 271.69 ms - 272.65 ms | 271.34 ms | +0.54% | - |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_65536_bits` | 11.36 ms | 11.32 ms - 11.40 ms | 11.28 ms | +0.87% | - |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_8192_bits` | 296.56 us | 296.14 us - 297.01 us | 296.13 us | +0.29% | - |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_1048576_bits` | 1.98 s | 1.98 s - 1.98 s | 1.98 s | +1.62% | - |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_16384_bits` | 877.29 us | 873.88 us - 880.94 us | 869.73 us | +1.64% | - |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_262144_bits` | 126.53 ms | 126.28 ms - 126.87 ms | 126.18 ms | +1.60% | - |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_65536_bits` | 8.93 ms | 8.91 ms - 8.96 ms | 8.90 ms | +1.27% | - |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_8192_bits` | 303.26 us | 301.45 us - 305.25 us | 300.77 us | +2.08% | - |
| `rational_algorithm_dispatch_speed/mul_backend_basecase_cold` | 361.29 ns | 257.72 ns - 566.34 ns | 260.99 ns | -6.46% | - |
| `rational_algorithm_dispatch_speed/mul_backend_half_karatsuba_cold` | 625.75 ns | 499.12 ns - 876.30 ns | 504.30 ns | +26.60% | - |
| `rational_algorithm_dispatch_speed/mul_backend_karatsuba_cold` | 816.38 ns | 813.13 ns - 819.81 ns | 819.18 ns | -1.44% | - |
| `rational_algorithm_dispatch_speed/mul_backend_reference_1048576_bits` | 12.88 ms | 12.82 ms - 12.95 ms | 12.75 ms | +1.97% | - |
| `rational_algorithm_dispatch_speed/mul_backend_reference_131072_bits` | 593.87 us | 591.53 us - 596.59 us | 588.84 us | +0.84% | - |
| `rational_algorithm_dispatch_speed/mul_backend_reference_16384_bits` | 26.39 us | 26.31 us - 26.48 us | 26.26 us | -0.18% | - |
| `rational_algorithm_dispatch_speed/mul_backend_reference_2097152_bits` | 35.60 ms | 35.53 ms - 35.68 ms | 35.48 ms | +1.08% | - |
| `rational_algorithm_dispatch_speed/mul_backend_reference_262144_bits` | 1.64 ms | 1.64 ms - 1.65 ms | 1.63 ms | +1.69% | - |
| `rational_algorithm_dispatch_speed/mul_backend_reference_4096_bits` | 2.80 us | 2.79 us - 2.82 us | 2.78 us | +1.00% | - |
| `rational_algorithm_dispatch_speed/mul_backend_reference_524288_bits` | 4.49 ms | 4.47 ms - 4.50 ms | 4.47 ms | +0.43% | - |
| `rational_algorithm_dispatch_speed/mul_backend_reference_65536_bits` | 203.03 us | 202.65 us - 203.45 us | 202.38 us | +0.24% | - |
| `rational_algorithm_dispatch_speed/mul_backend_toom3_cold` | 8.41 us | 8.37 us - 8.44 us | 8.37 us | +1.49% | - |
| `rational_algorithm_dispatch_speed/mul_backend_unbalanced_1258291_by_1048576` | 16.31 ms | 16.25 ms - 16.38 ms | 16.20 ms | +1.13% | - |
| `rational_algorithm_dispatch_speed/mul_backend_unbalanced_599186_by_524288` | 5.36 ms | 5.35 ms - 5.38 ms | 5.33 ms | +0.95% | - |
| `rational_algorithm_dispatch_speed/mul_ntt_candidate_1048576_bits` | 73.25 ms | 73.09 ms - 73.43 ms | 73.02 ms | +1.08% | - |
| `rational_algorithm_dispatch_speed/mul_ntt_candidate_262144_bits` | 16.20 ms | 16.15 ms - 16.26 ms | 16.14 ms | +0.84% | - |
| `rational_algorithm_dispatch_speed/mul_ntt_candidate_4194304_bits` | 328.43 ms | 327.81 ms - 329.07 ms | 327.43 ms | -0.12% | - |
| `rational_algorithm_dispatch_speed/mul_selected_1048576_bits` | 10.77 ms | 10.75 ms - 10.81 ms | 10.72 ms | +0.14% | - |
| `rational_algorithm_dispatch_speed/mul_selected_2097152_bits` | 28.23 ms | 28.12 ms - 28.36 ms | 28.04 ms | +0.41% | - |
| `rational_algorithm_dispatch_speed/mul_selected_262144_bits` | 1.52 ms | 1.52 ms - 1.53 ms | 1.52 ms | +0.43% | - |
| `rational_algorithm_dispatch_speed/mul_selected_4194304_bits` | 73.81 ms | 73.71 ms - 73.93 ms | 73.74 ms | -0.18% | - |
| `rational_algorithm_dispatch_speed/mul_selected_524288_bits` | 4.01 ms | 3.99 ms - 4.02 ms | 3.98 ms | +1.59% | - |
| `rational_algorithm_dispatch_speed/mul_selected_toom4_unbalanced_1258291_by_1048576` | 15.00 ms | 14.95 ms - 15.05 ms | 14.95 ms | +1.19% | - |
| `rational_algorithm_dispatch_speed/mul_selected_toom6_unbalanced_599186_by_524288` | 4.81 ms | 4.80 ms - 4.84 ms | 4.80 ms | +0.92% | - |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_1048576_bits` | 12.22 ms | 12.19 ms - 12.25 ms | 12.18 ms | +0.62% | - |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_16384_bits` | 34.69 us | 34.63 us - 34.76 us | 34.62 us | +1.05% | - |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_2097152_bits` | 32.91 ms | 32.84 ms - 32.98 ms | 32.76 ms | +0.55% | - |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_262144_bits` | 1.65 ms | 1.64 ms - 1.65 ms | 1.63 ms | +0.62% | - |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_4096_bits` | 8.27 us | 8.20 us - 8.34 us | 8.11 us | +2.97% | - |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_524288_bits` | 4.60 ms | 4.58 ms - 4.61 ms | 4.57 ms | +1.58% | - |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_65536_bits` | 225.45 us | 225.07 us - 225.99 us | 225.00 us | +0.06% | - |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_1048576_bits` | 10.92 ms | 10.89 ms - 10.97 ms | 10.87 ms | -0.12% | - |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_131072_bits` | 595.26 us | 594.53 us - 596.06 us | 594.15 us | +1.43% | - |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_2097152_bits` | 30.66 ms | 30.51 ms - 30.83 ms | 30.32 ms | +2.47% | - |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_262144_bits` | 1.59 ms | 1.58 ms - 1.59 ms | 1.58 ms | +1.63% | - |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_524288_bits` | 4.14 ms | 4.13 ms - 4.16 ms | 4.13 ms | +0.49% | - |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_1048576_bits` | 10.76 ms | 10.74 ms - 10.79 ms | 10.71 ms | +0.87% | - |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_131072_bits` | 603.41 us | 602.11 us - 604.92 us | 601.78 us | +0.21% | - |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_2097152_bits` | 28.30 ms | 28.20 ms - 28.41 ms | 28.14 ms | +1.15% | - |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_262144_bits` | 1.52 ms | 1.52 ms - 1.53 ms | 1.52 ms | +0.03% | - |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_4194304_bits` | 74.19 ms | 74.02 ms - 74.38 ms | 73.96 ms | +0.74% | - |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_524288_bits` | 4.00 ms | 3.99 ms - 4.02 ms | 3.98 ms | +0.62% | - |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_65536_bits` | 258.97 us | 257.92 us - 260.12 us | 256.73 us | +1.15% | - |
| `rational_algorithm_dispatch_speed/perfect_power_factor_reject` | 70.91 ns | 70.81 ns - 71.05 ns | 70.84 ns | -0.55% | - |
| `rational_algorithm_dispatch_speed/perfect_power_fixed_seventh` | 208.53 ns | 207.88 ns - 209.30 ns | 207.57 ns | +0.06% | - |
| `rational_algorithm_dispatch_speed/perfect_power_general_seventh` | 1.63 us | 1.63 us - 1.63 us | 1.63 us | -4.53% | - |
| `rational_algorithm_dispatch_speed/perfect_power_unfactored_reject` | 3.27 us | 3.26 us - 3.28 us | 3.26 us | -2.75% | - |
| `rational_algorithm_dispatch_speed/radix_format_fraction_decimal` | 2.67 us | 2.67 us - 2.67 us | 2.67 us | -0.60% | - |
| `rational_algorithm_dispatch_speed/radix_format_large_integer` | 3.00 us | 2.99 us - 3.00 us | 2.99 us | +1.10% | - |
| `rational_algorithm_dispatch_speed/radix_format_small_integer` | 948.97 ns | 941.01 ns - 958.28 ns | 934.42 ns | -6.00% | - |
| `rational_algorithm_dispatch_speed/radix_parse_backend_chunked_10240_digits` | 105.25 us | 104.99 us - 105.59 us | 105.00 us | -0.82% | - |
| `rational_algorithm_dispatch_speed/radix_parse_backend_chunked_20480_digits` | 380.91 us | 380.59 us - 381.28 us | 380.78 us | -0.66% | - |
| `rational_algorithm_dispatch_speed/radix_parse_divide_conquer_10240_digits` | 105.77 us | 105.65 us - 105.93 us | 105.71 us | -0.31% | - |
| `rational_algorithm_dispatch_speed/radix_parse_divide_conquer_20480_digits` | 297.51 us | 296.74 us - 298.47 us | 296.26 us | -1.57% | - |
| `rational_algorithm_dispatch_speed/radix_parse_large_integer` | 1.84 us | 1.84 us - 1.85 us | 1.84 us | +0.14% | - |
| `rational_algorithm_dispatch_speed/radix_parse_short_decimal` | 81.78 ns | 81.55 ns - 82.05 ns | 81.42 ns | -0.53% | - |
| `rational_algorithm_dispatch_speed/radix_parse_short_scientific` | 75.44 ns | 75.35 ns - 75.54 ns | 75.46 ns | -2.04% | - |
| `rational_algorithm_dispatch_speed/radix_parse_wide_scientific` | 42.86 us | 42.80 us - 42.97 us | 42.81 us | +0.34% | - |
| `rational_algorithm_dispatch_speed/radix_parse_wide_scientific_expanded` | 39.48 us | 39.46 us - 39.51 us | 39.47 us | +0.77% | - |
| `rational_algorithm_dispatch_speed/reduce_backend_knuth_cold` | 755.92 ns | 752.29 ns - 759.89 ns | 749.09 ns | +2.38% | - |
| `rational_algorithm_dispatch_speed/reduce_backend_large_knuth_cold` | 10.48 us | 10.42 us - 10.55 us | 10.37 us | +2.05% | - |
| `rational_algorithm_dispatch_speed/reduce_backend_single_limb_cold` | 136.07 ns | 135.69 ns - 136.56 ns | 135.59 ns | -2.21% | - |
| `rational_algorithm_dispatch_speed/reduce_fixed_512_coprime_cold` | 2.81 us | 2.79 us - 2.84 us | 2.77 us | +2.92% | - |
| `rational_ops/add_owned` | 10.78 ns | 10.73 ns - 10.84 ns | 10.68 ns | -1.35% | - |
| `rational_ops/add_refs` | 9.33 ns | 9.25 ns - 9.42 ns | 9.06 ns | +0.58% | - |
| `rational_ops/div_owned` | 182.78 ns | 181.90 ns - 183.76 ns | 182.13 ns | -3.52% | - |
| `rational_ops/div_refs` | 163.44 ns | 162.91 ns - 164.03 ns | 162.39 ns | -1.83% | - |
| `rational_ops/mul_owned` | 24.44 ns | 24.38 ns - 24.52 ns | 24.31 ns | -2.57% | - |
| `rational_ops/mul_refs` | 23.06 ns | 22.95 ns - 23.18 ns | 22.83 ns | -1.87% | - |
| `rational_ops/sub_owned` | 10.56 ns | 10.52 ns - 10.60 ns | 10.51 ns | -1.96% | - |
| `rational_ops/sub_refs` | 9.04 ns | 9.01 ns - 9.06 ns | 8.99 ns | -2.46% | - |
| `rational_powi/exact_17` | 79.61 ns | 79.22 ns - 80.03 ns | 78.79 ns | -2.35% | - |
| `rational_powi/oversized_20000_exhausted` | 18.29 ns | 18.25 ns - 18.33 ns | 18.23 ns | +2.12% | - |
| `raw_cache_hit_cost/e` | 58.89 ns | 58.73 ns - 59.10 ns | 58.68 ns | +4.52% | - |
| `raw_cache_hit_cost/one` | 32.26 ns | 32.17 ns - 32.37 ns | 32.15 ns | +5.04% | - |
| `raw_cache_hit_cost/pi` | 57.71 ns | 57.64 ns - 57.82 ns | 57.62 ns | +2.51% | - |
| `raw_cache_hit_cost/tau` | 57.42 ns | 57.37 ns - 57.49 ns | 57.36 ns | -0.16% | - |
| `raw_cache_hit_cost/two` | 32.27 ns | 32.22 ns - 32.34 ns | 32.23 ns | +5.08% | - |
| `raw_cache_hit_cost/zero` | 9.38 ns | 9.36 ns - 9.41 ns | 9.36 ns | +1.73% | - |
| `real_constants/e` | 21.14 ns | 21.11 ns - 21.17 ns | 21.09 ns | +4.94% | - |
| `real_constants/pi` | 16.57 ns | 16.56 ns - 16.59 ns | 16.55 ns | +2.30% | - |
| `real_cotangent/atan_two_direct_construct` | 34.78 ns | 34.67 ns - 34.91 ns | 34.62 ns | +0.07% | - |
| `real_cotangent/atan_two_inverse_tan_construct` | 4.42 us | 4.41 us - 4.43 us | 4.40 us | -5.63% | - |
| `real_cotangent/atan_two_quotient_construct` | 1.41 us | 1.40 us - 1.41 us | 1.40 us | -8.48% | - |
| `real_cotangent/tiny_direct_cold_p256` | 7.79 us | 7.77 us - 7.83 us | 7.76 us | -4.75% | - |
| `real_cotangent/tiny_inverse_tan_cold_p256` | 10.00 us | 9.99 us - 10.02 us | 10.00 us | -2.94% | - |
| `real_cotangent/tiny_quotient_cold_p256` | 7.62 us | 7.61 us - 7.63 us | 7.62 us | -5.43% | - |
| `real_exact_exp_log10/exp_ln_1000` | 63.80 ns | 63.70 ns - 63.92 ns | 63.72 ns | +19.96% | - |
| `real_exact_exp_log10/exp_ln_1_8` | 67.06 ns | 66.89 ns - 67.27 ns | 66.90 ns | +4.28% | - |
| `real_exact_exp_log10/log10_1000` | 33.76 ns | 33.57 ns - 33.99 ns | 33.45 ns | -1.50% | - |
| `real_exact_exp_log10/log10_1_1000` | 62.63 ns | 62.56 ns - 62.70 ns | 62.57 ns | -4.29% | - |
| `real_exact_exp_log10/log10_exp10_6411_4096` | 18.34 ns | 18.32 ns - 18.36 ns | 18.33 ns | -1.17% | - |
| `real_exact_exp_log10/log2_exp2_1_7` | 18.66 ns | 18.61 ns - 18.72 ns | 18.58 ns | -1.02% | - |
| `real_exact_exp_log10/pow10_log10_2` | 55.06 ns | 55.03 ns - 55.09 ns | 55.03 ns | +6.96% | - |
| `real_exact_exp_log10/pow2_log2_3` | 55.12 ns | 55.07 ns - 55.18 ns | 55.05 ns | +7.85% | - |
| `real_exact_inverse_trig/acos_1` | 18.47 ns | 18.37 ns - 18.59 ns | 18.28 ns | +0.16% | - |
| `real_exact_inverse_trig/acos_1_2` | 30.27 ns | 30.10 ns - 30.48 ns | 29.91 ns | +0.67% | - |
| `real_exact_inverse_trig/acos_minus_1` | 22.91 ns | 22.86 ns - 22.98 ns | 22.82 ns | +2.83% | - |
| `real_exact_inverse_trig/asin_1_2` | 29.35 ns | 29.23 ns - 29.51 ns | 29.20 ns | -0.39% | - |
| `real_exact_inverse_trig/asin_minus_1_2` | 41.08 ns | 40.99 ns - 41.17 ns | 40.95 ns | -4.79% | - |
| `real_exact_inverse_trig/asin_sin_pi_5` | 57.38 ns | 57.19 ns - 57.61 ns | 57.14 ns | -9.52% | - |
| `real_exact_inverse_trig/asin_sqrt_2_over_2` | 61.03 ns | 60.96 ns - 61.12 ns | 60.97 ns | -7.38% | - |
| `real_exact_inverse_trig/atan_1` | 23.67 ns | 23.64 ns - 23.71 ns | 23.64 ns | -6.20% | - |
| `real_exact_inverse_trig/atan_sqrt_3_over_3` | 66.29 ns | 65.94 ns - 66.72 ns | 65.62 ns | +1.73% | - |
| `real_exact_inverse_trig/atan_tan_pi_5` | 58.16 ns | 57.93 ns - 58.43 ns | 57.71 ns | +5.75% | - |
| `real_exact_ln/ln_1000` | 51.13 ns | 51.05 ns - 51.23 ns | 51.05 ns | -0.94% | - |
| `real_exact_ln/ln_1024` | 82.28 ns | 81.97 ns - 82.65 ns | 81.73 ns | -2.44% | - |
| `real_exact_ln/ln_1_8` | 77.19 ns | 76.96 ns - 77.50 ns | 76.90 ns | -1.33% | - |
| `real_exact_trig/cos_pi_3` | 429.80 ns | 428.85 ns - 430.81 ns | 428.23 ns | +1.73% | - |
| `real_exact_trig/sin_pi_6` | 517.86 ns | 515.87 ns - 520.42 ns | 514.77 ns | +9.12% | - |
| `real_exact_trig/tan_pi_5` | 234.35 ns | 233.56 ns - 235.18 ns | 233.88 ns | +0.52% | - |
| `real_format/pi_display_alt_32` | 5.37 us | 5.34 us - 5.41 us | 5.31 us | -0.21% | - |
| `real_format/pi_lower_exp_32` | 4.78 us | 4.77 us - 4.79 us | 4.77 us | -1.65% | - |
| `real_format/sqrt_two_display_alt_32` | 4.92 us | 4.91 us - 4.93 us | 4.91 us | -0.90% | - |
| `real_general_inverse_trig/acos_11_10_error` | 21.11 ns | 21.09 ns - 21.14 ns | 21.07 ns | -1.54% | - |
| `real_general_inverse_trig/acos_7_10` | 178.53 ns | 177.54 ns - 179.75 ns | 176.85 ns | -0.99% | - |
| `real_general_inverse_trig/acos_sqrt_2_over_3` | 729.87 ns | 728.27 ns - 731.54 ns | 727.20 ns | +1.63% | - |
| `real_general_inverse_trig/asin_11_10_error` | 23.26 ns | 23.18 ns - 23.36 ns | 23.14 ns | -2.04% | - |
| `real_general_inverse_trig/asin_7_10` | 183.48 ns | 182.09 ns - 185.08 ns | 180.48 ns | +0.62% | - |
| `real_general_inverse_trig/asin_near_minus_one` | 186.59 ns | 186.22 ns - 187.08 ns | 186.01 ns | -0.76% | - |
| `real_general_inverse_trig/asin_near_one` | 181.84 ns | 181.50 ns - 182.22 ns | 181.17 ns | -0.13% | - |
| `real_general_inverse_trig/asin_sqrt_2_over_3` | 325.87 ns | 324.79 ns - 327.17 ns | 324.18 ns | +3.26% | - |
| `real_general_inverse_trig/atan_8` | 372.79 ns | 371.61 ns - 374.14 ns | 370.57 ns | +2.92% | - |
| `real_general_inverse_trig/atan_sqrt_2` | 2.91 us | 2.90 us - 2.92 us | 2.90 us | -0.24% | - |
| `real_general_trig/tan_pi_sqrt_2_over_5` | 1.71 us | 1.70 us - 1.72 us | 1.70 us | -3.23% | - |
| `real_general_trig/tan_sqrt_2` | 411.75 ns | 410.87 ns - 412.83 ns | 410.32 ns | -1.92% | - |
| `real_geometry_polynomial_substrate/atan2_axis` | 30.01 ns | 29.94 ns - 30.10 ns | 29.92 ns | -3.86% | - |
| `real_geometry_polynomial_substrate/atan2_quadrant` | 186.56 ns | 185.67 ns - 187.55 ns | 184.83 ns | +2.66% | - |
| `real_geometry_polynomial_substrate/cos_pi_one_fourth` | 35.79 ns | 35.76 ns - 35.83 ns | 35.78 ns | -5.84% | - |
| `real_geometry_polynomial_substrate/cos_pi_one_seventh` | 245.98 ns | 245.12 ns - 247.11 ns | 244.59 ns | -3.44% | - |
| `real_geometry_polynomial_substrate/cosc_opaque_zero` | 4.43 us | 4.42 us - 4.46 us | 4.40 us | -0.07% | - |
| `real_geometry_polynomial_substrate/cosc_tiny` | 598.44 ns | 597.44 ns - 599.56 ns | 597.53 ns | +2.46% | - |
| `real_geometry_polynomial_substrate/diff_of_products_near_cancel` | 296.07 ns | 295.00 ns - 297.23 ns | 293.80 ns | +1.27% | - |
| `real_geometry_polynomial_substrate/eval_poly_horner` | 1.19 us | 1.18 us - 1.19 us | 1.18 us | -6.86% | - |
| `real_geometry_polynomial_substrate/eval_rational_poly` | 1.54 us | 1.54 us - 1.55 us | 1.53 us | -6.77% | - |
| `real_geometry_polynomial_substrate/hypot2_3_4` | 81.21 ns | 80.92 ns - 81.60 ns | 80.80 ns | +0.21% | - |
| `real_geometry_polynomial_substrate/hypot3_2_3_6` | 138.76 ns | 138.62 ns - 138.91 ns | 138.60 ns | +0.21% | - |
| `real_geometry_polynomial_substrate/hypot_minus_tiny` | 2.01 us | 2.00 us - 2.02 us | 1.99 us | -1.63% | - |
| `real_geometry_polynomial_substrate/mul_add_zero_product` | 35.02 ns | 34.85 ns - 35.22 ns | 34.67 ns | -2.48% | - |
| `real_geometry_polynomial_substrate/sin_pi_one_sixth` | 78.12 ns | 77.52 ns - 78.85 ns | 77.31 ns | +0.41% | - |
| `real_geometry_polynomial_substrate/sinc_opaque_zero` | 4.35 us | 4.31 us - 4.40 us | 4.27 us | -1.57% | - |
| `real_geometry_polynomial_substrate/sinc_pi_half` | 185.29 ns | 184.50 ns - 186.18 ns | 185.48 ns | +2.07% | - |
| `real_geometry_polynomial_substrate/sinc_pi_opaque_zero` | 6.71 us | 6.67 us - 6.76 us | 6.65 us | +1.89% | - |
| `real_geometry_polynomial_substrate/sinc_tiny` | 343.20 ns | 342.09 ns - 344.61 ns | 341.32 ns | +0.96% | - |
| `real_geometry_polynomial_substrate/sinc_zero` | 10.80 ns | 10.79 ns - 10.81 ns | 10.79 ns | -1.83% | - |
| `real_geometry_polynomial_substrate/sum_products_dense` | 1.97 us | 1.96 us - 1.98 us | 1.96 us | -0.75% | - |
| `real_geometry_polynomial_substrate/tan_pi_one_third` | 30.23 ns | 30.17 ns - 30.30 ns | 30.14 ns | -1.08% | - |
| `real_inverse_hyperbolic/acosh_1` | 12.46 ns | 12.45 ns - 12.48 ns | 12.44 ns | -0.26% | - |
| `real_inverse_hyperbolic/acosh_1_000_000` | 158.66 ns | 157.62 ns - 159.80 ns | 157.39 ns | +3.92% | - |
| `real_inverse_hyperbolic/acosh_2` | 41.62 ns | 41.41 ns - 41.87 ns | 41.24 ns | -0.66% | - |
| `real_inverse_hyperbolic/acosh_sqrt_2` | 107.81 ns | 99.55 ns - 124.14 ns | 99.41 ns | +7.65% | - |
| `real_inverse_hyperbolic/asinh_0` | 11.22 ns | 11.17 ns - 11.29 ns | 11.12 ns | +0.93% | - |
| `real_inverse_hyperbolic/asinh_1_000_000` | 194.89 ns | 194.37 ns - 195.48 ns | 193.98 ns | +1.49% | - |
| `real_inverse_hyperbolic/asinh_1_2` | 184.78 ns | 184.06 ns - 185.65 ns | 183.67 ns | +3.72% | - |
| `real_inverse_hyperbolic/asinh_minus_1_2` | 240.41 ns | 238.56 ns - 242.45 ns | 235.84 ns | +4.00% | - |
| `real_inverse_hyperbolic/asinh_sqrt_2` | 136.10 ns | 115.95 ns - 176.23 ns | 115.85 ns | +18.37% | - |
| `real_inverse_hyperbolic/atanh_0` | 11.14 ns | 11.12 ns - 11.16 ns | 11.12 ns | -0.43% | - |
| `real_inverse_hyperbolic/atanh_1_2` | 25.15 ns | 25.11 ns - 25.20 ns | 25.08 ns | -13.29% | - |
| `real_inverse_hyperbolic/atanh_1_error` | 10.38 ns | 10.34 ns - 10.41 ns | 10.46 ns | -1.42% | - |
| `real_inverse_hyperbolic/atanh_9_10` | 209.19 ns | 208.84 ns - 209.58 ns | 208.78 ns | +8.75% | - |
| `real_inverse_hyperbolic/atanh_minus_1_2` | 48.10 ns | 48.03 ns - 48.19 ns | 48.04 ns | -17.92% | - |
| `real_inverse_hyperbolic/atanh_sqrt_half` | 95.62 ns | 85.24 ns - 116.15 ns | 85.16 ns | +9.68% | - |
| `real_irrational_ops/add_owned` | 67.05 ns | 66.31 ns - 67.85 ns | 65.36 ns | +1.95% | - |
| `real_irrational_ops/add_refs` | 57.81 ns | 57.53 ns - 58.20 ns | 57.47 ns | -0.63% | - |
| `real_irrational_ops/div_owned` | 48.20 ns | 48.01 ns - 48.41 ns | 47.81 ns | +3.43% | - |
| `real_irrational_ops/div_refs` | 41.73 ns | 41.69 ns - 41.78 ns | 41.73 ns | +4.42% | - |
| `real_irrational_ops/mul_owned` | 253.08 ns | 252.30 ns - 254.21 ns | 252.43 ns | -4.41% | - |
| `real_irrational_ops/mul_refs` | 223.71 ns | 222.80 ns - 224.75 ns | 221.70 ns | -2.93% | - |
| `real_irrational_ops/sub_owned` | 106.58 ns | 106.01 ns - 107.19 ns | 105.14 ns | -0.85% | - |
| `real_irrational_ops/sub_refs` | 96.00 ns | 95.74 ns - 96.35 ns | 95.65 ns | -4.40% | - |
| `real_normal_scientific_substrate/beta_integer` | 132.71 ns | 132.38 ns - 133.09 ns | 132.19 ns | +1.39% | - |
| `real_normal_scientific_substrate/chi_square_sf` | 1.57 us | 1.57 us - 1.58 us | 1.56 us | -3.08% | - |
| `real_normal_scientific_substrate/dnorm_derivative_4` | 1.67 us | 1.66 us - 1.67 us | 1.66 us | -1.73% | - |
| `real_normal_scientific_substrate/erfc_zero` | 8.66 ns | 8.65 ns - 8.68 ns | 8.64 ns | +0.45% | - |
| `real_normal_scientific_substrate/erfcinv_tail` | 1.64 us | 1.64 us - 1.65 us | 1.64 us | +0.39% | - |
| `real_normal_scientific_substrate/erfcx_tail` | 1.59 us | 1.59 us - 1.60 us | 1.59 us | +0.46% | - |
| `real_normal_scientific_substrate/erfinv_mid` | 1.45 us | 1.44 us - 1.46 us | 1.43 us | -2.77% | - |
| `real_normal_scientific_substrate/gamma_half_integer` | 306.48 ns | 305.65 ns - 307.43 ns | 305.11 ns | +0.14% | - |
| `real_normal_scientific_substrate/gamma_integer` | 102.80 ns | 102.48 ns - 103.21 ns | 102.36 ns | -0.20% | - |
| `real_normal_scientific_substrate/hermite_8` | 1.33 us | 1.33 us - 1.34 us | 1.33 us | +0.11% | - |
| `real_normal_scientific_substrate/lgamma_half_integer` | 1.15 us | 1.15 us - 1.16 us | 1.15 us | -2.05% | - |
| `real_normal_scientific_substrate/ln_beta_half_integer` | 1.76 us | 1.75 us - 1.76 us | 1.75 us | -2.02% | - |
| `real_normal_scientific_substrate/log_dnorm_large` | 144.25 ns | 143.43 ns - 145.22 ns | 142.97 ns | +1.95% | - |
| `real_normal_scientific_substrate/log_normal_sf_tail` | 304.49 ns | 303.35 ns - 305.79 ns | 302.27 ns | +2.03% | - |
| `real_normal_scientific_substrate/log_normal_sf_zero` | 48.55 ns | 48.25 ns - 48.90 ns | 47.91 ns | +1.52% | - |
| `real_normal_scientific_substrate/log_pnorm_tail` | 286.37 ns | 282.45 ns - 293.21 ns | 281.18 ns | +1.55% | - |
| `real_normal_scientific_substrate/log_pnorm_zero` | 48.19 ns | 47.98 ns - 48.47 ns | 47.84 ns | +0.22% | - |
| `real_normal_scientific_substrate/normal_hazard_tail` | 2.93 us | 2.93 us - 2.94 us | 2.93 us | -0.45% | - |
| `real_normal_scientific_substrate/normal_hazard_zero` | 16.18 ns | 16.14 ns - 16.23 ns | 16.12 ns | +0.74% | - |
| `real_normal_scientific_substrate/normal_interval_moment_3` | 2.46 us | 2.45 us - 2.47 us | 2.44 us | -0.41% | - |
| `real_normal_scientific_substrate/normal_interval_narrow` | 759.05 ns | 755.85 ns - 762.60 ns | 752.96 ns | +0.20% | - |
| `real_normal_scientific_substrate/normal_inverse_mills_zero` | 16.02 ns | 15.97 ns - 16.08 ns | 15.94 ns | -0.73% | - |
| `real_normal_scientific_substrate/normal_mills_tail` | 2.86 us | 2.85 us - 2.87 us | 2.85 us | +0.78% | - |
| `real_normal_scientific_substrate/normal_mills_zero` | 15.87 ns | 15.82 ns - 15.93 ns | 15.82 ns | -3.33% | - |
| `real_normal_scientific_substrate/normal_pdf_parametric` | 1.29 us | 1.29 us - 1.30 us | 1.29 us | +1.23% | - |
| `real_normal_scientific_substrate/normal_sf_tail` | 369.96 ns | 368.98 ns - 371.13 ns | 368.70 ns | +8.07% | - |
| `real_normal_scientific_substrate/normal_survival_parametric` | 522.95 ns | 522.48 ns - 523.45 ns | 522.24 ns | -4.94% | - |
| `real_normal_scientific_substrate/pnorm_upper_tail` | 371.50 ns | 370.29 ns - 372.97 ns | 369.44 ns | +9.47% | - |
| `real_normal_scientific_substrate/qnorm_upper_tail` | 1.03 us | 1.02 us - 1.03 us | 1.02 us | +1.98% | - |
| `real_normal_scientific_substrate/regularized_beta_left_unity` | 329.26 ns | 328.64 ns - 329.96 ns | 328.48 ns | +1.26% | - |
| `real_normal_scientific_substrate/regularized_beta_mid` | 1.24 us | 1.23 us - 1.24 us | 1.23 us | +0.54% | - |
| `real_normal_scientific_substrate/regularized_beta_q_left_unity` | 237.16 ns | 236.79 ns - 237.65 ns | 236.74 ns | +2.12% | - |
| `real_normal_scientific_substrate/regularized_beta_q_mid` | 840.73 ns | 836.93 ns - 845.50 ns | 834.60 ns | +0.60% | - |
| `real_normal_scientific_substrate/regularized_beta_q_uniform` | 175.20 ns | 174.78 ns - 175.68 ns | 174.53 ns | +1.23% | - |
| `real_normal_scientific_substrate/regularized_beta_uniform` | 160.41 ns | 159.28 ns - 161.68 ns | 158.45 ns | -0.43% | - |
| `real_normal_scientific_substrate/regularized_gamma_p_half` | 1.85 us | 1.84 us - 1.86 us | 1.84 us | -2.76% | - |
| `real_normal_scientific_substrate/regularized_gamma_q_integer` | 659.09 ns | 657.60 ns - 660.87 ns | 656.51 ns | +2.03% | - |
| `real_normal_scientific_substrate/standard_normal_moment_12` | 144.37 ns | 143.95 ns - 144.85 ns | 143.54 ns | -11.70% | - |
| `real_normal_scientific_substrate/truncated_normal_mean` | 2.52 us | 2.51 us - 2.53 us | 2.50 us | +1.53% | - |
| `real_ops/add_owned` | 20.69 ns | 20.62 ns - 20.76 ns | 20.59 ns | -2.04% | - |
| `real_ops/add_refs` | 17.78 ns | 17.75 ns - 17.81 ns | 17.73 ns | -2.42% | - |
| `real_ops/div_owned` | 193.04 ns | 192.28 ns - 193.97 ns | 192.38 ns | -2.54% | - |
| `real_ops/div_refs` | 183.46 ns | 183.26 ns - 183.67 ns | 183.27 ns | -5.17% | - |
| `real_ops/mul_owned` | 34.19 ns | 33.99 ns - 34.40 ns | 33.75 ns | +0.51% | - |
| `real_ops/mul_refs` | 31.11 ns | 30.89 ns - 31.35 ns | 30.56 ns | -3.72% | - |
| `real_ops/sub_owned` | 23.03 ns | 22.97 ns - 23.12 ns | 22.93 ns | -1.41% | - |
| `real_ops/sub_refs` | 17.94 ns | 17.86 ns - 18.05 ns | 17.77 ns | -3.35% | - |
| `real_powi/exact_17` | 120.07 ns | 119.66 ns - 120.56 ns | 119.42 ns | +2.92% | - |
| `real_powi/exact_17_i64` | 77.99 ns | 77.86 ns - 78.15 ns | 77.78 ns | -2.91% | - |
| `real_powi/irrational_17` | 153.05 ns | 151.71 ns - 154.64 ns | 150.33 ns | +6.43% | - |
| `real_powi/large_exact_lazy_20000` | 48.51 us | 48.33 us - 48.72 us | 48.12 us | +1.22% | - |
| `real_powi/pi_negative_one` | 61.07 ns | 60.65 ns - 61.52 ns | 60.46 ns | +2.37% | - |
| `real_representation_construction_export/hyperreal_exact/const_offset` | 107.15 ns | 106.91 ns - 107.42 ns | 106.91 ns | -0.20% | 1 elements |
| `real_representation_construction_export/hyperreal_exact/const_product` | 1.31 us | 1.31 us - 1.31 us | 1.30 us | -0.17% | 1 elements |
| `real_representation_construction_export/hyperreal_exact/const_product_sqrt` | 2.56 us | 2.54 us - 2.57 us | 2.53 us | +3.39% | 1 elements |
| `real_representation_construction_export/hyperreal_exact/exp` | 189.99 ns | 189.48 ns - 190.67 ns | 189.14 ns | -4.78% | 1 elements |
| `real_representation_construction_export/hyperreal_exact/irrational` | 2.15 us | 2.14 us - 2.16 us | 2.14 us | -1.60% | 1 elements |
| `real_representation_construction_export/hyperreal_exact/ln` | 117.53 ns | 116.97 ns - 118.20 ns | 116.44 ns | -0.37% | 1 elements |
| `real_representation_construction_export/hyperreal_exact/ln_affine` | 155.93 ns | 155.79 ns - 156.09 ns | 155.75 ns | -1.35% | 1 elements |
| `real_representation_construction_export/hyperreal_exact/ln_product` | 773.53 ns | 769.96 ns - 777.63 ns | 765.92 ns | -0.58% | 1 elements |
| `real_representation_construction_export/hyperreal_exact/log10` | 1.39 us | 1.38 us - 1.40 us | 1.38 us | +0.65% | 1 elements |
| `real_representation_construction_export/hyperreal_exact/log2` | 1.24 us | 1.24 us - 1.24 us | 1.23 us | -0.03% | 1 elements |
| `real_representation_construction_export/hyperreal_exact/one` | 62.03 ns | 61.94 ns - 62.13 ns | 62.01 ns | -2.72% | 1 elements |
| `real_representation_construction_export/hyperreal_exact/pi` | 27.74 ns | 27.70 ns - 27.80 ns | 27.69 ns | +6.86% | 1 elements |
| `real_representation_construction_export/hyperreal_exact/pi_exp` | 138.87 ns | 138.64 ns - 139.12 ns | 138.42 ns | -1.20% | 1 elements |
| `real_representation_construction_export/hyperreal_exact/pi_inv` | 56.85 ns | 56.64 ns - 57.09 ns | 56.55 ns | +4.23% | 1 elements |
| `real_representation_construction_export/hyperreal_exact/pi_inv_exp` | 143.18 ns | 142.67 ns - 143.78 ns | 142.49 ns | +1.56% | 1 elements |
| `real_representation_construction_export/hyperreal_exact/pi_pow` | 108.68 ns | 107.98 ns - 109.46 ns | 107.17 ns | +0.13% | 1 elements |
| `real_representation_construction_export/hyperreal_exact/pi_sqrt` | 386.99 ns | 383.20 ns - 391.15 ns | 379.68 ns | -3.70% | 1 elements |
| `real_representation_construction_export/hyperreal_exact/sin_pi` | 3.75 us | 3.74 us - 3.76 us | 3.73 us | +1.26% | 1 elements |
| `real_representation_construction_export/hyperreal_exact/sqrt` | 105.29 ns | 104.93 ns - 105.74 ns | 104.66 ns | -3.54% | 1 elements |
| `real_representation_construction_export/hyperreal_exact/tan_pi` | 12.07 us | 12.03 us - 12.12 us | 11.99 us | +0.19% | 1 elements |
| `real_representation_construction_export/mpfr192/const_offset` | 60.71 ns | 60.43 ns - 61.02 ns | 60.00 ns | -3.23% | 1 elements |
| `real_representation_construction_export/mpfr192/const_product` | 1.42 us | 1.42 us - 1.43 us | 1.41 us | +0.30% | 1 elements |
| `real_representation_construction_export/mpfr192/const_product_sqrt` | 1.55 us | 1.54 us - 1.55 us | 1.54 us | +0.41% | 1 elements |
| `real_representation_construction_export/mpfr192/exp` | 1.41 us | 1.41 us - 1.41 us | 1.40 us | -0.68% | 1 elements |
| `real_representation_construction_export/mpfr192/irrational` | 1.03 us | 1.02 us - 1.03 us | 1.02 us | +1.22% | 1 elements |
| `real_representation_construction_export/mpfr192/ln` | 1.72 us | 1.71 us - 1.72 us | 1.71 us | -1.59% | 1 elements |
| `real_representation_construction_export/mpfr192/ln_affine` | 3.17 us | 3.16 us - 3.18 us | 3.15 us | +0.46% | 1 elements |
| `real_representation_construction_export/mpfr192/ln_product` | 3.58 us | 3.57 us - 3.59 us | 3.56 us | +0.25% | 1 elements |
| `real_representation_construction_export/mpfr192/log10` | 3.72 us | 3.70 us - 3.75 us | 3.70 us | -0.20% | 1 elements |
| `real_representation_construction_export/mpfr192/log2` | 1.86 us | 1.85 us - 1.87 us | 1.84 us | +2.29% | 1 elements |
| `real_representation_construction_export/mpfr192/one` | 30.62 ns | 30.56 ns - 30.69 ns | 30.50 ns | -1.25% | 1 elements |
| `real_representation_construction_export/mpfr192/pi` | 29.36 ns | 29.28 ns - 29.47 ns | 29.20 ns | -6.93% | 1 elements |
| `real_representation_construction_export/mpfr192/pi_exp` | 1.40 us | 1.39 us - 1.41 us | 1.39 us | +0.37% | 1 elements |
| `real_representation_construction_export/mpfr192/pi_inv` | 95.57 ns | 95.26 ns - 95.92 ns | 94.96 ns | -2.28% | 1 elements |
| `real_representation_construction_export/mpfr192/pi_inv_exp` | 1.44 us | 1.43 us - 1.45 us | 1.43 us | -2.01% | 1 elements |
| `real_representation_construction_export/mpfr192/pi_pow` | 55.46 ns | 55.32 ns - 55.64 ns | 55.26 ns | -6.77% | 1 elements |
| `real_representation_construction_export/mpfr192/pi_sqrt` | 150.26 ns | 149.88 ns - 150.69 ns | 149.50 ns | -0.72% | 1 elements |
| `real_representation_construction_export/mpfr192/sin_pi` | 1.23 us | 1.22 us - 1.24 us | 1.22 us | +0.18% | 1 elements |
| `real_representation_construction_export/mpfr192/sqrt` | 116.64 ns | 116.30 ns - 117.03 ns | 115.94 ns | -0.52% | 1 elements |
| `real_representation_construction_export/mpfr192/tan_pi` | 1.48 us | 1.47 us - 1.49 us | 1.47 us | +1.92% | 1 elements |
| `real_representation_prepared/hyperreal_cached_f64/const_offset` | 1.18 ns | 1.18 ns - 1.18 ns | 1.18 ns | -0.14% | 1 elements |
| `real_representation_prepared/hyperreal_cached_f64/const_product` | 1.18 ns | 1.18 ns - 1.19 ns | 1.18 ns | -0.11% | 1 elements |
| `real_representation_prepared/hyperreal_cached_f64/const_product_sqrt` | 1.18 ns | 1.18 ns - 1.18 ns | 1.18 ns | -0.10% | 1 elements |
| `real_representation_prepared/hyperreal_cached_f64/exp` | 1.18 ns | 1.18 ns - 1.18 ns | 1.18 ns | +0.24% | 1 elements |
| `real_representation_prepared/hyperreal_cached_f64/irrational` | 1.20 ns | 1.19 ns - 1.20 ns | 1.19 ns | +1.94% | 1 elements |
| `real_representation_prepared/hyperreal_cached_f64/ln` | 1.18 ns | 1.18 ns - 1.18 ns | 1.18 ns | +0.35% | 1 elements |
| `real_representation_prepared/hyperreal_cached_f64/ln_affine` | 1.18 ns | 1.18 ns - 1.19 ns | 1.18 ns | +0.79% | 1 elements |
| `real_representation_prepared/hyperreal_cached_f64/ln_product` | 1.17 ns | 1.17 ns - 1.17 ns | 1.17 ns | -0.66% | 1 elements |
| `real_representation_prepared/hyperreal_cached_f64/log10` | 1.18 ns | 1.18 ns - 1.19 ns | 1.17 ns | +0.39% | 1 elements |
| `real_representation_prepared/hyperreal_cached_f64/log2` | 1.19 ns | 1.18 ns - 1.20 ns | 1.18 ns | +0.48% | 1 elements |
| `real_representation_prepared/hyperreal_cached_f64/one` | 1.18 ns | 1.18 ns - 1.18 ns | 1.18 ns | -0.55% | 1 elements |
| `real_representation_prepared/hyperreal_cached_f64/pi` | 1.18 ns | 1.17 ns - 1.18 ns | 1.17 ns | -0.25% | 1 elements |
| `real_representation_prepared/hyperreal_cached_f64/pi_exp` | 1.19 ns | 1.18 ns - 1.20 ns | 1.17 ns | +0.81% | 1 elements |
| `real_representation_prepared/hyperreal_cached_f64/pi_inv` | 1.17 ns | 1.17 ns - 1.18 ns | 1.17 ns | -0.48% | 1 elements |
| `real_representation_prepared/hyperreal_cached_f64/pi_inv_exp` | 1.17 ns | 1.17 ns - 1.18 ns | 1.17 ns | -0.05% | 1 elements |
| `real_representation_prepared/hyperreal_cached_f64/pi_pow` | 1.18 ns | 1.18 ns - 1.19 ns | 1.18 ns | -0.11% | 1 elements |
| `real_representation_prepared/hyperreal_cached_f64/pi_sqrt` | 1.18 ns | 1.18 ns - 1.19 ns | 1.17 ns | +0.06% | 1 elements |
| `real_representation_prepared/hyperreal_cached_f64/sin_pi` | 1.18 ns | 1.18 ns - 1.18 ns | 1.18 ns | -1.13% | 1 elements |
| `real_representation_prepared/hyperreal_cached_f64/sqrt` | 1.18 ns | 1.17 ns - 1.18 ns | 1.17 ns | -1.16% | 1 elements |
| `real_representation_prepared/hyperreal_cached_f64/tan_pi` | 1.19 ns | 1.19 ns - 1.21 ns | 1.18 ns | +1.45% | 1 elements |
| `real_representation_prepared/hyperreal_certified_192/const_offset` | 191.40 ns | 191.03 ns - 191.81 ns | 190.85 ns | -0.28% | 1 elements |
| `real_representation_prepared/hyperreal_certified_192/const_product` | 225.82 ns | 225.47 ns - 226.25 ns | 225.29 ns | -2.29% | 1 elements |
| `real_representation_prepared/hyperreal_certified_192/const_product_sqrt` | 189.59 ns | 189.31 ns - 189.91 ns | 189.29 ns | -2.10% | 1 elements |
| `real_representation_prepared/hyperreal_certified_192/exp` | 227.82 ns | 227.14 ns - 228.77 ns | 226.73 ns | -1.83% | 1 elements |
| `real_representation_prepared/hyperreal_certified_192/irrational` | 197.89 ns | 196.25 ns - 199.70 ns | 194.26 ns | +3.37% | 1 elements |
| `real_representation_prepared/hyperreal_certified_192/ln` | 190.97 ns | 190.77 ns - 191.17 ns | 190.83 ns | -2.33% | 1 elements |
| `real_representation_prepared/hyperreal_certified_192/ln_affine` | 189.90 ns | 189.57 ns - 190.29 ns | 189.41 ns | -2.65% | 1 elements |
| `real_representation_prepared/hyperreal_certified_192/ln_product` | 193.24 ns | 193.14 ns - 193.36 ns | 193.13 ns | +1.19% | 1 elements |
| `real_representation_prepared/hyperreal_certified_192/log10` | 228.18 ns | 227.89 ns - 228.50 ns | 227.81 ns | -1.71% | 1 elements |
| `real_representation_prepared/hyperreal_certified_192/log2` | 227.05 ns | 226.41 ns - 227.87 ns | 225.98 ns | -2.85% | 1 elements |
| `real_representation_prepared/hyperreal_certified_192/one` | 13.26 ns | 13.22 ns - 13.30 ns | 13.21 ns | -0.64% | 1 elements |
| `real_representation_prepared/hyperreal_certified_192/pi` | 195.22 ns | 194.55 ns - 196.07 ns | 194.06 ns | -0.74% | 1 elements |
| `real_representation_prepared/hyperreal_certified_192/pi_exp` | 190.67 ns | 190.35 ns - 191.05 ns | 190.08 ns | -1.84% | 1 elements |
| `real_representation_prepared/hyperreal_certified_192/pi_inv` | 230.64 ns | 230.06 ns - 231.30 ns | 229.79 ns | -0.35% | 1 elements |
| `real_representation_prepared/hyperreal_certified_192/pi_inv_exp` | 193.19 ns | 192.84 ns - 193.60 ns | 192.60 ns | +0.42% | 1 elements |
| `real_representation_prepared/hyperreal_certified_192/pi_pow` | 228.63 ns | 228.09 ns - 229.26 ns | 227.67 ns | -0.69% | 1 elements |
| `real_representation_prepared/hyperreal_certified_192/pi_sqrt` | 190.18 ns | 189.83 ns - 190.63 ns | 189.72 ns | -2.55% | 1 elements |
| `real_representation_prepared/hyperreal_certified_192/sin_pi` | 233.21 ns | 232.24 ns - 234.52 ns | 231.85 ns | -0.43% | 1 elements |
| `real_representation_prepared/hyperreal_certified_192/sqrt` | 190.16 ns | 189.91 ns - 190.45 ns | 189.77 ns | -2.68% | 1 elements |
| `real_representation_prepared/hyperreal_certified_192/tan_pi` | 197.23 ns | 196.19 ns - 198.35 ns | 194.74 ns | +1.10% | 1 elements |
| `real_representation_prepared/hyperreal_clone/const_offset` | 28.19 ns | 28.16 ns - 28.24 ns | 28.13 ns | -0.00% | 1 elements |
| `real_representation_prepared/hyperreal_clone/const_product` | 22.49 ns | 22.44 ns - 22.55 ns | 22.45 ns | +3.67% | 1 elements |
| `real_representation_prepared/hyperreal_clone/const_product_sqrt` | 29.42 ns | 29.38 ns - 29.46 ns | 29.41 ns | -0.04% | 1 elements |
| `real_representation_prepared/hyperreal_clone/exp` | 18.97 ns | 18.96 ns - 18.99 ns | 18.95 ns | -0.08% | 1 elements |
| `real_representation_prepared/hyperreal_clone/irrational` | 13.58 ns | 13.54 ns - 13.62 ns | 13.50 ns | +0.46% | 1 elements |
| `real_representation_prepared/hyperreal_clone/ln` | 19.01 ns | 18.99 ns - 19.04 ns | 18.98 ns | -0.03% | 1 elements |
| `real_representation_prepared/hyperreal_clone/ln_affine` | 28.35 ns | 28.33 ns - 28.38 ns | 28.31 ns | -0.53% | 1 elements |
| `real_representation_prepared/hyperreal_clone/ln_product` | 28.31 ns | 28.29 ns - 28.33 ns | 28.30 ns | -1.09% | 1 elements |
| `real_representation_prepared/hyperreal_clone/log10` | 18.92 ns | 18.90 ns - 18.93 ns | 18.91 ns | -0.73% | 1 elements |
| `real_representation_prepared/hyperreal_clone/log2` | 18.91 ns | 18.89 ns - 18.92 ns | 18.90 ns | -0.70% | 1 elements |
| `real_representation_prepared/hyperreal_clone/one` | 9.70 ns | 9.68 ns - 9.72 ns | 9.68 ns | +0.14% | 1 elements |
| `real_representation_prepared/hyperreal_clone/pi` | 13.71 ns | 13.66 ns - 13.77 ns | 13.61 ns | +0.99% | 1 elements |
| `real_representation_prepared/hyperreal_clone/pi_exp` | 18.81 ns | 18.78 ns - 18.85 ns | 18.78 ns | -0.02% | 1 elements |
| `real_representation_prepared/hyperreal_clone/pi_inv` | 13.54 ns | 13.53 ns - 13.57 ns | 13.52 ns | +0.08% | 1 elements |
| `real_representation_prepared/hyperreal_clone/pi_inv_exp` | 18.78 ns | 18.75 ns - 18.81 ns | 18.75 ns | -0.37% | 1 elements |
| `real_representation_prepared/hyperreal_clone/pi_pow` | 13.68 ns | 13.64 ns - 13.72 ns | 13.62 ns | +0.86% | 1 elements |
| `real_representation_prepared/hyperreal_clone/pi_sqrt` | 18.94 ns | 18.92 ns - 18.95 ns | 18.92 ns | -0.27% | 1 elements |
| `real_representation_prepared/hyperreal_clone/sin_pi` | 19.00 ns | 18.98 ns - 19.02 ns | 19.00 ns | -0.03% | 1 elements |
| `real_representation_prepared/hyperreal_clone/sqrt` | 19.09 ns | 19.03 ns - 19.17 ns | 18.99 ns | +0.82% | 1 elements |
| `real_representation_prepared/hyperreal_clone/tan_pi` | 19.18 ns | 19.12 ns - 19.25 ns | 19.08 ns | +1.16% | 1 elements |
| `real_representation_prepared/mpfr192_clone/const_offset` | 17.18 ns | 17.16 ns - 17.20 ns | 17.15 ns | -0.36% | 1 elements |
| `real_representation_prepared/mpfr192_clone/const_product` | 17.24 ns | 17.22 ns - 17.27 ns | 17.19 ns | -0.19% | 1 elements |
| `real_representation_prepared/mpfr192_clone/const_product_sqrt` | 17.22 ns | 17.20 ns - 17.24 ns | 17.19 ns | -0.06% | 1 elements |
| `real_representation_prepared/mpfr192_clone/exp` | 17.23 ns | 17.21 ns - 17.25 ns | 17.20 ns | +0.26% | 1 elements |
| `real_representation_prepared/mpfr192_clone/irrational` | 17.51 ns | 17.40 ns - 17.63 ns | 17.28 ns | +1.14% | 1 elements |
| `real_representation_prepared/mpfr192_clone/ln` | 17.28 ns | 17.22 ns - 17.35 ns | 17.20 ns | +0.41% | 1 elements |
| `real_representation_prepared/mpfr192_clone/ln_affine` | 17.23 ns | 17.20 ns - 17.25 ns | 17.18 ns | -0.19% | 1 elements |
| `real_representation_prepared/mpfr192_clone/ln_product` | 17.23 ns | 17.20 ns - 17.26 ns | 17.18 ns | -0.18% | 1 elements |
| `real_representation_prepared/mpfr192_clone/log10` | 17.22 ns | 17.19 ns - 17.24 ns | 17.19 ns | -0.21% | 1 elements |
| `real_representation_prepared/mpfr192_clone/log2` | 17.18 ns | 17.16 ns - 17.20 ns | 17.17 ns | -0.30% | 1 elements |
| `real_representation_prepared/mpfr192_clone/one` | 17.18 ns | 17.17 ns - 17.21 ns | 17.17 ns | -0.06% | 1 elements |
| `real_representation_prepared/mpfr192_clone/pi` | 17.17 ns | 17.16 ns - 17.18 ns | 17.15 ns | -0.45% | 1 elements |
| `real_representation_prepared/mpfr192_clone/pi_exp` | 17.26 ns | 17.21 ns - 17.31 ns | 17.16 ns | +0.76% | 1 elements |
| `real_representation_prepared/mpfr192_clone/pi_inv` | 17.13 ns | 17.12 ns - 17.14 ns | 17.13 ns | -0.34% | 1 elements |
| `real_representation_prepared/mpfr192_clone/pi_inv_exp` | 17.18 ns | 17.17 ns - 17.20 ns | 17.17 ns | +0.16% | 1 elements |
| `real_representation_prepared/mpfr192_clone/pi_pow` | 17.19 ns | 17.17 ns - 17.21 ns | 17.16 ns | +0.12% | 1 elements |
| `real_representation_prepared/mpfr192_clone/pi_sqrt` | 17.21 ns | 17.17 ns - 17.25 ns | 17.16 ns | -0.08% | 1 elements |
| `real_representation_prepared/mpfr192_clone/sin_pi` | 17.15 ns | 17.12 ns - 17.19 ns | 17.09 ns | -0.94% | 1 elements |
| `real_representation_prepared/mpfr192_clone/sqrt` | 17.26 ns | 17.21 ns - 17.32 ns | 17.18 ns | -0.36% | 1 elements |
| `real_representation_prepared/mpfr192_clone/tan_pi` | 17.44 ns | 17.32 ns - 17.58 ns | 17.21 ns | +0.84% | 1 elements |
| `real_representation_prepared/mpfr192_f64/const_offset` | 8.03 ns | 8.01 ns - 8.05 ns | 8.00 ns | -2.68% | 1 elements |
| `real_representation_prepared/mpfr192_f64/const_product` | 8.05 ns | 8.03 ns - 8.08 ns | 8.01 ns | -6.16% | 1 elements |
| `real_representation_prepared/mpfr192_f64/const_product_sqrt` | 8.00 ns | 8.00 ns - 8.01 ns | 8.00 ns | -1.34% | 1 elements |
| `real_representation_prepared/mpfr192_f64/exp` | 7.34 ns | 7.31 ns - 7.39 ns | 7.30 ns | +0.55% | 1 elements |
| `real_representation_prepared/mpfr192_f64/irrational` | 8.19 ns | 8.15 ns - 8.24 ns | 8.11 ns | -0.64% | 1 elements |
| `real_representation_prepared/mpfr192_f64/ln` | 7.34 ns | 7.32 ns - 7.38 ns | 7.30 ns | +0.49% | 1 elements |
| `real_representation_prepared/mpfr192_f64/ln_affine` | 7.33 ns | 7.31 ns - 7.35 ns | 7.29 ns | +0.20% | 1 elements |
| `real_representation_prepared/mpfr192_f64/ln_product` | 7.34 ns | 7.32 ns - 7.35 ns | 7.32 ns | +0.22% | 1 elements |
| `real_representation_prepared/mpfr192_f64/log10` | 7.31 ns | 7.30 ns - 7.33 ns | 7.29 ns | -0.11% | 1 elements |
| `real_representation_prepared/mpfr192_f64/log2` | 8.03 ns | 8.03 ns - 8.04 ns | 8.02 ns | -0.20% | 1 elements |
| `real_representation_prepared/mpfr192_f64/one` | 8.03 ns | 8.01 ns - 8.06 ns | 8.00 ns | -1.72% | 1 elements |
| `real_representation_prepared/mpfr192_f64/pi` | 8.01 ns | 8.00 ns - 8.03 ns | 7.99 ns | -0.39% | 1 elements |
| `real_representation_prepared/mpfr192_f64/pi_exp` | 7.33 ns | 7.31 ns - 7.34 ns | 7.30 ns | +0.52% | 1 elements |
| `real_representation_prepared/mpfr192_f64/pi_inv` | 7.35 ns | 7.33 ns - 7.37 ns | 7.32 ns | +0.49% | 1 elements |
| `real_representation_prepared/mpfr192_f64/pi_inv_exp` | 8.05 ns | 8.03 ns - 8.07 ns | 8.01 ns | -0.35% | 1 elements |
| `real_representation_prepared/mpfr192_f64/pi_pow` | 8.05 ns | 8.03 ns - 8.07 ns | 8.00 ns | -0.96% | 1 elements |
| `real_representation_prepared/mpfr192_f64/pi_sqrt` | 8.05 ns | 8.03 ns - 8.08 ns | 8.00 ns | -2.28% | 1 elements |
| `real_representation_prepared/mpfr192_f64/sin_pi` | 7.44 ns | 7.41 ns - 7.47 ns | 7.36 ns | +0.49% | 1 elements |
| `real_representation_prepared/mpfr192_f64/sqrt` | 7.36 ns | 7.34 ns - 7.39 ns | 7.32 ns | +0.12% | 1 elements |
| `real_representation_prepared/mpfr192_f64/tan_pi` | 7.43 ns | 7.40 ns - 7.48 ns | 7.36 ns | -0.66% | 1 elements |
| `real_shortcut_adversarial/acos_domain_error` | 22.85 ns | 22.11 ns - 23.70 ns | 22.16 ns | -1.95% | - |
| `real_shortcut_adversarial/acos_exact_half` | 30.81 ns | 29.90 ns - 31.81 ns | 30.10 ns | +4.88% | - |
| `real_shortcut_adversarial/acosh_domain_error` | 9.38 ns | 9.19 ns - 9.61 ns | 9.28 ns | -4.80% | - |
| `real_shortcut_adversarial/asin_domain_error` | 24.94 ns | 24.43 ns - 25.44 ns | 24.90 ns | -0.18% | - |
| `real_shortcut_adversarial/asin_exact_half` | 32.12 ns | 30.72 ns - 33.55 ns | 32.03 ns | +6.49% | - |
| `real_shortcut_adversarial/atan_exact_one` | 25.82 ns | 24.97 ns - 26.88 ns | 25.04 ns | +2.28% | - |
| `real_shortcut_adversarial/atanh_domain_error` | 10.89 ns | 10.87 ns - 10.90 ns | 10.89 ns | -10.20% | - |
| `real_shortcut_adversarial/atanh_endpoint_infinity` | 10.49 ns | 10.30 ns - 10.73 ns | 10.45 ns | -0.48% | - |
| `real_shortcut_adversarial/cos_exact_pi_over_three` | 428.94 ns | 426.47 ns - 431.77 ns | 427.11 ns | -3.22% | - |
| `real_shortcut_adversarial/sin_exact_pi_over_six` | 472.57 ns | 469.04 ns - 478.15 ns | 470.02 ns | -10.67% | - |
| `real_shortcut_adversarial/tan_exact_pi_over_four` | 24.25 ns | 23.94 ns - 24.57 ns | 24.14 ns | -1.24% | - |
| `real_stable_scalar_substrate/cbrt_negative_perfect` | 141.28 ns | 140.92 ns - 141.64 ns | 141.99 ns | -0.63% | - |
| `real_stable_scalar_substrate/certified_compare_nested_radical_identity` | 234.52 ns | 232.33 ns - 237.15 ns | 230.36 ns | -2.25% | - |
| `real_stable_scalar_substrate/expm1_tiny` | 154.08 ns | 153.16 ns - 155.20 ns | 152.50 ns | +1.26% | - |
| `real_stable_scalar_substrate/floor_certified_rational` | 74.93 ns | 74.80 ns - 75.09 ns | 74.67 ns | +0.29% | - |
| `real_stable_scalar_substrate/floor_certified_sqrt2` | 3.05 us | 3.05 us - 3.06 us | 3.04 us | -1.64% | - |
| `real_stable_scalar_substrate/ln_1m_tiny` | 59.88 ns | 59.68 ns - 60.09 ns | 59.69 ns | +4.44% | - |
| `real_stable_scalar_substrate/ln_1p_tiny` | 54.88 ns | 54.70 ns - 55.07 ns | 54.84 ns | +7.84% | - |
| `real_stable_scalar_substrate/logaddexp_dominant` | 2.19 us | 2.19 us - 2.19 us | 2.19 us | -5.63% | - |
| `real_stable_scalar_substrate/logit_near_one` | 393.31 ns | 392.82 ns - 393.88 ns | 393.02 ns | -0.20% | - |
| `real_stable_scalar_substrate/logsubexp_near` | 291.53 ns | 290.56 ns - 292.70 ns | 290.04 ns | -0.99% | - |
| `real_stable_scalar_substrate/near_integer_rational` | 74.65 ns | 74.33 ns - 75.04 ns | 74.26 ns | -1.40% | - |
| `real_stable_scalar_substrate/near_integer_sqrt2` | 229.61 ns | 224.74 ns - 238.66 ns | 225.30 ns | -0.08% | - |
| `real_stable_scalar_substrate/pow_rational_negative_odd_denominator` | 246.44 ns | 240.99 ns - 256.77 ns | 240.75 ns | +4.92% | - |
| `real_stable_scalar_substrate/rem_euclid_certified_rational` | 311.79 ns | 311.30 ns - 312.29 ns | 311.82 ns | -3.00% | - |
| `real_stable_scalar_substrate/root_n_perfect_fourth` | 146.81 ns | 146.44 ns - 147.25 ns | 146.29 ns | +1.19% | - |
| `real_stable_scalar_substrate/sigmoid_large_positive` | 2.09 us | 2.08 us - 2.11 us | 2.07 us | -1.30% | - |
| `real_stable_scalar_substrate/softplus_large_negative` | 1.90 us | 1.89 us - 1.92 us | 1.89 us | -0.22% | - |
| `real_stable_scalar_substrate/softplus_large_positive` | 2.08 us | 2.06 us - 2.10 us | 2.05 us | +0.20% | - |
| `real_stable_scalar_substrate/sqrt1m1_tiny` | 724.02 ns | 722.96 ns - 725.36 ns | 722.65 ns | +0.89% | - |
| `real_stable_scalar_substrate/sqrt1pm1_tiny` | 704.75 ns | 703.74 ns - 706.04 ns | 703.38 ns | +0.79% | - |
| `real_stable_scalar_substrate/sqrt_quadratic_surd_perfect_norm` | 1.44 us | 1.44 us - 1.44 us | 1.44 us | +1.58% | - |
| `simple/eval_constants` | 1.23 us | 1.22 us - 1.24 us | 1.24 us | -0.87% | - |
| `simple/eval_exact` | 273.67 ns | 272.21 ns - 275.02 ns | 275.75 ns | -6.92% | - |
| `simple/eval_nested` | 1.71 us | 1.70 us - 1.72 us | 1.69 us | -5.39% | - |
| `simple/eval_nested_exact` | 899.19 ns | 893.73 ns - 904.45 ns | 910.34 ns | -3.60% | - |
| `simple/parse_nested` | 410.44 ns | 409.34 ns - 411.85 ns | 409.23 ns | -2.46% | - |
| `simple_inverse_error_functions/acos_sqrt_2` | 264.09 ns | 263.16 ns - 265.20 ns | 262.98 ns | -0.08% | - |
| `simple_inverse_error_functions/acosh_0` | 38.96 ns | 38.78 ns - 39.13 ns | 38.80 ns | +4.78% | - |
| `simple_inverse_error_functions/acosh_minus_2` | 39.19 ns | 39.01 ns - 39.38 ns | 39.08 ns | +4.00% | - |
| `simple_inverse_error_functions/asin_11_10` | 55.61 ns | 55.40 ns - 55.82 ns | 55.40 ns | +5.43% | - |
| `simple_inverse_error_functions/atanh_1` | 42.92 ns | 42.73 ns - 43.10 ns | 42.77 ns | +4.12% | - |
| `simple_inverse_error_functions/atanh_sqrt_2` | 159.13 ns | 158.82 ns - 159.42 ns | 158.99 ns | +0.78% | - |
| `simple_inverse_functions/acos_1_2` | 65.94 ns | 65.70 ns - 66.19 ns | 65.82 ns | +3.84% | - |
| `simple_inverse_functions/acos_general` | 215.01 ns | 213.44 ns - 216.82 ns | 211.92 ns | +1.70% | - |
| `simple_inverse_functions/acosh_2` | 63.54 ns | 63.35 ns - 63.72 ns | 63.49 ns | -11.48% | - |
| `simple_inverse_functions/acosh_sqrt_2` | 249.27 ns | 248.05 ns - 250.90 ns | 247.68 ns | -6.14% | - |
| `simple_inverse_functions/asin_1_2` | 66.31 ns | 66.07 ns - 66.54 ns | 66.28 ns | -0.94% | - |
| `simple_inverse_functions/asin_general` | 214.22 ns | 213.18 ns - 215.52 ns | 212.74 ns | -1.17% | - |
| `simple_inverse_functions/asinh_1_2` | 226.85 ns | 226.24 ns - 227.51 ns | 226.14 ns | -3.48% | - |
| `simple_inverse_functions/asinh_sqrt_2` | 297.36 ns | 296.91 ns - 297.81 ns | 296.99 ns | -1.60% | - |
| `simple_inverse_functions/atan_1` | 64.09 ns | 63.85 ns - 64.30 ns | 63.90 ns | +2.09% | - |
| `simple_inverse_functions/atan_general` | 402.07 ns | 400.92 ns - 403.91 ns | 401.19 ns | -0.71% | - |
| `simple_inverse_functions/atanh_1_2` | 58.94 ns | 58.68 ns - 59.22 ns | 58.79 ns | -3.79% | - |
| `simple_inverse_functions/atanh_minus_1_2` | 75.84 ns | 75.56 ns - 76.11 ns | 75.88 ns | -4.59% | - |
| `simple_new_function_surface/error_bundle` | 175.92 ns | 173.07 ns - 178.44 ns | 178.59 ns | -0.35% | - |
| `simple_new_function_surface/geometry_bundle` | 14.19 us | 14.14 us - 14.24 us | 14.11 us | -1.03% | - |
| `simple_new_function_surface/normal_bundle` | 30.82 us | 30.73 us - 30.94 us | 30.70 us | -0.46% | - |
| `simple_new_function_surface/scientific_bundle` | 12.97 us | 12.93 us - 13.02 us | 12.90 us | -0.53% | - |
| `simple_new_function_surface/stable_log_exp_bundle` | 8.79 us | 8.73 us - 8.86 us | 8.71 us | -1.07% | - |
| `structural_query_speed/dense_expr_detailed_facts` | 24.40 ns | 24.32 ns - 24.51 ns | 24.32 ns | -5.05% | - |
| `structural_query_speed/dense_expr_msd_query` | 15.60 ns | 15.54 ns - 15.67 ns | 15.51 ns | -8.35% | - |
| `structural_query_speed/dense_expr_sign_query` | 8.06 ns | 8.04 ns - 8.09 ns | 8.01 ns | +0.53% | - |
| `structural_query_speed/dense_expr_structural_facts` | 10.83 ns | 10.80 ns - 10.86 ns | 10.79 ns | +0.61% | - |
| `structural_query_speed/dense_expr_to_f64_lossy` | 1.18 ns | 1.18 ns - 1.19 ns | 1.18 ns | +0.51% | - |
| `structural_query_speed/dense_expr_zero_status` | 3.54 ns | 3.53 ns - 3.54 ns | 3.53 ns | -2.69% | - |
| `structural_query_speed/e_detailed_facts` | 35.03 ns | 34.88 ns - 35.21 ns | 34.72 ns | +0.90% | - |
| `structural_query_speed/e_msd_query` | 20.35 ns | 20.29 ns - 20.43 ns | 20.25 ns | -6.67% | - |
| `structural_query_speed/e_sign_query` | 17.66 ns | 17.54 ns - 17.80 ns | 17.37 ns | +3.01% | - |
| `structural_query_speed/e_structural_facts` | 17.30 ns | 17.24 ns - 17.38 ns | 17.20 ns | +0.93% | - |
| `structural_query_speed/e_to_f64_lossy` | 1.19 ns | 1.19 ns - 1.20 ns | 1.18 ns | +1.36% | - |
| `structural_query_speed/e_zero_status` | 1.08 ns | 1.08 ns - 1.09 ns | 1.08 ns | +1.21% | - |
| `structural_query_speed/negative_detailed_facts` | 45.69 ns | 45.56 ns - 45.86 ns | 45.47 ns | +0.34% | - |
| `structural_query_speed/negative_msd_query` | 17.27 ns | 17.23 ns - 17.31 ns | 17.22 ns | -6.72% | - |
| `structural_query_speed/negative_sign_query` | 13.32 ns | 13.24 ns - 13.42 ns | 13.19 ns | +1.60% | - |
| `structural_query_speed/negative_structural_facts` | 13.19 ns | 13.17 ns - 13.23 ns | 13.15 ns | -0.74% | - |
| `structural_query_speed/negative_to_f64_lossy` | 1.18 ns | 1.18 ns - 1.19 ns | 1.18 ns | +0.06% | - |
| `structural_query_speed/negative_zero_status` | 1.08 ns | 1.08 ns - 1.08 ns | 1.07 ns | -0.35% | - |
| `structural_query_speed/one_detailed_facts` | 50.58 ns | 50.38 ns - 50.81 ns | 50.22 ns | +1.30% | - |
| `structural_query_speed/one_msd_query` | 16.25 ns | 16.23 ns - 16.29 ns | 16.22 ns | -8.67% | - |
| `structural_query_speed/one_sign_query` | 11.55 ns | 11.53 ns - 11.57 ns | 11.53 ns | +0.26% | - |
| `structural_query_speed/one_structural_facts` | 11.90 ns | 11.82 ns - 12.02 ns | 11.77 ns | +1.48% | - |
| `structural_query_speed/one_to_f64_lossy` | 1.18 ns | 1.18 ns - 1.19 ns | 1.18 ns | +0.97% | - |
| `structural_query_speed/one_zero_status` | 1.08 ns | 1.08 ns - 1.08 ns | 1.07 ns | +0.07% | - |
| `structural_query_speed/pi_detailed_facts` | 34.83 ns | 34.75 ns - 34.93 ns | 34.68 ns | -0.08% | - |
| `structural_query_speed/pi_minus_three_detailed_facts` | 35.06 ns | 34.91 ns - 35.23 ns | 34.69 ns | +0.81% | - |
| `structural_query_speed/pi_minus_three_msd_query` | 20.47 ns | 20.40 ns - 20.56 ns | 20.32 ns | -6.53% | - |
| `structural_query_speed/pi_minus_three_sign_query` | 17.48 ns | 17.41 ns - 17.55 ns | 17.32 ns | -0.22% | - |
| `structural_query_speed/pi_minus_three_structural_facts` | 17.26 ns | 17.24 ns - 17.27 ns | 17.24 ns | -0.24% | - |
| `structural_query_speed/pi_minus_three_to_f64_lossy` | 1.18 ns | 1.18 ns - 1.18 ns | 1.18 ns | +0.20% | - |
| `structural_query_speed/pi_minus_three_zero_status` | 1.08 ns | 1.08 ns - 1.08 ns | 1.07 ns | -2.75% | - |
| `structural_query_speed/pi_msd_query` | 20.40 ns | 20.33 ns - 20.49 ns | 20.28 ns | -6.03% | - |
| `structural_query_speed/pi_sign_query` | 17.41 ns | 17.36 ns - 17.47 ns | 17.30 ns | +0.62% | - |
| `structural_query_speed/pi_structural_facts` | 17.33 ns | 17.29 ns - 17.38 ns | 17.25 ns | +0.73% | - |
| `structural_query_speed/pi_to_f64_lossy` | 1.18 ns | 1.18 ns - 1.19 ns | 1.18 ns | +1.12% | - |
| `structural_query_speed/pi_zero_status` | 1.08 ns | 1.08 ns - 1.08 ns | 1.08 ns | +0.44% | - |
| `structural_query_speed/sqrt_two_detailed_facts` | 34.43 ns | 34.38 ns - 34.49 ns | 34.37 ns | +0.71% | - |
| `structural_query_speed/sqrt_two_msd_query` | 20.36 ns | 20.28 ns - 20.47 ns | 20.28 ns | -6.38% | - |
| `structural_query_speed/sqrt_two_sign_query` | 17.41 ns | 17.35 ns - 17.48 ns | 17.27 ns | +1.33% | - |
| `structural_query_speed/sqrt_two_structural_facts` | 17.60 ns | 17.49 ns - 17.73 ns | 17.29 ns | +2.43% | - |
| `structural_query_speed/sqrt_two_to_f64_lossy` | 1.18 ns | 1.18 ns - 1.19 ns | 1.18 ns | +0.83% | - |
| `structural_query_speed/sqrt_two_zero_status` | 1.09 ns | 1.08 ns - 1.09 ns | 1.08 ns | +1.04% | - |
| `structural_query_speed/structural_negation_match` | 13.08 ns | 13.04 ns - 13.12 ns | 13.00 ns | -6.70% | - |
| `structural_query_speed/structural_negation_miss` | 3.78 ns | 3.78 ns - 3.79 ns | 3.77 ns | -12.78% | - |
| `structural_query_speed/tau_detailed_facts` | 39.10 ns | 38.93 ns - 39.29 ns | 38.73 ns | +1.10% | - |
| `structural_query_speed/tau_msd_query` | 26.09 ns | 26.06 ns - 26.13 ns | 26.06 ns | +0.02% | - |
| `structural_query_speed/tau_sign_query` | 21.99 ns | 21.87 ns - 22.14 ns | 21.78 ns | +0.79% | - |
| `structural_query_speed/tau_structural_facts` | 21.79 ns | 21.73 ns - 21.85 ns | 21.69 ns | +0.79% | - |
| `structural_query_speed/tau_to_f64_lossy` | 1.18 ns | 1.18 ns - 1.19 ns | 1.18 ns | +0.92% | - |
| `structural_query_speed/tau_zero_status` | 1.09 ns | 1.09 ns - 1.09 ns | 1.08 ns | +1.72% | - |
| `structural_query_speed/tiny_exact_detailed_facts` | 51.42 ns | 51.21 ns - 51.66 ns | 51.02 ns | +1.70% | - |
| `structural_query_speed/tiny_exact_msd_query` | 19.73 ns | 19.65 ns - 19.81 ns | 19.63 ns | -6.19% | - |
| `structural_query_speed/tiny_exact_sign_query` | 17.00 ns | 16.97 ns - 17.04 ns | 16.94 ns | +0.23% | - |
| `structural_query_speed/tiny_exact_structural_facts` | 17.17 ns | 17.10 ns - 17.24 ns | 17.00 ns | +1.24% | - |
| `structural_query_speed/tiny_exact_to_f64_lossy` | 1.18 ns | 1.18 ns - 1.18 ns | 1.18 ns | +0.40% | - |
| `structural_query_speed/tiny_exact_zero_status` | 1.08 ns | 1.08 ns - 1.09 ns | 1.08 ns | +0.40% | - |
| `structural_query_speed/zero_detailed_facts` | 23.70 ns | 23.63 ns - 23.79 ns | 23.59 ns | +0.27% | - |
| `structural_query_speed/zero_msd_query` | 13.53 ns | 13.51 ns - 13.56 ns | 13.51 ns | -2.99% | - |
| `structural_query_speed/zero_sign_query` | 4.74 ns | 4.73 ns - 4.76 ns | 4.71 ns | +0.77% | - |
| `structural_query_speed/zero_structural_facts` | 8.04 ns | 8.03 ns - 8.05 ns | 8.03 ns | +1.82% | - |
| `structural_query_speed/zero_to_f64_lossy` | 1.18 ns | 1.18 ns - 1.19 ns | 1.18 ns | +1.11% | - |
| `structural_query_speed/zero_zero_status` | 0.82 ns | 0.81 ns - 0.82 ns | 0.81 ns | +0.48% | - |
| `symbolic_reductions/div_const_product_sqrt_e` | 73.03 ns | 72.48 ns - 73.64 ns | 72.17 ns | -3.75% | - |
| `symbolic_reductions/div_const_products` | 412.48 ns | 399.55 ns - 436.63 ns | 397.49 ns | -7.86% | - |
| `symbolic_reductions/div_e_pi` | 120.16 ns | 119.25 ns - 121.22 ns | 118.47 ns | -10.27% | - |
| `symbolic_reductions/div_exp_exp` | 308.78 ns | 307.23 ns - 310.62 ns | 305.72 ns | -14.05% | - |
| `symbolic_reductions/div_one_pi` | 62.66 ns | 62.39 ns - 62.95 ns | 62.08 ns | -3.40% | - |
| `symbolic_reductions/div_pi_square_e` | 304.98 ns | 303.32 ns - 306.95 ns | 301.32 ns | -19.17% | - |
| `symbolic_reductions/div_rational_exp` | 184.01 ns | 164.62 ns - 221.62 ns | 162.78 ns | -9.97% | - |
| `symbolic_reductions/div_sqrt_two_sqrt_three` | 47.00 ns | 46.94 ns - 47.05 ns | 46.92 ns | -7.44% | - |
| `symbolic_reductions/inverse_const_product_sqrt` | 297.30 ns | 295.14 ns - 299.72 ns | 294.48 ns | -12.79% | - |
| `symbolic_reductions/inverse_pi` | 30.93 ns | 30.78 ns - 31.10 ns | 30.66 ns | -21.39% | - |
| `symbolic_reductions/inverse_sqrt_two` | 16.52 ns | 16.46 ns - 16.60 ns | 16.42 ns | -2.15% | - |
| `symbolic_reductions/ln_scaled_e` | 60.78 ns | 60.45 ns - 61.17 ns | 60.33 ns | -5.23% | - |
| `symbolic_reductions/mul_const_product_sqrt_sqrt` | 156.14 ns | 155.31 ns - 157.04 ns | 154.87 ns | -5.23% | - |
| `symbolic_reductions/mul_pi_e_sqrt_two` | 188.81 ns | 187.06 ns - 190.68 ns | 186.62 ns | -10.33% | - |
| `symbolic_reductions/mul_pi_inverse_pi` | 67.28 ns | 66.98 ns - 67.59 ns | 67.08 ns | -2.04% | - |
| `symbolic_reductions/pi_minus_three_facts` | 17.32 ns | 17.27 ns - 17.37 ns | 17.23 ns | -2.16% | - |
| `symbolic_reductions/sqrt_pi_e_square` | 546.99 ns | 546.21 ns - 547.81 ns | 546.74 ns | -5.06% | - |
| `symbolic_reductions/sqrt_pi_square` | 279.51 ns | 278.29 ns - 281.04 ns | 277.43 ns | -5.31% | - |
| `symbolic_reductions/sqrt_scaled_exp_squarefree` | 857.66 ns | 852.74 ns - 863.89 ns | 847.82 ns | -3.67% | - |
| `symbolic_reductions/sub_pi_three` | 43.30 ns | 43.11 ns - 43.51 ns | 42.95 ns | -6.32% | - |
| `trig_adversarial_approx/cos_1e30_p96` | 2.37 us | 2.35 us - 2.42 us | 2.35 us | -1.79% | - |
| `trig_adversarial_approx/cos_1e6_p96` | 2.45 us | 2.41 us - 2.49 us | 2.42 us | -3.08% | - |
| `trig_adversarial_approx/cos_f64_exact_p96` | 1.76 us | 1.75 us - 1.76 us | 1.76 us | -0.76% | - |
| `trig_adversarial_approx/cos_huge_pi_plus_offset_p96` | 2.06 us | 2.03 us - 2.09 us | 2.04 us | +0.98% | - |
| `trig_adversarial_approx/cos_medium_rational_p96` | 1.56 us | 1.53 us - 1.59 us | 1.54 us | +0.91% | - |
| `trig_adversarial_approx/cos_tiny_rational_p96` | 511.41 ns | 503.08 ns - 522.21 ns | 507.37 ns | +2.80% | - |
| `trig_adversarial_approx/sin_1e30_p96` | 2.28 us | 2.27 us - 2.30 us | 2.28 us | -1.72% | - |
| `trig_adversarial_approx/sin_1e6_p96` | 2.49 us | 2.46 us - 2.54 us | 2.46 us | -1.61% | - |
| `trig_adversarial_approx/sin_f64_exact_p96` | 1.87 us | 1.85 us - 1.92 us | 1.85 us | +0.01% | - |
| `trig_adversarial_approx/sin_huge_pi_plus_offset_p96` | 2.23 us | 2.18 us - 2.28 us | 2.19 us | +0.28% | - |
| `trig_adversarial_approx/sin_medium_rational_p96` | 1.63 us | 1.63 us - 1.64 us | 1.63 us | -7.26% | - |
| `trig_adversarial_approx/sin_tiny_rational_p96` | 466.96 ns | 461.60 ns - 472.42 ns | 466.71 ns | -2.28% | - |
| `trig_adversarial_approx/tan_1e30_p96` | 6.77 us | 6.63 us - 6.93 us | 6.71 us | +2.47% | - |
| `trig_adversarial_approx/tan_1e6_p96` | 7.81 us | 7.78 us - 7.87 us | 7.79 us | -0.02% | - |
| `trig_adversarial_approx/tan_huge_pi_plus_offset_p96` | 6.36 us | 6.31 us - 6.43 us | 6.33 us | -0.37% | - |
| `trig_adversarial_approx/tan_medium_rational_p96` | 6.13 us | 6.09 us - 6.20 us | 6.09 us | +0.64% | - |
| `trig_adversarial_approx/tan_near_half_pi_p96` | 20.64 us | 20.35 us - 20.97 us | 20.41 us | +6.61% | - |
| `trig_adversarial_approx/tan_promoted_generated_604_125_p96` | 6.48 us | 6.45 us - 6.52 us | 6.46 us | -0.86% | - |
| `trig_adversarial_approx/tan_tiny_rational_p96` | 1.76 us | 1.76 us - 1.77 us | 1.76 us | +0.40% | - |
| `trig_fuzz_adversarial_approx/cos_promoted_slow_candidates_p96` | 17.56 us | 17.49 us - 17.63 us | 17.52 us | -3.61% | - |
| `trig_fuzz_adversarial_approx/cos_sweep_768_p96` | 1.62 ms | 1.62 ms - 1.63 ms | 1.62 ms | -0.09% | - |
| `trig_fuzz_adversarial_approx/sin_promoted_slow_candidates_p96` | 17.02 us | 16.88 us - 17.18 us | 16.96 us | -7.67% | - |
| `trig_fuzz_adversarial_approx/sin_sweep_768_p96` | 1.60 ms | 1.59 ms - 1.62 ms | 1.59 ms | -3.44% | - |
| `trig_fuzz_adversarial_approx/tan_promoted_slow_candidates_p96` | 77.76 us | 76.87 us - 79.15 us | 77.04 us | -5.41% | - |
| `trig_fuzz_adversarial_approx/tan_sweep_768_p96` | 4.53 ms | 4.50 ms - 4.55 ms | 4.52 ms | -2.06% | - |

<!-- END COMPLETE BENCHMARK REPORT -->
