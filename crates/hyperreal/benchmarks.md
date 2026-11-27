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
| `construction_speed/rational_one` | 15.60 ns | 15.50 ns - 15.71 ns | Constructs `Rational::one()`. |
| `construction_speed/rational_new_one` | 18.76 ns | 18.73 ns - 18.80 ns | Constructs one through `Rational::new(1)`. |
| `construction_speed/rational_from_u8_four` | 19.74 ns | 19.55 ns - 19.93 ns | Constructs positive four through unsigned primitive conversion. |
| `construction_speed/rational_from_i8_minus_four` | 21.30 ns | 21.11 ns - 21.48 ns | Constructs negative four through signed primitive conversion. |
| `construction_speed/computable_one` | not run | not run | Constructs `Computable::one()`. |
| `construction_speed/real_new_rational_one` | 29.03 ns | 28.76 ns - 29.29 ns | Constructs one through `Real::new(Rational::one())`. |
| `construction_speed/real_one` | not run | not run | Constructs one through `Real::one()`. |
| `construction_speed/real_from_i32_one` | not run | not run | Constructs one through integer conversion. |
| `construction_speed/real_from_u8_four` | not run | not run | Constructs positive four as an exact `Real` from `u8`. |
| `construction_speed/real_from_i8_minus_four` | not run | not run | Constructs negative four as an exact `Real` from `i8`. |

### `raw_cache_hit_cost`

Cost of cold and cached `Computable::approx` calls for simple values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `raw_cache_hit_cost/zero` | not run | not run | Cached approximation request for exact zero. |
| `raw_cache_hit_cost/one` | not run | not run | Cached approximation request for exact one. |
| `raw_cache_hit_cost/two` | not run | not run | Cached approximation request for exact two. |
| `raw_cache_hit_cost/e` | not run | not run | Cached approximation request for Euler's constant. |
| `raw_cache_hit_cost/pi` | not run | not run | Cached approximation request for pi. |
| `raw_cache_hit_cost/tau` | not run | not run | Cached approximation request for two pi. |

### `structural_query_speed`

Speed of public structural queries across exact, transcendental, and composite `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `structural_query_speed/zero_zero_status` | not run | not run | Checks zero/nonzero facts for exact zero. |
| `structural_query_speed/zero_sign_query` | not run | not run | Reads sign facts for exact zero. |
| `structural_query_speed/zero_msd_query` | not run | not run | Reads magnitude facts for exact zero. |
| `structural_query_speed/zero_structural_facts` | not run | not run | Computes full structural facts for exact zero. |
| `structural_query_speed/one_zero_status` | not run | not run | Checks zero/nonzero facts for exact one. |
| `structural_query_speed/one_sign_query` | not run | not run | Reads sign facts for exact one. |
| `structural_query_speed/one_msd_query` | not run | not run | Reads magnitude facts for exact one. |
| `structural_query_speed/one_structural_facts` | not run | not run | Computes full structural facts for exact one. |
| `structural_query_speed/negative_zero_status` | not run | not run | Checks zero/nonzero facts for an exact negative integer. |
| `structural_query_speed/negative_sign_query` | not run | not run | Reads sign facts for an exact negative integer. |
| `structural_query_speed/negative_msd_query` | not run | not run | Reads magnitude facts for an exact negative integer. |
| `structural_query_speed/negative_structural_facts` | not run | not run | Computes full structural facts for an exact negative integer. |
| `structural_query_speed/tiny_exact_zero_status` | not run | not run | Checks zero/nonzero facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_sign_query` | not run | not run | Reads sign facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_msd_query` | not run | not run | Reads magnitude facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_structural_facts` | not run | not run | Computes full structural facts for a tiny exact rational. |
| `structural_query_speed/pi_zero_status` | not run | not run | Checks zero/nonzero facts for pi. |
| `structural_query_speed/pi_sign_query` | not run | not run | Reads sign facts for pi. |
| `structural_query_speed/pi_msd_query` | not run | not run | Reads magnitude facts for pi. |
| `structural_query_speed/pi_structural_facts` | not run | not run | Computes full structural facts for pi. |
| `structural_query_speed/e_zero_status` | not run | not run | Checks zero/nonzero facts for e. |
| `structural_query_speed/e_sign_query` | not run | not run | Reads sign facts for e. |
| `structural_query_speed/e_msd_query` | not run | not run | Reads magnitude facts for e. |
| `structural_query_speed/e_structural_facts` | not run | not run | Computes full structural facts for e. |
| `structural_query_speed/tau_zero_status` | not run | not run | Checks zero/nonzero facts for tau. |
| `structural_query_speed/tau_sign_query` | not run | not run | Reads sign facts for tau. |
| `structural_query_speed/tau_msd_query` | not run | not run | Reads magnitude facts for tau. |
| `structural_query_speed/tau_structural_facts` | not run | not run | Computes full structural facts for tau. |
| `structural_query_speed/sqrt_two_zero_status` | not run | not run | Checks zero/nonzero facts for sqrt(2). |
| `structural_query_speed/sqrt_two_sign_query` | not run | not run | Reads sign facts for sqrt(2). |
| `structural_query_speed/sqrt_two_msd_query` | not run | not run | Reads magnitude facts for sqrt(2). |
| `structural_query_speed/sqrt_two_structural_facts` | not run | not run | Computes full structural facts for sqrt(2). |
| `structural_query_speed/pi_minus_three_zero_status` | not run | not run | Checks zero/nonzero facts for pi - 3. |
| `structural_query_speed/pi_minus_three_sign_query` | not run | not run | Reads sign facts for pi - 3. |
| `structural_query_speed/pi_minus_three_msd_query` | not run | not run | Reads magnitude facts for pi - 3. |
| `structural_query_speed/pi_minus_three_structural_facts` | not run | not run | Computes full structural facts for pi - 3. |
| `structural_query_speed/dense_expr_zero_status` | not run | not run | Checks zero/nonzero facts for a dense composite expression. |
| `structural_query_speed/dense_expr_sign_query` | not run | not run | Reads sign facts for a dense composite expression. |
| `structural_query_speed/dense_expr_msd_query` | not run | not run | Reads magnitude facts for a dense composite expression. |
| `structural_query_speed/dense_expr_structural_facts` | not run | not run | Computes full structural facts for a dense composite expression. |
| `structural_query_speed/structural_negation_match` | not run | not run | Certifies opposite rational scales over one shared exact symbolic basis. |
| `structural_query_speed/structural_negation_miss` | not run | not run | Rejects an unrelated exact symbolic basis without constructing a negation. |

### `pure_scalar_algorithm_speed`

Core scalar algorithms that do not require high-precision transcendental approximation.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `pure_scalar_algorithm_speed/rational_add` | 25.22 ns | 25.20 ns - 25.27 ns | Adds two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_sub` | 26.05 ns | 25.85 ns - 26.26 ns | Subtracts two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_add_wide_dyadic_cold` | 214.78 ns | 211.91 ns - 217.62 ns | Adds fresh integer and wide-dyadic operands without retained work. |
| `pure_scalar_algorithm_speed/rational_sub_wide_dyadic_cold` | 209.02 ns | 205.17 ns - 213.03 ns | Subtracts fresh integer and wide-dyadic operands without retained work. |
| `pure_scalar_algorithm_speed/rational_add_shared_cold` | 252.45 ns | 249.21 ns - 255.76 ns | Adds fresh operands whose storage is cloned but whose arithmetic pair is not yet observed. |
| `pure_scalar_algorithm_speed/rational_sub_shared_cold` | 249.11 ns | 246.32 ns - 252.15 ns | Subtracts fresh operands whose storage is cloned but whose arithmetic pair is not yet observed. |
| `pure_scalar_algorithm_speed/rational_scaled_difference_composed_cold` | 563.91 ns | 557.86 ns - 570.00 ns | Computes a fresh wide-integer scaled difference through multiply then subtract. |
| `pure_scalar_algorithm_speed/rational_scaled_difference_fused_cold` | 203.78 ns | 201.13 ns - 206.41 ns | Computes the same fresh wide-integer scaled difference with the fused integer kernel. |
| `pure_scalar_algorithm_speed/rational_cross_difference_unit_divisor_composed_cold` | 1.232 us | 1.220 us - 1.244 us | Computes a fresh wide-integer cross difference and divides it by negative one through general operations. |
| `pure_scalar_algorithm_speed/rational_cross_difference_unit_divisor_fused_cold` | 440.27 ns | 431.91 ns - 450.34 ns | Computes the same cross difference through the checked fused unit-divisor path. |
| `pure_scalar_algorithm_speed/rational_mul` | 45.87 ns | 45.09 ns - 46.93 ns | Multiplies two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_mul_retained_general` | 31.97 ns | 31.77 ns - 32.16 ns | Reuses one retained exact product for an immutable rational operand pair. |
| `pure_scalar_algorithm_speed/rational_mul_wide_dyadic_cold` | 462.40 ns | 457.51 ns - 467.30 ns | Multiplies fresh wide-denominator dyadics whose numerators fit `u128`. |
| `pure_scalar_algorithm_speed/rational_mul_dyadic_general_cross_cancel` | 30.25 ns | 29.97 ns - 30.53 ns | Multiplies a wide dyadic rational by a general rational with a power-of-two numerator. |
| `pure_scalar_algorithm_speed/rational_div` | 464.21 ns | 459.70 ns - 468.60 ns | Divides two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_inverse_owned_cold` | 47.88 ns | 47.36 ns - 48.41 ns | Inverts a fresh uniquely owned nontrivial rational. |
| `pure_scalar_algorithm_speed/rational_inverse_retained` | 32.85 ns | 32.52 ns - 33.18 ns | Reuses the retained reciprocal of a shared nontrivial rational. |
| `pure_scalar_algorithm_speed/rational_neg_owned_cold` | 26.89 ns | 26.63 ns - 27.17 ns | Negates a fresh uniquely owned nontrivial rational in place. |
| `pure_scalar_algorithm_speed/rational_neg_retained` | 24.37 ns | 24.12 ns - 24.61 ns | Reuses the retained opposite sign of a shared nontrivial rational. |
| `pure_scalar_algorithm_speed/real_exact_powi_i64_owned_cold` | not run | not run | Raises a fresh uniquely owned exact rational Real to the fifth power. |
| `pure_scalar_algorithm_speed/real_exact_powi_i64_retained` | not run | not run | Reuses the bounded exact product chain for a shared fifth power. |
| `pure_scalar_algorithm_speed/real_exact_add` | not run | not run | Adds exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_average_pair` | not run | not run | Averages exact rational-backed `Real` values through the fused pair kernel. |
| `pure_scalar_algorithm_speed/real_exact_average_pair_expanded` | not run | not run | Averages exact rational-backed `Real` values through separate add and divide operations. |
| `pure_scalar_algorithm_speed/real_exact_sub` | not run | not run | Subtracts exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_mul` | not run | not run | Multiplies exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_mul_retained` | not run | not run | Reuses the retained exact product beneath rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_div` | not run | not run | Divides exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_sqrt_owned_cold` | not run | not run | Reduces a fresh uniquely owned exact square-root expression. |
| `pure_scalar_algorithm_speed/real_exact_sqrt_reduce` | not run | not run | Reuses the retained reduction of an exact square-root expression. |
| `pure_scalar_algorithm_speed/real_exact_dyadic_sqrt_reduce` | not run | not run | Reuses the square-root reduction of a large exact dyadic rational. |
| `pure_scalar_algorithm_speed/real_exact_general_sqrt_reduce` | not run | not run | Reuses the square-root reduction of a non-dyadic rational sum of squares. |
| `pure_scalar_algorithm_speed/real_exact_dyadic_radical_scale` | not run | not run | Scales an exact reciprocal radical by one exact binary64-derived dyadic coordinate. |
| `pure_scalar_algorithm_speed/real_exact_ln_reduce` | not run | not run | Reduces an exact logarithm of a power of two. |
| `pure_scalar_algorithm_speed/real_pow_small_integer_exponent` | not run | not run | Dispatches `Real::pow` with an exact small-integer exponent. |

### `rational_algorithm_dispatch_speed`

Cold backend algorithm families and retained rational fact dispatch selected from GMP-style operand shapes.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `rational_algorithm_dispatch_speed/dyadic_fact_cold` | 92.36 ns | 90.13 ns - 94.67 ns | Classifies a fresh non-dyadic denominator and retains the result. |
| `rational_algorithm_dispatch_speed/dyadic_fact_retained` | 3.32 ns | 3.32 ns - 3.33 ns | Reads an already-retained non-dyadic denominator classification. |
| `rational_algorithm_dispatch_speed/compare_leading_significand_retained_1024_bits` | 80.44 ns | 79.66 ns - 81.20 ns | Compares retained wide rational magnitudes through the certified leading-significand interval. |
| `rational_algorithm_dispatch_speed/compare_dyadic_shifted_retained_1024_bits` | 21.97 ns | 21.74 ns - 22.19 ns | Compares retained wide dyadics with equal scaled width and a five-bit denominator-shift difference. |
| `rational_algorithm_dispatch_speed/equality_leading_significand_retained_1024_bits` | 272.95 ns | 270.33 ns - 275.55 ns | Compares unequal retained wide fractions with separated leading significands. |
| `rational_algorithm_dispatch_speed/equality_dyadic_shifted_retained_1024_bits` | 20.95 ns | 20.77 ns - 21.14 ns | Rejects unequal retained wide dyadics without cross products. |
| `rational_algorithm_dispatch_speed/equality_shared_identity_retained` | 4.11 ns | 4.07 ns - 4.15 ns | Compares two references to one retained rational allocation. |
| `rational_algorithm_dispatch_speed/equality_same_denominator_retained` | 23.81 ns | 23.59 ns - 24.04 ns | Rejects unequal small numerators with a common denominator. |
| `rational_algorithm_dispatch_speed/equality_word_retained` | 22.49 ns | 22.27 ns - 22.73 ns | Rejects unequal small fractions with different non-dyadic denominators. |
| `rational_algorithm_dispatch_speed/equality_equal_distinct_retained` | 10.01 ns | 9.91 ns - 10.11 ns | Recognizes equal small fractions held in distinct allocations. |
| `rational_algorithm_dispatch_speed/equality_different_signs_retained` | 4.47 ns | 4.43 ns - 4.51 ns | Rejects wide fractions with opposite signs before magnitude comparison. |
| `rational_algorithm_dispatch_speed/equality_close_retained_1024_bits` | 293.40 ns | 290.51 ns - 296.29 ns | Rejects near-equal wide fractions through the exact cross-product fallback. |
| `rational_algorithm_dispatch_speed/mul_backend_basecase_cold` | 578.62 ns | 545.86 ns - 638.84 ns | Multiplies fresh balanced 16-limb integers through the backend basecase kernel. |
| `rational_algorithm_dispatch_speed/mul_backend_half_karatsuba_cold` | 1.110 us | 1.098 us - 1.121 us | Multiplies fresh unbalanced 33-by-66-limb integers through half-Karatsuba. |
| `rational_algorithm_dispatch_speed/mul_backend_karatsuba_cold` | 1.910 us | 1.899 us - 1.921 us | Multiplies fresh balanced 40-limb integers through Karatsuba. |
| `rational_algorithm_dispatch_speed/mul_backend_toom3_cold` | 20.134 us | 19.999 us - 20.262 us | Multiplies fresh balanced 257-limb integers through Toom-3. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_4096_bits` | 22.210 us | 22.089 us - 22.352 us | Runs Hyperreal's seven-product Rust-native Toom-4 candidate on balanced 4,096-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_4096_bits` | 5.522 us | 5.468 us - 5.574 us | Runs the native backend product on the same 4,096-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_16384_bits` | 96.279 us | 96.189 us - 96.438 us | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 16,384-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_16384_bits` | 50.352 us | 50.289 us - 50.465 us | Runs the native backend product on the same 16,384-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_65536_bits` | 599.672 us | 593.900 us - 605.650 us | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 65,536-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_65536_bits` | 436.036 us | 435.581 us - 436.683 us | Runs the native backend product on the same 65,536-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_262144_bits` | 4.055 ms | 4.020 ms - 4.093 ms | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 262,144-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_262144_bits` | 3.767 ms | 3.728 ms - 3.807 ms | Runs the native backend product on the same 262,144-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_524288_bits` | 10.514 ms | 10.458 ms - 10.579 ms | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 524,288-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_524288_bits` | 10.045 ms | 9.982 ms - 10.117 ms | Runs the native backend product on the same 524,288-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_1048576_bits` | 28.276 ms | 28.225 ms - 28.357 ms | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 1,048,576-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_1048576_bits` | 27.793 ms | 27.589 ms - 28.025 ms | Runs the native backend product on the same 1,048,576-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_2097152_bits` | 76.619 ms | 76.531 ms - 76.729 ms | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 2,097,152-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_2097152_bits` | 78.125 ms | 77.751 ms - 78.565 ms | Runs the native backend product on the same 2,097,152-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_1048576_bits` | 25.139 ms | 24.953 ms - 25.351 ms | Runs the retained production Toom-8 selector above its balanced crossover. |
| `rational_algorithm_dispatch_speed/mul_selected_2097152_bits` | 66.250 ms | 65.894 ms - 66.656 ms | Runs the retained production Toom-8 selector on balanced 2,097,152-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_1048576_bits` | 26.200 ms | 26.148 ms - 26.277 ms | Runs Hyperreal's eleven-product Rust-native Toom-6 candidate above its crossover. |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_131072_bits` | 1.513 ms | 1.512 ms - 1.513 ms | Runs Hyperreal's Rust-native Toom-6 candidate on balanced 131,072-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_131072_bits` | 1.304 ms | 1.291 ms - 1.317 ms | Runs the retained native backend selector on the same 131,072-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_262144_bits` | 3.907 ms | 3.872 ms - 3.944 ms | Runs Hyperreal's Rust-native Toom-6 candidate on balanced 262,144-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_524288_bits` | 10.140 ms | 10.066 ms - 10.220 ms | Runs Hyperreal's Rust-native Toom-6 candidate on balanced 524,288-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_524288_bits` | 9.921 ms | 9.835 ms - 10.014 ms | Runs the retained production Toom-8 selector above its balanced crossover. |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_2097152_bits` | 69.057 ms | 69.014 ms - 69.122 ms | Runs Hyperreal's Rust-native Toom-6 candidate on balanced 2,097,152-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_toom4_unbalanced_1258291_by_1048576` | 34.443 ms | 34.283 ms - 34.631 ms | Runs retained Toom-4 on a 6:5 operand pair outside Toom-6's balance band. |
| `rational_algorithm_dispatch_speed/mul_backend_unbalanced_1258291_by_1048576` | 36.233 ms | 35.869 ms - 36.588 ms | Runs the native backend on the same 6:5 operand pair. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_262144_bits` | 3.835 ms | 3.805 ms - 3.868 ms | Runs Hyperreal's fifteen-product Rust-native Toom-8 candidate on balanced 262,144-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_262144_bits` | 3.838 ms | 3.804 ms - 3.874 ms | Runs the retained production Toom-8 selector at its balanced crossover. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_65536_bits` | 670.766 us | 665.297 us - 676.610 us | Runs Hyperreal's Rust-native Toom-8 candidate on balanced 65,536-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_131072_bits` | 1.608 ms | 1.593 ms - 1.624 ms | Runs Hyperreal's Rust-native Toom-8 candidate on balanced 131,072-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_524288_bits` | 9.829 ms | 9.754 ms - 9.911 ms | Runs Hyperreal's Rust-native Toom-8 candidate at the Toom-6 crossover. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_1048576_bits` | 24.767 ms | 24.715 ms - 24.844 ms | Runs Hyperreal's Rust-native Toom-8 candidate on balanced 1,048,576-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_2097152_bits` | 66.241 ms | 65.870 ms - 66.668 ms | Runs Hyperreal's Rust-native Toom-8 candidate on balanced 2,097,152-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_4194304_bits` | 174.148 ms | 174.019 ms - 174.293 ms | Runs Hyperreal's Rust-native Toom-8 candidate on balanced 4,194,304-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_4194304_bits` | 174.073 ms | 173.708 ms - 174.576 ms | Runs the retained production Toom-8 selector on the same 4,194,304-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_toom6_unbalanced_599186_by_524288` | 11.532 ms | 11.491 ms - 11.584 ms | Runs retained Toom-6 on an 8:7 operand pair outside Toom-8's balance band. |
| `rational_algorithm_dispatch_speed/mul_backend_unbalanced_599186_by_524288` | 11.690 ms | 11.688 ms - 11.692 ms | Runs the native backend on the same 8:7 operand pair. |
| `rational_algorithm_dispatch_speed/mul_ntt_candidate_262144_bits` | 58.087 ms | 58.073 ms - 58.101 ms | Runs Hyperreal's exact two-prime Rust-native NTT/CRT candidate on balanced 262,144-bit operands. |
| `rational_algorithm_dispatch_speed/mul_ntt_candidate_1048576_bits` | 263.719 ms | 263.579 ms - 263.878 ms | Runs the Rust-native NTT/CRT candidate on balanced 1,048,576-bit operands. |
| `rational_algorithm_dispatch_speed/mul_ntt_candidate_4194304_bits` | 1.177 s | 1.177 s - 1.178 s | Runs the Rust-native NTT/CRT candidate on balanced 4,194,304-bit operands. |
| `rational_algorithm_dispatch_speed/reduce_backend_single_limb_cold` | 380.48 ns | 376.45 ns - 385.31 ns | Reduces a fresh wide fraction by a single-limb exact divisor. |
| `rational_algorithm_dispatch_speed/reduce_backend_knuth_cold` | 1.780 us | 1.762 us - 1.800 us | Reduces a fresh wide fraction through normalized Knuth basecase division. |
| `rational_algorithm_dispatch_speed/reduce_backend_large_knuth_cold` | 24.014 us | 23.984 us - 24.062 us | Reduces a fresh 129-limb numerator by a 65-limb exact divisor through normalized Knuth division. |
| `rational_algorithm_dispatch_speed/reduce_fixed_512_coprime_cold` | 7.735 us | 7.656 us - 7.822 us | Reduces fresh balanced 512-bit operands through the fixed-limb rational-operation GCD. |
| `rational_algorithm_dispatch_speed/exact_remainder_large_knuth` | 11.376 us | 11.267 us - 11.486 us | Computes a wide rational fractional remainder through the traced normalized Knuth backend. |
| `rational_algorithm_dispatch_speed/division_trivial_small_quotient` | 156.72 ns | 155.08 ns - 158.42 ns | Exercises the backend's zero-quotient magnitude division exit on wide operands. |
| `rational_algorithm_dispatch_speed/gcd_selected_128_bits` | 412.94 ns | 412.78 ns - 413.11 ns | Runs selected magnitude GCD on an ascending balanced two-limb pair. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_128_bits` | 12.848 us | 12.728 us - 12.984 us | Runs the full-width Euclidean baseline on the same 128-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_192_bits` | 13.348 us | 13.345 us - 13.352 us | Runs selected magnitude GCD at the retained three-limb Lehmer crossover. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_192_bits` | 20.338 us | 20.177 us - 20.511 us | Runs the full-width Euclidean baseline on the same 192-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_512_bits` | 31.252 us | 30.908 us - 31.600 us | Runs selected magnitude GCD above the Lehmer crossover. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_512_bits` | 75.196 us | 74.560 us - 75.880 us | Runs the full-width Euclidean baseline on the same 512-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_1024_bits` | 59.822 us | 59.231 us - 60.422 us | Runs selected magnitude GCD above the Lehmer crossover. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_1024_bits` | 171.714 us | 171.461 us - 172.183 us | Runs the full-width Euclidean baseline on the same 1,024-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_4096_bits` | 296.364 us | 294.009 us - 298.893 us | Runs selected magnitude GCD well above the Lehmer crossover. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_4096_bits` | 1.117 ms | 1.111 ms - 1.125 ms | Runs the full-width Euclidean baseline on the same 4,096-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_192_bits` | 19.591 us | 19.546 us - 19.666 us | Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 192 bits. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_192_bits` | 19.645 us | 19.485 us - 19.822 us | Runs the full-width Euclidean baseline on the same initially unbalanced pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_256_bits` | 20.204 us | 20.104 us - 20.321 us | Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 256 bits. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_256_bits` | 32.803 us | 32.500 us - 33.118 us | Runs the full-width Euclidean baseline on the same initially unbalanced pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_512_bits` | 33.316 us | 32.989 us - 33.638 us | Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 512 bits. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_512_bits` | 73.508 us | 72.866 us - 74.189 us | Runs the full-width Euclidean baseline on the same initially unbalanced pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_1024_bits` | 65.148 us | 64.518 us - 65.800 us | Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 1,024 bits. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_1024_bits` | 180.953 us | 178.972 us - 182.998 us | Runs the full-width Euclidean baseline on the same initially unbalanced pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_4096_bits` | 317.854 us | 316.057 us - 319.894 us | Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 4,096 bits. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_4096_bits` | 1.163 ms | 1.157 ms - 1.171 ms | Runs the full-width Euclidean baseline on the same initially unbalanced pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_8192_bits` | 715.039 us | 712.717 us - 718.043 us | Runs the recursive half-GCD candidate below its provisional crossover. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_8192_bits` | 730.530 us | 724.258 us - 737.166 us | Runs the quadratic Lehmer baseline on the same 8,192-bit pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_16384_bits` | 6.779 ms | 6.768 ms - 6.797 ms | Runs the recursive half-GCD candidate at its provisional crossover. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_16384_bits` | 2.094 ms | 2.075 ms - 2.114 ms | Runs the quadratic Lehmer baseline on the same 16,384-bit pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_65536_bits` | 23.347 ms | 23.208 ms - 23.507 ms | Runs the recursive half-GCD candidate well above its provisional crossover. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_65536_bits` | 20.432 ms | 20.266 ms - 20.607 ms | Runs the quadratic Lehmer baseline on the same 65,536-bit pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_262144_bits` | 626.308 ms | 626.004 ms - 626.630 ms | Runs recursive half-GCD with selected higher-Toom matrix products at 262,144 bits. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_262144_bits` | 289.488 ms | 289.305 ms - 289.689 ms | Runs the Lehmer baseline on the same 262,144-bit pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_1048576_bits` | 9.186 s | 9.185 s - 9.187 s | Runs recursive half-GCD with selected higher-Toom matrix products at 1,048,576 bits. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_1048576_bits` | 4.623 s | 4.622 s - 4.624 s | Runs the Lehmer baseline on the same 1,048,576-bit pair. |
| `rational_algorithm_dispatch_speed/barrett_one_shot_8192_by_1024` | 11.528 us | 11.432 us - 11.629 us | Prepares a Rust-native Barrett reciprocal and divides one 8,192-bit value by a 1,024-bit divisor. |
| `rational_algorithm_dispatch_speed/backend_one_shot_8192_by_1024` | 8.402 us | 8.325 us - 8.480 us | Runs the native backend div-rem baseline for the same one-shot operands. |
| `rational_algorithm_dispatch_speed/barrett_batch16_8192_by_1024` | 168.122 us | 166.535 us - 169.766 us | Amortizes one Rust-native Barrett reciprocal over sixteen 8,192-bit dividends. |
| `rational_algorithm_dispatch_speed/backend_batch16_8192_by_1024` | 137.276 us | 135.835 us - 138.728 us | Runs sixteen native backend div-rem operations on the same values. |
| `rational_algorithm_dispatch_speed/barrett_batch16_65536_by_4096` | 2.915 ms | 2.893 ms - 2.943 ms | Amortizes one Rust-native Barrett reciprocal over sixteen 65,536-bit dividends. |
| `rational_algorithm_dispatch_speed/backend_batch16_65536_by_4096` | 2.782 ms | 2.767 ms - 2.799 ms | Runs sixteen native backend div-rem operations on the same large values. |
| `rational_algorithm_dispatch_speed/perfect_power_factor_reject` | 176.51 ns | 174.85 ns - 178.09 ns | Rejects 12 after small-factor multiplicities collapse to gcd one. |
| `rational_algorithm_dispatch_speed/perfect_power_general_seventh` | 4.076 us | 4.038 us - 4.116 us | Discovers an exact rational seventh power whose base primes exceed the trial table. |
| `rational_algorithm_dispatch_speed/perfect_power_fixed_seventh` | 579.70 ns | 566.77 ns - 597.04 ns | Checks the same value when the seventh-root degree is already known. |
| `rational_algorithm_dispatch_speed/perfect_power_unfactored_reject` | 8.799 us | 8.708 us - 8.888 us | Rejects mismatched seventh- and fifth-power rational components beyond the trial table. |
| `rational_algorithm_dispatch_speed/radix_format_small_integer` | 4.504 us | 4.442 us - 4.572 us | Formats a 16-limb integer using repeated single-limb radix division. |
| `rational_algorithm_dispatch_speed/radix_format_large_integer` | 16.441 us | 16.285 us - 16.593 us | Formats a 32-limb integer using divide-and-conquer radix conversion. |
| `rational_algorithm_dispatch_speed/radix_parse_short_decimal` | 198.19 ns | 196.12 ns - 200.28 ns | Parses a short exact decimal through the checked word-sized path. |
| `rational_algorithm_dispatch_speed/radix_parse_short_scientific` | 174.78 ns | 173.22 ns - 176.38 ns | Parses a representative file-I/O scientific literal through the checked word-sized path. |
| `rational_algorithm_dispatch_speed/radix_parse_wide_scientific` | 79.975 us | 79.235 us - 80.751 us | Parses a 5,120-digit significand with a negative decimal exponent exactly. |
| `rational_algorithm_dispatch_speed/radix_parse_wide_scientific_expanded` | 96.329 us | 96.300 us - 96.363 us | Parses the same exact wide value after expanding its decimal point as a baseline. |
| `rational_algorithm_dispatch_speed/radix_parse_large_integer` | 3.313 us | 3.277 us - 3.370 us | Parses a large below-threshold decimal fixture through chunked multiply-add conversion. |
| `rational_algorithm_dispatch_speed/radix_parse_divide_conquer_10240_digits` | 194.914 us | 194.073 us - 195.926 us | Parses 10,240 digits through the divide-and-conquer product tree. |
| `rational_algorithm_dispatch_speed/radix_parse_backend_chunked_10240_digits` | 190.185 us | 188.546 us - 191.927 us | Parses the same 10,240 digits with the backend chunked multiply-add baseline. |
| `rational_algorithm_dispatch_speed/radix_parse_divide_conquer_20480_digits` | 607.112 us | 601.608 us - 612.571 us | Parses 20,480 digits through the divide-and-conquer product tree. |
| `rational_algorithm_dispatch_speed/radix_parse_backend_chunked_20480_digits` | 737.190 us | 731.278 us - 742.896 us | Parses the same 20,480 digits with the backend chunked multiply-add baseline. |
| `rational_algorithm_dispatch_speed/radix_format_fraction_decimal` | 6.394 us | 6.309 us - 6.496 us | Formats a rational decimal through exact repeated digit division. |

### `borrowed_op_overhead`

Borrowed versus owned operation overhead for rational and real operands.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `borrowed_op_overhead/rational_clone_pair` | 34.90 ns | 34.65 ns - 35.13 ns | Clones two rational values. |
| `borrowed_op_overhead/rational_add_refs` | 26.62 ns | 26.35 ns - 26.88 ns | Adds rational references. |
| `borrowed_op_overhead/rational_add_owned` | 33.69 ns | 33.34 ns - 34.03 ns | Adds owned rational values. |
| `borrowed_op_overhead/real_clone_pair` | not run | not run | Clones two scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_unscaled_add_refs` | not run | not run | Adds borrowed unscaled transcendental `Real` values. |
| `borrowed_op_overhead/real_unscaled_add_owned` | not run | not run | Adds owned unscaled transcendental `Real` values. |
| `borrowed_op_overhead/real_add_refs` | not run | not run | Adds borrowed scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_add_owned` | not run | not run | Adds owned scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_dot2_refs_dense_symbolic` | not run | not run | Computes a borrowed two-lane symbolic dot product with no rational shortcut terms. |
| `borrowed_op_overhead/real_active_dot2_refs_dense_symbolic` | not run | not run | Computes a borrowed two-lane symbolic dot product after the caller has already classified every lane active. |
| `borrowed_op_overhead/real_dot2_refs_mixed_structural` | not run | not run | Computes a borrowed two-lane symbolic dot product with an exact zero lane and a rational scale lane. |
| `borrowed_op_overhead/real_dot3_refs_dense_symbolic` | not run | not run | Computes a borrowed three-lane symbolic dot product with no rational shortcut terms. |
| `borrowed_op_overhead/real_active_dot3_refs_dense_symbolic` | not run | not run | Computes a borrowed three-lane symbolic dot product after the caller has already classified every lane active. |
| `borrowed_op_overhead/real_dot3_refs_mixed_structural` | not run | not run | Computes a borrowed three-lane symbolic dot product with exact zero and rational scale terms. |
| `borrowed_op_overhead/real_dot4_refs_dense_symbolic` | not run | not run | Computes a borrowed four-lane symbolic dot product with no rational shortcut terms. |
| `borrowed_op_overhead/real_active_dot4_refs_dense_symbolic` | not run | not run | Computes a borrowed four-lane symbolic dot product after the caller has already classified every lane active. |
| `borrowed_op_overhead/real_dot4_refs_mixed_structural` | not run | not run | Computes a borrowed four-lane symbolic dot product with exact zero and rational scale terms. |

### `dense_algebra`

Small dense algebra kernels that stress repeated exact and symbolic operations.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `dense_algebra/rational_dot_64` | 3.379 us | 3.376 us - 3.383 us | Computes a 64-element rational dot product. |
| `dense_algebra/rational_matmul_8` | 152.242 us | 150.724 us - 153.784 us | Computes an 8x8 rational matrix multiply. |
| `dense_algebra/real_dot_36` | not run | not run | Computes a 36-element dot product over symbolic `Real` values. |
| `dense_algebra/real_matmul_6` | not run | not run | Computes a 6x6 matrix multiply over symbolic `Real` values. |
| `dense_algebra/real_sum_refs_64_symbolic` | not run | not run | Constructs an arbitrary-length sum of 64 borrowed symbolic square roots. |
| `dense_algebra/real_sum_refs_64_symbolic_to_f64` | not run | not run | Constructs and approximates the same arbitrary-length symbolic sum. |
| `dense_algebra/real_sum_refs_1024_symbolic_sequential` | not run | not run | Constructs a 1,024-term symbolic sum with the former sequential fold. |
| `dense_algebra/real_sum_refs_1024_symbolic` | not run | not run | Constructs the same 1,024-term symbolic sum through the balanced public aggregate. |
| `dense_algebra/real_sum_refs_1024_symbolic_to_f64_sequential` | not run | not run | Sequentially constructs and approximates a 1,024-term symbolic sum with cold child caches. |
| `dense_algebra/real_sum_refs_1024_symbolic_to_f64` | not run | not run | Constructs and approximates the same cold 1,024-term symbolic sum through the balanced public aggregate. |
| `dense_algebra/real_sum_refs_1024_rational_sequential` | 42.537 us | 42.135 us - 42.950 us | Constructs a 1,024-term exact rational sum with the former sequential fold. |
| `dense_algebra/real_sum_refs_1024_rational` | 76.703 us | 75.943 us - 77.474 us | Constructs the same exact rational sum through the homogeneous-prefix public aggregate. |
| `dense_algebra/real_sum_owned_1024_symbolic_former_clone_path` | not run | not run | Consumes a 1,024-term symbolic sum while reproducing the former extra per-value clone. |
| `dense_algebra/real_sum_owned_1024_symbolic` | not run | not run | Consumes the same symbolic sum by moving owned values into the balanced reducer. |

### `exact_transcendental_special_forms`

Construction-time shortcuts for exact rational multiples of pi and inverse compositions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `exact_transcendental_special_forms/sin_pi_7` | not run | not run | Builds the exact special form for sin(pi/7). |
| `exact_transcendental_special_forms/cos_pi_7` | not run | not run | Builds the exact special form for cos(pi/7). |
| `exact_transcendental_special_forms/tan_pi_7` | not run | not run | Builds the exact special form for tan(pi/7). |
| `exact_transcendental_special_forms/asin_sin_6pi_7` | not run | not run | Recognizes the principal branch of asin(sin(6pi/7)). |
| `exact_transcendental_special_forms/acos_cos_9pi_7` | not run | not run | Recognizes the principal branch of acos(cos(9pi/7)). |
| `exact_transcendental_special_forms/atan_tan_6pi_7` | not run | not run | Recognizes the principal branch of atan(tan(6pi/7)). |
| `exact_transcendental_special_forms/asinh_large` | not run | not run | Builds a large inverse hyperbolic sine without exact intermediate Reals. |
| `exact_transcendental_special_forms/atanh_sqrt_half` | not run | not run | Builds atanh(sqrt(2)/2) after exact structural domain checks. |
| `exact_transcendental_special_forms/atanh_sqrt_two_error` | not run | not run | Rejects atanh(sqrt(2)) through exact structural domain checks. |
| `exact_transcendental_special_forms/sinh_ln_two` | not run | not run | Folds sinh(ln(2)) to the exact rational 3/4 via the integer-log-collapse shortcut. |
| `exact_transcendental_special_forms/cosh_ln_two` | not run | not run | Folds cosh(ln(2)) to the exact rational 5/4 via the integer-log-collapse shortcut. |
| `exact_transcendental_special_forms/tanh_ln_two` | not run | not run | Folds tanh(ln(2)) to the exact rational 3/5 via the integer-log-collapse shortcut. |
| `exact_transcendental_special_forms/sinh_rational_one` | 1.031 us | 1.017 us - 1.047 us | Builds sinh(1) through the generic (exp(x) - exp(-x))/2 identity path. |
| `exact_transcendental_special_forms/cosh_rational_one` | 925.32 ns | 913.36 ns - 937.48 ns | Builds cosh(1) through the generic (exp(x) + exp(-x))/2 identity path. |
| `exact_transcendental_special_forms/tanh_rational_one` | 1.556 us | 1.536 us - 1.578 us | Builds tanh(1) through the generic (exp(x) - exp(-x))/(exp(x) + exp(-x)) identity path. |
| `exact_transcendental_special_forms/atan2_origin` | not run | not run | Hits the origin (0, 0) short-circuit returning exact zero. |
| `exact_transcendental_special_forms/atan2_axis_positive_y` | not run | not run | Hits the positive-y axis short-circuit returning exact pi/2. |
| `exact_transcendental_special_forms/atan2_axis_negative_x` | not run | not run | Hits the negative-x axis short-circuit returning exact pi. |
| `exact_transcendental_special_forms/atan2_quadrant_one_unit_diagonal` | not run | not run | Quadrant I unit diagonal reduces to atan(1) = pi/4 exact special form. |
| `exact_transcendental_special_forms/atan2_quadrant_two_pi_correction` | not run | not run | Quadrant II (1, -2) exercises atan(small ratio) + pi correction. |
| `exact_transcendental_special_forms/atan2_quadrant_three_negative_pi` | not run | not run | Quadrant III (-1, -2) exercises atan(small ratio) - pi correction. |
| `exact_transcendental_special_forms/log2_power_of_two` | not run | not run | Folds log2(1024) to the exact rational 10 via the integer-log-detection shortcut. |
| `exact_transcendental_special_forms/log2_rational_three` | 326.56 ns | 323.08 ns - 329.98 ns | Builds log2(3) as a lightweight Log2 symbolic certificate. |
| `exact_transcendental_special_forms/log2_ln_quotient_fold` | not run | not run | Folds ln(5) / ln(2) into a Log2 certificate via the divide-recognize shortcut. |

### `real_cotangent`

Public cotangent construction and cold refinement against the two compositional alternatives.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_cotangent/atan_two_direct_construct` | not run | not run | Folds cot(atan(2)) through the public exact inverse-trig rewrite. |
| `real_cotangent/atan_two_quotient_construct` | not run | not run | Builds cot(atan(2)) as the control expression cos(x)/sin(x). |
| `real_cotangent/atan_two_inverse_tan_construct` | not run | not run | Builds cot(atan(2)) as the incomplete control expression 1/tan(x). |
| `real_cotangent/tiny_direct_cold_p256` | not run | not run | Constructs and certifies public cot(2^-10) at 256 fractional bits. |
| `real_cotangent/tiny_quotient_cold_p256` | not run | not run | Constructs and certifies cos(2^-10)/sin(2^-10) at 256 fractional bits. |
| `real_cotangent/tiny_inverse_tan_cold_p256` | not run | not run | Constructs and certifies 1/tan(2^-10) at 256 fractional bits. |

### `symbolic_reductions`

Existing symbolic constant algebra cases considered for additional reductions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `symbolic_reductions/sqrt_pi_square` | not run | not run | Reduces sqrt(pi^2). |
| `symbolic_reductions/sqrt_pi_e_square` | not run | not run | Reduces sqrt((pi * e)^2). |
| `symbolic_reductions/ln_scaled_e` | not run | not run | Reduces ln(2 * e). |
| `symbolic_reductions/sub_pi_three` | not run | not run | Builds the certified pi - 3 constant-offset form. |
| `symbolic_reductions/pi_minus_three_facts` | not run | not run | Reads structural facts for the cached pi - 3 offset form. |
| `symbolic_reductions/div_exp_exp` | not run | not run | Reduces e^3 / e. |
| `symbolic_reductions/div_pi_square_e` | not run | not run | Reduces pi^2 / e. |
| `symbolic_reductions/div_const_products` | not run | not run | Reduces (pi^3 * e^5) / (pi * e^2). |
| `symbolic_reductions/inverse_pi` | not run | not run | Builds the reciprocal of pi. |
| `symbolic_reductions/div_one_pi` | not run | not run | Reduces 1 / pi. |
| `symbolic_reductions/div_rational_exp` | 414.05 ns | 407.98 ns - 420.30 ns | Reduces 2 / e. |
| `symbolic_reductions/div_e_pi` | not run | not run | Reduces e / pi. |
| `symbolic_reductions/mul_pi_inverse_pi` | not run | not run | Multiplies pi by its reciprocal. |
| `symbolic_reductions/mul_pi_e_sqrt_two` | not run | not run | Builds the factored pi * e * sqrt(2) form. |
| `symbolic_reductions/mul_const_product_sqrt_sqrt` | not run | not run | Cancels sqrt(2) from (pi * e * sqrt(2)) * sqrt(2). |
| `symbolic_reductions/div_const_product_sqrt_e` | not run | not run | Reduces (pi * e * sqrt(2)) / e. |
| `symbolic_reductions/inverse_const_product_sqrt` | not run | not run | Builds a rationalized reciprocal of pi * e * sqrt(2). |
| `symbolic_reductions/inverse_sqrt_two` | not run | not run | Builds the rationalized reciprocal of unit-scaled sqrt(2). |
| `symbolic_reductions/div_sqrt_two_sqrt_three` | not run | not run | Rationalizes a quotient of two unit-scaled square roots. |

### `exact_product_sums`

Fixed product-sum reducers used by determinant and cofactor kernels.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `exact_product_sums/signed_product_sum_lcm_6x2` | not run | not run | Computes an exact rational six-term signed product sum with mixed denominators. |
| `exact_product_sums/signed_product_sum_common_scale_6x2` | not run | not run | Computes an exact rational six-term signed product sum through the carried common-scale reducer. |
| `exact_product_sums/signed_product_sum_sparse_single_6x2` | not run | not run | Computes a sparse exact rational six-term signed product sum with one active product. |
| `exact_product_sums/real_signed_product_sum_rational_det3` | 833.73 ns | 825.79 ns - 841.93 ns | Computes a 3x3 determinant-shaped signed product sum through the public `Real` builder. |
| `exact_product_sums/real_signed_product_sum_mixed_symbolic_det3` | not run | not run | Computes the same determinant-shaped builder with symbolic factors and rational scales. |
| `exact_product_sums/exact_rational_sparse_homogeneous_plane_intersection3` | 519.20 ns | 514.93 ns - 523.29 ns | Computes a canonical exact three-plane cofactor tuple from one sparse dyadic row. |

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
| `bounded_parameter_encodings` | custom timing | `serde` | `cargo bench --bench bounded_parameter_encodings --features serde` | [bounded_parameter_encodings_benchmarks.md](bounded_parameter_encodings_benchmarks.md) |
| `dispatch_trace` | diagnostic | `dispatch-trace` | `cargo bench --bench dispatch_trace --features dispatch-trace` | [dispatch_trace.md](dispatch_trace.md) |
| `elliptic_integrals` | custom timing | `default` | `cargo bench --bench elliptic_integrals` | [elliptic_integrals_benchmarks.md](elliptic_integrals_benchmarks.md) |
| `float_convert` | Criterion timing | `default` | `cargo bench --bench float_convert` | this file |
| `gmp_api` | Criterion timing | `default` | `cargo bench --bench gmp_api` | this file |
| `inverse_legendre` | custom timing | `default` | `cargo bench --bench inverse_legendre` | [inverse_legendre_benchmarks.md](inverse_legendre_benchmarks.md) |
| `jacobi` | custom timing | `default` | `cargo bench --bench jacobi` | [jacobi_benchmarks.md](jacobi_benchmarks.md) |
| `legendre_integrals` | custom timing | `default` | `cargo bench --bench legendre_integrals` | [legendre_integrals_benchmarks.md](legendre_integrals_benchmarks.md) |
| `library_perf` | Criterion timing | `simple` | `cargo bench --bench library_perf --features simple` | this file |
| `numerical_micro` | Criterion timing | `default` | `cargo bench --bench numerical_micro` | this file |
| `real_representations` | Criterion timing | `default` | `cargo bench --bench real_representations` | this file |
| `scalar_micro` | Criterion timing | `default` | `cargo bench --bench scalar_micro` | this file |

### Comparative results

Rows sharing a Criterion group and input are compared when they expose distinct implementations. Ratios are elapsed time relative to the fastest stored row; they do not imply identical guarantees or output semantics.

No paired Criterion results are currently stored.

### All Criterion results

| Benchmark | Mean | 95% CI | Median | Change vs baseline | Throughput |
| --- | ---: | ---: | ---: | ---: | ---: |
| `borrowed_op_overhead/rational_add_owned` | 33.69 ns | 33.34 ns - 34.03 ns | 34.82 ns | +1.40% | - |
| `borrowed_op_overhead/rational_add_refs` | 26.62 ns | 26.35 ns - 26.88 ns | 26.17 ns | +5.13% | - |
| `borrowed_op_overhead/rational_clone_pair` | 34.90 ns | 34.65 ns - 35.13 ns | 35.42 ns | +9.10% | - |
| `construction_speed/rational_from_i8_minus_four` | 21.30 ns | 21.11 ns - 21.48 ns | 21.83 ns | +5.17% | - |
| `construction_speed/rational_from_u8_four` | 19.74 ns | 19.55 ns - 19.93 ns | 19.07 ns | +1.86% | - |
| `construction_speed/rational_new_one` | 18.76 ns | 18.73 ns - 18.80 ns | 18.73 ns | +0.08% | - |
| `construction_speed/rational_one` | 15.60 ns | 15.50 ns - 15.71 ns | 15.75 ns | +1.56% | - |
| `construction_speed/real_new_rational_one` | 29.03 ns | 28.76 ns - 29.29 ns | 30.03 ns | +0.30% | - |
| `dense_algebra/rational_dot_64` | 3.38 us | 3.38 us - 3.38 us | 3.38 us | +0.95% | - |
| `dense_algebra/rational_matmul_8` | 152.24 us | 150.72 us - 153.78 us | 145.92 us | -0.18% | - |
| `dense_algebra/real_sum_refs_1024_rational` | 76.70 us | 75.94 us - 77.47 us | 73.37 us | +6.49% | - |
| `dense_algebra/real_sum_refs_1024_rational_sequential` | 42.54 us | 42.13 us - 42.95 us | 40.98 us | +3.84% | - |
| `exact_product_sums/exact_rational_sparse_homogeneous_plane_intersection3` | 519.20 ns | 514.93 ns - 523.29 ns | 527.81 ns | +0.17% | - |
| `exact_product_sums/real_signed_product_sum_rational_det3` | 833.73 ns | 825.79 ns - 841.93 ns | 802.38 ns | +3.64% | - |
| `exact_transcendental_special_forms/cosh_rational_one` | 925.32 ns | 913.36 ns - 937.48 ns | 935.74 ns | +1.87% | - |
| `exact_transcendental_special_forms/log2_rational_three` | 326.56 ns | 323.08 ns - 329.98 ns | 326.47 ns | +9.23% | - |
| `exact_transcendental_special_forms/sinh_rational_one` | 1.03 us | 1.02 us - 1.05 us | 981.30 ns | -0.37% | - |
| `exact_transcendental_special_forms/tanh_rational_one` | 1.56 us | 1.54 us - 1.58 us | 1.50 us | +11.00% | - |
| `pure_scalar_algorithm_speed/rational_add` | 25.22 ns | 25.20 ns - 25.27 ns | 25.19 ns | -0.08% | - |
| `pure_scalar_algorithm_speed/rational_add_shared_cold` | 252.45 ns | 249.21 ns - 255.76 ns | 247.02 ns | -2.54% | - |
| `pure_scalar_algorithm_speed/rational_add_wide_dyadic_cold` | 214.78 ns | 211.91 ns - 217.62 ns | 218.30 ns | -7.90% | - |
| `pure_scalar_algorithm_speed/rational_cross_difference_unit_divisor_composed_cold` | 1.23 us | 1.22 us - 1.24 us | 1.27 us | +1.69% | - |
| `pure_scalar_algorithm_speed/rational_cross_difference_unit_divisor_fused_cold` | 440.27 ns | 431.91 ns - 450.34 ns | 423.25 ns | +5.04% | - |
| `pure_scalar_algorithm_speed/rational_div` | 464.21 ns | 459.70 ns - 468.60 ns | 480.56 ns | +0.12% | - |
| `pure_scalar_algorithm_speed/rational_inverse_owned_cold` | 47.88 ns | 47.36 ns - 48.41 ns | 45.76 ns | -3.85% | - |
| `pure_scalar_algorithm_speed/rational_inverse_retained` | 32.85 ns | 32.52 ns - 33.18 ns | 34.30 ns | -1.26% | - |
| `pure_scalar_algorithm_speed/rational_mul` | 45.87 ns | 45.09 ns - 46.93 ns | 43.74 ns | +2.97% | - |
| `pure_scalar_algorithm_speed/rational_mul_dyadic_general_cross_cancel` | 30.25 ns | 29.97 ns - 30.53 ns | 31.27 ns | +1.08% | - |
| `pure_scalar_algorithm_speed/rational_mul_retained_general` | 31.97 ns | 31.77 ns - 32.16 ns | 32.19 ns | +3.00% | - |
| `pure_scalar_algorithm_speed/rational_mul_wide_dyadic_cold` | 462.40 ns | 457.51 ns - 467.30 ns | 480.41 ns | -2.28% | - |
| `pure_scalar_algorithm_speed/rational_neg_owned_cold` | 26.89 ns | 26.63 ns - 27.17 ns | 26.20 ns | +2.82% | - |
| `pure_scalar_algorithm_speed/rational_neg_retained` | 24.37 ns | 24.12 ns - 24.61 ns | 25.37 ns | +6.25% | - |
| `pure_scalar_algorithm_speed/rational_scaled_difference_composed_cold` | 563.91 ns | 557.86 ns - 570.00 ns | 587.12 ns | -0.61% | - |
| `pure_scalar_algorithm_speed/rational_scaled_difference_fused_cold` | 203.78 ns | 201.13 ns - 206.41 ns | 208.94 ns | +1.95% | - |
| `pure_scalar_algorithm_speed/rational_sub` | 26.05 ns | 25.85 ns - 26.26 ns | 25.53 ns | +2.02% | - |
| `pure_scalar_algorithm_speed/rational_sub_shared_cold` | 249.11 ns | 246.32 ns - 252.15 ns | 246.90 ns | -2.13% | - |
| `pure_scalar_algorithm_speed/rational_sub_wide_dyadic_cold` | 209.02 ns | 205.17 ns - 213.03 ns | 196.60 ns | -13.30% | - |
| `rational_algorithm_dispatch_speed/backend_batch16_65536_by_4096` | 2.78 ms | 2.77 ms - 2.80 ms | 2.76 ms | +0.84% | - |
| `rational_algorithm_dispatch_speed/backend_batch16_8192_by_1024` | 137.28 us | 135.83 us - 138.73 us | 143.07 us | +4.58% | - |
| `rational_algorithm_dispatch_speed/backend_one_shot_8192_by_1024` | 8.40 us | 8.32 us - 8.48 us | 8.52 us | +3.36% | - |
| `rational_algorithm_dispatch_speed/barrett_batch16_65536_by_4096` | 2.92 ms | 2.89 ms - 2.94 ms | 2.87 ms | +1.30% | - |
| `rational_algorithm_dispatch_speed/barrett_batch16_8192_by_1024` | 168.12 us | 166.53 us - 169.77 us | 162.04 us | +4.59% | - |
| `rational_algorithm_dispatch_speed/barrett_one_shot_8192_by_1024` | 11.53 us | 11.43 us - 11.63 us | 11.24 us | -3.52% | - |
| `rational_algorithm_dispatch_speed/compare_dyadic_shifted_retained_1024_bits` | 21.97 ns | 21.74 ns - 22.19 ns | 20.99 ns | -6.38% | - |
| `rational_algorithm_dispatch_speed/compare_leading_significand_retained_1024_bits` | 80.44 ns | 79.66 ns - 81.20 ns | 82.94 ns | +3.53% | - |
| `rational_algorithm_dispatch_speed/division_trivial_small_quotient` | 156.72 ns | 155.08 ns - 158.42 ns | 160.29 ns | +10.04% | - |
| `rational_algorithm_dispatch_speed/dyadic_fact_cold` | 92.36 ns | 90.13 ns - 94.67 ns | 83.71 ns | +3.51% | - |
| `rational_algorithm_dispatch_speed/dyadic_fact_retained` | 3.32 ns | 3.32 ns - 3.33 ns | 3.32 ns | -0.17% | - |
| `rational_algorithm_dispatch_speed/equality_close_retained_1024_bits` | 293.40 ns | 290.51 ns - 296.29 ns | 304.99 ns | +1.36% | - |
| `rational_algorithm_dispatch_speed/equality_different_signs_retained` | 4.47 ns | 4.43 ns - 4.51 ns | 4.63 ns | +1.55% | - |
| `rational_algorithm_dispatch_speed/equality_dyadic_shifted_retained_1024_bits` | 20.95 ns | 20.77 ns - 21.14 ns | 20.36 ns | +0.45% | - |
| `rational_algorithm_dispatch_speed/equality_equal_distinct_retained` | 10.01 ns | 9.91 ns - 10.11 ns | 9.56 ns | -2.85% | - |
| `rational_algorithm_dispatch_speed/equality_leading_significand_retained_1024_bits` | 272.95 ns | 270.33 ns - 275.55 ns | 283.36 ns | -2.98% | - |
| `rational_algorithm_dispatch_speed/equality_same_denominator_retained` | 23.81 ns | 23.59 ns - 24.04 ns | 23.00 ns | +112.43% | - |
| `rational_algorithm_dispatch_speed/equality_shared_identity_retained` | 4.11 ns | 4.07 ns - 4.15 ns | 4.23 ns | +1.90% | - |
| `rational_algorithm_dispatch_speed/equality_word_retained` | 22.49 ns | 22.27 ns - 22.73 ns | 21.61 ns | +3.89% | - |
| `rational_algorithm_dispatch_speed/exact_remainder_large_knuth` | 11.38 us | 11.27 us - 11.49 us | 11.87 us | +7.01% | - |
| `rational_algorithm_dispatch_speed/gcd_euclidean_1024_bits` | 171.71 us | 171.46 us - 172.18 us | 171.44 us | -0.17% | - |
| `rational_algorithm_dispatch_speed/gcd_euclidean_128_bits` | 12.85 us | 12.73 us - 12.98 us | 12.54 us | +2.83% | - |
| `rational_algorithm_dispatch_speed/gcd_euclidean_192_bits` | 20.34 us | 20.18 us - 20.51 us | 19.90 us | +2.11% | - |
| `rational_algorithm_dispatch_speed/gcd_euclidean_4096_bits` | 1.12 ms | 1.11 ms - 1.12 ms | 1.11 ms | -0.08% | - |
| `rational_algorithm_dispatch_speed/gcd_euclidean_512_bits` | 75.20 us | 74.56 us - 75.88 us | 73.37 us | +0.61% | - |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_1024_bits` | 180.95 us | 178.97 us - 183.00 us | 188.85 us | +7.61% | - |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_192_bits` | 19.65 us | 19.49 us - 19.82 us | 19.25 us | -3.50% | - |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_256_bits` | 32.80 us | 32.50 us - 33.12 us | 31.74 us | +0.99% | - |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_4096_bits` | 1.16 ms | 1.16 ms - 1.17 ms | 1.15 ms | +0.97% | - |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_512_bits` | 73.51 us | 72.87 us - 74.19 us | 71.44 us | +2.95% | - |
| `rational_algorithm_dispatch_speed/gcd_selected_1024_bits` | 59.82 us | 59.23 us - 60.42 us | 57.28 us | +0.80% | - |
| `rational_algorithm_dispatch_speed/gcd_selected_128_bits` | 412.94 ns | 412.78 ns - 413.11 ns | 412.92 ns | +0.51% | - |
| `rational_algorithm_dispatch_speed/gcd_selected_192_bits` | 13.35 us | 13.34 us - 13.35 us | 13.35 us | +4.88% | - |
| `rational_algorithm_dispatch_speed/gcd_selected_4096_bits` | 296.36 us | 294.01 us - 298.89 us | 290.04 us | +3.51% | - |
| `rational_algorithm_dispatch_speed/gcd_selected_512_bits` | 31.25 us | 30.91 us - 31.60 us | 32.24 us | +7.29% | - |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_1024_bits` | 65.15 us | 64.52 us - 65.80 us | 62.57 us | +3.76% | - |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_192_bits` | 19.59 us | 19.55 us - 19.67 us | 19.54 us | +0.13% | - |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_256_bits` | 20.20 us | 20.10 us - 20.32 us | 20.00 us | +3.46% | - |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_4096_bits` | 317.85 us | 316.06 us - 319.89 us | 314.43 us | +2.58% | - |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_512_bits` | 33.32 us | 32.99 us - 33.64 us | 34.74 us | +5.77% | - |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_1048576_bits` | 9.19 s | 9.19 s - 9.19 s | 9.19 s | -0.14% | - |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_16384_bits` | 6.78 ms | 6.77 ms - 6.80 ms | 6.77 ms | +0.40% | - |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_262144_bits` | 626.31 ms | 626.00 ms - 626.63 ms | 625.60 ms | -0.04% | - |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_65536_bits` | 23.35 ms | 23.21 ms - 23.51 ms | 23.07 ms | +1.38% | - |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_8192_bits` | 715.04 us | 712.72 us - 718.04 us | 712.42 us | +1.63% | - |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_1048576_bits` | 4.62 s | 4.62 s - 4.62 s | 4.62 s | -0.24% | - |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_16384_bits` | 2.09 ms | 2.07 ms - 2.11 ms | 2.03 ms | +3.22% | - |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_262144_bits` | 289.49 ms | 289.31 ms - 289.69 ms | 289.07 ms | +0.07% | - |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_65536_bits` | 20.43 ms | 20.27 ms - 20.61 ms | 19.99 ms | +1.69% | - |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_8192_bits` | 730.53 us | 724.26 us - 737.17 us | 712.74 us | +4.14% | - |
| `rational_algorithm_dispatch_speed/mul_backend_basecase_cold` | 578.62 ns | 545.86 ns - 638.84 ns | 571.34 ns | +3.35% | - |
| `rational_algorithm_dispatch_speed/mul_backend_half_karatsuba_cold` | 1.11 us | 1.10 us - 1.12 us | 1.13 us | -1.01% | - |
| `rational_algorithm_dispatch_speed/mul_backend_karatsuba_cold` | 1.91 us | 1.90 us - 1.92 us | 1.92 us | +5.67% | - |
| `rational_algorithm_dispatch_speed/mul_backend_reference_1048576_bits` | 27.79 ms | 27.59 ms - 28.02 ms | 27.33 ms | +1.38% | - |
| `rational_algorithm_dispatch_speed/mul_backend_reference_131072_bits` | 1.30 ms | 1.29 ms - 1.32 ms | 1.36 ms | +4.92% | - |
| `rational_algorithm_dispatch_speed/mul_backend_reference_16384_bits` | 50.35 us | 50.29 us - 50.47 us | 50.28 us | -0.16% | - |
| `rational_algorithm_dispatch_speed/mul_backend_reference_2097152_bits` | 78.12 ms | 77.75 ms - 78.57 ms | 77.54 ms | -1.17% | - |
| `rational_algorithm_dispatch_speed/mul_backend_reference_262144_bits` | 3.77 ms | 3.73 ms - 3.81 ms | 3.92 ms | +4.49% | - |
| `rational_algorithm_dispatch_speed/mul_backend_reference_4096_bits` | 5.52 us | 5.47 us - 5.57 us | 5.73 us | +5.38% | - |
| `rational_algorithm_dispatch_speed/mul_backend_reference_524288_bits` | 10.05 ms | 9.98 ms - 10.12 ms | 9.92 ms | +0.39% | - |
| `rational_algorithm_dispatch_speed/mul_backend_reference_65536_bits` | 436.04 us | 435.58 us - 436.68 us | 435.37 us | -0.03% | - |
| `rational_algorithm_dispatch_speed/mul_backend_toom3_cold` | 20.13 us | 20.00 us - 20.26 us | 20.47 us | +1.13% | - |
| `rational_algorithm_dispatch_speed/mul_backend_unbalanced_1258291_by_1048576` | 36.23 ms | 35.87 ms - 36.59 ms | 37.73 ms | +5.39% | - |
| `rational_algorithm_dispatch_speed/mul_backend_unbalanced_599186_by_524288` | 11.69 ms | 11.69 ms - 11.69 ms | 11.69 ms | -0.05% | - |
| `rational_algorithm_dispatch_speed/mul_ntt_candidate_1048576_bits` | 263.72 ms | 263.58 ms - 263.88 ms | 263.46 ms | +0.94% | - |
| `rational_algorithm_dispatch_speed/mul_ntt_candidate_262144_bits` | 58.09 ms | 58.07 ms - 58.10 ms | 58.08 ms | -0.06% | - |
| `rational_algorithm_dispatch_speed/mul_ntt_candidate_4194304_bits` | 1.18 s | 1.18 s - 1.18 s | 1.18 s | -0.02% | - |
| `rational_algorithm_dispatch_speed/mul_selected_1048576_bits` | 25.14 ms | 24.95 ms - 25.35 ms | 24.71 ms | +1.45% | - |
| `rational_algorithm_dispatch_speed/mul_selected_2097152_bits` | 66.25 ms | 65.89 ms - 66.66 ms | 65.55 ms | +0.79% | - |
| `rational_algorithm_dispatch_speed/mul_selected_262144_bits` | 3.84 ms | 3.80 ms - 3.87 ms | 3.75 ms | -0.26% | - |
| `rational_algorithm_dispatch_speed/mul_selected_4194304_bits` | 174.07 ms | 173.71 ms - 174.58 ms | 173.46 ms | +0.01% | - |
| `rational_algorithm_dispatch_speed/mul_selected_524288_bits` | 9.92 ms | 9.84 ms - 10.01 ms | 9.65 ms | +2.78% | - |
| `rational_algorithm_dispatch_speed/mul_selected_toom4_unbalanced_1258291_by_1048576` | 34.44 ms | 34.28 ms - 34.63 ms | 34.19 ms | -0.19% | - |
| `rational_algorithm_dispatch_speed/mul_selected_toom6_unbalanced_599186_by_524288` | 11.53 ms | 11.49 ms - 11.58 ms | 11.48 ms | -1.41% | - |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_1048576_bits` | 28.28 ms | 28.23 ms - 28.36 ms | 28.22 ms | -0.35% | - |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_16384_bits` | 96.28 us | 96.19 us - 96.44 us | 96.18 us | -0.03% | - |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_2097152_bits` | 76.62 ms | 76.53 ms - 76.73 ms | 76.49 ms | -0.76% | - |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_262144_bits` | 4.06 ms | 4.02 ms - 4.09 ms | 3.95 ms | -6.52% | - |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_4096_bits` | 22.21 us | 22.09 us - 22.35 us | 22.00 us | +1.01% | - |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_524288_bits` | 10.51 ms | 10.46 ms - 10.58 ms | 10.42 ms | +0.51% | - |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_65536_bits` | 599.67 us | 593.90 us - 605.65 us | 576.15 us | +1.37% | - |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_1048576_bits` | 26.20 ms | 26.15 ms - 26.28 ms | 26.15 ms | +0.02% | - |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_131072_bits` | 1.51 ms | 1.51 ms - 1.51 ms | 1.51 ms | -4.53% | - |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_2097152_bits` | 69.06 ms | 69.01 ms - 69.12 ms | 69.01 ms | -0.03% | - |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_262144_bits` | 3.91 ms | 3.87 ms - 3.94 ms | 3.79 ms | +1.28% | - |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_524288_bits` | 10.14 ms | 10.07 ms - 10.22 ms | 9.96 ms | +1.14% | - |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_1048576_bits` | 24.77 ms | 24.71 ms - 24.84 ms | 24.71 ms | -0.46% | - |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_131072_bits` | 1.61 ms | 1.59 ms - 1.62 ms | 1.55 ms | +3.06% | - |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_2097152_bits` | 66.24 ms | 65.87 ms - 66.67 ms | 65.49 ms | +1.04% | - |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_262144_bits` | 3.83 ms | 3.80 ms - 3.87 ms | 3.75 ms | +1.44% | - |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_4194304_bits` | 174.15 ms | 174.02 ms - 174.29 ms | 173.84 ms | -0.28% | - |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_524288_bits` | 9.83 ms | 9.75 ms - 9.91 ms | 9.64 ms | -5.75% | - |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_65536_bits` | 670.77 us | 665.30 us - 676.61 us | 655.41 us | +1.45% | - |
| `rational_algorithm_dispatch_speed/perfect_power_factor_reject` | 176.51 ns | 174.85 ns - 178.09 ns | 181.14 ns | +5.63% | - |
| `rational_algorithm_dispatch_speed/perfect_power_fixed_seventh` | 579.70 ns | 566.77 ns - 597.04 ns | 593.60 ns | +8.48% | - |
| `rational_algorithm_dispatch_speed/perfect_power_general_seventh` | 4.08 us | 4.04 us - 4.12 us | 3.93 us | -1.26% | - |
| `rational_algorithm_dispatch_speed/perfect_power_unfactored_reject` | 8.80 us | 8.71 us - 8.89 us | 9.14 us | -0.01% | - |
| `rational_algorithm_dispatch_speed/radix_format_fraction_decimal` | 6.39 us | 6.31 us - 6.50 us | 6.17 us | +11.54% | - |
| `rational_algorithm_dispatch_speed/radix_format_large_integer` | 16.44 us | 16.29 us - 16.59 us | 16.88 us | +0.98% | - |
| `rational_algorithm_dispatch_speed/radix_format_small_integer` | 4.50 us | 4.44 us - 4.57 us | 4.26 us | +1.38% | - |
| `rational_algorithm_dispatch_speed/radix_parse_backend_chunked_10240_digits` | 190.18 us | 188.55 us - 191.93 us | 185.18 us | -4.29% | - |
| `rational_algorithm_dispatch_speed/radix_parse_backend_chunked_20480_digits` | 737.19 us | 731.28 us - 742.90 us | 752.15 us | +8.18% | - |
| `rational_algorithm_dispatch_speed/radix_parse_divide_conquer_10240_digits` | 194.91 us | 194.07 us - 195.93 us | 193.73 us | -4.36% | - |
| `rational_algorithm_dispatch_speed/radix_parse_divide_conquer_20480_digits` | 607.11 us | 601.61 us - 612.57 us | 625.68 us | +3.38% | - |
| `rational_algorithm_dispatch_speed/radix_parse_large_integer` | 3.31 us | 3.28 us - 3.37 us | 3.23 us | +1.40% | - |
| `rational_algorithm_dispatch_speed/radix_parse_short_decimal` | 198.19 ns | 196.12 ns - 200.28 ns | 204.98 ns | +8.35% | - |
| `rational_algorithm_dispatch_speed/radix_parse_short_scientific` | 174.78 ns | 173.22 ns - 176.38 ns | 169.57 ns | +4.68% | - |
| `rational_algorithm_dispatch_speed/radix_parse_wide_scientific` | 79.98 us | 79.24 us - 80.75 us | 77.42 us | -2.45% | - |
| `rational_algorithm_dispatch_speed/radix_parse_wide_scientific_expanded` | 96.33 us | 96.30 us - 96.36 us | 96.30 us | -1.36% | - |
| `rational_algorithm_dispatch_speed/reduce_backend_knuth_cold` | 1.78 us | 1.76 us - 1.80 us | 1.71 us | +1.42% | - |
| `rational_algorithm_dispatch_speed/reduce_backend_large_knuth_cold` | 24.01 us | 23.98 us - 24.06 us | 23.98 us | +0.12% | - |
| `rational_algorithm_dispatch_speed/reduce_backend_single_limb_cold` | 380.48 ns | 376.45 ns - 385.31 ns | 370.84 ns | -2.33% | - |
| `rational_algorithm_dispatch_speed/reduce_fixed_512_coprime_cold` | 7.73 us | 7.66 us - 7.82 us | 7.84 us | +6.19% | - |
| `symbolic_reductions/div_rational_exp` | 414.05 ns | 407.98 ns - 420.30 ns | 427.97 ns | +0.34% | - |

<!-- END COMPLETE BENCHMARK REPORT -->
