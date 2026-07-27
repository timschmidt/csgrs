<!-- BEGIN promoted_slow_offender_score -->
## `promoted_slow_offender_score`

Deterministic lexicase score for the current 100 promoted slow offenders. The score is the average current best-of-five wall-clock probe across the promoted set; lower is better. Delta compares with the previous score recorded in this file, and derivative is the change in delta.

<!-- promoted_slow_score_nanos: 48601 -->
<!-- promoted_slow_previous_score_nanos: 48601 -->
<!-- promoted_slow_score_delta_nanos: 0 -->

| Metric | Value |
| --- | ---: |
| Cases scored | 100 |
| Average score | 48.601 us |
| Delta | 0 ns |
| Delta derivative | 0 ns |

| Rank | Current Time | Operation | Input |
| ---: | ---: | --- | --- |
| 1 | 126.282 us | `generated_tan_p96` | `generated[18246] -1 187/188` |
| 2 | 125.521 us | `generated_tan_p96` | `generated[3756] -1 123/214` |
| 3 | 123.411 us | `generated_tan_p96` | `generated[5916] -1 337/578` |
| 4 | 122.762 us | `generated_tan_p96` | `generated[11691] 1 431/439` |
| 5 | 122.512 us | `generated_tan_p96` | `generated[12186] -1 189/299` |
| 6 | 122.132 us | `generated_tan_p96` | `generated[8976] 1 71/73` |
| 7 | 121.481 us | `generated_tan_p96` | `generated[18276] -1 77/107` |
| 8 | 120.352 us | `generated_tan_p96` | `generated[321] 1 214/231` |
| 9 | 119.152 us | `generated_tan_p96` | `generated[486] 1 53/71` |
| 10 | 118.002 us | `generated_tan_p96` | `generated[2016] 1 101/141` |

<!-- END promoted_slow_offender_score -->

<!-- BEGIN numerical_micro -->
## `numerical_micro`

Low-level `Computable` microbenchmarks for approximation kernels, caches, structural facts, comparisons, and deep evaluator trees.

### `computable_cache`

Cold versus cached approximation of basic `Computable` expressions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_cache/ratio_approx_cold_p128` | 24.86 ns | 24.80 ns - 24.94 ns | Approximates a rational value at p=-128 from a fresh clone. |
| `computable_cache/ratio_approx_cached_p128` | 19.68 ns | 19.62 ns - 19.77 ns | Repeats an already cached rational approximation at p=-128. |
| `computable_cache/pi_approx_cold_p128` | 28.44 ns | 28.38 ns - 28.52 ns | Approximates pi at p=-128 from a fresh clone. |
| `computable_cache/pi_approx_cached_p128` | 20.54 ns | 20.52 ns - 20.58 ns | Repeats an already cached pi approximation at p=-128. |
| `computable_cache/pi_plus_tiny_cold_p128` | 28.46 ns | 28.36 ns - 28.58 ns | Approximates pi plus a tiny exact rational perturbation. |
| `computable_cache/pi_minus_tiny_cold_p128` | 28.33 ns | 28.30 ns - 28.37 ns | Approximates pi minus a tiny exact rational perturbation. |

### `computable_bounds`

Structural sign and bound discovery for deep or perturbed computable trees.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_bounds/deep_scaled_product_sign` | 5.69 ns | 5.69 ns - 5.70 ns | Finds the sign of a deep scaled product. |
| `computable_bounds/scaled_square_sign` | 5.97 ns | 5.93 ns - 6.02 ns | Finds the sign of repeated squaring with exact scale factors. |
| `computable_bounds/sqrt_scaled_square_sign` | 57.05 ns | 53.87 ns - 60.30 ns | Finds the sign after taking a square root of a scaled square. |
| `computable_bounds/deep_structural_bound_sign` | 5.95 ns | 5.92 ns - 6.00 ns | Finds sign through repeated multiply/inverse/negate structural transformations. |
| `computable_bounds/deep_structural_bound_sign_cached` | 4.18 ns | 4.12 ns - 4.24 ns | Reads the cached sign of the deep structural-bound chain. |
| `computable_bounds/deep_structural_bound_facts_cached` | 7.98 ns | 7.97 ns - 8.00 ns | Reads cached structural facts for the deep structural-bound chain. |
| `computable_bounds/perturbed_scaled_product_sign` | 5.91 ns | 5.90 ns - 5.92 ns | Finds sign for a deeply scaled value with a tiny perturbation. |
| `computable_bounds/perturbed_scaled_product_sign_until` | 6.14 ns | 6.14 ns - 6.15 ns | Refines sign for the perturbed scaled product only to p=-128. |
| `computable_bounds/pi_minus_tiny_sign` | 5.76 ns | 5.73 ns - 5.79 ns | Finds sign for pi minus a tiny exact rational. |
| `computable_bounds/pi_minus_tiny_sign_cached` | 4.13 ns | 4.07 ns - 4.18 ns | Reads cached sign for pi minus a tiny exact rational. |
| `computable_bounds/exp_unknown_sign_arg_sign` | 5.71 ns | 5.70 ns - 5.72 ns | Finds sign for exp(1 - pi), where exp can prove positivity structurally. |
| `computable_bounds/exp_unknown_sign_arg_sign_cached` | 4.03 ns | 4.00 ns - 4.06 ns | Reads cached sign for exp(1 - pi). |

### `computable_compare`

Ordering and absolute-comparison shortcuts.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_compare/compare_to_opposite_sign` | 11.11 ns | 11.09 ns - 11.14 ns | Compares values with known opposite signs. |
| `computable_compare/compare_to_exact_msd_gap` | 18.34 ns | 18.33 ns - 18.34 ns | Compares values with a large exact magnitude gap. |
| `computable_compare/compare_to_clone_shared_composite` | 3.54 ns | 3.53 ns - 3.55 ns | Compares two handles that share one composite expression node. |
| `computable_compare/compare_absolute_exact_rational` | 4.51 ns | 4.49 ns - 4.53 ns | Compares absolute values of exact rationals. |
| `computable_compare/compare_absolute_exact_rational_same_numerator` | 109.00 ns | 108.38 ns - 109.85 ns | Compares exact rational magnitudes with matching numerators. |
| `computable_compare/compare_absolute_dominant_add` | 14.35 ns | 14.34 ns - 14.37 ns | Compares a dominant term against the same term plus a tiny addend. |
| `computable_compare/compare_absolute_exact_msd_gap` | 15.57 ns | 15.55 ns - 15.60 ns | Compares absolute values with a large exact magnitude gap. |

### `computable_transcendentals`

Low-level approximation kernels and deep expression-tree stress cases.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_transcendentals/e_constant_cold_p128` | 39.26 ns | 39.03 ns - 39.48 ns | Approximates the shared e constant from a fresh clone. |
| `computable_transcendentals/e_constant_cached_p128` | 20.34 ns | 20.31 ns - 20.36 ns | Repeats a cached approximation of e. |
| `computable_transcendentals/exp_cold_p128` | 3.847 us | 3.841 us - 3.854 us | Approximates exp(7/5) from a fresh clone. |
| `computable_transcendentals/exp_cached_p128` | 20.12 ns | 20.07 ns - 20.16 ns | Repeats a cached exp(7/5) approximation. |
| `computable_transcendentals/exp_large_cold_p128` | 4.387 us | 4.382 us - 4.392 us | Approximates exp(128), exercising the bounded exact-integer power path. |
| `computable_transcendentals/exp_negative_integer_cold_p128` | 2.094 us | 2.091 us - 2.098 us | Approximates exp(-32), retaining signed ln(2) range reduction. |
| `computable_transcendentals/exp_integer_limit_cold_p128` | 6.228 us | 6.214 us - 6.243 us | Approximates exp(256), guarding the binary e-power limit. |
| `computable_transcendentals/exp_integer_above_limit_cold_p128` | 11.575 us | 11.559 us - 11.595 us | Approximates exp(257), retaining the ln(2) range-reduction fallback. |
| `computable_transcendentals/exp_half_cold_p128` | 2.877 us | 2.873 us - 2.883 us | Approximates exp(1/2). |
| `computable_transcendentals/exp_near_limit_cold_p128` | 2.887 us | 2.881 us - 2.895 us | Approximates exp near a prescaling threshold. |
| `computable_transcendentals/exp_near_limit_cached_p128` | 19.98 ns | 19.96 ns - 20.00 ns | Repeats a cached near-threshold exp approximation. |
| `computable_transcendentals/exp_zero_cold_p128` | 53.80 ns | 53.55 ns - 54.03 ns | Approximates exp(0). |
| `computable_transcendentals/ln_cold_p128` | 3.103 us | 3.086 us - 3.128 us | Approximates ln(11/7). |
| `computable_transcendentals/ln_cached_p128` | 19.82 ns | 19.80 ns - 19.84 ns | Repeats a cached ln(11/7) approximation. |
| `computable_transcendentals/ln_smooth_rational_cold_p128` | 634.20 ns | 626.72 ns - 641.82 ns | Approximates ln(45/14), which can decompose into shared prime-log constants. |
| `computable_transcendentals/ln_nonsmooth_rational_cold_p128` | 2.565 us | 2.558 us - 2.577 us | Approximates ln(11/13), guarding the generic exact-rational log fallback. |
| `computable_transcendentals/ln_large_cold_p128` | 909.78 ns | 900.73 ns - 919.47 ns | Approximates ln(1024), exercising large-input reduction. |
| `computable_transcendentals/ln_large_cached_p128` | 19.95 ns | 19.93 ns - 19.96 ns | Repeats a cached ln(1024) approximation. |
| `computable_transcendentals/ln_tiny_cold_p128` | 180.32 ns | 179.51 ns - 181.09 ns | Approximates ln(2^-1024), exercising tiny-input reduction. |
| `computable_transcendentals/ln_near_limit_cold_p128` | 3.232 us | 3.222 us - 3.244 us | Approximates ln near the prescaled-ln limit. |
| `computable_transcendentals/ln_near_limit_cached_p128` | 19.87 ns | 19.86 ns - 19.89 ns | Repeats a cached near-limit ln approximation. |
| `computable_transcendentals/ln_one_cold_p128` | 21.94 ns | 21.71 ns - 22.15 ns | Approximates ln(1). |
| `computable_transcendentals/sqrt_cold_p128` | 655.89 ns | 655.23 ns - 656.55 ns | Approximates sqrt(2). |
| `computable_transcendentals/sqrt_squarefree_scaled_cold_p128` | 102.16 ns | 101.24 ns - 103.17 ns | Approximates sqrt(12), which can reduce to 2*sqrt(3). |
| `computable_transcendentals/sqrt_cached_p128` | 19.83 ns | 19.81 ns - 19.86 ns | Repeats a cached sqrt(2) approximation. |
| `computable_transcendentals/sqrt_single_scaled_square_cold_p128` | 769.49 ns | 768.77 ns - 770.31 ns | Builds and approximates sqrt((7*pi/8)^2). |
| `computable_transcendentals/sin_cold_p96` | 1.608 us | 1.607 us - 1.610 us | Approximates sin(7/5). |
| `computable_transcendentals/sin_cached_p96` | 19.96 ns | 19.94 ns - 19.99 ns | Repeats a cached sin(7/5) approximation. |
| `computable_transcendentals/cos_cold_p96` | 1.513 us | 1.507 us - 1.520 us | Approximates cos(7/5). |
| `computable_transcendentals/sin_f64_cold_p96` | 1.765 us | 1.761 us - 1.770 us | Approximates sin of the exact binary64-derived dyadic for 1.23456789. |
| `computable_transcendentals/cos_f64_cold_p96` | 1.695 us | 1.692 us - 1.700 us | Approximates cos of the exact binary64-derived dyadic for 1.23456789. |
| `computable_transcendentals/sin_1e6_cold_p96` | 2.305 us | 2.303 us - 2.308 us | Approximates sin(1000000). |
| `computable_transcendentals/cos_1e6_cold_p96` | 2.288 us | 2.282 us - 2.296 us | Approximates cos(1000000). |
| `computable_transcendentals/sin_1e30_cold_p96` | 2.177 us | 2.172 us - 2.183 us | Approximates sin(10^30). |
| `computable_transcendentals/cos_1e30_cold_p96` | 2.171 us | 2.162 us - 2.181 us | Approximates cos(10^30). |
| `computable_transcendentals/cos_cached_p96` | 19.92 ns | 19.87 ns - 19.97 ns | Repeats a cached cos(7/5) approximation. |
| `computable_transcendentals/tan_cold_p96` | 6.103 us | 6.079 us - 6.134 us | Approximates tan(7/5). |
| `computable_transcendentals/tan_cached_p96` | 19.85 ns | 19.79 ns - 19.94 ns | Repeats a cached tan(7/5) approximation. |
| `computable_transcendentals/sin_zero_cold_p96` | 21.90 ns | 21.69 ns - 22.08 ns | Approximates sin(0). |
| `computable_transcendentals/cos_zero_cold_p96` | 59.39 ns | 59.16 ns - 59.60 ns | Approximates cos(0). |
| `computable_transcendentals/tan_zero_cold_p96` | 21.96 ns | 21.73 ns - 22.16 ns | Approximates tan(0). |
| `computable_transcendentals/tan_near_half_pi_cold_p96` | 10.232 us | 10.208 us - 10.253 us | Approximates tangent near pi/2. |
| `computable_transcendentals/tan_near_half_pi_cached_p96` | 19.97 ns | 19.91 ns - 20.04 ns | Repeats cached tangent near pi/2. |
| `computable_transcendentals/sin_huge_cold_p96` | 1.611 us | 1.608 us - 1.614 us | Approximates sine of a huge pi multiple plus offset. |
| `computable_transcendentals/cos_huge_cold_p96` | 1.499 us | 1.498 us - 1.500 us | Approximates cosine of a huge pi multiple plus offset. |
| `computable_transcendentals/tan_huge_cold_p96` | 6.058 us | 6.049 us - 6.067 us | Approximates tangent of a huge pi multiple plus offset. |
| `computable_transcendentals/asin_cold_p96` | 5.945 us | 5.930 us - 5.964 us | Approximates a computable asin expression. |
| `computable_transcendentals/asin_cached_p96` | 19.84 ns | 19.81 ns - 19.86 ns | Repeats a cached computable asin approximation. |
| `computable_transcendentals/acos_cold_p96` | 5.746 us | 5.723 us - 5.773 us | Approximates a computable acos expression. |
| `computable_transcendentals/acos_cached_p96` | 19.85 ns | 19.84 ns - 19.87 ns | Repeats a cached computable acos approximation. |
| `computable_transcendentals/asin_tiny_cold_p96` | 383.38 ns | 382.70 ns - 384.09 ns | Approximates asin(1e-12), exercising the tiny-input series. |
| `computable_transcendentals/acos_tiny_cold_p96` | 687.97 ns | 685.88 ns - 690.01 ns | Approximates acos(1e-12), exercising the tiny-input complement. |
| `computable_transcendentals/asin_near_one_cold_p96` | 1.827 us | 1.823 us - 1.832 us | Approximates asin(0.999999), exercising the endpoint complement. |
| `computable_transcendentals/acos_near_one_cold_p96` | 1.585 us | 1.578 us - 1.595 us | Approximates acos(0.999999), exercising the endpoint transform. |
| `computable_transcendentals/atan_cold_p96` | 1.801 us | 1.797 us - 1.806 us | Approximates atan(7/10). |
| `computable_transcendentals/atan_cached_p96` | 19.93 ns | 19.88 ns - 20.00 ns | Repeats a cached atan(7/10) approximation. |
| `computable_transcendentals/atan_large_cold_p96` | 1.562 us | 1.559 us - 1.564 us | Approximates atan(8), exercising argument reduction. |
| `computable_transcendentals/asin_zero_cold_p96` | 21.93 ns | 21.71 ns - 22.14 ns | Approximates asin(0) expression. |
| `computable_transcendentals/atan_zero_cold_p96` | 21.99 ns | 21.76 ns - 22.19 ns | Approximates atan(0). |
| `computable_transcendentals/asinh_cold_p128` | 6.374 us | 6.363 us - 6.387 us | Approximates a computable asinh expression. |
| `computable_transcendentals/asinh_three_quarters_cold_p128` | 4.630 us | 4.626 us - 4.634 us | Approximates asinh(3/4) across the series/ln1p crossover. |
| `computable_transcendentals/asinh_cached_p128` | 19.96 ns | 19.88 ns - 20.05 ns | Repeats a cached computable asinh approximation. |
| `computable_transcendentals/acosh_cold_p128` | 39.21 ns | 39.00 ns - 39.41 ns | Approximates a computable acosh expression. |
| `computable_transcendentals/acosh_cached_p128` | 20.20 ns | 20.15 ns - 20.25 ns | Repeats a cached computable acosh approximation. |
| `computable_transcendentals/atanh_cold_p128` | 145.13 ns | 144.56 ns - 145.64 ns | Approximates a computable atanh expression. |
| `computable_transcendentals/atanh_cached_p128` | 19.92 ns | 19.88 ns - 19.98 ns | Repeats a cached computable atanh approximation. |
| `computable_transcendentals/atanh_tiny_cold_p128` | 507.93 ns | 506.91 ns - 509.01 ns | Approximates atanh(1e-12), exercising the tiny-input series. |
| `computable_transcendentals/atanh_near_one_cold_p128` | 2.138 us | 2.131 us - 2.146 us | Approximates atanh(0.999999), exercising the endpoint log transform. |
| `computable_transcendentals/asinh_zero_cold_p128` | 21.88 ns | 21.68 ns - 22.06 ns | Approximates asinh(0) expression. |
| `computable_transcendentals/atanh_zero_cold_p128` | 21.81 ns | 21.61 ns - 21.98 ns | Approximates atanh(0) expression. |
| `computable_transcendentals/deep_add_chain_cold_p128` | 44.81 ns | 44.75 ns - 44.87 ns | Approximates a 5000-node addition chain. |
| `computable_transcendentals/deep_multiply_chain_cold_p128` | 45.00 ns | 44.89 ns - 45.14 ns | Approximates a 5000-node multiply-by-one chain. |
| `computable_transcendentals/deep_multiply_identity_chain_cold_p128` | 73.04 ns | 72.95 ns - 73.13 ns | Approximates a deep identity multiplication chain around pi. |
| `computable_transcendentals/deep_scaled_product_chain_cold_p128` | 28.52 ns | 28.47 ns - 28.57 ns | Approximates a deep product of exact scale factors. |
| `computable_transcendentals/perturbed_scaled_product_chain_cold_p128` | 28.53 ns | 28.48 ns - 28.60 ns | Approximates a deep scaled product with a tiny perturbation. |
| `computable_transcendentals/scaled_square_chain_cold_p128` | 29.95 ns | 29.92 ns - 29.99 ns | Approximates repeated squaring of a scaled irrational. |
| `computable_transcendentals/asymmetric_product_bad_order_cold_p128` | 29.76 ns | 29.71 ns - 29.82 ns | Approximates an asymmetric product order stress case. |
| `computable_transcendentals/sqrt_scaled_square_chain_cold_p128` | 456.23 ns | 453.08 ns - 459.50 ns | Approximates sqrt of a scaled-square chain. |
| `computable_transcendentals/warmed_zero_product_cold_p128` | 17.88 ns | 17.84 ns - 17.93 ns | Approximates a product involving a warmed zero sum. |
| `computable_transcendentals/inverse_scaled_product_chain_cold_p128` | 28.57 ns | 28.55 ns - 28.60 ns | Approximates the inverse of a deep scaled product. |
| `computable_transcendentals/deep_inverse_pair_chain_cold_p128` | 72.77 ns | 72.70 ns - 72.85 ns | Approximates a chain of inverse(inverse(x)) pairs. |
| `computable_transcendentals/deep_negated_square_chain_cold_p128` | 73.04 ns | 72.90 ns - 73.22 ns | Approximates repeated negate-square-sqrt transformations. |
| `computable_transcendentals/deep_negative_one_product_chain_cold_p128` | 73.49 ns | 73.21 ns - 73.82 ns | Approximates repeated multiplication by -1. |
| `computable_transcendentals/deep_half_product_chain_cold_p128` | 17.86 ns | 17.84 ns - 17.90 ns | Approximates repeated multiplication by 1/2. |
| `computable_transcendentals/deep_half_square_chain_cold_p128` | 28.56 ns | 28.54 ns - 28.59 ns | Approximates repeated squaring after scaling by 1/2. |
| `computable_transcendentals/deep_sqrt_square_chain_cold_p128` | 44.89 ns | 44.81 ns - 44.97 ns | Approximates repeated sqrt-square simplification. |
| `computable_transcendentals/inverse_half_product_chain_cold_p128` | 29.74 ns | 29.69 ns - 29.80 ns | Approximates the inverse of a deep half-product chain. |

<!-- END numerical_micro -->

<!-- BEGIN scalar_micro -->
## `scalar_micro`

Microbenchmarks for scalar operations, structural queries, cache hits, and dense exact arithmetic.

### `construction_speed`

Cost of constructing common exact scalar identities and small integers.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `construction_speed/rational_one` | 3.17 ns | 3.16 ns - 3.17 ns | Constructs `Rational::one()`. |
| `construction_speed/rational_new_one` | 3.76 ns | 3.76 ns - 3.77 ns | Constructs one through `Rational::new(1)`. |
| `construction_speed/rational_from_u8_four` | 5.75 ns | 5.70 ns - 5.81 ns | Constructs positive four through unsigned primitive conversion. |
| `construction_speed/rational_from_i8_minus_four` | 5.53 ns | 5.52 ns - 5.55 ns | Constructs negative four through signed primitive conversion. |
| `construction_speed/computable_one` | 13.65 ns | 13.63 ns - 13.68 ns | Constructs `Computable::one()`. |
| `construction_speed/real_new_rational_one` | 16.75 ns | 16.73 ns - 16.78 ns | Constructs one through `Real::new(Rational::one())`. |
| `construction_speed/real_one` | 16.06 ns | 16.04 ns - 16.07 ns | Constructs one through `Real::one()`. |
| `construction_speed/real_from_i32_one` | 16.40 ns | 16.34 ns - 16.48 ns | Constructs one through integer conversion. |
| `construction_speed/real_from_u8_four` | 18.40 ns | 18.34 ns - 18.47 ns | Constructs positive four as an exact `Real` from `u8`. |
| `construction_speed/real_from_i8_minus_four` | 19.97 ns | 19.93 ns - 20.01 ns | Constructs negative four as an exact `Real` from `i8`. |

### `raw_cache_hit_cost`

Cost of cold and cached `Computable::approx` calls for simple values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `raw_cache_hit_cost/zero` | 9.04 ns | 9.02 ns - 9.06 ns | Cached approximation request for exact zero. |
| `raw_cache_hit_cost/one` | 37.95 ns | 37.91 ns - 38.00 ns | Cached approximation request for exact one. |
| `raw_cache_hit_cost/two` | 37.81 ns | 37.78 ns - 37.86 ns | Cached approximation request for exact two. |
| `raw_cache_hit_cost/e` | 67.98 ns | 67.88 ns - 68.12 ns | Cached approximation request for Euler's constant. |
| `raw_cache_hit_cost/pi` | 68.10 ns | 67.89 ns - 68.33 ns | Cached approximation request for pi. |
| `raw_cache_hit_cost/tau` | 67.65 ns | 67.61 ns - 67.69 ns | Cached approximation request for two pi. |

### `structural_query_speed`

Speed of public structural queries across exact, transcendental, and composite `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `structural_query_speed/zero_zero_status` | 0.92 ns | 0.92 ns - 0.92 ns | Checks zero/nonzero facts for exact zero. |
| `structural_query_speed/zero_sign_query` | 4.50 ns | 4.49 ns - 4.51 ns | Reads sign facts for exact zero. |
| `structural_query_speed/zero_msd_query` | 6.11 ns | 6.11 ns - 6.11 ns | Reads magnitude facts for exact zero. |
| `structural_query_speed/zero_structural_facts` | 7.06 ns | 7.05 ns - 7.06 ns | Computes full structural facts for exact zero. |
| `structural_query_speed/one_zero_status` | 1.03 ns | 1.03 ns - 1.03 ns | Checks zero/nonzero facts for exact one. |
| `structural_query_speed/one_sign_query` | 12.91 ns | 12.90 ns - 12.93 ns | Reads sign facts for exact one. |
| `structural_query_speed/one_msd_query` | 14.78 ns | 14.76 ns - 14.80 ns | Reads magnitude facts for exact one. |
| `structural_query_speed/one_structural_facts` | 15.62 ns | 15.60 ns - 15.64 ns | Computes full structural facts for exact one. |
| `structural_query_speed/negative_zero_status` | 1.03 ns | 1.02 ns - 1.03 ns | Checks zero/nonzero facts for an exact negative integer. |
| `structural_query_speed/negative_sign_query` | 12.93 ns | 12.92 ns - 12.94 ns | Reads sign facts for an exact negative integer. |
| `structural_query_speed/negative_msd_query` | 15.43 ns | 15.38 ns - 15.50 ns | Reads magnitude facts for an exact negative integer. |
| `structural_query_speed/negative_structural_facts` | 16.23 ns | 16.21 ns - 16.26 ns | Computes full structural facts for an exact negative integer. |
| `structural_query_speed/tiny_exact_zero_status` | 1.06 ns | 1.05 ns - 1.07 ns | Checks zero/nonzero facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_sign_query` | 14.38 ns | 14.37 ns - 14.40 ns | Reads sign facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_msd_query` | 17.05 ns | 17.02 ns - 17.09 ns | Reads magnitude facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_structural_facts` | 17.47 ns | 17.45 ns - 17.49 ns | Computes full structural facts for a tiny exact rational. |
| `structural_query_speed/pi_zero_status` | 1.05 ns | 1.04 ns - 1.05 ns | Checks zero/nonzero facts for pi. |
| `structural_query_speed/pi_sign_query` | 19.08 ns | 19.05 ns - 19.12 ns | Reads sign facts for pi. |
| `structural_query_speed/pi_msd_query` | 20.53 ns | 20.51 ns - 20.56 ns | Reads magnitude facts for pi. |
| `structural_query_speed/pi_structural_facts` | 21.43 ns | 21.41 ns - 21.46 ns | Computes full structural facts for pi. |
| `structural_query_speed/e_zero_status` | 1.05 ns | 1.04 ns - 1.06 ns | Checks zero/nonzero facts for e. |
| `structural_query_speed/e_sign_query` | 19.13 ns | 19.08 ns - 19.20 ns | Reads sign facts for e. |
| `structural_query_speed/e_msd_query` | 20.82 ns | 20.70 ns - 20.95 ns | Reads magnitude facts for e. |
| `structural_query_speed/e_structural_facts` | 21.43 ns | 21.41 ns - 21.45 ns | Computes full structural facts for e. |
| `structural_query_speed/tau_zero_status` | 1.04 ns | 1.04 ns - 1.04 ns | Checks zero/nonzero facts for tau. |
| `structural_query_speed/tau_sign_query` | 19.33 ns | 19.31 ns - 19.36 ns | Reads sign facts for tau. |
| `structural_query_speed/tau_msd_query` | 20.87 ns | 20.84 ns - 20.91 ns | Reads magnitude facts for tau. |
| `structural_query_speed/tau_structural_facts` | 21.70 ns | 21.69 ns - 21.71 ns | Computes full structural facts for tau. |
| `structural_query_speed/sqrt_two_zero_status` | 1.04 ns | 1.04 ns - 1.04 ns | Checks zero/nonzero facts for sqrt(2). |
| `structural_query_speed/sqrt_two_sign_query` | 19.10 ns | 19.08 ns - 19.13 ns | Reads sign facts for sqrt(2). |
| `structural_query_speed/sqrt_two_msd_query` | 20.52 ns | 20.50 ns - 20.56 ns | Reads magnitude facts for sqrt(2). |
| `structural_query_speed/sqrt_two_structural_facts` | 21.41 ns | 21.39 ns - 21.42 ns | Computes full structural facts for sqrt(2). |
| `structural_query_speed/pi_minus_three_zero_status` | 1.03 ns | 1.03 ns - 1.03 ns | Checks zero/nonzero facts for pi - 3. |
| `structural_query_speed/pi_minus_three_sign_query` | 19.11 ns | 19.09 ns - 19.13 ns | Reads sign facts for pi - 3. |
| `structural_query_speed/pi_minus_three_msd_query` | 20.69 ns | 20.59 ns - 20.81 ns | Reads magnitude facts for pi - 3. |
| `structural_query_speed/pi_minus_three_structural_facts` | 21.41 ns | 21.39 ns - 21.43 ns | Computes full structural facts for pi - 3. |
| `structural_query_speed/dense_expr_zero_status` | 3.54 ns | 3.53 ns - 3.55 ns | Checks zero/nonzero facts for a dense composite expression. |
| `structural_query_speed/dense_expr_sign_query` | 19.24 ns | 19.16 ns - 19.35 ns | Reads sign facts for a dense composite expression. |
| `structural_query_speed/dense_expr_msd_query` | 20.55 ns | 20.55 ns - 20.56 ns | Reads magnitude facts for a dense composite expression. |
| `structural_query_speed/dense_expr_structural_facts` | 21.78 ns | 21.72 ns - 21.87 ns | Computes full structural facts for a dense composite expression. |

### `pure_scalar_algorithm_speed`

Core scalar algorithms that do not require high-precision transcendental approximation.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `pure_scalar_algorithm_speed/rational_add` | 8.32 ns | 8.30 ns - 8.35 ns | Adds two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_sub` | 9.30 ns | 9.28 ns - 9.32 ns | Subtracts two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_add_wide_dyadic_cold` | 92.49 ns | 91.49 ns - 93.39 ns | Adds fresh integer and wide-dyadic operands without retained work. |
| `pure_scalar_algorithm_speed/rational_sub_wide_dyadic_cold` | 97.91 ns | 96.17 ns - 100.42 ns | Subtracts fresh integer and wide-dyadic operands without retained work. |
| `pure_scalar_algorithm_speed/rational_add_shared_cold` | 97.67 ns | 97.03 ns - 98.22 ns | Adds fresh operands whose storage is cloned but whose arithmetic pair is not yet observed. |
| `pure_scalar_algorithm_speed/rational_sub_shared_cold` | 99.88 ns | 99.05 ns - 100.59 ns | Subtracts fresh operands whose storage is cloned but whose arithmetic pair is not yet observed. |
| `pure_scalar_algorithm_speed/rational_scaled_difference_composed_cold` | 316.87 ns | 315.56 ns - 318.15 ns | Computes a fresh wide-integer scaled difference through multiply then subtract. |
| `pure_scalar_algorithm_speed/rational_scaled_difference_fused_cold` | 96.41 ns | 95.43 ns - 97.39 ns | Computes the same fresh wide-integer scaled difference with the fused integer kernel. |
| `pure_scalar_algorithm_speed/rational_cross_difference_unit_divisor_composed_cold` | 669.42 ns | 666.63 ns - 673.67 ns | Computes a fresh wide-integer cross difference and divides it by negative one through general operations. |
| `pure_scalar_algorithm_speed/rational_cross_difference_unit_divisor_fused_cold` | 206.51 ns | 198.25 ns - 221.78 ns | Computes the same cross difference through the checked fused unit-divisor path. |
| `pure_scalar_algorithm_speed/rational_mul` | 23.06 ns | 23.04 ns - 23.09 ns | Multiplies two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_mul_retained_general` | 11.77 ns | 11.76 ns - 11.79 ns | Reuses one retained exact product for an immutable rational operand pair. |
| `pure_scalar_algorithm_speed/rational_mul_wide_dyadic_cold` | 196.85 ns | 194.05 ns - 199.47 ns | Multiplies fresh wide-denominator dyadics whose numerators fit `u128`. |
| `pure_scalar_algorithm_speed/rational_mul_dyadic_general_cross_cancel` | 11.81 ns | 11.78 ns - 11.85 ns | Multiplies a wide dyadic rational by a general rational with a power-of-two numerator. |
| `pure_scalar_algorithm_speed/rational_div` | 158.99 ns | 158.76 ns - 159.24 ns | Divides two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_inverse_owned_cold` | 21.02 ns | 20.99 ns - 21.05 ns | Inverts a fresh uniquely owned nontrivial rational. |
| `pure_scalar_algorithm_speed/rational_inverse_retained` | 7.50 ns | 7.49 ns - 7.51 ns | Reuses the retained reciprocal of a shared nontrivial rational. |
| `pure_scalar_algorithm_speed/rational_neg_owned_cold` | 8.56 ns | 8.53 ns - 8.59 ns | Negates a fresh uniquely owned nontrivial rational in place. |
| `pure_scalar_algorithm_speed/rational_neg_retained` | 8.25 ns | 8.24 ns - 8.26 ns | Reuses the retained opposite sign of a shared nontrivial rational. |
| `pure_scalar_algorithm_speed/real_exact_powi_i64_owned_cold` | 261.50 ns | 261.04 ns - 262.02 ns | Raises a fresh uniquely owned exact rational Real to the fifth power. |
| `pure_scalar_algorithm_speed/real_exact_powi_i64_retained` | 71.60 ns | 71.22 ns - 72.17 ns | Reuses the bounded exact product chain for a shared fifth power. |
| `pure_scalar_algorithm_speed/real_exact_add` | 24.53 ns | 24.40 ns - 24.67 ns | Adds exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_average_pair` | 141.87 ns | 140.81 ns - 143.11 ns | Averages exact rational-backed `Real` values through the fused pair kernel. |
| `pure_scalar_algorithm_speed/real_exact_average_pair_expanded` | 221.06 ns | 219.65 ns - 222.66 ns | Averages exact rational-backed `Real` values through separate add and divide operations. |
| `pure_scalar_algorithm_speed/real_exact_sub` | 24.17 ns | 24.13 ns - 24.21 ns | Subtracts exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_mul` | 33.65 ns | 33.63 ns - 33.67 ns | Multiplies exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_mul_retained` | 25.80 ns | 25.71 ns - 25.92 ns | Reuses the retained exact product beneath rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_div` | 196.42 ns | 195.78 ns - 197.12 ns | Divides exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_sqrt_owned_cold` | 174.94 ns | 174.69 ns - 175.18 ns | Reduces a fresh uniquely owned exact square-root expression. |
| `pure_scalar_algorithm_speed/real_exact_sqrt_reduce` | 81.48 ns | 81.30 ns - 81.69 ns | Reuses the retained reduction of an exact square-root expression. |
| `pure_scalar_algorithm_speed/real_exact_dyadic_sqrt_reduce` | 79.10 ns | 78.69 ns - 79.60 ns | Reuses the square-root reduction of a large exact dyadic rational. |
| `pure_scalar_algorithm_speed/real_exact_general_sqrt_reduce` | 55.62 ns | 55.47 ns - 55.85 ns | Reuses the square-root reduction of a non-dyadic rational sum of squares. |
| `pure_scalar_algorithm_speed/real_exact_dyadic_radical_scale` | 40.81 ns | 40.74 ns - 40.88 ns | Scales an exact reciprocal radical by one exact binary64-derived dyadic coordinate. |
| `pure_scalar_algorithm_speed/real_exact_ln_reduce` | 91.94 ns | 91.84 ns - 92.07 ns | Reduces an exact logarithm of a power of two. |
| `pure_scalar_algorithm_speed/real_pow_small_integer_exponent` | 135.72 ns | 135.36 ns - 136.29 ns | Dispatches `Real::pow` with an exact small-integer exponent. |

### `rational_algorithm_dispatch_speed`

Cold backend algorithm families and retained rational fact dispatch selected from GMP-style operand shapes.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `rational_algorithm_dispatch_speed/dyadic_fact_cold` | 38.59 ns | 37.54 ns - 39.56 ns | Classifies a fresh non-dyadic denominator and retains the result. |
| `rational_algorithm_dispatch_speed/dyadic_fact_retained` | 2.40 ns | 2.39 ns - 2.41 ns | Reads an already-retained non-dyadic denominator classification. |
| `rational_algorithm_dispatch_speed/mul_backend_basecase_cold` | 358.87 ns | 263.60 ns - 547.37 ns | Multiplies fresh balanced 16-limb integers through the backend basecase kernel. |
| `rational_algorithm_dispatch_speed/mul_backend_half_karatsuba_cold` | 528.33 ns | 524.28 ns - 532.27 ns | Multiplies fresh unbalanced 33-by-66-limb integers through half-Karatsuba. |
| `rational_algorithm_dispatch_speed/mul_backend_karatsuba_cold` | 825.48 ns | 823.14 ns - 827.85 ns | Multiplies fresh balanced 40-limb integers through Karatsuba. |
| `rational_algorithm_dispatch_speed/mul_backend_toom3_cold` | 8.334 us | 8.325 us - 8.344 us | Multiplies fresh balanced 257-limb integers through Toom-3. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_4096_bits` | 7.954 us | 7.943 us - 7.970 us | Runs Hyperreal's seven-product Rust-native Toom-4 candidate on balanced 4,096-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_4096_bits` | 2.773 us | 2.762 us - 2.785 us | Runs the native backend product on the same 4,096-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_16384_bits` | 34.733 us | 34.708 us - 34.761 us | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 16,384-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_16384_bits` | 26.348 us | 26.241 us - 26.540 us | Runs the native backend product on the same 16,384-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_65536_bits` | 225.549 us | 225.032 us - 226.239 us | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 65,536-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_65536_bits` | 202.522 us | 202.375 us - 202.698 us | Runs the native backend product on the same 65,536-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_262144_bits` | 1.622 ms | 1.620 ms - 1.623 ms | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 262,144-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_262144_bits` | 1.609 ms | 1.606 ms - 1.612 ms | Runs the native backend product on the same 262,144-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_524288_bits` | 4.539 ms | 4.533 ms - 4.547 ms | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 524,288-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_524288_bits` | 4.465 ms | 4.457 ms - 4.473 ms | Runs the native backend product on the same 524,288-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_1048576_bits` | 12.059 ms | 12.046 ms - 12.075 ms | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 1,048,576-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_1048576_bits` | 12.589 ms | 12.580 ms - 12.599 ms | Runs the native backend product on the same 1,048,576-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_2097152_bits` | 32.848 ms | 32.758 ms - 32.952 ms | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 2,097,152-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_2097152_bits` | 35.076 ms | 34.993 ms - 35.199 ms | Runs the native backend product on the same 2,097,152-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_1048576_bits` | 10.674 ms | 10.661 ms - 10.691 ms | Runs the retained production Toom-8 selector above its balanced crossover. |
| `rational_algorithm_dispatch_speed/mul_selected_2097152_bits` | 27.749 ms | 27.719 ms - 27.785 ms | Runs the retained production Toom-8 selector on balanced 2,097,152-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_1048576_bits` | 10.823 ms | 10.817 ms - 10.830 ms | Runs Hyperreal's eleven-product Rust-native Toom-6 candidate above its crossover. |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_131072_bits` | 587.647 us | 587.295 us - 588.039 us | Runs Hyperreal's Rust-native Toom-6 candidate on balanced 131,072-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_131072_bits` | 589.651 us | 588.274 us - 591.498 us | Runs the retained native backend selector on the same 131,072-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_262144_bits` | 1.568 ms | 1.567 ms - 1.570 ms | Runs Hyperreal's Rust-native Toom-6 candidate on balanced 262,144-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_524288_bits` | 4.113 ms | 4.107 ms - 4.120 ms | Runs Hyperreal's Rust-native Toom-6 candidate on balanced 524,288-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_524288_bits` | 3.947 ms | 3.940 ms - 3.957 ms | Runs the retained production Toom-8 selector above its balanced crossover. |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_2097152_bits` | 29.811 ms | 29.792 ms - 29.831 ms | Runs Hyperreal's Rust-native Toom-6 candidate on balanced 2,097,152-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_toom4_unbalanced_1258291_by_1048576` | 14.850 ms | 14.797 ms - 14.924 ms | Runs retained Toom-4 on a 6:5 operand pair outside Toom-6's balance band. |
| `rational_algorithm_dispatch_speed/mul_backend_unbalanced_1258291_by_1048576` | 16.263 ms | 16.203 ms - 16.331 ms | Runs the native backend on the same 6:5 operand pair. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_262144_bits` | 1.502 ms | 1.501 ms - 1.504 ms | Runs Hyperreal's fifteen-product Rust-native Toom-8 candidate on balanced 262,144-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_262144_bits` | 1.502 ms | 1.500 ms - 1.503 ms | Runs the retained production Toom-8 selector at its balanced crossover. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_65536_bits` | 254.385 us | 253.986 us - 254.838 us | Runs Hyperreal's Rust-native Toom-8 candidate on balanced 65,536-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_131072_bits` | 600.187 us | 599.821 us - 600.578 us | Runs Hyperreal's Rust-native Toom-8 candidate on balanced 131,072-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_524288_bits` | 3.939 ms | 3.935 ms - 3.944 ms | Runs Hyperreal's Rust-native Toom-8 candidate at the Toom-6 crossover. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_1048576_bits` | 10.672 ms | 10.657 ms - 10.689 ms | Runs Hyperreal's Rust-native Toom-8 candidate on balanced 1,048,576-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_2097152_bits` | 28.052 ms | 27.934 ms - 28.182 ms | Runs Hyperreal's Rust-native Toom-8 candidate on balanced 2,097,152-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_4194304_bits` | 73.226 ms | 73.167 ms - 73.293 ms | Runs Hyperreal's Rust-native Toom-8 candidate on balanced 4,194,304-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_4194304_bits` | 73.219 ms | 73.163 ms - 73.282 ms | Runs the retained production Toom-8 selector on the same 4,194,304-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_toom6_unbalanced_599186_by_524288` | 4.742 ms | 4.729 ms - 4.760 ms | Runs retained Toom-6 on an 8:7 operand pair outside Toom-8's balance band. |
| `rational_algorithm_dispatch_speed/mul_backend_unbalanced_599186_by_524288` | 5.335 ms | 5.323 ms - 5.348 ms | Runs the native backend on the same 8:7 operand pair. |
| `rational_algorithm_dispatch_speed/mul_ntt_candidate_262144_bits` | 16.193 ms | 16.163 ms - 16.224 ms | Runs Hyperreal's exact two-prime Rust-native NTT/CRT candidate on balanced 262,144-bit operands. |
| `rational_algorithm_dispatch_speed/mul_ntt_candidate_1048576_bits` | 72.858 ms | 72.756 ms - 72.962 ms | Runs the Rust-native NTT/CRT candidate on balanced 1,048,576-bit operands. |
| `rational_algorithm_dispatch_speed/mul_ntt_candidate_4194304_bits` | 327.418 ms | 327.140 ms - 327.718 ms | Runs the Rust-native NTT/CRT candidate on balanced 4,194,304-bit operands. |
| `rational_algorithm_dispatch_speed/reduce_backend_single_limb_cold` | 140.68 ns | 140.42 ns - 140.97 ns | Reduces a fresh wide fraction by a single-limb exact divisor. |
| `rational_algorithm_dispatch_speed/reduce_backend_knuth_cold` | 447.81 ns | 411.51 ns - 519.45 ns | Reduces a fresh wide fraction through normalized Knuth basecase division. |
| `rational_algorithm_dispatch_speed/reduce_backend_large_knuth_cold` | 102.945 us | 102.855 us - 103.057 us | Reduces a fresh 129-limb numerator by a 65-limb exact divisor through normalized Knuth division. |
| `rational_algorithm_dispatch_speed/reduce_fixed_512_coprime_cold` | 5.461 us | 5.434 us - 5.506 us | Reduces fresh balanced 512-bit operands through the fixed-limb rational-operation GCD. |
| `rational_algorithm_dispatch_speed/exact_remainder_large_knuth` | 5.034 us | 5.027 us - 5.042 us | Computes a wide rational fractional remainder through the traced normalized Knuth backend. |
| `rational_algorithm_dispatch_speed/division_trivial_small_quotient` | 98.88 ns | 98.83 ns - 98.97 ns | Exercises the backend's zero-quotient magnitude division exit on wide operands. |
| `rational_algorithm_dispatch_speed/gcd_selected_128_bits` | 141.37 ns | 141.10 ns - 141.70 ns | Runs selected magnitude GCD on an ascending balanced two-limb pair. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_128_bits` | 5.459 us | 5.454 us - 5.466 us | Runs the full-width Euclidean baseline on the same 128-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_192_bits` | 5.482 us | 5.467 us - 5.499 us | Runs selected magnitude GCD at the retained three-limb Lehmer crossover. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_192_bits` | 8.688 us | 8.673 us - 8.710 us | Runs the full-width Euclidean baseline on the same 192-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_512_bits` | 11.752 us | 11.744 us - 11.764 us | Runs selected magnitude GCD above the Lehmer crossover. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_512_bits` | 31.348 us | 31.301 us - 31.414 us | Runs the full-width Euclidean baseline on the same 512-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_1024_bits` | 23.034 us | 22.892 us - 23.196 us | Runs selected magnitude GCD above the Lehmer crossover. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_1024_bits` | 74.968 us | 74.900 us - 75.039 us | Runs the full-width Euclidean baseline on the same 1,024-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_4096_bits` | 121.032 us | 120.876 us - 121.227 us | Runs selected magnitude GCD well above the Lehmer crossover. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_4096_bits` | 487.740 us | 486.839 us - 488.740 us | Runs the full-width Euclidean baseline on the same 4,096-bit pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_8192_bits` | 313.666 us | 313.448 us - 313.903 us | Runs the recursive half-GCD candidate below its provisional crossover. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_8192_bits` | 315.217 us | 314.983 us - 315.513 us | Runs the quadratic Lehmer baseline on the same 8,192-bit pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_16384_bits` | 3.234 ms | 3.227 ms - 3.242 ms | Runs the recursive half-GCD candidate at its provisional crossover. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_16384_bits` | 926.757 us | 925.719 us - 927.990 us | Runs the quadratic Lehmer baseline on the same 16,384-bit pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_65536_bits` | 24.491 ms | 24.471 ms - 24.513 ms | Runs the recursive half-GCD candidate well above its provisional crossover. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_65536_bits` | 9.249 ms | 9.241 ms - 9.260 ms | Runs the quadratic Lehmer baseline on the same 65,536-bit pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_262144_bits` | 270.353 ms | 269.875 ms - 270.901 ms | Runs recursive half-GCD with selected higher-Toom matrix products at 262,144 bits. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_262144_bits` | 130.987 ms | 130.894 ms - 131.092 ms | Runs the Lehmer baseline on the same 262,144-bit pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_1048576_bits` | 3.839 s | 3.836 s - 3.842 s | Runs recursive half-GCD with selected higher-Toom matrix products at 1,048,576 bits. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_1048576_bits` | 2.032 s | 2.030 s - 2.035 s | Runs the Lehmer baseline on the same 1,048,576-bit pair. |
| `rational_algorithm_dispatch_speed/barrett_one_shot_8192_by_1024` | 5.702 us | 5.696 us - 5.710 us | Prepares a Rust-native Barrett reciprocal and divides one 8,192-bit value by a 1,024-bit divisor. |
| `rational_algorithm_dispatch_speed/backend_one_shot_8192_by_1024` | 2.794 us | 2.791 us - 2.796 us | Runs the native backend div-rem baseline for the same one-shot operands. |
| `rational_algorithm_dispatch_speed/barrett_batch16_8192_by_1024` | 85.383 us | 85.328 us - 85.446 us | Amortizes one Rust-native Barrett reciprocal over sixteen 8,192-bit dividends. |
| `rational_algorithm_dispatch_speed/backend_batch16_8192_by_1024` | 49.947 us | 49.696 us - 50.232 us | Runs sixteen native backend div-rem operations on the same values. |
| `rational_algorithm_dispatch_speed/barrett_batch16_65536_by_4096` | 1.476 ms | 1.473 ms - 1.481 ms | Amortizes one Rust-native Barrett reciprocal over sixteen 65,536-bit dividends. |
| `rational_algorithm_dispatch_speed/backend_batch16_65536_by_4096` | 1.213 ms | 1.212 ms - 1.214 ms | Runs sixteen native backend div-rem operations on the same large values. |
| `rational_algorithm_dispatch_speed/perfect_power_factor_reject` | 70.59 ns | 70.53 ns - 70.66 ns | Rejects 12 after small-factor multiplicities collapse to gcd one. |
| `rational_algorithm_dispatch_speed/perfect_power_general_seventh` | 1.609 us | 1.607 us - 1.610 us | Discovers an exact rational seventh power whose base primes exceed the trial table. |
| `rational_algorithm_dispatch_speed/perfect_power_fixed_seventh` | 209.64 ns | 209.42 ns - 209.88 ns | Checks the same value when the seventh-root degree is already known. |
| `rational_algorithm_dispatch_speed/perfect_power_unfactored_reject` | 3.281 us | 3.277 us - 3.286 us | Rejects mismatched seventh- and fifth-power rational components beyond the trial table. |
| `rational_algorithm_dispatch_speed/radix_format_small_integer` | 958.83 ns | 954.70 ns - 963.13 ns | Formats a 16-limb integer using repeated single-limb radix division. |
| `rational_algorithm_dispatch_speed/radix_format_large_integer` | 2.991 us | 2.985 us - 2.996 us | Formats a 32-limb integer using divide-and-conquer radix conversion. |
| `rational_algorithm_dispatch_speed/radix_parse_short_decimal` | 78.76 ns | 78.34 ns - 79.29 ns | Parses a short exact decimal through the checked word-sized path. |
| `rational_algorithm_dispatch_speed/radix_parse_large_integer` | 1.520 us | 1.519 us - 1.522 us | Parses a large below-threshold decimal fixture through chunked multiply-add conversion. |
| `rational_algorithm_dispatch_speed/radix_parse_divide_conquer_10240_digits` | 99.740 us | 99.653 us - 99.840 us | Parses 10,240 digits through the divide-and-conquer product tree. |
| `rational_algorithm_dispatch_speed/radix_parse_backend_chunked_10240_digits` | 105.045 us | 104.981 us - 105.126 us | Parses the same 10,240 digits with the backend chunked multiply-add baseline. |
| `rational_algorithm_dispatch_speed/radix_parse_divide_conquer_20480_digits` | 285.042 us | 284.223 us - 285.995 us | Parses 20,480 digits through the divide-and-conquer product tree. |
| `rational_algorithm_dispatch_speed/radix_parse_backend_chunked_20480_digits` | 389.109 us | 386.235 us - 392.546 us | Parses the same 20,480 digits with the backend chunked multiply-add baseline. |
| `rational_algorithm_dispatch_speed/radix_format_fraction_decimal` | 2.717 us | 2.712 us - 2.723 us | Formats a rational decimal through exact repeated digit division. |

### `borrowed_op_overhead`

Borrowed versus owned operation overhead for rational and real operands.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `borrowed_op_overhead/rational_clone_pair` | 7.29 ns | 7.28 ns - 7.31 ns | Clones two rational values. |
| `borrowed_op_overhead/rational_add_refs` | 8.28 ns | 8.26 ns - 8.33 ns | Adds rational references. |
| `borrowed_op_overhead/rational_add_owned` | 9.80 ns | 9.78 ns - 9.82 ns | Adds owned rational values. |
| `borrowed_op_overhead/real_clone_pair` | 83.18 ns | 83.08 ns - 83.31 ns | Clones two scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_unscaled_add_refs` | 52.18 ns | 52.09 ns - 52.28 ns | Adds borrowed unscaled transcendental `Real` values. |
| `borrowed_op_overhead/real_unscaled_add_owned` | 81.44 ns | 62.48 ns - 119.22 ns | Adds owned unscaled transcendental `Real` values. |
| `borrowed_op_overhead/real_add_refs` | 145.54 ns | 145.43 ns - 145.65 ns | Adds borrowed scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_add_owned` | 169.88 ns | 156.23 ns - 196.74 ns | Adds owned scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_dot2_refs_dense_symbolic` | 401.61 ns | 400.01 ns - 403.19 ns | Computes a borrowed two-lane symbolic dot product with no rational shortcut terms. |
| `borrowed_op_overhead/real_active_dot2_refs_dense_symbolic` | 398.16 ns | 397.27 ns - 399.13 ns | Computes a borrowed two-lane symbolic dot product after the caller has already classified every lane active. |
| `borrowed_op_overhead/real_dot2_refs_mixed_structural` | 51.32 ns | 51.15 ns - 51.58 ns | Computes a borrowed two-lane symbolic dot product with an exact zero lane and a rational scale lane. |
| `borrowed_op_overhead/real_dot3_refs_dense_symbolic` | 809.10 ns | 805.93 ns - 812.83 ns | Computes a borrowed three-lane symbolic dot product with no rational shortcut terms. |
| `borrowed_op_overhead/real_active_dot3_refs_dense_symbolic` | 794.87 ns | 793.68 ns - 796.14 ns | Computes a borrowed three-lane symbolic dot product after the caller has already classified every lane active. |
| `borrowed_op_overhead/real_dot3_refs_mixed_structural` | 170.79 ns | 170.43 ns - 171.20 ns | Computes a borrowed three-lane symbolic dot product with exact zero and rational scale terms. |
| `borrowed_op_overhead/real_dot4_refs_dense_symbolic` | 1.205 us | 1.201 us - 1.210 us | Computes a borrowed four-lane symbolic dot product with no rational shortcut terms. |
| `borrowed_op_overhead/real_active_dot4_refs_dense_symbolic` | 1.197 us | 1.194 us - 1.202 us | Computes a borrowed four-lane symbolic dot product after the caller has already classified every lane active. |
| `borrowed_op_overhead/real_dot4_refs_mixed_structural` | 216.29 ns | 215.74 ns - 216.97 ns | Computes a borrowed four-lane symbolic dot product with exact zero and rational scale terms. |

### `dense_algebra`

Small dense algebra kernels that stress repeated exact and symbolic operations.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `dense_algebra/rational_dot_64` | 1.474 us | 1.469 us - 1.479 us | Computes a 64-element rational dot product. |
| `dense_algebra/rational_matmul_8` | 58.570 us | 58.458 us - 58.716 us | Computes an 8x8 rational matrix multiply. |
| `dense_algebra/real_dot_36` | 3.741 us | 3.722 us - 3.763 us | Computes a 36-element dot product over symbolic `Real` values. |
| `dense_algebra/real_matmul_6` | 43.583 us | 43.438 us - 43.735 us | Computes a 6x6 matrix multiply over symbolic `Real` values. |
| `dense_algebra/real_sum_refs_64_symbolic` | 6.943 us | 6.927 us - 6.958 us | Constructs an arbitrary-length sum of 64 borrowed symbolic square roots. |
| `dense_algebra/real_sum_refs_64_symbolic_to_f64` | 34.701 us | 34.566 us - 34.864 us | Constructs and approximates the same arbitrary-length symbolic sum. |

### `exact_transcendental_special_forms`

Construction-time shortcuts for exact rational multiples of pi and inverse compositions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `exact_transcendental_special_forms/sin_pi_7` | 244.82 ns | 243.29 ns - 246.62 ns | Builds the exact special form for sin(pi/7). |
| `exact_transcendental_special_forms/cos_pi_7` | 122.32 ns | 121.74 ns - 122.98 ns | Builds the exact special form for cos(pi/7). |
| `exact_transcendental_special_forms/tan_pi_7` | 202.09 ns | 201.07 ns - 203.30 ns | Builds the exact special form for tan(pi/7). |
| `exact_transcendental_special_forms/asin_sin_6pi_7` | 427.96 ns | 425.99 ns - 430.18 ns | Recognizes the principal branch of asin(sin(6pi/7)). |
| `exact_transcendental_special_forms/acos_cos_9pi_7` | 424.00 ns | 422.88 ns - 425.15 ns | Recognizes the principal branch of acos(cos(9pi/7)). |
| `exact_transcendental_special_forms/atan_tan_6pi_7` | 494.82 ns | 493.21 ns - 496.63 ns | Recognizes the principal branch of atan(tan(6pi/7)). |
| `exact_transcendental_special_forms/asinh_large` | 126.86 ns | 126.39 ns - 127.43 ns | Builds a large inverse hyperbolic sine without exact intermediate Reals. |
| `exact_transcendental_special_forms/atanh_sqrt_half` | 109.17 ns | 108.67 ns - 109.68 ns | Builds atanh(sqrt(2)/2) after exact structural domain checks. |
| `exact_transcendental_special_forms/atanh_sqrt_two_error` | 56.68 ns | 55.94 ns - 57.42 ns | Rejects atanh(sqrt(2)) through exact structural domain checks. |
| `exact_transcendental_special_forms/sinh_ln_two` | 151.26 ns | 150.81 ns - 151.70 ns | Folds sinh(ln(2)) to the exact rational 3/4 via the integer-log-collapse shortcut. |
| `exact_transcendental_special_forms/cosh_ln_two` | 154.95 ns | 154.37 ns - 155.62 ns | Folds cosh(ln(2)) to the exact rational 5/4 via the integer-log-collapse shortcut. |
| `exact_transcendental_special_forms/tanh_ln_two` | 286.21 ns | 285.51 ns - 287.06 ns | Folds tanh(ln(2)) to the exact rational 3/5 via the integer-log-collapse shortcut. |
| `exact_transcendental_special_forms/sinh_rational_one` | 386.33 ns | 384.85 ns - 388.16 ns | Builds sinh(1) through the generic (exp(x) - exp(-x))/2 identity path. |
| `exact_transcendental_special_forms/cosh_rational_one` | 341.16 ns | 339.55 ns - 343.03 ns | Builds cosh(1) through the generic (exp(x) + exp(-x))/2 identity path. |
| `exact_transcendental_special_forms/tanh_rational_one` | 525.42 ns | 523.81 ns - 527.25 ns | Builds tanh(1) through the generic (exp(x) - exp(-x))/(exp(x) + exp(-x)) identity path. |
| `exact_transcendental_special_forms/atan2_origin` | 19.54 ns | 19.51 ns - 19.56 ns | Hits the origin (0, 0) short-circuit returning exact zero. |
| `exact_transcendental_special_forms/atan2_axis_positive_y` | 54.42 ns | 54.23 ns - 54.63 ns | Hits the positive-y axis short-circuit returning exact pi/2. |
| `exact_transcendental_special_forms/atan2_axis_negative_x` | 49.50 ns | 49.32 ns - 49.71 ns | Hits the negative-x axis short-circuit returning exact pi. |
| `exact_transcendental_special_forms/atan2_quadrant_one_unit_diagonal` | 103.41 ns | 103.21 ns - 103.62 ns | Quadrant I unit diagonal reduces to atan(1) = pi/4 exact special form. |
| `exact_transcendental_special_forms/atan2_quadrant_two_pi_correction` | 314.38 ns | 313.41 ns - 315.53 ns | Quadrant II (1, -2) exercises atan(small ratio) + pi correction. |
| `exact_transcendental_special_forms/atan2_quadrant_three_negative_pi` | 263.14 ns | 262.49 ns - 263.93 ns | Quadrant III (-1, -2) exercises atan(small ratio) - pi correction. |
| `exact_transcendental_special_forms/log2_power_of_two` | 69.46 ns | 69.24 ns - 69.72 ns | Folds log2(1024) to the exact rational 10 via the integer-log-detection shortcut. |
| `exact_transcendental_special_forms/log2_rational_three` | 110.57 ns | 110.14 ns - 111.12 ns | Builds log2(3) as a lightweight Log2 symbolic certificate. |
| `exact_transcendental_special_forms/log2_ln_quotient_fold` | 154.55 ns | 148.18 ns - 167.04 ns | Folds ln(5) / ln(2) into a Log2 certificate via the divide-recognize shortcut. |

### `symbolic_reductions`

Existing symbolic constant algebra cases considered for additional reductions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `symbolic_reductions/sqrt_pi_square` | 41.66 ns | 41.53 ns - 41.81 ns | Reduces sqrt(pi^2). |
| `symbolic_reductions/sqrt_pi_e_square` | 58.36 ns | 51.58 ns - 71.78 ns | Reduces sqrt((pi * e)^2). |
| `symbolic_reductions/ln_scaled_e` | 428.04 ns | 426.83 ns - 429.41 ns | Reduces ln(2 * e). |
| `symbolic_reductions/sub_pi_three` | 54.66 ns | 54.38 ns - 54.96 ns | Builds the certified pi - 3 constant-offset form. |
| `symbolic_reductions/pi_minus_three_facts` | 21.37 ns | 21.37 ns - 21.38 ns | Reads structural facts for the cached pi - 3 offset form. |
| `symbolic_reductions/div_exp_exp` | 316.05 ns | 315.61 ns - 316.52 ns | Reduces e^3 / e. |
| `symbolic_reductions/div_pi_square_e` | 264.44 ns | 263.74 ns - 265.36 ns | Reduces pi^2 / e. |
| `symbolic_reductions/div_const_products` | 459.54 ns | 455.70 ns - 462.92 ns | Reduces (pi^3 * e^5) / (pi * e^2). |
| `symbolic_reductions/inverse_pi` | 29.47 ns | 29.41 ns - 29.54 ns | Builds the reciprocal of pi. |
| `symbolic_reductions/div_one_pi` | 57.61 ns | 57.39 ns - 57.88 ns | Reduces 1 / pi. |
| `symbolic_reductions/div_rational_exp` | 150.25 ns | 149.82 ns - 150.76 ns | Reduces 2 / e. |
| `symbolic_reductions/div_e_pi` | 89.79 ns | 89.59 ns - 90.04 ns | Reduces e / pi. |
| `symbolic_reductions/mul_pi_inverse_pi` | 76.54 ns | 64.01 ns - 101.38 ns | Multiplies pi by its reciprocal. |
| `symbolic_reductions/mul_pi_e_sqrt_two` | 137.88 ns | 137.48 ns - 138.33 ns | Builds the factored pi * e * sqrt(2) form. |
| `symbolic_reductions/mul_const_product_sqrt_sqrt` | 273.98 ns | 244.71 ns - 330.00 ns | Cancels sqrt(2) from (pi * e * sqrt(2)) * sqrt(2). |
| `symbolic_reductions/div_const_product_sqrt_e` | 236.64 ns | 215.81 ns - 277.34 ns | Reduces (pi * e * sqrt(2)) / e. |
| `symbolic_reductions/inverse_const_product_sqrt` | 258.56 ns | 257.62 ns - 259.58 ns | Builds a rationalized reciprocal of pi * e * sqrt(2). |
| `symbolic_reductions/inverse_sqrt_two` | 23.43 ns | 23.40 ns - 23.47 ns | Builds the rationalized reciprocal of unit-scaled sqrt(2). |
| `symbolic_reductions/div_sqrt_two_sqrt_three` | 146.38 ns | 145.74 ns - 146.96 ns | Rationalizes a quotient of two unit-scaled square roots. |

### `exact_product_sums`

Fixed product-sum reducers used by determinant and cofactor kernels.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `exact_product_sums/signed_product_sum_lcm_6x2` | 348.95 ns | 348.77 ns - 349.14 ns | Computes an exact rational six-term signed product sum with mixed denominators. |
| `exact_product_sums/signed_product_sum_common_scale_6x2` | 208.63 ns | 208.49 ns - 208.79 ns | Computes an exact rational six-term signed product sum through the carried common-scale reducer. |
| `exact_product_sums/signed_product_sum_sparse_single_6x2` | 147.67 ns | 147.50 ns - 147.87 ns | Computes a sparse exact rational six-term signed product sum with one active product. |
| `exact_product_sums/real_signed_product_sum_rational_det3` | 314.17 ns | 312.74 ns - 315.87 ns | Computes a 3x3 determinant-shaped signed product sum through the public `Real` builder. |
| `exact_product_sums/real_signed_product_sum_mixed_symbolic_det3` | 1.737 us | 1.735 us - 1.740 us | Computes the same determinant-shaped builder with symbolic factors and rational scales. |

<!-- END scalar_micro -->

<!-- BEGIN library_perf -->
## `library_perf`

Library-level Criterion benchmarks for public `Rational`, `Real`, and `Simple` behavior.

### `real_format`

Formatting costs for important irrational `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_format/pi_lower_exp_32` | 4.796 us | 4.788 us - 4.803 us | Formats pi with 32 digits in lower-exponential form. |
| `real_format/pi_display_alt_32` | 5.005 us | 5.002 us - 5.009 us | Formats pi with alternate decimal display at 32 digits. |
| `real_format/sqrt_two_display_alt_32` | 4.770 us | 4.761 us - 4.781 us | Formats sqrt(2) with alternate decimal display at 32 digits. |

### `real_constants`

Construction cost for shared mathematical constants.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_constants/pi` | 36.30 ns | 36.25 ns - 36.37 ns | Constructs the symbolic pi value. |
| `real_constants/e` | 47.33 ns | 47.24 ns - 47.43 ns | Constructs the symbolic Euler constant value. |

### `simple`

Parser and evaluator costs for the `Simple` expression language.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple/parse_nested` | 411.99 ns | 411.06 ns - 413.13 ns | Parses a nested expression with powers, trig, and constants. |
| `simple/eval_nested` | 1.368 us | 1.364 us - 1.372 us | Evaluates a parsed mixed symbolic/numeric expression. |
| `simple/eval_constants` | 765.83 ns | 758.43 ns - 773.14 ns | Evaluates repeated built-in constants. |
| `simple/eval_exact` | 316.37 ns | 314.58 ns - 318.10 ns | Evaluates a rational-only expression through exact shortcuts. |
| `simple/eval_nested_exact` | 957.11 ns | 950.06 ns - 963.96 ns | Evaluates a nested rational-only expression through exact shortcuts. |

### `real_powi`

Integer exponentiation for exact and irrational `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_powi/exact_17` | 118.69 ns | 118.57 ns - 118.83 ns | Raises an exact rational-backed `Real` to the 17th power. |
| `real_powi/exact_17_i64` | 87.11 ns | 86.98 ns - 87.26 ns | Raises an exact rational-backed `Real` through the machine-sized exponent API. |
| `real_powi/irrational_17` | 162.99 ns | 162.83 ns - 163.16 ns | Raises sqrt(3) to the 17th power with symbolic simplification. |

### `rational_powi`

Integer exponentiation for `Rational`.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `rational_powi/exact_17` | 80.28 ns | 80.07 ns - 80.54 ns | Raises a rational value to the 17th power. |

### `real_exact_trig`

Exact and symbolic trig construction for known pi multiples.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_trig/sin_pi_6` | 100.52 ns | 100.10 ns - 100.93 ns | Computes sin(pi/6) via exact shortcut. |
| `real_exact_trig/cos_pi_3` | 50.52 ns | 50.18 ns - 50.88 ns | Computes cos(pi/3) via exact shortcut. |
| `real_exact_trig/tan_pi_5` | 209.47 ns | 208.97 ns - 210.28 ns | Builds tan(pi/5), a nontrivial symbolic tangent. |

### `real_general_trig`

General trig construction for irrational arguments.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_general_trig/tan_sqrt_2` | 851.89 ns | 818.81 ns - 915.92 ns | Builds tan(sqrt(2)). |
| `real_general_trig/tan_pi_sqrt_2_over_5` | 1.491 us | 1.431 us - 1.610 us | Builds tangent of an irrational multiple of pi. |

### `real_exact_inverse_trig`

Exact inverse trig shortcuts and symbolic inverse trig recognition.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_inverse_trig/asin_1_2` | 51.67 ns | 51.51 ns - 51.86 ns | Recognizes asin(1/2) as pi/6. |
| `real_exact_inverse_trig/asin_minus_1_2` | 66.35 ns | 66.26 ns - 66.46 ns | Recognizes asin(-1/2) as -pi/6. |
| `real_exact_inverse_trig/asin_sqrt_2_over_2` | 102.58 ns | 102.15 ns - 103.02 ns | Recognizes asin(sqrt(2)/2) as pi/4. |
| `real_exact_inverse_trig/asin_sin_pi_5` | 137.90 ns | 118.25 ns - 176.53 ns | Inverts a symbolic sin(pi/5). |
| `real_exact_inverse_trig/acos_1` | 26.50 ns | 26.46 ns - 26.56 ns | Recognizes acos(1) as zero. |
| `real_exact_inverse_trig/acos_minus_1` | 42.65 ns | 42.58 ns - 42.72 ns | Recognizes acos(-1) as pi. |
| `real_exact_inverse_trig/acos_1_2` | 53.57 ns | 53.39 ns - 53.76 ns | Recognizes acos(1/2) as pi/3. |
| `real_exact_inverse_trig/atan_1` | 45.90 ns | 45.63 ns - 46.24 ns | Recognizes atan(1) as pi/4. |
| `real_exact_inverse_trig/atan_sqrt_3_over_3` | 109.18 ns | 99.35 ns - 128.51 ns | Recognizes atan(sqrt(3)/3) as pi/6. |
| `real_exact_inverse_trig/atan_tan_pi_5` | 119.37 ns | 118.63 ns - 119.94 ns | Inverts a symbolic tan(pi/5). |

### `real_general_inverse_trig`

General inverse trig construction, domain errors, and atan range reduction.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_general_inverse_trig/asin_7_10` | 108.81 ns | 106.69 ns - 111.51 ns | Builds asin(7/10) through the rational-specialized path. |
| `real_general_inverse_trig/asin_near_one` | 104.40 ns | 103.94 ns - 104.93 ns | Builds a deferred exact-rational asin near the positive endpoint. |
| `real_general_inverse_trig/asin_near_minus_one` | 104.51 ns | 104.24 ns - 104.97 ns | Builds a deferred exact-rational asin near the negative endpoint. |
| `real_general_inverse_trig/asin_sqrt_2_over_3` | 260.27 ns | 252.84 ns - 274.79 ns | Builds asin(sqrt(2)/3) through the general path. |
| `real_general_inverse_trig/acos_7_10` | 119.23 ns | 118.62 ns - 119.96 ns | Builds acos(7/10) through the rational-specialized asin path. |
| `real_general_inverse_trig/acos_sqrt_2_over_3` | 212.36 ns | 204.12 ns - 227.47 ns | Builds acos(sqrt(2)/3) through the general path. |
| `real_general_inverse_trig/asin_11_10_error` | 26.09 ns | 26.07 ns - 26.11 ns | Rejects rational asin input outside [-1, 1]. |
| `real_general_inverse_trig/acos_11_10_error` | 24.91 ns | 24.89 ns - 24.94 ns | Rejects rational acos input outside [-1, 1]. |
| `real_general_inverse_trig/atan_8` | 121.86 ns | 121.46 ns - 122.43 ns | Builds atan(8), exercising large-argument reduction. |
| `real_general_inverse_trig/atan_sqrt_2` | 8.604 us | 8.494 us - 8.810 us | Builds atan(sqrt(2)). |

### `real_inverse_hyperbolic`

Inverse hyperbolic construction, exact exits, stable ln1p forms, and domain errors.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_inverse_hyperbolic/asinh_0` | 16.81 ns | 16.73 ns - 16.91 ns | Recognizes asinh(0) as zero. |
| `real_inverse_hyperbolic/asinh_1_2` | 112.96 ns | 112.73 ns - 113.24 ns | Builds asinh(1/2) through the stable moderate-input path. |
| `real_inverse_hyperbolic/asinh_sqrt_2` | 72.07 ns | 64.25 ns - 87.53 ns | Builds asinh(sqrt(2)) without cancellation-prone log construction. |
| `real_inverse_hyperbolic/asinh_minus_1_2` | 157.72 ns | 157.61 ns - 157.84 ns | Uses odd symmetry for negative asinh input. |
| `real_inverse_hyperbolic/asinh_1_000_000` | 133.43 ns | 124.68 ns - 150.81 ns | Builds asinh for a large positive rational. |
| `real_inverse_hyperbolic/acosh_1` | 19.00 ns | 18.96 ns - 19.06 ns | Recognizes acosh(1) as zero. |
| `real_inverse_hyperbolic/acosh_2` | 42.50 ns | 38.10 ns - 51.25 ns | Builds acosh(2) through the stable moderate-input path. |
| `real_inverse_hyperbolic/acosh_sqrt_2` | 131.19 ns | 121.75 ns - 149.73 ns | Builds acosh(sqrt(2)) through square-root domain specialization. |
| `real_inverse_hyperbolic/acosh_1_000_000` | 87.64 ns | 87.30 ns - 88.06 ns | Builds acosh for a large positive rational. |
| `real_inverse_hyperbolic/atanh_0` | 16.29 ns | 16.24 ns - 16.36 ns | Recognizes atanh(0) as zero. |
| `real_inverse_hyperbolic/atanh_1_2` | 52.87 ns | 52.49 ns - 53.33 ns | Builds exact-rational atanh(1/2). |
| `real_inverse_hyperbolic/atanh_minus_1_2` | 70.93 ns | 70.80 ns - 71.10 ns | Builds exact-rational atanh(-1/2). |
| `real_inverse_hyperbolic/atanh_sqrt_half` | 108.94 ns | 108.65 ns - 109.17 ns | Recognizes atanh(sqrt(2)/2) as asinh(1). |
| `real_inverse_hyperbolic/atanh_9_10` | 129.78 ns | 129.34 ns - 130.34 ns | Builds exact-rational atanh near the upper domain boundary. |
| `real_inverse_hyperbolic/atanh_1_error` | 9.49 ns | 9.48 ns - 9.51 ns | Rejects atanh(1) at the rational domain boundary. |

### `simple_inverse_functions`

Parsed/evaluated inverse trig and inverse hyperbolic expressions that should succeed.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple_inverse_functions/asin_1_2` | 82.78 ns | 82.55 ns - 82.98 ns | Evaluates `(asin 1/2)`. |
| `simple_inverse_functions/acos_1_2` | 85.87 ns | 85.61 ns - 86.10 ns | Evaluates `(acos 1/2)`. |
| `simple_inverse_functions/atan_1` | 80.26 ns | 79.75 ns - 80.87 ns | Evaluates `(atan 1)`. |
| `simple_inverse_functions/asin_general` | 142.49 ns | 142.23 ns - 142.75 ns | Evaluates `(asin 7/10)`. |
| `simple_inverse_functions/acos_general` | 156.93 ns | 156.53 ns - 157.40 ns | Evaluates `(acos 7/10)`. |
| `simple_inverse_functions/atan_general` | 159.09 ns | 158.54 ns - 159.76 ns | Evaluates `(atan 8)`. |
| `simple_inverse_functions/asinh_1_2` | 144.87 ns | 144.52 ns - 145.22 ns | Evaluates `(asinh 1/2)`. |
| `simple_inverse_functions/asinh_sqrt_2` | 194.72 ns | 193.57 ns - 196.02 ns | Evaluates `(asinh (sqrt 2))`. |
| `simple_inverse_functions/acosh_2` | 68.99 ns | 68.71 ns - 69.25 ns | Evaluates `(acosh 2)`. |
| `simple_inverse_functions/acosh_sqrt_2` | 232.55 ns | 231.73 ns - 233.53 ns | Evaluates `(acosh (sqrt 2))`. |
| `simple_inverse_functions/atanh_1_2` | 83.47 ns | 83.09 ns - 83.84 ns | Evaluates `(atanh 1/2)`. |
| `simple_inverse_functions/atanh_minus_1_2` | 98.26 ns | 97.99 ns - 98.52 ns | Evaluates `(atanh -1/2)`. |

### `simple_inverse_error_functions`

Parsed/evaluated inverse function expressions that should fail quickly with `NotANumber`.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple_inverse_error_functions/asin_11_10` | 56.33 ns | 56.12 ns - 56.51 ns | Rejects `(asin 11/10)`. |
| `simple_inverse_error_functions/acos_sqrt_2` | 247.92 ns | 247.47 ns - 248.33 ns | Rejects `(acos (sqrt 2))`. |
| `simple_inverse_error_functions/acosh_0` | 37.41 ns | 37.26 ns - 37.57 ns | Rejects `(acosh 0)`. |
| `simple_inverse_error_functions/acosh_minus_2` | 37.75 ns | 37.53 ns - 37.98 ns | Rejects `(acosh -2)`. |
| `simple_inverse_error_functions/atanh_1` | 41.54 ns | 41.35 ns - 41.74 ns | Rejects `(atanh 1)`. |
| `simple_inverse_error_functions/atanh_sqrt_2` | 146.10 ns | 145.73 ns - 146.42 ns | Rejects `(atanh (sqrt 2))`. |

### `real_exact_ln`

Exact logarithm construction and simplification for rational inputs.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_ln/ln_1024` | 92.24 ns | 92.01 ns - 92.51 ns | Recognizes ln(1024) as 10 ln(2). |
| `real_exact_ln/ln_1_8` | 90.61 ns | 90.52 ns - 90.71 ns | Recognizes ln(1/8) as -3 ln(2). |
| `real_exact_ln/ln_1000` | 66.04 ns | 65.95 ns - 66.13 ns | Simplifies ln(1000) via small integer logarithm factors. |

### `real_exact_exp_log10`

Exact inverse relationships among exp, ln, and log10.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_exp_log10/exp_ln_1000` | 67.18 ns | 66.92 ns - 67.41 ns | Simplifies exp(ln(1000)) back to 1000. |
| `real_exact_exp_log10/exp_ln_1_8` | 78.10 ns | 77.78 ns - 78.45 ns | Simplifies exp(ln(1/8)) back to 1/8. |
| `real_exact_exp_log10/log10_1000` | 33.08 ns | 33.03 ns - 33.13 ns | Recognizes log10(1000) as 3. |
| `real_exact_exp_log10/log10_1_1000` | 63.82 ns | 63.75 ns - 63.90 ns | Recognizes log10(1/1000) as -3. |

### `real_stable_scalar_substrate`

Stable scalar constructors that preserve small residuals, dominance, roots, rational powers, and certified integer decisions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_stable_scalar_substrate/ln_1p_tiny` | 47.57 ns | 47.47 ns - 47.68 ns | Builds ln(1 + tiny) without first adding one generically. |
| `real_stable_scalar_substrate/ln_1m_tiny` | 53.14 ns | 53.02 ns - 53.28 ns | Builds ln(1 - tiny) through the log1p companion path. |
| `real_stable_scalar_substrate/expm1_tiny` | 88.23 ns | 88.15 ns - 88.32 ns | Builds exp(tiny) - 1 through the dedicated expm1 node. |
| `real_stable_scalar_substrate/softplus_large_positive` | 1.946 us | 1.941 us - 1.953 us | Builds softplus for a dominant positive input. |
| `real_stable_scalar_substrate/softplus_large_negative` | 1.847 us | 1.832 us - 1.866 us | Builds softplus for a dominant negative input. |
| `real_stable_scalar_substrate/logaddexp_dominant` | 2.093 us | 2.085 us - 2.104 us | Builds logaddexp when one side is certifiably dominant. |
| `real_stable_scalar_substrate/logsubexp_near` | 247.94 ns | 247.43 ns - 248.48 ns | Builds logsubexp for a certifiably positive but small log-space difference. |
| `real_stable_scalar_substrate/sigmoid_large_positive` | 1.885 us | 1.882 us - 1.888 us | Builds a large positive sigmoid through the stable tail path. |
| `real_stable_scalar_substrate/logit_near_one` | 298.77 ns | 298.10 ns - 299.54 ns | Builds logit close to the upper probability boundary. |
| `real_stable_scalar_substrate/sqrt1pm1_tiny` | 557.35 ns | 556.43 ns - 558.55 ns | Builds sqrt(1 + tiny) - 1 through the stable helper. |
| `real_stable_scalar_substrate/sqrt1m1_tiny` | 600.15 ns | 598.38 ns - 602.36 ns | Builds sqrt(1 - tiny) - 1 through the stable helper. |
| `real_stable_scalar_substrate/cbrt_negative_perfect` | 148.55 ns | 147.81 ns - 149.40 ns | Collapses a negative perfect cube. |
| `real_stable_scalar_substrate/root_n_perfect_fourth` | 153.46 ns | 153.16 ns - 153.78 ns | Collapses an exact fourth root. |
| `real_stable_scalar_substrate/pow_rational_negative_odd_denominator` | 197.36 ns | 196.99 ns - 197.76 ns | Routes a negative rational base through odd-root symmetry. |
| `real_stable_scalar_substrate/floor_certified_rational` | 77.72 ns | 77.58 ns - 77.88 ns | Certifies rational floor structurally. |
| `real_stable_scalar_substrate/rem_euclid_certified_rational` | 362.15 ns | 361.03 ns - 363.41 ns | Computes rational Euclidean remainder through certified quotient floor. |

### `real_geometry_polynomial_substrate`

Geometry-facing scalar helpers for rational-turn trig, removable small-angle limits, vectors, product sums, and polynomial forms.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_geometry_polynomial_substrate/sin_pi_one_sixth` | 83.72 ns | 83.61 ns - 83.83 ns | Uses exact rational-turn sine. |
| `real_geometry_polynomial_substrate/cos_pi_one_fourth` | 102.12 ns | 101.97 ns - 102.31 ns | Uses exact rational-turn cosine. |
| `real_geometry_polynomial_substrate/cos_pi_one_seventh` | 125.15 ns | 124.27 ns - 126.18 ns | Builds a non-tabulated rational-turn cosine certificate. |
| `real_geometry_polynomial_substrate/tan_pi_one_third` | 97.14 ns | 97.01 ns - 97.32 ns | Uses exact rational-turn tangent. |
| `real_geometry_polynomial_substrate/sinc_zero` | 16.46 ns | 16.44 ns - 16.49 ns | Returns the removable sinc limit at zero. |
| `real_geometry_polynomial_substrate/sinc_tiny` | 175.47 ns | 175.14 ns - 175.90 ns | Builds sinc for a tiny exact input. |
| `real_geometry_polynomial_substrate/sinc_pi_half` | 198.32 ns | 198.07 ns - 198.76 ns | Builds normalized sinc for an exact half turn. |
| `real_geometry_polynomial_substrate/cosc_tiny` | 335.96 ns | 335.27 ns - 336.76 ns | Builds the small-angle (1 - cos x) / x^2 helper. |
| `real_geometry_polynomial_substrate/atan2_axis` | 48.71 ns | 48.64 ns - 48.79 ns | Classifies an axis-aligned atan2 input exactly. |
| `real_geometry_polynomial_substrate/atan2_quadrant` | 212.47 ns | 212.16 ns - 212.81 ns | Builds a quadrant-correct atan2 expression. |
| `real_geometry_polynomial_substrate/hypot2_3_4` | 81.09 ns | 80.91 ns - 81.30 ns | Collapses a 3-4-5 norm through exact dot products. |
| `real_geometry_polynomial_substrate/hypot3_2_3_6` | 141.52 ns | 141.25 ns - 141.81 ns | Collapses a 2-3-6 norm through exact dot products. |
| `real_geometry_polynomial_substrate/hypot_minus_tiny` | 1.867 us | 1.863 us - 1.871 us | Uses rationalized hypot-minus for cancellation resistance. |
| `real_geometry_polynomial_substrate/mul_add_zero_product` | 83.71 ns | 63.26 ns - 124.53 ns | Skips a known-zero product lane. |
| `real_geometry_polynomial_substrate/sum_products_dense` | 1.391 us | 1.389 us - 1.393 us | Builds a dense product sum. |
| `real_geometry_polynomial_substrate/diff_of_products_near_cancel` | 311.08 ns | 310.87 ns - 311.32 ns | Preserves determinant-like product difference structure. |
| `real_geometry_polynomial_substrate/eval_poly_horner` | 1.129 us | 1.125 us - 1.133 us | Evaluates a polynomial through Horner form. |
| `real_geometry_polynomial_substrate/eval_rational_poly` | 1.471 us | 1.455 us - 1.501 us | Evaluates numerator and denominator polynomial forms before division. |

### `real_normal_scientific_substrate`

Gaussian tail helpers and exact/finite scientific special-function forms added for higher numerical workloads.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_normal_scientific_substrate/erfc_zero` | 11.61 ns | 11.58 ns - 11.65 ns | Takes the exact erfc(0) exit. |
| `real_normal_scientific_substrate/erfcx_tail` | 534.77 ns | 525.24 ns - 553.27 ns | Builds scaled erfc in a positive tail. |
| `real_normal_scientific_substrate/normal_sf_tail` | 239.47 ns | 228.83 ns - 260.30 ns | Builds standard-normal upper-tail probability. |
| `real_normal_scientific_substrate/pnorm_upper_tail` | 228.76 ns | 228.27 ns - 229.32 ns | Builds the upper-tail alias. |
| `real_normal_scientific_substrate/log_pnorm_tail` | 184.94 ns | 178.25 ns - 197.26 ns | Builds lower log-CDF tail form. |
| `real_normal_scientific_substrate/log_pnorm_zero` | 68.45 ns | 68.26 ns - 68.68 ns | Takes the exact log-CDF value at zero. |
| `real_normal_scientific_substrate/log_normal_sf_tail` | 199.82 ns | 193.37 ns - 212.56 ns | Builds upper log-survival tail form. |
| `real_normal_scientific_substrate/log_normal_sf_zero` | 67.85 ns | 67.66 ns - 68.15 ns | Takes the exact log-survival value at zero. |
| `real_normal_scientific_substrate/log_dnorm_large` | 74.56 ns | 74.40 ns - 74.80 ns | Builds analytic log-density at a large input. |
| `real_normal_scientific_substrate/normal_interval_narrow` | 554.26 ns | 553.33 ns - 555.39 ns | Builds a narrow interval mass without spelling pnorm subtraction. |
| `real_normal_scientific_substrate/erfinv_mid` | 1.466 us | 1.463 us - 1.469 us | Builds inverse error function through qnorm transform. |
| `real_normal_scientific_substrate/erfcinv_tail` | 1.496 us | 1.493 us - 1.499 us | Builds inverse complementary error function through tail qnorm transform. |
| `real_normal_scientific_substrate/qnorm_upper_tail` | 991.07 ns | 919.76 ns - 1.131 us | Builds inverse survival quantile. |
| `real_normal_scientific_substrate/normal_pdf_parametric` | 651.34 ns | 649.38 ns - 653.63 ns | Standardizes exactly before density construction. |
| `real_normal_scientific_substrate/normal_survival_parametric` | 358.47 ns | 358.06 ns - 358.98 ns | Standardizes exactly before upper-tail construction. |
| `real_normal_scientific_substrate/normal_mills_tail` | 2.119 us | 2.108 us - 2.137 us | Builds Mills ratio through erfcx identity. |
| `real_normal_scientific_substrate/normal_mills_zero` | 21.19 ns | 21.16 ns - 21.22 ns | Takes the exact Mills ratio value at zero. |
| `real_normal_scientific_substrate/normal_hazard_tail` | 2.207 us | 2.196 us - 2.224 us | Builds reciprocal Mills hazard. |
| `real_normal_scientific_substrate/normal_hazard_zero` | 20.92 ns | 20.89 ns - 20.95 ns | Takes the exact hazard value at zero. |
| `real_normal_scientific_substrate/normal_inverse_mills_zero` | 20.90 ns | 20.87 ns - 20.93 ns | Takes the exact lower inverse Mills value at zero. |
| `real_normal_scientific_substrate/hermite_8` | 1.295 us | 1.294 us - 1.297 us | Builds an exact probabilists' Hermite polynomial. |
| `real_normal_scientific_substrate/dnorm_derivative_4` | 1.142 us | 1.135 us - 1.156 us | Combines exact Hermite polynomial with normal density. |
| `real_normal_scientific_substrate/standard_normal_moment_12` | 152.47 ns | 152.30 ns - 152.66 ns | Uses double-factorial closed form. |
| `real_normal_scientific_substrate/normal_interval_moment_3` | 1.216 us | 1.214 us - 1.219 us | Uses interval mass and density-boundary recurrence. |
| `real_normal_scientific_substrate/truncated_normal_mean` | 1.138 us | 1.136 us - 1.139 us | Builds truncated-normal mean from stable interval mass. |
| `real_normal_scientific_substrate/gamma_integer` | 224.20 ns | 222.79 ns - 226.06 ns | Uses exact integer gamma closed form. |
| `real_normal_scientific_substrate/gamma_half_integer` | 325.82 ns | 325.57 ns - 326.14 ns | Uses exact half-integer gamma closed form. |
| `real_normal_scientific_substrate/lgamma_half_integer` | 1.513 us | 1.509 us - 1.518 us | Logs the absolute half-integer gamma value. |
| `real_normal_scientific_substrate/beta_integer` | 317.42 ns | 316.86 ns - 318.08 ns | Builds integer beta through an exact factorial ratio. |
| `real_normal_scientific_substrate/ln_beta_half_integer` | 2.894 us | 2.890 us - 2.897 us | Builds log beta through lgamma sum. |
| `real_normal_scientific_substrate/regularized_beta_mid` | 1.253 us | 1.251 us - 1.254 us | Uses finite positive-integer beta binomial tail. |
| `real_normal_scientific_substrate/regularized_beta_uniform` | 132.20 ns | 132.08 ns - 132.33 ns | Takes the exact I_x(1, 1) identity. |
| `real_normal_scientific_substrate/regularized_beta_left_unity` | 300.59 ns | 298.73 ns - 302.81 ns | Reduces I_x(1, b) to one complement power. |
| `real_normal_scientific_substrate/regularized_beta_q_mid` | 906.81 ns | 905.03 ns - 909.07 ns | Uses finite positive-integer beta upper-tail form. |
| `real_normal_scientific_substrate/regularized_beta_q_uniform` | 157.52 ns | 146.50 ns - 179.28 ns | Takes the exact upper-tail I_x(1, 1) complement. |
| `real_normal_scientific_substrate/regularized_beta_q_left_unity` | 207.81 ns | 207.42 ns - 208.25 ns | Reduces the upper beta tail for a = 1 to one power. |
| `real_normal_scientific_substrate/regularized_gamma_p_half` | 1.203 us | 1.190 us - 1.228 us | Uses half-integer incomplete-gamma recurrence. |
| `real_normal_scientific_substrate/regularized_gamma_q_integer` | 1.124 us | 1.122 us - 1.125 us | Uses integer incomplete-gamma recurrence. |
| `real_normal_scientific_substrate/chi_square_sf` | 1.910 us | 1.901 us - 1.923 us | Wraps regularized upper gamma for chi-square upper tail. |

### `simple_new_function_surface`

Parser and evaluator coverage for the newly exposed stable scalar, geometry, normal, and scientific functions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple_new_function_surface/stable_log_exp_bundle` | 7.922 us | 7.816 us - 8.082 us | Evaluates log1p/log1m/expm1/softplus/logaddexp/logsubexp/sigmoid/logit together. |
| `simple_new_function_surface/geometry_bundle` | 9.266 us | 9.197 us - 9.362 us | Evaluates rational-turn trig, small-angle helpers, vector norms, product sums, and polynomials together. |
| `simple_new_function_surface/normal_bundle` | 21.603 us | 21.518 us - 21.698 us | Evaluates normal tails, log tails, interval mass, inverse tails, and moments together. |
| `simple_new_function_surface/scientific_bundle` | 14.872 us | 14.823 us - 14.932 us | Evaluates gamma, beta, regularized gamma/beta, and chi-square forms together. |
| `simple_new_function_surface/error_bundle` | 176.76 ns | 173.60 ns - 179.62 ns | Exercises fast domain failures for new public functions. |

<!-- END library_perf -->

<!-- BEGIN adversarial_transcendentals -->
## `adversarial_transcendentals`

Adversarial transcendental benchmarks for `hyperreal` trig, inverse trig, and inverse hyperbolic construction and approximation paths.

### `trig_adversarial_approx`

Cold approximation of sine, cosine, and tangent at exact, tiny, huge, and near-singular arguments.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `trig_adversarial_approx/sin_tiny_rational_p96` | 388.12 ns | 384.26 ns - 393.28 ns | Approximates sin(1e-12), stressing direct tiny-argument setup. |
| `trig_adversarial_approx/cos_tiny_rational_p96` | 431.97 ns | 429.63 ns - 434.50 ns | Approximates cos(1e-12), stressing direct tiny-argument setup. |
| `trig_adversarial_approx/tan_tiny_rational_p96` | 1.663 us | 1.644 us - 1.695 us | Approximates tan(1e-12), stressing direct tiny-argument setup. |
| `trig_adversarial_approx/sin_medium_rational_p96` | 1.610 us | 1.578 us - 1.649 us | Approximates sin(7/5), a moderate non-pi rational. |
| `trig_adversarial_approx/cos_medium_rational_p96` | 1.500 us | 1.495 us - 1.505 us | Approximates cos(7/5), a moderate non-pi rational. |
| `trig_adversarial_approx/tan_medium_rational_p96` | 6.053 us | 6.037 us - 6.073 us | Approximates tan(7/5), a moderate non-pi rational. |
| `trig_adversarial_approx/sin_f64_exact_p96` | 1.777 us | 1.773 us - 1.783 us | Approximates sin(1.23456789 imported as an exact dyadic rational). |
| `trig_adversarial_approx/cos_f64_exact_p96` | 1.760 us | 1.753 us - 1.768 us | Approximates cos(1.23456789 imported as an exact dyadic rational). |
| `trig_adversarial_approx/sin_1e6_p96` | 2.386 us | 2.348 us - 2.435 us | Approximates sin(1000000), stressing integer argument reduction. |
| `trig_adversarial_approx/cos_1e6_p96` | 2.308 us | 2.304 us - 2.315 us | Approximates cos(1000000), stressing integer argument reduction. |
| `trig_adversarial_approx/tan_1e6_p96` | 8.214 us | 8.196 us - 8.229 us | Approximates tan(1000000), stressing integer argument reduction. |
| `trig_adversarial_approx/sin_1e30_p96` | 2.139 us | 2.119 us - 2.173 us | Approximates sin(10^30), stressing very large integer reduction. |
| `trig_adversarial_approx/cos_1e30_p96` | 2.238 us | 2.228 us - 2.254 us | Approximates cos(10^30), stressing very large integer reduction. |
| `trig_adversarial_approx/tan_1e30_p96` | 7.007 us | 6.956 us - 7.064 us | Approximates tan(10^30), stressing very large integer reduction. |
| `trig_adversarial_approx/sin_huge_pi_plus_offset_p96` | 1.859 us | 1.838 us - 1.884 us | Approximates sin(2^512*pi + 7/5), stressing exact pi-multiple cancellation. |
| `trig_adversarial_approx/cos_huge_pi_plus_offset_p96` | 1.764 us | 1.752 us - 1.784 us | Approximates cos(2^512*pi + 7/5), stressing exact pi-multiple cancellation. |
| `trig_adversarial_approx/tan_huge_pi_plus_offset_p96` | 6.274 us | 6.258 us - 6.291 us | Approximates tan(2^512*pi + 7/5), stressing exact pi-multiple cancellation. |
| `trig_adversarial_approx/tan_near_half_pi_p96` | 19.616 us | 19.421 us - 19.904 us | Approximates tan(pi/2 - 2^-40), stressing the cotangent complement path. |
| `trig_adversarial_approx/tan_promoted_generated_604_125_p96` | 6.916 us | 6.875 us - 6.965 us | Promoted slow-performer tan(604/125), a generated top offender from the library-wide fuzz history. |

### `inverse_trig_adversarial_approx`

Cold approximation of asin, acos, and atan near exact values, zero, endpoints, and large atan inputs.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `inverse_trig_adversarial_approx/asin_zero_p96` | 36.75 ns | 36.63 ns - 36.93 ns | Approximates asin(0), which should collapse before the generic inverse-trig path. |
| `inverse_trig_adversarial_approx/acos_zero_p96` | 192.53 ns | 192.26 ns - 192.85 ns | Approximates acos(0), which should reduce to pi/2. |
| `inverse_trig_adversarial_approx/atan_zero_p96` | 38.38 ns | 38.29 ns - 38.48 ns | Approximates atan(0), which should collapse to zero. |
| `inverse_trig_adversarial_approx/asin_tiny_positive_p96` | 398.10 ns | 396.27 ns - 400.30 ns | Approximates asin(1e-12), stressing the tiny odd series. |
| `inverse_trig_adversarial_approx/acos_tiny_positive_p96` | 772.74 ns | 759.45 ns - 787.18 ns | Approximates acos(1e-12), stressing pi/2 minus the tiny asin path. |
| `inverse_trig_adversarial_approx/atan_tiny_positive_p96` | 303.34 ns | 302.27 ns - 304.39 ns | Approximates atan(1e-12), stressing direct tiny atan setup. |
| `inverse_trig_adversarial_approx/asin_mid_positive_p96` | 6.183 us | 6.158 us - 6.216 us | Approximates asin(7/10), a generic in-domain value. |
| `inverse_trig_adversarial_approx/acos_mid_positive_p96` | 5.625 us | 5.613 us - 5.640 us | Approximates acos(7/10), a generic in-domain value. |
| `inverse_trig_adversarial_approx/atan_mid_positive_p96` | 1.876 us | 1.872 us - 1.879 us | Approximates atan(7/10), a generic in-domain value. |
| `inverse_trig_adversarial_approx/atan_two_thirds_anchor_sweep_p96` | 8.838 us | 8.826 us - 8.851 us | Approximates atan at 11/20, 3/5, 7/10, and 4/5, covering the two-thirds table-reduction interval. |
| `inverse_trig_adversarial_approx/atan_two_thirds_anchor_sweep_p32` | 5.123 us | 5.102 us - 5.143 us | Repeats the two-thirds table-reduction interval sweep at 32-bit precision. |
| `inverse_trig_adversarial_approx/atan_two_thirds_anchor_sweep_p256` | 19.756 us | 19.694 us - 19.859 us | Repeats the two-thirds table-reduction interval sweep at 256-bit precision. |
| `inverse_trig_adversarial_approx/atan_two_thirds_anchor_upper_edge_p96` | 2.459 us | 2.456 us - 2.463 us | Approximates atan(4/5), guarding the upper edge of the two-thirds table-reduction interval against a local regression. |
| `inverse_trig_adversarial_approx/asin_near_one_p96` | 1.891 us | 1.879 us - 1.902 us | Approximates asin(0.999999), stressing endpoint transforms. |
| `inverse_trig_adversarial_approx/acos_near_one_p96` | 1.587 us | 1.583 us - 1.592 us | Approximates acos(0.999999), stressing endpoint transforms. |
| `inverse_trig_adversarial_approx/asin_near_minus_one_p96` | 1.867 us | 1.849 us - 1.893 us | Approximates asin(-0.999999), stressing odd symmetry near the endpoint. |
| `inverse_trig_adversarial_approx/acos_near_minus_one_p96` | 1.652 us | 1.645 us - 1.659 us | Approximates acos(-0.999999), stressing negative endpoint transforms. |
| `inverse_trig_adversarial_approx/atan_large_p96` | 1.624 us | 1.606 us - 1.648 us | Approximates atan(8), stressing reciprocal reduction. |
| `inverse_trig_adversarial_approx/atan_promoted_generated_783_412_p96` | 1.816 us | 1.788 us - 1.853 us | Promoted slow-performer atan(783/412), the generated exact-rational atan top offender. |
| `inverse_trig_adversarial_approx/ln_square_plus_one_promoted_generated_677_222_p96` | 30.35 ns | 29.08 ns - 32.48 ns | Promoted slow-performer ln((677/222)^2 + 1), the generated exact-rational log top offender. |
| `inverse_trig_adversarial_approx/atan_huge_p96` | 632.86 ns | 627.98 ns - 640.09 ns | Approximates atan(10^30), stressing very large reciprocal reduction. |

### `trig_fuzz_adversarial_approx`

Deterministic broad sweeps of sine, cosine, and tangent over tiny, ordinary, huge, pi-offset, and near-pole exact inputs.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `trig_fuzz_adversarial_approx/sin_sweep_768_p96` | 1.495 ms | 1.489 ms - 1.501 ms | Approximates sin over 768 deterministic exact inputs spanning tiny, ordinary, huge, dyadic, rational, and pi-offset cases. |
| `trig_fuzz_adversarial_approx/cos_sweep_768_p96` | 1.538 ms | 1.521 ms - 1.567 ms | Approximates cos over the same 768-input deterministic fuzz sweep. |
| `trig_fuzz_adversarial_approx/tan_sweep_768_p96` | 4.669 ms | 4.628 ms - 4.711 ms | Approximates tan over the same deterministic sweep, including near-half-pi stress cases. |
| `trig_fuzz_adversarial_approx/sin_promoted_slow_candidates_p96` | 14.735 us | 14.717 us - 14.755 us | Approximates sin over promoted slow candidates found by prior sweep-style runs. |
| `trig_fuzz_adversarial_approx/cos_promoted_slow_candidates_p96` | 15.429 us | 15.403 us - 15.459 us | Approximates cos over promoted slow candidates found by prior sweep-style runs. |
| `trig_fuzz_adversarial_approx/tan_promoted_slow_candidates_p96` | 80.997 us | 79.770 us - 82.505 us | Approximates tan over promoted near-pole and large-reduction slow candidates. |

### `promoted_library_slow_offenders_approx`

Fifty structurally varied worst offenders promoted from the library-wide slow-performer history.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `promoted_library_slow_offenders_approx/promoted_50_structural_slow_offenders_p96` | not run | not run | Approximates 50 individual promoted slow cases spanning ln(1+x^2), atan, tan, sin, and cos over varied exact-rational structures. |

### `inverse_hyperbolic_adversarial_approx`

Cold approximation of inverse hyperbolic functions at tiny, moderate, large, and endpoint-adjacent arguments.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `inverse_hyperbolic_adversarial_approx/asinh_tiny_positive_p128` | 493.00 ns | 491.74 ns - 494.48 ns | Approximates asinh(1e-12), stressing cancellation avoidance near zero. |
| `inverse_hyperbolic_adversarial_approx/asinh_mid_positive_p128` | 6.996 us | 6.962 us - 7.042 us | Approximates asinh(1/2), a moderate positive value. |
| `inverse_hyperbolic_adversarial_approx/asinh_large_positive_p128` | 5.938 us | 5.898 us - 5.991 us | Approximates asinh(10^6), stressing large-input logarithmic behavior. |
| `inverse_hyperbolic_adversarial_approx/asinh_large_negative_p128` | 6.164 us | 6.117 us - 6.223 us | Approximates asinh(-10^6), stressing odd symmetry for large inputs. |
| `inverse_hyperbolic_adversarial_approx/acosh_one_plus_tiny_p128` | 3.622 us | 3.613 us - 3.632 us | Approximates acosh(1 + 1e-12), stressing the near-one endpoint. |
| `inverse_hyperbolic_adversarial_approx/acosh_sqrt_two_p128` | 88.17 ns | 86.97 ns - 89.75 ns | Approximates acosh(sqrt(2)), a symbolic square-root input. |
| `inverse_hyperbolic_adversarial_approx/acosh_two_p128` | 45.48 ns | 45.35 ns - 45.62 ns | Approximates acosh(2), a moderate exact rational. |
| `inverse_hyperbolic_adversarial_approx/acosh_large_positive_p128` | 5.769 us | 5.755 us - 5.784 us | Approximates acosh(10^6), stressing large-input logarithmic behavior. |
| `inverse_hyperbolic_adversarial_approx/atanh_tiny_positive_p128` | 510.95 ns | 507.60 ns - 515.19 ns | Approximates atanh(1e-12), stressing the tiny odd series. |
| `inverse_hyperbolic_adversarial_approx/atanh_mid_positive_p128` | 163.78 ns | 161.80 ns - 165.90 ns | Approximates atanh(1/2), a moderate exact rational. |
| `inverse_hyperbolic_adversarial_approx/atanh_near_one_p128` | 3.102 us | 3.076 us - 3.132 us | Approximates atanh(0.999999), stressing endpoint logarithmic behavior. |
| `inverse_hyperbolic_adversarial_approx/atanh_near_minus_one_p128` | 3.138 us | 3.130 us - 3.145 us | Approximates atanh(-0.999999), stressing odd symmetry near the endpoint. |

### `real_shortcut_adversarial`

Public `Real` construction shortcuts and domain checks for the same transcendental families.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_shortcut_adversarial/sin_exact_pi_over_six` | 105.42 ns | 104.04 ns - 106.68 ns | Constructs sin(pi/6), which should return the exact rational 1/2. |
| `real_shortcut_adversarial/cos_exact_pi_over_three` | 47.95 ns | 47.59 ns - 48.34 ns | Constructs cos(pi/3), which should return the exact rational 1/2. |
| `real_shortcut_adversarial/tan_exact_pi_over_four` | 63.33 ns | 62.95 ns - 63.79 ns | Constructs tan(pi/4), which should return the exact rational 1. |
| `real_shortcut_adversarial/asin_exact_half` | 64.25 ns | 63.78 ns - 64.80 ns | Constructs asin(1/2), which should return pi/6. |
| `real_shortcut_adversarial/acos_exact_half` | 63.86 ns | 62.86 ns - 65.26 ns | Constructs acos(1/2), which should return pi/3. |
| `real_shortcut_adversarial/atan_exact_one` | 56.38 ns | 56.16 ns - 56.66 ns | Constructs atan(1), which should return pi/4. |
| `real_shortcut_adversarial/asin_domain_error` | 42.20 ns | 42.01 ns - 42.39 ns | Rejects asin(1 + 1e-12). |
| `real_shortcut_adversarial/acos_domain_error` | 41.21 ns | 40.79 ns - 41.73 ns | Rejects acos(1 + 1e-12). |
| `real_shortcut_adversarial/atanh_endpoint_infinity` | 10.42 ns | 10.40 ns - 10.44 ns | Rejects atanh(1) as an infinite endpoint. |
| `real_shortcut_adversarial/atanh_domain_error` | 26.81 ns | 26.73 ns - 26.89 ns | Rejects atanh(1 + 1e-12). |
| `real_shortcut_adversarial/acosh_domain_error` | 24.38 ns | 24.32 ns - 24.44 ns | Rejects acosh(1 - 1e-12). |

<!-- END adversarial_transcendentals -->

<!-- BEGIN borrowed_ops -->
## `borrowed_ops`

Compares owned arithmetic with borrowed arithmetic for exact and irrational values.

### `rational_ops`

Owned versus borrowed arithmetic for exact `Rational` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `rational_ops/add_owned` | 10.88 ns | 10.87 ns - 10.90 ns | Adds cloned owned operands. |
| `rational_ops/add_refs` | 9.40 ns | 9.40 ns - 9.41 ns | Adds borrowed operands without cloning both inputs. |
| `rational_ops/sub_owned` | 11.17 ns | 11.16 ns - 11.19 ns | Subtracts cloned owned operands. |
| `rational_ops/sub_refs` | 9.67 ns | 9.66 ns - 9.68 ns | Subtracts borrowed operands. |
| `rational_ops/mul_owned` | 28.00 ns | 27.89 ns - 28.14 ns | Multiplies cloned owned operands. |
| `rational_ops/mul_refs` | 26.21 ns | 26.17 ns - 26.24 ns | Multiplies borrowed operands. |
| `rational_ops/div_owned` | 171.22 ns | 170.74 ns - 171.81 ns | Divides cloned owned operands. |
| `rational_ops/div_refs` | 156.76 ns | 156.64 ns - 156.89 ns | Divides borrowed operands. |

### `real_ops`

Owned versus borrowed arithmetic for exact rational-backed `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_ops/add_owned` | 23.21 ns | 23.17 ns - 23.25 ns | Adds cloned owned operands. |
| `real_ops/add_refs` | 24.20 ns | 24.15 ns - 24.25 ns | Adds borrowed operands without cloning both inputs. |
| `real_ops/sub_owned` | 25.85 ns | 25.78 ns - 25.95 ns | Subtracts cloned owned operands. |
| `real_ops/sub_refs` | 24.17 ns | 24.12 ns - 24.26 ns | Subtracts borrowed operands. |
| `real_ops/mul_owned` | 40.10 ns | 39.98 ns - 40.24 ns | Multiplies cloned owned operands. |
| `real_ops/mul_refs` | 36.29 ns | 36.16 ns - 36.44 ns | Multiplies borrowed operands. |
| `real_ops/div_owned` | 196.82 ns | 196.38 ns - 197.58 ns | Divides cloned owned operands. |
| `real_ops/div_refs` | 189.46 ns | 189.29 ns - 189.64 ns | Divides borrowed operands. |

### `real_irrational_ops`

Owned versus borrowed arithmetic for symbolic irrational `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_irrational_ops/add_owned` | 115.06 ns | 66.92 ns - 210.82 ns | Adds cloned owned operands. |
| `real_irrational_ops/add_refs` | 59.52 ns | 59.41 ns - 59.65 ns | Adds borrowed operands without cloning both inputs. |
| `real_irrational_ops/sub_owned` | 114.50 ns | 87.82 ns - 167.67 ns | Subtracts cloned owned operands. |
| `real_irrational_ops/sub_refs` | 76.05 ns | 75.99 ns - 76.11 ns | Subtracts borrowed operands. |
| `real_irrational_ops/mul_owned` | 267.46 ns | 266.36 ns - 268.75 ns | Multiplies cloned owned operands. |
| `real_irrational_ops/mul_refs` | 226.43 ns | 226.02 ns - 226.91 ns | Multiplies borrowed operands. |
| `real_irrational_ops/div_owned` | 144.91 ns | 144.38 ns - 145.38 ns | Divides cloned owned operands. |
| `real_irrational_ops/div_refs` | 104.88 ns | 104.74 ns - 105.04 ns | Divides borrowed operands. |

<!-- END borrowed_ops -->

<!-- BEGIN float_convert -->
## `float_convert`

Covers exact import of floating-point values, including public `Real` conversion overhead.

### `float_convert`

Exact conversion from IEEE-754 floats into `Rational` and `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `float_convert/f32_normal` | 45.95 ns | 45.85 ns - 46.07 ns | Converts a normal `f32` into an exact `Rational`. |
| `float_convert/f64_normal` | 46.38 ns | 46.36 ns - 46.40 ns | Converts a normal `f64` into an exact `Rational`. |
| `float_convert/f64_binary_fraction` | 46.45 ns | 46.40 ns - 46.50 ns | Converts an exactly representable binary `f64` fraction into `Rational`. |
| `float_convert/f64_subnormal` | 54.18 ns | 54.04 ns - 54.34 ns | Converts a subnormal `f64` into an exact `Rational`. |
| `float_convert/real_f32_normal` | 71.72 ns | 71.60 ns - 71.85 ns | Converts a normal `f32` through the public `Real::try_from` path. |
| `float_convert/real_f64_normal` | 71.78 ns | 71.67 ns - 71.92 ns | Converts a normal `f64` through the public `Real::try_from` path. |
| `float_convert/real_f64_binary_fraction` | 71.83 ns | 71.63 ns - 72.09 ns | Converts an exactly representable binary `f64` fraction through the public `Real::try_from` path. |
| `float_convert/real_f64_subnormal` | 78.08 ns | 78.03 ns - 78.13 ns | Converts a subnormal `f64` through the public `Real::try_from` path. |

<!-- END float_convert -->
