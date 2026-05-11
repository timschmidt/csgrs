

<!-- BEGIN numerical_micro -->
## `numerical_micro`

Low-level `Computable` microbenchmarks for approximation kernels, caches, structural facts, comparisons, and deep evaluator trees.

### `computable_cache`

Cold versus cached approximation of basic `Computable` expressions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_cache/ratio_approx_cold_p128` | 110.55 ns | 109.69 ns - 111.44 ns | Approximates a rational value at p=-128 from a fresh clone. |
| `computable_cache/ratio_approx_cached_p128` | 21.13 ns | 21.03 ns - 21.24 ns | Repeats an already cached rational approximation at p=-128. |
| `computable_cache/pi_approx_cold_p128` | 44.66 ns | 44.38 ns - 45.00 ns | Approximates pi at p=-128 from a fresh clone. |
| `computable_cache/pi_approx_cached_p128` | 22.18 ns | 22.15 ns - 22.22 ns | Repeats an already cached pi approximation at p=-128. |
| `computable_cache/pi_plus_tiny_cold_p128` | 214.48 ns | 213.92 ns - 215.02 ns | Approximates pi plus a tiny exact rational perturbation. |
| `computable_cache/pi_minus_tiny_cold_p128` | 213.76 ns | 212.91 ns - 214.60 ns | Approximates pi minus a tiny exact rational perturbation. |

### `computable_bounds`

Structural sign and bound discovery for deep or perturbed computable trees.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_bounds/deep_scaled_product_sign` | 70.84 ns | 70.13 ns - 71.41 ns | Finds the sign of a deep scaled product. |
| `computable_bounds/scaled_square_sign` | 179.84 ns | 178.17 ns - 181.62 ns | Finds the sign of repeated squaring with exact scale factors. |
| `computable_bounds/sqrt_scaled_square_sign` | 161.19 ns | 158.74 ns - 163.52 ns | Finds the sign after taking a square root of a scaled square. |
| `computable_bounds/deep_structural_bound_sign` | 18.37 ns | 18.25 ns - 18.52 ns | Finds sign through repeated multiply/inverse/negate structural transformations. |
| `computable_bounds/deep_structural_bound_sign_cached` | 3.83 ns | 3.81 ns - 3.85 ns | Reads the cached sign of the deep structural-bound chain. |
| `computable_bounds/deep_structural_bound_facts_cached` | 12.42 ns | 12.40 ns - 12.45 ns | Reads cached structural facts for the deep structural-bound chain. |
| `computable_bounds/perturbed_scaled_product_sign` | 149.58 ns | 146.91 ns - 151.85 ns | Finds sign for a deeply scaled value with a tiny perturbation. |
| `computable_bounds/perturbed_scaled_product_sign_until` | 148.55 ns | 146.64 ns - 150.13 ns | Refines sign for the perturbed scaled product only to p=-128. |
| `computable_bounds/pi_minus_tiny_sign` | 71.77 ns | 70.79 ns - 72.72 ns | Finds sign for pi minus a tiny exact rational. |
| `computable_bounds/pi_minus_tiny_sign_cached` | 3.81 ns | 3.80 ns - 3.82 ns | Reads cached sign for pi minus a tiny exact rational. |
| `computable_bounds/exp_unknown_sign_arg_sign` | 74.28 ns | 73.94 ns - 74.75 ns | Finds sign for exp(1 - pi), where exp can prove positivity structurally. |
| `computable_bounds/exp_unknown_sign_arg_sign_cached` | 3.82 ns | 3.80 ns - 3.84 ns | Reads cached sign for exp(1 - pi). |

### `computable_compare`

Ordering and absolute-comparison shortcuts.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_compare/compare_to_opposite_sign` | 12.29 ns | 12.20 ns - 12.41 ns | Compares values with known opposite signs. |
| `computable_compare/compare_to_exact_msd_gap` | 18.72 ns | 18.63 ns - 18.82 ns | Compares values with a large exact magnitude gap. |
| `computable_compare/compare_absolute_exact_rational` | 3.90 ns | 3.89 ns - 3.92 ns | Compares absolute values of exact rationals. |
| `computable_compare/compare_absolute_dominant_add` | 14.94 ns | 14.86 ns - 15.03 ns | Compares a dominant term against the same term plus a tiny addend. |
| `computable_compare/compare_absolute_exact_msd_gap` | 18.87 ns | 18.76 ns - 19.00 ns | Compares absolute values with a large exact magnitude gap. |

### `computable_transcendentals`

Low-level approximation kernels and deep expression-tree stress cases.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_transcendentals/legacy_exp_one_p128` | 2.868 us | 2.860 us - 2.876 us | Runs the legacy direct exp series for input 1 at p=-128. |
| `computable_transcendentals/e_constant_cold_p128` | 43.58 ns | 43.16 ns - 44.05 ns | Approximates the shared e constant from a fresh clone. |
| `computable_transcendentals/e_constant_cached_p128` | 22.01 ns | 21.97 ns - 22.05 ns | Repeats a cached approximation of e. |
| `computable_transcendentals/legacy_exp_half_p128` | 2.530 us | 2.514 us - 2.548 us | Runs the legacy direct exp series for input 1/2 at p=-128. |
| `computable_transcendentals/exp_cold_p128` | 3.706 us | 3.696 us - 3.719 us | Approximates exp(7/5) from a fresh clone. |
| `computable_transcendentals/exp_cached_p128` | 21.18 ns | 21.09 ns - 21.29 ns | Repeats a cached exp(7/5) approximation. |
| `computable_transcendentals/exp_large_cold_p128` | 6.854 us | 6.831 us - 6.880 us | Approximates exp(128), exercising large-argument reduction. |
| `computable_transcendentals/exp_half_cold_p128` | 2.803 us | 2.797 us - 2.809 us | Approximates exp(1/2). |
| `computable_transcendentals/exp_near_limit_cold_p128` | 2.779 us | 2.773 us - 2.786 us | Approximates exp near a prescaling threshold. |
| `computable_transcendentals/exp_near_limit_cached_p128` | 21.35 ns | 21.32 ns - 21.38 ns | Repeats a cached near-threshold exp approximation. |
| `computable_transcendentals/exp_zero_cold_p128` | 73.31 ns | 72.77 ns - 74.00 ns | Approximates exp(0). |
| `computable_transcendentals/ln_cold_p128` | 4.263 us | 4.251 us - 4.275 us | Approximates ln(11/7). |
| `computable_transcendentals/ln_cached_p128` | 21.07 ns | 20.99 ns - 21.15 ns | Repeats a cached ln(11/7) approximation. |
| `computable_transcendentals/ln_smooth_rational_cold_p128` | 628.15 ns | 623.93 ns - 632.51 ns | Approximates ln(45/14), which can decompose into shared prime-log constants. |
| `computable_transcendentals/ln_nonsmooth_rational_cold_p128` | 2.623 us | 2.613 us - 2.634 us | Approximates ln(11/13), guarding the generic exact-rational log fallback. |
| `computable_transcendentals/ln_large_cold_p128` | 302.49 ns | 300.08 ns - 306.00 ns | Approximates ln(1024), exercising large-input reduction. |
| `computable_transcendentals/ln_large_cached_p128` | 20.85 ns | 20.84 ns - 20.87 ns | Repeats a cached ln(1024) approximation. |
| `computable_transcendentals/ln_tiny_cold_p128` | 188.23 ns | 187.23 ns - 189.37 ns | Approximates ln(2^-1024), exercising tiny-input reduction. |
| `computable_transcendentals/ln_near_limit_cold_p128` | 7.132 us | 7.093 us - 7.181 us | Approximates ln near the prescaled-ln limit. |
| `computable_transcendentals/ln_near_limit_cached_p128` | 21.27 ns | 21.16 ns - 21.38 ns | Repeats a cached near-limit ln approximation. |
| `computable_transcendentals/ln_one_cold_p128` | 33.45 ns | 33.25 ns - 33.66 ns | Approximates ln(1). |
| `computable_transcendentals/sqrt_cold_p128` | 790.13 ns | 773.89 ns - 818.76 ns | Approximates sqrt(2). |
| `computable_transcendentals/sqrt_squarefree_scaled_cold_p128` | 109.03 ns | 97.77 ns - 131.07 ns | Approximates sqrt(12), which can reduce to 2*sqrt(3). |
| `computable_transcendentals/sqrt_cached_p128` | 21.22 ns | 21.14 ns - 21.31 ns | Repeats a cached sqrt(2) approximation. |
| `computable_transcendentals/sqrt_single_scaled_square_cold_p128` | 1.088 us | 1.084 us - 1.093 us | Builds and approximates sqrt((7*pi/8)^2). |
| `computable_transcendentals/sin_cold_p96` | 1.686 us | 1.665 us - 1.712 us | Approximates sin(7/5). |
| `computable_transcendentals/sin_cached_p96` | 21.04 ns | 21.00 ns - 21.08 ns | Repeats a cached sin(7/5) approximation. |
| `computable_transcendentals/cos_cold_p96` | 1.624 us | 1.590 us - 1.666 us | Approximates cos(7/5). |
| `computable_transcendentals/sin_f64_cold_p96` | 1.843 us | 1.812 us - 1.880 us | Approximates sin(1.23456789 imported exactly from f64). |
| `computable_transcendentals/cos_f64_cold_p96` | 1.795 us | 1.763 us - 1.833 us | Approximates cos(1.23456789 imported exactly from f64). |
| `computable_transcendentals/sin_1e6_cold_p96` | 2.340 us | 2.285 us - 2.404 us | Approximates sin(1000000). |
| `computable_transcendentals/cos_1e6_cold_p96` | 2.392 us | 2.337 us - 2.459 us | Approximates cos(1000000). |
| `computable_transcendentals/sin_1e30_cold_p96` | 2.077 us | 2.049 us - 2.114 us | Approximates sin(10^30). |
| `computable_transcendentals/cos_1e30_cold_p96` | 2.451 us | 2.351 us - 2.560 us | Approximates cos(10^30). |
| `computable_transcendentals/cos_cached_p96` | 22.80 ns | 22.14 ns - 23.52 ns | Repeats a cached cos(7/5) approximation. |
| `computable_transcendentals/tan_cold_p96` | 3.593 us | 3.526 us - 3.672 us | Approximates tan(7/5). |
| `computable_transcendentals/tan_cached_p96` | 21.17 ns | 21.08 ns - 21.28 ns | Repeats a cached tan(7/5) approximation. |
| `computable_transcendentals/sin_zero_cold_p96` | 33.60 ns | 33.43 ns - 33.78 ns | Approximates sin(0). |
| `computable_transcendentals/cos_zero_cold_p96` | 82.70 ns | 79.41 ns - 86.65 ns | Approximates cos(0). |
| `computable_transcendentals/tan_zero_cold_p96` | 33.72 ns | 33.47 ns - 34.04 ns | Approximates tan(0). |
| `computable_transcendentals/tan_near_half_pi_cold_p96` | 5.594 us | 5.471 us - 5.738 us | Approximates tangent near pi/2. |
| `computable_transcendentals/tan_near_half_pi_cached_p96` | 21.18 ns | 21.11 ns - 21.26 ns | Repeats cached tangent near pi/2. |
| `computable_transcendentals/sin_huge_cold_p96` | 1.656 us | 1.653 us - 1.659 us | Approximates sine of a huge pi multiple plus offset. |
| `computable_transcendentals/cos_huge_cold_p96` | 1.568 us | 1.565 us - 1.572 us | Approximates cosine of a huge pi multiple plus offset. |
| `computable_transcendentals/tan_huge_cold_p96` | 3.496 us | 3.481 us - 3.512 us | Approximates tangent of a huge pi multiple plus offset. |
| `computable_transcendentals/asin_cold_p96` | 6.377 us | 6.349 us - 6.412 us | Approximates a computable asin expression. |
| `computable_transcendentals/asin_cached_p96` | 21.14 ns | 21.08 ns - 21.21 ns | Repeats a cached computable asin approximation. |
| `computable_transcendentals/acos_cold_p96` | 8.797 us | 8.502 us - 9.125 us | Approximates a computable acos expression. |
| `computable_transcendentals/acos_cached_p96` | 20.83 ns | 20.81 ns - 20.85 ns | Repeats a cached computable acos approximation. |
| `computable_transcendentals/asin_tiny_cold_p96` | 366.85 ns | 352.16 ns - 395.78 ns | Approximates asin(1e-12), exercising the tiny-input series. |
| `computable_transcendentals/acos_tiny_cold_p96` | 707.90 ns | 679.88 ns - 762.72 ns | Approximates acos(1e-12), exercising the tiny-input complement. |
| `computable_transcendentals/asin_near_one_cold_p96` | 4.472 us | 4.465 us - 4.479 us | Approximates asin(0.999999), exercising the endpoint complement. |
| `computable_transcendentals/acos_near_one_cold_p96` | 4.020 us | 4.015 us - 4.026 us | Approximates acos(0.999999), exercising the endpoint transform. |
| `computable_transcendentals/atan_cold_p96` | 7.175 us | 7.146 us - 7.209 us | Approximates atan(7/10). |
| `computable_transcendentals/atan_cached_p96` | 20.95 ns | 20.90 ns - 21.02 ns | Repeats a cached atan(7/10) approximation. |
| `computable_transcendentals/atan_large_cold_p96` | 2.094 us | 1.987 us - 2.208 us | Approximates atan(8), exercising argument reduction. |
| `computable_transcendentals/asin_zero_cold_p96` | 37.00 ns | 35.23 ns - 39.03 ns | Approximates asin(0) expression. |
| `computable_transcendentals/atan_zero_cold_p96` | 33.19 ns | 33.07 ns - 33.30 ns | Approximates atan(0). |
| `computable_transcendentals/asinh_cold_p128` | 6.294 us | 6.139 us - 6.491 us | Approximates a computable asinh expression. |
| `computable_transcendentals/asinh_cached_p128` | 21.87 ns | 21.35 ns - 22.51 ns | Repeats a cached computable asinh approximation. |
| `computable_transcendentals/acosh_cold_p128` | 9.704 us | 9.423 us - 10.030 us | Approximates a computable acosh expression. |
| `computable_transcendentals/acosh_cached_p128` | 22.87 ns | 22.22 ns - 23.60 ns | Repeats a cached computable acosh approximation. |
| `computable_transcendentals/atanh_cold_p128` | 266.41 ns | 156.28 ns - 482.11 ns | Approximates a computable atanh expression. |
| `computable_transcendentals/atanh_cached_p128` | 21.25 ns | 21.08 ns - 21.45 ns | Repeats a cached computable atanh approximation. |
| `computable_transcendentals/atanh_tiny_cold_p128` | 504.63 ns | 480.99 ns - 549.14 ns | Approximates atanh(1e-12), exercising the tiny-input series. |
| `computable_transcendentals/atanh_near_one_cold_p128` | 2.716 us | 2.707 us - 2.726 us | Approximates atanh(0.999999), exercising the endpoint log transform. |
| `computable_transcendentals/asinh_zero_cold_p128` | 33.54 ns | 33.40 ns - 33.69 ns | Approximates asinh(0) expression. |
| `computable_transcendentals/atanh_zero_cold_p128` | 43.29 ns | 40.84 ns - 45.87 ns | Approximates atanh(0) expression. |
| `computable_transcendentals/deep_add_chain_cold_p128` | 102.98 ns | 82.45 ns - 141.35 ns | Approximates a 5000-node addition chain. |
| `computable_transcendentals/deep_multiply_chain_cold_p128` | 92.99 ns | 86.85 ns - 99.72 ns | Approximates a 5000-node multiply-by-one chain. |
| `computable_transcendentals/deep_multiply_identity_chain_cold_p128` | 88.24 ns | 87.94 ns - 88.61 ns | Approximates a deep identity multiplication chain around pi. |
| `computable_transcendentals/deep_scaled_product_chain_cold_p128` | 698.20 ns | 568.83 ns - 952.04 ns | Approximates a deep product of exact scale factors. |
| `computable_transcendentals/perturbed_scaled_product_chain_cold_p128` | 843.07 ns | 836.00 ns - 850.81 ns | Approximates a deep scaled product with a tiny perturbation. |
| `computable_transcendentals/scaled_square_chain_cold_p128` | 1.343 us | 1.329 us - 1.361 us | Approximates repeated squaring of a scaled irrational. |
| `computable_transcendentals/asymmetric_product_bad_order_cold_p128` | 890.20 ns | 849.95 ns - 967.25 ns | Approximates an asymmetric product order stress case. |
| `computable_transcendentals/sqrt_scaled_square_chain_cold_p128` | 1.143 us | 1.111 us - 1.185 us | Approximates sqrt of a scaled-square chain. |
| `computable_transcendentals/warmed_zero_product_cold_p128` | 458.99 ns | 458.17 ns - 459.87 ns | Approximates a product involving a warmed zero sum. |
| `computable_transcendentals/inverse_scaled_product_chain_cold_p128` | 920.44 ns | 773.70 ns - 1.192 us | Approximates the inverse of a deep scaled product. |
| `computable_transcendentals/deep_inverse_pair_chain_cold_p128` | 88.35 ns | 88.06 ns - 88.68 ns | Approximates a chain of inverse(inverse(x)) pairs. |
| `computable_transcendentals/deep_negated_square_chain_cold_p128` | 88.27 ns | 87.85 ns - 88.76 ns | Approximates repeated negate-square-sqrt transformations. |
| `computable_transcendentals/deep_negative_one_product_chain_cold_p128` | 107.27 ns | 101.93 ns - 113.13 ns | Approximates repeated multiplication by -1. |
| `computable_transcendentals/deep_half_product_chain_cold_p128` | 142.96 ns | 137.14 ns - 149.68 ns | Approximates repeated multiplication by 1/2. |
| `computable_transcendentals/deep_half_square_chain_cold_p128` | 938.43 ns | 929.99 ns - 948.16 ns | Approximates repeated squaring after scaling by 1/2. |
| `computable_transcendentals/deep_sqrt_square_chain_cold_p128` | 78.50 ns | 77.75 ns - 79.30 ns | Approximates repeated sqrt-square simplification. |
| `computable_transcendentals/inverse_half_product_chain_cold_p128` | 514.93 ns | 489.52 ns - 544.53 ns | Approximates the inverse of a deep half-product chain. |

<!-- END numerical_micro -->

<!-- BEGIN scalar_micro -->
## `scalar_micro`

Microbenchmarks for scalar operations, structural queries, cache hits, and dense exact arithmetic.

### `construction_speed`

Cost of constructing common exact scalar identities.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `construction_speed/rational_one` | 17.58 ns | 16.77 ns - 18.47 ns | Constructs `Rational::one()`. |
| `construction_speed/rational_new_one` | 26.50 ns | 26.09 ns - 27.01 ns | Constructs one through `Rational::new(1)`. |
| `construction_speed/computable_one` | 25.35 ns | 24.56 ns - 26.30 ns | Constructs `Computable::one()`. |
| `construction_speed/real_new_rational_one` | 76.14 ns | 73.96 ns - 78.71 ns | Constructs one through `Real::new(Rational::one())`. |
| `construction_speed/real_one` | 84.41 ns | 77.48 ns - 93.31 ns | Constructs one through `Real::one()`. |
| `construction_speed/real_from_i32_one` | 74.53 ns | 73.96 ns - 75.19 ns | Constructs one through integer conversion. |

### `raw_cache_hit_cost`

Cost of cold and cached `Computable::approx` calls for simple values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `raw_cache_hit_cost/zero` | 48.57 ns | 48.19 ns - 49.09 ns | Cached approximation request for exact zero. |
| `raw_cache_hit_cost/one` | 66.94 ns | 66.79 ns - 67.11 ns | Cached approximation request for exact one. |
| `raw_cache_hit_cost/two` | 66.95 ns | 66.80 ns - 67.13 ns | Cached approximation request for exact two. |
| `raw_cache_hit_cost/e` | 70.92 ns | 70.79 ns - 71.06 ns | Cached approximation request for Euler's constant. |
| `raw_cache_hit_cost/pi` | 72.92 ns | 71.43 ns - 74.74 ns | Cached approximation request for pi. |
| `raw_cache_hit_cost/tau` | 74.06 ns | 72.46 ns - 75.91 ns | Cached approximation request for two pi. |

### `structural_query_speed`

Speed of public structural queries across exact, transcendental, and composite `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `structural_query_speed/zero_zero_status` | 0.75 ns | 0.73 ns - 0.78 ns | Checks zero/nonzero facts for exact zero. |
| `structural_query_speed/zero_sign_query` | 4.36 ns | 4.23 ns - 4.51 ns | Reads sign facts for exact zero. |
| `structural_query_speed/zero_msd_query` | 6.25 ns | 6.13 ns - 6.41 ns | Reads magnitude facts for exact zero. |
| `structural_query_speed/zero_structural_facts` | 6.99 ns | 6.83 ns - 7.19 ns | Computes full structural facts for exact zero. |
| `structural_query_speed/one_zero_status` | 0.95 ns | 0.92 ns - 0.98 ns | Checks zero/nonzero facts for exact one. |
| `structural_query_speed/one_sign_query` | 24.97 ns | 23.82 ns - 26.21 ns | Reads sign facts for exact one. |
| `structural_query_speed/one_msd_query` | 25.01 ns | 24.16 ns - 25.98 ns | Reads magnitude facts for exact one. |
| `structural_query_speed/one_structural_facts` | 25.69 ns | 24.94 ns - 26.61 ns | Computes full structural facts for exact one. |
| `structural_query_speed/negative_zero_status` | 0.96 ns | 0.93 ns - 1.00 ns | Checks zero/nonzero facts for an exact negative integer. |
| `structural_query_speed/negative_sign_query` | 23.60 ns | 22.86 ns - 24.48 ns | Reads sign facts for an exact negative integer. |
| `structural_query_speed/negative_msd_query` | 25.84 ns | 25.01 ns - 26.79 ns | Reads magnitude facts for an exact negative integer. |
| `structural_query_speed/negative_structural_facts` | 28.44 ns | 27.57 ns - 29.47 ns | Computes full structural facts for an exact negative integer. |
| `structural_query_speed/tiny_exact_zero_status` | 0.95 ns | 0.93 ns - 0.98 ns | Checks zero/nonzero facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_sign_query` | 27.76 ns | 26.63 ns - 29.09 ns | Reads sign facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_msd_query` | 30.76 ns | 29.52 ns - 32.15 ns | Reads magnitude facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_structural_facts` | 33.23 ns | 32.38 ns - 34.24 ns | Computes full structural facts for a tiny exact rational. |
| `structural_query_speed/pi_zero_status` | 0.93 ns | 0.91 ns - 0.96 ns | Checks zero/nonzero facts for pi. |
| `structural_query_speed/pi_sign_query` | 36.04 ns | 35.00 ns - 37.26 ns | Reads sign facts for pi. |
| `structural_query_speed/pi_msd_query` | 40.22 ns | 40.10 ns - 40.36 ns | Reads magnitude facts for pi. |
| `structural_query_speed/pi_structural_facts` | 36.29 ns | 36.24 ns - 36.36 ns | Computes full structural facts for pi. |
| `structural_query_speed/e_zero_status` | 0.90 ns | 0.90 ns - 0.91 ns | Checks zero/nonzero facts for e. |
| `structural_query_speed/e_sign_query` | 34.58 ns | 34.37 ns - 34.83 ns | Reads sign facts for e. |
| `structural_query_speed/e_msd_query` | 40.42 ns | 40.18 ns - 40.72 ns | Reads magnitude facts for e. |
| `structural_query_speed/e_structural_facts` | 36.96 ns | 36.82 ns - 37.12 ns | Computes full structural facts for e. |
| `structural_query_speed/tau_zero_status` | 0.89 ns | 0.89 ns - 0.90 ns | Checks zero/nonzero facts for tau. |
| `structural_query_speed/tau_sign_query` | 34.08 ns | 33.98 ns - 34.20 ns | Reads sign facts for tau. |
| `structural_query_speed/tau_msd_query` | 40.25 ns | 40.04 ns - 40.51 ns | Reads magnitude facts for tau. |
| `structural_query_speed/tau_structural_facts` | 37.08 ns | 36.92 ns - 37.26 ns | Computes full structural facts for tau. |
| `structural_query_speed/sqrt_two_zero_status` | 0.90 ns | 0.90 ns - 0.91 ns | Checks zero/nonzero facts for sqrt(2). |
| `structural_query_speed/sqrt_two_sign_query` | 34.29 ns | 34.22 ns - 34.37 ns | Reads sign facts for sqrt(2). |
| `structural_query_speed/sqrt_two_msd_query` | 40.16 ns | 40.06 ns - 40.28 ns | Reads magnitude facts for sqrt(2). |
| `structural_query_speed/sqrt_two_structural_facts` | 36.41 ns | 36.34 ns - 36.48 ns | Computes full structural facts for sqrt(2). |
| `structural_query_speed/pi_minus_three_zero_status` | 0.90 ns | 0.90 ns - 0.90 ns | Checks zero/nonzero facts for pi - 3. |
| `structural_query_speed/pi_minus_three_sign_query` | 34.86 ns | 34.71 ns - 35.01 ns | Reads sign facts for pi - 3. |
| `structural_query_speed/pi_minus_three_msd_query` | 40.69 ns | 40.50 ns - 40.91 ns | Reads magnitude facts for pi - 3. |
| `structural_query_speed/pi_minus_three_structural_facts` | 36.51 ns | 36.36 ns - 36.70 ns | Computes full structural facts for pi - 3. |
| `structural_query_speed/dense_expr_zero_status` | 3.45 ns | 3.42 ns - 3.48 ns | Checks zero/nonzero facts for a dense composite expression. |
| `structural_query_speed/dense_expr_sign_query` | 34.27 ns | 34.19 ns - 34.35 ns | Reads sign facts for a dense composite expression. |
| `structural_query_speed/dense_expr_msd_query` | 40.08 ns | 39.93 ns - 40.25 ns | Reads magnitude facts for a dense composite expression. |
| `structural_query_speed/dense_expr_structural_facts` | 37.77 ns | 37.71 ns - 37.84 ns | Computes full structural facts for a dense composite expression. |

### `pure_scalar_algorithm_speed`

Core scalar algorithms that do not require high-precision transcendental approximation.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `pure_scalar_algorithm_speed/rational_add` | 386.37 ns | 383.83 ns - 389.00 ns | Adds two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_mul` | 126.55 ns | 122.61 ns - 131.46 ns | Multiplies two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_div` | 643.17 ns | 622.57 ns - 667.38 ns | Divides two nontrivial rational values. |
| `pure_scalar_algorithm_speed/real_exact_add` | 480.56 ns | 462.36 ns - 501.65 ns | Adds exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_mul` | 209.40 ns | 201.90 ns - 217.89 ns | Multiplies exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_div` | 722.32 ns | 694.58 ns - 754.91 ns | Divides exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_sqrt_reduce` | 463.19 ns | 450.76 ns - 477.62 ns | Reduces an exact square-root expression. |
| `pure_scalar_algorithm_speed/real_exact_ln_reduce` | 310.92 ns | 250.85 ns - 424.44 ns | Reduces an exact logarithm of a power of two. |

### `borrowed_op_overhead`

Borrowed versus owned operation overhead for rational and real operands.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `borrowed_op_overhead/rational_clone_pair` | 43.69 ns | 43.43 ns - 43.98 ns | Clones two rational values. |
| `borrowed_op_overhead/rational_add_refs` | 374.54 ns | 372.89 ns - 376.20 ns | Adds rational references. |
| `borrowed_op_overhead/rational_add_owned` | 393.55 ns | 391.69 ns - 395.56 ns | Adds owned rational values. |
| `borrowed_op_overhead/real_clone_pair` | 265.47 ns | 264.44 ns - 266.60 ns | Clones two scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_unscaled_add_refs` | 174.25 ns | 173.45 ns - 175.18 ns | Adds borrowed unscaled transcendental `Real` values. |
| `borrowed_op_overhead/real_unscaled_add_owned` | 220.36 ns | 219.37 ns - 221.33 ns | Adds owned unscaled transcendental `Real` values. |
| `borrowed_op_overhead/real_add_refs` | 636.19 ns | 633.00 ns - 639.92 ns | Adds borrowed scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_add_owned` | 620.33 ns | 618.33 ns - 622.44 ns | Adds owned scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_dot3_refs_dense_symbolic` | 3.822 us | 3.813 us - 3.832 us | Computes a borrowed three-lane symbolic dot product with no rational shortcut terms. |
| `borrowed_op_overhead/real_dot3_refs_mixed_structural` | 781.25 ns | 776.00 ns - 787.19 ns | Computes a borrowed three-lane symbolic dot product with exact zero and rational scale terms. |
| `borrowed_op_overhead/real_dot4_refs_dense_symbolic` | 6.528 us | 6.479 us - 6.598 us | Computes a borrowed four-lane symbolic dot product with no rational shortcut terms. |
| `borrowed_op_overhead/real_dot4_refs_mixed_structural` | 828.36 ns | 825.78 ns - 831.20 ns | Computes a borrowed four-lane symbolic dot product with exact zero and rational scale terms. |

### `dense_algebra`

Small dense algebra kernels that stress repeated exact and symbolic operations.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `dense_algebra/rational_dot_64` | 42.155 us | 39.904 us - 44.587 us | Computes a 64-element rational dot product. |
| `dense_algebra/rational_matmul_8` | 231.105 us | 229.751 us - 232.676 us | Computes an 8x8 rational matrix multiply. |
| `dense_algebra/real_dot_36` | 33.570 us | 31.919 us - 35.275 us | Computes a 36-element dot product over symbolic `Real` values. |
| `dense_algebra/real_matmul_6` | 173.204 us | 164.440 us - 182.769 us | Computes a 6x6 matrix multiply over symbolic `Real` values. |

### `exact_transcendental_special_forms`

Construction-time shortcuts for exact rational multiples of pi and inverse compositions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `exact_transcendental_special_forms/sin_pi_7` | 455.33 ns | 428.62 ns - 483.63 ns | Builds the exact special form for sin(pi/7). |
| `exact_transcendental_special_forms/cos_pi_7` | 973.13 ns | 920.43 ns - 1.033 us | Builds the exact special form for cos(pi/7). |
| `exact_transcendental_special_forms/tan_pi_7` | 380.02 ns | 360.26 ns - 402.51 ns | Builds the exact special form for tan(pi/7). |
| `exact_transcendental_special_forms/asin_sin_6pi_7` | 1.063 us | 1.011 us - 1.121 us | Recognizes the principal branch of asin(sin(6pi/7)). |
| `exact_transcendental_special_forms/acos_cos_9pi_7` | 1.853 us | 1.780 us - 1.934 us | Recognizes the principal branch of acos(cos(9pi/7)). |
| `exact_transcendental_special_forms/atan_tan_6pi_7` | 1.022 us | 975.08 ns - 1.072 us | Recognizes the principal branch of atan(tan(6pi/7)). |
| `exact_transcendental_special_forms/asinh_large` | 385.09 ns | 370.30 ns - 401.07 ns | Builds a large inverse hyperbolic sine without exact intermediate Reals. |
| `exact_transcendental_special_forms/atanh_sqrt_half` | 7.794 us | 7.053 us - 8.985 us | Builds atanh(sqrt(2)/2) after exact structural domain checks. |

### `symbolic_reductions`

Existing symbolic constant algebra cases considered for additional reductions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `symbolic_reductions/sqrt_pi_square` | 131.95 ns | 131.14 ns - 132.92 ns | Reduces sqrt(pi^2). |
| `symbolic_reductions/sqrt_pi_e_square` | 173.85 ns | 172.69 ns - 174.99 ns | Reduces sqrt((pi * e)^2). |
| `symbolic_reductions/ln_scaled_e` | 1.678 us | 1.670 us - 1.687 us | Reduces ln(2 * e). |
| `symbolic_reductions/sub_pi_three` | 1.792 us | 1.783 us - 1.805 us | Builds the certified pi - 3 constant-offset form. |
| `symbolic_reductions/pi_minus_three_facts` | 35.89 ns | 35.76 ns - 36.04 ns | Reads structural facts for the cached pi - 3 offset form. |
| `symbolic_reductions/div_exp_exp` | 779.54 ns | 597.37 ns - 1.141 us | Reduces e^3 / e. |
| `symbolic_reductions/div_pi_square_e` | 486.05 ns | 484.30 ns - 488.28 ns | Reduces pi^2 / e. |
| `symbolic_reductions/div_const_products` | 884.74 ns | 877.76 ns - 892.38 ns | Reduces (pi^3 * e^5) / (pi * e^2). |
| `symbolic_reductions/inverse_pi` | 121.65 ns | 121.31 ns - 122.04 ns | Builds the reciprocal of pi. |
| `symbolic_reductions/div_one_pi` | 246.02 ns | 244.95 ns - 247.29 ns | Reduces 1 / pi. |
| `symbolic_reductions/div_e_pi` | 327.96 ns | 326.90 ns - 329.21 ns | Reduces e / pi. |
| `symbolic_reductions/mul_pi_inverse_pi` | 249.97 ns | 248.46 ns - 251.69 ns | Multiplies pi by its reciprocal. |
| `symbolic_reductions/mul_pi_e_sqrt_two` | 456.69 ns | 454.14 ns - 459.34 ns | Builds the factored pi * e * sqrt(2) form. |
| `symbolic_reductions/mul_const_product_sqrt_sqrt` | 673.02 ns | 668.60 ns - 677.76 ns | Cancels sqrt(2) from (pi * e * sqrt(2)) * sqrt(2). |
| `symbolic_reductions/div_const_product_sqrt_e` | 1.078 us | 1.063 us - 1.098 us | Reduces (pi * e * sqrt(2)) / e. |
| `symbolic_reductions/inverse_const_product_sqrt` | 558.37 ns | 556.18 ns - 560.83 ns | Builds a rationalized reciprocal of pi * e * sqrt(2). |

<!-- END scalar_micro -->

