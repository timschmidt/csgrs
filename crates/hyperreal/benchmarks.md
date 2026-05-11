

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
| `computable_transcendentals/legacy_exp_one_p128` | 2.925 us | 2.916 us - 2.935 us | Runs the legacy direct exp series for input 1 at p=-128. |
| `computable_transcendentals/e_constant_cold_p128` | 45.91 ns | 45.59 ns - 46.25 ns | Approximates the shared e constant from a fresh clone. |
| `computable_transcendentals/e_constant_cached_p128` | 22.00 ns | 21.96 ns - 22.04 ns | Repeats a cached approximation of e. |
| `computable_transcendentals/legacy_exp_half_p128` | 2.521 us | 2.510 us - 2.534 us | Runs the legacy direct exp series for input 1/2 at p=-128. |
| `computable_transcendentals/exp_cold_p128` | 3.697 us | 3.684 us - 3.711 us | Approximates exp(7/5) from a fresh clone. |
| `computable_transcendentals/exp_cached_p128` | 20.79 ns | 20.75 ns - 20.83 ns | Repeats a cached exp(7/5) approximation. |
| `computable_transcendentals/exp_large_cold_p128` | 6.847 us | 6.826 us - 6.871 us | Approximates exp(128), exercising large-argument reduction. |
| `computable_transcendentals/exp_half_cold_p128` | 2.723 us | 2.721 us - 2.725 us | Approximates exp(1/2). |
| `computable_transcendentals/exp_near_limit_cold_p128` | 2.754 us | 2.741 us - 2.771 us | Approximates exp near a prescaling threshold. |
| `computable_transcendentals/exp_near_limit_cached_p128` | 20.93 ns | 20.90 ns - 20.95 ns | Repeats a cached near-threshold exp approximation. |
| `computable_transcendentals/exp_zero_cold_p128` | 72.58 ns | 71.95 ns - 73.33 ns | Approximates exp(0). |
| `computable_transcendentals/ln_cold_p128` | 4.240 us | 4.229 us - 4.252 us | Approximates ln(11/7). |
| `computable_transcendentals/ln_cached_p128` | 20.93 ns | 20.82 ns - 21.05 ns | Repeats a cached ln(11/7) approximation. |
| `computable_transcendentals/ln_smooth_rational_cold_p128` | 830.71 ns | 795.38 ns - 869.86 ns | Approximates ln(45/14), which can decompose into shared prime-log constants. |
| `computable_transcendentals/ln_nonsmooth_rational_cold_p128` | 2.520 us | 2.507 us - 2.535 us | Approximates ln(11/13), guarding the generic exact-rational log fallback. |
| `computable_transcendentals/ln_large_cold_p128` | 339.12 ns | 322.20 ns - 357.96 ns | Approximates ln(1024), exercising large-input reduction. |
| `computable_transcendentals/ln_large_cached_p128` | 21.37 ns | 21.25 ns - 21.51 ns | Repeats a cached ln(1024) approximation. |
| `computable_transcendentals/ln_tiny_cold_p128` | 204.63 ns | 202.86 ns - 206.48 ns | Approximates ln(2^-1024), exercising tiny-input reduction. |
| `computable_transcendentals/ln_near_limit_cold_p128` | 7.119 us | 7.096 us - 7.147 us | Approximates ln near the prescaled-ln limit. |
| `computable_transcendentals/ln_near_limit_cached_p128` | 21.49 ns | 21.23 ns - 21.78 ns | Repeats a cached near-limit ln approximation. |
| `computable_transcendentals/ln_one_cold_p128` | 33.96 ns | 33.72 ns - 34.24 ns | Approximates ln(1). |
| `computable_transcendentals/sqrt_cold_p128` | 753.80 ns | 747.48 ns - 760.92 ns | Approximates sqrt(2). |
| `computable_transcendentals/sqrt_squarefree_scaled_cold_p128` | 107.89 ns | 105.05 ns - 111.19 ns | Approximates sqrt(12), which can reduce to 2*sqrt(3). |
| `computable_transcendentals/sqrt_cached_p128` | 21.46 ns | 21.37 ns - 21.60 ns | Repeats a cached sqrt(2) approximation. |
| `computable_transcendentals/sqrt_single_scaled_square_cold_p128` | 1.066 us | 1.063 us - 1.070 us | Builds and approximates sqrt((7*pi/8)^2). |
| `computable_transcendentals/sin_cold_p96` | 1.584 us | 1.578 us - 1.591 us | Approximates sin(7/5). |
| `computable_transcendentals/sin_cached_p96` | 20.91 ns | 20.81 ns - 21.01 ns | Repeats a cached sin(7/5) approximation. |
| `computable_transcendentals/cos_cold_p96` | 1.477 us | 1.465 us - 1.491 us | Approximates cos(7/5). |
| `computable_transcendentals/sin_f64_cold_p96` | 1.729 us | 1.718 us - 1.742 us | Approximates sin(1.23456789 imported exactly from f64). |
| `computable_transcendentals/cos_f64_cold_p96` | 1.674 us | 1.663 us - 1.690 us | Approximates cos(1.23456789 imported exactly from f64). |
| `computable_transcendentals/sin_1e6_cold_p96` | 2.267 us | 2.256 us - 2.280 us | Approximates sin(1000000). |
| `computable_transcendentals/cos_1e6_cold_p96` | 2.268 us | 2.257 us - 2.282 us | Approximates cos(1000000). |
| `computable_transcendentals/sin_1e30_cold_p96` | 1.996 us | 1.990 us - 2.003 us | Approximates sin(10^30). |
| `computable_transcendentals/cos_1e30_cold_p96` | 2.116 us | 2.109 us - 2.124 us | Approximates cos(10^30). |
| `computable_transcendentals/cos_cached_p96` | 20.75 ns | 20.71 ns - 20.79 ns | Repeats a cached cos(7/5) approximation. |
| `computable_transcendentals/tan_cold_p96` | 3.287 us | 3.281 us - 3.293 us | Approximates tan(7/5). |
| `computable_transcendentals/tan_cached_p96` | 21.29 ns | 21.11 ns - 21.50 ns | Repeats a cached tan(7/5) approximation. |
| `computable_transcendentals/sin_zero_cold_p96` | 33.84 ns | 33.64 ns - 34.05 ns | Approximates sin(0). |
| `computable_transcendentals/cos_zero_cold_p96` | 76.84 ns | 76.33 ns - 77.41 ns | Approximates cos(0). |
| `computable_transcendentals/tan_zero_cold_p96` | 34.07 ns | 33.81 ns - 34.37 ns | Approximates tan(0). |
| `computable_transcendentals/tan_near_half_pi_cold_p96` | 5.039 us | 5.028 us - 5.051 us | Approximates tangent near pi/2. |
| `computable_transcendentals/tan_near_half_pi_cached_p96` | 20.85 ns | 20.79 ns - 20.93 ns | Repeats cached tangent near pi/2. |
| `computable_transcendentals/sin_huge_cold_p96` | 1.585 us | 1.577 us - 1.594 us | Approximates sine of a huge pi multiple plus offset. |
| `computable_transcendentals/cos_huge_cold_p96` | 1.481 us | 1.474 us - 1.489 us | Approximates cosine of a huge pi multiple plus offset. |
| `computable_transcendentals/tan_huge_cold_p96` | 3.333 us | 3.314 us - 3.354 us | Approximates tangent of a huge pi multiple plus offset. |
| `computable_transcendentals/asin_cold_p96` | 6.281 us | 6.247 us - 6.318 us | Approximates a computable asin expression. |
| `computable_transcendentals/asin_cached_p96` | 21.07 ns | 20.96 ns - 21.20 ns | Repeats a cached computable asin approximation. |
| `computable_transcendentals/acos_cold_p96` | 7.987 us | 7.926 us - 8.062 us | Approximates a computable acos expression. |
| `computable_transcendentals/acos_cached_p96` | 20.95 ns | 20.86 ns - 21.06 ns | Repeats a cached computable acos approximation. |
| `computable_transcendentals/asin_tiny_cold_p96` | 357.99 ns | 357.18 ns - 358.87 ns | Approximates asin(1e-12), exercising the tiny-input series. |
| `computable_transcendentals/acos_tiny_cold_p96` | 682.46 ns | 676.37 ns - 689.90 ns | Approximates acos(1e-12), exercising the tiny-input complement. |
| `computable_transcendentals/asin_near_one_cold_p96` | 4.773 us | 4.499 us - 5.284 us | Approximates asin(0.999999), exercising the endpoint complement. |
| `computable_transcendentals/acos_near_one_cold_p96` | 4.064 us | 4.050 us - 4.083 us | Approximates acos(0.999999), exercising the endpoint transform. |
| `computable_transcendentals/atan_cold_p96` | 7.152 us | 7.120 us - 7.188 us | Approximates atan(7/10). |
| `computable_transcendentals/atan_cached_p96` | 21.17 ns | 21.03 ns - 21.33 ns | Repeats a cached atan(7/10) approximation. |
| `computable_transcendentals/atan_large_cold_p96` | 1.793 us | 1.781 us - 1.809 us | Approximates atan(8), exercising argument reduction. |
| `computable_transcendentals/asin_zero_cold_p96` | 38.36 ns | 36.96 ns - 39.96 ns | Approximates asin(0) expression. |
| `computable_transcendentals/atan_zero_cold_p96` | 42.78 ns | 40.16 ns - 45.55 ns | Approximates atan(0). |
| `computable_transcendentals/asinh_cold_p128` | 6.093 us | 6.049 us - 6.143 us | Approximates a computable asinh expression. |
| `computable_transcendentals/asinh_cached_p128` | 21.26 ns | 21.17 ns - 21.37 ns | Repeats a cached computable asinh approximation. |
| `computable_transcendentals/acosh_cold_p128` | 9.287 us | 9.247 us - 9.332 us | Approximates a computable acosh expression. |
| `computable_transcendentals/acosh_cached_p128` | 21.29 ns | 21.22 ns - 21.37 ns | Repeats a cached computable acosh approximation. |
| `computable_transcendentals/atanh_cold_p128` | 225.26 ns | 150.03 ns - 372.67 ns | Approximates a computable atanh expression. |
| `computable_transcendentals/atanh_cached_p128` | 20.94 ns | 20.88 ns - 21.00 ns | Repeats a cached computable atanh approximation. |
| `computable_transcendentals/atanh_tiny_cold_p128` | 472.50 ns | 471.90 ns - 473.12 ns | Approximates atanh(1e-12), exercising the tiny-input series. |
| `computable_transcendentals/atanh_near_one_cold_p128` | 2.798 us | 2.637 us - 3.112 us | Approximates atanh(0.999999), exercising the endpoint log transform. |
| `computable_transcendentals/asinh_zero_cold_p128` | 37.07 ns | 35.58 ns - 38.75 ns | Approximates asinh(0) expression. |
| `computable_transcendentals/atanh_zero_cold_p128` | 33.60 ns | 33.45 ns - 33.78 ns | Approximates atanh(0) expression. |
| `computable_transcendentals/deep_add_chain_cold_p128` | 108.58 ns | 83.24 ns - 158.62 ns | Approximates a 5000-node addition chain. |
| `computable_transcendentals/deep_multiply_chain_cold_p128` | 80.88 ns | 77.29 ns - 84.80 ns | Approximates a 5000-node multiply-by-one chain. |
| `computable_transcendentals/deep_multiply_identity_chain_cold_p128` | 90.36 ns | 89.25 ns - 91.83 ns | Approximates a deep identity multiplication chain around pi. |
| `computable_transcendentals/deep_scaled_product_chain_cold_p128` | 672.89 ns | 574.16 ns - 868.53 ns | Approximates a deep product of exact scale factors. |
| `computable_transcendentals/perturbed_scaled_product_chain_cold_p128` | 844.04 ns | 839.99 ns - 847.57 ns | Approximates a deep scaled product with a tiny perturbation. |
| `computable_transcendentals/scaled_square_chain_cold_p128` | 1.286 us | 1.282 us - 1.289 us | Approximates repeated squaring of a scaled irrational. |
| `computable_transcendentals/asymmetric_product_bad_order_cold_p128` | 906.02 ns | 870.56 ns - 963.80 ns | Approximates an asymmetric product order stress case. |
| `computable_transcendentals/sqrt_scaled_square_chain_cold_p128` | 1.087 us | 1.077 us - 1.098 us | Approximates sqrt of a scaled-square chain. |
| `computable_transcendentals/warmed_zero_product_cold_p128` | 450.93 ns | 449.70 ns - 452.45 ns | Approximates a product involving a warmed zero sum. |
| `computable_transcendentals/inverse_scaled_product_chain_cold_p128` | 860.83 ns | 757.13 ns - 1.066 us | Approximates the inverse of a deep scaled product. |
| `computable_transcendentals/deep_inverse_pair_chain_cold_p128` | 87.23 ns | 86.96 ns - 87.62 ns | Approximates a chain of inverse(inverse(x)) pairs. |
| `computable_transcendentals/deep_negated_square_chain_cold_p128` | 87.23 ns | 87.07 ns - 87.45 ns | Approximates repeated negate-square-sqrt transformations. |
| `computable_transcendentals/deep_negative_one_product_chain_cold_p128` | 88.33 ns | 87.71 ns - 89.30 ns | Approximates repeated multiplication by -1. |
| `computable_transcendentals/deep_half_product_chain_cold_p128` | 149.81 ns | 148.82 ns - 150.86 ns | Approximates repeated multiplication by 1/2. |
| `computable_transcendentals/deep_half_square_chain_cold_p128` | 933.55 ns | 928.33 ns - 938.47 ns | Approximates repeated squaring after scaling by 1/2. |
| `computable_transcendentals/deep_sqrt_square_chain_cold_p128` | 107.00 ns | 81.89 ns - 156.60 ns | Approximates repeated sqrt-square simplification. |
| `computable_transcendentals/inverse_half_product_chain_cold_p128` | 467.26 ns | 464.75 ns - 469.62 ns | Approximates the inverse of a deep half-product chain. |

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
| `structural_query_speed/zero_zero_status` | 0.71 ns | 0.71 ns - 0.72 ns | Checks zero/nonzero facts for exact zero. |
| `structural_query_speed/zero_sign_query` | 3.99 ns | 3.99 ns - 4.00 ns | Reads sign facts for exact zero. |
| `structural_query_speed/zero_msd_query` | 5.94 ns | 5.94 ns - 5.95 ns | Reads magnitude facts for exact zero. |
| `structural_query_speed/zero_structural_facts` | 6.61 ns | 6.60 ns - 6.61 ns | Computes full structural facts for exact zero. |
| `structural_query_speed/one_zero_status` | 0.89 ns | 0.89 ns - 0.89 ns | Checks zero/nonzero facts for exact one. |
| `structural_query_speed/one_sign_query` | 21.91 ns | 21.86 ns - 21.96 ns | Reads sign facts for exact one. |
| `structural_query_speed/one_msd_query` | 22.86 ns | 22.82 ns - 22.91 ns | Reads magnitude facts for exact one. |
| `structural_query_speed/one_structural_facts` | 24.06 ns | 24.02 ns - 24.09 ns | Computes full structural facts for exact one. |
| `structural_query_speed/negative_zero_status` | 0.89 ns | 0.89 ns - 0.90 ns | Checks zero/nonzero facts for an exact negative integer. |
| `structural_query_speed/negative_sign_query` | 22.11 ns | 22.07 ns - 22.16 ns | Reads sign facts for an exact negative integer. |
| `structural_query_speed/negative_msd_query` | 24.11 ns | 24.04 ns - 24.19 ns | Reads magnitude facts for an exact negative integer. |
| `structural_query_speed/negative_structural_facts` | 26.88 ns | 26.82 ns - 26.95 ns | Computes full structural facts for an exact negative integer. |
| `structural_query_speed/tiny_exact_zero_status` | 0.89 ns | 0.89 ns - 0.89 ns | Checks zero/nonzero facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_sign_query` | 25.28 ns | 25.22 ns - 25.35 ns | Reads sign facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_msd_query` | 27.96 ns | 27.91 ns - 28.02 ns | Reads magnitude facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_structural_facts` | 32.02 ns | 31.96 ns - 32.09 ns | Computes full structural facts for a tiny exact rational. |
| `structural_query_speed/pi_zero_status` | 0.90 ns | 0.89 ns - 0.90 ns | Checks zero/nonzero facts for pi. |
| `structural_query_speed/pi_sign_query` | 34.16 ns | 34.08 ns - 34.25 ns | Reads sign facts for pi. |
| `structural_query_speed/pi_msd_query` | 38.93 ns | 38.88 ns - 39.00 ns | Reads magnitude facts for pi. |
| `structural_query_speed/pi_structural_facts` | 35.85 ns | 35.78 ns - 35.92 ns | Computes full structural facts for pi. |
| `structural_query_speed/e_zero_status` | 0.90 ns | 0.89 ns - 0.90 ns | Checks zero/nonzero facts for e. |
| `structural_query_speed/e_sign_query` | 34.43 ns | 34.23 ns - 34.67 ns | Reads sign facts for e. |
| `structural_query_speed/e_msd_query` | 39.11 ns | 39.02 ns - 39.22 ns | Reads magnitude facts for e. |
| `structural_query_speed/e_structural_facts` | 36.00 ns | 35.86 ns - 36.16 ns | Computes full structural facts for e. |
| `structural_query_speed/tau_zero_status` | 0.89 ns | 0.89 ns - 0.89 ns | Checks zero/nonzero facts for tau. |
| `structural_query_speed/tau_sign_query` | 33.63 ns | 33.56 ns - 33.71 ns | Reads sign facts for tau. |
| `structural_query_speed/tau_msd_query` | 39.18 ns | 39.13 ns - 39.25 ns | Reads magnitude facts for tau. |
| `structural_query_speed/tau_structural_facts` | 36.89 ns | 36.71 ns - 37.11 ns | Computes full structural facts for tau. |
| `structural_query_speed/sqrt_two_zero_status` | 0.89 ns | 0.89 ns - 0.90 ns | Checks zero/nonzero facts for sqrt(2). |
| `structural_query_speed/sqrt_two_sign_query` | 34.45 ns | 34.31 ns - 34.60 ns | Reads sign facts for sqrt(2). |
| `structural_query_speed/sqrt_two_msd_query` | 39.24 ns | 39.12 ns - 39.38 ns | Reads magnitude facts for sqrt(2). |
| `structural_query_speed/sqrt_two_structural_facts` | 35.89 ns | 35.84 ns - 35.94 ns | Computes full structural facts for sqrt(2). |
| `structural_query_speed/pi_minus_three_zero_status` | 0.89 ns | 0.89 ns - 0.89 ns | Checks zero/nonzero facts for pi - 3. |
| `structural_query_speed/pi_minus_three_sign_query` | 34.13 ns | 34.06 ns - 34.22 ns | Reads sign facts for pi - 3. |
| `structural_query_speed/pi_minus_three_msd_query` | 39.15 ns | 39.03 ns - 39.29 ns | Reads magnitude facts for pi - 3. |
| `structural_query_speed/pi_minus_three_structural_facts` | 36.36 ns | 36.15 ns - 36.59 ns | Computes full structural facts for pi - 3. |
| `structural_query_speed/dense_expr_zero_status` | 3.31 ns | 3.30 ns - 3.31 ns | Checks zero/nonzero facts for a dense composite expression. |
| `structural_query_speed/dense_expr_sign_query` | 34.03 ns | 34.01 ns - 34.06 ns | Reads sign facts for a dense composite expression. |
| `structural_query_speed/dense_expr_msd_query` | 39.25 ns | 39.20 ns - 39.32 ns | Reads magnitude facts for a dense composite expression. |
| `structural_query_speed/dense_expr_structural_facts` | 37.63 ns | 37.55 ns - 37.73 ns | Computes full structural facts for a dense composite expression. |

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
| `pure_scalar_algorithm_speed/real_exact_sqrt_reduce` | 409.01 ns | 408.35 ns - 409.73 ns | Reduces an exact square-root expression. |
| `pure_scalar_algorithm_speed/real_exact_ln_reduce` | 228.47 ns | 228.06 ns - 228.95 ns | Reduces an exact logarithm of a power of two. |

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
| `borrowed_op_overhead/real_dot4_refs_dense_symbolic` | 6.472 us | 6.447 us - 6.500 us | Computes a borrowed four-lane symbolic dot product with no rational shortcut terms. |
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
| `exact_transcendental_special_forms/atanh_sqrt_half` | 5.605 us | 4.822 us - 7.037 us | Builds atanh(sqrt(2)/2) after exact structural domain checks. |

### `symbolic_reductions`

Existing symbolic constant algebra cases considered for additional reductions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `symbolic_reductions/sqrt_pi_square` | 131.95 ns | 131.14 ns - 132.92 ns | Reduces sqrt(pi^2). |
| `symbolic_reductions/sqrt_pi_e_square` | 173.85 ns | 172.69 ns - 174.99 ns | Reduces sqrt((pi * e)^2). |
| `symbolic_reductions/ln_scaled_e` | 1.771 us | 1.758 us - 1.788 us | Reduces ln(2 * e). |
| `symbolic_reductions/sub_pi_three` | 1.792 us | 1.783 us - 1.805 us | Builds the certified pi - 3 constant-offset form. |
| `symbolic_reductions/pi_minus_three_facts` | 35.89 ns | 35.76 ns - 36.04 ns | Reads structural facts for the cached pi - 3 offset form. |
| `symbolic_reductions/div_exp_exp` | 779.54 ns | 597.37 ns - 1.141 us | Reduces e^3 / e. |
| `symbolic_reductions/div_pi_square_e` | 502.41 ns | 498.33 ns - 507.31 ns | Reduces pi^2 / e. |
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

<!-- BEGIN library_perf -->
## `library_perf`

Library-level Criterion benchmarks for public `Rational`, `Real`, and `Simple` behavior.

### `real_format`

Formatting costs for important irrational `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_format/pi_lower_exp_32` | not run | not run | Formats pi with 32 digits in lower-exponential form. |
| `real_format/pi_display_alt_32` | not run | not run | Formats pi with alternate decimal display at 32 digits. |
| `real_format/sqrt_two_display_alt_32` | not run | not run | Formats sqrt(2) with alternate decimal display at 32 digits. |

### `real_constants`

Construction cost for shared mathematical constants.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_constants/pi` | not run | not run | Constructs the symbolic pi value. |
| `real_constants/e` | not run | not run | Constructs the symbolic Euler constant value. |

### `simple`

Parser and evaluator costs for the `Simple` expression language.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple/parse_nested` | not run | not run | Parses a nested expression with powers, trig, and constants. |
| `simple/eval_nested` | not run | not run | Evaluates a parsed mixed symbolic/numeric expression. |
| `simple/eval_constants` | not run | not run | Evaluates repeated built-in constants. |
| `simple/eval_exact` | not run | not run | Evaluates a rational-only expression through exact shortcuts. |
| `simple/eval_nested_exact` | not run | not run | Evaluates a nested rational-only expression through exact shortcuts. |

### `real_powi`

Integer exponentiation for exact and irrational `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_powi/exact_17` | not run | not run | Raises an exact rational-backed `Real` to the 17th power. |
| `real_powi/irrational_17` | not run | not run | Raises sqrt(3) to the 17th power with symbolic simplification. |

### `rational_powi`

Integer exponentiation for `Rational`.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `rational_powi/exact_17` | not run | not run | Raises a rational value to the 17th power. |

### `real_exact_trig`

Exact and symbolic trig construction for known pi multiples.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_trig/sin_pi_6` | 162.60 ns | 162.12 ns - 163.17 ns | Computes sin(pi/6) via exact shortcut. |
| `real_exact_trig/cos_pi_3` | 366.55 ns | 365.55 ns - 367.76 ns | Computes cos(pi/3) via exact shortcut. |
| `real_exact_trig/tan_pi_5` | 316.49 ns | 316.05 ns - 317.01 ns | Builds tan(pi/5), a nontrivial symbolic tangent. |

### `real_general_trig`

General trig construction for irrational arguments.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_general_trig/tan_sqrt_2` | 940.29 ns | 938.28 ns - 942.45 ns | Builds tan(sqrt(2)). |
| `real_general_trig/tan_pi_sqrt_2_over_5` | 1.391 us | 1.389 us - 1.394 us | Builds tangent of an irrational multiple of pi. |

### `real_exact_inverse_trig`

Exact inverse trig shortcuts and symbolic inverse trig recognition.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_inverse_trig/asin_1_2` | not run | not run | Recognizes asin(1/2) as pi/6. |
| `real_exact_inverse_trig/asin_minus_1_2` | not run | not run | Recognizes asin(-1/2) as -pi/6. |
| `real_exact_inverse_trig/asin_sqrt_2_over_2` | 267.03 ns | 266.25 ns - 267.84 ns | Recognizes asin(sqrt(2)/2) as pi/4. |
| `real_exact_inverse_trig/asin_sin_pi_5` | not run | not run | Inverts a symbolic sin(pi/5). |
| `real_exact_inverse_trig/acos_1` | not run | not run | Recognizes acos(1) as zero. |
| `real_exact_inverse_trig/acos_minus_1` | not run | not run | Recognizes acos(-1) as pi. |
| `real_exact_inverse_trig/acos_1_2` | not run | not run | Recognizes acos(1/2) as pi/3. |
| `real_exact_inverse_trig/atan_1` | not run | not run | Recognizes atan(1) as pi/4. |
| `real_exact_inverse_trig/atan_sqrt_3_over_3` | 418.19 ns | 416.67 ns - 419.52 ns | Recognizes atan(sqrt(3)/3) as pi/6. |
| `real_exact_inverse_trig/atan_tan_pi_5` | not run | not run | Inverts a symbolic tan(pi/5). |

### `real_general_inverse_trig`

General inverse trig construction, domain errors, and atan range reduction.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_general_inverse_trig/asin_7_10` | not run | not run | Builds asin(7/10) through the rational-specialized path. |
| `real_general_inverse_trig/asin_sqrt_2_over_3` | 4.985 us | 4.866 us - 5.132 us | Builds asin(sqrt(2)/3) through the general path. |
| `real_general_inverse_trig/acos_7_10` | not run | not run | Builds acos(7/10) through the rational-specialized asin path. |
| `real_general_inverse_trig/acos_sqrt_2_over_3` | 340.13 ns | 329.17 ns - 352.75 ns | Builds acos(sqrt(2)/3) through the general path. |
| `real_general_inverse_trig/asin_11_10_error` | not run | not run | Rejects rational asin input outside [-1, 1]. |
| `real_general_inverse_trig/acos_11_10_error` | not run | not run | Rejects rational acos input outside [-1, 1]. |
| `real_general_inverse_trig/atan_8` | not run | not run | Builds atan(8), exercising large-argument reduction. |
| `real_general_inverse_trig/atan_sqrt_2` | not run | not run | Builds atan(sqrt(2)). |

### `real_inverse_hyperbolic`

Inverse hyperbolic construction, exact exits, stable ln1p forms, and domain errors.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_inverse_hyperbolic/asinh_0` | not run | not run | Recognizes asinh(0) as zero. |
| `real_inverse_hyperbolic/asinh_1_2` | not run | not run | Builds asinh(1/2) through the stable moderate-input path. |
| `real_inverse_hyperbolic/asinh_sqrt_2` | not run | not run | Builds asinh(sqrt(2)) without cancellation-prone log construction. |
| `real_inverse_hyperbolic/asinh_minus_1_2` | not run | not run | Uses odd symmetry for negative asinh input. |
| `real_inverse_hyperbolic/asinh_1_000_000` | not run | not run | Builds asinh for a large positive rational. |
| `real_inverse_hyperbolic/acosh_1` | not run | not run | Recognizes acosh(1) as zero. |
| `real_inverse_hyperbolic/acosh_2` | not run | not run | Builds acosh(2) through the stable moderate-input path. |
| `real_inverse_hyperbolic/acosh_sqrt_2` | 292.34 ns | 291.46 ns - 293.28 ns | Builds acosh(sqrt(2)) through square-root domain specialization. |
| `real_inverse_hyperbolic/acosh_1_000_000` | not run | not run | Builds acosh for a large positive rational. |
| `real_inverse_hyperbolic/atanh_0` | not run | not run | Recognizes atanh(0) as zero. |
| `real_inverse_hyperbolic/atanh_1_2` | 1.025 us | 1.017 us - 1.038 us | Builds exact-rational atanh(1/2). |
| `real_inverse_hyperbolic/atanh_minus_1_2` | 1.211 us | 1.170 us - 1.258 us | Builds exact-rational atanh(-1/2). |
| `real_inverse_hyperbolic/atanh_9_10` | 227.95 ns | 218.52 ns - 238.89 ns | Builds exact-rational atanh near the upper domain boundary. |
| `real_inverse_hyperbolic/atanh_1_error` | not run | not run | Rejects atanh(1) at the rational domain boundary. |

### `simple_inverse_functions`

Parsed/evaluated inverse trig and inverse hyperbolic expressions that should succeed.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple_inverse_functions/asin_1_2` | not run | not run | Evaluates `(asin 1/2)`. |
| `simple_inverse_functions/acos_1_2` | not run | not run | Evaluates `(acos 1/2)`. |
| `simple_inverse_functions/atan_1` | not run | not run | Evaluates `(atan 1)`. |
| `simple_inverse_functions/asin_general` | not run | not run | Evaluates `(asin 7/10)`. |
| `simple_inverse_functions/acos_general` | not run | not run | Evaluates `(acos 7/10)`. |
| `simple_inverse_functions/atan_general` | not run | not run | Evaluates `(atan 8)`. |
| `simple_inverse_functions/asinh_1_2` | not run | not run | Evaluates `(asinh 1/2)`. |
| `simple_inverse_functions/asinh_sqrt_2` | not run | not run | Evaluates `(asinh (sqrt 2))`. |
| `simple_inverse_functions/acosh_2` | not run | not run | Evaluates `(acosh 2)`. |
| `simple_inverse_functions/acosh_sqrt_2` | 1.133 us | 1.129 us - 1.136 us | Evaluates `(acosh (sqrt 2))`. |
| `simple_inverse_functions/atanh_1_2` | 1.169 us | 1.123 us - 1.223 us | Evaluates `(atanh 1/2)`. |
| `simple_inverse_functions/atanh_minus_1_2` | 1.184 us | 1.182 us - 1.187 us | Evaluates `(atanh -1/2)`. |

### `simple_inverse_error_functions`

Parsed/evaluated inverse function expressions that should fail quickly with `NotANumber`.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple_inverse_error_functions/asin_11_10` | not run | not run | Rejects `(asin 11/10)`. |
| `simple_inverse_error_functions/acos_sqrt_2` | not run | not run | Rejects `(acos (sqrt 2))`. |
| `simple_inverse_error_functions/acosh_0` | not run | not run | Rejects `(acosh 0)`. |
| `simple_inverse_error_functions/acosh_minus_2` | not run | not run | Rejects `(acosh -2)`. |
| `simple_inverse_error_functions/atanh_1` | not run | not run | Rejects `(atanh 1)`. |
| `simple_inverse_error_functions/atanh_sqrt_2` | 918.70 ns | 916.70 ns - 920.72 ns | Rejects `(atanh (sqrt 2))`. |

### `real_exact_ln`

Exact logarithm construction and simplification for rational inputs.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_ln/ln_1024` | not run | not run | Recognizes ln(1024) as 10 ln(2). |
| `real_exact_ln/ln_1_8` | not run | not run | Recognizes ln(1/8) as -3 ln(2). |
| `real_exact_ln/ln_1000` | not run | not run | Simplifies ln(1000) via small integer logarithm factors. |

### `real_exact_exp_log10`

Exact inverse relationships among exp, ln, and log10.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_exp_log10/exp_ln_1000` | 344.84 ns | 343.97 ns - 345.69 ns | Simplifies exp(ln(1000)) back to 1000. |
| `real_exact_exp_log10/exp_ln_1_8` | 362.09 ns | 361.60 ns - 362.63 ns | Simplifies exp(ln(1/8)) back to 1/8. |
| `real_exact_exp_log10/log10_1000` | 103.36 ns | 103.09 ns - 103.72 ns | Recognizes log10(1000) as 3. |
| `real_exact_exp_log10/log10_1_1000` | 125.70 ns | 124.55 ns - 127.24 ns | Recognizes log10(1/1000) as -3. |

<!-- END library_perf -->

