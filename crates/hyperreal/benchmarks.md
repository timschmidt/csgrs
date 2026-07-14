<!-- BEGIN promoted_slow_offender_score -->
## `promoted_slow_offender_score`

Deterministic lexicase score for the current 100 promoted slow offenders. The score is the average current best-of-five wall-clock probe across the promoted set; lower is better. Delta compares with the previous score recorded in this file, and derivative is the change in delta.

<!-- promoted_slow_score_nanos: 2982 -->
<!-- promoted_slow_previous_score_nanos: 2982 -->
<!-- promoted_slow_score_delta_nanos: 0 -->

| Metric | Value |
| --- | ---: |
| Cases scored | 100 |
| Average score | 2.982 us |
| Delta | 0 ns |
| Delta derivative | 0 ns |

| Rank | Current Time | Operation | Input |
| ---: | ---: | --- | --- |
| 1 | 4.110 us | `generated_ln_abs_plus_one_p96` | `generated[15472] -3 13/50` |
| 2 | 4.109 us | `generated_tan_p96` | `generated[11841] -5 2/17` |
| 3 | 4.070 us | `generated_tan_p96` | `generated[13446] -5 15/187` |
| 4 | 4.069 us | `generated_ln_abs_plus_one_p96` | `generated[9457] -3 23/90` |
| 5 | 4.029 us | `generated_tan_p96` | `generated[18666] 5 15/17` |
| 6 | 4.000 us | `generated_tan_p96` | `generated[16806] 5 3/22` |
| 7 | 3.950 us | `generated_tan_p96` | `generated[15891] -5 23/33` |
| 8 | 3.920 us | `generated_ln_abs_plus_one_p96` | `generated[6877] -9 34/77` |
| 9 | 3.900 us | `generated_tan_p96` | `generated[3321] -4 17/107` |
| 10 | 3.879 us | `generated_ln_abs_plus_one_p96` | `generated[18352] -1 133/500` |

<!-- END promoted_slow_offender_score -->









<!-- BEGIN numerical_micro -->
## `numerical_micro`

Low-level `Computable` microbenchmarks for approximation kernels, caches, structural facts, comparisons, and deep evaluator trees.

### `computable_cache`

Cold versus cached approximation of basic `Computable` expressions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_cache/ratio_approx_cold_p128` | not run | not run | Approximates a rational value at p=-128 from a fresh clone. |
| `computable_cache/ratio_approx_cached_p128` | not run | not run | Repeats an already cached rational approximation at p=-128. |
| `computable_cache/pi_approx_cold_p128` | not run | not run | Approximates pi at p=-128 from a fresh clone. |
| `computable_cache/pi_approx_cached_p128` | not run | not run | Repeats an already cached pi approximation at p=-128. |
| `computable_cache/pi_plus_tiny_cold_p128` | not run | not run | Approximates pi plus a tiny exact rational perturbation. |
| `computable_cache/pi_minus_tiny_cold_p128` | not run | not run | Approximates pi minus a tiny exact rational perturbation. |

### `computable_bounds`

Structural sign and bound discovery for deep or perturbed computable trees.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_bounds/deep_scaled_product_sign` | not run | not run | Finds the sign of a deep scaled product. |
| `computable_bounds/scaled_square_sign` | not run | not run | Finds the sign of repeated squaring with exact scale factors. |
| `computable_bounds/sqrt_scaled_square_sign` | not run | not run | Finds the sign after taking a square root of a scaled square. |
| `computable_bounds/deep_structural_bound_sign` | not run | not run | Finds sign through repeated multiply/inverse/negate structural transformations. |
| `computable_bounds/deep_structural_bound_sign_cached` | not run | not run | Reads the cached sign of the deep structural-bound chain. |
| `computable_bounds/deep_structural_bound_facts_cached` | not run | not run | Reads cached structural facts for the deep structural-bound chain. |
| `computable_bounds/perturbed_scaled_product_sign` | not run | not run | Finds sign for a deeply scaled value with a tiny perturbation. |
| `computable_bounds/perturbed_scaled_product_sign_until` | not run | not run | Refines sign for the perturbed scaled product only to p=-128. |
| `computable_bounds/pi_minus_tiny_sign` | not run | not run | Finds sign for pi minus a tiny exact rational. |
| `computable_bounds/pi_minus_tiny_sign_cached` | not run | not run | Reads cached sign for pi minus a tiny exact rational. |
| `computable_bounds/exp_unknown_sign_arg_sign` | not run | not run | Finds sign for exp(1 - pi), where exp can prove positivity structurally. |
| `computable_bounds/exp_unknown_sign_arg_sign_cached` | not run | not run | Reads cached sign for exp(1 - pi). |

### `computable_compare`

Ordering and absolute-comparison shortcuts.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_compare/compare_to_opposite_sign` | not run | not run | Compares values with known opposite signs. |
| `computable_compare/compare_to_exact_msd_gap` | not run | not run | Compares values with a large exact magnitude gap. |
| `computable_compare/compare_absolute_exact_rational` | not run | not run | Compares absolute values of exact rationals. |
| `computable_compare/compare_absolute_exact_rational_same_numerator` | not run | not run | Compares exact rational magnitudes with matching numerators. |
| `computable_compare/compare_absolute_dominant_add` | not run | not run | Compares a dominant term against the same term plus a tiny addend. |
| `computable_compare/compare_absolute_exact_msd_gap` | not run | not run | Compares absolute values with a large exact magnitude gap. |

### `computable_transcendentals`

Low-level approximation kernels and deep expression-tree stress cases.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_transcendentals/e_constant_cold_p128` | not run | not run | Approximates the shared e constant from a fresh clone. |
| `computable_transcendentals/e_constant_cached_p128` | not run | not run | Repeats a cached approximation of e. |
| `computable_transcendentals/exp_cold_p128` | not run | not run | Approximates exp(7/5) from a fresh clone. |
| `computable_transcendentals/exp_cached_p128` | not run | not run | Repeats a cached exp(7/5) approximation. |
| `computable_transcendentals/exp_large_cold_p128` | not run | not run | Approximates exp(128), exercising large-argument reduction. |
| `computable_transcendentals/exp_half_cold_p128` | not run | not run | Approximates exp(1/2). |
| `computable_transcendentals/exp_near_limit_cold_p128` | not run | not run | Approximates exp near a prescaling threshold. |
| `computable_transcendentals/exp_near_limit_cached_p128` | not run | not run | Repeats a cached near-threshold exp approximation. |
| `computable_transcendentals/exp_zero_cold_p128` | not run | not run | Approximates exp(0). |
| `computable_transcendentals/ln_cold_p128` | not run | not run | Approximates ln(11/7). |
| `computable_transcendentals/ln_cached_p128` | not run | not run | Repeats a cached ln(11/7) approximation. |
| `computable_transcendentals/ln_smooth_rational_cold_p128` | not run | not run | Approximates ln(45/14), which can decompose into shared prime-log constants. |
| `computable_transcendentals/ln_nonsmooth_rational_cold_p128` | not run | not run | Approximates ln(11/13), guarding the generic exact-rational log fallback. |
| `computable_transcendentals/ln_large_cold_p128` | not run | not run | Approximates ln(1024), exercising large-input reduction. |
| `computable_transcendentals/ln_large_cached_p128` | not run | not run | Repeats a cached ln(1024) approximation. |
| `computable_transcendentals/ln_tiny_cold_p128` | not run | not run | Approximates ln(2^-1024), exercising tiny-input reduction. |
| `computable_transcendentals/ln_near_limit_cold_p128` | not run | not run | Approximates ln near the prescaled-ln limit. |
| `computable_transcendentals/ln_near_limit_cached_p128` | not run | not run | Repeats a cached near-limit ln approximation. |
| `computable_transcendentals/ln_one_cold_p128` | not run | not run | Approximates ln(1). |
| `computable_transcendentals/sqrt_cold_p128` | not run | not run | Approximates sqrt(2). |
| `computable_transcendentals/sqrt_squarefree_scaled_cold_p128` | not run | not run | Approximates sqrt(12), which can reduce to 2*sqrt(3). |
| `computable_transcendentals/sqrt_cached_p128` | not run | not run | Repeats a cached sqrt(2) approximation. |
| `computable_transcendentals/sqrt_single_scaled_square_cold_p128` | not run | not run | Builds and approximates sqrt((7*pi/8)^2). |
| `computable_transcendentals/sin_cold_p96` | not run | not run | Approximates sin(7/5). |
| `computable_transcendentals/sin_cached_p96` | not run | not run | Repeats a cached sin(7/5) approximation. |
| `computable_transcendentals/cos_cold_p96` | not run | not run | Approximates cos(7/5). |
| `computable_transcendentals/sin_f64_cold_p96` | not run | not run | Approximates sin(1.23456789 imported exactly from f64). |
| `computable_transcendentals/cos_f64_cold_p96` | not run | not run | Approximates cos(1.23456789 imported exactly from f64). |
| `computable_transcendentals/sin_1e6_cold_p96` | not run | not run | Approximates sin(1000000). |
| `computable_transcendentals/cos_1e6_cold_p96` | not run | not run | Approximates cos(1000000). |
| `computable_transcendentals/sin_1e30_cold_p96` | not run | not run | Approximates sin(10^30). |
| `computable_transcendentals/cos_1e30_cold_p96` | not run | not run | Approximates cos(10^30). |
| `computable_transcendentals/cos_cached_p96` | not run | not run | Repeats a cached cos(7/5) approximation. |
| `computable_transcendentals/tan_cold_p96` | not run | not run | Approximates tan(7/5). |
| `computable_transcendentals/tan_cached_p96` | not run | not run | Repeats a cached tan(7/5) approximation. |
| `computable_transcendentals/sin_zero_cold_p96` | not run | not run | Approximates sin(0). |
| `computable_transcendentals/cos_zero_cold_p96` | not run | not run | Approximates cos(0). |
| `computable_transcendentals/tan_zero_cold_p96` | not run | not run | Approximates tan(0). |
| `computable_transcendentals/tan_near_half_pi_cold_p96` | not run | not run | Approximates tangent near pi/2. |
| `computable_transcendentals/tan_near_half_pi_cached_p96` | not run | not run | Repeats cached tangent near pi/2. |
| `computable_transcendentals/sin_huge_cold_p96` | not run | not run | Approximates sine of a huge pi multiple plus offset. |
| `computable_transcendentals/cos_huge_cold_p96` | not run | not run | Approximates cosine of a huge pi multiple plus offset. |
| `computable_transcendentals/tan_huge_cold_p96` | not run | not run | Approximates tangent of a huge pi multiple plus offset. |
| `computable_transcendentals/asin_cold_p96` | not run | not run | Approximates a computable asin expression. |
| `computable_transcendentals/asin_cached_p96` | not run | not run | Repeats a cached computable asin approximation. |
| `computable_transcendentals/acos_cold_p96` | not run | not run | Approximates a computable acos expression. |
| `computable_transcendentals/acos_cached_p96` | not run | not run | Repeats a cached computable acos approximation. |
| `computable_transcendentals/asin_tiny_cold_p96` | not run | not run | Approximates asin(1e-12), exercising the tiny-input series. |
| `computable_transcendentals/acos_tiny_cold_p96` | not run | not run | Approximates acos(1e-12), exercising the tiny-input complement. |
| `computable_transcendentals/asin_near_one_cold_p96` | not run | not run | Approximates asin(0.999999), exercising the endpoint complement. |
| `computable_transcendentals/acos_near_one_cold_p96` | not run | not run | Approximates acos(0.999999), exercising the endpoint transform. |
| `computable_transcendentals/atan_cold_p96` | not run | not run | Approximates atan(7/10). |
| `computable_transcendentals/atan_cached_p96` | not run | not run | Repeats a cached atan(7/10) approximation. |
| `computable_transcendentals/atan_large_cold_p96` | not run | not run | Approximates atan(8), exercising argument reduction. |
| `computable_transcendentals/asin_zero_cold_p96` | not run | not run | Approximates asin(0) expression. |
| `computable_transcendentals/atan_zero_cold_p96` | not run | not run | Approximates atan(0). |
| `computable_transcendentals/asinh_cold_p128` | not run | not run | Approximates a computable asinh expression. |
| `computable_transcendentals/asinh_cached_p128` | not run | not run | Repeats a cached computable asinh approximation. |
| `computable_transcendentals/acosh_cold_p128` | not run | not run | Approximates a computable acosh expression. |
| `computable_transcendentals/acosh_cached_p128` | not run | not run | Repeats a cached computable acosh approximation. |
| `computable_transcendentals/atanh_cold_p128` | not run | not run | Approximates a computable atanh expression. |
| `computable_transcendentals/atanh_cached_p128` | not run | not run | Repeats a cached computable atanh approximation. |
| `computable_transcendentals/atanh_tiny_cold_p128` | not run | not run | Approximates atanh(1e-12), exercising the tiny-input series. |
| `computable_transcendentals/atanh_near_one_cold_p128` | not run | not run | Approximates atanh(0.999999), exercising the endpoint log transform. |
| `computable_transcendentals/asinh_zero_cold_p128` | not run | not run | Approximates asinh(0) expression. |
| `computable_transcendentals/atanh_zero_cold_p128` | not run | not run | Approximates atanh(0) expression. |
| `computable_transcendentals/deep_add_chain_cold_p128` | not run | not run | Approximates a 5000-node addition chain. |
| `computable_transcendentals/deep_multiply_chain_cold_p128` | not run | not run | Approximates a 5000-node multiply-by-one chain. |
| `computable_transcendentals/deep_multiply_identity_chain_cold_p128` | not run | not run | Approximates a deep identity multiplication chain around pi. |
| `computable_transcendentals/deep_scaled_product_chain_cold_p128` | not run | not run | Approximates a deep product of exact scale factors. |
| `computable_transcendentals/perturbed_scaled_product_chain_cold_p128` | not run | not run | Approximates a deep scaled product with a tiny perturbation. |
| `computable_transcendentals/scaled_square_chain_cold_p128` | not run | not run | Approximates repeated squaring of a scaled irrational. |
| `computable_transcendentals/asymmetric_product_bad_order_cold_p128` | not run | not run | Approximates an asymmetric product order stress case. |
| `computable_transcendentals/sqrt_scaled_square_chain_cold_p128` | not run | not run | Approximates sqrt of a scaled-square chain. |
| `computable_transcendentals/warmed_zero_product_cold_p128` | not run | not run | Approximates a product involving a warmed zero sum. |
| `computable_transcendentals/inverse_scaled_product_chain_cold_p128` | not run | not run | Approximates the inverse of a deep scaled product. |
| `computable_transcendentals/deep_inverse_pair_chain_cold_p128` | not run | not run | Approximates a chain of inverse(inverse(x)) pairs. |
| `computable_transcendentals/deep_negated_square_chain_cold_p128` | not run | not run | Approximates repeated negate-square-sqrt transformations. |
| `computable_transcendentals/deep_negative_one_product_chain_cold_p128` | not run | not run | Approximates repeated multiplication by -1. |
| `computable_transcendentals/deep_half_product_chain_cold_p128` | not run | not run | Approximates repeated multiplication by 1/2. |
| `computable_transcendentals/deep_half_square_chain_cold_p128` | not run | not run | Approximates repeated squaring after scaling by 1/2. |
| `computable_transcendentals/deep_sqrt_square_chain_cold_p128` | not run | not run | Approximates repeated sqrt-square simplification. |
| `computable_transcendentals/inverse_half_product_chain_cold_p128` | not run | not run | Approximates the inverse of a deep half-product chain. |

<!-- END numerical_micro -->

<!-- BEGIN scalar_micro -->
## `scalar_micro`

Microbenchmarks for scalar operations, structural queries, cache hits, and dense exact arithmetic.

### `construction_speed`

Cost of constructing common exact scalar identities.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `construction_speed/rational_one` | 17.95 ns | 17.83 ns - 18.08 ns | Constructs `Rational::one()`. |
| `construction_speed/rational_new_one` | 26.35 ns | 26.17 ns - 26.56 ns | Constructs one through `Rational::new(1)`. |
| `construction_speed/computable_one` | 23.09 ns | 23.01 ns - 23.18 ns | Constructs `Computable::one()`. |
| `construction_speed/real_new_rational_one` | 79.93 ns | 79.65 ns - 80.22 ns | Constructs one through `Real::new(Rational::one())`. |
| `construction_speed/real_one` | 81.02 ns | 80.02 ns - 82.22 ns | Constructs one through `Real::one()`. |
| `construction_speed/real_from_i32_one` | 78.94 ns | 78.41 ns - 79.52 ns | Constructs one through integer conversion. |

### `raw_cache_hit_cost`

Cost of cold and cached `Computable::approx` calls for simple values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `raw_cache_hit_cost/zero` | 50.54 ns | 50.37 ns - 50.75 ns | Cached approximation request for exact zero. |
| `raw_cache_hit_cost/one` | 68.96 ns | 68.46 ns - 69.60 ns | Cached approximation request for exact one. |
| `raw_cache_hit_cost/two` | 68.29 ns | 68.03 ns - 68.57 ns | Cached approximation request for exact two. |
| `raw_cache_hit_cost/e` | 69.06 ns | 68.88 ns - 69.25 ns | Cached approximation request for Euler's constant. |
| `raw_cache_hit_cost/pi` | 70.66 ns | 70.25 ns - 71.10 ns | Cached approximation request for pi. |
| `raw_cache_hit_cost/tau` | 69.57 ns | 69.33 ns - 69.84 ns | Cached approximation request for two pi. |

### `structural_query_speed`

Speed of public structural queries across exact, transcendental, and composite `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `structural_query_speed/zero_zero_status` | 2.65 ns | 2.63 ns - 2.67 ns | Checks zero/nonzero facts for exact zero. |
| `structural_query_speed/zero_sign_query` | 7.12 ns | 7.11 ns - 7.14 ns | Reads sign facts for exact zero. |
| `structural_query_speed/zero_msd_query` | 8.58 ns | 8.51 ns - 8.66 ns | Reads magnitude facts for exact zero. |
| `structural_query_speed/zero_structural_facts` | 8.86 ns | 8.81 ns - 8.90 ns | Computes full structural facts for exact zero. |
| `structural_query_speed/one_zero_status` | 2.41 ns | 2.41 ns - 2.42 ns | Checks zero/nonzero facts for exact one. |
| `structural_query_speed/one_sign_query` | 23.37 ns | 23.33 ns - 23.42 ns | Reads sign facts for exact one. |
| `structural_query_speed/one_msd_query` | 25.63 ns | 25.47 ns - 25.81 ns | Reads magnitude facts for exact one. |
| `structural_query_speed/one_structural_facts` | 31.94 ns | 31.74 ns - 32.17 ns | Computes full structural facts for exact one. |
| `structural_query_speed/negative_zero_status` | 2.48 ns | 2.46 ns - 2.49 ns | Checks zero/nonzero facts for an exact negative integer. |
| `structural_query_speed/negative_sign_query` | 24.78 ns | 24.71 ns - 24.87 ns | Reads sign facts for an exact negative integer. |
| `structural_query_speed/negative_msd_query` | 27.20 ns | 27.08 ns - 27.34 ns | Reads magnitude facts for an exact negative integer. |
| `structural_query_speed/negative_structural_facts` | 29.84 ns | 29.77 ns - 29.94 ns | Computes full structural facts for an exact negative integer. |
| `structural_query_speed/tiny_exact_zero_status` | 2.41 ns | 2.40 ns - 2.42 ns | Checks zero/nonzero facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_sign_query` | 28.34 ns | 28.24 ns - 28.44 ns | Reads sign facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_msd_query` | 30.68 ns | 30.59 ns - 30.78 ns | Reads magnitude facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_structural_facts` | 33.39 ns | 33.28 ns - 33.51 ns | Computes full structural facts for a tiny exact rational. |
| `structural_query_speed/pi_zero_status` | 2.40 ns | 2.39 ns - 2.40 ns | Checks zero/nonzero facts for pi. |
| `structural_query_speed/pi_sign_query` | 37.56 ns | 37.36 ns - 37.79 ns | Reads sign facts for pi. |
| `structural_query_speed/pi_msd_query` | 42.99 ns | 42.73 ns - 43.28 ns | Reads magnitude facts for pi. |
| `structural_query_speed/pi_structural_facts` | 41.24 ns | 40.97 ns - 41.58 ns | Computes full structural facts for pi. |
| `structural_query_speed/e_zero_status` | 2.45 ns | 2.44 ns - 2.47 ns | Checks zero/nonzero facts for e. |
| `structural_query_speed/e_sign_query` | 38.41 ns | 38.13 ns - 38.70 ns | Reads sign facts for e. |
| `structural_query_speed/e_msd_query` | 42.90 ns | 42.73 ns - 43.08 ns | Reads magnitude facts for e. |
| `structural_query_speed/e_structural_facts` | 41.02 ns | 40.91 ns - 41.16 ns | Computes full structural facts for e. |
| `structural_query_speed/tau_zero_status` | 2.43 ns | 2.42 ns - 2.45 ns | Checks zero/nonzero facts for tau. |
| `structural_query_speed/tau_sign_query` | 37.58 ns | 37.42 ns - 37.76 ns | Reads sign facts for tau. |
| `structural_query_speed/tau_msd_query` | 43.12 ns | 42.95 ns - 43.32 ns | Reads magnitude facts for tau. |
| `structural_query_speed/tau_structural_facts` | 41.71 ns | 41.56 ns - 41.87 ns | Computes full structural facts for tau. |
| `structural_query_speed/sqrt_two_zero_status` | 2.42 ns | 2.41 ns - 2.43 ns | Checks zero/nonzero facts for sqrt(2). |
| `structural_query_speed/sqrt_two_sign_query` | 37.86 ns | 37.67 ns - 38.08 ns | Reads sign facts for sqrt(2). |
| `structural_query_speed/sqrt_two_msd_query` | 42.81 ns | 42.64 ns - 43.00 ns | Reads magnitude facts for sqrt(2). |
| `structural_query_speed/sqrt_two_structural_facts` | 41.49 ns | 41.27 ns - 41.72 ns | Computes full structural facts for sqrt(2). |
| `structural_query_speed/pi_minus_three_zero_status` | 2.41 ns | 2.41 ns - 2.42 ns | Checks zero/nonzero facts for pi - 3. |
| `structural_query_speed/pi_minus_three_sign_query` | 38.01 ns | 37.73 ns - 38.35 ns | Reads sign facts for pi - 3. |
| `structural_query_speed/pi_minus_three_msd_query` | 43.39 ns | 42.83 ns - 44.16 ns | Reads magnitude facts for pi - 3. |
| `structural_query_speed/pi_minus_three_structural_facts` | 43.31 ns | 42.60 ns - 44.17 ns | Computes full structural facts for pi - 3. |
| `structural_query_speed/dense_expr_zero_status` | 7.45 ns | 7.42 ns - 7.48 ns | Checks zero/nonzero facts for a dense composite expression. |
| `structural_query_speed/dense_expr_sign_query` | 37.99 ns | 37.88 ns - 38.12 ns | Reads sign facts for a dense composite expression. |
| `structural_query_speed/dense_expr_msd_query` | 43.33 ns | 43.12 ns - 43.56 ns | Reads magnitude facts for a dense composite expression. |
| `structural_query_speed/dense_expr_structural_facts` | 41.73 ns | 41.61 ns - 41.86 ns | Computes full structural facts for a dense composite expression. |

### `pure_scalar_algorithm_speed`

Core scalar algorithms that do not require high-precision transcendental approximation.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `pure_scalar_algorithm_speed/rational_add` | 388.11 ns | 386.21 ns - 390.15 ns | Adds two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_mul` | 127.21 ns | 126.42 ns - 128.13 ns | Multiplies two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_div` | 36.46 ns | 36.13 ns - 36.84 ns | Divides two nontrivial rational values. |
| `pure_scalar_algorithm_speed/real_exact_add` | 464.78 ns | 461.59 ns - 468.25 ns | Adds exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_mul` | 204.68 ns | 203.34 ns - 206.12 ns | Multiplies exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_div` | 123.45 ns | 122.72 ns - 124.25 ns | Divides exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_sqrt_reduce` | 208.93 ns | 207.81 ns - 210.13 ns | Reduces an exact square-root expression. |
| `pure_scalar_algorithm_speed/real_exact_ln_reduce` | 223.11 ns | 222.00 ns - 224.36 ns | Reduces an exact logarithm of a power of two. |
| `pure_scalar_algorithm_speed/real_pow_small_integer_exponent` | 322.96 ns | 321.06 ns - 325.00 ns | Dispatches `Real::pow` with an exact small-integer exponent. |

### `borrowed_op_overhead`

Borrowed versus owned operation overhead for rational and real operands.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `borrowed_op_overhead/rational_clone_pair` | 7.46 ns | 7.44 ns - 7.47 ns | Clones two rational values. |
| `borrowed_op_overhead/rational_add_refs` | 388.22 ns | 386.30 ns - 390.35 ns | Adds rational references. |
| `borrowed_op_overhead/rational_add_owned` | 434.13 ns | 430.77 ns - 437.96 ns | Adds owned rational values. |
| `borrowed_op_overhead/real_clone_pair` | 293.85 ns | 290.85 ns - 297.60 ns | Clones two scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_unscaled_add_refs` | 177.57 ns | 176.72 ns - 178.54 ns | Adds borrowed unscaled transcendental `Real` values. |
| `borrowed_op_overhead/real_unscaled_add_owned` | 212.13 ns | 210.41 ns - 214.07 ns | Adds owned unscaled transcendental `Real` values. |
| `borrowed_op_overhead/real_add_refs` | 407.38 ns | 405.83 ns - 409.07 ns | Adds borrowed scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_add_owned` | 413.43 ns | 410.71 ns - 416.46 ns | Adds owned scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_dot2_refs_dense_symbolic` | 1.424 us | 1.416 us - 1.433 us | Computes a borrowed two-lane symbolic dot product with no rational shortcut terms. |
| `borrowed_op_overhead/real_active_dot2_refs_dense_symbolic` | 1.516 us | 1.509 us - 1.525 us | Computes a borrowed two-lane symbolic dot product after the caller has already classified every lane active. |
| `borrowed_op_overhead/real_dot2_refs_mixed_structural` | 115.17 ns | 114.63 ns - 115.72 ns | Computes a borrowed two-lane symbolic dot product with an exact zero lane and a rational scale lane. |
| `borrowed_op_overhead/real_dot3_refs_dense_symbolic` | 3.277 us | 3.254 us - 3.303 us | Computes a borrowed three-lane symbolic dot product with no rational shortcut terms. |
| `borrowed_op_overhead/real_active_dot3_refs_dense_symbolic` | 3.326 us | 3.317 us - 3.336 us | Computes a borrowed three-lane symbolic dot product after the caller has already classified every lane active. |
| `borrowed_op_overhead/real_dot3_refs_mixed_structural` | 629.76 ns | 625.96 ns - 634.21 ns | Computes a borrowed three-lane symbolic dot product with exact zero and rational scale terms. |
| `borrowed_op_overhead/real_dot4_refs_dense_symbolic` | 6.307 us | 6.280 us - 6.341 us | Computes a borrowed four-lane symbolic dot product with no rational shortcut terms. |
| `borrowed_op_overhead/real_active_dot4_refs_dense_symbolic` | 5.949 us | 5.921 us - 5.979 us | Computes a borrowed four-lane symbolic dot product after the caller has already classified every lane active. |
| `borrowed_op_overhead/real_dot4_refs_mixed_structural` | 651.51 ns | 647.96 ns - 655.41 ns | Computes a borrowed four-lane symbolic dot product with exact zero and rational scale terms. |

### `dense_algebra`

Small dense algebra kernels that stress repeated exact and symbolic operations.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `dense_algebra/rational_dot_64` | 46.599 us | 46.202 us - 47.078 us | Computes a 64-element rational dot product. |
| `dense_algebra/rational_matmul_8` | 246.717 us | 245.381 us - 248.305 us | Computes an 8x8 rational matrix multiply. |
| `dense_algebra/real_dot_36` | 27.549 us | 27.319 us - 27.810 us | Computes a 36-element dot product over symbolic `Real` values. |
| `dense_algebra/real_matmul_6` | 153.585 us | 152.575 us - 154.869 us | Computes a 6x6 matrix multiply over symbolic `Real` values. |

### `exact_transcendental_special_forms`

Construction-time shortcuts for exact rational multiples of pi and inverse compositions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `exact_transcendental_special_forms/sin_pi_7` | 402.20 ns | 400.43 ns - 404.13 ns | Builds the exact special form for sin(pi/7). |
| `exact_transcendental_special_forms/cos_pi_7` | 700.87 ns | 698.60 ns - 703.43 ns | Builds the exact special form for cos(pi/7). |
| `exact_transcendental_special_forms/tan_pi_7` | 344.17 ns | 342.39 ns - 346.22 ns | Builds the exact special form for tan(pi/7). |
| `exact_transcendental_special_forms/asin_sin_6pi_7` | 809.90 ns | 806.75 ns - 813.59 ns | Recognizes the principal branch of asin(sin(6pi/7)). |
| `exact_transcendental_special_forms/acos_cos_9pi_7` | 1.349 us | 1.342 us - 1.357 us | Recognizes the principal branch of acos(cos(9pi/7)). |
| `exact_transcendental_special_forms/atan_tan_6pi_7` | 750.03 ns | 745.07 ns - 755.73 ns | Recognizes the principal branch of atan(tan(6pi/7)). |
| `exact_transcendental_special_forms/asinh_large` | 219.55 ns | 218.63 ns - 220.52 ns | Builds a large inverse hyperbolic sine without exact intermediate Reals. |
| `exact_transcendental_special_forms/atanh_sqrt_half` | 208.98 ns | 207.56 ns - 210.46 ns | Builds atanh(sqrt(2)/2) after exact structural domain checks. |
| `exact_transcendental_special_forms/atanh_sqrt_two_error` | 329.28 ns | 123.01 ns - 740.97 ns | Rejects atanh(sqrt(2)) through exact structural domain checks. |
| `exact_transcendental_special_forms/sinh_ln_two` | 554.91 ns | 551.77 ns - 558.58 ns | Folds sinh(ln(2)) to the exact rational 3/4 via the integer-log-collapse shortcut. |
| `exact_transcendental_special_forms/cosh_ln_two` | 561.81 ns | 559.32 ns - 564.83 ns | Folds cosh(ln(2)) to the exact rational 5/4 via the integer-log-collapse shortcut. |
| `exact_transcendental_special_forms/tanh_ln_two` | 559.64 ns | 556.93 ns - 562.77 ns | Folds tanh(ln(2)) to the exact rational 3/5 via the integer-log-collapse shortcut. |
| `exact_transcendental_special_forms/sinh_rational_one` | 919.52 ns | 917.30 ns - 922.17 ns | Builds sinh(1) through the generic (exp(x) - exp(-x))/2 identity path. |
| `exact_transcendental_special_forms/cosh_rational_one` | 859.49 ns | 853.34 ns - 866.74 ns | Builds cosh(1) through the generic (exp(x) + exp(-x))/2 identity path. |
| `exact_transcendental_special_forms/tanh_rational_one` | 1.709 us | 1.639 us - 1.825 us | Builds tanh(1) through the generic (exp(x) - exp(-x))/(exp(x) + exp(-x)) identity path. |
| `exact_transcendental_special_forms/atan2_origin` | 81.28 ns | 81.00 ns - 81.62 ns | Hits the origin (0, 0) short-circuit returning exact zero. |
| `exact_transcendental_special_forms/atan2_axis_positive_y` | 157.95 ns | 157.49 ns - 158.49 ns | Hits the positive-y axis short-circuit returning exact pi/2. |
| `exact_transcendental_special_forms/atan2_axis_negative_x` | 149.85 ns | 148.03 ns - 152.33 ns | Hits the negative-x axis short-circuit returning exact pi. |
| `exact_transcendental_special_forms/atan2_quadrant_one_unit_diagonal` | 275.99 ns | 273.95 ns - 278.67 ns | Quadrant I unit diagonal reduces to atan(1) = pi/4 exact special form. |
| `exact_transcendental_special_forms/atan2_quadrant_two_pi_correction` | 765.13 ns | 756.38 ns - 774.52 ns | Quadrant II (1, -2) exercises atan(small ratio) + pi correction. |
| `exact_transcendental_special_forms/atan2_quadrant_three_negative_pi` | 613.62 ns | 610.52 ns - 617.05 ns | Quadrant III (-1, -2) exercises atan(small ratio) - pi correction. |
| `exact_transcendental_special_forms/log2_power_of_two` | 161.89 ns | 160.64 ns - 163.23 ns | Folds log2(1024) to the exact rational 10 via the integer-log-detection shortcut. |
| `exact_transcendental_special_forms/log2_rational_three` | 226.65 ns | 225.93 ns - 227.45 ns | Builds log2(3) as a lightweight Log2 symbolic certificate. |
| `exact_transcendental_special_forms/log2_ln_quotient_fold` | 621.87 ns | 493.40 ns - 877.71 ns | Folds ln(5) / ln(2) into a Log2 certificate via the divide-recognize shortcut. |

### `symbolic_reductions`

Existing symbolic constant algebra cases considered for additional reductions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `symbolic_reductions/sqrt_pi_square` | 142.93 ns | 142.40 ns - 143.48 ns | Reduces sqrt(pi^2). |
| `symbolic_reductions/sqrt_pi_e_square` | 186.05 ns | 184.20 ns - 188.13 ns | Reduces sqrt((pi * e)^2). |
| `symbolic_reductions/ln_scaled_e` | 1.223 us | 1.211 us - 1.237 us | Reduces ln(2 * e). |
| `symbolic_reductions/sub_pi_three` | 259.33 ns | 257.22 ns - 261.90 ns | Builds the certified pi - 3 constant-offset form. |
| `symbolic_reductions/pi_minus_three_facts` | 42.10 ns | 41.79 ns - 42.46 ns | Reads structural facts for the cached pi - 3 offset form. |
| `symbolic_reductions/div_exp_exp` | 456.28 ns | 453.32 ns - 459.48 ns | Reduces e^3 / e. |
| `symbolic_reductions/div_pi_square_e` | 496.07 ns | 493.23 ns - 499.11 ns | Reduces pi^2 / e. |
| `symbolic_reductions/div_const_products` | 1.048 us | 906.31 ns - 1.327 us | Reduces (pi^3 * e^5) / (pi * e^2). |
| `symbolic_reductions/inverse_pi` | 107.21 ns | 106.58 ns - 107.90 ns | Builds the reciprocal of pi. |
| `symbolic_reductions/div_one_pi` | 166.32 ns | 164.79 ns - 168.04 ns | Reduces 1 / pi. |
| `symbolic_reductions/div_rational_exp` | 293.69 ns | 292.52 ns - 294.93 ns | Reduces 2 / e. |
| `symbolic_reductions/div_e_pi` | 250.37 ns | 249.01 ns - 251.85 ns | Reduces e / pi. |
| `symbolic_reductions/mul_pi_inverse_pi` | 251.53 ns | 249.34 ns - 253.99 ns | Multiplies pi by its reciprocal. |
| `symbolic_reductions/mul_pi_e_sqrt_two` | 462.83 ns | 461.20 ns - 464.58 ns | Builds the factored pi * e * sqrt(2) form. |
| `symbolic_reductions/mul_const_product_sqrt_sqrt` | 701.24 ns | 695.27 ns - 708.19 ns | Cancels sqrt(2) from (pi * e * sqrt(2)) * sqrt(2). |
| `symbolic_reductions/div_const_product_sqrt_e` | 767.60 ns | 762.51 ns - 773.59 ns | Reduces (pi * e * sqrt(2)) / e. |
| `symbolic_reductions/inverse_const_product_sqrt` | 484.34 ns | 481.51 ns - 487.50 ns | Builds a rationalized reciprocal of pi * e * sqrt(2). |
| `symbolic_reductions/inverse_sqrt_two` | 110.08 ns | 108.94 ns - 111.34 ns | Builds the rationalized reciprocal of unit-scaled sqrt(2). |
| `symbolic_reductions/div_sqrt_two_sqrt_three` | 857.74 ns | 851.41 ns - 865.30 ns | Rationalizes a quotient of two unit-scaled square roots. |

### `exact_product_sums`

Fixed product-sum reducers used by determinant and cofactor kernels.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `exact_product_sums/signed_product_sum_lcm_6x2` | 389.85 ns | 387.87 ns - 392.10 ns | Computes an exact rational six-term signed product sum with mixed denominators. |
| `exact_product_sums/signed_product_sum_common_scale_6x2` | 184.95 ns | 183.38 ns - 186.67 ns | Computes an exact rational six-term signed product sum through the carried common-scale reducer. |
| `exact_product_sums/signed_product_sum_sparse_single_6x2` | 173.98 ns | 172.73 ns - 175.31 ns | Computes a sparse exact rational six-term signed product sum with one active product. |
| `exact_product_sums/real_signed_product_sum_rational_det3` | 282.87 ns | 277.50 ns - 289.46 ns | Computes a 3x3 determinant-shaped signed product sum through the public `Real` builder. |
| `exact_product_sums/real_signed_product_sum_mixed_symbolic_det3` | 5.673 us | 5.648 us - 5.701 us | Computes the same determinant-shaped builder with symbolic factors and rational scales. |

<!-- END scalar_micro -->

<!-- BEGIN library_perf -->
## `library_perf`

Library-level Criterion benchmarks for public `Rational`, `Real`, and `Simple` behavior.

### `real_format`

Formatting costs for important irrational `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_format/pi_lower_exp_32` | 4.791 us | 4.765 us - 4.819 us | Formats pi with 32 digits in lower-exponential form. |
| `real_format/pi_display_alt_32` | 4.968 us | 4.938 us - 5.002 us | Formats pi with alternate decimal display at 32 digits. |
| `real_format/sqrt_two_display_alt_32` | 6.290 us | 6.261 us - 6.324 us | Formats sqrt(2) with alternate decimal display at 32 digits. |

### `real_constants`

Construction cost for shared mathematical constants.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_constants/pi` | 115.98 ns | 115.78 ns - 116.20 ns | Constructs the symbolic pi value. |
| `real_constants/e` | 160.20 ns | 159.82 ns - 160.66 ns | Constructs the symbolic Euler constant value. |

### `simple`

Parser and evaluator costs for the `Simple` expression language.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple/parse_nested` | 857.14 ns | 852.45 ns - 863.78 ns | Parses a nested expression with powers, trig, and constants. |
| `simple/eval_nested` | 6.983 us | 6.973 us - 6.994 us | Evaluates a parsed mixed symbolic/numeric expression. |
| `simple/eval_constants` | 3.634 us | 3.302 us - 4.292 us | Evaluates repeated built-in constants. |
| `simple/eval_exact` | 1.707 us | 1.693 us - 1.722 us | Evaluates a rational-only expression through exact shortcuts. |
| `simple/eval_nested_exact` | 3.906 us | 3.884 us - 3.933 us | Evaluates a nested rational-only expression through exact shortcuts. |

### `real_powi`

Integer exponentiation for exact and irrational `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_powi/exact_17` | 290.51 ns | 289.50 ns - 291.74 ns | Raises an exact rational-backed `Real` to the 17th power. |
| `real_powi/irrational_17` | 393.20 ns | 391.66 ns - 395.14 ns | Raises sqrt(3) to the 17th power with symbolic simplification. |

### `rational_powi`

Integer exponentiation for `Rational`.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `rational_powi/exact_17` | 185.66 ns | 185.00 ns - 186.31 ns | Raises a rational value to the 17th power. |

### `real_exact_trig`

Exact and symbolic trig construction for known pi multiples.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_trig/sin_pi_6` | 194.21 ns | 193.62 ns - 194.84 ns | Computes sin(pi/6) via exact shortcut. |
| `real_exact_trig/cos_pi_3` | 419.75 ns | 416.93 ns - 423.90 ns | Computes cos(pi/3) via exact shortcut. |
| `real_exact_trig/tan_pi_5` | 348.85 ns | 347.76 ns - 350.01 ns | Builds tan(pi/5), a nontrivial symbolic tangent. |

### `real_general_trig`

General trig construction for irrational arguments.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_general_trig/tan_sqrt_2` | 961.41 ns | 959.14 ns - 964.15 ns | Builds tan(sqrt(2)). |
| `real_general_trig/tan_pi_sqrt_2_over_5` | 1.529 us | 1.519 us - 1.538 us | Builds tangent of an irrational multiple of pi. |

### `real_exact_inverse_trig`

Exact inverse trig shortcuts and symbolic inverse trig recognition.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_inverse_trig/asin_1_2` | 140.83 ns | 139.99 ns - 141.88 ns | Recognizes asin(1/2) as pi/6. |
| `real_exact_inverse_trig/asin_minus_1_2` | 146.51 ns | 145.78 ns - 147.31 ns | Recognizes asin(-1/2) as -pi/6. |
| `real_exact_inverse_trig/asin_sqrt_2_over_2` | 306.01 ns | 303.47 ns - 309.24 ns | Recognizes asin(sqrt(2)/2) as pi/4. |
| `real_exact_inverse_trig/asin_sin_pi_5` | 464.99 ns | 463.55 ns - 466.24 ns | Inverts a symbolic sin(pi/5). |
| `real_exact_inverse_trig/acos_1` | 92.39 ns | 92.04 ns - 92.79 ns | Recognizes acos(1) as zero. |
| `real_exact_inverse_trig/acos_minus_1` | 124.43 ns | 123.51 ns - 125.58 ns | Recognizes acos(-1) as pi. |
| `real_exact_inverse_trig/acos_1_2` | 139.18 ns | 138.65 ns - 139.80 ns | Recognizes acos(1/2) as pi/3. |
| `real_exact_inverse_trig/atan_1` | 134.83 ns | 133.45 ns - 136.38 ns | Recognizes atan(1) as pi/4. |
| `real_exact_inverse_trig/atan_sqrt_3_over_3` | 463.36 ns | 461.51 ns - 465.33 ns | Recognizes atan(sqrt(3)/3) as pi/6. |
| `real_exact_inverse_trig/atan_tan_pi_5` | 463.36 ns | 461.37 ns - 465.65 ns | Inverts a symbolic tan(pi/5). |

### `real_general_inverse_trig`

General inverse trig construction, domain errors, and atan range reduction.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_general_inverse_trig/asin_7_10` | 636.43 ns | 633.41 ns - 640.01 ns | Builds asin(7/10) through the rational-specialized path. |
| `real_general_inverse_trig/asin_sqrt_2_over_3` | 432.35 ns | 431.54 ns - 433.22 ns | Builds asin(sqrt(2)/3) through the general path. |
| `real_general_inverse_trig/acos_7_10` | 640.63 ns | 559.70 ns - 800.14 ns | Builds acos(7/10) through the rational-specialized asin path. |
| `real_general_inverse_trig/acos_sqrt_2_over_3` | 347.20 ns | 345.70 ns - 349.10 ns | Builds acos(sqrt(2)/3) through the general path. |
| `real_general_inverse_trig/asin_11_10_error` | 212.16 ns | 210.85 ns - 213.68 ns | Rejects rational asin input outside [-1, 1]. |
| `real_general_inverse_trig/acos_11_10_error` | 214.33 ns | 212.69 ns - 216.18 ns | Rejects rational acos input outside [-1, 1]. |
| `real_general_inverse_trig/atan_8` | 170.90 ns | 170.26 ns - 171.60 ns | Builds atan(8), exercising large-argument reduction. |
| `real_general_inverse_trig/atan_sqrt_2` | 7.058 us | 6.321 us - 8.516 us | Builds atan(sqrt(2)). |

### `real_inverse_hyperbolic`

Inverse hyperbolic construction, exact exits, stable ln1p forms, and domain errors.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_inverse_hyperbolic/asinh_0` | 67.57 ns | 67.32 ns - 67.89 ns | Recognizes asinh(0) as zero. |
| `real_inverse_hyperbolic/asinh_1_2` | 280.13 ns | 223.02 ns - 393.32 ns | Builds asinh(1/2) through the stable moderate-input path. |
| `real_inverse_hyperbolic/asinh_sqrt_2` | 310.46 ns | 309.39 ns - 311.60 ns | Builds asinh(sqrt(2)) without cancellation-prone log construction. |
| `real_inverse_hyperbolic/asinh_minus_1_2` | 267.33 ns | 266.32 ns - 268.55 ns | Uses odd symmetry for negative asinh input. |
| `real_inverse_hyperbolic/asinh_1_000_000` | 286.51 ns | 217.63 ns - 423.08 ns | Builds asinh for a large positive rational. |
| `real_inverse_hyperbolic/acosh_1` | 72.90 ns | 72.76 ns - 73.06 ns | Recognizes acosh(1) as zero. |
| `real_inverse_hyperbolic/acosh_2` | 108.27 ns | 107.85 ns - 108.80 ns | Builds acosh(2) through the stable moderate-input path. |
| `real_inverse_hyperbolic/acosh_sqrt_2` | 233.07 ns | 232.43 ns - 233.62 ns | Builds acosh(sqrt(2)) through square-root domain specialization. |
| `real_inverse_hyperbolic/acosh_1_000_000` | 207.44 ns | 151.74 ns - 318.28 ns | Builds acosh for a large positive rational. |
| `real_inverse_hyperbolic/atanh_0` | 66.97 ns | 66.86 ns - 67.10 ns | Recognizes atanh(0) as zero. |
| `real_inverse_hyperbolic/atanh_1_2` | 160.45 ns | 159.94 ns - 161.02 ns | Builds exact-rational atanh(1/2). |
| `real_inverse_hyperbolic/atanh_minus_1_2` | 202.06 ns | 169.67 ns - 265.61 ns | Builds exact-rational atanh(-1/2). |
| `real_inverse_hyperbolic/atanh_sqrt_half` | 205.19 ns | 204.07 ns - 206.44 ns | Recognizes atanh(sqrt(2)/2) as asinh(1). |
| `real_inverse_hyperbolic/atanh_9_10` | 400.53 ns | 398.43 ns - 403.21 ns | Builds exact-rational atanh near the upper domain boundary. |
| `real_inverse_hyperbolic/atanh_1_error` | 38.44 ns | 38.14 ns - 38.77 ns | Rejects atanh(1) at the rational domain boundary. |

### `simple_inverse_functions`

Parsed/evaluated inverse trig and inverse hyperbolic expressions that should succeed.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple_inverse_functions/asin_1_2` | 185.13 ns | 184.35 ns - 185.92 ns | Evaluates `(asin 1/2)`. |
| `simple_inverse_functions/acos_1_2` | 186.08 ns | 185.02 ns - 187.22 ns | Evaluates `(acos 1/2)`. |
| `simple_inverse_functions/atan_1` | 181.33 ns | 180.32 ns - 182.55 ns | Evaluates `(atan 1)`. |
| `simple_inverse_functions/asin_general` | 690.55 ns | 688.69 ns - 692.66 ns | Evaluates `(asin 7/10)`. |
| `simple_inverse_functions/acos_general` | 612.37 ns | 609.18 ns - 615.92 ns | Evaluates `(acos 7/10)`. |
| `simple_inverse_functions/atan_general` | 216.51 ns | 215.77 ns - 217.36 ns | Evaluates `(atan 8)`. |
| `simple_inverse_functions/asinh_1_2` | 264.55 ns | 263.09 ns - 266.24 ns | Evaluates `(asinh 1/2)`. |
| `simple_inverse_functions/asinh_sqrt_2` | 1.057 us | 1.048 us - 1.067 us | Evaluates `(asinh (sqrt 2))`. |
| `simple_inverse_functions/acosh_2` | 148.17 ns | 147.03 ns - 149.60 ns | Evaluates `(acosh 2)`. |
| `simple_inverse_functions/acosh_sqrt_2` | 933.12 ns | 922.67 ns - 946.45 ns | Evaluates `(acosh (sqrt 2))`. |
| `simple_inverse_functions/atanh_1_2` | 204.12 ns | 202.90 ns - 205.47 ns | Evaluates `(atanh 1/2)`. |
| `simple_inverse_functions/atanh_minus_1_2` | 209.28 ns | 208.00 ns - 210.81 ns | Evaluates `(atanh -1/2)`. |

### `simple_inverse_error_functions`

Parsed/evaluated inverse function expressions that should fail quickly with `NotANumber`.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple_inverse_error_functions/asin_11_10` | 271.29 ns | 260.86 ns - 290.22 ns | Rejects `(asin 11/10)`. |
| `simple_inverse_error_functions/acos_sqrt_2` | 898.53 ns | 892.82 ns - 905.27 ns | Rejects `(acos (sqrt 2))`. |
| `simple_inverse_error_functions/acosh_0` | 56.85 ns | 54.63 ns - 60.88 ns | Rejects `(acosh 0)`. |
| `simple_inverse_error_functions/acosh_minus_2` | 85.61 ns | 79.46 ns - 96.74 ns | Rejects `(acosh -2)`. |
| `simple_inverse_error_functions/atanh_1` | 84.09 ns | 82.27 ns - 87.33 ns | Rejects `(atanh 1)`. |
| `simple_inverse_error_functions/atanh_sqrt_2` | 815.12 ns | 812.04 ns - 818.50 ns | Rejects `(atanh (sqrt 2))`. |

### `real_exact_ln`

Exact logarithm construction and simplification for rational inputs.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_ln/ln_1024` | 225.20 ns | 224.21 ns - 226.41 ns | Recognizes ln(1024) as 10 ln(2). |
| `real_exact_ln/ln_1_8` | 212.95 ns | 212.07 ns - 213.99 ns | Recognizes ln(1/8) as -3 ln(2). |
| `real_exact_ln/ln_1000` | 201.67 ns | 200.43 ns - 203.34 ns | Simplifies ln(1000) via small integer logarithm factors. |

### `real_exact_exp_log10`

Exact inverse relationships among exp, ln, and log10.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_exp_log10/exp_ln_1000` | 214.39 ns | 213.45 ns - 215.49 ns | Simplifies exp(ln(1000)) back to 1000. |
| `real_exact_exp_log10/exp_ln_1_8` | 236.43 ns | 235.92 ns - 236.95 ns | Simplifies exp(ln(1/8)) back to 1/8. |
| `real_exact_exp_log10/log10_1000` | 110.25 ns | 109.68 ns - 110.92 ns | Recognizes log10(1000) as 3. |
| `real_exact_exp_log10/log10_1_1000` | 130.73 ns | 130.20 ns - 131.38 ns | Recognizes log10(1/1000) as -3. |

### `real_stable_scalar_substrate`

Stable scalar constructors that preserve small residuals, dominance, roots, rational powers, and certified integer decisions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_stable_scalar_substrate/ln_1p_tiny` | 150.59 ns | 146.87 ns - 154.55 ns | Builds ln(1 + tiny) without first adding one generically. |
| `real_stable_scalar_substrate/ln_1m_tiny` | 157.36 ns | 155.22 ns - 159.77 ns | Builds ln(1 - tiny) through the log1p companion path. |
| `real_stable_scalar_substrate/expm1_tiny` | 172.33 ns | 169.85 ns - 175.05 ns | Builds exp(tiny) - 1 through the dedicated expm1 node. |
| `real_stable_scalar_substrate/softplus_large_positive` | 3.139 us | 3.062 us - 3.225 us | Builds softplus for a dominant positive input. |
| `real_stable_scalar_substrate/softplus_large_negative` | 2.917 us | 2.846 us - 3.000 us | Builds softplus for a dominant negative input. |
| `real_stable_scalar_substrate/logaddexp_dominant` | 4.820 us | 4.762 us - 4.894 us | Builds logaddexp when one side is certifiably dominant. |
| `real_stable_scalar_substrate/logsubexp_near` | 483.67 ns | 474.14 ns - 494.53 ns | Builds logsubexp for a certifiably positive but small log-space difference. |
| `real_stable_scalar_substrate/sigmoid_large_positive` | 2.997 us | 2.946 us - 3.052 us | Builds a large positive sigmoid through the stable tail path. |
| `real_stable_scalar_substrate/logit_near_one` | 1.192 us | 1.178 us - 1.208 us | Builds logit close to the upper probability boundary. |
| `real_stable_scalar_substrate/sqrt1pm1_tiny` | 3.464 us | 3.427 us - 3.504 us | Builds sqrt(1 + tiny) - 1 through the stable helper. |
| `real_stable_scalar_substrate/sqrt1m1_tiny` | 4.041 us | 4.015 us - 4.071 us | Builds sqrt(1 - tiny) - 1 through the stable helper. |
| `real_stable_scalar_substrate/cbrt_negative_perfect` | 198.70 ns | 197.77 ns - 199.69 ns | Collapses a negative perfect cube. |
| `real_stable_scalar_substrate/root_n_perfect_fourth` | 200.38 ns | 197.21 ns - 204.22 ns | Collapses an exact fourth root. |
| `real_stable_scalar_substrate/pow_rational_negative_odd_denominator` | 265.53 ns | 262.78 ns - 268.53 ns | Routes a negative rational base through odd-root symmetry. |
| `real_stable_scalar_substrate/floor_certified_rational` | 75.39 ns | 74.63 ns - 76.30 ns | Certifies rational floor structurally. |
| `real_stable_scalar_substrate/rem_euclid_certified_rational` | 660.05 ns | 656.12 ns - 664.50 ns | Computes rational Euclidean remainder through certified quotient floor. |

### `real_geometry_polynomial_substrate`

Geometry-facing scalar helpers for rational-turn trig, removable small-angle limits, vectors, product sums, and polynomial forms.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_geometry_polynomial_substrate/sin_pi_one_sixth` | 167.30 ns | 166.60 ns - 168.10 ns | Uses exact rational-turn sine. |
| `real_geometry_polynomial_substrate/cos_pi_one_fourth` | 537.24 ns | 535.14 ns - 539.45 ns | Uses exact rational-turn cosine. |
| `real_geometry_polynomial_substrate/tan_pi_one_third` | 513.62 ns | 509.41 ns - 518.96 ns | Uses exact rational-turn tangent. |
| `real_geometry_polynomial_substrate/sinc_zero` | 75.03 ns | 74.60 ns - 75.56 ns | Returns the removable sinc limit at zero. |
| `real_geometry_polynomial_substrate/sinc_tiny` | 330.79 ns | 328.68 ns - 333.14 ns | Builds sinc for a tiny exact input. |
| `real_geometry_polynomial_substrate/sinc_pi_half` | 410.11 ns | 407.65 ns - 413.03 ns | Builds normalized sinc for an exact half turn. |
| `real_geometry_polynomial_substrate/cosc_tiny` | 1.278 us | 1.272 us - 1.284 us | Builds the small-angle (1 - cos x) / x^2 helper. |
| `real_geometry_polynomial_substrate/atan2_axis` | 134.19 ns | 132.72 ns - 135.95 ns | Classifies an axis-aligned atan2 input exactly. |
| `real_geometry_polynomial_substrate/atan2_quadrant` | 372.27 ns | 370.91 ns - 374.06 ns | Builds a quadrant-correct atan2 expression. |
| `real_geometry_polynomial_substrate/hypot2_3_4` | 366.68 ns | 361.98 ns - 372.04 ns | Collapses a 3-4-5 norm through exact dot products. |
| `real_geometry_polynomial_substrate/hypot3_2_3_6` | 433.86 ns | 430.78 ns - 437.18 ns | Collapses a 2-3-6 norm through exact dot products. |
| `real_geometry_polynomial_substrate/hypot_minus_tiny` | 6.810 us | 6.755 us - 6.870 us | Uses rationalized hypot-minus for cancellation resistance. |
| `real_geometry_polynomial_substrate/mul_add_zero_product` | 265.66 ns | 203.20 ns - 389.63 ns | Skips a known-zero product lane. |
| `real_geometry_polynomial_substrate/sum_products_dense` | 4.301 us | 4.060 us - 4.766 us | Builds a dense product sum. |
| `real_geometry_polynomial_substrate/diff_of_products_near_cancel` | 1.448 us | 1.442 us - 1.455 us | Preserves determinant-like product difference structure. |
| `real_geometry_polynomial_substrate/eval_poly_horner` | 3.784 us | 3.758 us - 3.815 us | Evaluates a polynomial through Horner form. |
| `real_geometry_polynomial_substrate/eval_rational_poly` | 4.952 us | 4.926 us - 4.981 us | Evaluates numerator and denominator polynomial forms before division. |

### `real_normal_scientific_substrate`

Gaussian tail helpers and exact/finite scientific special-function forms added for higher numerical workloads.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_normal_scientific_substrate/erfc_zero` | 62.84 ns | 62.63 ns - 63.09 ns | Takes the exact erfc(0) exit. |
| `real_normal_scientific_substrate/erfcx_tail` | 2.945 us | 2.920 us - 2.972 us | Builds scaled erfc in a positive tail. |
| `real_normal_scientific_substrate/normal_sf_tail` | 326.03 ns | 325.07 ns - 327.08 ns | Builds standard-normal upper-tail probability. |
| `real_normal_scientific_substrate/pnorm_upper_tail` | 334.78 ns | 332.41 ns - 337.41 ns | Builds the upper-tail alias. |
| `real_normal_scientific_substrate/log_pnorm_tail` | 289.79 ns | 288.50 ns - 291.57 ns | Builds lower log-CDF tail form. |
| `real_normal_scientific_substrate/log_pnorm_zero` | 178.09 ns | 176.78 ns - 179.54 ns | Takes the exact log-CDF value at zero. |
| `real_normal_scientific_substrate/log_normal_sf_tail` | 303.87 ns | 302.23 ns - 305.69 ns | Builds upper log-survival tail form. |
| `real_normal_scientific_substrate/log_normal_sf_zero` | 182.34 ns | 179.47 ns - 185.45 ns | Takes the exact log-survival value at zero. |
| `real_normal_scientific_substrate/log_dnorm_large` | 136.33 ns | 134.63 ns - 138.23 ns | Builds analytic log-density at a large input. |
| `real_normal_scientific_substrate/normal_interval_narrow` | 1.689 us | 1.671 us - 1.710 us | Builds a narrow interval mass without spelling pnorm subtraction. |
| `real_normal_scientific_substrate/erfinv_mid` | 2.333 us | 2.309 us - 2.361 us | Builds inverse error function through qnorm transform. |
| `real_normal_scientific_substrate/erfcinv_tail` | 3.458 us | 3.431 us - 3.487 us | Builds inverse complementary error function through tail qnorm transform. |
| `real_normal_scientific_substrate/qnorm_upper_tail` | 1.568 us | 1.562 us - 1.574 us | Builds inverse survival quantile. |
| `real_normal_scientific_substrate/normal_pdf_parametric` | 1.881 us | 1.874 us - 1.889 us | Standardizes exactly before density construction. |
| `real_normal_scientific_substrate/normal_survival_parametric` | 835.58 ns | 831.53 ns - 840.18 ns | Standardizes exactly before upper-tail construction. |
| `real_normal_scientific_substrate/normal_mills_tail` | 3.701 us | 3.675 us - 3.739 us | Builds Mills ratio through erfcx identity. |
| `real_normal_scientific_substrate/normal_mills_zero` | 165.41 ns | 164.66 ns - 166.27 ns | Takes the exact Mills ratio value at zero. |
| `real_normal_scientific_substrate/normal_hazard_tail` | 4.984 us | 4.955 us - 5.015 us | Builds reciprocal Mills hazard. |
| `real_normal_scientific_substrate/normal_hazard_zero` | 166.33 ns | 165.57 ns - 167.18 ns | Takes the exact hazard value at zero. |
| `real_normal_scientific_substrate/normal_inverse_mills_zero` | 165.59 ns | 164.64 ns - 166.72 ns | Takes the exact lower inverse Mills value at zero. |
| `real_normal_scientific_substrate/hermite_8` | 2.693 us | 2.674 us - 2.713 us | Builds an exact probabilists' Hermite polynomial. |
| `real_normal_scientific_substrate/dnorm_derivative_4` | 2.338 us | 2.319 us - 2.360 us | Combines exact Hermite polynomial with normal density. |
| `real_normal_scientific_substrate/standard_normal_moment_12` | 197.82 ns | 196.06 ns - 199.75 ns | Uses double-factorial closed form. |
| `real_normal_scientific_substrate/normal_interval_moment_3` | 7.229 us | 6.155 us - 9.313 us | Uses interval mass and density-boundary recurrence. |
| `real_normal_scientific_substrate/truncated_normal_mean` | 2.574 us | 2.560 us - 2.590 us | Builds truncated-normal mean from stable interval mass. |
| `real_normal_scientific_substrate/gamma_integer` | 342.80 ns | 341.16 ns - 344.60 ns | Uses exact integer gamma closed form. |
| `real_normal_scientific_substrate/gamma_half_integer` | 554.04 ns | 549.75 ns - 558.87 ns | Uses exact half-integer gamma closed form. |
| `real_normal_scientific_substrate/lgamma_half_integer` | 1.927 us | 1.909 us - 1.951 us | Logs the absolute half-integer gamma value. |
| `real_normal_scientific_substrate/beta_integer` | 433.16 ns | 431.13 ns - 436.33 ns | Builds integer beta through an exact factorial ratio. |
| `real_normal_scientific_substrate/ln_beta_half_integer` | 3.815 us | 3.794 us - 3.840 us | Builds log beta through lgamma sum. |
| `real_normal_scientific_substrate/regularized_beta_mid` | 2.482 us | 2.473 us - 2.492 us | Uses finite positive-integer beta binomial tail. |
| `real_normal_scientific_substrate/regularized_beta_uniform` | 302.34 ns | 301.59 ns - 303.14 ns | Takes the exact I_x(1, 1) identity. |
| `real_normal_scientific_substrate/regularized_beta_left_unity` | 585.60 ns | 581.29 ns - 590.79 ns | Reduces I_x(1, b) to one complement power. |
| `real_normal_scientific_substrate/regularized_beta_q_mid` | 1.695 us | 1.685 us - 1.705 us | Uses finite positive-integer beta upper-tail form. |
| `real_normal_scientific_substrate/regularized_beta_q_uniform` | 277.85 ns | 276.81 ns - 279.12 ns | Takes the exact upper-tail I_x(1, 1) complement. |
| `real_normal_scientific_substrate/regularized_beta_q_left_unity` | 504.99 ns | 501.53 ns - 509.79 ns | Reduces the upper beta tail for a = 1 to one power. |
| `real_normal_scientific_substrate/regularized_gamma_p_half` | 4.027 us | 3.404 us - 5.256 us | Uses half-integer incomplete-gamma recurrence. |
| `real_normal_scientific_substrate/regularized_gamma_q_integer` | 2.437 us | 2.414 us - 2.464 us | Uses integer incomplete-gamma recurrence. |
| `real_normal_scientific_substrate/chi_square_sf` | 5.334 us | 5.309 us - 5.362 us | Wraps regularized upper gamma for chi-square upper tail. |

### `simple_new_function_surface`

Parser and evaluator coverage for the newly exposed stable scalar, geometry, normal, and scientific functions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple_new_function_surface/stable_log_exp_bundle` | 51.141 us | 50.848 us - 51.468 us | Evaluates log1p/log1m/expm1/softplus/logaddexp/logsubexp/sigmoid/logit together. |
| `simple_new_function_surface/geometry_bundle` | 78.889 us | 77.862 us - 80.161 us | Evaluates rational-turn trig, small-angle helpers, vector norms, product sums, and polynomials together. |
| `simple_new_function_surface/normal_bundle` | 4.597 ms | 4.582 ms - 4.615 ms | Evaluates normal tails, log tails, interval mass, inverse tails, and moments together. |
| `simple_new_function_surface/scientific_bundle` | 71.943 us | 71.326 us - 72.905 us | Evaluates gamma, beta, regularized gamma/beta, and chi-square forms together. |
| `simple_new_function_surface/error_bundle` | 568.48 ns | 550.08 ns - 586.31 ns | Exercises fast domain failures for new public functions. |

<!-- END library_perf -->

<!-- BEGIN adversarial_transcendentals -->
## `adversarial_transcendentals`

Adversarial transcendental benchmarks for `hyperreal` trig, inverse trig, and inverse hyperbolic construction and approximation paths.

### `trig_adversarial_approx`

Cold approximation of sine, cosine, and tangent at exact, tiny, huge, and near-singular arguments.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `trig_adversarial_approx/sin_tiny_rational_p96` | 557.30 ns | 551.81 ns - 564.42 ns | Approximates sin(1e-12), stressing direct tiny-argument setup. |
| `trig_adversarial_approx/cos_tiny_rational_p96` | 567.52 ns | 557.38 ns - 582.54 ns | Approximates cos(1e-12), stressing direct tiny-argument setup. |
| `trig_adversarial_approx/tan_tiny_rational_p96` | 1.035 us | 1.031 us - 1.040 us | Approximates tan(1e-12), stressing direct tiny-argument setup. |
| `trig_adversarial_approx/sin_medium_rational_p96` | 1.723 us | 1.720 us - 1.725 us | Approximates sin(7/5), a moderate non-pi rational. |
| `trig_adversarial_approx/cos_medium_rational_p96` | 1.579 us | 1.567 us - 1.593 us | Approximates cos(7/5), a moderate non-pi rational. |
| `trig_adversarial_approx/tan_medium_rational_p96` | 3.545 us | 3.301 us - 3.845 us | Approximates tan(7/5), a moderate non-pi rational. |
| `trig_adversarial_approx/sin_f64_exact_p96` | 1.940 us | 1.911 us - 1.995 us | Approximates sin(1.23456789 imported as an exact dyadic rational). |
| `trig_adversarial_approx/cos_f64_exact_p96` | 1.880 us | 1.860 us - 1.905 us | Approximates cos(1.23456789 imported as an exact dyadic rational). |
| `trig_adversarial_approx/sin_1e6_p96` | 2.500 us | 2.440 us - 2.568 us | Approximates sin(1000000), stressing integer argument reduction. |
| `trig_adversarial_approx/cos_1e6_p96` | 2.427 us | 2.399 us - 2.477 us | Approximates cos(1000000), stressing integer argument reduction. |
| `trig_adversarial_approx/tan_1e6_p96` | 4.870 us | 4.596 us - 5.312 us | Approximates tan(1000000), stressing integer argument reduction. |
| `trig_adversarial_approx/sin_1e30_p96` | 2.222 us | 2.187 us - 2.260 us | Approximates sin(10^30), stressing very large integer reduction. |
| `trig_adversarial_approx/cos_1e30_p96` | 2.287 us | 2.281 us - 2.294 us | Approximates cos(10^30), stressing very large integer reduction. |
| `trig_adversarial_approx/tan_1e30_p96` | 4.047 us | 4.015 us - 4.086 us | Approximates tan(10^30), stressing very large integer reduction. |
| `trig_adversarial_approx/sin_huge_pi_plus_offset_p96` | 2.271 us | 2.258 us - 2.286 us | Approximates sin(2^512*pi + 7/5), stressing exact pi-multiple cancellation. |
| `trig_adversarial_approx/cos_huge_pi_plus_offset_p96` | 2.161 us | 2.140 us - 2.200 us | Approximates cos(2^512*pi + 7/5), stressing exact pi-multiple cancellation. |
| `trig_adversarial_approx/tan_huge_pi_plus_offset_p96` | 3.631 us | 3.609 us - 3.659 us | Approximates tan(2^512*pi + 7/5), stressing exact pi-multiple cancellation. |
| `trig_adversarial_approx/tan_near_half_pi_p96` | 5.866 us | 5.784 us - 5.950 us | Approximates tan(pi/2 - 2^-40), stressing the cotangent complement path. |
| `trig_adversarial_approx/tan_promoted_generated_604_125_p96` | 3.833 us | 3.786 us - 3.882 us | Promoted slow-performer tan(604/125), a generated top offender from the library-wide fuzz history. |

### `inverse_trig_adversarial_approx`

Cold approximation of asin, acos, and atan near exact values, zero, endpoints, and large atan inputs.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `inverse_trig_adversarial_approx/asin_zero_p96` | 82.66 ns | 79.44 ns - 86.93 ns | Approximates asin(0), which should collapse before the generic inverse-trig path. |
| `inverse_trig_adversarial_approx/acos_zero_p96` | 215.04 ns | 210.25 ns - 220.24 ns | Approximates acos(0), which should reduce to pi/2. |
| `inverse_trig_adversarial_approx/atan_zero_p96` | 76.31 ns | 74.99 ns - 78.15 ns | Approximates atan(0), which should collapse to zero. |
| `inverse_trig_adversarial_approx/asin_tiny_positive_p96` | 435.64 ns | 432.75 ns - 439.28 ns | Approximates asin(1e-12), stressing the tiny odd series. |
| `inverse_trig_adversarial_approx/acos_tiny_positive_p96` | 871.01 ns | 857.45 ns - 890.43 ns | Approximates acos(1e-12), stressing pi/2 minus the tiny asin path. |
| `inverse_trig_adversarial_approx/atan_tiny_positive_p96` | 343.09 ns | 333.84 ns - 353.78 ns | Approximates atan(1e-12), stressing direct tiny atan setup. |
| `inverse_trig_adversarial_approx/asin_mid_positive_p96` | 6.634 us | 6.582 us - 6.705 us | Approximates asin(7/10), a generic in-domain value. |
| `inverse_trig_adversarial_approx/acos_mid_positive_p96` | 6.083 us | 6.002 us - 6.177 us | Approximates acos(7/10), a generic in-domain value. |
| `inverse_trig_adversarial_approx/atan_mid_positive_p96` | 3.334 us | 3.323 us - 3.344 us | Approximates atan(7/10), a generic in-domain value. |
| `inverse_trig_adversarial_approx/asin_near_one_p96` | 2.535 us | 2.531 us - 2.538 us | Approximates asin(0.999999), stressing endpoint transforms. |
| `inverse_trig_adversarial_approx/acos_near_one_p96` | 1.895 us | 1.881 us - 1.910 us | Approximates acos(0.999999), stressing endpoint transforms. |
| `inverse_trig_adversarial_approx/asin_near_minus_one_p96` | 2.693 us | 2.656 us - 2.751 us | Approximates asin(-0.999999), stressing odd symmetry near the endpoint. |
| `inverse_trig_adversarial_approx/acos_near_minus_one_p96` | 2.190 us | 2.081 us - 2.331 us | Approximates acos(-0.999999), stressing negative endpoint transforms. |
| `inverse_trig_adversarial_approx/atan_large_p96` | 1.784 us | 1.707 us - 1.875 us | Approximates atan(8), stressing reciprocal reduction. |
| `inverse_trig_adversarial_approx/atan_promoted_generated_783_412_p96` | 2.250 us | 2.191 us - 2.307 us | Promoted slow-performer atan(783/412), the generated exact-rational atan top offender. |
| `inverse_trig_adversarial_approx/ln_square_plus_one_promoted_generated_677_222_p96` | 2.444 us | 2.422 us - 2.467 us | Promoted slow-performer ln((677/222)^2 + 1), the generated exact-rational log top offender. |
| `inverse_trig_adversarial_approx/atan_huge_p96` | 681.41 ns | 665.50 ns - 702.84 ns | Approximates atan(10^30), stressing very large reciprocal reduction. |

### `trig_fuzz_adversarial_approx`

Deterministic broad sweeps of sine, cosine, and tangent over tiny, ordinary, huge, pi-offset, and near-pole exact inputs.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `trig_fuzz_adversarial_approx/sin_sweep_768_p96` | 1.695 ms | 1.688 ms - 1.702 ms | Approximates sin over 768 deterministic exact inputs spanning tiny, ordinary, huge, dyadic, rational, and pi-offset cases. |
| `trig_fuzz_adversarial_approx/cos_sweep_768_p96` | 1.792 ms | 1.742 ms - 1.843 ms | Approximates cos over the same 768-input deterministic fuzz sweep. |
| `trig_fuzz_adversarial_approx/tan_sweep_768_p96` | 3.019 ms | 3.013 ms - 3.024 ms | Approximates tan over the same deterministic sweep, including near-half-pi stress cases. |
| `trig_fuzz_adversarial_approx/sin_promoted_slow_candidates_p96` | 17.478 us | 17.268 us - 17.722 us | Approximates sin over promoted slow candidates found by prior sweep-style runs. |
| `trig_fuzz_adversarial_approx/cos_promoted_slow_candidates_p96` | 18.047 us | 17.920 us - 18.183 us | Approximates cos over promoted slow candidates found by prior sweep-style runs. |
| `trig_fuzz_adversarial_approx/tan_promoted_slow_candidates_p96` | 31.919 us | 31.624 us - 32.252 us | Approximates tan over promoted near-pole and large-reduction slow candidates. |

### `promoted_library_slow_offenders_approx`

Fifty structurally varied worst offenders promoted from the library-wide slow-performer history.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `promoted_library_slow_offenders_approx/promoted_50_structural_slow_offenders_p96` | not run | not run | Approximates 50 individual promoted slow cases spanning ln(1+x^2), atan, tan, sin, and cos over varied exact-rational structures. |

### `inverse_hyperbolic_adversarial_approx`

Cold approximation of inverse hyperbolic functions at tiny, moderate, large, and endpoint-adjacent arguments.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `inverse_hyperbolic_adversarial_approx/asinh_tiny_positive_p128` | 602.08 ns | 592.53 ns - 614.74 ns | Approximates asinh(1e-12), stressing cancellation avoidance near zero. |
| `inverse_hyperbolic_adversarial_approx/asinh_mid_positive_p128` | 7.169 us | 7.123 us - 7.217 us | Approximates asinh(1/2), a moderate positive value. |
| `inverse_hyperbolic_adversarial_approx/asinh_large_positive_p128` | 7.065 us | 7.007 us - 7.141 us | Approximates asinh(10^6), stressing large-input logarithmic behavior. |
| `inverse_hyperbolic_adversarial_approx/asinh_large_negative_p128` | 7.277 us | 7.221 us - 7.333 us | Approximates asinh(-10^6), stressing odd symmetry for large inputs. |
| `inverse_hyperbolic_adversarial_approx/acosh_one_plus_tiny_p128` | 6.955 us | 6.829 us - 7.146 us | Approximates acosh(1 + 1e-12), stressing the near-one endpoint. |
| `inverse_hyperbolic_adversarial_approx/acosh_sqrt_two_p128` | 150.13 ns | 144.43 ns - 157.03 ns | Approximates acosh(sqrt(2)), a symbolic square-root input. |
| `inverse_hyperbolic_adversarial_approx/acosh_two_p128` | 112.23 ns | 108.63 ns - 116.38 ns | Approximates acosh(2), a moderate exact rational. |
| `inverse_hyperbolic_adversarial_approx/acosh_large_positive_p128` | 7.277 us | 7.246 us - 7.310 us | Approximates acosh(10^6), stressing large-input logarithmic behavior. |
| `inverse_hyperbolic_adversarial_approx/atanh_tiny_positive_p128` | 550.83 ns | 544.26 ns - 557.29 ns | Approximates atanh(1e-12), stressing the tiny odd series. |
| `inverse_hyperbolic_adversarial_approx/atanh_mid_positive_p128` | 273.30 ns | 271.89 ns - 275.07 ns | Approximates atanh(1/2), a moderate exact rational. |
| `inverse_hyperbolic_adversarial_approx/atanh_near_one_p128` | 3.551 us | 3.525 us - 3.577 us | Approximates atanh(0.999999), stressing endpoint logarithmic behavior. |
| `inverse_hyperbolic_adversarial_approx/atanh_near_minus_one_p128` | 3.726 us | 3.663 us - 3.803 us | Approximates atanh(-0.999999), stressing odd symmetry near the endpoint. |

### `real_shortcut_adversarial`

Public `Real` construction shortcuts and domain checks for the same transcendental families.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_shortcut_adversarial/sin_exact_pi_over_six` | 196.56 ns | 190.59 ns - 203.06 ns | Constructs sin(pi/6), which should return the exact rational 1/2. |
| `real_shortcut_adversarial/cos_exact_pi_over_three` | 396.46 ns | 392.91 ns - 400.70 ns | Constructs cos(pi/3), which should return the exact rational 1/2. |
| `real_shortcut_adversarial/tan_exact_pi_over_four` | 204.07 ns | 187.86 ns - 225.48 ns | Constructs tan(pi/4), which should return the exact rational 1. |
| `real_shortcut_adversarial/asin_exact_half` | 116.60 ns | 114.65 ns - 118.64 ns | Constructs asin(1/2), which should return pi/6. |
| `real_shortcut_adversarial/acos_exact_half` | 119.45 ns | 115.97 ns - 123.60 ns | Constructs acos(1/2), which should return pi/3. |
| `real_shortcut_adversarial/atan_exact_one` | 114.19 ns | 110.76 ns - 117.62 ns | Constructs atan(1), which should return pi/4. |
| `real_shortcut_adversarial/asin_domain_error` | 456.61 ns | 453.02 ns - 460.52 ns | Rejects asin(1 + 1e-12). |
| `real_shortcut_adversarial/acos_domain_error` | 454.38 ns | 450.90 ns - 458.08 ns | Rejects acos(1 + 1e-12). |
| `real_shortcut_adversarial/atanh_endpoint_infinity` | 36.71 ns | 36.09 ns - 37.35 ns | Rejects atanh(1) as an infinite endpoint. |
| `real_shortcut_adversarial/atanh_domain_error` | 54.95 ns | 52.97 ns - 57.22 ns | Rejects atanh(1 + 1e-12). |
| `real_shortcut_adversarial/acosh_domain_error` | 53.92 ns | 51.43 ns - 56.86 ns | Rejects acosh(1 - 1e-12). |

<!-- END adversarial_transcendentals -->

<!-- BEGIN borrowed_ops -->
## `borrowed_ops`

Compares owned arithmetic with borrowed arithmetic for exact and irrational values.

### `rational_ops`

Owned versus borrowed arithmetic for exact `Rational` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `rational_ops/add_owned` | 418.59 ns | 415.33 ns - 422.10 ns | Adds cloned owned operands. |
| `rational_ops/add_refs` | 382.54 ns | 380.37 ns - 384.97 ns | Adds borrowed operands without cloning both inputs. |
| `rational_ops/sub_owned` | 410.97 ns | 408.68 ns - 413.66 ns | Subtracts cloned owned operands. |
| `rational_ops/sub_refs` | 395.34 ns | 394.43 ns - 396.35 ns | Subtracts borrowed operands. |
| `rational_ops/mul_owned` | 150.86 ns | 149.76 ns - 152.07 ns | Multiplies cloned owned operands. |
| `rational_ops/mul_refs` | 126.04 ns | 125.27 ns - 126.93 ns | Multiplies borrowed operands. |
| `rational_ops/div_owned` | 91.69 ns | 63.68 ns - 147.17 ns | Divides cloned owned operands. |
| `rational_ops/div_refs` | 35.29 ns | 35.11 ns - 35.50 ns | Divides borrowed operands. |

### `real_ops`

Owned versus borrowed arithmetic for exact rational-backed `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_ops/add_owned` | 464.14 ns | 461.60 ns - 467.03 ns | Adds cloned owned operands. |
| `real_ops/add_refs` | 466.80 ns | 462.87 ns - 471.01 ns | Adds borrowed operands without cloning both inputs. |
| `real_ops/sub_owned` | 860.56 ns | 851.56 ns - 871.94 ns | Subtracts cloned owned operands. |
| `real_ops/sub_refs` | 847.54 ns | 841.05 ns - 854.36 ns | Subtracts borrowed operands. |
| `real_ops/mul_owned` | 246.99 ns | 245.61 ns - 248.53 ns | Multiplies cloned owned operands. |
| `real_ops/mul_refs` | 195.87 ns | 194.12 ns - 197.97 ns | Multiplies borrowed operands. |
| `real_ops/div_owned` | 145.53 ns | 120.96 ns - 193.21 ns | Divides cloned owned operands. |
| `real_ops/div_refs` | 114.74 ns | 114.04 ns - 115.54 ns | Divides borrowed operands. |

### `real_irrational_ops`

Owned versus borrowed arithmetic for symbolic irrational `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_irrational_ops/add_owned` | 436.31 ns | 432.50 ns - 440.24 ns | Adds cloned owned operands. |
| `real_irrational_ops/add_refs` | 350.79 ns | 349.84 ns - 351.82 ns | Adds borrowed operands without cloning both inputs. |
| `real_irrational_ops/sub_owned` | 456.66 ns | 453.69 ns - 459.75 ns | Subtracts cloned owned operands. |
| `real_irrational_ops/sub_refs` | 378.26 ns | 374.52 ns - 382.80 ns | Subtracts borrowed operands. |
| `real_irrational_ops/mul_owned` | 998.03 ns | 992.42 ns - 1.004 us | Multiplies cloned owned operands. |
| `real_irrational_ops/mul_refs` | 823.67 ns | 818.05 ns - 830.37 ns | Multiplies borrowed operands. |
| `real_irrational_ops/div_owned` | 883.75 ns | 870.41 ns - 900.03 ns | Divides cloned owned operands. |
| `real_irrational_ops/div_refs` | 713.36 ns | 708.32 ns - 719.21 ns | Divides borrowed operands. |

<!-- END borrowed_ops -->

<!-- BEGIN float_convert -->
## `float_convert`

Covers exact import of floating-point values, including public `Real` conversion overhead.

### `float_convert`

Exact conversion from IEEE-754 floats into `Rational` and `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `float_convert/f32_normal` | not run | not run | Converts a normal `f32` into an exact `Rational`. |
| `float_convert/f64_normal` | not run | not run | Converts a normal `f64` into an exact `Rational`. |
| `float_convert/f64_binary_fraction` | not run | not run | Converts an exactly representable binary `f64` fraction into `Rational`. |
| `float_convert/f64_subnormal` | not run | not run | Converts a subnormal `f64` into an exact `Rational`. |
| `float_convert/real_f32_normal` | not run | not run | Converts a normal `f32` through the public `Real::try_from` path. |
| `float_convert/real_f64_normal` | not run | not run | Converts a normal `f64` through the public `Real::try_from` path. |
| `float_convert/real_f64_binary_fraction` | not run | not run | Converts an exactly representable binary `f64` fraction through the public `Real::try_from` path. |
| `float_convert/real_f64_subnormal` | not run | not run | Converts a subnormal `f64` through the public `Real::try_from` path. |

<!-- END float_convert -->
