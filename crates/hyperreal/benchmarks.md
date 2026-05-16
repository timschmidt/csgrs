<!-- BEGIN promoted_slow_offender_score -->
## `promoted_slow_offender_score`

Deterministic lexicase score for the current 100 promoted slow offenders. The score is the average current best-of-five wall-clock probe across the promoted set; lower is better. Delta compares with the previous score recorded in this file, and derivative is the change in delta.

<!-- promoted_slow_score_nanos: 4173 -->
<!-- promoted_slow_previous_score_nanos: 4173 -->
<!-- promoted_slow_score_delta_nanos: 0 -->

| Metric | Value |
| --- | ---: |
| Cases scored | 100 |
| Average score | 4.173 us |
| Delta | 0 ns |
| Delta derivative | 0 ns |

| Rank | Current Time | Operation | Input |
| ---: | ---: | --- | --- |
| 1 | 5.909 us | `generated_ln_abs_plus_one_p96` | `generated[9457] -3 23/90` |
| 2 | 5.839 us | `generated_ln_abs_plus_one_p96` | `generated[15472] -3 13/50` |
| 3 | 5.750 us | `generated_ln_abs_plus_one_p96` | `generated[9862] -1 221/492` |
| 4 | 5.719 us | `generated_ln_abs_plus_one_p96` | `generated[18352] -1 133/500` |
| 5 | 5.690 us | `generated_ln_abs_plus_one_p96` | `generated[14947] 3 11/222` |
| 6 | 5.669 us | `generated_ln_abs_plus_one_p96` | `generated[8152] 3 11/62` |
| 7 | 5.650 us | `generated_ln_abs_plus_one_p96` | `generated[6592] 1 109/348` |
| 8 | 5.630 us | `generated_ln_abs_plus_one_p96` | `generated[6877] -9 34/77` |
| 9 | 5.629 us | `generated_ln_abs_plus_one_p96` | `generated[1297] -1 83/188` |
| 10 | 5.609 us | `generated_ln_abs_plus_one_p96` | `generated[15082] 1 181/356` |

<!-- END promoted_slow_offender_score -->











































<!-- BEGIN numerical_micro -->
## `numerical_micro`

Low-level `Computable` microbenchmarks for approximation kernels, caches, structural facts, comparisons, and deep evaluator trees.

### `computable_cache`

Cold versus cached approximation of basic `Computable` expressions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_cache/ratio_approx_cold_p128` | 106.39 ns | 106.11 ns - 106.69 ns | Approximates a rational value at p=-128 from a fresh clone. |
| `computable_cache/ratio_approx_cached_p128` | 21.79 ns | 21.62 ns - 22.04 ns | Repeats an already cached rational approximation at p=-128. |
| `computable_cache/pi_approx_cold_p128` | 41.67 ns | 41.32 ns - 42.10 ns | Approximates pi at p=-128 from a fresh clone. |
| `computable_cache/pi_approx_cached_p128` | 22.32 ns | 22.06 ns - 22.67 ns | Repeats an already cached pi approximation at p=-128. |
| `computable_cache/pi_plus_tiny_cold_p128` | 204.26 ns | 200.89 ns - 209.29 ns | Approximates pi plus a tiny exact rational perturbation. |
| `computable_cache/pi_minus_tiny_cold_p128` | 201.46 ns | 198.38 ns - 206.66 ns | Approximates pi minus a tiny exact rational perturbation. |

### `computable_bounds`

Structural sign and bound discovery for deep or perturbed computable trees.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_bounds/deep_scaled_product_sign` | 70.82 ns | 70.12 ns - 71.39 ns | Finds the sign of a deep scaled product. |
| `computable_bounds/scaled_square_sign` | 160.06 ns | 158.47 ns - 161.72 ns | Finds the sign of repeated squaring with exact scale factors. |
| `computable_bounds/sqrt_scaled_square_sign` | 157.17 ns | 154.24 ns - 160.11 ns | Finds the sign after taking a square root of a scaled square. |
| `computable_bounds/deep_structural_bound_sign` | 18.41 ns | 18.27 ns - 18.57 ns | Finds sign through repeated multiply/inverse/negate structural transformations. |
| `computable_bounds/deep_structural_bound_sign_cached` | 3.84 ns | 3.81 ns - 3.87 ns | Reads the cached sign of the deep structural-bound chain. |
| `computable_bounds/deep_structural_bound_facts_cached` | 14.13 ns | 14.09 ns - 14.18 ns | Reads cached structural facts for the deep structural-bound chain. |
| `computable_bounds/perturbed_scaled_product_sign` | 150.87 ns | 148.31 ns - 153.04 ns | Finds sign for a deeply scaled value with a tiny perturbation. |
| `computable_bounds/perturbed_scaled_product_sign_until` | 150.54 ns | 148.04 ns - 152.72 ns | Refines sign for the perturbed scaled product only to p=-128. |
| `computable_bounds/pi_minus_tiny_sign` | 73.40 ns | 72.18 ns - 74.74 ns | Finds sign for pi minus a tiny exact rational. |
| `computable_bounds/pi_minus_tiny_sign_cached` | 3.82 ns | 3.80 ns - 3.83 ns | Reads cached sign for pi minus a tiny exact rational. |
| `computable_bounds/exp_unknown_sign_arg_sign` | 75.42 ns | 74.79 ns - 76.14 ns | Finds sign for exp(1 - pi), where exp can prove positivity structurally. |
| `computable_bounds/exp_unknown_sign_arg_sign_cached` | 3.82 ns | 3.81 ns - 3.83 ns | Reads cached sign for exp(1 - pi). |

### `computable_compare`

Ordering and absolute-comparison shortcuts.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_compare/compare_to_opposite_sign` | 12.16 ns | 12.11 ns - 12.22 ns | Compares values with known opposite signs. |
| `computable_compare/compare_to_exact_msd_gap` | 18.60 ns | 18.49 ns - 18.71 ns | Compares values with a large exact magnitude gap. |
| `computable_compare/compare_absolute_exact_rational` | 4.04 ns | 4.03 ns - 4.07 ns | Compares absolute values of exact rationals. |
| `computable_compare/compare_absolute_exact_rational_same_numerator` | 4.07 ns | 4.05 ns - 4.09 ns | Compares exact rational magnitudes with matching numerators. |
| `computable_compare/compare_absolute_dominant_add` | 14.13 ns | 14.09 ns - 14.19 ns | Compares a dominant term against the same term plus a tiny addend. |
| `computable_compare/compare_absolute_exact_msd_gap` | 18.91 ns | 18.82 ns - 19.02 ns | Compares absolute values with a large exact magnitude gap. |

### `computable_transcendentals`

Low-level approximation kernels and deep expression-tree stress cases.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_transcendentals/legacy_exp_one_p128` | 2.893 us | 2.873 us - 2.918 us | Runs the legacy direct exp series for input 1 at p=-128. |
| `computable_transcendentals/e_constant_cold_p128` | 41.40 ns | 41.04 ns - 41.84 ns | Approximates the shared e constant from a fresh clone. |
| `computable_transcendentals/e_constant_cached_p128` | 31.63 ns | 29.14 ns - 34.38 ns | Repeats a cached approximation of e. |
| `computable_transcendentals/legacy_exp_half_p128` | 2.505 us | 2.484 us - 2.529 us | Runs the legacy direct exp series for input 1/2 at p=-128. |
| `computable_transcendentals/exp_cold_p128` | 3.754 us | 3.748 us - 3.761 us | Approximates exp(7/5) from a fresh clone. |
| `computable_transcendentals/exp_cached_p128` | 21.87 ns | 21.82 ns - 21.93 ns | Repeats a cached exp(7/5) approximation. |
| `computable_transcendentals/exp_large_cold_p128` | 7.266 us | 7.241 us - 7.294 us | Approximates exp(128), exercising large-argument reduction. |
| `computable_transcendentals/exp_half_cold_p128` | 2.803 us | 2.798 us - 2.809 us | Approximates exp(1/2). |
| `computable_transcendentals/exp_near_limit_cold_p128` | 2.797 us | 2.792 us - 2.802 us | Approximates exp near a prescaling threshold. |
| `computable_transcendentals/exp_near_limit_cached_p128` | 21.81 ns | 21.73 ns - 21.91 ns | Repeats a cached near-threshold exp approximation. |
| `computable_transcendentals/exp_zero_cold_p128` | 72.59 ns | 72.29 ns - 72.94 ns | Approximates exp(0). |
| `computable_transcendentals/ln_cold_p128` | 4.198 us | 4.192 us - 4.205 us | Approximates ln(11/7). |
| `computable_transcendentals/ln_cached_p128` | 21.81 ns | 21.75 ns - 21.88 ns | Repeats a cached ln(11/7) approximation. |
| `computable_transcendentals/ln_smooth_rational_cold_p128` | 732.27 ns | 725.52 ns - 738.51 ns | Approximates ln(45/14), which can decompose into shared prime-log constants. |
| `computable_transcendentals/ln_nonsmooth_rational_cold_p128` | 2.540 us | 2.467 us - 2.632 us | Approximates ln(11/13), guarding the generic exact-rational log fallback. |
| `computable_transcendentals/ln_large_cold_p128` | 245.62 ns | 241.02 ns - 252.07 ns | Approximates ln(1024), exercising large-input reduction. |
| `computable_transcendentals/ln_large_cached_p128` | 21.80 ns | 21.76 ns - 21.83 ns | Repeats a cached ln(1024) approximation. |
| `computable_transcendentals/ln_tiny_cold_p128` | 190.35 ns | 189.59 ns - 191.17 ns | Approximates ln(2^-1024), exercising tiny-input reduction. |
| `computable_transcendentals/ln_near_limit_cold_p128` | 3.420 us | 3.334 us - 3.520 us | Approximates ln near the prescaled-ln limit. |
| `computable_transcendentals/ln_near_limit_cached_p128` | 21.84 ns | 21.79 ns - 21.89 ns | Repeats a cached near-limit ln approximation. |
| `computable_transcendentals/ln_one_cold_p128` | 35.07 ns | 34.88 ns - 35.27 ns | Approximates ln(1). |
| `computable_transcendentals/sqrt_cold_p128` | 746.58 ns | 731.98 ns - 772.57 ns | Approximates sqrt(2). |
| `computable_transcendentals/sqrt_squarefree_scaled_cold_p128` | 128.97 ns | 97.97 ns - 190.45 ns | Approximates sqrt(12), which can reduce to 2*sqrt(3). |
| `computable_transcendentals/sqrt_cached_p128` | 21.68 ns | 21.66 ns - 21.69 ns | Repeats a cached sqrt(2) approximation. |
| `computable_transcendentals/sqrt_single_scaled_square_cold_p128` | 1.094 us | 1.087 us - 1.103 us | Builds and approximates sqrt((7*pi/8)^2). |
| `computable_transcendentals/sin_cold_p96` | 1.545 us | 1.538 us - 1.553 us | Approximates sin(7/5). |
| `computable_transcendentals/sin_cached_p96` | 21.88 ns | 21.82 ns - 21.96 ns | Repeats a cached sin(7/5) approximation. |
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
| `computable_transcendentals/atan_cold_p96` | 6.908 us | 6.864 us - 6.985 us | Approximates atan(7/10). |
| `computable_transcendentals/atan_cached_p96` | not run | not run | Repeats a cached atan(7/10) approximation. |
| `computable_transcendentals/atan_large_cold_p96` | 1.617 us | 1.611 us - 1.624 us | Approximates atan(8), exercising argument reduction. |
| `computable_transcendentals/asin_zero_cold_p96` | 34.60 ns | 34.33 ns - 34.89 ns | Approximates asin(0) expression. |
| `computable_transcendentals/atan_zero_cold_p96` | 37.55 ns | 37.21 ns - 37.91 ns | Approximates atan(0). |
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
| `computable_transcendentals/deep_multiply_chain_cold_p128` | 70.34 ns | 70.17 ns - 70.51 ns | Approximates a 5000-node multiply-by-one chain. |
| `computable_transcendentals/deep_multiply_identity_chain_cold_p128` | 41.78 ns | 41.49 ns - 42.16 ns | Approximates a deep identity multiplication chain around pi. |
| `computable_transcendentals/deep_scaled_product_chain_cold_p128` | 668.24 ns | 544.69 ns - 913.31 ns | Approximates a deep product of exact scale factors. |
| `computable_transcendentals/perturbed_scaled_product_chain_cold_p128` | not run | not run | Approximates a deep scaled product with a tiny perturbation. |
| `computable_transcendentals/scaled_square_chain_cold_p128` | not run | not run | Approximates repeated squaring of a scaled irrational. |
| `computable_transcendentals/asymmetric_product_bad_order_cold_p128` | not run | not run | Approximates an asymmetric product order stress case. |
| `computable_transcendentals/sqrt_scaled_square_chain_cold_p128` | not run | not run | Approximates sqrt of a scaled-square chain. |
| `computable_transcendentals/warmed_zero_product_cold_p128` | not run | not run | Approximates a product involving a warmed zero sum. |
| `computable_transcendentals/inverse_scaled_product_chain_cold_p128` | 701.64 ns | 626.73 ns - 833.05 ns | Approximates the inverse of a deep scaled product. |
| `computable_transcendentals/deep_inverse_pair_chain_cold_p128` | 115.44 ns | 111.89 ns - 119.09 ns | Approximates a chain of inverse(inverse(x)) pairs. |
| `computable_transcendentals/deep_negated_square_chain_cold_p128` | not run | not run | Approximates repeated negate-square-sqrt transformations. |
| `computable_transcendentals/deep_negative_one_product_chain_cold_p128` | 88.57 ns | 88.36 ns - 88.82 ns | Approximates repeated multiplication by -1. |
| `computable_transcendentals/deep_half_product_chain_cold_p128` | 132.39 ns | 131.34 ns - 133.52 ns | Approximates repeated multiplication by 1/2. |
| `computable_transcendentals/deep_half_square_chain_cold_p128` | not run | not run | Approximates repeated squaring after scaling by 1/2. |
| `computable_transcendentals/deep_sqrt_square_chain_cold_p128` | not run | not run | Approximates repeated sqrt-square simplification. |
| `computable_transcendentals/inverse_half_product_chain_cold_p128` | 145.10 ns | 140.31 ns - 150.06 ns | Approximates the inverse of a deep half-product chain. |

<!-- END numerical_micro -->

<!-- BEGIN scalar_micro -->
## `scalar_micro`

Microbenchmarks for scalar operations, structural queries, cache hits, and dense exact arithmetic.

### `construction_speed`

Cost of constructing common exact scalar identities.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `construction_speed/rational_one` | not run | not run | Constructs `Rational::one()`. |
| `construction_speed/rational_new_one` | not run | not run | Constructs one through `Rational::new(1)`. |
| `construction_speed/computable_one` | not run | not run | Constructs `Computable::one()`. |
| `construction_speed/real_new_rational_one` | not run | not run | Constructs one through `Real::new(Rational::one())`. |
| `construction_speed/real_one` | not run | not run | Constructs one through `Real::one()`. |
| `construction_speed/real_from_i32_one` | not run | not run | Constructs one through integer conversion. |

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

### `pure_scalar_algorithm_speed`

Core scalar algorithms that do not require high-precision transcendental approximation.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `pure_scalar_algorithm_speed/rational_add` | not run | not run | Adds two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_mul` | 155.42 ns | 146.03 ns - 165.44 ns | Multiplies two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_div` | 33.98 ns | 33.84 ns - 34.18 ns | Divides two nontrivial rational values. |
| `pure_scalar_algorithm_speed/real_exact_add` | 444.53 ns | 441.90 ns - 447.46 ns | Adds exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_mul` | 185.83 ns | 184.71 ns - 187.12 ns | Multiplies exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_div` | 106.72 ns | 106.55 ns - 106.90 ns | Divides exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_sqrt_reduce` | not run | not run | Reduces an exact square-root expression. |
| `pure_scalar_algorithm_speed/real_exact_ln_reduce` | not run | not run | Reduces an exact logarithm of a power of two. |
| `pure_scalar_algorithm_speed/real_pow_small_integer_exponent` | 308.44 ns | 307.37 ns - 309.61 ns | Dispatches `Real::pow` with an exact small-integer exponent. |

### `borrowed_op_overhead`

Borrowed versus owned operation overhead for rational and real operands.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `borrowed_op_overhead/rational_clone_pair` | not run | not run | Clones two rational values. |
| `borrowed_op_overhead/rational_add_refs` | not run | not run | Adds rational references. |
| `borrowed_op_overhead/rational_add_owned` | not run | not run | Adds owned rational values. |
| `borrowed_op_overhead/real_clone_pair` | not run | not run | Clones two scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_unscaled_add_refs` | 170.20 ns | 169.70 ns - 170.74 ns | Adds borrowed unscaled transcendental `Real` values. |
| `borrowed_op_overhead/real_unscaled_add_owned` | not run | not run | Adds owned unscaled transcendental `Real` values. |
| `borrowed_op_overhead/real_add_refs` | 584.69 ns | 561.82 ns - 606.92 ns | Adds borrowed scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_add_owned` | not run | not run | Adds owned scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_dot3_refs_dense_symbolic` | 3.069 us | 3.060 us - 3.079 us | Computes a borrowed three-lane symbolic dot product with no rational shortcut terms. |
| `borrowed_op_overhead/real_active_dot3_refs_dense_symbolic` | 3.284 us | 3.278 us - 3.291 us | Computes a borrowed three-lane symbolic dot product after the caller has already classified every lane active. |
| `borrowed_op_overhead/real_dot3_refs_mixed_structural` | 610.23 ns | 609.34 ns - 611.21 ns | Computes a borrowed three-lane symbolic dot product with exact zero and rational scale terms. |
| `borrowed_op_overhead/real_dot4_refs_dense_symbolic` | 5.636 us | 5.625 us - 5.653 us | Computes a borrowed four-lane symbolic dot product with no rational shortcut terms. |
| `borrowed_op_overhead/real_active_dot4_refs_dense_symbolic` | 5.677 us | 5.670 us - 5.688 us | Computes a borrowed four-lane symbolic dot product after the caller has already classified every lane active. |
| `borrowed_op_overhead/real_dot4_refs_mixed_structural` | 653.48 ns | 651.96 ns - 655.30 ns | Computes a borrowed four-lane symbolic dot product with exact zero and rational scale terms. |

### `dense_algebra`

Small dense algebra kernels that stress repeated exact and symbolic operations.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `dense_algebra/rational_dot_64` | not run | not run | Computes a 64-element rational dot product. |
| `dense_algebra/rational_matmul_8` | not run | not run | Computes an 8x8 rational matrix multiply. |
| `dense_algebra/real_dot_36` | not run | not run | Computes a 36-element dot product over symbolic `Real` values. |
| `dense_algebra/real_matmul_6` | not run | not run | Computes a 6x6 matrix multiply over symbolic `Real` values. |

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
| `exact_transcendental_special_forms/atanh_sqrt_half` | 192.18 ns | 189.98 ns - 194.71 ns | Builds atanh(sqrt(2)/2) after exact structural domain checks. |
| `exact_transcendental_special_forms/atanh_sqrt_two_error` | 199.45 ns | 121.09 ns - 355.20 ns | Rejects atanh(sqrt(2)) through exact structural domain checks. |

### `symbolic_reductions`

Existing symbolic constant algebra cases considered for additional reductions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `symbolic_reductions/sqrt_pi_square` | 137.90 ns | 134.60 ns - 141.59 ns | Reduces sqrt(pi^2). |
| `symbolic_reductions/sqrt_pi_e_square` | 174.66 ns | 173.75 ns - 175.54 ns | Reduces sqrt((pi * e)^2). |
| `symbolic_reductions/ln_scaled_e` | 1.394 us | 1.382 us - 1.408 us | Reduces ln(2 * e). |
| `symbolic_reductions/sub_pi_three` | 248.16 ns | 244.62 ns - 252.32 ns | Builds the certified pi - 3 constant-offset form. |
| `symbolic_reductions/pi_minus_three_facts` | 36.67 ns | 36.34 ns - 37.03 ns | Reads structural facts for the cached pi - 3 offset form. |
| `symbolic_reductions/div_exp_exp` | 566.66 ns | 562.95 ns - 570.95 ns | Reduces e^3 / e. |
| `symbolic_reductions/div_pi_square_e` | 464.50 ns | 463.12 ns - 465.90 ns | Reduces pi^2 / e. |
| `symbolic_reductions/div_const_products` | 855.78 ns | 852.01 ns - 860.32 ns | Reduces (pi^3 * e^5) / (pi * e^2). |
| `symbolic_reductions/inverse_pi` | 89.50 ns | 89.20 ns - 89.85 ns | Builds the reciprocal of pi. |
| `symbolic_reductions/div_one_pi` | 141.50 ns | 140.79 ns - 142.37 ns | Reduces 1 / pi. |
| `symbolic_reductions/div_rational_exp` | 292.12 ns | 289.84 ns - 294.69 ns | Reduces 2 / e. |
| `symbolic_reductions/div_e_pi` | 269.92 ns | 261.15 ns - 279.59 ns | Reduces e / pi. |
| `symbolic_reductions/mul_pi_inverse_pi` | 247.62 ns | 246.97 ns - 248.30 ns | Multiplies pi by its reciprocal. |
| `symbolic_reductions/mul_pi_e_sqrt_two` | 438.83 ns | 437.88 ns - 439.69 ns | Builds the factored pi * e * sqrt(2) form. |
| `symbolic_reductions/mul_const_product_sqrt_sqrt` | 685.04 ns | 676.75 ns - 693.88 ns | Cancels sqrt(2) from (pi * e * sqrt(2)) * sqrt(2). |
| `symbolic_reductions/div_const_product_sqrt_e` | 728.58 ns | 725.63 ns - 731.29 ns | Reduces (pi * e * sqrt(2)) / e. |
| `symbolic_reductions/inverse_const_product_sqrt` | 478.56 ns | 475.73 ns - 482.08 ns | Builds a rationalized reciprocal of pi * e * sqrt(2). |
| `symbolic_reductions/inverse_sqrt_two` | 101.33 ns | 100.92 ns - 101.81 ns | Builds the rationalized reciprocal of unit-scaled sqrt(2). |
| `symbolic_reductions/div_sqrt_two_sqrt_three` | 842.52 ns | 838.35 ns - 847.63 ns | Rationalizes a quotient of two unit-scaled square roots. |

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
| `real_powi/exact_17` | 274.58 ns | 273.74 ns - 275.45 ns | Raises an exact rational-backed `Real` to the 17th power. |
| `real_powi/irrational_17` | 373.70 ns | 372.75 ns - 374.67 ns | Raises sqrt(3) to the 17th power with symbolic simplification. |

### `rational_powi`

Integer exponentiation for `Rational`.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `rational_powi/exact_17` | 181.24 ns | 180.60 ns - 181.92 ns | Raises a rational value to the 17th power. |

### `real_exact_trig`

Exact and symbolic trig construction for known pi multiples.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_trig/sin_pi_6` | not run | not run | Computes sin(pi/6) via exact shortcut. |
| `real_exact_trig/cos_pi_3` | not run | not run | Computes cos(pi/3) via exact shortcut. |
| `real_exact_trig/tan_pi_5` | not run | not run | Builds tan(pi/5), a nontrivial symbolic tangent. |

### `real_general_trig`

General trig construction for irrational arguments.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_general_trig/tan_sqrt_2` | not run | not run | Builds tan(sqrt(2)). |
| `real_general_trig/tan_pi_sqrt_2_over_5` | not run | not run | Builds tangent of an irrational multiple of pi. |

### `real_exact_inverse_trig`

Exact inverse trig shortcuts and symbolic inverse trig recognition.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_inverse_trig/asin_1_2` | 118.53 ns | 118.28 ns - 118.81 ns | Recognizes asin(1/2) as pi/6. |
| `real_exact_inverse_trig/asin_minus_1_2` | not run | not run | Recognizes asin(-1/2) as -pi/6. |
| `real_exact_inverse_trig/asin_sqrt_2_over_2` | not run | not run | Recognizes asin(sqrt(2)/2) as pi/4. |
| `real_exact_inverse_trig/asin_sin_pi_5` | not run | not run | Inverts a symbolic sin(pi/5). |
| `real_exact_inverse_trig/acos_1` | 95.78 ns | 91.89 ns - 99.71 ns | Recognizes acos(1) as zero. |
| `real_exact_inverse_trig/acos_minus_1` | 111.44 ns | 111.19 ns - 111.73 ns | Recognizes acos(-1) as pi. |
| `real_exact_inverse_trig/acos_1_2` | 117.90 ns | 117.73 ns - 118.12 ns | Recognizes acos(1/2) as pi/3. |
| `real_exact_inverse_trig/atan_1` | not run | not run | Recognizes atan(1) as pi/4. |
| `real_exact_inverse_trig/atan_sqrt_3_over_3` | not run | not run | Recognizes atan(sqrt(3)/3) as pi/6. |
| `real_exact_inverse_trig/atan_tan_pi_5` | not run | not run | Inverts a symbolic tan(pi/5). |

### `real_general_inverse_trig`

General inverse trig construction, domain errors, and atan range reduction.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_general_inverse_trig/asin_7_10` | not run | not run | Builds asin(7/10) through the rational-specialized path. |
| `real_general_inverse_trig/asin_sqrt_2_over_3` | not run | not run | Builds asin(sqrt(2)/3) through the general path. |
| `real_general_inverse_trig/acos_7_10` | not run | not run | Builds acos(7/10) through the rational-specialized asin path. |
| `real_general_inverse_trig/acos_sqrt_2_over_3` | 322.31 ns | 321.38 ns - 323.43 ns | Builds acos(sqrt(2)/3) through the general path. |
| `real_general_inverse_trig/asin_11_10_error` | not run | not run | Rejects rational asin input outside [-1, 1]. |
| `real_general_inverse_trig/acos_11_10_error` | not run | not run | Rejects rational acos input outside [-1, 1]. |
| `real_general_inverse_trig/atan_8` | not run | not run | Builds atan(8), exercising large-argument reduction. |
| `real_general_inverse_trig/atan_sqrt_2` | not run | not run | Builds atan(sqrt(2)). |

### `real_inverse_hyperbolic`

Inverse hyperbolic construction, exact exits, stable ln1p forms, and domain errors.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_inverse_hyperbolic/asinh_0` | 67.59 ns | 67.07 ns - 68.17 ns | Recognizes asinh(0) as zero. |
| `real_inverse_hyperbolic/asinh_1_2` | 202.26 ns | 201.38 ns - 203.35 ns | Builds asinh(1/2) through the stable moderate-input path. |
| `real_inverse_hyperbolic/asinh_sqrt_2` | 287.22 ns | 286.16 ns - 288.51 ns | Builds asinh(sqrt(2)) without cancellation-prone log construction. |
| `real_inverse_hyperbolic/asinh_minus_1_2` | 245.75 ns | 245.18 ns - 246.44 ns | Uses odd symmetry for negative asinh input. |
| `real_inverse_hyperbolic/asinh_1_000_000` | 203.29 ns | 202.78 ns - 203.84 ns | Builds asinh for a large positive rational. |
| `real_inverse_hyperbolic/acosh_1` | 66.87 ns | 66.63 ns - 67.18 ns | Recognizes acosh(1) as zero. |
| `real_inverse_hyperbolic/acosh_2` | 97.77 ns | 97.52 ns - 98.05 ns | Builds acosh(2) through the stable moderate-input path. |
| `real_inverse_hyperbolic/acosh_sqrt_2` | 215.71 ns | 214.84 ns - 216.49 ns | Builds acosh(sqrt(2)) through square-root domain specialization. |
| `real_inverse_hyperbolic/acosh_1_000_000` | 136.36 ns | 136.06 ns - 136.68 ns | Builds acosh for a large positive rational. |
| `real_inverse_hyperbolic/atanh_0` | not run | not run | Recognizes atanh(0) as zero. |
| `real_inverse_hyperbolic/atanh_1_2` | 143.36 ns | 142.99 ns - 143.79 ns | Builds exact-rational atanh(1/2). |
| `real_inverse_hyperbolic/atanh_minus_1_2` | 153.54 ns | 153.12 ns - 154.05 ns | Builds exact-rational atanh(-1/2). |
| `real_inverse_hyperbolic/atanh_sqrt_half` | 201.12 ns | 199.82 ns - 202.41 ns | Recognizes atanh(sqrt(2)/2) as asinh(1). |
| `real_inverse_hyperbolic/atanh_9_10` | 344.34 ns | 343.51 ns - 345.27 ns | Builds exact-rational atanh near the upper domain boundary. |
| `real_inverse_hyperbolic/atanh_1_error` | 34.69 ns | 34.38 ns - 35.02 ns | Rejects atanh(1) at the rational domain boundary. |

### `simple_inverse_functions`

Parsed/evaluated inverse trig and inverse hyperbolic expressions that should succeed.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple_inverse_functions/asin_1_2` | 160.34 ns | 159.77 ns - 160.95 ns | Evaluates `(asin 1/2)`. |
| `simple_inverse_functions/acos_1_2` | 161.46 ns | 160.92 ns - 162.02 ns | Evaluates `(acos 1/2)`. |
| `simple_inverse_functions/atan_1` | not run | not run | Evaluates `(atan 1)`. |
| `simple_inverse_functions/asin_general` | not run | not run | Evaluates `(asin 7/10)`. |
| `simple_inverse_functions/acos_general` | not run | not run | Evaluates `(acos 7/10)`. |
| `simple_inverse_functions/atan_general` | not run | not run | Evaluates `(atan 8)`. |
| `simple_inverse_functions/asinh_1_2` | 244.55 ns | 243.30 ns - 246.08 ns | Evaluates `(asinh 1/2)`. |
| `simple_inverse_functions/asinh_sqrt_2` | 999.20 ns | 994.86 ns - 1.004 us | Evaluates `(asinh (sqrt 2))`. |
| `simple_inverse_functions/acosh_2` | 135.06 ns | 134.30 ns - 135.91 ns | Evaluates `(acosh 2)`. |
| `simple_inverse_functions/acosh_sqrt_2` | 865.84 ns | 863.41 ns - 868.80 ns | Evaluates `(acosh (sqrt 2))`. |
| `simple_inverse_functions/atanh_1_2` | 178.80 ns | 178.36 ns - 179.29 ns | Evaluates `(atanh 1/2)`. |
| `simple_inverse_functions/atanh_minus_1_2` | 188.56 ns | 188.13 ns - 189.04 ns | Evaluates `(atanh -1/2)`. |

### `simple_inverse_error_functions`

Parsed/evaluated inverse function expressions that should fail quickly with `NotANumber`.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple_inverse_error_functions/asin_11_10` | not run | not run | Rejects `(asin 11/10)`. |
| `simple_inverse_error_functions/acos_sqrt_2` | 841.16 ns | 837.64 ns - 845.09 ns | Rejects `(acos (sqrt 2))`. |
| `simple_inverse_error_functions/acosh_0` | 57.18 ns | 47.49 ns - 76.24 ns | Rejects `(acosh 0)`. |
| `simple_inverse_error_functions/acosh_minus_2` | 77.01 ns | 72.58 ns - 85.36 ns | Rejects `(acosh -2)`. |
| `simple_inverse_error_functions/atanh_1` | 79.63 ns | 73.89 ns - 90.53 ns | Rejects `(atanh 1)`. |
| `simple_inverse_error_functions/atanh_sqrt_2` | 765.03 ns | 761.86 ns - 768.42 ns | Rejects `(atanh (sqrt 2))`. |

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
| `real_exact_exp_log10/exp_ln_1000` | not run | not run | Simplifies exp(ln(1000)) back to 1000. |
| `real_exact_exp_log10/exp_ln_1_8` | not run | not run | Simplifies exp(ln(1/8)) back to 1/8. |
| `real_exact_exp_log10/log10_1000` | not run | not run | Recognizes log10(1000) as 3. |
| `real_exact_exp_log10/log10_1_1000` | not run | not run | Recognizes log10(1/1000) as -3. |

<!-- END library_perf -->

<!-- BEGIN adversarial_transcendentals -->
## `adversarial_transcendentals`

Adversarial transcendental benchmarks for `hyperreal` trig, inverse trig, and inverse hyperbolic construction and approximation paths.

### `trig_adversarial_approx`

Cold approximation of sine, cosine, and tangent at exact, tiny, huge, and near-singular arguments.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `trig_adversarial_approx/sin_tiny_rational_p96` | not run | not run | Approximates sin(1e-12), stressing direct tiny-argument setup. |
| `trig_adversarial_approx/cos_tiny_rational_p96` | not run | not run | Approximates cos(1e-12), stressing direct tiny-argument setup. |
| `trig_adversarial_approx/tan_tiny_rational_p96` | not run | not run | Approximates tan(1e-12), stressing direct tiny-argument setup. |
| `trig_adversarial_approx/sin_medium_rational_p96` | not run | not run | Approximates sin(7/5), a moderate non-pi rational. |
| `trig_adversarial_approx/cos_medium_rational_p96` | not run | not run | Approximates cos(7/5), a moderate non-pi rational. |
| `trig_adversarial_approx/tan_medium_rational_p96` | 4.379 us | 3.837 us - 4.913 us | Approximates tan(7/5), a moderate non-pi rational. |
| `trig_adversarial_approx/sin_f64_exact_p96` | not run | not run | Approximates sin(1.23456789 imported as an exact dyadic rational). |
| `trig_adversarial_approx/cos_f64_exact_p96` | not run | not run | Approximates cos(1.23456789 imported as an exact dyadic rational). |
| `trig_adversarial_approx/sin_1e6_p96` | not run | not run | Approximates sin(1000000), stressing integer argument reduction. |
| `trig_adversarial_approx/cos_1e6_p96` | not run | not run | Approximates cos(1000000), stressing integer argument reduction. |
| `trig_adversarial_approx/tan_1e6_p96` | not run | not run | Approximates tan(1000000), stressing integer argument reduction. |
| `trig_adversarial_approx/sin_1e30_p96` | 2.083 us | 2.077 us - 2.089 us | Approximates sin(10^30), stressing very large integer reduction. |
| `trig_adversarial_approx/cos_1e30_p96` | 2.197 us | 2.163 us - 2.242 us | Approximates cos(10^30), stressing very large integer reduction. |
| `trig_adversarial_approx/tan_1e30_p96` | 3.432 us | 3.398 us - 3.488 us | Approximates tan(10^30), stressing very large integer reduction. |
| `trig_adversarial_approx/sin_huge_pi_plus_offset_p96` | not run | not run | Approximates sin(2^512*pi + 7/5), stressing exact pi-multiple cancellation. |
| `trig_adversarial_approx/cos_huge_pi_plus_offset_p96` | not run | not run | Approximates cos(2^512*pi + 7/5), stressing exact pi-multiple cancellation. |
| `trig_adversarial_approx/tan_huge_pi_plus_offset_p96` | 3.801 us | 3.709 us - 3.931 us | Approximates tan(2^512*pi + 7/5), stressing exact pi-multiple cancellation. |
| `trig_adversarial_approx/tan_near_half_pi_p96` | 5.432 us | 5.343 us - 5.548 us | Approximates tan(pi/2 - 2^-40), stressing the cotangent complement path. |
| `trig_adversarial_approx/tan_promoted_generated_604_125_p96` | 6.559 us | 6.504 us - 6.644 us | Promoted slow-performer tan(604/125), a generated top offender from the library-wide fuzz history. |

### `inverse_trig_adversarial_approx`

Cold approximation of asin, acos, and atan near exact values, zero, endpoints, and large atan inputs.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `inverse_trig_adversarial_approx/asin_zero_p96` | not run | not run | Approximates asin(0), which should collapse before the generic inverse-trig path. |
| `inverse_trig_adversarial_approx/acos_zero_p96` | not run | not run | Approximates acos(0), which should reduce to pi/2. |
| `inverse_trig_adversarial_approx/atan_zero_p96` | not run | not run | Approximates atan(0), which should collapse to zero. |
| `inverse_trig_adversarial_approx/asin_tiny_positive_p96` | 794.29 ns | 769.26 ns - 825.79 ns | Approximates asin(1e-12), stressing the tiny odd series. |
| `inverse_trig_adversarial_approx/acos_tiny_positive_p96` | 1.456 us | 1.450 us - 1.463 us | Approximates acos(1e-12), stressing pi/2 minus the tiny asin path. |
| `inverse_trig_adversarial_approx/atan_tiny_positive_p96` | not run | not run | Approximates atan(1e-12), stressing direct tiny atan setup. |
| `inverse_trig_adversarial_approx/asin_mid_positive_p96` | 6.527 us | 6.474 us - 6.612 us | Approximates asin(7/10), a generic in-domain value. |
| `inverse_trig_adversarial_approx/acos_mid_positive_p96` | 6.199 us | 6.163 us - 6.241 us | Approximates acos(7/10), a generic in-domain value. |
| `inverse_trig_adversarial_approx/atan_mid_positive_p96` | 3.878 us | 3.824 us - 3.944 us | Approximates atan(7/10), a generic in-domain value. |
| `inverse_trig_adversarial_approx/asin_near_one_p96` | 2.781 us | 2.776 us - 2.786 us | Approximates asin(0.999999), stressing endpoint transforms. |
| `inverse_trig_adversarial_approx/acos_near_one_p96` | 2.268 us | 2.261 us - 2.276 us | Approximates acos(0.999999), stressing endpoint transforms. |
| `inverse_trig_adversarial_approx/asin_near_minus_one_p96` | 2.959 us | 2.913 us - 3.033 us | Approximates asin(-0.999999), stressing odd symmetry near the endpoint. |
| `inverse_trig_adversarial_approx/acos_near_minus_one_p96` | 2.516 us | 2.403 us - 2.726 us | Approximates acos(-0.999999), stressing negative endpoint transforms. |
| `inverse_trig_adversarial_approx/atan_large_p96` | 1.625 us | 1.613 us - 1.642 us | Approximates atan(8), stressing reciprocal reduction. |
| `inverse_trig_adversarial_approx/atan_promoted_generated_783_412_p96` | 2.871 us | 2.844 us - 2.907 us | Promoted slow-performer atan(783/412), the generated exact-rational atan top offender. |
| `inverse_trig_adversarial_approx/ln_square_plus_one_promoted_generated_677_222_p96` | 2.932 us | 2.908 us - 2.964 us | Promoted slow-performer ln((677/222)^2 + 1), the generated exact-rational log top offender. |
| `inverse_trig_adversarial_approx/atan_huge_p96` | 637.59 ns | 630.68 ns - 647.40 ns | Approximates atan(10^30), stressing very large reciprocal reduction. |

### `trig_fuzz_adversarial_approx`

Deterministic broad sweeps of sine, cosine, and tangent over tiny, ordinary, huge, pi-offset, and near-pole exact inputs.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `trig_fuzz_adversarial_approx/sin_sweep_768_p96` | 1.763 ms | 1.740 ms - 1.801 ms | Approximates sin over 768 deterministic exact inputs spanning tiny, ordinary, huge, dyadic, rational, and pi-offset cases. |
| `trig_fuzz_adversarial_approx/cos_sweep_768_p96` | 1.760 ms | 1.755 ms - 1.765 ms | Approximates cos over the same 768-input deterministic fuzz sweep. |
| `trig_fuzz_adversarial_approx/tan_sweep_768_p96` | 3.256 ms | 3.245 ms - 3.271 ms | Approximates tan over the same deterministic sweep, including near-half-pi stress cases. |
| `trig_fuzz_adversarial_approx/sin_promoted_slow_candidates_p96` | 16.769 us | 16.740 us - 16.799 us | Approximates sin over promoted slow candidates found by prior sweep-style runs. |
| `trig_fuzz_adversarial_approx/cos_promoted_slow_candidates_p96` | 17.499 us | 17.409 us - 17.635 us | Approximates cos over promoted slow candidates found by prior sweep-style runs. |
| `trig_fuzz_adversarial_approx/tan_promoted_slow_candidates_p96` | 30.067 us | 29.919 us - 30.241 us | Approximates tan over promoted near-pole and large-reduction slow candidates. |

### `promoted_library_slow_offenders_approx`

Fifty structurally varied worst offenders promoted from the library-wide slow-performer history.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `promoted_library_slow_offenders_approx/promoted_50_structural_slow_offenders_p96` | not run | not run | Approximates 50 individual promoted slow cases spanning ln(1+x^2), atan, tan, sin, and cos over varied exact-rational structures. |

### `inverse_hyperbolic_adversarial_approx`

Cold approximation of inverse hyperbolic functions at tiny, moderate, large, and endpoint-adjacent arguments.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `inverse_hyperbolic_adversarial_approx/asinh_tiny_positive_p128` | 592.97 ns | 591.06 ns - 595.48 ns | Approximates asinh(1e-12), stressing cancellation avoidance near zero. |
| `inverse_hyperbolic_adversarial_approx/asinh_mid_positive_p128` | 7.112 us | 6.969 us - 7.278 us | Approximates asinh(1/2), a moderate positive value. |
| `inverse_hyperbolic_adversarial_approx/asinh_large_positive_p128` | 8.537 us | 8.355 us - 8.734 us | Approximates asinh(10^6), stressing large-input logarithmic behavior. |
| `inverse_hyperbolic_adversarial_approx/asinh_large_negative_p128` | 8.536 us | 8.471 us - 8.611 us | Approximates asinh(-10^6), stressing odd symmetry for large inputs. |
| `inverse_hyperbolic_adversarial_approx/acosh_one_plus_tiny_p128` | 6.701 us | 6.630 us - 6.790 us | Approximates acosh(1 + 1e-12), stressing the near-one endpoint. |
| `inverse_hyperbolic_adversarial_approx/acosh_sqrt_two_p128` | 128.95 ns | 126.43 ns - 131.96 ns | Approximates acosh(sqrt(2)), a symbolic square-root input. |
| `inverse_hyperbolic_adversarial_approx/acosh_two_p128` | 100.74 ns | 93.07 ns - 114.21 ns | Approximates acosh(2), a moderate exact rational. |
| `inverse_hyperbolic_adversarial_approx/acosh_large_positive_p128` | 8.797 us | 8.692 us - 8.924 us | Approximates acosh(10^6), stressing large-input logarithmic behavior. |
| `inverse_hyperbolic_adversarial_approx/atanh_tiny_positive_p128` | 540.27 ns | 535.23 ns - 546.50 ns | Approximates atanh(1e-12), stressing the tiny odd series. |
| `inverse_hyperbolic_adversarial_approx/atanh_mid_positive_p128` | 268.55 ns | 263.71 ns - 274.73 ns | Approximates atanh(1/2), a moderate exact rational. |
| `inverse_hyperbolic_adversarial_approx/atanh_near_one_p128` | 5.350 us | 5.254 us - 5.482 us | Approximates atanh(0.999999), stressing endpoint logarithmic behavior. |
| `inverse_hyperbolic_adversarial_approx/atanh_near_minus_one_p128` | 5.449 us | 5.422 us - 5.478 us | Approximates atanh(-0.999999), stressing odd symmetry near the endpoint. |

### `real_shortcut_adversarial`

Public `Real` construction shortcuts and domain checks for the same transcendental families.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_shortcut_adversarial/sin_exact_pi_over_six` | not run | not run | Constructs sin(pi/6), which should return the exact rational 1/2. |
| `real_shortcut_adversarial/cos_exact_pi_over_three` | not run | not run | Constructs cos(pi/3), which should return the exact rational 1/2. |
| `real_shortcut_adversarial/tan_exact_pi_over_four` | not run | not run | Constructs tan(pi/4), which should return the exact rational 1. |
| `real_shortcut_adversarial/asin_exact_half` | not run | not run | Constructs asin(1/2), which should return pi/6. |
| `real_shortcut_adversarial/acos_exact_half` | not run | not run | Constructs acos(1/2), which should return pi/3. |
| `real_shortcut_adversarial/atan_exact_one` | not run | not run | Constructs atan(1), which should return pi/4. |
| `real_shortcut_adversarial/asin_domain_error` | not run | not run | Rejects asin(1 + 1e-12). |
| `real_shortcut_adversarial/acos_domain_error` | not run | not run | Rejects acos(1 + 1e-12). |
| `real_shortcut_adversarial/atanh_endpoint_infinity` | not run | not run | Rejects atanh(1) as an infinite endpoint. |
| `real_shortcut_adversarial/atanh_domain_error` | not run | not run | Rejects atanh(1 + 1e-12). |
| `real_shortcut_adversarial/acosh_domain_error` | not run | not run | Rejects acosh(1 - 1e-12). |

<!-- END adversarial_transcendentals -->

