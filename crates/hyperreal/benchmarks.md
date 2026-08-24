<!-- BEGIN promoted_slow_offender_score -->
## `promoted_slow_offender_score`

Deterministic lexicase score for the current 100 promoted slow offenders. The score is the average current best-of-five wall-clock probe across the promoted set; lower is better. Delta compares with the previous score recorded in this file, and derivative is the change in delta.

<!-- promoted_slow_score_nanos: 4622 -->
<!-- promoted_slow_previous_score_nanos: 4622 -->
<!-- promoted_slow_score_delta_nanos: 0 -->

| Metric | Value |
| --- | ---: |
| Cases scored | 100 |
| Average score | 4.622 us |
| Delta | 0 ns |
| Delta derivative | 0 ns |

| Rank | Current Time | Operation | Input |
| ---: | ---: | --- | --- |
| 1 | 10.609 us | `generated_tan_p96` | `generated[18246] -1 187/188` |
| 2 | 10.549 us | `generated_tan_p96` | `generated[5676] -1 215/229` |
| 3 | 10.490 us | `generated_tan_p96` | `generated[3756] -1 123/214` |
| 4 | 10.419 us | `generated_tan_p96` | `generated[5916] -1 337/578` |
| 5 | 10.249 us | `generated_tan_p96` | `generated[8976] 1 71/73` |
| 6 | 10.219 us | `generated_tan_p96` | `generated[12186] -1 189/299` |
| 7 | 10.210 us | `generated_tan_p96` | `generated[14136] -1 79/106` |
| 8 | 10.189 us | `generated_tan_p96` | `generated[11691] 1 431/439` |
| 9 | 10.129 us | `generated_tan_p96` | `generated[321] 1 214/231` |
| 10 | 10.119 us | `generated_tan_p96` | `generated[13911] -2 134/427` |

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
| `computable_compare/compare_to_clone_shared_composite` | not run | not run | Compares two handles that share one composite expression node. |
| `computable_compare/compare_absolute_exact_rational` | not run | not run | Compares exact rationals using an absolute error tolerance. |
| `computable_compare/compare_absolute_exact_rational_same_numerator` | not run | not run | Compares exact rationals with matching numerator magnitudes. |
| `computable_compare/compare_absolute_mixed_exact_leaf_kinds` | not run | not run | Compares opposite-sign exact values stored as `One` and `Ratio` leaves. |
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
| `computable_transcendentals/exp_large_cold_p128` | not run | not run | Approximates exp(128), exercising the bounded exact-integer power path. |
| `computable_transcendentals/exp_negative_integer_cold_p128` | not run | not run | Approximates exp(-32), retaining signed ln(2) range reduction. |
| `computable_transcendentals/exp_integer_limit_cold_p128` | not run | not run | Approximates exp(256), guarding the binary e-power limit. |
| `computable_transcendentals/exp_integer_above_limit_cold_p128` | not run | not run | Approximates exp(257), retaining the ln(2) range-reduction fallback. |
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
| `computable_transcendentals/sin_f64_cold_p96` | not run | not run | Approximates sin of the exact binary64-derived dyadic for 1.23456789. |
| `computable_transcendentals/cos_f64_cold_p96` | not run | not run | Approximates cos of the exact binary64-derived dyadic for 1.23456789. |
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
| `computable_transcendentals/asinh_three_quarters_cold_p128` | not run | not run | Approximates asinh(3/4) across the series/ln1p crossover. |
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

Cost of constructing common exact scalar identities and small integers.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `construction_speed/rational_one` | 3.02 ns | 3.00 ns - 3.03 ns | Constructs `Rational::one()`. |
| `construction_speed/rational_new_one` | 3.27 ns | 3.25 ns - 3.30 ns | Constructs one through `Rational::new(1)`. |
| `construction_speed/rational_from_u8_four` | 3.71 ns | 3.70 ns - 3.73 ns | Constructs positive four through unsigned primitive conversion. |
| `construction_speed/rational_from_i8_minus_four` | 4.17 ns | 4.15 ns - 4.20 ns | Constructs negative four through signed primitive conversion. |
| `construction_speed/computable_one` | 16.18 ns | 16.14 ns - 16.24 ns | Constructs `Computable::one()`. |
| `construction_speed/real_new_rational_one` | 15.25 ns | 15.22 ns - 15.29 ns | Constructs one through `Real::new(Rational::one())`. |
| `construction_speed/real_one` | 15.54 ns | 15.50 ns - 15.58 ns | Constructs one through `Real::one()`. |
| `construction_speed/real_from_i32_one` | 15.80 ns | 15.75 ns - 15.86 ns | Constructs one through integer conversion. |
| `construction_speed/real_from_u8_four` | 16.39 ns | 16.37 ns - 16.42 ns | Constructs positive four as an exact `Real` from `u8`. |
| `construction_speed/real_from_i8_minus_four` | 16.28 ns | 16.22 ns - 16.37 ns | Constructs negative four as an exact `Real` from `i8`. |

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
| `pure_scalar_algorithm_speed/rational_sub` | not run | not run | Subtracts two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_add_wide_dyadic_cold` | not run | not run | Adds fresh integer and wide-dyadic operands without retained work. |
| `pure_scalar_algorithm_speed/rational_sub_wide_dyadic_cold` | not run | not run | Subtracts fresh integer and wide-dyadic operands without retained work. |
| `pure_scalar_algorithm_speed/rational_add_shared_cold` | not run | not run | Adds fresh operands whose storage is cloned but whose arithmetic pair is not yet observed. |
| `pure_scalar_algorithm_speed/rational_sub_shared_cold` | not run | not run | Subtracts fresh operands whose storage is cloned but whose arithmetic pair is not yet observed. |
| `pure_scalar_algorithm_speed/rational_scaled_difference_composed_cold` | not run | not run | Computes a fresh wide-integer scaled difference through multiply then subtract. |
| `pure_scalar_algorithm_speed/rational_scaled_difference_fused_cold` | not run | not run | Computes the same fresh wide-integer scaled difference with the fused integer kernel. |
| `pure_scalar_algorithm_speed/rational_cross_difference_unit_divisor_composed_cold` | not run | not run | Computes a fresh wide-integer cross difference and divides it by negative one through general operations. |
| `pure_scalar_algorithm_speed/rational_cross_difference_unit_divisor_fused_cold` | not run | not run | Computes the same cross difference through the checked fused unit-divisor path. |
| `pure_scalar_algorithm_speed/rational_mul` | not run | not run | Multiplies two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_mul_retained_general` | not run | not run | Reuses one retained exact product for an immutable rational operand pair. |
| `pure_scalar_algorithm_speed/rational_mul_wide_dyadic_cold` | not run | not run | Multiplies fresh wide-denominator dyadics whose numerators fit `u128`. |
| `pure_scalar_algorithm_speed/rational_mul_dyadic_general_cross_cancel` | not run | not run | Multiplies a wide dyadic rational by a general rational with a power-of-two numerator. |
| `pure_scalar_algorithm_speed/rational_div` | not run | not run | Divides two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_inverse_owned_cold` | not run | not run | Inverts a fresh uniquely owned nontrivial rational. |
| `pure_scalar_algorithm_speed/rational_inverse_retained` | not run | not run | Reuses the retained reciprocal of a shared nontrivial rational. |
| `pure_scalar_algorithm_speed/rational_neg_owned_cold` | not run | not run | Negates a fresh uniquely owned nontrivial rational in place. |
| `pure_scalar_algorithm_speed/rational_neg_retained` | not run | not run | Reuses the retained opposite sign of a shared nontrivial rational. |
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
| `rational_algorithm_dispatch_speed/dyadic_fact_cold` | not run | not run | Classifies a fresh non-dyadic denominator and retains the result. |
| `rational_algorithm_dispatch_speed/dyadic_fact_retained` | not run | not run | Reads an already-retained non-dyadic denominator classification. |
| `rational_algorithm_dispatch_speed/compare_leading_significand_retained_1024_bits` | not run | not run | Compares retained wide rational magnitudes through the certified leading-significand interval. |
| `rational_algorithm_dispatch_speed/compare_dyadic_shifted_retained_1024_bits` | not run | not run | Compares retained wide dyadics with equal scaled width and a five-bit denominator-shift difference. |
| `rational_algorithm_dispatch_speed/mul_backend_basecase_cold` | not run | not run | Multiplies fresh balanced 16-limb integers through the backend basecase kernel. |
| `rational_algorithm_dispatch_speed/mul_backend_half_karatsuba_cold` | not run | not run | Multiplies fresh unbalanced 33-by-66-limb integers through half-Karatsuba. |
| `rational_algorithm_dispatch_speed/mul_backend_karatsuba_cold` | not run | not run | Multiplies fresh balanced 40-limb integers through Karatsuba. |
| `rational_algorithm_dispatch_speed/mul_backend_toom3_cold` | not run | not run | Multiplies fresh balanced 257-limb integers through Toom-3. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_4096_bits` | not run | not run | Runs Hyperreal's seven-product Rust-native Toom-4 candidate on balanced 4,096-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_4096_bits` | not run | not run | Runs the native backend product on the same 4,096-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_16384_bits` | not run | not run | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 16,384-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_16384_bits` | not run | not run | Runs the native backend product on the same 16,384-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_65536_bits` | not run | not run | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 65,536-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_65536_bits` | not run | not run | Runs the native backend product on the same 65,536-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_262144_bits` | not run | not run | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 262,144-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_262144_bits` | not run | not run | Runs the native backend product on the same 262,144-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_524288_bits` | not run | not run | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 524,288-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_524288_bits` | not run | not run | Runs the native backend product on the same 524,288-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_1048576_bits` | not run | not run | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 1,048,576-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_1048576_bits` | not run | not run | Runs the native backend product on the same 1,048,576-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom4_candidate_2097152_bits` | not run | not run | Runs Hyperreal's Rust-native Toom-4 candidate on balanced 2,097,152-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_2097152_bits` | not run | not run | Runs the native backend product on the same 2,097,152-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_1048576_bits` | not run | not run | Runs the retained production Toom-8 selector above its balanced crossover. |
| `rational_algorithm_dispatch_speed/mul_selected_2097152_bits` | not run | not run | Runs the retained production Toom-8 selector on balanced 2,097,152-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_1048576_bits` | not run | not run | Runs Hyperreal's eleven-product Rust-native Toom-6 candidate above its crossover. |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_131072_bits` | not run | not run | Runs Hyperreal's Rust-native Toom-6 candidate on balanced 131,072-bit operands. |
| `rational_algorithm_dispatch_speed/mul_backend_reference_131072_bits` | not run | not run | Runs the retained native backend selector on the same 131,072-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_262144_bits` | not run | not run | Runs Hyperreal's Rust-native Toom-6 candidate on balanced 262,144-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_524288_bits` | not run | not run | Runs Hyperreal's Rust-native Toom-6 candidate on balanced 524,288-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_524288_bits` | not run | not run | Runs the retained production Toom-8 selector above its balanced crossover. |
| `rational_algorithm_dispatch_speed/mul_toom6_candidate_2097152_bits` | not run | not run | Runs Hyperreal's Rust-native Toom-6 candidate on balanced 2,097,152-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_toom4_unbalanced_1258291_by_1048576` | not run | not run | Runs retained Toom-4 on a 6:5 operand pair outside Toom-6's balance band. |
| `rational_algorithm_dispatch_speed/mul_backend_unbalanced_1258291_by_1048576` | not run | not run | Runs the native backend on the same 6:5 operand pair. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_262144_bits` | not run | not run | Runs Hyperreal's fifteen-product Rust-native Toom-8 candidate on balanced 262,144-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_262144_bits` | not run | not run | Runs the retained production Toom-8 selector at its balanced crossover. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_65536_bits` | not run | not run | Runs Hyperreal's Rust-native Toom-8 candidate on balanced 65,536-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_131072_bits` | not run | not run | Runs Hyperreal's Rust-native Toom-8 candidate on balanced 131,072-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_524288_bits` | not run | not run | Runs Hyperreal's Rust-native Toom-8 candidate at the Toom-6 crossover. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_1048576_bits` | not run | not run | Runs Hyperreal's Rust-native Toom-8 candidate on balanced 1,048,576-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_2097152_bits` | not run | not run | Runs Hyperreal's Rust-native Toom-8 candidate on balanced 2,097,152-bit operands. |
| `rational_algorithm_dispatch_speed/mul_toom8_candidate_4194304_bits` | not run | not run | Runs Hyperreal's Rust-native Toom-8 candidate on balanced 4,194,304-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_4194304_bits` | not run | not run | Runs the retained production Toom-8 selector on the same 4,194,304-bit operands. |
| `rational_algorithm_dispatch_speed/mul_selected_toom6_unbalanced_599186_by_524288` | not run | not run | Runs retained Toom-6 on an 8:7 operand pair outside Toom-8's balance band. |
| `rational_algorithm_dispatch_speed/mul_backend_unbalanced_599186_by_524288` | not run | not run | Runs the native backend on the same 8:7 operand pair. |
| `rational_algorithm_dispatch_speed/mul_ntt_candidate_262144_bits` | not run | not run | Runs Hyperreal's exact two-prime Rust-native NTT/CRT candidate on balanced 262,144-bit operands. |
| `rational_algorithm_dispatch_speed/mul_ntt_candidate_1048576_bits` | not run | not run | Runs the Rust-native NTT/CRT candidate on balanced 1,048,576-bit operands. |
| `rational_algorithm_dispatch_speed/mul_ntt_candidate_4194304_bits` | not run | not run | Runs the Rust-native NTT/CRT candidate on balanced 4,194,304-bit operands. |
| `rational_algorithm_dispatch_speed/reduce_backend_single_limb_cold` | not run | not run | Reduces a fresh wide fraction by a single-limb exact divisor. |
| `rational_algorithm_dispatch_speed/reduce_backend_knuth_cold` | not run | not run | Reduces a fresh wide fraction through normalized Knuth basecase division. |
| `rational_algorithm_dispatch_speed/reduce_backend_large_knuth_cold` | not run | not run | Reduces a fresh 129-limb numerator by a 65-limb exact divisor through normalized Knuth division. |
| `rational_algorithm_dispatch_speed/reduce_fixed_512_coprime_cold` | not run | not run | Reduces fresh balanced 512-bit operands through the fixed-limb rational-operation GCD. |
| `rational_algorithm_dispatch_speed/exact_remainder_large_knuth` | not run | not run | Computes a wide rational fractional remainder through the traced normalized Knuth backend. |
| `rational_algorithm_dispatch_speed/division_trivial_small_quotient` | not run | not run | Exercises the backend's zero-quotient magnitude division exit on wide operands. |
| `rational_algorithm_dispatch_speed/gcd_selected_128_bits` | not run | not run | Runs selected magnitude GCD on an ascending balanced two-limb pair. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_128_bits` | not run | not run | Runs the full-width Euclidean baseline on the same 128-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_192_bits` | not run | not run | Runs selected magnitude GCD at the retained three-limb Lehmer crossover. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_192_bits` | not run | not run | Runs the full-width Euclidean baseline on the same 192-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_512_bits` | not run | not run | Runs selected magnitude GCD above the Lehmer crossover. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_512_bits` | not run | not run | Runs the full-width Euclidean baseline on the same 512-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_1024_bits` | not run | not run | Runs selected magnitude GCD above the Lehmer crossover. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_1024_bits` | not run | not run | Runs the full-width Euclidean baseline on the same 1,024-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_4096_bits` | not run | not run | Runs selected magnitude GCD well above the Lehmer crossover. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_4096_bits` | not run | not run | Runs the full-width Euclidean baseline on the same 4,096-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_192_bits` | not run | not run | Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 192 bits. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_192_bits` | not run | not run | Runs the full-width Euclidean baseline on the same initially unbalanced pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_256_bits` | not run | not run | Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 256 bits. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_256_bits` | not run | not run | Runs the full-width Euclidean baseline on the same initially unbalanced pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_512_bits` | not run | not run | Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 512 bits. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_512_bits` | not run | not run | Runs the full-width Euclidean baseline on the same initially unbalanced pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_1024_bits` | not run | not run | Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 1,024 bits. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_1024_bits` | not run | not run | Runs the full-width Euclidean baseline on the same initially unbalanced pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_4096_bits` | not run | not run | Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 4,096 bits. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_4096_bits` | not run | not run | Runs the full-width Euclidean baseline on the same initially unbalanced pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_8192_bits` | not run | not run | Runs the recursive half-GCD candidate below its provisional crossover. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_8192_bits` | not run | not run | Runs the quadratic Lehmer baseline on the same 8,192-bit pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_16384_bits` | not run | not run | Runs the recursive half-GCD candidate at its provisional crossover. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_16384_bits` | not run | not run | Runs the quadratic Lehmer baseline on the same 16,384-bit pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_65536_bits` | not run | not run | Runs the recursive half-GCD candidate well above its provisional crossover. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_65536_bits` | not run | not run | Runs the quadratic Lehmer baseline on the same 65,536-bit pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_262144_bits` | not run | not run | Runs recursive half-GCD with selected higher-Toom matrix products at 262,144 bits. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_262144_bits` | not run | not run | Runs the Lehmer baseline on the same 262,144-bit pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_1048576_bits` | not run | not run | Runs recursive half-GCD with selected higher-Toom matrix products at 1,048,576 bits. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_1048576_bits` | not run | not run | Runs the Lehmer baseline on the same 1,048,576-bit pair. |
| `rational_algorithm_dispatch_speed/barrett_one_shot_8192_by_1024` | not run | not run | Prepares a Rust-native Barrett reciprocal and divides one 8,192-bit value by a 1,024-bit divisor. |
| `rational_algorithm_dispatch_speed/backend_one_shot_8192_by_1024` | not run | not run | Runs the native backend div-rem baseline for the same one-shot operands. |
| `rational_algorithm_dispatch_speed/barrett_batch16_8192_by_1024` | not run | not run | Amortizes one Rust-native Barrett reciprocal over sixteen 8,192-bit dividends. |
| `rational_algorithm_dispatch_speed/backend_batch16_8192_by_1024` | not run | not run | Runs sixteen native backend div-rem operations on the same values. |
| `rational_algorithm_dispatch_speed/barrett_batch16_65536_by_4096` | not run | not run | Amortizes one Rust-native Barrett reciprocal over sixteen 65,536-bit dividends. |
| `rational_algorithm_dispatch_speed/backend_batch16_65536_by_4096` | not run | not run | Runs sixteen native backend div-rem operations on the same large values. |
| `rational_algorithm_dispatch_speed/perfect_power_factor_reject` | not run | not run | Rejects 12 after small-factor multiplicities collapse to gcd one. |
| `rational_algorithm_dispatch_speed/perfect_power_general_seventh` | not run | not run | Discovers an exact rational seventh power whose base primes exceed the trial table. |
| `rational_algorithm_dispatch_speed/perfect_power_fixed_seventh` | not run | not run | Checks the same value when the seventh-root degree is already known. |
| `rational_algorithm_dispatch_speed/perfect_power_unfactored_reject` | not run | not run | Rejects mismatched seventh- and fifth-power rational components beyond the trial table. |
| `rational_algorithm_dispatch_speed/radix_format_small_integer` | not run | not run | Formats a 16-limb integer using repeated single-limb radix division. |
| `rational_algorithm_dispatch_speed/radix_format_large_integer` | not run | not run | Formats a 32-limb integer using divide-and-conquer radix conversion. |
| `rational_algorithm_dispatch_speed/radix_parse_short_decimal` | 91.00 ns | 90.71 ns - 91.31 ns | Parses a short exact decimal through the checked word-sized path. |
| `rational_algorithm_dispatch_speed/radix_parse_short_scientific` | 75.16 ns | 74.64 ns - 75.80 ns | Parses a representative file-I/O scientific literal through the checked word-sized path. |
| `rational_algorithm_dispatch_speed/radix_parse_wide_scientific` | 42.509 us | 42.356 us - 42.676 us | Parses a 5,120-digit significand with a negative decimal exponent exactly. |
| `rational_algorithm_dispatch_speed/radix_parse_wide_scientific_expanded` | 39.326 us | 39.180 us - 39.521 us | Parses the same exact wide value after expanding its decimal point as a baseline. |
| `rational_algorithm_dispatch_speed/radix_parse_large_integer` | 1.859 us | 1.851 us - 1.868 us | Parses a large below-threshold decimal fixture through chunked multiply-add conversion. |
| `rational_algorithm_dispatch_speed/radix_parse_divide_conquer_10240_digits` | 106.618 us | 106.377 us - 106.883 us | Parses 10,240 digits through the divide-and-conquer product tree. |
| `rational_algorithm_dispatch_speed/radix_parse_backend_chunked_10240_digits` | 103.057 us | 102.660 us - 103.555 us | Parses the same 10,240 digits with the backend chunked multiply-add baseline. |
| `rational_algorithm_dispatch_speed/radix_parse_divide_conquer_20480_digits` | 301.227 us | 299.847 us - 302.753 us | Parses 20,480 digits through the divide-and-conquer product tree. |
| `rational_algorithm_dispatch_speed/radix_parse_backend_chunked_20480_digits` | 379.236 us | 377.065 us - 381.661 us | Parses the same 20,480 digits with the backend chunked multiply-add baseline. |
| `rational_algorithm_dispatch_speed/radix_format_fraction_decimal` | not run | not run | Formats a rational decimal through exact repeated digit division. |

### `borrowed_op_overhead`

Borrowed versus owned operation overhead for rational and real operands.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `borrowed_op_overhead/rational_clone_pair` | not run | not run | Clones two rational values. |
| `borrowed_op_overhead/rational_add_refs` | not run | not run | Adds rational references. |
| `borrowed_op_overhead/rational_add_owned` | not run | not run | Adds owned rational values. |
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
| `dense_algebra/rational_dot_64` | not run | not run | Computes a 64-element rational dot product. |
| `dense_algebra/rational_matmul_8` | not run | not run | Computes an 8x8 rational matrix multiply. |
| `dense_algebra/real_dot_36` | not run | not run | Computes a 36-element dot product over symbolic `Real` values. |
| `dense_algebra/real_matmul_6` | not run | not run | Computes a 6x6 matrix multiply over symbolic `Real` values. |
| `dense_algebra/real_sum_refs_64_symbolic` | not run | not run | Constructs an arbitrary-length sum of 64 borrowed symbolic square roots. |
| `dense_algebra/real_sum_refs_64_symbolic_to_f64` | not run | not run | Constructs and approximates the same arbitrary-length symbolic sum. |

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
| `exact_transcendental_special_forms/sinh_rational_one` | not run | not run | Builds sinh(1) through the generic (exp(x) - exp(-x))/2 identity path. |
| `exact_transcendental_special_forms/cosh_rational_one` | not run | not run | Builds cosh(1) through the generic (exp(x) + exp(-x))/2 identity path. |
| `exact_transcendental_special_forms/tanh_rational_one` | not run | not run | Builds tanh(1) through the generic (exp(x) - exp(-x))/(exp(x) + exp(-x)) identity path. |
| `exact_transcendental_special_forms/atan2_origin` | not run | not run | Hits the origin (0, 0) short-circuit returning exact zero. |
| `exact_transcendental_special_forms/atan2_axis_positive_y` | not run | not run | Hits the positive-y axis short-circuit returning exact pi/2. |
| `exact_transcendental_special_forms/atan2_axis_negative_x` | not run | not run | Hits the negative-x axis short-circuit returning exact pi. |
| `exact_transcendental_special_forms/atan2_quadrant_one_unit_diagonal` | not run | not run | Quadrant I unit diagonal reduces to atan(1) = pi/4 exact special form. |
| `exact_transcendental_special_forms/atan2_quadrant_two_pi_correction` | not run | not run | Quadrant II (1, -2) exercises atan(small ratio) + pi correction. |
| `exact_transcendental_special_forms/atan2_quadrant_three_negative_pi` | not run | not run | Quadrant III (-1, -2) exercises atan(small ratio) - pi correction. |
| `exact_transcendental_special_forms/log2_power_of_two` | not run | not run | Folds log2(1024) to the exact rational 10 via the integer-log-detection shortcut. |
| `exact_transcendental_special_forms/log2_rational_three` | not run | not run | Builds log2(3) as a lightweight Log2 symbolic certificate. |
| `exact_transcendental_special_forms/log2_ln_quotient_fold` | not run | not run | Folds ln(5) / ln(2) into a Log2 certificate via the divide-recognize shortcut. |

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
| `symbolic_reductions/div_rational_exp` | not run | not run | Reduces 2 / e. |
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
| `exact_product_sums/real_signed_product_sum_rational_det3` | not run | not run | Computes a 3x3 determinant-shaped signed product sum through the public `Real` builder. |
| `exact_product_sums/real_signed_product_sum_mixed_symbolic_det3` | not run | not run | Computes the same determinant-shaped builder with symbolic factors and rational scales. |
| `exact_product_sums/exact_rational_sparse_homogeneous_plane_intersection3` | not run | not run | Computes a canonical exact three-plane cofactor tuple from one sparse dyadic row. |

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
| `real_powi/exact_17_i64` | not run | not run | Raises an exact rational-backed `Real` through the machine-sized exponent API. |
| `real_powi/irrational_17` | not run | not run | Raises sqrt(3) to the 17th power with symbolic simplification. |
| `real_powi/large_exact_lazy_20000` | not run | not run | Routes an oversized exact rational power to its bounded lazy exact representation. |

### `rational_powi`

Integer exponentiation for `Rational`.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `rational_powi/exact_17` | not run | not run | Raises a rational value to the 17th power. |
| `rational_powi/oversized_20000_exhausted` | not run | not run | Rejects eager materialization before an oversized rational power allocates its result. |

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
| `real_exact_inverse_trig/asin_1_2` | not run | not run | Recognizes asin(1/2) as pi/6. |
| `real_exact_inverse_trig/asin_minus_1_2` | not run | not run | Recognizes asin(-1/2) as -pi/6. |
| `real_exact_inverse_trig/asin_sqrt_2_over_2` | not run | not run | Recognizes asin(sqrt(2)/2) as pi/4. |
| `real_exact_inverse_trig/asin_sin_pi_5` | not run | not run | Inverts a symbolic sin(pi/5). |
| `real_exact_inverse_trig/acos_1` | not run | not run | Recognizes acos(1) as zero. |
| `real_exact_inverse_trig/acos_minus_1` | not run | not run | Recognizes acos(-1) as pi. |
| `real_exact_inverse_trig/acos_1_2` | not run | not run | Recognizes acos(1/2) as pi/3. |
| `real_exact_inverse_trig/atan_1` | not run | not run | Recognizes atan(1) as pi/4. |
| `real_exact_inverse_trig/atan_sqrt_3_over_3` | not run | not run | Recognizes atan(sqrt(3)/3) as pi/6. |
| `real_exact_inverse_trig/atan_tan_pi_5` | not run | not run | Inverts a symbolic tan(pi/5). |

### `real_general_inverse_trig`

General inverse trig construction, domain errors, and atan range reduction.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_general_inverse_trig/asin_7_10` | not run | not run | Builds asin(7/10) through the rational-specialized path. |
| `real_general_inverse_trig/asin_near_one` | not run | not run | Builds a deferred exact-rational asin near the positive endpoint. |
| `real_general_inverse_trig/asin_near_minus_one` | not run | not run | Builds a deferred exact-rational asin near the negative endpoint. |
| `real_general_inverse_trig/asin_sqrt_2_over_3` | not run | not run | Builds asin(sqrt(2)/3) through the general path. |
| `real_general_inverse_trig/acos_7_10` | not run | not run | Builds acos(7/10) through the rational-specialized asin path. |
| `real_general_inverse_trig/acos_sqrt_2_over_3` | not run | not run | Builds acos(sqrt(2)/3) through the general path. |
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
| `real_inverse_hyperbolic/acosh_sqrt_2` | not run | not run | Builds acosh(sqrt(2)) through square-root domain specialization. |
| `real_inverse_hyperbolic/acosh_1_000_000` | not run | not run | Builds acosh for a large positive rational. |
| `real_inverse_hyperbolic/atanh_0` | not run | not run | Recognizes atanh(0) as zero. |
| `real_inverse_hyperbolic/atanh_1_2` | not run | not run | Builds exact-rational atanh(1/2). |
| `real_inverse_hyperbolic/atanh_minus_1_2` | not run | not run | Builds exact-rational atanh(-1/2). |
| `real_inverse_hyperbolic/atanh_sqrt_half` | not run | not run | Recognizes atanh(sqrt(2)/2) as asinh(1). |
| `real_inverse_hyperbolic/atanh_9_10` | not run | not run | Builds exact-rational atanh near the upper domain boundary. |
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
| `simple_inverse_functions/acosh_sqrt_2` | not run | not run | Evaluates `(acosh (sqrt 2))`. |
| `simple_inverse_functions/atanh_1_2` | not run | not run | Evaluates `(atanh 1/2)`. |
| `simple_inverse_functions/atanh_minus_1_2` | not run | not run | Evaluates `(atanh -1/2)`. |

### `simple_inverse_error_functions`

Parsed/evaluated inverse function expressions that should fail quickly with `NotANumber`.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple_inverse_error_functions/asin_11_10` | not run | not run | Rejects `(asin 11/10)`. |
| `simple_inverse_error_functions/acos_sqrt_2` | not run | not run | Rejects `(acos (sqrt 2))`. |
| `simple_inverse_error_functions/acosh_0` | not run | not run | Rejects `(acosh 0)`. |
| `simple_inverse_error_functions/acosh_minus_2` | not run | not run | Rejects `(acosh -2)`. |
| `simple_inverse_error_functions/atanh_1` | not run | not run | Rejects `(atanh 1)`. |
| `simple_inverse_error_functions/atanh_sqrt_2` | not run | not run | Rejects `(atanh (sqrt 2))`. |

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

### `real_stable_scalar_substrate`

Stable scalar constructors that preserve small residuals, dominance, roots, rational powers, and certified integer decisions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_stable_scalar_substrate/ln_1p_tiny` | not run | not run | Builds ln(1 + tiny) without first adding one generically. |
| `real_stable_scalar_substrate/ln_1m_tiny` | not run | not run | Builds ln(1 - tiny) through the log1p companion path. |
| `real_stable_scalar_substrate/expm1_tiny` | not run | not run | Builds exp(tiny) - 1 through the dedicated expm1 node. |
| `real_stable_scalar_substrate/softplus_large_positive` | not run | not run | Builds softplus for a dominant positive input. |
| `real_stable_scalar_substrate/softplus_large_negative` | not run | not run | Builds softplus for a dominant negative input. |
| `real_stable_scalar_substrate/logaddexp_dominant` | not run | not run | Builds logaddexp when one side is certifiably dominant. |
| `real_stable_scalar_substrate/logsubexp_near` | not run | not run | Builds logsubexp for a certifiably positive but small log-space difference. |
| `real_stable_scalar_substrate/sigmoid_large_positive` | not run | not run | Builds a large positive sigmoid through the stable tail path. |
| `real_stable_scalar_substrate/logit_near_one` | not run | not run | Builds logit close to the upper probability boundary. |
| `real_stable_scalar_substrate/sqrt1pm1_tiny` | not run | not run | Builds sqrt(1 + tiny) - 1 through the stable helper. |
| `real_stable_scalar_substrate/sqrt1m1_tiny` | not run | not run | Builds sqrt(1 - tiny) - 1 through the stable helper. |
| `real_stable_scalar_substrate/cbrt_negative_perfect` | not run | not run | Collapses a negative perfect cube. |
| `real_stable_scalar_substrate/root_n_perfect_fourth` | not run | not run | Collapses an exact fourth root. |
| `real_stable_scalar_substrate/pow_rational_negative_odd_denominator` | not run | not run | Routes a negative rational base through odd-root symmetry. |
| `real_stable_scalar_substrate/floor_certified_rational` | not run | not run | Certifies rational floor structurally. |
| `real_stable_scalar_substrate/rem_euclid_certified_rational` | not run | not run | Computes rational Euclidean remainder through certified quotient floor. |

### `real_geometry_polynomial_substrate`

Geometry-facing scalar helpers for rational-turn trig, removable small-angle limits, vectors, product sums, and polynomial forms.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_geometry_polynomial_substrate/sin_pi_one_sixth` | not run | not run | Uses exact rational-turn sine. |
| `real_geometry_polynomial_substrate/cos_pi_one_fourth` | not run | not run | Uses exact rational-turn cosine. |
| `real_geometry_polynomial_substrate/cos_pi_one_seventh` | not run | not run | Builds a non-tabulated rational-turn cosine certificate. |
| `real_geometry_polynomial_substrate/tan_pi_one_third` | not run | not run | Uses exact rational-turn tangent. |
| `real_geometry_polynomial_substrate/sinc_zero` | not run | not run | Returns the removable sinc limit at zero. |
| `real_geometry_polynomial_substrate/sinc_tiny` | not run | not run | Builds sinc for a tiny exact input. |
| `real_geometry_polynomial_substrate/sinc_pi_half` | not run | not run | Builds normalized sinc for an exact half turn. |
| `real_geometry_polynomial_substrate/cosc_tiny` | not run | not run | Builds the small-angle (1 - cos x) / x^2 helper. |
| `real_geometry_polynomial_substrate/atan2_axis` | not run | not run | Classifies an axis-aligned atan2 input exactly. |
| `real_geometry_polynomial_substrate/atan2_quadrant` | not run | not run | Builds a quadrant-correct atan2 expression. |
| `real_geometry_polynomial_substrate/hypot2_3_4` | not run | not run | Collapses a 3-4-5 norm through exact dot products. |
| `real_geometry_polynomial_substrate/hypot3_2_3_6` | not run | not run | Collapses a 2-3-6 norm through exact dot products. |
| `real_geometry_polynomial_substrate/hypot_minus_tiny` | not run | not run | Uses rationalized hypot-minus for cancellation resistance. |
| `real_geometry_polynomial_substrate/mul_add_zero_product` | not run | not run | Skips a known-zero product lane. |
| `real_geometry_polynomial_substrate/sum_products_dense` | not run | not run | Builds a dense product sum. |
| `real_geometry_polynomial_substrate/diff_of_products_near_cancel` | not run | not run | Preserves determinant-like product difference structure. |
| `real_geometry_polynomial_substrate/eval_poly_horner` | not run | not run | Evaluates a polynomial through Horner form. |
| `real_geometry_polynomial_substrate/eval_rational_poly` | not run | not run | Evaluates numerator and denominator polynomial forms before division. |

### `real_normal_scientific_substrate`

Gaussian tail helpers and exact/finite scientific special-function forms added for higher numerical workloads.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_normal_scientific_substrate/erfc_zero` | not run | not run | Takes the exact erfc(0) exit. |
| `real_normal_scientific_substrate/erfcx_tail` | not run | not run | Builds scaled erfc in a positive tail. |
| `real_normal_scientific_substrate/normal_sf_tail` | not run | not run | Builds standard-normal upper-tail probability. |
| `real_normal_scientific_substrate/pnorm_upper_tail` | not run | not run | Builds the upper-tail alias. |
| `real_normal_scientific_substrate/log_pnorm_tail` | not run | not run | Builds lower log-CDF tail form. |
| `real_normal_scientific_substrate/log_pnorm_zero` | not run | not run | Takes the exact log-CDF value at zero. |
| `real_normal_scientific_substrate/log_normal_sf_tail` | not run | not run | Builds upper log-survival tail form. |
| `real_normal_scientific_substrate/log_normal_sf_zero` | not run | not run | Takes the exact log-survival value at zero. |
| `real_normal_scientific_substrate/log_dnorm_large` | not run | not run | Builds analytic log-density at a large input. |
| `real_normal_scientific_substrate/normal_interval_narrow` | not run | not run | Builds a narrow interval mass without spelling pnorm subtraction. |
| `real_normal_scientific_substrate/erfinv_mid` | not run | not run | Builds inverse error function through qnorm transform. |
| `real_normal_scientific_substrate/erfcinv_tail` | not run | not run | Builds inverse complementary error function through tail qnorm transform. |
| `real_normal_scientific_substrate/qnorm_upper_tail` | not run | not run | Builds inverse survival quantile. |
| `real_normal_scientific_substrate/normal_pdf_parametric` | not run | not run | Standardizes exactly before density construction. |
| `real_normal_scientific_substrate/normal_survival_parametric` | not run | not run | Standardizes exactly before upper-tail construction. |
| `real_normal_scientific_substrate/normal_mills_tail` | not run | not run | Builds Mills ratio through erfcx identity. |
| `real_normal_scientific_substrate/normal_mills_zero` | not run | not run | Takes the exact Mills ratio value at zero. |
| `real_normal_scientific_substrate/normal_hazard_tail` | not run | not run | Builds reciprocal Mills hazard. |
| `real_normal_scientific_substrate/normal_hazard_zero` | not run | not run | Takes the exact hazard value at zero. |
| `real_normal_scientific_substrate/normal_inverse_mills_zero` | not run | not run | Takes the exact lower inverse Mills value at zero. |
| `real_normal_scientific_substrate/hermite_8` | not run | not run | Builds an exact probabilists' Hermite polynomial. |
| `real_normal_scientific_substrate/dnorm_derivative_4` | not run | not run | Combines exact Hermite polynomial with normal density. |
| `real_normal_scientific_substrate/standard_normal_moment_12` | not run | not run | Uses double-factorial closed form. |
| `real_normal_scientific_substrate/normal_interval_moment_3` | not run | not run | Uses interval mass and density-boundary recurrence. |
| `real_normal_scientific_substrate/truncated_normal_mean` | not run | not run | Builds truncated-normal mean from stable interval mass. |
| `real_normal_scientific_substrate/gamma_integer` | not run | not run | Uses exact integer gamma closed form. |
| `real_normal_scientific_substrate/gamma_half_integer` | not run | not run | Uses exact half-integer gamma closed form. |
| `real_normal_scientific_substrate/lgamma_half_integer` | not run | not run | Logs the absolute half-integer gamma value. |
| `real_normal_scientific_substrate/beta_integer` | not run | not run | Builds integer beta through an exact factorial ratio. |
| `real_normal_scientific_substrate/ln_beta_half_integer` | not run | not run | Builds log beta through lgamma sum. |
| `real_normal_scientific_substrate/regularized_beta_mid` | not run | not run | Uses finite positive-integer beta binomial tail. |
| `real_normal_scientific_substrate/regularized_beta_uniform` | not run | not run | Takes the exact I_x(1, 1) identity. |
| `real_normal_scientific_substrate/regularized_beta_left_unity` | not run | not run | Reduces I_x(1, b) to one complement power. |
| `real_normal_scientific_substrate/regularized_beta_q_mid` | not run | not run | Uses finite positive-integer beta upper-tail form. |
| `real_normal_scientific_substrate/regularized_beta_q_uniform` | not run | not run | Takes the exact upper-tail I_x(1, 1) complement. |
| `real_normal_scientific_substrate/regularized_beta_q_left_unity` | not run | not run | Reduces the upper beta tail for a = 1 to one power. |
| `real_normal_scientific_substrate/regularized_gamma_p_half` | not run | not run | Uses half-integer incomplete-gamma recurrence. |
| `real_normal_scientific_substrate/regularized_gamma_q_integer` | not run | not run | Uses integer incomplete-gamma recurrence. |
| `real_normal_scientific_substrate/chi_square_sf` | not run | not run | Wraps regularized upper gamma for chi-square upper tail. |

### `simple_new_function_surface`

Parser and evaluator coverage for the newly exposed stable scalar, geometry, normal, and scientific functions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple_new_function_surface/stable_log_exp_bundle` | not run | not run | Evaluates log1p/log1m/expm1/softplus/logaddexp/logsubexp/sigmoid/logit together. |
| `simple_new_function_surface/geometry_bundle` | not run | not run | Evaluates rational-turn trig, small-angle helpers, vector norms, product sums, and polynomials together. |
| `simple_new_function_surface/normal_bundle` | not run | not run | Evaluates normal tails, log tails, interval mass, inverse tails, and moments together. |
| `simple_new_function_surface/scientific_bundle` | not run | not run | Evaluates gamma, beta, regularized gamma/beta, and chi-square forms together. |
| `simple_new_function_surface/error_bundle` | not run | not run | Exercises fast domain failures for new public functions. |

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
| `trig_adversarial_approx/tan_medium_rational_p96` | not run | not run | Approximates tan(7/5), a moderate non-pi rational. |
| `trig_adversarial_approx/sin_f64_exact_p96` | not run | not run | Approximates sin(1.23456789 imported as an exact dyadic rational). |
| `trig_adversarial_approx/cos_f64_exact_p96` | not run | not run | Approximates cos(1.23456789 imported as an exact dyadic rational). |
| `trig_adversarial_approx/sin_1e6_p96` | not run | not run | Approximates sin(1000000), stressing integer argument reduction. |
| `trig_adversarial_approx/cos_1e6_p96` | not run | not run | Approximates cos(1000000), stressing integer argument reduction. |
| `trig_adversarial_approx/tan_1e6_p96` | not run | not run | Approximates tan(1000000), stressing integer argument reduction. |
| `trig_adversarial_approx/sin_1e30_p96` | not run | not run | Approximates sin(10^30), stressing very large integer reduction. |
| `trig_adversarial_approx/cos_1e30_p96` | not run | not run | Approximates cos(10^30), stressing very large integer reduction. |
| `trig_adversarial_approx/tan_1e30_p96` | not run | not run | Approximates tan(10^30), stressing very large integer reduction. |
| `trig_adversarial_approx/sin_huge_pi_plus_offset_p96` | not run | not run | Approximates sin(2^512*pi + 7/5), stressing exact pi-multiple cancellation. |
| `trig_adversarial_approx/cos_huge_pi_plus_offset_p96` | not run | not run | Approximates cos(2^512*pi + 7/5), stressing exact pi-multiple cancellation. |
| `trig_adversarial_approx/tan_huge_pi_plus_offset_p96` | not run | not run | Approximates tan(2^512*pi + 7/5), stressing exact pi-multiple cancellation. |
| `trig_adversarial_approx/tan_near_half_pi_p96` | not run | not run | Approximates tan(pi/2 - 2^-40), stressing the cotangent complement path. |
| `trig_adversarial_approx/tan_promoted_generated_604_125_p96` | not run | not run | Promoted slow-performer tan(604/125), a generated top offender from the library-wide fuzz history. |

### `inverse_trig_adversarial_approx`

Cold approximation of asin, acos, and atan near exact values, zero, endpoints, and large atan inputs.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `inverse_trig_adversarial_approx/asin_zero_p96` | not run | not run | Approximates asin(0), which should collapse before the generic inverse-trig path. |
| `inverse_trig_adversarial_approx/acos_zero_p96` | not run | not run | Approximates acos(0), which should reduce to pi/2. |
| `inverse_trig_adversarial_approx/atan_zero_p96` | not run | not run | Approximates atan(0), which should collapse to zero. |
| `inverse_trig_adversarial_approx/asin_tiny_positive_p96` | not run | not run | Approximates asin(1e-12), stressing the tiny odd series. |
| `inverse_trig_adversarial_approx/acos_tiny_positive_p96` | not run | not run | Approximates acos(1e-12), stressing pi/2 minus the tiny asin path. |
| `inverse_trig_adversarial_approx/atan_tiny_positive_p96` | not run | not run | Approximates atan(1e-12), stressing direct tiny atan setup. |
| `inverse_trig_adversarial_approx/asin_mid_positive_p96` | not run | not run | Approximates asin(7/10), a generic in-domain value. |
| `inverse_trig_adversarial_approx/acos_mid_positive_p96` | not run | not run | Approximates acos(7/10), a generic in-domain value. |
| `inverse_trig_adversarial_approx/atan_mid_positive_p96` | not run | not run | Approximates atan(7/10), a generic in-domain value. |
| `inverse_trig_adversarial_approx/atan_two_thirds_anchor_sweep_p96` | not run | not run | Approximates atan at 11/20, 3/5, 7/10, and 4/5, covering the two-thirds table-reduction interval. |
| `inverse_trig_adversarial_approx/atan_two_thirds_anchor_sweep_p32` | not run | not run | Repeats the two-thirds table-reduction interval sweep at 32-bit precision. |
| `inverse_trig_adversarial_approx/atan_two_thirds_anchor_sweep_p256` | not run | not run | Repeats the two-thirds table-reduction interval sweep at 256-bit precision. |
| `inverse_trig_adversarial_approx/atan_two_thirds_anchor_upper_edge_p96` | not run | not run | Approximates atan(4/5), guarding the upper edge of the two-thirds table-reduction interval against a local regression. |
| `inverse_trig_adversarial_approx/asin_near_one_p96` | not run | not run | Approximates asin(0.999999), stressing endpoint transforms. |
| `inverse_trig_adversarial_approx/acos_near_one_p96` | not run | not run | Approximates acos(0.999999), stressing endpoint transforms. |
| `inverse_trig_adversarial_approx/asin_near_minus_one_p96` | not run | not run | Approximates asin(-0.999999), stressing odd symmetry near the endpoint. |
| `inverse_trig_adversarial_approx/acos_near_minus_one_p96` | not run | not run | Approximates acos(-0.999999), stressing negative endpoint transforms. |
| `inverse_trig_adversarial_approx/atan_large_p96` | not run | not run | Approximates atan(8), stressing reciprocal reduction. |
| `inverse_trig_adversarial_approx/atan_promoted_generated_783_412_p96` | not run | not run | Promoted slow-performer atan(783/412), the generated exact-rational atan top offender. |
| `inverse_trig_adversarial_approx/ln_square_plus_one_promoted_generated_677_222_p96` | not run | not run | Promoted slow-performer ln((677/222)^2 + 1), the generated exact-rational log top offender. |
| `inverse_trig_adversarial_approx/atan_huge_p96` | not run | not run | Approximates atan(10^30), stressing very large reciprocal reduction. |

### `trig_fuzz_adversarial_approx`

Deterministic broad sweeps of sine, cosine, and tangent over tiny, ordinary, huge, pi-offset, and near-pole exact inputs.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `trig_fuzz_adversarial_approx/sin_sweep_768_p96` | not run | not run | Approximates sin over 768 deterministic exact inputs spanning tiny, ordinary, huge, dyadic, rational, and pi-offset cases. |
| `trig_fuzz_adversarial_approx/cos_sweep_768_p96` | not run | not run | Approximates cos over the same 768-input deterministic fuzz sweep. |
| `trig_fuzz_adversarial_approx/tan_sweep_768_p96` | not run | not run | Approximates tan over the same deterministic sweep, including near-half-pi stress cases. |
| `trig_fuzz_adversarial_approx/sin_promoted_slow_candidates_p96` | not run | not run | Approximates sin over promoted slow candidates found by prior sweep-style runs. |
| `trig_fuzz_adversarial_approx/cos_promoted_slow_candidates_p96` | not run | not run | Approximates cos over promoted slow candidates found by prior sweep-style runs. |
| `trig_fuzz_adversarial_approx/tan_promoted_slow_candidates_p96` | not run | not run | Approximates tan over promoted near-pole and large-reduction slow candidates. |

### `promoted_library_slow_offenders_approx`

Fifty structurally varied worst offenders promoted from the library-wide slow-performer history.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `promoted_library_slow_offenders_approx/promoted_50_structural_slow_offenders_p96` | not run | not run | Approximates 50 individual promoted slow cases spanning ln(1+x^2), atan, tan, sin, and cos over varied exact-rational structures. |

### `inverse_hyperbolic_adversarial_approx`

Cold approximation of inverse hyperbolic functions at tiny, moderate, large, and endpoint-adjacent arguments.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `inverse_hyperbolic_adversarial_approx/asinh_tiny_positive_p128` | not run | not run | Approximates asinh(1e-12), stressing cancellation avoidance near zero. |
| `inverse_hyperbolic_adversarial_approx/asinh_mid_positive_p128` | not run | not run | Approximates asinh(1/2), a moderate positive value. |
| `inverse_hyperbolic_adversarial_approx/asinh_large_positive_p128` | not run | not run | Approximates asinh(10^6), stressing large-input logarithmic behavior. |
| `inverse_hyperbolic_adversarial_approx/asinh_large_negative_p128` | not run | not run | Approximates asinh(-10^6), stressing odd symmetry for large inputs. |
| `inverse_hyperbolic_adversarial_approx/acosh_one_plus_tiny_p128` | not run | not run | Approximates acosh(1 + 1e-12), stressing the near-one endpoint. |
| `inverse_hyperbolic_adversarial_approx/acosh_sqrt_two_p128` | not run | not run | Approximates acosh(sqrt(2)), a symbolic square-root input. |
| `inverse_hyperbolic_adversarial_approx/acosh_two_p128` | not run | not run | Approximates acosh(2), a moderate exact rational. |
| `inverse_hyperbolic_adversarial_approx/acosh_large_positive_p128` | not run | not run | Approximates acosh(10^6), stressing large-input logarithmic behavior. |
| `inverse_hyperbolic_adversarial_approx/atanh_tiny_positive_p128` | not run | not run | Approximates atanh(1e-12), stressing the tiny odd series. |
| `inverse_hyperbolic_adversarial_approx/atanh_mid_positive_p128` | not run | not run | Approximates atanh(1/2), a moderate exact rational. |
| `inverse_hyperbolic_adversarial_approx/atanh_near_one_p128` | not run | not run | Approximates atanh(0.999999), stressing endpoint logarithmic behavior. |
| `inverse_hyperbolic_adversarial_approx/atanh_near_minus_one_p128` | not run | not run | Approximates atanh(-0.999999), stressing odd symmetry near the endpoint. |

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

<!-- BEGIN borrowed_ops -->
## `borrowed_ops`

Compares owned arithmetic with borrowed arithmetic for exact and irrational values.

### `rational_ops`

Owned versus borrowed arithmetic for exact `Rational` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `rational_ops/add_owned` | not run | not run | Adds cloned owned operands. |
| `rational_ops/add_refs` | not run | not run | Adds borrowed operands without cloning both inputs. |
| `rational_ops/sub_owned` | not run | not run | Subtracts cloned owned operands. |
| `rational_ops/sub_refs` | not run | not run | Subtracts borrowed operands. |
| `rational_ops/mul_owned` | not run | not run | Multiplies cloned owned operands. |
| `rational_ops/mul_refs` | not run | not run | Multiplies borrowed operands. |
| `rational_ops/div_owned` | not run | not run | Divides cloned owned operands. |
| `rational_ops/div_refs` | not run | not run | Divides borrowed operands. |

### `real_ops`

Owned versus borrowed arithmetic for exact rational-backed `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_ops/add_owned` | not run | not run | Adds cloned owned operands. |
| `real_ops/add_refs` | not run | not run | Adds borrowed operands without cloning both inputs. |
| `real_ops/sub_owned` | not run | not run | Subtracts cloned owned operands. |
| `real_ops/sub_refs` | not run | not run | Subtracts borrowed operands. |
| `real_ops/mul_owned` | not run | not run | Multiplies cloned owned operands. |
| `real_ops/mul_refs` | not run | not run | Multiplies borrowed operands. |
| `real_ops/div_owned` | not run | not run | Divides cloned owned operands. |
| `real_ops/div_refs` | not run | not run | Divides borrowed operands. |

### `real_irrational_ops`

Owned versus borrowed arithmetic for symbolic irrational `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_irrational_ops/add_owned` | not run | not run | Adds cloned owned operands. |
| `real_irrational_ops/add_refs` | not run | not run | Adds borrowed operands without cloning both inputs. |
| `real_irrational_ops/sub_owned` | not run | not run | Subtracts cloned owned operands. |
| `real_irrational_ops/sub_refs` | not run | not run | Subtracts borrowed operands. |
| `real_irrational_ops/mul_owned` | not run | not run | Multiplies cloned owned operands. |
| `real_irrational_ops/mul_refs` | not run | not run | Multiplies borrowed operands. |
| `real_irrational_ops/div_owned` | not run | not run | Divides cloned owned operands. |
| `real_irrational_ops/div_refs` | not run | not run | Divides borrowed operands. |

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
