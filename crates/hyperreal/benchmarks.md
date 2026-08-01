<!-- BEGIN promoted_slow_offender_score -->
## `promoted_slow_offender_score`

Deterministic lexicase score for the current 100 promoted slow offenders. The score is the average current best-of-five wall-clock probe across the promoted set; lower is better. Delta compares with the previous score recorded in this file, and derivative is the change in delta.

<!-- promoted_slow_score_nanos: 50490 -->
<!-- promoted_slow_previous_score_nanos: 50490 -->
<!-- promoted_slow_score_delta_nanos: 0 -->

| Metric | Value |
| --- | ---: |
| Cases scored | 100 |
| Average score | 50.490 us |
| Delta | 0 ns |
| Delta derivative | 0 ns |

| Rank | Current Time | Operation | Input |
| ---: | ---: | --- | --- |
| 1 | 128.232 us | `generated_tan_p96` | `generated[18246] -1 187/188` |
| 2 | 125.811 us | `generated_tan_p96` | `generated[3756] -1 123/214` |
| 3 | 124.701 us | `generated_tan_p96` | `generated[5916] -1 337/578` |
| 4 | 124.112 us | `generated_tan_p96` | `generated[8976] 1 71/73` |
| 5 | 123.612 us | `generated_tan_p96` | `generated[11691] 1 431/439` |
| 6 | 123.611 us | `generated_tan_p96` | `generated[18276] -1 77/107` |
| 7 | 123.201 us | `generated_tan_p96` | `generated[12186] -1 189/299` |
| 8 | 122.572 us | `generated_tan_p96` | `generated[321] 1 214/231` |
| 9 | 122.172 us | `generated_tan_p96` | `generated[15081] 1 205/259` |
| 10 | 121.121 us | `generated_tan_p96` | `generated[486] 1 53/71` |

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
| `construction_speed/rational_one` | not run | not run | Constructs `Rational::one()`. |
| `construction_speed/rational_new_one` | not run | not run | Constructs one through `Rational::new(1)`. |
| `construction_speed/rational_from_u8_four` | not run | not run | Constructs positive four through unsigned primitive conversion. |
| `construction_speed/rational_from_i8_minus_four` | not run | not run | Constructs negative four through signed primitive conversion. |
| `construction_speed/computable_one` | not run | not run | Constructs `Computable::one()`. |
| `construction_speed/real_new_rational_one` | not run | not run | Constructs one through `Real::new(Rational::one())`. |
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
| `rational_algorithm_dispatch_speed/gcd_selected_128_bits` | 132.43 ns | 131.89 ns - 133.17 ns | Runs selected magnitude GCD on an ascending balanced two-limb pair. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_128_bits` | not run | not run | Runs the full-width Euclidean baseline on the same 128-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_192_bits` | 5.370 us | 5.342 us - 5.410 us | Runs selected magnitude GCD at the retained three-limb Lehmer crossover. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_192_bits` | not run | not run | Runs the full-width Euclidean baseline on the same 192-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_512_bits` | 11.227 us | 11.204 us - 11.254 us | Runs selected magnitude GCD above the Lehmer crossover. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_512_bits` | not run | not run | Runs the full-width Euclidean baseline on the same 512-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_1024_bits` | 21.747 us | 21.676 us - 21.824 us | Runs selected magnitude GCD above the Lehmer crossover. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_1024_bits` | not run | not run | Runs the full-width Euclidean baseline on the same 1,024-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_4096_bits` | 118.069 us | 117.625 us - 118.632 us | Runs selected magnitude GCD well above the Lehmer crossover. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_4096_bits` | not run | not run | Runs the full-width Euclidean baseline on the same 4,096-bit pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_192_bits` | 8.464 us | 8.451 us - 8.479 us | Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 192 bits. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_192_bits` | 8.538 us | 8.497 us - 8.584 us | Runs the full-width Euclidean baseline on the same initially unbalanced pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_256_bits` | 8.316 us | 8.202 us - 8.452 us | Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 256 bits. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_256_bits` | 13.967 us | 13.920 us - 14.020 us | Runs the full-width Euclidean baseline on the same initially unbalanced pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_512_bits` | 12.293 us | 12.230 us - 12.373 us | Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 512 bits. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_512_bits` | 31.514 us | 31.336 us - 31.721 us | Runs the full-width Euclidean baseline on the same initially unbalanced pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_1024_bits` | 24.132 us | 24.049 us - 24.226 us | Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 1,024 bits. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_1024_bits` | 72.899 us | 72.721 us - 73.099 us | Runs the full-width Euclidean baseline on the same initially unbalanced pair. |
| `rational_algorithm_dispatch_speed/gcd_selected_unbalanced_to_lehmer_4096_bits` | 129.132 us | 128.630 us - 129.760 us | Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 4,096 bits. |
| `rational_algorithm_dispatch_speed/gcd_euclidean_unbalanced_to_lehmer_4096_bits` | 514.854 us | 512.048 us - 517.995 us | Runs the full-width Euclidean baseline on the same initially unbalanced pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_8192_bits` | not run | not run | Runs the recursive half-GCD candidate below its provisional crossover. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_8192_bits` | not run | not run | Runs the quadratic Lehmer baseline on the same 8,192-bit pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_16384_bits` | 3.206 ms | 3.196 ms - 3.218 ms | Runs the recursive half-GCD candidate at its provisional crossover. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_16384_bits` | 935.547 us | 931.676 us - 940.070 us | Runs the quadratic Lehmer baseline on the same 16,384-bit pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_65536_bits` | 24.918 ms | 24.849 ms - 24.995 ms | Runs the recursive half-GCD candidate well above its provisional crossover. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_65536_bits` | 9.509 ms | 9.490 ms - 9.529 ms | Runs the quadratic Lehmer baseline on the same 65,536-bit pair. |
| `rational_algorithm_dispatch_speed/half_gcd_candidate_262144_bits` | 358.124 ms | 340.411 ms - 376.454 ms | Runs recursive half-GCD with selected higher-Toom matrix products at 262,144 bits. |
| `rational_algorithm_dispatch_speed/half_gcd_lehmer_262144_bits` | 140.251 ms | 137.835 ms - 142.936 ms | Runs the Lehmer baseline on the same 262,144-bit pair. |
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
| `rational_algorithm_dispatch_speed/radix_parse_short_decimal` | not run | not run | Parses a short exact decimal through the checked word-sized path. |
| `rational_algorithm_dispatch_speed/radix_parse_large_integer` | not run | not run | Parses a large below-threshold decimal fixture through chunked multiply-add conversion. |
| `rational_algorithm_dispatch_speed/radix_parse_divide_conquer_10240_digits` | not run | not run | Parses 10,240 digits through the divide-and-conquer product tree. |
| `rational_algorithm_dispatch_speed/radix_parse_backend_chunked_10240_digits` | not run | not run | Parses the same 10,240 digits with the backend chunked multiply-add baseline. |
| `rational_algorithm_dispatch_speed/radix_parse_divide_conquer_20480_digits` | not run | not run | Parses 20,480 digits through the divide-and-conquer product tree. |
| `rational_algorithm_dispatch_speed/radix_parse_backend_chunked_20480_digits` | not run | not run | Parses the same 20,480 digits with the backend chunked multiply-add baseline. |
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

<!-- END scalar_micro -->

<!-- BEGIN library_perf -->
## `library_perf`

Library-level Criterion benchmarks for public `Rational`, `Real`, and `Simple` behavior.

### `real_format`

Formatting costs for important irrational `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_format/pi_lower_exp_32` | 4.956 us | 4.924 us - 5.003 us | Formats pi with 32 digits in lower-exponential form. |
| `real_format/pi_display_alt_32` | 4.949 us | 4.944 us - 4.954 us | Formats pi with alternate decimal display at 32 digits. |
| `real_format/sqrt_two_display_alt_32` | 4.861 us | 4.850 us - 4.873 us | Formats sqrt(2) with alternate decimal display at 32 digits. |

### `real_constants`

Construction cost for shared mathematical constants.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_constants/pi` | 37.72 ns | 37.68 ns - 37.76 ns | Constructs the symbolic pi value. |
| `real_constants/e` | 44.20 ns | 44.15 ns - 44.26 ns | Constructs the symbolic Euler constant value. |

### `simple`

Parser and evaluator costs for the `Simple` expression language.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple/parse_nested` | 397.84 ns | 397.55 ns - 398.13 ns | Parses a nested expression with powers, trig, and constants. |
| `simple/eval_nested` | 1.255 us | 1.251 us - 1.260 us | Evaluates a parsed mixed symbolic/numeric expression. |
| `simple/eval_constants` | 736.20 ns | 728.93 ns - 743.34 ns | Evaluates repeated built-in constants. |
| `simple/eval_exact` | 278.23 ns | 276.45 ns - 279.97 ns | Evaluates a rational-only expression through exact shortcuts. |
| `simple/eval_nested_exact` | 879.07 ns | 872.92 ns - 885.05 ns | Evaluates a nested rational-only expression through exact shortcuts. |

### `real_powi`

Integer exponentiation for exact and irrational `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_powi/exact_17` | 121.56 ns | 121.17 ns - 121.99 ns | Raises an exact rational-backed `Real` to the 17th power. |
| `real_powi/exact_17_i64` | 89.06 ns | 88.83 ns - 89.34 ns | Raises an exact rational-backed `Real` through the machine-sized exponent API. |
| `real_powi/irrational_17` | 151.81 ns | 151.53 ns - 152.11 ns | Raises sqrt(3) to the 17th power with symbolic simplification. |
| `real_powi/large_exact_lazy_20000` | 42.916 us | 42.803 us - 43.093 us | Routes an oversized exact rational power to its bounded lazy exact representation. |

### `rational_powi`

Integer exponentiation for `Rational`.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `rational_powi/exact_17` | 80.74 ns | 80.28 ns - 81.30 ns | Raises a rational value to the 17th power. |
| `rational_powi/oversized_20000_exhausted` | 18.14 ns | 18.13 ns - 18.16 ns | Rejects eager materialization before an oversized rational power allocates its result. |

### `real_exact_trig`

Exact and symbolic trig construction for known pi multiples.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_trig/sin_pi_6` | 99.00 ns | 98.01 ns - 100.05 ns | Computes sin(pi/6) via exact shortcut. |
| `real_exact_trig/cos_pi_3` | 48.89 ns | 48.67 ns - 49.12 ns | Computes cos(pi/3) via exact shortcut. |
| `real_exact_trig/tan_pi_5` | 186.27 ns | 185.19 ns - 187.65 ns | Builds tan(pi/5), a nontrivial symbolic tangent. |

### `real_general_trig`

General trig construction for irrational arguments.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_general_trig/tan_sqrt_2` | 827.51 ns | 784.76 ns - 911.03 ns | Builds tan(sqrt(2)). |
| `real_general_trig/tan_pi_sqrt_2_over_5` | 1.450 us | 1.416 us - 1.515 us | Builds tangent of an irrational multiple of pi. |

### `real_exact_inverse_trig`

Exact inverse trig shortcuts and symbolic inverse trig recognition.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_inverse_trig/asin_1_2` | 51.37 ns | 51.28 ns - 51.48 ns | Recognizes asin(1/2) as pi/6. |
| `real_exact_inverse_trig/asin_minus_1_2` | 63.07 ns | 62.90 ns - 63.27 ns | Recognizes asin(-1/2) as -pi/6. |
| `real_exact_inverse_trig/asin_sqrt_2_over_2` | 95.92 ns | 95.48 ns - 96.40 ns | Recognizes asin(sqrt(2)/2) as pi/4. |
| `real_exact_inverse_trig/asin_sin_pi_5` | 117.34 ns | 116.56 ns - 118.05 ns | Inverts a symbolic sin(pi/5). |
| `real_exact_inverse_trig/acos_1` | 28.03 ns | 27.89 ns - 28.20 ns | Recognizes acos(1) as zero. |
| `real_exact_inverse_trig/acos_minus_1` | 44.53 ns | 39.80 ns - 53.88 ns | Recognizes acos(-1) as pi. |
| `real_exact_inverse_trig/acos_1_2` | 57.61 ns | 50.21 ns - 71.66 ns | Recognizes acos(1/2) as pi/3. |
| `real_exact_inverse_trig/atan_1` | 46.83 ns | 41.89 ns - 56.64 ns | Recognizes atan(1) as pi/4. |
| `real_exact_inverse_trig/atan_sqrt_3_over_3` | 108.68 ns | 96.89 ns - 131.89 ns | Recognizes atan(sqrt(3)/3) as pi/6. |
| `real_exact_inverse_trig/atan_tan_pi_5` | 116.65 ns | 115.87 ns - 117.35 ns | Inverts a symbolic tan(pi/5). |

### `real_general_inverse_trig`

General inverse trig construction, domain errors, and atan range reduction.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_general_inverse_trig/asin_7_10` | 185.17 ns | 180.94 ns - 188.85 ns | Builds asin(7/10) through the rational-specialized path. |
| `real_general_inverse_trig/asin_near_one` | 192.11 ns | 188.97 ns - 195.13 ns | Builds a deferred exact-rational asin near the positive endpoint. |
| `real_general_inverse_trig/asin_near_minus_one` | 110.19 ns | 109.68 ns - 110.77 ns | Builds a deferred exact-rational asin near the negative endpoint. |
| `real_general_inverse_trig/asin_sqrt_2_over_3` | 267.14 ns | 260.05 ns - 280.37 ns | Builds asin(sqrt(2)/3) through the general path. |
| `real_general_inverse_trig/acos_7_10` | 119.50 ns | 107.53 ns - 137.20 ns | Builds acos(7/10) through the rational-specialized asin path. |
| `real_general_inverse_trig/acos_sqrt_2_over_3` | 313.32 ns | 292.20 ns - 339.17 ns | Builds acos(sqrt(2)/3) through the general path. |
| `real_general_inverse_trig/asin_11_10_error` | 29.87 ns | 29.79 ns - 29.97 ns | Rejects rational asin input outside [-1, 1]. |
| `real_general_inverse_trig/acos_11_10_error` | 27.53 ns | 27.46 ns - 27.62 ns | Rejects rational acos input outside [-1, 1]. |
| `real_general_inverse_trig/atan_8` | 118.85 ns | 118.61 ns - 119.14 ns | Builds atan(8), exercising large-argument reduction. |
| `real_general_inverse_trig/atan_sqrt_2` | 8.419 us | 8.379 us - 8.477 us | Builds atan(sqrt(2)). |

### `real_inverse_hyperbolic`

Inverse hyperbolic construction, exact exits, stable ln1p forms, and domain errors.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_inverse_hyperbolic/asinh_0` | 15.51 ns | 15.49 ns - 15.54 ns | Recognizes asinh(0) as zero. |
| `real_inverse_hyperbolic/asinh_1_2` | 107.02 ns | 106.47 ns - 107.69 ns | Builds asinh(1/2) through the stable moderate-input path. |
| `real_inverse_hyperbolic/asinh_sqrt_2` | 76.48 ns | 66.45 ns - 96.26 ns | Builds asinh(sqrt(2)) without cancellation-prone log construction. |
| `real_inverse_hyperbolic/asinh_minus_1_2` | 150.29 ns | 150.14 ns - 150.46 ns | Uses odd symmetry for negative asinh input. |
| `real_inverse_hyperbolic/asinh_1_000_000` | 130.81 ns | 121.38 ns - 149.29 ns | Builds asinh for a large positive rational. |
| `real_inverse_hyperbolic/acosh_1` | 18.43 ns | 18.34 ns - 18.55 ns | Recognizes acosh(1) as zero. |
| `real_inverse_hyperbolic/acosh_2` | 42.93 ns | 37.00 ns - 54.59 ns | Builds acosh(2) through the stable moderate-input path. |
| `real_inverse_hyperbolic/acosh_sqrt_2` | 119.02 ns | 117.52 ns - 121.67 ns | Builds acosh(sqrt(2)) through square-root domain specialization. |
| `real_inverse_hyperbolic/acosh_1_000_000` | 88.18 ns | 87.55 ns - 88.85 ns | Builds acosh for a large positive rational. |
| `real_inverse_hyperbolic/atanh_0` | 15.38 ns | 15.34 ns - 15.42 ns | Recognizes atanh(0) as zero. |
| `real_inverse_hyperbolic/atanh_1_2` | 58.58 ns | 51.19 ns - 73.07 ns | Builds exact-rational atanh(1/2). |
| `real_inverse_hyperbolic/atanh_minus_1_2` | 74.28 ns | 74.08 ns - 74.54 ns | Builds exact-rational atanh(-1/2). |
| `real_inverse_hyperbolic/atanh_sqrt_half` | 107.75 ns | 107.40 ns - 108.04 ns | Recognizes atanh(sqrt(2)/2) as asinh(1). |
| `real_inverse_hyperbolic/atanh_9_10` | 129.69 ns | 128.99 ns - 130.58 ns | Builds exact-rational atanh near the upper domain boundary. |
| `real_inverse_hyperbolic/atanh_1_error` | 10.12 ns | 10.05 ns - 10.21 ns | Rejects atanh(1) at the rational domain boundary. |

### `simple_inverse_functions`

Parsed/evaluated inverse trig and inverse hyperbolic expressions that should succeed.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple_inverse_functions/asin_1_2` | 79.41 ns | 79.19 ns - 79.62 ns | Evaluates `(asin 1/2)`. |
| `simple_inverse_functions/acos_1_2` | 83.79 ns | 83.21 ns - 84.46 ns | Evaluates `(acos 1/2)`. |
| `simple_inverse_functions/atan_1` | 73.75 ns | 73.49 ns - 74.05 ns | Evaluates `(atan 1)`. |
| `simple_inverse_functions/asin_general` | 132.42 ns | 132.04 ns - 132.81 ns | Evaluates `(asin 7/10)`. |
| `simple_inverse_functions/acos_general` | 124.26 ns | 123.86 ns - 124.70 ns | Evaluates `(acos 7/10)`. |
| `simple_inverse_functions/atan_general` | 161.51 ns | 160.75 ns - 162.40 ns | Evaluates `(atan 8)`. |
| `simple_inverse_functions/asinh_1_2` | 141.57 ns | 141.34 ns - 141.78 ns | Evaluates `(asinh 1/2)`. |
| `simple_inverse_functions/asinh_sqrt_2` | 210.51 ns | 210.00 ns - 211.04 ns | Evaluates `(asinh (sqrt 2))`. |
| `simple_inverse_functions/acosh_2` | 65.76 ns | 65.56 ns - 65.94 ns | Evaluates `(acosh 2)`. |
| `simple_inverse_functions/acosh_sqrt_2` | 236.20 ns | 235.72 ns - 236.72 ns | Evaluates `(acosh (sqrt 2))`. |
| `simple_inverse_functions/atanh_1_2` | 80.08 ns | 79.81 ns - 80.37 ns | Evaluates `(atanh 1/2)`. |
| `simple_inverse_functions/atanh_minus_1_2` | 103.89 ns | 103.64 ns - 104.14 ns | Evaluates `(atanh -1/2)`. |

### `simple_inverse_error_functions`

Parsed/evaluated inverse function expressions that should fail quickly with `NotANumber`.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple_inverse_error_functions/asin_11_10` | 55.01 ns | 54.85 ns - 55.16 ns | Rejects `(asin 11/10)`. |
| `simple_inverse_error_functions/acos_sqrt_2` | 253.44 ns | 252.86 ns - 254.03 ns | Rejects `(acos (sqrt 2))`. |
| `simple_inverse_error_functions/acosh_0` | 36.87 ns | 36.70 ns - 37.03 ns | Rejects `(acosh 0)`. |
| `simple_inverse_error_functions/acosh_minus_2` | 36.88 ns | 36.71 ns - 37.04 ns | Rejects `(acosh -2)`. |
| `simple_inverse_error_functions/atanh_1` | 40.85 ns | 40.69 ns - 41.01 ns | Rejects `(atanh 1)`. |
| `simple_inverse_error_functions/atanh_sqrt_2` | 161.63 ns | 161.15 ns - 162.14 ns | Rejects `(atanh (sqrt 2))`. |

### `real_exact_ln`

Exact logarithm construction and simplification for rational inputs.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_ln/ln_1024` | 102.81 ns | 102.65 ns - 103.02 ns | Recognizes ln(1024) as 10 ln(2). |
| `real_exact_ln/ln_1_8` | 102.67 ns | 102.46 ns - 102.91 ns | Recognizes ln(1/8) as -3 ln(2). |
| `real_exact_ln/ln_1000` | 83.88 ns | 77.68 ns - 96.20 ns | Simplifies ln(1000) via small integer logarithm factors. |

### `real_exact_exp_log10`

Exact inverse relationships among exp, ln, and log10.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_exp_log10/exp_ln_1000` | 68.85 ns | 67.89 ns - 70.15 ns | Simplifies exp(ln(1000)) back to 1000. |
| `real_exact_exp_log10/exp_ln_1_8` | 79.51 ns | 79.07 ns - 79.98 ns | Simplifies exp(ln(1/8)) back to 1/8. |
| `real_exact_exp_log10/log10_1000` | 41.10 ns | 41.03 ns - 41.17 ns | Recognizes log10(1000) as 3. |
| `real_exact_exp_log10/log10_1_1000` | 72.18 ns | 71.91 ns - 72.46 ns | Recognizes log10(1/1000) as -3. |

### `real_stable_scalar_substrate`

Stable scalar constructors that preserve small residuals, dominance, roots, rational powers, and certified integer decisions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_stable_scalar_substrate/ln_1p_tiny` | 46.80 ns | 46.50 ns - 47.20 ns | Builds ln(1 + tiny) without first adding one generically. |
| `real_stable_scalar_substrate/ln_1m_tiny` | 53.10 ns | 52.22 ns - 54.22 ns | Builds ln(1 - tiny) through the log1p companion path. |
| `real_stable_scalar_substrate/expm1_tiny` | 75.56 ns | 75.30 ns - 75.88 ns | Builds exp(tiny) - 1 through the dedicated expm1 node. |
| `real_stable_scalar_substrate/softplus_large_positive` | 1.862 us | 1.857 us - 1.867 us | Builds softplus for a dominant positive input. |
| `real_stable_scalar_substrate/softplus_large_negative` | 1.752 us | 1.747 us - 1.758 us | Builds softplus for a dominant negative input. |
| `real_stable_scalar_substrate/logaddexp_dominant` | 2.026 us | 2.020 us - 2.033 us | Builds logaddexp when one side is certifiably dominant. |
| `real_stable_scalar_substrate/logsubexp_near` | 246.07 ns | 244.30 ns - 248.21 ns | Builds logsubexp for a certifiably positive but small log-space difference. |
| `real_stable_scalar_substrate/sigmoid_large_positive` | 1.821 us | 1.814 us - 1.830 us | Builds a large positive sigmoid through the stable tail path. |
| `real_stable_scalar_substrate/logit_near_one` | 297.71 ns | 296.69 ns - 299.03 ns | Builds logit close to the upper probability boundary. |
| `real_stable_scalar_substrate/sqrt1pm1_tiny` | 541.71 ns | 539.18 ns - 544.73 ns | Builds sqrt(1 + tiny) - 1 through the stable helper. |
| `real_stable_scalar_substrate/sqrt1m1_tiny` | 598.78 ns | 597.11 ns - 600.75 ns | Builds sqrt(1 - tiny) - 1 through the stable helper. |
| `real_stable_scalar_substrate/cbrt_negative_perfect` | 160.12 ns | 146.79 ns - 185.87 ns | Collapses a negative perfect cube. |
| `real_stable_scalar_substrate/root_n_perfect_fourth` | 163.82 ns | 155.40 ns - 179.77 ns | Collapses an exact fourth root. |
| `real_stable_scalar_substrate/pow_rational_negative_odd_denominator` | 215.43 ns | 215.16 ns - 215.72 ns | Routes a negative rational base through odd-root symmetry. |
| `real_stable_scalar_substrate/floor_certified_rational` | 77.39 ns | 77.02 ns - 77.91 ns | Certifies rational floor structurally. |
| `real_stable_scalar_substrate/rem_euclid_certified_rational` | 302.46 ns | 301.79 ns - 303.49 ns | Computes rational Euclidean remainder through certified quotient floor. |

### `real_geometry_polynomial_substrate`

Geometry-facing scalar helpers for rational-turn trig, removable small-angle limits, vectors, product sums, and polynomial forms.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_geometry_polynomial_substrate/sin_pi_one_sixth` | 81.14 ns | 80.93 ns - 81.38 ns | Uses exact rational-turn sine. |
| `real_geometry_polynomial_substrate/cos_pi_one_fourth` | 98.70 ns | 98.15 ns - 99.33 ns | Uses exact rational-turn cosine. |
| `real_geometry_polynomial_substrate/cos_pi_one_seventh` | 116.75 ns | 116.38 ns - 117.19 ns | Builds a non-tabulated rational-turn cosine certificate. |
| `real_geometry_polynomial_substrate/tan_pi_one_third` | 92.02 ns | 91.81 ns - 92.31 ns | Uses exact rational-turn tangent. |
| `real_geometry_polynomial_substrate/sinc_zero` | 15.46 ns | 15.40 ns - 15.52 ns | Returns the removable sinc limit at zero. |
| `real_geometry_polynomial_substrate/sinc_tiny` | 173.77 ns | 173.49 ns - 174.10 ns | Builds sinc for a tiny exact input. |
| `real_geometry_polynomial_substrate/sinc_pi_half` | 182.29 ns | 179.11 ns - 188.32 ns | Builds normalized sinc for an exact half turn. |
| `real_geometry_polynomial_substrate/cosc_tiny` | 307.64 ns | 306.10 ns - 310.19 ns | Builds the small-angle (1 - cos x) / x^2 helper. |
| `real_geometry_polynomial_substrate/atan2_axis` | 49.45 ns | 49.29 ns - 49.66 ns | Classifies an axis-aligned atan2 input exactly. |
| `real_geometry_polynomial_substrate/atan2_quadrant` | 224.44 ns | 205.49 ns - 262.01 ns | Builds a quadrant-correct atan2 expression. |
| `real_geometry_polynomial_substrate/hypot2_3_4` | 86.20 ns | 85.82 ns - 86.65 ns | Collapses a 3-4-5 norm through exact dot products. |
| `real_geometry_polynomial_substrate/hypot3_2_3_6` | 140.87 ns | 140.14 ns - 142.18 ns | Collapses a 2-3-6 norm through exact dot products. |
| `real_geometry_polynomial_substrate/hypot_minus_tiny` | 1.868 us | 1.865 us - 1.871 us | Uses rationalized hypot-minus for cancellation resistance. |
| `real_geometry_polynomial_substrate/mul_add_zero_product` | 96.45 ns | 73.96 ns - 141.11 ns | Skips a known-zero product lane. |
| `real_geometry_polynomial_substrate/sum_products_dense` | 1.270 us | 1.267 us - 1.273 us | Builds a dense product sum. |
| `real_geometry_polynomial_substrate/diff_of_products_near_cancel` | 298.61 ns | 298.40 ns - 298.84 ns | Preserves determinant-like product difference structure. |
| `real_geometry_polynomial_substrate/eval_poly_horner` | 1.001 us | 981.50 ns - 1.038 us | Evaluates a polynomial through Horner form. |
| `real_geometry_polynomial_substrate/eval_rational_poly` | 1.337 us | 1.319 us - 1.361 us | Evaluates numerator and denominator polynomial forms before division. |

### `real_normal_scientific_substrate`

Gaussian tail helpers and exact/finite scientific special-function forms added for higher numerical workloads.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_normal_scientific_substrate/erfc_zero` | 11.78 ns | 11.67 ns - 11.93 ns | Takes the exact erfc(0) exit. |
| `real_normal_scientific_substrate/erfcx_tail` | 476.35 ns | 467.37 ns - 493.02 ns | Builds scaled erfc in a positive tail. |
| `real_normal_scientific_substrate/normal_sf_tail` | 239.33 ns | 229.72 ns - 257.67 ns | Builds standard-normal upper-tail probability. |
| `real_normal_scientific_substrate/pnorm_upper_tail` | 240.87 ns | 230.90 ns - 259.94 ns | Builds the upper-tail alias. |
| `real_normal_scientific_substrate/log_pnorm_tail` | 187.54 ns | 177.13 ns - 207.93 ns | Builds lower log-CDF tail form. |
| `real_normal_scientific_substrate/log_pnorm_zero` | 66.20 ns | 66.03 ns - 66.48 ns | Takes the exact log-CDF value at zero. |
| `real_normal_scientific_substrate/log_normal_sf_tail` | 190.50 ns | 190.01 ns - 191.11 ns | Builds upper log-survival tail form. |
| `real_normal_scientific_substrate/log_normal_sf_zero` | 100.62 ns | 97.86 ns - 103.15 ns | Takes the exact log-survival value at zero. |
| `real_normal_scientific_substrate/log_dnorm_large` | 125.86 ns | 120.81 ns - 130.72 ns | Builds analytic log-density at a large input. |
| `real_normal_scientific_substrate/normal_interval_narrow` | 882.69 ns | 824.08 ns - 940.58 ns | Builds a narrow interval mass without spelling pnorm subtraction. |
| `real_normal_scientific_substrate/erfinv_mid` | 1.325 us | 1.316 us - 1.336 us | Builds inverse error function through qnorm transform. |
| `real_normal_scientific_substrate/erfcinv_tail` | 1.615 us | 1.559 us - 1.681 us | Builds inverse complementary error function through tail qnorm transform. |
| `real_normal_scientific_substrate/qnorm_upper_tail` | 1.781 us | 1.766 us - 1.796 us | Builds inverse survival quantile. |
| `real_normal_scientific_substrate/normal_pdf_parametric` | 680.67 ns | 677.03 ns - 684.70 ns | Standardizes exactly before density construction. |
| `real_normal_scientific_substrate/normal_survival_parametric` | 350.43 ns | 349.74 ns - 351.15 ns | Standardizes exactly before upper-tail construction. |
| `real_normal_scientific_substrate/normal_mills_tail` | 2.088 us | 2.072 us - 2.109 us | Builds Mills ratio through erfcx identity. |
| `real_normal_scientific_substrate/normal_mills_zero` | 20.94 ns | 20.75 ns - 21.18 ns | Takes the exact Mills ratio value at zero. |
| `real_normal_scientific_substrate/normal_hazard_tail` | 2.208 us | 2.195 us - 2.224 us | Builds reciprocal Mills hazard. |
| `real_normal_scientific_substrate/normal_hazard_zero` | 20.47 ns | 20.35 ns - 20.62 ns | Takes the exact hazard value at zero. |
| `real_normal_scientific_substrate/normal_inverse_mills_zero` | 20.52 ns | 20.41 ns - 20.66 ns | Takes the exact lower inverse Mills value at zero. |
| `real_normal_scientific_substrate/hermite_8` | 1.265 us | 1.261 us - 1.270 us | Builds an exact probabilists' Hermite polynomial. |
| `real_normal_scientific_substrate/dnorm_derivative_4` | 1.051 us | 1.043 us - 1.067 us | Combines exact Hermite polynomial with normal density. |
| `real_normal_scientific_substrate/standard_normal_moment_12` | 152.59 ns | 152.18 ns - 153.07 ns | Uses double-factorial closed form. |
| `real_normal_scientific_substrate/normal_interval_moment_3` | 1.147 us | 1.144 us - 1.150 us | Uses interval mass and density-boundary recurrence. |
| `real_normal_scientific_substrate/truncated_normal_mean` | 1.123 us | 1.115 us - 1.132 us | Builds truncated-normal mean from stable interval mass. |
| `real_normal_scientific_substrate/gamma_integer` | 215.60 ns | 215.20 ns - 216.06 ns | Uses exact integer gamma closed form. |
| `real_normal_scientific_substrate/gamma_half_integer` | 320.97 ns | 319.59 ns - 322.65 ns | Uses exact half-integer gamma closed form. |
| `real_normal_scientific_substrate/lgamma_half_integer` | 1.531 us | 1.524 us - 1.539 us | Logs the absolute half-integer gamma value. |
| `real_normal_scientific_substrate/beta_integer` | 299.62 ns | 298.63 ns - 300.73 ns | Builds integer beta through an exact factorial ratio. |
| `real_normal_scientific_substrate/ln_beta_half_integer` | 2.845 us | 2.837 us - 2.853 us | Builds log beta through lgamma sum. |
| `real_normal_scientific_substrate/regularized_beta_mid` | 1.151 us | 1.145 us - 1.159 us | Uses finite positive-integer beta binomial tail. |
| `real_normal_scientific_substrate/regularized_beta_uniform` | 147.00 ns | 146.25 ns - 147.94 ns | Takes the exact I_x(1, 1) identity. |
| `real_normal_scientific_substrate/regularized_beta_left_unity` | 307.20 ns | 305.94 ns - 308.68 ns | Reduces I_x(1, b) to one complement power. |
| `real_normal_scientific_substrate/regularized_beta_q_mid` | 836.92 ns | 834.54 ns - 839.51 ns | Uses finite positive-integer beta upper-tail form. |
| `real_normal_scientific_substrate/regularized_beta_q_uniform` | 179.83 ns | 167.55 ns - 204.02 ns | Takes the exact upper-tail I_x(1, 1) complement. |
| `real_normal_scientific_substrate/regularized_beta_q_left_unity` | 211.16 ns | 210.33 ns - 212.38 ns | Reduces the upper beta tail for a = 1 to one power. |
| `real_normal_scientific_substrate/regularized_gamma_p_half` | 1.216 us | 1.199 us - 1.245 us | Uses half-integer incomplete-gamma recurrence. |
| `real_normal_scientific_substrate/regularized_gamma_q_integer` | 1.093 us | 1.091 us - 1.094 us | Uses integer incomplete-gamma recurrence. |
| `real_normal_scientific_substrate/chi_square_sf` | 1.839 us | 1.830 us - 1.853 us | Wraps regularized upper gamma for chi-square upper tail. |

### `simple_new_function_surface`

Parser and evaluator coverage for the newly exposed stable scalar, geometry, normal, and scientific functions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple_new_function_surface/stable_log_exp_bundle` | 7.497 us | 7.461 us - 7.540 us | Evaluates log1p/log1m/expm1/softplus/logaddexp/logsubexp/sigmoid/logit together. |
| `simple_new_function_surface/geometry_bundle` | 8.746 us | 8.728 us - 8.765 us | Evaluates rational-turn trig, small-angle helpers, vector norms, product sums, and polynomials together. |
| `simple_new_function_surface/normal_bundle` | 20.550 us | 20.414 us - 20.753 us | Evaluates normal tails, log tails, interval mass, inverse tails, and moments together. |
| `simple_new_function_surface/scientific_bundle` | 14.361 us | 14.291 us - 14.441 us | Evaluates gamma, beta, regularized gamma/beta, and chi-square forms together. |
| `simple_new_function_surface/error_bundle` | 172.20 ns | 168.87 ns - 175.19 ns | Exercises fast domain failures for new public functions. |

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
