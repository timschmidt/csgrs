# Benchmark Reference

This file is updated by the Criterion benchmark binaries. Run `cargo bench` to refresh the benchmark output catalogue.

Each table includes the latest Criterion mean and 95% confidence interval when results are available. Raw Criterion reports remain under `target/criterion/`.

<!-- BEGIN borrowed_ops -->
## `borrowed_ops`

Compares owned arithmetic with borrowed arithmetic for exact and irrational values.

### `rational_ops`

Owned versus borrowed arithmetic for exact `Rational` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `rational_ops/add_owned` | 389.45 ns | 386.25 ns - 393.63 ns | Adds cloned owned operands. |
| `rational_ops/add_refs` | 363.65 ns | 362.45 ns - 364.88 ns | Adds borrowed operands without cloning both inputs. |
| `rational_ops/sub_owned` | 392.96 ns | 391.58 ns - 394.35 ns | Subtracts cloned owned operands. |
| `rational_ops/sub_refs` | 371.11 ns | 368.75 ns - 374.14 ns | Subtracts borrowed operands. |
| `rational_ops/mul_owned` | 138.11 ns | 137.49 ns - 138.78 ns | Multiplies cloned owned operands. |
| `rational_ops/mul_refs` | 112.76 ns | 112.58 ns - 112.96 ns | Multiplies borrowed operands. |
| `rational_ops/div_owned` | 610.20 ns | 601.01 ns - 622.02 ns | Divides cloned owned operands. |
| `rational_ops/div_refs` | 567.84 ns | 566.78 ns - 568.94 ns | Divides borrowed operands. |

### `real_ops`

Owned versus borrowed arithmetic for exact rational-backed `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_ops/add_owned` | 478.35 ns | 476.83 ns - 479.91 ns | Adds cloned owned operands. |
| `real_ops/add_refs` | 420.44 ns | 419.59 ns - 421.43 ns | Adds borrowed operands without cloning both inputs. |
| `real_ops/sub_owned` | 481.36 ns | 480.02 ns - 482.73 ns | Subtracts cloned owned operands. |
| `real_ops/sub_refs` | 434.90 ns | 431.68 ns - 438.10 ns | Subtracts borrowed operands. |
| `real_ops/mul_owned` | 224.96 ns | 224.24 ns - 225.78 ns | Multiplies cloned owned operands. |
| `real_ops/mul_refs` | 169.71 ns | 169.37 ns - 170.12 ns | Multiplies borrowed operands. |
| `real_ops/div_owned` | 714.03 ns | 711.64 ns - 716.83 ns | Divides cloned owned operands. |
| `real_ops/div_refs` | 651.32 ns | 649.37 ns - 653.50 ns | Divides borrowed operands. |

### `real_irrational_ops`

Owned versus borrowed arithmetic for symbolic irrational `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_irrational_ops/add_owned` | 312.93 ns | 311.09 ns - 314.64 ns | Adds cloned owned operands. |
| `real_irrational_ops/add_refs` | 212.76 ns | 212.34 ns - 213.22 ns | Adds borrowed operands without cloning both inputs. |
| `real_irrational_ops/sub_owned` | 339.46 ns | 337.64 ns - 341.22 ns | Subtracts cloned owned operands. |
| `real_irrational_ops/sub_refs` | 235.88 ns | 235.37 ns - 236.46 ns | Subtracts borrowed operands. |
| `real_irrational_ops/mul_owned` | 1.308 us | 1.304 us - 1.312 us | Multiplies cloned owned operands. |
| `real_irrational_ops/mul_refs` | 1.165 us | 1.159 us - 1.172 us | Multiplies borrowed operands. |
| `real_irrational_ops/div_owned` | 2.034 us | 1.713 us - 2.671 us | Divides cloned owned operands. |
| `real_irrational_ops/div_refs` | 1.544 us | 1.542 us - 1.546 us | Divides borrowed operands. |

<!-- END borrowed_ops -->

<!-- BEGIN scalar_micro -->
## `scalar_micro`

Microbenchmarks for scalar operations, structural queries, cache hits, and dense exact arithmetic.

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
| `pure_scalar_algorithm_speed/rational_mul` | not run | not run | Multiplies two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_div` | not run | not run | Divides two nontrivial rational values. |
| `pure_scalar_algorithm_speed/real_exact_add` | not run | not run | Adds exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_mul` | not run | not run | Multiplies exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_div` | not run | not run | Divides exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_sqrt_reduce` | not run | not run | Reduces an exact square-root expression. |
| `pure_scalar_algorithm_speed/real_exact_ln_reduce` | not run | not run | Reduces an exact logarithm of a power of two. |

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
| `exact_transcendental_special_forms/sin_pi_7` | 552.99 ns | 546.44 ns - 561.66 ns | Builds the exact special form for sin(pi/7). |
| `exact_transcendental_special_forms/cos_pi_7` | 1.061 us | 1.056 us - 1.066 us | Builds the exact special form for cos(pi/7). |
| `exact_transcendental_special_forms/tan_pi_7` | 496.04 ns | 493.28 ns - 499.45 ns | Builds the exact special form for tan(pi/7). |
| `exact_transcendental_special_forms/asin_sin_6pi_7` | 1.089 us | 1.083 us - 1.095 us | Recognizes the principal branch of asin(sin(6pi/7)). |
| `exact_transcendental_special_forms/acos_cos_9pi_7` | 2.471 us | 2.458 us - 2.485 us | Recognizes the principal branch of acos(cos(9pi/7)). |
| `exact_transcendental_special_forms/atan_tan_6pi_7` | 1.115 us | 1.103 us - 1.129 us | Recognizes the principal branch of atan(tan(6pi/7)). |
| `exact_transcendental_special_forms/asinh_large` | 3.684 us | 3.661 us - 3.710 us | Builds a large inverse hyperbolic sine without exact intermediate Reals. |
| `exact_transcendental_special_forms/atanh_sqrt_half` | 8.187 us | 7.326 us - 9.874 us | Builds atanh(sqrt(2)/2) after exact structural domain checks. |

<!-- END scalar_micro -->

<!-- BEGIN float_convert -->
## `float_convert`

Covers exact import of floating-point values, including public `Real` conversion overhead.

### `float_convert`

Exact conversion from IEEE-754 floats into `Rational` and `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `float_convert/f32_normal` | 64.11 ns | 63.80 ns - 64.52 ns | Converts a normal `f32` into an exact `Rational`. |
| `float_convert/f64_normal` | 61.89 ns | 61.34 ns - 62.42 ns | Converts a normal `f64` into an exact `Rational`. |
| `float_convert/f64_binary_fraction` | 61.55 ns | 61.31 ns - 61.86 ns | Converts an exactly representable binary `f64` fraction into `Rational`. |
| `float_convert/f64_subnormal` | 75.76 ns | 74.85 ns - 76.68 ns | Converts a subnormal `f64` into an exact `Rational`. |
| `float_convert/real_f32_normal` | 135.61 ns | 134.08 ns - 137.13 ns | Converts a normal `f32` through the public `Real::try_from` path. |
| `float_convert/real_f64_normal` | 131.66 ns | 130.51 ns - 133.67 ns | Converts a normal `f64` through the public `Real::try_from` path. |
| `float_convert/real_f64_subnormal` | 146.39 ns | 145.65 ns - 147.40 ns | Converts a subnormal `f64` through the public `Real::try_from` path. |

<!-- END float_convert -->

<!-- BEGIN numerical_micro -->
## `numerical_micro`

Low-level `Computable` microbenchmarks for approximation kernels, caches, structural facts, comparisons, and deep evaluator trees.

### `computable_cache`

Cold versus cached approximation of basic `Computable` expressions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_cache/ratio_approx_cold_p128` | 104.11 ns | 103.71 ns - 104.54 ns | Approximates a rational value at p=-128 from a fresh clone. |
| `computable_cache/ratio_approx_cached_p128` | 41.87 ns | 41.74 ns - 42.01 ns | Repeats an already cached rational approximation at p=-128. |
| `computable_cache/pi_approx_cold_p128` | 63.53 ns | 63.13 ns - 64.03 ns | Approximates pi at p=-128 from a fresh clone. |
| `computable_cache/pi_approx_cached_p128` | 43.05 ns | 42.96 ns - 43.15 ns | Repeats an already cached pi approximation at p=-128. |
| `computable_cache/pi_plus_tiny_cold_p128` | 293.06 ns | 259.02 ns - 356.90 ns | Approximates pi plus a tiny exact rational perturbation. |
| `computable_cache/pi_minus_tiny_cold_p128` | 256.39 ns | 255.47 ns - 257.36 ns | Approximates pi minus a tiny exact rational perturbation. |

### `computable_bounds`

Structural sign and bound discovery for deep or perturbed computable trees.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_bounds/deep_scaled_product_sign` | 172.53 ns | 140.97 ns - 233.98 ns | Finds the sign of a deep scaled product. |
| `computable_bounds/scaled_square_sign` | 614.56 ns | 344.01 ns - 1.155 us | Finds the sign of repeated squaring with exact scale factors. |
| `computable_bounds/sqrt_scaled_square_sign` | 242.95 ns | 241.05 ns - 244.78 ns | Finds the sign after taking a square root of a scaled square. |
| `computable_bounds/deep_structural_bound_sign` | 36.470 us | 28.875 us - 51.584 us | Finds sign through repeated multiply/inverse/negate structural transformations. |
| `computable_bounds/deep_structural_bound_sign_cached` | 3.79 ns | 3.78 ns - 3.80 ns | Reads the cached sign of the deep structural-bound chain. |
| `computable_bounds/deep_structural_bound_facts_cached` | 12.39 ns | 12.37 ns - 12.42 ns | Reads cached structural facts for the deep structural-bound chain. |
| `computable_bounds/perturbed_scaled_product_sign` | 316.33 ns | 237.23 ns - 472.03 ns | Finds sign for a deeply scaled value with a tiny perturbation. |
| `computable_bounds/perturbed_scaled_product_sign_until` | 271.80 ns | 238.03 ns - 337.36 ns | Refines sign for the perturbed scaled product only to p=-128. |
| `computable_bounds/pi_minus_tiny_sign` | 417.61 ns | 383.73 ns - 484.17 ns | Finds sign for pi minus a tiny exact rational. |
| `computable_bounds/pi_minus_tiny_sign_cached` | 3.77 ns | 3.77 ns - 3.78 ns | Reads cached sign for pi minus a tiny exact rational. |

### `computable_compare`

Ordering and absolute-comparison shortcuts.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_compare/compare_to_opposite_sign` | 10.37 ns | 10.34 ns - 10.42 ns | Compares values with known opposite signs. |
| `computable_compare/compare_to_exact_msd_gap` | 17.25 ns | 17.21 ns - 17.29 ns | Compares values with a large exact magnitude gap. |
| `computable_compare/compare_absolute_exact_rational` | 86.73 ns | 86.49 ns - 87.00 ns | Compares absolute values of exact rationals. |
| `computable_compare/compare_absolute_dominant_add` | 150.21 ns | 149.93 ns - 150.54 ns | Compares a dominant term against the same term plus a tiny addend. |
| `computable_compare/compare_absolute_exact_msd_gap` | 151.49 ns | 151.03 ns - 152.02 ns | Compares absolute values with a large exact magnitude gap. |

### `computable_transcendentals`

Low-level approximation kernels and deep expression-tree stress cases.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_transcendentals/legacy_exp_one_p128` | 2.901 us | 2.892 us - 2.912 us | Runs the legacy direct exp series for input 1 at p=-128. |
| `computable_transcendentals/e_constant_cold_p128` | 63.99 ns | 63.50 ns - 64.60 ns | Approximates the shared e constant from a fresh clone. |
| `computable_transcendentals/e_constant_cached_p128` | 43.77 ns | 43.71 ns - 43.85 ns | Repeats a cached approximation of e. |
| `computable_transcendentals/legacy_exp_half_p128` | 2.507 us | 2.499 us - 2.518 us | Runs the legacy direct exp series for input 1/2 at p=-128. |
| `computable_transcendentals/exp_cold_p128` | 2.395 us | 2.392 us - 2.398 us | Approximates exp(7/5) from a fresh clone. |
| `computable_transcendentals/exp_cached_p128` | 42.05 ns | 42.01 ns - 42.09 ns | Repeats a cached exp(7/5) approximation. |
| `computable_transcendentals/exp_large_cold_p128` | 7.851 us | 7.831 us - 7.875 us | Approximates exp(128), exercising large-argument reduction. |
| `computable_transcendentals/exp_half_cold_p128` | 2.874 us | 2.870 us - 2.879 us | Approximates exp(1/2). |
| `computable_transcendentals/exp_near_limit_cold_p128` | 2.885 us | 2.874 us - 2.897 us | Approximates exp near a prescaling threshold. |
| `computable_transcendentals/exp_near_limit_cached_p128` | 42.12 ns | 41.99 ns - 42.32 ns | Repeats a cached near-threshold exp approximation. |
| `computable_transcendentals/exp_zero_cold_p128` | 93.81 ns | 88.35 ns - 104.47 ns | Approximates exp(0). |
| `computable_transcendentals/ln_cold_p128` | 4.473 us | 4.466 us - 4.481 us | Approximates ln(11/7). |
| `computable_transcendentals/ln_cached_p128` | 41.79 ns | 41.73 ns - 41.86 ns | Repeats a cached ln(11/7) approximation. |
| `computable_transcendentals/ln_large_cold_p128` | 290.37 ns | 289.46 ns - 291.42 ns | Approximates ln(1024), exercising large-input reduction. |
| `computable_transcendentals/ln_large_cached_p128` | 42.03 ns | 41.97 ns - 42.10 ns | Repeats a cached ln(1024) approximation. |
| `computable_transcendentals/ln_tiny_cold_p128` | 243.90 ns | 242.43 ns - 245.53 ns | Approximates ln(2^-1024), exercising tiny-input reduction. |
| `computable_transcendentals/ln_near_limit_cold_p128` | 4.499 us | 4.489 us - 4.510 us | Approximates ln near the prescaled-ln limit. |
| `computable_transcendentals/ln_near_limit_cached_p128` | 41.86 ns | 41.78 ns - 41.94 ns | Repeats a cached near-limit ln approximation. |
| `computable_transcendentals/ln_one_cold_p128` | 67.62 ns | 45.00 ns - 112.64 ns | Approximates ln(1). |
| `computable_transcendentals/sqrt_cold_p128` | 755.82 ns | 746.54 ns - 768.89 ns | Approximates sqrt(2). |
| `computable_transcendentals/sqrt_cached_p128` | 42.01 ns | 41.97 ns - 42.06 ns | Repeats a cached sqrt(2) approximation. |
| `computable_transcendentals/sqrt_single_scaled_square_cold_p128` | 2.283 us | 2.269 us - 2.300 us | Builds and approximates sqrt((7*pi/8)^2). |
| `computable_transcendentals/sin_cold_p96` | 2.081 us | 2.075 us - 2.087 us | Approximates sin(7/5). |
| `computable_transcendentals/sin_cached_p96` | 42.28 ns | 42.06 ns - 42.53 ns | Repeats a cached sin(7/5) approximation. |
| `computable_transcendentals/cos_cold_p96` | 4.851 us | 4.843 us - 4.860 us | Approximates cos(7/5). |
| `computable_transcendentals/cos_cached_p96` | 42.04 ns | 41.95 ns - 42.14 ns | Repeats a cached cos(7/5) approximation. |
| `computable_transcendentals/tan_cold_p96` | 3.991 us | 3.981 us - 4.004 us | Approximates tan(7/5). |
| `computable_transcendentals/tan_cached_p96` | 42.08 ns | 41.91 ns - 42.28 ns | Repeats a cached tan(7/5) approximation. |
| `computable_transcendentals/sin_zero_cold_p96` | 46.10 ns | 45.92 ns - 46.28 ns | Approximates sin(0). |
| `computable_transcendentals/cos_zero_cold_p96` | 84.53 ns | 67.55 ns - 118.23 ns | Approximates cos(0). |
| `computable_transcendentals/tan_zero_cold_p96` | 45.42 ns | 45.10 ns - 45.72 ns | Approximates tan(0). |
| `computable_transcendentals/tan_near_half_pi_cold_p96` | 3.221 us | 3.212 us - 3.230 us | Approximates tangent near pi/2. |
| `computable_transcendentals/tan_near_half_pi_cached_p96` | 41.91 ns | 41.86 ns - 41.98 ns | Repeats cached tangent near pi/2. |
| `computable_transcendentals/sin_huge_cold_p96` | 2.922 us | 2.906 us - 2.939 us | Approximates sine of a huge pi multiple plus offset. |
| `computable_transcendentals/cos_huge_cold_p96` | 8.352 us | 8.337 us - 8.367 us | Approximates cosine of a huge pi multiple plus offset. |
| `computable_transcendentals/tan_huge_cold_p96` | 4.737 us | 4.729 us - 4.745 us | Approximates tangent of a huge pi multiple plus offset. |
| `computable_transcendentals/asin_cold_p96` | 11.988 us | 11.975 us - 12.003 us | Approximates a computable asin expression. |
| `computable_transcendentals/asin_cached_p96` | 41.87 ns | 41.80 ns - 41.93 ns | Repeats a cached computable asin approximation. |
| `computable_transcendentals/acos_cold_p96` | 12.558 us | 12.536 us - 12.582 us | Approximates a computable acos expression. |
| `computable_transcendentals/acos_cached_p96` | 42.14 ns | 42.01 ns - 42.28 ns | Repeats a cached computable acos approximation. |
| `computable_transcendentals/atan_cold_p96` | 8.191 us | 8.160 us - 8.226 us | Approximates atan(7/10). |
| `computable_transcendentals/atan_cached_p96` | 42.08 ns | 41.78 ns - 42.44 ns | Repeats a cached atan(7/10) approximation. |
| `computable_transcendentals/atan_large_cold_p96` | 2.718 us | 2.712 us - 2.725 us | Approximates atan(8), exercising argument reduction. |
| `computable_transcendentals/asin_zero_cold_p96` | 45.98 ns | 45.63 ns - 46.26 ns | Approximates asin(0) expression. |
| `computable_transcendentals/atan_zero_cold_p96` | 45.01 ns | 44.77 ns - 45.19 ns | Approximates atan(0). |
| `computable_transcendentals/asinh_cold_p128` | 5.588 us | 5.575 us - 5.600 us | Approximates a computable asinh expression. |
| `computable_transcendentals/asinh_cached_p128` | 42.13 ns | 42.01 ns - 42.26 ns | Repeats a cached computable asinh approximation. |
| `computable_transcendentals/acosh_cold_p128` | 6.535 us | 6.522 us - 6.548 us | Approximates a computable acosh expression. |
| `computable_transcendentals/acosh_cached_p128` | 42.00 ns | 41.96 ns - 42.04 ns | Repeats a cached computable acosh approximation. |
| `computable_transcendentals/atanh_cold_p128` | 5.212 us | 5.200 us - 5.226 us | Approximates a computable atanh expression. |
| `computable_transcendentals/atanh_cached_p128` | 41.92 ns | 41.86 ns - 41.99 ns | Repeats a cached computable atanh approximation. |
| `computable_transcendentals/asinh_zero_cold_p128` | 45.23 ns | 44.70 ns - 45.79 ns | Approximates asinh(0) expression. |
| `computable_transcendentals/atanh_zero_cold_p128` | 45.29 ns | 44.91 ns - 45.61 ns | Approximates atanh(0) expression. |
| `computable_transcendentals/deep_add_chain_cold_p128` | 82.47 ns | 81.59 ns - 83.39 ns | Approximates a 5000-node addition chain. |
| `computable_transcendentals/deep_multiply_chain_cold_p128` | 109.96 ns | 89.45 ns - 150.58 ns | Approximates a 5000-node multiply-by-one chain. |
| `computable_transcendentals/deep_multiply_identity_chain_cold_p128` | 87.87 ns | 87.16 ns - 89.00 ns | Approximates a deep identity multiplication chain around pi. |
| `computable_transcendentals/deep_scaled_product_chain_cold_p128` | 551.72 ns | 550.13 ns - 553.27 ns | Approximates a deep product of exact scale factors. |
| `computable_transcendentals/perturbed_scaled_product_chain_cold_p128` | 915.50 ns | 910.22 ns - 921.00 ns | Approximates a deep scaled product with a tiny perturbation. |
| `computable_transcendentals/scaled_square_chain_cold_p128` | 1.326 us | 1.306 us - 1.353 us | Approximates repeated squaring of a scaled irrational. |
| `computable_transcendentals/asymmetric_product_bad_order_cold_p128` | 950.23 ns | 945.73 ns - 955.89 ns | Approximates an asymmetric product order stress case. |
| `computable_transcendentals/sqrt_scaled_square_chain_cold_p128` | 1.067 us | 1.064 us - 1.071 us | Approximates sqrt of a scaled-square chain. |
| `computable_transcendentals/warmed_zero_product_cold_p128` | 538.52 ns | 498.71 ns - 617.32 ns | Approximates a product involving a warmed zero sum. |
| `computable_transcendentals/inverse_scaled_product_chain_cold_p128` | 1.483 us | 1.479 us - 1.486 us | Approximates the inverse of a deep scaled product. |
| `computable_transcendentals/deep_inverse_pair_chain_cold_p128` | 87.29 ns | 87.14 ns - 87.47 ns | Approximates a chain of inverse(inverse(x)) pairs. |
| `computable_transcendentals/deep_negated_square_chain_cold_p128` | 88.91 ns | 87.45 ns - 90.78 ns | Approximates repeated negate-square-sqrt transformations. |
| `computable_transcendentals/deep_negative_one_product_chain_cold_p128` | 87.01 ns | 86.72 ns - 87.40 ns | Approximates repeated multiplication by -1. |
| `computable_transcendentals/deep_half_product_chain_cold_p128` | 224.38 ns | 223.42 ns - 225.34 ns | Approximates repeated multiplication by 1/2. |
| `computable_transcendentals/deep_half_square_chain_cold_p128` | 940.28 ns | 935.75 ns - 944.74 ns | Approximates repeated squaring after scaling by 1/2. |
| `computable_transcendentals/deep_sqrt_square_chain_cold_p128` | 120.23 ns | 81.38 ns - 197.17 ns | Approximates repeated sqrt-square simplification. |
| `computable_transcendentals/inverse_half_product_chain_cold_p128` | 557.80 ns | 554.40 ns - 561.15 ns | Approximates the inverse of a deep half-product chain. |

<!-- END numerical_micro -->

<!-- BEGIN library_perf -->
## `library_perf`

Library-level Criterion benchmarks for public `Rational`, `Real`, and `Simple` behavior.

### `real_format`

Formatting costs for important irrational `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_format/pi_lower_exp_32` | 4.937 us | 4.920 us - 4.957 us | Formats pi with 32 digits in lower-exponential form. |
| `real_format/pi_display_alt_32` | 5.025 us | 5.012 us - 5.042 us | Formats pi with alternate decimal display at 32 digits. |
| `real_format/sqrt_two_display_alt_32` | 6.183 us | 6.170 us - 6.204 us | Formats sqrt(2) with alternate decimal display at 32 digits. |

### `real_constants`

Construction cost for shared mathematical constants.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_constants/pi` | 54.97 ns | 54.88 ns - 55.08 ns | Constructs the symbolic pi value. |
| `real_constants/e` | 239.98 ns | 239.72 ns - 240.26 ns | Constructs the symbolic Euler constant value. |

### `simple`

Parser and evaluator costs for the `Simple` expression language.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple/parse_nested` | 679.07 ns | 677.57 ns - 680.83 ns | Parses a nested expression with powers, trig, and constants. |
| `simple/eval_nested` | 8.450 us | 8.422 us - 8.482 us | Evaluates a parsed mixed symbolic/numeric expression. |
| `simple/eval_constants` | 3.970 us | 3.947 us - 3.998 us | Evaluates repeated built-in constants. |
| `simple/eval_exact` | 1.574 us | 1.563 us - 1.586 us | Evaluates a rational-only expression through exact shortcuts. |
| `simple/eval_nested_exact` | 4.638 us | 4.629 us - 4.649 us | Evaluates a nested rational-only expression through exact shortcuts. |

### `real_powi`

Integer exponentiation for exact and irrational `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_powi/exact_17` | 1.907 us | 1.900 us - 1.914 us | Raises an exact rational-backed `Real` to the 17th power. |
| `real_powi/irrational_17` | 408.30 ns | 402.96 ns - 414.78 ns | Raises sqrt(3) to the 17th power with symbolic simplification. |

### `rational_powi`

Integer exponentiation for `Rational`.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `rational_powi/exact_17` | 1.873 us | 1.857 us - 1.891 us | Raises a rational value to the 17th power. |

### `real_exact_trig`

Exact and symbolic trig construction for known pi multiples.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_trig/sin_pi_6` | 190.61 ns | 189.87 ns - 191.52 ns | Computes sin(pi/6) via exact shortcut. |
| `real_exact_trig/cos_pi_3` | 492.26 ns | 490.59 ns - 494.19 ns | Computes cos(pi/3) via exact shortcut. |
| `real_exact_trig/tan_pi_5` | 851.08 ns | 848.81 ns - 854.01 ns | Builds tan(pi/5), a nontrivial symbolic tangent. |

### `real_general_trig`

General trig construction for irrational arguments.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_general_trig/tan_sqrt_2` | 900.17 ns | 893.81 ns - 907.41 ns | Builds tan(sqrt(2)). |
| `real_general_trig/tan_pi_sqrt_2_over_5` | 1.699 us | 1.696 us - 1.703 us | Builds tangent of an irrational multiple of pi. |

### `real_exact_inverse_trig`

Exact inverse trig shortcuts and symbolic inverse trig recognition.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_inverse_trig/asin_1_2` | 555.76 ns | 552.70 ns - 559.84 ns | Recognizes asin(1/2) as pi/6. |
| `real_exact_inverse_trig/asin_minus_1_2` | 580.19 ns | 577.98 ns - 582.82 ns | Recognizes asin(-1/2) as -pi/6. |
| `real_exact_inverse_trig/asin_sqrt_2_over_2` | 503.53 ns | 499.96 ns - 507.72 ns | Recognizes asin(sqrt(2)/2) as pi/4. |
| `real_exact_inverse_trig/asin_sin_pi_5` | 434.81 ns | 430.50 ns - 439.36 ns | Inverts a symbolic sin(pi/5). |
| `real_exact_inverse_trig/acos_1` | 85.66 ns | 85.32 ns - 86.18 ns | Recognizes acos(1) as zero. |
| `real_exact_inverse_trig/acos_minus_1` | 121.21 ns | 120.42 ns - 122.08 ns | Recognizes acos(-1) as pi. |
| `real_exact_inverse_trig/acos_1_2` | 1.328 us | 1.324 us - 1.333 us | Recognizes acos(1/2) as pi/3. |
| `real_exact_inverse_trig/atan_1` | 366.92 ns | 365.83 ns - 368.28 ns | Recognizes atan(1) as pi/4. |
| `real_exact_inverse_trig/atan_sqrt_3_over_3` | 796.98 ns | 791.44 ns - 803.93 ns | Recognizes atan(sqrt(3)/3) as pi/6. |
| `real_exact_inverse_trig/atan_tan_pi_5` | 425.92 ns | 424.14 ns - 427.66 ns | Inverts a symbolic tan(pi/5). |

### `real_general_inverse_trig`

General inverse trig construction, domain errors, and atan range reduction.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_general_inverse_trig/asin_7_10` | 11.333 us | 10.832 us - 12.258 us | Builds asin(7/10) through the rational-specialized path. |
| `real_general_inverse_trig/asin_sqrt_2_over_3` | 6.949 us | 6.934 us - 6.968 us | Builds asin(sqrt(2)/3) through the general path. |
| `real_general_inverse_trig/acos_7_10` | 13.697 us | 13.631 us - 13.774 us | Builds acos(7/10) through the rational-specialized asin path. |
| `real_general_inverse_trig/acos_sqrt_2_over_3` | 8.657 us | 8.629 us - 8.689 us | Builds acos(sqrt(2)/3) through the general path. |
| `real_general_inverse_trig/asin_11_10_error` | 535.42 ns | 533.49 ns - 537.57 ns | Rejects rational asin input outside [-1, 1]. |
| `real_general_inverse_trig/acos_11_10_error` | 1.502 us | 1.496 us - 1.509 us | Rejects rational acos input outside [-1, 1]. |
| `real_general_inverse_trig/atan_8` | 563.00 ns | 561.39 ns - 564.80 ns | Builds atan(8), exercising large-argument reduction. |
| `real_general_inverse_trig/atan_sqrt_2` | 7.509 us | 7.321 us - 7.843 us | Builds atan(sqrt(2)). |

### `real_inverse_hyperbolic`

Inverse hyperbolic construction, exact exits, stable ln1p forms, and domain errors.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_inverse_hyperbolic/asinh_0` | 68.43 ns | 68.26 ns - 68.62 ns | Recognizes asinh(0) as zero. |
| `real_inverse_hyperbolic/asinh_1_2` | 1.995 us | 1.987 us - 2.005 us | Builds asinh(1/2) through the stable moderate-input path. |
| `real_inverse_hyperbolic/asinh_sqrt_2` | 2.089 us | 2.083 us - 2.095 us | Builds asinh(sqrt(2)) without cancellation-prone log construction. |
| `real_inverse_hyperbolic/asinh_minus_1_2` | 2.003 us | 1.996 us - 2.010 us | Uses odd symmetry for negative asinh input. |
| `real_inverse_hyperbolic/asinh_1_000_000` | 4.680 us | 4.589 us - 4.797 us | Builds asinh for a large positive rational. |
| `real_inverse_hyperbolic/acosh_1` | 164.65 ns | 92.43 ns - 308.78 ns | Recognizes acosh(1) as zero. |
| `real_inverse_hyperbolic/acosh_2` | 1.917 us | 1.911 us - 1.924 us | Builds acosh(2) through the stable moderate-input path. |
| `real_inverse_hyperbolic/acosh_sqrt_2` | 2.721 us | 2.710 us - 2.733 us | Builds acosh(sqrt(2)) through square-root domain specialization. |
| `real_inverse_hyperbolic/acosh_1_000_000` | 5.882 us | 5.857 us - 5.913 us | Builds acosh for a large positive rational. |
| `real_inverse_hyperbolic/atanh_0` | 97.63 ns | 68.23 ns - 156.32 ns | Recognizes atanh(0) as zero. |
| `real_inverse_hyperbolic/atanh_1_2` | 1.697 us | 1.692 us - 1.702 us | Builds exact-rational atanh(1/2). |
| `real_inverse_hyperbolic/atanh_minus_1_2` | 1.786 us | 1.774 us - 1.799 us | Builds exact-rational atanh(-1/2). |
| `real_inverse_hyperbolic/atanh_9_10` | 3.102 us | 3.078 us - 3.129 us | Builds exact-rational atanh near the upper domain boundary. |
| `real_inverse_hyperbolic/atanh_1_error` | 165.30 ns | 77.05 ns - 341.10 ns | Rejects atanh(1) at the rational domain boundary. |

### `simple_inverse_functions`

Parsed/evaluated inverse trig and inverse hyperbolic expressions that should succeed.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple_inverse_functions/asin_1_2` | 597.15 ns | 594.18 ns - 600.44 ns | Evaluates `(asin 1/2)`. |
| `simple_inverse_functions/acos_1_2` | 1.578 us | 1.573 us - 1.584 us | Evaluates `(acos 1/2)`. |
| `simple_inverse_functions/atan_1` | 396.23 ns | 395.20 ns - 397.37 ns | Evaluates `(atan 1)`. |
| `simple_inverse_functions/asin_general` | 11.131 us | 10.903 us - 11.545 us | Evaluates `(asin 7/10)`. |
| `simple_inverse_functions/acos_general` | 15.352 us | 13.931 us - 18.138 us | Evaluates `(acos 7/10)`. |
| `simple_inverse_functions/atan_general` | 641.77 ns | 635.56 ns - 649.17 ns | Evaluates `(atan 8)`. |
| `simple_inverse_functions/asinh_1_2` | 2.033 us | 2.026 us - 2.041 us | Evaluates `(asinh 1/2)`. |
| `simple_inverse_functions/asinh_sqrt_2` | 2.842 us | 2.833 us - 2.853 us | Evaluates `(asinh (sqrt 2))`. |
| `simple_inverse_functions/acosh_2` | 1.994 us | 1.987 us - 2.001 us | Evaluates `(acosh 2)`. |
| `simple_inverse_functions/acosh_sqrt_2` | 3.498 us | 3.482 us - 3.516 us | Evaluates `(acosh (sqrt 2))`. |
| `simple_inverse_functions/atanh_1_2` | 1.784 us | 1.781 us - 1.787 us | Evaluates `(atanh 1/2)`. |
| `simple_inverse_functions/atanh_minus_1_2` | 1.880 us | 1.843 us - 1.929 us | Evaluates `(atanh -1/2)`. |

### `simple_inverse_error_functions`

Parsed/evaluated inverse function expressions that should fail quickly with `NotANumber`.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `simple_inverse_error_functions/asin_11_10` | 627.21 ns | 625.23 ns - 629.28 ns | Rejects `(asin 11/10)`. |
| `simple_inverse_error_functions/acos_sqrt_2` | 1.918 us | 1.905 us - 1.937 us | Rejects `(acos (sqrt 2))`. |
| `simple_inverse_error_functions/acosh_0` | 85.22 ns | 73.62 ns - 108.21 ns | Rejects `(acosh 0)`. |
| `simple_inverse_error_functions/acosh_minus_2` | 110.81 ns | 94.46 ns - 142.97 ns | Rejects `(acosh -2)`. |
| `simple_inverse_error_functions/atanh_1` | 132.99 ns | 115.37 ns - 167.64 ns | Rejects `(atanh 1)`. |
| `simple_inverse_error_functions/atanh_sqrt_2` | 933.12 ns | 930.37 ns - 935.89 ns | Rejects `(atanh (sqrt 2))`. |

### `real_exact_ln`

Exact logarithm construction and simplification for rational inputs.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_ln/ln_1024` | 423.99 ns | 423.22 ns - 424.83 ns | Recognizes ln(1024) as 10 ln(2). |
| `real_exact_ln/ln_1_8` | 434.34 ns | 433.55 ns - 435.23 ns | Recognizes ln(1/8) as -3 ln(2). |
| `real_exact_ln/ln_1000` | 967.83 ns | 965.92 ns - 969.91 ns | Simplifies ln(1000) via small integer logarithm factors. |

### `real_exact_exp_log10`

Exact inverse relationships among exp, ln, and log10.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_exact_exp_log10/exp_ln_1000` | 322.90 ns | 322.01 ns - 324.13 ns | Simplifies exp(ln(1000)) back to 1000. |
| `real_exact_exp_log10/exp_ln_1_8` | 529.25 ns | 527.14 ns - 531.66 ns | Simplifies exp(ln(1/8)) back to 1/8. |
| `real_exact_exp_log10/log10_1000` | 1.643 us | 1.637 us - 1.649 us | Recognizes log10(1000) as 3. |
| `real_exact_exp_log10/log10_1_1000` | 1.723 us | 1.715 us - 1.731 us | Recognizes log10(1/1000) as -3. |

<!-- END library_perf -->
