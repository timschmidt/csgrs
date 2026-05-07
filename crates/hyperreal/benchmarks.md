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

### `construction_speed`

Cost of constructing common exact scalar identities.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `construction_speed/rational_one` | 15.69 ns | 15.53 ns - 15.92 ns | Constructs `Rational::one()`. |
| `construction_speed/rational_new_one` | 19.62 ns | 19.40 ns - 19.88 ns | Constructs one through `Rational::new(1)`. |
| `construction_speed/computable_one` | 26.12 ns | 25.99 ns - 26.26 ns | Constructs `Computable::one()`. |
| `construction_speed/real_new_rational_one` | 103.53 ns | 102.48 ns - 104.67 ns | Constructs one through `Real::new(Rational::one())`. |
| `construction_speed/real_one` | 101.81 ns | 100.87 ns - 102.93 ns | Constructs one through `Real::one()`. |
| `construction_speed/real_from_i32_one` | 102.71 ns | 102.12 ns - 103.41 ns | Constructs one through integer conversion. |

### `raw_cache_hit_cost`

Cost of cold and cached `Computable::approx` calls for simple values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `raw_cache_hit_cost/zero` | 50.90 ns | 50.15 ns - 51.71 ns | Cached approximation request for exact zero. |
| `raw_cache_hit_cost/one` | 66.50 ns | 66.14 ns - 67.03 ns | Cached approximation request for exact one. |
| `raw_cache_hit_cost/two` | 67.10 ns | 66.41 ns - 67.88 ns | Cached approximation request for exact two. |
| `raw_cache_hit_cost/e` | 70.77 ns | 70.36 ns - 71.14 ns | Cached approximation request for Euler's constant. |
| `raw_cache_hit_cost/pi` | 70.53 ns | 70.26 ns - 70.80 ns | Cached approximation request for pi. |
| `raw_cache_hit_cost/tau` | 70.51 ns | 69.95 ns - 71.39 ns | Cached approximation request for two pi. |

### `structural_query_speed`

Speed of public structural queries across exact, transcendental, and composite `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `structural_query_speed/zero_zero_status` | 0.73 ns | 0.72 ns - 0.75 ns | Checks zero/nonzero facts for exact zero. |
| `structural_query_speed/zero_sign_query` | 4.41 ns | 4.35 ns - 4.49 ns | Reads sign facts for exact zero. |
| `structural_query_speed/zero_msd_query` | 6.43 ns | 6.29 ns - 6.66 ns | Reads magnitude facts for exact zero. |
| `structural_query_speed/zero_structural_facts` | 7.45 ns | 7.03 ns - 8.20 ns | Computes full structural facts for exact zero. |
| `structural_query_speed/one_zero_status` | 0.82 ns | 0.79 ns - 0.86 ns | Checks zero/nonzero facts for exact one. |
| `structural_query_speed/one_sign_query` | 24.11 ns | 23.31 ns - 25.05 ns | Reads sign facts for exact one. |
| `structural_query_speed/one_msd_query` | 24.35 ns | 24.16 ns - 24.55 ns | Reads magnitude facts for exact one. |
| `structural_query_speed/one_structural_facts` | 24.40 ns | 24.23 ns - 24.58 ns | Computes full structural facts for exact one. |
| `structural_query_speed/negative_zero_status` | 0.81 ns | 0.78 ns - 0.83 ns | Checks zero/nonzero facts for an exact negative integer. |
| `structural_query_speed/negative_sign_query` | 22.87 ns | 22.70 ns - 23.06 ns | Reads sign facts for an exact negative integer. |
| `structural_query_speed/negative_msd_query` | 27.33 ns | 26.33 ns - 28.45 ns | Reads magnitude facts for an exact negative integer. |
| `structural_query_speed/negative_structural_facts` | 27.45 ns | 27.22 ns - 27.74 ns | Computes full structural facts for an exact negative integer. |
| `structural_query_speed/tiny_exact_zero_status` | 0.82 ns | 0.80 ns - 0.85 ns | Checks zero/nonzero facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_sign_query` | 25.89 ns | 25.44 ns - 26.42 ns | Reads sign facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_msd_query` | 28.91 ns | 28.75 ns - 29.10 ns | Reads magnitude facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_structural_facts` | 33.08 ns | 32.37 ns - 34.04 ns | Computes full structural facts for a tiny exact rational. |
| `structural_query_speed/pi_zero_status` | 0.81 ns | 0.80 ns - 0.81 ns | Checks zero/nonzero facts for pi. |
| `structural_query_speed/pi_sign_query` | 35.75 ns | 34.21 ns - 37.53 ns | Reads sign facts for pi. |
| `structural_query_speed/pi_msd_query` | 39.77 ns | 39.49 ns - 40.08 ns | Reads magnitude facts for pi. |
| `structural_query_speed/pi_structural_facts` | 39.36 ns | 39.18 ns - 39.55 ns | Computes full structural facts for pi. |
| `structural_query_speed/e_zero_status` | 0.84 ns | 0.83 ns - 0.86 ns | Checks zero/nonzero facts for e. |
| `structural_query_speed/e_sign_query` | 34.10 ns | 33.80 ns - 34.45 ns | Reads sign facts for e. |
| `structural_query_speed/e_msd_query` | 38.99 ns | 38.78 ns - 39.21 ns | Reads magnitude facts for e. |
| `structural_query_speed/e_structural_facts` | 38.95 ns | 38.24 ns - 39.88 ns | Computes full structural facts for e. |
| `structural_query_speed/tau_zero_status` | 0.80 ns | 0.80 ns - 0.81 ns | Checks zero/nonzero facts for tau. |
| `structural_query_speed/tau_sign_query` | 33.45 ns | 32.85 ns - 34.39 ns | Reads sign facts for tau. |
| `structural_query_speed/tau_msd_query` | 39.05 ns | 38.34 ns - 39.92 ns | Reads magnitude facts for tau. |
| `structural_query_speed/tau_structural_facts` | 40.99 ns | 39.78 ns - 42.33 ns | Computes full structural facts for tau. |
| `structural_query_speed/sqrt_two_zero_status` | 0.78 ns | 0.78 ns - 0.79 ns | Checks zero/nonzero facts for sqrt(2). |
| `structural_query_speed/sqrt_two_sign_query` | 34.06 ns | 32.94 ns - 35.53 ns | Reads sign facts for sqrt(2). |
| `structural_query_speed/sqrt_two_msd_query` | 38.97 ns | 38.63 ns - 39.34 ns | Reads magnitude facts for sqrt(2). |
| `structural_query_speed/sqrt_two_structural_facts` | 36.74 ns | 36.37 ns - 37.14 ns | Computes full structural facts for sqrt(2). |
| `structural_query_speed/pi_minus_three_zero_status` | 0.92 ns | 0.91 ns - 0.94 ns | Checks zero/nonzero facts for pi - 3. |
| `structural_query_speed/pi_minus_three_sign_query` | 35.19 ns | 34.59 ns - 35.84 ns | Reads sign facts for pi - 3. |
| `structural_query_speed/pi_minus_three_msd_query` | 40.32 ns | 39.58 ns - 41.31 ns | Reads magnitude facts for pi - 3. |
| `structural_query_speed/pi_minus_three_structural_facts` | 40.74 ns | 40.19 ns - 41.41 ns | Computes full structural facts for pi - 3. |
| `structural_query_speed/dense_expr_zero_status` | 3.00 ns | 2.93 ns - 3.08 ns | Checks zero/nonzero facts for a dense composite expression. |
| `structural_query_speed/dense_expr_sign_query` | 34.56 ns | 34.16 ns - 34.99 ns | Reads sign facts for a dense composite expression. |
| `structural_query_speed/dense_expr_msd_query` | 38.49 ns | 38.12 ns - 39.08 ns | Reads magnitude facts for a dense composite expression. |
| `structural_query_speed/dense_expr_structural_facts` | 37.87 ns | 37.73 ns - 38.00 ns | Computes full structural facts for a dense composite expression. |

### `pure_scalar_algorithm_speed`

Core scalar algorithms that do not require high-precision transcendental approximation.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `pure_scalar_algorithm_speed/rational_add` | 404.28 ns | 398.48 ns - 410.75 ns | Adds two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_mul` | 117.67 ns | 117.17 ns - 118.21 ns | Multiplies two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_div` | 595.32 ns | 591.83 ns - 599.30 ns | Divides two nontrivial rational values. |
| `pure_scalar_algorithm_speed/real_exact_add` | 440.01 ns | 436.11 ns - 444.23 ns | Adds exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_mul` | 186.72 ns | 184.16 ns - 189.36 ns | Multiplies exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_div` | 672.76 ns | 668.24 ns - 677.34 ns | Divides exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_sqrt_reduce` | 746.55 ns | 706.79 ns - 796.26 ns | Reduces an exact square-root expression. |
| `pure_scalar_algorithm_speed/real_exact_ln_reduce` | 441.10 ns | 431.24 ns - 453.97 ns | Reduces an exact logarithm of a power of two. |

### `borrowed_op_overhead`

Borrowed versus owned operation overhead for rational and real operands.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `borrowed_op_overhead/rational_clone_pair` | 46.22 ns | 43.25 ns - 51.11 ns | Clones two rational values. |
| `borrowed_op_overhead/rational_add_refs` | 448.17 ns | 384.22 ns - 523.09 ns | Adds rational references. |
| `borrowed_op_overhead/rational_add_owned` | 1.006 us | 835.18 ns - 1.193 us | Adds owned rational values. |
| `borrowed_op_overhead/real_clone_pair` | 438.26 ns | 403.93 ns - 488.24 ns | Clones two scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_unscaled_add_refs` | 174.89 ns | 173.07 ns - 177.22 ns | Adds borrowed unscaled transcendental `Real` values. |
| `borrowed_op_overhead/real_unscaled_add_owned` | 222.18 ns | 219.08 ns - 225.57 ns | Adds owned unscaled transcendental `Real` values. |
| `borrowed_op_overhead/real_add_refs` | 889.12 ns | 867.87 ns - 912.13 ns | Adds borrowed scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_add_owned` | 826.04 ns | 812.20 ns - 842.52 ns | Adds owned scaled transcendental `Real` values. |

### `dense_algebra`

Small dense algebra kernels that stress repeated exact and symbolic operations.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `dense_algebra/rational_dot_64` | 37.427 us | 36.065 us - 38.907 us | Computes a 64-element rational dot product. |
| `dense_algebra/rational_matmul_8` | 237.989 us | 229.925 us - 249.364 us | Computes an 8x8 rational matrix multiply. |
| `dense_algebra/real_dot_36` | 28.885 us | 27.840 us - 30.195 us | Computes a 36-element dot product over symbolic `Real` values. |
| `dense_algebra/real_matmul_6` | 157.907 us | 153.698 us - 163.094 us | Computes a 6x6 matrix multiply over symbolic `Real` values. |

### `exact_transcendental_special_forms`

Construction-time shortcuts for exact rational multiples of pi and inverse compositions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `exact_transcendental_special_forms/sin_pi_7` | 581.05 ns | 570.93 ns - 592.04 ns | Builds the exact special form for sin(pi/7). |
| `exact_transcendental_special_forms/cos_pi_7` | 1.016 us | 1.013 us - 1.019 us | Builds the exact special form for cos(pi/7). |
| `exact_transcendental_special_forms/tan_pi_7` | 503.05 ns | 498.53 ns - 507.23 ns | Builds the exact special form for tan(pi/7). |
| `exact_transcendental_special_forms/asin_sin_6pi_7` | 1.097 us | 1.086 us - 1.109 us | Recognizes the principal branch of asin(sin(6pi/7)). |
| `exact_transcendental_special_forms/acos_cos_9pi_7` | 2.470 us | 2.369 us - 2.620 us | Recognizes the principal branch of acos(cos(9pi/7)). |
| `exact_transcendental_special_forms/atan_tan_6pi_7` | 1.095 us | 1.090 us - 1.101 us | Recognizes the principal branch of atan(tan(6pi/7)). |
| `exact_transcendental_special_forms/asinh_large` | 3.638 us | 3.615 us - 3.657 us | Builds a large inverse hyperbolic sine without exact intermediate Reals. |
| `exact_transcendental_special_forms/atanh_sqrt_half` | 6.880 us | 6.709 us - 7.079 us | Builds atanh(sqrt(2)/2) after exact structural domain checks. |

### `symbolic_reductions`

Existing symbolic constant algebra cases considered for additional reductions.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `symbolic_reductions/sqrt_pi_square` | 143.98 ns | 139.06 ns - 149.97 ns | Reduces sqrt(pi^2). |
| `symbolic_reductions/sqrt_pi_e_square` | 181.06 ns | 178.38 ns - 184.62 ns | Reduces sqrt((pi * e)^2). |
| `symbolic_reductions/ln_scaled_e` | 1.892 us | 1.847 us - 1.956 us | Reduces ln(2 * e). |
| `symbolic_reductions/sub_pi_three` | 1.751 us | 1.729 us - 1.774 us | Builds the certified pi - 3 constant-offset form. |
| `symbolic_reductions/pi_minus_three_facts` | 37.89 ns | 37.36 ns - 38.48 ns | Reads structural facts for the cached pi - 3 offset form. |
| `symbolic_reductions/div_exp_exp` | 3.730 us | 3.662 us - 3.822 us | Reduces e^3 / e. |
| `symbolic_reductions/div_pi_square_e` | 3.277 us | 3.246 us - 3.310 us | Reduces pi^2 / e. |
| `symbolic_reductions/div_const_products` | 4.542 us | 4.407 us - 4.703 us | Reduces (pi^3 * e^5) / (pi * e^2). |
| `symbolic_reductions/inverse_pi` | 96.94 ns | 95.48 ns - 99.13 ns | Builds the reciprocal of pi. |
| `symbolic_reductions/div_one_pi` | 286.93 ns | 280.01 ns - 294.58 ns | Reduces 1 / pi. |
| `symbolic_reductions/div_e_pi` | 343.53 ns | 341.01 ns - 346.18 ns | Reduces e / pi. |
| `symbolic_reductions/mul_pi_inverse_pi` | 270.73 ns | 264.59 ns - 276.42 ns | Multiplies pi by its reciprocal. |
| `symbolic_reductions/mul_pi_e_sqrt_two` | 996.75 ns | 971.54 ns - 1.028 us | Builds the factored pi * e * sqrt(2) form. |
| `symbolic_reductions/mul_const_product_sqrt_sqrt` | 809.61 ns | 742.89 ns - 908.87 ns | Cancels sqrt(2) from (pi * e * sqrt(2)) * sqrt(2). |
| `symbolic_reductions/div_const_product_sqrt_e` | 1.115 us | 1.098 us - 1.134 us | Reduces (pi * e * sqrt(2)) / e. |
| `symbolic_reductions/inverse_const_product_sqrt` | 3.796 us | 3.751 us - 3.850 us | Builds a rationalized reciprocal of pi * e * sqrt(2). |

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
| `computable_bounds/deep_structural_bound_sign` | 28.712 us | 28.102 us - 29.439 us | Finds sign through repeated multiply/inverse/negate structural transformations. |
| `computable_bounds/deep_structural_bound_sign_cached` | 3.86 ns | 3.83 ns - 3.88 ns | Reads the cached sign of the deep structural-bound chain. |
| `computable_bounds/deep_structural_bound_facts_cached` | 14.54 ns | 14.29 ns - 14.84 ns | Reads cached structural facts for the deep structural-bound chain. |
| `computable_bounds/perturbed_scaled_product_sign` | not run | not run | Finds sign for a deeply scaled value with a tiny perturbation. |
| `computable_bounds/perturbed_scaled_product_sign_until` | not run | not run | Refines sign for the perturbed scaled product only to p=-128. |
| `computable_bounds/pi_minus_tiny_sign` | 73.68 ns | 71.33 ns - 75.42 ns | Finds sign for pi minus a tiny exact rational. |
| `computable_bounds/pi_minus_tiny_sign_cached` | 3.83 ns | 3.82 ns - 3.85 ns | Reads cached sign for pi minus a tiny exact rational. |

### `computable_compare`

Ordering and absolute-comparison shortcuts.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_compare/compare_to_opposite_sign` | not run | not run | Compares values with known opposite signs. |
| `computable_compare/compare_to_exact_msd_gap` | not run | not run | Compares values with a large exact magnitude gap. |
| `computable_compare/compare_absolute_exact_rational` | not run | not run | Compares absolute values of exact rationals. |
| `computable_compare/compare_absolute_dominant_add` | not run | not run | Compares a dominant term against the same term plus a tiny addend. |
| `computable_compare/compare_absolute_exact_msd_gap` | not run | not run | Compares absolute values with a large exact magnitude gap. |

### `computable_transcendentals`

Low-level approximation kernels and deep expression-tree stress cases.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_transcendentals/legacy_exp_one_p128` | not run | not run | Runs the legacy direct exp series for input 1 at p=-128. |
| `computable_transcendentals/e_constant_cold_p128` | not run | not run | Approximates the shared e constant from a fresh clone. |
| `computable_transcendentals/e_constant_cached_p128` | not run | not run | Repeats a cached approximation of e. |
| `computable_transcendentals/legacy_exp_half_p128` | not run | not run | Runs the legacy direct exp series for input 1/2 at p=-128. |
| `computable_transcendentals/exp_cold_p128` | not run | not run | Approximates exp(7/5) from a fresh clone. |
| `computable_transcendentals/exp_cached_p128` | not run | not run | Repeats a cached exp(7/5) approximation. |
| `computable_transcendentals/exp_large_cold_p128` | not run | not run | Approximates exp(128), exercising large-argument reduction. |
| `computable_transcendentals/exp_half_cold_p128` | not run | not run | Approximates exp(1/2). |
| `computable_transcendentals/exp_near_limit_cold_p128` | not run | not run | Approximates exp near a prescaling threshold. |
| `computable_transcendentals/exp_near_limit_cached_p128` | not run | not run | Repeats a cached near-threshold exp approximation. |
| `computable_transcendentals/exp_zero_cold_p128` | not run | not run | Approximates exp(0). |
| `computable_transcendentals/ln_cold_p128` | not run | not run | Approximates ln(11/7). |
| `computable_transcendentals/ln_cached_p128` | not run | not run | Repeats a cached ln(11/7) approximation. |
| `computable_transcendentals/ln_large_cold_p128` | not run | not run | Approximates ln(1024), exercising large-input reduction. |
| `computable_transcendentals/ln_large_cached_p128` | not run | not run | Repeats a cached ln(1024) approximation. |
| `computable_transcendentals/ln_tiny_cold_p128` | not run | not run | Approximates ln(2^-1024), exercising tiny-input reduction. |
| `computable_transcendentals/ln_near_limit_cold_p128` | not run | not run | Approximates ln near the prescaled-ln limit. |
| `computable_transcendentals/ln_near_limit_cached_p128` | not run | not run | Repeats a cached near-limit ln approximation. |
| `computable_transcendentals/ln_one_cold_p128` | not run | not run | Approximates ln(1). |
| `computable_transcendentals/sqrt_cold_p128` | not run | not run | Approximates sqrt(2). |
| `computable_transcendentals/sqrt_cached_p128` | not run | not run | Repeats a cached sqrt(2) approximation. |
| `computable_transcendentals/sqrt_single_scaled_square_cold_p128` | not run | not run | Builds and approximates sqrt((7*pi/8)^2). |
| `computable_transcendentals/sin_cold_p96` | 79.13 ns | 77.72 ns - 80.44 ns | Approximates sin(7/5). |
| `computable_transcendentals/sin_cached_p96` | not run | not run | Repeats a cached sin(7/5) approximation. |
| `computable_transcendentals/cos_cold_p96` | 82.63 ns | 81.12 ns - 84.03 ns | Approximates cos(7/5). |
| `computable_transcendentals/sin_f64_cold_p96` | 81.98 ns | 80.41 ns - 83.44 ns | Approximates sin(1.23456789 imported exactly from f64). |
| `computable_transcendentals/cos_f64_cold_p96` | 83.71 ns | 82.26 ns - 85.07 ns | Approximates cos(1.23456789 imported exactly from f64). |
| `computable_transcendentals/sin_1e6_cold_p96` | 2.392 us | 2.375 us - 2.410 us | Approximates sin(1000000). |
| `computable_transcendentals/cos_1e6_cold_p96` | 2.765 us | 2.268 us - 3.745 us | Approximates cos(1000000). |
| `computable_transcendentals/sin_1e30_cold_p96` | 2.006 us | 1.996 us - 2.018 us | Approximates sin(10^30). |
| `computable_transcendentals/cos_1e30_cold_p96` | 2.078 us | 2.066 us - 2.090 us | Approximates cos(10^30). |
| `computable_transcendentals/cos_cached_p96` | not run | not run | Repeats a cached cos(7/5) approximation. |
| `computable_transcendentals/tan_cold_p96` | 3.439 us | 3.420 us - 3.463 us | Approximates tan(7/5). |
| `computable_transcendentals/tan_cached_p96` | not run | not run | Repeats a cached tan(7/5) approximation. |
| `computable_transcendentals/sin_zero_cold_p96` | 46.06 ns | 45.67 ns - 46.48 ns | Approximates sin(0). |
| `computable_transcendentals/cos_zero_cold_p96` | 70.24 ns | 69.59 ns - 70.93 ns | Approximates cos(0). |
| `computable_transcendentals/tan_zero_cold_p96` | 46.90 ns | 46.43 ns - 47.42 ns | Approximates tan(0). |
| `computable_transcendentals/tan_near_half_pi_cold_p96` | 3.315 us | 3.287 us - 3.352 us | Approximates tangent near pi/2. |
| `computable_transcendentals/tan_near_half_pi_cached_p96` | not run | not run | Repeats cached tangent near pi/2. |
| `computable_transcendentals/sin_huge_cold_p96` | 2.970 us | 2.602 us - 3.695 us | Approximates sine of a huge pi multiple plus offset. |
| `computable_transcendentals/cos_huge_cold_p96` | 2.709 us | 2.691 us - 2.729 us | Approximates cosine of a huge pi multiple plus offset. |
| `computable_transcendentals/tan_huge_cold_p96` | 4.324 us | 4.290 us - 4.361 us | Approximates tangent of a huge pi multiple plus offset. |
| `computable_transcendentals/asin_cold_p96` | 6.664 us | 6.504 us - 6.946 us | Approximates a computable asin expression. |
| `computable_transcendentals/asin_cached_p96` | 42.84 ns | 42.30 ns - 43.47 ns | Repeats a cached computable asin approximation. |
| `computable_transcendentals/acos_cold_p96` | 6.029 us | 5.730 us - 6.604 us | Approximates a computable acos expression. |
| `computable_transcendentals/acos_cached_p96` | 43.32 ns | 42.89 ns - 43.81 ns | Repeats a cached computable acos approximation. |
| `computable_transcendentals/asin_tiny_cold_p96` | 410.32 ns | 408.75 ns - 412.05 ns | Approximates asin(1e-12), exercising the tiny-input series. |
| `computable_transcendentals/acos_tiny_cold_p96` | 805.93 ns | 800.01 ns - 812.41 ns | Approximates acos(1e-12), exercising the tiny-input complement. |
| `computable_transcendentals/asin_near_one_cold_p96` | 2.037 us | 2.028 us - 2.047 us | Approximates asin(0.999999), exercising the endpoint complement. |
| `computable_transcendentals/acos_near_one_cold_p96` | 1.761 us | 1.744 us - 1.782 us | Approximates acos(0.999999), exercising the endpoint transform. |
| `computable_transcendentals/atan_cold_p96` | 8.451 us | 8.390 us - 8.517 us | Approximates atan(7/10). |
| `computable_transcendentals/atan_cached_p96` | 41.82 ns | 41.10 ns - 42.66 ns | Repeats a cached atan(7/10) approximation. |
| `computable_transcendentals/atan_large_cold_p96` | 2.699 us | 2.601 us - 2.883 us | Approximates atan(8), exercising argument reduction. |
| `computable_transcendentals/asin_zero_cold_p96` | 45.54 ns | 45.23 ns - 45.90 ns | Approximates asin(0) expression. |
| `computable_transcendentals/atan_zero_cold_p96` | 45.63 ns | 45.27 ns - 46.00 ns | Approximates atan(0). |
| `computable_transcendentals/asinh_cold_p128` | 6.448 us | 6.407 us - 6.494 us | Approximates a computable asinh expression. |
| `computable_transcendentals/asinh_cached_p128` | 41.52 ns | 41.40 ns - 41.65 ns | Repeats a cached computable asinh approximation. |
| `computable_transcendentals/acosh_cold_p128` | 10.324 us | 10.253 us - 10.402 us | Approximates a computable acosh expression. |
| `computable_transcendentals/acosh_cached_p128` | 41.48 ns | 41.22 ns - 41.77 ns | Repeats a cached computable acosh approximation. |
| `computable_transcendentals/atanh_cold_p128` | 182.95 ns | 181.32 ns - 184.72 ns | Approximates a computable atanh expression. |
| `computable_transcendentals/atanh_cached_p128` | 42.16 ns | 41.84 ns - 42.52 ns | Repeats a cached computable atanh approximation. |
| `computable_transcendentals/atanh_tiny_cold_p128` | 512.02 ns | 510.61 ns - 513.51 ns | Approximates atanh(1e-12), exercising the tiny-input series. |
| `computable_transcendentals/atanh_near_one_cold_p128` | 2.937 us | 2.916 us - 2.961 us | Approximates atanh(0.999999), exercising the endpoint log transform. |
| `computable_transcendentals/asinh_zero_cold_p128` | 47.37 ns | 46.88 ns - 48.03 ns | Approximates asinh(0) expression. |
| `computable_transcendentals/atanh_zero_cold_p128` | 45.85 ns | 45.53 ns - 46.21 ns | Approximates atanh(0) expression. |
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

<!-- BEGIN adversarial_transcendentals -->
## `adversarial_transcendentals`

Adversarial transcendental benchmarks for `hyperreal` trig, inverse trig, and inverse hyperbolic construction and approximation paths.

### `trig_adversarial_approx`

Cold approximation of sine, cosine, and tangent at exact, tiny, huge, and near-singular arguments.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `trig_adversarial_approx/sin_tiny_rational_p96` | 541.38 ns | 530.51 ns - 554.58 ns | Approximates sin(1e-12), stressing direct tiny-argument setup. |
| `trig_adversarial_approx/cos_tiny_rational_p96` | 551.40 ns | 545.38 ns - 560.73 ns | Approximates cos(1e-12), stressing direct tiny-argument setup. |
| `trig_adversarial_approx/tan_tiny_rational_p96` | 1.074 us | 1.055 us - 1.098 us | Approximates tan(1e-12), stressing direct tiny-argument setup. |
| `trig_adversarial_approx/sin_medium_rational_p96` | 4.073 us | 4.006 us - 4.159 us | Approximates sin(7/5), a moderate non-pi rational. |
| `trig_adversarial_approx/cos_medium_rational_p96` | 4.451 us | 4.417 us - 4.487 us | Approximates cos(7/5), a moderate non-pi rational. |
| `trig_adversarial_approx/tan_medium_rational_p96` | 6.134 us | 6.059 us - 6.227 us | Approximates tan(7/5), a moderate non-pi rational. |
| `trig_adversarial_approx/sin_f64_exact_p96` | 4.315 us | 4.230 us - 4.424 us | Approximates sin(1.23456789 imported as an exact dyadic rational). |
| `trig_adversarial_approx/cos_f64_exact_p96` | 4.685 us | 4.666 us - 4.702 us | Approximates cos(1.23456789 imported as an exact dyadic rational). |
| `trig_adversarial_approx/sin_1e6_p96` | 9.320 us | 9.279 us - 9.363 us | Approximates sin(1000000), stressing integer argument reduction. |
| `trig_adversarial_approx/cos_1e6_p96` | 9.388 us | 9.299 us - 9.489 us | Approximates cos(1000000), stressing integer argument reduction. |
| `trig_adversarial_approx/tan_1e6_p96` | 11.000 us | 10.928 us - 11.089 us | Approximates tan(1000000), stressing integer argument reduction. |
| `trig_adversarial_approx/sin_1e30_p96` | 11.307 us | 11.267 us - 11.339 us | Approximates sin(10^30), stressing very large integer reduction. |
| `trig_adversarial_approx/cos_1e30_p96` | 11.801 us | 11.599 us - 12.027 us | Approximates cos(10^30), stressing very large integer reduction. |
| `trig_adversarial_approx/tan_1e30_p96` | 12.727 us | 12.330 us - 13.178 us | Approximates tan(10^30), stressing very large integer reduction. |
| `trig_adversarial_approx/sin_huge_pi_plus_offset_p96` | 7.536 us | 7.349 us - 7.760 us | Approximates sin(2^512*pi + 7/5), stressing exact pi-multiple cancellation. |
| `trig_adversarial_approx/cos_huge_pi_plus_offset_p96` | 7.428 us | 7.347 us - 7.543 us | Approximates cos(2^512*pi + 7/5), stressing exact pi-multiple cancellation. |
| `trig_adversarial_approx/tan_huge_pi_plus_offset_p96` | 9.200 us | 8.953 us - 9.499 us | Approximates tan(2^512*pi + 7/5), stressing exact pi-multiple cancellation. |
| `trig_adversarial_approx/tan_near_half_pi_p96` | 2.200 us | 2.153 us - 2.256 us | Approximates tan(pi/2 - 2^-40), stressing the cotangent complement path. |

### `inverse_trig_adversarial_approx`

Cold approximation of asin, acos, and atan near exact values, zero, endpoints, and large atan inputs.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `inverse_trig_adversarial_approx/asin_zero_p96` | 84.57 ns | 80.88 ns - 89.48 ns | Approximates asin(0), which should collapse before the generic inverse-trig path. |
| `inverse_trig_adversarial_approx/acos_zero_p96` | 294.01 ns | 289.86 ns - 298.61 ns | Approximates acos(0), which should reduce to pi/2. |
| `inverse_trig_adversarial_approx/atan_zero_p96` | 86.71 ns | 83.11 ns - 92.04 ns | Approximates atan(0), which should collapse to zero. |
| `inverse_trig_adversarial_approx/asin_tiny_positive_p96` | 498.85 ns | 492.15 ns - 506.44 ns | Approximates asin(1e-12), stressing the tiny odd series. |
| `inverse_trig_adversarial_approx/acos_tiny_positive_p96` | 1.386 us | 1.362 us - 1.411 us | Approximates acos(1e-12), stressing pi/2 minus the tiny asin path. |
| `inverse_trig_adversarial_approx/atan_tiny_positive_p96` | 483.46 ns | 477.13 ns - 491.80 ns | Approximates atan(1e-12), stressing direct tiny atan setup. |
| `inverse_trig_adversarial_approx/asin_mid_positive_p96` | 11.924 us | 11.813 us - 12.040 us | Approximates asin(7/10), a generic in-domain value. |
| `inverse_trig_adversarial_approx/acos_mid_positive_p96` | 10.130 us | 10.009 us - 10.268 us | Approximates acos(7/10), a generic in-domain value. |
| `inverse_trig_adversarial_approx/atan_mid_positive_p96` | 12.238 us | 11.829 us - 12.716 us | Approximates atan(7/10), a generic in-domain value. |
| `inverse_trig_adversarial_approx/asin_near_one_p96` | 6.354 us | 6.310 us - 6.399 us | Approximates asin(0.999999), stressing endpoint transforms. |
| `inverse_trig_adversarial_approx/acos_near_one_p96` | 5.791 us | 5.736 us - 5.849 us | Approximates acos(0.999999), stressing endpoint transforms. |
| `inverse_trig_adversarial_approx/asin_near_minus_one_p96` | 6.540 us | 6.505 us - 6.572 us | Approximates asin(-0.999999), stressing odd symmetry near the endpoint. |
| `inverse_trig_adversarial_approx/acos_near_minus_one_p96` | 6.975 us | 6.915 us - 7.036 us | Approximates acos(-0.999999), stressing negative endpoint transforms. |
| `inverse_trig_adversarial_approx/atan_large_p96` | 2.910 us | 2.890 us - 2.932 us | Approximates atan(8), stressing reciprocal reduction. |
| `inverse_trig_adversarial_approx/atan_huge_p96` | 1.069 us | 1.042 us - 1.110 us | Approximates atan(10^30), stressing very large reciprocal reduction. |

### `inverse_hyperbolic_adversarial_approx`

Cold approximation of inverse hyperbolic functions at tiny, moderate, large, and endpoint-adjacent arguments.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `inverse_hyperbolic_adversarial_approx/asinh_tiny_positive_p128` | 7.001 us | 6.934 us - 7.101 us | Approximates asinh(1e-12), stressing cancellation avoidance near zero. |
| `inverse_hyperbolic_adversarial_approx/asinh_mid_positive_p128` | 8.740 us | 8.509 us - 9.063 us | Approximates asinh(1/2), a moderate positive value. |
| `inverse_hyperbolic_adversarial_approx/asinh_large_positive_p128` | 7.858 us | 7.687 us - 8.066 us | Approximates asinh(10^6), stressing large-input logarithmic behavior. |
| `inverse_hyperbolic_adversarial_approx/asinh_large_negative_p128` | 7.845 us | 7.793 us - 7.904 us | Approximates asinh(-10^6), stressing odd symmetry for large inputs. |
| `inverse_hyperbolic_adversarial_approx/acosh_one_plus_tiny_p128` | 6.715 us | 6.618 us - 6.830 us | Approximates acosh(1 + 1e-12), stressing the near-one endpoint. |
| `inverse_hyperbolic_adversarial_approx/acosh_sqrt_two_p128` | 9.198 us | 9.077 us - 9.366 us | Approximates acosh(sqrt(2)), a symbolic square-root input. |
| `inverse_hyperbolic_adversarial_approx/acosh_two_p128` | 11.870 us | 11.519 us - 12.497 us | Approximates acosh(2), a moderate exact rational. |
| `inverse_hyperbolic_adversarial_approx/acosh_large_positive_p128` | 7.880 us | 7.834 us - 7.956 us | Approximates acosh(10^6), stressing large-input logarithmic behavior. |
| `inverse_hyperbolic_adversarial_approx/atanh_tiny_positive_p128` | 615.51 ns | 599.21 ns - 639.32 ns | Approximates atanh(1e-12), stressing the tiny odd series. |
| `inverse_hyperbolic_adversarial_approx/atanh_mid_positive_p128` | 1.388 us | 1.371 us - 1.408 us | Approximates atanh(1/2), a moderate exact rational. |
| `inverse_hyperbolic_adversarial_approx/atanh_near_one_p128` | 6.029 us | 5.999 us - 6.061 us | Approximates atanh(0.999999), stressing endpoint logarithmic behavior. |
| `inverse_hyperbolic_adversarial_approx/atanh_near_minus_one_p128` | 6.266 us | 6.225 us - 6.320 us | Approximates atanh(-0.999999), stressing odd symmetry near the endpoint. |

### `real_shortcut_adversarial`

Public `Real` construction shortcuts and domain checks for the same transcendental families.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_shortcut_adversarial/sin_exact_pi_over_six` | 203.28 ns | 199.37 ns - 207.48 ns | Constructs sin(pi/6), which should return the exact rational 1/2. |
| `real_shortcut_adversarial/cos_exact_pi_over_three` | 448.22 ns | 439.32 ns - 458.22 ns | Constructs cos(pi/3), which should return the exact rational 1/2. |
| `real_shortcut_adversarial/tan_exact_pi_over_four` | 193.53 ns | 186.54 ns - 201.82 ns | Constructs tan(pi/4), which should return the exact rational 1. |
| `real_shortcut_adversarial/asin_exact_half` | 559.47 ns | 554.11 ns - 565.43 ns | Constructs asin(1/2), which should return pi/6. |
| `real_shortcut_adversarial/acos_exact_half` | 1.289 us | 1.272 us - 1.306 us | Constructs acos(1/2), which should return pi/3. |
| `real_shortcut_adversarial/atan_exact_one` | 265.77 ns | 257.22 ns - 275.47 ns | Constructs atan(1), which should return pi/4. |
| `real_shortcut_adversarial/asin_domain_error` | 1.017 us | 992.29 ns - 1.045 us | Rejects asin(1 + 1e-12). |
| `real_shortcut_adversarial/acos_domain_error` | 561.33 ns | 545.71 ns - 586.85 ns | Rejects acos(1 + 1e-12). |
| `real_shortcut_adversarial/atanh_endpoint_infinity` | 106.90 ns | 104.88 ns - 108.90 ns | Rejects atanh(1) as an infinite endpoint. |
| `real_shortcut_adversarial/atanh_domain_error` | 648.33 ns | 621.44 ns - 684.52 ns | Rejects atanh(1 + 1e-12). |
| `real_shortcut_adversarial/acosh_domain_error` | 682.91 ns | 655.05 ns - 713.20 ns | Rejects acosh(1 - 1e-12). |

<!-- END adversarial_transcendentals -->
