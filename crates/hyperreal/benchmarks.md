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
| `construction_speed/rational_one` | 15.64 ns | 15.56 ns - 15.72 ns | Constructs `Rational::one()`. |
| `construction_speed/rational_new_one` | 17.03 ns | 16.95 ns - 17.11 ns | Constructs one through `Rational::new(1)`. |
| `construction_speed/computable_one` | 26.39 ns | 26.17 ns - 26.62 ns | Constructs `Computable::one()`. |
| `construction_speed/real_new_rational_one` | 75.59 ns | 75.11 ns - 76.10 ns | Constructs one through `Real::new(Rational::one())`. |
| `construction_speed/real_one` | 76.24 ns | 75.78 ns - 76.77 ns | Constructs one through `Real::one()`. |
| `construction_speed/real_from_i32_one` | 75.37 ns | 74.71 ns - 76.09 ns | Constructs one through integer conversion. |

### `raw_cache_hit_cost`

Cost of cold and cached `Computable::approx` calls for simple values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `raw_cache_hit_cost/zero` | 46.85 ns | 46.62 ns - 47.10 ns | Cached approximation request for exact zero. |
| `raw_cache_hit_cost/one` | 69.47 ns | 68.90 ns - 70.18 ns | Cached approximation request for exact one. |
| `raw_cache_hit_cost/two` | 71.21 ns | 70.43 ns - 72.10 ns | Cached approximation request for exact two. |
| `raw_cache_hit_cost/e` | 73.68 ns | 73.31 ns - 74.10 ns | Cached approximation request for Euler's constant. |
| `raw_cache_hit_cost/pi` | 74.60 ns | 74.20 ns - 75.04 ns | Cached approximation request for pi. |
| `raw_cache_hit_cost/tau` | 73.17 ns | 72.85 ns - 73.53 ns | Cached approximation request for two pi. |

### `structural_query_speed`

Speed of public structural queries across exact, transcendental, and composite `Real` values.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `structural_query_speed/zero_zero_status` | 0.73 ns | 0.72 ns - 0.75 ns | Checks zero/nonzero facts for exact zero. |
| `structural_query_speed/zero_sign_query` | 4.41 ns | 4.35 ns - 4.49 ns | Reads sign facts for exact zero. |
| `structural_query_speed/zero_msd_query` | 6.43 ns | 6.29 ns - 6.66 ns | Reads magnitude facts for exact zero. |
| `structural_query_speed/zero_structural_facts` | 7.45 ns | 7.03 ns - 8.20 ns | Computes full structural facts for exact zero. |
| `structural_query_speed/one_zero_status` | 0.93 ns | 0.91 ns - 0.95 ns | Checks zero/nonzero facts for exact one. |
| `structural_query_speed/one_sign_query` | 22.30 ns | 22.21 ns - 22.40 ns | Reads sign facts for exact one. |
| `structural_query_speed/one_msd_query` | 23.72 ns | 23.58 ns - 23.86 ns | Reads magnitude facts for exact one. |
| `structural_query_speed/one_structural_facts` | 25.08 ns | 24.85 ns - 25.34 ns | Computes full structural facts for exact one. |
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
| `pure_scalar_algorithm_speed/real_exact_add` | 467.64 ns | 461.15 ns - 475.03 ns | Adds exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_mul` | 188.73 ns | 187.72 ns - 189.86 ns | Multiplies exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_div` | 669.34 ns | 665.86 ns - 673.13 ns | Divides exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_sqrt_reduce` | 464.62 ns | 462.32 ns - 467.12 ns | Reduces an exact square-root expression. |
| `pure_scalar_algorithm_speed/real_exact_ln_reduce` | 460.63 ns | 455.48 ns - 467.06 ns | Reduces an exact logarithm of a power of two. |

### `borrowed_op_overhead`

Borrowed versus owned operation overhead for rational and real operands.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `borrowed_op_overhead/rational_clone_pair` | 46.22 ns | 43.25 ns - 51.11 ns | Clones two rational values. |
| `borrowed_op_overhead/rational_add_refs` | 448.17 ns | 384.22 ns - 523.09 ns | Adds rational references. |
| `borrowed_op_overhead/rational_add_owned` | 1.006 us | 835.18 ns - 1.193 us | Adds owned rational values. |
| `borrowed_op_overhead/real_clone_pair` | 282.22 ns | 281.30 ns - 283.24 ns | Clones two scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_unscaled_add_refs` | 167.37 ns | 166.72 ns - 168.12 ns | Adds borrowed unscaled transcendental `Real` values. |
| `borrowed_op_overhead/real_unscaled_add_owned` | 222.18 ns | 219.08 ns - 225.57 ns | Adds owned unscaled transcendental `Real` values. |
| `borrowed_op_overhead/real_add_refs` | 505.37 ns | 503.69 ns - 507.27 ns | Adds borrowed scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_add_owned` | 826.04 ns | 812.20 ns - 842.52 ns | Adds owned scaled transcendental `Real` values. |
| `borrowed_op_overhead/real_dot3_refs_dense_symbolic` | 3.493 us | 3.479 us - 3.509 us | Computes a borrowed three-lane symbolic dot product with no rational shortcut terms. |
| `borrowed_op_overhead/real_dot3_refs_mixed_structural` | 698.07 ns | 695.13 ns - 701.45 ns | Computes a borrowed three-lane symbolic dot product with exact zero and rational scale terms. |
| `borrowed_op_overhead/real_dot4_refs_dense_symbolic` | 6.152 us | 6.127 us - 6.178 us | Computes a borrowed four-lane symbolic dot product with no rational shortcut terms. |
| `borrowed_op_overhead/real_dot4_refs_mixed_structural` | 750.01 ns | 747.47 ns - 752.72 ns | Computes a borrowed four-lane symbolic dot product with exact zero and rational scale terms. |

### `dense_algebra`

Small dense algebra kernels that stress repeated exact and symbolic operations.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `dense_algebra/rational_dot_64` | 36.684 us | 36.363 us - 37.069 us | Computes a 64-element rational dot product. |
| `dense_algebra/rational_matmul_8` | 233.879 us | 232.885 us - 234.970 us | Computes an 8x8 rational matrix multiply. |
| `dense_algebra/real_dot_36` | 27.014 us | 26.893 us - 27.144 us | Computes a 36-element dot product over symbolic `Real` values. |
| `dense_algebra/real_matmul_6` | 149.05 us | 148.47 us - 149.71 us | Computes a 6x6 matrix multiply over symbolic `Real` values. |

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
| `symbolic_reductions/sqrt_pi_square` | 132.06 ns | 131.34 ns - 132.85 ns | Reduces sqrt(pi^2). |
| `symbolic_reductions/sqrt_pi_e_square` | 174.40 ns | 173.37 ns - 175.50 ns | Reduces sqrt((pi * e)^2). |
| `symbolic_reductions/sqrt_scaled_exp_squarefree` | 781.80 ns | 777.99 ns - 786.04 ns | Guards the deferred generic sqrt path for scaled exponential square-free residuals after exact factoring regressed construction. |
| `symbolic_reductions/ln_scaled_e` | 1.892 us | 1.847 us - 1.956 us | Reduces ln(2 * e). |
| `symbolic_reductions/sub_pi_three` | 1.751 us | 1.729 us - 1.774 us | Builds the certified pi - 3 constant-offset form. |
| `symbolic_reductions/pi_minus_three_facts` | 38.85 ns | 38.03 ns - 39.97 ns | Reads structural facts for the cached pi - 3 offset form. |
| `symbolic_reductions/div_exp_exp` | 3.730 us | 3.662 us - 3.822 us | Reduces e^3 / e. |
| `symbolic_reductions/div_pi_square_e` | 3.277 us | 3.246 us - 3.310 us | Reduces pi^2 / e. |
| `symbolic_reductions/div_const_products` | 4.542 us | 4.407 us - 4.703 us | Reduces (pi^3 * e^5) / (pi * e^2). |
| `symbolic_reductions/inverse_pi` | 96.94 ns | 95.48 ns - 99.13 ns | Builds the reciprocal of pi. |
| `symbolic_reductions/div_one_pi` | 284.28 ns | 276.33 ns - 292.77 ns | Reduces 1 / pi. |
| `symbolic_reductions/div_e_pi` | 349.65 ns | 346.93 ns - 352.58 ns | Reduces e / pi. |
| `symbolic_reductions/mul_pi_inverse_pi` | 275.41 ns | 265.87 ns - 285.37 ns | Multiplies pi by its reciprocal. |
| `symbolic_reductions/mul_pi_e_sqrt_two` | 483.96 ns | 480.71 ns - 487.44 ns | Builds the factored pi * e * sqrt(2) form. |
| `symbolic_reductions/mul_const_product_sqrt_sqrt` | 665.87 ns | 661.72 ns - 670.62 ns | Cancels sqrt(2) from (pi * e * sqrt(2)) * sqrt(2). |
| `symbolic_reductions/div_const_product_sqrt_e` | 1.060 us | 1.054 us - 1.067 us | Reduces (pi * e * sqrt(2)) / e. |
| `symbolic_reductions/inverse_const_product_sqrt` | 3.211 us | 3.193 us - 3.231 us | Builds a rationalized reciprocal of pi * e * sqrt(2). |

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
| `computable_cache/ratio_approx_cold_p128` | 106.37 ns | 105.55 ns - 107.24 ns | Approximates a rational value at p=-128 from a fresh clone. |
| `computable_cache/ratio_approx_cached_p128` | 40.70 ns | 40.60 ns - 40.81 ns | Repeats an already cached rational approximation at p=-128. |
| `computable_cache/pi_approx_cold_p128` | 61.04 ns | 60.64 ns - 61.47 ns | Approximates pi at p=-128 from a fresh clone. |
| `computable_cache/pi_approx_cached_p128` | 45.89 ns | 45.75 ns - 46.05 ns | Repeats an already cached pi approximation at p=-128. |
| `computable_cache/pi_plus_tiny_cold_p128` | 185.04 ns | 184.65 ns - 185.45 ns | Approximates pi plus a tiny exact rational perturbation. |
| `computable_cache/pi_minus_tiny_cold_p128` | 264.29 ns | 262.01 ns - 267.97 ns | Approximates pi minus a tiny exact rational perturbation. |

### `computable_bounds`

Structural sign and bound discovery for deep or perturbed computable trees.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_bounds/deep_scaled_product_sign` | 205.40 ns | 153.37 ns - 308.68 ns | Finds the sign of a deep scaled product. |
| `computable_bounds/scaled_square_sign` | 738.87 ns | 336.18 ns - 1.543 us | Finds the sign of repeated squaring with exact scale factors. |
| `computable_bounds/sqrt_scaled_square_sign` | 246.62 ns | 242.57 ns - 250.61 ns | Finds the sign after taking a square root of a scaled square. |
| `computable_bounds/deep_structural_bound_sign` | 18.19 ns | 17.99 ns - 18.41 ns | Finds sign through repeated multiply/inverse/negate structural transformations. |
| `computable_bounds/deep_structural_bound_sign_cached` | 3.86 ns | 3.84 ns - 3.90 ns | Reads the cached sign of the deep structural-bound chain. |
| `computable_bounds/deep_structural_bound_facts_cached` | 12.77 ns | 12.72 ns - 12.83 ns | Reads cached structural facts for the deep structural-bound chain. |
| `computable_bounds/perturbed_scaled_product_sign` | 331.56 ns | 211.84 ns - 568.80 ns | Finds sign for a deeply scaled value with a tiny perturbation. |
| `computable_bounds/perturbed_scaled_product_sign_until` | 309.67 ns | 215.72 ns - 494.45 ns | Refines sign for the perturbed scaled product only to p=-128. |
| `computable_bounds/pi_minus_tiny_sign` | 71.02 ns | 70.15 ns - 71.82 ns | Finds sign for pi minus a tiny exact rational. |
| `computable_bounds/pi_minus_tiny_sign_cached` | 3.83 ns | 3.82 ns - 3.84 ns | Reads cached sign for pi minus a tiny exact rational. |

### `computable_compare`

Ordering and absolute-comparison shortcuts.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_compare/compare_to_opposite_sign` | 11.51 ns | 11.48 ns - 11.56 ns | Compares values with known opposite signs. |
| `computable_compare/compare_to_exact_msd_gap` | 17.80 ns | 17.71 ns - 17.91 ns | Compares values with a large exact magnitude gap. |
| `computable_compare/compare_absolute_exact_rational` | 97.19 ns | 96.67 ns - 97.75 ns | Compares absolute values of exact rationals. |
| `computable_compare/compare_absolute_dominant_add` | 153.29 ns | 152.08 ns - 154.75 ns | Compares a dominant term against the same term plus a tiny addend. |
| `computable_compare/compare_absolute_exact_msd_gap` | 152.03 ns | 151.27 ns - 152.91 ns | Compares absolute values with a large exact magnitude gap. |

### `computable_transcendentals`

Low-level approximation kernels and deep expression-tree stress cases.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `computable_transcendentals/legacy_exp_one_p128` | 2.914 us | 2.900 us - 2.929 us | Runs the legacy direct exp series for input 1 at p=-128. |
| `computable_transcendentals/e_constant_cold_p128` | 67.17 ns | 66.54 ns - 67.95 ns | Approximates the shared e constant from a fresh clone. |
| `computable_transcendentals/e_constant_cached_p128` | 45.01 ns | 44.84 ns - 45.21 ns | Repeats a cached approximation of e. |
| `computable_transcendentals/legacy_exp_half_p128` | 2.524 us | 2.514 us - 2.536 us | Runs the legacy direct exp series for input 1/2 at p=-128. |
| `computable_transcendentals/exp_cold_p128` | 2.366 us | 2.357 us - 2.376 us | Approximates exp(7/5) from a fresh clone. |
| `computable_transcendentals/exp_cached_p128` | 37.98 ns | 37.89 ns - 38.07 ns | Repeats a cached exp(7/5) approximation. |
| `computable_transcendentals/exp_large_cold_p128` | 7.179 us | 7.144 us - 7.216 us | Approximates exp(128), exercising large-argument reduction. |
| `computable_transcendentals/exp_half_cold_p128` | 2.819 us | 2.811 us - 2.827 us | Approximates exp(1/2). |
| `computable_transcendentals/exp_near_limit_cold_p128` | 2.808 us | 2.797 us - 2.819 us | Approximates exp near a prescaling threshold. |
| `computable_transcendentals/exp_near_limit_cached_p128` | 37.60 ns | 37.43 ns - 37.78 ns | Repeats a cached near-threshold exp approximation. |
| `computable_transcendentals/exp_zero_cold_p128` | 73.28 ns | 72.71 ns - 73.90 ns | Approximates exp(0). |
| `computable_transcendentals/ln_cold_p128` | 4.447 us | 4.411 us - 4.494 us | Approximates ln(11/7). |
| `computable_transcendentals/ln_cached_p128` | 37.75 ns | 37.62 ns - 37.89 ns | Repeats a cached ln(11/7) approximation. |
| `computable_transcendentals/ln_smooth_rational_cold_p128` | 912.48 ns | 905.93 ns - 918.87 ns | Approximates ln(45/14), which can decompose into shared prime-log constants. |
| `computable_transcendentals/ln_nonsmooth_rational_cold_p128` | 2.577 us | 2.567 us - 2.588 us | Approximates ln(11/13), guarding the generic exact-rational log fallback. |
| `computable_transcendentals/ln_large_cold_p128` | 306.17 ns | 304.62 ns - 307.88 ns | Approximates ln(1024), exercising large-input reduction. |
| `computable_transcendentals/ln_large_cached_p128` | 38.19 ns | 38.09 ns - 38.30 ns | Repeats a cached ln(1024) approximation. |
| `computable_transcendentals/ln_tiny_cold_p128` | 240.10 ns | 237.93 ns - 242.38 ns | Approximates ln(2^-1024), exercising tiny-input reduction. |
| `computable_transcendentals/ln_near_limit_cold_p128` | 4.541 us | 4.517 us - 4.571 us | Approximates ln near the prescaled-ln limit. |
| `computable_transcendentals/ln_near_limit_cached_p128` | 37.62 ns | 37.47 ns - 37.79 ns | Repeats a cached near-limit ln approximation. |
| `computable_transcendentals/ln_one_cold_p128` | 32.71 ns | 32.54 ns - 32.89 ns | Approximates ln(1). |
| `computable_transcendentals/sqrt_cold_p128` | 758.54 ns | 754.48 ns - 763.01 ns | Approximates sqrt(2). |
| `computable_transcendentals/sqrt_squarefree_scaled_cold_p128` | 181.71 ns | 180.35 ns - 183.15 ns | Approximates sqrt(12), which can reduce to 2*sqrt(3). |
| `computable_transcendentals/sqrt_cached_p128` | 38.23 ns | 38.12 ns - 38.35 ns | Repeats a cached sqrt(2) approximation. |
| `computable_transcendentals/sqrt_single_scaled_square_cold_p128` | 1.074 us | 1.069 us - 1.078 us | Builds and approximates sqrt((7*pi/8)^2). |
| `computable_transcendentals/sin_cold_p96` | 1.539 us | 1.530 us - 1.549 us | Approximates sin(7/5). |
| `computable_transcendentals/sin_cached_p96` | 37.96 ns | 37.73 ns - 38.32 ns | Repeats a cached sin(7/5) approximation. |
| `computable_transcendentals/cos_cold_p96` | 1.491 us | 1.485 us - 1.497 us | Approximates cos(7/5). |
| `computable_transcendentals/sin_f64_cold_p96` | 1.667 us | 1.662 us - 1.673 us | Approximates sin(1.23456789 imported exactly from f64). |
| `computable_transcendentals/cos_f64_cold_p96` | 1.656 us | 1.651 us - 1.661 us | Approximates cos(1.23456789 imported exactly from f64). |
| `computable_transcendentals/sin_1e6_cold_p96` | 2.310 us | 2.298 us - 2.324 us | Approximates sin(1000000). |
| `computable_transcendentals/cos_1e6_cold_p96` | 2.333 us | 2.318 us - 2.350 us | Approximates cos(1000000). |
| `computable_transcendentals/sin_1e30_cold_p96` | 2.078 us | 2.068 us - 2.089 us | Approximates sin(10^30). |
| `computable_transcendentals/cos_1e30_cold_p96` | 2.209 us | 2.199 us - 2.219 us | Approximates cos(10^30). |
| `computable_transcendentals/cos_cached_p96` | 37.36 ns | 37.28 ns - 37.45 ns | Repeats a cached cos(7/5) approximation. |
| `computable_transcendentals/tan_cold_p96` | 3.278 us | 3.271 us - 3.286 us | Approximates tan(7/5). |
| `computable_transcendentals/tan_cached_p96` | 37.18 ns | 37.13 ns - 37.24 ns | Repeats a cached tan(7/5) approximation. |
| `computable_transcendentals/sin_zero_cold_p96` | 34.62 ns | 34.31 ns - 34.99 ns | Approximates sin(0). |
| `computable_transcendentals/cos_zero_cold_p96` | 76.62 ns | 75.75 ns - 77.62 ns | Approximates cos(0). |
| `computable_transcendentals/tan_zero_cold_p96` | 34.27 ns | 34.02 ns - 34.61 ns | Approximates tan(0). |
| `computable_transcendentals/tan_near_half_pi_cold_p96` | 3.665 us | 2.871 us - 5.238 us | Approximates tangent near pi/2. |
| `computable_transcendentals/tan_near_half_pi_cached_p96` | 40.52 ns | 40.22 ns - 40.88 ns | Repeats cached tangent near pi/2. |
| `computable_transcendentals/sin_huge_cold_p96` | 1.621 us | 1.605 us - 1.639 us | Approximates sine of a huge pi multiple plus offset. |
| `computable_transcendentals/cos_huge_cold_p96` | 1.499 us | 1.492 us - 1.506 us | Approximates cosine of a huge pi multiple plus offset. |
| `computable_transcendentals/tan_huge_cold_p96` | 3.338 us | 3.322 us - 3.356 us | Approximates tangent of a huge pi multiple plus offset. |
| `computable_transcendentals/asin_cold_p96` | 6.901 us | 6.533 us - 7.610 us | Approximates a computable asin expression. |
| `computable_transcendentals/asin_cached_p96` | 37.71 ns | 37.53 ns - 37.92 ns | Repeats a cached computable asin approximation. |
| `computable_transcendentals/acos_cold_p96` | 8.990 us | 8.947 us - 9.036 us | Approximates a computable acos expression. |
| `computable_transcendentals/acos_cached_p96` | 37.65 ns | 37.38 ns - 37.97 ns | Repeats a cached computable acos approximation. |
| `computable_transcendentals/asin_tiny_cold_p96` | 379.78 ns | 376.14 ns - 384.04 ns | Approximates asin(1e-12), exercising the tiny-input series. |
| `computable_transcendentals/acos_tiny_cold_p96` | 705.94 ns | 700.98 ns - 711.52 ns | Approximates acos(1e-12), exercising the tiny-input complement. |
| `computable_transcendentals/asin_near_one_cold_p96` | 4.829 us | 4.801 us - 4.859 us | Approximates asin(0.999999), exercising the endpoint complement. |
| `computable_transcendentals/acos_near_one_cold_p96` | 4.368 us | 4.343 us - 4.394 us | Approximates acos(0.999999), exercising the endpoint transform. |
| `computable_transcendentals/atan_cold_p96` | 7.302 us | 7.264 us - 7.345 us | Approximates atan(7/10). |
| `computable_transcendentals/atan_cached_p96` | 37.64 ns | 37.53 ns - 37.76 ns | Repeats a cached atan(7/10) approximation. |
| `computable_transcendentals/atan_large_cold_p96` | 1.831 us | 1.811 us - 1.853 us | Approximates atan(8), exercising argument reduction. |
| `computable_transcendentals/asin_zero_cold_p96` | 33.26 ns | 33.08 ns - 33.45 ns | Approximates asin(0) expression. |
| `computable_transcendentals/atan_zero_cold_p96` | 34.79 ns | 34.30 ns - 35.38 ns | Approximates atan(0). |
| `computable_transcendentals/asinh_cold_p128` | 6.381 us | 6.352 us - 6.413 us | Approximates a computable asinh expression. |
| `computable_transcendentals/asinh_cached_p128` | 37.91 ns | 37.75 ns - 38.08 ns | Repeats a cached computable asinh approximation. |
| `computable_transcendentals/acosh_cold_p128` | 9.576 us | 9.526 us - 9.639 us | Approximates a computable acosh expression. |
| `computable_transcendentals/acosh_cached_p128` | 39.36 ns | 39.14 ns - 39.63 ns | Repeats a cached computable acosh approximation. |
| `computable_transcendentals/atanh_cold_p128` | 274.82 ns | 192.47 ns - 437.95 ns | Approximates a computable atanh expression. |
| `computable_transcendentals/atanh_cached_p128` | 37.71 ns | 37.59 ns - 37.84 ns | Repeats a cached computable atanh approximation. |
| `computable_transcendentals/atanh_tiny_cold_p128` | 473.10 ns | 470.93 ns - 475.47 ns | Approximates atanh(1e-12), exercising the tiny-input series. |
| `computable_transcendentals/atanh_near_one_cold_p128` | 2.898 us | 2.864 us - 2.938 us | Approximates atanh(0.999999), exercising the endpoint log transform. |
| `computable_transcendentals/asinh_zero_cold_p128` | 33.60 ns | 33.32 ns - 33.91 ns | Approximates asinh(0) expression. |
| `computable_transcendentals/atanh_zero_cold_p128` | 33.56 ns | 33.33 ns - 33.81 ns | Approximates atanh(0) expression. |
| `computable_transcendentals/deep_add_chain_cold_p128` | 90.61 ns | 90.26 ns - 90.95 ns | Approximates a 5000-node addition chain. |
| `computable_transcendentals/deep_multiply_chain_cold_p128` | 74.20 ns | 73.56 ns - 74.87 ns | Approximates a 5000-node multiply-by-one chain. |
| `computable_transcendentals/deep_multiply_identity_chain_cold_p128` | 89.32 ns | 88.86 ns - 89.81 ns | Approximates a deep identity multiplication chain around pi. |
| `computable_transcendentals/deep_scaled_product_chain_cold_p128` | 729.63 ns | 592.43 ns - 995.24 ns | Approximates a deep product of exact scale factors. |
| `computable_transcendentals/perturbed_scaled_product_chain_cold_p128` | 839.66 ns | 834.97 ns - 844.33 ns | Approximates a deep scaled product with a tiny perturbation. |
| `computable_transcendentals/scaled_square_chain_cold_p128` | 1.415 us | 1.400 us - 1.433 us | Approximates repeated squaring of a scaled irrational. |
| `computable_transcendentals/asymmetric_product_bad_order_cold_p128` | 1.130 us | 932.65 ns - 1.521 us | Approximates an asymmetric product order stress case. |
| `computable_transcendentals/sqrt_scaled_square_chain_cold_p128` | 1.087 us | 1.082 us - 1.094 us | Approximates sqrt of a scaled-square chain. |
| `computable_transcendentals/warmed_zero_product_cold_p128` | 502.57 ns | 501.40 ns - 503.96 ns | Approximates a product involving a warmed zero sum. |
| `computable_transcendentals/inverse_scaled_product_chain_cold_p128` | 1.016 us | 763.66 ns - 1.472 us | Approximates the inverse of a deep scaled product. |
| `computable_transcendentals/deep_inverse_pair_chain_cold_p128` | 90.70 ns | 90.14 ns - 91.31 ns | Approximates a chain of inverse(inverse(x)) pairs. |
| `computable_transcendentals/deep_negated_square_chain_cold_p128` | 88.49 ns | 88.01 ns - 89.01 ns | Approximates repeated negate-square-sqrt transformations. |
| `computable_transcendentals/deep_negative_one_product_chain_cold_p128` | 92.50 ns | 91.42 ns - 93.79 ns | Approximates repeated multiplication by -1. |
| `computable_transcendentals/deep_half_product_chain_cold_p128` | 219.91 ns | 217.32 ns - 222.83 ns | Approximates repeated multiplication by 1/2. |
| `computable_transcendentals/deep_half_square_chain_cold_p128` | 1.020 us | 1.010 us - 1.031 us | Approximates repeated squaring after scaling by 1/2. |
| `computable_transcendentals/deep_sqrt_square_chain_cold_p128` | 128.71 ns | 85.12 ns - 215.07 ns | Approximates repeated sqrt-square simplification. |
| `computable_transcendentals/inverse_half_product_chain_cold_p128` | 518.80 ns | 516.98 ns - 520.73 ns | Approximates the inverse of a deep half-product chain. |

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
| `trig_adversarial_approx/sin_tiny_rational_p96` | 633.92 ns | 625.70 ns - 644.82 ns | Approximates sin(1e-12), stressing direct tiny-argument setup. |
| `trig_adversarial_approx/cos_tiny_rational_p96` | 628.25 ns | 623.85 ns - 632.51 ns | Approximates cos(1e-12), stressing direct tiny-argument setup. |
| `trig_adversarial_approx/tan_tiny_rational_p96` | 1.144 us | 1.125 us - 1.165 us | Approximates tan(1e-12), stressing direct tiny-argument setup. |
| `trig_adversarial_approx/sin_medium_rational_p96` | 1.820 us | 1.809 us - 1.833 us | Approximates sin(7/5), a moderate non-pi rational. |
| `trig_adversarial_approx/cos_medium_rational_p96` | 1.696 us | 1.686 us - 1.708 us | Approximates cos(7/5), a moderate non-pi rational. |
| `trig_adversarial_approx/tan_medium_rational_p96` | 3.690 us | 3.623 us - 3.757 us | Approximates tan(7/5), a moderate non-pi rational. |
| `trig_adversarial_approx/sin_f64_exact_p96` | 1.948 us | 1.937 us - 1.962 us | Approximates sin(1.23456789 imported as an exact dyadic rational). |
| `trig_adversarial_approx/cos_f64_exact_p96` | 1.961 us | 1.933 us - 1.999 us | Approximates cos(1.23456789 imported as an exact dyadic rational). |
| `trig_adversarial_approx/sin_1e6_p96` | 2.367 us | 2.331 us - 2.411 us | Approximates sin(1000000), stressing integer argument reduction. |
| `trig_adversarial_approx/cos_1e6_p96` | 2.400 us | 2.365 us - 2.439 us | Approximates cos(1000000), stressing integer argument reduction. |
| `trig_adversarial_approx/tan_1e6_p96` | 4.567 us | 4.518 us - 4.638 us | Approximates tan(1000000), stressing integer argument reduction. |
| `trig_adversarial_approx/sin_1e30_p96` | 2.211 us | 2.188 us - 2.237 us | Approximates sin(10^30), stressing very large integer reduction. |
| `trig_adversarial_approx/cos_1e30_p96` | 2.344 us | 2.328 us - 2.365 us | Approximates cos(10^30), stressing very large integer reduction. |
| `trig_adversarial_approx/tan_1e30_p96` | 4.015 us | 3.980 us - 4.052 us | Approximates tan(10^30), stressing very large integer reduction. |
| `trig_adversarial_approx/sin_huge_pi_plus_offset_p96` | 2.337 us | 2.328 us - 2.347 us | Approximates sin(2^512*pi + 7/5), stressing exact pi-multiple cancellation. |
| `trig_adversarial_approx/cos_huge_pi_plus_offset_p96` | 2.239 us | 2.225 us - 2.256 us | Approximates cos(2^512*pi + 7/5), stressing exact pi-multiple cancellation. |
| `trig_adversarial_approx/tan_huge_pi_plus_offset_p96` | 4.163 us | 4.111 us - 4.224 us | Approximates tan(2^512*pi + 7/5), stressing exact pi-multiple cancellation. |
| `trig_adversarial_approx/tan_near_half_pi_p96` | 2.455 us | 2.437 us - 2.474 us | Approximates tan(pi/2 - 2^-40), stressing the cotangent complement path. |

### `inverse_trig_adversarial_approx`

Cold approximation of asin, acos, and atan near exact values, zero, endpoints, and large atan inputs.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `inverse_trig_adversarial_approx/asin_zero_p96` | 76.15 ns | 73.17 ns - 80.73 ns | Approximates asin(0), which should collapse before the generic inverse-trig path. |
| `inverse_trig_adversarial_approx/acos_zero_p96` | 305.44 ns | 293.66 ns - 315.80 ns | Approximates acos(0), which should reduce to pi/2. |
| `inverse_trig_adversarial_approx/atan_zero_p96` | 77.48 ns | 73.71 ns - 84.21 ns | Approximates atan(0), which should collapse to zero. |
| `inverse_trig_adversarial_approx/asin_tiny_positive_p96` | 446.42 ns | 438.98 ns - 457.41 ns | Approximates asin(1e-12), stressing the tiny odd series. |
| `inverse_trig_adversarial_approx/acos_tiny_positive_p96` | 919.10 ns | 883.82 ns - 961.65 ns | Approximates acos(1e-12), stressing pi/2 minus the tiny asin path. |
| `inverse_trig_adversarial_approx/atan_tiny_positive_p96` | 344.37 ns | 337.68 ns - 352.40 ns | Approximates atan(1e-12), stressing direct tiny atan setup. |
| `inverse_trig_adversarial_approx/asin_mid_positive_p96` | 11.134 us | 11.016 us - 11.241 us | Approximates asin(7/10), a generic in-domain value. |
| `inverse_trig_adversarial_approx/acos_mid_positive_p96` | 9.208 us | 9.138 us - 9.288 us | Approximates acos(7/10), a generic in-domain value. |
| `inverse_trig_adversarial_approx/atan_mid_positive_p96` | 7.397 us | 7.309 us - 7.490 us | Approximates atan(7/10), a generic in-domain value. |
| `inverse_trig_adversarial_approx/asin_near_one_p96` | 5.189 us | 5.114 us - 5.283 us | Approximates asin(0.999999), stressing endpoint transforms. |
| `inverse_trig_adversarial_approx/acos_near_one_p96` | 4.431 us | 4.376 us - 4.501 us | Approximates acos(0.999999), stressing endpoint transforms. |
| `inverse_trig_adversarial_approx/asin_near_minus_one_p96` | 5.353 us | 5.279 us - 5.441 us | Approximates asin(-0.999999), stressing odd symmetry near the endpoint. |
| `inverse_trig_adversarial_approx/acos_near_minus_one_p96` | 5.552 us | 5.431 us - 5.699 us | Approximates acos(-0.999999), stressing negative endpoint transforms. |
| `inverse_trig_adversarial_approx/atan_large_p96` | 1.880 us | 1.847 us - 1.932 us | Approximates atan(8), stressing reciprocal reduction. |
| `inverse_trig_adversarial_approx/atan_huge_p96` | 870.97 ns | 832.66 ns - 911.31 ns | Approximates atan(10^30), stressing very large reciprocal reduction. |

### `inverse_hyperbolic_adversarial_approx`

Cold approximation of inverse hyperbolic functions at tiny, moderate, large, and endpoint-adjacent arguments.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `inverse_hyperbolic_adversarial_approx/asinh_tiny_positive_p128` | 587.03 ns | 579.69 ns - 597.74 ns | Approximates asinh(1e-12), stressing cancellation avoidance near zero. |
| `inverse_hyperbolic_adversarial_approx/asinh_mid_positive_p128` | 10.514 us | 8.204 us - 12.974 us | Approximates asinh(1/2), a moderate positive value. |
| `inverse_hyperbolic_adversarial_approx/asinh_large_positive_p128` | 7.653 us | 7.478 us - 7.842 us | Approximates asinh(10^6), stressing large-input logarithmic behavior. |
| `inverse_hyperbolic_adversarial_approx/asinh_large_negative_p128` | 7.860 us | 7.710 us - 8.030 us | Approximates asinh(-10^6), stressing odd symmetry for large inputs. |
| `inverse_hyperbolic_adversarial_approx/acosh_one_plus_tiny_p128` | 6.559 us | 6.446 us - 6.691 us | Approximates acosh(1 + 1e-12), stressing the near-one endpoint. |
| `inverse_hyperbolic_adversarial_approx/acosh_sqrt_two_p128` | 8.839 us | 8.705 us - 9.054 us | Approximates acosh(sqrt(2)), a symbolic square-root input. |
| `inverse_hyperbolic_adversarial_approx/acosh_two_p128` | 11.137 us | 11.074 us - 11.203 us | Approximates acosh(2), a moderate exact rational. |
| `inverse_hyperbolic_adversarial_approx/acosh_large_positive_p128` | 7.941 us | 7.896 us - 7.992 us | Approximates acosh(10^6), stressing large-input logarithmic behavior. |
| `inverse_hyperbolic_adversarial_approx/atanh_tiny_positive_p128` | 542.70 ns | 537.50 ns - 551.87 ns | Approximates atanh(1e-12), stressing the tiny odd series. |
| `inverse_hyperbolic_adversarial_approx/atanh_mid_positive_p128` | 1.142 us | 1.106 us - 1.189 us | Approximates atanh(1/2), a moderate exact rational. |
| `inverse_hyperbolic_adversarial_approx/atanh_near_one_p128` | 5.514 us | 5.411 us - 5.675 us | Approximates atanh(0.999999), stressing endpoint logarithmic behavior. |
| `inverse_hyperbolic_adversarial_approx/atanh_near_minus_one_p128` | 5.668 us | 5.623 us - 5.724 us | Approximates atanh(-0.999999), stressing odd symmetry near the endpoint. |

### `real_shortcut_adversarial`

Public `Real` construction shortcuts and domain checks for the same transcendental families.

| Benchmark output | Mean | 95% CI | What it measures |
| --- | ---: | ---: | --- |
| `real_shortcut_adversarial/sin_exact_pi_over_six` | 200.51 ns | 195.28 ns - 207.92 ns | Constructs sin(pi/6), which should return the exact rational 1/2. |
| `real_shortcut_adversarial/cos_exact_pi_over_three` | 442.35 ns | 438.76 ns - 446.17 ns | Constructs cos(pi/3), which should return the exact rational 1/2. |
| `real_shortcut_adversarial/tan_exact_pi_over_four` | 191.17 ns | 187.08 ns - 196.56 ns | Constructs tan(pi/4), which should return the exact rational 1. |
| `real_shortcut_adversarial/asin_exact_half` | 561.31 ns | 556.37 ns - 566.68 ns | Constructs asin(1/2), which should return pi/6. |
| `real_shortcut_adversarial/acos_exact_half` | 1.287 us | 1.279 us - 1.298 us | Constructs acos(1/2), which should return pi/3. |
| `real_shortcut_adversarial/atan_exact_one` | 255.65 ns | 249.65 ns - 263.55 ns | Constructs atan(1), which should return pi/4. |
| `real_shortcut_adversarial/asin_domain_error` | 993.65 ns | 962.65 ns - 1.038 us | Rejects asin(1 + 1e-12). |
| `real_shortcut_adversarial/acos_domain_error` | 535.70 ns | 529.43 ns - 545.30 ns | Rejects acos(1 + 1e-12). |
| `real_shortcut_adversarial/atanh_endpoint_infinity` | 98.73 ns | 95.13 ns - 105.01 ns | Rejects atanh(1) as an infinite endpoint. |
| `real_shortcut_adversarial/atanh_domain_error` | 673.44 ns | 623.99 ns - 760.94 ns | Rejects atanh(1 + 1e-12). |
| `real_shortcut_adversarial/acosh_domain_error` | 592.28 ns | 587.51 ns - 597.67 ns | Rejects acosh(1 - 1e-12). |

<!-- END adversarial_transcendentals -->
