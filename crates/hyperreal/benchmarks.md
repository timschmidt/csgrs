# Benchmark Reference

This file is updated by the Criterion benchmark binaries. Run `cargo bench` to refresh the benchmark output catalogue.

The timings themselves are emitted by Criterion under `target/criterion/`; this file documents the benchmark IDs and what each one measures.

<!-- BEGIN borrowed_ops -->
## `borrowed_ops`

Compares owned arithmetic with borrowed arithmetic for exact and irrational values.

### `rational_ops`

Owned versus borrowed arithmetic for exact `Rational` values.

| Benchmark output | What it measures |
| --- | --- |
| `rational_ops/add_owned` | Adds cloned owned operands. |
| `rational_ops/add_refs` | Adds borrowed operands without cloning both inputs. |
| `rational_ops/sub_owned` | Subtracts cloned owned operands. |
| `rational_ops/sub_refs` | Subtracts borrowed operands. |
| `rational_ops/mul_owned` | Multiplies cloned owned operands. |
| `rational_ops/mul_refs` | Multiplies borrowed operands. |
| `rational_ops/div_owned` | Divides cloned owned operands. |
| `rational_ops/div_refs` | Divides borrowed operands. |

### `real_ops`

Owned versus borrowed arithmetic for exact rational-backed `Real` values.

| Benchmark output | What it measures |
| --- | --- |
| `real_ops/add_owned` | Adds cloned owned operands. |
| `real_ops/add_refs` | Adds borrowed operands without cloning both inputs. |
| `real_ops/sub_owned` | Subtracts cloned owned operands. |
| `real_ops/sub_refs` | Subtracts borrowed operands. |
| `real_ops/mul_owned` | Multiplies cloned owned operands. |
| `real_ops/mul_refs` | Multiplies borrowed operands. |
| `real_ops/div_owned` | Divides cloned owned operands. |
| `real_ops/div_refs` | Divides borrowed operands. |

### `real_irrational_ops`

Owned versus borrowed arithmetic for symbolic irrational `Real` values.

| Benchmark output | What it measures |
| --- | --- |
| `real_irrational_ops/add_owned` | Adds cloned owned operands. |
| `real_irrational_ops/add_refs` | Adds borrowed operands without cloning both inputs. |
| `real_irrational_ops/sub_owned` | Subtracts cloned owned operands. |
| `real_irrational_ops/sub_refs` | Subtracts borrowed operands. |
| `real_irrational_ops/mul_owned` | Multiplies cloned owned operands. |
| `real_irrational_ops/mul_refs` | Multiplies borrowed operands. |
| `real_irrational_ops/div_owned` | Divides cloned owned operands. |
| `real_irrational_ops/div_refs` | Divides borrowed operands. |

<!-- END borrowed_ops -->

<!-- BEGIN scalar_micro -->
## `scalar_micro`

Microbenchmarks for scalar operations, structural queries, cache hits, and dense exact arithmetic.

### `raw_cache_hit_cost`

Cost of cold and cached `Computable::approx` calls for simple values.

| Benchmark output | What it measures |
| --- | --- |
| `raw_cache_hit_cost/zero` | Cached approximation request for exact zero. |
| `raw_cache_hit_cost/one` | Cached approximation request for exact one. |
| `raw_cache_hit_cost/two` | Cached approximation request for exact two. |
| `raw_cache_hit_cost/e` | Cached approximation request for Euler's constant. |
| `raw_cache_hit_cost/pi` | Cached approximation request for pi. |
| `raw_cache_hit_cost/tau` | Cached approximation request for two pi. |

### `structural_query_speed`

Speed of public structural queries across exact, transcendental, and composite `Real` values.

| Benchmark output | What it measures |
| --- | --- |
| `structural_query_speed/zero_zero_status` | Checks zero/nonzero facts for exact zero. |
| `structural_query_speed/zero_sign_query` | Reads sign facts for exact zero. |
| `structural_query_speed/zero_msd_query` | Reads magnitude facts for exact zero. |
| `structural_query_speed/zero_structural_facts` | Computes full structural facts for exact zero. |
| `structural_query_speed/one_zero_status` | Checks zero/nonzero facts for exact one. |
| `structural_query_speed/one_sign_query` | Reads sign facts for exact one. |
| `structural_query_speed/one_msd_query` | Reads magnitude facts for exact one. |
| `structural_query_speed/one_structural_facts` | Computes full structural facts for exact one. |
| `structural_query_speed/negative_zero_status` | Checks zero/nonzero facts for an exact negative integer. |
| `structural_query_speed/negative_sign_query` | Reads sign facts for an exact negative integer. |
| `structural_query_speed/negative_msd_query` | Reads magnitude facts for an exact negative integer. |
| `structural_query_speed/negative_structural_facts` | Computes full structural facts for an exact negative integer. |
| `structural_query_speed/tiny_exact_zero_status` | Checks zero/nonzero facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_sign_query` | Reads sign facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_msd_query` | Reads magnitude facts for a tiny exact rational. |
| `structural_query_speed/tiny_exact_structural_facts` | Computes full structural facts for a tiny exact rational. |
| `structural_query_speed/pi_zero_status` | Checks zero/nonzero facts for pi. |
| `structural_query_speed/pi_sign_query` | Reads sign facts for pi. |
| `structural_query_speed/pi_msd_query` | Reads magnitude facts for pi. |
| `structural_query_speed/pi_structural_facts` | Computes full structural facts for pi. |
| `structural_query_speed/e_zero_status` | Checks zero/nonzero facts for e. |
| `structural_query_speed/e_sign_query` | Reads sign facts for e. |
| `structural_query_speed/e_msd_query` | Reads magnitude facts for e. |
| `structural_query_speed/e_structural_facts` | Computes full structural facts for e. |
| `structural_query_speed/tau_zero_status` | Checks zero/nonzero facts for tau. |
| `structural_query_speed/tau_sign_query` | Reads sign facts for tau. |
| `structural_query_speed/tau_msd_query` | Reads magnitude facts for tau. |
| `structural_query_speed/tau_structural_facts` | Computes full structural facts for tau. |
| `structural_query_speed/sqrt_two_zero_status` | Checks zero/nonzero facts for sqrt(2). |
| `structural_query_speed/sqrt_two_sign_query` | Reads sign facts for sqrt(2). |
| `structural_query_speed/sqrt_two_msd_query` | Reads magnitude facts for sqrt(2). |
| `structural_query_speed/sqrt_two_structural_facts` | Computes full structural facts for sqrt(2). |
| `structural_query_speed/pi_minus_three_zero_status` | Checks zero/nonzero facts for pi - 3. |
| `structural_query_speed/pi_minus_three_sign_query` | Reads sign facts for pi - 3. |
| `structural_query_speed/pi_minus_three_msd_query` | Reads magnitude facts for pi - 3. |
| `structural_query_speed/pi_minus_three_structural_facts` | Computes full structural facts for pi - 3. |
| `structural_query_speed/dense_expr_zero_status` | Checks zero/nonzero facts for a dense composite expression. |
| `structural_query_speed/dense_expr_sign_query` | Reads sign facts for a dense composite expression. |
| `structural_query_speed/dense_expr_msd_query` | Reads magnitude facts for a dense composite expression. |
| `structural_query_speed/dense_expr_structural_facts` | Computes full structural facts for a dense composite expression. |

### `pure_scalar_algorithm_speed`

Core scalar algorithms that do not require high-precision transcendental approximation.

| Benchmark output | What it measures |
| --- | --- |
| `pure_scalar_algorithm_speed/rational_add` | Adds two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_mul` | Multiplies two nontrivial rational values. |
| `pure_scalar_algorithm_speed/rational_div` | Divides two nontrivial rational values. |
| `pure_scalar_algorithm_speed/real_exact_add` | Adds exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_mul` | Multiplies exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_div` | Divides exact rational-backed `Real` values. |
| `pure_scalar_algorithm_speed/real_exact_sqrt_reduce` | Reduces an exact square-root expression. |
| `pure_scalar_algorithm_speed/real_exact_ln_reduce` | Reduces an exact logarithm of a power of two. |

### `borrowed_op_overhead`

Borrowed versus owned operation overhead for rational and real operands.

| Benchmark output | What it measures |
| --- | --- |
| `borrowed_op_overhead/rational_clone_pair` | Clones two rational values. |
| `borrowed_op_overhead/rational_add_refs` | Adds rational references. |
| `borrowed_op_overhead/rational_add_owned` | Adds owned rational values. |
| `borrowed_op_overhead/real_clone_pair` | Clones two `Real` values. |
| `borrowed_op_overhead/real_add_refs` | Adds `Real` references. |
| `borrowed_op_overhead/real_add_owned` | Adds owned `Real` values. |

### `dense_algebra`

Small dense algebra kernels that stress repeated exact and symbolic operations.

| Benchmark output | What it measures |
| --- | --- |
| `dense_algebra/rational_dot_64` | Computes a 64-element rational dot product. |
| `dense_algebra/rational_matmul_8` | Computes an 8x8 rational matrix multiply. |
| `dense_algebra/real_dot_36` | Computes a 36-element dot product over symbolic `Real` values. |
| `dense_algebra/real_matmul_6` | Computes a 6x6 matrix multiply over symbolic `Real` values. |

<!-- END scalar_micro -->

<!-- BEGIN float_convert -->
## `float_convert`

Covers exact import of floating-point values, including public `Real` conversion overhead.

### `float_convert`

Exact conversion from IEEE-754 floats into `Rational` and `Real` values.

| Benchmark output | What it measures |
| --- | --- |
| `float_convert/f32_normal` | Converts a normal `f32` into an exact `Rational`. |
| `float_convert/f64_normal` | Converts a normal `f64` into an exact `Rational`. |
| `float_convert/f64_binary_fraction` | Converts an exactly representable binary `f64` fraction into `Rational`. |
| `float_convert/f64_subnormal` | Converts a subnormal `f64` into an exact `Rational`. |
| `float_convert/real_f32_normal` | Converts a normal `f32` through the public `Real::try_from` path. |
| `float_convert/real_f64_normal` | Converts a normal `f64` through the public `Real::try_from` path. |
| `float_convert/real_f64_subnormal` | Converts a subnormal `f64` through the public `Real::try_from` path. |

<!-- END float_convert -->

<!-- BEGIN numerical_micro -->
## `numerical_micro`

Low-level `Computable` microbenchmarks for approximation kernels, caches, structural facts, comparisons, and deep evaluator trees.

### `computable_cache`

Cold versus cached approximation of basic `Computable` expressions.

| Benchmark output | What it measures |
| --- | --- |
| `computable_cache/ratio_approx_cold_p128` | Approximates a rational value at p=-128 from a fresh clone. |
| `computable_cache/ratio_approx_cached_p128` | Repeats an already cached rational approximation at p=-128. |
| `computable_cache/pi_approx_cold_p128` | Approximates pi at p=-128 from a fresh clone. |
| `computable_cache/pi_approx_cached_p128` | Repeats an already cached pi approximation at p=-128. |
| `computable_cache/pi_plus_tiny_cold_p128` | Approximates pi plus a tiny exact rational perturbation. |
| `computable_cache/pi_minus_tiny_cold_p128` | Approximates pi minus a tiny exact rational perturbation. |

### `computable_bounds`

Structural sign and bound discovery for deep or perturbed computable trees.

| Benchmark output | What it measures |
| --- | --- |
| `computable_bounds/deep_scaled_product_sign` | Finds the sign of a deep scaled product. |
| `computable_bounds/scaled_square_sign` | Finds the sign of repeated squaring with exact scale factors. |
| `computable_bounds/sqrt_scaled_square_sign` | Finds the sign after taking a square root of a scaled square. |
| `computable_bounds/deep_structural_bound_sign` | Finds sign through repeated multiply/inverse/negate structural transformations. |
| `computable_bounds/deep_structural_bound_sign_cached` | Reads the cached sign of the deep structural-bound chain. |
| `computable_bounds/deep_structural_bound_facts_cached` | Reads cached structural facts for the deep structural-bound chain. |
| `computable_bounds/perturbed_scaled_product_sign` | Finds sign for a deeply scaled value with a tiny perturbation. |
| `computable_bounds/perturbed_scaled_product_sign_until` | Refines sign for the perturbed scaled product only to p=-128. |
| `computable_bounds/pi_minus_tiny_sign` | Finds sign for pi minus a tiny exact rational. |
| `computable_bounds/pi_minus_tiny_sign_cached` | Reads cached sign for pi minus a tiny exact rational. |

### `computable_compare`

Ordering and absolute-comparison shortcuts.

| Benchmark output | What it measures |
| --- | --- |
| `computable_compare/compare_to_opposite_sign` | Compares values with known opposite signs. |
| `computable_compare/compare_to_exact_msd_gap` | Compares values with a large exact magnitude gap. |
| `computable_compare/compare_absolute_exact_rational` | Compares absolute values of exact rationals. |
| `computable_compare/compare_absolute_dominant_add` | Compares a dominant term against the same term plus a tiny addend. |
| `computable_compare/compare_absolute_exact_msd_gap` | Compares absolute values with a large exact magnitude gap. |

### `computable_transcendentals`

Low-level approximation kernels and deep expression-tree stress cases.

| Benchmark output | What it measures |
| --- | --- |
| `computable_transcendentals/legacy_exp_one_p128` | Runs the legacy direct exp series for input 1 at p=-128. |
| `computable_transcendentals/e_constant_cold_p128` | Approximates the shared e constant from a fresh clone. |
| `computable_transcendentals/e_constant_cached_p128` | Repeats a cached approximation of e. |
| `computable_transcendentals/legacy_exp_half_p128` | Runs the legacy direct exp series for input 1/2 at p=-128. |
| `computable_transcendentals/exp_cold_p128` | Approximates exp(7/5) from a fresh clone. |
| `computable_transcendentals/exp_cached_p128` | Repeats a cached exp(7/5) approximation. |
| `computable_transcendentals/exp_large_cold_p128` | Approximates exp(128), exercising large-argument reduction. |
| `computable_transcendentals/exp_half_cold_p128` | Approximates exp(1/2). |
| `computable_transcendentals/exp_near_limit_cold_p128` | Approximates exp near a prescaling threshold. |
| `computable_transcendentals/exp_near_limit_cached_p128` | Repeats a cached near-threshold exp approximation. |
| `computable_transcendentals/exp_zero_cold_p128` | Approximates exp(0). |
| `computable_transcendentals/ln_cold_p128` | Approximates ln(11/7). |
| `computable_transcendentals/ln_cached_p128` | Repeats a cached ln(11/7) approximation. |
| `computable_transcendentals/ln_large_cold_p128` | Approximates ln(1024), exercising large-input reduction. |
| `computable_transcendentals/ln_large_cached_p128` | Repeats a cached ln(1024) approximation. |
| `computable_transcendentals/ln_tiny_cold_p128` | Approximates ln(2^-1024), exercising tiny-input reduction. |
| `computable_transcendentals/ln_near_limit_cold_p128` | Approximates ln near the prescaled-ln limit. |
| `computable_transcendentals/ln_near_limit_cached_p128` | Repeats a cached near-limit ln approximation. |
| `computable_transcendentals/ln_one_cold_p128` | Approximates ln(1). |
| `computable_transcendentals/sqrt_cold_p128` | Approximates sqrt(2). |
| `computable_transcendentals/sqrt_cached_p128` | Repeats a cached sqrt(2) approximation. |
| `computable_transcendentals/sqrt_single_scaled_square_cold_p128` | Builds and approximates sqrt((7*pi/8)^2). |
| `computable_transcendentals/sin_cold_p96` | Approximates sin(7/5). |
| `computable_transcendentals/sin_cached_p96` | Repeats a cached sin(7/5) approximation. |
| `computable_transcendentals/cos_cold_p96` | Approximates cos(7/5). |
| `computable_transcendentals/cos_cached_p96` | Repeats a cached cos(7/5) approximation. |
| `computable_transcendentals/tan_cold_p96` | Approximates tan(7/5). |
| `computable_transcendentals/tan_cached_p96` | Repeats a cached tan(7/5) approximation. |
| `computable_transcendentals/sin_zero_cold_p96` | Approximates sin(0). |
| `computable_transcendentals/cos_zero_cold_p96` | Approximates cos(0). |
| `computable_transcendentals/tan_zero_cold_p96` | Approximates tan(0). |
| `computable_transcendentals/tan_near_half_pi_cold_p96` | Approximates tangent near pi/2. |
| `computable_transcendentals/tan_near_half_pi_cached_p96` | Repeats cached tangent near pi/2. |
| `computable_transcendentals/sin_huge_cold_p96` | Approximates sine of a huge pi multiple plus offset. |
| `computable_transcendentals/cos_huge_cold_p96` | Approximates cosine of a huge pi multiple plus offset. |
| `computable_transcendentals/tan_huge_cold_p96` | Approximates tangent of a huge pi multiple plus offset. |
| `computable_transcendentals/asin_cold_p96` | Approximates a computable asin expression. |
| `computable_transcendentals/asin_cached_p96` | Repeats a cached computable asin approximation. |
| `computable_transcendentals/acos_cold_p96` | Approximates a computable acos expression. |
| `computable_transcendentals/acos_cached_p96` | Repeats a cached computable acos approximation. |
| `computable_transcendentals/atan_cold_p96` | Approximates atan(7/10). |
| `computable_transcendentals/atan_cached_p96` | Repeats a cached atan(7/10) approximation. |
| `computable_transcendentals/atan_large_cold_p96` | Approximates atan(8), exercising argument reduction. |
| `computable_transcendentals/asin_zero_cold_p96` | Approximates asin(0) expression. |
| `computable_transcendentals/atan_zero_cold_p96` | Approximates atan(0). |
| `computable_transcendentals/asinh_cold_p128` | Approximates a computable asinh expression. |
| `computable_transcendentals/asinh_cached_p128` | Repeats a cached computable asinh approximation. |
| `computable_transcendentals/acosh_cold_p128` | Approximates a computable acosh expression. |
| `computable_transcendentals/acosh_cached_p128` | Repeats a cached computable acosh approximation. |
| `computable_transcendentals/atanh_cold_p128` | Approximates a computable atanh expression. |
| `computable_transcendentals/atanh_cached_p128` | Repeats a cached computable atanh approximation. |
| `computable_transcendentals/asinh_zero_cold_p128` | Approximates asinh(0) expression. |
| `computable_transcendentals/atanh_zero_cold_p128` | Approximates atanh(0) expression. |
| `computable_transcendentals/deep_add_chain_cold_p128` | Approximates a 5000-node addition chain. |
| `computable_transcendentals/deep_multiply_chain_cold_p128` | Approximates a 5000-node multiply-by-one chain. |
| `computable_transcendentals/deep_multiply_identity_chain_cold_p128` | Approximates a deep identity multiplication chain around pi. |
| `computable_transcendentals/deep_scaled_product_chain_cold_p128` | Approximates a deep product of exact scale factors. |
| `computable_transcendentals/perturbed_scaled_product_chain_cold_p128` | Approximates a deep scaled product with a tiny perturbation. |
| `computable_transcendentals/scaled_square_chain_cold_p128` | Approximates repeated squaring of a scaled irrational. |
| `computable_transcendentals/asymmetric_product_bad_order_cold_p128` | Approximates an asymmetric product order stress case. |
| `computable_transcendentals/sqrt_scaled_square_chain_cold_p128` | Approximates sqrt of a scaled-square chain. |
| `computable_transcendentals/warmed_zero_product_cold_p128` | Approximates a product involving a warmed zero sum. |
| `computable_transcendentals/inverse_scaled_product_chain_cold_p128` | Approximates the inverse of a deep scaled product. |
| `computable_transcendentals/deep_inverse_pair_chain_cold_p128` | Approximates a chain of inverse(inverse(x)) pairs. |
| `computable_transcendentals/deep_negated_square_chain_cold_p128` | Approximates repeated negate-square-sqrt transformations. |
| `computable_transcendentals/deep_negative_one_product_chain_cold_p128` | Approximates repeated multiplication by -1. |
| `computable_transcendentals/deep_half_product_chain_cold_p128` | Approximates repeated multiplication by 1/2. |
| `computable_transcendentals/deep_half_square_chain_cold_p128` | Approximates repeated squaring after scaling by 1/2. |
| `computable_transcendentals/deep_sqrt_square_chain_cold_p128` | Approximates repeated sqrt-square simplification. |
| `computable_transcendentals/inverse_half_product_chain_cold_p128` | Approximates the inverse of a deep half-product chain. |

<!-- END numerical_micro -->

<!-- BEGIN library_perf -->
## `library_perf`

Library-level Criterion benchmarks for public `Rational`, `Real`, and `Simple` behavior.

### `real_format`

Formatting costs for important irrational `Real` values.

| Benchmark output | What it measures |
| --- | --- |
| `real_format/pi_lower_exp_32` | Formats pi with 32 digits in lower-exponential form. |
| `real_format/pi_display_alt_32` | Formats pi with alternate decimal display at 32 digits. |
| `real_format/sqrt_two_display_alt_32` | Formats sqrt(2) with alternate decimal display at 32 digits. |

### `real_constants`

Construction cost for shared mathematical constants.

| Benchmark output | What it measures |
| --- | --- |
| `real_constants/pi` | Constructs the symbolic pi value. |
| `real_constants/e` | Constructs the symbolic Euler constant value. |

### `simple`

Parser and evaluator costs for the `Simple` expression language.

| Benchmark output | What it measures |
| --- | --- |
| `simple/parse_nested` | Parses a nested expression with powers, trig, and constants. |
| `simple/eval_nested` | Evaluates a parsed mixed symbolic/numeric expression. |
| `simple/eval_constants` | Evaluates repeated built-in constants. |
| `simple/eval_exact` | Evaluates a rational-only expression through exact shortcuts. |
| `simple/eval_nested_exact` | Evaluates a nested rational-only expression through exact shortcuts. |

### `real_powi`

Integer exponentiation for exact and irrational `Real` values.

| Benchmark output | What it measures |
| --- | --- |
| `real_powi/exact_17` | Raises an exact rational-backed `Real` to the 17th power. |
| `real_powi/irrational_17` | Raises sqrt(3) to the 17th power with symbolic simplification. |

### `rational_powi`

Integer exponentiation for `Rational`.

| Benchmark output | What it measures |
| --- | --- |
| `rational_powi/exact_17` | Raises a rational value to the 17th power. |

### `real_exact_trig`

Exact and symbolic trig construction for known pi multiples.

| Benchmark output | What it measures |
| --- | --- |
| `real_exact_trig/sin_pi_6` | Computes sin(pi/6) via exact shortcut. |
| `real_exact_trig/cos_pi_3` | Computes cos(pi/3) via exact shortcut. |
| `real_exact_trig/tan_pi_5` | Builds tan(pi/5), a nontrivial symbolic tangent. |

### `real_general_trig`

General trig construction for irrational arguments.

| Benchmark output | What it measures |
| --- | --- |
| `real_general_trig/tan_sqrt_2` | Builds tan(sqrt(2)). |
| `real_general_trig/tan_pi_sqrt_2_over_5` | Builds tangent of an irrational multiple of pi. |

### `real_exact_inverse_trig`

Exact inverse trig shortcuts and symbolic inverse trig recognition.

| Benchmark output | What it measures |
| --- | --- |
| `real_exact_inverse_trig/asin_1_2` | Recognizes asin(1/2) as pi/6. |
| `real_exact_inverse_trig/asin_minus_1_2` | Recognizes asin(-1/2) as -pi/6. |
| `real_exact_inverse_trig/asin_sqrt_2_over_2` | Recognizes asin(sqrt(2)/2) as pi/4. |
| `real_exact_inverse_trig/asin_sin_pi_5` | Inverts a symbolic sin(pi/5). |
| `real_exact_inverse_trig/acos_1` | Recognizes acos(1) as zero. |
| `real_exact_inverse_trig/acos_minus_1` | Recognizes acos(-1) as pi. |
| `real_exact_inverse_trig/acos_1_2` | Recognizes acos(1/2) as pi/3. |
| `real_exact_inverse_trig/atan_1` | Recognizes atan(1) as pi/4. |
| `real_exact_inverse_trig/atan_sqrt_3_over_3` | Recognizes atan(sqrt(3)/3) as pi/6. |
| `real_exact_inverse_trig/atan_tan_pi_5` | Inverts a symbolic tan(pi/5). |

### `real_general_inverse_trig`

General inverse trig construction, domain errors, and atan range reduction.

| Benchmark output | What it measures |
| --- | --- |
| `real_general_inverse_trig/asin_7_10` | Builds asin(7/10) through the rational-specialized path. |
| `real_general_inverse_trig/asin_sqrt_2_over_3` | Builds asin(sqrt(2)/3) through the general path. |
| `real_general_inverse_trig/acos_7_10` | Builds acos(7/10) through the rational-specialized asin path. |
| `real_general_inverse_trig/acos_sqrt_2_over_3` | Builds acos(sqrt(2)/3) through the general path. |
| `real_general_inverse_trig/asin_11_10_error` | Rejects rational asin input outside [-1, 1]. |
| `real_general_inverse_trig/acos_11_10_error` | Rejects rational acos input outside [-1, 1]. |
| `real_general_inverse_trig/atan_8` | Builds atan(8), exercising large-argument reduction. |
| `real_general_inverse_trig/atan_sqrt_2` | Builds atan(sqrt(2)). |

### `real_inverse_hyperbolic`

Inverse hyperbolic construction, exact exits, stable ln1p forms, and domain errors.

| Benchmark output | What it measures |
| --- | --- |
| `real_inverse_hyperbolic/asinh_0` | Recognizes asinh(0) as zero. |
| `real_inverse_hyperbolic/asinh_1_2` | Builds asinh(1/2) through the stable moderate-input path. |
| `real_inverse_hyperbolic/asinh_sqrt_2` | Builds asinh(sqrt(2)) without cancellation-prone log construction. |
| `real_inverse_hyperbolic/asinh_minus_1_2` | Uses odd symmetry for negative asinh input. |
| `real_inverse_hyperbolic/asinh_1_000_000` | Builds asinh for a large positive rational. |
| `real_inverse_hyperbolic/acosh_1` | Recognizes acosh(1) as zero. |
| `real_inverse_hyperbolic/acosh_2` | Builds acosh(2) through the stable moderate-input path. |
| `real_inverse_hyperbolic/acosh_sqrt_2` | Builds acosh(sqrt(2)) through square-root domain specialization. |
| `real_inverse_hyperbolic/acosh_1_000_000` | Builds acosh for a large positive rational. |
| `real_inverse_hyperbolic/atanh_0` | Recognizes atanh(0) as zero. |
| `real_inverse_hyperbolic/atanh_1_2` | Builds exact-rational atanh(1/2). |
| `real_inverse_hyperbolic/atanh_minus_1_2` | Builds exact-rational atanh(-1/2). |
| `real_inverse_hyperbolic/atanh_9_10` | Builds exact-rational atanh near the upper domain boundary. |
| `real_inverse_hyperbolic/atanh_1_error` | Rejects atanh(1) at the rational domain boundary. |

### `simple_inverse_functions`

Parsed/evaluated inverse trig and inverse hyperbolic expressions that should succeed.

| Benchmark output | What it measures |
| --- | --- |
| `simple_inverse_functions/asin_1_2` | Evaluates `(asin 1/2)`. |
| `simple_inverse_functions/acos_1_2` | Evaluates `(acos 1/2)`. |
| `simple_inverse_functions/atan_1` | Evaluates `(atan 1)`. |
| `simple_inverse_functions/asin_general` | Evaluates `(asin 7/10)`. |
| `simple_inverse_functions/acos_general` | Evaluates `(acos 7/10)`. |
| `simple_inverse_functions/atan_general` | Evaluates `(atan 8)`. |
| `simple_inverse_functions/asinh_1_2` | Evaluates `(asinh 1/2)`. |
| `simple_inverse_functions/asinh_sqrt_2` | Evaluates `(asinh (sqrt 2))`. |
| `simple_inverse_functions/acosh_2` | Evaluates `(acosh 2)`. |
| `simple_inverse_functions/acosh_sqrt_2` | Evaluates `(acosh (sqrt 2))`. |
| `simple_inverse_functions/atanh_1_2` | Evaluates `(atanh 1/2)`. |
| `simple_inverse_functions/atanh_minus_1_2` | Evaluates `(atanh -1/2)`. |

### `simple_inverse_error_functions`

Parsed/evaluated inverse function expressions that should fail quickly with `NotANumber`.

| Benchmark output | What it measures |
| --- | --- |
| `simple_inverse_error_functions/asin_11_10` | Rejects `(asin 11/10)`. |
| `simple_inverse_error_functions/acos_sqrt_2` | Rejects `(acos (sqrt 2))`. |
| `simple_inverse_error_functions/acosh_0` | Rejects `(acosh 0)`. |
| `simple_inverse_error_functions/acosh_minus_2` | Rejects `(acosh -2)`. |
| `simple_inverse_error_functions/atanh_1` | Rejects `(atanh 1)`. |
| `simple_inverse_error_functions/atanh_sqrt_2` | Rejects `(atanh (sqrt 2))`. |

### `real_exact_ln`

Exact logarithm construction and simplification for rational inputs.

| Benchmark output | What it measures |
| --- | --- |
| `real_exact_ln/ln_1024` | Recognizes ln(1024) as 10 ln(2). |
| `real_exact_ln/ln_1_8` | Recognizes ln(1/8) as -3 ln(2). |
| `real_exact_ln/ln_1000` | Simplifies ln(1000) via small integer logarithm factors. |

### `real_exact_exp_log10`

Exact inverse relationships among exp, ln, and log10.

| Benchmark output | What it measures |
| --- | --- |
| `real_exact_exp_log10/exp_ln_1000` | Simplifies exp(ln(1000)) back to 1000. |
| `real_exact_exp_log10/exp_ln_1_8` | Simplifies exp(ln(1/8)) back to 1/8. |
| `real_exact_exp_log10/log10_1000` | Recognizes log10(1000) as 3. |
| `real_exact_exp_log10/log10_1_1000` | Recognizes log10(1/1000) as -3. |

<!-- END library_perf -->
