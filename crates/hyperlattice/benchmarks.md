# Benchmarks

Run the Criterion benchmark suite:

```sh
cargo bench --bench mathbench
```

Refresh this file from existing Criterion estimates without rerunning the full suite:

```sh
cargo bench --bench mathbench -- --update-benchmarks-md
```

The `mathbench` suite benchmarks both crate backends and writes this file from Criterion's median estimates after a real benchmark run. The `astro-float 128` and `numerica128` comparison columns run at 128-bit precision, while the `symbolica` column exercises Symbolica's symbolic expression engine. Missing cells mean that the corresponding estimate was not present in `target/criterion` when this file was generated, or that the external library does not expose a directly comparable operation in this suite.

Each benchmarked operation rotates through adversarial inputs for its valid domain: near-zero values, large and tiny magnitudes, cancellation-prone vectors, near-singular matrices, range-reduction-heavy trigonometric arguments, and boundary-adjacent inverse trigonometric and inverse hyperbolic values.

## Operation Coverage

- Scalar construction/constants, arithmetic, reciprocal, powers, exponentials, logarithms, square root, trigonometric and hyperbolic functions, inverse helpers, zero-status checks, and abort-aware variants.
- Complex construction/constants, conjugate, norm squared, reciprocal, powers, checked division, scalar conversion, arithmetic, and real scalar division.
- Vector construction, zero, dot product, magnitude, normalization, vector/vector arithmetic, vector/scalar arithmetic, scalar division, and checked/abort-aware variants for 3D and 4D vectors.
- Matrix construction, zero, identity, transpose, determinant, inverse, reciprocal, powers, matrix/matrix arithmetic, matrix/scalar arithmetic, matrix/vector transformation, scalar division, matrix division, and checked/abort-aware variants for 3x3 and 4x4 matrices.
- Borrowed API operator coverage for scalar, vector, matrix, matrix/vector, and complex reference combinations.

## Benchmark Results

The following Criterion median estimates were collected on an AMD Ryzen 7 5800X3D on Fedora. Values are formatted to two digits after the decimal.

### Scalar Operations

#### Scalar Trigonometric And Inverse Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `sin 0.1` | - | - | - | - | - | - | - | - | - | - |
| `cos 0.1` | - | - | - | - | - | - | - | - | - | - |
| `sin 1.23456789` | - | - | - | - | - | - | - | - | - | - |
| `cos 1.23456789` | - | - | - | - | - | - | - | - | - | - |
| `sin 1e6` | - | - | - | - | - | - | - | - | - | - |
| `cos 1e6` | - | - | - | - | - | - | - | - | - | - |
| `sin 1e30` | - | - | - | - | - | - | - | - | - | - |
| `cos 1e30` | - | - | - | - | - | - | - | - | - | - |
| `sin pi_7` | - | - | - | - | - | - | - | - | - | - |
| `cos pi_7` | - | - | - | - | - | - | - | - | - | - |
| `sin 1000pi_eps` | - | - | - | - | - | - | - | - | - | - |
| `cos 1000pi_eps` | - | - | - | - | - | - | - | - | - | - |
| `asin 0.5` | - | - | - | - | - | - | - | - | - | - |
| `acos 0.5` | - | - | - | - | - | - | - | - | - | - |
| `atanh 0.5` | - | - | - | - | - | - | - | - | - | - |
| `asin neg_0.999999` | - | - | - | - | - | - | - | - | - | - |
| `acos neg_0.999999` | - | - | - | - | - | - | - | - | - | - |
| `atanh neg_0.999999` | - | - | - | - | - | - | - | - | - | - |
| `asin 0.999999` | - | - | - | - | - | - | - | - | - | - |
| `acos 0.999999` | - | - | - | - | - | - | - | - | - | - |
| `atanh 0.999999` | - | - | - | - | - | - | - | - | - | - |
| `asin 1e-12` | - | - | - | - | - | - | - | - | - | - |
| `acos 1e-12` | - | - | - | - | - | - | - | - | - | - |
| `atanh 1e-12` | - | - | - | - | - | - | - | - | - | - |
| `atan 0.5` | - | - | - | - | - | - | - | - | - | - |
| `asinh 0.5` | - | - | - | - | - | - | - | - | - | - |
| `atan neg_1e-12` | - | - | - | - | - | - | - | - | - | - |
| `asinh neg_1e-12` | - | - | - | - | - | - | - | - | - | - |
| `atan 1e6` | - | - | - | - | - | - | - | - | - | - |
| `asinh 1e6` | - | - | - | - | - | - | - | - | - | - |
| `atan neg_1e6` | - | - | - | - | - | - | - | - | - | - |
| `asinh neg_1e6` | - | - | - | - | - | - | - | - | - | - |
| `acosh 9` | - | - | - | - | - | - | - | - | - | - |
| `acosh 1_plus_1e-12` | - | - | - | - | - | - | - | - | - | - |
| `acosh 1e6` | - | - | - | - | - | - | - | - | - | - |
| `acosh e` | - | - | - | - | - | - | - | - | - | - |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | - | - | - | - | - | - | - | - | - | - |
| `one` | - | - | - | - | - | - | - | - | - | - |
| `e` | - | - | - | - | - | - | - | - | - | - |
| `pi` | - | - | - | - | - | - | - | - | - | - |
| `tau` | - | - | - | - | - | - | - | - | - | - |
| `add` | - | - | - | - | - | - | - | - | - | - |
| `sub` | - | - | - | - | - | - | - | - | - | - |
| `neg` | - | - | - | - | - | - | - | - | - | - |
| `mul` | - | - | - | - | - | - | - | - | - | - |
| `div` | - | - | - | - | - | - | - | - | - | - |
| `reciprocal` | - | - | - | - | - | - | - | - | - | - |
| `reciprocal checked` | - | - | - | - | - | - | - | - | - | - |
| `reciprocal checked abort` | - | - | - | - | - | - | - | - | - | - |
| `pow` | - | - | - | - | - | - | - | - | - | - |
| `powi` | - | - | - | - | - | - | - | - | - | - |
| `exp` | - | - | - | - | - | - | - | - | - | - |
| `ln` | - | - | - | - | - | - | - | - | - | - |
| `log10` | - | - | - | - | - | - | - | - | - | - |
| `log10 abort` | - | - | - | - | - | - | - | - | - | - |
| `sqrt` | - | - | - | - | - | - | - | - | - | - |
| `sin` | - | - | - | - | - | - | - | - | - | - |
| `cos` | - | - | - | - | - | - | - | - | - | - |
| `tan` | - | - | - | - | - | - | - | - | - | - |
| `sinh` | - | - | - | - | - | - | - | - | - | - |
| `cosh` | - | - | - | - | - | - | - | - | - | - |
| `tanh` | - | - | - | - | - | - | - | - | - | - |
| `asin` | - | - | - | - | - | - | - | - | - | - |
| `asin abort` | - | - | - | - | - | - | - | - | - | - |
| `acos` | - | - | - | - | - | - | - | - | - | - |
| `acos abort` | - | - | - | - | - | - | - | - | - | - |
| `atan` | - | - | - | - | - | - | - | - | - | - |
| `atan abort` | - | - | - | - | - | - | - | - | - | - |
| `asinh` | - | - | - | - | - | - | - | - | - | - |
| `asinh abort` | - | - | - | - | - | - | - | - | - | - |
| `acosh` | - | - | - | - | - | - | - | - | - | - |
| `acosh abort` | - | - | - | - | - | - | - | - | - | - |
| `atanh` | - | - | - | - | - | - | - | - | - | - |
| `atanh abort` | - | - | - | - | - | - | - | - | - | - |
| `zero status` | - | - | - | - | - | - | - | - | - | - |
| `zero status abort` | - | - | - | - | - | - | - | - | - | - |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | - | - | - | - | - | - | - | - | - | - |
| `one` | - | - | - | - | - | - | - | - | - | - |
| `i` | - | - | - | - | - | - | - | - | - | - |
| `free i` | - | - | - | - | - | - | - | - | - | - |
| `conjugate` | - | - | - | - | - | - | - | - | - | - |
| `norm squared` | - | - | - | - | - | - | - | - | - | - |
| `reciprocal` | - | - | - | - | - | - | - | - | - | - |
| `reciprocal checked` | - | - | - | - | - | - | - | - | - | - |
| `powi` | - | - | - | - | - | - | - | - | - | - |
| `powi checked` | - | - | - | - | - | - | - | - | - | - |
| `div checked` | - | - | - | - | - | - | - | - | - | - |
| `div real checked` | - | - | - | - | - | - | - | - | - | - |
| `from scalar` | - | - | - | - | - | - | - | - | - | - |
| `add` | - | - | - | - | - | - | - | - | - | - |
| `sub` | - | - | - | - | - | - | - | - | - | - |
| `neg` | - | - | - | - | - | - | - | - | - | - |
| `mul` | - | - | - | - | - | - | - | - | - | - |
| `div` | - | - | - | - | - | - | - | - | - | - |
| `div real` | - | - | - | - | - | - | - | - | - | - |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | - | - | - | - | - | - | - | - | - | - |
| `vec3 magnitude` | - | - | - | - | - | - | - | - | - | - |
| `vec3 normalize` | - | - | - | - | - | - | - | - | - | - |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | - | - | - | - | - | - | - | - | - | - |
| `vec3 zero` | - | - | - | - | - | - | - | - | - | - |
| `vec3 dot abort` | - | - | - | - | - | - | - | - | - | - |
| `vec3 magnitude abort` | - | 8.04 us | 7.39 us | - | - | - | - | - | - | - |
| `vec3 normalize checked` | - | 12.84 us | 9.50 us | - | - | - | - | - | - | - |
| `vec3 normalize checked abort` | - | 13.03 us | 10.40 us | - | - | - | - | - | - | - |
| `vec3 div scalar checked` | - | - | - | - | - | - | - | - | - | - |
| `vec3 div scalar checked abort` | - | - | - | - | - | - | - | - | - | - |
| `vec3 add` | - | - | - | - | - | - | - | - | - | - |
| `vec3 add scalar` | - | - | - | - | - | - | - | - | - | - |
| `vec3 sub` | - | - | - | - | - | - | - | - | - | - |
| `vec3 sub scalar` | - | - | - | - | - | - | - | - | - | - |
| `vec3 neg` | - | - | - | - | - | - | - | - | - | - |
| `vec3 mul scalar` | - | - | - | - | - | - | - | - | - | - |
| `vec3 div scalar` | - | - | - | - | - | - | - | - | - | - |
| `vec4 dot` | - | - | - | - | - | - | - | - | - | - |
| `vec4 magnitude` | - | 7.05 us | 5.95 us | - | - | - | - | - | - | - |
| `vec4 normalize` | - | 12.27 us | 8.44 us | - | - | - | - | - | - | - |
| `vec4 add` | - | - | - | - | - | - | - | - | - | - |
| `vec4 add scalar` | - | - | - | - | - | - | - | - | - | - |
| `vec4 sub` | - | - | - | - | - | - | - | - | - | - |
| `vec4 sub scalar` | - | - | - | - | - | - | - | - | - | - |
| `vec4 neg` | - | - | - | - | - | - | - | - | - | - |
| `vec4 mul scalar` | - | - | - | - | - | - | - | - | - | - |
| `vec4 div scalar` | - | - | - | - | - | - | - | - | - | - |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | - | - | - | - | - | - | - | - | - | - |
| `mat3 inverse` | - | - | - | - | - | - | - | - | - | - |
| `mat3 mul mat3` | - | - | - | - | - | - | - | - | - | - |
| `mat3 transform vec3` | - | - | - | - | - | - | - | - | - | - |
| `mat4 determinant` | - | - | - | - | - | - | - | - | - | - |
| `mat4 inverse` | - | - | - | - | - | - | - | - | - | - |
| `mat4 mul mat4` | - | - | - | - | - | - | - | - | - | - |
| `mat4 transform vec4` | - | - | - | - | - | - | - | - | - | - |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | - | - | - | - | - | - | - | - | - | - |
| `mat3 zero` | - | - | - | - | - | - | - | - | - | - |
| `mat3 identity` | - | - | - | - | - | - | - | - | - | - |
| `mat3 transpose` | - | - | - | - | - | - | - | - | - | - |
| `mat3 reciprocal` | - | - | - | - | - | - | - | - | - | - |
| `mat3 reciprocal checked` | - | - | - | - | - | - | - | - | - | - |
| `mat3 inverse checked` | - | - | - | - | - | - | - | - | - | - |
| `mat3 inverse checked abort` | - | - | - | - | - | - | - | - | - | - |
| `mat3 powi` | - | - | - | - | - | - | - | - | - | - |
| `mat3 powi checked` | - | - | - | - | - | - | - | - | - | - |
| `mat3 powi checked abort` | - | - | - | - | - | - | - | - | - | - |
| `mat3 div scalar checked` | - | - | - | - | - | - | - | - | - | - |
| `mat3 div scalar checked abort` | - | - | - | - | - | - | - | - | - | - |
| `mat3 div matrix checked` | - | - | - | - | - | - | - | - | - | - |
| `mat3 div matrix checked abort` | - | - | - | - | - | - | - | - | - | - |
| `mat3 add` | - | - | - | - | - | - | - | - | - | - |
| `mat3 add scalar` | - | - | - | - | - | - | - | - | - | - |
| `mat3 sub` | - | - | - | - | - | - | - | - | - | - |
| `mat3 sub scalar` | - | - | - | - | - | - | - | - | - | - |
| `mat3 neg` | - | - | - | - | - | - | - | - | - | - |
| `mat3 mul scalar` | - | - | - | - | - | - | - | - | - | - |
| `mat3 div scalar` | - | - | - | - | - | - | - | - | - | - |
| `mat3 div matrix` | - | - | - | - | - | - | - | - | - | - |
| `mat3 bitxor` | - | - | - | - | - | - | - | - | - | - |
| `mat4 zero` | - | - | - | - | - | - | - | - | - | - |
| `mat4 identity` | - | - | - | - | - | - | - | - | - | - |
| `mat4 transpose` | - | - | - | - | - | - | - | - | - | - |
| `mat4 reciprocal` | - | - | - | - | - | - | - | - | - | - |
| `mat4 reciprocal checked` | - | - | - | - | - | - | - | - | - | - |
| `mat4 powi` | - | - | - | - | - | - | - | - | - | - |
| `mat4 powi checked` | - | - | - | - | - | - | - | - | - | - |
| `mat4 add` | - | - | - | - | - | - | - | - | - | - |
| `mat4 add scalar` | - | - | - | - | - | - | - | - | - | - |
| `mat4 sub` | - | - | - | - | - | - | - | - | - | - |
| `mat4 sub scalar` | - | - | - | - | - | - | - | - | - | - |
| `mat4 neg` | - | - | - | - | - | - | - | - | - | - |
| `mat4 mul scalar` | - | - | - | - | - | - | - | - | - | - |
| `mat4 div scalar` | - | - | - | - | - | - | - | - | - | - |
| `mat4 div matrix` | - | - | - | - | - | - | - | - | - | - |
| `mat4 bitxor` | - | - | - | - | - | - | - | - | - | - |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 9.20 ns | 669.36 ns | 656.08 ns | - | - | - | 72.77x | - | - | - |
| `scalar add ref_owned` | 5.46 ns | 676.19 ns | 693.56 ns | - | - | - | 123.83x | - | - | - |
| `scalar add refs` | 5.62 ns | 583.88 ns | 584.30 ns | - | - | - | 103.90x | - | - | - |
| `scalar sub owned_ref` | 9.02 ns | 972.90 ns | 723.56 ns | - | - | - | 107.91x | - | - | - |
| `scalar sub ref_owned` | 5.74 ns | 988.67 ns | 769.56 ns | - | - | - | 172.19x | - | - | - |
| `scalar sub refs` | 5.65 ns | 876.12 ns | 648.25 ns | - | - | - | 155.14x | - | - | - |
| `scalar mul owned_ref` | 9.87 ns | 745.11 ns | 601.96 ns | - | - | - | 75.53x | - | - | - |
| `scalar mul ref_owned` | 6.28 ns | 725.57 ns | 618.58 ns | - | - | - | 115.54x | - | - | - |
| `scalar mul refs` | 6.56 ns | 378.64 ns | 374.24 ns | - | - | - | 57.71x | - | - | - |
| `scalar div owned_ref` | 15.72 ns | 635.95 ns | 501.19 ns | - | - | - | 40.46x | - | - | - |
| `scalar div ref_owned` | 15.92 ns | 695.08 ns | 560.99 ns | - | - | - | 43.67x | - | - | - |
| `scalar div refs` | 6.92 ns | 283.60 ns | 159.23 ns | - | - | - | 40.97x | - | - | - |
| `vec3 add refs` | - | 1.55 us | 1.43 us | - | - | - | - | - | - | - |
| `vec3 sub refs` | - | - | - | - | - | - | - | - | - | - |
| `vec3 neg ref` | - | - | - | - | - | - | - | - | - | - |
| `vec3 add_scalar_ref` | - | - | - | - | - | - | - | - | - | - |
| `vec3 sub_scalar_ref` | - | - | - | - | - | - | - | - | - | - |
| `vec3 mul_scalar_ref` | - | 2.11 us | 2.27 us | - | - | - | - | - | - | - |
| `vec3 div_scalar_ref` | - | - | - | - | - | - | - | - | - | - |
| `vec4 add refs` | - | - | - | - | - | - | - | - | - | - |
| `vec4 sub refs` | - | - | - | - | - | - | - | - | - | - |
| `vec4 neg ref` | - | - | - | - | - | - | - | - | - | - |
| `vec4 add_scalar_ref` | - | - | - | - | - | - | - | - | - | - |
| `vec4 sub_scalar_ref` | - | - | - | - | - | - | - | - | - | - |
| `vec4 mul_scalar_ref` | - | 2.55 us | 2.55 us | - | - | - | - | - | - | - |
| `vec4 div_scalar_ref` | - | - | - | - | - | - | - | - | - | - |
| `mat3 add refs` | - | 3.43 us | 4.16 us | - | - | - | - | - | - | - |
| `mat3 sub refs` | - | - | - | - | - | - | - | - | - | - |
| `mat3 mul refs` | - | 24.95 us | 32.48 us | - | - | - | - | - | - | - |
| `mat3 div refs` | - | - | - | - | - | - | - | - | - | - |
| `mat3 neg ref` | - | - | - | - | - | - | - | - | - | - |
| `mat3 add_scalar_ref` | - | - | - | - | - | - | - | - | - | - |
| `mat3 sub_scalar_ref` | - | - | - | - | - | - | - | - | - | - |
| `mat3 mul_scalar_ref` | - | - | - | - | - | - | - | - | - | - |
| `mat3 div_scalar_ref` | - | - | - | - | - | - | - | - | - | - |
| `mat4 add refs` | - | - | - | - | - | - | - | - | - | - |
| `mat4 sub refs` | - | - | - | - | - | - | - | - | - | - |
| `mat4 mul refs` | - | 30.14 us | 33.91 us | - | - | - | - | - | - | - |
| `mat4 div refs` | - | - | - | - | - | - | - | - | - | - |
| `mat4 neg ref` | - | - | - | - | - | - | - | - | - | - |
| `mat4 add_scalar_ref` | - | - | - | - | - | - | - | - | - | - |
| `mat4 sub_scalar_ref` | - | - | - | - | - | - | - | - | - | - |
| `mat4 mul_scalar_ref` | - | - | - | - | - | - | - | - | - | - |
| `mat4 div_scalar_ref` | - | - | - | - | - | - | - | - | - | - |
| `mat3 transform_vec refs` | - | 10.78 us | 12.06 us | - | - | - | - | - | - | - |
| `mat4 transform_vec refs` | - | 13.31 us | 11.41 us | - | - | - | - | - | - | - |
| `complex add refs` | - | 660.95 ns | 711.39 ns | - | - | - | - | - | - | - |
| `complex sub refs` | - | - | - | - | - | - | - | - | - | - |
| `complex mul refs` | - | 3.01 us | 3.23 us | - | - | - | - | - | - | - |
| `complex div refs` | - | 6.43 us | 6.80 us | - | - | - | - | - | - | - |
| `complex neg ref` | - | - | - | - | - | - | - | - | - | - |
| `complex div_real_ref` | - | 794.70 ns | 790.07 ns | - | - | - | - | - | - | - |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | - |
| `astro sin 160` | - |
| `astro sin 192` | - |
| `astro sin 256` | - |
