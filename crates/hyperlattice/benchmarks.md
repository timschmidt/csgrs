# Benchmarks

Run the Criterion benchmark suite:

```sh
cargo bench --bench mathbench
```

Run dispatch path tracing separately:

```sh
cargo bench --bench mathbench --features hyperreal-dispatch-trace -- --write-dispatch-trace-md
```

Refresh this file from existing Criterion estimates without rerunning the full suite:

```sh
cargo bench --bench mathbench -- --update-benchmarks-md
```

The `mathbench` suite benchmarks the Real-primary crate path and writes this file from Criterion's median estimates after a real benchmark run. The exact-dyadic column imports each finite binary64 fixture as its exact dyadic rational value; it does not perform binary64 arithmetic. The explicit-rational column constructs the corresponding authored rational inputs directly. The `numerica128` comparison column runs at 128-bit precision, `gmp_mpfr128` uses Rug's GMP/MPFR stack with 128-bit MPFR scalars, and the `symbolica` column exercises Symbolica's symbolic expression engine. Missing cells mean that the corresponding estimate was not present in `target/criterion` when this file was generated.

Each benchmarked operation rotates through adversarial inputs for its valid domain: near-zero values, large and tiny magnitudes, cancellation-prone vectors, near-singular matrices, range-reduction-heavy trigonometric arguments, and boundary-adjacent inverse trigonometric and inverse hyperbolic values.

## Operation Coverage

- Real construction/constants, arithmetic, reciprocal, powers, exponentials, logarithms, square root, trigonometric and hyperbolic functions, inverse helpers, zero-status checks, and abort-aware variants.
- Complex construction/constants, conjugate, norm squared, reciprocal, powers, checked division, scalar conversion, arithmetic, and real scalar division.
- Vector construction, zero, dot product, magnitude, normalization, vector/vector arithmetic, vector/scalar arithmetic, scalar division, and checked/abort-aware variants for 3D and 4D vectors.
- Matrix construction, zero, identity, transpose, determinant, inverse, reciprocal, powers, matrix/matrix arithmetic, matrix/scalar arithmetic, matrix/vector transformation, scalar division, matrix division, and checked/abort-aware variants for 3x3 and 4x4 matrices.
- Borrowed API operator coverage for scalar, vector, matrix, matrix/vector, and complex reference combinations.

## Benchmark Results

The following Criterion median estimates were collected on an AMD Ryzen 7 5800X3D on Fedora. Values are formatted to two digits after the decimal.

### Real Operations

#### Real Trigonometric And Inverse Comparisons

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `sin 0.1` | - | - | - | - | - | - | - | - |
| `cos 0.1` | - | - | - | - | - | - | - | - |
| `sin 1.23456789` | - | - | - | - | - | - | - | - |
| `cos 1.23456789` | - | - | - | - | - | - | - | - |
| `sin 1e6` | - | - | - | - | - | - | - | - |
| `cos 1e6` | - | - | - | - | - | - | - | - |
| `sin 1e30` | - | - | - | - | - | - | - | - |
| `cos 1e30` | - | - | - | - | - | - | - | - |
| `sin pi_7` | - | - | - | - | - | - | - | - |
| `cos pi_7` | - | - | - | - | - | - | - | - |
| `sin 1000pi_eps` | - | - | - | - | - | - | - | - |
| `cos 1000pi_eps` | - | - | - | - | - | - | - | - |
| `asin 0.5` | - | - | - | - | - | - | - | - |
| `acos 0.5` | - | - | - | - | - | - | - | - |
| `atanh 0.5` | - | - | - | - | - | - | - | - |
| `asin neg_0.999999` | - | - | - | - | - | - | - | - |
| `acos neg_0.999999` | - | - | - | - | - | - | - | - |
| `atanh neg_0.999999` | - | - | - | - | - | - | - | - |
| `asin 0.999999` | - | - | - | - | - | - | - | - |
| `acos 0.999999` | - | - | - | - | - | - | - | - |
| `atanh 0.999999` | - | - | - | - | - | - | - | - |
| `asin 1e-12` | - | - | - | - | - | - | - | - |
| `acos 1e-12` | - | - | - | - | - | - | - | - |
| `atanh 1e-12` | - | - | - | - | - | - | - | - |
| `atan 0.5` | - | - | - | - | - | - | - | - |
| `asinh 0.5` | - | - | - | - | - | - | - | - |
| `atan neg_1e-12` | - | - | - | - | - | - | - | - |
| `asinh neg_1e-12` | - | - | - | - | - | - | - | - |
| `atan 1e6` | - | - | - | - | - | - | - | - |
| `asinh 1e6` | - | - | - | - | - | - | - | - |
| `atan neg_1e6` | - | - | - | - | - | - | - | - |
| `asinh neg_1e6` | - | - | - | - | - | - | - | - |
| `acosh 9` | - | - | - | - | - | - | - | - |
| `acosh 1_plus_1e-12` | - | - | - | - | - | - | - | - |
| `acosh 1e6` | - | - | - | - | - | - | - | - |
| `acosh e` | - | - | - | - | - | - | - | - |

#### Forward Hyperbolic Construction Cases

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `sinh half` | - | - | - | - | - | - | - | - |
| `cosh half` | - | - | - | - | - | - | - | - |
| `tanh half` | - | - | - | - | - | - | - | - |
| `sinh negative_tiny` | - | - | - | - | - | - | - | - |
| `cosh negative_tiny` | - | - | - | - | - | - | - | - |
| `tanh negative_tiny` | - | - | - | - | - | - | - | - |
| `sinh positive_20` | - | - | - | - | - | - | - | - |
| `cosh positive_20` | - | - | - | - | - | - | - | - |
| `tanh positive_20` | - | - | - | - | - | - | - | - |
| `sinh negative_20` | - | - | - | - | - | - | - | - |
| `cosh negative_20` | - | - | - | - | - | - | - | - |
| `tanh negative_20` | - | - | - | - | - | - | - | - |

#### Forward Hyperbolic Explicit f64 Output Cases

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `sinh half` | - | - | - | - | - | - | - | - |
| `cosh half` | - | - | - | - | - | - | - | - |
| `tanh half` | - | - | - | - | - | - | - | - |
| `sinh negative_tiny` | - | - | - | - | - | - | - | - |
| `cosh negative_tiny` | - | - | - | - | - | - | - | - |
| `tanh negative_tiny` | - | - | - | - | - | - | - | - |
| `sinh positive_20` | - | - | - | - | - | - | - | - |
| `cosh positive_20` | - | - | - | - | - | - | - | - |
| `tanh positive_20` | - | - | - | - | - | - | - | - |
| `sinh negative_20` | - | - | - | - | - | - | - | - |
| `cosh negative_20` | - | - | - | - | - | - | - | - |
| `tanh negative_20` | - | - | - | - | - | - | - | - |

#### Real API Operations

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | - | - | - | - | - | - | - | - |
| `one` | - | - | - | - | - | - | - | - |
| `e` | - | - | - | - | - | - | - | - |
| `pi` | - | - | - | - | - | - | - | - |
| `tau` | - | - | - | - | - | - | - | - |
| `add` | - | - | - | - | - | - | - | - |
| `sub` | - | - | - | - | - | - | - | - |
| `neg` | - | - | - | - | - | - | - | - |
| `mul` | - | - | - | - | - | - | - | - |
| `div` | - | - | - | - | - | - | - | - |
| `reciprocal` | - | - | - | - | - | - | - | - |
| `reciprocal checked` | - | - | - | - | - | - | - | - |
| `reciprocal checked abort` | - | - | - | - | - | - | - | - |
| `pow` | - | - | - | - | - | - | - | - |
| `powi` | - | - | - | - | - | - | - | - |
| `exp` | - | - | - | - | - | - | - | - |
| `exp 128` | - | - | - | - | - | - | - | - |
| `ln` | - | - | - | - | - | - | - | - |
| `log10` | - | - | - | - | - | - | - | - |
| `log10 abort` | - | - | - | - | - | - | - | - |
| `sqrt` | - | - | - | - | - | - | - | - |
| `sin` | - | - | - | - | - | - | - | - |
| `cos` | - | - | - | - | - | - | - | - |
| `tan` | - | - | - | - | - | - | - | - |
| `sinh` | - | - | - | - | - | - | - | - |
| `cosh` | - | - | - | - | - | - | - | - |
| `tanh` | - | - | - | - | - | - | - | - |
| `asin` | - | - | - | - | - | - | - | - |
| `asin abort` | - | - | - | - | - | - | - | - |
| `acos` | - | - | - | - | - | - | - | - |
| `acos abort` | - | - | - | - | - | - | - | - |
| `atan` | - | - | - | - | - | - | - | - |
| `atan abort` | - | - | - | - | - | - | - | - |
| `asinh` | - | - | - | - | - | - | - | - |
| `asinh abort` | - | - | - | - | - | - | - | - |
| `acosh` | - | - | - | - | - | - | - | - |
| `acosh abort` | - | - | - | - | - | - | - | - |
| `atanh` | - | - | - | - | - | - | - | - |
| `atanh abort` | - | - | - | - | - | - | - | - |
| `zero status` | - | - | - | - | - | - | - | - |
| `zero status abort` | - | - | - | - | - | - | - | - |

### Complex Operations

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | - | - | - | - | - | - | - | - |
| `one` | - | - | - | - | - | - | - | - |
| `i` | - | - | - | - | - | - | - | - |
| `free i` | - | - | - | - | - | - | - | - |
| `conjugate` | - | - | - | - | - | - | - | - |
| `norm squared` | - | - | - | - | - | - | - | - |
| `reciprocal` | - | - | - | - | - | - | - | - |
| `reciprocal checked` | - | - | - | - | - | - | - | - |
| `powi` | - | - | - | - | - | - | - | - |
| `powi checked` | - | - | - | - | - | - | - | - |
| `div checked` | - | - | - | - | - | - | - | - |
| `div real checked` | - | - | - | - | - | - | - | - |
| `from scalar` | - | - | - | - | - | - | - | - |
| `add` | - | - | - | - | - | - | - | - |
| `sub` | - | - | - | - | - | - | - | - |
| `neg` | - | - | - | - | - | - | - | - |
| `mul` | - | - | - | - | - | - | - | - |
| `div` | - | - | - | - | - | - | - | - |
| `div real` | - | - | - | - | - | - | - | - |

#### Cold Complex Multiplication

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `varying exact inputs` | - | - | - | - | - | - | - | - |

### Vector Operations

#### Vector Comparisons

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | - | - | - | - | - | - | - | - |
| `vec3 magnitude` | - | - | - | - | - | - | - | - |
| `vec3 normalize` | - | - | - | - | - | - | - | - |

#### Vector API Operations

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | - | - | - | - | - | - | - | - |
| `vec3 zero` | - | - | - | - | - | - | - | - |
| `vec3 dot abort` | - | - | - | - | - | - | - | - |
| `vec3 magnitude abort` | - | - | - | - | - | - | - | - |
| `vec3 normalize checked` | - | - | - | - | - | - | - | - |
| `vec3 normalize checked abort` | - | - | - | - | - | - | - | - |
| `vec3 div scalar checked` | - | - | - | - | - | - | - | - |
| `vec3 div scalar checked abort` | - | - | - | - | - | - | - | - |
| `vec3 add` | - | - | - | - | - | - | - | - |
| `vec3 add scalar` | - | - | - | - | - | - | - | - |
| `vec3 sub` | - | - | - | - | - | - | - | - |
| `vec3 sub scalar` | - | - | - | - | - | - | - | - |
| `vec3 neg` | - | - | - | - | - | - | - | - |
| `vec3 mul scalar` | - | - | - | - | - | - | - | - |
| `vec3 div scalar` | - | - | - | - | - | - | - | - |
| `vec4 dot` | - | - | - | - | - | - | - | - |
| `vec4 magnitude` | - | - | - | - | - | - | - | - |
| `vec4 normalize` | - | - | - | - | - | - | - | - |
| `vec4 add` | - | - | - | - | - | - | - | - |
| `vec4 add scalar` | - | - | - | - | - | - | - | - |
| `vec4 sub` | - | - | - | - | - | - | - | - |
| `vec4 sub scalar` | - | - | - | - | - | - | - | - |
| `vec4 neg` | - | - | - | - | - | - | - | - |
| `vec4 mul scalar` | - | - | - | - | - | - | - | - |
| `vec4 div scalar` | - | - | - | - | - | - | - | - |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | - | - | - | - | - | - | - | - |
| `mat3 inverse` | - | - | - | - | - | - | - | - |
| `mat3 mul mat3` | - | - | - | - | - | - | - | - |
| `mat3 transform vec3` | - | - | - | - | - | - | - | - |
| `mat4 determinant` | - | - | - | - | - | - | - | - |
| `mat4 inverse` | - | - | - | - | - | - | - | - |
| `mat4 mul mat4` | - | - | - | - | - | - | - | - |
| `mat4 transform vec4` | - | - | - | - | - | - | - | - |

#### Matrix API Operations

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | - | - | - | - | - | - | - | - |
| `mat3 zero` | - | - | - | - | - | - | - | - |
| `mat3 identity` | - | - | - | - | - | - | - | - |
| `mat3 transpose` | - | - | - | - | - | - | - | - |
| `mat3 reciprocal` | - | - | - | - | - | - | - | - |
| `mat3 reciprocal checked` | - | - | - | - | - | - | - | - |
| `mat3 inverse checked` | - | - | - | - | - | - | - | - |
| `mat3 inverse checked abort` | - | - | - | - | - | - | - | - |
| `mat3 powi` | - | - | - | - | - | - | - | - |
| `mat3 powi checked` | - | - | - | - | - | - | - | - |
| `mat3 powi checked abort` | - | - | - | - | - | - | - | - |
| `mat3 div scalar checked` | - | - | - | - | - | - | - | - |
| `mat3 div scalar checked abort` | - | - | - | - | - | - | - | - |
| `mat3 div matrix checked` | - | - | - | - | - | - | - | - |
| `mat3 div matrix checked abort` | - | - | - | - | - | - | - | - |
| `mat3 add` | - | - | - | - | - | - | - | - |
| `mat3 add scalar` | - | - | - | - | - | - | - | - |
| `mat3 sub` | - | - | - | - | - | - | - | - |
| `mat3 sub scalar` | - | - | - | - | - | - | - | - |
| `mat3 neg` | - | - | - | - | - | - | - | - |
| `mat3 mul scalar` | - | - | - | - | - | - | - | - |
| `mat3 div scalar` | - | - | - | - | - | - | - | - |
| `mat3 div matrix` | - | - | - | - | - | - | - | - |
| `mat3 bitxor` | - | - | - | - | - | - | - | - |
| `mat4 zero` | - | - | - | - | - | - | - | - |
| `mat4 identity` | - | - | - | - | - | - | - | - |
| `mat4 transpose` | - | - | - | - | - | - | - | - |
| `mat4 reciprocal` | - | - | - | - | - | - | - | - |
| `mat4 reciprocal checked` | - | - | - | - | - | - | - | - |
| `mat4 powi` | - | - | - | - | - | - | - | - |
| `mat4 powi checked` | - | - | - | - | - | - | - | - |
| `mat4 add` | - | - | - | - | - | - | - | - |
| `mat4 add scalar` | - | - | - | - | - | - | - | - |
| `mat4 sub` | - | - | - | - | - | - | - | - |
| `mat4 sub scalar` | - | - | - | - | - | - | - | - |
| `mat4 neg` | - | - | - | - | - | - | - | - |
| `mat4 mul scalar` | - | - | - | - | - | - | - | - |
| `mat4 div scalar` | - | - | - | - | - | - | - | - |
| `mat4 div matrix` | - | - | - | - | - | - | - | - |
| `mat4 bitxor` | - | - | - | - | - | - | - | - |

### Borrowed API Operations

| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | - | - | - | - | - | - | - | - |
| `scalar add ref_owned` | - | - | - | - | - | - | - | - |
| `scalar add refs` | - | - | - | - | - | - | - | - |
| `scalar add owned_ref_with_clone` | - | - | - | - | - | - | - | - |
| `scalar add ref_owned_with_clone` | - | - | - | - | - | - | - | - |
| `scalar sub owned_ref` | - | - | - | - | - | - | - | - |
| `scalar sub ref_owned` | - | - | - | - | - | - | - | - |
| `scalar sub refs` | - | - | - | - | - | - | - | - |
| `scalar sub owned_ref_with_clone` | - | - | - | - | - | - | - | - |
| `scalar sub ref_owned_with_clone` | - | - | - | - | - | - | - | - |
| `scalar mul owned_ref` | - | - | - | - | - | - | - | - |
| `scalar mul ref_owned` | - | - | - | - | - | - | - | - |
| `scalar mul refs` | - | - | - | - | - | - | - | - |
| `scalar mul owned_ref_with_clone` | - | - | - | - | - | - | - | - |
| `scalar mul ref_owned_with_clone` | - | - | - | - | - | - | - | - |
| `scalar div owned_ref` | - | - | - | - | - | - | - | - |
| `scalar div ref_owned` | - | - | - | - | - | - | - | - |
| `scalar div refs` | - | - | - | - | - | - | - | - |
| `scalar div owned_ref_with_clone` | - | - | - | - | - | - | - | - |
| `scalar div ref_owned_with_clone` | - | - | - | - | - | - | - | - |
| `vec3 add refs` | - | - | - | - | - | - | - | - |
| `vec3 sub refs` | - | - | - | - | - | - | - | - |
| `vec3 neg ref` | - | - | - | - | - | - | - | - |
| `vec3 add_scalar_ref` | - | - | - | - | - | - | - | - |
| `vec3 sub_scalar_ref` | - | - | - | - | - | - | - | - |
| `vec3 mul_scalar_ref` | - | - | - | - | - | - | - | - |
| `vec3 div_scalar_ref` | - | - | - | - | - | - | - | - |
| `vec4 add refs` | - | - | - | - | - | - | - | - |
| `vec4 sub refs` | - | - | - | - | - | - | - | - |
| `vec4 neg ref` | - | - | - | - | - | - | - | - |
| `vec4 add_scalar_ref` | - | - | - | - | - | - | - | - |
| `vec4 sub_scalar_ref` | - | - | - | - | - | - | - | - |
| `vec4 mul_scalar_ref` | - | - | - | - | - | - | - | - |
| `vec4 div_scalar_ref` | - | - | - | - | - | - | - | - |
| `mat3 add refs` | - | - | - | - | - | - | - | - |
| `mat3 sub refs` | - | - | - | - | - | - | - | - |
| `mat3 mul refs` | - | - | - | - | - | - | - | - |
| `mat3 div refs` | - | - | - | - | - | - | - | - |
| `mat3 neg ref` | - | - | - | - | - | - | - | - |
| `mat3 add_scalar_ref` | - | - | - | - | - | - | - | - |
| `mat3 sub_scalar_ref` | - | - | - | - | - | - | - | - |
| `mat3 mul_scalar_ref` | - | - | - | - | - | - | - | - |
| `mat3 div_scalar_ref` | - | - | - | - | - | - | - | - |
| `mat4 add refs` | - | - | - | - | - | - | - | - |
| `mat4 sub refs` | - | - | - | - | - | - | - | - |
| `mat4 mul refs` | - | - | - | - | - | - | - | - |
| `mat4 div refs` | - | - | - | - | - | - | - | - |
| `mat4 neg ref` | - | - | - | - | - | - | - | - |
| `mat4 add_scalar_ref` | - | - | - | - | - | - | - | - |
| `mat4 sub_scalar_ref` | - | - | - | - | - | - | - | - |
| `mat4 mul_scalar_ref` | - | - | - | - | - | - | - | - |
| `mat4 div_scalar_ref` | - | - | - | - | - | - | - | - |
| `mat3 transform_vec refs` | - | - | - | - | - | - | - | - |
| `mat4 transform_vec refs` | - | - | - | - | - | - | - | - |
| `complex add refs` | - | - | - | - | - | - | - | - |
| `complex sub refs` | - | - | - | - | - | - | - | - |
| `complex mul refs` | - | - | - | - | - | - | - | - |
| `complex div refs` | - | - | - | - | - | - | - | - |
| `complex neg ref` | - | - | - | - | - | - | - | - |
| `complex div_real_ref` | - | - | - | - | - | - | - | - |
