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

The `mathbench` suite benchmarks the Real-primary crate path and writes this file from Criterion's median estimates after a real benchmark run. The `numerica128` comparison column runs at 128-bit precision, while the `symbolica` column exercises Symbolica's symbolic expression engine. Missing cells mean that the corresponding estimate was not present in `target/criterion` when this file was generated.

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

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `sin 0.1` | 52.62 ns | 53.10 ns | 789.35 ns | 1.84 us | 0.07x | 0.03x |
| `cos 0.1` | 52.27 ns | 52.67 ns | 516.80 ns | 1.70 us | 0.10x | 0.03x |
| `sin 1.23456789` | 58.59 ns | 55.76 ns | 834.87 ns | 1.83 us | 0.07x | 0.03x |
| `cos 1.23456789` | 58.55 ns | 55.77 ns | 600.31 ns | 1.64 us | 0.10x | 0.04x |
| `sin 1e6` | 41.79 ns | 41.67 ns | 1.11 us | 2.02 us | 0.04x | 0.02x |
| `cos 1e6` | 41.62 ns | 41.56 ns | 820.70 ns | 1.84 us | 0.05x | 0.02x |
| `sin 1e30` | 41.68 ns | 41.58 ns | 2.86 us | 3.54 us | 0.01x | 0.01x |
| `cos 1e30` | 41.55 ns | 41.65 ns | 981.71 ns | 3.08 us | 0.04x | 0.01x |
| `sin pi_7` | 52.70 ns | 207.61 ns | 750.29 ns | 1.89 us | 0.07x | 0.03x |
| `cos pi_7` | 52.33 ns | 483.56 ns | 547.63 ns | 1.73 us | 0.10x | 0.03x |
| `sin 1000pi_eps` | 41.78 ns | 183.87 ns | 2.38 us | 3.07 us | 0.02x | 0.01x |
| `cos 1000pi_eps` | 41.51 ns | 182.74 ns | 571.66 ns | 3.48 us | 0.07x | 0.01x |
| `asin 0.5` | 99.09 ns | 98.36 ns | 2.95 us | 27.38 us | 0.03x | 0.00x |
| `acos 0.5` | 98.76 ns | 98.95 ns | 2.96 us | 28.83 us | 0.03x | 0.00x |
| `atanh 0.5` | 59.98 ns | 59.89 ns | 1.68 us | 27.74 us | 0.04x | 0.00x |
| `asin neg_0.999999` | 330.14 ns | 338.09 ns | 2.55 us | 27.79 us | 0.13x | 0.01x |
| `acos neg_0.999999` | 280.80 ns | 272.60 ns | 2.74 us | 28.33 us | 0.10x | 0.01x |
| `atanh neg_0.999999` | 178.66 ns | 161.28 ns | 1.68 us | 27.12 us | 0.11x | 0.01x |
| `asin 0.999999` | 216.62 ns | 237.58 ns | 2.55 us | 28.18 us | 0.09x | 0.01x |
| `acos 0.999999` | 147.97 ns | 163.60 ns | 2.70 us | 28.63 us | 0.05x | 0.01x |
| `atanh 0.999999` | 137.59 ns | 133.06 ns | 1.64 us | 27.57 us | 0.08x | 0.00x |
| `asin 1e-12` | 127.20 ns | 136.42 ns | 1.41 us | 15.45 us | 0.09x | 0.01x |
| `acos 1e-12` | 212.08 ns | 389.24 ns | 1.40 us | 14.99 us | 0.15x | 0.01x |
| `atanh 1e-12` | 95.07 ns | 99.71 ns | 167.97 ns | 20.23 us | 0.57x | 0.00x |
| `atan 0.5` | 66.62 ns | 67.29 ns | 2.75 us | 17.76 us | 0.02x | 0.00x |
| `asinh 0.5` | 94.82 ns | 94.09 ns | 1.60 us | 7.41 us | 0.06x | 0.01x |
| `atan neg_1e-12` | 152.37 ns | 153.97 ns | 1.11 us | 15.23 us | 0.14x | 0.01x |
| `asinh neg_1e-12` | 193.09 ns | 314.33 ns | 8.55 us | 11.85 us | 0.02x | 0.02x |
| `atan 1e6` | 116.74 ns | 123.18 ns | 1.42 us | 17.79 us | 0.08x | 0.01x |
| `asinh 1e6` | 110.67 ns | 112.36 ns | 1.67 us | 7.19 us | 0.07x | 0.02x |
| `atan neg_1e6` | 248.46 ns | 248.40 ns | 1.42 us | 17.77 us | 0.18x | 0.01x |
| `asinh neg_1e6` | 182.28 ns | 184.07 ns | 1.67 us | 6.95 us | 0.11x | 0.03x |
| `acosh 9` | 124.48 ns | 124.91 ns | 1.66 us | 9.77 us | 0.07x | 0.01x |
| `acosh 1_plus_1e-12` | 251.92 ns | 283.40 ns | 8.33 us | 11.38 us | 0.03x | 0.02x |
| `acosh 1e6` | 122.27 ns | 150.40 ns | 1.66 us | 9.87 us | 0.07x | 0.01x |
| `acosh e` | 112.90 ns | 1.13 us | 1.68 us | 9.82 us | 0.07x | 0.01x |

#### Real API Operations

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 10.24 ns | 11.93 ns | 15.58 ns | 0.94 ns | 0.66x | 10.86x |
| `one` | 10.61 ns | 11.96 ns | 29.89 ns | 42.26 ns | 0.36x | 0.25x |
| `e` | 48.85 ns | 48.70 ns | 1.06 us | 436.71 ns | 0.05x | 0.11x |
| `pi` | 36.70 ns | 36.22 ns | 47.55 ns | 444.48 ns | 0.77x | 0.08x |
| `tau` | 36.67 ns | 36.57 ns | 100.31 ns | 3.85 us | 0.37x | 0.01x |
| `add` | 111.63 ns | 109.42 ns | 41.91 ns | 2.68 us | 2.66x | 0.04x |
| `sub` | 139.20 ns | 123.56 ns | 44.74 ns | 5.09 us | 3.11x | 0.03x |
| `neg` | 58.17 ns | 58.21 ns | 20.23 ns | 2.21 us | 2.88x | 0.03x |
| `mul` | 171.36 ns | 144.54 ns | 44.48 ns | 3.16 us | 3.85x | 0.05x |
| `div` | 108.71 ns | 76.90 ns | 62.20 ns | 5.25 us | 1.75x | 0.02x |
| `reciprocal` | 62.38 ns | 62.68 ns | 59.05 ns | 3.23 us | 1.06x | 0.02x |
| `reciprocal checked` | 69.52 ns | 70.21 ns | 59.18 ns | 2.18 us | 1.17x | 0.03x |
| `reciprocal checked abort` | 82.76 ns | 84.84 ns | 59.09 ns | 1.56 us | 1.40x | 0.05x |
| `pow` | 6.36 us | 2.91 us | 2.93 us | 2.34 us | 2.17x | 2.71x |
| `powi` | 603.09 ns | 2.92 us | 83.15 ns | 1.52 us | 7.25x | 0.40x |
| `exp` | 87.76 ns | 92.14 ns | 915.33 ns | 1.90 us | 0.10x | 0.05x |
| `ln` | 1.23 us | 733.63 ns | 1.34 us | 1.82 us | 0.92x | 0.68x |
| `log10` | 1.41 us | 911.94 ns | 2.81 us | 6.73 us | 0.50x | 0.21x |
| `log10 abort` | 1.42 us | 930.20 ns | 2.80 us | 6.76 us | 0.51x | 0.21x |
| `sqrt` | 566.40 ns | 201.57 ns | 96.96 ns | 1.46 us | 5.84x | 0.39x |
| `sin` | 45.59 ns | 49.12 ns | 1.27 us | 2.30 us | 0.04x | 0.02x |
| `cos` | 45.67 ns | 48.81 ns | 636.51 ns | 1.75 us | 0.07x | 0.03x |
| `tan` | 49.81 ns | 50.76 ns | 1.60 us | 6.63 us | 0.03x | 0.01x |
| `sinh` | 2.15 us | 2.16 us | 1.11 us | 11.07 us | 1.93x | 0.19x |
| `cosh` | 2.16 us | 2.13 us | 1.04 us | 9.70 us | 2.07x | 0.22x |
| `tanh` | 4.11 us | 4.10 us | 1.19 us | 23.57 us | 3.44x | 0.17x |
| `asin` | 200.15 ns | 210.29 ns | 2.42 us | 13.90 us | 0.08x | 0.01x |
| `asin abort` | 217.01 ns | 233.48 ns | 2.43 us | 14.53 us | 0.09x | 0.01x |
| `acos` | 198.39 ns | 203.97 ns | 2.51 us | 14.36 us | 0.08x | 0.01x |
| `acos abort` | 215.89 ns | 228.60 ns | 2.51 us | 13.88 us | 0.09x | 0.02x |
| `atan` | 78.67 ns | 66.97 ns | 2.25 us | 19.33 us | 0.03x | 0.00x |
| `atan abort` | 106.96 ns | 99.47 ns | 2.26 us | 20.70 us | 0.05x | 0.01x |
| `asinh` | 99.87 ns | 96.40 ns | 1.67 us | 7.60 us | 0.06x | 0.01x |
| `asinh abort` | 143.54 ns | 138.57 ns | 1.68 us | 7.84 us | 0.09x | 0.02x |
| `acosh` | 167.01 ns | 161.35 ns | 4.33 us | 10.78 us | 0.04x | 0.02x |
| `acosh abort` | 186.53 ns | 182.26 ns | 3.41 us | 20.80 us | 0.05x | 0.01x |
| `atanh` | 120.97 ns | 115.10 ns | 1.33 us | 31.00 us | 0.09x | 0.00x |
| `atanh abort` | 144.99 ns | 140.79 ns | 1.31 us | 15.45 us | 0.11x | 0.01x |
| `zero status` | 1.04 ns | 1.09 ns | 6.77 ns | 8.12 ns | 0.15x | 0.13x |
| `zero status abort` | 1.05 ns | 1.19 ns | 6.75 ns | 9.65 ns | 0.16x | 0.11x |

### Complex Operations

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 21.04 ns | 15.79 ns | 28.33 ns | 1.88 ns | 0.74x | 11.22x |
| `one` | 21.15 ns | 16.26 ns | 42.46 ns | 30.48 ns | 0.50x | 0.69x |
| `i` | 21.38 ns | 16.23 ns | 45.53 ns | 29.70 ns | 0.47x | 0.72x |
| `free i` | 21.27 ns | 16.21 ns | 45.41 ns | 29.80 ns | 0.47x | 0.71x |
| `conjugate` | 96.94 ns | 74.55 ns | 37.83 ns | 1.06 us | 2.56x | 0.09x |
| `norm squared` | 422.55 ns | 223.52 ns | 123.86 ns | 4.28 us | 3.41x | 0.10x |
| `reciprocal` | 2.39 us | 1.14 us | 251.41 ns | 10.77 us | 9.51x | 0.22x |
| `reciprocal checked` | 2.37 us | 1.14 us | 251.51 ns | 10.91 us | 9.43x | 0.22x |
| `powi` | 2.96 us | 9.84 us | 1.21 us | 43.90 us | 2.44x | 0.07x |
| `powi checked` | 3.01 us | 9.91 us | 1.21 us | 43.26 us | 2.48x | 0.07x |
| `div checked` | 2.35 us | 1.66 us | 550.85 ns | 21.39 us | 4.27x | 0.11x |
| `div real checked` | 224.07 ns | 233.40 ns | 115.58 ns | 5.21 us | 1.94x | 0.04x |
| `from scalar` | 28.72 ns | 28.62 ns | 31.86 ns | 9.81 ns | 0.90x | 2.93x |
| `add` | 208.67 ns | 201.71 ns | 84.09 ns | 2.56 us | 2.48x | 0.08x |
| `sub` | 248.58 ns | 229.06 ns | 91.08 ns | 4.81 us | 2.73x | 0.05x |
| `neg` | 96.52 ns | 97.58 ns | 36.05 ns | 2.11 us | 2.68x | 0.05x |
| `mul` | 1.30 us | 1.13 us | 243.60 ns | 9.84 us | 5.32x | 0.13x |
| `div` | 1.94 us | 1.66 us | 541.39 ns | 21.74 us | 3.59x | 0.09x |
| `div real` | 220.08 ns | 233.34 ns | 115.43 ns | 5.17 us | 1.91x | 0.04x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 420.12 ns | 407.39 ns | 253.67 ns | 7.10 us | 1.66x | 0.06x |
| `vec3 magnitude` | 3.09 us | 2.24 us | 348.26 ns | 8.75 us | 8.87x | 0.35x |
| `vec3 normalize` | 5.10 us | 4.41 us | 599.57 ns | 16.56 us | 8.51x | 0.31x |

#### Vector API Operations

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 246.09 ns | 1.48 us | 56.41 ns | 825.22 ns | 4.36x | 0.30x |
| `vec3 zero` | 44.71 ns | 46.89 ns | 30.95 ns | 2.82 ns | 1.44x | 15.84x |
| `vec3 dot abort` | 415.75 ns | 347.07 ns | 204.28 ns | 7.19 us | 2.04x | 0.06x |
| `vec3 magnitude abort` | 3.10 us | 1.92 us | 324.35 ns | 8.97 us | 9.56x | 0.35x |
| `vec3 normalize checked` | 5.52 us | 3.95 us | 535.22 ns | 17.03 us | 10.31x | 0.32x |
| `vec3 normalize checked abort` | 5.54 us | 3.94 us | 532.19 ns | 17.32 us | 10.41x | 0.32x |
| `vec3 div scalar checked` | 457.45 ns | 429.43 ns | 171.88 ns | 7.69 us | 2.66x | 0.06x |
| `vec3 div scalar checked abort` | 469.60 ns | 452.38 ns | 172.41 ns | 7.68 us | 2.72x | 0.06x |
| `vec3 add` | 459.83 ns | 436.68 ns | 124.44 ns | 3.98 us | 3.70x | 0.12x |
| `vec3 add scalar` | 485.06 ns | 414.07 ns | 133.24 ns | 3.96 us | 3.64x | 0.12x |
| `vec3 sub` | 536.80 ns | 483.31 ns | 137.03 ns | 7.52 us | 3.92x | 0.07x |
| `vec3 sub scalar` | 452.64 ns | 403.79 ns | 123.86 ns | 7.01 us | 3.65x | 0.06x |
| `vec3 neg` | 188.72 ns | 191.45 ns | 50.51 ns | 3.35 us | 3.74x | 0.06x |
| `vec3 mul scalar` | 728.06 ns | 738.93 ns | 123.45 ns | 4.41 us | 5.90x | 0.16x |
| `vec3 div scalar` | 447.89 ns | 428.86 ns | 172.79 ns | 7.69 us | 2.59x | 0.06x |
| `vec4 dot` | 380.35 ns | 251.40 ns | 326.88 ns | 9.68 us | 1.16x | 0.04x |
| `vec4 magnitude` | 4.40 us | 1.30 us | 409.78 ns | 11.65 us | 10.73x | 0.38x |
| `vec4 normalize` | 5.28 us | 2.25 us | 683.22 ns | 22.38 us | 7.73x | 0.24x |
| `vec4 add` | 569.07 ns | 453.60 ns | 176.78 ns | 5.24 us | 3.22x | 0.11x |
| `vec4 add scalar` | 565.56 ns | 437.35 ns | 177.42 ns | 5.26 us | 3.19x | 0.11x |
| `vec4 sub` | 583.21 ns | 492.30 ns | 175.96 ns | 9.80 us | 3.31x | 0.06x |
| `vec4 sub scalar` | 532.46 ns | 406.39 ns | 170.91 ns | 9.52 us | 3.12x | 0.06x |
| `vec4 neg` | 253.25 ns | 234.00 ns | 66.48 ns | 3.98 us | 3.81x | 0.06x |
| `vec4 mul scalar` | 970.51 ns | 845.61 ns | 158.54 ns | 5.73 us | 6.12x | 0.17x |
| `vec4 div scalar` | 488.59 ns | 449.93 ns | 223.13 ns | 10.11 us | 2.19x | 0.05x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 842.03 ns | 397.30 ns | 841.14 ns | 21.93 us | 1.00x | 0.04x |
| `mat3 inverse` | 11.87 us | 2.16 us | 2.46 us | 82.19 us | 4.82x | 0.14x |
| `mat3 mul mat3` | 5.36 us | 4.00 us | 2.39 us | 60.97 us | 2.25x | 0.09x |
| `mat3 transform vec3` | 2.75 us | 2.04 us | 872.17 ns | 20.00 us | 3.16x | 0.14x |
| `mat4 determinant` | 1.85 us | 600.89 ns | 4.05 us | 96.11 us | 0.46x | 0.02x |
| `mat4 inverse` | 23.97 us | 5.69 us | 9.00 us | 345.43 us | 2.66x | 0.07x |
| `mat4 mul mat4` | 8.79 us | 6.81 us | 5.29 us | 146.47 us | 1.66x | 0.06x |
| `mat4 transform vec4` | 4.11 us | 2.97 us | 1.64 us | 35.63 us | 2.51x | 0.12x |

#### Matrix API Operations

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 764.93 ns | 2.21 us | 222.81 ns | 2.38 us | 3.43x | 0.32x |
| `mat3 zero` | 223.45 ns | 218.56 ns | 200.35 ns | 11.85 ns | 1.12x | 18.86x |
| `mat3 identity` | 240.55 ns | 238.99 ns | 251.74 ns | 180.71 ns | 0.96x | 1.33x |
| `mat3 transpose` | 270.70 ns | 264.70 ns | 191.91 ns | 107.12 ns | 1.41x | 2.53x |
| `mat3 reciprocal` | 12.01 us | 3.61 us | 2.28 us | 82.16 us | 5.28x | 0.15x |
| `mat3 reciprocal checked` | 11.93 us | 3.68 us | 2.27 us | 81.25 us | 5.25x | 0.15x |
| `mat3 inverse checked` | 12.05 us | 3.67 us | 2.27 us | 82.01 us | 5.30x | 0.15x |
| `mat3 inverse checked abort` | 12.04 us | 3.73 us | 2.28 us | 81.05 us | 5.29x | 0.15x |
| `mat3 powi` | 6.97 us | 11.51 us | 6.19 us | 154.11 us | 1.13x | 0.05x |
| `mat3 powi checked` | 6.99 us | 11.55 us | 6.21 us | 151.43 us | 1.12x | 0.05x |
| `mat3 powi checked abort` | 6.85 us | 11.47 us | 6.19 us | 149.78 us | 1.11x | 0.05x |
| `mat3 div scalar checked` | 1.45 us | 1.39 us | 809.02 ns | 22.39 us | 1.79x | 0.06x |
| `mat3 div scalar checked abort` | 1.45 us | 1.40 us | 807.49 ns | 22.04 us | 1.79x | 0.07x |
| `mat3 div matrix checked` | 17.14 us | 11.94 us | 4.46 us | 177.35 us | 3.84x | 0.10x |
| `mat3 div matrix checked abort` | 17.57 us | 11.88 us | 4.42 us | 159.34 us | 3.98x | 0.11x |
| `mat3 add` | 1.42 us | 1.40 us | 483.84 ns | 11.72 us | 2.94x | 0.12x |
| `mat3 add scalar` | 1.27 us | 1.27 us | 700.42 ns | 11.90 us | 1.81x | 0.11x |
| `mat3 sub` | 1.51 us | 1.47 us | 510.43 ns | 20.92 us | 2.95x | 0.07x |
| `mat3 sub scalar` | 1.29 us | 1.31 us | 695.68 ns | 21.35 us | 1.85x | 0.06x |
| `mat3 neg` | 775.02 ns | 748.29 ns | 466.58 ns | 8.53 us | 1.66x | 0.09x |
| `mat3 mul scalar` | 1.62 us | 1.66 us | 670.82 ns | 12.09 us | 2.42x | 0.13x |
| `mat3 div scalar` | 1.45 us | 1.38 us | 809.79 ns | 22.45 us | 1.79x | 0.06x |
| `mat3 div matrix` | 17.29 us | 11.86 us | 4.43 us | 157.16 us | 3.90x | 0.11x |
| `mat3 bitxor` | 6.84 us | 11.47 us | 6.21 us | 152.68 us | 1.10x | 0.04x |
| `mat4 zero` | 217.98 ns | 205.93 ns | 325.06 ns | 15.79 ns | 0.67x | 13.81x |
| `mat4 identity` | 281.48 ns | 274.47 ns | 384.33 ns | 253.41 ns | 0.73x | 1.11x |
| `mat4 transpose` | 325.56 ns | 326.84 ns | 335.75 ns | 162.96 ns | 0.97x | 2.00x |
| `mat4 reciprocal` | 24.32 us | 20.56 us | 8.78 us | 360.93 us | 2.77x | 0.07x |
| `mat4 reciprocal checked` | 24.57 us | 20.46 us | 8.79 us | 343.75 us | 2.80x | 0.07x |
| `mat4 powi` | 12.68 us | 21.73 us | 13.70 us | 667.14 us | 0.93x | 0.02x |
| `mat4 powi checked` | 12.73 us | 21.54 us | 13.79 us | 685.21 us | 0.92x | 0.02x |
| `mat4 add` | 1.87 us | 1.72 us | 792.58 ns | 40.57 us | 2.36x | 0.05x |
| `mat4 add scalar` | 1.98 us | 1.79 us | 1.17 us | 42.79 us | 1.69x | 0.05x |
| `mat4 sub` | 2.14 us | 1.89 us | 855.57 ns | 76.37 us | 2.50x | 0.03x |
| `mat4 sub scalar` | 2.02 us | 1.80 us | 1.17 us | 79.13 us | 1.72x | 0.03x |
| `mat4 neg` | 1.20 us | 987.40 ns | 762.11 ns | 29.74 us | 1.57x | 0.04x |
| `mat4 mul scalar` | 2.30 us | 2.23 us | 1.12 us | 20.43 us | 2.06x | 0.11x |
| `mat4 div scalar` | 2.14 us | 1.95 us | 1.38 us | 76.26 us | 1.54x | 0.03x |
| `mat4 div matrix` | 29.33 us | 32.86 us | 14.10 us | 533.86 us | 2.08x | 0.05x |
| `mat4 bitxor` | 12.46 us | 21.39 us | 13.74 us | 721.23 us | 0.91x | 0.02x |

### Borrowed API Operations

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 151.66 ns | 79.06 ns | 44.31 ns | 1.29 us | 3.42x | 0.12x |
| `scalar add ref_owned` | 150.53 ns | 81.80 ns | 44.29 ns | 1.28 us | 3.40x | 0.12x |
| `scalar add refs` | 157.91 ns | 90.16 ns | 44.29 ns | 1.29 us | 3.57x | 0.12x |
| `scalar add owned_ref_with_clone` | 170.09 ns | 99.00 ns | 58.47 ns | 1.27 us | 2.91x | 0.13x |
| `scalar add ref_owned_with_clone` | 171.25 ns | 94.34 ns | 55.92 ns | 1.31 us | 3.06x | 0.13x |
| `scalar sub owned_ref` | 190.87 ns | 87.33 ns | 47.03 ns | 2.46 us | 4.06x | 0.08x |
| `scalar sub ref_owned` | 171.66 ns | 88.54 ns | 46.86 ns | 2.45 us | 3.66x | 0.07x |
| `scalar sub refs` | 123.80 ns | 103.65 ns | 46.86 ns | 2.46 us | 2.64x | 0.05x |
| `scalar sub owned_ref_with_clone` | 123.04 ns | 111.03 ns | 62.11 ns | 2.46 us | 1.98x | 0.05x |
| `scalar sub ref_owned_with_clone` | 122.22 ns | 107.63 ns | 59.09 ns | 2.48 us | 2.07x | 0.05x |
| `scalar mul owned_ref` | 141.12 ns | 114.06 ns | 49.08 ns | 1.54 us | 2.88x | 0.09x |
| `scalar mul ref_owned` | 142.71 ns | 115.52 ns | 49.04 ns | 1.55 us | 2.91x | 0.09x |
| `scalar mul refs` | 155.11 ns | 130.22 ns | 49.05 ns | 1.54 us | 3.16x | 0.10x |
| `scalar mul owned_ref_with_clone` | 165.05 ns | 136.66 ns | 62.67 ns | 1.56 us | 2.63x | 0.11x |
| `scalar mul ref_owned_with_clone` | 157.22 ns | 132.80 ns | 61.53 ns | 1.57 us | 2.56x | 0.10x |
| `scalar div owned_ref` | 85.21 ns | 56.10 ns | 65.84 ns | 2.59 us | 1.29x | 0.03x |
| `scalar div ref_owned` | 87.41 ns | 57.67 ns | 65.68 ns | 2.58 us | 1.33x | 0.03x |
| `scalar div refs` | 87.68 ns | 59.48 ns | 65.69 ns | 2.60 us | 1.33x | 0.03x |
| `scalar div owned_ref_with_clone` | 96.96 ns | 68.35 ns | 80.40 ns | 4.92 us | 1.21x | 0.02x |
| `scalar div ref_owned_with_clone` | 97.54 ns | 66.92 ns | 76.61 ns | 2.91 us | 1.27x | 0.03x |
| `vec3 add refs` | 335.23 ns | 272.97 ns | 122.49 ns | 4.22 us | 2.74x | 0.08x |
| `vec3 sub refs` | 394.42 ns | 341.58 ns | 136.15 ns | 7.56 us | 2.90x | 0.05x |
| `vec3 neg ref` | 169.24 ns | 166.53 ns | 49.23 ns | 3.15 us | 3.44x | 0.05x |
| `vec3 add_scalar_ref` | 471.89 ns | 381.17 ns | 134.25 ns | 3.97 us | 3.52x | 0.12x |
| `vec3 sub_scalar_ref` | 467.29 ns | 399.44 ns | 126.26 ns | 7.17 us | 3.70x | 0.07x |
| `vec3 mul_scalar_ref` | 774.49 ns | 716.01 ns | 154.91 ns | 4.44 us | 5.00x | 0.17x |
| `vec3 div_scalar_ref` | 441.90 ns | 399.86 ns | 184.56 ns | 7.76 us | 2.39x | 0.06x |
| `vec4 add refs` | 382.04 ns | 306.02 ns | 171.48 ns | 5.24 us | 2.23x | 0.07x |
| `vec4 sub refs` | 369.78 ns | 328.02 ns | 264.87 ns | 9.65 us | 1.40x | 0.04x |
| `vec4 neg ref` | 214.43 ns | 195.67 ns | 64.33 ns | 4.00 us | 3.33x | 0.05x |
| `vec4 add_scalar_ref` | 514.59 ns | 406.30 ns | 175.46 ns | 5.17 us | 2.93x | 0.10x |
| `vec4 sub_scalar_ref` | 507.54 ns | 423.29 ns | 167.11 ns | 9.27 us | 3.04x | 0.05x |
| `vec4 mul_scalar_ref` | 899.76 ns | 823.85 ns | 161.07 ns | 5.65 us | 5.59x | 0.16x |
| `vec4 div_scalar_ref` | 493.06 ns | 463.92 ns | 227.41 ns | 10.03 us | 2.17x | 0.05x |
| `mat3 add refs` | 679.18 ns | 633.75 ns | 473.74 ns | 11.75 us | 1.43x | 0.06x |
| `mat3 sub refs` | 757.42 ns | 718.32 ns | 496.79 ns | 21.01 us | 1.52x | 0.04x |
| `mat3 mul refs` | 4.89 us | 4.20 us | 2.16 us | 61.15 us | 2.26x | 0.08x |
| `mat3 div refs` | 16.67 us | 11.24 us | 4.36 us | 160.10 us | 3.82x | 0.10x |
| `mat3 neg ref` | 538.23 ns | 532.92 ns | 456.00 ns | 8.45 us | 1.18x | 0.06x |
| `mat3 add_scalar_ref` | 1.25 us | 1.27 us | 697.18 ns | 12.03 us | 1.80x | 0.10x |
| `mat3 sub_scalar_ref` | 1.32 us | 1.26 us | 688.32 ns | 21.48 us | 1.92x | 0.06x |
| `mat3 mul_scalar_ref` | 1.62 us | 1.62 us | 659.98 ns | 12.13 us | 2.45x | 0.13x |
| `mat3 div_scalar_ref` | 1.39 us | 1.36 us | 806.63 ns | 22.50 us | 1.73x | 0.06x |
| `mat4 add refs` | 807.89 ns | 763.78 ns | 789.65 ns | 19.88 us | 1.02x | 0.04x |
| `mat4 sub refs` | 964.41 ns | 885.06 ns | 846.00 ns | 36.52 us | 1.14x | 0.03x |
| `mat4 mul refs` | 8.30 us | 7.62 us | 4.92 us | 144.85 us | 1.69x | 0.06x |
| `mat4 div refs` | 28.26 us | 32.85 us | 14.10 us | 540.77 us | 2.00x | 0.05x |
| `mat4 neg ref` | 766.60 ns | 755.02 ns | 732.10 ns | 13.70 us | 1.05x | 0.06x |
| `mat4 add_scalar_ref` | 1.75 us | 1.77 us | 1.17 us | 20.94 us | 1.49x | 0.08x |
| `mat4 sub_scalar_ref` | 1.82 us | 1.79 us | 1.16 us | 37.12 us | 1.57x | 0.05x |
| `mat4 mul_scalar_ref` | 2.14 us | 2.26 us | 1.11 us | 20.21 us | 1.92x | 0.11x |
| `mat4 div_scalar_ref` | 2.00 us | 1.97 us | 1.37 us | 37.99 us | 1.46x | 0.05x |
| `mat3 transform_vec refs` | 2.44 us | 2.05 us | 677.53 ns | 20.54 us | 3.60x | 0.12x |
| `mat4 transform_vec refs` | 3.79 us | 3.19 us | 1.28 us | 35.26 us | 2.96x | 0.11x |
| `complex add refs` | 160.77 ns | 153.64 ns | 84.28 ns | 2.52 us | 1.91x | 0.06x |
| `complex sub refs` | 188.79 ns | 174.94 ns | 90.98 ns | 4.89 us | 2.08x | 0.04x |
| `complex mul refs` | 1.28 us | 1.09 us | 244.88 ns | 9.94 us | 5.23x | 0.13x |
| `complex div refs` | 1.91 us | 1.63 us | 543.99 ns | 21.88 us | 3.50x | 0.09x |
| `complex neg ref` | 93.45 ns | 95.17 ns | 35.42 ns | 2.13 us | 2.64x | 0.04x |
| `complex div_real_ref` | 219.38 ns | 238.92 ns | 115.42 ns | 5.23 us | 1.90x | 0.04x |
