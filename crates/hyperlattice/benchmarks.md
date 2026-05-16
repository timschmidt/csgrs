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
| `sin 0.1` | 146.25 ns | 146.70 ns | 769.55 ns | 1.86 us | 0.19x | 0.08x |
| `cos 0.1` | 145.47 ns | 144.89 ns | 495.17 ns | 1.67 us | 0.29x | 0.09x |
| `sin 1.23456789` | 197.82 ns | 185.07 ns | 821.81 ns | 1.84 us | 0.24x | 0.11x |
| `cos 1.23456789` | 192.33 ns | 179.43 ns | 589.60 ns | 1.66 us | 0.33x | 0.12x |
| `sin 1e6` | 89.90 ns | 90.08 ns | 1.08 us | 2.03 us | 0.08x | 0.04x |
| `cos 1e6` | 88.66 ns | 89.06 ns | 810.46 ns | 1.82 us | 0.11x | 0.05x |
| `sin 1e30` | 90.02 ns | 89.82 ns | 2.87 us | 3.57 us | 0.03x | 0.03x |
| `cos 1e30` | 88.60 ns | 88.96 ns | 957.20 ns | 3.08 us | 0.09x | 0.03x |
| `sin pi_7` | 146.36 ns | 346.43 ns | 734.41 ns | 1.90 us | 0.20x | 0.08x |
| `cos pi_7` | 145.08 ns | 606.73 ns | 535.16 ns | 1.71 us | 0.27x | 0.09x |
| `sin 1000pi_eps` | 89.81 ns | 726.05 ns | 2.27 us | 2.80 us | 0.04x | 0.03x |
| `cos 1000pi_eps` | 88.88 ns | 721.66 ns | 584.93 ns | 1.67 us | 0.15x | 0.05x |
| `asin 0.5` | 136.78 ns | 137.28 ns | 3.00 us | 13.12 us | 0.05x | 0.01x |
| `acos 0.5` | 133.02 ns | 133.33 ns | 2.92 us | 13.01 us | 0.05x | 0.01x |
| `atanh 0.5` | 146.36 ns | 146.47 ns | 1.63 us | 12.70 us | 0.09x | 0.01x |
| `asin neg_0.999999` | 548.23 ns | 517.91 ns | 2.50 us | 12.70 us | 0.22x | 0.04x |
| `acos neg_0.999999` | 364.64 ns | 323.61 ns | 2.68 us | 12.83 us | 0.14x | 0.03x |
| `atanh neg_0.999999` | 272.03 ns | 247.75 ns | 1.57 us | 12.40 us | 0.17x | 0.02x |
| `asin 0.999999` | 490.73 ns | 704.78 ns | 2.48 us | 12.66 us | 0.20x | 0.04x |
| `acos 0.999999` | 335.38 ns | 780.28 ns | 2.69 us | 12.78 us | 0.12x | 0.03x |
| `atanh 0.999999` | 292.10 ns | 512.38 ns | 1.56 us | 12.40 us | 0.19x | 0.02x |
| `asin 1e-12` | 265.13 ns | 463.57 ns | 1.42 us | 15.04 us | 0.19x | 0.02x |
| `acos 1e-12` | 537.57 ns | 958.58 ns | 1.41 us | 14.98 us | 0.38x | 0.04x |
| `atanh 1e-12` | 305.71 ns | 485.93 ns | 170.10 ns | 19.75 us | 1.80x | 0.02x |
| `atan 0.5` | 159.02 ns | 163.55 ns | 2.72 us | 17.36 us | 0.06x | 0.01x |
| `asinh 0.5` | 212.03 ns | 211.36 ns | 1.62 us | 7.36 us | 0.13x | 0.03x |
| `atan neg_1e-12` | 279.13 ns | 263.46 ns | 1.12 us | 15.29 us | 0.25x | 0.02x |
| `asinh neg_1e-12` | 461.58 ns | 380.63 ns | 8.45 us | 11.76 us | 0.05x | 0.04x |
| `atan 1e6` | 158.84 ns | 160.57 ns | 1.42 us | 17.76 us | 0.11x | 0.01x |
| `asinh 1e6` | 209.11 ns | 213.25 ns | 1.59 us | 7.06 us | 0.13x | 0.03x |
| `atan neg_1e6` | 262.52 ns | 262.34 ns | 1.47 us | 17.72 us | 0.18x | 0.01x |
| `asinh neg_1e6` | 251.45 ns | 254.14 ns | 1.59 us | 7.02 us | 0.16x | 0.04x |
| `acosh 9` | 140.65 ns | 141.26 ns | 1.58 us | 9.67 us | 0.09x | 0.01x |
| `acosh 1_plus_1e-12` | 278.58 ns | 263.21 ns | 8.25 us | 11.78 us | 0.03x | 0.02x |
| `acosh 1e6` | 140.67 ns | 146.52 ns | 1.58 us | 9.76 us | 0.09x | 0.01x |
| `acosh e` | 180.22 ns | 2.26 us | 1.61 us | 9.63 us | 0.11x | 0.02x |

#### Real API Operations

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 18.23 ns | 18.66 ns | 15.57 ns | 0.95 ns | 1.17x | 19.20x |
| `one` | 24.08 ns | 26.84 ns | 30.63 ns | 31.96 ns | 0.79x | 0.75x |
| `e` | 86.58 ns | 89.19 ns | 1.04 us | 221.93 ns | 0.08x | 0.39x |
| `pi` | 49.67 ns | 49.86 ns | 49.42 ns | 224.11 ns | 1.00x | 0.22x |
| `tau` | 49.85 ns | 49.62 ns | 102.21 ns | 1.86 us | 0.49x | 0.03x |
| `add` | 224.85 ns | 323.88 ns | 42.06 ns | 1.27 us | 5.35x | 0.18x |
| `sub` | 264.93 ns | 497.45 ns | 44.81 ns | 2.41 us | 5.91x | 0.11x |
| `neg` | 39.35 ns | 40.88 ns | 20.62 ns | 1.05 us | 1.91x | 0.04x |
| `mul` | 139.35 ns | 404.02 ns | 44.29 ns | 1.50 us | 3.15x | 0.09x |
| `div` | 323.07 ns | 364.30 ns | 62.13 ns | 2.54 us | 5.20x | 0.13x |
| `reciprocal` | 73.46 ns | 76.47 ns | 59.13 ns | 1.53 us | 1.24x | 0.05x |
| `reciprocal checked` | 75.90 ns | 71.51 ns | 58.78 ns | 1.54 us | 1.29x | 0.05x |
| `reciprocal checked abort` | 83.22 ns | 79.35 ns | 58.97 ns | 1.54 us | 1.41x | 0.05x |
| `pow` | 6.36 us | 3.49 us | 2.83 us | 2.33 us | 2.24x | 2.73x |
| `powi` | 380.21 ns | 2.64 us | 86.84 ns | 1.59 us | 4.38x | 0.24x |
| `exp` | 253.52 ns | 233.17 ns | 927.76 ns | 1.90 us | 0.27x | 0.13x |
| `ln` | 1.41 us | 992.94 ns | 1.30 us | 1.79 us | 1.09x | 0.79x |
| `log10` | 1.55 us | 1.17 us | 2.76 us | 6.62 us | 0.56x | 0.23x |
| `log10 abort` | 1.54 us | 1.19 us | 2.75 us | 6.57 us | 0.56x | 0.23x |
| `sqrt` | 1.53 us | 1.61 us | 95.90 ns | 1.48 us | 15.96x | 1.03x |
| `sin` | 122.99 ns | 122.77 ns | 1.24 us | 2.22 us | 0.10x | 0.06x |
| `cos` | 121.19 ns | 120.43 ns | 627.09 ns | 1.77 us | 0.19x | 0.07x |
| `tan` | 169.16 ns | 173.48 ns | 1.57 us | 6.58 us | 0.11x | 0.03x |
| `sinh` | 3.59 us | 3.61 us | 1.11 us | 10.74 us | 3.22x | 0.33x |
| `cosh` | 3.47 us | 3.56 us | 1.05 us | 9.60 us | 3.32x | 0.36x |
| `tanh` | 5.98 us | 6.00 us | 1.20 us | 22.79 us | 4.98x | 0.26x |
| `asin` | 366.70 ns | 579.50 ns | 2.43 us | 13.57 us | 0.15x | 0.03x |
| `asin abort` | 376.88 ns | 588.71 ns | 2.47 us | 13.74 us | 0.15x | 0.03x |
| `acos` | 351.27 ns | 742.56 ns | 2.53 us | 13.65 us | 0.14x | 0.03x |
| `acos abort` | 358.27 ns | 744.12 ns | 2.52 us | 13.73 us | 0.14x | 0.03x |
| `atan` | 166.16 ns | 168.64 ns | 2.25 us | 19.17 us | 0.07x | 0.01x |
| `atan abort` | 170.92 ns | 161.69 ns | 2.25 us | 19.02 us | 0.08x | 0.01x |
| `asinh` | 212.05 ns | 220.34 ns | 1.62 us | 7.56 us | 0.13x | 0.03x |
| `asinh abort` | 214.82 ns | 228.03 ns | 1.61 us | 7.38 us | 0.13x | 0.03x |
| `acosh` | 192.94 ns | 190.67 ns | 3.28 us | 10.39 us | 0.06x | 0.02x |
| `acosh abort` | 191.93 ns | 185.29 ns | 3.32 us | 10.35 us | 0.06x | 0.02x |
| `atanh` | 258.48 ns | 469.66 ns | 1.26 us | 14.63 us | 0.21x | 0.02x |
| `atanh abort` | 262.75 ns | 465.45 ns | 1.26 us | 14.73 us | 0.21x | 0.02x |
| `zero status` | 1.07 ns | 1.08 ns | 7.15 ns | 8.41 ns | 0.15x | 0.13x |
| `zero status abort` | 1.16 ns | 1.12 ns | 7.26 ns | 8.46 ns | 0.16x | 0.14x |

### Complex Operations

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 40.02 ns | 40.21 ns | 22.47 ns | 1.89 ns | 1.78x | 21.19x |
| `one` | 44.76 ns | 44.69 ns | 41.46 ns | 30.11 ns | 1.08x | 1.49x |
| `i` | 50.33 ns | 50.07 ns | 42.82 ns | 31.95 ns | 1.18x | 1.58x |
| `free i` | 49.51 ns | 49.92 ns | 42.77 ns | 31.86 ns | 1.16x | 1.55x |
| `conjugate` | 90.87 ns | 90.44 ns | 35.62 ns | 1.07 us | 2.55x | 0.08x |
| `norm squared` | 165.71 ns | 562.32 ns | 117.54 ns | 4.24 us | 1.41x | 0.04x |
| `reciprocal` | 1.75 us | 2.82 us | 247.21 ns | 10.66 us | 7.09x | 0.16x |
| `reciprocal checked` | 1.78 us | 2.81 us | 246.00 ns | 10.70 us | 7.22x | 0.17x |
| `powi` | 1.34 us | 6.60 us | 1.22 us | 42.75 us | 1.10x | 0.03x |
| `powi checked` | 1.34 us | 6.58 us | 1.23 us | 43.64 us | 1.09x | 0.03x |
| `div checked` | 2.38 us | 4.65 us | 532.77 ns | 21.44 us | 4.47x | 0.11x |
| `div real checked` | 645.68 ns | 621.09 ns | 117.10 ns | 5.17 us | 5.51x | 0.12x |
| `from scalar` | 62.81 ns | 62.12 ns | 30.26 ns | 10.48 ns | 2.08x | 5.99x |
| `add` | 485.06 ns | 870.77 ns | 84.35 ns | 2.54 us | 5.75x | 0.19x |
| `sub` | 531.28 ns | 1.14 us | 90.77 ns | 4.70 us | 5.85x | 0.11x |
| `neg` | 87.79 ns | 86.78 ns | 32.25 ns | 2.11 us | 2.72x | 0.04x |
| `mul` | 907.83 ns | 3.49 us | 246.15 ns | 9.97 us | 3.69x | 0.09x |
| `div` | 2.33 us | 4.66 us | 535.73 ns | 21.83 us | 4.36x | 0.11x |
| `div real` | 658.87 ns | 614.56 ns | 117.06 ns | 5.16 us | 5.63x | 0.13x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 212.60 ns | 2.35 us | 256.27 ns | 7.14 us | 0.83x | 0.03x |
| `vec3 magnitude` | 3.96 us | 5.41 us | 348.59 ns | 8.74 us | 11.35x | 0.45x |
| `vec3 normalize` | 8.18 us | 11.39 us | 588.76 ns | 16.72 us | 13.89x | 0.49x |

#### Vector API Operations

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 281.79 ns | 1.37 us | 58.41 ns | 738.81 ns | 4.82x | 0.38x |
| `vec3 zero` | 145.40 ns | 145.03 ns | 31.53 ns | 2.84 ns | 4.61x | 51.22x |
| `vec3 dot abort` | 212.87 ns | 904.95 ns | 206.08 ns | 7.29 us | 1.03x | 0.03x |
| `vec3 magnitude abort` | 4.03 us | 3.17 us | 322.90 ns | 8.68 us | 12.47x | 0.46x |
| `vec3 normalize checked` | 8.23 us | 7.40 us | 536.09 ns | 16.77 us | 15.35x | 0.49x |
| `vec3 normalize checked abort` | 8.22 us | 7.18 us | 541.59 ns | 16.90 us | 15.19x | 0.49x |
| `vec3 div scalar checked` | 1.31 us | 1.54 us | 181.93 ns | 7.50 us | 7.20x | 0.17x |
| `vec3 div scalar checked abort` | 1.29 us | 1.52 us | 182.32 ns | 7.47 us | 7.08x | 0.17x |
| `vec3 add` | 932.39 ns | 1.71 us | 128.38 ns | 3.97 us | 7.26x | 0.24x |
| `vec3 add scalar` | 763.18 ns | 1.45 us | 135.53 ns | 3.82 us | 5.63x | 0.20x |
| `vec3 sub` | 1.03 us | 2.47 us | 140.17 ns | 7.51 us | 7.34x | 0.14x |
| `vec3 sub scalar` | 758.05 ns | 1.24 us | 128.63 ns | 7.11 us | 5.89x | 0.11x |
| `vec3 neg` | 157.80 ns | 157.18 ns | 52.67 ns | 3.11 us | 3.00x | 0.05x |
| `vec3 mul scalar` | 517.06 ns | 2.01 us | 126.70 ns | 4.35 us | 4.08x | 0.12x |
| `vec3 div scalar` | 1.30 us | 1.57 us | 176.56 ns | 7.73 us | 7.37x | 0.17x |
| `vec4 dot` | 255.70 ns | 646.00 ns | 320.22 ns | 9.68 us | 0.80x | 0.03x |
| `vec4 magnitude` | 3.74 us | 2.55 us | 408.71 ns | 11.42 us | 9.16x | 0.33x |
| `vec4 normalize` | 8.50 us | 6.35 us | 704.55 ns | 22.64 us | 12.07x | 0.38x |
| `vec4 add` | 1.12 us | 1.80 us | 173.09 ns | 5.28 us | 6.50x | 0.21x |
| `vec4 add scalar` | 966.26 ns | 1.68 us | 176.01 ns | 5.09 us | 5.49x | 0.19x |
| `vec4 sub` | 1.22 us | 2.25 us | 175.76 ns | 9.59 us | 6.94x | 0.13x |
| `vec4 sub scalar` | 964.46 ns | 1.46 us | 170.33 ns | 9.41 us | 5.66x | 0.10x |
| `vec4 neg` | 189.38 ns | 190.40 ns | 65.45 ns | 4.04 us | 2.89x | 0.05x |
| `vec4 mul scalar` | 648.72 ns | 2.03 us | 163.35 ns | 5.62 us | 3.97x | 0.12x |
| `vec4 div scalar` | 1.81 us | 1.56 us | 231.59 ns | 10.50 us | 7.80x | 0.17x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 700.31 ns | 2.06 us | 839.49 ns | 22.03 us | 0.83x | 0.03x |
| `mat3 inverse` | 14.44 us | 7.89 us | 2.45 us | 81.16 us | 5.89x | 0.18x |
| `mat3 mul mat3` | 3.30 us | 7.61 us | 2.33 us | 60.60 us | 1.41x | 0.05x |
| `mat3 transform vec3` | 1.53 us | 4.90 us | 890.50 ns | 19.95 us | 1.72x | 0.08x |
| `mat4 determinant` | 2.14 us | 1.48 us | 4.32 us | 94.00 us | 0.49x | 0.02x |
| `mat4 inverse` | 24.64 us | 8.43 us | 9.06 us | 336.94 us | 2.72x | 0.07x |
| `mat4 mul mat4` | 6.19 us | 4.65 us | 5.33 us | 140.06 us | 1.16x | 0.04x |
| `mat4 transform vec4` | 2.56 us | 2.30 us | 1.65 us | 34.98 us | 1.55x | 0.07x |

#### Matrix API Operations

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 1.38 us | 3.93 us | 237.94 ns | 2.09 us | 5.82x | 0.66x |
| `mat3 zero` | 577.13 ns | 546.63 ns | 207.46 ns | 11.43 ns | 2.78x | 50.50x |
| `mat3 identity` | 676.25 ns | 670.98 ns | 238.66 ns | 157.69 ns | 2.83x | 4.29x |
| `mat3 transpose` | 719.97 ns | 731.32 ns | 208.20 ns | 111.04 ns | 3.46x | 6.48x |
| `mat3 reciprocal` | 15.02 us | 25.06 us | 2.31 us | 80.92 us | 6.51x | 0.19x |
| `mat3 reciprocal checked` | 14.74 us | 24.90 us | 2.32 us | 80.81 us | 6.36x | 0.18x |
| `mat3 inverse checked` | 14.98 us | 25.42 us | 2.27 us | 82.88 us | 6.61x | 0.18x |
| `mat3 inverse checked abort` | 14.72 us | 25.12 us | 2.26 us | 80.99 us | 6.50x | 0.18x |
| `mat3 powi` | 4.54 us | 39.85 us | 6.24 us | 148.64 us | 0.73x | 0.03x |
| `mat3 powi checked` | 4.55 us | 39.26 us | 6.21 us | 145.41 us | 0.73x | 0.03x |
| `mat3 powi checked abort` | 4.46 us | 38.84 us | 6.21 us | 147.86 us | 0.72x | 0.03x |
| `mat3 div scalar checked` | 4.23 us | 5.54 us | 803.17 ns | 22.03 us | 5.27x | 0.19x |
| `mat3 div scalar checked abort` | 4.38 us | 5.55 us | 803.84 ns | 22.11 us | 5.45x | 0.20x |
| `mat3 div matrix checked` | 18.51 us | 44.19 us | 4.43 us | 160.17 us | 4.18x | 0.12x |
| `mat3 div matrix checked abort` | 18.44 us | 43.88 us | 4.47 us | 161.47 us | 4.12x | 0.11x |
| `mat3 add` | 3.07 us | 5.32 us | 498.66 ns | 11.57 us | 6.16x | 0.27x |
| `mat3 add scalar` | 2.34 us | 4.12 us | 699.25 ns | 12.06 us | 3.35x | 0.19x |
| `mat3 sub` | 3.80 us | 7.09 us | 527.50 ns | 20.98 us | 7.20x | 0.18x |
| `mat3 sub scalar` | 2.41 us | 3.84 us | 699.35 ns | 21.11 us | 3.44x | 0.11x |
| `mat3 neg` | 632.57 ns | 644.32 ns | 460.06 ns | 8.46 us | 1.37x | 0.07x |
| `mat3 mul scalar` | 1.73 us | 5.83 us | 682.83 ns | 11.92 us | 2.53x | 0.14x |
| `mat3 div scalar` | 4.22 us | 5.58 us | 802.51 ns | 22.24 us | 5.26x | 0.19x |
| `mat3 div matrix` | 18.54 us | 44.73 us | 4.50 us | 159.66 us | 4.12x | 0.12x |
| `mat3 bitxor` | 4.49 us | 38.70 us | 6.20 us | 147.82 us | 0.72x | 0.03x |
| `mat4 zero` | 995.00 ns | 923.22 ns | 356.42 ns | 14.02 ns | 2.79x | 70.97x |
| `mat4 identity` | 1.16 us | 1.10 us | 412.87 ns | 230.01 ns | 2.81x | 5.04x |
| `mat4 transpose` | 1.11 us | 1.07 us | 369.82 ns | 200.94 ns | 2.99x | 5.50x |
| `mat4 reciprocal` | 25.19 us | 41.12 us | 8.71 us | 340.60 us | 2.89x | 0.07x |
| `mat4 reciprocal checked` | 25.01 us | 41.47 us | 8.76 us | 343.63 us | 2.86x | 0.07x |
| `mat4 powi` | 8.81 us | 52.49 us | 13.74 us | 344.50 us | 0.64x | 0.03x |
| `mat4 powi checked` | 8.91 us | 54.50 us | 13.94 us | 357.86 us | 0.64x | 0.02x |
| `mat4 add` | 4.87 us | 5.86 us | 854.73 ns | 19.90 us | 5.69x | 0.24x |
| `mat4 add scalar` | 3.91 us | 5.44 us | 1.18 us | 21.92 us | 3.31x | 0.18x |
| `mat4 sub` | 5.16 us | 6.92 us | 895.58 ns | 36.88 us | 5.77x | 0.14x |
| `mat4 sub scalar` | 3.70 us | 5.37 us | 1.17 us | 37.91 us | 3.17x | 0.10x |
| `mat4 neg` | 1.06 us | 1.00 us | 752.28 ns | 14.20 us | 1.41x | 0.07x |
| `mat4 mul scalar` | 3.00 us | 7.37 us | 1.14 us | 20.22 us | 2.64x | 0.15x |
| `mat4 div scalar` | 7.50 us | 7.73 us | 1.38 us | 39.25 us | 5.44x | 0.19x |
| `mat4 div matrix` | 28.76 us | 61.45 us | 14.11 us | 517.02 us | 2.04x | 0.06x |
| `mat4 bitxor` | 8.69 us | 52.11 us | 13.70 us | 349.64 us | 0.63x | 0.02x |

### Borrowed API Operations

| Benchmark | Hyperreal from f64 | Hyperreal rational | numerica128 | symbolica | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 188.12 ns | 283.13 ns | 44.60 ns | 1.27 us | 4.22x | 0.15x |
| `scalar add ref_owned` | 203.54 ns | 290.63 ns | 44.69 ns | 1.27 us | 4.55x | 0.16x |
| `scalar add refs` | 173.01 ns | 269.03 ns | 45.02 ns | 1.27 us | 3.84x | 0.14x |
| `scalar add owned_ref_with_clone` | 200.06 ns | 310.00 ns | 63.44 ns | 1.27 us | 3.15x | 0.16x |
| `scalar add ref_owned_with_clone` | 197.94 ns | 313.81 ns | 58.04 ns | 1.28 us | 3.41x | 0.15x |
| `scalar sub owned_ref` | 227.04 ns | 461.47 ns | 47.31 ns | 2.41 us | 4.80x | 0.09x |
| `scalar sub ref_owned` | 227.24 ns | 462.08 ns | 47.41 ns | 2.43 us | 4.79x | 0.09x |
| `scalar sub refs` | 210.35 ns | 440.70 ns | 47.23 ns | 2.42 us | 4.45x | 0.09x |
| `scalar sub owned_ref_with_clone` | 236.43 ns | 491.83 ns | 66.55 ns | 2.42 us | 3.55x | 0.10x |
| `scalar sub ref_owned_with_clone` | 238.17 ns | 486.39 ns | 60.89 ns | 2.42 us | 3.91x | 0.10x |
| `scalar mul owned_ref` | 96.47 ns | 368.56 ns | 47.36 ns | 1.51 us | 2.04x | 0.06x |
| `scalar mul ref_owned` | 96.22 ns | 368.26 ns | 47.31 ns | 1.52 us | 2.03x | 0.06x |
| `scalar mul refs` | 80.20 ns | 352.20 ns | 47.27 ns | 1.51 us | 1.70x | 0.05x |
| `scalar mul owned_ref_with_clone` | 109.34 ns | 394.59 ns | 63.49 ns | 1.52 us | 1.72x | 0.07x |
| `scalar mul ref_owned_with_clone` | 107.92 ns | 398.74 ns | 59.02 ns | 1.51 us | 1.83x | 0.07x |
| `scalar div owned_ref` | 263.34 ns | 323.93 ns | 64.28 ns | 2.49 us | 4.10x | 0.11x |
| `scalar div ref_owned` | 265.02 ns | 315.77 ns | 64.39 ns | 2.48 us | 4.12x | 0.11x |
| `scalar div refs` | 252.82 ns | 306.73 ns | 64.44 ns | 2.50 us | 3.92x | 0.10x |
| `scalar div owned_ref_with_clone` | 279.14 ns | 343.95 ns | 80.38 ns | 2.49 us | 3.47x | 0.11x |
| `scalar div ref_owned_with_clone` | 277.65 ns | 344.27 ns | 76.46 ns | 2.52 us | 3.63x | 0.11x |
| `vec3 add refs` | 637.09 ns | 1.41 us | 124.37 ns | 3.98 us | 5.12x | 0.16x |
| `vec3 sub refs` | 759.09 ns | 2.18 us | 139.73 ns | 7.45 us | 5.43x | 0.10x |
| `vec3 neg ref` | 178.77 ns | 183.81 ns | 45.16 ns | 3.30 us | 3.96x | 0.05x |
| `vec3 add_scalar_ref` | 759.34 ns | 1.46 us | 136.31 ns | 3.79 us | 5.57x | 0.20x |
| `vec3 sub_scalar_ref` | 772.64 ns | 1.27 us | 128.24 ns | 7.23 us | 6.03x | 0.11x |
| `vec3 mul_scalar_ref` | 513.61 ns | 2.03 us | 125.87 ns | 4.34 us | 4.08x | 0.12x |
| `vec3 div_scalar_ref` | 1.35 us | 1.53 us | 171.57 ns | 7.74 us | 7.88x | 0.17x |
| `vec4 add refs` | 749.54 ns | 1.38 us | 170.48 ns | 5.42 us | 4.40x | 0.14x |
| `vec4 sub refs` | 830.28 ns | 1.83 us | 172.31 ns | 9.52 us | 4.82x | 0.09x |
| `vec4 neg ref` | 228.97 ns | 238.67 ns | 64.08 ns | 3.96 us | 3.57x | 0.06x |
| `vec4 add_scalar_ref` | 985.17 ns | 1.73 us | 174.48 ns | 5.27 us | 5.65x | 0.19x |
| `vec4 sub_scalar_ref` | 1.03 us | 1.51 us | 166.52 ns | 9.31 us | 6.21x | 0.11x |
| `vec4 mul_scalar_ref` | 645.58 ns | 2.07 us | 161.10 ns | 5.54 us | 4.01x | 0.12x |
| `vec4 div_scalar_ref` | 1.74 us | 1.55 us | 225.31 ns | 10.06 us | 7.70x | 0.17x |
| `mat3 add refs` | 1.57 us | 3.73 us | 447.52 ns | 11.69 us | 3.51x | 0.13x |
| `mat3 sub refs` | 1.81 us | 5.78 us | 476.83 ns | 21.04 us | 3.79x | 0.09x |
| `mat3 mul refs` | 2.31 us | 14.84 us | 2.12 us | 61.76 us | 1.09x | 0.04x |
| `mat3 div refs` | 17.13 us | 42.95 us | 4.41 us | 159.65 us | 3.88x | 0.11x |
| `mat3 neg ref` | 497.77 ns | 493.95 ns | 439.66 ns | 8.68 us | 1.13x | 0.06x |
| `mat3 add_scalar_ref` | 2.24 us | 3.97 us | 688.92 ns | 12.15 us | 3.25x | 0.18x |
| `mat3 sub_scalar_ref` | 2.36 us | 3.94 us | 678.54 ns | 21.64 us | 3.48x | 0.11x |
| `mat3 mul_scalar_ref` | 1.70 us | 5.69 us | 655.43 ns | 11.95 us | 2.59x | 0.14x |
| `mat3 div_scalar_ref` | 4.30 us | 5.58 us | 804.55 ns | 22.13 us | 5.34x | 0.19x |
| `mat4 add refs` | 2.44 us | 3.39 us | 861.96 ns | 20.06 us | 2.83x | 0.12x |
| `mat4 sub refs` | 2.69 us | 4.60 us | 904.87 ns | 36.13 us | 2.97x | 0.07x |
| `mat4 mul refs` | 4.27 us | 19.28 us | 5.00 us | 142.78 us | 0.85x | 0.03x |
| `mat4 div refs` | 27.14 us | 59.98 us | 13.87 us | 526.44 us | 1.96x | 0.05x |
| `mat4 neg ref` | 851.71 ns | 747.90 ns | 727.38 ns | 14.11 us | 1.17x | 0.06x |
| `mat4 add_scalar_ref` | 3.85 us | 5.59 us | 1.17 us | 20.61 us | 3.28x | 0.19x |
| `mat4 sub_scalar_ref` | 3.86 us | 5.57 us | 1.16 us | 37.61 us | 3.32x | 0.10x |
| `mat4 mul_scalar_ref` | 2.96 us | 7.33 us | 1.10 us | 20.15 us | 2.70x | 0.15x |
| `mat4 div_scalar_ref` | 7.48 us | 7.73 us | 1.36 us | 38.05 us | 5.51x | 0.20x |
| `mat3 transform_vec refs` | 931.40 ns | 5.06 us | 672.35 ns | 20.11 us | 1.39x | 0.05x |
| `mat4 transform_vec refs` | 1.46 us | 5.89 us | 1.27 us | 36.34 us | 1.15x | 0.04x |
| `complex add refs` | 344.50 ns | 734.65 ns | 85.07 ns | 2.57 us | 4.05x | 0.13x |
| `complex sub refs` | 385.50 ns | 987.17 ns | 94.98 ns | 5.06 us | 4.06x | 0.08x |
| `complex mul refs` | 777.18 ns | 3.40 us | 244.55 ns | 10.05 us | 3.18x | 0.08x |
| `complex div refs` | 2.19 us | 4.55 us | 567.94 ns | 21.81 us | 3.85x | 0.10x |
| `complex neg ref` | 71.89 ns | 81.91 ns | 32.46 ns | 2.22 us | 2.21x | 0.03x |
| `complex div_real_ref` | 614.42 ns | 587.54 ns | 117.06 ns | 5.23 us | 5.25x | 0.12x |
