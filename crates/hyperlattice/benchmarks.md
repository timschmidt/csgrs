# Benchmarks

Run the Criterion benchmark suite:

```sh
cargo bench --bench mathbench
```

Refresh this file from existing Criterion estimates without rerunning the full suite:

```sh
cargo bench --bench mathbench -- --update-benchmarks-md
```

The `mathbench` suite benchmarks both crate backends and writes this file from Criterion's median estimates after a real benchmark run. The `astro-float` and `arpfloat` comparison columns run at 128-bit precision. Missing cells mean that the corresponding estimate was not present in `target/criterion` when this file was generated, or that the external library does not expose a directly comparable operation in this suite.

Each benchmarked operation rotates through adversarial inputs for its valid domain: near-zero values, large and tiny magnitudes, cancellation-prone vectors, near-singular matrices, and range-reduction-heavy trigonometric arguments.

## Operation Coverage

- Scalar construction/constants, arithmetic, reciprocal, powers, exponentials, logarithms, square root, trigonometric and hyperbolic functions, inverse helpers, zero-status checks, and abort-aware variants.
- Complex construction/constants, conjugate, norm squared, reciprocal, powers, checked division, scalar conversion, arithmetic, and real scalar division.
- Vector construction, zero, dot product, magnitude, normalization, vector/vector arithmetic, vector/scalar arithmetic, scalar division, and checked/abort-aware variants for 3D and 4D vectors.
- Matrix construction, zero, identity, transpose, determinant, inverse, reciprocal, powers, matrix/matrix arithmetic, matrix/scalar arithmetic, matrix/vector transformation, scalar division, matrix division, and checked/abort-aware variants for 3x3 and 4x4 matrices.

## Benchmark Results

The following Criterion median estimates were collected on an AMD Ryzen 7 5800X3D on Fedora. Values are formatted to two digits after the decimal.

### Scalar Operations

#### Scalar Trigonometric Comparisons

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `sin 0.1` | 10.85 ns | 2.37 us | 2.34 us | 10.43 us | 128.76 us | 218.09x | 0.23x | 0.02x |
| `cos 0.1` | 11.50 ns | 232.65 ns | 221.15 ns | 10.16 us | 25.91 us | 20.22x | 0.02x | 0.01x |
| `sin 1.23456789` | 11.67 ns | 1.83 us | 1.89 us | 11.85 us | 269.01 us | 157.11x | 0.15x | 0.01x |
| `cos 1.23456789` | 11.94 ns | 416.88 ns | 414.80 ns | 10.06 us | 173.22 us | 34.92x | 0.04x | 0.00x |
| `sin 1e6` | 12.51 ns | 34.94 us | 35.03 us | 15.72 us | 271.91 us | 2792.34x | 2.22x | 0.13x |
| `cos 1e6` | 12.21 ns | 21.80 us | 21.60 us | 14.00 us | 171.71 us | 1785.97x | 1.56x | 0.13x |
| `sin 1e30` | 65.71 ns | 182.13 us | 184.50 us | 18.12 us | 277.78 us | 2771.47x | 10.05x | 0.66x |
| `cos 1e30` | 68.02 ns | 137.41 us | 138.21 us | 15.60 us | 172.01 us | 2020.14x | 8.81x | 0.80x |
| `sin pi_7` | 11.67 ns | 2.39 us | 4.26 us | 11.51 us | 127.91 us | 204.84x | 0.21x | 0.02x |
| `cos pi_7` | 11.52 ns | 219.10 ns | 4.43 us | 10.35 us | 28.12 us | 19.02x | 0.02x | 0.01x |
| `sin 1000pi_eps` | 11.70 ns | 22.91 us | 33.93 us | 15.75 us | 267.79 us | 1958.69x | 1.45x | 0.09x |
| `cos 1000pi_eps` | 12.24 ns | 13.56 us | 24.12 us | 13.81 us | 156.08 us | 1107.92x | 0.98x | 0.09x |

#### Scalar API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.47 ns | 40.43 ns | 39.75 ns | 24.42 ns | 13.98 ns | 86.47x | 1.66x | 2.89x |
| `one` | 0.47 ns | 68.03 ns | 66.60 ns | 38.40 ns | 45.35 ns | 144.87x | 1.77x | 1.50x |
| `e` | 0.47 ns | 1.17 us | 1.21 us | 47.46 ns | 53.07 us | 2495.30x | 24.62x | 0.02x |
| `pi` | 0.47 ns | 175.18 ns | 182.86 ns | 43.69 ns | 103.49 us | 373.35x | 4.01x | 0.00x |
| `tau` | 0.47 ns | 308.04 ns | 324.82 ns | 115.89 ns | 103.17 us | 660.64x | 2.66x | 0.00x |
| `add` | 5.17 ns | 399.59 ns | 406.43 ns | 50.52 ns | 101.95 ns | 77.35x | 7.91x | 3.92x |
| `sub` | 5.07 ns | 659.28 ns | 454.25 ns | 55.83 ns | 148.27 ns | 130.13x | 11.81x | 4.45x |
| `neg` | 5.01 ns | 64.18 ns | 81.51 ns | 22.30 ns | 16.25 ns | 12.82x | 2.88x | 3.95x |
| `mul` | 5.38 ns | 434.20 ns | 326.92 ns | 56.93 ns | 174.53 ns | 80.67x | 7.63x | 2.49x |
| `div` | 7.77 ns | 404.65 ns | 249.54 ns | 132.62 ns | 879.20 ns | 52.05x | 3.05x | 0.46x |
| `reciprocal` | 8.53 ns | 105.29 ns | 102.18 ns | 155.78 ns | 1.31 us | 12.35x | 0.68x | 0.08x |
| `reciprocal checked` | 8.71 ns | 106.72 ns | 99.72 ns | 155.76 ns | 1.29 us | 12.25x | 0.69x | 0.08x |
| `reciprocal checked abort` | 26.19 ns | 195.20 ns | 165.53 ns | 155.78 ns | 1.31 us | 7.45x | 1.25x | 0.15x |
| `pow` | 24.89 ns | 11.52 us | 8.39 us | 57.50 us | 381.52 us | 462.93x | 0.20x | 0.03x |
| `powi` | 6.55 ns | 2.85 us | 3.04 us | 285.63 ns | 1.40 us | 435.30x | 9.99x | 2.04x |
| `exp` | 10.16 ns | 1.05 us | 1.11 us | 14.35 us | 71.33 us | 103.51x | 0.07x | 0.01x |
| `ln` | 11.59 ns | 2.91 us | 3.16 us | 32.62 us | 263.74 us | 251.37x | 0.09x | 0.01x |
| `log10` | 21.76 ns | 9.83 us | 8.47 us | 38.29 us | 479.54 us | 451.75x | 0.26x | 0.02x |
| `log10 abort` | 18.27 ns | 9.88 us | 8.46 us | 37.98 us | 476.86 us | 540.82x | 0.26x | 0.02x |
| `sqrt` | 8.40 ns | 1.70 us | 1.44 us | 5.43 us | 29.61 us | 202.86x | 0.31x | 0.06x |
| `sin` | 14.69 ns | 15.38 us | 15.50 us | 13.49 us | 200.90 us | 1046.93x | 1.14x | 0.08x |
| `cos` | 18.40 ns | 8.95 us | 9.17 us | 11.93 us | 96.72 us | 486.59x | 0.75x | 0.09x |
| `tan` | 24.23 ns | 24.54 us | 24.74 us | 28.52 us | 230.43 us | 1012.74x | 0.86x | 0.11x |
| `sinh` | 17.66 ns | 3.37 us | 3.35 us | 3.25 us | 179.85 ns | 190.77x | 1.04x | 18.73x |
| `cosh` | 17.61 ns | 3.20 us | 3.30 us | 7.99 us | 168.13 ns | 181.88x | 0.40x | 19.06x |
| `tanh` | 45.82 ns | 8.18 us | 6.90 us | 3.45 us | 180.93 ns | 178.52x | 2.37x | 45.21x |
| `asin` | 9.03 ns | 1.18 us | 1.14 us | 22.33 us | 166.01 ns | 130.66x | 0.05x | 7.10x |
| `asin abort` | 12.63 ns | 1.18 us | 1.21 us | 22.29 us | 164.92 ns | 93.70x | 0.05x | 7.18x |
| `acos` | 9.51 ns | 1.24 us | 1.24 us | 27.21 us | 169.82 ns | 130.14x | 0.05x | 7.29x |
| `acos abort` | 18.96 ns | 1.27 us | 1.25 us | 27.24 us | 166.68 ns | 66.77x | 0.05x | 7.60x |
| `atan` | 16.18 ns | 1.12 us | 1.23 us | 19.14 us | 164.86 ns | 69.15x | 0.06x | 6.78x |
| `atan abort` | 16.74 ns | 1.10 us | 1.27 us | 19.18 us | 162.45 ns | 65.99x | 0.06x | 6.80x |
| `asinh` | 32.91 ns | 1.14 us | 1.28 us | 42.33 us | 193.60 ns | 34.66x | 0.03x | 5.89x |
| `asinh abort` | 25.00 ns | 1.14 us | 1.27 us | 42.61 us | 191.28 ns | 45.42x | 0.03x | 5.94x |
| `acosh` | 8.32 ns | 1.02 us | 1.17 us | 43.70 us | 167.26 ns | 123.12x | 0.02x | 6.12x |
| `acosh abort` | 12.25 ns | 1.00 us | 1.17 us | 43.62 us | 164.16 ns | 81.96x | 0.02x | 6.11x |
| `atanh` | 9.23 ns | 1.23 us | 1.22 us | 37.62 us | 169.94 ns | 132.85x | 0.03x | 7.22x |
| `atanh abort` | 16.80 ns | 1.29 us | 1.23 us | 37.57 us | 161.79 ns | 76.55x | 0.03x | 7.95x |
| `zero status` | 1.20 ns | 2.29 ns | 2.29 ns | 1.06 ns | 0.93 ns | 1.91x | 2.15x | 2.45x |
| `zero status abort` | 3.31 ns | 58.52 ns | 57.95 ns | 1.06 ns | 0.94 ns | 17.67x | 55.22x | 62.34x |

### Complex Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.93 ns | 112.13 ns | 105.81 ns | 50.34 ns | 20.43 ns | 120.55x | 2.23x | 5.49x |
| `one` | 5.05 ns | 141.48 ns | 132.83 ns | 64.74 ns | 57.59 ns | 28.03x | 2.19x | 2.46x |
| `i` | 0.92 ns | 141.43 ns | 140.75 ns | 66.05 ns | 54.98 ns | 153.39x | 2.14x | 2.57x |
| `free i` | 0.92 ns | 142.01 ns | 140.20 ns | 66.33 ns | 54.80 ns | 153.55x | 2.14x | 2.59x |
| `conjugate` | 2.10 ns | 164.56 ns | 161.54 ns | 40.49 ns | 24.05 ns | 78.28x | 4.06x | 6.84x |
| `norm squared` | 5.75 ns | 1.68 us | 1.67 us | 152.54 ns | 519.37 ns | 292.15x | 11.01x | 3.23x |
| `reciprocal` | 11.29 ns | 3.19 us | 3.27 us | 450.64 ns | 3.26 us | 282.65x | 7.08x | 0.98x |
| `reciprocal checked` | 11.78 ns | 3.21 us | 3.23 us | 442.25 ns | 3.25 us | 272.59x | 7.26x | 0.99x |
| `powi` | 19.00 ns | 13.65 us | 16.79 us | 1.46 us | 4.76 us | 718.11x | 9.35x | 2.87x |
| `powi checked` | 19.15 ns | 13.65 us | 16.84 us | 1.46 us | 4.84 us | 712.67x | 9.34x | 2.82x |
| `div checked` | 28.12 ns | 6.43 us | 6.78 us | 786.17 ns | 4.43 us | 228.58x | 8.18x | 1.45x |
| `div real checked` | 18.33 ns | 870.06 ns | 789.92 ns | 258.50 ns | 1.84 us | 47.47x | 3.37x | 0.47x |
| `from scalar` | 1.40 ns | 131.02 ns | 102.79 ns | 45.43 ns | 23.40 ns | 93.34x | 2.88x | 5.60x |
| `add` | 6.01 ns | 905.63 ns | 902.14 ns | 104.92 ns | 234.30 ns | 150.60x | 8.63x | 3.87x |
| `sub` | 5.98 ns | 1.02 us | 859.65 ns | 116.82 ns | 309.62 ns | 169.75x | 8.69x | 3.28x |
| `neg` | 2.57 ns | 139.17 ns | 140.35 ns | 42.19 ns | 23.30 ns | 54.20x | 3.30x | 5.97x |
| `mul` | 7.43 ns | 3.25 us | 3.47 us | 305.26 ns | 1.00 us | 437.87x | 10.65x | 3.24x |
| `div` | 15.79 ns | 6.28 us | 6.80 us | 795.28 ns | 4.44 us | 397.95x | 7.90x | 1.42x |
| `div real` | 18.23 ns | 801.15 ns | 798.01 ns | 262.63 ns | 1.83 us | 43.95x | 3.05x | 0.44x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.68 ns | 3.66 us | 6.28 us | 315.52 ns | 887.46 ns | 548.05x | 11.60x | 4.12x |
| `vec3 magnitude` | 12.62 ns | 7.86 us | 10.19 us | 5.91 us | 39.59 us | 622.92x | 1.33x | 0.20x |
| `vec3 normalize` | 32.38 ns | 13.12 us | 12.98 us | 6.49 us | 44.03 us | 405.22x | 2.02x | 0.30x |

#### Vector API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.04 ns | 1.35 us | 2.17 us | 71.28 ns | 230.96 ns | 444.62x | 18.94x | 5.84x |
| `vec3 zero` | 1.38 ns | 160.08 ns | 159.15 ns | 58.33 ns | 32.06 ns | 116.02x | 2.74x | 4.99x |
| `vec3 dot abort` | 27.00 ns | 3.78 us | 4.40 us | 258.93 ns | 884.92 ns | 139.93x | 14.59x | 4.27x |
| `vec3 magnitude abort` | 37.95 ns | 7.82 us | 7.09 us | 5.82 us | 39.15 us | 206.12x | 1.34x | 0.20x |
| `vec3 normalize checked` | 32.37 ns | 13.12 us | 10.28 us | 6.49 us | 43.75 us | 405.27x | 2.02x | 0.30x |
| `vec3 normalize checked abort` | 60.92 ns | 13.38 us | 10.65 us | 6.47 us | 43.99 us | 219.59x | 2.07x | 0.30x |
| `vec3 div scalar checked` | 10.23 ns | 1.94 us | 1.84 us | 395.65 ns | 3.13 us | 189.45x | 4.90x | 0.62x |
| `vec3 div scalar checked abort` | 30.00 ns | 2.09 us | 1.94 us | 395.35 ns | 3.08 us | 69.65x | 5.29x | 0.68x |
| `vec3 add` | 7.09 ns | 2.47 us | 2.31 us | 149.59 ns | 375.71 ns | 348.36x | 16.50x | 6.57x |
| `vec3 add scalar` | 6.23 ns | 2.16 us | 2.07 us | 151.77 ns | 446.78 ns | 346.74x | 14.22x | 4.83x |
| `vec3 sub` | 7.15 ns | 2.44 us | 2.37 us | 160.96 ns | 498.38 ns | 341.52x | 15.17x | 4.90x |
| `vec3 sub scalar` | 6.34 ns | 1.94 us | 1.92 us | 150.36 ns | 373.20 ns | 306.49x | 12.93x | 5.21x |
| `vec3 neg` | 3.71 ns | 492.63 ns | 476.80 ns | 57.33 ns | 43.38 ns | 132.95x | 8.59x | 11.36x |
| `vec3 mul scalar` | 6.87 ns | 2.30 us | 2.37 us | 166.12 ns | 561.83 ns | 334.39x | 13.82x | 4.09x |
| `vec3 div scalar` | 9.96 ns | 1.96 us | 1.83 us | 396.94 ns | 3.11 us | 196.95x | 4.94x | 0.63x |
| `vec4 dot` | 7.48 ns | 3.85 us | 3.61 us | 434.26 ns | 1.30 us | 514.75x | 8.87x | 2.96x |
| `vec4 magnitude` | 13.80 ns | 7.55 us | 6.34 us | 6.03 us | 39.91 us | 547.18x | 1.25x | 0.19x |
| `vec4 normalize` | 37.33 ns | 13.23 us | 9.31 us | 6.66 us | 46.12 us | 354.30x | 1.99x | 0.29x |
| `vec4 add` | 7.29 ns | 2.82 us | 2.43 us | 206.71 ns | 507.90 ns | 386.70x | 13.63x | 5.55x |
| `vec4 add scalar` | 6.82 ns | 2.60 us | 2.25 us | 216.59 ns | 590.11 ns | 382.10x | 12.03x | 4.41x |
| `vec4 sub` | 5.13 ns | 2.71 us | 2.32 us | 209.19 ns | 600.60 ns | 528.14x | 12.96x | 4.51x |
| `vec4 sub scalar` | 4.39 ns | 2.31 us | 2.01 us | 204.26 ns | 515.77 ns | 527.22x | 11.33x | 4.49x |
| `vec4 neg` | 4.85 ns | 690.07 ns | 674.41 ns | 79.18 ns | 53.62 ns | 142.28x | 8.71x | 12.87x |
| `vec4 mul scalar` | 7.29 ns | 2.66 us | 2.57 us | 222.94 ns | 723.32 ns | 365.05x | 11.94x | 3.68x |
| `vec4 div scalar` | 14.00 ns | 2.49 us | 1.97 us | 569.62 ns | 4.14 us | 177.79x | 4.37x | 0.60x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 15.91 ns | 8.69 us | 3.58 us | 1.07 us | 2.59 us | 545.83x | 8.11x | 3.35x |
| `mat3 inverse` | 62.67 ns | 37.04 us | 14.71 us | 3.41 us | 11.50 us | 591.01x | 10.85x | 3.22x |
| `mat3 mul mat3` | 77.21 ns | 26.81 us | 13.70 us | 2.94 us | 8.81 us | 347.19x | 9.13x | 3.04x |
| `mat3 transform vec3` | 18.53 ns | 11.74 us | 12.42 us | 1.15 us | 2.88 us | 633.27x | 10.21x | 4.08x |
| `mat4 determinant` | 46.92 ns | 18.40 us | 6.14 us | 5.17 us | 11.31 us | 392.13x | 3.56x | 1.63x |
| `mat4 inverse` | 132.83 ns | 78.47 us | 22.58 us | 11.84 us | 41.23 us | 590.79x | 6.63x | 1.90x |
| `mat4 mul mat4` | 125.90 ns | 38.91 us | 17.22 us | 6.81 us | 20.92 us | 309.01x | 5.71x | 1.86x |
| `mat4 transform vec4` | 29.58 ns | 15.69 us | 4.99 us | 2.03 us | 5.01 us | 530.50x | 7.73x | 3.13x |

#### Matrix API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 34.90 ns | 3.72 us | 6.03 us | 261.51 ns | 717.93 ns | 106.52x | 14.21x | 5.18x |
| `mat3 zero` | 15.84 ns | 769.40 ns | 759.23 ns | 320.62 ns | 232.89 ns | 48.56x | 2.40x | 3.30x |
| `mat3 identity` | 9.89 ns | 853.67 ns | 842.03 ns | 383.92 ns | 322.32 ns | 86.29x | 2.22x | 2.65x |
| `mat3 transpose` | 9.29 ns | 972.09 ns | 962.34 ns | 279.34 ns | 244.57 ns | 104.63x | 3.48x | 3.97x |
| `mat3 reciprocal` | 63.20 ns | 36.65 us | 44.63 us | 3.16 us | 11.20 us | 579.87x | 11.61x | 3.27x |
| `mat3 reciprocal checked` | 141.78 ns | 61.89 us | 61.60 us | 3.15 us | 11.14 us | 436.54x | 19.68x | 5.55x |
| `mat3 inverse checked` | 142.85 ns | 62.07 us | 62.07 us | 3.15 us | 11.06 us | 434.50x | 19.68x | 5.61x |
| `mat3 inverse checked abort` | 163.40 ns | 62.46 us | 61.83 us | 3.15 us | 11.19 us | 382.24x | 19.84x | 5.58x |
| `mat3 powi` | 162.66 ns | 87.42 us | 112.33 us | 7.15 us | 20.71 us | 537.45x | 12.23x | 4.22x |
| `mat3 powi checked` | 161.62 ns | 87.13 us | 113.35 us | 7.14 us | 20.77 us | 539.11x | 12.20x | 4.19x |
| `mat3 powi checked abort` | 160.98 ns | 86.40 us | 113.29 us | 7.11 us | 20.78 us | 536.75x | 12.15x | 4.16x |
| `mat3 div scalar checked` | 24.44 ns | 5.95 us | 5.93 us | 1.56 us | 11.92 us | 243.63x | 3.81x | 0.50x |
| `mat3 div scalar checked abort` | 40.09 ns | 6.00 us | 6.08 us | 1.57 us | 11.82 us | 149.62x | 3.83x | 0.51x |
| `mat3 div matrix checked` | 222.13 ns | 128.27 us | 126.31 us | 5.76 us | 20.04 us | 577.45x | 22.27x | 6.40x |
| `mat3 div matrix checked abort` | 244.07 ns | 128.66 us | 126.06 us | 5.73 us | 20.33 us | 527.16x | 22.47x | 6.33x |
| `mat3 add` | 14.76 ns | 6.15 us | 6.96 us | 549.97 ns | 1.34 us | 416.56x | 11.18x | 4.57x |
| `mat3 add scalar` | 10.61 ns | 6.15 us | 7.01 us | 840.40 ns | 1.66 us | 580.29x | 7.32x | 3.71x |
| `mat3 sub` | 13.02 ns | 6.31 us | 6.97 us | 549.74 ns | 1.55 us | 484.40x | 11.48x | 4.06x |
| `mat3 sub scalar` | 10.54 ns | 6.89 us | 6.78 us | 845.64 ns | 1.62 us | 653.51x | 8.15x | 4.25x |
| `mat3 neg` | 10.70 ns | 2.02 us | 2.16 us | 517.63 ns | 520.94 ns | 188.80x | 3.90x | 3.88x |
| `mat3 mul scalar` | 12.32 ns | 6.19 us | 7.07 us | 883.58 ns | 1.92 us | 502.14x | 7.00x | 3.22x |
| `mat3 div scalar` | 24.53 ns | 5.85 us | 5.97 us | 1.57 us | 11.88 us | 238.62x | 3.74x | 0.49x |
| `mat3 div matrix` | 138.41 ns | 106.45 us | 108.87 us | 5.74 us | 20.15 us | 769.08x | 18.55x | 5.28x |
| `mat3 bitxor` | 162.76 ns | 87.53 us | 112.76 us | 7.11 us | 20.78 us | 537.80x | 12.30x | 4.21x |
| `mat4 zero` | 11.29 ns | 1.27 us | 1.41 us | 624.44 ns | 401.11 ns | 112.63x | 2.04x | 3.17x |
| `mat4 identity` | 11.14 ns | 1.45 us | 1.59 us | 668.62 ns | 527.99 ns | 129.90x | 2.16x | 2.74x |
| `mat4 transpose` | 10.06 ns | 1.66 us | 1.81 us | 557.06 ns | 386.52 ns | 165.54x | 2.99x | 4.31x |
| `mat4 reciprocal` | 133.09 ns | 77.25 us | 77.62 us | 11.37 us | 40.99 us | 580.48x | 6.79x | 1.88x |
| `mat4 reciprocal checked` | 199.62 ns | 129.13 us | 109.37 us | 11.35 us | 41.03 us | 646.90x | 11.38x | 3.15x |
| `mat4 powi` | 323.77 ns | 138.57 us | 157.16 us | 16.57 us | 46.14 us | 428.00x | 8.36x | 3.00x |
| `mat4 powi checked` | 325.54 ns | 140.37 us | 158.80 us | 16.63 us | 46.92 us | 431.18x | 8.44x | 2.99x |
| `mat4 add` | 52.93 ns | 7.58 us | 7.88 us | 1.00 us | 2.13 us | 143.14x | 7.56x | 3.57x |
| `mat4 add scalar` | 16.15 ns | 8.90 us | 9.92 us | 1.59 us | 2.74 us | 550.98x | 5.61x | 3.25x |
| `mat4 sub` | 38.40 ns | 8.06 us | 8.18 us | 1.06 us | 2.62 us | 209.99x | 7.63x | 3.08x |
| `mat4 sub scalar` | 14.74 ns | 10.34 us | 9.68 us | 1.56 us | 2.68 us | 701.65x | 6.61x | 3.86x |
| `mat4 neg` | 13.58 ns | 3.40 us | 3.64 us | 980.31 ns | 811.81 ns | 250.29x | 3.47x | 4.19x |
| `mat4 mul scalar` | 50.26 ns | 8.94 us | 9.58 us | 1.65 us | 3.33 us | 177.91x | 5.40x | 2.69x |
| `mat4 div scalar` | 32.56 ns | 9.10 us | 8.53 us | 2.89 us | 20.52 us | 279.38x | 3.15x | 0.44x |
| `mat4 div matrix` | 294.40 ns | 218.05 us | 185.61 us | 17.72 us | 61.61 us | 740.65x | 12.31x | 3.54x |
| `mat4 bitxor` | 323.31 ns | 139.12 us | 156.43 us | 16.58 us | 47.23 us | 430.29x | 8.39x | 2.95x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 11.84 us |
| `astro sin 160` | 13.02 us |
| `astro sin 192` | 12.93 us |
| `astro sin 256` | 15.80 us |
| `arp sin 128` | 266.25 us |
| `arp sin 160` | 332.39 us |
| `arp sin 192` | 389.50 us |
| `arp sin 256` | 592.51 us |
