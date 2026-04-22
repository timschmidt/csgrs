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
| `sin 0.1` | 10.75 ns | 2.40 us | 2.40 us | 10.56 us | 126.97 us | 223.64x | 0.23x | 0.02x |
| `cos 0.1` | 11.50 ns | 223.39 ns | 226.14 ns | 10.18 us | 25.80 us | 19.42x | 0.02x | 0.01x |
| `sin 1.23456789` | 11.56 ns | 1.86 us | 1.88 us | 12.01 us | 266.04 us | 161.01x | 0.15x | 0.01x |
| `cos 1.23456789` | 11.87 ns | 429.41 ns | 437.46 ns | 10.14 us | 172.94 us | 36.19x | 0.04x | 0.00x |
| `sin 1e6` | 12.49 ns | 35.62 us | 35.78 us | 15.83 us | 270.19 us | 2852.56x | 2.25x | 0.13x |
| `cos 1e6` | 12.22 ns | 22.14 us | 22.20 us | 13.88 us | 171.60 us | 1811.86x | 1.59x | 0.13x |
| `sin 1e30` | 65.13 ns | 187.58 us | 188.05 us | 18.17 us | 275.64 us | 2880.36x | 10.32x | 0.68x |
| `cos 1e30` | 67.73 ns | 138.87 us | 142.49 us | 15.56 us | 171.75 us | 2050.28x | 8.92x | 0.81x |
| `sin pi_7` | 11.60 ns | 2.40 us | 4.28 us | 11.67 us | 126.90 us | 206.93x | 0.21x | 0.02x |
| `cos pi_7` | 11.53 ns | 232.48 ns | 4.42 us | 10.28 us | 27.64 us | 20.15x | 0.02x | 0.01x |
| `sin 1000pi_eps` | 11.54 ns | 23.09 us | 34.61 us | 15.69 us | 266.41 us | 2001.40x | 1.47x | 0.09x |
| `cos 1000pi_eps` | 12.31 ns | 13.85 us | 25.09 us | 13.93 us | 154.45 us | 1125.03x | 0.99x | 0.09x |

#### Scalar API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.47 ns | 41.66 ns | 40.35 ns | 24.43 ns | 14.06 ns | 89.56x | 1.71x | 2.96x |
| `one` | 0.47 ns | 75.79 ns | 75.69 ns | 38.69 ns | 47.01 ns | 161.78x | 1.96x | 1.61x |
| `e` | 0.47 ns | 1.17 us | 1.23 us | 47.83 ns | 52.91 us | 2509.44x | 24.55x | 0.02x |
| `pi` | 0.47 ns | 175.60 ns | 187.54 ns | 42.94 ns | 103.89 us | 374.44x | 4.09x | 0.00x |
| `tau` | 0.46 ns | 299.90 ns | 317.98 ns | 117.22 ns | 104.95 us | 645.19x | 2.56x | 0.00x |
| `add` | 5.13 ns | 404.13 ns | 390.19 ns | 51.57 ns | 100.29 ns | 78.84x | 7.84x | 4.03x |
| `sub` | 5.11 ns | 658.69 ns | 464.08 ns | 57.22 ns | 145.46 ns | 128.95x | 11.51x | 4.53x |
| `neg` | 4.98 ns | 78.87 ns | 63.37 ns | 22.90 ns | 16.21 ns | 15.85x | 3.44x | 4.87x |
| `mul` | 5.33 ns | 461.14 ns | 352.31 ns | 57.42 ns | 170.53 ns | 86.47x | 8.03x | 2.70x |
| `div` | 8.69 ns | 431.42 ns | 279.88 ns | 133.88 ns | 880.15 ns | 49.63x | 3.22x | 0.49x |
| `reciprocal` | 8.48 ns | 121.66 ns | 96.35 ns | 158.34 ns | 1.29 us | 14.34x | 0.77x | 0.09x |
| `reciprocal checked` | 8.68 ns | 107.60 ns | 99.52 ns | 159.56 ns | 1.29 us | 12.39x | 0.67x | 0.08x |
| `reciprocal checked abort` | 26.09 ns | 160.10 ns | 157.21 ns | 158.40 ns | 1.28 us | 6.14x | 1.01x | 0.13x |
| `pow` | 24.80 ns | 11.54 us | 8.34 us | 57.05 us | 381.18 us | 465.38x | 0.20x | 0.03x |
| `powi` | 6.08 ns | 2.86 us | 3.06 us | 292.64 ns | 1.37 us | 470.10x | 9.76x | 2.09x |
| `exp` | 10.13 ns | 1.06 us | 1.11 us | 14.40 us | 71.77 us | 104.55x | 0.07x | 0.01x |
| `ln` | 11.70 ns | 2.89 us | 3.17 us | 31.93 us | 264.48 us | 247.23x | 0.09x | 0.01x |
| `log10` | 21.76 ns | 8.96 us | 8.44 us | 37.88 us | 478.53 us | 411.72x | 0.24x | 0.02x |
| `log10 abort` | 17.60 ns | 8.93 us | 8.44 us | 37.72 us | 476.99 us | 507.10x | 0.24x | 0.02x |
| `sqrt` | 8.32 ns | 1.72 us | 1.40 us | 5.45 us | 29.33 us | 206.71x | 0.32x | 0.06x |
| `sin` | 14.69 ns | 15.44 us | 15.68 us | 13.69 us | 200.12 us | 1051.35x | 1.13x | 0.08x |
| `cos` | 18.16 ns | 8.96 us | 9.04 us | 12.12 us | 97.04 us | 493.50x | 0.74x | 0.09x |
| `tan` | 24.18 ns | 24.97 us | 25.28 us | 31.20 us | 228.38 us | 1032.67x | 0.80x | 0.11x |
| `sinh` | 17.66 ns | 3.36 us | 3.54 us | 3.26 us | 172.18 ns | 190.07x | 1.03x | 19.50x |
| `cosh` | 17.65 ns | 3.29 us | 3.42 us | 8.05 us | 160.68 ns | 186.23x | 0.41x | 20.46x |
| `tanh` | 24.39 ns | 5.76 us | 6.17 us | 3.47 us | 172.28 ns | 236.11x | 1.66x | 33.42x |
| `asin` | 8.98 ns | 1.22 us | 1.15 us | 22.22 us | 163.51 ns | 135.49x | 0.05x | 7.44x |
| `asin abort` | 12.67 ns | 1.25 us | 1.14 us | 22.05 us | 163.26 ns | 98.67x | 0.06x | 7.66x |
| `acos` | 9.51 ns | 1.27 us | 1.21 us | 27.35 us | 161.15 ns | 133.81x | 0.05x | 7.89x |
| `acos abort` | 18.92 ns | 1.27 us | 1.20 us | 27.53 us | 161.09 ns | 67.16x | 0.05x | 7.89x |
| `atan` | 16.20 ns | 1.13 us | 1.25 us | 19.33 us | 156.02 ns | 69.71x | 0.06x | 7.24x |
| `atan abort` | 16.77 ns | 1.13 us | 1.23 us | 19.07 us | 155.79 ns | 67.66x | 0.06x | 7.28x |
| `asinh` | 32.71 ns | 1.16 us | 1.26 us | 42.46 us | 189.30 ns | 35.61x | 0.03x | 6.15x |
| `asinh abort` | 24.96 ns | 1.16 us | 1.26 us | 42.45 us | 187.38 ns | 46.35x | 0.03x | 6.17x |
| `acosh` | 8.30 ns | 1.05 us | 1.17 us | 44.29 us | 161.88 ns | 126.97x | 0.02x | 6.51x |
| `acosh abort` | 12.23 ns | 1.06 us | 1.20 us | 44.16 us | 161.06 ns | 86.41x | 0.02x | 6.56x |
| `atanh` | 12.77 ns | 1.28 us | 1.22 us | 37.99 us | 165.68 ns | 100.58x | 0.03x | 7.75x |
| `atanh abort` | 16.51 ns | 1.29 us | 1.22 us | 37.58 us | 165.60 ns | 78.40x | 0.03x | 7.82x |
| `zero status` | 1.20 ns | 2.29 ns | 2.29 ns | 1.05 ns | 1.45 ns | 1.90x | 2.17x | 1.57x |
| `zero status abort` | 3.31 ns | 76.03 ns | 57.35 ns | 1.07 ns | 1.46 ns | 22.97x | 70.82x | 52.05x |

### Complex Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.92 ns | 108.60 ns | 105.06 ns | 50.26 ns | 20.88 ns | 118.21x | 2.16x | 5.20x |
| `one` | 4.96 ns | 137.41 ns | 135.21 ns | 65.00 ns | 60.47 ns | 27.70x | 2.11x | 2.27x |
| `i` | 0.92 ns | 136.71 ns | 133.68 ns | 65.41 ns | 54.02 ns | 148.68x | 2.09x | 2.53x |
| `free i` | 0.92 ns | 137.09 ns | 135.00 ns | 65.39 ns | 54.13 ns | 148.33x | 2.10x | 2.53x |
| `conjugate` | 2.10 ns | 169.86 ns | 165.56 ns | 40.71 ns | 24.77 ns | 80.81x | 4.17x | 6.86x |
| `norm squared` | 5.68 ns | 1.66 us | 1.66 us | 153.02 ns | 507.37 ns | 292.28x | 10.84x | 3.27x |
| `reciprocal` | 18.02 ns | 3.38 us | 3.29 us | 441.51 ns | 3.23 us | 187.70x | 7.66x | 1.05x |
| `reciprocal checked` | 14.71 ns | 3.36 us | 3.16 us | 444.58 ns | 3.23 us | 228.46x | 7.56x | 1.04x |
| `powi` | 18.96 ns | 13.62 us | 16.59 us | 1.45 us | 4.67 us | 718.58x | 9.38x | 2.92x |
| `powi checked` | 19.11 ns | 13.61 us | 16.55 us | 1.45 us | 4.69 us | 712.29x | 9.38x | 2.90x |
| `div checked` | 18.49 ns | 6.58 us | 6.81 us | 787.88 ns | 4.29 us | 356.04x | 8.35x | 1.53x |
| `div real checked` | 9.24 ns | 937.18 ns | 858.97 ns | 259.51 ns | 1.81 us | 101.40x | 3.61x | 0.52x |
| `from scalar` | 1.40 ns | 129.00 ns | 100.41 ns | 45.20 ns | 22.29 ns | 92.40x | 2.85x | 5.79x |
| `add` | 5.95 ns | 942.01 ns | 902.79 ns | 107.84 ns | 220.67 ns | 158.19x | 8.74x | 4.27x |
| `sub` | 5.96 ns | 1.05 us | 883.72 ns | 118.06 ns | 282.02 ns | 176.74x | 8.92x | 3.73x |
| `neg` | 2.55 ns | 173.33 ns | 153.64 ns | 42.34 ns | 24.58 ns | 67.84x | 4.09x | 7.05x |
| `mul` | 7.39 ns | 3.37 us | 3.53 us | 300.83 ns | 942.99 ns | 456.58x | 11.22x | 3.58x |
| `div` | 17.90 ns | 6.60 us | 6.95 us | 793.41 ns | 4.29 us | 368.90x | 8.32x | 1.54x |
| `div real` | 10.01 ns | 941.67 ns | 856.12 ns | 261.91 ns | 1.81 us | 94.03x | 3.60x | 0.52x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.62 ns | 3.67 us | 6.40 us | 310.85 ns | 857.18 ns | 554.08x | 11.81x | 4.28x |
| `vec3 magnitude` | 12.54 ns | 7.93 us | 10.19 us | 6.01 us | 39.42 us | 632.34x | 1.32x | 0.20x |
| `vec3 normalize` | 32.07 ns | 13.35 us | 13.09 us | 6.40 us | 43.95 us | 416.32x | 2.09x | 0.30x |

#### Vector API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.03 ns | 1.38 us | 2.25 us | 71.36 ns | 232.29 ns | 456.15x | 19.37x | 5.95x |
| `vec3 zero` | 1.38 ns | 169.62 ns | 172.48 ns | 59.15 ns | 32.38 ns | 123.12x | 2.87x | 5.24x |
| `vec3 dot abort` | 27.08 ns | 3.77 us | 4.48 us | 262.12 ns | 836.25 ns | 139.30x | 14.39x | 4.51x |
| `vec3 magnitude abort` | 37.89 ns | 7.79 us | 7.17 us | 6.13 us | 39.50 us | 205.54x | 1.27x | 0.20x |
| `vec3 normalize checked` | 32.21 ns | 13.24 us | 10.31 us | 6.42 us | 43.56 us | 411.06x | 2.06x | 0.30x |
| `vec3 normalize checked abort` | 61.41 ns | 13.43 us | 10.54 us | 6.41 us | 43.75 us | 218.66x | 2.09x | 0.31x |
| `vec3 div scalar checked` | 10.34 ns | 1.95 us | 1.92 us | 399.26 ns | 3.09 us | 188.19x | 4.87x | 0.63x |
| `vec3 div scalar checked abort` | 30.04 ns | 2.10 us | 1.92 us | 395.02 ns | 3.08 us | 70.05x | 5.33x | 0.68x |
| `vec3 add` | 6.63 ns | 2.39 us | 2.28 us | 149.48 ns | 364.64 ns | 359.95x | 15.98x | 6.55x |
| `vec3 add scalar` | 6.26 ns | 2.16 us | 2.12 us | 149.80 ns | 436.63 ns | 345.21x | 14.42x | 4.95x |
| `vec3 sub` | 6.63 ns | 2.44 us | 2.32 us | 160.31 ns | 490.54 ns | 367.57x | 15.20x | 4.97x |
| `vec3 sub scalar` | 6.26 ns | 2.00 us | 1.90 us | 147.80 ns | 369.92 ns | 318.70x | 13.51x | 5.40x |
| `vec3 neg` | 3.70 ns | 484.37 ns | 497.93 ns | 57.93 ns | 43.92 ns | 130.81x | 8.36x | 11.03x |
| `vec3 mul scalar` | 6.81 ns | 2.31 us | 2.44 us | 165.83 ns | 531.43 ns | 338.70x | 13.92x | 4.34x |
| `vec3 div scalar` | 18.63 ns | 2.04 us | 1.86 us | 408.31 ns | 3.10 us | 109.71x | 5.01x | 0.66x |
| `vec4 dot` | 7.35 ns | 3.79 us | 3.60 us | 437.86 ns | 1.32 us | 516.14x | 8.66x | 2.88x |
| `vec4 magnitude` | 13.81 ns | 7.63 us | 6.26 us | 6.08 us | 39.94 us | 552.53x | 1.25x | 0.19x |
| `vec4 normalize` | 37.07 ns | 13.42 us | 9.20 us | 6.75 us | 45.93 us | 362.09x | 1.99x | 0.29x |
| `vec4 add` | 7.21 ns | 2.83 us | 2.36 us | 204.80 ns | 488.64 ns | 392.27x | 13.81x | 5.79x |
| `vec4 add scalar` | 6.78 ns | 2.54 us | 2.23 us | 215.04 ns | 563.60 ns | 374.48x | 11.81x | 4.51x |
| `vec4 sub` | 4.98 ns | 2.66 us | 2.32 us | 213.31 ns | 563.52 ns | 532.99x | 12.45x | 4.71x |
| `vec4 sub scalar` | 4.50 ns | 2.32 us | 2.02 us | 208.43 ns | 500.22 ns | 514.90x | 11.12x | 4.63x |
| `vec4 neg` | 4.88 ns | 683.61 ns | 679.10 ns | 79.17 ns | 55.31 ns | 140.14x | 8.63x | 12.36x |
| `vec4 mul scalar` | 7.16 ns | 2.67 us | 2.63 us | 222.70 ns | 705.04 ns | 372.05x | 11.97x | 3.78x |
| `vec4 div scalar` | 14.21 ns | 2.30 us | 2.02 us | 534.35 ns | 3.98 us | 161.65x | 4.30x | 0.58x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 15.84 ns | 8.75 us | 3.63 us | 1.08 us | 2.69 us | 552.27x | 8.13x | 3.25x |
| `mat3 inverse` | 62.93 ns | 32.53 us | 12.45 us | 3.39 us | 11.21 us | 516.84x | 9.60x | 2.90x |
| `mat3 mul mat3` | 72.77 ns | 26.78 us | 13.54 us | 2.93 us | 8.54 us | 367.97x | 9.12x | 3.14x |
| `mat3 transform vec3` | 18.54 ns | 11.65 us | 12.46 us | 1.15 us | 2.74 us | 628.32x | 10.15x | 4.26x |
| `mat4 determinant` | 46.27 ns | 17.79 us | 5.95 us | 5.13 us | 11.28 us | 384.49x | 3.47x | 1.58x |
| `mat4 inverse` | 131.34 ns | 79.80 us | 21.77 us | 11.81 us | 40.70 us | 607.57x | 6.76x | 1.96x |
| `mat4 mul mat4` | 118.97 ns | 39.07 us | 15.72 us | 6.80 us | 19.65 us | 328.39x | 5.74x | 1.99x |
| `mat4 transform vec4` | 29.54 ns | 15.43 us | 5.05 us | 2.06 us | 5.07 us | 522.23x | 7.50x | 3.05x |

#### Matrix API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 35.33 ns | 3.68 us | 6.07 us | 278.09 ns | 703.82 ns | 104.08x | 13.22x | 5.22x |
| `mat3 zero` | 15.67 ns | 732.74 ns | 781.33 ns | 295.55 ns | 252.92 ns | 46.76x | 2.48x | 2.90x |
| `mat3 identity` | 9.85 ns | 897.53 ns | 918.72 ns | 357.07 ns | 358.32 ns | 91.12x | 2.51x | 2.50x |
| `mat3 transpose` | 8.63 ns | 925.51 ns | 991.45 ns | 273.22 ns | 213.47 ns | 107.30x | 3.39x | 4.34x |
| `mat3 reciprocal` | 62.69 ns | 32.68 us | 38.67 us | 3.08 us | 11.05 us | 521.27x | 10.60x | 2.96x |
| `mat3 reciprocal checked` | 142.19 ns | 62.04 us | 61.77 us | 3.08 us | 11.03 us | 436.31x | 20.13x | 5.63x |
| `mat3 inverse checked` | 141.84 ns | 62.16 us | 62.03 us | 3.07 us | 11.14 us | 438.24x | 20.22x | 5.58x |
| `mat3 inverse checked abort` | 154.52 ns | 62.02 us | 60.88 us | 3.08 us | 11.12 us | 401.41x | 20.14x | 5.58x |
| `mat3 powi` | 161.12 ns | 86.01 us | 112.92 us | 6.99 us | 20.03 us | 533.84x | 12.30x | 4.29x |
| `mat3 powi checked` | 161.09 ns | 86.15 us | 112.55 us | 6.96 us | 19.99 us | 534.78x | 12.39x | 4.31x |
| `mat3 powi checked abort` | 162.35 ns | 85.99 us | 112.55 us | 6.99 us | 20.02 us | 529.67x | 12.31x | 4.29x |
| `mat3 div scalar checked` | 24.64 ns | 5.83 us | 5.90 us | 1.57 us | 11.76 us | 236.42x | 3.72x | 0.50x |
| `mat3 div scalar checked abort` | 39.65 ns | 5.93 us | 6.02 us | 1.57 us | 11.70 us | 149.44x | 3.77x | 0.51x |
| `mat3 div matrix checked` | 119.23 ns | 79.39 us | 75.97 us | 5.57 us | 19.59 us | 665.80x | 14.24x | 4.05x |
| `mat3 div matrix checked abort` | 131.68 ns | 79.61 us | 75.88 us | 5.57 us | 19.65 us | 604.55x | 14.30x | 4.05x |
| `mat3 add` | 14.29 ns | 5.84 us | 6.67 us | 550.47 ns | 1.32 us | 409.03x | 10.62x | 4.44x |
| `mat3 add scalar` | 10.13 ns | 6.01 us | 6.85 us | 822.00 ns | 1.61 us | 593.76x | 7.32x | 3.74x |
| `mat3 sub` | 13.44 ns | 6.05 us | 6.51 us | 559.37 ns | 1.50 us | 450.11x | 10.82x | 4.05x |
| `mat3 sub scalar` | 10.75 ns | 6.72 us | 6.83 us | 842.71 ns | 1.56 us | 625.39x | 7.98x | 4.32x |
| `mat3 neg` | 10.21 ns | 2.09 us | 2.13 us | 496.23 ns | 497.11 ns | 205.05x | 4.22x | 4.21x |
| `mat3 mul scalar` | 12.24 ns | 6.19 us | 6.88 us | 837.33 ns | 1.90 us | 505.70x | 7.39x | 3.26x |
| `mat3 div scalar` | 24.72 ns | 5.85 us | 5.92 us | 1.58 us | 11.78 us | 236.58x | 3.71x | 0.50x |
| `mat3 div matrix` | 115.26 ns | 79.48 us | 75.70 us | 5.59 us | 19.72 us | 689.59x | 14.23x | 4.03x |
| `mat3 bitxor` | 161.80 ns | 86.62 us | 112.05 us | 7.00 us | 20.23 us | 535.35x | 12.37x | 4.28x |
| `mat4 zero` | 13.16 ns | 1.19 us | 1.22 us | 580.52 ns | 417.65 ns | 90.33x | 2.05x | 2.85x |
| `mat4 identity` | 10.33 ns | 1.37 us | 1.37 us | 624.10 ns | 520.24 ns | 132.41x | 2.19x | 2.63x |
| `mat4 transpose` | 9.16 ns | 1.71 us | 1.78 us | 497.84 ns | 350.31 ns | 186.75x | 3.43x | 4.88x |
| `mat4 reciprocal` | 131.25 ns | 77.61 us | 76.00 us | 11.25 us | 40.33 us | 591.32x | 6.90x | 1.92x |
| `mat4 reciprocal checked` | 199.87 ns | 130.80 us | 106.76 us | 11.28 us | 40.53 us | 654.45x | 11.60x | 3.23x |
| `mat4 powi` | 321.39 ns | 139.54 us | 157.07 us | 16.53 us | 44.83 us | 434.19x | 8.44x | 3.11x |
| `mat4 powi checked` | 319.16 ns | 138.13 us | 154.84 us | 16.54 us | 45.22 us | 432.80x | 8.35x | 3.05x |
| `mat4 add` | 49.54 ns | 7.99 us | 8.39 us | 983.78 ns | 2.04 us | 161.23x | 8.12x | 3.91x |
| `mat4 add scalar` | 16.19 ns | 8.97 us | 9.61 us | 1.56 us | 2.64 us | 554.05x | 5.75x | 3.39x |
| `mat4 sub` | 36.65 ns | 8.43 us | 8.49 us | 1.04 us | 2.54 us | 230.17x | 8.10x | 3.32x |
| `mat4 sub scalar` | 14.44 ns | 10.50 us | 9.59 us | 1.57 us | 2.65 us | 726.81x | 6.70x | 3.96x |
| `mat4 neg` | 14.26 ns | 3.47 us | 3.49 us | 981.73 ns | 869.55 ns | 243.54x | 3.54x | 4.00x |
| `mat4 mul scalar` | 48.93 ns | 9.09 us | 9.45 us | 1.62 us | 3.25 us | 185.67x | 5.60x | 2.80x |
| `mat4 div scalar` | 32.30 ns | 9.24 us | 8.51 us | 2.85 us | 20.24 us | 286.21x | 3.24x | 0.46x |
| `mat4 div matrix` | 190.82 ns | 157.51 us | 126.06 us | 17.75 us | 61.13 us | 825.42x | 8.87x | 2.58x |
| `mat4 bitxor` | 321.39 ns | 139.13 us | 156.65 us | 16.48 us | 44.57 us | 432.89x | 8.44x | 3.12x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.12 us |
| `astro sin 160` | 13.19 us |
| `astro sin 192` | 13.22 us |
| `astro sin 256` | 15.82 us |
| `arp sin 128` | 266.60 us |
| `arp sin 160` | 331.42 us |
| `arp sin 192` | 392.23 us |
| `arp sin 256` | 594.25 us |
