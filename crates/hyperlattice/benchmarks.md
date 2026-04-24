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
- Borrowed API operator coverage for scalar, vector, matrix, matrix/vector, and complex reference combinations.

## Benchmark Results

The following Criterion median estimates were collected on an AMD Ryzen 7 5800X3D on Fedora. Values are formatted to two digits after the decimal.

### Scalar Operations

#### Scalar Trigonometric Comparisons

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `sin 0.1` | 10.76 ns | 228.19 ns | 229.79 ns | 10.60 us | 125.73 us | 21.20x | 0.02x | 0.00x |
| `cos 0.1` | 11.54 ns | 225.03 ns | 226.14 ns | 9.96 us | 25.61 us | 19.50x | 0.02x | 0.01x |
| `sin 1.23456789` | 11.58 ns | 1.97 us | 1.97 us | 11.91 us | 264.93 us | 170.29x | 0.17x | 0.01x |
| `cos 1.23456789` | 11.85 ns | 445.55 ns | 437.22 ns | 10.00 us | 172.90 us | 37.61x | 0.04x | 0.00x |
| `sin 1e6` | 12.40 ns | 21.45 us | 21.78 us | 15.76 us | 270.49 us | 1729.57x | 1.36x | 0.08x |
| `cos 1e6` | 12.14 ns | 20.74 us | 21.05 us | 13.43 us | 173.27 us | 1708.63x | 1.54x | 0.12x |
| `sin 1e30` | 64.98 ns | 135.00 us | 137.97 us | 17.77 us | 277.75 us | 2077.66x | 7.60x | 0.49x |
| `cos 1e30` | 67.70 ns | 132.27 us | 131.82 us | 14.91 us | 173.64 us | 1953.81x | 8.87x | 0.76x |
| `sin pi_7` | 11.56 ns | 228.87 ns | 2.33 us | 11.66 us | 125.25 us | 19.80x | 0.02x | 0.00x |
| `cos pi_7` | 11.49 ns | 226.85 ns | 5.80 us | 10.39 us | 27.69 us | 19.74x | 0.02x | 0.01x |
| `sin 1000pi_eps` | 11.53 ns | 13.55 us | 23.84 us | 15.75 us | 267.20 us | 1175.41x | 0.86x | 0.05x |
| `cos 1000pi_eps` | 12.13 ns | 13.09 us | 23.78 us | 13.32 us | 155.50 us | 1078.91x | 0.98x | 0.08x |

#### Scalar API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.47 ns | 62.44 ns | 62.68 ns | 24.67 ns | 14.57 ns | 133.50x | 2.53x | 4.29x |
| `one` | 0.47 ns | 74.43 ns | 87.79 ns | 38.89 ns | 46.91 ns | 159.23x | 1.91x | 1.59x |
| `e` | 0.47 ns | 232.85 ns | 232.79 ns | 52.34 ns | 53.63 us | 500.01x | 4.45x | 0.00x |
| `pi` | 0.47 ns | 233.15 ns | 231.27 ns | 43.04 ns | 103.31 us | 498.78x | 5.42x | 0.00x |
| `tau` | 0.47 ns | 738.49 ns | 759.91 ns | 114.03 ns | 104.29 us | 1577.62x | 6.48x | 0.01x |
| `add` | 5.10 ns | 466.58 ns | 457.09 ns | 49.86 ns | 108.63 ns | 91.46x | 9.36x | 4.30x |
| `sub` | 5.09 ns | 719.15 ns | 521.58 ns | 56.38 ns | 152.11 ns | 141.42x | 12.76x | 4.73x |
| `neg` | 4.99 ns | 59.73 ns | 59.91 ns | 22.51 ns | 16.27 ns | 11.97x | 2.65x | 3.67x |
| `mul` | 5.38 ns | 510.45 ns | 403.34 ns | 56.52 ns | 161.69 ns | 94.92x | 9.03x | 3.16x |
| `div` | 8.26 ns | 448.57 ns | 325.46 ns | 134.92 ns | 893.58 ns | 54.31x | 3.32x | 0.50x |
| `reciprocal` | 8.53 ns | 131.50 ns | 133.75 ns | 156.70 ns | 1.29 us | 15.41x | 0.84x | 0.10x |
| `reciprocal checked` | 8.74 ns | 95.96 ns | 95.81 ns | 156.84 ns | 1.30 us | 10.98x | 0.61x | 0.07x |
| `reciprocal checked abort` | 27.17 ns | 172.39 ns | 173.18 ns | 156.59 ns | 1.25 us | 6.34x | 1.10x | 0.14x |
| `pow` | 24.83 ns | 13.06 us | 9.21 us | 56.26 us | 382.26 us | 525.99x | 0.23x | 0.03x |
| `powi` | 5.77 ns | 2.50 us | 2.72 us | 290.72 ns | 1.33 us | 433.72x | 8.60x | 1.88x |
| `exp` | 10.17 ns | 350.78 ns | 349.34 ns | 14.30 us | 71.55 us | 34.48x | 0.02x | 0.00x |
| `ln` | 11.80 ns | 2.68 us | 2.15 us | 31.62 us | 263.72 us | 226.81x | 0.08x | 0.01x |
| `log10` | 21.97 ns | 8.20 us | 5.89 us | 36.47 us | 479.91 us | 373.15x | 0.22x | 0.02x |
| `log10 abort` | 17.52 ns | 8.50 us | 5.88 us | 37.92 us | 479.94 us | 485.36x | 0.22x | 0.02x |
| `sqrt` | 8.37 ns | 2.10 us | 1.88 us | 5.42 us | 29.55 us | 250.94x | 0.39x | 0.07x |
| `sin` | 14.78 ns | 10.42 us | 9.14 us | 13.62 us | 198.94 us | 705.03x | 0.77x | 0.05x |
| `cos` | 18.15 ns | 9.30 us | 8.90 us | 11.78 us | 97.10 us | 512.75x | 0.79x | 0.10x |
| `tan` | 24.31 ns | 9.04 us | 8.98 us | 29.89 us | 228.04 us | 371.65x | 0.30x | 0.04x |
| `sinh` | 17.68 ns | 3.63 us | 3.58 us | 3.30 us | 180.59 ns | 205.43x | 1.10x | 20.11x |
| `cosh` | 17.73 ns | 3.50 us | 3.49 us | 8.09 us | 165.33 ns | 197.19x | 0.43x | 21.15x |
| `tanh` | 22.97 ns | 7.58 us | 7.57 us | 3.54 us | 177.03 ns | 329.99x | 2.14x | 42.81x |
| `asin` | 8.96 ns | 1.44 us | 1.38 us | 22.07 us | 166.13 ns | 160.91x | 0.07x | 8.67x |
| `asin abort` | 12.66 ns | 1.46 us | 1.38 us | 21.93 us | 166.75 ns | 115.11x | 0.07x | 8.74x |
| `acos` | 9.56 ns | 1.47 us | 1.46 us | 27.70 us | 168.76 ns | 153.45x | 0.05x | 8.69x |
| `acos abort` | 20.31 ns | 1.48 us | 1.46 us | 27.83 us | 168.79 ns | 72.80x | 0.05x | 8.76x |
| `atan` | 16.28 ns | 1.31 us | 1.60 us | 19.13 us | 163.91 ns | 80.23x | 0.07x | 7.97x |
| `atan abort` | 22.69 ns | 1.31 us | 1.60 us | 18.98 us | 163.67 ns | 57.64x | 0.07x | 7.99x |
| `asinh` | 32.89 ns | 1.33 us | 1.62 us | 42.23 us | 191.35 ns | 40.54x | 0.03x | 6.97x |
| `asinh abort` | 24.97 ns | 1.35 us | 1.65 us | 42.17 us | 194.25 ns | 54.15x | 0.03x | 6.96x |
| `acosh` | 8.33 ns | 1.16 us | 1.46 us | 43.44 us | 164.49 ns | 138.84x | 0.03x | 7.03x |
| `acosh abort` | 12.20 ns | 1.15 us | 1.46 us | 43.53 us | 164.97 ns | 94.42x | 0.03x | 6.98x |
| `atanh` | 9.29 ns | 1.49 us | 1.44 us | 37.17 us | 174.61 ns | 160.56x | 0.04x | 8.55x |
| `atanh abort` | 16.87 ns | 1.49 us | 1.47 us | 37.06 us | 176.09 ns | 88.29x | 0.04x | 8.46x |
| `zero status` | 1.20 ns | 2.64 ns | 2.64 ns | 1.06 ns | 1.47 ns | 2.20x | 2.50x | 1.79x |
| `zero status abort` | 3.31 ns | 57.32 ns | 56.83 ns | 1.05 ns | 1.48 ns | 17.34x | 54.54x | 38.76x |

### Complex Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.93 ns | 135.79 ns | 134.11 ns | 48.89 ns | 20.32 ns | 146.55x | 2.78x | 6.68x |
| `one` | 4.96 ns | 141.82 ns | 142.33 ns | 62.74 ns | 60.33 ns | 28.58x | 2.26x | 2.35x |
| `i` | 0.92 ns | 139.77 ns | 139.03 ns | 64.55 ns | 57.69 ns | 152.04x | 2.17x | 2.42x |
| `free i` | 0.92 ns | 139.17 ns | 139.83 ns | 64.38 ns | 58.10 ns | 151.27x | 2.16x | 2.40x |
| `conjugate` | 2.10 ns | 156.21 ns | 155.61 ns | 40.47 ns | 25.38 ns | 74.37x | 3.86x | 6.15x |
| `norm squared` | 5.74 ns | 1.60 us | 1.59 us | 154.77 ns | 514.36 ns | 278.26x | 10.31x | 3.10x |
| `reciprocal` | 15.98 ns | 3.19 us | 3.16 us | 450.18 ns | 3.08 us | 199.44x | 7.08x | 1.04x |
| `reciprocal checked` | 14.02 ns | 3.21 us | 3.22 us | 453.00 ns | 3.12 us | 229.31x | 7.10x | 1.03x |
| `powi` | 17.31 ns | 12.03 us | 14.94 us | 1.47 us | 4.80 us | 694.92x | 8.15x | 2.51x |
| `powi checked` | 17.31 ns | 12.07 us | 14.87 us | 1.48 us | 4.85 us | 697.20x | 8.18x | 2.49x |
| `div checked` | 19.52 ns | 6.84 us | 7.07 us | 785.21 ns | 4.19 us | 350.64x | 8.72x | 1.63x |
| `div real checked` | 9.25 ns | 842.94 ns | 805.04 ns | 267.82 ns | 1.82 us | 91.11x | 3.15x | 0.46x |
| `from scalar` | 1.40 ns | 144.17 ns | 114.75 ns | 44.95 ns | 23.45 ns | 103.23x | 3.21x | 6.15x |
| `add` | 6.06 ns | 1.07 us | 1.04 us | 103.59 ns | 230.20 ns | 177.28x | 10.37x | 4.67x |
| `sub` | 6.11 ns | 1.18 us | 963.76 ns | 118.98 ns | 306.93 ns | 193.38x | 9.93x | 3.85x |
| `neg` | 2.56 ns | 156.87 ns | 126.62 ns | 42.23 ns | 23.19 ns | 61.26x | 3.71x | 6.76x |
| `mul` | 7.49 ns | 3.57 us | 3.79 us | 304.58 ns | 947.74 ns | 476.12x | 11.71x | 3.76x |
| `div` | 18.15 ns | 6.77 us | 7.11 us | 795.33 ns | 4.35 us | 372.74x | 8.51x | 1.56x |
| `div real` | 9.93 ns | 822.07 ns | 805.86 ns | 267.29 ns | 1.83 us | 82.82x | 3.08x | 0.45x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.69 ns | 3.37 us | 6.02 us | 319.82 ns | 862.24 ns | 503.53x | 10.53x | 3.90x |
| `vec3 magnitude` | 9.57 ns | 7.87 us | 11.28 us | 6.03 us | 38.89 us | 821.74x | 1.30x | 0.20x |
| `vec3 normalize` | 24.97 ns | 13.13 us | 13.72 us | 6.33 us | 44.05 us | 525.61x | 2.07x | 0.30x |

#### Vector API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.04 ns | 1.36 us | 2.33 us | 70.64 ns | 233.21 ns | 448.20x | 19.26x | 5.83x |
| `vec3 zero` | 1.39 ns | 201.85 ns | 222.02 ns | 59.01 ns | 31.99 ns | 145.65x | 3.42x | 6.31x |
| `vec3 dot abort` | 27.34 ns | 4.07 us | 4.73 us | 266.58 ns | 827.41 ns | 149.04x | 15.28x | 4.92x |
| `vec3 magnitude abort` | 38.65 ns | 8.47 us | 7.89 us | 5.85 us | 39.17 us | 219.28x | 1.45x | 0.22x |
| `vec3 normalize checked` | 25.75 ns | 13.16 us | 10.32 us | 6.30 us | 43.28 us | 511.09x | 2.09x | 0.30x |
| `vec3 normalize checked abort` | 60.81 ns | 13.82 us | 11.19 us | 6.31 us | 43.38 us | 227.27x | 2.19x | 0.32x |
| `vec3 div scalar checked` | 15.48 ns | 1.82 us | 1.65 us | 399.11 ns | 3.11 us | 117.37x | 4.55x | 0.59x |
| `vec3 div scalar checked abort` | 29.67 ns | 1.89 us | 1.76 us | 398.62 ns | 3.01 us | 63.85x | 4.75x | 0.63x |
| `vec3 add` | 7.14 ns | 2.04 us | 1.90 us | 148.21 ns | 371.09 ns | 285.14x | 13.74x | 5.49x |
| `vec3 add scalar` | 6.25 ns | 1.94 us | 1.81 us | 151.16 ns | 445.58 ns | 310.48x | 12.83x | 4.35x |
| `vec3 sub` | 7.14 ns | 2.18 us | 2.00 us | 160.21 ns | 507.54 ns | 305.77x | 13.62x | 4.30x |
| `vec3 sub scalar` | 6.28 ns | 1.71 us | 1.69 us | 147.05 ns | 371.54 ns | 271.52x | 11.60x | 4.59x |
| `vec3 neg` | 3.71 ns | 280.08 ns | 252.92 ns | 57.30 ns | 54.02 ns | 75.53x | 4.89x | 5.19x |
| `vec3 mul scalar` | 6.81 ns | 2.05 us | 2.20 us | 164.08 ns | 514.49 ns | 301.19x | 12.51x | 3.99x |
| `vec3 div scalar` | 9.89 ns | 1.76 us | 1.69 us | 402.37 ns | 3.03 us | 177.54x | 4.36x | 0.58x |
| `vec4 dot` | 7.12 ns | 3.80 us | 3.63 us | 432.44 ns | 1.32 us | 534.41x | 8.80x | 2.88x |
| `vec4 magnitude` | 12.88 ns | 7.98 us | 7.51 us | 6.00 us | 39.48 us | 619.50x | 1.33x | 0.20x |
| `vec4 normalize` | 35.55 ns | 13.55 us | 9.79 us | 6.53 us | 45.12 us | 381.04x | 2.07x | 0.30x |
| `vec4 add` | 7.26 ns | 2.37 us | 2.04 us | 203.68 ns | 501.80 ns | 326.32x | 11.63x | 4.72x |
| `vec4 add scalar` | 6.78 ns | 2.11 us | 1.81 us | 210.41 ns | 586.47 ns | 310.76x | 10.01x | 3.59x |
| `vec4 sub` | 5.13 ns | 2.30 us | 1.91 us | 213.92 ns | 599.33 ns | 448.84x | 10.76x | 3.84x |
| `vec4 sub scalar` | 4.50 ns | 1.85 us | 1.59 us | 201.95 ns | 522.61 ns | 411.03x | 9.16x | 3.54x |
| `vec4 neg` | 4.87 ns | 342.25 ns | 321.70 ns | 78.33 ns | 51.74 ns | 70.26x | 4.37x | 6.61x |
| `vec4 mul scalar` | 7.23 ns | 2.26 us | 2.19 us | 219.90 ns | 684.44 ns | 311.98x | 10.26x | 3.30x |
| `vec4 div scalar` | 13.96 ns | 2.02 us | 1.72 us | 536.03 ns | 3.92 us | 144.68x | 3.77x | 0.52x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 15.85 ns | 9.20 us | 4.25 us | 1.07 us | 2.64 us | 580.35x | 8.63x | 3.48x |
| `mat3 inverse` | 78.58 ns | 33.30 us | 13.12 us | 3.49 us | 11.05 us | 423.77x | 9.55x | 3.01x |
| `mat3 mul mat3` | 76.27 ns | 27.22 us | 13.86 us | 2.89 us | 8.20 us | 356.91x | 9.41x | 3.32x |
| `mat3 transform vec3` | 15.68 ns | 11.13 us | 12.52 us | 1.15 us | 2.76 us | 709.66x | 9.71x | 4.04x |
| `mat4 determinant` | 44.46 ns | 17.99 us | 8.19 us | 5.13 us | 11.24 us | 404.53x | 3.51x | 1.60x |
| `mat4 inverse` | 147.65 ns | 80.07 us | 21.23 us | 12.07 us | 40.73 us | 542.27x | 6.63x | 1.97x |
| `mat4 mul mat4` | 120.23 ns | 39.43 us | 16.99 us | 6.86 us | 19.17 us | 327.93x | 5.75x | 2.06x |
| `mat4 transform vec4` | 25.03 ns | 14.30 us | 5.26 us | 2.05 us | 5.10 us | 571.28x | 6.96x | 2.80x |

#### Matrix API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 34.32 ns | 3.66 us | 6.07 us | 264.31 ns | 707.10 ns | 106.68x | 13.85x | 5.18x |
| `mat3 zero` | 16.65 ns | 1.04 us | 1.03 us | 326.12 ns | 272.04 ns | 62.49x | 3.19x | 3.82x |
| `mat3 identity` | 9.98 ns | 1.04 us | 1.05 us | 380.50 ns | 371.19 ns | 104.57x | 2.74x | 2.81x |
| `mat3 transpose` | 8.74 ns | 880.76 ns | 926.44 ns | 311.95 ns | 207.15 ns | 100.72x | 2.82x | 4.25x |
| `mat3 reciprocal` | 78.77 ns | 34.35 us | 40.01 us | 3.11 us | 10.73 us | 436.03x | 11.04x | 3.20x |
| `mat3 reciprocal checked` | 78.70 ns | 34.57 us | 40.14 us | 3.11 us | 10.90 us | 439.26x | 11.13x | 3.17x |
| `mat3 inverse checked` | 78.74 ns | 33.18 us | 40.44 us | 3.12 us | 10.92 us | 421.43x | 10.65x | 3.04x |
| `mat3 inverse checked abort` | 118.91 ns | 34.56 us | 40.12 us | 3.13 us | 10.86 us | 290.66x | 11.05x | 3.18x |
| `mat3 powi` | 154.05 ns | 78.85 us | 104.52 us | 7.19 us | 20.38 us | 511.86x | 10.96x | 3.87x |
| `mat3 powi checked` | 154.18 ns | 78.47 us | 103.89 us | 7.07 us | 20.86 us | 508.95x | 11.10x | 3.76x |
| `mat3 powi checked abort` | 153.90 ns | 77.48 us | 102.75 us | 7.15 us | 20.90 us | 503.44x | 10.84x | 3.71x |
| `mat3 div scalar checked` | 24.72 ns | 5.58 us | 5.66 us | 1.58 us | 11.63 us | 225.52x | 3.52x | 0.48x |
| `mat3 div scalar checked abort` | 40.38 ns | 5.60 us | 5.59 us | 1.58 us | 11.76 us | 138.76x | 3.54x | 0.48x |
| `mat3 div matrix checked` | 119.27 ns | 76.38 us | 76.18 us | 5.69 us | 19.54 us | 640.43x | 13.42x | 3.91x |
| `mat3 div matrix checked abort` | 131.90 ns | 80.98 us | 76.38 us | 5.70 us | 19.57 us | 613.92x | 14.20x | 4.14x |
| `mat3 add` | 14.41 ns | 5.25 us | 5.95 us | 548.89 ns | 1.34 us | 364.21x | 9.56x | 3.90x |
| `mat3 add scalar` | 10.58 ns | 5.12 us | 6.13 us | 844.04 ns | 1.66 us | 483.86x | 6.07x | 3.09x |
| `mat3 sub` | 13.87 ns | 5.48 us | 5.92 us | 551.36 ns | 1.54 us | 394.99x | 9.94x | 3.57x |
| `mat3 sub scalar` | 11.07 ns | 5.78 us | 5.86 us | 850.26 ns | 1.62 us | 522.05x | 6.80x | 3.57x |
| `mat3 neg` | 10.02 ns | 1.09 us | 1.08 us | 510.70 ns | 525.90 ns | 108.53x | 2.13x | 2.07x |
| `mat3 mul scalar` | 12.29 ns | 5.32 us | 6.01 us | 868.45 ns | 1.98 us | 432.79x | 6.12x | 2.69x |
| `mat3 div scalar` | 24.89 ns | 5.50 us | 5.61 us | 1.58 us | 11.94 us | 220.95x | 3.49x | 0.46x |
| `mat3 div matrix` | 114.94 ns | 78.97 us | 75.22 us | 5.64 us | 19.79 us | 687.05x | 14.01x | 3.99x |
| `mat3 bitxor` | 154.29 ns | 78.07 us | 103.33 us | 7.20 us | 20.44 us | 506.00x | 10.85x | 3.82x |
| `mat4 zero` | 13.06 ns | 1.51 us | 1.46 us | 593.98 ns | 430.68 ns | 115.43x | 2.54x | 3.50x |
| `mat4 identity` | 10.10 ns | 1.51 us | 1.59 us | 689.01 ns | 558.72 ns | 149.97x | 2.20x | 2.71x |
| `mat4 transpose` | 9.20 ns | 1.68 us | 1.67 us | 534.26 ns | 354.28 ns | 183.20x | 3.15x | 4.76x |
| `mat4 reciprocal` | 166.78 ns | 80.68 us | 79.49 us | 11.51 us | 40.19 us | 483.77x | 7.01x | 2.01x |
| `mat4 reciprocal checked` | 149.42 ns | 80.76 us | 79.71 us | 11.53 us | 40.36 us | 540.46x | 7.01x | 2.00x |
| `mat4 powi` | 253.25 ns | 126.59 us | 143.00 us | 16.67 us | 45.25 us | 499.87x | 7.59x | 2.80x |
| `mat4 powi checked` | 249.96 ns | 126.35 us | 142.21 us | 16.73 us | 45.83 us | 505.50x | 7.55x | 2.76x |
| `mat4 add` | 51.62 ns | 6.79 us | 7.08 us | 998.29 ns | 2.12 us | 131.59x | 6.80x | 3.21x |
| `mat4 add scalar` | 16.15 ns | 7.42 us | 8.06 us | 1.59 us | 2.80 us | 459.45x | 4.66x | 2.65x |
| `mat4 sub` | 38.06 ns | 7.22 us | 7.33 us | 1.07 us | 2.61 us | 189.68x | 6.72x | 2.76x |
| `mat4 sub scalar` | 14.77 ns | 8.66 us | 7.90 us | 1.60 us | 2.72 us | 585.95x | 5.42x | 3.19x |
| `mat4 neg` | 14.30 ns | 1.87 us | 1.98 us | 981.66 ns | 794.91 ns | 131.01x | 1.91x | 2.36x |
| `mat4 mul scalar` | 20.69 ns | 7.45 us | 7.99 us | 1.67 us | 3.19 us | 359.89x | 4.46x | 2.33x |
| `mat4 div scalar` | 32.52 ns | 8.37 us | 7.80 us | 2.90 us | 20.31 us | 257.36x | 2.89x | 0.41x |
| `mat4 div matrix` | 191.07 ns | 153.39 us | 123.48 us | 17.84 us | 60.75 us | 802.82x | 8.60x | 2.53x |
| `mat4 bitxor` | 248.92 ns | 125.82 us | 140.27 us | 16.70 us | 45.49 us | 505.46x | 7.53x | 2.77x |

### Borrowed API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 8.93 ns | 368.21 ns | 368.20 ns | - | - | 41.25x | - | - |
| `scalar add ref_owned` | 5.48 ns | 376.92 ns | 362.33 ns | - | - | 68.74x | - | - |
| `scalar add refs` | 5.33 ns | 373.48 ns | 353.67 ns | - | - | 70.09x | - | - |
| `scalar sub owned_ref` | 9.02 ns | 641.01 ns | 413.37 ns | - | - | 71.06x | - | - |
| `scalar sub ref_owned` | 5.66 ns | 720.13 ns | 491.84 ns | - | - | 127.22x | - | - |
| `scalar sub refs` | 5.54 ns | 626.32 ns | 407.60 ns | - | - | 113.12x | - | - |
| `scalar mul owned_ref` | 9.62 ns | 427.79 ns | 296.40 ns | - | - | 44.45x | - | - |
| `scalar mul ref_owned` | 7.76 ns | 428.21 ns | 295.45 ns | - | - | 55.18x | - | - |
| `scalar mul refs` | 6.25 ns | 422.48 ns | 290.51 ns | - | - | 67.64x | - | - |
| `scalar div owned_ref` | 15.39 ns | 342.68 ns | 199.14 ns | - | - | 22.26x | - | - |
| `scalar div ref_owned` | 8.00 ns | 441.22 ns | 285.56 ns | - | - | 55.17x | - | - |
| `scalar div refs` | 6.82 ns | 334.10 ns | 200.77 ns | - | - | 48.99x | - | - |
| `vec3 add refs` | 6.33 ns | 1.74 us | 1.55 us | - | - | 274.96x | - | - |
| `vec3 sub refs` | 6.19 ns | 1.86 us | 1.62 us | - | - | 300.27x | - | - |
| `vec3 neg ref` | 3.28 ns | 298.86 ns | 251.20 ns | - | - | 91.16x | - | - |
| `vec3 add_scalar_ref` | 6.43 ns | 1.77 us | 1.67 us | - | - | 275.86x | - | - |
| `vec3 sub_scalar_ref` | 6.49 ns | 1.56 us | 1.46 us | - | - | 240.79x | - | - |
| `vec3 mul_scalar_ref` | 6.92 ns | 1.92 us | 1.97 us | - | - | 277.02x | - | - |
| `vec3 div_scalar_ref` | 9.68 ns | 1.75 us | 1.62 us | - | - | 180.67x | - | - |
| `vec4 add refs` | 6.36 ns | 1.90 us | 1.58 us | - | - | 299.51x | - | - |
| `vec4 sub refs` | 3.06 ns | 1.80 us | 1.47 us | - | - | 587.82x | - | - |
| `vec4 neg ref` | 4.23 ns | 375.00 ns | 367.54 ns | - | - | 88.67x | - | - |
| `vec4 add_scalar_ref` | 6.89 ns | 1.96 us | 1.68 us | - | - | 284.09x | - | - |
| `vec4 sub_scalar_ref` | 4.20 ns | 1.72 us | 1.44 us | - | - | 408.85x | - | - |
| `vec4 mul_scalar_ref` | 7.34 ns | 2.10 us | 2.03 us | - | - | 286.75x | - | - |
| `vec4 div_scalar_ref` | 13.87 ns | 1.98 us | 1.68 us | - | - | 142.55x | - | - |
| `mat3 add refs` | 12.43 ns | 3.74 us | 4.55 us | - | - | 301.20x | - | - |
| `mat3 sub refs` | 12.32 ns | 3.97 us | 4.43 us | - | - | 322.56x | - | - |
| `mat3 mul refs` | 79.51 ns | 25.39 us | 33.78 us | - | - | 319.33x | - | - |
| `mat3 div refs` | 115.82 ns | 79.50 us | 76.43 us | - | - | 686.44x | - | - |
| `mat3 neg ref` | 9.49 ns | 930.57 ns | 893.45 ns | - | - | 98.09x | - | - |
| `mat3 add_scalar_ref` | 10.23 ns | 4.97 us | 5.89 us | - | - | 485.55x | - | - |
| `mat3 sub_scalar_ref` | 12.81 ns | 5.60 us | 5.65 us | - | - | 437.44x | - | - |
| `mat3 mul_scalar_ref` | 11.89 ns | 5.21 us | 6.00 us | - | - | 438.30x | - | - |
| `mat3 div_scalar_ref` | 24.38 ns | 5.45 us | 5.58 us | - | - | 223.50x | - | - |
| `mat4 add refs` | 50.04 ns | 4.10 us | 4.46 us | - | - | 81.97x | - | - |
| `mat4 sub refs` | 36.84 ns | 4.56 us | 4.63 us | - | - | 123.89x | - | - |
| `mat4 mul refs` | 118.43 ns | 35.99 us | 41.18 us | - | - | 303.91x | - | - |
| `mat4 div refs` | 192.39 ns | 158.31 us | 127.62 us | - | - | 822.90x | - | - |
| `mat4 neg ref` | 12.52 ns | 1.59 us | 1.67 us | - | - | 126.69x | - | - |
| `mat4 add_scalar_ref` | 14.10 ns | 7.30 us | 7.95 us | - | - | 517.66x | - | - |
| `mat4 sub_scalar_ref` | 14.96 ns | 8.57 us | 7.74 us | - | - | 572.97x | - | - |
| `mat4 mul_scalar_ref` | 48.80 ns | 7.45 us | 7.77 us | - | - | 152.75x | - | - |
| `mat4 div_scalar_ref` | 29.36 ns | 8.33 us | 7.66 us | - | - | 283.55x | - | - |
| `mat3 transform_vec refs` | 16.16 ns | 10.90 us | 12.46 us | - | - | 674.52x | - | - |
| `mat4 transform_vec refs` | 25.70 ns | 13.73 us | 12.62 us | - | - | 534.20x | - | - |
| `complex add refs` | 7.62 ns | 824.87 ns | 854.45 ns | - | - | 108.24x | - | - |
| `complex sub refs` | 7.99 ns | 969.80 ns | 832.86 ns | - | - | 121.45x | - | - |
| `complex mul refs` | 8.02 ns | 3.12 us | 3.37 us | - | - | 389.05x | - | - |
| `complex div refs` | 18.75 ns | 6.17 us | 6.58 us | - | - | 328.78x | - | - |
| `complex neg ref` | 2.35 ns | 109.06 ns | 105.90 ns | - | - | 46.40x | - | - |
| `complex div_real_ref` | 9.94 ns | 825.44 ns | 803.66 ns | - | - | 83.07x | - | - |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.01 us |
| `astro sin 160` | 13.05 us |
| `astro sin 192` | 13.10 us |
| `astro sin 256` | 15.95 us |
| `arp sin 128` | 265.13 us |
| `arp sin 160` | 332.54 us |
| `arp sin 192` | 391.65 us |
| `arp sin 256` | 589.88 us |
