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
| `sin 0.1` | 10.87 ns | 189.70 ns | 2.37 us | 10.55 us | 126.29 us | 17.45x | 0.02x | 0.00x |
| `cos 0.1` | 11.54 ns | 235.92 ns | 238.04 ns | 10.11 us | 25.70 us | 20.45x | 0.02x | 0.01x |
| `sin 1.23456789` | 11.59 ns | 1.95 us | 1.82 us | 12.15 us | 264.91 us | 167.80x | 0.16x | 0.01x |
| `cos 1.23456789` | 11.83 ns | 430.31 ns | 436.66 ns | 10.14 us | 171.40 us | 36.37x | 0.04x | 0.00x |
| `sin 1e6` | 12.54 ns | 21.96 us | 34.87 us | 15.99 us | 269.47 us | 1750.52x | 1.37x | 0.08x |
| `cos 1e6` | 12.24 ns | 21.78 us | 21.64 us | 13.37 us | 172.88 us | 1779.98x | 1.63x | 0.13x |
| `sin 1e30` | 65.27 ns | 139.91 us | 186.12 us | 17.98 us | 277.12 us | 2143.63x | 7.78x | 0.50x |
| `cos 1e30` | 70.45 ns | 138.38 us | 138.78 us | 15.69 us | 172.28 us | 1964.32x | 8.82x | 0.80x |
| `sin pi_7` | 11.61 ns | 191.99 ns | 4.26 us | 11.65 us | 124.42 us | 16.54x | 0.02x | 0.00x |
| `cos pi_7` | 11.53 ns | 236.24 ns | 4.39 us | 10.31 us | 27.87 us | 20.48x | 0.02x | 0.01x |
| `sin 1000pi_eps` | 11.62 ns | 13.81 us | 33.81 us | 16.05 us | 266.42 us | 1188.31x | 0.86x | 0.05x |
| `cos 1000pi_eps` | 12.36 ns | 13.65 us | 24.54 us | 13.64 us | 155.84 us | 1104.76x | 1.00x | 0.09x |

#### Scalar API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.47 ns | 49.67 ns | 50.64 ns | 26.75 ns | 14.46 ns | 105.18x | 1.86x | 3.43x |
| `one` | 0.47 ns | 69.13 ns | 56.36 ns | 39.94 ns | 47.05 ns | 146.86x | 1.73x | 1.47x |
| `e` | 0.47 ns | 814.30 ns | 871.52 ns | 51.93 ns | 52.75 us | 1734.42x | 15.68x | 0.02x |
| `pi` | 0.47 ns | 220.37 ns | 218.10 ns | 45.13 ns | 102.26 us | 467.06x | 4.88x | 0.00x |
| `tau` | 0.47 ns | 348.53 ns | 377.38 ns | 120.69 ns | 103.85 us | 741.70x | 2.89x | 0.00x |
| `add` | 5.12 ns | 529.32 ns | 417.00 ns | 50.78 ns | 101.05 ns | 103.37x | 10.42x | 5.24x |
| `sub` | 5.18 ns | 954.19 ns | 492.67 ns | 56.00 ns | 146.10 ns | 184.15x | 17.04x | 6.53x |
| `neg` | 5.01 ns | 80.48 ns | 68.14 ns | 22.82 ns | 16.55 ns | 16.05x | 3.53x | 4.86x |
| `mul` | 5.36 ns | 553.24 ns | 350.90 ns | 56.38 ns | 170.76 ns | 103.14x | 9.81x | 3.24x |
| `div` | 8.81 ns | 419.24 ns | 259.10 ns | 138.45 ns | 881.46 ns | 47.57x | 3.03x | 0.48x |
| `reciprocal` | 8.81 ns | 104.67 ns | 130.38 ns | 160.58 ns | 1.25 us | 11.88x | 0.65x | 0.08x |
| `reciprocal checked` | 9.06 ns | 100.19 ns | 124.10 ns | 160.81 ns | 1.24 us | 11.06x | 0.62x | 0.08x |
| `reciprocal checked abort` | 26.95 ns | 169.07 ns | 169.18 ns | 160.91 ns | 1.24 us | 6.27x | 1.05x | 0.14x |
| `pow` | 25.26 ns | 12.13 us | 8.37 us | 57.54 us | 380.55 us | 480.08x | 0.21x | 0.03x |
| `powi` | 6.18 ns | 2.46 us | 2.68 us | 298.08 ns | 1.26 us | 398.72x | 8.27x | 1.95x |
| `exp` | 10.37 ns | 1.06 us | 1.11 us | 14.42 us | 71.78 us | 102.42x | 0.07x | 0.01x |
| `ln` | 11.38 ns | 2.94 us | 3.14 us | 32.06 us | 263.11 us | 258.59x | 0.09x | 0.01x |
| `log10` | 22.26 ns | 10.00 us | 8.51 us | 37.32 us | 478.69 us | 449.32x | 0.27x | 0.02x |
| `log10 abort` | 17.06 ns | 9.87 us | 8.49 us | 37.68 us | 477.01 us | 578.61x | 0.26x | 0.02x |
| `sqrt` | 8.80 ns | 1.69 us | 1.42 us | 5.36 us | 29.58 us | 192.61x | 0.32x | 0.06x |
| `sin` | 14.88 ns | 9.16 us | 15.70 us | 13.63 us | 199.00 us | 615.30x | 0.67x | 0.05x |
| `cos` | 18.13 ns | 8.87 us | 8.97 us | 11.69 us | 96.87 us | 489.21x | 0.76x | 0.09x |
| `tan` | 24.72 ns | 24.79 us | 25.02 us | 29.28 us | 226.85 us | 1002.71x | 0.85x | 0.11x |
| `sinh` | 17.95 ns | 3.74 us | 3.54 us | 3.31 us | 170.67 ns | 208.12x | 1.13x | 21.89x |
| `cosh` | 17.93 ns | 3.22 us | 3.42 us | 7.95 us | 158.30 ns | 179.68x | 0.40x | 20.35x |
| `tanh` | 23.10 ns | 5.71 us | 6.18 us | 3.52 us | 168.99 ns | 247.05x | 1.62x | 33.77x |
| `asin` | 9.46 ns | 1.23 us | 1.13 us | 21.93 us | 162.05 ns | 129.72x | 0.06x | 7.57x |
| `asin abort` | 16.34 ns | 1.25 us | 1.15 us | 21.94 us | 162.95 ns | 76.30x | 0.06x | 7.65x |
| `acos` | 9.88 ns | 1.22 us | 1.19 us | 27.82 us | 160.67 ns | 123.10x | 0.04x | 7.57x |
| `acos abort` | 20.07 ns | 1.27 us | 1.19 us | 27.76 us | 161.32 ns | 63.44x | 0.05x | 7.89x |
| `atan` | 16.30 ns | 1.12 us | 1.23 us | 19.23 us | 157.45 ns | 68.50x | 0.06x | 7.09x |
| `atan abort` | 16.83 ns | 1.14 us | 1.25 us | 19.44 us | 158.29 ns | 67.79x | 0.06x | 7.21x |
| `asinh` | 32.59 ns | 1.14 us | 1.27 us | 42.00 us | 185.56 ns | 35.11x | 0.03x | 6.17x |
| `asinh abort` | 25.18 ns | 1.16 us | 1.30 us | 41.88 us | 183.67 ns | 46.19x | 0.03x | 6.33x |
| `acosh` | 8.42 ns | 1.04 us | 1.18 us | 43.59 us | 160.27 ns | 124.06x | 0.02x | 6.52x |
| `acosh abort` | 12.34 ns | 1.03 us | 1.17 us | 43.36 us | 161.74 ns | 83.29x | 0.02x | 6.35x |
| `atanh` | 9.39 ns | 1.27 us | 1.22 us | 37.27 us | 166.36 ns | 135.28x | 0.03x | 7.64x |
| `atanh abort` | 16.98 ns | 1.30 us | 1.23 us | 37.36 us | 165.75 ns | 76.48x | 0.03x | 7.84x |
| `zero status` | 1.21 ns | 2.29 ns | 2.29 ns | 1.07 ns | 0.94 ns | 1.90x | 2.15x | 2.44x |
| `zero status abort` | 3.32 ns | 72.84 ns | 56.63 ns | 1.08 ns | 0.94 ns | 21.96x | 67.72x | 77.58x |

### Complex Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.93 ns | 146.20 ns | 144.53 ns | 46.60 ns | 20.82 ns | 157.46x | 3.14x | 7.02x |
| `one` | 5.20 ns | 150.75 ns | 150.11 ns | 62.31 ns | 62.84 ns | 28.96x | 2.42x | 2.40x |
| `i` | 0.93 ns | 158.84 ns | 151.94 ns | 63.50 ns | 56.48 ns | 171.31x | 2.50x | 2.81x |
| `free i` | 0.93 ns | 149.85 ns | 150.28 ns | 63.39 ns | 61.32 ns | 161.75x | 2.36x | 2.44x |
| `conjugate` | 2.10 ns | 172.07 ns | 169.99 ns | 40.90 ns | 23.96 ns | 81.98x | 4.21x | 7.18x |
| `norm squared` | 5.74 ns | 1.70 us | 1.67 us | 159.33 ns | 503.72 ns | 295.56x | 10.64x | 3.37x |
| `reciprocal` | 18.60 ns | 3.37 us | 3.30 us | 456.30 ns | 3.05 us | 181.33x | 7.39x | 1.11x |
| `reciprocal checked` | 14.03 ns | 3.34 us | 3.34 us | 456.30 ns | 3.10 us | 238.45x | 7.33x | 1.08x |
| `powi` | 23.39 ns | 12.21 us | 15.16 us | 1.45 us | 4.59 us | 522.30x | 8.44x | 2.66x |
| `powi checked` | 23.41 ns | 12.24 us | 15.18 us | 1.45 us | 4.66 us | 522.97x | 8.41x | 2.63x |
| `div checked` | 18.46 ns | 6.71 us | 6.83 us | 814.05 ns | 4.31 us | 363.30x | 8.24x | 1.55x |
| `div real checked` | 9.27 ns | 951.72 ns | 843.10 ns | 269.18 ns | 1.83 us | 102.62x | 3.54x | 0.52x |
| `from scalar` | 1.40 ns | 133.95 ns | 117.24 ns | 44.50 ns | 22.15 ns | 95.48x | 3.01x | 6.05x |
| `add` | 6.03 ns | 1.10 us | 892.37 ns | 104.67 ns | 242.17 ns | 182.02x | 10.49x | 4.54x |
| `sub` | 6.05 ns | 1.49 us | 846.64 ns | 117.64 ns | 302.11 ns | 246.76x | 12.69x | 4.94x |
| `neg` | 2.56 ns | 313.46 ns | 139.44 ns | 41.16 ns | 23.69 ns | 122.51x | 7.62x | 13.23x |
| `mul` | 11.24 ns | 3.98 us | 3.50 us | 312.31 ns | 926.77 ns | 354.59x | 12.76x | 4.30x |
| `div` | 17.45 ns | 7.48 us | 6.82 us | 813.21 ns | 4.29 us | 428.88x | 9.20x | 1.74x |
| `div real` | 10.02 ns | 893.96 ns | 833.67 ns | 269.76 ns | 1.81 us | 89.26x | 3.31x | 0.49x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.65 ns | 3.71 us | 6.44 us | 317.54 ns | 919.39 ns | 558.13x | 11.69x | 4.04x |
| `vec3 magnitude` | 9.60 ns | 7.73 us | 10.20 us | 5.88 us | 40.07 us | 804.86x | 1.31x | 0.19x |
| `vec3 normalize` | 29.39 ns | 12.92 us | 12.86 us | 6.44 us | 43.79 us | 439.53x | 2.01x | 0.29x |

#### Vector API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.05 ns | 1.35 us | 2.21 us | 69.47 ns | 234.21 ns | 443.29x | 19.47x | 5.77x |
| `vec3 zero` | 1.38 ns | 218.96 ns | 206.45 ns | 63.93 ns | 33.26 ns | 158.25x | 3.42x | 6.58x |
| `vec3 dot abort` | 27.39 ns | 3.81 us | 4.42 us | 254.47 ns | 866.80 ns | 139.03x | 14.96x | 4.39x |
| `vec3 magnitude abort` | 39.05 ns | 7.79 us | 6.97 us | 5.96 us | 39.07 us | 199.52x | 1.31x | 0.20x |
| `vec3 normalize checked` | 25.47 ns | 12.95 us | 10.12 us | 6.41 us | 43.82 us | 508.63x | 2.02x | 0.30x |
| `vec3 normalize checked abort` | 60.16 ns | 13.41 us | 10.47 us | 6.51 us | 43.47 us | 222.90x | 2.06x | 0.31x |
| `vec3 div scalar checked` | 10.33 ns | 1.85 us | 1.70 us | 401.16 ns | 3.09 us | 179.32x | 4.62x | 0.60x |
| `vec3 div scalar checked abort` | 29.14 ns | 1.96 us | 1.78 us | 401.93 ns | 3.09 us | 67.11x | 4.86x | 0.63x |
| `vec3 add` | 6.46 ns | 2.30 us | 1.81 us | 150.37 ns | 368.32 ns | 355.76x | 15.29x | 6.24x |
| `vec3 add scalar` | 6.26 ns | 2.15 us | 1.84 us | 152.23 ns | 438.02 ns | 342.89x | 14.10x | 4.90x |
| `vec3 sub` | 6.54 ns | 2.83 us | 1.89 us | 164.09 ns | 486.06 ns | 433.64x | 17.27x | 5.83x |
| `vec3 sub scalar` | 6.32 ns | 2.25 us | 1.75 us | 147.78 ns | 366.24 ns | 355.92x | 15.22x | 6.14x |
| `vec3 neg` | 3.73 ns | 381.66 ns | 290.84 ns | 55.87 ns | 44.66 ns | 102.40x | 6.83x | 8.55x |
| `vec3 mul scalar` | 6.84 ns | 2.06 us | 2.14 us | 159.00 ns | 511.16 ns | 301.02x | 12.94x | 4.03x |
| `vec3 div scalar` | 16.97 ns | 1.85 us | 1.73 us | 411.51 ns | 3.12 us | 108.78x | 4.49x | 0.59x |
| `vec4 dot` | 7.14 ns | 3.89 us | 3.63 us | 447.90 ns | 1.24 us | 544.77x | 8.68x | 3.13x |
| `vec4 magnitude` | 12.84 ns | 7.52 us | 6.34 us | 6.03 us | 39.73 us | 585.42x | 1.25x | 0.19x |
| `vec4 normalize` | 35.63 ns | 13.13 us | 8.91 us | 6.53 us | 45.78 us | 368.56x | 2.01x | 0.29x |
| `vec4 add` | 7.75 ns | 2.84 us | 1.88 us | 203.68 ns | 500.17 ns | 367.22x | 13.96x | 5.69x |
| `vec4 add scalar` | 6.71 ns | 2.38 us | 1.96 us | 213.48 ns | 583.25 ns | 354.42x | 11.14x | 4.08x |
| `vec4 sub` | 5.19 ns | 3.09 us | 1.83 us | 209.48 ns | 583.83 ns | 596.02x | 14.76x | 5.29x |
| `vec4 sub scalar` | 4.48 ns | 2.56 us | 1.78 us | 202.85 ns | 504.37 ns | 571.01x | 12.61x | 5.07x |
| `vec4 neg` | 4.89 ns | 488.03 ns | 386.30 ns | 81.05 ns | 50.88 ns | 99.70x | 6.02x | 9.59x |
| `vec4 mul scalar` | 7.20 ns | 2.25 us | 2.32 us | 216.36 ns | 667.67 ns | 312.40x | 10.40x | 3.37x |
| `vec4 div scalar` | 13.98 ns | 2.07 us | 1.86 us | 543.50 ns | 4.05 us | 148.19x | 3.81x | 0.51x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 15.81 ns | 8.76 us | 3.69 us | 1.09 us | 2.60 us | 553.83x | 8.06x | 3.37x |
| `mat3 inverse` | 83.36 ns | 33.59 us | 13.26 us | 3.47 us | 10.88 us | 402.99x | 9.67x | 3.09x |
| `mat3 mul mat3` | 75.04 ns | 26.09 us | 13.33 us | 3.01 us | 8.24 us | 347.73x | 8.66x | 3.17x |
| `mat3 transform vec3` | 19.74 ns | 11.91 us | 12.26 us | 1.17 us | 2.84 us | 603.13x | 10.22x | 4.19x |
| `mat4 determinant` | 44.79 ns | 18.34 us | 6.10 us | 5.30 us | 11.10 us | 409.33x | 3.46x | 1.65x |
| `mat4 inverse` | 166.55 ns | 79.87 us | 21.21 us | 11.78 us | 40.61 us | 479.55x | 6.78x | 1.97x |
| `mat4 mul mat4` | 123.44 ns | 38.02 us | 16.66 us | 6.69 us | 20.08 us | 308.01x | 5.68x | 1.89x |
| `mat4 transform vec4` | 24.22 ns | 15.23 us | 5.13 us | 2.10 us | 4.95 us | 628.72x | 7.26x | 3.08x |

#### Matrix API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 34.34 ns | 3.80 us | 5.97 us | 257.87 ns | 712.36 ns | 110.79x | 14.75x | 5.34x |
| `mat3 zero` | 15.99 ns | 818.92 ns | 805.21 ns | 304.13 ns | 223.03 ns | 51.21x | 2.69x | 3.67x |
| `mat3 identity` | 9.64 ns | 858.74 ns | 848.36 ns | 364.30 ns | 336.82 ns | 89.08x | 2.36x | 2.55x |
| `mat3 transpose` | 9.03 ns | 967.46 ns | 923.08 ns | 266.48 ns | 218.62 ns | 107.08x | 3.63x | 4.43x |
| `mat3 reciprocal` | 78.61 ns | 33.33 us | 38.60 us | 3.14 us | 10.70 us | 424.03x | 10.62x | 3.12x |
| `mat3 reciprocal checked` | 79.02 ns | 33.31 us | 38.69 us | 3.13 us | 10.54 us | 421.56x | 10.63x | 3.16x |
| `mat3 inverse checked` | 79.86 ns | 33.40 us | 38.78 us | 3.13 us | 10.59 us | 418.21x | 10.66x | 3.16x |
| `mat3 inverse checked abort` | 114.90 ns | 33.57 us | 39.05 us | 3.14 us | 10.64 us | 292.18x | 10.69x | 3.15x |
| `mat3 powi` | 152.73 ns | 74.04 us | 99.24 us | 7.14 us | 19.79 us | 484.81x | 10.38x | 3.74x |
| `mat3 powi checked` | 152.52 ns | 73.51 us | 99.14 us | 7.13 us | 19.93 us | 482.01x | 10.31x | 3.69x |
| `mat3 powi checked abort` | 152.66 ns | 74.00 us | 98.62 us | 7.15 us | 19.93 us | 484.78x | 10.35x | 3.71x |
| `mat3 div scalar checked` | 23.95 ns | 5.74 us | 5.63 us | 1.62 us | 11.43 us | 239.58x | 3.53x | 0.50x |
| `mat3 div scalar checked abort` | 39.13 ns | 5.81 us | 5.87 us | 1.60 us | 11.92 us | 148.59x | 3.64x | 0.49x |
| `mat3 div matrix checked` | 119.23 ns | 79.16 us | 76.36 us | 5.70 us | 19.01 us | 663.89x | 13.88x | 4.16x |
| `mat3 div matrix checked abort` | 134.01 ns | 79.64 us | 76.80 us | 5.69 us | 18.92 us | 594.29x | 14.00x | 4.21x |
| `mat3 add` | 14.37 ns | 6.18 us | 5.80 us | 547.34 ns | 1.32 us | 430.05x | 11.29x | 4.68x |
| `mat3 add scalar` | 10.52 ns | 6.04 us | 6.29 us | 840.90 ns | 1.61 us | 574.50x | 7.18x | 3.75x |
| `mat3 sub` | 12.88 ns | 7.54 us | 5.80 us | 559.94 ns | 1.51 us | 585.96x | 13.47x | 5.00x |
| `mat3 sub scalar` | 10.23 ns | 7.54 us | 6.22 us | 852.03 ns | 1.61 us | 737.21x | 8.85x | 4.69x |
| `mat3 neg` | 10.78 ns | 1.98 us | 1.20 us | 519.72 ns | 519.73 ns | 183.47x | 3.80x | 3.80x |
| `mat3 mul scalar` | 12.11 ns | 5.63 us | 6.30 us | 864.22 ns | 1.79 us | 464.90x | 6.51x | 3.15x |
| `mat3 div scalar` | 23.90 ns | 5.72 us | 5.67 us | 1.61 us | 11.41 us | 239.21x | 3.56x | 0.50x |
| `mat3 div matrix` | 116.99 ns | 88.58 us | 75.69 us | 5.68 us | 19.07 us | 757.15x | 15.59x | 4.64x |
| `mat3 bitxor` | 153.11 ns | 73.91 us | 98.64 us | 7.11 us | 19.85 us | 482.69x | 10.39x | 3.72x |
| `mat4 zero` | 11.42 ns | 1.61 us | 1.70 us | 605.64 ns | 390.25 ns | 141.24x | 2.66x | 4.13x |
| `mat4 identity` | 10.78 ns | 1.70 us | 1.79 us | 666.38 ns | 502.92 ns | 157.61x | 2.55x | 3.38x |
| `mat4 transpose` | 9.74 ns | 1.76 us | 1.76 us | 527.36 ns | 370.36 ns | 180.42x | 3.33x | 4.75x |
| `mat4 reciprocal` | 141.85 ns | 76.68 us | 76.07 us | 11.43 us | 40.23 us | 540.55x | 6.71x | 1.91x |
| `mat4 reciprocal checked` | 146.05 ns | 76.70 us | 75.97 us | 11.50 us | 40.12 us | 525.14x | 6.67x | 1.91x |
| `mat4 powi` | 248.71 ns | 115.57 us | 133.27 us | 16.69 us | 44.93 us | 464.66x | 6.92x | 2.57x |
| `mat4 powi checked` | 249.49 ns | 116.19 us | 132.76 us | 16.67 us | 44.77 us | 465.70x | 6.97x | 2.60x |
| `mat4 add` | 51.02 ns | 8.37 us | 6.14 us | 991.05 ns | 2.13 us | 164.00x | 8.44x | 3.94x |
| `mat4 add scalar` | 16.15 ns | 8.67 us | 8.58 us | 1.56 us | 2.70 us | 536.61x | 5.57x | 3.21x |
| `mat4 sub` | 38.33 ns | 10.32 us | 6.40 us | 1.06 us | 2.56 us | 269.24x | 9.78x | 4.03x |
| `mat4 sub scalar` | 14.79 ns | 11.65 us | 8.48 us | 1.52 us | 2.75 us | 787.95x | 7.65x | 4.23x |
| `mat4 neg` | 13.50 ns | 3.56 us | 2.21 us | 980.27 ns | 874.03 ns | 263.52x | 3.63x | 4.07x |
| `mat4 mul scalar` | 50.31 ns | 7.79 us | 8.26 us | 1.61 us | 3.15 us | 154.89x | 4.85x | 2.47x |
| `mat4 div scalar` | 33.63 ns | 8.90 us | 8.02 us | 2.89 us | 21.06 us | 264.61x | 3.08x | 0.42x |
| `mat4 div matrix` | 191.73 ns | 176.93 us | 126.44 us | 17.90 us | 60.81 us | 922.80x | 9.89x | 2.91x |
| `mat4 bitxor` | 249.61 ns | 115.76 us | 133.37 us | 16.64 us | 45.13 us | 463.77x | 6.96x | 2.57x |

### Borrowed API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 8.92 ns | 449.60 ns | 448.67 ns | - | - | 50.43x | - | - |
| `scalar add ref_owned` | 5.46 ns | 450.15 ns | 455.02 ns | - | - | 82.52x | - | - |
| `scalar add refs` | 5.29 ns | 445.06 ns | 450.99 ns | - | - | 84.09x | - | - |
| `scalar sub owned_ref` | 8.95 ns | 822.82 ns | 622.54 ns | - | - | 91.92x | - | - |
| `scalar sub ref_owned` | 5.64 ns | 887.09 ns | 669.95 ns | - | - | 157.30x | - | - |
| `scalar sub refs` | 5.53 ns | 817.92 ns | 621.69 ns | - | - | 148.03x | - | - |
| `scalar mul owned_ref` | 9.62 ns | 439.69 ns | 368.01 ns | - | - | 45.72x | - | - |
| `scalar mul ref_owned` | 7.73 ns | 438.51 ns | 369.42 ns | - | - | 56.74x | - | - |
| `scalar mul refs` | 6.26 ns | 446.43 ns | 373.30 ns | - | - | 71.26x | - | - |
| `scalar div owned_ref` | 13.13 ns | 332.00 ns | 203.41 ns | - | - | 25.29x | - | - |
| `scalar div ref_owned` | 8.02 ns | 394.45 ns | 300.51 ns | - | - | 49.17x | - | - |
| `scalar div refs` | 6.80 ns | 326.27 ns | 194.60 ns | - | - | 47.97x | - | - |
| `vec3 add refs` | 5.97 ns | 2.38 us | 2.26 us | - | - | 399.11x | - | - |
| `vec3 sub refs` | 5.98 ns | 2.77 us | 2.76 us | - | - | 462.29x | - | - |
| `vec3 neg ref` | 3.26 ns | 359.35 ns | 412.69 ns | - | - | 110.35x | - | - |
| `vec3 add_scalar_ref` | 6.45 ns | 2.00 us | 1.89 us | - | - | 309.57x | - | - |
| `vec3 sub_scalar_ref` | 6.42 ns | 2.06 us | 2.00 us | - | - | 320.52x | - | - |
| `vec3 mul_scalar_ref` | 6.83 ns | 1.97 us | 2.06 us | - | - | 288.43x | - | - |
| `vec3 div_scalar_ref` | 9.65 ns | 1.80 us | 1.67 us | - | - | 186.34x | - | - |
| `vec4 add refs` | 6.54 ns | 2.79 us | 2.47 us | - | - | 426.26x | - | - |
| `vec4 sub refs` | 3.05 ns | 3.06 us | 2.70 us | - | - | 1002.20x | - | - |
| `vec4 neg ref` | 4.20 ns | 478.75 ns | 475.85 ns | - | - | 114.02x | - | - |
| `vec4 add_scalar_ref` | 6.87 ns | 2.27 us | 1.96 us | - | - | 331.03x | - | - |
| `vec4 sub_scalar_ref` | 4.18 ns | 2.42 us | 2.18 us | - | - | 579.19x | - | - |
| `vec4 mul_scalar_ref` | 7.33 ns | 2.20 us | 2.12 us | - | - | 300.11x | - | - |
| `vec4 div_scalar_ref` | 13.11 ns | 2.10 us | 1.89 us | - | - | 160.34x | - | - |
| `mat3 add refs` | 12.29 ns | 6.14 us | 6.89 us | - | - | 499.64x | - | - |
| `mat3 sub refs` | 11.39 ns | 7.23 us | 7.77 us | - | - | 634.92x | - | - |
| `mat3 mul refs` | 80.04 ns | 31.01 us | 39.46 us | - | - | 387.37x | - | - |
| `mat3 div refs` | 116.95 ns | 87.41 us | 84.78 us | - | - | 747.42x | - | - |
| `mat3 neg ref` | 9.78 ns | 1.52 us | 1.56 us | - | - | 155.52x | - | - |
| `mat3 add_scalar_ref` | 10.56 ns | 5.60 us | 6.46 us | - | - | 530.84x | - | - |
| `mat3 sub_scalar_ref` | 11.99 ns | 7.20 us | 7.20 us | - | - | 600.13x | - | - |
| `mat3 mul_scalar_ref` | 11.47 ns | 5.35 us | 6.07 us | - | - | 466.38x | - | - |
| `mat3 div_scalar_ref` | 22.85 ns | 5.64 us | 5.63 us | - | - | 246.95x | - | - |
| `mat4 add refs` | 51.75 ns | 8.20 us | 8.44 us | - | - | 158.51x | - | - |
| `mat4 sub refs` | 38.45 ns | 10.02 us | 10.05 us | - | - | 260.56x | - | - |
| `mat4 mul refs` | 120.56 ns | 49.86 us | 53.98 us | - | - | 413.56x | - | - |
| `mat4 div refs` | 193.94 ns | 176.27 us | 150.35 us | - | - | 908.87x | - | - |
| `mat4 neg ref` | 11.91 ns | 2.54 us | 2.57 us | - | - | 213.32x | - | - |
| `mat4 add_scalar_ref` | 13.91 ns | 8.39 us | 9.01 us | - | - | 602.74x | - | - |
| `mat4 sub_scalar_ref` | 16.17 ns | 11.32 us | 10.50 us | - | - | 700.06x | - | - |
| `mat4 mul_scalar_ref` | 49.52 ns | 7.56 us | 7.94 us | - | - | 152.67x | - | - |
| `mat4 div_scalar_ref` | 28.84 ns | 8.60 us | 7.86 us | - | - | 298.32x | - | - |
| `mat3 transform_vec refs` | 19.97 ns | 13.50 us | 15.37 us | - | - | 675.96x | - | - |
| `mat4 transform_vec refs` | 24.39 ns | 18.62 us | 17.61 us | - | - | 763.42x | - | - |
| `complex add refs` | 7.66 ns | 1.14 us | 1.14 us | - | - | 149.46x | - | - |
| `complex sub refs` | 7.91 ns | 1.46 us | 1.32 us | - | - | 184.16x | - | - |
| `complex mul refs` | 7.95 ns | 3.83 us | 4.05 us | - | - | 481.68x | - | - |
| `complex div refs` | 18.53 ns | 7.23 us | 7.69 us | - | - | 390.35x | - | - |
| `complex neg ref` | 2.34 ns | 224.44 ns | 218.22 ns | - | - | 95.96x | - | - |
| `complex div_real_ref` | 9.95 ns | 875.42 ns | 829.23 ns | - | - | 88.02x | - | - |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.30 us |
| `astro sin 160` | 13.28 us |
| `astro sin 192` | 13.28 us |
| `astro sin 256` | 16.25 us |
| `arp sin 128` | 264.17 us |
| `arp sin 160` | 332.38 us |
| `arp sin 192` | 392.90 us |
| `arp sin 256` | 603.71 us |
