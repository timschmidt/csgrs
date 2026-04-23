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
| `sin 0.1` | 10.87 ns | 2.40 us | 2.37 us | 10.55 us | 126.29 us | 220.65x | 0.23x | 0.02x |
| `cos 0.1` | 11.54 ns | 235.92 ns | 238.04 ns | 10.11 us | 25.70 us | 20.45x | 0.02x | 0.01x |
| `sin 1.23456789` | 11.59 ns | 1.84 us | 1.82 us | 12.15 us | 264.91 us | 158.82x | 0.15x | 0.01x |
| `cos 1.23456789` | 11.83 ns | 430.31 ns | 436.66 ns | 10.14 us | 171.40 us | 36.37x | 0.04x | 0.00x |
| `sin 1e6` | 12.54 ns | 34.96 us | 34.87 us | 15.99 us | 269.47 us | 2787.03x | 2.19x | 0.13x |
| `cos 1e6` | 12.24 ns | 21.78 us | 21.64 us | 13.37 us | 172.88 us | 1779.98x | 1.63x | 0.13x |
| `sin 1e30` | 65.27 ns | 185.02 us | 186.12 us | 17.98 us | 277.12 us | 2834.72x | 10.29x | 0.67x |
| `cos 1e30` | 70.45 ns | 138.38 us | 138.78 us | 15.69 us | 172.28 us | 1964.32x | 8.82x | 0.80x |
| `sin pi_7` | 11.61 ns | 2.42 us | 4.26 us | 11.65 us | 124.42 us | 208.27x | 0.21x | 0.02x |
| `cos pi_7` | 11.53 ns | 236.24 ns | 4.39 us | 10.31 us | 27.87 us | 20.48x | 0.02x | 0.01x |
| `sin 1000pi_eps` | 11.62 ns | 22.86 us | 33.81 us | 16.05 us | 266.42 us | 1967.16x | 1.42x | 0.09x |
| `cos 1000pi_eps` | 12.36 ns | 13.65 us | 24.54 us | 13.64 us | 155.84 us | 1104.76x | 1.00x | 0.09x |

#### Scalar API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.47 ns | 49.67 ns | 50.64 ns | 26.75 ns | 14.46 ns | 105.18x | 1.86x | 3.43x |
| `one` | 0.47 ns | 69.13 ns | 56.36 ns | 39.94 ns | 47.05 ns | 146.86x | 1.73x | 1.47x |
| `e` | 0.47 ns | 814.30 ns | 871.52 ns | 51.93 ns | 52.75 us | 1734.42x | 15.68x | 0.02x |
| `pi` | 0.47 ns | 220.37 ns | 218.10 ns | 45.13 ns | 102.26 us | 467.06x | 4.88x | 0.00x |
| `tau` | 0.47 ns | 348.53 ns | 377.38 ns | 120.69 ns | 103.85 us | 741.70x | 2.89x | 0.00x |
| `add` | 5.12 ns | 406.03 ns | 417.00 ns | 50.78 ns | 101.05 ns | 79.30x | 8.00x | 4.02x |
| `sub` | 5.18 ns | 693.16 ns | 492.67 ns | 56.00 ns | 146.10 ns | 133.77x | 12.38x | 4.74x |
| `neg` | 5.01 ns | 70.22 ns | 68.14 ns | 22.82 ns | 16.55 ns | 14.00x | 3.08x | 4.24x |
| `mul` | 5.36 ns | 438.65 ns | 350.90 ns | 56.38 ns | 170.76 ns | 81.78x | 7.78x | 2.57x |
| `div` | 8.81 ns | 409.53 ns | 259.10 ns | 138.45 ns | 881.46 ns | 46.47x | 2.96x | 0.46x |
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
| `sin` | 14.88 ns | 15.49 us | 15.70 us | 13.63 us | 199.00 us | 1040.77x | 1.14x | 0.08x |
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
| `add` | 6.03 ns | 922.36 ns | 892.37 ns | 104.67 ns | 242.17 ns | 152.85x | 8.81x | 3.81x |
| `sub` | 6.05 ns | 1.04 us | 846.64 ns | 117.64 ns | 302.11 ns | 171.71x | 8.83x | 3.44x |
| `neg` | 2.56 ns | 157.88 ns | 139.44 ns | 41.16 ns | 23.69 ns | 61.71x | 3.84x | 6.67x |
| `mul` | 11.24 ns | 3.39 us | 3.50 us | 312.31 ns | 926.77 ns | 301.40x | 10.84x | 3.65x |
| `div` | 17.45 ns | 6.62 us | 6.82 us | 813.21 ns | 4.29 us | 379.32x | 8.14x | 1.54x |
| `div real` | 10.02 ns | 933.80 ns | 833.67 ns | 269.76 ns | 1.81 us | 93.24x | 3.46x | 0.52x |

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
| `vec3 add` | 6.46 ns | 1.98 us | 1.81 us | 150.37 ns | 368.32 ns | 306.58x | 13.17x | 5.38x |
| `vec3 add scalar` | 6.26 ns | 1.98 us | 1.84 us | 152.23 ns | 438.02 ns | 316.19x | 13.00x | 4.52x |
| `vec3 sub` | 6.54 ns | 2.12 us | 1.89 us | 164.09 ns | 486.06 ns | 324.05x | 12.91x | 4.36x |
| `vec3 sub scalar` | 6.32 ns | 1.81 us | 1.75 us | 147.78 ns | 366.24 ns | 285.58x | 12.21x | 4.93x |
| `vec3 neg` | 3.73 ns | 309.65 ns | 290.84 ns | 55.87 ns | 44.66 ns | 83.08x | 5.54x | 6.93x |
| `vec3 mul scalar` | 6.84 ns | 2.09 us | 2.14 us | 159.00 ns | 511.16 ns | 305.71x | 13.15x | 4.09x |
| `vec3 div scalar` | 16.97 ns | 1.85 us | 1.73 us | 411.51 ns | 3.12 us | 108.96x | 4.49x | 0.59x |
| `vec4 dot` | 7.14 ns | 3.89 us | 3.63 us | 447.90 ns | 1.24 us | 544.77x | 8.68x | 3.13x |
| `vec4 magnitude` | 12.84 ns | 7.52 us | 6.34 us | 6.03 us | 39.73 us | 585.42x | 1.25x | 0.19x |
| `vec4 normalize` | 35.63 ns | 13.13 us | 8.91 us | 6.53 us | 45.78 us | 368.56x | 2.01x | 0.29x |
| `vec4 add` | 7.75 ns | 2.16 us | 1.88 us | 203.68 ns | 500.17 ns | 278.22x | 10.58x | 4.31x |
| `vec4 add scalar` | 6.71 ns | 2.22 us | 1.96 us | 213.48 ns | 583.25 ns | 330.83x | 10.40x | 3.81x |
| `vec4 sub` | 5.19 ns | 2.09 us | 1.83 us | 209.48 ns | 583.83 ns | 403.94x | 10.00x | 3.59x |
| `vec4 sub scalar` | 4.48 ns | 2.06 us | 1.78 us | 202.85 ns | 504.37 ns | 459.84x | 10.16x | 4.08x |
| `vec4 neg` | 4.89 ns | 398.43 ns | 386.30 ns | 81.05 ns | 50.88 ns | 81.40x | 4.92x | 7.83x |
| `vec4 mul scalar` | 7.20 ns | 2.45 us | 2.32 us | 216.36 ns | 667.67 ns | 340.25x | 11.33x | 3.67x |
| `vec4 div scalar` | 13.98 ns | 2.18 us | 1.86 us | 543.50 ns | 4.05 us | 156.02x | 4.01x | 0.54x |

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
| `mat3 add` | 14.37 ns | 5.03 us | 5.80 us | 547.34 ns | 1.32 us | 349.87x | 9.19x | 3.80x |
| `mat3 add scalar` | 10.52 ns | 5.64 us | 6.29 us | 840.90 ns | 1.61 us | 536.35x | 6.71x | 3.50x |
| `mat3 sub` | 12.88 ns | 5.21 us | 5.80 us | 559.94 ns | 1.51 us | 404.74x | 9.31x | 3.45x |
| `mat3 sub scalar` | 10.23 ns | 6.32 us | 6.22 us | 852.03 ns | 1.61 us | 617.39x | 7.41x | 3.93x |
| `mat3 neg` | 10.78 ns | 1.19 us | 1.20 us | 519.72 ns | 519.73 ns | 110.76x | 2.30x | 2.30x |
| `mat3 mul scalar` | 12.11 ns | 5.73 us | 6.30 us | 864.22 ns | 1.79 us | 472.83x | 6.62x | 3.21x |
| `mat3 div scalar` | 23.90 ns | 5.70 us | 5.67 us | 1.61 us | 11.41 us | 238.53x | 3.55x | 0.50x |
| `mat3 div matrix` | 116.99 ns | 79.16 us | 75.69 us | 5.68 us | 19.07 us | 676.63x | 13.93x | 4.15x |
| `mat3 bitxor` | 153.11 ns | 73.91 us | 98.64 us | 7.11 us | 19.85 us | 482.69x | 10.39x | 3.72x |
| `mat4 zero` | 11.42 ns | 1.61 us | 1.70 us | 605.64 ns | 390.25 ns | 141.24x | 2.66x | 4.13x |
| `mat4 identity` | 10.78 ns | 1.70 us | 1.79 us | 666.38 ns | 502.92 ns | 157.61x | 2.55x | 3.38x |
| `mat4 transpose` | 9.74 ns | 1.76 us | 1.76 us | 527.36 ns | 370.36 ns | 180.42x | 3.33x | 4.75x |
| `mat4 reciprocal` | 141.85 ns | 76.68 us | 76.07 us | 11.43 us | 40.23 us | 540.55x | 6.71x | 1.91x |
| `mat4 reciprocal checked` | 146.05 ns | 76.70 us | 75.97 us | 11.50 us | 40.12 us | 525.14x | 6.67x | 1.91x |
| `mat4 powi` | 248.71 ns | 115.57 us | 133.27 us | 16.69 us | 44.93 us | 464.66x | 6.92x | 2.57x |
| `mat4 powi checked` | 249.49 ns | 116.19 us | 132.76 us | 16.67 us | 44.77 us | 465.70x | 6.97x | 2.60x |
| `mat4 add` | 51.02 ns | 5.79 us | 6.14 us | 991.05 ns | 2.13 us | 113.41x | 5.84x | 2.72x |
| `mat4 add scalar` | 16.15 ns | 7.81 us | 8.58 us | 1.56 us | 2.70 us | 483.57x | 5.02x | 2.89x |
| `mat4 sub` | 38.33 ns | 6.31 us | 6.40 us | 1.06 us | 2.56 us | 164.60x | 5.98x | 2.46x |
| `mat4 sub scalar` | 14.79 ns | 9.52 us | 8.48 us | 1.52 us | 2.75 us | 643.82x | 6.25x | 3.46x |
| `mat4 neg` | 13.50 ns | 2.16 us | 2.21 us | 980.27 ns | 874.03 ns | 159.68x | 2.20x | 2.47x |
| `mat4 mul scalar` | 50.31 ns | 7.90 us | 8.26 us | 1.61 us | 3.15 us | 157.09x | 4.92x | 2.51x |
| `mat4 div scalar` | 33.63 ns | 8.79 us | 8.02 us | 2.89 us | 21.06 us | 261.42x | 3.04x | 0.42x |
| `mat4 div matrix` | 191.73 ns | 155.04 us | 126.44 us | 17.90 us | 60.81 us | 808.63x | 8.66x | 2.55x |
| `mat4 bitxor` | 249.61 ns | 115.76 us | 133.37 us | 16.64 us | 45.13 us | 463.77x | 6.96x | 2.57x |

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
