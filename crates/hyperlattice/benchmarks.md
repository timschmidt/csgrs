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
| `sin 0.1` | 10.85 ns | 240.41 ns | 238.35 ns | 10.81 us | 124.63 us | 22.17x | 0.02x | 0.00x |
| `cos 0.1` | 11.66 ns | 242.61 ns | 229.83 ns | 10.49 us | 25.35 us | 20.81x | 0.02x | 0.01x |
| `sin 1.23456789` | 11.76 ns | 1.94 us | 1.96 us | 12.49 us | 265.96 us | 165.01x | 0.16x | 0.01x |
| `cos 1.23456789` | 11.96 ns | 430.74 ns | 442.72 ns | 10.37 us | 173.08 us | 36.02x | 0.04x | 0.00x |
| `sin 1e6` | 12.54 ns | 21.75 us | 21.68 us | 16.63 us | 269.49 us | 1734.30x | 1.31x | 0.08x |
| `cos 1e6` | 12.46 ns | 21.37 us | 21.71 us | 13.90 us | 172.91 us | 1714.39x | 1.54x | 0.12x |
| `sin 1e30` | 65.74 ns | 136.19 us | 137.86 us | 19.37 us | 276.79 us | 2071.62x | 7.03x | 0.49x |
| `cos 1e30` | 68.47 ns | 136.54 us | 139.64 us | 15.60 us | 175.21 us | 1994.11x | 8.75x | 0.78x |
| `sin pi_7` | 11.83 ns | 245.60 ns | 2.35 us | 11.90 us | 123.65 us | 20.77x | 0.02x | 0.00x |
| `cos pi_7` | 11.70 ns | 233.22 ns | 5.87 us | 10.56 us | 27.70 us | 19.94x | 0.02x | 0.01x |
| `sin 1000pi_eps` | 11.78 ns | 13.76 us | 23.87 us | 16.65 us | 265.90 us | 1167.65x | 0.83x | 0.05x |
| `cos 1000pi_eps` | 12.44 ns | 13.69 us | 23.78 us | 13.77 us | 156.74 us | 1099.84x | 0.99x | 0.09x |

#### Scalar API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.47 ns | 52.90 ns | 51.00 ns | 26.91 ns | 14.51 ns | 111.84x | 1.97x | 3.65x |
| `one` | 0.47 ns | 56.13 ns | 77.17 ns | 40.38 ns | 44.25 ns | 118.25x | 1.39x | 1.27x |
| `e` | 0.47 ns | 194.80 ns | 266.71 ns | 49.70 ns | 53.47 us | 414.68x | 3.92x | 0.00x |
| `pi` | 0.47 ns | 251.21 ns | 249.66 ns | 46.17 ns | 103.63 us | 532.99x | 5.44x | 0.00x |
| `tau` | 0.47 ns | 720.80 ns | 745.05 ns | 122.25 ns | 106.02 us | 1527.17x | 5.90x | 0.01x |
| `add` | 5.12 ns | 449.87 ns | 497.80 ns | 50.84 ns | 103.10 ns | 87.87x | 8.85x | 4.36x |
| `sub` | 5.18 ns | 707.38 ns | 573.77 ns | 56.83 ns | 149.16 ns | 136.63x | 12.45x | 4.74x |
| `neg` | 5.06 ns | 55.79 ns | 71.23 ns | 23.03 ns | 16.50 ns | 11.03x | 2.42x | 3.38x |
| `mul` | 5.39 ns | 514.93 ns | 448.06 ns | 57.73 ns | 162.14 ns | 95.62x | 8.92x | 3.18x |
| `div` | 7.86 ns | 432.08 ns | 347.38 ns | 136.83 ns | 897.40 ns | 54.99x | 3.16x | 0.48x |
| `reciprocal` | 8.92 ns | 110.07 ns | 120.56 ns | 158.12 ns | 1.32 us | 12.34x | 0.70x | 0.08x |
| `reciprocal checked` | 9.05 ns | 108.94 ns | 97.91 ns | 159.24 ns | 1.33 us | 12.04x | 0.68x | 0.08x |
| `reciprocal checked abort` | 26.10 ns | 171.30 ns | 169.22 ns | 160.76 ns | 1.32 us | 6.56x | 1.07x | 0.13x |
| `pow` | 34.62 ns | 13.36 us | 9.19 us | 58.26 us | 382.80 us | 385.94x | 0.23x | 0.03x |
| `powi` | 5.84 ns | 2.51 us | 2.72 us | 304.42 ns | 1.23 us | 430.64x | 8.26x | 2.05x |
| `exp` | 10.47 ns | 343.90 ns | 350.32 ns | 14.61 us | 71.85 us | 32.85x | 0.02x | 0.00x |
| `ln` | 11.45 ns | 2.61 us | 2.11 us | 32.22 us | 268.38 us | 227.69x | 0.08x | 0.01x |
| `log10` | 22.73 ns | 7.96 us | 5.73 us | 38.39 us | 483.23 us | 350.06x | 0.21x | 0.02x |
| `log10 abort` | 18.62 ns | 7.91 us | 5.80 us | 38.29 us | 482.41 us | 424.64x | 0.21x | 0.02x |
| `sqrt` | 8.79 ns | 2.08 us | 1.81 us | 5.42 us | 30.20 us | 236.88x | 0.38x | 0.07x |
| `sin` | 14.91 ns | 8.98 us | 9.03 us | 14.30 us | 198.23 us | 601.91x | 0.63x | 0.05x |
| `cos` | 18.11 ns | 8.77 us | 8.83 us | 12.07 us | 96.70 us | 484.38x | 0.73x | 0.09x |
| `tan` | 24.83 ns | 8.87 us | 8.83 us | 31.14 us | 228.01 us | 357.07x | 0.28x | 0.04x |
| `sinh` | 18.09 ns | 3.44 us | 3.62 us | 3.39 us | 177.98 ns | 190.20x | 1.01x | 19.33x |
| `cosh` | 18.09 ns | 3.36 us | 3.58 us | 8.13 us | 162.58 ns | 185.57x | 0.41x | 20.65x |
| `tanh` | 23.50 ns | 8.92 us | 7.89 us | 3.63 us | 179.56 ns | 379.68x | 2.46x | 49.70x |
| `asin` | 9.04 ns | 1.47 us | 1.37 us | 22.20 us | 165.78 ns | 162.22x | 0.07x | 8.85x |
| `asin abort` | 16.62 ns | 1.47 us | 1.42 us | 22.27 us | 162.10 ns | 88.68x | 0.07x | 9.09x |
| `acos` | 9.71 ns | 1.50 us | 1.49 us | 27.76 us | 166.70 ns | 154.12x | 0.05x | 8.98x |
| `acos abort` | 19.35 ns | 1.51 us | 1.47 us | 27.68 us | 161.30 ns | 77.84x | 0.05x | 9.34x |
| `atan` | 16.41 ns | 1.33 us | 1.58 us | 19.65 us | 168.96 ns | 80.82x | 0.07x | 7.85x |
| `atan abort` | 17.04 ns | 1.29 us | 1.63 us | 19.55 us | 160.37 ns | 75.95x | 0.07x | 8.07x |
| `asinh` | 33.16 ns | 1.36 us | 1.62 us | 42.71 us | 190.30 ns | 41.06x | 0.03x | 7.15x |
| `asinh abort` | 25.50 ns | 1.35 us | 1.67 us | 42.71 us | 189.24 ns | 53.01x | 0.03x | 7.14x |
| `acosh` | 8.49 ns | 1.14 us | 1.44 us | 43.79 us | 163.12 ns | 133.86x | 0.03x | 6.97x |
| `acosh abort` | 12.44 ns | 1.20 us | 1.45 us | 43.85 us | 162.26 ns | 96.38x | 0.03x | 7.39x |
| `atanh` | 9.49 ns | 1.50 us | 1.44 us | 37.70 us | 169.14 ns | 158.48x | 0.04x | 8.89x |
| `atanh abort` | 16.98 ns | 1.51 us | 1.51 us | 37.90 us | 170.37 ns | 88.89x | 0.04x | 8.86x |
| `zero status` | 1.21 ns | 2.64 ns | 2.63 ns | 1.01 ns | 0.94 ns | 2.18x | 2.61x | 2.79x |
| `zero status abort` | 3.36 ns | 77.98 ns | 59.21 ns | 1.01 ns | 0.94 ns | 23.21x | 77.47x | 82.89x |

### Complex Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.93 ns | 140.26 ns | 139.78 ns | 47.50 ns | 20.40 ns | 150.11x | 2.95x | 6.88x |
| `one` | 5.32 ns | 137.65 ns | 149.72 ns | 62.24 ns | 55.01 ns | 25.88x | 2.21x | 2.50x |
| `i` | 0.94 ns | 136.64 ns | 148.22 ns | 64.30 ns | 53.53 ns | 146.01x | 2.12x | 2.55x |
| `free i` | 0.94 ns | 147.20 ns | 148.44 ns | 64.97 ns | 54.94 ns | 157.36x | 2.27x | 2.68x |
| `conjugate` | 2.11 ns | 157.16 ns | 155.21 ns | 41.00 ns | 23.87 ns | 74.39x | 3.83x | 6.58x |
| `norm squared` | 5.80 ns | 1.60 us | 1.66 us | 160.27 ns | 487.54 ns | 276.41x | 10.00x | 3.29x |
| `reciprocal` | 18.65 ns | 3.20 us | 3.08 us | 464.59 ns | 3.27 us | 171.79x | 6.90x | 0.98x |
| `reciprocal checked` | 14.40 ns | 3.25 us | 3.11 us | 453.54 ns | 3.27 us | 225.68x | 7.16x | 0.99x |
| `powi` | 18.10 ns | 12.15 us | 15.01 us | 1.52 us | 4.47 us | 671.18x | 8.00x | 2.72x |
| `powi checked` | 18.07 ns | 12.11 us | 15.15 us | 1.51 us | 4.54 us | 670.20x | 8.05x | 2.67x |
| `div checked` | 18.92 ns | 6.61 us | 6.79 us | 830.47 ns | 4.34 us | 349.15x | 7.96x | 1.52x |
| `div real checked` | 9.37 ns | 889.36 ns | 776.74 ns | 269.59 ns | 1.85 us | 94.92x | 3.30x | 0.48x |
| `from scalar` | 1.41 ns | 147.42 ns | 116.12 ns | 44.95 ns | 21.99 ns | 104.59x | 3.28x | 6.70x |
| `add` | 6.13 ns | 1.04 us | 1.02 us | 108.20 ns | 236.33 ns | 169.69x | 9.61x | 4.40x |
| `sub` | 6.08 ns | 1.16 us | 946.19 ns | 117.36 ns | 291.34 ns | 190.08x | 9.84x | 3.97x |
| `neg` | 2.60 ns | 137.52 ns | 120.02 ns | 41.26 ns | 23.54 ns | 52.80x | 3.33x | 5.84x |
| `mul` | 7.77 ns | 3.40 us | 3.51 us | 319.66 ns | 920.21 ns | 438.47x | 10.65x | 3.70x |
| `div` | 17.90 ns | 6.28 us | 6.75 us | 833.52 ns | 4.34 us | 351.14x | 7.54x | 1.45x |
| `div real` | 10.04 ns | 886.26 ns | 769.54 ns | 269.73 ns | 1.86 us | 88.30x | 3.29x | 0.48x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.77 ns | 3.39 us | 6.06 us | 313.10 ns | 840.15 ns | 500.33x | 10.82x | 4.03x |
| `vec3 magnitude` | 9.65 ns | 7.87 us | 11.38 us | 5.91 us | 39.40 us | 816.32x | 1.33x | 0.20x |
| `vec3 normalize` | 25.02 ns | 13.04 us | 13.83 us | 6.46 us | 44.16 us | 521.22x | 2.02x | 0.30x |

#### Vector API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.06 ns | 1.31 us | 2.26 us | 71.32 ns | 229.97 ns | 426.52x | 18.31x | 5.68x |
| `vec3 zero` | 1.39 ns | 227.79 ns | 227.05 ns | 63.73 ns | 32.51 ns | 163.37x | 3.57x | 7.01x |
| `vec3 dot abort` | 27.57 ns | 3.95 us | 4.62 us | 256.31 ns | 805.90 ns | 143.15x | 15.40x | 4.90x |
| `vec3 magnitude abort` | 39.34 ns | 8.35 us | 7.80 us | 6.31 us | 39.56 us | 212.26x | 1.32x | 0.21x |
| `vec3 normalize checked` | 25.95 ns | 13.00 us | 10.33 us | 6.57 us | 43.61 us | 501.01x | 1.98x | 0.30x |
| `vec3 normalize checked abort` | 54.94 ns | 13.55 us | 11.11 us | 6.53 us | 43.65 us | 246.71x | 2.08x | 0.31x |
| `vec3 div scalar checked` | 10.45 ns | 1.78 us | 1.70 us | 404.74 ns | 3.13 us | 170.38x | 4.40x | 0.57x |
| `vec3 div scalar checked abort` | 18.06 ns | 1.81 us | 1.71 us | 404.37 ns | 3.12 us | 100.30x | 4.48x | 0.58x |
| `vec3 add` | 6.63 ns | 2.10 us | 2.00 us | 148.16 ns | 395.93 ns | 317.32x | 14.21x | 5.32x |
| `vec3 add scalar` | 6.43 ns | 1.89 us | 1.78 us | 150.49 ns | 445.78 ns | 294.69x | 12.59x | 4.25x |
| `vec3 sub` | 6.66 ns | 2.21 us | 2.05 us | 164.25 ns | 488.80 ns | 331.57x | 13.45x | 4.52x |
| `vec3 sub scalar` | 6.39 ns | 1.62 us | 1.60 us | 147.33 ns | 379.43 ns | 254.01x | 11.02x | 4.28x |
| `vec3 neg` | 3.76 ns | 248.69 ns | 272.56 ns | 56.55 ns | 42.74 ns | 66.21x | 4.40x | 5.82x |
| `vec3 mul scalar` | 6.89 ns | 1.99 us | 2.20 us | 160.44 ns | 491.26 ns | 288.69x | 12.40x | 4.05x |
| `vec3 div scalar` | 18.40 ns | 1.77 us | 1.75 us | 409.27 ns | 3.15 us | 96.40x | 4.33x | 0.56x |
| `vec4 dot` | 7.20 ns | 3.55 us | 3.35 us | 445.95 ns | 1.33 us | 493.04x | 7.96x | 2.66x |
| `vec4 magnitude` | 13.10 ns | 7.82 us | 7.11 us | 6.03 us | 39.96 us | 597.02x | 1.30x | 0.20x |
| `vec4 normalize` | 35.92 ns | 13.44 us | 9.52 us | 6.62 us | 45.83 us | 374.23x | 2.03x | 0.29x |
| `vec4 add` | 7.81 ns | 2.33 us | 2.01 us | 216.23 ns | 516.20 ns | 298.96x | 10.80x | 4.52x |
| `vec4 add scalar` | 6.86 ns | 2.10 us | 1.87 us | 214.13 ns | 585.91 ns | 306.58x | 9.82x | 3.59x |
| `vec4 sub` | 5.19 ns | 2.22 us | 1.96 us | 213.08 ns | 583.69 ns | 427.47x | 10.41x | 3.80x |
| `vec4 sub scalar` | 4.44 ns | 1.86 us | 1.66 us | 202.88 ns | 532.04 ns | 419.29x | 9.19x | 3.50x |
| `vec4 neg` | 4.92 ns | 330.01 ns | 354.52 ns | 79.16 ns | 51.41 ns | 67.07x | 4.17x | 6.42x |
| `vec4 mul scalar` | 7.35 ns | 2.24 us | 2.26 us | 216.69 ns | 644.38 ns | 304.97x | 10.35x | 3.48x |
| `vec4 div scalar` | 14.07 ns | 2.00 us | 1.73 us | 539.70 ns | 4.05 us | 142.27x | 3.71x | 0.49x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 11.23 ns | 8.69 us | 3.82 us | 1.08 us | 2.59 us | 773.48x | 8.03x | 3.35x |
| `mat3 inverse` | 88.89 ns | 32.53 us | 12.81 us | 3.51 us | 10.82 us | 365.91x | 9.26x | 3.01x |
| `mat3 mul mat3` | 70.82 ns | 26.36 us | 12.86 us | 2.96 us | 9.28 us | 372.25x | 8.90x | 2.84x |
| `mat3 transform vec3` | 15.92 ns | 11.10 us | 12.26 us | 1.15 us | 2.78 us | 697.32x | 9.69x | 4.00x |
| `mat4 determinant` | 45.80 ns | 17.31 us | 6.05 us | 5.38 us | 10.90 us | 377.97x | 3.22x | 1.59x |
| `mat4 inverse` | 158.72 ns | 77.97 us | 22.29 us | 12.24 us | 40.81 us | 491.20x | 6.37x | 1.91x |
| `mat4 mul mat4` | 118.50 ns | 37.41 us | 13.97 us | 6.91 us | 19.86 us | 315.71x | 5.41x | 1.88x |
| `mat4 transform vec4` | 25.26 ns | 14.48 us | 4.62 us | 2.11 us | 4.98 us | 573.16x | 6.85x | 2.91x |

#### Matrix API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 34.73 ns | 3.62 us | 6.09 us | 256.43 ns | 709.55 ns | 104.37x | 14.13x | 5.11x |
| `mat3 zero` | 16.23 ns | 821.86 ns | 828.38 ns | 301.22 ns | 221.82 ns | 50.63x | 2.73x | 3.71x |
| `mat3 identity` | 9.67 ns | 868.81 ns | 878.58 ns | 362.85 ns | 323.45 ns | 89.82x | 2.39x | 2.69x |
| `mat3 transpose` | 9.17 ns | 927.40 ns | 923.02 ns | 264.46 ns | 218.91 ns | 101.18x | 3.51x | 4.24x |
| `mat3 reciprocal` | 88.90 ns | 31.90 us | 38.03 us | 3.19 us | 10.57 us | 358.85x | 10.00x | 3.02x |
| `mat3 reciprocal checked` | 109.79 ns | 32.15 us | 37.79 us | 3.19 us | 10.51 us | 292.85x | 10.09x | 3.06x |
| `mat3 inverse checked` | 109.40 ns | 32.02 us | 37.90 us | 3.20 us | 10.55 us | 292.67x | 10.01x | 3.03x |
| `mat3 inverse checked abort` | 115.81 ns | 32.01 us | 37.83 us | 3.18 us | 10.62 us | 276.38x | 10.05x | 3.01x |
| `mat3 powi` | 143.84 ns | 70.04 us | 94.39 us | 7.25 us | 19.59 us | 486.89x | 9.66x | 3.57x |
| `mat3 powi checked` | 198.06 ns | 70.28 us | 94.39 us | 7.22 us | 19.71 us | 354.83x | 9.73x | 3.57x |
| `mat3 powi checked abort` | 147.11 ns | 70.21 us | 94.98 us | 7.21 us | 19.59 us | 477.27x | 9.73x | 3.58x |
| `mat3 div scalar checked` | 24.69 ns | 5.47 us | 5.61 us | 1.58 us | 11.98 us | 221.70x | 3.47x | 0.46x |
| `mat3 div scalar checked abort` | 28.96 ns | 5.43 us | 5.54 us | 1.57 us | 11.92 us | 187.54x | 3.46x | 0.46x |
| `mat3 div matrix checked` | 147.44 ns | 71.33 us | 68.08 us | 5.79 us | 18.95 us | 483.80x | 12.32x | 3.76x |
| `mat3 div matrix checked abort` | 160.70 ns | 71.50 us | 68.24 us | 5.80 us | 18.91 us | 444.95x | 12.33x | 3.78x |
| `mat3 add` | 14.42 ns | 5.27 us | 5.99 us | 550.92 ns | 1.33 us | 365.17x | 9.56x | 3.97x |
| `mat3 add scalar` | 10.33 ns | 5.36 us | 6.09 us | 842.72 ns | 1.70 us | 518.71x | 6.36x | 3.15x |
| `mat3 sub` | 12.90 ns | 5.49 us | 6.05 us | 556.99 ns | 1.55 us | 425.66x | 9.86x | 3.53x |
| `mat3 sub scalar` | 10.62 ns | 5.93 us | 5.90 us | 850.16 ns | 1.68 us | 558.38x | 6.98x | 3.53x |
| `mat3 neg` | 10.85 ns | 1.05 us | 1.06 us | 514.46 ns | 499.07 ns | 96.92x | 2.04x | 2.11x |
| `mat3 mul scalar` | 12.30 ns | 5.47 us | 6.17 us | 872.68 ns | 1.76 us | 445.08x | 6.27x | 3.12x |
| `mat3 div scalar` | 24.13 ns | 5.54 us | 5.49 us | 1.59 us | 11.94 us | 229.38x | 3.49x | 0.46x |
| `mat3 div matrix` | 145.08 ns | 71.12 us | 68.12 us | 5.76 us | 18.97 us | 490.24x | 12.35x | 3.75x |
| `mat3 bitxor` | 143.81 ns | 69.69 us | 94.47 us | 7.21 us | 19.66 us | 484.60x | 9.66x | 3.54x |
| `mat4 zero` | 11.45 ns | 1.56 us | 1.62 us | 894.66 ns | 389.55 ns | 135.91x | 1.74x | 3.99x |
| `mat4 identity` | 10.87 ns | 1.68 us | 1.73 us | 661.30 ns | 485.73 ns | 154.94x | 2.55x | 3.47x |
| `mat4 transpose` | 9.76 ns | 1.75 us | 1.75 us | 524.23 ns | 362.90 ns | 178.82x | 3.33x | 4.81x |
| `mat4 reciprocal` | 158.54 ns | 75.57 us | 74.33 us | 11.68 us | 40.61 us | 476.65x | 6.47x | 1.86x |
| `mat4 reciprocal checked` | 172.54 ns | 75.28 us | 74.65 us | 11.66 us | 40.72 us | 436.31x | 6.45x | 1.85x |
| `mat4 powi` | 239.60 ns | 108.23 us | 125.12 us | 17.01 us | 43.75 us | 451.73x | 6.36x | 2.47x |
| `mat4 powi checked` | 242.04 ns | 108.59 us | 124.21 us | 17.04 us | 43.99 us | 448.66x | 6.37x | 2.47x |
| `mat4 add` | 51.40 ns | 6.58 us | 6.81 us | 986.86 ns | 2.17 us | 127.93x | 6.66x | 3.04x |
| `mat4 add scalar` | 16.38 ns | 7.51 us | 8.08 us | 1.60 us | 2.72 us | 458.66x | 4.70x | 2.76x |
| `mat4 sub` | 38.75 ns | 6.95 us | 6.96 us | 1.06 us | 2.77 us | 179.37x | 6.57x | 2.51x |
| `mat4 sub scalar` | 15.09 ns | 8.73 us | 7.89 us | 1.60 us | 2.73 us | 578.45x | 5.47x | 3.20x |
| `mat4 neg` | 13.61 ns | 1.92 us | 1.94 us | 976.04 ns | 819.09 ns | 140.78x | 1.96x | 2.34x |
| `mat4 mul scalar` | 21.50 ns | 7.51 us | 7.96 us | 1.65 us | 2.97 us | 349.29x | 4.54x | 2.53x |
| `mat4 div scalar` | 32.80 ns | 8.43 us | 7.81 us | 2.89 us | 20.57 us | 257.16x | 2.92x | 0.41x |
| `mat4 div matrix` | 216.35 ns | 139.28 us | 110.25 us | 18.30 us | 60.54 us | 643.74x | 7.61x | 2.30x |
| `mat4 bitxor` | 238.94 ns | 109.00 us | 124.68 us | 16.97 us | 43.92 us | 456.17x | 6.42x | 2.48x |

### Borrowed API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 8.98 ns | 376.25 ns | 366.96 ns | - | - | 41.92x | - | - |
| `scalar add ref_owned` | 5.48 ns | 375.19 ns | 370.86 ns | - | - | 68.47x | - | - |
| `scalar add refs` | 5.33 ns | 364.72 ns | 365.38 ns | - | - | 68.41x | - | - |
| `scalar sub owned_ref` | 8.96 ns | 654.72 ns | 433.22 ns | - | - | 73.04x | - | - |
| `scalar sub ref_owned` | 5.66 ns | 693.95 ns | 513.27 ns | - | - | 122.54x | - | - |
| `scalar sub refs` | 5.55 ns | 620.90 ns | 417.57 ns | - | - | 111.85x | - | - |
| `scalar mul owned_ref` | 9.65 ns | 412.51 ns | 308.90 ns | - | - | 42.73x | - | - |
| `scalar mul ref_owned` | 6.17 ns | 413.02 ns | 309.48 ns | - | - | 66.93x | - | - |
| `scalar mul refs` | 6.07 ns | 445.20 ns | 306.21 ns | - | - | 73.34x | - | - |
| `scalar div owned_ref` | 13.18 ns | 372.48 ns | 203.41 ns | - | - | 28.27x | - | - |
| `scalar div ref_owned` | 15.78 ns | 455.83 ns | 305.20 ns | - | - | 28.88x | - | - |
| `scalar div refs` | 6.91 ns | 336.84 ns | 202.43 ns | - | - | 48.76x | - | - |
| `vec3 add refs` | 6.04 ns | 1.79 us | 1.59 us | - | - | 297.05x | - | - |
| `vec3 sub refs` | 6.05 ns | 1.91 us | 1.65 us | - | - | 315.46x | - | - |
| `vec3 neg ref` | 3.29 ns | 323.86 ns | 280.74 ns | - | - | 98.50x | - | - |
| `vec3 add_scalar_ref` | 6.44 ns | 1.88 us | 1.66 us | - | - | 291.35x | - | - |
| `vec3 sub_scalar_ref` | 6.45 ns | 1.62 us | 1.48 us | - | - | 250.82x | - | - |
| `vec3 mul_scalar_ref` | 6.87 ns | 1.97 us | 1.99 us | - | - | 286.79x | - | - |
| `vec3 div_scalar_ref` | 9.70 ns | 1.88 us | 1.65 us | - | - | 193.82x | - | - |
| `vec4 add refs` | 6.62 ns | 1.95 us | 1.60 us | - | - | 295.00x | - | - |
| `vec4 sub refs` | 3.07 ns | 1.96 us | 1.46 us | - | - | 638.85x | - | - |
| `vec4 neg ref` | 4.23 ns | 424.30 ns | 358.26 ns | - | - | 100.31x | - | - |
| `vec4 add_scalar_ref` | 6.86 ns | 2.09 us | 1.71 us | - | - | 305.09x | - | - |
| `vec4 sub_scalar_ref` | 4.21 ns | 1.81 us | 1.49 us | - | - | 429.28x | - | - |
| `vec4 mul_scalar_ref` | 7.32 ns | 2.19 us | 2.07 us | - | - | 299.47x | - | - |
| `vec4 div_scalar_ref` | 13.19 ns | 2.12 us | 1.74 us | - | - | 160.75x | - | - |
| `mat3 add refs` | 11.68 ns | 3.71 us | 4.62 us | - | - | 317.68x | - | - |
| `mat3 sub refs` | 11.17 ns | 3.93 us | 4.58 us | - | - | 351.53x | - | - |
| `mat3 mul refs` | 53.69 ns | 24.09 us | 32.92 us | - | - | 448.63x | - | - |
| `mat3 div refs` | 145.62 ns | 70.73 us | 67.35 us | - | - | 485.75x | - | - |
| `mat3 neg ref` | 9.88 ns | 929.72 ns | 903.50 ns | - | - | 94.11x | - | - |
| `mat3 add_scalar_ref` | 10.69 ns | 5.13 us | 5.76 us | - | - | 479.99x | - | - |
| `mat3 sub_scalar_ref` | 12.07 ns | 5.65 us | 5.58 us | - | - | 468.32x | - | - |
| `mat3 mul_scalar_ref` | 11.55 ns | 5.20 us | 5.93 us | - | - | 450.67x | - | - |
| `mat3 div_scalar_ref` | 23.13 ns | 5.47 us | 5.71 us | - | - | 236.27x | - | - |
| `mat4 add refs` | 17.58 ns | 4.15 us | 4.56 us | - | - | 236.12x | - | - |
| `mat4 sub refs` | 16.74 ns | 4.52 us | 4.59 us | - | - | 270.20x | - | - |
| `mat4 mul refs` | 102.72 ns | 35.92 us | 40.21 us | - | - | 349.65x | - | - |
| `mat4 div refs` | 223.20 ns | 138.22 us | 110.56 us | - | - | 619.26x | - | - |
| `mat4 neg ref` | 12.06 ns | 1.59 us | 1.73 us | - | - | 132.20x | - | - |
| `mat4 add_scalar_ref` | 14.09 ns | 7.50 us | 8.01 us | - | - | 532.42x | - | - |
| `mat4 sub_scalar_ref` | 16.35 ns | 8.60 us | 7.80 us | - | - | 525.74x | - | - |
| `mat4 mul_scalar_ref` | 50.01 ns | 7.50 us | 7.95 us | - | - | 149.93x | - | - |
| `mat4 div_scalar_ref` | 29.52 ns | 8.42 us | 7.93 us | - | - | 285.22x | - | - |
| `mat3 transform_vec refs` | 15.60 ns | 10.49 us | 12.10 us | - | - | 672.53x | - | - |
| `mat4 transform_vec refs` | 24.69 ns | 13.51 us | 12.57 us | - | - | 547.44x | - | - |
| `complex add refs` | 7.63 ns | 816.12 ns | 815.45 ns | - | - | 107.03x | - | - |
| `complex sub refs` | 7.97 ns | 937.67 ns | 797.78 ns | - | - | 117.70x | - | - |
| `complex mul refs` | 7.93 ns | 3.13 us | 3.32 us | - | - | 394.36x | - | - |
| `complex div refs` | 16.62 ns | 6.37 us | 6.57 us | - | - | 383.06x | - | - |
| `complex neg ref` | 2.35 ns | 108.92 ns | 112.89 ns | - | - | 46.31x | - | - |
| `complex div_real_ref` | 9.99 ns | 911.17 ns | 787.76 ns | - | - | 91.24x | - | - |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.20 us |
| `astro sin 160` | 13.34 us |
| `astro sin 192` | 13.55 us |
| `astro sin 256` | 16.55 us |
| `arp sin 128` | 264.72 us |
| `arp sin 160` | 330.24 us |
| `arp sin 192` | 392.33 us |
| `arp sin 256` | 590.99 us |
