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

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `sin 0.1` | 11.18 ns | 212.37 ns | 204.82 ns | 11.07 us | 782.43 ns | 1.93 us | 18.99x | 0.02x | 0.27x | 0.11x |
| `cos 0.1` | 11.82 ns | 209.54 ns | 202.29 ns | 10.91 us | 508.67 ns | 1.75 us | 17.72x | 0.02x | 0.41x | 0.12x |
| `sin 1.23456789` | 11.97 ns | 1.98 us | 1.94 us | 12.83 us | 821.80 ns | 1.89 us | 165.05x | 0.15x | 2.40x | 1.05x |
| `cos 1.23456789` | 12.17 ns | 553.99 ns | 532.76 ns | 11.02 us | 613.57 ns | 1.68 us | 45.53x | 0.05x | 0.90x | 0.33x |
| `sin 1e6` | 12.88 ns | 11.29 us | 11.14 us | 15.88 us | 1.11 us | 2.05 us | 876.45x | 0.71x | 10.19x | 5.51x |
| `cos 1e6` | 12.63 ns | 11.17 us | 11.20 us | 14.05 us | 830.12 ns | 1.85 us | 884.55x | 0.80x | 13.45x | 6.03x |
| `sin 1e30` | 66.59 ns | 17.82 us | 17.71 us | 18.64 us | 2.94 us | 3.70 us | 267.58x | 0.96x | 6.07x | 4.81x |
| `cos 1e30` | 69.63 ns | 18.52 us | 17.69 us | 16.03 us | 998.09 ns | 3.19 us | 265.97x | 1.16x | 18.56x | 5.81x |
| `sin pi_7` | 12.09 ns | 208.21 ns | 2.36 us | 12.42 us | 748.73 ns | 1.95 us | 17.22x | 0.02x | 0.28x | 0.11x |
| `cos pi_7` | 11.93 ns | 204.34 ns | 5.41 us | 11.29 us | 542.27 ns | 1.79 us | 17.12x | 0.02x | 0.38x | 0.11x |
| `sin 1000pi_eps` | 12.06 ns | 11.18 us | 17.49 us | 15.97 us | 2.30 us | 2.84 us | 926.55x | 0.70x | 4.86x | 3.93x |
| `cos 1000pi_eps` | 12.58 ns | 11.19 us | 17.34 us | 13.59 us | 568.16 ns | 1.69 us | 889.32x | 0.82x | 19.69x | 6.63x |

#### Scalar API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.48 ns | 61.21 ns | 57.28 ns | 26.34 ns | 15.91 ns | 0.95 ns | 126.30x | 2.32x | 3.85x | 64.32x |
| `one` | 0.48 ns | 67.78 ns | 62.55 ns | 41.62 ns | 30.19 ns | 32.29 ns | 139.85x | 1.63x | 2.24x | 2.10x |
| `e` | 0.48 ns | 763.82 ns | 758.91 ns | 55.97 ns | 1.06 us | 223.69 ns | 1596.39x | 13.65x | 0.72x | 3.41x |
| `pi` | 0.48 ns | 192.48 ns | 190.63 ns | 44.63 ns | 48.96 ns | 223.14 ns | 402.85x | 4.31x | 3.93x | 0.86x |
| `tau` | 0.49 ns | 447.20 ns | 447.67 ns | 127.54 ns | 103.17 ns | 1.90 us | 914.69x | 3.51x | 4.33x | 0.24x |
| `add` | 5.24 ns | 438.42 ns | 415.79 ns | 52.71 ns | 42.48 ns | 1.35 us | 83.64x | 8.32x | 10.32x | 0.32x |
| `sub` | 5.33 ns | 689.82 ns | 473.90 ns | 57.17 ns | 45.51 ns | 2.51 us | 129.36x | 12.07x | 15.16x | 0.27x |
| `neg` | 5.15 ns | 64.38 ns | 63.53 ns | 22.91 ns | 23.29 ns | 1.09 us | 12.49x | 2.81x | 2.76x | 0.06x |
| `mul` | 5.44 ns | 472.25 ns | 358.10 ns | 60.52 ns | 47.39 ns | 1.63 us | 86.74x | 7.80x | 9.97x | 0.29x |
| `div` | 8.96 ns | 411.90 ns | 284.52 ns | 149.72 ns | 67.26 ns | 2.60 us | 45.99x | 2.75x | 6.12x | 0.16x |
| `reciprocal` | 8.82 ns | 119.70 ns | 120.34 ns | 167.83 ns | 61.76 ns | 1.57 us | 13.57x | 0.71x | 1.94x | 0.08x |
| `reciprocal checked` | 9.10 ns | 119.74 ns | 119.15 ns | 167.75 ns | 61.79 ns | 1.55 us | 13.16x | 0.71x | 1.94x | 0.08x |
| `reciprocal checked abort` | 27.66 ns | 210.81 ns | 210.11 ns | 167.85 ns | 62.45 ns | 1.54 us | 7.62x | 1.26x | 3.38x | 0.14x |
| `pow` | 25.85 ns | 37.08 us | 33.12 us | 55.98 us | 2.97 us | 2.37 us | 1434.32x | 0.66x | 12.47x | 15.67x |
| `powi` | 6.18 ns | 2.55 us | 2.74 us | 281.53 ns | 84.58 ns | 1.56 us | 413.09x | 9.07x | 30.18x | 1.64x |
| `exp` | 10.40 ns | 7.57 us | 7.56 us | 14.63 us | 953.60 ns | 1.96 us | 728.14x | 0.52x | 7.94x | 3.87x |
| `ln` | 11.59 ns | 3.00 us | 2.67 us | 30.50 us | 1.38 us | 1.85 us | 258.72x | 0.10x | 2.18x | 1.62x |
| `log10` | 22.45 ns | 9.03 us | 6.83 us | 35.62 us | 2.89 us | 6.93 us | 402.21x | 0.25x | 3.13x | 1.30x |
| `log10 abort` | 18.37 ns | 9.07 us | 6.84 us | 36.07 us | 2.87 us | 6.82 us | 493.75x | 0.25x | 3.16x | 1.33x |
| `sqrt` | 8.54 ns | 1.78 us | 1.35 us | 5.19 us | 100.36 ns | 1.50 us | 208.34x | 0.34x | 17.73x | 1.18x |
| `sin` | 15.11 ns | 5.75 us | 5.89 us | 14.05 us | 1.29 us | 2.29 us | 380.19x | 0.41x | 4.47x | 2.51x |
| `cos` | 18.62 ns | 5.62 us | 5.82 us | 12.22 us | 634.60 ns | 1.80 us | 301.98x | 0.46x | 8.86x | 3.12x |
| `tan` | 25.03 ns | 5.76 us | 5.84 us | 32.60 us | 1.62 us | 6.77 us | 229.99x | 0.18x | 3.55x | 0.85x |
| `sinh` | 18.19 ns | 19.42 us | 19.32 us | 3.31 us | 1.19 us | 11.23 us | 1067.99x | 5.86x | 16.33x | 1.73x |
| `cosh` | 18.36 ns | 19.23 us | 19.41 us | 8.13 us | 1.08 us | 9.88 us | 1047.01x | 2.37x | 17.82x | 1.95x |
| `tanh` | 25.78 ns | 23.49 us | 23.04 us | 3.43 us | 1.21 us | 23.48 us | 911.26x | 6.84x | 19.34x | 1.00x |
| `asin` | 9.16 ns | 1.31 us | 1.23 us | 21.92 us | 2.48 us | 14.50 us | 142.50x | 0.06x | 0.53x | 0.09x |
| `asin abort` | 16.70 ns | 1.30 us | 1.24 us | 21.79 us | 2.45 us | 14.56 us | 77.63x | 0.06x | 0.53x | 0.09x |
| `acos` | 9.74 ns | 1.29 us | 1.26 us | 27.31 us | 2.56 us | 14.44 us | 132.71x | 0.05x | 0.50x | 0.09x |
| `acos abort` | 20.87 ns | 1.30 us | 1.27 us | 27.55 us | 2.60 us | 14.36 us | 62.44x | 0.05x | 0.50x | 0.09x |
| `atan` | 16.56 ns | 1.26 us | 1.56 us | 18.76 us | 2.30 us | 19.54 us | 76.19x | 0.07x | 0.55x | 0.06x |
| `atan abort` | 17.17 ns | 1.27 us | 1.58 us | 18.59 us | 2.28 us | 19.51 us | 74.13x | 0.07x | 0.56x | 0.07x |
| `asinh` | 33.98 ns | 1.26 us | 1.56 us | 39.98 us | 1.68 us | 7.82 us | 37.10x | 0.03x | 0.75x | 0.16x |
| `asinh abort` | 26.86 ns | 1.27 us | 1.58 us | 40.06 us | 1.67 us | 7.80 us | 47.44x | 0.03x | 0.76x | 0.16x |
| `acosh` | 8.59 ns | 1.10 us | 1.45 us | 41.37 us | 3.43 us | 10.93 us | 128.53x | 0.03x | 0.32x | 0.10x |
| `acosh abort` | 12.76 ns | 1.12 us | 1.45 us | 41.73 us | 3.53 us | 10.97 us | 88.03x | 0.03x | 0.32x | 0.10x |
| `atanh` | 9.72 ns | 1.34 us | 1.28 us | 35.75 us | 1.31 us | 15.36 us | 137.70x | 0.04x | 1.02x | 0.09x |
| `atanh abort` | 17.08 ns | 1.34 us | 1.31 us | 35.40 us | 1.32 us | 15.40 us | 78.39x | 0.04x | 1.02x | 0.09x |
| `zero status` | 1.25 ns | 2.36 ns | 2.35 ns | 1.04 ns | 6.86 ns | 7.99 ns | 1.88x | 2.26x | 0.34x | 0.30x |
| `zero status abort` | 3.43 ns | 73.27 ns | 73.32 ns | 1.06 ns | 6.77 ns | 8.21 ns | 21.33x | 69.24x | 10.83x | 8.92x |

### Complex Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.95 ns | 118.94 ns | 119.33 ns | 47.71 ns | - | 1.89 ns | 125.24x | 2.49x | - | 62.89x |
| `one` | 5.37 ns | 120.18 ns | 121.06 ns | 64.15 ns | - | 30.21 ns | 22.40x | 1.87x | - | 3.98x |
| `i` | 0.95 ns | 128.43 ns | 128.57 ns | 66.01 ns | - | 32.06 ns | 135.13x | 1.95x | - | 4.01x |
| `free i` | 0.95 ns | 128.18 ns | 127.63 ns | 65.08 ns | - | 32.61 ns | 135.23x | 1.97x | - | 3.93x |
| `conjugate` | 2.13 ns | 132.94 ns | 135.60 ns | 41.81 ns | - | 1.10 us | 62.33x | 3.18x | - | 0.12x |
| `norm squared` | 5.93 ns | 1.61 us | 1.59 us | 154.08 ns | - | 4.53 us | 271.04x | 10.42x | - | 0.35x |
| `reciprocal` | 18.17 ns | 3.08 us | 3.11 us | 471.68 ns | - | 11.00 us | 169.56x | 6.53x | - | 0.28x |
| `reciprocal checked` | 14.05 ns | 3.14 us | 3.09 us | 468.60 ns | - | 11.03 us | 223.36x | 6.70x | - | 0.28x |
| `powi` | 18.21 ns | 12.36 us | 15.19 us | 1.46 us | - | 44.98 us | 678.70x | 8.44x | - | 0.27x |
| `powi checked` | 18.13 ns | 12.24 us | 15.20 us | 1.46 us | - | 45.34 us | 675.13x | 8.39x | - | 0.27x |
| `div checked` | 20.82 ns | 6.42 us | 6.76 us | 825.26 ns | - | 22.22 us | 308.57x | 7.78x | - | 0.29x |
| `div real checked` | 9.43 ns | 794.46 ns | 786.33 ns | 269.57 ns | - | 5.22 us | 84.29x | 2.95x | - | 0.15x |
| `from scalar` | 1.43 ns | 124.67 ns | 120.56 ns | 45.78 ns | - | 9.86 ns | 87.11x | 2.72x | - | 12.64x |
| `add` | 5.91 ns | 884.43 ns | 903.99 ns | 104.62 ns | - | 2.67 us | 149.65x | 8.45x | - | 0.33x |
| `sub` | 6.03 ns | 1.00 us | 881.58 ns | 115.69 ns | - | 4.92 us | 166.50x | 8.68x | - | 0.20x |
| `neg` | 2.62 ns | 140.07 ns | 137.79 ns | 42.73 ns | - | 2.19 us | 53.51x | 3.28x | - | 0.06x |
| `mul` | 11.36 ns | 3.28 us | 3.47 us | 318.53 ns | - | 10.33 us | 288.37x | 10.28x | - | 0.32x |
| `div` | 17.91 ns | 6.38 us | 6.75 us | 821.30 ns | - | 22.24 us | 356.11x | 7.77x | - | 0.29x |
| `div real` | 10.14 ns | 763.35 ns | 746.55 ns | 269.68 ns | - | 5.27 us | 75.30x | 2.83x | - | 0.14x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.89 ns | 3.60 us | 6.16 us | 313.68 ns | 256.58 ns | 7.54 us | 523.05x | 11.48x | 14.04x | 0.48x |
| `vec3 magnitude` | 10.13 ns | 7.52 us | 10.35 us | 5.64 us | 355.43 ns | 9.17 us | 742.09x | 1.33x | 21.16x | 0.82x |
| `vec3 normalize` | 25.80 ns | 12.49 us | 12.72 us | 6.08 us | 596.47 ns | 17.47 us | 484.02x | 2.05x | 20.94x | 0.71x |

#### Vector API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.09 ns | 1.16 us | 2.21 us | 75.14 ns | 58.16 ns | 720.57 ns | 375.26x | 15.45x | 19.96x | 1.61x |
| `vec3 zero` | 1.41 ns | 240.70 ns | 242.89 ns | 60.60 ns | 31.46 ns | 2.87 ns | 170.12x | 3.97x | 7.65x | 83.76x |
| `vec3 dot abort` | 27.88 ns | 3.97 us | 4.58 us | 261.15 ns | 201.08 ns | 7.73 us | 142.51x | 15.22x | 19.76x | 0.51x |
| `vec3 magnitude abort` | 39.15 ns | 7.86 us | 7.20 us | 5.51 us | 328.11 ns | 9.19 us | 200.82x | 1.43x | 23.96x | 0.86x |
| `vec3 normalize checked` | 26.10 ns | 12.41 us | 9.57 us | 6.08 us | 544.59 ns | 17.38 us | 475.61x | 2.04x | 22.79x | 0.71x |
| `vec3 normalize checked abort` | 55.69 ns | 13.15 us | 10.23 us | 5.98 us | 543.39 ns | 17.31 us | 236.10x | 2.20x | 24.20x | 0.76x |
| `vec3 div scalar checked` | 14.48 ns | 1.74 us | 1.64 us | 406.56 ns | - | - | 119.93x | 4.27x | - | - |
| `vec3 div scalar checked abort` | 18.05 ns | 1.74 us | 1.66 us | 408.14 ns | - | - | 96.44x | 4.26x | - | - |
| `vec3 add` | 7.28 ns | 2.05 us | 1.86 us | 150.95 ns | 126.98 ns | 4.18 us | 281.20x | 13.57x | 16.13x | 0.49x |
| `vec3 add scalar` | 6.41 ns | 1.83 us | 1.73 us | 157.38 ns | 131.97 ns | 3.92 us | 285.86x | 11.64x | 13.89x | 0.47x |
| `vec3 sub` | 7.27 ns | 2.15 us | 1.97 us | 165.97 ns | 135.90 ns | 7.54 us | 296.39x | 12.98x | 15.86x | 0.29x |
| `vec3 sub scalar` | 6.44 ns | 1.62 us | 1.54 us | 148.31 ns | 126.30 ns | 7.16 us | 251.42x | 10.91x | 12.81x | 0.23x |
| `vec3 neg` | 3.79 ns | 256.33 ns | 255.25 ns | 62.81 ns | 46.69 ns | 3.23 us | 67.64x | 4.08x | 5.49x | 0.08x |
| `vec3 mul scalar` | 6.98 ns | 2.01 us | 2.09 us | 167.61 ns | 118.48 ns | 4.52 us | 288.29x | 12.01x | 16.99x | 0.45x |
| `vec3 div scalar` | 18.74 ns | 1.73 us | 1.64 us | 404.20 ns | 170.87 ns | 7.80 us | 92.09x | 4.27x | 10.10x | 0.22x |
| `vec4 dot` | 7.29 ns | 3.58 us | 3.24 us | 434.18 ns | 316.57 ns | 10.36 us | 491.36x | 8.25x | 11.31x | 0.35x |
| `vec4 magnitude` | 13.32 ns | 7.09 us | 5.79 us | 5.69 us | 401.72 ns | 11.83 us | 532.47x | 1.25x | 17.66x | 0.60x |
| `vec4 normalize` | 36.87 ns | 12.34 us | 8.16 us | 6.24 us | 685.08 ns | 23.00 us | 334.62x | 1.98x | 18.01x | 0.54x |
| `vec4 add` | 7.42 ns | 2.24 us | 1.93 us | 205.60 ns | 174.76 ns | 5.55 us | 301.43x | 10.87x | 12.79x | 0.40x |
| `vec4 add scalar` | 7.02 ns | 2.08 us | 1.78 us | 221.45 ns | 178.16 ns | 5.50 us | 295.94x | 9.37x | 11.65x | 0.38x |
| `vec4 sub` | 5.24 ns | 2.13 us | 1.81 us | 212.53 ns | 176.07 ns | 10.19 us | 405.38x | 10.00x | 12.07x | 0.21x |
| `vec4 sub scalar` | 4.49 ns | 1.80 us | 1.56 us | 204.58 ns | 170.23 ns | 9.75 us | 400.53x | 8.79x | 10.56x | 0.18x |
| `vec4 neg` | 4.94 ns | 297.25 ns | 296.75 ns | 80.14 ns | 60.49 ns | 4.30 us | 60.16x | 3.71x | 4.91x | 0.07x |
| `vec4 mul scalar` | 7.35 ns | 2.27 us | 2.21 us | 223.37 ns | 157.86 ns | 5.86 us | 309.25x | 10.18x | 14.40x | 0.39x |
| `vec4 div scalar` | 14.18 ns | 1.94 us | 1.68 us | 537.80 ns | 227.36 ns | 10.17 us | 136.56x | 3.60x | 8.52x | 0.19x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 11.31 ns | 8.76 us | 3.54 us | 975.86 ns | 860.93 ns | 23.74 us | 774.45x | 8.98x | 10.18x | 0.37x |
| `mat3 inverse` | 90.16 ns | 32.07 us | 12.16 us | 3.25 us | 2.51 us | 85.08 us | 355.67x | 9.87x | 12.77x | 0.38x |
| `mat3 mul mat3` | 72.27 ns | 24.17 us | 11.38 us | 2.85 us | 2.37 us | 64.70 us | 334.52x | 8.48x | 10.21x | 0.37x |
| `mat3 transform vec3` | 16.12 ns | 10.81 us | 11.61 us | 1.04 us | 912.90 ns | 21.36 us | 670.28x | 10.40x | 11.84x | 0.51x |
| `mat4 determinant` | 46.70 ns | 16.83 us | 5.31 us | 4.79 us | 4.17 us | 98.14 us | 360.49x | 3.52x | 4.04x | 0.17x |
| `mat4 inverse` | 160.66 ns | 74.70 us | 18.85 us | 12.24 us | 9.36 us | 352.84 us | 464.97x | 6.10x | 7.98x | 0.21x |
| `mat4 mul mat4` | 120.33 ns | 31.48 us | 11.45 us | 6.60 us | 5.56 us | 149.09 us | 261.63x | 4.77x | 5.67x | 0.21x |
| `mat4 transform vec4` | 25.89 ns | 13.74 us | 3.72 us | 1.91 us | 1.69 us | 36.70 us | 530.58x | 7.20x | 8.15x | 0.37x |

#### Matrix API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 36.13 ns | 2.93 us | 6.05 us | 209.25 ns | 224.29 ns | 2.03 us | 80.97x | 13.98x | 13.04x | 1.44x |
| `mat3 zero` | 16.33 ns | 862.40 ns | 836.23 ns | 273.12 ns | 202.72 ns | 11.35 ns | 52.80x | 3.16x | 4.25x | 75.99x |
| `mat3 identity` | 10.11 ns | 924.63 ns | 913.98 ns | 324.23 ns | 237.15 ns | 157.26 ns | 91.45x | 2.85x | 3.90x | 5.88x |
| `mat3 transpose` | 9.60 ns | 904.94 ns | 881.44 ns | 233.86 ns | 205.62 ns | 115.66 ns | 94.23x | 3.87x | 4.40x | 7.82x |
| `mat3 reciprocal` | 90.44 ns | 32.42 us | 37.56 us | 2.97 us | 2.29 us | 84.24 us | 358.44x | 10.91x | 14.15x | 0.38x |
| `mat3 reciprocal checked` | 112.54 ns | 33.50 us | 37.95 us | 2.99 us | 2.28 us | 83.56 us | 297.70x | 11.22x | 14.70x | 0.40x |
| `mat3 inverse checked` | 112.47 ns | 33.69 us | 37.95 us | 2.97 us | 2.28 us | 83.27 us | 299.58x | 11.35x | 14.78x | 0.40x |
| `mat3 inverse checked abort` | 117.65 ns | 32.81 us | 37.65 us | 2.95 us | 2.28 us | 84.48 us | 278.89x | 11.11x | 14.39x | 0.39x |
| `mat3 powi` | 148.57 ns | 70.91 us | 93.43 us | 6.73 us | 6.12 us | 152.05 us | 477.28x | 10.53x | 11.59x | 0.47x |
| `mat3 powi checked` | 148.74 ns | 70.77 us | 93.43 us | 6.72 us | 6.09 us | 151.70 us | 475.77x | 10.53x | 11.61x | 0.47x |
| `mat3 powi checked abort` | 146.48 ns | 70.52 us | 93.37 us | 6.71 us | 6.06 us | 151.95 us | 481.41x | 10.51x | 11.63x | 0.46x |
| `mat3 div scalar checked` | 24.76 ns | 5.47 us | 5.43 us | 1.49 us | 795.76 ns | 22.13 us | 221.04x | 3.67x | 6.88x | 0.25x |
| `mat3 div scalar checked abort` | 30.51 ns | 5.54 us | 5.53 us | 1.47 us | 804.20 ns | 22.07 us | 181.71x | 3.77x | 6.89x | 0.25x |
| `mat3 div matrix checked` | 151.59 ns | 72.33 us | 66.43 us | 5.53 us | 4.42 us | 162.31 us | 477.11x | 13.09x | 16.35x | 0.45x |
| `mat3 div matrix checked abort` | 167.09 ns | 71.83 us | 66.71 us | 5.51 us | 4.42 us | 162.03 us | 429.88x | 13.04x | 16.24x | 0.44x |
| `mat3 add` | 14.75 ns | 5.41 us | 6.19 us | 511.08 ns | 480.81 ns | 12.11 us | 366.81x | 10.58x | 11.25x | 0.45x |
| `mat3 add scalar` | 10.63 ns | 5.22 us | 6.00 us | 789.16 ns | 702.51 ns | 12.40 us | 490.93x | 6.62x | 7.43x | 0.42x |
| `mat3 sub` | 13.22 ns | 5.72 us | 6.11 us | 532.19 ns | 510.07 ns | 21.54 us | 432.93x | 10.75x | 11.22x | 0.27x |
| `mat3 sub scalar` | 10.95 ns | 5.90 us | 5.84 us | 797.82 ns | 699.46 ns | 21.85 us | 538.59x | 7.39x | 8.43x | 0.27x |
| `mat3 neg` | 10.92 ns | 988.23 ns | 971.55 ns | 472.93 ns | 450.85 ns | 8.66 us | 90.50x | 2.09x | 2.19x | 0.11x |
| `mat3 mul scalar` | 12.68 ns | 5.44 us | 6.01 us | 791.77 ns | 663.04 ns | 12.27 us | 429.15x | 6.87x | 8.21x | 0.44x |
| `mat3 div scalar` | 25.16 ns | 5.49 us | 5.42 us | 1.47 us | 797.57 ns | 22.09 us | 218.12x | 3.74x | 6.88x | 0.25x |
| `mat3 div matrix` | 149.64 ns | 71.76 us | 66.58 us | 5.47 us | 4.50 us | 161.95 us | 479.53x | 13.12x | 15.94x | 0.44x |
| `mat3 bitxor` | 144.96 ns | 70.63 us | 94.04 us | 6.73 us | 6.12 us | 151.51 us | 487.23x | 10.49x | 11.53x | 0.47x |
| `mat4 zero` | 11.40 ns | 1.59 us | 1.59 us | 482.21 ns | 344.17 ns | 14.03 ns | 139.44x | 3.30x | 4.62x | 113.24x |
| `mat4 identity` | 11.42 ns | 1.74 us | 1.72 us | 529.05 ns | 406.06 ns | 228.45 ns | 152.70x | 3.30x | 4.30x | 7.64x |
| `mat4 transpose` | 10.38 ns | 1.60 us | 1.58 us | 424.04 ns | 361.23 ns | 174.76 ns | 153.90x | 3.77x | 4.42x | 9.14x |
| `mat4 reciprocal` | 168.04 ns | 74.75 us | 73.17 us | 11.94 us | 8.84 us | 342.75 us | 444.83x | 6.26x | 8.46x | 0.22x |
| `mat4 reciprocal checked` | 178.50 ns | 74.75 us | 72.74 us | 11.99 us | 8.86 us | 343.42 us | 418.78x | 6.24x | 8.44x | 0.22x |
| `mat4 powi` | 243.02 ns | 104.29 us | 117.98 us | 15.76 us | 13.92 us | 351.21 us | 429.15x | 6.62x | 7.49x | 0.30x |
| `mat4 powi checked` | 244.14 ns | 105.21 us | 118.24 us | 15.74 us | 13.93 us | 352.76 us | 430.94x | 6.68x | 7.55x | 0.30x |
| `mat4 add` | 51.79 ns | 6.57 us | 6.79 us | 897.05 ns | 835.96 ns | 20.24 us | 126.82x | 7.32x | 7.86x | 0.32x |
| `mat4 add scalar` | 16.67 ns | 7.73 us | 8.28 us | 1.43 us | 1.18 us | 21.17 us | 463.75x | 5.41x | 6.57x | 0.37x |
| `mat4 sub` | 40.45 ns | 6.93 us | 6.99 us | 956.50 ns | 886.93 ns | 36.39 us | 171.41x | 7.25x | 7.82x | 0.19x |
| `mat4 sub scalar` | 15.42 ns | 8.95 us | 8.26 us | 1.44 us | 1.16 us | 37.77 us | 580.60x | 6.23x | 7.73x | 0.24x |
| `mat4 neg` | 14.17 ns | 1.87 us | 1.87 us | 923.13 ns | 745.93 ns | 14.29 us | 132.21x | 2.03x | 2.51x | 0.13x |
| `mat4 mul scalar` | 21.30 ns | 7.74 us | 8.08 us | 1.46 us | 1.12 us | 20.57 us | 363.42x | 5.31x | 6.90x | 0.38x |
| `mat4 div scalar` | 33.93 ns | 8.52 us | 7.79 us | 2.66 us | 1.37 us | 37.80 us | 251.05x | 3.21x | 6.20x | 0.23x |
| `mat4 div matrix` | 225.26 ns | 136.59 us | 106.68 us | 17.24 us | 13.99 us | 537.46 us | 606.36x | 7.92x | 9.76x | 0.25x |
| `mat4 bitxor` | 245.98 ns | 103.36 us | 117.78 us | 15.70 us | 13.97 us | 351.11 us | 420.21x | 6.58x | 7.40x | 0.29x |

### Borrowed API Operations

| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 8.95 ns | 374.84 ns | 377.22 ns | - | - | 1.34 us | 41.88x | - | - | 0.28x |
| `scalar add ref_owned` | 5.56 ns | 375.19 ns | 362.52 ns | - | - | 1.34 us | 67.49x | - | - | 0.28x |
| `scalar add refs` | 5.34 ns | 385.40 ns | 366.68 ns | - | - | 1.34 us | 72.15x | - | - | 0.29x |
| `scalar sub owned_ref` | 8.98 ns | 640.65 ns | 434.85 ns | - | - | 2.58 us | 71.32x | - | - | 0.25x |
| `scalar sub ref_owned` | 5.69 ns | 683.13 ns | 468.74 ns | - | - | 2.55 us | 120.10x | - | - | 0.27x |
| `scalar sub refs` | 5.58 ns | 661.97 ns | 423.66 ns | - | - | 2.56 us | 118.68x | - | - | 0.26x |
| `scalar mul owned_ref` | 9.73 ns | 423.60 ns | 320.34 ns | - | - | 1.57 us | 43.55x | - | - | 0.27x |
| `scalar mul ref_owned` | 7.76 ns | 417.01 ns | 304.60 ns | - | - | 1.56 us | 53.77x | - | - | 0.27x |
| `scalar mul refs` | 6.03 ns | 432.43 ns | 309.62 ns | - | - | 1.56 us | 71.72x | - | - | 0.28x |
| `scalar div owned_ref` | 15.39 ns | 356.72 ns | 218.41 ns | - | - | 2.60 us | 23.17x | - | - | 0.14x |
| `scalar div ref_owned` | 8.00 ns | 389.37 ns | 268.06 ns | - | - | 2.60 us | 48.70x | - | - | 0.15x |
| `scalar div refs` | 6.87 ns | 338.04 ns | 235.13 ns | - | - | 2.60 us | 49.19x | - | - | 0.13x |
| `vec3 add refs` | 6.03 ns | 1.72 us | 1.57 us | - | - | 4.07 us | 285.02x | - | - | 0.42x |
| `vec3 sub refs` | 6.06 ns | 1.80 us | 1.60 us | - | - | 7.99 us | 296.44x | - | - | 0.22x |
| `vec3 neg ref` | 3.28 ns | 282.23 ns | 281.13 ns | - | - | 3.16 us | 86.07x | - | - | 0.09x |
| `vec3 add_scalar_ref` | 6.50 ns | 1.80 us | 1.71 us | - | - | 3.88 us | 276.92x | - | - | 0.46x |
| `vec3 sub_scalar_ref` | 6.45 ns | 1.55 us | 1.48 us | - | - | 7.62 us | 240.69x | - | - | 0.20x |
| `vec3 mul_scalar_ref` | 6.87 ns | 1.94 us | 2.01 us | - | - | 4.50 us | 282.30x | - | - | 0.43x |
| `vec3 div_scalar_ref` | 9.48 ns | 1.72 us | 1.60 us | - | - | 7.87 us | 181.48x | - | - | 0.22x |
| `vec4 add refs` | 6.66 ns | 1.89 us | 1.53 us | - | - | 5.46 us | 283.20x | - | - | 0.35x |
| `vec4 sub refs` | 3.08 ns | 1.74 us | 1.44 us | - | - | 9.88 us | 566.44x | - | - | 0.18x |
| `vec4 neg ref` | 4.22 ns | 313.32 ns | 306.24 ns | - | - | 4.11 us | 74.28x | - | - | 0.08x |
| `vec4 add_scalar_ref` | 6.93 ns | 1.96 us | 1.66 us | - | - | 5.23 us | 282.09x | - | - | 0.37x |
| `vec4 sub_scalar_ref` | 4.22 ns | 1.70 us | 1.45 us | - | - | 9.69 us | 402.76x | - | - | 0.18x |
| `vec4 mul_scalar_ref` | 7.38 ns | 2.16 us | 2.06 us | - | - | 5.74 us | 293.07x | - | - | 0.38x |
| `vec4 div_scalar_ref` | 12.93 ns | 1.98 us | 1.73 us | - | - | 10.26 us | 153.30x | - | - | 0.19x |
| `mat3 add refs` | 11.11 ns | 3.68 us | 4.40 us | - | - | 11.93 us | 330.94x | - | - | 0.31x |
| `mat3 sub refs` | 10.67 ns | 3.91 us | 4.41 us | - | - | 21.44 us | 366.82x | - | - | 0.18x |
| `mat3 mul refs` | 52.76 ns | 23.83 us | 32.89 us | - | - | 62.22 us | 451.73x | - | - | 0.38x |
| `mat3 div refs` | 148.28 ns | 68.90 us | 65.62 us | - | - | 161.66 us | 464.67x | - | - | 0.43x |
| `mat3 neg ref` | 9.95 ns | 735.81 ns | 738.76 ns | - | - | 8.55 us | 73.98x | - | - | 0.09x |
| `mat3 add_scalar_ref` | 10.84 ns | 4.98 us | 5.80 us | - | - | 12.28 us | 459.82x | - | - | 0.41x |
| `mat3 sub_scalar_ref` | 11.98 ns | 5.65 us | 5.65 us | - | - | 21.67 us | 471.56x | - | - | 0.26x |
| `mat3 mul_scalar_ref` | 11.45 ns | 5.22 us | 5.85 us | - | - | 12.23 us | 456.05x | - | - | 0.43x |
| `mat3 div_scalar_ref` | 22.87 ns | 5.48 us | 5.46 us | - | - | 22.02 us | 239.44x | - | - | 0.25x |
| `mat4 add refs` | 17.60 ns | 3.91 us | 4.25 us | - | - | 19.98 us | 222.08x | - | - | 0.20x |
| `mat4 sub refs` | 21.29 ns | 4.38 us | 4.40 us | - | - | 36.36 us | 205.90x | - | - | 0.12x |
| `mat4 mul refs` | 110.57 ns | 32.83 us | 37.23 us | - | - | 144.41 us | 296.90x | - | - | 0.23x |
| `mat4 div refs` | 221.94 ns | 135.56 us | 106.53 us | - | - | 538.95 us | 610.81x | - | - | 0.25x |
| `mat4 neg ref` | 12.16 ns | 1.32 us | 1.32 us | - | - | 14.02 us | 108.75x | - | - | 0.09x |
| `mat4 add_scalar_ref` | 14.20 ns | 7.33 us | 8.00 us | - | - | 21.06 us | 516.43x | - | - | 0.35x |
| `mat4 sub_scalar_ref` | 21.77 ns | 8.63 us | 7.75 us | - | - | 38.67 us | 396.20x | - | - | 0.22x |
| `mat4 mul_scalar_ref` | 48.94 ns | 7.40 us | 7.76 us | - | - | 20.56 us | 151.29x | - | - | 0.36x |
| `mat4 div_scalar_ref` | 31.05 ns | 8.44 us | 7.69 us | - | - | 37.76 us | 271.92x | - | - | 0.22x |
| `mat3 transform_vec refs` | 15.57 ns | 10.47 us | 11.97 us | - | - | 20.40 us | 672.45x | - | - | 0.51x |
| `mat4 transform_vec refs` | 24.62 ns | 13.03 us | 11.92 us | - | - | 35.84 us | 529.15x | - | - | 0.36x |
| `complex add refs` | 7.66 ns | 756.36 ns | 786.36 ns | - | - | 2.62 us | 98.76x | - | - | 0.29x |
| `complex sub refs` | 7.91 ns | 881.94 ns | 765.47 ns | - | - | 4.87 us | 111.52x | - | - | 0.18x |
| `complex mul refs` | 7.99 ns | 3.18 us | 3.35 us | - | - | 10.22 us | 398.12x | - | - | 0.31x |
| `complex div refs` | 17.37 ns | 6.16 us | 6.53 us | - | - | 22.11 us | 354.96x | - | - | 0.28x |
| `complex neg ref` | 2.35 ns | 144.54 ns | 124.67 ns | - | - | 2.16 us | 61.57x | - | - | 0.07x |
| `complex div_real_ref` | 10.01 ns | 785.78 ns | 769.76 ns | - | - | 5.26 us | 78.53x | - | - | 0.15x |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.24 us |
| `astro sin 160` | 13.82 us |
| `astro sin 192` | 13.76 us |
| `astro sin 256` | 15.58 us |
