# Benchmarks

Run the Criterion benchmark suite:

```sh
cargo bench --bench mathbench
```

Refresh this file from existing Criterion estimates without rerunning the full suite:

```sh
cargo bench --bench mathbench -- --update-benchmarks-md
```

The `mathbench` suite benchmarks both crate backends and writes this file from Criterion's median estimates after a real benchmark run. The `astro-float`, `symbolica`, and `arpfloat` comparison columns run at 128-bit precision. Missing cells mean that the corresponding estimate was not present in `target/criterion` when this file was generated, or that the external library does not expose a directly comparable operation in this suite.

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

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | symbolica 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / symbolica | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `sin 0.1` | 10.74 ns | 203.39 ns | 202.75 ns | 10.65 us | 758.17 ns | 118.43 us | 18.94x | 0.02x | 0.27x | 0.00x |
| `cos 0.1` | 11.85 ns | 201.55 ns | 202.42 ns | 10.49 us | 488.71 ns | 24.04 us | 17.01x | 0.02x | 0.41x | 0.01x |
| `sin 1.23456789` | 11.84 ns | 1.93 us | 1.93 us | 12.40 us | 810.74 ns | 252.28 us | 162.77x | 0.16x | 2.38x | 0.01x |
| `cos 1.23456789` | 12.05 ns | 542.24 ns | 536.67 ns | 10.50 us | 583.60 ns | 163.83 us | 44.99x | 0.05x | 0.93x | 0.00x |
| `sin 1e6` | 12.75 ns | 11.34 us | 11.34 us | 15.79 us | 1.08 us | 254.76 us | 889.42x | 0.72x | 10.46x | 0.04x |
| `cos 1e6` | 12.39 ns | 11.48 us | 11.26 us | 13.93 us | 824.85 ns | 165.28 us | 926.93x | 0.82x | 13.92x | 0.07x |
| `sin 1e30` | 65.92 ns | 17.87 us | 17.90 us | 18.47 us | 2.89 us | 262.11 us | 271.14x | 0.97x | 6.18x | 0.07x |
| `cos 1e30` | 69.16 ns | 17.82 us | 17.71 us | 16.05 us | 945.99 ns | 163.53 us | 257.68x | 1.11x | 18.84x | 0.11x |
| `sin pi_7` | 11.76 ns | 202.15 ns | 2.39 us | 12.17 us | 723.05 ns | 118.03 us | 17.19x | 0.02x | 0.28x | 0.00x |
| `cos pi_7` | 11.64 ns | 201.92 ns | 5.47 us | 10.88 us | 524.27 ns | 25.99 us | 17.35x | 0.02x | 0.39x | 0.01x |
| `sin 1000pi_eps` | 11.76 ns | 11.14 us | 17.43 us | 16.04 us | 2.29 us | 252.86 us | 947.42x | 0.69x | 4.87x | 0.04x |
| `cos 1000pi_eps` | 12.44 ns | 11.13 us | 17.26 us | 13.97 us | 552.45 ns | 148.41 us | 894.70x | 0.80x | 20.14x | 0.07x |

#### Scalar API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | symbolica 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / symbolica | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.48 ns | 57.94 ns | 55.60 ns | 27.36 ns | 15.75 ns | 15.58 ns | 120.07x | 2.12x | 3.68x | 3.72x |
| `one` | 0.48 ns | 66.69 ns | 62.35 ns | 42.05 ns | 30.21 ns | 34.07 ns | 139.50x | 1.59x | 2.21x | 1.96x |
| `e` | 0.48 ns | 801.94 ns | 824.99 ns | 55.65 ns | 1.06 us | 51.34 us | 1687.71x | 14.41x | 0.75x | 0.02x |
| `pi` | 0.48 ns | 192.33 ns | 188.33 ns | 45.02 ns | 48.88 ns | 99.55 us | 403.31x | 4.27x | 3.93x | 0.00x |
| `tau` | 0.47 ns | 456.62 ns | 451.89 ns | 118.44 ns | 101.97 ns | 101.73 us | 962.18x | 3.86x | 4.48x | 0.00x |
| `add` | 5.22 ns | 425.87 ns | 417.92 ns | 52.93 ns | 42.35 ns | 96.31 ns | 81.66x | 8.05x | 10.06x | 4.42x |
| `sub` | 5.17 ns | 695.36 ns | 475.97 ns | 58.38 ns | 44.90 ns | 127.94 ns | 134.48x | 11.91x | 15.49x | 5.43x |
| `neg` | 5.08 ns | 63.72 ns | 62.16 ns | 23.37 ns | 20.24 ns | 17.48 ns | 12.54x | 2.73x | 3.15x | 3.65x |
| `mul` | 5.38 ns | 466.44 ns | 358.35 ns | 59.64 ns | 44.60 ns | 188.90 ns | 86.77x | 7.82x | 10.46x | 2.47x |
| `div` | 8.97 ns | 434.08 ns | 323.54 ns | 141.66 ns | 62.44 ns | 854.34 ns | 48.39x | 3.06x | 6.95x | 0.51x |
| `reciprocal` | 8.74 ns | 113.41 ns | 113.96 ns | 163.90 ns | 78.22 ns | 1.24 us | 12.97x | 0.69x | 1.45x | 0.09x |
| `reciprocal checked` | 9.11 ns | 116.96 ns | 115.83 ns | 165.60 ns | 77.70 ns | 1.25 us | 12.84x | 0.71x | 1.51x | 0.09x |
| `reciprocal checked abort` | 27.28 ns | 190.19 ns | 187.99 ns | 168.50 ns | 78.59 ns | 1.26 us | 6.97x | 1.13x | 2.42x | 0.15x |
| `pow` | 34.73 ns | 36.46 us | 32.61 us | 56.87 us | 2.90 us | 375.26 us | 1049.87x | 0.64x | 12.58x | 0.10x |
| `powi` | 5.84 ns | 2.55 us | 2.76 us | 283.63 ns | 84.02 ns | 1.33 us | 436.21x | 8.98x | 30.31x | 1.92x |
| `exp` | 10.56 ns | 7.55 us | 7.67 us | 16.04 us | 922.73 ns | 70.13 us | 714.64x | 0.47x | 8.18x | 0.11x |
| `ln` | 11.39 ns | 3.14 us | 2.84 us | 31.37 us | 1.35 us | 259.10 us | 275.52x | 0.10x | 2.33x | 0.01x |
| `log10` | 22.35 ns | 9.23 us | 7.05 us | 37.64 us | 2.79 us | 471.87 us | 412.93x | 0.25x | 3.31x | 0.02x |
| `log10 abort` | 18.08 ns | 9.21 us | 6.89 us | 37.65 us | 2.77 us | 474.07 us | 509.50x | 0.24x | 3.32x | 0.02x |
| `sqrt` | 8.79 ns | 1.87 us | 1.39 us | 5.00 us | 105.03 ns | 28.77 us | 212.35x | 0.37x | 17.77x | 0.06x |
| `sin` | 14.90 ns | 5.81 us | 6.08 us | 14.32 us | 1.26 us | 191.59 us | 389.83x | 0.41x | 4.63x | 0.03x |
| `cos` | 18.27 ns | 5.80 us | 5.97 us | 12.70 us | 627.88 ns | 94.49 us | 317.16x | 0.46x | 9.23x | 0.06x |
| `tan` | 26.37 ns | 5.81 us | 6.08 us | 29.87 us | 1.60 us | 226.60 us | 220.34x | 0.19x | 3.62x | 0.03x |
| `sinh` | 18.02 ns | 19.21 us | 19.31 us | 3.46 us | 1.13 us | 147.04 ns | 1065.88x | 5.55x | 17.01x | 130.62x |
| `cosh` | 17.95 ns | 19.16 us | 19.35 us | 8.11 us | 1.06 us | 137.21 ns | 1067.18x | 2.36x | 18.13x | 139.64x |
| `tanh` | 23.46 ns | 23.40 us | 23.38 us | 3.40 us | 1.21 us | 144.47 ns | 997.23x | 6.87x | 19.35x | 161.96x |
| `asin` | 9.30 ns | 1.28 us | 1.19 us | 22.48 us | 2.46 us | 135.39 ns | 137.76x | 0.06x | 0.52x | 9.46x |
| `asin abort` | 13.01 ns | 1.29 us | 1.19 us | 22.59 us | 2.46 us | 140.46 ns | 98.79x | 0.06x | 0.52x | 9.15x |
| `acos` | 9.94 ns | 1.26 us | 1.23 us | 27.67 us | 2.54 us | 137.77 ns | 127.01x | 0.05x | 0.50x | 9.17x |
| `acos abort` | 19.34 ns | 1.27 us | 1.23 us | 27.69 us | 2.54 us | 136.37 ns | 65.53x | 0.05x | 0.50x | 9.30x |
| `atan` | 16.40 ns | 1.22 us | 1.53 us | 19.45 us | 2.29 us | 141.70 ns | 74.59x | 0.06x | 0.53x | 8.63x |
| `atan abort` | 17.21 ns | 1.23 us | 1.55 us | 19.38 us | 2.31 us | 137.88 ns | 71.64x | 0.06x | 0.53x | 8.94x |
| `asinh` | 33.39 ns | 1.23 us | 1.55 us | 41.59 us | 1.71 us | 170.12 ns | 36.93x | 0.03x | 0.72x | 7.25x |
| `asinh abort` | 26.09 ns | 1.26 us | 1.56 us | 41.63 us | 1.72 us | 168.36 ns | 48.11x | 0.03x | 0.73x | 7.45x |
| `acosh` | 8.52 ns | 1.09 us | 1.40 us | 42.74 us | 3.47 us | 143.11 ns | 127.44x | 0.03x | 0.31x | 7.59x |
| `acosh abort` | 12.42 ns | 1.10 us | 1.43 us | 42.80 us | 3.43 us | 144.99 ns | 88.60x | 0.03x | 0.32x | 7.59x |
| `atanh` | 9.40 ns | 1.29 us | 1.25 us | 37.47 us | 1.33 us | 150.39 ns | 137.11x | 0.03x | 0.97x | 8.57x |
| `atanh abort` | 16.97 ns | 1.31 us | 1.27 us | 37.41 us | 1.34 us | 151.30 ns | 77.07x | 0.03x | 0.98x | 8.64x |
| `zero status` | 1.21 ns | 2.38 ns | 2.32 ns | 1.06 ns | 6.76 ns | 0.97 ns | 1.97x | 2.25x | 0.35x | 2.46x |
| `zero status abort` | 3.37 ns | 58.27 ns | 59.87 ns | 1.05 ns | 6.80 ns | 0.97 ns | 17.30x | 55.30x | 8.57x | 60.22x |

### Complex Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | symbolica 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / symbolica | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `zero` | 0.96 ns | 123.84 ns | 122.27 ns | 51.87 ns | - | 23.47 ns | 129.23x | 2.39x | - | 5.28x |
| `one` | 5.18 ns | 128.07 ns | 124.77 ns | 68.53 ns | - | 46.34 ns | 24.71x | 1.87x | - | 2.76x |
| `i` | 0.95 ns | 128.62 ns | 129.23 ns | 67.14 ns | - | 44.58 ns | 136.10x | 1.92x | - | 2.88x |
| `free i` | 0.95 ns | 128.85 ns | 130.78 ns | 67.46 ns | - | 44.74 ns | 136.21x | 1.91x | - | 2.88x |
| `conjugate` | 2.13 ns | 132.18 ns | 133.74 ns | 42.54 ns | - | 27.92 ns | 62.01x | 3.11x | - | 4.73x |
| `norm squared` | 5.96 ns | 1.62 us | 1.58 us | 155.14 ns | - | 511.42 ns | 271.04x | 10.41x | - | 3.16x |
| `reciprocal` | 18.56 ns | 3.12 us | 3.11 us | 451.11 ns | - | 3.16 us | 167.91x | 6.91x | - | 0.99x |
| `reciprocal checked` | 14.19 ns | 3.12 us | 3.18 us | 450.50 ns | - | 3.14 us | 220.09x | 6.93x | - | 0.99x |
| `powi` | 18.63 ns | 12.57 us | 15.63 us | 1.50 us | - | 4.43 us | 674.94x | 8.38x | - | 2.84x |
| `powi checked` | 18.56 ns | 12.62 us | 15.69 us | 1.48 us | - | 4.41 us | 679.94x | 8.52x | - | 2.86x |
| `div checked` | 19.47 ns | 6.43 us | 6.87 us | 801.30 ns | - | 4.12 us | 330.17x | 8.02x | - | 1.56x |
| `div real checked` | 9.43 ns | 792.15 ns | 754.03 ns | 269.26 ns | - | 1.77 us | 84.00x | 2.94x | - | 0.45x |
| `from scalar` | 1.43 ns | 133.60 ns | 133.59 ns | 46.91 ns | - | 25.72 ns | 93.65x | 2.85x | - | 5.20x |
| `add` | 6.17 ns | 887.31 ns | 920.52 ns | 110.27 ns | - | 209.32 ns | 143.84x | 8.05x | - | 4.24x |
| `sub` | 6.13 ns | 1.01 us | 897.80 ns | 119.14 ns | - | 245.36 ns | 165.04x | 8.49x | - | 4.12x |
| `neg` | 2.62 ns | 153.41 ns | 151.52 ns | 43.85 ns | - | 27.71 ns | 58.57x | 3.50x | - | 5.54x |
| `mul` | 11.54 ns | 3.32 us | 3.50 us | 309.21 ns | - | 997.64 ns | 288.12x | 10.75x | - | 3.33x |
| `div` | 18.73 ns | 6.50 us | 6.88 us | 800.26 ns | - | 4.15 us | 346.88x | 8.12x | - | 1.56x |
| `div real` | 10.37 ns | 778.32 ns | 740.45 ns | 272.70 ns | - | 1.74 us | 75.05x | 2.85x | - | 0.45x |

### Vector Operations

#### Vector Comparisons

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | symbolica 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / symbolica | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 dot` | 6.81 ns | 3.52 us | 6.26 us | 311.57 ns | 254.72 ns | 878.74 ns | 517.81x | 11.31x | 13.84x | 4.01x |
| `vec3 magnitude` | 10.00 ns | 7.52 us | 10.79 us | 5.59 us | 352.62 ns | 37.81 us | 751.55x | 1.35x | 21.32x | 0.20x |
| `vec3 normalize` | 25.42 ns | 12.60 us | 12.76 us | 6.09 us | 579.27 ns | 41.83 us | 495.95x | 2.07x | 21.76x | 0.30x |

#### Vector API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | symbolica 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / symbolica | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `vec3 new` | 3.11 ns | 1.15 us | 2.26 us | 69.58 ns | 57.97 ns | 201.46 ns | 370.61x | 16.56x | 19.88x | 5.72x |
| `vec3 zero` | 1.44 ns | 255.22 ns | 248.83 ns | 61.55 ns | 30.51 ns | 36.39 ns | 177.09x | 4.15x | 8.37x | 7.01x |
| `vec3 dot abort` | 28.16 ns | 4.22 us | 4.74 us | 255.07 ns | 205.74 ns | 830.12 ns | 149.70x | 16.53x | 20.49x | 5.08x |
| `vec3 magnitude abort` | 39.86 ns | 8.19 us | 7.50 us | 5.63 us | 325.71 ns | 37.68 us | 205.35x | 1.45x | 25.13x | 0.22x |
| `vec3 normalize checked` | 26.43 ns | 13.06 us | 10.03 us | 6.04 us | 538.32 ns | 42.06 us | 494.16x | 2.16x | 24.26x | 0.31x |
| `vec3 normalize checked abort` | 55.94 ns | 13.15 us | 10.48 us | 6.07 us | 539.68 ns | 41.90 us | 235.01x | 2.17x | 24.36x | 0.31x |
| `vec3 div scalar checked` | 15.73 ns | 1.74 us | 1.64 us | 405.94 ns | - | 3.01 us | 110.60x | 4.28x | - | 0.58x |
| `vec3 div scalar checked abort` | 17.90 ns | 1.77 us | 1.67 us | 407.06 ns | - | 2.99 us | 98.60x | 4.34x | - | 0.59x |
| `vec3 add` | 6.82 ns | 2.06 us | 1.89 us | 154.04 ns | 126.28 ns | 308.83 ns | 302.67x | 13.40x | 16.35x | 6.69x |
| `vec3 add scalar` | 6.57 ns | 1.89 us | 1.75 us | 153.71 ns | 133.39 ns | 361.43 ns | 288.58x | 12.33x | 14.21x | 5.24x |
| `vec3 sub` | 6.78 ns | 2.17 us | 1.97 us | 164.30 ns | 137.69 ns | 408.73 ns | 319.80x | 13.20x | 15.76x | 5.31x |
| `vec3 sub scalar` | 6.47 ns | 1.64 us | 1.59 us | 150.14 ns | 125.02 ns | 315.26 ns | 253.24x | 10.92x | 13.11x | 5.20x |
| `vec3 neg` | 3.79 ns | 237.92 ns | 231.75 ns | 57.63 ns | 50.44 ns | 45.06 ns | 62.74x | 4.13x | 4.72x | 5.28x |
| `vec3 mul scalar` | 6.98 ns | 2.06 us | 2.14 us | 164.48 ns | 125.68 ns | 585.89 ns | 294.96x | 12.52x | 16.38x | 3.51x |
| `vec3 div scalar` | 10.27 ns | 1.76 us | 1.66 us | 411.11 ns | 176.08 ns | 2.98 us | 171.15x | 4.28x | 9.99x | 0.59x |
| `vec4 dot` | 7.38 ns | 3.63 us | 3.34 us | 441.83 ns | 319.60 ns | 1.25 us | 491.66x | 8.21x | 11.35x | 2.91x |
| `vec4 magnitude` | 13.17 ns | 7.24 us | 5.84 us | 5.58 us | 414.09 ns | 38.31 us | 549.33x | 1.30x | 17.47x | 0.19x |
| `vec4 normalize` | 36.69 ns | 12.52 us | 8.06 us | 6.29 us | 719.51 ns | 44.13 us | 341.14x | 1.99x | 17.40x | 0.28x |
| `vec4 add` | 8.12 ns | 2.31 us | 1.99 us | 206.63 ns | 173.16 ns | 418.22 ns | 284.67x | 11.19x | 13.35x | 5.53x |
| `vec4 add scalar` | 6.97 ns | 2.14 us | 1.85 us | 215.83 ns | 177.39 ns | 488.60 ns | 307.17x | 9.92x | 12.07x | 4.38x |
| `vec4 sub` | 5.34 ns | 2.22 us | 1.88 us | 213.96 ns | 176.53 ns | 484.08 ns | 415.08x | 10.36x | 12.56x | 4.58x |
| `vec4 sub scalar` | 4.61 ns | 1.89 us | 1.65 us | 205.04 ns | 168.64 ns | 420.31 ns | 409.51x | 9.20x | 11.19x | 4.49x |
| `vec4 neg` | 4.99 ns | 319.89 ns | 314.76 ns | 80.95 ns | 66.69 ns | 50.06 ns | 64.17x | 3.95x | 4.80x | 6.39x |
| `vec4 mul scalar` | 7.41 ns | 2.34 us | 2.25 us | 224.80 ns | 162.22 ns | 749.95 ns | 315.56x | 10.41x | 14.42x | 3.12x |
| `vec4 div scalar` | 14.45 ns | 2.00 us | 1.72 us | 543.13 ns | 230.94 ns | 3.83 us | 138.36x | 3.68x | 8.66x | 0.52x |

### Matrix Operations

#### Matrix Comparisons

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | symbolica 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / symbolica | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 determinant` | 11.16 ns | 8.55 us | 3.40 us | 968.71 ns | 859.89 ns | 2.50 us | 765.68x | 8.82x | 9.94x | 3.41x |
| `mat3 inverse` | 100.36 ns | 32.00 us | 12.08 us | 3.26 us | 2.50 us | 10.48 us | 318.85x | 9.81x | 12.78x | 3.05x |
| `mat3 mul mat3` | 84.23 ns | 24.26 us | 10.99 us | 2.86 us | 2.36 us | 8.18 us | 288.04x | 8.49x | 10.29x | 2.97x |
| `mat3 transform vec3` | 15.95 ns | 10.61 us | 11.24 us | 1.02 us | 896.32 ns | 2.71 us | 665.48x | 10.37x | 11.84x | 3.92x |
| `mat4 determinant` | 46.31 ns | 16.37 us | 5.17 us | 4.65 us | 4.17 us | 10.72 us | 353.58x | 3.52x | 3.93x | 1.53x |
| `mat4 inverse` | 183.58 ns | 73.55 us | 19.08 us | 11.16 us | 9.38 us | 38.26 us | 400.65x | 6.59x | 7.84x | 1.92x |
| `mat4 mul mat4` | 118.83 ns | 31.22 us | 11.20 us | 6.38 us | 5.57 us | 19.15 us | 262.73x | 4.89x | 5.61x | 1.63x |
| `mat4 transform vec4` | 25.24 ns | 13.50 us | 3.66 us | 1.93 us | 1.71 us | 4.71 us | 534.89x | 6.99x | 7.89x | 2.86x |

#### Matrix API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | symbolica 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / symbolica | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `mat3 new` | 35.01 ns | 2.94 us | 6.02 us | 208.59 ns | 229.05 ns | 589.46 ns | 83.97x | 14.09x | 12.84x | 4.99x |
| `mat3 zero` | 16.38 ns | 881.56 ns | 896.58 ns | 263.44 ns | 208.49 ns | 189.93 ns | 53.81x | 3.35x | 4.23x | 4.64x |
| `mat3 identity` | 9.89 ns | 932.15 ns | 943.46 ns | 332.57 ns | 242.14 ns | 269.27 ns | 94.27x | 2.80x | 3.85x | 3.46x |
| `mat3 transpose` | 9.11 ns | 903.33 ns | 906.56 ns | 232.02 ns | 210.93 ns | 197.86 ns | 99.16x | 3.89x | 4.28x | 4.57x |
| `mat3 reciprocal` | 89.71 ns | 32.97 us | 38.97 us | 3.00 us | 2.31 us | 12.44 us | 367.52x | 11.00x | 14.30x | 2.65x |
| `mat3 reciprocal checked` | 111.39 ns | 32.85 us | 38.86 us | 3.01 us | 2.31 us | 10.02 us | 294.92x | 10.91x | 14.20x | 3.28x |
| `mat3 inverse checked` | 111.38 ns | 32.99 us | 39.19 us | 3.02 us | 2.32 us | 10.08 us | 296.22x | 10.91x | 14.21x | 3.27x |
| `mat3 inverse checked abort` | 117.65 ns | 32.97 us | 39.34 us | 2.99 us | 2.32 us | 10.15 us | 280.20x | 11.03x | 14.20x | 3.25x |
| `mat3 powi` | 146.99 ns | 70.12 us | 96.43 us | 6.82 us | 6.16 us | 19.16 us | 477.03x | 10.28x | 11.38x | 3.66x |
| `mat3 powi checked` | 145.31 ns | 70.49 us | 96.63 us | 6.80 us | 6.20 us | 19.00 us | 485.11x | 10.37x | 11.37x | 3.71x |
| `mat3 powi checked abort` | 146.28 ns | 70.57 us | 96.93 us | 6.74 us | 6.15 us | 19.11 us | 482.44x | 10.47x | 11.48x | 3.69x |
| `mat3 div scalar checked` | 25.14 ns | 5.63 us | 5.64 us | 1.52 us | 810.84 ns | 11.46 us | 224.13x | 3.72x | 6.95x | 0.49x |
| `mat3 div scalar checked abort` | 29.63 ns | 5.66 us | 5.67 us | 1.49 us | 810.51 ns | 11.41 us | 191.10x | 3.79x | 6.99x | 0.50x |
| `mat3 div matrix checked` | 147.49 ns | 72.02 us | 67.88 us | 5.55 us | 4.48 us | 17.86 us | 488.34x | 12.99x | 16.09x | 4.03x |
| `mat3 div matrix checked abort` | 163.12 ns | 72.15 us | 68.72 us | 5.54 us | 4.46 us | 17.80 us | 442.31x | 13.02x | 16.19x | 4.05x |
| `mat3 add` | 14.52 ns | 5.44 us | 6.34 us | 530.85 ns | 486.97 ns | 1.10 us | 375.03x | 10.26x | 11.18x | 4.96x |
| `mat3 add scalar` | 10.48 ns | 5.23 us | 6.00 us | 781.72 ns | 710.77 ns | 1.43 us | 498.87x | 6.69x | 7.36x | 3.65x |
| `mat3 sub` | 12.99 ns | 5.59 us | 6.10 us | 541.09 ns | 520.23 ns | 1.23 us | 430.45x | 10.34x | 10.75x | 4.54x |
| `mat3 sub scalar` | 10.72 ns | 5.88 us | 5.86 us | 792.38 ns | 701.60 ns | 1.40 us | 548.75x | 7.43x | 8.39x | 4.20x |
| `mat3 neg` | 10.85 ns | 1.04 us | 1.05 us | 480.67 ns | 453.97 ns | 447.69 ns | 96.16x | 2.17x | 2.30x | 2.33x |
| `mat3 mul scalar` | 12.26 ns | 5.54 us | 6.17 us | 786.93 ns | 672.10 ns | 1.98 us | 451.90x | 7.04x | 8.24x | 2.80x |
| `mat3 div scalar` | 24.41 ns | 5.61 us | 5.55 us | 1.49 us | 814.47 ns | 11.52 us | 229.90x | 3.77x | 6.89x | 0.49x |
| `mat3 div matrix` | 146.58 ns | 70.91 us | 69.48 us | 5.58 us | 4.47 us | 17.75 us | 483.72x | 12.70x | 15.87x | 3.99x |
| `mat3 bitxor` | 145.40 ns | 70.09 us | 96.89 us | 6.83 us | 6.16 us | 19.13 us | 482.07x | 10.26x | 11.37x | 3.66x |
| `mat4 zero` | 11.56 ns | 1.54 us | 1.54 us | 555.47 ns | 348.42 ns | 312.24 ns | 133.09x | 2.77x | 4.42x | 4.93x |
| `mat4 identity` | 11.10 ns | 1.69 us | 1.68 us | 606.13 ns | 420.55 ns | 412.22 ns | 151.81x | 2.78x | 4.01x | 4.09x |
| `mat4 transpose` | 9.90 ns | 1.65 us | 1.61 us | 487.16 ns | 372.29 ns | 289.65 ns | 166.39x | 3.38x | 4.42x | 5.68x |
| `mat4 reciprocal` | 187.80 ns | 75.06 us | 74.52 us | 10.83 us | 8.90 us | 37.42 us | 399.68x | 6.93x | 8.43x | 2.01x |
| `mat4 reciprocal checked` | 177.08 ns | 75.71 us | 76.20 us | 10.84 us | 8.88 us | 37.71 us | 427.53x | 6.99x | 8.53x | 2.01x |
| `mat4 powi` | 241.66 ns | 106.03 us | 122.84 us | 15.59 us | 14.29 us | 43.17 us | 438.75x | 6.80x | 7.42x | 2.46x |
| `mat4 powi checked` | 242.12 ns | 106.31 us | 123.46 us | 15.87 us | 14.27 us | 43.43 us | 439.09x | 6.70x | 7.45x | 2.45x |
| `mat4 add` | 51.77 ns | 6.55 us | 6.93 us | 914.89 ns | 872.97 ns | 1.72 us | 126.46x | 7.16x | 7.50x | 3.81x |
| `mat4 add scalar` | 16.46 ns | 7.71 us | 8.57 us | 1.40 us | 1.18 us | 2.37 us | 468.22x | 5.49x | 6.51x | 3.26x |
| `mat4 sub` | 39.39 ns | 6.94 us | 7.01 us | 976.74 ns | 920.86 ns | 2.09 us | 176.09x | 7.10x | 7.53x | 3.32x |
| `mat4 sub scalar` | 15.22 ns | 8.91 us | 8.25 us | 1.43 us | 1.18 us | 2.25 us | 585.82x | 6.22x | 7.57x | 3.96x |
| `mat4 neg` | 13.66 ns | 1.84 us | 1.86 us | 909.05 ns | 758.61 ns | 703.40 ns | 135.09x | 2.03x | 2.43x | 2.62x |
| `mat4 mul scalar` | 21.56 ns | 7.88 us | 8.19 us | 1.42 us | 1.16 us | 3.46 us | 365.61x | 5.54x | 6.81x | 2.28x |
| `mat4 div scalar` | 33.48 ns | 8.84 us | 8.02 us | 2.64 us | 1.39 us | 19.63 us | 263.96x | 3.34x | 6.36x | 0.45x |
| `mat4 div matrix` | 223.06 ns | 142.61 us | 111.39 us | 17.01 us | 14.15 us | 55.51 us | 639.32x | 8.38x | 10.08x | 2.57x |
| `mat4 bitxor` | 243.03 ns | 105.92 us | 122.52 us | 15.75 us | 14.27 us | 43.42 us | 435.83x | 6.73x | 7.42x | 2.44x |

### Borrowed API Operations

| Benchmark | Approx | Realistic from f64 | Realistic rational | astro-float 128 | symbolica 128 | arpfloat 128 | Realistic f64 / approx | Realistic f64 / astro | Realistic f64 / symbolica | Realistic f64 / arp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `scalar add owned_ref` | 9.07 ns | 379.75 ns | 362.25 ns | - | - | - | 41.88x | - | - | - |
| `scalar add ref_owned` | 5.53 ns | 376.99 ns | 362.39 ns | - | - | - | 68.14x | - | - | - |
| `scalar add refs` | 5.39 ns | 370.69 ns | 353.98 ns | - | - | - | 68.77x | - | - | - |
| `scalar sub owned_ref` | 9.17 ns | 641.60 ns | 415.73 ns | - | - | - | 69.98x | - | - | - |
| `scalar sub ref_owned` | 5.73 ns | 693.56 ns | 470.82 ns | - | - | - | 120.96x | - | - | - |
| `scalar sub refs` | 5.69 ns | 633.04 ns | 406.51 ns | - | - | - | 111.27x | - | - | - |
| `scalar mul owned_ref` | 9.85 ns | 426.15 ns | 305.45 ns | - | - | - | 43.26x | - | - | - |
| `scalar mul ref_owned` | 6.27 ns | 426.79 ns | 307.13 ns | - | - | - | 68.04x | - | - | - |
| `scalar mul refs` | 6.38 ns | 419.58 ns | 300.05 ns | - | - | - | 65.80x | - | - | - |
| `scalar div owned_ref` | 15.60 ns | 338.29 ns | 210.68 ns | - | - | - | 21.68x | - | - | - |
| `scalar div ref_owned` | 15.90 ns | 401.03 ns | 272.35 ns | - | - | - | 25.22x | - | - | - |
| `scalar div refs` | 6.90 ns | 338.39 ns | 206.99 ns | - | - | - | 49.08x | - | - | - |
| `vec3 add refs` | 6.31 ns | 1.74 us | 1.58 us | - | - | - | 275.42x | - | - | - |
| `vec3 sub refs` | 6.37 ns | 1.86 us | 1.67 us | - | - | - | 292.61x | - | - | - |
| `vec3 neg ref` | 3.32 ns | 251.37 ns | 245.37 ns | - | - | - | 75.62x | - | - | - |
| `vec3 add_scalar_ref` | 6.50 ns | 1.84 us | 1.73 us | - | - | - | 282.91x | - | - | - |
| `vec3 sub_scalar_ref` | 6.57 ns | 1.60 us | 1.56 us | - | - | - | 244.01x | - | - | - |
| `vec3 mul_scalar_ref` | 6.97 ns | 2.00 us | 2.08 us | - | - | - | 287.42x | - | - | - |
| `vec3 div_scalar_ref` | 9.82 ns | 1.78 us | 1.68 us | - | - | - | 181.58x | - | - | - |
| `vec4 add refs` | 6.39 ns | 1.89 us | 1.57 us | - | - | - | 296.35x | - | - | - |
| `vec4 sub refs` | 3.09 ns | 1.77 us | 1.45 us | - | - | - | 571.15x | - | - | - |
| `vec4 neg ref` | 4.24 ns | 378.41 ns | 368.38 ns | - | - | - | 89.16x | - | - | - |
| `vec4 add_scalar_ref` | 7.03 ns | 2.03 us | 1.76 us | - | - | - | 289.35x | - | - | - |
| `vec4 sub_scalar_ref` | 4.28 ns | 1.75 us | 1.50 us | - | - | - | 409.19x | - | - | - |
| `vec4 mul_scalar_ref` | 7.53 ns | 2.21 us | 2.16 us | - | - | - | 293.54x | - | - | - |
| `vec4 div_scalar_ref` | 13.76 ns | 2.05 us | 1.69 us | - | - | - | 148.92x | - | - | - |
| `mat3 add refs` | 11.39 ns | 3.67 us | 4.48 us | - | - | - | 321.80x | - | - | - |
| `mat3 sub refs` | 10.67 ns | 3.87 us | 4.42 us | - | - | - | 362.71x | - | - | - |
| `mat3 mul refs` | 54.76 ns | 23.85 us | 32.77 us | - | - | - | 435.62x | - | - | - |
| `mat3 div refs` | 146.66 ns | 71.31 us | 67.04 us | - | - | - | 486.26x | - | - | - |
| `mat3 neg ref` | 9.83 ns | 742.07 ns | 759.61 ns | - | - | - | 75.46x | - | - | - |
| `mat3 add_scalar_ref` | 10.33 ns | 5.01 us | 6.78 us | - | - | - | 484.72x | - | - | - |
| `mat3 sub_scalar_ref` | 12.76 ns | 5.66 us | 6.56 us | - | - | - | 443.47x | - | - | - |
| `mat3 mul_scalar_ref` | 11.63 ns | 5.25 us | 5.92 us | - | - | - | 451.54x | - | - | - |
| `mat3 div_scalar_ref` | 23.92 ns | 5.65 us | 5.63 us | - | - | - | 236.11x | - | - | - |
| `mat4 add refs` | 16.93 ns | 3.81 us | 4.35 us | - | - | - | 224.83x | - | - | - |
| `mat4 sub refs` | 16.52 ns | 4.26 us | 4.50 us | - | - | - | 257.76x | - | - | - |
| `mat4 mul refs` | 103.43 ns | 33.50 us | 39.31 us | - | - | - | 323.90x | - | - | - |
| `mat4 div refs` | 212.53 ns | 142.45 us | 112.12 us | - | - | - | 670.26x | - | - | - |
| `mat4 neg ref` | 12.54 ns | 1.27 us | 1.24 us | - | - | - | 101.09x | - | - | - |
| `mat4 add_scalar_ref` | 14.16 ns | 7.52 us | 8.22 us | - | - | - | 531.06x | - | - | - |
| `mat4 sub_scalar_ref` | 15.37 ns | 8.88 us | 8.00 us | - | - | - | 577.61x | - | - | - |
| `mat4 mul_scalar_ref` | 48.93 ns | 7.66 us | 7.95 us | - | - | - | 156.54x | - | - | - |
| `mat4 div_scalar_ref` | 29.04 ns | 8.69 us | 7.87 us | - | - | - | 299.34x | - | - | - |
| `mat3 transform_vec refs` | 15.70 ns | 10.64 us | 12.25 us | - | - | - | 678.07x | - | - | - |
| `mat4 transform_vec refs` | 24.68 ns | 13.17 us | 12.07 us | - | - | - | 533.82x | - | - | - |
| `complex add refs` | 7.76 ns | 766.13 ns | 797.12 ns | - | - | - | 98.78x | - | - | - |
| `complex sub refs` | 8.05 ns | 892.06 ns | 785.44 ns | - | - | - | 110.77x | - | - | - |
| `complex mul refs` | 8.05 ns | 3.15 us | 3.36 us | - | - | - | 391.33x | - | - | - |
| `complex div refs` | 17.58 ns | 6.23 us | 6.73 us | - | - | - | 354.32x | - | - | - |
| `complex neg ref` | 2.38 ns | 132.72 ns | 131.46 ns | - | - | - | 55.79x | - | - | - |
| `complex div_real_ref` | 10.09 ns | 775.23 ns | 752.77 ns | - | - | - | 76.81x | - | - | - |

### Precision Scaling

| Benchmark | Median |
| --- | ---: |
| `astro sin 128` | 12.01 us |
| `astro sin 160` | 13.14 us |
| `astro sin 192` | 13.23 us |
| `astro sin 256` | 15.78 us |
| `arp sin 128` | 253.69 us |
| `arp sin 160` | 320.21 us |
| `arp sin 192` | 377.19 us |
| `arp sin 256` | 574.62 us |
